"""MuJoCo hardware adapter for the vehicle firmware.

This module contains sensor/frame conversion and actuator/plant physics only.
The imported C tasks own control laws, observers, gates, modes and parameters.
"""
from __future__ import annotations

import json
from pathlib import Path
import mujoco
import numpy as np

from ..paths import REPO_ROOT, CALIBRATED_STAND_POSE_PATH, ENCODER_ZERO_POSE_PATH
from ..plant import load_model
from ..sysid.apply import load_profile
from .runtime import VehicleRuntime, Feedback, Command, Output
from .calibration import load_calibration
from .build import FIRMWARE
from .schedule import DISPATCH_ORDER

HIP_NAMES = ("l_j1", "r_j1", "r_j2", "l_j2")  # DM IDs 1,2,3,4: LB,RB,RF,LF
WHEEL_NAMES = ("l_wheel", "r_wheel")


def unit(v):
    v = np.asarray(v, dtype=float)
    length = float(np.linalg.norm(v))
    if length < 1e-9:
        raise ValueError("degenerate CAD sensor axis")
    return v / length


def place_pose(model, data, path: Path, wheel_radius: float, z_offset=0.0):
    """Set an initial condition only. No pose writes occur during stepping."""
    pose = json.loads(path.read_text())
    mujoco.mj_resetData(model, data)
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    address = model.jnt_qposadr[base]
    data.qpos[address:address+7] = pose["base_free"]
    for i in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        if name in pose and name != "base_free":
            data.qpos[model.jnt_qposadr[i]] = pose[name]
    mujoco.mj_forward(model, data)
    heights = [data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name), 2] - wheel_radius
               for name in WHEEL_NAMES]
    data.qpos[address+2] += z_offset - min(heights)
    mujoco.mj_forward(model, data)


class VehicleSimulation:
    def __init__(self, variant="fast", *, plant="identified_2026-09-21", pose="encoder", z_offset=None,
                 interface_mapping="original", yaw_damping_nms_rad=None, controller_profile=None):
        if interface_mapping not in ("original", "coherent_candidate"):
            raise ValueError("unknown hardware interface mapping")
        self.interface_mapping = interface_mapping
        self.controller_profile = controller_profile
        self.variant, self.plant = variant, plant
        self.model, self.plant_applied = load_model(variant, plant=plant)
        self.data = mujoco.MjData(self.model)
        self.runtime = VehicleRuntime(profile=controller_profile)
        self.tick_s = self.runtime.timing["control_tick_us"] * 1e-6
        self.model.opt.timestep = self.tick_s
        self.base = self._id(mujoco.mjtObj.mjOBJ_BODY, "base_link")
        self.gimbal = self._id(mujoco.mjtObj.mjOBJ_BODY, "gimbal_yaw")
        names = HIP_NAMES + WHEEL_NAMES + ("gimbal_yaw",)
        self.joints = {n: self._id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in names}
        self.qadr = {n: int(self.model.jnt_qposadr[i]) for n, i in self.joints.items()}
        self.dadr = {n: int(self.model.jnt_dofadr[i]) for n, i in self.joints.items()}
        self.yaw_damping_override = yaw_damping_nms_rad
        if yaw_damping_nms_rad is not None:
            if not np.isfinite(yaw_damping_nms_rad) or yaw_damping_nms_rad < 0:
                raise ValueError("yaw damping must be finite and nonnegative")
            self.model.dof_damping[self.dadr["gimbal_yaw"]] = yaw_damping_nms_rad
        self.actuators = {n: self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, n + "_motor") for n in names}
        self.zero, self.calibration = load_calibration()
        self.zero["gimbal_yaw"] = json.loads(ENCODER_ZERO_POSE_PATH.read_text())["gimbal_yaw"]
        self.gas = []
        parameters = self.runtime.parameters
        for side, key in (("l", "gas_force_left_n"), ("r", "gas_force_right_n")):
            self.gas.append((self._id(mujoco.mjtObj.mjOBJ_SITE, side + "_gas_thigh"),
                             self._id(mujoco.mjtObj.mjOBJ_SITE, side + "_gas_leg"), parameters[key]))
        if plant != "nominal":
            profile = load_profile(plant)
            force = profile.number("gas_springs", "force_n") * profile.number("gas_springs", "scale")
            self.gas = [(a, b, force) for a, b, _ in self.gas]
        # Firmware defines the command envelope. Identified motor gains can
        # exceed unity, so hips also need a physical output cap after that gain.
        # Retain the identified gain/friction/inertia and the yaw limit policy.
        for name in HIP_NAMES + ("gimbal_yaw",):
            aid = self.actuators[name]
            self.model.actuator_ctrlrange[aid] = (parameters["dm_torque_min"], parameters["dm_torque_max"])
            if name in HIP_NAMES:
                self.model.actuator_forcelimited[aid] = True
            if self.model.actuator_forcelimited[aid]:
                self.model.actuator_forcerange[aid] = self.model.actuator_ctrlrange[aid]
        reference = mujoco.MjData(self.model)
        place_pose(self.model, reference, CALIBRATED_STAND_POSE_PATH, parameters["wheel_radius_m"])
        rotation = reference.xmat[self.base].reshape(3, 3)
        mount = rotation.T @ (reference.xpos[self.gimbal] - reference.xpos[self.base])
        mount[1] = 0.0  # CAD Y is chassis-up
        forward = unit(mount)
        up = unit(rotation.T @ np.array([0., 0., 1.]))
        lateral = unit(np.cross(forward, up))
        forward = unit(np.cross(up, lateral))
        # Raw IMU XYZ = lateral, forward, up. These are right-handed.
        self.sensor_body = np.column_stack((lateral, forward, up))
        self.gimbal_forward = reference.xmat[self.gimbal].reshape(3, 3).T @ (rotation @ forward)
        self.reference_heading = np.arctan2((rotation @ forward)[1], (rotation @ forward)[0])
        self.pose = pose
        self.z_offset = (0.597 if variant == "platform" else 0.0) if z_offset is None else z_offset
        self.spawn_xy = (0.0, 0.0)
        if variant == "rmuc2025":
            from ..plant.rmuc import SPAWN_XY, SPAWN_SURFACE_Z
            self.spawn_xy = SPAWN_XY
            if z_offset is None:
                self.z_offset = SPAWN_SURFACE_Z
        elif variant in ("rmuc2026", "rmuc2027"):
            # The package's passive assembly example is beside a facility
            # wall. The operator starts on independently sampled open floor.
            spawn = json.loads((REPO_ROOT / ("terrain_" + variant) / "operator_spawn.json").read_text())
            self.spawn_xy = tuple(spawn["xy_m"])
            if z_offset is None:
                self.z_offset = spawn["collision_height_m"]
        self.reset(pose)

    def _id(self, kind, name):
        value = mujoco.mj_name2id(self.model, kind, name)
        if value < 0:
            raise ValueError(f"model is missing {name}")
        return value

    def reset(self, pose=None):
        if getattr(self, "ticks", 0):
            self.runtime.close()
            self.runtime = VehicleRuntime(profile=self.controller_profile)
        if pose is not None:
            self.pose = pose
        paths = {"stand": CALIBRATED_STAND_POSE_PATH,
                 "encoder": FIRMWARE / "calibration/evidence/encoder_zero_candidate_pose.json"}
        if self.pose not in paths:
            raise ValueError("pose must be stand or encoder")
        place_pose(self.model, self.data, paths[self.pose], self.runtime.parameters["wheel_radius_m"], self.z_offset)
        if self.spawn_xy != (0.0, 0.0):
            address = self.model.jnt_qposadr[self._id(mujoco.mjtObj.mjOBJ_JOINT, "base_free")]
            self.data.qpos[address:address+2] += self.spawn_xy
            mujoco.mj_forward(self.model, self.data)
        self.ticks = 0
        self.output = Output()
        self.feedback = Feedback()
        self.feedback_mask = 0x1FF
        self._rx_tx_counts = np.zeros(5, dtype=np.uint32)
        self._previous_velocity = self._velocity(self.base)[3:]
        self._heading_last = None
        self._yaw_total = 0.0

    def _velocity(self, body):
        result = np.empty(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY, body, result, 0)
        return result

    def read_feedback(self) -> Feedback:
        data = self.data
        # DM responses follow each addressed command, rather than refreshing
        # all five encoders every control tick. One host tick models the reply
        # latency; the initial stationary read primes every motor once.
        arrivals = 0x170  # wheel RX + BMI088 + availability of requested yaw IMU
        counts = np.asarray(self.output.motor_tx_count, dtype=np.uint32)
        for i, bit in enumerate((1,2,4,8,128)):
            if not self.ticks or counts[i] != self._rx_tx_counts[i]:
                arrivals |= bit
        self._rx_tx_counts = counts.copy()
        f = Feedback(time_us=(self.ticks + 1) * self.runtime.timing["control_tick_us"],
                     feedback_mask=self.feedback_mask & arrivals, raw_imu=1)
        for i, name in enumerate(HIP_NAMES):
            f.hip_angle[i] = data.qpos[self.qadr[name]] - self.zero[name]
            f.hip_speed[i] = data.qvel[self.dadr[name]]
            f.hip_torque[i] = data.actuator_force[self.actuators[name]]
        for i, name in enumerate(WHEEL_NAMES):
            f.wheel_angle[i] = data.qpos[self.qadr[name]]
            f.wheel_speed[i] = data.qvel[self.dadr[name]]
            f.wheel_torque[i] = data.actuator_force[self.actuators[name]]
        f.yaw_motor_angle = data.qpos[self.qadr["gimbal_yaw"]] - self.zero["gimbal_yaw"]
        f.yaw_motor_speed = data.qvel[self.dadr["gimbal_yaw"]]
        f.yaw_motor_torque = data.actuator_force[self.actuators["gimbal_yaw"]]
        sensor_world = data.xmat[self.base].reshape(3, 3) @ self.sensor_body
        velocity = self._velocity(self.base)
        acceleration = (velocity[3:] - self._previous_velocity) / self.tick_s
        self._previous_velocity = velocity[3:].copy()
        f.gyro[:] = sensor_world.T @ velocity[:3]
        # Ideal corrected BMI088 specific force at the base COM. Original C
        # INS removes gravity, filters acceleration and estimates orientation.
        f.accel[:] = sensor_world.T @ (acceleration - self.model.opt.gravity)
        gimbal_rotation = data.xmat[self.gimbal].reshape(3, 3)
        gimbal_forward = gimbal_rotation @ self.gimbal_forward
        heading = float(np.arctan2(gimbal_forward[1], gimbal_forward[0]))
        if self._heading_last is None:
            self._yaw_total = float(np.arctan2(np.sin(heading-self.reference_heading), np.cos(heading-self.reference_heading)))
        else:
            self._yaw_total += float(np.arctan2(np.sin(heading-self._heading_last), np.cos(heading-self._heading_last)))
        self._heading_last = heading
        f.gimbal_yaw_deg = -np.degrees(self._yaw_total)
        f.gimbal_yaw_rate = -self._velocity(self.gimbal)[2]
        if self.interface_mapping == "coherent_candidate":
            # Explicit audit candidate: wheel motor axes oppose CAD axes;
            # yaw motor/output shafts are related by the real 1:1 reverse gear.
            # All conjugate position/rate/torque quantities change together.
            for field in (f.wheel_angle, f.wheel_speed, f.wheel_torque):
                for i in range(2):
                    field[i] *= -1
            f.yaw_motor_angle *= -1
            f.yaw_motor_speed *= -1
            f.yaw_motor_torque *= -1
            f.gimbal_yaw_deg *= -1
            f.gimbal_yaw_rate *= -1
        return f

    def _apply_mit(self, name, packet, angle, speed):
        p, v, kp, kd, torque = packet
        command = kp * (p - angle) + kd * (v - speed) + torque
        params = self.runtime.parameters
        self.data.ctrl[self.actuators[name]] = np.clip(command, params["dm_torque_min"], params["dm_torque_max"])

    def apply_outputs(self, output: Output, feedback: Feedback):
        self.data.ctrl[:] = 0.0
        self.data.qfrc_applied[:] = 0.0
        for i, name in enumerate(HIP_NAMES):
            self._apply_mit(name, output.hip_command[i], feedback.hip_angle[i], feedback.hip_speed[i])
        self._apply_mit("gimbal_yaw", output.yaw_command, feedback.yaw_motor_angle, feedback.yaw_motor_speed)
        for i, name in enumerate(WHEEL_NAMES):
            self.data.ctrl[self.actuators[name]] = output.wheel_current[i] / self.runtime.parameters["wheel_current_per_nm"]
        if self.interface_mapping == "coherent_candidate":
            for name in WHEEL_NAMES + ("gimbal_yaw",):
                self.data.ctrl[self.actuators[name]] *= -1
        # Gas springs are passive plant forces and remain active in RELAX.
        for thigh, leg, force in self.gas:
            a, b = self.data.site_xpos[thigh], self.data.site_xpos[leg]
            delta = b-a
            length = float(np.linalg.norm(delta))
            if length <= 1e-9:
                continue
            direction = delta / length
            mujoco.mj_applyFT(self.model, self.data, -force*direction, np.zeros(3), a,
                             int(self.model.site_bodyid[thigh]), self.data.qfrc_applied)
            mujoco.mj_applyFT(self.model, self.data, force*direction, np.zeros(3), b,
                             int(self.model.site_bodyid[leg]), self.data.qfrc_applied)

    def step(self, command: Command) -> Output:
        self.feedback = self.read_feedback()
        self.output = self.runtime.step(self.feedback, command)
        self.apply_outputs(self.output, self.feedback)
        previous_time = float(self.data.time)
        bad_warnings = (mujoco.mjtWarning.mjWARN_BADQPOS, mujoco.mjtWarning.mjWARN_BADQVEL,
                        mujoco.mjtWarning.mjWARN_BADQACC)
        previous_warnings = [int(self.data.warning[w].number) for w in bad_warnings]
        mujoco.mj_step(self.model, self.data)
        self.ticks += 1
        if any(int(self.data.warning[w].number) > count for w, count in zip(bad_warnings, previous_warnings)):
            raise RuntimeError(f"MuJoCo reported an invalid physical state at tick {self.ticks}")
        if not np.isclose(self.data.time, previous_time + self.tick_s, atol=1e-10, rtol=0):
            raise RuntimeError(f"MuJoCo restarted the plant at tick {self.ticks}; control history is invalid")
        if not np.isfinite(self.data.qpos).all() or not np.isfinite(self.data.qvel).all():
            raise RuntimeError(f"plant became non-finite at tick {self.ticks}")
        return self.output

    def metadata(self):
        return {"controller": "vehicle_c", "firmware": self.runtime.manifest["fingerprint"],
                "controller_profile": self.runtime.profile_name,
                "simulation_profile": self.runtime.controller_profile,
                "native_build": self.runtime.native_fingerprint,
                "source_revision": self.runtime.manifest["revision"], "timing": self.runtime.timing,
                "dispatch_order": DISPATCH_ORDER,
                "dispatch_phase": "nominal task creation order; TIM4 after tasks; DM reply next tick",
                "dm_poll_policy": "driving: one motor per TIM4 tick; jump/landing: all five per TIM4 tick",
                "variant": self.variant, "plant": self.plant, "plant_applied": self.plant_applied,
                "plant_mass_kg": float(self.model.body_mass.sum()), "parameters": self.runtime.parameters,
                "pose": self.pose, "z_offset_m": self.z_offset, "spawn_xy_m": list(self.spawn_xy),
                "encoder_zero": {n: self.zero[n] for n in HIP_NAMES}, "calibration": self.calibration,
                "sensor_body": self.sensor_body.tolist(), "sensor": "ideal corrected BMI088 at base COM",
                "gimbal": "yaw only; pitch feedback at centre",
                "yaw_damping_nms_rad": float(self.model.dof_damping[self.dadr["gimbal_yaw"]]),
                "yaw_damping_status": "explicit uncalibrated bearing assumption" if
                    self.yaw_damping_override is not None else "generic MJCF default; not identified by leg profile",
                "interface_mapping": self.interface_mapping,
                "interface_mapping_status": "unverified vehicle installation candidate" if
                    self.interface_mapping == "coherent_candidate" else "original adapter"}

    def close(self):
        self.runtime.close()
