"""Runtime-only MuJoCo probe for jump34 against firmware-v8 landing inputs.

No production/controller/model/XML files are modified.  Run from rl-lab with
the configured environment:
  /home/kaiser/miniconda3/envs/rl-lab/bin/python tools/probes/jump34_v8_landing_probe.py

Output: compact per-case summaries and /tmp/jump34_v8_landing_probe.csv.
This intentionally reports model/controller mismatches rather than claiming
that the MuJoCo cases reproduce the firmware binary exactly.
"""
from __future__ import annotations

import csv
import math
import sys
import types
from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import balance_lab.control.tuning as tuning
import balance_lab.control.jump as jump_module
import balance_lab.control.controller as controller_module

# Match the firmware's nominal 1 kHz control task and the existing rate probe.
tuning.CONTROL_PERIOD_S = 0.001
jump_module.CONTROL_PERIOD_S = 0.001
controller_module.CONTROL_PERIOD_S = 0.001

from balance_lab import (  # noqa: E402
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    apply_saved_pose,
)

MODEL = ROOT / "model" / "whole_body_lqr_fast.xml"
TRACE = Path("/tmp/jump34_v8_landing_probe.csv")


@dataclass(frozen=True)
class Case:
    name: str
    firmware_air_profile: bool
    capture_theta_zero: bool = False
    landing_wheel_kp: float = 10.0
    landing_wheel_kd: float = 1.0
    tuck_length: float = 0.24
    tuck_time: float = 0.06
    tuck_hold: float = 0.04
    compact_pitch_deg: float = 12.0
    landing_theta_kd: float = 8.0
    compact_descent_hold_min: float = 0.0
    ground_pitch_kp: float = 0.0
    ground_pitch_kd: float = 0.0
    ground_pitch_limit: float = 0.0
    ground_pitch_slew: float = 120.0


CASES = (
    # Existing regression defaults, to expose the tested-vs-v8 profile gap.
    Case("sim_test_defaults", False),
    # Firmware-v8 control allocations on the sim plant, including captured
    # first-owner theta reference and no wheel-speed damping.
    Case("v8_profile_locked_theta", True),
    # This 24/2 landing wheel gain pair is an unverified probe value.
    Case("candidate_landing_wheel24_2_unverified", True, False, 24.0, 2.0),
    # Root-requested single-gain test: reduce landing theta damping only.
    Case("candidate_landing_thetaKD4", True, False, 10.0, 1.0,
         landing_theta_kd=4.0),
    # Profile candidate: deeper compact reference, extended tuck duration,
    # and 4-degree compact pitch bias; landing controller stays at v8 gains.
    Case("FINAL_v9_profile18_14_04_bias4", True, False, 10.0, 1.0,
         tuck_length=0.18, tuck_time=0.14, tuck_hold=0.04,
         compact_pitch_deg=4.0, compact_descent_hold_min=0.02),
)


def _theta(controller: TraditionalLQR, data: mujoco.MjData, side: str) -> float:
    kin = controller.vmc_kinematics(data, side)
    return math.pi / 2.0 - kin.phi0 + controller.body_pitch(data)


def run_case(case: Case, writer: csv.DictWriter) -> dict[str, object]:
    if case.firmware_air_profile:
        jump_module.JUMP_THRUST_BOOST_TIME = 0.32
        jump_module.JUMP_THRUST_LENGTH = 0.36
    else:
        jump_module.JUMP_THRUST_BOOST_TIME = 0.26
        jump_module.JUMP_THRUST_LENGTH = 0.305
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    c = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    c.reset(data, calibrate_frame=True)
    c.enable_calibrated_lqr(data)
    c.set_drive_command(0.0, 0.0)

    if case.firmware_air_profile:
        c.jump_air_tuck_length = case.tuck_length
        c.jump_air_tuck_time = case.tuck_time
        c.jump_air_tuck_hold_time = case.tuck_hold
        c.jump_air_tuck_ref_rate = 1.40
        c.jump_air_landing_length = 0.27
        c.jump_air_deploy_ref_rate = 1.20
        # Approximate the v8 airborne individual 80/12 theta hold and ±28
        # virtual-Tp allocation. The sim does not implement the firmware's
        # observer timestamp pipeline or exact VMC/current/budget path.
        c.jump_air_theta_kp = 80.0
        c.jump_air_theta_kd = 12.0
        c.jump_air_theta_limit = 40.0
        c.jump_air_theta_weight_tuck = 1.0
        c.jump_air_theta_weight_deploy = 1.0
        c.jump_air_common_tp_limit = 28.0  # Firmware JUMP_AIR_TP_LIMIT
        c.jump_air_tuck_pitch_bias = math.radians(case.compact_pitch_deg)

    c.jump_air_wheel_speed_kd = 0.0  # firmware v8 has no wheel-speed term
    c.jump_air_wheel_taper_delay = 0.10
    c.jump_air_wheel_taper_time = 0.20
    if case.firmware_air_profile:
        c.jump_landing_theta_kp = 22.0
        c.jump_landing_theta_kd = case.landing_theta_kd
        c.jump_landing_theta_limit = 28.0
        c.jump_landing_capture_wheel_pitch_kp = case.landing_wheel_kp
        c.jump_landing_capture_wheel_pitch_kd = case.landing_wheel_kd
        c.jump_landing_capture_wheel_torque_limit = 3.69
        c.jump_landing_wheel_pitch_kp = case.landing_wheel_kp
        c.jump_landing_wheel_pitch_kd = case.landing_wheel_kd
        c.jump_landing_wheel_torque_limit = 3.69
        c.jump_landing_capture_wheel_slew_rate = 1000.0
        c.jump_landing_wheel_slew_rate = 1000.0

    # Firmware locks the common theta reference when grounded landing ownership
    # first becomes eligible. Sim normally captures immediately at raw contact;
    # retain that event timing, while changing only the reference/controller law.
    original_hip_control = c._apply_jump_hip_control
    state: dict[str, object] = {
        "theta_ref": None, "first_contact": None, "ground_pitch_tp": 0.0,
        "air_profile_seed_count": 0,
    }
    original_start_air_profile = c._start_jump_air_profile

    def firmware_start_air_profile(self, sample):
        was_active = bool(self.jump_air_profile_active)
        if was_active:
            return
        original_start_air_profile(sample)
        if not self.jump_air_profile_active:
            return
        # The jump profile captures the current mean l0 and clips it to the
        # configured jump-air window. The +20 N/Kp offset belongs to the
        # non-jump airborne path and is intentionally not applied here.
        seeded = float(np.clip(
            0.5 * sum(self.vmc_kinematics(sample, side).l0
                      for side in ("l", "r")),
            0.27, 0.37,
        ))
        self.jump_air_reference = {"l": seeded, "r": seeded}
        state["air_profile_seed_count"] = int(
            state["air_profile_seed_count"]
        ) + 1

    if case.firmware_air_profile:
        c._start_jump_air_profile = types.MethodType(
            firmware_start_air_profile, c
        )

    def v8_landing_hip_control(self, sample, dt):
        original_hip_control(sample, dt)
        if self.jump_off_ground:
            elapsed = float(self.jump_air_elapsed)
            wheel_compact_time = (
                case.tuck_time + case.tuck_hold
                if case.name == "FINAL_v9_profile18_14_04_bias4"
                else 0.06 + 0.04
            )
            if case.name == "FINAL_v9_profile18_14_04_bias4":
                descent_begin = case.tuck_time + case.compact_descent_hold_min
                descending = float(sample.qvel[2]) < -0.20
                compact = elapsed < wheel_compact_time and not (
                    descending and elapsed >= descent_begin
                )
            else:
                compact = elapsed < wheel_compact_time
            pitch = self.body_pitch(sample)
            pitch_rate = -float(self.last_state["l"][5])
            pitch_ref = math.radians(
                case.compact_pitch_deg if compact else 4.0
            )
            theta_common = 0.5 * sum(
                _theta(self, sample, side) for side in ("l", "r")
            )
            theta_rate_common = 0.5 * sum(self.theta_rate_lpf.values())
            theta_hold = float(np.clip(
                30.0 * theta_common + 4.0 * theta_rate_common,
                -22.0, 22.0,
            ))
            theta_weight = 0.8 if compact else 1.5
            wheel_feedback = (
                35.0 * (pitch - pitch_ref) + 5.0 * pitch_rate
                + 0.10 * theta_weight * theta_hold
            )
            # Keep the v8 wheel taper envelope independent of extended compact
            # timing: it starts 0.20 s after release/profile-origin.
            taper_start = 0.20
            wheel_feedback = float(np.clip(wheel_feedback, -3.69, 3.69))
            if elapsed > taper_start:
                progress = float(np.clip(
                    (elapsed - taper_start) / 0.20, 0.0, 1.0
                ))
                wheel_feedback *= 1.0 - progress
            self.jump_air_wheel_torque = wheel_feedback
        if self.landing_buffer_phase == "inactive" or self.jump_off_ground:
            return
        thetas = {side: _theta(self, sample, side) for side in ("l", "r")}
        if case.capture_theta_zero:
            ref = 0.0
        else:
            if state["theta_ref"] is None:
                state["theta_ref"] = 0.5 * (thetas["l"] + thetas["r"])
            ref = float(state["theta_ref"])
        for side in ("l", "r"):
            output = float(np.clip(
                self.jump_landing_theta_kp * (thetas[side] - ref)
                + self.jump_landing_theta_kd * self.theta_rate_lpf[side],
                -self.jump_landing_theta_limit,
                self.jump_landing_theta_limit,
            ))
            # Firmware v8 capture output has no slew filter.
            self.jump_landing_tp_command[side] = output
            self.last_output[side][1] = output
        if self.landing_buffer_phase in {"capture", "hold"}:
            pitch = self.body_pitch(sample)
            pitch_rate_raw = -float(self.last_state["l"][5])
            tau = 0.012 if case.ground_pitch_limit > 0.0 else 0.0
            previous_rate = float(state.get("pitch_rate_lpf", 0.0))
            if tau > 0.0:
                alpha = dt / (tau + dt)
                pitch_rate = previous_rate + alpha * (pitch_rate_raw - previous_rate)
                state["pitch_rate_lpf"] = pitch_rate
            else:
                pitch_rate = pitch_rate_raw
            pitch_target = float(np.clip(
                -case.ground_pitch_kp * pitch
                -case.ground_pitch_kd * pitch_rate,
                -case.ground_pitch_limit,
                case.ground_pitch_limit,
            ))
            previous = float(state["ground_pitch_tp"])
            max_delta = case.ground_pitch_slew * min(dt, 0.001)
            common_pitch = previous + float(np.clip(
                pitch_target - previous, -max_delta, max_delta
            ))
            state["ground_pitch_tp"] = common_pitch
            for side in ("l", "r"):
                self.last_output[side][1] = float(np.clip(
                    self.last_output[side][1] + common_pitch,
                    -28.0, 28.0,
                ))

    if case.firmware_air_profile:
        c._apply_jump_hip_control = types.MethodType(v8_landing_hip_control, c)
        def firmware_air_profile_update(self, dt):
            self.jump_air_elapsed += dt
            descending = float(self._jump_data.qvel[2]) < -0.20
            compact = self.jump_air_elapsed < (
                self.jump_air_tuck_time + self.jump_air_tuck_hold_time
            )
            descent_begin = (
                self.jump_air_tuck_time + case.compact_descent_hold_min
            )
            if descending and self.jump_air_elapsed >= descent_begin:
                compact = False
            target = self.jump_air_tuck_length if compact else self.jump_air_landing_length
            rate = self.jump_air_tuck_ref_rate if compact else self.jump_air_deploy_ref_rate
            for side in ("l", "r"):
                self.jump_air_reference[side] = self._move_towards(
                    self.jump_air_reference[side], target, rate * dt
                )

        c._update_jump_air_profile = types.MethodType(
            firmware_air_profile_update, c
        )

    settle_end = data.time + 3.0
    while data.time < settle_end:
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)

    start_time = float(data.time)
    start_position = data.xpos[c.base].copy()
    start_z = float(data.xpos[c.base, 2])
    accepted, reason = c.begin_jump(data)
    if not accepted:
        raise RuntimeError(f"{case.name}: jump rejected: {reason}")

    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    wheel_bodies = {ids.wheel_body for ids in c.sides.values()}
    first_contact_t = math.nan
    first_contact_pitch = math.nan
    first_contact_logical_wheel_rate = math.nan
    first_contact_raw_wheel_rate_l = math.nan
    first_contact_raw_wheel_rate_r = math.nan
    first_contact_l0 = [math.nan, math.nan]
    first_contact_position = None
    prev_contacts = {"l": False, "r": False}
    prev_off = False
    was_airborne = False
    bounce_off_edges = 0
    bounce_contact_edges = 0
    contact_edges_after_first = 0
    max_pitch_after_contact = 0.0
    min_l0 = [float("inf"), float("inf")]
    air_min_l0 = [float("inf"), float("inf")]
    max_pitch_before_contact = 0.0
    max_flight_pitch_precontact = 0.0
    flight_start_t = math.nan
    air_profile_first_t = math.nan
    air_profile_first_ref = math.nan
    air_profile_deploy_t = math.nan
    previous_air_target = math.nan
    previous_air_elapsed = math.nan
    previous_air_reference = math.nan
    max_air_reference_step = 0.0
    csv_next = start_time
    tail_state = None
    last_pitch_rate = 0.0
    max_abs_pitch_rate_after = 0.0
    tail_samples = 0
    tail_stable_samples = 0
    tail_both_samples = 0
    end_time = start_time + 3.0

    while data.time < end_time:
        c.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        now = float(data.time)
        pitch_deg = math.degrees(c.body_pitch(data))
        pitch_rate = -float(c.last_state["l"][5])
        last_pitch_rate = pitch_rate
        was_airborne = was_airborne or bool(c.jump_off_ground)
        if c.jump_off_ground and math.isnan(flight_start_t):
            flight_start_t = now - start_time
        if c.jump_air_profile_active and math.isnan(air_profile_first_t):
            air_profile_first_t = now - start_time
            air_profile_first_ref = 0.5 * sum(c.jump_air_reference.values())
        if c.jump_air_profile_active:
            elapsed_now = float(c.jump_air_elapsed)
            target_now = 0.5 * sum(c.jump_air_reference.values())
            if not math.isnan(previous_air_elapsed):
                if elapsed_now + 1e-9 < previous_air_elapsed:
                    raise AssertionError(
                        f"{case.name}: air elapsed reset inside active profile"
                    )
                ref_step = abs(target_now - previous_air_reference)
                max_air_reference_step = max(max_air_reference_step, ref_step)
                if ref_step > 0.01:
                    raise AssertionError(
                        f"{case.name}: air reference jumped {ref_step:.4f} m"
                    )
            previous_air_elapsed = elapsed_now
            previous_air_reference = target_now
            if (not math.isnan(previous_air_target)
                    and math.isnan(air_profile_deploy_t)
                    and target_now > previous_air_target + 1e-6):
                air_profile_deploy_t = now - start_time
            previous_air_target = target_now

        contacts = c._wheel_contacts(data)
        if c.jump_off_ground:
            for i, side in enumerate(("l", "r")):
                air_min_l0[i] = min(
                    air_min_l0[i], c.vmc_kinematics(data, side).l0
                )
            if math.isnan(first_contact_t):
                max_flight_pitch_precontact = max(
                    max_flight_pitch_precontact, abs(pitch_deg)
                )
        for side in ("l", "r"):
            kin = c.vmc_kinematics(data, side)
            min_l0[0 if side == "l" else 1] = min(
                min_l0[0 if side == "l" else 1], kin.l0
            )
            if contacts[side] and not prev_contacts[side] and not math.isnan(first_contact_t):
                contact_edges_after_first += 1
                bounce_contact_edges += 1
        if not math.isnan(first_contact_t):
            max_pitch_after_contact = max(max_pitch_after_contact, abs(pitch_deg))
            max_abs_pitch_rate_after = max(max_abs_pitch_rate_after, abs(pitch_rate))
        if c.jump_off_ground and not prev_off and not math.isnan(first_contact_t):
            bounce_off_edges += 1

        wheels_touching = set()
        for idx in range(data.ncon):
            con = data.contact[idx]
            if floor not in (con.geom1, con.geom2):
                continue
            other = con.geom2 if con.geom1 == floor else con.geom1
            body = int(model.geom_bodyid[other])
            if body in wheel_bodies:
                wheels_touching.add(body)
        if was_airborne and math.isnan(first_contact_t) and wheels_touching:
            first_contact_t = now - start_time
            first_contact_pitch = pitch_deg
            first_contact_position = data.xpos[c.base].copy()
            for i, side in enumerate(("l", "r")):
                ids = c.sides[side]
                dof = model.jnt_dofadr[ids.wheel_joint]
                raw = float(data.qvel[dof])
                if side == "l":
                    first_contact_raw_wheel_rate_l = raw
                else:
                    first_contact_raw_wheel_rate_r = raw
                first_contact_l0[i] = c.vmc_kinematics(data, side).l0
            first_contact_logical_wheel_rate = 0.5 * sum(
                c.wheel_sign[side] * float(data.qvel[
                    model.jnt_dofadr[c.sides[side].wheel_joint]
                ]) for side in ("l", "r")
            )

        both = contacts["l"] and contacts["r"]
        phase = c.landing_buffer_phase
        stable = (
            phase == "inactive"
            and c.jump_state == "idle"
            and both
            and abs(pitch_deg) < 5.0
            and abs(pitch_rate) < math.radians(20.0)
        )
        tail_state = {
            "phase": phase,
            "jump_state": c.jump_state,
            "both": both,
            "stable": stable,
            "pitch": pitch_deg,
            "pitch_rate": math.degrees(pitch_rate),
            "vx": c.chassis_vx,
        }
        if now >= end_time - 0.2:
            tail_samples += 1
            tail_stable_samples += int(stable)
            tail_both_samples += int(both)

        if now + 1e-12 >= csv_next:
            position = data.xpos[c.base]
            post_contact_drift = math.nan
            if first_contact_position is not None:
                post_contact_drift = -float(np.dot(
                    position - first_contact_position, c.forward_world
                ))
            row = {
                "case": case.name,
                "t_from_jump_s": now - start_time,
                "jump_state": c.jump_state,
                "buffer_phase": phase,
                "off_ground": int(c.jump_off_ground),
                "wheel_contact_l": int(contacts["l"]),
                "wheel_contact_r": int(contacts["r"]),
                "base_z_m": float(data.xpos[c.base, 2]),
                "root_vz_qvel": float(data.qvel[2]),
                "air_elapsed_s": float(c.jump_air_elapsed),
                "air_ref_avg_m": (
                    0.5 * sum(c.jump_air_reference.values())
                    if c.jump_air_profile_active else math.nan
                ),
                "pitch_deg": pitch_deg,
                "pitch_rate_deg_s": math.degrees(pitch_rate),
                "theta_l_rad": _theta(c, data, "l"),
                "theta_r_rad": _theta(c, data, "r"),
                "l0_l_m": c.vmc_kinematics(data, "l").l0,
                "l0_r_m": c.vmc_kinematics(data, "r").l0,
                "ref_l_m": c.length_reference["l"],
                "ref_r_m": c.length_reference["r"],
                "hip_tp_l": float(c.last_output["l"][1]),
                "hip_tp_r": float(c.last_output["r"][1]),
                "wheel_cmd_l_nm": float(data.ctrl[c.sides["l"].wheel_motor]),
                "wheel_cmd_r_nm": float(data.ctrl[c.sides["r"].wheel_motor]),
                "wheel_rate_l_raw_rad_s": float(data.qvel[
                    model.jnt_dofadr[c.sides["l"].wheel_joint]
                ]),
                "wheel_rate_r_raw_rad_s": float(data.qvel[
                    model.jnt_dofadr[c.sides["r"].wheel_joint]
                ]),
                "wheel_rate_logical_rad_s": 0.5 * sum(
                    c.wheel_sign[side] * float(data.qvel[
                        model.jnt_dofadr[c.sides[side].wheel_joint]
                    ]) for side in ("l", "r")
                ),
                "postcontact_drift_m": post_contact_drift,
                "jump_fault": c.jump_fault,
            }
            writer.writerow(row)
            csv_next += 0.010
        prev_contacts = contacts
        prev_off = c.jump_off_ground

    forward_delta = math.nan
    post_contact_drift = math.nan
    if first_contact_position is not None:
        forward_delta = -float(np.dot(
            first_contact_position - start_position, c.forward_world
        ))
        post_contact_drift = -float(np.dot(
            data.xpos[c.base] - first_contact_position, c.forward_world
        ))
    # Simple standing capture indicator in the final 200 ms. The probe does
    # not treat simulator success as hardware safety evidence.
    if tail_state is not None:
        landed_stable = bool(tail_state["stable"])
    else:
        landed_stable = False
    result = {
        "case": case.name,
        "air_profile_t": air_profile_first_t,
        "air_profile_ref_at_start": air_profile_first_ref,
        "air_profile_seed_count": state["air_profile_seed_count"],
        "max_air_reference_step_m": max_air_reference_step,
        "deploy_t": air_profile_deploy_t,
        "jump_start_to_first_contact_s": first_contact_t,
        "takeoff_to_first_contact_s": (
            first_contact_t - flight_start_t
            if not math.isnan(first_contact_t) and not math.isnan(flight_start_t)
            else math.nan
        ),
        "contact_pitch_deg": first_contact_pitch,
        "contact_theta_ref_rad": state["theta_ref"],
        "contact_l0_l": first_contact_l0[0],
        "contact_l0_r": first_contact_l0[1],
        "min_l0_l": min_l0[0],
        "min_l0_r": min_l0[1],
        "flight_min_l0_l": air_min_l0[0],
        "flight_min_l0_r": air_min_l0[1],
        "contact_wheel_rate_logical": first_contact_logical_wheel_rate,
        "contact_wheel_rate_l_raw": first_contact_raw_wheel_rate_l,
        "contact_wheel_rate_r_raw": first_contact_raw_wheel_rate_r,
        "max_abs_pitch_flight_precontact_deg": max_flight_pitch_precontact,
        "max_abs_pitch_postcontact_deg": max_pitch_after_contact,
        "max_abs_pitch_rate_postcontact_deg_s": math.degrees(max_abs_pitch_rate_after),
        "bounce_off_edges": bounce_off_edges,
        "wheel_contact_edges_after_first": contact_edges_after_first,
        "buffer_final": c.landing_buffer_phase,
        "jump_state_final": c.jump_state,
        "fault_final": c.jump_fault,
        "stable_final_sample": landed_stable,
        "last200ms_stable_ratio": tail_stable_samples / max(tail_samples, 1),
        "last200ms_bothwheel_ratio": tail_both_samples / max(tail_samples, 1),
        "forward_to_contact_m": forward_delta,
        "postcontact_drift_m": post_contact_drift,
        "final_pitch_deg": tail_state["pitch"] if tail_state else math.nan,
        "final_pitch_rate_deg_s": tail_state["pitch_rate"] if tail_state else math.nan,
        "final_vx_m_s": tail_state["vx"] if tail_state else math.nan,
    }
    if case.firmware_air_profile and state["air_profile_seed_count"] != 1:
        raise AssertionError(
            f"{case.name}: profile seeded {state['air_profile_seed_count']} times"
        )
    print(
        f"{case.name}: takeoff->contact={result['takeoff_to_first_contact_s']:.3f}s "
        f"airMinL0={air_min_l0[0]:.3f}/{air_min_l0[1]:.3f}m "
        f"contact pitch={first_contact_pitch:+.2f}deg "
        f"thetaRef={float(state['theta_ref']) if state['theta_ref'] is not None else math.nan:+.3f}rad "
        f"l0min={min_l0[0]:.3f}/{min_l0[1]:.3f}m "
        f"wheelRate(logical)={first_contact_logical_wheel_rate:+.1f}rad/s "
        f"postPitchMax={max_pitch_after_contact:.1f}deg "
        f"bounceOff={bounce_off_edges} buffer={c.landing_buffer_phase} "
        f"fault={c.jump_fault} drift={post_contact_drift:+.3f}m "
        f"finalPitch/Vx={result['final_pitch_deg']:+.1f}deg/"
        f"{result['final_vx_m_s']:+.2f}m/s"
    )
    return result


def main() -> None:
    fields = (
        "case", "t_from_jump_s", "jump_state", "buffer_phase", "off_ground",
        "wheel_contact_l", "wheel_contact_r", "base_z_m", "root_vz_qvel",
        "air_elapsed_s", "air_ref_avg_m", "pitch_deg",
        "pitch_rate_deg_s", "theta_l_rad", "theta_r_rad", "l0_l_m", "l0_r_m",
        "ref_l_m", "ref_r_m", "hip_tp_l", "hip_tp_r", "wheel_cmd_l_nm",
        "wheel_cmd_r_nm", "wheel_rate_l_raw_rad_s", "wheel_rate_r_raw_rad_s",
        "wheel_rate_logical_rad_s", "postcontact_drift_m", "jump_fault",
    )
    results = []
    with TRACE.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for case in (
            CASES[0],  # true simulator test defaults
            CASES[1],  # firmware-v8-like
            CASES[4],  # final isolated v9 profile candidate
        ):
            results.append(run_case(case, writer))
    print(f"trace_csv={TRACE}")
    print("case_summary=" + repr(results))


if __name__ == "__main__":
    main()
