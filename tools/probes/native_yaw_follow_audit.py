"""Diagnose yaw feedback and mechanical Follow with the unchanged vehicle C.

Run through jointpd's heavy_run.sh. Angle-only cases isolate the outer loop.
The coherent_gimbal case separately tests reverse gearing and a world-positive
gimbal IMU convention; this candidate still requires actual hardware validation.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from balance_lab.firmware import Command, Output, VehicleRuntime
from balance_lab.firmware.recording import fields, restore
from balance_lab.firmware.runtime import Feedback
from balance_lab.firmware.simulation import WHEEL_NAMES
from balance_lab.firmware.viewer import summary
from tools.probes.native_friction_audit import contacts
from tools.probes.native_wheel_handoff_audit import WheelHandoffSimulation

PAGE50 = ("time_s", "mode", "stand", "leg_level", "relative_rad", "error_rad",
          "rate_limit", "rate_desired", "rate_ref", "rate_fdb", "rate_error",
          "yaw_raw", "yaw_output", "yaw_integral", "yaw_state", "blocked", "vx_cmd",
          "vx_fdb", "vx_delta", "net_l", "net_r", "speed_l", "speed_r", "torque_l",
          "torque_r", "yaw_cmd_l", "yaw_cmd_r", "leg_split", "theta_split", "roll", "f_roll", "safety")


class FollowDiagnosticSimulation(WheelHandoffSimulation):
    def __init__(self, *args, follow_case="unchanged", **kwargs):
        self.follow_case = follow_case
        self.first_follow_override_s = None
        super().__init__(*args, wheel_case="both_neg_from_start", **kwargs)

    def read_feedback(self):
        f = super().read_feedback()
        if self.follow_case == "coherent_gimbal":
            f.yaw_motor_angle *= -1
            f.yaw_motor_speed *= -1
            f.yaw_motor_torque *= -1
            f.gimbal_yaw_deg *= -1
            f.gimbal_yaw_rate *= -1
            return f
        # Original homing still receives its real motor position/rate. In GYRO,
        # the gimbal's own control uses world IMU angle/rate, not this position.
        if self.output.chassis_mode == 5 and self.output.gimbal_ready:
            if self.follow_case != "unchanged" and self.first_follow_override_s is None:
                self.first_follow_override_s = f.time_us / 1e6
            if self.follow_case == "angle_neg":
                f.yaw_motor_angle *= -1
            elif self.follow_case == "angle_zero":
                f.yaw_motor_angle = 0
        return f

    def apply_outputs(self, output, feedback):
        super().apply_outputs(output, feedback)
        if self.follow_case == "coherent_gimbal":
            self.data.ctrl[self.actuators["gimbal_yaw"]] *= -1

    def metadata(self):
        result = super().metadata()
        result["follow_counterfactual"] = self.follow_case
        result["follow_counterfactual_scope"] = (
            "all ticks: motor angle/rate/torque=-CAD output joint; CAD torque=-motor torque; gimbal IMU yaw/rate=positive world heading/rate"
            if self.follow_case == "coherent_gimbal" else
            "yaw motor angle only while previous output is Follow and homing-ready; no IMU/rate/torque change")
        return result


def replay_trace(record, output):
    rows = []
    count = 0
    with VehicleRuntime() as runtime, record.open() as stream, output.open("w", newline="") as dest:
        writer = csv.writer(dest)
        writer.writerow(PAGE50 + ("raw_z", "published_z", "chassis_yaw_deg", "gimbal_yaw_deg", "motor_angle", "relative_published", "current_l", "current_r", "gimbal_torque"))
        for line in stream:
            row = json.loads(line)
            if "metadata" in row:
                meta = row["metadata"]
                if meta["firmware"] != runtime.manifest["fingerprint"] or meta["native_build"] != runtime.native_fingerprint:
                    raise ValueError("recording identity differs from current C runtime")
                continue
            f = restore(Feedback, row["feedback"])
            out = runtime.step(f, restore(Command, row["command"]))
            if fields(out) != row["output"]:
                raise AssertionError(f"replay differs at {f.time_us}")
            for page, values in row.get("pages", {}).items():
                if runtime.debug_page(int(page)) != values:
                    raise AssertionError(f"debug replay differs at {f.time_us}, page {page}")
            count += 1
            rows.append((f.time_us/1e6, f.gyro[2], out.imu[5], out.imu[2], f.gimbal_yaw_deg,
                         f.yaw_motor_angle, out.yaw_relative))
            if count % 10 == 0:
                writer.writerow(tuple(runtime.debug_page(50)[:32]) +
                                (f.gyro[2], out.imu[5], out.imu[2], f.gimbal_yaw_deg,
                                 f.yaw_motor_angle, out.yaw_relative, *out.wheel_current, out.yaw_command[4]))
    a = np.asarray(rows)
    yaw = np.radians(a[:, 3])  # ABI contains original accumulated yaw, already unwrapped.
    gimbal = np.radians(a[:, 4])
    relative = np.unwrap(a[:, 6])
    motor = np.unwrap(a[:, 5])
    mask = a[:, 0] >= 1.5
    # Compare 100 ms angle increments with mean gyro. Small residuals reflect
    # Euler/body-rate coupling, estimator correction and one-tick phase.
    errors = []
    derivatives = []
    rates = []
    for start in range(1500, len(a)-100, 100):
        derivative = (yaw[start+100]-yaw[start]) / .1
        rate = float(np.mean(a[start:start+100, 2]))
        errors.append(derivative-rate); derivatives.append(derivative); rates.append(rate)
    return {"ticks": count, "matched": True, "metadata": meta,
            "raw_z_plus_published_z_max_rad_s": float(np.max(abs(a[:, 1]+a[:, 2]))),
            "yaw_derivative_gyro_correlation_100ms": float(np.corrcoef(derivatives, rates)[0, 1]),
            "yaw_derivative_minus_gyro_rms_rad_s_100ms": float(np.sqrt(np.mean(np.square(errors)))),
            "relative_vs_gimbal_minus_chassis_slope": float(np.polyfit((gimbal-yaw)[mask], relative[mask], 1)[0]),
            "relative_vs_negative_motor_slope": float(np.polyfit(-motor[mask], relative[mask], 1)[0]),
            "trace_csv": str(output.resolve())}


def gyro_z(model, data, sim):
    velocity = np.zeros(6)
    mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, sim.base, velocity, 0)
    frame = data.xmat[sim.base].reshape(3, 3) @ sim.sensor_body
    return -float((frame.T @ velocity[:3])[2])


def matched_pulses():
    sim = WheelHandoffSimulation("fast", pose="encoder", wheel_case="both_neg_from_start")
    try:
        for _ in range(1600):
            t = sim.ticks*sim.tick_s
            out = sim.step(Command.remote(right_switch=1 if t < .05 else 3))
        if out.chassis_mode != 5 or out.stand_state != 2 or not out.gimbal_ready:
            raise AssertionError("matched pulse state is not homed, standing Follow")
        spec = mujoco.mjtState.mjSTATE_INTEGRATION
        state = np.zeros(mujoco.mj_stateSize(sim.model, spec))
        mujoco.mj_getState(sim.model, sim.data, state, spec)
        before = gyro_z(sim.model, sim.data, sim)
        rows = []
        for mapping, direction in (("default", -1), ("both_neg", 1)):
            for u in (-.1, .1):
                data = mujoco.MjData(sim.model)
                mujoco.mj_setState(sim.model, data, state, spec)
                mujoco.mj_forward(sim.model, data)
                for name in WHEEL_NAMES:
                    data.ctrl[sim.actuators[name]] += direction*u
                for _ in range(10):
                    mujoco.mj_step(sim.model, data)
                if any(data.warning[w].number for w in (mujoco.mjtWarning.mjWARN_BADQPOS,
                       mujoco.mjtWarning.mjWARN_BADQVEL, mujoco.mjtWarning.mjWARN_BADQACC)):
                    raise RuntimeError("invalid signed yaw pulse")
                after = gyro_z(sim.model, data, sim)
                rows.append({"mapping": mapping, "virtual_yaw_increment_nm": u,
                             "cad_wheel_torque_increment_nm": [direction*u]*2,
                             "gyro_before_rad_s": before, "gyro_after_rad_s": after,
                             "mean_yaw_accel_rad_s2": (after-before)/.01})
        slopes = {}
        for mapping in ("default", "both_neg"):
            a, b = [r for r in rows if r["mapping"] == mapping]
            slopes[mapping] = (b["gyro_after_rad_s"]-a["gyro_after_rad_s"])/(.2*.01)
        return {"time_s": 1.6, "state": summary(sim), "contacts": contacts(sim), "duration_s": .01,
                "probe": "same integration state, held hips/gimbal/gas/contact; +/-0.1 Nm increment in each wheel; C does not run during pulse",
                "yaw_acceleration_per_virtual_yaw_nm": slopes, "rows": rows}
    finally:
        sim.close()


def run_follow(case, seconds, directory, scenario="zero"):
    sim = FollowDiagnosticSimulation("fast", pose="encoder", follow_case=case)
    path = directory/(case+("_"+scenario if scenario != "zero" else "")+".csv")
    tail = []
    try:
        with path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(PAGE50 + ("pitch_deg", "roll_deg", "raw_z", "yaw_relative", "gimbal_yaw_deg",
                            "current_l", "current_r", "contact_l_n", "contact_r_n", "other_n"))
            for _ in range(round(seconds/sim.tick_s)):
                t = sim.ticks*sim.tick_s
                yaw_cmd = (100 if scenario == "gimbal_pos" else -100 if scenario == "gimbal_neg" else 0) if 5 <= t < 5.4 else 0
                out = sim.step(Command.remote(right_switch=1 if t < .05 else 3, channels=(0, 0, yaw_cmd, 0, 0)))
                if sim.ticks % 10 == 0:
                    p = sim.runtime.debug_page(50)
                    c = contacts(sim)
                    writer.writerow(tuple(p[:32])+(out.imu[1], out.imu[0], sim.feedback.gyro[2], out.yaw_relative,
                                     sim.feedback.gimbal_yaw_deg, *out.wheel_current, c["left_n"], c["right_n"], c["other_n"]))
                    if sim.ticks*sim.tick_s > seconds-1:
                        tail.append((out.imu[1], out.imu[0], out.imu[5], out.yaw_relative,
                                     min(c['left_n'], c['right_n']) > 1, c['other_n'], out.stand_state == 2))
        a = np.asarray(tail)
        return {"follow_case": case, "scenario": scenario, "duration_s": seconds, "first_follow_override_s": sim.first_follow_override_s,
                "metadata": sim.metadata(), "final": summary(sim), "tail_window_s": [seconds-1, seconds],
                "metrics": {"pitch_abs_max_deg": float(max(abs(a[:, 0]))), "roll_abs_max_deg": float(max(abs(a[:, 1]))),
                            "yaw_rate_abs_max_rad_s": float(max(abs(a[:, 2]))),
                            "yaw_relative_abs_max_deg": float(np.degrees(max(abs(a[:, 3])))),
                            "both_wheels_contact_fraction": float(np.mean(a[:, 4])),
                            "nonwheel_contact_peak_n": float(max(a[:, 5])), "stand_ack_fraction": float(np.mean(a[:, 6]))},
                "trace_csv": str(path.resolve())}
    finally:
        sim.close()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--record", type=Path, default=ROOT/"out/native/wheel_handoff_video_20261014/recovery_fast.jsonl")
    p.add_argument("--output", type=Path, default=ROOT/"out/native/yaw_follow_audit_20261014")
    p.add_argument("--seconds", type=float, default=8)
    p.add_argument("--cases", nargs="+", choices=("angle_neg", "angle_zero", "coherent_gimbal"), default=("angle_neg", "angle_zero"))
    p.add_argument("--gimbal-check", action="store_true", help="also run +/-100 remote gimbal commands at 5--5.4 s for coherent_gimbal")
    args = p.parse_args()
    if not 3 <= args.seconds <= 10:
        p.error("duration must be 3 to 10 seconds")
    if args.gimbal_check and (args.seconds < 7 or "coherent_gimbal" not in args.cases):
        p.error("gimbal check requires coherent_gimbal and at least 7 seconds")
    args.output.mkdir(parents=True, exist_ok=True)
    report_path = args.output/"report.json"
    if report_path.exists():
        p.error("report exists; choose a new output directory")
    report = {"probe_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "record_sha256": hashlib.sha256(args.record.read_bytes()).hexdigest(), "results": []}
    def save():
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    report["replay"] = replay_trace(args.record, args.output/"record_page50.csv")
    save(); print("replay", report["replay"], flush=True)
    report["matched_pulses"] = matched_pulses()
    save(); print("matched pulses", report["matched_pulses"]["yaw_acceleration_per_virtual_yaw_nm"], flush=True)
    for case in args.cases:
        result = run_follow(case, args.seconds, args.output)
        report["results"].append(result)
        save(); print(case, result["metrics"], flush=True)
        if case == "coherent_gimbal" and args.gimbal_check:
            for scenario in ("gimbal_pos", "gimbal_neg"):
                result = run_follow(case, args.seconds, args.output, scenario)
                report["results"].append(result)
                save(); print(case, scenario, result["metrics"], flush=True)


if __name__ == "__main__":
    main()
