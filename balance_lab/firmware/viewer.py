"""Interactive and headless entry points using the shared C controller."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import threading
import time
import numpy as np
from .runtime import Command, ChassisMode, GimbalMode
from .simulation import VehicleSimulation
from .recording import Recorder


def command_for(args, elapsed):
    right = {"relax": 1, "remote": 3, "spin": 2}.get(args.mode, 3)
    if args.mode in {"relax", "remote", "spin"}:
        wheel = 660 if args.jump_at is not None and args.jump_at <= elapsed < args.jump_at + .050 else 0
        left = 1 if args.bump_at is not None and args.bump_at <= elapsed < args.bump_at + .100 else args.leg_switch
        return Command.remote(channels=(0, args.vx_raw, args.yaw_raw, 0, wheel), right_switch=right, left_switch=left)
    mode = {"open": ChassisMode.OPEN_LOOP, "recovery": ChassisMode.RECOVERY,
            "follow": ChassisMode.FOLLOW_GIMBAL}[args.mode]
    return Command(chassis_mode=mode, gimbal_mode=GimbalMode.GYRO, vx_raw=args.vx_raw,
                   gimbal_yaw_raw=args.yaw_raw, leg_level={2:0,3:1,1:2}[args.leg_switch],
                   jump_request=int(args.jump_at is not None and args.jump_at <= elapsed < args.jump_at+.050),
                   bump_request=int(args.bump_at is not None and args.bump_at <= elapsed))


def summary(sim):
    out = sim.output
    return {"elapsed_s": round(sim.ticks*sim.tick_s, 6), "base_z_m": float(sim.data.xpos[sim.base, 2]),
            "pitch_deg": float(out.imu[1]), "roll_deg": float(out.imu[0]),
            "leg_length_m": list(out.length), "leg_phi0_deg": np.degrees(list(out.phi0)).tolist(),
            "chassis_mode": out.chassis_mode, "stand_state": out.stand_state,
            "recovery_state": out.recovery_state, "recovery_fault": out.recovery_fault,
            "jump_state": out.jump_state, "jump_fault": out.jump_fault, "bump_state": out.bump_state,
            "gimbal_ready": out.gimbal_ready, "yaw_relative_rad": out.yaw_relative,
            "wheel_current": list(out.wheel_current), "motor_tx_count": list(out.motor_tx_count)}


def headless(args):
    sim = VehicleSimulation(args.variant, plant=args.plant, pose=args.pose)
    recorder = Recorder(args.record, sim.metadata(), args.pages) if args.record else None
    try:
        for _ in range(round(args.seconds / sim.tick_s)):
            command = command_for(args, sim.ticks*sim.tick_s)
            sim.step(command)
            if recorder:
                recorder.tick(sim, command)
        result = {"metadata": sim.metadata(), "final": summary(sim)}
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
        print(json.dumps(result["final"], ensure_ascii=False, indent=2, allow_nan=False))
    finally:
        if recorder:
            recorder.close()
        sim.close()


def interactive(args):
    import mujoco
    import mujoco.viewer
    from mujoco.glfw import glfw
    sim = VehicleSimulation(args.variant, plant=args.plant, pose=args.pose)
    keys, lock = [], threading.Lock()
    state = {"paused": True, "right": 3, "left": args.leg_switch, "window": None,
             "jump_until": -1.0, "mode": args.mode, "follow": True}
    def callback(key):
        if state["window"] is None:
            state["window"] = glfw.get_current_context()
        with lock:
            keys.append(key)
    try:
        with mujoco.viewer.launch_passive(sim.model, sim.data, key_callback=callback) as view:
            view.cam.distance = 1.5
            view.cam.elevation = -15
            while view.is_running():
                start = time.monotonic()
                with lock:
                    pending = list(keys); keys.clear()
                with view.lock():
                    for key in pending:
                        if key == glfw.KEY_SPACE: state["paused"] = not state["paused"]
                        elif key == glfw.KEY_R:
                            sim.reset()
                            state["paused"] = True
                            state["jump_until"] = -1.0
                        elif key == glfw.KEY_1: state["right"] = 1; state["mode"] = "remote"
                        elif key == glfw.KEY_2: state["right"] = 3; state["mode"] = "remote"
                        elif key == glfw.KEY_3: sim.reset("stand"); state["paused"] = True; state["jump_until"] = -1.0
                        elif key == glfw.KEY_5: sim.reset("encoder"); state["paused"] = True; state["jump_until"] = -1.0
                        elif key == glfw.KEY_7: state["left"] = 2
                        elif key == glfw.KEY_8: state["left"] = 3
                        elif key in (glfw.KEY_9, glfw.KEY_B): state["left"] = 1
                        elif key == glfw.KEY_T: state["right"] = 2 if state["right"] != 2 else 3; state["mode"] = "remote"
                        elif key == glfw.KEY_J: state["jump_until"] = sim.ticks*sim.tick_s + .050
                        elif key == glfw.KEY_F: state["follow"] = not state["follow"]
                        elif key == glfw.KEY_X: state["right"] = 1; state["mode"] = "remote"
                def held(key):
                    return state["window"] is not None and glfw.get_key(state["window"], key) == glfw.PRESS
                vx = 330 * (int(held(glfw.KEY_UP)) - int(held(glfw.KEY_DOWN)))
                yaw = 100 * (int(held(glfw.KEY_RIGHT)) - int(held(glfw.KEY_LEFT)))
                if not state["paused"]:
                    for _ in range(round(.010/sim.tick_s)):
                        elapsed = sim.ticks*sim.tick_s
                        cmd = Command.remote(channels=(0,vx,yaw,0,660 if elapsed < state["jump_until"] else 0),
                                             right_switch=state["right"], left_switch=state["left"])
                        if state["mode"] not in {"remote", "relax", "spin"}:
                            cmd = command_for(args, elapsed)
                        elif state["mode"] == "relax": cmd.rc_switches[0] = 1
                        elif state["mode"] == "spin": cmd.rc_switches[0] = 2
                        sim.step(cmd)
                s = summary(sim)
                # MuJoCo's bundled font is Latin only; transliterate the UI so
                # the native overlay remains readable on all installations.
                status = (f"Vehicle C | {'PAUSED' if state['paused'] else 'RUNNING'} | {sim.ticks*sim.tick_s:.2f} s\n"
                          f"mode/stand/recovery {s['chassis_mode']}/{s['stand_state']}/{s['recovery_state']}\n"
                          f"recovery fault {s['recovery_fault']}\n"
                          f"pitch/roll {s['pitch_deg']:+.2f}/{s['roll_deg']:+.2f} deg\n"
                          f"leg L/R {s['leg_length_m'][0]:.3f}/{s['leg_length_m'][1]:.3f} m\n"
                          f"leg phi L/R {s['leg_phi0_deg'][0]:.1f}/{s['leg_phi0_deg'][1]:.1f} deg\n"
                          f"wheel current L/R {s['wheel_current'][0]:+.0f}/{s['wheel_current'][1]:+.0f}\n"
                          f"jump/fault/bump {s['jump_state']}/{s['jump_fault']}/{s['bump_state']}\n"
                          f"yaw ready {s['gimbal_ready']} relative {np.degrees(s['yaw_relative_rad']):+.2f} deg")
                help_text = ("Space run/pause | R reset + pause | F follow camera\n1 relax | 2 RC | T spin | X relax\n"
                             "3 stand spawn | 5 encoder-zero spawn\nArrows drive / gimbal yaw | J RC jump\n"
                             "7 LOW | 8 MID | 9/B HIGH + bump request")
                with view.lock():
                    if state["follow"]: view.cam.lookat[:] = sim.data.xpos[sim.base]
                    view.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, status, help_text))
                view.sync()
                time.sleep(max(0.0, .010-(time.monotonic()-start)))
    finally:
        sim.close()


def main():
    parser = argparse.ArgumentParser(description="MuJoCo trial ground running the vehicle C firmware")
    parser.add_argument("--controller", choices=("vehicle", "legacy"), default="vehicle")
    parser.add_argument("--sysid", action="store_true", help="legacy compatibility; vehicle already defaults to the identified plant")
    parser.add_argument("--plant", default="identified_2026-09-21")
    parser.add_argument("--variant", choices=("fast", "full", "platform"), default="fast")
    parser.add_argument("--pose", choices=("stand", "encoder"), default="encoder")
    parser.add_argument("--mode", choices=("relax", "remote", "spin", "open", "follow", "recovery"), default="remote")
    parser.add_argument("--leg-switch", choices=(1,2,3), type=int, default=2)
    parser.add_argument("--vx-raw", type=int, default=0)
    parser.add_argument("--yaw-raw", type=int, default=0)
    parser.add_argument("--jump-at", type=float)
    parser.add_argument("--bump-at", type=float)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--record", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--pages", type=int, nargs="*", default=(43,54,64))
    args = parser.parse_args()
    if args.record and not args.headless:
        parser.error("--record currently requires --headless")
    if args.controller == "legacy":
        if args.headless:
            parser.error("legacy headless runs use the historical probes/tests")
        from ..sim.viewer import main as legacy
        legacy()
    elif args.headless:
        if args.seconds <= 0: parser.error("--seconds must be positive")
        headless(args)
    else:
        interactive(args)
