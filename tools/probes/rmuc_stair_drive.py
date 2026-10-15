"""Bounded real-control arena approach with contact and BUMP diagnostics."""
import argparse
import faulthandler
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import mujoco
import numpy as np
from balance_lab.firmware.simulation import VehicleSimulation
from balance_lab.firmware.operator_console import OperatorInput, OperatorRenderer, telemetry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", default="rmuc2026")
    parser.add_argument("--spawn", type=float, nargs=2, default=(-5.75, -2.25))
    parser.add_argument("--heading", type=float, default=0)
    parser.add_argument("--seconds", type=float, default=20)
    parser.add_argument("--bump", action="store_true")
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    stack = (args.output / "stacks.log").open("w")
    faulthandler.dump_traceback_later(90, file=stack, exit=True)
    sim = VehicleSimulation(args.variant, interface_mapping="coherent_candidate",
                            yaw_damping_nms_rad=0, controller_profile="jump46")
    state = OperatorInput(paused=False)
    wb = [sim._id(mujoco.mjtObj.mjOBJ_BODY, n) for n in ("l_wheel", "r_wheel")]
    rows, snapshots, worst, began = [], [], 0., time.monotonic()
    renderer = None
    try:
        group = np.array([0, 0, 0, 1, 0, 0], dtype=np.uint8)
        gid = np.empty(1, dtype=np.int32)
        z = 5 - mujoco.mj_ray(sim.model, sim.data, np.array([*args.spawn, 5.]),
                              np.array([0., 0., -1.]), group, 1, -1, gid)
        assert z < 5
        sim.spawn_xy, sim.z_offset = tuple(args.spawn), z
        sim.reset("encoder")
        a = np.radians(args.heading) / 2
        quat = np.array([np.cos(a), 0., 0., np.sin(a)])
        old = sim.data.qpos[3:7].copy()
        mujoco.mju_mulQuat(sim.data.qpos[3:7], quat, old)
        mujoco.mj_forward(sim.model, sim.data)
        if args.render:
            renderer = OperatorRenderer(sim)
        print("SPAWN", args.spawn, z, "heading", args.heading, flush=True)
        with (args.output / "trace.jsonl").open("w", buffering=1) as stream:
            for tick in range(round(args.seconds / sim.tick_s)):
                elapsed = sim.ticks * sim.tick_s
                if tick == 5000 and args.bump:
                    state.press("c", elapsed)
                state.keys = {"w"} if elapsed >= 5.5 else set()
                before = time.perf_counter()
                sim.step(state.command(elapsed))
                duration = time.perf_counter() - before
                worst = max(worst, duration)
                if tick % 20 == 19:
                    row = dict(telemetry(sim), position_m=sim.data.xpos[sim.base].tolist(),
                        wheel_bottom_m=(sim.data.xpos[wb, 2] - sim.runtime.parameters["wheel_radius_m"]).tolist(),
                        bump_page=sim.runtime.debug_page(54), contacts=sim.data.ncon,
                        step_ms=duration * 1000)
                    if tick % 1000 == 999:
                        force = np.empty(6)
                        row["contact_pairs"] = []
                        for ci, contact in enumerate(sim.data.contact):
                            mujoco.mj_contactForce(sim.model, sim.data, ci, force)
                            row["contact_pairs"].append(dict(
                                geoms=[mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in contact.geom],
                                pos=contact.pos.tolist(), force_n=float(force[0])))
                    rows.append(row)
                    snapshots.append((sim.data.qpos.copy(), sim.data.qvel.copy()))
                    stream.write(json.dumps(row) + "\n")
                    if tick % 1000 == 999:
                        print(json.dumps({k:v for k,v in row.items() if k != "bump_page"}), flush=True)
                if renderer and tick % 40 == 39:
                    rb = time.monotonic()
                    frame = renderer.render(state, telemetry(sim))
                    if time.monotonic()-rb > .1:
                        print("SLOW_RENDER", tick, time.monotonic()-rb, flush=True)
                    if tick % 1000 == 999:
                        frame.save(args.output / "latest.png")
        np.savez_compressed(args.output / "trajectory.npz",
                            qpos=np.array([x[0] for x in snapshots]),
                            qvel=np.array([x[1] for x in snapshots]))
        report = dict(variant=args.variant, spawn=args.spawn, heading=args.heading,
                      bump=args.bump, simulated_s=sim.ticks*sim.tick_s,
                      wall_s=time.monotonic()-began, worst_step_ms=worst*1000,
                      max_bump_state=max(r["bump"] for r in rows), final=rows[-1])
        (args.output / "report.json").write_text(json.dumps(report, indent=2)+"\n")
        print("RESULT", json.dumps(report), flush=True)
    finally:
        faulthandler.cancel_dump_traceback_later()
        stack.close()
        if renderer:
            renderer.close()
        sim.close()


if __name__ == "__main__":
    main()
