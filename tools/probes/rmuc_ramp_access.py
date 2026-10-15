"""Drive the original C controller up/down the CAD central ramp; record GPU replay."""
from pathlib import Path
import hashlib
import json
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import imageio.v2 as imageio
import mujoco
import numpy as np
from balance_lab.firmware.simulation import VehicleSimulation
from balance_lab.firmware.operator_console import OperatorInput, OperatorRenderer, telemetry


def main():
    output = ROOT / "out/native/rmuc_access_20261015"
    output.mkdir(parents=True, exist_ok=True)
    sim = VehicleSimulation("rmuc2025", interface_mapping="coherent_candidate",
                            yaw_damping_nms_rad=0, controller_profile="jump46")
    renderer = None
    try:
        # Set the trial's initial condition only. All later motion is driven
        # through real remote-control channels and physical contacts.
        sim.spawn_xy, sim.z_offset = (-2.3, -6.5), .2029995
        sim.reset("encoder")
        quat = np.array([np.sqrt(.5), 0, 0, np.sqrt(.5)])
        original = sim.data.qpos[3:7].copy()
        mujoco.mju_mulQuat(sim.data.qpos[3:7], quat, original)
        mujoco.mj_forward(sim.model, sim.data)
        state = OperatorInput(paused=False)
        wheel_bodies = [sim._id(mujoco.mjtObj.mjOBJ_BODY, n) for n in ("l_wheel", "r_wheel")]
        radius = sim.runtime.parameters["wheel_radius_m"]
        trace, recording, phase = [], [], "approach"
        up_at = down_at = None
        began = time.monotonic()
        for tick in range(20000):
            elapsed = sim.ticks*sim.tick_s
            bottom = sim.data.xpos[wheel_bodies, 2]-radius
            position = sim.data.xpos[sim.base]
            if up_at is None and elapsed > 5 and position[1] > -4.6 and min(bottom) > .29:
                up_at, phase = elapsed, "hold_top"
            if up_at is not None and down_at is None and elapsed > up_at+1.5:
                phase = "descend"
            if (down_at is None and phase == "descend" and position[1] < -6.05 and max(bottom) < .215):
                down_at, phase = elapsed, "hold_bottom"
            if down_at is not None:
                phase = "hold_bottom"
            state.keys = ({"w"} if elapsed >= 5 and up_at is None else
                          {"s", "control"} if phase == "descend" else set())
            sim.step(state.command(elapsed))
            if tick % 20 == 19:
                row = dict(telemetry(sim), phase=phase,
                           position_m=sim.data.xpos[sim.base].tolist(),
                           wheel_bottom_m=(sim.data.xpos[wheel_bodies, 2]-radius).tolist())
                trace.append(row)
                if elapsed >= 4.5:
                    recording.append((sim.data.qpos.copy(), sim.data.qvel.copy(), row))
            if down_at is not None and elapsed > down_at+2:
                break
        physics_wall = time.monotonic()-began
        report = dict(up_reached_s=up_at, down_reached_s=down_at,
                      physics_wall_s=physics_wall, simulated_s=sim.ticks*sim.tick_s,
                      trace_50hz=trace, final=telemetry(sim))
        assert up_at is not None, "did not physically reach the raised surface"
        assert down_at is not None, "did not physically return down the ramp"
        assert report["final"]["recovery_fault"] == report["final"]["jump_fault"] == 0
        travel = [r for r in trace if r["time_s"] >= 5]
        assert max(abs(r["pitch_deg"]) for r in travel) < 15
        assert max(abs(r["roll_deg"]) for r in travel) < 10
        source = ROOT / "out/native/jump_moving_20261015/original_source_before.json"
        before = json.loads(source.read_text())
        original_root = Path("/home/kaiser/robomaster/code_914/HNU_Embedded_Framework")
        changed = [n for n, h in before.items() if hashlib.sha256((original_root/n).read_bytes()).hexdigest() != h]
        assert not changed, changed
        report["original_source_files_unchanged"] = len(before)
        renderer = OperatorRenderer(sim)
        renderer.external_snapshot = True
        state.keys = set()
        state.camera_distance = 2.7
        video = output / "central_ramp_up_down.mp4"
        with imageio.get_writer(video, fps=25, codec="libx264", pixelformat="yuv420p",
                                macro_block_size=2, quality=7) as writer:
            for qpos, qvel, row in recording[::2]:
                renderer.sim.data.qpos[:] = qpos
                renderer.sim.data.qvel[:] = qvel
                renderer.sim.data.time = row["time_s"]
                renderer.sim.ticks = round(row["time_s"]/sim.tick_s)
                frame = renderer.render(state, row)
                writer.append_data(np.asarray(frame))
                if row["phase"] == "hold_top":
                    frame.save(output/"central_ramp_top.png")
        report["gpu"] = renderer.gpu
        report["video"] = str(video)
        (output/"ramp_acceptance.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
        print("PASS", json.dumps({k: v for k, v in report.items() if k != "trace_50hz"}), flush=True)
    finally:
        if renderer:
            renderer.close()
        sim.close()


if __name__ == "__main__":
    main()
