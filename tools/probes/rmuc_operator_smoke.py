"""Bounded GPU/physical check of the operator in the existing RMUC arena."""
import argparse
import hashlib
import json
from pathlib import Path
import resource
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
    parser.add_argument("--variant", choices=("rmuc2025", "rmuc2026", "rmuc2027"), default="rmuc2025")
    parser.add_argument("--output", type=Path, default=ROOT / "out/native/rmuc_operator_20261015")
    args = parser.parse_args()
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    began = time.monotonic()
    from tools.sync_firmware import inventory
    original = ROOT.parent / "code_914/HNU_Embedded_Framework"
    source_before = inventory(original) if original.is_dir() else None
    sim = VehicleSimulation(args.variant, interface_mapping="coherent_candidate",
                            yaw_damping_nms_rad=0, controller_profile="jump46")
    renderer = None
    try:
        assert sim.model.nu == 7
        assert mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_GEOM, "ground") == -1
        initial = sim.data.qpos.copy()
        print("Robot/arena loaded", sim.model.ngeom, "geoms", flush=True)
        state = OperatorInput(paused=False)
        rows = []
        for tick in range(8000):
            if tick == 5500:
                start = sim.data.xpos[sim.base].copy()
                muzzle = sim.data.xmat[sim.gimbal].reshape(3, 3)[:, 0].copy()
            state.keys = {"w", "control"} if 5500 <= tick < 7000 else set()
            sim.step(state.command(sim.ticks * sim.tick_s))
            if tick % 1000 == 999:
                row = telemetry(sim)
                row["position_m"] = sim.data.xpos[sim.base].tolist()
                rows.append(row)
                print(json.dumps(row), flush=True)
            assert np.isfinite(sim.data.qpos).all()
        displacement = float(np.dot(sim.data.xpos[sim.base] - start, muzzle))
        assert displacement > .1, displacement
        assert sim.output.stand_state == 2, telemetry(sim)
        assert sim.output.recovery_fault == 0
        assert abs(telemetry(sim)["pitch_deg"]) < 15
        contact_names = sorted({mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, int(g))
                                for c in sim.data.contact for g in c.geom
                                if sim.model.geom_bodyid[g] == 0})
        assert any(n.startswith(args.variant + "_contact_") for n in contact_names), contact_names
        renderer = OperatorRenderer(sim)
        frame_times = []
        visibility = {}
        for camera in ("third", "first", "overview"):
            state.camera = camera
            state.paused = True
            for _ in range(3):
                before = time.monotonic()
                frame = renderer.render(state, telemetry(sim))
                frame_times.append(time.monotonic() - before)
            frame.save(output / (camera + ".png"))
            renderer.cull_tiles = False
            unculled = renderer.render(state, telemetry(sim))
            renderer.cull_tiles = True
            difference = np.abs(np.asarray(frame, dtype=np.int16)-np.asarray(unculled, dtype=np.int16))
            changed = float(np.mean(np.max(difference, axis=2) > 8))
            visibility[camera] = changed
            assert changed < .005, (camera, changed)
        sim.reset("encoder")
        assert np.array_equal(sim.data.qpos, initial), "reset must return to the same map spawn"
        if source_before is not None:
            assert inventory(original) == source_before, "vehicle source changed during probe"
        report = dict(gpu=renderer.gpu, metadata=sim.metadata(), trace_1hz=rows,
                      forward_displacement_m=displacement, terrain_contacts=contact_names,
                      reset_returns_to_spawn=True, original_source_unchanged=inventory(original) == source_before if source_before is not None else None,
                      culling_changed_pixel_fraction=visibility,
                      render_seconds_median=float(np.median(frame_times)),
                      peak_rss_mib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024,
                      wall_seconds=time.monotonic() - began)
        (output / "report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print("PASS", json.dumps({k: report[k] for k in ("gpu", "forward_displacement_m",
              "render_seconds_median", "peak_rss_mib", "wall_seconds")}), flush=True)
    finally:
        if renderer:
            renderer.close()
        sim.close()


if __name__ == "__main__":
    main()
