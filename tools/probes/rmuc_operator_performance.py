"""Measure real operator costs before changing physics or display quality."""
import cProfile
import io
import json
from pathlib import Path
import pstats
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

import numpy as np
from balance_lab.firmware.simulation import VehicleSimulation
from balance_lab.firmware.operator_console import OperatorInput, OperatorRenderer, telemetry


def main():
    output = ROOT / "out/native/rmuc_performance_20261015"
    output.mkdir(parents=True, exist_ok=True)
    sim = VehicleSimulation("rmuc2025", interface_mapping="coherent_candidate",
                            yaw_damping_nms_rad=0, controller_profile="jump46")
    state = OperatorInput(paused=False)
    report = {}
    try:
        profile = cProfile.Profile()
        profile.enable()
        began = time.perf_counter()
        for _ in range(5000):
            sim.step(state.command(sim.ticks*sim.tick_s))
        wall = time.perf_counter()-began
        profile.disable()
        report["profiled_physics_rtf"] = 5/wall
        stream = io.StringIO()
        pstats.Stats(profile, stream=stream).sort_stats("cumtime").print_stats(25)
        (output/"physics_profile.txt").write_text(stream.getvalue())
        for width in (960, 800):
            renderer = OperatorRenderer(sim, width, width*9//16)
            costs = []
            try:
                for frame in range(60):
                    state.begin_frame(.05)
                    began = time.perf_counter()
                    for _ in range(50):
                        sim.step(state.command(sim.ticks*sim.tick_s))
                    physical = (time.perf_counter()-began)*1000
                    image = renderer.render(state, telemetry(sim))
                    if frame >= 5:
                        costs.append(dict(physics_ms=physical, **renderer.last_timings))
                    if frame == 59:
                        image.save(output/f"baseline_{width}.png")
                medians = {key: float(np.median([row[key] for row in costs])) for key in costs[0]}
                report[str(width)] = dict(median=medians, total_ms=sum(medians.values()),
                                         rtf_without_Tk=50/sum(medians.values()))
                print(width, json.dumps(report[str(width)]), flush=True)
            finally:
                renderer.close()
        (output/"baseline.json").write_text(json.dumps(report, indent=2)+"\n")
    finally:
        sim.close()


if __name__ == "__main__":
    main()
