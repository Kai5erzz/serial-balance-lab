"""Bounded real Tk/GPU/physics throughput and reset check in the arena."""
import argparse
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from balance_lab.firmware.simulation import VehicleSimulation
from balance_lab.firmware.operator_console import OperatorRenderer, OperatorWindow, telemetry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=("rmuc2025", "rmuc2026", "rmuc2027"), default="rmuc2025")
    parser.add_argument("--output", type=Path, default=ROOT / "out/native/rmuc_performance_20261015")
    args = parser.parse_args()
    output = args.output
    output.mkdir(parents=True, exist_ok=True)
    sim = VehicleSimulation(args.variant, interface_mapping="coherent_candidate",
                            yaw_damping_nms_rad=0, controller_profile="jump46")
    spawn_xy = list(sim.spawn_xy)
    renderer = OperatorRenderer(sim)
    window = OperatorWindow(sim, renderer, fps=30)
    window.root.geometry("+20+20")
    # This is a fixed zero-command throughput measurement. Do not steal the
    # user's typing or turn a terminal keypress into a jump in the benchmark.
    window.root.unbind("<KeyPress>")
    window.root.unbind("<KeyRelease>")
    # A controlled benchmark must continue if the user switches back to their
    # own window; the interactive operator retains its normal focus pause.
    window.root.unbind("<FocusOut>")
    window.state.release()
    window.root.update()
    rows, began, tick_start = [], time.monotonic(), 0
    real_render = renderer.render
    def render(*args, **kwargs):
        frame = real_render(*args, **kwargs)
        rows.append(dict(wall=time.monotonic(), ticks=renderer.sim.ticks,
                         keys=sorted(window.state.keys), spin=window.state.spin,
                         jump_until=window.state.jump_until,
                         position_m=renderer.sim.data.xpos[sim.base].tolist(),
                         **renderer.last_timings))
        return frame
    renderer.render = render
    result = {}
    def measure():
        nonlocal began, tick_start
        began, tick_start = time.monotonic(), sim.ticks
        rows.clear()
        window.root.after(8000, finish)
    def finish():
        wall = time.monotonic()-began
        window.state.paused = True
        with window.worker.lock:
            info = telemetry(sim)
            window.worker.snapshot()
            elapsed = (sim.ticks-tick_start)*sim.tick_s
        window.frame_image.save(output/"interactive_30fps.png")
        result.update(wall_seconds=wall, simulation_seconds=elapsed,
                      realtime_factor=elapsed/wall, frames=len(rows), fps=len(rows)/wall,
                      gpu=renderer.gpu, final=info, frames_timing=rows,
                      worker_error=str(window.worker.error) if window.worker.error else None)
        window.after_action("reset")
        with window.worker.lock:
            assert sim.ticks == 0
            assert list(sim.spawn_xy) == spawn_xy
        result["pause_reset_passed"] = True
        window.close()
    try:
        window.state.paused = False
        window.root.after(1500, measure)
        window.run()
        (output/"window_performance.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n")
        print(json.dumps({k: v for k, v in result.items() if k != "frames_timing"}), flush=True)
        assert result["worker_error"] is None
        assert result["realtime_factor"] > .97
        assert result["fps"] > 24
        assert result["final"]["recovery_fault"] == 0
        assert result["final"]["jump_fault"] == 0
    finally:
        if not window.closed:
            window.close()
        renderer.close()
        sim.close()


if __name__ == "__main__":
    main()
