"""Record operator direction and world-yaw hold using the original C runtime.

Run with mujoco-gpu. The console's explicit, uncalibrated zero yaw-bearing
damping assumption is recorded, never substituted into the identified leg fit.
"""
from pathlib import Path
import argparse
import json
import resource
import sys

import imageio.v2 as imageio
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from balance_lab.firmware.operator_console import OperatorInput, OperatorRenderer, telemetry
from balance_lab.firmware.recording import Recorder, replay
from balance_lab.firmware.simulation import VehicleSimulation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    sim = VehicleSimulation(interface_mapping="coherent_candidate", yaw_damping_nms_rad=0)
    renderer = OperatorRenderer(sim)
    recorder = Recorder(args.output/"commands.jsonl", sim.metadata(), pages=(39,49,50,54))
    state = OperatorInput(paused=False)
    headings = []
    rows = []
    try:
        with imageio.get_writer(args.output/"operator.mp4", fps=20, codec="libx264",
                pixelformat="yuv420p", macro_block_size=2,
                ffmpeg_params=["-crf","22","-preset","fast","-threads","2","-movflags","+faststart"]) as writer:
            for index in range(300):
                t = sim.ticks*sim.tick_s
                state.keys = {"w","control"} if 5<=t<6.5 else {"s","control"} if 6.5<=t<9 else set()
                state.spin = t>=10
                state.camera = "first" if t>=11 else "third"
                state.begin_frame(.05)
                for _ in range(50):
                    command = state.command(sim.ticks*sim.tick_s)
                    sim.step(command)
                    recorder.tick(sim,command)
                    if sim.ticks>=10000:
                        gun=sim.data.xmat[sim.gimbal].reshape(3,3)[:,0]
                        base=sim.data.xmat[sim.base].reshape(3,3)@sim.sensor_body[:,1]
                        headings.append([np.arctan2(gun[1],gun[0]),np.arctan2(base[1],base[0])])
                info=telemetry(sim)
                frame=renderer.render(state,info)
                writer.append_data(np.asarray(frame))
                if index in (120,260):
                    frame.save(args.output/(state.camera+"_person.png"))
                if index%20==0:
                    rows.append(info)
                    print(f"operator acceptance {index//20+1}/15 s | mode {info['mode']}",flush=True)
        recorder.close()
        angles=np.unwrap(np.array(headings),axis=0)
        report={"metadata":sim.metadata(),"gpu":renderer.gpu,"trace_1hz":rows,
                "spin_seconds":5,"chassis_rotations":float((angles[-1,1]-angles[0,1])/(2*np.pi)),
                "gun_max_drift_deg":float(np.degrees(np.max(abs(angles[:,0]-angles[0,0])))),
                "gun_end_drift_deg":float(np.degrees(angles[-1,0]-angles[0,0])),
                "peak_rss_mb":resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                "replay":replay(args.output/"commands.jsonl")}
        (args.output/"report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2)+"\n")
        assert abs(report["chassis_rotations"])>7,report
        assert report["gun_max_drift_deg"]<4,report
        assert abs(report["gun_end_drift_deg"])<2,report
        print(json.dumps({key:report[key] for key in ("chassis_rotations","gun_max_drift_deg","gun_end_drift_deg","replay","peak_rss_mb")}),flush=True)
    finally:
        if not recorder.file.closed:
            recorder.close()
        renderer.close()
        sim.close()


if __name__=="__main__":
    main()
