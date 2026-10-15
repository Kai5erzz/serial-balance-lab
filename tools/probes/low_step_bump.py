"""Measure the native controller on low, vertical steps with arena robot contacts.

The fixture height is measured from a level floor. No state or feedback is
changed after initial placement; the only commands are the operator's C/W keys.
"""
import argparse
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import mujoco
import numpy as np
from balance_lab.paths import MODEL_FAST
from balance_lab.firmware.simulation import VehicleSimulation
from balance_lab.firmware.operator_console import OperatorInput, telemetry
from balance_lab.sysid.apply import apply_model_parameters, load_profile
from terrain_rmuc2025.robot_contact import refine_chassis_contacts, fit_recovery_support


def fixture(height, path):
    root = ET.parse(MODEL_FAST).getroot()
    refine_chassis_contacts(root)
    for asset in root.findall("asset/*"):
        if "file" in asset.attrib:
            asset.set("file", str((MODEL_FAST.parent / asset.get("file")).resolve()))
    # Four metre top with enough width to put both wheels on the same edge.
    # Original ground, masses, inertias, legs, motors and materials remain.
    ET.SubElement(root.find("worldbody"), "geom", name="low_step_fixture", type="box",
                  pos=f"2 0 {height/2}", size=f"2 2 {height/2}",
                  rgba=".75 .42 .12 1", friction=".9 .005 .0001", group="0")
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    model = mujoco.MjModel.from_xml_path(str(path))
    fit_recovery_support(model)
    applied = apply_model_parameters(model, load_profile("identified_2026-09-21"))
    return model, applied


def trial(height, output, seconds, *, guide=True, record=False, heading=0, drive="normal", hold=2):
    output.mkdir(parents=True, exist_ok=True)
    model, applied = fixture(height, output / "scene.xml")
    if not guide:
        # Diagnostic A/B only: identify load carried by the extra skid proxy.
        # This is never used by the operator scene or an acceptance claim.
        gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "guide_wheel_contact")
        model.geom_contype[gid] = model.geom_conaffinity[gid] = 0
    with patch("balance_lab.firmware.simulation.load_model", return_value=(model, applied)):
        sim = VehicleSimulation(interface_mapping="coherent_candidate",
                                yaw_damping_nms_rad=0, controller_profile="jump46")
    state = OperatorInput(paused=False)
    wheel = [sim._id(mujoco.mjtObj.mjOBJ_BODY, n) for n in ("l_wheel", "r_wheel")]
    radius = sim.runtime.parameters["wheel_radius_m"]
    rows, poses, topped_at = [], [], None
    recorder = None
    began = time.monotonic()
    try:
        sim.spawn_xy = (-1.5, 0)
        sim.reset("encoder")
        if heading:
            rotation = np.array([np.cos(np.radians(heading)/2), 0, 0, np.sin(np.radians(heading)/2)])
            original = sim.data.qpos[3:7].copy()
            mujoco.mju_mulQuat(sim.data.qpos[3:7], rotation, original)
            mujoco.mj_forward(sim.model, sim.data)
        if record:
            from balance_lab.firmware.recording import Recorder
            recorder = Recorder(output / "commands.jsonl", sim.metadata(), pages=(39,54))
        with (output / "trace.jsonl").open("w") as stream:
            for tick in range(round(seconds / sim.tick_s)):
                t = sim.ticks*sim.tick_s
                if tick == 5000:
                    state.press("c", t)
                state.keys = ({"w", "control"} if drive == "slow" else
                              {"w", "shift"} if drive == "full" else {"w"}) if t >= 5.5 and topped_at is None else set()
                command = state.command(t)
                sim.step(command)
                if recorder:
                    recorder.tick(sim, command)
                bottom = sim.data.xpos[wheel, 2]-radius
                if topped_at is None and min(sim.data.xpos[wheel, 0]) > .35 and min(bottom) > height-.008:
                    topped_at = sim.ticks*sim.tick_s
                if tick % 20 == 19:
                    force = np.empty(6)
                    contacts = []
                    for ci, contact in enumerate(sim.data.contact):
                        if 0 not in sim.model.geom_bodyid[contact.geom]:
                            continue
                        mujoco.mj_contactForce(sim.model, sim.data, ci, force)
                        if force[0] > .01:
                            contacts.append(dict(geoms=[mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, int(g)) for g in contact.geom],
                                position_m=contact.pos.tolist(), force_n=float(force[0])))
                    row = dict(telemetry(sim), position_m=sim.data.xpos[sim.base].tolist(),
                        wheel_bottom_m=bottom.tolist(), wheel_position_m=sim.data.xpos[wheel].tolist(),
                        bump_page=sim.runtime.debug_page(54), contact_pairs=contacts)
                    rows.append(row)
                    poses.append((sim.data.qpos.copy(), sim.data.qvel.copy()))
                    stream.write(json.dumps(row)+"\n")
                if topped_at is not None and t > topped_at+hold:
                    break
        np.savez_compressed(output / "trajectory.npz", qpos=np.array([p[0] for p in poses]),
                            qvel=np.array([p[1] for p in poses]))
        changes, previous = [], None
        for row in rows:
            key = (row["bump"], row["bump_page"][3])
            if key != previous:
                changes.append(dict(time_s=row["time_s"], state=key[0], abort=key[1],
                    position_m=row["position_m"], pitch_deg=row["pitch_deg"], roll_deg=row["roll_deg"]))
                previous = key
        if recorder:
            from balance_lab.firmware.recording import replay
            recorder.close()
            replay_result = replay(output / "commands.jsonl")
        else:
            replay_result = None
        report = dict(step_height_m=height, guide_collision_enabled=guide, drive=drive, heading_deg=heading,
                      topped_at_s=topped_at, wall_s=time.monotonic()-began,
                      transitions=changes, final=rows[-1], metadata=sim.metadata(), replay=replay_result)
        (output / "report.json").write_text(json.dumps(report, indent=2)+"\n")
        print(json.dumps({k:v for k,v in report.items() if k not in ("metadata", "final")}), flush=True)
        print("FINAL", json.dumps({k:v for k,v in rows[-1].items() if k not in ("bump_page", "contact_pairs")}), flush=True)
    finally:
        if recorder and not recorder.file.closed:
            recorder.close()
        sim.close()


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--heights", nargs="+", type=float, default=(.05, .08, .10))
    p.add_argument("--seconds", type=float, default=18)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--record", action="store_true")
    p.add_argument("--drive", choices=("normal", "slow", "full"), default="normal")
    p.add_argument("--heading", type=float, default=0)
    p.add_argument("--hold", type=float, default=2)
    args = p.parse_args()
    for height in args.heights:
        if not 0 < height <= .15:
            p.error("this low-step fixture is limited to 0–15 cm")
        trial(height, args.output / f"{round(height*1000):03d}mm", args.seconds,
              record=args.record, drive=args.drive, heading=args.heading, hold=args.hold)
