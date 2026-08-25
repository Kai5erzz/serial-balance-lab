"""Restore the ORIGINAL model structure in whole_body_lqr.xml and
whole_body_lqr_fast.xml (removed by re-running the build pipeline):

- l_leg/r_leg passive joint ranges back to the real mechanical stops
- hip motor torque limits back to +/-40 Nm
- recovery_skid ground-contact geom back under base_link

Inertias stay as repaired (CAD/CSV = original values; wheels = MATLAB LQR).
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

MODEL_DIR = Path(r"D:\User\balance_infantry\mujoco_lqr_whole_body\model")
FILES = [MODEL_DIR / "whole_body_lqr.xml", MODEL_DIR / "whole_body_lqr_fast.xml"]

L_LEG_RANGE = "-0.402691455 0.749471192"
R_LEG_RANGE = "-0.748581838 0.401377801"
SKID = ET.Element(
    "geom",
    {
        "name": "recovery_skid",
        "type": "box",
        "pos": "0.0166 -0.1530 0.0165",
        "xyaxes": "-0.707365 0 -0.706848 -0.706848 0 0.707365",
        "size": "0.22 0.12 0.005",
        "density": "0",
        "contype": "1",
        "conaffinity": "4",
        "condim": "3",
        "friction": "0.8 .02 .001",
        "solref": ".008 1",
        "solimp": ".92 .99 .002",
        "rgba": "0 0 0 0",
    },
)

for path in FILES:
    tree = ET.parse(path)
    root = tree.getroot()

    legs = 0
    for joint in root.findall(".//joint"):
        if joint.get("name") == "l_leg":
            joint.set("range", L_LEG_RANGE)
            legs += 1
        elif joint.get("name") == "r_leg":
            joint.set("range", R_LEG_RANGE)
            legs += 1

    motors = 0
    for motor in root.findall(".//motor"):
        if motor.get("name") in ("l_j1_motor", "l_j2_motor", "r_j1_motor", "r_j2_motor"):
            motor.set("ctrlrange", "-40 40")
            motors += 1

    base = next(
        (body for body in root.findall(".//body") if body.get("name") == "base_link"),
        None,
    )
    skid_added = False
    if base is not None:
        existing = base.find("geom[@name='recovery_skid']")
        if existing is None:
            base.append(SKID)
            skid_added = True

    tree.write(path, encoding="utf-8", xml_declaration=True)
    print(f"{path.name}: legs={legs} motors={motors} skid_added={skid_added}")

# verify
import mujoco
for path in FILES:
    model = mujoco.MjModel.from_xml_path(str(path))
    names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i)
        for i in range(1, model.nbody)
    }
    assert names is not None
    assert model.actuator_ctrlrange is not None
    print(f"{path.name}: loads OK, total mass={model.body_mass.sum():.3f} kg")
