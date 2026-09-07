"""Build a single-tier 270 mm platform terrain for moving jump-up tests.

One rotated-45 degree box (face normals along the world diagonals, same
convention as the stair platform) so the diagonal approach machinery and
radial-distance measurements carry over unchanged.  The face the robot
attacks sits at radial distance = half-extent from the origin; the top
surface is at 0.27 m.

Run:  python tools/model/build_platform_270.py
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import xml.etree.ElementTree as ET

MODEL_DIR = REPO_ROOT / "model"
FAST = MODEL_DIR / "whole_body_lqr_fast.xml"
OUT = MODEL_DIR / "whole_body_lqr_fast_p270.xml"

HALF = 1.2      # face at radial 1.2 m on the diagonal
TOP = 0.27      # platform height, m


def main() -> None:
    tree = ET.parse(FAST)
    wb = tree.getroot().find("worldbody")
    ground = wb.find("geom[@name='ground']")
    if ground is not None:
        wb.remove(ground)
    wb.append(ET.Element("geom", {
        "name": "ground_outer", "type": "plane", "pos": "0 0 0",
        "size": "50 50 .1", "friction": "1.2 .02 .001",
        "material": "lqr_ground_material", "contype": "4",
        "conaffinity": "3"}))
    wb.append(ET.Element("geom", {
        "name": "p270_top", "type": "box",
        "pos": f"0 0 {TOP / 2:.4f}",
        "quat": "0.382683 0 0 0.923880",
        "size": f"{HALF} {HALF} {TOP / 2:.4f}",
        "density": "0", "contype": "4", "conaffinity": "3",
        "condim": "3", "friction": "1.2 .02 .001",
        "solref": ".008 1", "solimp": ".92 .99 .002",
        "rgba": "0.55 0.62 0.45 1"}))

    tree.write(OUT, encoding="utf-8", xml_declaration=True)
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(OUT))
    print(f"wrote {OUT}: ngeom={m.ngeom}")


if __name__ == "__main__":
    main()
