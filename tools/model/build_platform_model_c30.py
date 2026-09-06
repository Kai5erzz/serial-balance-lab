"""Build a platform-terrain variant with worn (filleted) stair nosings.

The sharp 90-degree tier edges of whole_body_lqr_fast_platform.xml are an
idealization no real stair has: concrete nosings are chamfered/worn to a
10-30 mm radius, and a pneumatic tire rolls over such a fillet instead of
catching it.  This builder adds a 30 mm fillet cylinder along every tier
top edge (3 tiers x 4 sides), buried half inside the stair solid (static
geoms of the same body never collide with each other, so the buried half
is inert).

Run:  python tools/model/build_platform_model_c30.py
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
OUT = MODEL_DIR / "whole_body_lqr_fast_platform_c30.xml"

LAYERS = [
    ("platform_top", 1.0, 0.50, 0.6),
    ("platform_step1", 1.3, 0.30, 0.4),
    ("platform_step2", 1.6, 0.10, 0.2),
    ("platform_step3", 1.9, -0.10, 0.0),
]
FILLET_R = 0.030
SQ = 0.70710678


def edge_quat(nx: float, ny: float) -> str:
    """Quaternion rotating the cylinder z-axis onto the edge tangent."""
    # tangent t = (-ny, nx, 0); rotate (0,0,1) -> t by 90 deg about
    # axis = (0,0,1) x t = (-nx, -ny, 0)
    import numpy as np
    axis = np.array([-nx, -ny, 0.0])
    axis = axis / np.linalg.norm(axis)
    c, s = np.cos(np.pi / 4), np.sin(np.pi / 4)
    w, x, y, z = c, *(axis * s)
    return f"{w:.6f} {x:.6f} {y:.6f} {z:.6f}"


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

    for name, half, z, top in LAYERS:
        wb.append(ET.Element("geom", {
            "name": name, "type": "box", "pos": f"0 0 {z}",
            "quat": "0.382683 0 0 0.923880", "size": f"{half} {half} 0.1",
            "density": "0", "contype": "4", "conaffinity": "3",
            "condim": "3", "friction": "1.2 .02 .001",
            "solref": ".008 1", "solimp": ".92 .99 .002",
            "rgba": "0.55 0.62 0.45 1"}))
        # 30 mm fillet along each of the 4 top edges
        for nx, ny in ((SQ, SQ), (SQ, -SQ), (-SQ, SQ), (-SQ, -SQ)):
            cx, cy = nx * half, ny * half
            wb.append(ET.Element("geom", {
                "name": f"{name}_fillet_{nx:.2f}_{ny:.2f}",
                "type": "cylinder",
                "pos": f"{cx:.6f} {cy:.6f} {top:.3f}",
                "quat": edge_quat(nx, ny),
                "size": f"{FILLET_R} {half:.3f}",
                "density": "0", "contype": "4", "conaffinity": "3",
                "condim": "3", "friction": "1.2 .02 .001",
                "solref": ".008 1", "solimp": ".92 .99 .002",
                "rgba": "0.50 0.57 0.42 1"}))

    tree.write(OUT, encoding="utf-8", xml_declaration=True)
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(OUT))
    print(f"wrote {OUT}: ngeom={m.ngeom}")


if __name__ == "__main__":
    main()
