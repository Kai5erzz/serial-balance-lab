"""Build a platform-terrain variant with relieved (V-notched) stair nosings.

The sharp 90-degree tier edge is the single blocker for the jump-mount:
at the required touch-down speed the corner contact ejects the robot
(see artifacts/climb_matrix/PROGRESS.md).  A real stair presents a worn
or chamfered nosing, and a pneumatic tire's flat contact patch wraps the
edge, so the effective corner of the real system sits several cm lower
and further in than this model's ideal edge.

Each tier is rebuilt from two boxes:
  riser box: z from tier bottom to top-c, full half-extent
  tread box: z from top-c to top,    half-extent minus c
The residual V-notch bottom corner sits at (half, top-c) -- the wheel's
clearance requirement at the lip plane drops from top+r to top-c+r, i.e.
c of extra effective carry, and the edge the wheel must roll over is
lowered by the same amount.

c = 50 mm, within the worn-nosing range for concrete stairs.

Run:  python tools/model/build_platform_model_v50.py
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
OUT = MODEL_DIR / "whole_body_lqr_fast_platform_v50.xml"

C = 0.050  # nosing relief, m
# name, half-extent, z bottom, z top (tier tread height)
LAYERS = [
    ("platform_top", 1.0, 0.4, 0.6),
    ("platform_step1", 1.3, 0.2, 0.4),
    ("platform_step2", 1.6, 0.0, 0.2),
    ("platform_step3", 1.9, -0.2, 0.0),
]


def box(name, half, z_lo, z_hi, rgba):
    return ET.Element("geom", {
        "name": name, "type": "box",
        "pos": f"0 0 {(z_lo + z_hi) / 2:.3f}",
        "quat": "0.382683 0 0 0.923880",
        "size": f"{half:.3f} {half:.3f} {(z_hi - z_lo) / 2:.3f}",
        "density": "0", "contype": "4", "conaffinity": "3",
        "condim": "3", "friction": "1.2 .02 .001",
        "solref": ".008 1", "solimp": ".92 .99 .002", "rgba": rgba})


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

    for name, half, z_lo, z_hi in LAYERS:
        wb.append(box(f"{name}_riser", half, z_lo, z_hi - C,
                      "0.55 0.62 0.45 1"))
        wb.append(box(f"{name}_tread", half - C, z_hi - C, z_hi,
                      "0.52 0.59 0.42 1"))

    tree.write(OUT, encoding="utf-8", xml_declaration=True)
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(OUT))
    print(f"wrote {OUT}: ngeom={m.ngeom}")


if __name__ == "__main__":
    main()
