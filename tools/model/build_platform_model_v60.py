"""Build a platform-terrain variant with stepped (worn) stair nosings.

A sharp 90-degree tier edge ejects the wheel at jump-mount touchdown
speeds (see artifacts/climb_matrix/PROGRESS.md).  Real concrete stairs
present a worn/chamfered nosing: the edge is replaced by a near-45
degree ramp over c millimetres, and a rolling wheel climbs that ramp
instead of catching the corner.

Box geometry cannot subtract the corner triangle, so this builder
approximates the 45 degree chamfer with an outward staircase of N small
steps (2 cm each for c=60 mm, N=3).  A wheel rolling at 1.3 m/s pops a
2 cm step trivially (0.014 m centre rise, ~1.8 J vs ~11 J of kinetic
energy), so the staircase acts as the worn edge it models.

Run:  python tools/model/build_platform_model_v60.py
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
OUT = MODEL_DIR / "whole_body_lqr_fast_platform_v60.xml"

C = 0.060   # nosing relief, m
N = 3       # steps in the staircase chamfer
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
        "pos": f"0 0 {(z_lo + z_hi) / 2:.4f}",
        "quat": "0.382683 0 0 0.923880",
        "size": f"{half:.4f} {half:.4f} {(z_hi - z_lo) / 2:.4f}",
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
        # main body below the nosing band (full width)
        wb.append(box(f"{name}_riser", half, z_lo, z_hi - C,
                      "0.55 0.62 0.45 1"))
        # staircase chamfer, bottom-up: each band recedes by C/N so the
        # top band (under the tread face) is recessed by C(N-1)/N and the
        # original sharp corner no longer exists
        for k in range(N):
            zb = z_hi - C + k * (C / N)
            zt = zb + C / N
            radial = half - k * (C / N)
            wb.append(box(f"{name}_nose{k}", radial, zb, zt,
                          "0.53 0.60 0.43 1"))
        # tread surface
        wb.append(box(f"{name}_tread", half - C, z_hi - C, z_hi,
                      "0.52 0.59 0.42 1"))

    tree.write(OUT, encoding="utf-8", xml_declaration=True)
    import mujoco
    m = mujoco.MjModel.from_xml_path(str(OUT))
    print(f"wrote {OUT}: ngeom={m.ngeom}")


if __name__ == "__main__":
    main()
