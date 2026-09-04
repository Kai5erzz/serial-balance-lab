"""Build a platform-terrain variant of the fast model for descent testing.

Terrain: a fixed 2000x2000 mm square platform, top at 600 mm, surrounded by
three 300 mm wide x 200 mm high steps (0.2/0.4/0.6 m levels), i.e. a
four-level truncated pyramid centered at the world origin.
"""
from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parent / "model"
FAST = MODEL_DIR / "whole_body_lqr_fast.xml"
OUT = MODEL_DIR / "whole_body_lqr_fast_platform.xml"

# Pyramid layers bottom-up, each slab 200 mm thick (MuJoCo box size is the
# half-extent, so the listed z is the slab CENTER).  Faces stack contiguously
# so no solid overlaps another solid:
#   top    : top face 0.6 -> center 0.50   (spans 0.4..0.6)
#   step 1 : top face 0.4 -> center 0.30   (spans 0.2..0.4)
#   step 2 : top face 0.2 -> center 0.10   (spans 0.0..0.2)
#   step 3 : top face 0.0 -> center -0.10  (spans -0.2..0.0)
# The previous table put platform_top at 0.3 and step1 at 0.5: that placed
# the nominal deck at z=0.4 inside step1's own volume, spawned every descent
# run standing on a mislabeled interpenetrating terrain, and made the
# geometric edge probe see phantom ledges everywhere.
LAYERS = [
    ("platform_top", 1.0, 0.50),
    ("platform_step1", 1.3, 0.30),
    ("platform_step2", 1.6, 0.10),
    ("platform_step3", 1.9, -0.10),
]


def main() -> None:
    tree = ET.parse(FAST)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    assert worldbody is not None

    # remove the flat ground plane (the platform replaces it around the base,
    # but the ground returns outside the 1.9 m pyramid footprint so a drive
    # off the platform cannot fall through the world)
    ground = worldbody.find("geom[@name='ground']")
    if ground is not None:
        worldbody.remove(ground)
    ground = ET.Element(
        "geom",
        {
            "name": "ground_outer",
            "type": "plane",
            "pos": "0 0 0",
            "size": "50 50 .1",
            "friction": "1.2 .02 .001",
            "material": "lqr_ground_material",
            "contype": "4",
            "conaffinity": "3",
        },
    )
    worldbody.append(ground)

    for name, half, z in LAYERS:
        geom = ET.Element(
            "geom",
            {
                "name": name,
                "type": "box",
                "pos": f"0 0 {z}",
                # yaw 45 deg so the platform edges run perpendicular to the
                # chassis forward frame ((-0.707, +0.707)): both wheels cross
                # an edge simultaneously, as a driver faces an edge head-on.
                "quat": "0.382683 0 0 0.923880",
                "size": f"{half} {half} 0.1",
                "density": "0",
                "contype": "4",
                "conaffinity": "3",
                "condim": "3",
                "friction": "1.2 .02 .001",
                "solref": ".008 1",
                "solimp": ".92 .99 .002",
                "rgba": "0.55 0.62 0.45 1",
            },
        )
        worldbody.append(geom)

    tree.write(OUT, encoding="utf-8", xml_declaration=True)
    print(f"wrote {OUT}")

    # quick load check
    import mujoco
    model = mujoco.MjModel.from_xml_path(str(OUT))
    print(f"loads OK: ngeom={model.ngeom}, nbody={model.nbody}")


if __name__ == "__main__":
    main()
