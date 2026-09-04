"""Rebuild whole_body.urdf and whole_body_native.urdf from the revised export.

The revised SW2URDF export fixes the l_wheel/r_wheel joint axis (was [0 0 1]/
[0 0 -1] for l/r, now mirrored the other way).  We keep the MATLAB wheel
inertia override (mw=0.577, Iw=0.001941) that the project applies on top of
the CAD export, and normalize the mesh package name back to whole_body.
"""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import xml.etree.ElementTree as ET
from pathlib import Path

REVISED = Path(
    r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body_revised\urdf\whole_body_revised.urdf"
)
WHEEL_OVERRIDE = {
    "mass": "0.577",
    "inertia": {
        "ixx": "0.0009705",
        "ixy": "0",
        "ixz": "0",
        "iyy": "0.0009705",
        "iyz": "0",
        "izz": "0.001941",
    },
}
OUTS = [
    Path(r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\urdf\whole_body.urdf"),
    Path(r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\mujoco_whole_body\whole_body_native.urdf"),
]


def main() -> None:
    tree = ET.parse(REVISED)
    root = tree.getroot()
    for link in root.findall("link"):
        name = link.get("name")
        if name not in ("l_wheel", "r_wheel"):
            continue
        inertial = link.find("inertial")
        inertial.find("mass").set("value", WHEEL_OVERRIDE["mass"])
        inertia = inertial.find("inertia")
        for field, value in WHEEL_OVERRIDE["inertia"].items():
            inertia.set(field, value)
        # normalize mesh package references to the canonical export name
        for mesh in link.iter():
            if mesh.tag == "mesh":
                fn = mesh.get("filename", "")
                mesh.set("filename", fn.replace(
                    "whole_body_revised", "whole_body"
                ))
    ET.indent(tree, space="  ")
    for out in OUTS:
        tree.write(out, encoding="utf-8", xml_declaration=True)
        print(f"wrote {out}")

    # verify
    tree2 = ET.parse(OUTS[0])
    for j in tree2.getroot().findall("joint"):
        if j.get("name") in ("l_wheel", "r_wheel"):
            print(j.get("name"), "axis =", j.find("axis").get("xyz"))
    for l in tree2.getroot().findall("link"):
        if l.get("name") in ("l_wheel", "r_wheel"):
            i = l.find("inertial")
            print(l.get("name"), "mass =", i.find("mass").get("value"),
                  "izz =", i.find("inertia").get("izz"))


if __name__ == "__main__":
    main()