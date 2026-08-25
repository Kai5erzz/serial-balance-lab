"""Safely update only l_wheel/r_wheel to the MATLAB LQR-model inertia."""

from pathlib import Path
import xml.etree.ElementTree as ET


MASS = "0.577"
URDF_INERTIA = {
    # The transverse terms are half the requested axial inertia.  Keep the
    # extra half micro-unit so the rounded values still satisfy Ixx+Iyy>=Izz.
    "ixx": "0.0009705",
    "ixy": "0",
    "ixz": "0",
    "iyy": "0.0009705",
    "iyz": "0",
    "izz": "0.001941",
}
MJ_INERTIA = "0.001941 0.0009705 0.0009705"
WHEELS = {"l_wheel", "r_wheel"}

targets = [
    Path(r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\urdf\whole_body.urdf"),
    Path(r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\mujoco_whole_body\whole_body_native.urdf"),
    Path(r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\mujoco_whole_body\whole_body_closed.xml"),
    Path(r"D:\User\balance_infantry\mujoco_lqr_whole_body\model\whole_body_lqr.xml"),
    Path(r"D:\User\balance_infantry\mujoco_lqr_whole_body\model\whole_body_lqr_fast.xml"),
]

for path in targets:
    tree = ET.parse(path)
    root = tree.getroot()
    updated: set[str] = set()
    if path.suffix == ".urdf":
        for link in root.findall("link"):
            name = link.get("name")
            if name not in WHEELS:
                continue
            inertial = link.find("inertial")
            mass = inertial.find("mass") if inertial is not None else None
            inertia = inertial.find("inertia") if inertial is not None else None
            if mass is None or inertia is None:
                raise ValueError(f"Incomplete wheel inertial in {path}: {name}")
            mass.set("value", MASS)
            inertia.attrib.update(URDF_INERTIA)
            updated.add(name)
    else:
        for body in root.findall(".//body"):
            name = body.get("name")
            if name not in WHEELS:
                continue
            inertial = body.find("inertial")
            if inertial is None:
                raise ValueError(f"Missing wheel inertial in {path}: {name}")
            inertial.set("mass", MASS)
            inertial.set("diaginertia", MJ_INERTIA)
            updated.add(name)
    if updated != WHEELS:
        raise ValueError(f"Expected exactly two wheel bodies in {path}, got {updated}")
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    print(f"{path.name}: updated only {', '.join(sorted(updated))}")
