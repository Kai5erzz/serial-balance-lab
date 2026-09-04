"""Restore every non-wheel URDF inertia from the preserved SolidWorks CSV."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path
import xml.etree.ElementTree as ET

from extract_log_inertia import INERTIA_FIELDS, load_export_inertias


SOURCE_URDF = Path(
    r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\urdf\whole_body.urdf"
)
WHEEL_LINKS = {"l_wheel", "r_wheel"}
WHEEL_MASS = 0.577
WHEEL_URDF_INERTIA = {
    "ixx": 0.0009705,
    "ixy": 0.0,
    "ixz": 0.0,
    "iyy": 0.0009705,
    "iyz": 0.0,
    "izz": 0.001941,
}


def _number(value: float) -> str:
    return f"{value:.15g}"


def restore_urdf(path: Path = SOURCE_URDF) -> None:
    cad = load_export_inertias()
    tree = ET.parse(path)
    root = tree.getroot()
    links = {link.get("name"): link for link in root.findall("link")}
    missing = sorted(set(cad) - set(links))
    extra = sorted(set(links) - set(cad))
    if missing or extra:
        raise ValueError(f"URDF/CSV link mismatch: missing={missing}, extra={extra}")

    restored = 0
    wheels = 0
    for name, link in links.items():
        inertial = link.find("inertial")
        if inertial is None:
            raise ValueError(f"Missing inertial block for {name}")
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass is None or inertia is None:
            raise ValueError(f"Incomplete inertial block for {name}")
        if name in WHEEL_LINKS:
            mass.set("value", _number(WHEEL_MASS))
            values = WHEEL_URDF_INERTIA
            wheels += 1
        else:
            cad_mass = cad[name]["mass"]
            current_mass = float(mass.get("value", "nan"))
            if abs(current_mass - cad_mass) > max(1e-9, 1e-8 * cad_mass):
                raise ValueError(
                    f"Refusing to alter inertia with unexpected mass for {name}: "
                    f"URDF={current_mass}, CSV={cad_mass}"
                )
            values = cad[name]
            restored += 1
        for field in INERTIA_FIELDS:
            inertia.set(field, _number(values[field]))

    if restored != len(cad) - len(WHEEL_LINKS) or wheels != len(WHEEL_LINKS):
        raise ValueError(f"Unexpected repair count: restored={restored}, wheels={wheels}")
    ET.indent(tree, space="  ")
    tree.write(path, encoding="utf-8", xml_declaration=True)
    print(
        f"Restored {restored} CAD inertias and retained {wheels} wheel overrides "
        f"in {path}"
    )


if __name__ == "__main__":
    restore_urdf()
