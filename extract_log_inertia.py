"""Extract the untouched CAD inertias from the SW2URDF serialized export log."""

from __future__ import annotations

import csv
from pathlib import Path
import xml.etree.ElementTree as ET


LOG = Path(
    r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body_revised\export.log"
)
CAD_CSV = LOG.parent / "urdf" / "whole_body_revised.csv"
INERTIA_FIELDS = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
CSV_INERTIA_FIELDS = {
    "ixx": "Moment Ixx",
    "ixy": "Moment Ixy",
    "ixz": "Moment Ixz",
    "iyy": "Moment Iyy",
    "iyz": "Moment Iyz",
    "izz": "Moment Izz",
}


def _local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def _direct_child(element: ET.Element, name: str) -> ET.Element | None:
    return next(
        (child for child in element if _local_name(child.tag) == name),
        None,
    )


def _serialized_type(element: ET.Element) -> str | None:
    return next(
        (value for key, value in element.attrib.items() if _local_name(key) == "type"),
        None,
    )


def _attribute_values(element: ET.Element) -> dict[str, str]:
    attributes = _direct_child(element, "Attributes")
    if attributes is None:
        return {}
    result: dict[str, str] = {}
    for attribute in attributes:
        kind = _direct_child(attribute, "AttributeType")
        value = _direct_child(attribute, "Value")
        if kind is not None and kind.text and value is not None and value.text:
            result[kind.text] = value.text
    return result


def load_export_inertias(path: Path = CAD_CSV) -> dict[str, dict[str, float]]:
    """Return mass and the full six-component inertia tensor for every link."""
    if path.suffix.lower() == ".csv":
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            rows = csv.DictReader(stream)
            result = {
                row["Link Name"]: {
                    "mass": float(row["Mass"]),
                    **{
                        field: float(row[column])
                        for field, column in CSV_INERTIA_FIELDS.items()
                    },
                }
                for row in rows
            }
        if not result:
            raise ValueError(f"No inertial records decoded from {path}")
        return result

    serialized = next(
        (
            line.strip()
            for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
            if line.lstrip().startswith("<Link ")
        ),
        None,
    )
    if serialized is None:
        raise ValueError(f"No serialized SW2URDF Link tree found in {path}")
    root = ET.fromstring(serialized)
    result: dict[str, dict[str, float]] = {}
    for link in (item for item in root.iter() if _local_name(item.tag) == "Link"):
        name = _attribute_values(link).get("name")
        children = _direct_child(link, "ChildElements")
        if not name or children is None:
            continue
        inertial = next(
            (
                item
                for item in children
                if _local_name(item.tag) == "URDFElement"
                and _serialized_type(item) == "Inertial"
            ),
            None,
        )
        inertial_children = (
            _direct_child(inertial, "ChildElements") if inertial is not None else None
        )
        if inertial_children is None:
            continue
        mass: float | None = None
        tensor: dict[str, float] | None = None
        for item in inertial_children:
            item_type = _serialized_type(item)
            values = _attribute_values(item)
            if item_type == "Mass" and "value" in values:
                mass = float(values["value"])
            elif item_type == "Inertia":
                tensor = {field: float(values[field]) for field in INERTIA_FIELDS}
        if mass is not None and tensor is not None:
            result[name] = {"mass": mass, **tensor}
    if not result:
        raise ValueError(f"No inertial records decoded from {path}")
    return result


def main() -> None:
    results = load_export_inertias()
    for name, values in sorted(results.items()):
        print(
            f"{name:16s} mass={values['mass']:.9g} "
            + " ".join(
                f"{field}={values[field]:.9g}" for field in INERTIA_FIELDS
            )
        )
    print(f"\ntotal links: {len(results)}")


if __name__ == "__main__":
    main()
