"""Parse the SW2URDF serialized Link tree from export.log properly.

The first <Link ...> line contains the complete robot tree (root Link with
nested Links). Extract name/origin/mass/inertia for every link.
"""
from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

LOG = Path(r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\export.log")


def local(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def attr_values(element: ET.Element) -> dict[str, str]:
    result: dict[str, str] = {}
    for child in element:
        if local(child.tag) != "Attributes":
            continue
        for attribute in child:
            kind = next(
                (item.text for item in attribute if local(item.tag) == "AttributeType"),
                None,
            )
            value = next(
                (item.text for item in attribute if local(item.tag) == "Value"),
                None,
            )
            if kind is not None and value is not None:
                result[kind] = value
    return result


def doubles(element: ET.Element) -> list[float]:
    return [
        float(node.text)
        for node in element.iter()
        if local(node.tag) == "double" and node.text is not None
    ]


def walk_links(element: ET.Element) -> list[ET.Element]:
    links = []
    for node in element.iter():
        if local(node.tag) == "Link":
            links.append(node)
    return links


def kind_of(element: ET.Element) -> str | None:
    return next(
        (value for key, value in element.attrib.items() if local(key) == "type"),
        None,
    )


def direct_urdf_children(element: ET.Element) -> list[ET.Element]:
    result = []
    for child in element:
        if local(child.tag) != "ChildElements":
            continue
        result.extend(item for item in child if local(item.tag) == "URDFElement")
    return result


def parse(path: Path) -> dict[str, dict]:
    serialized = next(
        (
            line.strip()
            for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
            if line.lstrip().startswith("<Link ")
        ),
        None,
    )
    root = ET.fromstring(serialized)
    result: dict[str, dict] = {}
    for link in walk_links(root):
        values = attr_values(link)
        name = values.get("name")
        if not name or name in result:
            continue
        inertial = next(
            (item for item in direct_urdf_children(link) if kind_of(item) == "Inertial"),
            None,
        )
        if inertial is None:
            result[name] = {"note": "no inertial"}
            continue
        origin = next(
            (item for item in direct_urdf_children(inertial) if kind_of(item) == "Origin"),
            None,
        )
        mass = next(
            (item for item in direct_urdf_children(inertial) if kind_of(item) == "Mass"),
            None,
        )
        inertia = next(
            (item for item in direct_urdf_children(inertial) if kind_of(item) == "Inertia"),
            None,
        )
        entry: dict = {}
        if origin is not None:
            entry["xyz"], entry["rpy"] = doubles(origin)[:3], doubles(origin)[3:6]
        if mass is not None:
            entry["mass"] = float(attr_values(mass)["value"])
        if inertia is not None:
            values = attr_values(inertia)
            fields = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
            entry.update({f: float(values[f]) for f in fields})
        result[name] = entry
    return result


def main() -> None:
    data = parse(LOG)
    print(f"links: {len(data)}")
    for name in sorted(data):
        entry = data[name]
        if "mass" not in entry:
            print(f"{name}: {entry}")
            continue
        print(
            f"{name:16s} mass={entry['mass']:.9g} xyz={entry.get('xyz')} "
            f"ixx={entry.get('ixx'):.9g} ixy={entry.get('ixy'):.9g} "
            f"ixz={entry.get('ixz'):.9g} iyy={entry.get('iyy'):.9g} "
            f"iyz={entry.get('iyz'):.9g} izz={entry.get('izz'):.9g}"
        )


if __name__ == "__main__":
    main()
