"""Compose this field with an external robot MJCF without editing either source."""
from pathlib import Path
import argparse
import copy
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent


def compose(robot, output, *, pack=ROOT/"pack", profile="full", spawn=(-8., -4., .6), perimeter=False):
    robot, pack, output = Path(robot).resolve(), Path(pack).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    root = ET.parse(robot).getroot()
    # The fast chassis AABB and flat-scene support overhang can hit ramps
    # ahead of the wheels. Use the already available CAD chassis contacts.
    if root.find("worldbody/body[@name='base_link']/geom[@name='fast_collision_base_link']") is not None:
        from terrain_rmuc2025.robot_contact import refine_chassis_contacts
        refine_chassis_contacts(root)
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", "") if compiler is not None else ""
    texturedir = compiler.get("texturedir", "") if compiler is not None else ""
    for asset in root.findall("asset/*"):
        if "file" in asset.attrib:
            prefix = meshdir if asset.tag == "mesh" else texturedir if asset.tag == "texture" else ""
            asset.set("file", str((robot.parent/prefix/asset.get("file")).resolve()))
    if compiler is not None:
        compiler.attrib.pop("meshdir", None)
        compiler.attrib.pop("texturedir", None)
    world = root.find("worldbody")
    if world is None:
        raise ValueError("robot has no worldbody")
    # RL-lab's existing flat floor is the only map element removed. Other
    # world geoms are preserved so unrelated robot environments fail visibly.
    removed = []
    for geom in list(world.findall("geom")):
        if geom.get("name") in ("ground", "ground_outer"):
            removed.append(geom.get("name")); world.remove(geom)
    if profile == "interactive":
        from terrain_rmuc2025.interactive_visual import interactive_field
        field_path = interactive_field(pack)
    elif profile in ("full", "collision"):
        field_path = pack/("field.xml" if profile == "full" else "field_collision.xml")
    else:
        raise ValueError(f"unknown field profile {profile!r}")
    field = ET.parse(field_path).getroot()
    assets = root.find("asset")
    if assets is None:
        assets = ET.SubElement(root, "asset")
    for asset in field.find("asset"):
        item = copy.deepcopy(asset)
        if "file" in item.attrib:
            item.set("file", str(pack/item.get("file")))
        assets.append(item)
    for geom in field.find("worldbody"):
        world.append(copy.deepcopy(geom))
    if perimeter:
        for geom in ET.parse(pack/"field_perimeter.xml").getroot().find("worldbody"):
            world.append(copy.deepcopy(geom))
    body = world.find("body[@name='base_link']")
    if body is None:
        body = next((b for b in world.findall("body") if b.find("freejoint") is not None), None)
    if body is None:
        raise ValueError("robot needs one top-level free body for the spawn position")
    body.set("pos", " ".join(str(float(x)) for x in spawn))
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(root)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    return {"robot": str(robot), "scene": str(output), "profile": profile,
            "spawn_m": list(spawn), "removed_ground": removed, "optional_perimeter": perimeter}


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--robot", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--pack", type=Path, default=ROOT/"pack")
    p.add_argument("--profile", choices=("full", "collision", "interactive"), default="full")
    p.add_argument("--spawn", type=float, nargs=3, default=(-8, -4, .6))
    p.add_argument("--perimeter", action="store_true", help="add simplified rules-based outer walls")
    a = p.parse_args()
    print(compose(a.robot, a.output, pack=a.pack, profile=a.profile, spawn=a.spawn, perimeter=a.perimeter))
