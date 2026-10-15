"""Compose this field with an external robot MJCF without editing either source."""
from pathlib import Path
import argparse
import copy
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parent


def compose(robot, output, *, pack=ROOT/"pack", profile="full", spawn=(-8., -4., .6), perimeter=True):
    robot, pack, output = Path(robot).resolve(), Path(pack).resolve(), Path(output).resolve()
    if output.exists():
        raise FileExistsError(output)
    root = ET.parse(robot).getroot()
    # Match the arena-tested robot contact geometry; the flat-scene chassis
    # AABB otherwise reaches a slope before the real CAD chassis does.
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
    if profile in ("full", "collision"):
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
            item = copy.deepcopy(geom)
            # Keep operator bounds in the tiled display model as well as the
            # physical model. Map contact proxies use the rmuc2026_ prefix
            # and are deliberately stripped out of that display copy.
            name = item.get("name").removeprefix("rmuc2026_perimeter_")
            item.set("name", "operator_boundary_" + name)
            item.set("group", "4")
            item.set("rgba", ".12 .65 .85 .22")
            position = [float(x) for x in item.get("pos").split()]
            size = [float(x) for x in item.get("size").split()]
            # The CAD floor is not globally horizontal. Extend the wall below
            # zero so there is no gap at its negative-height perimeter.
            position[2], size[2] = 1.0, 1.4  # bottom -0.4 m, top 2.4 m
            item.set("pos", " ".join(map(str, position)))
            item.set("size", " ".join(map(str, size)))
            world.append(item)
            rail = copy.deepcopy(item)
            rail.set("name", "operator_boundary_rail_" + name)
            rail.set("contype", "0")
            rail.set("conaffinity", "0")
            rail.set("rgba", ".12 .8 .95 1")
            position[2], size[2] = 2.38, .02
            rail.set("pos", " ".join(map(str, position)))
            rail.set("size", " ".join(map(str, size)))
            world.append(rail)
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
    p.add_argument("--profile", choices=("full", "collision"), default="full")
    p.add_argument("--spawn", type=float, nargs=3, default=(-8, -4, .6))
    p.add_argument("--perimeter", action=argparse.BooleanOptionalAction, default=True,
                   help="visible solid outer bounds; use --no-perimeter for the unbounded CAD composition")
    a = p.parse_args()
    print(compose(a.robot, a.output, pack=a.pack, profile=a.profile, spawn=a.spawn, perimeter=a.perimeter))
