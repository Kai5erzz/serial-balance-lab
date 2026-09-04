"""Compare whole_body_revised.urdf vs current whole_body.urdf:
joints and link inertials (origin/mass/inertia)."""
from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import xml.etree.ElementTree as ET
import numpy as np

CUR = r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\urdf\whole_body.urdf"
NEW = r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body_revised\urdf\whole_body_revised.urdf"


def parse(path):
    tree = ET.parse(path)
    root = tree.getroot()
    links = {}
    joints = {}
    for link in root.findall("link"):
        name = link.get("name")
        inertial = link.find("inertial")
        entry = {}
        if inertial is not None:
            origin = inertial.find("origin")
            mass = inertial.find("mass")
            inertia = inertial.find("inertia")
            if origin is not None:
                entry["xyz"] = [float(v) for v in origin.get("xyz").split()]
                entry["rpy"] = [float(v) for v in origin.get("rpy").split()]
            if mass is not None:
                entry["mass"] = float(mass.get("value"))
            if inertia is not None:
                entry["inertia"] = {
                    f: float(inertia.get(f)) for f in
                    ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
                }
        links[name] = entry
    for joint in root.findall("joint"):
        name = joint.get("name")
        joints[name] = {
            "type": joint.get("type"),
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "origin": [float(v) for v in joint.find("origin").get("xyz").split()],
            "rpy": [float(v) for v in joint.find("origin").get("rpy").split()],
            "axis": [float(v) for v in joint.find("axis").get("xyz").split()],
        }
    return links, joints


def fmt(v, sig=9):
    return ", ".join(f"{x:.{sig}g}" for x in v) if isinstance(v, list) else f"{v:.{sig}g}"


cur_links, cur_joints = parse(CUR)
new_links, new_joints = parse(NEW)

print(f"current links={len(cur_links)} joints={len(cur_joints)}")
print(f"revised links={len(new_links)} joints={len(new_joints)}")

print("\n=== JOINT diffs (origin/axis) ===")
for name in sorted(set(cur_joints) | set(new_joints)):
    c = cur_joints.get(name)
    n = new_joints.get(name)
    if c is None or n is None:
        print(f"  {name}: only in {'new' if c is None else 'current'}")
        continue
    diffs = []
    for key in ("type", "parent", "child"):
        if c[key] != n[key]:
            diffs.append(f"{key}: {c[key]} vs {n[key]}")
    for key in ("origin", "rpy", "axis"):
        if not np.allclose(c[key], n[key], atol=1e-9):
            diffs.append(f"{key}: cur=[{fmt(c[key])}] new=[{fmt(n[key])}]")
    if diffs:
        print(f"  {name}:")
        for d in diffs:
            print(f"      {d}")

print("\n=== LINK inertial diffs ===")
for name in sorted(set(cur_links) | set(new_links)):
    c = cur_links.get(name)
    n = new_links.get(name)
    if c is None or n is None:
        print(f"  {name}: only in {'new' if c is None else 'current'}")
        continue
    diffs = []
    if "mass" in c and "mass" in n and abs(c["mass"] - n["mass"]) > 1e-9:
        diffs.append(f"mass: {c['mass']:.9g} vs {n['mass']:.9g}")
    for key in ("xyz", "rpy"):
        if key in c and key in n and not np.allclose(c[key], n[key], atol=1e-9):
            diffs.append(f"{key}: cur=[{fmt(c[key])}] new=[{fmt(n[key])}]")
    if "inertia" in c and "inertia" in n:
        for f in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            if abs(c["inertia"][f] - n["inertia"][f]) > 1e-9:
                diffs.append(f"inertia.{f}: {c['inertia'][f]:.9g} vs {n['inertia'][f]:.9g}")
    if diffs:
        print(f"  {name}:")
        for d in diffs:
            print(f"      {d}")