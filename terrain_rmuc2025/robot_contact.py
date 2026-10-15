"""Use the existing chassis CAD contacts and a fitted recovery support in RMUC."""
import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def refine_chassis_contacts(root):
    """Replace the fast chassis AABB with its existing CAD convex parts.

    Keep the fast leg/wheel model, body tree and explicit mass/inertia. The
    diagonal front corner of the AABB otherwise grounds on an uphill ramp
    before the real chassis does. These are the same passive collisions
    already available in the full robot model, with the same materials.
    """
    from balance_lab.paths import MODEL_FULL

    reference = ET.parse(MODEL_FULL).getroot()
    chassis = root.find("worldbody/body[@name='base_link']")
    old = chassis.find("geom[@name='fast_collision_base_link']")
    if old is None:
        raise ValueError("expected the fast chassis collision box")
    reference_chassis = reference.find("worldbody/body[@name='base_link']")
    meshes = {asset.get("name"): asset for asset in reference.findall("asset/mesh")}
    parts = [geom for geom in reference_chassis.findall("geom")
             if geom.get("mesh", "").startswith("base_link_collision_")]
    if not parts:
        raise ValueError("full robot has no CAD chassis collision parts")
    chassis.remove(old)
    for index, geom in enumerate(parts):
        original_name = geom.get("mesh")
        name = f"rmuc_robot_chassis_{index:03d}"
        asset = copy.deepcopy(meshes[original_name])
        asset.set("name", name)
        asset.set("file", str((MODEL_FULL.parent / asset.get("file")).resolve()))
        root.find("asset").append(asset)
        contact = copy.deepcopy(geom)
        contact.set("name", name)
        contact.set("mesh", name)
        contact.set("group", "3")
        chassis.append(contact)
    return len(parts)


def fit_recovery_support(model):
    """Keep the fixed, high-friction skid, without its oversized overhang.

    The flat-scene helper spans 440 x 240 mm. Its forward edge reaches a
    ramp while both drive wheels are still on the flat. Fit its horizontal
    footprint to the lowest 8 mm of the existing chassis CAD instead. Keep
    its original height, thickness and material for prone recovery.
    """
    kind = mujoco.mjtObj.mjOBJ_GEOM
    patch = mujoco.mj_name2id(model, kind, "guide_wheel_contact")
    visual = mujoco.mj_name2id(model, kind, "fast_visual_base_link")
    if patch < 0 or visual < 0:
        raise ValueError("arena robot is missing its chassis CAD/support patch")
    mesh = int(model.geom_dataid[visual])
    begin, count = int(model.mesh_vertadr[mesh]), int(model.mesh_vertnum[mesh])
    rotation = np.empty(9)
    mujoco.mju_quat2Mat(rotation, model.geom_quat[visual])
    vertices = model.mesh_vert[begin:begin+count] @ rotation.reshape(3, 3).T + model.geom_pos[visual]
    # Chassis CAD Y is the support plane normal. Preserve the real bottom
    # guide-wheel footprint, including its non-rolling contact assumption.
    support = vertices[vertices[:, 1] <= vertices[:, 1].min() + .008]
    mujoco.mju_quat2Mat(rotation, model.geom_quat[patch])
    rotation = rotation.reshape(3, 3)
    points = (support - model.geom_pos[patch]) @ rotation
    lower, upper = points[:, :2].min(axis=0)-.002, points[:, :2].max(axis=0)+.002
    old_size, old_pos = model.geom_size[patch].copy(), model.geom_pos[patch].copy()
    if not (np.all(lower >= -old_size[:2]) and np.all(upper <= old_size[:2])):
        raise ValueError("CAD support footprint must be contained in the original helper")
    offset = np.r_[(lower+upper)/2, 0.]
    model.geom_pos[patch] += rotation @ offset
    model.geom_size[patch, :2] = (upper-lower)/2
    model.geom_aabb[patch, :3] = 0
    model.geom_aabb[patch, 3:] = model.geom_size[patch]
    model.geom_rbound[patch] = np.linalg.norm(model.geom_size[patch])
    # The compiled body's original BVH encloses this smaller box, so it
    # remains conservative. The next mj_forward updates the geom transform.
    return dict(old_position_m=old_pos.tolist(), old_half_size_m=old_size.tolist(),
                position_m=model.geom_pos[patch].tolist(),
                half_size_m=model.geom_size[patch].tolist(), cad_support_vertices=len(support))
