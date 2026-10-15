"""Build a colored MJCF field with independently decomposed CAD collisions.

The source remains the official 2025 national/repechage assembly. This script
does not generate a single convex hull or a heightfield for the whole arena.
"""
from pathlib import Path
import argparse
import copy
import ctypes
import gc
from collections import defaultdict
import hashlib
import json
import resource
import re
import shutil
import struct
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parent
sys.path.append(str(ROOT/".caddeps"))
# CAD is Y-up and its long direction is Z. This proper cyclic rotation maps
# X,Y,Z to Z,X,Y: the MuJoCo long side is X and vertical is Z.
CAD_TO_WORLD = np.array([[0., 0, 1], [1., 0, 0], [0, 1, 0]])


def write_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)


def numbers(values):
    return " ".join(f"{float(x):.10g}" for x in values)


def write_msh(mesh, path):
    """Stream MuJoCo's binary mesh format instead of formatting a giant OBJ.

    Header: vertex, normal, texcoord and triangle counts (little-endian int32).
    Normal/UV counts zero ask MuJoCo to compute normals; vertex float32 and
    face int32 rows follow. Conversion buffers are bounded to 8192 rows.
    """
    with Path(path).open("wb") as stream:
        np.asarray([len(mesh.vertices), 0, 0, len(mesh.faces)], dtype="<i4").tofile(stream)
        for array, dtype in ((mesh.vertices, "<f4"), (mesh.faces, "<i4")):
            for start in range(0, len(array), 8192):
                np.asarray(array[start:start+8192], dtype=dtype).tofile(stream)


def material_rgba(mesh):
    material = getattr(mesh.visual, "material", None)
    rgba = getattr(material, "main_color", np.array([178, 178, 178, 255]))
    return tuple(int(x) for x in rgba)


def planar_prisms(mesh, source_volume):
    """Preserve thin extruded plates and holes using their top triangulation.

    Convex adjacent polygons are merged only when their combined area equals
    their convex hull area. This avoids voxelizing millimetre-thick CAD sheets.
    """
    axis = int(np.argmin(mesh.extents))
    thickness = float(mesh.extents[axis])
    if not 1e-5 < thickness < .03:
        return None
    vertices = np.asarray(mesh.vertices)
    lo, hi = mesh.bounds[:, axis]
    triangles = vertices[mesh.faces]
    selected = np.all(abs(triangles[:, :, axis]-hi) < 1e-6, axis=1) & (mesh.area_faces > 1e-12)
    top_faces = mesh.faces[selected]
    if not len(top_faces):
        return None
    top_area = float(mesh.area_faces[selected].sum())
    if abs(top_area*thickness-source_volume) > max(source_volume*.003, 1e-10):
        return None
    plane_axes = [x for x in range(3) if x != axis]
    points = vertices[:, plane_axes]
    polygons = [list(face) for face in top_faces]
    areas = [float(value) for value in mesh.area_faces[selected]]
    for _ in range(32):
        edges = defaultdict(list)
        for i, poly in enumerate(polygons):
            if poly is not None:
                for j, vertex in enumerate(poly):
                    edges[tuple(sorted((vertex, poly[(j+1) % len(poly)])))].append(i)
        merged = 0
        for adjacent in edges.values():
            if len(adjacent) != 2:
                continue
            left, right = adjacent
            if polygons[left] is None or polygons[right] is None:
                continue
            indices = list(set(polygons[left]+polygons[right]))
            hull = ConvexHull(points[indices])
            combined = areas[left]+areas[right]
            if abs(hull.volume-combined) < max(1e-11, combined*1e-7):
                polygons[left] = [indices[x] for x in hull.vertices]
                areas[left] = combined
                polygons[right] = None
                merged += 1
        if not merged:
            break
    # Tiny drilled/etched components can contain thousands of little polygons.
    # They use the same 1 cm CoACD approximation as other hardware; large arena
    # plates keep the exact prisms so traversable holes remain open.
    if sum(poly is not None for poly in polygons) > 128 and mesh.extents.max() < 2:
        return None
    hulls = []
    for poly in polygons:
        if poly is not None:
            top = vertices[poly].copy(); top[:, axis] = hi
            bottom = top.copy(); bottom[:, axis] = lo
            hulls.append(trimesh.Trimesh(vertices=np.concatenate([top, bottom]), faces=None).convex_hull)
    return hulls


def exact_convex_parts(mesh):
    """Constrained tetrahedra, merged only when their union is convex."""
    import tetgen
    generator = tetgen.TetGen(np.asarray(mesh.vertices), np.asarray(mesh.faces, dtype=np.int32))
    cavities = []
    components = trimesh.graph.connected_components(mesh.face_adjacency, nodes=np.arange(len(mesh.faces)), min_len=1)
    for component in components:
        shell = mesh.submesh([component], append=True, repair=False)
        if shell.volume < -1e-10:
            face = int(np.argmax(shell.area_faces))
            point = shell.triangles_center[face]+shell.face_normals[face]*1e-5
            generator.add_hole(point.tolist())
            cavities.append(point.tolist())
    nodes, elements, *_ = generator.tetrahedralize(switches="pzYQ")
    vertices = nodes[elements]
    volumes = abs(np.linalg.det(vertices[:, 1:]-vertices[:, :1]))/6
    total = float(volumes.sum())
    if abs(total-abs(mesh.volume)) > max(1e-9, abs(mesh.volume)*1e-7):
        raise RuntimeError(f"constrained tetrahedra do not reproduce the CAD volume: {total} vs {mesh.volume}")
    parent = list(range(len(elements)))
    groups = [set(map(int, tet)) for tet in elements]
    facets = defaultdict(list)
    for i, tet in enumerate(elements):
        for face in ((0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)):
            facets[tuple(sorted(int(tet[k]) for k in face))].append(i)
    adjacent = [values for values in facets.values() if len(values) == 2]
    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index
    for _ in range(32):
        merges = 0; attempted = set()
        for left, right in adjacent:
            left, right = root(left), root(right)
            pair = tuple(sorted((left, right)))
            if left == right or pair in attempted:
                continue
            attempted.add(pair)
            union = groups[left] | groups[right]
            hull = ConvexHull(nodes[list(union)])
            combined = volumes[left]+volumes[right]
            if abs(hull.volume-combined) < max(1e-10, combined*1e-8):
                parent[right] = left
                groups[left] = union
                groups[right] = set()
                volumes[left] = combined
                merges += 1
        if not merges:
            break
    hulls = [trimesh.Trimesh(vertices=nodes[list(indices)], faces=None).convex_hull
             for i, indices in enumerate(groups) if indices and root(i) == i and volumes[i] > 1e-12]
    audit = {"switches": "pzYQ", "tetrahedra": len(elements), "convex_parts": len(hulls),
             "cad_cavity_markers": cavities,
             "tetra_volume_m3": total, "mesh_volume_m3": abs(mesh.volume),
             "merged_volume_m3": sum(abs(hull.volume) for hull in hulls)}
    if abs(audit["merged_volume_m3"]-total) > max(1e-8, total*1e-6):
        raise RuntimeError("convex merging changed the CAD volume")
    return hulls, audit


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--cad", type=Path, default=ROOT/"build/cad78")
    p.add_argument("--output", type=Path, default=ROOT/"pack")
    p.add_argument("--resume", action="store_true", help="resume this builder's partially generated directory")
    p.add_argument("--visual-detail", choices=("compact", "cad"), default="compact")
    args = p.parse_args()
    if args.output.exists() and not args.resume:
        p.error("output exists; choose a new directory")
    resource.setrlimit(resource.RLIMIT_AS, (4*1024**3, 4*1024**3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    started = time.monotonic()
    source = json.loads((args.cad/"assembly.json").read_text())
    args.output.mkdir(parents=True, exist_ok=args.resume)
    (args.output/"meshes").mkdir(exist_ok=args.resume)
    state_path = args.output/"build_state.json"
    state = {"source_sha256": source["source_sha256"], "policy": "coacd_1cm_64_seed2025_prisms_v4_exact_terrain", "parts": {}}
    if state_path.exists():
        previous = json.loads(state_path.read_text())
        if previous["source_sha256"] != state["source_sha256"] or previous["policy"] not in (
                state["policy"], "coacd_1cm_64_seed2025_v1", "coacd_1cm_64_seed2025_prisms_v2", "coacd_1cm_64_seed2025_prisms_v3"):
            raise ValueError("resume source or collision settings differ")
        previous["policy"] = state["policy"]
        state = previous
    # Interrupted legacy runs predate the checkpoint file. Every lower ID is
    # complete; the highest written ID is recomputed, including all its pieces.
    prior_files = defaultdict(list)
    if args.resume:
        for path in sorted((args.output/"meshes").glob("rmuc2025_solid_*.msh")):
            match = re.fullmatch(r"rmuc2025_solid_(\d+)_(\d+)\.msh", path.name)
            if match:
                prior_files[int(match[1])].append(path)
    last_prior = max(prior_files, default=-1)
    # Keep the original tessellation for independent collision-height checks.
    if (args.output/"field.xml").exists() and not (args.output/"field_cad_reference.xml").exists():
        original_field = ET.parse(args.output/"field.xml").getroot()
        reference_dir = args.output/"reference_meshes"
        reference_dir.mkdir(exist_ok=True)
        for asset in original_field.find("asset"):
            if asset.get("name", "").startswith("rmuc2025_visual_"):
                destination = reference_dir/Path(asset.get("file")).name
                shutil.copyfile(args.output/asset.get("file"), destination)
                asset.set("file", str(destination.relative_to(args.output)))
        write_xml(original_field, args.output/"field_cad_reference.xml")
        reference_scene = ET.parse(args.output/"scene.xml").getroot()
        reference_scene.find("include").set("file", "field_cad_reference.xml")
        write_xml(reference_scene, args.output/"scene_cad_reference.xml")
    scene = trimesh.load(args.cad/source["visual_glb"], force="scene", process=False)
    # This STEP's XCAF metadata already describes Y-up. The pinned OCP exporter
    # preserves CAD XYZ and converts millimetres to glTF metres. Check that
    # contract against the B-rep bounds rather than assuming an axis convention.
    raw_bounds = np.asarray(scene.bounds)
    b = np.asarray(source["cad_bounds_mm"])
    expected = b.reshape(2, 3)*.001
    error = float(np.max(abs(raw_bounds-expected)))
    if error > .015:
        raise ValueError(f"glTF/CAD axis or unit mismatch: {raw_bounds.tolist()} vs {expected.tolist()}")
    visuals = []
    visual_cache = {}
    simplification = []
    bins = defaultdict(float)
    sums = defaultdict(float)
    for node in scene.graph.nodes_geometry:
        transform, name = scene.graph[node]
        original = scene.geometry[name]
        rgba = material_rgba(original)
        if name not in visual_cache:
            simplified = original
            target = min(len(original.faces), max(128, min(len(original.faces)//5,
                3000 if original.extents.max() > 3 else 1500 if original.extents.max() > 1 else 600)))
            if args.visual_detail == "compact" and target < len(original.faces):
                simplified = original.simplify_quadric_decimation(face_count=target, aggression=5)
                if len(simplified.faces) < 4 or np.max(abs(simplified.bounds-original.bounds)) > .003:
                    simplified = original
            visual_cache[name] = simplified
            simplification.append({"geometry": name, "cad_faces": len(original.faces),
                                   "visual_faces": len(simplified.faces),
                                   "bounds_error_m": float(np.max(abs(simplified.bounds-original.bounds)))})
        mesh = visual_cache[name].copy()
        mesh.apply_transform(transform)
        mesh.vertices = np.asarray(mesh.vertices)[:, [2, 0, 1]]
        normals = mesh.face_normals
        mask = normals[:, 2] > .999
        heights, areas = mesh.triangles_center[mask, 2], mesh.area_faces[mask]
        for height, area in zip(heights, areas):
            key = round(float(height)/.005)
            bins[key] += float(area)
            sums[key] += float(height*area)
        visuals.append((mesh, rgba))
    if not bins:
        raise ValueError("no upward CAD floor surfaces")
    winner = max(bins, key=bins.get)
    floor = sums[winner]/bins[winner]
    # Center only the horizontal envelope; the main CAD floor is world z=0.
    world_bounds = np.array([[b[2], b[0], b[1]], [b[5], b[3], b[4]]])*.001
    origin = np.array([*(world_bounds.mean(axis=0)[:2]), floor])
    colored = defaultdict(list)
    for mesh, rgba in visuals:
        mesh.apply_translation(-origin)
        # The CAD marks its PVC floor blue. The rules/reference photos show
        # dark PVC; recolor only the large flat floor sheets, preserving LEDs.
        if rgba == (0, 0, 255, 255):
            z = mesh.triangles[:, :, 2]
            floor_faces = (np.max(z, axis=1) < .004) & (np.min(z, axis=1) > -.01) & (mesh.area_faces > .05) & (abs(mesh.face_normals[:, 2]) > .999)
            if np.any(floor_faces):
                colored[(28, 31, 36, 255)].append(mesh.submesh([np.flatnonzero(floor_faces)], append=True))
                if np.all(floor_faces):
                    continue
                mesh = mesh.submesh([np.flatnonzero(~floor_faces)], append=True)
        colored[rgba].append(mesh)
    full = ET.Element("mujoco", model="rmuc2025_field")
    assets = ET.SubElement(full, "asset")
    world = ET.SubElement(full, "worldbody")
    for index, (rgba, meshes) in enumerate(sorted(colored.items())):
        mesh = trimesh.util.concatenate(meshes)
        name = f"rmuc2025_visual_{index:02d}"
        filename = f"meshes/{name}.msh"
        write_msh(mesh, args.output/filename)
        ET.SubElement(assets, "mesh", name=name, file=filename, inertia="shell", maxhullvert="64")
        ET.SubElement(world, "geom", name=name, type="mesh", mesh=name,
                      contype="0", conaffinity="0", group="1", density="0", rgba=numbers(np.array(rgba)/255))
    print("visual palettes", len(colored), "triangles", sum(len(m.faces) for m, _ in visuals),
          "floor", floor, "origin", origin, flush=True)
    visual_face_count = sum(len(m.faces) for m, _ in visuals)
    del scene, visuals, colored, visual_cache, original, simplified, meshes, mesh, normals, mask, heights, areas
    gc.collect()
    ctypes.CDLL(None).malloc_trim(0)
    collision = ET.Element("mujoco", model="rmuc2025_collision")
    cassets = ET.SubElement(collision, "asset")
    cworld = ET.SubElement(collision, "worldbody")
    parts = {}
    audit = []
    worker_dir = ROOT/"build/decomposition"
    worker_dir.mkdir(parents=True, exist_ok=True)
    for record in source["unique_solids"]:
        data = np.load(args.cad/record["mesh"])
        mesh = trimesh.Trimesh(vertices=data["vertices_mm"]*.001, faces=data["faces"], process=True)
        volume = record["volume_mm3"]*1e-9
        index = record["id"]
        if mesh.extents.max() < .025 or volume < 1e-8:
            parts[index] = []
            audit.append({"id": index, "collision": "omitted small hardware", "volume_m3": volume,
                          "max_extent_m": float(mesh.extents.max())})
            continue
        try:
            hull = mesh.convex_hull
        except Exception as exc:
            raise RuntimeError(f"solid {index} convex hull failed") from exc
        ratio = volume/max(abs(hull.volume), 1e-12)
        threshold = .9999 if mesh.extents.max() > 3 else .985
        decomposition = ratio < threshold
        hulls = [hull]
        method = "convex hull"
        repair = None
        coacd_options = None
        exact_audit = None
        cached = state["parts"].get(str(index))
        oversized_prisms = cached is not None and cached["audit"]["collision"].startswith("planar") and len(cached["files"]) > 128 and mesh.extents.max() < 2
        exact_terrain = index in (73, 83)
        requires_exact = exact_terrain and (cached is None or cached["audit"]["collision"] != "tetgen PLC with convex merging")
        reused = not (oversized_prisms or requires_exact) and (cached is not None or (index < last_prior and index in prior_files))
        if oversized_prisms or requires_exact:
            cached = None
        if cached and cached["audit"]["collision"] == "coacd" and not cached["audit"].get("coacd_parameters"):
            worker_output = worker_dir/f"output_{index}.npz"
            if worker_output.exists():
                with np.load(worker_output) as checkpoint:
                    if "parameters" in checkpoint.files:
                        cached["audit"]["coacd_parameters"] = json.loads(str(checkpoint["parameters"]))
            if not cached["audit"].get("coacd_parameters"):
                repair_info = cached["audit"].get("repair")
                cached["audit"]["coacd_parameters"] = {"sampling": 1000, "mcts_iterations": 30,
                    "mcts_nodes": 8, "mcts_max_depth": 3, "preprocess_mode": "auto" if repair_info else "off",
                    "preprocess_resolution": repair_info["resolution"] if repair_info else 200}
        if reused:
            paths = [args.output/x for x in cached["files"]] if cached else prior_files[index]
            for path in paths:
                with path.open("rb") as stream:
                    nv, nn, nt, nf = struct.unpack("<4i", stream.read(16))
                if path.stat().st_size != 16+12*(nv+nn+nf)+8*nt or nv < 4:
                    raise ValueError(f"incomplete checkpoint mesh: {path}")
            hulls = [None]*len(paths)
        elif exact_terrain:
            hulls, exact_audit = exact_convex_parts(mesh)
            method = "tetgen PLC with convex merging"
            print("exact terrain solid", index, exact_audit, flush=True)
        elif decomposition:
            hulls = planar_prisms(mesh, volume)
            if hulls is not None:
                method = "planar triangle prisms, convex adjacent polygons merged"
                print("planar CAD solid", index, "prisms", len(hulls), flush=True)
        if not reused and decomposition and hulls is None:
            preprocess = "off"
            # CAD already gives outward solid normals, including inward normals
            # on cavity shells. Flipping every component would fill those voids.
            mesh.fix_normals(multibody=False)
            if not mesh.is_watertight:
                mesh.merge_vertices(digits_vertex=6)
                mesh.update_faces(mesh.nondegenerate_faces())
                mesh.update_faces(mesh.unique_faces())
                mesh.remove_unreferenced_vertices()
                mesh.fill_holes()
                mesh.fix_normals(multibody=False)
            if not mesh.is_watertight:
                relative_volume_error = abs(abs(mesh.volume)-volume)/max(volume, 1e-12)
                if relative_volume_error > .02:
                    raise RuntimeError(f"nonconvex solid {index} has an unclosed CAD volume ({relative_volume_error:.3%})")
                preprocess = "auto"
                repair = {"method": "coacd manifold preprocessing", "resolution": 200,
                          "source_tessellation_volume_error_relative": relative_volume_error}
            print("decomposing CAD solid", index, "ratio", round(ratio, 4), "extent", mesh.extents.tolist(), flush=True)
            input_file = worker_dir/f"input_{index}.npz"
            output_file = worker_dir/f"output_{index}.npz"
            np.savez(input_file, vertices=np.asarray(mesh.vertices), faces=np.asarray(mesh.faces, dtype=np.int32))
            gc.collect()
            available_mb = int(next(line.split()[1] for line in Path("/proc/meminfo").read_text().splitlines()
                                    if line.startswith("MemAvailable:")))/1024
            if available_mb < 3500:
                raise RuntimeError(f"RAM guard before worker: {available_mb:.0f} MiB; resume later")
            resolutions = (50, 25) if mesh.extents.max() < .25 else ((100, 50) if mesh.extents.max() < .5 else (200, 100))
            for preprocess_resolution in resolutions:
                command = [sys.executable, str(ROOT/"decompose_worker.py"), str(input_file), str(output_file),
                           "--preprocess", preprocess, "--preprocess-resolution", str(preprocess_resolution)]
                try:
                    result = subprocess.run(command, timeout=90)
                except subprocess.TimeoutExpired:
                    print("CoACD worker time guard; retrying", index, flush=True)
                    continue
                if result.returncode == 0:
                    if repair:
                        repair["resolution"] = preprocess_resolution
                    break
                if preprocess == "off":
                    raise RuntimeError(f"bounded CoACD worker failed: solid {index}, exit {result.returncode}")
            else:
                raise RuntimeError(f"bounded CoACD preprocessing failed: solid {index}")
            with np.load(output_file) as output:
                if "parameters" in output.files:
                    coacd_options = json.loads(str(output["parameters"]))
                hulls = [trimesh.Trimesh(vertices=output[f"v{k}"], faces=output[f"f{k}"], process=False)
                         for k in range(len(output.files)//2)]
            method = "coacd"
        names = []
        for k, hull in enumerate(hulls):
            name = f"rmuc2025_solid_{index:04d}_{k:02d}"
            file = f"meshes/{name}.msh"
            if not reused:
                write_msh(hull, args.output/file)
            ET.SubElement(cassets, "mesh", name=name, file=file, inertia="convex")
            names.append(name)
        parts[index] = names
        audit.append({"id": index, "collision": cached["audit"]["collision"] if cached else ("coacd" if reused and decomposition else method),
                      "source_volume_m3": volume, "source_hull_volume_ratio": ratio,
                      "pieces": len(names), "mesh_watertight": bool(mesh.is_watertight),
                      "repair": cached["audit"].get("repair") if cached else repair,
                      "coacd_parameters": cached["audit"].get("coacd_parameters") if cached else coacd_options,
                      "exact_decomposition": cached["audit"].get("exact_decomposition") if cached else exact_audit})
        state["parts"][str(index)] = {"files": [f"meshes/{name}.msh" for name in names], "audit": audit[-1]}
        state_path.write_text(json.dumps(state)+"\n")
        if len(audit) % 25 == 0:
            print("collision solids processed", len(audit), "/", len(source["unique_solids"]), flush=True)
        gc.collect()
    for i, instance in enumerate(source["instances"]):
        pose = np.asarray(instance["cad_transform_mm"])
        rotation = CAD_TO_WORLD@pose[:3, :3]
        quat = Rotation.from_matrix(rotation).as_quat()[[3, 0, 1, 2]]
        position = CAD_TO_WORLD@pose[:3, 3]*.001-origin
        for k, name in enumerate(parts[instance["solid_id"]]):
            ET.SubElement(cworld, "geom", name=f"rmuc2025_contact_{i:05d}_{k:02d}", type="mesh", mesh=name,
                          pos=numbers(position), quat=numbers(quat), contype="4", conaffinity="3", group="3",
                          density="0", condim="3", friction="1.2 .02 .001", solref=".008 1",
                          solimp=".92 .99 .002", rgba=".5 .55 .6 1")
    # Full and collision-only entrypoints contain exactly the same contact geoms.
    write_xml(collision, args.output/"field_collision.xml")
    for a in cassets:
        assets.append(a)
    for g in cworld:
        visual_collision = copy.deepcopy(g)
        visual_collision.set("rgba", "0 0 0 0")
        world.append(visual_collision)
    write_xml(full, args.output/"field.xml")
    reference_path = args.output/"field_cad_reference.xml"
    if reference_path.exists():
        reference = ET.parse(reference_path).getroot()
        rassets, rworld = reference.find("asset"), reference.find("worldbody")
        for asset in list(rassets):
            if asset.get("name", "").startswith("rmuc2025_solid_"):
                rassets.remove(asset)
        for geom in list(rworld):
            if geom.get("name", "").startswith("rmuc2025_contact_"):
                rworld.remove(geom)
        for asset in cassets:
            rassets.append(copy.deepcopy(asset))
        for geom in cworld:
            item = copy.deepcopy(geom); item.set("rgba", "0 0 0 0"); rworld.append(item)
        write_xml(reference, reference_path)
    perimeter = ET.Element("mujoco", model="optional_rules_perimeter")
    pworld = ET.SubElement(perimeter, "worldbody")
    for name, position, size in (("west", (-14.025, 0, 1.2), (.025, 7.5, 1.2)),
                                 ("east", (14.025, 0, 1.2), (.025, 7.5, 1.2)),
                                 ("south", (0, -7.525, 1.2), (14.05, .025, 1.2)),
                                 ("north", (0, 7.525, 1.2), (14.05, .025, 1.2))):
        ET.SubElement(pworld, "geom", name=f"rmuc2025_perimeter_{name}", type="box",
                      pos=numbers(position), size=numbers(size), density="0", group="4",
                      contype="4", conaffinity="3", friction="1.2 .02 .001", rgba=".15 .18 .22 .2")
    write_xml(perimeter, args.output/"field_perimeter.xml")
    for profile, entry in (("full", "field.xml"), ("collision", "field_collision.xml")):
        root = ET.Element("mujoco", model=f"RMUC2025_{profile}")
        ET.SubElement(root, "compiler", angle="radian")
        ET.SubElement(root, "option", timestep=".001", gravity="0 0 -9.81", integrator="implicitfast")
        visual = ET.SubElement(root, "visual")
        ET.SubElement(visual, "global", offwidth="1280", offheight="720")
        ET.SubElement(visual, "headlight", ambient=".35 .35 .35", diffuse=".6 .6 .6", specular=".1 .1 .1")
        background = ET.SubElement(root, "asset")
        ET.SubElement(background, "texture", name="rmuc2025_sky", type="skybox", builtin="gradient",
                      rgb1=".16 .21 .29", rgb2=".025 .035 .055", width="64", height="384")
        ET.SubElement(root, "include", file=entry)
        wb = ET.SubElement(root, "worldbody")
        ET.SubElement(wb, "light", pos="0 0 12", dir="0 0 -1", directional="true", castshadow="false", diffuse=".5 .5 .5")
        write_xml(root, args.output/("scene.xml" if profile == "full" else "scene_collision.xml"))
        ET.SubElement(root, "include", file="field_perimeter.xml")
        write_xml(root, args.output/("scene_bounded.xml" if profile == "full" else "scene_collision_bounded.xml"))
    manifest = {"source": {"page": "https://bbs.robomaster.com/article/372035", "sha256": source["source_sha256"],
                           "version": "RMUC2025 national/repechage V2.0.0", "units": "m"},
                "official_core_size_m": [28, 15], "official_perimeter_height_m": 2.4,
                "cad_bounds_mm": source["cad_bounds_mm"], "world_origin_before_shift_m": origin.tolist(),
                "cad_to_world_rotation": CAD_TO_WORLD.tolist(), "glb_bounds_error_m": error,
                "dominant_floor_upward_area_m2": bins[winner],
                "collision_instances": len(cworld), "collision_parts": sum(len(x) for x in parts.values()),
                "unique_cad_solids": len(source["unique_solids"]), "cad_solid_instances": source["solid_count"],
                "visual_detail": args.visual_detail, "visual_faces": visual_face_count,
                "visual_simplification": simplification,
                "floor_color": "dark PVC per official rules; original blue CAD stored in reference_meshes",
                "optional_perimeter": "rules-based 28x15m inner rectangle, 2.4m high, 50mm assumed thickness; transparent for inspection; entrances not modeled",
                "collision_policy": "individual CAD solids; convex decomposition for concave parts and exact triangulated extrusion prisms for thin plates; no whole-map hull/heightfield",
                "coacd": {"threshold_m": .01, "max_hulls": 64, "seed": 2025},
                "exact_terrain_solids": [73, 83],
                "friction": {"values": [1.2, .02, .001], "measured_official_material": False,
                             "meaning": "RL-lab baseline, editable by caller"},
                "scope": "static CAD snapshot; referee logic and moving mechanisms are not simulated",
                "solid_collision_audit": audit,
                "wall_seconds": time.monotonic()-started, "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,
                "files": {}}
    for path in sorted(args.output.rglob("*")):
        if path.is_file() and path.name != "manifest.json":
            manifest["files"][str(path.relative_to(args.output))] = hashlib.sha256(path.read_bytes()).hexdigest()
    (args.output/"manifest.json").write_text(json.dumps(manifest, indent=2)+"\n")
    print("field ready", args.output, "contact geoms", len(cworld), "RSS MiB", manifest["peak_rss_mb"], flush=True)


if __name__ == "__main__":
    main()
