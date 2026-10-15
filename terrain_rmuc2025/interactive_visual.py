"""Display-only arena LOD. The original CAD and every collision stay intact."""
from pathlib import Path
import fcntl
import hashlib
import json
import os
import struct
import sys
import xml.etree.ElementTree as ET

import numpy as np


def simplify_parts(points, faces, simplify):
    """Keep each disconnected CAD part, including slender boundary fixtures."""
    from scipy.sparse import coo_matrix
    from scipy.sparse.csgraph import connected_components

    a = np.concatenate((faces[:, 0], faces[:, 1], faces[:, 2]))
    b = np.concatenate((faces[:, 1], faces[:, 2], faces[:, 0]))
    graph = coo_matrix((np.ones(len(a), dtype=np.uint8), (a, b)),
                       shape=(len(points), len(points))).tocsr()
    _, labels = connected_components(graph, directed=False)
    # Preserve global extremities even when they belong to small decorative
    # caps; omit other isolated sub-30 mm CAD fragments in the operator view.
    used = np.unique(faces)
    extremities = np.r_[used[np.argmin(points[used], axis=0)],
                        used[np.argmax(points[used], axis=0)]]
    face_labels = labels[faces[:, 0]]
    order = np.argsort(face_labels, kind="stable")
    ends = np.r_[0, np.flatnonzero(np.diff(face_labels[order])) + 1, len(order)]
    del graph, a, b, labels, face_labels
    chunks, indices, offset = [], [], 0
    fallback, omitted = 0, 0
    for start, end in zip(ends[:-1], ends[1:]):
        part = faces[order[start:end]]
        used, remap = np.unique(part, return_inverse=True)
        vertices, triangles = points[used], remap.reshape(-1, 3).astype(np.int32)
        if np.max(np.ptp(vertices, axis=0)) < .03:
            # Keep the six global extrema without retaining the sparse graph.
            if not np.any(np.isin(used, extremities)):
                omitted += 1
                continue
        target = min(len(triangles), max(16, round(len(triangles)*.04)))
        if target < len(triangles):
            simplified, reduced = simplify(vertices, triangles, target_count=target,
                agg=7, preserve_border=bool(np.max(np.ptp(vertices, axis=0)) > .5))
            bounds_error = np.max(np.abs(np.array([simplified.min(0), simplified.max(0)]) -
                                         np.array([vertices.min(0), vertices.max(0)])))
            if bounds_error <= .003:
                vertices, triangles = simplified, reduced
            else:
                fallback += 1
        chunks.append(vertices)
        indices.append(triangles + offset)
        offset += len(vertices)
    return np.concatenate(chunks), np.concatenate(indices), len(ends)-1, fallback, omitted


def interactive_field(pack: Path) -> Path:
    pack = Path(pack).resolve()
    original = pack / "field.xml"
    digest = hashlib.sha256(original.read_bytes() + Path(__file__).read_bytes()).hexdigest()[:20]
    cache = pack.parent.parent / "out/models" / ("rmuc-visual-" + digest)
    cache.mkdir(parents=True, exist_ok=True)
    field_path = cache / "field.xml"
    with (cache / "build.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if field_path.exists():
            return field_path
        sys.path.insert(0, str(pack.parent / ".caddeps"))
        import fast_simplification

        root = ET.parse(original).getroot()
        report = []
        for asset in root.findall("asset/mesh"):
            source = pack / asset.get("file")
            if not asset.get("name", "").startswith("rmuc2025_visual_"):
                asset.set("file", str(source))
                continue
            with source.open("rb") as stream:
                nv, nn, nt, nf = struct.unpack("<4i", stream.read(16))
                points = np.fromfile(stream, dtype="<f4", count=nv*3).reshape(-1, 3)
                stream.seek(12*nn + 8*nt, 1)
                faces = np.fromfile(stream, dtype="<i4", count=nf*3).reshape(-1, 3)
            # CAD face patches duplicate vertices at their seams. Weld those
            # display vertices at 10 micrometres before preserving borders;
            # otherwise every CAD patch edge prevents useful simplification.
            welded, inverse = np.unique(np.rint(points.astype(np.float64)*1e5).astype(np.int64),
                                         axis=0, return_inverse=True)
            visual_points = welded.astype(np.float64)*1e-5
            visual_faces = inverse[faces].astype(np.int32)
            distinct = ((visual_faces[:, 0] != visual_faces[:, 1]) &
                        (visual_faces[:, 1] != visual_faces[:, 2]) &
                        (visual_faces[:, 2] != visual_faces[:, 0]))
            visual_faces = visual_faces[distinct]
            # Keep open road/floor edges. Remove fine CAD tessellation and
            # small hardware detail; control/contact geometry is never read
            # or passed through this simplifier.
            vertices, triangles, parts, fallbacks, omitted = simplify_parts(
                visual_points, visual_faces, fast_simplification.simplify)
            error = float(np.max(np.abs(np.array([vertices.min(0), vertices.max(0)]) -
                                       np.array([points.min(0), points.max(0)]))))
            if error > .02:
                raise RuntimeError(f"visual LOD changed arena bounds by {error} m: {source.name}")
            destination = cache / source.name
            with destination.open("wb") as stream:
                np.asarray([len(vertices), 0, 0, len(triangles)], dtype="<i4").tofile(stream)
                np.asarray(vertices, dtype="<f4").tofile(stream)
                np.asarray(triangles, dtype="<i4").tofile(stream)
            asset.set("file", str(destination))
            row = dict(mesh=source.name, original_faces=nf, display_faces=len(triangles),
                       bounds_error_m=error, parts=parts, unreduced_parts=fallbacks,
                       omitted_sub_30mm_fragments=omitted)
            report.append(row)
            print(f"Display LOD {source.stem}: {nf} -> {len(triangles)} triangles", flush=True)
        original_contacts = ET.parse(original).getroot().findall("worldbody/geom")
        new_contacts = root.findall("worldbody/geom")
        assert [ET.tostring(g) for g in original_contacts] == [ET.tostring(g) for g in new_contacts]
        manifest = dict(meshes=report, collision_geoms_unchanged=True,
                        original_visual_faces=sum(row["original_faces"] for row in report),
                        display_visual_faces=sum(row["display_faces"] for row in report))
        (cache / "report.json").write_text(json.dumps(manifest, indent=2)+"\n")
        ET.indent(root)
        temporary = cache / "field.tmp.xml"
        ET.ElementTree(root).write(temporary, encoding="utf-8", xml_declaration=True)
        os.replace(temporary, field_path)
        return field_path
