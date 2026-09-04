"""Create high-fidelity convex collision pieces from the exported CAD STLs.

The visual CAD files are never changed.  MuJoCo treats a dynamic mesh as a
single convex hull, which bridges holes and open frames in a wheel-leg.  This
script makes a compact CoACD union for every moving component so ground
contact follows the actual CAD envelope instead of an imaginary solid hull.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import argparse
from pathlib import Path
import struct

import coacd
import numpy as np
import trimesh


ROOT = REPO_ROOT
SOURCE_DIR = ROOT / "model" / "visual_meshes"
OUTPUT_DIR = ROOT / "model" / "collision_meshes_vhacd"
# The collision model need not preserve sub-millimetre STL tessellation.  The
# untouched visual meshes still do; this only limits collision complexity.
VOXEL_M = 0.002


def write_binary_stl(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    triangles = np.asarray(vertices, dtype=np.float32)[np.asarray(faces, dtype=np.int32)]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    normals[lengths > 1e-12] /= lengths[lengths > 1e-12, None]
    records = np.zeros((len(triangles), 50), dtype=np.uint8)
    records[:, :12] = normals.astype("<f4").view(np.uint8).reshape(-1, 12)
    records[:, 12:48] = triangles.astype("<f4").reshape(-1, 9).view(np.uint8).reshape(-1, 36)
    with path.open("wb") as stream:
        stream.write(b"CoACD collision piece".ljust(80, b" "))
        stream.write(struct.pack("<I", len(triangles)))
        stream.write(records.tobytes())


def reduce_mesh(mesh: trimesh.Trimesh) -> tuple[np.ndarray, np.ndarray]:
    """Merge only collision vertices that fall in the same 2-mm cell."""
    vertices = np.asarray(mesh.vertices, dtype=np.float64)
    faces = np.asarray(mesh.faces, dtype=np.int32)
    cells = np.rint(vertices / VOXEL_M).astype(np.int32)
    _, inverse = np.unique(cells, axis=0, return_inverse=True)
    reduced = np.zeros((inverse.max() + 1, 3), dtype=np.float64)
    counts = np.bincount(inverse, minlength=len(reduced)).astype(np.float64)
    np.add.at(reduced, inverse, vertices)
    reduced /= counts[:, None]
    faces = inverse[faces]
    keep = ((faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2]))
    return reduced, faces[keep].astype(np.int32)


def decompose(stem: str, overwrite: bool) -> None:
    source = SOURCE_DIR / f"{stem}.STL"
    if not source.is_file():
        raise FileNotFoundError(source)
    existing = sorted(OUTPUT_DIR.glob(f"{stem}_*.STL"))
    if existing and not overwrite:
        print(f"{stem}: use {len(existing)} existing collision pieces")
        return
    for stale in existing:
        stale.unlink()
    mesh = trimesh.load_mesh(source, process=False)
    if not isinstance(mesh, trimesh.Trimesh):
        mesh = mesh.dump(concatenate=True)
    mesh.merge_vertices(digits_vertex=6)
    vertices, faces = reduce_mesh(mesh)
    print(f"{stem}: {len(mesh.faces):,} CAD triangles -> {len(faces):,} collision triangles", flush=True)
    pieces = coacd.run_coacd(
        coacd.Mesh(vertices, faces),
        threshold=.06,
        max_convex_hull=16,
        preprocess_mode="auto",
        preprocess_resolution=40,
        resolution=250,
        mcts_nodes=6,
        mcts_iterations=20,
        mcts_max_depth=2,
        merge=True,
        max_ch_vertex=128,
        decimate=True,
        seed=7,
    )
    for index, (piece_vertices, piece_faces) in enumerate(pieces):
        write_binary_stl(OUTPUT_DIR / f"{stem}_{index:03d}.STL", piece_vertices, piece_faces)
    print(f"{stem}: wrote {len(pieces)} convex pieces", flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("parts", nargs="*", help="mesh stems; omit to process every non-base STL")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    parts = args.parts or [path.stem for path in sorted(SOURCE_DIR.glob("*.STL")) if path.stem != "base_link"]
    coacd.set_log_level("warn")
    for stem in parts:
        decompose(stem, args.overwrite)


if __name__ == "__main__":
    main()
