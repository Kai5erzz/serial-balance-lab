"""Make an exact mesh-convex exterior collision asset for rotating round parts.

For a tyre, its convex exterior is the physical contact envelope: fine spoke
and hub cavities must not catch the ground.  This is intentionally used only
for the two wheels and the normally elevated gimbal cover; articulated leg
members use the multi-convex CoACD assets instead.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import struct
import sys
from pathlib import Path

import numpy as np
import trimesh


ROOT = REPO_ROOT
SOURCE_DIR = ROOT / "model" / "visual_meshes"
OUTPUT_DIR = ROOT / "model" / "collision_meshes_vhacd"


def write_binary_stl(path: Path, vertices: np.ndarray, faces: np.ndarray) -> None:
    triangles = np.asarray(vertices, dtype=np.float32)[np.asarray(faces, dtype=np.int32)]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    normals[lengths > 1e-12] /= lengths[lengths > 1e-12, None]
    records = np.zeros((len(triangles), 50), dtype=np.uint8)
    records[:, :12] = normals.astype("<f4").view(np.uint8).reshape(-1, 12)
    records[:, 12:48] = triangles.astype("<f4").reshape(-1, 9).view(np.uint8).reshape(-1, 36)
    with path.open("wb") as stream:
        stream.write(b"CAD convex exterior collision".ljust(80, b" "))
        stream.write(struct.pack("<I", len(triangles)))
        stream.write(records.tobytes())


def main() -> None:
    if len(sys.argv) < 2:
        raise SystemExit("usage: make_exterior_collision_mesh.py <mesh-stem> [...]")
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for stem in sys.argv[1:]:
        source = SOURCE_DIR / f"{stem}.STL"
        for stale in OUTPUT_DIR.glob(f"{stem}_*.STL"):
            stale.unlink()
        mesh = trimesh.load_mesh(source, process=False)
        if not isinstance(mesh, trimesh.Trimesh):
            mesh = mesh.dump(concatenate=True)
        mesh.merge_vertices(digits_vertex=6)
        hull = mesh.convex_hull
        write_binary_stl(OUTPUT_DIR / f"{stem}_000.STL", hull.vertices, hull.faces)
        print(f"{stem}: exterior hull {len(hull.faces):,} triangles")


if __name__ == "__main__":
    main()
