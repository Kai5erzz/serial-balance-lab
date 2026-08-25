"""Generate high-fidelity convex collision pieces for the chassis STL.

MuJoCo treats every dynamic mesh as one convex hull.  The chassis is strongly
concave, so its visual STL falsely touched the floor through empty space.  A
CoACD decomposition keeps the original visual mesh untouched and gives the
physics model a union of convex pieces instead.
"""

from __future__ import annotations

from pathlib import Path
import struct

import coacd
import numpy as np


ROOT = Path(__file__).resolve().parent
SOURCE = Path(r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\meshes\base_link.STL")
OUTPUT = ROOT / "model" / "collision_meshes"
PREPROCESS_VOXEL_M = .008


def read_binary_stl(path: Path) -> tuple[np.ndarray, np.ndarray]:
    with path.open("rb") as stream:
        stream.seek(80)
        faces = struct.unpack("<I", stream.read(4))[0]
        raw = np.frombuffer(stream.read(50 * faces), dtype=np.uint8).reshape(faces, 50)
    # Each STL facet is [normal:12][v0:12][v1:12][v2:12][attribute:2].
    vertices = raw[:, 12:48].copy().reshape(faces * 3, 12).view("<f4").reshape(-1, 3).astype(np.float64)
    indices = np.arange(vertices.shape[0], dtype=np.int32).reshape(-1, 3)
    return vertices, indices


def write_binary_stl(path: Path, vertices: np.ndarray, indices: np.ndarray) -> None:
    triangles = np.asarray(vertices, dtype=np.float32)[np.asarray(indices, dtype=np.int32)]
    normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    lengths = np.linalg.norm(normals, axis=1)
    normals[lengths > 1e-12] /= lengths[lengths > 1e-12, None]
    records = np.zeros((triangles.shape[0], 50), dtype=np.uint8)
    records[:, :12] = normals.astype("<f4").view(np.uint8).reshape(-1, 12)
    records[:, 12:48] = triangles.astype("<f4").reshape(-1, 9).view(np.uint8).reshape(-1, 36)
    with path.open("wb") as stream:
        stream.write(b"CoACD chassis collision".ljust(80, b" "))
        stream.write(struct.pack("<I", triangles.shape[0]))
        stream.write(records.tobytes())


def voxel_reduce(vertices: np.ndarray, indices: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Merge sub-3-mm tessellation detail before convex decomposition.

    This affects collision only; the untouched visual STL remains at its
    exported resolution.  It avoids spending many minutes on CAD tessellation
    that is well below the contact-model precision.
    """
    cells = np.rint(vertices / PREPROCESS_VOXEL_M).astype(np.int32)
    unique_cells, inverse = np.unique(cells, axis=0, return_inverse=True)
    reduced_vertices = np.zeros((unique_cells.shape[0], 3), dtype=np.float64)
    counts = np.bincount(inverse, minlength=unique_cells.shape[0]).astype(np.float64)
    np.add.at(reduced_vertices, inverse, vertices)
    reduced_vertices /= counts[:, None]
    reduced_indices = inverse[indices]
    nondegenerate = ((reduced_indices[:, 0] != reduced_indices[:, 1]) &
                     (reduced_indices[:, 1] != reduced_indices[:, 2]) &
                     (reduced_indices[:, 0] != reduced_indices[:, 2]))
    return reduced_vertices, reduced_indices[nondegenerate].astype(np.int32)


def main() -> None:
    vertices, indices = read_binary_stl(SOURCE)
    print(f"input: {indices.shape[0]:,} triangles", flush=True)
    vertices, indices = voxel_reduce(vertices, indices)
    print(f"preprocessed: {vertices.shape[0]:,} vertices, {indices.shape[0]:,} triangles at {PREPROCESS_VOXEL_M * 1e3:.1f} mm", flush=True)
    coacd.set_log_level("warn")
    pieces = coacd.run_coacd(
        coacd.Mesh(vertices, indices),
        threshold=.07,
        max_convex_hull=12,
        preprocess_mode="auto",
        preprocess_resolution=50,
        resolution=400,
        mcts_nodes=8,
        mcts_iterations=40,
        mcts_max_depth=2,
        merge=True,
        max_ch_vertex=256,
        decimate=True,
        seed=7,
    )
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for stale in OUTPUT.glob("base_link_collision_*.STL"):
        stale.unlink()
    for index, piece in enumerate(pieces):
        destination = OUTPUT / f"base_link_collision_{index:03d}.STL"
        write_binary_stl(destination, piece.vertices, piece.indices)
    print(f"wrote {len(pieces)} convex collision pieces to {OUTPUT}", flush=True)


if __name__ == "__main__":
    main()
