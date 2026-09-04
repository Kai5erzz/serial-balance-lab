"""Estimate gimbal_yaw inertia from its STL mesh (uniform density, known mass).

Validates the principal-axis frame against the preserved inertial quat in the
damaged model, then prints the three principal inertias.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import struct
from pathlib import Path

import numpy as np

STL = Path(r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\meshes\gimbal_yaw.STL")
KNOWN_MASS = 2.41015
INERTIAL_QUAT = np.array([0.615109, 0.334013, 0.335399, 0.630542])


def read_stl(path: Path) -> np.ndarray:
    data = path.read_bytes()
    if data[:5] == b"solid":
        raise SystemExit("ASCII STL not supported; convert to binary")
    n = struct.unpack_from("<I", data, 80)[0]
    tris = []
    for i in range(n):
        off = 84 + i * 50
        normal = np.frombuffer(data, dtype="<f4", count=3, offset=off)
        verts = np.frombuffer(data, dtype="<f4", count=9, offset=off + 12).reshape(3, 3)
        tris.append(verts)
    return np.stack(tris)


def mesh_properties(tris: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Volume, centroid, inertia tensor about origin via 4-point Gauss
    quadrature over tetrahedra (triangle + origin), density = 1."""
    a = tris[:, 0]
    b = tris[:, 1]
    c = tris[:, 2]
    det = np.einsum("ij,ij->i", a, np.cross(b, c))
    vol6 = np.abs(det)
    vol = vol6.sum() / 6.0
    # 4-point rule, barycentric coords
    alpha = 0.5854101966249685
    beta = 0.1381966011250105
    pts = (
        np.stack([np.full_like(a, alpha), np.full_like(a, beta), np.full_like(a, beta), np.full_like(a, beta)], 0) * a[None]
        + np.stack([np.full_like(b, beta), np.full_like(b, alpha), np.full_like(b, beta), np.full_like(b, beta)], 0) * b[None]
        + np.stack([np.full_like(c, beta), np.full_like(c, beta), np.full_like(c, alpha), np.full_like(c, beta)], 0) * c[None]
        + np.stack([np.full_like(a, beta), np.full_like(a, beta), np.full_like(a, beta), np.full_like(a, alpha)], 0) * 0.0
    )  # shape (4, N, 3); last term is the origin vertex -> zero
    w = vol6 / 24.0  # (N,): each of 4 quadrature points carries V/4 = vol6/6/4
    com = np.zeros(3)
    i0 = np.zeros((3, 3))
    for k in range(4):
        p = pts[k]
        r2 = np.einsum("ij,ij->i", p, p)
        com += (w[:, None] * p).sum(axis=0)
        i0 += np.einsum("i,ij,ik->jk", w, p, p)  # r r^T
        for d in range(3):
            i0[d, d] -= np.sum(w * r2)
    i0 = -i0  # f = (r.r) I - r r^T  -> negative of the sums above
    com /= vol
    return vol, com, i0


def quat_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def main() -> None:
    tris = read_stl(STL)
    vol, com, i0 = mesh_properties(tris)
    density = KNOWN_MASS / vol
    i0 = i0 * density
    r = com
    ic = i0 - KNOWN_MASS * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    eigvals, eigvecs = np.linalg.eigh(ic)
    print(f"mesh: {len(tris)} tris, volume={vol:.6e} m^3, density={density:.1f} kg/m^3")
    print(f"com (mesh frame): {com}")
    print(f"inertia about com (mesh frame):\n{ic}")
    print(f"principal inertias: {eigvals}")
    print(f"principal axes (columns):\n{eigvecs}")
    # compare with the preserved inertial quat frame
    R = quat_rot(INERTIAL_QUAT)
    print(f"inertial-quat axes (columns):\n{R}")
    # inertia expressed in the inertial-quat frame (should be ~diagonal if quat matches mesh)
    diag = np.diag(R.T @ ic @ R)
    off = (R.T @ ic @ R) - np.diag(diag)
    print(f"inertia in quat frame diagonal: {diag}")
    print(f"off-diagonal magnitude in quat frame: {np.abs(off).max():.3e}")


if __name__ == "__main__":
    main()
