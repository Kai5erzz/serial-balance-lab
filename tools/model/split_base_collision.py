"""Split the chassis STL into its real connected CAD components for contact.

The base export is a single STL file but contains thousands of disconnected
parts.  Feeding it to MuJoCo as one dynamic mesh creates one huge convex hull
that bridges empty space below the chassis.  Keeping its substantial connected
components separate preserves the actual outer envelope without replacing it
with boxes or capsules.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import trimesh


ROOT = REPO_ROOT
SOURCE = Path(r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\meshes\base_link.STL")
OUTPUT = ROOT / "model" / "collision_meshes"
MIN_FACES = 1_000


def main() -> None:
    mesh = trimesh.load_mesh(SOURCE, process=False)
    mesh.merge_vertices(digits_vertex=6)
    components = mesh.split(only_watertight=False)
    selected = [component for component in components if len(component.faces) >= MIN_FACES]
    selected.sort(key=lambda component: len(component.faces), reverse=True)
    print(f"base STL: {len(mesh.faces):,} faces; collision components: {len(selected)} (>= {MIN_FACES:,} faces)")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for stale in OUTPUT.glob("base_link_collision_*.STL"):
        stale.unlink()
    for index, component in enumerate(selected):
        destination = OUTPUT / f"base_link_collision_{index:03d}.STL"
        component.export(destination, file_type="stl")
    print(f"wrote {len(selected)} independent chassis collision meshes")


if __name__ == "__main__":
    main()
