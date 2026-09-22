"""Regression guard for the CAD inertias and the two wheel overrides.

Two guards with different environment requirements:

- Always runnable: the committed ``model/whole_body_lqr.xml`` and
  ``model/whole_body_lqr_fast.xml`` must carry identical masses/inertias,
  only ``l_wheel``/``r_wheel`` may use the wheel inertia override, and the
  total mass must stay 12.827403 kg (12.801 kg plus the 2026-09-18 left-leg
  mirror fix: the URDF export had dropped parts on ``l_j1`` (-23.31 g) and
  ``l_leg`` (-3.09 g); rl-serial-balance physical_fk_2026-09-18_v2 carries
  the same correction, ported here together with the identified profile).
- Windows-CAD-only: URDF/CAD-CSV cross-validation against the original
  chassis export (``D:\\User\\balance_infantry\\...``). When those files
  are absent the guard reports SKIP instead of failing -- the committed
  XMLs are the only inertia source of truth on this machine.
"""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "tools" / "model"))

import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from extract_log_inertia import INERTIA_FIELDS, load_export_inertias

SOURCE_URDF = Path(
    r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\urdf\whole_body.urdf"
)
NATIVE_URDF = SOURCE_URDF.parent.parent / "mujoco_whole_body" / "whole_body_native.urdf"
CLOSED_MJCF = NATIVE_URDF.with_name("whole_body_closed.xml")
FULL_MJCF = REPO_ROOT / "model" / "whole_body_lqr.xml"
FAST_MJCF = REPO_ROOT / "model" / "whole_body_lqr_fast.xml"
WHEELS = {"l_wheel", "r_wheel"}
WHEEL_MASS = 0.577
WHEEL_URDF_INERTIA = np.array((0.0009705, 0.0, 0.0, 0.0009705, 0.0, 0.001941))
WHEEL_PRINCIPAL_INERTIA = np.array((0.001941, 0.0009705, 0.0009705))


def validate_urdf(path: Path) -> None:
    cad = load_export_inertias()
    tree = ET.parse(path)
    links = {link.get("name"): link for link in tree.getroot().findall("link")}
    assert set(links) == set(cad), f"{path.name}: link set differs from CAD CSV"

    for name, link in links.items():
        inertial = link.find("inertial")
        assert inertial is not None
        mass_node = inertial.find("mass")
        inertia_node = inertial.find("inertia")
        assert mass_node is not None and inertia_node is not None
        mass = float(mass_node.get("value"))
        inertia = np.array([float(inertia_node.get(field)) for field in INERTIA_FIELDS])
        if name in WHEELS:
            np.testing.assert_allclose(mass, WHEEL_MASS, rtol=0.0, atol=1e-12)
            np.testing.assert_allclose(inertia, WHEEL_URDF_INERTIA, rtol=0.0, atol=1e-12)
        else:
            np.testing.assert_allclose(mass, cad[name]["mass"], rtol=0.0, atol=1e-12)
            expected = np.array([cad[name][field] for field in INERTIA_FIELDS])
            np.testing.assert_allclose(inertia, expected, rtol=0.0, atol=1e-12)


def body_data(model: mujoco.MjModel) -> dict[str, tuple[float, np.ndarray]]:
    result = {}
    for body_id in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if name:
            result[name] = (float(model.body_mass[body_id]), model.body_inertia[body_id].copy())
    return result


def validate_mjcf() -> None:
    full = body_data(mujoco.MjModel.from_xml_path(str(FULL_MJCF)))
    fast = body_data(mujoco.MjModel.from_xml_path(str(FAST_MJCF)))

    assert set(full) == set(fast), "full/fast body sets differ"
    for name in full:
        np.testing.assert_allclose(full[name][0], fast[name][0], rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(full[name][1], fast[name][1], rtol=0.0, atol=1e-12)

    if CLOSED_MJCF.is_file():
        closed = body_data(mujoco.MjModel.from_xml_path(str(CLOSED_MJCF)))
        # l_j1 / l_leg now carry the 2026-09-18 left-leg mirror fix and
        # intentionally differ from the raw Windows CAD/URDF export.
        mirror_fix = {"l_j1", "l_leg"}
        for name in set(closed) & set(full) - {"base_link"} - mirror_fix:
            np.testing.assert_allclose(closed[name][0], full[name][0], rtol=2e-5, atol=1e-12)
            np.testing.assert_allclose(closed[name][1], full[name][1], rtol=2e-5, atol=1e-12)
    else:
        print("SKIP: closed-chain MJCF (Windows CAD export) not present; "
              "full/fast comparison only.")

    matching_wheel_signature = set()
    for name, (mass, inertia) in full.items():
        if np.allclose(inertia, WHEEL_PRINCIPAL_INERTIA, rtol=0.0, atol=1e-12):
            matching_wheel_signature.add(name)
        if name in WHEELS:
            np.testing.assert_allclose(mass, WHEEL_MASS, rtol=0.0, atol=1e-12)
    assert matching_wheel_signature == WHEELS, (
        f"wheel inertia leaked to non-wheel bodies: {sorted(matching_wheel_signature - WHEELS)}"
    )
    np.testing.assert_allclose(sum(value[0] for value in full.values()), 12.827403, atol=1e-9)


def main() -> None:
    ran_urdf = False
    for urdf in (SOURCE_URDF, NATIVE_URDF):
        if urdf.is_file():
            validate_urdf(urdf)
            ran_urdf = True
    if not ran_urdf:
        print("SKIP: Windows CAD export not present; URDF cross-validation skipped.")
    validate_mjcf()
    print("PASS: full/fast rigid-body masses and inertias match; total mass=12.827403 kg.")
    print("PASS: only l_wheel/r_wheel use the wheel inertia override.")


if __name__ == "__main__":
    main()
