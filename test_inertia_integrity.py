"""Regression guard for the CAD inertias and the two wheel overrides."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from extract_log_inertia import INERTIA_FIELDS, load_export_inertias


PROJECT = Path(__file__).resolve().parent
SOURCE_URDF = Path(
    r"D:\User\balance_infantry\rl_balance\chassis_export\whole_body\urdf\whole_body.urdf"
)
NATIVE_URDF = SOURCE_URDF.parent.parent / "mujoco_whole_body" / "whole_body_native.urdf"
CLOSED_MJCF = NATIVE_URDF.with_name("whole_body_closed.xml")
FULL_MJCF = PROJECT / "model" / "whole_body_lqr.xml"
FAST_MJCF = PROJECT / "model" / "whole_body_lqr_fast.xml"
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
    closed = body_data(mujoco.MjModel.from_xml_path(str(CLOSED_MJCF)))
    full = body_data(mujoco.MjModel.from_xml_path(str(FULL_MJCF)))
    fast = body_data(mujoco.MjModel.from_xml_path(str(FAST_MJCF)))

    assert set(full) == set(fast), "full/fast body sets differ"
    for name in full:
        np.testing.assert_allclose(full[name][0], fast[name][0], rtol=0.0, atol=1e-12)
        np.testing.assert_allclose(full[name][1], fast[name][1], rtol=0.0, atol=1e-12)

    for name in set(closed) & set(full) - {"base_link"}:
        np.testing.assert_allclose(closed[name][0], full[name][0], rtol=2e-5, atol=1e-12)
        np.testing.assert_allclose(closed[name][1], full[name][1], rtol=2e-5, atol=1e-12)

    matching_wheel_signature = set()
    for name, (mass, inertia) in full.items():
        if np.allclose(inertia, WHEEL_PRINCIPAL_INERTIA, rtol=0.0, atol=1e-12):
            matching_wheel_signature.add(name)
        if name in WHEELS:
            np.testing.assert_allclose(mass, WHEEL_MASS, rtol=0.0, atol=1e-12)
    assert matching_wheel_signature == WHEELS, (
        f"wheel inertia leaked to non-wheel bodies: {sorted(matching_wheel_signature - WHEELS)}"
    )
    np.testing.assert_allclose(sum(value[0] for value in full.values()), 12.801, atol=1e-9)


def main() -> None:
    validate_urdf(SOURCE_URDF)
    validate_urdf(NATIVE_URDF)
    validate_mjcf()
    print("PASS: 22 CAD inertias restored; only l_wheel/r_wheel use the wheel override.")
    print("PASS: full/fast rigid-body masses and inertias match; total mass=12.801 kg.")


if __name__ == "__main__":
    main()
