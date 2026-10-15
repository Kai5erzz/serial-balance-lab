"""Validated physical encoder coordinates imported from jointpd."""
import hashlib
import json
from pathlib import Path
from .build import FIRMWARE


def load_calibration():
    root = FIRMWARE / "calibration"
    path = root / "encoder_zero_2026-09-13.json"
    raw = path.read_bytes()
    record = json.loads(raw)
    if record["encoder_coordinate"] != "continuous_e_equals_q_minus_znew" or record["encoder_to_cad_sign"] != [1,1,1,1]:
        raise ValueError("unsupported physical encoder coordinate contract")
    for name, expected in record["bundled_evidence_sha256"].items():
        if hashlib.sha256((root/name).read_bytes()).hexdigest() != expected:
            raise ValueError(f"encoder evidence differs: {name}")
    provenance = json.loads((root / "provenance.json").read_text())
    if hashlib.sha256(raw).hexdigest() != provenance["files"][path.name]:
        raise ValueError("encoder calibration differs from imported provenance")
    zeros = dict(zip(record["joint_order"], record["encoder_zero_cad_rad"]))
    return zeros, {"calibration_id": record["calibration_id"], "sha256": hashlib.sha256(raw).hexdigest(),
                   "source": provenance["source"], "coordinate": record["encoder_coordinate"]}
