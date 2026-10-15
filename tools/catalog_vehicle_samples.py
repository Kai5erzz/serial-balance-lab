"""Read-only inventory of VOFA captures; filenames do not prove firmware identity."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def inspect(path: Path):
    with path.open() as file:
        rows=list(csv.reader(file))
    header=rows[0]
    data=np.array([[float(v) for v in row] for row in rows[1:] if row],dtype=float)
    intervals=np.diff(data[:,0])
    positive=intervals[intervals>0]
    guess="unidentified"
    if path.stem.startswith("bump") and data.shape[1]==32:
        guess="candidate Page54; verify source/version before dynamic comparison"
    elif path.stem.startswith("roll"):
        guess="candidate Page43; versions differ in source documentation"
    elif path.stem=="jump3":
        guess="Page62 v32 according to code_914/docs/jump_landing_phase_gate_v35_20261011.md"
    item={"path":str(path.resolve()),"sha256":hashlib.sha256(path.read_bytes()).hexdigest(),
          "rows":len(data),"columns":len(header),"time_start_s":float(data[0,0]),
          "time_end_s":float(data[-1,0]),"median_dt_ms":float(np.median(positive)*1000) if len(positive) else None,
          "max_gap_ms":float(max(positive)*1000) if len(positive) else None,
          "nonpositive_time_steps":int(np.count_nonzero(intervals<=0)),
          "nonfinite_values":int(np.count_nonzero(~np.isfinite(data))),"page_interpretation":guess,
          "exact_current_firmware_identity":False,
          "full_sensor_motor_input_replay":False}
    if guess.startswith("candidate Page54"):
        item["page54_candidate"]={"states":sorted(set(int(v) for v in data[:,2])),
                                  "faults":sorted(set(int(v) for v in data[:,3])),
                                  "length_min_max_m":[[float(min(data[:,i])),float(max(data[:,i]))] for i in (8,9)],
                                  "phi0_min_max_deg":[[float(np.degrees(min(data[:,i]))),float(np.degrees(max(data[:,i])))] for i in (12,13)]}
    return item


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--directory",type=Path,default=Path.home()/"sample")
    parser.add_argument("--output",type=Path,default=Path("out/native/sample_catalog.json"))
    args=parser.parse_args()
    result={"source_directory":str(args.directory.resolve()),
            "captures":[inspect(path) for path in sorted(args.directory.glob("*.csv"))],
            "replay_requirement":"1 ms timestamped raw/corrected IMU, four hip encoders/rates/torques, two wheel encoders/rates/currents, yaw encoder/IMU, RC and feedback availability; diagnostic pages alone omit inputs"}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n")
    print(f"catalogued {len(result['captures'])} captures: {args.output.resolve()}")
