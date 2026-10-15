"""Bounded friction diagnosis of the shared vehicle C recovery chain.

Overrides are isolated experiment inputs, never edits to the adopted plant.
Contact-group experiments preserve baseline effective friction in the other
group despite MuJoCo's element-wise maximum material combination.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path
import sys

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from balance_lab.firmware import Command
from balance_lab.firmware.simulation import VehicleSimulation
from balance_lab.firmware.viewer import summary
from balance_lab.plant import VARIANTS

CASES = ("baseline", "wheel_mu_0.3", "wheel_mu_1.0", "wheel_mu_2.0", "nonwheel_mu_0.05",
         "nonwheel_mu_0.3", "nonwheel_mu_1.0", "all_contact_mu_0", "no_leg_coulomb",
         "no_leg_viscous", "leg_viscous_half", "leg_viscous_double", "no_wheel_viscous",
         "wheel_viscous_quarter", "wheel_viscous_fourfold", "wheel_viscous_nominal",
         "wheel_coulomb_0.09", "guide_mu_2.0", "nonwheel_mu_2.0", "no_joint_resistance")


def ids(model, kind):
    count = model.ngeom if kind == mujoco.mjtObj.mjOBJ_GEOM else model.njnt
    return [(i, mujoco.mj_id2name(model, kind, i)) for i in range(count)]


def parameters(sim):
    model = sim.model
    joints = {}
    for i, name in ids(model, mujoco.mjtObj.mjOBJ_JOINT):
        if model.jnt_type[i] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        dof = int(model.jnt_dofadr[i])
        joints[name] = {"damping_nms_rad": float(model.dof_damping[dof]),
                        "coulomb_nm": float(model.dof_frictionloss[dof]),
                        "armature_kg_m2": float(model.dof_armature[dof])}
    geoms = {}
    for i, name in ids(model, mujoco.mjtObj.mjOBJ_GEOM):
        if model.geom_contype[i] or model.geom_conaffinity[i]:
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, int(model.geom_bodyid[i]))
            name = name or f"{body_name}__geom_{i}"
            geoms[name] = {"friction": model.geom_friction[i].tolist(),
                           "condim": int(model.geom_condim[i]),
                           "priority": int(model.geom_priority[i]),
                           "body": body_name}
    return {"joints": joints, "geoms": geoms,
            "solver": {"algorithm": int(model.opt.solver), "cone": int(model.opt.cone),
                       "iterations": int(model.opt.iterations), "tolerance": float(model.opt.tolerance),
                       "timestep_s": float(model.opt.timestep)},
            "gas_tendon_stiffness": model.tendon_stiffness.tolist(),
            "gas_tendon_damping": model.tendon_damping.tolist(),
            "gas_tendon_frictionloss": model.tendon_frictionloss.tolist(),
            "passive_gas_forces_n": [float(row[2]) for row in sim.gas]}


def override(sim, case):
    model = sim.model
    before = parameters(sim)
    if "mu" in case:
        if model.npair or np.any(model.geom_priority):
            raise ValueError("review explicit contact pairs/priorities before changing group friction")
        wheel_mu, other_mu = 1.2, 1.2
        if case.startswith("wheel_mu_"): wheel_mu = float(case.rsplit("_", 1)[1])
        elif case.startswith("nonwheel_mu_"): other_mu = float(case.rsplit("_", 1)[1])
        elif case == "all_contact_mu_0": wheel_mu = other_mu = 0.0
        for i, name in ids(model, mujoco.mjtObj.mjOBJ_GEOM):
            if not (model.geom_contype[i] or model.geom_conaffinity[i]):
                continue
            body = int(model.geom_bodyid[i])
            body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
            # A neutral terrain coefficient lets each robot material select
            # its intended effective mu, without changing normal compliance.
            model.geom_friction[i, 0] = (0.0 if model.body_weldid[body] == 0 else
                                         wheel_mu if body_name in ("l_wheel", "r_wheel") else other_mu)
            if case.startswith("guide_mu_") and name in ("guide_wheel_contact", "recovery_skid"):
                model.geom_friction[i, 0] = float(case.rsplit("_", 1)[1])
    for i, name in ids(model, mujoco.mjtObj.mjOBJ_JOINT):
        if not name or model.jnt_type[i] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        dof = int(model.jnt_dofadr[i])
        wheel = name in ("l_wheel", "r_wheel")
        leg = name.startswith(("l_", "r_")) and not wheel
        if case == "no_joint_resistance" or (leg and case == "no_leg_coulomb"):
            model.dof_frictionloss[dof] = 0
        if case == "no_joint_resistance" or (leg and case == "no_leg_viscous") or (wheel and case == "no_wheel_viscous"):
            model.dof_damping[dof] = 0
        if leg and case in ("leg_viscous_half", "leg_viscous_double"):
            model.dof_damping[dof] *= .5 if case.endswith("half") else 2.0
        if wheel and case in ("wheel_viscous_quarter", "wheel_viscous_fourfold"):
            model.dof_damping[dof] *= .25 if case.endswith("quarter") else 4.0
        if wheel and case == "wheel_viscous_nominal":
            model.dof_damping[dof] = .08
        if wheel and case == "wheel_coulomb_0.09":
            model.dof_frictionloss[dof] = .09
    # Recompute the initial constraint/passive force cache after model edits.
    mujoco.mj_forward(model, sim.data)
    return {"case": case, "before": before, "after": parameters(sim)}


def contacts(sim, *, measure_slip=False):
    model, data = sim.model, sim.data
    result = {"left_n": 0.0, "right_n": 0.0, "other_n": 0.0,
              "wheel_tangent_n": 0.0, "other_tangent_n": 0.0}
    details = []
    wrench = np.empty(6)
    jacp, jacr = np.empty((3, model.nv)), np.empty((3, model.nv))
    for i in range(data.ncon):
        contact = data.contact[i]
        geoms = (int(contact.geom1), int(contact.geom2))
        bodies = [int(model.geom_bodyid[g]) for g in geoms]
        terrain = [b for b in bodies if model.body_weldid[b] == 0]
        if len(terrain) != 1:
            continue
        body = next(b for b in bodies if b not in terrain)
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        robot_geom = next(g for g in geoms if model.geom_bodyid[g] == body)
        geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, robot_geom) or f"{body_name}__geom_{robot_geom}"
        wheel = body_name in ("l_wheel", "r_wheel")
        mujoco.mj_contactForce(model, data, i, wrench)
        normal = abs(float(wrench[0]))
        if normal < 1e-6:
            continue
        tangent = float(np.linalg.norm(wrench[1:3]))
        result[{"l_wheel": "left_n", "r_wheel": "right_n"}.get(body_name, "other_n")] += normal
        result["wheel_tangent_n" if wheel else "other_tangent_n"] += tangent
        item = {"geom": geom_name, "wheel": wheel, "normal_n": normal,
                "tangent_n": tangent, "mu": float(contact.friction[0]), "dim": int(contact.dim),
                "cone_utilization": tangent / max(1e-9, float(contact.friction[0])*normal)}
        if measure_slip:
            mujoco.mj_jac(model, data, jacp, jacr, contact.pos, body)
            tangent_velocity = contact.frame.reshape(3, 3)[1:] @ (jacp @ data.qvel)
            item["slip_m_s"] = float(np.linalg.norm(tangent_velocity))
        details.append(item)
    result["details"] = details
    return result


def run(case, args):
    sim = VehicleSimulation(args.variant, plant=args.plant, pose=args.pose)
    settings = override(sim, case)
    path = args.output / (case + ".csv")
    phases, tail, contact_rows = {}, [], []
    transitions = []
    failure = None
    previous_state = None
    touched = {}
    sample_fields = ("time_s", "pitch_deg", "roll_deg", "recovery_state", "recovery_fault", "stand_state",
                     "length_l", "length_r", "phi0_l_deg", "phi0_r_deg", "current_l", "current_r",
                     "left_n", "right_n", "other_n", "wheel_tangent_n", "other_tangent_n",
                     "wheel_speed_l", "wheel_speed_r", "wheel_slip_max_m_s", "other_slip_max_m_s")
    try:
        with path.open("w", newline="") as file:
            writer = csv.DictWriter(file, fieldnames=sample_fields); writer.writeheader()
            for tick in range(round(args.seconds / sim.tick_s)):
                t = tick * sim.tick_s
                cmd = Command.remote(right_switch=1 if t < .05 else 3)
                try:
                    out = sim.step(cmd)
                except RuntimeError as error:
                    failure = str(error)
                    break
                state = (out.recovery_state, out.recovery_fault, out.stand_state)
                if state != previous_state:
                    transitions.append({"time_s": round((tick+1)*sim.tick_s, 6), "state_fault_stand": list(state)})
                    previous_state = state
                for key in ("recovery_state", "recovery_fault", "chassis_mode", "stand_state"):
                    phases.setdefault(key, set()).add(int(getattr(out, key)))
                sampled = (tick+1) % 10 == 0
                contact = contacts(sim, measure_slip=sampled)
                for item in contact["details"]:
                    entry = touched.setdefault(item["geom"], {"mu": set(), "dim": set(), "normal_peak_n": 0.0})
                    entry["mu"].add(item["mu"]); entry["dim"].add(item["dim"])
                    entry["normal_peak_n"] = max(entry["normal_peak_n"], item["normal_n"])
                if sampled:
                    contact_rows.append({"time_s": round((tick+1)*sim.tick_s, 6), "contacts": contact["details"]})
                    slips = lambda wheel: max([x.get("slip_m_s", 0) for x in contact["details"] if x["wheel"] == wheel], default=0.0)
                    writer.writerow(dict(zip(sample_fields, (
                        round((tick+1)*sim.tick_s, 6), *list(out.imu)[:2][::-1], *state,
                        *out.length, *np.degrees(list(out.phi0)), *out.wheel_current,
                        *[contact[k] for k in ("left_n", "right_n", "other_n", "wheel_tangent_n", "other_tangent_n")],
                        *sim.feedback.wheel_speed, slips(True), slips(False)))))
                if t >= args.seconds-1:
                    tail.append({"pitch": float(out.imu[1]), "roll": float(out.imu[0]),
                                 "phi0": np.degrees(list(out.phi0)), "stand": out.stand_state,
                                 "wheel_down": min(contact["left_n"], contact["right_n"]) > 1,
                                 "other": contact["other_n"],
                                 # This fixed diagnostic threshold is close to
                                 # M3508_TOR_MAX * M3508_TOR_TO_CUR. It is not
                                 # the active output limit in every mode.
                                 "current_ge_8190": max(abs(x) for x in out.wheel_current) >= 8190})
        metrics = None if failure else {"pitch_abs_max_deg": max(abs(x["pitch"]) for x in tail),
                   "roll_abs_max_deg": max(abs(x["roll"]) for x in tail),
                   "both_wheels_contact_fraction": np.mean([x["wheel_down"] for x in tail]).item(),
                   "nonwheel_contact_peak_n": max(x["other"] for x in tail),
                   "stand_ack_fraction": np.mean([x["stand"] in (2,5) for x in tail]).item(),
                   "phi0_deviation_max_deg": max(float(np.max(abs(x["phi0"]-90))) for x in tail),
                   "wheel_current_ge_8190_fraction": np.mean([x["current_ge_8190"] for x in tail]).item()}
        balanced = (not failure and metrics["pitch_abs_max_deg"] < 5 and metrics["roll_abs_max_deg"] < 5 and
                    metrics["both_wheels_contact_fraction"] > .95 and metrics["nonwheel_contact_peak_n"] < 1 and
                    metrics["stand_ack_fraction"] > .95 and metrics["phi0_deviation_max_deg"] < 20)
        details_path = args.output / (case + "_contacts.json")
        details_path.write_text(json.dumps(contact_rows, allow_nan=False) + "\n")
        for entry in touched.values():
            entry["mu"] = sorted(entry["mu"]); entry["dim"] = sorted(entry["dim"])
        return {"case": case, "finite": not failure, "failure": failure, "balanced_at_end": balanced, "metrics": metrics,
                "final": summary(sim), "metadata": sim.metadata(), "overrides": settings,
                "phases": {k: sorted(v) for k,v in phases.items()}, "transitions": transitions,
                "actual_contacts": touched, "trace_csv": str(path.resolve()), "contact_trace": str(details_path.resolve())}
    finally:
        sim.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=("fast", "full", "platform"), default="fast")
    parser.add_argument("--plant", default="identified_2026-09-21")
    parser.add_argument("--pose", choices=("stand", "encoder"), default="encoder")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--cases", nargs="+", choices=CASES, default=CASES)
    parser.add_argument("--output", type=Path, default=Path("out/native/friction_audit"))
    args = parser.parse_args()
    if args.seconds < 3: parser.error("use at least 3 s so the last 1 s is separated from startup")
    args.output.mkdir(parents=True, exist_ok=True)
    report = {"purpose": "friction sensitivity; extreme cases are diagnosis only, not accepted real-vehicle parameters",
              "duration_s": args.seconds, "pose": args.pose, "variant": args.variant,
              "mujoco_version": mujoco.__version__,
              "model_sha256": hashlib.sha256(Path(VARIANTS[args.variant]).read_bytes()).hexdigest(),
              "adapter_sha256": hashlib.sha256((ROOT/'balance_lab/firmware/simulation.py').read_bytes()).hexdigest(),
              "probe_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "results": []}
    for case in args.cases:
        result = run(case, args); report["results"].append(result)
        (args.output/"report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        if result["failure"]:
            print(case, "numerical failure", result["failure"], flush=True)
        else:
            print(case, "balanced", result["balanced_at_end"], "pitch", round(result["metrics"]["pitch_abs_max_deg"], 2),
                  "wheel down", result["metrics"]["both_wheels_contact_fraction"],
                  "faults", result["phases"]["recovery_fault"], flush=True)


if __name__ == "__main__":
    main()
