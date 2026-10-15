"""Bounded physical trials; a failed vehicle recovery is reported as a failure."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
import numpy as np
import mujoco
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from balance_lab.firmware import Command
from balance_lab.firmware.simulation import VehicleSimulation
from balance_lab.firmware.recording import Recorder
from balance_lab.firmware.viewer import summary


def contact_forces(sim):
    wheel_names={"l_wheel","r_wheel"}
    wheels={name:0.0 for name in wheel_names}
    other=0.0
    forces=np.zeros(6)
    for i in range(sim.data.ncon):
        contact=sim.data.contact[i]
        bodies=[int(sim.model.geom_bodyid[gid]) for gid in (contact.geom1,contact.geom2)]
        terrain=[b for b in bodies if int(sim.model.body_weldid[b])==0]
        if len(terrain)!=1:
            continue
        body=next(b for b in bodies if b not in terrain)
        name=mujoco.mj_id2name(sim.model,mujoco.mjtObj.mjOBJ_BODY,body)
        mujoco.mj_contactForce(sim.model,sim.data,i,forces)
        normal=abs(float(forces[0]))
        if name in wheels: wheels[name]+=normal
        else: other+=normal
    return wheels,other


def trial(name, args):
    pose="encoder" if name=="recovery" else "stand"
    sim=VehicleSimulation(args.variant,plant=args.plant,pose=pose)
    recorder=Recorder(args.output/(name+".jsonl"),sim.metadata(),pages=(39,43,47,54,55,57,62,64))
    phases={key:set() for key in ("chassis_mode","recovery_state","recovery_fault","jump_state","jump_fault","bump_state")}
    tail=[]
    try:
        for tick in range(round(args.seconds/sim.tick_s)):
            t=tick*sim.tick_s
            # Startup/relax is an actual RC command, never a controller-state
            # override. Requests subsequently use the real switch/wheel gates.
            right=1 if t<.050 else (2 if name=="spin" else 3)
            if name=="relax": right=1
            yaw=100 if name=="yaw_follow" and t>=1 else 0
            vx=200 if name=="drive" and t>=1 else 0
            wheel=660 if name=="jump" and 2<=t<2.050 else 0
            left=1 if name=="bump" and t>=2 else 2
            cmd=Command.remote(channels=(0,vx,yaw,0,wheel),right_switch=right,left_switch=left)
            out=sim.step(cmd);recorder.tick(sim,cmd)
            for key in phases: phases[key].add(int(getattr(out,key)))
            if t>=args.seconds-1:
                wheels,other=contact_forces(sim)
                tail.append({"pitch":float(out.imu[1]),"roll":float(out.imu[0]),
                             "phi0":np.degrees(list(out.phi0)),"stand":out.stand_state,
                             "wheels_down":min(wheels.values())>1,"other_contact_n":other})
        metrics={"tail_pitch_abs_max_deg":max(abs(x["pitch"]) for x in tail),
                 "tail_roll_abs_max_deg":max(abs(x["roll"]) for x in tail),
                 "tail_both_wheels_contact_fraction":sum(x["wheels_down"] for x in tail)/len(tail),
                 "tail_nonwheel_contact_max_n":max(x["other_contact_n"] for x in tail),
                 "tail_stand_ack_fraction":sum(x["stand"] in (2,5) for x in tail)/len(tail),
                 "tail_phi0_deviation_max_deg":max(float(max(abs(x["phi0"]-90))) for x in tail)}
        balanced=(metrics["tail_pitch_abs_max_deg"]<5 and metrics["tail_roll_abs_max_deg"]<5 and
                  metrics["tail_both_wheels_contact_fraction"]>.95 and metrics["tail_nonwheel_contact_max_n"]<1 and
                  metrics["tail_stand_ack_fraction"]>.95 and metrics["tail_phi0_deviation_max_deg"]<20)
        return {"scenario":name,"finite":True,"balanced_at_end":balanced if name!="relax" else None,
                "metrics":metrics,"phases_seen":{k:sorted(v) for k,v in phases.items()},"final":summary(sim),
                "metadata":sim.metadata(),"recording":str(recorder.path.resolve())}
    finally:
        recorder.close();sim.close()


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=Path("out/native/physical_trials"))
    parser.add_argument("--plant",default="identified_2026-09-21")
    parser.add_argument("--variant",default="fast",choices=("fast","full","platform"))
    parser.add_argument("--seconds",type=float,default=5)
    parser.add_argument("--scenarios",nargs="*",default=("relax","standing_start","recovery","drive","yaw_follow","spin","jump","bump"))
    args=parser.parse_args()
    if args.seconds<3: parser.error("trials require at least 3 seconds")
    args.output.mkdir(parents=True,exist_ok=True)
    results=[]
    for name in args.scenarios:
        result=trial(name,args);results.append(result)
        print(name,"finite",result["finite"],"balanced",result["balanced_at_end"],"recovery faults",result["phases_seen"]["recovery_fault"],flush=True)
    report={"duration_s":args.seconds,"results":results,
            "acceptance":"last-second pitch/roll <5 deg; both wheels >1 N on >95% ticks; all nonwheel contacts <1 N; stand/jump-complete ack >95%; phi0 within 20 deg of upright",
            "one_to_one_physical_validation_complete":False}
    (args.output/"report.json").write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+"\n")
