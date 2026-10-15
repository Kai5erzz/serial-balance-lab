"""Isolate wheel actuation and feedback signs at the real C LQR handoff.

All sign changes are explicit counterfactuals in local simulation instances;
the shared firmware, default adapter and physical parameters remain unchanged.
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
from balance_lab.firmware.simulation import VehicleSimulation, WHEEL_NAMES
from balance_lab.firmware.viewer import summary
from tools.probes.native_friction_audit import contacts

CASES = ("baseline", "torque_neg", "feedback_neg", "both_neg", "wheel_zero", "both_neg_from_start",
         "both_swap_from_start")


class WheelHandoffSimulation(VehicleSimulation):
    def __init__(self, *args, wheel_case="baseline", **kwargs):
        if wheel_case not in CASES:
            raise ValueError(wheel_case)
        self.wheel_case = wheel_case
        self.wheel_sign_active = wheel_case.endswith("from_start")
        self.first_handoff_s = None
        super().__init__(*args, **kwargs)

    def read_feedback(self):
        f = super().read_feedback()
        if self.wheel_sign_active and self.wheel_case in ("feedback_neg", "both_neg", "both_neg_from_start"):
            for field in (f.wheel_angle, f.wheel_speed, f.wheel_torque):
                for i in range(2): field[i] = -field[i]
        elif self.wheel_case == "both_swap_from_start":
            for field in (f.wheel_angle, f.wheel_speed, f.wheel_torque):
                field[0], field[1] = field[1], field[0]
        return f

    def apply_outputs(self, output, feedback):
        if output.chassis_mode == 3 and output.recovery_state == 7:
            if self.first_handoff_s is None:
                self.first_handoff_s = (self.ticks+1)*self.tick_s
            self.wheel_sign_active = True
        super().apply_outputs(output, feedback)
        if self.wheel_sign_active:
            for name in WHEEL_NAMES:
                index = self.actuators[name]
                if self.wheel_case in ("torque_neg", "both_neg", "both_neg_from_start"):
                    self.data.ctrl[index] *= -1
                elif self.wheel_case == "wheel_zero":
                    self.data.ctrl[index] = 0
            if self.wheel_case == "both_swap_from_start":
                left,right=[self.actuators[n] for n in WHEEL_NAMES]
                self.data.ctrl[left],self.data.ctrl[right]=self.data.ctrl[right],self.data.ctrl[left]

    def metadata(self):
        result = super().metadata()
        result["wheel_counterfactual"] = self.wheel_case
        result["wheel_counterfactual_scope"] = "all ticks" if self.wheel_case.endswith("from_start") else "latched at first real C RECOVERY state 7; feedback change starts next tick"
        return result


def run(case, variant, directory, seconds=5, scenario="zero"):
    sim = WheelHandoffSimulation(variant, pose="encoder", wheel_case=case)
    tail, transitions, previous, failure = [], [], None, None
    prefix = hashlib.sha256()
    fields = ("time_s", "pitch_deg", "roll_deg", "state", "fault", "stand", "length_l", "length_r",
              "phi_l_deg", "phi_r_deg", "current_l", "current_r", "applied_torque_l", "applied_torque_r",
              "raw_speed_l", "raw_speed_r", "c_speed_l", "c_speed_r", "lqr_wheel_l", "lqr_wheel_r",
              "lqr_hip_l", "lqr_hip_r", "support_l", "support_r", "vx_obs", "phi_obs", "dphi_obs",
              "wheel_contact_l_n", "wheel_contact_r_n", "nonwheel_contact_n", "yaw_relative_deg", "yaw_rate_rad_s")
    path = directory/(case+".csv")
    try:
        with path.open("w", newline="") as stream:
            writer=csv.writer(stream); writer.writerow(fields)
            for _ in range(round(seconds/sim.tick_s)):
                t=sim.ticks*sim.tick_s
                vx=(60 if scenario=="drive_pos" else -60 if scenario=="drive_neg" else 0) if t>=5 else 0
                yaw=(30 if scenario=="yaw_pos" else -30 if scenario=="yaw_neg" else 0) if t>=5 else 0
                try: out=sim.step(Command.remote(right_switch=1 if t<.05 else 3,channels=(0,vx,yaw,0,0)))
                except RuntimeError as exc:
                    failure=str(exc); break
                elapsed=sim.ticks*sim.tick_s
                if sim.first_handoff_s is None:
                    prefix.update(sim.data.qpos.tobytes()); prefix.update(sim.data.qvel.tobytes())
                state=(out.recovery_state,out.recovery_fault,out.stand_state)
                if state!=previous:
                    transitions.append({"time_s":elapsed,"state_fault_stand":state}); previous=state
                c=contacts(sim)
                if sim.ticks%10==0:
                    writer.writerow((elapsed,out.imu[1],out.imu[0],*state,*out.length,*np.degrees(list(out.phi0)),
                        *out.wheel_current,*[sim.data.ctrl[sim.actuators[n]] for n in WHEEL_NAMES],
                        *[sim.data.qvel[sim.dadr[n]] for n in WHEEL_NAMES],*sim.feedback.wheel_speed,
                        out.lqr[0],out.lqr[2],out.lqr[1],out.lqr[3],*out.support,out.obs[3],out.obs[4],out.obs[5],c['left_n'],c['right_n'],c['other_n'],np.degrees(out.yaw_relative),out.imu[5]))
                if elapsed>seconds-1:
                    tail.append((float(out.imu[1]),float(out.imu[0]),min(c['left_n'],c['right_n'])>1,
                                 c['other_n'],out.stand_state in (2,5),max(abs(np.degrees(list(out.phi0))-90)),
                                 float(np.degrees(out.yaw_relative)),float(out.imu[5])))
        metrics=None if failure else {
            "pitch_abs_max_deg":max(abs(x[0]) for x in tail),"roll_abs_max_deg":max(abs(x[1]) for x in tail),
            "both_wheels_contact_fraction":float(np.mean([x[2] for x in tail])),
            "nonwheel_contact_peak_n":max(x[3] for x in tail),"stand_ack_fraction":float(np.mean([x[4] for x in tail])),
            "phi0_deviation_max_deg":max(x[5] for x in tail),
            "yaw_relative_abs_max_deg":max(abs(x[6]) for x in tail),"yaw_rate_abs_max_rad_s":max(abs(x[7]) for x in tail)}
        passed=bool(not failure and metrics['pitch_abs_max_deg']<5 and metrics['roll_abs_max_deg']<5 and
                    metrics['both_wheels_contact_fraction']>.95 and metrics['nonwheel_contact_peak_n']<1 and
                    metrics['stand_ack_fraction']>.95 and metrics['phi0_deviation_max_deg']<20)
        return {"case":case,"variant":variant,"scenario":scenario,"failure":failure,"physically_balanced":passed,"metrics":metrics,
                "first_handoff_s":sim.first_handoff_s,"physical_prefix_sha256":prefix.hexdigest(),
                "final":summary(sim),"metadata":sim.metadata(),"transitions":transitions,"trace_csv":str(path.resolve())}
    finally: sim.close()


def local_wheel_response(variant):
    """Compare wheel common +/- torque at identical handoff physics state.

    Re-evaluate accelerations using the same contacts, inertia and all other
    actuator/passive forces. This does not integrate or change C history.
    """
    sim=VehicleSimulation(variant,pose="encoder")
    try:
        while sim.ticks<2000:
            t=sim.ticks*sim.tick_s
            out=sim.step(Command.remote(right_switch=1 if t<.05 else 3))
            if out.recovery_state==7: break
        free=sim._id(mujoco.mjtObj.mjOBJ_JOINT,"base_free")
        va=int(sim.model.jnt_dofadr[free])
        controls=sim.data.ctrl.copy()
        rows=[]
        for torque in (0,.1,-.1):
            sim.data.ctrl[:]=controls
            for name,sign in zip(WHEEL_NAMES,(-1,1)):
                sim.data.ctrl[sim.actuators[name]]=sign*torque
            mujoco.mj_forward(sim.model,sim.data)
            sensor_accel=sim.sensor_body.T@sim.data.qacc[va+3:va+6]
            frame=sim.data.xmat[sim.base].reshape(3,3)@sim.sensor_body
            rows.append({"logical_forward_common_nm":torque,"cad_torque_nm":[-torque,torque],
                         "pitch_accel_rad_s2":float(sensor_accel[0]),
                         "forward_base_accel_m_s2":float(np.dot(frame[:,1],sim.data.qacc[va:va+3]))})
        return {"time_s":sim.ticks*sim.tick_s,"c_pitch_deg":float(out.imu[1]),
                "c_observation":list(out.obs),"c_lqr_output":list(out.lqr),
                "c_packet_current":list(out.wheel_current),"wheel_response":rows,
                "interpretation":"near upright pitch rate is raw IMU X; free-joint rotational accelerations are expressed in body coordinates"}
    finally: sim.close()


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--variant",choices=("fast","full"),default="fast")
    p.add_argument("--cases",choices=CASES,nargs="+",default=CASES)
    p.add_argument("--seconds",type=float,default=5)
    p.add_argument("--scenario",choices=("zero","drive_pos","drive_neg","yaw_pos","yaw_neg"),default="zero")
    p.add_argument("--output",type=Path,default=Path("out/native/wheel_handoff_audit/fast"))
    args=p.parse_args(); args.output.mkdir(parents=True,exist_ok=True)
    if not 3<=args.seconds<=20: p.error("diagnostic duration must be between 3 and 20 s")
    report={"purpose":"test wheel polarity at handoff without changing shared firmware or defaults",
            "probe_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "duration_s":args.seconds,"tail_window_s":[args.seconds-1,args.seconds],"local_wheel_response":local_wheel_response(args.variant),"results":[]}
    for case in args.cases:
        row=run(case,args.variant,args.output,args.seconds,args.scenario); report['results'].append(row)
        (args.output/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n')
        print(case,'balanced',row['physically_balanced'],'handoff',row['first_handoff_s'],
              'metrics',row['metrics'],'failure',row['failure'],flush=True)


if __name__=="__main__": main()
