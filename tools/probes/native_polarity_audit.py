"""Audit sensor, wire and actuator polarity using the original vehicle C code.

Suspended pulse experiments remove gravity/contact/joint resistance only in
their own model instances. They identify direction, not vehicle performance.
"""
from __future__ import annotations

import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from balance_lab.firmware import Command, ChassisMode, GimbalMode
from balance_lab.firmware.build import HOST, VEHICLE, build
from balance_lab.firmware.simulation import HIP_NAMES, WHEEL_NAMES, VehicleSimulation
from balance_lab.plant import VARIANTS

CHECK_GROUPS = ("hip_wire", "wheel_wire", "live_hip_callbacks", "imu", "longitudinal_acceleration",
                "actuator_pulses", "vmc_pulses", "gimbal_closed_loop")


class VmcDiagnostic:
    def __init__(self, directory):
        build()  # Synchronise the C headers before compiling the diagnostic ABI.
        source = Path(__file__).with_name("native_polarity_helpers.c")
        target = Path(directory) / "polarity.so"
        include = [HOST / "include", VEHICLE / "src"]
        include += sorted({p.parent for p in VEHICLE.rglob("*.h")}
                          - {VEHICLE / "src/algorithm/arm_math"})
        subprocess.run(["gcc", "-std=c11", "-shared", "-fPIC", "-O2", "-ffp-contract=off",
                        *[f"-I{p}" for p in include], str(source), "-lm", "-o", str(target)],
                       check=True, capture_output=True, text=True)
        self.library = ct.CDLL(str(target))
        self.library.audit_vmc.argtypes = [ct.c_void_p] + [ct.c_float] * 5 + [ct.POINTER(ct.c_float)]
        self.library.audit_vmc.restype = None
        self.library.audit_constants.argtypes = [ct.POINTER(ct.c_float)]
        self.library.audit_constants.restype = None
        constants = (ct.c_float * 5)()
        self.library.audit_constants(constants)
        self.constants = dict(zip(("wheel_reduction_l", "wheel_reduction_r", "rpm_to_rad_s",
                                   "gimbal_yaw_drive_sign", "gimbal_yaw_gear_ratio"), constants))

    def evaluate(self, sim, side, *, angles=None, force=0, tp=0, pitch=0):
        sim.runtime.lib.chassis_control_module_leg.argtypes = [ct.c_int32]
        sim.runtime.lib.chassis_control_module_leg.restype = ct.c_void_p
        pointer = sim.runtime.lib.chassis_control_module_leg(side)
        names = ("l_j2", "l_j1") if side == 0 else ("r_j2", "r_j1")
        if angles is None:
            angles = [float(sim.data.qpos[sim.qadr[n]] - sim.zero[n]) % (2*np.pi) for n in names]
        result = (ct.c_float * 11)()
        self.library.audit_vmc(pointer, *angles, pitch, force, tp, result)
        a = np.array(result, dtype=float)
        return {"length_m": a[0], "phi0_rad": a[1], "theta_rad": a[2],
                "phi1_rad": a[3], "phi4_rad": a[4],
                "jacobian_front_back": a[5:9].reshape(2, 2), "torque_front_back_nm": a[9:11]}


def vector_angle(a, b):
    return float(np.arctan2(np.sin(a-b), np.cos(a-b)))


def cad_leg(sim, side):
    prefix = ("l", "r")[side]
    frame = sim.data.xmat[sim.base].reshape(3, 3) @ sim.sensor_body
    hip = .5 * (sim.data.xanchor[sim.joints[prefix+"_j1"]] + sim.data.xanchor[sim.joints[prefix+"_j2"]])
    wheel = sim.data.xpos[sim._id(mujoco.mjtObj.mjOBJ_BODY, prefix+"_wheel")]
    v = frame.T @ (wheel-hip)
    return {"length_m":float(np.linalg.norm(v[1:])),"phi0_rad":float(np.arctan2(-v[2], v[1]))}


def suspended(sim):
    sim.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_GRAVITY)
    sim.model.geom_contype[:] = 0
    sim.model.geom_conaffinity[:] = 0
    sim.model.dof_damping[:] = 0
    sim.model.dof_frictionloss[:] = 0
    sim.gas = []
    sim.data.qvel[:] = 0
    mujoco.mj_forward(sim.model, sim.data)


def pulse_steps(sim, ticks=40):
    start = float(sim.data.time)
    for _ in range(ticks):
        mujoco.mj_step(sim.model, sim.data)
        if any(sim.data.warning[w].number for w in (mujoco.mjtWarning.mjWARN_BADQPOS,
               mujoco.mjtWarning.mjWARN_BADQVEL, mujoco.mjtWarning.mjWARN_BADQACC)):
            raise RuntimeError("invalid state in suspended polarity pulse")
    if not np.isclose(sim.data.time,start+ticks*sim.tick_s,atol=1e-10,rtol=0):
        raise RuntimeError("plant restarted during suspended polarity pulse")


def geometry_and_vmc(diag, variant, pose):
    sim = VehicleSimulation(variant, pose=pose)
    try:
        frame = sim.data.xmat[sim.base].reshape(3, 3) @ sim.sensor_body
        rows = []
        for side, prefix in enumerate(("l", "r")):
            c = diag.evaluate(sim, side)
            hip = .5 * (sim.data.xanchor[sim.joints[prefix+"_j1"]] + sim.data.xanchor[sim.joints[prefix+"_j2"]])
            wheel = sim.data.xpos[sim._id(mujoco.mjtObj.mjOBJ_BODY, prefix+"_wheel")]
            v = frame.T @ (wheel-hip)
            phi = float(np.arctan2(-v[2], v[1]))
            # CAD [J1,J2] = firmware [BACK,FRONT]; C left callbacks negate,
            # right callbacks preserve the VMC torque. These are not a second
            # sign in the hardware adapter.
            sign = -1 if side == 0 else 1
            jac = sign * c["jacobian_front_back"][[1, 0], :]
            names = (prefix+"_j1", prefix+"_j2")
            finite_difference = []
            for index, name in enumerate(names):
                angles = [float(sim.data.qpos[sim.qadr[n]]-sim.zero[n]) % (2*np.pi)
                          for n in names[::-1]]
                h = .001
                angles[1-index] += h
                plus = diag.evaluate(sim, side, angles=angles)
                angles[1-index] -= 2*h
                minus = diag.evaluate(sim, side, angles=angles)
                finite_difference.append([(plus["length_m"]-minus["length_m"])/(2*h),
                                          vector_angle(plus["phi0_rad"],minus["phi0_rad"])/(2*h)])
            finite_difference = np.array(finite_difference)
            rows.append({"side":prefix,"c_length_m":c["length_m"],"c_phi0_deg":np.degrees(c["phi0_rad"]),
                         "cad_projected_phi0_deg":np.degrees(phi),
                         "cad_projected_length_m":float(np.linalg.norm(v[1:])),
                         "phi_difference_deg":np.degrees(vector_angle(c["phi0_rad"],phi)),
                         "c_raw_encoder_jacobian":jac.tolist(),
                         "c_finite_difference_jacobian":finite_difference.tolist(),
                         "jacobian_signs_match":bool(np.all(np.sign(jac)==np.sign(finite_difference))),
                         "max_jacobian_error":float(np.max(abs(jac-finite_difference)))})
        wheels = []
        for side,name in enumerate(WHEEL_NAMES):
            coefficient = float(np.dot(np.cross(sim.data.xaxis[sim.joints[name]],[0,0,1]),frame[:,1]))
            wheels.append({"joint":name,"forward_speed_per_qdot_radius":coefficient,
                           "c_forward_speed_sign":(-1,1)[side],
                           "matches":bool(np.sign(coefficient)==(-1,1)[side])})
        return {"variant":variant,"pose":pose,"legs":rows,"wheels":wheels,
                "sensor_frame_determinant":float(np.linalg.det(sim.sensor_body))}
    finally:
        sim.close()


def hip_wire_channel(index, sign):
    sim = VehicleSimulation(pose="stand")
    try:
        name = HIP_NAMES[index]
        sim.data.qpos[sim.qadr[name]] += sign*.1
        sim.data.qvel[sim.dadr[name]] = sign*.4
        mujoco.mj_forward(sim.model,sim.data)
        f = sim.read_feedback()
        # Isolate the driver payload check from the initial forward cache.
        f.hip_torque[index] = sign*1.5
        sim.runtime.step(f,Command())
        page = sim.runtime.debug_page(47)
        actual = [page[4+index],page[8+index],page[12+index]]
        expected = [float(f.hip_angle[index]),float(f.hip_speed[index]),float(f.hip_torque[index])]
        errors = [vector_angle(actual[0],expected[0]),actual[1]-expected[1],actual[2]-expected[2]]
        return {"joint":name,"dm_id":index+1,"direction":sign,"expected":expected,"decoded":actual,
                "angle_speed_torque_errors":errors,
                "passed":abs(errors[0])<.0003 and abs(errors[1])<.023 and abs(errors[2])<.035}
    finally:
        sim.close()


def wheel_wire_channel(diag, index, sign):
    sim = VehicleSimulation(pose="stand")
    try:
        f = sim.read_feedback()
        f.wheel_speed[index] = sign*.4
        f.wheel_torque[index] = sign*1.5
        first=None
        for tick in range(1,101):
            f.time_us=tick*1000
            sim.runtime.step(f,Command())
            page=sim.runtime.debug_page(47)
            if first is None: first=[page[26+index],page[28+index]]
        ratio=diag.constants[("wheel_reduction_l","wheel_reduction_r")[index]]
        expected=[int(float(f.wheel_speed[index])*ratio/diag.constants['rpm_to_rad_s']),
                  int(float(f.wheel_torque[index])*sim.runtime.parameters['wheel_current_per_nm'])]
        actual=[page[26+index],page[28+index]]
        return {"joint":WHEEL_NAMES[index],"direction":sign,"expected_rpm_current":expected,
                "first_filtered_rpm_current":first,"settled_rpm_current":actual,
                "passed":bool(np.allclose(expected,actual,atol=1,rtol=0))}
    finally:
        sim.close()


def live_hip_callbacks():
    sim=VehicleSimulation(pose="stand")
    try:
        counts=np.zeros(5,dtype=np.uint32)
        comparisons=[]
        command=Command(chassis_mode=ChassisMode.OPEN_LOOP)
        for tick in range(1,151):
            f=sim.read_feedback();f.time_us=tick*1000;f.feedback_mask=511
            out=sim.runtime.step(f,command)
            new=np.array(out.motor_tx_count,dtype=np.uint32)
            if tick>50:
                for slot,vmc_index,sign in ((0,1,-1),(1,3,1),(2,2,1),(3,0,-1)):
                    if new[slot]!=counts[slot]:
                        expected=float(np.clip(sign*out.vmc[vmc_index],-20,20))
                        actual=float(out.hip_command[slot][4])
                        comparisons.append({"time_us":tick*1000,"dm_id":slot+1,"expected_torque_nm":expected,
                                            "wire_torque_nm":actual,"error_nm":actual-expected})
            counts=new
        error=max(abs(row['error_nm']) for row in comparisons)
        return {"scenario":"static stand sensors, original OPEN_LOOP callbacks and DM round robin",
                "checked_packets":len(comparisons),"max_error_nm":error,
                "packets":comparisons,"passed":bool(error<70/4095+.0001 and len(comparisons)==40)}
    finally:
        sim.close()


def imu_rotation(axis, sign):
    sim = VehicleSimulation(pose="stand")
    try:
        free = sim._id(mujoco.mjtObj.mjOBJ_JOINT,"base_free")
        qa,va = int(sim.model.jnt_qposadr[free]),int(sim.model.jnt_dofadr[free])
        initial = sim.data.qpos[qa+3:qa+7].copy()
        basis = sim.sensor_body[:,axis]
        rate = sign*.2
        for tick in range(1,201):
            delta = np.empty(4); rotated=np.empty(4)
            mujoco.mju_axisAngle2Quat(delta,basis,rate*tick*sim.tick_s)
            mujoco.mju_mulQuat(rotated,initial,delta)
            sim.data.qpos[qa+3:qa+7]=rotated
            sim.data.qvel[:] = 0
            sim.data.qvel[va+3:va+6]=basis*rate
            mujoco.mj_forward(sim.model,sim.data)
            sim.ticks=tick-1
            f=sim.read_feedback()
            # Prescribed rotation, zero translational specific acceleration.
            frame=sim.data.xmat[sim.base].reshape(3,3)@sim.sensor_body
            f.accel[:]=frame.T@(-sim.model.opt.gravity)
            out=sim.runtime.step(f,Command());sim.output=out
        channel=(1,0,2)[axis]
        expected_angle=(1,-1,-1)[axis]*np.degrees(rate*.2)
        actual_angle=float(out.imu[channel])
        expected_gyro=np.zeros(3);expected_gyro[axis]=(-1,1,-1)[axis]*rate
        return {"raw_axis":"XYZ"[axis],"rate_rad_s":rate,"published_angle_channel":("pitch","roll","yaw")[axis],
                "expected_angle_deg":expected_angle,"published_angle_deg":actual_angle,
                "published_gyro_rad_s":list(out.imu)[3:6],"expected_gyro_rad_s":expected_gyro.tolist(),
                "angle_error_deg":actual_angle-expected_angle,
                "passed":bool(abs(actual_angle-expected_angle)<.05 and np.max(abs(np.array(out.imu)[3:6]-expected_gyro))<1e-5)}
    finally:
        sim.close()


def longitudinal_acceleration(sign):
    sim=VehicleSimulation(pose="stand")
    try:
        free=sim._id(mujoco.mjtObj.mjOBJ_JOINT,"base_free")
        va=int(sim.model.jnt_dofadr[free])
        forward=(sim.data.xmat[sim.base].reshape(3,3)@sim.sensor_body)[:,1]
        raw=[]
        for tick in range(1,201):
            sim.ticks=tick-1
            sim.data.qvel[:]=0
            sim.data.qvel[va:va+3]=forward*sign*.5*max(0,tick-100)*sim.tick_s
            mujoco.mj_forward(sim.model,sim.data)
            f=sim.read_feedback()
            out=sim.runtime.step(f,Command());sim.output=out
            if tick>100: raw.append(float(f.accel[1]))
        return {"acceleration_m_s2":sign*.5,"raw_forward_specific_force_mean_m_s2":float(np.mean(raw)),
                "published_forward_motion_accel_m_s2":float(out.imu[7]),
                "passed":bool(sign*np.mean(raw)>.49 and sign*out.imu[7]>.01)}
    finally:
        sim.close()


def actuator_pulse(name, sign):
    sim=VehicleSimulation(pose="stand")
    try:
        suspended(sim)
        q0=float(sim.data.qpos[sim.qadr[name]])
        sim.data.ctrl[:]=0
        sim.data.ctrl[sim.actuators[name]]=sign*.1
        pulse_steps(sim)
        dq=float(sim.data.qpos[sim.qadr[name]]-q0)
        velocity=float(sim.data.qvel[sim.dadr[name]])
        f=sim.read_feedback()
        return {"joint":name,"requested_torque_nm":sign*.1,"delta_q_rad":dq,"qdot_rad_s":velocity,
                "readback_speed_rad_s":float(f.yaw_motor_speed if name=="gimbal_yaw" else f.wheel_speed[WHEEL_NAMES.index(name)]),
                "gimbal_imu_yaw_deg":float(f.gimbal_yaw_deg) if name=="gimbal_yaw" else None,
                "passed":sign*dq>0 and sign*velocity>0}
    finally:
        sim.close()


def vmc_pulse(diag, side, kind, sign):
    sim=VehicleSimulation(pose="stand")
    try:
        suspended(sim)
        prefix=("l","r")[side]
        before=diag.evaluate(sim,side)
        cad_before=cad_leg(sim,side)
        force=sign*20 if kind=="extension" else 0
        tp=sign*.4 if kind=="swing" else 0
        c=diag.evaluate(sim,side,force=force,tp=tp)
        torque=(-1 if side==0 else 1)*c["torque_front_back_nm"]
        sim.data.ctrl[:]=0
        for name,value in zip((prefix+"_j2",prefix+"_j1"),torque): sim.data.ctrl[sim.actuators[name]]=value
        pulse_steps(sim)
        after=diag.evaluate(sim,side)
        cad_after=cad_leg(sim,side)
        dl=after["length_m"]-before["length_m"]
        dphi=vector_angle(after["phi0_rad"],before["phi0_rad"])
        cad_dl=cad_after["length_m"]-cad_before["length_m"]
        cad_dphi=vector_angle(cad_after["phi0_rad"],cad_before["phi0_rad"])
        return {"side":prefix,"kind":kind,"direction":sign,"front_back_torque_nm":torque.tolist(),
                "delta_length_m":dl,"delta_phi0_rad":dphi,"virtual_work_j":force*dl+tp*dphi,
                "cad_delta_length_m":cad_dl,"cad_delta_phi0_rad":cad_dphi,
                "passed":bool(sign*(dl if kind=="extension" else dphi)>0 and
                              sign*(cad_dl if kind=="extension" else cad_dphi)>0)}
    finally:
        sim.close()


def gimbal_closed_loop(sign):
    sim=VehicleSimulation(pose="stand")
    try:
        suspended(sim)
        initial=sim.data.qpos[sim.qadr['gimbal_yaw']]
        for tick in range(500):
            command=Command(chassis_mode=ChassisMode.INIT,
                            gimbal_mode=GimbalMode.INIT if tick<200 else GimbalMode.GYRO,
                            gimbal_yaw_raw=0 if tick<200 else sign*100)
            out=sim.step(command)
        relative=sim.data.qpos[sim.qadr['gimbal_yaw']]-initial
        return {"manual_command":sign*100,"target_rad":float(out.yaw_target),
                "joint_delta_rad":float(relative),"motor_relative_yaw_rad":float(out.yaw_relative),
                "imu_world_yaw_rad":float(np.radians(sim.feedback.gimbal_yaw_deg)),
                "gimbal_ready":int(out.gimbal_ready),"passed":bool(sign*out.yaw_target<-.01 and
                    sign*relative>.01 and sign*sim.feedback.gimbal_yaw_deg<-.1 and out.gimbal_ready==1)}
    finally:
        sim.close()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output",type=Path,default=Path("out/native/polarity_audit"))
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="rl-lab-polarity-") as directory:
        diag=VmcDiagnostic(directory)
        with_sim=VehicleSimulation(pose="stand")
        try: metadata=with_sim.metadata()
        finally: with_sim.close()
        report={"purpose":"polarity checks, not physical standing acceptance","mujoco_version":mujoco.__version__,
                "metadata":metadata,"c_constants":diag.constants,
                "model_xml_sha256":{v:hashlib.sha256(Path(VARIANTS[v]).read_bytes()).hexdigest() for v in ('fast','full')},
                "adapter_sha256":hashlib.sha256((ROOT/'balance_lab/firmware/simulation.py').read_bytes()).hexdigest(),
                "bench":"40 ms actuator pulses and 500 ms yaw test disable gravity forces, contacts, joint damping and Coulomb resistance; plant gains and inertia remain. IMU tests prescribe rotation or translation and use original INS. No standing acceptance is performed.",
                "diagnostic_sha256":hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                "helper_sha256":hashlib.sha256(Path(__file__).with_name('native_polarity_helpers.c').read_bytes()).hexdigest(),
                "geometry":[geometry_and_vmc(diag,v,p) for v in ("fast","full") for p in ("stand","encoder")],
                "hip_wire":[hip_wire_channel(i,s) for i in range(4) for s in (-1,1)],
                "wheel_wire":[wheel_wire_channel(diag,i,s) for i in range(2) for s in (-1,1)],
                "live_hip_callbacks":[live_hip_callbacks()],
                "imu":[imu_rotation(i,s) for i in range(3) for s in (-1,1)],
                "longitudinal_acceleration":[longitudinal_acceleration(s) for s in (-1,1)],
                "actuator_pulses":[actuator_pulse(n,s) for n in WHEEL_NAMES+("gimbal_yaw",) for s in (-1,1)],
                "vmc_pulses":[vmc_pulse(diag,i,k,s) for i in range(2) for k in ("extension","swing") for s in (-1,1)],
                "gimbal_closed_loop":[gimbal_closed_loop(s) for s in (-1,1)]}
    report['directional_cases']=sum(len(report[g]) for g in CHECK_GROUPS)
    report['directional_cases_passed']=sum(r['passed'] for g in CHECK_GROUPS for r in report[g])
    report['geometry_cases_passed']=sum(all(x['matches'] for x in r['wheels']) and
        all(x['jacobian_signs_match'] for x in r['legs']) and abs(r['sensor_frame_determinant']-1)<1e-6
        for r in report['geometry'])
    report['all_polarity_checks_passed']=(report['directional_cases_passed']==report['directional_cases'] and
                                        report['geometry_cases_passed']==len(report['geometry']))
    (args.output/'report.json').write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    for group in CHECK_GROUPS:
        failures=[r for r in report[group] if not r['passed']]
        print(group,len(report[group])-len(failures),'/',len(report[group]),'passed')
        for row in failures: print('FAILED',json.dumps(row,ensure_ascii=False))
    for row in report['geometry']:
        print('geometry',row['variant'],row['pose'],'wheel signs',[w['matches'] for w in row['wheels']],
              'VMC signs',[l['jacobian_signs_match'] for l in row['legs']],
              'CAD phi gaps',[round(l['phi_difference_deg'],4) for l in row['legs']])
    print((args.output/'report.json').resolve())
    if not report['all_polarity_checks_passed']: raise SystemExit(1)


if __name__=="__main__": main()
