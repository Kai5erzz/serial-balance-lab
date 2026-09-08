import sys; sys.path.insert(0,'.')
import mujoco, numpy as np
import balance_lab.control.jump as _jm
from balance_lab.plant import load_model
from balance_lab.paths import CALIBRATED_STAND_POSE_PATH
from balance_lab.plant.poses import apply_saved_pose
from balance_lab.control.controller import TraditionalLQR
_jm.JUMP_THRUST_LENGTH=0.33; _jm.JUMP_PRELOAD_LENGTH=0.117
_jm.JUMP_THRUST_THETA_LQR_GAIN=0.2; _jm.JUMP_THRUST_BOOST_FORCE=100.0
_jm.JUMP_LENGTH_LIMIT_PRELOAD=110.0
SP=5.0; SPEED=1.26; FACE=1.2
def run(brake_at, jump_at, jd=0.0):
    m=mujoco.MjModel.from_xml_path('model/whole_body_lqr_fast_p270.xml')
    d=mujoco.MjData(m); c=TraditionalLQR(m)
    apply_saved_pose(m,d,CALIBRATED_STAND_POSE_PATH); mujoco.mj_forward(m,d)
    c.reset(d,calibrate_frame=True); h,_=c._heading(d)
    d.qpos[0]+=float(h[0])*SP; d.qpos[1]+=float(h[1])*SP; mujoco.mj_forward(m,d)
    c.enable_calibrated_lqr(d); c.disable_descent_guard=True; c.disable_danger_check=True
    c.set_leg_length(0.24); c.jump_thrust_pitch_rate_target=-2.0
    DT=m.opt.timestep
    while d.time<2.5: c.step(d,DT); mujoco.mj_step(m,d)
    c.set_drive_command(0.35,0.0); t0=d.time
    while d.time<16:
        cmd=min(SPEED,0.35+0.4*(d.time-t0)); c.set_drive_command(cmd,0.0)
        c.step(d,DT); mujoco.mj_step(m,d)
        if float(np.hypot(d.xpos[c.base][0],d.xpos[c.base][1])) <= FACE+brake_at: break
    c.set_drive_command(0.0,0.0)
    while d.time<16:
        c.step(d,DT); mujoco.mj_step(m,d)
        if float(np.hypot(d.xpos[c.base][0],d.xpos[c.base][1])) <= FACE+jump_at: break
    ok,r=c.begin_jump(d)
    if not ok: print(f"CFG b={brake_at} j={jump_at}: REJECTED {r}"); return
    took=None; apex_w=0.0; crossed=False; cross_z=None; lv=0; prev=True
    while c.jump_state!="idle" and d.time<22:
        if c.jump_state in ("preload","thrust"): c.set_drive_command(jd,0.0)
        c.step(d,DT); mujoco.mj_step(m,d)
        zw=min(float(d.xpos[c.sides[s].wheel_body][2]) for s in ("l","r"))
        dw=max(float(np.hypot(d.xpos[c.sides[s].wheel_body][0],d.xpos[c.sides[s].wheel_body][1])) for s in ("l","r"))
        apex_w=max(apex_w,zw)
        g=bool(c._wheel_contacts(d)["l"]) and bool(c._wheel_contacts(d)["r"])
        if not g and prev and took is None:
            took=1
            print(f"TAKEOFF b={brake_at} j={jump_at} jd={jd}: vx={float(c.chassis_vx):.2f} vz={float(d.qvel[2]):.2f} dw={dw:.3f}")
        if not g and not crossed and dw<=FACE: crossed=True; cross_z=zw
        prev=g
    for i in range(d.ncon):
        con=d.contact[i]
        for g2 in (int(con.geom1),int(con.geom2)):
            nm=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,g2)
            if nm=="p270_top" and abs(con.pos[2]-0.27)<=0.035: lv=1
    print(f"VERDICT b={brake_at} j={jump_at} jd={jd}: crossed={crossed} cross_z={cross_z} wheel_apex={apex_w:.3f} ontop={lv} fault={c.jump_fault}")
for ba,ja,jd in ((0.50,0.20,0.0),(0.50,0.15,0.0),(0.55,0.20,0.0)):
    run(ba,ja,jd)
