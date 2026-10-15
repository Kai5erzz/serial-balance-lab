"""Temporary, runtime-only firmware-vs-simulation jump probe.

Run with:
  /home/kaiser/miniconda3/envs/rl-lab/bin/python tools/probes/firmware_jump_gate_probe.py

It leaves the repository controller untouched. Runtime overrides approximate
the firmware's current .36 m THRUST target and its boost/floor fade. During
powered THRUST supports are clamped nonnegative; after thrust release the
profile-start path switches final axial support to the air deploy controller,
while the firmware separately records its pre-override support candidate for
the force-based ground estimator. Delayed-contact cases postpone software
off-ground confirmation until THRUST elapsed reaches .32 s. The theta cases
also apply firmware grounded/air theta gains and captured air reference. This
is still not a full firmware plant model: it does not reproduce the force
observer, support-candidate estimator, raw encoder-zero path, motor feedback
filters, or driver delays.
"""
import sys, math, types, inspect, textwrap
sys.path.insert(0, '/home/kaiser/robomaster/rl-lab')
import mujoco, numpy as np
import balance_lab.control.jump as jm
import balance_lab.control.tuning as tuning
from balance_lab import CALIBRATED_STAND_POSE_PATH, TraditionalLQR, apply_saved_pose
from pathlib import Path
MODEL=Path('/home/kaiser/robomaster/rl-lab/model/whole_body_lqr_fast.xml')

def run(name, boost, thrust_len, tuck_len, tuck_time, hold, delayed=False, length_release=False, early_wheel=False, fw_support=False, full_theta=False, hip_limit=40., rate_scale=1.):
    jm.JUMP_THRUST_BOOST_TIME=boost; jm.JUMP_THRUST_LENGTH=thrust_len
    model=mujoco.MjModel.from_xml_path(str(MODEL)); d=mujoco.MjData(model); c=TraditionalLQR(model)
    if hip_limit < 40.:
        for ids in c.sides.values():
            for actuator in ids.hip_motors: model.actuator_ctrlrange[actuator]=[-hip_limit,hip_limit]
    c.jump_air_tuck_length=tuck_len; c.jump_air_tuck_time=tuck_time; c.jump_air_tuck_hold_time=hold
    if fw_support or full_theta: c.jump_air_tuck_ref_rate=1.40
    if full_theta:
        # Match the current firmware's grounded pitch/theta and airborne
        # theta allocations. These are per-instance probe settings only.
        c.jump_thrust_pitch_kp=8.; c.jump_thrust_pitch_kd=20.
        c.jump_thrust_pitch_rate_filter_tau=.060
        c.jump_thrust_pitch_rate_target=1.48
        c.jump_thrust_theta_kp=80.; c.jump_thrust_theta_kd=12.
        c.jump_thrust_theta_limit=40.
        c.jump_air_theta_kp=80.; c.jump_air_theta_kd=12.
        c.jump_air_theta_limit=40.
        c.jump_air_theta_weight_tuck=1.; c.jump_air_theta_weight_deploy=1.
        c.jump_air_common_tp_limit=22.
        c.jump_air_pitch_kp=0.; c.jump_air_pitch_kd=0.
        # Apply the firmware's +/-0.20 rad theta-error clamp in both axes
        # while keeping the independent yield authority on the raw error.
        f=c._apply_jump_hip_control.__func__
        src=textwrap.dedent(inspect.getsource(f))
        old='* (theta_common - self.jump_thrust_theta_ref)'
        new='* float(np.clip(theta_common - self.jump_thrust_theta_ref, -0.20, 0.20))'
        assert old in src
        src=src.replace(old,new)
        old='self.jump_air_theta_kp * theta_common'
        new='self.jump_air_theta_kp * float(np.clip(theta_common - self.jump_air_theta_ref, -0.20, 0.20))'
        assert old in src
        src=src.replace(old,new)
        scope={}
        exec(compile(src.replace('def _apply_jump_hip_control(', 'def _apply_jump_hip_control_fw('),inspect.getsourcefile(f),'exec'),f.__globals__,scope)
        c._apply_jump_hip_control=types.MethodType(scope['_apply_jump_hip_control_fw'],c)
        c.jump_air_theta_ref=0.; c.jump_air_theta_ref_valid=False
        original_step=c.step
        def step_with_theta_ref(data,dt):
            if c.jump_state in {'thrust','flight'}:
                if not c.jump_air_theta_ref_valid:
                    th=[math.pi/2-c.vmc_kinematics(data,s).phi0+c.body_pitch(data) for s in ('l','r')]
                    c.jump_air_theta_ref=.5*sum(th); c.jump_air_theta_ref_valid=True
                else:
                    c.jump_air_theta_ref+=float(np.clip(-c.jump_air_theta_ref,-dt,dt))
            return original_step(data,dt)
        c.step=step_with_theta_ref
    if rate_scale != 1.0:
        # Sensitivity probe for the firmware's legacy 5 ms VMC dt running in
        # a task nominally scheduled at 1 ms: scale measured leg/theta rates
        # at the raw-derivative inputs. Do not rescale stored filtered values
        # each control cycle (which would compound attenuation over time).
        for method_name, old, new in (
            ('_state',
             'self._wrap_pi(theta - self.previous[f"theta_{side}"]) / dt',
             f'self._wrap_pi(theta - self.previous[f"theta_{{side}}"])*{rate_scale} / dt'),
            ('_apply_vmc_support',
             '            length_rate_raw = (\n                lengths[side] - self.previous[f"length_{side}"]\n            ) / control_dt',
             f'            length_rate_raw = (\n                lengths[side] - self.previous[f"length_{{side}}"]\n            )*{rate_scale} / control_dt'),
        ):
            original_method = getattr(c, method_name).__func__
            source = textwrap.dedent(inspect.getsource(original_method))
            assert old in source, (method_name, old)
            source = source.replace(old, new)
            namespace = {}
            renamed = f'def {method_name}_rate_scaled('
            source = source.replace(f'def {method_name}(', renamed, 1)
            exec(compile(source, inspect.getsourcefile(original_method), 'exec'), original_method.__globals__, namespace)
            setattr(c, method_name, types.MethodType(namespace[f'{method_name}_rate_scaled'], c))
    if fw_support:
        # Runtime-only port of the firmware THRUST/release support shape.
        # Keep sim geometry and VMC mapping; match the force fade/floor and
        # the negative-force release branch for an informative comparison.
        def support(self, lengths):
            if self.jump_off_ground or (self.jump_state=='thrust' and self.jump_air_profile_active):
                air=self._jump_air_supports(lengths)
                if self.landing_buffer_phase!='inactive':
                    landing=self._landing_supports(lengths)
                    blend=float(np.clip(self.landing_buffer_elapsed/tuning.LANDING_SUPPORT_BLEND_TIME,0,1))
                    return {s:air[s]+blend*(landing[s]-air[s]) for s in ('l','r')}
                return air
            if self.landing_buffer_phase!='inactive': return self._landing_supports(lengths)
            thrust=self.jump_state=='thrust'
            preload=self.jump_state=='preload'
            if not (thrust or preload): return self._jump_supports_original(lengths)
            if thrust: kp,kd,limit=600.,16.,120.
            else: kp,kd,limit=450.,28.,70.
            out={}
            for s in ('l','r'):
                spring=self._gas_virtual_support(lengths[s]) if self.gas_enabled else 0.
                lf=float(np.clip(kp*(self.jump_length_target-lengths[s])-kd*self.length_rate_lpf[s],-limit,limit))
                out[s]=.5*tuning.TOTAL_MASS*tuning.GRAVITY-spring+lf+self.support_trim[s]+(self.roll_force if s=='l' else -self.roll_force)
            if thrust:
                sync=float(np.clip(tuning.JUMP_LENGTH_SYNC_KP*(lengths['r']-lengths['l'])+tuning.JUMP_LENGTH_SYNC_KD*(self.length_rate_lpf['r']-self.length_rate_lpf['l']),-tuning.JUMP_LENGTH_SYNC_LIMIT,tuning.JUMP_LENGTH_SYNC_LIMIT))
                out['l']+=sync; out['r']-=sync
            avg=.5*(lengths['l']+lengths['r'])
            scale=float(np.clip((thrust_len-avg)/(thrust_len-.26),0.,1.))
            if thrust and self.jump_elapsed<jm.JUMP_THRUST_BOOST_TIME:
                floor=120.*scale; boost=70.*scale
                for s in ('l','r'): out[s]=max(out[s]+boost,floor)
                # Firmware's final grounded THRUST force clamp.
                for s in ('l','r'): out[s]=max(out[s],0.)
            return {s:float(np.clip(out[s],-tuning.SUPPORT_FORCE_LIMIT,tuning.SUPPORT_FORCE_LIMIT)) for s in ('l','r')}
        c._jump_supports_original=c._jump_supports
        c._jump_supports=types.MethodType(support,c)
    apply_saved_pose(model,d,CALIBRATED_STAND_POSE_PATH); c.reset(d,calibrate_frame=True); c.enable_calibrated_lqr(d); c.set_drive_command(0.,0.)
    while d.time<3.: c.step(d,model.opt.timestep); mujoco.mj_step(model,d)
    t0=d.time; c.begin_jump(d); z0=float(d.xpos[c.base,2]); original=c._update_jump_contacts
    def contacts(data):
        if delayed and c.jump_state=='thrust' and c.jump_elapsed<.32:
            raw=c._wheel_contacts(data); c.jump_raw_wheel_contact=raw
            c.jump_takeoff_count=(c.jump_takeoff_count+1) if not any(raw.values()) else 0
            return
        original(data)
    c._update_jump_contacts=contacts
    released=False; release_t=physical_t=confirm_t=first_contact_t=None; prev_off=False; no_contact_start=None; longest_no_contact=0.
    transitions=[]; max_pitch=0.; max_pitch_thrust=0.; max_rate=0.; signed_p_min=999.; signed_p_max=-999.; max_thrust_theta_error=0.; max_air_theta_error=0.; peak_wheel_req=peak_wheel_cmd=0.; max_z=z0; min_z=z0
    hip_requested_peak={'l1':0.,'l2':0.,'r1':0.,'r2':0.}; hip_applied_peak={'l1':0.,'l2':0.,'r1':0.,'r2':0.}; hip_over35_ticks=0
    for _ in range(int(3.7/model.opt.timestep)):
        if length_release and c.jump_state=='thrust' and not released:
            ls=[c.vmc_kinematics(d,s).l0 for s in ('l','r')]
            if min(ls)>=.33 and .5*sum(ls)>=.35:
                jm.JUMP_THRUST_BOOST_TIME=min(jm.JUMP_THRUST_BOOST_TIME,c.jump_elapsed)
                released=True; release_t=float(d.time)
        if c.jump_state=='thrust' and not released and c.jump_elapsed>=jm.JUMP_THRUST_BOOST_TIME:
            released=True; release_t=float(d.time)
        c.step(d,model.opt.timestep)
        if early_wheel and released and c.jump_state=='thrust' and not c.jump_off_ground:
            pitch=c.body_pitch(d); rate=-float(c.last_state['l'][5])
            compact=c.jump_air_elapsed < (tuck_time+hold)
            pref=(c.jump_air_tuck_pitch_bias if compact else c.jump_air_deploy_pitch_bias)+c.jump_air_moving_pitch_gain*c.drive_forward_target
            pref=float(np.clip(pref,-c.jump_air_pitch_reference_limit,c.jump_air_pitch_reference_limit))
            fb=c.jump_air_wheel_pitch_kp*(pitch-pref)+c.jump_air_wheel_pitch_kd*rate
            ths=[math.pi/2-c.vmc_kinematics(d,s).phi0+pitch for s in ('l','r')]
            theta_hold=float(np.clip(c.jump_air_theta_kp*.5*sum(ths)+c.jump_air_theta_kd*.5*sum(c.theta_rate_lpf.values()),-c.jump_air_theta_limit,c.jump_air_theta_limit))
            wt=c.jump_air_theta_weight_tuck if compact else c.jump_air_theta_weight_deploy
            req=c.jump_air_wheel_sign*(fb+c.jump_air_wheel_theta_ff*wt*theta_hold)
            taper=tuck_time+hold+c.jump_air_wheel_taper_delay
            if c.jump_air_elapsed>taper:
                req*=1-float(np.clip((c.jump_air_elapsed-taper)/c.jump_air_wheel_taper_time,0,1))
            c.jump_air_wheel_torque=float(np.clip(req,-c.jump_air_wheel_torque_limit,c.jump_air_wheel_torque_limit))
            for s,ids in c.sides.items(): d.ctrl[ids.wheel_motor]=float(np.clip(c.wheel_sign[s]*c.jump_air_wheel_torque,-c.jump_air_wheel_torque_limit,c.jump_air_wheel_torque_limit))
        p=math.degrees(c.body_pitch(d)); rate=math.degrees(-float(c.last_state['l'][5]));
        max_pitch=max(max_pitch,abs(p)); max_rate=max(max_rate,abs(rate))
        signed_p_min=min(signed_p_min,p); signed_p_max=max(signed_p_max,p)
        th=[math.pi/2-c.vmc_kinematics(d,s).phi0+c.body_pitch(d) for s in ('l','r')]
        theta_common=.5*sum(th)
        if c.jump_state=='thrust': max_thrust_theta_error=max(max_thrust_theta_error,abs(theta_common-c.jump_thrust_theta_ref))
        if c.jump_state=='flight' and full_theta: max_air_theta_error=max(max_air_theta_error,abs(theta_common-c.jump_air_theta_ref))
        if c.jump_state=='thrust': max_pitch_thrust=max(max_pitch_thrust,abs(p))
        z=float(d.xpos[c.base,2]); max_z=max(max_z,z); min_z=min(min_z,z)
        raw=c._wheel_contacts(d); contact=any(raw.values())
        if not contact:
            if no_contact_start is None: no_contact_start=float(d.time)
            longest_no_contact=max(longest_no_contact,float(d.time)-no_contact_start)
        else:
            if no_contact_start is not None:
                gap=float(d.time)-no_contact_start
                longest_no_contact=max(longest_no_contact,gap)
                if gap>=.02 and physical_t is None:
                    physical_t=no_contact_start; first_contact_t=float(d.time)
            no_contact_start=None
        if c.jump_off_ground and not prev_off: confirm_t=float(d.time)
        prev_off=c.jump_off_ground
        wh=max(abs(float(d.ctrl[ids.wheel_motor])) for ids in c.sides.values())
        peak_wheel_req=max(peak_wheel_req,abs(float(c.jump_air_wheel_torque))); peak_wheel_cmd=max(peak_wheel_cmd,wh)
        for side,ids in c.sides.items():
            for j,act in enumerate(ids.hip_motors):
                label=f'{side}{j+1}'; val=abs(float(c.last_motor_torque[side][j]))
                hip_requested_peak[label]=max(hip_requested_peak[label],val)
                if val>35.0: hip_over35_ticks+=1
        mujoco.mj_step(model,d)
        for side,ids in c.sides.items():
            for j,act in enumerate(ids.hip_motors):
                label=f'{side}{j+1}'; hip_applied_peak[label]=max(hip_applied_peak[label],abs(float(d.actuator_force[act])))
    tail_raw=c._wheel_contacts(d)
    print(f'{name}: release={None if release_t is None else round(release_t-t0,3)}s; physical_takeoff={None if physical_t is None else round(physical_t-t0,3)}s; confirmed={None if confirm_t is None else round(confirm_t-t0,3)}s; release_to_confirm={None if release_t is None or confirm_t is None else round(confirm_t-release_t,3)}s; flight={None if physical_t is None or first_contact_t is None else round(first_contact_t-physical_t,3)}s (max contact gap {longest_no_contact:.3f}s); z0/apex/min={z0:.3f}/{max_z:.3f}/{min_z:.3f}m; |pitch|max={max_pitch:.2f}deg (THRUST {max_pitch_thrust:.2f}; signed min/max {signed_p_min:.2f}/{signed_p_max:.2f}); |rate|max={max_rate:.1f}deg/s; theta_err max thrust/air={max_thrust_theta_error:.3f}/{max_air_theta_error:.3f}rad; wheel_req/cmd={peak_wheel_req:.2f}/{peak_wheel_cmd:.2f}Nm; hip_limit={hip_limit:.0f}Nm request_peak={hip_requested_peak} applied_peak={hip_applied_peak} request_over35_count={hip_over35_ticks}; final_pitch/roll={math.degrees(c.body_pitch(d)):.2f}/{math.degrees(c.body_roll(d)):.2f}deg, final_contact={tail_raw}; state={c.jump_state}, fault={c.jump_fault}')


run('sim-baseline',.26,.305,.130,.11,.12)
run('partial-support-shape-delayed-timer-release',.32,.36,.24,.06,.04,delayed=True,fw_support=True)
run('partial-support-shape-delayed-length-release',.32,.36,.24,.06,.04,delayed=True,length_release=True,fw_support=True)
run('partial-support-shape-delayed-length-release-early-wheel',.32,.36,.24,.06,.04,delayed=True,length_release=True,early_wheel=True,fw_support=True)
run('sim-fw-parameter-aligned-truth-contact',.32,.36,.24,.06,.04,fw_support=True,full_theta=True)
run('sim-fw-parameter-aligned-delayed-confirm',.32,.36,.24,.06,.04,delayed=True,fw_support=True,full_theta=True)
run('sim-fw-parameter-aligned-delayed-confirm-hip35',.32,.36,.24,.06,.04,delayed=True,fw_support=True,full_theta=True,hip_limit=35.)
run('sim-fw-aligned-truth-contact-rate-scale0.2',.32,.36,.24,.06,.04,fw_support=True,full_theta=True,rate_scale=.2)
run('sim-fw-aligned-delayed-confirm-rate-scale0.2',.32,.36,.24,.06,.04,delayed=True,fw_support=True,full_theta=True,rate_scale=.2)
