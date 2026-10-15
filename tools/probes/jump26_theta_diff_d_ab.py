"""Temporary, runtime-only firmware-vs-simulation jump probe.

Run with:
  /home/kaiser/miniconda3/envs/rl-lab/bin/python tools/probes/jump_rate_isolation_probe.py

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
import balance_lab.control.controller as controller_module
from balance_lab import CALIBRATED_STAND_POSE_PATH, TraditionalLQR, apply_saved_pose
from pathlib import Path
tuning.CONTROL_PERIOD_S=0.001
jm.CONTROL_PERIOD_S=0.001
controller_module.CONTROL_PERIOD_S=0.001
MODEL=Path('/home/kaiser/robomaster/rl-lab/model/whole_body_lqr_fast.xml')
# Runtime-only alignment: firmware jump mode has no theta-sync guard; use the
# observer only for grounded powered THRUST differential sync D, preserving
# per-leg LQR state/rates and every common/air/landing feedback path.
_step_fn=TraditionalLQR.step
_step_src=textwrap.dedent(inspect.getsource(_step_fn))
_old='theta_rate_diff = float(\n                self.last_state["l"][1] - self.last_state["r"][1]\n            )'
_new='theta_rate_diff = float(\n                self.last_state["l"][1] - self.last_state["r"][1]\n            )\n            if (self.jump_state == "thrust" and not self.jump_off_ground and\n                    getattr(self, "jump_theta_sync_rate_override", None) is not None):\n                theta_rate_diff = float(\n                    self.jump_theta_sync_rate_override["l"] -\n                    self.jump_theta_sync_rate_override["r"]\n                )'
assert _old in _step_src
_step_src=_step_src.replace(_old,_new)
_old_guard='sync_torque += float(np.copysign(\n                THETA_SYNC_GUARD_LIMIT * guard_progress,\n                theta_diff,\n            ))'
_new_guard='if self.jump_state == "idle":\n                sync_torque += float(np.copysign(\n                    THETA_SYNC_GUARD_LIMIT * guard_progress,\n                    theta_diff,\n                ))'
assert _old_guard in _step_src
_step_src=_step_src.replace(_old_guard,_new_guard)
_old_torque='sync_torque = sync_kp * theta_diff + sync_kd * theta_rate_diff'
_new_torque='sync_torque = sync_kp * theta_diff + sync_kd * theta_rate_diff\n            self.jump_probe_theta_sync_d = (\n                sync_kd * theta_rate_diff\n                if self.jump_state == "thrust" and not self.jump_off_ground\n                else 0.0\n            )'
assert _old_torque in _step_src
_step_src=_step_src.replace(_old_torque,_new_torque)
_step_ns={}
exec(compile(_step_src,inspect.getsourcefile(_step_fn),'exec'),_step_fn.__globals__,_step_ns)
TraditionalLQR.step=_step_ns['step']

def run(name, boost, thrust_len, tuck_len, tuck_time, hold, delayed=False, length_release=False, early_wheel=False, fw_support=False, full_theta=False, hip_limit=40., theta_jump_scale=1., length_jump_scale=1., observer_tau=None, observer_theta=False, observer_length=False, sync_theta_observer=False):
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
    # Preserve legacy normal theta/LQR scaling (.2), while the main length PID
    # uses actual 1 ms samples in both cases, matching PID.c's DWT-based D dt.
    for method_name, old, new in (
        ('_state',
         'self._wrap_pi(theta - self.previous[f"theta_{side}"]) / dt',
         'self._wrap_pi(theta - self.previous[f"theta_{side}"])*.2 / dt'),
    ):
        original_method = getattr(c, method_name).__func__
        source = textwrap.dedent(inspect.getsource(original_method))
        assert old in source, (method_name, old)
        source = source.replace(old, new)
        namespace = {}
        renamed = f'def {method_name}_legacy_rate_scaled('
        source = source.replace(f'def {method_name}(', renamed, 1)
        exec(compile(source, inspect.getsourcefile(original_method), 'exec'), original_method.__globals__, namespace)
        setattr(c, method_name, types.MethodType(namespace[f'{method_name}_legacy_rate_scaled'], c))

    # Sample kinematic rates at each 1 ms MuJoCo frame. The observer matches the
    # firmware's five-point (4 ms span) difference and first-order LPF; current
    # sample is supplied to the same update. It does not replay firmware PID.c's
    # exact derivative filtering or the unsampled hardware/driver dynamics.
    histories = {kind: {s: [] for s in ('l', 'r')} for kind in ('theta', 'length')}
    observer = {kind: {s: 0.0 for s in ('l', 'r')} for kind in ('theta', 'length')}
    raw_jump = {kind: {s: 0.0 for s in ('l', 'r')} for kind in ('theta', 'length')}
    current_jump_rates = {kind: {s: 0.0 for s in ('l', 'r')} for kind in ('theta', 'length')}
    current_jump_rates['theta_scale'] = theta_jump_scale
    c.jump_theta_sync_rate_override = current_jump_rates['theta'] if sync_theta_observer else None
    current_jump_rates['length_scale'] = length_jump_scale
    original_step_probe = c.step
    def sample_jump_rate_probe(data, dt):
        now = float(data.time)
        for side in ('l', 'r'):
            kin = c.vmc_kinematics(data, side)
            vals = {'theta': math.pi/2-kin.phi0+c.body_pitch(data), 'length': kin.l0}
            for kind, value in vals.items():
                hist = histories[kind][side]
                hist.append((now, value))
                if len(hist) > 5: hist.pop(0)
                if len(hist) >= 2:
                    t0, x0 = hist[-2]
                    dx = value - x0
                    if kind == 'theta':
                        while dx > math.pi: dx -= 2*math.pi
                        while dx < -math.pi: dx += 2*math.pi
                    raw_jump[kind][side] = dx / max(now-t0, 1e-9)
                if len(hist) >= 5:
                    t0, x0 = hist[0]
                    dx = value - x0
                    if kind == 'theta':
                        while dx > math.pi: dx -= 2*math.pi
                        while dx < -math.pi: dx += 2*math.pi
                    raw = dx / max(now-t0, 1e-9)
                    tau = observer_tau if observer_tau is not None else .010
                    alpha = dt / (tau + dt)
                    observer[kind][side] += alpha * (raw-observer[kind][side])
        active = c.jump_state != 'idle' or c.landing_buffer_phase != 'inactive'
        for side in ('l', 'r'):
            for kind, scale_key in (('theta', 'theta_scale'), ('length', 'length_scale')):
                use_observer = observer_tau is not None and (observer_theta if kind == 'theta' else observer_length)
                base = observer[kind][side] if use_observer else raw_jump[kind][side]
                current_jump_rates[kind][side] = base * current_jump_rates[scale_key] if active else 0.0

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
                sync_rates = current_jump_rates['length']
                sync=float(np.clip(tuning.JUMP_LENGTH_SYNC_KP*(lengths['r']-lengths['l'])+tuning.JUMP_LENGTH_SYNC_KD*(sync_rates['r']-sync_rates['l']),-tuning.JUMP_LENGTH_SYNC_LIMIT,tuning.JUMP_LENGTH_SYNC_LIMIT))
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
    # Mirror only direct jump-specific length-rate consumers in code_914:
    # - THRUST grounded sync KD (custom support port above),
    # - air/pre-tuck axial KD and air length-sync KD,
    # - landing-buffer differential sync KD.
    # The VMC/main preload-thrust-landing PID D term always keeps true 1 ms dt.
    original_air_support = c._jump_air_supports
    def air_support_with_jump_length_observer(lengths):
        if c.jump_off_ground or c.jump_air_profile_active:
            saved = c.length_rate_lpf
            c.length_rate_lpf = {s: current_jump_rates['length'][s] for s in ('l', 'r')}
            try:
                return original_air_support(lengths)
            finally:
                c.length_rate_lpf = saved
        return original_air_support(lengths)
    c._jump_air_supports = air_support_with_jump_length_observer

    original_landing_support = c._landing_supports
    fn = original_landing_support.__func__
    src = textwrap.dedent(inspect.getsource(fn))
    old = '(self.length_rate_lpf["r"] - self.length_rate_lpf["l"])'
    new = '(current_jump_rates["length"]["r"] - current_jump_rates["length"]["l"])'
    assert old in src
    src = src.replace(old, new)
    scope = dict(fn.__globals__)
    scope['current_jump_rates'] = current_jump_rates
    ns = {}
    src = src.replace('def _landing_supports(', 'def _landing_supports_jump_rate_probe(', 1)
    exec(compile(src, inspect.getsourcefile(fn), 'exec'), scope, ns)
    c._landing_supports = types.MethodType(ns['_landing_supports_jump_rate_probe'], c)
    # Sample first, then run the complete controller wrapper chain (including
    # captured theta reference) on the same simulation frame.
    final_step = c.step
    def step_with_jump_rate_probe(data, dt):
        sample_jump_rate_probe(data, dt)
        return final_step(data, dt)
    c.step = step_with_jump_rate_probe
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
    transitions=[]; max_pitch=0.; max_pitch_thrust=0.; max_rate=0.; stage_peaks={}; signed_p_min=999.; signed_p_max=-999.; max_thrust_theta_error=0.; max_air_theta_error=0.; peak_wheel_req=peak_wheel_cmd=0.; max_z=z0; min_z=z0; theta_sync_d_stats={}
    hip_requested_peak={'l1':0.,'l2':0.,'r1':0.,'r2':0.}; hip_applied_peak={'l1':0.,'l2':0.,'r1':0.,'r2':0.}; hip_over35_ticks=0; bilateral={}; hip_stage={}
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
        stage='landing_buffer' if c.landing_buffer_phase != 'inactive' else c.jump_state
        if c.jump_state=='thrust' and stage=='thrust': stage='thrust_powered' if not released else 'thrust_release'
        m=stage_peaks.setdefault(stage, {'pitch':0.0,'rate':0.0,'theta_rate':0.0,'length_rate':0.0})
        m['pitch']=max(m['pitch'],abs(p)); m['rate']=max(m['rate'],abs(rate))
        m['theta_rate']=max(m['theta_rate'],max(abs(math.degrees(x)) for x in current_jump_rates['theta'].values()))
        m['length_rate']=max(m['length_rate'],max(abs(x) for x in current_jump_rates['length'].values()))
        if c.jump_state=='thrust' and not c.jump_off_ground:
            ds=theta_sync_d_stats.setdefault(stage, {'signed_min':999.,'signed_max':-999.,'abs_max':0.,'samples':0})
            dv=float(getattr(c,'jump_probe_theta_sync_d',0.)); ds['samples']+=1; ds['signed_min']=min(ds['signed_min'],dv); ds['signed_max']=max(ds['signed_max'],dv); ds['abs_max']=max(ds['abs_max'],abs(dv))
        max_pitch=max(max_pitch,abs(p)); max_rate=max(max_rate,abs(rate))
        signed_p_min=min(signed_p_min,p); signed_p_max=max(signed_p_max,p)
        th=[math.pi/2-c.vmc_kinematics(d,s).phi0+c.body_pitch(d) for s in ('l','r')]
        theta_common=.5*sum(th); theta_diff=th[0]-th[1]
        lvals=[c.vmc_kinematics(d,s).l0 for s in ('l','r')]; length_diff=lvals[0]-lvals[1]
        bm=bilateral.setdefault(stage, {'theta_common_abs_max':0.,'theta_common_min':999.,'theta_common_max':-999.,'theta_diff_abs_max':0.,'theta_diff_min':999.,'theta_diff_max':-999.,'left_minus_right_length_abs_max':0.,'length_diff_min':999.,'length_diff_max':-999.,'samples':0})
        bm['samples']+=1; bm['theta_common_abs_max']=max(bm['theta_common_abs_max'],abs(theta_common)); bm['theta_common_min']=min(bm['theta_common_min'],theta_common); bm['theta_common_max']=max(bm['theta_common_max'],theta_common); bm['theta_diff_abs_max']=max(bm['theta_diff_abs_max'],abs(theta_diff)); bm['theta_diff_min']=min(bm['theta_diff_min'],theta_diff); bm['theta_diff_max']=max(bm['theta_diff_max'],theta_diff); bm['left_minus_right_length_abs_max']=max(bm['left_minus_right_length_abs_max'],abs(length_diff)); bm['length_diff_min']=min(bm['length_diff_min'],length_diff); bm['length_diff_max']=max(bm['length_diff_max'],length_diff)
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
        requests_now=[]
        hs=hip_stage.setdefault(stage, {'samples':0,'requests_over35':0,'applied_at_cap':0,'request_peak':{k:0. for k in hip_requested_peak},'applied_peak':{k:0. for k in hip_applied_peak}}); hs['samples']+=1
        for side,ids in c.sides.items():
            for j,act in enumerate(ids.hip_motors):
                label=f'{side}{j+1}'; val=abs(float(c.last_motor_torque[side][j])); requests_now.append((label,act,val))
                hip_requested_peak[label]=max(hip_requested_peak[label],val); hs['request_peak'][label]=max(hs['request_peak'][label],val)
                if val>35.0: hip_over35_ticks+=1; hs['requests_over35']+=1
        mujoco.mj_step(model,d)
        for label,act,req in requests_now:
            applied=abs(float(d.actuator_force[act])); hip_applied_peak[label]=max(hip_applied_peak[label],applied); hs['applied_peak'][label]=max(hs['applied_peak'][label],applied)
            if applied>=34.99: hs['applied_at_cap']+=1
    tail_raw=c._wheel_contacts(d)
    print(f'{name}: release={None if release_t is None else round(release_t-t0,3)}s; physical_takeoff={None if physical_t is None else round(physical_t-t0,3)}s; confirmed={None if confirm_t is None else round(confirm_t-t0,3)}s; release_to_confirm={None if release_t is None or confirm_t is None else round(confirm_t-release_t,3)}s; flight={None if physical_t is None or first_contact_t is None else round(first_contact_t-physical_t,3)}s (max contact gap {longest_no_contact:.3f}s); z0/apex/min={z0:.3f}/{max_z:.3f}/{min_z:.3f}m; |pitch|max={max_pitch:.2f}deg (THRUST {max_pitch_thrust:.2f}; signed min/max {signed_p_min:.2f}/{signed_p_max:.2f}); |rate|max={max_rate:.1f}deg/s; theta_err max thrust/air={max_thrust_theta_error:.3f}/{max_air_theta_error:.3f}rad; wheel_req/cmd={peak_wheel_req:.2f}/{peak_wheel_cmd:.2f}Nm; hip_limit={hip_limit:.0f}Nm request_peak={hip_requested_peak} applied_peak={hip_applied_peak} request_over35_count={hip_over35_ticks}; final_pitch/roll={math.degrees(c.body_pitch(d)):.2f}/{math.degrees(c.body_roll(d)):.2f}deg, final_contact={tail_raw}; state={c.jump_state}, fault={c.jump_fault}')
    print(f'{name} stage_peaks={stage_peaks}')
    print(f'{name} bilateral_stage={bilateral}')
    print(f'{name} hip_stage={hip_stage}')
    print(f'{name} grounded_thrust_theta_sync_D={theta_sync_d_stats}')


base = (.32,.36,.24,.06,.04)
common = dict(delayed=True, fw_support=True, full_theta=True, hip_limit=35.)
run('strict1ms-v2-length-only-legacy-grounded-thrust-dtheta', *base, theta_jump_scale=.2, length_jump_scale=1., observer_tau=.004, observer_length=True, **common)
run('strict1ms-v2-length-only-observer-grounded-thrust-dtheta', *base, theta_jump_scale=1., length_jump_scale=1., observer_tau=.004, observer_theta=True, observer_length=True, sync_theta_observer=True, **common)
