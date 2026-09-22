"""Terrain safety: off-ground detection, ledge-descent guard, and the firmware danger check.

Mixin part of :class:`balance_lab.control.controller.TraditionalLQR`; every
method accesses the shared controller state through ``self``.

Support-mass scope: these chassis_safety laws (off-ground force
reconstruction, LOW landing buffer) keep the firmware
``CHASSIS_SUPPORT_MASS`` they were ported with; the 2026-09-21 ruling mass
(13.592626 kg, 66.6718 N/leg) applies to the VMC support path in
controller.py -- see jump.py's module note for the measured reason the
jump window must not be re-anchored.
"""

from __future__ import annotations

import mujoco  # noqa: F401
import numpy as np

from .tuning import *  # noqa: F401,F403
from .kinematics import unit  # noqa: F401


class TerrainSafetyMixin:
    "Terrain safety: off-ground detection, ledge-descent guard, and the firmware danger check."

    def _ground_edge_distance_ahead(
        self, data: mujoco.MjData, direction: np.ndarray
    ) -> float | None:
        """Return the nearest contacted box-terrain edge along ``direction``.

        A force-only ledge detector is too noisy for this model: suspension
        motion on a flat tread can briefly reduce the reconstructed force by
        the same amount as a real drop.  The platform terrain is represented
        by fixed box geoms, so use the actual contacted box boundary as a
        geometric confirmation before applying the high-authority brake.
        ``None`` means the current ground is a plane or another terrain type
        without a usable box boundary.
        """
        direction = np.asarray(direction, dtype=float)
        norm = float(np.linalg.norm(direction[:2]))
        if norm <= 1e-9:
            return None
        direction = direction / norm
        wheel_bodies = {
            ids.wheel_body for ids in self.sides.values()
        }
        nearest: float | None = None
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            geom_ids = (int(contact.geom1), int(contact.geom2))
            terrain_ids = [
                geom_id for geom_id in geom_ids
                if geom_id in self.ground_geoms
                and self.model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_BOX
            ]
            wheel_ids = [
                geom_id for geom_id in geom_ids
                if int(self.model.geom_bodyid[geom_id]) in wheel_bodies
            ]
            if not terrain_ids or not wheel_ids:
                continue
            wheel_body = int(self.model.geom_bodyid[wheel_ids[0]])
            wheel_position = data.xpos[wheel_body]
            for terrain_id in terrain_ids:
                rotation = data.geom_xmat[terrain_id].reshape(3, 3)
                local_position = rotation.T @ (
                    wheel_position - data.geom_xpos[terrain_id]
                )
                local_direction = rotation.T @ direction
                distances: list[float] = []
                for axis in range(2):
                    component = float(local_direction[axis])
                    if component > 1e-6:
                        distances.append(
                            (self.model.geom_size[terrain_id, axis]
                             - local_position[axis]) / component
                        )
                    elif component < -1e-6:
                        distances.append(
                            (-self.model.geom_size[terrain_id, axis]
                             - local_position[axis]) / component
                        )
                if not distances:
                    continue
                distance = min(distances)
                if distance >= -0.05 and (
                    nearest is None or distance < nearest
                ):
                    nearest = max(0.0, float(distance))
        return nearest

    def _base_vertical_accel_raw(self, data: mujoco.MjData) -> float:
        """World-vertical acceleration of the free base (incl. gravity)."""
        if getattr(self, "base_lin_z_adr", None) is None:
            try:
                adr = self.model.jnt_qveladr[self._joint("base_free")]
            except Exception:
                adr = 0
            self.base_lin_z_adr = int(adr) + 5
        return float(data.qacc[self.base_lin_z_adr])

    def _begin_generic_airborne_profile(self, data: mujoco.MjData) -> None:
        """Start the non-jump airborne leg profile used by the firmware."""
        self.landing_airborne_elapsed = 0.0
        for side in ("l", "r"):
            length = self.vmc_kinematics(data, side).l0
            if not np.isfinite(length):
                length = self.normal_leg_length
            length = float(np.clip(length, 0.10, 0.34))
            # Do not add the virtual 20 N preload to the measured geometry at
            # the instant of takeoff.  On a ledge the measured length can
            # already be near the four-bar's 0.33 m extension singularity;
            # adding preload there commands still more extension just as the
            # upper tread is leaving the leg.  The air profile may deploy
            # toward 0.27 m later, but it must start no longer than that.
            self.landing_air_deploy_ref[side] = float(np.clip(
                min(length, LANDING_AIR_DEPLOY_LENGTH),
                0.10,
                LANDING_AIR_DEPLOY_LENGTH,
            ))

    def _update_generic_airborne_profile(self, dt: float) -> None:
        if not self.off_ground or dt <= 0.0:
            return
        self.landing_airborne_elapsed += dt
        for side in ("l", "r"):
            self.landing_air_deploy_ref[side] = self._move_towards(
                self.landing_air_deploy_ref[side],
                LANDING_AIR_DEPLOY_LENGTH,
                LANDING_AIR_DEPLOY_REF_RATE * dt,
            )

    def _begin_descent_guard(self) -> None:
        """Arm the edge-speed brake used for an uncommanded ledge drop."""
        self.descent_guard_active = True
        self.descent_guard_drop_seen = False
        self.descent_guard_preedge_elapsed = 0.0
        # Do not change the operator-selected leg length before the wheel has
        # actually crossed the edge.  With a wheel constrained on the upper
        # tread, shortening the four-bar lowers the chassis into that tread;
        # the leg collision then happens before the wheel can drop.  The
        # edge mode below keeps the selected length reference common-mode and
        # hands the wheel a small, direction-aware roll-over torque instead.
        self.descent_edge_profile_active = False
        for side in ("l", "r"):
            self.descent_edge_length_reference[side] = self.normal_leg_length

    def _update_descent_guard(
        self,
        data: mujoco.MjData,
        dt: float,
        fn_avg: float,
        raw_wheel_contacts: dict[str, bool],
        takeoff_edge: bool,
        landing_edge: bool,
    ) -> None:
        """Stop horizontal motion around a generic off-ground event.

        A normal LQR has no terrain preview, so its speed reference remains
        active right up to the ledge.  The force observer provides a short
        preview of a wheel unloading; this guard changes the reference to
        zero before the drop and applies a common wheel brake after contact.
        After a real drop it remains latched until an explicit stop/reset,
        preventing the next step from being reached with residual momentum.
        """
        if dt <= 0.0:
            return
        if getattr(self, "disable_descent_guard", False):
            # Opt-out for climbing scenarios: the uphill wall otherwise
            # reads as a drop edge and the approach brake kills the run-up.
            return

        # chassis_vx is nose-positive after step()'s single frame
        # translation, so the guard's speed sense matches the UI value.  The
        # previous negation made a genuine forward approach read as reverse
        # and the force-preview brake never armed before the ledge.
        ui_speed = self.chassis_vx
        requested_speed = self.drive_forward_target
        moving_towards_command = (
            abs(requested_speed) >= DESCENT_GUARD_MIN_SPEED
            and (
                abs(ui_speed) < 0.02
                or requested_speed * ui_speed >= 0.0
            )
        )
        heading, _ = self._heading(data)
        direction = heading if requested_speed >= 0.0 else -heading
        edge_distance = self._ground_edge_distance_ahead(data, direction)
        force_preview = (
            not self.off_ground
            and not self.descent_guard_active
            and moving_towards_command
            and edge_distance is not None
            and edge_distance <= DESCENT_GUARD_EDGE_LOOKAHEAD
        )
        if force_preview:
            self._begin_descent_guard()
        if takeoff_edge:
            if not self.descent_guard_active:
                self._begin_descent_guard()
            self.descent_mode_active = True
            self.descent_guard_drop_seen = True
        if landing_edge:
            if not self.descent_guard_active:
                self._begin_descent_guard()
            self.descent_guard_drop_seen = True

        if not self.descent_guard_active:
            return

        if self.off_ground:
            self.descent_mode_active = True
            self.descent_guard_drop_seen = True
            return

        if not self.descent_guard_drop_seen:
            self.descent_guard_preedge_elapsed += dt
            # A short force dip can occur without a real ledge.  Do not hold
            # the brake forever in that case; the next force dip will arm it
            # again if the vehicle reaches an actual edge.  A geometric edge
            # is different: while it is still in the look-ahead window the
            # guard must remain armed, otherwise the normal speed loop and
            # the preview guard fight each other and the vehicle dithers in
            # front of the riser.
            edge_still_near = (
                edge_distance is not None
                and edge_distance <= 1.5 * DESCENT_GUARD_EDGE_LOOKAHEAD
            )
            if (
                self.descent_guard_preedge_elapsed
                >= DESCENT_GUARD_PREEDGE_TIMEOUT
                and abs(ui_speed) <= 0.08
                and not edge_still_near
            ):
                self.descent_guard_active = False
                self.descent_guard_preedge_elapsed = 0.0
                self.descent_edge_profile_active = False
            return

        # Once a real drop has been observed, keep the safety latch active.
        # Returning the original drive command automatically is unsafe here:
        # it can reverse the vehicle back toward the last riser before the
        # operator has confirmed the new support surface.
        return

    def _generic_landing_supports(
        self, lengths: dict[str, float]
    ) -> dict[str, float]:
        """Landing-buffer axial support using the real LOW-leg gains."""
        supports: dict[str, float] = {}
        for side in ("l", "r"):
            length = lengths[side]
            spring = self._gas_virtual_support(length) if self.gas_enabled else 0.0
            length_force = float(np.clip(
                LANDING_LENGTH_KP * (self.gbuf_length_ref[side] - length)
                - LANDING_LENGTH_KD_LOW * self.length_rate_lpf[side],
                -LANDING_LENGTH_OUTPUT_LIMIT_LOW,
                LANDING_LENGTH_OUTPUT_LIMIT_LOW,
            ))
            support = (
                0.5 * TOTAL_MASS * GRAVITY  # firmware CHASSIS_SUPPORT_MASS (safety path)
                - spring
                + length_force
                + self.support_trim[side]
                + (self.roll_force if side == "l" else -self.roll_force)
            )
            # A gas spring can make the raw LOW-leg force negative while the
            # wheel has just moved to a lower tread and the chassis is still
            # above it.  Letting that negative force through removes the
            # vertical support exactly when the body is crossing the riser.
            # Keep a small bilateral support floor for the short landing
            # buffer; the normal standing PID resumes after it exits.
            supports[side] = float(np.clip(
                max(support, GBUF_SUPPORT_FLOOR),
                -SUPPORT_FORCE_LIMIT,
                SUPPORT_FORCE_LIMIT,
            ))
        return supports

    def _update_danger_check(self, data: mujoco.MjData, dt: float) -> None:
        """Port of chassis_safety.c Security_Checking().

        Hard attitude/leg-range violations are immediate; soft pitch/roll/
        theta candidates debounce while the chassis is grounded.  A confirmed
        danger state with the machine otherwise idle hands control to the
        recovery module (begin_recovery), the ported equivalent of the
        firmware DANGER -> CHASSIS_RECOVERY takeover.
        """
        if dt <= 0.0:
            return
        if getattr(self, "disable_danger_check", False):
            # Opt-out for scripted gaits that intentionally fold the legs.
            return
        pitch_deg = float(np.degrees(self.body_pitch(data)))
        roll_deg = float(np.degrees(self.body_roll(data)))
        reason = "none"
        if abs(pitch_deg) > DANGER_HARD_ATTITUDE_DEG:
            reason = "hard_pitch"
        elif abs(roll_deg) > DANGER_HARD_ATTITUDE_DEG:
            reason = "hard_roll"
        else:
            low = DANGER_LEG_SAFE_AREA_DEG
            high = 180.0 - low
            for side in ("l", "r"):
                phi = float(np.degrees(self.vmc_kinematics(data, side).phi0))
                if phi <= low or phi >= high:
                    reason = f"hard_{side}_phi"
                    break
        # The soft candidate keeps accumulating through touch-and-go
        # bounces: a fall that tumbles across terrain must still trigger the
        # instant the chassis finally settles grounded, not only when the
        # whole 0.08 s window happens to fit inside one contact spell.
        if reason == "none":
            candidate = "none"
            theta_bad = False
            for side in ("l", "r"):
                kin = self.vmc_kinematics(data, side)
                theta = np.pi / 2.0 - kin.phi0 + self.body_pitch(data)
                if abs(theta) > DANGER_SOFT_THETA_RAD:
                    theta_bad = True
                    break
            if abs(pitch_deg) > DANGER_SOFT_PITCH_DEG:
                candidate = "soft_pitch"
            elif abs(roll_deg) > DANGER_SOFT_ROLL_DEG:
                candidate = "soft_roll"
            elif theta_bad:
                candidate = "soft_theta"
            if candidate != "none":
                if candidate != getattr(self, "_danger_candidate", "none"):
                    self._danger_candidate = candidate
                    self._danger_timer = 0.0
                self._danger_timer = getattr(self, "_danger_timer", 0.0) + dt
                if self._danger_timer >= DANGER_SOFT_CONFIRM_TIME:
                    reason = candidate
            else:
                self._danger_candidate = "none"
                self._danger_timer = 0.0

        if reason != "none":
            self.danger_reason = reason
            self.danger_confirmed = True
            if (
                self.recovery_state == "idle"
                and self.jump_state == "idle"
                and self.enabled
                and not self.off_ground
                and not self.jump_off_ground
            ):
                # The firmware chassis task routes DANGER to the recovery
                # module; do the same with the ported self-rescue machine.
                self.begin_fall_recovery(data)
        else:
            self.danger_reason = "none"
            self.danger_confirmed = False

    def _update_off_ground(self, data: mujoco.MjData, dt: float) -> None:
        """Real-vehicle off-ground detection and airborne handling.

        Port of chassis_safety.c: each wheel's vertical contact force is
        reconstructed from the commanded VMC axial force, gas-spring support,
        leg torque and the unsprung-mass vertical acceleration, filtered over
        four samples, and debounced before setting the off-ground flag.
        While airborne the wheels are shut, the hips use the real bounded LQR
        plus theta-hold command, the axial force follows the real 0.27 m
        deploy profile, and the integrators are frozen.  A landing edge starts
        the real LOW-leg COMPRESS->HOLD->RETURN buffer.

        Only active when the jump machine is idle (stair / ledge drops).
        """
        if self.jump_state != "idle":
            return
        if not self.enabled:
            return
        if dt <= 0.0:
            return

        # The start-up window only disarms the Ground_Detect decision below.
        # The support-force reconstruction itself keeps running: freezing the
        # four-sample history instead would leave cold-start values in the
        # filter, and the first live evaluation after the window would then
        # misread a genuine stand-up transient as takeoff (observed as an
        # immediate airborne deploy hop at t~3 s on the platform scene).
        self.offground_suppress_elapsed += dt

        pitch = self.body_pitch(data)
        base_az = self._base_vertical_accel_raw(data)
        raw_wheel_contacts = self._wheel_contacts(data)

        for side in ("l", "r"):
            kin = self.vmc_kinematics(data, side)
            theta = np.pi / 2.0 - kin.phi0 + pitch
            l0 = max(kin.l0, 0.08)
            d_l0 = self.length_rate_lpf[side]
            d_theta = self.theta_rate_lpf[side]
            if self.offground_deriv_init:
                dd_l0 = (d_l0 - self.offground_last_d_l0[side]) / dt
                dd_theta = (d_theta - self.offground_last_d_theta[side]) / dt
            else:
                dd_l0 = 0.0
                dd_theta = 0.0
            self.offground_last_d_l0[side] = d_l0
            self.offground_last_d_theta[side] = d_theta
            leg_vertical_accel = (
                base_az
                - dd_l0 * np.cos(theta)
                + 2.0 * d_l0 * d_theta * np.sin(theta)
                + l0 * dd_theta * np.sin(theta)
                + l0 * d_theta * d_theta * np.cos(theta)
            )
            support = self.last_support[side]
            tp = self.last_output[side][1]
            fn = (
                support * np.cos(theta)
                + self._gas_virtual_support(l0)
                + tp * np.sin(theta) / l0
                + OFFGROUND_UNSPRUNG_MASS * leg_vertical_accel
            )
            self.offground_fn_raw[side] = (
                fn if np.isfinite(fn) else 0.5 * TOTAL_MASS * GRAVITY  # firmware CHASSIS_SUPPORT_MASS (safety path)
            )
            self.offground_fn_history[side][self.offground_filter_index] = (
                self.offground_fn_raw[side]
            )
        self.offground_filter_index = (self.offground_filter_index + 1) % 4
        self.offground_deriv_init = True
        for side in ("l", "r"):
            self.offground_fn_filtered[side] = (
                sum(self.offground_fn_history[side]) / 4.0
            )
        fn_avg = 0.5 * sum(self.offground_fn_filtered.values())
        support_contacts = self._wheel_support_contacts(data)
        # The generic airborne branch normally shuts the drive wheels and
        # uses them only as a reaction axis.  During a guarded stair descent,
        # however, a wheel may regain a real vertical tread contact while the
        # body is still outside the conservative landing attitude basin.  In
        # that short interval a small common drive torque is physically
        # meaningful and lets the wheel roll off the next tread; it is never
        # enabled for a riser scrape or while both wheels are unsupported.
        self.descent_edge_contact_drive = bool(
            self.descent_guard_active
            and self.off_ground
            and any(support_contacts.values())
            and self.gbuf_phase == "inactive"
        )
        heading, _ = self._heading(data)
        direction = (
            heading if self.drive_forward_target >= 0.0 else -heading
        )
        edge_distance = self._ground_edge_distance_ahead(data, direction)
        edge_hold_preview = (
            self.descent_guard_active
            and not self.off_ground
            and edge_distance is not None
            and edge_distance <= DESCENT_GUARD_EDGE_HOLD
        )
        edge_contact_loss = (
            self.descent_guard_active
            and self.descent_edge_transition_active
            and not any(support_contacts.values())
            and not any(raw_wheel_contacts.values())
        )
        if edge_contact_loss:
            self.descent_edge_loss_count += 1
        else:
            self.descent_edge_loss_count = 0
        if self.gbuf_phase != "inactive":
            if all(support_contacts.values()):
                self.gbuf_contact_loss_count = 0
            else:
                self.gbuf_contact_loss_count += 1
        self.descent_edge_transition_active = bool(
            self.descent_guard_active
            and not self.off_ground
            and (
                self.descent_edge_transition_active
                or not all(support_contacts.values())
                or edge_hold_preview
            )
        )

        # debounced state machine (Ground_Detect)
        detector_armed = (
            self.offground_suppress_elapsed >= OFFGROUND_STARTUP_SUPPRESS_S
        )
        if not detector_armed:
            self.off_ground = False
            self.off_ground_cnt = 0
        elif (
            not self.off_ground
            and self.gbuf_phase == "inactive"
        ):
            force_takeoff_candidate = (
                fn_avg < OFFGROUND_TAKEOFF_FN_LOW
                and self.offground_fn_filtered["l"] < OFFGROUND_JUMP_TAKEOFF_SIDE_MAX
                and self.offground_fn_filtered["r"] < OFFGROUND_JUMP_TAKEOFF_SIDE_MAX
            )
            # During the geometric pre-edge clearance motion the gas-spring
            # cancellation can make the reconstructed virtual force negative
            # even though both wheels are still visibly carrying the chassis.
            # That is a support-model transient, not takeoff.  Let the real
            # contact geometry own this particular handoff; otherwise the
            # controller declares airborne while parked at the ledge.
            if (
                self.descent_edge_profile_active
                and self.descent_guard_active
                and not self.descent_guard_drop_seen
            ):
                force_takeoff_candidate = False
            # A ledge drop is often unilateral at first: one wheel is still
            # on the upper tread while the other has already crossed the
            # edge.  Waiting for both reconstructed forces to become small
            # leaves the normal ground VMC in charge during that interval;
            # its wheel/leg loop then treats the riser scrape as a pose error
            # and winds the chassis into roll.  Once the geometric edge guard
            # is active, a real vertical tread on only one wheel is enough to
            # hand the transition to the airborne profile.  The same normal
            # and contact checks are retained for the eventual landing.
            unilateral_edge_candidate = (
                DESCENT_UNILATERAL_TAKEOFF_ENABLE
                and
                self.descent_guard_active
                and self.descent_edge_transition_active
                and any(support_contacts.values())
                and not all(support_contacts.values())
                and any(raw_wheel_contacts.values())
                and edge_distance is not None
                and edge_distance <= DESCENT_GUARD_EDGE_HOLD
            )
            # Once both wheels have actually left the vertical treads, the
            # force observer still contains the previous axial command and
            # can take several hundred milliseconds to fall below its
            # threshold.  Keep the normal LQR from chasing that stale force:
            # the guarded edge transition plus three consecutive contact-free
            # samples is a physical takeoff confirmation for this scene.
            geometry_edge_candidate = (
                self.descent_edge_loss_count
                >= DESCENT_EDGE_TAKEOFF_CONFIRM
            )
            takeoff_candidate = (
                force_takeoff_candidate
                or unilateral_edge_candidate
                or geometry_edge_candidate
            )
            if takeoff_candidate:
                self.off_ground_cnt += 1
                if self.off_ground_cnt >= OFFGROUND_TAKEOFF_CONFIRM_LOW:
                    self.off_ground = True
                    self.off_ground_cnt = 0
            else:
                self.off_ground_cnt = 0
        elif self.off_ground:
            self.descent_edge_transition_active = False
            landing_attitude_stable = (
                abs(pitch) <= OFFGROUND_LANDING_ATTITUDE_MAX
                and abs(self.body_roll(data)) <= OFFGROUND_LANDING_ATTITUDE_MAX
                and abs(float(data.qvel[2]))
                <= OFFGROUND_LANDING_VERTICAL_SPEED_MAX
            )
            force_landing = (
                self.offground_fn_filtered["l"] > OFFGROUND_LANDING_FN_LOW
                and self.offground_fn_filtered["r"] > OFFGROUND_LANDING_FN_LOW
                and all(support_contacts.values())
                and landing_attitude_stable
            )
            if force_landing:
                self.off_ground_cnt += 1
                if self.off_ground_cnt >= OFFGROUND_LANDING_CONFIRM_LOW:
                    self.off_ground = False
                    self.off_ground_cnt = 0
            else:
                self.off_ground_cnt = 0
        else:
            # The LOW landing buffer owns the contact transition.  Its
            # captured support force can dip below the takeoff threshold as
            # the legs compress; allowing Ground_Detect to run again here
            # would re-enter airborne mode in the middle of a landing and
            # produce the observed repeated hop / runaway drive.
            self.off_ground_cnt = 0
            self.off_ground = False
            self.descent_edge_transition_active = False

        # The generic landing buffer is allowed to absorb the first tread,
        # but it must not mask a second edge.  A short debounce filters the
        # occasional missing contact normal from solver contact churn; a
        # sustained loss means the wheels are on a riser or airborne again.
        # Re-open the airborne path before calculating the edge transition so
        # the next landing can create a fresh buffer of its own.
        if (
            detector_armed
            and self.gbuf_phase != "inactive"
            and self.descent_guard_active
            and self.gbuf_contact_loss_count >= GBUF_SUPPORT_LOSS_CONFIRM
        ):
            self.gbuf_phase = "inactive"
            self.gbuf_elapsed = 0.0
            self.gbuf_exit_stable = 0.0
            self.gbuf_contact_loss_count = 0
            self.off_ground = True
            self.off_ground_cnt = 0
            self.descent_edge_transition_active = False
            self.descent_edge_loss_count = 0

        takeoff_edge = self.off_ground and not self.off_ground_last
        landing_edge = (not self.off_ground) and self.off_ground_last
        self.off_ground_last = self.off_ground

        if takeoff_edge:
            self._begin_generic_airborne_profile(data)
        self._update_generic_airborne_profile(dt)

        # landing buffer state machine (generic airborne path)
        if self.gbuf_phase != "inactive":
            self.gbuf_elapsed += dt
            lengths = {
                side: self.vmc_kinematics(data, side).l0 for side in ("l", "r")
            }
            average_speed = 0.5 * sum(
                abs(self.length_rate_lpf[side]) for side in ("l", "r")
            )
            if self.gbuf_phase == "compress":
                # Keep the captured contact geometry during compression.  The
                # real LOW buffer lets the physical leg shorten against that
                # fixed reference; it does not immediately pull the reference
                # back to the normal standing length.
                for side in ("l", "r"):
                    self.gbuf_length_ref[side] = self._move_towards(
                        self.gbuf_length_ref[side],
                        self.gbuf_absorb_ref[side],
                        LANDING_COMPRESS_REF_RATE * dt,
                    )
                if self.gbuf_elapsed >= GBUF_COMPRESS_TIME:
                    self.gbuf_phase = "hold"
                    self.gbuf_elapsed = 0.0
            elif self.gbuf_phase == "hold":
                for side in ("l", "r"):
                    ratchet_target = min(
                        lengths[side] + LANDING_LOW_RATCHET_MARGIN,
                        self.normal_leg_length,
                    )
                    # Never command the reference back beyond the captured
                    # contact geometry while the body is still absorbing the
                    # drop.  Otherwise the measured leg extension ratchets
                    # the reference upward, launches the chassis, and turns
                    # the landing buffer into a second jump.
                    ratchet_target = min(
                        ratchet_target, self.gbuf_length_ref[side]
                    )
                    self.gbuf_length_ref[side] = self._move_towards(
                        self.gbuf_length_ref[side],
                        ratchet_target,
                        LANDING_LOW_RATCHET_REF_RATE * dt,
                    )
                settled = (
                    abs(0.5 * sum(lengths.values())
                        - 0.5 * sum(self.gbuf_length_ref.values()))
                    <= GBUF_SETTLE_LENGTH_ERROR
                    and average_speed <= GBUF_SETTLE_LENGTH_SPEED
                )
                if (
                    self.gbuf_elapsed >= GBUF_HOLD_MAX_TIME
                    or (
                        self.gbuf_elapsed >= GBUF_HOLD_MIN_TIME and settled
                    )
                ):
                    self.gbuf_phase = "return"
                    self.gbuf_elapsed = 0.0
            elif self.gbuf_phase == "return":
                for side in ("l", "r"):
                    self.gbuf_length_ref[side] = self._move_towards(
                        self.gbuf_length_ref[side],
                        self.normal_leg_length,
                        GBUF_RETURN_REF_RATE * dt,
                    )
                target_reached = all(
                    abs(v - self.normal_leg_length) <= 0.0005
                    for v in self.gbuf_length_ref.values()
                )
                if (
                    target_reached
                    and average_speed <= GBUF_SETTLE_LENGTH_SPEED
                ):
                    self.gbuf_exit_stable += dt
                else:
                    self.gbuf_exit_stable = 0.0
                if (
                    target_reached
                    and self.gbuf_elapsed >= GBUF_PID_RETURN_TIME
                    and self.gbuf_exit_stable >= GBUF_EXIT_STABLE_TIME
                ):
                    self.gbuf_phase = "inactive"
                    self.gbuf_elapsed = 0.0
                    self.gbuf_exit_stable = 0.0

        # A ledge guard is scoped to one physical edge.  Once the captured
        # landing reference has returned and the IMU/velocity are settled,
        # release the guard so the operator's still-held command can approach
        # the next step at the normal controlled speed.  An explicit stop is
        # still sufficient to clear it immediately.
        if (
            self.descent_guard_active
            and self.descent_guard_drop_seen
            and not self.off_ground
            and self.gbuf_phase == "inactive"
            and abs(self.chassis_vx) <= DESCENT_GUARD_MIN_SPEED
            and abs(pitch) <= OFFGROUND_LANDING_ATTITUDE_MAX
            and abs(self.body_roll(data)) <= OFFGROUND_LANDING_ATTITUDE_MAX
        ):
            self.descent_guard_active = False
            self.descent_mode_active = False
            self.descent_guard_drop_seen = False
            self.descent_guard_preedge_elapsed = 0.0
            self.descent_edge_profile_active = False

        if landing_edge:
            self.descent_edge_transition_active = False
            self.gbuf_phase = "compress"
            self.gbuf_elapsed = 0.0
            self.gbuf_exit_stable = 0.0
            self.gbuf_contact_loss_count = 0
            refs = {
                side: float(np.clip(
                    self.vmc_kinematics(data, side).l0, 0.10, 0.34
                ))
                for side in ("l", "r")
            }
            ref_max = min(refs.values()) + LANDING_LOW_CONTACT_SPLIT_MAX
            for side in ("l", "r"):
                self.gbuf_contact_length[side] = self.vmc_kinematics(
                    data, side
                ).l0
                refs[side] = min(refs[side], ref_max)
                self.gbuf_length_ref[side] = refs[side]
                self.gbuf_absorb_ref[side] = refs[side]

        self._update_descent_guard(
            data,
            dt,
            fn_avg,
            raw_wheel_contacts,
            takeoff_edge,
            landing_edge,
        )

        if (
            self.descent_edge_transition_active
            and not self.off_ground
            and self.gbuf_phase == "inactive"
        ):
            # The upper tread can stop carrying one wheel a few samples
            # before Ground_Detect declares takeoff.  During that interval a
            # normal hip row is still based on the old ground equilibrium and
            # lets the chassis rotate into the riser.  Apply the same bounded
            # common pitch hold used immediately after takeoff, but keep the
            # axial support and wheel shutoff in their contact-safe modes.
            pitch_rate = self.body_pitch_rate(data)
            edge_pitch_tp = float(np.clip(
                DESCENT_EDGE_PITCH_BIAS
                - OFFGROUND_PITCH_KP * pitch
                - OFFGROUND_PITCH_KD * pitch_rate,
                -OFFGROUND_PITCH_TP_LIMIT,
                OFFGROUND_PITCH_TP_LIMIT,
            ))
            for side in ("l", "r"):
                self.last_output[side][1] = edge_pitch_tp

        # --- airborne / buffer output override ---
        if self.descent_edge_transition_active and self.gbuf_phase == "inactive":
            # The normal LQR can spike when the wheel contact point reaches a
            # sharp box edge.  Pass only a small common roll-over torque to
            # the motors; this keeps the wheel moving over the edge without
            # allowing the contact impulse to become a propulsion command.
            direction_sign = (
                1.0 if self.drive_forward_target >= 0.0 else -1.0
            )
            edge_wheel = -direction_sign * DESCENT_EDGE_WHEEL_TORQUE
            self.last_output["l"][0] = edge_wheel
            self.last_output["r"][0] = edge_wheel
        self.airborne_wheel_shut = self.off_ground
        if self.off_ground:
            # The wheels are still useful in free flight as a reaction wheel:
            # use their angular momentum to oppose chassis pitch while the
            # hips deploy the legs.  This is internal attitude control only;
            # it does not assume a wheel-ground contact.
            pitch_rate = self.body_pitch_rate(data)
            self.offground_wheel_torque = float(np.clip(
                OFFGROUND_WHEEL_PITCH_KP * pitch
                + OFFGROUND_WHEEL_PITCH_KD * pitch_rate,
                -OFFGROUND_WHEEL_TORQUE_LIMIT,
                OFFGROUND_WHEEL_TORQUE_LIMIT,
            ))
            lengths = {
                side: self.vmc_kinematics(data, side).l0
                for side in ("l", "r")
            }
            air_support = {}
            raw_air_tp = {}
            for side in ("l", "r"):
                kin = self.vmc_kinematics(data, side)
                theta = np.pi / 2.0 - kin.phi0 + pitch
                # The real generic airborne branch zeros the wheel row and
                # retains only the hip theta/theta-rate rows.  Do not reuse
                # the already-computed normal-ground Tp here: it also contains
                # body-pitch feedback, which the firmware explicitly removes
                # for non-jump airborne motion.
                lqr_row = self.gain(kin.l0)[1]
                lqr_tp_raw = float(
                    lqr_row[0] * self.last_state[side][0]
                    + lqr_row[1] * self.last_state[side][1]
                )
                lqr_tp = float(np.clip(
                    OFFGROUND_LQR_TP_GAIN * lqr_tp_raw,
                    -OFFGROUND_LQR_TP_LIMIT,
                    OFFGROUND_LQR_TP_LIMIT,
                ))
                theta_hold = float(np.clip(
                    OFFGROUND_THETA_HOLD_KP * theta
                    + OFFGROUND_THETA_HOLD_KD * self.theta_rate_lpf[side],
                    -OFFGROUND_THETA_HOLD_TP_LIMIT,
                    OFFGROUND_THETA_HOLD_TP_LIMIT,
                ))
                pitch_rate = self.body_pitch_rate(data)
                pitch_tp = float(np.clip(
                    -OFFGROUND_PITCH_KP * pitch
                    - OFFGROUND_PITCH_KD * pitch_rate,
                    -OFFGROUND_PITCH_TP_LIMIT,
                    OFFGROUND_PITCH_TP_LIMIT,
                ))
                raw_air_tp[side] = lqr_tp + theta_hold + pitch_tp
                spring = self._gas_virtual_support(kin.l0) if self.gas_enabled else 0.0
                air_force = OFFGROUND_SOFT_FORCE - spring
                deploy_force = float(np.clip(
                    LANDING_AIR_DEPLOY_KP * (
                        self.landing_air_deploy_ref[side] - kin.l0
                    )
                    - LANDING_AIR_DEPLOY_KD * self.length_rate_lpf[side]
                    - OFFGROUND_SOFT_FORCE,
                    -LANDING_AIR_DEPLOY_FORCE_LIMIT,
                    LANDING_AIR_DEPLOY_FORCE_LIMIT,
                ))
                air_support[side] = air_force + deploy_force

            common_support = 0.5 * (air_support["l"] + air_support["r"])
            differential_support = 0.5 * (
                air_support["l"] - air_support["r"]
            )
            self.airborne_length_sync_force = float(np.clip(
                OFFGROUND_LENGTH_SYNC_KP * (lengths["r"] - lengths["l"])
                + OFFGROUND_LENGTH_SYNC_KD * (
                    self.length_rate_lpf["r"] - self.length_rate_lpf["l"]
                ),
                -OFFGROUND_LENGTH_SYNC_LIMIT,
                OFFGROUND_LENGTH_SYNC_LIMIT,
            ))
            air_support["l"] = common_support + differential_support + self.airborne_length_sync_force
            air_support["r"] = common_support - differential_support - self.airborne_length_sync_force
            for side in ("l", "r"):
                self.last_output[side][0] = 0.0
                self.airborne_lqr_tp[side] = float(
                    np.clip(raw_air_tp[side], -OFFGROUND_LQR_TP_LIMIT,
                            OFFGROUND_LQR_TP_LIMIT)
                )
                self.airborne_theta_hold_tp[side] = float(
                    raw_air_tp[side] - lqr_tp
                )
                self.last_output[side][1] = self.airborne_lqr_tp[side]
                self.last_support[side] = float(np.clip(
                    air_support[side],
                    -OFFGROUND_AIR_RETRACT_LIMIT,
                    OFFGROUND_SPRING_COMP_LIMIT,
                ))
            if self.descent_guard_active and any(support_contacts.values()):
                # Apply the load-transfer term after the bilateral air-force
                # clamp.  Before this point both sides can sit at the
                # negative retract limit, which silently discards the entire
                # differential correction and lets the supported wheel tip
                # the chassis.  A riser scrape is excluded by the vertical
                # contact mask.
                edge_roll_force = float(np.clip(
                    OFFGROUND_ROLL_KP_N_PER_DEG
                    * np.degrees(self.body_roll(data))
                    + OFFGROUND_ROLL_KD_N_PER_DEG_S
                    * self.roll_rate_lpf_deg_s,
                    -OFFGROUND_ROLL_FORCE_LIMIT,
                    OFFGROUND_ROLL_FORCE_LIMIT,
                ))
                self.last_support["l"] = float(np.clip(
                    self.last_support["l"] + edge_roll_force,
                    -OFFGROUND_AIR_RETRACT_LIMIT,
                    OFFGROUND_SPRING_COMP_LIMIT,
                ))
                self.last_support["r"] = float(np.clip(
                    self.last_support["r"] - edge_roll_force,
                    -OFFGROUND_AIR_RETRACT_LIMIT,
                    OFFGROUND_SPRING_COMP_LIMIT,
                ))
            air_roll_tp = float(np.clip(
                -OFFGROUND_ROLL_TP_KP_NM_PER_DEG
                * np.degrees(self.body_roll(data))
                - OFFGROUND_ROLL_TP_KD_NM_PER_DEG_S
                * self.roll_rate_lpf_deg_s,
                -OFFGROUND_ROLL_TP_LIMIT,
                OFFGROUND_ROLL_TP_LIMIT,
            ))
            pitch_rate = self.body_pitch_rate(data)
            air_pitch_extra_tp = float(np.clip(
                -OFFGROUND_PITCH_TP_EXTRA_KP_NM_PER_DEG
                * np.degrees(pitch)
                - OFFGROUND_PITCH_TP_EXTRA_KD_NM_PER_DEG_S
                * pitch_rate,
                -OFFGROUND_PITCH_TP_EXTRA_LIMIT,
                OFFGROUND_PITCH_TP_EXTRA_LIMIT,
            ))
            self.last_output["l"][1] += air_roll_tp
            self.last_output["r"][1] -= air_roll_tp
            self.last_output["l"][1] += air_pitch_extra_tp
            self.last_output["r"][1] += air_pitch_extra_tp
        elif self.gbuf_phase != "inactive":
            self.offground_wheel_torque = 0.0
            # Contact is already confirmed by the landing edge.  Keep the
            # wheel LQR alive while the LOW axial landing reference returns.
            lengths = {
                side: self.vmc_kinematics(data, side).l0
                for side in ("l", "r")
            }
            self.last_support = self._generic_landing_supports(lengths)
            for side in ("l", "r"):
                self.length_reference[side] = self.gbuf_length_ref[side]
            if self.descent_guard_active:
                pitch_rate = self.body_pitch_rate(data)
                gbuf_pitch_tp = float(np.clip(
                    -GBUF_PITCH_KP * pitch - GBUF_PITCH_KD * pitch_rate,
                    -GBUF_PITCH_TP_LIMIT,
                    GBUF_PITCH_TP_LIMIT,
                ))
                gbuf_brake = float(np.clip(
                    # A brake must oppose the measured nose-positive
                    # Kept in the firmware-frame sense of the rest of this
                    # port: the accepted stationary-jump dynamics couple
                    # through this exact expression.
                    -GBUF_BRAKE_KP * (-self.chassis_vx),
                    -GBUF_BRAKE_LIMIT,
                    GBUF_BRAKE_LIMIT,
                ))
                for side in ("l", "r"):
                    self.last_output[side][0] = gbuf_brake
                    # The landing buffer owns the hip row.  Adding its
                    # pitch correction on top of the previous normal-LQR
                    # output and theta synchronizer can leave a stale
                    # command of tens of Nm in ``last_output``; the actuator
                    # then saturates and the buffer itself becomes a second
                    # attitude impulse.  Keep only the bounded common pitch
                    # damper while the captured support reference settles.
                    self.last_output[side][1] = gbuf_pitch_tp
            # freeze leg-length / theta integrators (firmware behavior)
        else:
            self.offground_wheel_torque = 0.0
        if self.off_ground or self.gbuf_phase != "inactive":
            for side in ("l", "r"):
                if hasattr(self, "length_integral"):
                    self.length_integral[side] = 0.0
