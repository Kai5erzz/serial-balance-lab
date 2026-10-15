"""Acceptance of a jump's actual airborne motion, independent of state labels."""
from math import isfinite, atan2, sin, cos, radians, degrees


def analyze_jump_yaw(rows, *, max_yaw_deg=5.0):
    """Measure CAD body heading from the accepted request through recovery.

    Wrapped world heading, rather than a commanded yaw or the gimbal heading,
    must satisfy the bound. The final idle sample before PRELOAD is the origin.
    """
    start = next((i for i, row in enumerate(rows) if row["state"][0] == 1), None)
    if start is None or start == 0:
        return {"passed": False, "errors": ["missing pre-jump heading sample"]}
    measured = rows[start-1:]
    if any(not isfinite(row.get("actual_yaw", float("nan"))) for row in measured):
        return {"passed": False, "errors": ["missing finite physical yaw"]}
    reference = measured[0]["actual_yaw"]
    delta = lambda row: degrees(atan2(sin(radians(row["actual_yaw"]-reference)),
                                     cos(radians(row["actual_yaw"]-reference))))
    air = [row for row in measured if row["state"][0] == 3 and sum(row["contact"]) < 1.0]
    maximum = max(abs(delta(row)) for row in measured)
    errors = [] if maximum <= max_yaw_deg else ["physical jump yaw exceeds bound"]
    if not air:
        errors.append("no physical flight for yaw measurement")
    return {"passed": not errors, "errors": errors,
            "reference_heading_deg": reference,
            "max_jump_yaw_deg": maximum,
            "max_air_yaw_deg": max((abs(delta(row)) for row in air), default=None),
            "final_yaw_error_deg": delta(measured[-1])}


def analyze_flight(rows, *, compact_length=.145, compact_hold_s=.080,
                   max_air_roll_deg=3.0):
    """Rows use seconds, metres, degrees, and measured MuJoCo contact forces.

    An extended wheel with zero contact is airborne even if the controller
    reports grounded. Likewise a FLIGHT label cannot stand in for a real tuck.
    This is only an acceptance observer; it never feeds back into control.
    """
    errors = []
    air = [r for r in rows if sum(r["contact"]) < 1.0]
    if not air:
        return {"passed": False, "errors": ["no physical flight"]}
    takeoff = air[0]["t"]
    contact = [r for r in rows if r["t"] > takeoff+.010
               and min(r["contact"][:2]) >= 5.0]
    touchdown = contact[0]["t"] if contact else None
    if touchdown is None:
        errors.append("no two-wheel touchdown")
    flight = [r for r in air if touchdown is None or r["t"] < touchdown]
    longest = elapsed = 0.0
    last = None
    compact_end = None
    first_deploy = None
    for r in rows:
        if r["t"] < takeoff or (touchdown is not None and r["t"] >= touchdown):
            continue
        if r["state"][3] & 256 and first_deploy is None:
            first_deploy = r["t"]
        compact = (sum(r["contact"]) < 1.0 and
                   all(isfinite(x) and .05 < x <= compact_length for x in r["length"]) and
                   all(isfinite(x) and .05 < x <= compact_length+.010
                       for x in r.get("cad_length", r["length"])) and
                   all(abs(x-.13) <= .002 for x in r["ref"]))
        dt = r["t"]-last["t"] if last is not None else 0.0
        if compact:
            if last is not None and last["compact"] and 0 < dt <= .012:
                elapsed += dt
            else:
                elapsed = 0.0
            longest = max(longest, elapsed)
            if elapsed >= compact_hold_s-1e-6 and compact_end is None:
                compact_end = r["t"]
        else:
            elapsed = 0.0
        last = {"t": r["t"], "compact": compact}
    if compact_end is None:
        errors.append("both legs did not complete the measured airborne compact dwell")
    if first_deploy is None:
        errors.append("no airborne landing deployment")
    elif compact_end is None or first_deploy < compact_end-1e-6:
        errors.append("deployment started before the measured compact dwell")
    deployment_extension = 0.0
    deployed_both_length = None
    if first_deploy is not None:
        anchors = [r for r in rows if takeoff <= r["t"] < first_deploy]
        if anchors:
            anchor = anchors[-1]
            for r in flight:
                if r["t"] < first_deploy:
                    continue
                growth = min(r["length"][i]-anchor["length"][i] for i in (0,1))
                if "cad_length" in r and "cad_length" in anchor:
                    growth = min(growth, *(r["cad_length"][i]-anchor["cad_length"][i] for i in (0,1)))
                deployment_extension = max(deployment_extension, growth)
                both = min(r["length"])
                deployed_both_length = max(deployed_both_length or 0.0, both)
        if deployment_extension < .020-1e-6:
            errors.append("both actual legs did not extend 20 mm before touchdown")
    early_landing = [r["t"] for r in rows if r["state"][0] == 4 and
                     (touchdown is None or r["t"] < touchdown-.002)]
    if early_landing:
        errors.append("controller entered LANDING before physical two-wheel contact")
    phases = []
    for r in rows:
        phase = r["state"][0]
        if not phases or phases[-1] != phase:
            phases.append(phase)
    if phases != [0, 1, 2, 3, 4, 0]:
        errors.append(f"incomplete or reordered jump phases: {phases}")
    roll = max(abs(r.get("actual_roll", r["roll"])) for r in flight)
    if roll > max_air_roll_deg:
        errors.append(f"airborne roll {roll:.2f} deg exceeds {max_air_roll_deg:.2f} deg")
    return {"passed": not errors, "errors": errors,
            "physical_takeoff_s": takeoff, "physical_touchdown_s": touchdown,
            "min_airborne_both_leg_max_m": min(max(r["length"]) for r in flight),
            "longest_actual_compact_hold_s": longest,
            "compact_hold_completed_s": compact_end,
            "deploy_started_s": first_deploy,
            "actual_bilateral_deploy_extension_m": deployment_extension,
            "airborne_deployed_both_length_m": deployed_both_length, "max_air_roll_deg": roll,
            "phases": phases}
