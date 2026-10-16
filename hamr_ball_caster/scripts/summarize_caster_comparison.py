#!/usr/bin/env python3
"""Deterministic partial/final scientific summary of recorded caster pairs."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics
import xml.etree.ElementTree as ET


PATH_EXPLANATIONS = {
    "straight_y": "Straight motion mainly tests startup acceleration and steady tracking. Both caster models can support this direction; small residuals include feedback age and sampled control. A flat floor does not remove the configured wheel acceleration limit.",
    "horizontal_x": "The chassis starts at +90 degrees while the requested path points along +X. The tracked point is ahead of the rear axle, so chassis turning and position control are coupled. A swivel must passively reorient; the ball has no corresponding forward-facing angle.",
    "reversal_180": "The reference abruptly changes travel direction. The ordinary caster can first roll backward along its existing axis, then swivel through a brief sideways transient. Both architectures still need the rear wheels to reverse through the same acceleration limit.",
    "horizontal_shuttle": "Fast, short alternating horizontal legs leave little recovery time and repeatedly clip drive commands. Replacing the front contact mechanism cannot make an infeasible discontinuous velocity schedule exactly trackable.",
    "square_360": "Longer straight sides permit recovery between abrupt right-angle corners. The ordinary casters need different local velocities during each chassis turn; neither is required to point exactly along the robot-center tangent.",
    "circle_large_360": "The larger circular route permits a steady nonzero chassis/path heading offset in the ideal offset-drive model. Visible caster/path angular mismatch can be ordinary turning geometry rather than sideways rolling.",
    "circle_tight_three_laps": "The 0.20 m radius is smaller than the 0.27114 m axle-to-tracked-point offset. A constant chassis/path heading offset has no ideal solution. The same drive geometry remains with ball casters; changing the caster does not remove that condition.",
    "figure_eight": "Curvature changes sign through the crossing and continually changes the two caster locations' velocities. Evaluate position tracking and local mechanism behavior, rather than expecting either caster to match one common robot-center heading.",
    "hairpin": "The narrow return combines large heading change with corner transitions. Chassis rotation, the front contact mechanism and drive slew all contribute; the shape differs from a straight backward-rolling reversal.",
    "sharp_zigzag": "Rapid alternating sharp corners demand frequent velocity and yaw-rate changes. The ordinary run is strongly drive-limited, so removing a swivel-orientation transient alone cannot remove all tracking lag.",
    "star_reversals": "Repeated acute corners and large direction changes excite reorientation while the chassis accumulates large yaw changes. Ball joints avoid a single swivel-facing requirement but retain inertia, contact dynamics and drive limits.",
    "tiny_square_four_laps": "Each side is only 0.12 m, shorter than the tracked-point offset. At 0.25 m/s, corners arrive every 0.48 s and the ordinary controller is frequently limited. RMS improvement may coexist with worse individual corner peaks.",
    "smooth_s_curve": "The smoother curvature sequence gives both mechanisms more time to respond and generally demands less abrupt drive action. It is still implemented by polygonal waypoints and sampled feedback, so a perfectly flat floor does not imply zero error.",
    "tiny_square_slow": "The same small square is slowed to 0.08 m/s. The ordinary model tracks position well but has repeated passive caster reorientation transients; along-track and cross-track errors identify which part of tracking the ball actually changes.",
    "tight_circle_slow": "Slowing the tight circle reduces drive demands, but radius remains smaller than the axle offset. The geometric heading condition survives the speed change and the caster substitution.",
    "horizontal_shuttle_half_speed": "This retains the shuttle geometry at 0.20 m/s. Reduced command clipping separates some timing effects from front-contact behavior, while every reversal still has finite acceleration and passive dynamics.",
    "horizontal_shuttle_slow": "At 0.08 m/s the shuttle has little drive limiting and small position error. Passive swivel flips can take longer in seconds at lower local speed; a ball has no single forward-facing gate, but drive reversals remain finite-time events.",
    "reversal_repeat": "This is a fresh nominal reversal recording. Compare it with the original reversal to expose run-to-run variation before interpreting small differences as an architectural advantage.",
}

REPEAT_GROUPS = {
    "nominal_ball_tiny_square": ["tiny_square_four_laps", "swivel_drag_10x", "swivel_coulomb_0p1", "inertia_3x", "caster_grip_0p2"],
    "nominal_ball_reversal": ["reversal_180", "reversal_repeat", "reversal_drag_10x", "reversal_inertia_3x"],
}


def number(value, scale=1., digits=2):
    return "—" if value is None else f"{value*scale:.{digits}f}"


def route_explanation(case):
    name = case["name"]
    if name in PATH_EXPLANATIONS:
        return PATH_EXPLANATIONS[name]
    if case["category"] == "initial_heading":
        if case.get("startup_policy") == "none":
            return "The unassisted 270-degree start permits backward chassis travel. A ball has no swivel-facing constraint, but its presence does not command the chassis to turn 180 degrees. Compare measured chassis recovery separately from path tracking."
        return "Both runs are configured to start at the requested chassis yaw and perform physical chassis recovery plus lead-in. Only the ordinary caster uses a swivel-facing release gate. Prepared source positions can differ; later legs are the primary comparison and first-leg/startup values are reported separately."
    if case["category"] == "sensitivity":
        override = ", ".join(case.get("swivel_xacro_overrides", [])) or "the saved physical override"
        return f"The ordinary-caster run changes {override}. There is no one-to-one equivalent intervention on the split-ball mechanism, so this ball recording retains nominal parameters. It is a repeated baseline, not evidence that the same parameter change improves a ball caster."
    return case.get("comparison_note", "Read the saved route and protocol when interpreting this pair.")


def distribution(values):
    if not values:
        return {"count": 0, "mean": None, "sample_std": None, "min": None, "max": None}
    return {"count": len(values), "mean": statistics.mean(values),
            "sample_std": statistics.stdev(values) if len(values) > 1 else None,
            "min": min(values), "max": max(values)}


def model_masses(directory):
    path = Path(directory)/"robot.urdf"
    if not path.exists():
        return None
    model = ET.parse(path).getroot()
    masses = {link.get("name"): float(link.find("inertial/mass").get("value"))
              for link in model.findall("link") if link.find("inertial/mass") is not None}
    return {"whole_model_kg": sum(masses.values()),
            "left_caster_group_kg": sum(value for name, value in masses.items() if name.startswith("left_caster_"))}


def build_summary(directory):
    directory = Path(directory).expanduser().resolve()
    matrix = json.loads((directory/"comparison_matrix.json").read_text())
    records = []
    for case in matrix:
        path = directory/"pairs"/case["name"]/"comparison.json"
        record = {"case": case, "available": path.exists(), "explanation": route_explanation(case)}
        if path.exists():
            pair = json.loads(path.read_text())
            primary = pair["regions"][pair["primary_region"]]
            record.update({"pair": pair, "primary": primary, "primary_region": pair["primary_region"]})
            caster_path = Path(case["swivel_directory"])/"caster_stress.json"
            if caster_path.exists():
                caster = json.loads(caster_path.read_text())
                diagnostic = caster.get("source_time_alignment_diagnostic", {})
                source = diagnostic["casters"] if diagnostic.get("available") else caster["casters"]
                record["ordinary_alignment"] = {"source_timing_aligned": bool(diagnostic.get("available")),
                    "regions": {region: {side: {
                        "local_axis15_longest_s": source[side]["regions"][region]["local_wheel"]["rolling_axis"]["exceeds"]["15"]["longest_episode_s"],
                        "local_axis_p95_deg": source[side]["regions"][region]["local_wheel"]["rolling_axis"]["p95_abs"],
                        "near_backward150_longest_s": source[side]["regions"][region]["local_wheel"]["directed"]["exceeds"].get("150", {}).get("longest_episode_s"),
                        "path_axis_p95_deg": source[side]["regions"][region]["commanded_path"]["rolling_axis"]["p95_abs"]}
                        for side in ("left", "right")} for region in (pair["primary_region"], "first_leg")}}
            for model in ("swivel", "ball"):
                trajectory_path = Path(case[model+"_directory"])/"trajectory.json"
                data = json.loads(trajectory_path.read_text())
                record[model+"_startup"] = data.get("startup_alignment")
            record["successful_pair"] = all(status["completed"] and status["trajectory_passed"] for status in pair["run_status"].values())
        records.append(record)
    lookup = {record["case"]["name"]: record for record in records}
    repeatability = {"interpretation": "Nominal ball repeats share route, speed and ball parameters; variation includes asynchronous sampling and numerical/contact evolution. These ranges are descriptive, not hardware uncertainty bounds or statistical confidence intervals.", "groups": {}}
    for group, names in REPEAT_GROUPS.items():
        eligible = [lookup[name] for name in names if name in lookup and lookup[name]["available"] and lookup[name]["pair"]["run_status"]["ball"]["completed"] and lookup[name]["pair"]["run_status"]["ball"]["trajectory_passed"]]
        values = {}
        for metric in ("rms", "max_abs"):
            values["tracking_"+metric+"_m"] = distribution([record["pair"]["regions"]["motion"]["ball"]["tracking_error_m"][metric] for record in eligible])
        values["limited_fraction"] = distribution([record["pair"]["regions"]["motion"]["ball"]["either_limited_fraction"] for record in eligible])
        repeatability["groups"][group] = {"expected_names": names, "completed_names": [r["case"]["name"] for r in eligible], **values}
    available = [record for record in records if record["available"]]
    masses = {model: model_masses(available[0]["case"][model+"_directory"]) if available else None for model in ("swivel", "ball")}
    ride_pairs = [record for record in available if all(record["primary"][model]["body_height_range_m"] is not None for model in ("swivel", "ball"))]
    ride = {"available_pairs": len(ride_pairs), "ball_height_range_larger_count": sum(record["primary"]["ball"]["body_height_range_m"] > record["primary"]["swivel"]["body_height_range_m"] for record in ride_pairs)}
    key_findings_path = directory/"KEY_FINDINGS.md"
    return {"schema_version": 1, "directory": str(directory), "expected_count": len(matrix),
            "key_findings_markdown": key_findings_path.read_text().strip() if key_findings_path.exists() else "",
            "speed_plot_available": (directory/"speed_tradeoffs.png").exists(),
            "analyzed_count": len(available), "successful_pair_count": sum(r["successful_pair"] for r in available),
            "complete": len(available) == len(matrix), "records": records,
            "repeatability": repeatability, "nominal_model_masses": masses, "ride_comparison": ride}


def compact_rows(summary):
    rows = []
    for record in summary["records"]:
        case = record["case"]
        row = {"name": case["name"], "category": case["category"], "available": record["available"],
               "primary_region": case["tracking_comparison_region"], "speed_m_s": case["speed_m_s"],
               "successful_pair": record.get("successful_pair"),
               "ball_role": case.get("ball_role"), "explanation": record["explanation"]}
        for model in ("swivel", "ball"):
            primary = record.get("primary", {}).get(model, {})
            tracking = primary.get("tracking_error_m", {})
            row[model+"_rms_m"] = tracking.get("rms")
            row[model+"_peak_m"] = tracking.get("max_abs")
            row[model+"_limited_fraction"] = primary.get("either_limited_fraction")
            row[model+"_along_track_rms_m"] = primary.get("along_track_error_m", {}).get("rms")
            row[model+"_cross_track_rms_m"] = primary.get("cross_track_error_m", {}).get("rms")
        row["rms_change_percent"] = record.get("primary", {}).get("tracking_rms_change_percent")
        rows.append(row)
    return rows


def pair_sentence(record):
    if not record["available"]:
        return "**Pending:** no completed comparison artifact is available yet."
    p = record["primary"]
    s, b = p["swivel"], p["ball"]
    st, bt = s["tracking_error_m"], b["tracking_error_m"]
    status = "Both runs passed their configured execution checks." if record["successful_pair"] else "**At least one run did not complete/pass its configured checks; this is not a successful equivalent execution.**"
    result = (f"Using **{record['primary_region']}**, ordinary → ball tracking RMS is **{number(st['rms'],1000)} → {number(bt['rms'],1000)} mm** "
        f"({number(p['tracking_rms_change_percent'])}%, absolute change {number(p['tracking_rms_ball_minus_swivel_m'],1000,3)} mm); sampled peak is **{number(st['max_abs'],1000)} → {number(bt['max_abs'],1000)} mm**. "
        f"Drive limiting is {number(s['either_limited_fraction'],100,1)}% → {number(b['either_limited_fraction'],100,1)}%. ")
    delta, peak_delta = p["tracking_rms_ball_minus_swivel_m"], p["tracking_peak_ball_minus_swivel_m"]
    if delta is not None and peak_delta is not None:
        if delta*peak_delta < 0:
            result += "RMS and worst error change in opposite directions: this is a tracking tradeoff. "
        elif delta < 0 and peak_delta < 0:
            result += "Both tracking measures are numerically lower for the ball in this pair; practical significance is not established by that fact alone. "
        elif delta > 0 and peak_delta > 0:
            result += "Both tracking measures are numerically higher for the ball in this pair. "
    along_s, along_b = s.get("along_track_error_m", {}).get("rms"), b.get("along_track_error_m", {}).get("rms")
    cross_s, cross_b = s.get("cross_track_error_m", {}).get("rms"), b.get("cross_track_error_m", {}).get("rms")
    if along_s is not None and along_b is not None:
        result += f"Along/cross-route RMS is {number(along_s,1000)}/{number(cross_s,1000)} → {number(along_b,1000)}/{number(cross_b,1000)} mm. "
    result += f"Body-height range is {number(s['body_height_range_m'],1000,3)} → {number(b['body_height_range_m'],1000,3)} mm; maximum absolute pitch is {number(s['body_pitch_deg']['max_abs'],digits=3)}° → {number(b['body_pitch_deg']['max_abs'],digits=3)}°. "
    return result+status


def alignment_sentence(record):
    alignment = record.get("ordinary_alignment")
    if alignment is None:
        return ""
    sides = alignment["regions"][record["primary_region"]]
    left, right = sides["left"], sides["right"]
    timing = "joint-source-time aligned" if alignment["source_timing_aligned"] else "raw paired samples; joint source timestamps unavailable"
    result = (f"Ordinary-caster alignment in **{record['primary_region']}** ({timing}): longest local rolling-axis error >15° is "
        f"**{number(left['local_axis15_longest_s'])} / {number(right['local_axis15_longest_s'])} s left/right**; "
        f"95th-percentile local-axis error is **{number(left['local_axis_p95_deg'])}° / {number(right['local_axis_p95_deg'])}°**; "
        f"95th-percentile angle to the commanded path axis is **{number(left['path_axis_p95_deg'])}° / {number(right['path_axis_p95_deg'])}°**. "
        "The local metric identifies forward and reverse rolling as equivalent; the path-axis metric can be large during geometrically correct turns. No corresponding ball-carrier heading metric is assigned.")
    backward = [side.get("near_backward150_longest_s") for side in (left, right)]
    if any(value is not None and value > 0 for value in backward):
        result += f" Directed near-backward orientation (>150° relative to its own wheel-center travel) persists up to {number(backward[0])}/{number(backward[1])} s left/right; this is distinct from sideways rolling."
    return result


def measured_case_note(record):
    """Specific interpretation backed by fields in the recorded pair."""
    if not record["available"] or record["case"]["name"] != "circle_large_360":
        return ""
    primary = record["primary"]
    swivel, ball = primary["swivel"], primary["ball"]
    result = (f"For this larger circle, cross-route RMS changes from {number(swivel['cross_track_error_m']['rms'],1000,3)} "
        f"to {number(ball['cross_track_error_m']['rms'],1000,3)} mm, while along-route RMS changes from "
        f"{number(swivel['along_track_error_m']['rms'],1000,3)} to {number(ball['along_track_error_m']['rms'],1000,3)} mm. ")
    pair = record["pair"]
    ages = [pair[model+"_feedback_timing"]["odometry_age_s"]["mean_abs"] for model in ("swivel", "ball")]
    result += f"Mean logged odometry age over the complete records is {number(ages[0],1000,3)} → {number(ages[1],1000,3)} ms. "
    alignment = record.get("ordinary_alignment", {}).get("regions", {}).get(record["primary_region"], {})
    if alignment and all(side["local_axis15_longest_s"] == 0 for side in alignment.values()):
        result += "The ordinary casters have no local rolling-axis episodes above 15° here; that threshold result does not mean zero finite angle error or zero contact slip. "
    values = []
    for key, label, scale, unit in (("yaw_rate_residual_rad_s", "yaw-rate", 1., "rad/s"),
                                  ("axle_lateral_velocity_residual_m_s", "axle lateral", 1000., "mm/s"),
                                  ("axle_forward_velocity_residual_m_s", "axle forward", 1000., "mm/s")):
        first = swivel["drive_residual"][key]["rms"]
        second = ball["drive_residual"][key]["rms"]
        values.append(f"{label} {number(first,scale,4)} → {number(second,scale,4)} {unit}")
    result += "Measured planar drive-residual RMS changes are " + "; ".join(values) + ". These are descriptive diagnostics, not a measurement or attribution of caster contact forces."
    return result


def startup_sentence(record):
    if not record["available"]:
        return ""
    parts = []
    for model in ("swivel", "ball"):
        startup = record.get(model+"_startup") or {}
        if not startup.get("enabled"):
            parts.append(f"{model}: no preparation gate")
            continue
        xy = startup.get("executed_start_position_m", [None, None])
        turn = startup.get("actual_chassis_yaw_change_rad")
        gate = startup.get("caster_orientation_gate_applied", model == "swivel")
        parts.append(f"{model}: preparation {number(startup.get('duration_s'))} s, actual turn {number(math.degrees(turn) if turn is not None else None)}°, "
            f"release ({number(xy[0],digits=3)}, {number(xy[1],digits=3)}) m, gate passed={startup.get('passed')}, swivel-facing gate={'yes' if gate else 'no'}")
    text = "Preparation: "+"; ".join(parts)+"."
    if record.get("ordinary_alignment"):
        first = record["ordinary_alignment"]["regions"]["first_leg"]
        text += f" Ordinary-caster first-leg local-axis >15° episodes last at most {number(first['left']['local_axis15_longest_s'])}/{number(first['right']['local_axis15_longest_s'])} s left/right."
    return text


def render_markdown(summary):
    done, total = summary["analyzed_count"], summary["expected_count"]
    state = "All planned pairs analyzed" if summary["complete"] else "Partial study — captures/analysis still in progress"
    lines = ["# Ordinary swivel wheels versus passive split-ball casters", "",
        f"**{state}: {done}/{total} pairs available; {summary['successful_pair_count']} pairs passed both configured execution checks.**",
        "", "This study compares recorded model behavior. A smaller tracking error in one trial is an observed benefit for that route and setup, not proof of universal superiority or calibrated hardware performance. Ball casters have no ordinary swivel-wheel forward-facing angle, so their carrier angle is never used as a substitute for the ordinary caster alignment metric.",
        "", "Open the [interactive comparison gallery](index.html), [per-trajectory view](PER_TRAJECTORY.md), [compact results](comparison_results.csv), and [repeatability data](repeatability.json). Each case below links its measured comparison plot.",
        ]
    if summary.get("key_findings_markdown"):
        lines += ["", summary["key_findings_markdown"]]
    lines += ["", "## What is held constant, and what differs", "",
        "The primary route and speed cases retain the same original pose, requested waypoints, reference speed, controller gains and wheel-rate/acceleration limits, floor friction, and physics settings. The five assisted heading starts retain the original pose and destinations but use model-appropriate release conditions: both are configured to recover chassis heading and perform forward lead-in; only the ordinary caster requires a swivel-facing gate. Their executed first sources can differ, so **later legs** are the primary comparison. Approach states inherited from preparation can still differ.",
        "", "The six ordinary-caster physical variants have no one-to-one ball intervention. Their ball recordings use nominal ball parameters and are labeled repeated baselines. They are useful checks of run variation, not matched friction/inertia experiments.",
        "", "The ball model is a CAD-derived split-shell mechanism with a horizontal carrier, two independently rotating shells and two polar rollers per caster. Its faceted shell, 20 mm seam and roller protrusion differ from an ideal sphere. The ordinary caster uses a smooth rigid cylinder and a passive yaw joint with trail. Contact geometry, estimated masses/inertias and bearing damping differ; this experiment compares those complete estimated models rather than changing shape at equal mass and damping."]
    lines += ["", "Nominal viscous joint damping is **0.015 N m s/rad at the ordinary swivel and 0.006 at its wheel**, versus **0.0002 at each ball carrier/shell joint and 0.00005 at each polar roller**. These values are engineering assumptions, not measured loaded bearing losses. The floor friction coefficient is **0.8**; contact geometry and material assumptions also differ. See the [ball physical assumptions snapshot](reproducibility/hamr_ball_caster/config/physical_properties.yaml) and [ordinary-caster model assumptions](../caster_stress_20261004/PHYSICS_ASSUMPTIONS.md)."]
    masses = summary["nominal_model_masses"]
    if all(masses.values()):
        lines += ["", f"The nominal saved models weigh **{number(masses['swivel']['whole_model_kg'],digits=3)} kg ordinary / {number(masses['ball']['whole_model_kg'],digits=3)} kg ball**. "
            f"Each prefixed caster group weighs {number(masses['swivel']['left_caster_group_kg'],digits=3)} / {number(masses['ball']['left_caster_group_kg'],digits=3)} kg. The ordinary group includes its adapter; the purchased-caster assembly estimate itself is 2.640 kg."]
    lines += ["", "## Compact results", "",
        "Values are duration-weighted RMS and sampled peak tracking error over the selected primary region. Percent change is `(ball−ordinary)/ordinary`; negative means smaller RMS. Drive limiting is the fraction of that region with wheel-speed or acceleration clipping. A pass certifies the configured execution checks, not perfect tracking.", "",
        "Here, nominal or normal speed means the inherited **0.25 m/s simulation setting**, not a measured normal operating speed for the hardware. Fast stress cases use 0.40 m/s; slower controls use 0.20 or 0.08 m/s.", "",
        "| Case | Speed (m/s) | Region | Ordinary RMS / peak (mm) | Ball RMS / peak (mm) | RMS change | Limited ordinary / ball |",
        "|---|---:|---|---:|---:|---:|---:|"]
    for record in summary["records"]:
        case = record["case"]
        if not record["available"]:
            lines.append(f"| {case['name']} | {case['speed_m_s']:.2f} | {case['tracking_comparison_region']} | Pending | Pending | — | — |")
            continue
        p = record["primary"]
        s, b = p["swivel"], p["ball"]
        lines.append(f"| [{case['name']}](pairs/{case['name']}/comparison.png) | {case['speed_m_s']:.2f} | {record['primary_region']} | "
            f"{number(s['tracking_error_m']['rms'],1000)} / {number(s['tracking_error_m']['max_abs'],1000)} | "
            f"{number(b['tracking_error_m']['rms'],1000)} / {number(b['tracking_error_m']['max_abs'],1000)} | "
            f"{number(p['tracking_rms_change_percent'])}% | {number(s['either_limited_fraction'],100,1)}% / {number(b['either_limited_fraction'],100,1)}% |")
    ride = summary["ride_comparison"]
    if ride["available_pairs"]:
        lines += ["", f"In the **{ride['available_pairs']} currently analyzed pairs**, the ball model has a larger measured root-height range in **{ride['ball_height_range_larger_count']}**. Per-case height and pitch values below distinguish tracking accuracy from vertical smoothness. Faceted/seamed ball contacts and different model properties limit any direct hardware ride-quality inference."]
    lines += ["", "## Why speed and a perfect floor do not imply perfect tracking", ""]
    if summary.get("speed_plot_available"):
        lines += ["![Measured tracking RMS, sampled peak and drive limiting versus requested speed](speed_tradeoffs.png)", "",
            "[Download the speed comparison as SVG](speed_tradeoffs.svg). Values come from the completed speed controls, with identical geometry within each shape.", ""]
    lines += [
        "A flat, deterministic environment removes some disturbances; it does not remove joint inertia, friction, passive reorientation, the offset drive geometry, finite controller updates or explicit actuator command limits. A piecewise-linear position reference has instantaneous velocity changes at its corners. A wheel with finite acceleration cannot reproduce those changes exactly without anticipation or a smoothed time profile.",
        "", "For an ideal straight reversal from speed +v to −v, rear-wheel radius r=0.1075 m and the configured acceleration cap α=15 rad/s² give a maximum translational acceleration `a=rα=1.6125 m/s²`. Starting at speed with no anticipation, even ideal contact requires `t_reverse ≥ 2v/a`. Against an instantaneous reversing reference, accumulated lag by reaching −v is `e=2v²/a`. This assumes equal-wheel straight motion at maximum deceleration; it is not a fitted prediction for turning shuttles or a complete controller model.", "",
        "| Speed (m/s) | Minimum straight reversal time (s) | Ideal ramp lag (mm) |", "|---:|---:|---:|"]
    for speed in (.08, .20, .25, .40):
        lines.append(f"| {speed:.2f} | {2*speed/1.6125:.3f} | {2*speed*speed/1.6125*1000:.2f} |")
    lines += ["", "For a trailing swivel, write wheel position as `w=p−l e`. Requiring zero lateral wheel velocity gives `ψ̇_c=(e_perp·V_p)/l`. For straight pivot travel, caster error obeys `α̇=−(v_p/l)sinα`; near alignment the ideal time scale is `l/v_p`. With 45 mm trail it is 0.56 s at 0.08 m/s pivot speed and 0.18 s at 0.25 m/s. This excludes inertia/friction dynamics and is not fitted to recorded episodes. Exact backward orientation is an unstable equilibrium: a wheel can initially roll backward without sideways scrub. A longer directed-heading mismatch is therefore different from a long sideways rolling-axis error.",
        "", "The tracked point is `a=0.27114 m` ahead of the rear axle. Ideal planar no-slip kinematics give `θ̇=(v/a)sin(ψ−θ)`. On a counterclockwise circle, `δ=ψ−θ` obeys `δ̇=v/R−(v/a)sinδ`, and a constant offset needs `sinδ=a/R`. The tight 0.20 m circle has `R<a`, so no constant-offset solution exists. This concerns steady body-to-tangent heading offset, not the possibility of ideal tracked-point position following. Ball casters retain this same rear-drive geometry. Even on a larger circle, a caster's correct local velocity generally differs from the tracked-point tangent.",
        "", "The legacy simple controller also compares current reference samples with latest odometry. Unlike its optional aligned-feedback mode, the reference timestamp is not corrected to the odometry source time in these legacy runs. A 10 ms age at 0.25 m/s corresponds to 2.5 mm along the path; this can contribute to millimeter-scale reported residuals. Both models use the same policy, but tiny pair differences are not automatically physical caster effects.",
        "", "## Every trajectory and control case", ""]
    for record in summary["records"]:
        case = record["case"]
        lines += [f"### {case['name']}", "", record["explanation"], "", pair_sentence(record)]
        alignment = alignment_sentence(record)
        if alignment:
            lines += ["", alignment]
        note = measured_case_note(record)
        if note:
            lines += ["", note]
        if case["category"] == "initial_heading":
            startup = startup_sentence(record)
            if startup:
                lines += ["", startup]
        lines += [""]
    lines += ["## Repeatability checks", "", summary["repeatability"]["interpretation"], "",
        "| Nominal ball group | Completed repeats | RMS range / mean (mm) | Peak range (mm) |", "|---|---:|---:|---:|"]
    for name, group in summary["repeatability"]["groups"].items():
        rms, peak = group["tracking_rms_m"], group["tracking_max_abs_m"]
        lines.append(f"| {name} | {len(group['completed_names'])}/{len(group['expected_names'])} | {number(rms['min'],1000)}–{number(rms['max'],1000)} / {number(rms['mean'],1000)} | {number(peak['min'],1000)}–{number(peak['max'],1000)} |")
    lines += ["", "Do not rank small parameter or architecture differences more strongly than these observed ranges justify. Repeated nominal ball runs do not turn the ordinary-only parameter cases into equivalent ball interventions, and they do not supply independent hardware trials.",
        "", "## Measurement and scope", "",
        "Common metrics use deduplicated odometry source times, midpoint-duration weights, and the same motion/first-leg/later-leg definitions. Along-track error projects position error on the commanded tangent; cross-track error projects perpendicular to it. These describe tracking, not caster orientation. Startup and final hold are separate. Large timestamp gaps are excluded from derivative support and duration weighting.",
        "", "Ordinary-caster angle statistics include only motion with the relevant local speed at least 0.03 m/s; direction at rest is undefined. Source-timed joint interpolation is used when available, with the original untimestamped samples retained for the first three ordinary recordings. These are finite-sampling geometric diagnostics, not exact measured tire contact slip. A brief threshold episode is different from a jammed swivel, and a 180-degree directed heading can represent backward rolling with zero undirected axis error.",
        "", "Ball joint coordinates are accumulated Gazebo coordinates; fast polar-roller increments are preserved rather than modulo-unwrapped. Joint activity confirms passive motion but does not measure load share or bearing work. No ball swivel-heading score or ideal-sphere contact-slip score is assigned. Rear-drive velocity residuals compare measured poses with ideal planar wheel kinematics; they include timing, derivatives, suspension and contact effects and are not measured contact forces or isolated caster slip.",
        "", "Both models use ideal rear-wheel velocity servos with software rate/acceleration limits. Motor torque/current, stall and thermal feasibility are not established. Tire compliance, floor roughness, bearing backlash and measured material properties are absent or estimated. Faceted ball contacts can introduce vertical/pitch variation even when body tracking improves. The prior ordinary-caster timestep check showed contact-derived details more sensitive than gross tracking; this comparison is not a convergence proof for either architecture.",
        "", "Per-case input models, configurations, world files, source-timed joint observations and separate closeup/overview videos are retained in the linked run directories. Original ordinary-caster recordings are preserved. The deterministic exporter can regenerate this document as additional pairs complete.", ""]
    return "\n".join(lines)


def export_summary(directory):
    summary = build_summary(directory)
    root = Path(summary["directory"])
    rows = compact_rows(summary)
    (root/"FINDINGS.md").write_text(render_markdown(summary))
    (root/"repeatability.json").write_text(json.dumps(summary["repeatability"], indent=2, allow_nan=False)+"\n")
    (root/"comparison_results.json").write_text(json.dumps(rows, indent=2, allow_nan=False)+"\n")
    if rows:
        with (root/"comparison_results.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite_directory", type=Path)
    args = parser.parse_args()
    summary = export_summary(args.suite_directory)
    print(json.dumps({key: summary[key] for key in ("complete", "analyzed_count", "expected_count", "successful_pair_count")}, indent=2))


if __name__ == "__main__":
    main()
