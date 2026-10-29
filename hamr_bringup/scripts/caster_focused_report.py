"""The four plots and five-section HTML used by visualize_caster_bag."""
import html
from pathlib import Path

import numpy as np


BASE_TOPICS = ("/HAMR_base/pose", "/HAMR_base/odom", "/vicon/odom", "/mocap/odom")
EKF_TOPICS = ("/local_HAMR/odom", "/odometry/filtered")
SECTIONS = (
    ("simulation", "Simulation video"),
    ("encoder_angles", "Caster encoder angles"),
    ("robot_trajectory", "Robot trajectory"),
    ("robot_casters", "Robot trajectory with left and right casters"),
    ("robot_ekf", "Robot trajectory with aligned EKF localization"),
)
MAX_POSE_GAP_S = .25


def pose_series(recording, topics):
    topic = next((name for name in topics if recording.poses.get(name)), None)
    if topic is None:
        raise ValueError("Missing pose topic: " + " or ".join(topics))
    rows = recording.poses[topic]
    frames = {row["frame_id"] for row in rows}
    if len(frames) != 1 or not next(iter(frames)):
        raise ValueError(f"{topic} must have one nonempty coordinate frame")
    t = np.array([(row["stamp_ns"] - recording.start_ns) / 1e9 for row in rows])
    xy = np.array([[row["x"], row["y"]] for row in rows], dtype=float)
    q = np.array([[row["q" + axis] for axis in "xyzw"] for row in rows], dtype=float)
    norm = np.linalg.norm(q, axis=1)
    with np.errstate(invalid="ignore", divide="ignore"):
        x, y, z, w = (q / norm[:, None]).T
        yaw = np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    yaw[norm < 1e-9] = np.nan
    # Keep the last sample at duplicate receipt times; never interpolate a zero interval.
    unique = np.r_[np.diff(t) > 0, True]
    return dict(topic=topic, frame=next(iter(frames)), t=t[unique],
                xy=xy[unique], yaw=yaw[unique])


def interpolate_pose(series, times):
    """Interpolate locally; gaps and invalid samples stay unavailable."""
    t = series["t"]
    if len(t) < 2:
        raise ValueError(f"Too few poses on {series['topic']}")
    right = np.clip(np.searchsorted(t, times, side="right"), 1, len(t)-1)
    left = right - 1
    dt = t[right] - t[left]
    fraction = (times - t[left]) / dt
    xy = series["xy"][left] + fraction[:, None] * (series["xy"][right] - series["xy"][left])
    delta = series["yaw"][right] - series["yaw"][left]
    yaw = series["yaw"][left] + fraction * np.arctan2(np.sin(delta), np.cos(delta))
    valid = ((times >= t[0]) & (times <= t[-1]) & (dt <= MAX_POSE_GAP_S)
             & np.isfinite(xy).all(axis=1) & np.isfinite(yaw))
    return xy, yaw, valid


def align_ekf(base, ekf, mode="initial-pose"):
    """One constant SE(2) transform, without scale fitting or clock shifting."""
    start = max(base["t"][0], ekf["t"][0])
    end = min(base["t"][-1], ekf["t"][-1])
    if end - start < .02:
        raise ValueError("Robot and EKF poses do not have a usable shared time interval")
    times = np.arange(start, end, .02)
    b, by, bv = interpolate_pose(base, times)
    e, ey, ev = interpolate_pose(ekf, times)
    valid = bv & ev
    if np.count_nonzero(valid) < 2:
        raise ValueError("Too few valid synchronized robot/EKF poses")
    b, e, by, ey, paired_t = b[valid], e[valid], by[valid], ey[valid], times[valid]
    if mode == "initial-pose":
        angle = by[0] - ey[0]
        c, s = np.cos(angle), np.sin(angle)
        rot = np.array([[c, -s], [s, c]])
        translation = b[0] - e[0] @ rot.T
    elif mode == "rigid-fit":
        a0, b0 = e - e.mean(axis=0), b - b.mean(axis=0)
        if min(np.linalg.norm(a0), np.linalg.norm(b0)) < 1e-4:
            raise ValueError("Rigid-fit alignment needs robot and EKF position changes")
        u, _, vt = np.linalg.svd(a0.T @ b0)
        rot = (u @ np.diag([1., np.linalg.det(u @ vt)]) @ vt).T
        angle = np.arctan2(rot[1, 0], rot[0, 0])
        translation = b.mean(axis=0) - e.mean(axis=0) @ rot.T
    else:
        raise ValueError(f"Unknown EKF alignment mode: {mode}")
    error = np.linalg.norm(e @ rot.T + translation - b, axis=1)
    aligned = dict(ekf, xy=ekf["xy"] @ rot.T + translation, frame=base["frame"])
    metrics = dict(
        method=mode, rotation_deg=float(np.degrees(np.arctan2(np.sin(angle), np.cos(angle)))),
        translation_m=translation.tolist(), samples=len(error),
        comparison_start_s=float(paired_t[0]), comparison_end_s=float(paired_t[-1]),
        rms_difference_m=float(np.sqrt(np.mean(error**2))),
        final_difference_m=float(error[-1]), source_topic=ekf["topic"], target_topic=base["topic"],
        source_frame=ekf["frame"], target_frame=base["frame"],
        time_basis="Bag receipt time; 50 Hz interpolation; gaps over 0.25 s excluded; no clock shift",
        limitation="Marker-to-body extrinsics are not calibrated here; residual is a pose difference, not a calibrated sensor accuracy.",
    )
    return aligned, metrics


def plot_path(ax, series, label, color):
    xy = series["xy"]
    breaks = np.r_[False, np.diff(series["t"]) > MAX_POSE_GAP_S]
    points = np.insert(xy, np.flatnonzero(breaks), np.nan, axis=0)
    ax.plot(points[:, 0], points[:, 1], label=label, color=color)
    valid = xy[np.isfinite(xy).all(axis=1)]
    if len(valid):
        ax.scatter(*valid[0], marker="o", s=30, color=color, zorder=5)
        ax.scatter(*valid[-1], marker="x", s=40, color=color, zorder=5)


def generate_plots(recording, output, alignment="initial-pose"):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import pyplot as plt
    plt.rcParams.update({"font.size": 11, "lines.linewidth": 1.4})
    stages = {}

    def save(fig, key, filename, detail, **metadata):
        fig.savefig(output / filename, dpi=160)
        plt.close(fig)
        stages[key] = dict(status="complete", file=filename, detail=detail, **metadata)

    topics = recording.encoder_topics()
    if topics:
        fig, axes = plt.subplots(3, 2, figsize=(12, 8), sharex=True, layout="constrained")
        missing = []
        for col, wheel in enumerate(("front_left", "front_right")):
            for row, role in enumerate(("H1", "H2", "CENTER")):
                ax = axes[row, col]
                topic = topics.get((wheel, role))
                rows = recording.numeric.get(topic, [])
                if rows:
                    t = np.array([(r["stamp_ns"]-recording.start_ns)/1e9 for r in rows])
                    angle = np.array([r.get("angle_rad", np.nan) for r in rows], dtype=float)
                    broken = np.r_[False, (np.diff(t) <= 0) | (np.diff(t) > .1)]
                    broken |= np.array([False] + [a.get("segment") != b.get("segment")
                                                 for a, b in zip(rows, rows[1:])])
                    if role == "CENTER":
                        broken |= np.r_[False, np.abs(np.diff(angle)) > np.pi]
                    ax.plot(np.insert(t, np.flatnonzero(broken), np.nan),
                            np.insert(angle, np.flatnonzero(broken), np.nan))
                else:
                    missing.append(f"{wheel}/{role}")
                    ax.text(.5, .5, "Not recorded", ha="center", transform=ax.transAxes)
                ax.set(title=f"{'Left' if col == 0 else 'Right'} caster — {role}", ylabel="Angle (rad)")
                ax.grid(alpha=.2)
                if row == 2:
                    ax.set_xlabel("Bag time (s)")
        fig.suptitle("Caster encoder angles")
        detail = "Recorded source angles. CENTER wraps at 2π; source signs and phases are retained."
        save(fig, "encoder_angles", "caster_encoder_angles.png", detail, missing_channels=missing)
        if missing:
            stages["encoder_angles"]["status"] = "partial"
            stages["encoder_angles"]["detail"] += " Missing: " + ", ".join(missing) + "."
    else:
        stages["encoder_angles"] = dict(status="unavailable", detail="No caster encoder angles were recorded.")

    try:
        base = pose_series(recording, BASE_TOPICS)
    except ValueError as error:
        for key in ("robot_trajectory", "robot_casters", "robot_ekf"):
            stages[key] = dict(status="unavailable", detail=str(error))
        return stages

    for key, filename, title in (
        ("robot_trajectory", "robot_trajectory.png", "Robot trajectory"),
        ("robot_casters", "robot_casters_trajectory.png", "Robot and caster trajectories"),
        ("robot_ekf", "robot_ekf_aligned.png", "Robot trajectory and aligned EKF"),
    ):
        fig, ax = plt.subplots(figsize=(8, 7), layout="constrained")
        plot_path(ax, base, "Robot (Vicon)", "#e78922")
        detail = f"{base['topic']} in {base['frame']}. Circle = start; cross = finish."
        extra = {}
        try:
            if key == "robot_casters":
                for subject, label, color in (("Left_Caster", "Left caster", "#249440"),
                                               ("Right_Caster", "Right caster", "#8753bc")):
                    caster = pose_series(recording, (f"/{subject}/pose", f"/{subject}/odom"))
                    if caster["frame"] != base["frame"]:
                        raise ValueError(f"{caster['topic']} is in {caster['frame']}; robot is in {base['frame']}")
                    plot_path(ax, caster, label, color)
                detail += " Casters show recorded fork-marker positions."
            elif key == "robot_ekf":
                ekf, metrics = align_ekf(base, pose_series(recording, EKF_TOPICS), alignment)
                plot_path(ax, ekf, "EKF (aligned)", "#147cc1")
                method = "Starting-pose alignment" if alignment == "initial-pose" else "Whole-run rigid fit"
                detail = (f"{method}: rotation {metrics['rotation_deg']:.2f}°, "
                          f"RMS difference {metrics['rms_difference_m']:.3f} m, "
                          f"final difference {metrics['final_difference_m']:.3f} m. "
                          f"Compared over {metrics['comparison_start_s']:.2f}–{metrics['comparison_end_s']:.2f} s. "
                          "No scale fitting or time shift; marker-to-body calibration is not applied.")
                if alignment == "rigid-fit":
                    detail += " Fitted on this run; this is not independent localization accuracy."
                extra["alignment"] = metrics
                ax.set_title(f"{title}\n{method} · {metrics['rotation_deg']:.2f}° rotation")
        except ValueError as error:
            plt.close(fig)
            stages[key] = dict(status="unavailable", detail=str(error))
            continue
        if key != "robot_ekf":
            ax.set_title(title)
        ax.set(xlabel=f"x (m) — {base['frame']}", ylabel=f"y (m) — {base['frame']}")
        ax.set_aspect("equal", adjustable="box")
        ax.grid(alpha=.2)
        ax.legend()
        save(fig, key, filename, detail, **extra)
    return stages


def write_index(output, manifest):
    sections = []
    for key, title in SECTIONS:
        stage = manifest["stages"][key]
        filename = stage.get("file")
        media = ""
        if filename:
            filename = html.escape(filename, quote=True)
            if Path(filename).suffix == ".mp4":
                media = f'<video controls preload="metadata" src="{filename}"></video>'
            else:
                media = f'<a href="{filename}"><img loading="lazy" src="{filename}" alt="{html.escape(title)}"></a>'
        sections.append(f'<section><h2>{html.escape(title)}</h2>{media}<p>{html.escape(stage["detail"])}</p></section>')
    (output / "index.html").write_text(f'''<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{html.escape(manifest['recording_name'])} — HAMR analysis</title>
<style>body{{font:16px system-ui,sans-serif;max-width:1200px;margin:32px auto;padding:0 20px;background:#f4f7fa;color:#193047}}
h1{{font-size:28px}}h2{{font-size:21px}}section{{background:white;padding:20px;margin:24px 0;border-radius:8px}}
img,video{{display:block;max-width:100%;height:auto;margin:auto}}p{{line-height:1.5}}code{{overflow-wrap:anywhere}}a{{color:#1764ab}}</style>
</head><body><h1>{html.escape(manifest['recording_name'])}</h1>
<p>Source: <code>{html.escape(manifest['source'])}</code></p>
{''.join(sections)}
</body></html>''')
