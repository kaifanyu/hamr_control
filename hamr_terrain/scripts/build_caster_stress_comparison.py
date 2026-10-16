#!/usr/bin/env python3
"""Build an offline, descriptive comparison from saved caster campaign artifacts.

No simulator or analyzer is invoked. The matrix defines the canonical population;
missing measurements and failed runs stay visible. Figures use analyzer-provided
common-horizon contrasts. Videos align to each run's trajectory clock.
"""
import argparse
import csv
from datetime import datetime, timezone
import hashlib
import html
import json
import math
from pathlib import Path
import shutil
from urllib.parse import quote

import numpy as np

MODELS = ("swivel", "ball")
COLORS = {"swivel": "#c66a25", "ball": "#246a9a"}
METRICS = (
    ("xy_rmse_m", "XY tracking RMS", "mm", 1000.),
    ("roll_rms_deg", "Body roll RMS", "deg", 1.),
    ("pitch_rms_deg", "Body pitch RMS", "deg", 1.),
    ("low_progress_fraction", "Low-progress time", "percentage points", 100.),
)
TABLE_METRICS = METRICS + (
    ("body_yaw_maximum_contiguous_span_deg", "Measured chassis yaw span", "deg", 1.),
    ("body_yaw_absolute_travel_deg", "Measured chassis yaw travel", "deg", 1.),
    ("any_limited_fraction", "Drive command-limited time", "%", 100.),
)
PAIRED_VIDEO_CLIPS = (
    ('high_pad_zigzag_overviews_side_by_side',
     'High staggered pads · zigzag · synchronized overviews', 'overview'),
    ('high_pad_reversal_closeups_side_by_side',
     'High staggered pads · reversal · synchronized caster close-ups', 'close-up'),
    ('washboard_l1_reversal_overviews_side_by_side',
     'Washboard level 1 · reversal · synchronized overviews', 'overview'),
    ('scan_edge25_tight_eight_overviews_side_by_side',
     '25 cm scan-derived terrain · tight figure eight · synchronized overviews', 'overview'),
)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def load(path, errors):
    if not path.is_file():
        return {}
    try:
        return json.loads(path.read_text())
    except (ValueError, OSError) as error:
        errors.append(f"{path}: {error}")
        return {}


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n")
    temporary.replace(path)


def relative(root, path):
    return quote(path.relative_to(root).as_posix(), safe="/")


def canonical(root, value):
    path = (root/str(value)).resolve()
    if not path.is_relative_to(root):
        raise ValueError(f"Matrix directory must be inside the campaign root: {value}")
    return path


def matches_directory(saved, expected, relative_directory):
    """Accept a relocated campaign only when its complete matrix suffix matches."""
    if not saved:
        return False
    candidate = Path(saved)
    return candidate == expected or tuple(candidate.parts[-len(relative_directory.parts):]) == relative_directory.parts


def run_state(summary, exists):
    if not exists:
        return "pending"
    if summary.get("physical_failure"):
        return "physical failure"
    if summary.get("infrastructure_failure") or summary.get("error"):
        return "infrastructure failure"
    if summary.get("tracking_passed") is False:
        return "outside tracking limits"
    if summary.get("tracking_passed") is True:
        return "tracking passed"
    return "partial / unknown"


def video_sources(root, directory, summary, errors):
    result = []
    validation = load(directory/"replay_validation.json", errors)
    for filename, label, metadata_name in (
            ("replay.mp4", "Annotated overview", "camera.json"),
            ("camera.mp4", "Original overview", "camera.json"),
            ("closeup.mp4", "Caster close-up", "closeup.json")):
        video = directory/filename
        if not video.is_file():
            continue
        metadata = load(directory/metadata_name, errors)
        origin = metadata.get("first_sim_time_s")
        duration = metadata.get("expected_video_duration_s")
        motion_start = summary.get("motion_start_s")
        usable = all(finite(v) for v in (origin, duration, motion_start)) and duration > 0
        # The recorder writes a constant-rate simulation timeline, including
        # duplicate frames. encoded_source_timestamps_s lists fresh frames only
        # and must NOT be interpreted as an array of playback frame indices.
        result.append({"label": label, "url": relative(root, video),
                       "metadata": relative(root, directory/metadata_name) if metadata else None,
                       "synchronizable": usable,
                       "trajectory_origin_s": origin-motion_start if usable else None,
                       "duration_s": duration if finite(duration) else None,
                       "fps": metadata.get("fps"),
                       "max_source_frame_gap_s": metadata.get("max_image_gap_s"),
                       "validated": bool(validation.get("passed")) if filename == "replay.mp4" else
                       bool(summary.get("recording_complete")) if filename == "camera.mp4" else
                       bool(summary.get("closeup_video", {}).get("valid"))})
    return result


def trace_data(directory, errors, max_gap=.25):
    samples = load(directory/"trajectory.json", errors).get("samples", [])
    points, last_stamp = [], None
    for sample in samples:
        if sample.get("phase") not in ("motion", "failure_hold", "final_hold"):
            continue
        position = sample.get("position_m", [])
        stamp = sample.get("odometry_stamp_s", sample.get("simulation_time_s"))
        if len(position) < 2 or not all(finite(v) for v in position[:2]):
            points.append([float("nan"), float("nan")])
            continue
        if finite(stamp) and last_stamp is not None and (stamp <= last_stamp or stamp-last_stamp > max_gap):
            points.append([float("nan"), float("nan")])
        points.append(position[:2])
        last_stamp = stamp if finite(stamp) else None
    return np.asarray(points, dtype=float).reshape(-1, 2)


def collect(root, analysis_path):
    errors = []
    matrix_path = root/"experiment_matrix.json"
    matrix = load(matrix_path, errors)
    if not isinstance(matrix, list) or not matrix:
        raise ValueError("experiment_matrix.json must contain a nonempty run list")
    analysis = load(analysis_path, errors)
    if not isinstance(analysis, dict):
        raise ValueError("Analysis must be a JSON object")
    max_gap = analysis.get('settings', {}).get('max_gap_s', .25)
    if not finite(max_gap) or max_gap <= 0:
        max_gap = .25
    groups, input_hashes = {}, {"experiment_matrix.json": digest(matrix_path)}
    if analysis_path.is_file():
        input_hashes[relative(root, analysis_path)] = digest(analysis_path)
    for case in matrix:
        terrain, shape, model = (case[k] for k in ("terrain", "shape", "caster"))
        if model not in MODELS:
            raise ValueError(f"Unknown caster model in matrix: {model}")
        trial = str(case.get("trial", case.get("repeat", case.get("trial_id", 0))))
        key = (terrain, shape, trial)
        group = groups.setdefault(key, {"terrain": terrain, "shape": shape, "trial": trial,
                                        "models": {}, "pair_analysis": None})
        if model in group["models"]:
            raise ValueError(f"Duplicate matrix condition {key}/{model}; use distinct trial IDs")
        directory = canonical(root, case["directory"])
        summary_path = directory/"run_summary.json"
        summary = load(summary_path, errors)
        rel = directory.relative_to(root)
        analyzed = [run for run in analysis.get("runs", [])
                    if matches_directory(run.get("run_directory"), directory, rel)]
        if len(analyzed) > 1:
            errors.append(f"Duplicate analysis entries for {rel}; metrics withheld")
        measured = analyzed[0] if len(analyzed) == 1 else {}
        artifacts = {}
        for name in ("trajectory.csv", "trajectory.json", "run_summary.json", "configuration.json",
                     "robot.urdf", "world.sdf", "terrain.json", "camera.json", "closeup.json",
                     "replay_validation.json"):
            path = directory/name
            if path.is_file():
                artifacts[name] = relative(root, path)
                if name in ("trajectory.json", "run_summary.json", "configuration.json", "robot.urdf", "world.sdf"):
                    input_hashes[artifacts[name]] = digest(path)
        # Refuse stale numerical analysis instead of silently combining source
        # traces with an earlier analysis of the same directory.
        stale = [name for name, value in measured.get("input_hashes", {}).items()
                 if not (directory/name).is_file() or digest(directory/name) != value]
        group["models"][model] = {
            "directory": rel.as_posix(), "state": run_state(summary, summary_path.exists()),
            "summary": summary, "analysis": measured, "analysis_stale_inputs": stale,
            "artifacts": artifacts, "videos": video_sources(root, directory, summary, errors),
            "route": case.get("route", {}), "trace": trace_data(directory, errors, max_gap),
            "terrain_bounds_m": load(directory/'terrain.json', errors).get('bounds_m', {}),
        }
    for group in groups.values():
        candidates = [p for p in analysis.get("pairs", []) if
                      (p.get("terrain"), p.get("shape"), str(p.get("trial", 0))) ==
                      (group["terrain"], group["shape"], group["trial"])]
        if len(candidates) == 1:
            pair = candidates[0]
            directories_agree = all(
                model in group["models"] and
                matches_directory(pair.get(model+"_run"), root/group["models"][model]["directory"],
                                  Path(group["models"][model]["directory"])) for model in MODELS)
            fresh = all(not model.get("analysis_stale_inputs") for model in group["models"].values())
            if directories_agree and fresh:
                group["pair_analysis"] = pair
            else:
                group["analysis_note"] = "Paired numerical contrast withheld: source directories differ or analysis is stale."
        elif len(candidates) > 1:
            group["analysis_note"] = "Paired numerical contrast withheld: duplicate analysis conditions."
    return list(groups.values()), analysis, input_hashes, errors


def deltas(group):
    pair = group.get("pair_analysis") or {}
    return pair.get("common_horizon_ball_minus_swivel", {}) if pair.get("paired") else {}


def label(group):
    terrain = group["terrain"].removeprefix("caster_stress_").replace("_", " ")
    shape = group["shape"].removeprefix("stress_").replace("_", " ")
    trial = f" · trial {group['trial']}" if group["trial"] != "0" else ""
    return terrain+" / "+shape+trial


def patch_bounds(group):
    """Use saved terrain dimensions, including exploratory 8×8 m worlds."""
    for run in group['models'].values():
        bounds = run.get('terrain_bounds_m', {})
        if all(isinstance(bounds.get(axis), list) and len(bounds[axis]) == 2
               and all(finite(value) for value in bounds[axis])
               and bounds[axis][0] < bounds[axis][1] for axis in ('x', 'y')):
            return bounds
    return {'x':[-3.,3.], 'y':[-4.,4.]}


def make_figures(root, groups):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9,
                         "svg.fonttype": "none", "savefig.facecolor": "white"})
    output = root/"comparison_assets"
    output.mkdir(exist_ok=True)
    height = max(4.2, .30*len(groups)+1.9)
    figure, axes = plt.subplots(1, len(METRICS), figsize=(12, height), sharey=True)
    for axis, (key, title, unit, scale) in zip(axes, METRICS):
        values = [deltas(g).get(key) for g in groups]
        array = np.array([v*scale if finite(v) else np.nan for v in values]).reshape(-1, 1)
        limit = max(float(np.nanmax(abs(array))) if np.isfinite(array).any() else 0., 1e-6)
        cmap = plt.get_cmap("RdBu_r").copy(); cmap.set_bad("#e6e7e8")
        im = axis.imshow(np.ma.masked_invalid(array), cmap=cmap, vmin=-limit, vmax=limit, aspect="auto")
        axis.set_xticks([]); axis.set_title(f"{title}\nΔ ball − swivel ({unit})", fontsize=9)
        axis.set_yticks(range(len(groups)))
        axis.set_yticklabels([label(g) for g in groups], fontsize=8)
        for i, (value, group) in enumerate(zip(array[:, 0], groups)):
            pair = group.get("pair_analysis") or {}
            marker = " !" if any(m["state"] in ("physical failure", "infrastructure failure", "outside tracking limits") for m in group["models"].values()) else ""
            marker += " †" if pair.get("paired") and not pair.get("fully_verified_pair") else ""
            text = (f"{value:+.2f}" if np.isfinite(value) else "—")+marker
            axis.text(0, i, text, ha="center", va="center", fontsize=8,
                      color="white" if np.isfinite(value) and abs(value) > .62*limit else "#152433")
        if np.isfinite(array).any():
            figure.colorbar(im, ax=axis, orientation="horizontal", pad=.03, fraction=.025)
        else:
            axis.set_xlabel("No paired values available", fontsize=8)
    figure.suptitle("Matched common-horizon contrasts · descriptive simulation measurements", y=.995, fontsize=12)
    figure.text(.02, .01, "Gray: unavailable; !: failed/outside-tolerance run retained; †: matching conditions incompletely verified.\nEach column has its own symmetric color scale. Negative means the ball model has the lower metric, not a universal design advantage.", fontsize=8)
    figure.tight_layout(rect=(0, .055, 1, .98))
    for extension in ("png", "svg"):
        figure.savefig(output/f"paired_heatmap.{extension}", dpi=240)
    plt.close(figure)
    def draw(axis, group, compact=False):
        route = next(iter(group["models"].values()))["route"]
        points = np.array(route.get("points_m", []), dtype=float).reshape(-1, 2)
        if len(points):
            axis.plot(points[:, 0], points[:, 1], "--", color="#63776a", lw=1.3, label="Reference", zorder=2)
            axis.scatter(*points[0], marker="o", s=22, color="#243746", zorder=4)
        for model in MODELS:
            run = group["models"].get(model)
            if run is None:
                continue
            xy = run["trace"]
            if len(xy):
                axis.plot(xy[:, 0], xy[:, 1], color=COLORS[model], lw=1.15, label=model.title())
                good = np.isfinite(xy).all(axis=1)
                if good.any():
                    axis.scatter(*xy[good][-1], marker="x", s=32, color=COLORS[model], zorder=5)
        states = "; ".join(f"{m}: {group['models'][m]['state']}" if m in group['models'] else f"{m}: missing from matrix" for m in MODELS)
        title = label(group).replace(" / ", "\n", 1) if compact else label(group)+f"\n{states}"
        axis.set(xlabel="World X (m)", ylabel="World Y (m)", title=title)
        if compact:
            axis.title.set_fontsize(9)
        axis.set_aspect("equal", adjustable="box"); axis.grid(alpha=.2)
        # All references fit the physical patch, but failed trajectories outside
        # the patch remain visible rather than being silently clipped.
        bounds = patch_bounds(group)
        for boundary in bounds['x']:
            axis.axvline(boundary, color="#c2c7cc", lw=.6)
        for boundary in bounds['y']:
            axis.axhline(boundary, color="#c2c7cc", lw=.6)
        axis.legend(fontsize=7, loc="best")

    cols = min(4, len(groups)); rows = math.ceil(len(groups)/cols)
    figure, axes = plt.subplots(rows, cols, figsize=(3.8*cols, 4.1*rows), squeeze=False)
    for i, (axis, group) in enumerate(zip(axes.flat, groups)):
        draw(axis, group, compact=True)
        group["figure_prefix"] = f"comparison_assets/pair_{i+1:03d}"
        detail, detail_axis = plt.subplots(figsize=(6.7, 7.2))
        draw(detail_axis, group)
        detail_axis.title.set_fontsize(9)
        detail.text(.02, .015, "Full observed trajectory, including failure/final holds; × last observed position.\nMatched common-horizon scalar contrasts are reported separately. Gray lines: patch edges.", fontsize=8)
        detail.tight_layout(rect=(0, .065, 1, 1))
        for extension in ("png", "svg"):
            detail.savefig(root/f"{group['figure_prefix']}.{extension}", dpi=240)
        plt.close(detail)
    for axis in list(axes.flat)[len(groups):]:
        axis.axis("off")
    figure.suptitle("Paired measured XY traces · all canonical cases retained", fontsize=13)
    figure.tight_layout(rect=(0, 0, 1, .98))
    for extension in ("png", "svg"):
        figure.savefig(output/f"paired_xy_overview.{extension}", dpi=180)
    plt.close(figure)


def copy_terrain_figures(root, groups):
    """Snapshot source figures into the portable campaign; never link to checkout."""
    from generate_terrains import package_directory
    source = package_directory()/"figures"
    destination = root/"comparison_assets"
    destination.mkdir(exist_ok=True)
    figures = []
    for terrain in dict.fromkeys(g["terrain"] for g in groups):
        item = {"terrain": terrain, "files": {}}
        for extension in ("png", "svg"):
            path = source/f"{terrain}_height_slope.{extension}"
            if not path.is_file():
                continue
            target = destination/path.name
            shutil.copyfile(path, target)
            item["files"][extension] = {"url":relative(root,target), "sha256":digest(target),
                                         "source_file":str(path), "source_sha256":digest(path)}
        if item["files"]:
            figures.append(item)
    return figures


def escaped(value):
    return html.escape(str(value), quote=True)


def link(url, text):
    return f'<a href="{escaped(url)}">{escaped(text)}</a>'


def format_number(value, scale=1., unit=""):
    return f"{value*scale:.3g} {unit}".strip() if finite(value) else "Unavailable"


STYLE = """
:root{--ink:#173043;--muted:#526875;--line:#d8dedf;--paper:#fffefa;--ball:#246a9a;--swivel:#c66a25}
*{box-sizing:border-box}body{margin:0;background:#f0f2ef;color:var(--ink);font:15px/1.55 system-ui,sans-serif}main{max-width:1480px;margin:auto;padding:36px 4vw 70px}h1{font:600 clamp(30px,4vw,54px)/1.12 Georgia,serif;max-width:1050px;margin:12px 0 20px}h2{font:600 28px/1.2 Georgia,serif}h3{margin:0 0 12px}a{color:#205d86;text-underline-offset:3px}.eyebrow{letter-spacing:.13em;text-transform:uppercase;font-size:12px;font-weight:700}.lede{max-width:1020px;font-size:17px}.muted,figcaption{color:var(--muted);font-size:13px}.stats{display:flex;gap:18px;flex-wrap:wrap;margin:28px 0}.stat{background:var(--paper);border-top:3px solid var(--ink);padding:12px 22px;min-width:145px}.stat strong{display:block;font-size:28px}nav{display:flex;gap:16px;flex-wrap:wrap}.panel,.case{background:var(--paper);border:1px solid var(--line);padding:24px;margin:24px 0;border-radius:4px}figure{margin:18px 0}figure img{max-width:100%;height:auto}.figure-grid{display:grid;grid-template-columns:1fr 1fr;gap:24px}.tools{display:flex;gap:12px;align-items:center;flex-wrap:wrap;padding:14px 0;border-block:1px solid var(--line);margin:18px 0}.tools input{flex:1;min-width:170px}.models{display:grid;grid-template-columns:1fr 1fr;gap:20px}.model{border-top:4px solid var(--swivel);padding-top:12px;min-width:0}.model.ball{border-color:var(--ball)}.model video{display:block;width:100%;aspect-ratio:4/3;background:#12212b}.status{display:inline-block;padding:3px 8px;margin-bottom:10px;border-radius:3px;font-size:12px;font-weight:700;background:#e7edf0}.status.failed{background:#f7ded4;color:#842e13}.status.passed{background:#e2efea;color:#265d43}.status.pending{background:#edf0f2;color:#596670}button,select{font:inherit;padding:7px 11px;background:white;border:1px solid #a8b6bd;border-radius:3px;color:var(--ink)}button{cursor:pointer}button:disabled{opacity:.45;cursor:default}select{max-width:100%}.links{display:flex;flex-wrap:wrap;gap:7px 14px;font-size:13px;margin:10px 0}table{width:100%;border-collapse:collapse;font-size:13px}th,td{text-align:left;padding:8px;border-bottom:1px solid var(--line)}th{font-weight:600}details{margin:16px 0}summary{cursor:pointer;font-weight:600}.pair-bottom{display:grid;grid-template-columns:minmax(260px,.7fr) 1fr;gap:22px}.pair-bottom img{width:100%;height:auto}.alert{border-left:3px solid #ad542b;padding-left:12px}.filter{position:sticky;top:0;background:#f0f2eff5;padding:12px 0;z-index:10;display:flex;gap:12px;flex-wrap:wrap}.case[hidden]{display:none}pre{white-space:pre-wrap;word-break:break-word;max-height:400px;overflow:auto;font-size:11px}footer{border-top:1px solid var(--line);padding-top:18px;font-size:12px;color:var(--muted)}@media(max-width:850px){.models,.figure-grid,.pair-bottom{grid-template-columns:1fr}main{padding:24px 16px}.panel,.case{padding:16px}}@media print{.tools,.filter,video{display:none}.case{break-inside:avoid}a{color:inherit}.models{grid-template-columns:1fr 1fr}}
"""

SCRIPT = r"""
'use strict';
const cases=JSON.parse(document.getElementById('comparison-json').textContent);
const timers=[];
document.querySelectorAll('.case').forEach((section,index)=>{
 const data=cases[index], slots=[...section.querySelectorAll('.model')], play=section.querySelector('.play'), restart=section.querySelector('.restart'), slider=section.querySelector('.timeline'), speed=section.querySelector('.speed'), stamp=section.querySelector('.stamp');
 let active=false, target=0, lower=0, upper=0, last=0;
 const selected=()=>slots.map(slot=>{const name=slot.dataset.model, pick=slot.querySelector('select');return {video:slot.querySelector('video'),source:data.models[name]?.videos[Number(pick?.value||0)]};});
 function pause(){active=false;play.textContent='Play synchronized pair';selected().forEach(x=>x.video?.pause());}
 function seek(){selected().forEach(({video,source})=>{if(!video||!source?.synchronizable)return;const time=target-source.trajectory_origin_s;if(Number.isFinite(video.duration))video.currentTime=Math.max(0,Math.min(video.duration-.001,time));});slider.value=target;stamp.textContent=`Trajectory t = ${target.toFixed(2)} s`;}
 function range(){pause();const choices=selected();const valid=choices.length===2&&choices.every(x=>x.video&&x.source?.synchronizable);lower=valid?Math.max(0,...choices.map(x=>x.source.trajectory_origin_s)):0;upper=valid?Math.min(...choices.map(x=>x.source.trajectory_origin_s+x.source.duration_s)):0;const usable=valid&&upper>lower;[play,restart,slider].forEach(x=>x.disabled=!usable);slider.min=lower;slider.max=Math.max(lower,upper-.001);target=lower;seek();if(!usable)stamp.textContent='Synchronization unavailable: two videos and timing metadata are required.';}
 slots.forEach(slot=>{const select=slot.querySelector('select'),video=slot.querySelector('video');if(!select||!video)return;select.addEventListener('change',()=>{pause();video.src=data.models[slot.dataset.model].videos[Number(select.value)].url;video.load();range();});video.addEventListener('loadedmetadata',seek);});
 play.addEventListener('click',()=>{if(active){pause();return;}timers.forEach(stop=>stop());if(target>=upper-.05)target=lower;seek();active=true;last=performance.now();play.textContent='Pause pair';selected().forEach(({video})=>{video.playbackRate=Number(speed.value);video.play().catch(()=>pause());});});
 restart.addEventListener('click',()=>{pause();target=lower;seek();});slider.addEventListener('input',()=>{pause();target=Number(slider.value);seek();});speed.addEventListener('change',()=>selected().forEach(x=>{if(x.video)x.video.playbackRate=Number(speed.value);}));
 function tick(now){if(active){const items=selected();if(items.some(x=>x.video.readyState<3)){last=now;}else{target+=(now-last)/1000*Number(speed.value);last=now;if(target>=upper-.02){target=upper-.02;pause();}items.forEach(({video,source})=>{const wanted=target-source.trajectory_origin_s;if(Math.abs(video.currentTime-wanted)>.12)video.currentTime=Math.max(0,wanted);});slider.value=target;stamp.textContent=`Trajectory t = ${target.toFixed(2)} s`;}}requestAnimationFrame(tick);}timers.push(pause);range();requestAnimationFrame(tick);
});
function filter(){const terrain=document.getElementById('terrain-filter').value,shape=document.getElementById('shape-filter').value,state=document.getElementById('state-filter').value;document.querySelectorAll('.case').forEach((node,index)=>{const item=cases[index],values=Object.values(item.models).map(x=>x.state);node.hidden=(terrain&&item.terrain!==terrain)||(shape&&item.shape!==shape)||(state==='failure'&&!values.some(x=>x.includes('failure')||x==='outside tracking limits'))||(state==='pending'&&!values.includes('pending'));});}
['terrain-filter','shape-filter','state-filter'].forEach(id=>document.getElementById(id).addEventListener('change',filter));
"""


def build_page(root, groups, analysis, report):
    pair_html = []
    for index, group in enumerate(groups):
        pair = group.get("pair_analysis") or {}
        cards = []
        for model in MODELS:
            run = group["models"].get(model)
            if run is None:
                cards.append(f'<article class="model {model}" data-model="{model}"><h3>{model.title()} caster</h3><p>Missing from matrix.</p></article>')
                continue
            state = run["state"]
            css = "passed" if state == "tracking passed" else "pending" if state == "pending" else "failed"
            videos = run["videos"]
            video_html = '<p class="muted">Video pending or unavailable; no recording is implied.</p>'
            if videos:
                options = ''.join(f'<option value="{i}">{escaped(v["label"])}'+(' · unvalidated' if not v['validated'] else '')+'</option>' for i, v in enumerate(videos))
                video_html = f'<select aria-label="{model} video source">{options}</select><video controls muted playsinline preload="none" src="{escaped(videos[0]["url"])}"></video>'
            assets = [link(v["url"], v["label"]+" MP4") for v in videos]
            assets.extend(link(url, name) for name, url in run["artifacts"].items())
            reason = run["summary"].get("physical_failure") or run["summary"].get("infrastructure_failure")
            warning = f'<p class="alert">{escaped(reason)}</p>' if reason else ''
            if run["analysis_stale_inputs"]:
                warning += '<p class="alert">Analysis is stale for '+escaped(', '.join(run['analysis_stale_inputs']))+'. Numerical contrasts are withheld.</p>'
            cards.append(f'<article class="model {model}" data-model="{model}"><h3>{model.title()} caster</h3><span class="status {css}">{escaped(state)}</span>{warning}{video_html}<div class="links">'+''.join(assets)+'</div></article>')
        metrics = []
        common = pair.get("common_horizon_metrics", {}) if pair.get("paired") else {}
        for key, title, unit, scale in TABLE_METRICS:
            model_unit = '%' if key.endswith('fraction') else unit
            delta_unit = 'percentage points' if key.endswith('fraction') else unit
            values = [format_number(common.get(model, {}).get(key), scale, model_unit) for model in MODELS]
            delta = format_number(deltas(group).get(key), scale, delta_unit)
            metrics.append(f'<tr><th>{escaped(title)}</th><td>{values[0]}</td><td>{values[1]}</td><td>{delta}</td></tr>')
        if pair.get("fully_verified_pair"):
            pair_note = 'Matched conditions verified by the analyzer.'
        elif pair.get("paired"):
            pair_note = 'Available conditions match, but some condition fields are missing; verification is incomplete.'
        else:
            pair_note = group.get('analysis_note', 'Matched numerical analysis pending or conditions differ; no paired contrast is shown.')
        horizon = format_number(pair.get('common_observed_horizon_s'), unit='s')
        figure = group.get("figure_prefix")
        figure_html = (f'<figure><a href="{figure}.svg"><img loading="lazy" src="{figure}.png" alt="Paired XY trace for {escaped(label(group))}"></a><figcaption>Full observed traces; '+link(figure+'.png','PNG')+' · '+link(figure+'.svg','editable SVG')+'</figcaption></figure>') if figure else ''
        routes = [m["route"] for m in group["models"].values()]
        speed = format_number(routes[0].get('speed_m_s'), unit='m/s')
        pair_html.append(f'''<section class="case" id="pair-{index+1}"><p class="eyebrow">Case {index+1:02d} · {escaped(speed)}</p><h2>{escaped(label(group))}</h2>
<div class="tools"><button class="play">Play synchronized pair</button><button class="restart">Restart</button><input class="timeline" aria-label="Trajectory time" type="range" step="0.01"><select class="speed" aria-label="Playback speed"><option value="0.25">¼ speed</option><option value="0.5">½ speed</option><option value="1" selected>1× speed</option><option value="2">2× speed</option></select><span class="stamp muted"></span></div>
<div class="models">{''.join(cards)}</div><div class="pair-bottom">{figure_html}<div><h3>Common observed horizon</h3><p>{escaped(pair_note)} Horizon: {horizon}.</p><table><thead><tr><th>Measurement</th><th>Swivel</th><th>Ball</th><th>Ball − swivel</th></tr></thead><tbody>{''.join(metrics)}</tbody></table><p class="muted">Values are time weighted over observed motion intervals. A shorter failed run can have a smaller error; read the outcome and exposure together. Positive differences mean the ball model has the larger value.</p><details><summary>Condition checks and provenance</summary><pre>{escaped(json.dumps(pair or {'status':'Analysis unavailable'},indent=2))}</pre></details></div></div></section>''')
    payload = [{k: v for k, v in g.items() if k != 'pair_analysis'} for g in groups]
    # Only player metadata belongs in the browser payload; raw trajectories and
    # expanded analyses remain separately linked, reducing page memory use.
    payload = [{"terrain": g["terrain"], "shape": g["shape"],
                "models": {m:{"state":r["state"], "videos":r["videos"]} for m,r in g["models"].items()}} for g in payload]
    inline = json.dumps(payload, allow_nan=False).replace('<', '\\u003c').replace('>', '\\u003e').replace('&', '\\u0026')
    navigation = [link('experiment_matrix.json','Experiment matrix'), link('comparison_data.csv','Comparison CSV'), link('comparison_data.json','Comparison JSON'),link('comparison_audit.json','Provenance audit')]
    for name in ('caster_analysis/analysis.json','caster_analysis/runs.json','caster_analysis/paired.json','caster_analysis/runs.csv','caster_analysis/paired.csv','index.html','README.md','FINDINGS.md'):
        if (root/name).is_file():
            navigation.append(link(name,name))
    select = lambda key: ''.join(f'<option value="{escaped(v)}">{escaped(v.replace("_"," "))}</option>' for v in dict.fromkeys(g[key] for g in groups))
    figures = ''
    if groups[0].get('figure_prefix'):
        figures = '''<section class="panel"><h2>Paper figures</h2><p>Four descriptive contrasts share case ordering. Each heatmap column has its own units and color scale. Unavailable values stay gray; failed cases remain in place.</p><div class="figure-grid"><figure><a href="comparison_assets/paired_heatmap.svg"><img src="comparison_assets/paired_heatmap.png" alt="Common horizon ball minus swivel heatmaps"></a><figcaption><a href="comparison_assets/paired_heatmap.png">Heatmap PNG</a> · <a href="comparison_assets/paired_heatmap.svg">Heatmap SVG</a></figcaption></figure><figure><a href="comparison_assets/paired_xy_overview.svg"><img src="comparison_assets/paired_xy_overview.png" alt="All paired observed XY paths"></a><figcaption><a href="comparison_assets/paired_xy_overview.png">XY overview PNG</a> · <a href="comparison_assets/paired_xy_overview.svg">XY overview SVG</a></figcaption></figure></div></section>'''
    terrain_cards = []
    for terrain in report['terrain_figures']:
        files = terrain['files']
        image = f'<img loading="lazy" src="{escaped(files["png"]["url"])}" alt="Height and slope for {escaped(terrain["terrain"])}">' if 'png' in files else ''
        terrain_cards.append('<figure>'+image+'<figcaption>'+escaped(terrain['terrain'])+' · '+
                             ' · '.join(link(item['url'], extension.upper()) for extension,item in files.items())+'</figcaption></figure>')
    terrain_section = '<details class="panel"><summary>Terrain height and slope · portable source figures</summary><div class="figure-grid">'+''.join(terrain_cards)+'</div><p class="muted">These are generated geometry figures, not camera images or measured wheel contact. Figure hashes and source paths are recorded in the provenance audit.</p></details>' if terrain_cards else ''
    clip_sections = []
    for clip in report['paired_video_clips']:
        provenance = ' · '+link(clip['provenance'],'Timing and source hashes') if clip.get('provenance') else ''
        poster = f' poster="{escaped(clip["poster"])}"' if clip.get('poster') else ''
        clip_sections.append(f'<section class="panel"><h2>{escaped(clip["label"])}</h2><video controls muted playsinline preload="none" style="width:100%"{poster} src="{escaped(clip["url"])}"></video><p>'+link(clip['url'],f'Download synchronized {clip["view"]} MP4')+provenance+'</p><p class="muted">Actual saved camera views aligned on the common trajectory clock. Model labels identify the existing assemblies; the clip does not establish a causal design advantage.</p></section>')
    page = f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Caster models · measured terrain comparison</title><style>{STYLE}</style></head><body><main>
<header><p class="eyebrow">HAMR · simulation evidence · offline research view</p><h1>Two caster assemblies.<br>Matched terrain and trajectory demands.</h1><p class="lede">A descriptive comparison of the existing passive swivel-wheel and split-ball caster models. Every requested case appears below, including failed and unfinished runs. The measurements describe these model assemblies; they do not isolate caster topology or establish calibrated hardware performance.</p><nav>{' '.join(navigation)}</nav></header>
<div class="stats"><div class="stat"><strong>{report['expected_runs']}</strong>requested runs</div><div class="stat"><strong>{report['expected_pairs']}</strong>paired cases</div><div class="stat"><strong>{report['summaries_available']}</strong>run summaries</div><div class="stat"><strong>{report['verified_pairs']}</strong>verified matched analyses</div><div class="stat"><strong>{report['failure_runs']}</strong>failures / outside tolerance</div></div>
<section class="panel"><h2>How to read the comparison</h2><p>The common-horizon contrasts use the analyzer’s overlap of observed trajectory time. Full XY traces include final or failure holds and are marked separately. Low progress measures insufficient velocity projected along the demanded direction; it is not proof of motor stall. World-root height includes terrain climb. Caster wheel-center transverse velocity, where available in JSON, is a planar kinematic proxy, not measured contact slip.</p><p>Pair playback aligns <strong>trajectory time</strong> using each camera’s simulation-time origin and that vehicle’s motion-start time. It stops at the shorter available video interval. Browser synchronization is approximate (drift corrected above 0.12 s); use source timestamps and CSV for numerical timing. Source MP4s are unchanged. Individual video controls and downloads remain available.</p><p class="muted">No inferential significance tests, winner ranking, or causal design claim is generated. Mechanical dimensions, inertia, damping and contact geometry differ between the assemblies. Only canonical matrix directories are included.</p></section>
{terrain_section}{''.join(clip_sections)}{figures}<h2>Inspect every paired case</h2><div class="filter"><select id="terrain-filter" aria-label="Terrain filter"><option value="">All terrains</option>{select('terrain')}</select><select id="shape-filter" aria-label="Path filter"><option value="">All paths</option>{select('shape')}</select><select id="state-filter" aria-label="Outcome filter"><option value="">All outcomes</option><option value="failure">Failures / outside tolerance</option><option value="pending">Pending runs</option></select></div>{''.join(pair_html)}
<footer>Generated {escaped(report['generated_utc'])}. Input files and SHA-256 hashes are recorded in comparison_audit.json. Rebuild to include new results; this page makes no network requests.</footer>
<script id="comparison-json" type="application/json">{inline}</script><script>{SCRIPT}</script></main></body></html>'''
    (root/'comparison.html').write_text(page)


def build_comparison(root, analysis_path=None, figures=True):
    root = Path(root).expanduser().resolve()
    analysis_path = Path(analysis_path).resolve() if analysis_path else root/'caster_analysis/analysis.json'
    if not analysis_path.is_relative_to(root):
        raise ValueError('Analysis must be inside the campaign root for portable offline links')
    groups, analysis, hashes, errors = collect(root, analysis_path)
    terrain_figures = copy_terrain_figures(root, groups)
    clips = []
    for stem, label, view in PAIRED_VIDEO_CLIPS:
        clip = root/'comparison_assets'/f'{stem}.mp4'
        if not clip.is_file():
            continue
        provenance = clip.with_suffix('.json')
        poster_candidates = [clip.with_name(stem+'_poster.png'), clip.with_suffix('.png'),
                             *sorted(clip.parent.glob(stem+'_frame*.png'))]
        poster = next((p for p in poster_candidates if p.is_file()), None)
        clips.append({'label':label,'view':view,
                      'url':relative(root,clip),'sha256':digest(clip),
                      'provenance':relative(root,provenance) if provenance.is_file() else None,
                      'provenance_sha256':digest(provenance) if provenance.is_file() else None,
                      'poster':relative(root,poster) if poster else None,
                      'poster_sha256':digest(poster) if poster else None})
    if figures:
        make_figures(root, groups)
    models = [r for g in groups for r in g['models'].values()]
    report = {'schema_version': 1, 'generated_utc': datetime.now(timezone.utc).isoformat(),
              'expected_runs': len(models), 'expected_pairs': len(groups),
              'summaries_available': sum('run_summary.json' in r['artifacts'] for r in models),
              'pending_runs': sum(r['state']=='pending' for r in models),
              'failure_runs': sum(r['state'] in ('physical failure','infrastructure failure','outside tracking limits') for r in models),
              'verified_pairs': sum(bool((g.get('pair_analysis') or {}).get('fully_verified_pair')) for g in groups),
              'source_video_files': sum(len(r['videos']) for r in models),
              'runs_with_validated_overviews': sum(all(any(v['label']==name and v['validated'] for v in r['videos']) for name in ('Annotated overview','Original overview')) for r in models),
              'analysis_available': analysis_path.is_file(), 'input_errors': errors,
              'terrain_figures': terrain_figures,
              'paired_video_clips': clips,
              'builder_sha256': digest(__file__), 'input_sha256': hashes,
              'scope': 'Descriptive saved simulation artifacts; figures and HTML do not validate or alter physical motion.'}
    report['complete'] = (report['summaries_available']==report['expected_runs']
                          and report['verified_pairs']==report['expected_pairs']
                          and report['runs_with_validated_overviews']==report['expected_runs'] and not errors)
    rows = []
    for group in groups:
        pair = group.get('pair_analysis') or {}
        row = {k:group[k] for k in ('terrain','shape','trial')}
        row.update({model+'_state': group['models'].get(model,{}).get('state','missing_from_matrix') for model in MODELS})
        row.update(common_observed_horizon_s=pair.get('common_observed_horizon_s'),
                   paired=bool(pair.get('paired')),fully_verified_pair=bool(pair.get('fully_verified_pair')))
        for key, _, _, _ in TABLE_METRICS:
            row['common_horizon_ball_minus_swivel_'+key] = deltas(group).get(key)
        rows.append(row)
    save(root/'comparison_data.json', {'scope':'Common-horizon analyzer contrasts; units are encoded in metric names. Fractions are not percentages.', 'pairs':rows})
    with (root/'comparison_data.csv').open('w',newline='') as handle:
        writer=csv.DictWriter(handle,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
    build_page(root, groups, analysis, report)
    report['output_sha256'] = {p.relative_to(root).as_posix():digest(p) for p in [root/'comparison.html', root/'comparison_data.json', root/'comparison_data.csv', *sorted((root/'comparison_assets').glob('*'))] if p.is_file()}
    save(root/'comparison_audit.json', report)
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path,help='Directory containing experiment_matrix.json')
    parser.add_argument('--analysis',type=Path,help='Analysis JSON inside root; default caster_analysis/analysis.json')
    parser.add_argument('--no-figures',action='store_true',help='Lightweight page/data refresh; do not render figures')
    parser.add_argument('--require-complete',action='store_true')
    args=parser.parse_args(argv)
    result=build_comparison(args.root,args.analysis,not args.no_figures)
    print(json.dumps({k:v for k,v in result.items() if k not in ('input_sha256','output_sha256')},indent=2))
    return int(args.require_complete and not result['complete'])


if __name__=='__main__':
    raise SystemExit(main())
