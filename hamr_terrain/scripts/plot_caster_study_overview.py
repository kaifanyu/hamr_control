#!/usr/bin/env python3
"""Export the frozen 6-terrain x 4-route caster study as publication figures.

Uses archived common-horizon metrics; never starts or modifies a simulation.
Refuses incomplete matrices, stale source data, or changed terrain grids.
"""
import argparse
import csv
import hashlib
import html
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
from matplotlib.colors import LinearSegmentedColormap
from matplotlib.patches import Rectangle
import numpy as np


TERRAINS = {
    "caster_stress_washboard_l1": "Washboard · 20 mm",
    "caster_stress_washboard_l2": "Washboard · 36 mm",
    "caster_stress_staggered_l1": "Staggered pads · 30 mm",
    "caster_stress_staggered_l2": "Staggered pads · 55 mm",
    "caster_stress_saddle_l1": "Cross-slopes · 35 mm",
    "caster_stress_saddle_l2": "Cross-slopes · 60 mm",
}
ROUTES = {
    "stress_reversal": "Reversal",
    "stress_tight_figure_eight": "Tight figure-eight",
    "stress_lateral": "Lateral turns",
    "stress_zigzag": "Zigzag",
}
COLORS = {"swivel": "#c66a25", "ball": "#246a9a"}
LABELS = {"swivel": "Ordinary swivel", "ball": "Split ball"}
DELTA_CMAP = LinearSegmentedColormap.from_list(
    "caster_delta", [COLORS["ball"], "#fafafa", COLORS["swivel"]])
NOTE = ("48 runs · 24 matched conditions · nominal speed 0.25 m/s · one execution per model per cell\n"
        "Descriptive simulation results; no confidence intervals. Exploratory scan-derived tests are excluded.")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text())


def collect(root, package):
    matrix_file = root / "experiment_matrix.json"
    analysis_file = root / "caster_analysis/analysis.json"
    matrix, analysis = read(matrix_file), read(analysis_file)
    hashes = {str(p): digest(p) for p in (matrix_file, analysis_file, Path(__file__))}
    expected = {(t, s, m) for t in TERRAINS for s in ROUTES for m in COLORS}
    keys = [(r["terrain"], r["shape"], r["caster"]) for r in matrix]
    if len(keys) != 48 or set(keys) != expected:
        raise ValueError("Expected the complete fixed 6 x 4 x 2 primary matrix")
    runs = {(r["terrain"], r["shape"], r["caster"]): r for r in analysis["runs"]}
    if len(analysis["runs"]) != 48 or set(runs) != expected:
        raise ValueError("Analysis and experiment matrix disagree")
    routes, terrains, grids = {}, {}, {}
    for case in matrix:
        key = (case["terrain"], case["shape"], case["caster"])
        run = runs[key]
        directory = (root / case["directory"]).resolve()
        if not directory.is_relative_to(root):
            raise ValueError("Run lies outside campaign")
        if run["input_errors"] or run["status"] != "complete" or not run["tracking_passed_reported"]:
            raise ValueError(f"Cannot label this campaign all-passed: {key}")
        for name, saved_hash in run["input_hashes"].items():
            path = directory / name
            hashes[str(path)] = digest(path)
            if hashes[str(path)] != saved_hash:
                raise ValueError(f"Stale analysis: {path}")
        route = case["route"]
        if route["speed_m_s"] != .25 or run["condition"]["route"] != route:
            raise ValueError(f"Reference mismatch: {key}")
        if case["shape"] in routes and routes[case["shape"]] != route:
            raise ValueError("Different reference on different terrains")
        routes[case["shape"]] = route
        terrain_file = directory / "terrain.json"
        hashes[str(terrain_file)] = digest(terrain_file)
        terrain = read(terrain_file)
        grid_path = (package / "config" / terrain["height_grid"]).resolve()
        if str(grid_path) not in hashes:
            hashes[str(grid_path)] = digest(grid_path)
        if hashes[str(grid_path)] != run["condition"]["grid_sha256"]:
            raise ValueError(f"Terrain grid changed: {grid_path}")
        if case["terrain"] not in grids:
            with np.load(grid_path) as saved:
                grids[case["terrain"]] = {k: saved[k].copy() for k in ("x", "y", "z")}
        terrains[case["terrain"]] = terrain
    pairs = {(p["terrain"], p["shape"]): p for p in analysis["pairs"]}
    if len(analysis["pairs"]) != 24 or set(pairs) != {(t, s) for t in TERRAINS for s in ROUTES}:
        raise ValueError("Incomplete or duplicated paired results")
    for key, pair in pairs.items():
        if not pair["fully_verified_pair"] or pair["condition_mismatches"]:
            raise ValueError(f"Unverified pair: {key}")
        for metric in ("xy_rmse_m", "xy_p95_m", "xy_peak_m", "roll_rms_deg", "pitch_rms_deg"):
            values = [pair["common_horizon_metrics"][m][metric] for m in COLORS]
            delta = pair["common_horizon_ball_minus_swivel"][metric]
            if not np.isfinite(values).all() or not np.isclose(values[1]-values[0], delta, atol=1e-12):
                raise ValueError(f"Invalid paired metric: {key}/{metric}")
    return pairs, routes, terrains, grids, hashes


def finish(fig, title, subtitle, footer=NOTE):
    fig.suptitle(title, x=.04, y=.98, ha="left", fontsize=18, weight="bold")
    fig.text(.04, .928, subtitle, fontsize=10, color="#435361")
    fig.text(.04, .025, footer, fontsize=8, color="#435361", va="bottom")


def tracking_bars(pairs):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), sharex=True)
    fig.subplots_adjust(left=.18, right=.97, bottom=.12, top=.83, hspace=.38, wspace=.62)
    largest = max(p["common_horizon_metrics"][m]["xy_rmse_m"]*1000 for p in pairs.values() for m in COLORS)
    for ax, shape in zip(axes.flat, ROUTES):
        y = np.arange(len(TERRAINS))
        for model, offset in (("swivel", -.18), ("ball", .18)):
            vals = [pairs[t, shape]["common_horizon_metrics"][model]["xy_rmse_m"]*1000 for t in TERRAINS]
            bars = ax.barh(y+offset, vals, height=.32, color=COLORS[model], label=LABELS[model], zorder=3)
            ax.bar_label(bars, fmt="%.2f", fontsize=8, padding=3)
        ax.set_yticks(y, list(TERRAINS.values()), fontsize=9)
        ax.invert_yaxis()
        ax.set_xlim(0, largest*1.18)
        ax.set_xlabel("Timed XY tracking RMS (mm) — lower is better")
        ax.set_title(ROUTES[shape], loc="left", weight="bold", pad=10)
        ax.grid(axis="x", alpha=.22, zorder=0)
    handles, labels = axes.flat[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper left", bbox_to_anchor=(.035, .903), ncol=2, frameon=False)
    finish(fig, "Tracking comparison across all 24 conditions", "Each bar uses the pair's common observed motion window. All 48 runs passed tracking acceptance.")
    return fig


def difference_maps(pairs, metrics, title):
    fig, axes = plt.subplots(1, len(metrics), figsize=(14, 6.8), squeeze=False)
    fig.subplots_adjust(left=.18, right=.97, bottom=.19, top=.80, wspace=.35)
    for ax, (metric, label, scale, unit) in zip(axes.flat, metrics):
        data = np.array([[pairs[t, s]["common_horizon_ball_minus_swivel"][metric]*scale for s in ROUTES] for t in TERRAINS])
        limit = max(abs(data).max(), 1e-9)
        im = ax.imshow(data, cmap=DELTA_CMAP, vmin=-limit, vmax=limit, aspect="auto")
        ax.set_xticks(range(4), list(ROUTES.values()), fontsize=9)
        ax.set_yticks(range(6), list(TERRAINS.values()) if ax is axes[0, 0] else [])
        ax.set_title(label, loc="left", weight="bold", pad=12)
        for (row, col), value in np.ndenumerate(data):
            ax.text(col, row, f"{value:+.2f}", ha="center", va="center", fontsize=11,
                    color="white" if abs(value) > .66*limit else "#182b3a")
        ax.set_xticks(np.arange(-.5, 4, 1), minor=True)
        ax.set_yticks(np.arange(-.5, 6, 1), minor=True)
        ax.grid(which="minor", color="white", linewidth=2)
        ax.tick_params(which="minor", bottom=False, left=False)
        fig.colorbar(im, ax=ax, orientation="horizontal", pad=.14, fraction=.045,
                     label=f"Split ball − ordinary swivel ({unit}); independent panel scale")
    finish(fig, title, "Blue / negative: ball has the lower value. Orange / positive: swivel has the lower value.")
    return fig


def route_figure(routes):
    fig, axes = plt.subplots(1, 4, figsize=(14, 6.4), sharex=True, sharey=True)
    fig.subplots_adjust(left=.06, right=.98, bottom=.23, top=.81, wspace=.22)
    details = ["Two 180° travel reversals", "Continuous changing curvature", "Two 90° travel turns", "Alternating diagonal turns"]
    for ax, (shape, label), detail in zip(axes, ROUTES.items(), details):
        route = routes[shape]
        xy = np.array(route["points_m"])
        ax.plot(xy[:, 0], xy[:, 1], color="#334f65", linewidth=2)
        ax.scatter(*xy[0], s=65, facecolor="white", edgecolor="#334f65", linewidth=2, zorder=4)
        if shape == "stress_reversal":
            for x, a, b, text in ((-.24, -.6, .6, "1, 3"), (.24, .6, -.6, "2")):
                ax.annotate("", xy=(x, b), xytext=(x, a), arrowprops=dict(arrowstyle="->", color="#334f65", lw=1.5))
                ax.text(x+(-.2 if x<0 else .15), 0, text, ha="center")
        else:
            indices = np.linspace(50, len(xy)-60, 3).astype(int) if len(xy)>100 else np.arange(len(xy)-1)
            for i in indices:
                a, b = (xy[i], xy[i+18]) if len(xy)>100 else (.6*xy[i]+.4*xy[i+1], .4*xy[i]+.6*xy[i+1])
                ax.annotate("", xy=b, xytext=a, arrowprops=dict(arrowstyle="->", color="#334f65", lw=1.5))
        ax.set_title(label, loc="left", weight="bold", pad=12)
        ax.set(xlim=(-1.65, 1.65), ylim=(-1.75, 1.75), xlabel="World X (m)")
        ax.set_aspect("equal")
        ax.grid(alpha=.2)
        ax.text(.5, -.30, f"{detail}\n{route['route_length_m']:.2f} m · {route['reference_duration_s']:.2f} s", transform=ax.transAxes, ha="center", fontsize=9)
    axes[0].set_ylabel("World Y (m)")
    finish(fig, "The four prescribed trajectories", "The identical reference is tested on every terrain and with both vehicles. Open circle = start.",
           "Reference routes, not measured tracks. All axes use the same meter scale; reversal arrows are offset for readability.\n"
           "Reversal, lateral and zigzag routes brake to rest at corners. Chassis heading is a controller outcome, not prescribed here.")
    return fig


def terrain_figure(terrains, grids):
    order = list(TERRAINS)[::2] + list(TERRAINS)[1::2]
    fig, axes = plt.subplots(2, 3, figsize=(13, 10), sharex=True, sharey=True)
    fig.subplots_adjust(left=.07, right=.87, bottom=.11, top=.86, hspace=.28, wspace=.12)
    for ax, terrain_id in zip(axes.flat, order):
        grid = grids[terrain_id]
        im = ax.pcolormesh(grid["x"], grid["y"], grid["z"]*1000, shading="auto", cmap="terrain", vmin=0, vmax=60, rasterized=True)
        ax.add_patch(Rectangle((-1.8, -2), 3.6, 4, fill=False, edgecolor="#202e39", linestyle="--", linewidth=1))
        ax.set_title(TERRAINS[terrain_id], fontsize=11, loc="left", weight="bold")
        ax.set_aspect("equal")
        ax.set(xlabel="World X (m)", ylabel="World Y (m)")
    cax = fig.add_axes([.90, .23, .015, .50])
    fig.colorbar(im, cax=cax, label="Surface elevation (mm) — shared scale")
    finish(fig, "Six maps: three terrain families at two heights", "Top row: level 1. Bottom row: level 2. Each map is 6 × 8 m; only height changes within a family.",
           "Generated collision-grid height plots, not simulator camera images. Dashed box: intended route / approximate footprint region.\n"
           "Cross-slope levels are nominally 35 / 60 mm; sampled relief is 34.945 / 59.906 mm. Flat margins lie outside the feature field.")
    return fig


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replays", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    root = args.replays.expanduser().resolve()
    output = (args.output or root.parent / "paper_figures/overview").expanduser().resolve()
    package = Path(__file__).resolve().parents[1]
    pairs, routes, terrains, grids, hashes = collect(root, package)
    output.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10, "svg.fonttype": "none",
                         "pdf.fonttype": 42, "axes.spines.top": False, "axes.spines.right": False,
                         "axes.edgecolor": "#c0c9ce", "savefig.facecolor": "white"})
    specs = [
        ("tracking_rms", "Tracking RMS", lambda: tracking_bars(pairs)),
        ("tracking_differences", "Tracking differences", lambda: difference_maps(pairs,
            [("xy_rmse_m", "Difference in timed XY RMS", 1000, "mm"),
             ("xy_peak_m", "Difference in sampled peak XY error", 1000, "mm")], "Where each vehicle tracks more accurately")),
        ("body_attitude_differences", "Body attitude differences", lambda: difference_maps(pairs,
            [("roll_rms_deg", "Difference in body roll RMS", 1, "deg"),
             ("pitch_rms_deg", "Difference in body pitch RMS", 1, "deg")], "Body roll and pitch do not rank the designs identically")),
        ("trajectory_layout", "Four trajectories", lambda: route_figure(routes)),
        ("terrain_layout", "Six terrain maps", lambda: terrain_figure(terrains, grids)),
    ]
    with PdfPages(output / "caster_comparison_plots.pdf") as pdf:
        for name, _, render in specs:
            fig = render()
            for extension in ("png", "svg", "pdf"):
                fig.savefig(output / f"{name}.{extension}", dpi=200)
            pdf.savefig(fig)
            plt.close(fig)
    rows = []
    for t in TERRAINS:
        for s in ROUTES:
            p = pairs[t, s]
            for model in COLORS:
                rows.append({"terrain": t, "route": s, "model": model, "tracking_passed": True,
                             "common_start_s": p["common_observed_start_s"],
                             "common_duration_s": p["common_observed_horizon_s"],
                             **p["common_horizon_metrics"][model]})
    with (output / "common_horizon_metrics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    delta = [p["common_horizon_ball_minus_swivel"]["xy_rmse_m"]*1000 for p in pairs.values()]
    summary = {"runs": 48, "pairs": 24, "trajectories": 4, "maps": 6, "tracking_passes": 48,
               "ball_lower_rms_cells": sum(x < 0 for x in delta),
               "equal_cell_mean_ball_minus_swivel_rms_mm": float(np.mean(delta)),
               "scope": NOTE, "input_sha256": hashes,
               "reproduce": f"/usr/bin/python3 {Path(__file__).resolve()} {root} --output {output}"}
    sections = "\n".join(f'<section id="{n}"><h2>{html.escape(title)}</h2><p>' +
        ' · '.join(f'<a href="{n}.{ext}">{ext.upper()}</a>' for ext in ("png", "svg", "pdf")) +
        f'</p><a href="{n}.png"><img src="{n}.png" alt="{html.escape(title)}"></a></section>' for n, title, _ in specs)
    lower_roll = sum(p["common_horizon_ball_minus_swivel"]["roll_rms_deg"] > 0 for p in pairs.values())
    lower_pitch = sum(p["common_horizon_ball_minus_swivel"]["pitch_rms_deg"] < 0 for p in pairs.values())
    lower_rms = summary["ball_lower_rms_cells"]
    mean_delta = summary["equal_cell_mean_ball_minus_swivel_rms_mm"]
    largest_gap = max(pairs.values(), key=lambda p: abs(p["common_horizon_ball_minus_swivel"]["xy_rmse_m"]))
    ball_advantage = min(pairs.values(), key=lambda p: p["common_horizon_ball_minus_swivel"]["xy_rmse_m"])
    def case_text(p):
        values = p["common_horizon_metrics"]
        return (f"{TERRAINS[p['terrain']]} / {ROUTES[p['shape']]} "
                f"(swivel {values['swivel']['xy_rmse_m']*1000:.3f} mm; "
                f"ball {values['ball']['xy_rmse_m']*1000:.3f} mm)")
    findings = (f"<p>Ball has lower timed XY RMS in {lower_rms} of 24 cells, but the equally weighted mean "
                f"difference is only {mean_delta:+.3f} mm. Swivel has lower roll RMS in {lower_roll} cells, "
                f"while ball has lower pitch RMS in {lower_pitch}. These counts describe this matrix; "
                "they are not statistical evidence of a universal winner.</p>"
                f"<p>The largest absolute primary RMS gap is {html.escape(case_text(largest_gap))}. "
                f"The strongest ball-favoring RMS contrast is {html.escape(case_text(ball_advantage))}.</p>"
                "<p>Low-washboard reversal has substantially different chassis yaw histories "
                f"(swivel {pairs['caster_stress_washboard_l1', 'stress_reversal']['common_horizon_metrics']['swivel']['body_yaw_maximum_contiguous_span_deg']:.1f}° "
                f"versus ball {pairs['caster_stress_washboard_l1', 'stress_reversal']['common_horizon_metrics']['ball']['body_yaw_maximum_contiguous_span_deg']:.1f}° span). "
                "Geometry, mass, friction and bearing assumptions also differ between the configured vehicles.</p>")
    (output / "index.html").write_text('''<!doctype html><html lang="en"><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1"><title>HAMR caster comparison plots</title>
<style>body{font:17px/1.6 system-ui,sans-serif;color:#203443;background:#f4f6f8;max-width:1200px;margin:auto;padding:28px}h1{line-height:1.2}section{background:white;padding:22px;margin:28px 0;border-radius:12px}img{width:100%;height:auto}a{color:#176994}p{max-width:1000px}</style>
<h1>Four trajectories × six maps × two caster configurations</h1>
<p>The fixed study contains 48 runs in 24 matched conditions at a nominal 0.25 m/s.
All passed the simulator's tracking acceptance. The six maps are three terrain families at two heights.
These routes stress different maneuvers; they are not an established optimum test set.</p>
<p><a href="caster_comparison_plots.pdf">Download all five figures (PDF)</a> ·
<a href="common_horizon_metrics.csv">Download all plotted source metrics (CSV)</a> ·
<a href="plot_manifest.json">Provenance and reproduction command</a></p>
''' + findings + '''
<p>Metrics use the archived common observed motion window for each pair. RMS measures error against the
time-indexed reference, not distance to the nearest point on the path. Peak values are sampled peaks.
One nominal execution per model per cell; no error bars or inferential claims. Roll/pitch describe body
motion, not a measured tipping margin. Scan-derived exploratory tests are excluded.</p>
''' + sections + '</html>\n')
    summary["output_sha256"] = {p.name: digest(p) for p in output.iterdir() if p.is_file() and p.name != "plot_manifest.json"}
    (output / "plot_manifest.json").write_text(json.dumps(summary, indent=2, allow_nan=False)+"\n")
    print(json.dumps({k: v for k, v in summary.items() if not k.endswith("sha256")}, indent=2))


if __name__ == "__main__":
    main()
