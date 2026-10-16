#!/usr/bin/env python3
"""Build an honest gallery of official ADTG standalone terrain adaptations.

Collection completeness is separate from physical pass/fail. Failed runs and
their verified Gazebo images remain visible; archived attempts are not silently
substituted. This script never runs inference or the simulator.
"""
import argparse
import hashlib
from html import escape
import json
from pathlib import Path

import numpy as np

from build_terrain_gallery import (CSS, figure, href, link, number, probe_table,
                                   profiles, read_report, report_links,
                                   robot_table, status)


UPSTREAM = "https://github.com/youwyu/Adaptive-Diffusion-Terrain/tree/262488cd981f616898541198537a4b302fa2922a/contexts/standalone"
KINDS = ("probes", "swivel", "ball")


def default_metadata():
    source = Path(__file__).resolve().parents[1]/"config/adtg_terrains.json"
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("hamr_terrain"))/"config/adtg_terrains.json"


def verified_image(directory, report, key):
    """Verify recorded origin and bytes without filtering physical outcomes."""
    entry = (report or {}).get("images", {}).get(key, {})
    if entry.get("source") != "Actual Gazebo camera sensor":
        return None
    filename = entry.get("file")
    if not isinstance(filename, str):
        return None
    path = (directory/filename).resolve()
    if not path.is_relative_to(directory.resolve()) or not path.is_file():
        return None
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest != entry.get("sha256"):
        return None
    return {"path": path, "sha256": digest,
            "source_stamp_s": entry.get("source_stamp_s")}


def camera_figure(image, root, caption):
    if image is None:
        return '<div class="placeholder">'+escape(caption)+"<br>No verified camera image available.</div>"
    return figure(image, root, caption)


def metadata_files(value, base):
    """Expose existing provenance files, resolving paths relative to metadata."""
    paths = []
    if isinstance(value, dict):
        for key, item in value.items():
            for label, path in metadata_files(item, base):
                paths.append((f"{key}.{label}".rstrip("."), path))
    elif isinstance(value, list):
        for i, item in enumerate(value):
            paths.extend((f"{i}.{label}".rstrip("."), path)
                         for label, path in metadata_files(item, base))
    elif isinstance(value, str) and len(value) < 4096 and "\n" not in value:
        candidate = Path(value).expanduser()
        if not candidate.is_absolute():
            candidate = base/candidate
        try:
            if candidate.is_file():
                paths.append(("", candidate.resolve()))
        except OSError:
            pass
    return paths


def diagnostics(metadata, metadata_directory):
    records = {}
    for terrain in metadata:
        with np.load((metadata_directory/terrain["height_grid"]).resolve()) as grid:
            x, y, z = (grid[k] for k in ("x", "y", "z"))
        dy, dx = np.gradient(z, y, x)
        records[terrain["id"]] = {
            "shape": list(z.shape), "minimum_m": float(z.min()),
            "maximum_m": float(z.max()), "relief_m": float(np.ptp(z)),
            "height_standard_deviation_m": float(z.std()),
            "grid_node_maximum_slope_deg": float(np.degrees(np.arctan(np.hypot(dx, dy))).max()),
        }
    return records


def generation_overview(suite, metadata, stats):
    """Present measured raw output alongside the geometry used in Gazebo."""
    manifest_path = suite/"raw/manifest.json"
    if not manifest_path.exists() and metadata:
        recorded = metadata[0].get("generation", {}).get("raw_manifest")
        if recorded:
            manifest_path = Path(recorded)
    if not manifest_path.exists():
        return '<p class="pending">Raw generation manifest not yet available.</p>'
    manifest = json.loads(manifest_path.read_text())
    request = manifest.get("generation", {})
    shape, seed = request.get("shape_y_x", [None, None]), request.get("seed")
    scales = sorted({t.get("adaptation", {}).get("vertical_multiplier") for t in metadata} - {None})
    scale_description = (f'a shared {scales[0]:g} height multiplier' if len(scales)==1
                         else 'the height multipliers recorded below')
    samples = {int(s["difficulty"]): s for s in manifest.get("samples", [])}
    rows = []
    for terrain in metadata:
        try:
            difficulty = int(terrain["id"].split("_d")[-1])
        except ValueError:
            continue
        raw, adapted = samples.get(difficulty, {}), stats[terrain["id"]]
        rows.append(f'<tr><td>{difficulty}</td><td>{number(raw.get("height_max_m"),digits=3)}</td>'
                    f'<td>{number(raw.get("height_percentiles_m",{}).get("p95"),digits=3)}</td>'
                    f'<td>{number(adapted["maximum_m"],1000,1)}</td>'
                    f'<td>{number(terrain.get("corridor_slope_p95_deg"),digits=2)}</td>'
                    f'<td>{number(adapted["grid_node_maximum_slope_deg"],digits=2)}</td></tr>')
    checkpoint = manifest.get("official_checkpoint", {})
    raw_directory = manifest_path.parent
    links = " · ".join(link(raw_directory/filename,suite,label) for filename,label in
        (("manifest.json","Generation manifest"),("generate_raw.py","Exact inference script"),
         ("sample_adtg.py","Recorded inference script"),
         ("requirements.lock.txt","Inference environment"),("generation.log","Inference log"),
         ("README.md","Raw-output notes")) if (raw_directory/filename).exists())
    return ('<section class="card"><h2>Raw model output and physical adaptation</h2>'
        f'<p>Each source array has {shape[0]} × {shape[1]} samples. Seed {seed} is reset for every difficulty, '
        'so the input noise is shared across this set. The official API uses overlapping tile blending '
        f'and zero-floor decoding. HAMR uses {scale_description} and 25 mm horizontal spacing '
        '(6 × 8 m), with flat approach and boundary transitions. No per-map amplitude normalization '
        'or additional smoothing is applied.</p>'
        '<p class="small">The boundary transition multiplies heights near the edges, so the final '
        'maximum need not equal the vertical multiplier times the raw maximum. Neither peak height nor traversability '
        'is guaranteed to increase with the requested difficulty label.</p>'
        '<div class="table-scroll"><table><thead><tr><th>Difficulty</th><th>Raw maximum (m)</th>'
        '<th>Raw 95th-percentile height (m)</th><th>Adapted maximum (mm)</th>'
        '<th>Corridor 95th-percentile grade (°)</th>'
        '<th>Adapted max grid grade (°)</th></tr></thead><tbody>'+"".join(rows)+'</tbody></table></div>'
        '<p class="small">Corridor statistics use |x| ≤ 0.6 m and −0.8 ≤ y ≤ 2.4 m, excluding the approach and exit blends. Whole-surface maxima include the blends and regions outside that corridor.</p>'
        '<p class="small">Recorded checkpoint SHA-256:</p><pre>'+escape(checkpoint.get("sha256","Unavailable"))+'</pre>'
        '<p class="links">'+links+'</p></section>')


def contact_sheet(entries, output, difficulty_labels):
    if not entries:
        return False
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = (len(entries)+2)//2
    fig, axes = plt.subplots(rows, 2, figsize=(13, 4*rows), layout="constrained", squeeze=False)
    for ax in axes.flat:
        ax.set_axis_off()
    for ax, (title, image) in zip(axes.flat, entries):
        ax.imshow(plt.imread(image["path"]))
        ax.set_title(title, fontsize=12, loc="left")
    axes.flat[-1].text(.03, .9,
        f"Actual Gazebo camera captures\n\nOfficial learned heightfields\nwith recorded HAMR scaling\n\nDifficulty labels {' / '.join(map(str,difficulty_labels))}\nare generator inputs, not\nmeasured robot capability.\n\nProfile plots separately exaggerate\nheight scale for readability.",
        va="top", fontsize=13, linespacing=1.5, transform=axes.flat[-1].transAxes)
    fig.savefig(output, dpi=150, facecolor="white")
    plt.close(fig)
    return True


def build_gallery(suite, metadata_path):
    suite = Path(suite).expanduser().resolve()
    metadata_path = Path(metadata_path).expanduser().resolve()
    suite.mkdir(parents=True, exist_ok=True)
    metadata = json.loads(metadata_path.read_text())
    if isinstance(metadata, dict):
        metadata = metadata["terrains"]
    profiles(metadata, metadata_path.parent, suite/"profiles")
    stats = diagnostics(metadata, metadata_path.parent)
    difficulty_labels = [t.get("generation", {}).get("difficulty",t["id"]) for t in metadata]
    records, images, cards, sheet = [], [], [], []
    for terrain in metadata:
        name = terrain["id"]
        directories = {kind: suite/name/kind for kind in KINDS}
        reports = {kind: read_report(path) for kind, path in directories.items()}
        records.append({"id": name, "reports": {
            kind: {"available": report is not None,
                   "passed": bool(report and report.get("passed"))}
            for kind, report in reports.items()}})
        selected = {}
        for kind in KINDS:
            for key in (("overview", "detail") if kind == "probes" else ("final_detail",)):
                item = verified_image(directories[kind], reports[kind], key)
                selected[kind, key] = item
                if item:
                    images.append({"terrain": name, "run": kind, "key": key,
                                   **{k: str(v) if isinstance(v, Path) else v for k, v in item.items()}})
        if selected["probes", "detail"]:
            sheet.append((terrain["title"], selected["probes", "detail"]))
        bounds = terrain["bounds_m"]
        width, length = (bounds[axis][1]-bounds[axis][0] for axis in ("x", "y"))
        s = stats[name]
        provenance = {key: terrain[key] for key in ("generation", "raw_statistics", "adaptation") if key in terrain}
        files = metadata_files(terrain, metadata_path.parent)
        evidence_links = " · ".join(link(path, suite, label) for label, path in files)
        cards += [f'<section class="card" id="{escape(name)}"><div class="eyebrow">{escape(name)}</div><h2>{escape(terrain["title"])}</h2><p>{escape(terrain["description"])}</p>',
            f'<div class="stats"><div><strong>{width:g} × {length:g} m</strong>adapted surface</div><div><strong>{number(s["minimum_m"],1000,1)} to {number(s["maximum_m"],1000,1)} mm</strong>adapted height relative to apron</div>',
            f'<div><strong>{number(terrain.get("corridor_slope_p95_deg"))}° / {number(terrain.get("corridor_slope_max_deg"))}°</strong>corridor grade: P95 / maximum</div><div><strong>{number(s["grid_node_maximum_slope_deg"])}°</strong>whole-top maximum grade</div><div><strong>{number(terrain["grid_spacing_m"],1000,0)} mm</strong>horizontal sample spacing</div></div>',
            '<div class="images">'+camera_figure(selected["probes", "overview"],suite,"Actual Gazebo overview")+camera_figure(selected["probes", "detail"],suite,"Actual Gazebo terrain detail")+'</div>',
            '<h3>Generation and physical adaptation</h3><p class="small">These are recorded source parameters, raw sample statistics, and the operations used to build this physical mesh. Difficulty is the neural model input; measured relief and slope describe the adapted surface.</p>',
            '<details><summary>Exact generation, raw statistics, and adaptation parameters</summary><pre>'+escape(json.dumps(provenance, indent=2))+'</pre></details>',
            '<details><summary>Source arrays, world, and mesh files</summary><p class="links">'+evidence_links+'</p></details>',
            f'<div class="profile">{link(suite/"profiles"/(name+".svg"),suite,"Download profile SVG")}<img loading="lazy" src="profiles/{name}.png" alt="Adapted terrain height sections"><p class="small muted">Profiles exaggerate height: meters horizontally, millimeters vertically. Camera images show the actual simulated geometry. Grades use grid-node differences and exclude closing side walls; whole-top grades include the blends. Corridor: |x| ≤ 0.6 m, −0.8 ≤ y ≤ 2.4 m.</p></div>',
            f'<h3>Primitive contact checks · {status(reports["probes"])}</h3>'+probe_table(reports["probes"]),
            '<p class="links">'+report_links(directories["probes"],suite)+'</p>',
            '<h3>Robot wheel-drive outcomes</h3>'+robot_table(reports)]
        for caster in ("swivel", "ball"):
            report = reports[caster]
            robot = (report or {}).get("robot", {})
            label = "Ordinary swivel" if caster == "swivel" else "Split ball"
            cards += [f'<details><summary>{label}: {status(report)} — image and evidence</summary>',
                camera_figure(selected[caster, "final_detail"],suite,f"Actual Gazebo {label.lower()} after its wheel-drive attempt"),
                '<p class="links">'+report_links(directories[caster],suite)+'</p>']
            if report:
                cards += ['<p class="small">'+escape(robot.get("scope", "See the saved report for measurement scope."))+'</p>']
                if not report.get("passed"):
                    cards += ['<p class="failed">This recorded attempt did not pass every check. Its outcome is retained.</p>',
                        '<pre>'+escape(json.dumps({"error": report.get("error"), "checks": robot.get("checks")}, indent=2))+'</pre>']
            cards += ['</details>']
        archived = sorted({path for path in (suite/name).glob("**/report.json")
                           if path.parent not in directories.values()}
                          | set((suite/"diagnostic_attempts").glob(name+"_*/report.json")))
        if archived:
            cards += ['<details><summary>Archived diagnostic attempts</summary><p class="small">These earlier records are retained for audit and are not substituted for the canonical outcomes above. A diagnosed command-delivery fault is an instrumentation result, not evidence of terrain friction.</p><ul>']
            for path in archived:
                record = json.loads(path.read_text())
                cards += ['<li>'+link(path,suite,str(path.parent.relative_to(suite)))+' · '+status(record)+'</li>']
            cards += ['</ul></details>']
        command = f"cd /home/para/hamr_control\nsource hamr_terrain/scripts/env.sh\nros2 launch hamr_terrain terrain_sim.launch.py terrain:={name} caster:=swivel"
        cards += ['<details><summary>Open this terrain</summary><pre>'+escape(command)+'</pre><p class="small">Use <code>caster:=ball</code> or <code>caster:=none</code> as needed. Driving uses a separate controller.</p></details></section>']
    available = sum(r["available"] for item in records for r in item["reports"].values())
    passed = sum(r["passed"] for item in records for r in item["reports"].values())
    expected = len(metadata)*len(KINDS)
    complete = available == expected and len(images) == len(metadata)*4
    probe_count = sum(len((read_report(suite/t["id"]/"probes") or {}).get("probes", {})) for t in metadata)
    sheet_ready = contact_sheet(sheet, suite/"adtg_contact_sheet.png", difficulty_labels)
    documentation = metadata_path.parent.parent
    doclinks = " · ".join(link(path,suite,label) for path,label in
        ((documentation/"docs/ADTG_TERRAINS.md","Method and reproduction"),
         (documentation/"docs/PHYSICS_ASSUMPTIONS.md","Physical assumptions"),
         (metadata_path,"Terrain metadata"),
         (suite/"validation_audit.json","Runtime validation audit")) if path.exists())
    banner = ("Complete collection" if complete else "Partial collection")+f": {available}/{expected} reports; {passed} passed, {available-passed} failed; {probe_count} probe records"
    navigation = "".join(f'<a href="#{t["id"]}">{escape(t["title"])}</a>' for t in metadata)
    page = f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>HAMR — learned ADTG terrains in Gazebo</title><style>{CSS}</style></head><body><main>
<header><div class="eyebrow">HAMR · Official ADTG project generator · Gazebo Harmonic</div><h1>Learned uneven terrain, adapted for HAMR.</h1><p>{len(metadata)} samples from the official <a href="{UPSTREAM}">adaptive-terrain-diffusion 0.4.2</a> standalone model at difficulty labels {escape(', '.join(map(str,difficulty_labels)))}. The images below come from actual Gazebo cameras.</p><p class="{'passed' if complete and passed==expected else 'pending'}">{banner}</p><nav>{navigation}</nav></header>
<div class="scope"><strong>What was generated</strong><p>The released one-step, difficulty-conditioned consistency model produced the raw heightfields. Recorded physical scaling and boundary transitions adapt them to this small caster robot. This is not the original ADTG policy-adaptive DDPM curriculum or the ADEPT two-layer canopy and learned-control pipeline; no new navigation policy was trained.</p><p>Model labels are inputs, not calibrated HAMR difficulty ratings or a guarantee that every seed and route gets harder in numerical order. Original raw arrays, checkpoint provenance, adapted meshes, and measurements remain linked below.</p></div>
<p class="small">The terrain is static and rigid: no deformable soil, loose gravel, vegetation contact, or calibrated hardware torque/current limits. Surface colors aid visualization and do not assign different materials or friction. Selected-point support probes and short wheel-drive attempts provide bounded contact evidence. A failed wheel-drive check remains a result; collecting every report does not require every robot to traverse every terrain.</p>
<p class="small">DART dynamics use ODE collision detection and the Dantzig solver with a 1 ms timestep. Terrain material targets μ = μ₂ = 1 and zero restitution, matching the mesh attachment defaults. Sliding-box values are approximate horizontal-speed deceleration/g, not precision friction calibration; see each recorded estimator and acceptance tolerance.</p>
<p class="links">{doclinks}</p>{('<p>'+link(suite/'adtg_contact_sheet.png',suite,'View actual Gazebo terrain images together')+'</p>') if sheet_ready else ''}
{generation_overview(suite,metadata,stats)}
{''.join(cards)}<footer class="footer"><p>Only canonical <code>probes</code>, <code>swivel</code> and <code>ball</code> reports are selected. Archived attempts are not substituted. Displayed images require recorded camera origin and a matching SHA-256, regardless of the physical result. Plots are derived geometry, not camera captures.</p></footer></main></body></html>'''
    (suite/"index.html").write_text(page)
    result = {"complete": complete, "complete_means": "All planned reports and final images collected; physical failures may remain.",
        "final_reports_available": available, "final_reports_passed": passed,
        "final_reports_failed": available-passed, "final_reports_expected": expected,
        "probe_count": probe_count, "verified_camera_images": images,
        "contact_sheet_available": sheet_ready, "terrains": records,
        "adapted_grid_statistics": stats, "metadata_path": str(metadata_path),
        "metadata_sha256": hashlib.sha256(metadata_path.read_bytes()).hexdigest()}
    (suite/"gallery_report.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite_directory", type=Path)
    parser.add_argument("--metadata", type=Path)
    parser.add_argument("--require-complete", action="store_true")
    args = parser.parse_args()
    result = build_gallery(args.suite_directory, args.metadata or default_metadata())
    print(json.dumps({key:result[key] for key in ("complete", "final_reports_available", "final_reports_passed", "final_reports_failed", "final_reports_expected", "probe_count")}))
    return 1 if args.require_complete and not result["complete"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
