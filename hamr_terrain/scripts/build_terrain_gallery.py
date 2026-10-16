#!/usr/bin/env python3
"""Build a local gallery from final terrain reports and actual Gazebo images.

Only <terrain>/probes, <terrain>/ball and <terrain>/swivel are selected. Archived
attempts are never substituted. A partial build stays explicitly incomplete;
--require-complete returns an error unless all fifteen final reports pass.
"""
import argparse
import hashlib
from html import escape
import json
import os
from pathlib import Path
from urllib.parse import quote

import numpy as np


PROFILE_X = {"road_bumps": (0.,), "gentle_hill": (0.,),
             "cross_slope": (-.7, 0., .7), "rutted_track": (0., .22),
             "rock_garden": (0., -.10, .32)}
CROSS_Y = {"road_bumps": 1.75, "gentle_hill": .7, "cross_slope": .5,
           "rutted_track": .9, "rock_garden": 1.25}


def default_metadata():
    source = Path(__file__).resolve().parents[1]/"config/terrains.json"
    if source.is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("hamr_terrain"))/"config/terrains.json"


def href(path, root):
    return quote(os.path.relpath(Path(path), Path(root)), safe="/._-")


def link(path, root, label):
    return f'<a href="{href(path, root)}">{escape(label)}</a>'


def number(value, scale=1., digits=2):
    return "—" if value is None else f"{value*scale:.{digits}f}"


def read_report(directory):
    path = directory/"report.json"
    return json.loads(path.read_text()) if path.exists() else None


def checked_image(directory, report, key):
    """Never present a failed/archived run or a changed image as final evidence."""
    if not report or not report.get("passed"):
        return None
    entry = report.get("images", {}).get(key)
    if not entry or entry.get("source") != "Actual Gazebo camera sensor":
        return None
    path = (directory/entry["file"]).resolve()
    if not path.is_relative_to(directory.resolve()) or not path.is_file():
        return None
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if entry.get("sha256") != digest:
        return None
    return {"path": path, "sha256": digest, "source_stamp_s": entry.get("source_stamp_s")}


def triangle_height(x, y, z, xx, yy):
    """Interpolate the saved a-b-c / a-c-d mesh, including its diagonal."""
    ix = np.clip(np.searchsorted(x, xx, side="right")-1, 0, len(x)-2)
    iy = np.clip(np.searchsorted(y, yy, side="right")-1, 0, len(y)-2)
    u, v = (xx-x[ix])/(x[ix+1]-x[ix]), (yy-y[iy])/(y[iy+1]-y[iy])
    a, b, c, d = z[iy,ix], z[iy,ix+1], z[iy+1,ix+1], z[iy+1,ix]
    return np.where(u >= v, a+(b-a)*u+(c-b)*v, a+(c-d)*u+(d-a)*v)


def profiles(metadata, config_directory, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    results = {}
    for terrain in metadata:
        name = terrain["id"]
        with np.load((config_directory/terrain["height_grid"]).resolve()) as grid:
            x, y, z = (grid[k] for k in ("x", "y", "z"))
        dy, dx = np.gradient(z, y, x)
        slope = np.degrees(np.arctan(np.hypot(dx, dy)))
        center = int(np.argmin(abs(x)))
        results[name] = {"corridor_slope_deg": float(np.max(slope[:, abs(x) <= .6])),
                         "centerline_longitudinal_slope_deg": float(np.max(np.degrees(np.arctan(abs(dy[:, center])))))}
        fig, axes = plt.subplots(1, 2, figsize=(11, 2.65), gridspec_kw={"width_ratios": [1.6, 1.]}, layout="constrained")
        longitudinal = np.linspace(y[0], y[-1], 8*(len(y)-1)+1)
        for offset in PROFILE_X.get(name, (0.,)):
            along = triangle_height(x, y, z, np.full_like(longitudinal, offset), longitudinal)
            axes[0].plot(longitudinal, along*1000, linewidth=1.7, label=f"x = {offset:+.2f} m")
        cross_y = CROSS_Y.get(name, .7)
        lateral = np.linspace(x[0], x[-1], 8*(len(x)-1)+1)
        across = triangle_height(x, y, z, lateral, np.full_like(lateral, cross_y))
        axes[1].plot(lateral, across*1000, color="#7451a8", linewidth=1.7)
        axes[0].set(xlabel="Longitudinal Y (m)", ylabel="Height Z (mm)", title="Travel-direction sections")
        axes[1].set(xlabel="Lateral X (m)", title=f"Cross section at y = {cross_y:.2f} m")
        axes[0].legend(fontsize=8, loc="upper right")
        for ax in axes:
            ax.axhline(0., color="#87919d", linewidth=.65)
            ax.grid(alpha=.18)
            ax.spines[["top", "right"]].set_visible(False)
            ax.tick_params(labelsize=8)
        fig.suptitle(f"{terrain['title']} · height scale exaggerated", fontsize=11)
        for extension in ("png", "svg"):
            fig.savefig(output/f"{name}.{extension}", dpi=160)
        plt.close(fig)
    return results


def report_links(directory, root):
    labels = (("report.json", "Checks"), ("measurements.json", "Raw measurements"),
              ("commands.json", "Run configuration"), ("metadata.json", "Terrain parameters"),
              ("world.sdf", "World snapshot"),
              ("gazebo.log", "Gazebo log"), ("bridge.log", "Bridge log"))
    return " · ".join(link(directory/name, root, label) for name, label in labels if (directory/name).exists())


def status(report):
    if report is None:
        return '<span class="pending">Pending</span>'
    return '<span class="passed">Passed</span>' if report.get("passed") else '<span class="failed">Check failed</span>'


def figure(image, root, caption):
    if image is None:
        return f'<div class="placeholder">{escape(caption)}<br>Awaiting a verified image from a passed final run.</div>'
    return (f'<figure><a href="{href(image["path"], root)}"><img loading="lazy" src="{href(image["path"], root)}" alt="{escape(caption)}"></a>'
            f'<figcaption>{escape(caption)} · simulation time {number(image["source_stamp_s"])} s</figcaption></figure>')


def probe_table(report):
    if not report:
        return "<p>Final contact measurements are pending.</p>"
    rows = []
    for name, probe in report.get("probes", {}).items():
        spec = probe["specification"]
        kind = spec["kind"]
        if kind == "guided_drop":
            result = f"Support error {number(probe.get('support_height_error_m'),1000,3)} mm"
        elif kind == "sliding_box":
            result = f"Deceleration/g {number(probe.get('measured_sliding_mu'),digits=4)}; nominal μ {number(spec.get('expected_combined_mu'),digits=2)}"
        else:
            result = f"Free displacement {number(probe.get('horizontal_displacement_m'),1000,1)} mm"
        xy = spec.get("xy_m", [None, None])
        rows.append(f'<tr><td>{escape(name)}</td><td>({number(xy[0])}, {number(xy[1])})</td><td>{result}</td>'
                    f'<td>{number(probe.get("maximum_contact_depth_m"),1000,3)}</td><td>{status(probe)}</td></tr>')
    error = f'<p class="failed">{escape(report["error"])}</p>' if report.get("error") else ""
    return error+'<div class="table-scroll"><table><thead><tr><th>Physical probe</th><th>Initial XY (m)</th><th>Measured result</th><th>Peak depth (mm)</th><th>Check</th></tr></thead><tbody>'+"".join(rows)+"</tbody></table></div>"


def robot_table(records):
    rows = []
    for caster in ("swivel", "ball"):
        report = records[caster]
        robot = (report or {}).get("robot", {})
        displacement = robot.get("displacement_m", [None, None, None])
        tilt = robot.get("maximum_abs_roll_pitch_deg", [None, None])
        rows.append(f'<tr><td>{"Ordinary swivel" if caster == "swivel" else "Split ball"}</td><td>{status(report)}</td>'
                    f'<td>{number(displacement[1],digits=3)}</td><td>{number(tilt[0])} / {number(tilt[1])}</td>'
                    f'<td>{number(robot.get("maximum_contact_depth_m"),1000,3)}</td><td>{robot.get("contact_messages", "—")}</td></tr>')
    return '<div class="table-scroll"><table><thead><tr><th>Robot</th><th>Run</th><th>Forward ΔY (m)</th><th>Max roll / pitch (°)</th><th>Peak depth (mm)</th><th>Contact messages</th></tr></thead><tbody>'+"".join(rows)+"</tbody></table></div>"


def contact_sheet(entries, output):
    """A labeled figure containing only verified, unaltered camera captures."""
    if len(entries) != 5:
        return False
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 2, figsize=(13, 12.5), layout="constrained")
    for ax, (title, image) in zip(axes.flat, entries):
        ax.imshow(plt.imread(image["path"]))
        ax.set_title(title, fontsize=12, loc="left")
        ax.set_axis_off()
    axes.flat[-1].set_axis_off()
    axes.flat[-1].text(.03, .9, "Actual Gazebo camera captures\n\n5 rigid terrain maps\n6 × 8 m generated surface each\n\nPhysical heights are unchanged.\nProfile plots separately exaggerate\nheight scale for readability.", transform=axes.flat[-1].transAxes, va="top", fontsize=14, linespacing=1.6)
    fig.savefig(output, dpi=150, facecolor="white")
    plt.close(fig)
    return True


CSS = """
:root{color-scheme:light;--ink:#1b2a36;--muted:#586977;--line:#dfe5e8;--green:#176e53}
*{box-sizing:border-box}body{margin:0;background:#f4f6f6;color:var(--ink);font:16px/1.6 system-ui,sans-serif}
main{max-width:1240px;margin:auto;padding:35px 24px 70px}header{max-width:960px;margin:0 0 26px}
h1{font-size:clamp(30px,4vw,44px);line-height:1.14;letter-spacing:-.025em;margin:12px 0 20px}
h2{font-size:28px;margin:0 0 7px}h3{font-size:19px;margin:24px 0 10px}p{margin:10px 0}
a{color:#16668a;text-decoration-thickness:1px;text-underline-offset:3px}nav{display:flex;flex-wrap:wrap;gap:9px;margin:22px 0}
nav a{padding:7px 14px;border:1px solid #cbd7dc;border-radius:30px;text-decoration:none;background:white}
.eyebrow,.muted,figcaption{color:var(--muted)}.eyebrow{font-size:13px;letter-spacing:.08em;text-transform:uppercase}
.scope{background:#e8efec;border-left:4px solid #40846b;padding:16px 20px;margin:22px 0}
.card{background:white;border:1px solid var(--line);border-radius:14px;margin:30px 0;padding:26px;scroll-margin-top:18px}
.stats{display:flex;flex-wrap:wrap;gap:8px 25px;color:var(--muted);margin:15px 0}.stats strong{display:block;color:var(--ink)}
.images{display:grid;grid-template-columns:1fr 1fr;gap:16px;margin:18px 0}figure{margin:0}img{width:100%;height:auto;display:block;border-radius:7px;background:#eef1f2}
figcaption{font-size:13px;padding-top:5px}.placeholder{background:#eef1f2;border:1px dashed #b6c4cc;padding:35px 20px;color:var(--muted)}
.profile{margin:20px 0}.profile img{border:1px solid #e8edf0}.table-scroll{overflow:auto}table{border-collapse:collapse;width:100%;font-size:14px}
th,td{text-align:left;padding:9px 12px;border-bottom:1px solid var(--line);white-space:nowrap}th{background:#f4f6f7;font-weight:600}
.passed{color:var(--green);font-weight:650}.failed{color:#a82c33;font-weight:650}.pending{color:#8a5e13;font-weight:650}
details{margin:17px 0}summary{cursor:pointer;font-weight:600}pre{overflow:auto;background:#182a35;color:#eaf1f5;padding:14px 17px;border-radius:7px;font-size:13px}
.links{font-size:14px;margin:10px 0}.small{font-size:14px}.footer{border-top:1px solid #cbd7dc;padding-top:20px;color:var(--muted)}
@media(max-width:760px){main{padding:24px 12px}.card{padding:18px}.images{grid-template-columns:1fr}h2{font-size:24px}}
"""


def build_gallery(suite, metadata_path):
    suite, metadata_path = Path(suite).expanduser().resolve(), Path(metadata_path).expanduser().resolve()
    suite.mkdir(parents=True, exist_ok=True)
    metadata = json.loads(metadata_path.read_text())
    grade = profiles(metadata, metadata_path.parent, suite/"profiles")
    records, images, cards, sheet = [], [], [], []
    for terrain in metadata:
        name = terrain["id"]
        directories = {kind: suite/name/kind for kind in ("probes", "swivel", "ball")}
        reports = {kind: read_report(path) for kind, path in directories.items()}
        records.append({"id": name, "reports": {kind: {"available": report is not None, "passed": bool(report and report.get("passed"))} for kind, report in reports.items()}})
        terrain_images = {}
        for key in ("overview", "detail"):
            terrain_images[key] = checked_image(directories["probes"], reports["probes"], key)
            if terrain_images[key]:
                images.append({"terrain": name, "key": key, **{k: str(v) if isinstance(v, Path) else v for k, v in terrain_images[key].items()}})
        if terrain_images["detail"]:
            sheet.append((terrain["title"], terrain_images["detail"]))
        bounds = terrain["bounds_m"]
        width, length = (bounds[axis][1]-bounds[axis][0] for axis in ("x", "y"))
        cards += [f'<section class="card" id="{escape(name)}"><div class="eyebrow">{escape(name)}</div><h2>{escape(terrain["title"])}</h2><p>{escape(terrain["description"])}</p>',
                  f'<div class="stats"><div><strong>{width:g} × {length:g} m</strong>generated patch</div><div><strong>{number(terrain["min_height_m"],1000,1)} to {number(terrain["max_height_m"],1000,1)} mm</strong>top height relative to apron</div>',
                  f'<div><strong>{number(grade[name]["corridor_slope_deg"])}° / {number(terrain["max_surface_slope_deg"])}°</strong>maximum grade: corridor / whole top</div><div><strong>{number(terrain["grid_spacing_m"],1000,0)} mm</strong>surface grid</div></div>',
                  '<div class="images">'+figure(terrain_images["overview"],suite,"Actual Gazebo overview")+figure(terrain_images["detail"],suite,"Actual Gazebo terrain detail")+'</div>',
                  f'<div class="profile">{link(suite/"profiles"/(name+".svg"),suite,"Download profile SVG")}<img loading="lazy" src="profiles/{name}.png" alt="Height sections of the generated {escape(terrain["title"])}"><p class="small muted">Profiles use meters horizontally and millimeters vertically; height is exaggerated for readability. Images above use the original physical geometry. Corridor means |x| ≤ 0.6 m; grades are grid-node estimates and exclude closing side walls.</p></div>',
                  f'<h3>Physical probes · {status(reports["probes"])}</h3>'+probe_table(reports["probes"]),
                  '<p class="links">'+report_links(directories["probes"],suite)+'</p>',
                  '<h3>Robot contact smoke checks</h3>'+robot_table(reports)]
        for caster in ("swivel", "ball"):
            cards += [f'<details><summary>{"Ordinary swivel" if caster == "swivel" else "Split-ball"} run: images and evidence</summary>',
                      '<p class="links">'+report_links(directories[caster],suite)+'</p>']
            final = checked_image(directories[caster], reports[caster], "final_detail")
            if final:
                cards += [figure(final,suite,"Actual Gazebo robot after the measured wheel-drive check")]
            if reports[caster]:
                check = reports[caster].get("robot", {})
                cards += ['<p class="small">'+escape(check.get("scope", "See the saved run report for measurement scope."))+'</p>']
                names = check.get("contact_collision_names", [])
                if names:
                    cards += ['<details><summary>Observed collision names</summary><pre>'+escape("\n".join(names))+'</pre></details>']
            cards += ['</details>']
        command = f"cd /home/para/hamr_control\nsource hamr_terrain/scripts/env.sh\nros2 launch hamr_terrain terrain_sim.launch.py terrain:={name} caster:=swivel"
        cards += ['<details><summary>Open this map in Gazebo</summary><pre>'+escape(command)+'</pre><p class="small">Use <code>caster:=ball</code> for split-ball casters or <code>caster:=none</code> for terrain only. The launch opens the world; a drive or trajectory controller is a separate command.</p></details></section>']
    passed = sum(report["passed"] for record in records for report in record["reports"].values())
    expected = 3*len(metadata)
    complete = passed == expected and len(images) == 2*len(metadata)
    probe_count = sum(len((read_report(suite/t["id"]/"probes") or {}).get("probes", {})) for t in metadata)
    sheet_ready = contact_sheet(sheet, suite/"terrain_contact_sheet.png")
    documentation = metadata_path.parent.parent
    docs_links = " · ".join(link(path, suite, label) for path, label in
                            ((documentation/"README.md", "Setup and usage"),
                             (documentation/"docs/PHYSICS_ASSUMPTIONS.md", "Physical assumptions")) if path.exists())
    banner = f'{passed}/{expected} final runs passed · {probe_count} measured probes' if complete else f'Partial validation: {passed}/{expected} final runs passed; pending or failed checks remain visible'
    navigation = "".join(f'<a href="#{t["id"]}">{escape(t["title"])}</a>' for t in metadata)
    page = f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>HAMR terrain maps — actual Gazebo images and contact checks</title><style>{CSS}</style></head><body><main>
<header><div class="eyebrow">HAMR · Gazebo Harmonic</div><h1>Five terrains, with physical contact checks.</h1><p>Bumps, gentle elevation, a banked lane, ruts and fixed rocks. Both caster vehicles use the same generated terrain surface.</p><p class="{'passed' if complete else 'pending'}">{banner}</p><nav>{navigation}</nav></header>
<div class="scope"><strong>What these results establish</strong><p>Actual Gazebo camera images show the terrain. Guided sphere drops test support at selected points; sliding boxes test effective friction on the flat approach; a freely rolling sphere tests unconstrained motion. Separate short rear-wheel drive runs exercise each robot on part of each map. A passed smoke check is bounded evidence of those runs, not validation of every feature or an entire trajectory.</p><p>The surface is static and rigid. There is no deformable soil, loose gravel, tire compliance or calibrated motor torque/current limit. These maps are test environments, not a hardware climbing or off-road capability claim.</p></div>
<p class="small">DART dynamics · ODE collision detection · Dantzig solver · 1 ms physics timestep. Terrain material targets μ = μ₂ = 1 and zero restitution, matching the DART mesh-attachment defaults; the measured sliding-box results below test that response. The previous caster comparison used a different floor coefficient (0.8).</p>
<p class="small">Sliding-box values are approximate post-pulse deceleration divided by gravity, not precision material measurements. Box rotation and discrete contact dynamics remain in the observations. The configured acceptance tolerance is a broad ±0.18 around the nominal combined coefficient; read the exact values and saved fit method, not only the pass label.</p>
<p class="links">{docs_links}</p>
{('<p>'+link(suite/'terrain_contact_sheet.png',suite,'Open the five actual terrain images together')+'</p>') if sheet_ready else ''}
{''.join(cards)}
<footer class="footer"><p>Only the final <code>probes</code>, <code>swivel</code> and <code>ball</code> directories are selected. Archived attempts are retained in the artifact directory but are never substituted into this gallery. Displayed camera images must match their successful report’s recorded SHA-256.</p><p>The terrain’s serialized material fields alone do not prove that the mesh loader applies them. See the <a href="https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/physics/Physics.cc">Gazebo mesh attachment path</a> and <a href="https://github.com/dartsim/dart/blob/v6.13.2/dart/constraint/ContactSurface.cpp">DART contact coefficient combination</a>. Common terrain does not make the two robot models’ masses, bearing losses or effective tire friction equal.</p></footer>
</main></body></html>'''
    (suite/"index.html").write_text(page)
    result = {"complete": complete, "final_reports_expected": expected, "final_reports_passed": passed,
              "probe_count": probe_count, "verified_camera_images": images,
              "contact_sheet_available": sheet_ready, "terrains": records,
              "metadata_path": str(metadata_path), "metadata_sha256": hashlib.sha256(metadata_path.read_bytes()).hexdigest()}
    (suite/"gallery_report.json").write_text(json.dumps(result,indent=2,allow_nan=False)+"\n")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("suite_directory", type=Path)
    parser.add_argument("--metadata", type=Path, default=None)
    parser.add_argument("--require-complete", action="store_true")
    args = parser.parse_args()
    result = build_gallery(args.suite_directory, args.metadata or default_metadata())
    print(json.dumps({key: result[key] for key in ("complete", "final_reports_passed", "final_reports_expected", "probe_count", "contact_sheet_available")}))
    return 1 if args.require_complete and not result["complete"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
