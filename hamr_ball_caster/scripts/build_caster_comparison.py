#!/usr/bin/env python3
"""Build an offline, paired-video comparison with an explanation for every case."""
import argparse
import csv
from datetime import datetime, timezone
import html
import json
import math
import os
import re
from pathlib import Path


PATH_EXPLANATIONS = {
    'straight_y': 'The initial chassis and ordinary casters already face the straight +Y path. No steering reversal is demanded, so this is a baseline for controller timing and rolling contact.',
    'horizontal_x': 'The vehicle starts facing +Y while the path points +X. The offset-drive controller turns the chassis as it translates; each front contact follows its own turning velocity, not exactly the center-path direction.',
    'reversal_180': 'The reference abruptly reverses along the same line. An ordinary wheel can initially roll backward without lateral scrub, then its trailing swivel flips. Both models still face the same finite rear-wheel acceleration limit.',
    'horizontal_shuttle': 'Repeated reversals leave little distance to settle. At0.40m/s the ordinary run spends most of its motion at drive-command limits, so large body error cannot be attributed solely to the passive casters.',
    'square_360': 'The four main90° corners turn the chassis through approximately a full revolution. Left and right caster locations have different turning velocities; their different headings can be correct rolling geometry.',
    'circle_large_360': 'This0.8m-radius polygonal circle is wider than the0.27114m rear-axle-to-tracked-point offset. A steady chassis heading lag is kinematically possible; the caster headings still need not equal the center-path tangent.',
    'circle_tight_three_laps': 'This0.2m-radius three-lap route is tighter than the0.27114m offset. Even ideal contacts cannot give the chassis a constant heading offset to the path tangent. Changing the support caster does not remove this driven-base constraint.',
    'figure_eight': 'Curvature changes sign at the crossover and varies along both lobes. It exercises repeated left/right steering changes without the exact180° reversal symmetry of the shuttle.',
    'hairpin': 'The0.12m connectors are short relative to the vehicle geometry, and the long legs reverse direction. The center can track a narrow hairpin while contact paths and caster headings differ substantially from its tangent.',
    'sharp_zigzag': 'Short alternating diagonals at0.40m/s demand rapid velocity changes. Rear-wheel speed/acceleration limiting and repeated caster reorientation both matter; compare their measured contributions rather than treating all error as caster failure.',
    'star_reversals': 'The pentagram repeatedly changes direction by large angles. These are strong but non-exact reversals, exciting swivel transients and drive slew limits at each vertex.',
    'tiny_square_four_laps': 'Each side is only0.12m, traversed in0.48s at0.25m/s. The repeated sharp corners are small relative to both the tracked-point offset and passive swivel alignment distance. Drive limiting can round the actual motion even when the reference corners remain sharp.',
    'smooth_s_curve': 'The S shape approximates smoothly varying curvature with short straight chords. It has no exact180° reversal; most visible path-heading discrepancy can be turning geometry. Its timing still has small discontinuities at chord boundaries.',
}

ALIASES = {'tiny_square_slow': 'tiny_square_four_laps', 'tight_circle_slow': 'circle_tight_three_laps',
    'horizontal_shuttle_half_speed': 'horizontal_shuttle', 'horizontal_shuttle_slow': 'horizontal_shuttle',
    'swivel_drag_10x': 'tiny_square_four_laps', 'swivel_coulomb_0p1': 'tiny_square_four_laps',
    'inertia_3x': 'tiny_square_four_laps', 'caster_grip_0p2': 'tiny_square_four_laps',
    'reversal_repeat': 'reversal_180', 'reversal_drag_10x': 'reversal_180', 'reversal_inertia_3x': 'reversal_180'}

THEORY = [
    'An ordinary swivel caster can roll without lateral slip in an ideal kinematic model. It does not need to point along the robot-center path tangent: while the vehicle and fork turn, its own wheel center follows a different velocity.',
    'Instant sharp corners and velocity reversals are not exactly trackable at nonzero speed with finite acceleration. Both caster designs retain the same rear-drive limits. For a straight reversal at0.25m/s, the configured1.6125m/s² drive acceleration needs at least0.31s to reverse velocity; against an instantaneous reversal reference, this alone can create about77.5mm of position lag before reverse speed is reached. This is an ideal limiting example, not a fitted explanation for every path.',
    'Going slower reduces acceleration demands and often contact forces, but does not make swivel orientation instantaneous. For ideal straight pivot travel, caster error obeys dα/dt=−(v/l)sinα. Its small-error time scale is l/v, while alignment distance depends on trail l. With45mm trail, the time scale is0.18s at0.25m/s pivot speed and0.56s at0.08m/s. Slower motion can therefore produce longer angular transients in seconds while improving body tracking.',
    'The split ball has no directly equivalent swivel-facing angle. Its carrier, two hemispheres and two polar rollers provide different passive rolling freedoms. A missing swivel angle is not a measured zero-slip result; compare body tracking, drive limits, vertical motion and the recorded mechanism behavior.',
    'The CAD ball vehicle is about5.04kg heavier, with different estimated bearing losses and rigid mesh contacts. The original controller uses ideal velocity servos without measured motor torque limits. These trials cannot establish motor-current savings, force efficiency or superiority of calibrated hardware.'
]


def prose(text):
    """Space human-readable quantities without altering identifiers or numeric data."""
    text = re.sub(r'(?<=[A-Za-z])(?=\d)', ' ', text)
    text = re.sub(r'(?<=\d)(?=[A-Za-z])', ' ', text)
    return text.replace('toward+Y', 'toward +Y')


def read_json(path):
    try:
        return json.loads(Path(path).read_text())
    except FileNotFoundError:
        return None


def rel(path, suite):
    return os.path.relpath(path, suite)


def finite(value):
    return isinstance(value, (float, int)) and math.isfinite(value)


def tracking_label(pair):
    if not pair:
        return 'Pending'
    status = pair.get('run_status', {})
    if any(not status.get(model, {}).get('completed') or not status.get(model, {}).get('trajectory_passed')
           for model in ('swivel', 'ball')):
        return 'Inspect run checks'
    region = pair['regions'][pair['primary_region']]
    s, b = [region[m]['tracking_error_m'] for m in ('swivel', 'ball')]
    if not all(finite(v) for v in (s['rms'], b['rms'], s['max_abs'], b['max_abs'])):
        return 'Insufficient samples'
    threshold = max(.002, .15*s['rms'])
    peak_threshold = max(.002, .15*s['max_abs'])
    delta, peak_delta = b['rms']-s['rms'], b['max_abs']-s['max_abs']
    if abs(delta) <= threshold:
        return 'Similar RMS tracking'
    if delta < 0:
        return 'Mixed tracking result' if peak_delta > peak_threshold else 'Ball lower tracking error'
    return 'Mixed tracking result' if peak_delta < -peak_threshold else 'Swivel lower tracking error'


def model_data(directory, suite):
    directory = Path(directory)
    trajectory = read_json(directory/'trajectory.json')
    summary = read_json(directory/'run_summary.json')
    names = ['overview.mp4', 'overview_annotated.mp4', 'caster_closeup.mp4', 'trajectory.json',
             'run_summary.json', 'caster_joints.csv', 'ball_stress.json', 'ball_stress.csv',
             'ball_stress.png', 'caster_stress.json', 'caster_stress.csv', 'caster_stress.png',
             'configuration.yaml', 'physical_properties.yaml', 'robot.urdf', 'commands.json',
             'world.sdf', 'video_validation.json', 'gazebo.log', 'trajectory.log']
    names += sorted(p.name for p in directory.iterdir() if p.is_file() and p.name not in names) if directory.is_dir() else []
    files = {n: rel(directory/n, suite) for n in names if (directory/n).is_file()
             and (summary is not None or not n.endswith('.mp4'))}
    data = {'files': files, 'ready': summary is not None,
            'passed': summary.get('passed') if summary else None,
            'failure': summary.get('failure') if summary else None, 'samples': [], 'startup': None,
            'camera_offsets': {}, 'configuration': {}, 'requested': [], 'executed': []}
    if not trajectory:
        return data
    samples = trajectory.get('samples', [])
    motion = [s for s in samples if s['phase'] == 'motion']
    stride = max(1, len(samples)//650)
    chosen = samples[::stride]
    if samples and chosen[-1] is not samples[-1]:
        chosen.append(samples[-1])
    data['samples'] = [[s['position_m'][0], s['position_m'][1], s.get('trajectory_time_s', 0),
                        s['position_error_m'], s['base_rpy_rad'][2]*180/math.pi, s['phase']]
                       for s in chosen]
    data['startup'] = trajectory.get('startup_alignment')
    data['configuration'] = trajectory.get('configuration', {})
    data['requested'] = trajectory.get('requested_route_points_m', data['configuration'].get('points_m', []))
    data['executed'] = trajectory.get('executed_route_points_m', data['requested'])
    first_later = next((s for s in motion if s.get('segment', 0) >= 1), None)
    for camera in ('overview', 'caster_closeup'):
        metadata = read_json(directory/(camera+'_metadata.json'))
        if metadata and motion:
            first = metadata.get('first_sim_time_s')
            if finite(first):
                data['camera_offsets'][camera] = {
                    'motion': max(0., motion[0]['simulation_time_s']-first),
                    'later_legs': max(0., first_later['simulation_time_s']-first) if first_later else None}
    return data


def narrative(case, pair, swivel_stress):
    name, base = case['name'], ALIASES.get(case['name'], case['name'])
    geometry = PATH_EXPLANATIONS.get(base, '')
    if base == 'tiny_square_four_laps':
        geometry = (f"Each side is only 0.12 m, traversed in {0.12/case['speed_m_s']:.2f} s at {case['speed_m_s']:.2f} m/s. "
            'The repeated sharp corners are small relative to both the tracked-point offset and passive swivel alignment distance. '
            'Drive limiting can round the actual motion even when the reference corners remain sharp.')
    if base == 'horizontal_shuttle':
        geometry = ('Repeated reversals leave little distance to settle. The faster 0.40 m/s ordinary baseline spends most of its motion '
            'at drive-command limits. Speed controls separate that demand from caster reorientation; large body error alone does not identify a passive-caster defect.')
    if name.startswith('heading_'):
        geometry = ('The chassis starts at a different yaw on the original simple-waypoint route. '
                    'Assisted cases physically turn toward+Y and perform a forward lead-in; '
                    'the ordinary model additionally waits for both swivel angles. The unassisted270° control can drive backward without turning the chassis180°.')
    if case['category'] == 'speed_control':
        geometry += f" This control uses {case['speed_m_s']:.2f}m/s on exactly the corresponding faster case's geometry."
    if case.get('swivel_xacro_overrides'):
        geometry += ' Ordinary parameter change: '+', '.join(case['swivel_xacro_overrides'])+'. The ball trial keeps nominal physical parameters and is a repeat control, not an equivalent physical intervention.'
    ordinary = 'Ordinary-caster analysis pending.'
    if swivel_stress:
        diagnostic = swivel_stress.get('source_time_alignment_diagnostic', {})
        casters = diagnostic.get('casters') if diagnostic.get('available') else swivel_stress.get('casters')
        phase = case['tracking_comparison_region']
        regions = [casters[side]['regions'][phase] for side in ('left', 'right')]
        longest = max(r['local_wheel']['rolling_axis']['exceeds']['15']['longest_episode_s'] for r in regions)
        path_p95 = [r['commanded_path']['rolling_axis']['p95_abs'] for r in regions]
        local_p95 = [r['local_wheel']['rolling_axis']['p95_abs'] for r in regions]
        ordinary = ('No sampled local wheel-axis error exceeded15° in the compared region. '
                    if longest <= 1e-9 else f'Local wheel-axis error exceeded15° in transient episodes lasting up to {longest:.2f}s. ')
        if all(finite(v) for v in local_p95):
            ordinary += f'Local rolling-axis 95th-percentile errors are {local_p95[0]:.2f}° left / {local_p95[1]:.2f}° right. '
        if all(finite(v) for v in path_p95):
            ordinary += f'Path-tangent axis discrepancies had95th percentiles {path_p95[0]:.1f}° left / {path_p95[1]:.1f}° right. These are different from sideways rolling. '
        ordinary += 'Local direction metrics exclude speeds below0.03m/s. '
        ordinary += ('Joint states are interpolated to odometry source time for this diagnostic.'
                     if diagnostic.get('available') else
                     'This older run lacks independent joint timestamps; the diagnostic uses the originally paired samples.')
    evidence = 'Ball capture and paired analysis pending.'
    ride = ''
    if pair:
        phase = pair['primary_region']
        region = pair['regions'][phase]
        s, b = region['swivel'], region['ball']
        st, bt = s['tracking_error_m'], b['tracking_error_m']
        if all(finite(v) for v in (st['rms'], bt['rms'], st['max_abs'], bt['max_abs'])):
            evidence = (f"In {phase.replace('_', ' ')}, ordinary / ball tracking RMS is {1000*st['rms']:.2f} / {1000*bt['rms']:.2f}mm; "
                        f"sampled peak is {1000*st['max_abs']:.2f} / {1000*bt['max_abs']:.2f}mm. "
                        f"Drive commands are limited for {100*s['either_limited_fraction']:.1f}% / {100*b['either_limited_fraction']:.1f}% of observed time. ")
            evidence += pair['assessment']
            along = [m.get('along_track_error_m', {}).get('rms') for m in (s, b)]
            cross = [m.get('cross_track_error_m', {}).get('rms') for m in (s, b)]
            if all(finite(v) for v in along + cross):
                evidence += (f' Along-route RMS is {1000*along[0]:.2f} / {1000*along[1]:.2f} mm; '
                    f'cross-route RMS is {1000*cross[0]:.2f} / {1000*cross[1]:.2f} mm. '
                    'These use the scheduled segment direction, not nearest-path distance.')
        if finite(s['body_height_range_m']) and finite(b['body_height_range_m']):
            ride = (f"Body-height range is {1000*s['body_height_range_m']:.3f}mm ordinary / {1000*b['body_height_range_m']:.3f}mm ball. "
                    'Ball mesh facets, seams and polar-roller transitions can affect ride motion; pose measurements do not identify individual contact forces.')
    # Keep prose legible without changing case identifiers or source values.
    for number in ('0.03', '15', '95', '90', '180', '270'):
        geometry = geometry.replace('the'+number, 'the '+number).replace('unassisted'+number, 'unassisted '+number)
    return {k: prose(v) for k, v in {'geometry': geometry, 'ordinary': ordinary, 'evidence': evidence, 'ride': ride}.items()}


def collect(suite):
    suite = Path(suite).resolve()
    matrix = json.loads((suite/'comparison_matrix.json').read_text())
    cases = []
    for item in matrix:
        pair = read_json(suite/'pairs'/item['name']/'comparison.json')
        ordinary = read_json(Path(item['swivel_directory'])/'caster_stress.json')
        entry = {**item, 'pair': pair, 'label': tracking_label(pair),
                 'swivel': model_data(item['swivel_directory'], suite),
                 'ball': model_data(item['ball_directory'], suite),
                 'why': narrative(item, pair, ordinary)}
        entry['pair_files'] = {name: rel(suite/'pairs'/item['name']/name, suite)
                               for name in ('comparison.png', 'comparison.json')
                               if (suite/'pairs'/item['name']/name).exists()}
        cases.append(entry)
    return {'generated_utc': datetime.now(timezone.utc).isoformat(), 'cases': cases,
            'recorded_ball_cases': sum(c['ball']['ready'] for c in cases),
            'analyzed_pairs': sum(c['pair'] is not None for c in cases),
            'theory': [prose(t) for t in THEORY],
            'key_findings': (suite/'KEY_FINDINGS.md').read_text().strip().split('\n\n') if (suite/'KEY_FINDINGS.md').exists() else [],
            'files': {name: name for name in ('FINDINGS.md', 'KEY_FINDINGS.md', 'PER_TRAJECTORY.md', 'speed_tradeoffs.png', 'speed_tradeoffs.svg', 'input_and_artifact_audit.json', 'comparison_results.csv', 'repeatability.json', 'comparison_summary.csv', 'paired_index.csv',
                'comparison_matrix.json', 'ball_video_audit.json', 'BALL_COMPARISON_ASSUMPTIONS.md')
                if (suite/name).exists()}}


HTML = r'''<!doctype html><html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>HAMR — ball versus swivel casters</title>
<style>
:root{--ink:#183247;--muted:#5b7182;--blue:#2678b6;--orange:#db8429;--line:#dbe4ea;--bg:#f2f5f7}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:15px/1.55 system-ui,sans-serif}header{padding:26px 32px;background:#132f43;color:white}h1{font-size:29px;margin:0 0 5px}h2{font-size:21px;margin:0 0 12px}h3{font-size:16px;margin:0 0 8px}p{margin:6px 0 15px}.muted{color:var(--muted);font-size:13px}header .muted{color:#c4d6e2}.layout{display:grid;grid-template-columns:255px minmax(0,1fr);gap:22px;max-width:1750px;margin:auto;padding:24px}aside{position:sticky;top:15px;align-self:start;max-height:95vh;overflow:auto}.panel{padding:21px;background:white;border:1px solid var(--line);border-radius:12px;margin-bottom:18px}.grid{display:grid;grid-template-columns:1fr 1fr;gap:20px}.case{display:block;text-align:left;width:100%;padding:10px;background:white;border:1px solid var(--line);border-radius:7px;margin:0 0 6px;cursor:pointer;color:var(--ink)}.case.active{border-color:var(--blue);background:#eaf4fb}.case small{display:block;color:var(--muted);margin-top:3px}.tag{display:inline-block;border-radius:5px;padding:4px 8px;background:#e9f1f7;font-size:13px}.notice{border-left:3px solid var(--orange);padding:10px 15px;background:#fff8ee}.plot{width:100%;display:block;background:#fafcfd;border-radius:7px}video{width:100%;aspect-ratio:16/9;background:#102130;border-radius:8px}.blue{color:var(--blue)}.orange{color:var(--orange)}input,select,button.control{font:inherit;border:1px solid #c5d3dd;border-radius:6px;padding:7px;background:white;color:var(--ink)}aside input,aside select{width:100%;margin-bottom:8px}button.control{cursor:pointer;margin:0 8px 12px 0}table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums;font-size:14px}th,td{text-align:right;border-bottom:1px solid var(--line);padding:8px}th:first-child,td:first-child{text-align:left}.tablewrap{overflow:auto}.links{display:flex;flex-wrap:wrap;gap:8px 18px;font-size:13px}a{color:#246c9c;text-decoration:none}a:hover{text-decoration:underline}details{margin-top:12px}summary{cursor:pointer;color:#246c9c}pre{white-space:pre-wrap;word-break:break-word;font-size:12px;background:#f2f6f8;padding:12px}.metricnote{margin-top:10px;font-size:12px;color:var(--muted)}.counts{display:flex;flex-wrap:wrap;gap:12px;margin:12px 0}.counts b{font-size:20px}.threshold{display:flex;align-items:center;gap:8px;flex-wrap:wrap}#tolerance{width:85px}.rowmodel{border-top:3px solid var(--blue)}.rowmodel.ball{border-color:var(--orange)}@media(max-width:1100px){.layout{grid-template-columns:215px 1fr;padding:15px}.grid{grid-template-columns:1fr}}@media(max-width:700px){.layout{display:block}aside{position:static;max-height:280px;margin-bottom:20px}header{padding:20px}.panel{padding:15px}}
</style>
<header><h1>HAMR · ball versus swivel casters</h1><p>Paired physical simulations, recordings and trajectory-by-trajectory explanations.</p><div id="progress" class="muted"></div></header>
<div class="layout"><aside><input id="search" placeholder="Find a trajectory" aria-label="Search trajectories"><select id="category" aria-label="Category"><option value="all">All cases</option></select><div id="caseList"></div><div class="muted">Tracking labels describe these runs. They are not hardware certifications.</div></aside>
<main><section class="panel"><h2>Does the ball caster improve tracking?</h2><p>Compare the same requested route and drive settings. The ball and ordinary models retain their own estimated masses, bearing losses and contact geometry. Assisted starts use different caster preparation checks.</p><div id="keyFindings"></div><div class="threshold"><label for="tolerance">Explore RMS error tolerance:</label><input id="tolerance" type="number" min="0.1" step="1" value="20"><span>mm · selected for exploration, not a hardware requirement</span></div><div id="toleranceCounts" class="counts"></div><p class="muted">Only the 13 nominal trajectory-shape cases contribute to these counts. Full tables below show every case; sensitivity repeats and assisted starts are identified separately.</p><details><summary>Why slow speed and a perfect environment do not guarantee perfect tracking</summary><div id="theory"></div></details></section>
<section class="panel" id="speedSection" hidden><h2>Measured speed controls</h2><img id="speedChart" class="plot" alt="Tracking RMS and peak error at the tested speeds for three route shapes"></section>
<section class="panel"><h2 id="name"></h2><span id="label" class="tag"></span><p id="subtitle" class="muted"></p><div id="pairNote" class="notice"></div><p id="status"></p><div class="tablewrap"><table><thead><tr><th id="regionLabel">Compared region</th><th class="blue">Ordinary swivel</th><th class="orange">Split ball</th></tr></thead><tbody id="metrics"></tbody></table></div><p class="metricnote">RMS values are duration-weighted; peaks are sampled maxima. Legacy reference/odometry timing contributes to small errors. “Similar” means RMS differs by no more than the larger of 2 mm or 15% of ordinary RMS; it is an exploratory display rule, not a statistical equivalence test.</p></section>
<section class="panel"><h2>Why this trajectory behaves this way</h2><p id="geometry"></p><h3>Ordinary caster</h3><p id="ordinaryWhy"></p><h3>Observed comparison</h3><p id="evidence"></p><p id="ride"></p></section>
<div class="grid"><section class="panel"><h3>Requested and measured route</h3><svg id="routePlot" class="plot" viewBox="0 0 640 440" aria-label="Paired trajectories"></svg><p class="muted">Gray: requested · <span class="blue">blue: ordinary</span> · <span class="orange">orange: ball</span>. Dashed traces show startup preparation.</p></section><section class="panel"><h3>Tracking error during normal motion</h3><svg id="errorPlot" class="plot" viewBox="0 0 640 440" aria-label="Paired tracking error"></svg><p class="muted">Time begins at each model’s route start. Assisted first segments differ; the comparison table uses later legs. Display traces are sampled; metrics use all supported samples.</p></section></div>
<section class="panel"><h2>Vehicle overview recordings</h2><button class="control" id="overviewStart">Play both from start</button><button class="control" id="overviewMotion">Play compared route region</button><div class="grid"><div class="rowmodel"><h3 class="blue">Ordinary swivel</h3><video id="swivelOverview" controls preload="none"></video></div><div class="rowmodel ball"><h3 class="orange">Split ball</h3><video id="ballOverview" controls preload="none"></video></div></div><p class="muted">Normal simulation-time playback. Region alignment uses recorded timestamps; browser playback synchronization is approximate.</p></section>
<section class="panel"><h2>Caster close-ups</h2><button class="control" id="casterStart">Play both from start</button><button class="control" id="casterMotion">Play compared route region</button><div class="grid"><div class="rowmodel"><h3 class="blue">Passive fork and wheel</h3><video id="swivelCaster" controls preload="none"></video></div><div class="rowmodel ball"><h3 class="orange">Passive carrier, hemispheres and rollers</h3><video id="ballCaster" controls preload="none"></video></div></div><p class="muted">Ball carrier angle has no direct equivalence to swivel heading. No zero-slip claim is inferred from the absence of a swivel joint.</p></section>
<section class="panel"><h2>Preparation and detailed plots</h2><div id="startup" class="grid"></div><div id="plotLink"></div><details><summary>Pair checks and measurements</summary><pre id="raw"></pre></details></section>
<section class="panel"><h2>Run data and configuration</h2><h3>Ordinary swivel</h3><div id="swivelFiles" class="links"></div><h3 style="margin-top:15px">Split ball</h3><div id="ballFiles" class="links"></div><h3 style="margin-top:15px">Study exports</h3><div id="suiteFiles" class="links"></div></section>
<section class="panel"><h2>All paired results</h2><div class="tablewrap"><table><thead><tr><th>Case</th><th>Speed (m/s)</th><th>Ordinary RMS (mm)</th><th>Ball RMS (mm)</th><th>Change</th><th>Observed result</th></tr></thead><tbody id="allRows"></tbody></table></div></section>
</main></div><script id="data" type="application/json">__DATA__</script>
<script>
const study=JSON.parse(document.getElementById('data').textContent), cases=study.cases, $=id=>document.getElementById(id);let current=null;
const scale=(v,k=1)=>typeof v==='number'&&Number.isFinite(v)?v*k:null;
const fmt=(v,n=2)=>typeof v==='number'&&Number.isFinite(v)?v.toFixed(n):'—';
function node(tag,text,cls){const e=document.createElement(tag);if(text!==undefined)e.textContent=text;if(cls)e.className=cls;return e;}
function link(text,url){const a=node('a',text);a.href=url;return a;}
function links(id,files){$(id).replaceChildren();Object.entries(files).forEach(([n,u])=>$(id).append(link(n,u)));}
function svgEl(tag,attrs){const e=document.createElementNS('http://www.w3.org/2000/svg',tag);Object.entries(attrs).forEach(([k,v])=>e.setAttribute(k,v));return e;}
function draw(id,series,equal,xlabel,ylabel){const svg=$(id);svg.replaceChildren();const all=series.flatMap(s=>s.points).filter(p=>p.every(Number.isFinite));if(!all.length){svg.append(svgEl('text',{x:40,y:60}));svg.lastChild.textContent='Waiting for measurements';return;}let xmin=Math.min(...all.map(p=>p[0])),xmax=Math.max(...all.map(p=>p[0])),ymin=Math.min(...all.map(p=>p[1])),ymax=Math.max(...all.map(p=>p[1]));let dx=Math.max(.01,xmax-xmin),dy=Math.max(.01,ymax-ymin);if(equal){const d=Math.max(dx,dy*560/355);const cx=(xmin+xmax)/2,cy=(ymin+ymax)/2;dx=d;dy=d*355/560;xmin=cx-dx/2;ymin=cy-dy/2;}xmin-=dx*.07;ymin-=dy*.07;dx*=1.14;dy*=1.14;const X=x=>55+560*(x-xmin)/dx,Y=y=>385-355*(y-ymin)/dy;for(let i=0;i<=4;i++){const x=xmin+dx*i/4,y=ymin+dy*i/4;svg.append(svgEl('line',{x1:X(x),x2:X(x),y1:30,y2:385,stroke:'#e1e7ec'}));svg.append(svgEl('line',{x1:55,x2:615,y1:Y(y),y2:Y(y),stroke:'#e1e7ec'}));for(const [xx,yy,t,anchor]of[[X(x),406,fmt(x,dx<.5?2:1),'middle'],[48,Y(y)+4,fmt(y,dy<.5?2:1),'end']]){const e=svgEl('text',{x:xx,y:yy,'font-size':11,fill:'#607788','text-anchor':anchor});e.textContent=t;svg.append(e);}}series.forEach(s=>{if(s.points.length>1)svg.append(svgEl('polyline',{points:s.points.map(p=>[X(p[0]),Y(p[1])].join(',')).join(' '),fill:'none',stroke:s.color,'stroke-width':s.width||2,'stroke-dasharray':s.dash||''}));});for(const [x,y,text]of[[310,433,xlabel],[55,20,ylabel]]){const e=svgEl('text',{x,y,'font-size':12,fill:'#38566d'});e.textContent=text;svg.append(e);}}
function video(id,url){const e=$(id);e.pause();if(url)e.src=url;else e.removeAttribute('src');e.load();}
function play(kind,region){if(!current)return;['swivel','ball'].forEach(model=>{const id=model+(kind==='overview'?'Overview':'Caster'),e=$(id),key=kind==='overview'?'overview':'caster_closeup';if(!e.getAttribute('src'))return;const at=region?(current[model].camera_offsets[key]?.[current.tracking_comparison_region]||0):0;const start=()=>{e.currentTime=at;e.play().catch(()=>{});};if(e.readyState)start();else{e.addEventListener('loadedmetadata',start,{once:true});e.load();}});}
function setCase(name){const c=cases.find(c=>c.name===name);if(!c)return;current=c;location.hash=encodeURIComponent(name);$('name').textContent=c.name.replaceAll('_',' ');$('label').textContent=c.label;$('subtitle').textContent=c.category+' · reference '+fmt(c.speed_m_s)+' m/s · initial chassis yaw '+fmt(c.initial_yaw_deg,0)+'°';$('pairNote').textContent=c.comparison_note+' '+c.physical_note;$('status').textContent='Run checks: ordinary '+(c.swivel.ready?(c.swivel.passed?'passed':'failed'):'pending')+' / ball '+(c.ball.ready?(c.ball.passed?'passed':'failed'):'pending')+'. These checks do not certify caster alignment.';for(const [id,key]of[['geometry','geometry'],['ordinaryWhy','ordinary'],['evidence','evidence'],['ride','ride']])$(id).textContent=c.why[key];$('metrics').replaceChildren();const p=c.pair?.regions[c.tracking_comparison_region];$('regionLabel').textContent='Compared: '+c.tracking_comparison_region.replaceAll('_',' ');const metrics=[['Tracking RMS (mm)',m=>scale(m.tracking_error_m.rms,1000)],['Along-route RMS (mm)',m=>scale(m.along_track_error_m?.rms,1000)],['Cross-route RMS (mm)',m=>scale(m.cross_track_error_m?.rms,1000)],['Sampled peak error (mm)',m=>scale(m.tracking_error_m.max_abs,1000)],['Drive limited (% time)',m=>scale(m.either_limited_fraction,100)],['Body height range (mm)',m=>scale(m.body_height_range_m,1000)],['Maximum absolute roll (deg)',m=>m.body_roll_deg.max_abs],['Maximum absolute pitch (deg)',m=>m.body_pitch_deg.max_abs],['Net chassis yaw change (deg)',m=>m.body_heading.net_change_deg]];metrics.forEach(([label,get])=>{const tr=node('tr');tr.append(node('td',label));for(const model of['swivel','ball'])tr.append(node('td',p?fmt(get(p[model])):'—'));$('metrics').append(tr);});const route=[{points:c.points_m,color:'#aab5bd',dash:'7 5'}],error=[];for(const [model,color]of[['swivel','#2678b6'],['ball','#db8429']]){const all=c[model].samples,motion=all.filter(s=>s[5]==='motion'),prep=all.filter(s=>s[5].startsWith('startup'));route.push({points:motion.map(s=>s.slice(0,2)),color});route.push({points:prep.map(s=>s.slice(0,2)),color,dash:'4 4',width:1.3});error.push({points:motion.map(s=>[s[2],s[3]*1000]),color});video(model+'Overview',c[model].files['overview_annotated.mp4']||c[model].files['overview.mp4']);video(model+'Caster',c[model].files['caster_closeup.mp4']);links(model+'Files',c[model].files);}draw('routePlot',route,true,'World X (m)','World Y (m)');draw('errorPlot',error,false,'Route time (s)','Position error (mm)');$('startup').replaceChildren();for(const model of['swivel','ball']){const div=node('div');div.append(node('h3',model==='swivel'?'Ordinary swivel':'Split ball'));const s=c[model].startup;div.append(node('p',s?('Preparation '+fmt(s.duration_s)+' s; actual chassis turn '+fmt(scale(s.actual_chassis_yaw_change_rad,180/Math.PI))+'°; forward lead-in '+fmt(s.lead_in_distance_m,3)+' m.'): 'No assisted preparation.'));if(s)div.append(node('p',model==='ball'?'Ball has no swivel-facing gate.':'Both passive swivel angles must satisfy the release tolerance.','muted'));$('startup').append(div);}$('plotLink').replaceChildren();if(c.pair_files['comparison.png']){const a=link('',c.pair_files['comparison.png']),img=node('img');img.src=c.pair_files['comparison.png'];img.className='plot';img.alt='Full paired trajectory, tracking, heading and height plots';a.append(img);$('plotLink').append(a);}$('raw').textContent=JSON.stringify(c.pair||{status:'analysis pending'},null,2);renderList();}
function renderList(){const q=$('search').value.toLowerCase(),category=$('category').value;$('caseList').replaceChildren();cases.filter(c=>c.name.replaceAll('_',' ').includes(q.replaceAll('_',' '))&&(category==='all'||c.category===category)).forEach(c=>{const b=node('button',c.name.replaceAll('_',' '),'case'+(current?.name===c.name?' active':''));b.append(node('small',c.label));b.onclick=()=>setCase(c.name);$('caseList').append(b);});}
function tolerance(){const limit=Number($('tolerance').value)/1000;const counts={both:0,ballOnly:0,swivelOnly:0,neither:0,pending:0};cases.filter(c=>c.category==='trajectory').forEach(c=>{const p=c.pair?.regions.motion;if(!p||c.label==='Inspect run checks'){counts.pending++;return;}const s=p.swivel.tracking_error_m.rms,b=p.ball.tracking_error_m.rms;if(typeof s!=='number'||typeof b!=='number'){counts.pending++;return;}counts[s<=limit?(b<=limit?'both':'swivelOnly'):(b<=limit?'ballOnly':'neither')]++;});$('toleranceCounts').replaceChildren();for(const[k,l]of[['both','Both within tolerance'],['ballOnly','Only ball within'],['swivelOnly','Only swivel within'],['neither','Neither within'],['pending','Pending / check run']]){const e=node('span');e.append(node('b',counts[k]+' '),node('span',l));$('toleranceCounts').append(e);}}
const cats=[...new Set(cases.map(c=>c.category))];cats.forEach(c=>{const o=node('option',c.replaceAll('_',' '));o.value=c;$('category').append(o);});$('progress').textContent=study.recorded_ball_cases+'/'+cases.length+' ball recordings · '+study.analyzed_pairs+' analyzed pairs · generated '+new Date(study.generated_utc).toLocaleString();study.theory.forEach(t=>$('theory').append(node('p',t)));study.key_findings.forEach(t=>$('keyFindings').append(node('p',t)));links('suiteFiles',study.files);if(study.files['speed_tradeoffs.png']){$('speedChart').src=study.files['speed_tradeoffs.png'];$('speedSection').hidden=false;}cases.forEach(c=>{const tr=node('tr'),first=node('td'),a=link(c.name,'#'+encodeURIComponent(c.name));a.onclick=e=>{e.preventDefault();setCase(c.name);};first.append(a);tr.append(first,node('td',fmt(c.speed_m_s)));const p=c.pair?.regions[c.tracking_comparison_region];tr.append(node('td',p?fmt(scale(p.swivel.tracking_error_m.rms,1000)):'—'),node('td',p?fmt(scale(p.ball.tracking_error_m.rms,1000)):'—'),node('td',p?fmt(p.tracking_rms_change_percent,1)+'%':'—'),node('td',c.label));$('allRows').append(tr);});$('search').oninput=renderList;$('category').onchange=renderList;$('tolerance').oninput=tolerance;$('overviewStart').onclick=()=>play('overview',false);$('overviewMotion').onclick=()=>play('overview',true);$('casterStart').onclick=()=>play('caster',false);$('casterMotion').onclick=()=>play('caster',true);window.onhashchange=()=>{const n=decodeURIComponent(location.hash.slice(1));if(n!==current?.name)setCase(n);};tolerance();renderList();const initial=decodeURIComponent(location.hash.slice(1));if(cases.length)setCase(cases.some(c=>c.name===initial)?initial:cases[0].name);
</script></html>'''


def build_report(suite):
    suite = Path(suite).resolve()
    report = collect(suite)
    rows = []
    for case in report['cases']:
        row = {'name': case['name'], 'category': case['category'], 'speed_m_s': case['speed_m_s'],
               'initial_yaw_deg': case['initial_yaw_deg'], 'region': case['tracking_comparison_region'],
               'ball_role': case['ball_role'], 'observed_tracking_label': case['label'],
               'swivel_passed': case['swivel']['passed'], 'ball_passed': case['ball']['passed']}
        if case['pair']:
            region = case['pair']['regions'][case['tracking_comparison_region']]
            for model in ('swivel', 'ball'):
                value = region[model]
                row.update({f'{model}_rms_m': value['tracking_error_m']['rms'],
                            f'{model}_peak_m': value['tracking_error_m']['max_abs'],
                            f'{model}_limited_fraction': value['either_limited_fraction'],
                            f'{model}_height_range_m': value['body_height_range_m']})
        rows.append(row)
    with (suite/'comparison_summary.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(k for r in rows for k in r)))
        writer.writeheader()
        writer.writerows(rows)
    report['files']['comparison_summary.csv'] = 'comparison_summary.csv'
    (suite/'comparison_summary.json').write_text(json.dumps({**report, 'cases': [{
        k: v for k, v in c.items() if k not in ('swivel', 'ball')} for c in report['cases']]}, indent=2, allow_nan=False)+'\n')
    payload = json.dumps(report, separators=(',', ':'), allow_nan=False).replace('<','\\u003c').replace('>','\\u003e').replace('&','\\u0026')
    (suite/'index.html').write_text(HTML.replace('__DATA__', payload))
    (suite/'PER_TRAJECTORY.md').write_text('# Paired trajectory explanations\n\n'+
        f"{report['analyzed_pairs']} / {len(report['cases'])} pairs analyzed.\n\n"+
        '\n\n'.join('## '+c['name']+'\n\n'+c['label']+'\n\n'+
                    '\n\n'.join(v for v in c['why'].values() if v) for c in report['cases'])+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('suite', type=Path)
    args = parser.parse_args()
    report = build_report(args.suite)
    print(json.dumps({'recordings': report['recorded_ball_cases'], 'pairs': report['analyzed_pairs'],
                      'gallery': str(args.suite.resolve()/'index.html')}))


if __name__ == '__main__':
    main()
