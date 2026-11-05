"""Render per-recording CENTER/H1/H2 comparisons from reconstructed histories.

This file only reads numerical reconstruction outputs. It does not fit, alter,
or feed encoder measurements back into the predictions.
"""
from pathlib import Path
from _paths import DATA_ROOT, OUTPUT_ROOT, reconstruction_source, prepared_inputs, relocated
import argparse
import html
import json
import shutil
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

WHEELS = ('front_left', 'front_right')
CHANNELS = ('CENTER', 'H1', 'H2')
COLORS = {'measured': '#be652e', 'predicted': '#146e9a', 'coast': '#7d8490'}
SELECTED = 'encoder_20261030T005046Z_7a2266b7'


def pick(a, names, required=True):
    for name in names:
        if name in a:
            return np.asarray(a[name])
    if required:
        raise KeyError(f'Missing one of {names}')
    return None


def channel_arrays(a, wheel, channel):
    k = wheel + '__'
    if k + 'primary_change_rad' in a:
        index = CHANNELS.index(channel)
        obs = a[k + 'observed_change_rad'][:, index]
        pred = a[k + 'primary_change_rad'][:, index]
        coast = a[k + 'coast_change_rad'][:, index] if channel != 'CENTER' else None
        return obs, pred, coast
    if channel == 'CENTER':
        obs = pick(a, [k + 'delta_enc_rad'])
        pred = pick(a, [k + 'delta_pred_rad'])
        return obs - obs[0], pred - obs[0], None
    h = channel.lower()
    obs = pick(a, [k + f'beta_{h}_change_rad', k + f'beta_{h}_enc_rad'])
    pred = pick(a, [k + f'beta_{h}_pred_rad', k + f'beta_{h}_hold_rad'])
    coast = pick(a, [k + f'beta_{h}_coast_rad'], required=False)
    return obs - obs[0], pred, coast


def load_histories(directory):
    grouped = {}
    for path in sorted(directory.glob('*.npz')):
        # Case identifiers are stable and do not contain a double underscore.
        case = path.stem.split('__')[0]
        with np.load(path) as z:
            a = {key: z[key].copy() for key in z.files}
        t = pick(a, ['case_elapsed_s', 'elapsed_s'])
        grouped.setdefault(case, []).append({'path': path, 'arrays': a, 'time': t})
    for segments in grouped.values():
        segments.sort(key=lambda s: float(s['time'][0]))
    return grouped


def shade_boolean(ax, t, mask, color='#9aa4b0', alpha=.13):
    if len(t) < 2:
        return
    mask = np.asarray(mask, dtype=bool)
    if len(mask) == len(t) - 1:
        mask = np.r_[mask, mask[-1]]
    if len(mask) != len(t):
        return
    starts = np.flatnonzero(mask & np.r_[True, ~mask[:-1]])
    stops = np.flatnonzero(mask & np.r_[~mask[1:], True])
    for left, right in zip(starts, stops):
        end = t[min(right + 1, len(t) - 1)]
        ax.axvspan(t[left], end, color=color, alpha=alpha, lw=0, zorder=-4)


def nominal_mode(a, wheel):
    k = wheel + '__'
    return pick(a, [k + 'nominal_mode', k + 'mode_pred', k + 'nominal_mode_pred',
                    k + 'predicted_mode', k + 'mode_code', k + 'mode'], required=False)


def case_metrics(segments):
    metrics = {}
    for wheel in WHEELS:
        metrics[wheel] = {}
        for channel in CHANNELS:
            errors = []
            coast_errors = []
            for segment in segments:
                obs, pred, coast = channel_arrays(segment['arrays'], wheel, channel)
                errors.append(np.degrees(pred - obs))
                if coast is not None:
                    coast_errors.append(np.degrees(coast - obs))
            error = np.concatenate(errors)
            wrapped = np.degrees(np.arctan2(np.sin(np.radians(error)), np.cos(np.radians(error))))
            row = {
                'unwrapped_rmse_deg': float(np.sqrt(np.mean(error ** 2))),
                'wrapped_rmse_deg': float(np.sqrt(np.mean(wrapped ** 2))),
                'max_absolute_error_deg': float(np.max(np.abs(error))),
                'last_segment_final_error_deg': float(error[-1]),
                'sample_count': len(error),
            }
            if coast_errors:
                coast_error = np.concatenate(coast_errors)
                row['coasting_sensitivity_rmse_deg'] = float(np.sqrt(np.mean(coast_error ** 2)))
            metrics[wheel][channel] = row
    return metrics


def short_case(case):
    parts = case.split('_')
    stamp = parts[1].split('T')[-1].rstrip('Z')
    return f'{stamp[:2]}:{stamp[2:4]}:{stamp[4:6]} UTC · {parts[2]}'


def plot_case(case, segments, out, residual=False):
    plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 10,
                         'axes.spines.top': False, 'axes.spines.right': False})
    fig, axes = plt.subplots(3, 2, figsize=(14, 10), sharex='col')
    metrics = case_metrics(segments)
    all_times = np.concatenate([s['time'] for s in segments])
    low, high = float(all_times.min()), float(all_times.max())
    for col, wheel in enumerate(WHEELS):
        for row, channel in enumerate(CHANNELS):
            ax = axes[row, col]
            for index, segment in enumerate(segments):
                a, t = segment['arrays'], segment['time']
                obs, pred, coast = channel_arrays(a, wheel, channel)
                mode = nominal_mode(a, wheel)
                if mode is not None:
                    shade_boolean(ax, t, (mode != 1) & (mode != 2))
                if residual:
                    ax.plot(t, np.degrees(pred - obs), color=COLORS['predicted'], lw=1.35)
                    if coast is not None:
                        ax.plot(t, np.degrees(coast - obs), color=COLORS['coast'],
                                lw=.95, ls=(0, (4, 3)), alpha=.9)
                else:
                    ax.plot(t, obs / (2 * np.pi), color=COLORS['measured'], lw=1.5)
                    ax.plot(t, pred / (2 * np.pi), color=COLORS['predicted'], lw=1.2)
                    if coast is not None:
                        ax.plot(t, coast / (2 * np.pi), color=COLORS['coast'],
                                lw=.95, ls=(0, (4, 3)), alpha=.85)
                if index:
                    ax.axvline(t[0], color='#9c5371', lw=.8, ls=':')
            for previous, following in zip(segments, segments[1:]):
                ax.axvspan(previous['time'][-1], following['time'][0], facecolor='#e5d7dd',
                           alpha=.55, edgecolor='#bd9aaa', hatch='///', lw=0, zorder=-3)
            ax.set_ylabel(f'{channel} error (deg)' if residual else f'{channel} change (turns)')
            ax.set_xlim(low, high)
            ax.grid(alpha=.18)
            if residual:
                ax.axhline(0, color='#526579', lw=.7, alpha=.65)
                rms = metrics[wheel][channel]['unwrapped_rmse_deg']
                ax.text(.985, .93, f'RMSE {rms:,.1f}°', transform=ax.transAxes,
                        ha='right', va='top', fontsize=10, color='#344759',
                        bbox={'facecolor': 'white', 'edgecolor': 'none', 'alpha': .8, 'pad': 2})
            if row == 0:
                ax.set_title(wheel.replace('_', ' ').title(), fontsize=13, pad=10)
            if row == 2:
                ax.set_xlabel('Time from shared case interval start (s)')
    legend = []
    if not residual:
        legend.append(Line2D([0], [0], color=COLORS['measured'], lw=2, label='Recorded encoder'))
    legend += [Line2D([0], [0], color=COLORS['predicted'], lw=2,
                      label='Prediction − encoder' if residual else 'Ideal prediction: axial phases held off shell'),
               Line2D([0], [0], color=COLORS['coast'], lw=1.3, ls='--', label='Coasting sensitivity'),
               Patch(facecolor='#9aa4b0', alpha=.2, label='Predicted non-shell sector')]
    if len(segments) > 1:
        legend.append(Patch(facecolor='#e5d7dd', hatch='///', label='Excluded data gap'))
    heading = 'Accumulated joint-angle differences' if residual else 'Recorded and reconstructed joint rotations'
    fig.suptitle(f'{heading} · {short_case(case)}', fontsize=17, y=.99)
    fig.legend(handles=legend, loc='upper center', bbox_to_anchor=(.5, .951),
               ncol=3 if len(legend) > 4 else 4, frameon=False, fontsize=9)
    if len(segments) == 1:
        scope = 'One initialization; no later encoder corrections.'
    else:
        scope = f'{len(segments)} independently initialized valid segments; curves restart at each segment.'
    fig.text(.5, .026, f'R = 100 mm. {scope}\n'
             'H1/H2 are axial joint phases; all bodies also follow CENTER and base yaw. '
             'Coasting is a sensitivity model, not an uncertainty bound.',
             ha='center', color='#526579', fontsize=9.5)
    fig.subplots_adjust(top=.86, bottom=.105, left=.085, right=.98, hspace=.23, wspace=.20)
    filename = case + ('_residuals.png' if residual else '_angles.png')
    fig.savefig(out / filename, dpi=170)
    plt.close(fig)
    return filename, metrics


def candidate_cases(manifest, results, grouped):
    cases = {case: {} for case in grouped}
    for doc in (manifest, results):
        for key in ('cases', 'runs', 'recordings'):
            rows = doc.get(key, []) if isinstance(doc, dict) else []
            if isinstance(rows, dict):
                rows = [dict(value, run=name) if isinstance(value, dict) else {'run': name}
                        for name, value in rows.items()]
            for row in rows:
                name = row.get('run', row.get('case', row.get('case_id', row.get('name'))))
                if name:
                    cases.setdefault(name, {}).update(row)
        for key in ('unavailable_cases', 'skipped_cases', 'failures'):
            for row in doc.get(key, []) if isinstance(doc, dict) else []:
                if not isinstance(row, dict):
                    continue
                name = row.get('run', row.get('case', row.get('case_id')))
                if name:
                    cases.setdefault(name, {}).update(row)
    return cases


def text_reason(row):
    for key in ('reason', 'unavailable_reason', 'unavailable_reasons', 'error', 'status_reason', 'notes'):
        value = row.get(key)
        if value:
            return '; '.join(map(str, value)) if isinstance(value, list) else str(value)
    return 'No usable synchronized trajectory and six-channel encoder history was supplied to this reconstruction.'


def plot_native_only(case, row, out):
    value = row.get('native_encoder_npz')
    if not value:
        return None
    path = relocated(value)
    bundled_path = out / 'native_encoder' / path.name
    if bundled_path.exists():
        path = bundled_path
    if not path.exists():
        return None
    native_dir = out / 'native_encoder'
    native_dir.mkdir(exist_ok=True)
    if path.resolve() != (native_dir / path.name).resolve():
        shutil.copyfile(path, native_dir / path.name)
    with np.load(path) as z:
        a = {key: z[key].copy() for key in z.files}
    fig, axes = plt.subplots(3, 2, figsize=(14, 9), sharex=False)
    for col, wheel in enumerate(WHEELS):
        for index, (channel, node) in enumerate((('CENTER', 3), ('H1', 1), ('H2', 2))):
            ax = axes[index, col]
            prefix = f'{wheel}__node{node}__'
            for key in sorted(a):
                if key.startswith(prefix) and key.endswith('__board_elapsed_s'):
                    base = key[:-len('board_elapsed_s')]
                    t = a[key]
                    angle = a[base + 'angle_change_rad']
                    ax.plot(t, angle / (2 * np.pi), color=COLORS['measured'], lw=1.3)
            ax.grid(alpha=.18)
            ax.set_ylabel(f'{channel} change (turns)')
            ax.set_xlabel('Seconds from this encoder board’s first sample')
            if index == 0:
                ax.set_title(wheel.replace('_', ' ').title(), fontsize=13)
    fig.suptitle(f'Encoder histories only · {short_case(case)}', fontsize=17, y=.99)
    fig.text(.5, .933, 'No synchronized local Vicon trajectory: no reconstructed curves or comparison errors are shown.',
             ha='center', color='#8b5b1d', fontsize=11)
    fig.text(.5, .026, 'Each channel uses its own board clock. Curves are not synchronized across boards.\n'
             'Left H1 ends early; other channels continue. Initial phase is zeroed independently for display.',
             ha='center', color='#526579', fontsize=10)
    fig.subplots_adjust(top=.86, bottom=.12, left=.085, right=.98, hspace=.34, wspace=.20)
    filename = case + '_encoder_only.png'
    fig.savefig(out / 'plots' / filename, dpi=170)
    plt.close(fig)
    return filename, path.name


def write_html(out, grouped, case_rows, metrics, plot_paths, source_dir):
    esc = html.escape
    total_seconds = sum(float(s['time'][-1] - s['time'][0]) for ss in grouped.values() for s in ss)
    total_segments = sum(map(len, grouped.values()))
    summaries, articles, options = [], [], []
    for case in sorted(case_rows):
        idx = 'case-' + case
        label = short_case(case)
        options.append(f'<option value="{esc(idx)}">{esc(label)}{ " — unavailable" if case not in grouped else ""}</option>')
        if case not in grouped:
            reason = text_reason(case_rows[case])
            native = plot_native_only(case, case_rows[case], out)
            native_block = '' if native is None else f'<figure><a href="plots/{esc(native[0])}"><img src="plots/{esc(native[0])}" alt="Six recorded encoder channels for incomplete case {esc(case)}"></a><figcaption>Recorded encoders only, on separate board clocks. These curves are not time-aligned with one another or Vicon.</figcaption></figure><p><a href="native_encoder/{esc(native[1])}">Download native encoder arrays</a></p>'
            summaries.append(f'<tr><td><a href="#{idx}" data-case="{idx}">{esc(label)}</a></td><td colspan="7">Unavailable: {esc(reason)}</td></tr>')
            articles.append(f'<article id="{idx}" class="case"><h2>{esc(label)}</h2><p class="filename">{esc(case)}</p><p class="notice">Reconstruction unavailable: {esc(reason)} The saved clock mapping also rejects or lacks two channels. The original session is marked needs_recovery.</p>{native_block}</article>')
            continue
        segments = grouped[case]
        duration = sum(float(s['time'][-1] - s['time'][0]) for s in segments)
        scope = f'{duration:.2f} s · {len(segments)} segment' + ('s' if len(segments) != 1 else '')
        vals = [metrics[case][w][c]['unwrapped_rmse_deg'] for w in WHEELS for c in CHANNELS]
        cells = ''.join(f'<td>{value:,.1f}</td>' for value in vals)
        summaries.append(f'<tr><td><a href="#{idx}" data-case="{idx}">{esc(label)}</a></td><td>{esc(scope)}</td>{cells}</tr>')
        interval_rows = ''.join(f'<tr><td>{i + 1}</td><td>{s["time"][0]:.3f}–{s["time"][-1]:.3f}</td><td>{s["time"][-1] - s["time"][0]:.3f}</td><td><a href="histories/{esc(s["path"].name)}">NPZ history</a></td></tr>' for i, s in enumerate(segments))
        mrows = []
        for wheel in WHEELS:
            for channel in CHANNELS:
                m = metrics[case][wheel][channel]
                coast = m.get('coasting_sensitivity_rmse_deg')
                mrows.append(f'<tr><td>{wheel.replace("_", " ").title()}</td><td>{channel}</td><td>{m["unwrapped_rmse_deg"]:,.2f}</td><td>{m["wrapped_rmse_deg"]:,.2f}</td><td>{m["last_segment_final_error_deg"] / 360:+.3f}</td><td>{"—" if coast is None else f"{coast:,.2f}"}</td></tr>')
        angle, residual = plot_paths[case]
        articles.append(f'''<article id="{idx}" class="case">
<div class="case-head"><div><h2>{esc(label)}</h2><p class="filename">{esc(case)}</p></div><span class="badge">{esc(scope)}</span></div>
<p>All angles are changes from each valid segment’s initial state. The main prediction uses no later encoder feedback. {'This case has multiple independently initialized segments; accumulated drift is not continued across excluded gaps.' if len(segments) > 1 else 'This case is propagated continuously over its available common interval.'}</p>
<div class="tabs"><button type="button" data-view="angles" class="active">Joint rotations</button><button type="button" data-view="residuals">Prediction − encoder</button></div>
<figure class="plot angles"><a href="plots/{esc(angle)}"><img loading="lazy" src="plots/{esc(angle)}" alt="CENTER, H1 and H2 predictions compared with recorded encoders for {esc(case)}"></a><figcaption>Orange: recorded. Blue: main ideal reconstruction. Dashed gray: off-contact coasting sensitivity. Gray bands: predicted gap or polar sector; these are geometric labels, not observed contact modes.</figcaption></figure>
<figure class="plot residuals" hidden><a href="plots/{esc(residual)}"><img loading="lazy" src="plots/{esc(residual)}" alt="Unwrapped CENTER, H1 and H2 residuals for {esc(case)}"></a><figcaption>Unwrapped errors preserve missed or extra revolutions. The coasting curve changes only the assumed evolution of axial spin when the shell is not active.</figcaption></figure>
<details><summary>Numerical comparison and valid intervals</summary><div class="table-scroll"><table><thead><tr><th>Caster</th><th>Channel</th><th>Unwrapped RMSE °</th><th>Wrapped RMSE °</th><th>Final error, turns*</th><th>Coasting RMSE °</th></tr></thead><tbody>{''.join(mrows)}</tbody></table></div><p class="small">*Final error refers to the final valid segment. RMSE pools samples across independently initialized segments; wrapped error alone can conceal missing turns.</p><table><thead><tr><th>Segment</th><th>Case time, seconds</th><th>Duration, seconds</th><th>Arrays</th></tr></thead><tbody>{interval_rows}</tbody></table></details>
</article>''')
    source_links = []
    for path in sorted(source_dir.glob('*.py')):
        source_links.append(f'<a href="source/{esc(path.name)}">{esc(path.name)}</a>')
    default = 'case-' + (SELECTED if SELECTED in grouped else next(iter(sorted(case_rows))))
    html_text = f'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Caster reconstruction · all recordings</title>
<style>
:root{{--ink:#173047;--muted:#5f7282;--line:#d9e2e9;--blue:#146e9a;--paper:#fff;--bg:#edf2f6}}
*{{box-sizing:border-box}}body{{margin:0;background:var(--bg);color:var(--ink);font:16px/1.55 system-ui,-apple-system,Segoe UI,sans-serif}}main{{max-width:1440px;margin:auto;padding:34px 28px 60px}}h1{{font-size:34px;line-height:1.15;margin:0 0 12px}}h2{{font-size:24px;line-height:1.2;margin:0}}h3{{font-size:18px}}p{{max-width:1120px}}a{{color:#086b9c;text-underline-offset:3px}}.lead{{font-size:18px;color:#354e62}}.eyebrow{{font-size:12px;letter-spacing:.13em;font-weight:700;text-transform:uppercase;color:var(--blue);margin:0 0 12px}}.cards{{display:flex;flex-wrap:wrap;gap:14px;margin:24px 0}}.stat{{background:white;padding:16px 22px;border:1px solid var(--line);border-radius:10px;min-width:190px}}.stat b{{display:block;font-size:27px}}.stat span{{color:var(--muted);font-size:14px}}.panel,.case{{background:var(--paper);border:1px solid var(--line);border-radius:12px;padding:24px;margin-top:20px}}.notice{{background:#fff6df;border-left:4px solid #c7922b;padding:13px 17px;max-width:none}}.filename{{font:12px/1.5 ui-monospace,SFMono-Regular,Consolas,monospace;color:var(--muted);overflow-wrap:anywhere;margin:7px 0}}.case-head{{display:flex;justify-content:space-between;align-items:flex-start;gap:20px}}.badge{{background:#eaf3f8;border-radius:20px;padding:7px 13px;white-space:nowrap;font-size:13px;color:#2d5c75}}.small,figcaption{{font-size:13px;color:var(--muted)}}.selector{{display:flex;align-items:center;gap:14px;flex-wrap:wrap;margin:24px 0 0}}select{{font:inherit;background:white;border:1px solid #9bafbf;border-radius:7px;padding:9px 12px;min-width:330px;max-width:100%}}button{{font:inherit;cursor:pointer;border:1px solid var(--line);border-radius:6px;padding:8px 13px;background:white;color:var(--ink)}}button.active{{background:var(--blue);color:white;border-color:var(--blue)}}.tabs{{display:flex;gap:8px;margin:22px 0 8px}}figure{{margin:0}}img{{display:block;width:100%;height:auto;border:1px solid #e2e8ed}}figcaption{{padding:9px 2px}}details{{margin-top:20px}}summary{{cursor:pointer;font-weight:600}}table{{border-collapse:collapse;font-size:13px;width:100%;margin:12px 0}}th,td{{padding:9px 10px;border-bottom:1px solid var(--line);text-align:right;vertical-align:top}}th{{background:#f0f5f8;font-weight:600}}th:first-child,td:first-child{{text-align:left}}.table-scroll{{overflow-x:auto}}.assumptions{{display:grid;grid-template-columns:1fr 1fr;gap:16px 28px}}.assumptions p{{margin:6px 0}}code{{font:13px ui-monospace,Consolas,monospace}}footer{{font-size:13px;color:var(--muted);margin-top:28px}}[hidden]{{display:none!important}}@media(max-width:800px){{main{{padding:22px 12px}}h1{{font-size:28px}}.panel,.case{{padding:17px}}.case-head{{display:block}}.badge{{display:inline-block;margin-top:8px}}.assumptions{{grid-template-columns:1fr}}.stat{{min-width:140px;flex:1}}}}
</style></head><body><main>
<p class="eyebrow">HAMR ball caster · recorded trajectory reconstruction</p><h1>CENTER and both hemispheres, for every available recording</h1>
<p class="lead">An independent kinematic history driven by the recorded caster path, base yaw and initial encoder state. Each comparison shows all six channels: CENTER, H1 and H2 for the front-left and front-right casters.</p>
<div class="cards"><div class="stat"><b>{len(grouped)} / {len(case_rows)}</b><span>cases reconstructed</span></div><div class="stat"><b>{total_segments}</b><span>valid segments</span></div><div class="stat"><b>{total_seconds:,.1f} s</b><span>recorded time reconstructed</span></div><div class="stat"><b>6 channels</b><span>per reconstructed case</span></div></div>
<section class="panel"><h2>What this ideal model assumes</h2><div class="assumptions"><div><h3>Motion and initial state</h3><p>The sphere radius is fixed at 100 mm. CENTER integrates forward displacement in the upright fork frame. Base Vicon yaw supplies fork heading with the existing 90° offset; caster-marker XY is the current ball-center proxy.</p><p>Encoder readings initialize each valid segment once. Subsequent readings are comparison data only. No radius, timing, yaw or phase parameters are fitted to these traces.</p></div><div><h3>Hemisphere continuation</h3><p>Only the nominal lower shell rotates under the ideal rolling equation, using the predicted CENTER state. Its effective lever arm is <code>R × |sin δ|</code>. The 20 mm gap and 50 mm polar opening define excluded smooth-shell sectors.</p><p>The main model holds axial spin phase while a shell is inactive or in a non-shell sector. The dashed sensitivity instead coasts at its last active axial rate, starting at zero if initially inactive. These assumptions do not hold the full shell orientation fixed: it still follows CENTER and base yaw.</p></div></div>
<p class="notice">Gray sectors do not reconstruct physical rim or roller contact. They identify where smooth-shell rolling is not applied. Neither the main closure nor the coasting sensitivity is a calibrated contact-dynamics model. H1/H2 signs and scale, extrinsics and clock alignment retain the existing recording assumptions.</p>
<details><summary>Equations and how to read the errors</summary><p><code>δ̇ = −vₓ/R</code>. H1 is nominally active when <code>cos δ &lt; 0</code>, with <code>β̇₁ = vᵧ/(R sin δ)</code>; H2 is active when <code>cos δ &gt; 0</code>, with <code>β̇₂ = −vᵧ/(R sin δ)</code>. A shell is eligible only when <code>0.1 ≤ |cos δ| ≤ √(1 − 0.25²)</code>. Therefore the active-shell denominator is bounded away from zero.</p><p>Plots use initial-aligned joint rotations. Unwrapped residuals preserve extra or missing turns. Wrapped residuals describe phase agreement modulo one revolution. Differences combine model mismatch and retained measurement/calibration assumptions; they are not isolated measurements of slip.</p></details></section>
<section class="panel"><h2>All-case comparison</h2><p class="small">Unwrapped phase RMSE in degrees. Segmented cases are reinitialized after excluded gaps, so their pooled errors do not represent uninterrupted full-run drift.</p><div class="table-scroll"><table><thead><tr><th rowspan="2">Recording start / ID</th><th rowspan="2">Coverage</th><th colspan="3">Front left</th><th colspan="3">Front right</th></tr><tr><th>CENTER</th><th>H1</th><th>H2</th><th>CENTER</th><th>H1</th><th>H2</th></tr></thead><tbody>{''.join(summaries)}</tbody></table></div></section>
<div class="selector"><label for="case-select"><strong>Inspect a recording</strong></label><select id="case-select">{''.join(options)}</select><button id="previous" type="button">Previous</button><button id="next" type="button">Next</button></div>
{''.join(articles)}
<section class="panel"><h2>Data and reproducibility</h2><p><a href="results.json">Full numerical results</a> · <a href="input_manifest.json">Input coverage and exclusions</a> · <a href="plot_metrics.json">Values shown in this report</a></p><p>{' · '.join(source_links)}</p><p class="small">Each segment’s NPZ download is available inside its recording panel. Original rosbags and repository files were read only.</p></section>
<footer>Offline reconstruction of recorded motion. This report does not predict how caster forces would change the robot’s path.</footer></main>
<script>
const selector=document.getElementById('case-select');const cases=[...document.querySelectorAll('article.case')];
function selectCase(id,scroll=false){{if(!cases.some(e=>e.id===id))id={json.dumps(default)};selector.value=id;cases.forEach(e=>e.hidden=e.id!==id);if(scroll)document.getElementById(id).scrollIntoView({{behavior:'smooth',block:'start'}});}}
selector.addEventListener('change',()=>{{selectCase(selector.value);history.replaceState(null,'','#'+selector.value);}});
document.querySelectorAll('[data-case]').forEach(a=>a.addEventListener('click',e=>{{e.preventDefault();selectCase(a.dataset.case,true);history.replaceState(null,'','#'+a.dataset.case);}}));
document.querySelectorAll('.tabs button').forEach(b=>b.addEventListener('click',()=>{{const article=b.closest('article');article.querySelectorAll('.plot').forEach(p=>p.hidden=!p.classList.contains(b.dataset.view));article.querySelectorAll('.tabs button').forEach(x=>x.classList.toggle('active',x===b));}}));
function move(step){{selector.selectedIndex=(selector.selectedIndex+step+selector.options.length)%selector.options.length;selector.dispatchEvent(new Event('change'));}}
document.getElementById('previous').onclick=()=>move(-1);document.getElementById('next').onclick=()=>move(1);selectCase(location.hash.slice(1)||{json.dumps(default)});
</script></body></html>'''
    (out / 'index.html').write_text(html_text, encoding='utf-8')


def main():
    here = Path(__file__).resolve()
    project = DATA_ROOT
    parser = argparse.ArgumentParser(description=__doc__)
    default_output = OUTPUT_ROOT/'all_caster_reconstruction'
    parser.add_argument('--output-dir', type=Path, default=default_output)
    parser.add_argument('--history-dir', type=Path)
    parser.add_argument('--manifest', type=Path)
    args = parser.parse_args()
    out = args.output_dir
    history_dir = args.history_dir or out / 'histories'
    manifest_path = args.manifest or out / 'input_manifest.json'
    results = json.loads((out / 'results.json').read_text())
    manifest = json.loads(manifest_path.read_text()) if manifest_path.exists() else results.get('input_manifest', {})
    if not manifest_path.exists():
        (out / 'input_manifest.json').write_text(json.dumps(manifest, indent=2))
    grouped = load_histories(history_dir)
    if not grouped:
        raise ValueError('No reconstructed history files found')
    plot_dir = out / 'plots'
    plot_dir.mkdir(exist_ok=True, parents=True)
    source_dir = out / 'source'
    source_dir.mkdir(exist_ok=True, parents=True)
    if here != (source_dir / here.name).resolve():
        shutil.copyfile(here, source_dir / here.name)
    case_rows = candidate_cases(manifest, results, grouped)
    metrics, paths = {}, {}
    for case, segments in sorted(grouped.items()):
        angle, metric = plot_case(case, segments, plot_dir, residual=False)
        residual, _ = plot_case(case, segments, plot_dir, residual=True)
        metrics[case] = metric
        paths[case] = (angle, residual)
        print(case + ' plotted', flush=True)
    (out / 'plot_metrics.json').write_text(json.dumps(metrics, indent=2))
    write_html(out, grouped, case_rows, metrics, paths, source_dir)
    print(str(out / 'index.html'), flush=True)


if __name__ == '__main__':
    main()
