"""Build a separate local page; never edit the previous comparison report."""
from pathlib import Path
from _paths import DATA_ROOT, OUTPUT_ROOT, reconstruction_source, prepared_inputs, relocated
import html
import json
import re
import shutil

HERE = Path(__file__).resolve()
ROOT = DATA_ROOT
OUT = OUTPUT_ROOT/'caster_trajectory_replay'
FEATURED = 'encoder_20261030T005046Z_7a2266b7'
ESC = html.escape

def label(case):
    match = re.search(r'T(\d{2})(\d{2})(\d{2})Z_([a-f0-9]+)', case)
    return f'{match[1]}:{match[2]}:{match[3]} UTC · {match[4]}' if match else case

def relfile(value):
    p = Path(value)
    if p.is_absolute():
        return p.relative_to(OUT).as_posix()
    if (OUT/p).exists():
        return p.as_posix()
    if (OUT/'contact'/p).exists():
        return ('contact'/p).as_posix()
    return 'contact/'+p.name

def num(value, places=3):
    return '—' if value is None else f'{value:,.{places}f}'

def main():
    summary = json.loads((OUT/'contact/contact_metrics.json').read_text())
    cases = sorted(summary['cases'], key=lambda x:x['case'])
    global FEATURED
    if FEATURED not in {c['case'] for c in cases}:FEATURED=cases[0]['case']
    cards = []
    missing = []
    plot_titles = {'phase':'Contact rotations','increment':'Change within each contact',
                   'residual':'Accumulated error','rate':'Contact rotation rates'}
    captions = {
        'phase':'Both curves are shown only during the same encoder-inferred contact intervals. Accumulated rotations keep their original segment origins.',
        'increment':'Each curve is shown as its change from the first valid sample of that inferred contact episode. Zero at entry is constructed; the original predictor is not reset.',
        'residual':'Prediction minus encoder, without wrapping away complete turns. Only encoder-inferred contact intervals are shown.',
        'rate':'Rates are compared only across time steps wholly within the encoder-inferred shell-contact sector. Boundary-crossing steps are excluded.'}
    for case in cases:
        name = case['case']
        videos = sorted((OUT/'videos').glob(name+'*.mp4'))
        posters = sorted((OUT/'videos').glob(name+'*.png')) + sorted((OUT/'posters').glob(name+'*.png'))
        if not videos:
            missing.append(name)
            videohtml = '<p>Video unavailable.</p>'
        else:
            video = videos[0].relative_to(OUT).as_posix()
            poster = f' poster="{ESC(posters[0].relative_to(OUT).as_posix())}"' if posters else ''
            videohtml = f'<video controls preload="metadata" playsinline{poster}><source src="{ESC(video)}" type="video/mp4"></video><p class="small"><a href="{ESC(video)}">Open or download this MP4</a></p>'
        buttons, plots = [], []
        files = case['files']
        if isinstance(files,list):
            files={next((k for k in plot_titles if k in Path(v).stem),'phase'):v for v in files}
        for i, kind in enumerate(plot_titles):
            candidates = [value for key,value in files.items() if kind in key]
            if not candidates:
                candidates = [str(p) for p in sorted((OUT/'contact').glob(name+'*'+kind+'*.png'))]
            if not candidates:
                continue
            target = relfile(candidates[0])
            buttons.append(f'<button type="button" data-kind="{kind}" class="{"active" if i==0 else ""}">{plot_titles[kind]}</button>')
            plots.append(f'<figure data-kind="{kind}" {"hidden" if i else ""}><a href="{ESC(target)}"><img loading="lazy" src="{ESC(target)}" alt="{ESC(plot_titles[kind]+": "+name)}"></a><figcaption>{captions[kind]}</figcaption></figure>')
        rows=[]
        for wheel in ('front_left','front_right'):
            for channel in ('center','h1','h2'):
                m=case['wheels'][wheel].get(channel)
                if m is None:
                    continue
                agreement=m.get('predicted_contact_agreement_fraction_of_observed_contact_samples',
                    m.get('predicted_same_hemisphere_fraction_of_observed_contact_samples'))
                rows.append('<tr>'+''.join(f'<td>{value}</td>' for value in (
                    wheel.replace('_',' ').title(),channel.upper(),
                    num(m.get('observed_inferred_contact_duration_s'),2),
                    num(m['primary_observed_contact_unwrapped_phase_error_turns']['rmse']),
                    num(m['primary_strict_observed_contact_rate_error_turns_s']['rmse']),
                    num(100*agreement,1)+'%' if agreement is not None else '—'))+'</tr>')
        gaps = len(case['segments'])>1
        coverage = f"{case['available_duration_s']:.2f} s of valid shared data · {len(case['segments'])} segment{'s' if gaps else ''}"
        note = '<p class="notice">This recording contains excluded data. The video labels the gap and the model reinitializes at the next valid segment. No motion or phase is interpolated across it.</p>' if gaps else ''
        cards.append(f'''<article class="case" id="case-{ESC(name)}">
<div class="case-head"><div><h2>{ESC(label(name))}</h2><p class="filename">{ESC(name)}</p></div><span class="badge">{coverage}</span></div>
{note}{videohtml}
<p class="video-note"><b>Left:</b> Vicon path + encoder rotations. <b>Right:</b> the same Vicon path + predicted rotations. Matching translation is imposed by the recorded input. Videos use 15 frames/s; the plots use the 100 Hz analysis histories.</p>
<h3>Only during inferred hemisphere–ground contact</h3><div class="tabs">{''.join(buttons)}</div>{''.join(plots)}
<details><summary>Contact-only metrics and coverage</summary><div class="table-wrap"><table><thead><tr><th>Caster</th><th>Channel</th><th>Contact time, s</th><th>Phase RMSE, turns</th><th>Rate RMSE, turns/s</th><th>Predicted contact agreement</th></tr></thead><tbody>{''.join(rows)}</tbody></table></div>
<p class="small">Contact selection uses the recorded CENTER state. Agreement is the fraction of those contact samples for which the prediction selects the same hemisphere (or any shell for CENTER). The score does not discard disagreement intervals. Secondary intersection-only statistics and per-contact increments are in the JSON.</p>
<p><a href="contact/{ESC(name)}__contact_metrics.json">Full case contact metrics</a></p></details></article>''')
    options=''.join(f'<option value="case-{ESC(c["case"])}">{ESC(label(c["case"]))}</option>' for c in cases)
    page='''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1"><title>Caster trajectory replay and contact-only comparisons</title>
<style>
:root{--ink:#1b3449;--muted:#597083;--blue:#126f98;--line:#dce5eb;--paper:#fff;--bg:#eef3f6}*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--ink);font:16px/1.55 system-ui,Segoe UI,sans-serif}main{max-width:1460px;margin:auto;padding:32px 26px 60px}h1{font-size:34px;line-height:1.15;margin:10px 0 14px}h2{font-size:24px;margin:0}h3{font-size:20px;margin:25px 0 10px}p{max-width:1140px}a{color:var(--blue);text-underline-offset:3px}.eyebrow{font-size:12px;font-weight:700;letter-spacing:.11em;text-transform:uppercase;color:var(--blue)}.lead{font-size:18px;color:#3c566a}.panel,.case{background:white;border:1px solid var(--line);border-radius:12px;padding:24px;margin-top:22px}.small,figcaption{font-size:13px;color:var(--muted)}.filename{font:12px/1.5 ui-monospace,Consolas,monospace;color:var(--muted);overflow-wrap:anywhere}.case-head{display:flex;justify-content:space-between;align-items:flex-start;gap:20px;margin-bottom:18px}.badge{font-size:13px;padding:7px 12px;background:#edf5f9;border-radius:18px;white-space:nowrap}.notice{background:#fff5df;border-left:4px solid #be8b27;padding:12px 15px}.selector,.tabs{display:flex;flex-wrap:wrap;gap:9px;align-items:center}.selector{margin-top:24px}select,button{font:inherit;border:1px solid #b7c8d4;border-radius:7px;padding:9px 13px;background:white;color:var(--ink)}select{min-width:320px;max-width:100%}button{cursor:pointer}button.active{background:var(--blue);color:white;border-color:var(--blue)}video{display:block;width:100%;background:#172d3c;border-radius:7px}figure{margin:16px 0 0}img{display:block;width:100%;height:auto;border:1px solid var(--line)}figcaption{padding:8px 2px}.video-note{font-size:14px;color:var(--muted)}details{margin-top:20px}summary{cursor:pointer;font-weight:650}table{width:100%;font-size:13px;border-collapse:collapse;margin-top:12px}th,td{padding:9px 10px;border-bottom:1px solid var(--line);text-align:right}th{background:#eef5f8}th:first-child,td:first-child{text-align:left}.table-wrap{overflow:auto}.grid{display:grid;grid-template-columns:1fr 1fr;gap:22px}.grid p{margin:6px 0}footer{font-size:13px;color:var(--muted);margin-top:25px}[hidden]{display:none!important}@media(max-width:800px){main{padding:20px 12px}h1{font-size:28px}.panel,.case{padding:15px}.case-head{display:block}.badge{display:inline-block}.grid{grid-template-columns:1fr}select{min-width:240px}}
</style></head><body><main>
<p class="eyebrow">HAMR · flat-ground replay</p><h1>Recorded and predicted caster rotation along the Vicon trajectory</h1>
<p class="lead">A side-by-side rendered replay for each of the 11 recordings with synchronized motion data, followed by rotation comparisons restricted to encoder-inferred shell contact.</p>
<p><a href="../all_caster_reconstruction/index.html">Open the all-channel comparison page</a></p>
<section class="panel"><div class="grid"><div><h3>What moves in the video</h3><p>Both panes use the recorded Vicon trajectory. Encoder histories rotate the casters on the left; the existing ideal kinematic prediction rotates them on the right. The floor is flattened and the camera is a schematic view, not recorded camera footage.</p></div><div><h3>What counts as contact</h3><p>The recorded CENTER angle and the 100 mm radius, 20 mm gap and 50 mm polar opening determine nominal H1/H2 shell contact. Gap and polar sectors are excluded. CENTER is included only while either hemisphere is in its shell-contact sector.</p></div></div>
<p class="small">Contact is inferred geometrically, not measured by a contact sensor. The predictor is unchanged: inactive/non-shell axial phases are held. Plot filtering never feeds encoder measurements back into it. Marker-to-center, timing and encoder sign/scale assumptions are retained from the original report.</p></section>
<div class="selector"><label for="case-select"><b>Recording</b></label><select id="case-select">__OPTIONS__</select><button id="previous" type="button">Previous</button><button id="next" type="button">Next</button></div>
__CARDS__
<section class="panel"><h3>Coverage and files</h3><p>11 recordings, 13 valid segments, 479.15 seconds of usable shared data. The incomplete 02:11:39 recording has no locally available Vicon bag, so it has no trajectory replay. Its encoder-only view remains on the original comparison page.</p>
<p><a href="contact/contact_metrics.json">All contact metrics and assumptions</a> · <a href="contact/contact_analysis.py">Contact analysis source</a> · <a href="source/render_videos.py">Video renderer</a> · <a href="verification.json">Verification</a></p></section>
<footer>The original comparison page and its datasets are preserved. The new views reuse its predictions and measured histories.</footer></main>
<script>
const cases=[...document.querySelectorAll('article.case')],selector=document.getElementById('case-select');
function selectCase(id){if(!cases.some(c=>c.id===id))id='case-__FEATURED__';selector.value=id;cases.forEach(c=>{if(c.id!==id)c.querySelectorAll('video').forEach(v=>v.pause());c.hidden=c.id!==id;});history.replaceState(null,'','#'+id);}
selector.addEventListener('change',()=>selectCase(selector.value));
function move(n){selector.selectedIndex=(selector.selectedIndex+n+selector.options.length)%selector.options.length;selectCase(selector.value);}
document.getElementById('previous').onclick=()=>move(-1);document.getElementById('next').onclick=()=>move(1);
document.querySelectorAll('.tabs button').forEach(b=>b.addEventListener('click',()=>{const a=b.closest('article');a.querySelectorAll('figure[data-kind]').forEach(f=>f.hidden=f.dataset.kind!==b.dataset.kind);a.querySelectorAll('.tabs button').forEach(x=>x.classList.toggle('active',x===b));}));
selectCase(location.hash.slice(1)||'case-__FEATURED__');
</script></body></html>'''
    page=page.replace('__OPTIONS__',options).replace('__CARDS__',''.join(cards)).replace('__FEATURED__',FEATURED)
    page=page.replace('each of the 11 recordings',f'each of the {len(cases)} recordings')
    count=sum(len(c['segments']) for c in cases);seconds=sum(c['available_duration_s'] for c in cases)
    page=page.replace('11 recordings, 13 valid segments, 479.15 seconds',f'{len(cases)} recordings, {count} valid segments, {seconds:.2f} seconds')
    if len(cases) < 11:
        page=page.replace('The incomplete 02:11:39 recording has no locally available Vicon bag, so it has no trajectory replay. Its encoder-only view remains on the original comparison page.',
                          'This page contains only the cases included in this reproduction run. Use an unfiltered reconstruction to reproduce the full available dataset.')
    (OUT/'index.html').write_text(page,encoding='utf-8')
    (OUT/'source').mkdir(exist_ok=True)
    destination=OUT/'source/build_page.py'
    if Path(__file__).resolve()!=destination.resolve():
        shutil.copyfile(__file__,destination)
    print(json.dumps({'cases':len(cases),'videos_missing':missing,'page':str(OUT/'index.html')},indent=2))

if __name__=='__main__':
    main()
