#!/usr/bin/env python3
"""Portable entry point for the published caster reconstruction analysis."""
from pathlib import Path
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
SELECTED = 'encoder_20261030T005046Z_7a2266b7'


def run(script, data, out, extra=()):
    command = [sys.executable, str(HERE/script), '--data-root', str(data),
               '--output-dir', str(out), *map(str, extra)]
    print('Running:', ' '.join(command), flush=True)
    subprocess.run(command, check=True)


def source_bundle(out):
    for folder in ('caster_trajectory_replay/source', 'all_caster_reconstruction/source'):
        destination = out/folder
        destination.mkdir(parents=True, exist_ok=True)
        for path in HERE.glob('*.py'):
            if path.resolve() != (destination/path.name).resolve():
                shutil.copy2(path, destination/path.name)


def ensure_reconstruction(data, out, case=None):
    destination = out/'all_caster_reconstruction'
    if (destination/'results.json').exists():
        return
    source = data/'outputs/all_caster_reconstruction'
    if not (source/'results.json').exists():
        raise SystemExit('No reconstruction output found. Run the reconstruct stage first.')
    destination.mkdir(parents=True, exist_ok=True)
    (destination/'histories').mkdir(exist_ok=True)
    summary = json.loads((source/'results.json').read_text())
    manifest = json.loads((source/'input_manifest.json').read_text())
    if case:
        summary['segments'] = [s for s in summary['segments'] if s['case'] == case]
        summary['case_count'] = len({s['case'] for s in summary['segments']})
        manifest['cases'] = [c for c in manifest['cases'] if c['run'] == case]
        summary['input_manifest'] = manifest
    for segment in summary['segments']:
        relative = Path(segment['history_file'])
        shutil.copy2(source/relative, destination/relative)
    (destination/'results.json').write_text(json.dumps(summary, indent=2))
    (destination/'input_manifest.json').write_text(json.dumps(manifest, indent=2))


def verification(out):
    target = out/'caster_trajectory_replay'
    target.mkdir(parents=True, exist_ok=True)
    records = []
    for relative in ('all_caster_reconstruction/results.json',
                     'caster_trajectory_replay/contact/contact_metrics.json',
                     'caster_trajectory_replay/videos_manifest.json'):
        path = out/relative
        if path.exists():
            records.append({'path': relative, 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()})
    (target/'verification.json').write_text(json.dumps({
        'scope': 'Portable reproduction artifact provenance. Numerical checks are recorded inside the stage results.',
        'released_inputs_modified': False, 'artifacts': records}, indent=2))


def report(data, out):
    run('build_report.py', data, out)
    source_bundle(out)
    verification(out)
    if (out/'caster_trajectory_replay/contact/contact_metrics.json').exists():
        run('build_page.py', data, out)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stage', choices=('prepare', 'reconstruct', 'contact', 'videos', 'report'))
    parser.add_argument('--data-root', type=Path, default=os.environ.get('CASTER_DATA_ROOT'))
    parser.add_argument('--output-dir', type=Path, help='Fresh output root; defaults to DATA_ROOT/reproduced.')
    parser.add_argument('--case', help='Exact recording folder name; omitted means all for prepare/reconstruct/contact.')
    parser.add_argument('--all', action='store_true', help='Render every available video (videos stage).')
    parser.add_argument('--poster-only', action='store_true', help='Render video posters without ffmpeg encoding.')
    parser.add_argument('--skip-existing', action='store_true', help='Skip already completed videos.')
    args = parser.parse_args()
    if args.data_root is None:
        parser.error('Provide --data-root PATH or set CASTER_DATA_ROOT.')
    data = args.data_root.expanduser().resolve()
    out = (args.output_dir or data/'reproduced').expanduser().resolve()
    if not data.is_dir():
        parser.error(f'Data directory does not exist: {data}')
    if out == data or any(out == data/p or data/p in out.parents for p in ('outputs','work','recordings','config')):
        parser.error('Output must be separate from released evidence; use DATA_ROOT/reproduced.')
    chosen = ['--case', args.case] if args.case else []
    if args.stage == 'prepare':
        run('prepare_inputs.py', data, out, chosen)
    elif args.stage == 'reconstruct':
        run('reconstruct_all.py', data, out, chosen)
        report(data, out)
    elif args.stage == 'contact':
        ensure_reconstruction(data, out, args.case)
        run('contact_analysis.py', data, out, chosen)
        report(data, out)
    elif args.stage == 'videos':
        ensure_reconstruction(data, out, args.case)
        extra = ['--all'] if args.all else ['--case', args.case or SELECTED]
        if args.poster_only: extra.append('--poster-only')
        if args.skip_existing: extra.append('--skip-existing')
        run('render_videos.py', data, out, extra)
        source_bundle(out); verification(out)
        if (out/'caster_trajectory_replay/contact/contact_metrics.json').exists():
            run('build_page.py', data, out)
    else:
        ensure_reconstruction(data, out, args.case)
        report(data, out)


if __name__ == '__main__':
    main()
