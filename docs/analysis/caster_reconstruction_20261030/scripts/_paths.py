"""Shared portable CLI paths; released evidence is always read-only."""
from pathlib import Path
import argparse
import os
import sys


def configure():
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--data-root', type=Path, default=os.environ.get('CASTER_DATA_ROOT'))
    parser.add_argument('--output-dir', type=Path)
    args, remaining = parser.parse_known_args()
    if args.data_root is None:
        parser.error('Pass --data-root PATH or set CASTER_DATA_ROOT to the extracted release data folder.')
    data = args.data_root.expanduser().resolve()
    if not data.is_dir():
        parser.error(f'Data directory does not exist: {data}')
    out = (args.output_dir or data/'reproduced').expanduser().resolve()
    protected = [data/name for name in ('outputs', 'work', 'recordings', 'config')]
    if out == data or any(out == p or p in out.parents for p in protected):
        parser.error('Output must be separate from the released outputs/work/recordings/config trees; use DATA_ROOT/reproduced.')
    sys.argv = [sys.argv[0], *remaining]
    return data, out


DATA_ROOT, OUTPUT_ROOT = configure()


def reconstruction_source():
    fresh = OUTPUT_ROOT/'all_caster_reconstruction'
    return fresh if (fresh/'results.json').exists() else DATA_ROOT/'outputs/all_caster_reconstruction'


def prepared_inputs():
    fresh = OUTPUT_ROOT/'prepared/inputs'
    return fresh if fresh.is_dir() else DATA_ROOT/'work/all_caster_reconstruction/inputs'


def relocated(value):
    """Resolve paths in original manifests against the unpacked release root."""
    normalized = str(value).replace('\\', '/')
    for component in ('work', 'outputs', 'recordings', 'config'):
        marker = '/'+component+'/'
        if marker in normalized:
            candidate = DATA_ROOT/component/normalized.split(marker, 1)[1]
            if candidate.exists():
                return candidate
    name = Path(normalized).name
    for directory in (OUTPUT_ROOT/'prepared/native_encoder',
                      DATA_ROOT/'work/all_caster_reconstruction/native_encoder'):
        candidate = directory/name
        if candidate.exists():
            return candidate
    if normalized.startswith(('work/', 'outputs/', 'recordings/', 'config/')):
        return DATA_ROOT/normalized
    # Do not silently read the original author's computer when a release
    # component is absent. Callers can report this as an unavailable file.
    return DATA_ROOT/'unresolved-original-path'/name
