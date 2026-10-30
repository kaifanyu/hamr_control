#!/usr/bin/env python3
"""Download and verify the published caster data without modifying existing files."""
from pathlib import Path, PurePosixPath
import argparse
import hashlib
import json
import shutil
import stat
import urllib.request
import zipfile


def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            h.update(chunk)
    return h.hexdigest()


def extract_verified(archive, destination):
    root = destination.resolve()
    with zipfile.ZipFile(archive) as z:
        for item in z.infolist():
            relative = PurePosixPath(item.filename)
            if relative.is_absolute() or '..' in relative.parts or any(':' in p or '\\' in p for p in relative.parts):
                raise ValueError('Unsafe archive path: '+item.filename)
            if stat.S_IFMT(item.external_attr >> 16) == stat.S_IFLNK:
                raise ValueError('Archive symlink rejected: '+item.filename)
            target = root.joinpath(*relative.parts)
            if not target.resolve().is_relative_to(root):
                raise ValueError('Archive path escapes destination: '+item.filename)
            if item.is_dir():
                target.mkdir(parents=True, exist_ok=True)
                continue
            target.parent.mkdir(parents=True, exist_ok=True)
            if target.exists():
                h = hashlib.sha256()
                with z.open(item) as stream:
                    for chunk in iter(lambda: stream.read(1024*1024), b''):
                        h.update(chunk)
                if digest(target) != h.hexdigest():
                    raise FileExistsError(f'Existing file differs; choose a fresh data directory: {target}')
                continue
            with z.open(item) as source, target.open('xb') as output:
                shutil.copyfileobj(source, output, 1024*1024)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', type=Path, required=True)
    parser.add_argument('--all', action='store_true', help='Also download all raw recordings and the original analysis batch.')
    parser.add_argument('--recordings', action='store_true', help='Also download every raw recording archive.')
    parser.add_argument('--case', help='Additionally download one exact recording folder name.')
    parser.add_argument('--list', action='store_true', help='List selected files without downloading.')
    args = parser.parse_args()
    manifest = json.loads((Path(__file__).resolve().parent.parent/'artifacts.json').read_text())
    selection = [a for a in manifest['assets'] if a['kind']=='analysis' or args.all
                 or (a['kind']=='recording' and (args.recordings or a.get('case')==args.case))]
    if args.case and not any(a.get('case')==args.case for a in selection):
        parser.error('Unknown recording case: '+args.case)
    print(f"Selected {len(selection)} archives, {sum(a['bytes'] for a in selection)/1e6:.2f} MB")
    for asset in selection:
        print(asset['name'], asset['sha256'])
    if args.list:
        return
    destination = args.destination.expanduser().resolve()
    cache = destination/'.downloads'
    cache.mkdir(parents=True, exist_ok=True)
    for asset in selection:
        archive = cache/asset['name']
        if not archive.exists() or digest(archive)!=asset['sha256']:
            temporary = archive.with_suffix(archive.suffix+'.part')
            request = urllib.request.Request(asset['url'], headers={'User-Agent':'hamr-caster-analysis-handoff'})
            print('Downloading '+asset['name'], flush=True)
            with urllib.request.urlopen(request, timeout=120) as source, temporary.open('wb') as target:
                shutil.copyfileobj(source, target, 1024*1024)
            if temporary.stat().st_size!=asset['bytes'] or digest(temporary)!=asset['sha256']:
                raise ValueError('Download size/SHA-256 mismatch: '+asset['name'])
            temporary.replace(archive)
        print('Extracting '+asset['name'], flush=True)
        extract_verified(archive, destination)
    print('Ready: '+str(destination/'outputs/caster_trajectory_replay/index.html'))


if __name__=='__main__':
    main()
