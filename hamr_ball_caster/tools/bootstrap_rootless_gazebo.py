#!/usr/bin/env python3
"""Extract Gazebo Harmonic's official apt packages into a user cache.

For Ubuntu 24.04 with an existing ROS 2 Jazzy installation and current apt package
lists. This performs no apt installation, runs no package maintainer scripts,
changes no system directory, and sends no CAD files. Normal system installation
is preferable for long-term use; this provides an isolated validation fallback.
"""

from __future__ import annotations

import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import urllib.parse
import urllib.request


REQUESTED = ("ros-jazzy-ros-gz-sim", "ros-jazzy-ros-gz-bridge", "ros-jazzy-xacro", "ffmpeg")
OMIT = {"fonts-lato", "fonts-open-sans", "pocketsphinx-en-us", "spirv-headers",
        "spirv-tools", "zip", "rake"}


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--cache-dir", type=Path,
                        default=Path.home() / ".cache" / "hamr-gazebo-root")
    parser.add_argument("--audit-existing", action="store_true",
                        help="Verify existing DEBs and write provenance/environment; do not download or extract")
    args = parser.parse_args()
    root = args.cache_dir.expanduser().resolve()
    debs = root / "debs"
    debs.mkdir(parents=True, exist_ok=True)
    completed = subprocess.run(
        ["apt-get", "--print-uris", "--yes", "--download-only", "install", *REQUESTED],
        check=True, capture_output=True, text=True,
    )
    packages = []
    for line in completed.stdout.splitlines():
        if not line.startswith(("'http://", "'https://")):
            continue
        uri, filename, size, _legacy_checksum = shlex.split(line)
        name, encoded_version, _arch = filename[:-4].split("_")
        if name.endswith("-dev") or name in OMIT:
            continue
        packages.append({"package": name, "version": urllib.parse.unquote(encoded_version),
                         "uri": uri, "filename": filename, "size": int(size)})
    if not packages:
        print("apt reports no missing packages; use the existing system Gazebo installation.")
        return
    # Get SHA-256 from the local apt metadata, not from the downloaded archive.
    metadata = subprocess.run(
        ["apt-cache", "show", *(f"{p['package']}={p['version']}" for p in packages)],
        check=True, capture_output=True, text=True,
    ).stdout
    records = {}
    for record in metadata.split("\n\n"):
        fields = {}
        for line in record.splitlines():
            if line and not line[0].isspace() and ": " in line:
                key, value = line.split(": ", 1)
                fields[key] = value
        if "Package" in fields and "Version" in fields and "SHA256" in fields:
            records[(fields["Package"], fields["Version"])] = fields
    for package in packages:
        package["sha256"] = records[(package["package"], package["version"])]["SHA256"]
    print(f"{len(packages)} missing runtime archives; "
          f"{sum(p['size'] for p in packages) / 1e6:.1f} MB", flush=True)

    def obtain(package):
        path = debs / package["filename"]
        if not path.exists() or path.stat().st_size != package["size"]:
            if args.audit_existing:
                raise RuntimeError(f"Missing existing archive: {path}")
            # Use precisely the repository URL selected by apt. ROS's configured
            # repository uses HTTP; archive integrity is checked against apt's SHA.
            partial = path.with_suffix(".download")
            urllib.request.urlretrieve(package["uri"], partial)
            partial.replace(path)
        if sha256(path) != package["sha256"]:
            raise RuntimeError(f"SHA-256 does not match apt metadata: {path.name}")
        return path

    with ThreadPoolExecutor(max_workers=8) as workers:
        paths = list(workers.map(obtain, packages))
    if not args.audit_existing:
        for path in paths:
            subprocess.run(["dpkg-deb", "-x", str(path), str(root)], check=True)
    vendor_root = root / "opt/ros/jazzy/opt"
    if not vendor_root.exists():
        raise RuntimeError("No extracted Gazebo vendor prefix found")
    for path in vendor_root.glob("*/share/gz/*.yaml"):
        content = path.read_text()
        relocated = re.sub(r"(?m)^(library_path:\s*)/opt/ros/jazzy/",
                           lambda m: m.group(1) + str(root) + "/opt/ros/jazzy/", content)
        if relocated != content:
            path.write_text(relocated)
    template = Path(__file__).with_name("rootless_env.bash.in").read_text()
    (root / "env.sh").write_text(template.replace("@CACHE_ROOT@", shlex.quote(str(root))))
    manifest = {
        "method": "apt-get --print-uris; SHA-256 verified against local apt metadata; dpkg-deb -x",
        "requested_packages": REQUESTED,
        "system_package_installation_performed": False,
        "maintainer_scripts_executed": False,
        "host_requirement": "Ubuntu 24.04 amd64, existing ROS Jazzy desktop and its already-installed dependencies",
        "skipped": "Development-only packages, fonts, speech model data, SPIR-V build tools, zip and rake",
        "relocations": ["Gazebo command YAML library_path prefixes", "env.sh search paths"],
        "packages": packages,
    }
    (root / "rootless_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"Ready: source {root / 'env.sh'}", flush=True)
    print("Then: gz sim --versions; gz sdf --versions")


if __name__ == "__main__":
    main()
