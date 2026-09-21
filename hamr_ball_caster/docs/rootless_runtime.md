# Optional Gazebo runtime without sudo

Commands in this guide assume a checkout of this repository. From any directory
inside that checkout, set these portable paths before running the examples:

```bash
export HAMR_REPO="$(git rev-parse --show-toplevel)"
export HAMR_RUNS="${HAMR_RUNS:-$HOME/Videos/hamr_sim}"
```

Recorded videos and full run reports are external artifacts, not files supplied
by a Git clone. Paths under `${HAMR_RUNS}` identify retained runs or new outputs
created by the recording commands.

The normal setup uses the Ubuntu/ROS packages listed in the main README. A local
fallback was also exercised on this WSL machine: the same official Gazebo
Harmonic packages were downloaded and extracted into
`${HOME}/.cache/hamr-gazebo-root`. No system package installation or package
maintainer script was run.

This fallback **requires an existing Ubuntu 24.04 amd64 ROS 2 Jazzy installation**
at `/opt/ros/jazzy` and its already installed desktop dependencies. It is not a
standalone ROS installer. It relies on current local apt package lists and uses
the versions selected by those lists.

To recreate the fallback on a similarly prepared system:

```bash
cd "${HAMR_REPO}"
/usr/bin/python3 hamr_ball_caster/tools/bootstrap_rootless_gazebo.py
source ~/.cache/hamr-gazebo-root/env.sh
gz sim --versions
gz sdf --versions
```

The script runs `apt-get --print-uris --download-only` to resolve missing runtime
archives; it uses `dpkg-deb -x` to extract them. It verifies each archive's SHA-256
against local apt metadata before extraction. Archive URLs, exact versions and
checksums are retained in the cache's `rootless_manifest.json`; the manifest for
this validation is also checked in as `assets/provenance/rootless_gazebo_packages.json`.
Development packages, font collections and speech models are omitted.

The checked environment now includes 134 missing archives (90.8 MB), including
FFmpeg for the MP4 recording pipeline, Gazebo Sim 8.15.0, SDFormat 14.9.0, and the
DART physics backend. The environment script sets the
Gazebo, Ogre, Ruby, Qt/QML and ROS search paths and relocates Gazebo command YAML
paths within the cache. Source that script **before** the built simulation
workspace's `install/setup.bash`. It sources `/opt/ros/jazzy/setup.bash` itself.

For an existing cache, this command verifies the archives again and refreshes its
manifest/environment without downloading or extracting packages:

```bash
/usr/bin/python3 hamr_ball_caster/tools/bootstrap_rootless_gazebo.py --audit-existing
```

`config/rig_gui.config` opens a close view of the 200 mm caster and load fixture.
`config/compa_gui.config` frames the larger robot. The launch file selects the
matching camera configuration automatically. Gazebo's native rendering output
from this WSL session is saved in `docs/gazebo_rig_verified.png`.

On this machine, Ogre2 rendered successfully using `llvmpipe` software rendering.
The rootless GUI may log that the default installed GUI config cannot be copied
from `/opt/ros/jazzy/opt/gz_sim_vendor`; the explicit package GUI config still loads.
The system-package installation provides that default path. Qt's WorldStats
layout and unsupported antialiasing warnings were also nonfatal in the verified
rendering session.

The rootless cache is local to this machine. The extracted libraries are not
bundled in the ROS package, and other machines should normally follow the main
README's apt installation steps.
