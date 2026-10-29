#!/usr/bin/env bash
# Render an existing encoder recording with the available CENTER reference.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
Usage: bash render_caster_simulation.sh RECORDING_FOLDER [OUTPUT_FOLDER] [OPTIONS]

Render the caster animation from a downloaded encoder_<run> folder containing
packets.jsonl. Front-right automatically uses its saved absolute CENTER reference
and measured starting count; H1/H2 retain recorded relative shell angles.
Without a saved reference, starts with H1 forward (+X), H2 back (-X) and both
hemispheres at floor level; CENTER is reversed and the first fresh pose is zeroed.

The video has no angle plots. Defaults: 1x speed, 30 fps, 1600x900.
Output defaults to a new timestamped folder under ~/Videos/hamr_sim/.
An explicit output folder must be empty or absent.

Options:
  --wheel front_left|front_right  Required if the recording has both wheels
  --center-reference auto|none|PATH  Saved CENTER mapping (default: auto)
  --rate NUMBER                  Playback speed (default: 1)
  --fps NUMBER                   Frames per second (default: 30)
  --width NUMBER                 Width >=800, divisible by 32 (default: 1600)
  --full-detail                  Include internal bearing and fastener meshes
  --preview-time SECONDS         Write a preview PNG instead of an MP4
  -h, --help                     Show this help

Uses render_caster_recording.sh and its Ubuntu/ROS rendering dependencies.
The no-plots mode is fixed by this preset. A saved CENTER reference takes
precedence over its otherwise assumed ground-contact start and CENTER direction.
See docs/LOCAL_CASTER_RECORDING.md for dependency installation.

Example (from the repository root):
  bash hamr_bringup/scripts/render_caster_simulation.sh "/path/to/encoder_trial_002"
EOF
}

if (( $# == 0 )); then
  usage >&2
  exit 2
fi
for argument in "$@"; do
  if [[ "$argument" == --help || "$argument" == -h ]]; then
    usage
    exit 0
  fi
done

# Locate the mapping in a source checkout or a CMake-installed ROS package.
mapping_config="$script_dir/../config/caster_encoder_replay_simulation.yaml"
if [[ ! -f "$mapping_config" ]]; then
  mapping_config="$script_dir/../../share/hamr_bringup/config/caster_encoder_replay_simulation.yaml"
fi

# Apply the negative CENTER sign before ground-contact computes its reference
# zero, keeping the initial alignment while reversing subsequent rotation.
exec bash "$script_dir/render_caster_recording.sh" "$@" \
  --no-plots --start-pose ground-contact --mapping-config "$mapping_config"
