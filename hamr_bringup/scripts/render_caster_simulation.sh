#!/usr/bin/env bash
# Render an existing encoder recording with a fixed upright starting pose.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat <<'EOF'
Usage: bash render_caster_simulation.sh RECORDING_FOLDER [OUTPUT_FOLDER] [OPTIONS]

Render the caster animation from a downloaded encoder_<run> folder containing
packets.jsonl. Starts with the fork vertical, H1 forward (+X), H2 back (-X),
and both hemispheres at floor level. Subsequent recorded angle changes are kept.
The first complete fresh encoder pose defines this assumed starting reference.

The video has no angle plots. Defaults: 1x speed, 30 fps, 1600x900.
Output defaults to a new timestamped folder under ~/Videos/hamr_sim/.
An explicit output folder must be empty or absent.

Options:
  --wheel front_left|front_right  Required if the recording has both wheels
  --rate NUMBER                  Playback speed (default: 1)
  --fps NUMBER                   Frames per second (default: 30)
  --width NUMBER                 Width >=800, divisible by 32 (default: 1600)
  --full-detail                  Include internal bearing and fastener meshes
  --preview-time SECONDS         Write a preview PNG instead of an MP4
  -h, --help                     Show this help

Uses render_caster_recording.sh and its Ubuntu/ROS rendering dependencies.
The ground-contact starting pose and no-plots mode are fixed by this preset.
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

# Append the preset so it cannot accidentally inherit a recorded absolute pose
# or enable plots. All path handling, validation and rendering stay shared.
exec bash "$script_dir/render_caster_recording.sh" "$@" \
  --no-plots --start-pose ground-contact
