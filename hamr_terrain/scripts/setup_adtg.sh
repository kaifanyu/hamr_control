#!/usr/bin/env bash
# Install only the pinned standalone model in a dedicated environment.
set -euo pipefail

commit=262488cd981f616898541198537a4b302fa2922a
repository=https://github.com/youwyu/Adaptive-Diffusion-Terrain.git
cache_dir="${HAMR_ADTG_CACHE:-$HOME/.cache/hamr-adtg}"
python_bin="${HAMR_ADTG_PYTHON:-/usr/bin/python3}"
if [[ $# -gt 0 ]]; then
  if [[ "$1" == "--help" || "$1" == "-h" ]]; then
    echo 'Usage: setup_adtg.sh [CACHE_DIR]'
    echo 'Default: $HAMR_ADTG_CACHE or ~/.cache/hamr-adtg; uses its own venv.'
    exit 0
  fi
  cache_dir="$1"
  shift
fi
if [[ $# -ne 0 ]]; then
  echo 'Usage: setup_adtg.sh [CACHE_DIR]' >&2
  exit 2
fi
mkdir -p "$cache_dir"
cache_dir="$(cd "$cache_dir" && pwd)"
source_dir="$cache_dir/source"

if [[ ! -e "$source_dir" ]]; then
  # Fetch only commit/tree metadata first; sparse checkout downloads standalone
  # files and the 45 MB ONNX model, avoiding the elevation training dataset.
  git init -q "$source_dir"
  git -C "$source_dir" remote add origin "$repository"
  git -C "$source_dir" config remote.origin.promisor true
  git -C "$source_dir" config remote.origin.partialclonefilter blob:none
  git -C "$source_dir" sparse-checkout init --cone
  git -C "$source_dir" sparse-checkout set contexts/standalone
  git -C "$source_dir" fetch --depth 1 --filter=blob:none origin "$commit"
  git -C "$source_dir" checkout --detach FETCH_HEAD
fi

if [[ ! -d "$source_dir/.git" ]]; then
  echo "Refusing to replace existing non-repository path: $source_dir" >&2
  exit 1
fi
if [[ "$(git -C "$source_dir" rev-parse HEAD)" != "$commit" ]]; then
  echo "Existing checkout is not the pinned commit; choose another cache directory." >&2
  exit 1
fi
if [[ -n "$(git -C "$source_dir" status --porcelain --untracked-files=normal -- contexts/standalone)" ]]; then
  echo "Existing standalone source has modifications; refusing to replace or install it." >&2
  exit 1
fi

package_dir="$source_dir/contexts/standalone"
if [[ ! -f "$package_dir/pyproject.toml" ]]; then
  echo "Pinned checkout lacks standalone files; choose another cache directory." >&2
  exit 1
fi
"$python_bin" - "$package_dir" <<'PY'
import hashlib
from pathlib import Path
import sys

if sys.version_info < (3, 10):
    raise SystemExit('Python 3.10 or newer is required')
package = Path(sys.argv[1])
expected = {
    'pyproject.toml': '55c1ecd4dd58e3e2c69674028da29c055b3207d722204c7d1b51765fb67b84b6',
    'terrain_diffusion/weights/terrain_consistency_unet_difficulty100.onnx':
        '95be54b4a39d58320e187a7ec533351c957f16a00be58773c86074828ee67887',
}
for relative, digest in expected.items():
    if hashlib.sha256((package/relative).read_bytes()).hexdigest() != digest:
        raise SystemExit(f'Official source/checkpoint hash mismatch: {relative}')
PY

venv_dir="$cache_dir/venv"
if [[ ! -e "$venv_dir" ]]; then
  "$python_bin" -m venv "$venv_dir"
fi
if [[ ! -f "$venv_dir/pyvenv.cfg" || ! -x "$venv_dir/bin/python" ]]; then
  echo "Refusing to use an invalid existing virtual environment: $venv_dir" >&2
  exit 1
fi
venv_python="$venv_dir/bin/python"
"$venv_python" -I -m pip --isolated install --require-virtualenv \
  pip==24.0 setuptools==80.9.0 wheel==0.45.1 \
  numpy==2.2.6 onnxruntime==1.22.1 coloredlogs==15.0.1 \
  flatbuffers==25.12.19 packaging==26.3 protobuf==7.36.2 \
  sympy==1.14.0 humanfriendly==10.0 mpmath==1.3.0
# The generator explicitly supports CPU ONNX Runtime. Its Linux package
# metadata defaults to the GPU distribution; do not install that dependency.
"$venv_python" -I -m pip --isolated install --require-virtualenv \
  --no-deps --no-build-isolation "$package_dir"
"$venv_python" -I -m pip freeze > "$cache_dir/requirements.lock.txt"
echo "Pinned standalone generator ready: $venv_python"
echo 'Run sample_adtg.py --output-dir NEW_DIRECTORY; it uses this isolated environment.'
