#!/usr/bin/env bash
# OFFLINE solver only. No apt, USB forwarding, Pi access or motor operations.
set -euo pipefail
if [[ "$(uname -s)" != Linux || "$(uname -m)" != x86_64 ]]; then
  echo 'Phase 0 setup requires Linux x86-64.' >&2
  exit 1
fi
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
phase0_root="${PHASE0_ROOT:-$HOME/.local/share/lerobot-phase0}"
uv="$phase0_root/bootstrap/bin/uv"
if [[ ! -x "$uv" ]]; then
  python3 -m pip install --target "$phase0_root/bootstrap" 'uv==0.9.8'
fi
export UV_PYTHON_INSTALL_DIR="$phase0_root/python"
export UV_CACHE_DIR="$phase0_root/cache"
"$uv" python install 3.12.12 --no-bin
if [[ ! -x "$phase0_root/venv/bin/python" ]]; then
  "$uv" venv --python 3.12.12 "$phase0_root/venv"
fi
requirements="$repo/requirements-phase0-wsl.lock.txt"
if [[ ! -f "$requirements" ]]; then
  requirements="$repo/requirements-phase0-wsl.txt"
fi
"$uv" pip install --python "$phase0_root/venv/bin/python" \
  --index-strategy unsafe-best-match \
  --extra-index-url https://download.pytorch.org/whl/cpu \
  -r "$requirements"
"$uv" pip check --python "$phase0_root/venv/bin/python"
"$phase0_root/venv/bin/python" -c 'import placo, pinocchio, torch; from lerobot.model.kinematics import RobotKinematics; print("OFFLINE imports OK; torch", torch.__version__, "CUDA build", torch.version.cuda)'
echo "Phase 0 interpreter: $phase0_root/venv/bin/python"
