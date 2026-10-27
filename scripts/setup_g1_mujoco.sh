#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
export PYTHONNOUSERSITE=1
proxy_args=()
if [[ -n "${G1_PIP_PROXY:-}" ]]; then proxy_args=(--proxy "$G1_PIP_PROXY"); fi
bash scripts/create_g1_venv.sh .venv-g1-mujoco
.venv-g1-mujoco/bin/python -m pip install -r requirements-g1-mujoco.txt "${proxy_args[@]}"
