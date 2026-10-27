#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
unset PYTHONPATH LD_LIBRARY_PATH AMENT_PREFIX_PATH COLCON_PREFIX_PATH
export PYTHONNOUSERSITE=1
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export OMP_NUM_THREADS=1
if [[ ! -x .venv-g1-mujoco/bin/python ]]; then
    echo 'Run scripts/setup_g1_mujoco.sh first.' >&2
    exit 1
fi
exec .venv-g1-mujoco/bin/python -u scripts/benchmark_g1_mujoco.py "$@"
