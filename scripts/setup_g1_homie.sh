#!/usr/bin/env bash
# Independent CPU policy environment plus pinned SDK, original weights and model.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
unset PYTHONPATH LD_LIBRARY_PATH CYCLONEDDS_URI AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH ROS_DISTRO RMW_IMPLEMENTATION
export PYTHONNOUSERSITE=1
bash scripts/create_g1_venv.sh .venv-g1-deploy
proxy_args=()
if [[ -n "${G1_PIP_PROXY:-}" ]]; then proxy_args=(--proxy "$G1_PIP_PROXY"); fi
.venv-g1-deploy/bin/python -m pip install -r requirements-g1-deploy.txt "${proxy_args[@]}"
revision=814556d15970dd2ecf1c9984e845ca02ab07e206
sdk=.cache-g1/unitree_sdk2_python
if [[ ! -d "$sdk/.git" ]]; then
    git init "$sdk"
    git -C "$sdk" remote add origin https://github.com/unitreerobotics/unitree_sdk2_python.git
    git -C "$sdk" fetch --depth 1 origin "$revision"
    git -C "$sdk" checkout --detach FETCH_HEAD
fi
if [[ "$(git -C "$sdk" rev-parse HEAD)" != "$revision" ]]; then
    echo "Expected SDK commit $revision; existing checkout left intact." >&2
    exit 1
fi
.venv-g1-deploy/bin/python -m pip install --no-deps -e "$sdk" "${proxy_args[@]}"
.venv-g1-deploy/bin/python scripts/fetch_g1_homie.py
.venv-g1-deploy/bin/python scripts/fetch_g1_locked_model.py
echo 'OpenHomie ready. No robot connection or motor commands were made by this installer.'
