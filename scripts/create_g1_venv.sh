#!/usr/bin/env bash
# Bootstrap a separate Python 3.10 environment using system pip, never Isaac Sim.
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
target="${1:?Usage: create_g1_venv.sh .venv-g1-name}"
if [[ ! -x "$target/bin/python" ]]; then
    /usr/bin/python3 -m venv --without-pip "$target"
fi
if ! "$target/bin/python" -m pip --version >/dev/null 2>&1; then
    mkdir -p .cache-g1/pip-bootstrap
    proxy_args=()
    if [[ -n "${G1_PIP_PROXY:-}" ]]; then proxy_args=(--proxy "$G1_PIP_PROXY"); fi
    /usr/bin/python3 -m pip download --no-deps --only-binary=:all: \
        --dest .cache-g1/pip-bootstrap pip==25.2 "${proxy_args[@]}"
    wheel="$PWD/.cache-g1/pip-bootstrap/pip-25.2-py3-none-any.whl"
    PYTHONPATH="$wheel" "$target/bin/python" -m pip install --no-deps --no-index "$wheel"
fi
