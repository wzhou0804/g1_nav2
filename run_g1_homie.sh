#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
if [[ "${1:-}" == --stop ]]; then exec ./run_g1_student.sh --stop; fi
exec ./run_g1_student.sh --policy homie --output .cache-g1/homie-deploy/current "$@"
