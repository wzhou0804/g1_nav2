#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
if [[ "${1:-}" == --stop ]]; then
    pid_file=.cache-g1/student-deploy/controller.pid
    if [[ ! -f "$pid_file" ]]; then echo 'No Student process recorded.'; exit 0; fi
    student_pid="$(cat "$pid_file")"
    if [[ ! "$student_pid" =~ ^[1-9][0-9]*$ ]]; then echo 'Invalid Student PID record.' >&2; exit 1; fi
    if [[ -f "/proc/$student_pid/cmdline" ]] &&
       [[ "$(readlink "/proc/$student_pid/cwd")" == "$PWD" ]] &&
       tr '\0' ' ' < "/proc/$student_pid/cmdline" | grep -q 'scripts/deploy_g1_student.py'; then
        kill -TERM "$student_pid"
        echo 'Stop requested; the controller will exit through its damping path if LowCmd is active.'
    else
        echo 'Recorded PID is not this workspace Student controller; no signal sent.'
    fi
    exit 0
fi
if [[ ! -x .venv-g1-deploy/bin/python ]]; then
    echo 'Missing .venv-g1-deploy; run scripts/setup_g1_homie.sh for OpenHomie.' >&2
    exit 1
fi
# SDK CycloneDDS 0.10.2 and CPU Python must not inherit ROS/Isaac paths.
unset PYTHONPATH LD_LIBRARY_PATH CYCLONEDDS_URI AMENT_PREFIX_PATH COLCON_PREFIX_PATH
unset CMAKE_PREFIX_PATH ROS_DISTRO RMW_IMPLEMENTATION
export PYTHONNOUSERSITE=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
exec .venv-g1-deploy/bin/python -u scripts/deploy_g1_student.py "$@"
