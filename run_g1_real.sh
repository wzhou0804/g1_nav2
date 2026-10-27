#!/usr/bin/env bash
set -e
cd "$(dirname "$0")"
mkdir -p .cache-g1/real-runtime
if [[ "${1:-}" == '--stop' ]]; then
    exec /usr/bin/python3 - <<'PY'
import os, signal
from pathlib import Path
root = Path.cwd()
file = root / '.cache-g1/real-runtime/launch.pid'
if not file.exists():
    raise SystemExit('No recorded G1 launch PID.')
pid = int(file.read_text().strip())
process = Path('/proc') / str(pid)
if not process.exists():
    raise SystemExit('Recorded launch has already exited.')
args = (process / 'cmdline').read_bytes().split(b'\0')
if ((process / 'cwd').resolve() != root or b'launch' not in args
        or b'navigation/g1_real_navigation.launch.py' not in args):
    raise SystemExit('PID does not identify this workspace navigation launch; left untouched.')
os.kill(pid, signal.SIGINT)
print('Stopping this G1 navigation launch. Original robot controller remains running.')
PY
fi
# The lock stays open across exec and is released when this launch exits.
exec 9>.cache-g1/real-runtime/launch.lock
if ! flock -n 9; then
    echo 'G1 real navigation is already running from this workspace.' >&2
    exit 1
fi
source scripts/activate_g1_real.sh
if ! address_lines=$(ip -o -4 address show); then
    echo 'Unable to read local IPv4 addresses with ip; navigation was not started.' >&2
    exit 1
fi
if ! robot_address_lines=$(awk -v expected="$G1_ROBOT_ADDRESS" '
    $3 == "inet" {
        split($4, address, "/")
        if (address[1] == expected) { print; found = 1 }
    }
    END { exit !found }
' <<< "$address_lines"); then
    echo "Robot subnet address $G1_ROBOT_ADDRESS is missing. Add it to the robot's current wired interface:" >&2
    echo "  nmcli device modify <robot-interface> +ipv4.addresses $G1_ROBOT_ADDRESS/24" >&2
    ip -brief -4 address show >&2
    echo 'See docs/G1_REAL_NAVIGATION_ZH.md for the mapping setup.' >&2
    exit 1
fi
printf '%s\n' "$robot_address_lines"
printf '%s\n' "$$" > .cache-g1/real-runtime/launch.pid
exec ros2 launch navigation/g1_real_navigation.launch.py "$@"
