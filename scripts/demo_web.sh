#!/usr/bin/env bash
# Current local Docker fixture only. Does not contact the robot or start real drivers.
set -euo pipefail
PKG="$(cd "$(dirname "$0")/.." && pwd)"
BOX="${OUTDOOR_TEST_CONTAINER:-outdoor-service-test}"
docker inspect "$BOX" >/dev/null
mount=$(docker inspect --format '{{range .Mounts}}{{if eq .Destination "/service"}}{{.Source}}{{end}}{{end}}' "$BOX")
[[ "$mount" == "$PKG" ]] || { echo 'Container /service does not point at this checkout' >&2; exit 2; }
owner_action() {
  docker exec -i "$BOX" python3 - "$1" <<'PYCODE'
from pathlib import Path
import os,signal,sys,time
file=Path('/service/rbnx-build/manual.pid')
if not file.is_file():raise SystemExit(0)
pid=int(file.read_text());proc=Path(f'/proc/{pid}')
try:
    if (proc/'stat').read_text().split()[2]=='Z':raise SystemExit(0)
    cmd=(proc/'cmdline').read_bytes()
    if b'run_isolated_flow.py' not in cmd or b'--manual' not in cmd:
        raise SystemExit('PID is not the manual demo; refusing to signal')
    if sys.argv[1]=='check':
        raise SystemExit('Demo already running: http://127.0.0.1:19865/ (use demo_web.sh stop first)')
    os.kill(pid,signal.SIGTERM)
    for _ in range(400):
        try:
            if (proc/'stat').read_text().split()[2]=='Z':break
        except FileNotFoundError:break
        time.sleep(.1)
    else:raise SystemExit('Demo stop timed out; inspect its run log')
except FileNotFoundError:pass
PYCODE
}
if [[ "${1:-start}" == stop ]]; then owner_action stop; exit; fi
[[ "${1:-start}" == start ]] || { echo 'Usage: demo_web.sh [start|stop]' >&2; exit 2; }
mkdir -p "$PKG/rbnx-build"
exec 9>"$PKG/rbnx-build/manual.lock"
flock -n 9 || { echo 'Demo already running: http://127.0.0.1:19865/' >&2; exit 2; }
owner_action check
trap 'owner_action stop || true' INT TERM EXIT
# Long-running: leave this terminal open; use the browser independently.
docker exec "$BOX" bash -lc '
source /opt/ros/humble/setup.bash
source /baseline/.ros-install/setup.bash
export PYTHONPATH=/service:/service/rbnx-build/codegen/proto_gen:/service/rbnx-build/codegen/robonix_mcp_types:/baseline:/baseline/ros_ws/src/outdoornav_ros:/baseline/.venv/lib/python3.10/site-packages:${PYTHONPATH:-}
export AMENT_PREFIX_PATH=/tmp/outdoor-extra-prefix:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/tmp/outdoor-extra-prefix/lib:${LD_LIBRARY_PATH:-}
export OUTDOOR_SERVICE_PYTHON=/tmp/service-sdk-venv/bin/python
exec /baseline/.venv/bin/python /service/tests/run_isolated_flow.py --manual
' &
child=$!
wait "$child"
