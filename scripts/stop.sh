#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
# Validate the recorded process before signaling it; never pkill generic Nav2 names.
python3 - "$PKG" <<'PY'
import os,signal,sys
from pathlib import Path
root=Path(sys.argv[1]).resolve();file=root/'rbnx-build/bridge.pid'
if not file.exists():raise SystemExit(0)
pid=int(file.read_text());proc=Path('/proc')/str(pid)
try:
    cmd=(proc/'cmdline').read_bytes()
    if b'outdoor_nav.atlas_bridge' not in cmd or (proc/'cwd').resolve()!=root:
        raise SystemExit('Recorded PID does not belong to this package; not stopping it')
    os.kill(pid,signal.SIGTERM)
except FileNotFoundError:pass
PY
