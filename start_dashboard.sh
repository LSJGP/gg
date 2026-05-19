#!/usr/bin/env bash
# Start the Hywgrading batch-sim web dashboard (stdlib HTTP server).
#
# Usage:
#   ./start_dashboard.sh
#   ./start_dashboard.sh --port 8765
#   HOST=0.0.0.0 ./start_dashboard.sh --port 9000
#
# Open http://127.0.0.1:8765/ in a browser after startup.

set -euo pipefail

ROOT="$(cd "$(dirname "$0")" && pwd)"
cd "$ROOT"

HOST="${HOST:-127.0.0.1}"
PORT="${PORT:-8765}"

mkdir -p output/batch output/log output/report output/viz

GRADING_BIN="${ROOT}/grading_mini/bazel-bin/src/entry/grading_main"
if [[ ! -x "${GRADING_BIN}" ]]; then
  echo "[dashboard] note: grading_main not built yet."
  echo "[dashboard]   cd grading_mini && bazel build //src/entry:grading_main"
fi

SIM_RUNNER="${ROOT}/sim/bazel-bin/cpp/sim_runner"
if [[ ! -x "${SIM_RUNNER}" ]]; then
  echo "[dashboard] note: sim_runner not built yet."
  echo "[dashboard]   cd sim && bazel build //cpp:sim_runner"
fi

echo "[dashboard] repo: ${ROOT}"
echo "[dashboard] url:  http://${HOST}:${PORT}/"
echo "[dashboard] stop: Ctrl+C"
echo ""

args=(--host "${HOST}" --port "${PORT}")
if [[ $# -gt 0 ]]; then
  args=("$@")
fi
exec python3 "${ROOT}/web/server.py" "${args[@]}"
