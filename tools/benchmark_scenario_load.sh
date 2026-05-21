#!/usr/bin/env bash
# Compare bulk vs stream scenario loading for sim_runner.
#
# Prerequisite:
#   python3 tools/split_existing_dynamic_objects.py <scenario_dir>
#
# Usage:
#   ./tools/benchmark_scenario_load.sh scenarios/waymo_scenario_6
#   RUNS=3 MAX_SECONDS=9.0 ./tools/benchmark_scenario_load.sh scenarios/waymo_scenario_6

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SCENARIO_DIR="${1:-}"
if [[ -z "$SCENARIO_DIR" ]]; then
  echo "usage: $0 <scenario_dir>" >&2
  exit 1
fi
if [[ "$SCENARIO_DIR" != /* ]]; then
  SCENARIO_DIR="$ROOT/$SCENARIO_DIR"
fi
SCENARIO_DIR="$(cd "$SCENARIO_DIR" && pwd)"

FRAMES_DIR="$SCENARIO_DIR/dynamic_objects/frames"
if [[ ! -d "$FRAMES_DIR" ]]; then
  echo "[bench] missing $FRAMES_DIR" >&2
  echo "[bench] run: python3 tools/split_existing_dynamic_objects.py $SCENARIO_DIR" >&2
  exit 1
fi

RUNS="${RUNS:-5}"
MAX_SECONDS="${MAX_SECONDS:-9.0}"
DT="${DT:-0.1}"
SIM_DIR="$ROOT/sim"

median_of() {
  python3 - "$@" <<'PY'
import json
import sys

def median(vals):
    s = sorted(vals)
    n = len(s)
    if n == 0:
        return 0.0
    m = n // 2
    if n % 2:
        return s[m]
    return 0.5 * (s[m - 1] + s[m])

key = sys.argv[1]
rows = [json.loads(line) for line in sys.stdin if line.strip().startswith("{")]
vals = [float(r[key]) for r in rows if key in r]
print(f"{median(vals):.3f}")
PY
}

run_mode() {
  local mode="$1"
  local -a samples=()
  local i
  for ((i = 0; i < RUNS; i++)); do
    local out
    out="$(
      cd "$SIM_DIR"
      bazel run //cpp:sim_runner -- \
        --scenario-dir "$SCENARIO_DIR" \
        --scenario-load "$mode" \
        --benchmark \
        --max-seconds "$MAX_SECONDS" \
        --dt "$DT" \
        2>/dev/null | tail -n 1
    )"
    samples+=("$out")
    echo "$out"
  done
  printf '%s\n' "${samples[@]}"
}

echo "[bench] scenario=$SCENARIO_DIR runs=$RUNS max_seconds=$MAX_SECONDS"
echo "[bench] building sim_runner..."
(cd "$SIM_DIR" && bazel build //cpp:sim_runner >/dev/null)

echo "[bench] mode=bulk"
bulk_lines="$(run_mode bulk)"
echo "[bench] mode=stream"
stream_lines="$(run_mode stream)"

report() {
  local label="$1"
  local lines="$2"
  echo "[bench] --- $label (median ms) ---"
  printf '%s\n' "$lines" | median_of load_meta_map_ms | xargs -I{} echo "  load_meta_map_ms: {}"
  printf '%s\n' "$lines" | median_of load_dynamic_ms | xargs -I{} echo "  load_dynamic_ms: {}"
  printf '%s\n' "$lines" | median_of load_frames_ms | xargs -I{} echo "  load_frames_ms: {}"
  printf '%s\n' "$lines" | median_of sim_loop_ms | xargs -I{} echo "  sim_loop_ms: {}"
  printf '%s\n' "$lines" | median_of total_ms | xargs -I{} echo "  total_ms: {}"
}

report bulk "$bulk_lines"
report stream "$stream_lines"

OUT_DIR="$ROOT/output/benchmark"
mkdir -p "$OUT_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
BASE="$(basename "$SCENARIO_DIR")"
{
  echo "# benchmark $BASE $STAMP"
  echo "runs=$RUNS max_seconds=$MAX_SECONDS"
  echo "## bulk"
  report bulk "$bulk_lines"
  echo "## stream"
  report stream "$stream_lines"
} | tee "$OUT_DIR/${BASE}_${STAMP}.txt"

echo "[bench] wrote $OUT_DIR/${BASE}_${STAMP}.txt"
