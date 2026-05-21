#!/usr/bin/env bash
# Compare bulk vs stream scenario loading; json vs proto input.
#
# Prerequisite (stream):
#   python3 tools/split_existing_dynamic_objects.py --format <json|proto|both> <scenario_dir>
# Proto scenarios:
#   tools/.venv/bin/python tools/json_scenario_to_proto.py <dir> --split-dynamic-frames
#
# Usage:
#   ./tools/benchmark_scenario_load.sh scenarios/waymo_scenario_6
#   INPUT_FORMAT=proto RUNS=3 ./tools/benchmark_scenario_load.sh scenarios/waymo_scenario_6
#   COMPARE_FORMATS=1 ./tools/benchmark_scenario_load.sh scenarios/waymo_scenario_6

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
  echo "[bench] run: python3 tools/split_existing_dynamic_objects.py --format both $SCENARIO_DIR" >&2
  exit 1
fi

RUNS="${RUNS:-5}"
MAX_SECONDS="${MAX_SECONDS:-9.0}"
DT="${DT:-0.1}"
WITH_RSS="${WITH_RSS:-0}"
INPUT_FORMAT="${INPUT_FORMAT:-json}"
COMPARE_FORMATS="${COMPARE_FORMATS:-0}"
SIM_DIR="$ROOT/sim"
OUT_DIR="$ROOT/output/benchmark"
mkdir -p "$OUT_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
BASE="$(basename "$SCENARIO_DIR")"

echo "[bench] scenario=$SCENARIO_DIR runs=$RUNS max_seconds=$MAX_SECONDS dt=$DT"
echo "[bench] building sim_runner..."
(cd "$SIM_DIR" && bazel build //cpp:sim_runner >/dev/null)

run_mode() {
  local mode="$1"
  local fmt="$2"
  local out_file="$3"
  : >"$out_file"
  local i
  for ((i = 1; i <= RUNS; i++)); do
    echo "[bench] ${fmt}/${mode} run $i/$RUNS ..."
    local json_line rss_kb=""
    if [[ "$WITH_RSS" == "1" ]]; then
      local time_out
      time_out="$(mktemp)"
      /usr/bin/time -f "RSS_KB:%M" -o "$time_out" \
        bash -c "cd '$SIM_DIR' && bazel run //cpp:sim_runner -- \
          --scenario-dir '$SCENARIO_DIR' \
          --scenario-load '$mode' \
          --input-format '$fmt' \
          --benchmark \
          --max-seconds '$MAX_SECONDS' \
          --dt '$DT' 2>/dev/null | tail -n 1" || true
      rss_kb="$(grep -o 'RSS_KB:[0-9]*' "$time_out" | cut -d: -f2 || true)"
      rm -f "$time_out"
      json_line="$(cd "$SIM_DIR" && bazel run //cpp:sim_runner -- \
        --scenario-dir "$SCENARIO_DIR" \
        --scenario-load "$mode" \
        --input-format "$fmt" \
        --benchmark \
        --max-seconds "$MAX_SECONDS" \
        --dt "$DT" 2>/dev/null | tail -n 1)"
    else
      json_line="$(cd "$SIM_DIR" && bazel run //cpp:sim_runner -- \
        --scenario-dir "$SCENARIO_DIR" \
        --scenario-load "$mode" \
        --input-format "$fmt" \
        --benchmark \
        --max-seconds "$MAX_SECONDS" \
        --dt "$DT" 2>/dev/null | tail -n 1)"
    fi
    if [[ -n "$rss_kb" ]]; then
      json_line="$(python3 -c "import json,sys; d=json.loads(sys.argv[1]); d['peak_rss_kb']=int(sys.argv[2]); print(json.dumps(d))" "$json_line" "$rss_kb")"
    fi
    echo "$json_line" >>"$out_file"
  done
}

if [[ "$COMPARE_FORMATS" == "1" ]]; then
  RAW_JSON_BULK="$OUT_DIR/${BASE}_${STAMP}_json_bulk.jsonl"
  RAW_JSON_STREAM="$OUT_DIR/${BASE}_${STAMP}_json_stream.jsonl"
  RAW_PROTO_BULK="$OUT_DIR/${BASE}_${STAMP}_proto_bulk.jsonl"
  RAW_PROTO_STREAM="$OUT_DIR/${BASE}_${STAMP}_proto_stream.jsonl"
  REPORT_MD="$OUT_DIR/${BASE}_${STAMP}_compare_report.md"
  run_mode bulk json "$RAW_JSON_BULK"
  run_mode stream json "$RAW_JSON_STREAM"
  run_mode bulk proto "$RAW_PROTO_BULK"
  run_mode stream proto "$RAW_PROTO_STREAM"
  python3 - "$REPORT_MD" "$RAW_JSON_BULK" "$RAW_JSON_STREAM" "$RAW_PROTO_BULK" "$RAW_PROTO_STREAM" "$SCENARIO_DIR" "$RUNS" "$MAX_SECONDS" "$DT" <<'PY'
import json
import statistics
import sys
from datetime import datetime
from pathlib import Path

report_path = Path(sys.argv[1])
datasets = [
    ("json", "bulk", Path(sys.argv[2])),
    ("json", "stream", Path(sys.argv[3])),
    ("proto", "bulk", Path(sys.argv[4])),
    ("proto", "stream", Path(sys.argv[5])),
]
scenario_dir = sys.argv[6]
runs = int(sys.argv[7])
max_seconds = float(sys.argv[8])
dt = float(sys.argv[9])
METRICS = [
    ("load_meta_map_ms", "Meta + map"),
    ("load_dynamic_ms", "Dynamic load"),
    ("load_frames_ms", "Frame I/O"),
    ("sim_loop_ms", "Sim loop"),
    ("total_ms", "Total"),
]


def load_rows(path: Path) -> list[dict]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line.startswith("{"):
            rows.append(json.loads(line))
    return rows


def median(vals: list[float]) -> float:
    return statistics.median(vals) if vals else 0.0


lines = [
    f"# Scenario load benchmark — {Path(scenario_dir).name}",
    "",
    f"- Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
    f"- Runs: {runs}, max_seconds={max_seconds}, dt={dt}",
    "",
    "## Median timing (ms)",
    "",
    "| input_format | scenario_load | total | load_meta_map | load_dynamic | load_frames | sim_loop |",
    "|---|---|---:|---:|---:|---:|---:|",
]
for fmt, mode, path in datasets:
    rows = load_rows(path)
    lines.append(
        f"| {fmt} | {mode} "
        f"| {median([r['total_ms'] for r in rows]):.2f} "
        f"| {median([r['load_meta_map_ms'] for r in rows]):.2f} "
        f"| {median([r['load_dynamic_ms'] for r in rows]):.2f} "
        f"| {median([r['load_frames_ms'] for r in rows]):.2f} "
        f"| {median([r['sim_loop_ms'] for r in rows]):.2f} |"
    )
lines.append("")
report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(report_path.read_text(encoding="utf-8"))
PY
  echo "[bench] report: $REPORT_MD"
  exit 0
fi

RAW_BULK="$OUT_DIR/${BASE}_${STAMP}_${INPUT_FORMAT}_bulk.jsonl"
RAW_STREAM="$OUT_DIR/${BASE}_${STAMP}_${INPUT_FORMAT}_stream.jsonl"
REPORT_MD="$OUT_DIR/${BASE}_${STAMP}_${INPUT_FORMAT}_report.md"

run_mode bulk "$INPUT_FORMAT" "$RAW_BULK"
run_mode stream "$INPUT_FORMAT" "$RAW_STREAM"

python3 - "$REPORT_MD" "$RAW_BULK" "$RAW_STREAM" "$SCENARIO_DIR" "$RUNS" "$MAX_SECONDS" "$DT" "$INPUT_FORMAT" <<'PY'
import json
import statistics
import sys
from datetime import datetime
from pathlib import Path

report_path = Path(sys.argv[1])
bulk_path = Path(sys.argv[2])
stream_path = Path(sys.argv[3])
scenario_dir = sys.argv[4]
runs = int(sys.argv[5])
max_seconds = float(sys.argv[6])
dt = float(sys.argv[7])
input_format = sys.argv[8]

METRICS = [
    ("load_meta_map_ms", "Meta + lane_graph 加载"),
    ("load_dynamic_ms", "Dynamic 启动加载"),
    ("load_frames_ms", "仿真中逐帧读盘 (stream)"),
    ("sim_loop_ms", "仿真主循环"),
    ("total_ms", "端到端总耗时"),
]


def load_rows(path: Path) -> list[dict]:
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line.startswith("{"):
            rows.append(json.loads(line))
    return rows


def stats(vals: list[float]) -> dict:
    if not vals:
        return {"median": 0, "min": 0, "max": 0, "mean": 0, "stdev": 0}
    return {
        "median": statistics.median(vals),
        "min": min(vals),
        "max": max(vals),
        "mean": statistics.mean(vals),
        "stdev": statistics.stdev(vals) if len(vals) > 1 else 0.0,
    }


def fmt_ms(v: float) -> str:
    return f"{v:.2f}"


def pct_delta(bulk_m: float, stream_m: float) -> str:
    if bulk_m < 1e-6:
        return "n/a"
    d = (stream_m - bulk_m) / bulk_m * 100.0
    sign = "+" if d >= 0 else ""
    return f"{sign}{d:.1f}%"


bulk = load_rows(bulk_path)
stream = load_rows(stream_path)
lines: list[str] = []

lines.append(f"# Scenario 加载性能报告 — {Path(scenario_dir).name}")
lines.append("")
lines.append(f"- 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
lines.append(f"- 场景目录: `{scenario_dir}`")
lines.append(f"- input_format: **{input_format}**")
lines.append(f"- 每组运行次数: **{runs}**（取中位数为主指标）")
lines.append(f"- 仿真参数: `max_seconds={max_seconds}`, `dt={dt}`, `--benchmark`")
lines.append("")

dyn_json = Path(scenario_dir) / "dynamic_objects.json"
dyn_pb = Path(scenario_dir) / "dynamic_objects.pb"
lane_json = Path(scenario_dir) / "lane_graph.json"
lane_pb = Path(scenario_dir) / "lane_graph.pb"
frames_dir = Path(scenario_dir) / "dynamic_objects" / "frames"
lines.append("## 数据规模")
lines.append("")
if dyn_json.is_file():
    lines.append(f"- `dynamic_objects.json`: {dyn_json.stat().st_size / 1024:.1f} KiB")
if dyn_pb.is_file():
    lines.append(f"- `dynamic_objects.pb`: {dyn_pb.stat().st_size / 1024:.1f} KiB")
if lane_json.is_file():
    lines.append(f"- `lane_graph.json`: {lane_json.stat().st_size / 1024:.1f} KiB")
if lane_pb.is_file():
    lines.append(f"- `lane_graph.pb`: {lane_pb.stat().st_size / 1024:.1f} KiB")
if frames_dir.is_dir():
    n_json = len(list(frames_dir.glob("*.json")))
    n_pb = len(list(frames_dir.glob("*.pb")))
    if n_json:
        total = sum(f.stat().st_size for f in frames_dir.glob("*.json"))
        lines.append(f"- `frames/*.json`: {n_json} files, {total / 1024:.1f} KiB")
    if n_pb:
        total = sum(f.stat().st_size for f in frames_dir.glob("*.pb"))
        lines.append(f"- `frames/*.pb`: {n_pb} files, {total / 1024:.1f} KiB")
lines.append("")

for label, rows in [("bulk", bulk), ("stream", stream)]:
    lines.append(f"## 原始数据 — `{label}`")
    lines.append("")
    header = "| Run | load_meta_map | load_dynamic | load_frames | sim_loop | total | frames |"
    sep = "|---|---:|---:|---:|---:|---:|---:|"
    if rows and "peak_rss_kb" in rows[0]:
        header += " RSS(KiB) |"
        sep += "---:|"
    lines.append(header)
    lines.append(sep)
    for i, r in enumerate(rows, 1):
        row = (
            f"| {i} "
            f"| {fmt_ms(r.get('load_meta_map_ms', 0))} "
            f"| {fmt_ms(r.get('load_dynamic_ms', 0))} "
            f"| {fmt_ms(r.get('load_frames_ms', 0))} "
            f"| {fmt_ms(r.get('sim_loop_ms', 0))} "
            f"| {fmt_ms(r.get('total_ms', 0))} "
            f"| {r.get('frames', '-')} |"
        )
        if "peak_rss_kb" in r:
            row += f" {r['peak_rss_kb'] / 1024:.1f} |"
        lines.append(row)
    lines.append("")

lines.append("## 汇总统计（ms）")
lines.append("")
lines.append("| 指标 | bulk 中位数 | stream 中位数 | stream vs bulk |")
lines.append("|---|---:|---:|---|")

summary = {}
for key, desc in METRICS:
    bv = [float(r[key]) for r in bulk if key in r]
    sv = [float(r[key]) for r in stream if key in r]
    bs, ss = stats(bv), stats(sv)
    summary[key] = (bs, ss)
    lines.append(
        f"| {desc} (`{key}`) "
        f"| **{fmt_ms(bs['median'])}** "
        f"| **{fmt_ms(ss['median'])}** "
        f"| {pct_delta(bs['median'], ss['median'])} |"
    )
lines.append("")

b = summary["total_ms"][0]["median"]
s = summary["total_ms"][1]["median"]
lines.append("## 结论")
lines.append("")
lines.append(f"- input_format=`{input_format}`: bulk median **{fmt_ms(b)}** ms vs stream **{fmt_ms(s)}** ms.")
lines.append("")
lines.append("---")
lines.append(f"*Raw JSONL: `{bulk_path.name}`, `{stream_path.name}`*")

report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(report_path.read_text(encoding="utf-8"))
PY

echo ""
echo "[bench] report: $REPORT_MD"
echo "[bench] raw:    $RAW_BULK"
echo "[bench] raw:    $RAW_STREAM"
