#!/usr/bin/env bash
# Compare bulk vs stream scenario loading; 5 runs (default), median + detailed report.
#
# Prerequisite:
#   python3 tools/split_existing_dynamic_objects.py <scenario_dir>
#
# Usage:
#   ./tools/benchmark_scenario_load.sh scenarios/waymo_scenario_6
#   RUNS=5 MAX_SECONDS=9.0 ./tools/benchmark_scenario_load.sh scenarios/waymo_scenario_6

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
WITH_RSS="${WITH_RSS:-0}"
SIM_DIR="$ROOT/sim"
OUT_DIR="$ROOT/output/benchmark"
mkdir -p "$OUT_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
BASE="$(basename "$SCENARIO_DIR")"
RAW_BULK="$OUT_DIR/${BASE}_${STAMP}_bulk.jsonl"
RAW_STREAM="$OUT_DIR/${BASE}_${STAMP}_stream.jsonl"
REPORT_MD="$OUT_DIR/${BASE}_${STAMP}_report.md"

echo "[bench] scenario=$SCENARIO_DIR runs=$RUNS max_seconds=$MAX_SECONDS dt=$DT"
echo "[bench] building sim_runner..."
(cd "$SIM_DIR" && bazel build //cpp:sim_runner >/dev/null)

run_mode() {
  local mode="$1"
  local out_file="$2"
  : >"$out_file"
  local i
  for ((i = 1; i <= RUNS; i++)); do
    echo "[bench] $mode run $i/$RUNS ..."
    local json_line rss_kb=""
    if [[ "$WITH_RSS" == "1" ]]; then
      local time_out
      time_out="$(mktemp)"
      /usr/bin/time -f "RSS_KB:%M" -o "$time_out" \
        bash -c "cd '$SIM_DIR' && bazel run //cpp:sim_runner -- \
          --scenario-dir '$SCENARIO_DIR' \
          --scenario-load '$mode' \
          --benchmark \
          --max-seconds '$MAX_SECONDS' \
          --dt '$DT' 2>/dev/null | tail -n 1" || true
      rss_kb="$(grep -o 'RSS_KB:[0-9]*' "$time_out" | cut -d: -f2 || true)"
      rm -f "$time_out"
      json_line="$(cd "$SIM_DIR" && bazel run //cpp:sim_runner -- \
        --scenario-dir "$SCENARIO_DIR" \
        --scenario-load "$mode" \
        --benchmark \
        --max-seconds "$MAX_SECONDS" \
        --dt "$DT" 2>/dev/null | tail -n 1)"
    else
      json_line="$(cd "$SIM_DIR" && bazel run //cpp:sim_runner -- \
        --scenario-dir "$SCENARIO_DIR" \
        --scenario-load "$mode" \
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

run_mode bulk "$RAW_BULK"
run_mode stream "$RAW_STREAM"

python3 - "$REPORT_MD" "$RAW_BULK" "$RAW_STREAM" "$SCENARIO_DIR" "$RUNS" "$MAX_SECONDS" "$DT" <<'PY'
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

METRICS = [
    ("load_meta_map_ms", "Meta + lane_graph 加载"),
    ("load_dynamic_ms", "Dynamic 启动加载"),
    ("load_frames_ms", "仿真中逐帧读盘 (stream)"),
    ("sim_loop_ms", "仿真主循环"),
    ("total_ms", "端到端总耗时"),
]
OPTIONAL = ["frames", "peak_rss_kb"]


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
lines.append(f"- 每组运行次数: **{runs}**（取中位数为主指标）")
lines.append(f"- 仿真参数: `max_seconds={max_seconds}`, `dt={dt}`, `--benchmark`（无 grading / 无 sim_log）")
lines.append("")

# File sizes
dyn_json = Path(scenario_dir) / "dynamic_objects.json"
lane_json = Path(scenario_dir) / "lane_graph.json"
frames_dir = Path(scenario_dir) / "dynamic_objects" / "frames"
lines.append("## 数据规模")
lines.append("")
if dyn_json.is_file():
    lines.append(f"- `dynamic_objects.json`: {dyn_json.stat().st_size / 1024:.1f} KiB")
if lane_json.is_file():
    lines.append(f"- `lane_graph.json`: {lane_json.stat().st_size / 1024:.1f} KiB")
if frames_dir.is_dir():
    n_frames = len(list(frames_dir.glob("*.json")))
    total = sum(f.stat().st_size for f in frames_dir.glob("*.json"))
    lines.append(f"- `dynamic_objects/frames/`: {n_frames} 个文件, 合计 {total / 1024:.1f} KiB")
lines.append("")

# Per-run tables
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

# Summary median table
lines.append("## 汇总统计（ms）")
lines.append("")
lines.append("| 指标 | bulk 中位数 | bulk min–max | stream 中位数 | stream min–max | stream vs bulk |")
lines.append("|---|---:|---|---:|---:|---|")

summary = {}
for key, desc in METRICS:
    bv = [float(r[key]) for r in bulk if key in r]
    sv = [float(r[key]) for r in stream if key in r]
    bs, ss = stats(bv), stats(sv)
    summary[key] = (bs, ss)
    lines.append(
        f"| {desc} (`{key}`) "
        f"| **{fmt_ms(bs['median'])}** "
        f"| {fmt_ms(bs['min'])}–{fmt_ms(bs['max'])} "
        f"| **{fmt_ms(ss['median'])}** "
        f"| {fmt_ms(ss['min'])}–{fmt_ms(ss['max'])} "
        f"| {pct_delta(bs['median'], ss['median'])} |"
    )
lines.append("")

# Breakdown interpretation
b = summary["total_ms"][0]["median"]
s = summary["total_ms"][1]["median"]
bd = summary["load_dynamic_ms"][0]["median"]
sd = summary["load_dynamic_ms"][1]["median"]
bf = summary["load_frames_ms"][0]["median"]
sf = summary["load_frames_ms"][1]["median"]
bsim = summary["sim_loop_ms"][0]["median"]
ssim = summary["sim_loop_ms"][1]["median"]

lines.append("## 解读")
lines.append("")
lines.append("1. **启动期 dynamic 加载**：bulk 一次性解析整份 `dynamic_objects.json`；stream 只读 `header.json` + `sdc_states.json`，故 `load_dynamic_ms` 通常更低。")
lines.append("2. **仿真期读帧**：仅 stream 有 `load_frames_ms`（累计逐帧 JSON 读盘 + 解析）；bulk 为 0，NPC 数据已在内存。")
lines.append("3. **仿真主循环**：`sim_loop_ms` 含规划与积分；stream 往往更高，因每步可能触发磁盘读取（即使有 2 帧缓存）。")
lines.append("4. **端到端**：本组 median `total_ms` — bulk **{:.2f}** ms vs stream **{:.2f}** ms。".format(b, s))
if sd < bd:
    lines.append(f"   - stream 启动加载节省约 **{bd - sd:.2f}** ms（dynamic 部分）。")
if sf > 0:
    lines.append(f"   - stream 仿真期额外读帧约 **{sf:.2f}** ms（median）。")
if ssim > bsim:
    lines.append(f"   - stream 主循环 median 慢 **{ssim - bsim:.2f}** ms（{pct_delta(bsim, ssim)}）。")
lines.append("")

lines.append("## 结论（本场景）")
lines.append("")
if s > b:
    lines.append(
        f"- 在 `{max_seconds}s` × `{runs}` 次运行下，**stream 端到端略慢于 bulk**（median {fmt_ms(s)} vs {fmt_ms(b)} ms）。"
    )
else:
    lines.append(
        f"- 在 `{max_seconds}s` × `{runs}` 次运行下，**stream 端到端略快于 bulk**（median {fmt_ms(s)} vs {fmt_ms(b)} ms）。"
    )
if sf > bf and sf > sd:
    lines.append("- 主要额外开销来自 **仿真期逐帧 I/O**（`load_frames_ms`），而非启动加载。")
lines.append("- 若场景更大或仿真步数更多，stream 的 per-frame 成本会线性增长；bulk 内存占用更高但热路径无读盘。")
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
