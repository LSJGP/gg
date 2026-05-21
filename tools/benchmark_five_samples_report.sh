#!/usr/bin/env bash
# Prepare 5 scenarios and run json/proto × bulk/stream benchmarks; write one report.
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SIM_DIR="$ROOT/sim"
OUT_DIR="$ROOT/output/benchmark"
PYTHON="${ROOT}/tools/.venv/bin/python"
RUNS="${RUNS:-5}"
MAX_SECONDS="${MAX_SECONDS:-9.0}"
DT="${DT:-0.1}"
STAMP="$(date +%Y%m%d_%H%M%S)"
REPORT="$OUT_DIR/five_samples_${STAMP}_report.md"
RAW_DIR="$OUT_DIR/five_samples_${STAMP}_raw"
mkdir -p "$OUT_DIR" "$RAW_DIR"

# Five sample scenario indices (waymo_scenario_0 .. _4).
SAMPLES=(0 1 2 3 4)

if [[ ! -x "$PYTHON" ]]; then
  echo "[five] creating tools/.venv ..."
  python3 -m venv "$ROOT/tools/.venv"
  "$ROOT/tools/.venv/bin/pip" install -q -r "$ROOT/tools/requirements-proto.txt"
fi
if [[ ! -f "$ROOT/tools/gen/proto/sim/scenario_pb2.py" ]]; then
  bash "$ROOT/tools/gen_sim_protos.sh"
fi

echo "[five] building sim_runner..."
(cd "$SIM_DIR" && bazel build //cpp:sim_runner >/dev/null)

prepare_scenario() {
  local dir="$1"
  local name
  name="$(basename "$dir")"
  echo "[five] prepare $name ..."
  if [[ ! -f "$dir/dynamic_objects.json" ]]; then
    echo "[five] skip $name: no dynamic_objects.json" >&2
    return 1
  fi
  "$PYTHON" "$ROOT/tools/json_scenario_to_proto.py" "$dir" 2>/dev/null || true
  "$PYTHON" "$ROOT/tools/split_existing_dynamic_objects.py" --format both "$dir"
}

run_bench() {
  local dir="$1"
  local fmt="$2"
  local mode="$3"
  local out="$4"
  : >"$out"
  local i
  for ((i = 1; i <= RUNS; i++)); do
    local line
    line="$(cd "$SIM_DIR" && bazel run //cpp:sim_runner -- \
      --scenario-dir "$dir" \
      --scenario-load "$mode" \
      --input-format "$fmt" \
      --benchmark \
      --max-seconds "$MAX_SECONDS" \
      --dt "$DT" 2>/dev/null | tail -n 1)" || true
    if [[ "$line" =~ total_ms ]]; then
      echo "$line" >>"$out"
    else
      echo "{\"error\":\"sim_failed\",\"scenario_dir\":\"$dir\",\"input_format\":\"$fmt\",\"scenario_load\":\"$mode\"}" >>"$out"
    fi
  done
}

for idx in "${SAMPLES[@]}"; do
  dir="$ROOT/scenarios/waymo_scenario_${idx}"
  if [[ ! -d "$dir" ]]; then
    echo "[five] missing $dir, skip" >&2
    continue
  fi
  prepare_scenario "$dir" || continue
  base="waymo_scenario_${idx}"
  for fmt in json proto; do
    for mode in bulk stream; do
      run_bench "$dir" "$fmt" "$mode" "$RAW_DIR/${base}_${fmt}_${mode}.jsonl"
      echo "[five] done $base $fmt $mode"
    done
  done
done

"$PYTHON" - "$REPORT" "$RAW_DIR" "$RUNS" "$MAX_SECONDS" "$DT" "${SAMPLES[@]}" <<'PY'
import json
import statistics
import sys
from datetime import datetime
from pathlib import Path

report_path = Path(sys.argv[1])
raw_dir = Path(sys.argv[2])
runs = int(sys.argv[3])
max_seconds = float(sys.argv[4])
dt = float(sys.argv[5])
samples = [int(x) for x in sys.argv[6:]]

METRICS = [
    ("load_meta_map_ms", "Meta+地图"),
    ("load_dynamic_ms", "Dynamic"),
    ("load_frames_ms", "逐帧I/O"),
    ("sim_loop_ms", "仿真循环"),
    ("total_ms", "总耗时"),
]


def load_rows(path: Path) -> list[dict]:
    if not path.is_file():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line.startswith("{"):
            rows.append(json.loads(line))
    return rows


def median(vals: list[float]) -> float:
    return statistics.median(vals) if vals else float("nan")


def mean(vals: list[float]) -> float:
    return statistics.mean(vals) if vals else float("nan")


def pct(a: float, b: float) -> str:
    if a != a or b != b or abs(a) < 1e-6:
        return "n/a"
    d = (b - a) / a * 100.0
    return f"{d:+.1f}%"


lines = [
    "# Sim 场景加载性能报告（5 样本）",
    "",
    f"- 生成时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
    f"- 每配置运行次数: **{runs}**",
    f"- 仿真: `max_seconds={max_seconds}`, `dt={dt}`, `--benchmark`",
    f"- 对比维度: `input_format` × `scenario_load`（json/proto × bulk/stream）",
    "",
]

# Per-scenario table
lines.append("## 分场景 median 总耗时 (ms)")
lines.append("")
lines.append("| 场景 | json bulk | json stream | proto bulk | proto stream | proto vs json bulk |")
lines.append("|---|---:|---:|---:|---:|---|")

all_rows: dict[tuple[str, str], list[dict]] = {}
for idx in samples:
    base = f"waymo_scenario_{idx}"
    cells = {}
    for fmt in ("json", "proto"):
        for mode in ("bulk", "stream"):
            path = raw_dir / f"{base}_{fmt}_{mode}.jsonl"
            rows = load_rows(path)
            all_rows[(base, f"{fmt}_{mode}")] = rows
            cells[f"{fmt}_{mode}"] = median([r["total_ms"] for r in rows])

    jb = cells.get("json_bulk", float("nan"))
    js = cells.get("json_stream", float("nan"))
    pb = cells.get("proto_bulk", float("nan"))
    ps = cells.get("proto_stream", float("nan"))
    lines.append(
        f"| `{base}` | {jb:.1f} | {js:.1f} | {pb:.1f} | {ps:.1f} | {pct(jb, pb)} |"
    )
lines.append("")

# Aggregate across 5 samples
lines.append("## 五样本汇总 median (ms)")
lines.append("")
lines.append("| 指标 | json bulk | json stream | proto bulk | proto stream |")
lines.append("|---|---:|---:|---:|---:|")

for key, label in METRICS:
    vals = {f"{fmt}_{mode}": [] for fmt in ("json", "proto") for mode in ("bulk", "stream")}
    for idx in samples:
        base = f"waymo_scenario_{idx}"
        for fmt in ("json", "proto"):
            for mode in ("bulk", "stream"):
                rows = load_rows(raw_dir / f"{base}_{fmt}_{mode}.jsonl")
                vals[f"{fmt}_{mode}"].extend(float(r.get(key, 0)) for r in rows)
    lines.append(
        f"| {label} | {median(vals['json_bulk']):.1f} | {median(vals['json_stream']):.1f} "
        f"| {median(vals['proto_bulk']):.1f} | {median(vals['proto_stream']):.1f} |"
    )
lines.append("")

# Speedup summary
lines.append("## 加速比（五样本 total_ms median）")
lines.append("")
lines.append("| 对比 | 倍数 (json/proto) | 说明 |")
lines.append("|---|---:|---|")
jb_all, pb_all, js_all, ps_all = [], [], [], []
for idx in samples:
    base = f"waymo_scenario_{idx}"
    for fmt, mode, acc in [
        ("json", "bulk", jb_all),
        ("proto", "bulk", pb_all),
        ("json", "stream", js_all),
        ("proto", "stream", ps_all),
    ]:
        rows = load_rows(raw_dir / f"{base}_{fmt}_{mode}.jsonl")
        if rows:
            acc.append(median([r["total_ms"] for r in rows]))

def speedup(a, b):
    if a != a or b != b or b < 1e-6:
        return "n/a"
    return f"{a / b:.2f}x"

lines.append(f"| bulk 端到端 | {speedup(median(jb_all), median(pb_all))} | proto 跳过 lane_graph Struct 解析 |")
lines.append(f"| stream 端到端 | {speedup(median(js_all), median(ps_all))} | stream 含仿真期读帧 |")
lines.append("")

lines.append("## 解读")
lines.append("")
lines.append("1. **proto bulk** 主要收益在 `load_meta_map_ms`（`lane_graph.pb` 直接 ParseFromString）。"
             "五样本汇总见上表。")
lines.append("2. **proto stream** 帧文件为二进制 `DynamicFrame.pb`，`load_frames_ms` 通常低于 JSON 逐帧 Struct。")
lines.append("3. **bulk vs stream**：stream 启动 `load_dynamic_ms` 更低，但仿真期有 `load_frames_ms`；"
             "短仿真下 total 接近，长场景 stream 读盘成本上升。")
lines.append("4. 原始 JSONL 位于 `" + str(raw_dir) + "`。")
lines.append("")

report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
print(report_path.read_text(encoding="utf-8"))
PY

echo ""
echo "[five] report: $REPORT"
echo "[five] raw:    $RAW_DIR"
