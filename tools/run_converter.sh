#!/usr/bin/env bash
# 在装了 tensorflow + waymo-open-dataset 的 conda 环境里跑 waymo_to_scenario.py。
#
# 用法:
#   tools/run_converter.sh [out_dir]
#
# 环境变量 (可选):
#   SCENARIO_INDEX=0               TFRecord shard 内第几个 scenario
#   TFRECORD=/abs/path.tfrecord    显式 TFRecord (不指定就找 HYW_DATA_DIR 下第一个)
#   HYW_DATA_DIR=/abs/path         Waymo 数据目录 (默认: <repo>/data)
#   CONDA_ENV=waymo_env            conda 环境名
#   NO_CENTER=1                    保留 Waymo 原始全局坐标 (默认是把 SDC 起点平移到原点)

set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
export HYW_DATA_DIR="${HYW_DATA_DIR:-${GRADING_DATA_DIR:-$ROOT/data}}"

OUT_DIR="${1:-$ROOT/scenarios/waymo_scenario_${SCENARIO_INDEX:-0}}"
SCENARIO="${SCENARIO_INDEX:-0}"
CONDA_ENV="${CONDA_ENV:-waymo_env}"

ARGS=(--out-dir "$OUT_DIR" --scenario-index "$SCENARIO")
if [[ -n "${TFRECORD:-}" ]]; then
  ARGS+=(--tfrecord "$TFRECORD")
fi
if [[ -n "${NO_CENTER:-}" ]]; then
  ARGS+=(--no-center)
fi

conda run --no-capture-output -n "$CONDA_ENV" \
  python "$ROOT/tools/waymo_to_scenario.py" "${ARGS[@]}"

echo
echo "Scenario written to: $OUT_DIR"
echo "  scenario_meta.json    (init/goal/world_offset/统计)"
echo "  dynamic_objects.json  (NPC 逐帧轨迹)"
echo "  lane_graph.json       (车道连通图，给 planner 路由用)"
echo
echo "下一步:"
echo "    cd $ROOT/sim"
echo "    python3 run_sim.py --scenario-dir $OUT_DIR --output /tmp/sim_log.json"
