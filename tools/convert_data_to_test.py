#!/usr/bin/env python3
"""Convert Waymo TFRecord under repo ``data/`` into ``test/`` and print a full dataset report.

Uses ``waymo_to_scenario.py`` (conda env ``waymo_env`` by default). After conversion,
inspects both the raw Scenario proto and the three output JSON files.

Example:
  ./tools/convert_data_to_test.py
  ./tools/convert_data_to_test.py --scenario-index 1 --out-dir test/scenario_1
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_DATA_DIR = REPO_ROOT / "data"
DEFAULT_OUT_DIR = REPO_ROOT / "test"
CONVERTER = REPO_ROOT / "tools" / "waymo_to_scenario.py"
DEFAULT_CONDA_PYTHON = Path.home() / "miniconda3/envs/waymo_env/bin/python"


def _resolve_tfrecord(data_dir: Path, explicit: str) -> Path:
    if explicit:
        p = Path(explicit).expanduser().resolve()
        if not p.is_file():
            raise FileNotFoundError(f"TFRecord not found: {p}")
        return p
    if not data_dir.is_dir():
        raise FileNotFoundError(f"Data directory missing: {data_dir}")
    records = sorted(data_dir.glob("*.tfrecord*"))
    if not records:
        raise FileNotFoundError(f"No *.tfrecord* under {data_dir}")
    return records[0]


def _pick_python(explicit: str) -> Path:
    if explicit:
        return Path(explicit).expanduser().resolve()
    if DEFAULT_CONDA_PYTHON.is_file():
        return DEFAULT_CONDA_PYTHON
    return Path(sys.executable)


def run_converter(
    python: Path,
    tfrecord: Path,
    out_dir: Path,
    scenario_index: int,
    center_on_sdc: bool,
) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        str(python),
        str(CONVERTER),
        "--tfrecord",
        str(tfrecord),
        "--scenario-index",
        str(scenario_index),
        "--out-dir",
        str(out_dir),
    ]
    if not center_on_sdc:
        cmd.append("--no-center")
    print(f"[convert] {' '.join(cmd)}")
    subprocess.run(cmd, check=True, cwd=str(REPO_ROOT))


def _load_json(path: Path) -> Any:
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _fmt_pose(p: Optional[Dict]) -> str:
    if not p:
        return "(none)"
    return f"x={p.get('x', 0):.3f}, y={p.get('y', 0):.3f}, yaw={p.get('yaw', 0):.4f}"


def analyze_raw_scenario(tfrecord: Path, scenario_index: int, python: Path) -> Dict[str, Any]:
    """Load one Scenario from TFRecord via waymo_env (same stack as converter)."""
    code = r"""
import json, math, sys
from collections import Counter
from pathlib import Path
import tensorflow as tf
from waymo_open_dataset.protos import scenario_pb2

tf_path = Path(sys.argv[1])
idx = int(sys.argv[2])
ds = tf.data.TFRecordDataset(str(tf_path), compression_type="")
scenario = None
for i, raw in enumerate(ds):
    if i == idx:
        scenario = scenario_pb2.Scenario()
        scenario.ParseFromString(bytes(raw.numpy()))
        break
if scenario is None:
    print(json.dumps({"error": f"scenario_index {idx} out of range"}))
    sys.exit(1)

WAYMO_OBJECT_TYPE = {0:"UNSET",1:"VEHICLE",2:"PEDESTRIAN",3:"CYCLIST",4:"OTHER"}
WAYMO_LANE_TYPE = {0:"UNDEFINED",1:"FREEWAY",2:"SURFACE_STREET",3:"BIKE_LANE"}

def poly_len(poly):
    pts = [(float(p.x), float(p.y)) for p in poly]
    d = 0.0
    for a, b in zip(pts, pts[1:]):
        d += math.hypot(b[0]-a[0], b[1]-a[1])
    return d, len(pts)

map_counts = Counter()
map_detail = {"lane_types": Counter(), "road_line_types": Counter(), "road_edge_types": Counter()}
lane_speeds = []
for mf in scenario.map_features:
    which = mf.WhichOneof("feature_data")
    map_counts[which or "unknown"] += 1
    if which == "lane":
        map_detail["lane_types"][WAYMO_LANE_TYPE.get(int(mf.lane.type), "?")] += 1
        mph = float(getattr(mf.lane, "speed_limit_mph", 0) or 0)
        if mph > 0:
            lane_speeds.append(mph * 1.609344)
    elif which == "road_line":
        map_detail["road_line_types"][str(int(mf.road_line.type))] += 1
    elif which == "road_edge":
        map_detail["road_edge_types"][str(int(mf.road_edge.type))] += 1

track_types = Counter()
sdc_idx = int(scenario.sdc_track_index) if scenario.HasField("sdc_track_index") else -1
sdc_travel = 0.0
sdc_valid_frames = 0
for ti, tr in enumerate(scenario.tracks):
    tname = WAYMO_OBJECT_TYPE.get(int(tr.object_type), "OTHER")
    track_types[tname] += 1
    if ti == sdc_idx:
        valid = [st for st in tr.states if st.valid]
        sdc_valid_frames = len(valid)
        for a, b in zip(valid, valid[1:]):
            sdc_travel += math.hypot(b.center_x - a.center_x, b.center_y - a.center_y)

out = {
    "scenario_id": scenario.scenario_id if scenario.HasField("scenario_id") else None,
    "num_scenarios_in_file_hint": None,
    "timestamps_count": len(scenario.timestamps_seconds),
    "duration_s": float(scenario.timestamps_seconds[-1] - scenario.timestamps_seconds[0]) if len(scenario.timestamps_seconds) >= 2 else 0.0,
    "current_time_index": int(scenario.current_time_index) if scenario.HasField("current_time_index") else None,
    "sdc_track_index": sdc_idx,
    "tracks_total": len(scenario.tracks),
    "tracks_by_type": dict(track_types),
    "sdc_valid_frames": sdc_valid_frames,
    "sdc_travel_m": round(sdc_travel, 2),
    "map_features_by_kind": dict(map_counts),
    "lane_types_in_raw": dict(map_detail["lane_types"]),
    "road_line_type_ids": dict(map_detail["road_line_types"]),
    "road_edge_type_ids": dict(map_detail["road_edge_types"]),
    "lane_speed_kmh_minmax": [min(lane_speeds), max(lane_speeds)] if lane_speeds else None,
    "dynamic_map_states_count": len(scenario.dynamic_map_states),
    "tracks_to_predict_count": len(scenario.tracks_to_predict),
}
print(json.dumps(out))
"""
    proc = subprocess.run(
        [str(python), "-c", code, str(tfrecord), str(scenario_index)],
        capture_output=True,
        text=True,
        check=False,
    )
    if proc.returncode != 0:
        return {"error": proc.stderr or proc.stdout}
    return json.loads(proc.stdout.strip())


def analyze_converted(out_dir: Path) -> Dict[str, Any]:
    meta = _load_json(out_dir / "scenario_meta.json")
    objs = _load_json(out_dir / "dynamic_objects.json")
    graph = _load_json(out_dir / "lane_graph.json")

    tracks = objs.get("tracks", [])
    lanes = graph.get("lanes", [])
    road_lines = graph.get("road_lines", [])
    road_edges = graph.get("road_edges", [])
    crosswalks = graph.get("crosswalks", [])
    stop_signs = graph.get("stop_signs", [])
    driveways = graph.get("driveways", [])
    speed_bumps = graph.get("speed_bumps", [])
    map_counts = graph.get("map_feature_counts", meta.get("stats", {}))
    ts = objs.get("timestamps_seconds", [])

    type_counts = Counter()
    valid_state_counts: List[int] = []
    for t in tracks:
        if t.get("is_sdc"):
            continue
        type_counts[t.get("object_type", "?")] += 1
        valid_state_counts.append(sum(1 for s in t.get("states", []) if s.get("valid")))

    lane_types = Counter(l.get("type") for l in lanes)
    cl_lens = [len(l.get("centerline", [])) for l in lanes]
    speeds = [l.get("speed_limit_kmh", 0) for l in lanes]
    entry_deg = [len(l.get("entry_lanes", [])) for l in lanes]
    exit_deg = [len(l.get("exit_lanes", [])) for l in lanes]

    return {
        "files": {
            "scenario_meta.json": (out_dir / "scenario_meta.json").stat().st_size,
            "dynamic_objects.json": (out_dir / "dynamic_objects.json").stat().st_size,
            "lane_graph.json": (out_dir / "lane_graph.json").stat().st_size,
        },
        "meta": meta,
        "dynamic_objects_summary": {
            "tracks_total": len(tracks),
            "sdc_track_index": objs.get("sdc_track_index"),
            "non_sdc_by_type": dict(type_counts),
            "timestamps": len(ts),
            "dt_approx_s": (ts[1] - ts[0]) if len(ts) >= 2 else None,
            "duration_s": (ts[-1] - ts[0]) if len(ts) >= 2 else 0,
            "valid_states_per_track_minmax": [
                min(valid_state_counts) if valid_state_counts else 0,
                max(valid_state_counts) if valid_state_counts else 0,
            ],
        },
        "lane_graph_summary": {
            "map_feature_counts": map_counts,
            "lanes_exported": len(lanes),
            "road_lines": len(road_lines),
            "road_edges": len(road_edges),
            "crosswalks": len(crosswalks),
            "stop_signs": len(stop_signs),
            "driveways": len(driveways),
            "speed_bumps": len(speed_bumps),
            "lane_types": dict(lane_types),
            "centerline_points_minmax_avg": [
                min(cl_lens) if cl_lens else 0,
                max(cl_lens) if cl_lens else 0,
                sum(cl_lens) / len(cl_lens) if cl_lens else 0,
            ],
            "speed_limit_kmh_minmax": [min(speeds), max(speeds)] if speeds else None,
            "connectivity_entry_max": max(entry_deg) if entry_deg else 0,
            "connectivity_exit_max": max(exit_deg) if exit_deg else 0,
        },
        "static_map_keys": [
            "lanes",
            "road_lines",
            "road_edges",
            "crosswalks",
            "stop_signs",
            "driveways",
            "speed_bumps",
        ],
        "track_state_fields": [
            "valid",
            "x",
            "y",
            "z",
            "yaw",
            "vx",
            "vy",
            "length",
            "width",
            "height",
        ],
        "lane_fields": [
            "id",
            "type",
            "speed_limit_kmh",
            "centerline",
            "entry_lanes",
            "exit_lanes",
        ],
    }


def print_report(
    tfrecord: Path,
    scenario_index: int,
    out_dir: Path,
    raw: Dict[str, Any],
    converted: Dict[str, Any],
) -> None:
    meta = converted["meta"]
    print("\n" + "=" * 72)
    print("DATASET REPORT (Waymo Motion Scenario → hywgrading test/)")
    print("=" * 72)

    print("\n## 1. 原始数据（data/ 下的 TFRecord）")
    print(f"  文件: {tfrecord}")
    print(f"  大小: {tfrecord.stat().st_size / (1024*1024):.1f} MB")
    print(f"  格式: Waymo Open Dataset Motion TFRecord（每条 record = 一个 Scenario protobuf）")
    print(f"  本次转换: scenario_index = {scenario_index}")
    if "error" not in raw:
        print(f"  scenario_id: {raw.get('scenario_id')}")
        print(f"  时间: {raw.get('timestamps_count')} 帧, 时长 {raw.get('duration_s'):.3f} s")
        print(f"  current_time_index: {raw.get('current_time_index')} (Waymo 标注的“当前”时刻)")
        print(f"  SDC: track_index={raw.get('sdc_track_index')}, "
              f"有效帧={raw.get('sdc_valid_frames')}, 轨迹长约 {raw.get('sdc_travel_m')} m")
        print(f"  动态物体 tracks: 共 {raw.get('tracks_total')} 条, 类型分布 {raw.get('tracks_by_type')}")
        print(f"  地图 map_features (原始, 未全部导出):")
        for k, v in sorted(raw.get("map_features_by_kind", {}).items()):
            print(f"    - {k}: {v}")
        print(f"    lane 类型: {raw.get('lane_types_in_raw')}")
        if raw.get("lane_speed_kmh_minmax"):
            lo, hi = raw["lane_speed_kmh_minmax"]
            print(f"    lane 限速(km/h): {lo:.2f} ~ {hi:.2f}")
        print(f"  静态地图 7 类要素均已写入 lane_graph.json")
        print(f"  dynamic_map_states: {raw.get('dynamic_map_states_count')} 条 (交通灯等动态地图)")
        print(f"  tracks_to_predict: {raw.get('tracks_to_predict_count')} 条 (竞赛预测目标)")

    print("\n## 2. 转换输出（test/ 三个 JSON）")
    for name, size in converted["files"].items():
        print(f"  {name}: {size / 1024:.1f} KB")

    print("\n### scenario_meta.json — 场景元数据")
    print(f"  scenario_id: {meta.get('scenario_id')}")
    print(f"  world_offset: {meta.get('world_offset')}  (全局→局部的平移)")
    print(f"  init_pose (SDC 起点): {_fmt_pose(meta.get('init_pose'))}")
    print(f"  goal_pose (SDC 终点): {_fmt_pose(meta.get('goal_pose'))}")
    print(f"  bbox (局部 XY): {meta.get('bbox')}")
    st = meta.get("stats", {})
    print(f"  stats: {json.dumps(st, ensure_ascii=False)}")

    print("\n### dynamic_objects.json — 所有交通参与者逐帧状态")
    ds = converted["dynamic_objects_summary"]
    print(f"  tracks 总数: {ds['tracks_total']} (含 1 条 SDC)")
    print(f"  非 SDC 类型: {ds['non_sdc_by_type']}")
    print(f"  timestamps_seconds: {ds['timestamps']} 个, dt≈{ds['dt_approx_s']} s, 时长 {ds['duration_s']:.3f} s")
    print(f"  每条 track 的 states 字段: {converted['track_state_fields']}")
    print("  说明: valid=false 的帧表示该物体未观测到；坐标已减 world_offset")

    print("\n### lane_graph.json — 静态地图全集（7 类 map feature）")
    lg = converted["lane_graph_summary"]
    print(f"  map_feature_counts: {lg.get('map_feature_counts')}")
    print(f"  lanes: {lg['lanes_exported']}  road_lines: {lg['road_lines']}  "
          f"road_edges: {lg['road_edges']}  crosswalks: {lg['crosswalks']}")
    print(f"  stop_signs: {lg['stop_signs']}  driveways: {lg['driveways']}  "
          f"speed_bumps: {lg['speed_bumps']}")
    print(f"  车道类型: {lg['lane_types']}")
    print(f"  lanes 字段: {converted['lane_fields']} + interpolating, left/right_boundaries, left/right_neighbors")
    print(f"  road_lines/road_edges: id, type, polyline")
    print(f"  crosswalks/driveways/speed_bumps: id, polygon")
    print(f"  stop_signs: id, position, lanes")
    mn, mx, avg = lg["centerline_points_minmax_avg"]
    print(f"  lane centerline 点数/条: min={mn}, max={mx}, avg={avg:.1f}")
    if lg["speed_limit_kmh_minmax"]:
        print(f"  speed_limit_kmh: {lg['speed_limit_kmh_minmax'][0]:.2f} ~ {lg['speed_limit_kmh_minmax'][1]:.2f}")
    print(f"  拓扑: entry_lanes / exit_lanes 最大度数 {lg['connectivity_entry_max']} / {lg['connectivity_exit_max']}")

    print("\n## 3. 仍未导出的 Waymo 信息（非静态地图或动态层）")
    print("  - dynamic_map_states（交通灯等随时间变化）")
    print("  - tracks_to_predict（竞赛预测目标列表）")

    print("\n## 4. sim 模块如何使用这些数据")
    print("  - scenario_loader: 读 meta + dynamic_objects → Scenario struct")
    print("  - lane_graph: 读 lane_graph.json → 最短路/参考线 (reference-source=map)")
    print("  - WorldSimulator: 用 tracks 插值 NPC，Planner 每帧输出 PlanCommand")
    print("=" * 72 + "\n")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    p.add_argument("--tfrecord", default="", help="显式指定 TFRecord（默认 data/ 下第一个）")
    p.add_argument("--out-dir", type=Path, default=DEFAULT_OUT_DIR)
    p.add_argument("--scenario-index", type=int, default=0)
    p.add_argument("--python", default="", help="含 tensorflow + waymo-open-dataset 的 Python")
    p.add_argument("--no-center", action="store_true", help="不平移 SDC 到原点")
    p.add_argument("--skip-convert", action="store_true", help="仅分析已有 test/ 输出")
    args = p.parse_args()

    python = _pick_python(args.python)
    try:
        tfrecord = _resolve_tfrecord(args.data_dir, args.tfrecord)
    except FileNotFoundError as e:
        print(e, file=sys.stderr)
        return 2

    out_dir = args.out_dir.expanduser().resolve()

    if not args.skip_convert:
        try:
            run_converter(
                python,
                tfrecord,
                out_dir,
                args.scenario_index,
                center_on_sdc=not args.no_center,
            )
        except subprocess.CalledProcessError as e:
            print(f"转换失败: {e}", file=sys.stderr)
            return e.returncode or 1

    for name in ("scenario_meta.json", "dynamic_objects.json", "lane_graph.json"):
        if not (out_dir / name).is_file():
            print(f"缺少 {out_dir / name}，请先完成转换", file=sys.stderr)
            return 2

    raw = analyze_raw_scenario(tfrecord, args.scenario_index, python)
    converted = analyze_converted(out_dir)
    print_report(tfrecord, args.scenario_index, out_dir, raw, converted)
    return 0


if __name__ == "__main__":
    sys.exit(main())
