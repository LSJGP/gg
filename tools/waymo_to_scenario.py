#!/usr/bin/env python3
"""Convert one Waymo Motion Scenario into the 3 JSONs the hywgrading sim eats.

Outputs (under --out-dir):
  scenario_meta.json     init / goal / world_offset / scenario_id / 统计信息
  dynamic_objects.json   每个 track 的逐帧状态（车 / 人 / 自行车 / SDC）
  lane_graph.json        可路由车道图（id, type, speed_limit_kmh, centerline,
                                       entry_lanes, exit_lanes）

By default the SDC's first valid pose is anchored at (0, 0, 0); all map
features and all NPC tracks are translated by the same world_offset, so the
sim works in a local SDC-centric frame.

需要在 conda 环境里跑（含 tensorflow + waymo-open-dataset），见 run_converter.sh。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


DEFAULT_SPEED_LIMIT_KMH = 50.0


# ---------------- Waymo enum mappings ----------------

WAYMO_OBJECT_TYPE = {
    0: "UNSET",
    1: "VEHICLE",
    2: "PEDESTRIAN",
    3: "CYCLIST",
    4: "OTHER",
}

WAYMO_LANE_TYPE = {
    0: "UNDEFINED",
    1: "FREEWAY",
    2: "SURFACE_STREET",
    3: "BIKE_LANE",
}


# ---------------- helpers ----------------

def _polyline_to_xyz(polyline) -> List[Tuple[float, float, float]]:
    return [(float(p.x), float(p.y), float(getattr(p, "z", 0.0))) for p in polyline]


def _lane_speed_kmh(ln) -> float:
    mph = float(getattr(ln, "speed_limit_mph", 0.0) or 0.0)
    if mph > 0.0:
        return mph * 1.609344
    return DEFAULT_SPEED_LIMIT_KMH


# ---------------- core dataclass ----------------

@dataclass
class ConvertedScene:
    scenario_id: str
    world_offset: Tuple[float, float, float]
    init_pose: Optional[Tuple[float, float, float]]
    goal_pose: Optional[Tuple[float, float, float]]
    timestamps_seconds: List[float]
    current_time_index: int
    sdc_track_index: int
    tracks: List[Dict]
    track_type_counts: Dict[str, int]
    lane_graph: List[Dict]
    bbox: Tuple[float, float, float, float]
    num_lanes: int
    num_road_lines: int
    num_road_edges: int
    num_crosswalks: int


# ---------------- extraction ----------------

def _extract_tracks(scenario, ox: float, oy: float, oz: float, sdc_idx: int):
    """Per-track 逐帧状态（应用 world_offset）。返回 (tracks, type_counts)."""
    tracks: List[Dict] = []
    counts: Dict[str, int] = {}
    for ti, tr in enumerate(scenario.tracks):
        type_name = WAYMO_OBJECT_TYPE.get(int(tr.object_type), "OTHER")
        states = []
        for st in tr.states:
            if not st.valid:
                states.append({"valid": False})
                continue
            states.append(
                {
                    "valid": True,
                    "x": float(st.center_x) - ox,
                    "y": float(st.center_y) - oy,
                    "z": float(getattr(st, "center_z", 0.0)) - oz,
                    "yaw": float(st.heading),
                    "vx": float(st.velocity_x),
                    "vy": float(st.velocity_y),
                    "length": float(st.length),
                    "width": float(st.width),
                    "height": float(st.height),
                }
            )
        is_sdc = ti == sdc_idx
        tracks.append(
            {
                "track_index": ti,
                "id": int(tr.id),
                "object_type": type_name,
                "is_sdc": is_sdc,
                "states": states,
            }
        )
        if not is_sdc:
            counts[type_name] = counts.get(type_name, 0) + 1
    return tracks, counts


def _extract_lane_graph(scenario, ox: float, oy: float, oz: float) -> List[Dict]:
    """车道连通图：每条 lane 的 type / 限速 / 中心线 / 前后接邻 ID。"""
    out: List[Dict] = []
    for mf in scenario.map_features:
        if mf.WhichOneof("feature_data") != "lane":
            continue
        ln = mf.lane
        poly = _polyline_to_xyz(ln.polyline)
        if len(poly) < 2:
            continue
        out.append(
            {
                "id": int(mf.id),
                "type": WAYMO_LANE_TYPE.get(int(ln.type), "UNDEFINED"),
                "speed_limit_kmh": _lane_speed_kmh(ln),
                "centerline": [
                    [p[0] - ox, p[1] - oy, p[2] - oz] for p in poly
                ],
                "entry_lanes": [int(x) for x in ln.entry_lanes],
                "exit_lanes": [int(x) for x in ln.exit_lanes],
            }
        )
    return out


def _scene_stats(scenario) -> Tuple[int, int, int, int, Tuple[float, float, float, float]]:
    """统计：lanes / road_lines / road_edges / crosswalks 计数 + 整张图 XY bbox。"""
    nl = nrl = nre = ncw = 0
    xs: List[float] = []
    ys: List[float] = []
    for mf in scenario.map_features:
        which = mf.WhichOneof("feature_data")
        poly = []
        if which == "lane":
            nl += 1
            poly = _polyline_to_xyz(mf.lane.polyline)
        elif which == "road_line":
            nrl += 1
            poly = _polyline_to_xyz(mf.road_line.polyline)
        elif which == "road_edge":
            nre += 1
            poly = _polyline_to_xyz(mf.road_edge.polyline)
        elif which == "crosswalk":
            ncw += 1
            poly = _polyline_to_xyz(mf.crosswalk.polygon)
        for x, y, _ in poly:
            xs.append(x)
            ys.append(y)
    if not xs:
        bbox = (0.0, 0.0, 0.0, 0.0)
    else:
        bbox = (min(xs), min(ys), max(xs), max(ys))
    return nl, nrl, nre, ncw, bbox


def convert_scenario(scenario, center_on_sdc: bool = True) -> ConvertedScene:
    """Top-level: 解一个 Waymo Scenario,  返回 ConvertedScene."""
    init_pose: Optional[Tuple[float, float, float]] = None
    goal_pose: Optional[Tuple[float, float, float]] = None
    ox = oy = oz = 0.0
    sdc_idx = -1

    try:
        sdc_idx = int(scenario.sdc_track_index)
        sdc = scenario.tracks[sdc_idx]
        valid = [s for s in sdc.states if s.valid]
        if valid:
            s0, s1 = valid[0], valid[-1]
            init_pose = (float(s0.center_x), float(s0.center_y), float(s0.heading))
            goal_pose = (float(s1.center_x), float(s1.center_y), float(s1.heading))
            if center_on_sdc:
                ox, oy = init_pose[0], init_pose[1]
                oz = float(getattr(s0, "center_z", 0.0))
    except (AttributeError, IndexError, ValueError):
        pass

    if init_pose is not None:
        init_pose = (init_pose[0] - ox, init_pose[1] - oy, init_pose[2])
    if goal_pose is not None:
        goal_pose = (goal_pose[0] - ox, goal_pose[1] - oy, goal_pose[2])

    nl, nrl, nre, ncw, raw_bbox = _scene_stats(scenario)
    bbox = (raw_bbox[0] - ox, raw_bbox[1] - oy, raw_bbox[2] - ox, raw_bbox[3] - oy)

    tracks, counts = _extract_tracks(scenario, ox, oy, oz, sdc_idx)
    lane_graph = _extract_lane_graph(scenario, ox, oy, oz)

    sid = scenario.scenario_id if scenario.HasField("scenario_id") else "unknown"
    timestamps = list(scenario.timestamps_seconds)
    cti = int(scenario.current_time_index) if scenario.HasField("current_time_index") else 0

    return ConvertedScene(
        scenario_id=str(sid),
        world_offset=(ox, oy, oz),
        init_pose=init_pose,
        goal_pose=goal_pose,
        timestamps_seconds=timestamps,
        current_time_index=cti,
        sdc_track_index=sdc_idx,
        tracks=tracks,
        track_type_counts=counts,
        lane_graph=lane_graph,
        bbox=bbox,
        num_lanes=nl,
        num_road_lines=nrl,
        num_road_edges=nre,
        num_crosswalks=ncw,
    )


# ---------------- writers ----------------

def _pose(p: Optional[Tuple[float, float, float]]):
    if p is None:
        return None
    return {"x": float(p[0]), "y": float(p[1]), "yaw": float(p[2])}


def write_meta(scene: ConvertedScene, path: Path, source: str, scenario_index: int) -> None:
    duration = (
        float(scene.timestamps_seconds[-1] - scene.timestamps_seconds[0])
        if len(scene.timestamps_seconds) >= 2
        else 0.0
    )
    doc = {
        "source": source,
        "scenario_id": scene.scenario_id,
        "scenario_index": scenario_index,
        "world_offset": {
            "x": scene.world_offset[0],
            "y": scene.world_offset[1],
            "z": scene.world_offset[2],
        },
        "init_pose": _pose(scene.init_pose),
        "goal_pose": _pose(scene.goal_pose),
        "bbox": {
            "xmin": scene.bbox[0],
            "ymin": scene.bbox[1],
            "xmax": scene.bbox[2],
            "ymax": scene.bbox[3],
        },
        "stats": {
            "lanes": scene.num_lanes,
            "road_lines": scene.num_road_lines,
            "road_edges": scene.num_road_edges,
            "crosswalks": scene.num_crosswalks,
            "timestamps": len(scene.timestamps_seconds),
            "duration_s": duration,
            "current_time_index": scene.current_time_index,
            "tracks_total": len(scene.tracks),
            "tracks_non_sdc_by_type": dict(scene.track_type_counts),
        },
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f, indent=2)
        f.write("\n")


def write_dynamic_objects(scene: ConvertedScene, path: Path, source: str) -> None:
    doc = {
        "source": source,
        "scenario_id": scene.scenario_id,
        "world_offset": {
            "x": scene.world_offset[0],
            "y": scene.world_offset[1],
            "z": scene.world_offset[2],
        },
        "timestamps_seconds": scene.timestamps_seconds,
        "current_time_index": scene.current_time_index,
        "sdc_track_index": scene.sdc_track_index,
        "tracks": scene.tracks,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f)
        f.write("\n")


def write_lane_graph(scene: ConvertedScene, path: Path, source: str) -> None:
    doc = {
        "source": source,
        "scenario_id": scene.scenario_id,
        "world_offset": {
            "x": scene.world_offset[0],
            "y": scene.world_offset[1],
            "z": scene.world_offset[2],
        },
        "lanes": scene.lane_graph,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(doc, f)
        f.write("\n")


# ---------------- driver ----------------

def _resolve_data_dir(explicit: str) -> str:
    explicit = (explicit or "").strip()
    if explicit and Path(explicit).is_dir():
        return str(Path(explicit).resolve())
    env = (os.environ.get("HYW_DATA_DIR") or os.environ.get("GRADING_DATA_DIR") or "").strip()
    if env and Path(env).is_dir():
        return str(Path(env).resolve())
    here = Path(__file__).resolve().parent.parent
    cand = here / "data"
    if cand.is_dir() and any(cand.glob("*.tfrecord*")):
        return str(cand.resolve())
    return ""


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument("--tfrecord", default="", help="显式 TFRecord 路径")
    p.add_argument("--data-dir", default="", help="Waymo 数据目录（或设 HYW_DATA_DIR）")
    p.add_argument("--scenario-index", type=int, default=0, help="shard 内第几个 scenario")
    p.add_argument("--out-dir", required=True, help="输出目录（3 个 JSON 都写到这里）")
    p.add_argument(
        "--no-center", dest="center_on_sdc", action="store_false",
        help="保留 Waymo 原始全局坐标（默认是把 SDC 起点平移到原点）",
    )
    p.set_defaults(center_on_sdc=True)
    args = p.parse_args()

    try:
        import tensorflow as tf  # noqa: F401
        from waymo_open_dataset.protos import scenario_pb2
    except ImportError as e:
        print(
            "需要 tensorflow + waymo-open-dataset。请用 run_converter.sh 在 conda 环境里跑。\n"
            f"原始错误: {e}",
            file=sys.stderr,
        )
        return 2

    if args.tfrecord:
        tf_path = Path(args.tfrecord).expanduser().resolve()
        if not tf_path.is_file():
            print(f"--tfrecord 不存在: {tf_path}", file=sys.stderr)
            return 2
    else:
        ddir = _resolve_data_dir(args.data_dir)
        if not ddir:
            print("找不到数据目录。请用 --data-dir 或设 HYW_DATA_DIR。", file=sys.stderr)
            return 2
        records = sorted(Path(ddir).glob("*.tfrecord*"))
        if not records:
            print(f"{ddir} 下没有 *.tfrecord*", file=sys.stderr)
            return 2
        tf_path = records[0]

    print(f"[converter] tfrecord = {tf_path}")
    print(f"[converter] scenario index = {args.scenario_index}")

    import tensorflow as tf  # type: ignore
    ds = tf.data.TFRecordDataset(str(tf_path), compression_type="")
    scenario = None
    for i, raw in enumerate(ds):
        if i < args.scenario_index:
            continue
        scenario = scenario_pb2.Scenario()
        scenario.ParseFromString(bytes(raw.numpy()))
        break
    if scenario is None:
        print(f"scenario_index={args.scenario_index} 越界", file=sys.stderr)
        return 2

    scene = convert_scenario(scenario, center_on_sdc=args.center_on_sdc)
    out_dir = Path(args.out_dir).expanduser().resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    meta_path = out_dir / "scenario_meta.json"
    objs_path = out_dir / "dynamic_objects.json"
    graph_path = out_dir / "lane_graph.json"

    write_meta(scene, meta_path, source=str(tf_path), scenario_index=args.scenario_index)
    write_dynamic_objects(scene, objs_path, source=str(tf_path))
    write_lane_graph(scene, graph_path, source=str(tf_path))

    print(f"[converter] features: lanes={scene.num_lanes} "
          f"road_lines={scene.num_road_lines} road_edges={scene.num_road_edges} "
          f"crosswalks={scene.num_crosswalks}")
    print(f"[converter] tracks: total={len(scene.tracks)} "
          f"non_sdc_by_type={dict(scene.track_type_counts)} "
          f"timestamps={len(scene.timestamps_seconds)}")
    print(f"[converter] lane_graph: lanes={len(scene.lane_graph)}")
    print(f"[converter] init_pose = {scene.init_pose}")
    print(f"[converter] goal_pose = {scene.goal_pose}")
    print(f"[converter] wrote {meta_path}")
    print(f"[converter] wrote {objs_path}")
    print(f"[converter] wrote {graph_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
