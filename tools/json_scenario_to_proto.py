#!/usr/bin/env python3
"""Convert existing scenario JSON files to hyw_sim .pb (no TFRecord needed)."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict

_TOOLS = Path(__file__).resolve().parent
if str(_TOOLS) not in sys.path:
    sys.path.insert(0, str(_TOOLS))

from hyw_proto_convert import (  # noqa: E402
    dynamic_objects_from_json_doc,
    lane_graph_doc_to_static_map,
    write_message_pb,
)
from proto.sim import scenario_pb2  # noqa: E402


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open(encoding="utf-8") as f:
        return json.load(f)


def convert_scenario_dir(scenario_dir: Path, split_stream: bool) -> None:
    scenario_dir = scenario_dir.expanduser().resolve()
    meta_j = scenario_dir / "scenario_meta.json"
    dyn_j = scenario_dir / "dynamic_objects.json"
    map_j = scenario_dir / "lane_graph.json"
    for p in (meta_j, dyn_j, map_j):
        if not p.is_file():
            raise FileNotFoundError(f"missing {p}")

    meta_doc = _load_json(meta_j)
    dyn_doc = _load_json(dyn_j)
    map_doc = _load_json(map_j)

    meta = scenario_pb2.ScenarioMeta()
    meta.source = str(meta_doc.get("source", ""))
    meta.scenario_id = str(meta_doc.get("scenario_id", ""))
    meta.scenario_index = int(meta_doc.get("scenario_index", 0))
    wo = meta_doc.get("world_offset", {})
    meta.world_offset.x = float(wo.get("x", 0))
    meta.world_offset.y = float(wo.get("y", 0))
    meta.world_offset.z = float(wo.get("z", 0))
    ip = meta_doc.get("init_pose", {})
    meta.init_pose.x = float(ip.get("x", 0))
    meta.init_pose.y = float(ip.get("y", 0))
    meta.init_pose.yaw = float(ip.get("yaw", 0))
    gp = meta_doc.get("goal_pose", {})
    meta.goal_pose.x = float(gp.get("x", 0))
    meta.goal_pose.y = float(gp.get("y", 0))
    meta.goal_pose.yaw = float(gp.get("yaw", 0))
    bb = meta_doc.get("bbox", {})
    meta.bbox.xmin = float(bb.get("xmin", 0))
    meta.bbox.ymin = float(bb.get("ymin", 0))
    meta.bbox.xmax = float(bb.get("xmax", 0))
    meta.bbox.ymax = float(bb.get("ymax", 0))
    st = meta_doc.get("stats", {})
    for k in (
        "lanes",
        "road_lines",
        "road_edges",
        "crosswalks",
        "stop_signs",
        "driveways",
        "speed_bumps",
    ):
        if k in st:
            setattr(meta.stats, k, int(st[k]))
    meta.stats.timestamps = int(st.get("timestamps", 0))
    meta.stats.duration_s = float(st.get("duration_s", 0))
    meta.stats.current_time_index = int(st.get("current_time_index", 0))
    meta.stats.tracks_total = int(st.get("tracks_total", 0))
    for t, c in st.get("tracks_non_sdc_by_type", {}).items():
        meta.stats.tracks_non_sdc_by_type[t] = int(c)

    dyn = dynamic_objects_from_json_doc(dyn_doc)
    sm = lane_graph_doc_to_static_map(map_doc)

    write_message_pb(meta, scenario_dir / "scenario_meta.pb")
    write_message_pb(dyn, scenario_dir / "dynamic_objects.pb")
    write_message_pb(sm, scenario_dir / "lane_graph.pb")
    print(f"[json_to_proto] wrote .pb under {scenario_dir}")

    if split_stream:
        from split_existing_dynamic_objects import split_dynamic_objects

        split_dynamic_objects(scenario_dir, fmt="proto")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("scenario_dirs", nargs="+")
    p.add_argument(
        "--split-dynamic-frames",
        action="store_true",
        help="Also split dynamic_objects into stream .pb layout",
    )
    args = p.parse_args()
    try:
        import hyw_proto_convert  # noqa: F401
    except ImportError:
        print("Run: bash tools/gen_sim_protos.sh", file=sys.stderr)
        return 2
    for d in args.scenario_dirs:
        try:
            convert_scenario_dir(Path(d), args.split_dynamic_frames)
        except Exception as e:
            print(f"[json_to_proto] failed {d}: {e}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
