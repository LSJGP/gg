#!/usr/bin/env python3
"""Split dynamic_objects.json into per-frame files for stream loading.

Output layout under <scenario_dir>/dynamic_objects/:
  header.json       metadata + track list without states
  sdc_states.json   SDC track with full states (for reference-source sdc)
  frames/NNNNN.json per native timestamp, non-SDC states only
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List


def split_dynamic_objects(scenario_dir: Path) -> None:
    scenario_dir = scenario_dir.expanduser().resolve()
    src = scenario_dir / "dynamic_objects.json"
    if not src.is_file():
        raise FileNotFoundError(f"missing {src}")

    with src.open(encoding="utf-8") as f:
        doc = json.load(f)

    timestamps: List[float] = list(map(float, doc.get("timestamps_seconds", [])))
    if not timestamps:
        raise ValueError("timestamps_seconds is empty")

    tracks: List[Dict[str, Any]] = doc.get("tracks", [])
    sdc_idx = int(doc.get("sdc_track_index", -1))
    num_frames = len(timestamps)

    out_base = scenario_dir / "dynamic_objects"
    frames_dir = out_base / "frames"
    frames_dir.mkdir(parents=True, exist_ok=True)

    track_meta: List[Dict[str, Any]] = []
    sdc_track: Dict[str, Any] | None = None

    for tr in tracks:
        meta = {
            "track_index": int(tr["track_index"]),
            "id": int(tr["id"]),
            "object_type": str(tr.get("object_type", "OTHER")),
            "is_sdc": bool(tr.get("is_sdc", False)),
        }
        track_meta.append(meta)
        if meta["is_sdc"] or meta["track_index"] == sdc_idx:
            sdc_track = {
                "track_index": meta["track_index"],
                "id": meta["id"],
                "object_type": meta["object_type"],
                "is_sdc": True,
                "states": tr.get("states", []),
            }

    header = {
        "source": doc.get("source", ""),
        "scenario_id": doc.get("scenario_id", ""),
        "world_offset": doc.get("world_offset", {"x": 0, "y": 0, "z": 0}),
        "timestamps_seconds": timestamps,
        "current_time_index": int(doc.get("current_time_index", 0)),
        "sdc_track_index": sdc_idx,
        "tracks": track_meta,
    }
    with (out_base / "header.json").open("w", encoding="utf-8") as f:
        json.dump(header, f, indent=2)
        f.write("\n")

    if sdc_track is not None:
        with (out_base / "sdc_states.json").open("w", encoding="utf-8") as f:
            json.dump(sdc_track, f)
            f.write("\n")

    for fi in range(num_frames):
        t = timestamps[fi]
        states_out: List[Dict[str, Any]] = []
        for tr in tracks:
            if tr.get("is_sdc") or int(tr.get("track_index", -1)) == sdc_idx:
                continue
            states = tr.get("states", [])
            if fi >= len(states):
                continue
            st = states[fi]
            entry = {
                "track_index": int(tr["track_index"]),
                "id": int(tr["id"]),
                "object_type": str(tr.get("object_type", "OTHER")),
            }
            if st.get("valid", False):
                entry.update(
                    {
                        "valid": True,
                        "x": float(st["x"]),
                        "y": float(st["y"]),
                        "z": float(st.get("z", 0.0)),
                        "yaw": float(st.get("yaw", 0.0)),
                        "vx": float(st.get("vx", 0.0)),
                        "vy": float(st.get("vy", 0.0)),
                        "length": float(st.get("length", 4.5)),
                        "width": float(st.get("width", 1.85)),
                        "height": float(st.get("height", 1.6)),
                    }
                )
            else:
                entry["valid"] = False
            states_out.append(entry)

        frame_doc = {
            "frame_index": fi,
            "timestamp": t,
            "states": states_out,
        }
        path = frames_dir / f"{fi:05d}.json"
        with path.open("w", encoding="utf-8") as f:
            json.dump(frame_doc, f)
            f.write("\n")

    print(
        f"[split] {scenario_dir.name}: {num_frames} frames, "
        f"{len(track_meta)} tracks -> {out_base}"
    )


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument(
        "scenario_dirs",
        nargs="+",
        help="Scenario directories containing dynamic_objects.json",
    )
    args = p.parse_args()
    for d in args.scenario_dirs:
        try:
            split_dynamic_objects(Path(d))
        except (OSError, ValueError, json.JSONDecodeError) as e:
            print(f"[split] failed {d}: {e}", file=sys.stderr)
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
