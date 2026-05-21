#!/usr/bin/env python3
"""Compatibility wrapper: keep old CLI, run C++ sim_runner."""

from __future__ import annotations

import argparse
import shlex
import subprocess
import sys
from pathlib import Path


THIS_DIR = Path(__file__).resolve().parent
REPO_ROOT = THIS_DIR.parent
DEFAULT_LOG_DIR = REPO_ROOT / "output" / "log"
DEFAULT_REPORT_DIR = REPO_ROOT / "output" / "report"
DEFAULT_SIMLOG_PATH = DEFAULT_LOG_DIR / "sim_log.json"
DEFAULT_GRADING_REPORT_PATH = DEFAULT_REPORT_DIR / "grading_report.json"


def _parse_args(argv) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Run C++ sim_runner with mostly-compatible run_sim.py flags.\n"
            "Planner is now C++ only. Python planner path is removed."
        )
    )
    p.add_argument("--scenario-dir", required=True)
    p.add_argument(
        "--scenario-load",
        choices=("bulk", "stream"),
        default="bulk",
        help="bulk: read dynamic_objects.json once; stream: per-frame files under dynamic_objects/",
    )
    p.add_argument(
        "--benchmark",
        action="store_true",
        help="Benchmark mode: --cpp-mode off, skip sim_log, print timing JSON",
    )
    p.add_argument("--planner", default="local_dwa")
    p.add_argument(
        "--reference-source",
        choices=("map", "sdc"),
        default="map",
        help="Reference polyline: lane_graph route (map) or SDC track (sdc).",
    )
    p.add_argument("--output", default=str(DEFAULT_SIMLOG_PATH))
    p.add_argument("--source-tag", default="waymo_sim_cpp")
    p.add_argument("--dt", type=float, default=0.1)
    p.add_argument("--max-seconds", type=float, default=0.0)
    p.add_argument("--stop-on-collision", action="store_true")
    p.add_argument("--desired-speed", type=float, default=13.9)
    p.add_argument("--ego-length", type=float, default=4.5)
    p.add_argument("--ego-width", type=float, default=1.85)
    p.add_argument("--ego-wheelbase", type=float, default=2.7)
    p.add_argument("--ego-rear-overhang", type=float, default=0.95)
    p.add_argument("--ego-max-speed", type=float, default=33.3)
    p.add_argument("--grading-bin", default="")
    p.add_argument("--grading-report", default="")
    p.add_argument("--metrics-config", default="")
    p.add_argument(
        "--log-dir",
        default="",
        help=(
            "Directory for spdlog file log (sim_*.log). "
            "If empty and --log-level is not info|off, defaults to <repo>/output/log."
        ),
    )
    p.add_argument("--log-level", default="info")
    p.add_argument("--cpp-mode", choices=("online", "offline", "both", "off"), default="online")

    # Keep legacy args accepted to reduce script breakage.
    p.add_argument("--no-interpolate-npcs", action="store_true")
    p.add_argument("--reference-step", type=float, default=1.0)
    p.add_argument("--no-python-grader", action="store_true")
    p.add_argument("--print-every", type=int, default=10)
    p.add_argument("--seed", type=int, default=0)
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = _parse_args(argv if argv is not None else sys.argv[1:])
    scenario_dir = Path(args.scenario_dir).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()
    DEFAULT_LOG_DIR.mkdir(parents=True, exist_ok=True)
    DEFAULT_REPORT_DIR.mkdir(parents=True, exist_ok=True)
    output.parent.mkdir(parents=True, exist_ok=True)
    report = (
        Path(args.grading_report).expanduser().resolve()
        if args.grading_report
        else DEFAULT_GRADING_REPORT_PATH
    )

    if args.no_interpolate_npcs:
        print("[sim] --no-interpolate-npcs ignored in current C++ runner", file=sys.stderr)
    if args.no_python_grader:
        print("[sim] --no-python-grader is now always true (Python grader removed)", file=sys.stderr)

    cmd = [
        "bazel",
        "run",
        "//cpp:sim_runner",
        "--",
        "--scenario-dir",
        str(scenario_dir),
        "--output",
        str(output),
        "--source-tag",
        args.source_tag,
        "--dt",
        str(args.dt),
        "--max-seconds",
        str(args.max_seconds),
        "--desired-speed",
        str(args.desired_speed),
        "--ego-length",
        str(args.ego_length),
        "--ego-width",
        str(args.ego_width),
        "--ego-wheelbase",
        str(args.ego_wheelbase),
        "--ego-rear-overhang",
        str(args.ego_rear_overhang),
        "--ego-max-speed",
        str(args.ego_max_speed),
        "--cpp-mode",
        args.cpp_mode,
        "--planner",
        args.planner,
        "--reference-source",
        args.reference_source,
        "--reference-step",
        str(args.reference_step),
        "--scenario-load",
        args.scenario_load,
    ]
    if args.benchmark:
        cmd.append("--benchmark")
    if args.stop_on_collision:
        cmd.append("--stop-on-collision")
    if args.grading_bin:
        report.parent.mkdir(parents=True, exist_ok=True)
        cmd.extend(["--grading-bin", str(Path(args.grading_bin).expanduser().resolve())])
        cmd.extend(["--grading-report", str(report)])
    if args.metrics_config:
        cmd.extend(
            [
                "--metrics-config",
                str(Path(args.metrics_config).expanduser().resolve()),
            ]
        )
    log_dir = (args.log_dir or "").strip()
    if not log_dir:
        level = (args.log_level or "info").strip().lower()
        if level and level not in ("info", "off"):
            log_dir = str(DEFAULT_LOG_DIR)
    if log_dir:
        cmd.extend(["--log-dir", str(Path(log_dir).expanduser().resolve())])
        cmd.extend(["--log-level", str(args.log_level)])

    print("[sim] exec:", " ".join(shlex.quote(x) for x in cmd))
    proc = subprocess.run(cmd, cwd=THIS_DIR, check=False)
    return proc.returncode


if __name__ == "__main__":
    sys.exit(main())
