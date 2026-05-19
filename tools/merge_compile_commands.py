#!/usr/bin/env python3
"""Merge sim/ and grading_mini/ compile_commands.json into repo-root for IDE."""
from __future__ import annotations

import json
import sys
from pathlib import Path


def load_entries(path: Path) -> list[dict]:
    if not path.is_file():
        print(f"warn: missing {path}", file=sys.stderr)
        return []
    with path.open(encoding="utf-8") as f:
        data = json.load(f)
    return data if isinstance(data, list) else []


def merge(repo_root: Path) -> list[dict]:
    merged: list[dict] = []
    for sub, prefix in (("sim", "sim"), ("grading_mini", "grading_mini")):
        src = repo_root / sub / "compile_commands.json"
        base_dir = repo_root / sub
        for entry in load_entries(src):
            e = dict(entry)
            e["directory"] = str(base_dir.resolve())
            file_field = e.get("file", "")
            if file_field and not Path(file_field).is_absolute():
                e["file"] = str((base_dir / file_field).resolve())
            merged.append(e)
    return merged


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    out = repo_root / "compile_commands.json"
    merged = merge(repo_root)
    out.write_text(json.dumps(merged, indent=2) + "\n", encoding="utf-8")
    print(f"[ide] wrote {out} ({len(merged)} entries)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
