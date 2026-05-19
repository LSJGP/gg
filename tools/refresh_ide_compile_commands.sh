#!/usr/bin/env bash
# Regenerate compile_commands.json for clangd (sim + grading_mini).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"

echo "[ide] sim: build protos + refresh compile_commands"
cd "$ROOT/sim"
bazel build //proto/sim:runtime_cc_proto //cpp:sim_runner
bazel run //:refresh_compile_commands

echo "[ide] grading_mini: refresh compile_commands"
cd "$ROOT/grading_mini"
bazel build //src/entry:grading_main
bazel run //:refresh_compile_commands

echo "[ide] merge compile_commands.json at repo root"
python3 "$ROOT/tools/merge_compile_commands.py"

echo "[ide] done. Restart clangd: Command Palette -> clangd: Restart language server"
echo "      (If still red: disable Microsoft C++ extension for this workspace)"
