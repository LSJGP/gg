#!/usr/bin/env bash
# Generate Python stubs for hyw_sim protos (used by waymo_to_scenario / split scripts).
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PROTO_INC="$ROOT/sim"
OUT="$ROOT/tools/gen"

if ! command -v protoc >/dev/null 2>&1; then
  echo "protoc not found; install protobuf compiler (e.g. apt install protobuf-compiler)" >&2
  exit 1
fi

mkdir -p "$OUT"
protoc -I "$PROTO_INC" \
  --python_out="$OUT" \
  "$PROTO_INC/proto/sim/common.proto" \
  "$PROTO_INC/proto/sim/map.proto" \
  "$PROTO_INC/proto/sim/scenario.proto"

touch "$OUT/__init__.py" "$OUT/proto/__init__.py" "$OUT/proto/sim/__init__.py" 2>/dev/null || true
echo "[gen_sim_protos] wrote Python stubs under $OUT/proto/sim/"
echo "[gen_sim_protos] Python runtime: pip install -r tools/requirements-proto.txt (or use tools/.venv)"
