#!/usr/bin/env bash
# Regenerates tools/control.desc, the compiled control service that
# tools/control.py loads at run time, from proto/plutosky/v1/control.proto.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
protoc -I "$ROOT/proto" --descriptor_set_out="$ROOT/tools/control.desc" \
    plutosky/v1/control.proto
