#!/usr/bin/env bash
# Gather a finished remote Vivado build into build_artifacts/.
#
# Usage: collect_artifacts.sh <remote build dir> [artifact dir]
#   REMOTE=user@host          ssh destination (required)
#   VIVADO=<settings64.sh>    Vivado settings script on the remote
#   VENDOR=<dir>              directory holding vendor_fsbl.elf and vendor_u-boot.elf (required)
#
# The remote directory is the one build_remote.sh was pointed at, holding
# plutosky_stream/gateware/ with the routed design. bootgen runs there to make
# the SD boot image and the fpga_manager bitstream, then the bitstreams, every
# report and the log come back, and the per-clock timing table is extracted so
# it can be read without opening the log.
set -euo pipefail

REMOTE_DIR="${1:?remote build dir}"
OUT="${2:-build_artifacts/vivado}"
: "${REMOTE:?set REMOTE=user@host}"
VIVADO="${VIVADO:-/opt/AMD/2026.1/Vivado/settings64.sh}"
HERE="$(cd "$(dirname "$0")" && pwd)"
VENDOR="${VENDOR:?set VENDOR to the directory holding vendor_fsbl.elf and vendor_u-boot.elf}"
GW="$REMOTE_DIR/plutosky_stream/gateware"

scp -q "$VENDOR/vendor_fsbl.elf" "$REMOTE:$GW/fsbl.elf"
scp -q "$VENDOR/vendor_u-boot.elf" "$REMOTE:$GW/u-boot.elf"
scp -q "$HERE/boot/bootgen.bif" "$REMOTE:$GW/bootgen.bif"
ssh "$REMOTE" "cd $GW && source $VIVADO >/dev/null && \
    bootgen -image bootgen.bif -arch zynq -o BOOT.bin -w on >/dev/null && \
    printf 'all:\n{\n\tplutosky_stream.bit\n}\n' > bit.bif && \
    bootgen -image bit.bif -arch zynq -process_bitstream bin -w on >/dev/null"

mkdir -p "$OUT"
# The placed and routed reports only.
ssh "$REMOTE" "cd $GW && tar -czf - plutosky_stream.bit plutosky_stream.bit.bin BOOT.bin vivado.log \
    plutosky_stream_drc.rpt plutosky_stream_io.rpt plutosky_stream_route_status.rpt \
    plutosky_stream_timing.rpt plutosky_stream_utilization_place.rpt \
    plutosky_stream_utilization_hierarchical_place.rpt" \
    | tar -C "$OUT" -xzf -

# Report headers include the build host name.
sed -i -E '/^\| Host +: /d' "$OUT"/*.rpt

# The per-clock table, as the log prints it after the final optimization.
awk '/Design Timing Summary/{f=1} f && /Intra Clock Table/{g=1} g{print} g && /Inter Clock Table/{exit}' \
    "$OUT/vivado.log" > "$OUT/timing_summary.txt"

ls -la "$OUT"
sha256sum "$OUT"/*.bit "$OUT"/*.bin
