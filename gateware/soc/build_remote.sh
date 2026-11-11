#!/usr/bin/env bash
# Run the Vivado step of a LiteX build on a remote x86-64 host.
#
# Usage: build_remote.sh [build_dir]     (default: build/plutosky_stream)
#   REMOTE=user@host        ssh destination (required)
#   VIVADO=/opt/AMD/2026.1/Vivado/settings64.sh   settings script on the remote
#   REMOTE_DIR=~/plutosky-gateware                remote scratch directory
#
# The local build directory (Verilog, XDC, Tcl from --build --no-compile) is
# copied up, `vivado -mode batch -source <top>.tcl` runs there, and the
# bitstream plus the timing and utilization reports come back into the same
# local directory. rsync is used when both ends have it and tar over ssh
# otherwise.
set -euo pipefail

BUILD_DIR="${1:-build/plutosky_stream}"
: "${REMOTE:?set REMOTE=user@host}"
VIVADO="${VIVADO:-/opt/AMD/2026.1/Vivado/settings64.sh}"
REMOTE_DIR="${REMOTE_DIR:-~/plutosky-gateware}"

GATEWARE_DIR="$BUILD_DIR/gateware"
TCL="$(ls "$GATEWARE_DIR"/*.tcl | head -n1)"
[ -n "$TCL" ] || { echo "no .tcl in $GATEWARE_DIR; run the LiteX target with --build --no-compile first" >&2; exit 1; }
TOP="$(basename "${TCL%.tcl}")"
NAME="$(basename "$BUILD_DIR")"

# LiteX writes absolute local paths into the Tcl. Rewrite them for the remote tree.
LOCAL_ABS="$(cd "$GATEWARE_DIR" && pwd)"
REMOTE_GW="$REMOTE_DIR/$NAME/gateware"

if command -v rsync >/dev/null && ssh "$REMOTE" 'command -v rsync >/dev/null'; then
    TRANSPORT=rsync
else
    TRANSPORT=tar
fi
echo "transport: $TRANSPORT"

ssh "$REMOTE" "rm -rf $REMOTE_GW && mkdir -p $REMOTE_GW"
if [ "$TRANSPORT" = rsync ]; then
    rsync -az --delete "$GATEWARE_DIR/" "$REMOTE:$REMOTE_GW/"
else
    tar -C "$GATEWARE_DIR" -czf - . | ssh "$REMOTE" "tar -C $REMOTE_GW -xzf -"
fi

REMOTE_ABS="$(ssh "$REMOTE" "cd $REMOTE_GW && pwd")"
ssh "$REMOTE" "cd $REMOTE_GW && sed -i 's#$LOCAL_ABS#$REMOTE_ABS#g' $TOP.tcl && \
    source $VIVADO && vivado -mode batch -source $TOP.tcl -log vivado.log -journal vivado.jou"

if [ "$TRANSPORT" = rsync ]; then
    rsync -az --include='*.bit' --include='*.bin' --include='*.rpt' --include='vivado.log' --include='*.ltx' \
        --exclude='*' "$REMOTE:$REMOTE_GW/" "$GATEWARE_DIR/"
else
    ssh "$REMOTE" "cd $REMOTE_GW && tar -czf - \$(ls *.bit *.bin *.rpt *.ltx vivado.log 2>/dev/null)" \
        | tar -C "$GATEWARE_DIR" -xzf -
fi
echo "bitstream: $GATEWARE_DIR/$TOP.bit"
