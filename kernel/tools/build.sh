#!/bin/bash
#
# Cross-compile gemslot_test for the board and run the host-side library test.
#
# The classifier header comes out of the patched kernel tree, so run
# kernel/build.sh first.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
kernel_dir=$(dirname "$here")
linux=$kernel_dir/build/linux
out=$kernel_dir/build/tools

cross=${CROSS_COMPILE:-arm-linux-gnueabihf-}
cc=${CC:-${cross}gcc-11}

cadence=$linux/drivers/net/ethernet/cadence
if [ ! -f "$cadence/macb_gemslot_lib.h" ]; then
	echo "patched kernel tree not found, run kernel/build.sh first" >&2
	exit 1
fi

mkdir -p "$out/include/linux"

# Stage the one uapi header rather than putting the whole kernel include tree
# on the search path, where it would shadow the sysroot's own linux headers.
cp "$linux/include/uapi/linux/gemslot.h" "$out/include/linux/"

echo "==> gemslot_test (armv7)"
# The vendor kernel does not export __stack_chk_guard, so anything that has to
# load or run beside it is built without the stack protector.
"$cc" -Wall -Wextra -O2 -static -s -fno-stack-protector \
	-I "$out/include" \
	-o "$out/gemslot_test" "$here/gemslot_test.c"
file "$out/gemslot_test" | sed 's/^/    /'

echo "==> gemslot_lib_test (host)"
cc -Wall -Wextra -O2 -I "$cadence" -o "$out/gemslot_lib_test" \
	"$here/gemslot_lib_test.c"
"$out/gemslot_lib_test"
