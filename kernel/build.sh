#!/bin/bash
#
# Build the plutosky-stream kernel: fetch the pinned ADI tree, apply the macb
# slot patch, build zImage and the device tree, and leave both in kernel/out.
#
# Usage: build.sh [clean]
#
# Everything downloaded or generated lives under kernel/build, which is not in
# the repository. Nothing here talks to the board.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
build=$here/build
linux=$build/linux
out=$here/out
ccwrap=$build/ccwrap

# The linux submodule pinned by plutosdr-fw v0.38, the kernel the board runs.
linux_remote=https://github.com/analogdevicesinc/linux.git
linux_sha=e14e351533f934047ba0473e836e561682ec67fe

jobs=${JOBS:-2}
cross=${CROSS_COMPILE:-arm-linux-gnueabihf-}
real_cc=${REAL_CC:-arm-linux-gnueabihf-gcc-11}

export ARCH=arm
export CROSS_COMPILE=$cross
# The kernel version string includes the builder. Fixed values keep the build
# host out of the shipped image.
export KBUILD_BUILD_USER=${KBUILD_BUILD_USER:-plutosky}
export KBUILD_BUILD_HOST=${KBUILD_BUILD_HOST:-plutosky-stream}

if [ "${1:-}" = "clean" ]; then
	rm -rf "$build"
	echo "removed $build"
	exit 0
fi

mkdir -p "$build" "$out"

if ! command -v "$real_cc" > /dev/null; then
	echo "$real_cc not found" >&2
	exit 1
fi

# A modern arm-linux-gnueabihf-gcc defaults to hard float, which makes the
# kernel's -march=armv7-a probe fall back to armv5t and then reject dmb ish.
# Forcing soft float on the compiler driver keeps the probe honest. The wrapper
# only shadows the compiler, so as, ld and objcopy still come from PATH.
mkdir -p "$ccwrap"
cat > "$ccwrap/${cross}gcc" <<EOF
#!/bin/sh
exec $real_cc -mfloat-abi=soft "\$@"
EOF
chmod +x "$ccwrap/${cross}gcc"
export PATH=$ccwrap:$PATH

echo "==> fetching linux $linux_sha"
mkdir -p "$linux"
if [ ! -d "$linux/.git" ]; then
	git -C "$linux" init -q
	git -C "$linux" remote add origin "$linux_remote"
fi
if ! git -C "$linux" cat-file -e "$linux_sha^{commit}" 2> /dev/null; then
	git -C "$linux" fetch --depth 1 origin "$linux_sha"
fi

echo "==> resetting the tree to the pin"
git -C "$linux" checkout -q --detach "$linux_sha"
git -C "$linux" reset -q --hard "$linux_sha"

# Only tracked files come back from the reset, so drop the files the patches
# add before applying them again. Object files stay, which keeps a rebuild
# after an edit to the patch short.
echo "==> applying patches"
shopt -s nullglob
for p in "$here"/patches/*.patch; do
	echo "    $(basename "$p")"
	# grep exits 1 for a patch that adds no files, which pipefail treats as fatal.
	{ grep -A3 '^new file mode' "$p" || true; } | sed -n 's|^+++ b/||p' |
		while read -r f; do rm -f "$linux/$f"; done
	git -C "$linux" apply --whitespace=nowarn "$p"
done
shopt -u nullglob

echo "==> configuring"
cp "$here/board.config" "$linux/.config"
make -C "$linux" olddefconfig > /dev/null

echo "==> building zImage with -j$jobs"
make -C "$linux" -j"$jobs" zImage

echo "==> building the device trees"
build_dtb() {
	# Drops the vendor tree's unit_address_vs_reg warnings, which do not mean this build failed.
	dtc -I dts -O dtb -o "$out/$2" "$here/dts/$1" 2>&1 |
		grep -v "unit_address_vs_reg" || true
}
build_dtb plutosky-stream.dts devicetree.dtb
build_dtb plutosky-stream-split.dts devicetree-split.dtb

cp "$linux/arch/arm/boot/zImage" "$out/zImage"

echo
echo "zImage                 $(stat -c %s "$out/zImage") bytes -> $out/zImage"
echo "devicetree.dtb         $(stat -c %s "$out/devicetree.dtb") bytes -> $out/devicetree.dtb"
echo "devicetree-split.dtb   $(stat -c %s "$out/devicetree-split.dtb") bytes -> $out/devicetree-split.dtb"
echo
echo "next: kernel/tools/build.sh, then VENDOR_RAMDISK=<stock uramdisk.image.gz> kernel/ramdisk/build.sh [path/to/daemon] [path/to/csr.json]"
