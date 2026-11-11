#!/usr/bin/env bash
# Build the SD card boot set for the PlutoSky: the vendor u-boot's sdboot loads
# uImage, devicetree.dtb and uramdisk.image.gz from the FAT partition and the
# FSBL inside BOOT.bin programs the PL. Pass BOOT.bin when one exists.
#
# Usage: make_sd.sh <zImage> <devicetree.dtb> <rootfs.cpio.gz> <out_dir> [BOOT.bin]
set -euo pipefail

KERNEL="${1:?zImage}"
DTB="${2:?devicetree.dtb}"
ROOTFS="${3:?rootfs.cpio.gz}"
OUT="${4:?output directory}"
BOOTBIN="${5:-}"
HERE="$(cd "$(dirname "$0")" && pwd)"
LOAD=0x8000

command -v mkimage >/dev/null || { echo "mkimage not on PATH" >&2; exit 1; }
mkdir -p "$OUT"
mkimage -A arm -O linux -T kernel -C none -a $LOAD -e $LOAD -n plutosky-stream -d "$KERNEL" "$OUT/uImage" >/dev/null
mkimage -A arm -O linux -T ramdisk -C gzip -n plutosky-stream -d "$ROOTFS" "$OUT/uramdisk.image.gz" >/dev/null
cp "$DTB" "$OUT/devicetree.dtb"
cp "$HERE/uEnv.sd.txt" "$OUT/uEnv.txt"
[ -n "$BOOTBIN" ] && cp "$BOOTBIN" "$OUT/BOOT.bin"
(cd "$OUT" && sha256sum $(ls BOOT.bin devicetree.dtb uEnv.txt uImage uramdisk.image.gz 2>/dev/null) > SHA256SUMS)
ls -la "$OUT"
