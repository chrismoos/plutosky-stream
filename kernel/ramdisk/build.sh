#!/bin/bash
#
# Build rootfs.cpio.gz from the vendor PlutoSDR ramdisk.
#
# Takes the vendor image apart, drops in /opt/plutosky and its init script,
# takes the iiod autostart out of the USB gadget setup, and repacks. Dropbear,
# the USB gadget and the vendor network scripts are left as they are, so SSH
# over USB and over Ethernet both keep working.
#
# Usage: VENDOR_RAMDISK=<uramdisk.image.gz> build.sh [path/to/daemon] [path/to/csr.json]
#
# VENDOR_RAMDISK is the stock PlutoSDR ramdisk from the board's SD card.
#
# csr.json is the register map of the bitstream the card will load. The daemon
# refuses to start without one, so pass the map from the same build_artifacts
# directory as the BOOT.bin.
#
# The daemon is built outside this tree. Without one, the ramdisk still boots
# and the init script reports it.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
kernel_dir=$(dirname "$here")
build=$kernel_dir/build/ramdisk
out=$kernel_dir/out

vendor_image=${VENDOR_RAMDISK:-}
daemon=${1:-}
csr_json=${2:-}

if [ -z "$vendor_image" ] || [ ! -f "$vendor_image" ]; then
	echo "set VENDOR_RAMDISK to the stock PlutoSDR uramdisk.image.gz" >&2
	exit 1
fi

if [ -n "$daemon" ] && [ ! -f "$daemon" ]; then
	echo "daemon not found at $daemon" >&2
	exit 1
fi

# Unpack, edit and repack all happen in one fakeroot session. Ownership and the
# /dev/console device node only exist in that session's records, so splitting
# the work across two of them loses them.
if ! command -v fakeroot > /dev/null; then
	echo "fakeroot is required so the repacked archive keeps root ownership" >&2
	exit 1
fi

rm -rf "$build"
mkdir -p "$build" "$out"

# u-boot legacy image header, ahead of the gzip cpio.
dd if="$vendor_image" of="$build/vendor.cpio.gz" bs=64 skip=1 status=none

cat > "$build/inner.sh" <<'INNER'
set -eu

build=$1
overlay=$2
tools=$3
daemon=$4
archive=$5
csr_json=$6

root=$build/root
mkdir -p "$root"

echo "==> unpacking"
(cd "$root" && gzip -dc "$build/vendor.cpio.gz" | cpio -idm --quiet)

echo "==> installing /opt/plutosky"
cp -a "$overlay/." "$root/"
chown -R 0:0 "$root/opt/plutosky" "$root/etc/init.d/S99plutosky"
chmod 755 "$root/etc/init.d/S99plutosky"

if [ -n "$daemon" ]; then
	install -m 755 -o 0 -g 0 "$daemon" "$root/opt/plutosky/daemon"
	echo "    daemon from $daemon"
else
	echo "    no daemon given, the init script starts without one"
fi

if [ -n "$csr_json" ]; then
	install -m 644 -o 0 -g 0 "$csr_json" "$root/opt/plutosky/csr.json"
	echo "    csr.json from $csr_json"
else
	echo "    no csr.json given, the daemon will not start without one" >&2
fi

if [ -f "$tools/gemslot_test" ]; then
	install -m 755 -o 0 -g 0 "$tools/gemslot_test" "$root/opt/plutosky/"
	echo "    gemslot_test installed"
else
	echo "    gemslot_test not built, run kernel/tools/build.sh first" >&2
fi

# The PL has no ADI converter cores, so iiod has nothing to serve. Leave
# the rest of the gadget alone, but the ffs.iio_ffs function has to leave the
# configuration with iiod: iiod owns those FunctionFS endpoints, and binding
# the UDC with an unserved function fails, which takes USB networking with it.
echo "==> disabling the vendor iiod autostart"
udc=$root/etc/init.d/S23udc
disable() {
	if ! grep -q "$1" "$udc"; then
		echo "S23udc has no line matching '$1', the vendor image changed" >&2
		exit 1
	fi
	sed -i "\\#$1#s#^#\\#plutosky disabled: #" "$udc"
}
disable "ln -s \$GADGET/functions/ffs.iio_ffs"
disable "mkdir -p /dev/iio_ffs"
disable "mount iio_ffs -t functionfs"
disable "start-stop-daemon -S -b -q -m -p /var/run/iiod.pid"
disable "start-stop-daemon -K -q -p /var/run/iiod.pid"
disable "rm \$GADGET/configs/c.1/ffs.iio_ffs"
# sed -i rewrites the file, so put the init script mode and owner back.
chown 0:0 "$udc"
chmod 755 "$udc"

echo "==> repacking"
chown 0:0 "$root"
chmod 755 "$root"
(cd "$root" && find . | cpio -o -H newc --quiet | gzip -9) > "$archive"
INNER

fakeroot bash "$build/inner.sh" "$build" "$here/overlay" \
	"$kernel_dir/build/tools" "$daemon" "$out/rootfs.cpio.gz" "$csr_json"

echo
echo "rootfs.cpio.gz  $(stat -c %s "$out/rootfs.cpio.gz") bytes -> $out/rootfs.cpio.gz"
