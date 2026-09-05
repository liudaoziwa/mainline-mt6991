#!/bin/bash
# pack_boot.sh [N] -- turn out/arch/arm64/boot/Image into a flashable boot.img.
#
# Uses bt/magiskboot ONLY (the legacy top-level bootimg.py is a stub and must
# never be used).  Flow: unpack the stock boot image, swap the kernel for our
# Image, repack, then unpack the result and byte-compare the extracted kernel
# against the Image that went in -- a repack that mangles or truncates the
# kernel would otherwise boot as a silent hang.
#
# Round number: explicit argument, else max(new-boot.img.N)+1.  Artifacts:
#   bt/new-boot.img.N     flashable image
#   bt/pack/vN/kernel     kernel extracted back out of it (cmp proof)
set -euo pipefail

K=$(cd "$(dirname "$0")" && pwd)
BT=$K/bt
IMG=$K/out/arch/arm64/boot/Image
STOCK=$BT/boot.img              # stock boot image, never modified
MB=$BT/magiskboot
RUN=$BT/pack

[ -x "$MB" ]  || { echo "missing $MB"; exit 1; }
[ -f "$STOCK" ] || { echo "missing $STOCK (stock boot.img for the slot being flashed)"; exit 1; }
[ -f "$IMG" ] || { echo "missing $IMG - run ./build.sh first"; exit 1; }

if [ -n "${1:-}" ]; then
	N=$1
else
	N=0
	for f in "$BT"/new-boot.img.*; do
		[ -e "$f" ] || continue
		n=${f##*.}
		[ "$n" -gt "$N" ] 2>/dev/null && N=$n
	done
	N=$((N+1))
fi

rm -rf "$RUN"; mkdir -p "$RUN"
cp "$IMG" "/tmp/image$N"        # reference for the roundtrip check

cd "$RUN"
"$MB" unpack "$STOCK"           # extracts ./kernel (stock), ramdisk, dtb, ...
cp "/tmp/image$N" kernel        # swap in our Image
"$MB" repack "$STOCK" "$BT/new-boot.img.$N"
ls -la "$BT/new-boot.img.$N"

# roundtrip verification
mkdir -p "v$N"
cd "v$N"
"$MB" unpack "$BT/new-boot.img.$N"
cmp kernel "/tmp/image$N" \
	&& echo "==> roundtrip OK: kernel extracted from new-boot.img.$N is byte-identical to the Image" \
	|| { echo "==> roundtrip FAILED: extracted kernel differs"; exit 1; }
echo
echo "flash bt/new-boot.img.$N ($(stat -c%s "$BT/new-boot.img.$N") bytes)"
