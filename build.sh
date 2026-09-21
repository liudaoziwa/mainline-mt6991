#!/bin/bash
# Build a mainline kernel + DTB for realme GT7 (RMX6688) / MT6991.
#
# Runs natively on the phone itself (aarch64 host, no cross toolchain needed).
# The repo root IS the kernel tree; all build products land under out/
# (kbuild O=out) so the git tree stays clean.
#
# Produces:
#   out/arch/arm64/boot/Image
#   out/arch/arm64/boot/dts/mediatek/mt6991-realme-rmx6688.dtb
#
# The initramfs is linked into the Image rather than shipped as a separate
# ramdisk, so the kernel is self-contained and does not depend on whatever
# ramdisk the boot image happens to carry.
#
# Foreground note: step [1/5] compiles the init. When running this script in
# the background, run that gcc line by hand first so a compile error is not
# swallowed by the background chain.
set -euo pipefail

K=$(cd "$(dirname "$0")" && pwd)   # repo root = linux-7.2.3 kernel tree
IRFS=$K/initramfs
OUT=$K/out
DTB=mediatek/mt6991-realme-rmx6688.dtb

# Building on-device with ~5 GB free: 4 jobs keeps memory pressure survivable.
# Override with JOBS=n ./build.sh once you know it fits.
JOBS=${JOBS:-4}

mkdir -p "$OUT"
cd "$K"

echo "==> [1/5] static init"
# INIT_SRC picks the init source.  Default init-rootfs.c is the minimal
# rootfs-switch init; INIT_SRC=init.c builds the bring-up test init (the
# display/touch register tests, then a reboot to Android).
INIT_SRC=${INIT_SRC:-init-rootfs.c}
gcc -static -O2 -Wall -Wextra -o "$IRFS/init" "$IRFS/$INIT_SRC"
strip "$IRFS/init"
ls -la "$IRFS/init"

echo "==> [2/5] initramfs cpio_list"
# gen_init_cpio's list format declares device nodes without needing mknod,
# which proot would not allow anyway.
cat > "$IRFS/cpio_list" <<EOF
dir /dev 0755 0 0
nod /dev/console 0600 0 0 c 5 1
nod /dev/null 0666 0 0 c 1 3
nod /dev/kmsg 0644 0 0 c 1 11
dir /proc 0755 0 0
dir /sys 0755 0 0
dir /bin 0755 0 0
file /bin/busybox $IRFS/busybox 0755 0 0
dir /lib 0755 0 0
dir /lib/firmware 0755 0 0
dir /lib/firmware/tp 0755 0 0
dir /lib/firmware/tp/24618 0755 0 0
file /lib/firmware/tp/24618/FW_S3910_TIANMA_HBP.img $IRFS/firmware/tp/24618/FW_S3910_TIANMA_HBP.img 0644 0 0
file /lib/firmware/tp/24618/FW_S3910_TIANMA_HBP_FAE.img $IRFS/firmware/tp/24618/FW_S3910_TIANMA_HBP_FAE.img 0644 0 0
file /lib/firmware/tp/24618/LIMIT_S3910_TIANMA_HBP.img $IRFS/firmware/tp/24618/LIMIT_S3910_TIANMA_HBP.img 0644 0 0
file /lib/firmware/tp/24618/LIMIT_S3910_TIANMA_HBP_AGING.img $IRFS/firmware/tp/24618/LIMIT_S3910_TIANMA_HBP_AGING.img 0644 0 0
dir /lib/firmware/arm 0755 0 0
dir /lib/firmware/arm/mali 0755 0 0
dir /lib/firmware/arm/mali/arch13.8 0755 0 0
file /lib/firmware/arm/mali/arch13.8/mali_csffw.bin $IRFS/firmware/arm/mali/arch13.8/mali_csffw.bin 0644 0 0
file /init $IRFS/init 0755 0 0
EOF
cat "$IRFS/cpio_list"

echo "==> [3/5] config"
# CONFIG_INITRAMFS_SOURCE is generated here so the fragment stays path-free.
cat "$K/rmx6688.fragment" > "$OUT/.fragment.gen"
echo "CONFIG_INITRAMFS_SOURCE=\"$IRFS/cpio_list\"" >> "$OUT/.fragment.gen"

ARCH=arm64 ./scripts/kconfig/merge_config.sh \
	-m -O "$OUT" arch/arm64/configs/defconfig "$OUT/.fragment.gen"
ARCH=arm64 make O="$OUT" olddefconfig

echo "--- verifying the options that matter survived the merge ---"
fail=0
for o in CONFIG_FB=y CONFIG_FB_SIMPLE=y CONFIG_FRAMEBUFFER_CONSOLE=y \
	 CONFIG_MEDIATEK_WATCHDOG=y CONFIG_PSTORE_RAM=y CONFIG_PSTORE_CONSOLE=y \
	 CONFIG_CMDLINE_FORCE=y CONFIG_SERIAL_8250_MT6577=y \
	 CONFIG_BLK_DEV_INITRD=y CONFIG_LOGO=y \
	 CONFIG_ARM64_EMBEDDED_DTB=y CONFIG_INITRAMFS_FORCE=y \
	 CONFIG_DRM_MEDIATEK=y CONFIG_MTK_MMSYS=y CONFIG_MTK_CMDQ=y \
	 CONFIG_DRM_PANEL_AE031_DSI_VDO=y; do
	if grep -qx "$o" "$OUT/.config"; then
		echo "  ok      $o"
	else
		echo "  MISSING $o  -> $(grep "^${o%%=*}[= ]" "$OUT/.config" || echo 'not present')"
		fail=1
	fi
done
if grep -qx "CONFIG_DRM_SIMPLEDRM=y" "$OUT/.config"; then
	echo "  PROBLEM CONFIG_DRM_SIMPLEDRM=y conflicts with FB_SIMPLE"
	fail=1
fi
grep -q "^CONFIG_INITRAMFS_SOURCE=\"$IRFS/cpio_list\"" "$OUT/.config" \
	&& echo "  ok      CONFIG_INITRAMFS_SOURCE" || { echo "  MISSING CONFIG_INITRAMFS_SOURCE"; fail=1; }
grep -q "^CONFIG_ARM64_EMBEDDED_DTB_SOURCE=\"arch/arm64/boot/dts/$DTB\"" "$OUT/.config" \
	&& echo "  ok      CONFIG_ARM64_EMBEDDED_DTB_SOURCE" \
	|| { echo "  MISSING CONFIG_ARM64_EMBEDDED_DTB_SOURCE -> $(grep '^CONFIG_ARM64_EMBEDDED_DTB_SOURCE' "$OUT/.config" || echo 'not present')"; fail=1; }
echo "  cmdline $(grep '^CONFIG_CMDLINE=' "$OUT/.config")"
[ "$fail" = 0 ] || { echo "config verification FAILED"; exit 1; }

echo "==> [4/5] DTB"
# Build only our board's DTB, not the whole tree: an unrelated DTB failing
# elsewhere in mainline would otherwise abort this step before ever reaching
# mediatek/.  The target is given relative to arch/arm64/boot/dts/.
#
# Do NOT pass DTC_FLAGS= on the command line.  A command-line variable wins
# over every 'DTC_FLAGS +=' in the makefiles, including the '-@' that
# scripts/Makefile.dtbs adds for base DTBs of composite (overlay) targets --
# their base then comes out without a /__symbols__ node and fdtoverlay fails.
# That is what silently broke the amlogic composite DTBs here.
ARCH=arm64 make O="$OUT" -j"$JOBS" "${DTB}"
ls -la "$OUT/arch/arm64/boot/dts/$DTB"

echo "--- dtc warnings for our DTB (should be none) ---"
"$OUT/scripts/dtc/dtc" -I dtb -O dts "$OUT/arch/arm64/boot/dts/$DTB" -o /dev/null

echo "==> [5/5] kernel Image (this is the long part)"
ARCH=arm64 make O="$OUT" -j"$JOBS" Image

echo
echo "--- verifying the DTB actually embedded in the Image ---"
# The whole point of CONFIG_ARM64_EMBEDDED_DTB is that the Image carries the
# device tree, so the thing worth checking is the Image itself, not the .dtb
# sitting next to it.  Locate the blob by symbol and compare byte for byte: a
# stale embedded copy is otherwise invisible and would boot the wrong tree.
text=$(grep -E " _text$" "$OUT/System.map" | cut -d' ' -f1)
dtb_s=$(grep " __embedded_dtb_start" "$OUT/System.map" | cut -d' ' -f1)
dtb_e=$(grep " __embedded_dtb_end" "$OUT/System.map" | cut -d' ' -f1)
[ -n "$text" ] && [ -n "$dtb_s" ] && [ -n "$dtb_e" ] || {
	echo "  FAILED: embedded DTB symbols not found in System.map"; exit 1; }
off=$(printf '%d' $((0x$dtb_s - 0x$text)))
len=$(printf '%d' $((0x$dtb_e - 0x$dtb_s)))
[ $((0x$dtb_s % 32)) -eq 0 ] \
	&& echo "  ok      32-byte aligned at 0x$dtb_s" \
	|| { echo "  FAILED: misaligned at 0x$dtb_s"; exit 1; }
dd if="$OUT/arch/arm64/boot/Image" of="$OUT/.embedded.dtb" bs=1 skip="$off" count="$len" status=none
cmp "$OUT/.embedded.dtb" "$OUT/arch/arm64/boot/dts/$DTB" \
	&& echo "  ok      embedded DTB is identical to $DTB ($len bytes @ +$off)" \
	|| { echo "  FAILED: embedded DTB differs from the built one"; exit 1; }
rm -f "$OUT/.embedded.dtb"

echo
echo "==================== BUILD OK ===================="
ls -la "$OUT/arch/arm64/boot/Image" "$OUT/arch/arm64/boot/dts/$DTB"
echo
echo "Image : $(stat -c%s "$OUT/arch/arm64/boot/Image") bytes  (DTB embedded, flash boot only)"
echo "DTB   : $(stat -c%s "$OUT/arch/arm64/boot/dts/$DTB") bytes"
echo "kernel: $(make -s O="$OUT" kernelrelease 2>/dev/null || echo '?')"
