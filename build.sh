#!/bin/bash
# Build a mainline kernel + DTB for realme GT7 (RMX6688) / MT6991.
#
# The repo root IS the kernel tree; all build products land under out/
# (kbuild O=out) so the git tree stays clean.
#
# Runs natively on the phone itself (aarch64 host, no cross toolchain needed)
# and unchanged on an x86_64 host: the default clang/LLVM toolchain targets
# aarch64 via --target (no cross binutils), and the init compile below does
# the same when the host is not aarch64.
# Produces:
#   out/arch/arm64/boot/Image
#   out/arch/arm64/boot/dts/mediatek/mt6991-realme-rmx6688.dtb
#
# The initramfs is linked into the Image rather than shipped as a separate
# ramdisk, so the kernel is self-contained and does not depend on whatever
# ramdisk the boot image happens to carry.
#
# Order matters: the external modules (scp / ccci_fsm_scp / connscp /
# clk-mt6991-drv / gps_drv_dl_v062 / sensorhub_probe) are built *before* the
# initramfs list is generated and the Image is linked, so the .ko files that
# go into the initramfs are the ones built by this run.  A single `./build.sh`
# always produces an Image whose initramfs carries the current modules.
#
# Foreground note: step [1/6] compiles the init. When running this script in
# the background, run that static-init compile by hand first so a compile
# error is not swallowed by the background chain.
set -euo pipefail

K=$(cd "$(dirname "$0")" && pwd)   # repo root = linux-7.2.3 kernel tree
IRFS=$K/initramfs
OUT=$K/out
DTB=mediatek/mt6991-realme-rmx6688.dtb

# Building on-device with ~5 GB free: 4 jobs keeps memory pressure survivable.
# Override with JOBS=n ./build.sh once you know it fits.
JOBS=${JOBS:-4}

# Toolchain: clang/LLVM/lld by default (kbuild LLVM=1 -> clang, ld.lld,
# llvm-ar/nm/objcopy/strip/...).  TOOLCHAIN=gcc selects the gcc/binutils
# toolchain instead; LLVM=-<N> pins a versioned clang (e.g. LLVM=-18).
case "${TOOLCHAIN:-clang}" in
gcc)
	unset LLVM
	HOST_CC=${CROSS_COMPILE:-}gcc
	HOST_STRIP=${CROSS_COMPILE:-}strip
	;;
clang|llvm|lld)
	export LLVM=${LLVM:-1}
	HOST_CC=clang${LLVM#1}
	HOST_STRIP=llvm-strip${LLVM#1}
	;;
*)
	echo "TOOLCHAIN must be clang (default) or gcc" >&2
	exit 1
	;;
esac
echo "==> toolchain: ${TOOLCHAIN:-clang}, LLVM=${LLVM:-no} (static init via $HOST_CC)"

# Step [1/6] runs $(HOST_CC) directly, outside kbuild -- which derives the
# aarch64 triple from ARCH=arm64 on its own (scripts/Makefile.clang).  Match
# that here so an x86_64 checkout runs unmodified: target aarch64 explicitly
# and link with lld, which removes any need for cross binutils (the host
# strip cannot handle foreign ELF objects, hence the LLVM strip above).
if [ "$(uname -m)" = aarch64 ]; then
	TARGET_CC_FLAGS=
	TARGET_LD_FLAGS=
else
	TARGET_CC_FLAGS="--target=aarch64-linux-gnu"
	TARGET_LD_FLAGS="-fuse-ld=lld"
fi

mkdir -p "$OUT"
cd "$K"

# Write $IRFS/cpio_list, the initramfs file list.  The .ko entries reference
# whatever module files currently exist in $K, so it must be (re)generated
# after the module builds and before 'make Image'.  Every other file is under
# $IRFS/firmware/, which is committed in the nested initramfs repo so this
# tree stays self-contained.
gen_cpio_list() {
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
dir /lib 0755 0 0
dir /lib/firmware 0755 0 0
dir /lib/firmware/tp 0755 0 0
dir /lib/firmware/tp/24618 0755 0 0
# synaptics-tcm-spi.c has exactly one request_firmware(), for this file
# (it parses the panel id/max coordinates; flash_firmware defaults to off,
#  so it is never written back).  The _FAE / LIMIT / LIMIT_AGING siblings in
# the vendor 24618/ directory belong to the Android touch HAL and are never
# requested -- do not ship them again.
file /lib/firmware/tp/24618/FW_S3910_TIANMA_HBP.img $IRFS/firmware/tp/24618/FW_S3910_TIANMA_HBP.img 0644 0 0
dir /lib/firmware/arm 0755 0 0
dir /lib/firmware/arm/mali 0755 0 0
dir /lib/firmware/arm/mali/arch13.8 0755 0 0
file /lib/firmware/arm/mali/arch13.8/mali_csffw.bin $IRFS/firmware/arm/mali/arch13.8/mali_csffw.bin 0644 0 0
dir /lib/firmware/mediatek 0755 0 0
file /lib/firmware/BT_RAM_CODE_MT6653_2g2a_1_hdr.bin $IRFS/firmware/BT_RAM_CODE_MT6653_2g2a_1_hdr.bin 0644 0 0
file /lib/firmware/BT_RAM_CODE_MT6653_1_1_hdr.bin $IRFS/firmware/BT_RAM_CODE_MT6653_1_1_hdr.bin 0644 0 0
file /lib/firmware/BT_FW.cfg $IRFS/firmware/BT_FW.cfg 0644 0 0
file /lib/firmware/mediatek/BT_RAM_CODE_MT6653_2g2a_1_hdr.bin $IRFS/firmware/BT_RAM_CODE_MT6653_2g2a_1_hdr.bin 0644 0 0
file /lib/firmware/WIFI_RAM_CODE_MT6653_2g2a_1.bin $IRFS/firmware/WIFI_RAM_CODE_MT6653_2g2a_1.bin 0644 0 0
file /lib/firmware/WIFI_MT6653_PATCH_MCU_2g2a_1_hdr.bin $IRFS/firmware/WIFI_MT6653_PATCH_MCU_2g2a_1_hdr.bin 0644 0 0
file /lib/firmware/WIFI_MT6653_PHY_RAM_CODE_2g2a_1.bin $IRFS/firmware/WIFI_MT6653_PHY_RAM_CODE_2g2a_1.bin 0644 0 0
file /lib/firmware/conninfra.cfg $IRFS/firmware/conninfra.cfg 0644 0 0
file /lib/firmware/connfem.cfg $IRFS/firmware/connfem.cfg 0644 0 0
file /lib/firmware/mediatek/WIFI_RAM_CODE_MT6653_2g2a_1.bin $IRFS/firmware/WIFI_RAM_CODE_MT6653_2g2a_1.bin 0644 0 0
file /lib/firmware/mediatek/WIFI_MT6653_PATCH_MCU_2g2a_1_hdr.bin $IRFS/firmware/WIFI_MT6653_PATCH_MCU_2g2a_1_hdr.bin 0644 0 0
file /lib/firmware/mediatek/WIFI_MT6653_PHY_RAM_CODE_2g2a_1.bin $IRFS/firmware/WIFI_MT6653_PHY_RAM_CODE_2g2a_1.bin 0644 0 0
dir /lib/firmware/mediatek/mt6991 0755 0 0
file /lib/firmware/WIFI $IRFS/firmware/WIFI 0644 0 0
file /lib/firmware/mediatek/WIFI $IRFS/firmware/WIFI 0644 0 0
file /lib/firmware/mediatek/mt6991/WIFI $IRFS/firmware/WIFI 0644 0 0
file /lib/firmware/tfa98xx.cnt $IRFS/firmware/tfa98xx.cnt 0644 0 0
file /lib/firmware/md1drdi.bin $IRFS/firmware/md1drdi.bin 0644 0 0
file /init $IRFS/init 0755 0 0
EOF
	# Loadable modules, built by the module step below and cached at $K.
	echo "dir /lib/modules 0755 0 0" >> "$IRFS/cpio_list"
	for ko in scp.ko ccci_fsm_scp.ko connscp.ko clk-mt6991-drv.ko \
			gps_drv_dl_v062.ko sensorhub_probe.ko; do
		if [ -f "$K/$ko" ]; then
			echo "file /lib/modules/$ko $K/$ko 0644 0 0" >> "$IRFS/cpio_list"
		else
			echo "  WARNING: $ko missing from $K (not embedded)"
		fi
	done
}

echo "==> [1/6] static init"
# INIT_SRC picks the init source.  Default init-rootfs.c is the minimal
# rootfs-switch init; INIT_SRC=init.c builds the bring-up test init (the
# display/touch register tests, then a reboot to Android).
INIT_SRC=${INIT_SRC:-init-rootfs.c}
if [ "$INIT_SRC" = init-rootfs.c ]; then
	# init-rootfs.c is freestanding: it links against the in-tree minic
	# (no host libc at all), so it can be built with the same toolchain
	# as the kernel.  The bring-up init.c keeps its glibc path.
	"$HOST_CC" $TARGET_CC_FLAGS $TARGET_LD_FLAGS \
		-static -O2 -Wall -Wextra -ffreestanding -fno-builtin \
		-fno-stack-protector -fno-pie -nostdinc -nostdlib \
		-Wl,-no-pie -Wl,--build-id=none -I "$IRFS/minic/include" \
		-o "$IRFS/init" \
		"$IRFS/minic/crt0.S" "$IRFS/minic/syscalls.c" \
		"$IRFS/minic/string.c" "$IRFS/minic/stdio.c" \
		"$IRFS/minic/alloc.c" "$IRFS/minic/dirent.c" \
		"$IRFS/minic/misc.c" "$IRFS/$INIT_SRC"
else
	"$HOST_CC" $TARGET_CC_FLAGS $TARGET_LD_FLAGS \
		-static -O2 -Wall -Wextra -o "$IRFS/init" "$IRFS/$INIT_SRC"
fi
"$HOST_STRIP" "$IRFS/init"
ls -la "$IRFS/init"

echo "==> [2/6] config"
# CONFIG_INITRAMFS_SOURCE is generated here so the fragment stays path-free.
gen_cpio_list

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
	 CONFIG_DRM_PANEL_AE031_DSI_VDO=y \
	 CONFIG_NETFILTER=y CONFIG_NF_CONNTRACK=y CONFIG_NF_NAT=y \
	 CONFIG_NF_TABLES=y CONFIG_NF_TABLES_INET=y \
	 CONFIG_NF_TABLES_IPV4=y CONFIG_NFT_CT=y CONFIG_NFT_NAT=y \
	 CONFIG_NFT_MASQ=y; do
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

echo "==> [3/6] DTB"
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

echo "==> [4/6] kernel modules (scp, ccci_fsm_scp, connscp, clk-mt6991-drv, gps, sensorhub_probe)"
# The external-module builds link against the tree's Module.symvers, which a
# kernel link produces.  On an already-built tree it is present; on a first
# build, produce vmlinux once now (with a first-cut initramfs list) so the
# module builds have something to link against.
if [ ! -f "$OUT/vmlinux" ]; then
	echo "  (no vmlinux yet; building it first)"
	ARCH=arm64 make O="$OUT" -j"$JOBS" vmlinux
fi

# kbuild only emits vmlinux.symvers unless a 'make modules' pass runs.  The
# external module builds here only need the vmlinux exports (mbox/ipi/...),
# which vmlinux.symvers already carries, so make it visible as Module.symvers.
if [ ! -f "$OUT/Module.symvers" ] && [ -f "$OUT/vmlinux.symvers" ]; then
	cp "$OUT/vmlinux.symvers" "$OUT/Module.symvers"
fi

# include/generated/utsrelease.h carries UTS_RELEASE, which both the kernel
# banner (init/version.o) and every module's vermagic are compiled from.
# kbuild's rule is "utsrelease.h: include/config/kernel.release FORCE" and
# filechk only rewrites the file when the string changes, but an external
# module build (make M=<dir> modules) has no dependency on that rule, so the
# .ko get linked from whatever the header happened to hold -- after a commit
# that is the previous release, and the running kernel then refuses the module
# with -ENOEXEC.  Write the header explicitly (exactly what filechk produces,
# including its 64-char cap) before anything is compiled, so the Image and
# every .ko carry the same string.
want="$(make -s O="$OUT" kernelrelease)"
if [ ${#want} -gt 64 ]; then
	echo "  FAILED: '$want' exceeds 64 characters"; exit 1
fi
if [ "$(cat "$OUT/include/generated/utsrelease.h" 2>/dev/null)" \
	!= "#define UTS_RELEASE \"$want\"" ]; then
	mkdir -p "$OUT/include/generated"
	printf '#define UTS_RELEASE "%s"\n' "$want" \
		> "$OUT/include/generated/utsrelease.h"
	echo "  utsrelease.h refreshed to $want"
fi

# External modules embed the vermagic of the kernel release they were built
# against, and every commit changes $(git describe) -> UTS_RELEASE.  kbuild
# considers an unchanged .o up to date and never re-runs modpost, so a .ko
# cached at the repo root keeps the OLD release and the running kernel then
# refuses it: init_module() -> -ENOEXEC -> "Exec format error" in the init
# log, and with scp.ko gone the SCP never comes up so the modem stalls at HS1.
# Compare the cached .ko against the release we just built and drop its
# modpost inputs so kbuild relinks with the current string.
want="$want SMP preempt mod_unload aarch64"
relink_if_stale() {
	local mdir=$1 mko=$2 ko
	ko=$(find "$mdir" -name "$mko" -print -quit 2>/dev/null || true)
	[ -n "$ko" ] || return 0
	if grep -q -a -F "$want" "$ko"; then
		return 0
	fi
	echo "  stale $mko (vermagic != $want) -> relinking"
	find "$mdir" \( -name '*.mod.c' -o -name '*.mod.o' \
		-o -name "$mko" -o -name 'modules.order' \) -exec rm -f {} +
}

# scripts/Makefile.modfinal's "%.ko" pattern rule lists
# $(objtree)/scripts/module.lds as a prerequisite.  Under O=out that file only
# appears via 'make modules_prepare' (Makefile: `modules_prepare: prepare`),
# and with no rule for it inside Makefile.modfinal itself its absence silently
# rules out the pattern -> "No rule to make target '<mod>.ko', needed by
# '__modfinal'".  modules_prepare is too heavy here, so preprocess the one
# linker script directly with the same cpp invocation the kernel uses (the
# include list is LINUXINCLUDE in O=out form), which reproduces the in-tree
# scripts/module.lds byte for byte.
if [ ! -f "$OUT/scripts/module.lds" ]; then
	mkdir -p "$OUT/scripts"
	"$HOST_CC" $TARGET_CC_FLAGS -E -nostdinc \
		-I arch/arm64/include -I "$OUT"/arch/arm64/include/generated \
		-I include -I "$OUT"/include \
		-I arch/arm64/include/uapi -I "$OUT"/arch/arm64/include/generated/uapi \
		-I include/uapi -I "$OUT"/include/generated/uapi \
		-include include/linux/compiler-version.h -include include/linux/kconfig.h \
		-D__KERNEL__ -mlittle-endian -DKASAN_SHADOW_SCALE_SHIFT= \
		-P -Uarm64 -D__ASSEMBLY__ -DLINKER_SCRIPT \
		-o "$OUT/scripts/module.lds" scripts/module.lds.S
	echo "  generated $OUT/scripts/module.lds"
fi

# Each spec is "<make -M dir>:<module.ko>".  KBUILD_MODPOST_WARN lets the .ko
# carry the (exported by vmlinux) mbox/ipi symbols unresolved; they are
# resolved against vmlinux at insmod time.  This avoids a full 'make modules'
# (which build.sh does not otherwise do).
for spec in \
	"drivers/misc/mediatek/scp:scp.ko" \
	"drivers/misc/mediatek/eccci/fsm:ccci_fsm_scp.ko" \
	"drivers/misc/mediatek/conn_scp:connscp.ko" \
	"drivers/misc/mediatek/sensorhub_probe:sensorhub_probe.ko" \
	"drivers/clk/mediatek/mt6991:clk-mt6991-drv.ko"; do
	mdir=${spec%%:*}; mko=${spec##*:}
	relink_if_stale "$mdir" "$mko"
	ARCH=arm64 make O="$OUT" -j"$JOBS" KBUILD_MODPOST_WARN=1 \
		M="$mdir" modules 2>&1 | tail -3
	# With O=out the external-module .ko still lands in-tree under $mdir.
	# -print -quit + "|| true": under 'set -euo pipefail' neither a piped
	# head (SIGPIPE on find) nor find's own exit 1 (a $OUT/$mdir that does
	# not exist for this module) may abort the script.
	ko=$(find "$OUT/$mdir" "$mdir" -name "$mko" -print -quit 2>/dev/null || true)
	if [ -n "$ko" ]; then
		cp "$ko" "$K/$mko"
		echo "  cached $ko -> $K/$mko"
	else
		echo "  WARNING: $mko not found ($mdir)"
	fi
done

# gps_drv_dl_v062.ko ---
# Vendor MT6991 GPS data-link driver (module).  Its Kbuild is gated on
# CONFIG_MTK_GPS_SUPPORT=y and resolves the built-in conninfra API through the
# kernel symvers, so refresh Module.symvers from vmlinux.symvers first (the
# in-tree Module.symvers is only regenerated by 'make modules').
if [ -f "$OUT/vmlinux.symvers" ]; then
	cp -f "$OUT/vmlinux.symvers" "$OUT/Module.symvers"
fi
ARCH=arm64 make O="$OUT" -j"$JOBS" KBUILD_MODPOST_WARN=1 CONFIG_MTK_GPS_SUPPORT=y \
	M=drivers/misc/mediatek/connectivity/gps/data_link/plat/v062 modules 2>&1 | tail -3
mdir=drivers/misc/mediatek/connectivity/gps/data_link/plat/v062
relink_if_stale "$mdir" gps_drv_dl_v062.ko
ko=$(find "$OUT/$mdir" "$mdir" -name 'gps_drv_dl_v062.ko' -print -quit 2>/dev/null || true)
if [ -n "$ko" ]; then
	cp "$ko" "$K/gps_drv_dl_v062.ko"
	echo "  cached $ko -> $K/gps_drv_dl_v062.ko"
else
	echo "  WARNING: gps_drv_dl_v062.ko not found"
fi

echo "==> [5/6] initramfs cpio_list"
# Regenerate now that this run's .ko are cached, then link.
gen_cpio_list
cat "$IRFS/cpio_list"

echo "==> [6/6] kernel Image (this is the long part)"
ARCH=arm64 make O="$OUT" -j"$JOBS" Image

# gen_init_cpio only *warns* (exit 0) about lines it cannot parse and silently
# drops them; if /init is dropped the image has no init and the kernel panics
# with "VFS: Cannot open root device".  Re-run it and fail on any error.
if [ -x "$OUT/usr/gen_init_cpio" ]; then
	err=$("$OUT/usr/gen_init_cpio" "$IRFS/cpio_list" 2>&1 >/dev/null)
	if [ -n "$err" ]; then
		echo "  FAILED: gen_init_cpio reported errors (cpio entry dropped?):"
		echo "$err"
		exit 1
	fi
	echo "  ok      initramfs cpio_list parsed cleanly"
fi

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
