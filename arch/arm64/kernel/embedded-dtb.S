/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Wrap a device tree blob into the kernel image.
 *
 * This deliberately lives in arch/arm64/kernel/ rather than under
 * arch/arm64/boot/dts/: the DT directories are not part of arm64's obj-y
 * (see arch/arm64/Kbuild, which only descends into kernel/ mm/ net/), so a
 * .dtb.o built there would never be linked into vmlinux.
 *
 * The blob goes into .rodata, which arm64 maps read-only and never frees, so
 * initial_boot_params stays valid for the lifetime of the kernel - including
 * the late_initcall that publishes /sys/firmware/fdt.
 *
 * STRUCT_ALIGNMENT (32) comfortably covers the 8-byte alignment libfdt and
 * setup_machine_fdt() expect of a blob.
 */

#include <asm-generic/vmlinux.lds.h>

	.section .rodata, "a"
	.balign STRUCT_ALIGNMENT
	.global __embedded_dtb_start
__embedded_dtb_start:
	.incbin CONFIG_ARM64_EMBEDDED_DTB_SOURCE
	.global __embedded_dtb_end
__embedded_dtb_end:
	.balign STRUCT_ALIGNMENT
