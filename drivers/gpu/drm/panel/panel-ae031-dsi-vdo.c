// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Mainline Port
// DRM driver for ae031_p_3_a0026 DSI video mode panel
// realme GT7 (RMX6688) / MT6991

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/display/drm_dsc.h>
#include <drm/drm_probe_helper.h>

#define AE031_NUM_SUPPLIES 3

struct ae031 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct regulator_bulk_data supplies[AE031_NUM_SUPPLIES];
	struct gpio_desc *reset_gpio;
	bool boot_handoff_done; /* bootloader already initialised the panel */
};

static const char * const ae031_supply_names[AE031_NUM_SUPPLIES] = {
	"vddio",
	"vddr",
	"vci",
};

static inline struct ae031 *to_ae031(struct drm_panel *panel)
{
	return container_of(panel, struct ae031, panel);
}

/*
 * Init sequence extracted from vendor FDT:
 * oplus,panel_ae031_p_3_a0026_dsi_vdo
 *
 * 134 commands total, sent in LP mode.
 * Only timing-specific commands (cmd 47: 2f XX) differ per refresh rate.
 * This is the 120Hz init (2f 01); the DRM mode is 120Hz.
 */
static int ae031_init(struct ae031 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	/* Vendor 120Hz on-command (134 commands, panel-ae031-p-3-a0026) */
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc3, 0x0a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xea, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xea, 0xc0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x07);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xea, 0x01, 0x02, 0x01, 0x34, 0x01, 0x34, 0x01, 0x34, 0x04, 0xd1);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x18);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xea, 0x01, 0x79, 0x01, 0xc4, 0x01, 0xc4, 0x01, 0xc4, 0x07, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x24);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf8, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xc0, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xd2, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf8, 0x02);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x84);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0xa9);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0xf3);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2a, 0x00, 0x00, 0x04, 0xff);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2b, 0x00, 0x00, 0x0a, 0xef);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x90, 0x43);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x91, 0xab, 0xa8, 0x00, 0x28, 0xd2, 0x00, 0x02, 0x5c, 0x04, 0x06, 0x00, 0x08, 0x02, 0xab, 0x02, 0x20, 0x10, 0xe0);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x53, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0x00, 0x1c, 0x00, 0x74, 0x00, 0x1c, 0x00, 0x7c, 0x00, 0x1c, 0x04, 0x54, 0x00, 0x1c, 0x00, 0x7c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x3b, 0x00, 0x1c, 0x00, 0x7c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x81, 0x01, 0x19);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x88, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x2f, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x35, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x5f, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x03);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb5, 0xd5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf9, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x20);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf4, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xfc, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x08);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0, 0x41, 0x01, 0x01, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe0, 0x00, 0x01, 0x01, 0x2c, 0x01, 0x9b, 0x02, 0x52, 0x03, 0xcb, 0x04, 0x10, 0x04, 0x40, 0x08, 0x51, 0x0b, 0x6a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x06, 0x0d, 0x30, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x1a, 0x15, 0x0c, 0x07, 0x07, 0x0c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x0a, 0x15, 0x30, 0x2a, 0x2a, 0x2a, 0x2a, 0x2a, 0x23, 0x1d, 0x12, 0x0e, 0x0d, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3, 0x00, 0x00, 0x00, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x07, 0x08, 0x0b, 0x06);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x04, 0x08, 0x20, 0x0f, 0x0f, 0x0f, 0x0f, 0x0f, 0x07, 0x07, 0x06, 0x06, 0x09, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x05, 0x0a, 0x20, 0x13, 0x13, 0x13, 0x13, 0x13, 0x0a, 0x07, 0x06, 0x06, 0x07, 0x09);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0a, 0x0b, 0x0e, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x02, 0x04, 0x1b, 0x07, 0x07, 0x07, 0x07, 0x07, 0x04, 0x03, 0x06, 0x07, 0x0b, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x03, 0x06, 0x1b, 0x0d, 0x0d, 0x0c, 0x0c, 0x0c, 0x06, 0x06, 0x06, 0x07, 0x09, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x1c);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x0a, 0x0b, 0x0d, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x2a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x02, 0x04, 0x11, 0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x05, 0x08, 0x09, 0x0c, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x2a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x03, 0x06, 0x11, 0x0d, 0x0b, 0x0a, 0x08, 0x06, 0x04, 0x05, 0x06, 0x08, 0x0a, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x2a);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x07, 0x09, 0x0c, 0x0f, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x38);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x02, 0x04, 0x0b, 0x07, 0x06, 0x05, 0x04, 0x03, 0x05, 0x08, 0x0b, 0x0c, 0x0d, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x38);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x02, 0x04, 0x0b, 0x08, 0x07, 0x06, 0x05, 0x04, 0x05, 0x07, 0x09, 0x0b, 0x0c, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x38);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3, 0x00, 0x00, 0x00, 0x02, 0x02, 0x02, 0x02, 0x02, 0x05, 0x08, 0x09, 0x0a, 0x0c, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x46);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x02, 0x03, 0x09, 0x06, 0x06, 0x05, 0x04, 0x04, 0x06, 0x09, 0x0a, 0x0b, 0x0d, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x46);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x02, 0x03, 0x09, 0x06, 0x06, 0x05, 0x04, 0x04, 0x05, 0x07, 0x09, 0x0b, 0x0b, 0x0e);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x46);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x09, 0x0a, 0x0b, 0x0c, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x54);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x02, 0x03, 0x08, 0x06, 0x06, 0x05, 0x04, 0x04, 0x06, 0x0a, 0x0b, 0x0c, 0x0e, 0x11);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x54);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x02, 0x04, 0x08, 0x08, 0x07, 0x06, 0x05, 0x04, 0x06, 0x08, 0x0a, 0x0b, 0x0c, 0x0f);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x54);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3, 0x00, 0x00, 0x00, 0x03, 0x03, 0x04, 0x04, 0x04, 0x06, 0x09, 0x0a, 0x0b, 0x0d, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x62);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x00, 0x01, 0x03, 0x02, 0x02, 0x03, 0x04, 0x04, 0x07, 0x08, 0x0a, 0x0c, 0x0e, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x62);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x00, 0x01, 0x03, 0x02, 0x02, 0x03, 0x04, 0x04, 0x06, 0x08, 0x09, 0x0b, 0x0d, 0x10);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x62);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3, 0x00, 0x00, 0x00, 0x02, 0x02, 0x03, 0x04, 0x04, 0x06, 0x09, 0x0c, 0x0e, 0x11, 0x16);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe1, 0x00, 0x01, 0x02, 0x02, 0x02, 0x03, 0x04, 0x04, 0x06, 0x08, 0x0b, 0x0d, 0x0f, 0x14);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe2, 0x00, 0x01, 0x02, 0x02, 0x02, 0x03, 0x04, 0x04, 0x05, 0x07, 0x09, 0x0c, 0x0f, 0x13);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x70);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xe3, 0x00, 0x00, 0x00, 0x02, 0x02, 0x03, 0x04, 0x04, 0x06, 0x09, 0x0b, 0x0f, 0x11, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf0, 0x55, 0xaa, 0x52, 0x08, 0x01);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb0, 0x37, 0x37);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb2, 0xaa, 0x2a, 0x55, 0x15);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xb7, 0x2d, 0x2d, 0x2d, 0x2d, 0x2d, 0x00, 0x2d);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa9, 0x02, 0x00, 0xc0, 0x0b, 0x0c, 0x64, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x00, 0x00);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x04);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x51, 0x0f, 0xfe);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xff, 0xaa, 0x55, 0xa5, 0x80);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x6f, 0x47);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xf2, 0x21);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x11);
	mipi_dsi_msleep(&dsi_ctx, 120);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0x29);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, 0xa9, 0x01, 0x00, 0x2f, 0x00, 0x00, 0x01);

	return dsi_ctx.accum_err;
}

static int ae031_off(struct ae031 *ctx)
{
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	/* Vendor off-command: DISPLAY_OFF + 5ms, SLEEP_IN + 120ms (LP mode) */
	ctx->dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_SET_DISPLAY_OFF);
	mipi_dsi_msleep(&dsi_ctx, 5);
	mipi_dsi_dcs_write_seq_multi(&dsi_ctx, MIPI_DCS_ENTER_SLEEP_MODE);
	mipi_dsi_msleep(&dsi_ctx, 120);

	return dsi_ctx.accum_err;
}

static int ae031_prepare(struct drm_panel *panel)
{
	struct ae031 *ctx = to_ae031(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	/*
	 * The bootloader already powered on and initialised the panel.  On the
	 * first prepare, skip the power-up/init sequence, mark the handoff done
	 * and take a reference on the supplies so the driver's enable/disable
	 * counts stay balanced when a DPMS cycle powers them down later.
	 * The flag is never cleared, so every later enable runs the real
	 * power-up + reset + init sequence.
	 */
	if (!ctx->boot_handoff_done) {
		ctx->boot_handoff_done = true;
		ret = regulator_enable(ctx->supplies[0].consumer); /* vddio */
		if (!ret)
			ret = regulator_enable(ctx->supplies[2].consumer); /* vci */
		if (!ret)
			ret = regulator_enable(ctx->supplies[1].consumer); /* vddr */
		if (ret)
			dev_warn(dev, "Failed to take supply reference: %d\n",
				 ret);
		return 0;
	}

	/* Power sequence: vddio -> vci -> vddr */
	ret = regulator_enable(ctx->supplies[0].consumer); /* vddio */
	if (ret < 0) {
		dev_err(dev, "Failed to enable vddio: %d\n", ret);
		return ret;
	}
	msleep(3);

	ret = regulator_enable(ctx->supplies[2].consumer); /* vci */
	if (ret < 0) {
		dev_err(dev, "Failed to enable vci: %d\n", ret);
		goto err_vci;
	}
	msleep(3);

	ret = regulator_enable(ctx->supplies[1].consumer); /* vddr */
	if (ret < 0) {
		dev_err(dev, "Failed to enable vddr: %d\n", ret);
		goto err_vddr;
	}
	msleep(10);

	/*
	 * Vendor reset sequence <2 10> <1 1> <0 1> <1 20>:
	 * initial 10ms delay, then high 1ms, low 1ms, and finally high
	 * (released) with a 20ms panel boot delay.  The pin must stay high
	 * while the init sequence runs.
	 */
	msleep(10);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(1);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	msleep(1);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	msleep(20);

	ret = ae031_init(ctx);
	if (ret < 0) {
		dev_err(dev, "Failed to initialize panel: %d\n", ret);
		goto err_init;
	}

	return 0;

err_init:
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	regulator_disable(ctx->supplies[1].consumer); /* vddr */
err_vddr:
	regulator_disable(ctx->supplies[2].consumer); /* vci */
err_vci:
	regulator_disable(ctx->supplies[0].consumer); /* vddio */
	return ret;
}

static int ae031_unprepare(struct drm_panel *panel)
{
	struct ae031 *ctx = to_ae031(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;


	ret = ae031_off(ctx);
	if (ret < 0)
		dev_err(dev, "Failed to un-initialize panel: %d\n", ret);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	/* Power off sequence: vddr -> vci -> vddio */
	regulator_disable(ctx->supplies[1].consumer); /* vddr */
	msleep(3);
	regulator_disable(ctx->supplies[2].consumer); /* vci */
	msleep(3);
	regulator_disable(ctx->supplies[0].consumer); /* vddio */

	return 0;
}

/*
 * Panel timing: 1280x2800 @ 120Hz
 * H: 1280 + 112(fp) + 4(sa) + 4(bp) = 1400
 * V: 2800 + 124(fp) + 2(sa) + 26(bp) = 2952
 * Clock: ~496 MHz
 *
 * The bootloader leaves the panel in its 120Hz timing (vfp = 124); the
 * panel only accepts the DSI stream that matches its current refresh-rate
 * mode unless the "2f xx" timing-switch command is sent first.
 */
static const struct drm_display_mode ae031_mode = {
	.clock = 496000,
	.hdisplay = 1280,
	.hsync_start = 1280 + 112,
	.hsync_end = 1280 + 112 + 4,
	.htotal = 1280 + 112 + 4 + 4,
	.vdisplay = 2800,
	.vsync_start = 2800 + 124,
	.vsync_end = 2800 + 124 + 2,
	.vtotal = 2800 + 124 + 2 + 26,
	.width_mm = 68,   /* 11800um */
	.height_mm = 153,  /* 26598um */
	.type = DRM_MODE_TYPE_DRIVER,
};

static int ae031_get_modes(struct drm_panel *panel,
			   struct drm_connector *connector)
{
	struct drm_display_mode *mode;


	mode = drm_mode_duplicate(connector->dev, &ae031_mode);
	if (!mode)
		return -ENOMEM;

	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_panel_funcs ae031_panel_funcs = {
	.prepare = ae031_prepare,
	.unprepare = ae031_unprepare,
	.get_modes = ae031_get_modes,
};

static int ae031_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ae031 *ctx;
	int ret;


	ctx = devm_drm_panel_alloc(dev, struct ae031, panel,
				   &ae031_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	/* Get regulators */
	ctx->supplies[0].supply = ae031_supply_names[0];
	ctx->supplies[1].supply = ae031_supply_names[1];
	ctx->supplies[2].supply = ae031_supply_names[2];
	ret = devm_regulator_bulk_get(dev, AE031_NUM_SUPPLIES,
				      ctx->supplies);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	/* Get reset GPIO */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "Failed to get reset GPIO\n");

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	/* DSI configuration: 4 lanes, RGB888, Video Burst */
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS;

	/* DSC configuration: VESA DSC 1.2, 10bpc, 8bpp */
	dsi->dsc = devm_kzalloc(dev, sizeof(*dsi->dsc), GFP_KERNEL);
	if (dsi->dsc) {
		dsi->dsc->dsc_version_major = 1;
		dsi->dsc->dsc_version_minor = 2;
		dsi->dsc->bits_per_component = 10;
		dsi->dsc->bits_per_pixel = 128;
		dsi->dsc->slice_height = 40;
		dsi->dsc->slice_width = 640;
		dsi->dsc->pic_width = 1280;
		dsi->dsc->pic_height = 2800;
		dsi->dsc->rc_tgt_offset_high = 3;
		dsi->dsc->rc_tgt_offset_low = 3;
		dsi->dsc->rc_model_size = 8192;
		dsi->dsc->rc_edge_factor = 6;
		dsi->dsc->rc_quant_incr_limit0 = 15;
		dsi->dsc->rc_quant_incr_limit1 = 15;
		dsi->dsc->initial_offset = 6144;
		dsi->dsc->flatness_min_qp = 7;
		dsi->dsc->flatness_max_qp = 16;
		dsi->dsc->nfl_bpg_offset = 683;
		dsi->dsc->slice_bpg_offset = 544;
		dsi->dsc->final_offset = 4320;
		dsi->dsc->initial_xmit_delay = 512;
		dsi->dsc->initial_dec_delay = 604;
		dsi->dsc->initial_scale_value = 32;
		dsi->dsc->scale_increment_interval = 1030;
		dsi->dsc->scale_decrement_interval = 8;
		dsi->dsc->line_buf_depth = 11;
		dsi->dsc->block_pred_enable = true;
		dsi->dsc->slice_chunk_size = 640;
		dsi->dsc->slice_count = 2;
	}

	ctx->panel.prepare_prev_first = true;

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get backlight\n");

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "Failed to attach to DSI host\n");
	}

	return 0;
}

static void ae031_remove(struct mipi_dsi_device *dsi)
{
	struct ae031 *ctx = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "Failed to detach from DSI host: %d\n", ret);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ae031_of_match[] = {
	{ .compatible = "oplus,panel-ae031-p-3-a0026-dsi-vdo" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ae031_of_match);

static struct mipi_dsi_driver ae031_driver = {
	.probe = ae031_probe,
	.remove = ae031_remove,
	.driver = {
		.name = "panel-ae031-dsi-vdo",
		.of_match_table = ae031_of_match,
	},
};
module_mipi_dsi_driver(ae031_driver);

MODULE_AUTHOR("Mainline Port");
MODULE_DESCRIPTION("DRM driver for ae031 P_3 A0026 DSI video mode panel");
MODULE_LICENSE("GPL");
