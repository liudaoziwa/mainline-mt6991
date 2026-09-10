// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek eUSB2 NXP Repeater Driver
 *
 * Ported from vendor kernel for realme GT7 (RMX6688) / MT6991 mainline bring-up.
 * Original Copyright (c) 2023 MediaTek Inc.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinctrl-state.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/bits.h>

#define LINK_CONTROL_1			0x02
#define ROLE_CONTROL			GENMASK(5, 4)
#define ROLE_CONTROL_SHIFT		4
#define ROLE_FORCE_DUAL			0x0
#define ROLE_FORCE_HOST			0x1
#define ROLE_FORCE_DEVICE		0x2

#define EUSB2RXCONTROL			0x04
#define EUSB2_RXSQUELCH			GENMASK(5, 4)
#define EUSB2_RXSQUELCH_SHIFT		4
#define EUSB2_RXEQUALIZATION		GENMASK(2, 0)
#define EUSB2_RXEQUALIZATION_SHIFT	0

#define EUSB2TXCONTROL			0x05
#define EUSB2_TXOUTPUTSWING		GENMASK(5, 4)
#define EUSB2_TXOUTPUTSWING_SHIFT	4
#define EUSB2_TXDEEMPHASIS		GENMASK(1, 0)
#define EUSB2_TXDEEMPHASIS_SHIFT	0

#define USB2RXCONTROL			0x06
#define RXSQUELCH			GENMASK(6, 4)
#define RXSQUELCH_SHIFT			4
#define RXEQUALIZATION			GENMASK(2, 0)
#define RXEQUALIZATION_SHIFT		0

#define USB2TXCONTROL1			0x07
#define TXDEEMPHASIS			GENMASK(2, 0)
#define TXDEEMPHASIS_SHIFT		0

#define USB2TXCONTROL2			0x08
#define FSRISEFALLTIME			BIT(6)
#define FSRISEFALLTIME_SHIFT		6
#define HSRISEFALLTIME			GENMASK(5, 4)
#define HSRISEFALLTIME_SHIFT		4
#define TXOUTPUTSWING			GENMASK(2, 0)
#define TXOUTPUTSWING_SHIFT		0

#define USB2HSTERMINATION		0x09
#define HSTERMINATION			GENMASK(2, 0)
#define HSTERMINATION_SHIFT		0

#define USB2HSDISCH			0x0A
#define HSDISCH				GENMASK(1, 0)
#define HSDISCH_SHIFT			0

#define NXP_REPEATER_CID		0x3222
#define CHIP_ID_0			0x14
#define CHIP_ID_1			0x15

struct eusb2_repeater {
	struct device *dev;
	struct i2c_client *i2c;
	struct phy *phy;
	struct pinctrl *pinctrl;
	struct pinctrl_state *enable;
	struct pinctrl_state *disable;
	int eusb2_squelch;
	int eusb2_equalization;
	int eusb2_output_swing;
	int eusb2_deemphasis;
	int squelch;
	int equalization;
	int ouput_swing;
	int deemphasis;
	int hs_termination;
	int discth;
	int fs_rise_fall;
	int hs_rise_fall;
	enum phy_mode mode;
};

static void eusb2_rptr_prop_parse(struct eusb2_repeater *rptr)
{
	struct device *dev = rptr->dev;

	device_property_read_u32(dev, "mediatek,rptr-eusb2-squelch",
				 &rptr->eusb2_squelch);
	device_property_read_u32(dev, "mediatek,rptr-eusb2-equalization",
				 &rptr->eusb2_equalization);
	device_property_read_u32(dev, "mediatek,rptr-eusb2-output-swing",
				 &rptr->eusb2_output_swing);
	device_property_read_u32(dev, "mediatek,rptr-eusb2-deemphasis",
				 &rptr->eusb2_deemphasis);
	device_property_read_u32(dev, "mediatek,rptr-squelch",
				 &rptr->squelch);
	device_property_read_u32(dev, "mediatek,rptr-equalization",
				 &rptr->equalization);
	device_property_read_u32(dev, "mediatek,rptr-ouput-swing",
				 &rptr->ouput_swing);
	device_property_read_u32(dev, "mediatek,rptr-demphasis",
				 &rptr->deemphasis);
	device_property_read_u32(dev, "mediatek,rptr-hs-termination",
				 &rptr->hs_termination);
	device_property_read_u32(dev, "mediatek,rptr-discth",
				 &rptr->discth);
	device_property_read_u32(dev, "mediatek,rptr-fs-rise-fall",
				 &rptr->fs_rise_fall);
	device_property_read_u32(dev, "mediatek,rptr-hs-rise-fall",
				 &rptr->hs_rise_fall);
}

static void eusb2_rptr_prop_set(struct eusb2_repeater *rptr)
{
	u8 val;

	if (rptr->eusb2_squelch) {
		val = i2c_smbus_read_byte_data(rptr->i2c, EUSB2RXCONTROL);
		val &= ~EUSB2_RXSQUELCH;
		val |= (rptr->eusb2_squelch << EUSB2_RXSQUELCH_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, EUSB2RXCONTROL, val);
	}

	if (rptr->eusb2_equalization) {
		val = i2c_smbus_read_byte_data(rptr->i2c, EUSB2RXCONTROL);
		val &= ~EUSB2_RXEQUALIZATION;
		val |= (rptr->eusb2_equalization << EUSB2_RXEQUALIZATION_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, EUSB2RXCONTROL, val);
	}

	if (rptr->eusb2_output_swing) {
		val = i2c_smbus_read_byte_data(rptr->i2c, EUSB2TXCONTROL);
		val &= ~EUSB2_TXOUTPUTSWING;
		val |= (rptr->eusb2_output_swing << EUSB2_TXOUTPUTSWING_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, EUSB2TXCONTROL, val);
	}

	if (rptr->eusb2_deemphasis) {
		val = i2c_smbus_read_byte_data(rptr->i2c, EUSB2TXCONTROL);
		val &= ~EUSB2_TXDEEMPHASIS;
		val |= (rptr->eusb2_deemphasis << EUSB2_TXDEEMPHASIS_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, EUSB2TXCONTROL, val);
	}

	if (rptr->squelch) {
		val = i2c_smbus_read_byte_data(rptr->i2c, USB2RXCONTROL);
		val &= ~RXSQUELCH;
		val |= (rptr->squelch << RXSQUELCH_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, USB2RXCONTROL, val);
	}

	if (rptr->equalization) {
		val = i2c_smbus_read_byte_data(rptr->i2c, USB2RXCONTROL);
		val &= ~RXEQUALIZATION;
		val |= (rptr->equalization << RXEQUALIZATION_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, USB2RXCONTROL, val);
	}

	if (rptr->ouput_swing) {
		val = i2c_smbus_read_byte_data(rptr->i2c, USB2TXCONTROL2);
		val &= ~TXOUTPUTSWING;
		val |= (rptr->ouput_swing << TXOUTPUTSWING_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, USB2TXCONTROL2, val);
	}

	if (rptr->deemphasis) {
		val = i2c_smbus_read_byte_data(rptr->i2c, USB2TXCONTROL1);
		val &= ~TXDEEMPHASIS;
		val |= (rptr->deemphasis << TXDEEMPHASIS_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, USB2TXCONTROL1, val);
	}

	if (rptr->hs_termination) {
		val = i2c_smbus_read_byte_data(rptr->i2c, USB2HSTERMINATION);
		val &= ~HSTERMINATION;
		val |= (rptr->hs_termination << HSTERMINATION_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, USB2HSTERMINATION, val);
	}

	if (rptr->discth) {
		val = i2c_smbus_read_byte_data(rptr->i2c, USB2HSDISCH);
		val &= ~HSDISCH;
		val |= (rptr->discth << HSDISCH_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, USB2HSDISCH, val);
	}

	if (rptr->fs_rise_fall) {
		val = i2c_smbus_read_byte_data(rptr->i2c, USB2TXCONTROL2);
		val &= ~FSRISEFALLTIME;
		val |= (rptr->fs_rise_fall << FSRISEFALLTIME_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, USB2TXCONTROL2, val);
	}

	if (rptr->hs_rise_fall) {
		val = i2c_smbus_read_byte_data(rptr->i2c, USB2TXCONTROL2);
		val &= ~HSRISEFALLTIME;
		val |= (rptr->hs_rise_fall << HSRISEFALLTIME_SHIFT);
		i2c_smbus_write_byte_data(rptr->i2c, USB2TXCONTROL2, val);
	}
}

static bool eusb2_repeater_getcid(struct eusb2_repeater *rptr)
{
	u16 val;
	int ret;

	if (rptr->pinctrl && rptr->disable) {
		pinctrl_select_state(rptr->pinctrl, rptr->disable);
		udelay(10);
	}

	if (rptr->pinctrl && rptr->enable) {
		pinctrl_select_state(rptr->pinctrl, rptr->enable);
		mdelay(4);
	}

	ret = i2c_smbus_read_byte_data(rptr->i2c, CHIP_ID_0);
	if (ret < 0) {
		dev_err(rptr->dev, "failed to read chip id0: %d\n", ret);
		return false;
	}
	val = (u8)ret << 8;

	ret = i2c_smbus_read_byte_data(rptr->i2c, CHIP_ID_1);
	if (ret < 0) {
		dev_err(rptr->dev, "failed to read chip id1: %d\n", ret);
		return false;
	}

	val += (u8)ret;

	if (val != NXP_REPEATER_CID) {
		dev_err(rptr->dev, "unexpected chip id: 0x%04x (expected 0x%04x)\n",
			val, NXP_REPEATER_CID);
		return false;
	}

	dev_info(rptr->dev, "NXP eUSB2 repeater detected, CID: 0x%04x\n", val);
	return true;
}

static int eusb2_repeater_init(struct phy *phy)
{
	struct eusb2_repeater *rptr = phy_get_drvdata(phy);

	if (rptr->pinctrl && rptr->disable) {
		pinctrl_select_state(rptr->pinctrl, rptr->disable);
		udelay(10);
	}

	if (rptr->pinctrl && rptr->enable) {
		pinctrl_select_state(rptr->pinctrl, rptr->enable);
		mdelay(4);
	}

	return 0;
}

static int eusb2_repeater_exit(struct phy *phy)
{
	struct eusb2_repeater *rptr = phy_get_drvdata(phy);

	if (rptr->pinctrl && rptr->disable)
		pinctrl_select_state(rptr->pinctrl, rptr->disable);

	return 0;
}

static int eusb2_repeater_power_on(struct phy *phy)
{
	struct eusb2_repeater *rptr = phy_get_drvdata(phy);

	eusb2_rptr_prop_set(rptr);

	return 0;
}

static int eusb2_repeater_power_off(struct phy *phy)
{
	return 0;
}

static int eusb2_repeater_set_mode(struct phy *phy,
				   enum phy_mode mode, int submode)
{
	struct eusb2_repeater *rptr = phy_get_drvdata(phy);
	u8 val;

	val = i2c_smbus_read_byte_data(rptr->i2c, LINK_CONTROL_1);
	val &= ~ROLE_CONTROL;

	switch (mode) {
	case PHY_MODE_USB_HOST:
		val |= ROLE_FORCE_HOST << ROLE_CONTROL_SHIFT;
		break;
	case PHY_MODE_USB_DEVICE:
		val |= ROLE_FORCE_DEVICE << ROLE_CONTROL_SHIFT;
		break;
	default:
		val |= ROLE_FORCE_DUAL << ROLE_CONTROL_SHIFT;
		break;
	}

	i2c_smbus_write_byte_data(rptr->i2c, LINK_CONTROL_1, val);

	return 0;
}

static const struct phy_ops eusb2_repeater_ops = {
	.init		= eusb2_repeater_init,
	.exit		= eusb2_repeater_exit,
	.power_on	= eusb2_repeater_power_on,
	.power_off	= eusb2_repeater_power_off,
	.set_mode	= eusb2_repeater_set_mode,
	.owner		= THIS_MODULE,
};

static int eusb2_repeater_probe(struct i2c_client *client)
{
	struct eusb2_repeater *rptr;
	struct device *dev = &client->dev;
	struct phy_provider *phy_provider;
	struct device_node *np = dev->of_node;
	int ret;

	rptr = devm_kzalloc(dev, sizeof(*rptr), GFP_KERNEL);
	if (!rptr)
		return -ENOMEM;

	rptr->i2c = client;
	rptr->dev = dev;
	i2c_set_clientdata(client, rptr);

	rptr->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(rptr->pinctrl)) {
		dev_warn(dev, "no pinctrl, relying on bootloader GPIO state\n");
		rptr->pinctrl = NULL;
	}

	if (rptr->pinctrl) {
		rptr->enable = pinctrl_lookup_state(rptr->pinctrl, "enable");
		if (IS_ERR(rptr->enable)) {
			dev_warn(dev, "no 'enable' pinctrl state\n");
			rptr->enable = NULL;
		}

		rptr->disable = pinctrl_lookup_state(rptr->pinctrl, "disable");
		if (IS_ERR(rptr->disable)) {
			dev_warn(dev, "no 'disable' pinctrl state\n");
			rptr->disable = NULL;
		}
	}

	if (!eusb2_repeater_getcid(rptr))
		return -ENODEV;

	eusb2_rptr_prop_parse(rptr);

	rptr->phy = devm_phy_create(dev, np, &eusb2_repeater_ops);
	if (IS_ERR(rptr->phy)) {
		ret = PTR_ERR(rptr->phy);
		dev_err(dev, "failed to create PHY: %d\n", ret);
		return ret;
	}

	phy_set_drvdata(rptr->phy, rptr);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider))
		return PTR_ERR(phy_provider);

	dev_info(dev, "NXP eUSB2 repeater probed successfully\n");

	return 0;
}

static void eusb2_repeater_remove(struct i2c_client *client)
{
	struct eusb2_repeater *rptr = i2c_get_clientdata(client);

	if (rptr)
		eusb2_repeater_exit(rptr->phy);
}

static const struct of_device_id eusb2_repeater_of_match[] = {
	{ .compatible = "mtk,nxp-eusb2-repeater" },
	{ .compatible = "nxp,ptn36241" },
	{ }
};
MODULE_DEVICE_TABLE(of, eusb2_repeater_of_match);

static struct i2c_driver eusb2_repeater_driver = {
	.driver = {
		.name	= "nxp-eusb2-repeater",
		.of_match_table = eusb2_repeater_of_match,
	},
	.probe = eusb2_repeater_probe,
	.remove = eusb2_repeater_remove,
};
module_i2c_driver(eusb2_repeater_driver);

MODULE_DESCRIPTION("NXP eUSB2 Repeater (PTN36241) driver for MediaTek platforms");
MODULE_LICENSE("GPL");
