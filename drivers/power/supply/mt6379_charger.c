// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal MT6379 charger driver for the realme GT7 (MT6991) mainline port.
 *
 * The MT6379 is an SPMI PMIC (USID 0x0e) whose charger block the bootloader
 * already enables (BUCK_EN + CHG_EN, sane CV/MIVR), but it leaves IBUS_AICR at
 * the 500 mA USB-SDP default.  On this phone the screen-on system draw is
 * ~580 mA, so the battery gets ~0 until the input current limit is raised.
 *
 * The charger re-runs adapter detection on every plug and resets AICR, so a
 * one-shot write at boot is not enough: this driver re-asserts the limits on a
 * poll timer and exposes them (plus VBUS presence / charge state) through the
 * power_supply class.
 *
 * Register map and encodings come from the vendor mt6379-charger.c:
 *   0x70  CHG_STAT0   bit0 = PWR_RDY (VBUS present)
 *   0x121 CHG_STAT    bits[3:0] = charger state (0 sleep .. 7 done, 15 otg)
 *   0x109 IBUS_AICR   100000 + val*25000 uA
 *   0x10f ICHG        300000 + val*50000 uA
 *   0x114 CHG_WDT     bit5 BUCK_EN, bit4 CHG_EN, bit3 WDT_EN
 *   0x108 CHG_TOP2    bit0 = OTG 5 V boost (the enable bit of the vendor's
 *                     "mt6379-usb-otg-vbus" regulator); 0x135 HD_PP7 bit5
 *                     (PP_CV_FLOW_IDLE) must be released first, through the
 *                     0x07 test-mode passcode
 *   0x11c OTG_CV_MSB  9-bit big-endian boost voltage selector, stored at
 *                     0x11c/0x11d; volt = 4.85 V + (sel - 20) * 25 mV
 *   0x11e OTG_C       bits[2:0] boost current limit selector
 *                     (500 + idx * 300 mA, idx 0..6)
 *
 * The USB port runs as a host now (see the board DTS).  The MT6379's own
 * TCPC (vendor driver: typec/tcpc/tcpc_mt6379.c) is brought back up in DRP
 * mode and polled here, and the OTG 5 V boost follows whatever is plugged
 * in:
 *
 *   nothing      -> boost off; the PMIC sees external VBUS, so a charger
 *                   plugged next charges normally
 *   OTG device   -> boost on; the mtu3/XHCI host enumerates it
 *   charger / PC -> boost off; charging through the PMIC as usual
 *
 * The boost masks the PMIC's own VBUS detection, hence "off" by default;
 * the otg_boost sysfs file can still override it by hand for debugging.
 */

#include <linux/delay.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/spmi.h>
#include <linux/workqueue.h>

#define MT6379_REG_CHG_STAT0		0x70
#define MT6379_REG_CHG_STAT		0x121
#define MT6379_REG_CHG_TOP2		0x108
#define MT6379_REG_CHG_IBUS_AICR	0x109
#define MT6379_REG_CHG_ICHG		0x10f
#define MT6379_REG_CHG_OTG_CV_MSB	0x11c
#define MT6379_REG_CHG_OTG_C		0x11e
#define MT6379_REG_CHG_HD_PP7		0x135
#define MT6379_REG_TM_PAS_CODE1		0x07
#define MT6379_REG_CORECTRL2		0x006	/* PMU window: vendor 0x106 */

/*
 * MT6379 TCPC.  The vendor tcpc_mt6379.c addresses registers through a
 * tag translation ("PD" 0x0000 -> 0x400, "PMU" 0x0100 -> 0x000); the
 * values below are the resulting raw SPMI addresses.  Standard TCPCI
 * registers live at 0x400 + offset, the vendor CC/PD PHY block at 0x480+.
 */
#define MT6379_REG_TCPC_CTRL		0x0419
#define MT6379_REG_TCPC_ROLE_CTRL	0x041a
#define MT6379_REG_TCPC_POWER_CTRL	0x041c
#define MT6379_REG_TCPC_CC_STATUS	0x041d
#define MT6379_REG_TCPC_COMMAND		0x0423
#define MT6379_REG_TCPC_PHYCTRL1	0x0480
#define MT6379_REG_TCPC_PHYCTRL2	0x0481
#define MT6379_REG_TCPC_PHYCTRL3	0x0482
#define MT6379_REG_TCPC_PHYCTRL7	0x0486
#define MT6379_REG_TCPC_VCONCTRL3	0x048c
#define MT6379_REG_TCPC_SYSCTRL1	0x048f
#define MT6379_REG_TCPC_PHYCTRL9	0x04ac
#define MT6379_REG_TCPC_SYSCTRL3	0x04b0
#define MT6379_REG_TCPCCTRL1		0x04b1
#define MT6379_REG_TCPCCTRL2		0x04b2
#define MT6379_REG_TCPCCTRL3		0x04b3
#define MT6379_REG_TCPC_LPWRCTRL3	0x04bb
#define MT6379_REG_TCPC_LPWRCTRL5	0x04bd
#define MT6379_REG_TCPC_WATCHDOGCTRL	0x04be
#define MT6379_REG_TCPC_HILOCTRL9	0x04c8
#define MT6379_REG_TCPC_SHIELDCTRL1	0x04ca
#define MT6379_REG_TCPC_FODCTRL		0x04cf

#define MT6379_CHG_TOP2_OTG_EN		BIT(0)
#define MT6379_CHG_HD_PP7_FLOW_IDLE	BIT(5)

/*
 * ROLE_CONTROL: both CC = Rd; the stock state machine arms toggling with
 * Rp level 1 (its set_cc(TYPEC_CC_DRP) log "mt6379_set_cc 12"), i.e. 0x5a.
 */
#define MT6379_TCPC_ROLE_CTRL_RD_RD	(BIT(3) | BIT(1))
#define MT6379_TCPC_ROLE_CTRL_ARM	(BIT(6) | BIT(4) | MT6379_TCPC_ROLE_CTRL_RD_RD)
/*
 * CC_STATUS: bit 5 = DRP_TOGGLING (1 = unattached).  bit 4 (DRP_RESULT)
 * is NOT usable for attach detection: it stays set after the charger is
 * unplugged (0x10: sink latched, CC lines open).
 */
#define MT6379_TCPC_CC_STATUS_TOGGLING	BIT(5)
/* CC_STATUS CCx field: 0 = Ra, 1 = Rp, 2 = Rd, 3 = open */
#define MT6379_TCPC_CC_STATUS_CC1(reg)	((reg) & 0x3)
#define MT6379_TCPC_CC_STATUS_CC2(reg)	(((reg) & 0xc) >> 2)
#define MT6379_TCPC_CC_STATUS_RP	1
#define MT6379_TCPC_CC_STATUS_RD	2
#define MT6379_TCPC_CMD_LOOK_CONNECTION	0x99

/* vendor's usb-otg-vbus regulator defaults: 5.0 V, 2 A */
#define MT6379_OTG_CV_SEL_5V		26	/* 4.85 + (26 - 20) * 0.025 */
#define MT6379_OTG_CC_SEL_2000MA	5	/* 500 + 5 * 300 mA */

#define MT6379_CHG_STAT_DONE		7
#define MT6379_CHG_STAT_FAULT		8
#define MT6379_CHG_STAT_OTG		15

#define MT6379_AICR_MIN			100000
#define MT6379_AICR_MAX			4275000
#define MT6379_AICR_STEP		25000
#define MT6379_ICHG_MIN			300000
#define MT6379_ICHG_MAX			4300000
#define MT6379_ICHG_STEP		50000

/* 500 ms: fast enough to follow a cable swap / CC role change */
#define MT6379_POLL_MS			500

/* provided by the mtu3 gadget driver; absent in host-only builds */
#if IS_ENABLED(CONFIG_USB_MTU3_GADGET) || IS_ENABLED(CONFIG_USB_MTU3_DUAL_ROLE)
extern void mtu3_usb_reconnect(void);
#else
static inline void mtu3_usb_reconnect(void) { }
#endif

enum mt6379_cc_state {
	MT6379_CC_NONE = 0,
	MT6379_CC_SINK,
	MT6379_CC_SOURCE,
};

struct mt6379_chg {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct delayed_work poll;
	struct mutex lock;
	u32 aicr;	/* uA */
	u32 ichg;	/* uA */
	bool online;
	bool otg_on;
	enum mt6379_cc_state cc_state;
	u8 cc_regs[4];		/* last ROLE_CTRL..CC_STATUS snapshot */
	unsigned int cc_polls;
	unsigned int cc_rd_fails;
};

static int mt6379_set_aicr(struct mt6379_chg *c, u32 ua)
{
	u32 v;

	if (ua < MT6379_AICR_MIN || ua > MT6379_AICR_MAX)
		return -EINVAL;
	v = (ua - MT6379_AICR_MIN) / MT6379_AICR_STEP;

	return regmap_write(c->regmap, MT6379_REG_CHG_IBUS_AICR, v);
}

static int mt6379_set_ichg(struct mt6379_chg *c, u32 ua)
{
	u32 v;

	if (ua < MT6379_ICHG_MIN || ua > MT6379_ICHG_MAX)
		return -EINVAL;
	v = (ua - MT6379_ICHG_MIN) / MT6379_ICHG_STEP;

	return regmap_write(c->regmap, MT6379_REG_CHG_ICHG, v);
}

static int mt6379_apply(struct mt6379_chg *c)
{
	int ret;

	ret = mt6379_set_aicr(c, c->aicr);
	if (ret)
		return ret;

	return mt6379_set_ichg(c, c->ichg);
}

/*
 * Toggle the OTG 5 V boost with the vendor's usb-otg-vbus regulator
 * sequence: unlock test mode, release PP_CV_FLOW_IDLE, lock test mode
 * again, then flip the boost bit in CHG_TOP2.
 */
static int mt6379_otg_boost(struct mt6379_chg *c, bool on)
{
	static const u8 tm_passcode[] = { 0x69, 0x96, 0x63, 0x79 };
	int ret;

	mutex_lock(&c->lock);

	ret = regmap_bulk_write(c->regmap, MT6379_REG_TM_PAS_CODE1,
				tm_passcode, ARRAY_SIZE(tm_passcode));
	if (ret)
		goto out;

	ret = regmap_update_bits(c->regmap, MT6379_REG_CHG_HD_PP7,
				 MT6379_CHG_HD_PP7_FLOW_IDLE,
				 on ? 0 : MT6379_CHG_HD_PP7_FLOW_IDLE);
	regmap_write(c->regmap, MT6379_REG_TM_PAS_CODE1, 0);
	if (ret)
		goto out;

	ret = regmap_update_bits(c->regmap, MT6379_REG_CHG_TOP2,
				 MT6379_CHG_TOP2_OTG_EN,
				 on ? MT6379_CHG_TOP2_OTG_EN : 0);
	if (!ret)
		c->otg_on = on;
out:
	mutex_unlock(&c->lock);
	return ret;
}

/*
 * Arm DRP toggling (vendor mt6379_set_cc(TYPEC_CC_DRP)).  Re-issued from
 * the poll while the port stays unattached: the stock state machine does
 * the same on every plug event, a standing arm alone does not latch one.
 */
static int mt6379_tcpc_arm(struct mt6379_chg *c)
{
	int ret;

	ret = regmap_write(c->regmap, MT6379_REG_TCPC_ROLE_CTRL,
			   MT6379_TCPC_ROLE_CTRL_ARM);
	if (ret)
		return ret;

	/* vendor: give ROLE_CTRL >= 30us before LOOK_CONNECTION */
	udelay(32);
	return regmap_write(c->regmap, MT6379_REG_TCPC_COMMAND,
			    MT6379_TCPC_CMD_LOOK_CONNECTION);
}

/*
 * TCPC bring-up, copied from the vendor mt6379_tcpc_init(): sw_reset,
 * CC/PD PHY and low-power tuning, then arm DRP toggling.  Interrupt mask
 * setup is skipped: the CC state is polled here, the IRQ line is unused.
 */
static int mt6379_tcpc_init(struct mt6379_chg *c)
{
	static const u8 drp_duty[2] = { 0x34, 0x01 };	/* 308/1024, LE */
	int ret;

	ret = regmap_write(c->regmap, MT6379_REG_TCPC_SYSCTRL3, 0x01);
	if (ret)
		return ret;
	usleep_range(1000, 2000);

	/* stock DTS has no en-fod: keep the FOD sink path off (vendor probe) */
	ret = regmap_update_bits(c->regmap, MT6379_REG_TCPC_FODCTRL,
				 BIT(6), 0);
	if (ret)
		return ret;

	ret = regmap_write(c->regmap, MT6379_REG_TCPC_ROLE_CTRL,
			   MT6379_TCPC_ROLE_CTRL_RD_RD);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPCCTRL1, 0x14);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPCCTRL2, 0x00);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPCCTRL3, drp_duty[0]);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPCCTRL3 + 1, drp_duty[1]);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPC_PHYCTRL1, 0x74);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPC_PHYCTRL2, 0x3a);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPC_PHYCTRL3, 0x82);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPC_PHYCTRL7, 0x36);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPC_PHYCTRL9, 0x3c);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPC_VCONCTRL3, 0x51);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPC_LPWRCTRL5, 0x2f);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPC_HILOCTRL9, 0xaa);
	if (ret)
		return ret;
	ret = regmap_write(c->regmap, MT6379_REG_TCPC_LPWRCTRL3, 0xf8);
	if (ret)
		return ret;

	/* CC open 40ms on PMIC supply glitch */
	ret = regmap_update_bits(c->regmap, MT6379_REG_CORECTRL2,
				 BIT(4), BIT(4));
	if (ret)
		return ret;
	ret = regmap_update_bits(c->regmap, MT6379_REG_TCPC_SHIELDCTRL1,
				 BIT(4), BIT(4));
	if (ret)
		return ret;
	/* alert latch when looking4connection changes */
	ret = regmap_update_bits(c->regmap, MT6379_REG_TCPC_CTRL,
				 BIT(6), BIT(6));
	if (ret)
		return ret;
	ret = regmap_update_bits(c->regmap, MT6379_REG_TCPC_WATCHDOGCTRL,
				 BIT(6), BIT(6));
	if (ret)
		return ret;
	/* no bleed discharge */
	ret = regmap_update_bits(c->regmap, MT6379_REG_TCPC_POWER_CTRL,
				 BIT(3), 0);
	if (ret)
		return ret;
	ret = regmap_update_bits(c->regmap, MT6379_REG_TCPC_SYSCTRL1,
				 BIT(7), BIT(7));
	if (ret)
		return ret;
	/* auto idle, timeout 6.4ms */
	ret = regmap_update_bits(c->regmap, MT6379_REG_TCPC_SYSCTRL1,
				 GENMASK(2, 0) | BIT(3), BIT(3));
	if (ret)
		return ret;
	/* shipping mode off */
	ret = regmap_update_bits(c->regmap, MT6379_REG_TCPC_SYSCTRL1,
				 BIT(5), BIT(5));
	if (ret)
		return ret;
	mdelay(1);

	return mt6379_tcpc_arm(c);
}

/*
 * Read the standard TCPCI block one byte at a time.  Multi-byte SPMI
 * reads of the MT6379 PD window fail on this bus (regmap_bulk_read()
 * returns -EIO and yields an all-zero buffer), single-byte reads work.
 */
static int mt6379_tcpc_read_regs(struct mt6379_chg *c, u8 r[4])
{
	unsigned int v;
	int ret;

	ret = regmap_read(c->regmap, MT6379_REG_TCPC_ROLE_CTRL, &v);
	if (ret)
		return ret;
	r[0] = v;
	ret = regmap_read(c->regmap, MT6379_REG_TCPC_CTRL, &v);
	if (ret)
		return ret;
	r[1] = v;
	ret = regmap_read(c->regmap, MT6379_REG_TCPC_POWER_CTRL, &v);
	if (ret)
		return ret;
	r[2] = v;
	ret = regmap_read(c->regmap, MT6379_REG_TCPC_CC_STATUS, &v);
	if (ret)
		return ret;
	r[3] = v;
	return 0;
}

/*
 * Follow the TCPC's autonomous DRP result: toggling with nothing attached
 * keeps the boost off (charge-ready), a detected sink (charger/PC) keeps
 * it off and lets the PMIC charge, a detected source (OTG device) turns
 * the 5 V boost on so the mtu3 host can talk to it.
 */
static void mt6379_cc_poll(struct mt6379_chg *c)
{
	u8 r[4];	/* ROLE_CTRL, TCPC_CTRL, POWER_CTRL, CC_STATUS */
	int ret;

	ret = mt6379_tcpc_read_regs(c, r);
	if (ret) {
		if (++c->cc_rd_fails % 120 == 1)
			dev_warn(c->dev, "TCPC reg read failed: %d\n", ret);
		return;
	}
	c->cc_rd_fails = 0;

	if (memcmp(r, c->cc_regs, sizeof(r))) {
		memcpy(c->cc_regs, r, sizeof(r));
		dev_info(c->dev,
			 "TCPC regs: role=%02x ctl=%02x pwr=%02x ccst=%02x\n",
			 r[0], r[1], r[2], r[3]);
	}
	if (++c->cc_polls >= 60) {
		c->cc_polls = 0;
		dev_info(c->dev,
			 "TCPC hb: role=%02x ccst=%02x state=%d boost=%d\n",
			 r[0], r[3], c->cc_state, c->otg_on);
	}

	/*
	 * Attach detection only trusts real CC terminations: a sink device
	 * pulls Rd, a charger pulls Rp.  Everything else -- toggling, or a
	 * latched state with the CC lines open (0x00 after an OTG device
	 * was unplugged, 0x10 after a charger was) -- is unattached.
	 */
	if (!(r[3] & MT6379_TCPC_CC_STATUS_TOGGLING)) {
		if (MT6379_TCPC_CC_STATUS_CC1(r[3]) == MT6379_TCPC_CC_STATUS_RP ||
		    MT6379_TCPC_CC_STATUS_CC2(r[3]) == MT6379_TCPC_CC_STATUS_RP) {
			if (c->cc_state != MT6379_CC_SINK) {
				c->cc_state = MT6379_CC_SINK;
				dev_info(c->dev, "CC: charger attached (sink)\n");
			}
			if (c->otg_on)
				mt6379_otg_boost(c, false);
			return;
		}
		if (MT6379_TCPC_CC_STATUS_CC1(r[3]) == MT6379_TCPC_CC_STATUS_RD ||
		    MT6379_TCPC_CC_STATUS_CC2(r[3]) == MT6379_TCPC_CC_STATUS_RD) {
			if (c->cc_state != MT6379_CC_SOURCE) {
				c->cc_state = MT6379_CC_SOURCE;
				dev_info(c->dev, "CC: OTG device attached (source)\n");
			}
			if (!c->otg_on)
				mt6379_otg_boost(c, true);
			return;
		}
	}

	/*
	 * Unattached: the DRP engine does not resume toggling by itself
	 * after an unplug.  Keep the boost off so it cannot mask charger
	 * detection, and re-arm to restart toggling.
	 */
	if (c->cc_state != MT6379_CC_NONE) {
		c->cc_state = MT6379_CC_NONE;
		dev_info(c->dev, "CC: detached, OTG 5V boost off\n");
	}
	if (c->otg_on)
		mt6379_otg_boost(c, false);
	ret = mt6379_tcpc_arm(c);
	if (ret)
		dev_warn(c->dev, "TCPC re-arm failed: %d\n", ret);
}

static ssize_t otg_boost_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct mt6379_chg *c = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", c->otg_on);
}

static ssize_t otg_boost_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct mt6379_chg *c = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	ret = mt6379_otg_boost(c, on);
	if (ret)
		return ret;

	dev_info(c->dev, "OTG 5V boost %s\n", on ? "on" : "off");
	return count;
}
static DEVICE_ATTR_RW(otg_boost);

static ssize_t cc_state_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct mt6379_chg *c = dev_get_drvdata(dev);
	static const char * const names[] = { "none", "sink", "source" };
	u8 r[4];
	int ret;

	ret = mt6379_tcpc_read_regs(c, r);
	if (ret)
		return sysfs_emit(buf, "read failed (%d)\n", ret);

	return sysfs_emit(buf, "state=%s role_ctrl=0x%02x cc_status=0x%02x\n",
			  names[c->cc_state], r[0], r[3]);
}
static DEVICE_ATTR_RO(cc_state);

static void mt6379_poll_work(struct work_struct *work)
{
	struct mt6379_chg *c = container_of(work, struct mt6379_chg,
					    poll.work);
	unsigned int stat0 = 0;
	bool online;

	if (!regmap_read(c->regmap, MT6379_REG_CHG_STAT0, &stat0)) {
		online = !!(stat0 & BIT(0));
		if (online != c->online) {
			c->online = online;
			dev_info(c->dev, "VBUS %s\n",
				 online ? "present" : "absent");
			/*
			 * Cable (re)plug: the eUSB2 PHY chain cannot report
			 * VBUS to the mtu3 (it forces VBUS_FRC_EN and never sees
			 * a disconnect), so make the mtu3 redo its D+ connect --
			 * which also makes the xsphy re-handshake the MT6379
			 * repeater -- so the host re-enumerates.  Device-mode
			 * only; in the host-only build this is an empty stub.
			 */
			if (online)
				mtu3_usb_reconnect();
			power_supply_changed(c->psy);
		}

		/* The charger resets AICR on every plug/re-detection. */
		mt6379_apply(c);
	}

	mt6379_cc_poll(c);

	schedule_delayed_work(&c->poll, msecs_to_jiffies(MT6379_POLL_MS));
}

static enum power_supply_property mt6379_chg_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
};

static int mt6379_chg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct mt6379_chg *c = power_supply_get_drvdata(psy);
	unsigned int v;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = c->online;
		return 0;
	case POWER_SUPPLY_PROP_STATUS:
		if (!c->online) {
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			return 0;
		}
		if (regmap_read(c->regmap, MT6379_REG_CHG_STAT, &v))
			v = MT6379_CHG_STAT_DONE;
		switch (v & 0xf) {
		case MT6379_CHG_STAT_DONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		case MT6379_CHG_STAT_FAULT:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case MT6379_CHG_STAT_OTG:
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		}
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		val->intval = c->aicr;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		val->intval = c->ichg;
		return 0;
	default:
		return -EINVAL;
	}
}

static int mt6379_chg_set_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   const union power_supply_propval *val)
{
	struct mt6379_chg *c = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mt6379_set_aicr(c, val->intval);
		if (ret)
			return ret;
		c->aicr = val->intval;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mt6379_set_ichg(c, val->intval);
		if (ret)
			return ret;
		c->ichg = val->intval;
		return 0;
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc mt6379_chg_desc = {
	.name			= "mt6379-charger",
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= mt6379_chg_props,
	.num_properties		= ARRAY_SIZE(mt6379_chg_props),
	.get_property		= mt6379_chg_get_property,
	.set_property		= mt6379_chg_set_property,
};

static const struct regmap_config mt6379_regmap_config = {
	.reg_bits	= 16,
	.val_bits	= 8,
	.max_register	= 0x1ffff,
};

static int mt6379_chg_probe(struct spmi_device *sdev)
{
	struct power_supply_config cfg = {};
	struct mt6379_chg *c;
	int ret;

	c = devm_kzalloc(&sdev->dev, sizeof(*c), GFP_KERNEL);
	if (!c)
		return -ENOMEM;

	c->dev = &sdev->dev;
	c->regmap = devm_regmap_init_spmi_ext(sdev, &mt6379_regmap_config);
	if (IS_ERR(c->regmap))
		return dev_err_probe(c->dev, PTR_ERR(c->regmap),
				     "failed to init SPMI regmap\n");

	/* Conservative 5V default; the sysfs/power_supply can raise these. */
	c->aicr = 3000000;
	c->ichg = 3000000;
	mutex_init(&c->lock);

	cfg.drv_data = c;
	cfg.fwnode = dev_fwnode(c->dev);
	c->psy = devm_power_supply_register(c->dev, &mt6379_chg_desc, &cfg);
	if (IS_ERR(c->psy))
		return dev_err_probe(c->dev, PTR_ERR(c->psy),
				     "failed to register power supply\n");

	spmi_device_set_drvdata(sdev, c);

	ret = device_create_file(&sdev->dev, &dev_attr_otg_boost);
	if (ret)
		return dev_err_probe(c->dev, ret,
				     "failed to create otg_boost\n");

	ret = device_create_file(&sdev->dev, &dev_attr_cc_state);
	if (ret)
		return dev_err_probe(c->dev, ret,
				     "failed to create cc_state\n");

	mt6379_apply(c);

	/*
	 * Android's charger HAL programs the boost's target voltage (5.0 V)
	 * and current limit before flipping it on, and the PMIC reset values
	 * are not documented, so do the same explicitly here.  The selector
	 * encodings come from the vendor's regulator_desc.
	 */
	{
		const u8 otg_cv[] = { 0x00, MT6379_OTG_CV_SEL_5V };

		if (regmap_bulk_write(c->regmap, MT6379_REG_CHG_OTG_CV_MSB,
				      otg_cv, sizeof(otg_cv)) ||
		    regmap_update_bits(c->regmap, MT6379_REG_CHG_OTG_C,
				       GENMASK(2, 0), MT6379_OTG_CC_SEL_2000MA))
			dev_warn(c->dev, "failed to program OTG boost defaults\n");
	}

	/* Cache VBUS state so the first poll does not log a spurious edge. */
	{
		unsigned int stat0 = 0;

		if (!regmap_read(c->regmap, MT6379_REG_CHG_STAT0, &stat0))
			c->online = !!(stat0 & BIT(0));
	}

	/*
	 * Bring the MT6379 TCPC up and arm DRP toggling: the port then
	 * decides sink (charger) vs source (OTG device) on its own and the
	 * poll flips the 5 V boost to match.
	 */
	ret = mt6379_tcpc_init(c);
	if (ret) {
		dev_warn(c->dev,
			 "TCPC init failed (%d), use otg_boost sysfs manually\n",
			 ret);
	} else {
		u8 rb[4] = { };
		unsigned int sy3 = 0, tc1 = 0, sy1 = 0, fod = 0;
		unsigned int t3l = 0, t3h = 0;

		mt6379_tcpc_read_regs(c, rb);
		regmap_read(c->regmap, MT6379_REG_TCPC_SYSCTRL3, &sy3);
		regmap_read(c->regmap, MT6379_REG_TCPCCTRL1, &tc1);
		regmap_read(c->regmap, MT6379_REG_TCPC_SYSCTRL1, &sy1);
		regmap_read(c->regmap, MT6379_REG_TCPC_FODCTRL, &fod);
		regmap_read(c->regmap, MT6379_REG_TCPCCTRL3, &t3l);
		regmap_read(c->regmap, MT6379_REG_TCPCCTRL3 + 1, &t3h);
		dev_info(c->dev,
			 "TCPC ready, DRP armed: role=%02x ccst=%02x sy3=%02x tc1=%02x sy1=%02x fod=%02x tc3=%02x%02x\n",
			 rb[0], rb[3], sy3, tc1, sy1, fod, t3h, t3l);
	}

	INIT_DELAYED_WORK(&c->poll, mt6379_poll_work);

	/*
	 * Populate child devices that share the MT6379 regmap -- in particular
	 * the eUSB2 repeater (mtk,mt6379-eusb2-repeater), which is a phy
	 * provider for the SoC xsphy and whose regmap is dev_get_regmap()'d
	 * from this SPMI device.  Do this before arming the poll work so a
	 * failure here cannot leave the work running against freed memory.
	 */
	ret = of_platform_populate(sdev->dev.of_node, NULL, NULL, &sdev->dev);
	if (ret)
		return dev_err_probe(c->dev, ret,
				     "failed to populate MT6379 children\n");

	schedule_delayed_work(&c->poll, msecs_to_jiffies(MT6379_POLL_MS));

	dev_info(c->dev, "MT6379 charger registered (AICR %u uA, ICHG %u uA)\n",
		 c->aicr, c->ichg);

	return 0;
}

static void mt6379_chg_remove(struct spmi_device *sdev)
{
	struct mt6379_chg *c = spmi_device_get_drvdata(sdev);

	of_platform_depopulate(&sdev->dev);
	cancel_delayed_work_sync(&c->poll);
	device_remove_file(&sdev->dev, &dev_attr_cc_state);
	device_remove_file(&sdev->dev, &dev_attr_otg_boost);
}

static const struct of_device_id mt6379_chg_of_match[] = {
	{ .compatible = "mediatek,mt6379" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6379_chg_of_match);

static struct spmi_driver mt6379_chg_driver = {
	.probe = mt6379_chg_probe,
	.remove = mt6379_chg_remove,
	.driver = {
		.name = "mt6379-charger",
		.of_match_table = mt6379_chg_of_match,
	},
};
module_spmi_driver(mt6379_chg_driver);

MODULE_AUTHOR("realme GT7 mainline port");
MODULE_DESCRIPTION("MT6379 charger + TCPC + OTG boost driver (MT6991)");
MODULE_LICENSE("GPL");
