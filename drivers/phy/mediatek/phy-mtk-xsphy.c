// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek USB3.1 gen2 xsphy Driver
 *
 * Copyright (c) 2018 MediaTek Inc.
 * Author: Chunfeng Yun <chunfeng.yun@mediatek.com>
 *
 * Modified for MT6991: added full vendor u2_phy_instance_power_on()
 * sequence and u2_phy_instance_set_mode() DPPULLUP support.
 */

#include <dt-bindings/phy/phy.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "phy-mtk-io.h"

/* u2 phy banks */
#define SSUSB_SIFSLV_MISC		0x000
#define SSUSB_SIFSLV_U2FREQ		0x100
#define SSUSB_SIFSLV_U2PHY_COM	0x300

/* u3 phy shared banks */
#define SSPXTP_SIFSLV_DIG_GLB		0x000
#define SSPXTP_SIFSLV_PHYA_GLB		0x100

/* u3 phy banks */
#define SSPXTP_SIFSLV_DIG_LN_TOP	0x000
#define SSPXTP_SIFSLV_DIG_LN_TX0	0x100
#define SSPXTP_SIFSLV_DIG_LN_RX0	0x200
#define SSPXTP_SIFSLV_DIG_LN_DAIF	0x300
#define SSPXTP_SIFSLV_PHYA_LN		0x400

#define XSP_U2FREQ_FMCR0	((SSUSB_SIFSLV_U2FREQ) + 0x00)
#define P2F_RG_FREQDET_EN	BIT(24)
#define P2F_RG_CYCLECNT		GENMASK(23, 0)

#define XSP_U2FREQ_MMONR0  ((SSUSB_SIFSLV_U2FREQ) + 0x0c)

#define XSP_U2FREQ_FMMONR1	((SSUSB_SIFSLV_U2FREQ) + 0x10)
#define P2F_RG_FRCK_EN		BIT(8)
#define P2F_USB_FM_VALID	BIT(0)

#define XSP_USBPHYACR0	((SSUSB_SIFSLV_U2PHY_COM) + 0x00)
#define P2A0_RG_INTR_EN	BIT(5)
#define P2A0_RG_USB20_TX_PH_ROT_SEL	BIT(20)
#define P2A0_RG_USB20_CHP_EN	BIT(1)

#define XSP_USBPHYACR1		((SSUSB_SIFSLV_U2PHY_COM) + 0x04)
#define P2A1_RG_INTR_CAL		GENMASK(23, 19)
#define P2A1_RG_VRT_SEL			GENMASK(14, 12)
#define P2A1_RG_TERM_SEL		GENMASK(10, 8)

#define XSP_USBPHYACR4		((SSUSB_SIFSLV_U2PHY_COM) + 0x10)
#define P2A4_RG_USB20_FS_CR		GENMASK(10, 8)
#define P2A4_RG_USB20_GPIO_CTL		BIT(9)
#define P2A4_USB20_GPIO_MODE		BIT(8)
#define P2A4_U2_GPIO_CTR_MSK (P2A4_RG_USB20_GPIO_CTL | P2A4_USB20_GPIO_MODE)

#define XSP_USBPHYACR5		((SSUSB_SIFSLV_U2PHY_COM) + 0x014)
#define P2A5_RG_HSTX_SRCAL_EN	BIT(15)
#define P2A5_RG_HSTX_SRCTRL		GENMASK(14, 12)
#define P2A6_RG_USB20_SQD		GENMASK(23, 22)

#define XSP_USBPHYACR6		((SSUSB_SIFSLV_U2PHY_COM) + 0x018)
#define P2A6_RG_U2_PHY_REV6		GENMASK(31, 30)
#define P2A6_RG_U2_PHY_REV6_VAL(x)	((0x3 & (x)) << 30)
#define P2A6_RG_U2_PHY_REV1		BIT(25)
#define P2A6_RG_BC11_SW_EN	BIT(23)
#define P2A6_RG_OTG_VBUSCMP_EN	BIT(20)
#define P2A6_RG_U2_DISCTH		GENMASK(7, 4)
#define P2A6_RG_U2_SQTH			GENMASK(3, 0)

#define XSP_USBPHYACR3		((SSUSB_SIFSLV_U2PHY_COM) + 0x01c)
#define P2A3_RG_USB20_PUPD_BIST_EN	BIT(12)
#define P2A3_RG_USB20_EN_PU_DP		BIT(9)

#define XSP_U2PHYACR4		((SSUSB_SIFSLV_U2PHY_COM) + 0x020)

#define XSP_USBPHYA_RESERVE	((SSUSB_SIFSLV_U2PHY_COM) + 0x030)
#define P2AR_RG_INTR_CAL		GENMASK(29, 24)
#define P2AR_RG_INTR_CAL_MASK		(0x3f)
#define P2AR_RG_INTR_CAL_OFET		(24)

#define XSP_USBPHYA_RESERVEA	((SSUSB_SIFSLV_U2PHY_COM) + 0x034)
#define P2ARA_RG_TERM_CAL		GENMASK(11, 8)
#define P2ARA_RG_TERM_CAL_MASK          (0xf)
#define P2ARA_RG_TERM_CAL_OFET		(8)

#define XSP_U2PHYA_RESERVE0	((SSUSB_SIFSLV_U2PHY_COM) + 0x040)
#define P2A2R0_RG_PLL_FBKSEL         BIT(31)
#define P2A2R0_RG_HSRX_VREF_SEL		GENMASK(6, 4)

#define XSP_U2PHYA_RESERVE1	((SSUSB_SIFSLV_U2PHY_COM) + 0x044)
#define P2A2R1_RG_PLL_POSDIV    GENMASK(2, 0)
#define P2A2R1_RG_PLL_REFCLK_SEL        BIT(5)

#define XSP_U2PHYDCR1		((SSUSB_SIFSLV_U2PHY_COM) + 0x064)
#define P2C_RG_USB20_SW_PLLMODE	GENMASK(19, 18)

#define XSP_U2PHYDTM0		((SSUSB_SIFSLV_U2PHY_COM) + 0x068)
#define P2D_FORCE_UART_EN		BIT(26)
#define P2D_FORCE_DATAIN		BIT(23)
#define P2D_FORCE_SUSPENDM		BIT(18)
#define P2D_RG_SUSPENDM			BIT(3)
#define P2D_RG_XCVRSEL			GENMASK(5, 4)
#define P2D_RG_DATAIN			GENMASK(13, 10)
#define P2D_DTM0_PART_MASK \
		(P2D_FORCE_DATAIN | P2D_FORCE_SUSPENDM | \
		P2D_RG_XCVRSEL | P2D_RG_DATAIN)

#define XSP_U2PHYDTM1		((SSUSB_SIFSLV_U2PHY_COM) + 0x06C)
#define P2D_FORCE_IDDIG		BIT(9)
#define P2D_RG_VBUSVALID	BIT(5)
#define P2D_RG_SESSEND		BIT(4)
#define P2D_RG_AVALID		BIT(2)
#define P2D_RG_IDDIG		BIT(1)
#define P2D_RG_UART_EN		BIT(6)

#define SSPXTP_PHYA_GLB_00		((SSPXTP_SIFSLV_PHYA_GLB) + 0x00)
#define RG_XTP_GLB_BIAS_INTR_CTRL		GENMASK(21, 16)

#define SSPXTP_PHYA_LN_04	((SSPXTP_SIFSLV_PHYA_LN) + 0x04)
#define RG_XTP_LN0_TX_IMPSEL		GENMASK(4, 0)

#define SSPXTP_PHYA_LN_14	((SSPXTP_SIFSLV_PHYA_LN) + 0x014)
#define RG_XTP_LN0_RX_IMPSEL		GENMASK(4, 0)

#define XSP_REF_CLK		26	/* MHZ */
#define XSP_SLEW_RATE_COEF	17
#define XSP_SR_COEF_DIVISOR	1000
#define XSP_FM_DET_CYCLE_CNT	1024

/* PHY switch between pcie/usb3/sgmii */
#define USB_PHY_SWITCH_CTRL	0x0
#define RG_PHY_SW_TYPE		GENMASK(3, 0)
#define RG_PHY_SW_PCIE		0x0
#define RG_PHY_SW_USB3		0x1
#define RG_PHY_SW_SGMII		0x2

struct xsphy_instance {
	struct phy *phy;
	void __iomem *port_base;
	struct clk *ref_clk;	/* reference clock of anolog phy */
	u32 index;
	u32 type;
	struct regmap *type_sw;
	u32 type_sw_reg;
	u32 type_sw_index;
	/* only for HQA test */
	bool property_ready;
	int efuse_intr;
	int efuse_term_cal;
	int efuse_tx_imp;
	int efuse_rx_imp;
	int intr_ofs;
	int term_ofs;
	int host_intr_ofs;
	int host_term_ofs;
	int pll_fbksel;
	int pll_posdiv;
	/* u2 eye diagram */
	int eye_src;
	int eye_vrt;
	int eye_term;
	int discth;
	int rx_sqth;
	int host_rx_sqth;
	int rx_sqd;
	int host_rx_sqd;
	int rev6;
	int hsrx_vref_sel;
	int fs_cr;
	/* u2 eye diagram for host */
	int eye_src_host;
	int eye_vrt_host;
	int eye_term_host;
	int rev6_host;
	/* refclk source */
	bool refclk_sel;
	/* HWPLL mode setting */
	bool hwpll_mode;
	bool chp_en_disable;
};

struct mtk_xsphy {
	struct device *dev;
	void __iomem *glb_base;	/* only shared u3 sif */
	int nphys;
	int src_ref_clk; /* MHZ, reference clock for slew rate calibrate */
	int src_coef;    /* coefficient for slew rate calibrate */
	bool tx_chirpK_disable;
	bool bc11_switch_disable;
	int sw_ver;
	struct xsphy_instance *phys[] __counted_by(nphys);
};

static void u2_phy_slew_rate_calibrate(struct mtk_xsphy *xsphy,
					struct xsphy_instance *inst)
{
	void __iomem *pbase = inst->port_base;
	int calib_val;
	int fm_out;
	u32 tmp;

	if (inst->eye_src)
		return;

	mtk_phy_set_bits(pbase + XSP_USBPHYACR5, P2A5_RG_HSTX_SRCAL_EN);
	udelay(1);

	mtk_phy_set_bits(pbase + XSP_U2FREQ_FMMONR1, P2F_RG_FRCK_EN);

	mtk_phy_update_field(pbase + XSP_U2FREQ_FMCR0, P2F_RG_CYCLECNT,
			     XSP_FM_DET_CYCLE_CNT);

	mtk_phy_set_bits(pbase + XSP_U2FREQ_FMCR0, P2F_RG_FREQDET_EN);

	readl_poll_timeout(pbase + XSP_U2FREQ_FMMONR1, tmp,
			   (tmp & P2F_USB_FM_VALID), 10, 200);

	fm_out = readl(pbase + XSP_U2FREQ_MMONR0);

	mtk_phy_clear_bits(pbase + XSP_U2FREQ_FMCR0, P2F_RG_FREQDET_EN);

	mtk_phy_clear_bits(pbase + XSP_U2FREQ_FMMONR1, P2F_RG_FRCK_EN);

	if (fm_out) {
		tmp = xsphy->src_ref_clk * xsphy->src_coef;
		tmp = (tmp * XSP_FM_DET_CYCLE_CNT) / fm_out;
		calib_val = DIV_ROUND_CLOSEST(tmp, XSP_SR_COEF_DIVISOR);
	} else {
		calib_val = 3;
	}

	mtk_phy_update_field(pbase + XSP_USBPHYACR5, P2A5_RG_HSTX_SRCTRL, calib_val);

	mtk_phy_clear_bits(pbase + XSP_USBPHYACR5, P2A5_RG_HSTX_SRCAL_EN);
}

static void u2_phy_instance_init(struct mtk_xsphy *xsphy,
				 struct xsphy_instance *inst)
{
	void __iomem *pbase = inst->port_base;
	u32 tmp;

	if (inst->efuse_intr == -EINVAL) {
		tmp = readl(pbase + XSP_USBPHYA_RESERVE);
		tmp >>= P2AR_RG_INTR_CAL_OFET;
		inst->efuse_intr = tmp & P2AR_RG_INTR_CAL_MASK;
	}

	if (inst->efuse_term_cal == -EINVAL) {
		tmp = readl(pbase + XSP_USBPHYA_RESERVEA);
		tmp >>= P2ARA_RG_TERM_CAL_OFET;
		inst->efuse_term_cal = tmp & P2ARA_RG_TERM_CAL_MASK;
	}

	mtk_phy_clear_bits(pbase + XSP_USBPHYACR6, P2A6_RG_BC11_SW_EN);

	mtk_phy_set_bits(pbase + XSP_USBPHYACR0, P2A0_RG_INTR_EN);
}

static void u2_phy_instance_power_on(struct mtk_xsphy *xsphy,
				     struct xsphy_instance *inst)
{
	void __iomem *pbase = inst->port_base;
	u32 index = inst->index;

	/* PLL refclk select */
	if (inst->refclk_sel) {
		mtk_phy_set_bits(pbase + XSP_U2PHYA_RESERVE1,
					P2A2R1_RG_PLL_REFCLK_SEL);
		udelay(250);
	}

	/* suspend release dance */
	mtk_phy_set_bits(pbase + XSP_U2PHYDTM0, P2D_FORCE_SUSPENDM);
	mtk_phy_clear_bits(pbase + XSP_U2PHYDTM0, P2D_RG_SUSPENDM);
	mtk_phy_set_bits(pbase + XSP_U2PHYDTM0, P2D_RG_SUSPENDM);
	udelay(30);
	mtk_phy_clear_bits(pbase + XSP_U2PHYDTM0, P2D_FORCE_SUSPENDM);
	mtk_phy_clear_bits(pbase + XSP_U2PHYDTM0, P2D_RG_SUSPENDM);

	/* clear test modes */
	mtk_phy_clear_bits(pbase + XSP_U2PHYDTM0, P2D_FORCE_UART_EN);
	mtk_phy_clear_bits(pbase + XSP_U2PHYDTM1, P2D_RG_UART_EN);
	mtk_phy_clear_bits(pbase + XSP_U2PHYACR4, P2A4_U2_GPIO_CTR_MSK);

	/* clear force bits */
	mtk_phy_clear_bits(pbase + XSP_U2PHYDTM0, P2D_FORCE_SUSPENDM);
	mtk_phy_clear_bits(pbase + XSP_U2PHYDTM0,
			   (P2D_RG_XCVRSEL | P2D_RG_DATAIN | P2D_DTM0_PART_MASK));

	/* HWPLL mode */
	if (inst->hwpll_mode)
		mtk_phy_clear_bits(pbase + XSP_U2PHYDCR1, P2C_RG_USB20_SW_PLLMODE);

	/* BC11 disable */
	mtk_phy_clear_bits(pbase + XSP_USBPHYACR6, P2A6_RG_BC11_SW_EN);

	/* VBUS detect */
	mtk_phy_set_bits(pbase + XSP_USBPHYACR6, P2A6_RG_OTG_VBUSCMP_EN);

	mtk_phy_update_bits(pbase + XSP_U2PHYDTM1,
			    P2D_RG_VBUSVALID | P2D_RG_AVALID | P2D_RG_SESSEND,
			    P2D_RG_VBUSVALID | P2D_RG_AVALID);

	/* clear TX phase rotation */
	mtk_phy_clear_bits(pbase + XSP_USBPHYACR0, P2A0_RG_USB20_TX_PH_ROT_SEL);

	/* REV6=1, REV1=0 then settle */
	mtk_phy_clear_bits(pbase + XSP_USBPHYACR6,
			   (P2A6_RG_U2_PHY_REV6 | P2A6_RG_U2_PHY_REV1));
	mtk_phy_set_bits(pbase + XSP_USBPHYACR6, P2A6_RG_U2_PHY_REV6_VAL(1));

	udelay(800);

	/* CHP_EN disable */
	if (inst->chp_en_disable)
		mtk_phy_clear_bits(pbase + XSP_USBPHYACR0, P2A0_RG_USB20_CHP_EN);

	dev_dbg(xsphy->dev, "%s(%d)\n", __func__, index);
}

static void u2_phy_instance_power_off(struct mtk_xsphy *xsphy,
				      struct xsphy_instance *inst)
{
	void __iomem *pbase = inst->port_base;
	u32 index = inst->index;

	mtk_phy_clear_bits(pbase + XSP_U2PHYDTM0, P2D_FORCE_UART_EN);
	mtk_phy_clear_bits(pbase + XSP_U2PHYDTM1, P2D_RG_UART_EN);
	mtk_phy_clear_bits(pbase + XSP_U2PHYACR4, P2A4_U2_GPIO_CTR_MSK);
	mtk_phy_clear_bits(pbase + XSP_USBPHYACR6, P2A6_RG_BC11_SW_EN);
	mtk_phy_clear_bits(pbase + XSP_USBPHYACR6, P2A6_RG_OTG_VBUSCMP_EN);

	mtk_phy_update_bits(pbase + XSP_U2PHYDTM1,
			    P2D_RG_VBUSVALID | P2D_RG_AVALID | P2D_RG_SESSEND,
			    P2D_RG_SESSEND);

	mtk_phy_set_bits(pbase + XSP_U2PHYDTM0, (P2D_RG_SUSPENDM | P2D_FORCE_SUSPENDM));
	mdelay(2);
	mtk_phy_clear_bits(pbase + XSP_U2PHYDTM0, P2D_FORCE_SUSPENDM);
	mtk_phy_clear_bits(pbase + XSP_U2PHYDTM0, P2D_RG_SUSPENDM);

	dev_dbg(xsphy->dev, "%s(%d)\n", __func__, index);
}

static void u2_phy_props_set(struct mtk_xsphy *xsphy,
			     struct xsphy_instance *inst)
{
	void __iomem *pbase = inst->port_base;

	if (inst->efuse_intr != -EINVAL) {
		int intr_val = inst->efuse_intr + inst->intr_ofs;

		if (inst->intr_ofs < -P2AR_RG_INTR_CAL_MASK ||
			inst->intr_ofs > P2AR_RG_INTR_CAL_MASK ||
			intr_val < 0 || intr_val > P2AR_RG_INTR_CAL_MASK)
			intr_val = inst->efuse_intr;

		mtk_phy_update_field(pbase + XSP_USBPHYA_RESERVE, P2AR_RG_INTR_CAL, intr_val);
	}

	if (inst->efuse_term_cal != -EINVAL) {
		int term_val = inst->efuse_term_cal + inst->term_ofs;

		if (inst->term_ofs < -P2ARA_RG_TERM_CAL_MASK ||
			inst->term_ofs > P2ARA_RG_TERM_CAL_MASK ||
			term_val < 0 || term_val > P2ARA_RG_TERM_CAL_MASK)
			term_val = inst->efuse_term_cal;

		mtk_phy_update_field(pbase + XSP_USBPHYA_RESERVEA, P2ARA_RG_TERM_CAL, term_val);
	}

	if (inst->eye_src != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR5, P2A5_RG_HSTX_SRCTRL,
				     inst->eye_src);

	if (inst->eye_vrt != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR1, P2A1_RG_VRT_SEL,
				     inst->eye_vrt);

	if (inst->eye_term != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR1, P2A1_RG_TERM_SEL,
				     inst->eye_term);

	if (inst->discth != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR6, P2A6_RG_U2_DISCTH,
				    inst->discth);

	if (inst->rx_sqth != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR6, P2A6_RG_U2_SQTH,
				    inst->rx_sqth);

	if (inst->rx_sqd != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR5, P2A6_RG_USB20_SQD,
				    inst->rx_sqd);

	if (inst->rev6 != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR6, P2A6_RG_U2_PHY_REV6,
				     inst->rev6);

	if (inst->pll_fbksel != -EINVAL)
		mtk_phy_update_field(pbase + XSP_U2PHYA_RESERVE0, P2A2R0_RG_PLL_FBKSEL,
				     inst->pll_fbksel);

	if (inst->pll_posdiv != -EINVAL)
		mtk_phy_update_field(pbase + XSP_U2PHYA_RESERVE1, P2A2R1_RG_PLL_POSDIV,
				     inst->pll_posdiv);

	if (inst->hsrx_vref_sel != -EINVAL) {
		if (!xsphy->sw_ver)
			mtk_phy_update_field(pbase + XSP_U2PHYA_RESERVE0, P2A2R0_RG_HSRX_VREF_SEL,
				     inst->hsrx_vref_sel);
	}

	if (inst->fs_cr != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR4, P2A4_RG_USB20_FS_CR,
				     inst->fs_cr);
}

static void u2_phy_host_props_set(struct mtk_xsphy *xsphy,
			     struct xsphy_instance *inst)
{
	void __iomem *pbase = inst->port_base;

	if (inst->efuse_intr != -EINVAL) {
		int host_intr_val = inst->efuse_intr + inst->host_intr_ofs;

		if (inst->host_intr_ofs < -P2AR_RG_INTR_CAL_MASK ||
			inst->host_intr_ofs > P2AR_RG_INTR_CAL_MASK ||
			host_intr_val < 0 || host_intr_val > P2AR_RG_INTR_CAL_MASK)
			host_intr_val = inst->efuse_intr;

		mtk_phy_update_field(pbase + XSP_USBPHYA_RESERVE, P2AR_RG_INTR_CAL, host_intr_val);
	}

	if (inst->efuse_term_cal != -EINVAL) {
		int host_term_val = inst->efuse_term_cal + inst->host_term_ofs;

		if (inst->host_term_ofs < -P2ARA_RG_TERM_CAL_MASK ||
			inst->host_term_ofs > P2ARA_RG_TERM_CAL_MASK ||
			host_term_val < 0 || host_term_val > P2ARA_RG_TERM_CAL_MASK)
			host_term_val = inst->efuse_term_cal;

		mtk_phy_update_field(pbase + XSP_USBPHYA_RESERVEA, P2ARA_RG_TERM_CAL, host_term_val);
	}

	if (inst->eye_src_host != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR5, P2A5_RG_HSTX_SRCTRL,
				     inst->eye_src_host);

	if (inst->eye_vrt_host != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR1, P2A1_RG_VRT_SEL,
				     inst->eye_vrt_host);

	if (inst->eye_term_host != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR1, P2A1_RG_TERM_SEL,
				     inst->eye_term_host);

	if (inst->rev6_host != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR6, P2A6_RG_U2_PHY_REV6,
				     inst->rev6_host);

	if (inst->host_rx_sqth != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR6, P2A6_RG_U2_SQTH,
				    inst->host_rx_sqth);

	if (inst->host_rx_sqd != -EINVAL)
		mtk_phy_update_field(pbase + XSP_USBPHYACR5, P2A6_RG_USB20_SQD,
				    inst->host_rx_sqd);
}

static void u3_phy_props_set(struct mtk_xsphy *xsphy,
			     struct xsphy_instance *inst)
{
	void __iomem *pbase = inst->port_base;

	if (inst->efuse_intr)
		mtk_phy_update_field(xsphy->glb_base + SSPXTP_PHYA_GLB_00,
				     RG_XTP_GLB_BIAS_INTR_CTRL, inst->efuse_intr);

	if (inst->efuse_tx_imp)
		mtk_phy_update_field(pbase + SSPXTP_PHYA_LN_04,
				     RG_XTP_LN0_TX_IMPSEL, inst->efuse_tx_imp);

	if (inst->efuse_rx_imp)
		mtk_phy_update_field(pbase + SSPXTP_PHYA_LN_14,
				     RG_XTP_LN0_RX_IMPSEL, inst->efuse_rx_imp);
}

/* type switch for usb3/pcie/sgmii */
static int phy_type_syscon_get(struct xsphy_instance *instance,
			       struct device_node *dn)
{
	struct of_phandle_args args;
	int ret;

	if (!of_property_present(dn, "mediatek,syscon-type"))
		return 0;

	ret = of_parse_phandle_with_fixed_args(dn, "mediatek,syscon-type",
					       2, 0, &args);
	if (ret)
		return ret;

	instance->type_sw_reg = args.args[0];
	instance->type_sw_index = args.args[1] & 0x3;
	instance->type_sw = syscon_node_to_regmap(args.np);
	of_node_put(args.np);

	return PTR_ERR_OR_ZERO(instance->type_sw);
}

static int phy_type_set(struct xsphy_instance *instance)
{
	int type;
	u32 offset;

	if (!instance->type_sw)
		return 0;

	switch (instance->type) {
	case PHY_TYPE_USB3:
		type = RG_PHY_SW_USB3;
		break;
	case PHY_TYPE_PCIE:
		type = RG_PHY_SW_PCIE;
		break;
	case PHY_TYPE_SGMII:
		type = RG_PHY_SW_SGMII;
		break;
	case PHY_TYPE_USB2:
	default:
		return 0;
	}

	offset = instance->type_sw_index * BITS_PER_BYTE;
	regmap_update_bits(instance->type_sw, instance->type_sw_reg,
			   RG_PHY_SW_TYPE << offset, type << offset);

	return 0;
}

static void phy_parse_property(struct mtk_xsphy *xsphy,
				struct xsphy_instance *inst)
{
	struct device *dev = &inst->phy->dev;
	const char *ofs_str;

	switch (inst->type) {
	case PHY_TYPE_USB2:
		if (device_property_read_u32(dev, "mediatek,efuse-intr",
					 &inst->efuse_intr) || inst->efuse_intr < 0)
			inst->efuse_intr = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,efuse-term",
					 &inst->efuse_term_cal) || inst->efuse_term_cal < 0)
			inst->efuse_term_cal = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,eye-src",
					 &inst->eye_src) || inst->eye_src < 0)
			inst->eye_src = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,eye-vrt",
					 &inst->eye_vrt) || inst->eye_vrt < 0)
			inst->eye_vrt = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,eye-term",
					 &inst->eye_term) || inst->eye_term < 0)
			inst->eye_term = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,discth",
					 &inst->discth) || inst->discth < 0)
			inst->discth = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,rx-sqth",
					 &inst->rx_sqth) || inst->rx_sqth < 0)
			inst->rx_sqth = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,host-rx-sqth",
					 &inst->host_rx_sqth) || inst->host_rx_sqth < 0)
			inst->host_rx_sqth = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,rx-sqd",
					 &inst->rx_sqd) || inst->rx_sqd < 0)
			inst->rx_sqd = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,host-rx-sqd",
					 &inst->host_rx_sqd) || inst->host_rx_sqd < 0)
			inst->host_rx_sqd = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,rev6",
					 &inst->rev6) || inst->rev6 < 0)
			inst->rev6 = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,hsrx-vref-sel",
					&inst->hsrx_vref_sel) || inst->hsrx_vref_sel < 0)
			inst->hsrx_vref_sel = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,fs-cr",
					&inst->fs_cr) || inst->fs_cr < 0)
			inst->fs_cr = -EINVAL;
		if (device_property_read_string(dev, "mediatek,intr-ofs",
					 &ofs_str) || kstrtoint(ofs_str, 10, &inst->intr_ofs) < 0)
			inst->intr_ofs = -(P2AR_RG_INTR_CAL_MASK + 1);
		if (device_property_read_string(dev, "mediatek,host-intr-ofs",
					 &ofs_str) || kstrtoint(ofs_str, 10, &inst->host_intr_ofs) < 0)
			inst->host_intr_ofs = -(P2AR_RG_INTR_CAL_MASK + 1);
		if (device_property_read_string(dev, "mediatek,term-ofs",
					 &ofs_str) || kstrtoint(ofs_str, 10, &inst->term_ofs) < 0)
			inst->term_ofs = -(P2ARA_RG_TERM_CAL_MASK + 1);
		if (device_property_read_string(dev, "mediatek,host-term-ofs",
					 &ofs_str) || kstrtoint(ofs_str, 10, &inst->host_term_ofs) < 0)
			inst->host_term_ofs = -(P2ARA_RG_TERM_CAL_MASK + 1);
		if (device_property_read_u32(dev, "mediatek,pll-fbksel",
				 &inst->pll_fbksel) || inst->pll_fbksel < 0)
			inst->pll_fbksel = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,pll-posdiv",
				 &inst->pll_posdiv) || inst->pll_posdiv < 0)
			inst->pll_posdiv = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,eye-src-host",
					 &inst->eye_src_host) || inst->eye_src_host < 0)
			inst->eye_src_host = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,eye-vrt-host",
					 &inst->eye_vrt_host) || inst->eye_vrt_host < 0)
			inst->eye_vrt_host = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,eye-term-host",
					 &inst->eye_term_host) || inst->eye_term_host < 0)
			inst->eye_term_host = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,rev6-host",
					&inst->rev6_host) || inst->rev6_host < 0)
			inst->rev6_host = -EINVAL;
		inst->hwpll_mode = device_property_read_bool(dev, "mediatek,hwpll-mode");
		inst->refclk_sel = device_property_read_bool(dev, "mediatek,refclk-sel");
		inst->chp_en_disable = device_property_read_bool(dev, "mediatek,chp-en-disable");

		dev_info(dev, "device: vrt:%d term:%d rev6:%d | u2_intr:%d term_cal:%d | discth:%d rx_sqth:%d rx_sqd:%d fs_cr:%d\n",
			inst->eye_vrt, inst->eye_term, inst->rev6,
			inst->efuse_intr, inst->efuse_term_cal,
			inst->discth, inst->rx_sqth, inst->rx_sqd, inst->fs_cr);
		break;
	case PHY_TYPE_USB3:
		if (device_property_read_u32(dev, "mediatek,efuse-intr",
					 &inst->efuse_intr) || inst->efuse_intr < 0)
			inst->efuse_intr = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,efuse-tx-imp",
					 &inst->efuse_tx_imp) || inst->efuse_tx_imp < 0)
			inst->efuse_tx_imp = -EINVAL;
		if (device_property_read_u32(dev, "mediatek,efuse-rx-imp",
					 &inst->efuse_rx_imp) || inst->efuse_rx_imp < 0)
			inst->efuse_rx_imp = -EINVAL;
		break;
	case PHY_TYPE_PCIE:
	case PHY_TYPE_SGMII:
		break;
	default:
		dev_err(xsphy->dev, "incompatible phy type\n");
		return;
	}
}

static int mtk_phy_init(struct phy *phy)
{
	struct xsphy_instance *inst = phy_get_drvdata(phy);
	struct mtk_xsphy *xsphy = dev_get_drvdata(phy->dev.parent);
	int ret;

	ret = clk_prepare_enable(inst->ref_clk);
	if (ret) {
		dev_err(xsphy->dev, "failed to enable ref_clk\n");
		return ret;
	}

	switch (inst->type) {
	case PHY_TYPE_USB2:
		u2_phy_instance_init(xsphy, inst);
		u2_phy_props_set(xsphy, inst);
		break;
	case PHY_TYPE_USB3:
		u3_phy_props_set(xsphy, inst);
		break;
	case PHY_TYPE_PCIE:
	case PHY_TYPE_SGMII:
		break;
	default:
		dev_err(xsphy->dev, "incompatible phy type\n");
		clk_disable_unprepare(inst->ref_clk);
		return -EINVAL;
	}

	return 0;
}

static int mtk_phy_power_on(struct phy *phy)
{
	struct xsphy_instance *inst = phy_get_drvdata(phy);
	struct mtk_xsphy *xsphy = dev_get_drvdata(phy->dev.parent);

	if (inst->type == PHY_TYPE_USB2) {
		u2_phy_instance_power_on(xsphy, inst);
		u2_phy_slew_rate_calibrate(xsphy, inst);
	}

	return 0;
}

static int mtk_phy_power_off(struct phy *phy)
{
	struct xsphy_instance *inst = phy_get_drvdata(phy);
	struct mtk_xsphy *xsphy = dev_get_drvdata(phy->dev.parent);

	if (inst->type == PHY_TYPE_USB2)
		u2_phy_instance_power_off(xsphy, inst);

	return 0;
}

static int mtk_phy_exit(struct phy *phy)
{
	struct xsphy_instance *inst = phy_get_drvdata(phy);

	clk_disable_unprepare(inst->ref_clk);
	return 0;
}

static int mtk_phy_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct xsphy_instance *inst = phy_get_drvdata(phy);
	struct mtk_xsphy *xsphy = dev_get_drvdata(phy->dev.parent);

	if (inst->type == PHY_TYPE_USB2) {
		/* For device mode: apply device props and set IDDIG */
		if (mode == PHY_MODE_USB_DEVICE && !submode) {
			u2_phy_props_set(xsphy, inst);
			mtk_phy_set_bits(inst->port_base + XSP_U2PHYDTM1,
					 P2D_FORCE_IDDIG | P2D_RG_IDDIG);
		} else if (mode == PHY_MODE_USB_HOST && !submode) {
			u2_phy_host_props_set(xsphy, inst);
			mtk_phy_set_bits(inst->port_base + XSP_U2PHYDTM1,
					 P2D_FORCE_IDDIG);
			mtk_phy_clear_bits(inst->port_base + XSP_U2PHYDTM1,
					   P2D_RG_IDDIG);
		} else if (!submode) {
			/* OTG: clear force */
			mtk_phy_clear_bits(inst->port_base + XSP_U2PHYDTM1,
					   P2D_FORCE_IDDIG | P2D_RG_IDDIG);
		}

		/* DPPULLUP_SET: SoC EN_PU_DP */
		if (submode == 5) { /* PHY_MODE_DPPULLUP_SET */
			mtk_phy_set_bits(inst->port_base + XSP_USBPHYACR3,
					(P2A3_RG_USB20_PUPD_BIST_EN |
					P2A3_RG_USB20_EN_PU_DP));
		} else if (submode == 6) { /* PHY_MODE_DPPULLUP_CLR */
			mtk_phy_clear_bits(inst->port_base + XSP_USBPHYACR3,
					(P2A3_RG_USB20_PUPD_BIST_EN |
					P2A3_RG_USB20_EN_PU_DP));
		}
	}

	return 0;
}

static struct phy *mtk_phy_xlate(struct device *dev,
				 const struct of_phandle_args *args)
{
	struct mtk_xsphy *xsphy = dev_get_drvdata(dev);
	struct xsphy_instance *inst = NULL;
	struct device_node *phy_np = args->np;
	int index;

	if (args->args_count != 1) {
		dev_err(dev, "invalid number of cells in 'phy' property\n");
		return ERR_PTR(-EINVAL);
	}

	for (index = 0; index < xsphy->nphys; index++)
		if (phy_np == xsphy->phys[index]->phy->dev.of_node) {
			inst = xsphy->phys[index];
			break;
		}

	if (!inst) {
		dev_err(dev, "failed to find appropriate phy\n");
		return ERR_PTR(-EINVAL);
	}

	inst->type = args->args[0];
	if (!(inst->type == PHY_TYPE_USB2 ||
	      inst->type == PHY_TYPE_USB3 ||
	      inst->type == PHY_TYPE_PCIE ||
	      inst->type == PHY_TYPE_SGMII)) {
		dev_err(dev, "unsupported phy type: %d\n", inst->type);
		return ERR_PTR(-EINVAL);
	}

	phy_parse_property(xsphy, inst);
	phy_type_set(inst);

	return inst->phy;
}

static const struct phy_ops mtk_xsphy_ops = {
	.init		= mtk_phy_init,
	.exit		= mtk_phy_exit,
	.power_on	= mtk_phy_power_on,
	.power_off	= mtk_phy_power_off,
	.set_mode	= mtk_phy_set_mode,
	.owner		= THIS_MODULE,
};

static const struct of_device_id mtk_xsphy_id_table[] = {
	{ .compatible = "mediatek,xsphy", },
	{ },
};
MODULE_DEVICE_TABLE(of, mtk_xsphy_id_table);

static int mtk_xsphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct phy_provider *provider;
	struct resource *glb_res;
	struct mtk_xsphy *xsphy;
	struct resource res;
	size_t nphys;
	int port;

	nphys = of_get_child_count(np);
	xsphy = devm_kzalloc(dev, offsetof(struct mtk_xsphy, phys) +
			     nphys * sizeof(xsphy->phys[0]), GFP_KERNEL);
	if (!xsphy)
		return -ENOMEM;

	xsphy->nphys = nphys;
	xsphy->dev = dev;
	platform_set_drvdata(pdev, xsphy);

	glb_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	/* optional, may not exist if no u3 phys */
	if (glb_res) {
		xsphy->glb_base = devm_ioremap_resource(dev, glb_res);
		if (IS_ERR(xsphy->glb_base)) {
			dev_err(dev, "failed to remap glb regs\n");
			return PTR_ERR(xsphy->glb_base);
		}
	}

	xsphy->src_ref_clk = XSP_REF_CLK;
	xsphy->src_coef = XSP_SLEW_RATE_COEF;
	device_property_read_u32(dev, "mediatek,src-ref-clk-mhz",
				 &xsphy->src_ref_clk);
	device_property_read_u32(dev, "mediatek,src-coef", &xsphy->src_coef);

	xsphy->tx_chirpK_disable = device_property_read_bool(dev,
				"tx-chirpk-capable");
	xsphy->bc11_switch_disable = device_property_read_bool(dev,
			"bc11-switch-disable");

	port = 0;
	for_each_child_of_node_scoped(np, child_np) {
		struct xsphy_instance *inst;
		struct phy *phy;
		int retval;

		inst = devm_kzalloc(dev, sizeof(*inst), GFP_KERNEL);
		if (!inst)
			return -ENOMEM;

		xsphy->phys[port] = inst;

		phy = devm_phy_create(dev, child_np, &mtk_xsphy_ops);
		if (IS_ERR(phy)) {
			dev_err(dev, "failed to create phy\n");
			return PTR_ERR(phy);
		}

		retval = of_address_to_resource(child_np, 0, &res);
		if (retval) {
			dev_err(dev, "failed to get address resource(id-%d)\n",
				port);
			return retval;
		}

		inst->port_base = devm_ioremap_resource(&phy->dev, &res);
		if (IS_ERR(inst->port_base)) {
			dev_err(dev, "failed to remap phy regs\n");
			return PTR_ERR(inst->port_base);
		}

		inst->phy = phy;
		inst->index = port;
		phy_set_drvdata(phy, inst);
		port++;

		inst->ref_clk = devm_clk_get(&phy->dev, "ref");
		if (IS_ERR(inst->ref_clk)) {
			dev_err(dev, "failed to get ref_clk(id-%d)\n", port);
			return PTR_ERR(inst->ref_clk);
		}

		retval = phy_type_syscon_get(inst, child_np);
		if (retval)
			return retval;
	}

	provider = devm_of_phy_provider_register(dev, mtk_phy_xlate);
	return PTR_ERR_OR_ZERO(provider);
}

static struct platform_driver mtk_xsphy_driver = {
	.probe		= mtk_xsphy_probe,
	.driver		= {
		.name	= "mtk-xsphy",
		.of_match_table = mtk_xsphy_id_table,
	},
};
module_platform_driver(mtk_xsphy_driver);

MODULE_AUTHOR("Chunfeng Yun <chunfeng.yun@mediatek.com>");
MODULE_DESCRIPTION("MediaTek USB XS-PHY driver");
MODULE_LICENSE("GPL v2");
