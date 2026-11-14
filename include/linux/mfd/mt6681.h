/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#ifndef __MT6681_H__
#define __MT6681_H__

#include <linux/regmap.h>

#define MT6681_PMIC_SLAVEID	(0x6B)

struct i2c_adapter;
struct i2c_client;

struct mt6681_pmic_info {
	struct i2c_client *i2c;
	struct device *dev;
	struct regmap *regmap;
	unsigned int chip_rev;
	struct mutex io_lock;
};

/*
 * The vendor codec brackets every register access with these to keep the SCP
 * awake when the PMIC sits on the SCP I2C controller.  We drive it over a
 * plain i2c-gpio bus, so there is nothing to wake.
 */
static inline int scp_wake_request(struct i2c_adapter *adap)
{
	return 0;
}

static inline int scp_wake_release(struct i2c_adapter *adap)
{
	return 0;
}

#endif /* __MT6681_H__ */
