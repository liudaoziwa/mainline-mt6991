/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2019 MediaTek Inc.
 */
#ifndef __MTK_SIP_SVC_H
#define __MTK_SIP_SVC_H

/* Error Code */
#define SIP_SVC_E_SUCCESS               0
#define SIP_SVC_E_NOT_SUPPORTED         -1
#define SIP_SVC_E_INVALID_PARAMS        -2
#define SIP_SVC_E_INVALID_RANGE         -3
#define SIP_SVC_E_PERMISSION_DENIED     -4

#ifdef CONFIG_ARM64
#define MTK_SIP_SMC_CONVENTION          ARM_SMCCC_SMC_64
#else
#define MTK_SIP_SMC_CONVENTION          ARM_SMCCC_SMC_32
#endif

#define MTK_SIP_SMC_CMD(fn_id) \
	ARM_SMCCC_CALL_VAL(ARM_SMCCC_FAST_CALL, MTK_SIP_SMC_CONVENTION, \
			   ARM_SMCCC_OWNER_SIP, fn_id)

/* Modem related SMC call */
#define MTK_SIP_KERNEL_CCCI_CONTROL	MTK_SIP_SMC_CMD(0x505)

/* DVFSRC SMC calls */
#define MTK_SIP_DVFSRC_VCOREFS_CONTROL	MTK_SIP_SMC_CMD(0x506)

/* IOMMU related SMC call */
#define MTK_SIP_KERNEL_IOMMU_CONTROL	MTK_SIP_SMC_CMD(0x514)

/* Debug related SMC call (modem register dump, vendor value) */
#define MTK_SIP_KERNEL_CCCI_CONTROL	MTK_SIP_SMC_CMD(0x505)

/* HWCCF (voter clock control) SMC call, vendor value */
#define MTK_SIP_KERNEL_HWCCF_CONTROL	MTK_SIP_SMC_CMD(0x540)

/* SCP (tinysys) control SMC call, vendor value */
#define MTK_SIP_TINYSYS_SCP_CONTROL	MTK_SIP_SMC_CMD(0x528)

/* SCP DVFS control SMC call, vendor value */
#define MTK_SIP_SCP_DVFS_CONTROL	MTK_SIP_SMC_CMD(0x232)

/* Connectivity / PCIe / GPUEB control (values shared with the vendor ATF) */
#define MTK_SIP_KERNEL_PCIE_CONTROL	MTK_SIP_SMC_CMD(0x52F)
#define MTK_SIP_KERNEL_GPUEB_CONTROL	MTK_SIP_SMC_CMD(0x530)
#define MTK_SIP_KERNEL_CONNSYS_CONTROL	MTK_SIP_SMC_CMD(0x534)
#define MTK_SIP_KERNEL_WLAN_CONTROL	MTK_SIP_SMC_CMD(0x535)
#define MTK_SIP_KERNEL_BT_CONTROL	MTK_SIP_SMC_CMD(0x536)
#define MTK_SIP_KERNEL_GPS_CONTROL	MTK_SIP_SMC_CMD(0x537)

#endif
