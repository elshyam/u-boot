/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2016 Rockchip Electronics Co., Ltd
 */

#ifndef __EVB_RK3399_H
#define __EVB_RK3399_H

#include <configs/rk3399_common.h>

#ifndef CONFIG_TPL_BUILD

#define CONFIG_SPL_OS_BOOT

/* Falcon Mode */
#define CONFIG_SPL_FS_LOAD_ARGS_NAME   "args"
#define CONFIG_SPL_FS_LOAD_KERNEL_NAME "uImage"
#define CONFIG_CMD_SPL
#define CONFIG_SYS_SPL_ARGS_ADDR       0x0ffe5000
#define CONFIG_CMD_SPL_WRITE_SIZE      (128 * SZ_1K)

/* Falcon Mode - MMC support: args@1MB kernel@2MB */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTOR  0x800   /* 1MB */
#define CONFIG_SYS_MMCSD_RAW_MODE_ARGS_SECTORS (CONFIG_CMD_SPL_WRITE_SIZE / 512)
#define CONFIG_SYS_MMCSD_RAW_MODE_KERNEL_SECTOR        0x1000  /* 2MB */
#endif


#define CONFIG_SYS_MMC_ENV_DEV 1

#define SDRAM_BANK_SIZE			(2UL << 30)

#endif
