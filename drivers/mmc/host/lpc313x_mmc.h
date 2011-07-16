/* LPC313x MultiMedia Card Interface driver
 *
 * Copyright (C)2009 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __DRIVERS_MMC_LPC313x_MCI_H__
#define __DRIVERS_MMC_LPC313x_MCI_H__

#define MCI_SLOT 0

/* Register access macros */
#define mci_readl(reg)				\
	__raw_readl(&SDMMC_##reg)
#define mci_writel(reg,value)			\
	__raw_writel((value),&SDMMC_##reg)

#endif
