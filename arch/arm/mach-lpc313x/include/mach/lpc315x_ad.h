/*
 * arch/arm/mach-lpc313x/include/mach/lpc315x_ad.h
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2010 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
  */

#ifndef __MACH_LPC315X_AD_H
#define __MACH_LPC315X_AD_H

/* Get I2C client structure function */
extern struct i2c_client *lpc315x_ad_get_i2c_client_struct(void);

#endif
