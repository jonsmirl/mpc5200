/*
 * Maxim max9485 Programmable Audio Clock Generator driver
 *
 * Written by: Jon Smirl <jonsmirl@gmail.com>
 *
 * Copyright (C) 2008 Digispeaker.com
 * Copyright (C) 2008 Jon Smirl <jonsmirl@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_I2C_MAX9485_H
#define __LINUX_I2C_MAX9485_H

struct i2c_client;

/* Defines for Maxim MAX9485 Audio Clock Generator */

#define MAX9485_MCLK         (1 << 7)
#define MAX9485_CLK_OUT_2    (1 << 6)
#define MAX9485_CLK_OUT_1    (1 << 5)
#define MAX9485_DOUBLED      (1 << 4)
#define MAX9485_SCALE_256    (0 << 2)
#define MAX9485_SCALE_384    (1 << 2)
#define MAX9485_SCALE_768    (2 << 2)
#define MAX9485_FREQUENCY_12  0
#define MAX9485_FREQUENCY_32  1
#define MAX9485_FREQUENCY_441 2
#define MAX9485_FREQUENCY_48  3

/* Combinations that minimize jitter */
#define MAX9485_245760 (MAX9485_SCALE_256 | MAX9485_FREQUENCY_48 | MAX9485_DOUBLED)
#define MAX9485_225792 (MAX9485_SCALE_256 | MAX9485_FREQUENCY_441 | MAX9485_DOUBLED)

int max9485_set(struct i2c_client *max9485, u8 value);

#endif /*  __LINUX_I2C_MAX9485_H */
