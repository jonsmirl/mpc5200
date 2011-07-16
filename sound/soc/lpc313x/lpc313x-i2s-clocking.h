/*
 * sound/soc/lpc313x/lpc313x-i2s-clocking.h
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2009 NXP Semiconductors
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
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * This file provides the necessary clocking and control functions for the
 * sound drivers on the LPC313X. These functions include clock enable and
 * disable, and clock frequency setup on the WS, BCLK, and SYSCLK pins.
 *
 * Pin mapping is as follows:
 *     Chip signal name         CODEC signal name
 *     CLK_256FS                Audio CODEC clock
 *     I2SRX_WS0                I2S RX channel 0 Word select clock
 *     I2SRX_WS1                I2S RX channel 1 Word select clock
 *     I2STX_WS0                I2S TX channel 0 Word select clock
 *     I2STX_WS1                I2S TX channel 1 Word select clock
 *     I2SRX_BCK0               I2S RX channel 0 serial data bit clock
 *     I2SRX_BCK1               I2S RX channel 1 serial data bit clock
 *     I2STX_BCK0               I2S TX channel 0 serial data bit clock
 *     I2STX_BCK1               I2S TX channel 1 serial data bit clock
 */

#ifndef __LPC313X_I2S_CLOCKING_H
#define __LPC313X_I2S_CLOCKING_H

#include <linux/types.h>

/*
 * I2S supported clock groupings
 */
enum i2s_supp_clks {CLK_RX_0, CLK_TX_0, CLK_RX_1, CLK_TX_1};

/*
 * Set the audio CODEC clock rate or 0 to disable the clock, returns
 * the actual programmed clock rate. The programmed rate is generated on
 * the FS256 pin has a rate of (256 * clkrate).
 */
u32 lpc313x_main_clk_rate(u32 clkrate);

/*
 * Set a specific channel's bit clock and word select rates. his applies
 * to the channel's WS and BCLK signals. Returns the actual programmed
 * WS clock rate.
 */
u32 lpc313x_chan_clk_enable(enum i2s_supp_clks chclk, u32 ws_freq, u32 bit_freq);

#endif /* __LPC313X_I2S_CLOCKING */
