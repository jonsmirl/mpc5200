/* linux/arch/arm/mach-lpc313x/include/mach/i2c.h
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * LPC313x-specific tweaks for NXP I2C block
 *
 * Based on mach-pnx4008/include/mach/i2c.h by Vitaly Wool <vwool@ru.mvista.com>
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

#ifndef __ASM_ARCH_I2C_H__
#define __ASM_ARCH_I2C_H__

enum {
	mstatus_tdi = 0x00000001,
	mstatus_afi = 0x00000002,
	mstatus_nai = 0x00000004,
	mstatus_drmi = 0x00000008,
	mstatus_active = 0x00000020,
	mstatus_scl = 0x00000040,
	mstatus_sda = 0x00000080,
	mstatus_rff = 0x00000100,
	mstatus_rfe = 0x00000200,
	mstatus_tff = 0x00000400,
	mstatus_tfe = 0x00000800,
};

enum {
	mcntrl_tdie = 0x00000001,
	mcntrl_afie = 0x00000002,
	mcntrl_naie = 0x00000004,
	mcntrl_drmie = 0x00000008,
	mcntrl_rffie = 0x00000020,
	mcntrl_daie = 0x00000040,
	mcntrl_tffie = 0x00000080,
	mcntrl_reset = 0x00000100,
};

enum {
	rw_bit = 1 << 0,
	start_bit = 1 << 8,
	stop_bit = 1 << 9,
};

#define I2C_REG_RX(a)	((a)->ioaddr)		/* Rx FIFO reg (RO) */
#define I2C_REG_TX(a)	((a)->ioaddr)		/* Tx FIFO reg (WO) */
#define I2C_REG_STS(a)	((a)->ioaddr + 0x04)	/* Status reg (RO) */
#define I2C_REG_CTL(a)	((a)->ioaddr + 0x08)	/* Ctl reg */
#define I2C_REG_CKH(a)	((a)->ioaddr + 0x0C)	/* Clock divider high */
#define I2C_REG_CKL(a)	((a)->ioaddr + 0x10)	/* Clock divider low */
#define I2C_REG_ADR(a)	((a)->ioaddr + 0x14)	/* I2C address */
#define I2C_REG_RFL(a)	((a)->ioaddr + 0x18)	/* Rx FIFO level (RO) */
#define I2C_REG_TFL(a)	((a)->ioaddr + 0x1c)	/* Tx FIFO level (RO) */
#define I2C_REG_RXB(a)	((a)->ioaddr + 0x20)	/* Num of bytes Rx-ed (RO) */
#define I2C_REG_TXB(a)	((a)->ioaddr + 0x24)	/* Num of bytes Tx-ed (RO) */
#define I2C_REG_TXS(a)	((a)->ioaddr + 0x28)	/* Tx slave FIFO (RO) */
#define I2C_REG_STFL(a)	((a)->ioaddr + 0x2c)	/* Tx slave FIFO level (RO) */

#define I2C_CHIP_NAME		"LPC313x-I2C"

#endif				/* __ASM_ARCH_I2C_H___ */
