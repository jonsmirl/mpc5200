/*  arch/arm/mach-lpc313x/generic.h
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  Defines prototypes for generic init functions LPC313x and LPC315x SoCs.
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
#ifndef __MACH_BOARD_H
#define __MACH_BOARD_H

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mmc/host.h>
#include <linux/mtd/partitions.h>
#include <linux/mmc/host.h>


extern void __init lpc313x_map_io(void);
extern void __init lpc313x_init_irq(void);
extern int __init lpc313x_init(void);
extern int __init lpc313x_register_i2c_devices(void);
extern void lpc313x_vbus_power(int enable);
extern int lpc313x_entering_suspend_mem(void);


struct sys_timer;
extern struct sys_timer lpc313x_timer;

/*
 * Timing information structure for the NAND interface. Although there are
 * multiple chip selects for the NAND controller, there is only 1 set of
 * timing data shared among all chip selects. All the parts should be of
 * the same type or very similar timings. These values are used to adjust
 * the NAND timing to the current system clock speed.
 *
 * These values are NanoSecond timings. See the LPC31xx Users Guide for
 * information on what these timings are and set the value for each timing
 * with the matching value from the NAND device data sheet.
 */
struct lpc313x_nand_timing
{
	u32 ns_trsd;
	u32 ns_tals;
	u32 ns_talh;
	u32 ns_tcls;
	u32 ns_tclh;
	u32 ns_tdrd;
	u32 ns_tebidel;
	u32 ns_tch;
	u32 ns_tcs;
	u32 ns_treh;
	u32 ns_trp;
	u32 ns_trw;
	u32 ns_twp;
};

/*
 * This structure is required for each chip select with an attached device
 * and partitioning scheme. One of these structures is required for each
 * device attached to a chip select of the NAND controller.
 */
struct lpc313x_nand_dev_info
{
	char *name; /* Informational name only */
	int nr_partitions; /* Number of partitions on this device */
	struct mtd_partition *partitions; /* Pointer to partition table */
};

/*
 * High level NAND configuration structure
 */
struct lpc313x_nand_cfg {
	int nr_devices;
	struct lpc313x_nand_dev_info *devices;
	struct lpc313x_nand_timing *timing;
	int support_16bit;
};

/*
 * Specifies behaviour of each supported chip select
 */
typedef void (*spi_cs_sel)(int, int);
struct lpc313x_spics_cfg {
	/* spi_spo is the serial clock polarity between transfers, 1 = high level,
	   0 = low */
	u8 spi_spo;
	/* spi_sph is the control for clock edge capture, 0 = capture data on 1rst
	   clock edge, 1 = second edge capture */
	u8 spi_sph;
	spi_cs_sel spi_cs_set; /* Sets state of SPI chip select */
};

/*
 * Defines the number of chip selects and the cs data
 */
struct lpc313x_spi_cfg {
	u32 num_cs; /* Number of CS supported on this board */
	/* Array of cs setup data (num_cs entries) */
	struct lpc313x_spics_cfg *spics_cfg;
};

#if defined (CONFIG_MACH_VAL3153) 
#define MAX_MCI_SLOTS		2
#else
#define MAX_MCI_SLOTS		1
#endif

/*
 * the board-type specific routines
 */
struct lpc313x_mci_board {
	u32 num_slots;
	u32 detect_delay_ms; /* delay in mS before detecting cards after interrupt */
	int (*init)(u32 slot_id, irq_handler_t , void *);
	int (*get_ro)(u32 slot_id);
	int (*get_cd)(u32 slot_id);
	int (*get_ocr)(u32 slot_id);
	int (*get_bus_wd)(u32 slot_id);
	/*
	 * Enable power to selected slot and set voltage to desired level.
	 * Voltage levels are specified using MMC_VDD_xxx defines defined
	 * in linux/mmc/host.h file.
	 */
	void (*setpower)(u32 slot_id, u32 volt);
	void (*exit)(u32 slot_id);
	void (*select_slot)(u32 slot_id);
};

struct lpc313x_mci_irq_data {
	u32 irq;
	irq_handler_t irq_hdlr;
	void* data;
};

#endif /*__MACH_BOARD_H*/

