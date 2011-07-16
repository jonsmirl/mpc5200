/*
 * LPC313x NAND Controller Interface driver
 *
 * drivers/mtd/nand/lpc313x_nand.c
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
 * TODO
 * Verify BB check code - I don't think it's working for all device sizes
 * Huge block support may not work due to kernel limitations
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/cpufreq.h>
#include <linux/dma-mapping.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/mtd/nand_ecc.h>
#include <linux/mtd/partitions.h>

#include <asm/io.h>

#include <mach/irqs.h>
#include <mach/registers.h>
#include <linux/mmc/host.h>
#include <mach/gpio.h>
#include <mach/cgu.h>
#include <mach/board.h>
/* for time being use arch specific DMA framework
 * instead of generic framework
 * */
#include <mach/dma.h>

/*  Enable DMA transfer for better throughput
 * */
#define USE_DMA

/* Maximum number of DMA descritpors in SG table
 * */
#define NAND_DMA_MAX_DESC 4

/* Register access macros */
#define nand_readl(reg)		__raw_readl(&NAND_##reg)
#define nand_writel(reg,value)	__raw_writel((value),&NAND_##reg)
#define sys_writel(reg,value)	__raw_writel((value),&SYS_##reg)

#define OOB_FREE_OFFSET 4

/* Enable for polling support only. Polling support will compile the
   code without interrupts during read and write cycles. Device ready
   status, buffer status, and ECC status are all polled. This may give
   a slight performance improvement at the expense of CPU usage. For
   very slow NAND devices, you wouldn't want to use polling. */
//#define STATUS_POLLING

/* Huge block support not working in 2.6.28.2 kernel, don't use this! */
//#define HUGE_BLOCK_SUPPORT

/* Device specific MTD structure, 1 per chip select */
struct lpc313x_nand_mtd {
	struct mtd_info mtd;
	struct nand_chip chip;
	struct lpc313x_nand_info *host;
};

/* Local driver data structure */
struct lpc313x_nand_info {
	struct nand_hw_control controller;
	struct lpc313x_nand_cfg *platform;
	struct lpc313x_nand_mtd *mtds;
	struct device *dev;
	u32 nandconfig;
	int current_cs;
#ifdef USE_DMA
	int	dma_chn;
	dma_addr_t sg_dma;
	dma_sg_ll_t *sg_cpu;
	wait_queue_head_t dma_waitq;
	volatile u32 dmapending;
#endif
	int irq;
	wait_queue_head_t irq_waitq;
	volatile u32 intspending;
};

/* Chip select specific ready check masks */
static const int rdymasks[4] = {
	NAND_NANDCHECKSTS_RB1_LVL,
	NAND_NANDCHECKSTS_RB2_LVL,
	NAND_NANDCHECKSTS_RB3_LVL,
	NAND_NANDCHECKSTS_RB4_LVL
};

/* Decode and encode buffer ECC status masks */
static const u32 nand_buff_dec_mask[2] = {
	NAND_NANDIRQSTATUS1_ECC_DEC_RAM0, NAND_NANDIRQSTATUS1_ECC_DEC_RAM1};
static const u32 nand_buff_enc_mask[2] = {
	NAND_NANDIRQSTATUS1_ECC_ENC_RAM0, NAND_NANDIRQSTATUS1_ECC_ENC_RAM1};
static const u32 nand_buff_wr_mask[2] = {NAND_NANDIRQSTATUS1_WR_RAM0,
	NAND_NANDIRQSTATUS1_WR_RAM1};

/* Decode buffer addresses */
static const void *nand_buff_addr[2] = {
	(void *) &NAND_BUFFER_ADRESS, (void *) (&NAND_BUFFER_ADRESS + 256)};

#ifdef USE_DMA
/* Decode buffer physical addresses */
static const u32 nand_buff_phys_addr[2] = {
	IO_NAND_BUF_PHYS, (IO_NAND_BUF_PHYS + 0x400)};
#endif

/*
 *
 * OOB data placement structures for small/large/huge block FLASH
 *
 */

/*
 * Autoplacement pattern for 512+16 bytes small block NAND FLASH
 */
static struct nand_ecclayout nand_hw_eccoob_16 = {
	.eccbytes	= 12,
	.eccpos		= {
				4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15
			  },
	.oobfree	= {
				{0, 4},
			  }
};

/*
 * Autoplacement pattern for 2048+64 bytes large block NAND FLASH
 */
static struct nand_ecclayout nand_hw_eccoob_64 = {
	.eccbytes	= 48,
	.eccpos		= {
				4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
				20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
				36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
				52, 53, 54, 55, 56, 67, 58, 59, 60, 61, 62, 63
			  },
	.oobfree	= {
				{0, 4},
				{16, 4},
				{32, 4},
				{48, 4}
			  }
};

#ifdef HUGE_BLOCK_SUPPORT
/*
 * Autoplacement pattern for 4096+128 bytes large block NAND FLASH
 */
static struct nand_ecclayout nand_hw_eccoob_128 = {
	.eccbytes	= 96,
	.eccpos		= {
				4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15,
				20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31,
				36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47,
				52, 53, 54, 55, 56, 67, 58, 59, 60, 61, 62, 63,
				68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79,
				84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95,
				100, 101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111,
				116, 117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127
			  },
	.oobfree	= {
				{0, 4},
				{16, 4},
				{32, 4},
				{48, 4},
				{64, 4},
				{80, 4},
				{96, 4},
				{112, 4}
			  }
};
#endif

/*
 *
 * Bad block descriptors for small/large/huge block FLASH
 *
 */
/*
 *	hardware specific flash bbt decriptors
 */
static uint8_t bbt_pattern[] = { 'B', 'b', 't', '0' };
static uint8_t mirror_pattern[] = { '1', 't', 'b', 'B' };

static struct nand_bbt_descr lpc313x_bbt_main_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 32,
	.len = 4,
	.veroffs = 48,
	.maxblocks = 4,
	.pattern = bbt_pattern
};

static struct nand_bbt_descr lpc313x_bbt_mirror_descr = {
	.options = NAND_BBT_LASTBLOCK | NAND_BBT_CREATE | NAND_BBT_WRITE
		| NAND_BBT_2BIT | NAND_BBT_VERSION | NAND_BBT_PERCHIP,
	.offs = 32,
	.len = 4,
	.veroffs = 48,
	.maxblocks = 4,
	.pattern = mirror_pattern
};

// Dummies bytes for bad block ( just for HARDWARE ECC: inaccurate )
static uint8_t scan_ff_pattern[] = { 0xff, 0xff };

static struct nand_bbt_descr lpc313x_largepage_flashbased = {
	.options = NAND_BBT_SCAN2NDPAGE,
	.offs = 50,
	.len = 2,
	.pattern = scan_ff_pattern
};

#ifdef USE_DMA
/*
 * DMA transfer callback function
 * @ chn: Channel number
 * @ type: Interrupt type
 * @ arg: Function argument
 */
static void lpc313x_nand_dma_irq(int chn, dma_irq_type_t type,
		void *arg)
{
	struct lpc313x_nand_info *host = (struct lpc313x_nand_info *)arg;

	/* SG Table ended */
	if (type == DMA_IRQ_SOFTINT)
	{
		/* Flag event and wakeup */
		host->dmapending = 1;
		wake_up(&host->dma_waitq);
	}
	else if (type == DMA_IRQS_ABORT)
	{
		/* DMA data abort - this might not be a
		   error for this channel. Ignore */
	}

	return;
}

/*
 * DMA mapping function
 * host : Pointer to ilpc313x_nand_info structure
 * addr : Address to be DMA mapped
 * size : Size of the buffer
 * rd : DMA direction (1: read operation, 0: Write operation
 */
static dma_addr_t lpc313x_nand_dma_map(struct lpc313x_nand_info *host,
		u32 addr, u32 size, int rd)
{
	void *addr_map;
	dma_addr_t dma_addr;
	enum dma_data_direction dir = rd ? DMA_FROM_DEVICE :
								DMA_TO_DEVICE;

	if ((void *)addr >= high_memory) {
		/* For vmalloced buffers, check if buffer is within PAGE_SIZE.
	 	* If buffer is not within PAGE_SIZE, DMA map will not work
		* If buffer not within PAGE_SIZE, return with 0 (mapping failed)
		* Else DMA map the buffer.
	 	* */
		struct page *p1;

		if (((size_t)addr & PAGE_MASK) !=
				((size_t)(addr + size - 1) & PAGE_MASK)) {
			dev_err(host->dev, "Buffer not within page \r\n");
			return 0;
		}

		/* Get page address address */
		p1 = vmalloc_to_page((void *)addr);
		if (!p1) {
			dev_err(host->dev, "vmalloc_to_page failure \r\n");
			return 0;
		}
		addr_map = page_address(p1) + ((size_t)addr & ~PAGE_MASK);
	}
	else {
		/* kmalloced buffer */
		addr_map = (void *)addr;
	}

	/* Get DMA mapping */
	dma_addr = (u32) dma_map_single(host->dev, (void *) addr_map,
				size, dir);
	if (dma_mapping_error(host->dev, dma_addr))
	{
		dev_err(host->dev, "DMA mapping failure \r\n");
		return 0;
	}

	return dma_addr;
}

/*
 * DMA Scatter Gather transfer function
 * mtd : Pointer to mtd_info structure
 * chip : Pointer to nand_chip structure
 * bufrdy : SRAM buffer index
 * pay_load : Pay load buffer physical address
 * oob_data : OOB data buffer physical address
 * rd : read flag (1: read operation 0: write operation)
 */
static void lpc313x_nand_dma_sg_tfr(struct mtd_info *mtd,
		struct nand_chip *chip, int bufrdy,	u32 pay_load, u32 oob_data, int rd)
{
	struct lpc313x_nand_mtd *nmtd;
	struct lpc313x_nand_info *host;
	int eccsize = chip->ecc.size;
	int oob_size = rd ? chip->ecc.bytes : OOB_FREE_OFFSET;

	nmtd = chip->priv;
	host = nmtd->host;

	/* SG entry to transfer pay load */
	host->sg_cpu[0].setup.src_address = rd ? nand_buff_phys_addr[bufrdy] :
			pay_load;
	host->sg_cpu[0].setup.dest_address = rd ? pay_load :
			nand_buff_phys_addr[bufrdy];
	host->sg_cpu[0].setup.trans_length = (eccsize >> 2) - 1;
	host->sg_cpu[0].setup.cfg = DMA_CFG_CMP_CH_EN |
			DMA_CFG_CMP_CH_NR(host->dma_chn) | DMA_CFG_TX_WORD;
	host->sg_cpu[0].next_entry = host->sg_dma + sizeof(dma_sg_ll_t);

	/* SG entry to transfer OOB data */
	host->sg_cpu[1].setup.src_address = rd ? (nand_buff_phys_addr[bufrdy] +
			eccsize) : oob_data;
	host->sg_cpu[1].setup.dest_address = rd ? oob_data :
			(nand_buff_phys_addr[bufrdy] + eccsize);
	host->sg_cpu[1].setup.trans_length = (oob_size >> 2) - 1;
	host->sg_cpu[1].setup.cfg = DMA_CFG_CMP_CH_EN |
		      DMA_CFG_CMP_CH_NR(host->dma_chn) | DMA_CFG_TX_WORD;
	host->sg_cpu[1].next_entry = host->sg_dma + (sizeof(dma_sg_ll_t) * 2);

	/* SG entry to transfer OOB data */
	host->sg_cpu[2].setup.src_address = host->sg_dma;
	host->sg_cpu[2].setup.dest_address = DMACH_SOFT_INT_PHYS;
	host->sg_cpu[2].setup.trans_length = 1;
	host->sg_cpu[2].setup.cfg = DMA_CFG_TX_WORD;
	host->sg_cpu[2].next_entry = 0;

	/* Program the SG channel */
	dma_prog_sg_channel(host->dma_chn, host->sg_dma);

	/* Enable FINISHED interrupt */
	dma_set_irq_mask(host->dma_chn, 1, 0);
	dma_set_irq_mask((host->dma_chn - 1), 1, 0);

	/* Set counter to 0 */
	dma_write_counter((host->dma_chn - 1), 0);

	/* Start the transfer */
	host->dmapending = 0;
	dma_start_channel(host->dma_chn);

	/* Wait for FINISHED interrupt */
	wait_event(host->dma_waitq, host->dmapending);

	/* Mask all the interrupts for the channel */
	dma_set_irq_mask(host->dma_chn, 1, 1);
	dma_set_irq_mask((host->dma_chn - 1), 1, 1);

	/* Stop the channel */
	dma_stop_channel(host->dma_chn);

	return;
}
#endif

/*
 *
 * NAND controller hardware support functions
 *
 */

/* Enable or disable NAND controller clocks */
static void lpc313x_nand_clocks_disen(int en) {
	/* Enable or disable clocks for NAND Controller */
	cgu_clk_en_dis(CGU_SB_NANDFLASH_S0_CLK_ID, en);
	cgu_clk_en_dis(CGU_SB_NANDFLASH_ECC_CLK_ID, en);
	cgu_clk_en_dis(CGU_SB_NANDFLASH_NAND_CLK_ID, en);
	cgu_clk_en_dis(CGU_SB_NANDFLASH_PCLK_ID, en);

	/* Needed for LPC3143/54 chips only */
	cgu_clk_en_dis(CGU_SB_NANDFLASH_AES_CLK_ID, en);
}

/*
 * Setup NAND interface timing
 */
static void lpc313x_nand_setrate(struct lpc313x_nand_timing *timing) {
	u32 tmp, timing1, timing2, srcclk;

	/* Get the NAND controller base clock rate */
	srcclk = cgu_get_clk_freq(CGU_SB_NANDFLASH_NAND_CLK_ID);

	/* Compute number of clocks for timing1 parameters */
	tmp = srcclk / (1000000000 / timing->ns_trsd);
	if (tmp > 0x3)
		tmp = 0x3;
	timing1 = NAND_NANDTIMING1_TSRD(tmp);
	tmp = srcclk / (1000000000 / timing->ns_tals);
	if (tmp > 0x7)
		tmp = 0x7;
	timing1 |= NAND_NANDTIMING1_TALS(tmp);
	tmp = srcclk / (1000000000 / timing->ns_talh);
	if (tmp > 0x7)
		tmp = 0x7;
	timing1 |= NAND_NANDTIMING1_TALH(tmp);
	tmp = srcclk / (1000000000 / timing->ns_tcls);
	if (tmp > 0x7)
		tmp = 0x7;
	timing1 |= NAND_NANDTIMING1_TCLS(tmp);
	tmp = srcclk / (1000000000 / timing->ns_tclh);
	if (tmp > 0x7)
		tmp = 0x7;
	timing1 |= NAND_NANDTIMING1_TCLH(tmp);
	nand_writel(TIMING1, timing1);


	/* Compute number of clocks for timing2 parameters */
	tmp = srcclk / (1000000000 / timing->ns_tdrd);
	if (tmp > 0x3)
		tmp = 0x3;
	timing2 = NAND_NANDTIMING2_TDRD(tmp);
	tmp = srcclk / (1000000000 / timing->ns_tebidel);
	if (tmp > 0x7)
		tmp = 0x7;
	timing2 |= NAND_NANDTIMING2_TEBI(tmp);
	tmp = srcclk / (1000000000 / timing->ns_tch);
	if (tmp > 0x7)
		tmp = 0x7;
	timing2 |= NAND_NANDTIMING2_TCH(tmp);
	tmp = srcclk / (1000000000 / timing->ns_tcs);
	if (tmp > 0x7)
		tmp = 0x7;
	timing2 |= NAND_NANDTIMING2_TCS(tmp);
	tmp = srcclk / (1000000000 / timing->ns_treh);
	if (tmp > 0x7)
		tmp = 0x7;
	timing2 |= NAND_NANDTIMING2_TRH(tmp);
	tmp = srcclk / (1000000000 / timing->ns_trp);
	if (tmp > 0x7)
		tmp = 0x7;
	timing2 |= NAND_NANDTIMING2_TRP(tmp);
	tmp = srcclk / (1000000000 / timing->ns_trw);
	if (tmp > 0x7)
		tmp = 0x7;
	timing2 |= NAND_NANDTIMING2_TWH(tmp);
	tmp = srcclk / (1000000000 / timing->ns_twp);
	if (tmp > 0x7)
		tmp = 0x7;
	timing2 |= NAND_NANDTIMING2_TWP(tmp);
	nand_writel(TIMING2, timing2);
}

/*
 * Initialize the NAND interface
 */
static int lpc313x_nand_inithw(struct lpc313x_nand_info *host) {
	unsigned long reg;

	/* Disable all NAND interrupts */
	nand_writel(IRQMASK1, ~0);
	nand_writel(IRQMASK2, ~0);

	/* Setup device and controller timing */
	lpc313x_nand_setrate(host->platform->timing);

	/* enable the controller and de-assert nFCE */
	reg = nand_readl(CONFIG) | host->nandconfig;
	nand_writel(CONFIG, reg);

	return 0;
}

/*
 * Enable NAND interrupts
 */
static inline void lpc313x_nand_int_en(u32 mask) {
#if !defined(STATUS_POLLING)
	u32 tmp = nand_readl(IRQMASK1) & ~mask;

	nand_writel(IRQMASK1, tmp);
#endif
}

/*
 * Disable NAND interrupts
 */
static inline void lpc313x_nand_int_dis(u32 mask) {
#if !defined(STATUS_POLLING)
	u32 tmp = nand_readl(IRQMASK1) | mask;

	nand_writel(IRQMASK1, tmp);
#endif
}

/*
 * Clear NAND interrupts
 */
static inline void lpc313x_nand_int_clear(u32 mask) {
	nand_writel(IRQSTATUSRAW1, mask);
}

/*
 * Return pending NAND interrupts status
 */
static inline u32 lpc313x_nand_int_get(void) {
	return nand_readl(IRQSTATUS1);
}

/*
 * Return raw NAND interrupts status
 */
static inline u32 lpc313x_nand_raw_get(void) {
	return nand_readl(IRQSTATUSRAW1);
}

/*
 * Wait for NAND event
 */
static inline void lpc313x_wait_irq(struct lpc313x_nand_info *host) {
	wait_event(host->irq_waitq, host->intspending);
}

/*
 * Handle the NAND interrupt
 */
static irqreturn_t lpc313x_nandc_irq(int irq, void *dev_id)
{
	/* IRQs not working yet */
	struct lpc313x_nand_info *host = (struct lpc313x_nand_info *) dev_id;

	/* Disable interrupts for now, but don't clear status yet */
	host->intspending = lpc313x_nand_int_get();
	lpc313x_nand_int_dis(~0);

	/* Wakeup pending request */
	wake_up(&host->irq_waitq);

	return IRQ_HANDLED;
}

/*
 * Start a RAM read operation on RAM0 or RAM1
 */
static inline void lpc313x_ram_read(int bufnum) {
	if (bufnum == 0) {
		/* Use RAM buffer 0 */
		nand_writel(CONTROLFLOW, NAND_CTRL_RD_RAM0);
	}
	else {
		/* Use RAM buffer 1 */
		nand_writel(CONTROLFLOW, NAND_CTRL_RD_RAM1);
	}

	lpc313x_nand_int_en(nand_buff_dec_mask[bufnum]);
}

/*
 * Start a RAM write operation on RAM0 or RAM1
 */
static inline void lpc313x_ram_write(int bufnum) {
	if (bufnum == 0) {
		/* Use RAM buffer 0 */
		nand_writel(CONTROLFLOW, NAND_CTRL_WR_RAM0);
	}
	else {
		/* Use RAM buffer 1 */
		nand_writel(CONTROLFLOW, NAND_CTRL_WR_RAM1);
	}

	lpc313x_nand_int_en(nand_buff_wr_mask[bufnum]);
}

/*
 *
 * NAND driver callbacks
 *
 */

/*
 * Asserts and deasserts chip selects (callback)
 */
static void lpc313x_nand_select_chip(struct mtd_info *mtd, int chip_sel) {
	struct nand_chip *chip = mtd->priv;
	struct lpc313x_nand_mtd *nmtd;
	struct lpc313x_nand_info *host;
	int i = 0, cssel = -1;

	nmtd = chip->priv;
	host = nmtd->host;

	if (chip_sel == -1) {
		/* De-assert all the chip selects */
		nand_writel(SETCE, NAND_NANDSETCE_CV_MASK);
	}
	else {
		/* We can determine which chip select should be used by
		   examining the MTD pointer */
		while (i < host->platform->nr_devices) {
			if (mtd == &host->mtds[i].mtd) {
				/* Match */
				cssel = i;
				break;
			}

			i++;
		}

		if (cssel >= 0) {
			host->current_cs = cssel;
			nand_writel(SETCE,
				(NAND_NANDSETCE_CV_MASK & NAND_NANDSETCE_CV(cssel)));
		}
	}
}

/*
 * Issue command and address cycles to the chip (callback)
 */
static void lpc313x_nand_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl) {
	(void) mtd;
	(void) ctrl;

	if (cmd == NAND_CMD_NONE)
		return;

	if (ctrl & NAND_CLE) {
		nand_writel(SETCMD, (u32) cmd);
	}
	else if (ctrl & NAND_ALE) {
		nand_writel(SETADDR, (u32) cmd);
	}
}

/*
 * Returns NAND busy(0)/ready(!0) status callback
 */

static int lpc313x_nand_devready(struct mtd_info *mtd) {
	struct nand_chip *chip = mtd->priv;
	struct lpc313x_nand_mtd *nmtd;
	struct lpc313x_nand_info *host;

	nmtd = chip->priv;
	host = nmtd->host;

	return nand_readl(CHECKSTS) & rdymasks[host->current_cs];
}

/*
 * MTD hardware ECC enable callback
 */
static void lpc313x_nand_enable_hwecc(struct mtd_info *mtd, int mode) {
	(void) mtd;
	(void) mode;

	/* Nothing to really do here, ECC is enabled and used by default */
}

/*
 * MTD ECC data correction callback
 */
static int lpc313x_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				     u_char *read_ecc, u_char *calc_ecc)
{
	u32 tmp;
	int errs_corrected = 0;

	(void) mtd;
	(void) dat;
	(void) calc_ecc;

	/* Data is corrected in hardware, just verify that data is correct per HW */
	if ((nand_readl(IRQSTATUSRAW1) & NAND_NANDIRQSTATUS1_ERR_UNR_RAM0) &
		(read_ecc[OOB_FREE_OFFSET] != 0xFF)) {
		return -1;
	}

	/* Generate correction statistics */
	tmp = lpc313x_nand_raw_get();
	if (!(tmp & (NAND_NANDIRQSTATUS1_NOERR_RAM0 | NAND_NANDIRQSTATUS1_NOERR_RAM1))) {
		if (tmp & (NAND_NANDIRQSTATUS1_ERR1_RAM0 | NAND_NANDIRQSTATUS1_ERR1_RAM1)) {
			errs_corrected = 1;
		}
		else if (tmp & (NAND_NANDIRQSTATUS1_ERR2_RAM0 | NAND_NANDIRQSTATUS1_ERR2_RAM1)) {
			errs_corrected = 2;
		}
		else if (tmp & (NAND_NANDIRQSTATUS1_ERR3_RAM0 | NAND_NANDIRQSTATUS1_ERR3_RAM1)) {
			errs_corrected = 3;
		}
		else if (tmp & (NAND_NANDIRQSTATUS1_ERR4_RAM0 | NAND_NANDIRQSTATUS1_ERR4_RAM1)) {
			errs_corrected = 4;
		}
		else if (tmp & (NAND_NANDIRQSTATUS1_ERR5_RAM0 | NAND_NANDIRQSTATUS1_ERR5_RAM1)) {
			errs_corrected = 5;
		}

		mtd->ecc_stats.corrected += errs_corrected;
	}
	else if (tmp & (NAND_NANDIRQSTATUS1_ERR_UNR_RAM0 | NAND_NANDIRQSTATUS1_ERR_UNR_RAM1)) {
		mtd->ecc_stats.failed++;
	}

	return 0;
}

/*
 * MTD calculate ECC callback
 */
static int lpc313x_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat, u_char *ecc_code)
{
	(void) mtd;
	(void) dat;
	(void) ecc_code;

	/* ECC is calculated automatically in hardware, nothing to do */
	return 0;
}

/*
 * Verify a buffer written to hardware against the passed buffer (callback)
 */
static int lpc313x_nand_verify_hwecc(struct mtd_info *mtd, const uint8_t *buf, int len)
{
	struct nand_chip *chip = mtd->priv;
	struct lpc313x_nand_mtd *nmtd;
	struct lpc313x_nand_info *host;
	int i, status = 0, curbuf = 0, bufrdy = -1;

	nmtd = chip->priv;
	host = nmtd->host;

	/* Read back the data stored in the hardware and check it against the buffer */
	for (i = 0; i < len; i += chip->ecc.size) {
		/* Clear all current statuses */
		lpc313x_nand_int_clear(~0);

		/* Start read into RAM0 or RAM1 */
#if !defined(STATUS_POLLING)
		host->intspending = 0;
#endif
		lpc313x_ram_read(curbuf);

		/* Compare current buffer while next buffer is loading */
		if (bufrdy >= 0) {
			if (memcmp(buf, nand_buff_addr[bufrdy], chip->ecc.size) != 0) {
				status = -EIO;
			}

			buf += chip->ecc.size;
		}

#if defined(STATUS_POLLING)
		/* Polling for buffer loaded and decoded */
		while (!((nand_readl(IRQSTATUSRAW1)) & nand_buff_dec_mask[curbuf]));

#else
		/* Interrupt based wait operation */
		lpc313x_wait_irq(host);
#endif

		bufrdy = curbuf;
		curbuf = 1 - curbuf;
	}

	/* Compare against buffer */
	if (memcmp(buf, nand_buff_addr[bufrdy], chip->ecc.size) != 0) {
		status = -EIO;
	}

	/* Disable all interrupts */
	lpc313x_nand_int_dis(~0);

	return status;
}

/*
 * 8-bit direct NAND interface read callback
 */
static void lpc313x_nand_read_buf8(struct mtd_info *mtd, u_char *buf, int len) {
	struct nand_chip *chip = mtd->priv;

	__raw_readsb(chip->IO_ADDR_R, buf, len);
}

/*
 * 16-bit direct NAND interface read callback
 */
static void lpc313x_nand_read_buf16(struct mtd_info *mtd, u_char *buf, int len) {
	struct nand_chip *chip = mtd->priv;

	len >>= 1;
	__raw_readsw(chip->IO_ADDR_R, buf, len);
}

/*
 * 8-bit direct NAND interface write callback
 */
static void lpc313x_nand_write_buf8(struct mtd_info *mtd, const u_char *buf,
		int len) {
	struct nand_chip *chip = mtd->priv;

	__raw_writesb(chip->IO_ADDR_W, buf, len);
}

/*
 * 16-bit direct NAND interface write callback
 */
static void lpc313x_nand_write_buf16(struct mtd_info *mtd, const u_char *buf,
		int len) {
	struct nand_chip *chip = mtd->priv;

	len >>= 1;
	__raw_writesw(chip->IO_ADDR_W, buf, len);
}

/*
 * Read the payload and OOB data from the device in the hardware storage format
 */
static int lpc313x_nand_read_page_syndrome(struct mtd_info *mtd, struct nand_chip *chip,
				   uint8_t *buf)
{
	int i, curbuf = 0, bufrdy = -1, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	uint8_t *p = buf;
	uint8_t *oob = chip->oob_poi;
#ifdef USE_DMA
	int use_dma = 0;
	dma_addr_t pmapped = 0, oobmapped = 0;
	u32 p1 = 0, oob1 = 0;
#endif

#if !defined(STATUS_POLLING)
	struct lpc313x_nand_mtd *nmtd;
	struct lpc313x_nand_info *host;

	nmtd = chip->priv;
	host = nmtd->host;
#endif

#ifdef USE_DMA
	/* Get DMA mappings for buffers */
	pmapped = lpc313x_nand_dma_map(host, (u32) p, (eccsize * eccsteps), 1);
	oobmapped = lpc313x_nand_dma_map(host, (u32) oob, (eccbytes * eccsteps), 1);
	if((oobmapped) && (pmapped)) {
		p1 = pmapped;
		oob1 = oobmapped;
		use_dma = 1;
	}
#endif

	for (i = eccsteps; i > 0; i--) {
		/* Clear all current statuses */
		lpc313x_nand_int_clear(~0);

		/* Start read into RAM0 or RAM1 */
#if !defined(STATUS_POLLING)
		host->intspending = 0;
#endif
		lpc313x_ram_read(curbuf);

		/* Read current buffer while next buffer is loading */
		if (bufrdy >= 0) {

#ifdef USE_DMA
			/* If DMA mapping succesful, use DMA for transfer.
			 * Else use memcpy for transfer
			 * */
			if(use_dma) {
				/* Read payload & oob using DMA */
				lpc313x_nand_dma_sg_tfr(mtd, chip, bufrdy, p1, oob1, 1);

				/* Update buffers offsets */
				p1 += eccsize;
				oob1 += eccbytes;
			}
			else
#endif
			{
				/* Read payload portion of the transfer */
				memcpy((void *)p, nand_buff_addr[bufrdy], eccsize);
				p += eccsize;

				/* Read OOB data portion of the transfer */
				memcpy((void *)oob, nand_buff_addr[bufrdy] + eccsize, eccbytes);
				oob += eccbytes;
			}
		}

#if defined(STATUS_POLLING)
		/* Polling for buffer loaded and decoded */
		while (!((nand_readl(IRQSTATUSRAW1)) & nand_buff_dec_mask[curbuf]));

#else
		/* Interrupt based wait operation */
		lpc313x_wait_irq(host);
#endif

		bufrdy = curbuf;
		curbuf = 1 - curbuf;

		chip->ecc.correct(mtd, p, oob, NULL);
	}

#ifdef USE_DMA
	if(use_dma) {
		/* Transfer payload & oob using DMA */
		lpc313x_nand_dma_sg_tfr(mtd, chip, bufrdy, p1, oob1, 1);

		/* Unmap DMA mappings */
		dma_unmap_single(host->dev, pmapped, (eccsize * eccsteps),
				DMA_FROM_DEVICE);
		dma_unmap_single(host->dev, oobmapped, (eccbytes * eccsteps),
				DMA_FROM_DEVICE);
	}
	else
#endif
	{
		/* Read payload portion of the transfer */
		memcpy((void *)p, nand_buff_addr[bufrdy], eccsize);

		/* Read OOB data portion of the transfer */
		memcpy((void *)oob, nand_buff_addr[bufrdy] + eccsize, eccbytes);
	}

	/* Disable all interrupts */
	lpc313x_nand_int_dis(~0);

	return 0;
}

/*
 * Read the OOB data from the device in the hardware storage format
 */
static int lpc313x_nand_read_oob_syndrome(struct mtd_info *mtd, struct nand_chip *chip,
				  int page, int sndcmd)
{
	uint8_t *buf = chip->oob_poi;
	int length = mtd->oobsize;
	int chunk = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;
	int eccsize = chip->ecc.size, eccsteps = chip->ecc.steps;
	uint8_t *bufpoi = buf;
	int i, toread, sndrnd = sndcmd, pos;

	chip->cmdfunc(mtd, NAND_CMD_READ0, chip->ecc.size, page);
	for (i = eccsteps; i > 0; i--) {
		/* Random position read needed? */
		if (sndrnd) {
			pos = eccsize + i * (eccsize + chunk);
			if (mtd->writesize > 512)
				chip->cmdfunc(mtd, NAND_CMD_RNDOUT, pos, -1);
			else
				chip->cmdfunc(mtd, NAND_CMD_READ0, pos, page);
		} else {
			sndrnd = 1;
		}

		toread = min_t(int, length, chunk);
		chip->read_buf(mtd, bufpoi, toread);
		bufpoi += toread;
		length -= toread;
	}
	if (length > 0)
		chip->read_buf(mtd, bufpoi, length);

	return 1;
}

/*
 * Write the payload and OOB data to the device in the hardware storage format
 */
static void lpc313x_nand_write_page_syndrome(struct mtd_info *mtd,
				    struct nand_chip *chip, const uint8_t *buf)
{
	int i, curbuf = 0, bufrdy = 0, eccsize = chip->ecc.size;
	int eccbytes = chip->ecc.bytes;
	int eccsteps = chip->ecc.steps;
	const uint8_t *p = buf;
	uint8_t *oob = chip->oob_poi;
#ifdef USE_DMA
	dma_addr_t pmapped, oobmapped;
	u32 p1 = 0, oob1 = 0;
	int use_dma = 0;
#endif

#if !defined(STATUS_POLLING)
	struct lpc313x_nand_mtd *nmtd;
	struct lpc313x_nand_info *host;

	nmtd = chip->priv;
	host = nmtd->host;
#endif

#ifdef USE_DMA
	pmapped = lpc313x_nand_dma_map(host, (u32) p, (eccsize * eccsteps), 0);
	oobmapped = lpc313x_nand_dma_map(host, (u32) oob, (eccbytes * eccsteps), 0);
	if((pmapped) && (oobmapped)) {
		use_dma = 1;
		p1 = pmapped;
		oob1 = oobmapped;
	}
#endif

	/* Clear all current statuses */
	lpc313x_nand_int_clear(~0);
#ifdef USE_DMA
	/* If DMA mapping succesful, use DMA for transfer.
	 * Else use memcpy for transfer
	 * */
	if(use_dma) {
		/* Transfer pay load & OOB using DMA */
		lpc313x_nand_dma_sg_tfr(mtd, chip, bufrdy, p1, oob1, 0);

		/* Update buffer offsets */
		p1 += eccsize;
		oob1 += eccbytes;
	}
	else
#endif
	{
		/* Copy payload and OOB data to the buffer */
		memcpy((void *) nand_buff_addr[bufrdy], p, eccsize);
		memcpy((void *) nand_buff_addr[bufrdy] + eccsize, oob, OOB_FREE_OFFSET);
		p += eccsize;
		oob += eccbytes;
	}

	while(!((nand_readl(IRQSTATUSRAW1)) & nand_buff_enc_mask[bufrdy]));

	for (i = eccsteps; i > 0; i--) {
		/* Buffer management */
		curbuf = bufrdy;
		bufrdy = 1 - bufrdy;

		/* Start the transfer to the device */
		lpc313x_nand_int_clear(~0);
#if !defined(STATUS_POLLING)
		host->intspending = 0;
#endif
		lpc313x_ram_write(curbuf);

		/* Copy next payload and OOB data to the buffer while current
		   buffer is transferring */
		if (i > 1) {

#ifdef USE_DMA
			/* If DMA mapping succesful, use DMA for transfer.
			 * Else use memcpy for transfer
			 * */
			if(use_dma) {
				/* Transfer pay load & OOB using DMA */
				lpc313x_nand_dma_sg_tfr(mtd, chip, bufrdy, p1, oob1, 0);

				/* Update buffer offsets */
				p1 += eccsize;
				oob1 += eccbytes;
			}
			else
#endif
			{
				memcpy((void *) nand_buff_addr[bufrdy], p, eccsize);
				memcpy((void *) nand_buff_addr[bufrdy] + eccsize, oob, OOB_FREE_OFFSET);
				p += eccsize;
				oob += eccbytes;
			}
			while(!((nand_readl(IRQSTATUSRAW1)) & nand_buff_enc_mask[bufrdy]));
		}

#if defined(STATUS_POLLING)
		/* Polling for buffer loaded and decoded */
		while (!((nand_readl(IRQSTATUSRAW1)) & nand_buff_wr_mask[curbuf]));

#else
		/* Interrupt based wait operation */
		lpc313x_wait_irq(host);
#endif
	}

	/* Calculate remaining oob bytes */
	i = mtd->oobsize - (oob - chip->oob_poi);
	if (i)
		chip->write_buf(mtd, oob, i);

#ifdef USE_DMA
	/* Unmap DMA mappings */
	if(use_dma) {
		dma_unmap_single(host->dev, pmapped, (eccsize * eccsteps),
				DMA_TO_DEVICE);
		dma_unmap_single(host->dev, oobmapped, (eccbytes * eccsteps),
				DMA_TO_DEVICE);
	}
#endif

	/* Disable all interrupts */
	lpc313x_nand_int_dis(~0);
}

/*
 * Write the OOB data to the device in the hardware storage format
 */
static int lpc313x_nand_write_oob_syndrome(struct mtd_info *mtd,
				   struct nand_chip *chip, int page)
{
	int chunk = chip->ecc.bytes + chip->ecc.prepad + chip->ecc.postpad;
	int eccsize = chip->ecc.size, length = mtd->oobsize;
	int i, len, pos, status = 0, sndcmd = 0, steps = chip->ecc.steps;
	const uint8_t *bufpoi = chip->oob_poi;

	pos = eccsize;

	chip->cmdfunc(mtd, NAND_CMD_SEQIN, pos, page);
	for (i = 0; i < steps; i++) {
		if (sndcmd) {
			if (mtd->writesize <= 512) {
				uint32_t fill = 0xFFFFFFFF;

				len = eccsize;
				while (len > 0) {
					int num = min_t(int, len, 4);
					chip->write_buf(mtd, (uint8_t *)&fill,
							num);
					len -= num;
				}
			} else {
				pos = eccsize + i * (eccsize + chunk);
				chip->cmdfunc(mtd, NAND_CMD_RNDIN, pos, -1);
			}
		} else {
			sndcmd = 1;
		}

		len = min_t(int, length, chunk);
		chip->write_buf(mtd, bufpoi, len);
		bufpoi += len;
		length -= len;
	}
	if (length > 0)
		chip->write_buf(mtd, bufpoi, length);

	chip->cmdfunc(mtd, NAND_CMD_PAGEPROG, -1, -1);
	status = chip->waitfunc(mtd, chip);

	return status & NAND_STATUS_FAIL ? -EIO : 0;
}

/*
 * Add MTD partitions and a single MTD device
 */
static int lpc313x_nand_add_partition(struct lpc313x_nand_info *host,
		struct lpc313x_nand_mtd *bmtd, struct lpc313x_nand_dev_info *device)
{
	struct mtd_info *mtd = &bmtd->mtd;

#ifdef CONFIG_MTD_PARTITIONS
	struct mtd_partition *partitions = NULL;
	int num_partitions = 0;

#ifdef CONFIG_MTD_CMDLINE_PARTS
	const char *part_probes[] = {"cmdlinepart", NULL};

	/* Check for partitions from the CMDLINE first, these will override the
	   board specific partitions */
	mtd->name = "lpc313x_nand";
	num_partitions = parse_mtd_partitions(mtd, part_probes,
					      &partitions, 0);
#endif

	if ((num_partitions <= 0) && (device->partitions))
	{
		/* No CMDLINE partitions, try board specific partitions */
		partitions = device->partitions;
		num_partitions = device->nr_partitions;
	}

	if ((!partitions) || (num_partitions == 0)) {
		dev_dbg(host->dev, "No parititions defined\n");
		return ENXIO;
	}

	return add_mtd_partitions(mtd, partitions, num_partitions);

#else
	return add_mtd_device(mtd);
#endif
}

/*
 * Init a single instance of an chip
 */
static void lpc313x_nand_init_chip(struct lpc313x_nand_info *host,
				struct lpc313x_nand_mtd *nmtd) {
	struct nand_chip *chip = &nmtd->chip;

	if (host->platform->support_16bit) {
		chip->write_buf = lpc313x_nand_write_buf16;
		chip->read_buf = lpc313x_nand_read_buf16;
	}
	else {
		chip->write_buf = lpc313x_nand_write_buf8;
		chip->read_buf = lpc313x_nand_read_buf8;
	}
	chip->select_chip = lpc313x_nand_select_chip;
	chip->chip_delay = 20;
	chip->priv = nmtd;
	chip->controller = &host->controller;

	chip->IO_ADDR_W = (void *) &NAND_WRITEDATA;
	chip->cmd_ctrl = lpc313x_nand_hwcontrol;
	chip->dev_ready = lpc313x_nand_devready;
	chip->IO_ADDR_R = (void *) &NAND_READDATA;

	nmtd->host = host;
	nmtd->mtd.priv = chip;
	nmtd->mtd.owner = THIS_MODULE;

	chip->ecc.mode = NAND_ECC_HW_SYNDROME;
	chip->ecc.read_page_raw = lpc313x_nand_read_page_syndrome;
	chip->ecc.read_page = lpc313x_nand_read_page_syndrome;
	chip->ecc.write_page = lpc313x_nand_write_page_syndrome;
	chip->ecc.write_oob = lpc313x_nand_write_oob_syndrome;
	chip->ecc.read_oob = lpc313x_nand_read_oob_syndrome;
	chip->ecc.calculate = lpc313x_nand_calculate_ecc;
	chip->ecc.correct   = lpc313x_nand_correct_data;
	chip->ecc.hwctl = lpc313x_nand_enable_hwecc;

	chip->verify_buf = lpc313x_nand_verify_hwecc;
	chip->options |= NAND_USE_FLASH_BBT;
	if (host->platform->support_16bit) {
		chip->options |= NAND_BUSWIDTH_16;
	}

	/* Assume large block FLASH for now, will adjust after detection */
	chip->ecc.layout = &nand_hw_eccoob_64;
}

/*
 * Post-probe chip update, to change any items, such as the
 * layout for large page nand
 */
static void lpc313x_nand_update_chip(struct lpc313x_nand_info *info,
		struct lpc313x_nand_mtd *nmtd) {
	struct nand_chip *chip = &nmtd->chip;

	chip->bbt_td = &lpc313x_bbt_main_descr;
	chip->bbt_md = &lpc313x_bbt_mirror_descr;

	/* Select bad block algorithm and ECC layout based on whether
	   small, large, or hig block FLASH is used */
	if (chip->page_shift <= 10) {
		/* Small block FLASH */
		chip->ecc.layout = &nand_hw_eccoob_16;
		// FIXME unknown if this works
	}
#ifdef HUGE_BLOCK_SUPPORT
	else if (chip->page_shift >= 13) {
		/* Huge block FLASH */
		chip->ecc.layout = &nand_hw_eccoob_128;
		// FIXME bad block descriptors for huge block FLASH not done
	}
#endif
	else {
		/* Large block FLASH */
		chip->ecc.layout = &nand_hw_eccoob_64;
		chip->bbt_td = &lpc313x_bbt_main_descr;
		chip->bbt_md = &lpc313x_bbt_mirror_descr;
		chip->badblock_pattern = &lpc313x_largepage_flashbased;
	}

	/* These sizes remain the same regardless of page/block size */
	chip->ecc.size = 512;
	chip->ecc.bytes = 16;
	chip->ecc.prepad = 0;
}

/*
 * Called by device layer when it finds a device matching
 * one our driver can handled. This code checks to see if
 * it can allocate all necessary resources then calls the
 * nand layer to look for devices.
 */
static int lpc313x_nand_probe(struct platform_device *pdev) {
	struct lpc313x_nand_info *host = NULL;
	struct lpc313x_nand_cfg *plat = pdev->dev.platform_data;
	int irq, scan_res, mtdsize, i, err = 0;

	/* Get required resources */
	irq = platform_get_irq(pdev, 0);
	if ((irq < 0) | (irq >= NR_IRQS))
	{
		return -EBUSY;
	}

	host = kmalloc(sizeof (struct lpc313x_nand_info), GFP_KERNEL);
	if (host == NULL) {
		dev_err(&pdev->dev, "No memory for flash info\n");
		err = -ENOMEM;
		goto exit_error;
	}

	memset(host, 0, sizeof(*host));
	/* Register driver data with platform */
	platform_set_drvdata(pdev, host);

	host->dev = &pdev->dev;
	host->platform = plat;
	host->irq = irq;
	host->current_cs = 0;

	/* Exit if no platform data */
	if (plat == NULL) {
		dev_err(&pdev->dev, "No memory for flash info\n");
		goto exit_error;
	}

	/* Initialize lock and queue used by higher level NAND driver */
	spin_lock_init(&host->controller.lock);
	init_waitqueue_head(&host->controller.wq);

	/* Enable clocks for NAND Controller */
	lpc313x_nand_clocks_disen(1);

	/* Reset NAND controller */
	cgu_soft_reset_module(NANDFLASH_CTRL_NAND_RESET_N_SOFT);
	cgu_soft_reset_module(NANDFLASH_CTRL_ECC_RESET_N_SOFT);

	/* Needed for LPC315x series only */
	cgu_soft_reset_module(NANDFLASH_CTRL_AES_RESET_N_SOFT);

	/* check NAND mux signals */
	sys_writel(MUX_NAND_MCI, 0);

	/* Setup NAND configuration */
	if (plat->support_16bit) {
		/* 16-bit mode */
		host->nandconfig = NAND_NANDCONFIG_DC | NAND_NANDCONFIG_ECGC |
			NAND_NANDCONFIG_EC | NAND_NANDCONFIG_WD;
	}
	else {
		/* 8-bit mode */
		host->nandconfig = NAND_NANDCONFIG_DC | NAND_NANDCONFIG_ECGC |
			NAND_NANDCONFIG_EC;
	}

	/* Initialize the hardware */
	err = lpc313x_nand_inithw(host);
	if (err != 0)
		goto exit_error;

	/* Attach interrupt handler */
	err = request_irq(host->irq, lpc313x_nandc_irq,
		IRQF_DISABLED, "nandirq", host);
	if (err)
	{
		goto exit_error;
	}

	/* IRQ event queue */
	init_waitqueue_head(&host->irq_waitq);

	/* Allocate space for the MTD data */
	mtdsize = sizeof(struct lpc313x_nand_mtd) * host->platform->nr_devices;
	host->mtds = kmalloc(mtdsize, GFP_KERNEL);
	if (host->mtds == NULL) {
		dev_err(&pdev->dev, "Failed to allocate mtd storage\n");
		err = -ENOMEM;
		goto exit_error2;
	}
	memset(host->mtds, 0, mtdsize);

#ifdef USE_DMA
	/* Allocate sg channel for DMA transfers */
	host->dma_chn = dma_request_sg_channel("NAND", lpc313x_nand_dma_irq,
			host, 1);
	if(host->dma_chn < 0) {
		dev_err(&pdev->dev, "Failed to allocate DMA SG channel\n");
		err = host->dma_chn;
		goto exit_error3;
	}

	/* Allocate memory for SG Table */
	host->sg_cpu = dma_alloc_coherent(&pdev->dev,
			NAND_DMA_MAX_DESC * sizeof(dma_sg_ll_t), &host->sg_dma, GFP_KERNEL);
	if (host->sg_cpu == NULL) {
		dev_err(&pdev->dev, "could not alloc dma memory\n");
		goto exit_error4;
	}

	/* Initialise DMA wait queue */
	init_waitqueue_head(&host->dma_waitq);
#endif

	/* Add MTDs and partitions */
	for (i = 0; i < host->platform->nr_devices; i++) {
		dev_dbg(&pdev->dev, "Initializing NAND device on CS%d (%s)\n",
			i, host->platform->devices[i].name);

		/* Populdate device callbacks used by MTD driver */
		lpc313x_nand_init_chip(host, &host->mtds[i]);

		/* Scan NAND flash device */
		scan_res = nand_scan_ident(&host->mtds[i].mtd, 1);

		/* Continue if a device is found */
		if (scan_res == 0) {
			/* Update callbacks based on NAND sizing data */
			lpc313x_nand_update_chip(host, &host->mtds[i]);

			/* Post architecture MTD init */
			nand_scan_tail(&host->mtds[i].mtd);

			/* Add partitions and MTD device */
			if (lpc313x_nand_add_partition(host, &host->mtds[i],
				(plat->devices + i)) < 0) {
				nand_release(&host->mtds[i].mtd);
			}
		}
		else {
			dev_dbg(&pdev->dev, "No device detected on CS%d (%s)\n",
				i, host->platform->devices[i].name);
		}
	}

	return 0;

#ifdef USE_DMA
exit_error4:
	/* Release sg channel */
	dma_release_sg_channel(host->dma_chn);

exit_error3:
	/* Release memory */
	if(host->mtds != NULL)
		kfree(host->mtds);
#endif

exit_error2:
	/* Release IRQ */
	free_irq(host->irq, pdev);

exit_error:
	if (host != NULL)
		kfree(host);

	/* Disable clocks for NAND Controller */
	lpc313x_nand_clocks_disen(0);

	return err;
}

/*
 * Device removal
 */
static int lpc313x_nand_remove(struct platform_device *pdev) {
	struct lpc313x_nand_info *host = platform_get_drvdata(pdev);
	int i;

	platform_set_drvdata(pdev, NULL);

	if (host == NULL)
		return 0;

	/* Release all the MTDs */
	for (i = 0; i < host->platform->nr_devices; i++) {
		dev_dbg(&pdev->dev, "Releasing mtd device %d (%s)\n", i,
			host->platform->devices[i].name);
		nand_release(&host->mtds[i].mtd);
	}

	/* Disable clocks for NAND Controller */
	lpc313x_nand_clocks_disen(1);

#ifdef USE_DMA
	/* Release memory allocated for SG table */
	dma_free_coherent(host->dev, NAND_DMA_MAX_DESC * sizeof(dma_sg_ll_t),
			host->sg_cpu, host->sg_dma);

	/* Release sg channel */
	dma_release_sg_channel(host->dma_chn);
#endif

	/* Release IRQ */
	free_irq(host->irq, pdev);

	kfree(host->mtds);
	kfree(host);

	return 0;
}

#if defined(CONFIG_PM)
static int lpc313x_nand_resume(struct platform_device *pdev)
{
	/* Enables clocks for NAND Controller */
	lpc313x_nand_clocks_disen(1);

	return 0;
}

static int lpc313x_nand_suspend(struct platform_device *pdev, pm_message_t pm)
{
	/* Disable clocks for NAND Controller */
	lpc313x_nand_clocks_disen(0);

	return 0;
}

#else
#define lpc313x_nand_resume NULL
#define lpc313x_nand_suspend NULL
#endif

static struct platform_driver lpc313x_nand_driver = {
	.probe		= lpc313x_nand_probe,
	.remove		= lpc313x_nand_remove,
	.resume		= lpc313x_nand_resume,
	.suspend	= lpc313x_nand_suspend,
	.driver = {
		.name = "lpc313x_nand",
		.owner = THIS_MODULE,
	},
};

static int __init lpc313x_nand_init(void)
{
	return platform_driver_register(&lpc313x_nand_driver);
}

static void __exit lpc313x_nand_exit(void)
{
	platform_driver_unregister(&lpc313x_nand_driver);
}

module_init( lpc313x_nand_init);
module_exit( lpc313x_nand_exit);

MODULE_DESCRIPTION("LPC313x NAND Controller driver");
MODULE_AUTHOR("NXP Semiconductor VietNam");
MODULE_LICENSE("GPL v2");

