/*
 * drivers/spi/spi_lpc313x.c
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
 * LPC313X SPI notes
 *
 * The LPC313X SPI Linux driver supports many chip selects using GPO based CS
 * (hardware chip selects are not supported due to timing constraints), clock
 * speeds up to 45MBps, data widths from 4 to 16 bits, DMA support, and full
 * power management.
 */

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>

#include <mach/registers.h>
#include <mach/dma.h>
#include <mach/board.h>

/* Register access macros */
#define spi_readl(reg) __raw_readl(&SPI_##reg)
#define spi_writel(reg,value) __raw_writel((value),&SPI_##reg)

struct lpc313xspi
{
	spinlock_t lock;
	struct platform_device *pdev;
	struct workqueue_struct	*workqueue;
	struct work_struct work;
	struct list_head queue;
	wait_queue_head_t waitq;
	struct spi_master *master;
	int irq;
	int id;
	u32 spi_base_clock;
	struct lpc313x_spi_cfg *psppcfg;
	u32 current_speed_hz [3]; /* Per CS */
	u8 current_bits_wd [3]; /* Per CS */

	/* DMA allocated regions */
	u32 dma_base_v;
	dma_addr_t dma_base_p;

	/* DMA TX and RX physical and mapped spaces */
	u32 dma_tx_base_v, dma_rx_base_v, dma_tx_base_p, dma_rx_base_p;

	/* Allocated DMA channels */
	int tx_dma_ch, rx_dma_ch;

	/* DMA event flah */
	volatile int rxdmaevent;
};

/*
 * Enable or disable the SPI clocks
 */
static void lpc313x_spi_clks_disen(struct lpc313xspi *spidat, int enable)
{
	int en = (enable != 0);

	cgu_clk_en_dis(CGU_SB_SPI_PCLK_ID, en);
	cgu_clk_en_dis(CGU_SB_SPI_PCLK_GATED_ID, en);
	cgu_clk_en_dis(CGU_SB_SPI_CLK_ID, en);
	cgu_clk_en_dis(CGU_SB_SPI_CLK_GATED_ID, en);
}

/*
 * Flush the TX and RX FIFOs
 */
static void lpc313x_fifo_flush(struct lpc313xspi *spidat)
{
	volatile u32 tmp;

	/* Clear TX FIFO first */
	spi_writel(TXF_FLUSH_REG, SPI_TXFF_FLUSH);

	/* Clear RX FIFO */
	while (!(spi_readl(STS_REG) & SPI_ST_RX_EMPTY))
	{
		tmp = spi_readl(FIFO_DATA_REG);
	}
}

/*
 * Clear a latched SPI interrupt
 */
static inline void lpc313x_int_clr(struct lpc313xspi *spidat, u32 ints)
{
	spi_writel(INT_CLRS_REG, ints);
}

/*
 * Disable a SPI interrupt
 */
static inline void lpc313x_int_dis(struct lpc313xspi *spidat, u32 ints)
{
	spi_writel(INT_CLRE_REG, ints);
}

/*
 * Enable a SPI interrupt
 */
static inline void lpc313x_int_en(struct lpc313xspi *spidat, u32 ints)
{
	spi_writel(INT_SETE_REG, ints);
}

/*
 * Set a SPI chip select state
 */
static inline void spi_force_cs(struct lpc313xspi *spidat, u8 cs, uint cs_state)
{
	spidat->psppcfg->spics_cfg[cs].spi_cs_set((int) cs, (int) cs_state);
}

/*
 * Set data width for the SPI chip select
 */
static void lpc313x_set_cs_data_bits(struct lpc313xspi *spidat, u8 cs, u8 data_width)
{
	if (spidat->current_bits_wd[cs] != data_width)
	{
		u32 tmp = spi_readl(SLV_SET2_REG(0));
		tmp &= ~SPI_SLV2_WD_SZ(0x1F);
		tmp |= SPI_SLV2_WD_SZ((u32) (data_width - 1));
		spi_writel(SLV_SET2_REG(0), tmp);

		spidat->current_bits_wd[cs] = data_width;
	}
}

/*
 * Set clock rate and delays for the SPI chip select
 */
static void lpc313x_set_cs_clock(struct lpc313xspi *spidat, u8 cs, u32 clockrate)
{
	u32 reg, div, ps, div1;

	if (clockrate != spidat->current_speed_hz[cs])
	{
		reg = spi_readl(SLV_SET1_REG(0));
		reg &= ~0xFFFF;

		div = (spidat->spi_base_clock + clockrate / 2) / clockrate;
		if (div > SPI_MAX_DIVIDER)
			div = SPI_MAX_DIVIDER;
		if (div < SPI_MIN_DIVIDER)
			div = SPI_MIN_DIVIDER;

		ps = (((div - 1) / 512) + 1) * 2;
		div1 = (((div + ps / 2) / ps) - 1);

		spi_writel(SLV_SET1_REG(0),
			(reg | SPI_SLV1_CLK_PS(ps) | SPI_SLV1_CLK_DIV1(div1)));

		spidat->current_speed_hz[cs] = clockrate;
	}
}

/*
 * Setup the initial state of the SPI interface
 */
static void lpc313x_spi_prep(struct lpc313xspi *spidat)
{
	u32 tmp;

	/* Reset SPI block */
	spi_writel(CONFIG_REG, SPI_CFG_SW_RESET);

	/* Clear FIFOs */
	lpc313x_fifo_flush(spidat);

	/* Clear latched interrupts */
	lpc313x_int_dis(spidat, SPI_ALL_INTS);
	lpc313x_int_clr(spidat, SPI_ALL_INTS);

	/* Setup master mode, normal transmit mode, and interslave delay */
	spi_writel(CONFIG_REG, SPI_CFG_INTER_DLY(1));

	/* Make sure all 3 chip selects are initially disabled */
	spi_writel(SLV_ENAB_REG, 0);
	spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) | SPI_CFG_UPDATE_EN));

	/* FIFO trip points at 50% */
	spi_writel(INT_TRSH_REG, (SPI_INT_TSHLD_TX(0x20) | SPI_INT_TSHLD_RX(0x20)));

	/* Only chip select 0 is used in this driver. However, the timings for this
	   chip select effect transfer speed and need to be adjusted for each GPO
	   based chip select. Use a default value to start with for now. */
	/* Inter-transfer delay is 0 (not used) */
	tmp = spi_readl(SLV_SET1_REG(0));
	tmp &= ~SPI_SLV1_INTER_TX_DLY(0xFF);
	spi_writel(SLV_SET1_REG(0), (tmp | SPI_SLV1_INTER_TX_DLY(0)));

	/* Configure enabled chip select slave setting 2 */
	tmp = SPI_SLV2_PPCS_DLY(0) | SPI_SLV2_CS_HIGH | SPI_SLV2_SPO;
	spi_writel(SLV_SET2_REG(0), tmp);

	/* Use a default of 8 data bits and a 100K clock for now */
	lpc313x_set_cs_data_bits(spidat, 0, 8);
	lpc313x_set_cs_clock(spidat, 0, 100000);

	/* We'll always use CS0 for this driver. Since the chip select is generated
	   by a GPO, it doesn't matter which one we use */
	spi_writel(SLV_ENAB_REG, SPI_SLV_EN(0));
	spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) | SPI_CFG_UPDATE_EN));

	/* Controller stays disabled until a transfer occurs */
}

/*
 * Setup a SPI transfer
 */
static int lpc313x_spi_setup(struct spi_device *spi)
{
	unsigned int bits = spi->bits_per_word;

	/* There really isn't anuthing to do in this function, so verify the
	   parameters are correct for the transfer */
	if (spi->chip_select > spi->master->num_chipselect)
	{
		dev_dbg(&spi->dev,
			"setup: invalid chipselect %u (%u defined)\n",
			spi->chip_select, spi->master->num_chipselect);
		return -EINVAL;
	}

	if (bits == 0)
	{
		bits = 8;
	}
	if ((bits < 4) || (bits > 16))
	{
		dev_dbg(&spi->dev,
			"setup: invalid bits_per_word %u (8 to 16)\n", bits);
		return -EINVAL;
	}

	return 0;
}

/*
 * Handle the SPI interrupt
 */
static irqreturn_t lpc313x_spi_irq(int irq, void *dev_id)
{
	struct lpc313xspi *spidat = dev_id;

	/* Disable interrupts for now, do not clear the interrupt states */
	lpc313x_int_dis(spidat, SPI_ALL_INTS);
	spidat->rxdmaevent = 1;

	wake_up(&spidat->waitq);

	return IRQ_HANDLED;
}

/*
 * SPI DMA TX callback
 */
static void lpc313x_dma_tx_spi_irq(int ch, dma_irq_type_t dtype, void *handle)
{
	/* Nothing really needs to be done with the DMA TX interrupt, all the work
	   is done with RX, so just return and let the DMA handler clear it. The
	   SPI will stall the clock if the TX FIFO becomes empty so there is no
	   chance of some type of underflow. */
}

/*
 * SPI DMA RX callback
 */
static void lpc313x_dma_rx_spi_irq(int ch, dma_irq_type_t dtype, void *handle)
{
	struct lpc313xspi *spidat = (struct lpc313xspi *) handle;

	if (dtype == DMA_IRQ_FINISHED)
	{
		/* Disable interrupts for now */
		dma_set_irq_mask(spidat->rx_dma_ch, 1, 1);

		/* Flag event and wakeup */
		spidat->rxdmaevent = 1;
		wake_up(&spidat->waitq);
	}
	else if (dtype == DMA_IRQS_ABORT)
	{
		/* DMA data abort - this is global for the entire DMA
		   peripheral. Do nothing, as this might not be a
		   error for this channel. */
	}
}

/*
 * Handle a DMA transfer
 */
static int lpc313x_spi_dma_transfer(struct lpc313xspi *spidat, struct spi_transfer *t,
					u8 bits_per_word, int dmamapped)
{
	dma_setup_t dmarx, dmatx;
	int status = 0;
	u32 src, dest, srcmapped = 0, destmapped = 0;
	struct device *dev = &spidat->pdev->dev;

	/* Set the FIFO trip level to the transfer size */
	spi_writel(INT_TRSH_REG, (SPI_INT_TSHLD_TX(16) |
		SPI_INT_TSHLD_RX(1)));
	spi_writel(DMA_SET_REG, (SPI_DMA_TX_EN | SPI_DMA_RX_EN));
	lpc313x_int_dis(spidat, SPI_ALL_INTS);
	lpc313x_int_en(spidat, SPI_OVR_INT);

	/* Setup transfer */
	if (bits_per_word > 8)
	{
		dmarx.cfg = DMA_CFG_TX_HWORD | DMA_CFG_RD_SLV_NR(DMA_SLV_SPI_RX) |
			DMA_CFG_WR_SLV_NR(0);
		dmatx.cfg = DMA_CFG_TX_HWORD | DMA_CFG_RD_SLV_NR(0) |
			DMA_CFG_WR_SLV_NR(DMA_SLV_SPI_TX);
	}
	else
	{
		dmarx.cfg = DMA_CFG_TX_BYTE | DMA_CFG_RD_SLV_NR(DMA_SLV_SPI_RX) |
			DMA_CFG_WR_SLV_NR(0);
		dmatx.cfg = DMA_CFG_TX_BYTE | DMA_CFG_RD_SLV_NR(0) |
			DMA_CFG_WR_SLV_NR(DMA_SLV_SPI_TX);
	}

	/* Determine the DMA source and destination addresses. If DMA buffers weren't
	   passed to this handler, they need to be mapped */
	if (dmamapped)
	{
		src = t->tx_dma;
		dest = t->rx_dma;
		if ((src == 0) && (dest == 0))
		{
			/* DMA mapped flag set, but not mapped */
			status = -ENOMEM;
			goto exit;
		}

		/* At least one of the DMA buffers are already mapped */
		if (src == 0)
		{
			/* Use temporary buffer for TX */
			src = spidat->dma_tx_base_p;
		}
		else
		{
			dest = spidat->dma_rx_base_p;
		}
	}
	else
	{
		/* Does TX buffer need to be DMA mapped */
		if (t->tx_buf == NULL)
		{
			/* Use the temporary buffer for this transfer */
			src = spidat->dma_tx_base_p;
		}
		else
		{
			/* Map DMA buffer */
			src = (u32) dma_map_single(dev, (void *) t->tx_buf,
				t->len, DMA_TO_DEVICE);
			if (dma_mapping_error(dev, src))
			{
				status = -ENOMEM;
				goto exit;
			}

			srcmapped = src;
		}

		/* Does RX buffer need to be DMA mapped */
		if (t->rx_buf == NULL)
		{
			/* Use the temporary buffer for this transfer */
			dest = spidat->dma_rx_base_p;
		}
		else
		{
			/* Map DMA buffer */
			dest = (u32) dma_map_single(dev, (void *) t->rx_buf,
				t->len, DMA_FROM_DEVICE);
			if (dma_mapping_error(dev, dest))
			{
				status = -ENOMEM;
				goto exit;
			}

			destmapped = dest;
		}
	}

	/* Setup transfer data for DMA */
	dmarx.trans_length = (t->len - 1);
	dmarx.src_address = (SPI_PHYS + 0x0C);
	dmarx.dest_address = dest;
	dmatx.trans_length = (t->len - 1);
	dmatx.src_address = src;
	dmatx.dest_address = (SPI_PHYS + 0x0C);

	/* Setup the channels */
	dma_prog_channel(spidat->rx_dma_ch, &dmarx);
	dma_prog_channel(spidat->tx_dma_ch, &dmatx);

	/* Make sure the completion interrupt is enabled for RX, TX disabled */
	dma_set_irq_mask(spidat->rx_dma_ch, 1, 0);
	dma_set_irq_mask(spidat->tx_dma_ch, 1, 1);

	/* Start the transfer */
	spidat->rxdmaevent = 0;
	dma_start_channel(spidat->rx_dma_ch);
	dma_start_channel(spidat->tx_dma_ch);

	/* Wait for DMA to complete */
	wait_event_interruptible(spidat->waitq, spidat->rxdmaevent);

exit:
	dma_stop_channel(spidat->tx_dma_ch);
	dma_stop_channel(spidat->rx_dma_ch);

	/* Unmap buffers */
	if (srcmapped != 0)
	{
		dma_unmap_single(dev, srcmapped, t->len, DMA_TO_DEVICE);
	}
	if (destmapped != 0)
	{
		dma_unmap_single(dev, destmapped, t->len, DMA_FROM_DEVICE);
	}

	return status;
}

/*
 * The bulk of the transfer work is done in this function. Based on the transfer
 * size, either FIFO (PIO) or DMA mode may be used.
 */
static void lpc313x_work_one(struct lpc313xspi *spidat, struct spi_message *m)
{
	struct spi_device *spi = m->spi;
	struct spi_transfer *t;
	unsigned int wsize, cs_change = 1;
	int status = 0;
	unsigned long flags;
	u32 tmp;

	/* Enable SPI clock and interrupts */
	spin_lock_irqsave(&spidat->lock, flags);
	lpc313x_spi_clks_disen(spidat, 1);
	enable_irq(spidat->irq);

	/* Make sure FIFO is flushed and clear any pending interrupts */
	lpc313x_fifo_flush(spidat);
	/* lpc313x_int_clr(spidat, SPI_ALL_INTS); ***fix from JPP*** */

	/* Process each transfer in the message */
	list_for_each_entry (t, &m->transfers, transfer_list) {
		const void *txbuf = t->tx_buf;
		void *rxbuf = t->rx_buf;
		u32 data;
		unsigned int rlen, tlen = t->len;
		u32 speed_hz = t->speed_hz ? : spi->max_speed_hz;
		u8 bits_per_word = t->bits_per_word ? : spi->bits_per_word;

		/* Bits per word, data transfer size, and transfer counter */
		bits_per_word = bits_per_word ? : 8;
		wsize = bits_per_word >> 3;
		rlen = tlen;

		/* Setup the appropriate chip select */
		lpc313x_set_cs_clock(spidat, spi->chip_select, speed_hz);
		lpc313x_set_cs_data_bits(spidat, spi->chip_select, bits_per_word);
 
		/* Setup timing and levels before initial chip select */
		tmp = spi_readl(SLV_SET2_REG(0)) & ~(SPI_SLV2_SPO | SPI_SLV2_SPH);
		if (spidat->psppcfg->spics_cfg[spi->chip_select].spi_spo != 0)
		{
			/* Clock high between transfers */
			tmp |= SPI_SLV2_SPO;
		}
		if (spidat->psppcfg->spics_cfg[spi->chip_select].spi_sph != 0)
		{
			/* Data captured on 2nd clock edge */
			tmp |= SPI_SLV2_SPH;
		}
		spi_writel(SLV_SET2_REG(0), tmp);

		lpc313x_int_clr(spidat, SPI_ALL_INTS);  /****fix from JPP*** */

		/* Make sure FIFO is flushed, clear pending interrupts, DMA
		   initially disabled, and then enable SPI interface */
		spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) | SPI_CFG_ENABLE));

		/* Assert selected chip select */
		if (cs_change)
		{
			/* Force CS assertion */
			spi_force_cs(spidat, spi->chip_select, 0);
		}
		cs_change = t->cs_change;

		/* The driver will pick the best transfer method based on the
		   current transfer size. For sizes smaller than the FIFO depth,
		   the FIFOs are used without DMA support. For transfers larger
		   than the FIFO depth, DMA is used. When dealing with fast SPI
		   clock rates and transfers larger than the FIFO depth, DMA is
		   required. The higher level SPI functions will limit transfer
		   sizes to 4Kbytes. */
		/***Fix from JPP ******/
		/*if (tlen < (SPI_FIFO_DEPTH * wsize)) {*/
		if (0) { //***MOD: Non-DMA SPI code broken -> possibly off-by-one bit error.
			if ((txbuf == NULL) && (rxbuf == NULL))
			{
				/* A PIO mode DMA transfer requires mapped
				   memory. Something is wrong. DMA transfer? */
				dev_err(&spidat->pdev->dev, "No mapped buffers\n");
				status = -EIO;
				goto exit;
			}

			/* Set the FIFO trip level to the transfer size */
			spi_writel(INT_TRSH_REG, (SPI_INT_TSHLD_TX(0) |
				SPI_INT_TSHLD_RX(tlen - 1)));
			spi_writel(DMA_SET_REG, 0);

			/* Fill TX FIFO */
			while ((!(spi_readl(STS_REG) & SPI_ST_TX_FF)) && (tlen > 0))
			{
				/* Fill FIFO */
				if (txbuf)
			{
					data = (wsize == 1)
						? *(const u8 *) txbuf
						: *(const u16 *) txbuf;
					spi_writel(FIFO_DATA_REG, data);
					txbuf += wsize;
				}
				else
				{
					/* Send dummy data */
					spi_writel(FIFO_DATA_REG, 0xFFFF);
				}

				tlen--;
			}

			/* Wait for data */
			lpc313x_int_en(spidat, (SPI_RX_INT | SPI_TO_INT | SPI_OVR_INT));
			spin_unlock_irqrestore(&spidat->lock, flags);
			wait_event_interruptible(spidat->waitq,
				(spi_readl(INT_STS_REG) & (SPI_RX_INT | SPI_OVR_INT)));
			spin_lock_irqsave(&spidat->lock, flags);

			/* Has an overflow occurred? */
			if (unlikely(spi_readl(INT_STS_REG) & SPI_OVR_INT))
			{
				/* RX FIFO overflow */
				dev_err(&spidat->pdev->dev, "RX FIFO overflow.\n");
				status = -EIO;
				goto exit;
			}

			/* Is there any data to read? */
			while (!(spi_readl(STS_REG) & SPI_ST_RX_EMPTY))
			{
				data = spi_readl(FIFO_DATA_REG);
				/* The data can be tossed if there is no RX buffer */
				if (rxbuf)
				{
					if (wsize == 1)
					{
						*(u8 *)rxbuf = (u8) data;
					}
					else
					{
						*(u16 *)rxbuf = (u16) data;
					}

					rxbuf += wsize;
				}

				rlen--;
			}
		}
		else {
			/* DMA will be used for the transfer */
			spin_unlock_irqrestore(&spidat->lock, flags);
			status = lpc313x_spi_dma_transfer(spidat, t, bits_per_word,
				m->is_dma_mapped);
			spin_lock_irqsave(&spidat->lock, flags);
			if (status < 0)
				goto exit;
		}

		m->actual_length += t->len;
		if (t->delay_usecs)
		{
			udelay(t->delay_usecs);
		}

		if (!cs_change)
			continue;

		if (t->transfer_list.next == &m->transfers)
			break;
	}

exit:
	if (!(status == 0 && cs_change))
	{
		spi_force_cs(spidat, spi->chip_select, 1);
	}

	/* Disable SPI, stop SPI clock to save power */
	spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) & ~SPI_CFG_ENABLE));
	disable_irq(spidat->irq);
	lpc313x_spi_clks_disen(spidat, 0);

	spin_unlock_irqrestore(&spidat->lock, flags);
	m->status = status;
	m->complete(m->context);
}

/*
 * Work queue function
 */
static void lpc313x_work(struct work_struct *work)
{
	struct lpc313xspi *spidat = container_of(work, struct lpc313xspi, work);
	unsigned long flags;

	spin_lock_irqsave(&spidat->lock, flags);

	while (!list_empty(&spidat->queue))
	{
		struct spi_message *m;

		m = container_of(spidat->queue.next, struct spi_message, queue);
		list_del_init(&m->queue);

		spin_unlock_irqrestore(&spidat->lock, flags);
		lpc313x_work_one(spidat, m);
		spin_lock_irqsave(&spidat->lock, flags);
	}

	spin_unlock_irqrestore(&spidat->lock, flags);
}

/*
 * Kick off a SPI transfer
 */
static int lpc313x_spi_transfer(struct spi_device *spi, struct spi_message *m)
{
	struct spi_master *master = spi->master;
	struct lpc313xspi *spidat = spi_master_get_devdata(master);
	//struct device *controller = spi->master->dev.parent;
	struct spi_transfer *t;
	unsigned long flags;

	m->actual_length = 0;

	/* check each transfer's parameters */
	list_for_each_entry (t, &m->transfers, transfer_list)
	{
		u8 bits_per_word = t->bits_per_word ? : spi->bits_per_word;

		bits_per_word = bits_per_word ? : 8;
		if ((!t->tx_buf) && (!t->rx_buf) && (t->len))
		{
			return -EINVAL;
		}
		if ((bits_per_word < 4) || (bits_per_word > 16))
		{
			return -EINVAL;
		}

		/*dev_dbg(controller,
			"  xfer %p: len %u tx %p/%08x rx %p/%08x DMAmapped=%d\n",
			t, t->len, t->tx_buf, t->tx_dma,
			t->rx_buf, t->rx_dma, m->is_dma_mapped); */   //***MOD-
	}

	spin_lock_irqsave(&spidat->lock, flags);
	list_add_tail(&m->queue, &spidat->queue);
	queue_work(spidat->workqueue, &spidat->work);
	spin_unlock_irqrestore(&spidat->lock, flags);

	return 0;
}

/*
 * SPI driver probe
 */
static int __init lpc313x_spi_probe(struct platform_device *pdev)
{
	struct spi_master *master;
	struct lpc313xspi *spidat;
	int ret, irq, i;
	dma_addr_t dma_handle;

	/* Get required resources */
	irq = platform_get_irq(pdev, 0);
	if ((irq < 0) | (irq >= NR_IRQS))
	{
		return -EBUSY;
	}

	master = spi_alloc_master(&pdev->dev, sizeof(struct lpc313xspi));
	if (!master)
	{
		return -ENODEV;
	}
	spidat = spi_master_get_devdata(master);
	platform_set_drvdata(pdev, master);

	/* Is a board specific configuration available? */
	spidat->psppcfg = (struct lpc313x_spi_cfg *) pdev->dev.platform_data;
	if (spidat->psppcfg == NULL)
	{
		/* No platform data, exit */
		ret = -ENODEV;
		goto errout;
	}
	if (spidat->psppcfg->num_cs < 1)
	{
		/* No chip selects supported in board structure, exit */
		ret = -ENODEV;
		goto errout;
	}
	for (i = 0; i < spidat->psppcfg->num_cs; i++)
	{
		if (spidat->psppcfg->spics_cfg[i].spi_cs_set == NULL)
		{
			/* Missing hardware CS control callback, exit */
			ret = -ENODEV;
			goto errout;
		}
	}

	/* Save ID for this device */
	spidat->pdev = pdev;
	spidat->id = pdev->id;
	spidat->irq = irq;
	spin_lock_init(&spidat->lock);

	INIT_WORK(&spidat->work, lpc313x_work);
	INIT_LIST_HEAD(&spidat->queue);
	init_waitqueue_head(&spidat->waitq);
	spidat->workqueue = create_singlethread_workqueue(dev_name(master->dev.parent));	//***MOD:Fix from JPP to compile to latest versions of Linux
	if (!spidat->workqueue)
	{
		ret = -ENOMEM;
		goto errout;
	}

	/* Enable clocks */
	lpc313x_spi_clks_disen(spidat, 1);
	cgu_soft_reset_module(SPI_PNRES_APB_SOFT);
	cgu_soft_reset_module(SPI_PNRES_IP_SOFT);

	ret = request_irq(spidat->irq, lpc313x_spi_irq,
		IRQF_DISABLED, "spiirq", spidat);
	if (ret)
	{
		ret = -EBUSY;
		goto errout2;
	}
	disable_irq(spidat->irq);

	master->bus_num = spidat->id;
	master->setup = lpc313x_spi_setup;
	master->transfer = lpc313x_spi_transfer;
	master->num_chipselect = spidat->psppcfg->num_cs;

	/* Setup several work DMA buffers for dummy TX and RX data. These buffers just
	   hold the temporary TX or RX data for the unused half of the transfer and have
	   a size of 4K (the maximum size of a transfer) */
	spidat->dma_base_v = (u32) dma_alloc_coherent(&pdev->dev, (4096 << 1),
		&dma_handle, GFP_KERNEL);
	if (spidat->dma_base_v == (u32) NULL)
	{
		dev_err(&pdev->dev, "error getting DMA region.\n");
		ret = -ENOMEM;
		goto errout3;
	}
	spidat->dma_base_p = dma_handle;;

	spidat->dma_tx_base_p = (u32) spidat->dma_base_p;
	spidat->dma_tx_base_v = spidat->dma_base_v;
	spidat->dma_rx_base_p = (u32) spidat->dma_base_p + 4096;
	spidat->dma_rx_base_v = spidat->dma_base_v + 4096;

	/* Fill dummy TX buffer with 0 */
	memset((void *) spidat->dma_tx_base_v, 0, 4096);

	/* Initial setup of SPI */
	spidat->spi_base_clock = cgu_get_clk_freq(CGU_SB_SPI_CLK_ID);
	lpc313x_spi_prep(spidat);

	/* Keep SPI clocks off until a transfer is performed to save power */
	lpc313x_spi_clks_disen(spidat, 0);

	/* Request RX and TX DMA channels */
	spidat->tx_dma_ch = spidat->rx_dma_ch = -1;
	spidat->tx_dma_ch = dma_request_channel("spi_tx", lpc313x_dma_tx_spi_irq, spidat);
	if (spidat->tx_dma_ch < 0)
	{
		dev_err(&pdev->dev, "error getting TX DMA channel.\n");
		ret = -EBUSY;
		goto errout4;
	}
	spidat->rx_dma_ch = dma_request_channel("spi_rx", lpc313x_dma_rx_spi_irq, spidat);
	if (spidat->rx_dma_ch < 0)
	{
		dev_err(&pdev->dev, "error getting RX DMA channel.\n");
		ret = -EBUSY;
		goto errout4;
	}

	ret = spi_register_master(master);
	if (ret)
	{
		goto errout4;
	}

	dev_info(&pdev->dev, "LPC313x SPI driver\n");

	return 0;

errout4:
	if (spidat->tx_dma_ch != -1)
		dma_release_channel(spidat->tx_dma_ch);
	if (spidat->rx_dma_ch != -1)
		dma_release_channel(spidat->rx_dma_ch);
	dma_free_coherent(&pdev->dev, (4096 << 1), (void *) spidat->dma_base_v,
		spidat->dma_base_p);
errout3:
	free_irq(spidat->irq, pdev);
errout2:
	lpc313x_spi_clks_disen(spidat, 0);
	destroy_workqueue(spidat->workqueue);
errout:
	platform_set_drvdata(pdev, NULL);
	spi_master_put(master);

	return ret;
}

/*
 * SPI driver removal
 */
static int __devexit lpc313x_spi_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct lpc313xspi *spidat = spi_master_get_devdata(master);

	/* Disable SPI interface */
	spi_writel(CONFIG_REG, (spi_readl(CONFIG_REG) & ~SPI_CFG_ENABLE));
	lpc313x_spi_clks_disen(spidat, 0);

	spi_unregister_master(master);
	platform_set_drvdata(pdev, NULL);

	if (spidat->tx_dma_ch != -1)
		dma_release_channel(spidat->tx_dma_ch);
	if (spidat->rx_dma_ch != -1)
		dma_release_channel(spidat->rx_dma_ch);

	dma_free_coherent(&pdev->dev, (4096 << 1), (void *) spidat->dma_base_v,
		spidat->dma_base_p);

	/* Free resources */
	free_irq(spidat->irq, pdev);

	destroy_workqueue(spidat->workqueue);
	spi_master_put(master);

	return 0;
}

/**
 * Suspend SPI by switching off the IP clocks
 **/
static int lpc313x_spi_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifdef CONFIG_PM
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct lpc313xspi *spidat = spi_master_get_devdata(master);

	/* Check if SPI is idle before we pull off the clock */
	if (unlikely(!list_empty(&spidat->queue)))
		return 0;

	/* Pull the clocks off */
	lpc313x_spi_clks_disen(spidat, 0);
#endif
	return 0;
}

/**
 * Resume SPI by switching on the IP clocks
 **/
static int lpc313x_spi_resume(struct platform_device *pdev)
{
#ifdef CONFIG_PM
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct lpc313xspi *spidat = spi_master_get_devdata(master);

	/* Switch on the clocks */
	lpc313x_spi_clks_disen(spidat, 1);
#endif
	return 0;
}

static struct platform_driver lpc313x_spi_driver = {
	.probe		= lpc313x_spi_probe,
	.remove		= __devexit_p(lpc313x_spi_remove),
	.suspend    = lpc313x_spi_suspend,
	.resume     = lpc313x_spi_resume,
	.driver		= {
		.name	= "spi_lpc313x",
		.owner	= THIS_MODULE,
	},
};

static int __init lpc313x_spi_init(void)
{
	return platform_driver_register(&lpc313x_spi_driver);
}

static void __exit lpc313x_spi_exit(void)
{
	platform_driver_unregister(&lpc313x_spi_driver);
}

module_init(lpc313x_spi_init);
module_exit(lpc313x_spi_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com");
MODULE_DESCRIPTION("LPC313X SPI Driver");
MODULE_LICENSE("GPL");
