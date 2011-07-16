/*  arch/arm/mach-lpc313x/dma.c
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  DMA driver for machines with LPC313x and LPC315x SoCs.
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/init.h>
#include <linux/interrupt.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <asm/dma.h>
#include <mach/cgu.h>
#include <mach/dma.h>


static spinlock_t driver_lock = SPIN_LOCK_UNLOCKED; /* to guard state variables */

static inline void lpc313x_dma_lock(void)
{
	spin_lock(&driver_lock);
}

static inline void lpc313x_dma_unlock(void)
{
	spin_unlock(&driver_lock);
}

static struct dma_channel {
	char *name;
	dma_cb_t callback_handler;
	void *data;
} dma_channels[DMA_MAX_CHANNELS];

static unsigned int     dma_irq_mask = 0xFFFFFFFF;
static int sg_higher_channel[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int softirqmask[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static int softirqen = 0;

static int dma_channels_requested = 0;

static inline void dma_increment_usage(void)
{
	if (!dma_channels_requested++) {
		cgu_clk_en_dis(CGU_SB_DMA_CLK_GATED_ID, 1);
		cgu_clk_en_dis(CGU_SB_DMA_PCLK_ID, 1);
	}
}
static inline void dma_decrement_usage(void)
{
	if (!--dma_channels_requested) {
		cgu_clk_en_dis(CGU_SB_DMA_CLK_GATED_ID, 0);
		cgu_clk_en_dis(CGU_SB_DMA_PCLK_ID, 0);
	}
}

static inline int dma_valid_config(const dma_setup_t *dma_setup)
{
	if (!dma_setup)
		return -EINVAL;

	if (DMA_CFG_GET_RD_SLV_NR(dma_setup->cfg) > DMA_SLV_SDMMC   ||
	    DMA_CFG_GET_WR_SLV_NR(dma_setup->cfg) > DMA_SLV_SDMMC    ||
	    DMA_CFG_GET_CMP_CH_NR(dma_setup->cfg) >= DMA_MAX_CHANNELS)
		return -EINVAL;

	return 0;
}

int dma_prog_channel (unsigned int chn, dma_setup_t *dma_setup)
{
	if ((chn >= DMA_MAX_CHANNELS) || !dma_channels[chn].name ||
		dma_valid_config(dma_setup) )
		return -EINVAL;
	
	DMACH_SRC_ADDR(chn) = dma_setup->src_address;
	DMACH_DST_ADDR(chn) = dma_setup->dest_address;
	DMACH_LEN(chn) = dma_setup->trans_length;
	DMACH_CFG(chn) = dma_setup->cfg;

	return 0;
}

int dma_request_channel (char *name, dma_cb_t cb, void *data)
{
	unsigned int mask;
	unsigned int chn;
	unsigned long flags;
	dma_setup_t  dma_setup;

	if (!name)
		return -EINVAL;

	lpc313x_dma_lock();

	memset(&dma_setup, 0, sizeof(dma_setup));

	for (chn = 0, mask = 1; chn < DMA_MAX_CHANNELS; chn++) 
	{
		if (!dma_channels[chn].name) 
		{
			dma_increment_usage();
			dma_channels[chn].name = name;
			if (cb) {
				dma_channels[chn].callback_handler = cb;
				dma_channels[chn].data = data;
			}
			dma_prog_channel (chn, &dma_setup);
			local_irq_save(flags);
		        dma_irq_mask &= ~mask;  /* enable the IRQ: dafault behavior */
			DMACH_IRQ_MASK = dma_irq_mask;
			local_irq_restore(flags);
			lpc313x_dma_unlock();
			return chn;
		}
		mask = mask << 2;
	}
	lpc313x_dma_unlock();
	return -EBUSY;
}


int dma_request_specific_channel (int chn, char *name, void (*cb)(int, dma_irq_type_t, void *), void *data)
{
	unsigned long flags;
	dma_setup_t  dma_setup;

	if (chn >= DMA_MAX_CHANNELS || !name)
		return -EINVAL;

	if (dma_channels[chn].name) 
		return -EBUSY;

	lpc313x_dma_lock();

	memset(&dma_setup, 0, sizeof(dma_setup));

	dma_increment_usage();
	dma_channels[chn].name = name;
	if (cb) {
		dma_channels[chn].callback_handler = cb;
		dma_channels[chn].data = data;
	}
	dma_prog_channel (chn, &dma_setup);
	local_irq_save(flags);
	dma_irq_mask &= ~(1 << (2 * chn));  /* enable the IRQ: dafault behavior */
	DMACH_IRQ_MASK = dma_irq_mask;
	local_irq_restore(flags);
	lpc313x_dma_unlock();
	return chn;
}


int dma_set_irq_mask(unsigned int chn, int half_int, int fin_int)
{
	unsigned long flags;

	if (chn >= DMA_MAX_CHANNELS || !dma_channels[chn].name) {
		return -EINVAL;
	}

	lpc313x_dma_lock();
	local_irq_save(flags);

	if (fin_int)
		dma_irq_mask |= (1 << (chn * 2));
	else
		dma_irq_mask &= ~(1 << (chn * 2));

	if  (half_int)
		dma_irq_mask |= (1 << (chn * 2 + 1));
	else
		dma_irq_mask &= ~(1 << (chn * 2 + 1));

	DMACH_IRQ_MASK = dma_irq_mask;

	local_irq_restore(flags);
	lpc313x_dma_unlock();

	return 0;
}

int dma_start_channel (unsigned int chn)
{
	if (chn >= DMA_MAX_CHANNELS || !dma_channels[chn].name) {
		return -EINVAL;
	}

	DMACH_EN(chn) = 1;
	return 0;
}

int dma_stop_channel (unsigned int chn)
{
	if (chn >= DMA_MAX_CHANNELS || !dma_channels[chn].name) {
		return -EINVAL;
	}

	DMACH_EN(chn) = 0;
	return 0;
}

int dma_stop_channel_sg (unsigned int chn)
{
	if (chn >= DMA_MAX_CHANNELS || !dma_channels[chn].name) {
		return -EINVAL;
	}

	/* Disable the companion channel only */
	DMACH_EN(chn - 1) = 0;
	return 0;
}

int dma_release_channel (unsigned int chn)
{
	unsigned int mask = (0x3 << (chn * 2));
	unsigned long flags;

	if (chn >= DMA_MAX_CHANNELS || !dma_channels[chn].name) {
		return -EINVAL;
	}

	lpc313x_dma_lock();
	
	local_irq_save(flags);

	/* Otherwise an unexpected interrupt can occur when the channel is reallocated for another purpose */
	DMACH_IRQ_STATUS = mask;
	dma_irq_mask |= mask;
	DMACH_IRQ_MASK = dma_irq_mask;
	/* reset counter */
	DMACH_TCNT(chn) = 0;
	local_irq_restore(flags);

	dma_channels[chn].name = NULL;
	dma_channels[chn].callback_handler = NULL;
	dma_channels[chn].data = NULL;

	lpc313x_dma_unlock();
	dma_decrement_usage();
	
	return 0;
}

static irqreturn_t dma_irq_handler (int irq, void *dev_id)
{
	unsigned int mask;
	unsigned int chn;
	unsigned int dma_irq_status;

	dma_irq_status = DMACH_IRQ_STATUS;
	dma_irq_status &= ~dma_irq_mask;

	for (chn = 0, mask = 1; chn < DMA_MAX_CHANNELS; chn++) {
		if (dma_irq_status & mask) {
			DMACH_IRQ_STATUS = mask;
			if (dma_channels[chn].callback_handler)
				(dma_channels[chn].callback_handler) 
					(chn, DMA_IRQ_FINISHED, dma_channels[chn].data);
		}
		mask = mask << 1;
		if (dma_irq_status & mask) {
			DMACH_IRQ_STATUS = mask;
			if (dma_channels[chn].callback_handler)
				(dma_channels[chn].callback_handler) 
					(chn, DMA_IRQ_HALFWAY, dma_channels[chn].data);
		}
		mask = mask << 1;
	}

	if (dma_irq_status & DMA_IRQS_SOFT) { /* Soft int */ 
		DMACH_IRQ_STATUS = DMA_IRQS_SOFT;
		for (chn = 0; chn < DMA_MAX_CHANNELS; chn++) {
			if (sg_higher_channel[chn] && softirqmask[chn] &&
				dma_channels[sg_higher_channel[chn]].callback_handler)
			(dma_channels[sg_higher_channel[chn]].callback_handler)
				(sg_higher_channel[chn], DMA_IRQ_SOFTINT,
				dma_channels[sg_higher_channel[chn]].data);
		}
	}

	if (dma_irq_status & DMA_IRQS_ABORT) { /* DMA abort */
		printk(KERN_WARNING "DMA abort signalled\n");
		DMACH_IRQ_STATUS = DMA_IRQS_ABORT;
		for (chn = 0; chn < DMA_MAX_CHANNELS; chn++)
			if (dma_channels[chn].callback_handler)
				(dma_channels[chn].callback_handler) 
					(chn, DMA_IRQ_DMAABORT, dma_channels[chn].data);
	}

	return IRQ_HANDLED;
}

int dma_read_counter (unsigned int chn, unsigned int * pcnt)
{
	if (chn >= DMA_MAX_CHANNELS || !dma_channels[chn].name) {
		return -EINVAL;
	}
	*pcnt = DMACH_TCNT(chn);
	return 0;
}

int dma_write_counter (unsigned int chn, u32 cnt)
{
	if (chn >= DMA_MAX_CHANNELS || !dma_channels[chn].name) {
		return -EINVAL;
	}
	DMACH_TCNT(chn) = cnt;
	return 0;
}


int dma_current_state (unsigned int   chn,
                        unsigned int * psrc,
                        unsigned int * pdst,
                        unsigned int * plen,
                        unsigned int * pcfg,
                        unsigned int * pena,
                        unsigned int * pcnt)
{
	if (chn >= DMA_MAX_CHANNELS || !dma_channels[chn].name) {
		return -EINVAL;
	}

	*psrc = DMACH_SRC_ADDR(chn);
	*pdst = DMACH_DST_ADDR(chn);
	*plen  = DMACH_LEN(chn);
	*pcfg = DMACH_CFG(chn);
	*pena = DMACH_EN(chn);
	*pcnt  = DMACH_TCNT(chn);
	return 0;
}

int dma_request_sg_channel (char *name, dma_cb_t cb, void *data, int usesoftirq)
{
	unsigned int chn;
	unsigned long flags;
	dma_setup_t  dma_setup;

	if (!name)
		return -EINVAL;

	if (softirqen & usesoftirq)
		return -EBUSY;

	lpc313x_dma_lock();

	for (chn = 0; chn < DMA_MAX_CHANNELS - 1; chn++) 
		if (!dma_channels[chn].name && !dma_channels[chn + 1].name) {
			sg_higher_channel[chn] = chn + 1;
			break;
		}

	if (!sg_higher_channel[chn]) {
		lpc313x_dma_unlock();
		return -EBUSY;
	}

	memset(&dma_setup, 0, sizeof(dma_setup));

	dma_increment_usage();
	dma_channels[sg_higher_channel[chn]].name = name;
	dma_channels[sg_higher_channel[chn] - 1].name = name;

	if (cb) {
		dma_channels[sg_higher_channel[chn]].callback_handler = cb;
		dma_channels[sg_higher_channel[chn]].data = data;
	}
	dma_prog_channel (sg_higher_channel[chn], &dma_setup);

	if (usesoftirq) {
		local_irq_save(flags);
		softirqen = 1;
		softirqmask[chn] = 1;
		dma_irq_mask &= ~DMA_IRQS_SOFT;  /* enable the soft IRQ */
		DMACH_IRQ_MASK = dma_irq_mask;
		local_irq_restore(flags);
	}

	lpc313x_dma_unlock();

	return sg_higher_channel[chn];
}

int dma_request_specific_sg_channel (int chn, char *name, dma_cb_t cb, void *data, int usesoftirq)
{
	unsigned long flags;
	dma_setup_t  dma_setup;

	if (!name)
		return -EINVAL;

	if (softirqen & usesoftirq)
		return -EBUSY;

	if (sg_higher_channel[chn] || dma_channels[chn].name || dma_channels[chn - 1].name)
		return -EBUSY;

	lpc313x_dma_lock();
	
	sg_higher_channel[chn] = chn;

	memset(&dma_setup, 0, sizeof(dma_setup));

	dma_increment_usage();
	dma_channels[sg_higher_channel[chn]].name = name;
	dma_channels[sg_higher_channel[chn] - 1].name = name;

	if (cb) {
		dma_channels[sg_higher_channel[chn]].callback_handler = cb;
		dma_channels[sg_higher_channel[chn]].data = data;
	}
	dma_prog_channel (sg_higher_channel[chn], &dma_setup);

	if (usesoftirq) {
		local_irq_save(flags);
		softirqen = 1;
		softirqmask[chn] = 1;
		dma_irq_mask &= ~DMA_IRQS_SOFT;  /* enable the soft IRQ */
		DMACH_IRQ_MASK = dma_irq_mask;
		local_irq_restore(flags);
	}

	return sg_higher_channel[chn];
}

int dma_prog_sg_channel(int chn, u32 dma_sg_list)
{
	u32 dma_config;

	if (chn >= DMA_MAX_CHANNELS)
		return -EINVAL;

	dma_config = DMA_CFG_CMP_CH_EN | DMA_CFG_CMP_CH_NR(chn - 1);

	lpc313x_dma_lock();
	DMACH_SRC_ADDR(chn) = dma_sg_list;
	DMACH_DST_ADDR(chn) = DMACH_ALT_PHYS(chn - 1);
	DMACH_LEN(chn) = 0x4;
	DMACH_CFG(chn) = dma_config;
	lpc313x_dma_unlock();

	return 0;
}

int dma_channel_enabled(unsigned int chn)
{
	if (chn >= DMA_MAX_CHANNELS || !dma_channels[chn].name) {
		return -EINVAL;
	}

	return (DMACH_EN(chn) & 1);
}

static int __init lpc313x_dma_init (void)
{
	int ret = 0;

	memset(dma_channels, 0, sizeof(struct dma_channel) * DMA_MAX_CHANNELS);

	dma_irq_mask = 0xFFFFFFFF;
	DMACH_IRQ_MASK = dma_irq_mask;
	ret = request_irq (IRQ_DMA, dma_irq_handler, 0, "DMAC", NULL);
	if (ret)
		printk (KERN_ERR "request_irq() returned error %d\n", ret);

	return ret;
}

int dma_release_sg_channel (unsigned int chn)
{
	unsigned long flags;

	if (chn >= DMA_MAX_CHANNELS || !dma_channels[chn].name) {
		return -EINVAL;
	}

	lpc313x_dma_lock();

	if (softirqmask[chn] != 0) {
		local_irq_save(flags);
		softirqen = 0;
		softirqmask[chn] = 0;
		dma_irq_mask |= DMA_IRQS_SOFT;
		DMACH_IRQ_MASK = dma_irq_mask;
		local_irq_restore(flags);
	}
	
	dma_channels[chn].name = NULL;
	dma_channels[chn].callback_handler = NULL;
	dma_channels[chn].data = NULL;
	
	chn--;
	dma_channels[chn].name = NULL;
	dma_channels[chn].callback_handler = NULL;
	dma_channels[chn].data = NULL;
	
	sg_higher_channel[chn] = 0;

	lpc313x_dma_unlock();
	dma_decrement_usage();
	return 0;
}
int dma_prepare_sg_list(int n, dma_sg_ll_t * sg)
{
    /* fixed me: not yet implement */
    return 0;
}

device_initcall(lpc313x_dma_init);


EXPORT_SYMBOL(dma_prog_channel);
EXPORT_SYMBOL(dma_request_channel);
EXPORT_SYMBOL(dma_request_specific_channel);
EXPORT_SYMBOL(dma_start_channel);
EXPORT_SYMBOL(dma_stop_channel);
EXPORT_SYMBOL(dma_release_channel);
EXPORT_SYMBOL(dma_set_irq_mask);
EXPORT_SYMBOL(dma_read_counter);
EXPORT_SYMBOL(dma_write_counter);
EXPORT_SYMBOL(dma_current_state);
EXPORT_SYMBOL(dma_request_sg_channel);
EXPORT_SYMBOL(dma_request_specific_sg_channel);
EXPORT_SYMBOL(dma_prog_sg_channel);
EXPORT_SYMBOL(dma_release_sg_channel);
EXPORT_SYMBOL(dma_prepare_sg_list);
EXPORT_SYMBOL(dma_channel_enabled);
