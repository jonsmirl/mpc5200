/*
 * sound/soc/lpc313x/lpc313x-pcm.c
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
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <mach/dma.h>
#include "lpc313x-pcm.h"

#define SND_NAME "lpc313x-audio"
static u64 lpc313x_pcm_dmamask = DMA_BIT_MASK(32);

#if defined (CONFIG_SND_USE_DMA_LINKLIST)
/* The DMA controller in the LPC31XX has limited interrupt support
   for DMA. A timer is used instead to update the current buffer
   position */
#define MIN_PERIODS 8
#define MAX_PERIODS 32
#define DMA_LIST_SIZE (MAX_PERIODS * sizeof(dma_sg_ll_t))
#define MIN_BYTES_PERIOD 2048
#define MAX_BYTES_PERIOD 4096
#define MINTICKINC ((MIN_BYTES_PERIOD * HZ * (MIN_PERIODS / 4)) / (48000 * 2 * 2))

#else
#define MIN_PERIODS 2
#define MAX_PERIODS 2
#define MIN_BYTES_PERIOD (32 * 1024)
#define MAX_BYTES_PERIOD (32 * 1024)
#endif

#if defined (CONFIG_SND_I2S_TX0_MASTER)
#define TX_FIFO_ADDR (I2S_PHYS + 0x0E0)
#define TX_DMA_CHCFG DMA_SLV_I2STX0_L
#else
#define TX_FIFO_ADDR (I2S_PHYS + 0x160)
#define TX_DMA_CHCFG DMA_SLV_I2STX1_L
#endif
#if defined (CONFIG_SND_I2S_RX0_MASTER) | defined (CONFIG_SND_I2S_RX0_SLAVE)
#define RX_FIFO_ADDR (I2S_PHYS + 0x1E0)
#define RX_DMA_CHCFG DMA_SLV_I2SRX0_L
#else
#define RX_FIFO_ADDR (I2S_PHYS + 0x260)
#define RX_DMA_CHCFG DMA_SLV_I2SRX1_L
#endif

static const struct snd_pcm_hardware lpc313x_pcm_hardware = {
	.info = (SNDRV_PCM_INFO_MMAP |
		 SNDRV_PCM_INFO_MMAP_VALID |
		 SNDRV_PCM_INFO_INTERLEAVED |
		 SNDRV_PCM_INFO_RESUME |
		 SNDRV_PCM_INFO_BLOCK_TRANSFER),
	.formats = (SND_SOC_DAIFMT_I2S),
	.period_bytes_min = MIN_BYTES_PERIOD,
	.period_bytes_max = MAX_BYTES_PERIOD,
	.periods_min = MIN_PERIODS,
	.periods_max = MAX_PERIODS,
	.buffer_bytes_max = (MAX_PERIODS * MAX_BYTES_PERIOD)
};

struct lpc313x_dma_data {
	dma_addr_t dma_buffer;	/* physical address of DMA buffer */
	dma_addr_t dma_buffer_end; /* first address beyond DMA buffer */
	size_t period_size;
	int num_periods;

	/* DMA configuration and support */
	int dmach;
	volatile dma_addr_t dma_cur;
	u32 dma_cfg_base;
#if defined (CONFIG_SND_USE_DMA_LINKLIST)
	dma_sg_ll_t *p_sg_cpu;
	dma_sg_ll_t *p_sg_dma;
	struct timer_list timer[2];
	int inccount;
#endif
};

/*
 * DMA ISR - occurs when a new DMA buffer is needed
 */
static void lpc313x_pcm_dma_irq(int ch, dma_irq_type_t dtype, void *handle) {
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *) handle;
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct lpc313x_dma_data *prtd = rtd->private_data;

	(void) dtype;
	(void) ch;

	/* Last buffer is finished */
	prtd->dma_cur += prtd->period_size;
	if (prtd->dma_cur >= prtd->dma_buffer_end)
		prtd->dma_cur = prtd->dma_buffer;

	/* Tell audio system more buffer space is available */
	snd_pcm_period_elapsed(substream);
}

#if defined (CONFIG_SND_USE_DMA_LINKLIST)
static void lpc313x_check_dmall(unsigned long data) {
	struct snd_pcm_substream *substream = (struct snd_pcm_substream *) data;
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct lpc313x_dma_data *prtd = rtd->private_data;

	/* Determine buffer position from current DMA position. We don't need
	   the exact address, just the last finished period */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mod_timer(&prtd->timer[0], jiffies + MINTICKINC);
		prtd->dma_cur = (dma_addr_t) DMACH_SRC_ADDR(prtd->dmach - 1);
	}
	else {
		mod_timer(&prtd->timer[1], jiffies + MINTICKINC);
		prtd->dma_cur = (dma_addr_t) DMACH_DST_ADDR(prtd->dmach - 1);
	}

	/* Tell audio system more buffer space is available */
	snd_pcm_period_elapsed(substream);
}
#endif

static int lpc313x_pcm_allocate_dma_buffer(struct snd_pcm *pcm, int stream)
{
	struct snd_pcm_substream *substream = pcm->streams[stream].substream;
	struct snd_dma_buffer *dmabuf = &substream->dma_buffer;
	size_t size = lpc313x_pcm_hardware.buffer_bytes_max;

	dmabuf->dev.type = SNDRV_DMA_TYPE_DEV;
	dmabuf->dev.dev = pcm->card->dev;
	dmabuf->private_data = NULL;
	dmabuf->area = dma_alloc_coherent(pcm->card->dev, size,
					  &dmabuf->addr, GFP_KERNEL);

	pr_debug("lpc313x-pcm:"
		"preallocate_dma_buffer: area=%p, addr=%p, size=%d\n",
		(void *) dmabuf->area,
		(void *) dmabuf->addr,
		size);

	if (!dmabuf->area)
		return -ENOMEM;

	dmabuf->bytes = size;

	return 0;
}

/*
 * PCM operations
 */
static int lpc313x_pcm_hw_params(struct snd_pcm_substream *substream,
			         struct snd_pcm_hw_params *params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpc313x_dma_data *prtd = runtime->private_data;

	/* this may get called several times by oss emulation
	 * with different params
	 */
	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	runtime->dma_bytes = params_buffer_bytes(params);

	prtd->dma_buffer = runtime->dma_addr;
	prtd->dma_buffer_end = runtime->dma_addr + runtime->dma_bytes;
	prtd->period_size = params_period_bytes(params);
	prtd->num_periods = params_periods(params);

	return 0;
}

static int lpc313x_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct lpc313x_dma_data *prtd = substream->runtime->private_data;

	/* Return the DMA channel */
	if (prtd->dmach != -1) {
#if defined (CONFIG_SND_USE_DMA_LINKLIST)
		dma_release_sg_channel(prtd->dmach);

		/* Return the linked list area */
		dma_free_coherent(substream->pcm->card->dev, 
			DMA_LIST_SIZE, prtd->p_sg_cpu, (dma_addr_t)prtd->p_sg_dma);
#else
		dma_release_channel((unsigned int) prtd->dmach);
#endif

		prtd->dmach = -1;
	}

	return 0;
}

static int lpc313x_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct lpc313x_dma_data *prtd = substream->runtime->private_data;

	/* Setup DMA channel */
	if (prtd->dmach == -1) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
#if defined (CONFIG_SND_USE_DMA_LINKLIST)
			prtd->dmach = dma_request_sg_channel("I2STX",
				lpc313x_pcm_dma_irq, substream, 0);
			prtd->dma_cfg_base = DMA_CFG_TX_WORD |
				DMA_CFG_RD_SLV_NR(0) | DMA_CFG_CMP_CH_EN |
				DMA_CFG_WR_SLV_NR(TX_DMA_CHCFG) |
				DMA_CFG_CMP_CH_NR(prtd->dmach);

#else
			prtd->dmach = dma_request_channel("I2STX",
				lpc313x_pcm_dma_irq, substream);
			prtd->dma_cfg_base = DMA_CFG_TX_WORD |
				DMA_CFG_RD_SLV_NR(0) | DMA_CFG_CIRC_BUF |
				DMA_CFG_WR_SLV_NR(TX_DMA_CHCFG);
#endif
		}
		else {
#if defined (CONFIG_SND_USE_DMA_LINKLIST)
			prtd->dmach = dma_request_sg_channel("I2SRX",
				lpc313x_pcm_dma_irq, substream, 0);
			prtd->dma_cfg_base = DMA_CFG_TX_WORD |
				DMA_CFG_WR_SLV_NR(0) | DMA_CFG_CMP_CH_EN |
				DMA_CFG_RD_SLV_NR(RX_DMA_CHCFG) |
				DMA_CFG_CMP_CH_NR(prtd->dmach);

#else
			prtd->dmach = dma_request_channel("I2SRX",
				lpc313x_pcm_dma_irq, substream);
			prtd->dma_cfg_base = DMA_CFG_TX_WORD |
				DMA_CFG_WR_SLV_NR(0) | DMA_CFG_CIRC_BUF |
				DMA_CFG_RD_SLV_NR(RX_DMA_CHCFG);
#endif
		}

		if (prtd->dmach < 0) {
			pr_err("Error allocating DMA channel\n");
			return prtd->dmach;
		}

#if defined (CONFIG_SND_USE_DMA_LINKLIST)
		/* Allocate space for a DMA linked list */
		prtd->p_sg_cpu = (dma_sg_ll_t *) dma_alloc_coherent(
			substream->pcm->card->dev, DMA_LIST_SIZE,
			(dma_addr_t *) &prtd->p_sg_dma, GFP_KERNEL);

		if (prtd->p_sg_cpu == NULL) {
			pr_err("Error allocating DMA sg list\n");
			dma_release_sg_channel(prtd->dmach);
			prtd->dmach = -1;
			return -ENOMEM;
		}
#endif
	}

	return 0;
}

int dma_stop_channel_sg (unsigned int chn);

static int lpc313x_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_pcm_runtime *rtd = substream->runtime;
	struct lpc313x_dma_data *prtd = rtd->private_data;
	int ret = 0;

#if defined (CONFIG_SND_USE_DMA_LINKLIST)
	int i, tch;
	u32 addr;
	dma_sg_ll_t *p_sg_cpuw, *p_sg_dmaw;
	unsigned long timeout;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		tch = 0;
	}
	else {
		tch = 1;
	}

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		prtd->dma_cur = prtd->dma_buffer;
		p_sg_cpuw = prtd->p_sg_cpu;
		p_sg_dmaw = prtd->p_sg_dma;

		/* Build a linked list that wraps around */
		addr = (u32) prtd->dma_buffer;
		for (i = 0; i < prtd->num_periods; i++) {
			p_sg_cpuw->setup.trans_length = (prtd->period_size / 4) - 1;
			p_sg_cpuw->setup.cfg = prtd->dma_cfg_base;
			p_sg_cpuw->next_entry = (u32) (p_sg_dmaw + 1);

			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
				p_sg_cpuw->setup.src_address = addr;
				p_sg_cpuw->setup.dest_address = TX_FIFO_ADDR;
			}
			else {
				p_sg_cpuw->setup.dest_address = addr;
				p_sg_cpuw->setup.src_address = RX_FIFO_ADDR;
			}

			/* Wrap end of list back to start? */
			if (i == (prtd->num_periods - 1))
				p_sg_cpuw->next_entry = (u32) prtd->p_sg_dma;

			p_sg_cpuw++;
			p_sg_dmaw++;
			addr += prtd->period_size;
		}

		/* Add and start audio data position timer */
		init_timer(&prtd->timer[tch]);
		prtd->timer[tch].data = (unsigned long) substream;
		prtd->timer[tch].function = lpc313x_check_dmall;
		prtd->timer[tch].expires = jiffies + MINTICKINC;
		add_timer(&prtd->timer[tch]);

		/* Program DMA channel and start it */
		dma_prog_sg_channel(prtd->dmach, (u32) prtd->p_sg_dma);
		dma_set_irq_mask(prtd->dmach, 1, 1);
#else
	dma_setup_t dmasetup;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		prtd->dma_cur = prtd->dma_buffer;

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
			dmasetup.src_address = (u32) prtd->dma_buffer;
			dmasetup.dest_address = TX_FIFO_ADDR;
		}
		else {
			dmasetup.dest_address = (u32) prtd->dma_buffer;
			dmasetup.src_address = RX_FIFO_ADDR;
		}

		dmasetup.cfg = prtd->dma_cfg_base;
		dmasetup.trans_length = (2 * prtd->period_size / 4) - 1;

		/* Program DMA channel and start it */
		dma_prog_channel(prtd->dmach, &dmasetup);
		dma_set_irq_mask(prtd->dmach, 0, 0);
#endif
		dma_start_channel(prtd->dmach);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
#if defined (CONFIG_SND_USE_DMA_LINKLIST)
		del_timer_sync(&prtd->timer[tch]);
#endif
		/* Stop the companion channel and let the current DMA
		   transfer finish */
		dma_stop_channel_sg(prtd->dmach);
		timeout = jiffies + (HZ / 20);
		while ((dma_channel_enabled(prtd->dmach)) &&
			(jiffies < timeout)) {
			cpu_relax();
		}

//		dma_stop_channel(prtd->dmach);
		break;

	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		break;

	default:
		pr_warning("lpc313x_pcm_trigger: Unsupported cmd: %d\n",
				cmd);
		ret = -EINVAL;
	}

	return ret;
}

static snd_pcm_uframes_t lpc313x_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpc313x_dma_data *prtd = runtime->private_data;
	snd_pcm_uframes_t x;

	/* Return an offset into the DMA buffer for the next data */
	x = bytes_to_frames(runtime, (prtd->dma_cur - runtime->dma_addr));
	if (x >= runtime->buffer_size)
		x = 0;

	return x;
}

static int lpc313x_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct lpc313x_dma_data *prtd;
	int ret = 0;

	snd_soc_set_runtime_hwparams(substream, &lpc313x_pcm_hardware);

	/* ensure that buffer size is a multiple of period size */
	ret = snd_pcm_hw_constraint_integer(runtime,
		SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0)
		goto out;

	prtd = kzalloc(sizeof(*prtd), GFP_KERNEL);
	if (prtd == NULL) {
		ret = -ENOMEM;
		goto out;
	}
	runtime->private_data = prtd;
	prtd->dmach = -1;

out:
	return ret;
}

static int lpc313x_pcm_close(struct snd_pcm_substream *substream)
{
	struct lpc313x_dma_data *prtd = substream->runtime->private_data;

	kfree(prtd);
	return 0;
}

static int lpc313x_pcm_mmap(struct snd_pcm_substream *substream,
			    struct vm_area_struct *vma)
{
	struct snd_pcm_runtime *runtime = substream->runtime;

	return dma_mmap_writecombine(substream->pcm->card->dev, vma,
				     runtime->dma_area,
				     runtime->dma_addr,
				     runtime->dma_bytes);
}

static struct snd_pcm_ops lpc313x_pcm_ops = {
	.open = lpc313x_pcm_open,
	.close = lpc313x_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = lpc313x_pcm_hw_params,
	.hw_free = lpc313x_pcm_hw_free,
	.prepare = lpc313x_pcm_prepare,
	.trigger = lpc313x_pcm_trigger,
	.pointer = lpc313x_pcm_pointer,
	.mmap = lpc313x_pcm_mmap,
};

/*
 * ASoC platform driver
 */
static int lpc313x_pcm_new(struct snd_card *card,
			   struct snd_soc_dai *dai,
			   struct snd_pcm *pcm)
{
	int ret = 0;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &lpc313x_pcm_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (dai->playback.channels_min) {
		ret = lpc313x_pcm_allocate_dma_buffer(
			  pcm, SNDRV_PCM_STREAM_PLAYBACK);
		if (ret)
			goto out;
	}

	if (dai->capture.channels_min) {
		pr_debug("%s: Allocating PCM capture DMA buffer\n", SND_NAME);
		ret = lpc313x_pcm_allocate_dma_buffer(
			  pcm, SNDRV_PCM_STREAM_CAPTURE);
		if (ret)
			goto out;
	}

out:
	return ret;
}

static void lpc313x_pcm_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	struct snd_dma_buffer *buf;
	int stream;

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (substream == NULL)
			continue;

		buf = &substream->dma_buffer;
		if (!buf->area)
			continue;

		dma_free_coherent(pcm->card->dev, buf->bytes,
				  buf->area, buf->addr);

		buf->area = NULL;
	}
}

#if defined(CONFIG_PM)
static int lpc313x_pcm_suspend(struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = dai->runtime;
	struct lpc313x_dma_data *prtd;

	if (runtime == NULL)
		return 0;

	prtd = runtime->private_data;

	/* Nothing to do here */

	return 0;
}

static int lpc313x_pcm_resume(struct snd_soc_dai *dai)
{
	struct snd_pcm_runtime *runtime = dai->runtime;
	struct lpc313x_dma_data *prtd;

	if (runtime == NULL)
		return 0;

	prtd = runtime->private_data;

	/* Nothing to do here */

	return 0;
}

#else
#define lpc313x_pcm_suspend	NULL
#define lpc313x_pcm_resume	NULL
#endif

struct snd_soc_platform lpc313x_soc_platform = {
	.name = SND_NAME,
	.pcm_ops = &lpc313x_pcm_ops,
	.pcm_new = lpc313x_pcm_new,
	.pcm_free = lpc313x_pcm_free_dma_buffers,
	.suspend = lpc313x_pcm_suspend,
	.resume = lpc313x_pcm_resume,
};
EXPORT_SYMBOL_GPL(lpc313x_soc_platform);
static int __init lpc313x_soc_platform_init(void)
{
	return snd_soc_register_platform(&lpc313x_soc_platform);
}
module_init(lpc313x_soc_platform_init);

static void __exit lpc313x_soc_platform_exit(void)
{
	snd_soc_unregister_platform(&lpc313x_soc_platform);
}
module_exit(lpc313x_soc_platform_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("NXP LPC313X PCM module");
MODULE_LICENSE("GPL");
