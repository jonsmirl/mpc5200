/*
 * Freescale DMA ALSA SoC PCM driver
 *
 * Author: Timur Tabi <timur@freescale.com>
 *
 * Copyright 2007-2008 Freescale Semiconductor, Inc.  This file is licensed
 * under the terms of the GNU General Public License version 2.  This
 * program is licensed "as is" without any warranty of any kind, whether
 * express or implied.
 *
 * This driver implements ASoC support for the Elo DMA controller, which is
 * the DMA controller on Freescale 83xx, 85xx, and 86xx SOCs. In ALSA terms,
 * the PCM driver is what handles the DMA buffer.
 *
 * The DMA driver is not really a stand-alone driver as much as it's a library
 * for the SSI driver.  This is because the DMA channels cannot really stand
 * alone.  A particular SSI device needs to know which two DMA channels to use
 * for playback and capture.  If the DMA driver were a regular OF driver, it
 * would get probed for each DMA channel, and then it would have to figure out
 * which SSI to use for those channels.  That's just too complicated.  So we
 * load like a normal platform driver and wait for the SSI driver to give us the
 * information we need via cpu_dai->dma_data.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include <asm/io.h>

#include "fsl_dma.h"

/*
 * The formats that the DMA controller supports, which is anything
 * that is 8, 16, or 32 bits.
 */
#define FSLDMA_PCM_FORMATS (SNDRV_PCM_FMTBIT_S8 	| \
			    SNDRV_PCM_FMTBIT_U8 	| \
			    SNDRV_PCM_FMTBIT_S16_LE     | \
			    SNDRV_PCM_FMTBIT_S16_BE     | \
			    SNDRV_PCM_FMTBIT_U16_LE     | \
			    SNDRV_PCM_FMTBIT_U16_BE     | \
			    SNDRV_PCM_FMTBIT_S24_LE     | \
			    SNDRV_PCM_FMTBIT_S24_BE     | \
			    SNDRV_PCM_FMTBIT_U24_LE     | \
			    SNDRV_PCM_FMTBIT_U24_BE     | \
			    SNDRV_PCM_FMTBIT_S32_LE     | \
			    SNDRV_PCM_FMTBIT_S32_BE     | \
			    SNDRV_PCM_FMTBIT_U32_LE     | \
			    SNDRV_PCM_FMTBIT_U32_BE)

#define FSLDMA_PCM_RATES (SNDRV_PCM_RATE_5512 | SNDRV_PCM_RATE_8000_192000 | \
			  SNDRV_PCM_RATE_CONTINUOUS)

/*
 * The number of DMA links to use.  Two is the bare minimum, but if you
 * have really small links you might need more.
 */
#define NUM_DMA_LINKS   2

/** fsl_dma_private: per-substream DMA data
 *
 * Each substream has a 1-to-1 association with a DMA channel.
 *
 * The link[] array is first because it needs to be aligned on a 32-byte
 * boundary, so putting it first will ensure alignment without padding the
 * structure.
 *
 * @link[]: array of link descriptors
 * @controller_id: which DMA controller (0, 1, ...)
 * @channel_id: which DMA channel on the controller (0, 1, 2, ...)
 * @dma_channel: pointer to the DMA channel's registers
 * @irq: IRQ for this DMA channel
 * @substream: pointer to the substream object, needed by the ISR
 * @ssi_sxx_phys: bus address of the STX or SRX register to use
 * @ld_buf_phys: physical address of the LD buffer
 * @current_link: index into link[] of the link currently being processed
 * @dma_buf_phys: physical address of the DMA buffer
 * @dma_buf_next: physical address of the next period to process
 * @dma_buf_end: physical address of the byte after the end of the DMA
 * @buffer period_size: the size of a single period
 * @num_periods: the number of periods in the DMA buffer
 */
struct fsl_dma_private {
	struct fsl_dma_link_descriptor link[NUM_DMA_LINKS];
	struct fsl_dma_info *dma_info;
	struct snd_pcm_substream *substream;
	dma_addr_t ld_buf_phys;
	unsigned int current_link;
	dma_addr_t dma_buf_phys;
	dma_addr_t dma_buf_next;
	dma_addr_t dma_buf_end;
	size_t period_size;
	unsigned int num_periods;
};

/**
 * fsl_dma_hardare: define characteristics of the PCM hardware.
 *
 * The PCM hardware is the Freescale DMA controller.  This structure defines
 * the capabilities of that hardware.
 *
 * Since the sampling rate and data format are not controlled by the DMA
 * controller, we specify no limits for those values.  The only exception is
 * period_bytes_min, which is set to a reasonably low value to prevent the
 * DMA controller from generating too many interrupts per second.
 *
 * Since each link descriptor has a 32-bit byte count field, we set
 * period_bytes_max to the largest 32-bit number.  We also have no maximum
 * number of periods.
 */
static const struct snd_pcm_hardware fsl_dma_hardware = {
	.info   		= SNDRV_PCM_INFO_INTERLEAVED |
				  SNDRV_PCM_INFO_MMAP |
				  SNDRV_PCM_INFO_MMAP_VALID,
	.formats		= FSLDMA_PCM_FORMATS,
	.rates  		= FSLDMA_PCM_RATES,
	.rate_min       	= 5512,
	.rate_max       	= 192000,
	.period_bytes_min       = 512,  	/* A reasonable limit */
	.period_bytes_max       = (uint32_t) -1,
	.periods_min    	= NUM_DMA_LINKS,
	.periods_max    	= (unsigned int) -1,
	.buffer_bytes_max       = 128 * 1024,   /* A reasonable limit */
};

/**
 * fsl_dma_abort_stream: tell ALSA that the DMA transfer has aborted
 *
 * This function should be called by the ISR whenever the DMA controller
 * halts data transfer.
 */
static void fsl_dma_abort_stream(struct snd_pcm_substream *substream)
{
	unsigned long flags;

	snd_pcm_stream_lock_irqsave(substream, flags);

	if (snd_pcm_running(substream))
		snd_pcm_stop(substream, SNDRV_PCM_STATE_XRUN);

	snd_pcm_stream_unlock_irqrestore(substream, flags);
}

/**
 * fsl_dma_update_pointers - update LD pointers to point to the next period
 *
 * As each period is completed, this function changes the the link
 * descriptor pointers for that period to point to the next period.
 */
static void fsl_dma_update_pointers(struct fsl_dma_private *dma_private)
{
	struct fsl_dma_link_descriptor *link =
		&dma_private->link[dma_private->current_link];

	/* Update our link descriptors to point to the next period */
	if (dma_private->substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		link->source_addr =
			cpu_to_be32(dma_private->dma_buf_next);
	else
		link->dest_addr =
			cpu_to_be32(dma_private->dma_buf_next);

	/* Update our variables for next time */
	dma_private->dma_buf_next += dma_private->period_size;

	if (dma_private->dma_buf_next >= dma_private->dma_buf_end)
		dma_private->dma_buf_next = dma_private->dma_buf_phys;

	if (++dma_private->current_link >= NUM_DMA_LINKS)
		dma_private->current_link = 0;
}

/**
 * fsl_dma_isr: interrupt handler for the DMA controller
 *
 * @irq: IRQ of the DMA channel
 * @dev_id: pointer to the dma_private structure for this DMA channel
 */
static irqreturn_t fsl_dma_isr(int irq, void *dev_id)
{
	struct fsl_dma_private *dma_private = dev_id;
	struct ccsr_dma_channel __iomem *dma_channel =
		dma_private->dma_info->channel;
	irqreturn_t ret = IRQ_NONE;
	uint32_t sr, sr2 = 0;

	/* We got an interrupt, so read the status register to see what we
	   were interrupted for.
	 */
	sr = in_be32(&dma_channel->sr);

	if (sr & CCSR_DMA_SR_TE) {
		dev_err(dma_private->substream->pcm->card->dev,
			"DMA%u transmit error (channel=%u irq=%u)\n",
			dma_private->dma_info->controller_id,
			dma_private->dma_info->channel_id, irq);
		fsl_dma_abort_stream(dma_private->substream);
		sr2 |= CCSR_DMA_SR_TE;
		ret = IRQ_HANDLED;
	}

	if (sr & CCSR_DMA_SR_CH)
		ret = IRQ_HANDLED;

	if (sr & CCSR_DMA_SR_PE) {
		dev_err(dma_private->substream->pcm->card->dev,
			"DMA%u programming error (channel=%u irq=%u)\n",
			dma_private->dma_info->controller_id,
			dma_private->dma_info->channel_id, irq);
		fsl_dma_abort_stream(dma_private->substream);
		sr2 |= CCSR_DMA_SR_PE;
		ret = IRQ_HANDLED;
	}

	if (sr & CCSR_DMA_SR_EOLNI) {
		sr2 |= CCSR_DMA_SR_EOLNI;
		ret = IRQ_HANDLED;
	}

	if (sr & CCSR_DMA_SR_CB)
		ret = IRQ_HANDLED;

	if (sr & CCSR_DMA_SR_EOSI) {
		/* Tell ALSA we completed a period. */
		snd_pcm_period_elapsed(dma_private->substream);

		/*
		 * Update our link descriptors to point to the next period. We
		 * only need to do this if the number of periods is not equal to
		 * the number of links.
		 */
		if (dma_private->num_periods != NUM_DMA_LINKS)
			fsl_dma_update_pointers(dma_private);

		sr2 |= CCSR_DMA_SR_EOSI;
		ret = IRQ_HANDLED;
	}

	if (sr & CCSR_DMA_SR_EOLSI) {
		sr2 |= CCSR_DMA_SR_EOLSI;
		ret = IRQ_HANDLED;
	}

	/* Clear the bits that we set */
	if (sr2)
		out_be32(&dma_channel->sr, sr2);

	return ret;
}

/**
 * fsl_dma_new: initialize this PCM driver.
 *
 * This function is called when the codec driver calls snd_soc_new_pcms(),
 * once for each .dai_link in the soc_card driver's snd_soc_card
 * structure.
 *
 * The DMA controller does support 36-bit addresses for the DMA buffers, but
 * supporting that is clunky so we restrict ourselves to 32-bit addresses.
 */
static int fsl_dma_new(struct snd_soc_platform *platform,
	struct snd_card *card, int playback, int capture, struct snd_pcm *pcm)
{
	static u64 fsl_dma_dmamask = DMA_BIT_MASK(32);
	int ret;

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &fsl_dma_dmamask;

	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = fsl_dma_dmamask;

	if (playback) {
		ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, pcm->dev,
			fsl_dma_hardware.buffer_bytes_max,
			&pcm->streams[0].substream->dma_buffer);
		if (ret) {
			dev_err(card->dev,
				"cannot allocate playback buffer (size=%u)\n",
				fsl_dma_hardware.buffer_bytes_max);
			goto error;
		}
	}

	if (capture) {
		ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, pcm->dev,
			fsl_dma_hardware.buffer_bytes_max,
			&pcm->streams[1].substream->dma_buffer);
		if (ret) {
			dev_err(card->dev,
				"cannot allocate capture buffer (size=%u)\n",
				fsl_dma_hardware.buffer_bytes_max);
			goto error;
		}
	}

	return 0;

error:
	if (pcm->streams[0].substream->dma_buffer.area)
		snd_dma_free_pages(&pcm->streams[0].substream->dma_buffer);

	if (pcm->streams[1].substream->dma_buffer.area)
		snd_dma_free_pages(&pcm->streams[1].substream->dma_buffer);

	return ret;
}

/**
 * fsl_dma_open: open a new substream.
 *
 * Each substream has its own DMA buffer.
 */
static int fsl_dma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct fsl_dma_info *dma_info = cpu_dai->dma_data;

	struct fsl_dma_private *dma_private;
	struct ccsr_dma_channel __iomem *dma_channel;
	dma_addr_t ld_buf_phys;
	u64 temp_link;  	/* Pointer to next link descriptor */
	u32 mr;
	unsigned int channel;
	int ret = 0;
	unsigned int i;

	/*
	 * Reject any DMA buffer whose size is not a multiple of the period
	 * size.  We need to make sure that the DMA buffer can be evenly divided
	 * into periods.
	 */
	ret = snd_pcm_hw_constraint_integer(runtime,
		SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret < 0) {
		dev_err(substream->pcm->card->dev, "invalid buffer size\n");
		return ret;
	}

	channel = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ? 0 : 1;

	dma_private = dma_alloc_coherent(substream->pcm->dev,
		sizeof(struct fsl_dma_private), &ld_buf_phys, GFP_KERNEL);
	if (!dma_private) {
		dev_err(substream->pcm->card->dev,
			"can't allocate DMA private data\n");
		return -ENOMEM;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		dma_private->dma_info = &dma_info[0];
	else
		dma_private->dma_info = &dma_info[1];

	dma_private->substream = substream;
	dma_private->ld_buf_phys = ld_buf_phys;
	dma_private->dma_buf_phys = substream->dma_buffer.addr;

	ret = request_irq(dma_private->dma_info->irq,
		fsl_dma_isr, 0, "DMA", dma_private);
	if (ret) {
		dev_err(substream->pcm->card->dev,
			"can't register ISR for IRQ %u (ret=%i)\n",
			dma_private->dma_info->irq, ret);
		dma_free_coherent(substream->pcm->dev,
			sizeof(struct fsl_dma_private),
			dma_private, dma_private->ld_buf_phys);
		return ret;
	}

	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);
	snd_soc_set_runtime_hwparams(substream, &fsl_dma_hardware);
	runtime->private_data = dma_private;

	/* Program the fixed DMA controller parameters */

	dma_channel = dma_private->dma_info->channel;

	temp_link = dma_private->ld_buf_phys +
		sizeof(struct fsl_dma_link_descriptor);

	for (i = 0; i < NUM_DMA_LINKS; i++) {
		struct fsl_dma_link_descriptor *link = &dma_private->link[i];

		link->source_attr = cpu_to_be32(CCSR_DMA_ATR_SNOOP);
		link->dest_attr = cpu_to_be32(CCSR_DMA_ATR_SNOOP);
		link->next = cpu_to_be64(temp_link);

		temp_link += sizeof(struct fsl_dma_link_descriptor);
	}
	/* The last link descriptor points to the first */
	dma_private->link[i - 1].next = cpu_to_be64(dma_private->ld_buf_phys);

	/* Tell the DMA controller where the first link descriptor is */
	out_be32(&dma_channel->clndar,
		CCSR_DMA_CLNDAR_ADDR(dma_private->ld_buf_phys));
	out_be32(&dma_channel->eclndar,
		CCSR_DMA_ECLNDAR_ADDR(dma_private->ld_buf_phys));

	/* The manual says the BCR must be clear before enabling EMP */
	out_be32(&dma_channel->bcr, 0);

	/*
	 * Program the mode register for interrupts, external master control,
	 * and source/destination hold.  Also clear the Channel Abort bit.
	 */
	mr = in_be32(&dma_channel->mr) &
		~(CCSR_DMA_MR_CA | CCSR_DMA_MR_DAHE | CCSR_DMA_MR_SAHE);

	/*
	 * We want External Master Start and External Master Pause enabled,
	 * because the SSI is controlling the DMA controller.  We want the DMA
	 * controller to be set up in advance, and then we signal only the SSI
	 * to start transfering.
	 *
	 * We want End-Of-Segment Interrupts enabled, because this will generate
	 * an interrupt at the end of each segment (each link descriptor
	 * represents one segment).  Each DMA segment is the same thing as an
	 * ALSA period, so this is how we get an interrupt at the end of every
	 * period.
	 *
	 * We want Error Interrupt enabled, so that we can get an error if
	 * the DMA controller is mis-programmed somehow.
	 */
	mr |= CCSR_DMA_MR_EOSIE | CCSR_DMA_MR_EIE | CCSR_DMA_MR_EMP_EN |
		CCSR_DMA_MR_EMS_EN;

	/* For playback, we want the destination address to be held.  For
	   capture, set the source address to be held. */
	mr |= (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) ?
		CCSR_DMA_MR_DAHE : CCSR_DMA_MR_SAHE;

	out_be32(&dma_channel->mr, mr);

	return 0;
}

/**
 * fsl_dma_hw_params: allocate the DMA buffer and the DMA link descriptors.
 *
 * ALSA divides the DMA buffer into N periods.  We create NUM_DMA_LINKS link
 * descriptors that ping-pong from one period to the next.  For example, if
 * there are six periods and two link descriptors, this is how they look
 * before playback starts:
 *
 *      	   The last link descriptor
 *   ____________  points back to the first
 *  |   	 |
 *  V   	 |
 *  ___    ___   |
 * |   |->|   |->|
 * |___|  |___|
 *   |      |
 *   |      |
 *   V      V
 *  _________________________________________
 * |      |      |      |      |      |      |  The DMA buffer is
 * |      |      |      |      |      |      |    divided into 6 parts
 * |______|______|______|______|______|______|
 *
 * and here's how they look after the first period is finished playing:
 *
 *   ____________
 *  |   	 |
 *  V   	 |
 *  ___    ___   |
 * |   |->|   |->|
 * |___|  |___|
 *   |      |
 *   |______________
 *          |       |
 *          V       V
 *  _________________________________________
 * |      |      |      |      |      |      |
 * |      |      |      |      |      |      |
 * |______|______|______|______|______|______|
 *
 * The first link descriptor now points to the third period.  The DMA
 * controller is currently playing the second period.  When it finishes, it
 * will jump back to the first descriptor and play the third period.
 *
 * There are four reasons we do this:
 *
 * 1. The only way to get the DMA controller to automatically restart the
 *    transfer when it gets to the end of the buffer is to use chaining
 *    mode.  Basic direct mode doesn't offer that feature.
 * 2. We need to receive an interrupt at the end of every period.  The DMA
 *    controller can generate an interrupt at the end of every link transfer
 *    (aka segment).  Making each period into a DMA segment will give us the
 *    interrupts we need.
 * 3. By creating only two link descriptors, regardless of the number of
 *    periods, we do not need to reallocate the link descriptors if the
 *    number of periods changes.
 * 4. All of the audio data is still stored in a single, contiguous DMA
 *    buffer, which is what ALSA expects.  We're just dividing it into
 *    contiguous parts, and creating a link descriptor for each one.
 *
 * Note that due to a quirk of the SSI's STX register, the target address
 * for the DMA operations depends on the sample size.  So we don't program
 * the dest_addr (for playback -- source_addr for capture) fields in the
 * link descriptors here.  We do that in fsl_dma_prepare()
 */
static int fsl_dma_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *hw_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct fsl_dma_private *dma_private = runtime->private_data;

	dma_addr_t temp_addr;   /* Pointer to next period */

	unsigned int i;

	/* Get all the parameters we need */
	size_t buffer_size = params_buffer_bytes(hw_params);
	size_t period_size = params_period_bytes(hw_params);

	/* Initialize our DMA tracking variables */
	dma_private->period_size = period_size;
	dma_private->num_periods = params_periods(hw_params);
	dma_private->dma_buf_end = dma_private->dma_buf_phys + buffer_size;
	dma_private->dma_buf_next = dma_private->dma_buf_phys +
		(NUM_DMA_LINKS * period_size);
	if (dma_private->dma_buf_next >= dma_private->dma_buf_end)
		dma_private->dma_buf_next = dma_private->dma_buf_phys;

	/*
	 * The actual address in STX0 (destination for playback, source for
	 * capture) is based on the sample size, but we don't know the sample
	 * size in this function, so we'll have to adjust that later.  See
	 * comments in fsl_dma_prepare().
	 *
	 * The DMA controller does not have a cache, so the CPU does not
	 * need to tell it to flush its cache.  However, the DMA
	 * controller does need to tell the CPU to flush its cache.
	 * That's what the SNOOP bit does.
	 *
	 * Also, even though the DMA controller supports 36-bit addressing, for
	 * simplicity we currently support only 32-bit addresses for the audio
	 * buffer itself.
	 */
	temp_addr = substream->dma_buffer.addr;

	for (i = 0; i < NUM_DMA_LINKS; i++) {
		struct fsl_dma_link_descriptor *link = &dma_private->link[i];

		link->count = cpu_to_be32(period_size);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			link->source_addr = cpu_to_be32(temp_addr);
		else
			link->dest_addr = cpu_to_be32(temp_addr);

		temp_addr += period_size;
	}

	return 0;
}

/**
 * fsl_dma_prepare - prepare the DMA registers for playback.
 *
 * This function is called after the specifics of the audio data are known,
 * i.e. snd_pcm_runtime is initialized.
 *
 * In this function, we finish programming the registers of the DMA
 * controller that are dependent on the sample size.
 *
 * One of the drawbacks with big-endian is that when copying integers of
 * different sizes to a fixed-sized register, the address to which the
 * integer must be copied is dependent on the size of the integer.
 *
 * For example, if P is the address of a 32-bit register, and X is a 32-bit
 * integer, then X should be copied to address P.  However, if X is a 16-bit
 * integer, then it should be copied to P+2.  If X is an 8-bit register,
 * then it should be copied to P+3.
 *
 * So for playback of 8-bit samples, the DMA controller must transfer single
 * bytes from the DMA buffer to the last byte of the STX0 register, i.e.
 * offset by 3 bytes. For 16-bit samples, the offset is two bytes.
 *
 * For 24-bit samples, the offset is 1 byte.  However, the DMA controller
 * does not support 3-byte copies (the DAHTS register supports only 1, 2, 4,
 * and 8 bytes at a time).  So we do not support packed 24-bit samples.
 * 24-bit data must be padded to 32 bits.
 */
static int fsl_dma_prepare(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct fsl_dma_private *dma_private = runtime->private_data;
	struct ccsr_dma_channel __iomem *dma_channel =
		dma_private->dma_info->channel;
	uint32_t mr;
	unsigned int i;
	dma_addr_t ssi_sxx_phys;	/* Bus address of SSI STX register */
	unsigned int frame_size;	/* Number of bytes per frame */

	ssi_sxx_phys = dma_private->dma_info->ssi_sxx_phys;

	mr = in_be32(&dma_channel->mr) & ~(CCSR_DMA_MR_BWC_MASK |
		  CCSR_DMA_MR_SAHTS_MASK | CCSR_DMA_MR_DAHTS_MASK);

	switch (runtime->sample_bits) {
	case 8:
		mr |= CCSR_DMA_MR_DAHTS_1 | CCSR_DMA_MR_SAHTS_1;
		ssi_sxx_phys += 3;
		break;
	case 16:
		mr |= CCSR_DMA_MR_DAHTS_2 | CCSR_DMA_MR_SAHTS_2;
		ssi_sxx_phys += 2;
		break;
	case 32:
		mr |= CCSR_DMA_MR_DAHTS_4 | CCSR_DMA_MR_SAHTS_4;
		break;
	default:
		dev_err(substream->pcm->card->dev,
			"unsupported sample size %u\n", runtime->sample_bits);
		return -EINVAL;
	}

	frame_size = runtime->frame_bits / 8;
	/*
	 * BWC should always be a multiple of the frame size.  BWC determines
	 * how many bytes are sent/received before the DMA controller checks the
	 * SSI to see if it needs to stop.  For playback, the transmit FIFO can
	 * hold three frames, so we want to send two frames at a time. For
	 * capture, the receive FIFO is triggered when it contains one frame, so
	 * we want to receive one frame at a time.
	 */

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		mr |= CCSR_DMA_MR_BWC(2 * frame_size);
	else
		mr |= CCSR_DMA_MR_BWC(frame_size);

	out_be32(&dma_channel->mr, mr);

	/*
	 * Program the address of the DMA transfer to/from the SSI.
	 */
	for (i = 0; i < NUM_DMA_LINKS; i++) {
		struct fsl_dma_link_descriptor *link = &dma_private->link[i];

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			link->dest_addr = cpu_to_be32(ssi_sxx_phys);
		else
			link->source_addr = cpu_to_be32(ssi_sxx_phys);
	}

	return 0;
}

/**
 * fsl_dma_pointer: determine the current position of the DMA transfer
 *
 * This function is called by ALSA when ALSA wants to know where in the
 * stream buffer the hardware currently is.
 *
 * For playback, the SAR register contains the physical address of the most
 * recent DMA transfer.  For capture, the value is in the DAR register.
 *
 * The base address of the buffer is stored in the source_addr field of the
 * first link descriptor.
 */
static snd_pcm_uframes_t fsl_dma_pointer(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct fsl_dma_private *dma_private = runtime->private_data;
	struct ccsr_dma_channel __iomem *dma_channel =
		dma_private->dma_info->channel;
	dma_addr_t position;
	snd_pcm_uframes_t frames;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		position = in_be32(&dma_channel->sar);
	else
		position = in_be32(&dma_channel->dar);

	frames = bytes_to_frames(runtime, position - dma_private->dma_buf_phys);

	/*
	 * If the current address is just past the end of the buffer, wrap it
	 * around.
	 */
	if (frames == runtime->buffer_size)
		frames = 0;

	return frames;
}

/**
 * fsl_dma_hw_free: release resources allocated in fsl_dma_hw_params()
 *
 * Release the resources allocated in fsl_dma_hw_params() and de-program the
 * registers.
 *
 * This function can be called multiple times.
 */
static int fsl_dma_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct fsl_dma_private *dma_private = runtime->private_data;

	if (dma_private) {
		struct ccsr_dma_channel __iomem *dma_channel;

		dma_channel = dma_private->dma_info->channel;

		/* Stop the DMA */
		out_be32(&dma_channel->mr, CCSR_DMA_MR_CA);
		out_be32(&dma_channel->mr, 0);

		/* Reset all the other registers */
		out_be32(&dma_channel->sr, -1);
		out_be32(&dma_channel->clndar, 0);
		out_be32(&dma_channel->eclndar, 0);
		out_be32(&dma_channel->satr, 0);
		out_be32(&dma_channel->sar, 0);
		out_be32(&dma_channel->datr, 0);
		out_be32(&dma_channel->dar, 0);
		out_be32(&dma_channel->bcr, 0);
		out_be32(&dma_channel->nlndar, 0);
		out_be32(&dma_channel->enlndar, 0);
	}

	return 0;
}

/**
 * fsl_dma_close: close the stream.
 */
static int fsl_dma_close(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct fsl_dma_private *dma_private = runtime->private_data;

	if (dma_private) {
		if (dma_private->dma_info->irq)
			free_irq(dma_private->dma_info->irq, dma_private);

		if (dma_private->ld_buf_phys) {
			dma_unmap_single(substream->pcm->dev,
				dma_private->ld_buf_phys,
				sizeof(dma_private->link), DMA_TO_DEVICE);
		}

		/* Deallocate the fsl_dma_private structure */
		dma_free_coherent(substream->pcm->dev,
			sizeof(struct fsl_dma_private),
			dma_private, dma_private->ld_buf_phys);
		substream->runtime->private_data = NULL;
	}

	return 0;
}

/*
 * Remove this PCM driver.
 */
static void fsl_dma_free_dma_buffers(struct snd_pcm *pcm)
{
	struct snd_pcm_substream *substream;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(pcm->streams); i++) {
		substream = pcm->streams[i].substream;
		if (substream) {
			snd_dma_free_pages(&substream->dma_buffer);
			substream->dma_buffer.area = NULL;
			substream->dma_buffer.addr = 0;
		}
	}
}

static struct snd_pcm_ops fsl_dma_ops = {
	.open   	= fsl_dma_open,
	.close  	= fsl_dma_close,
	.ioctl  	= snd_pcm_lib_ioctl,
	.hw_params      = fsl_dma_hw_params,
	.hw_free	= fsl_dma_hw_free,
	.prepare	= fsl_dma_prepare,
	.pointer	= fsl_dma_pointer,
};

const char fsl_platform_id[] = "fsl_pcm";
EXPORT_SYMBOL_GPL(fsl_platform_id);

static struct snd_soc_platform_new fsl_dma_platform = {
	.name		= fsl_platform_id,
	.pcm_ops	= &fsl_dma_ops,
	.pcm_new	= fsl_dma_new,
	.pcm_free	= fsl_dma_free_dma_buffers,
};

static int fsl_dma_probe(struct platform_device *pdev)
{
	struct snd_soc_platform *platform;
	int ret;
	
	platform = snd_soc_new_platform(&fsl_dma_platform);
	if (!platform) {
		dev_err(&pdev->dev, "unable to allocate platform\n");
		return -ENOMEM;
	}

	ret = snd_soc_register_platform(platform, &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "could not register platform\n");
		snd_soc_free_platform(platform);
		return ret;
	}

	platform_set_drvdata(pdev, platform);

	return 0;
}

static int fsl_dma_remove(struct platform_device *pdev)
{
	struct snd_soc_platform *platform = platform_get_drvdata(pdev);

	if (platform)
		snd_soc_free_platform(platform);
	
	return 0;
}

static struct platform_driver fsl_dma_driver = {
	.driver = {
		.name		= fsl_platform_id,
		.owner		= THIS_MODULE,
	},
	.probe		= fsl_dma_probe,
	.remove		= __devexit_p(fsl_dma_remove),
};

static struct platform_device *pdev;

static __init int fsl_dma_init(void)
{
	int ret;

	printk(KERN_INFO "Freescale Elo DMA ASoC PCM driver\n");

	ret = platform_driver_register(&fsl_dma_driver);
	if (ret < 0) {
		pr_err("fsl-dma: could not register platform\n");
		return ret;
	}

	pdev = platform_device_register_simple(fsl_platform_id, 0, NULL, 0);
	if (!pdev) {
		pr_err("fsl-dma: could not register device\n");
		platform_driver_unregister(&fsl_dma_driver);
		return ret;
	}

	return 0;
}

static __exit void fsl_dma_exit(void)
{
	platform_device_unregister(pdev);
	platform_driver_unregister(&fsl_dma_driver);
}

module_init(fsl_dma_init);
module_exit(fsl_dma_exit);

MODULE_AUTHOR("Timur Tabi <timur@freescale.com>");
MODULE_DESCRIPTION("Freescale Elo DMA ASoC PCM driver");
MODULE_LICENSE("GPL");

