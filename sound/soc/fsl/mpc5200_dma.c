/*
 * Freescale MPC5200 PSC DMA
 * ALSA SoC Platform driver
 *
 * Copyright (C) 2008 Secret Lab Technologies Ltd.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/dma-mapping.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-of-simple.h>

#include <sysdev/bestcomm/bestcomm.h>
#include <sysdev/bestcomm/gen_bd.h>
#include <asm/time.h>
#include <asm/mpc52xx.h>
#include <asm/mpc52xx_psc.h>

#include "mpc5200_dma.h"

MODULE_AUTHOR("Grant Likely <grant.likely@secretlab.ca>");
MODULE_DESCRIPTION("Freescale MPC5200 PSC in DMA mode ASoC Driver");
MODULE_LICENSE("GPL");

/*
 * Interrupt handlers
 */
static irqreturn_t psc_dma_status_irq(int irq, void *_psc_dma)
{
	struct psc_dma *psc_dma = _psc_dma;
	struct mpc52xx_psc __iomem *regs = psc_dma->psc_regs;
	u16 isr;

	isr = in_be16(&regs->mpc52xx_psc_isr);

	/* Playback underrun error */
	if (psc_dma->playback.active && (isr & MPC52xx_PSC_IMR_TXEMP))
		psc_dma->stats.underrun_count++;

	/* Capture overrun error */
	if (psc_dma->capture.active && (isr & MPC52xx_PSC_IMR_ORERR))
		psc_dma->stats.overrun_count++;

	out_8(&regs->command, MPC52xx_PSC_RST_ERR_STAT);

	return IRQ_HANDLED;
}

/**
 * psc_dma_bcom_enqueue_next_buffer - Enqueue another audio buffer
 * @s: pointer to stream private data structure
 *
 * Enqueues another audio period buffer into the bestcomm queue.
 *
 * Note: The routine must only be called when there is space available in
 * the queue.  Otherwise the enqueue will fail and the audio ring buffer
 * will get out of sync
 */
static void psc_dma_bcom_enqueue_next_buffer(struct psc_dma_stream *s)
{
	struct bcom_bd *bd;

	/* Prepare and enqueue the next buffer descriptor */
	bd = bcom_prepare_next_buffer(s->bcom_task);
	bd->status = s->period_bytes;
	bd->data[0] = s->period_next_pt;
	bcom_submit_next_buffer(s->bcom_task, NULL);

	/* Update for next period */
	s->period_next_pt += s->period_bytes;
	if (s->period_next_pt >= s->period_end)
		s->period_next_pt = s->period_start;
}

static void psc_dma_bcom_enqueue_tx(struct psc_dma_stream *s)
{
	while (s->appl_ptr < s->runtime->control->appl_ptr) {

		if (bcom_queue_full(s->bcom_task))
			return;

		s->appl_ptr += s->period_size;

		psc_dma_bcom_enqueue_next_buffer(s);
	}
}

/* Bestcomm DMA irq handler */
static irqreturn_t psc_dma_bcom_irq_tx(int irq, void *_psc_dma_stream)
{
	struct psc_dma_stream *s = _psc_dma_stream;

	/* For each finished period, dequeue the completed period buffer
	 * and enqueue a new one in it's place. */
	while (bcom_buffer_done(s->bcom_task)) {
		bcom_retrieve_buffer(s->bcom_task, NULL, NULL);

		s->period_current_pt += s->period_bytes;
		if (s->period_current_pt >= s->period_end)
			s->period_current_pt = s->period_start;
	}
	psc_dma_bcom_enqueue_tx(s);

	/* If the stream is active, then also inform the PCM middle layer
	 * of the period finished event. */
	if (s->active)
		snd_pcm_period_elapsed(s->stream);

	return IRQ_HANDLED;
}

static irqreturn_t psc_dma_bcom_irq_rx(int irq, void *_psc_dma_stream)
{
	struct psc_dma_stream *s = _psc_dma_stream;

	/* For each finished period, dequeue the completed period buffer
	 * and enqueue a new one in it's place. */
	while (bcom_buffer_done(s->bcom_task)) {
		bcom_retrieve_buffer(s->bcom_task, NULL, NULL);

		s->period_current_pt += s->period_bytes;
		if (s->period_current_pt >= s->period_end)
			s->period_current_pt = s->period_start;

		psc_dma_bcom_enqueue_next_buffer(s);
	}

	/* If the stream is active, then also inform the PCM middle layer
	 * of the period finished event. */
	if (s->active)
		snd_pcm_period_elapsed(s->stream);

	return IRQ_HANDLED;
}

static int psc_dma_hw_free(struct snd_pcm_substream *substream)
{
	snd_pcm_set_runtime_buffer(substream, NULL);
	return 0;
}

/**
 * psc_dma_trigger: start and stop the DMA transfer.
 *
 * This function is called by ALSA to start, stop, pause, and resume the DMA
 * transfer of data.
 */
static int psc_dma_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct psc_dma *psc_dma = rtd->dai->cpu_dai->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct psc_dma_stream *s;
	struct mpc52xx_psc __iomem *regs = psc_dma->psc_regs;
	u16 imr;
	u8 psc_cmd;
	unsigned long flags;
	int i;

	if (substream->pstr->stream == SNDRV_PCM_STREAM_CAPTURE)
		s = &psc_dma->capture;
	else
		s = &psc_dma->playback;

	dev_dbg(psc_dma->dev, "psc_dma_trigger(substream=%p, cmd=%i)"
		" stream_id=%i\n",
		substream, cmd, substream->pstr->stream);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		s->period_bytes = frames_to_bytes(runtime,
						  runtime->period_size);
		s->period_start = virt_to_phys(runtime->dma_area);
		s->period_end = s->period_start +
				(s->period_bytes * runtime->periods);
		s->period_next_pt = s->period_start;
		s->period_current_pt = s->period_start;
		s->period_size = runtime->period_size;
		s->active = 1;

		s->runtime = runtime;
		s->appl_ptr = s->runtime->control->appl_ptr - (runtime->period_size * runtime->periods);

		/* Fill up the bestcomm bd queue and enable DMA.
		 * This will begin filling the PSC's fifo. */
		if (substream->pstr->stream == SNDRV_PCM_STREAM_CAPTURE) {
			bcom_gen_bd_rx_reset(s->bcom_task);
			for (i = 0; i < runtime->periods; i++)
				if (!bcom_queue_full(s->bcom_task))
					psc_dma_bcom_enqueue_next_buffer(s);
		} else {
			bcom_gen_bd_tx_reset(s->bcom_task);
			psc_dma_bcom_enqueue_tx(s);
		}

		bcom_enable(s->bcom_task);

		/* Due to errata in the dma mode; need to line up enabling
		 * the transmitter with a transition on the frame sync
		 * line */

		spin_lock_irqsave(&psc_dma->lock, flags);
		/* first make sure it is low */
		while ((in_8(&regs->ipcr_acr.ipcr) & 0x80) != 0)
			;
		/* then wait for the transition to high */
		while ((in_8(&regs->ipcr_acr.ipcr) & 0x80) == 0)
			;
		/* Finally, enable the PSC.
		 * Receiver must always be enabled; even when we only want
		 * transmit.  (see 15.3.2.3 of MPC5200B User's Guide) */
		psc_cmd = MPC52xx_PSC_RX_ENABLE;
		if (substream->pstr->stream == SNDRV_PCM_STREAM_PLAYBACK)
			psc_cmd |= MPC52xx_PSC_TX_ENABLE;
		//out_8(&regs->command, psc_cmd);

		out_8(&regs->command, MPC52xx_PSC_RST_ERR_STAT);

		spin_unlock_irqrestore(&psc_dma->lock, flags);

		break;

	case SNDRV_PCM_TRIGGER_STOP:
		s->active = 0;

		bcom_disable(s->bcom_task);
		if (substream->pstr->stream == SNDRV_PCM_STREAM_CAPTURE)
			bcom_gen_bd_rx_reset(s->bcom_task);
		else
			bcom_gen_bd_tx_reset(s->bcom_task);

		break;

	default:
		dev_dbg(psc_dma->dev, "invalid command\n");
		return -EINVAL;
	}

	/* Update interrupt enable settings */
	imr = 0;
	if (psc_dma->playback.active)
		imr |= MPC52xx_PSC_IMR_TXEMP;
	if (psc_dma->capture.active)
		imr |= MPC52xx_PSC_IMR_ORERR;
	out_be16(&regs->isr_imr.imr, psc_dma->imr | imr);

	return 0;
}


/* ---------------------------------------------------------------------
 * The PSC DMA 'ASoC platform' driver
 *
 * Can be referenced by an 'ASoC machine' driver
 * This driver only deals with the audio bus; it doesn't have any
 * interaction with the attached codec
 */

static const struct snd_pcm_hardware psc_dma_hardware = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_BATCH,
	.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_BE |
		SNDRV_PCM_FMTBIT_S24_BE | SNDRV_PCM_FMTBIT_S32_BE,
	.rate_min = 8000,
	.rate_max = 48000,
	.channels_min = 1,
	.channels_max = 2,
	.period_bytes_max	= 1024 * 1024,
	.period_bytes_min	= 32,
	.periods_min		= 2,
	.periods_max		= 256,
	.buffer_bytes_max	= 2 * 1024 * 1024,
	.fifo_size		= 512,
};

static int psc_dma_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct psc_dma *psc_dma = rtd->dai->cpu_dai->private_data;
	struct psc_dma_stream *s;
	int rc;

	dev_dbg(psc_dma->dev, "psc_dma_open(substream=%p)\n", substream);

	if (substream->pstr->stream == SNDRV_PCM_STREAM_CAPTURE)
		s = &psc_dma->capture;
	else
		s = &psc_dma->playback;

	snd_soc_set_runtime_hwparams(substream, &psc_dma_hardware);

	rc = snd_pcm_hw_constraint_integer(runtime,
		SNDRV_PCM_HW_PARAM_PERIODS);
	if (rc < 0) {
		dev_err(substream->pcm->card->dev, "invalid buffer size\n");
		return rc;
	}

	if (!psc_dma->playback.active &&
	    !psc_dma->capture.active) {
		/* Setup the IRQs */
	}

	s->stream = substream;
	return 0;
}

static int psc_dma_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct psc_dma *psc_dma = rtd->dai->cpu_dai->private_data;
	struct psc_dma_stream *s;

	dev_dbg(psc_dma->dev, "psc_dma_close(substream=%p)\n", substream);

	if (substream->pstr->stream == SNDRV_PCM_STREAM_CAPTURE)
		s = &psc_dma->capture;
	else
		s = &psc_dma->playback;

	if (!psc_dma->playback.active &&
	    !psc_dma->capture.active) {

		/* Disable all interrupts and reset the PSC */
		out_be16(&psc_dma->psc_regs->isr_imr.imr, psc_dma->imr);
		//out_8(&psc_dma->psc_regs->command, 3 << 4); /* reset tx */
		//out_8(&psc_dma->psc_regs->command, 2 << 4); /* reset rx */
		//out_8(&psc_dma->psc_regs->command, 1 << 4); /* reset mode */
		//out_8(&psc_dma->psc_regs->command, 4 << 4); /* reset error */

	}

	s->stream = NULL;
	return 0;
}

static snd_pcm_uframes_t
psc_dma_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct psc_dma *psc_dma = rtd->dai->cpu_dai->private_data;
	struct psc_dma_stream *s;
	dma_addr_t count;
	snd_pcm_uframes_t frames;

	if (substream->pstr->stream == SNDRV_PCM_STREAM_CAPTURE)
		s = &psc_dma->capture;
	else
		s = &psc_dma->playback;

	count = s->period_current_pt - s->period_start;

	frames = bytes_to_frames(substream->runtime, count);
	return frames;
}

static int
psc_dma_hw_params(struct snd_pcm_substream *substream,
			 struct snd_pcm_hw_params *params)
{
  	snd_pcm_set_runtime_buffer(substream, &substream->dma_buffer);

  	return 0;
}

static struct snd_pcm_ops psc_dma_ops = {
	.open		= psc_dma_open,
	.close		= psc_dma_close,
	.hw_free	= psc_dma_hw_free,
	.ioctl		= snd_pcm_lib_ioctl,
	.pointer	= psc_dma_pointer,
	.trigger	= psc_dma_trigger,
	.hw_params	= psc_dma_hw_params,
};

static u64 psc_dma_dmamask = 0xffffffff;
static int psc_dma_new(struct snd_card *card, struct snd_soc_dai *dai,
			   struct snd_pcm *pcm)
{
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct psc_dma *psc_dma = rtd->dai->cpu_dai->private_data;
	size_t size = psc_dma_hardware.buffer_bytes_max;
	int rc = 0;

	dev_dbg(rtd->socdev->dev, "psc_dma_new(card=%p, dai=%p, pcm=%p)\n",
		card, dai, pcm);

	if (!card->dev->dma_mask)
		card->dev->dma_mask = &psc_dma_dmamask;
	if (!card->dev->coherent_dma_mask)
		card->dev->coherent_dma_mask = 0xffffffff;

	if (pcm->streams[0].substream) {
		rc = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, pcm->card->dev, size,
					&pcm->streams[0].substream->dma_buffer);
		if (rc)
			goto playback_alloc_err;
	}

	if (pcm->streams[1].substream) {
		rc = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV, pcm->card->dev, size,
					&pcm->streams[1].substream->dma_buffer);
		if (rc)
			goto capture_alloc_err;
	}

	if (rtd->socdev->card->codec->ac97)
		rtd->socdev->card->codec->ac97->private_data = psc_dma;

	return 0;

 capture_alloc_err:
	if (pcm->streams[0].substream)
		snd_dma_free_pages(&pcm->streams[0].substream->dma_buffer);
 playback_alloc_err:
	dev_err(card->dev, "Cannot allocate buffer(s)\n");
	return -ENOMEM;
}

static void psc_dma_free(struct snd_pcm *pcm)
{
	struct snd_soc_pcm_runtime *rtd = pcm->private_data;
	struct snd_pcm_substream *substream;
	int stream;

	dev_dbg(rtd->socdev->dev, "psc_dma_free(pcm=%p)\n", pcm);

	for (stream = 0; stream < 2; stream++) {
		substream = pcm->streams[stream].substream;
		if (substream) {
			snd_dma_free_pages(&substream->dma_buffer);
			substream->dma_buffer.area = NULL;
			substream->dma_buffer.addr = 0;
		}
	}
}

struct snd_soc_platform mpc5200_audio_dma_platform = {
	.name		= "mpc5200-psc-audio",
	.pcm_ops	= &psc_dma_ops,
	.pcm_new	= &psc_dma_new,
	.pcm_free	= &psc_dma_free,
};
EXPORT_SYMBOL_GPL(mpc5200_audio_dma_platform);

/* ---------------------------------------------------------------------
 * Sysfs attributes for debugging
 */

static ssize_t psc_dma_status_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct psc_dma *psc_dma = dev_get_drvdata(dev);

	return sprintf(buf, "status=%.4x sicr=%.8x rfnum=%i rfstat=0x%.4x "
			"tfnum=%i tfstat=0x%.4x\n",
			in_be16(&psc_dma->psc_regs->sr_csr.status),
			in_be32(&psc_dma->psc_regs->sicr),
			in_be16(&psc_dma->fifo_regs->rfnum) & 0x1ff,
			in_be16(&psc_dma->fifo_regs->rfstat),
			in_be16(&psc_dma->fifo_regs->tfnum) & 0x1ff,
			in_be16(&psc_dma->fifo_regs->tfstat));
}

static int *psc_dma_get_stat_attr(struct psc_dma *psc_dma, const char *name)
{
	if (strcmp(name, "playback_underrun") == 0)
		return &psc_dma->stats.underrun_count;
	if (strcmp(name, "capture_overrun") == 0)
		return &psc_dma->stats.overrun_count;

	return NULL;
}

static ssize_t psc_dma_stat_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct psc_dma *psc_dma = dev_get_drvdata(dev);
	int *attrib;

	attrib = psc_dma_get_stat_attr(psc_dma, attr->attr.name);
	if (!attrib)
		return 0;

	return sprintf(buf, "%i\n", *attrib);
}

static ssize_t psc_dma_stat_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	struct psc_dma *psc_dma = dev_get_drvdata(dev);
	int *attrib;

	attrib = psc_dma_get_stat_attr(psc_dma, attr->attr.name);
	if (!attrib)
		return 0;

	*attrib = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR(status, 0644, psc_dma_status_show, NULL);
static DEVICE_ATTR(playback_underrun, 0644, psc_dma_stat_show,
			psc_dma_stat_store);
static DEVICE_ATTR(capture_overrun, 0644, psc_dma_stat_show,
			psc_dma_stat_store);


int mpc5200_audio_dma_create(struct of_device *op, struct snd_soc_dai *template, int tsize)
{
	phys_addr_t fifo;
	struct psc_dma *psc_dma;
	struct resource res;
	int i, nDAI, size, psc_id, irq, rc;
	const __be32 *prop;
	void __iomem *regs;

	/* Get the PSC ID */
	prop = of_get_property(op->node, "cell-index", &size);
	if (!prop || size < sizeof *prop)
		return -ENODEV;
	psc_id = be32_to_cpu(*prop);

	/* Fetch the registers and IRQ of the PSC */
	irq = irq_of_parse_and_map(op->node, 0);
	if (of_address_to_resource(op->node, 0, &res)) {
		dev_err(&op->dev, "Missing reg property\n");
		return -ENODEV;
	}
	regs = ioremap(res.start, 1 + res.end - res.start);
	if (!regs) {
		dev_err(&op->dev, "Could not map registers\n");
		return -ENODEV;
	}

	/* Allocate and initialize the driver private data */
	psc_dma = kzalloc(sizeof *psc_dma, GFP_KERNEL);
	if (!psc_dma) {
		iounmap(regs);
		return -ENOMEM;
	}

	spin_lock_init(&psc_dma->lock);
	psc_dma->irq = irq;
	psc_dma->psc_regs = regs;
	psc_dma->fifo_regs = regs + sizeof *psc_dma->psc_regs;
	psc_dma->dev = &op->dev;
	psc_dma->playback.psc_dma = psc_dma;
	psc_dma->capture.psc_dma = psc_dma;
	snprintf(psc_dma->name, sizeof psc_dma->name, "PSC%u", psc_id+1);

	/* Fill out the CPU DAI structure */
	nDAI = min(tsize, SOC_OF_SIMPLE_MAX_DAI);
	for (i = 0; i < nDAI; i++) {
		memcpy(&psc_dma->dai[i], &template[i], sizeof(struct snd_soc_dai));
		psc_dma->dai[i].private_data = psc_dma;
		snprintf(psc_dma->stream_name[i], PSC_STREAM_NAME_LEN, template[i].name, psc_dma->name);
		psc_dma->dai[i].name = psc_dma->stream_name[i];
		psc_dma->dai[i].id = psc_id;
	}

	/* Find the address of the fifo data registers and setup the
	 * DMA tasks */
	fifo = res.start + offsetof(struct mpc52xx_psc, buffer.buffer_32);
	psc_dma->capture.bcom_task =
		bcom_psc_gen_bd_rx_init(psc_id, 10, fifo, 512);
	psc_dma->playback.bcom_task =
		bcom_psc_gen_bd_tx_init(psc_id, 10, fifo);
	if (!psc_dma->capture.bcom_task ||
	    !psc_dma->playback.bcom_task) {
		dev_err(&op->dev, "Could not allocate bestcomm tasks\n");
		iounmap(regs);
		kfree(psc_dma);
		return -ENODEV;
	}

	/* Disable all interrupts and reset the PSC */
	out_be16(&psc_dma->psc_regs->isr_imr.imr, psc_dma->imr);
	out_8(&psc_dma->psc_regs->command, MPC52xx_PSC_RST_RX); /* reset receiver */
	out_8(&psc_dma->psc_regs->command, MPC52xx_PSC_RST_TX); /* reset transmitter */
	out_8(&psc_dma->psc_regs->command, MPC52xx_PSC_RST_ERR_STAT); /* reset error */
	out_8(&psc_dma->psc_regs->command, MPC52xx_PSC_SEL_MODE_REG_1); /* reset mode */

	/* Set up mode register;
	 * First write: RxRdy (FIFO Alarm) generates rx FIFO irq
	 * Second write: register Normal mode for non loopback
	 */
	out_8(&psc_dma->psc_regs->mode, 0);
	out_8(&psc_dma->psc_regs->mode, 0);

	/* Set the TX and RX fifo alarm thresholds */
	out_be16(&psc_dma->fifo_regs->rfalarm, 0x100);
	out_8(&psc_dma->fifo_regs->rfcntl, 0x4);
	out_be16(&psc_dma->fifo_regs->tfalarm, 0x100);
	out_8(&psc_dma->fifo_regs->tfcntl, 0x7);

	/* Lookup the IRQ numbers */
	psc_dma->playback.irq =
		bcom_get_task_irq(psc_dma->playback.bcom_task);
	psc_dma->capture.irq =
		bcom_get_task_irq(psc_dma->capture.bcom_task);

	rc = request_irq(psc_dma->irq, &psc_dma_status_irq, IRQF_SHARED,
			 "psc-dma-status", psc_dma);
	rc |= request_irq(psc_dma->capture.irq,
			  &psc_dma_bcom_irq_rx, IRQF_SHARED,
			  "psc-dma-capture", &psc_dma->capture);
	rc |= request_irq(psc_dma->playback.irq,
			  &psc_dma_bcom_irq_tx, IRQF_SHARED,
			  "psc-dma-playback", &psc_dma->playback);
	if (rc) {
		free_irq(psc_dma->irq, psc_dma);
		free_irq(psc_dma->capture.irq,
			 &psc_dma->capture);
		free_irq(psc_dma->playback.irq,
			 &psc_dma->playback);
		return -ENODEV;
	}

	/* Save what we've done so it can be found again later */
	dev_set_drvdata(&op->dev, psc_dma);

	/* Register the SYSFS files */
	rc = device_create_file(psc_dma->dev, &dev_attr_status);
	rc |= device_create_file(psc_dma->dev, &dev_attr_capture_overrun);
	rc |= device_create_file(psc_dma->dev, &dev_attr_playback_underrun);
	if (rc)
		dev_info(psc_dma->dev, "error creating sysfs files\n");

	rc = snd_soc_register_dais(psc_dma->dai, nDAI);
	if (rc != 0) {
		pr_err("Failed to register DAI\n");
		return 0;
	}

	/* Tell the ASoC OF helpers about it */
	of_snd_soc_register_cpu_dai(op->node, psc_dma->dai, nDAI);

	return 0;
}
EXPORT_SYMBOL_GPL(mpc5200_audio_dma_create);

int mpc5200_audio_dma_destroy(struct of_device *op)
{
	struct psc_dma *psc_dma = dev_get_drvdata(&op->dev);

	dev_dbg(&op->dev, "mpc5200_audio_dma_destroy()\n");

	bcom_gen_bd_rx_release(psc_dma->capture.bcom_task);
	bcom_gen_bd_tx_release(psc_dma->playback.bcom_task);

	/* Release irqs */
	free_irq(psc_dma->irq, psc_dma);
	free_irq(psc_dma->capture.irq, &psc_dma->capture);
	free_irq(psc_dma->playback.irq, &psc_dma->playback);

	iounmap(psc_dma->psc_regs);
	iounmap(psc_dma->fifo_regs);
	kfree(psc_dma);
	dev_set_drvdata(&op->dev, NULL);

	return 0;
}
EXPORT_SYMBOL_GPL(mpc5200_audio_dma_destroy);

static int __init mpc5200_soc_platform_init(void)
{
	/* Tell the ASoC OF helpers about it */
	of_snd_soc_register_platform(&mpc5200_audio_dma_platform);
	return snd_soc_register_platform(&mpc5200_audio_dma_platform);
}
module_init(mpc5200_soc_platform_init);

static void __exit mpc5200_soc_platform_exit(void)
{
	snd_soc_unregister_platform(&mpc5200_audio_dma_platform);
}
module_exit(mpc5200_soc_platform_exit);

