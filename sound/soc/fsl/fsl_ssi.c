/*
 * Freescale SSI ALSA SoC Digital Audio Interface (DAI) driver
 *
 * Author: Timur Tabi <timur@freescale.com>
 *
 * Copyright 2007-2008 Freescale Semiconductor, Inc.  This file is licensed
 * under the terms of the GNU General Public License version 2.  This
 * program is licensed "as is" without any warranty of any kind, whether
 * express or implied.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <asm/immap_86xx.h>

#include "fsl_dma.h"
#include "fsl_ssi.h"

/**
 * FSLSSI_I2S_RATES: sample rates supported by the I2S
 *
 * This driver currently only supports the SSI running in I2S slave mode,
 * which means the codec determines the sample rate.  Therefore, we tell
 * ALSA that we support all rates and let the codec driver decide what rates
 * are really supported.
 */
#define FSLSSI_I2S_RATES (SNDRV_PCM_RATE_5512 | SNDRV_PCM_RATE_8000_192000 | \
			  SNDRV_PCM_RATE_CONTINUOUS)

/**
 * FSLSSI_I2S_FORMATS: audio formats supported by the SSI
 *
 * This driver currently only supports the SSI running in I2S slave mode.
 *
 * The SSI has a limitation in that the samples must be in the same byte
 * order as the host CPU.  This is because when multiple bytes are written
 * to the STX register, the bytes and bits must be written in the same
 * order.  The STX is a shift register, so all the bits need to be aligned
 * (bit-endianness must match byte-endianness).  Processors typically write
 * the bits within a byte in the same order that the bytes of a word are
 * written in.  So if the host CPU is big-endian, then only big-endian
 * samples will be written to STX properly.
 */
#ifdef __BIG_ENDIAN
#define FSLSSI_I2S_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_BE | \
	 SNDRV_PCM_FMTBIT_S18_3BE | SNDRV_PCM_FMTBIT_S20_3BE | \
	 SNDRV_PCM_FMTBIT_S24_3BE | SNDRV_PCM_FMTBIT_S24_BE)
#else
#define FSLSSI_I2S_FORMATS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE | \
	 SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	 SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S24_LE)
#endif

/**
 * fsl_ssi_isr: SSI interrupt handler
 *
 * Although it's possible to use the interrupt handler to send and receive
 * data to/from the SSI, we use the DMA instead.  Programming is more
 * complicated, but the performance is much better.
 *
 * This interrupt handler is used only to gather statistics.
 *
 * @irq: IRQ of the SSI device
 * @dev_id: pointer to the ssi_info structure for this SSI device
 */
static irqreturn_t fsl_ssi_isr(int irq, void *dev_id)
{
	struct fsl_ssi_info *ssi_info = dev_id;
	struct ccsr_ssi __iomem *ssi = ssi_info->ssi;
	irqreturn_t ret = IRQ_NONE;
	__be32 sisr;
	__be32 sisr2 = 0;

	/* We got an interrupt, so read the status register to see what we
	   were interrupted for.  We mask it with the Interrupt Enable register
	   so that we only check for events that we're interested in.
	 */
	sisr = in_be32(&ssi->sisr) & in_be32(&ssi->sier);

	if (sisr & CCSR_SSI_SISR_RFRC) {
		ssi_info->stats.rfrc++;
		sisr2 |= CCSR_SSI_SISR_RFRC;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_TFRC) {
		ssi_info->stats.tfrc++;
		sisr2 |= CCSR_SSI_SISR_TFRC;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_CMDAU) {
		ssi_info->stats.cmdau++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_CMDDU) {
		ssi_info->stats.cmddu++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_RXT) {
		ssi_info->stats.rxt++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_RDR1) {
		ssi_info->stats.rdr1++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_RDR0) {
		ssi_info->stats.rdr0++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_TDE1) {
		ssi_info->stats.tde1++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_TDE0) {
		ssi_info->stats.tde0++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_ROE1) {
		ssi_info->stats.roe1++;
		sisr2 |= CCSR_SSI_SISR_ROE1;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_ROE0) {
		ssi_info->stats.roe0++;
		sisr2 |= CCSR_SSI_SISR_ROE0;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_TUE1) {
		ssi_info->stats.tue1++;
		sisr2 |= CCSR_SSI_SISR_TUE1;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_TUE0) {
		ssi_info->stats.tue0++;
		sisr2 |= CCSR_SSI_SISR_TUE0;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_TFS) {
		ssi_info->stats.tfs++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_RFS) {
		ssi_info->stats.rfs++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_TLS) {
		ssi_info->stats.tls++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_RLS) {
		ssi_info->stats.rls++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_RFF1) {
		ssi_info->stats.rff1++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_RFF0) {
		ssi_info->stats.rff0++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_TFE1) {
		ssi_info->stats.tfe1++;
		ret = IRQ_HANDLED;
	}

	if (sisr & CCSR_SSI_SISR_TFE0) {
		ssi_info->stats.tfe0++;
		ret = IRQ_HANDLED;
	}

	/* Clear the bits that we set */
	if (sisr2)
		out_be32(&ssi->sisr, sisr2);

	return ret;
}

/**
 * fsl_ssi_startup: create a new substream
 *
 * This is the first function called when a stream is opened.
 *
 * If this is the first stream open, then grab the IRQ and program most of
 * the SSI registers.
 */
static int fsl_ssi_startup(struct snd_pcm_substream *substream,
	struct snd_soc_dai *cpu_dai)
{
	struct fsl_ssi_info *ssi_info = cpu_dai->private_data;

	/*
	 * If this is the first stream opened, then request the IRQ
	 * and initialize the SSI registers.
	 */
	if (!ssi_info->playback && !ssi_info->capture) {
		struct ccsr_ssi __iomem *ssi = ssi_info->ssi;
		int ret;

		ret = request_irq(ssi_info->irq, fsl_ssi_isr, 0,
				  ssi_info->name, ssi_info);
		if (ret < 0) {
			dev_err(substream->pcm->card->dev,
				"could not claim irq %u\n", ssi_info->irq);
			return ret;
		}

		/*
		 * Section 16.5 of the MPC8610 reference manual says that the
		 * SSI needs to be disabled before updating the registers we set
		 * here.
		 */
		clrbits32(&ssi->scr, CCSR_SSI_SCR_SSIEN);

		/*
		 * Program the SSI into I2S Slave Non-Network Synchronous mode.
		 * Also enable the transmit and receive FIFO.  Make sure TE and
		 * RE are disabled.
		 *
		 * FIXME: Little-endian samples require a different shift dir
		 */
		clrsetbits_be32(&ssi->scr, CCSR_SSI_SCR_I2S_MODE_MASK |
			CCSR_SSI_SCR_TE | CCSR_SSI_SCR_RE,
			CCSR_SSI_SCR_TFR_CLK_DIS |
			CCSR_SSI_SCR_I2S_MODE_SLAVE | CCSR_SSI_SCR_SYN);

		out_be32(&ssi->stcr,
			 CCSR_SSI_STCR_TXBIT0 | CCSR_SSI_STCR_TFEN0 |
			 CCSR_SSI_STCR_TFSI | CCSR_SSI_STCR_TEFS |
			 CCSR_SSI_STCR_TSCKP);

		out_be32(&ssi->srcr,
			 CCSR_SSI_SRCR_RXBIT0 | CCSR_SSI_SRCR_RFEN0 |
			 CCSR_SSI_SRCR_RFSI | CCSR_SSI_SRCR_REFS |
			 CCSR_SSI_SRCR_RSCKP);

		/*
		 * The DC and PM bits are only used if the SSI is the clock
		 * master.
		 */

		/* 4. Enable the interrupts and DMA requests */
		out_be32(&ssi->sier,
			 CCSR_SSI_SIER_TFRC_EN | CCSR_SSI_SIER_TDMAE |
			 CCSR_SSI_SIER_TIE | CCSR_SSI_SIER_TUE0_EN |
			 CCSR_SSI_SIER_TUE1_EN | CCSR_SSI_SIER_RFRC_EN |
			 CCSR_SSI_SIER_RDMAE | CCSR_SSI_SIER_RIE |
			 CCSR_SSI_SIER_ROE0_EN | CCSR_SSI_SIER_ROE1_EN);

		/*
		 * Set the watermark for transmit FIFI 0 and receive FIFO 0. We
		 * don't use FIFO 1.  Since the SSI only supports stereo, the
		 * watermark should never be an odd number.
		 */
		out_be32(&ssi->sfcsr,
			 CCSR_SSI_SFCSR_TFWM0(6) | CCSR_SSI_SFCSR_RFWM0(2));

		spin_unlock(&ssi_info->lock);

		/*
		 * We keep the SSI disabled because if we enable it, then the
		 * DMA controller will start.  It's not supposed to start until
		 * the SCR.TE (or SCR.RE) bit is set, but it does anyway.  The
		 * DMA controller will transfer one "BWC" of data (i.e. the
		 * amount of data that the MR.BWC bits are set to).  The reason
		 * this is bad is because at this point, the PCM driver has not
		 * finished initializing the DMA controller.
		 */
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ssi_info->playback = 1;
	else
		ssi_info->capture = 1;

	cpu_dai->dma_data = ssi_info->dma_info;

	return 0;
}

/**
 * fsl_ssi_prepare: prepare the SSI.
 *
 * Most of the SSI registers have been programmed in the startup function,
 * but the word length must be programmed here.  Unfortunately, programming
 * the SxCCR.WL bits requires the SSI to be temporarily disabled.  This can
 * cause a problem with supporting simultaneous playback and capture.  If
 * the SSI is already playing a stream, then that stream may be temporarily
 * stopped when you start capture.
 *
 * Note: The SxCCR.DC and SxCCR.PM bits are only used if the SSI is the
 * clock master.
 */
static int fsl_ssi_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai *cpu_dai)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct fsl_ssi_info *ssi_info = cpu_dai->private_data;
	struct ccsr_ssi __iomem *ssi = ssi_info->ssi;
	uint32_t wl;
	u32 scr;
	u32 sxccr;

	wl = CCSR_SSI_SxCCR_WL(snd_pcm_format_width(runtime->format));

	spin_lock(&ssi_info->lock);

	scr = in_be32(&ssi->scr);

	/* If the SSI is currently enabled, then we want to keep it disabled for
	 * as short as time as possible, so we pre-calculate all the values for
	 * the appropriate registers, and then program them rapidly.
	 */

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		sxccr = (in_be32(&ssi->stccr) & ~CCSR_SSI_SxCCR_WL_MASK) | wl;
		out_be32(&ssi->scr, scr & ~CCSR_SSI_SCR_SSIEN);
		out_be32(&ssi->stccr, sxccr);
		out_be32(&ssi->scr, scr);
	} else {
		sxccr = (in_be32(&ssi->srccr) & ~CCSR_SSI_SxCCR_WL_MASK) | wl;
		out_be32(&ssi->scr, scr & ~CCSR_SSI_SCR_SSIEN);
		out_be32(&ssi->srccr, sxccr);
		out_be32(&ssi->scr, scr);
	}

	spin_unlock(&ssi_info->lock);

	return 0;
}

/**
 * fsl_ssi_trigger: start and stop the DMA transfer.
 *
 * This function is called by ALSA to start, stop, pause, and resume the DMA
 * transfer of data.
 *
 * The DMA channel is in external master start and pause mode, which
 * means the SSI completely controls the flow of data.
 */
static int fsl_ssi_trigger(struct snd_pcm_substream *substream, int cmd,
	struct snd_soc_dai *cpu_dai)
{

	struct fsl_ssi_info *ssi_info = cpu_dai->private_data;
	struct ccsr_ssi __iomem *ssi = ssi_info->ssi;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock(&ssi_info->lock);

		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			setbits32(&ssi->scr,
				CCSR_SSI_SCR_SSIEN | CCSR_SSI_SCR_TE);
		else {
			setbits32(&ssi->scr,
				CCSR_SSI_SCR_SSIEN | CCSR_SSI_SCR_RE);

			/*
			 * I think we need this delay to allow time for the SSI
			 * to put data into its FIFO.  Without it, ALSA starts
			 * to complain about overruns.
			 */
			mdelay(1);
		}

		spin_unlock(&ssi_info->lock);
		break;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			clrbits32(&ssi->scr, CCSR_SSI_SCR_TE);
		else
			clrbits32(&ssi->scr, CCSR_SSI_SCR_RE);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * fsl_ssi_shutdown: shutdown the SSI
 *
 * Shutdown the SSI if there are no other substreams open.
 */
static void fsl_ssi_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *cpu_dai)
{
	struct fsl_ssi_info *ssi_info = cpu_dai->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		ssi_info->playback = 0;
	else
		ssi_info->capture = 0;

	/*
	 * If this is the last active substream, disable the SSI and release
	 * the IRQ.
	 */
	if (!ssi_info->playback && !ssi_info->capture) {
		struct ccsr_ssi __iomem *ssi = ssi_info->ssi;

		clrbits32(&ssi->scr, CCSR_SSI_SCR_SSIEN);

		free_irq(ssi_info->irq, ssi_info);
	}
}

/**
 * fsl_ssi_set_sysclk: set the clock frequency and direction
 *
 * This function is called by the fabric driver to tell us what the clock
 * frequency and direction are.
 *
 * Currently, we only support operating as a clock slave (SND_SOC_CLOCK_IN),
 * and we don't care about the frequency.  Return an error if the direction
 * is not SND_SOC_CLOCK_IN.
 *
 * @clk_id: reserved, should be zero
 * @freq: the frequency of the given clock ID, currently ignored
 * @dir: SND_SOC_CLOCK_IN (clock slave) or SND_SOC_CLOCK_OUT (clock master)
 */
static int fsl_ssi_set_sysclk(struct snd_soc_dai *dai, int clk_id,
	unsigned int freq, int dir)
{

	return (dir == SND_SOC_CLOCK_IN) ? 0 : -EINVAL;
}

/**
 * fsl_ssi_set_fmt: set the serial format.
 *
 * This function is called by the soc_card driver to tell us what serial
 * format to use.
 *
 * Currently, we only support I2S mode.  Return an error if the format is
 * not SND_SOC_DAIFMT_I2S.
 *
 * @format: one of SND_SOC_DAIFMT_xxx
 */
static int fsl_ssi_set_fmt(struct snd_soc_dai *dai, unsigned int format)
{
	return (format == SND_SOC_DAIFMT_I2S) ? 0 : -EINVAL;
}

/**
 * fsl_sysfs_ssi_show: display SSI statistics
 *
 * Display the statistics for the current SSI device.
 */
static ssize_t fsl_sysfs_ssi_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct fsl_ssi_info *ssi_info =
	container_of(attr, struct fsl_ssi_info, dev_attr);
	ssize_t length;

	length = sprintf(buf, "rfrc=%u", ssi_info->stats.rfrc);
	length += sprintf(buf + length, "\ttfrc=%u", ssi_info->stats.tfrc);
	length += sprintf(buf + length, "\tcmdau=%u", ssi_info->stats.cmdau);
	length += sprintf(buf + length, "\tcmddu=%u", ssi_info->stats.cmddu);
	length += sprintf(buf + length, "\trxt=%u", ssi_info->stats.rxt);
	length += sprintf(buf + length, "\trdr1=%u", ssi_info->stats.rdr1);
	length += sprintf(buf + length, "\trdr0=%u", ssi_info->stats.rdr0);
	length += sprintf(buf + length, "\ttde1=%u", ssi_info->stats.tde1);
	length += sprintf(buf + length, "\ttde0=%u", ssi_info->stats.tde0);
	length += sprintf(buf + length, "\troe1=%u", ssi_info->stats.roe1);
	length += sprintf(buf + length, "\troe0=%u", ssi_info->stats.roe0);
	length += sprintf(buf + length, "\ttue1=%u", ssi_info->stats.tue1);
	length += sprintf(buf + length, "\ttue0=%u", ssi_info->stats.tue0);
	length += sprintf(buf + length, "\ttfs=%u", ssi_info->stats.tfs);
	length += sprintf(buf + length, "\trfs=%u", ssi_info->stats.rfs);
	length += sprintf(buf + length, "\ttls=%u", ssi_info->stats.tls);
	length += sprintf(buf + length, "\trls=%u", ssi_info->stats.rls);
	length += sprintf(buf + length, "\trff1=%u", ssi_info->stats.rff1);
	length += sprintf(buf + length, "\trff0=%u", ssi_info->stats.rff0);
	length += sprintf(buf + length, "\ttfe1=%u", ssi_info->stats.tfe1);
	length += sprintf(buf + length, "\ttfe0=%u\n", ssi_info->stats.tfe0);

	return length;
}

static struct snd_soc_dai_caps playback = {
	/* The SSI does not support monaural audio. */
	.channels_min	= 2,
	.channels_max	= 2,
	.rates		= FSLSSI_I2S_RATES,
	.formats	= FSLSSI_I2S_FORMATS,
};

static struct snd_soc_dai_caps capture = {
	.channels_min	= 2,
	.channels_max	= 2,
	.rates		= FSLSSI_I2S_RATES,
	.formats	= FSLSSI_I2S_FORMATS,
};

static struct snd_soc_dai_ops ops = {
	.startup	= fsl_ssi_startup,
	.prepare	= fsl_ssi_prepare,
	.shutdown	= fsl_ssi_shutdown,
	.trigger	= fsl_ssi_trigger,

	.set_sysclk	= fsl_ssi_set_sysclk,
	.set_fmt	= fsl_ssi_set_fmt,
};

/**
 * find_dma_node: find the DMA node to use in a legacy device tree
 *
 * This function returns a device node matching a given DMA channel on a
 * given DMA controller.
 *
 * The original device tree for the MPC8610 HPCD did not include phandles
 * from the SSI nodes to the DMA nodes.  To support these device trees, we
 * declare that SSIx will use DMA channels 0 and 1 of DMA controller DMAx.
 */
static struct device_node *find_dma_node(unsigned int controller,
	unsigned int channel)
{
	struct device_node *np = NULL;
	struct device_node *np2 = NULL;
	const uint32_t *iprop;

	for_each_compatible_node(np, NULL, "fsl,eloplus-dma") {
		iprop = of_get_property(np, "cell-index", NULL);
		if (!iprop) {
			pr_err("fsl-ssi: cell-index property not found\n");
			return NULL;
		}
		if (*iprop == controller)
			break;
	}

	if (!np) {
		pr_err("fsl-ssi: cannot find node for DMA controller %u\n",
			controller);
		return NULL;
	}

	for_each_child_of_node(np, np2) {
		if (!of_device_is_compatible(np2, "fsl,eloplus-dma-channel"))
			continue;

		iprop = of_get_property(np2, "cell-index", NULL);
		if (!iprop) {
			pr_err("fsl-ssi: cell-index property not found\n");
			return NULL;
		}
		if (*iprop == channel)
			break;
	}

	if (!np2) {
		pr_err("fsl-ssi: cannot find node for DMA channel %u\n",
			channel);
		return NULL;
	}

	of_node_put(np);

	return np2;
}

/**
 * Initialize a dma_info structure from a pointer to a DMA node
 */
static int get_dma_info(struct device_node *np, struct fsl_dma_info *dma_info)
{
	const uint32_t *iprop;

	if (!np)
		return 0;

	iprop = of_get_property(of_get_parent(np), "cell-index", NULL);
	if (!iprop)
		return 0;

	dma_info->controller_id = *iprop;

	iprop = of_get_property(np, "cell-index", NULL);
	if (!iprop)
		return 0;

	dma_info->channel_id = *iprop;

	dma_info->channel = of_iomap(np, 0);
	dma_info->irq = irq_of_parse_and_map(np, 0);

	return 1;
}

static int fsl_ssi_probe(struct of_device *ofdev,
	const struct of_device_id *match)
{
	struct device_node *np = ofdev->node;
	struct device_node *codec_np = NULL;
	struct device_node *dma_np[2] = {NULL, NULL};
	const phandle *codec_ph;
	const phandle *dma_ph;
	const char *sprop;
	const uint32_t *iprop;
	struct resource res;

	struct fsl_ssi_info *ssi_info;
	int ret = -ENODEV;

	struct snd_soc_dai_new dai_template;

	ssi_info = kzalloc(sizeof(struct fsl_ssi_info), GFP_KERNEL);
	if (!ssi_info) {
		dev_err(&ofdev->dev, "cannot allocate ssi_info\n");
		return -ENOMEM;
	}

	/*
	 * We are only interested in SSIs with a codec phandle in them, so let's
	 * make sure this SSI has one.
	 */
	codec_ph = of_get_property(np, "codec-handle", NULL);
	if (!codec_ph) {
		dev_dbg(&ofdev->dev, "no codec handle for this SSI\n");
		goto error;
	}

	codec_np = of_find_node_by_phandle(*codec_ph);
	if (!codec_np) {
		dev_err(&ofdev->dev, "codec handle does not exist\n");
		goto error;
	}

	/* The MPC8610 HPCD only knows about the CS4270 codec, so reject
	 * anything else.
	 * FIXME: This should be unnecessary
	 */
	if (!of_device_is_compatible(codec_np, "cirrus,cs4270")) {
		dev_dbg(&ofdev->dev, "unknown codec %s\n",
			(char *) of_get_property(codec_np, "compatible", NULL));
		goto error;
	}

	/* Get the device ID */
	iprop = of_get_property(np, "cell-index", NULL);
	if (!iprop) {
		dev_err(&ofdev->dev, "cell-index property not found\n");
		ret = -EINVAL;
		goto error;
	}
	ssi_info->id = *iprop;

	sprintf(ssi_info->name, "ssi%u", ssi_info->id);

	/* Get the serial format and clock direction. */
	sprop = of_get_property(np, "fsl,mode", NULL);
	if (!sprop) {
		dev_err(&ofdev->dev, "fsl,mode property not found\n");
		ret = -EINVAL;
		goto error;
	}

	if (strcasecmp(sprop, "i2s-slave") == 0) {
		ssi_info->dai_format = SND_SOC_DAIFMT_I2S;
		ssi_info->codec_clk_direction = SND_SOC_CLOCK_OUT;
		ssi_info->cpu_clk_direction = SND_SOC_CLOCK_IN;
	} else if (strcasecmp(sprop, "i2s-master") == 0) {
		ssi_info->dai_format = SND_SOC_DAIFMT_I2S;
		ssi_info->codec_clk_direction = SND_SOC_CLOCK_IN;
		ssi_info->cpu_clk_direction = SND_SOC_CLOCK_OUT;
	} else if (strcasecmp(sprop, "lj-slave") == 0) {
		ssi_info->dai_format = SND_SOC_DAIFMT_LEFT_J;
		ssi_info->codec_clk_direction = SND_SOC_CLOCK_OUT;
		ssi_info->cpu_clk_direction = SND_SOC_CLOCK_IN;
	} else if (strcasecmp(sprop, "lj-master") == 0) {
		ssi_info->dai_format = SND_SOC_DAIFMT_LEFT_J;
		ssi_info->codec_clk_direction = SND_SOC_CLOCK_IN;
		ssi_info->cpu_clk_direction = SND_SOC_CLOCK_OUT;
	} else if (strcasecmp(sprop, "rj-slave") == 0) {
		ssi_info->dai_format = SND_SOC_DAIFMT_RIGHT_J;
		ssi_info->codec_clk_direction = SND_SOC_CLOCK_OUT;
		ssi_info->cpu_clk_direction = SND_SOC_CLOCK_IN;
	} else if (strcasecmp(sprop, "rj-master") == 0) {
		ssi_info->dai_format = SND_SOC_DAIFMT_RIGHT_J;
		ssi_info->codec_clk_direction = SND_SOC_CLOCK_IN;
		ssi_info->cpu_clk_direction = SND_SOC_CLOCK_OUT;
	} else if (strcasecmp(sprop, "ac97-slave") == 0) {
		ssi_info->dai_format = SND_SOC_DAIFMT_AC97;
		ssi_info->codec_clk_direction = SND_SOC_CLOCK_OUT;
		ssi_info->cpu_clk_direction = SND_SOC_CLOCK_IN;
	} else if (strcasecmp(sprop, "ac97-master") == 0) {
		ssi_info->dai_format = SND_SOC_DAIFMT_AC97;
		ssi_info->codec_clk_direction = SND_SOC_CLOCK_IN;
		ssi_info->cpu_clk_direction = SND_SOC_CLOCK_OUT;
	} else {
		dev_err(&ofdev->dev,
			"unrecognized fsl,mode property \"%s\"\n", sprop);
		ret = -EINVAL;
		goto error;
	}

	/* If the codec is the clock source, then the codec node should
	   contain the clock frequency. */
	if (ssi_info->codec_clk_direction == SND_SOC_CLOCK_OUT) {
		iprop = of_get_property(codec_np, "clock-frequency", NULL);
		if (!iprop || !*iprop) {
			dev_err(&ofdev->dev, "codec clock-frequency property "
				"is missing or invalid\n");
			ret = -EINVAL;
			goto error;
		}
		ssi_info->clk_frequency = *iprop;
	}

	if (!ssi_info->clk_frequency) {
		dev_err(&ofdev->dev, "unknown clock frequency\n");
		ret = -EINVAL;
		goto error;
	}

	/* Read the SSI hardware information from the device tree */
	ret = of_address_to_resource(np, 0, &res);
	if (ret) {
		dev_err(&ofdev->dev, "could not obtain SSI address\n");
		goto error;
	}
	if (!res.start) {
		dev_err(&ofdev->dev, "invalid SSI address\n");
		goto error;
	}
	ssi_info->ssi_phys = res.start;

	ssi_info->ssi = ioremap(ssi_info->ssi_phys, sizeof(struct ccsr_ssi));
	if (!ssi_info->ssi) {
		dev_err(&ofdev->dev, "could not map SSI address %x\n",
			ssi_info->ssi_phys);
		ret = -EINVAL;
		goto error;
	}

	/* Get the IRQ of the SSI */
	ssi_info->irq = irq_of_parse_and_map(np, 0);
	if (!ssi_info->irq) {
		dev_err(&ofdev->dev, "could not get SSI IRQ\n");
		ret = -EINVAL;
		goto error;
	}

	ssi_info->dma_info[0].ssi_sxx_phys = ssi_info->ssi_phys +
		offsetof(struct ccsr_ssi, stx0);
	ssi_info->dma_info[1].ssi_sxx_phys = ssi_info->ssi_phys +
		offsetof(struct ccsr_ssi, srx0);

	/*
	 * Get the DMA information.  If it's an older device tree (i.e. without
	 * an "fsl,playback-dma" property), then we assume that SSI1 uses DMA1
	 * Channels 0 and 1, and SSI2 uses DMA2 Channels 0 and 1.
	 */
	dma_ph = of_get_property(np, "fsl,playback-dma", NULL);
	if (!dma_ph) {
		dev_warn(&ofdev->dev, "please update your device tree\n");
		dma_np[0] = find_dma_node(ssi_info->id, 0);
		dma_np[1] = find_dma_node(ssi_info->id, 1);
	} else {
		dma_np[0] = of_find_node_by_phandle(*dma_ph);
		dma_ph = of_get_property(np, "fsl,capture-dma", NULL);
		if (dma_ph)
			dma_np[1] = of_find_node_by_phandle(*dma_ph);
	}

	if (!get_dma_info(dma_np[0], &ssi_info->dma_info[0])) {
		dev_err(&ofdev->dev, "could not obtain playback DMA info\n");
		goto error;
	}

	if (!get_dma_info(dma_np[1], &ssi_info->dma_info[1])) {
		dev_err(&ofdev->dev, "could not obtain capture DMA info\n");
		goto error;
	}

	memset(&dai_template, 0, sizeof(dai_template));
	dai_template.name = ssi_info->name;
	dai_template.id = ssi_info->id;
	dai_template.playback = &playback;
	dai_template.capture = &capture;
	dai_template.ops = &ops;

	/* If the other three drivers have loaded, then the call to
	   snd_soc_register_platform_dai() will initialize the system. */
	ssi_info->dai =
		snd_soc_register_platform_dai(&dai_template, &ofdev->dev);
	if (!ssi_info->dai) {
		dev_err(&ofdev->dev, "could not register platform DAI\n");
		ret = -ENOMEM;
		goto error;
	}

	ssi_info->dai->private_data = ssi_info;

	ssi_info->dev_attr.attr.name = "stats";
	ssi_info->dev_attr.attr.mode = S_IRUGO;
	ssi_info->dev_attr.show = fsl_sysfs_ssi_show;

	ret = device_create_file(&ofdev->dev, &ssi_info->dev_attr);
	if (ret) {
		dev_err(&ofdev->dev, "could not create sysfs %s file\n",
			ssi_info->dev_attr.attr.name);
		goto error;
	}

	dev_set_drvdata(&ofdev->dev, ssi_info);

	return 0;

error:
	if (ssi_info->dai)
		snd_soc_unregister_platform_dai(ssi_info->dai);

	if (ssi_info->irq)
		irq_dispose_mapping(ssi_info->irq);

	if (ssi_info->ssi)
		iounmap(ssi_info->ssi);

	kfree(ssi_info);

	return ret;
}

/**
 * fsl_ssi_remove: remove the OF device
 *
 * This function is called when the OF device is removed.
 */
static int fsl_ssi_remove(struct of_device *ofdev)
{
	struct fsl_ssi_info *ssi_info = dev_get_drvdata(&ofdev->dev);

	if (ssi_info) {
		device_remove_file(&ofdev->dev, &ssi_info->dev_attr);
		snd_soc_unregister_platform_dai(ssi_info->dai);
		irq_dispose_mapping(ssi_info->irq);
		iounmap(ssi_info->ssi);
		kfree(ssi_info);
		dev_set_drvdata(&ofdev->dev, NULL);
	}

	return 0;
}

static struct of_device_id fsl_ssi_match[] = {
	{
		.compatible = "fsl,mpc8610-ssi",
	},
	{}
};
MODULE_DEVICE_TABLE(of, fsl_ssi_match);

static struct of_platform_driver fsl_ssi_of_driver = {
	.owner  	= THIS_MODULE,
	.name   	= "fsl_ssi",
	.match_table    = fsl_ssi_match,
	.probe  	= fsl_ssi_probe,
	.remove 	= fsl_ssi_remove,
};

/**
 * fsl_ssi_init: fabric driver initialization.
 *
 * This function is called when this module is loaded.
 */
static int __init fsl_ssi_init(void)
{
	int ret;
	
	pr_info("Freescale SSI ASoC CPU driver\n");
	
	ret = of_register_platform_driver(&fsl_ssi_of_driver);

	if (ret)
		printk(KERN_ERR
			"fsl-ssi: failed to register SSI driver\n");

	return ret;
}

/**
 * fsl_ssi_exit: fabric driver exit
 *
 * This function is called when this driver is unloaded.
 */
static void __exit fsl_ssi_exit(void)
{
	of_unregister_platform_driver(&fsl_ssi_of_driver);
}

module_init(fsl_ssi_init);
module_exit(fsl_ssi_exit);

MODULE_AUTHOR("Timur Tabi <timur@freescale.com>");
MODULE_DESCRIPTION("Freescale SSI ASoC CPU driver");
MODULE_LICENSE("GPL");
