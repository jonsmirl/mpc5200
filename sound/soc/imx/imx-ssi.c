/*
 * imx-ssi.c  --  SSI driver for Freescale IMX
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  Based on mxc-alsa-mc13783 (C) 2006 Freescale.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    29th Aug 2006   Initial version.
 *
 * TODO:
 *   Need to rework SSI register defs when new defs go into mainline.
 *   Add support for TDM and FIFO 1.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <asm/arch/dma.h>
#include <asm/arch/clock.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>

#include "imx-ssi.h"
#include "imx31-pcm.h"

/* debug */
#define IMX_SSI_DEBUG 0
#if IMX_SSI_DEBUG
#define dbg(format, arg...) printk(KERN_DEBUG format, ## arg)
#else
#define dbg(format, arg...)
#endif

#define SSI1_PORT	0
#define SSI2_PORT	1

static int ssi_active[2] = { 0, 0 };

static struct mxc_pcm_dma_params imx_ssi1_pcm_stereo_out0 = {
	.name = "SSI1 PCM Stereo out 0",
	.params = {
		   .bd_number = 1,
		   .transfer_type = emi_2_per,
		   .watermark_level = SDMA_TXFIFO_WATERMARK,
		   .per_address = SSI1_BASE_ADDR,
		   .event_id = DMA_REQ_SSI1_TX1,
		   .peripheral_type = SSI,
		   },
};

static struct mxc_pcm_dma_params imx_ssi1_pcm_stereo_out1 = {
	.name = "SSI1 PCM Stereo out 1",
	.params = {
		   .bd_number = 1,
		   .transfer_type = emi_2_per,
		   .watermark_level = SDMA_TXFIFO_WATERMARK,
		   .per_address = SSI1_BASE_ADDR + 0x4,
		   .event_id = DMA_REQ_SSI1_TX2,
		   .peripheral_type = SSI,
		   },
};

static struct mxc_pcm_dma_params imx_ssi1_pcm_stereo_in0 = {
	.name = "SSI1 PCM Stereo in 0",
	.params = {
		   .bd_number = 1,
		   .transfer_type = per_2_emi,
		   .watermark_level = SDMA_RXFIFO_WATERMARK,
		   .per_address = SSI1_BASE_ADDR + 0x8,
		   .event_id = DMA_REQ_SSI1_RX1,
		   .peripheral_type = SSI,
		   },
};

static struct mxc_pcm_dma_params imx_ssi1_pcm_stereo_in1 = {
	.name = "SSI1 PCM Stereo in 1",
	.params = {
		   .bd_number = 1,
		   .transfer_type = per_2_emi,
		   .watermark_level = SDMA_RXFIFO_WATERMARK,
		   .per_address = SSI1_BASE_ADDR + 0xc,
		   .event_id = DMA_REQ_SSI1_RX2,
		   .peripheral_type = SSI,
		   },
};

static struct mxc_pcm_dma_params imx_ssi2_pcm_stereo_out0 = {
	.name = "SSI2 PCM Stereo out 0",
	.params = {
		   .bd_number = 1,
		   .transfer_type = emi_2_per,
		   .watermark_level = SDMA_TXFIFO_WATERMARK,
		   .per_address = SSI2_BASE_ADDR,
		   .event_id = DMA_REQ_SSI2_TX1,
		   .peripheral_type = SSI,
		   },
};

static struct mxc_pcm_dma_params imx_ssi2_pcm_stereo_out1 = {
	.name = "SSI2 PCM Stereo out 1",
	.params = {
		   .bd_number = 1,
		   .transfer_type = emi_2_per,
		   .watermark_level = SDMA_TXFIFO_WATERMARK,
		   .per_address = SSI2_BASE_ADDR + 0x4,
		   .event_id = DMA_REQ_SSI2_TX2,
		   .peripheral_type = SSI,
		   },
};

static struct mxc_pcm_dma_params imx_ssi2_pcm_stereo_in0 = {
	.name = "SSI2 PCM Stereo in 0",
	.params = {
		   .bd_number = 1,
		   .transfer_type = per_2_emi,
		   .watermark_level = SDMA_RXFIFO_WATERMARK,
		   .per_address = SSI2_BASE_ADDR + 0x8,
		   .event_id = DMA_REQ_SSI2_RX1,
		   .peripheral_type = SSI,
		   },
};

static struct mxc_pcm_dma_params imx_ssi2_pcm_stereo_in1 = {
	.name = "SSI2 PCM Stereo in 1",
	.params = {
		   .bd_number = 1,
		   .transfer_type = per_2_emi,
		   .watermark_level = SDMA_RXFIFO_WATERMARK,
		   .per_address = SSI2_BASE_ADDR + 0xc,
		   .event_id = DMA_REQ_SSI2_RX2,
		   .peripheral_type = SSI,
		   },
};

static struct clk *ssi_clk0, *ssi_clk1;

int get_ssi_clk(int ssi, struct device *dev)
{
	switch (ssi) {
	case 0:
		ssi_clk0 = clk_get(dev, "ssi_clk.0");
		if (IS_ERR(ssi_clk0))
			return PTR_ERR(ssi_clk0);
		return 0;
	case 1:
		ssi_clk1 = clk_get(dev, "ssi_clk.1");
		if (IS_ERR(ssi_clk1))
			return PTR_ERR(ssi_clk1);
		return 0;
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL(get_ssi_clk);

void put_ssi_clk(int ssi)
{
	switch (ssi) {
	case 0:
		clk_put(ssi_clk0);
		ssi_clk0 = NULL;
		break;
	case 1:
		clk_put(ssi_clk1);
		ssi_clk1 = NULL;
		break;
	}
}
EXPORT_SYMBOL(put_ssi_clk);

/*
 * SSI system clock configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				  int clk_id, unsigned int freq, int dir)
{
	u32 scr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		scr = SSI1_SCR;
	else
		scr = SSI2_SCR;

	if (scr & SSI_SCR_SSIEN)
		return 0;

	switch (clk_id) {
	case IMX_SSP_SYS_CLK:
		if (dir == SND_SOC_CLOCK_OUT)
			scr |= SSI_SCR_SYS_CLK_EN;
		else
			scr &= ~SSI_SCR_SYS_CLK_EN;
		break;
	default:
		return -EINVAL;
	}

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		SSI1_SCR = scr;
	else
		SSI2_SCR = scr;

	return 0;
}

/*
 * SSI Clock dividers
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				  int div_id, int div)
{
	u32 stccr, srccr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		if (SSI1_SCR & SSI_SCR_SSIEN)
			return 0;

		srccr = SSI1_STCCR;
		stccr = SSI1_STCCR;
	} else {
		if (SSI2_SCR & SSI_SCR_SSIEN)
			return 0;

		srccr = SSI2_STCCR;
		stccr = SSI2_STCCR;
	}

	switch (div_id) {
	case IMX_SSI_TX_DIV_2:
		stccr &= ~SSI_STCCR_DIV2;
		stccr |= div;
		break;
	case IMX_SSI_TX_DIV_PSR:
		stccr &= ~SSI_STCCR_PSR;
		stccr |= div;
		break;
	case IMX_SSI_TX_DIV_PM:
		stccr &= ~0xff;
		stccr |= SSI_STCCR_PM(div);
		break;
	case IMX_SSI_RX_DIV_2:
		stccr &= ~SSI_STCCR_DIV2;
		stccr |= div;
		break;
	case IMX_SSI_RX_DIV_PSR:
		stccr &= ~SSI_STCCR_PSR;
		stccr |= div;
		break;
	case IMX_SSI_RX_DIV_PM:
		stccr &= ~0xff;
		stccr |= SSI_STCCR_PM(div);
		break;
	default:
		return -EINVAL;
	}

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		SSI1_STCCR = stccr;
		SSI1_SRCCR = srccr;
	} else {
		SSI2_STCCR = stccr;
		SSI2_SRCCR = srccr;
	}
	return 0;
}

/*
 * SSI Network Mode or TDM slots configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 */
static int imx_ssi_set_dai_tdm_slot(struct snd_soc_dai *cpu_dai,
				    unsigned int mask, int slots)
{
	u32 stmsk, srmsk, stccr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		if (SSI1_SCR & SSI_SCR_SSIEN)
			return 0;
		stccr = SSI1_STCCR;
	} else {
		if (SSI2_SCR & SSI_SCR_SSIEN)
			return 0;
		stccr = SSI2_STCCR;
	}

	stmsk = srmsk = mask;
	stccr &= ~SSI_STCCR_DC_MASK;
	stccr |= SSI_STCCR_DC(slots - 1);

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		SSI1_STMSK = stmsk;
		SSI1_SRMSK = srmsk;
		SSI1_SRCCR = SSI1_STCCR = stccr;
	} else {
		SSI2_STMSK = stmsk;
		SSI2_SRMSK = srmsk;
		SSI2_SRCCR = SSI2_STCCR = stccr;
	}

	return 0;
}

/*
 * SSI DAI format configuration.
 * Should only be called when port is inactive (i.e. SSIEN = 0).
 * Note: We don't use the I2S modes but instead manually configure the
 * SSI for I2S.
 */
static int imx_ssi_set_dai_fmt(struct snd_soc_dai *cpu_dai, unsigned int fmt)
{
	u32 stcr = 0, srcr = 0, scr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		scr = SSI1_SCR & ~(SSI_SCR_SYN | SSI_SCR_NET);
	else
		scr = SSI2_SCR & ~(SSI_SCR_SYN | SSI_SCR_NET);

	if (scr & SSI_SCR_SSIEN)
		return 0;

	/* DAI mode */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		/* data on rising edge of bclk, frame low 1clk before data */
		stcr |= SSI_STCR_TFSI | SSI_STCR_TEFS | SSI_STCR_TXBIT0;
		srcr |= SSI_SRCR_RFSI | SSI_SRCR_REFS | SSI_SRCR_RXBIT0;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		/* data on rising edge of bclk, frame high with data */
		stcr |= SSI_STCR_TXBIT0;
		srcr |= SSI_SRCR_RXBIT0;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		/* data on rising edge of bclk, frame high with data */
		stcr |= SSI_STCR_TFSL;
		srcr |= SSI_SRCR_RFSL;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		/* data on rising edge of bclk, frame high 1clk before data */
		stcr |= SSI_STCR_TFSL | SSI_STCR_TEFS;
		srcr |= SSI_SRCR_RFSL | SSI_SRCR_REFS;
		break;
	}

	/* DAI clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		stcr |= SSI_STCR_TFSI;
		stcr &= ~SSI_STCR_TSCKP;
		srcr |= SSI_SRCR_RFSI;
		srcr &= ~SSI_SRCR_RSCKP;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		stcr &= ~(SSI_STCR_TSCKP | SSI_STCR_TFSI);
		srcr &= ~(SSI_SRCR_RSCKP | SSI_SRCR_RFSI);
		break;
	case SND_SOC_DAIFMT_NB_IF:
		stcr |= SSI_STCR_TFSI | SSI_STCR_TSCKP;
		srcr |= SSI_SRCR_RFSI | SSI_SRCR_RSCKP;
		break;
	case SND_SOC_DAIFMT_NB_NF:
		stcr &= ~SSI_STCR_TFSI;
		stcr |= SSI_STCR_TSCKP;
		srcr &= ~SSI_SRCR_RFSI;
		srcr |= SSI_SRCR_RSCKP;
		break;
	}

	/* DAI clock master masks */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		stcr |= SSI_STCR_TFDIR | SSI_STCR_TXDIR;
		srcr |= SSI_SRCR_RFDIR | SSI_SRCR_RXDIR;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		stcr |= SSI_STCR_TFDIR;
		srcr |= SSI_SRCR_RFDIR;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		stcr |= SSI_STCR_TXDIR;
		srcr |= SSI_SRCR_RXDIR;
		break;
	}

	/* sync */
	if (!(fmt & SND_SOC_DAIFMT_ASYNC))
		scr |= SSI_SCR_SYN;

	/* tdm - only for stereo atm */
	if (fmt & SND_SOC_DAIFMT_TDM)
		scr |= SSI_SCR_NET;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		SSI1_STCR = stcr;
		SSI1_SRCR = srcr;
		SSI1_SCR = scr;
	} else {
		SSI2_STCR = stcr;
		SSI2_SRCR = srcr;
		SSI2_SCR = scr;
	}
	return 0;
}

static int imx_ssi_startup(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *cpu_dai)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;

	/* we cant really change any SSI values after SSI is enabled
	 * need to fix in software for max flexibility - lrg */
	if (pcm_runtime->playback_active || pcm_runtime->capture_active)
		return 0;

	/* reset the SSI port - Sect 45.4.4 */
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {

		if (!ssi_clk0)
			return -EINVAL;

		if (ssi_active[SSI1_PORT]++)
			return 0;

		SSI1_SCR = 0;
		clk_enable(ssi_clk0);

		/* BIG FAT WARNING
		 * SDMA FIFO watermark must == SSI FIFO watermark for
		 * best results.
		 */
		SSI1_SFCSR = SSI_SFCSR_RFWM1(SSI_RXFIFO_WATERMARK) |
		    SSI_SFCSR_RFWM0(SSI_RXFIFO_WATERMARK) |
		    SSI_SFCSR_TFWM1(SSI_TXFIFO_WATERMARK) |
		    SSI_SFCSR_TFWM0(SSI_TXFIFO_WATERMARK);
	} else {
		if (!ssi_clk1)
			return -EINVAL;

		if (ssi_active[SSI2_PORT]++)
			return 0;

		SSI2_SCR = 0;
		clk_enable(ssi_clk1);

		/* above warning applies here too */
		SSI2_SFCSR = SSI_SFCSR_RFWM1(SSI_RXFIFO_WATERMARK) |
		    SSI_SFCSR_RFWM0(SSI_RXFIFO_WATERMARK) |
		    SSI_SFCSR_TFWM1(SSI_TXFIFO_WATERMARK) |
		    SSI_SFCSR_TFWM0(SSI_TXFIFO_WATERMARK);
	}
	return 0;
}

static int imx_ssi_hw_tx_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *cpu_dai)
{
	u32 stccr, stcr, sier;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		stccr = SSI1_STCCR & ~SSI_STCCR_WL_MASK;
		stcr = SSI1_STCR;
		sier = SSI1_SIER;
	} else {
		stccr = SSI2_STCCR & ~SSI_STCCR_WL_MASK;
		stcr = SSI2_STCR;
		sier = SSI2_SIER;
	}

	/* DAI data (word) size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		stccr |= SSI_STCCR_WL(16);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		stccr |= SSI_STCCR_WL(20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		stccr |= SSI_STCCR_WL(24);
		break;
	}

	/* enable interrupts */
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		stcr |= SSI_STCR_TFEN0;
	else
		stcr |= SSI_STCR_TFEN1;
	sier |= SSI_SIER_TDMAE;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		SSI1_STCR = stcr;
		SSI1_STCCR = stccr;
		SSI1_SIER = sier;
	} else {
		SSI2_STCR = stcr;
		SSI2_STCCR = stccr;
		SSI2_SIER = sier;
	}

	return 0;
}

static int imx_ssi_hw_rx_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *cpu_dai)
{
	u32 srccr, srcr, sier;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		srccr = SSI1_SRCCR & ~SSI_SRCCR_WL_MASK;
		srcr = SSI1_SRCR;
		sier = SSI1_SIER;
	} else {
		srccr = SSI2_SRCCR & ~SSI_SRCCR_WL_MASK;
		srcr = SSI2_SRCR;
		sier = SSI2_SIER;
	}

	/* DAI data (word) size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		srccr |= SSI_SRCCR_WL(16);
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		srccr |= SSI_SRCCR_WL(20);
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		srccr |= SSI_SRCCR_WL(24);
		break;
	}

	/* enable interrupts */
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		srcr |= SSI_SRCR_RFEN0;
	else
		srcr |= SSI_SRCR_RFEN1;
	sier |= SSI_SIER_RDMAE;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		SSI1_SRCR = srcr;
		SSI1_SRCCR = srccr;
		SSI1_SIER = sier;
	} else {
		SSI2_SRCR = srcr;
		SSI2_SRCCR = srccr;
		SSI2_SIER = sier;
	}
	return 0;
}

/*
 * Should only be called when port is inactive (i.e. SSIEN = 0),
 * although can be called multiple times by upper layers.
 */
static int imx_ssi_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params,
			     struct snd_soc_dai *cpu_dai)
{
	/* Tx/Rx config */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* set up DMA params */
		switch (cpu_dai->id) {
		case IMX_DAI_SSI0:
			cpu_dai->dma_data = &imx_ssi1_pcm_stereo_out0;
			break;
		case IMX_DAI_SSI1:
			cpu_dai->dma_data = &imx_ssi1_pcm_stereo_out1;
			break;
		case IMX_DAI_SSI2:
			cpu_dai->dma_data = &imx_ssi2_pcm_stereo_out0;
			break;
		case IMX_DAI_SSI3:
			cpu_dai->dma_data = &imx_ssi2_pcm_stereo_out1;
			break;
		}

		/* cant change any parameters when SSI is running */
		if (cpu_dai->id == IMX_DAI_SSI0 ||
		    cpu_dai->id == IMX_DAI_SSI1) {
			if (SSI1_SCR & SSI_SCR_SSIEN)
				return 0;
		} else {
			if (SSI2_SCR & SSI_SCR_SSIEN)
				return 0;
		}
		return imx_ssi_hw_tx_params(substream, params, cpu_dai);
	} else {
		/* set up DMA params */
		switch (cpu_dai->id) {
		case IMX_DAI_SSI0:
			cpu_dai->dma_data = &imx_ssi1_pcm_stereo_in0;
			break;
		case IMX_DAI_SSI1:
			cpu_dai->dma_data = &imx_ssi1_pcm_stereo_in1;
			break;
		case IMX_DAI_SSI2:
			cpu_dai->dma_data = &imx_ssi2_pcm_stereo_in0;
			break;
		case IMX_DAI_SSI3:
			cpu_dai->dma_data = &imx_ssi2_pcm_stereo_in1;
			break;
		}

		/* cant change any parameters when SSI is running */
		if (cpu_dai->id == IMX_DAI_SSI0 ||
		    cpu_dai->id == IMX_DAI_SSI1) {
			if (SSI1_SCR & SSI_SCR_SSIEN)
				return 0;
		} else {
			if (SSI2_SCR & SSI_SCR_SSIEN)
				return 0;
		}
		return imx_ssi_hw_rx_params(substream, params, cpu_dai);
	}
}

static int imx_ssi_prepare(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *cpu_dai)
{
	u32 scr;

	/* enable the SSI port, note that no other port config
	 * should happen after SSIEN is set */
	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		scr = SSI1_SCR;
		SSI1_SCR = scr | SSI_SCR_SSIEN;
	} else {
		scr = SSI2_SCR;
		SSI2_SCR = scr | SSI_SCR_SSIEN;
	}
	return 0;
}

static int imx_ssi_trigger(struct snd_pcm_substream *substream, int cmd,
			   struct snd_soc_dai *cpu_dai)
{
	u32 scr;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		scr = SSI1_SCR;
	else
		scr = SSI2_SCR;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scr |= SSI_SCR_TE;
		else
			scr |= SSI_SCR_RE;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			scr &= ~SSI_SCR_TE;
		else
			scr &= ~SSI_SCR_RE;
		break;
	default:
		return -EINVAL;
	}

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1)
		SSI1_SCR = scr;
	else
		SSI2_SCR = scr;

	return 0;
}

static void imx_ssi_shutdown(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *cpu_dai)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;

	/* shutdown SSI if neither Tx or Rx is active */
	if (pcm_runtime->playback_active || pcm_runtime->capture_active)
		return;

	if (cpu_dai->id == IMX_DAI_SSI0 || cpu_dai->id == IMX_DAI_SSI1) {
		if (--ssi_active[SSI1_PORT] > 1)
			return;

		SSI1_SCR = 0;
		clk_disable(ssi_clk0);
	} else {
		if (--ssi_active[SSI2_PORT])
			return;
		SSI2_SCR = 0;
		clk_disable(ssi_clk1);
	}
}

static int ssi1_underrun_counter;
static int ssi2_underrun_counter;
static int ssi1_overrun_counter;
static int ssi2_overrun_counter;

static ssize_t ssi_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ssi1: %d %d ssi2: %d %d\n",
		       ssi1_underrun_counter, ssi1_overrun_counter,
		       ssi2_underrun_counter, ssi2_overrun_counter);
}

static DEVICE_ATTR(errors, 0444, ssi_show, NULL);

static irqreturn_t ssi1_irq(int irq, void *dev_id)
{
	u32 sisr = SSI1_SISR;

	if (sisr & (SSI_SIER_TUE0_EN | SSI_SIER_TUE1_EN))
		ssi1_underrun_counter++;
	if (sisr & (SSI_SIER_ROE0_EN | SSI_SIER_ROE1_EN))
		ssi1_overrun_counter++;

	SSI1_SISR = sisr;
	return IRQ_HANDLED;
}

static irqreturn_t ssi2_irq(int irq, void *dev_id)
{
	u32 sisr = SSI2_SISR;

	if (sisr & (SSI_SIER_TUE0_EN | SSI_SIER_TUE1_EN))
		ssi2_underrun_counter++;
	if (sisr & (SSI_SIER_ROE0_EN | SSI_SIER_ROE1_EN))
		ssi2_overrun_counter++;

	SSI2_SISR = sisr;
	return IRQ_HANDLED;
}

#define IMX_SSI_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
	SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | \
	SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | \
	SNDRV_PCM_RATE_96000)

#define IMX_SSI_BITS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_caps ssi_playback = {
	.channels_min = 1,
	.channels_max = 2,
	.rates = IMX_SSI_RATES,
	.formats = IMX_SSI_BITS,
};

static struct snd_soc_dai_caps ssi_capture = {
	.channels_min = 1,
	.channels_max = 2,
	.rates = IMX_SSI_RATES,
	.formats = IMX_SSI_BITS,
};

static struct snd_soc_dai_ops imx_ssi_ops = {
	/* alsa ops */
	.startup = imx_ssi_startup,
	.shutdown = imx_ssi_shutdown,
	.trigger = imx_ssi_trigger,
	.prepare = imx_ssi_prepare,
	.hw_params = imx_ssi_hw_params,

	/* dai ops */
	.set_sysclk = imx_ssi_set_dai_sysclk,
	.set_clkdiv = imx_ssi_set_dai_clkdiv,
	.set_fmt = imx_ssi_set_dai_fmt,
	.set_tdm_slot = imx_ssi_set_dai_tdm_slot,
};

struct ssi_data {
	struct snd_soc_dai *dai[4];
};

/* for modprobe */
const char imx_ssi_id1_0[] = "ssi-1.0";
EXPORT_SYMBOL_GPL(imx_ssi_id1_0);
const char imx_ssi_id1_1[] = "ssi-1.1";
EXPORT_SYMBOL_GPL(imx_ssi_id1_1);
const char imx_ssi_id2_0[] = "ssi-2.0";
EXPORT_SYMBOL_GPL(imx_ssi_id2_0);
const char imx_ssi_id2_1[] = "ssi-2.1";
EXPORT_SYMBOL_GPL(imx_ssi_id2_1);

struct snd_soc_dai_new ssi_dai[] = {
{
	.name		= imx_ssi_id1_0,
	.playback	= &ssi_playback,
	.capture	= &ssi_capture,
	.ops		= &imx_ssi_ops,
},
{
	.name		= imx_ssi_id1_1,
	.playback	= &ssi_playback,
	.capture	= &ssi_capture,
	.ops		= &imx_ssi_ops,
},
{
	.name		= imx_ssi_id2_0,
	.playback	= &ssi_playback,
	.capture	= &ssi_capture,
	.ops		= &imx_ssi_ops,
},
{
	.name		= imx_ssi_id2_1,
	.playback	= &ssi_playback,
	.capture	= &ssi_capture,
	.ops		= &imx_ssi_ops,
},
};

/* we could pass in some platform data here to set up our AUDMUX etc.
 * consider splitting into 2 platform devices */
static int imx_ssi_probe(struct platform_device *pdev)
{
	struct ssi_data *ssi;
	int ret, i;

	ssi = kzalloc(sizeof(*ssi), GFP_KERNEL);
	if (ssi == NULL)
		return -ENOMEM;

	ret = request_irq(INT_SSI1, ssi1_irq, 0, "ssi1", pdev);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to request irq %s\n", __func__,
		       "ssi1");
		goto irq1_err;
	}

	ret = request_irq(INT_SSI2, ssi2_irq, 0, "ssi2", pdev);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to request irq %s\n", __func__,
		       "ssi2");
		goto irq2_err;
	}

	platform_set_drvdata(pdev, ssi);
	for (i = 0; i < ARRAY_SIZE(ssi_dai); i++) {
		ssi->dai[i] = snd_soc_register_platform_dai(&ssi_dai[i],
							    &pdev->dev);
		if (ret < 0)
			goto unwind_reg;
	}

	ret = device_create_file(&pdev->dev, &dev_attr_errors);
	if (ret < 0) {
		printk(KERN_WARNING "%s: failed to add sysfs entry\n",
		       __func__);
		goto unwind_reg;
	}

	/* enable SSI interrupts */
	SSI1_SIER = SSI_SIER_TIE | SSI_SIER_TUE0_EN | SSI_SIER_TFE0_EN |
	    SSI_SIER_TFE1_EN | SSI_SIER_TUE1_EN | SSI_SIER_RFF0_EN |
	    SSI_SIER_RFF1_EN | SSI_SIER_ROE0_EN | SSI_SIER_ROE1_EN;
	SSI2_SIER = SSI_SIER_TIE | SSI_SIER_TUE0_EN | SSI_SIER_TFE0_EN |
	    SSI_SIER_TFE1_EN | SSI_SIER_TUE1_EN | SSI_SIER_RFF0_EN |
	    SSI_SIER_RFF1_EN | SSI_SIER_ROE0_EN | SSI_SIER_ROE1_EN;
	return ret;

unwind_reg:
	i--;
	for (; i >= 0; i--)
		snd_soc_unregister_platform_dai(ssi->dai[i]);
	free_irq(INT_SSI2, pdev);
irq2_err:
	free_irq(INT_SSI1, pdev);
irq1_err:
	kfree(ssi);
	return ret;
}

static int imx_ssi_remove(struct platform_device *pdev)
{
	struct ssi_data *ssi = platform_get_drvdata(pdev);
	int i;

	device_remove_file(&pdev->dev, &dev_attr_errors);
	for (i = ARRAY_SIZE(ssi_dai) - 1; i >= 0; i--)
		snd_soc_unregister_platform_dai(ssi->dai[i]);

	kfree(ssi);
	free_irq(INT_SSI1, pdev);
	free_irq(INT_SSI2, pdev);
	return 0;
}

static struct platform_driver imx_ssi_driver = {
	.driver = {
		   .name = "imx-ssi",
		   .owner = THIS_MODULE,
		   },
	.probe = imx_ssi_probe,
	.remove = __devexit_p(imx_ssi_remove),
};

static __init int imx_ssi_init(void)
{
	return platform_driver_register(&imx_ssi_driver);
}

static __exit void imx_ssi_exit(void)
{
	platform_driver_unregister(&imx_ssi_driver);
}

module_init(imx_ssi_init);
module_exit(imx_ssi_exit);

MODULE_AUTHOR("Liam Girdwood");
MODULE_DESCRIPTION("Freescale i.MX SSI module");
MODULE_LICENSE("GPL");
