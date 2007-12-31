/*
 * linux/sound/arm/pxa2xx-pcm.h -- ALSA PCM interface for the Intel PXA2xx chip
 *
 * Author:	Nicolas Pitre
 * Created:	Nov 30, 2004
 * Copyright:	MontaVista Software, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef _PXA2XX_PCM_H
#define _PXA2XX_PCM_H

/* 
 * PXA Digital Audio Interface ID's 
 */
#define PXA2XX_DAI_I2S			0
#define PXA2XX_DAI_AC97_HIFI		1
#define PXA2XX_DAI_AC97_AUX		2
#define PXA2XX_DAI_AC97_MIC		3
#define PXA2XX_DAI_SSP1			4
#define PXA2XX_DAI_SSP2			5
#define PXA2XX_DAI_SSP3			6

/*
 * PXA Audio Clocking.
 */
 
/* I2S clock */
#define PXA2XX_I2S_SYSCLK		0

/* SSP clock sources */
#define PXA2XX_SSP_CLK_PLL		0
#define PXA2XX_SSP_CLK_EXT		1
#define PXA2XX_SSP_CLK_NET		2
#define PXA2XX_SSP_CLK_AUDIO		3
#define PXA2XX_SSP_CLK_NET_PLL		4

/* SSP audio dividers */
#define PXA2XX_SSP_AUDIO_DIV_ACDS	0
#define PXA2XX_SSP_AUDIO_DIV_SCDB	1
#define PXA2XX_SSP_DIV_SCR		2

/* SSP ACDS audio dividers values */
#define PXA2XX_SSP_CLK_AUDIO_DIV_1	0
#define PXA2XX_SSP_CLK_AUDIO_DIV_2	1
#define PXA2XX_SSP_CLK_AUDIO_DIV_4	2
#define PXA2XX_SSP_CLK_AUDIO_DIV_8	3
#define PXA2XX_SSP_CLK_AUDIO_DIV_16	4
#define PXA2XX_SSP_CLK_AUDIO_DIV_32	5

/* SSP divider bypass */
#define PXA2XX_SSP_CLK_SCDB_4		0
#define PXA2XX_SSP_CLK_SCDB_1		1


/* Audio DMA parameter */
struct pxa2xx_pcm_dma_params {
	char *name;			/* stream identifier */
	u32 dcmd;			/* DMA descriptor dcmd field */
	volatile u32 *drcmr;		/* the DMA request channel to use */
	u32 dev_addr;			/* device physical address for DMA */
};

/* DAI GPIO parameters */
struct pxa2xx_gpio {
	u32 sys;
	u32 rx;
	u32 tx;
	u32 clk;
	u32 frm;
};

/* PXA audio platform ID */
extern const char pxa_platform_id[];

extern struct snd_soc_dai pxa2xx_i2s;
extern struct snd_soc_dai pxa2xx_ac97[];
extern struct snd_soc_dai pxa2xx_ssp[];

#endif
