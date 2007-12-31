/*
 * linux/sound/soc-dai.h -- ALSA SoC Layer
 *
 * Author:		Liam Girdwood
 * Created:		Aug 11th 2005
 * Copyright:	Wolfson Microelectronics. PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_SOC_DAI_H
#define __LINUX_SND_SOC_DAI_H


#include <linux/types.h>
#include <linux/workqueue.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>
#include <sound/ac97_codec.h>

/*
 * DAI hardware audio formats
 */
#define SND_SOC_DAIFMT_I2S	0	/* I2S mode */
#define SND_SOC_DAIFMT_RIGHT_J	1	/* Right justified mode */
#define SND_SOC_DAIFMT_LEFT_J	2	/* Left Justified mode */
#define SND_SOC_DAIFMT_DSP_A	3	/* L data msb after FRM or LRC */
#define SND_SOC_DAIFMT_DSP_B	4	/* L data msb during FRM or LRC */
#define SND_SOC_DAIFMT_AC97	5	/* AC97 */

#define SND_SOC_DAIFMT_MSB 	SND_SOC_DAIFMT_LEFT_J
#define SND_SOC_DAIFMT_LSB	SND_SOC_DAIFMT_RIGHT_J

/*
 * DAI Gating
 */
#define SND_SOC_DAIFMT_CONT		(0 << 4)	/* continuous clock */
#define SND_SOC_DAIFMT_GATED		(1 << 4)	/* clock is gated when not Tx/Rx */

/*
 * DAI Sync
 * Synchronous LR (Left Right) clocks and Frame signals.
 */
#define SND_SOC_DAIFMT_SYNC		(0 << 5)	/* Tx FRM = Rx FRM */ 
#define SND_SOC_DAIFMT_ASYNC		(1 << 5)	/* Tx FRM ~ Rx FRM */ 

/*
 * TDM
 */
#define SND_SOC_DAIFMT_TDM		(1 << 6)

/*
 * DAI hardware signal inversions
 */
#define SND_SOC_DAIFMT_NB_NF		(0 << 8)	/* normal bit clock + frame */
#define SND_SOC_DAIFMT_NB_IF		(1 << 8)	/* normal bclk + inv frm */
#define SND_SOC_DAIFMT_IB_NF		(2 << 8)	/* invert bclk + nor frm */
#define SND_SOC_DAIFMT_IB_IF		(3 << 8)	/* invert bclk + frm */

/*
 * DAI hardware clock masters
 * This is wrt the codec, the inverse is true for the interface
 * i.e. if the codec is clk and frm master then the interface is
 * clk and frame slave.
 */
#define SND_SOC_DAIFMT_CBM_CFM	(0 << 12) /* codec clk & frm master */
#define SND_SOC_DAIFMT_CBS_CFM	(1 << 12) /* codec clk slave & frm master */
#define SND_SOC_DAIFMT_CBM_CFS	(2 << 12) /* codec clk master & frame slave */
#define SND_SOC_DAIFMT_CBS_CFS	(3 << 12) /* codec clk & frm slave */

#define SND_SOC_DAIFMT_FORMAT_MASK		0x000f
#define SND_SOC_DAIFMT_CLOCK_MASK		0x00f0
#define SND_SOC_DAIFMT_INV_MASK			0x0f00
#define SND_SOC_DAIFMT_MASTER_MASK		0xf000

/*
 * Master Clock Directions
 */
#define SND_SOC_CLOCK_IN	0
#define SND_SOC_CLOCK_OUT	1


struct snd_soc_dai;
struct snd_soc_dai_runtime;

/* 
 * Digital Audio Interface control and clocking.
 */
 
/**
 * snd_soc_dai_set_sysclk - create new ASoC platform.
 * @machine: parent machine
 * @platform_id: platform ID name
 *
 * Creates a new ASoC audio platform device and attaches to parent machine.
 */
int snd_soc_dai_set_sysclk(struct snd_soc_dai_runtime *dai, int clk_id, 
	unsigned int freq, int dir);
int snd_soc_dai_set_clkdiv(struct snd_soc_dai_runtime *dai, 
	int div_id, int div);
int snd_soc_dai_set_pll(struct snd_soc_dai_runtime *dai,
	int pll_id, unsigned int freq_in, unsigned int freq_out);
int snd_soc_dai_set_fmt(struct snd_soc_dai_runtime *dai, unsigned int fmt);
int snd_soc_dai_set_tdm_slot(struct snd_soc_dai_runtime *dai,
	unsigned int mask, int slots);
int snd_soc_dai_set_tristate(struct snd_soc_dai_runtime *dai, int tristate);
int snd_soc_dai_digital_mute(struct snd_soc_dai_runtime *dai, int mute);


struct snd_soc_dai_caps {
	char * stream_name;
	u64 formats;			/* SNDRV_PCM_FMTBIT_* */
	unsigned int rates;		/* SNDRV_PCM_RATE_* */
	unsigned int rate_min;		/* min rate */
	unsigned int rate_max;		/* max rate */
	unsigned int channels_min;	/* min channels */
	unsigned int channels_max;	/* max channels */
};


/*
 * Digital Audio Interface.
 * 
 * Describes the Digital Audio Interface in terms of it's ALSA, DAI and AC97 
 * operations an capabilities. Codec and platfom drivers will register a this
 * structure for every DAI they have.
 * 
 * This structure covers the clocking, formating and ALSA operations for each
 * interface a
 */
struct snd_soc_dai {
	char *name;
	int id;
	int ac97_control; /* are we an AC97 control interface */
	
	/* playback and capture capabilities */
	struct snd_soc_dai_caps playback;
	struct snd_soc_dai_caps capture;
	
	/*
	 * Private resources can be requested and released here - optional.
	 */
	int (*new)(struct snd_soc_dai_runtime *dai);
	void (*free)(struct snd_soc_dai_runtime *dai);

	/* 
	 * DAI clocking configuration - all optional.
	 * Called by machine drivers, usually in their hw_params().
	 */
	int (*set_sysclk)(struct snd_soc_dai_runtime *dai, int clk_id, 
		unsigned int freq, int dir);
	int (*set_clkdiv)(struct snd_soc_dai_runtime *dai,
		int div_id, int div);
	int (*set_pll)(struct snd_soc_dai_runtime *dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out);

	/* 
	 * DAI format configuration - all optional.
	 * Called by machine drivers, usually in their hw_params().
	 */
	int (*set_fmt)(struct snd_soc_dai_runtime *dai,
		unsigned int fmt);
	int (*set_tdm_slot)(struct snd_soc_dai_runtime *dai,
		unsigned int mask, int slots);
	int (*set_tristate)(struct snd_soc_dai_runtime *dai, int tristate);
	
	/*
	 * DAI digital mute - optional.
	 * Called by soc-core to minimise any pops.
	 */
	int (*digital_mute)(struct snd_soc_dai_runtime *dai, int mute);
	
	/* 
	 * ALSA PCM audio operations - all optional.
	 * Called by soc-core during audio PCM operations.
	 */
	int (*startup)(struct snd_pcm_substream *, 
		struct snd_soc_dai_runtime *);
	void (*shutdown)(struct snd_pcm_substream *, 
		struct snd_soc_dai_runtime *);
	int (*hw_params)(struct snd_pcm_substream *, 
		struct snd_pcm_hw_params *, struct snd_soc_dai_runtime *);
	int (*hw_free)(struct snd_pcm_substream *, 
		struct snd_soc_dai_runtime *);
	int (*prepare)(struct snd_pcm_substream *, 
		struct snd_soc_dai_runtime *);
	int (*trigger)(struct snd_pcm_substream *, int, 
		struct snd_soc_dai_runtime *);
	
	/* AC97 bus operations - for ac97 control interface only */
	struct snd_ac97_bus_ops *ac97_ops;
};

/*
 * Digital Audio Interface runtime data.
 * 
 * Holds runtime data for a DAI.
 */
struct snd_soc_dai_runtime {
	/* runtime info */
	struct snd_soc_dai *dai;
	struct snd_pcm_runtime *runtime;
	union {
		struct snd_soc_codec *codec;
		struct snd_soc_platform *platform;
	};
	struct list_head list;
	int active;
	
	/* driver and dai data */
	void *drv_data;
	void *private_data;
	void *dma_data;
};

#endif
