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
 *
 * Digital Audio Interface (DAI) API.
 */

#ifndef __LINUX_SND_SOC_DAI_H
#define __LINUX_SND_SOC_DAI_H


#include <linux/list.h>

/*
 * DAI hardware audio formats.
 *
 * Describes the physical PCM data formating and clocking. Add new formats
 * to the end.
 */
#define SND_SOC_DAIFMT_I2S		0 /* I2S mode */
#define SND_SOC_DAIFMT_RIGHT_J		1 /* Right Justified mode */
#define SND_SOC_DAIFMT_LEFT_J		2 /* Left Justified mode */
#define SND_SOC_DAIFMT_DSP_A		3 /* L data msb after FRM LRC */
#define SND_SOC_DAIFMT_DSP_B		4 /* L data msb during FRM LRC */
#define SND_SOC_DAIFMT_AC97		5 /* AC97 */

/* left and right justified also known as MSB and LSB respectively */
#define SND_SOC_DAIFMT_MSB		SND_SOC_DAIFMT_LEFT_J
#define SND_SOC_DAIFMT_LSB		SND_SOC_DAIFMT_RIGHT_J

/*
 * DAI Clock gating.
 *
 * DAI bit clocks can be be gated (disabled) when not the DAI is not
 * sending or receiving PCM data in a frame. This can be used to save power.
 */
#define SND_SOC_DAIFMT_CONT		(0 << 4) /* continuous clock */
#define SND_SOC_DAIFMT_GATED		(1 << 4) /* clock is gated */

/*
 * DAI Left/Right Clocks.
 *
 * Specifies whether the DAI can support different samples for similtanious
 * playback and capture. This usually requires a seperate physical frame
 * clock for playback and capture.
 */
#define SND_SOC_DAIFMT_SYNC		(0 << 5) /* Tx FRM = Rx FRM */
#define SND_SOC_DAIFMT_ASYNC		(1 << 5) /* Tx FRM ~ Rx FRM */

/*
 * TDM
 *
 * Time Division Multiplexing. Allows PCM data to be multplexed with other
 * data on the DAI.
 */
#define SND_SOC_DAIFMT_TDM		(1 << 6)

/*
 * DAI hardware signal inversions.
 *
 * Specifies whether the DAI can also support inverted clocks for the specified
 * format.
 */
#define SND_SOC_DAIFMT_NB_NF		(0 << 8) /* normal bit clock + frame */
#define SND_SOC_DAIFMT_NB_IF		(1 << 8) /* normal bclk + inv frm */
#define SND_SOC_DAIFMT_IB_NF		(2 << 8) /* invert bclk + nor frm */
#define SND_SOC_DAIFMT_IB_IF		(3 << 8) /* invert bclk + frm */

/*
 * DAI hardware clock masters.
 *
 * This is wrt the codec, the inverse is true for the interface
 * i.e. if the codec is clk and frm master then the interface is
 * clk and frame slave.
 */
#define SND_SOC_DAIFMT_CBM_CFM		(0 << 12) /* codec clk & frm master */
#define SND_SOC_DAIFMT_CBS_CFM		(1 << 12) /* codec clk slave & frm master */
#define SND_SOC_DAIFMT_CBM_CFS		(2 << 12) /* codec clk master & frame slave */
#define SND_SOC_DAIFMT_CBS_CFS		(3 << 12) /* codec clk & frm slave */

#define SND_SOC_DAIFMT_FORMAT_MASK	0x000f
#define SND_SOC_DAIFMT_CLOCK_MASK	0x00f0
#define SND_SOC_DAIFMT_INV_MASK		0x0f00
#define SND_SOC_DAIFMT_MASTER_MASK	0xf000

/*
 * Master Clock Directions
 */
#define SND_SOC_CLOCK_IN		0
#define SND_SOC_CLOCK_OUT		1


struct snd_soc_dai_ops;
struct snd_soc_dai;
struct snd_ac97_bus_ops;

/**
 * snd_soc_dai_allocate - allocate and initialize a DAI.
 * @codec: codec driver
 *
 * Allocates and initializes struct dai before calling register.
 */
struct snd_soc_dai *snd_soc_dai_allocate(void);

/**
 * snd_soc_dai_free - free codec.
 * @codec: codec driver
 */
static inline void snd_soc_dai_free(struct snd_soc_dai *dai)
{
	kfree(dai);
}

/*
 * Digital Audio Interface control and clocking API.
 */

/**
 * snd_soc_dai_set_sysclk - configure DAI system or master clock.
 * @dai: DAI
 * @clk_id: DAI specific clock ID
 * @freq: new clock frequency in Hz
 * @dir: new clock direction - input/output.
 *
 * Configures the DAI master (MCLK) or system (SYSCLK) clocking.
 */
int snd_soc_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id,
	unsigned int freq, int dir);

/**
 * snd_soc_dai_set_clkdiv - configure DAI clock dividers.
 * @dai: DAI
 * @clk_id: DAI specific clock divider ID
 * @div: new clock divisor.
 *
 * Configures the clock dividers. This is used to derive the best DAI bit and
 * frame clocks from the system or master clock. It's best to set the DAI bit
 * and frame clocks as low as possible to save system power.
 */
int snd_soc_dai_set_clkdiv(struct snd_soc_dai *dai,
	int div_id, int div);

/**
 * snd_soc_dai_set_pll - configure DAI PLL.
 * @dai: DAI
 * @pll_id: DAI specific PLL ID
 * @freq_in: PLL input clock frequency in Hz
 * @freq_out: requested PLL output clock frequency in Hz
 *
 * Configures and enables PLL to generate output clock based on input clock.
 */
int snd_soc_dai_set_pll(struct snd_soc_dai *dai,
	int pll_id, unsigned int freq_in, unsigned int freq_out);

/**
 * snd_soc_dai_set_fmt - configure DAI hardware audio format.
 * @dai: DAI
 * @clk_id: DAI specific clock ID
 * @fmt: SND_SOC_DAIFMT_ format value.
 *
 * Configures the DAI hardware format and clocking.
 */
int snd_soc_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt);

/**
 * snd_soc_dai_set_tdm_slot - configure DAI TDM.
 * @dai: DAI
 * @mask: DAI specific mask representing used slots.
 * @slots: Number of slots in use.
 *
 * Configures a DAI for TDM operation. Both mask and slots are codec and DAI
 * specific.
 */
int snd_soc_dai_set_tdm_slot(struct snd_soc_dai *dai,
	unsigned int mask, int slots);

/**
 * snd_soc_dai_set_tristate - configure DAI system or master clock.
 * @dai: DAI
 * @tristate: tristate enable
 *
 * Tristates the DAI so that others can use it.
 */
int snd_soc_dai_set_tristate(struct snd_soc_dai *dai, int tristate);

/**
 * snd_soc_dai_digital_mute - configure DAI system or master clock.
 * @dai: DAI
 * @mute: mute enable
 *
 * Mutes the DAI DAC.
 */
int snd_soc_dai_digital_mute(struct snd_soc_dai *dai, int mute);

/*
 * DAI Capabilities.
 *
 * Describes the PCM audio capabilities for a Digital Audio interface.
 */
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
struct snd_soc_dai_ops {

	/* Private resources can be requested and released here - optional. */
	int (*new)(struct snd_soc_dai *dai);
	void (*free)(struct snd_soc_dai *dai);

	/*
	 * DAI clocking configuration - all optional.
	 * Called by machine drivers, usually in their hw_params().
	 */
	int (*set_sysclk)(struct snd_soc_dai *dai, int clk_id,
		unsigned int freq, int dir);
	int (*set_clkdiv)(struct snd_soc_dai *dai,
		int div_id, int div);
	int (*set_pll)(struct snd_soc_dai *dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out);

	/*
	 * DAI format configuration - all optional.
	 * Called by machine drivers, usually in their hw_params().
	 */
	int (*set_fmt)(struct snd_soc_dai *dai,
		unsigned int fmt);
	int (*set_tdm_slot)(struct snd_soc_dai *dai,
		unsigned int mask, int slots);
	int (*set_tristate)(struct snd_soc_dai *dai, int tristate);

	/*
	 * DAI digital mute - optional.
	 * Called by soc-core to minimise any pops.
	 */
	int (*digital_mute)(struct snd_soc_dai *dai, int mute);

	/*
	 * ALSA PCM audio operations - all optional.
	 * Called by soc-core during audio PCM operations.
	 */
	int (*startup)(struct snd_pcm_substream *,
		struct snd_soc_dai *);
	void (*shutdown)(struct snd_pcm_substream *,
		struct snd_soc_dai *);
	int (*hw_params)(struct snd_pcm_substream *,
		struct snd_pcm_hw_params *, struct snd_soc_dai *);
	int (*hw_free)(struct snd_pcm_substream *,
		struct snd_soc_dai *);
	int (*prepare)(struct snd_pcm_substream *,
		struct snd_soc_dai *);
	int (*trigger)(struct snd_pcm_substream *, int,
		struct snd_soc_dai *);

	/* AC97 bus operations - for ac97 control interface only */
	struct snd_ac97_bus_ops *ac97_ops;
};

/*
 * Digital Audio Interface runtime data.
 *
 * Holds runtime data for a DAI.
 */
struct snd_soc_dai {
	/* runtime info */
	const char *name;
	int id;
	int ac97_control; /* are we an AC97 control interface */

	/* playback and capture capabilities */
	struct snd_soc_dai_caps *playback;
	struct snd_soc_dai_caps *capture;

	struct snd_soc_dai_ops *ops;
	struct snd_pcm_runtime *runtime;
	struct device *dev;
	int active;

	/* parent codec/platform */
	union {
		struct snd_soc_codec *codec;
		struct snd_soc_platform *platform;
	};

	struct list_head list;

	/* driver and dai data */
	void *drv_data;
	void *private_data;
	void *dma_data;
};

#endif
