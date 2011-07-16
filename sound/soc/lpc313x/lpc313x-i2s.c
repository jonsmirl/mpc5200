/*
 * sound/soc/lpc313x/lpc313x-i2s.c
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

#include <linux/init.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/initval.h>
#include <sound/soc.h>

#include <mach/cgu.h>
#include <mach/board.h>
#include <mach/registers.h>

#include "lpc313x-i2s-clocking.h"
#include "lpc313x-pcm.h"
#include "lpc313x-i2s.h"

#define I2S_NAME "lpc313x-i2s"

/* All major audio rates are support and 16-bit I2S data is supported */
#define LPC313X_I2S_RATES \
    (SNDRV_PCM_RATE_8000  | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
     SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
     SNDRV_PCM_RATE_48000)
#define LPC313X_I2S_FORMATS (SNDRV_PCM_FMTBIT_S16)

#define CH_PLAY 0
#define CH_REC  1

/* Structure that keeps I2S direction data */
struct lpc313x_i2s_ch_info {
	char *name;                 /* Name of this channel */
	unsigned short ch_on;       /* Flag used to indicate if clocks are on */
	unsigned short daifmt;
	u32 ws_freq;
	int i2s_ch;
	enum i2s_supp_clks chclk;
};

/* Common I2S structure data */
struct lpc313x_i2s_info {
	spinlock_t lock;
	unsigned short initialized;
	struct lpc313x_i2s_ch_info ch_info[2];
	u32 freq;
};

/* Common I2S structure data */
static struct lpc313x_i2s_info i2s_info =
{
	.lock = __SPIN_LOCK_UNLOCKED(i2s_info.lock),
	.initialized = 0,
	.ch_info = 
	{
		[0] =
		{
#if defined (CONFIG_SND_I2S_TX0_MASTER)
			.name  = "i2s0_play",
			.chclk = CLK_TX_0,
			.i2s_ch = I2S_CH_TX0,
#else
			.name  = "i2s1_play",
			.chclk = CLK_TX_1,
			.i2s_ch = I2S_CH_TX1,
#endif
			.ch_on = 0,
		},
		[1] =
		{
#if defined (CONFIG_SND_I2S_RX0_MASTER)
			.name  = "i2s0_mrec",
			.chclk = CLK_RX_0,
			.i2s_ch = I2S_CH_RX0,
#endif
#if defined (CONFIG_SND_I2S_RX1_MASTER)
			.name  = "i2s1_mrec",
			.chclk = CLK_RX_1,
			.i2s_ch = I2S_CH_RX1,
#endif
#if defined (SND_I2S_RX0_SLAVE)
			.name  = "i2s0_srec",
			/* Not supported yet, generate an error */
			.i2s_ch = I2S_CH_RX0,
#error
#endif
#if defined (SND_I2S_RX1_SLAVE)
			.name  = "i2s1_srec",
			/* Not supported yet, generate an error */
			.i2s_ch = I2S_CH_RX1,
#error
#endif
			.ch_on = 0,
		},
	},
};

static inline int lpc313x_get_ch_dir(struct snd_pcm_substream *substream)
{
	int dir = CH_REC;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dir = CH_PLAY;
	}

	return dir;
}

static void lpc313x_i2s_shutdown(struct snd_pcm_substream *substream,
									struct snd_soc_dai *dai)
{
	int dir = lpc313x_get_ch_dir(substream);

	if (i2s_info.ch_info[dir].ch_on == 0) {
		/* This channel is not enabled! */
		pr_warning("%s: I2S channel is not on!\n", i2s_info.ch_info[dir].name);
		return;
	}

	/* Channel specific shutdown */
	lpc313x_chan_clk_enable(i2s_info.ch_info[dir].chclk, 0, 0);
	i2s_info.ch_info[dir].ch_on = 0;

	/* Can we shutdown I2S interface to save some power? */
	if (i2s_info.ch_info[1 - dir].ch_on != 0) {
		/* Other channel is busy, exit without shutting down main clock */
		return;
	}

	/* Safe to shut down */
	if (i2s_info.initialized == 0) {
		/* Nothing is enabled! */
		pr_warning("I2S shutdown (%s) when nothing is enabled!\n",
			i2s_info.ch_info[dir].name);
		return;
	}

	/* Disable I2S register access clock */
	cgu_clk_en_dis(CGU_SB_I2S_CFG_PCLK_ID, 0);
	cgu_clk_en_dis(CGU_SB_EDGE_DET_PCLK_ID, 0);

	i2s_info.initialized = 0;
}

static int lpc313x_i2s_startup(struct snd_pcm_substream *substream,
								struct snd_soc_dai *dai)
{
	int dir = lpc313x_get_ch_dir(substream);

	/* Select master/slave mode for RX channel */
	if (dir == CH_REC) {
#if defined (CONFIG_SND_I2S_RX0_SLAVE) | defined (CONFIG_SND_I2S_RX1_SLAVE)
		I2S_CFG_MUX_SETTINGS = 0;
#endif
#if defined (CONFIG_SND_I2S_RX0_MASTER)
		I2S_CFG_MUX_SETTINGS = I2S_RXO_SELECT_MASTER;
#endif
#if defined (CONFIG_SND_I2S_RX1_MASTER)
		I2S_CFG_MUX_SETTINGS = I2S_RX1_SELECT_MASTER;
#endif
	}

	if (i2s_info.ch_info[dir].ch_on != 0) {
		/* This channel already enabled! */
		pr_warning("%s: I2S channel is busy!\n", i2s_info.ch_info[dir].name);
		return -EBUSY;
	}

	/* Initialize I2S interface */
	if (i2s_info.initialized == 0) {
		/* Enable I2S register access clock */
		cgu_clk_en_dis(CGU_SB_I2S_CFG_PCLK_ID, 1);
		cgu_clk_en_dis(CGU_SB_EDGE_DET_PCLK_ID, 1);

		i2s_info.initialized = 1;
	}

	/* Channel specific init, ok to leave the clocks off for now */
	i2s_info.ch_info[dir].ch_on = 1;
	lpc313x_chan_clk_enable(i2s_info.ch_info[dir].chclk, 0, 0);

	/* Mask all interrupts for the I2S channel */
	I2S_CH_INT_MASK(i2s_info.ch_info[dir].i2s_ch) = I2S_FIFO_ALL_MASK;

	return 0;
}

static int lpc313x_i2s_set_dai_sysclk(struct snd_soc_dai *cpu_dai,
				      int clk_id, unsigned int freq, int dir)
{
	/* Will use in HW params later */
//	i2s_info.ch_info[cpu_dai->id].ws_freq = freq;
	i2s_info.ch_info[CH_REC].ws_freq = freq;
	i2s_info.ch_info[CH_PLAY].ws_freq = freq;
	return 0;
}

static int lpc313x_i2s_set_dai_fmt(struct snd_soc_dai *cpu_dai,
				   unsigned int fmt)
{
	/* Will use in HW params later */
//	i2s_info.ch_info[cpu_dai->id].daifmt = fmt;
	i2s_info.ch_info[CH_REC].daifmt = fmt;
	i2s_info.ch_info[CH_PLAY].daifmt = fmt;

	return 0;
}

static int lpc313x_i2s_set_dai_clkdiv(struct snd_soc_dai *cpu_dai,
				      int div_id, int div)
{
	/* This function isn't used */
	(void) cpu_dai;
	(void) div_id;
	(void) div;

	return 0;
}

static int lpc313x_i2s_hw_params(struct snd_pcm_substream *substream,
			         struct snd_pcm_hw_params *params,
					 struct snd_soc_dai *dai)
{
	int dir = lpc313x_get_ch_dir(substream);
	u32 tmp;

	/* Setup the I2S data format */
	tmp = 0;
	switch (i2s_info.ch_info[dir].daifmt & SND_SOC_DAIFMT_FORMAT_MASK) {
		case SND_SOC_DAIFMT_I2S:
			spin_lock_irq(&i2s_info.lock);
			tmp = I2S_FORMAT_SETTINGS &
				~I2S_SET_FORMAT(i2s_info.ch_info[dir].i2s_ch,
				I2S_FORMAT_MASK);
			I2S_FORMAT_SETTINGS = tmp | I2S_SET_FORMAT(i2s_info.ch_info[dir].i2s_ch,
				I2S_FORMAT_I2S);
			spin_unlock_irq(&i2s_info.lock);
			break;

		default:
			pr_warning("%s: Unsupported audio data format\n",
				i2s_info.ch_info[dir].name);
			return -EINVAL;
	}

#if defined (CONFIG_SND_DEBUG_VERBOSE)
	pr_info("Desired clock rate : %d\n", i2s_info.ch_info[dir].ws_freq);
	pr_info("Channels           : %d\n", params_channels(params));
	pr_info("Data format        : %d\n", i2s_info.ch_info[dir].daifmt);
#endif

	/* The playback and record rates are shared, so just set the CODEC clock
	   to the selected rate (will actually generate 256 * rate) */
	i2s_info.freq = i2s_info.ch_info[dir].ws_freq;
	if (lpc313x_main_clk_rate(i2s_info.freq) == 0)
	{
		pr_warning("Unsupported audio data rate (%d)\n",
			i2s_info.freq);
		return -EINVAL;
	}

	/* Now setup the selected channel clocks (WS and BCK) */
	if (lpc313x_chan_clk_enable(i2s_info.ch_info[dir].chclk, i2s_info.freq,
		(i2s_info.freq * 32)) == 0)
	{
		pr_warning("Unsupported channel data rates (ws=%d, bck=%d)\n",
			i2s_info.freq, (i2s_info.freq * 32));
		return -EINVAL;
	}

	return 0;
}

static int lpc313x_i2s_prepare(struct snd_pcm_substream *substream)
{
	/* Nothing to do here */
	return 0;
}


static int lpc313x_i2s_trigger(struct snd_pcm_substream *substream, int cmd)
{
	int ret = 0;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_RESUME:
		break;

	default:
		pr_warning("lpc313x_i2s_triggers: Unsupported cmd: %d\n",
				cmd);
		ret = -EINVAL;
	}

	return ret;
}

#ifdef CONFIG_PM
static int lpc313x_i2s_suspend(struct snd_soc_dai *cpu_dai)
{
	/* Shutdown active clocks */
	if (i2s_info.ch_info[CH_PLAY].ch_on != 0) {
		lpc313x_chan_clk_enable(i2s_info.ch_info[CH_PLAY].chclk, 0, 0);
	}
	if (i2s_info.ch_info[CH_REC].ch_on != 0) {
		lpc313x_chan_clk_enable(i2s_info.ch_info[CH_REC].chclk, 0, 0);
	}

	/* Disable I2S register access clock */
	cgu_clk_en_dis(CGU_SB_I2S_CFG_PCLK_ID, 0);
	cgu_clk_en_dis(CGU_SB_EDGE_DET_PCLK_ID, 0);
	/* shutdown main clocks */
	lpc313x_main_clk_rate(0);

	return 0;
}

static int lpc313x_i2s_resume(struct snd_soc_dai *cpu_dai)
{
	/* resume main clocks */
	lpc313x_main_clk_rate(i2s_info.freq);
	/* Enable I2S register access clock */
	cgu_clk_en_dis(CGU_SB_I2S_CFG_PCLK_ID, 1);
	cgu_clk_en_dis(CGU_SB_EDGE_DET_PCLK_ID, 1);

	/* resume active clocks */
	if (i2s_info.ch_info[CH_PLAY].ch_on != 0) {
		lpc313x_chan_clk_enable(i2s_info.ch_info[CH_PLAY].chclk,
		i2s_info.ch_info[CH_PLAY].ws_freq, (i2s_info.freq * 32));
	}
	if (i2s_info.ch_info[CH_REC].ch_on != 0) {
		lpc313x_chan_clk_enable(i2s_info.ch_info[CH_REC].chclk,
		i2s_info.ch_info[CH_REC].ws_freq, (i2s_info.freq * 32));
	}

	return 0;
}

#else
#define lpc313x_i2s_suspend	NULL
#define lpc313x_i2s_resume	NULL
#endif

static struct snd_soc_dai_ops lpc313x_i2s_dai_ops = {
		.startup = lpc313x_i2s_startup,
		.shutdown = lpc313x_i2s_shutdown,
		.trigger = lpc313x_i2s_trigger,
		.hw_params = lpc313x_i2s_hw_params,
		.set_sysclk = lpc313x_i2s_set_dai_sysclk,
		.set_fmt = lpc313x_i2s_set_dai_fmt,
		.set_clkdiv = lpc313x_i2s_set_dai_clkdiv,		
};

struct snd_soc_dai lpc313x_i2s_dai = {
	 .name = I2S_NAME,
	 .id = 0,
	 .suspend = lpc313x_i2s_suspend,
	 .resume = lpc313x_i2s_resume,
	 .playback = {
		      .channels_min = 2,
		      .channels_max = 2,
		      .rates = LPC313X_I2S_RATES,
		      .formats = LPC313X_I2S_FORMATS,
		      },
	 .capture = {
		     .channels_min = 2,
		     .channels_max = 2,
		     .rates = LPC313X_I2S_RATES,
		     .formats = LPC313X_I2S_FORMATS,
		     },
	 .ops = &lpc313x_i2s_dai_ops,
	 .private_data = &i2s_info,
	.symmetric_rates = 1,
};
EXPORT_SYMBOL_GPL(lpc313x_i2s_dai);

static int __init lpc313x_i2s_init(void)
{
	return snd_soc_register_dai(&lpc313x_i2s_dai);
}
module_init(lpc313x_i2s_init);

static void __exit lpc313x_i2s_exit(void)
{
	snd_soc_unregister_dai(&lpc313x_i2s_dai);
}
module_exit(lpc313x_i2s_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("ASoC LPC313X I2S interface");
MODULE_LICENSE("GPL");

