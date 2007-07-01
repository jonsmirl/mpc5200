/*
 * wm8956.c  --  WM8956 ALSA SoC Audio driver
 *
 * Author: Liam Girdwood
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "wm8956.h"

#define AUDIO_NAME "wm8956"
#define WM8956_VERSION "0.2"

/*
 * Debug
 */

#define WM8956_DEBUG 0

#ifdef WM8956_DEBUG
#define dbg(format, arg...) \
	printk(KERN_DEBUG AUDIO_NAME ": " format "\n" , ## arg)
#else
#define dbg(format, arg...) do {} while (0)
#endif
#define err(format, arg...) \
	printk(KERN_ERR AUDIO_NAME ": " format "\n" , ## arg)
#define info(format, arg...) \
	printk(KERN_INFO AUDIO_NAME ": " format "\n" , ## arg)
#define warn(format, arg...) \
	printk(KERN_WARNING AUDIO_NAME ": " format "\n" , ## arg)


/*
 * wm8956 register cache
 * We can't read the WM8956 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8956_reg[WM8956_CACHEREGNUM] = {
	0x0097, 0x0097, 0x0000, 0x0000,
	0x0000, 0x0008, 0x0000, 0x000a,
	0x01c0, 0x0000, 0x00ff, 0x00ff,
	0x0000, 0x0000, 0x0000, 0x0000, //r15
	0x0000, 0x007b, 0x0100, 0x0032,
	0x0000, 0x00c3, 0x00c3, 0x01c0,
	0x0000, 0x0000, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0000, 0x0000, //r31
	0x0100, 0x0100, 0x0050, 0x0050,
	0x0050, 0x0050, 0x0000, 0x0000,
	0x0000, 0x0000, 0x0040, 0x0000,
	0x0000, 0x0050, 0x0050, 0x0000, //47
	0x0002, 0x0037, 0x004d, 0x0080,
	0x0008, 0x0031, 0x0026, 0x00e9,
};

/*
 * read wm8956 register cache
 */
static inline unsigned int wm8956_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == WM8956_RESET)
		return 0;
	if (reg >= WM8956_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write wm8956 register cache
 */
static inline void wm8956_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WM8956_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the WM8956 register space
 */
static int wm8956_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	/* data is
	 *   D15..D9 WM8956 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	wm8956_write_reg_cache (codec, reg, value);
	if (codec->mach_write(codec->control_data, (long)data, 2) == 2)
		return 0;
	else
		return -EIO;
}

#define wm8956_reset(c)	wm8956_write(c, WM8956_RESET, 0)

/* todo - complete enumerated controls */
static const char *wm8956_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};

static const struct soc_enum wm8956_enum[] = {
	SOC_ENUM_SINGLE(WM8956_DACCTL1, 1, 4, wm8956_deemph),
};

/* to complete */
static const struct snd_kcontrol_new wm8956_snd_controls[] = {

SOC_DOUBLE_R("Headphone Playback Volume", WM8956_LOUT1, WM8956_ROUT1,
	0, 127, 0),
SOC_DOUBLE_R("Headphone Playback ZC Switch", WM8956_LOUT1, WM8956_ROUT1,
	7, 1, 0),
SOC_DOUBLE_R("PCM Volume", WM8956_LDAC, WM8956_RDAC,
	0, 127, 0),

SOC_ENUM("Playback De-emphasis", wm8956_enum[0]),
};

/* add non dapm controls */
static int wm8956_add_controls(struct snd_soc_codec *codec, 
	struct snd_card *card)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8956_snd_controls); i++) {
		if ((err = snd_ctl_add(card,
				snd_soc_cnew(&wm8956_snd_controls[i],
					codec, NULL))) < 0)
			return err;
	}

	return 0;
}

/* Left Output Mixer */
static const struct snd_kcontrol_new wm8956_loutput_mixer_controls[] = {
SOC_DAPM_SINGLE("Left PCM Playback Switch", WM8956_LOUTMIX1, 8, 1, 0),
};

/* Right Output Mixer */
static const struct snd_kcontrol_new wm8956_routput_mixer_controls[] = {
SOC_DAPM_SINGLE("Right PCM Playback Switch", WM8956_ROUTMIX2, 8, 1, 0),
};

static const struct snd_soc_dapm_widget wm8956_dapm_widgets[] = {
SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
	&wm8956_loutput_mixer_controls[0],
	ARRAY_SIZE(wm8956_loutput_mixer_controls)),
SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
	&wm8956_loutput_mixer_controls[0],
	ARRAY_SIZE(wm8956_routput_mixer_controls)),
};

static const char *intercon[][3] = {
	/* TODO */
	/* terminator */
	{NULL, NULL, NULL},
};

static int wm8956_add_widgets(struct snd_soc_codec *codec, 
	struct snd_soc_machine *machine)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(wm8956_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec, 
			&wm8956_dapm_widgets[i]);
	}

	/* set up audio path interconnects */
	for(i = 0; intercon[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, intercon[i][0],
			intercon[i][1], intercon[i][2]);
	}

	snd_soc_dapm_new_widgets(machine);
	return 0;
}

static int wm8956_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		iface |= 0x0040;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		iface |= 0x0013;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0090;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	/* set iface */
	wm8956_write(codec, WM8956_IFACE1, iface);
	return 0;
}

static int wm8956_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;
	u16 iface = wm8956_read_reg_cache(codec, WM8956_IFACE1) & 0xfff3;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0008;
		break;
	}

	/* set iface */
	wm8956_write(codec, WM8956_IFACE1, iface);
	return 0;
}

static int wm8956_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = wm8956_read_reg_cache(codec, WM8956_DACCTL1) & 0xfff7;

	if (mute)
		wm8956_write(codec, WM8956_DACCTL1, mute_reg | 0x8);
	else
		wm8956_write(codec, WM8956_DACCTL1, mute_reg);
	return 0;
}

static int wm8956_dapm_event(struct snd_soc_codec *codec, int event)
{
#if 0
	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		/* vref/mid, osc on, dac unmute */

		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		/* everything off except vref/vmid, */
		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		/* everything off, dac mute, inactive */
		break;
	}
#endif
	// tmp
	wm8956_write(codec, WM8956_POWER1, 0xfffe);
	wm8956_write(codec, WM8956_POWER2, 0xffff);
	wm8956_write(codec, WM8956_POWER3, 0xffff);
	codec->dapm_state = event;
	return 0;
}

/* PLL divisors */
struct _pll_div {
	u32 pre_div:1;
	u32 n:4;
	u32 k:24;
};

static struct _pll_div pll_div;

/* The size in bits of the pll divide multiplied by 10
 * to allow rounding later */
#define FIXED_PLL_SIZE ((1 << 24) * 10)

static void pll_factors(unsigned int target, unsigned int source)
{
	unsigned long long Kpart;
	unsigned int K, Ndiv, Nmod;

	Ndiv = target / source;
	if (Ndiv < 6) {
		source >>= 1;
		pll_div.pre_div = 1;
		Ndiv = target / source;
	} else
		pll_div.pre_div = 0;

	if ((Ndiv < 6) || (Ndiv > 12))
		printk(KERN_WARNING
			"WM8956 N value outwith recommended range! N = %d\n",Ndiv);

	pll_div.n = Ndiv;
	Nmod = target % source;
	Kpart = FIXED_PLL_SIZE * (long long)Nmod;

	do_div(Kpart, source);

	K = Kpart & 0xFFFFFFFF;

	/* Check if we need to round */
	if ((K % 10) >= 5)
		K += 5;

	/* Move down to proper range now rounding is done */
	K /= 10;

	pll_div.k = K;
}

static int wm8956_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;
	int found = 0;
#if 0
	if (freq_in == 0 || freq_out == 0) {
		/* disable the pll */
		/* turn PLL power off */
	}
#endif

	pll_factors(freq_out * 8, freq_in);

	if (!found)
		return -EINVAL;

	reg = wm8956_read_reg_cache(codec, WM8956_PLLN) & 0x1e0;
	wm8956_write(codec, WM8956_PLLN, reg | (1<<5) | (pll_div.pre_div << 4)
		| pll_div.n);
	wm8956_write(codec, WM8956_PLLK1, pll_div.k >> 16 );
	wm8956_write(codec, WM8956_PLLK2, (pll_div.k >> 8) & 0xff);
	wm8956_write(codec, WM8956_PLLK3, pll_div.k &0xff);
	wm8956_write(codec, WM8956_CLOCK1, 4);

	return 0;
}

static int wm8956_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	switch (div_id) {
	case WM8956_SYSCLKSEL:
		reg = wm8956_read_reg_cache(codec, WM8956_CLOCK1) & 0x1fe;
		wm8956_write(codec, WM8956_CLOCK1, reg | div);
		break;
	case WM8956_SYSCLKDIV:
		reg = wm8956_read_reg_cache(codec, WM8956_CLOCK1) & 0x1f9;
		wm8956_write(codec, WM8956_CLOCK1, reg | div);
		break;
	case WM8956_DACDIV:
		reg = wm8956_read_reg_cache(codec, WM8956_CLOCK1) & 0x1c7;
		wm8956_write(codec, WM8956_CLOCK1, reg | div);
		break;
	case WM8956_OPCLKDIV:
		reg = wm8956_read_reg_cache(codec, WM8956_PLLN) & 0x03f;
		wm8956_write(codec, WM8956_PLLN, reg | div);
		break;
	case WM8956_DCLKDIV:
		reg = wm8956_read_reg_cache(codec, WM8956_CLOCK2) & 0x03f;
		wm8956_write(codec, WM8956_CLOCK2, reg | div);
		break;
	case WM8956_TOCLKSEL:
		reg = wm8956_read_reg_cache(codec, WM8956_ADDCTL1) & 0x1fd;
		wm8956_write(codec, WM8956_ADDCTL1, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

#define WM8956_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000)

#define WM8956_FORMATS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE)

/* To complete PM */
static int wm8956_suspend(struct device *dev, pm_message_t state)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	wm8956_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	return 0;
}

static int wm8956_resume(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8956_reg); i++) {
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->mach_write(codec->control_data, (long)data, 2);
	}
	wm8956_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	wm8956_dapm_event(codec, codec->suspend_dapm_state);
	return 0;
}

/*
 * initialise the WM8956 codec
 */
static int wm8956_codec_io_probe(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	u16 reg;
	wm8956_reset(codec);

	/* power on device */
	wm8956_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	/*  set the update bits */
	reg = wm8956_read_reg_cache(codec, WM8956_LOUT1);
	wm8956_write(codec, WM8956_LOUT1, reg | 0x0100);
	reg = wm8956_read_reg_cache(codec, WM8956_ROUT1);
	wm8956_write(codec, WM8956_ROUT1, reg | 0x0100);
	
	wm8956_add_controls(codec, machine->card);
	wm8956_add_widgets(codec, machine);

	return 0;
}

static struct snd_soc_codec_ops wm8956_codec_ops = {
	.dapm_event	= wm8956_dapm_event,
	.read		= wm8956_read_reg_cache,
	.write		= wm8956_write,
	.io_probe	= wm8956_codec_io_probe,
};

static int wm8956_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	info("WM8956 Audio Codec %s", WM8956_VERSION);

	codec->reg_cache = kmemdup(wm8956_reg, sizeof(wm8956_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	codec->reg_cache_size = sizeof(wm8956_reg);
	
	codec->owner = THIS_MODULE;
	codec->ops = &wm8956_codec_ops;
	return 0;
}

static int wm8956_codec_remove(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	
	if (codec->control_data)
		wm8956_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	kfree(codec->reg_cache);
	return 0;
}

#define WM8956_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
		SNDRV_PCM_RATE_96000)

#define WM8956_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_pcm_stream wm8956_dai_playback = {
	.stream_name	= "Playback",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM8956_RATES,
	.formats	= WM8956_FORMATS,
};

static const struct snd_soc_pcm_stream wm8956_dai_capture = {
	.stream_name	= "Capture",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM8956_RATES,
	.formats	= WM8956_FORMATS,
};

/* dai ops, called by machine drivers */
static const struct snd_soc_dai_ops wm8956_dai_ops = {
	.digital_mute	= wm8956_digital_mute,
	.set_fmt	= wm8956_set_dai_fmt,
	.set_pll	= wm8956_set_dai_pll,
	.set_clkdiv	= wm8956_set_dai_clkdiv,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops wm8956_dai_audio_ops = {
	.hw_params	= wm8956_hw_params,
};

static int wm8956_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);

	dai->ops = &wm8956_dai_ops;
	dai->audio_ops = &wm8956_dai_audio_ops;
	dai->capture = &wm8956_dai_capture;
	dai->playback = &wm8956_dai_playback;
	return 0;
}

const char wm8956_codec[SND_SOC_CODEC_NAME_SIZE] = "wm8956-codec";
EXPORT_SYMBOL_GPL(wm8956_codec);

static struct snd_soc_device_driver wm8956_codec_driver = {
	.type	= SND_SOC_BUS_TYPE_CODEC,
	.driver	= {
		.name 		= wm8956_codec,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8956_codec_probe,
		.remove		= __devexit_p(wm8956_codec_remove),
		.suspend	= wm8956_suspend,
		.resume		= wm8956_resume,
	},
};

const char wm8956_hifi_dai[SND_SOC_CODEC_NAME_SIZE] = "wm8956-hifi-dai";
EXPORT_SYMBOL_GPL(wm8956_hifi_dai);

static struct snd_soc_device_driver wm8956_hifi_dai_driver = {
	.type	= SND_SOC_BUS_TYPE_DAI,
	.driver	= {
		.name 		= wm8956_hifi_dai,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8956_dai_probe,
	},
};

static __init int wm8956_init(void)
{
	int ret = 0;
	
	ret = driver_register(&wm8956_codec_driver.driver);
	if (ret < 0)
		return ret;
	ret = driver_register(&wm8956_hifi_dai_driver.driver);
	if (ret < 0) {
		driver_unregister(&wm8956_codec_driver.driver);
		return ret;
	}
	return ret;
}

static __exit void wm8956_exit(void)
{
	driver_unregister(&wm8956_hifi_dai_driver.driver);
	driver_unregister(&wm8956_codec_driver.driver);
}

module_init(wm8956_init);
module_exit(wm8956_exit);


MODULE_DESCRIPTION("ASoC WM8956 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
