/*
 * wm8731.c  --  WM8731 ALSA SoC Audio driver
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@openedhand.com>
 *
 * Based on wm8753.c by Liam Girdwood
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
#include <linux/platform_device.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "wm8731.h"

#define AUDIO_NAME "wm8731"
#define WM8731_VERSION "0.20"

/*
 * Debug
 */

#define WM8731_DEBUG 0

#ifdef WM8731_DEBUG
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

/* codec private data */
struct wm8731_priv {
	unsigned int sysclk;
};

/*
 * wm8731 register cache
 * We can't read the WM8731 register space when we are
 * using 2 wire for device control, so we cache them instead.
 * There is no point in caching the reset register
 */
static const u16 wm8731_reg[WM8731_CACHEREGNUM] = {
    0x0097, 0x0097, 0x0079, 0x0079,
    0x000a, 0x0008, 0x009f, 0x000a,
    0x0000, 0x0000
};

/*
 * read wm8731 register cache
 */
static inline unsigned int wm8731_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == WM8731_RESET)
		return 0;
	if (reg >= WM8731_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write wm8731 register cache
 */
static inline void wm8731_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WM8731_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the WM8731 register space
 */
static int wm8731_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	/* data is
	 *   D15..D9 WM8731 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	wm8731_write_reg_cache (codec, reg, value);
	if (codec->mach_write(codec->control_data, (long)data, 2) == 2)
		return 0;
	else
		return -EIO;
}

#define wm8731_reset(c)	wm8731_write(c, WM8731_RESET, 0)

static const char *wm8731_input_select[] = {"Line In", "Mic"};
static const char *wm8731_deemph[] = {"None", "32Khz", "44.1Khz", "48Khz"};

static const struct soc_enum wm8731_enum[] = {
	SOC_ENUM_SINGLE(WM8731_APANA, 2, 2, wm8731_input_select),
	SOC_ENUM_SINGLE(WM8731_APDIGI, 1, 4, wm8731_deemph),
};

static const struct snd_kcontrol_new wm8731_snd_controls[] = {

SOC_DOUBLE_R("Master Playback Volume", WM8731_LOUT1V, WM8731_ROUT1V,
	0, 127, 0),
SOC_DOUBLE_R("Master Playback ZC Switch", WM8731_LOUT1V, WM8731_ROUT1V,
	7, 1, 0),

SOC_DOUBLE_R("Capture Volume", WM8731_LINVOL, WM8731_RINVOL, 0, 31, 0),
SOC_DOUBLE_R("Line Capture Switch", WM8731_LINVOL, WM8731_RINVOL, 7, 1, 1),

SOC_SINGLE("Mic Boost (+20dB)", WM8731_APANA, 0, 1, 0),
SOC_SINGLE("Capture Mic Switch", WM8731_APANA, 1, 1, 1),

SOC_SINGLE("Sidetone Playback Volume", WM8731_APANA, 6, 3, 1),

SOC_SINGLE("ADC High Pass Filter Switch", WM8731_APDIGI, 0, 1, 1),
SOC_SINGLE("Store DC Offset Switch", WM8731_APDIGI, 4, 1, 0),

SOC_ENUM("Playback De-emphasis", wm8731_enum[1]),
};

/* add non dapm controls */
static int wm8731_add_controls(struct snd_soc_codec *codec, 
	struct snd_card *card)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8731_snd_controls); i++) {
		if ((err = snd_ctl_add(card,
				snd_soc_cnew(&wm8731_snd_controls[i],
					codec, NULL))) < 0)
			return err;
	}

	return 0;
}

/* Output Mixer */
static const struct snd_kcontrol_new wm8731_output_mixer_controls[] = {
SOC_DAPM_SINGLE("Line Bypass Switch", WM8731_APANA, 3, 1, 0),
SOC_DAPM_SINGLE("Mic Sidetone Switch", WM8731_APANA, 5, 1, 0),
SOC_DAPM_SINGLE("HiFi Playback Switch", WM8731_APANA, 4, 1, 0),
};

/* Input mux */
static const struct snd_kcontrol_new wm8731_input_mux_controls =
SOC_DAPM_ENUM("Input Select", wm8731_enum[0]);

static const struct snd_soc_dapm_widget wm8731_dapm_widgets[] = {
SND_SOC_DAPM_MIXER("Output Mixer", WM8731_PWR, 4, 1,
	&wm8731_output_mixer_controls[0],
	ARRAY_SIZE(wm8731_output_mixer_controls)),
SND_SOC_DAPM_DAC("DAC", "HiFi Playback", WM8731_PWR, 3, 1),
SND_SOC_DAPM_OUTPUT("LOUT"),
SND_SOC_DAPM_OUTPUT("LHPOUT"),
SND_SOC_DAPM_OUTPUT("ROUT"),
SND_SOC_DAPM_OUTPUT("RHPOUT"),
SND_SOC_DAPM_ADC("ADC", "HiFi Capture", WM8731_PWR, 2, 1),
SND_SOC_DAPM_MUX("Input Mux", SND_SOC_NOPM, 0, 0, &wm8731_input_mux_controls),
SND_SOC_DAPM_PGA("Line Input", WM8731_PWR, 0, 1, NULL, 0),
SND_SOC_DAPM_MICBIAS("Mic Bias", WM8731_PWR, 1, 1),
SND_SOC_DAPM_INPUT("MICIN"),
SND_SOC_DAPM_INPUT("RLINEIN"),
SND_SOC_DAPM_INPUT("LLINEIN"),
};

static const char *intercon[][3] = {
	/* output mixer */
	{"Output Mixer", "Line Bypass Switch", "Line Input"},
	{"Output Mixer", "HiFi Playback Switch", "DAC"},
	{"Output Mixer", "Mic Sidetone Switch", "Mic Bias"},

	/* outputs */
	{"RHPOUT", NULL, "Output Mixer"},
	{"ROUT", NULL, "Output Mixer"},
	{"LHPOUT", NULL, "Output Mixer"},
	{"LOUT", NULL, "Output Mixer"},

	/* input mux */
	{"Input Mux", "Line In", "Line Input"},
	{"Input Mux", "Mic", "Mic Bias"},
	{"ADC", NULL, "Input Mux"},

	/* inputs */
	{"Line Input", NULL, "LLINEIN"},
	{"Line Input", NULL, "RLINEIN"},
	{"Mic Bias", NULL, "MICIN"},

	/* terminator */
	{NULL, NULL, NULL},
};

static int wm8731_add_widgets(struct snd_soc_codec *codec, 
	struct snd_soc_machine *machine)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(wm8731_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec, 
			&wm8731_dapm_widgets[i]);
	}

	/* set up audio path interconnects */
	for(i = 0; intercon[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, intercon[i][0],
			intercon[i][1], intercon[i][2]);
	}

	snd_soc_dapm_new_widgets(machine);
	return 0;
}

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:4;
	u8 bosr:1;
	u8 usb:1;
};

/* codec mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 48k */
	{12288000, 48000, 256, 0x0, 0x0, 0x0},
	{18432000, 48000, 384, 0x0, 0x1, 0x0},
	{12000000, 48000, 250, 0x0, 0x0, 0x1},

	/* 32k */
	{12288000, 32000, 384, 0x6, 0x0, 0x0},
	{18432000, 32000, 576, 0x6, 0x1, 0x0},
	{12000000, 32000, 375, 0x6, 0x0, 0x1},

	/* 8k */
	{12288000, 8000, 1536, 0x3, 0x0, 0x0},
	{18432000, 8000, 2304, 0x3, 0x1, 0x0},
	{11289600, 8000, 1408, 0xb, 0x0, 0x0},
	{16934400, 8000, 2112, 0xb, 0x1, 0x0},
	{12000000, 8000, 1500, 0x3, 0x0, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0x7, 0x0, 0x0},
	{18432000, 96000, 192, 0x7, 0x1, 0x0},
	{12000000, 96000, 125, 0x7, 0x0, 0x1},

	/* 44.1k */
	{11289600, 44100, 256, 0x8, 0x0, 0x0},
	{16934400, 44100, 384, 0x8, 0x1, 0x0},
	{12000000, 44100, 272, 0x8, 0x1, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0xf, 0x0, 0x0},
	{16934400, 88200, 192, 0xf, 0x1, 0x0},
	{12000000, 88200, 136, 0xf, 0x1, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}
	return 0;
}

static int wm8731_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;
	struct snd_soc_dai *dai = pcm_link->codec_dai;
	struct wm8731_priv *wm8731 = dai->private_data;
	u16 iface = wm8731_read_reg_cache(codec, WM8731_IFACE) & 0xfff3;
	int i = get_coeff(wm8731->sysclk, params_rate(params));
	u16 srate = (coeff_div[i].sr << 2) |
		(coeff_div[i].bosr << 1) | coeff_div[i].usb;

	wm8731_write(codec, WM8731_SRATE, srate);

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

	wm8731_write(codec, WM8731_IFACE, iface);
	return 0;
}

static int wm8731_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;

	/* set active */
	wm8731_write(codec, WM8731_ACTIVE, 0x0001);

	return 0;
}

static void wm8731_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;

	/* deactivate */
	if (!codec->active) {
		udelay(50);
		wm8731_write(codec, WM8731_ACTIVE, 0x0);
	}
}

static int wm8731_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = wm8731_read_reg_cache(codec, WM8731_APDIGI) & 0xfff7;

	if (mute)
		wm8731_write(codec, WM8731_APDIGI, mute_reg | 0x8);
	else
		wm8731_write(codec, WM8731_APDIGI, mute_reg);
	return 0;
}

static int wm8731_set_dai_sysclk(struct snd_soc_dai *dai,
		int clk_id, unsigned int freq, int dir)
{
	struct wm8731_priv *wm8731 = dai->private_data;

	switch (freq) {
	case 11289600:
	case 12000000:
	case 12288000:
	case 16934400:
	case 18432000:
		wm8731->sysclk = freq;
		return 0;
	}
	return -EINVAL;
}


static int wm8731_set_dai_fmt(struct snd_soc_dai *dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = dai->codec;
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
	wm8731_write(codec, WM8731_IFACE, iface);
	return 0;
}

static int wm8731_dapm_event(struct snd_soc_codec *codec, int event)
{
	u16 reg = wm8731_read_reg_cache(codec, WM8731_PWR) & 0xff7f;

	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		/* vref/mid, osc on, dac unmute */
		wm8731_write(codec, WM8731_PWR, reg);
		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		/* everything off except vref/vmid, */
		wm8731_write(codec, WM8731_PWR, reg | 0x0040);
		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		/* everything off, dac mute, inactive */
		wm8731_write(codec, WM8731_ACTIVE, 0x0);
		wm8731_write(codec, WM8731_PWR, 0xffff);
		break;
	}
	codec->dapm_state = event;
	return 0;
}

static int wm8731_suspend(struct device *dev, pm_message_t state)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	wm8731_write(codec, WM8731_ACTIVE, 0x0);
	wm8731_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	return 0;
}

static int wm8731_resume(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8731_reg); i++) {
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->mach_write(codec->control_data, (long)data, 2);
	}
	wm8731_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	wm8731_dapm_event(codec, codec->suspend_dapm_state);
	return 0;
}

/*
 * initialise the WM8731 codec
 */
static int wm8731_probe_codec(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	int reg;

	wm8731_reset(codec);

	/* power on device */
	wm8731_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	/* set the update bits */
	reg = wm8731_read_reg_cache(codec, WM8731_LOUT1V);
	wm8731_write(codec, WM8731_LOUT1V, reg | 0x0100);
	reg = wm8731_read_reg_cache(codec, WM8731_ROUT1V);
	wm8731_write(codec, WM8731_ROUT1V, reg | 0x0100);
	reg = wm8731_read_reg_cache(codec, WM8731_LINVOL);
	wm8731_write(codec, WM8731_LINVOL, reg | 0x0100);
	reg = wm8731_read_reg_cache(codec, WM8731_RINVOL);
	wm8731_write(codec, WM8731_RINVOL, reg | 0x0100);
	
	wm8731_add_controls(codec, machine->card);
	wm8731_add_widgets(codec, machine);

	return 0;
}

static struct snd_soc_codec_ops wm8731_codec_ops = {
	.dapm_event	= wm8731_dapm_event,
	.read		= wm8731_read_reg_cache,
	.write		= wm8731_write,
	.probe_codec	= wm8731_probe_codec,
};

static int wm8731_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	info("WM8731 Audio Codec %s", WM8731_VERSION);

	codec->reg_cache = kmemdup(wm8731_reg, sizeof(wm8731_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	codec->reg_cache_size = sizeof(wm8731_reg);
	
	codec->owner = THIS_MODULE;
	codec->ops = &wm8731_codec_ops;
	return 0;
}

static int wm8731_codec_remove(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	
	if (codec->control_data)
		wm8731_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	kfree(codec->reg_cache);
	return 0;
}

#define WM8731_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
		SNDRV_PCM_RATE_96000)

#define WM8731_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_pcm_stream wm8731_dai_playback = {
	.stream_name	= "Playback",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM8731_RATES,
	.formats	= WM8731_FORMATS,
};

static const struct snd_soc_pcm_stream wm8731_dai_capture = {
	.stream_name	= "Capture",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM8731_RATES,
	.formats	= WM8731_FORMATS,
};

/* dai ops, called by machine drivers */
static const struct snd_soc_dai_ops wm8731_dai_ops = {
	.digital_mute	= wm8731_mute,
	.set_sysclk	= wm8731_set_dai_sysclk,
	.set_fmt	= wm8731_set_dai_fmt,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops wm8731_dai_audio_ops = {
	.hw_params	= wm8731_hw_params,
	.prepare	= wm8731_prepare,
	.shutdown	= wm8731_shutdown,
};

static int wm8731_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);
	struct wm8731_priv *wm8731;
	
	wm8731 = kzalloc(sizeof(struct wm8731_priv), GFP_KERNEL);
	if (wm8731 == NULL)
		return -ENOMEM;
	
	dai->private_data = wm8731;
	dai->ops = &wm8731_dai_ops;
	dai->audio_ops = &wm8731_dai_audio_ops;
	dai->capture = &wm8731_dai_capture;
	dai->playback = &wm8731_dai_playback;
	return 0;
}

static int wm8731_dai_remove(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);
	kfree(dai->private_data);
	return 0;
}

const char wm8731_codec[SND_SOC_CODEC_NAME_SIZE] = "wm8731-codec";
EXPORT_SYMBOL_GPL(wm8731_codec);

static struct snd_soc_device_driver wm8731_codec_driver = {
	.type	= SND_SOC_BUS_TYPE_CODEC,
	.driver	= {
		.name 		= wm8731_codec,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8731_codec_probe,
		.remove		= __devexit_p(wm8731_codec_remove),
		.suspend	= wm8731_suspend,
		.resume		= wm8731_resume,
	},
};

const char wm8731_hifi_dai[SND_SOC_CODEC_NAME_SIZE] = "wm8731-hifi-dai";
EXPORT_SYMBOL_GPL(wm8731_hifi_dai);

static struct snd_soc_device_driver wm8731_hifi_dai_driver = {
	.type	= SND_SOC_BUS_TYPE_DAI,
	.driver	= {
		.name 		= wm8731_hifi_dai,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8731_dai_probe,
		.remove		= __devexit_p(wm8731_dai_remove),
	},
};

static __init int wm8731_init(void)
{
	int ret = 0;
	
	ret = driver_register(&wm8731_codec_driver.driver);
	if (ret < 0)
		return ret;
	ret = driver_register(&wm8731_hifi_dai_driver.driver);
	if (ret < 0) {
		driver_unregister(&wm8731_codec_driver.driver);
		return ret;
	}
	return ret;
}

static __exit void wm8731_exit(void)
{
	driver_unregister(&wm8731_hifi_dai_driver.driver);
	driver_unregister(&wm8731_codec_driver.driver);
}

module_init(wm8731_init);
module_exit(wm8731_exit);

MODULE_DESCRIPTION("ASoC WM8731 driver");
MODULE_AUTHOR("Richard Purdie");
MODULE_LICENSE("GPL");
