/*
 * wm8731.c  --  WM8731 ALSA SoC Audio driver
 *
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Richard Purdie <richard@openedhand.com>
 *
 * Based on wm8731.c by Liam Girdwood
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
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "wm8731.h"

#define AUDIO_NAME "wm8731"
#define WM8731_VERSION "0.21"

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
struct wm8731_data {
	unsigned int sysclk;
	struct snd_soc_dai *dai;
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
	if (codec->soc_phys_write(codec->control_data, (long)data, 2) == 2)
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

static const struct snd_soc_dapm_route audio_map[] = {
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
};

static int wm8731_add_widgets(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card)
{
	int ret;

	ret = snd_soc_dapm_new_controls(soc_card, codec,
					wm8731_dapm_widgets,
					ARRAY_SIZE(wm8731_dapm_widgets));
	if (ret < 0)
		return ret;

	/* set up audio path audio_map */
	ret = snd_soc_dapm_add_routes(soc_card, audio_map,
				     ARRAY_SIZE(audio_map));
	if (ret < 0)
		return ret;

	return snd_soc_dapm_init(soc_card);
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
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct wm8731_data *wm8731 = dai->codec->private_data;
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

static int wm8731_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	/* set active */
	wm8731_write(codec, WM8731_ACTIVE, 0x0001);

	return 0;
}

static void wm8731_shutdown(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

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
	struct wm8731_data *wm8731 = dai->codec->private_data;

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

static int wm8731_set_bias_level(struct snd_soc_codec *codec, 
	enum snd_soc_dapm_bias_level level)
{
	u16 reg = wm8731_read_reg_cache(codec, WM8731_PWR) & 0xff7f;

	switch (level) {
	case SND_SOC_BIAS_ON: /* full On */
		/* vref/mid, osc on, dac unmute */
		wm8731_write(codec, WM8731_PWR, reg);
		break;
	case SND_SOC_BIAS_PREPARE: /* partial On */
		break;
	case SND_SOC_BIAS_STANDBY: /* Off, with power */
		/* everything off except vref/vmid, */
		wm8731_write(codec, WM8731_PWR, reg | 0x0040);
		break;
	case SND_SOC_BIAS_OFF: /* Off, without power */
		/* everything off, dac mute, inactive */
		wm8731_write(codec, WM8731_ACTIVE, 0x0);
		wm8731_write(codec, WM8731_PWR, 0xffff);
		break;
	}
	codec->bias_level = level;
	return 0;
}

static int wm8731_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_codec *codec = platform_get_drvdata(pdev);

	wm8731_write(codec, WM8731_ACTIVE, 0x0);
	wm8731_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int wm8731_resume(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = platform_get_drvdata(pdev);
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8731_reg); i++) {
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->soc_phys_write(codec->control_data, (long)data, 2);
	}
	wm8731_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	wm8731_set_bias_level(codec, codec->suspend_bias_level);
	return 0;
}

/*
 * initialise the WM8731 codec
 */
static int wm8731_codec_init(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card)
{
	int reg;

	wm8731_reset(codec);

	/* power on device */
	wm8731_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* set the update bits */
	reg = wm8731_read_reg_cache(codec, WM8731_LOUT1V);
	wm8731_write(codec, WM8731_LOUT1V, reg & ~0x0100);
	reg = wm8731_read_reg_cache(codec, WM8731_ROUT1V);
	wm8731_write(codec, WM8731_ROUT1V, reg & ~0x0100);
	reg = wm8731_read_reg_cache(codec, WM8731_LINVOL);
	wm8731_write(codec, WM8731_LINVOL, reg & ~0x0100);
	reg = wm8731_read_reg_cache(codec, WM8731_RINVOL);
	wm8731_write(codec, WM8731_RINVOL, reg & ~0x0100);
		
	snd_soc_add_new_controls(soc_card, wm8731_snd_controls, codec,
		ARRAY_SIZE(wm8731_snd_controls));
	wm8731_add_widgets(codec, soc_card);

	return 0;
}

static void wm8731_codec_exit(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card)
{
	wm8731_set_bias_level(codec, SND_SOC_BIAS_OFF);
}

#define WM8731_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
		SNDRV_PCM_RATE_96000)

#define WM8731_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_caps wm8731_playback = {
	.stream_name	= "Playback",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM8731_RATES,
	.formats	= WM8731_FORMATS,
};

static struct snd_soc_dai_caps wm8731_capture = {
	.stream_name	= "Capture",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM8731_RATES,
	.formats	= WM8731_FORMATS,
};

static struct snd_soc_dai_ops wm8731_dai_ops = {
	/* alsa ops */
	.hw_params	= wm8731_hw_params,
	.prepare	= wm8731_prepare,
	.shutdown	= wm8731_shutdown,
	
	/* dai ops */
	.digital_mute	= wm8731_mute,
	.set_sysclk	= wm8731_set_dai_sysclk,
	.set_fmt	= wm8731_set_dai_fmt,
};

/* for modprobe */
const char wm8731_codec_id[] = "wm8731-codec";
EXPORT_SYMBOL_GPL(wm8731_codec_id);

const char wm8731_codec_dai_id[] = "wm8731-codec-dai";
EXPORT_SYMBOL_GPL(wm8731_codec_dai_id);

struct snd_soc_dai_new wm8731_hifi_dai = {
	.name		= wm8731_codec_dai_id,
	.playback	= &wm8731_playback,
	.capture	= &wm8731_capture,
	.ops		= &wm8731_dai_ops,
};

static struct snd_soc_codec_new wm8731_codec = {
	.name		= wm8731_codec_id,
	.reg_cache_size = sizeof(wm8731_reg),
	.reg_cache_step = 1,
	.set_bias_level	= wm8731_set_bias_level,
	.init		= wm8731_codec_init,
	.exit		= wm8731_codec_exit,
	.codec_read	= wm8731_read_reg_cache,
	.codec_write	= wm8731_write,
};

static int wm8731_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_codec *codec;
	struct wm8731_data *wm8731;
	int ret;

	info("WM8731 Audio Codec %s", WM8731_VERSION);

	codec = snd_soc_new_codec(&wm8731_codec, (char *) wm8731_reg);
	if (codec == NULL)
		return -ENOMEM;

	wm8731 = kzalloc(sizeof(struct wm8731_data), GFP_KERNEL);
	if (wm8731 == NULL) {
		ret = -ENOMEM;
		goto wm8731_err;
	}
	codec->private_data = wm8731;
	platform_set_drvdata(pdev, codec);
		
	ret = snd_soc_register_codec(codec, &pdev->dev);
	if (ret < 0)
		goto codec_err;
	wm8731->dai = snd_soc_register_codec_dai(&wm8731_hifi_dai, &pdev->dev);
	if (wm8731->dai == NULL)
		goto codec_err;
	return ret;

codec_err:
	kfree(wm8731);
wm8731_err:
	snd_soc_unregister_codec(codec);
	kfree(codec->reg_cache);
	kfree(codec);
	return ret;
}

static int wm8731_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = platform_get_drvdata(pdev);
	struct wm8731_data *wm8731 = codec->private_data;
	
	snd_soc_unregister_codec_dai(wm8731->dai);
	kfree(wm8731);
	snd_soc_unregister_codec(codec);
	kfree(codec->reg_cache);
	kfree(codec);
	return 0;
}

static struct platform_driver wm8731_codec_driver = {
	.driver = {
		.name		= wm8731_codec_id,
		.owner		= THIS_MODULE,
	},
	.probe		= wm8731_codec_probe,
	.remove		= __devexit_p(wm8731_codec_remove),
	.suspend	= wm8731_suspend,
	.resume		= wm8731_resume,
};

static __init int wm8731_init(void)
{
	return platform_driver_register(&wm8731_codec_driver);
}

static __exit void wm8731_exit(void)
{
	platform_driver_unregister(&wm8731_codec_driver);
}

module_init(wm8731_init);
module_exit(wm8731_exit);

MODULE_DESCRIPTION("ASoC WM8731 driver");
MODULE_AUTHOR("Richard Purdie");
MODULE_LICENSE("GPL");
