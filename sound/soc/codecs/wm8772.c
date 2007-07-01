/*
 * wm8772.c  --  WM8772 ALSA Soc Audio driver
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "wm8772.h"

#define AUDIO_NAME "WM8772"
#define WM8772_VERSION "0.4"

/* codec private data */
struct wm8772_priv {
	unsigned int adcclk;
	unsigned int dacclk;
};

/*
 * wm8772 register cache
 * We can't read the WM8772 register space when we
 * are using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8772_reg[] = {
	0x00ff, 0x00ff, 0x0120, 0x0000,  /*  0 */
	0x00ff, 0x00ff, 0x00ff, 0x00ff,  /*  4 */
	0x00ff, 0x0000, 0x0080, 0x0040,  /*  8 */
	0x0000
};

/*
 * read wm8772 register cache
 */
static inline unsigned int wm8772_read_reg_cache(struct snd_soc_codec * codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg > WM8772_CACHE_REGNUM)
		return -1;
	return cache[reg];
}

/*
 * write wm8772 register cache
 */
static inline void wm8772_write_reg_cache(struct snd_soc_codec * codec,
	unsigned int reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg > WM8772_CACHE_REGNUM)
		return;
	cache[reg] = value;
}

static int wm8772_write(struct snd_soc_codec * codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	/* data is
	 *   D15..D9 WM8772 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	wm8772_write_reg_cache (codec, reg, value);
	if (codec->mach_write(codec->control_data, (long)data, 2) == 2)
		return 0;
	else
		return -EIO;
}

#define wm8772_reset(c)	wm8772_write(c, WM8772_RESET, 0)

/*
 * WM8772 Controls
 */
static const char *wm8772_zero_flag[] = {"All Ch", "Ch 1", "Ch 2", "Ch3"};

static const struct soc_enum wm8772_enum[] = {
SOC_ENUM_SINGLE(WM8772_DACCTRL, 0, 4, wm8772_zero_flag),
};

static const struct snd_kcontrol_new wm8772_snd_controls[] = {

SOC_SINGLE("Left1 Playback Volume", WM8772_LDAC1VOL, 0, 255, 0),
SOC_SINGLE("Left2 Playback Volume", WM8772_LDAC2VOL, 0, 255, 0),
SOC_SINGLE("Left3 Playback Volume", WM8772_LDAC3VOL, 0, 255, 0),
SOC_SINGLE("Right1 Playback Volume", WM8772_RDAC1VOL, 0, 255, 0),
SOC_SINGLE("Right1 Playback Volume", WM8772_RDAC2VOL, 0, 255, 0),
SOC_SINGLE("Right1 Playback Volume", WM8772_RDAC3VOL, 0, 255, 0),
SOC_SINGLE("Master Playback Volume", WM8772_MDACVOL, 0, 255, 0),

SOC_SINGLE("Playback Switch", WM8772_DACCH, 0, 1, 0),
SOC_SINGLE("Capture Switch", WM8772_ADCCTRL, 2, 1, 0),

SOC_SINGLE("Demp1 Playback Switch", WM8772_DACCTRL, 6, 1, 0),
SOC_SINGLE("Demp2 Playback Switch", WM8772_DACCTRL, 7, 1, 0),
SOC_SINGLE("Demp3 Playback Switch", WM8772_DACCTRL, 8, 1, 0),

SOC_SINGLE("Phase Invert 1 Switch", WM8772_IFACE, 6, 1, 0),
SOC_SINGLE("Phase Invert 2 Switch", WM8772_IFACE, 7, 1, 0),
SOC_SINGLE("Phase Invert 3 Switch", WM8772_IFACE, 8, 1, 0),

SOC_SINGLE("Playback ZC Switch", WM8772_DACCTRL, 0, 1, 0),

SOC_SINGLE("Capture High Pass Switch", WM8772_ADCCTRL, 3, 1, 0),
};

/* add non dapm controls */
static int wm8772_add_controls(struct snd_soc_codec *codec, 
	struct snd_card *card)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8772_snd_controls); i++) {
		err = snd_ctl_add(card,
				snd_soc_cnew(&wm8772_snd_controls[i],
					codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

static int wm8772_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct wm8772_priv *wm8772 = codec_dai->private_data;

	switch (freq) {
	case 4096000:
	case 5644800:
	case 6144000:
	case 8192000:
	case 8467000:
	case 9216000:
	case 11289600:
	case 12000000:
	case 12288000:
	case 16934400:
	case 18432000:
	case 22579200:
	case 24576000:
	case 33868800:
	case 36864000:
		if (clk_id == WM8772_DACCLK) {
			wm8772->dacclk = freq;
			return 0;
		} else if (clk_id == WM8772_ADCCLK) {
			wm8772->adcclk = freq;
			return 0;
		}
	}
	return -EINVAL;
}

static int wm8772_set_dac_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 diface = wm8772_read_reg_cache(codec, WM8772_IFACE) & 0x1f0;
	u16 diface_ctrl = wm8772_read_reg_cache(codec, WM8772_DACRATE) & 0x1ef;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		diface_ctrl |= 0x0010;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		diface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		diface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		diface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		diface |= 0x0007;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		diface |= 0x0008;
		break;
	default:
		return -EINVAL;
	}

	wm8772_write(codec, WM8772_DACRATE, diface_ctrl);
	wm8772_write(codec, WM8772_IFACE, diface);
	return 0;
}

static int wm8772_set_adc_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 aiface = 0;
	u16 aiface_ctrl = wm8772_read_reg_cache(codec, WM8772_ADCCTRL) & 0x1cf;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		aiface |= 0x0010;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		aiface |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		aiface |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		aiface |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		aiface |= 0x0003;
		aiface_ctrl |= 0x0010;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_NF:
		aiface_ctrl |= 0x0020;
		break;
	default:
		return -EINVAL;
	}

	wm8772_write(codec, WM8772_ADCCTRL, aiface_ctrl);
	wm8772_write(codec, WM8772_ADCRATE, aiface);
	return 0;
}

static int wm8772_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	int ret;
	
	ret = wm8772_set_dac_dai_fmt(codec_dai, fmt);
	if (ret < 0)
		return ret;
	return wm8772_set_adc_dai_fmt(codec_dai, fmt);	
}

static int wm8772_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;
	struct wm8772_priv *wm8772 = pcm_link->codec_dai->private_data;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {

		u16 diface = wm8772_read_reg_cache(codec, WM8772_IFACE) & 0x1cf;
		u16 diface_ctrl = wm8772_read_reg_cache(codec, WM8772_DACRATE) & 0x3f;

		/* bit size */
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			break;
		case SNDRV_PCM_FORMAT_S20_3LE:
			diface |= 0x0010;
			break;
		case SNDRV_PCM_FORMAT_S24_3LE:
			diface |= 0x0020;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			diface |= 0x0030;
			break;
		}

		/* set rate */
		switch (wm8772->dacclk / params_rate(params)) {
		case 768:
			diface_ctrl |= (0x5 << 6);
			break;
		case 512:
			diface_ctrl |= (0x4 << 6);
			break;
		case 384:
			diface_ctrl |= (0x3 << 6);
			break;
		case 256:
			diface_ctrl |= (0x2 << 6);
			break;
		case 192:
			diface_ctrl |= (0x1 << 6);
			break;
		}

		wm8772_write(codec, WM8772_DACRATE, diface_ctrl);
		wm8772_write(codec, WM8772_IFACE, diface);

	} else {

		u16 aiface = wm8772_read_reg_cache(codec, WM8772_ADCRATE) & 0x113;

		/* bit size */
		switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			break;
		case SNDRV_PCM_FORMAT_S20_3LE:
			aiface |= 0x0004;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			aiface |= 0x0008;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			aiface |= 0x000c;
			break;
		}

		/* set rate */
		switch (wm8772->adcclk / params_rate(params)) {
		case 768:
			aiface |= (0x5 << 5);
			break;
		case 512:
			aiface |= (0x4 << 5);
			break;
		case 384:
			aiface |= (0x3 << 5);
			break;
		case 256:
			aiface |= (0x2 << 5);
			break;
		}

		wm8772_write(codec, WM8772_ADCRATE, aiface);
	}

	return 0;
}

static int wm8772_dapm_event(struct snd_soc_codec *codec, int event)
{
	u16 master = wm8772_read_reg_cache(codec, WM8772_DACRATE) & 0xffe0;

	switch (event) {
		case SNDRV_CTL_POWER_D0: /* full On */
			/* vref/mid, clk and osc on, dac unmute, active */
			wm8772_write(codec, WM8772_DACRATE, master);
			break;
		case SNDRV_CTL_POWER_D1: /* partial On */
		case SNDRV_CTL_POWER_D2: /* partial On */
			break;
		case SNDRV_CTL_POWER_D3hot: /* Off, with power */
			/* everything off except vref/vmid, dac mute, inactive */
			wm8772_write(codec, WM8772_DACRATE, master | 0x0f);
			break;
		case SNDRV_CTL_POWER_D3cold: /* Off, without power */
			/* everything off, dac mute, inactive */
			wm8772_write(codec, WM8772_DACRATE, master | 0x1f);
			break;
	}
	codec->dapm_state = event;
	return 0;
}

static int wm8772_suspend(struct device *dev, pm_message_t state)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	wm8772_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	return 0;
}

static int wm8772_resume(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8772_reg); i++) {
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->mach_write(codec->control_data, (long)data, 2);
	}
	wm8772_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	wm8772_dapm_event(codec, codec->suspend_dapm_state);
	return 0;
}

/*
 * initialise the WM8772 codec
 */
static int wm8772_codec_io_probe(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	int reg;

	wm8772_reset(codec);

	/* power on device */
	wm8772_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	/* set the update bits */
	reg = wm8772_read_reg_cache(codec, WM8772_MDACVOL);
	wm8772_write(codec, WM8772_MDACVOL, reg | 0x0100);
	reg = wm8772_read_reg_cache(codec, WM8772_LDAC1VOL);
	wm8772_write(codec, WM8772_LDAC1VOL, reg | 0x0100);
	reg = wm8772_read_reg_cache(codec, WM8772_LDAC2VOL);
	wm8772_write(codec, WM8772_LDAC2VOL, reg | 0x0100);
	reg = wm8772_read_reg_cache(codec, WM8772_LDAC3VOL);
	wm8772_write(codec, WM8772_LDAC3VOL, reg | 0x0100);
	reg = wm8772_read_reg_cache(codec, WM8772_RDAC1VOL);
	wm8772_write(codec, WM8772_RDAC1VOL, reg | 0x0100);
	reg = wm8772_read_reg_cache(codec, WM8772_RDAC2VOL);
	wm8772_write(codec, WM8772_RDAC2VOL, reg | 0x0100);
	reg = wm8772_read_reg_cache(codec, WM8772_RDAC3VOL);
	wm8772_write(codec, WM8772_RDAC3VOL, reg | 0x0100);
	
	wm8772_add_controls(codec, machine->card);
	return 0;
}

static struct snd_soc_codec_ops wm8772_codec_ops = {
	.dapm_event	= wm8772_dapm_event,
	.read		= wm8772_read_reg_cache,
	.write		= wm8772_write,
	.io_probe	= wm8772_codec_io_probe,
};

static int wm8772_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	printk(KERN_INFO "WM8772 Audio Codec %s", WM8772_VERSION);

	codec->reg_cache = kmemdup(wm8772_reg, sizeof(wm8772_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	codec->reg_cache_size = sizeof(wm8772_reg);
	
	codec->owner = THIS_MODULE;
	codec->ops = &wm8772_codec_ops;
	return 0;
}

static int wm8772_codec_remove(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	
	if (codec->control_data)
		wm8772_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	kfree(codec->reg_cache);
	return 0;
}

#define WM8772_RATES (SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |\
		SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_88200 |\
		SNDRV_PCM_RATE_96000)

#define WM8772_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)

static const struct snd_soc_pcm_stream wm8772_dai_playback = {
	.stream_name	= "Playback",
	.channels_min	= 2,
	.channels_max	= 6,
	.rates		= WM8772_RATES | SNDRV_PCM_RATE_192000,
	.formats	= WM8772_FORMATS,
};

static const struct snd_soc_pcm_stream wm8772_dai_capture = {
	.stream_name	= "Capture",
	.channels_min	= 2,
	.channels_max	= 2,
	.rates		= WM8772_RATES,
	.formats	= WM8772_FORMATS,
};

/* dai ops, called by machine drivers */
static const struct snd_soc_dai_ops wm8772_dai_ops = {
	.set_sysclk	= wm8772_set_dai_sysclk,
	.set_fmt	= wm8772_set_dai_fmt,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops wm8772_dai_audio_ops = {
	.hw_params	= wm8772_hw_params,
};

static int wm8772_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);
	struct wm8772_priv *wm8772;
	
	wm8772 = kzalloc(sizeof(struct wm8772_priv), GFP_KERNEL);
	if (wm8772 == NULL)
		return -ENOMEM;
	
	dai->private_data = wm8772;
	dai->ops = &wm8772_dai_ops;
	dai->audio_ops = &wm8772_dai_audio_ops;
	dai->capture = &wm8772_dai_capture;
	dai->playback = &wm8772_dai_playback;
	return 0;
}

static int wm8772_dai_remove(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);
	kfree(dai->private_data);
	return 0;
}

const char wm8772_codec[SND_SOC_CODEC_NAME_SIZE] = "wm8772-codec";
EXPORT_SYMBOL_GPL(wm8772_codec);

static struct snd_soc_device_driver wm8772_codec_driver = {
	.type	= SND_SOC_BUS_TYPE_CODEC,
	.driver	= {
		.name 		= wm8772_codec,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8772_codec_probe,
		.remove		= __devexit_p(wm8772_codec_remove),
		.suspend	= wm8772_suspend,
		.resume		= wm8772_resume,
	},
};

const char wm8772_hifi_dai[SND_SOC_CODEC_NAME_SIZE] = "wm8772-hifi-dai";
EXPORT_SYMBOL_GPL(wm8772_hifi_dai);

static struct snd_soc_device_driver wm8772_hifi_dai_driver = {
	.type	= SND_SOC_BUS_TYPE_DAI,
	.driver	= {
		.name 		= wm8772_hifi_dai,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8772_dai_probe,
		.remove		= __devexit_p(wm8772_dai_remove),
	},
};

static __init int wm8772_init(void)
{
	int ret = 0;
	
	ret = driver_register(&wm8772_codec_driver.driver);
	if (ret < 0)
		return ret;
	ret = driver_register(&wm8772_hifi_dai_driver.driver);
	if (ret < 0) {
		driver_unregister(&wm8772_codec_driver.driver);
		return ret;
	}
	return ret;
}

static __exit void wm8772_exit(void)
{
	driver_unregister(&wm8772_hifi_dai_driver.driver);
	driver_unregister(&wm8772_codec_driver.driver);
}

module_init(wm8772_init);
module_exit(wm8772_exit);


MODULE_DESCRIPTION("ASoC WM8772 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
