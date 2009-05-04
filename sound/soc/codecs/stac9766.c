/*
 * stac9766.c  --  ALSA SoC STAC9766 codec support
 *
 * Copyright 2009 Jon Smirl, Digispeaker
 * Author: Jon Smirl <jonsmirl@gmail.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Features:-
 *
 *   o Support for AC97 Codec, S/PDIF
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-of-simple.h>

#include "stac9766.h"

#define STAC9766_VERSION "0.10"

/*
 * STAC9766 register cache
 */
static const u16 stac9766_reg[] = {
	0x6A90, 0x8000, 0x8000, 0x8000, /* 6 */
	0x0000, 0x0000, 0x8008, 0x8008, /* e */
	0x8808, 0x8808, 0x8808, 0x8808, /* 16 */
	0x8808, 0x0000, 0x8000, 0x0000, /* 1e */
	0x0000, 0x0000, 0x0000, 0x000f, /* 26 */
	0x0a05, 0x0400, 0xbb80, 0x0000, /* 2e */
	0x0000, 0xbb80, 0x0000, 0x0000, /* 36 */
	0x0000, 0x2000, 0x0000, 0x0100, /* 3e */
	0x0000, 0x0000, 0x0080, 0x0000, /* 46 */
	0x0000, 0x0000, 0x0003, 0xffff, /* 4e */
	0x0000, 0x0000, 0x0000, 0x0000, /* 56 */
	0x4000, 0x0000, 0x0000, 0x0000, /* 5e */
	0x1201, 0xFFFF, 0xFFFF, 0x0000, /* 66 */
	0x0000, 0x0000, 0x0000, 0x0000, /* 6e */
	0x0000, 0x0000, 0x0000, 0x0006, /* 76 */
	0x0000, 0x0000, 0x0000, 0x0000, /* 7e */
};

static const char *stac9766_record_mux[] = {"Mic", "CD", "Video", "AUX", "Line", "Stereo Mix", "Mono Mix", "Phone"};
static const char *stac9766_mono_mux[] = {"Mix", "Mic"};
static const char *stac9766_mic_mux[] = {"Mic1", "Mic2"};
static const char *stac9766_SPDIF_mux[] = {"PCM", "ADC Record"};
static const char *stac9766_popbypass_mux[] = {"Normal", "Bypass Mixer"};
static const char *stac9766_record_all_mux[] = {"All analog", "Analog plus DAC"};
static const char *stac9766_boost1[] = {"0dB", "10dB"};
static const char *stac9766_boost2[] = {"0dB", "20dB"};
static const char *stac9766_stereo_mic[] = {"Off", "On"};

static const struct soc_enum stac9766_record_enum =
	SOC_ENUM_DOUBLE(AC97_REC_SEL, 8, 0, 8, stac9766_record_mux); /* Record Mux 0 */
static const struct soc_enum stac9766_mono_enum =
	SOC_ENUM_SINGLE(AC97_GENERAL_PURPOSE, 9, 2, stac9766_mono_mux); /* Mono Mux 1 */
static const struct soc_enum stac9766_mic_enum =
	SOC_ENUM_SINGLE(AC97_GENERAL_PURPOSE, 8, 2, stac9766_mic_mux); /* Mic1/2 Mux 2 */
static const struct soc_enum stac9766_SPDIF_enum =
	SOC_ENUM_SINGLE(AC97_SENSE_INFO, 1, 2, stac9766_SPDIF_mux); /* SPDIF Mux 3 */
static const struct soc_enum stac9766_popbypass_enum =
	SOC_ENUM_SINGLE(AC97_GENERAL_PURPOSE, 15, 2, stac9766_popbypass_mux); /* Pop Bypass Mux 4 */
static const struct soc_enum stac9766_record_all_enum =
	SOC_ENUM_SINGLE(AC97_STAC_ANALOG_SPECIAL, 12, 2, stac9766_record_all_mux); /* Record All Mux 5 */
static const struct soc_enum stac9766_boost1_enum =
	SOC_ENUM_SINGLE(AC97_MIC, 6, 2, stac9766_boost1); /* 0/10dB */
static const struct soc_enum stac9766_boost2_enum =
	SOC_ENUM_SINGLE(AC97_STAC_ANALOG_SPECIAL, 2, 2, stac9766_boost2); /* 0/20dB */
static const struct soc_enum stac9766_stereo_mic_enum =
	SOC_ENUM_SINGLE(AC97_STAC_STEREO_MIC, 2, 1, stac9766_stereo_mic);

static const struct snd_kcontrol_new stac9766_snd_ac97_controls[] = {
	SOC_DOUBLE("Speaker Volume", AC97_MASTER, 8, 0, 31, 1),
	SOC_SINGLE("Speaker Switch", AC97_MASTER, 15, 1, 1),
	SOC_DOUBLE("Headphone Volume", AC97_HEADPHONE, 8, 0, 31, 1),
	SOC_SINGLE("Headphone Switch", AC97_HEADPHONE, 15, 1, 1),
	SOC_SINGLE("Mono Out Volume", AC97_MASTER_MONO, 0, 31, 1),
	SOC_SINGLE("Mono Out Switch", AC97_MASTER_MONO, 15, 1, 1),

	SOC_DOUBLE("Record Volume", AC97_REC_GAIN, 8, 0, 31, 1),
	SOC_SINGLE("Record Switch", AC97_REC_GAIN, 15, 1, 1),


	SOC_SINGLE("Beep Volume", AC97_PC_BEEP, 1, 15, 1),
	SOC_SINGLE("Beep Switch", AC97_PC_BEEP, 15, 1, 1),
	SOC_SINGLE("Beep Frequency", AC97_PC_BEEP, 5, 127, 1),
	SOC_SINGLE("Phone Volume", AC97_PHONE, 0, 31, 1),
	SOC_SINGLE("Phone Switch", AC97_PHONE, 15, 1, 1),

	SOC_ENUM("Mic Boost1", stac9766_boost1_enum),
	SOC_ENUM("Mic Boost2", stac9766_boost2_enum),
	SOC_SINGLE("Mic Volume", AC97_MIC, 0, 31, 1),
	SOC_SINGLE("Mic Switch", AC97_MIC, 15, 1, 1),
	SOC_ENUM("Stereo Mic", stac9766_stereo_mic_enum),

	SOC_DOUBLE("Line Volume", AC97_LINE, 8, 0, 31, 1),
	SOC_SINGLE("Line Switch", AC97_LINE, 15, 1, 1),
	SOC_DOUBLE("CD Volume", AC97_CD, 8, 0, 31, 1),
	SOC_SINGLE("CD Switch", AC97_CD, 15, 1, 1),
	SOC_DOUBLE("AUX Volume", AC97_AUX, 8, 0, 31, 1),
	SOC_SINGLE("AUX Switch", AC97_AUX, 15, 1, 1),
	SOC_DOUBLE("Video Volume", AC97_VIDEO, 8, 0, 31, 1),
	SOC_SINGLE("Video Switch", AC97_VIDEO, 15, 1, 1),

	SOC_DOUBLE("DAC Volume", AC97_PCM, 8, 0, 31, 1),
	SOC_SINGLE("DAC Switch", AC97_PCM, 15, 1, 1),
	SOC_SINGLE("Loopback Test Switch", AC97_GENERAL_PURPOSE, 7, 1, 0),
	SOC_SINGLE("3D Volume", AC97_3D_CONTROL, 3, 2, 1),
	SOC_SINGLE("3D Switch", AC97_GENERAL_PURPOSE, 13, 1, 0),

	SOC_ENUM("SPDIF Mux", stac9766_SPDIF_enum),
	SOC_ENUM("Mic1/2 Mux", stac9766_mic_enum),
	SOC_ENUM("Record All Mux", stac9766_record_all_enum),
	SOC_ENUM("Record Mux", stac9766_record_enum),
	SOC_ENUM("Mono Mux", stac9766_mono_enum),
	SOC_ENUM("Pop Bypass Mux", stac9766_popbypass_enum),
};


unsigned int stac9766_ac97_read(struct snd_soc_codec *codec, unsigned int reg)
{
	u16 val = 0, *cache = codec->reg_cache;

	if (reg / 2 > ARRAY_SIZE(stac9766_reg))
		return -EIO;

	if (reg == AC97_RESET || reg == AC97_GPIO_STATUS || AC97_INT_PAGING || reg
	                                == AC97_VENDOR_ID1 || reg
	                                == AC97_VENDOR_ID2) {

		val = soc_ac97_ops.read(codec->ac97, reg);
		return val;
	}
	return cache[reg / 2];
}

int stac9766_ac97_write(struct snd_soc_codec *codec, unsigned int reg,
                                unsigned int val)
{
	u16 *cache = codec->reg_cache;

	if (reg / 2 > ARRAY_SIZE(stac9766_reg))
		return -EIO;

	soc_ac97_ops.write(codec->ac97, reg, val);
	cache[reg / 2] = val;
	return 0;
}

static int ac97_analog_prepare(struct snd_pcm_substream *substream,
                                struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned short reg, vra;

	vra = stac9766_ac97_read(codec, AC97_EXTENDED_STATUS);

	//vra |= 0x4;

	stac9766_ac97_write(codec, AC97_EXTENDED_STATUS, vra | 0x1);

	printk("AC97_EXTENDED_STATUS %x\n", vra);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg = AC97_PCM_FRONT_DAC_RATE;
	else
		reg = AC97_PCM_LR_ADC_RATE;

	return stac9766_ac97_write(codec, reg, runtime->rate);
}

static int ac97_digital_prepare(struct snd_pcm_substream *substream,
                                struct snd_soc_dai *dai)
{
	printk("stac9766: ac97_digital_prepare\n");

	return 0;
}

static int stac9766_set_bias_level(struct snd_soc_codec *codec,
                                enum snd_soc_bias_level level)
{
	switch (level) {
	case SND_SOC_BIAS_ON: /* full On */
		stac9766_ac97_write(codec, AC97_POWERDOWN, 0x0000);
		break;
	case SND_SOC_BIAS_PREPARE: /* partial On */
		stac9766_ac97_write(codec, AC97_POWERDOWN, 0x0000);
		break;
	case SND_SOC_BIAS_STANDBY: /* Off, with power */
		stac9766_ac97_write(codec, AC97_POWERDOWN, 0x0000);
		break;
	case SND_SOC_BIAS_OFF: /* Off, without power */
		/* disable everything including AC link */
		stac9766_ac97_write(codec, AC97_POWERDOWN, 0xffff);
		break;
	}
	codec->bias_level = level;
	return 0;
}

static int stac9766_codec_suspend(struct platform_device *pdev,
                                pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	stac9766_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int stac9766_codec_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	u16 id;

	/* give the codec an AC97 warm reset to start the link */
	codec->ac97->bus->ops->warm_reset(codec->ac97);
	id = soc_ac97_ops.read(codec->ac97, AC97_VENDOR_ID2);
	if (id != 0x4c13) {
		printk(KERN_ERR "stac9766 failed to resume");
		return -EIO;
	}
	stac9766_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	if (codec->suspend_bias_level == SND_SOC_BIAS_ON)
		stac9766_set_bias_level(codec, SND_SOC_BIAS_ON);

	return 0;
}

static struct snd_soc_dai_ops stac9766_dai_ops_analog =
{
	.prepare = ac97_analog_prepare,
};

static struct snd_soc_dai_ops stac9766_dai_ops_digital =
{
	.prepare = ac97_digital_prepare,
};

struct snd_soc_dai stac9766_dai[] = {
{
	.name = "stac9766 analog",
	.id = 0,
	.ac97_control = 1,

	/* stream cababilities */
	.playback = {
		.stream_name = "stac9766 analog",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_BE |
				SNDRV_PCM_FMTBIT_S24_BE | SNDRV_PCM_FMTBIT_S32_BE,
	},
	.capture = {
		.stream_name = "stac9766 analog",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000_48000,
		.formats = SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_BE |
				SNDRV_PCM_FMTBIT_S24_BE | SNDRV_PCM_FMTBIT_S32_BE,
	},
	/* alsa ops */
	.ops = &stac9766_dai_ops_analog,
},
{
	.name = "stac9766 digital",
	.id = 1,
	.ac97_control = 1,

	/* stream cababilities */
	.playback = {
		.stream_name = "stac9766 digital",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_32000 |
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_IEC958_SUBFRAME,
	},
	/* alsa ops */
	.ops = &stac9766_dai_ops_digital,
}};
EXPORT_SYMBOL_GPL(stac9766_dai);

int stac9766_reset(struct snd_soc_codec *codec, int try_warm)
{
	if (try_warm && soc_ac97_ops.warm_reset) {
		soc_ac97_ops.warm_reset(codec->ac97);
		if (stac9766_ac97_read(codec, 0) == stac9766_reg[0])
			return 1;
	}

	soc_ac97_ops.reset(codec->ac97);
	if (soc_ac97_ops.warm_reset)
		soc_ac97_ops.warm_reset(codec->ac97);
	if (stac9766_ac97_read(codec, 0) != stac9766_reg[0])
		return -EIO;
	return 0;
}

static int stac9766_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;

	printk(KERN_INFO "STAC9766 SoC Audio Codec %s\n", STAC9766_VERSION);

	socdev->card->codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
	if (socdev->card->codec == NULL)
		return -ENOMEM;
	codec = socdev->card->codec;
	mutex_init(&codec->mutex);

	codec->reg_cache = kmemdup(stac9766_reg, sizeof(stac9766_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL) {
		ret = -ENOMEM;
		goto cache_err;
	}
	codec->reg_cache_size = sizeof(stac9766_reg);
	codec->reg_cache_step = 2;

	codec->name = "STAC9766";
	codec->owner = THIS_MODULE;
	codec->dai = stac9766_dai;
	codec->num_dai = ARRAY_SIZE(stac9766_dai);
	codec->write = stac9766_ac97_write;
	codec->read = stac9766_ac97_read;
	codec->set_bias_level = stac9766_set_bias_level;
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);

	ret = snd_soc_new_ac97_codec(codec, &soc_ac97_ops, 0);
	if (ret < 0)
		goto codec_err;

	/* register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0)
		goto pcm_err;

	/* do a cold reset for the controller and then try
	 * a warm reset followed by an optional cold reset for codec */
	stac9766_reset(codec, 0);
	ret = stac9766_reset(codec, 1);
	if (ret < 0) {
		printk(KERN_ERR "Failed to reset STAC9766: AC97 link error\n");
		goto reset_err;
	}

	stac9766_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	snd_soc_add_controls(codec, stac9766_snd_ac97_controls, ARRAY_SIZE(
	                                stac9766_snd_ac97_controls));

	ret = snd_soc_init_card(socdev);
	if (ret < 0)
		goto reset_err;
	return 0;

reset_err:
	snd_soc_free_pcms(socdev);
pcm_err:
	snd_soc_free_ac97_codec(codec);
codec_err:
	kfree(codec->private_data);
cache_err:
	kfree(socdev->card->codec);
	socdev->card->codec = NULL;
	return ret;
}

static int stac9766_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	if (codec == NULL)
		return 0;

	snd_soc_free_pcms(socdev);
	snd_soc_free_ac97_codec(codec);
	kfree(codec->reg_cache);
	kfree(codec);
	return 0;
}

struct snd_soc_codec_device soc_codec_dev_stac9766 =
{
	.probe = stac9766_codec_probe,
	.remove = stac9766_codec_remove,
	.suspend = stac9766_codec_suspend,
	.resume = stac9766_codec_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_dev_stac9766);

static int __init stac9766_probe(struct platform_device *pdev)
{
	snd_soc_register_dais(stac9766_dai, ARRAY_SIZE(stac9766_dai));
#if defined(CONFIG_SND_SOC_OF_SIMPLE)
	/* Tell the of_soc helper about this codec */
	of_snd_soc_register_codec(&soc_codec_dev_stac9766, pdev->dev.archdata.of_node,
									stac9766_dai, ARRAY_SIZE(stac9766_dai),
									pdev->dev.archdata.of_node);
#endif
	return 0;
}

static struct platform_driver stac9766_driver =
{
	.probe = stac9766_probe,
	.driver = {
			.name = "stac9766",
	},
};

static __init int stac9766_driver_init(void)
{
	return platform_driver_register(&stac9766_driver);
}

static __exit void stac9766_driver_exit(void)
{
}

module_init(stac9766_driver_init);
module_exit(stac9766_driver_exit);

MODULE_DESCRIPTION("ASoC stac9766 driver");
MODULE_AUTHOR("Jon Smirl <jonsmirl@gmail.com>");
MODULE_LICENSE("GPL");
