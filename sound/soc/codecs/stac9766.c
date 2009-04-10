/*
 * stac9766.c  --  ALSA Soc STAC9766 codec support
 *
 * Copyright 2008 Jon Smirl, Digispeaker
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
 *   o Support for DAPM
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "stac9766.h"

#define STAC9766_VERSION "0.10"

/*
 * STAC9766 register cache
 */
static const u16 stac9766_reg[] = {
	0x6A90, 0x8000, 0x8000, 0x8000, // 6
	0x0000, 0x0000, 0x8008, 0x8008, // e
	0x8808, 0x8808, 0x8808, 0x8808, // 16
	0x8808, 0x0000, 0x8000, 0x0000, // 1e
	0x0000, 0x0000, 0x0000, 0x000f, // 26
	0x0a05, 0x0400, 0xbb80, 0x0000, // 2e
	0x0000, 0xbb80, 0x0000, 0x0000, // 36
	0x0000, 0x2000, 0x0000, 0x0100, // 3e
	0x0000, 0x0000, 0x0080, 0x0000, // 46
	0x0000, 0x0000, 0x0003, 0xffff, // 4e
	0x0000, 0x0000, 0x0000, 0x0000, // 56
	0x4000, 0x0000, 0x0000, 0x0000, // 5e
	0x1201, 0xFFFF, 0xFFFF, 0x0000, // 66
	0x0000, 0x0000, 0x0000, 0x0000, // 6e
	0x0000, 0x0000, 0x0000, 0x0006, // 76
	0x0000, 0x0000, 0x0000, 0x0000, // 7e
};

static const char *stac9766_record_mux[] = {"Mic", "CD", "Video", "AUX", "Line", "Stereo Mix", "Mono Mix", "Phone"};
static const char *stac9766_mono_mux[] = {"Mix", "Mic"};
static const char *stac9766_mic_mux[] = {"Mic1", "Mic2"};
static const char *stac9766_SPDIF_mux[] = {"PCM", "ADC Record"};
static const char *stac9766_popbypass_mux[] = {"Normal", "Bypass Mixer"};
static const char *stac9766_record_all_mux[] = {"All analog", "Analog plus DAC"};
static const char *stac9766_3D_separation[] = {"Off", "Low", "Medium", "High"};

static const struct soc_enum stac9766_enum[] = {
	SOC_ENUM_DOUBLE(AC97_REC_SEL, 8, 0, 8, stac9766_record_mux), /* Record Mux 0 */
	SOC_ENUM_SINGLE(AC97_GENERAL_PURPOSE, 9, 2, stac9766_mono_mux), /* Mono Mux 1 */
	SOC_ENUM_SINGLE(AC97_GENERAL_PURPOSE, 8, 2, stac9766_mic_mux), /* Mic1/2 Mux 2 */
	SOC_ENUM_SINGLE(AC97_SENSE_INFO, 1, 2, stac9766_SPDIF_mux), /* SPDIF Mux 3 */
	SOC_ENUM_SINGLE(AC97_GENERAL_PURPOSE, 15, 2, stac9766_popbypass_mux), /* Pop Bypass Mux 4 */
	SOC_ENUM_SINGLE(AC97_STAC_ANALOG_SPECIAL, 12, 2, stac9766_record_all_mux), /* Record All Mux 5 */
	SOC_ENUM_SINGLE(AC97_3D_CONTROL, 2, 4, stac9766_3D_separation), /* 3D Separation 7 */
};

static const struct snd_kcontrol_new stac9766_snd_ac97_controls[] = {
	SOC_DOUBLE("Speaker Playback Volume", AC97_MASTER, 8, 0, 31, 1),
	SOC_SINGLE("Speaker Playback Switch", AC97_MASTER, 15, 1, 1),
	SOC_DOUBLE("Headphone Playback Volume", AC97_HEADPHONE, 8, 0, 31, 1),
	SOC_SINGLE("Headphone Playback Switch", AC97_HEADPHONE, 15, 1, 1),
	SOC_SINGLE("Mono Playback Volume", AC97_MASTER_MONO, 0, 31, 1),
	SOC_SINGLE("Mono Playback Switch", AC97_MASTER_MONO, 15, 1, 1),
	SOC_SINGLE("Mixer PC Beep Volume", AC97_PC_BEEP, 1, 15, 1),
	SOC_SINGLE("PC Beep Frequency", AC97_PC_BEEP, 5, 127, 1),
	SOC_SINGLE("Mixer Phone Volume", AC97_PHONE, 0, 31, 1),
	SOC_SINGLE("Mixer Mic Volume", AC97_MIC, 0, 31, 1),
	SOC_SINGLE("Mic Boost", AC97_MIC, 6, 1, 1),
	SOC_SINGLE("Mic Gain", AC97_STAC_ANALOG_SPECIAL, 2, 1, 1),
	SOC_SINGLE("Stereo Mic", AC97_STAC_STEREO_MIC, 0, 1, 1),
	SOC_DOUBLE("Mixer Line Volume", AC97_LINE, 8, 0, 31, 1),
	SOC_DOUBLE("Mixer CD Volume", AC97_CD, 8, 0, 31, 1),
	SOC_DOUBLE("Mixer Video Volume", AC97_VIDEO, 8, 0, 31, 1),
	SOC_DOUBLE("Mixer AUX Volume", AC97_AUX, 8, 0, 31, 1),
	SOC_DOUBLE("All Analog PCM Volume", AC97_PCM, 8, 0, 31, 1),
	SOC_DOUBLE("Record Gain", AC97_REC_GAIN, 8, 0, 31, 1),
	SOC_SINGLE("Record Gain Switch", AC97_REC_GAIN, 15, 1, 1),
	SOC_SINGLE("3D Effect Switch", AC97_GENERAL_PURPOSE, 13, 1, 0),
	SOC_SINGLE("Loopback Test Switch", AC97_GENERAL_PURPOSE, 7, 1, 0),
	SOC_ENUM("3D Separation", stac9766_enum[6]),
};

/* add non dapm controls */
static int stac9766_add_controls(struct snd_soc_codec *codec, 
	struct snd_card *card)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(stac9766_snd_ac97_controls); i++) {
		err = snd_ctl_add(card,
				snd_soc_cnew(&stac9766_snd_ac97_controls[i],
				codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}


/* Mixer */
static const struct snd_kcontrol_new stac9766_main_mixer_controls[] = {
	SOC_DAPM_SINGLE("PC Beep Switch", AC97_PC_BEEP, 15, 1, 1),
	SOC_DAPM_SINGLE("Phone Switch", AC97_PHONE, 15, 1, 1),
	SOC_DAPM_SINGLE("Mic Switch", AC97_MIC, 15, 1, 1),
	SOC_DAPM_SINGLE("Line Switch", AC97_LINE, 15, 1, 1),
	SOC_DAPM_SINGLE("CD Switch", AC97_CD, 15, 1, 1),
	SOC_DAPM_SINGLE("AUX Switch", AC97_AUX, 15, 1, 1),
	SOC_DAPM_SINGLE("Video Switch", AC97_VIDEO, 15, 1, 1),
};

/* Record All Mixer */
static const struct snd_kcontrol_new stac9766_record_all_mixer_controls[] = {
	SOC_DAPM_SINGLE("PCM Switch", AC97_PCM, 15, 1, 1),
	SOC_DAPM_SINGLE("Mixer", 0, 0, 0, 0),
};

/* Record Mux 0 */
static const struct snd_kcontrol_new stac9766_record_mux_controls =
	SOC_DAPM_ENUM("Route", stac9766_enum[0]);

/* Mono Mux 1 */
static const struct snd_kcontrol_new stac9766_mono_mux_controls =
	SOC_DAPM_ENUM("Route", stac9766_enum[1]);

/* Mic1/2 Mux 2 */
static const struct snd_kcontrol_new stac9766_mic_mux_controls =
	SOC_DAPM_ENUM("Route", stac9766_enum[2]);

/* SPDIF Mux 3 */
static const struct snd_kcontrol_new stac9766_spdif_mux_controls =
	SOC_DAPM_ENUM("Route", stac9766_enum[3]);

/* Pop Bypass 4 */
static const struct snd_kcontrol_new stac9766_popbypass_mux_controls =
	SOC_DAPM_ENUM("Route", stac9766_enum[4]);

/* Record All 5 */
static const struct snd_kcontrol_new stac9766_record_all_mux_controls =
	SOC_DAPM_ENUM("Route", stac9766_enum[5]);

static const struct snd_soc_dapm_widget stac9766_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("PCMOut"),
	SND_SOC_DAPM_INPUT("PCBEEP"),
	SND_SOC_DAPM_INPUT("Phone"),
	SND_SOC_DAPM_INPUT("MIC1"),
	SND_SOC_DAPM_INPUT("MIC2"),
	SND_SOC_DAPM_INPUT("Line"),
	SND_SOC_DAPM_INPUT("CD"),
	SND_SOC_DAPM_INPUT("AUX"),
	SND_SOC_DAPM_INPUT("Video"),
	SND_SOC_DAPM_DAC("DAC", "Analog Playback", AC97_POWERDOWN, 9, 1),
	SND_SOC_DAPM_MUX("SPDIF Mux", SND_SOC_NOPM, 0, 0,
		&stac9766_spdif_mux_controls),
	SND_SOC_DAPM_MUX("Mic1/2 Mux", SND_SOC_NOPM, 0, 0,
		&stac9766_mic_mux_controls),
	SND_SOC_DAPM_MIXER("Mixer", SND_SOC_NOPM, 0, 1,
		&stac9766_main_mixer_controls[0], ARRAY_SIZE(stac9766_main_mixer_controls)),
	SND_SOC_DAPM_MUX("Record All Mux", SND_SOC_NOPM, 0, 0,
		&stac9766_record_all_mux_controls),
	SND_SOC_DAPM_MUX("Record Mux", SND_SOC_NOPM, 0, 0,
		&stac9766_record_mux_controls),
	SND_SOC_DAPM_MUX("Mono Mux", SND_SOC_NOPM, 0, 0,
		&stac9766_mono_mux_controls),
	SND_SOC_DAPM_MUX("Pop Bypass Mux", SND_SOC_NOPM, 0, 0,
		&stac9766_popbypass_mux_controls),
	SND_SOC_DAPM_MIXER("All Analog", SND_SOC_NOPM, 0, 1,
		&stac9766_record_all_mixer_controls[0], ARRAY_SIZE(stac9766_record_all_mixer_controls)),
	SND_SOC_DAPM_ADC("ADC", "Analog Capture", AC97_POWERDOWN, 8, 1),
	SND_SOC_DAPM_OUTPUT("HP"),
	SND_SOC_DAPM_OUTPUT("LINE"),
	SND_SOC_DAPM_OUTPUT("MONO"),
	SND_SOC_DAPM_OUTPUT("PCMIn"),
	SND_SOC_DAPM_VMID("VMID"),
};

static const char *audio_map[][3] = {
	{"HP", NULL, "Pop Bypass Mux"},
	{"LINE", NULL, "Pop Bypass Mux"},
	
	/* Mono Mux */
	{"MONO", NULL, "Pop Bypass Mux"},
	{"MONO", NULL, "Mic1/2 Mux"},
	
	/* Pop Bypass Mux */
	{"Pop Bypass Mux", NULL, "DAC"},
	{"Pop Bypass Mux", NULL, "All Analog Mixer"},

	/* Record Mux */
	{"ADC", NULL, "Mic1/2 Mux"},
	{"ADC", NULL, "CD"},
	{"ADC", NULL, "Video"},
	{"ADC", NULL, "AUX"},
	{"ADC", NULL, "Line"},
	{"ADC", NULL, "Record All Mux"},
	{"ADC", NULL, "Phone"},

	{"PCMIn", NULL, "ADC"},
	
	/* Record All Mux */
	{"Record All Mux", NULL, "Mixer"},
	{"Record All Mux", NULL, "All Analog Mixer"},
	
	/* All Analog Mixer */
	{"All Analog", NULL, "Mixer"},
	{"All Analog", "PCM Switch", "DAC"},

	/* Mixer */
	{"Mixer", "PC Beep Switch", "PCBEEP"},
	{"Mixer", "Phone Switch", "Phone"},
	{"Mixer", "Mic Switch", "Mic1/2 Mux"},
	{"Mixer", "Line Switch", "Line"},
	{"Mixer", "CD Switch", "CD"},
	{"Mixer", "AUX Switch", "AUX"},
	{"Mixer", "Video Switch", "Video"},

	{"DAC", NULL, "PCMOut"},
	
	/* Mic1/2 Mux */
	{"Mic1/2 Mux", NULL, "MIC1"},
	{"Mic1/2 Mux", NULL, "MIC2"},
	
	/* SPDIF Mux */
	{"SPDIF Mux", NULL, "PCMOut"},
	{"SPDIF Mux", NULL, "ADC"},

	{NULL, NULL, NULL},
};

static int stac9766_add_widgets(struct snd_soc_codec *codec, 
	struct snd_soc_machine *machine)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(stac9766_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec, 
			&stac9766_dapm_widgets[i]);
	}

	/* set up audio path audio_map */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_add_route(machine, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_init(machine);
	return 0;
}

unsigned int stac9766_ac97_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 val = 0, *cache = codec->reg_cache;

	if (reg/2 > ARRAY_SIZE(stac9766_reg))
		return -EIO;
		
	if (reg == AC97_RESET || reg == AC97_GPIO_STATUS || AC97_INT_PAGING ||
		reg == AC97_VENDOR_ID1 || reg == AC97_VENDOR_ID2) {

		val = codec->machine_read(codec->ac97, reg);
		printk("stac9766_ac97_read actual reg %02x val %04x\n", reg, val);
		return val;
	}
	printk("stac9766_ac97_read cache reg %02x val %04x\n", reg, cache[reg/2]);
	return cache[reg/2];
}

int stac9766_ac97_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val)
{
	u16 *cache = codec->reg_cache;
	
	printk("stac9766_ac97_write reg %02x val %04x\n", reg, val);
	if (reg/2 > ARRAY_SIZE(stac9766_reg))
		return -EIO;
		
	codec->machine_write(codec->ac97, val, reg);
	cache[reg/2] = val;
	return 0;
}

static int ac97_analog_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai_runtime *dai)
{
	struct snd_soc_pcm_runtime *pcm_link = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_codec *codec = pcm_link->codec;
	u16 vra;
	
	printk("stac9766: ac97_analog_prepare\n");

	vra = stac9766_ac97_read(codec, AC97_EXTENDED_STATUS);
	stac9766_ac97_write(codec, AC97_EXTENDED_STATUS, vra | 0x1);

	return stac9766_ac97_write(codec, AC97_PCM_FRONT_DAC_RATE, runtime->rate);
}

static int ac97_digital_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai_runtime *dai)
{
	printk("stac9766: ac97_digital_prepare\n");

	return 0;
}

static int stac9766_set_bias_level(struct snd_soc_codec *codec, 
	enum snd_soc_dapm_bias_level level)
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

static int stac9766_codec_suspend(struct device *dev, pm_message_t state)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	stac9766_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int stac9766_codec_resume(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	u16 id;

	/* give the codec an AC97 warm reset to start the link */
	codec->ac97->bus->ops->warm_reset(codec->ac97);
	id = codec->machine_read(codec->ac97, AC97_VENDOR_ID2); 
	if (id != 0x4c13) {
		printk(KERN_ERR "stac9766 failed to resume");
		return -EIO;
	}
	stac9766_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	if (codec->suspend_bias_level == SND_SOC_BIAS_ON)
		stac9766_set_bias_level(codec, SND_SOC_BIAS_ON);

	return 0;
}

static int stac9766_codec_init(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	printk("stac9766: stac9766_codec_init\n");

	stac9766_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	
	stac9766_add_controls(codec, machine->card);
	stac9766_add_widgets(codec, machine);

	return 0;
}


static struct snd_soc_dai stac9766_dai[] = {
{
	.name	= "stac9766 analog",
	.id	= 0,
	
	/* stream cababilities */
	.playback = {
		.stream_name	= "stac9766 analog",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FORMAT_S32,
	},
	.capture = {
		.stream_name	= "stac9766 analog",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_8000_48000,
		.formats	= SNDRV_PCM_FMTBIT_S32,
	},
	/* alsa ops */
	.prepare = ac97_analog_prepare,
},
{
	.name	= "stac9766 digital",
	.id	= 1,
	
	/* stream cababilities */
	.playback = {
		.stream_name	= "stac9766 digital",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_32000 | \
			SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
		.formats	= SNDRV_PCM_FMTBIT_IEC958_SUBFRAME,
	},
	/* alsa ops */
	.prepare = ac97_digital_prepare,
}};

static int stac9766_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	int ret = 0;

	printk(KERN_INFO "STAC9766 SoC Audio Codec %s\n", STAC9766_VERSION);

	codec->reg_cache = kmemdup(stac9766_reg, sizeof(stac9766_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
		
	codec->reg_cache_size = sizeof(stac9766_reg);
	codec->reg_cache_step = 2;
	codec->set_bias_level = stac9766_set_bias_level;
	codec->codec_read = stac9766_ac97_read;
	codec->codec_write = stac9766_ac97_write;
	codec->init = stac9766_codec_init;

	ret = snd_soc_codec_add_dai(codec, stac9766_dai, ARRAY_SIZE(stac9766_dai));
	if (ret < 0)
		goto err;
		
 	ret = snd_soc_register_codec(codec);
 	if (ret < 0)
 		goto err;
 		
	return ret;
err:
	kfree(codec->reg_cache);
	return ret;
}

static int stac9766_codec_remove(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	snd_soc_unregister_codec(codec);
	kfree(codec->reg_cache);
	return 0;
}

const char stac9766_codec_id[] = "stac9766-codec";
EXPORT_SYMBOL_GPL(stac9766_codec_id);

static struct device_driver stac9766_codec_driver = {
	.name 		= stac9766_codec_id,
	.owner		= THIS_MODULE,
	.bus 		= &asoc_bus_type,
	.probe		= stac9766_codec_probe,
	.remove		= __devexit_p(stac9766_codec_remove),
	.suspend	= stac9766_codec_suspend,
	.resume		= stac9766_codec_resume,
};

static __init int stac9766_init(void)
{
	printk("stac9766_init\n");
	return driver_register(&stac9766_codec_driver);
}

static __exit void stac9766_exit(void)
{
	driver_unregister(&stac9766_codec_driver);
}

module_init(stac9766_init);
module_exit(stac9766_exit);

MODULE_DESCRIPTION("ASoC STAC9766 driver");
MODULE_AUTHOR("Jon Smirl");
MODULE_LICENSE("GPL");
