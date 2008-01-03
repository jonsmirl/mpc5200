/*
 * tlv320.c  --  TLV 320 ALSA Soc Audio driver
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Copyright 2006 Atlab srl.
 *
 * Authors: Liam Girdwood <liam.girdwood@wolfsonmicro.com>
 *          Nicola Perrino <nicola.perrino@atlab.it>
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
#include <linux/platform_device.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "tlv320.h"

#define AUDIO_NAME "tlv320"
#define TLV320_VERSION "0.1"

/*
 * Debug
 */

#define TLV320_DEBUG 0

#ifdef TLV320_DEBUG
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


static int setting = 1;
module_param(setting, int, 0);

#ifdef TLV320AIC24K
/* ADDR table */
static const unsigned char tlv320_reg_addr[] = {
	0x00,	/* CONTROL REG 0 No Operation */
	0x01,	/* CONTROL REG 1  */
        0x02,	/* CONTROL REG 2  */
        0x03,	/* CONTROL REG 3A */
        0x03,	/* CONTROL REG 3B */
        0x03, 	/* CONTROL REG 3C */
        0x03,	/* CONTROL REG 3D */
        0x04,	/* CONTROL REG 4  */
        0x04,	/* CONTROL REG 4 Bis */
        0x05,	/* CONTROL REG 5A */
        0x05,	/* CONTROL REG 5B */
        0x05,	/* CONTROL REG 5C */
        0x05,	/* CONTROL REG 5D */
        0x06,	/* CONTROL REG 6A */
        0x06,	/* CONTROL REG 6B */
};

/*
 *  DATA case digital SET1:
 *     SSP -> DAC -> OUT
 *     IN  -> ADC -> SSP
 *     IN = HNSI (MIC)
 *     OUT = HDSO (SPKG)
 *  Usage: playback, capture streams
 */
static const unsigned char tlv320_reg_data_init_set1[] = {
	0x00,	/* CONTROL REG 0 No Operation */
	0x49,	/* CONTROL REG 1  */
        0x20,	/* CONTROL REG 2  */
        0x01,	/* CONTROL REG 3A */
        0x40,	/* CONTROL REG 3B */
        0x81,	/* CONTROL REG 3C */
        0xc0,	/* CONTROL REG 3D */
        0x02,//0x42(16khz),//0x10,	/* CONTROL REG 4  */
        0x88,//0x90,	/* CONTROL REG 4 Bis */
        0x00,	/* CONTROL REG 5A */
        0x40,//(0dB)	/* CONTROL REG 5B */
        0xbf,	/* CONTROL REG 5C */
        0xc0,	/* CONTROL REG 5D */
        0x02,//(HNSI) 	/* CONTROL REG 6A */
        0x81 //(HDSO) 	/* CONTROL REG 6B */
};

/*
 *  DATA case digital SET2:
 *     SSP -> DAC -> OUT
 *     IN  -> ADC -> SSP
 *     IN = HDSI (PHONE IN)
 *     OUT = HNSO (PHONE OUT)
 *  Usage: playback, capture streams
 */
static const unsigned char tlv320_reg_data_init_set2[] = {
	0x00,	/* CONTROL REG 0 No Operation */
	0x49,	/* CONTROL REG 1  */
        0x20,	/* CONTROL REG 2  */
        0x01,	/* CONTROL REG 3A */
        0x40,	/* CONTROL REG 3B */
        0x81,	/* CONTROL REG 3C */
        0xc0,	/* CONTROL REG 3D */
        0x02,//0x42(16khz),//0x10,	/* CONTROL REG 4  */
        0x88,//0x90,	/* CONTROL REG 4 Bis */
        0x00,	/* CONTROL REG 5A */
        0x52,//(-27dB) 	/* CONTROL REG 5B */
        0xbf,	/* CONTROL REG 5C */
        0xc0,	/* CONTROL REG 5D */
        0x01,//(PHONE IN) 	/* CONTROL REG 6A */
        0x82 //(PHONE OUT) 	/* CONTROL REG 6B */
};

/*
 *  DATA case analog:
 *     ADC, DAC, SSP off
 *     Headset input to output (HDSI2O -> 1)
 *     Handset input to output (HNSI2O -> 1)
 *  Usage: room monitor
 */
static const unsigned char tlv320_reg_data_init_set3[] = {
	0x00,	/* CONTROL REG 0 No Operation */
	0x08,   /* CONTROL REG 1  */
        0x20,   /* CONTROL REG 2  */
        0x11,   /* CONTROL REG 3A */
        0x40,	/* CONTROL REG 3B */
        0x80,   /* CONTROL REG 3C */
        0xc0,   /* CONTROL REG 3D */
        0x00,   /* CONTROL REG 4  */
        0x00,   /* CONTROL REG 5A */
        0x40,   /* CONTROL REG 5B */
        0x80,   /* CONTROL REG 5C */
        0xc0,   /* CONTROL REG 5D */
        0x60,   /* CONTROL REG 6A */
        0x80    /* CONTROL REG 6B */
};

#else // TLV320AIC14k

/* ADDR table */
static const unsigned char tlv320_reg_addr[] = {
	0x00,	/* CONTROL REG 0 No Operation */
	0x01,	/* CONTROL REG 1  */
        0x02,	/* CONTROL REG 2  */
        0x03,	/* CONTROL REG 3 */
        0x04,	/* CONTROL REG 4  */
        0x04,	/* CONTROL REG 4 Bis */
        0x05,	/* CONTROL REG 5A */
        0x05,	/* CONTROL REG 5B */
        0x05,	/* CONTROL REG 5C */
        0x05,	/* CONTROL REG 5D */
        0x06	/* CONTROL REG 6 */
};

/*
 *  DATA case digital:
 *     SSP -> DAC -> OUT
 *     IN  -> ADC -> SSP
 *  Usage: playback, capture streams
 */
static const unsigned char tlv320_reg_data_init_set1[] = {
	0x00,	/* CONTROL REG 0 No Operation */
	0x41,	/* CONTROL REG 1  */
        0x20,	/* CONTROL REG 2  */
        0x09,	/* CONTROL REG 3 */
        0x02,//0x42(16khz),//0x10,	/* CONTROL REG 4  */
        0x88,//0x90,	/* CONTROL REG 4 Bis */
        0x2A,	/* CONTROL REG 5A */
        0x6A,	/* CONTROL REG 5B */
        0xbc,	/* CONTROL REG 5C */
        0xc0,	/* CONTROL REG 5D */
        0x00	/* CONTROL REG 6 */
};
#endif
/*
 * read tlv320 register cache
 */
static inline unsigned int tlv320_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 *cache = codec->reg_cache;
	if (reg > ARRAY_SIZE(tlv320_reg_addr))
		return -1;
	return cache[reg];
}

/*
 * write tlv320 register cache
 */
static inline void tlv320_write_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	u8 *cache = codec->reg_cache;
	if (reg > ARRAY_SIZE(tlv320_reg_addr))
		return;
	cache[reg] = value;
}

/*
 * read tlv320
 */
static int tlv320_read (struct snd_soc_codec *codec, u8 reg)
{
	return i2c_smbus_read_byte_data(codec->control_data, reg);
}

/*
 * write tlv320
 */
static int tlv320_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	if (tlv320_reg_addr[reg] > 0x06)
		return -1;

	tlv320_write_reg_cache (codec, reg, value);

	return i2c_smbus_write_byte_data(codec->control_data, tlv320_reg_addr[reg], value);
}


/*
 * write block tlv320
 */
static int tlv320_write_block (struct snd_soc_codec *codec,
	const u8 *data, unsigned int len)
{
	int ret = -1;
	int i;

	for (i=0; i<len; i++) {
		dbg("addr = 0x%02x, data = 0x%02x", tlv320_reg_addr[i], data[i]);
		if ((ret = tlv320_write(codec, i, data[i])) < 0)
			break;
	}

	return ret;
}

/*
 * initialise the WM8731 codec
 */
static int tlv320_codec_init(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	int ret, len;
	static const unsigned char *data;
	
	switch(setting) {
	case 1:
		data = tlv320_reg_data_init_set1;
		len = sizeof(tlv320_reg_data_init_set1);
		break;
	case 2:
		data = tlv320_reg_data_init_set2;
		len = sizeof(tlv320_reg_data_init_set2);
		break;
	case 3:
		data = tlv320_reg_data_init_set3;
		len = sizeof(tlv320_reg_data_init_set3);
		break;
	default:
		data = tlv320_reg_data_init_set1;
		len = sizeof(tlv320_reg_data_init_set1);
		break;
	}

	ret = tlv320_write_block(codec, data, len);

	if (ret < 0)
		err("attach error: init status %d\n", ret);
	else
		info("attach: chip tlv320 at address 0x%02x",
			tlv320_read(codec, 0x02) << 1);

        //tlv320_write(codec, CODEC_REG6B, 0x80);
#if 0
	int value;
	int i;

	for (i=0; i<len; i++) {
		value = tlv320_read(codec, tlv320_reg_addr[i]);
		dbg("read addr = 0x%02x, data = 0x%02x", tlv320_reg_addr[i], value);
		mdelay(10);
	}

#endif

	return 0;
}

#define TLV320_VOICE_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)


#define TLV320_VOICE_BITS \
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE | \
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE)


static int tlv320_dai_probe(struct snd_soc_dai_runtime *dai)
{
	struct snd_soc_dai_runtime *dai = to_snd_soc_dai_runtime(dev);
	struct tlv320_priv *tlv320;
	
	tlv320 = kzalloc(sizeof(struct tlv320_priv), GFP_KERNEL);
	if (tlv320 == NULL)
		return -ENOMEM;
	
	dai->private_data = tlv320;
	return 0;
}

static void wm8731_dai_free(struct snd_soc_dai_runtime *dai)
{
	kfree(dai->private_data);
}

static struct snd_soc_dai wm8731_dai = {
	.name	= "wm8731-HiFi",
	.id	= WM8731_DAI,
	.new	= wm8731_dai_new,
	.free	= wm8731_dai_free,
	
	/* stream cababilities */
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= WM8731_RATES,
		.formats	= WM8731_FORMATS,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 1,
		.channels_max	= 2,
		.rates		= WM8731_RATES,
		.formats	= WM8731_FORMATS,
	},
	/* alsa ops */
	.hw_params	= tlv320_hw_params,
	.prepare	= tlv320_prepare,
	.shutdown	= tlv320_shutdown,
	
	/* dai ops */
	.digital_mute	= tlv320_mute,
	.set_sysclk	= tlv320_set_dai_sysclk,
	.set_fmt	= tlv320_set_dai_fmt,
};

static int tlv320_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	int ret;

	info("WM8731 Audio Codec %s", WM8731_VERSION);

	codec->reg_cache = kmemdup(tlv320_reg, sizeof(tlv320_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	codec->reg_cache_size = sizeof(tlv320_reg);

	codec->set_bias_power = tlv320_set_bias_power;
	codec->codec_read = tlv320_read_reg_cache;
	codec->codec_write = tlv320_write;
	codec->init = tlv320_codec_init;
	
	ret = snd_soc_codec_add_dai(codec, &tlv320_dai, 1);
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

static int tlv320_codec_remove(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	snd_soc_unregister_codec(codec);
	kfree(codec->reg_cache);
	return 0;
}

const char tlv320_codec_id[] = "tlv320-codec";
EXPORT_SYMBOL_GPL(tlv320_codec_id);

static struct device_driver tlv320_codec_driver = {
	.name 		= tlv320_codec_id,
	.owner		= THIS_MODULE,
	.bus 		= &asoc_bus_type,
	.probe		= tlv320_codec_probe,
	.remove		= __devexit_p(tlv320_codec_remove),
	.suspend	= tlv320_suspend,
	.resume		= tlv320_resume,
};

static __init int tlv320_init(void)
{
	return  driver_register(&tlv320_codec_driver);
}

static __exit void tlv320_exit(void)
{
	driver_unregister(&tlv320_codec_driver);
}

module_init(tlv320_init);
module_exit(tlv320_exit);

MODULE_DESCRIPTION("ASoC TLV320 driver");
MODULE_AUTHOR("Nicola Perrino");
MODULE_LICENSE("GPL");
