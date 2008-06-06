/*
 * amesom_tlv320.c  --  SoC audio for Amesom
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
 *  Revision history
 *    5th Dec 2006   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>

#include <sound/initval.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/audio.h>

#include "../codecs/tlv320.h"
#include "pxa2xx-pcm.h"


/*
 * SSP2 GPIO's
 */

#define GPIO11_SSP2RX_MD	(11 | GPIO_ALT_FN_2_IN)
#define GPIO13_SSP2TX_MD	(13 | GPIO_ALT_FN_1_OUT)
#define GPIO50_SSP2CLKS_MD	(50 | GPIO_ALT_FN_3_IN)
#define GPIO14_SSP2FRMS_MD	(14 | GPIO_ALT_FN_2_IN)
#define GPIO50_SSP2CLKM_MD	(50 | GPIO_ALT_FN_3_OUT)
#define GPIO14_SSP2FRMM_MD	(14 | GPIO_ALT_FN_2_OUT)


/*
 * Tlv320 uses SSP port for playback.
 */
static int tlv320_voice_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	int ret = 0;

	//printk("tlv320_voice_hw_params enter\n");
	switch(params_rate(params)) {
	case 8000:
		//printk("tlv320_voice_hw_params 8000\n");
		break;
	case 16000:
		//printk("tlv320_voice_hw_params 16000\n");
		break;
	default:
		break;
	}

	// CODEC MASTER, SSP SLAVE

	/* set codec DAI configuration */
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_MSB |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_MSB |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set the SSP system clock as input (unused) */
	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA2XX_SSP_CLK_NET_PLL, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set SSP slots */
	//ret = cpu_dai->dai_ops.set_tdm_slot(cpu_dai, 0x1, slots);
	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0x3, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops tlv320_voice_ops = {
	.hw_params = tlv320_voice_hw_params,
};

static unsigned short normal_i2c[] = {
#ifdef TLV320AIC24K
	0x41,
#else
	0x40,
#endif
	 I2C_CLIENT_END,
};

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver tlv320_i2c_driver;
static struct i2c_client client_template;

static int amesom_tlv320_write(void *control_data, long data, int size)
{
	return i2c_master_send((struct i2c_client*)control_data,
		(char*) data, size);
}

/*
 * Logic for a tlv320 as connected on a Sharp SL-C7x0 Device
 */
static int amesom_tlv320_init(struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;

	codec = snd_soc_card_get_codec(soc_card, tlv320_codec_id, 0);
	if (codec == NULL)
		return -ENODEV;

	/* SSP port 2 slave */
	pxa_gpio_mode(GPIO11_SSP2RX_MD);
	pxa_gpio_mode(GPIO13_SSP2TX_MD);
	pxa_gpio_mode(GPIO50_SSP2CLKS_MD);
	pxa_gpio_mode(GPIO14_SSP2FRMS_MD);

	snd_soc_card_config_codec(codec, NULL, amesom_tlv320_write,
		soc_card->private_data);

	snd_soc_card_init_codec(codec, soc_card);

	return 0;
}

static struct snd_soc_pcm_config hifi_pcm_config = {
	.name		= "HiFi",
	.codec		= tlv320_codec_id,
//	.codec_dai	= tlv320_codec_dai_id,
	.platform	= pxa_platform_id,
//	.cpu_dai	= pxa2xx_ssp2_dai_id,
	.ops		= &tlv320_voice_ops,
	.playback	= 1,
	.capture	= 1,
};

static int tlv320_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_card *soc_card;
	struct i2c_client *i2c;
	int ret;

	if (addr != 0x40 || addr != 0x41)
		return -ENODEV;

	client_template.adapter = adap;
	client_template.addr = addr;

	i2c = kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL)
		return -ENOMEM;

	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		printk("failed to attach codec at addr %x\n", addr);
		goto attach_err;
	}

	soc_card = snd_soc_card_create("amesom", &i2c->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (soc_card == NULL)
		return -ENOMEM;

	soc_card->longname = "tlv320";
	soc_card->init = amesom_tlv320_init;
	soc_card->private_data = i2c;
	i2c_set_clientdata(i2c, soc_card);

	ret = snd_soc_card_create_pcms(soc_card, &hifi_pcm_config, 1);
	if (ret < 0)
		goto err;

	ret = snd_soc_card_register(soc_card);
	return ret;

err:
	snd_soc_card_free(soc_card);
attach_err:
	i2c_detach_client(i2c);
	kfree(i2c);
	return ret;
}

static int tlv320_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_card *soc_card = i2c_get_clientdata(client);

	snd_soc_card_free(soc_card);
	i2c_detach_client(client);
	kfree(client);
	return 0;
}

static int tlv320_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, tlv320_i2c_probe);
}

static struct i2c_driver tlv320_i2c_driver = {
	.driver = {
		.name = "tlv320 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_TLV320,
	.attach_adapter = tlv320_i2c_attach,
	.detach_client =  tlv320_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "tlv320",
	.driver = &tlv320_i2c_driver,
};

static int __init amesom_tlv320_probe(struct platform_device *pdev)
{
	int ret;

	/* register I2C driver for tlv320 codec control */
	ret = i2c_add_driver(&tlv320_i2c_driver);
	if (ret < 0)
		printk (KERN_ERR "%s: failed to add i2c driver\n",
			__FUNCTION__);
	return ret;
}

static int __exit amesom_tlv320_remove(struct platform_device *pdev)
{
	i2c_del_driver(&tlv320_i2c_driver);
	return 0;
}


static struct platform_driver amesom_tlv320_driver = {
	.probe		= amesom_tlv320_probe,
	.remove		= __devexit_p(amesom_tlv320_remove),
	.driver		= {
		.name		= "amesom-tlv320",
		.owner		= THIS_MODULE,
	},
};

static int __init amesom_init(void)
{
	return platform_driver_register(&amesom_tlv320_driver);
}

static void __exit amesom_exit(void)
{
	platform_driver_unregister(&amesom_tlv320_driver);
}

module_init(amesom_init);
module_exit(amesom_exit);

/* Module information */
MODULE_AUTHOR("Nicola Perrino");
MODULE_DESCRIPTION("ALSA SoC TLV320 Amesom");
MODULE_LICENSE("GPL");
