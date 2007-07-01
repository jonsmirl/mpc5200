/*
 * mainstone.c  --  SoC audio for Mainstone
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  Mainstone audio amplifier code taken from arch/arm/mach-pxa/mainstone.c
 *  Copyright:	MontaVista Software Inc.
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    30th Oct 2005   Initial version.
 *
 * 
 * This is an example machine driver for a wm8753 connected to a
 * Mainstone II. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mainstone.h>
#include <asm/arch/audio.h>

#include "../codecs/wm8753.h"
#include "pxa2xx-pcm.h"
#include "pxa2xx-i2s.h"
#include "pxa2xx-ssp.h"

/*
 * SSP GPIO's
 */
#define GPIO11_SSP2RX_MD	(11 | GPIO_ALT_FN_2_IN)
#define GPIO13_SSP2TX_MD	(13 | GPIO_ALT_FN_1_OUT)
#define GPIO22_SSP2CLKS_MD	(22 | GPIO_ALT_FN_3_IN)
#define GPIO88_SSP2FRMS_MD	(88 | GPIO_ALT_FN_3_IN)
#define GPIO22_SSP2CLKM_MD	(22 | GPIO_ALT_FN_3_OUT)
#define GPIO88_SSP2FRMM_MD	(88 | GPIO_ALT_FN_3_OUT)
#define GPIO22_SSP2SYSCLK_MD	(22 | GPIO_ALT_FN_2_OUT)

static struct snd_soc_machine *mainstone_mach;

static int mainstone_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	unsigned int pll_out = 0, bclk = 0, fmt = 0;
	int ret = 0;

	/*
	 * The WM8753 is far better at generating accurate audio clocks than the
	 * pxa2xx I2S controller, so we will use it as master when we can.
	 * i.e all rates except 8k and 16k as BCLK must be 64 * rate when the
	 * pxa27x or pxa25x is slave. Note this restriction does not apply to SSP
	 * I2S emulation mode.
	 */
	switch (params_rate(params)) {
	case 8000:
	case 16000:
		fmt = SND_SOC_DAIFMT_CBS_CFS;
		pll_out = 12288000;
		break;
	case 48000:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 12288000;
		break;
	case 96000:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 12288000;
		break;
	case 11025:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_16;
		pll_out = 11289600;
		break;
	case 22050:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_8;
		pll_out = 11289600;
		break;
	case 44100:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_4;
		pll_out = 11289600;
		break;
	case 88200:
		fmt = SND_SOC_DAIFMT_CBM_CFS;
		bclk = WM8753_BCLK_DIV_2;
		pll_out = 11289600;
		break;
	}

	/* set codec DAI configuration */
	ret = codec_dai->ops->set_fmt(codec_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | fmt);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->ops->set_fmt(cpu_dai,
		SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF | fmt);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->ops->set_sysclk(codec_dai, WM8753_MCLK, pll_out,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the I2S system clock as input (unused) */
	ret = cpu_dai->ops->set_sysclk(cpu_dai, PXA2XX_I2S_SYSCLK, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec BCLK division for sample rate */
	ret = codec_dai->ops->set_clkdiv(codec_dai, WM8753_BCLKDIV, bclk);
	if (ret < 0)
		return ret;

	/* codec PLL input is 13 MHz */
	ret = codec_dai->ops->set_pll(codec_dai, WM8753_PLL1, 13000000, pll_out);
	if (ret < 0)
		return ret;

	return 0;
}

static int mainstone_hifi_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;

	/* disable the PLL */
	return codec_dai->ops->set_pll(codec_dai, WM8753_PLL1, 0, 0);
}

/*
 * Mainstone WM8753 HiFi DAI opserations.
 */
static const struct snd_soc_ops mainstone_hifi_ops = {
	.hw_params = mainstone_hifi_hw_params,
	.hw_free = mainstone_hifi_hw_free,
};

static int hifi_pcm_new(struct snd_soc_pcm_link *pcm_link)
{
	pcm_link->audio_ops = &mainstone_hifi_ops;
	return snd_soc_pcm_new(pcm_link, 1, 1);
}

struct snd_soc_pcm_link_ops hifi_pcm = {
	.new	= hifi_pcm_new,
};

static int mainstone_voice_startup(struct snd_pcm_substream *substream)
{
	/* enable USB on the go MUX so we can use SSPFRM2 */
	MST_MSCWR2 |= MST_MSCWR2_USB_OTG_SEL;
	MST_MSCWR2 &= ~MST_MSCWR2_USB_OTG_RST;

	return 0;
}

static void mainstone_voice_shutdown(struct snd_pcm_substream *substream)
{
	/* disable USB on the go MUX so we can use ttyS0 */
	MST_MSCWR2 &= ~MST_MSCWR2_USB_OTG_SEL;
	MST_MSCWR2 |= MST_MSCWR2_USB_OTG_RST;
}

static int mainstone_voice_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	unsigned int pll_out = 0, bclk = 0, pcmdiv = 0;
	int ret = 0;

	/*
	 * The WM8753 is far better at generating accurate audio clocks than the
	 * pxa2xx SSP controller, so we will use it as master when we can.
	 */
	switch (params_rate(params)) {
	case 8000:
		pll_out = 12288000;
		pcmdiv = WM8753_PCM_DIV_6; /* 2.048 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 256kHz */
		break;
	case 16000:
		pll_out = 12288000;
		pcmdiv = WM8753_PCM_DIV_3; /* 4.096 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 512kHz */
		break;
	case 48000:
		pll_out = 12288000;
		pcmdiv = WM8753_PCM_DIV_1; /* 12.288 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 1.536 MHz */
		break;
	case 11025:
		pll_out = 11289600;
		pcmdiv = WM8753_PCM_DIV_4; /* 11.2896 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 352.8 kHz */
		break;
	case 22050:
		pll_out = 11289600;
		pcmdiv = WM8753_PCM_DIV_2; /* 11.2896 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 705.6 kHz */
		break;
	case 44100:
		pll_out = 11289600;
		pcmdiv = WM8753_PCM_DIV_1; /* 11.2896 MHz */
		bclk = WM8753_VXCLK_DIV_8; /* 1.4112 MHz */
		break;
	}

	/* set codec DAI configuration */
	ret = codec_dai->ops->set_fmt(codec_dai, SND_SOC_DAIFMT_DSP_A |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->ops->set_fmt(cpu_dai, SND_SOC_DAIFMT_DSP_A |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBM_CFM);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->ops->set_sysclk(codec_dai, WM8753_PCMCLK, pll_out,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the SSP system clock as input (unused) */
	ret = cpu_dai->ops->set_sysclk(cpu_dai, PXA2XX_SSP_CLK_PLL, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec BCLK division for sample rate */
	ret = codec_dai->ops->set_clkdiv(codec_dai, WM8753_VXCLKDIV, bclk);
	if (ret < 0)
		return ret;

	/* set codec PCM division for sample rate */
	ret = codec_dai->ops->set_clkdiv(codec_dai, WM8753_PCMDIV, pcmdiv);
	if (ret < 0)
		return ret;

	/* codec PLL input is 13 MHz */
	ret = codec_dai->ops->set_pll(codec_dai, WM8753_PLL2, 13000000, pll_out);
	if (ret < 0)
		return ret;

	return 0;
}

static int mainstone_voice_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;

	/* disable the PLL */
	return codec_dai->ops->set_pll(codec_dai, WM8753_PLL2, 0, 0);
}

static const struct snd_soc_ops mainstone_voice_ops = {
	.startup = mainstone_voice_startup,
	.shutdown = mainstone_voice_shutdown,
	.hw_params = mainstone_voice_hw_params,
	.hw_free = mainstone_voice_hw_free,
};

static int voice_pcm_new(struct snd_soc_pcm_link *pcm_link)
{
	/* mainstone wm8753 voice interface */
	pxa_gpio_mode(GPIO11_SSP2RX_MD);
	pxa_gpio_mode(GPIO13_SSP2TX_MD);
	pxa_gpio_mode(GPIO22_SSP2CLKS_MD);
	pxa_gpio_mode(GPIO88_SSP2FRMS_MD);
	
	pcm_link->audio_ops = &mainstone_voice_ops;
	return snd_soc_pcm_new(pcm_link, 1, 1);
}

struct snd_soc_pcm_link_ops voice_pcm = {
	.new	= voice_pcm_new,
};

/* example machine audio_mapnections */
static const char* audio_map[][3] = {

	/* mic is connected to mic1 - with bias */
	{"MIC1", NULL, "Mic Bias"},
	{"MIC1N", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic1 Jack"},
	{"Mic Bias", NULL, "Mic1 Jack"},

	{"ACIN", NULL, "ACOP"},
	{NULL, NULL, NULL},
};

/* headphone detect support on my board */
static const char * hp_pol[] = {"Headphone", "Speaker"};
static const struct soc_enum wm8753_enum =
	SOC_ENUM_SINGLE(WM8753_OUTCTL, 1, 2, hp_pol);

static const struct snd_kcontrol_new wm8753_mainstone_controls[] = {
	SOC_SINGLE("Headphone Detect Switch", WM8753_OUTCTL, 6, 1, 0),
	SOC_ENUM("Headphone Detect Polarity", wm8753_enum),
};

/*
 * WM8753 2 wire address is determined by GPIO5
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1b
 */
#define WM8753_I2C_ADDR	0x1a
static unsigned short normal_i2c[] = { WM8753_I2C_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver wm8753_i2c_driver;
static struct i2c_client client_template;

static int mainstone_wm8753_write(void *control_data, long data, int size)
{
	return i2c_master_send((struct i2c_client*)control_data, 
		(char*) data, size);
}

static int mainstone_mach_probe(struct snd_soc_machine *machine)
{
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_link *pcm_link;
	int i, ret;
	
	pcm_link = list_first_entry(&machine->active_list, 
		struct snd_soc_pcm_link, active_list);
	codec = pcm_link->codec;
		
	/* set up mainstone codec pins */
	snd_soc_dapm_set_endpoint(machine, "RXP", 0);
	snd_soc_dapm_set_endpoint(machine, "RXN", 0);
	snd_soc_dapm_set_endpoint(machine, "MIC2", 0);

	/* add mainstone specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8753_mainstone_controls); i++) {
		if ((ret = snd_ctl_add(machine->card,
				snd_soc_cnew(&wm8753_mainstone_controls[i],
					machine, NULL))) < 0)
			return ret;
	}

	/* set up mainstone specific audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, audio_map[i][0], 
			audio_map[i][1], audio_map[i][2]);
	}
	
	snd_soc_dapm_sync_endpoints(machine);
	
	codec->control_data = mainstone_mach->private_data;
	codec->mach_write = mainstone_wm8753_write;
	codec->ops->io_probe(codec, mainstone_mach);
	
	/* register card with ALSA upper layers */
	ret = snd_soc_register_card(mainstone_mach);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register sound card\n",
			__FUNCTION__);
		return ret;
	}
	
	/* enable speaker */
	MST_MSCWR2 &= ~MST_MSCWR2_AC97_SPKROFF;
	return 0;
}

struct snd_soc_machine_ops mainstone_mach_ops = {
	.mach_probe = mainstone_mach_probe,	
};

static int wm8753_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *i2c;
	struct snd_soc_pcm_link * hifi, *voice;
	int ret;

	if (addr != WM8753_I2C_ADDR)
		return -ENODEV;

	client_template.adapter = adap;
	client_template.addr = addr;

	i2c = kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL)
		return -ENOMEM;
	i2c_set_clientdata(i2c, mainstone_mach);
	mainstone_mach->private_data = i2c;
	
	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		printk("failed to attach codec at addr %x\n", addr);
		goto attach_err;
	}
	
	/* mainstone wm8753 hifi interface */
	hifi = snd_soc_pcm_link_new(mainstone_mach, "mainstone-hifi", 
		&hifi_pcm, pxa2xx_pcm, wm8753_codec, wm8753_hifi_dai, 
		pxa2xx_i2s);
	if (hifi == NULL) {
		printk("failed to create HiFi PCM link\n");
		goto attach_err;
	}
	ret =  snd_soc_pcm_link_attach(hifi);
	if (ret < 0) 
		goto link_err;
		
	voice = snd_soc_pcm_link_new(mainstone_mach, "mainstone-voice",
		&voice_pcm, pxa2xx_pcm, wm8753_codec, wm8753_voice_dai, 
		pxa2xx_ssp_2);
	if (hifi == NULL) {
		printk("failed to create HiFi PCM link\n");
		goto link_err;
	}
	ret =  snd_soc_pcm_link_attach(voice);
	if (ret < 0) 
		goto link_err;
	
	return ret;

link_err:
	snd_soc_machine_free(mainstone_mach);
attach_err:
	i2c_detach_client(i2c);
	kfree(i2c);
	return ret;
}

static int wm8753_i2c_detach(struct i2c_client *client)
{
	snd_soc_machine_free(mainstone_mach);
	i2c_detach_client(client);
	kfree(client);
	return 0;
}

static int wm8753_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, wm8753_i2c_probe);
}

static struct i2c_driver wm8753_i2c_driver = {
	.driver = {
		.name = "WM8753 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_WM8753,
	.attach_adapter = wm8753_i2c_attach,
	.detach_client =  wm8753_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "WM8753",
	.driver = &wm8753_i2c_driver,
};

/*
 * This function will register the snd_soc_pcm_link drivers.
 * It also registers devices for platform DMA, I2S, SSP and registers an 
 * I2C driver to probe the codec.
 */
static int __init mainstone_wm8753_probe(struct platform_device *pdev)
{
	struct snd_soc_machine *machine;
	int ret;

	machine = kzalloc(sizeof(struct snd_soc_machine), GFP_KERNEL);
	if (machine == NULL)
		return -ENOMEM;

	machine->owner = THIS_MODULE;
	machine->pdev = pdev;
	machine->name = "Mainstone";
	machine->longname = "WM8753";
	machine->ops = &mainstone_mach_ops;
	pdev->dev.driver_data = machine;

	/* register card */
	mainstone_mach = machine;
	ret = snd_soc_new_card(machine, 2, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to create pcms\n", __func__);
		kfree(machine);
		return ret;
	}
	
	/* register I2C driver for WM8753 codec control */
	ret = i2c_add_driver(&wm8753_i2c_driver);
	if (ret < 0) { 
		printk (KERN_ERR "%s: failed to add i2c driver\n",
			__FUNCTION__);
		goto err;
	}

	return ret;
	
err:
	kfree(machine);
	return ret;
}

static int __exit mainstone_wm8753_remove(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	
	i2c_del_driver(&wm8753_i2c_driver);
	mainstone_mach = NULL;
	kfree(machine);
	
	/* disable speaker */
	MST_MSCWR2 |= MST_MSCWR2_AC97_SPKROFF;
	return 0;
}

#ifdef CONFIG_PM
static long mst_audio_suspend_mask;

static int mainstone_wm8753_suspend(struct platform_device *pdev, 
	pm_message_t state)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	
	mst_audio_suspend_mask = MST_MSCWR2;
	MST_MSCWR2 |= MST_MSCWR2_AC97_SPKROFF;
	return snd_soc_suspend(machine, state);
}

static int mainstone_wm8753_resume(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	
	MST_MSCWR2 &= mst_audio_suspend_mask | ~MST_MSCWR2_AC97_SPKROFF;
	return snd_soc_resume(machine);
}

#else
#define mainstone_machine_suspend NULL
#define mainstone_machine_resume  NULL
#endif

static struct platform_driver mainstone_wm8753_driver = {
	.probe		= mainstone_wm8753_probe,
	.remove		= __devexit_p(mainstone_wm8753_remove),
	.suspend	= mainstone_wm8753_suspend,
	.resume		= mainstone_wm8753_resume,
	.driver		= {
		.name 		= "Mainstone-WM8753",
		.owner		= THIS_MODULE,
	},
};

static int __init mainstone_asoc_init(void)
{
	return platform_driver_register(&mainstone_wm8753_driver);
}

static void __exit mainstone_asoc_exit(void)
{
	platform_driver_unregister(&mainstone_wm8753_driver);
}

module_init(mainstone_asoc_init);
module_exit(mainstone_asoc_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("ALSA SoC WM8753 Mainstone");
MODULE_LICENSE("GPL");
