/*
 * mainstone.c  --  SoC audio for Mainstone
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
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
 *    29th Jan 2006   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>

#include <asm/hardware.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mainstone.h>
#include <asm/arch/audio.h>

#include "../codecs/wm9713.h"
#include "pxa2xx-pcm.h"
#include "pxa2xx-ac97.h"
#include "pxa2xx-ssp.h"

#define GPIO11_SSP2RX_MD	(11 | GPIO_ALT_FN_2_IN)
#define GPIO13_SSP2TX_MD	(13 | GPIO_ALT_FN_1_OUT)
#define GPIO22_SSP2CLKS_MD	(22 | GPIO_ALT_FN_3_IN)
#define GPIO88_SSP2FRMS_MD	(88 | GPIO_ALT_FN_3_IN)
#define GPIO22_SSP2CLKM_MD	(22 | GPIO_ALT_FN_3_OUT)
#define GPIO88_SSP2FRMM_MD	(88 | GPIO_ALT_FN_3_OUT)
#define GPIO22_SSP2SYSCLK_MD	(22 | GPIO_ALT_FN_2_OUT)

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
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	unsigned int bclk = 0, pcmdiv = 0;
	int ret = 0;

	switch (params_rate(params)) {
	case 8000:
		pcmdiv = WM9713_PCMDIV(12); /* 2.048 MHz */
		bclk = WM9713_PCMBCLK_DIV_16; /* 128kHz */
		break;
	case 16000:
		pcmdiv = WM9713_PCMDIV(6); /* 4.096 MHz */
		bclk = WM9713_PCMBCLK_DIV_16; /* 256kHz */
		break;
	case 48000:
		pcmdiv = WM9713_PCMDIV(2); /* 12.288 MHz */
		bclk = WM9713_PCMBCLK_DIV_16; /* 512kHz */
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

	/* set the SSP system clock as input (unused) */
	ret = cpu_dai->ops->set_sysclk(cpu_dai, PXA2XX_SSP_CLK_PLL, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set codec BCLK division for sample rate */
	ret = codec_dai->ops->set_clkdiv(codec_dai, WM9713_PCMBCLK_DIV, bclk);
	if (ret < 0)
		return ret;

	/* set codec PCM division for sample rate */
	ret = codec_dai->ops->set_clkdiv(codec_dai, WM9713_PCMCLK_DIV, pcmdiv);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct snd_soc_ops mainstone_voice_ops = {
	.startup = mainstone_voice_startup,
	.shutdown = mainstone_voice_shutdown,
	.hw_params = mainstone_voice_hw_params,
};

static int hifi_pcm_new(struct snd_soc_pcm_link *pcm_link)
{
	return snd_soc_pcm_new(pcm_link, 1, 1);
}

static struct snd_soc_pcm_link_ops hifi_pcm = {
	.new	= hifi_pcm_new,
};

static int aux_pcm_new(struct snd_soc_pcm_link *pcm_link)
{
	return snd_soc_pcm_new(pcm_link, 1, 0);
}

static struct snd_soc_pcm_link_ops aux_pcm = {
	.new	= aux_pcm_new,
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

static struct snd_soc_pcm_link_ops voice_pcm = {
	.new	= voice_pcm_new,
};

/* mainstone machine dapm widgets */
static const struct snd_soc_dapm_widget mainstone_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("Mic 1", NULL),
	SND_SOC_DAPM_MIC("Mic 2", NULL),
	SND_SOC_DAPM_MIC("Mic 3", NULL),
};

/* example machine audio_mapnections */
static const char* audio_map[][3] = {

	/* mic is connected to mic1 - with bias */
	{"MIC1", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic 1"},
	/* mic is connected to mic2A - with bias */
	{"MIC2A", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic 2"},
	/* mic is connected to mic2B - with bias */
	{"MIC2B", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "Mic 3"},

	{NULL, NULL, NULL},
};

static int mainstone_wm9713_write(void *control_data, long val, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	ac97->bus->ops->write(ac97, reg, val);
	return 0;
}

static int mainstone_wm9713_read(void *control_data, long val, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	val = ac97->bus->ops->read(ac97, reg);
	return 0;
}

static int mainstone_mach_probe(struct snd_soc_machine *machine)
{
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_link *pcm_link;
	struct snd_ac97_bus_ops *ac97_ops;
	int i, ret;

	pcm_link = list_first_entry(&machine->active_list, 
		struct snd_soc_pcm_link, active_list);
	codec = pcm_link->codec;
	
	codec->control_data = codec->ac97;
	codec->mach_write = mainstone_wm9713_write;
	codec->mach_read = mainstone_wm9713_read;
	ac97_ops = pcm_link->cpu_dai->ac97_ops;
	
	/* register with AC97 bus for ad-hoc driver access */
	ret = snd_soc_new_ac97_codec(pcm_link, ac97_ops, 0);
	if (ret < 0)
		return ret;
		
	/* do a cold reset for the controller and then try
	 * a warm reset followed by an optional cold reset for codec */
	ac97_ops->reset(codec->ac97);
	ac97_ops->warm_reset(codec->ac97);
	if (ac97_ops->read(codec->ac97, AC97_VENDOR_ID1) == 0) { //lg
		printk(KERN_ERR "AC97 link error\n");
		return ret;
	}	
	codec->ops->probe_codec(codec, machine);

	/* set up mainstone codec pins */
	snd_soc_dapm_set_endpoint(machine, "RXP", 0);
	snd_soc_dapm_set_endpoint(machine, "RXN", 0);
#if 0
	/* Add test specific controls */
	for (i = 0; i < ARRAY_SIZE(mainstone_controls); i++) {
		if ((ret = snd_ctl_add(machine->card,
				snd_soc_cnew(&mainstone_controls[i], 
					codec, NULL))) < 0)
			return ret;
	}
#endif
	/* Add mainstone specific widgets */
	for(i = 0; i < ARRAY_SIZE(mainstone_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec, 
			&mainstone_dapm_widgets[i]);
	}

	/* set up mainstone specific audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, audio_map[i][0], 
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_sync_endpoints(machine);
	
	/* register card with ALSA upper layers */
	ret = snd_soc_register_card(machine);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register sound card\n",
			__FUNCTION__);
		return ret;
	}
	
	MST_MSCWR2 &= ~MST_MSCWR2_AC97_SPKROFF;
	return 0;
}

struct snd_soc_machine_ops mainstone_mach_ops = {
	.mach_probe = mainstone_mach_probe,	
};

/*
 * This is an example machine initialisation for a wm9713 connected to a
 * Mainstone II. It is missing logic to detect hp/mic insertions and logic
 * to re-route the audio in such an event.
 */
static int mainstone_wm9713_probe(struct platform_device *pdev)
{
	struct snd_soc_machine *machine;
	struct snd_soc_pcm_link * hifi, *voice, *aux;
	int ret;

	machine = kzalloc(sizeof(struct snd_soc_machine), GFP_KERNEL);
	if (machine == NULL)
		return -ENOMEM;

	machine->owner = THIS_MODULE;
	machine->pdev = pdev;
	machine->name = "Mainstone";
	machine->longname = "WM9713";
	machine->ops = &mainstone_mach_ops;
	pdev->dev.driver_data = machine;
	
	/* register card */
	ret = snd_soc_new_card(machine, 3, SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to create sound card\n", __func__);
		goto card_err;
	}

	/* mainstone wm9713 hifi interface */
	hifi = snd_soc_pcm_link_new(machine, "mainstone-hifi", &hifi_pcm,
		pxa2xx_pcm, wm9713_codec, wm9713_hifi_dai, pxa2xx_ac97_hifi);
	if (hifi == NULL) {
		printk("failed to create HiFi PCM link\n");
		goto link_err;
	}
	ret =  snd_soc_pcm_link_attach(hifi);
	if (ret < 0) 
		goto link_err;
	
	/* mainstone wm8753 aux interface */
	aux = snd_soc_pcm_link_new(machine, "mainstone-aux", &aux_pcm,
		pxa2xx_pcm, wm9713_codec, wm9713_aux_dai, pxa2xx_ac97_aux);
	if (aux == NULL) {
		printk("failed to create AUX PCM link\n");
		goto link_err;
	}
	ret =  snd_soc_pcm_link_attach(aux);
	if (ret < 0) 
		goto link_err;
	
	voice = snd_soc_pcm_link_new(machine, "mainstone-voice", &voice_pcm,
		pxa2xx_pcm, wm9713_codec, wm9713_voice_dai, pxa2xx_ssp_2);
	if (voice == NULL) {
		printk("failed to create Voice PCM link\n");
		goto link_err;
	}
	ret =  snd_soc_pcm_link_attach(voice);
	if (ret < 0) 
		goto link_err;	
	return 0;

link_err:
	snd_soc_machine_free(machine);
card_err:
	kfree(machine);
	return ret;
}

static int __exit mainstone_wm9713_remove(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;

	snd_soc_machine_free(machine);
	kfree(machine);

	/* disable speaker */
	MST_MSCWR2 |= MST_MSCWR2_AC97_SPKROFF;
	return 0;
}

#ifdef CONFIG_PM
static long mst_audio_suspend_mask;

static int mainstone_wm9713_suspend(struct platform_device *pdev, 
	pm_message_t state)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	
	mst_audio_suspend_mask = MST_MSCWR2;
	MST_MSCWR2 |= MST_MSCWR2_AC97_SPKROFF;
	return snd_soc_suspend(machine, state);
}

static int mainstone_wm9713_resume(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	
	MST_MSCWR2 &= mst_audio_suspend_mask | ~MST_MSCWR2_AC97_SPKROFF;
	return snd_soc_resume(machine);
}

#else
#define mainstone_machine_suspend NULL
#define mainstone_machine_resume  NULL
#endif

static struct platform_driver mainstone_wm9713_driver = {
	.probe		= mainstone_wm9713_probe,
	.remove		= __devexit_p(mainstone_wm9713_remove),
	.suspend	= mainstone_wm9713_suspend,
	.resume		= mainstone_wm9713_resume,
	.driver		= {
		.name 		= "Mainstone-WM9713",
		.owner		= THIS_MODULE,
	},
};

static int __init mainstone_asoc_init(void)
{
	return platform_driver_register(&mainstone_wm9713_driver);
}

static void __exit mainstone_asoc_exit(void)
{
	platform_driver_unregister(&mainstone_wm9713_driver);
}

module_init(mainstone_asoc_init);
module_exit(mainstone_asoc_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("ALSA SoC WM9713 Mainstone");
MODULE_LICENSE("GPL");
