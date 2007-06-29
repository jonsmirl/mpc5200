/*
 * corgi.c  --  SoC audio for Corgi
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Copyright 2005 Openedhand Ltd.
 *
 * Authors: Liam Girdwood <liam.girdwood@wolfsonmicro.com>
 *          Richard Purdie <richard@openedhand.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    30th Nov 2005   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <asm/mach-types.h>
#include <asm/hardware/scoop.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/hardware.h>
#include <asm/arch/corgi.h>
#include <asm/arch/audio.h>

#include "../codecs/wm8731.h"
#include "pxa2xx-pcm.h"
#include "pxa2xx-i2s.h"

#define CORGI_HP        0
#define CORGI_MIC       1
#define CORGI_LINE      2
#define CORGI_HEADSET   3
#define CORGI_HP_OFF    4
#define CORGI_SPK_ON    0
#define CORGI_SPK_OFF   1

 /* audio clock in Hz - rounded from 12.235MHz */
#define CORGI_AUDIO_CLOCK 12288000

static int corgi_jack_func;
static int corgi_spk_func;
static struct snd_soc_machine *corgi_mach;

static void corgi_ext_control(struct snd_soc_machine *machine)
{
	int spk = 0, mic = 0, line = 0, hp = 0, hs = 0;

	/* set up jack connection */
	switch (corgi_jack_func) {
	case CORGI_HP:
		hp = 1;
		/* set = unmute headphone */
		set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_L);
		set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_R);
		break;
	case CORGI_MIC:
		mic = 1;
		/* reset = mute headphone */
		reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_L);
		reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_R);
		break;
	case CORGI_LINE:
		line = 1;
		reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_L);
		reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_R);
		break;
	case CORGI_HEADSET:
		hs = 1;
		mic = 1;
		reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_L);
		set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_R);
		break;
	}

	if (corgi_spk_func == CORGI_SPK_ON)
		spk = 1;

	/* set the enpoints to their new connetion states */
	snd_soc_dapm_set_endpoint(machine, "Ext Spk", spk);
	snd_soc_dapm_set_endpoint(machine, "Mic Jack", mic);
	snd_soc_dapm_set_endpoint(machine, "Line Jack", line);
	snd_soc_dapm_set_endpoint(machine, "Headphone Jack", hp);
	snd_soc_dapm_set_endpoint(machine, "Headset Jack", hs);

	/* signal a DAPM event */
	snd_soc_dapm_sync_endpoints(machine);
}

static int corgi_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_machine *machine = pcm_link->machine;

	/* check the jack status at stream startup */
	corgi_ext_control(machine);
	return 0;
}

/* we need to unmute the HP at shutdown as the mute burns power on corgi */
static void corgi_shutdown(struct snd_pcm_substream *substream)
{
	/* set = unmute headphone */
	set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_L);
	set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MUTE_R);
}

static int corgi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	unsigned int clk = 0;
	int ret = 0;

	switch (params_rate(params)) {
	case 8000:
	case 16000:
	case 48000:
	case 96000:
		clk = 12288000;
		break;
	case 11025:
	case 22050:
	case 44100:
		clk = 11289600;
		break;
	}

	/* set codec DAI configuration */
	ret = codec_dai->ops->set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = cpu_dai->ops->set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = codec_dai->ops->set_sysclk(codec_dai, WM8731_SYSCLK, clk,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the I2S system clock as input (unused) */
	ret = cpu_dai->ops->set_sysclk(cpu_dai, PXA2XX_I2S_SYSCLK, 0,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops corgi_ops = {
	.startup = corgi_startup,
	.hw_params = corgi_hw_params,
	.shutdown = corgi_shutdown,
};

static int corgi_pcm_new(struct snd_soc_pcm_link *pcm_link)
{
	pcm_link->audio_ops = &corgi_ops;
	return snd_soc_pcm_new(pcm_link, 1, 1);
}

struct snd_soc_pcm_link_ops corgi_pcm = {
	.new	= corgi_pcm_new,
};

static int corgi_get_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = corgi_jack_func;
	return 0;
}

static int corgi_set_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_machine *machine = snd_kcontrol_chip(kcontrol);

	if (corgi_jack_func == ucontrol->value.integer.value[0])
		return 0;

	corgi_jack_func = ucontrol->value.integer.value[0];
	corgi_ext_control(machine);
	return 1;
}

static int corgi_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value[0] = corgi_spk_func;
	return 0;
}

static int corgi_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_machine *machine =  snd_kcontrol_chip(kcontrol);

	if (corgi_spk_func == ucontrol->value.integer.value[0])
		return 0;

	corgi_spk_func = ucontrol->value.integer.value[0];
	corgi_ext_control(machine);
	return 1;
}

static int corgi_amp_event(struct snd_soc_dapm_widget *w, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_APM_ON);
	else
		reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_APM_ON);

	return 0;
}

static int corgi_mic_event(struct snd_soc_dapm_widget *w, int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event))
		set_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MIC_BIAS);
	else
		reset_scoop_gpio(&corgiscoop_device.dev, CORGI_SCP_MIC_BIAS);

	return 0;
}

/* corgi machine dapm widgets */
static const struct snd_soc_dapm_widget wm8731_dapm_widgets[] = {
SND_SOC_DAPM_HP("Headphone Jack", NULL),
SND_SOC_DAPM_MIC("Mic Jack", corgi_mic_event),
SND_SOC_DAPM_SPK("Ext Spk", corgi_amp_event),
SND_SOC_DAPM_LINE("Line Jack", NULL),
SND_SOC_DAPM_HP("Headset Jack", NULL),
};

/* Corgi machine audio map (connections to the codec pins) */
static const char *audio_map[][3] = {

	/* headset Jack  - in = micin, out = LHPOUT*/
	{"Headset Jack", NULL, "LHPOUT"},

	/* headphone connected to LHPOUT1, RHPOUT1 */
	{"Headphone Jack", NULL, "LHPOUT"},
	{"Headphone Jack", NULL, "RHPOUT"},

	/* speaker connected to LOUT, ROUT */
	{"Ext Spk", NULL, "ROUT"},
	{"Ext Spk", NULL, "LOUT"},

	/* mic is connected to MICIN (via right channel of headphone jack) */
	{"MICIN", NULL, "Mic Jack"},

	/* Same as the above but no mic bias for line signals */
	{"MICIN", NULL, "Line Jack"},

	{NULL, NULL, NULL},
};

static const char *jack_function[] = {"Headphone", "Mic", "Line", "Headset",
	"Off"};
static const char *spk_function[] = {"On", "Off"};
static const struct soc_enum corgi_enum[] = {
	SOC_ENUM_SINGLE_EXT(5, jack_function),
	SOC_ENUM_SINGLE_EXT(2, spk_function),
};

static const struct snd_kcontrol_new wm8731_corgi_controls[] = {
	SOC_ENUM_EXT("Jack Function", corgi_enum[0], corgi_get_jack,
		corgi_set_jack),
	SOC_ENUM_EXT("Speaker Function", corgi_enum[1], corgi_get_spk,
		corgi_set_spk),
};

/*
 * WM8731 2 wire address is determined by GPIO5
 * state during powerup.
 *    low  = 0x1a
 *    high = 0x1b
 */
#define WM8731_I2C_ADDR	0x1b
static unsigned short normal_i2c[] = { WM8731_I2C_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver wm8731_i2c_driver;
static struct i2c_client client_template;

static int corgi_wm8731_write(void *control_data, long data, int size)
{
	return i2c_master_send((struct i2c_client*)control_data, 
		(char*) data, size);
}

/*
 * Logic for a wm8731 as connected on a Sharp SL-C7x0 Device
 */
static int corgi_mach_probe(struct snd_soc_machine *machine)
{
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_link *pcm_link;
	int i, ret;
	
	pcm_link = list_first_entry(&machine->active_list, 
		struct snd_soc_pcm_link, active_list);
	codec = pcm_link->codec;
		
	/* set up corgi codec pins */
	snd_soc_dapm_set_endpoint(machine, "LLINEIN", 0);
	snd_soc_dapm_set_endpoint(machine, "RLINEIN", 0);

	/* add corgi specific controls */
	for (i = 0; i < ARRAY_SIZE(wm8731_corgi_controls); i++) {
		if ((ret = snd_ctl_add(machine->card,
				snd_soc_cnew(&wm8731_corgi_controls[i],
					machine, NULL))) < 0)
			return ret;
	}

	/* Add corgi specific widgets */
	for(i = 0; i < ARRAY_SIZE(wm8731_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec, 
			&wm8731_dapm_widgets[i]);
	}

	/* Set up corgi specific audio path audio_map */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}
	
	snd_soc_dapm_sync_endpoints(machine);
	
	codec->control_data = corgi_mach->private_data;
	codec->mach_write = corgi_wm8731_write;
	codec->ops->probe_codec(codec, corgi_mach);
	
	/* register card with ALSA upper layers */
	ret = snd_soc_register_card(corgi_mach);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to register sound card\n",
			__FUNCTION__);
		return ret;
	}
	
	return 0;
}

struct snd_soc_machine_ops corgi_mach_ops = {
	.mach_probe = corgi_mach_probe,	
};

static int wm8731_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct i2c_client *i2c;
	struct snd_soc_pcm_link *hifi;
	int ret;

	if (addr != WM8731_I2C_ADDR)
		return -ENODEV;

	client_template.adapter = adap;
	client_template.addr = addr;

	i2c = kmemdup(&client_template, sizeof(client_template), GFP_KERNEL);
	if (i2c == NULL)
		return -ENOMEM;
	i2c_set_clientdata(i2c, corgi_mach);
	corgi_mach->private_data = i2c;
	
	ret = i2c_attach_client(i2c);
	if (ret < 0) {
		printk("failed to attach codec at addr %x\n", addr);
		goto attach_err;
	}
	
	/* corgi wm8731 hifi interface */
	hifi = snd_soc_pcm_link_new(corgi_mach, "corgi-hifi", 
		&corgi_pcm, pxa2xx_pcm, wm8731_codec, wm8731_hifi_dai, 
		pxa2xx_i2s);
	if (hifi == NULL) {
		printk("failed to create HiFi PCM link\n");
		goto attach_err;
	}
	ret =  snd_soc_pcm_link_attach(hifi);
	if (ret < 0) 
		goto link_err;
	
	return ret;

link_err:
	snd_soc_machine_free(corgi_mach);
attach_err:
	i2c_detach_client(i2c);
	kfree(i2c);
	return ret;
}

static int wm8731_i2c_detach(struct i2c_client *client)
{
	snd_soc_machine_free(corgi_mach);
	i2c_detach_client(client);
	kfree(client);
	return 0;
}

static int wm8731_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, wm8731_i2c_probe);
}

static struct i2c_driver wm8731_i2c_driver = {
	.driver = {
		.name = "WM8731 I2C Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_WM8731,
	.attach_adapter = wm8731_i2c_attach,
	.detach_client =  wm8731_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "WM8731",
	.driver = &wm8731_i2c_driver,
};

static int __init corgi_wm8731_probe(struct platform_device *pdev)
{
	struct snd_soc_machine *machine;
	int ret;

	machine = kzalloc(sizeof(struct snd_soc_machine), GFP_KERNEL);
	if (machine == NULL)
		return -ENOMEM;

	machine->owner = THIS_MODULE;
	machine->pdev = pdev;
	machine->name = "corgi";
	machine->longname = "wm8731";
	machine->ops = &corgi_mach_ops;
	pdev->dev.driver_data = machine;

	/* register card */
	corgi_mach = machine;
	ret = snd_soc_new_card(machine, 1, SNDRV_DEFAULT_IDX1, 
		SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		printk(KERN_ERR "%s: failed to create pcms\n", __func__);
		kfree(machine);
		return ret;
	}
	
	/* register I2C driver for WM8731 codec control */
	ret = i2c_add_driver(&wm8731_i2c_driver);
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

static int __exit corgi_wm8731_remove(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	
	i2c_del_driver(&wm8731_i2c_driver);
	corgi_mach = NULL;
	kfree(machine);
	return 0;
}

#ifdef CONFIG_PM
static int corgi_wm8731_suspend(struct platform_device *pdev, 
	pm_message_t state)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	return snd_soc_suspend(machine, state);
}

static int corgi_wm8731_resume(struct platform_device *pdev)
{
	struct snd_soc_machine *machine = pdev->dev.driver_data;
	return snd_soc_resume(machine);
}

#else
#define corgi_wm8731_suspend NULL
#define corgi_wm8731_resume  NULL
#endif

static struct platform_driver corgi_wm8731_driver = {
	.probe		= corgi_wm8731_probe,
	.remove		= __devexit_p(corgi_wm8731_remove),
	.suspend	= corgi_wm8731_suspend,
	.resume		= corgi_wm8731_resume,
	.driver		= {
		.name 		= "corgi-wm8731",
		.owner		= THIS_MODULE,
	},
};

static int __init corgi_asoc_init(void)
{
	return platform_driver_register(&corgi_wm8731_driver);
}

static void __exit corgi_asoc_exit(void)
{
	platform_driver_unregister(&corgi_wm8731_driver);
}

module_init(corgi_asoc_init);
module_exit(corgi_asoc_exit);

/* Module information */
MODULE_AUTHOR("Richard Purdie");
MODULE_DESCRIPTION("ALSA SoC Corgi");
MODULE_LICENSE("GPL");
