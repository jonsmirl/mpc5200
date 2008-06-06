/*
 *
 * Copyright (c) 2007 Milan Plzik <mmp@handhelds.org>
 *
 * based on spitz.c,
 * Authors: Liam Girdwood <liam.girdwood@wolfsonmicro.com>
 *          Richard Purdie <richard@openedhand.com>
 *
 * This code is released under GPL (GNU Public License) with
 * absolutely no warranty. Please see http://www.gnu.org/ for a
 * complete discussion of the GPL.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>

#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/initval.h>
#include <sound/soc-dapm.h>

#include <asm/mach-types.h>
#include <asm-arm/arch/gpio.h>
#include <asm/arch-pxa/h5400-asic.h>
#include <asm/arch-pxa/h5400-gpio.h>
#include <asm/hardware/samcop_base.h>

#include "pxa2xx-pcm.h"
#include "../codecs/ak4535.h"


#define H5000_OFF	0
#define H5000_HP	1
#define H5000_MIC	2

#define H5000_SPK_OFF	0
#define H5000_SPK_ON	1

static int h5000_spk_func = 0;
static int h5000_jack_func = 0;


static void h5000_ext_control(struct snd_soc_card *soc_card)
{
	switch (h5000_spk_func) {
	case H5000_SPK_OFF:
		snd_soc_dapm_disable_pin(soc_card, "Ext Spk");
		break;
	case H5000_SPK_ON:
		snd_soc_dapm_enable_pin(soc_card, "Ext Spk");
		break;
	default:
		printk (KERN_ERR "%s: invalid value %d for h5000_spk_func\n",
			__FUNCTION__, h5000_spk_func);
		break;
	};

	switch (h5000_jack_func) {
	case H5000_OFF:
		snd_soc_dapm_disable_pin(soc_card, "Headphone Jack");
		snd_soc_dapm_enable_pin(soc_card, "Internal Mic");
		snd_soc_dapm_disable_pin(soc_card, "Mic Jack");
		break;
	case H5000_HP:
		snd_soc_dapm_enable_pin(soc_card, "Headphone Jack");
		snd_soc_dapm_enable_pin(soc_card, "Internal Mic");
		snd_soc_dapm_disable_pin(soc_card, "Mic Jack");
		break;
	case H5000_MIC:
		snd_soc_dapm_disable_pin(soc_card, "Headphone Jack");
		snd_soc_dapm_disable_pin(soc_card, "Internal Mic");
		snd_soc_dapm_enable_pin(soc_card, "Mic Jack");
		break;
	default:
		printk(KERN_ERR "%s: invalid value %d for h5000_jack_func\n",
			__FUNCTION__, h5000_jack_func);
		break;
	};

	snd_soc_dapm_sync(soc_card);
};

static int h5000_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_card *soc_card = pcm_runtime->soc_card;

	h5000_ext_control(soc_card);
	return 0;
};

static void h5000_shutdown(struct snd_pcm_substream *substream)
{
};

static int h5000_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
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
	ret = snd_soc_dai_set_fmt(codec_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set cpu DAI configuration */
	ret = snd_soc_dai_set_fmt(cpu_dai, SND_SOC_DAIFMT_I2S |
		SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* set the codec system clock for DAC and ADC */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, clk,
		SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* set the I2S system clock as input (unused) */
	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA2XX_I2S_SYSCLK, 0,
		SND_SOC_CLOCK_OUT);
	if (ret < 0)
		return ret;

	return 0;
};

static struct snd_soc_ops h5000_ops = {
	.startup = h5000_startup,
	.shutdown = h5000_shutdown,
	.hw_params = h5000_hw_params,
};

static int h5000_get_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value [0] = h5000_jack_func;
	return 0;
};

static int h5000_set_jack(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *soc_card = snd_kcontrol_chip(kcontrol);

	if (h5000_jack_func == ucontrol->value.integer.value [0])
		return 0;

	h5000_jack_func = ucontrol->value.integer.value [0];
	h5000_ext_control(soc_card);
	return 1;
};

static int h5000_get_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	ucontrol->value.integer.value [0] = h5000_spk_func;
	return 0;
};

static int h5000_set_spk(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *soc_card = snd_kcontrol_chip(kcontrol);

	if (h5000_spk_func == ucontrol->value.integer.value [0])
		return 0;

	h5000_spk_func = ucontrol->value.integer.value [0];
	h5000_ext_control(soc_card);
	return 1;
};

static int h5000_audio_power(struct snd_soc_dapm_widget *widget,
	struct snd_kcontrol *k, int event)
{
	// mp - why do we need the ref count, dapm core should ref count all widget use.
	static int power_use_count = 0;

	if (SND_SOC_DAPM_EVENT_ON (event))
		power_use_count ++;
	else
		power_use_count --;

	samcop_set_gpio_b (&h5400_samcop.dev, SAMCOP_GPIO_GPB_AUDIO_POWER_ON,
		(power_use_count > 0) ? SAMCOP_GPIO_GPB_AUDIO_POWER_ON : 0 );

	return 0;
};

static const struct snd_soc_dapm_widget ak4535_dapm_widgets[] = {
	SND_SOC_DAPM_SPK ("Ext Spk", h5000_audio_power),
	SND_SOC_DAPM_HP ("Headphone Jack", h5000_audio_power),
	SND_SOC_DAPM_MIC ("Internal Mic", h5000_audio_power),
};

/* I'm really not sure about this, please fix if neccessary */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Speaker is connected to speaker +- pins */
	{ "Ext Spk", NULL, "SPP" },
	{ "Ext Spk", NULL, "SPN" },

	/* Mono input to MOUT2 according to reference */
	{ "MIN", NULL, "MOUT2" },

	/* Headphone output is connected to left and right output */
	{ "Headphone Jack", NULL, "HPL" },
	{ "Headphone Jack", NULL, "HPR" },

	/* MICOUT is connected to AIN */
	{ "AIN", NULL, "MICOUT"},

	/* Microphones */
	{ "MICIN", NULL, "Internal Mic" },
	{ "MICEXT", NULL, "Mic Jack" },
};

static const char *jack_function [] = { "Off", "Headphone", "Mic", };
static const char *spk_function [] = { "Off", "On", };

static const struct soc_enum h5000_enum [] = {
	SOC_ENUM_SINGLE_EXT (2, jack_function),
	SOC_ENUM_SINGLE_EXT (2, spk_function),
};

static const struct snd_kcontrol_new ak4535_h5000_controls[] = {
	SOC_ENUM_EXT ("Jack Function", h5000_enum[0], h5000_get_jack, h5000_set_jack),
	SOC_ENUM_EXT ("Speaker Function", h5000_enum[1], h5000_get_spk, h5000_set_spk),

};

#define AK4535_I2C_ADDR	0x10
static unsigned short normal_i2c[] = { AK4535_I2C_ADDR, I2C_CLIENT_END };

/* Magic definition of all other variables and things */
I2C_CLIENT_INSMOD;

static struct i2c_driver ak4535_i2c_driver;
static struct i2c_client client_template;

static int h5000_ak4535_write(void *control_data, long data, int size)
{
	return i2c_master_send((struct i2c_client*)control_data,
		(char*) data, size);
}

static int h5000_ak4535_init (struct snd_soc_card *soc_card)
{
	struct snd_soc_codec *codec;
	int ret;

	codec = snd_soc_card_get_codec(soc_card, ak4535_codec_id, 0);
	if (codec == NULL)
		return -ENODEV;

	/* NC codec pins */
	snd_soc_dapm_disable_pin(soc_card, "MOUT1");
	snd_soc_dapm_disable_pin(soc_card, "LOUT");
	snd_soc_dapm_disable_pin(soc_card, "ROUT");

	// mp - not sure I understand here, is the codec driver wrong ?
	snd_soc_dapm_disable_pin(soc_card, "MOUT2");	/* FIXME: These pins are marked as INPUTS */
	snd_soc_dapm_disable_pin(soc_card, "MIN");	/* FIXME: and OUTPUTS in ak4535.c . We need to do this in order */
	snd_soc_dapm_disable_pin(soc_card, "AIN");	/* FIXME: to get DAPM working properly, because the pins are connected */
	snd_soc_dapm_disable_pin(soc_card, "MICOUT");	/* FIXME: OUTPUT -> INPUT. */

	/* Add h5000 specific controls */
	ret = snd_soc_add_new_controls(soc_card, ak4535_h5000_controls,
		soc_card, ARRAY_SIZE(ak4535_h5000_controls));
	if (ret < 0)
		return ret;
	/* Add h5000 specific widgets */
	ret = snd_soc_dapm_new_controls(soc_card, codec,
			ak4535_dapm_widgets, ARRAY_SIZE(ak4535_dapm_widgets));
	if (ret < 0)
		return ret;
	/* Set up h5000 specific audio path audio_map */
	ret = snd_soc_dapm_add_routes(soc_card, audio_map,
				     ARRAY_SIZE(audio_map));
	if (ret < 0)
		return ret;

	snd_soc_card_config_codec(codec, NULL, h5000_ak4535_write,
		soc_card->private_data);

	snd_soc_dapm_sync(soc_card);

	snd_soc_card_init_codec(codec, soc_card);

	return 0;
};

static struct snd_soc_pcm_config hifi_pcm_config = {
	.name		= "HiFi",
	.codec		= ak4535_codec_id,
	.codec_dai	= ak4535_codec_dai_id,
	.platform	= pxa_platform_id,
	.cpu_dai	= pxa2xx_i2s_dai_id,
	.ops		= &h5000_ops,
	.playback	= 1,
	.capture	= 1,
};

static int h5000_i2c_probe(struct i2c_adapter *adap, int addr, int kind)
{
	struct snd_soc_card *soc_card;
	struct i2c_client *i2c;
	int ret;

	if (addr != AK4535_I2C_ADDR)
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

	soc_card = snd_soc_card_create("h5000", &i2c->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (soc_card == NULL)
		return -ENOMEM;

	soc_card->longname = "h5000";
	soc_card->init = h5000_ak4535_init;
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

static int h5000_i2c_detach(struct i2c_client *client)
{
	struct snd_soc_card *soc_card = i2c_get_clientdata(client);

	snd_soc_card_free(soc_card);
	i2c_detach_client(client);
	kfree(client);
	return 0;
}

static int h5000_i2c_attach(struct i2c_adapter *adap)
{
	return i2c_probe(adap, &addr_data, h5000_i2c_probe);
}

static struct i2c_driver ak4535_i2c_driver = {
	.driver = {
		.name = "ak4535 Codec",
		.owner = THIS_MODULE,
	},
	.id =             I2C_DRIVERID_AK4535,
	.attach_adapter = h5000_i2c_attach,
	.detach_client =  h5000_i2c_detach,
	.command =        NULL,
};

static struct i2c_client client_template = {
	.name =   "h5000",
	.driver = &ak4535_i2c_driver,
};

static int __init h5000_init(void)
{
	int ret;

	if (!machine_is_h5400 ())
		return -ENODEV;

	request_module("i2c-pxa");

	/* enable audio codec */
	samcop_set_gpio_b(&h5400_samcop.dev,
		SAMCOP_GPIO_GPB_CODEC_POWER_ON, SAMCOP_GPIO_GPB_CODEC_POWER_ON);

	ret = i2c_add_driver(&ak4535_i2c_driver);
	if (ret < 0)
		printk (KERN_ERR "%s: failed to add i2c driver\n",
			__FUNCTION__);

	return ret;
};

static void __exit h5000_exit(void)
{
	i2c_del_driver(&ak4535_i2c_driver);

	samcop_set_gpio_b(&h5400_samcop.dev,
		SAMCOP_GPIO_GPB_CODEC_POWER_ON | SAMCOP_GPIO_GPB_AUDIO_POWER_ON, 0);
};

module_init (h5000_init);
module_exit (h5000_exit);

MODULE_AUTHOR ("Milan Plzik");
MODULE_DESCRIPTION ("ALSA SoC iPAQ h5000");
MODULE_LICENSE ("GPL");
