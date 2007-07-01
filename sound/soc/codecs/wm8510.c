/*
 * wm8510.c  --  WM8510 ALSA Soc Audio driver
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 *
 * Author: Liam Girdwood <liam.girdwood@wolfsonmicro.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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

#include "wm8510.h"

#define AUDIO_NAME "wm8510"
#define WM8510_VERSION "0.6"

/*
 * Debug
 */

#define WM8510_DEBUG 0

#ifdef WM8510_DEBUG
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


/*
 * wm8510 register cache
 * We can't read the WM8510 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8510_reg[WM8510_CACHEREGNUM] = {
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0050, 0x0000, 0x0140, 0x0000,
    0x0000, 0x0000, 0x0000, 0x00ff,
    0x0000, 0x0000, 0x0100, 0x00ff,
    0x0000, 0x0000, 0x012c, 0x002c,
    0x002c, 0x002c, 0x002c, 0x0000,
    0x0032, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0038, 0x000b, 0x0032, 0x0000,
    0x0008, 0x000c, 0x0093, 0x00e9,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0003, 0x0010, 0x0000, 0x0000,
    0x0000, 0x0002, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0039, 0x0000,
    0x0000,
};

/*
 * read wm8510 register cache
 */
static inline unsigned int wm8510_read_reg_cache(struct snd_soc_codec * codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == WM8510_RESET)
		return 0;
	if (reg >= WM8510_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write wm8510 register cache
 */
static inline void wm8510_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WM8510_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the WM8510 register space
 */
static int wm8510_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	/* data is
	 *   D15..D9 WM8510 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	wm8510_write_reg_cache (codec, reg, value);
	if (codec->mach_write(codec->control_data, (long)data, 2) == 2)
		return 0;
	else
		return -EIO;
}

#define wm8510_reset(c)	wm8510_write(c, WM8510_RESET, 0)

static const char *wm8510_companding[] = {"Off", "NC", "u-law", "A-law" };
static const char *wm8510_deemp[] = {"None", "32kHz", "44.1kHz", "48kHz" };
static const char *wm8510_alc[] = {"ALC", "Limiter" };

static const struct soc_enum wm8510_enum[] = {
	SOC_ENUM_SINGLE(WM8510_COMP, 1, 4, wm8510_companding), /* adc */
	SOC_ENUM_SINGLE(WM8510_COMP, 3, 4, wm8510_companding), /* dac */
	SOC_ENUM_SINGLE(WM8510_DAC,  4, 4, wm8510_deemp),
	SOC_ENUM_SINGLE(WM8510_ALC3,  8, 2, wm8510_alc),
};

static const struct snd_kcontrol_new wm8510_snd_controls[] = {

SOC_SINGLE("Digital Loopback Switch", WM8510_COMP, 0, 1, 0),

SOC_ENUM("DAC Companding", wm8510_enum[1]),
SOC_ENUM("ADC Companding", wm8510_enum[0]),

SOC_ENUM("Playback De-emphasis", wm8510_enum[2]),
SOC_SINGLE("DAC Inversion Switch", WM8510_DAC, 0, 1, 0),

SOC_SINGLE("Master Playback Volume", WM8510_DACVOL, 0, 127, 0),

SOC_SINGLE("High Pass Filter Switch", WM8510_ADC, 8, 1, 0),
SOC_SINGLE("High Pass Cut Off", WM8510_ADC, 4, 7, 0),
SOC_SINGLE("ADC Inversion Switch", WM8510_COMP, 0, 1, 0),

SOC_SINGLE("Capture Volume", WM8510_ADCVOL,  0, 127, 0),

SOC_SINGLE("DAC Playback Limiter Switch", WM8510_DACLIM1,  8, 1, 0),
SOC_SINGLE("DAC Playback Limiter Decay", WM8510_DACLIM1,  4, 15, 0),
SOC_SINGLE("DAC Playback Limiter Attack", WM8510_DACLIM1,  0, 15, 0),

SOC_SINGLE("DAC Playback Limiter Threshold", WM8510_DACLIM2,  4, 7, 0),
SOC_SINGLE("DAC Playback Limiter Boost", WM8510_DACLIM2,  0, 15, 0),

SOC_SINGLE("ALC Enable Switch", WM8510_ALC1,  8, 1, 0),
SOC_SINGLE("ALC Capture Max Gain", WM8510_ALC1,  3, 7, 0),
SOC_SINGLE("ALC Capture Min Gain", WM8510_ALC1,  0, 7, 0),

SOC_SINGLE("ALC Capture ZC Switch", WM8510_ALC2,  8, 1, 0),
SOC_SINGLE("ALC Capture Hold", WM8510_ALC2,  4, 7, 0),
SOC_SINGLE("ALC Capture Target", WM8510_ALC2,  0, 15, 0),

SOC_ENUM("ALC Capture Mode", wm8510_enum[3]),
SOC_SINGLE("ALC Capture Decay", WM8510_ALC3,  4, 15, 0),
SOC_SINGLE("ALC Capture Attack", WM8510_ALC3,  0, 15, 0),

SOC_SINGLE("ALC Capture Noise Gate Switch", WM8510_NGATE,  3, 1, 0),
SOC_SINGLE("ALC Capture Noise Gate Threshold", WM8510_NGATE,  0, 7, 0),

SOC_SINGLE("Capture PGA ZC Switch", WM8510_INPPGA,  7, 1, 0),
SOC_SINGLE("Capture PGA Volume", WM8510_INPPGA,  0, 63, 0),

SOC_SINGLE("Speaker Playback ZC Switch", WM8510_SPKVOL,  7, 1, 0),
SOC_SINGLE("Speaker Playback Switch", WM8510_SPKVOL,  6, 1, 1),
SOC_SINGLE("Speaker Playback Volume", WM8510_SPKVOL,  0, 63, 0),
SOC_SINGLE("Speaker Boost", WM8510_OUTPUT, 2, 1, 0),

SOC_SINGLE("Capture Boost(+20dB)", WM8510_ADCBOOST,  8, 1, 0),
SOC_SINGLE("Mono Playback Switch", WM8510_MONOMIX, 6, 1, 0),
};

/* add non dapm controls */
static int wm8510_add_controls(struct snd_soc_codec *codec, 
	struct snd_card *card)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8510_snd_controls); i++) {
		err = snd_ctl_add(card,
				snd_soc_cnew(&wm8510_snd_controls[i],
					codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

/* Speaker Output Mixer */
static const struct snd_kcontrol_new wm8510_speaker_mixer_controls[] = {
SOC_DAPM_SINGLE("Line Bypass Switch", WM8510_SPKMIX, 1, 1, 0),
SOC_DAPM_SINGLE("Aux Playback Switch", WM8510_SPKMIX, 5, 1, 0),
SOC_DAPM_SINGLE("PCM Playback Switch", WM8510_SPKMIX, 0, 1, 1),
};

/* Mono Output Mixer */
static const struct snd_kcontrol_new wm8510_mono_mixer_controls[] = {
SOC_DAPM_SINGLE("Line Bypass Switch", WM8510_MONOMIX, 1, 1, 0),
SOC_DAPM_SINGLE("Aux Playback Switch", WM8510_MONOMIX, 2, 1, 0),
SOC_DAPM_SINGLE("PCM Playback Switch", WM8510_MONOMIX, 0, 1, 1),
};

/* AUX Input boost vol */
static const struct snd_kcontrol_new wm8510_aux_boost_controls =
SOC_DAPM_SINGLE("Aux Volume", WM8510_ADCBOOST, 0, 7, 0);

/* Mic Input boost vol */
static const struct snd_kcontrol_new wm8510_mic_boost_controls =
SOC_DAPM_SINGLE("Mic Volume", WM8510_ADCBOOST, 4, 7, 0);

/* Capture boost switch */
static const struct snd_kcontrol_new wm8510_capture_boost_controls =
SOC_DAPM_SINGLE("Capture Boost Switch", WM8510_INPPGA,  6, 1, 0);

/* Aux In to PGA */
static const struct snd_kcontrol_new wm8510_aux_capture_boost_controls =
SOC_DAPM_SINGLE("Aux Capture Boost Switch", WM8510_INPPGA,  2, 1, 0);

/* Mic P In to PGA */
static const struct snd_kcontrol_new wm8510_micp_capture_boost_controls =
SOC_DAPM_SINGLE("Mic P Capture Boost Switch", WM8510_INPPGA,  0, 1, 0);

/* Mic N In to PGA */
static const struct snd_kcontrol_new wm8510_micn_capture_boost_controls =
SOC_DAPM_SINGLE("Mic N Capture Boost Switch", WM8510_INPPGA,  1, 1, 0);

static const struct snd_soc_dapm_widget wm8510_dapm_widgets[] = {
SND_SOC_DAPM_MIXER("Speaker Mixer", WM8510_POWER3, 2, 0,
	&wm8510_speaker_mixer_controls[0],
	ARRAY_SIZE(wm8510_speaker_mixer_controls)),
SND_SOC_DAPM_MIXER("Mono Mixer", WM8510_POWER3, 3, 0,
	&wm8510_mono_mixer_controls[0],
	ARRAY_SIZE(wm8510_mono_mixer_controls)),
SND_SOC_DAPM_DAC("DAC", "HiFi Playback", WM8510_POWER3, 0, 0),
SND_SOC_DAPM_ADC("ADC", "HiFi Capture", WM8510_POWER3, 0, 0),
SND_SOC_DAPM_PGA("Aux Input", WM8510_POWER1, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("SpkN Out", WM8510_POWER3, 5, 0, NULL, 0),
SND_SOC_DAPM_PGA("SpkP Out", WM8510_POWER3, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mono Out", WM8510_POWER3, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mic PGA", WM8510_POWER2, 2, 0, NULL, 0),

SND_SOC_DAPM_PGA("Aux Boost", SND_SOC_NOPM, 0, 0,
	&wm8510_aux_boost_controls, 1),
SND_SOC_DAPM_PGA("Mic Boost", SND_SOC_NOPM, 0, 0,
	&wm8510_mic_boost_controls, 1),
SND_SOC_DAPM_SWITCH("Capture Boost", SND_SOC_NOPM, 0, 0,
	&wm8510_capture_boost_controls),

SND_SOC_DAPM_MIXER("Boost Mixer", WM8510_POWER2, 4, 0, NULL, 0),

SND_SOC_DAPM_MICBIAS("Mic Bias", WM8510_POWER1, 4, 0),

SND_SOC_DAPM_INPUT("MICN"),
SND_SOC_DAPM_INPUT("MICP"),
SND_SOC_DAPM_INPUT("AUX"),
SND_SOC_DAPM_OUTPUT("MONOOUT"),
SND_SOC_DAPM_OUTPUT("SPKOUTP"),
SND_SOC_DAPM_OUTPUT("SPKOUTN"),
};

static const char *audio_map[][3] = {
	/* Mono output mixer */
	{"Mono Mixer", "PCM Playback Switch", "DAC"},
	{"Mono Mixer", "Aux Playback Switch", "Aux Input"},
	{"Mono Mixer", "Line Bypass Switch", "Boost Mixer"},

	/* Speaker output mixer */
	{"Speaker Mixer", "PCM Playback Switch", "DAC"},
	{"Speaker Mixer", "Aux Playback Switch", "Aux Input"},
	{"Speaker Mixer", "Line Bypass Switch", "Boost Mixer"},

	/* Outputs */
	{"Mono Out", NULL, "Mono Mixer"},
	{"MONOOUT", NULL, "Mono Out"},
	{"SpkN Out", NULL, "Speaker Mixer"},
	{"SpkP Out", NULL, "Speaker Mixer"},
	{"SPKOUTN", NULL, "SpkN Out"},
	{"SPKOUTP", NULL, "SpkP Out"},

	/* Boost Mixer */
	{"Boost Mixer", NULL, "ADC"},
	{"Capture Boost Switch", "Aux Capture Boost Switch", "AUX"},
	{"Aux Boost", "Aux Volume", "Boost Mixer"},
	{"Capture Boost", "Capture Switch", "Boost Mixer"},
	{"Mic Boost", "Mic Volume", "Boost Mixer"},

	/* Inputs */
	{"MICP", NULL, "Mic Boost"},
	{"MICN", NULL, "Mic PGA"},
	{"Mic PGA", NULL, "Capture Boost"},
	{"AUX", NULL, "Aux Input"},

	/* terminator */
	{NULL, NULL, NULL},
};

static int wm8510_add_widgets(struct snd_soc_codec *codec, 
	struct snd_soc_machine *machine)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(wm8510_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec, 
			&wm8510_dapm_widgets[i]);
	}

	/* set up audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_new_widgets(machine);
	return 0;
}

struct pll_ {
	unsigned int pre_div:4; /* prescale - 1 */
	unsigned int n:4;
	unsigned int k;
};

static struct pll_ pll_div;

/* The size in bits of the pll divide multiplied by 10
 * to allow rounding later */
#define FIXED_PLL_SIZE ((1 << 24) * 10)

static void pll_factors(unsigned int target, unsigned int source)
{
	unsigned long long Kpart;
	unsigned int K, Ndiv, Nmod;

	Ndiv = target / source;
	if (Ndiv < 6) {
		source >>= 1;
		pll_div.pre_div = 1;
		Ndiv = target / source;
	} else
		pll_div.pre_div = 0;

	if ((Ndiv < 6) || (Ndiv > 12))
		printk(KERN_WARNING
			"WM8510 N value outwith recommended range! N = %d\n",Ndiv);

	pll_div.n = Ndiv;
	Nmod = target % source;
	Kpart = FIXED_PLL_SIZE * (long long)Nmod;

	do_div(Kpart, source);

	K = Kpart & 0xFFFFFFFF;

	/* Check if we need to round */
	if ((K % 10) >= 5)
		K += 5;

	/* Move down to proper range now rounding is done */
	K /= 10;

	pll_div.k = K;
}

static int wm8510_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	if(freq_in == 0 || freq_out == 0) {
		reg = wm8510_read_reg_cache(codec, WM8510_POWER1);
		wm8510_write(codec, WM8510_POWER1, reg & 0x1df);
		return 0;
	}

	pll_factors(freq_out * 8, freq_in);

	wm8510_write(codec, WM8510_PLLN, (pll_div.pre_div << 4) | pll_div.n);
	wm8510_write(codec, WM8510_PLLK1, pll_div.k >> 18);
	wm8510_write(codec, WM8510_PLLK1, (pll_div.k >> 9) && 0x1ff);
	wm8510_write(codec, WM8510_PLLK1, pll_div.k && 0x1ff);
	reg = wm8510_read_reg_cache(codec, WM8510_POWER1);
	wm8510_write(codec, WM8510_POWER1, reg | 0x020);
	return 0;

}

/*
 * Configure WM8510 clock dividers.
 */
static int wm8510_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	switch (div_id) {
	case WM8510_OPCLKDIV:
		reg = wm8510_read_reg_cache(codec, WM8510_GPIO & 0x1cf);
		wm8510_write(codec, WM8510_GPIO, reg | div);
		break;
	case WM8510_MCLKDIV:
		reg = wm8510_read_reg_cache(codec, WM8510_CLOCK & 0x1f);
		wm8510_write(codec, WM8510_CLOCK, reg | div);
		break;
	case WM8510_ADCCLK:
		reg = wm8510_read_reg_cache(codec, WM8510_ADC & 0x1f7);
		wm8510_write(codec, WM8510_ADC, reg | div);
		break;
	case WM8510_DACCLK:
		reg = wm8510_read_reg_cache(codec, WM8510_DAC & 0x1f7);
		wm8510_write(codec, WM8510_DAC, reg | div);
		break;
	case WM8510_BCLKDIV:
		reg = wm8510_read_reg_cache(codec, WM8510_CLOCK & 0x1e3);
		wm8510_write(codec, WM8510_CLOCK, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wm8510_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = 0;
	u16 clk = wm8510_read_reg_cache(codec, WM8510_CLOCK) & 0x1fe;

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		clk |= 0x0001;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		break;
	default:
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		iface |= 0x0010;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		iface |= 0x0008;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		iface |= 0x00018;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x0180;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x0100;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface |= 0x0080;
		break;
	default:
		return -EINVAL;
	}

	wm8510_write(codec, WM8510_IFACE, iface);
	wm8510_write(codec, WM8510_CLOCK, clk);
	return 0;
}

static int wm8510_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;
	u16 iface = wm8510_read_reg_cache(codec, WM8510_IFACE) & 0x19f;
	u16 adn = wm8510_read_reg_cache(codec, WM8510_ADD) & 0x1f1;

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		iface |= 0x0020;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		iface |= 0x0040;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		iface |= 0x0060;
		break;
	}

	/* filter coefficient */
	switch (params_rate(params)) {
	case SNDRV_PCM_RATE_8000:
		adn |= 0x5 << 1;
		break;
	case SNDRV_PCM_RATE_11025:
		adn |= 0x4 << 1;
		break;
	case SNDRV_PCM_RATE_16000:
		adn |= 0x3 << 1;
		break;
	case SNDRV_PCM_RATE_22050:
		adn |= 0x2 << 1;
		break;
	case SNDRV_PCM_RATE_32000:
		adn |= 0x1 << 1;
		break;
	case SNDRV_PCM_RATE_44100:
		break;
	}

	wm8510_write(codec, WM8510_IFACE, iface);
	wm8510_write(codec, WM8510_ADD, adn);
	return 0;
}

static int wm8510_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = wm8510_read_reg_cache(codec, WM8510_DAC) & 0xffbf;

	if(mute)
		wm8510_write(codec, WM8510_DAC, mute_reg | 0x40);
	else
		wm8510_write(codec, WM8510_DAC, mute_reg);
	return 0;
}

/* liam need to make this lower power with dapm */
static int wm8510_dapm_event(struct snd_soc_codec *codec, int event)
{

	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		/* vref/mid, clk and osc on, dac unmute, active */
		wm8510_write(codec, WM8510_POWER1, 0x1ff);
		wm8510_write(codec, WM8510_POWER2, 0x1ff);
		wm8510_write(codec, WM8510_POWER3, 0x1ff);
		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		/* everything off except vref/vmid, dac mute, inactive */

		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		/* everything off, dac mute, inactive */
		wm8510_write(codec, WM8510_POWER1, 0x0);
		wm8510_write(codec, WM8510_POWER2, 0x0);
		wm8510_write(codec, WM8510_POWER3, 0x0);
		break;
	}
	codec->dapm_state = event;
	return 0;
}

static int wm8510_suspend(struct device *dev, pm_message_t state)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	wm8510_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	return 0;
}

static int wm8510_resume(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8510_reg); i++) {
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->mach_write(codec->control_data, (long)data, 2);
	}
	wm8510_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	wm8510_dapm_event(codec, codec->suspend_dapm_state);
	return 0;
}


/*
 * initialise the WM8510 codec
 */
static int wm8510_codec_io_probe(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	wm8510_reset(codec);

	/* power on device */
	wm8510_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	
	wm8510_add_controls(codec, machine->card);
	wm8510_add_widgets(codec, machine);

	return 0;
}

static struct snd_soc_codec_ops wm8510_codec_ops = {
	.dapm_event	= wm8510_dapm_event,
	.read		= wm8510_read_reg_cache,
	.write		= wm8510_write,
	.io_probe	= wm8510_codec_io_probe,
};

static int wm8510_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	info("WM8510 Audio Codec %s", WM8510_VERSION);

	codec->reg_cache = kmemdup(wm8510_reg, sizeof(wm8510_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	codec->reg_cache_size = sizeof(wm8510_reg);
	
	codec->owner = THIS_MODULE;
	codec->ops = &wm8510_codec_ops;
	return 0;
}

static int wm8510_codec_remove(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	
	if (codec->control_data)
		wm8510_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	kfree(codec->reg_cache);
	return 0;
}

#define WM8510_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000)

#define WM8510_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE)

static const struct snd_soc_pcm_stream wm8510_dai_playback = {
	.stream_name	= "Playback",
	.channels_min	= 1,
	.channels_max	= 1,
	.rates		= WM8510_RATES,
	.formats	= WM8510_FORMATS,
};

static const struct snd_soc_pcm_stream wm8510_dai_capture = {
	.stream_name	= "Capture",
	.channels_min	= 1,
	.channels_max	= 1,
	.rates		= WM8510_RATES,
	.formats	= WM8510_FORMATS,
};

/* dai ops, called by machine drivers */
static const struct snd_soc_dai_ops wm8510_dai_ops = {
	.digital_mute	= wm8510_digital_mute,
	.set_clkdiv	= wm8510_set_dai_clkdiv,
	.set_fmt	= wm8510_set_dai_fmt,
	.set_pll	= wm8510_set_dai_pll,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops wm8510_dai_audio_ops = {
	.hw_params	= wm8510_hw_params,
};

static int wm8510_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);

	dai->ops = &wm8510_dai_ops;
	dai->audio_ops = &wm8510_dai_audio_ops;
	dai->capture = &wm8510_dai_capture;
	dai->playback = &wm8510_dai_playback;
	return 0;
}

const char wm8510_codec[SND_SOC_CODEC_NAME_SIZE] = "wm8510-codec";
EXPORT_SYMBOL_GPL(wm8510_codec);

static struct snd_soc_device_driver wm8510_codec_driver = {
	.type	= SND_SOC_BUS_TYPE_CODEC,
	.driver	= {
		.name 		= wm8510_codec,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8510_codec_probe,
		.remove		= __devexit_p(wm8510_codec_remove),
		.suspend	= wm8510_suspend,
		.resume		= wm8510_resume,
	},
};

const char wm8510_hifi_dai[SND_SOC_CODEC_NAME_SIZE] = "wm8510-hifi-dai";
EXPORT_SYMBOL_GPL(wm8510_hifi_dai);

static struct snd_soc_device_driver wm8510_hifi_dai_driver = {
	.type	= SND_SOC_BUS_TYPE_DAI,
	.driver	= {
		.name 		= wm8510_hifi_dai,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8510_dai_probe,
	},
};

static __init int wm8510_init(void)
{
	int ret = 0;
	
	ret = driver_register(&wm8510_codec_driver.driver);
	if (ret < 0)
		return ret;
	ret = driver_register(&wm8510_hifi_dai_driver.driver);
	if (ret < 0) {
		driver_unregister(&wm8510_codec_driver.driver);
		return ret;
	}
	return ret;
}

static __exit void wm8510_exit(void)
{
	driver_unregister(&wm8510_hifi_dai_driver.driver);
	driver_unregister(&wm8510_codec_driver.driver);
}

module_init(wm8510_init);
module_exit(wm8510_exit);

MODULE_DESCRIPTION("ASoC WM8510 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
