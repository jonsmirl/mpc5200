/*
 * wm8976.c  --  WM8976 ALSA Soc Audio driver
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
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

#include "wm8976.h"

#define AUDIO_NAME "wm8976"
#define WM8976_VERSION "0.20"

/*
 * Debug
 */

#define WM8976_DEBUG 0

#ifdef WM8976_DEBUG
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
 * wm8976 register cache
 * We can't read the WM8976 register space when we are
 * using 2 wire for device control, so we cache them instead.
 */
static const u16 wm8976_reg[WM8976_CACHEREGNUM] = {
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0050, 0x0000, 0x0140, 0x0000,
    0x0000, 0x0000, 0x0000, 0x00ff,
    0x00ff, 0x0000, 0x0100, 0x00ff,
    0x00ff, 0x0000, 0x012c, 0x002c,
    0x002c, 0x002c, 0x002c, 0x0000,
    0x0032, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0038, 0x000b, 0x0032, 0x0000,
    0x0008, 0x000c, 0x0093, 0x00e9,
    0x0000, 0x0000, 0x0000, 0x0000,
    0x0033, 0x0010, 0x0010, 0x0100,
    0x0100, 0x0002, 0x0001, 0x0001,
    0x0039, 0x0039, 0x0039, 0x0039,
    0x0001, 0x0001,
};

/*
 * read wm8976 register cache
 */
static inline unsigned int wm8976_read_reg_cache(struct snd_soc_codec  *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;
	if (reg == WM8976_RESET)
		return 0;
	if (reg >= WM8976_CACHEREGNUM)
		return -1;
	return cache[reg];
}

/*
 * write wm8976 register cache
 */
static inline void wm8976_write_reg_cache(struct snd_soc_codec  *codec,
	u16 reg, unsigned int value)
{
	u16 *cache = codec->reg_cache;
	if (reg >= WM8976_CACHEREGNUM)
		return;
	cache[reg] = value;
}

/*
 * write to the WM8976 register space
 */
static int wm8976_write(struct snd_soc_codec  *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	/* data is
	 *   D15..D9 WM8976 register offset
	 *   D8...D0 register data
	 */
	data[0] = (reg << 1) | ((value >> 8) & 0x0001);
	data[1] = value & 0x00ff;

	wm8976_write_reg_cache (codec, reg, value);
	if (codec->mach_write(codec->control_data, (long)data, 2) == 2)
		return 0;
	else
		return -EIO;
}

#define wm8976_reset(c)	wm8976_write(c, WM8976_RESET, 0)

static const char *wm8976_companding[] = {"Off", "NC", "u-law", "A-law" };
static const char *wm8976_deemp[] = {"None", "32kHz", "44.1kHz", "48kHz" };
static const char *wm8976_eqmode[] = {"Capture", "Playback" };
static const char *wm8976_bw[] = {"Narrow", "Wide" };
static const char *wm8976_eq1[] = {"80Hz", "105Hz", "135Hz", "175Hz" };
static const char *wm8976_eq2[] = {"230Hz", "300Hz", "385Hz", "500Hz" };
static const char *wm8976_eq3[] = {"650Hz", "850Hz", "1.1kHz", "1.4kHz" };
static const char *wm8976_eq4[] = {"1.8kHz", "2.4kHz", "3.2kHz", "4.1kHz" };
static const char *wm8976_eq5[] = {"5.3kHz", "6.9kHz", "9kHz", "11.7kHz" };
static const char *wm8976_alc[] =
    {"ALC both on", "ALC left only", "ALC right only", "Limiter" };

static const struct soc_enum wm8976_enum[] = {
	SOC_ENUM_SINGLE(WM8976_COMP, 1, 4, wm8976_companding), /* adc */
	SOC_ENUM_SINGLE(WM8976_COMP, 3, 4, wm8976_companding), /* dac */
	SOC_ENUM_SINGLE(WM8976_DAC,  4, 4, wm8976_deemp),
	SOC_ENUM_SINGLE(WM8976_EQ1,  8, 2, wm8976_eqmode),

	SOC_ENUM_SINGLE(WM8976_EQ1,  5, 4, wm8976_eq1),
	SOC_ENUM_SINGLE(WM8976_EQ2,  8, 2, wm8976_bw),
	SOC_ENUM_SINGLE(WM8976_EQ2,  5, 4, wm8976_eq2),
	SOC_ENUM_SINGLE(WM8976_EQ3,  8, 2, wm8976_bw),

	SOC_ENUM_SINGLE(WM8976_EQ3,  5, 4, wm8976_eq3),
	SOC_ENUM_SINGLE(WM8976_EQ4,  8, 2, wm8976_bw),
	SOC_ENUM_SINGLE(WM8976_EQ4,  5, 4, wm8976_eq4),
	SOC_ENUM_SINGLE(WM8976_EQ5,  8, 2, wm8976_bw),

	SOC_ENUM_SINGLE(WM8976_EQ5,  5, 4, wm8976_eq5),
	SOC_ENUM_SINGLE(WM8976_ALC3,  8, 2, wm8976_alc),
};

static const struct snd_kcontrol_new wm8976_snd_controls[] = {
SOC_SINGLE("Digital Loopback Switch", WM8976_COMP, 0, 1, 0),

SOC_ENUM("ADC Companding", wm8976_enum[0]),
SOC_ENUM("DAC Companding", wm8976_enum[1]),

SOC_SINGLE("Jack Detection Enable", WM8976_JACK1, 6, 1, 0),

SOC_DOUBLE("DAC Inversion Switch", WM8976_DAC, 0, 1, 1, 0),

SOC_DOUBLE_R("Headphone Playback Volume", WM8976_DACVOLL, WM8976_DACVOLR, 0, 127, 0),

SOC_SINGLE("High Pass Filter Switch", WM8976_ADC, 8, 1, 0),
SOC_SINGLE("High Pass Filter Switch", WM8976_ADC, 8, 1, 0),
SOC_SINGLE("High Pass Cut Off", WM8976_ADC, 4, 7, 0),

SOC_DOUBLE("ADC Inversion Switch", WM8976_ADC, 0, 1, 1, 0),

SOC_SINGLE("Capture Volume", WM8976_ADCVOL,  0, 127, 0),

SOC_ENUM("Equaliser Function", wm8976_enum[3]),
SOC_ENUM("EQ1 Cut Off", wm8976_enum[4]),
SOC_SINGLE("EQ1 Volume", WM8976_EQ1,  0, 31, 1),

SOC_ENUM("Equaliser EQ2 Bandwith", wm8976_enum[5]),
SOC_ENUM("EQ2 Cut Off", wm8976_enum[6]),
SOC_SINGLE("EQ2 Volume", WM8976_EQ2,  0, 31, 1),

SOC_ENUM("Equaliser EQ3 Bandwith", wm8976_enum[7]),
SOC_ENUM("EQ3 Cut Off", wm8976_enum[8]),
SOC_SINGLE("EQ3 Volume", WM8976_EQ3,  0, 31, 1),

SOC_ENUM("Equaliser EQ4 Bandwith", wm8976_enum[9]),
SOC_ENUM("EQ4 Cut Off", wm8976_enum[10]),
SOC_SINGLE("EQ4 Volume", WM8976_EQ4,  0, 31, 1),

SOC_ENUM("Equaliser EQ5 Bandwith", wm8976_enum[11]),
SOC_ENUM("EQ5 Cut Off", wm8976_enum[12]),
SOC_SINGLE("EQ5 Volume", WM8976_EQ5,  0, 31, 1),

SOC_SINGLE("DAC Playback Limiter Switch", WM8976_DACLIM1,  8, 1, 0),
SOC_SINGLE("DAC Playback Limiter Decay", WM8976_DACLIM1,  4, 15, 0),
SOC_SINGLE("DAC Playback Limiter Attack", WM8976_DACLIM1,  0, 15, 0),

SOC_SINGLE("DAC Playback Limiter Threshold", WM8976_DACLIM2,  4, 7, 0),
SOC_SINGLE("DAC Playback Limiter Boost", WM8976_DACLIM2,  0, 15, 0),

SOC_SINGLE("ALC Enable Switch", WM8976_ALC1,  8, 1, 0),
SOC_SINGLE("ALC Capture Max Gain", WM8976_ALC1,  3, 7, 0),
SOC_SINGLE("ALC Capture Min Gain", WM8976_ALC1,  0, 7, 0),

SOC_SINGLE("ALC Capture ZC Switch", WM8976_ALC2,  8, 1, 0),
SOC_SINGLE("ALC Capture Hold", WM8976_ALC2,  4, 7, 0),
SOC_SINGLE("ALC Capture Target", WM8976_ALC2,  0, 15, 0),

SOC_ENUM("ALC Capture Mode", wm8976_enum[13]),
SOC_SINGLE("ALC Capture Decay", WM8976_ALC3,  4, 15, 0),
SOC_SINGLE("ALC Capture Attack", WM8976_ALC3,  0, 15, 0),

SOC_SINGLE("ALC Capture Noise Gate Switch", WM8976_NGATE,  3, 1, 0),
SOC_SINGLE("ALC Capture Noise Gate Threshold", WM8976_NGATE,  0, 7, 0),

SOC_SINGLE("Capture PGA ZC Switch", WM8976_INPPGA,  7, 1, 0),
SOC_SINGLE("Capture PGA Volume", WM8976_INPPGA,  0, 63, 0),

SOC_DOUBLE_R("Headphone Playback ZC Switch", WM8976_HPVOLL,  WM8976_HPVOLR, 7, 1, 0),
SOC_DOUBLE_R("Headphone Playback Switch", WM8976_HPVOLL,  WM8976_HPVOLR, 6, 1, 1),
SOC_DOUBLE_R("Headphone Playback Volume", WM8976_HPVOLL,  WM8976_HPVOLR, 0, 63, 0),

SOC_DOUBLE_R("Speaker Playback ZC Switch", WM8976_SPKVOLL,  WM8976_SPKVOLR, 7, 1, 0),
SOC_DOUBLE_R("Speaker Playback Switch", WM8976_SPKVOLL,  WM8976_SPKVOLR, 6, 1, 1),
SOC_DOUBLE_R("Speaker Playback Volume", WM8976_SPKVOLL,  WM8976_SPKVOLR, 0, 63, 0),

SOC_SINGLE("Capture Boost(+20dB)", WM8976_ADCBOOST, 8, 1, 0),
};

/* add non dapm controls */
static int wm8976_add_controls(struct snd_soc_codec *codec, 
	struct snd_card *card)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm8976_snd_controls); i++) {
		err = snd_ctl_add(card,
				snd_soc_cnew(&wm8976_snd_controls[i],
					codec, NULL));
		if (err < 0)
			return err;
	}

	return 0;
}

/* Left Output Mixer */
static const struct snd_kcontrol_new wm8976_left_mixer_controls[] = {
SOC_DAPM_SINGLE("Right PCM Playback Switch", WM8976_OUTPUT, 6, 1, 1),
SOC_DAPM_SINGLE("Left PCM Playback Switch", WM8976_MIXL, 0, 1, 1),
SOC_DAPM_SINGLE("Line Bypass Switch", WM8976_MIXL, 1, 1, 0),
SOC_DAPM_SINGLE("Aux Playback Switch", WM8976_MIXL, 5, 1, 0),
};

/* Right Output Mixer */
static const struct snd_kcontrol_new wm8976_right_mixer_controls[] = {
SOC_DAPM_SINGLE("Left PCM Playback Switch", WM8976_OUTPUT, 5, 1, 1),
SOC_DAPM_SINGLE("Right PCM Playback Switch", WM8976_MIXR, 0, 1, 1),
SOC_DAPM_SINGLE("Line Bypass Switch", WM8976_MIXR, 1, 1, 0),
SOC_DAPM_SINGLE("Aux Playback Switch", WM8976_MIXR, 5, 1, 0),
};

/* Left AUX Input boost vol */
static const struct snd_kcontrol_new wm8976_laux_boost_controls =
SOC_DAPM_SINGLE("Aux Volume", WM8976_ADCBOOST, 0, 3, 0);

/* Left Input boost vol */
static const struct snd_kcontrol_new wm8976_lmic_boost_controls =
SOC_DAPM_SINGLE("Input Volume", WM8976_ADCBOOST, 4, 3, 0);

/* Left Aux In to PGA */
static const struct snd_kcontrol_new wm8976_laux_capture_boost_controls =
SOC_DAPM_SINGLE("Capture Switch", WM8976_ADCBOOST,  8, 1, 0);

/* Left Input P In to PGA */
static const struct snd_kcontrol_new wm8976_lmicp_capture_boost_controls =
SOC_DAPM_SINGLE("Input P Capture Boost Switch", WM8976_INPUT,  0, 1, 0);

/* Left Input N In to PGA */
static const struct snd_kcontrol_new wm8976_lmicn_capture_boost_controls =
SOC_DAPM_SINGLE("Input N Capture Boost Switch", WM8976_INPUT,  1, 1, 0);

// TODO Widgets
static const struct snd_soc_dapm_widget wm8976_dapm_widgets[] = {
#if 0
//SND_SOC_DAPM_MUTE("Mono Mute", WM8976_MONOMIX, 6, 0),
//SND_SOC_DAPM_MUTE("Speaker Mute", WM8976_SPKMIX, 6, 0),

SND_SOC_DAPM_MIXER("Speaker Mixer", WM8976_POWER3, 2, 0,
	&wm8976_speaker_mixer_controls[0],
	ARRAY_SIZE(wm8976_speaker_mixer_controls)),
SND_SOC_DAPM_MIXER("Mono Mixer", WM8976_POWER3, 3, 0,
	&wm8976_mono_mixer_controls[0],
	ARRAY_SIZE(wm8976_mono_mixer_controls)),
SND_SOC_DAPM_DAC("DAC", "HiFi Playback", WM8976_POWER3, 0, 0),
SND_SOC_DAPM_ADC("ADC", "HiFi Capture", WM8976_POWER3, 0, 0),
SND_SOC_DAPM_PGA("Aux Input", WM8976_POWER1, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("SpkN Out", WM8976_POWER3, 5, 0, NULL, 0),
SND_SOC_DAPM_PGA("SpkP Out", WM8976_POWER3, 6, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mono Out", WM8976_POWER3, 7, 0, NULL, 0),
SND_SOC_DAPM_PGA("Mic PGA", WM8976_POWER2, 2, 0, NULL, 0),

SND_SOC_DAPM_PGA("Aux Boost", SND_SOC_NOPM, 0, 0,
	&wm8976_aux_boost_controls, 1),
SND_SOC_DAPM_PGA("Mic Boost", SND_SOC_NOPM, 0, 0,
	&wm8976_mic_boost_controls, 1),
SND_SOC_DAPM_SWITCH("Capture Boost", SND_SOC_NOPM, 0, 0,
	&wm8976_capture_boost_controls),

SND_SOC_DAPM_MIXER("Boost Mixer", WM8976_POWER2, 4, 0, NULL, 0),

SND_SOC_DAPM_MICBIAS("Mic Bias", WM8976_POWER1, 4, 0),

SND_SOC_DAPM_INPUT("MICN"),
SND_SOC_DAPM_INPUT("MICP"),
SND_SOC_DAPM_INPUT("AUX"),
SND_SOC_DAPM_OUTPUT("MONOOUT"),
SND_SOC_DAPM_OUTPUT("SPKOUTP"),
SND_SOC_DAPM_OUTPUT("SPKOUTN"),
#endif
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

    /*  */

	/* terminator */
	{NULL, NULL, NULL},
};

static int wm8976_add_widgets(struct snd_soc_codec *codec, 
	struct snd_soc_machine *machine)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(wm8976_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec, 
			&wm8976_dapm_widgets[i]);
	}

	/* set up audio path map */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, audio_map[i][0], 
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_new_widgets(machine);
	return 0;
}

struct _pll_div {
	unsigned int pre:4; /* prescale - 1 */
	unsigned int n:4;
	unsigned int k;
};

static struct _pll_div pll_div;

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
		pll_div.pre = 1;
		Ndiv = target / source;
	} else
		pll_div.pre = 0;

	if ((Ndiv < 6) || (Ndiv > 12))
		printk(KERN_WARNING
			"WM8976 N value outwith recommended range! N = %d\n",Ndiv);

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

static int wm8976_set_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	if (freq_in == 0 || freq_out == 0) {
		reg = wm8976_read_reg_cache(codec, WM8976_POWER1);
		wm8976_write(codec, WM8976_POWER1, reg & 0x1df);
		return 0;
	}

	pll_factors(freq_out * 8, freq_in);

	wm8976_write(codec, WM8976_PLLN, (pll_div.pre << 4) | pll_div.n);
	wm8976_write(codec, WM8976_PLLK1, pll_div.k >> 18);
	wm8976_write(codec, WM8976_PLLK1, (pll_div.k >> 9) && 0x1ff);
	wm8976_write(codec, WM8976_PLLK1, pll_div.k && 0x1ff);
	reg = wm8976_read_reg_cache(codec, WM8976_POWER1);
	wm8976_write(codec, WM8976_POWER1, reg | 0x020);
	return 0;
}

static int wm8976_set_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 iface = wm8976_read_reg_cache(codec, WM8976_IFACE) & 0x3;
	u16 clk = wm8976_read_reg_cache(codec, WM8976_CLOCK) & 0xfffe;

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

	wm8976_write(codec, WM8976_IFACE, iface);
	wm8976_write(codec, WM8976_CLOCK, clk);

	return 0;
}

static int wm8976_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;
	u16 iface = wm8976_read_reg_cache(codec, WM8976_IFACE) & 0xff9f;
	u16 adn = wm8976_read_reg_cache(codec, WM8976_ADD) & 0x1f1;

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
	}

	/* set iface */
	wm8976_write(codec, WM8976_IFACE, iface);
	wm8976_write(codec, WM8976_ADD, adn);
	return 0;
}

static int wm8976_set_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	switch (div_id) {
	case WM8976_MCLKDIV:
		reg = wm8976_read_reg_cache(codec, WM8976_CLOCK) & 0x11f;
		wm8976_write(codec, WM8976_CLOCK, reg | div);
		break;
	case WM8976_BCLKDIV:
		reg = wm8976_read_reg_cache(codec, WM8976_CLOCK) & 0x1c7;
		wm8976_write(codec, WM8976_CLOCK, reg | div);
		break;
	case WM8976_OPCLKDIV:
		reg = wm8976_read_reg_cache(codec, WM8976_GPIO) & 0x1cf;
		wm8976_write(codec, WM8976_GPIO, reg | div);
		break;
	case WM8976_DACOSR:
		reg = wm8976_read_reg_cache(codec, WM8976_DAC) & 0x1f7;
		wm8976_write(codec, WM8976_DAC, reg | div);
		break;
	case WM8976_ADCOSR:
		reg = wm8976_read_reg_cache(codec, WM8976_ADC) & 0x1f7;
		wm8976_write(codec, WM8976_ADC, reg | div);
		break;
	case WM8976_MCLKSEL:
		reg = wm8976_read_reg_cache(codec, WM8976_CLOCK) & 0x0ff;
		wm8976_write(codec, WM8976_CLOCK, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int wm8976_digital_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	u16 mute_reg = wm8976_read_reg_cache(codec, WM8976_DAC) & 0xffbf;

	if(mute)
		wm8976_write(codec, WM8976_DAC, mute_reg | 0x40);
	else
		wm8976_write(codec, WM8976_DAC, mute_reg);

	return 0;
}

/* TODO: liam need to make this lower power with dapm */
static int wm8976_dapm_event(struct snd_soc_codec *codec, int event)
{

	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		/* vref/mid, clk and osc on, dac unmute, active */
		wm8976_write(codec, WM8976_POWER1, 0x1ff);
		wm8976_write(codec, WM8976_POWER2, 0x1ff);
		wm8976_write(codec, WM8976_POWER3, 0x1ff);
		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		/* everything off except vref/vmid, dac mute, inactive */

		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		/* everything off, dac mute, inactive */
		wm8976_write(codec, WM8976_POWER1, 0x0);
		wm8976_write(codec, WM8976_POWER2, 0x0);
		wm8976_write(codec, WM8976_POWER3, 0x0);
		break;
	}
	codec->dapm_state = event;
	return 0;
}

static int wm8976_suspend(struct device *dev, pm_message_t state)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	wm8976_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	return 0;
}

static int wm8976_resume(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	int i;
	u8 data[2];
	u16 *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(wm8976_reg); i++) {
		data[0] = (i << 1) | ((cache[i] >> 8) & 0x0001);
		data[1] = cache[i] & 0x00ff;
		codec->mach_write(codec->control_data, (long)data, 2);
	}
	wm8976_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	wm8976_dapm_event(codec, codec->suspend_dapm_state);
	return 0;
}

/*
 * initialise the WM8976 codec
 */
static int wm8976_codec_io_probe(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	wm8976_reset(codec);

	/* power on device */
	wm8976_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	
	wm8976_add_controls(codec, machine->card);
	wm8976_add_widgets(codec, machine);

	return 0;
}

static struct snd_soc_codec_ops wm8976_codec_ops = {
	.dapm_event	= wm8976_dapm_event,
	.read		= wm8976_read_reg_cache,
	.write		= wm8976_write,
	.io_probe	= wm8976_codec_io_probe,
};

static int wm8976_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	info("WM8976 Audio Codec %s", WM8976_VERSION);

	codec->reg_cache = kmemdup(wm8976_reg, sizeof(wm8976_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	codec->reg_cache_size = sizeof(wm8976_reg);
	
	codec->owner = THIS_MODULE;
	codec->ops = &wm8976_codec_ops;
	return 0;
}

static int wm8976_codec_remove(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	
	if (codec->control_data)
		wm8976_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	kfree(codec->reg_cache);
	return 0;
}

#define WM8976_RATES \
	(SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 | \
	SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 | \
	SNDRV_PCM_RATE_48000)

#define WM8976_FORMATS \
	(SNDRV_PCM_FORMAT_S16_LE | SNDRV_PCM_FORMAT_S20_3LE | \
	SNDRV_PCM_FORMAT_S24_3LE | SNDRV_PCM_FORMAT_S24_LE)

static const struct snd_soc_pcm_stream wm8976_dai_playback = {
	.stream_name	= "Playback",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM8976_RATES,
	.formats	= WM8976_FORMATS,
};

static const struct snd_soc_pcm_stream wm8976_dai_capture = {
	.stream_name	= "Capture",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM8976_RATES,
	.formats	= WM8976_FORMATS,
};

/* dai ops, called by machine drivers */
static const struct snd_soc_dai_ops wm8976_dai_ops = {
	.digital_mute = wm8976_digital_mute,
	.set_fmt = wm8976_set_fmt,
	.set_clkdiv = wm8976_set_clkdiv,
	.set_pll = wm8976_set_pll,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops wm8976_dai_audio_ops = {
	.hw_params	= wm8976_hw_params,
};

static int wm8976_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);

	dai->ops = &wm8976_dai_ops;
	dai->audio_ops = &wm8976_dai_audio_ops;
	dai->capture = &wm8976_dai_capture;
	dai->playback = &wm8976_dai_playback;
	return 0;
}

const char wm8976_codec[SND_SOC_CODEC_NAME_SIZE] = "wm8976-codec";
EXPORT_SYMBOL_GPL(wm8976_codec);

static struct snd_soc_device_driver wm8976_codec_driver = {
	.type	= SND_SOC_BUS_TYPE_CODEC,
	.driver	= {
		.name 		= wm8976_codec,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8976_codec_probe,
		.remove		= __devexit_p(wm8976_codec_remove),
		.suspend	= wm8976_suspend,
		.resume		= wm8976_resume,
	},
};

const char wm8976_hifi_dai[SND_SOC_CODEC_NAME_SIZE] = "wm8976-hifi-dai";
EXPORT_SYMBOL_GPL(wm8976_hifi_dai);

static struct snd_soc_device_driver wm8976_hifi_dai_driver = {
	.type	= SND_SOC_BUS_TYPE_DAI,
	.driver	= {
		.name 		= wm8976_hifi_dai,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm8976_dai_probe,
	},
};

static __init int wm8976_init(void)
{
	int ret = 0;
	
	ret = driver_register(&wm8976_codec_driver.driver);
	if (ret < 0)
		return ret;
	ret = driver_register(&wm8976_hifi_dai_driver.driver);
	if (ret < 0) {
		driver_unregister(&wm8976_codec_driver.driver);
		return ret;
	}
	return ret;
}

static __exit void wm8976_exit(void)
{
	driver_unregister(&wm8976_hifi_dai_driver.driver);
	driver_unregister(&wm8976_codec_driver.driver);
}

module_init(wm8976_init);
module_exit(wm8976_exit);


MODULE_DESCRIPTION("ASoC WM8976 driver");
MODULE_AUTHOR("Graeme Gregory");
MODULE_LICENSE("GPL");
