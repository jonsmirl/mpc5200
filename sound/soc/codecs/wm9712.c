/*
 * wm9712.c  --  ALSA Soc WM9712 codec support
 *
 * Copyright 2006 Wolfson Microelectronics PLC.
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    4th Feb 2006   Initial version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include "wm9712.h"

#define WM9712_VERSION "0.21"

struct wm9712_data {
	struct snd_soc_dai *hifi_dai;
	struct snd_soc_dai *aux_dai;
};

static unsigned int wm9712_ac97_read(struct snd_soc_codec *codec,
	unsigned int reg);
static int wm9712_ac97_write(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int val);

/*
 * WM9712 register cache
 */
static const u16 wm9712_reg[] = {
	0x6174, 0x8000, 0x8000, 0x8000, /*  6 */
	0x0f0f, 0xaaa0, 0xc008, 0x6808, /*  e */
	0xe808, 0xaaa0, 0xad00, 0x8000, /* 16 */
	0xe808, 0x3000, 0x8000, 0x0000, /* 1e */
	0x0000, 0x0000, 0x0000, 0x000f, /* 26 */
	0x0405, 0x0410, 0xbb80, 0xbb80, /* 2e */
	0x0000, 0xbb80, 0x0000, 0x0000, /* 36 */
	0x0000, 0x2000, 0x0000, 0x0000, /* 3e */
	0x0000, 0x0000, 0x0000, 0x0000, /* 46 */
	0x0000, 0x0000, 0xf83e, 0xffff, /* 4e */
	0x0000, 0x0000, 0x0000, 0xf83e, /* 56 */
	0x0008, 0x0000, 0x0000, 0x0000, /* 5e */
	0xb032, 0x3e00, 0x0000, 0x0000, /* 66 */
	0x0000, 0x0000, 0x0000, 0x0000, /* 6e */
	0x0000, 0x0000, 0x0000, 0x0006, /* 76 */
	0x0001, 0x0000, 0x574d, 0x4c12, /* 7e */
	0x0000, 0x0000 /* virtual hp mixers */
};

/* virtual HP mixers regs */
#define HPL_MIXER	0x80
#define HPR_MIXER	0x82

static const char *wm9712_alc_select[] = {"None", "Left", "Right", "Stereo"};
static const char *wm9712_alc_mux[] = {"Stereo", "Left", "Right", "None"};
static const char *wm9712_out3_src[] = {"Left", "VREF", "Left + Right",
	"Mono"};
static const char *wm9712_spk_src[] = {"Speaker Mix", "Headphone Mix"};
static const char *wm9712_rec_adc[] = {"Stereo", "Left", "Right", "Mute"};
static const char *wm9712_base[] = {"Linear Control", "Adaptive Boost"};
static const char *wm9712_rec_gain[] = {"+1.5dB Steps", "+0.75dB Steps"};
static const char *wm9712_mic[] = {"Mic 1", "Differential", "Mic 2",
	"Stereo"};
static const char *wm9712_rec_sel[] = {"Mic", "NC", "NC", "Speaker Mixer",
	"Line", "Headphone Mixer", "Phone Mixer", "Phone"};
static const char *wm9712_ng_type[] = {"Constant Gain", "Mute"};
static const char *wm9712_diff_sel[] = {"Mic", "Line"};

static const struct soc_enum wm9712_enum[] = {
SOC_ENUM_SINGLE(AC97_PCI_SVID, 14, 4, wm9712_alc_select),
SOC_ENUM_SINGLE(AC97_VIDEO, 12, 4, wm9712_alc_mux),
SOC_ENUM_SINGLE(AC97_AUX, 9, 4, wm9712_out3_src),
SOC_ENUM_SINGLE(AC97_AUX, 8, 2, wm9712_spk_src),
SOC_ENUM_SINGLE(AC97_REC_SEL, 12, 4, wm9712_rec_adc),
SOC_ENUM_SINGLE(AC97_MASTER_TONE, 15, 2, wm9712_base),
SOC_ENUM_DOUBLE(AC97_REC_GAIN, 14, 6, 2, wm9712_rec_gain),
SOC_ENUM_SINGLE(AC97_MIC, 5, 4, wm9712_mic),
SOC_ENUM_SINGLE(AC97_REC_SEL, 8, 8, wm9712_rec_sel),
SOC_ENUM_SINGLE(AC97_REC_SEL, 0, 8, wm9712_rec_sel),
SOC_ENUM_SINGLE(AC97_PCI_SVID, 5, 2, wm9712_ng_type),
SOC_ENUM_SINGLE(0x5c, 8, 2, wm9712_diff_sel),
};

static const struct snd_kcontrol_new wm9712_snd_ac97_controls[] = {
SOC_DOUBLE("Speaker Playback Volume", AC97_MASTER, 8, 0, 31, 1),
SOC_SINGLE("Speaker Playback Switch", AC97_MASTER, 15, 1, 1),
SOC_DOUBLE("Headphone Playback Volume", AC97_HEADPHONE, 8, 0, 31, 1),
SOC_SINGLE("Headphone Playback Switch", AC97_HEADPHONE, 15, 1, 1),
SOC_DOUBLE("PCM Playback Volume", AC97_PCM, 8, 0, 31, 1),

SOC_SINGLE("Speaker Playback ZC Switch", AC97_MASTER, 7, 1, 0),
SOC_SINGLE("Speaker Playback Invert Switch", AC97_MASTER, 6, 1, 0),
SOC_SINGLE("Headphone Playback ZC Switch", AC97_HEADPHONE, 7, 1, 0),
SOC_SINGLE("Mono Playback ZC Switch", AC97_MASTER_MONO, 7, 1, 0),
SOC_SINGLE("Mono Playback Volume", AC97_MASTER_MONO, 0, 31, 1),
SOC_SINGLE("Mono Playback Switch", AC97_MASTER_MONO, 15, 1, 1),

SOC_SINGLE("ALC Target Volume", AC97_CODEC_CLASS_REV, 12, 15, 0),
SOC_SINGLE("ALC Hold Time", AC97_CODEC_CLASS_REV, 8, 15, 0),
SOC_SINGLE("ALC Decay Time", AC97_CODEC_CLASS_REV, 4, 15, 0),
SOC_SINGLE("ALC Attack Time", AC97_CODEC_CLASS_REV, 0, 15, 0),
SOC_ENUM("ALC Function", wm9712_enum[0]),
SOC_SINGLE("ALC Max Volume", AC97_PCI_SVID, 11, 7, 0),
SOC_SINGLE("ALC ZC Timeout", AC97_PCI_SVID, 9, 3, 1),
SOC_SINGLE("ALC ZC Switch", AC97_PCI_SVID, 8, 1, 0),
SOC_SINGLE("ALC NG Switch", AC97_PCI_SVID, 7, 1, 0),
SOC_ENUM("ALC NG Type", wm9712_enum[10]),
SOC_SINGLE("ALC NG Threshold", AC97_PCI_SVID, 0, 31, 1),

SOC_SINGLE("Mic Headphone  Volume", AC97_VIDEO, 12, 7, 1),
SOC_SINGLE("ALC Headphone Volume", AC97_VIDEO, 7, 7, 1),

SOC_SINGLE("Out3 Switch", AC97_AUX, 15, 1, 1),
SOC_SINGLE("Out3 ZC Switch", AC97_AUX, 7, 1, 1),
SOC_SINGLE("Out3 Volume", AC97_AUX, 0, 31, 1),

SOC_SINGLE("PCBeep Bypass Headphone Volume", AC97_PC_BEEP, 12, 7, 1),
SOC_SINGLE("PCBeep Bypass Speaker Volume", AC97_PC_BEEP, 8, 7, 1),
SOC_SINGLE("PCBeep Bypass Phone Volume", AC97_PC_BEEP, 4, 7, 1),

SOC_SINGLE("Aux Playback Headphone Volume", AC97_CD, 12, 7, 1),
SOC_SINGLE("Aux Playback Speaker Volume", AC97_CD, 8, 7, 1),
SOC_SINGLE("Aux Playback Phone Volume", AC97_CD, 4, 7, 1),

SOC_SINGLE("Phone Volume", AC97_PHONE, 0, 15, 1),
SOC_DOUBLE("Line Capture Volume", AC97_LINE, 8, 0, 31, 1),

SOC_SINGLE("Capture 20dB Boost Switch", AC97_REC_SEL, 14, 1, 0),
SOC_SINGLE("Capture to Phone 20dB Boost Switch", AC97_REC_SEL, 11, 1, 1),

SOC_SINGLE("3D Upper Cut-off Switch", AC97_3D_CONTROL, 5, 1, 1),
SOC_SINGLE("3D Lower Cut-off Switch", AC97_3D_CONTROL, 4, 1, 1),
SOC_SINGLE("3D Playback Volume", AC97_3D_CONTROL, 0, 15, 0),

SOC_ENUM("Bass Control", wm9712_enum[5]),
SOC_SINGLE("Bass Cut-off Switch", AC97_MASTER_TONE, 12, 1, 1),
SOC_SINGLE("Tone Cut-off Switch", AC97_MASTER_TONE, 4, 1, 1),
SOC_SINGLE("Playback Attenuate (-6dB) Switch", AC97_MASTER_TONE, 6, 1, 0),
SOC_SINGLE("Bass Volume", AC97_MASTER_TONE, 8, 15, 1),
SOC_SINGLE("Treble Volume", AC97_MASTER_TONE, 0, 15, 1),

SOC_SINGLE("Capture ADC Switch", AC97_REC_GAIN, 15, 1, 1),
SOC_ENUM("Capture Volume Steps", wm9712_enum[6]),
SOC_DOUBLE("Capture Volume", AC97_REC_GAIN, 8, 0, 63, 1),
SOC_SINGLE("Capture ZC Switch", AC97_REC_GAIN, 7, 1, 0),

SOC_SINGLE("Mic 1 Volume", AC97_MIC, 8, 31, 1),
SOC_SINGLE("Mic 2 Volume", AC97_MIC, 0, 31, 1),
SOC_SINGLE("Mic 20dB Boost Switch", AC97_MIC, 7, 1, 0),
};

/* We have to create a fake left and right HP mixers because
 * the codec only has a single control that is shared by both channels.
 * This makes it impossible to determine the audio path.
 */
static int mixer_event (struct snd_soc_dapm_widget *w,
	struct snd_kcontrol *k, int event)
{
	u16 l, r, beep, line, phone, mic, pcm, aux;

	l = wm9712_ac97_read(w->codec, HPL_MIXER);
	r = wm9712_ac97_read(w->codec, HPR_MIXER);
	beep = wm9712_ac97_read(w->codec, AC97_PC_BEEP);
	mic = wm9712_ac97_read(w->codec, AC97_VIDEO);
	phone = wm9712_ac97_read(w->codec, AC97_PHONE);
	line = wm9712_ac97_read(w->codec, AC97_LINE);
	pcm = wm9712_ac97_read(w->codec, AC97_PCM);
	aux = wm9712_ac97_read(w->codec, AC97_CD);

	if (l & 0x1 || r & 0x1)
		wm9712_ac97_write(w->codec, AC97_VIDEO, mic & 0x7fff);
	else
		wm9712_ac97_write(w->codec, AC97_VIDEO, mic | 0x8000);

	if (l & 0x2 || r & 0x2)
		wm9712_ac97_write(w->codec, AC97_PCM, pcm & 0x7fff);
	else
		wm9712_ac97_write(w->codec, AC97_PCM, pcm | 0x8000);

	if (l & 0x4 || r & 0x4)
		wm9712_ac97_write(w->codec, AC97_LINE, line & 0x7fff);
	else
		wm9712_ac97_write(w->codec, AC97_LINE, line | 0x8000);

	if (l & 0x8 || r & 0x8)
		wm9712_ac97_write(w->codec, AC97_PHONE, phone & 0x7fff);
	else
		wm9712_ac97_write(w->codec, AC97_PHONE, phone | 0x8000);

	if (l & 0x10 || r & 0x10)
		wm9712_ac97_write(w->codec, AC97_CD, aux & 0x7fff);
	else
		wm9712_ac97_write(w->codec, AC97_CD, aux | 0x8000);

	if (l & 0x20 || r & 0x20)
		wm9712_ac97_write(w->codec, AC97_PC_BEEP, beep & 0x7fff);
	else
		wm9712_ac97_write(w->codec, AC97_PC_BEEP, beep | 0x8000);

	return 0;
}

/* Left Headphone Mixers */
static const struct snd_kcontrol_new wm9712_hpl_mixer_controls[] = {
	SOC_DAPM_SINGLE("PCBeep Bypass Switch", HPL_MIXER, 5, 1, 0),
	SOC_DAPM_SINGLE("Aux Playback Switch", HPL_MIXER, 4, 1, 0),
	SOC_DAPM_SINGLE("Phone Bypass Switch", HPL_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("Line Bypass Switch", HPL_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("PCM Playback Switch", HPL_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("Mic Sidetone Switch", HPL_MIXER, 0, 1, 0),
};

/* Right Headphone Mixers */
static const struct snd_kcontrol_new wm9712_hpr_mixer_controls[] = {
	SOC_DAPM_SINGLE("PCBeep Bypass Switch", HPR_MIXER, 5, 1, 0),
	SOC_DAPM_SINGLE("Aux Playback Switch", HPR_MIXER, 4, 1, 0),
	SOC_DAPM_SINGLE("Phone Bypass Switch", HPR_MIXER, 3, 1, 0),
	SOC_DAPM_SINGLE("Line Bypass Switch", HPR_MIXER, 2, 1, 0),
	SOC_DAPM_SINGLE("PCM Playback Switch", HPR_MIXER, 1, 1, 0),
	SOC_DAPM_SINGLE("Mic Sidetone Switch", HPR_MIXER, 0, 1, 0),
};

/* Speaker Mixer */
static const struct snd_kcontrol_new wm9712_speaker_mixer_controls[] = {
	SOC_DAPM_SINGLE("PCBeep Bypass Switch", AC97_PC_BEEP, 11, 1, 1),
	SOC_DAPM_SINGLE("Aux Playback Switch", AC97_CD, 11, 1, 1),
	SOC_DAPM_SINGLE("Phone Bypass Switch", AC97_PHONE, 14, 1, 1),
	SOC_DAPM_SINGLE("Line Bypass Switch", AC97_LINE, 14, 1, 1),
	SOC_DAPM_SINGLE("PCM Playback Switch", AC97_PCM, 14, 1, 1),
};

/* Phone Mixer */
static const struct snd_kcontrol_new wm9712_phone_mixer_controls[] = {
	SOC_DAPM_SINGLE("PCBeep Bypass Switch", AC97_PC_BEEP, 7, 1, 1),
	SOC_DAPM_SINGLE("Aux Playback Switch", AC97_CD, 7, 1, 1),
	SOC_DAPM_SINGLE("Line Bypass Switch", AC97_LINE, 13, 1, 1),
	SOC_DAPM_SINGLE("PCM Playback Switch", AC97_PCM, 13, 1, 1),
	SOC_DAPM_SINGLE("Mic 1 Sidetone Switch", AC97_MIC, 14, 1, 1),
	SOC_DAPM_SINGLE("Mic 2 Sidetone Switch", AC97_MIC, 13, 1, 1),
};

/* ALC headphone mux */
static const struct snd_kcontrol_new wm9712_alc_mux_controls =
SOC_DAPM_ENUM("Route", wm9712_enum[1]);

/* out 3 mux */
static const struct snd_kcontrol_new wm9712_out3_mux_controls =
SOC_DAPM_ENUM("Route", wm9712_enum[2]);

/* spk mux */
static const struct snd_kcontrol_new wm9712_spk_mux_controls =
SOC_DAPM_ENUM("Route", wm9712_enum[3]);

/* Capture to Phone mux */
static const struct snd_kcontrol_new wm9712_capture_phone_mux_controls =
SOC_DAPM_ENUM("Route", wm9712_enum[4]);

/* Capture left select */
static const struct snd_kcontrol_new wm9712_capture_selectl_controls =
SOC_DAPM_ENUM("Route", wm9712_enum[8]);

/* Capture right select */
static const struct snd_kcontrol_new wm9712_capture_selectr_controls =
SOC_DAPM_ENUM("Route", wm9712_enum[9]);

/* Mic select */
static const struct snd_kcontrol_new wm9712_mic_src_controls =
SOC_DAPM_ENUM("Route", wm9712_enum[7]);

/* diff select */
static const struct snd_kcontrol_new wm9712_diff_sel_controls =
SOC_DAPM_ENUM("Route", wm9712_enum[11]);

static const struct snd_soc_dapm_widget wm9712_dapm_widgets[] = {
SND_SOC_DAPM_MUX("ALC Sidetone Mux", SND_SOC_NOPM, 0, 0,
	&wm9712_alc_mux_controls),
SND_SOC_DAPM_MUX("Out3 Mux", SND_SOC_NOPM, 0, 0,
	&wm9712_out3_mux_controls),
SND_SOC_DAPM_MUX("Speaker Mux", SND_SOC_NOPM, 0, 0,
	&wm9712_spk_mux_controls),
SND_SOC_DAPM_MUX("Capture Phone Mux", SND_SOC_NOPM, 0, 0,
	&wm9712_capture_phone_mux_controls),
SND_SOC_DAPM_MUX("Left Capture Select", SND_SOC_NOPM, 0, 0,
	&wm9712_capture_selectl_controls),
SND_SOC_DAPM_MUX("Right Capture Select", SND_SOC_NOPM, 0, 0,
	&wm9712_capture_selectr_controls),
SND_SOC_DAPM_MUX("Mic Select Source", SND_SOC_NOPM, 0, 0,
	&wm9712_mic_src_controls),
SND_SOC_DAPM_MUX("Differential Source", SND_SOC_NOPM, 0, 0,
	&wm9712_diff_sel_controls),
SND_SOC_DAPM_MIXER("AC97 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER_E("Left HP Mixer", AC97_INT_PAGING, 9, 1,
	&wm9712_hpl_mixer_controls[0], ARRAY_SIZE(wm9712_hpl_mixer_controls),
	mixer_event, SND_SOC_DAPM_POST_REG),
SND_SOC_DAPM_MIXER_E("Right HP Mixer", AC97_INT_PAGING, 8, 1,
	&wm9712_hpr_mixer_controls[0], ARRAY_SIZE(wm9712_hpr_mixer_controls),
	 mixer_event, SND_SOC_DAPM_POST_REG),
SND_SOC_DAPM_MIXER("Phone Mixer", AC97_INT_PAGING, 6, 1,
	&wm9712_phone_mixer_controls[0], ARRAY_SIZE(wm9712_phone_mixer_controls)),
SND_SOC_DAPM_MIXER("Speaker Mixer", AC97_INT_PAGING, 7, 1,
	&wm9712_speaker_mixer_controls[0],
	ARRAY_SIZE(wm9712_speaker_mixer_controls)),
SND_SOC_DAPM_MIXER("Mono Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_DAC("Left DAC", "Left HiFi Playback", AC97_INT_PAGING, 14, 1),
SND_SOC_DAPM_DAC("Right DAC", "Right HiFi Playback", AC97_INT_PAGING, 13, 1),
SND_SOC_DAPM_DAC("Aux DAC", "Aux Playback", SND_SOC_NOPM, 0, 0),
SND_SOC_DAPM_ADC("Left ADC", "Left HiFi Capture", AC97_INT_PAGING, 12, 1),
SND_SOC_DAPM_ADC("Right ADC", "Right HiFi Capture", AC97_INT_PAGING, 11, 1),
SND_SOC_DAPM_PGA("Headphone PGA", AC97_INT_PAGING, 4, 1, NULL, 0),
SND_SOC_DAPM_PGA("Speaker PGA", AC97_INT_PAGING, 3, 1, NULL, 0),
SND_SOC_DAPM_PGA("Out 3 PGA", AC97_INT_PAGING, 5, 1, NULL, 0),
SND_SOC_DAPM_PGA("Line PGA", AC97_INT_PAGING, 2, 1, NULL, 0),
SND_SOC_DAPM_PGA("Phone PGA", AC97_INT_PAGING, 1, 1, NULL, 0),
SND_SOC_DAPM_PGA("Mic PGA", AC97_INT_PAGING, 0, 1, NULL, 0),
SND_SOC_DAPM_MICBIAS("Mic Bias", AC97_INT_PAGING, 10, 1),
SND_SOC_DAPM_OUTPUT("MONOOUT"),
SND_SOC_DAPM_OUTPUT("HPOUTL"),
SND_SOC_DAPM_OUTPUT("HPOUTR"),
SND_SOC_DAPM_OUTPUT("LOUT2"),
SND_SOC_DAPM_OUTPUT("ROUT2"),
SND_SOC_DAPM_OUTPUT("OUT3"),
SND_SOC_DAPM_INPUT("LINEINL"),
SND_SOC_DAPM_INPUT("LINEINR"),
SND_SOC_DAPM_INPUT("PHONE"),
SND_SOC_DAPM_INPUT("PCBEEP"),
SND_SOC_DAPM_INPUT("MIC1"),
SND_SOC_DAPM_INPUT("MIC2"),
};

static const struct snd_soc_dapm_route audio_map[] = {
	/* virtual mixer - mixes left & right channels for spk and mono */
	{"AC97 Mixer", NULL, "Left DAC"},
	{"AC97 Mixer", NULL, "Right DAC"},

	/* Left HP mixer */
	{"Left HP Mixer", "PCBeep Bypass Switch", "PCBEEP"},
	{"Left HP Mixer", "Aux Playback Switch",  "Aux DAC"},
	{"Left HP Mixer", "Phone Bypass Switch",  "Phone PGA"},
	{"Left HP Mixer", "Line Bypass Switch",   "Line PGA"},
	{"Left HP Mixer", "PCM Playback Switch",  "Left DAC"},
	{"Left HP Mixer", "Mic Sidetone Switch",  "Mic PGA"},
	{"Left HP Mixer", NULL,  "ALC Sidetone Mux"},

	/* Right HP mixer */
	{"Right HP Mixer", "PCBeep Bypass Switch", "PCBEEP"},
	{"Right HP Mixer", "Aux Playback Switch",  "Aux DAC"},
	{"Right HP Mixer", "Phone Bypass Switch",  "Phone PGA"},
	{"Right HP Mixer", "Line Bypass Switch",   "Line PGA"},
	{"Right HP Mixer", "PCM Playback Switch",  "Right DAC"},
	{"Right HP Mixer", "Mic Sidetone Switch",  "Mic PGA"},
	{"Right HP Mixer", NULL,  "ALC Sidetone Mux"},

	/* speaker mixer */
	{"Speaker Mixer", "PCBeep Bypass Switch", "PCBEEP"},
	{"Speaker Mixer", "Line Bypass Switch",   "Line PGA"},
	{"Speaker Mixer", "PCM Playback Switch",  "AC97 Mixer"},
	{"Speaker Mixer", "Phone Bypass Switch",  "Phone PGA"},
	{"Speaker Mixer", "Aux Playback Switch",  "Aux DAC"},

	/* Phone mixer */
	{"Phone Mixer", "PCBeep Bypass Switch",  "PCBEEP"},
	{"Phone Mixer", "Line Bypass Switch",    "Line PGA"},
	{"Phone Mixer", "Aux Playback Switch",   "Aux DAC"},
	{"Phone Mixer", "PCM Playback Switch",   "AC97 Mixer"},
	{"Phone Mixer", "Mic 1 Sidetone Switch", "Mic PGA"},
	{"Phone Mixer", "Mic 2 Sidetone Switch", "Mic PGA"},

	/* inputs */
	{"Line PGA", NULL, "LINEINL"},
	{"Line PGA", NULL, "LINEINR"},
	{"Phone PGA", NULL, "PHONE"},
	{"Mic PGA", NULL, "MIC1"},
	{"Mic PGA", NULL, "MIC2"},

	/* left capture selector */
	{"Left Capture Select", "Mic", "MIC1"},
	{"Left Capture Select", "Speaker Mixer", "Speaker Mixer"},
	{"Left Capture Select", "Line", "LINEINL"},
	{"Left Capture Select", "Headphone Mixer", "Left HP Mixer"},
	{"Left Capture Select", "Phone Mixer", "Phone Mixer"},
	{"Left Capture Select", "Phone", "PHONE"},

	/* right capture selector */
	{"Right Capture Select", "Mic", "MIC2"},
	{"Right Capture Select", "Speaker Mixer", "Speaker Mixer"},
	{"Right Capture Select", "Line", "LINEINR"},
	{"Right Capture Select", "Headphone Mixer", "Right HP Mixer"},
	{"Right Capture Select", "Phone Mixer", "Phone Mixer"},
	{"Right Capture Select", "Phone", "PHONE"},

	/* ALC Sidetone */
	{"ALC Sidetone Mux", "Stereo", "Left Capture Select"},
	{"ALC Sidetone Mux", "Stereo", "Right Capture Select"},
	{"ALC Sidetone Mux", "Left", "Left Capture Select"},
	{"ALC Sidetone Mux", "Right", "Right Capture Select"},

	/* ADC's */
	{"Left ADC", NULL, "Left Capture Select"},
	{"Right ADC", NULL, "Right Capture Select"},

	/* outputs */
	{"MONOOUT", NULL, "Phone Mixer"},
	{"HPOUTL", NULL, "Headphone PGA"},
	{"Headphone PGA", NULL, "Left HP Mixer"},
	{"HPOUTR", NULL, "Headphone PGA"},
	{"Headphone PGA", NULL, "Right HP Mixer"},

	/* mono hp mixer */
	{"Mono HP Mixer", NULL, "Left HP Mixer"},
	{"Mono HP Mixer", NULL, "Right HP Mixer"},

	/* Out3 Mux */
	{"Out3 Mux", "Left", "Left HP Mixer"},
	{"Out3 Mux", "Mono", "Phone Mixer"},
	{"Out3 Mux", "Left + Right", "Mono HP Mixer"},
	{"Out 3 PGA", NULL, "Out3 Mux"},
	{"OUT3", NULL, "Out 3 PGA"},

	/* speaker Mux */
	{"Speaker Mux", "Speaker Mix", "Speaker Mixer"},
	{"Speaker Mux", "Headphone Mix", "Mono HP Mixer"},
	{"Speaker PGA", NULL, "Speaker Mux"},
	{"LOUT2", NULL, "Speaker PGA"},
	{"ROUT2", NULL, "Speaker PGA"},
};

static int wm9712_add_widgets(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card)
{
	int ret;

	ret = snd_soc_dapm_new_controls(soc_card, codec,
					wm9712_dapm_widgets,
					ARRAY_SIZE(wm9712_dapm_widgets));
	if (ret < 0)
		return ret;

	/* set up audio path audio_map */
	ret = snd_soc_dapm_add_routes(soc_card, audio_map,
				     ARRAY_SIZE(audio_map));
	if (ret < 0)
		return ret;

	return snd_soc_dapm_init(soc_card);
}

static unsigned int wm9712_ac97_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 *cache = codec->reg_cache;

	if (reg == AC97_RESET || reg == AC97_GPIO_STATUS ||
		reg == AC97_VENDOR_ID1 || reg == AC97_VENDOR_ID2 ||
		reg == AC97_REC_GAIN) {
			return codec->soc_phys_read(codec->ac97, reg);
		} else {
			reg = reg >> 1;
			if (reg > (ARRAY_SIZE(wm9712_reg)))
				return -EIO;
		return cache[reg];
	}
}

static int wm9712_ac97_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val)
{
	u16 *cache = codec->reg_cache;

	codec->soc_phys_write(codec->ac97, val, reg);
	reg = reg >> 1;
	if (reg <= (ARRAY_SIZE(wm9712_reg)))
		cache[reg] = val;

	return 0;
}

static int ac97_hifi_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *pcm_link = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_codec *codec = pcm_link->codec;
	int reg;
	u16 vra;

	vra = wm9712_ac97_read(codec, AC97_EXTENDED_STATUS);
	wm9712_ac97_write(codec, AC97_EXTENDED_STATUS, vra | 0x1);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg = AC97_PCM_FRONT_DAC_RATE;
	else
		reg = AC97_PCM_LR_ADC_RATE;

	return wm9712_ac97_write(codec, reg, runtime->rate);
}

static int ac97_aux_prepare(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *pcm_link = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_codec *codec = pcm_link->codec;
	u16 vra, xsle;

	vra = wm9712_ac97_read(codec, AC97_EXTENDED_STATUS);
	wm9712_ac97_write(codec, AC97_EXTENDED_STATUS, vra | 0x1);
	xsle = wm9712_ac97_read(codec, AC97_PCI_SID);
	wm9712_ac97_write(codec, AC97_PCI_SID, xsle | 0x8000);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -ENODEV;

	return wm9712_ac97_write(codec, AC97_PCM_SURR_DAC_RATE, runtime->rate);
}

static int wm9712_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_dapm_bias_level level)
{
	u16 reg;

	switch (level) {
	case SND_SOC_BIAS_ON: /* full On */
		/* liam - maybe enable thermal shutdown */
		reg = wm9712_ac97_read(codec, AC97_EXTENDED_MID) & 0xdfff;
		wm9712_ac97_write(codec, AC97_EXTENDED_MID, reg);
		break;
	case SND_SOC_BIAS_PREPARE: /* partial On */
		break;
	case SND_SOC_BIAS_STANDBY: /* Off, with power */
		/* enable master bias and vmid */
		wm9712_ac97_write(codec, AC97_POWERDOWN, 0x0000);
		break;
	case SND_SOC_BIAS_OFF: /* Off, without power */
		/* disable everything including AC link */
		wm9712_ac97_write(codec, AC97_POWERDOWN, 0xffff);
		break;
	}
	codec->bias_level = level;
	return 0;
}

static int wm9712_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct snd_soc_codec *codec = platform_get_drvdata(pdev);

	wm9712_set_bias_level(codec, SND_SOC_BIAS_OFF);
	return 0;
}

static int wm9712_resume(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = platform_get_drvdata(pdev);
	int i;
	u16 *cache = codec->reg_cache;

	wm9712_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* Sync reg_cache with the hardware after cold reset */
	for (i = 2; i < ARRAY_SIZE(wm9712_reg) << 1; i+=2) {
		if (i == AC97_INT_PAGING || i == AC97_POWERDOWN ||
			(i > 0x58 && i != 0x5c))
			continue;
		codec->soc_phys_write(codec->ac97, i, cache[i>>1]);
	}

	if (codec->suspend_bias_level == SND_SOC_BIAS_ON)
		wm9712_set_bias_level(codec, SND_SOC_BIAS_ON);

	return 0;
}

static int wm9712_codec_init(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card)
{
	/* set alc mux to none */
	wm9712_ac97_write(codec, AC97_VIDEO,
		wm9712_ac97_read(codec, AC97_VIDEO) | 0x3000);

	wm9712_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	snd_soc_add_new_controls(soc_card, wm9712_snd_ac97_controls, codec,
		ARRAY_SIZE(wm9712_snd_ac97_controls));
	wm9712_add_widgets(codec, soc_card);

	return 0;
}

#define WM9712_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000)

#define WM9712_PCM_FORMATS \
	(SNDRV_PCM_FMTBIT_S16  | SNDRV_PCM_FMTBIT_S24)

static struct snd_soc_dai_caps wm9712_hifi_playback = {
	.stream_name	= "HiFi Playback",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM9712_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16,
};

static struct snd_soc_dai_caps wm9712_capture = {
	.stream_name	= "HiFi Capture",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM9712_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16,
};

static struct snd_soc_dai_caps wm9712_aux_playback = {
	.stream_name	= "Aux Playback",
	.channels_min	= 1,
	.channels_max	= 1,
	.rates		= WM9712_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16,
};

static struct snd_soc_dai_ops wm9712_hifi_dai_ops = {
	/* alsa ops */
	.prepare = ac97_hifi_prepare,
};

static struct snd_soc_dai_ops wm9712_aux_dai_ops = {
	/* alsa ops */
	.prepare = ac97_aux_prepare,
};

/* for modprobe */
const char wm9712_codec_id[] = "wm9712-codec";
EXPORT_SYMBOL_GPL(wm9712_codec_id);

const char wm9712_codec_hifi_dai_id[] = "wm9712-codec-hifi-dai";
EXPORT_SYMBOL_GPL(wm9712_codec_hifi_dai_id);

const char wm9712_codec_aux_dai_id[] = "wm9712-codec-aux-dai";
EXPORT_SYMBOL_GPL(wm9712_codec_aux_dai_id);

struct snd_soc_dai_new wm9712_hifi_dai = {
	.name		= wm9712_codec_hifi_dai_id,
	.playback	= &wm9712_hifi_playback,
	.capture	= &wm9712_capture,
	.ops		= &wm9712_hifi_dai_ops,
};

struct snd_soc_dai_new wm9712_aux_dai = {
	.name		= wm9712_codec_aux_dai_id,
	.playback	= &wm9712_aux_playback,
	.capture	= &wm9712_capture,
	.ops		= &wm9712_aux_dai_ops,
};

static struct snd_soc_codec_new wm9712_codec = {
	.name		= wm9712_codec_id,
	.reg_cache_size = sizeof(wm9712_reg),
	.reg_cache_step = 2,
	.set_bias_level	= wm9712_set_bias_level,
	.init		= wm9712_codec_init,
	.codec_read	= wm9712_ac97_read,
	.codec_write	= wm9712_ac97_write,
};

static int wm9712_codec_probe(struct platform_device *pdev)
{
	struct wm9712_data *wm9712;
	struct snd_soc_codec *codec;
	int ret;

	printk(KERN_INFO "WM9712 SoC Audio Codec %s\n", WM9712_VERSION);

	codec = snd_soc_new_codec(&wm9712_codec, (char *) wm9712_reg);
	if (codec == NULL)
		return -ENOMEM;

	wm9712 = kzalloc(sizeof(struct wm9712_data), GFP_KERNEL);
	if (wm9712 == NULL) {
		ret =  -ENOMEM;
		goto priv_err;
	}
	codec->private_data = wm9712;
	platform_set_drvdata(pdev, wm9712);

	ret = snd_soc_register_codec(codec, &pdev->dev);
	if (ret < 0)
		goto codec_err;
	wm9712->hifi_dai = snd_soc_register_codec_dai(&wm9712_hifi_dai, &pdev->dev);
	if (wm9712->hifi_dai == NULL)
		goto codec_err;
	wm9712->aux_dai = snd_soc_register_codec_dai(&wm9712_aux_dai, &pdev->dev);
	if (wm9712->aux_dai == NULL)
		goto aux_dai_err;

	return ret;
aux_dai_err:
	snd_soc_unregister_codec_dai(wm9712->hifi_dai);
codec_err:
	kfree(wm9712);
priv_err:
	snd_soc_unregister_codec(codec);
	kfree(codec->reg_cache);
	kfree(codec);
	return ret;
}

static int wm9712_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = platform_get_drvdata(pdev);
	struct wm9712_data *wm9712 = codec->private_data;

	snd_soc_unregister_codec_dai(wm9712->hifi_dai);
	snd_soc_unregister_codec_dai(wm9712->aux_dai);

	kfree(codec->private_data);
	snd_soc_unregister_codec(codec);
	kfree(codec->reg_cache);
	kfree(codec);
	return 0;
}

static struct platform_driver wm9712_codec_driver = {
	.driver = {
		.name		= wm9712_codec_id,
		.owner		= THIS_MODULE,
	},
	.probe		= wm9712_codec_probe,
	.remove		= __devexit_p(wm9712_codec_remove),
	.suspend	= wm9712_suspend,
	.resume		= wm9712_resume,
};

static __init int wm9712_init(void)
{
	return platform_driver_register(&wm9712_codec_driver);
}

static __exit void wm9712_exit(void)
{
	platform_driver_unregister(&wm9712_codec_driver);
}

module_init(wm9712_init);
module_exit(wm9712_exit);

MODULE_DESCRIPTION("ASoC WM9711/WM9712 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
