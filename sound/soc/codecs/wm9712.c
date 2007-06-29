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
#include <linux/device.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#define WM9712_VERSION "0.20"

static unsigned int ac97_read(struct snd_soc_codec *codec,
	unsigned int reg);
static int ac97_write(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int val);

/*
 * WM9712 register cache
 */
static const u16 wm9712_reg[] = {
	0x6174, 0x8000, 0x8000, 0x8000, // 6
	0x0f0f, 0xaaa0, 0xc008, 0x6808, // e
	0xe808, 0xaaa0, 0xad00, 0x8000, // 16
	0xe808, 0x3000, 0x8000, 0x0000, // 1e
	0x0000, 0x0000, 0x0000, 0x000f, // 26
	0x0405, 0x0410, 0xbb80, 0xbb80, // 2e
	0x0000, 0xbb80, 0x0000, 0x0000, // 36
	0x0000, 0x2000, 0x0000, 0x0000, // 3e
	0x0000, 0x0000, 0x0000, 0x0000, // 46
	0x0000, 0x0000, 0xf83e, 0xffff, // 4e
	0x0000, 0x0000, 0x0000, 0xf83e, // 56
	0x0008, 0x0000, 0x0000, 0x0000, // 5e
	0xb032, 0x3e00, 0x0000, 0x0000, // 66
	0x0000, 0x0000, 0x0000, 0x0000, // 6e
	0x0000, 0x0000, 0x0000, 0x0006, // 76
	0x0001, 0x0000, 0x574d, 0x4c12, // 7e
	0x0000, 0x0000 // virtual hp mixers
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
SOC_SINGLE("Headphone Playback Switch", AC97_HEADPHONE,15, 1, 1),
SOC_DOUBLE("PCM Playback Volume", AC97_PCM, 8, 0, 31, 1),

SOC_SINGLE("Speaker Playback ZC Switch", AC97_MASTER, 7, 1, 0),
SOC_SINGLE("Speaker Playback Invert Switch", AC97_MASTER, 6, 1, 0),
SOC_SINGLE("Headphone Playback ZC Switch", AC97_HEADPHONE, 7, 1, 0),
SOC_SINGLE("Mono Playback ZC Switch", AC97_MASTER_MONO, 7, 1, 0),
SOC_SINGLE("Mono Playback Volume", AC97_MASTER_MONO, 0, 31, 0),

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

SOC_SINGLE("Phone Volume", AC97_PHONE, 0, 15, 0),
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
SOC_SINGLE("Bass Volume", AC97_MASTER_TONE, 8, 15, 0),
SOC_SINGLE("Treble Volume", AC97_MASTER_TONE, 0, 15, 0),

SOC_SINGLE("Capture ADC Switch", AC97_REC_GAIN, 15, 1, 1),
SOC_ENUM("Capture Volume Steps", wm9712_enum[6]),
SOC_DOUBLE("Capture Volume", AC97_REC_GAIN, 8, 0, 63, 1),
SOC_SINGLE("Capture ZC Switch", AC97_REC_GAIN, 7, 1, 0),

SOC_SINGLE("Mic 1 Volume", AC97_MIC, 8, 31, 1),
SOC_SINGLE("Mic 2 Volume", AC97_MIC, 0, 31, 1),
SOC_SINGLE("Mic 20dB Boost Switch", AC97_MIC, 7, 1, 0),
};

/* add non dapm controls */
static int wm9712_add_controls(struct snd_soc_codec *codec)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm9712_snd_ac97_controls); i++) {
		err = snd_ctl_add(codec->card,
				snd_soc_cnew(&wm9712_snd_ac97_controls[i],codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

/* We have to create a fake left and right HP mixers because
 * the codec only has a single control that is shared by both channels.
 * This makes it impossible to determine the audio path.
 */
static int mixer_event (struct snd_soc_dapm_widget *w, int event)
{
	u16 l, r, beep, line, phone, mic, pcm, aux;

	l = ac97_read(w->codec, HPL_MIXER);
	r = ac97_read(w->codec, HPR_MIXER);
	beep = ac97_read(w->codec, AC97_PC_BEEP);
	mic = ac97_read(w->codec, AC97_VIDEO);
	phone = ac97_read(w->codec, AC97_PHONE);
	line = ac97_read(w->codec, AC97_LINE);
	pcm = ac97_read(w->codec, AC97_PCM);
	aux = ac97_read(w->codec, AC97_CD);

	if (l & 0x1 || r & 0x1)
		ac97_write(w->codec, AC97_VIDEO, mic & 0x7fff);
	else
		ac97_write(w->codec, AC97_VIDEO, mic | 0x8000);

	if (l & 0x2 || r & 0x2)
		ac97_write(w->codec, AC97_PCM, pcm & 0x7fff);
	else
		ac97_write(w->codec, AC97_PCM, pcm | 0x8000);

	if (l & 0x4 || r & 0x4)
		ac97_write(w->codec, AC97_LINE, line & 0x7fff);
	else
		ac97_write(w->codec, AC97_LINE, line | 0x8000);

	if (l & 0x8 || r & 0x8)
		ac97_write(w->codec, AC97_PHONE, phone & 0x7fff);
	else
		ac97_write(w->codec, AC97_PHONE, phone | 0x8000);

	if (l & 0x10 || r & 0x10)
		ac97_write(w->codec, AC97_CD, aux & 0x7fff);
	else
		ac97_write(w->codec, AC97_CD, aux | 0x8000);

	if (l & 0x20 || r & 0x20)
		ac97_write(w->codec, AC97_PC_BEEP, beep & 0x7fff);
	else
		ac97_write(w->codec, AC97_PC_BEEP, beep | 0x8000);

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

static const char *audio_map[][3] = {
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
	//{"Right HP Mixer", NULL, "HP Mixer"},

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

	{NULL, NULL, NULL},
};

static int wm9712_add_widgets(struct snd_soc_codec *codec)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(wm9712_dapm_widgets); i++) {
		snd_soc_dapm_new_control(codec, &wm9712_dapm_widgets[i]);
	}

	/* set up audio path audio_mapnects */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(codec, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_new_widgets(codec);
	return 0;
}

static unsigned int ac97_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 val = 0, *cache = codec->reg_cache;

	if (reg == AC97_RESET || reg == AC97_GPIO_STATUS ||
		reg == AC97_VENDOR_ID1 || reg == AC97_VENDOR_ID2 ||
		reg == AC97_REC_GAIN)
		codec->mach_read(codec->ac97, (long)&val, reg);
		return val;
	else {
		reg = reg >> 1;

		if (reg > (ARRAY_SIZE(wm9712_reg)))
			return -EIO;

		return cache[reg];
	}
}

static int ac97_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val)
{
	u16 *cache = codec->reg_cache;

	codec->mach_write(codec->ac97, val, reg);
	reg = reg >> 1;
	if (reg <= (ARRAY_SIZE(wm9712_reg)))
		cache[reg] = val;

	return 0;
}

static int ac97_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_codec *codec = pcm_link->codec;
	int reg;
	u16 vra;

	vra = ac97_read(codec, AC97_EXTENDED_STATUS);
	ac97_write(codec, AC97_EXTENDED_STATUS, vra | 0x1);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg = AC97_PCM_FRONT_DAC_RATE;
	else
		reg = AC97_PCM_LR_ADC_RATE;

	return ac97_write(codec, reg, runtime->rate);
}

static int ac97_aux_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_codec *codec = pcm_link->codec;
	u16 vra, xsle;

	vra = ac97_read(codec, AC97_EXTENDED_STATUS);
	ac97_write(codec, AC97_EXTENDED_STATUS, vra | 0x1);
	xsle = ac97_read(codec, AC97_PCI_SID);
	ac97_write(codec, AC97_PCI_SID, xsle | 0x8000);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -ENODEV;

	return ac97_write(codec, AC97_PCM_SURR_DAC_RATE, runtime->rate);
}

static int wm9712_dapm_event(struct snd_soc_codec *codec, int event)
{
	u16 reg;

	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		/* liam - maybe enable thermal shutdown */
		reg = ac97_read(codec, AC97_EXTENDED_MID) & 0xdfff;
		ac97_write(codec, AC97_EXTENDED_MID, reg);
		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		/* enable master bias and vmid */
		reg = ac97_read(codec, AC97_EXTENDED_MID) & 0xbbff;
		ac97_write(codec, AC97_EXTENDED_MID, reg);
		ac97_write(codec, AC97_POWERDOWN, 0x0000);
		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		/* disable everything including AC link */
		ac97_write(codec, AC97_EXTENDED_MID, 0xffff);
		ac97_write(codec, AC97_EXTENDED_MSTATUS, 0xffff);
		ac97_write(codec, AC97_POWERDOWN, 0xffff);
		break;
	}
	codec->dapm_state = event;
	return 0;
}

static int wm9712_reset(struct snd_soc_codec *codec, int try_warm)
{
	if (try_warm && soc_ac97_ops.warm_reset) {
		soc_ac97_ops.warm_reset(codec->ac97);
		if (!(ac97_read(codec, 0) & 0x8000))
			return 1;
	}

	soc_ac97_ops.reset(codec->ac97);
	if (ac97_read(codec, 0) & 0x8000)
		goto err;
	return 0;

err:
	printk(KERN_ERR "WM9712 AC97 reset failed\n");
	return -EIO;
}

static int wm9712_soc_suspend(struct device *dev, pm_message_t state)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	wm9712_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	return 0;
}

static int wm9712_soc_resume(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	int i, ret;
	u16 *cache = codec->reg_cache;

	ret = wm9712_reset(codec, 1);
	if (ret < 0){
		printk(KERN_ERR "could not reset AC97 codec\n");
		return ret;
	}

	wm9712_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	if (ret == 0) {
		/* Sync reg_cache with the hardware after cold reset */
		for (i = 2; i < ARRAY_SIZE(wm9712_reg) << 1; i+=2) {
			if (i == AC97_INT_PAGING || i == AC97_POWERDOWN ||
				(i > 0x58 && i != 0x5c))
				continue;
			soc_ac97_ops.write(codec->ac97, i, cache[i>>1]);
		}
	}

	if (codec->suspend_dapm_state == SNDRV_CTL_POWER_D0)
		wm9712_dapm_event(codec, SNDRV_CTL_POWER_D0);

	return ret;
}

static int wm9712_probe_codec(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	int reg;

	/* set alc mux to none */
	ac97_write(codec, AC97_VIDEO, ac97_read(codec, AC97_VIDEO) | 0x3000);

	wm9712_dapm_event(codec, SNDRV_CTL_POWER_D3hot);
	wm9712_add_controls(codec, machine->card);
	wm9712_add_widgets(codec, machine);
	
	return 0;
}

static const struct snd_soc_codec_ops wm9712_codec_ops = {
	.dapm_event	= wm9712_dapm_event,
	.read		= wm9712_ac97_read,
	.write		= wm9712_ac97_write,
	.probe_codec	= wm9712_probe_codec,
};

static int wm9712_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	int ret = 0;

	printk(KERN_INFO "WM9712 SoC Audio Codec %s\n", WM9712_VERSION);

	codec->reg_cache = kmemdup(wm9712_reg, sizeof(wm9712_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	codec->reg_cache_size = sizeof(wm9712_reg);
	codec->reg_cache_step = 2;

	codec->private_data = kzalloc(sizeof(struct wm9712_priv), GFP_KERNEL);
	if (codec->private_data == NULL) {
		kfree(codec->reg_cache);
		return -ENOMEM;
	}

	codec->owner = THIS_MODULE;
	codec->ops = &wm9712_codec_ops;
	snd_soc_register_codec(codec);
	return ret;
}

static int wm9712_codec_remove(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	kfree(codec->private_data);
	kfree(codec->reg_cache);
	return 0;
}

#define WM9712_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000)

#define WM9712_PCM_FORMATS \
	(SNDRV_PCM_FMTBIT_S16  | SNDRV_PCM_FMTBIT_S24)

static const struct snd_soc_pcm_stream wm9712_hifi_dai_playback = {
	.stream_name	= "HiFi Playback",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM9712_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16,
};

static const struct snd_soc_pcm_stream wm9712_hifi_dai_capture = {
	.stream_name	= "HiFi Capture",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM9712_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16,
};

static const struct snd_soc_pcm_stream wm9712_aux_dai_playback = {
	.stream_name	= "Aux Playback",
	.channels_min	= 1,
	.channels_max	= 1,
	.rates		= WM9712_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops wm9712_hifi_dai_audio_ops = {
	.prepare = ac97_hifi_prepare,
};
/* audio ops, called by alsa */
static const struct snd_soc_ops wm9712_aux_dai_audio_ops = {
	.prepare = ac97_aux_prepare,
};

static int wm9712_hifi_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);
		
	dai->audio_ops = &wm9712_hifi_dai_audio_ops;
	dai->capture = &wm9712_hifi_dai_capture;
	dai->playback = &wm9712_hifi_dai_playback;
	snd_soc_register_codec_dai(dai);
	return 0;
}

static int wm9712_aux_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);
		
	dai->audio_ops = &wm9712_aux_dai_audio_ops;
	dai->playback = &wm9712_aux_dai_playback;
	snd_soc_register_codec_dai(dai);
	return 0;
}


const char wm9712_codec[SND_SOC_CODEC_NAME_SIZE] = "wm9712-codec";
EXPORT_SYMBOL_GPL(wm9712_codec);

static struct snd_soc_device_driver wm9712_codec_driver = {
	.type	= SND_SOC_BUS_TYPE_CODEC,
	.driver	= {
		.name 		= wm9712_codec,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm9712_codec_probe,
		.remove		= __devexit_p(wm9712_codec_remove),
		.suspend	= wm9712_codec_suspend,
		.resume		= wm9712_codec_resume,
	},
};

const char wm9712_hifi_dai[SND_SOC_CODEC_NAME_SIZE] = "wm9712-hifi-dai";
EXPORT_SYMBOL_GPL(wm9712_hifi_dai);

static struct snd_soc_device_driver wm9712_hifi_dai_driver = {
	.type	= SND_SOC_BUS_TYPE_DAI,
	.driver	= {
		.name 		= wm9712_hifi_dai,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm9712_hifi_dai_probe,
	},
};

const char wm9712_aux_dai[SND_SOC_CODEC_NAME_SIZE] = "wm9712-aux-dai";
EXPORT_SYMBOL_GPL(wm9712_aux_dai);

static struct snd_soc_device_driver wm9712_aux_dai_driver = {
	.type	= SND_SOC_BUS_TYPE_DAI,
	.driver	= {
		.name 		= wm9712_aux_dai,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm9712_aux_dai_probe,
	},
};

static __init int wm9712_init(void)
{
	int ret = 0;
	
	ret = driver_register(&wm9712_codec_driver.driver);
	if (ret < 0)
		return ret;
	ret = driver_register(&wm9712_hifi_dai_driver.driver);
	if (ret < 0)
		goto hifi_err;
	ret = driver_register(&wm9712_aux_dai_driver.driver);
	if (ret < 0)
		goto aux_err;
		
	return ret;
aux_err:
	driver_unregister(&wm9712_hifi_dai_driver.driver);
hifi_err:
	driver_unregister(&wm9712_codec_driver.driver);
	return ret;
}

static __exit void wm9712_exit(void)
{
	driver_unregister(&wm9712_aux_dai_driver.driver);
	driver_unregister(&wm9712_hifi_dai_driver.driver);
	driver_unregister(&wm9712_codec_driver.driver);
}

module_init(wm9712_init);
module_exit(wm9712_exit);

MODULE_DESCRIPTION("ASoC WM9711/WM9712 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
