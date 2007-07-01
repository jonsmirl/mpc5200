/*
 * wm9713.c  --  ALSA Soc WM9713 codec support
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
 *
 *  Features:-
 *
 *   o Support for AC97 Codec, Voice DAC and Aux DAC
 *   o Support for DAPM
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/ac97_codec.h>
#include <sound/initval.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

#include "wm9713.h"

#define WM9713_VERSION "0.20"

struct wm9713_priv {
	u32 pll_in; /* PLL input frequency */
	u32 pll_out; /* PLL output frequency */
};

static unsigned int wm9713_ac97_read(struct snd_soc_codec *codec,
	unsigned int reg);
static int wm9713_ac97_write(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int val);

/*
 * WM9713 register cache
 * Reg 0x3c bit 15 is used by touch driver.
 */
static const u16 wm9713_reg[] = {
	0x6174, 0x8080, 0x8080, 0x8080, // 6
	0xc880, 0xe808, 0xe808, 0x0808, // e
	0x00da, 0x8000, 0xd600, 0xaaa0, // 16
	0xaaa0, 0xaaa0, 0x0000, 0x0000, // 1e
	0x0f0f, 0x0040, 0x0000, 0x7f00, // 26
	0x0405, 0x0410, 0xbb80, 0xbb80, // 2e
	0x0000, 0xbb80, 0x0000, 0x4523, // 36
	0x0000, 0x2000, 0x7eff, 0xffff, // 3e
	0x0000, 0x0000, 0x0080, 0x0000, // 46
	0x0000, 0x0000, 0xfffe, 0xffff, // 4e
	0x0000, 0x0000, 0x0000, 0xfffe, // 56
	0x4000, 0x0000, 0x0000, 0x0000, // 5e
	0xb032, 0x3e00, 0x0000, 0x0000, // 66
	0x0000, 0x0000, 0x0000, 0x0000, // 6e
	0x0000, 0x0000, 0x0000, 0x0006, // 76
	0x0001, 0x0000, 0x574d, 0x4c13, // 7e
	0x0000, 0x0000, 0x0000 // virtual hp & mic mixers
};

/* virtual HP mixers regs */
#define HPL_MIXER	0x80
#define HPR_MIXER	0x82
#define MICB_MUX	0x82

static const char *wm9713_mic_mixer[] = {"Stereo", "Mic 1", "Mic 2", "Mute"};
static const char *wm9713_rec_mux[] = {"Stereo", "Left", "Right", "Mute"};
static const char *wm9713_rec_src[] =
	{"Mic 1", "Mic 2", "Line", "Mono In", "Headphone", "Speaker",
	"Mono Out", "Zh"};
static const char *wm9713_rec_gain[] = {"+1.5dB Steps", "+0.75dB Steps"};
static const char *wm9713_alc_select[] = {"None", "Left", "Right", "Stereo"};
static const char *wm9713_mono_pga[] = {"Vmid", "Zh", "Mono", "Inv",
	"Mono Vmid", "Inv Vmid"};
static const char *wm9713_spk_pga[] =
	{"Vmid", "Zh", "Headphone", "Speaker", "Inv", "Headphone Vmid",
	"Speaker Vmid", "Inv Vmid"};
static const char *wm9713_hp_pga[] = {"Vmid", "Zh", "Headphone",
	"Headphone Vmid"};
static const char *wm9713_out3_pga[] = {"Vmid", "Zh", "Inv 1", "Inv 1 Vmid"};
static const char *wm9713_out4_pga[] = {"Vmid", "Zh", "Inv 2", "Inv 2 Vmid"};
static const char *wm9713_dac_inv[] =
	{"Off", "Mono", "Speaker", "Left Headphone", "Right Headphone",
	"Headphone Mono", "NC", "Vmid"};
static const char *wm9713_bass[] = {"Linear Control", "Adaptive Boost"};
static const char *wm9713_ng_type[] = {"Constant Gain", "Mute"};
static const char *wm9713_mic_select[] = {"Mic 1", "Mic 2 A", "Mic 2 B"};
static const char *wm9713_micb_select[] = {"MPB", "MPA"};

static const struct soc_enum wm9713_enum[] = {
SOC_ENUM_SINGLE(AC97_LINE, 3, 4, wm9713_mic_mixer), /* record mic mixer 0 */
SOC_ENUM_SINGLE(AC97_VIDEO, 14, 4, wm9713_rec_mux), /* record mux hp 1 */
SOC_ENUM_SINGLE(AC97_VIDEO, 9, 4, wm9713_rec_mux),  /* record mux mono 2 */
SOC_ENUM_SINGLE(AC97_VIDEO, 3, 8, wm9713_rec_src),  /* record mux left 3 */
SOC_ENUM_SINGLE(AC97_VIDEO, 0, 8, wm9713_rec_src),  /* record mux right 4*/
SOC_ENUM_DOUBLE(AC97_CD, 14, 6, 2, wm9713_rec_gain), /* record step size 5 */
SOC_ENUM_SINGLE(AC97_PCI_SVID, 14, 4, wm9713_alc_select), /* alc source select 6*/
SOC_ENUM_SINGLE(AC97_REC_GAIN, 14, 4, wm9713_mono_pga), /* mono input select 7 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 11, 8, wm9713_spk_pga), /* speaker left input select 8 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 8, 8, wm9713_spk_pga), /* speaker right input select 9 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 6, 3, wm9713_hp_pga), /* headphone left input 10 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 4, 3, wm9713_hp_pga), /* headphone right input 11 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 2, 4, wm9713_out3_pga), /* out 3 source 12 */
SOC_ENUM_SINGLE(AC97_REC_GAIN, 0, 4, wm9713_out4_pga), /* out 4 source 13 */
SOC_ENUM_SINGLE(AC97_REC_GAIN_MIC, 13, 8, wm9713_dac_inv), /* dac invert 1 14 */
SOC_ENUM_SINGLE(AC97_REC_GAIN_MIC, 10, 8, wm9713_dac_inv), /* dac invert 2 15 */
SOC_ENUM_SINGLE(AC97_GENERAL_PURPOSE, 15, 2, wm9713_bass), /* bass control 16 */
SOC_ENUM_SINGLE(AC97_PCI_SVID, 5, 2, wm9713_ng_type), /* noise gate type 17 */
SOC_ENUM_SINGLE(AC97_3D_CONTROL, 12, 3, wm9713_mic_select), /* mic selection 18 */
SOC_ENUM_SINGLE(MICB_MUX, 0, 2, wm9713_micb_select), /* mic selection 19 */
};

static const struct snd_kcontrol_new wm9713_snd_ac97_controls[] = {
SOC_DOUBLE("Speaker Playback Volume", AC97_MASTER, 8, 0, 31, 1),
SOC_DOUBLE("Speaker Playback Switch", AC97_MASTER, 15, 7, 1, 1),
SOC_DOUBLE("Headphone Playback Volume", AC97_HEADPHONE, 8, 0, 31, 1),
SOC_DOUBLE("Headphone Playback Switch", AC97_HEADPHONE,15, 7, 1, 1),
SOC_DOUBLE("Line In Volume", AC97_PC_BEEP, 8, 0, 31, 1),
SOC_DOUBLE("PCM Playback Volume", AC97_PHONE, 8, 0, 31, 1),
SOC_SINGLE("Mic 1 Volume", AC97_MIC, 8, 31, 1),
SOC_SINGLE("Mic 2 Volume", AC97_MIC, 0, 31, 1),

SOC_SINGLE("Mic Boost (+20dB) Switch", AC97_LINE, 5, 1, 0),
SOC_SINGLE("Mic Headphone Mixer Volume", AC97_LINE, 0, 7, 1),

SOC_SINGLE("Capture Switch", AC97_CD, 15, 1, 1),
SOC_ENUM("Capture Volume Steps", wm9713_enum[5]),
SOC_DOUBLE("Capture Volume", AC97_CD, 8, 0, 31, 0),
SOC_SINGLE("Capture ZC Switch", AC97_CD, 7, 1, 0),

SOC_SINGLE("Capture to Headphone Volume", AC97_VIDEO, 11, 7, 1),
SOC_SINGLE("Capture to Mono Boost (+20dB) Switch", AC97_VIDEO, 8, 1, 0),
SOC_SINGLE("Capture ADC Boost (+20dB) Switch", AC97_VIDEO, 6, 1, 0),

SOC_SINGLE("ALC Target Volume", AC97_CODEC_CLASS_REV, 12, 15, 0),
SOC_SINGLE("ALC Hold Time", AC97_CODEC_CLASS_REV, 8, 15, 0),
SOC_SINGLE("ALC Decay Time ", AC97_CODEC_CLASS_REV, 4, 15, 0),
SOC_SINGLE("ALC Attack Time", AC97_CODEC_CLASS_REV, 0, 15, 0),
SOC_ENUM("ALC Function", wm9713_enum[6]),
SOC_SINGLE("ALC Max Volume", AC97_PCI_SVID, 11, 7, 0),
SOC_SINGLE("ALC ZC Timeout", AC97_PCI_SVID, 9, 3, 0),
SOC_SINGLE("ALC ZC Switch", AC97_PCI_SVID, 8, 1, 0),
SOC_SINGLE("ALC NG Switch", AC97_PCI_SVID, 7, 1, 0),
SOC_ENUM("ALC NG Type", wm9713_enum[17]),
SOC_SINGLE("ALC NG Threshold", AC97_PCI_SVID, 0, 31, 0),

SOC_DOUBLE("Speaker Playback ZC Switch", AC97_MASTER, 14, 6, 1, 0),
SOC_DOUBLE("Headphone Playback ZC Switch", AC97_HEADPHONE, 14, 6, 1, 0),

SOC_SINGLE("Out4 Playback Switch", AC97_MASTER_MONO, 15, 1, 1),
SOC_SINGLE("Out4 Playback ZC Switch", AC97_MASTER_MONO, 14, 1, 0),
SOC_SINGLE("Out4 Playback Volume", AC97_MASTER_MONO, 8, 63, 1),

SOC_SINGLE("Out3 Playback Switch", AC97_MASTER_MONO, 7, 1, 1),
SOC_SINGLE("Out3 Playback ZC Switch", AC97_MASTER_MONO, 6, 1, 0),
SOC_SINGLE("Out3 Playback Volume", AC97_MASTER_MONO, 0, 63, 1),

SOC_SINGLE("Mono Capture Volume", AC97_MASTER_TONE, 8, 31, 1),
SOC_SINGLE("Mono Playback Switch", AC97_MASTER_TONE, 7, 1, 1),
SOC_SINGLE("Mono Playback ZC Switch", AC97_MASTER_TONE, 6, 1, 0),
SOC_SINGLE("Mono Playback Volume", AC97_MASTER_TONE, 0, 31, 1),

SOC_SINGLE("PC Beep Playback Headphone Volume", AC97_AUX, 12, 7, 1),
SOC_SINGLE("PC Beep Playback Speaker Volume", AC97_AUX, 8, 7, 1),
SOC_SINGLE("PC Beep Playback Mono Volume", AC97_AUX, 4, 7, 1),

SOC_SINGLE("Voice Playback Headphone Volume", AC97_PCM, 12, 7, 1),
SOC_SINGLE("Voice Playback Master Volume", AC97_PCM, 8, 7, 1),
SOC_SINGLE("Voice Playback Mono Volume", AC97_PCM, 4, 7, 1),

SOC_SINGLE("Aux Playback Headphone Volume", AC97_REC_SEL, 12, 7, 1),
SOC_SINGLE("Aux Playback Master Volume", AC97_REC_SEL, 8, 7, 1),
SOC_SINGLE("Aux Playback Mono Volume", AC97_REC_SEL, 4, 7, 1),

SOC_ENUM("Bass Control", wm9713_enum[16]),
SOC_SINGLE("Bass Cut-off Switch", AC97_GENERAL_PURPOSE, 12, 1, 1),
SOC_SINGLE("Tone Cut-off Switch", AC97_GENERAL_PURPOSE, 4, 1, 1),
SOC_SINGLE("Playback Attenuate (-6dB) Switch", AC97_GENERAL_PURPOSE, 6, 1, 0),
SOC_SINGLE("Bass Volume", AC97_GENERAL_PURPOSE, 8, 15, 1),
SOC_SINGLE("Tone Volume", AC97_GENERAL_PURPOSE, 0, 15, 1),

SOC_SINGLE("3D Upper Cut-off Switch", AC97_REC_GAIN_MIC, 5, 1, 0),
SOC_SINGLE("3D Lower Cut-off Switch", AC97_REC_GAIN_MIC, 4, 1, 0),
SOC_SINGLE("3D Depth", AC97_REC_GAIN_MIC, 0, 15, 1),
};

/* add non dapm controls */
static int wm9713_add_controls(struct snd_soc_codec *codec, 
	struct snd_card *card)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(wm9713_snd_ac97_controls); i++) {
		err = snd_ctl_add(card,
				snd_soc_cnew(&wm9713_snd_ac97_controls[i],
				codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}

/* We have to create a fake left and right HP mixers because
 * the codec only has a single control that is shared by both channels.
 * This makes it impossible to determine the audio path using the current
 * register map, thus we add a new (virtual) register to help determine the
 * audio route within the device.
 */
static int mixer_event (struct snd_soc_dapm_widget *w, int event)
{
	u16 l, r, beep, tone, phone, rec, pcm, aux;

	l = wm9713_ac97_read(w->codec, HPL_MIXER);
	r = wm9713_ac97_read(w->codec, HPR_MIXER);
	beep = wm9713_ac97_read(w->codec, AC97_PC_BEEP);
	tone = wm9713_ac97_read(w->codec, AC97_MASTER_TONE);
	phone = wm9713_ac97_read(w->codec, AC97_PHONE);
	rec = wm9713_ac97_read(w->codec, AC97_REC_SEL);
	pcm = wm9713_ac97_read(w->codec, AC97_PCM);
	aux = wm9713_ac97_read(w->codec, AC97_AUX);

	if (event & SND_SOC_DAPM_PRE_REG)
		return 0;
	if (l & 0x1 || r & 0x1)
		wm9713_ac97_write(w->codec, AC97_PC_BEEP, beep & 0x7fff);
	else
		wm9713_ac97_write(w->codec, AC97_PC_BEEP, beep | 0x8000);

	if (l & 0x2 || r & 0x2)
		wm9713_ac97_write(w->codec, AC97_MASTER_TONE, tone & 0x7fff);
	else
		wm9713_ac97_write(w->codec, AC97_MASTER_TONE, tone | 0x8000);

	if (l & 0x4 || r & 0x4)
		wm9713_ac97_write(w->codec, AC97_PHONE, phone & 0x7fff);
	else
		wm9713_ac97_write(w->codec, AC97_PHONE, phone | 0x8000);

	if (l & 0x8 || r & 0x8)
		wm9713_ac97_write(w->codec, AC97_REC_SEL, rec & 0x7fff);
	else
		wm9713_ac97_write(w->codec, AC97_REC_SEL, rec | 0x8000);

	if (l & 0x10 || r & 0x10)
		wm9713_ac97_write(w->codec, AC97_PCM, pcm & 0x7fff);
	else
		wm9713_ac97_write(w->codec, AC97_PCM, pcm | 0x8000);

	if (l & 0x20 || r & 0x20)
		wm9713_ac97_write(w->codec, AC97_AUX, aux & 0x7fff);
	else
		wm9713_ac97_write(w->codec, AC97_AUX, aux | 0x8000);

	return 0;
}

/* Left Headphone Mixers */
static const struct snd_kcontrol_new wm9713_hpl_mixer_controls[] = {
SOC_DAPM_SINGLE("PC Beep Playback Switch", HPL_MIXER, 5, 1, 0),
SOC_DAPM_SINGLE("Voice Playback Switch", HPL_MIXER, 4, 1, 0),
SOC_DAPM_SINGLE("Aux Playback Switch", HPL_MIXER, 3, 1, 0),
SOC_DAPM_SINGLE("PCM Playback Switch", HPL_MIXER, 2, 1, 0),
SOC_DAPM_SINGLE("MonoIn Playback Switch", HPL_MIXER, 1, 1, 0),
SOC_DAPM_SINGLE("Bypass Playback Switch", HPL_MIXER, 0, 1, 0),
};

/* Right Headphone Mixers */
static const struct snd_kcontrol_new wm9713_hpr_mixer_controls[] = {
SOC_DAPM_SINGLE("PC Beep Playback Switch", HPR_MIXER, 5, 1, 0),
SOC_DAPM_SINGLE("Voice Playback Switch", HPR_MIXER, 4, 1, 0),
SOC_DAPM_SINGLE("Aux Playback Switch", HPR_MIXER, 3, 1, 0),
SOC_DAPM_SINGLE("PCM Playback Switch", HPR_MIXER, 2, 1, 0),
SOC_DAPM_SINGLE("MonoIn Playback Switch", HPR_MIXER, 1, 1, 0),
SOC_DAPM_SINGLE("Bypass Playback Switch", HPR_MIXER, 0, 1, 0),
};

/* headphone capture mux */
static const struct snd_kcontrol_new wm9713_hp_rec_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[1]);

/* headphone mic mux */
static const struct snd_kcontrol_new wm9713_hp_mic_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[0]);

/* Speaker Mixer */
static const struct snd_kcontrol_new wm9713_speaker_mixer_controls[] = {
SOC_DAPM_SINGLE("PC Beep Playback Switch", AC97_AUX, 11, 1, 1),
SOC_DAPM_SINGLE("Voice Playback Switch", AC97_PCM, 11, 1, 1),
SOC_DAPM_SINGLE("Aux Playback Switch", AC97_REC_SEL, 11, 1, 1),
SOC_DAPM_SINGLE("PCM Playback Switch", AC97_PHONE, 14, 1, 1),
SOC_DAPM_SINGLE("MonoIn Playback Switch", AC97_MASTER_TONE, 14, 1, 1),
SOC_DAPM_SINGLE("Bypass Playback Switch", AC97_PC_BEEP, 14, 1, 1),
};

/* Mono Mixer */
static const struct snd_kcontrol_new wm9713_mono_mixer_controls[] = {
SOC_DAPM_SINGLE("PC Beep Playback Switch", AC97_AUX, 7, 1, 1),
SOC_DAPM_SINGLE("Voice Playback Switch", AC97_PCM, 7, 1, 1),
SOC_DAPM_SINGLE("Aux Playback Switch", AC97_REC_SEL, 7, 1, 1),
SOC_DAPM_SINGLE("PCM Playback Switch", AC97_PHONE, 13, 1, 1),
SOC_DAPM_SINGLE("MonoIn Playback Switch", AC97_MASTER_TONE, 13, 1, 1),
SOC_DAPM_SINGLE("Bypass Playback Switch", AC97_PC_BEEP, 13, 1, 1),
SOC_DAPM_SINGLE("Mic 1 Sidetone Switch", AC97_LINE, 7, 1, 1),
SOC_DAPM_SINGLE("Mic 2 Sidetone Switch", AC97_LINE, 6, 1, 1),
};

/* mono mic mux */
static const struct snd_kcontrol_new wm9713_mono_mic_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[2]);

/* mono output mux */
static const struct snd_kcontrol_new wm9713_mono_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[7]);

/* speaker left output mux */
static const struct snd_kcontrol_new wm9713_hp_spkl_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[8]);

/* speaker right output mux */
static const struct snd_kcontrol_new wm9713_hp_spkr_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[9]);

/* headphone left output mux */
static const struct snd_kcontrol_new wm9713_hpl_out_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[10]);

/* headphone right output mux */
static const struct snd_kcontrol_new wm9713_hpr_out_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[11]);

/* Out3 mux */
static const struct snd_kcontrol_new wm9713_out3_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[12]);

/* Out4 mux */
static const struct snd_kcontrol_new wm9713_out4_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[13]);

/* DAC inv mux 1 */
static const struct snd_kcontrol_new wm9713_dac_inv1_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[14]);

/* DAC inv mux 2 */
static const struct snd_kcontrol_new wm9713_dac_inv2_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[15]);

/* Capture source left */
static const struct snd_kcontrol_new wm9713_rec_srcl_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[3]);

/* Capture source right */
static const struct snd_kcontrol_new wm9713_rec_srcr_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[4]);

/* mic source */
static const struct snd_kcontrol_new wm9713_mic_sel_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[18]);

/* mic source B virtual control */
static const struct snd_kcontrol_new wm9713_micb_sel_mux_controls =
SOC_DAPM_ENUM("Route", wm9713_enum[19]);

static const struct snd_soc_dapm_widget wm9713_dapm_widgets[] = {
SND_SOC_DAPM_MUX("Capture Headphone Mux", SND_SOC_NOPM, 0, 0,
	&wm9713_hp_rec_mux_controls),
SND_SOC_DAPM_MUX("Sidetone Mux", SND_SOC_NOPM, 0, 0,
	&wm9713_hp_mic_mux_controls),
SND_SOC_DAPM_MUX("Capture Mono Mux", SND_SOC_NOPM, 0, 0,
	&wm9713_mono_mic_mux_controls),
SND_SOC_DAPM_MUX("Mono Out Mux", SND_SOC_NOPM, 0, 0,
	&wm9713_mono_mux_controls),
SND_SOC_DAPM_MUX("Left Speaker Out Mux", SND_SOC_NOPM, 0, 0,
	&wm9713_hp_spkl_mux_controls),
SND_SOC_DAPM_MUX("Right Speaker Out Mux", SND_SOC_NOPM, 0, 0,
	&wm9713_hp_spkr_mux_controls),
SND_SOC_DAPM_MUX("Left Headphone Out Mux", SND_SOC_NOPM, 0, 0,
	&wm9713_hpl_out_mux_controls),
SND_SOC_DAPM_MUX("Right Headphone Out Mux", SND_SOC_NOPM, 0, 0,
	&wm9713_hpr_out_mux_controls),
SND_SOC_DAPM_MUX("Out 3 Mux", SND_SOC_NOPM, 0, 0,
	&wm9713_out3_mux_controls),
SND_SOC_DAPM_MUX("Out 4 Mux", SND_SOC_NOPM, 0, 0,
	&wm9713_out4_mux_controls),
SND_SOC_DAPM_MUX("DAC Inv Mux 1", SND_SOC_NOPM, 0, 0,
	&wm9713_dac_inv1_mux_controls),
SND_SOC_DAPM_MUX("DAC Inv Mux 2", SND_SOC_NOPM, 0, 0,
	&wm9713_dac_inv2_mux_controls),
SND_SOC_DAPM_MUX("Left Capture Source", SND_SOC_NOPM, 0, 0,
	&wm9713_rec_srcl_mux_controls),
SND_SOC_DAPM_MUX("Right Capture Source", SND_SOC_NOPM, 0, 0,
	&wm9713_rec_srcr_mux_controls),
SND_SOC_DAPM_MUX("Mic A Source", SND_SOC_NOPM, 0, 0,
	&wm9713_mic_sel_mux_controls ),
SND_SOC_DAPM_MUX("Mic B Source", SND_SOC_NOPM, 0, 0,
	&wm9713_micb_sel_mux_controls ),
SND_SOC_DAPM_MIXER_E("Left HP Mixer", AC97_EXTENDED_MID, 3, 1,
	&wm9713_hpl_mixer_controls[0], ARRAY_SIZE(wm9713_hpl_mixer_controls),
	mixer_event, SND_SOC_DAPM_POST_REG),
SND_SOC_DAPM_MIXER_E("Right HP Mixer", AC97_EXTENDED_MID, 2, 1,
	&wm9713_hpr_mixer_controls[0], ARRAY_SIZE(wm9713_hpr_mixer_controls),
	mixer_event, SND_SOC_DAPM_POST_REG),
SND_SOC_DAPM_MIXER("Mono Mixer", AC97_EXTENDED_MID, 0, 1,
	&wm9713_mono_mixer_controls[0], ARRAY_SIZE(wm9713_mono_mixer_controls)),
SND_SOC_DAPM_MIXER("Speaker Mixer", AC97_EXTENDED_MID, 1, 1,
	&wm9713_speaker_mixer_controls[0],
	ARRAY_SIZE(wm9713_speaker_mixer_controls)),
SND_SOC_DAPM_DAC("Left DAC", "Left HiFi Playback", AC97_EXTENDED_MID, 7, 1),
SND_SOC_DAPM_DAC("Right DAC", "Right HiFi Playback", AC97_EXTENDED_MID, 6, 1),
SND_SOC_DAPM_MIXER("AC97 Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("HP Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_MIXER("Capture Mixer", SND_SOC_NOPM, 0, 0, NULL, 0),
SND_SOC_DAPM_DAC("Voice DAC", "Voice Playback", AC97_EXTENDED_MID, 12, 1),
SND_SOC_DAPM_DAC("Aux DAC", "Aux Playback", AC97_EXTENDED_MID, 11, 1),
SND_SOC_DAPM_ADC("Left ADC", "Left HiFi Capture", AC97_EXTENDED_MID, 5, 1),
SND_SOC_DAPM_ADC("Right ADC", "Right HiFi Capture", AC97_EXTENDED_MID, 4, 1),
SND_SOC_DAPM_PGA("Left Headphone", AC97_EXTENDED_MSTATUS, 10, 1, NULL, 0),
SND_SOC_DAPM_PGA("Right Headphone", AC97_EXTENDED_MSTATUS, 9, 1, NULL, 0),
SND_SOC_DAPM_PGA("Left Speaker", AC97_EXTENDED_MSTATUS, 8, 1, NULL, 0),
SND_SOC_DAPM_PGA("Right Speaker", AC97_EXTENDED_MSTATUS, 7, 1, NULL, 0),
SND_SOC_DAPM_PGA("Out 3", AC97_EXTENDED_MSTATUS, 11, 1, NULL, 0),
SND_SOC_DAPM_PGA("Out 4", AC97_EXTENDED_MSTATUS, 12, 1, NULL, 0),
SND_SOC_DAPM_PGA("Mono Out", AC97_EXTENDED_MSTATUS, 13, 1, NULL, 0),
SND_SOC_DAPM_PGA("Left Line In", AC97_EXTENDED_MSTATUS, 6, 1, NULL, 0),
SND_SOC_DAPM_PGA("Right Line In", AC97_EXTENDED_MSTATUS, 5, 1, NULL, 0),
SND_SOC_DAPM_PGA("Mono In", AC97_EXTENDED_MSTATUS, 4, 1, NULL, 0),
SND_SOC_DAPM_PGA("Mic A PGA", AC97_EXTENDED_MSTATUS, 3, 1, NULL, 0),
SND_SOC_DAPM_PGA("Mic B PGA", AC97_EXTENDED_MSTATUS, 2, 1, NULL, 0),
SND_SOC_DAPM_PGA("Mic A Pre Amp", AC97_EXTENDED_MSTATUS, 1, 1, NULL, 0),
SND_SOC_DAPM_PGA("Mic B Pre Amp", AC97_EXTENDED_MSTATUS, 0, 1, NULL, 0),
SND_SOC_DAPM_MICBIAS("Mic Bias", AC97_EXTENDED_MSTATUS, 14, 1),
SND_SOC_DAPM_OUTPUT("MONO"),
SND_SOC_DAPM_OUTPUT("HPL"),
SND_SOC_DAPM_OUTPUT("HPR"),
SND_SOC_DAPM_OUTPUT("SPKL"),
SND_SOC_DAPM_OUTPUT("SPKR"),
SND_SOC_DAPM_OUTPUT("OUT3"),
SND_SOC_DAPM_OUTPUT("OUT4"),
SND_SOC_DAPM_INPUT("LINEL"),
SND_SOC_DAPM_INPUT("LINER"),
SND_SOC_DAPM_INPUT("MONOIN"),
SND_SOC_DAPM_INPUT("PCBEEP"),
SND_SOC_DAPM_INPUT("MIC1"),
SND_SOC_DAPM_INPUT("MIC2A"),
SND_SOC_DAPM_INPUT("MIC2B"),
SND_SOC_DAPM_VMID("VMID"),
};

static const char *audio_map[][3] = {
	/* left HP mixer */
	{"Left HP Mixer", "PC Beep Playback Switch", "PCBEEP"},
	{"Left HP Mixer", "Voice Playback Switch",   "Voice DAC"},
	{"Left HP Mixer", "Aux Playback Switch",     "Aux DAC"},
	{"Left HP Mixer", "Bypass Playback Switch",  "Left Line In"},
	{"Left HP Mixer", "PCM Playback Switch",     "Left DAC"},
	{"Left HP Mixer", "MonoIn Playback Switch",  "Mono In"},
	{"Left HP Mixer", NULL,  "Capture Headphone Mux"},

	/* right HP mixer */
	{"Right HP Mixer", "PC Beep Playback Switch", "PCBEEP"},
	{"Right HP Mixer", "Voice Playback Switch",   "Voice DAC"},
	{"Right HP Mixer", "Aux Playback Switch",     "Aux DAC"},
	{"Right HP Mixer", "Bypass Playback Switch",  "Right Line In"},
	{"Right HP Mixer", "PCM Playback Switch",     "Right DAC"},
	{"Right HP Mixer", "MonoIn Playback Switch",  "Mono In"},
	{"Right HP Mixer", NULL,  "Capture Headphone Mux"},

	/* virtual mixer - mixes left & right channels for spk and mono */
	{"AC97 Mixer", NULL, "Left DAC"},
	{"AC97 Mixer", NULL, "Right DAC"},
	{"Line Mixer", NULL, "Right Line In"},
	{"Line Mixer", NULL, "Left Line In"},
	{"HP Mixer", NULL, "Left HP Mixer"},
	{"HP Mixer", NULL, "Right HP Mixer"},
	{"Capture Mixer", NULL, "Left Capture Source"},
	{"Capture Mixer", NULL, "Right Capture Source"},

	/* speaker mixer */
	{"Speaker Mixer", "PC Beep Playback Switch", "PCBEEP"},
	{"Speaker Mixer", "Voice Playback Switch",   "Voice DAC"},
	{"Speaker Mixer", "Aux Playback Switch",     "Aux DAC"},
	{"Speaker Mixer", "Bypass Playback Switch",  "Line Mixer"},
	{"Speaker Mixer", "PCM Playback Switch",     "AC97 Mixer"},
	{"Speaker Mixer", "MonoIn Playback Switch",  "Mono In"},

	/* mono mixer */
	{"Mono Mixer", "PC Beep Playback Switch", "PCBEEP"},
	{"Mono Mixer", "Voice Playback Switch",   "Voice DAC"},
	{"Mono Mixer", "Aux Playback Switch",     "Aux DAC"},
	{"Mono Mixer", "Bypass Playback Switch",  "Line Mixer"},
	{"Mono Mixer", "PCM Playback Switch",     "AC97 Mixer"},
	{"Mono Mixer", NULL,  "Capture Mono Mux"},

	/* DAC inv mux 1 */
	{"DAC Inv Mux 1", "Mono", "Mono Mixer"},
	{"DAC Inv Mux 1", "Speaker", "Speaker Mixer"},
	{"DAC Inv Mux 1", "Left Headphone", "Left HP Mixer"},
	{"DAC Inv Mux 1", "Right Headphone", "Right HP Mixer"},
	{"DAC Inv Mux 1", "Headphone Mono", "HP Mixer"},

	/* DAC inv mux 2 */
	{"DAC Inv Mux 2", "Mono", "Mono Mixer"},
	{"DAC Inv Mux 2", "Speaker", "Speaker Mixer"},
	{"DAC Inv Mux 2", "Left Headphone", "Left HP Mixer"},
	{"DAC Inv Mux 2", "Right Headphone", "Right HP Mixer"},
	{"DAC Inv Mux 2", "Headphone Mono", "HP Mixer"},

	/* headphone left mux */
	{"Left Headphone Out Mux", "Headphone", "Left HP Mixer"},

	/* headphone right mux */
	{"Right Headphone Out Mux", "Headphone", "Right HP Mixer"},

	/* speaker left mux */
	{"Left Speaker Out Mux", "Headphone", "Left HP Mixer"},
	{"Left Speaker Out Mux", "Speaker", "Speaker Mixer"},
	{"Left Speaker Out Mux", "Inv", "DAC Inv Mux 1"},

	/* speaker right mux */
	{"Right Speaker Out Mux", "Headphone", "Right HP Mixer"},
	{"Right Speaker Out Mux", "Speaker", "Speaker Mixer"},
	{"Right Speaker Out Mux", "Inv", "DAC Inv Mux 2"},

	/* mono mux */
	{"Mono Out Mux", "Mono", "Mono Mixer"},
	{"Mono Out Mux", "Inv", "DAC Inv Mux 1"},

	/* out 3 mux */
	{"Out 3 Mux", "Inv 1", "DAC Inv Mux 1"},

	/* out 4 mux */
	{"Out 4 Mux", "Inv 2", "DAC Inv Mux 2"},

	/* output pga */
	{"HPL", NULL, "Left Headphone"},
	{"Left Headphone", NULL, "Left Headphone Out Mux"},
	{"HPR", NULL, "Right Headphone"},
	{"Right Headphone", NULL, "Right Headphone Out Mux"},
	{"OUT3", NULL, "Out 3"},
	{"Out 3", NULL, "Out 3 Mux"},
	{"OUT4", NULL, "Out 4"},
	{"Out 4", NULL, "Out 4 Mux"},
	{"SPKL", NULL, "Left Speaker"},
	{"Left Speaker", NULL, "Left Speaker Out Mux"},
	{"SPKR", NULL, "Right Speaker"},
	{"Right Speaker", NULL, "Right Speaker Out Mux"},
	{"MONO", NULL, "Mono Out"},
	{"Mono Out", NULL, "Mono Out Mux"},

	/* input pga */
	{"Left Line In", NULL, "LINEL"},
	{"Right Line In", NULL, "LINER"},
	{"Mono In", NULL, "MONOIN"},
	{"Mic A PGA", NULL, "Mic A Pre Amp"},
	{"Mic B PGA", NULL, "Mic B Pre Amp"},

	/* left capture select */
	{"Left Capture Source", "Mic 1", "Mic A Pre Amp"},
	{"Left Capture Source", "Mic 2", "Mic B Pre Amp"},
	{"Left Capture Source", "Line", "LINEL"},
	{"Left Capture Source", "Mono In", "MONOIN"},
	{"Left Capture Source", "Headphone", "Left HP Mixer"},
	{"Left Capture Source", "Speaker", "Speaker Mixer"},
	{"Left Capture Source", "Mono Out", "Mono Mixer"},

	/* right capture select */
	{"Right Capture Source", "Mic 1", "Mic A Pre Amp"},
	{"Right Capture Source", "Mic 2", "Mic B Pre Amp"},
	{"Right Capture Source", "Line", "LINER"},
	{"Right Capture Source", "Mono In", "MONOIN"},
	{"Right Capture Source", "Headphone", "Right HP Mixer"},
	{"Right Capture Source", "Speaker", "Speaker Mixer"},
	{"Right Capture Source", "Mono Out", "Mono Mixer"},

	/* left ADC */
	{"Left ADC", NULL, "Left Capture Source"},

	/* right ADC */
	{"Right ADC", NULL, "Right Capture Source"},

	/* mic */
	{"Mic A Pre Amp", NULL, "Mic A Source"},
	{"Mic A Source", "Mic 1", "MIC1"},
	{"Mic A Source", "Mic 2 A", "MIC2A"},
	{"Mic A Source", "Mic 2 B", "Mic B Source"},
	{"Mic B Pre Amp", "MPB", "Mic B Source"},
	{"Mic B Source", NULL, "MIC2B"},

	/* headphone capture */
	{"Capture Headphone Mux", "Stereo", "Capture Mixer"},
	{"Capture Headphone Mux", "Left", "Left Capture Source"},
	{"Capture Headphone Mux", "Right", "Right Capture Source"},

	/* mono capture */
	{"Capture Mono Mux", "Stereo", "Capture Mixer"},
	{"Capture Mono Mux", "Left", "Left Capture Source"},
	{"Capture Mono Mux", "Right", "Right Capture Source"},

	{NULL, NULL, NULL},
};

static int wm9713_add_widgets(struct snd_soc_codec *codec, 
	struct snd_soc_machine *machine)
{
	int i;

	for(i = 0; i < ARRAY_SIZE(wm9713_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec, 
			&wm9713_dapm_widgets[i]);
	}

	/* set up audio path audio_map */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_connect_input(machine, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}

	snd_soc_dapm_new_widgets(machine);
	return 0;
}

static unsigned int wm9713_ac97_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u16 val = 0, *cache = codec->reg_cache;

	if (reg == AC97_RESET || reg == AC97_GPIO_STATUS ||
		reg == AC97_VENDOR_ID1 || reg == AC97_VENDOR_ID2 ||
		reg == AC97_CD) {
		codec->mach_read(codec->ac97, (long)&val, reg);
		return val;
	} else {
		reg = reg >> 1;

		if (reg > (ARRAY_SIZE(wm9713_reg)))
			return -EIO;

		return cache[reg];
	}
}

static int wm9713_ac97_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int val)
{
	u16 *cache = codec->reg_cache;
	if (reg < 0x7c)
		codec->mach_write(codec->ac97, val, reg);
	reg = reg >> 1;
	if (reg <= (ARRAY_SIZE(wm9713_reg)))
		cache[reg] = val;

	return 0;
}

struct pll_ {
	unsigned int in_hz;
	unsigned int lf:1; /* allows low frequency use */
	unsigned int sdm:1; /* allows fraction n div */
	unsigned int divsel:1; /* enables input clock div */
	unsigned int divctl:1; /* input clock divider */
	unsigned int n:4;
	unsigned int k;
};

struct pll_ pll[] = {
	{13000000, 0, 1, 0, 0, 7, 0x23f488},
	{2048000,  1, 0, 0, 0, 12, 0x0},
	{4096000,  1, 0, 0, 0, 6, 0x0},
	{12288000, 0, 0, 0, 0, 8, 0x0},
	/* liam - add more entries */
};

static int wm9713_set_pll(struct snd_soc_codec *codec,
	int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct wm9713_priv *wm9713 = codec->private_data;
	int i;
	u16 reg, reg2;

	/* turn PLL off ? */
	if (freq_in == 0 || freq_out == 0) {
		/* disable PLL power and select ext source */
		reg = wm9713_ac97_read(codec, AC97_HANDSET_RATE);
		wm9713_ac97_write(codec, AC97_HANDSET_RATE, reg | 0x0080);
		reg = wm9713_ac97_read(codec, AC97_EXTENDED_MID);
		wm9713_ac97_write(codec, AC97_EXTENDED_MID, reg | 0x0200);
		wm9713->pll_out = 0;
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(pll); i++) {
		if (pll[i].in_hz == freq_in)
			goto found;
	}
	return -EINVAL;

found:
	if (pll[i].sdm == 0) {
		reg = (pll[i].n << 12) | (pll[i].lf << 11) |
			(pll[i].divsel << 9) | (pll[i].divctl << 8);
		wm9713_ac97_write(codec, AC97_LINE1_LEVEL, reg);
	} else {
		/* write the fractional k to the reg 0x46 pages */
		reg2 = (pll[i].n << 12) | (pll[i].lf << 11) | (pll[i].sdm << 10) |
			(pll[i].divsel << 9) | (pll[i].divctl << 8);

		reg = reg2 | (0x5 << 4) | (pll[i].k >> 20); /* K [21:20] */
		wm9713_ac97_write(codec, AC97_LINE1_LEVEL, reg);

		reg = reg2 | (0x4 << 4) | ((pll[i].k >> 16) & 0xf); /* K [19:16] */
		wm9713_ac97_write(codec, AC97_LINE1_LEVEL, reg);

		reg = reg2 | (0x3 << 4) | ((pll[i].k >> 12) & 0xf); /* K [15:12] */
		wm9713_ac97_write(codec, AC97_LINE1_LEVEL, reg);

		reg = reg2 | (0x2 << 4) | ((pll[i].k >> 8) & 0xf); /* K [11:8] */
		wm9713_ac97_write(codec, AC97_LINE1_LEVEL, reg);

		reg = reg2 | (0x1 << 4) | ((pll[i].k >> 4) & 0xf); /* K [7:4] */
		wm9713_ac97_write(codec, AC97_LINE1_LEVEL, reg);

		reg = reg2 | (0x0 << 4) | (pll[i].k & 0xf); /* K [3:0] */
		wm9713_ac97_write(codec, AC97_LINE1_LEVEL, reg);
	}

	/* turn PLL on and select as source */
	reg = wm9713_ac97_read(codec, AC97_EXTENDED_MID);
	wm9713_ac97_write(codec, AC97_EXTENDED_MID, reg & 0xfdff);
	reg = wm9713_ac97_read(codec, AC97_HANDSET_RATE);
	wm9713_ac97_write(codec, AC97_HANDSET_RATE, reg & 0xff7f);
	wm9713->pll_out = freq_out;
	wm9713->pll_in = freq_in;

	/* wait 10ms AC97 link frames for the link to stabilise */
	schedule_timeout_interruptible(msecs_to_jiffies(10));
	return 0;
}

static int wm9713_set_dai_pll(struct snd_soc_dai *codec_dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	return wm9713_set_pll(codec, pll_id, freq_in, freq_out);
}

/*
 * Tristate the PCM DAI lines, tristate can be disabled by calling
 * wm9713_set_dai_fmt()
 */
static int wm9713_set_dai_tristate(struct snd_soc_dai *codec_dai,
	int tristate)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg = wm9713_ac97_read(codec, AC97_CENTER_LFE_MASTER) & 0x9fff;

	if (tristate)
		wm9713_ac97_write(codec, AC97_CENTER_LFE_MASTER, reg);

	return 0;
}

/*
 * Configure WM9713 clock dividers.
 * Voice DAC needs 256 FS
 */
static int wm9713_set_dai_clkdiv(struct snd_soc_dai *codec_dai,
		int div_id, int div)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 reg;

	switch (div_id) {
	case WM9713_PCMCLK_DIV:
		reg = wm9713_ac97_read(codec, AC97_HANDSET_RATE) & 0xf0ff;
		wm9713_ac97_write(codec, AC97_HANDSET_RATE, reg | div);
		break;
	case WM9713_CLKA_MULT:
		reg = wm9713_ac97_read(codec, AC97_HANDSET_RATE) & 0xfffd;
		wm9713_ac97_write(codec, AC97_HANDSET_RATE, reg | div);
		break;
	case WM9713_CLKB_MULT:
		reg = wm9713_ac97_read(codec, AC97_HANDSET_RATE) & 0xfffb;
		wm9713_ac97_write(codec, AC97_HANDSET_RATE, reg | div);
		break;
	case WM9713_HIFI_DIV:
		reg = wm9713_ac97_read(codec, AC97_HANDSET_RATE) & 0x8fff;
		wm9713_ac97_write(codec, AC97_HANDSET_RATE, reg | div);
		break;
	case WM9713_PCMBCLK_DIV:
		reg = wm9713_ac97_read(codec, AC97_CENTER_LFE_MASTER) & 0xf1ff;
		wm9713_ac97_write(codec, AC97_CENTER_LFE_MASTER, reg | div);
		break;
	default:
		return -EINVAL;
	}

	return 0;
};

static int wm9713_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u16 gpio = wm9713_ac97_read(codec, AC97_GPIO_CFG) & 0xffe2;
	u16 reg = 0x8000;

	/* clock masters */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK){
	case SND_SOC_DAIFMT_CBM_CFM:
		reg |= 0x4000;
		gpio |= 0x0008;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		reg |= 0x6000;
		gpio |= 0x000c;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		reg |= 0x0200;
		gpio |= 0x000d;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		gpio |= 0x0009;
		break;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_IB_IF:
		reg |= 0x00c0;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		reg |= 0x0080;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		reg |= 0x0040;
		break;
	}

	/* DAI format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		reg |= 0x0002;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		reg |= 0x0001;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		reg |= 0x0003;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		reg |= 0x0043;
		break;
	}

	wm9713_ac97_write(codec, AC97_GPIO_CFG, gpio);
	wm9713_ac97_write(codec, AC97_CENTER_LFE_MASTER, reg);
	return 0;
}

static int wm9713_pcm_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;
	u16 reg = wm9713_ac97_read(codec, AC97_CENTER_LFE_MASTER) & 0xfff3;

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		reg |= 0x0004;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		reg |= 0x0008;
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		reg |= 0x000c;
		break;
	}

	/* enable PCM interface in master mode */
	wm9713_ac97_write(codec, AC97_CENTER_LFE_MASTER, reg);
	return 0;
}

static void wm9713_voiceshutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_codec *codec = pcm_link->codec;
	u16 status;

	/* Gracefully shut down the voice interface. */
	status = wm9713_ac97_read(codec, AC97_EXTENDED_STATUS) | 0x1000;
	wm9713_ac97_write(codec,AC97_HANDSET_RATE,0x0280);
	schedule_timeout_interruptible(msecs_to_jiffies(1));
	wm9713_ac97_write(codec,AC97_HANDSET_RATE,0x0F80);
	wm9713_ac97_write(codec,AC97_EXTENDED_MID,status);
}

static int ac97_hifi_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_codec *codec = pcm_link->codec;
	int reg;
	u16 vra;

	vra = wm9713_ac97_read(codec, AC97_EXTENDED_STATUS);
	wm9713_ac97_write(codec, AC97_EXTENDED_STATUS, vra | 0x1);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		reg = AC97_PCM_FRONT_DAC_RATE;
	else
		reg = AC97_PCM_LR_ADC_RATE;

	return wm9713_ac97_write(codec, reg, runtime->rate);
}

static int ac97_aux_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_codec *codec = pcm_link->codec;
	u16 vra, xsle;

	vra = wm9713_ac97_read(codec, AC97_EXTENDED_STATUS);
	wm9713_ac97_write(codec, AC97_EXTENDED_STATUS, vra | 0x1);
	xsle = wm9713_ac97_read(codec, AC97_PCI_SID);
	wm9713_ac97_write(codec, AC97_PCI_SID, xsle | 0x8000);

	if (substream->stream != SNDRV_PCM_STREAM_PLAYBACK)
		return -ENODEV;

	return wm9713_ac97_write(codec, AC97_PCM_SURR_DAC_RATE, runtime->rate);
}

static int wm9713_dapm_event(struct snd_soc_codec *codec, int event)
{
	u16 reg;

	switch (event) {
	case SNDRV_CTL_POWER_D0: /* full On */
		/* enable thermal shutdown */
		reg = wm9713_ac97_read(codec, AC97_EXTENDED_MID) & 0x1bff;
		wm9713_ac97_write(codec, AC97_EXTENDED_MID, reg);
		break;
	case SNDRV_CTL_POWER_D1: /* partial On */
	case SNDRV_CTL_POWER_D2: /* partial On */
		break;
	case SNDRV_CTL_POWER_D3hot: /* Off, with power */
		/* enable master bias and vmid */
		reg = wm9713_ac97_read(codec, AC97_EXTENDED_MID) & 0x3bff;
		wm9713_ac97_write(codec, AC97_EXTENDED_MID, reg);
		wm9713_ac97_write(codec, AC97_POWERDOWN, 0x0000);
		break;
	case SNDRV_CTL_POWER_D3cold: /* Off, without power */
		/* disable everything including AC link */
		wm9713_ac97_write(codec, AC97_EXTENDED_MID, 0xffff);
		wm9713_ac97_write(codec, AC97_EXTENDED_MSTATUS, 0xffff);
		wm9713_ac97_write(codec, AC97_POWERDOWN, 0xffff);
		break;
	}
	codec->dapm_state = event;
	return 0;
}

static int wm9713_codec_suspend(struct device *dev, pm_message_t state)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	wm9713_dapm_event(codec, SNDRV_CTL_POWER_D3cold);
	return 0;
}

static int wm9713_codec_resume(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	struct wm9713_priv *wm9713 = codec->private_data;
	int i, ret;
	u16 *cache = codec->reg_cache;

	wm9713_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	/* do we need to re-start the PLL ? */
	if (wm9713->pll_out)
		wm9713_set_pll(codec, 0, wm9713->pll_in, wm9713->pll_out);
#if 0 //lg 
	/* only synchronise the codec if warm reset failed */
	if (ret == 0) {
		for (i = 2; i < ARRAY_SIZE(wm9713_reg) << 1; i+=2) {
			if (i == AC97_POWERDOWN || i == AC97_EXTENDED_MID ||
				i == AC97_EXTENDED_MSTATUS || i > 0x66)
				continue;
			codec->mach_write(codec->ac97, i, cache[i>>1]);
		}
	}
#endif
	if (codec->suspend_dapm_state == SNDRV_CTL_POWER_D0)
		wm9713_dapm_event(codec, SNDRV_CTL_POWER_D0);

	return ret;
}

static int wm9713_codec_io_probe(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	int reg;

	wm9713_dapm_event(codec, SNDRV_CTL_POWER_D3hot);

	/* unmute the adc - move to kcontrol */
	reg = wm9713_ac97_read(codec, AC97_CD) & 0x7fff;
	wm9713_ac97_write(codec, AC97_CD, reg);
	
	wm9713_add_controls(codec, machine->card);
	wm9713_add_widgets(codec, machine);
	
	return 0;
}

static const struct snd_soc_codec_ops wm9713_codec_ops = {
	.dapm_event	= wm9713_dapm_event,
	.read		= wm9713_ac97_read,
	.write		= wm9713_ac97_write,
	.io_probe	= wm9713_codec_io_probe,
};

static int wm9713_codec_probe(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	int ret = 0;

	printk(KERN_INFO "WM9713/WM9714 SoC Audio Codec %s\n", WM9713_VERSION);

	codec->reg_cache = kmemdup(wm9713_reg, sizeof(wm9713_reg), GFP_KERNEL);
	if (codec->reg_cache == NULL)
		return -ENOMEM;
	codec->reg_cache_size = sizeof(wm9713_reg);
	codec->reg_cache_step = 2;

	codec->private_data = kzalloc(sizeof(struct wm9713_priv), GFP_KERNEL);
	if (codec->private_data == NULL) {
		kfree(codec->reg_cache);
		return -ENOMEM;
	}

	codec->owner = THIS_MODULE;
	codec->ops = &wm9713_codec_ops;
	snd_soc_register_codec(codec);
	return ret;
}

static int wm9713_codec_remove(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	kfree(codec->private_data);
	kfree(codec->reg_cache);
	return 0;
}

#define WM9713_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_44100 | \
		SNDRV_PCM_RATE_48000)

#define WM9713_PCM_FORMATS \
	(SNDRV_PCM_FMTBIT_S16  | SNDRV_PCM_FMTBIT_S24)

static const struct snd_soc_pcm_stream wm9713_hifi_dai_playback = {
	.stream_name	= "HiFi Playback",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM9713_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16,
};

static const struct snd_soc_pcm_stream wm9713_hifi_dai_capture = {
	.stream_name	= "HiFi Capture",
	.channels_min	= 1,
	.channels_max	= 2,
	.rates		= WM9713_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16,
};

static const struct snd_soc_pcm_stream wm9713_voice_dai_playback = {
	.stream_name	= "Voice Playback",
	.channels_min	= 1,
	.channels_max	= 1,
	.rates		= WM9713_RATES,
	.formats	= WM9713_PCM_FORMATS,
};

static const struct snd_soc_pcm_stream wm9713_aux_dai_playback = {
	.stream_name	= "Aux Playback",
	.channels_min	= 1,
	.channels_max	= 1,
	.rates		= WM9713_RATES,
	.formats	= SNDRV_PCM_FMTBIT_S16,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops wm9713_hifi_dai_audio_ops = {
	.prepare = ac97_hifi_prepare,
};

/* dai ops, called by machine drivers */
static const struct snd_soc_dai_ops wm9713_voice_dai_ops = {
	.set_clkdiv = wm9713_set_dai_clkdiv,
	.set_pll = wm9713_set_dai_pll,
	.set_fmt = wm9713_set_dai_fmt,
	.set_tristate = wm9713_set_dai_tristate,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops wm9713_voice_dai_audio_ops = {
	.hw_params = wm9713_pcm_hw_params,
	.shutdown = wm9713_voiceshutdown,
};

/* audio ops, called by alsa */
static const struct snd_soc_ops wm9713_aux_dai_audio_ops = {
	.prepare = ac97_aux_prepare,
};

static int wm9713_hifi_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);
		
	dai->audio_ops = &wm9713_hifi_dai_audio_ops;
	dai->capture = &wm9713_hifi_dai_capture;
	dai->playback = &wm9713_hifi_dai_playback;
	snd_soc_register_codec_dai(dai);
	return 0;
}

static int wm9713_voice_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);
	
	dai->ops = &wm9713_voice_dai_ops;
	dai->audio_ops = &wm9713_voice_dai_audio_ops;
	dai->playback = &wm9713_voice_dai_playback;
	snd_soc_register_codec_dai(dai);
	return 0;
}

static int wm9713_aux_dai_probe(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);
		
	dai->audio_ops = &wm9713_aux_dai_audio_ops;
	dai->playback = &wm9713_aux_dai_playback;
	snd_soc_register_codec_dai(dai);
	return 0;
}


const char wm9713_codec[SND_SOC_CODEC_NAME_SIZE] = "wm9713-codec";
EXPORT_SYMBOL_GPL(wm9713_codec);

static struct snd_soc_device_driver wm9713_codec_driver = {
	.type	= SND_SOC_BUS_TYPE_CODEC,
	.driver	= {
		.name 		= wm9713_codec,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm9713_codec_probe,
		.remove		= __devexit_p(wm9713_codec_remove),
		.suspend	= wm9713_codec_suspend,
		.resume		= wm9713_codec_resume,
	},
};

const char wm9713_hifi_dai[SND_SOC_CODEC_NAME_SIZE] = "wm9713-hifi-dai";
EXPORT_SYMBOL_GPL(wm9713_hifi_dai);

static struct snd_soc_device_driver wm9713_hifi_dai_driver = {
	.type	= SND_SOC_BUS_TYPE_DAI,
	.driver	= {
		.name 		= wm9713_hifi_dai,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm9713_hifi_dai_probe,
	},
};

const char wm9713_voice_dai[SND_SOC_CODEC_NAME_SIZE] = "wm9713-voice-dai";
EXPORT_SYMBOL_GPL(wm9713_voice_dai);

static struct snd_soc_device_driver wm9713_voice_dai_driver = {
	.type	= SND_SOC_BUS_TYPE_DAI,
	.driver	= {
		.name 		= wm9713_voice_dai,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm9713_voice_dai_probe,
	},
};

const char wm9713_aux_dai[SND_SOC_CODEC_NAME_SIZE] = "wm9713-aux-dai";
EXPORT_SYMBOL_GPL(wm9713_aux_dai);

static struct snd_soc_device_driver wm9713_aux_dai_driver = {
	.type	= SND_SOC_BUS_TYPE_DAI,
	.driver	= {
		.name 		= wm9713_aux_dai,
		.owner		= THIS_MODULE,
		.bus 		= &asoc_bus_type,
		.probe		= wm9713_aux_dai_probe,
	},
};

static __init int wm9713_init(void)
{
	int ret = 0;
	
	ret = driver_register(&wm9713_codec_driver.driver);
	if (ret < 0)
		return ret;
	ret = driver_register(&wm9713_hifi_dai_driver.driver);
	if (ret < 0)
		goto hifi_err;
	ret = driver_register(&wm9713_voice_dai_driver.driver);
	if (ret < 0)
		goto voice_err;
	ret = driver_register(&wm9713_aux_dai_driver.driver);
	if (ret < 0)
		goto aux_err;
		
	return ret;
aux_err:
	driver_unregister(&wm9713_voice_dai_driver.driver);
voice_err:
	driver_unregister(&wm9713_hifi_dai_driver.driver);
hifi_err:
	driver_unregister(&wm9713_codec_driver.driver);
	return ret;
}

static __exit void wm9713_exit(void)
{
	driver_unregister(&wm9713_aux_dai_driver.driver);
	driver_unregister(&wm9713_voice_dai_driver.driver);
	driver_unregister(&wm9713_hifi_dai_driver.driver);
	driver_unregister(&wm9713_codec_driver.driver);
}

module_init(wm9713_init);
module_exit(wm9713_exit);

MODULE_DESCRIPTION("ASoC WM9713/WM9714 driver");
MODULE_AUTHOR("Liam Girdwood");
MODULE_LICENSE("GPL");
