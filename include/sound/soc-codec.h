/*
 * linux/sound/soc-codec.h -- ALSA SoC codec interface
 *
 * Author:		Liam Girdwood
 * Created:		Aug 11th 2005
 * Copyright:	Wolfson Microelectronics. PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * ALSA SoC codec driver interface.
 */

#ifndef __LINUX_SND_SOC_CODEC_H
#define __LINUX_SND_SOC_CODEC_H

#include <linux/workqueue.h>
#include <sound/soc-dapm.h>

#define SOC_SINGLE_VALUE(reg,shift,max,invert) ((reg) | ((shift) << 8) |\
	((shift) << 12) | ((max) << 16) | ((invert) << 24))
#define SOC_SINGLE_VALUE_EXT(reg,max,invert) ((reg) | ((max) << 16) |\
	((invert) << 31))

/*
 * Convenience kcontrol builders.
 *
 * @SINGLE:       Mono kcontrol.
 * @SINGLE_TLV:   Mono Table Lookup Value kcontrol.
 * @DOUBLE:       Stereo kcontrol.
 * @DOUBLE_R:     Stereo kcontrol that spans 2 codec registers.
 * @DOUBLE_TLV:   Stereo Table Lookup Value kcontrol.
 * @DOUBLE_R_TLV: Stereo Table Lookup Value kcontrol that spans 2 registers.
 * @ENUM_SINGLE:  Mono enumerated kcontrol.
 * @ENUM_DOUBLE:  Stereo enumerated kcontrol.
 * @SINGLE_EXT:   Mono external kcontrol.
 * @DOUBLE_EXT:   Stereo external kcontrol.
 * @ENUM_EXT:     Mono external enumerated kcontrol.
 */

#define SOC_SINGLE(xname, reg, shift, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = snd_soc_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }
#define SOC_SINGLE_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw,\
	.put = snd_soc_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }
#define SOC_DOUBLE(xname, reg, shift_left, shift_right, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw, \
	.put = snd_soc_put_volsw, \
	.private_value = (reg) | ((shift_left) << 8) | \
		((shift_right) << 12) | ((max) << 16) | ((invert) << 24) }
#define SOC_DOUBLE_R(xname, reg_left, reg_right, shift, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw_2r, \
	.get = snd_soc_get_volsw_2r, .put = snd_soc_put_volsw_2r, \
	.private_value = (reg_left) | ((shift) << 8)  | \
		((max) << 12) | ((invert) << 20) | ((reg_right) << 24) }
#define SOC_DOUBLE_TLV(xname, reg, shift_left, shift_right, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_get_volsw, \
	.put = snd_soc_put_volsw, \
	.private_value = (reg) | ((shift_left) << 8) | \
		((shift_right) << 12) | ((max) << 16) | ((invert) << 24) }
#define SOC_DOUBLE_R_TLV(xname, reg_left, reg_right, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname),\
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw_2r, \
	.get = snd_soc_get_volsw_2r, .put = snd_soc_put_volsw_2r, \
	.private_value = (reg_left) | ((shift) << 8)  | \
		((max) << 12) | ((invert) << 20) | ((reg_right) << 24) }
#define SOC_ENUM_DOUBLE(xreg, xshift_l, xshift_r, xmask, xtexts) \
{	.reg = xreg, .shift_l = xshift_l, .shift_r = xshift_r, \
	.mask = xmask, .texts = xtexts }
#define SOC_ENUM_SINGLE(xreg, xshift, xmask, xtexts) \
	SOC_ENUM_DOUBLE(xreg, xshift, xshift, xmask, xtexts)
#define SOC_ENUM_SINGLE_EXT(xmask, xtexts) \
{	.mask = xmask, .texts = xtexts }
#define SOC_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname,\
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_get_enum_double, .put = snd_soc_put_enum_double, \
	.private_value = (unsigned long)&xenum }
#define SOC_SINGLE_EXT(xname, xreg, xshift, xmask, xinvert,\
	 xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = SOC_SINGLE_VALUE(xreg, xshift, xmask, xinvert) }
#define SOC_SINGLE_BOOL_EXT(xname, xdata, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_bool_ext, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = xdata }
#define SOC_ENUM_EXT(xname, xenum, xhandler_get, xhandler_put) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_ext, \
	.get = xhandler_get, .put = xhandler_put, \
	.private_value = (unsigned long)&xenum }

struct snd_soc_codec;
struct snd_soc_card;
struct snd_soc_dai;
struct snd_ac97_bus_ops;
struct snd_kcontrol;
struct snd_kcontrol_new;
struct snd_soc_dai_new;
struct snd_soc_codec_new;

/*
 * KControls.
 *
 * Called by the convenience macros to get/put/info kcontrols.
 */
int snd_soc_info_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo);
int snd_soc_info_enum_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo);
int snd_soc_get_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_put_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_info_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo);
int snd_soc_info_volsw_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo);
int snd_soc_info_bool_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo);
int snd_soc_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_info_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo);
int snd_soc_get_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_put_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);

/* control creation */
struct snd_kcontrol *snd_soc_cnew(const struct snd_kcontrol_new *_template,
	void *data, char *long_name);

int snd_soc_add_new_controls(struct snd_soc_card *soc_card,
	const struct snd_kcontrol_new *_template, void *data, int num);

/* codec creation and registration */
struct snd_soc_codec *snd_soc_new_codec(
	struct snd_soc_codec_new *template, const char *cache);

int snd_soc_register_codec(struct snd_soc_codec *codec, struct device *dev);

void snd_soc_unregister_codec(struct snd_soc_codec *codec);

int snd_soc_new_ac97_codec(struct snd_soc_codec *codec,
	struct snd_ac97_bus_ops *ops, struct snd_card *card,
	int num, int bus_no);

/* codec dia registration */
struct snd_soc_dai *snd_soc_register_codec_dai(
	struct snd_soc_dai_new *template, struct device *dev);

void snd_soc_unregister_codec_dai(struct snd_soc_dai *dai);

/*
 * Enumerated kcontrol
 */
struct soc_enum {
	unsigned short reg;
	unsigned short reg2;
	unsigned char shift_l;
	unsigned char shift_r;
	unsigned int mask;
	const char **texts;
	void *dapm;
};

/*
 * New codec template - used for conveniently creating new codec drivers.
 * The fields have the same meaning as snd_soc_codec below.
 */
struct snd_soc_codec_new {
	const char *name;
	short reg_cache_size;
	short reg_cache_step;

	int (*set_bias_level)(struct snd_soc_codec *codec,
		enum snd_soc_dapm_bias_level level);

	int (*init)(struct snd_soc_codec *codec,
		struct snd_soc_card *soc_card);
	void (*exit)(struct snd_soc_codec *codec,
		struct snd_soc_card *soc_card);

	unsigned int (*codec_read)(struct snd_soc_codec *codec,
		unsigned int reg);
	int (*codec_write)(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value);
};

/*
 * SoC Audio Codec.
 *
 * Describes a rutime ASoC audio codec.
 */
struct snd_soc_codec {

	/*
	 * Codec runtime
	 */
	const char *name;
	struct device *dev;
	unsigned int active;		/* is codec active */
	struct delayed_work delayed_work;
	struct snd_ac97 *ac97;		/* for ad-hoc ac97 devices */
	struct mutex mutex;		/* codec mutex */
	struct list_head list;
	struct list_head dai_list;	/* list of DAI's */
	struct snd_soc_card *soc_card;	/* parent soc_card */
	int num;

	/*  Codec power control and state. Optional. */
	enum snd_soc_dapm_bias_level bias_level;
	enum snd_soc_dapm_bias_level suspend_bias_level;
	int (*set_bias_level)(struct snd_soc_codec *codec,
		enum snd_soc_dapm_bias_level level);

	/*
	 * Initialisation and cleanup - Optional, both can perform IO.
	 * Normally used to power up/down codec power domain (bias) and do any
	 * codec init/exit IO. Called by snd_soc_card_config_codec() and
	 * snd_soc_codec_exit() in soc card driver.
	 */
	int (*init)(struct snd_soc_codec *codec,
		struct snd_soc_card *soc_card);
	void (*exit)(struct snd_soc_codec *codec,
		struct snd_soc_card *soc_card);

	/*
	 * Codec control IO.
	 *
	 * All codec IO is performed by calling codec_read() and codec_write().
	 * codec_read/write() formats the IO data for the codec and then calls
	 * the soc_phys_read and soc_phys_write respectively to physically
	 * perform the IO operation. IOW, codec read/write only does formatting
	 * whilst soc card read/write does the physical IO.
	 *
	 * The soc_phys_read and soc_phys_write functions can either wrap the
	 * kernel I2C, SPI read and write functions or do custom IO.
	 */
	unsigned int (*codec_read)(struct snd_soc_codec *codec,
		unsigned int reg);
	int (*codec_write)(struct snd_soc_codec *codec, unsigned int reg,
		unsigned int value);
	int (*soc_phys_write)(void *control_data, long data, int bytes);
	unsigned int (*soc_phys_read)(void *control_data, int length);
	void *control_data;			/* codec control data */


	/*
	 * Register cacheing. Codec registers can be cached to siginificantly
	 * speed up IO operations over slow busses. e.g. I2C, SPI
	 */
	void *reg_cache;			/* cache data */
	short reg_cache_size;			/* number of registers */
	short reg_cache_step;			/* register size (bytes) */

	void *private_data;			/* core doesnt touch this */
	void *platform_data;			/* or this */
};

/* Codec register read and write */
static inline unsigned int snd_soc_read(struct snd_soc_codec *codec, int reg)
{
	return codec->codec_read(codec, reg);
}

static inline int snd_soc_write(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	return codec->codec_write(codec, reg, value);
}

int snd_soc_update_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned short mask, unsigned short value);

int snd_soc_test_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned short mask, unsigned short value);

#endif
