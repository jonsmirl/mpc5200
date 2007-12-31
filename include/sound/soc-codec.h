/*
 * linux/sound/soc-codec.h -- ALSA SoC Layer
 *
 * Author:		Liam Girdwood
 * Created:		Aug 11th 2005
 * Copyright:	Wolfson Microelectronics. PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_SOC_CODEC_H
#define __LINUX_SND_SOC_CODEC_H

#include <linux/workqueue.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>

/*
 * Convenience kcontrol builders
 */
#define SOC_SINGLE_VALUE(reg,shift,max,invert) ((reg) | ((shift) << 8) |\
	((shift) << 12) | ((max) << 16) | ((invert) << 24))
#define SOC_SINGLE_VALUE_EXT(reg,max,invert) ((reg) | ((max) << 16) |\
	((invert) << 31))
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

/* enumerated kcontrol */
struct soc_enum {
	unsigned short reg;
	unsigned short reg2;
	unsigned char shift_l;
	unsigned char shift_r;
	unsigned int mask;
	const char **texts;
	void *dapm;
};

struct snd_soc_codec;
struct snd_soc_machine;
struct snd_soc_dai;
struct snd_ac97_bus_ops;

/* SoC Audio Codec */
struct snd_soc_codec {
	struct device dev;
	char *name;

	/* runtime */
	unsigned int active;
	enum snd_soc_dapm_bias_power dapm_state;
	enum snd_soc_dapm_bias_power suspend_dapm_state;
	struct snd_ac97 *ac97;  /* for ad-hoc ac97 devices */
	struct mutex mutex;
	struct list_head list;
	struct list_head dai_list;
	struct snd_soc_machine *machine;
	
	int (*set_bias_power)(struct snd_soc_codec *codec, 
		enum snd_soc_dapm_bias_power level);
	
	/* codec probe/remove - this can perform IO */
	int (*init)(struct snd_soc_codec *codec, 
		struct snd_soc_machine *machine);
	void (*exit)(struct snd_soc_codec *codec, 
		struct snd_soc_machine *machine);
	
	/* codec IO */
	
	unsigned int (*codec_read)(struct snd_soc_codec *codec, 
		unsigned int reg);
	int (*codec_write)(struct snd_soc_codec *codec, unsigned int reg, 
		unsigned int value);
	
	void *control_data; /* codec control data */
	/* machine read/write can be used in 2 ways :-
	 *  1. data points to buffer and arg 3 is size (bytes)
	 *  2. data is reg val and arg 3 is register
	 * This depends on codec and machine.
	 */
	int (*machine_write)(void *control_data, long data, int);
	int (*machine_read)(void *control_data, long data, int);
	
	/* register cache */
	void *reg_cache;
	short reg_cache_size;
	short reg_cache_step;

	struct delayed_work delayed_work;
	
	void *private_data;
	void *platform_data;
};
#define to_snd_soc_codec(d) \
	container_of(d, struct snd_soc_codec, dev)
	
static inline void snd_soc_codec_set_io(struct snd_soc_codec *codec,
	int (*machine_read)(void *, long, int), 
	int (*machine_write)(void *, long, int), void *control_data)
{
	mutex_lock(&codec->mutex);
	codec->control_data = control_data;
	codec->machine_read = machine_read;
	codec->machine_write = machine_write;
	mutex_unlock(&codec->mutex);
}

static inline int snd_soc_codec_init(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{
	int ret = 0;
	
	mutex_lock(&codec->mutex);
	
	if (codec->control_data == NULL || codec->codec_write == NULL) {
		ret = -EIO;
		goto out;
	}
		
	if (codec->init)
		ret = codec->init(codec, machine);

out:
	mutex_unlock(&codec->mutex);
	return ret;
}

static inline void snd_soc_codec_exit(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine)
{	
	mutex_lock(&codec->mutex);
	if (codec->exit)
		codec->exit(codec, machine);
	
	mutex_unlock(&codec->mutex);
}
 
/*
 * Controls
 */
struct snd_kcontrol *snd_soc_cnew(const struct snd_kcontrol_new *_template,
	void *data, char *long_name);
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

int snd_soc_codec_create(struct snd_soc_machine *machine,
	const char *codec_id);
int snd_soc_codec_add_dai(struct snd_soc_codec *codec, 
	struct snd_soc_dai *dai, int num);
int snd_soc_register_codec(struct snd_soc_codec *codec);
void snd_soc_unregister_codec(struct snd_soc_codec *codec);

/* codec IO */
#define snd_soc_read(codec, reg) codec->codec_read(codec, reg)
#define snd_soc_write(codec, reg, value) codec->codec_write(codec, reg, value)

/* codec register bit access */
int snd_soc_update_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned short mask, unsigned short value);
int snd_soc_test_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned short mask, unsigned short value);
				
int snd_soc_new_ac97_codec(struct snd_soc_codec *codec,
	struct snd_ac97_bus_ops *ops, struct snd_card *card, 
	int num, int bus_no);

#endif
