/*
 * linux/sound/soc.h -- ALSA SoC Layer
 *
 * Author:		Liam Girdwood
 * Created:		Aug 11th 2005
 * Copyright:	Wolfson Microelectronics. PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_SOC_H
#define __LINUX_SND_SOC_H

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/control.h>
#include <sound/ac97_codec.h>

#define SND_SOC_VERSION "0.20"

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

/*
 * Digital Audio Interface (DAI) types
 */
#define SND_SOC_DAI_AC97	0x1
#define SND_SOC_DAI_I2S		0x2
#define SND_SOC_DAI_PCM		0x4
#define SND_SOC_DAI_AC97_BUS	0x8	/* for custom i.e. non ac97_codec.c */

/*
 * DAI hardware audio formats
 */
#define SND_SOC_DAIFMT_I2S		0	/* I2S mode */
#define SND_SOC_DAIFMT_RIGHT_J	1	/* Right justified mode */
#define SND_SOC_DAIFMT_LEFT_J	2	/* Left Justified mode */
#define SND_SOC_DAIFMT_DSP_A	3	/* L data msb after FRM or LRC */
#define SND_SOC_DAIFMT_DSP_B	4	/* L data msb during FRM or LRC */
#define SND_SOC_DAIFMT_AC97		5	/* AC97 */

#define SND_SOC_DAIFMT_MSB 	SND_SOC_DAIFMT_LEFT_J
#define SND_SOC_DAIFMT_LSB	SND_SOC_DAIFMT_RIGHT_J

/*
 * DAI Gating
 */
#define SND_SOC_DAIFMT_CONT		(0 << 4)	/* continuous clock */
#define SND_SOC_DAIFMT_GATED		(1 << 4)	/* clock is gated when not Tx/Rx */

/*
 * DAI Sync
 * Synchronous LR (Left Right) clocks and Frame signals.
 */
#define SND_SOC_DAIFMT_SYNC		(0 << 5)	/* Tx FRM = Rx FRM */ 
#define SND_SOC_DAIFMT_ASYNC		(1 << 5)	/* Tx FRM ~ Rx FRM */ 

/*
 * TDM
 */
#define SND_SOC_DAIFMT_TDM		(1 << 6)

/*
 * DAI hardware signal inversions
 */
#define SND_SOC_DAIFMT_NB_NF		(0 << 8)	/* normal bit clock + frame */
#define SND_SOC_DAIFMT_NB_IF		(1 << 8)	/* normal bclk + inv frm */
#define SND_SOC_DAIFMT_IB_NF		(2 << 8)	/* invert bclk + nor frm */
#define SND_SOC_DAIFMT_IB_IF		(3 << 8)	/* invert bclk + frm */

/*
 * DAI hardware clock masters
 * This is wrt the codec, the inverse is true for the interface
 * i.e. if the codec is clk and frm master then the interface is
 * clk and frame slave.
 */
#define SND_SOC_DAIFMT_CBM_CFM	(0 << 12) /* codec clk & frm master */
#define SND_SOC_DAIFMT_CBS_CFM	(1 << 12) /* codec clk slave & frm master */
#define SND_SOC_DAIFMT_CBM_CFS	(2 << 12) /* codec clk master & frame slave */
#define SND_SOC_DAIFMT_CBS_CFS	(3 << 12) /* codec clk & frm slave */

#define SND_SOC_DAIFMT_FORMAT_MASK		0x000f
#define SND_SOC_DAIFMT_CLOCK_MASK		0x00f0
#define SND_SOC_DAIFMT_INV_MASK			0x0f00
#define SND_SOC_DAIFMT_MASTER_MASK		0xf000

/*
 * Master Clock Directions
 */
#define SND_SOC_CLOCK_IN	0
#define SND_SOC_CLOCK_OUT	1

/*
 * AC97 codec ID's bitmask
 */
#define SND_SOC_DAI_AC97_ID0	(1 << 0)
#define SND_SOC_DAI_AC97_ID1	(1 << 1)
#define SND_SOC_DAI_AC97_ID2	(1 << 2)
#define SND_SOC_DAI_AC97_ID3	(1 << 3)

#define SND_SOC_MACHINE_NAME_SIZE	32
#define SND_SOC_CODEC_NAME_SIZE	32
#define SND_SOC_PLATFORM_NAME_SIZE	32
#define SND_SOC_DAI_NAME_SIZE	32
#define SND_SOC_PCM_NAME_SIZE	32

struct snd_soc_pcm_stream;
struct snd_soc_ops;
struct soc_enum;
struct snd_soc_machine;
struct snd_soc_dai;
struct snd_soc_codec;
struct snd_soc_platform;
struct snd_soc_pcm_link;
struct snd_soc_pcm_link_ops;

/* ASoC audio device types */
enum snd_soc_bus_device_type {
	SND_SOC_BUS_TYPE_PCM = 0,
	SND_SOC_BUS_TYPE_DMA,
	SND_SOC_BUS_TYPE_CODEC,
	SND_SOC_BUS_TYPE_DAI,
};

extern struct bus_type asoc_bus_type;

/* pcm <-> DAI connect */
int snd_soc_pcm_new(struct snd_soc_pcm_link *pcm_link, int playback, 
	int capture);
void snd_soc_machine_free(struct snd_soc_machine *machine);
int snd_soc_new_card(struct snd_soc_machine *machine, int num_pcm_links, 
	int idx, const char *xid);
int snd_soc_register_card(struct snd_soc_machine *machine);
struct snd_soc_pcm_link *snd_soc_pcm_link_new(
	struct snd_soc_machine *machine, const char *name,
	const struct snd_soc_pcm_link_ops *link_ops, 
	const char *platform_id, const char *codec_id, 
	const char *codec_dai_id, const char *cpu_dai_id);
int snd_soc_pcm_link_attach(struct snd_soc_pcm_link *pcm_link);

int snd_soc_register_codec_dai(struct snd_soc_dai *dai);
int snd_soc_register_cpu_dai(struct snd_soc_dai *dai);
int snd_soc_register_codec(struct snd_soc_codec *codec);
int snd_soc_register_platform(struct snd_soc_platform *platform);

/* set runtime hw params */
int snd_soc_set_runtime_hwparams(struct snd_pcm_substream *substream,
	const struct snd_pcm_hardware *hw);

/* codec IO */
#define snd_soc_read(codec, reg) codec->ops->read(codec, reg)
#define snd_soc_write(codec, reg, value) codec->ops->write(codec, reg, value)

/* codec register bit access */
int snd_soc_update_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned short mask, unsigned short value);
int snd_soc_test_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned short mask, unsigned short value);

int snd_soc_new_ac97_codec(struct snd_soc_pcm_link *pcm_link,
	struct snd_ac97_bus_ops *ops, int num);
void snd_soc_free_ac97_codec(struct snd_soc_pcm_link *pcm_link);

/* suspend and resume */
int snd_soc_suspend(struct snd_soc_machine *machine, pm_message_t state);
int snd_soc_resume(struct snd_soc_machine *machine);

/*
 *Controls
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

/* SoC PCM stream information */
struct snd_soc_pcm_stream {
	char *stream_name;
	u64 formats;			/* SNDRV_PCM_FMTBIT_* */
	unsigned int rates;		/* SNDRV_PCM_RATE_* */
	unsigned int rate_min;		/* min rate */
	unsigned int rate_max;		/* max rate */
	unsigned int channels_min;	/* min channels */
	unsigned int channels_max;	/* max channels */
	unsigned int active:1;		/* stream is in use */
};

/* SoC audio ops */
struct snd_soc_ops {
	int (*startup)(struct snd_pcm_substream *);
	void (*shutdown)(struct snd_pcm_substream *);
	int (*hw_params)(struct snd_pcm_substream *, struct snd_pcm_hw_params *);
	int (*hw_free)(struct snd_pcm_substream *);
	int (*prepare)(struct snd_pcm_substream *);
	int (*trigger)(struct snd_pcm_substream *, int);
};

/* ASoC DAI ops */
struct snd_soc_dai_ops {
	/* DAI clocking configuration */
	int (*set_sysclk)(struct snd_soc_dai *dai, int clk_id, 
		unsigned int freq, int dir);
	int (*set_clkdiv)(struct snd_soc_dai *dai,
		int div_id, int div);
	int (*set_pll)(struct snd_soc_dai *dai,
		int pll_id, unsigned int freq_in, unsigned int freq_out);

	/* DAI format configuration */
	int (*set_fmt)(struct snd_soc_dai *dai,
		unsigned int fmt);
	int (*set_tdm_slot)(struct snd_soc_dai *dai,
		unsigned int mask, int slots);
	int (*set_tristate)(struct snd_soc_dai *dai, int tristate);
	
	int (*digital_mute)(struct snd_soc_dai *dai, int mute);
};

struct snd_soc_dai {
	char name[SND_SOC_DAI_NAME_SIZE];
	struct device dev;
	struct module *owner;
	struct list_head list;
	struct list_head codec_list;
	enum snd_soc_bus_device_type type;
	int id;

	/* ops */
	const struct snd_soc_ops *audio_ops;
	const struct snd_soc_dai_ops *ops;
	struct snd_ac97_bus_ops *ac97_ops;
	
	/* hw capabilities */
	const struct snd_soc_pcm_stream *capture;
	const struct snd_soc_pcm_stream *playback;

	/* runtime info */
	struct snd_pcm_runtime *runtime;
	struct snd_soc_codec *codec; /* codec dai only */
	int capture_active;
	int playback_active;
	int active;
	int pop_wait;
	void *dma_data; /* cpu dai only */
	
	void *private_data;
};
#define to_snd_soc_dai(d) container_of(d, struct snd_soc_dai, dev)

/* ASoC codec ops */
struct snd_soc_codec_ops {
	int (*dapm_event)(struct snd_soc_codec *codec, int event);
	
	unsigned int (*read)(struct snd_soc_codec *codec, unsigned int reg);
	int (*write)(struct snd_soc_codec *codec, unsigned int reg, 
		unsigned int value);
	
	/* codec probe/remove - this can perform IO */
	int (*io_probe)(struct snd_soc_codec *codec, 
		struct snd_soc_machine *machine);
	int (*io_remove)(struct snd_soc_codec *codec, 
		struct snd_soc_machine *machine);
};


/* SoC Audio Codec */
struct snd_soc_codec {
	char name[SND_SOC_CODEC_NAME_SIZE];
	struct device dev;
	struct module *owner;
	struct mutex mutex;
	struct list_head list;
	struct list_head dai_list;
	enum snd_soc_bus_device_type type;

	/* runtime */
	unsigned int active;
	unsigned int dapm_state;
	unsigned int suspend_dapm_state;
	struct snd_ac97 *ac97;  /* for ad-hoc ac97 devices */

	/* ops */
	const struct snd_soc_codec_ops *ops;

	/* codec IO */
	void *control_data; /* codec control data */
	/* machine read/write can be used in 2 ways :-
	 *  1. data points to buffer and arg 3 is size (bytes)
	 *  2. data is reg val and arg 3 is register
	 * This depends on codec and machine.
	 */
	int (*mach_write)(void *control_data, long data, int);
	int (*mach_read)(void *control_data, long data, int);
	
	/* register cache */
	void *reg_cache;
	short reg_cache_size;
	short reg_cache_step;

	struct delayed_work delayed_work;
	
	void *private_data;
};
#define to_snd_soc_codec(d) container_of(d, struct snd_soc_codec, dev)

/* ASoC platform ops */
struct snd_soc_platform_ops {
	/* pcm creation and destruction */
	int (*pcm_new)(struct snd_soc_platform *platform, 
		struct snd_card *card, int playback, int capture, 
		struct snd_pcm *pcm);
	void (*pcm_free)(struct snd_pcm *pcm);
};

/* SoC platform interface */
struct snd_soc_platform {
	char name[SND_SOC_PLATFORM_NAME_SIZE];
	struct device dev;
	struct module *owner;
	struct list_head list;
	enum snd_soc_bus_device_type type;

	/* platform ops */
	const struct snd_pcm_ops *pcm_ops;
	const struct snd_soc_platform_ops *platform_ops;
	
	void *private_data;
};
#define to_snd_soc_platform(d) container_of(d, struct snd_soc_platform, dev)

/* ASoC pcm device ops */
struct snd_soc_pcm_link_ops {
	int (*new)(struct snd_soc_pcm_link *pcm_link);
	int (*free)(struct snd_soc_pcm_link *pcm_link);
};

/* ASoC PCM, glues a codec, cpu DAI, codec DAI and platform together */
struct snd_soc_pcm_link {
	char name[SND_SOC_PCM_NAME_SIZE];
	struct list_head active_list;
	struct list_head all_list;
	struct delayed_work delayed_work;
	enum snd_soc_bus_device_type type;
	int id;
	int probed;
	
	/* runtime devices */
	const char *codec_id;
	struct snd_soc_codec *codec;
	const char *cpu_dai_id;
	struct snd_soc_dai *cpu_dai;
	const char *codec_dai_id;
	struct snd_soc_dai *codec_dai;
	const char *platform_id;
	struct snd_soc_platform *platform;
	struct snd_soc_machine *machine;

	/* dai stream operations */
	const struct snd_soc_ops *audio_ops;
	const struct snd_soc_pcm_link_ops *link_ops;
	
	/* DAI pcm */
	struct snd_pcm *pcm;
	
	void *private_data;
};

struct snd_soc_device_driver {
	struct device_driver driver;
	enum snd_soc_bus_device_type type;
};
#define to_snd_soc_drv(d) container_of(d, struct snd_soc_device_driver, driver)

/* ASoC machine ops */
struct snd_soc_machine_ops {
	int (*mach_probe)(struct snd_soc_machine *machine);
	int (*mach_remove)(struct snd_soc_machine *machine);
	int (*dapm_event)(struct snd_soc_machine *machine, int event);
	int (*suspend_pre) (struct snd_soc_machine *machine, 
		pm_message_t state);
	int (*suspend_post) (struct snd_soc_machine *machine, 
		pm_message_t state);
	int (*resume_pre) (struct snd_soc_machine *machine);
	int (*resume_post) (struct snd_soc_machine *machine);
};

/* SoC machine */
struct snd_soc_machine {
	const char *name;
	const char *longname;
	struct module *owner;
	struct mutex mutex;
	struct platform_device *pdev;
	struct snd_card *card;
	const struct snd_soc_machine_ops *ops;
	
	/* dapm */
	struct list_head dapm_widgets;
	struct list_head dapm_paths;
	unsigned int dapm_state;
	
	/* component pcm link devs */
	struct list_head active_list;
	int pcm_links;
	int pcm_links_total;
	int disconnect;

	void *private_data;
};

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

#endif
