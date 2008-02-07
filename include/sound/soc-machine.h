/*
 * linux/sound/soc-machine.h -- ALSA SoC Layer
 *
 * Author:		Liam Girdwood
 * Created:		Aug 11th 2005
 * Copyright:	Wolfson Microelectronics. PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_SOC_MACHINE_H
#define __LINUX_SND_SOC_MACHINE_H

#include <linux/types.h>
#include <linux/workqueue.h>

struct snd_soc_pcm_runtime;
struct snd_soc_machine;
struct snd_soc_ops;
struct snd_soc_pcm_config;

/**
 * snd_soc_machine_create - create new ASoC machine.
 * @name: machine name
 * @parent: parent device
 * @idx: sound card index
 * @xid: sound card ID
 *
 * Creates a new ASoC audio machine device and sound card.
 */
struct snd_soc_machine *snd_soc_machine_create(const char *name,
	struct device *parent, int idx, const char *xid);


/**
 * snd_soc_pcm_create - create new ASoC PCM.
 * @machine: Machine
 *
 * Joins a codec and platform DAI together and creates a ALSA PCM(s).
 */
int snd_soc_pcm_create(struct snd_soc_machine *machine,
	struct snd_soc_pcm_config *config);

/**
 * snd_soc_machine_register - registers ASoC machine .
 * @machine: machine
 *
 * Registers a machine and it's PCMs. This should be called after all
 * codecs, platforms and PCM's have been created.
 */
int snd_soc_machine_register(struct snd_soc_machine *machine);

/**
 * snd_soc_machine_free - free machine.
 * @machine: machine
 *
 * Frees all machine resources. Can be called at any time during machine
 * initialisation process.
 */
void snd_soc_machine_free(struct snd_soc_machine *machine);

/**
 * snd_soc_suspend_pcms - suspends machine pcms.
 * @machine: machine
 * @state: suspend state
 *
 * Suspends all machine pcms.
 */
int snd_soc_suspend_pcms(struct snd_soc_machine *machine, pm_message_t state);

/**
 * snd_soc_resume_pcms - resume machine pcms.
 * @machine: machine
 *
 * Resumes machine pcms.
 */
int snd_soc_resume_pcms(struct snd_soc_machine *machine);

/**
 * snd_soc_get_pcm - get pcm.
 * @machine: machine
 * @pcm_id: pcm ID
 *
 * Gets pcm from ID.
 */
struct snd_soc_pcm_runtime *snd_soc_get_pcm(struct snd_soc_machine *machine,
	const char *pcm_id);

/**
 * snd_soc_get_ac97_ops - get AC97 operations.
 * @machine: machine
 * @dai_id:  ID
 *
 * Gets AC97 operations from Digital Audio Interface.
 */
struct snd_ac97_bus_ops *snd_soc_get_ac97_ops(struct snd_soc_machine *machine,
	int dai_id);

/**
 * snd_soc_get_codec - get codec.
 * @machine: machine
 * @codec_id: codec ID
 *
 * Gets codec from ID.
 */
struct snd_soc_codec *snd_soc_get_codec(struct snd_soc_machine *machine,
	const char *codec_id);

/**
 * snd_soc_get_platform - get platform.
 * @machine: machine
 * @codec_id: platform ID
 *
 * Gets platform from ID.
 */
struct snd_soc_platform * snd_soc_get_platform(struct snd_soc_machine *machine,
	const char *platform_id);

/**
 * snd_soc_codec_set_io - initialise codec IO.
 * @codec: codec
 * @machine_read: read function called by codec.
 * @machine_write: write function called by codec.
 * @control_data: IO control data - usually I2C, SPI, etc pointer
 *
 * Initialises the codec IO system with the machines codec IO mechanism. 
 */
void snd_soc_codec_set_io(struct snd_soc_codec *codec,
	int (*machine_read)(void *, long, int),
	int (*machine_write)(void *, long, int), void *control_data);

/**
 * snd_soc_codec_init - initialises codec
 * @codec: codec
 * @machine: machine
 *
 * Initialises codec hardware. Can perform IO and must only be called after a
 * successful call to snd_soc_codec_set_io(). 
 */
int snd_soc_codec_init(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine);

/**
 * snd_soc_codec_exit - shutsdown codec hw
 * @codec: codec
 * @machine: machine
 *
 * Shutsdown codec hardware.
 */
void snd_soc_codec_exit(struct snd_soc_codec *codec,
	struct snd_soc_machine *machine);

/*
 * PCM configuration information.
 *
 * Defines every ASoC PCM in machine driver i.e. valid Digital Audio Interface
 * with two endpoints (e.g codec and platform).
 */
struct snd_soc_pcm_config {
	const char *name;		/* pcm name */
	const char *codec;		/* codec identification */
	const char *codec_dai;		/* codec dai identification */
	int codec_num;			/* codec number (if > 1 same codec) */
	const char *platform;		/* platform identification */
	const char *cpu_dai;		/* platform dai identification */
	int playback;			/* alsa playback pcms */
	int capture;			/* alsa capture alsa pcms */
	struct snd_soc_ops *ops;	/* machine pcm ops */
};

struct snd_soc_ops {
	/* ALSA audio operations - optional */
	int (*startup)(struct snd_pcm_substream *);
	void (*shutdown)(struct snd_pcm_substream *);
	int (*hw_params)(struct snd_pcm_substream *,
		struct snd_pcm_hw_params *);
	int (*hw_free)(struct snd_pcm_substream *);
	int (*prepare)(struct snd_pcm_substream *);
	int (*trigger)(struct snd_pcm_substream *, int);
};

/*
 * PCM runtime information.
 *
 * Holds information on every ASoC PCM i.e. valid Digital Audio Interface with
 * two endpoints (e.g codec and platform).
 */
struct snd_soc_pcm_runtime {
	const char *name;

	/* runtime status */
	unsigned int playback_active:1;
	unsigned int capture_active:1;
	unsigned int pop_wait:1;

	/* runtime devices */
	struct snd_soc_codec *codec;
	struct snd_soc_dai *cpu_dai;
	struct snd_soc_dai *codec_dai;
	struct snd_soc_platform *platform;
	struct snd_soc_machine *machine;

	/* ALSA audio operations - optional */
	struct snd_soc_ops *ops;
	
	/* DAI pcm */
	struct snd_pcm *pcm;
	
	struct delayed_work delayed_work;
	struct list_head list;
	
	void *private_data;
};


/*
 * ASoC machine
 *
 * The ASoC sound card. This glues the codecs to the platforms and provides
 * machine specific operations per PCM along with any othe machine specifics.
 */
struct snd_soc_machine {
	const char *name;
	const char *longname;

	/* Runtime */
	struct device *dev;
	struct mutex mutex;
	struct snd_card *card;
	int pcms;
	int is_registered;
	int is_probed;

	/* init() and exit() - init is called by snd_soc_machine_register()
	 * whilst exit by snd_soc_machine_free() */
	int (*init)(struct snd_soc_machine *machine);
	void (*exit)(struct snd_soc_machine *machine);
	
	/* bias power level */
	int (*set_bias_level)(struct snd_soc_machine *machine,
		enum snd_soc_dapm_bias_level level);
	enum snd_soc_dapm_bias_level bias_level;

	/* lists of runtime components */
	struct list_head pcm_list;
	struct list_head config_list;
	struct list_head list;

	/* DAPM */
	struct list_head dapm_widgets;
	struct list_head dapm_paths;
	enum snd_soc_dapm_policy policy;
	
	void *private_data;
};

#endif
