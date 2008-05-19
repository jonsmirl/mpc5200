/*
 * soc-core.c  --  ALSA SoC Audio Layer
 *
 * Copyright 2005 Wolfson Microelectronics PLC.
 * Copyright 2005 Openedhand Ltd.
 *
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *         with code, comments and ideas from :-
 *         Richard Purdie <richard@openedhand.com>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    12th Aug 2005   Initial version.
 *    25th Oct 2005   Working Codec, Interface and Platform registration.
 *
 *  TODO:
 *   o Add hw rules to enforce rates, etc.
 *   o More testing with other codecs/soc_cards.
 *   o Add more codecs and platforms to ensure good API coverage.
 *   o Support TDM on PCM and I2S
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>
#include <sound/ac97_codec.h>

/* debug */
#define SOC_DEBUG 0
#if SOC_DEBUG
#define dbg(format, arg...) printk(format, ## arg)
#else
#define dbg(format, arg...)
#endif

/* dapm sys fs - used by the core */
int snd_soc_dapm_sys_add(struct snd_soc_card *soc_card);
void snd_soc_dapm_free(struct snd_soc_card *soc_card);

static DEFINE_MUTEX(pcm_mutex);
static DEFINE_MUTEX(io_mutex);
static DEFINE_MUTEX(client_mutex);
static DECLARE_WAIT_QUEUE_HEAD(soc_pm_waitq);
static LIST_HEAD(platform_list);
static LIST_HEAD(codec_list);
static LIST_HEAD(cpu_dai_list);
static LIST_HEAD(codec_dai_list);
static LIST_HEAD(soc_card_list);

struct soc_pcm_config {
	struct snd_soc_pcm_config *config;
	struct list_head list;
	struct snd_soc_codec *codec;
	struct snd_soc_dai *codec_dai;
	struct snd_soc_platform *platform;
	struct snd_soc_dai *cpu_dai;
	int ready;
};

/*
 * This is a timeout to do a DAPM powerdown after a stream is closed().
 * It can be used to eliminate pops between different playback streams, e.g.
 * between two audio tracks.
 */
static int pmdown_time = 5000;
module_param(pmdown_time, int, 0);
MODULE_PARM_DESC(pmdown_time, "DAPM stream powerdown time (msecs)");

/*
 * This function forces any delayed work to be queued and run.
 */
static int run_delayed_work(struct delayed_work *dwork)
{
	int ret;

	/* cancel any work waiting to be queued. */
	ret = cancel_delayed_work(dwork);

	/* if there was any work waiting then we run it now and
	 * wait for it's completion */
	if (ret) {
		schedule_delayed_work(dwork, 0);
		flush_scheduled_work();
	}
	return ret;
}

#ifdef CONFIG_SND_SOC_AC97_BUS
/* unregister ac97 codec */
static int soc_ac97_dev_unregister(struct snd_soc_codec *codec)
{
	struct snd_ac97 *ac97 = codec->ac97;

	BUG_ON(!codec->ac97);

	if (ac97->dev.bus) {
		device_unregister(&ac97->dev);
		ac97->dev.bus = 0;
	}
	return 0;
}

static void soc_ac97_device_release(struct device *dev)
{
	/* A SoC AC97 device has no resources to free, it only talks
	 * through bus operations.
	 */
}

static int soc_ac97_dev_register(struct snd_soc_codec *codec)
{
	struct snd_ac97 *ac97 = codec->ac97;
	int err;

	BUG_ON(!codec->ac97);

	if (ac97->dev.bus)
		return 0;

	ac97->dev.bus = &ac97_bus_type;
	ac97->dev.parent = codec->dev;
	ac97->dev.release = soc_ac97_device_release;

	snprintf(ac97->dev.bus_id, BUS_ID_SIZE, "%d-%d:%s",
		 ac97->num, ac97->addr, codec->name);
	err = device_register(&ac97->dev);
	if (err < 0) {
		snd_printk(KERN_ERR "Can't register ac97 bus\n");
		ac97->dev.bus = NULL;
		return err;
	}
	return 0;
}

static int soc_ac97_pcm_create(struct snd_soc_card *soc_card)
{
	struct snd_soc_pcm_runtime *pcm_runtime;
	int ret = 0;

	list_for_each_entry(pcm_runtime, &soc_card->pcm_list, list) {
		if (pcm_runtime->cpu_dai->ac97_control) {
			ret = soc_ac97_dev_register(pcm_runtime->codec);
			if (ret < 0) {
				printk(KERN_ERR "asoc: AC97 device register"
					" failed\n");

				return ret;
			}
		}
	}
	return ret;
}
#endif

/*
 * Called by ALSA when a PCM substream is opened, the runtime->hw record is
 * then initialized and any private data can be allocated. This also calls
 * startup for the cpu DAI, platform, soc_card and codec DAI.
 */
static int soc_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_platform *platform = pcm_runtime->platform;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	struct snd_soc_codec *codec = pcm_runtime->codec;
	int ret = 0;

	mutex_lock(&pcm_mutex);

	/* startup the audio subsystem */
	if (cpu_dai->ops->startup) {
		ret = cpu_dai->ops->startup(substream, cpu_dai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't open interface %s\n",
				cpu_dai->name);
			goto out;
		}
	}

	if (platform->pcm_ops->open) {
		ret = platform->pcm_ops->open(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't open platform %s\n", platform->name);
			goto platform_err;
		}
	}

	if (codec_dai->ops->startup) {
		ret = codec_dai->ops->startup(substream, codec_dai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't open codec %s\n",
				codec_dai->name);
			goto codec_dai_err;
		}
	}

	if (pcm_runtime->ops->startup) {
		ret = pcm_runtime->ops->startup(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: %s startup failed\n", pcm_runtime->name);
			goto pcm_runtime_err;
		}
	}

	/* Check that the codec and cpu DAIs are compatible */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		struct snd_soc_dai_caps *codec_caps = codec_dai->playback;
		struct snd_soc_dai_caps *cpu_caps = cpu_dai->playback;

		runtime->hw.rate_min =
			max(codec_caps->rate_min, cpu_caps->rate_min);
		runtime->hw.rate_max =
			min(codec_caps->rate_max, cpu_caps->rate_max);
		runtime->hw.channels_min =
			max(codec_caps->channels_min, cpu_caps->channels_min);
		runtime->hw.channels_max =
			min(codec_caps->channels_max, cpu_caps->channels_max);
		runtime->hw.formats = codec_caps->formats & cpu_caps->formats;
		runtime->hw.rates = codec_caps->rates & cpu_caps->rates;
	} else {
		struct snd_soc_dai_caps *codec_caps = codec_dai->capture;
		struct snd_soc_dai_caps *cpu_caps = cpu_dai->capture;

		runtime->hw.rate_min =
			max(codec_caps->rate_min, cpu_caps->rate_min);
		runtime->hw.rate_max =
			min(codec_caps->rate_max, cpu_caps->rate_max);
		runtime->hw.channels_min =
			max(codec_caps->channels_min, cpu_caps->channels_min);
		runtime->hw.channels_max =
			min(codec_caps->channels_max, cpu_caps->channels_max);
		runtime->hw.formats = codec_caps->formats & cpu_caps->formats;
		runtime->hw.rates = codec_caps->rates & cpu_caps->rates;
	}

	ret = -EINVAL;
	snd_pcm_limit_hw_rates(runtime);
	if (!runtime->hw.rates) {
		printk(KERN_ERR "asoc: %s <-> %s No matching rates\n",
			codec_dai->name, cpu_dai->name);
		goto pcm_runtime_err;
	}
	if (!runtime->hw.formats) {
		printk(KERN_ERR "asoc: %s <-> %s No matching formats\n",
			codec_dai->name, cpu_dai->name);
		goto pcm_runtime_err;
	}
	if (!runtime->hw.channels_min || !runtime->hw.channels_max) {
		printk(KERN_ERR "asoc: %s <-> %s No matching channels\n",
			codec_dai->name, cpu_dai->name);
		goto pcm_runtime_err;
	}

	dbg("asoc: %s <-> %s info:\n",codec_dai->name, cpu_dai->name);
	dbg("asoc: rate mask 0x%x\n", runtime->hw.rates);
	dbg("asoc: min ch %d max ch %d\n", runtime->hw.channels_min,
		runtime->hw.channels_max);
	dbg("asoc: min rate %d max rate %d\n", runtime->hw.rate_min,
		runtime->hw.rate_max);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pcm_runtime->playback_active = 1;
	else
		pcm_runtime->capture_active = 1;
	codec->active++;

	mutex_unlock(&pcm_mutex);
	return 0;

pcm_runtime_err:
	if (pcm_runtime->ops->shutdown)
		pcm_runtime->ops->shutdown(substream);

codec_dai_err:
	if (platform->pcm_ops->close)
		platform->pcm_ops->close(substream);

platform_err:
	if (cpu_dai->ops->shutdown)
		cpu_dai->ops->shutdown(substream, cpu_dai);
out:
	mutex_unlock(&pcm_mutex);
	return ret;
}

/*
 * Power down the audio subsytem pmdown_time msecs after close is called.
 * This is to ensure there are no pops or clicks in between any music tracks
 * due to DAPM power cycling.
 */
static void close_delayed_work(struct work_struct *work)
{
	struct snd_soc_pcm_runtime *pcm_runtime =
		container_of(work, struct snd_soc_pcm_runtime,
			delayed_work.work);
	struct snd_soc_codec *codec = pcm_runtime->codec;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	struct snd_soc_card *soc_card = pcm_runtime->soc_card;

	mutex_lock(&pcm_mutex);

	dbg("pop wq checking: %s status: %s waiting: %s\n",
		codec_dai->name,
		pcm_runtime->playback_active ? "active" : "inactive",
		pcm_runtime->pop_wait ? "yes" : "no");

	/* are we waiting on this codec DAI stream */
	if (pcm_runtime->pop_wait == 1) {

		/* power down the codec to D1 if no longer active */
		if (codec->active == 0) {
			dbg("pop wq D1 %s %s\n", codec->name,
				codec_dai->playback->stream_name);
			snd_soc_dapm_set_bias(pcm_runtime,
				SND_SOC_BIAS_PREPARE);
		}

		pcm_runtime->pop_wait = 0;
		snd_soc_dapm_stream_event(soc_card,
			codec_dai->playback->stream_name,
			SND_SOC_DAPM_STREAM_STOP);

		/* power down the codec power domain if no longer active */
		if (codec->active == 0) {
			dbg("pop wq D3 %s %s\n", codec->name,
				codec_dai->playback->stream_name);
			snd_soc_dapm_set_bias(pcm_runtime,
				SND_SOC_BIAS_STANDBY);
		}
	}
	mutex_unlock(&pcm_mutex);
}

/*
 * Called by ALSA when a PCM substream is closed. Private data can be
 * freed here. The cpu DAI, codec DAI, soc_card and platform are also
 * shutdown.
 */
static int soc_codec_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_platform *platform = pcm_runtime->platform;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	struct snd_soc_codec *codec = pcm_runtime->codec;
	struct snd_soc_card *soc_card = pcm_runtime->soc_card;

	mutex_lock(&pcm_mutex);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pcm_runtime->playback_active = 0;
	else
		pcm_runtime->capture_active = 0;
	codec->active--;

	if (cpu_dai->ops->shutdown)
		cpu_dai->ops->shutdown(substream, cpu_dai);

	if (codec_dai->ops->shutdown)
		codec_dai->ops->shutdown(substream, codec_dai);

	if (pcm_runtime->ops->shutdown)
		pcm_runtime->ops->shutdown(substream);

	if (platform->pcm_ops->close)
		platform->pcm_ops->close(substream);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* start delayed pop wq here for playback streams */
		pcm_runtime->pop_wait = 1;
		schedule_delayed_work(&pcm_runtime->delayed_work,
			msecs_to_jiffies(pmdown_time));
	} else {
		/* capture streams can be powered down now */
		snd_soc_dapm_stream_event(soc_card,
			codec_dai->capture->stream_name,
			SND_SOC_DAPM_STREAM_STOP);

		if (codec->active == 0 && pcm_runtime->pop_wait == 0)
			snd_soc_dapm_set_bias(pcm_runtime,
				SND_SOC_BIAS_STANDBY);
	}

	mutex_unlock(&pcm_mutex);
	return 0;
}

/*
 * Called by ALSA when the PCM substream is prepared, can set format, sample
 * rate, etc.  This function is non atomic and can be called multiple times,
 * it can refer to the runtime info.
 */
static int soc_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_platform *platform = pcm_runtime->platform;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	struct snd_soc_codec *codec = pcm_runtime->codec;
	struct snd_soc_card *soc_card = pcm_runtime->soc_card;
	int ret = 0;

	mutex_lock(&pcm_mutex);

	if (pcm_runtime->ops->prepare) {
		ret = pcm_runtime->ops->prepare(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: soc_card prepare error\n");
			goto out;
		}
	}

	if (platform->pcm_ops->prepare) {
		ret = platform->pcm_ops->prepare(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: platform prepare error\n");
			goto out;
		}
	}

	if (codec_dai->ops->prepare) {
		ret = codec_dai->ops->prepare(substream, codec_dai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: codec DAI prepare error\n");
			goto out;
		}
	}

	if (cpu_dai->ops->prepare) {
		ret = cpu_dai->ops->prepare(substream, cpu_dai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: cpu DAI prepare error\n");
			goto out;
		}
	}

	/* we only want to start a DAPM playback stream if we are not waiting
	 * on an existing one stopping */
	if (pcm_runtime->pop_wait) {
		/* we are waiting for the delayed work to start */
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
				snd_soc_dapm_stream_event(soc_card,
					codec_dai->capture->stream_name,
					SND_SOC_DAPM_STREAM_START);
		else {
			pcm_runtime->pop_wait = 0;
			cancel_delayed_work(&pcm_runtime->delayed_work);
			if (codec_dai->ops->digital_mute)
				codec_dai->ops->digital_mute(codec_dai, 0);
		}
	} else {
		/* no delayed work - do we need to power up codec */
		if (codec->bias_level != SND_SOC_BIAS_ON) {

			snd_soc_dapm_set_bias(pcm_runtime,
				SND_SOC_BIAS_PREPARE);

			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				snd_soc_dapm_stream_event(soc_card,
					codec_dai->playback->stream_name,
					SND_SOC_DAPM_STREAM_START);
			else
				snd_soc_dapm_stream_event(soc_card,
					codec_dai->capture->stream_name,
					SND_SOC_DAPM_STREAM_START);

			snd_soc_dapm_set_bias(pcm_runtime, SND_SOC_BIAS_ON);
			if (codec_dai->ops->digital_mute)
				codec_dai->ops->digital_mute(codec_dai, 0);

		} else {
			/* codec already powered - power on widgets */
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				snd_soc_dapm_stream_event(soc_card,
					codec_dai->playback->stream_name,
					SND_SOC_DAPM_STREAM_START);
			else
				snd_soc_dapm_stream_event(soc_card,
					codec_dai->capture->stream_name,
					SND_SOC_DAPM_STREAM_START);
			if (codec_dai->ops->digital_mute)
				codec_dai->ops->digital_mute(codec_dai, 0);
		}
	}

out:
	mutex_unlock(&pcm_mutex);
	return ret;
}

/*
 * Called by ALSA when the hardware params are set by application. This
 * function can also be called multiple times and can allocate buffers
 * (using snd_pcm_lib_* ). It's non-atomic.
 */
static int soc_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_platform *platform = pcm_runtime->platform;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	int ret = 0;

	mutex_lock(&pcm_mutex);

	if (pcm_runtime->ops->hw_params) {
		ret = pcm_runtime->ops->hw_params(substream, params);
		if (ret < 0) {
			printk(KERN_ERR "asoc: soc_card hw_params failed\n");
			goto out;
		}
	}

	if (codec_dai->ops->hw_params) {
		ret = codec_dai->ops->hw_params(substream, params, codec_dai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't set codec %s hw params\n",
				codec_dai->name);
			goto codec_err;
		}
	}

	if (cpu_dai->ops->hw_params) {
		ret = cpu_dai->ops->hw_params(substream, params, cpu_dai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't set interface %s "
				"hw params\n", cpu_dai->name);
			goto interface_err;
		}
	}

	if (platform->pcm_ops->hw_params) {
		ret = platform->pcm_ops->hw_params(substream, params);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't set platform %s "
				"hw params\n", platform->name);
			goto platform_err;
		}
	}

out:
	mutex_unlock(&pcm_mutex);
	return ret;

platform_err:
	if (cpu_dai->ops->hw_free)
		cpu_dai->ops->hw_free(substream, cpu_dai);

interface_err:
	if (codec_dai->ops->hw_free)
		codec_dai->ops->hw_free(substream, codec_dai);

codec_err:
	if(pcm_runtime->ops->hw_free)
		pcm_runtime->ops->hw_free(substream);

	mutex_unlock(&pcm_mutex);
	return ret;
}

/*
 * Free's resources allocated by hw_params, can be called multiple times
 */
static int soc_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_platform *platform = pcm_runtime->platform;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	struct snd_soc_codec *codec = pcm_runtime->codec;

	mutex_lock(&pcm_mutex);

	/* apply codec digital mute */
	if (!codec->active && codec_dai->ops->digital_mute)
		codec_dai->ops->digital_mute(codec_dai, 1);

	/* free any soc_card hw params */
	if (pcm_runtime->ops->hw_free)
		pcm_runtime->ops->hw_free(substream);

	/* free any DMA resources */
	if (platform->pcm_ops->hw_free)
		platform->pcm_ops->hw_free(substream);

	/* now free hw params for the DAI's  */
	if (codec_dai->ops->hw_free)
		codec_dai->ops->hw_free(substream, codec_dai);

	if (cpu_dai->ops->hw_free)
		cpu_dai->ops->hw_free(substream, cpu_dai);

	mutex_unlock(&pcm_mutex);
	return 0;
}

static int soc_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_platform *platform = pcm_runtime->platform;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	int ret;

	if (codec_dai->ops->trigger) {
		ret = codec_dai->ops->trigger(substream, cmd, codec_dai);
		if (ret < 0)
			return ret;
	}

	if (platform->pcm_ops->trigger) {
		ret = platform->pcm_ops->trigger(substream, cmd);
		if (ret < 0)
			return ret;
	}

	if (cpu_dai->ops->trigger) {
		ret = cpu_dai->ops->trigger(substream, cmd, cpu_dai);
		if (ret < 0)
			return ret;
	}
	return 0;
}

/* ASoC PCM operations */
static struct snd_pcm_ops soc_pcm_ops = {
	.open		= soc_pcm_open,
	.close		= soc_codec_close,
	.hw_params	= soc_pcm_hw_params,
	.hw_free	= soc_pcm_hw_free,
	.prepare	= soc_pcm_prepare,
	.trigger	= soc_pcm_trigger,
};

#ifdef CONFIG_PM
/**
 * snd_soc_card_suspend - suspend soc_card
 * @soc_card: soc soc_card
 *
 * Mutes, then suspends all soc_card PCM's. NOTE: soc_card driver will still
 * have to suspend codecs/platform.
 */
int snd_soc_card_suspend_pcms(struct snd_soc_card *soc_card, pm_message_t state)
{
	struct snd_soc_dai *codec_dai;
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_runtime *pcm_runtime;
	char *stream;

	/* mute any active DAC's */
	list_for_each_entry(pcm_runtime, &soc_card->pcm_list, list) {
		codec_dai = pcm_runtime->codec_dai;
		if (codec_dai->ops->digital_mute &&
			pcm_runtime->playback_active)
			codec_dai->ops->digital_mute(codec_dai, 1);
	}

	snd_power_change_state(soc_card->card, SNDRV_CTL_POWER_D3cold);

	/* suspend all pcm's */
	list_for_each_entry(pcm_runtime, &soc_card->pcm_list, list)
		snd_pcm_suspend_all(pcm_runtime->pcm);

	/* close any waiting streams and save state */
	list_for_each_entry(pcm_runtime, &soc_card->pcm_list, list) {
		codec = pcm_runtime->codec;
		run_delayed_work(&pcm_runtime->delayed_work);
		codec->suspend_bias_level = codec->bias_level;
	}

	/* DAPM power down */
	list_for_each_entry(pcm_runtime, &soc_card->pcm_list, list) {
		codec_dai = pcm_runtime->codec_dai;
		stream = codec_dai->playback->stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(soc_card, stream,
				SND_SOC_DAPM_STREAM_SUSPEND);
		if (!codec_dai->capture)
			continue;
		stream = codec_dai->capture->stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(soc_card, stream,
				SND_SOC_DAPM_STREAM_SUSPEND);
	}

	return 0;
}

/**
 * snd_soc_card_resume - resume soc_card
 * @soc_card: soc soc_card
 *
 * Resumes and unmutes all soc_card PCM's. NOTE: soc_card driver will still
 * have to resume codecs/platform.
 */
int snd_soc_card_resume_pcms(struct snd_soc_card *soc_card)
{
	struct snd_soc_dai *codec_dai;
	struct snd_soc_pcm_runtime *pcm_runtime;
	char *stream;

	list_for_each_entry(pcm_runtime, &soc_card->pcm_list, list) {
		codec_dai = pcm_runtime->codec_dai;
		stream = codec_dai->playback->stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(soc_card, stream,
				SND_SOC_DAPM_STREAM_RESUME);
		if (!codec_dai->capture)
			continue;
		stream = codec_dai->capture->stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(soc_card, stream,
				SND_SOC_DAPM_STREAM_RESUME);
	}

	/* unmute any active DAC's */
	list_for_each_entry(pcm_runtime, &soc_card->pcm_list, list) {
		codec_dai = pcm_runtime->codec_dai;
		if (codec_dai->ops->digital_mute &&
			pcm_runtime->playback_active)
			codec_dai->ops->digital_mute(codec_dai, 0);
	}

	snd_power_change_state(soc_card->card, SNDRV_CTL_POWER_D3hot);
	return 0;
}

#else
int snd_soc_card_suspend_pcms(struct snd_soc_card *soc_card, pm_message_t state)
{
	return 0;
}

int snd_soc_card_resume_pcms(struct snd_soc_card *soc_card)
{
	return 0;
}
#endif
EXPORT_SYMBOL_GPL(snd_soc_card_suspend_pcms);
EXPORT_SYMBOL_GPL(snd_soc_card_resume_pcms);

/* Creates a new PCM based upon a pcm_config in the soc_card driver */
static int soc_create_pcm(struct snd_soc_card *soc_card,
	struct soc_pcm_config *pcm_config)
{
	struct snd_soc_pcm_runtime *pcm_runtime;
	struct snd_soc_dai *codec_dai = NULL, *cpu_dai = NULL;
	struct snd_soc_codec *codec = NULL;
	struct snd_soc_platform *platform = NULL;
	struct snd_pcm *pcm;
	struct snd_soc_pcm_config *config = pcm_config->config;
	int ret = 0;

	/* do we need to find the codec */
	if (pcm_config->codec)
		goto codec_dai;

	/* yes, then check codec list */
	list_for_each_entry(codec, &codec_list, list) {
		if (!strcmp(config->codec, codec->name) &&
		    codec->num == config->codec_num &&
		    try_module_get(codec->dev->driver->owner)) {
			dbg("ASoC %s %s: Match for %s.%d %s.%d\n",
			    soc_card->name, config->name,
			    config->codec, config->codec_num,
			    codec->name, codec->num);

			pcm_config->codec = codec;
			goto codec_dai;
		} else {
			dbg("ASoC %s %s: No match for %s.%d %s.%d\n",
			    soc_card->name, config->name,
			    config->codec, config->codec_num,
			    codec->name, codec->num);
		}
	}

codec_dai:
	/* do we need to find the codec dai */
	if (pcm_config->codec_dai)
		goto platform;

	/* yes, then check the codec_dai list */
	list_for_each_entry(codec_dai, &codec_dai_list, list) {
		if (!strcmp(config->codec_dai, codec_dai->name) &&
		    try_module_get(codec_dai->dev->driver->owner)) {
			dbg("ASoC %s %s: Match for %s %s\n",
			    soc_card->name, config->name,
			    config->codec_dai, codec_dai->name);

			pcm_config->codec_dai = codec_dai;
			goto platform;
		} else {
			dbg("ASoC %s %s: No match for %s %s\n",
			    soc_card->name, config->name,
			    config->codec_dai, codec_dai->name);
		}
	}

platform:
	/* do we need to find the platform */
	if (pcm_config->platform)
		goto cpu_dai;

	/* yes, then check the platform list */
	list_for_each_entry(platform, &platform_list, list) {
		if (!strcmp(config->platform, platform->name) &&
		    try_module_get(platform->dev->driver->owner)) {
			dbg("ASoC %s %s: Match for %s %s\n",
			    soc_card->name, config->name,
			    config->platform, platform->name);

			pcm_config->platform = platform;
			goto cpu_dai;
		} else {
			dbg("ASoC %s %s: No match for %s %s\n",
			    soc_card->name, config->name,
			    config->platform, platform->name);
		}
	}

cpu_dai:
	/* do we need to find the platform dai */
	if (pcm_config->cpu_dai)
		goto check;

	/* yes, then check the codec_dai list */
	list_for_each_entry(cpu_dai, &cpu_dai_list, list) {
		if (!strcmp(config->cpu_dai, cpu_dai->name) &&
		    try_module_get(cpu_dai->dev->driver->owner)) {
			dbg("ASoC %s %s: Match for %s %s\n",
			    soc_card->name, config->name,
			    config->cpu_dai, cpu_dai->name);

			pcm_config->cpu_dai = cpu_dai;
			goto check;
		} else {
			dbg("ASoC %s %s: No match for %s %s\n",
			    soc_card->name, config->name,
			    config->cpu_dai, cpu_dai->name);
		}
	}

check:
	/* we should have all the pcm components at this point */
	if (!pcm_config->codec || !pcm_config->codec_dai ||
	    !pcm_config->platform || !pcm_config->cpu_dai) {
		dbg("ASoC %s %s: incomplete codec %d (DAI %d) platform"
		    " %d (DAI %d)\n",
		    soc_card->name, config->name,
		    !pcm_config->codec == 0, !pcm_config->codec_dai == 0,
		    !pcm_config->platform == 0, !pcm_config->cpu_dai == 0);
		return 0;
	}

	/* now create and register the pcm */
	pcm_runtime = kzalloc(sizeof(*pcm_runtime), GFP_KERNEL);
	if (!pcm_runtime)
		return -ENOMEM;

	INIT_DELAYED_WORK(&pcm_runtime->delayed_work, close_delayed_work);
	pcm_runtime->codec_dai = pcm_config->codec_dai;
	pcm_runtime->cpu_dai = pcm_config->cpu_dai;
	pcm_runtime->soc_card = soc_card;
	pcm_runtime->name = config->name;
	codec = pcm_runtime->codec = pcm_config->codec;
	platform = pcm_runtime->platform = pcm_config->platform;
	pcm_config->cpu_dai->platform = pcm_config->platform;
	pcm_config->codec_dai->codec = pcm_config->codec;

	ret = snd_pcm_new(soc_card->card, (char*)pcm_runtime->name,
		soc_card->pcms++, config->playback, config->capture, &pcm);
	if (ret < 0) {
		printk(KERN_ERR "asoc: can't create pcm for codec %s\n",
			codec->name);
		goto err;
	}

	pcm_runtime->pcm = pcm;
	pcm_runtime->ops = config->ops;
	pcm->private_data = pcm_runtime;
	platform = pcm_config->platform;
	soc_pcm_ops.mmap = platform->pcm_ops->mmap;

	soc_pcm_ops.pointer = platform->pcm_ops->pointer;
	soc_pcm_ops.ioctl = platform->pcm_ops->ioctl;
	soc_pcm_ops.copy = platform->pcm_ops->copy;
	soc_pcm_ops.silence = platform->pcm_ops->silence;
	soc_pcm_ops.ack = platform->pcm_ops->ack;
	soc_pcm_ops.page = platform->pcm_ops->page;

	if (config->playback)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &soc_pcm_ops);

	if (config->capture)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &soc_pcm_ops);

	ret = platform->pcm_new(platform, soc_card->card,
		config->playback, config->capture, pcm);
	if (ret < 0) {
		printk(KERN_ERR "asoc: platform pcm constructor failed\n");
		goto err;
	}

	pcm->private_free = platform->pcm_free;
	printk(KERN_INFO "asoc: %s <-> %s mapping ok\n",
		pcm_runtime->codec_dai->name, pcm_runtime->cpu_dai->name);
	list_add(&pcm_runtime->list, &soc_card->pcm_list);
	return 1;
err:
	module_put(pcm_runtime->codec->dev->driver->owner);
	module_put(pcm_runtime->codec_dai->dev->driver->owner);
	module_put(pcm_runtime->platform->dev->driver->owner);
	module_put(pcm_runtime->cpu_dai->dev->driver->owner);
	kfree(pcm_runtime);
	return ret;
}

static int soc_match_pcm(struct snd_soc_card *soc_card)
{
	struct soc_pcm_config *config;
	int ret, err;

	/* scan and build each pcm configuration */
	list_for_each_entry(config, &soc_card->config_list, list) {
		config->ready = soc_create_pcm(soc_card, config);
		if (config->ready < 0)
			return config->ready;
	}

	/* are all pcm's ready - then register soc_card */
	list_for_each_entry(config, &soc_card->config_list, list) {
		if (config->ready == 0)
			return 0;
	}

	if (soc_card->init) {
		ret = soc_card->init(soc_card);
		if (ret < 0)
			goto out;
	}

#ifdef CONFIG_SND_SOC_AC97_BUS
	ret = soc_ac97_pcm_create(soc_card);
	if (ret < 0) {
		printk(KERN_ERR "asoc: failed to register AC97 device\n");
		goto out;
	}
#endif

	mutex_lock(&soc_card->mutex);
	snprintf(soc_card->card->shortname, sizeof(soc_card->card->shortname),
		 "%s", soc_card->name);
	snprintf(soc_card->card->longname, sizeof(soc_card->card->longname),
		 "%s (%s)", soc_card->name, soc_card->longname);

	ret = snd_card_register(soc_card->card);
	if (ret < 0) {
		printk(KERN_ERR "asoc: failed to register soundcard for "
			"codec %s\n", soc_card->name);
		goto out_mutex;
	}

	err = snd_soc_dapm_sys_add(soc_card);
	if (err < 0)
		printk(KERN_WARNING "asoc: failed to add dapm sysfs entries\n");
	soc_card->is_probed = 1;

out_mutex:
	mutex_unlock(&soc_card->mutex);
out:
	return 0;
}

static void soc_match_soc_card_pcms(void)
{
	struct snd_soc_card *soc_card;

	mutex_lock(&client_mutex);
	list_for_each_entry(soc_card, &soc_card_list, list) {
		if (soc_card->is_registered && !soc_card->is_probed)
			soc_match_pcm(soc_card);
	}
	mutex_unlock(&client_mutex);
}

/* codec register dump */
static ssize_t codec_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct snd_soc_codec *codec = dev_get_drvdata(dev);
	int i, step = 1, count = 0;

	if (!codec->reg_cache_size)
		return 0;

	if (codec->reg_cache_step)
		step = codec->reg_cache_step;

	count += sprintf(buf, "%s registers\n", codec->name);
	for(i = 0; i < codec->reg_cache_size; i += step)
		count += sprintf(buf + count, "%2x: %4x\n", i,
			codec->codec_read(codec, i));

	return count;
}
static DEVICE_ATTR(codec_reg, 0444, codec_reg_show, NULL);

static void soc_codec_exit(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card)
{
	device_remove_file(codec->dev, &dev_attr_codec_reg);
	if (codec->exit)
		codec->exit(codec, soc_card);
}

static int soc_ac97_write(void *control_data, long val, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	ac97->bus->ops->write(ac97, reg, val);
	return 0;
}

static unsigned int soc_ac97_read(void *control_data, int reg)
{
	struct snd_ac97 *ac97 = (struct snd_ac97 *)control_data;
	return ac97->bus->ops->read(ac97, reg);
}

/**
 * snd_soc_new_ac97_codec - create new AC97 codec.
 * @codec: codec
 * @ops: AC97 bus operations
 * @card: ALSA sound card.
 * @num: codec number.
 * @bus_no: AC97 bus number.
 *
 * Creates a new AC97 codec and initialises AC97 codec resources for use by
 * ad-hoc AC97 devices.
 */
int snd_soc_new_ac97_codec(struct snd_soc_codec *codec,
	struct snd_ac97_bus_ops *ops, struct snd_card *card,
	int num, int bus_no)
{
	struct snd_ac97 *ac97;

	snd_assert(codec != NULL, return -EINVAL);
	snd_assert(ops != NULL, return -EINVAL);

	mutex_lock(&codec->mutex);

	ac97 = kzalloc(sizeof(struct snd_ac97), GFP_KERNEL);
	if (ac97 == NULL) {
		mutex_unlock(&codec->mutex);
		return -ENOMEM;
	}

	ac97->bus = kzalloc(sizeof(struct snd_ac97_bus), GFP_KERNEL);
	if (ac97->bus == NULL) {
		kfree(ac97);
		mutex_unlock(&codec->mutex);
		return -ENOMEM;
	}

	ac97->bus->ops = ops;
	ac97->num = num;
	ac97->bus->card = card;
	ac97->bus->clock = 48000;
	ac97->bus->num = bus_no;
	spin_lock_init(&ac97->bus->bus_lock);
	codec->ac97 = ac97;

	snd_soc_card_config_codec(codec, soc_ac97_read, soc_ac97_write,
			     codec->ac97);

	mutex_unlock(&codec->mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_new_ac97_codec);

/**
 * snd_soc_free_ac97_codec - free AC97 codec device
 * @codec: audio codec
 *
 * Frees AC97 codec device resources.
 */
void snd_soc_free_ac97_codec(struct snd_soc_codec *codec)
{
	struct snd_ac97 *ac97 = codec->ac97;

	kfree(ac97->bus);
	kfree(ac97);
	codec->ac97 = NULL;
}

/**
 * snd_soc_update_bits - update codec register bits
 * @codec: audio codec
 * @reg: codec register
 * @mask: register mask
 * @value: new value
 *
 * Writes new register value.
 *
 * Returns 1 for change else 0.
 */
int snd_soc_update_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned short mask, unsigned short value)
{
	int change;
	unsigned short old, new;

	mutex_lock(&io_mutex);
	old = snd_soc_read(codec, reg);
	new = (old & ~mask) | value;
	change = old != new;
	if (change)
		snd_soc_write(codec, reg, new);

	mutex_unlock(&io_mutex);
	return change;
}
EXPORT_SYMBOL_GPL(snd_soc_update_bits);

/**
 * snd_soc_test_bits - test register for change
 * @codec: audio codec
 * @reg: codec register
 * @mask: register mask
 * @value: new value
 *
 * Tests a register with a new value and checks if the new value is
 * different from the old value.
 *
 * Returns 1 for change else 0.
 */
int snd_soc_test_bits(struct snd_soc_codec *codec, unsigned short reg,
				unsigned short mask, unsigned short value)
{
	int change;
	unsigned short old, new;

	mutex_lock(&io_mutex);
	old = snd_soc_read(codec, reg);
	new = (old & ~mask) | value;
	change = old != new;
	mutex_unlock(&io_mutex);

	return change;
}
EXPORT_SYMBOL_GPL(snd_soc_test_bits);

/**
 * snd_soc_set_runtime_hwparams - set the runtime hardware parameters
 * @substream: the pcm substream
 * @hw: the hardware parameters
 *
 * Sets the substream runtime hardware parameters.
 */
int snd_soc_set_runtime_hwparams(struct snd_pcm_substream *substream,
	const struct snd_pcm_hardware *hw)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	runtime->hw.info = hw->info;
	runtime->hw.formats = hw->formats;
	runtime->hw.period_bytes_min = hw->period_bytes_min;
	runtime->hw.period_bytes_max = hw->period_bytes_max;
	runtime->hw.periods_min = hw->periods_min;
	runtime->hw.periods_max = hw->periods_max;
	runtime->hw.buffer_bytes_max = hw->buffer_bytes_max;
	runtime->hw.fifo_size = hw->fifo_size;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_set_runtime_hwparams);

/**
 * snd_soc_cnew - create new control
 * @_template: control template
 * @data: control private data
 * @lnng_name: control long name
 *
 * Create a new mixer control from a template control.
 *
 * Returns 0 for success, else error.
 */
struct snd_kcontrol *snd_soc_cnew(const struct snd_kcontrol_new *_template,
	void *data, char *long_name)
{
	struct snd_kcontrol_new template;

	memcpy(&template, _template, sizeof(template));
	if (long_name)
		template.name = long_name;
	template.index = 0;

	return snd_ctl_new1(&template, data);
}
EXPORT_SYMBOL_GPL(snd_soc_cnew);

/**
 * snd_soc_add_new_controls - create and add new controls
 * @_template: control template
 * @data: control private data
 * @num: number of controls
 *
 * Create new mixer controls from template controls and add to
 * the sound card.
 *
 * Returns 0 for success, else error. On error all resources can be freed
 * with a call to snd_soc_card_free().
 */
int snd_soc_add_new_controls(struct snd_soc_card *soc_card,
	const struct snd_kcontrol_new *_template, void *data, int num)
{
	struct snd_kcontrol *control;
	int i, ret;

	for (i = 0; i < num; i++) {
		control = snd_soc_cnew(_template++, data, NULL);
		if (control == NULL)
			return -ENOMEM;

		ret = snd_ctl_add(soc_card->card, control);
		if (ret < 0) {
			kfree(control);
			return ret;
		}
	}
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_add_new_controls);

/**
 * snd_soc_info_enum_double - enumerated double mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a double enumerated
 * mixer control.
 *
 * Returns 0 for success.
 */
int snd_soc_info_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = e->shift_l == e->shift_r ? 1 : 2;
	uinfo->value.enumerated.items = e->mask;

	if (uinfo->value.enumerated.item > e->mask - 1)
		uinfo->value.enumerated.item = e->mask - 1;
	strcpy(uinfo->value.enumerated.name,
		e->texts[uinfo->value.enumerated.item]);
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_enum_double);

/**
 * snd_soc_get_enum_double - enumerated double mixer get callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to get the value of a double enumerated mixer.
 *
 * Returns 0 for success.
 */
int snd_soc_get_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned short val, bitmask;

	for (bitmask = 1; bitmask < e->mask; bitmask <<= 1)
		;
	val = snd_soc_read(codec, e->reg);
	ucontrol->value.enumerated.item[0] = (val >> e->shift_l) & (bitmask - 1);
	if (e->shift_l != e->shift_r)
		ucontrol->value.enumerated.item[1] =
			(val >> e->shift_r) & (bitmask - 1);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_get_enum_double);

/**
 * snd_soc_put_enum_double - enumerated double mixer put callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to set the value of a double enumerated mixer.
 *
 * Returns 0 for success.
 */
int snd_soc_put_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;
	unsigned short val;
	unsigned short mask, bitmask;

	for (bitmask = 1; bitmask < e->mask; bitmask <<= 1)
		;
	if (ucontrol->value.enumerated.item[0] > e->mask - 1)
		return -EINVAL;
	val = ucontrol->value.enumerated.item[0] << e->shift_l;
	mask = (bitmask - 1) << e->shift_l;
	if (e->shift_l != e->shift_r) {
		if (ucontrol->value.enumerated.item[1] > e->mask - 1)
			return -EINVAL;
		val |= ucontrol->value.enumerated.item[1] << e->shift_r;
		mask |= (bitmask - 1) << e->shift_r;
	}

	return snd_soc_update_bits(codec, e->reg, mask, val);
}
EXPORT_SYMBOL_GPL(snd_soc_put_enum_double);

/**
 * snd_soc_info_enum_ext - external enumerated single mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about an external enumerated
 * single mixer.
 *
 * Returns 0 for success.
 */
int snd_soc_info_enum_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	struct soc_enum *e = (struct soc_enum *)kcontrol->private_value;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_ENUMERATED;
	uinfo->count = 1;
	uinfo->value.enumerated.items = e->mask;

	if (uinfo->value.enumerated.item > e->mask - 1)
		uinfo->value.enumerated.item = e->mask - 1;
	strcpy(uinfo->value.enumerated.name,
		e->texts[uinfo->value.enumerated.item]);
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_enum_ext);

/**
 * snd_soc_info_volsw_ext - external single mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a single external mixer control.
 *
 * Returns 0 for success.
 */
int snd_soc_info_volsw_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	int max = kcontrol->private_value;

	if (max == 1)
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_volsw_ext);

/**
 * snd_soc_info_bool_ext - external single boolean mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a single boolean external mixer control.
 *
 * Returns 0 for success.
 */
int snd_soc_info_bool_ext(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	uinfo->count = 1;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = 1;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_bool_ext);

/**
 * snd_soc_info_volsw - single mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a single mixer control.
 *
 * Returns 0 for success.
 */
int snd_soc_info_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	int max = (kcontrol->private_value >> 16) & 0xff;
	int shift = (kcontrol->private_value >> 8) & 0x0f;
	int rshift = (kcontrol->private_value >> 12) & 0x0f;

	if (max == 1)
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = shift == rshift ? 1 : 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_volsw);

/**
 * snd_soc_get_volsw - single mixer get callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to get the value of a single mixer control.
 *
 * Returns 0 for success.
 */
int snd_soc_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & 0xff;
	int shift = (kcontrol->private_value >> 8) & 0x0f;
	int rshift = (kcontrol->private_value >> 12) & 0x0f;
	int max = (kcontrol->private_value >> 16) & 0xff;
	int mask = (1 << fls(max)) - 1;
	int invert = (kcontrol->private_value >> 24) & 0x01;

	ucontrol->value.integer.value[0] =
		(snd_soc_read(codec, reg) >> shift) & mask;
	if (shift != rshift)
		ucontrol->value.integer.value[1] =
			(snd_soc_read(codec, reg) >> rshift) & mask;
	if (invert) {
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];
		if (shift != rshift)
			ucontrol->value.integer.value[1] =
				max - ucontrol->value.integer.value[1];
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_get_volsw);

/**
 * snd_soc_put_volsw - single mixer put callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to set the value of a single mixer control.
 *
 * Returns 0 for success.
 */
int snd_soc_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & 0xff;
	int shift = (kcontrol->private_value >> 8) & 0x0f;
	int rshift = (kcontrol->private_value >> 12) & 0x0f;
	int max = (kcontrol->private_value >> 16) & 0xff;
	int mask = (1 << fls(max)) - 1;
	int invert = (kcontrol->private_value >> 24) & 0x01;
	unsigned short val, val2, val_mask;

	val = (ucontrol->value.integer.value[0] & mask);
	if (invert)
		val = max - val;
	val_mask = mask << shift;
	val = val << shift;
	if (shift != rshift) {
		val2 = (ucontrol->value.integer.value[1] & mask);
		if (invert)
			val2 = max - val2;
		val_mask |= mask << rshift;
		val |= val2 << rshift;
	}
	return snd_soc_update_bits(codec, reg, val_mask, val);
}
EXPORT_SYMBOL_GPL(snd_soc_put_volsw);

/**
 * snd_soc_info_volsw_2r - double mixer info callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to provide information about a double mixer control that
 * spans 2 codec registers.
 *
 * Returns 0 for success.
 */
int snd_soc_info_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_info *uinfo)
{
	int max = (kcontrol->private_value >> 12) & 0xff;

	if (max == 1)
		uinfo->type = SNDRV_CTL_ELEM_TYPE_BOOLEAN;
	else
		uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;

	uinfo->count = 2;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = max;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_info_volsw_2r);

/**
 * snd_soc_get_volsw_2r - double mixer get callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to get the value of a double mixer control that spans 2 registers.
 *
 * Returns 0 for success.
 */
int snd_soc_get_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & 0xff;
	int reg2 = (kcontrol->private_value >> 24) & 0xff;
	int shift = (kcontrol->private_value >> 8) & 0x0f;
	int max = (kcontrol->private_value >> 12) & 0xff;
	int mask = (1<<fls(max))-1;
	int invert = (kcontrol->private_value >> 20) & 0x01;

	ucontrol->value.integer.value[0] =
		(snd_soc_read(codec, reg) >> shift) & mask;
	ucontrol->value.integer.value[1] =
		(snd_soc_read(codec, reg2) >> shift) & mask;
	if (invert) {
		ucontrol->value.integer.value[0] =
			max - ucontrol->value.integer.value[0];
		ucontrol->value.integer.value[1] =
			max - ucontrol->value.integer.value[1];
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_get_volsw_2r);

/**
 * snd_soc_put_volsw_2r - double mixer set callback
 * @kcontrol: mixer control
 * @uinfo: control element information
 *
 * Callback to set the value of a double mixer control that spans 2 registers.
 *
 * Returns 0 for success.
 */
int snd_soc_put_volsw_2r(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	int reg = kcontrol->private_value & 0xff;
	int reg2 = (kcontrol->private_value >> 24) & 0xff;
	int shift = (kcontrol->private_value >> 8) & 0x0f;
	int max = (kcontrol->private_value >> 12) & 0xff;
	int mask = (1 << fls(max)) - 1;
	int invert = (kcontrol->private_value >> 20) & 0x01;
	int err;
	unsigned short val, val2, val_mask;

	val_mask = mask << shift;
	val = (ucontrol->value.integer.value[0] & mask);
	val2 = (ucontrol->value.integer.value[1] & mask);

	if (invert) {
		val = max - val;
		val2 = max - val2;
	}

	val = val << shift;
	val2 = val2 << shift;

	if ((err = snd_soc_update_bits(codec, reg, val_mask, val)) < 0)
		return err;

	err = snd_soc_update_bits(codec, reg2, val_mask, val2);
	return err;
}
EXPORT_SYMBOL_GPL(snd_soc_put_volsw_2r);

/**
 * snd_soc_dai_set_sysclk - configure DAI system or master clock.
 * @dai: DAI
 * @clk_id: DAI specific clock ID
 * @freq: new clock frequency in Hz
 * @dir: new clock direction - input/output.
 *
 * Configures the DAI master (MCLK) or system (SYSCLK) clocking.
 */
int snd_soc_dai_set_sysclk(struct snd_soc_dai *dai, int clk_id,
	unsigned int freq, int dir)
{
	if (dai->ops->set_sysclk)
		return dai->ops->set_sysclk(dai, clk_id, freq, dir);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_sysclk);

/**
 * snd_soc_dai_set_clkdiv - configure DAI clock dividers.
 * @dai: DAI
 * @clk_id: DAI specific clock divider ID
 * @div: new clock divisor.
 *
 * Configures the clock dividers. This is used to derive the best DAI bit and
 * frame clocks from the system or master clock. It's best to set the DAI bit
 * and frame clocks as low as possible to save system power.
 */
int snd_soc_dai_set_clkdiv(struct snd_soc_dai *dai,
	int div_id, int div)
{
	if (dai->ops->set_clkdiv)
		return dai->ops->set_clkdiv(dai, div_id, div);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_clkdiv);

/**
 * snd_soc_dai_set_pll - configure DAI PLL.
 * @dai: DAI
 * @pll_id: DAI specific PLL ID
 * @freq_in: PLL input clock frequency in Hz
 * @freq_out: requested PLL output clock frequency in Hz
 *
 * Configures and enables PLL to generate output clock based on input clock.
 */
int snd_soc_dai_set_pll(struct snd_soc_dai *dai,
	int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	if (dai->ops->set_pll)
		return dai->ops->set_pll(dai, pll_id, freq_in, freq_out);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_pll);

/**
 * snd_soc_dai_set_fmt - configure DAI hardware audio format.
 * @dai: DAI
 * @clk_id: DAI specific clock ID
 * @fmt: SND_SOC_DAIFMT_ format value.
 *
 * Configures the DAI hardware format and clocking.
 */
int snd_soc_dai_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if (dai->ops->set_fmt)
		return dai->ops->set_fmt(dai, fmt);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_fmt);

/**
 * snd_soc_dai_set_tdm_slot - configure DAI TDM.
 * @dai: DAI
 * @mask: DAI specific mask representing used slots.
 * @slots: Number of slots in use.
 *
 * Configures a DAI for TDM operation. Both mask and slots are codec and DAI
 * specific.
 */
int snd_soc_dai_set_tdm_slot(struct snd_soc_dai *dai,
	unsigned int mask, int slots)
{
	if (dai->ops->set_sysclk)
		return dai->ops->set_tdm_slot(dai, mask, slots);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_tdm_slot);

/**
 * snd_soc_dai_set_tristate - configure DAI system or master clock.
 * @dai: DAI
 * @tristate: tristate enable
 *
 * Tristates the DAI so that others can use it.
 */
int snd_soc_dai_set_tristate(struct snd_soc_dai *dai, int tristate)
{
	if (dai->ops->set_sysclk)
		return dai->ops->set_tristate(dai, tristate);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_tristate);

/**
 * snd_soc_dai_digital_mute - configure DAI system or master clock.
 * @dai: DAI
 * @mute: mute enable
 *
 * Mutes the DAI DAC.
 */
int snd_soc_dai_digital_mute(struct snd_soc_dai *dai, int mute)
{
	if (dai->ops->digital_mute)
		return dai->ops->digital_mute(dai, mute);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_digital_mute);

/**
 * snd_soc_register_codec_dai - register a codec DAI.
 * @template: pointer to DAI template
 * @dev: device
 *
 * Creates and registers a codec Digital Audio Interface with ASoC core.
 */
struct snd_soc_dai *snd_soc_register_codec_dai(
	struct snd_soc_dai_new *template, struct device *dev)
{
	struct snd_soc_dai *dai;

	BUG_ON(!dev);
	BUG_ON(!template->name);

	dai = kzalloc(sizeof(*dai), GFP_KERNEL);
	if (dai == NULL)
		return NULL;

	INIT_LIST_HEAD(&dai->list);
	dai->dev = dev;
	dai->ops = template->ops;
	dai->playback = template->playback;
	dai->capture = template->capture;
	dai->name = template->name;
	dai->ac97_control = template->ac97_control;
	mutex_lock(&client_mutex);
	list_add(&dai->list, &codec_dai_list);
	mutex_unlock(&client_mutex);
	soc_match_soc_card_pcms();
	return dai;
}
EXPORT_SYMBOL_GPL(snd_soc_register_codec_dai);

/**
 * snd_soc_unregister_codec_dai - add DAI to codec.
 * @dai: pointer to DAI
 *
 * Unregisters a codec Digital Audio Interface with ASoC core.
 */
void snd_soc_unregister_codec_dai(struct snd_soc_dai *dai_runtime)
{
	mutex_lock(&client_mutex);
	list_del(&dai_runtime->list);
	mutex_unlock(&client_mutex);
	kfree(dai_runtime);
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_codec_dai);

/**
 * snd_soc_new_codec - create new codec driver.
 * @template: new codec driver template
 * @cache: default register cache or NULL
 *
 * Creates a new codec and allocates resources including register cache.
 */
struct snd_soc_codec *snd_soc_new_codec(
	struct snd_soc_codec_new *template, const char *cache)
{
	struct snd_soc_codec *codec;

	BUG_ON(!template->name);

	codec = kzalloc(sizeof(*codec), GFP_KERNEL);
	if (codec == NULL)
		return NULL;

	if (cache) {
		codec->reg_cache = kmemdup(cache, template->reg_cache_size,
			GFP_KERNEL);
		if (codec->reg_cache == NULL) {
			kfree(codec);
			return NULL;
		}
	}

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->list);
	INIT_LIST_HEAD(&codec->dai_list);

	codec->name = template->name;
	codec->reg_cache_size = template->reg_cache_size;
	codec->reg_cache_step = template->reg_cache_step;
	codec->set_bias_level = template->set_bias_level;
	codec->init = template->init;
	codec->exit = template->exit;
	codec->codec_read = template->codec_read;
	codec->codec_write = template->codec_write;

	return codec;
}
EXPORT_SYMBOL_GPL(snd_soc_new_codec);

/**
 * snd_soc_register_codec - register codec driver.
 * @codec: codec driver
 * @dev: device
 *
 * Registers a new codec driver with ASoC core.
 */
int snd_soc_register_codec(struct snd_soc_codec *codec, struct device *dev)
{
	BUG_ON(!dev);

	mutex_lock(&client_mutex);
	codec->dev = dev;
	list_add(&codec->list, &codec_list);
	mutex_unlock(&client_mutex);
	soc_match_soc_card_pcms();
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_register_codec);

/**
 * snd_soc_unregister_codec - unregister a codec.
 * @codec: codec driver
 *
 * Unregisters a codec driver with the core.
 */
void snd_soc_unregister_codec(struct snd_soc_codec *codec)
{
	mutex_lock(&client_mutex);
	list_del(&codec->list);
	mutex_unlock(&client_mutex);
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_codec);

/**
 * snd_soc_register_platform_dai - registers a  platform DAI.
 * @dai: pointer to DAI
 *
 * Registers a platform Digital Audio Interfaces with ASoC core.
 */
struct snd_soc_dai *snd_soc_register_platform_dai(
	struct snd_soc_dai_new *template, struct device *dev)
{
	struct snd_soc_dai *dai;

	BUG_ON(!dev);
	BUG_ON(!template->name);

	dai = kzalloc(sizeof(*dai), GFP_KERNEL);
	if (dai == NULL)
		return NULL;

	INIT_LIST_HEAD(&dai->list);
	dai->dev = dev;
	dai->ops = template->ops;
	dai->playback = template->playback;
	dai->capture = template->capture;
	dai->name = template->name;
	dai->ac97_control = template->ac97_control;
	mutex_lock(&client_mutex);
	list_add(&dai->list, &cpu_dai_list);
	mutex_unlock(&client_mutex);
	soc_match_soc_card_pcms();
	return dai;
}
EXPORT_SYMBOL_GPL(snd_soc_register_platform_dai);

/**
 * snd_soc_unregister_platform_dai - unregisters a  platform DAI.
 * @dai: pointer to DAI
 *
 * Unregisters platform Digital Audio Interfaces with ASoC core.
 */
void snd_soc_unregister_platform_dai(struct snd_soc_dai *dai_runtime)
{
	mutex_lock(&client_mutex);
	list_del(&dai_runtime->list);
	mutex_unlock(&client_mutex);
	kfree(dai_runtime);
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_platform_dai);

/**
 * snd_soc_new_platform - register platform driver.
 * @template: new platform driver template
 *
 * Creates a new platform and allocates resources including register cache.
 */
struct snd_soc_platform *snd_soc_new_platform(
	struct snd_soc_platform_new *template)
{
	struct snd_soc_platform *platform;

	BUG_ON(!template->name);

	platform = kzalloc(sizeof(*platform), GFP_KERNEL);
	if (platform == NULL)
		return NULL;

	mutex_init(&platform->mutex);
	INIT_LIST_HEAD(&platform->dai_list);
	platform->name = template->name;
	platform->pcm_ops = template->pcm_ops;
	platform->pcm_new = template->pcm_new;
	platform->pcm_free = template->pcm_free;

	return platform;
}
EXPORT_SYMBOL_GPL(snd_soc_new_platform);

/**
 * snd_soc_register_platform - register platform driver.
 * @platform: platform driver
 *
 * Registers a platform driver with ASoC core.
 */
int snd_soc_register_platform(struct snd_soc_platform *platform,
	struct device *dev)
{
	BUG_ON(!dev);

	mutex_lock(&client_mutex);
	platform->dev = dev;
	list_add(&platform->list, &platform_list);
	mutex_unlock(&client_mutex);
	soc_match_soc_card_pcms();
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_register_platform);

/**
 * snd_soc_free_platform - unregister and free platform.
 * @platform: platform driver
 *
 * Unregisters platform with core and frees all resources.
 */
void snd_soc_free_platform(struct snd_soc_platform *platform)
{
	mutex_lock(&client_mutex);
	list_del(&platform->list);
	mutex_unlock(&client_mutex);
	kfree(platform);
}
EXPORT_SYMBOL_GPL(snd_soc_free_platform);

/**
 * snd_soc_card_create - create new ASoC sound card.
 * @name: soc_card name
 * @parent: parent device
 * @idx: sound card index
 * @xid: sound card ID
 *
 * Creates a new ASoC sound card.
 */
struct snd_soc_card *snd_soc_card_create(const char *name,
	struct device *parent, int idx, const char *xid)
{
	struct snd_soc_card *soc_card;

	soc_card = kzalloc(sizeof(*soc_card), GFP_KERNEL);
	if (soc_card == NULL)
		return NULL;

	soc_card->name = kstrdup(name, GFP_KERNEL);
	if (soc_card->name == NULL) {
		kfree(soc_card);
		return NULL;
	}

	soc_card->dev = parent;
	mutex_init(&soc_card->mutex);
	INIT_LIST_HEAD(&soc_card->dapm_widgets);
	INIT_LIST_HEAD(&soc_card->dapm_paths);
	INIT_LIST_HEAD(&soc_card->pcm_list);
	INIT_LIST_HEAD(&soc_card->config_list);

	/* register a sound card */
	soc_card->card = snd_card_new(idx, xid, THIS_MODULE, 0);
	if (!soc_card->card) {
		printk(KERN_ERR "asoc: can't create sound card for "
			"soc_card %s\n", soc_card->name);
		kfree(soc_card);
		return ERR_PTR(-ENODEV);
	}
	soc_card->card->dev = parent;
	return soc_card;
}
EXPORT_SYMBOL_GPL(snd_soc_card_create);

static int soc_pcm_new(struct snd_soc_card *soc_card,
	struct snd_soc_pcm_config *config)
{
	struct soc_pcm_config *_config;

	if (config->name == NULL)
		return -EINVAL;
	if (!config->playback && !config->capture) {
		printk(KERN_ERR "asoc: invalid codec for new pcm %s\n",
		       config->name);
		return -EINVAL;
	}
	if (config->codec == NULL || config->codec_dai == NULL) {
		printk(KERN_ERR "asoc: invalid codec for new pcm %s\n",
		       config->name);
		return -EINVAL;
	}
	if (config->platform == NULL || config->cpu_dai == NULL) {
		printk(KERN_ERR "asoc: invalid cpu for new pcm %s\n",
		       config->name);
		return -EINVAL;
	}
	if (!config->ops) {
		printk(KERN_ERR "asoc: no operations new pcm %s\n",
		       config->name);
		return -EINVAL;
	}

	_config = kzalloc(sizeof(*_config), GFP_KERNEL);
	if (_config == NULL)
		return -ENOMEM;

	_config->config = config;
	mutex_lock(&client_mutex);
	list_add(&_config->list, &soc_card->config_list);
	mutex_unlock(&client_mutex);
	soc_match_soc_card_pcms();
	return 0;
}

/**
 * snd_soc_card_create_pcms - create several ASoC PCM.
 * @soc_card: Machine
 * @configs: Array of PCM configurations
 * @num: Size of array
 *
 * Creates a new ALSA pcm for each config entry passed in. The config entry
 * will contain the codec, codec_dai, platform and platform_dai ID's.
 */
int snd_soc_card_create_pcms(struct snd_soc_card *soc_card,
			struct snd_soc_pcm_config *config, int num)
{
	int i, ret;

	for (i = 0; i < num; i++) {
		ret = soc_pcm_new(soc_card, &config[i]);
		if (ret != 0) {
			if (config[i].name)
				printk(KERN_ERR "asoc %s: Failed to register "
					"%s\n", soc_card->name, config[i].name);
			else
				printk(KERN_ERR "asoc %s: Failed to register "
					"unnamed PCM\n", soc_card->name);
			return ret;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_card_create_pcms);

/**
 * snd_soc_card_register - registers ASoC soc_card .
 * @soc_card: soc_card
 *
 * Registers a soc_card and it's PCMs. This should be called after all
 * codecs, platforms and PCM's have been created.
 */
int snd_soc_card_register(struct snd_soc_card *soc_card)
{
	mutex_lock(&client_mutex);
	soc_card->is_registered = 1;
	list_add(&soc_card->list, &soc_card_list);
	mutex_unlock(&client_mutex);
	soc_match_soc_card_pcms();
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_card_register);

/**
 * snd_soc_card_free - free soc_card.
 * @soc_card: soc_card
 *
 * Frees all soc_card resources. Can be called at any time during soc_card
 * initialisation process. This frees all soc card resources.
 */
void snd_soc_card_free(struct snd_soc_card *soc_card)
{
	struct snd_soc_pcm_runtime *pcm_runtime, *_pcm_runtime;
	struct soc_pcm_config *config;

	snd_card_free(soc_card->card);

	list_for_each_entry_safe(pcm_runtime, _pcm_runtime,
		&soc_card->pcm_list, list)
		run_delayed_work(&pcm_runtime->delayed_work);

	if (soc_card->exit && soc_card->is_probed)
		soc_card->exit(soc_card);

	list_for_each_entry_safe(pcm_runtime, _pcm_runtime,
		&soc_card->pcm_list, list) {
		kfree(pcm_runtime);
	}
	list_for_each_entry(config, &soc_card->config_list, list) {
		if (config->codec) {
#ifdef CONFIG_SND_SOC_AC97_BUS
			if (config->codec->ac97) {
				soc_codec_exit(config->codec, soc_card);
				soc_ac97_dev_unregister(config->codec);
				snd_soc_free_ac97_codec(config->codec);
			}
#else
			soc_codec_exit(config->codec, soc_card);
#endif
			module_put(config->codec->dev->driver->owner);
		}
		if (config->codec_dai)
			module_put(config->codec_dai->dev->driver->owner);
		if (config->platform)
			module_put(config->platform->dev->driver->owner);
		if (config->cpu_dai)
			module_put(config->cpu_dai->dev->driver->owner);
	}
	snd_soc_dapm_free(soc_card);
	list_del(&soc_card->list);
	kfree(soc_card);
}
EXPORT_SYMBOL_GPL(snd_soc_card_free);

/**
 * snd_soc_card_get_codec - get codec.
 * @soc_card: soc sound card
 * @codec_id: codec ID
 *
 * Get a codec from a codec ID.
 */
struct snd_soc_codec *snd_soc_card_get_codec(struct snd_soc_card *soc_card,
	const char *codec_id)
{
	struct soc_pcm_config *config;

	list_for_each_entry(config, &soc_card->config_list, list) {
		if (config->codec && !strcmp(config->codec->name, codec_id))
			return config->codec;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_card_get_codec);

/**
 * snd_soc_card_get_platform - get platform.
 * @soc_card: soc sound card
 * @codec_id: platform ID
 *
 * Get a platform from a platform ID.
 */
struct snd_soc_platform * snd_soc_card_get_platform(struct snd_soc_card *soc_card,
	const char *platform_id)
{
	struct soc_pcm_config *config;

	list_for_each_entry(config, &soc_card->config_list, list) {
		if (config->platform &&
			!strcmp(config->platform->name, platform_id))
			return config->platform;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_card_get_platform);

/**
 * snd_soc_card_get_dai - get dai.
 * @soc_card: soc_card
 * @codec_id: dai ID
 *
 * Get a codec or platform dai from a dai ID.
 */
struct snd_soc_dai *snd_soc_card_get_dai(struct snd_soc_card *soc_card,
	const char *dai_id)
{
	struct soc_pcm_config *config;

	list_for_each_entry(config, &soc_card->config_list, list) {
		if (config->codec_dai &&
			!strcmp(config->codec_dai->name, dai_id))
			return config->codec_dai;
		if (config->cpu_dai &&
			!strcmp(config->cpu_dai->name, dai_id))
			return config->cpu_dai;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_card_get_dai);

/**
 * snd_soc_card_get_pcm - get pcm.
 * @soc_card: soc_card
 * @pcm_id: pcm ID
 *
 * Get a pcm runtime pointer from pcm ID.
 */
struct snd_soc_pcm_runtime *snd_soc_card_get_pcm(struct snd_soc_card *soc_card,
	const char *pcm_id)
{
	struct snd_soc_pcm_runtime *pcm_runtime;

	list_for_each_entry(pcm_runtime, &soc_card->pcm_list, list) {
		if (!strcmp(pcm_runtime->name, pcm_id))
			return pcm_runtime;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_card_get_pcm);

/**
 * snd_soc_card_get_ac97_ops - get AC97 operations.
 * @soc_card: soc_card
 * @dai_id:  ID
 *
 * Get AC97 bus operations from Digital Audio Interface ID.
 */
struct snd_ac97_bus_ops *snd_soc_card_get_ac97_ops(struct snd_soc_card *soc_card,
					      const char *dai_id)
{
	struct snd_soc_pcm_runtime *pcm_runtime;

	list_for_each_entry(pcm_runtime, &soc_card->pcm_list, list) {
		if (strcmp(pcm_runtime->cpu_dai->name, dai_id) == 0)
			return pcm_runtime->cpu_dai->ops->ac97_ops;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_card_get_ac97_ops);

/**
 * snd_soc_card_config_codec - initialise codec IO.
 * @codec: codec
 * @soc_phys_read: read function called by codec.
 * @soc_phys_write: write function called by codec.
 * @control_data: IO control data - usually I2C, SPI, etc pointer
 *
 * Initialises the codec IO system with the codec IO mechanism. Codec will
 * be able to perform IO after this point.
 */
void snd_soc_card_config_codec(struct snd_soc_codec *codec,
	unsigned int (*soc_phys_read)(void *, int),
	int (*soc_phys_write)(void *, long, int), void *control_data)
{
	codec->control_data = control_data;
	codec->soc_phys_read = soc_phys_read;
	codec->soc_phys_write = soc_phys_write;
}
EXPORT_SYMBOL_GPL(snd_soc_card_config_codec);

/**
 * snd_soc_card_init_codec - initialises codec
 * @codec: codec
 * @soc_card: soc sound card
 *
 * Initialises codec hardware. Can perform IO and must only be called after a
 * successful call to snd_soc_card_config_codec().
 */
int snd_soc_card_init_codec(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card)
{
	int ret;

	/* we can add our sysfs register access now codec IO is set */
	ret = device_create_file(codec->dev, &dev_attr_codec_reg);
	if (ret < 0) {
		printk(KERN_WARNING "asoc: failed to add codec sysfs "
			"entries\n");
		return ret;
	}
	if (codec->init)
		ret = codec->init(codec, soc_card);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_card_init_codec);

static int __init asoc_init(void)
{
	printk(KERN_INFO "ASoC version %s\n", SND_SOC_VERSION);
	return 0;
}
subsys_initcall(asoc_init);

static void __exit asoc_exit(void)
{
}
module_exit(asoc_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("ALSA SoC Core");
MODULE_LICENSE("GPL");
