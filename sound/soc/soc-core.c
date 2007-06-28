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
 *   o More testing with other codecs/machines.
 *   o Add more codecs and platforms to ensure good API coverage.
 *   o Support TDM on PCM and I2S
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

/* debug */
#define SOC_DEBUG 0
#if SOC_DEBUG
#define dbg(format, arg...) printk(format, ## arg)
#else
#define dbg(format, arg...)
#endif

static DEFINE_MUTEX(pcm_mutex);
static DEFINE_MUTEX(io_mutex);
static DEFINE_MUTEX(list_mutex);
static DECLARE_WAIT_QUEUE_HEAD(soc_pm_waitq);
static LIST_HEAD(soc_codec_list);
static LIST_HEAD(soc_codec_dai_list);
static LIST_HEAD(soc_cpu_dai_list);
static LIST_HEAD(soc_platform_list);
static LIST_HEAD(soc_pcm_link_list);

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
	
	if (ac97->dev.bus)
		device_unregister(&ac97->dev);
	return 0;
}

/* stop no dev release warning */
static void soc_ac97_device_release(struct device *dev){}

/* register ac97 codec to bus */
static int soc_ac97_dev_register(struct snd_soc_codec *codec, char *name)
{
	struct snd_ac97 *ac97 = codec->ac97;
	int err;

	ac97->dev.bus = &ac97_bus_type;
	ac97->dev.parent = NULL;
	ac97->dev.release = soc_ac97_device_release;

	snprintf(ac97->dev.bus_id, BUS_ID_SIZE, "%d-%d:%s",
		 0, 0, name);
	err = device_register(&ac97->dev);
	if (err < 0) {
		snd_printk(KERN_ERR "Can't register ac97 bus\n");
		ac97->dev.bus = NULL;
		return err;
	}
	return 0;
}
#endif

/*
 * Called by ALSA when a PCM substream is opened, the runtime->hw record is
 * then initialized and any private data can be allocated. This also calls
 * startup for the cpu DAI, platform, machine and codec DAI.
 */
static int soc_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_platform *platform = pcm_link->platform;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	struct snd_soc_codec *codec = pcm_link->codec;
	int ret = 0;

	mutex_lock(&pcm_mutex);

	/* startup the audio subsystem */
	if (cpu_dai->audio_ops && cpu_dai->audio_ops->startup) {
		ret = cpu_dai->audio_ops->startup(substream);
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

	if (codec_dai->audio_ops && codec_dai->audio_ops->startup) {
		ret = codec_dai->audio_ops->startup(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't open codec %s\n",
				codec_dai->name);
			goto codec_dai_err;
		}
	}

	if (pcm_link->audio_ops && pcm_link->audio_ops->startup) {
		ret = pcm_link->audio_ops->startup(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: %s startup failed\n", pcm_link->name);
			goto pcm_link_err;
		}
	}

	/* Check that the codec and cpu DAI's are compatible */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		runtime->hw.rate_min =
			max(codec_dai->playback->rate_min, 
				cpu_dai->playback->rate_min);
		runtime->hw.rate_max =
			min(codec_dai->playback->rate_max, 
				cpu_dai->playback->rate_max);
		runtime->hw.channels_min =
			max(codec_dai->playback->channels_min,
				cpu_dai->playback->channels_min);
		runtime->hw.channels_max =
			min(codec_dai->playback->channels_max,
				cpu_dai->playback->channels_max);
		runtime->hw.formats =
			codec_dai->playback->formats & 
				cpu_dai->playback->formats;
		runtime->hw.rates =
			codec_dai->playback->rates & cpu_dai->playback->rates;
	} else {
		runtime->hw.rate_min =
			max(codec_dai->capture->rate_min, 
				cpu_dai->capture->rate_min);
		runtime->hw.rate_max =
			min(codec_dai->capture->rate_max, 
				cpu_dai->capture->rate_max);
		runtime->hw.channels_min =
			max(codec_dai->capture->channels_min,
				cpu_dai->capture->channels_min);
		runtime->hw.channels_max =
			min(codec_dai->capture->channels_max,
				cpu_dai->capture->channels_max);
		runtime->hw.formats =
			codec_dai->capture->formats & cpu_dai->capture->formats;
		runtime->hw.rates =
			codec_dai->capture->rates & cpu_dai->capture->rates;
	}

	ret = -EINVAL;
	snd_pcm_limit_hw_rates(runtime);
	if (!runtime->hw.rates) {
		printk(KERN_ERR "asoc: %s <-> %s No matching rates\n",
			codec_dai->name, cpu_dai->name);
		goto pcm_link_err;
	}
	if (!runtime->hw.formats) {
		printk(KERN_ERR "asoc: %s <-> %s No matching formats\n",
			codec_dai->name, cpu_dai->name);
		goto pcm_link_err;
	}
	if (!runtime->hw.channels_min || !runtime->hw.channels_max) {
		printk(KERN_ERR "asoc: %s <-> %s No matching channels\n",
			codec_dai->name, cpu_dai->name);
		goto pcm_link_err;
	}

	dbg("asoc: %s <-> %s info:\n",codec_dai->name, cpu_dai->name);
	dbg("asoc: rate mask 0x%x\n", runtime->hw.rates);
	dbg("asoc: min ch %d max ch %d\n", runtime->hw.channels_min,
		runtime->hw.channels_max);
	dbg("asoc: min rate %d max rate %d\n", runtime->hw.rate_min,
		runtime->hw.rate_max);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cpu_dai->playback_active = codec_dai->playback_active = 1;
	else
		cpu_dai->capture_active = codec_dai->capture_active = 1;
	cpu_dai->active = codec_dai->active = 1;
	cpu_dai->runtime = runtime;
	codec->active++;
	mutex_unlock(&pcm_mutex);
	return 0;

pcm_link_err:
	if (pcm_link->audio_ops && pcm_link->audio_ops->shutdown)
		pcm_link->audio_ops->shutdown(substream);

codec_dai_err:
	if (platform->pcm_ops->close)
		platform->pcm_ops->close(substream);

platform_err:
	if (cpu_dai->audio_ops && cpu_dai->audio_ops->shutdown)
		cpu_dai->audio_ops->shutdown(substream);
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
	struct snd_soc_pcm_link *pcm_link =
		container_of(work, struct snd_soc_pcm_link, 
			delayed_work.work);
	struct snd_soc_codec *codec = pcm_link->codec;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	struct snd_soc_machine *machine = pcm_link->machine;

	mutex_lock(&pcm_mutex);

	dbg("pop wq checking: %s status: %s waiting: %s\n",
		codec_dai->playback->stream_name,
		codec_dai->playback_active ? "active" : "inactive",
		codec_dai->pop_wait ? "yes" : "no");

	/* are we waiting on this codec DAI stream */
	if (codec_dai->pop_wait == 1) {

		/* power down the codec to D1 if no longer active */
		if (codec->active == 0) {
			dbg("pop wq D1 %s %s\n", codec->name,
				codec_dai->playback->stream_name);
			snd_soc_dapm_device_event(pcm_link, 
				SNDRV_CTL_POWER_D1);
		}

		codec_dai->pop_wait = 0;
		snd_soc_dapm_stream_event(machine, 
			codec_dai->playback->stream_name,
			SND_SOC_DAPM_STREAM_STOP);

		/* power down the codec power domain if no longer active */
		if (codec->active == 0) {
			dbg("pop wq D3 %s %s\n", codec->name,
				codec_dai->playback->stream_name);
	 		snd_soc_dapm_device_event(pcm_link, 
	 			SNDRV_CTL_POWER_D3hot);
		}
	}
	mutex_unlock(&pcm_mutex);
}

/*
 * Called by ALSA when a PCM substream is closed. Private data can be
 * freed here. The cpu DAI, codec DAI, machine and platform are also
 * shutdown.
 */
static int soc_codec_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_platform *platform = pcm_link->platform;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	struct snd_soc_codec *codec = pcm_link->codec;
	struct snd_soc_machine *machine = pcm_link->machine;

	mutex_lock(&pcm_mutex);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		cpu_dai->playback_active = codec_dai->playback_active = 0;
	else
		cpu_dai->capture_active = codec_dai->capture_active = 0;

	if (codec_dai->playback_active == 0 &&
		codec_dai->capture_active == 0) {
		cpu_dai->active = codec_dai->active = 0;
	}
	codec->active--;

	if (cpu_dai->audio_ops && cpu_dai->audio_ops->shutdown)
		cpu_dai->audio_ops->shutdown(substream);

	if (codec_dai->audio_ops && codec_dai->audio_ops->shutdown)
		codec_dai->audio_ops->shutdown(substream);

	if (pcm_link->audio_ops && pcm_link->audio_ops->shutdown)
		pcm_link->audio_ops->shutdown(substream);

	if (platform->pcm_ops->close)
		platform->pcm_ops->close(substream);
	cpu_dai->runtime = NULL;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* start delayed pop wq here for playback streams */
		codec_dai->pop_wait = 1;
		schedule_delayed_work(&pcm_link->delayed_work,
			msecs_to_jiffies(pmdown_time));
	} else {
		/* capture streams can be powered down now */
		snd_soc_dapm_stream_event(machine,
			codec_dai->capture->stream_name, 
			SND_SOC_DAPM_STREAM_STOP);

		if (codec->active == 0 && codec_dai->pop_wait == 0)
			snd_soc_dapm_device_event(pcm_link, 
				SNDRV_CTL_POWER_D3hot);
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
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_platform *platform = pcm_link->platform;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	struct snd_soc_codec *codec = pcm_link->codec;
	struct snd_soc_machine *machine = pcm_link->machine;
	int ret = 0;

	mutex_lock(&pcm_mutex);

	if (pcm_link->audio_ops && pcm_link->audio_ops->prepare) {
		ret = pcm_link->audio_ops->prepare(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: machine prepare error\n");
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

	if (codec_dai->audio_ops && codec_dai->audio_ops->prepare) {
		ret = codec_dai->audio_ops->prepare(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: codec DAI prepare error\n");
			goto out;
		}
	}

	if (cpu_dai->audio_ops && cpu_dai->audio_ops->prepare) {
		ret = cpu_dai->audio_ops->prepare(substream);
		if (ret < 0) {
			printk(KERN_ERR "asoc: cpu DAI prepare error\n");
			goto out;
		}
	}

	/* we only want to start a DAPM playback stream if we are not waiting
	 * on an existing one stopping */
	if (codec_dai->pop_wait) {
		/* we are waiting for the delayed work to start */
		if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
				snd_soc_dapm_stream_event(machine,
					codec_dai->capture->stream_name,
					SND_SOC_DAPM_STREAM_START);
		else {
			codec_dai->pop_wait = 0;
			cancel_delayed_work(&pcm_link->delayed_work);
			if (codec_dai->ops && codec_dai->ops->digital_mute)
				codec_dai->ops->digital_mute(codec_dai, 0);
		}
	} else {
		/* no delayed work - do we need to power up codec */
		if (codec->dapm_state != SNDRV_CTL_POWER_D0) {

			snd_soc_dapm_device_event(pcm_link, SNDRV_CTL_POWER_D1);

			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				snd_soc_dapm_stream_event(machine,
					codec_dai->playback->stream_name,
					SND_SOC_DAPM_STREAM_START);
			else
				snd_soc_dapm_stream_event(machine,
					codec_dai->capture->stream_name,
					SND_SOC_DAPM_STREAM_START);

			snd_soc_dapm_device_event(pcm_link, SNDRV_CTL_POWER_D0);
			if (codec_dai->ops && codec_dai->ops->digital_mute)
				codec_dai->ops->digital_mute(codec_dai, 0);

		} else {
			/* codec already powered - power on widgets */
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				snd_soc_dapm_stream_event(machine,
					codec_dai->playback->stream_name,
					SND_SOC_DAPM_STREAM_START);
			else
				snd_soc_dapm_stream_event(machine,
					codec_dai->capture->stream_name,
					SND_SOC_DAPM_STREAM_START);
			if (codec_dai->ops && codec_dai->ops->digital_mute)
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
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_platform *platform = pcm_link->platform;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	int ret = 0;

	mutex_lock(&pcm_mutex);

	if (pcm_link->audio_ops && pcm_link->audio_ops->hw_params) {
		ret = pcm_link->audio_ops->hw_params(substream, params);
		if (ret < 0) {
			printk(KERN_ERR "asoc: machine hw_params failed\n");
			goto out;
		}
	}

	if (codec_dai->audio_ops && codec_dai->audio_ops->hw_params) {
		ret = codec_dai->audio_ops->hw_params(substream, params);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't set codec %s hw params\n",
				codec_dai->name);
			goto codec_err;
		}
	}

	if (cpu_dai->audio_ops && cpu_dai->audio_ops->hw_params) {
		ret = cpu_dai->audio_ops->hw_params(substream, params);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't set interface %s hw params\n",
				cpu_dai->name);
			goto interface_err;
		}
	}

	if (platform->pcm_ops->hw_params) {
		ret = platform->pcm_ops->hw_params(substream, params);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't set platform %s hw params\n",
				platform->name);
			goto platform_err;
		}
	}

out:
	mutex_unlock(&pcm_mutex);
	return ret;

platform_err:
	if (cpu_dai->audio_ops && cpu_dai->audio_ops->hw_free)
		cpu_dai->audio_ops->hw_free(substream);

interface_err:
	if (codec_dai->audio_ops && codec_dai->audio_ops->hw_free)
		codec_dai->audio_ops->hw_free(substream);

codec_err:
	if(pcm_link->audio_ops && pcm_link->audio_ops->hw_free)
		pcm_link->audio_ops->hw_free(substream);

	mutex_unlock(&pcm_mutex);
	return ret;
}

/*
 * Free's resources allocated by hw_params, can be called multiple times
 */
static int soc_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_platform *platform = pcm_link->platform;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	struct snd_soc_codec *codec = pcm_link->codec;

	mutex_lock(&pcm_mutex);

	/* apply codec digital mute */
	if (!codec->active && codec_dai->ops && codec_dai->ops->digital_mute)
		codec_dai->ops->digital_mute(codec_dai, 1);

	/* free any machine hw params */
	if (pcm_link->audio_ops && pcm_link->audio_ops->hw_free)
		pcm_link->audio_ops->hw_free(substream);

	/* free any DMA resources */
	if (platform->pcm_ops->hw_free)
		platform->pcm_ops->hw_free(substream);

	/* now free hw params for the DAI's  */
	if (codec_dai->audio_ops && codec_dai->audio_ops->hw_free)
		codec_dai->audio_ops->hw_free(substream);

	if (cpu_dai->audio_ops && cpu_dai->audio_ops->hw_free)
		cpu_dai->audio_ops->hw_free(substream);

	mutex_unlock(&pcm_mutex);
	return 0;
}

static int soc_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_link *pcm_link = substream->private_data;
	struct snd_soc_platform *platform = pcm_link->platform;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	int ret;

	if (codec_dai->audio_ops && codec_dai->audio_ops->trigger) {
		ret = codec_dai->audio_ops->trigger(substream, cmd);
		if (ret < 0)
			return ret;
	}

	if (platform->pcm_ops->trigger) {
		ret = platform->pcm_ops->trigger(substream, cmd);
		if (ret < 0)
			return ret;
	}

	if (cpu_dai->audio_ops && cpu_dai->audio_ops->trigger) {
		ret = cpu_dai->audio_ops->trigger(substream, cmd);
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
/* powers down audio subsystem for suspend */
int snd_soc_suspend(struct snd_soc_machine *machine, pm_message_t state)
{
 	struct snd_soc_platform *platform;
	struct snd_soc_dai *cpu_dai;
	struct snd_soc_dai *codec_dai;
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_link *pcm_link;
	char *stream;

	if (machine->ops && machine->ops->suspend_pre)
		machine->ops->suspend_pre(machine, state);
		
	/* mute any active DAC's */
	list_for_each_entry(pcm_link, &machine->active_list, active_list) {
		codec_dai = pcm_link->codec_dai;
		if (codec_dai->ops && 
			codec_dai->ops->digital_mute &&
			codec_dai->playback_active)
			codec_dai->ops->digital_mute(codec_dai, 1);
	}

	snd_power_change_state(machine->card, SNDRV_CTL_POWER_D3cold);
	
	/* suspend all pcm's */
	list_for_each_entry(pcm_link, &machine->active_list, active_list)
		snd_pcm_suspend_all(pcm_link->pcm);

	list_for_each_entry(pcm_link, &machine->active_list, active_list) {
		cpu_dai = pcm_link->cpu_dai;
		platform = pcm_link->platform;
		if (cpu_dai->dev.driver->suspend && 
			cpu_dai->type != SND_SOC_DAI_AC97)
			cpu_dai->dev.driver->suspend(&cpu_dai->dev, state);
		if (platform->dev.driver->suspend)
			platform->dev.driver->suspend(&platform->dev, state);
	}

	/* close any waiting streams and save state */
	list_for_each_entry(pcm_link, &machine->active_list, active_list) {
		codec = pcm_link->codec;
		run_delayed_work(&pcm_link->delayed_work);
		codec->suspend_dapm_state = codec->dapm_state;
	}

	list_for_each_entry(pcm_link, &machine->active_list, active_list) {
		codec_dai = pcm_link->codec_dai;
		stream = codec_dai->playback->stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(machine, stream,
				SND_SOC_DAPM_STREAM_SUSPEND);
		stream = codec_dai->capture->stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(machine, stream,
				SND_SOC_DAPM_STREAM_SUSPEND);
	}

	list_for_each_entry(pcm_link, &machine->active_list, active_list) {
		codec = pcm_link->codec;
		if (codec->dev.driver->suspend)
			codec->dev.driver->suspend(&codec->dev, state);
	}

	list_for_each_entry(pcm_link, &machine->active_list, active_list) {
		cpu_dai = pcm_link->cpu_dai;
		if (cpu_dai->dev.driver->suspend && 
			cpu_dai->type == SND_SOC_DAI_AC97)
			cpu_dai->dev.driver->suspend(&cpu_dai->dev, state);
	}

	if (machine->ops && machine->ops->suspend_post)
		machine->ops->suspend_post(machine, state);

	return 0;
}

/* powers up audio subsystem after a suspend */
int snd_soc_resume(struct snd_soc_machine *machine)
{
 	struct snd_soc_platform *platform;
	struct snd_soc_dai *cpu_dai;
	struct snd_soc_dai *codec_dai;
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_link *pcm_link;
	char *stream;
	
	if (machine->ops && machine->ops->resume_pre)
		machine->ops->resume_pre(machine);

	list_for_each_entry(pcm_link, &soc_pcm_link_list, active_list) {
		cpu_dai = pcm_link->cpu_dai;
		codec = pcm_link->codec;
		
		if (cpu_dai->dev.driver->resume && 
			cpu_dai->type == SND_SOC_DAI_AC97)
			cpu_dai->dev.driver->resume(&cpu_dai->dev);
		if (codec->dev.driver->resume)
			codec->dev.driver->resume(&codec->dev);
	}

	list_for_each_entry(pcm_link, &machine->active_list, active_list) {
		codec_dai = pcm_link->codec_dai;
		stream = codec_dai->playback->stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(machine, stream,
				SND_SOC_DAPM_STREAM_RESUME);
		stream = codec_dai->capture->stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(machine, stream,
				SND_SOC_DAPM_STREAM_RESUME);
	}

	/* unmute any active DAC's */
	list_for_each_entry(pcm_link, &machine->active_list, active_list) {
		codec_dai = pcm_link->codec_dai;
		if (codec_dai->ops && 
			codec_dai->ops->digital_mute &&
			codec_dai->playback_active)
			codec_dai->ops->digital_mute(codec_dai, 0);
	}

	list_for_each_entry(pcm_link, &machine->active_list, active_list) {
		cpu_dai = pcm_link->cpu_dai;
		platform = pcm_link->platform;
		if (cpu_dai->dev.driver->resume && 
			cpu_dai->type != SND_SOC_DAI_AC97)
			cpu_dai->dev.driver->resume(&cpu_dai->dev);
		if (platform->dev.driver->resume)
			platform->dev.driver->resume(&platform->dev);
	}

	if (machine->ops && machine->ops->resume_post)
		machine->ops->resume_post(machine);

	snd_power_change_state(machine->card, SNDRV_CTL_POWER_D3hot);
	return 0;
}

#else
int snd_soc_suspend(struct snd_soc_machine *machine, pm_message_t state)
{
	return 0;
}

int snd_soc_resume(struct snd_soc_machine *machine)
{
	return 0;
}
#endif
EXPORT_SYMBOL_GPL(snd_soc_suspend);
EXPORT_SYMBOL_GPL(snd_soc_resume);

static void soc_match_components(void)
{
	struct snd_soc_codec *codec;
	struct snd_soc_dai *codec_dai, *cpu_dai;
	struct snd_soc_platform *platform;
	struct snd_soc_pcm_link *pcm_link, *tmp;
	struct snd_soc_machine *machine = NULL;
	int probe = 0;

	mutex_lock(&list_mutex);
	list_for_each_entry_safe(pcm_link, tmp, &soc_pcm_link_list, all_list) {

		if (pcm_link->probed)
			continue;
		machine = pcm_link->machine;
		probe = 0;
	
		if (!pcm_link->codec) {	
			list_for_each_entry(codec, &soc_codec_list, list) {
				if (!strcmp(codec->name, pcm_link->codec_id)) {
					pcm_link->codec = codec;
					break;
				}
			}
		}

		if (!pcm_link->codec_dai) {
			list_for_each_entry(codec_dai, &soc_codec_dai_list, list) {
				if (!strcmp(codec_dai->name, pcm_link->codec_dai_id)) {
					pcm_link->codec_dai = codec_dai;
					break;
				}
			}
		}

		if (!pcm_link->cpu_dai) {
			list_for_each_entry(cpu_dai, &soc_cpu_dai_list, list) {
				if (!strcmp(cpu_dai->name, pcm_link->cpu_dai_id)) {
					pcm_link->cpu_dai = cpu_dai;
					break;
				}
			}
		}

		if (!pcm_link->platform) {
			list_for_each_entry(platform, &soc_platform_list, list) {
				if (!strcmp(platform->name, pcm_link->platform_id)) {
					pcm_link->platform = platform;
					break;
				}
			}
		}

		if (pcm_link->platform && pcm_link->cpu_dai &&
			pcm_link->codec_dai && pcm_link->codec) {
			pcm_link->link_ops->new(pcm_link);
			probe = pcm_link->probed = 1;
			list_add(&pcm_link->codec->dai_list, 
				&pcm_link->codec_dai->codec_list);
			list_add(&pcm_link->active_list, &machine->active_list);
			pcm_link->codec_dai->codec = pcm_link->codec;
		}
	}

	/* are all pcm_links now created ? */
	if (probe && machine && machine->pcm_links == machine->pcm_links_total)
		machine->ops->mach_probe(machine);
	mutex_unlock(&list_mutex);
}

static void soc_pcm_link_remove(struct snd_soc_pcm_link *pcm_link)
{
	struct snd_soc_machine *machine = pcm_link->machine;
	
	if (pcm_link->probed && pcm_link->link_ops->free) {
		if (machine && !machine->disconnect) {
			snd_card_disconnect(machine->card);
			machine->disconnect = 1;
			machine->pcm_links--;
		}
		run_delayed_work(&pcm_link->delayed_work);
		pcm_link->link_ops->free(pcm_link);
		pcm_link->probed = 0;
		list_del(&pcm_link->active_list);
	}
	if (machine && machine->pcm_links == 0) {
		if (machine->ops->mach_remove)
			machine->ops->mach_remove(machine);
	}
}

static void soc_remove_components(const char *name)
{
	struct snd_soc_codec *codec;
	struct snd_soc_dai *codec_dai, *cpu_dai;
	struct snd_soc_platform *platform;
	struct snd_soc_pcm_link *pcm_link, *tmp;

	mutex_lock(&list_mutex);
	list_for_each_entry_safe(pcm_link, tmp, &soc_pcm_link_list, all_list) {
		
		codec = pcm_link->codec;
		codec_dai = pcm_link->codec_dai;
		cpu_dai = pcm_link->cpu_dai;
		platform = pcm_link->platform;
	
		if (codec && !strcmp(name, codec->name)) {
			soc_pcm_link_remove(pcm_link);
			pcm_link->codec = NULL;
			goto free;
		} else if (codec_dai && !strcmp(name, codec_dai->name)) {
			soc_pcm_link_remove(pcm_link);
			pcm_link->codec_dai = NULL;
			goto free;
		} else if (cpu_dai && !strcmp(name, cpu_dai->name)) {
			soc_pcm_link_remove(pcm_link);
			pcm_link->cpu_dai = NULL;
			goto free;
		} else if (platform && !strcmp(name, platform->name)) {
			soc_pcm_link_remove(pcm_link);
			pcm_link->platform = NULL;
			goto free;
		}
free:
		if (!pcm_link->codec && !pcm_link->platform &&
			!pcm_link->codec_dai && !pcm_link->cpu_dai) {
			list_del(&pcm_link->all_list);
			kfree(pcm_link);
		}
	}
	mutex_unlock(&list_mutex);
}

static void soc_dai_dev_release(struct device *dev)
{
	struct snd_soc_dai *dai = to_snd_soc_dai(dev);
	
	list_del(&dai->list);
	soc_remove_components(dai->name);
	kfree(dai);
}

static void soc_codec_dev_release(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);

	list_del(&codec->list);
	soc_remove_components(codec->name);
	kfree(codec);
}

static void soc_platform_dev_release(struct device *dev)
{
	struct snd_soc_platform *platform = to_snd_soc_platform(dev);
	list_del(&platform->list);
	soc_remove_components(platform->name);
	kfree(platform);
}

/* create a new pcm */
int snd_soc_pcm_new(struct snd_soc_pcm_link *pcm_link, int playback, 
	int capture)
{
	struct snd_soc_codec *codec = pcm_link->codec;
	struct snd_soc_dai *codec_dai = pcm_link->codec_dai;
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_platform *platform = pcm_link->platform;
	struct snd_soc_machine *machine = pcm_link->machine;
	struct snd_pcm *pcm;
	int ret = 0;

	snd_assert(codec != NULL, return -EINVAL);
	snd_assert(codec_dai != NULL, return -EINVAL);
	snd_assert(cpu_dai != NULL, return -EINVAL);
	snd_assert(platform != NULL, return -EINVAL);
	snd_assert(machine != NULL, return -EINVAL);

	ret = snd_pcm_new(machine->card, (char*)pcm_link->name, 
		machine->pcm_links++, playback, capture, &pcm);
	if (ret < 0) {
		printk(KERN_ERR "asoc: can't create pcm for codec %s\n", codec->name);
		return ret;
	}

	pcm_link->pcm = pcm;
	pcm->private_data = pcm_link;
	soc_pcm_ops.mmap = platform->pcm_ops->mmap;
	soc_pcm_ops.pointer = platform->pcm_ops->pointer;
	soc_pcm_ops.ioctl = platform->pcm_ops->ioctl;
	soc_pcm_ops.copy = platform->pcm_ops->copy;
	soc_pcm_ops.silence = platform->pcm_ops->silence;
	soc_pcm_ops.ack = platform->pcm_ops->ack;
	soc_pcm_ops.page = platform->pcm_ops->page;

	if (playback)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &soc_pcm_ops);

	if (capture)
		snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE, &soc_pcm_ops);

	ret = platform->platform_ops->pcm_new(platform, machine->card, 
		playback, capture, pcm);
	if (ret < 0) {
		printk(KERN_ERR "asoc: platform pcm constructor failed\n");
		return ret;
	}

	pcm->private_free = platform->platform_ops->pcm_free;
	printk(KERN_INFO "asoc: %s <-> %s mapping ok\n", codec_dai->name,
		cpu_dai->name);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_pcm_new);

/* codec register dump */
static ssize_t codec_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	int i, step = 1, count = 0;

	if (!codec->reg_cache_size)
		return 0;

	if (codec->reg_cache_step)
		step = codec->reg_cache_step;

	count += sprintf(buf, "%s registers\n", codec->name);
	for(i = 0; i < codec->reg_cache_size; i += step)
		count += sprintf(buf + count, "%2x: %4x\n", i, codec->ops->read(codec, i));

	return count;
}
static DEVICE_ATTR(codec_reg, 0444, codec_reg_show, NULL);

/**
 * snd_soc_new_ac97_codec - initailise AC97 device
 * @codec: audio codec
 * @ops: AC97 bus operations
 * @num: AC97 codec number
 *
 * Initialises AC97 codec resources for use by ad-hoc devices only.
 */
int snd_soc_new_ac97_codec(struct snd_soc_pcm_link *pcm_link,
	struct snd_ac97_bus_ops *ops, int num)
{
	struct snd_soc_dai *cpu_dai = pcm_link->cpu_dai;
	struct snd_soc_machine *machine = pcm_link->machine;
	struct snd_ac97 *ac97;
	
	snd_assert(machine != NULL, return -EINVAL);
	snd_assert(cpu_dai != NULL, return -EINVAL);
	snd_assert(ops != NULL, return -EINVAL);
	
	mutex_lock(&machine->mutex);

	ac97 = kzalloc(sizeof(struct snd_ac97), GFP_KERNEL);
	if (ac97 == NULL) {
		mutex_unlock(&machine->mutex);
		return -ENOMEM;
	}

	ac97->bus = kzalloc(sizeof(struct snd_ac97_bus), GFP_KERNEL);
	if (ac97->bus == NULL) {
		kfree(ac97);
		mutex_unlock(&machine->mutex);
		return -ENOMEM;
	}

	ac97->bus->ops = ops;
	ac97->num = num;
	ac97->bus->card = machine->card;
	ac97->bus->clock = 48000;
	ac97->bus->num = cpu_dai->id;
	spin_lock_init(&ac97->bus->bus_lock);
	pcm_link->codec->ac97 = ac97;
	mutex_unlock(&machine->mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_new_ac97_codec);

/**
 * snd_soc_free_ac97_codec - free AC97 codec device
 * @codec: audio codec
 *
 * Frees AC97 codec device resources.
 */
void snd_soc_free_ac97_codec(struct snd_soc_pcm_link *pcm_link)
{
	struct snd_soc_machine *machine = pcm_link->machine;
	struct snd_ac97 *ac97 = pcm_link->codec->ac97;
	
	mutex_lock(&machine->mutex);
	kfree(ac97->bus);
	kfree(ac97);
	pcm_link->codec->ac97 = NULL;
	mutex_unlock(&machine->mutex);
}
EXPORT_SYMBOL_GPL(snd_soc_free_ac97_codec);

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
 * snd_soc_new_card - create new sound card
 * @socdev: the SoC audio device
 *
 * Create a new sound card based upon the machine.
 *
 * Returns 0 for success, else error.
 */
int snd_soc_new_card(struct snd_soc_machine *machine, int num_pcm_links, 
	int idx, const char *xid)
{
	int ret = 0;

	snd_assert(machine->name != NULL, return -EINVAL);
	snd_assert(machine->longname != NULL, return -EINVAL);
	snd_assert(machine->ops != NULL, return -EINVAL);
	snd_assert(machine->ops->mach_probe != NULL, return -EINVAL);
	snd_assert(num_pcm_links != 0, return -EINVAL);
	mutex_init(&machine->mutex);
	mutex_lock(&machine->mutex);
	INIT_LIST_HEAD(&machine->dapm_widgets);
	INIT_LIST_HEAD(&machine->dapm_paths);
	INIT_LIST_HEAD(&machine->active_list);

	/* register a sound card */
	machine->card = snd_card_new(idx, xid, machine->owner, 0);
	if (!machine->card) {
		printk(KERN_ERR "asoc: can't create sound card for machine %s\n",
			machine->name);
		mutex_unlock(&machine->mutex);
		return -ENODEV;
	}

	machine->card->dev = &machine->pdev->dev;
	machine->card->private_data = machine;
	machine->pcm_links_total = num_pcm_links;
	strncpy(machine->card->driver, machine->name, 
		sizeof(machine->card->driver));
	mutex_unlock(&machine->mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_new_card);

/**
 * snd_soc_register_card - register sound card
 * @socdev: the SoC audio device
 *
 * Register a SoC sound card. Also registers an AC97 device if the
 * codec is AC97 for ad hoc devices.
 *
 * Returns 0 for success, else error.
 */
int snd_soc_register_card(struct snd_soc_machine *machine)
{
	struct snd_soc_pcm_link *pcm_link;
	int ret = 0, err = 0;

	snd_assert(machine->card != NULL, return -EINVAL);
	snd_assert(machine->pdev != NULL, return -EINVAL);
	
	mutex_lock(&machine->mutex);
	list_for_each_entry(pcm_link, &machine->active_list, active_list) {
#ifdef CONFIG_SND_SOC_AC97_BUS
		if (pcm_link->codec_dai->type == SND_SOC_DAI_AC97_BUS) {
			ret = soc_ac97_dev_register(pcm_link->codec, 
				(char*)pcm_link->name);
			if (ret < 0) {
				printk(KERN_ERR "asoc: AC97 device register failed\n");
				snd_card_free(machine->card);
				goto out;
			}
		}
#endif
	}
	snprintf(machine->card->shortname, sizeof(machine->card->shortname),
		 "%s", machine->name);
	snprintf(machine->card->longname, sizeof(machine->card->longname),
		 "%s (%s)", machine->name, machine->longname);

	ret = snd_card_register(machine->card);
	if (ret < 0) {
		printk(KERN_ERR "asoc: failed to register soundcard for codec %s\n",
				machine->name);
		goto out;
	}

	err = snd_soc_dapm_sys_add(&machine->pdev->dev);
	if (err < 0)
		printk(KERN_WARNING "asoc: failed to add dapm sysfs entries\n");

	err = device_create_file(&machine->pdev->dev, &dev_attr_codec_reg);
	if (err < 0)
		printk(KERN_WARNING "asoc: failed to add codec sysfs entries\n");
out:
	mutex_unlock(&machine->mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_register_card);

/**
 * snd_soc_machine_free - free sound card and pcms
 * @machine: the SoC audio machine
 *
 * Frees sound card and pcms associated with the machine.
 * Also unregister the codec if it is an AC97 device.
 */
void snd_soc_machine_free(struct snd_soc_machine *machine)
{
	struct snd_soc_codec* codec, *codec_tmp;
	struct snd_soc_dai *dai, *dai_tmp;
	struct snd_soc_platform *platform, *platform_tmp;
#ifdef CONFIG_SND_SOC_AC97_BUS	
	struct snd_soc_pcm_link *pcm_link, *pcm_link_tmp;
#endif

	mutex_lock(&machine->mutex);
#ifdef CONFIG_SND_SOC_AC97_BUS
	list_for_each_entry_safe(pcm_link, pcm_link_tmp,
		&machine->active_list, active_list) {
		if (pcm_link->codec->ac97)
			soc_ac97_dev_unregister(pcm_link->codec);
	}
#endif
	if (machine->card)
		snd_card_free(machine->card);
	device_remove_file(&machine->pdev->dev, &dev_attr_codec_reg);
	mutex_unlock(&machine->mutex);
	
	list_for_each_entry_safe(codec, codec_tmp, &soc_codec_list, list) {
		if (device_is_registered(&codec->dev)) 
			device_unregister(&codec->dev);
	}
	list_for_each_entry_safe(dai, dai_tmp, &soc_codec_dai_list, list) {
		if (device_is_registered(&dai->dev))
			device_unregister(&dai->dev);
	}
	list_for_each_entry_safe(dai, dai_tmp, &soc_cpu_dai_list, list) {
		if (device_is_registered(&dai->dev))
			device_unregister(&dai->dev);
	}
	list_for_each_entry_safe(platform, platform_tmp, 
		&soc_platform_list, list) {
		if (device_is_registered(&platform->dev))
			device_unregister(&platform->dev);
	}
}
EXPORT_SYMBOL_GPL(snd_soc_machine_free);

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

	uinfo->type =
		max == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN : SNDRV_CTL_ELEM_TYPE_INTEGER;
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

	uinfo->type =
		max == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN : SNDRV_CTL_ELEM_TYPE_INTEGER;
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

	uinfo->type =
		max == 1 ? SNDRV_CTL_ELEM_TYPE_BOOLEAN : SNDRV_CTL_ELEM_TYPE_INTEGER;
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

int snd_soc_register_codec_dai(struct snd_soc_dai *dai)
{
	mutex_lock(&list_mutex);
	list_add(&dai->list, &soc_codec_dai_list);
	mutex_unlock(&list_mutex);
	soc_match_components();
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_register_codec_dai);

int snd_soc_register_cpu_dai(struct snd_soc_dai *dai)
{
	mutex_lock(&list_mutex);
	list_add(&dai->list, &soc_cpu_dai_list);
	mutex_unlock(&list_mutex);
	soc_match_components();
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_register_cpu_dai);

int snd_soc_register_codec(struct snd_soc_codec *codec)
{
	mutex_lock(&list_mutex);
	list_add(&codec->list, &soc_codec_list);
	mutex_unlock(&list_mutex);
	soc_match_components();
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_register_codec);

int snd_soc_register_platform(struct snd_soc_platform *platform)
{
	mutex_lock(&list_mutex);
	list_add(&platform->list, &soc_platform_list);
	mutex_unlock(&list_mutex);
	soc_match_components();
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_register_platform);

struct snd_soc_platform *snd_soc_new_platform (struct device *parent,
	const char *platform_id)
{
	struct snd_soc_platform *platform = NULL, *lplatform;
	int err = 0, new_platform = 1;
	
	snd_assert(platform_id != NULL, return -EINVAL);
	
	mutex_lock(&list_mutex);
	list_for_each_entry(lplatform, &soc_platform_list, list) {
		if (!strcmp(lplatform->name, platform_id)) {
			platform = lplatform;
			new_platform = 0;
			break;
		}
	}
	mutex_unlock(&list_mutex);
	if (new_platform) {
		platform = kzalloc(sizeof(struct snd_soc_platform), GFP_KERNEL);
		if (platform == NULL)
			return NULL;
	
		INIT_LIST_HEAD(&platform->list);
		strcpy(platform->dev.bus_id, platform_id);
		strcpy(platform->name, platform_id);
		platform->dev.bus = &asoc_bus_type;
		platform->dev.parent = parent;
		platform->dev.release = soc_platform_dev_release;
		err = device_register(&platform->dev);
		if (err < 0)
			goto dev_err;
	}
	soc_match_components();
	return platform;
dev_err:
	kfree(platform);
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_new_platform);

struct snd_soc_codec *snd_soc_new_codec (struct device *parent, 
	const char *codec_id)
{
	struct snd_soc_codec *codec = NULL, *lcodec;
	int err = 0, new_codec = 1;
	
	snd_assert(codec_id != NULL, return -EINVAL);
	
	mutex_lock(&list_mutex);
	list_for_each_entry(lcodec, &soc_codec_list, list) {
		if (!strcmp(lcodec->name, codec_id)) {
			codec = lcodec;
			new_codec = 0;
			break;
		}
	}
	mutex_unlock(&list_mutex);
	if (new_codec) {
		codec = kzalloc(sizeof(struct snd_soc_codec), GFP_KERNEL);
		if (codec == NULL)
			return NULL;
		INIT_LIST_HEAD(&codec->list);
		INIT_LIST_HEAD(&codec->dai_list);
		mutex_init(&codec->mutex);
		strcpy(codec->dev.bus_id, codec_id);
		strcpy(codec->name, codec_id);
		codec->dev.bus = &asoc_bus_type;
		codec->dev.parent = parent;
		codec->dev.release = soc_codec_dev_release;
		err = device_register(&codec->dev);
		if (err < 0)
			goto dev_err;
	}
	soc_match_components();
	return codec;
dev_err:
	kfree(codec);
	return NULL;
}
EXPORT_SYMBOL_GPL(snd_soc_new_codec);

static struct snd_soc_dai *snd_soc_new_dai (struct device *parent,
	const char *dai_id)
{
	struct snd_soc_dai *dai = NULL, *ldai;
	int err = 0, new_dai = 1;
	
	snd_assert(dai_id != NULL, return -EINVAL);
	
	mutex_lock(&list_mutex);
	list_for_each_entry(ldai, &soc_cpu_dai_list, list) {
		if (!strcmp(ldai->name, dai_id)) {
			dai = ldai;
			new_dai = 0;
			goto out;
		}
	}
	list_for_each_entry(ldai, &soc_codec_dai_list, list) {
		if (!strcmp(ldai->name, dai_id)) {
			dai = ldai;
			new_dai = 0;
			goto out;
		}
	}
out:
	mutex_unlock(&list_mutex);
	if (new_dai) {
		dai = kzalloc(sizeof(struct snd_soc_dai), GFP_KERNEL);
		if (dai == NULL)
			return NULL;
		INIT_LIST_HEAD(&dai->list);
		INIT_LIST_HEAD(&dai->codec_list);
		strcpy(dai->dev.bus_id, dai_id);
		strcpy(dai->name, dai_id);
		dai->dev.bus = &asoc_bus_type;
		dai->dev.parent = parent;
		dai->dev.release = soc_dai_dev_release;
		err = device_register(&dai->dev);
		if (err < 0)
			goto dev_err;
	}
	soc_match_components();
	return dai;
dev_err:
	kfree(dai);
	return NULL;	
}
EXPORT_SYMBOL_GPL(snd_soc_new_dai);

struct snd_soc_pcm_link *snd_soc_pcm_link_new(
	struct snd_soc_machine *machine, const char *name,
	const struct snd_soc_pcm_link_ops *link_ops, 
	const char *platform_id, const char *codec_id, 
	const char *codec_dai_id, const char *cpu_dai_id)
{
	struct snd_soc_pcm_link *pcm_link;

	snd_assert(name != NULL, return -EINVAL);
	snd_assert(codec_dai_id != NULL, return -EINVAL);
	snd_assert(cpu_dai_id != NULL, return -EINVAL);
	snd_assert(codec_id != NULL, return -EINVAL);
	snd_assert(platform_id != NULL, return -EINVAL);
	
	pcm_link = kzalloc(sizeof(struct snd_soc_pcm_link), GFP_KERNEL);
	if (pcm_link == NULL)
		return NULL;

	INIT_LIST_HEAD(&pcm_link->active_list);
	INIT_LIST_HEAD(&pcm_link->all_list);
	INIT_DELAYED_WORK(&pcm_link->delayed_work, close_delayed_work);
	pcm_link->machine = machine;
	pcm_link->link_ops = link_ops;
	pcm_link->codec_id = codec_id;
	pcm_link->cpu_dai_id = cpu_dai_id;
	pcm_link->codec_dai_id = codec_dai_id;
	pcm_link->platform_id = platform_id;
	strcpy(pcm_link->name, name);
	mutex_lock(&list_mutex);
	list_add(&pcm_link->all_list, &soc_pcm_link_list);
	mutex_unlock(&list_mutex);
	soc_match_components();
	
	return pcm_link;
}
EXPORT_SYMBOL_GPL(snd_soc_pcm_link_new);

int snd_soc_pcm_link_attach(struct snd_soc_pcm_link *pcm_link)
{
	struct device *parent;
	struct snd_soc_codec *codec;
	struct snd_soc_dai *codec_dai, *cpu_dai;
	struct snd_soc_platform *platform;	
	
	snd_assert(pcm_link->machine->pdev != NULL, return -EINVAL);
	parent = &pcm_link->machine->pdev->dev;
	
	platform = snd_soc_new_platform(parent, pcm_link->platform_id);
	if (platform == NULL)
		goto platform_new_err;
	codec_dai = snd_soc_new_dai(parent, pcm_link->codec_dai_id);
	if (codec_dai == NULL)
		goto codec_dai_new_err;
	cpu_dai = snd_soc_new_dai(parent, pcm_link->cpu_dai_id);
	if (cpu_dai == NULL)
		goto cpu_dai_new_err;
	codec = snd_soc_new_codec(parent, pcm_link->codec_id);
	if (codec == NULL)
		goto codec_new_err;
		
	soc_match_components();	
	return 0;

codec_new_err:
	device_unregister(&cpu_dai->dev);
cpu_dai_new_err:
	device_unregister(&codec_dai->dev);
codec_dai_new_err:
	device_unregister(&platform->dev);
platform_new_err:
	printk(KERN_ERR "asoc: failed to create pcm link\n");
	return -ENODEV;
}
EXPORT_SYMBOL_GPL(snd_soc_pcm_link_attach);

static int asoc_bus_match(struct device *dev, struct device_driver *drv)
{	
	if (strstr(dev->bus_id, drv->name)) //lg also check id
		return 1;
	return 0;
}

struct bus_type asoc_bus_type = {
	.name		= "asoc",
	.match		= asoc_bus_match,
};
EXPORT_SYMBOL(asoc_bus_type);

static int __init asoc_bus_init(void)
{	
	printk(KERN_INFO "ASoC version %s\n", SND_SOC_VERSION);
	return bus_register(&asoc_bus_type);
}
subsys_initcall(asoc_bus_init);

static void __exit asoc_bus_exit(void)
{
	bus_unregister(&asoc_bus_type);
}

module_exit(asoc_bus_exit);

/* Module information */
MODULE_AUTHOR("Liam Girdwood, liam.girdwood@wolfsonmicro.com, www.wolfsonmicro.com");
MODULE_DESCRIPTION("ALSA SoC Core");
MODULE_LICENSE("GPL");
