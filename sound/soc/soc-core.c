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
#include <linux/string.h>
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
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_platform *platform = pcm_runtime->platform;
	struct snd_soc_dai_runtime *cpu_rdai = pcm_runtime->cpu_dai;
	struct snd_soc_dai_runtime *codec_rdai = pcm_runtime->codec_dai;
	struct snd_soc_codec *codec = pcm_runtime->codec;
	int ret = 0;

	mutex_lock(&pcm_mutex);

	/* startup the audio subsystem */
	if (cpu_rdai->dai->startup) {
		ret = cpu_rdai->dai->startup(substream, cpu_rdai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't open interface %s\n",
				cpu_rdai->dai->name);
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

	if (codec_rdai->dai->startup) {
		ret = codec_rdai->dai->startup(substream, codec_rdai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't open codec %s\n",
				codec_rdai->dai->name);
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

	/* Check that the codec and cpu DAI's are compatible */
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		struct snd_soc_dai_caps *codec_caps = 
			&codec_rdai->dai->playback;
		struct snd_soc_dai_caps *cpu_caps = 
			&cpu_rdai->dai->playback;
		
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
		struct snd_soc_dai_caps *codec_caps = 
			&codec_rdai->dai->capture;
		struct snd_soc_dai_caps *cpu_caps = 
			&cpu_rdai->dai->capture;
			
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
			codec_rdai->dai->name, cpu_rdai->dai->name);
		goto pcm_runtime_err;
	}
	if (!runtime->hw.formats) {
		printk(KERN_ERR "asoc: %s <-> %s No matching formats\n",
			codec_rdai->dai->name, cpu_rdai->dai->name);
		goto pcm_runtime_err;
	}
	if (!runtime->hw.channels_min || !runtime->hw.channels_max) {
		printk(KERN_ERR "asoc: %s <-> %s No matching channels\n",
			codec_rdai->dai->name, cpu_rdai->dai->name);
		goto pcm_runtime_err;
	}

	dbg("asoc: %s <-> %s info:\n",codec_rdai->dai->name, cpu_rdai->dai->name);
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
	if (cpu_rdai->dai->shutdown)
		cpu_rdai->dai->shutdown(substream, cpu_rdai);
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
	struct snd_soc_dai_runtime *codec_rdai = pcm_runtime->codec_dai;
	struct snd_soc_machine *machine = pcm_runtime->machine;

	mutex_lock(&pcm_mutex);

	dbg("pop wq checking: %s status: %s waiting: %s\n",
		codec_rdai->dai->playback,
		pcm_runtime->playback_active ? "active" : "inactive",
		pcm_runtime->pop_wait ? "yes" : "no");

	/* are we waiting on this codec DAI stream */
	if (pcm_runtime->pop_wait == 1) {

		/* power down the codec to D1 if no longer active */
		if (codec->active == 0) {
			dbg("pop wq D1 %s %s\n", codec->name,
				codec_rdai->dai->playback.stream_name);
			snd_soc_dapm_device_event(pcm_runtime, 
				SND_SOC_BIAS_PREPARE);
		}

		pcm_runtime->pop_wait = 0;
		snd_soc_dapm_stream_event(machine, 
			codec_rdai->dai->playback.stream_name,
			SND_SOC_DAPM_STREAM_STOP);

		/* power down the codec power domain if no longer active */
		if (codec->active == 0) {
			dbg("pop wq D3 %s %s\n", codec->name,
				codec_rdai->dai->playback.stream_name);
	 		snd_soc_dapm_device_event(pcm_runtime, 
	 			SND_SOC_BIAS_STANDBY);
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
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_platform *platform = pcm_runtime->platform;
	struct snd_soc_dai_runtime *cpu_rdai = pcm_runtime->cpu_dai;
	struct snd_soc_dai_runtime *codec_rdai = pcm_runtime->codec_dai;
	struct snd_soc_codec *codec = pcm_runtime->codec;
	struct snd_soc_machine *machine = pcm_runtime->machine;

	mutex_lock(&pcm_mutex);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		pcm_runtime->playback_active = 0;
	else
		pcm_runtime->capture_active = 0;
	codec->active--;

	if (cpu_rdai->dai->shutdown)
		cpu_rdai->dai->shutdown(substream, cpu_rdai);

	if (codec_rdai->dai->shutdown)
		codec_rdai->dai->shutdown(substream, codec_rdai);

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
		snd_soc_dapm_stream_event(machine,
			codec_rdai->dai->capture.stream_name, 
			SND_SOC_DAPM_STREAM_STOP);

		if (codec->active == 0 && pcm_runtime->pop_wait == 0)
			snd_soc_dapm_device_event(pcm_runtime, 
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
	struct snd_soc_dai_runtime *cpu_rdai = pcm_runtime->cpu_dai;
	struct snd_soc_dai_runtime *codec_rdai = pcm_runtime->codec_dai;
	struct snd_soc_codec *codec = pcm_runtime->codec;
	struct snd_soc_machine *machine = pcm_runtime->machine;
	int ret = 0;

	mutex_lock(&pcm_mutex);

	if (pcm_runtime->ops->prepare) {
		ret = pcm_runtime->ops->prepare(substream);
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

	if (codec_rdai->dai->prepare) {
		ret = codec_rdai->dai->prepare(substream, codec_rdai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: codec DAI prepare error\n");
			goto out;
		}
	}

	if (cpu_rdai->dai->prepare) {
		ret = cpu_rdai->dai->prepare(substream, cpu_rdai);
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
				snd_soc_dapm_stream_event(machine,
					codec_rdai->dai->capture.stream_name,
					SND_SOC_DAPM_STREAM_START);
		else {
			pcm_runtime->pop_wait = 0;
			cancel_delayed_work(&pcm_runtime->delayed_work);
			if (codec_rdai->dai->digital_mute)
				codec_rdai->dai->digital_mute(codec_rdai, 0);
		}
	} else {
		/* no delayed work - do we need to power up codec */
		if (codec->dapm_state != SND_SOC_BIAS_ON) {

			snd_soc_dapm_device_event(pcm_runtime, SND_SOC_BIAS_PREPARE);

			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				snd_soc_dapm_stream_event(machine,
					codec_rdai->dai->playback.stream_name,
					SND_SOC_DAPM_STREAM_START);
			else
				snd_soc_dapm_stream_event(machine,
					codec_rdai->dai->capture.stream_name,
					SND_SOC_DAPM_STREAM_START);

			snd_soc_dapm_device_event(pcm_runtime, SND_SOC_BIAS_ON);
			if (codec_rdai->dai->digital_mute)
				codec_rdai->dai->digital_mute(codec_rdai, 0);
		} else {
			/* codec already powered - power on widgets */
			if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
				snd_soc_dapm_stream_event(machine,
					codec_rdai->dai->playback.stream_name,
					SND_SOC_DAPM_STREAM_START);
			else
				snd_soc_dapm_stream_event(machine,
					codec_rdai->dai->capture.stream_name,
					SND_SOC_DAPM_STREAM_START);
			if (codec_rdai->dai->digital_mute)
				codec_rdai->dai->digital_mute(codec_rdai, 0);
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
	struct snd_soc_dai_runtime *cpu_rdai = pcm_runtime->cpu_dai;
	struct snd_soc_dai_runtime *codec_rdai = pcm_runtime->codec_dai;
	int ret = 0;

	mutex_lock(&pcm_mutex);

	if (pcm_runtime->ops->hw_params) {
		ret = pcm_runtime->ops->hw_params(substream, params);
		if (ret < 0) {
			printk(KERN_ERR "asoc: machine hw_params failed\n");
			goto out;
		}
	}

	if (codec_rdai->dai->hw_params) {
		ret = codec_rdai->dai->hw_params(substream, params, codec_rdai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't set codec %s hw params\n",
				codec_rdai->dai->name);
			goto codec_err;
		}
	}

	if (cpu_rdai->dai->hw_params) {
		ret = cpu_rdai->dai->hw_params(substream, params, cpu_rdai);
		if (ret < 0) {
			printk(KERN_ERR "asoc: can't set interface %s hw params\n",
				cpu_rdai->dai->name);
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
	if (cpu_rdai->dai->hw_free)
		cpu_rdai->dai->hw_free(substream, cpu_rdai);

interface_err:
	if (codec_rdai->dai->hw_free)
		codec_rdai->dai->hw_free(substream, codec_rdai);

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
	struct snd_soc_dai_runtime *cpu_rdai = pcm_runtime->cpu_dai;
	struct snd_soc_dai_runtime *codec_rdai = pcm_runtime->codec_dai;
	struct snd_soc_codec *codec = pcm_runtime->codec;

	mutex_lock(&pcm_mutex);

	/* apply codec digital mute */
	if (!codec->active && codec_rdai->dai->digital_mute)
		codec_rdai->dai->digital_mute(codec_rdai, 1);

	/* free any machine hw params */
	if (pcm_runtime->ops->hw_free)
		pcm_runtime->ops->hw_free(substream);

	/* free any DMA resources */
	if (platform->pcm_ops->hw_free)
		platform->pcm_ops->hw_free(substream);

	/* now free hw params for the DAI's  */
	if (codec_rdai->dai->hw_free)
		codec_rdai->dai->hw_free(substream, codec_rdai);

	if (cpu_rdai->dai->hw_free)
		cpu_rdai->dai->hw_free(substream, cpu_rdai);

	mutex_unlock(&pcm_mutex);
	return 0;
}

static int soc_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_platform *platform = pcm_runtime->platform;
	struct snd_soc_dai_runtime *cpu_rdai = pcm_runtime->cpu_dai;
	struct snd_soc_dai_runtime *codec_rdai = pcm_runtime->codec_dai;
	int ret;

	if (codec_rdai->dai->trigger) {
		ret = codec_rdai->dai->trigger(substream, cmd, codec_rdai);
		if (ret < 0)
			return ret;
	}

	if (platform->pcm_ops->trigger) {
		ret = platform->pcm_ops->trigger(substream, cmd);
		if (ret < 0)
			return ret;
	}

	if (cpu_rdai->dai->trigger) {
		ret = cpu_rdai->dai->trigger(substream, cmd, cpu_rdai);
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
	struct snd_soc_dai_runtime *codec_rdai;
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_runtime *pcm_runtime;
	char *stream;

	if (machine->suspend_early)
		machine->suspend_early(machine, state);
		
	/* mute any active DAC's */
	list_for_each_entry(pcm_runtime, &machine->pcm_list, list) {
		codec_rdai = pcm_runtime->codec_dai;
		if (codec_rdai->dai->digital_mute &&
			pcm_runtime->playback_active)
			codec_rdai->dai->digital_mute(codec_rdai, 1);
	}

	snd_power_change_state(machine->card, SND_SOC_BIAS_OFF);
	
	/* suspend all pcm's */
	list_for_each_entry(pcm_runtime, &machine->pcm_list, list)
		snd_pcm_suspend_all(pcm_runtime->pcm);

	/* close any waiting streams and save state */
	list_for_each_entry(pcm_runtime, &machine->pcm_list, list) {
		codec = pcm_runtime->codec;
		run_delayed_work(&pcm_runtime->delayed_work);
		codec->suspend_dapm_state = codec->dapm_state;
	}

	list_for_each_entry(pcm_runtime, &machine->pcm_list, list) {
		codec_rdai = pcm_runtime->codec_dai;
		stream = codec_rdai->dai->playback.stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(machine, stream,
				SND_SOC_DAPM_STREAM_SUSPEND);
		stream = codec_rdai->dai->capture.stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(machine, stream,
				SND_SOC_DAPM_STREAM_SUSPEND);
	}

	list_for_each_entry(codec, &machine->codec_list, list) {
		if (codec->dev.driver->suspend)
			codec->dev.driver->suspend(&codec->dev, state);
	}

	list_for_each_entry(platform, &machine->platform_list, list) {
		if (platform->dev.driver->suspend)
			platform->dev.driver->suspend(&platform->dev, state);
	}

	if (machine->suspend_late)
		machine->suspend_late(machine, state);

	return 0;
}

/* powers up audio subsystem after a suspend */
int snd_soc_resume(struct snd_soc_machine *machine)
{
 	struct snd_soc_platform *platform;
	struct snd_soc_dai_runtime *codec_rdai;
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_runtime *pcm_runtime;
	char *stream;
	
	if (machine->resume_early)
		machine->resume_early(machine);

	list_for_each_entry(platform, &machine->platform_list, list) {
		if (platform->dev.driver->resume)
			platform->dev.driver->resume(&platform->dev);
	}

	list_for_each_entry(codec, &machine->codec_list, list) {
		if (codec->dev.driver->resume)
			codec->dev.driver->resume(&codec->dev);
	}

	list_for_each_entry(pcm_runtime, &machine->pcm_list, list) {
		codec_rdai = pcm_runtime->codec_dai;
		stream = codec_rdai->dai->playback.stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(machine, stream,
				SND_SOC_DAPM_STREAM_RESUME);
		stream = codec_rdai->dai->capture.stream_name;
		if (stream != NULL)
			snd_soc_dapm_stream_event(machine, stream,
				SND_SOC_DAPM_STREAM_RESUME);
	}

	/* unmute any active DAC's */
	list_for_each_entry(pcm_runtime, &machine->pcm_list, list) {
		codec_rdai = pcm_runtime->codec_dai;
		if (codec_rdai->dai->digital_mute &&
			pcm_runtime->playback_active)
			codec_rdai->dai->digital_mute(codec_rdai, 0);
	}

	if (machine->resume_late)
		machine->resume_late(machine);

	snd_power_change_state(machine->card, SND_SOC_BIAS_STANDBY);
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

/* codec register dump */
static ssize_t codec_reg_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct snd_soc_codec *codec = NULL;//liam - to_snd_soc_codec(dev);
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

/**
 * snd_soc_new_ac97_codec - initailise AC97 device
 * @codec: audio codec
 * @ops: AC97 bus operations
 * @num: AC97 codec number
 *
 * Initialises AC97 codec resources for use by ad-hoc devices only.
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
void snd_soc_free_ac97_codec(struct snd_soc_pcm_runtime *pcm_runtime)
{
	struct snd_soc_machine *machine = pcm_runtime->machine;
	struct snd_ac97 *ac97 = pcm_runtime->codec->ac97;
	
	mutex_lock(&machine->mutex);
	kfree(ac97->bus);
	kfree(ac97);
	pcm_runtime->codec->ac97 = NULL;
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

int snd_soc_dai_set_sysclk(struct snd_soc_dai_runtime *rdai, int clk_id, 
	unsigned int freq, int dir)
{
	if (rdai->dai->set_sysclk)
		return rdai->dai->set_sysclk(rdai, clk_id, freq, dir);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_sysclk);

int snd_soc_dai_set_clkdiv(struct snd_soc_dai_runtime *rdai, 
	int div_id, int div)
{
	if (rdai->dai->set_clkdiv)
		return rdai->dai->set_clkdiv(rdai, div_id, div);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_clkdiv);

int snd_soc_dai_set_pll(struct snd_soc_dai_runtime *rdai,
	int pll_id, unsigned int freq_in, unsigned int freq_out)
{
	if (rdai->dai->set_pll)
		return rdai->dai->set_pll(rdai, pll_id, freq_in, freq_out);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_pll);

int snd_soc_dai_set_fmt(struct snd_soc_dai_runtime *rdai, unsigned int fmt)
{
	if (rdai->dai->set_fmt)
		return rdai->dai->set_fmt(rdai, fmt);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_fmt);

int snd_soc_dai_set_tdm_slot(struct snd_soc_dai_runtime *rdai,
	unsigned int mask, int slots)
{
	if (rdai->dai->set_sysclk)
		return rdai->dai->set_tdm_slot(rdai, mask, slots);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_tdm_slot);

int snd_soc_dai_set_tristate(struct snd_soc_dai_runtime *rdai, int tristate)
{
	if (rdai->dai->set_sysclk)
		return rdai->dai->set_tristate(rdai, tristate);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_set_tristate);

int snd_soc_dai_digital_mute(struct snd_soc_dai_runtime *rdai, int mute)
{
	if (rdai->dai->digital_mute)
		return rdai->dai->digital_mute(rdai, mute);
	else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_soc_dai_digital_mute);

int snd_soc_codec_add_dai(struct snd_soc_codec *codec, 
	struct snd_soc_dai *dai, int num)
{
	struct snd_soc_dai_runtime *dai_runtime;
	int i;

	mutex_lock(&codec->mutex);

	for (i = 0; i < num; i++) {
		dai_runtime = kzalloc(sizeof(*dai_runtime), GFP_KERNEL);
		if (dai_runtime == NULL)
			goto err;
			
		dai_runtime->dai = dai;
		dai_runtime->codec = codec;
		
		list_add(&dai_runtime->list, &codec->dai_list);
	}
	mutex_unlock(&codec->mutex);
	return 0;
err:
	list_for_each_entry(dai_runtime, &codec->dai_list, list)
		kfree(dai_runtime);
	mutex_unlock(&codec->mutex);
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(snd_soc_codec_add_dai);

int snd_soc_register_codec(struct snd_soc_codec *codec)
{
	if (list_empty(&codec->dai_list))
		return -EINVAL;
// lrg - add sysfs

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_register_codec);

void snd_soc_unregister_codec(struct snd_soc_codec *codec)
{		
	struct snd_soc_dai_runtime *dai_runtime, *d;

	list_for_each_entry_safe(dai_runtime, d, &codec->dai_list, list)
		kfree(dai_runtime);
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_codec);

int snd_soc_platform_add_dai(struct snd_soc_platform *platform, 
	struct snd_soc_dai *dai, int num)
{
	struct snd_soc_dai_runtime *dai_runtime;
	int i;
	
	mutex_lock(&platform->mutex);
	for (i = 0; i < num; i++) {
		dai_runtime = kzalloc(sizeof(*dai_runtime), GFP_KERNEL);
		if (dai_runtime == NULL)
			goto err;
			
		dai_runtime->dai = dai;
		dai_runtime->platform = platform;
		
		list_add(&dai_runtime->list, &platform->dai_list);
	}
	mutex_unlock(&platform->mutex);
	return 0;
err:
	list_for_each_entry(dai_runtime, &platform->dai_list, list)
		kfree(dai_runtime);
	mutex_unlock(&platform->mutex);
	return -ENOMEM;
}
EXPORT_SYMBOL_GPL(snd_soc_platform_add_dai);

int snd_soc_register_platform(struct snd_soc_platform *platform)
{
	if (list_empty(&platform->dai_list))
		return -EINVAL;
	
	
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_register_platform);

void snd_soc_unregister_platform(struct snd_soc_platform *platform)
{		
	struct snd_soc_dai_runtime *dai_runtime, *d;

	list_for_each_entry_safe(dai_runtime, d, &platform->dai_list, list)
		kfree(dai_runtime);
}
EXPORT_SYMBOL_GPL(snd_soc_unregister_platform);

struct snd_soc_machine *snd_soc_machine_create(const char *name, 
	struct device *parent, int idx, const char *xid)
{
	struct snd_soc_machine *machine;

	machine = kzalloc(sizeof(*machine), GFP_KERNEL);
	if (machine == NULL)
		return NULL;
	
	machine->name = kstrdup(name, GFP_KERNEL);
	if (machine->name == NULL) {
		kfree(machine);
		return NULL;
	}
	
	machine->dev = parent;
	mutex_init(&machine->mutex);
	INIT_LIST_HEAD(&machine->dapm_widgets);
	INIT_LIST_HEAD(&machine->dapm_paths);
	INIT_LIST_HEAD(&machine->codec_list);
	INIT_LIST_HEAD(&machine->platform_list);
	INIT_LIST_HEAD(&machine->pcm_list);

	/* register a sound card */
	machine->card = snd_card_new(idx, xid, THIS_MODULE, 0);
	if (!machine->card) {
		printk(KERN_ERR "asoc: can't create sound card for machine %s\n",
			machine->name);
		kfree(machine);
		return ERR_PTR(-ENODEV);
	}
	machine->card->dev = parent;
	return machine;
}
EXPORT_SYMBOL_GPL(snd_soc_machine_create);

static void codec_dev_release(struct device *dev)
{
	struct snd_soc_codec *codec = to_snd_soc_codec(dev);
	kfree(codec);
}

int snd_soc_codec_create(struct snd_soc_machine *machine,
	const char *codec_id)
{
	struct snd_soc_codec *codec;
	int ret;
	
	codec = kzalloc(sizeof(*codec), GFP_KERNEL);
	if (codec == NULL)
		return -ENOMEM;

	strcpy(codec->dev.bus_id, codec_id);
	codec->dev.bus = &asoc_bus_type;
	codec->dev.parent = machine->dev;
	codec->dev.release = codec_dev_release;
	codec->machine = machine;
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dai_list);

	ret = device_register(&codec->dev);
	if (ret < 0) {
		printk(KERN_ERR "failed to register codec device\n");
		kfree(codec);
		return ret;
	}
	list_add(&codec->list, &machine->codec_list);
	
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_codec_create);

static void snd_platform_dev_release(struct device *dev)
{
	struct snd_soc_platform *platform = to_snd_soc_platform(dev);
	kfree(platform);
}

int snd_soc_platform_create(struct snd_soc_machine *machine,
	const char *platform_id)
{
	struct snd_soc_platform *platform;
	int ret;

	platform = kzalloc(sizeof(*platform), GFP_KERNEL);
	if (platform == NULL)
		return -ENOMEM;

	strcpy(platform->dev.bus_id, platform_id);
	platform->dev.bus = &asoc_bus_type;
	platform->dev.parent = machine->dev;
	platform->dev.release = snd_platform_dev_release;
	platform->machine = machine;
	mutex_init(&platform->mutex);
	INIT_LIST_HEAD(&platform->dai_list);
	
	ret = device_register(&platform->dev);
	if (ret < 0) {
		printk(KERN_ERR "failed to register platform device\n");
		kfree(platform);
		return ret;
	}
	
	list_add(&platform->list, &machine->platform_list);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_platform_create);

int snd_soc_pcm_create(struct snd_soc_machine *machine,
	struct snd_soc_ops *pcm_ops, int codec_dai_id, 
	int platform_dai_id, int playback, int capture)
{	
	struct snd_soc_pcm_runtime *pcm_runtime;
	struct snd_soc_dai_runtime *dai_runtime, 
		*codec_rdai = NULL, *cpu_rdai = NULL;
	struct snd_soc_codec *codec;
	struct snd_soc_platform *platform;
	struct snd_pcm *pcm;
	int ret = 0;
	
	if (list_empty(&machine->codec_list))
		return -ENODEV;

	if (list_empty(&machine->platform_list))
		return -ENODEV;

	/* find codec dai */
	list_for_each_entry(codec, &machine->codec_list, list) {
		list_for_each_entry(dai_runtime, &codec->dai_list, list) {
			if (codec_dai_id == dai_runtime->dai->id) {
				codec_rdai = dai_runtime;
				break;
			}
		}
	}
	if (!codec_rdai)
		return -EINVAL;

	/* find platform dai */
	list_for_each_entry(platform, &machine->platform_list, list) {
		list_for_each_entry(dai_runtime, &platform->dai_list, list) {
			if (platform_dai_id == dai_runtime->dai->id) {
				cpu_rdai = dai_runtime;
				break;
			}
		}		
	}
	if (!cpu_rdai)
		return -EINVAL;

	pcm_runtime = kzalloc(sizeof(*pcm_runtime), GFP_KERNEL);
	if (!pcm_runtime)
		return -ENOMEM;

	INIT_DELAYED_WORK(&pcm_runtime->delayed_work, close_delayed_work);
	pcm_runtime->codec_dai = codec_rdai;
	pcm_runtime->cpu_dai = cpu_rdai;
	pcm_runtime->machine = machine;
	codec = pcm_runtime->codec = codec_rdai->codec;
	platform = pcm_runtime->platform = cpu_rdai->platform;

	ret = snd_pcm_new(machine->card, (char*)pcm_runtime->name, 
		machine->pcms++, playback, capture, &pcm);
	if (ret < 0) {
		printk(KERN_ERR "asoc: can't create pcm for codec %s\n", codec->name);
		kfree(pcm_runtime);
		return ret;
	}

	pcm_runtime->pcm = pcm;
	pcm_runtime->ops = pcm_ops;
	pcm->private_data = pcm_runtime;
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

	ret = platform->pcm_new(platform, machine->card, 
		playback, capture, pcm);
	if (ret < 0) {
		printk(KERN_ERR "asoc: platform pcm constructor failed\n");
		kfree(pcm_runtime);
		return ret;
	}

	pcm->private_free = platform->pcm_free;
	printk(KERN_INFO "asoc: %s <-> %s mapping ok\n", codec_rdai->dai->name,
		cpu_rdai->dai->name);
	list_add(&pcm_runtime->list, &machine->pcm_list);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_pcm_create);

int snd_soc_machine_register(struct snd_soc_machine *machine)
{
	struct snd_soc_pcm_runtime *pcm_runtime;
	int ret, err;
	
	if (list_empty(&machine->pcm_list))
		return -ENODEV;
	if (list_empty(&machine->codec_list))
		return -ENODEV;
	if (list_empty(&machine->platform_list))
		return -ENODEV;
	
	list_for_each_entry(pcm_runtime, &machine->pcm_list, list) {
#ifdef CONFIG_SND_SOC_AC97_BUS
		if (pcm_runtime->cpu_dai->dai->ac97_control) {
			ret = soc_ac97_dev_register(pcm_runtime->codec, 
				(char*)pcm_runtime->name);
			if (ret < 0) {
				printk(KERN_ERR "asoc: AC97 device register failed\n");
				goto out;
			}
		}
#endif
	}
	
	if (machine->init) {
		ret = machine->init(machine);
		if (ret < 0)
			goto out;
	}
	mutex_lock(&machine->mutex);
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

	err = snd_soc_dapm_sys_add(machine->dev);
	if (err < 0)
		printk(KERN_WARNING "asoc: failed to add dapm sysfs entries\n");

	err = device_create_file(machine->dev, &dev_attr_codec_reg);
	if (err < 0)
		printk(KERN_WARNING "asoc: failed to add codec sysfs entries\n");
out:
	mutex_unlock(&machine->mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(snd_soc_machine_register);

void snd_soc_machine_free(struct snd_soc_machine *machine)
{
	struct snd_soc_codec *codec, *c;
	struct snd_soc_platform *platform, *p;
	struct snd_soc_pcm_runtime *pcm_runtime, *_pcm_runtime;
		
	snd_card_free(machine->card);
	
	list_for_each_entry_safe(pcm_runtime, _pcm_runtime, &machine->pcm_list, list)
		run_delayed_work(&pcm_runtime->delayed_work);
	
	if (machine->exit)
		machine->exit(machine);

	list_for_each_entry_safe(pcm_runtime, _pcm_runtime, &machine->pcm_list, list) {
#ifdef CONFIG_SND_SOC_AC97_BUS
		if (pcm_runtime->cpu_dai->dai->ac97_control)
			soc_ac97_dev_unregister(pcm_runtime->codec);
#endif
		kfree(pcm_runtime);
	}

	list_for_each_entry_safe(platform, p, &machine->platform_list, list)
		device_unregister(&platform->dev);

	list_for_each_entry_safe(codec, c, &machine->codec_list, list)
		device_unregister(&codec->dev);

	kfree(machine);
}
EXPORT_SYMBOL_GPL(snd_soc_machine_free);

static int asoc_bus_match(struct device *dev, struct device_driver *drv)
{      
	if (strstr(dev->bus_id, drv->name))
		return 1;
	return 0;
}

struct bus_type asoc_bus_type = {
       .name           = "asoc",
       .match          = asoc_bus_match,
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
