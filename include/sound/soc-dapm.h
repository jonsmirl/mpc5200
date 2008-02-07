/*
 * linux/sound/soc-dapm.h -- ALSA SoC Dynamic Audio Power Management
 *
 * Author:		Liam Girdwood
 * Created:		Aug 11th 2005
 * Copyright:	Wolfson Microelectronics. PLC.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LINUX_SND_SOC_DAPM_H
#define __LINUX_SND_SOC_DAPM_H

#include <linux/device.h>
#include <linux/types.h>
#include <sound/control.h>

/* widget has no PM register bit */
#define SND_SOC_NOPM	-1

/*
 * SoC dynamic audio power managment
 *
 * We can have upto 4 power domains
 *  1. Codec domain - VREF, VMID
 *     Usually controlled at codec probe/remove, although can be set
 *     at stream time if power is not needed for sidetone, etc.
 *  2. Platform/Machine domain - physically connected inputs and outputs
 *     Is platform/machine and user action specific, is set in the machine
 *     driver and by userspace e.g when HP are inserted
 *  3. Path domain - Internal codec path mixers
 *     Are automatically set when mixer and mux settings are
 *     changed by the user.
 *  4. Stream domain - DAC's and ADC's.
 *     Enabled when stream playback/capture is started.
 */

/* codec domain */
#define SND_SOC_DAPM_VMID(wname) \
{	.id = snd_soc_dapm_vmid, .name = wname, .kcontrols = NULL, \
	.num_kcontrols = 0}

/* platform domain */
#define SND_SOC_DAPM_INPUT(wname) \
{	.id = snd_soc_dapm_input, .name = wname, .kcontrols = NULL, \
	.num_kcontrols = 0}
#define SND_SOC_DAPM_OUTPUT(wname) \
{	.id = snd_soc_dapm_output, .name = wname, .kcontrols = NULL, \
	.num_kcontrols = 0}
#define SND_SOC_DAPM_MIC(wname, wevent) \
{	.id = snd_soc_dapm_mic, .name = wname, .kcontrols = NULL, \
	.num_kcontrols = 0, .event = wevent, \
	.event_flags = SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD}
#define SND_SOC_DAPM_HP(wname, wevent) \
{	.id = snd_soc_dapm_hp, .name = wname, .kcontrols = NULL, \
	.num_kcontrols = 0, .event = wevent, \
	.event_flags = SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD}
#define SND_SOC_DAPM_SPK(wname, wevent) \
{	.id = snd_soc_dapm_spk, .name = wname, .kcontrols = NULL, \
	.num_kcontrols = 0, .event = wevent, \
	.event_flags = SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD}
#define SND_SOC_DAPM_LINE(wname, wevent) \
{	.id = snd_soc_dapm_line, .name = wname, .kcontrols = NULL, \
	.num_kcontrols = 0, .event = wevent, \
	.event_flags = SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_PRE_PMD}

/* path domain */
#define SND_SOC_DAPM_PGA(wname, wreg, wshift, winvert,\
	 wcontrols, wncontrols) \
{	.id = snd_soc_dapm_pga, .name = wname, .reg = wreg, .shift = wshift, \
	.invert = winvert, .kcontrols = wcontrols, .num_kcontrols = wncontrols}
#define SND_SOC_DAPM_MIXER(wname, wreg, wshift, winvert, \
	 wcontrols, wncontrols)\
{	.id = snd_soc_dapm_mixer, .name = wname, .reg = wreg, .shift = wshift, \
	.invert = winvert, .kcontrols = wcontrols, .num_kcontrols = wncontrols}
#define SND_SOC_DAPM_MICBIAS(wname, wreg, wshift, winvert) \
{	.id = snd_soc_dapm_micbias, .name = wname, .reg = wreg, .shift = wshift, \
	.invert = winvert, .kcontrols = NULL, .num_kcontrols = 0}
#define SND_SOC_DAPM_SWITCH(wname, wreg, wshift, winvert, wcontrols) \
{	.id = snd_soc_dapm_switch, .name = wname, .reg = wreg, .shift = wshift, \
	.invert = winvert, .kcontrols = wcontrols, .num_kcontrols = 1}
#define SND_SOC_DAPM_MUX(wname, wreg, wshift, winvert, wcontrols) \
{	.id = snd_soc_dapm_mux, .name = wname, .reg = wreg, .shift = wshift, \
	.invert = winvert, .kcontrols = wcontrols, .num_kcontrols = 1}

/* dapm event condition flags */
#define SND_SOC_DAPM_PRE_PMU	0x1	/* before widget power up */
#define SND_SOC_DAPM_POST_PMU	0x2	/* after widget power up */
#define SND_SOC_DAPM_PRE_PMD	0x4	/* before widget power down */
#define SND_SOC_DAPM_POST_PMD	0x8	/* after widget power down */
#define SND_SOC_DAPM_PRE_REG	0x10	/* before audio path setup */
#define SND_SOC_DAPM_POST_REG	0x20	/* after audio path setup */

/* convenience event condition detection */
#define SND_SOC_DAPM_EVENT_ON(e)	\
	(e & (SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMU))
#define SND_SOC_DAPM_EVENT_OFF(e)	\
	(e & (SND_SOC_DAPM_PRE_PMD | SND_SOC_DAPM_POST_PMD))

/* DAPM Path domain with event callback.
 *
 * Event handler must return 0 for success. Event flags detailed above.
 */
#define SND_SOC_DAPM_PGA_E(wname, wreg, wshift, winvert, wcontrols, \
	wncontrols, wevent, wflags) \
{	.id = snd_soc_dapm_pga, .name = wname, .reg = wreg, .shift = wshift, \
	.invert = winvert, .kcontrols = wcontrols, .num_kcontrols = wncontrols, \
	.event = wevent, .event_flags = wflags}
#define SND_SOC_DAPM_MIXER_E(wname, wreg, wshift, winvert, wcontrols, \
	wncontrols, wevent, wflags) \
{	.id = snd_soc_dapm_mixer, .name = wname, .reg = wreg, .shift = wshift, \
	.invert = winvert, .kcontrols = wcontrols, .num_kcontrols = wncontrols, \
	.event = wevent, .event_flags = wflags}
#define SND_SOC_DAPM_MICBIAS_E(wname, wreg, wshift, winvert, wevent, wflags) \
{	.id = snd_soc_dapm_micbias, .name = wname, .reg = wreg, .shift = wshift, \
	.invert = winvert, .kcontrols = NULL, .num_kcontrols = 0, \
	.event = wevent, .event_flags = wflags}
#define SND_SOC_DAPM_SWITCH_E(wname, wreg, wshift, winvert, wcontrols, \
	wevent, wflags) \
{	.id = snd_soc_dapm_switch, .name = wname, .reg = wreg, .shift = wshift, \
	.invert = winvert, .kcontrols = wcontrols, .num_kcontrols = 1 \
	.event = wevent, .event_flags = wflags}
#define SND_SOC_DAPM_MUX_E(wname, wreg, wshift, winvert, wcontrols, \
	wevent, wflags) \
{	.id = snd_soc_dapm_mux, .name = wname, .reg = wreg, .shift = wshift, \
	.invert = winvert, .kcontrols = wcontrols, .num_kcontrols = 1, \
	.event = wevent, .event_flags = wflags}

/* events that are pre and post DAPM */
#define SND_SOC_DAPM_PRE(wname, wevent) \
{	.id = snd_soc_dapm_pre, .name = wname, .kcontrols = NULL, \
	.num_kcontrols = 0, .event = wevent, \
	.event_flags = SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_PRE_PMD}
#define SND_SOC_DAPM_POST(wname, wevent) \
{	.id = snd_soc_dapm_post, .name = wname, .kcontrols = NULL, \
	.num_kcontrols = 0, .event = wevent, \
	.event_flags = SND_SOC_DAPM_POST_PMU | SND_SOC_DAPM_POST_PMD}

/* stream domain */
#define SND_SOC_DAPM_DAC(wname, stname, wreg, wshift, winvert) \
{	.id = snd_soc_dapm_dac, .name = wname, .sname = stname, .reg = wreg, \
	.shift = wshift, .invert = winvert}
#define SND_SOC_DAPM_ADC(wname, stname, wreg, wshift, winvert) \
{	.id = snd_soc_dapm_adc, .name = wname, .sname = stname, .reg = wreg, \
	.shift = wshift, .invert = winvert}

/*
 * Convenience DAPM kcontrol builders.
 *
 * @SINGLE:       Mono kcontrol.
 * @SINGLE_TLV:   Mono Table Lookup Value kcontrol.
 * @DOUBLE:       Stereo kcontrol.
 * @DOUBLE_TLV:   Stereo Table Lookup Value kcontrol.
 * @ENUM       :  Mono enumerated kcontrol.
 */
#define SOC_DAPM_SINGLE(xname, reg, shift, max, invert) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_dapm_get_volsw, .put = snd_soc_dapm_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }
#define SOC_DAPM_DOUBLE(xname, reg, shift_left, shift_right, max, invert, \
	power) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_dapm_get_volsw, .put = snd_soc_dapm_put_volsw, \
	.private_value = (reg) | ((shift_left) << 8) | ((shift_right) << 12) |\
		 ((max) << 16) | ((invert) << 24) }
#define SOC_DAPM_SINGLE_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_volsw, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.get = snd_soc_dapm_get_volsw, .put = snd_soc_dapm_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }
#define SOC_DAPM_DOUBLE_TLV(xname, reg, shift_left, shift_right, max, invert, \
	power, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = (xname), \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ | SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, \
	.get = snd_soc_dapm_get_volsw, .put = snd_soc_dapm_put_volsw, \
	.private_value = (reg) | ((shift_left) << 8) | ((shift_right) << 12) |\
		 ((max) << 16) | ((invert) << 24) }
#define SOC_DAPM_ENUM(xname, xenum) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.info = snd_soc_info_enum_double, \
	.get = snd_soc_dapm_get_enum_double, \
	.put = snd_soc_dapm_put_enum_double, \
	.private_value = (unsigned long)&xenum }

/*
 * Bias levels
 *
 * @ON:      Bias is fully on for audio playback and capture operations.
 * @PREPARE: Prepare for audio operations. Called before DAPM switching for
 *           stream start and stop operations.
 * @STANDBY: Low power standby state when no playback/capture operations are
 *           in progress. NOTE: The transition time between STANDBY and ON
 *           should be as fast as possible and no longer than 10ms.
 * @OFF:     Power Off. No restrictions on transition times.
 */
enum snd_soc_dapm_bias_level {
	SND_SOC_BIAS_ON,
	SND_SOC_BIAS_PREPARE,
	SND_SOC_BIAS_STANDBY,
	SND_SOC_BIAS_OFF,
};

/*
 * DAPM Stream Events
 *
 * Events sent to the DAPM subsystem to signal stream operations.
 */
enum snd_soc_dapm_stream_event {
	SND_SOC_DAPM_STREAM_NOP,
	SND_SOC_DAPM_STREAM_START,
	SND_SOC_DAPM_STREAM_STOP,
	SND_SOC_DAPM_STREAM_SUSPEND,
	SND_SOC_DAPM_STREAM_RESUME,
	SND_SOC_DAPM_STREAM_PAUSE_PUSH,
	SND_SOC_DAPM_STREAM_PAUSE_RELEASE,
};

/* DAPM Policy
 * 
 * @AUTOMATIC:        Default DAPM policy where all power switching is fully
 *                    automatic for best power savings.
 * @WIDGETS_MANUAL:   Widgets will not be power switched by DAPM core.
 * @BIAS_MANUAL:      Bias will not be power switched by DAPM core.
 * @ALL_MANUAL:       Widgets and Bias will not be power switched by DAPM core.
 * @WIDGETS_ON:       Widgets are always on iff they are part of a valid audio
 *                    signal path. This path does to need to be active.
 * @BIAS_ON:          Bias is always on.
 * @ALL_ON:           Entire audio system is on (PC power state).     
 */
enum snd_soc_dapm_policy {
	SND_SOC_DAPM_POLICY_AUTOMATIC,
	SND_SOC_DAPM_POLICY_WIDGETS_MANUAL,
	SND_SOC_DAPM_POLICY_BIAS_MANUAL,
	SND_SOC_DAPM_POLICY_ALL_MANUAL,
	SND_SOC_DAPM_POLICY_WIDGETS_ON,
	SND_SOC_DAPM_POLICY_BIAS_ON,
	SND_SOC_DAPM_POLICY_ALL_ON,	
};

/*
 * DAPM Widget types
 */
enum snd_soc_dapm_type {
	snd_soc_dapm_input = 0,	/* input pin */
	snd_soc_dapm_output,	/* output pin */
	snd_soc_dapm_mux,	/* selects 1 analog signal from many inputs */
	snd_soc_dapm_mixer,	/* mixes several analog signals together */
	snd_soc_dapm_pga,	/* programmable gain/attenuation (volume) */
	snd_soc_dapm_adc,	/* analog to digital converter */
	snd_soc_dapm_dac,	/* digital to analog converter */
	snd_soc_dapm_micbias,	/* microphone bias (power) */
	snd_soc_dapm_mic,	/* microphone */
	snd_soc_dapm_hp,	/* headphones */
	snd_soc_dapm_spk,	/* speaker */
	snd_soc_dapm_line,	/* line input/output */
	snd_soc_dapm_switch,	/* analog switch */
	snd_soc_dapm_vmid,	/* codec bias/vmid - to minimise pops */
	snd_soc_dapm_pre,	/* machine specific pre widget - exec first */
	snd_soc_dapm_post,	/* machine specific post widget - exec last */
};

struct snd_soc_dapm_widget;
struct snd_soc_dapm_path;
struct snd_soc_dapm_pin;
struct snd_soc_machine;
struct snd_soc_codec;
struct snd_soc_pcm_runtime;

/*
 * DAPM ALSA kcontrol get/set/info for above macros
 */
int snd_soc_dapm_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_dapm_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_dapm_get_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_dapm_put_enum_double(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol);
int snd_soc_dapm_new_control(struct snd_soc_machine *machine,
	struct snd_soc_codec *codec, const struct snd_soc_dapm_widget *widget);

/**
 * snd_soc_dapm_add_route - adds DAPM audio route.
 * @machine: SoC machine
 * @sink_name: sink (audio route destination) name.
 * @control_name: ALSA kcontrol name - or NULL for no kcontrol.
 * @source_name: source (audio route start) name.
 *
 * Adds a DAPM audio route between source and sink. The route connection
 * status is controlled by the ALSA kcontrol. i.e. a MUX or Mixer.
 */
int snd_soc_dapm_add_route(struct snd_soc_machine *machine,
	const char *sink_name, const char *control_name, const char *src_name);

/**
 * snd_soc_dapm_init - Initialise DAPM.
 * @machine: SoC machine
 *
 * Initialises DAPM resources after any new widgets or routes have been added.
 */
int snd_soc_dapm_init(struct snd_soc_machine *machine);

/**
 * snd_soc_dapm_exit - Frees DAPM resources.
 * @machine: SoC machine
 *
 * Frees all DAPM resources.
 */
void snd_soc_dapm_exit(struct snd_soc_machine *machine);

/**
 * snd_soc_dapm_set_policy - Set DAPM policy.
 * @machine: SoC machine
 *
 * Sets the DAPM power switching policy.
 */
int snd_soc_dapm_set_policy(struct snd_soc_machine *machine,
	enum snd_soc_dapm_policy policy);

/**
 * snd_soc_dapm_stream_event - Send DAPM stream event.
 * @machine: SoC machine
 * @stream: stream name
 * @event: event to send
 *
 * Sends a device event to the dapm core. The core then makes any
 * necessary machine or codec power changes.
 */
int snd_soc_dapm_stream_event(struct snd_soc_machine *machine, char *stream,
	enum snd_soc_dapm_stream_event event);

/**
 * snd_soc_dapm_set_bias - Sets the DAPM bias level.
 * @codec: SoC codec
 * @level: bias (power) level.
 *
 * Sets machine and codec to new bias (power) level.
 */
int snd_soc_dapm_set_bias(struct snd_soc_pcm_runtime *pcm_runtime,
	enum snd_soc_dapm_bias_level level);

/**
 * snd_soc_dapm_enable_pin - enable pin.
 * @machine: SoC machine
 * @pin: pin name
 *
 * Enables input/output pin and it's parents or children widgets iff there is
 * a valid audio route and active audio stream.
 * NOTE: snd_soc_dapm_resync() needs to be called after this for DAPM to
 * do any widget power switching.
 */
int snd_soc_dapm_enable_pin(struct snd_soc_machine *machine, char *pin);

/**
 * snd_soc_dapm_disable_pin - disable pin.
 * @machine: SoC machine
 * @pin: pin name
 *
 * Disables input/output pin and it's parents or children widgets.
 * NOTE: snd_soc_dapm_resync() needs to be called after this for DAPM to
 * do any widget power switching.
 */
int snd_soc_dapm_disable_pin(struct snd_soc_machine *machine, char *pin);

/**
 * snd_soc_dapm_sync - disable pin.
 * @machine: SoC machine
 *
 * Resynchronises DAPM widget power state with pin, stream and audio path
 * state changes. This may cause DAPM power switching.
 */
int snd_soc_dapm_sync(struct snd_soc_machine *machine);

/*
 * DAPM audio route.
 *
 * DAPM audio path between two widgets.
 */
struct snd_soc_dapm_path {
	char *name;
	char *long_name;

	/* Source (input) and sink (output) widgets */
	struct snd_soc_dapm_widget *source;
	struct snd_soc_dapm_widget *sink;
	struct snd_kcontrol *kcontrol;

	/* Runtime status */
	u32 connect:1;			/* source and sink are connected */
	u32 walked:1;			/* path has been walked */

	struct list_head list_source;
	struct list_head list_sink;
	struct list_head list;
};

/*
 * DAPM Widget
 *
 * Audio component that can be individually powered down when not used.
 * Called Widget for want of a better adjective atm.
 */
struct snd_soc_dapm_widget {
	enum snd_soc_dapm_type id;
	char *name;			/* widget name */
	char *sname;			/* stream name */
	struct snd_soc_codec *codec;
	struct snd_soc_machine *machine;
	struct list_head list;

	/* Runtime dapm control and status */
	short reg;			/* negative reg = no direct dapm */
	unsigned char shift;		/* bits to shift */
	unsigned int saved_value;	/* widget saved value */
	unsigned int value;		/* widget current value */
	unsigned char power:1;		/* block power status */
	unsigned char invert:1;		/* invert the power bit */
	unsigned char active:1;		/* active stream on DAC, ADC's */
	unsigned char connected:1;	/* connected codec pin */
	unsigned char new:1;		/* cnew complete */
	unsigned char ext:1;		/* has external widgets */
	unsigned char muted:1;		/* muted for pop reduction */
	unsigned char suspend:1;	/* was active before suspend */
	unsigned char pmdown:1;		/* waiting for timeout */

	/* external events */
	unsigned short event_flags;	/* flags to specify event types */
	int (*event)(struct snd_soc_dapm_widget*, int);

	/* ALSA kcontrols that relate to this widget */
	int num_kcontrols;
	const struct snd_kcontrol_new *kcontrols;

	/* Widget input (source) and outputs (sinks) */
	struct list_head sources;
	struct list_head sinks;
};

#endif
