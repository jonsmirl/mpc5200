/*
 * imx32ads-wm8350.c  --  i.MX31ADS Driver for Wolfson WM8350 Codec
 *
 * Copyright 2007 Wolfson Microelectronics PLC.
 *
 * Author: Liam Girdwood
 *         liam.girdwood@wolfsonmicro.com or linux@wolfsonmicro.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    19th Jun 2007   Initial version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/bitops.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/regulator/regulator.h>
#include <linux/regulator/wm8350/audio.h>
#include <linux/regulator/wm8350/bus.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include <asm/arch/dma.h>
#include <asm/arch/spba.h>
#include <asm/arch/clock.h>

#include "../sound/soc/imx/imx-ssi.h"
#include "../sound/soc/imx/imx31-pcm.h"


/* SSI BCLK and LRC master */
#define WM8350_SSI_MASTER	1
/* clock select - either MCLK 0 or 32K 1 */
#define WM8350_USE_FLL_32K	1

#define IMX31ADS_AUDIO_VERSION "0.3"

struct imx31ads_pcm_state {
	int lr_clk_active;
};

struct imx31ads_data {
	struct regulator *analog_supply;
	struct wm8350 *wm8350;
};

struct _wm8350_audio {
	unsigned int channels;
	snd_pcm_format_t format;
	unsigned int rate;
	unsigned int sysclk;
	unsigned int bclkdiv;
	unsigned int clkdiv;
	unsigned int lr_rate;
};

/* in order of power consumption per rate (lowest first) */
static const struct _wm8350_audio wm8350_audio[] = {
	/* 16bit mono modes */
	{1, SNDRV_PCM_FORMAT_S16_LE, 8000, 12288000 >> 1,
		WM8350_BCLK_DIV_48, WM8350_DACDIV_3, 16,},

	/* 16 bit stereo modes */
	{2, SNDRV_PCM_FORMAT_S16_LE, 8000, 12288000,
		WM8350_BCLK_DIV_48, WM8350_DACDIV_6, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 16000, 12288000,
		WM8350_BCLK_DIV_24, WM8350_DACDIV_3, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 32000, 12288000,
		WM8350_BCLK_DIV_12, WM8350_DACDIV_1_5, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 48000, 12288000,
		WM8350_BCLK_DIV_8, WM8350_DACDIV_1, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 96000, 24576000,
		WM8350_BCLK_DIV_8, WM8350_DACDIV_1, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 11025, 11289600,
		WM8350_BCLK_DIV_32, WM8350_DACDIV_4, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 22050, 11289600,
		WM8350_BCLK_DIV_16, WM8350_DACDIV_2, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 44100, 11289600,
		WM8350_BCLK_DIV_8, WM8350_DACDIV_1, 32,},
	{2, SNDRV_PCM_FORMAT_S16_LE, 88200, 22579200,
		WM8350_BCLK_DIV_8, WM8350_DACDIV_1, 32,},

	/* 24bit stereo modes */
	{2, SNDRV_PCM_FORMAT_S24_LE, 48000, 12288000,
		WM8350_BCLK_DIV_4, WM8350_DACDIV_1, 64,},
	{2, SNDRV_PCM_FORMAT_S24_LE, 96000, 24576000,
		WM8350_BCLK_DIV_4, WM8350_DACDIV_1, 64,},
	{2, SNDRV_PCM_FORMAT_S24_LE, 44100, 11289600,
		WM8350_BCLK_DIV_4, WM8350_DACDIV_1, 64,},
	{2, SNDRV_PCM_FORMAT_S24_LE, 88200, 22579200,
		WM8350_BCLK_DIV_4, WM8350_DACDIV_1, 64,},
};

#if WM8350_SSI_MASTER
static int imx32ads_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_codec *codec = pcm_runtime->codec;
	struct wm8350* wm8350 = codec->control_data;
	struct imx31ads_pcm_state *state = pcm_runtime->private_data;

	/* In master mode the LR clock can come from either the DAC or ADC.
	 * We use the LR clock from whatever stream is enabled first.
	 */
	if (!state->lr_clk_active) {
		if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
			wm8350_clear_bits(wm8350, WM8350_CLOCK_CONTROL_2,
				WM8350_LRC_ADC_SEL);
		else
			wm8350_set_bits(wm8350, WM8350_CLOCK_CONTROL_2,
				WM8350_LRC_ADC_SEL);
	}
	state->lr_clk_active++;
	return 0;
}
#else
#define imx32ads_startup NULL
#endif

static int imx32ads_hifi_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *cpu_dai = pcm_runtime->cpu_dai;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	struct imx31ads_pcm_state *state = pcm_runtime->private_data;
	int i, found = 0;
	snd_pcm_format_t format = params_format(params);
	unsigned int rate = params_rate(params);
	unsigned int channels = params_channels(params);
	u32 dai_format;

	/* only need to do this once as capture and playback are sync */
	if (state->lr_clk_active > 1)
		return 0;

	/* find the correct audio parameters */
	for (i = 0; i < ARRAY_SIZE(wm8350_audio); i++) {
		if (rate == wm8350_audio[i].rate &&
			format == wm8350_audio[i].format &&
			channels == wm8350_audio[i].channels) {
			found = 1;
			break;
		}
	}
	if (!found)
		return -EINVAL;

#if WM8350_SSI_MASTER
	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBM_CFM | SND_SOC_DAIFMT_SYNC;
	if (channels == 2)
		dai_format |= SND_SOC_DAIFMT_TDM;

	/* set codec DAI configuration */
	snd_soc_dai_set_fmt(codec_dai, dai_format);

	/* set cpu DAI configuration */
	snd_soc_dai_set_fmt(cpu_dai, dai_format);

#if WM8350_USE_FLL_32K
	/* set 32K clock as the codec system clock for DAC and ADC */
	snd_soc_dai_set_sysclk(codec_dai, WM8350_MCLK_SEL_PLL_32K,
		wm8350_audio[i].sysclk, SND_SOC_CLOCK_IN);
#else
	/* set MCLK as the codec system clock for DAC and ADC */
	snd_soc_dai_set_sysclk(codec_dai, WM8350_MCLK_SEL_PLL_MCLK,
		wm8350_audio[i].sysclk, SND_SOC_CLOCK_IN);
#endif

#else

	dai_format = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
		SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_SYNC;
	if (channels == 2)
		format |= SND_SOC_DAIFMT_TDM;

	/* set codec DAI configuration */
	snd_soc_dai_set_fmt(codec_dai, dai_format);

	/* set cpu DAI configuration */
	snd_soc_dai_set_fmt(cpu_dai, dai_format);

	/* set DAC LRC as the codec system clock for DAC and ADC */
	snd_soc_dai_set_sysclk(codec_dai, WM8350_MCLK_SEL_PLL_DAC,
		wm8350_audio[i].sysclk, SND_SOC_CLOCK_IN);
#endif

	/* set i.MX active slot mask */
	snd_soc_dai_set_tdm_slot(cpu_dai,
		channels == 1 ? 0xfffffffe : 0xfffffffc, channels);

	/* set the SSI system clock as input (unused) */
	snd_soc_dai_set_sysclk(cpu_dai, IMX_SSP_SYS_CLK, 0,
		SND_SOC_CLOCK_IN);

	/* set codec BCLK division for sample rate */
	snd_soc_dai_set_clkdiv(codec_dai, WM8350_BCLK_CLKDIV,
		wm8350_audio[i].bclkdiv);

	/* DAI is synchronous and clocked with DAC LRCLK */
	snd_soc_dai_set_clkdiv(codec_dai,
			WM8350_DACLR_CLKDIV, wm8350_audio[i].lr_rate);

	/* now configure DAC and ADC clocks */
	snd_soc_dai_set_clkdiv(codec_dai,
		WM8350_DAC_CLKDIV, wm8350_audio[i].clkdiv);

	snd_soc_dai_set_clkdiv(codec_dai,
		WM8350_ADC_CLKDIV, wm8350_audio[i].clkdiv);

#if WM8350_SSI_MASTER
#if WM8350_USE_FLL_32K
	/* codec FLL input is 32 kHz from WM8350 */
	snd_soc_dai_set_pll(codec_dai, 0, 32000,
		wm8350_audio[i].sysclk);
#else
	/* codec FLL input is 14.75 MHz from MCLK */
	snd_soc_dai_set_pll(codec_dai, 0, 14750000,
		wm8350_audio[i].sysclk);
#endif
#else
	/* codec FLL input is rate from DAC LRC */
	snd_soc_dai_set_pll(codec_dai, 0, rate,
		wm8350_audio[i].sysclk);
#endif

	return 0;
}

static void imx32ads_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_dai *codec_dai = pcm_runtime->codec_dai;
	struct snd_soc_codec *codec = pcm_runtime->codec;
	struct imx31ads_pcm_state *state = pcm_runtime->private_data;

	/* disable the PLL if there are no active Tx or Rx channels */
	if (!codec->active)
		snd_soc_dai_set_pll(codec_dai, 0, 0, 0);
	state->lr_clk_active--;
}

/*
 * imx32ads WM8350 HiFi DAI opserations.
 */
static struct snd_soc_ops imx32ads_hifi_ops = {
	.startup = imx32ads_startup,
	.shutdown = imx32ads_shutdown,
	.hw_params = imx32ads_hifi_hw_params,
};

/* need to refine these */
static struct wm8350_audio_platform_data imx32ads_wm8350_setup = {
	.vmid_discharge_msecs = 1000,
	.drain_msecs = 30,
	.cap_discharge_msecs = 700,
	.vmid_charge_msecs = 700,
	.vmid_s_curve = WM8350_S_CURVE_SLOW,
	.dis_out4 = WM8350_DISCHARGE_SLOW,
	.dis_out3 = WM8350_DISCHARGE_SLOW,
	.dis_out2 = WM8350_DISCHARGE_SLOW,
	.dis_out1 = WM8350_DISCHARGE_SLOW,
	.vroi_out4 = WM8350_TIE_OFF_500R,
	.vroi_out3 = WM8350_TIE_OFF_500R,
	.vroi_out2 = WM8350_TIE_OFF_500R,
	.vroi_out1 = WM8350_TIE_OFF_500R,
	.vroi_enable = 0,
	.codec_current_on = WM8350_CODEC_ISEL_1_0,
	.codec_current_standby = WM8350_CODEC_ISEL_0_5,
	.codec_current_charge = WM8350_CODEC_ISEL_1_5,
};

/* imx32ads machine dapm widgets */
static const struct snd_soc_dapm_widget imx32ads_dapm_widgets[] = {
	SND_SOC_DAPM_MIC("SiMIC", NULL),
	SND_SOC_DAPM_MIC("Mic1 Jack", NULL),
	SND_SOC_DAPM_MIC("Mic2 Jack", NULL),
	SND_SOC_DAPM_LINE("Line In Jack", NULL),
	SND_SOC_DAPM_LINE("Line Out Jack", NULL),
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
};

/* imx32ads machine audio map */
static const char* audio_map[][3] = {

	/* SiMIC --> IN1LN (with automatic bias) via SP1 */
	{"IN1LN", NULL, "Mic Bias"},
	{"Mic Bias", NULL, "SiMIC"},

	/* Mic 1 Jack --> IN1LN and IN1LP (with automatic bias) */
	{"IN1LN", NULL, "Mic Bias"},
	{"IN1LP", NULL, "Mic1 Jack"},
	{"Mic Bias", NULL, "Mic1 Jack"},

	/* Mic 2 Jack --> IN1RN and IN1RP (with automatic bias) */
	{"IN1RN", NULL, "Mic Bias"},
	{"IN1RP", NULL, "Mic1 Jack"},
	{"Mic Bias", NULL, "Mic1 Jack"},

	/* Line in Jack --> AUX (L+R) */
	{"IN3R", NULL, "Line In Jack"},
	{"IN3L", NULL, "Line In Jack"},

	/* Out1 --> Headphone Jack */
	{"Headphone Jack", NULL, "OUT1R"},
	{"Headphone Jack", NULL, "OUT1L"},

	/* Out1 --> Line Out Jack */
	{"Line Out Jack", NULL, "OUT2R"},
	{"Line Out Jack", NULL, "OUT2L"},

	{NULL, NULL, NULL},
};

#ifdef CONFIG_PM
static int imx32ads_wm8350_audio_suspend(struct platform_device *pdev,
	pm_message_t state)
{
	struct wm8350 *wm8350 = platform_get_drvdata(pdev);
	struct snd_soc_machine *machine = wm8350->audio;

	return snd_soc_suspend_pcms(machine, state);
}

static int imx32ads_wm8350_audio_resume(struct platform_device *pdev)
{
	struct wm8350 *wm8350 = platform_get_drvdata(pdev);
	struct snd_soc_machine *machine = wm8350->audio;

	return snd_soc_resume_pcms(machine);
}

#else
#define imx32ads_wm8350_audio_suspend	NULL
#define imx32ads_wm8350_audio_resume	NULL
#endif

static void imx32ads_jack_handler(struct wm8350 *wm8350, int irq, void *data)
{
	struct snd_soc_machine *machine = (struct snd_soc_machine *)data;
	u16 reg;

	/* debounce for 200ms */
	schedule_timeout_interruptible(msecs_to_jiffies(200));
	reg = wm8350_reg_read(wm8350, WM8350_JACK_PIN_STATUS);

	if (reg & WM8350_JACK_R_LVL) {
		snd_soc_dapm_disable_line(machine, "Line Out Jack");
		snd_soc_dapm_enable_headphone(machine, "Headphone Jack");
	} else {
		snd_soc_dapm_disable_headphone(machine, "Headphone Jack");
		snd_soc_dapm_enable_line(machine, "Line Out Jack");
	}
	snd_soc_dapm_resync(machine);
}

int imx32_audio_init(struct snd_soc_machine *machine)
{
	struct snd_soc_codec *codec;
	struct snd_soc_pcm_runtime *pcm_runtime;
	struct imx31ads_data* audio_data = machine->private_data;
	struct wm8350 *wm8350 = audio_data->wm8350;
	struct imx31ads_pcm_state *state;
	int i;
	u16 reg;

	pcm_runtime = snd_soc_get_pcm(machine, "HiFi");
	if (pcm_runtime == NULL)
		return -ENODEV;

	codec = snd_soc_get_codec(machine, wm8350_codec_id);
	if (codec == NULL)
		return -ENODEV;

	state = kzalloc(sizeof(struct imx31ads_pcm_state), GFP_KERNEL);
	if (state == NULL)
		return -ENOMEM;
	pcm_runtime->private_data = state;

	codec->platform_data = &imx32ads_wm8350_setup;
	snd_soc_codec_set_io(codec, NULL, NULL,	wm8350);
	snd_soc_codec_init(codec, machine);

#if 0
	/* add imx32ads specific controls */
	for (i = 0; i < ARRAY_SIZE(imx32ads_wm8350_audio_controls); i++) {
		if ((err = snd_ctl_add(machine->card,
				snd_soc_cnew(&imx32ads_wm8350_audio_controls[i],
					codec, NULL))) < 0)
			return err;
	}
#endif

	/* Add imx32ads specific widgets */
	for(i = 0; i < ARRAY_SIZE(imx32ads_dapm_widgets); i++) {
		snd_soc_dapm_new_control(machine, codec,
			&imx32ads_dapm_widgets[i]);
	}

	/* set up imx32ads specific audio path audio map */
	for(i = 0; audio_map[i][0] != NULL; i++) {
		snd_soc_dapm_add_route(machine, audio_map[i][0],
			audio_map[i][1], audio_map[i][2]);
	}

	/* disable unused imx32ads WM8350 codec pins */
	snd_soc_dapm_disable_pin(machine, "OUT3");
	snd_soc_dapm_disable_pin(machine, "OUT4");
	snd_soc_dapm_disable_pin(machine, "IN2R");
	snd_soc_dapm_disable_pin(machine, "IN2L");
	snd_soc_dapm_disable_pin(machine, "OUT2L");
	snd_soc_dapm_disable_pin(machine, "OUT2R");

	/* connect and enable all imx32ads WM8350 jacks (for now) */
	snd_soc_dapm_enable_pin(machine, "SiMIC");
	snd_soc_dapm_enable_pin(machine, "Mic1 Jack");
	snd_soc_dapm_enable_pin(machine, "Mic2 Jack");
	snd_soc_dapm_enable_pin(machine, "Line In Jack");
//	snd_soc_dapm_set_policy(machine, SND_SOC_DAPM_POLICY_STREAM);
	snd_soc_dapm_resync(machine);

	/* enable slow clock gen for jack detect */
	reg = wm8350_reg_read(wm8350, WM8350_POWER_MGMT_4);
	wm8350_reg_write(wm8350, WM8350_POWER_MGMT_4,
		reg | WM8350_TOCLK_ENA);

	/* enable jack detect */
	reg = wm8350_reg_read(wm8350, WM8350_JACK_DETECT);
	wm8350_reg_write(wm8350, WM8350_JACK_DETECT,
		reg | WM8350_JDR_ENA);
	wm8350_register_irq(wm8350, WM8350_IRQ_CODEC_JCK_DET_R,
			    imx32ads_jack_handler, machine);
	wm8350_unmask_irq(wm8350, WM8350_IRQ_CODEC_JCK_DET_R);

	return 0;
}

static void imx32_audio_exit(struct snd_soc_machine *machine)
{
	struct snd_soc_pcm_runtime *pcm_runtime;
	struct snd_soc_codec *codec;

	codec = snd_soc_get_codec(machine, wm8350_codec_id);
	if (codec)
		snd_soc_codec_exit(codec, machine);

	pcm_runtime = snd_soc_get_pcm(machine, "HiFi");
	if (pcm_runtime)
		kfree(pcm_runtime->private_data);
}

static struct snd_soc_pcm_config hifi_pcm_config = {
	.name		= "HiFi",
	.codec		= wm8350_codec_id,
	.codec_dai	= wm8350_codec_dai_id,
	.platform	= imx31_platform_id,
	.cpu_dai	= imx_ssi_id1_0,
	.ops		= &imx32ads_hifi_ops,
	.playback	= 1,
	.capture	= 1,
};

static int __devinit imx32ads_wm8350_audio_probe(struct platform_device *pdev)
{
	struct snd_soc_machine *machine;
	struct imx31ads_data *audio_data;
	struct wm8350 *wm8350 = platform_get_drvdata(pdev);
	int ret;

	printk(KERN_INFO "i.MX32ADS WM8350 audio\n");
	audio_data = kzalloc(sizeof(*audio_data), GFP_KERNEL);
	if (audio_data == NULL)
		return -ENOMEM;

	ret = get_ssi_clk(0, &pdev->dev);
	if (ret < 0) {
		printk(KERN_ERR "%s: cant get ssi clock\n", __func__);
		goto ssi_err;
	}
	audio_data->analog_supply = regulator_get(&pdev->dev, "LDO2");
	if (IS_ERR(audio_data->analog_supply)) {
		printk(KERN_ERR "%s: cant get regulator\n", __func__);
		goto ssi_err;
	}
	ret = regulator_set_voltage(audio_data->analog_supply, mV_to_uV(3300));
	if (ret < 0) {
		printk(KERN_ERR "%s: cant set voltage\n", __func__);
		goto reg_err;
	}
	ret = regulator_enable(audio_data->analog_supply);
	if (ret < 0) {
		printk(KERN_ERR "%s: cant enable regulator\n", __func__);
		goto reg_err;
	}

	machine = snd_soc_machine_create("imx31ads", &pdev->dev,
		SNDRV_DEFAULT_IDX1, SNDRV_DEFAULT_STR1);
	if (machine == NULL) {
		ret = -ENOMEM;
		goto reg_err;
	}

	audio_data->wm8350 = wm8350;
	machine->longname = "WM8350";
	machine->init = imx32_audio_init,
	machine->exit = imx32_audio_exit,
	machine->private_data = audio_data;
	machine->dev = &pdev->dev;
	wm8350->audio = machine;

	ret = snd_soc_pcm_create(machine, &hifi_pcm_config);
	if (ret < 0)
		goto err;

	/* WM8350 uses SSI1 via AUDMUX port 5 for audio */

	/* reset port 1 & 5 */
	DAM_PTCR1 = 0;
	DAM_PDCR1 = 0;
	DAM_PTCR5 = 0;
	DAM_PDCR5 = 0;

	/* set to synchronous */
	DAM_PTCR1 |= AUDMUX_PTCR_SYN;
	DAM_PTCR5 |= AUDMUX_PTCR_SYN;

#if WM8350_SSI_MASTER
	/* set Rx sources 1 <--> 5 */
	DAM_PDCR1 |= AUDMUX_PDCR_RXDSEL(5);
	DAM_PDCR5 |= AUDMUX_PDCR_RXDSEL(1);

	/* set Tx frame direction and source  5 --> 1 output */
	DAM_PTCR1 |= AUDMUX_PTCR_TFSDIR;
	DAM_PTCR1 |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, 5);

	/* set Tx Clock direction and source 5--> 1 output */
	DAM_PTCR1 |= AUDMUX_PTCR_TCLKDIR;
	DAM_PTCR1 |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, 5);
#else
	/* set Rx sources 1 <--> 5 */
	DAM_PDCR1 |= AUDMUX_PDCR_RXDSEL(5);
	DAM_PDCR5 |= AUDMUX_PDCR_RXDSEL(1);

	/* set Tx frame direction and source  1 --> 5 output */
	DAM_PTCR5 |= AUDMUX_PTCR_TFSDIR;
	DAM_PTCR5 |= AUDMUX_PTCR_TFSSEL(AUDMUX_FROM_TXFS, 1);

	/* set Tx Clock direction and source 1--> 5 output */
	DAM_PTCR5 |= AUDMUX_PTCR_TCLKDIR;
	DAM_PTCR5 |= AUDMUX_PTCR_TCSEL(AUDMUX_FROM_TXFS, 1);
#endif
	ret = snd_soc_machine_register(machine);
	return ret;

err:
	snd_soc_machine_free(machine);
reg_err:
	regulator_put(audio_data->analog_supply);
ssi_err:
	kfree(audio_data);
	put_ssi_clk(0);
	return ret;
}

static int __devexit imx32ads_wm8350_audio_remove(struct platform_device *pdev)
{
	struct wm8350 *wm8350 = platform_get_drvdata(pdev);
	struct snd_soc_machine *machine = wm8350->audio;
	struct imx31ads_data *audio_data = machine->private_data;

	wm8350_mask_irq(wm8350, WM8350_IRQ_CODEC_JCK_DET_R);
	wm8350_free_irq(wm8350, WM8350_IRQ_CODEC_JCK_DET_R);
	snd_soc_machine_free(machine);

	regulator_disable(audio_data->analog_supply);
	regulator_put(audio_data->analog_supply);
	kfree(audio_data);
	put_ssi_clk(0);
	return 0;
}

static struct platform_driver imx32ads_wm8350_audio_driver = {
	.probe		= imx32ads_wm8350_audio_probe,
	.remove		= __devexit_p(imx32ads_wm8350_audio_remove),
	.suspend	= imx32ads_wm8350_audio_suspend,
	.resume		= imx32ads_wm8350_audio_resume,
	.driver		= {
		.name	= "imx31ads-audio",
	},
};

static int __init imx32ads_wm8350_audio_init(void)
{
	return platform_driver_register(&imx32ads_wm8350_audio_driver);
}

static void __exit imx32ads_wm8350_audio_exit(void)
{
	platform_driver_unregister(&imx32ads_wm8350_audio_driver);
}

module_init(imx32ads_wm8350_audio_init);
module_exit(imx32ads_wm8350_audio_exit);

MODULE_AUTHOR("Liam Girdwood");
MODULE_DESCRIPTION("PMIC WM8350 Driver for i.MX32ADS");
MODULE_LICENSE("GPL");
