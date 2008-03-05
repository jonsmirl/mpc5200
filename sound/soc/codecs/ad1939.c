/*
 * AD1939 I2S Codec driver
 *
 * (c) 2007 MSC Vertriebsges.m.b.H, <mlau@msc-ge.com>
 *
 * licensed under the GPLv2
 */

/*
 * Analog Devices AD1939 I2S codec; with I2C or SPI interface.
 *  supports 2 channels in plain I2S mode,
 *  and up to 16 channels in TDM modes.
 * As Master, the codec always assumes a frame is a multiple of 32bits!
 * TDM Modes:
 *	- multiple channels over I2S
 *	- 32 fixed slot width, TDM stereo is I2S with 32bits/sample.
 *
 * TODO: driver switches between stereo and TDM mode based on the number
 *	 of  channels  for record / playback (ad1939_hw_params()).  This
 *	 should be  overhauled if someone  wants to daisy-chain a few of
 *	 these  together  (to  get 8 DAC channels total).  Maybe use TDM
 *	 mode  exclusively  when codec  is LRCK/BCK  master, and let the
 *	 user set WHICH TDM mode to use through setup_data.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <sound/driver.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc-codec.h>
#include <sound/soc-dai.h>
#include <sound/soc-card.h>
#include <sound/soc-dapm.h>
#include <sound/initval.h>

#include "ad1939.h"

#define AUDIO_NAME "AD1939"
#define AD1939_VERSION	"0.2"

#define msg(x...) printk(KERN_INFO AUDIO_NAME ": " x)

/* #define CODEC_DEBUG */

#ifdef CODEC_DEBUG
#define dbg(x...)	printk(KERN_INFO AUDIO_NAME ": " x)
#else
#define dbg(x...)	do {} while(0)
#endif

struct ad1939_data {
	struct snd_soc_codec *codec;
	struct snd_soc_dai *dai;
	unsigned long sysclk;
	unsigned char tdm_mode;
};

/* default register contents after reset */
static const u16 ad1939_regcache[AD1939_REGCOUNT] __devinitdata = {
	0, 0, 0, 0, 0, 0, 255, 255, 255, 255, 255, 255, 255, 255, 0, 0, 0
};

static inline unsigned int ad1939_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 *cache = codec->reg_cache;
	if (reg >= AD1939_REGCOUNT)
		return -1;
	return cache[reg];
}

static inline unsigned int ad1939_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	u8 data;
	/* the PLLCTL1 has one read-only bit: PLL lock indicator.
	 * all other regs keep what was set
	 */
	if (likely(reg != AD1939_PLLCTL1))
		return ad1939_read_reg_cache(codec, reg);

	data = reg & 0xff;
	if (codec->soc_phys_read(codec->control_data, (long)&data, 1) != 1)
		return -EIO;

	return data;
};

/*
 * write ad1939 register cache
 */
static inline void ad1939_write_reg_cache(struct snd_soc_codec *codec,
	u16 reg, unsigned int value)
{
	u8 *cache = codec->reg_cache;
	if (reg >= AD1939_REGCOUNT)
		return;
	cache[reg] = value;
}

/*
 * write to the AD1939 register space
 */
static int ad1939_write(struct snd_soc_codec *codec, unsigned int reg,
	unsigned int value)
{
	u8 data[2];

	data[0] = reg & 0xff;
	data[1] = value & 0xff;

	ad1939_write_reg_cache (codec, reg, value);
	if (codec->soc_phys_write(codec->control_data, (long)data, 2) == 2)
		return 0;
	else
		return -EIO;
}

/***** controls ******/

static const char *dac_deemph[] = {"Flat", "48kHz", "44.1kHz", "32kHz"};
static const char *dac_outpol[] = {"Normal", "Inverted"};

static const struct soc_enum ad1939_enum[] = {
      /*SOC_ENUM_SINGLE(register, startbit, choices, choices-texts) */
	SOC_ENUM_SINGLE(AD1939_DACCTL2, 1, 4, dac_deemph),
	SOC_ENUM_SINGLE(AD1939_DACCTL2, 5, 1, dac_outpol),
};

static const struct snd_kcontrol_new ad1939_snd_ctls[] = {
SOC_DOUBLE_R("Master Playback", AD1939_VOL1L, AD1939_VOL1R, 0, 255, 1),
SOC_DOUBLE_R("Channel 2 Playback", AD1939_VOL2L, AD1939_VOL2R, 0, 255, 1),
SOC_DOUBLE_R("Channel 3 Playback", AD1939_VOL3L, AD1939_VOL3R, 0, 255, 1),
SOC_DOUBLE_R("Channel 4 Playback", AD1939_VOL4L, AD1939_VOL4R, 0, 255, 1),
SOC_ENUM("DAC Deemphasis", ad1939_enum[0]),
SOC_ENUM("DAC output polarity", ad1939_enum[1]),
};

/* add non dapm controls */
static int ad1939_add_controls(struct snd_soc_codec *codec, 
	struct snd_card *card)
{
	int err, i;

	for (i = 0; i < ARRAY_SIZE(ad1939_snd_ctls); i++) {
		err = snd_ctl_add(card,
			snd_soc_cnew(&ad1939_snd_ctls[i], codec, NULL));
		if (err < 0)
			return err;
	}
	return 0;
}


/***** chip interface config ******/


static int ad1939_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *pcm_runtime = substream->private_data;
	struct snd_soc_codec *codec = pcm_runtime->codec;
	struct ad1939_data *ad = codec->private_data;
	unsigned char dac0, dac1, dac2, adc0, adc1, adc2;
	unsigned long rate;
	unsigned int bits;

	dbg("ad1939_hw_params");

	dac0 = ad1939_read(codec, AD1939_DACCTL0);
	dac1 = ad1939_read(codec, AD1939_DACCTL1);
	dac2 = ad1939_read(codec, AD1939_DACCTL2);
	adc0 = ad1939_read(codec, AD1939_ADCCTL0);
	adc1 = ad1939_read(codec, AD1939_ADCCTL1);
	adc2 = ad1939_read(codec, AD1939_ADCCTL2);

	rate = params_rate(params);
	bits = params->msbits;

	dbg("bits %d srate %lu/%d chans %d\n", bits, rate,
	     params->rate_den, params_channels(params));

	/*
	 * sample rate
	 */
	dac0 &= ~(3<<1);	/* 48kHz */
	adc0 &= ~(3<<6);	/* 48kHz */
	switch (rate) {
	case 32000 ... 48000:
		break;
	case 64000 ... 96000:
		dac0 |= (1<<1);
		adc0 |= (1<<6);
		break;
	case 128000 ... 192000:
		dac0 |= (2<<1);
		adc0 |= (2<<6);
		break;
	default:
		dbg("rejecting srate %lu\n", rate);
		return -EINVAL;
	}

	/*
	 * sample width (bits)
	 */
	dac2 &= ~(3<<3);	/* 24 bits */
	adc1 &= ~(3<<0);	/* 24 bits */
	switch (bits) {
	case 16: dac2 |= (3<<3); adc1 |= (3<<0); break;
	case 20: dac2 |= (1<<3); adc1 |= (1<<0); break;
	case 24: break;
	default:
		dbg("rejecting bits %d\n", bits);
		return -EINVAL;
	}

	/*
	 * channels
	 */
	dac0 &= ~(3<<6);	/* DAC I2S stereo */
	dac1 &= ~(3<<1);	/* 2 channels */
	adc1 &= ~(3<<5);	/* ADC I2S stereo */
	adc2 &= ~(3<<4);	/* 2 channels */
	switch (params_channels(params)) {
	case 2:	/* I2S stereo mode */
		break;
	case 4:	/* TDM mode */
		dac0 |= (ad->tdm_mode & 3) << 6;
		dac1 |= (1<<1);
		adc1 |= (ad->tdm_mode & 3) << 5;
		adc2 |= (1<<4);
		break;
	case 8:	/* TDM mode */
		dac0 |= (ad->tdm_mode & 3) << 6;
		dac1 |= (2<<1);
		adc1 |= (ad->tdm_mode & 3) << 5;
		adc2 |= (2<<4);
		break;
	case 16: /* TDM mode */
		dac0 |= (ad->tdm_mode & 3) << 6;
		dac1 |= (3<<1);
		adc1 |= (ad->tdm_mode & 3) << 5;
		adc2 |= (3<<4);
		break;
	default:
		dbg("%d channels not supported\n",
			params_channels(params));
		return -EINVAL;
	}

	ad1939_write(codec, AD1939_DACCTL0, dac0);
	ad1939_write(codec, AD1939_DACCTL1, dac1);
	ad1939_write(codec, AD1939_DACCTL2, dac2);
	ad1939_write(codec, AD1939_ADCCTL0, adc0);
	ad1939_write(codec, AD1939_ADCCTL1, adc1);
	ad1939_write(codec, AD1939_ADCCTL2, adc2);

	return 0;
}

static int ad1939_set_dai_fmt(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	unsigned char dac0, dac1, adc1, adc2;

	dbg("ad1939_set_dai_fmt(0x%08lx)", fmt);

	dac0 = ad1939_read(codec, AD1939_DACCTL0);
	dac1 = ad1939_read(codec, AD1939_DACCTL1);
	adc1 = ad1939_read(codec, AD1939_ADCCTL1);
	adc2 = ad1939_read(codec, AD1939_ADCCTL2);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:
		dac1 |= (1<<4) | (1<<5); /* LRCK master, BCK master */
		adc2 |= (1<<3) | (1<<6); /* LRCK master, BCK master */
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		dac1 |= (1<<4);		/* LRCK master */
		dac1 &= ~(1<<5);	/* BCK slave */
		adc2 |= (1<<3);		/* LRCK master */
		adc2 &= ~(1<<6);	/* BCK slave */
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		dac1 &= ~(1<<4);	/* LRCK slave */
		dac1 |= (1<<5);		/* BCK master */
		adc2 &= ~(1<<3);	/* LRCK slave */
		adc2 |= (1<<6);		/* BCK master */
		break;
	case SND_SOC_DAIFMT_CBS_CFS:
		dac1 &= ~((1<<4) | (1<<5)); /* LRCK BCK slave */
		adc2 &= ~((1<<3) | (1<<6)); /* LRCK BCK slave */
		break;
	default:
		dbg("invalid master/slave configuration\n");
		return -EINVAL;
	}

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		dac0 &= ~(7<<3); /* DAC: SDATA delay 1 */
		adc1 &= ~(7<<2); /* ADC: SDATA delay 1 */
		break;
	case SND_SOC_DAIFMT_MSB: /* LEFT_J */
		dac0 |= (1<<3);	/* no SDATA delay */
		adc1 |= (1<<2); /* no SDATA delay */
		break;
#if 0
	case SND_SOC_DAIFMT_LSB:
		/* FIXME: need to know if in TDM/Master mode and sample
		 * size, then program bitdelay accordingly
		 */
		break;
#endif
	default:
		dbg("invalid I2S interface format\n");
		return -EINVAL;
	}

	/* clock inversion */
	dac1 &= ~((1<<7) | (1<<3)); /* norm BCK LRCK */
	adc2 &= ~((1<<1) | (1<<2)); /* norm BCK LRCK */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		break;
	case SND_SOC_DAIFMT_NB_IF:
		dac1 |= (1<<3);		/* inv LRCK */
		adc2 |= (1<<2);
		break;
	case SND_SOC_DAIFMT_IB_NF:
		dac1 |= (1<<7);		/* inv BCK */
		adc2 |= (1<<1);
		break;
	case SND_SOC_DAIFMT_IB_IF:
		dac1 |= (1<<3) | (1<<7); /* inv LRCK BCK */
		adc2 |= (1<<1) | (1<<2);
		break;
	default:
		dbg("invalid clock inversion configuration\n");
		return -EINVAL;
	}

	ad1939_write(codec, AD1939_DACCTL0, dac0);
	ad1939_write(codec, AD1939_DACCTL1, dac1);
	ad1939_write(codec, AD1939_ADCCTL1, adc1);
	ad1939_write(codec, AD1939_ADCCTL2, adc2);

	return 0;
}

static int ad1939_set_bias_level(struct snd_soc_codec *codec, 
	enum snd_soc_dapm_bias_level level)
{
	unsigned char pll0, adc0, dac0;

	/* the codec doesn't really have  sophisticated PM like the
	 * AD1939 for example; one can merely turn off DAC, ADC and
	 * the internal PLL
	 */
	pll0 = ad1939_read(codec, AD1939_PLLCTL0) & 0xfe;
	dac0 = ad1939_read(codec, AD1939_DACCTL0) & 0xfe;
	adc0 = ad1939_read(codec, AD1939_ADCCTL0) & 0xfe;

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
		ad1939_write(codec, AD1939_PLLCTL0, pll0);
		ad1939_write(codec, AD1939_DACCTL0, dac0);
		ad1939_write(codec, AD1939_ADCCTL0, adc0);
		break;
	case SND_SOC_BIAS_STANDBY:
	case SND_SOC_BIAS_OFF:
		/* turn of internal PLL and DAC/ADCs */
		ad1939_write(codec, AD1939_PLLCTL0, pll0 | 1);
		ad1939_write(codec, AD1939_DACCTL0, dac0 | 1);
		ad1939_write(codec, AD1939_ADCCTL0, adc0 | 1);
		break;
	}
	codec->bias_level = level;
	return 0;
}

static int ad1939_digmute(struct snd_soc_dai *dai, int mute)
{
	ad1939_write(dai->codec, AD1939_DACMUTE, mute ? 0xff : 0);
	return 0;
}

static int ad1939_set_dai_sysclk(struct snd_soc_dai *codec_dai,
		int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct ad1939_data *ad = codec->private_data;

	dbg("sysck id %d f %d dir %d\n", clk_id, freq, dir);
	switch (freq) {
	case 11288000:
		ad->sysclk = freq;
		break;
	default:
		dbg("invalid sysclk\n");
		return -EINVAL;
	}
	return 0;
}

/*
 * initialise the AD1939 codec
 */
static int ad1939_codec_init(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card)
{
	struct ad1939_setup_data *setup = codec->platform_data;
	struct ad1939_data *ad = codec->private_data;
	unsigned char r0, r1;

	/* "initialize" the codec with default data */
	for (r0 = 0; r0 < AD1939_REGCOUNT; r0++)
		ad1939_write(codec, r0, ad1939_regcache[r0]);

	/*
	 * remember TDM mode and setup clock routing
	 */
	ad->tdm_mode = setup->tdm_mode;
	/* use default TDM mode if noone wants one */
	if ((ad->tdm_mode > AD1939_TDM_MODE_DUALLINE) || (ad->tdm_mode < 1))
		ad->tdm_mode = AD1939_TDM_MODE_TDM;

	r0 = ad1939_read(codec, AD1939_PLLCTL0) & ~(3<<5);
	r1 = ad1939_read(codec, AD1939_PLLCTL1) & 3;

	r0 |= (setup->pll_src & 3) << 5;
	r0 |= (1<<7);	/* enable internal master clock (i.e. the DAC/ADCs) */
	r1 |= setup->dac_adc_clksrc & 3;
	ad1939_write(codec, AD1939_PLLCTL0, r0);
	ad1939_write(codec, AD1939_PLLCTL1, r1);

	/* Bitclock sources for the ADC and DAC I2S interfaces */
	r0 = ad1939_read(codec, AD1939_DACCTL1);
	r1 = ad1939_read(codec, AD1939_ADCCTL2);
	r0 &= ~AD1939_BCLKSRC_DAC_PLL;
	r1 &= ~AD1939_BCLKSRC_ADC_PLL;
	r0 |= setup->dac_adc_clksrc & AD1939_BCLKSRC_DAC_PLL;
	r1 |= setup->dac_adc_clksrc & AD1939_BCLKSRC_ADC_PLL;
	ad1939_write(codec, AD1939_DACCTL1, r0);
	ad1939_write(codec, AD1939_ADCCTL2, r1);

	ad1939_add_controls(codec, soc_card->card);
	ad1939_set_bias_level(codec, SND_SOC_BIAS_ON);

	return 0;
}

static void ad1939_codec_exit(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card)
{
	ad1939_set_bias_level(codec, SND_SOC_BIAS_OFF);
}


#define AD1939_RATES	\
	(SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000 | \
	 SNDRV_PCM_RATE_192000)

#define AD1939_FORMATS	\
	(SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE)

static struct snd_soc_dai_caps ad1939_playback = {
	.stream_name	= "Playback",
	.channels_min	= 2,
	.channels_max	= 8,
	.rates		= AD1939_RATES,
	.formats	= AD1939_FORMATS,
};

static struct snd_soc_dai_caps ad1939_capture = {
	.stream_name	= "Capture",
	.channels_min	= 2,
	.channels_max	= 4,
	.rates		= AD1939_RATES,
	.formats	= AD1939_FORMATS,
};

static struct snd_soc_dai_ops ad1939_dai_ops = {
	/* alsa ops */
	.hw_params	= ad1939_hw_params,
	
	/* dai ops */
	.digital_mute	= ad1939_digmute,
	.set_sysclk	= ad1939_set_dai_sysclk,
	.set_fmt	= ad1939_set_dai_fmt,
};

/* for modprobe */
const char ad1939_codec_id[] = "ad1939-codec";
EXPORT_SYMBOL_GPL(ad1939_codec_id);

const char ad1939_codec_dai_id[] = "ad1939-codec-dai";
EXPORT_SYMBOL_GPL(ad1939_codec_dai_id);

static int ad1939_dai_probe(struct ad1939_data *ad1939, struct device *dev)
{
	struct snd_soc_dai *dai;
	int ret;

	dai = snd_soc_dai_allocate();
	if (dai == NULL)
		return -ENOMEM;

	dai->name = ad1939_codec_dai_id;
	dai->ops = &ad1939_dai_ops;
	dai->playback = &ad1939_playback;
	dai->capture = &ad1939_capture;
	dai->dev = dev;
	ret = snd_soc_register_codec_dai(dai);
	if (ret < 0) {
		snd_soc_dai_free(dai);
		return ret;
	}
	ad1939->dai = dai;
	return 0;
}

static int ad1939_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_codec *codec;
	struct ad1939_data *ad1939;
	int ret;

	printk(KERN_INFO "AD1939 Audio Codec %s", AD1939_VERSION);

	codec = snd_soc_codec_allocate();
	if (codec == NULL)
		return -ENOMEM;

	ad1939 = kzalloc(sizeof(struct ad1939_data), GFP_KERNEL);
	if (ad1939 == NULL) {
		ret = -ENOMEM;
		goto ad1939_err;
	}

	codec->dev = &pdev->dev;
	codec->name = ad1939_codec_id;
	codec->set_bias_level = ad1939_set_bias_level;
	codec->codec_read = ad1939_read_reg_cache;
	codec->codec_write = ad1939_write;
	codec->init = ad1939_codec_init;
	codec->exit = ad1939_codec_exit;
	codec->reg_cache_size = AD1939_REGCOUNT;
	codec->reg_cache_step = 1;
	codec->private_data = ad1939;
	platform_set_drvdata(pdev, codec);
		
	ret = snd_soc_register_codec(codec);
	if (ret < 0)
		goto codec_err;
	ret = ad1939_dai_probe(ad1939, &pdev->dev);
	if (ret < 0)
		goto dai_err;
	return ret;

dai_err:
	snd_soc_register_codec(codec);
codec_err:
	kfree(ad1939);
ad1939_err:
	snd_soc_codec_free(codec);
	return ret;
}

static int ad1939_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_codec *codec = platform_get_drvdata(pdev);
	struct ad1939_data *ad1939 = codec->private_data;
	
	snd_soc_unregister_codec_dai(ad1939->dai);
	snd_soc_dai_free(ad1939->dai);
	kfree(ad1939);
	snd_soc_unregister_codec(codec);
	snd_soc_codec_free(codec);
	return 0;
}

static struct platform_driver ad1939_codec_driver = {
	.driver = {
		.name		= ad1939_codec_id,
		.owner		= THIS_MODULE,
	},
	.probe		= ad1939_codec_probe,
	.remove		= __devexit_p(ad1939_codec_remove),
};

static __init int ad1939_init(void)
{
	return platform_driver_register(&ad1939_codec_driver);
}

static __exit void ad1939_exit(void)
{
	platform_driver_unregister(&ad1939_codec_driver);
}

module_init(ad1939_init);
module_exit(ad1939_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ASoC AD1939 I2S Codec driver");
MODULE_AUTHOR("Manuel Lauss <mlau@msc-ge.com>");
