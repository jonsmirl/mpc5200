/*
 * CS4270 ALSA SoC (ASoC) codec driver
 *
 * Author: Timur Tabi <timur@freescale.com>
 *
 * Copyright 2007-2008 Freescale Semiconductor, Inc.
 *
 * This file is licensed under the terms of the GNU General Public License
 * version 2.  This program is licensed "as is" without any warranty of any
 * kind, whether express or implied.
 *
 * This is an ASoC device driver for the Cirrus Logic CS4270 codec.
 *
 * Current features/limitations:
 *
 * 1) Software mode is supported. Stand-alone mode is not supported.
 * 2) Only I2C is supported, not SPI
 * 3) Only Master mode is supported, not Slave.
 * 4) The soc_card driver's 'startup' function must call
 *    cs4270_set_dai_sysclk() with the value of MCLK.
 * 5) Only I2S and left-justified modes are supported
 * 6) Power management is not supported
 * 7) The only supported control is volume and hardware mute (if enabled)
 */

#include <linux/module.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc-codec.h>
#include <sound/soc-dai.h>
#include <sound/soc-card.h>
#include <sound/initval.h>
#include <linux/i2c.h>

/*
 * The codec isn't really big-endian or little-endian, since the I2S
 * interface requires data to be sent serially with the MSbit first.
 * However, to support BE and LE I2S devices, we specify both here.  That
 * way, ALSA will always match the bit patterns.
 */
#define CS4270_FORMATS (SNDRV_PCM_FMTBIT_S8      | \
			SNDRV_PCM_FMTBIT_S16_LE  | SNDRV_PCM_FMTBIT_S16_BE  | \
			SNDRV_PCM_FMTBIT_S18_3LE | SNDRV_PCM_FMTBIT_S18_3BE | \
			SNDRV_PCM_FMTBIT_S20_3LE | SNDRV_PCM_FMTBIT_S20_3BE | \
			SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S24_3BE | \
			SNDRV_PCM_FMTBIT_S24_LE  | SNDRV_PCM_FMTBIT_S24_BE)

/* CS4270 registers addresses */
#define CS4270_CHIPID	0x01	/* Chip ID */
#define CS4270_PWRCTL	0x02	/* Power Control */
#define CS4270_MODE	0x03	/* Mode Control */
#define CS4270_FORMAT	0x04	/* Serial Format, ADC/DAC Control */
#define CS4270_TRANS	0x05	/* Transition Control */
#define CS4270_MUTE	0x06	/* Mute Control */
#define CS4270_VOLA	0x07	/* DAC Channel A Volume Control */
#define CS4270_VOLB	0x08	/* DAC Channel B Volume Control */

#define CS4270_FIRSTREG	0x01
#define CS4270_LASTREG	0x08
#define CS4270_NUMREGS	(CS4270_LASTREG - CS4270_FIRSTREG + 1)

/* Bit masks for the CS4270 registers */
#define CS4270_CHIPID_ID	0xF0
#define CS4270_CHIPID_REV	0x0F
#define CS4270_PWRCTL_FREEZE	0x80
#define CS4270_PWRCTL_PDN_ADC	0x20
#define CS4270_PWRCTL_PDN_DAC	0x02
#define CS4270_PWRCTL_PDN	0x01
#define CS4270_MODE_SPEED_MASK	0x30
#define CS4270_MODE_1X		0x00
#define CS4270_MODE_2X		0x10
#define CS4270_MODE_4X		0x20
#define CS4270_MODE_SLAVE	0x30
#define CS4270_MODE_DIV_MASK	0x0E
#define CS4270_MODE_DIV1	0x00
#define CS4270_MODE_DIV15	0x02
#define CS4270_MODE_DIV2	0x04
#define CS4270_MODE_DIV3	0x06
#define CS4270_MODE_DIV4	0x08
#define CS4270_MODE_POPGUARD	0x01
#define CS4270_FORMAT_FREEZE_A	0x80
#define CS4270_FORMAT_FREEZE_B	0x40
#define CS4270_FORMAT_LOOPBACK	0x20
#define CS4270_FORMAT_DAC_MASK	0x18
#define CS4270_FORMAT_DAC_LJ	0x00
#define CS4270_FORMAT_DAC_I2S	0x08
#define CS4270_FORMAT_DAC_RJ16	0x18
#define CS4270_FORMAT_DAC_RJ24	0x10
#define CS4270_FORMAT_ADC_MASK	0x01
#define CS4270_FORMAT_ADC_LJ	0x00
#define CS4270_FORMAT_ADC_I2S	0x01
#define CS4270_TRANS_ONE_VOL	0x80
#define CS4270_TRANS_SOFT	0x40
#define CS4270_TRANS_ZERO	0x20
#define CS4270_TRANS_INV_ADC_A	0x08
#define CS4270_TRANS_INV_ADC_B	0x10
#define CS4270_TRANS_INV_DAC_A	0x02
#define CS4270_TRANS_INV_DAC_B	0x04
#define CS4270_TRANS_DEEMPH	0x01
#define CS4270_MUTE_AUTO	0x20
#define CS4270_MUTE_ADC_A	0x08
#define CS4270_MUTE_ADC_B	0x10
#define CS4270_MUTE_POLARITY	0x04
#define CS4270_MUTE_DAC_A	0x01
#define CS4270_MUTE_DAC_B	0x02

/* Private data for the CS4270 */
struct cs4270_private {
	struct snd_soc_codec codec;
	char name[16];
	unsigned int mclk; /* Input frequency of the MCLK pin */
	unsigned int mode; /* The mode (I2S or left-justified) */
	struct snd_soc_dai_ops dai_ops;
	struct snd_soc_dai *dai;
	struct snd_soc_dai_new dai_new;
	struct snd_soc_dai_caps playback;
	struct snd_soc_dai_caps capture;
	u8 reg_cache[CS4270_NUMREGS];
};

/*
 * Clock Ratio Selection for Master Mode with I2C enabled
 *
 * The data for this chart is taken from Table 5 of the CS4270 reference
 * manual.
 *
 * This table is used to determine how to program the Mode Control register.
 * It is also used by cs4270_set_dai_sysclk() to tell ALSA which sampling
 * rates the CS4270 currently supports.
 *
 * Each element in this array corresponds to the ratios in mclk_ratios[].
 * These two arrays need to be in sync.
 *
 * 'speed_mode' is the corresponding bit pattern to be written to the
 * MODE bits of the Mode Control Register
 *
 * 'mclk' is the corresponding bit pattern to be wirten to the MCLK bits of
 * the Mode Control Register.
 *
 * In situations where a single ratio is represented by multiple speed
 * modes, we favor the slowest speed.  E.g, for a ratio of 128, we pick
 * double-speed instead of quad-speed.  However, the CS4270 errata states
 * that Divide-By-1.5 can cause failures, so we avoid that mode where
 * possible.
 *
 * ERRATA: There is an errata for the CS4270 where divide-by-1.5 does not
 * work if VD = 3.3V.  If this effects you, select the
 * CONFIG_SND_SOC_CS4270_VD33_ERRATA Kconfig option, and the driver will
 * never select any sample rates that require divide-by-1.5.
 */
static struct {
	unsigned int ratio;
	u8 speed_mode;
	u8 mclk;
} cs4270_mode_ratios[] = {
	{64, CS4270_MODE_4X, CS4270_MODE_DIV1},
#ifndef CONFIG_SND_SOC_CS4270_VD33_ERRATA
	{96, CS4270_MODE_4X, CS4270_MODE_DIV15},
#endif
	{128, CS4270_MODE_2X, CS4270_MODE_DIV1},
	{192, CS4270_MODE_4X, CS4270_MODE_DIV3},
	{256, CS4270_MODE_1X, CS4270_MODE_DIV1},
	{384, CS4270_MODE_2X, CS4270_MODE_DIV3},
	{512, CS4270_MODE_1X, CS4270_MODE_DIV2},
	{768, CS4270_MODE_1X, CS4270_MODE_DIV3},
	{1024, CS4270_MODE_1X, CS4270_MODE_DIV4}
};

/* The number of MCLK/LRCK ratios supported by the CS4270 */
#define NUM_MCLK_RATIOS		ARRAY_SIZE(cs4270_mode_ratios)

/*
 * Determine the CS4270 samples rates.
 *
 * 'freq' is the input frequency to MCLK.  The other parameters are ignored.
 *
 * The value of MCLK is used to determine which sample rates are supported
 * by the CS4270.  The ratio of MCLK / Fs must be equal to one of nine
 * support values: 64, 96, 128, 192, 256, 384, 512, 768, and 1024.
 *
 * This function calculates the nine ratios and determines which ones match
 * a standard sample rate.  If there's a match, then it is added to the list
 * of support sample rates.
 *
 * This function must be called by the soc_card driver's 'startup' function,
 * otherwise the list of supported sample rates will not be available in
 * time for ALSA.
 *
 * Note that in stand-alone mode, the sample rate is determined by input
 * pins M0, M1, MDIV1, and MDIV2.  Also in stand-alone mode, divide-by-3
 * is not a programmable option.  However, divide-by-3 is not an available
 * option in stand-alone mode.  This cases two problems: a ratio of 768 is
 * not available (it requires divide-by-3) and B) ratios 192 and 384 can
 * only be selected with divide-by-1.5, but there is an errate that make
 * this selection difficult.
 *
 * In addition, there is no mechanism for communicating with the soc_card
 * driver what the input settings can be.  This would need to be implemented
 * for stand-alone mode to work.
 */
static int cs4270_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs4270_private *cs4270 =
		container_of(codec, struct cs4270_private, codec);
	unsigned int rates = 0;
	unsigned int rate_min = -1;
	unsigned int rate_max = 0;
	unsigned int i;

	cs4270->mclk = freq;

	for (i = 0; i < NUM_MCLK_RATIOS; i++) {
		unsigned int rate = freq / cs4270_mode_ratios[i].ratio;
		rates |= snd_pcm_rate_to_rate_bit(rate);
		if (rate < rate_min)
			rate_min = rate;
		if (rate > rate_max)
			rate_max = rate;
	}
	/* FIXME: soc should support a rate list */
	rates &= ~SNDRV_PCM_RATE_KNOT;

	if (!rates) {
		printk(KERN_ERR "cs4270: could not find a valid sample rate\n");
		return -EINVAL;
	}

	codec_dai->playback->rates = rates;
	codec_dai->playback->rate_min = rate_min;
	codec_dai->playback->rate_max = rate_max;

	codec_dai->capture->rates = rates;
	codec_dai->capture->rate_min = rate_min;
	codec_dai->capture->rate_max = rate_max;

	return 0;
}

/*
 * Configure the codec for the selected audio format
 *
 * This function takes a bitmask of SND_SOC_DAIFMT_x bits and programs the
 * codec accordingly.
 *
 * Currently, this function only supports SND_SOC_DAIFMT_I2S and
 * SND_SOC_DAIFMT_LEFT_J.  The CS4270 codec also supports right-justified
 * data for playback only, but ASoC currently does not support different
 * formats for playback vs. record.
 */
static int cs4270_set_dai_fmt(struct snd_soc_dai *codec_dai,
			      unsigned int format)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct cs4270_private *cs4270 =
		container_of(codec, struct cs4270_private, codec);
	int ret = 0;

	switch (format & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
	case SND_SOC_DAIFMT_LEFT_J:
		cs4270->mode = format & SND_SOC_DAIFMT_FORMAT_MASK;
		break;
	default:
		printk(KERN_ERR "cs4270: invalid DAI format\n");
		ret = -EINVAL;
	}

	return ret;
}

/*
 * Read from the CS4270 register cache.
 *
 * This CS4270 registers are cached to avoid excessive I2C I/O operations.
 * After the initial read to pre-fill the cache, the CS4270 never updates
 * the register values, so we won't have a cache coherncy problem.
 */
static unsigned int cs4270_read_reg_cache(struct snd_soc_codec *codec,
	unsigned int reg)
{
	struct cs4270_private *cs4270 =
		container_of(codec, struct cs4270_private, codec);
	u8 *cache = cs4270->reg_cache;

	if ((reg < CS4270_FIRSTREG) || (reg > CS4270_LASTREG))
		return -EIO;

	return cache[reg - CS4270_FIRSTREG];
}

/*
 * Write to a CS4270 register via the I2C bus.
 *
 * This function writes the given value to the given CS4270 register, and
 * also updates the register cache.
 *
 * Note that we don't use the hw_write function pointer of snd_soc_codec.
 * That's because it's too clunky: the hw_write_t prototype does not match
 * i2c_smbus_write_byte_data(), and it's just another layer of overhead.
 */
static int cs4270_i2c_write(struct snd_soc_codec *codec, unsigned int reg,
			    unsigned int value)
{
	struct cs4270_private *cs4270 =
		container_of(codec, struct cs4270_private, codec);
	u8 *cache = cs4270->reg_cache;

	if ((reg < CS4270_FIRSTREG) || (reg > CS4270_LASTREG))
		return -EIO;

	/* Only perform an I2C operation if the new value is different */
	if (cache[reg - CS4270_FIRSTREG] != value) {
		struct i2c_client *client = codec->control_data;

		if (i2c_smbus_write_byte_data(client, reg, value)) {
			printk(KERN_ERR "cs4270: I2C write failed\n");
			return -EIO;
		}

		/* We've written to the hardware, so update the cache */
		cache[reg - CS4270_FIRSTREG] = value;
	}

	return 0;
}

/*
 * Program the CS4270 with the given hardware parameters.
 *
 * The .dai_ops functions are used to provide board-specific data, like
 * input frequencies, to this driver.  This function takes that information,
 * combines it with the hardware parameters provided, and programs the
 * hardware accordingly.
 */
static int cs4270_hw_params(struct snd_pcm_substream *substream,
			    struct snd_pcm_hw_params *params,
			    struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	struct cs4270_private *cs4270 =
		container_of(codec, struct cs4270_private, codec);
	int ret = 0;
	unsigned int i;
	unsigned int rate;
	unsigned int ratio;
	int reg;

	/* Figure out which MCLK/LRCK ratio to use */

	rate = params_rate(params);	/* Sampling rate, in Hz */
	ratio = cs4270->mclk / rate;	/* MCLK/LRCK ratio */

	for (i = 0; i < NUM_MCLK_RATIOS; i++) {
		if (cs4270_mode_ratios[i].ratio == ratio)
			break;
	}

	if (i == NUM_MCLK_RATIOS) {
		/* We did not find a matching ratio */
		printk(KERN_ERR "cs4270: could not find matching ratio\n");
		return -EINVAL;
	}

	/* Freeze and power-down the codec */

	ret = cs4270_i2c_write(codec, CS4270_PWRCTL, CS4270_PWRCTL_FREEZE |
		CS4270_PWRCTL_PDN_ADC | CS4270_PWRCTL_PDN_DAC |
		CS4270_PWRCTL_PDN);
	if (ret < 0) {
		printk(KERN_ERR "cs4270: I2C write failed\n");
		return ret;
	}

	/* Program the mode control register */

	reg = cs4270_read_reg_cache(codec, CS4270_MODE);
	reg &= ~(CS4270_MODE_SPEED_MASK | CS4270_MODE_DIV_MASK);
	reg |= cs4270_mode_ratios[i].speed_mode | cs4270_mode_ratios[i].mclk;

	ret = cs4270_i2c_write(codec, CS4270_MODE, reg);
	if (ret < 0) {
		printk(KERN_ERR "cs4270: I2C write failed\n");
		return ret;
	}

	/* Program the format register */

	reg = cs4270_read_reg_cache(codec, CS4270_FORMAT);
	reg &= ~(CS4270_FORMAT_DAC_MASK | CS4270_FORMAT_ADC_MASK);

	switch (cs4270->mode) {
	case SND_SOC_DAIFMT_I2S:
		reg |= CS4270_FORMAT_DAC_I2S | CS4270_FORMAT_ADC_I2S;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		reg |= CS4270_FORMAT_DAC_LJ | CS4270_FORMAT_ADC_LJ;
		break;
	default:
		printk(KERN_ERR "cs4270: unknown format\n");
		return -EINVAL;
	}

	ret = cs4270_i2c_write(codec, CS4270_FORMAT, reg);
	if (ret < 0) {
		printk(KERN_ERR "cs4270: I2C write failed\n");
		return ret;
	}

	/* Disable auto-mute.  This feature appears to be buggy, because in
	   some situations, auto-mute will not deactivate when it should. */

	reg = cs4270_read_reg_cache(codec, CS4270_MUTE);
	reg &= ~CS4270_MUTE_AUTO;
	ret = cs4270_i2c_write(codec, CS4270_MUTE, reg);
	if (ret < 0) {
		printk(KERN_ERR "cs4270: I2C write failed\n");
		return ret;
	}

	/* Thaw and power-up the codec */

	ret = cs4270_i2c_write(codec, CS4270_PWRCTL, 0);
	if (ret < 0) {
		printk(KERN_ERR "cs4270: I2C write failed\n");
		return ret;
	}

	return ret;
}

#ifdef CONFIG_SND_SOC_CS4270_HWMUTE
/*
 * Set the CS4270 external mute
 *
 * This function toggles the mute bits in the MUTE register.  The CS4270's
 * mute capability is intended for external muting circuitry, so if the
 * board does not have the MUTEA or MUTEB pins connected to such circuitry,
 * then this function will do nothing.
 */
static int cs4270_mute(struct snd_soc_dai *dai, int mute)
{
	struct snd_soc_codec *codec = dai->codec;
	int reg6;

	reg6 = cs4270_read_reg_cache(codec, CS4270_MUTE);

	if (mute)
		reg6 |= CS4270_MUTE_ADC_A | CS4270_MUTE_ADC_B |
			CS4270_MUTE_DAC_A | CS4270_MUTE_DAC_B;
	else
		reg6 &= ~(CS4270_MUTE_ADC_A | CS4270_MUTE_ADC_B |
			  CS4270_MUTE_DAC_A | CS4270_MUTE_DAC_B);

	return cs4270_i2c_write(codec, CS4270_MUTE, reg6);
}
#else
#define cs4270_mute NULL
#endif

/* A list of non-DAPM controls that the CS4270 supports */
static const struct snd_kcontrol_new cs4270_snd_controls[] = {
	SOC_DOUBLE_R("Master Playback Volume",
		CS4270_VOLA, CS4270_VOLB, 0, 0xFF, 1)
};

static int cs4270_codec_init(struct snd_soc_codec *codec,
	struct snd_soc_card *soc_card)
{
	int ret;
	unsigned int i;

	/* Add the non-DAPM controls */

	for (i = 0; i < ARRAY_SIZE(cs4270_snd_controls); i++) {
		struct snd_kcontrol *kctrl;

		kctrl = snd_soc_cnew(&cs4270_snd_controls[i], codec, NULL);

		ret = snd_ctl_add(soc_card->card, kctrl);
		if (ret < 0) {
			dev_err(soc_card->card->dev, "could not add control\n");
			return ret;
		}
	}

	return 0;
}

/*
 * Initialize the I2C interface of the CS4270
 *
 * This function is called for whenever the I2C subsystem finds a device
 * at a particular address.
 *
 * Note: snd_soc_new_pcms() must be called before this function can be called,
 * because of snd_ctl_add().
 */
static int cs4270_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct cs4270_private *cs4270;
	struct snd_soc_codec *codec = NULL;
	int ret = 0;
	int registered = 0;	/* 1 == the codec has been registered */

	/* Verify that we have a CS4270 */

	ret = i2c_smbus_read_byte_data(client, CS4270_CHIPID);
	if (ret < 0) {
		dev_err(&client->dev, "failed to read I2C\n");
		return ret;
	}
	/* The top four bits of the chip ID should be 1100. */
	if ((ret & 0xF0) != 0xC0)
		/* The device at this address is not a CS4270 codec */
		return -ENODEV;

	dev_info(&client->dev, "found device at address %X\n", client->addr);
	dev_info(&client->dev, "hardware revision %X\n", ret & 0xF);

	/*
	 * Normally, we'd call snd_soc_new_codec, but that function
	 * allocates a snd_soc_codec struct, and we don't want that.  So we have
	 * to initialize all the fields ourselves.  This might break in the
	 * future.
	 */
	cs4270 = kzalloc(sizeof(struct cs4270_private), GFP_KERNEL);
	if (!cs4270) {
		dev_err(&client->dev, "could not allocate codec structure\n");
		return -ENOMEM;
	}

	/* The I2C interface is set up, so pre-fill our register cache */
	ret = i2c_smbus_read_i2c_block_data(client, CS4270_FIRSTREG | 0x80,
		CS4270_NUMREGS, cs4270->reg_cache);
	if (ret != CS4270_NUMREGS) {
		dev_err(&client->dev, "failed to fill register cache\n");
		ret = -EIO;
		goto error;
	}

	strcpy(cs4270->name, "cirrus,cs4270");

	cs4270->playback.stream_name = "Playback";
	cs4270->playback.channels_min = 1;
	cs4270->playback.channels_max = 2;
	cs4270->playback.formats = CS4270_FORMATS;

	cs4270->capture.stream_name = "Capture";
	cs4270->capture.channels_min = 1;
	cs4270->capture.channels_max = 2;
	cs4270->capture.formats = CS4270_FORMATS;

	cs4270->dai_ops.hw_params = cs4270_hw_params;
	cs4270->dai_ops.set_sysclk = cs4270_set_dai_sysclk;
	cs4270->dai_ops.set_fmt = cs4270_set_dai_fmt;
	cs4270->dai_ops.digital_mute = cs4270_mute;

	codec = &cs4270->codec;

	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->list);
	INIT_LIST_HEAD(&codec->dai_list);
	codec->name = cs4270->name;

	codec->control_data = client;
	codec->init = cs4270_codec_init;

	ret = snd_soc_register_codec(codec, &client->dev);
	if (ret < 0) {
		printk(KERN_ERR "cs4270: failed to register card\n");
		goto error;
	}
	registered = 1;

	cs4270->dai_new.name = cs4270->name;
	cs4270->dai_new.playback = &cs4270->playback;
	cs4270->dai_new.capture = &cs4270->capture;
	cs4270->dai_new.ops = &cs4270->dai_ops;

	cs4270->dai =
		snd_soc_register_codec_dai(&cs4270->dai_new, &client->dev);
	if (!cs4270->dai) {
		ret = -EINVAL;
		goto error;
	}

	i2c_set_clientdata(client, codec);

	return 0;

error:
	if (cs4270->dai)
		snd_soc_unregister_codec_dai(cs4270->dai);

	if (registered)
		snd_soc_unregister_codec(codec);

	kfree(cs4270);

	return ret;
}

static int cs4270_i2c_remove(struct i2c_client *client)
{
	struct snd_soc_codec *codec = i2c_get_clientdata(client);
	struct cs4270_private *cs4270 =
		container_of(codec, struct cs4270_private, codec);

	i2c_set_clientdata(client, NULL);

	snd_soc_unregister_codec_dai(cs4270->dai);
	snd_soc_unregister_codec(codec);

	kfree(cs4270);

	return 0;
}

static const struct i2c_device_id cs4270_id[] = {
	{"cs4270", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cs4270_id);

static struct i2c_driver cs4270_i2c_driver = {
	.driver = {
		.name = "cs4270",
		.owner = THIS_MODULE,
	},
	.id_table = cs4270_id,
	.probe = cs4270_i2c_probe,
	.remove = cs4270_i2c_remove,
};

/**
 * cs4270_init: codec driver initialization.
 *
 * This function is called when this module is loaded.
 */
static int __init cs4270_init(void)
{
	int ret;

	printk(KERN_INFO "Cirrus Logic CS4270 ASoC codec driver\n");

	/* i2c_add_driver() will call cs4270_i2c_probe() */
	ret = i2c_add_driver(&cs4270_i2c_driver);

	if (ret)
		printk(KERN_ERR "cs4270: failed to register I2C driver\n");

	return ret;
}

/**
 * cs4270_exit: codec driver exit
 *
 * This function is called when this driver is unloaded.
 */
static void __exit cs4270_exit(void)
{
	i2c_del_driver(&cs4270_i2c_driver);
}

module_init(cs4270_init);
module_exit(cs4270_exit);

MODULE_AUTHOR("Timur Tabi <timur@freescale.com>");
MODULE_DESCRIPTION("Cirrus Logic CS4270 ASoC codec driver");
MODULE_LICENSE("GPL");
