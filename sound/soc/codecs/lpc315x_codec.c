/*
 * sound/soc/codecs/lpc315x_codec.c
 *
 * Author: Kevin Wells <kevin.wells@nxp.com>
 *
 * Copyright (C) 2010 NXP Semiconductors
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <sound/core.h>
#include <sound/control.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/tlv.h>

#include <mach/lpc315x_ad.h>
#include "lpc315x_codec.h"

#define ANALOG_DIE_I2C_ADDR 	(0x0C)

#define DEC_GAIN 		(0x0)
#define AVC_COA_VOL 		(0x7C0)
#define AVC_FIN_VOL 		(0x1)

/*
 * LPC315X CODEC structure
 * */
static struct snd_soc_codec *lpc315x_codec;

/*
 * LPC315X codec private data
 * */
struct lpc315x_codec_priv {
	struct snd_soc_codec	codec;
	u32			lpc315x_reg_cache[LPC315X_CODEC_NUM_REGS];
	struct work_struct	work;
};
static struct lpc315x_codec_priv *lpc315x_codec_prv;

/*
 * LPC315X CODEC register cache
 */
static const u32 lpc315x_codec_regs[LPC315X_CODEC_NUM_REGS] = {
	0x00000000, /* PGA */
	0x0002FFFC, /* AIN_1 */
	0x8000F5FB, /* AOUT */
	0x00000000, /* DEC */
	0x00000200, /* INT0 */
	0x00000000, /* INT1 */
	0x00010303, /* I2SMUX1 */
	0x00000000, /* DEC_STA */
};

/* Register cache dirty flags */
static unsigned long lpc315x_codec_cache_dirty;

/*
 * Read LPC315X CODEC register cache
 */
static inline u32 lpc315x_codec_read_reg_cache(struct snd_soc_codec *codec,
		unsigned int reg)
{
	u32 *cache = codec->reg_cache;

	/* Check if register offset is valid */
	if (reg > LPC315X_CODEC_DEC_STA) {
		return -1;
	}

	/* Return data */
	return cache[reg - LPC315X_CODEC_CODEC_START];
}

/*
 * Write LPC315X CODEC register cache
 */
static inline void lpc315x_codec_write_reg_cache(struct snd_soc_codec
		*codec, unsigned int reg, u32 value)
{
	u32 *cache = codec->reg_cache;

	/* Check if register offset is valid */
	if (reg > LPC315X_CODEC_DEC_STA)
		return;

	/* Set dirty flag */
	if (cache[reg - LPC315X_CODEC_CODEC_START] != value) {
		set_bit((reg - LPC315X_CODEC_CODEC_START),
				&lpc315x_codec_cache_dirty);
	}

	/* Write to register cache */
	cache[reg - LPC315X_CODEC_CODEC_START] = value;
	return;
}

/*
 * Write to LPC315X CODEC registers using I2C functions
 */
static int lpc315x_codec_register_rw(struct snd_soc_codec *codec,
		unsigned int reg, u32 *value, int read)
{
	int ret = 0;
	u8 off[2], data[6];
	u32 val = 0;

	if(read) {
		/*
		* Send register offsets:
		*  data[0] is MSB of register offset (0x00)
	 	*  data[1] is LSB of register offset
		*  Read register data:
	 	*  data[0] is Data bits 31:24
	 	*  data[1] is Data bits 23:16
	 	*  data[2] is Data bits 15:8
	 	*  data[3] is Data bits 7:0
	 	*/
		/* Register offsets */
		off[0] = (u8) ((reg >> 8) & 0xFF);
		off[1] = (u8) (reg & 0xFF);

		/* Read Operation */
		if (codec->hw_write(codec->control_data, off, 2) == 2) {
			if (i2c_master_recv(codec->control_data, data, 4) == 4) {
				*value = ((data[0] & 0xFF) << 24);
				*value |= ((data[1] & 0xFF) << 16);
				*value |= ((data[2] & 0xFF) << 8);
				*value |= (data[3] & 0xFF);
				pr_debug("lpc315x codec read: hw read %x val %x\n", reg, *value);
			}
			else {
				pr_debug("lpc315x codec read: hw read reg %x failed \n", reg);
				ret = -EIO;
			}
		}
		else {
			pr_debug("lpc315x codec read: hw send addres %x failed \n", reg);
			ret = -EIO;
		}
	}
	else {
		/*  Send register offsets, followed by data bytes
	 	 *  data[0] is MSB of register offset (0x00)
	 	 *  data[1] is LSB of register offset
	 	 *  data[2] is Data bits 31:24
	 	 *  data[3] is Data bits 23:16
	 	 *  data[4] is Data bits 15:8
	 	 *  data[5] is Data bits 7:0
	 	*/
		val = *value;
		data[0] = (u8) ((reg >> 8) & 0xFF);
		data[1] = (u8) (reg & 0xFF);
		data[2] = (u8) ((val >> 24) & 0xFF);
		data[3] = (u8) ((val >> 16) & 0xFF);
		data[4] = (u8) ((val >> 8) & 0xFF);
		data[5] = (u8) (val & 0xFF);

		/* Write to CODEC register */
		if (codec->hw_write(codec->control_data, data, 6) != 6) {
			pr_debug("lpc315x codec write: hw reg write  %x failed \n", reg);
			ret = -EIO;
		}
	}

	return ret;
}

/*
 * Write to LPC315X CODEC register space
 */
static int lpc315x_codec_write(struct snd_soc_codec *codec,
		unsigned int reg, unsigned int value)
{
	int ret = 0;

	/* Write to reg cache */
	lpc315x_codec_write_reg_cache(codec, reg, value);

	/* Write to Analog Die register */
	ret = lpc315x_codec_register_rw(codec, reg, &value, 0);
	if(!ret) {
		pr_debug("lpc315x codec: hw write %x val %x\n", reg, value);

		/* Clear cache dirty flag */
		clear_bit((reg - LPC315X_CODEC_CODEC_START),
			&lpc315x_codec_cache_dirty);
	}

	return ret;
}

/*
 * LPC315X CODEC work queue function
 * */
static void lpc315x_codec_work(struct work_struct *work)
{
	u16 bit, reg;
	u32 data;

	for_each_bit(bit, &lpc315x_codec_cache_dirty,
			(LPC315X_CODEC_DEC_STA - LPC315X_CODEC_CODEC_START)) {
		reg = bit + LPC315X_CODEC_CODEC_START;
		data = lpc315x_codec_read_reg_cache(lpc315x_codec, reg);
		lpc315x_codec_write(lpc315x_codec, reg, data);
		clear_bit(bit, &lpc315x_codec_cache_dirty);
	}
}

/*
 * LPC315X CODEC volatge ramp up & ramp down function
 * for Playback functionality
 * This will be called after the DAC widget is powered up
 * snd after powered down
 * */
static int lpc315x_codec_ref_vol(struct snd_soc_dapm_widget *w,
				 struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	unsigned int cgu_reg = 0;
	u32 aout_reg, int0_reg, ain_reg, int1_reg, i2srx1_reg;

	switch(event) {
	case SND_SOC_DAPM_POST_PMU:
		/* Complete the power up sequence for playback.
		 * This code will be executed after DACs are powered up
		 * */
		/* By Default, decimator is connected Interpolator.
		 * Connect I2SRX1 output Interpolator.
		 * */
		i2srx1_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_I2SMUX1);
		i2srx1_reg &= ~(LPC315X_CODEC_I2SMUX1_DEMUX_MSK);
		lpc315x_codec_write(codec, LPC315X_CODEC_I2SMUX1, i2srx1_reg);

		/* Connect DAC outputs to HP Amplifiers */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg |= LPC315X_CODEC_AOUT_SWDAC_ON;
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);

		/* Ramp up Interpolator volatge */
		int0_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_INT0);
		int0_reg &= ~(LPC315X_CODEC_INT0_PD_DAC);
		lpc315x_codec_write(codec, LPC315X_CODEC_INT0, int0_reg);
		mdelay(30);

		/* Ramp up Reference Volatge */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg |= LPC315X_CODEC_AOUT_VREF_SLOW_UP;
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);
		mdelay(500);

		/* Set Interpolator Volume & Unmute */
		int1_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_INT1);
		int1_reg &= ~(LPC315X_CODEC_INT1_MAS_MUTE |
				LPC315X_CODEC_INT1_MAS_VOL_L_MSK |
		     	     LPC315X_CODEC_INT1_MAS_VOL_R_MSK);
		lpc315x_codec_write(codec, LPC315X_CODEC_INT1, int1_reg);

		/* By default AVC is muted, set AVC Volume & unmute */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg &= ~(LPC315X_CODEC_AOUT_AVC_COA_GAIN_MSK |
				LPC315X_CODEC_AOUT_AVC_FIN_GAIN_MSK);
		aout_reg |=
			((AVC_COA_VOL << LPC315X_CODEC_AOUT_AVC_COA_GAIN_POS)|
			(AVC_FIN_VOL << LPC315X_CODEC_AOUT_AVC_FIN_GAIN_POS));
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);

		break;

	case SND_SOC_DAPM_POST_PMD:
		/* Complete the power down sequence for playback.
		 * This code will be executed aftet DACs are powered down
		 * */

		/* Power down Central Reference source */
		ain_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AIN1);
		ain_reg |= (LPC315X_CODEC_AIN1_PD_VCOM_VREF1);
		lpc315x_codec_write(codec, LPC315X_CODEC_AIN1, ain_reg);

		/* Power down Reference buffer Voltage */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg |= (LPC315X_CODEC_AOUT_VREF_SLOW);
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);

		/* Power down HP Amplifiers */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg |= (LPC315X_CODEC_AOUT_PD_HP_L |
				LPC315X_CODEC_AOUT_PD_HP_C |
				LPC315X_CODEC_AOUT_PD_HP_R );
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);

		/* Switch off NS, DSP, DAC clocks */
		lpc315x_codec_register_rw(codec, LPC315X_CODEC_CGU,
				&cgu_reg, 1);
		cgu_reg |= (LPC315X_CODEC_CGU_PD_DSP_CLK |
				LPC315X_CODEC_CGU_PD_NS_CLK |
				LPC315X_CODEC_CGU_PD_I2SRX_BCLK |
				LPC315X_CODEC_CGU_PD_I2SRX_SYSCLK |
				LPC315X_CODEC_CGU_PD_I2C256FS_CLK |
				LPC315X_CODEC_CGU_PD_DAC_CLK);
		lpc315x_codec_register_rw(codec, LPC315X_CODEC_CGU,
				&cgu_reg, 0);
		break;
	default:
		BUG();
	}

	return 0;
}

/* Digital Demux control */
static const char *lpc315x_codec_dmux_sel[] = {
	"I2SRX1",
	"Decimator",
};

/* De-emphasis control */
static const char *lpc315x_codec_deemp_sel[] = {
	"None",
	"32kHz",
	"44.1kHz",
	"48kHz",
	"96kHz",
};

/* Silence detection time window */
static const char *lpc315x_codec_sdet_sel[] = {
	"3200fs samples",
	"4800fs samples",
	"9600fs samples",
	"19200fs samples",
};

/* Nosie Shaper Enums */
static const char *lpc315x_codec_ns_sel[] = {
	"1-bit",
	"4-bit",
	"5-bit",
	"6-bit",
};

/* Interpolator Input data rate Enums */
static const char *lpc315x_codec_os_sel[] = {
	"1fs",
	"2fs0",
	"2fs1",
	"8fs",
};

/* Interpolator Filter Settings Enums */
static const char *lpc315x_codec_fil_sel[] = {
	"2fs-8fs FIR for slow0",
	"2fs-8fs FIR for slow1",
	"2fs-8fs FIR for slow2",
	"2fs-8fs FIR for sharp",
};

/* Data Weight Enums */
static const char *lpc315x_codec_dwa_sel[] = {
	"Uni-Directional",
	"Bi-Directional",
};

/* MUX0 enum */
static const char *lpc315x_mux0_sel[] = {
	"Tuner In",
	"Line In",
};

/* MUX1 enum */
static const char *lpc315x_mux1_sel[] = {
	"Lin/Tun In",
	"Mic In",
};

/* DAC Switch enum */
static const char *lpc315x_swdac_sel[] = {
	"AVC only",
	"Both AVC & SDAC",
};

/* Digital Demux selection */
static const struct soc_enum lpc315x_codec_dmux_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_I2SMUX1, 16, 2, lpc315x_codec_dmux_sel);

/* from -64 to 63 dB in 0.5 dB steps (-128...63) */
static DECLARE_TLV_DB_SCALE(int_gain_tlv, -6400, 50, 1);

/* De-emphasis settings */
static const struct soc_enum lpc315x_codec_deemp_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_INT0, 0, 5, lpc315x_codec_deemp_sel);

/* Silence Detector Settings */
static const struct soc_enum lpc315x_codec_sdet_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_INT0, 4, 4, lpc315x_codec_sdet_sel);

/* Interpolator Input data rate */
static const struct soc_enum lpc315x_codec_ns_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_INT0, 9, 4, lpc315x_codec_ns_sel);

/* Interpolator Input data rate */
static const struct soc_enum lpc315x_codec_os_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_INT0, 11, 4, lpc315x_codec_os_sel);

/* Interpolator Filter Settings */
static const struct soc_enum lpc315x_codec_fil_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_INT0, 13, 4, lpc315x_codec_fil_sel);

/* from -64 to 63 dB in 0.5 dB steps (-128...63) */
static DECLARE_TLV_DB_SCALE(dec_gain_tlv, -6400, 50, 1);

/* Data weight algorithm */
static const struct soc_enum lpc315x_codec_dwa_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_AOUT, 2, 2, lpc315x_codec_dwa_sel);

/* from 0 to 24dB in 3dB steps */
static DECLARE_TLV_DB_SCALE(line_pga_tlv, 0, 300, 0);

/* from 0 to 24dB in 3dB steps */
static DECLARE_TLV_DB_SCALE(mic_pga_tlv, 0, 300, 0);

/*
 * LPC315X AVC Coarse Volume control functions
 * */
/* from 0 to -60dB in 6dB steps */
static DECLARE_TLV_DB_SCALE(avc_coa_tlv, -6000, 600, 1);

/*
 * LPC315X AVC Coarse Volume put function
 * */
static int snd_soc_lpc315x_coa_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 i, val, reg, avc_gain;

	val = (ucontrol->value.integer.value[0] & 0xF);
	/* conversion to AVC Coarse volume bit pattern
	* 0000  = Mute  (00000000000) = 0x000
	* 0001  = -60dB (10000000000) = 0x400
	* 0010  = -54dB (11000000000) = 0x600
	* 0011  = -48dB (11100000000) = 0x700
	* 0100  = -42dB (11110000000) = 0x780
	* 0101  = -36dB (11111000000) = 0x7C0
	* 0110  = -30dB (11111100000) = 0x7E0
	* 0111  = -24dB (11111110000) = 0x7F0
	* 1000  = -18dB (11111111000) = 0x7F8
	* 1001  = -12dB (11111111100) = 0x7FC
	* 1010  = -06dB (11111111110) = 0x7FE
	* 1011  =  00dB (11111111111) = 0x7FF (Max)
	*/
	if(val >= 0xB) {
		avc_gain = 0x7FF; /* 0dB */
	}
	else if(!val) {
		avc_gain = 0; /* Mute */
	}
	else {
		/* Calculate bit pattern */
		avc_gain = 0x400;
		for(i = 1; i < val; i++) {
			avc_gain |= (avc_gain >> 1);
		}
	}

	/* Write to register */
	reg = lpc315x_codec_read_reg_cache(codec, LPC315X_CODEC_AOUT);
       	reg &= ~(0x7FF << LPC315X_CODEC_AOUT_AVC_COA_GAIN_POS);
	reg |= (avc_gain << LPC315X_CODEC_AOUT_AVC_COA_GAIN_POS);
	lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, reg);

	return 0;
}

/*
 * LPC315X AVC Coarse Volume get function
 * */
static int snd_soc_lpc315x_coa_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 reg, cnt;

	/* Get value from register */
	reg = lpc315x_codec_read_reg_cache(codec, LPC315X_CODEC_AOUT);

	/* Calculate count value from register value */
       	reg = ((reg >> LPC315X_CODEC_AOUT_AVC_COA_GAIN_POS) & 0x7FF);
	if(reg >= 0x7FF) {
		cnt = 0xB;
	}
	else if(!reg) {
		cnt = 0;
	}
	else {
		for(cnt = 1; reg; cnt++) {
			reg &= (reg - 1);
		}
	}
	ucontrol->value.integer.value[0] = cnt;
	return 0;

}

/* AVC Coarse Volume TLV macro */
#define SOC_LPC315X_AVCCOA_SINGLE_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_lpc315x_coa_get_volsw,\
	.put = snd_soc_lpc315x_coa_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }

/*
 * LPC315X AVC Fine Volume control functions
 * */
/* from 0 to -4.5dB in 1.5dB steps */
static DECLARE_TLV_DB_SCALE(avc_fin_tlv, -450, 150, 1);

/*
 * LPC315X AVC Fine Volume put function
 * */
static int snd_soc_lpc315x_fin_put_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 val, reg, avc_gain;

	val = (ucontrol->value.integer.value[0] & 0x7);
	/* conversion to AVC Fine volume bit pattern
	* 000  = -4.5dB (000) = 0x0
	* 001  = -3.0dB (001) = 0x1
	* 010  = -1.5dB (011) = 0x3
	* 011  =  0.0dB (111) = 0x7 (Max)
	*/
	if( val >= 0x3) {
		val = 0x3;
	}
	avc_gain = (1 << val) - 1;

	/* Write into register */
	reg = lpc315x_codec_read_reg_cache(codec, LPC315X_CODEC_AOUT);
       	reg &= ~(0x7 << LPC315X_CODEC_AOUT_AVC_FIN_GAIN_POS);
	reg |= (avc_gain << LPC315X_CODEC_AOUT_AVC_FIN_GAIN_POS);
	lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, reg);

	return 0;
}

/*
 * LPC315X AVC Fine Volume get function
 * */
static int snd_soc_lpc315x_fin_get_volsw(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_codec *codec = snd_kcontrol_chip(kcontrol);
	u32 reg, cnt;

	/* Get value from register */
	reg = lpc315x_codec_read_reg_cache(codec, LPC315X_CODEC_AOUT);

	/* Calculate count value from register value */
       	reg = ((reg >> LPC315X_CODEC_AOUT_AVC_FIN_GAIN_POS) & 0x7);
	cnt = 0;
	while(reg != ((1 << cnt) - 1)) {
		cnt++;
	}

	if(cnt >= 0x3) {
		cnt = 0x3;
	}
	ucontrol->value.integer.value[0] = cnt;
	return 0;

}

/* AVC Fine Volume TLV macro */
#define SOC_LPC315X_AVCFIN_SINGLE_TLV(xname, reg, shift, max, invert, tlv_array) \
{	.iface = SNDRV_CTL_ELEM_IFACE_MIXER, .name = xname, \
	.access = SNDRV_CTL_ELEM_ACCESS_TLV_READ |\
		 SNDRV_CTL_ELEM_ACCESS_READWRITE,\
	.tlv.p = (tlv_array), \
	.info = snd_soc_info_volsw, .get = snd_soc_lpc315x_fin_get_volsw,\
	.put = snd_soc_lpc315x_fin_put_volsw, \
	.private_value =  SOC_SINGLE_VALUE(reg, shift, max, invert) }
/*
 * Audio Controls
 * */
static const struct snd_kcontrol_new lpc315x_codec_snd_ctrls[] = {
	/* MIC PGA Gain Control */
	SOC_SINGLE_TLV("Mic Capture Volume", LPC315X_CODEC_PGA, 4, 8, 0,
			mic_pga_tlv),
	/* PGA Gain Control for Left & Right channels */
	SOC_DOUBLE_TLV("Line Capture Volume", LPC315X_CODEC_PGA, 8, 0, 8, 0,
			line_pga_tlv),
	/* ADC Dither Input for Left & Right Channels */
	SOC_DOUBLE("ADC Dither Input", LPC315X_CODEC_AIN1, 3, 2, 1, 0),
	/* Feedback in Loopfilter */
	SOC_DOUBLE("Loopfilter Feedback", LPC315X_CODEC_AIN1, 1, 0, 1, 0),
	/* AVC outputs to AB Amplifier */
	SOC_SINGLE("AVC Output Switch", LPC315X_CODEC_AOUT, 31, 1, 0),
	/* AVC Coarse Volume Control */
	SOC_LPC315X_AVCCOA_SINGLE_TLV("AVC Coarse Volume Control",
			LPC315X_CODEC_AOUT, 16, 11, 0, avc_coa_tlv),
	/* AVC Fine Volume Control */
	SOC_LPC315X_AVCFIN_SINGLE_TLV("AVC Fine Volume Control",
			LPC315X_CODEC_AOUT, 27, 3, 0, avc_fin_tlv),
	/* Data Weight Algorithm */
	SOC_ENUM("Data Weight Algorithm", lpc315x_codec_dwa_enum),
	/* AGC Time settings */
	SOC_SINGLE("AGC Timing", LPC315X_CODEC_DEC, 25, 7, 0),
	/* AGC Levels */
	SOC_SINGLE("AGC Target level", LPC315X_CODEC_DEC, 23, 3, 1),
	/* AGC Switch */
	SOC_SINGLE("AGC Switch", LPC315X_CODEC_DEC, 22, 1, 0),
	/* Decimator Mute */
	SOC_SINGLE("ADC Dec Mute", LPC315X_CODEC_DEC, 21, 1, 0),
	/* Channel Polarity Inverting Switch */
	SOC_SINGLE("ADC Polarity Inv Switch", LPC315X_CODEC_DEC,
			20, 1, 0),
	/* Input DC blocking filter (before Decimator) */
	SOC_SINGLE("Input DC Filter Switch", LPC315X_CODEC_DEC, 19, 1, 0),
	/* Output DC blocking filter (after Decimator) */
	SOC_SINGLE("Output DC Filter Switch", LPC315X_CODEC_DEC, 18, 1, 0),
	/* Enable soft start up after reset switch */
	SOC_SINGLE("Db Linear After Reset Switch", LPC315X_CODEC_DEC,
			17, 1, 0),
	/* Enable  sy timer after reset switch */
	SOC_SINGLE("Timer After Reset Switch", LPC315X_CODEC_DEC, 16, 1, 0),
	/* ADC Capture volume */
	SOC_DOUBLE_S8_TLV("ADC Capture Volume", LPC315X_CODEC_DEC,
			-128, 127, dec_gain_tlv),
	/* DAC Polarity Inversion Switch */
	SOC_SINGLE("DAC Polarity inverting Switch", LPC315X_CODEC_INT0,
			15, 1, 0),
	/* Interpolator Filter Settings */
	SOC_ENUM("INT Filter Settings", lpc315x_codec_fil_enum),
	/* Input data rate */
	SOC_ENUM("Oversampling Input", lpc315x_codec_os_enum),
	/* Nosie Shaper Settings */
	SOC_ENUM("Noise Shaper", lpc315x_codec_ns_enum),
	/* Silence Enable Switch */
	SOC_SINGLE("Overule Silence Dit Switch", LPC315X_CODEC_INT0,
			3, 1, 0),
	/* Silence Detector Switch */
	SOC_SINGLE("Silence Detector Switch", LPC315X_CODEC_INT0, 6, 1, 0),
	/* Silence Detector Settings */
	SOC_ENUM("Silence Detector Setting", lpc315x_codec_sdet_enum),
	/* De-emphasis settings */
	SOC_ENUM("PCM Playback De-emphasis", lpc315x_codec_deemp_enum),
	/* Interpolator Volume Control Settings */
	SOC_DOUBLE_S8_TLV("ADC Playback Volume", LPC315X_CODEC_INT1,
			-128, 127, int_gain_tlv),
	/* Digital Mux Selection */
	SOC_ENUM("Digital Mux Switch", lpc315x_codec_dmux_enum),
};

static const struct soc_enum lpc315x_muxl0_sel_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_AIN1, 17, 2, lpc315x_mux0_sel);
static const struct soc_enum lpc315x_muxr0_sel_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_AIN1, 15, 2, lpc315x_mux0_sel);
static const struct soc_enum lpc315x_muxl1_sel_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_AIN1, 18, 2, lpc315x_mux1_sel);
static const struct soc_enum lpc315x_muxr1_sel_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_AIN1, 16, 2, lpc315x_mux1_sel);
static const struct soc_enum lpc315x_swdac_sel_enum =
	SOC_ENUM_SINGLE(LPC315X_CODEC_AOUT, 31, 2, lpc315x_swdac_sel);
static const struct snd_kcontrol_new lpc315x_muxl0_control =
	SOC_DAPM_ENUM("Route", lpc315x_muxl0_sel_enum);
static const struct snd_kcontrol_new lpc315x_muxr0_control =
	SOC_DAPM_ENUM("Route", lpc315x_muxr0_sel_enum);
static const struct snd_kcontrol_new lpc315x_muxl1_control =
	SOC_DAPM_ENUM("Route", lpc315x_muxl1_sel_enum);
static const struct snd_kcontrol_new lpc315x_muxr1_control =
	SOC_DAPM_ENUM("Route", lpc315x_muxr1_sel_enum);
static const struct snd_kcontrol_new lpc315x_output_mux_control =
	SOC_DAPM_ENUM("Route", lpc315x_swdac_sel_enum);

/*
 * Audio widgets
 * */
static const struct snd_soc_dapm_widget lpc315x_codec_dapm_widgets[] = {
	SND_SOC_DAPM_MUX("Output Mux", SND_SOC_NOPM, 0, 0,
		&lpc315x_output_mux_control),
	SND_SOC_DAPM_MUX("MUX_L0", SND_SOC_NOPM, 0, 0,
		&lpc315x_muxl0_control),
	SND_SOC_DAPM_MUX("MUX_R0", SND_SOC_NOPM, 0, 0,
		&lpc315x_muxr0_control),
	SND_SOC_DAPM_MUX("MUX_L1", SND_SOC_NOPM, 0, 0,
		&lpc315x_muxl1_control),
	SND_SOC_DAPM_MUX("MUX_R1", SND_SOC_NOPM, 0, 0,
		&lpc315x_muxr1_control),
	SND_SOC_DAPM_PGA("Mic LNA", LPC315X_CODEC_AIN1, 14, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Mic PGA", LPC315X_CODEC_AIN1, 12, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Left PGA", LPC315X_CODEC_AIN1, 13, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right PGA", LPC315X_CODEC_AIN1, 11, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Mic SDC", LPC315X_CODEC_AIN1, 9, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Left SDC", LPC315X_CODEC_AIN1, 10, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right SDC", LPC315X_CODEC_AIN1, 8, 1, NULL, 0),
	SND_SOC_DAPM_ADC("SADC Left", "Left Capture",
			LPC315X_CODEC_AIN1, 7, 1),
	SND_SOC_DAPM_ADC("SADC Right", "Right Capture",
			LPC315X_CODEC_AIN1, 6, 1),
	SND_SOC_DAPM_INPUT("ADC_MIC"),
	SND_SOC_DAPM_INPUT("ADC_VINL"),
	SND_SOC_DAPM_INPUT("ADC_VINR"),
	SND_SOC_DAPM_INPUT("ADC_TINL"),
	SND_SOC_DAPM_INPUT("ADC_TINR"),
	SND_SOC_DAPM_OUTPUT("HP_OUTL"),
	SND_SOC_DAPM_OUTPUT("HP_OUTC"),
	SND_SOC_DAPM_OUTPUT("HP_OUTR"),
	SND_SOC_DAPM_PGA("HP Amp Left", LPC315X_CODEC_AOUT, 14, 1, NULL, 0),
	SND_SOC_DAPM_PGA("HP Amp Right", LPC315X_CODEC_AOUT, 12, 1, NULL, 0),
	SND_SOC_DAPM_PGA("AVC Left", LPC315X_CODEC_AOUT, 1, 1, NULL, 0),
	SND_SOC_DAPM_PGA("AVC Right", LPC315X_CODEC_AOUT, 0, 1, NULL, 0),
	SND_SOC_DAPM_DAC("SDAC Right", "Right Playback",
			LPC315X_CODEC_AOUT, 4, 1),
	SND_SOC_DAPM_DAC_E("SDAC Left", "Left Playback",
			LPC315X_CODEC_AOUT, 5, 1, lpc315x_codec_ref_vol,
			SND_SOC_DAPM_POST_PMD | SND_SOC_DAPM_POST_PMU),
};

/*
 * Audio paths
 * */
static const struct snd_soc_dapm_route audio_map[] = {
	/* Audio output path */
	{"HP_OUTL", NULL, "HP Amp Left"},
	{"HP_OUTR", NULL, "HP Amp Right"},

	/* Headphone Amplifier */
	{"HP Amp Left", NULL, "AVC Left"},
	{"HP Amp Right", NULL, "AVC Right"},
	{"HP Amp Right", "Both AVC & SDAC", "SDAC Right"},
	{"HP Amp Left", "Both AVC & SDAC", "SDAC Left"},

	/* Audio  Input paths */
	/* ADC <- MUX1 */
	{"SADC Left", NULL, "MUX_L1"},
	{"SADC Right", NULL, "MUX_R1"},

	/* MUX1 <- Mic SDC */
	{"MUX_L1", "Mic In", "Mic SDC"},
	{"MUX_R1", "Mic In", "Mic SDC"},
	/* Mic SDC <- Mic PGA */
	{"Mic SDC", NULL, "Mic PGA"},
	/* Mic PGA <- Mic LNA */
	{"Mic PGA", NULL, "Mic LNA"},
	/* Mic LNA <- MIC Input */
	{"Mic LNA", NULL, "ADC_MIC"},

	/* MUX1 <- Left & Right SDC */
	{"MUX_L1", "Lin/Tun In", "Left SDC"},
	{"MUX_R1", "Lin/Tun In", "Right SDC"},
	/* Left & Right SDC <- Left & Right PGA */
	{"Left SDC", NULL, "Left PGA"},
	{"Right SDC", NULL, "Right PGA"},
	/* Left & Right PGA <- MUX0 */
	{"Left PGA", NULL, "MUX_L0"},
	{"Right PGA", NULL, "MUX_R0"},
	/* MUX0 <- Line/Tuner Stereo Inputs */
	{"MUX_L0", "Line In", "ADC_VINL"},
	{"MUX_R0", "Line In", "ADC_VINR"},
	{"MUX_L0", "Tuner In", "ADC_TINL"},
	{"MUX_R0", "Tuner In", "ADC_TINR"},

};

/*
 * LPC315X CODEC Add Audio widgets function
 * */
static int lpc315x_codec_add_widgets(struct snd_soc_codec *codec)
{
	/* Add LPC315X CODEC audio widgets */
	snd_soc_dapm_new_controls(codec, lpc315x_codec_dapm_widgets,
				  ARRAY_SIZE(lpc315x_codec_dapm_widgets));

	/* Add LPC315X CODEC audio paths */
	snd_soc_dapm_add_routes(codec, audio_map, ARRAY_SIZE(audio_map));
	return 0;
}

/*
 * LPC315X CODEC set format function for both Playback & Capture
 * */
static int lpc315x_codec_set_dai_fmt_both(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u32 i2s_mux_reg;

	/* Set up DAI based upon fmt */
	i2s_mux_reg = lpc315x_codec_read_reg_cache(codec,
			LPC315X_CODEC_I2SMUX1);
	i2s_mux_reg &= ~(LPC315X_CODEC_I2SMUX1_TX_FMT_MSK |
			LPC315X_CODEC_I2SMUX1_RX_FMT_MSK);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		i2s_mux_reg |= LPC315X_CODEC_I2SMUX1_TX_FMT_I2S |
			LPC315X_CODEC_I2SMUX1_RX_FMT_I2S;
		break;
	case SND_SOC_DAIFMT_LSB:
		i2s_mux_reg |= LPC315X_CODEC_I2SMUX1_TX_FMT_LSB_16 |
			LPC315X_CODEC_I2SMUX1_RX_FMT_LSB_16;
		break;
	}

	/* DATAI is slave only, so in single-link mode,
	 * this has to be slave */
	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS)
		return -EINVAL;

	lpc315x_codec_write(codec, LPC315X_CODEC_I2SMUX1, i2s_mux_reg);

	return 0;
}

/*
 * LPC315X CODEC set format function for Playback
 * */
static int lpc315x_codec_set_dai_fmt_playback(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u32 i2s_mux_reg;

	/* Set up DAI based upon fmt */
	i2s_mux_reg = lpc315x_codec_read_reg_cache(codec,
			LPC315X_CODEC_I2SMUX1);
	i2s_mux_reg &= ~(LPC315X_CODEC_I2SMUX1_RX_FMT_MSK);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		i2s_mux_reg |= LPC315X_CODEC_I2SMUX1_RX_FMT_I2S;
		break;
	case SND_SOC_DAIFMT_LSB:
		i2s_mux_reg |= LPC315X_CODEC_I2SMUX1_RX_FMT_LSB_16;
		break;
	}

	/* DATAI is slave only, so this has to be slave */
	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS)
		return -EINVAL;

	lpc315x_codec_write(codec, LPC315X_CODEC_I2SMUX1, i2s_mux_reg);

	return 0;
}

/*
 * LPC315X CODEC set format function for Capture
 * */
static int lpc315x_codec_set_dai_fmt_capture(struct snd_soc_dai *codec_dai,
		unsigned int fmt)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u32 i2s_mux_reg;

	/* set up DAI based upon fmt */
	i2s_mux_reg = lpc315x_codec_read_reg_cache(codec,
			LPC315X_CODEC_I2SMUX1);
	i2s_mux_reg &= ~(LPC315X_CODEC_I2SMUX1_TX_FMT_MSK);

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		i2s_mux_reg |= LPC315X_CODEC_I2SMUX1_TX_FMT_I2S;
		break;
	case SND_SOC_DAIFMT_LSB:
		i2s_mux_reg |= LPC315X_CODEC_I2SMUX1_TX_FMT_LSB_16;
		break;
	}

	/* DATAI is slave only, so this has to be slave */
	if ((fmt & SND_SOC_DAIFMT_MASTER_MASK) != SND_SOC_DAIFMT_CBS_CFS)
		return -EINVAL;

	lpc315x_codec_write(codec, LPC315X_CODEC_I2SMUX1, i2s_mux_reg);

	return 0;
}

/*
 * LPC315X CODEC trigger function
 * */
static int lpc315x_codec_trigger(struct snd_pcm_substream *substream,
		int cmd, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	struct lpc315x_codec_priv *prv = codec->private_data;
	u32 int0_reg = lpc315x_codec_read_reg_cache(codec,
			LPC315X_CODEC_INT0);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		lpc315x_codec_write_reg_cache(codec, LPC315X_CODEC_INT0,
			int0_reg & ~LPC315X_CODEC_INT0_SET_SIL);
		schedule_work(&prv->work);
		break;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		lpc315x_codec_write_reg_cache(codec, LPC315X_CODEC_INT0,
				int0_reg | LPC315X_CODEC_INT0_SET_SIL);
		schedule_work(&prv->work);
		break;
	}

	return 0;
}

/*
 * LPC315X CODEC HW parameters function
 * */
static int lpc315x_codec_pcm_hw_params(struct snd_pcm_substream *substream,
		struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	unsigned int cgu_reg = 0;
	u32 aout_reg, ain_reg, int0_reg, dec_reg;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Playback power up sequence */
		/* Power up DAC Clocks */
		lpc315x_codec_register_rw(codec, LPC315X_CODEC_CGU,
				&cgu_reg, 1);
		cgu_reg &= ~(LPC315X_CODEC_CGU_PD_DSP_CLK |
				LPC315X_CODEC_CGU_PD_NS_CLK |
				LPC315X_CODEC_CGU_PD_I2SRX_BCLK |
				LPC315X_CODEC_CGU_PD_I2SRX_SYSCLK |
				LPC315X_CODEC_CGU_PD_I2C256FS_CLK |
				LPC315X_CODEC_CGU_PD_DAC_CLK);
		lpc315x_codec_register_rw(codec, LPC315X_CODEC_CGU,
				&cgu_reg, 0);
		/* Power up HP */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg &= ~(LPC315X_CODEC_AOUT_PD_HP_L |
			LPC315X_CODEC_AOUT_PD_HP_C |
			LPC315X_CODEC_AOUT_PD_HP_R);
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);

		/* Power up Reference buffer */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg &= ~(LPC315X_CODEC_AOUT_VREF_SLOW);
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);

		/* Power up Central Reference source */
		ain_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AIN1);
		ain_reg &= ~(LPC315X_CODEC_AIN1_PD_VCOM_VREF1);
		lpc315x_codec_write(codec, LPC315X_CODEC_AIN1, ain_reg);

		/* Power down INT DAC */
		int0_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_INT0);
		int0_reg |= (LPC315X_CODEC_INT0_PD_DAC);
		lpc315x_codec_write(codec, LPC315X_CODEC_INT0, int0_reg);
	}
	else {
		/* Recording power up sequence */
		/* Switch on ADC, Decimator clocks */
		lpc315x_codec_register_rw(codec, LPC315X_CODEC_CGU,
				&cgu_reg, 1);
		cgu_reg &= ~(LPC315X_CODEC_CGU_PD_DEC_CLK |
				LPC315X_CODEC_CGU_PD_I2STX_BCLK |
				LPC315X_CODEC_CGU_PD_I2STX_SYSCLK |
				LPC315X_CODEC_CGU_PD_I2C256FS_CLK |
				LPC315X_CODEC_CGU_PD_ADCSYS_CLK |
				LPC315X_CODEC_CGU_PD_ADC2_CLK |
				LPC315X_CODEC_CGU_PD_ADC1_CLK);
		lpc315x_codec_register_rw(codec, LPC315X_CODEC_CGU,
				&cgu_reg, 0);
		/* Mute */
		dec_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_DEC);
		dec_reg |= (LPC315X_CODEC_DEC_MUTE);
		dec_reg &= ~(LPC315X_CODEC_DEC_AGC_EN);
		lpc315x_codec_write(codec, LPC315X_CODEC_DEC, dec_reg);

		/* Power up Reference buffer */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg &= ~(LPC315X_CODEC_AOUT_VREF_SLOW);
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);

		/* Power up Central Reference source */
		ain_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AIN1);
		ain_reg &= ~(LPC315X_CODEC_AIN1_PD_VCOM_VREF1 |
				LPC315X_CODEC_AIN1_PD_BIAS);
		lpc315x_codec_write(codec, LPC315X_CODEC_AIN1, ain_reg);

		/* Ramp up Reference voltage */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg |= LPC315X_CODEC_AOUT_VREF_SLOW_UP;
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);
		mdelay(500);

		/* Enable Decimator & set Volume */
		dec_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_DEC);
		dec_reg &= ~(LPC315X_CODEC_DEC_GAIN_R_MASK |
				LPC315X_CODEC_DEC_GAIN_L_MASK |
				LPC315X_CODEC_DEC_MUTE);
		dec_reg |= ((DEC_GAIN << LPC315X_CODEC_DEC_GAIN_R_POS) |
				(DEC_GAIN << LPC315X_CODEC_DEC_GAIN_L_POS) |
				LPC315X_CODEC_DEC_DC_IN_FIL |
				LPC315X_CODEC_DEC_DC_OUT_FIL);
		lpc315x_codec_write(codec, LPC315X_CODEC_DEC, dec_reg);
	}

	return 0;
}

/*
 * LPC315X CODEC PCM shutdown function
 * */
static void lpc315x_codec_pcm_shutdown(struct snd_pcm_substream *substream,
				 struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_device *socdev = rtd->socdev;
	struct snd_soc_codec *codec = socdev->card->codec;
	unsigned int cgu_reg = 0;
	u32 aout_reg, dec_reg, int0_reg, ain_reg, int1_reg;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		/* Set Interpolator to mute */
		int1_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_INT1);
		int1_reg |= (LPC315X_CODEC_INT1_MAS_MUTE);
		lpc315x_codec_write(codec, LPC315X_CODEC_INT1, int1_reg);

		/* Set AVC to mute  */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg &= ~(LPC315X_CODEC_AOUT_AVC_COA_GAIN_MSK |
				LPC315X_CODEC_AOUT_AVC_FIN_GAIN_MSK);
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);

		/* Playback power down sequence */
		/* Ramp down Reference Volatge */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg &= ~(LPC315X_CODEC_AOUT_VREF_SLOW_UP);
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);
		mdelay(500);

		/* Ramp down INT DAC volatge */
		int0_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_INT0);
		int0_reg |= (LPC315X_CODEC_INT0_PD_DAC);
		lpc315x_codec_write(codec, LPC315X_CODEC_INT0, int0_reg);
		mdelay(30);

		/* Disconnect DAC outputs to HP */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg &= ~(LPC315X_CODEC_AOUT_SWDAC_ON);
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);

	}
	else {
		/* Mute */
		dec_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_DEC);
		dec_reg |= (LPC315X_CODEC_DEC_MUTE);
		dec_reg &= ~(LPC315X_CODEC_DEC_AGC_EN);
		lpc315x_codec_write(codec, LPC315X_CODEC_DEC, dec_reg);

		/* Recording power down sequence */
		/* Ramp down Reference voltage */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg &= ~(LPC315X_CODEC_AOUT_VREF_SLOW_UP);
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);
		mdelay(500);

		/* Power up Central Reference source */
		ain_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AIN1);
		ain_reg |= (LPC315X_CODEC_AIN1_PD_VCOM_VREF1);
		lpc315x_codec_write(codec, LPC315X_CODEC_AIN1, ain_reg);

		/* Power down Reference buffer */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg |= (LPC315X_CODEC_AOUT_VREF_SLOW);
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);

		/* Power down AVC */
		aout_reg = lpc315x_codec_read_reg_cache(codec,
				LPC315X_CODEC_AOUT);
		aout_reg |= (LPC315X_CODEC_AOUT_PD_ANVC_L |
			LPC315X_CODEC_AOUT_PD_ANVC_R);
		lpc315x_codec_write(codec, LPC315X_CODEC_AOUT, aout_reg);

		/* Switch off ADC, Decimator clocks */
		lpc315x_codec_register_rw(codec, LPC315X_CODEC_CGU,
				&cgu_reg, 1);
		cgu_reg &= ~(LPC315X_CODEC_CGU_PD_DEC_CLK |
				LPC315X_CODEC_CGU_PD_I2SRX_BCLK |
				LPC315X_CODEC_CGU_PD_I2SRX_SYSCLK |
				LPC315X_CODEC_CGU_PD_I2C256FS_CLK |
				LPC315X_CODEC_CGU_PD_ADCSYS_CLK |
				LPC315X_CODEC_CGU_PD_ADC2_CLK |
				LPC315X_CODEC_CGU_PD_ADC1_CLK);
		lpc315x_codec_register_rw(codec, LPC315X_CODEC_CGU,
				&cgu_reg, 0);
	}

	return;
}

/*
 * LPC315X CODEC Mute function
 * */
static int lpc315x_codec_mute(struct snd_soc_dai *codec_dai,
		int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	u32 int1_reg = lpc315x_codec_read_reg_cache(codec, LPC315X_CODEC_INT1)
		& ~LPC315X_CODEC_INT1_MAS_MUTE;

	/* Mute */
	if (mute) {
		lpc315x_codec_write(codec, LPC315X_CODEC_INT1,
				int1_reg | LPC315X_CODEC_INT1_MAS_MUTE);
	}
	else {
		lpc315x_codec_write(codec, LPC315X_CODEC_INT1, int1_reg);
	}

	return 0;
}

/*
 * LPC315X CODEC BIAS control function
 * */
static int lpc315x_codec_set_bias_level(struct snd_soc_codec *codec,
	enum snd_soc_bias_level level)
{
	u32 ain1_reg = lpc315x_codec_read_reg_cache(codec,
			LPC315X_CODEC_AIN1);

	switch (level) {
	case SND_SOC_BIAS_ON:
	case SND_SOC_BIAS_PREPARE:
	case SND_SOC_BIAS_STANDBY:
		ain1_reg &= ~LPC315X_CODEC_AIN1_PD_BIAS;
		lpc315x_codec_write(codec, LPC315X_CODEC_AIN1, ain1_reg);
		break;
	case SND_SOC_BIAS_OFF:
		lpc315x_codec_write(codec, LPC315X_CODEC_AIN1,
				LPC315X_CODEC_AIN1_BIAS_OFF);
		break;
	}

	codec->bias_level = level;
	return 0;
}

/*
 * LPC315X CODEC supported data rates
 * */
#define LPC315X_CODEC_RATES (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 |\
		       SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 |\
		       SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000)

/*
 * DAI operations for both Playback & Capture
 * */
static struct snd_soc_dai_ops lpc315x_codec_ops = {
	.hw_params	= lpc315x_codec_pcm_hw_params,
	.shutdown	= lpc315x_codec_pcm_shutdown,
	.trigger	= lpc315x_codec_trigger,
	.digital_mute = lpc315x_codec_mute,
	.set_fmt	= lpc315x_codec_set_dai_fmt_both,
};

/*
 * DAI operations for Playback
 * */
static struct snd_soc_dai_ops lpc315x_codec_ops_playback = {
	.hw_params	= lpc315x_codec_pcm_hw_params,
	.shutdown	= lpc315x_codec_pcm_shutdown,
	.trigger	= lpc315x_codec_trigger,
	.digital_mute = lpc315x_codec_mute,
	.set_fmt	= lpc315x_codec_set_dai_fmt_playback,
};

/*
 * DAI operations for Capture
 * */
static struct snd_soc_dai_ops lpc315x_codec_ops_capture = {
	.hw_params	= lpc315x_codec_pcm_hw_params,
	.shutdown	= lpc315x_codec_pcm_shutdown,
	.trigger	= lpc315x_codec_trigger,
	.digital_mute = lpc315x_codec_mute,
	.set_fmt	= lpc315x_codec_set_dai_fmt_capture,
};

/*
 * LPC315X CODEC DAI
 * */
struct snd_soc_dai lpc315x_codec_dais[] = {
{
	.name = "LPC315X_CODEC",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = LPC315X_CODEC_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = LPC315X_CODEC_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &lpc315x_codec_ops,
},
{ /* playback only - dual interface */
	.name = "LPC315X_CODEC",
	.playback = {
		.stream_name = "Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = LPC315X_CODEC_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &lpc315x_codec_ops_playback,
},
{ /* capture only - dual interface*/
	.name = "LPC315X_CODEC",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = LPC315X_CODEC_RATES,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.ops = &lpc315x_codec_ops_capture,
},
};
EXPORT_SYMBOL_GPL(lpc315x_codec_dais);

/*
 * LPC315X CODEC suspend function
 * */
static int lpc315x_codec_suspend(struct platform_device *pdev,
		pm_message_t state)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	/* Set to OFF */
	lpc315x_codec_set_bias_level(codec, SND_SOC_BIAS_OFF);

	return 0;
}

/*
 * LPC315X CODEC resume function
 * */
static int lpc315x_codec_resume(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;
	u8 data[6];
	u32 i, *cache = codec->reg_cache;

	/* Sync reg_cache with the hardware */
	for (i = 0; i < ARRAY_SIZE(lpc315x_codec_regs); i++) {
		data[0] = ((i >> 8) & 0xFF);
		data[1] = (i & 0xFF);
		data[2] = ((cache[i]  >> 24) & 0xFF);
		data[3] = ((cache[i] >> 16) & 0xFF);
		data[4] = ((cache[i] >> 8) & 0xFF);
		data[5] = (cache[i] & 0xFF);
		codec->hw_write(codec->control_data, data, 6);
	}

	/* Set the BIAS levels */
	lpc315x_codec_set_bias_level(codec, SND_SOC_BIAS_STANDBY);
	lpc315x_codec_set_bias_level(codec, codec->suspend_bias_level);

	return 0;
}

/*
 * LPC315X CODEC probe function
 * */
static int lpc315x_codec_probe(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec;
	int ret = 0;
	u32 cgu_reg = 0;

	/* Check if LPC315X CODECX registered */
	if (lpc315x_codec == NULL) {
		dev_err(&pdev->dev, "Codec device not registered\n");
		return -ENODEV;
	}

	socdev->card->codec = lpc315x_codec;
	codec = lpc315x_codec;

	/* Register pcms */
	ret = snd_soc_new_pcms(socdev, SNDRV_DEFAULT_IDX1,
			SNDRV_DEFAULT_STR1);
	if (ret < 0) {
		dev_err(codec->dev, "failed to create pcms: %d\n", ret);
		goto pcm_err;
	}

	/* Select ADC & DAC clocks */
	lpc315x_codec_register_rw(codec, LPC315X_CODEC_CGU,
			&cgu_reg, 1);
	cgu_reg &= ~(LPC315X_CODEC_CGU_PD_I2C256FS_CLK |
			LPC315X_CODEC_CGU_ADCSYS_256FS |
			LPC315X_CODEC_CGU_ADC2_128FS |
			LPC315X_CODEC_CGU_ADC1_OFF |
			LPC315X_CODEC_CGU_CLKDAC_256FS |
			LPC315X_CODEC_CGU_CLKINT_256FS);
	/* DAC Clock not inverted */
	lpc315x_codec_register_rw(codec, LPC315X_CODEC_CGU, &cgu_reg, 0);

	/* power on device */
	lpc315x_codec_set_bias_level(codec, SND_SOC_BIAS_STANDBY);

	/* Add LPC315X CODEC controls */
	snd_soc_add_controls(codec, lpc315x_codec_snd_ctrls,
				ARRAY_SIZE(lpc315x_codec_snd_ctrls));

	/* Add Widgets & Audio map */
	lpc315x_codec_add_widgets(codec);
pcm_err:
	return ret;
}

/*
 * LPC315X CODEC remove function
 * */
static int lpc315x_codec_remove(struct platform_device *pdev)
{
	struct snd_soc_device *socdev = platform_get_drvdata(pdev);
	struct snd_soc_codec *codec = socdev->card->codec;

	/* Set BIAS level to off */
	if (codec->control_data)
		lpc315x_codec_set_bias_level(codec, SND_SOC_BIAS_OFF);

	/* Unregister PCMs */
	snd_soc_free_pcms(socdev);

	/* Remove DAPM widgets */
	snd_soc_dapm_free(socdev);

	return 0;
}

/*
 * CODEC device
 * */
struct snd_soc_codec_device soc_codec_lpc315x_dev = {
	.probe = 	lpc315x_codec_probe,
	.remove = 	lpc315x_codec_remove,
	.suspend = 	lpc315x_codec_suspend,
	.resume =	lpc315x_codec_resume,
};
EXPORT_SYMBOL_GPL(soc_codec_lpc315x_dev);

/*
 * LPC315X CODEC register function
 * */
static int lpc315x_codec_register(struct lpc315x_codec_priv *codec_prv)
{
	int ret = 0, i;
	struct snd_soc_codec *codec = &codec_prv->codec;

	/* Check if LPC315X CODEC registered */
	if (lpc315x_codec) {
		dev_err(codec->dev, "Another LPC315X CODEC is registered\n");
		return -EINVAL;
	}

	/* Initialise CODEC structure */
	mutex_init(&codec->mutex);
	INIT_LIST_HEAD(&codec->dapm_widgets);
	INIT_LIST_HEAD(&codec->dapm_paths);
	codec->private_data = codec_prv;
	codec->name = "LPC315X_CODEC";
	codec->owner = THIS_MODULE;
	codec->read = lpc315x_codec_read_reg_cache;
	codec->write = lpc315x_codec_write;
	codec->bias_level = SND_SOC_BIAS_OFF;
	codec->set_bias_level = lpc315x_codec_set_bias_level;
	codec->dai = lpc315x_codec_dais;
	codec->num_dai = ARRAY_SIZE(lpc315x_codec_dais);
	codec->reg_cache_size = ARRAY_SIZE(lpc315x_codec_regs);
	codec->reg_cache = &codec_prv->lpc315x_reg_cache;
	codec->reg_cache_step = 1;

	/* Copy register contents */
	memcpy(codec->reg_cache, lpc315x_codec_regs,
			sizeof(lpc315x_codec_regs));

	/* Initialise work queue */
	INIT_WORK(&codec_prv->work, lpc315x_codec_work);

	/* Store devier structure */
	for (i = 0; i < ARRAY_SIZE(lpc315x_codec_dais); i++)
		lpc315x_codec_dais[i].dev = codec->dev;

	/* Initialise LPC315X CODEC structure */
	lpc315x_codec = codec;

	/* Register CODEC */
	ret = snd_soc_register_codec(codec);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register codec: %d\n", ret);
		goto err_reset;
	}

	/* Register CODEC DAIs */
	ret = snd_soc_register_dais(lpc315x_codec_dais,
			ARRAY_SIZE(lpc315x_codec_dais));
	if (ret != 0) {
		dev_err(codec->dev, "Failed to register DAIs: %d\n", ret);
		goto err_dai;
	}

	return 0;

err_dai:
	snd_soc_unregister_codec(codec);
err_reset:
	return ret;
}

/*
 * LPC315X CODEC unregister function
 * */
static void lpc315x_codec_unregister(struct lpc315x_codec_priv
		    *codec_prv)
{
	/* Unregister CODEC DAIs */
	snd_soc_unregister_dais(lpc315x_codec_dais,
			ARRAY_SIZE(lpc315x_codec_dais));

	/* Unregister CODEC */
	snd_soc_unregister_codec(&codec_prv->codec);

	/* Free memory */
	kfree(codec_prv);

	/* Invalidate CODEC structure pointer */
	lpc315x_codec = NULL;
}

/*
 * LPC315X CODEC driver module initialisation function
 * */
static int __init lpc315x_codec_init(void)
{
	struct i2c_client *i2c;
	struct snd_soc_codec *codec;
	int ret = 0;

	printk(KERN_INFO "CODEC I2C Probe called \r\n");

	/* Get I2C client structure structure */
	i2c = lpc315x_ad_get_i2c_client_struct();
	if (!i2c) {
		pr_err("Unable to get I2C client \n");
		return -ENODEV;
	}

	/* Allocate memory for LPC315X CODEC private structure */
	lpc315x_codec_prv = kzalloc(sizeof(struct lpc315x_codec_priv),
			GFP_KERNEL);
	if (lpc315x_codec_prv == NULL) {
		pr_err("Failed to allocate memory \n");
		return -ENOMEM;
	}

	/* Initialise CODEC structure */
	codec = &lpc315x_codec_prv->codec;
	codec->hw_write = (hw_write_t) i2c_master_send;
	codec->control_data = i2c;
	codec->dev = &i2c->dev;

	/* Register LPC315X CODEC */
	ret = lpc315x_codec_register(lpc315x_codec_prv);
	if (ret != 0) {
		pr_err("Failed to register LPC315X CODEC \n");
		kfree(lpc315x_codec_prv);
	}

	return ret;
}

/*
 * LPC315X CODEC driver module exit function
 * */
static void __exit lpc315x_codec_exit(void)
{
	/* Unregister LPC315X CODEC */
	lpc315x_codec_unregister(lpc315x_codec_prv);

	return;
}

module_init(lpc315x_codec_init);
module_exit(lpc315x_codec_exit);

MODULE_AUTHOR("Kevin Wells <kevin.wells@nxp.com>");
MODULE_DESCRIPTION("ASoC Audio CODEC driver for LPC315X codec");
MODULE_LICENSE("GPL");
