/*
 * Texas Instruments TAS5504 low power audio CODEC
 * ALSA SoC CODEC driver
 *
 * Copyright (C) Jon Smirl <jonsmirl@gmail.com>
 */

#define TAS_REG_CLOCK_CONTROL    0x0000
#define TAS_REG_GENERAL_STATUS   0x0001
#define TAS_REG_ERROR_STATUS     0x0002
#define TAS_REG_SYS_CONTROL_1    0x0003
#define TAS_REG_SYS_CONTROL_2    0x0004
#define TAS_REG_CH_CONFIG_1      0x0005
#define TAS_REG_CH_CONFIG_2      0x0006
#define TAS_REG_CH_CONFIG_3      0x000b
#define TAS_REG_CH_CONFIG_4      0x000c
#define TAS_REG_HP_CONFIG        0x000d
#define TAS_REG_SERIAL_CONTROL   0x000e
#define TAS_REG_SOFT_MUTE        0x000f
#define TAS_REG_AUTO_MUTE        0x0014
#define TAS_REG_AUTO_MUTE_PWM    0x0015
#define TAS_REG_MODULATE_LIMIT   0x0016
#define TAS_REG_IC_DELAY_1       0x001b
#define TAS_REG_IC_DELAY_2       0x001c
#define TAS_REG_IC_DELAY_3       0x0021
#define TAS_REG_IC_DELAY_4       0x0022
#define TAS_REG_IC_OFFSET        0x0023

#define TAS_REG_BANK_SWITCH      0x0040
#define TAS_REG_IN8X4_1          0x0041
#define TAS_REG_IN8X4_2          0x0042
#define TAS_REG_IN8X4_3          0x0047
#define TAS_REG_IN8X4_4          0x0048
#define TAS_REG_IPMIX_1_TO_CH4   0x0049
#define TAS_REG_IPMIX_2_TO_CH4   0x004a
#define TAS_REG_IPMIX_3_TO_CH2   0x004b
#define TAS_REG_CH3_BP_BQ2       0x004c
#define TAS_REG_CH3_BP           0x004d
#define TAS_REG_IPMIX_4_TO_CH12  0x004e
#define TAS_REG_CH4_BP_BQ2       0x004f
#define TAS_REG_CH4_BP           0x0050

#define TAS_REG_CH1_BQ_1         0x0051
#define TAS_REG_CH1_BQ_2         0x0052
#define TAS_REG_CH1_BQ_3         0x0053
#define TAS_REG_CH1_BQ_4         0x0054
#define TAS_REG_CH1_BQ_5         0x0055
#define TAS_REG_CH1_BQ_6         0x0056
#define TAS_REG_CH1_BQ_7         0x0057
#define TAS_REG_CH2_BQ_1         0x0058
#define TAS_REG_CH2_BQ_2         0x0059
#define TAS_REG_CH2_BQ_3         0x005a
#define TAS_REG_CH2_BQ_4         0x005b
#define TAS_REG_CH2_BQ_5         0x005c
#define TAS_REG_CH2_BQ_6         0x005d
#define TAS_REG_CH2_BQ_7         0x005e
#define TAS_REG_CH3_BQ_1         0x007b
#define TAS_REG_CH3_BQ_2         0x007c
#define TAS_REG_CH3_BQ_3         0x007d
#define TAS_REG_CH3_BQ_4         0x007e
#define TAS_REG_CH3_BQ_5         0x007f
#define TAS_REG_CH3_BQ_6         0x0080
#define TAS_REG_CH3_BQ_7         0x0081
#define TAS_REG_CH4_BQ_1         0x0082
#define TAS_REG_CH4_BQ_2         0x0083
#define TAS_REG_CH4_BQ_3         0x0084
#define TAS_REG_CH4_BQ_4         0x0085
#define TAS_REG_CH4_BQ_5         0x0086
#define TAS_REG_CH4_BQ_6         0x0087
#define TAS_REG_CH4_BQ_7         0x0088

#define TAS_REG_BT_BYPASS_CH1    0x0089
#define TAS_REG_BT_INLINE_CH1    0x0189
#define TAS_REG_BT_BYPASS_CH2    0x008a
#define TAS_REG_BT_INLINE_CH2    0x018a
#define TAS_REG_BT_BYPASS_CH3    0x008f
#define TAS_REG_BT_INLINE_CH3    0x018f
#define TAS_REG_BT_BYPASS_CH4    0x0090
#define TAS_REG_BT_INLINE_CH4    0x0190
#define TAS_REG_LOUDNESS_LG      0x0091
#define TAS_REG_LOUDNESS_LO_U    0x0092
#define TAS_REG_LOUDNESS_LO_L    0x0192
#define TAS_REG_LOUDNESS_G       0x0093
#define TAS_REG_LOUDNESS_O_U     0x0094
#define TAS_REG_LOUDNESS_O_L     0x0194
#define TAS_REG_LOUDNESS_BQ_B0   0x0095
#define TAS_REG_LOUDNESS_BQ_B1   0x0195
#define TAS_REG_LOUDNESS_BQ_B2   0x0295
#define TAS_REG_LOUDNESS_BQ_A0   0x0395
#define TAS_REG_LOUDNESS_BQ_A1   0x0495
#define TAS_REG_DRC1_CNTL_123    0x0096
#define TAS_REG_DRC2_CNTL_4      0x0097
#define TAS_REG_DRC1_ENERGY      0x0098
#define TAS_REG_DRC1_ENERGY_1E   0x0198
#define TAS_REG_DRC1_THRESH_T1_U 0x0099
#define TAS_REG_DRC1_THRESH_T1_L 0x0199
#define TAS_REG_DRC1_THRESH_T2_U 0x0299
#define TAS_REG_DRC1_THRESH_T2_L 0x0399
#define TAS_REG_DRC1_SLOPE_K0    0x009a
#define TAS_REG_DRC1_SLOPE_K1    0x019a
#define TAS_REG_DRC1_SLOPE_K2    0x029a
#define TAS_REG_DRC1_OFFSET_O1_U 0x009b
#define TAS_REG_DRC1_OFFSET_O1_L 0x019b
#define TAS_REG_DRC1_OFFSET_O2_U 0x029b
#define TAS_REG_DRC1_OFFSET_O2_L 0x039b
#define TAS_REG_DRC1_ATTACK      0x009c
#define TAS_REG_DRC1_ATTACK_1A   0x019c
#define TAS_REG_DRC1_DECAY       0x029c
#define TAS_REG_DRC1_DECAY_1D    0x039c
#define TAS_REG_DRC2_ENERGY      0x009d
#define TAS_REG_DRC2_ENERGY_1E   0x019d
#define TAS_REG_DRC2_THRESH_T1_U 0x019e
#define TAS_REG_DRC2_THRESH_T1_L 0x029e
#define TAS_REG_DRC2_THRESH_T2_U 0x039e
#define TAS_REG_DRC2_THRESH_T2_L 0x049e
#define TAS_REG_DRC2_SLOPE_K0    0x009f
#define TAS_REG_DRC2_SLOPE_K1    0x019f
#define TAS_REG_DRC2_SLOPE_K2    0x029f
#define TAS_REG_DRC2_OFFSET_O1_U 0x00a0
#define TAS_REG_DRC2_OFFSET_O1_L 0x01a0
#define TAS_REG_DRC2_OFFSET_O2_U 0x02a0
#define TAS_REG_DRC2_OFFSET_O2_L 0x03a0
#define TAS_REG_DRC2_ATTACK      0x00a1
#define TAS_REG_DRC2_ATTACK_1A   0x01a1
#define TAS_REG_DRC2_DECAY       0x02a1
#define TAS_REG_DRC2_DECAY_1D    0x03a1
#define TAS_REG_DRC_BYPASS_1     0x00a2
#define TAS_REG_DRC_INLINE_1     0x01a2
#define TAS_REG_DRC_BYPASS_2     0x00a3
#define TAS_REG_DRC_INLINE_2     0x01a3
#define TAS_REG_DRC_BYPASS_3     0x00a8
#define TAS_REG_DRC_INLINE_3     0x01a8
#define TAS_REG_DRC_BYPASS_4     0x00a9
#define TAS_REG_DRC_INLINE_4     0x01a9
#define TAS_REG_SEL_OP14_S       0x00aa
#define TAS_REG_SEL_OP14_S_G     0x01aa
#define TAS_REG_SEL_OP14_T       0x00ab
#define TAS_REG_SEL_OP14_T_G     0x01ab
#define TAS_REG_SEL_OP14_Y       0x00b0
#define TAS_REG_SEL_OP14_Y_G     0x01b0
#define TAS_REG_SEL_OP14_Z       0x00b1
#define TAS_REG_SEL_OP14_Z_G     0x01b1
#define TAS_REG_VOLUME_BQ        0x00cf
#define TAS_REG_VOL_TB_SLEW      0x00d0
#define TAS_REG_VOL_CH1          0x00d1
#define TAS_REG_VOL_CH2          0x00d2
#define TAS_REG_VOL_CH3          0x00d7
#define TAS_REG_VOL_CH4          0x00d8
#define TAS_REG_VOL_MASTER       0x00d9
#define TAS_REG_BASS_SET         0x00da
#define TAS_REG_BASS_INDEX       0x00db
#define TAS_REG_TREBLE_SET       0x00dc
#define TAS_REG_TREBLE_INDEX     0x00dd
#define TAS_REG_AM_MODE          0x00de
#define TAS_REG_PSVC             0x00df
#define TAS_REG_GENERAL_CONTROL  0x00e0

#define TAS_MAX_8B_REG 0x40
#define TAS_REG_MAX 0xe1

#define TAS5504_RATES (SNDRV_PCM_RATE_32000 |\
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
				SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |\
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000)
#define TAS5504_FORMATS SNDRV_PCM_FMTBIT_S32

