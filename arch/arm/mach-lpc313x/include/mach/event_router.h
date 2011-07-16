/* linux/arch/arm/mach-lpc313x/include/mach/event_router.h
 *  
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * Event router defines for LPC313x and LPC315x SoCs.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#ifndef __ASM_ARCH_EVT_IRQS_H
#define __ASM_ARCH_EVT_IRQS_H


/* event PIN or internal signal */
typedef enum _EVT_TYPE_
{
  EVT_ipint_int = 0,
  EVT_mLCD_DB_0,
  EVT_mLCD_DB_1,
  EVT_mLCD_DB_2,
  EVT_mLCD_DB_3,
  EVT_mLCD_DB_4,
  EVT_mLCD_DB_5,
  EVT_mLCD_DB_6,
  EVT_mLCD_DB_7,
  EVT_mLCD_DB_8,
  EVT_mLCD_DB_9,
  EVT_mLCD_DB_10,
  EVT_mLCD_DB_11,
  EVT_mLCD_DB_12,
  EVT_mLCD_DB_13,
  EVT_mLCD_DB_14,
  EVT_mLCD_DB_15,
  EVT_mLCD_RS,
  EVT_mLCD_CSB,
  EVT_mLCD_E_RD,
  EVT_mLCD_RW_WR,
  EVT_mNAND_RYBN0,
  EVT_mNAND_RYBN1,
  EVT_mNAND_RYBN2,
  EVT_mNAND_RYBN3,
  EVT_EBI_D_0,
  EVT_EBI_D_1,
  EVT_EBI_D_2,
  EVT_EBI_D_3,
  EVT_EBI_D_4,
  EVT_EBI_D_5,
  EVT_EBI_D_6,
  EVT_EBI_D_7,
  EVT_EBI_D_8,
  EVT_EBI_D_9,
  EVT_EBI_D_10,
  EVT_EBI_D_11,
  EVT_EBI_D_12,
  EVT_EBI_D_13,
  EVT_EBI_D_14,
  EVT_EBI_D_15,
  EVT_EBI_NWE,
  EVT_EBI_A_0_ALE,
  EVT_EBI_A_1_CLE,
  EVT_EBI_DQM_0_NOE,
  EVT_EBI_NCAS_BLOUT_0,
  EVT_EBI_NRAS_BLOUT_1,
  EVT_GPIO1,
  EVT_GPIO0,
  EVT_GPIO2,
  EVT_GPIO3,
  EVT_GPIO4,
  EVT_mGPIO5,
  EVT_mGPIO6,
  EVT_mGPIO7,
  EVT_mGPIO8,
  EVT_mGPIO9,
  EVT_mGPIO10,
  EVT_GPIO11,
  EVT_GPIO12,
  EVT_GPIO13,
  EVT_GPIO14,
  EVT_GPIO15,
  EVT_GPIO16,
  EVT_GPIO17,
  EVT_GPIO18,
  EVT_NAND_NCS_0,
  EVT_NAND_NCS_1,
  EVT_NAND_NCS_2,
  EVT_NAND_NCS_3,
  EVT_SPI_MISO,
  EVT_SPI_MOSI,
  EVT_SPI_CS_IN,
  EVT_SPI_SCK,
  EVT_SPI_CS_OUT0,
  EVT_UART_RXD,
  EVT_UART_TXD,
  EVT_mUART_CTS_N,
  EVT_mUART_RTS_N,
  EVT_mI2STX_CLK0,
  EVT_mI2STX_BCK0,
  EVT_mI2STX_DATA0,
  EVT_mI2STX_WS0,
  EVT_I2SRX_BCK0,
  EVT_I2SRX_DATA0,
  EVT_I2SRX_WS0,
  EVT_I2SRX_DATA1,
  EVT_I2SRX_BCK1,
  EVT_I2SRX_WS1,
  EVT_I2STX_DATA1,
  EVT_I2STX_BCK1,
  EVT_I2STX_WS1,
  EVT_CLK_256FS_O,
  EVT_I2C_SDA1,
  EVT_I2C_SCL1,
  EVT_PWM_DATA,
  EVT_AD_NINT_I,
  EVT_PLAY_DET_I,
  EVT_timer0_intct1,
  EVT_timer1_intct1,
  EVT_timer2_intct1,
  EVT_timer3_intct1,
  EVT_adc_int,
  EVT_wdog_m0,
  EVT_uart_rxd,
  EVT_i2c0_scl_n,
  EVT_i2c1_scl_n,
  EVT_arm926_nfiq,
  EVT_arm926_nirq,
  EVT_MCI_DAT_0,
  EVT_MCI_DAT_1,
  EVT_MCI_DAT_2,
  EVT_MCI_DAT_3,
  EVT_MCI_DAT_4,
  EVT_MCI_DAT_5,
  EVT_MCI_DAT_6,
  EVT_MCI_DAT_7,
  EVT_MCI_CMD,
  EVT_MCI_CLK,
  EVT_USB_VBUS,
  EVT_usb_otg_ahb_needclk,
  EVT_usb_atx_pll_lock,
  EVT_usb_otg_vbus_pwr_en,
  EVT_USB_ID,
  EVT_isram0_mrc_finished,
  EVT_isram1_mrc_finished,
  EVT_LAST
} EVENT_T;

/* External interrupt type enumerations */
typedef enum
{
  EVT_ACTIVE_LOW,
  EVT_ACTIVE_HIGH,
  EVT_FALLING_EDGE,
  EVT_RISING_EDGE,
  EVT_BOTH_EDGE
} EVENT_TYPE_T;

/* Macros to compute the bank based on EVENT_T */
#define EVT_GET_BANK(evt)   (((evt) >> 5) & 0x3)

/* structure to map board IRQ to event pin */
typedef struct {
	u32 irq;
	EVENT_T event_pin;
	EVENT_TYPE_T type;
} IRQ_EVENT_MAP_T;

#define EVT_MAX_VALID_BANKS   4
#define EVT_MAX_VALID_INT_OUT 5

/* Activation polarity register defines */
#define EVT_APR_HIGH    1
#define EVT_APR_LOW     0
#define EVT_APR_BANK0_DEF 0x00000001
#define EVT_APR_BANK1_DEF 0x00000000
#define EVT_APR_BANK2_DEF 0x00000000
#define EVT_APR_BANK3_DEF 0x0FFFFFFC

/* Activation type register defines */
#define EVT_ATR_EDGE    1
#define EVT_ATR_LEVEL   0
#define EVT_ATR_BANK0_DEF 0x00000001
#define EVT_ATR_BANK1_DEF 0x00000000
#define EVT_ATR_BANK2_DEF 0x00000000
#define EVT_ATR_BANK3_DEF 0x077FFFFC


#endif
