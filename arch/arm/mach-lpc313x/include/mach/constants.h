/* linux/arch/arm/mach-lpc313x/include/mach/constants.h
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * Manifest constants for LPC313x and LPC315x SoCs.
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
/* ----- Globals */
#define _BIT(n)	  ((1) << (n))
/* shift bit field */
#define _SBF(f,v) ((v) << (f))


/***********************************************************************
 * Physical bases
 **********************************************************************/
#define EVTR_PHYS         (0x13000000)
#define ADC_PHYS          (0x13002000)
#define WDT_PHYS          (0x13002400)
#define SYS_PHYS          (0x13002800)
#define GPIO_PHYS         (0x13003000)
#define CGU_SB_PHYS       (0x13004000)
#define CGU_CFG_PHYS      (0x13004c00)
#define TIMER0_PHYS       (0x13008000)
#define TIMER1_PHYS       (0x13008400)
#define TIMER2_PHYS       (0x13008800)
#define TIMER3_PHYS       (0x13008c00)
#define I2C0_PHYS         (0x1300a000)
#define I2C1_PHYS         (0x1300a400)
#define UART_PHYS         (0x15001000)
#define SPI_PHYS          (0x15002000)
#define I2S_PHYS          (0x16000000)
#define DMA_PHYS          (0x17000000)
#define MPMC_PHYS         (0x17008000)
#define SDMMC_PHYS        (0x18000000)
#define USBOTG_PHYS       (0x19000000)
#define INTC_PHYS         (0x60000000)
#define NANDC_PHYS        (0x17000800)
/***********************************************************************
 * Memory definitions
 **********************************************************************/
#define EXT_SDRAM_PHYS    (0x30000000)
#define EXT_SRAM0_PHYS    (0x20000000)
#define EXT_SRAM1_PHYS    (0x20020000)
#define ISRAM0_PHYS       (0x11028000)
#define ISRAM0_LENGTH     (0x00018000)
#define ISRAM1_PHYS       (0x11040000)
#define ISRAM1_LENGTH     (0x00018000)

/***********************************************************************
 * XTAL clock definitions
 **********************************************************************/
#define XTAL_CLOCK        (12000000)
#define FFAST_CLOCK       XTAL_CLOCK

/* SoC CPU IO addressing */
/* APB0 & APB1 address range*/
#define IO_APB01_PHYS     (0x13000000)
#define IO_APB01_SIZE     (0x0000B000)
/* APB2 address range*/
#define IO_APB2_PHYS      (0x15000000)
#define IO_APB2_SIZE      (0x00003000)
/* APB3 address range*/
#define IO_APB3_PHYS      (0x16000000)
#define IO_APB3_SIZE      (0x00001000)
/* APB4 address range*/
#define IO_APB4_PHYS      (0x17000000)
#define IO_APB4_SIZE      (0x00001000)
/* DMA registers address range*/
#define IO_DMA_REG_PHYS  (DMA_PHYS)
#define IO_DMA_REG_SIZE  (0x0000800)
/* MPMC config registers address range*/
#define IO_MPMC_CFG_PHYS  (0x17008000)
#define IO_MPMC_CFG_SIZE  (0x00001000)
/* SD/MMC address range*/
#define IO_SDMMC_PHYS     (SDMMC_PHYS)
#define IO_SDMMC_SIZE     (0x00001000)
/* USB OTG address range*/
#define IO_USB_PHYS       (USBOTG_PHYS)
#define IO_USB_SIZE       (0x00001000)
/* Interrupt controller address range*/
#define IO_INTC_PHYS      (0x60000000)
#define IO_INTC_SIZE      (0x00001000)
/* NAND address range*/
#define IO_NAND_PHYS	  (NANDC_PHYS)
#define IO_NAND_SIZE	  (0x00000800)
/* NAND buffer address range*/
#define IO_NAND_BUF_PHYS  (0x70000000)
#define IO_NAND_BUF_SIZE  (0x00001000)
/* ISRAM address range*/
#define IO_ISRAM0_PHYS     (0x11028000)
#define IO_ISRAM0_SIZE     (0x00018000)
