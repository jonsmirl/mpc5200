/* linux/arch/arm/mach-lpc313x/include/mach/registers.h
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * Register defines for LPC313x and LPC315x SoCs.
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

#ifndef __ASM_ARCH_REGISTERS_H
#define __ASM_ARCH_REGISTERS_H


/***********************************************************************
 * Interrupt controller register definitions
 **********************************************************************/
#define INTC_IRQ_PRI_MASK     __REG(INTC_PHYS + 0x000)
#define INTC_FIQ_PRI_MASK     __REG(INTC_PHYS + 0x004)
#define INTC_IRQ_VEC_BASE     __REG(INTC_PHYS + 0x100)
#define INTC_FIQ_VEC_BASE     __REG(INTC_PHYS + 0x104)
#define INTC_REQ_REG(irq)     __REG(INTC_PHYS + 0x400 + ((irq) << 2))

#define INTC_REQ_PEND         _BIT(31)
#define INTC_REQ_SET_SWINT    _BIT(30)
#define INTC_REQ_CLR_SWINT    _BIT(29)
#define INTC_REQ_WE_PRIO_LVL  _BIT(28)
#define INTC_REQ_WE_TARGET    _BIT(27)
#define INTC_REQ_WE_ENABLE    _BIT(26)
#define INTC_REQ_WE_ACT_LOW   _BIT(25)
#define INTC_REQ_ACT_LOW      _BIT(17)
#define INTC_REQ_ENABLE       _BIT(16)
#define INTC_REQ_TARGET(n)    _SBF(8, ((n) & 0x3F))
#define INTC_REQ_PRIO_LVL(n)  ((n) & 0xFF)
#define INTC_REQ_TARGET_IRQ   (INTC_REQ_WE_TARGET)
#define INTC_REQ_TARGET_FIQ   (INTC_REQ_WE_TARGET | _BIT(8))

/***********************************************************************
 * Event router register definitions
 **********************************************************************/
#define EVRT_INT_PEND(bank)  __REG (EVTR_PHYS + 0xC00 + ((bank) << 2))
#define EVRT_INT_CLR(bank)   __REG (EVTR_PHYS + 0xC20 + ((bank) << 2))
#define EVRT_INT_SET(bank)   __REG (EVTR_PHYS + 0xC40 + ((bank) << 2))
#define EVRT_MASK(bank)      __REG (EVTR_PHYS + 0xC60 + ((bank) << 2))
#define EVRT_MASK_CLR(bank)  __REG (EVTR_PHYS + 0xC80 + ((bank) << 2))
#define EVRT_MASK_SET(bank)  __REG (EVTR_PHYS + 0xCA0 + ((bank) << 2))
#define EVRT_APR(bank)       __REG (EVTR_PHYS + 0xCC0 + ((bank) << 2))
#define EVRT_ATR(bank)       __REG (EVTR_PHYS + 0xCE0 + ((bank) << 2))
#define EVRT_RSR(bank)       __REG (EVTR_PHYS + 0xD20 + ((bank) << 2))
#define EVRT_OUT_PEND(vec,bank)     __REG (EVTR_PHYS + 0x1000 + ((vec) << 5) + ((bank) << 2))
#define EVRT_OUT_MASK(vec,bank)     __REG (EVTR_PHYS + 0x1400 + ((vec) << 5) + ((bank) << 2))
#define EVRT_OUT_MASK_CLR(vec,bank) __REG (EVTR_PHYS + 0x1800 + ((vec) << 5) + ((bank) << 2))
#define EVRT_OUT_MASK_SET(vec,bank) __REG (EVTR_PHYS + 0x1C00 + ((vec) << 5) + ((bank) << 2))

/***********************************************************************
 * WDT register definitions
 **********************************************************************/
#define WDT_IR       __REG (WDT_PHYS + 0x00)
#define WDT_TCR      __REG (WDT_PHYS + 0x04)
#define WDT_TC       __REG (WDT_PHYS + 0x08)
#define WDT_PR       __REG (WDT_PHYS + 0x0c)
#define WDT_MCR      __REG (WDT_PHYS + 0x14)
#define WDT_MR0      __REG (WDT_PHYS + 0x18)
#define WDT_MR1      __REG (WDT_PHYS + 0x1c)
#define WDT_EMR      __REG (WDT_PHYS + 0x3c)

#define WDT_IR_MR1        _BIT(1)
#define WDT_IR_MR0        _BIT(0)
#define WDT_TCR_CNT_RESET _BIT(1)
#define WDT_TCR_CNT_EN    _BIT(0)
#define WDT_MCR_STOP_MR1  _BIT(5)
#define WDT_MCR_RESET_MR1 _BIT(4)
#define WDT_MCR_INT_MR1   _BIT(3)
#define WDT_MCR_STOP_MR0  _BIT(2)
#define WDT_MCR_RESET_MR0 _BIT(1)
#define WDT_MCR_INT_MR0   _BIT(0)
#define WDT_EMR_CTRL0(n)  _SBF(4,((n) &0x3))
#define WDT_EMR_CTRL1(n)  _SBF(6,((n) &0x3))
#define WDT_EMR_M1        _BIT(1)
#define WDT_EMR_M0        _BIT(0)

/***********************************************************************
 * Timer register definitions
 **********************************************************************/
#define TIMER_LOAD(base)      __REG ((base) + 0x00)
#define TIMER_VALUE(base)     __REG ((base) + 0x04)
#define TIMER_CONTROL(base)   __REG ((base) + 0x08)
#define TIMER_CLEAR(base)     __REG ((base) + 0x0c)

#define TM_CTRL_ENABLE    _BIT(7)
#define TM_CTRL_MODE      _BIT(6)
#define TM_CTRL_PERIODIC  _BIT(6)
#define TM_CTRL_PS1       _SBF(2, 0)
#define TM_CTRL_PS16      _SBF(2, 1)
#define TM_CTRL_PS256     _SBF(2, 2)
#define TM_CTRL_PS_MASK   _SBF(2, 0x3)

/***********************************************************************
 * UART register definitions
 **********************************************************************/
#define UART_DLL_REG      __REG (UART_PHYS + 0x00)
#define UART_FIFO_REG     __REG (UART_PHYS + 0x00)
#define UART_IE_REG       __REG (UART_PHYS + 0x04)
#define UART_DLM_REG      __REG (UART_PHYS + 0x04)
#define UART_IIR_REG      __REG (UART_PHYS + 0x08)
#define UART_FCR_REG      __REG (UART_PHYS + 0x08)
#define UART_LCR_REG      __REG (UART_PHYS + 0x0c)
#define UART_MCR_REG      __REG (UART_PHYS + 0x10)
#define UART_LSR_REG      __REG (UART_PHYS + 0x14)
#define UART_MSR_REG      __REG (UART_PHYS + 0x18)
#define UART_SCR_REG      __REG (UART_PHYS + 0x1c)
#define UART_ACR_REG      __REG (UART_PHYS + 0x20)
#define UART_ICR_REG      __REG (UART_PHYS + 0x24)
#define UART_FDR_REG      __REG (UART_PHYS + 0x28)

/***********************************************************************
 * SPI register definitions
 **********************************************************************/
#define SPI_CONFIG_REG    __REG (SPI_PHYS + 0x00)
#define SPI_SLV_ENAB_REG  __REG (SPI_PHYS + 0x04)
#define SPI_TXF_FLUSH_REG __REG (SPI_PHYS + 0x08)
#define SPI_FIFO_DATA_REG __REG (SPI_PHYS + 0x0C)
#define SPI_NHP_POP_REG   __REG (SPI_PHYS + 0x10)
#define SPI_NHP_MODE_REG  __REG (SPI_PHYS + 0x14)
#define SPI_DMA_SET_REG   __REG (SPI_PHYS + 0x18)
#define SPI_STS_REG       __REG (SPI_PHYS + 0x1C)
#define SPI_HWINFO_REG    __REG (SPI_PHYS + 0x20)
#define SPI_SLV_SET1_REG(slv) __REG (SPI_PHYS + 0x24 + (8 * slv))
#define SPI_SLV_SET2_REG(slv) __REG (SPI_PHYS + 0x28 + (8 * slv))
#define SPI_INT_TRSH_REG  __REG (SPI_PHYS + 0xFD4)
#define SPI_INT_CLRE_REG  __REG (SPI_PHYS + 0xFD8)
#define SPI_INT_SETE_REG  __REG (SPI_PHYS + 0xFDC)
#define SPI_INT_STS_REG   __REG (SPI_PHYS + 0xFE0)
#define SPI_INT_ENAB_REG  __REG (SPI_PHYS + 0xFE4)
#define SPI_INT_CLRS_REG  __REG (SPI_PHYS + 0xFE8)
#define SPI_INT_SETS_REG  __REG (SPI_PHYS + 0xFEC)
#define SPI_MOD_ID_REG    __REG (SPI_PHYS + 0xFFC)

/* SPI device contants */
#define SPI_FIFO_DEPTH  64 /* 64 words (16bit) deep */
#define SPI_NUM_SLAVES  3  /* number of slaves supported */
#define SPI_MAX_DIV2    254
#define SPI_MAX_DIVIDER 65024 /* = 254 * (255 + 1) */
#define SPI_MIN_DIVIDER 2

/* SPI Configuration register definitions (SPI_CONFIG_REG) */
#define SPI_CFG_INTER_DLY(n)      _SBF(16, ((n) & 0xFFFF))
#define SPI_CFG_INTER_DLY_GET(n)  (((n) >> 16) & 0xFFFF)
#define SPI_CFG_UPDATE_EN         _BIT(7)
#define SPI_CFG_SW_RESET          _BIT(6)
#define SPI_CFG_SLAVE_DISABLE     _BIT(4)
#define SPI_CFG_MULTI_SLAVE       _BIT(3)
#define SPI_CFG_LOOPBACK          _BIT(2)
#define SPI_CFG_SLAVE_MODE        _BIT(1)
#define SPI_CFG_ENABLE            _BIT(0)

/* SPI slave_enable register definitions (SPI_SLV_ENAB_REG) */
#define SPI_SLV_EN(n)             _SBF(((n) << 1), 0x1)
#define SPI_SLV_SUSPEND(n)        _SBF(((n) << 1), 0x3)

/* SPI tx_fifo_flush register definitions (SPI_TXF_FLUSH_REG) */
#define SPI_TXFF_FLUSH            _BIT(1)

/* SPI dma_settings register definitions (SPI_DMA_SET_REG) */
#define SPI_DMA_TX_EN             _BIT(1)
#define SPI_DMA_RX_EN             _BIT(0)

/* SPI status register definitions (SPI_STS_REG) */
#define SPI_ST_SMS_BUSY           _BIT(5)
#define SPI_ST_BUSY               _BIT(4)
#define SPI_ST_RX_FF              _BIT(3)
#define SPI_ST_RX_EMPTY           _BIT(2)
#define SPI_ST_TX_FF              _BIT(1)
#define SPI_ST_TX_EMPTY           _BIT(0)

/* SPI slv_setting registers definitions (SPI_SLV_SET1_REG) */
#define SPI_SLV1_INTER_TX_DLY(n)  _SBF(24, ((n) & 0xFF))
#define SPI_SLV1_NUM_WORDS(n)     _SBF(16, ((n) & 0xFF))
#define SPI_SLV1_CLK_PS(n)        _SBF(8, ((n) & 0xFF))
#define SPI_SLV1_CLK_PS_GET(n)    (((n) >> 8) & 0xFF)
#define SPI_SLV1_CLK_DIV1(n)      ((n) & 0xFF)
#define SPI_SLV1_CLK_DIV1_GET(n)  ((n) & 0xFF)

/* SPI slv_setting registers definitions (SPI_SLV_SET2_REG) */
#define SPI_SLV2_PPCS_DLY(n)      _SBF(9, ((n) & 0xFF))
#define SPI_SLV2_CS_HIGH          _BIT(8)
#define SPI_SLV2_SSI_MODE         _BIT(7)
#define SPI_SLV2_SPO              _BIT(6)
#define SPI_SLV2_SPH              _BIT(5)
#define SPI_SLV2_WD_SZ(n)         ((n) & 0x1F)

/* SPI int_threshold registers definitions (SPI_INT_TRSH_REG) */
#define SPI_INT_TSHLD_TX(n)       _SBF(8, ((n) & 0xFF))
#define SPI_INT_TSHLD_RX(n)       ((n) & 0xFF)

/* SPI intterrupt registers definitions ( SPI_INT_xxx) */
#define SPI_SMS_INT               _BIT(4)
#define SPI_TX_INT                _BIT(3)
#define SPI_RX_INT                _BIT(2)
#define SPI_TO_INT                _BIT(1)
#define SPI_OVR_INT               _BIT(0)
#define SPI_ALL_INTS              (SPI_SMS_INT | SPI_TX_INT | SPI_RX_INT | SPI_TO_INT | SPI_OVR_INT)

/***********************************************************************
* Audio Subsystem (ADSS) register definitions
**********************************************************************/

/* I2S Controller Module Register Structure */
#define I2S_FORMAT_SETTINGS       __REG (I2S_PHYS + 0x00)
#define I2S_CFG_MUX_SETTINGS      __REG (I2S_PHYS + 0x04)
#define I2S_CLASSED_CFG           __REG (I2S_PHYS + 0x08)
#define I2S_CLASSED_STS           __REG (I2S_PHYS + 0x0C)
#define I2S_N_SOF_COUNTER         __REG (I2S_PHYS + 0x10)

/* I2S channels */
#define I2S_CH_TX0 1
#define I2S_CH_TX1 2
#define I2S_CH_RX0 3
#define I2S_CH_RX1 4

/* I2S channel specific registers */
#define I2S_CH_LEFT_16BIT(n)      __REG (I2S_PHYS + ((n) * 0x80) + 0x00)
#define I2S_CH_RIGHT_16BIT(n)     __REG (I2S_PHYS + ((n) * 0x80) + 0x04)
#define I2S_CH_LEFT_24BIT(n)      __REG (I2S_PHYS + ((n) * 0x80) + 0x08)
#define I2S_CH_RIGHT_24BIT(n)     __REG (I2S_PHYS + ((n) * 0x80) + 0x0C)
#define I2S_CH_INT_STATUS(n)      __REG (I2S_PHYS + ((n) * 0x80) + 0x10)
#define I2S_CH_INT_MASK(n)        __REG (I2S_PHYS + ((n) * 0x80) + 0x14)
#define I2S_CH_LEFT32(n)          __REG (I2S_PHYS + ((n) * 0x80) + 0x20)
#define I2S_CH_RIGHT32(n)         __REG (I2S_PHYS + ((n) * 0x80) + 0x40)
#define I2S_CH_INTERLEAVED(n)     __REG (I2S_PHYS + ((n) * 0x80) + 0x60)

/* I2S format settings register defines */
#define I2S_FORMAT_I2S            0x3
#define I2S_FORMAT_LSB16          0x4
#define I2S_FORMAT_LSB18          0x5
#define I2S_FORMAT_LSB20          0x6
#define I2S_FORMAT_LSB24          0x7
#define I2S_FORMAT_MASK           0x7
#define I2S_SET_FORMAT(n, s)      ((s) << (((n) - 1) * 3))

/* I2S Mux configuration setting defines */
#define I2S_RXO_SELECT_MASTER     _BIT(1)
#define I2S_RX1_SELECT_MASTER     _BIT(2)

/* I2S interrupt status and mask bits */
#define I2S_FIFO_RIGHT_UNDERRUN   _BIT(0)
#define I2S_FIFO_LEFT_UNDERRUN    _BIT(1)
#define I2S_FIFO_RIGHT_OVERRUN    _BIT(2)
#define I2S_FIFO_LEFT_OVERRUN     _BIT(3)
#define I2S_FIFO_LEFT_FULL        _BIT(4)
#define I2S_FIFO_LEFT_HALF_FULL   _BIT(5) /* RX only */
#define I2S_FIFO_LEFT_HALF_EMPTY  _BIT(5) /* TX only */
#define I2S_FIFO_LEFT_NOT_EMPTY   _BIT(6) /* RX only */
#define I2S_FIFO_LEFT_EMPTY       _BIT(6) /* TX only */
#define I2S_FIFO_RIGHT_FULL       _BIT(7)
#define I2S_FIFO_RIGHT_HALF_FULL  _BIT(8) /* RX only */
#define I2S_FIFO_RIGHT_HALF_EMPTY _BIT(8) /* TX only */
#define I2S_FIFO_RIGHT_NOT_EMPTY  _BIT(9) /* RX only */
#define I2S_FIFO_RIGHT_EMPTY      _BIT(9) /* TX only */
#define I2S_FIFO_ALL_MASK         0x3FF

/***********************************************************************
 * ADC_REG register definitions
 **********************************************************************/
#define ADC_CON_REG            __REG (ADC_PHYS + 0x20)

/***********************************************************************
 * SYS_REG register definitions
 **********************************************************************/
#define SYS_RNG_OSC_CFG        __REG (SYS_PHYS + 0x14)
#define SYS_ADC_PD             __REG (SYS_PHYS + 0x18)
#define SYS_SDMMC_DELAYMODES   __REG (SYS_PHYS + 0x2C)
#define SYS_USB_ATX_PLL_PD_REG __REG (SYS_PHYS + 0x30)
#define SYS_USB_OTG_CFG        __REG (SYS_PHYS + 0x34)
#define SYS_USB_OTG_LED_CTL    __REG (SYS_PHYS + 0x38)
#define SYS_USB_PLL_NDEC       __REG (SYS_PHYS + 0x40)
#define SYS_USB_PLL_MDEC       __REG (SYS_PHYS + 0x44)
#define SYS_USB_PLL_PDEC       __REG (SYS_PHYS + 0x48)
#define SYS_USB_PLL_SELR       __REG (SYS_PHYS + 0x4C)
#define SYS_USB_PLL_SELI       __REG (SYS_PHYS + 0x50)
#define SYS_USB_PLL_SELP       __REG (SYS_PHYS + 0x54)

#define SYS_MPMC_DELAY      __REG (SYS_PHYS + 0x68)
#define SYS_MPMC_WTD_DEL0   __REG (SYS_PHYS + 0x6C)
#define SYS_MPMC_WTD_DEL1   __REG (SYS_PHYS + 0x70)
#define SYS_MPMC_TESTMODE0  __REG (SYS_PHYS + 0x78)
#define SYS_REMAP_ADDR      __REG (SYS_PHYS + 0x84)
#define SYS_MUX_LCD_EBI     __REG (SYS_PHYS + 0x90)
#define SYS_MUX_GPIO_MCI    __REG (SYS_PHYS + 0x94)
#define SYS_MUX_NAND_MCI    __REG (SYS_PHYS + 0x98)

/***********************************************************************
 * GPIO register definitions
 **********************************************************************/
#define GPIO_STATE(port)     __REG (GPIO_PHYS + (port) + 0x00)
#define GPIO_STATE_M0(port)  __REG (GPIO_PHYS + (port) + 0x10)
#define GPIO_M0_SET(port)    __REG (GPIO_PHYS + (port) + 0x14)
#define GPIO_M0_RESET(port)  __REG (GPIO_PHYS + (port) + 0x18)
#define GPIO_STATE_M1(port)  __REG (GPIO_PHYS + (port) + 0x20)
#define GPIO_M1_SET(port)    __REG (GPIO_PHYS + (port) + 0x24)
#define GPIO_M1_RESET(port)  __REG (GPIO_PHYS + (port) + 0x28)

#define GPIO_OUT_LOW(port, pin)  do { GPIO_M1_SET(port) = pin; GPIO_M0_RESET(port) = pin;} while(0)
#define GPIO_OUT_HIGH(port, pin) do { GPIO_M1_SET(port) = pin; GPIO_M0_SET(port) = pin;} while(0)
#define GPIO_IN(port, pin)       do { GPIO_M1_RESET(port) = pin; GPIO_M0_RESET(port) = pin;} while(0)
#define GPIO_DRV_IP(port, pin)   do { GPIO_M1_RESET(port) = pin; GPIO_M0_SET(port) = pin;} while(0)

#define IOCONF_EBI_MCI       (0x000)
#define IOCONF_EBI_I2STX_0   (0x040)
#define IOCONF_CGU           (0x080)
#define IOCONF_I2SRX_0       (0x0c0)
#define IOCONF_I2SRX_1       (0x100)
#define IOCONF_I2STX_1       (0x140)
#define IOCONF_EBI           (0x180)
#define IOCONF_GPIO          (0x1c0)
#define IOCONF_I2C1          (0x200)
#define IOCONF_SPI           (0x240)
#define IOCONF_NAND_CTRL     (0x280)
#define IOCONF_PWM           (0x2c0)
#define IOCONF_UART          (0x300)


/***********************************************************************
 * MPMC memory controller register definitions
 **********************************************************************/
#define MPMC_CTRL           __REG (MPMC_PHYS + 0x000)
#define MPMC_STATUS         __REG (MPMC_PHYS + 0x004)
#define MPMC_CONFIG         __REG (MPMC_PHYS + 0x008)
#define MPMC_DYNCTL         __REG (MPMC_PHYS + 0x020)
#define MPMC_DYNREF         __REG (MPMC_PHYS + 0x024)
#define MPMC_DYRDCFG        __REG (MPMC_PHYS + 0x028)
#define MPMC_DYTRP          __REG (MPMC_PHYS + 0x030)
#define MPMC_DYTRAS         __REG (MPMC_PHYS + 0x034)
#define MPMC_DYTSREX        __REG (MPMC_PHYS + 0x038)
#define MPMC_DYTAPR         __REG (MPMC_PHYS + 0x03C)
#define MPMC_DYTDAL         __REG (MPMC_PHYS + 0x040)
#define MPMC_DYTWR          __REG (MPMC_PHYS + 0x044)
#define MPMC_DYTRC          __REG (MPMC_PHYS + 0x048)
#define MPMC_DYTRFC         __REG (MPMC_PHYS + 0x04C)
#define MPMC_DYTXSR         __REG (MPMC_PHYS + 0x050)
#define MPMC_DYTRRD         __REG (MPMC_PHYS + 0x054)
#define MPMC_DYTMRD         __REG (MPMC_PHYS + 0x058)
#define MPMC_STEXDWT        __REG (MPMC_PHYS + 0x080)
#define MPMC_DYCONFIG       __REG (MPMC_PHYS + 0x100)
#define MPMC_DYRASCAS       __REG (MPMC_PHYS + 0x104)
#define MPMC_STCONFIG0      __REG (MPMC_PHYS + 0x200)
#define MPMC_STWTWEN0       __REG (MPMC_PHYS + 0x204)
#define MPMC_STWTOEN0       __REG (MPMC_PHYS + 0x208)
#define MPMC_STWTRD0        __REG (MPMC_PHYS + 0x20C)
#define MPMC_STWTPG0        __REG (MPMC_PHYS + 0x210)
#define MPMC_STWTWR0        __REG (MPMC_PHYS + 0x214)
#define MPMC_STWTTURN0      __REG (MPMC_PHYS + 0x218)
#define MPMC_STCONFIG1      __REG (MPMC_PHYS + 0x220)
#define MPMC_STWTWEN1       __REG (MPMC_PHYS + 0x224)
#define MPMC_STWTOEN1       __REG (MPMC_PHYS + 0x228)
#define MPMC_STWTRD1        __REG (MPMC_PHYS + 0x22C)
#define MPMC_STWTPG1        __REG (MPMC_PHYS + 0x230)
#define MPMC_STWTWR1        __REG (MPMC_PHYS + 0x234)
#define MPMC_STWTTURN1      __REG (MPMC_PHYS + 0x238)

#define NS_TO_MPMCCLK(ns, clk)	(((ns)*((clk + 500)/1000) + 500000)/1000000)
/* MPMC Controller Bit Field constants*/
#define MPMC_CTL_LOW_PWR               _BIT(2)
#define MPMC_CTL_ENABLE                _BIT(0)
/* MPMC status Bit Field constants*/
#define MPMC_STATUS_SA                 _BIT(2)
#define MPMC_STATUS_WR_BUF             _BIT(1)
#define MPMC_STATUS_BUSY               _BIT(0)
/* MPMC config Bit Field constants*/
#define MPMC_CFG_SDCCLK_1_2            _BIT(8)
#define MPMC_CFG_SDCCLK_1_1            (0)
/* SDRAM Controller Bit Field constants*/
#define MPMC_SDRAMC_CTL_DP             _BIT(13)
#define MPMC_SDRAMC_CTL_NORMAL_CMD     _SBF(7,0)
#define MPMC_SDRAMC_CTL_MODE_CMD       _SBF(7,1)
#define MPMC_SDRAMC_CTL_PALL_CMD       _SBF(7,2)
#define MPMC_SDRAMC_CTL_NOP_CMD        _SBF(7,3)
#define MPMC_SDRAMC_CTL_MCC            _BIT(5)
#define MPMC_SDRAMC_CTL_SR             _BIT(2)
#define MPMC_SDRAMC_CTL_CS             _BIT(1)
#define MPMC_SDRAMC_CTL_CE             _BIT(0)
/* SDRAM Config Bit Field constants*/
#define MPMC_SDRAMC_CFG_SDRAM_MD       _SBF(3,0)
#define MPMC_SDRAMC_CFG_LOW_PWR_MD     _SBF(3,1)
#define MPMC_SDRAMC_CFG_SYNC_FLASH     _SBF(3,2)
#define MPMC_SDRAMC_CFG_BUF_EN         _BIT(19)
#define MPMC_SDRAMC_CFG_WP             _BIT(20)


/* SDRAM Read Config Bit Field constants*/
#define MPMC_SDRAMC_RDCFG_CLKOUTDELAY_STG       _SBF(0,0)
#define MPMC_SDRAMC_RDCFG_CMDDELAY_STG          _SBF(0,1)
#define MPMC_SDRAMC_RDCFG_CMDDELAY_P1_STG       _SBF(0,2)
#define MPMC_SDRAMC_RDCFG_CMDDELAY_P2_STG       _SBF(0,3)
/* SDRAM RASCAS Bit Field constants*/
#define MPMC_SDRAMC_RASCAS_CAS0        _SBF(8,0)
#define MPMC_SDRAMC_RASCAS_CAS1        _SBF(8,1)
#define MPMC_SDRAMC_RASCAS_CAS2        _SBF(8,2)
#define MPMC_SDRAMC_RASCAS_CAS3        _SBF(8,3)
#define MPMC_SDRAMC_RASCAS_RAS0        _SBF(0,0)
#define MPMC_SDRAMC_RASCAS_RAS1        _SBF(0,1)
#define MPMC_SDRAMC_RASCAS_RAS2        _SBF(0,2)
#define MPMC_SDRAMC_RASCAS_RAS3        _SBF(0,3)

/***********************************************************************
 * SD/MMC MCI register definitions
 **********************************************************************/
#define SDMMC_CTRL            __REG (SDMMC_PHYS + 0x000)
#define SDMMC_PWREN           __REG (SDMMC_PHYS + 0x004)
#define SDMMC_CLKDIV          __REG (SDMMC_PHYS + 0x008)
#define SDMMC_CLKSRC          __REG (SDMMC_PHYS + 0x00c)
#define SDMMC_CLKENA          __REG (SDMMC_PHYS + 0x010)
#define SDMMC_TMOUT           __REG (SDMMC_PHYS + 0x014)
#define SDMMC_CTYPE           __REG (SDMMC_PHYS + 0x018)
#define SDMMC_BLKSIZ          __REG (SDMMC_PHYS + 0x01c)
#define SDMMC_BYTCNT          __REG (SDMMC_PHYS + 0x020)
#define SDMMC_INTMASK         __REG (SDMMC_PHYS + 0x024)
#define SDMMC_CMDARG          __REG (SDMMC_PHYS + 0x028)
#define SDMMC_CMD             __REG (SDMMC_PHYS + 0x02c)
#define SDMMC_RESP0           __REG (SDMMC_PHYS + 0x030)
#define SDMMC_RESP1           __REG (SDMMC_PHYS + 0x034)
#define SDMMC_RESP2           __REG (SDMMC_PHYS + 0x038)
#define SDMMC_RESP3           __REG (SDMMC_PHYS + 0x03c)
#define SDMMC_MINTSTS         __REG (SDMMC_PHYS + 0x040)
#define SDMMC_RINTSTS         __REG (SDMMC_PHYS + 0x044)
#define SDMMC_STATUS          __REG (SDMMC_PHYS + 0x048)
#define SDMMC_FIFOTH          __REG (SDMMC_PHYS + 0x04c)
#define SDMMC_TCBCNT          __REG (SDMMC_PHYS + 0x05c)
#define SDMMC_TBBCNT          __REG (SDMMC_PHYS + 0x060)
#define SDMMC_DEBNCE          __REG (SDMMC_PHYS + 0x064)
#define SDMMC_USRID           __REG (SDMMC_PHYS + 0x068)
#define SDMMC_VERID           __REG (SDMMC_PHYS + 0x06c)
#define SDMMC_HCON            __REG (SDMMC_PHYS + 0x070)
#define SDMMC_DATA            __REG (SDMMC_PHYS + 0x100)
#define SDMMC_DATA_ADR        (SDMMC_PHYS + 0x100)

/* Control register defines */
#define SDMMC_CTRL_CEATA_INT_EN   _BIT(11)
#define SDMMC_CTRL_SEND_AS_CCSD   _BIT(10)
#define SDMMC_CTRL_SEND_CCSD      _BIT(9)
#define SDMMC_CTRL_ABRT_READ_DATA _BIT(8)
#define SDMMC_CTRL_SEND_IRQ_RESP  _BIT(7)
#define SDMMC_CTRL_READ_WAIT      _BIT(6)
#define SDMMC_CTRL_DMA_ENABLE     _BIT(5)
#define SDMMC_CTRL_INT_ENABLE     _BIT(4)
#define SDMMC_CTRL_DMA_RESET      _BIT(2)
#define SDMMC_CTRL_FIFO_RESET     _BIT(1)
#define SDMMC_CTRL_RESET          _BIT(0)
/* Clock Enable register defines */
#define SDMMC_CLKEN_LOW_PWR      _BIT(16)
#define SDMMC_CLKEN_ENABLE       _BIT(0)
/* time-out register defines */
#define SDMMC_TMOUT_DATA(n)      _SBF(8, (n))
#define SDMMC_TMOUT_DATA_MSK     0xFFFFFF00
#define SDMMC_TMOUT_RESP(n)      ((n) & 0xFF)
#define SDMMC_TMOUT_RESP_MSK     0xFF
/* card-type register defines */
#define SDMMC_CTYPE_8BIT         _BIT(16)
#define SDMMC_CTYPE_4BIT         _BIT(0)
/* Interrupt status & mask register defines */
#define SDMMC_INT_SDIO           _BIT(16)
#define SDMMC_INT_EBE            _BIT(15)
#define SDMMC_INT_ACD            _BIT(14)
#define SDMMC_INT_SBE            _BIT(13)
#define SDMMC_INT_HLE            _BIT(12)
#define SDMMC_INT_FRUN           _BIT(11)
#define SDMMC_INT_HTO            _BIT(10)
#define SDMMC_INT_DTO            _BIT(9)
#define SDMMC_INT_RTO            _BIT(8)
#define SDMMC_INT_DCRC           _BIT(7)
#define SDMMC_INT_RCRC           _BIT(6)
#define SDMMC_INT_RXDR           _BIT(5)
#define SDMMC_INT_TXDR           _BIT(4)
#define SDMMC_INT_DATA_OVER      _BIT(3)
#define SDMMC_INT_CMD_DONE       _BIT(2)
#define SDMMC_INT_RESP_ERR       _BIT(1)
#define SDMMC_INT_CD             _BIT(0)
#define SDMMC_INT_ERROR          0xbfc2
/* Command register defines */
#define SDMMC_CMD_START         _BIT(31)
#define SDMMC_CMD_CCS_EXP       _BIT(23)
#define SDMMC_CMD_CEATA_RD      _BIT(22)
#define SDMMC_CMD_UPD_CLK       _BIT(21)
#define SDMMC_CMD_INIT          _BIT(15)
#define SDMMC_CMD_STOP          _BIT(14)
#define SDMMC_CMD_PRV_DAT_WAIT  _BIT(13)
#define SDMMC_CMD_SEND_STOP     _BIT(12)
#define SDMMC_CMD_STRM_MODE     _BIT(11)
#define SDMMC_CMD_DAT_WR        _BIT(10)
#define SDMMC_CMD_DAT_EXP       _BIT(9)
#define SDMMC_CMD_RESP_CRC      _BIT(8)
#define SDMMC_CMD_RESP_LONG     _BIT(7)
#define SDMMC_CMD_RESP_EXP      _BIT(6)
#define SDMMC_CMD_INDX(n)       ((n) & 0x1F)
/* Status register defines */
#define SDMMC_GET_FCNT(x)       (((x)>>17) & 0x1FF)
#define SDMMC_FIFO_SZ           32


/***********************************************************************
 * NAND Controller register definitions
 **********************************************************************/
#define NAND_IRQSTATUS1                __REG (NANDC_PHYS + 0x000)
#define NAND_IRQMASK1                  __REG (NANDC_PHYS + 0x004)
#define NAND_IRQSTATUSRAW1             __REG (NANDC_PHYS + 0x008)
#define NAND_CONFIG                    __REG (NANDC_PHYS + 0x00C)
#define NAND_IOCONFIG                  __REG (NANDC_PHYS + 0x010)
#define NAND_TIMING1                   __REG (NANDC_PHYS + 0x014)
#define NAND_TIMING2                   __REG (NANDC_PHYS + 0x018)
#define NAND_SETCMD                    __REG (NANDC_PHYS + 0x020)
#define NAND_SETADDR                   __REG (NANDC_PHYS + 0x024)
#define NAND_WRITEDATA                 __REG (NANDC_PHYS + 0x028)
#define NAND_SETCE                     __REG (NANDC_PHYS + 0x02C)
#define NAND_READDATA                  __REG (NANDC_PHYS + 0x030)
#define NAND_CHECKSTS                  __REG (NANDC_PHYS + 0x034)
#define NAND_CONTROLFLOW               __REG (NANDC_PHYS + 0x038)
#define NAND_GPIO1                     __REG (NANDC_PHYS + 0x040)
#define NAND_GPIO2                     __REG (NANDC_PHYS + 0x044)
#define NAND_IRQSTATUS2                __REG (NANDC_PHYS + 0x048)
#define NAND_IRQMASK2                  __REG (NANDC_PHYS + 0x04C)
#define NAND_IRQSTATUSRAW2             __REG (NANDC_PHYS + 0x050)
#define NAND_ECCERRSTATUS              __REG (NANDC_PHYS + 0x078)

/* NAND internal SDRAM address definitions*/
#define NAND_BUFFER_ADRESS             __REG (IO_NAND_BUF_PHYS)

/* Register description of NANDIRQSTATUS1 */
#define NAND_NANDIRQSTATUS1_RB4_POS_EDGE       _BIT(31)
#define NAND_NANDIRQSTATUS1_RB3_POS_EDGE       _BIT(30)
#define NAND_NANDIRQSTATUS1_RB2_POS_EDGE       _BIT(29)
#define NAND_NANDIRQSTATUS1_RB1_POS_EDGE       _BIT(28)
#define NAND_NANDIRQSTATUS1_ERASED_RAM1        _BIT(27)
#define NAND_NANDIRQSTATUS1_ERASED_RAM0        _BIT(26)
#define NAND_NANDIRQSTATUS1_WR_RAM1            _BIT(25)
#define NAND_NANDIRQSTATUS1_WR_RAM0            _BIT(24)
#define NAND_NANDIRQSTATUS1_RD_RAM1            _BIT(23)
#define NAND_NANDIRQSTATUS1_RD_RAM0            _BIT(22)
#define NAND_NANDIRQSTATUS1_ECC_DEC_RAM0       _BIT(21)
#define NAND_NANDIRQSTATUS1_ECC_ENC_RAM0       _BIT(20)
#define NAND_NANDIRQSTATUS1_ECC_DEC_RAM1       _BIT(19)
#define NAND_NANDIRQSTATUS1_ECC_ENC_RAM1       _BIT(18)
#define NAND_NANDIRQSTATUS1_NOERR_RAM0         _BIT(17)
#define NAND_NANDIRQSTATUS1_ERR1_RAM0          _BIT(16)
#define NAND_NANDIRQSTATUS1_ERR2_RAM0          _BIT(15)
#define NAND_NANDIRQSTATUS1_ERR3_RAM0          _BIT(14)
#define NAND_NANDIRQSTATUS1_ERR4_RAM0          _BIT(13)
#define NAND_NANDIRQSTATUS1_ERR5_RAM0          _BIT(12)
#define NAND_NANDIRQSTATUS1_ERR_UNR_RAM0       _BIT(11)
#define NAND_NANDIRQSTATUS1_NOERR_RAM1         _BIT(10)
#define NAND_NANDIRQSTATUS1_ERR1_RAM1          _BIT(9)
#define NAND_NANDIRQSTATUS1_ERR2_RAM1          _BIT(8)
#define NAND_NANDIRQSTATUS1_ERR3_RAM1          _BIT(7)
#define NAND_NANDIRQSTATUS1_ERR4_RAM1          _BIT(6)
#define NAND_NANDIRQSTATUS1_ERR5_RAM1          _BIT(5)
#define NAND_NANDIRQSTATUS1_ERR_UNR_RAM1       _BIT(4)
#define NAND_NANDIRQSTATUS1_AES_DONE_RAM1      _BIT(1)
#define NAND_NANDIRQSTATUS1_AES_DONE_RAM0      _BIT(0)

/* Register description of NANDCONFIG */
#define NAND_NANDCONFIG_PEC                _BIT(15) /* Power off ECC clock*/
#define NAND_NANDCONFIG_ECGC               _BIT(13) /* Enable ECC clock gating*/
#define NAND_NANDCONFIG_8BIT_ECC           _BIT(12) /* ECC mode*/
#define NAND_NANDCONFIG_TL_528             _SBF(10, 0x0) /* Transfer limit*/
#define NAND_NANDCONFIG_TL_516             _SBF(10, 0x2) /* Transfer limit*/
#define NAND_NANDCONFIG_TL_512             _SBF(10, 0x3) /* Transfer limit*/
#define NAND_NANDCONFIG_TL_MASK            _SBF(10, 0x3) /* Transfer limit*/
#define NAND_NANDCONFIG_EO                 _BIT(9) /* */
#define NAND_NANDCONFIG_DC                 _BIT(8) /* Deactivate CE enable*/
#define NAND_NANDCONFIG_M                  _BIT(7) /* 512 mode*/
#define NAND_NANDCONFIG_LC_0               _SBF(5, 0x0) /* Latency Configuration: zero wait state*/
#define NAND_NANDCONFIG_LC_1               _SBF(5, 0x1) /* Latency Configuration: one wait state*/
#define NAND_NANDCONFIG_LC_2               _SBF(5, 0x2) /* Latency Configuration: two wait state*/
#define NAND_NANDCONFIG_LC_MASK            _SBF(5, 0x3) /* Latency Configuration*/
#define NAND_NANDCONFIG_ES                 _BIT(4) /* Endianess setting */
#define NAND_NANDCONFIG_DE                 _BIT(3) /* DMA external enable*/
#define NAND_NANDCONFIG_AO                 _BIT(2) /* */
#define NAND_NANDCONFIG_WD                 _BIT(1) /* Wide device*/
#define NAND_NANDCONFIG_EC                 _BIT(0) /* ECC on*/

/* Register description of NANDIOCONFIG */
#define NAND_NANDIOCONFIG_CFG_IO_DRIVE        _BIT(24) /* Nand IO drive default*/
#define NAND_NANDIOCONFIG_CFG_DATA_DEF(n)     _SBF(8, ((n) & 0xFFFF)) /* Data to nand default*/
#define NAND_NANDIOCONFIG_CFG_CLE_1           _SBF(6, 0x01) /* CLE default*/
#define NAND_NANDIOCONFIG_CFG_ALE_1           _SBF(4, 0x01) /* ALE default*/
#define NAND_NANDIOCONFIG_CFG_WE_1            _SBF(2, 0x01) /* WE_n default*/
#define NAND_NANDIOCONFIG_CFG_RE_1            _SBF(0, 0x01) /* RE_n default*/

/* Register description of NANDTIMING1 */
#define NAND_NANDTIMING1_DEFAULT		   (0x000FFFFF)
#define NAND_NANDTIMING1_TSRD(n)           _SBF(20, ((n) & 0x3)) /* Single data input delay*/
#define NAND_NANDTIMING1_TALS(n)           _SBF(16, ((n) & 0x7)) /* Address setup time*/
#define NAND_NANDTIMING1_TALH(n)           _SBF(12, ((n) & 0x7)) /* Address hold time*/
#define NAND_NANDTIMING1_TCLS(n)           _SBF(4, ((n) & 0x7)) /* Command setup time*/
#define NAND_NANDTIMING1_TCLH(n)           ((n) & 0x7) /* Command hold time*/

/* Register description of NANDTIMING2 */
#define NAND_NANDTIMING2_DEFAULT		   (0xFFFFFFFF)
#define NAND_NANDTIMING2_TDRD(n)           _SBF(28, ((n) & 0x7)) /* Data input delay*/
#define NAND_NANDTIMING2_TEBI(n)           _SBF(24, ((n) & 0x7)) /* EBI delay time*/
#define NAND_NANDTIMING2_TCH(n)            _SBF(20, ((n) & 0x7)) /* Chip select hold time*/
#define NAND_NANDTIMING2_TCS(n)            _SBF(16, ((n) & 0x7)) /* Chip select setup time*/
#define NAND_NANDTIMING2_TRH(n)            _SBF(12, ((n) & 0x7)) /* Read enable high hold*/
#define NAND_NANDTIMING2_TRP(n)            _SBF(8, ((n) & 0x7)) /* Read enable pulse width*/
#define NAND_NANDTIMING2_TWH(n)            _SBF(4, ((n) & 0x7)) /* Write enable high hold*/
#define NAND_NANDTIMING2_TWP(n)            ((n) & 0x7) /* Write enable pulse width*/

/* Register description of NANDSETCE */
#define NAND_NANDSETCE_OVR_EN(n)        _BIT(((n) & 0x3) + 12) /* */
#define NAND_NANDSETCE_OVR_V(n)         _BIT(((n) & 0x3) + 8) /* */
#define NAND_NANDSETCE_WP               _BIT(4) /* WP_n pin value*/
#define NAND_NANDSETCE_CV_MASK          0x0F /* Chip select value*/
#define NAND_NANDSETCE_CV(n)            (0x0F & ~_BIT(((n) & 0x3))) /* Chip select value*/
#define NAND_NANDSETCE_CV0              _BIT(0)

/* Register description of NANDCHECKSTS */
#define NAND_NANDCHECKSTS_RB4_EDGE       _BIT(8) /* mNAND_RYBN3 rising edge*/
#define NAND_NANDCHECKSTS_RB3_EDGE       _BIT(7) /* mNAND_RYBN2 rising edge*/
#define NAND_NANDCHECKSTS_RB2_EDGE       _BIT(6) /* mNAND_RYBN1 rising edge*/
#define NAND_NANDCHECKSTS_RB1_EDGE       _BIT(5) /* mNAND_RYBN0 rising edge*/
#define NAND_NANDCHECKSTS_RB4_LVL        _BIT(4) /* mNAND_RYBN3 value*/
#define NAND_NANDCHECKSTS_RB3_LVL        _BIT(3) /* mNAND_RYBN2 value*/
#define NAND_NANDCHECKSTS_RB2_LVL        _BIT(2) /* mNAND_RYBN1 value*/
#define NAND_NANDCHECKSTS_RB1_LVL        _BIT(1) /* mNAND_RYBN0 value*/
#define NAND_NANDCHECKSTS_APB_BSY        _BIT(0) /* APB busy*/

/* Register description of NANDCONTROLFLOW */

/* Write the contents of SRAM1 to the NAND flash*/
#define NAND_CTRL_WR_RAM1           _BIT(5)
/* Write the contents of SRAM0 to the NAND flash */
#define NAND_CTRL_WR_RAM0           _BIT(4)
/* Read a defined number of bytes from the NAND flash and store them in SRAM1*/
#define NAND_CTRL_RD_RAM1           _BIT(1)
/* Read a defined number of bytes from the NAND flash and store them in SRAM0*/
#define NAND_CTRL_RD_RAM0           _BIT(0)

/* Register description of ... */
#define NAND_AES_AHB_EN             _BIT(7)
#define NAND_AES_AHB_DCRYPT_RAM1    _BIT(1)
#define NAND_AES_AHB_DCRYPT_RAM0    _BIT(0)




#endif  /* __ASM_ARCH_REGISTERS_H */
