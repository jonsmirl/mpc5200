/* linux/arch/arm/mach-lpc313x/include/mach/cgu.h
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 *  CGU defines and register structures for LPC313x and LPC315x SoCs.
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

#ifndef LPC313X_CGU_H
#define LPC313X_CGU_H

 /***********************************************************************
 * CGU register definitions
 **********************************************************************/
#define CGU_SB_NR_BASE     12
#define CGU_SB_NR_CLK      92
#define CGU_SB_NR_BCR      5
#define CGU_SB_NR_FRACDIV  24
#define CGU_SB_NR_DYN_FDIV 7
#define CGU_SB_NR_ESR      89
#define CGU_SB_BASE0_FDIV_CNT           7
#define CGU_SB_BASE0_FDIV_LOW_ID        0
#define CGU_SB_BASE0_FDIV_HIGH_ID       6
#define CGU_SB_BASE0_FDIV0_W            8

#define CGU_SB_BASE1_FDIV_CNT           2
#define CGU_SB_BASE1_FDIV_LOW_ID        7
#define CGU_SB_BASE1_FDIV_HIGH_ID       8

#define CGU_SB_BASE2_FDIV_CNT           2
#define CGU_SB_BASE2_FDIV_LOW_ID        9
#define CGU_SB_BASE2_FDIV_HIGH_ID       10

#define CGU_SB_BASE3_FDIV_CNT           3
#define CGU_SB_BASE3_FDIV_LOW_ID        11
#define CGU_SB_BASE3_FDIV_HIGH_ID       13

#define CGU_SB_BASE4_FDIV_CNT           1
#define CGU_SB_BASE4_FDIV_LOW_ID        14
#define CGU_SB_BASE4_FDIV_HIGH_ID       14

#define CGU_SB_BASE5_FDIV_CNT           1
#define CGU_SB_BASE5_FDIV_LOW_ID        15
#define CGU_SB_BASE5_FDIV_HIGH_ID       15

#define CGU_SB_BASE6_FDIV_CNT           1
#define CGU_SB_BASE6_FDIV_LOW_ID        16
#define CGU_SB_BASE6_FDIV_HIGH_ID       16

#define CGU_SB_BASE7_FDIV_CNT           6
#define CGU_SB_BASE7_FDIV_LOW_ID        17
#define CGU_SB_BASE7_FDIV_HIGH_ID       22
#define CGU_SB_BASE7_FDIV0_W            13

#define CGU_SB_BASE10_FDIV_CNT          1
#define CGU_SB_BASE10_FDIV_LOW_ID       23
#define CGU_SB_BASE10_FDIV_HIGH_ID      23


typedef volatile struct
{
  /* Switches controls */
  volatile u32 base_scr[12]; /* Switch control */
  volatile u32 base_fs1[12]; /* Frequency select side 1 */
  volatile u32 base_fs2[12]; /* Frequency select side 2 */
  volatile u32 base_ssr[12]; /* Switch status */
  /* Clock enable controls (positive and inverted clock pairs share control register)*/
  volatile u32 clk_pcr[92]; /* power control */
  volatile u32 clk_psr[92]; /* power status */
  /* enable select from fractional dividers (positive and inverted clock pairs share esr)*/
  volatile u32 clk_esr[89]; /* enable select */
  /* Base controls, currently only fd_run (base wide fractional divider enable) bit.*/
  volatile u32 base_bcr[5]; /* Base control */
  /* Fractional divider controls & configuration*/
  volatile u32 base_fdc[24]; /* Fractional divider config & ctrl */
  volatile u32 base_dyn_fdc[7]; /* Fractional divider config & ctrl for dynamic fracdivs */
  volatile u32 base_dyn_sel[7]; /* Fractional divider register for selecting an external signal to trigger high-speed operation*/
} CGU_SB_REGS_T;

/* ----------------
* HP PLL Registers
* ----------------
*/
typedef volatile struct
{
  volatile u32 fin_select;
  volatile u32 mdec;
  volatile u32 ndec;
  volatile u32 pdec;
  volatile u32 mode;
  volatile u32 status;
  volatile u32 ack;
  volatile u32 req;
  volatile u32 inselr;
  volatile u32 inseli;
  volatile u32 inselp;
  volatile u32 selr;
  volatile u32 seli;
  volatile u32 selp;
} CGU_HP_CFG_REGS;

typedef volatile struct
{
  volatile u32 powermode;
  volatile u32 wd_bark;
  volatile u32 ffast_on;
  volatile u32 ffast_bypass;
  volatile u32 resetn_soft[56];
  CGU_HP_CFG_REGS hp[2];
} CGU_CONFIG_REGS;

#define CGU_SB    ((CGU_SB_REGS_T*) io_p2v(CGU_SB_PHYS))
#define CGU_CFG   ((CGU_CONFIG_REGS*) io_p2v(CGU_CFG_PHYS))


/* Switch Control Register */
#define CGU_SB_SCR_EN1              _BIT(0)
#define CGU_SB_SCR_EN2              _BIT(1)
#define CGU_SB_SCR_RST              _BIT(2)
#define CGU_SB_SCR_STOP             _BIT(3)
#define CGU_SB_SCR_FS_MASK          0x3

/* Switch Status Register */
#define CGU_SB_SSR_FS_GET(x)        ( ((x) >> 2) & 0x7)
/* Power Control Register */
#define CGU_SB_PCR_RUN              _BIT(0)
#define CGU_SB_PCR_AUTO             _BIT(1)
#define CGU_SB_PCR_WAKE_EN          _BIT(2)
#define CGU_SB_PCR_EXTEN_EN         _BIT(3)
#define CGU_SB_PCR_ENOUT_EN         _BIT(4)
/* Power Status Register */
#define CGU_SB_PSR_ACTIVE           _BIT(0)
#define CGU_SB_PSR_WAKEUP           _BIT(1)
/* Enable Select Register */
#define CGU_SB_ESR_ENABLE           _BIT(0)
#define CGU_SB_ESR_SELECT(x)        _SBF(1, (x))
#define CGU_SB_ESR_SEL_GET(x)       (((x) >> 1) & 0x7)

/* Base control Register */
#define CGU_SB_BCR_FD_RUN           _BIT(0)
/* Fractional Divider Configuration Register */
#define CGU_SB_FDC_RUN              _BIT(0)
#define CGU_SB_FDC_RESET            _BIT(1)
#define CGU_SB_FDC_STRETCH          _BIT(2)
#define CGU_SB_FDC_MADD(x)          _SBF( 3, ((x) & 0xFF))
#define CGU_SB_FDC_MSUB(x)          _SBF(11, ((x) & 0xFF))
#define CGU_SB_FDC17_MADD(x)        _SBF( 3, ((x) & 0x1FFF))
#define CGU_SB_FDC17_MSUB(x)        _SBF(16, ((x) & 0x1FFF))
#define CGU_SB_FDC_MADD_GET(x)      (((x) >> 3) & 0xFF)
#define CGU_SB_FDC_MSUB_GET(x)      ((((x) >> 11) & 0xFF) | 0xFFFFFF00)
#define CGU_SB_FDC17_MADD_GET(x)    (((x) >> 3) & 0x1FFF)
#define CGU_SB_FDC17_MSUB_GET(x)    ((((x) >> 16) & 0x1FFF) | 0xFFFFE000)
#define CGU_SB_FDC_MADD_POS         3

/* Dynamic Fractional Divider Configuration Register */
#define CGU_SB_DYN_FDC_RUN          _BIT(0)
#define CGU_SB_DYN_FDC_ALLOW        _BIT(1)
#define CGU_SB_DYN_FDC_STRETCH      _BIT(2)

/**********************************************************************
* Register description of POWERMODE
**********************************************************************/
#define CGU_POWERMODE_MASK     0x3
#define CGU_POWERMODE_NORMAL   0x1
#define CGU_POWERMODE_WAKEUP   0x3

/**********************************************************************
* Register description of WD_BARK
**********************************************************************/
#define CGU_WD_BARK            0x1

/**********************************************************************
* Register description of FFAST_ON
**********************************************************************/
#define CGU_FFAST_ON           0x1

/**********************************************************************
* Register description of FFAST_BYPASS
**********************************************************************/
#define CGU_FFAST_BYPASS       0x1

/**********************************************************************
* Register description of soft reset registers
**********************************************************************/
#define CGU_CONFIG_SOFT_RESET  0x1

/**********************************************************************
* Register description of HP_FIN_SELECT
**********************************************************************/
#define CGU_HPLL_FIN_SEL_MASK       0xf
#define CGU_FIN_SELECT_FFAST        0x0
#define CGU_FIN_SELECT_XT_DAI_BCK0  0x1
#define CGU_FIN_SELECT_XT_DAI_WS0   0x2
#define CGU_FIN_SELECT_XT_DAI_BCK1  0x3
#define CGU_FIN_SELECT_XT_DAI_WS1   0x4
#define CGU_FIN_SELECT_HPPLL0       0x5
#define CGU_FIN_SELECT_HPPLL1       0x6
#define CGU_FIN_SELECT_MAX          7

/**********************************************************************
* Register description of HP_MDEC
**********************************************************************/
#define CGU_HPLL_MDEC_MASK          0x1ffff
/**********************************************************************
* Register description of HP_NDEC
**********************************************************************/
#define CGU_HPLL_NDEC_MSK           0x3ff
/**********************************************************************
* Register description of HP_PDEC
**********************************************************************/
#define CGU_HPLL_PDEC_MSK           0x7f
/**********************************************************************
* Register description of HP_MODE
**********************************************************************/
#define CGU_HPLL_MODE_POR_VAL       0x6
#define CGU_HPLL_MODE_CLKEN         _BIT(0)
#define CGU_HPLL_MODE_SKEWEN        _BIT(1)
#define CGU_HPLL_MODE_PD            _BIT(2)
#define CGU_HPLL_MODE_DIRECTO       _BIT(3)
#define CGU_HPLL_MODE_DIRECTI       _BIT(4)
#define CGU_HPLL_MODE_FRM           _BIT(5)
#define CGU_HPLL_MODE_BANDSEL       _BIT(6)
#define CGU_HPLL_MODE_LIMUP_OFF     _BIT(7)
#define CGU_HPLL_MODE_BYPASS        _BIT(8)

/**********************************************************************
* Register description of HP1_STATUS
**********************************************************************/
#define CGU_HPLL_STATUS_FR          _BIT(1)
#define CGU_HPLL_STATUS_LOCK        _BIT(0)

/**********************************************************************
* Register description of HP_ACK & HP_REQ
**********************************************************************/
#define CGU_HPLL_ACK_P              _BIT(2)
#define CGU_HPLL_ACK_N              _BIT(1)
#define CGU_HPLL_ACK_M              _BIT(0)

/**********************************************************************
* Register description of HP1_INSELR
**********************************************************************/
#define CGU_HPLL_INSELR_MASK        0xf
/**********************************************************************
* Register description of HP1_INSELI
**********************************************************************/
#define CGU_HPLL_INSELI_MASK        0x3f
/**********************************************************************
* Register description of HP1_INSELP
**********************************************************************/
#define CGU_HPLL_INSELP_MASK        0x1f
/**********************************************************************
* Register description of HP1_SELR
**********************************************************************/
#define CGU_HPLL_SELR_MASK          0xf
/**********************************************************************
* Register description of HP1_SELI
**********************************************************************/
#define CGU_HPLL_SELI_MASK          0x3f
/**********************************************************************
* Register description of HP1_SELP
**********************************************************************/
#define CGU_HPLL_SELP_MASK          0x1f

/***********************************************************************
* Clock domain base id's
***********************************************************************/
typedef enum
{
  CGU_SB_SYS_BASE_ID = 0,
  CGU_SB_BASE_FIRST = CGU_SB_SYS_BASE_ID,
  CGU_SB_AHB0_APB0_BASE_ID,
  CGU_SB_AHB0_APB1_BASE_ID,
  CGU_SB_AHB0_APB2_BASE_ID,
  CGU_SB_AHB0_APB3_BASE_ID,
  CGU_SB_IPINT_BASE_ID,
  CGU_SB_UARTCLK_BASE_ID,
  CGU_SB_CLK1024FS_BASE_ID,
  CGU_SB_I2SRX_BCK0_BASE_ID,
  CGU_SB_I2SRX_BCK1_BASE_ID,
  CGU_SB_SPI_CLK_BASE_ID,
  CGU_SB_SYSCLK_O_BASE_ID,
  CGU_SB_BASE_LAST = CGU_SB_SYSCLK_O_BASE_ID
} CGU_DOMAIN_ID_T;

/***********************************************************************
// Clock id's (= clkid in address calculation)
***********************************************************************/
typedef enum
{
  /* domain 0 = SYS_BASE */
  CGU_SB_APB0_CLK_ID = 0,
  CGU_SYS_FIRST = CGU_SB_APB0_CLK_ID,
  CGU_SB_APB1_CLK_ID,
  CGU_SB_APB2_CLK_ID,
  CGU_SB_APB3_CLK_ID,
  CGU_SB_APB4_CLK_ID,
  CGU_SB_AHB2INTC_CLK_ID,
  CGU_SB_AHB0_CLK_ID,
  CGU_SB_EBI_CLK_ID,
  CGU_SB_DMA_PCLK_ID,
  CGU_SB_DMA_CLK_GATED_ID,
  CGU_SB_NANDFLASH_S0_CLK_ID,
  CGU_SB_NANDFLASH_ECC_CLK_ID,
  CGU_SB_NANDFLASH_AES_CLK_ID, /* valid on LPC3153 & LPC3154 only */
  CGU_SB_NANDFLASH_NAND_CLK_ID,
  CGU_SB_NANDFLASH_PCLK_ID,
  CGU_SB_CLOCK_OUT_ID,
  CGU_SB_ARM926_CORE_CLK_ID,
  CGU_SB_ARM926_BUSIF_CLK_ID,
  CGU_SB_ARM926_RETIME_CLK_ID,
  CGU_SB_SD_MMC_HCLK_ID,
  CGU_SB_SD_MMC_CCLK_IN_ID,
  CGU_SB_USB_OTG_AHB_CLK_ID,
  CGU_SB_ISRAM0_CLK_ID,
  CGU_SB_RED_CTL_RSCLK_ID,
  CGU_SB_ISRAM1_CLK_ID,
  CGU_SB_ISROM_CLK_ID,
  CGU_SB_MPMC_CFG_CLK_ID,
  CGU_SB_MPMC_CFG_CLK2_ID,
  CGU_SB_MPMC_CFG_CLK3_ID,
  CGU_SB_INTC_CLK_ID,
  CGU_SYS_LAST = CGU_SB_INTC_CLK_ID,

  /* domain 1 = AHB0APB0_BASE */
  CGU_SB_AHB2APB0_ASYNC_PCLK_ID,
  CGU_AHB0APB0_FIRST = CGU_SB_AHB2APB0_ASYNC_PCLK_ID,
  CGU_SB_EVENT_ROUTER_PCLK_ID,
  CGU_SB_ADC_PCLK_ID,
  CGU_SB_ADC_CLK_ID,
  CGU_SB_WDOG_PCLK_ID,
  CGU_SB_IOCONF_PCLK_ID,
  CGU_SB_CGU_PCLK_ID,
  CGU_SB_SYSCREG_PCLK_ID,
  CGU_SB_OTP_PCLK_ID, /* valid on LPC315x series only */
  CGU_SB_RNG_PCLK_ID,
  CGU_AHB0APB0_LAST = CGU_SB_RNG_PCLK_ID,


  /* domain 2 = AHB0APB1_BASE */
  CGU_SB_AHB2APB1_ASYNC_PCLK_ID,
  CGU_AHB0APB1_FIRST = CGU_SB_AHB2APB1_ASYNC_PCLK_ID,
  CGU_SB_TIMER0_PCLK_ID,
  CGU_SB_TIMER1_PCLK_ID,
  CGU_SB_TIMER2_PCLK_ID,
  CGU_SB_TIMER3_PCLK_ID,
  CGU_SB_PWM_PCLK_ID,
  CGU_SB_PWM_PCLK_REGS_ID,
  CGU_SB_PWM_CLK_ID,
  CGU_SB_I2C0_PCLK_ID,
  CGU_SB_I2C1_PCLK_ID,
  CGU_AHB0APB1_LAST = CGU_SB_I2C1_PCLK_ID,

  /* domain 3 = AHB0APB2_BASE */
  CGU_SB_AHB2APB2_ASYNC_PCLK_ID,
  CGU_AHB0APB2_FIRST = CGU_SB_AHB2APB2_ASYNC_PCLK_ID,
  CGU_SB_PCM_PCLK_ID,
  CGU_SB_PCM_APB_PCLK_ID,
  CGU_SB_UART_APB_CLK_ID,
  CGU_SB_LCD_PCLK_ID,
  CGU_SB_LCD_CLK_ID,
  CGU_SB_SPI_PCLK_ID,
  CGU_SB_SPI_PCLK_GATED_ID,
  CGU_AHB0APB2_LAST = CGU_SB_SPI_PCLK_GATED_ID,

  /* domain 4 = AHB0APB3_BASE */
  CGU_SB_AHB2APB3_ASYNC_PCLK_ID,
  CGU_AHB0APB3_FIRST = CGU_SB_AHB2APB3_ASYNC_PCLK_ID,
  CGU_SB_I2S_CFG_PCLK_ID,
  CGU_SB_EDGE_DET_PCLK_ID,
  CGU_SB_I2STX_FIFO_0_PCLK_ID,
  CGU_SB_I2STX_IF_0_PCLK_ID,
  CGU_SB_I2STX_FIFO_1_PCLK_ID,
  CGU_SB_I2STX_IF_1_PCLK_ID,
  CGU_SB_I2SRX_FIFO_0_PCLK_ID,
  CGU_SB_I2SRX_IF_0_PCLK_ID,
  CGU_SB_I2SRX_FIFO_1_PCLK_ID,
  CGU_SB_I2SRX_IF_1_PCLK_ID,
  CGU_SB_RSVD69_ID,
  CGU_SB_AHB2APB3_RSVD_ID,
  CGU_AHB0APB3_LAST = CGU_SB_AHB2APB3_RSVD_ID,

  /* domain 5 = PCM_BASE */
  CGU_SB_PCM_CLK_IP_ID,
  CGU_PCM_FIRST = CGU_SB_PCM_CLK_IP_ID,
  CGU_PCM_LAST = CGU_SB_PCM_CLK_IP_ID,

  /* domain 6 = UART_BASE */
  CGU_SB_UART_U_CLK_ID,
  CGU_UART_FIRST = CGU_SB_UART_U_CLK_ID,
  CGU_UART_LAST = CGU_SB_UART_U_CLK_ID,

  /* domain 7 = CLK1024FS_BASE */
  CGU_SB_I2S_EDGE_DETECT_CLK_ID,
  CGU_CLK1024FS_FIRST = CGU_SB_I2S_EDGE_DETECT_CLK_ID,
  CGU_SB_I2STX_BCK0_N_ID,
  CGU_SB_I2STX_WS0_ID,
  CGU_SB_I2STX_CLK0_ID,
  CGU_SB_I2STX_BCK1_N_ID,
  CGU_SB_I2STX_WS1_ID,
  CGU_SB_CLK_256FS_ID,
  CGU_SB_I2SRX_BCK0_N_ID,
  CGU_SB_I2SRX_WS0_ID,
  CGU_SB_I2SRX_BCK1_N_ID,
  CGU_SB_I2SRX_WS1_ID,
  CGU_SB_RSVD84_ID,
  CGU_SB_RSVD85_ID,
  CGU_SB_RSVD86_ID,
  CGU_CLK1024FS_LAST = CGU_SB_RSVD86_ID,

  /* domain 8 = BCK0_BASE */
  CGU_SB_I2SRX_BCK0_ID,
  CGU_I2SRX_BCK0_FIRST = CGU_SB_I2SRX_BCK0_ID,
  CGU_I2SRX_BCK0_LAST = CGU_SB_I2SRX_BCK0_ID,

  /* domain 9 = BCK1_BASE */
  CGU_SB_I2SRX_BCK1_ID,
  CGU_I2SRX_BCK1_FIRST = CGU_SB_I2SRX_BCK1_ID,
  CGU_I2SRX_BCK1_LAST = CGU_SB_I2SRX_BCK1_ID,

  /* domain 10 = SPI_BASE */
  CGU_SB_SPI_CLK_ID,
  CGU_SPI_FIRST = CGU_SB_SPI_CLK_ID,
  CGU_SB_SPI_CLK_GATED_ID,
  CGU_SPI_LAST = CGU_SB_SPI_CLK_GATED_ID,

  /* domain 11 = SYSCLKO_BASE */
  CGU_SB_SYSCLK_O_ID,
  CGU_SYSCLK_O_FIRST = CGU_SB_SYSCLK_O_ID,
  CGU_SYSCLK_O_LAST = CGU_SB_SYSCLK_O_ID,

  CGU_SB_INVALID_CLK_ID = -1
} CGU_CLOCK_ID_T;


/***********************************************************************
* CGU driver defines - MACROS & constants
**********************************************************************/
#define CGU_INVALID_ID  0xFFFF

/* Clocks which which need wake_en set. These are system clocks not
 * managed by individual drivers. All other clocks should be disabled
 * at startup.
 */
#define CGU_WKE_CLKS_0_31   ( _BIT(CGU_SB_APB0_CLK_ID) | _BIT(CGU_SB_APB1_CLK_ID) | \
                              _BIT(CGU_SB_APB2_CLK_ID) | _BIT(CGU_SB_APB3_CLK_ID) | _BIT(CGU_SB_APB4_CLK_ID) | \
                              _BIT(CGU_SB_AHB2INTC_CLK_ID) | _BIT(CGU_SB_AHB0_CLK_ID) | \
                              _BIT(CGU_SB_EBI_CLK_ID) | _BIT(CGU_SB_DMA_PCLK_ID) | _BIT(CGU_SB_DMA_CLK_GATED_ID) | \
                              _BIT(CGU_SB_ARM926_CORE_CLK_ID) | _BIT(CGU_SB_ARM926_BUSIF_CLK_ID) | \
                              _BIT(CGU_SB_ARM926_RETIME_CLK_ID) | _BIT(CGU_SB_ISRAM0_CLK_ID) | \
                              _BIT(CGU_SB_ISRAM1_CLK_ID) | _BIT(CGU_SB_ISROM_CLK_ID) | \
                              _BIT(CGU_SB_MPMC_CFG_CLK_ID) | _BIT(CGU_SB_MPMC_CFG_CLK2_ID) | _BIT(CGU_SB_MPMC_CFG_CLK3_ID) | \
                              _BIT(CGU_SB_INTC_CLK_ID) | _BIT(CGU_SB_AHB2APB0_ASYNC_PCLK_ID) | \
                              _BIT(CGU_SB_EVENT_ROUTER_PCLK_ID) /*| _BIT(CGU_SB_CLOCK_OUT_ID)*/)

#define CGU_WKE_CLKS_32_63 ( _BIT(CGU_SB_IOCONF_PCLK_ID - 32) | _BIT(CGU_SB_CGU_PCLK_ID - 32) | \
                             _BIT(CGU_SB_SYSCREG_PCLK_ID - 32) | \
                             _BIT(CGU_SB_AHB2APB1_ASYNC_PCLK_ID - 32) | _BIT(CGU_SB_AHB2APB2_ASYNC_PCLK_ID - 32) | \
                             _BIT(CGU_SB_AHB2APB3_ASYNC_PCLK_ID - 32) | _BIT(CGU_SB_TIMER0_PCLK_ID - 32) )


#define CGU_WKE_CLKS_64_92 ( 0 )


/***********************************************************************
* CGU driver enumerations
**********************************************************************/
/* Possible HPLL ids */
typedef enum {CGU_HPLL0_ID, CGU_HPLL1_ID} CGU_HPLL_ID_T;

/* CGU soft reset module ID enumerations */
typedef enum
{
  APB0_RST_SOFT = 0,
  AHB2APB0_PNRES_SOFT,
  APB1_RST_SOFT,
  AHB2APB1_PNRES_SOFT,
  APB2_RESETN_SOFT,
  AHB2APB2_PNRES_SOFT,
  APB3_RESETN_SOFT,
  AHB2APB3_PNRES_SOFT,
  APB4_RESETN_SOFT,
  AHB2INTC_RESETN_SOFT,
  AHB0_RESETN_SOFT,
  EBI_RESETN_SOFT,
  PCM_PNRES_SOFT,
  PCM_RESET_N_SOFT,
  PCM_RESET_ASYNC_N_SOFT,
  TIMER0_PNRES_SOFT,
  TIMER1_PNRES_SOFT,
  TIMER2_PNRES_SOFT,
  TIMER3_PNRES_SOFT,
  ADC_PRESETN_SOFT,
  ADC_RESETN_ADC10BITS_SOFT,
  PWM_RESET_AN_SOFT,
  UART_SYS_RST_AN_SOFT,
  I2C0_PNRES_SOFT,
  I2C1_PNRES_SOFT,
  I2S_CFG_RST_N_SOFT,
  I2S_NSOF_RST_N_SOFT,
  EDGE_DET_RST_N_SOFT,
  I2STX_FIFO_0_RST_N_SOFT,
  I2STX_IF_0_RST_N_SOFT,
  I2STX_FIFO_1_RST_N_SOFT,
  I2STX_IF_1_RST_N_SOFT,
  I2SRX_FIFO_0_RST_N_SOFT,
  I2SRX_IF_0_RST_N_SOFT,
  I2SRX_FIFO_1_RST_N_SOFT,
  I2SRX_IF_1_RST_N_SOFT,
  RSRVD_0_SOFT,
  RSRVD_1_SOFT,
  RSRVD_2_SOFT,
  RSRVD_3_SOFT,
  RSRVD_4_SOFT,
  LCD_PNRES_SOFT,
  SPI_PNRES_APB_SOFT,
  SPI_PNRES_IP_SOFT,
  DMA_PNRES_SOFT,
  NANDFLASH_CTRL_ECC_RESET_N_SOFT,
  NANDFLASH_CTRL_AES_RESET_N_SOFT,
  NANDFLASH_CTRL_NAND_RESET_N_SOFT,
  RNG_RESETN_SOFT,
  SD_MMC_PNRES_SOFT,
  SD_MMC_NRES_CCLK_IN_SOFT,
  USB_OTG_AHB_RST_N_SOFT,
  RED_CTL_RESET_N_SOFT,
  AHB_MPMC_HRESETN_SOFT,
  AHB_MPMC_REFRESH_RESETN_SOFT,
  INTC_RESETN_SOFT
} CGU_MOD_ID_T;

/***********************************************************************
* CGU driver structures
**********************************************************************/
/* CGU HPLL config settings structure type */
typedef struct
{
  u32 fin_select;
  u32 ndec;
  u32 mdec;
  u32 pdec;
  u32 selr;
  u32 seli;
  u32 selp;
  u32 mode;
  u32 freq; /* in MHz for driver internal data */
} CGU_HPLL_SETUP_T;

/* CGU fractional divider settings structure type */
typedef struct
{
  u8 stretch; /* Fractional divider stretch enable. */
  u8 n;       /* Fractional divider nominal nominator */
  u16 m;      /* Fractional divider nominal denominator */
} CGU_FDIV_SETUP_T;

/***********************************************************************
* CGU driver functions
**********************************************************************/
/* Return the current base frequecy of the requested domain*/
u32 cgu_get_base_freq(CGU_DOMAIN_ID_T baseid);

/* Change the base frequency for the requested domain */
void cgu_set_base_freq(CGU_DOMAIN_ID_T baseid, u32 fin_sel);

/* Return the current frequecy of the requested clock*/
u32 cgu_get_clk_freq(CGU_CLOCK_ID_T clkid);

/* Change the sub-domain frequency for the requested clock */
void cgu_set_subdomain_freq(CGU_CLOCK_ID_T clkid, CGU_FDIV_SETUP_T fdiv_cfg);

/* Configure the selected HPLL */
void cgu_hpll_config(CGU_HPLL_ID_T id, CGU_HPLL_SETUP_T* pllsetup);

/* enable / disable external enabling of the requested clock in CGU */
void cgu_clk_set_exten(CGU_CLOCK_ID_T clkid, u32 enable);

/* frac divider config function */
u32 cgu_fdiv_config(u32 fdId, CGU_FDIV_SETUP_T fdivCfg, u32 enable);

/***********************************************************************
* CGU driver inline (ANSI C99 based) functions
**********************************************************************/
/* enable / disable the requested clock in CGU */
static inline void cgu_clk_en_dis(CGU_CLOCK_ID_T clkid, u32 enable)
{
  if (enable)
  {
    CGU_SB->clk_pcr[clkid] |= CGU_SB_PCR_RUN;
  }
  else
  {
    CGU_SB->clk_pcr[clkid] &= ~CGU_SB_PCR_RUN;
  }

}
/* Issue a software reset to the requested module */
static inline void cgu_soft_reset_module(CGU_MOD_ID_T modId)
{
  volatile u32 i;

  /* clear and set the register */
  CGU_CFG->resetn_soft[modId] = 0;
  /* introduce some delay */
  for (i = 0;i < 1000;i++);

  CGU_CFG->resetn_soft[modId] = CGU_CONFIG_SOFT_RESET;
}


/***********************************************************************
* Enable/Disable frequency input to the selected base
**********************************************************************/
static inline void cgu_endis_base_freq(CGU_DOMAIN_ID_T baseid, int en)
{
	/* Let us not disturb anything except STOP */
	if (!en){
		CGU_SB->base_scr[baseid] |= CGU_SB_SCR_STOP;
	} else {
		CGU_SB->base_scr[baseid] &= ~CGU_SB_SCR_STOP;
	}
}

#endif /* LPC313X_CGU_DRIVER_H */
