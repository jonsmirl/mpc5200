/* linux/arch/arm/mach-lpc313x/usb.c -- platform level USB initialization
 *
 *  Author:	Durgesh Pattamatta
 *  Copyright (C) 2009 NXP semiconductors
 *
 * USB initialization code.
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

#undef	DEBUG

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/mod_devicetable.h>
#include <linux/usb.h>
#include <linux/usb/otg.h>
#include <linux/fsl_devices.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>

#include <asm/irq.h>
#include <asm/system.h>
#include <mach/board.h>
#include <mach/gpio.h>

/****************************************************************************
* USBOTG register definition
****************************************************************************/
#define USB_DEV_USBCMD			__REG(USBOTG_PHYS + 0x140)
#define USB_DEV_USBSTS			__REG(USBOTG_PHYS + 0x144)
#define USB_DEV_USBINTR			__REG(USBOTG_PHYS + 0x148)
#define USB_DEV_FRINDEX			__REG(USBOTG_PHYS + 0x14C)
#define USB_DEV_CONFIGFLAG              __REG(USBOTG_PHYS + 0x180)
#define USB_DEV_PORTSC1			__REG(USBOTG_PHYS + 0x184)
#define USB_DEV_OTGSC			__REG(USBOTG_PHYS + 0x1A4)
#define USB_DEV_USBMODE			__REG(USBOTG_PHYS + 0x1A8)

/* bit defines for USBCMD register */
#define USBCMD_RS	  _BIT(0)
#define USBCMD_RST	  _BIT(1)
#define USBCMD_ATDTW	  _BIT(12)
#define USBCMD_SUTW	  _BIT(13)

/* bit defines for PRTSC1 register */
#define USBPRTS_CCS	  _BIT(0)
#define USBPRTS_PE	  _BIT(2)
#define USBPRTS_FPR	  _BIT(6)
#define USBPRTS_SUSP	  _BIT(7)
#define USBPRTS_PR	  _BIT(8)
#define USBPRTS_HSP	  _BIT(9)
#define USBPRTS_PLPSCD	  _BIT(23)
#define USBPRTS_PFSC	  _BIT(24)

/* bit defines for OTGSC register */
#define OTGSC_VD          _BIT(0)
#define OTGSC_VC          _BIT(1)
#define OTGSC_HAAR        _BIT(2)
#define OTGSC_OT          _BIT(3)
#define OTGSC_DP          _BIT(4)
#define OTGSC_IDPU        _BIT(5)
#define OTGSC_HADP        _BIT(6)
#define OTGSC_HABA        _BIT(7)

#define OTGSC_ID_INT      0
#define OTGSC_AVV_INT     1
#define OTGSC_ASV_INT     2
#define OTGSC_BSV_INT     3
#define OTGSC_BSE_INT     4
#define OTGSC_1mST_INT    5
#define OTGSC_DPS_INT     6
#define OTGSC_STATUS(n)   _BIT(8 + (n))
#define OTGSC_INT_STAT(n) _BIT(16 + (n))
#define OTGSC_INT_EN(n)   _BIT(24 + (n))
#define OTGSC_INT_STAT_MASK (0x007F0000)

/*-------------------------------------------------------------------------*/
static struct resource lpc313x_usb_resource[] = {
	[0] = {
		.start = (u32) (USBOTG_PHYS),
		.end   = (u32) (USBOTG_PHYS + SZ_4K),
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_USB,
		.end   = IRQ_USB,
		.flags = IORESOURCE_IRQ,
	}
};

struct lpc313x_usb_board_t {
	/* timer for VBUS enable */
	struct timer_list	vbus_timer;
	/* board specific over current monitor */
	int	vbus_ovrc_irq;
};

static struct lpc313x_usb_board_t lpc313x_usb_brd;

static u64 usb_dmamask = 0xffffffffUL;;
static void	lpc313x_usb_release(struct device *dev);

struct fsl_usb2_platform_data lpc313x_fsl_config = {
#if defined(CONFIG_USB_OTG) || (defined(CONFIG_USB_EHCI_HCD) && defined(CONFIG_USB_GADGET_FSL_USB2))
	.operating_mode = FSL_USB2_DR_OTG,
#elif defined(CONFIG_USB_GADGET_FSL_USB2) && !defined(CONFIG_USB_EHCI_HCD)
	.operating_mode = FSL_USB2_DR_DEVICE,
#elif !defined(CONFIG_USB_GADGET_FSL_USB2) && defined(CONFIG_USB_EHCI_HCD)
	.operating_mode = FSL_USB2_DR_HOST,
#endif
	.phy_mode = FSL_USB2_PHY_UTMI,
};

#if defined(CONFIG_USB_GADGET_FSL_USB2) || defined(CONFIG_USB_OTG)

static struct platform_device lpc313x_udc_device = {
	.name = "fsl-usb2-udc",
	.dev = {
		.dma_mask          = &usb_dmamask,
		.coherent_dma_mask = 0xffffffff,
		.release           = lpc313x_usb_release,
		.platform_data     = &lpc313x_fsl_config,
	},
	.num_resources = ARRAY_SIZE(lpc313x_usb_resource),
	.resource      = lpc313x_usb_resource,
};
#endif

#if defined(CONFIG_USB_EHCI_HCD) || defined(CONFIG_USB_OTG)

static struct platform_device lpc313x_ehci_device = {
	.name		= "lpc-ehci",
	.dev = {
		.dma_mask          = &usb_dmamask,
		.coherent_dma_mask = 0xffffffff,
		.release           = lpc313x_usb_release,
		.platform_data     = &lpc313x_fsl_config,
	},
	.num_resources = ARRAY_SIZE(lpc313x_usb_resource),
	.resource      = lpc313x_usb_resource,
};
#endif


/*-------------------------------------------------------------------------*/
static void	lpc313x_usb_release(struct device *dev)
{
	// do nothing
}

static irqreturn_t lpc313x_vbus_ovrc_irq(int irq, void *data)
{
	struct lpc313x_usb_board_t* brd = data;
	/* disable VBUS power */
	lpc313x_vbus_power(0);
	/* Disable over current IRQ */
	disable_irq_nosync(irq);
	printk(KERN_INFO "Disabling VBUS as device is drawing too much current!!\n");
	printk(KERN_INFO "Please disconnect the high-power USB device!!\n");

	/* start the timer to re-enable power to VBUS and IRQ */
	mod_timer(&brd->vbus_timer, jiffies + msecs_to_jiffies(2000));

	return IRQ_HANDLED;
}

static void lpc313x_vbusen_timer(unsigned long data)
{
	struct lpc313x_usb_board_t* brd = (struct lpc313x_usb_board_t*)data;
	/* enable VBUS power */
	lpc313x_vbus_power(1);
	msleep(2);
	/* enable the VBUS overcurrent monitoring IRQ */
	enable_irq(brd->vbus_ovrc_irq);
}


/*-------------------------------------------------------------------------*/
int __init usbotg_init(void)
{
	u32 bank = EVT_GET_BANK(EVT_usb_atx_pll_lock);
	u32 bit_pos = EVT_usb_atx_pll_lock & 0x1F;
	int retval = 0;

	/* enable USB to AHB clock */
	cgu_clk_en_dis(CGU_SB_USB_OTG_AHB_CLK_ID, 1);
	/* enable clock to Event router */
	cgu_clk_en_dis(CGU_SB_EVENT_ROUTER_PCLK_ID, 1);

	/* reset USB block */
	cgu_soft_reset_module(USB_OTG_AHB_RST_N_SOFT);

	/* check if bootloader already enabled USB PLL */
	if (SYS_USB_ATX_PLL_PD_REG != 0) {
		/* enable USB OTG PLL */
		SYS_USB_ATX_PLL_PD_REG = 0x0;
		/* wait for PLL to lock */
		while (!(EVRT_RSR(bank) & _BIT(bit_pos)));
	}

	/* reset the controller */
	USB_DEV_USBCMD = USBCMD_RST;
	/* wait for reset to complete */
	while (USB_DEV_USBCMD & USBCMD_RST);

	/* enable pull-up on ID pin so that we detect external pull-downs*/
	USB_DEV_OTGSC |= OTGSC_IDPU;
	/* delay */
	udelay(5);
	
	/* check ID state */
	if ((USB_DEV_OTGSC & OTGSC_STATUS(OTGSC_ID_INT))) {
#if defined(CONFIG_USB_GADGET_FSL_USB2)
		/* register gadget */
		printk(KERN_INFO "Registering USB gadget 0x%08x 0x%08x (%d)\n", USB_DEV_OTGSC, EVRT_RSR(bank), bank);
		retval = platform_device_register(&lpc313x_udc_device);
		if ( 0 != retval )
			printk(KERN_INFO "Can't register lpc313x_udc_device device\n");
#else
		printk(KERN_ERR "Unable to register USB gadget. Check USB_ID jumper!!!!!\n");
#endif
	} else {
#if defined(CONFIG_USB_EHCI_HCD)
		/* enable VBUS power */
		lpc313x_vbus_power(1);
		msleep(2);

		/* register host */
		printk(KERN_INFO "Registering USB host 0x%08x 0x%08x (%d)\n", USB_DEV_OTGSC, EVRT_RSR(bank), bank);
		retval = platform_device_register(&lpc313x_ehci_device);
		if ( 0 != retval )
			printk(KERN_INFO "Can't register lpc313x_ehci_device device\n");

		/* Create VBUS enable timer */
		setup_timer(&lpc313x_usb_brd.vbus_timer, lpc313x_vbusen_timer,
				(unsigned long)&lpc313x_usb_brd);

#if defined(CONFIG_MACH_EA313X) || defined(CONFIG_MACH_EA3152)
		/* set thw I2SRX_WS0 pin as GPIO_IN for vbus overcurrent flag */
		retval = gpio_request(GPIO_I2SRX_WS0, "vbus overcurrent");
		if ( 0 != retval )
			printk(KERN_INFO "Can't acquire GPIO_I2SRX_WS0\n");
		gpio_direction_input(GPIO_I2SRX_WS0);
		lpc313x_usb_brd.vbus_ovrc_irq = IRQ_EA_VBUS_OVRC;

#else
		lpc313x_usb_brd.vbus_ovrc_irq = IRQ_VBUS_OVRC;
#endif

		/* request IRQ to handle VBUS power event */
		retval = request_irq( lpc313x_usb_brd.vbus_ovrc_irq, lpc313x_vbus_ovrc_irq, 
			IRQF_DISABLED, "VBUSOVR", 
			&lpc313x_usb_brd);

		if ( 0 != retval )
			printk(KERN_INFO "Unable to register IRQ_VBUS_OVRC handler\n");
		
#else
		printk(KERN_ERR "Unable to register USB host. Check USB_ID jumper!!!!!\n");
#endif
	}
	
#if !defined(CONFIG_USB_GADGET_FSL_USB2) && !defined(CONFIG_USB_OTG) && !defined(CONFIG_USB_EHCI_HCD)
	/* if no USB component is enabled power-down USB block */
	/* put in host mode */
	USB_DEV_USBMODE = 0x3;

	/* switch off PHY clock */
	USB_DEV_PORTSC1 |= 0x00000080;
	USB_DEV_PORTSC1 |= 0x00800000;

	/* power off USB PLL */
	SYS_USB_ATX_PLL_PD_REG = 1;
	/* disable USB to AHB clock */
	cgu_clk_en_dis(CGU_SB_USB_OTG_AHB_CLK_ID, 0);
#endif

	return retval;
}

arch_initcall(usbotg_init);
