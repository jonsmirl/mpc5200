/*
 * LPC313x & LPC315x EHCI Host Controller Driver
 *
 * Author: Durgesh Pattamatta <durgesh.pattamatta@nxp.com>
 *
 * Based on "ehci-fsl.c" by Randy Vinson <rvinson@mvista.com>
 *
 * 2009 (c) NXP Semiconductors. This file is licensed under
 * the terms of the GNU General Public License version 2. This program
 * is licensed "as is" without any warranty of any kind, whether express
 * or implied.
 */

#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/usb/otg.h>
#include <mach/board.h>

static struct platform_driver ehci_lpc_driver;

static int lpc_ehci_init(struct usb_hcd *hcd)
{
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	int retval = 0;

	ehci->caps = hcd->regs + 0x100;
	ehci->regs = hcd->regs + 0x100
		+ HC_LENGTH(ehci_readl(ehci, &ehci->caps->hc_capbase));
	/* cache this readonly data; minimize chip reads */
	ehci->hcs_params = ehci_readl(ehci, &ehci->caps->hcs_params);

	/* data structure init */
	retval = ehci_halt(ehci);
	if (retval)
		return retval;

	retval = ehci_init(hcd);
	if (retval)
		return retval;
	
	hcd->has_tt = 1;

	ehci->sbrn = 0x20;
	ehci_reset(ehci);

	ehci_port_power(ehci, 0);
	/* board vbus power */
	//lpc313x_vbus_power(0);

	return retval;
}

static const struct hc_driver lpc_ehci_hc_driver = {
	.description		= hcd_name,
	.product_desc		= "LPC EHCI Host Controller",
	.hcd_priv_size		= sizeof(struct ehci_hcd),
	.irq			= ehci_irq,
	.flags			= HCD_MEMORY | HCD_USB2,
	.reset			= lpc_ehci_init,
	.start			= ehci_run,
	.stop			= ehci_stop,
	.shutdown		= ehci_shutdown,
	.urb_enqueue		= ehci_urb_enqueue,
	.urb_dequeue		= ehci_urb_dequeue,
	.endpoint_disable	= ehci_endpoint_disable,
	.get_frame_number	= ehci_get_frame,
	.hub_status_data	= ehci_hub_status_data,
	.hub_control		= ehci_hub_control,
#if defined(CONFIG_PM)
	.bus_suspend		= ehci_bus_suspend,
	.bus_resume		= ehci_bus_resume,
#endif
	.relinquish_port	= ehci_relinquish_port,
	.port_handed_over	= ehci_port_handed_over,

	.clear_tt_buffer_complete = ehci_clear_tt_buffer_complete,
};

static int lpc_ehci_probe(struct platform_device *pdev)
{
	struct fsl_usb2_platform_data *pdata;
	struct usb_hcd *hcd;
	const struct hc_driver *driver = &lpc_ehci_hc_driver;
	struct resource *res;
	int irq;
	int retval;

	if (usb_disabled())
		return -ENODEV;

	/* Need platform data for setup */
	pdata = (struct fsl_usb2_platform_data *)pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev,
			"No platform data for %s.\n", dev_name(&pdev->dev));
		return -ENODEV;
	}

	/*
	 * This is a host mode driver, verify that we're supposed to be
	 * in host mode.
	 */
	if (!((pdata->operating_mode == FSL_USB2_DR_HOST) ||
	      (pdata->operating_mode == FSL_USB2_DR_OTG))) {
		dev_err(&pdev->dev,
			"Non Host Mode configured for %s. Wrong driver linked.\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}

	res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no IRQ. Check %s setup!\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}
	irq = res->start;

	hcd = usb_create_hcd(driver, &pdev->dev, dev_name(&pdev->dev));
	if (!hcd) {
		retval = -ENOMEM;
		goto fail_create_hcd;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev,
			"Found HC with no register addr. Check %s setup!\n",
			dev_name(&pdev->dev));
		retval = -ENODEV;
		goto fail_request_resource;
	}
	hcd->rsrc_start = res->start;
	hcd->rsrc_len = res->end - res->start + 1;
/*	
	if (!request_mem_region(hcd->rsrc_start, hcd->rsrc_len,
				driver->description)) {
		dev_dbg(&pdev->dev, "controller already in use\n");
		retval = -EBUSY;
		goto fail_request_resource;
	}
*/
	hcd->regs = ioremap_nocache(hcd->rsrc_start, hcd->rsrc_len);
	if (hcd->regs == NULL) {
		dev_dbg(&pdev->dev, "error mapping memory\n");
		retval = -EFAULT;
		goto fail_ioremap;
	}

	/* Set to Host mode */
	writel(USBMODE_CM_HC, (hcd->regs + 0x1a8));

	
	retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
	if (retval)
		goto fail_add_hcd;

#if defined(CONFIG_USB_OTG)
	if (pdata->operating_mode == FSL_USB2_DR_OTG) {
		struct ehci_hcd *ehci = hcd_to_ehci(hcd);

		dbg("pdev=0x%p  hcd=0x%p  ehci=0x%p\n", pdev, hcd, ehci);

		ehci->transceiver = otg_get_transceiver();

		printk(KERN_INFO "ehci->transceiver=0x%p, driver=0x%p\n", (void*)ehci->transceiver, (void*)&ehci_lpc_driver);

		if (ehci->transceiver) {
			retval = otg_set_host(ehci->transceiver,
					      &ehci_to_hcd(ehci)->self);
			if (retval) {
				if (ehci->transceiver)
					otg_put_transceiver(ehci->transceiver);
				goto fail_add_hcd;
			}
		} else {
			printk(KERN_ERR "can't find transceiver\n");
			retval = -ENODEV;
			goto fail_add_hcd;
		}
	}
#endif
	return retval;

fail_add_hcd:
	iounmap(hcd->regs);
fail_ioremap:
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
fail_request_resource:
	usb_put_hcd(hcd);
fail_create_hcd:
	dev_err(&pdev->dev, "init %s fail, %d\n", dev_name(&pdev->dev), retval);
	return retval;
}

static int lpc_ehci_remove(struct platform_device *pdev)
{
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
#if defined(CONFIG_USB_OTG)
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);

	if (ehci->transceiver) {
		(void)otg_set_host(ehci->transceiver, 0);
		otg_put_transceiver(ehci->transceiver);
	}
#endif

	usb_remove_hcd(hcd);
	iounmap(hcd->regs);
	release_mem_region(hcd->rsrc_start, hcd->rsrc_len);
	usb_put_hcd(hcd);

	return 0;
}

#ifdef CONFIG_USB_OTG
volatile static struct ehci_regs usb_ehci_regs;

/* suspend/resume, section 4.3 */

/* These routines rely on the bus (pci, platform, etc)
 * to handle powerdown and wakeup, and currently also on
 * transceivers that don't need any software attention to set up
 * the right sort of wakeup.
 *
 * They're also used for turning on/off the port when doing OTG.
 */
static int lpc_ehci_suspend(struct device *dev, pm_message_t state)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 cmd;

	dev_dbg(dev, "%s pdev=0x%p  ehci=0x%p  hcd=0x%p\n",
		 __FUNCTION__, pdev, ehci, hcd);
	dev_dbg(dev, "%s ehci->regs=0x%p  hcd->regs=0x%p  hcd->state=%d\n",
		 __FUNCTION__, ehci->regs, hcd->regs, hcd->state);

	hcd->state = HC_STATE_SUSPENDED;
	pdev->dev.power.power_state = PMSG_SUSPEND;

	/* ignore non-host interrupts */
	clear_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);

	cmd = ehci_readl(ehci, &ehci->regs->command);
	cmd &= ~CMD_RUN;
	ehci_writel(ehci, cmd, &ehci->regs->command);

	memcpy((void *)&usb_ehci_regs, ehci->regs, sizeof(struct ehci_regs));
	usb_ehci_regs.port_status[0] &=
	    cpu_to_le32(~(PORT_PEC | PORT_OCC | PORT_CSC));

	/* put the device in idele mode */
	writel(0, (hcd->regs + 0x1a8));
	/* board vbus power */
	//lpc313x_vbus_power(0);

	return 0;
}

static int lpc_ehci_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct usb_hcd *hcd = platform_get_drvdata(pdev);
	struct ehci_hcd *ehci = hcd_to_ehci(hcd);
	u32 tmp;

	dbg("%s pdev=0x%p  pdata=0x%p  ehci=0x%p  hcd=0x%p\n",
	    __FUNCTION__, pdev, pdata, ehci, hcd);

	vdbg("%s ehci->regs=0x%p  hcd->regs=0x%p  usbmode=0x%x\n",
	     __FUNCTION__, ehci->regs, hcd->regs, pdata->usbmode);

	/* Set to Host mode */
	writel(USBMODE_CM_HC, (hcd->regs + 0x1a8));

	memcpy(ehci->regs, (void *)&usb_ehci_regs, sizeof(struct ehci_regs));

	set_bit(HCD_FLAG_HW_ACCESSIBLE, &hcd->flags);
	hcd->state = HC_STATE_RUNNING;
	pdev->dev.power.power_state = PMSG_ON;

	tmp = ehci_readl(ehci, &ehci->regs->command);
	tmp |= CMD_RUN;
	ehci_writel(ehci, tmp, &ehci->regs->command);

	/* board vbus power */
	//lpc313x_vbus_power(1);


	usb_hcd_resume_root_hub(hcd);

	return 0;
}
#endif				/* CONFIG_USB_OTG */
/**
 * FIXME: This should get into a common header
 * currently declared in arch/arm/mach-lpc313x/usb.c
 **/
#define USB_DEV_PORTSC1			__REG(USBOTG_PHYS + 0x184)
#define USBPRTS_PLPSCD	_BIT(23)
static int lpc313x_ehci_suspend(struct platform_device *pdev, pm_message_t state)
{
#ifdef CONFIG_PM
	disable_irq(IRQ_VBUS_OVRC);
	/* Shutoff vbus power */
	lpc313x_vbus_power(0);
	/* Bring PHY to low power state */
	USB_DEV_PORTSC1 |= USBPRTS_PLPSCD;
	/* Bring PLL to low power state */
	SYS_USB_ATX_PLL_PD_REG = 0x1;
	/* Shutoff IP Clock */
	cgu_clk_en_dis(CGU_SB_USB_OTG_AHB_CLK_ID, 0);
#endif
	return 0;
}

static int lpc313x_ehci_resume(struct platform_device * pdev)
{
#ifdef CONFIG_PM
	u32 bank = EVT_GET_BANK(EVT_usb_atx_pll_lock);
	u32 bit_pos = EVT_usb_atx_pll_lock & 0x1F;
	int tout = 100;

	/* Turn on IP Clock */
	cgu_clk_en_dis(CGU_SB_USB_OTG_AHB_CLK_ID, 1);
	/* Bring PLL to low power state */
	SYS_USB_ATX_PLL_PD_REG = 0x0;
	/* wait for PLL to lock */
	while (!(EVRT_RSR(bank) & _BIT(bit_pos))){
		udelay(5);
		if (!tout--)
			break;
	}
	/* Bring PHY to active state */
	USB_DEV_PORTSC1 &= ~USBPRTS_PLPSCD;
	lpc313x_vbus_power(1);
	enable_irq(IRQ_VBUS_OVRC);
#endif
	return 0;
}

static struct platform_driver ehci_lpc_driver = {
	.probe = lpc_ehci_probe,
	.remove = lpc_ehci_remove,
	.suspend = lpc313x_ehci_suspend,
	.resume = lpc313x_ehci_resume,
	.driver = {
		.name = "lpc-ehci",
#ifdef CONFIG_USB_OTG
		.suspend = lpc_ehci_suspend,
		.resume  = lpc_ehci_resume,
#endif
	},
};

MODULE_ALIAS("platform:lpc-ehci");
