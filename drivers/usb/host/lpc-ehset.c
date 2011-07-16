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


#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/kref.h>
#include <linux/usb.h>
#include <linux/timer.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <asm/uaccess.h>
#include <linux/workqueue.h>
#include "../core/usb.h"
#include "../core/hcd.h"

#include <asm/irq.h>
#include <asm/system.h>
#include <mach/hardware.h>

/****************************************************************************
* USBOTG register definition
****************************************************************************/
#define USB_DEV_PORTSC1			__REG(USBOTG_PHYS + 0x184)

/*-------------------------------------------------------------------------*/

#define USB_PORT_TEST_DISABLE			0x00
#define USB_PORT_TEST_J				0x01
#define USB_PORT_TEST_K				0x02
#define USB_PORT_TEST_SE0_NAK			0x03
#define USB_PORT_TEST_PACKET			0x04
#define USB_PORT_TEST_FORCE_ENABLE_HS		0x05
#define USB_PORT_TEST_FORCE_ENABLE_FS		0x06
#define USB_PORT_TEST_FORCE_ENABLE_LS		0x07

/*---------------------------------------------------------------------------*/
/* This is the list of VID/PID that the HS OPT card will use. */
static struct usb_device_id hset_table [] = {
	{ USB_DEVICE(6666, 0x0101) },	/* TEST_SE0_NAK */
	{ USB_DEVICE(6666, 0x0102) },	/* TEST_J */
	{ USB_DEVICE(6666, 0x0103) },	/* TEST_K */
	{ USB_DEVICE(6666, 0x0104) },	/* TEST_PACKET */
	{ USB_DEVICE(6666, 0x0105) },	/* TEST_FORCE_ENABLE */
	{ USB_DEVICE(6666, 0x0106) },	/* HS_HOST_PORT_SUSPEND_RESUME */
	{ USB_DEVICE(6666, 0x0107) },	/* SINGLE_STEP_GET_DEV_DESC */
	{ USB_DEVICE(6666, 0x0108) },	/* SINGLE_STEP_SET_FEATURE */
	{ }				/* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, hset_table);

/* Structure to hold all of our device specific stuff */
struct usb_hset {
	struct usb_device *	udev;	/* the usb device for this device */
	struct usb_interface *	interface; /* the interface for this device */
	struct kref		kref;
	struct usb_hcd *	hcd;
};
#define to_hset_dev(d) container_of(d, struct usb_hset, kref)

static struct usb_hset *the_hset;
static struct usb_driver hset_driver;

/*---------------------------------------------------------------------------*/
/* Test routines */
static inline void test_se0_nak(struct usb_hset *hset)
{
	u32 temp = USB_DEV_PORTSC1;
	printk(KERN_INFO "%s\n", __FUNCTION__);
	
	temp &= ~(0xF << 16);
	temp |= USB_PORT_TEST_SE0_NAK << 16;
	USB_DEV_PORTSC1 = temp;
}

static inline void test_j(struct usb_hset *hset)
{
	u32 temp = USB_DEV_PORTSC1;
	printk(KERN_INFO "%s\n", __FUNCTION__);
	
	temp &= ~(0xF << 16);
	temp |= USB_PORT_TEST_J << 16;
	USB_DEV_PORTSC1 = temp;
}

static inline void test_k(struct usb_hset *hset)
{
	u32 temp = USB_DEV_PORTSC1;
	printk(KERN_INFO "%s\n", __FUNCTION__);
	
	temp &= ~(0xF << 16);
	temp |= USB_PORT_TEST_K << 16;
	USB_DEV_PORTSC1 = temp;
}

static inline void test_packet(struct usb_hset *hset)
{
	u32 temp = USB_DEV_PORTSC1;
	printk(KERN_INFO "%s\n", __FUNCTION__);
	
	temp &= ~(0xF << 16);
	temp |= USB_PORT_TEST_PACKET << 16;
	USB_DEV_PORTSC1 = temp;
}

static inline void test_force_enable(struct usb_hset *hset)
{
	u32 temp = USB_DEV_PORTSC1;
	printk(KERN_INFO "%s\n", __FUNCTION__);
	
	temp &= ~(0xF << 16);
	temp |= USB_PORT_TEST_FORCE_ENABLE_HS << 16;
	USB_DEV_PORTSC1 = temp;
}

static void suspend(struct usb_hset *hset)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	
	USB_DEV_PORTSC1 |= (1 << 7);
}

static void resume(struct usb_hset *hset)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	USB_DEV_PORTSC1 |= (1 << 6);
}

static void test_suspend_resume(struct usb_hset *hset)
{
	printk(KERN_INFO "%s\n", __FUNCTION__);
	suspend(hset);
	msleep(15000);	/* Wait for 15 sec */
	resume(hset);
}

static void test_single_step_get_dev_desc(struct usb_hset *hset)
{
	struct usb_device *udev;
	struct usb_bus *bus = hcd_to_bus(hset->hcd);

	printk(KERN_INFO "%s\n", __FUNCTION__);
	if (!bus || !bus->root_hub) {
		printk(KERN_ERR "Host controller not ready!\n");
		return;
	}
	udev = bus->root_hub->children[0];
	if (!udev) {
		printk(KERN_DEBUG "No device connected.\n");
		return;
	}
	usb_get_device_descriptor(udev, sizeof(struct usb_device_descriptor));
}

static void test_single_step_set_feature(struct usb_hset *hset)
{
	struct usb_device *udev;
	struct usb_bus *bus = hcd_to_bus(hset->hcd);

	printk(KERN_INFO "%s\n", __FUNCTION__);
	if (!bus || !bus->root_hub) {
		printk(KERN_ERR "Host controller not ready!\n");
		return;
	}

	udev = bus->root_hub->children[0];
	if (!udev) {
		printk(KERN_DEBUG "No device connected.\n");
		return;
	}
	usb_control_msg(udev,
			usb_sndctrlpipe(udev, 0),
			USB_REQ_SET_FEATURE, 0, 0,
			0, NULL, 0, HZ * USB_CTRL_SET_TIMEOUT);
}

static void enumerate_bus(struct work_struct *ignored)
{
	struct usb_hset *hset = the_hset;
	struct usb_device *udev;
	struct usb_bus *bus = hcd_to_bus(hset->hcd);

	printk(KERN_INFO "%s\n", __FUNCTION__);
	if (!bus || !bus->root_hub) {
		printk(KERN_ERR "Host controller not ready!\n");
		return;
	}
	udev = bus->root_hub->children[0];
	if (udev)
		usb_reset_device(udev);
}

DECLARE_WORK(enumerate, enumerate_bus);

/*---------------------------------------------------------------------------*/

/* This function can be called either by musb_init_hset() or usb_hset_probe().
 * musb_init_hset() is called by the controller driver during its init(),
 * while usb_hset_probe() is called when an OPT is attached. We take care not
 * to allocate the usb_hset structure twice.
 */
static struct usb_hset* init_hset_dev(void *controller)
{
	struct usb_hset *hset = NULL;

	/* if already allocated, just increment use count and return */
	if (the_hset) {
		kref_get(&the_hset->kref);
		return the_hset;
	}

	hset = kmalloc(sizeof(*hset), GFP_KERNEL);
	if (hset == NULL) {
		err("Out of memory");
		return NULL;
	}
	memset(hset, 0x00, sizeof(*hset));
	hset->hcd = (struct usb_hcd *)(controller);

	kref_init(&hset->kref);
	the_hset = hset;
	return hset;
}

static void hset_delete(struct kref *kref)
{
	struct usb_hset *dev = to_hset_dev(kref);

	kfree (dev);
}
/*---------------------------------------------------------------------------*/
/* Usage of HS OPT */


/* Called when the HS OPT is attached as a device */
static int hset_probe(struct usb_interface *interface,
		      const struct usb_device_id *id)
{
	struct usb_hset *hset;
	struct usb_device *udev;
	struct usb_hcd *hcd;
	int retval = -ENOMEM;

	udev = usb_get_dev(interface_to_usbdev(interface));
	hcd = bus_to_hcd(udev->bus);
	hset = init_hset_dev(hcd);
	if (!hset)
		return retval;

	hset->udev = udev;
	hset->interface = interface;
	usb_set_intfdata(interface, hset);

	switch(id->idProduct) {
	case 0x0101:
		test_se0_nak(hset);
		break;
	case 0x0102:
		test_j(hset);
		break;
	case 0x0103:
		test_k(hset);
		break;
	case 0x0104:
		test_packet(hset);
		break;
	case 0x0105:
		test_force_enable(hset);
		break;
	case 0x0106:
		test_suspend_resume(hset);
		break;
	case 0x0107:
		msleep(15000);	/* SOFs for 15 sec */
		test_single_step_get_dev_desc(hset);
		break;
	case 0x0108:
		test_single_step_get_dev_desc(hset);
		msleep(15000);	/* SOFs for 15 sec */
		test_single_step_set_feature(hset);
		break;
	};
	return 0;
}

static void hset_disconnect(struct usb_interface *interface)
{
	struct usb_hset *hset;

	/* prevent hset_open() from racing hset_disconnect() */
	//lock_kernel();
	hset = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);
	//unlock_kernel();

	usb_put_dev(hset->udev);
	kref_put(&hset->kref, hset_delete);
}

static struct usb_driver hset_driver = {
//	.owner =	THIS_MODULE,
	.name =		"hset",
	.probe =	hset_probe,
	.disconnect =	hset_disconnect,
	.id_table =	hset_table,
};

static int __init usb_hset_init(void)
{
	int result;

	/* register this driver with the USB subsystem */
	result = usb_register(&hset_driver);
	if (result)
		err("usb_register failed. Error number %d", result);

	return result;
}

static void __exit usb_hset_exit(void)
{
	/* deregister this driver with the USB subsystem */
	usb_deregister(&hset_driver);
}

module_init (usb_hset_init);
module_exit (usb_hset_exit);

MODULE_LICENSE("GPL");

