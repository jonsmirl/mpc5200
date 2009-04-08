/*
 *   lirc_imon.c:  LIRC/VFD/LCD driver for Ahanix/Soundgraph IMON IR/VFD/LCD
 *		   including the iMON PAD model
 *
 *   Copyright(C) 2004  Venky Raju(dev@venky.ws)
 *
 *   lirc_imon is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/usb.h>

#include "lirc.h"
#include "lirc_dev.h"


#define MOD_AUTHOR	"Venky Raju <dev@venky.ws>"
#define MOD_DESC	"Driver for Soundgraph iMON MultiMedia IR/Display"
#define MOD_NAME	"lirc_imon"
#define MOD_VERSION	"0.3-jw"

#define DISPLAY_MINOR_BASE	144
#define DEVICE_NAME	"lcd%d"

#define BUF_CHUNK_SIZE	4
#define BUF_SIZE	128

#define BIT_DURATION	250	/* each bit received is 250us */

/*** P R O T O T Y P E S ***/

/* USB Callback prototypes */
static int imon_probe(struct usb_interface *interface,
		      const struct usb_device_id *id);
static void imon_disconnect(struct usb_interface *interface);
static void usb_rx_callback(struct urb *urb);
static void usb_tx_callback(struct urb *urb);

/* Display file_operations function prototypes */
static int display_open(struct inode *inode, struct file *file);
static int display_close(struct inode *inode, struct file *file);

/* VFD write operation */
static ssize_t vfd_write(struct file *file, const char *buf,
			 size_t n_bytes, loff_t *pos);

/* LCD file_operations override function prototypes */
static ssize_t lcd_write(struct file *file, const char *buf,
			 size_t n_bytes, loff_t *pos);

/* LIRC driver function prototypes */
static int ir_open(void *data);
static void ir_close(void *data);

/* Driver init/exit prototypes */
static int __init imon_init(void);
static void __exit imon_exit(void);

/*** G L O B A L S ***/

struct imon_context {
	struct usb_device *dev;
	int display_supported;		/* not all controllers do */
	int display_isopen;		/* Display port has been opened */
	int ir_isopen;			/* IR port open	*/
	int ir_isassociating;		/* IR port open for association */
	int dev_present;		/* USB device presence */
	struct mutex lock;		/* to lock this object */
	wait_queue_head_t remove_ok;	/* For unexpected USB disconnects */

	int vfd_proto_6p;		/* VFD requires 6th packet */
	int ir_onboard_decode;		/* IR signals decoded onboard */

	struct lirc_driver *driver;
	struct usb_endpoint_descriptor *rx_endpoint;
	struct usb_endpoint_descriptor *tx_endpoint;
	struct urb *rx_urb;
	struct urb *tx_urb;
	int tx_control;
	unsigned char usb_rx_buf[8];
	unsigned char usb_tx_buf[8];

	struct rx_data {
		int count;		/* length of 0 or 1 sequence */
		int prev_bit;		/* logic level of sequence */
		int initial_space;	/* initial space flag */
	} rx;

	struct tx_t {
		unsigned char data_buf[35];	/* user data buffer */
		struct completion finished;	/* wait for write to finish */
		atomic_t busy;			/* write in progress */
		int status;			/* status of tx completion */
	} tx;
};

/* Display file operations. Nb: lcd_write will be subbed in as needed later */
static struct file_operations display_fops = {
	.owner		= THIS_MODULE,
	.open		= &display_open,
	.write		= &vfd_write,
	.release	= &display_close
};

/* USB Device ID for IMON USB Control Board */
static struct usb_device_id imon_usb_id_table[] = {
	/* IMON USB Control Board (IR & VFD) */
	{ USB_DEVICE(0x0aa8, 0xffda) },
	/* IMON USB Control Board (IR only) */
	{ USB_DEVICE(0x0aa8, 0x8001) },
	/* IMON USB Control Board (IR & VFD) */
	{ USB_DEVICE(0x15c2, 0xffda) },
	/* IMON USB Control Board (IR only) */
	{ USB_DEVICE(0x15c2, 0xffdc) },
	/* IMON USB Control Board (IR & LCD) */
	{ USB_DEVICE(0x15c2, 0x0036) },
	/* IMON USB Control Board (IR & LCD) */
	{ USB_DEVICE(0x15c2, 0x0038) },
	/* IMON USB Control Board (ext IR only) */
	{ USB_DEVICE(0x04e8, 0xff30) },
	{}
};

/* Some iMON VFD models requires a 6th packet */
static struct usb_device_id vfd_proto_6p_list[] = {
	{ USB_DEVICE(0x15c2, 0xffda) },
	{ USB_DEVICE(0x15c2, 0xffdc) },
	{ USB_DEVICE(0x15c2, 0x0038) },
	{}
};
static unsigned char vfd_packet6[] = {
	0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF };

/* Some iMON LCD models use control endpoint */
static struct usb_device_id lcd_device_list[] = {
	{ USB_DEVICE(0x15c2, 0x0036) },
	{ USB_DEVICE(0x15c2, 0x0038) },
	{}
};

/* Newer iMON models decode the signal onboard */
static struct usb_device_id ir_onboard_decode_list[] = {
	{ USB_DEVICE(0x15c2, 0xffdc) },
	{ USB_DEVICE(0x15c2, 0x0036) },
	{ USB_DEVICE(0x15c2, 0x0038) },
	{}
};

/* Some iMon devices have no lcd/vfd */
static struct usb_device_id ir_only_list[] = {
	{ USB_DEVICE(0x0aa8, 0x8001) },
	/*
	 * Nb: this device ID might actually be used by multiple devices, some
	 * with a display, some without. iMon Knob has this ID, is w/o.
	 */
	{ USB_DEVICE(0x15c2, 0xffdc) },
	{}
};

/* Some iMon devices apparently need a larger buffer */
static struct usb_device_id large_buffer_list[] = {
	{ USB_DEVICE(0x15c2, 0x0036) },
	{}
};

/* USB Device data */
static struct usb_driver imon_driver = {
	.name		= MOD_NAME,
	.probe		= imon_probe,
	.disconnect	= imon_disconnect,
	.id_table	= imon_usb_id_table,
};

static struct usb_class_driver imon_class = {
	.name		= DEVICE_NAME,
	.fops		= &display_fops,
	.minor_base	= DISPLAY_MINOR_BASE,
};

/* to prevent races between open() and disconnect() */
static DEFINE_MUTEX(disconnect_lock);

static int debug;

/* lcd, vfd or none? should be auto-detected, but can be overridden... */
static int display_type;


/***  M O D U L E   C O D E ***/

MODULE_AUTHOR(MOD_AUTHOR);
MODULE_DESCRIPTION(MOD_DESC);
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(usb, imon_usb_id_table);
module_param(debug, int, 0);
MODULE_PARM_DESC(debug, "Debug messages: 0=no, 1=yes(default: no)");
module_param(display_type, int, 0);
MODULE_PARM_DESC(display_type, "Type of attached display. 0=autodetect, "
		 "1=vfd, 2=lcd, 3=none (default: autodetect)");

static void delete_context(struct imon_context *context)
{
	if (context->display_supported)
		usb_free_urb(context->tx_urb);
	usb_free_urb(context->rx_urb);
	lirc_buffer_free(context->driver->rbuf);
	kfree(context->driver->rbuf);
	kfree(context->driver);
	kfree(context);

	if (debug)
		info("%s: context deleted", __func__);
}

static void deregister_from_lirc(struct imon_context *context)
{
	int retval;
	int minor = context->driver->minor;

	retval = lirc_unregister_driver(minor);
	if (retval)
		err("%s: unable to deregister from lirc(%d)",
			__func__, retval);
	else
		info("Deregistered iMON driver(minor:%d)", minor);

}

/**
 * Called when the Display device (e.g. /dev/lcd0)
 * is opened by the application.
 */
static int display_open(struct inode *inode, struct file *file)
{
	struct usb_interface *interface;
	struct imon_context *context = NULL;
	int subminor;
	int retval = 0;

	/* prevent races with disconnect */
	mutex_lock(&disconnect_lock);

	subminor = iminor(inode);
	interface = usb_find_interface(&imon_driver, subminor);
	if (!interface) {
		err("%s: could not find interface for minor %d",
		    __func__, subminor);
		retval = -ENODEV;
		goto exit;
	}
	context = usb_get_intfdata(interface);

	if (!context) {
		err("%s: no context found for minor %d",
					__func__, subminor);
		retval = -ENODEV;
		goto exit;
	}

	mutex_lock(&context->lock);

	if (!context->display_supported) {
		err("%s: Display not supported by device", __func__);
		retval = -ENODEV;
	} else if (context->display_isopen) {
		err("%s: Display port is already open", __func__);
		retval = -EBUSY;
	} else {
		context->display_isopen = 1;
		file->private_data = context;
		info("Display port opened");
	}

	mutex_unlock(&context->lock);

exit:
	mutex_unlock(&disconnect_lock);
	return retval;
}

/**
 * Called when the Display device (e.g. /dev/lcd0)
 * is closed by the application.
 */
static int display_close(struct inode *inode, struct file *file)
{
	struct imon_context *context = NULL;
	int retval = 0;

	context = (struct imon_context *) file->private_data;

	if (!context) {
		err("%s: no context for device", __func__);
		return -ENODEV;
	}

	mutex_lock(&context->lock);

	if (!context->display_supported) {
		err("%s: Display not supported by device", __func__);
		retval = -ENODEV;
	} else if (!context->display_isopen) {
		err("%s: Display is not open", __func__);
		retval = -EIO;
	} else {
		context->display_isopen = 0;
		info("Display port closed");
		if (!context->dev_present && !context->ir_isopen) {
			/*
			 * Device disconnected before close and IR port is not
			 * open. If IR port is open, context will be deleted by
			 * ir_close.
			 */
			mutex_unlock(&context->lock);
			delete_context(context);
			return retval;
		}
	}

	mutex_unlock(&context->lock);
	return retval;
}

/**
 * Sends a packet to the Display.
 */
static int send_packet(struct imon_context *context)
{
	unsigned int pipe;
	int interval = 0;
	int retval = 0;
	struct usb_ctrlrequest *control_req = NULL;

	/* Check if we need to use control or interrupt urb */
	if (!context->tx_control) {
		pipe = usb_sndintpipe(context->dev,
				      context->tx_endpoint->bEndpointAddress);
		interval = context->tx_endpoint->bInterval;

		usb_fill_int_urb(context->tx_urb, context->dev, pipe,
				 context->usb_tx_buf,
				 sizeof(context->usb_tx_buf),
				 usb_tx_callback, context, interval);

		context->tx_urb->actual_length = 0;
	} else {
		/* fill request into kmalloc'ed space: */
		control_req = kmalloc(sizeof(struct usb_ctrlrequest),
				      GFP_KERNEL);
		if (control_req == NULL)
			return -ENOMEM;

		/* setup packet is '21 09 0200 0001 0008' */
		control_req->bRequestType = 0x21;
		control_req->bRequest = 0x09;
		control_req->wValue = cpu_to_le16(0x0002);
		control_req->wIndex = cpu_to_le16(0x0100);
		control_req->wLength = cpu_to_le16(0x0800);

		/* control pipe is endpoint 0x00 */
		pipe = usb_sndctrlpipe(context->dev, 0);

		/* build the control urb */
		usb_fill_control_urb(context->tx_urb, context->dev, pipe,
				     (unsigned char *)control_req,
				     context->usb_tx_buf,
				     sizeof(context->usb_tx_buf),
				     usb_tx_callback, context);
		context->tx_urb->actual_length = 0;
	}

	init_completion(&context->tx.finished);
	atomic_set(&(context->tx.busy), 1);

	retval = usb_submit_urb(context->tx_urb, GFP_KERNEL);
	if (retval) {
		atomic_set(&(context->tx.busy), 0);
		err("%s: error submitting urb(%d)", __func__, retval);
	} else {
		/* Wait for transmission to complete (or abort) */
		mutex_unlock(&context->lock);
		retval = wait_for_completion_interruptible(
				&context->tx.finished);
		if (retval)
			err("%s: task interrupted", __func__);
		mutex_lock(&context->lock);

		retval = context->tx.status;
		if (retval)
			err("%s: packet tx failed (%d)", __func__, retval);
	}

	kfree(control_req);

	return retval;
}

/**
 * Sends an associate packet to the iMON 2.4G.
 *
 * This might not be such a good idea, since it has an id collision with
 * some versions of the "IR & VFD" combo. The only way to determine if it
 * is an RF version is to look at the product description string. (Which
 * we currently do not fetch).
 */
static int send_associate_24g(struct imon_context *context)
{
	int retval;
	const unsigned char packet[8] = { 0x01, 0x00, 0x00, 0x00,
					  0x00, 0x00, 0x00, 0x20 };

	if (!context) {
		err("%s: no context for device", __func__);
		return -ENODEV;
	}

	mutex_lock(&context->lock);

	if (!context->dev_present) {
		err("%s: no iMON device present", __func__);
		retval = -ENODEV;
		goto exit;
	}

	memcpy(context->usb_tx_buf, packet, sizeof(packet));
	retval = send_packet(context);

exit:
	mutex_unlock(&context->lock);

	return retval;
}

/**
 * These are the sysfs functions to handle the association on the iMON 2.4G LT.
 */

static ssize_t show_associate_remote(struct device *d,
				     struct device_attribute *attr,
				     char *buf)
{
	struct imon_context *context = dev_get_drvdata(d);

	if (!context)
		return -ENODEV;

	mutex_lock(&context->lock);
	if (context->ir_isassociating) {
		strcpy(buf, "associating\n");
	} else if (context->ir_isopen) {
		strcpy(buf, "open\n");
	} else {
		strcpy(buf, "closed\n");
	}
	info("Visit http://www.lirc.org/html/imon-24g.html for instructions "
	     "on how to associate your iMON 2.4G DT/LT remote");
	mutex_unlock(&context->lock);
	return strlen(buf);
}

static ssize_t store_associate_remote(struct device *d,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct imon_context *context;

	context = dev_get_drvdata(d);

	if (!context)
		return -ENODEV;

	mutex_lock(&context->lock);
	if (!context->ir_isopen) {
		mutex_unlock(&context->lock);
		return -EINVAL;
	}

	if (context->ir_isopen) {
		context->ir_isassociating = 1;
		send_associate_24g(context);
	}
	mutex_unlock(&context->lock);

	return count;
}

static DEVICE_ATTR(associate_remote, S_IWUSR | S_IRUGO, show_associate_remote,
		   store_associate_remote);

static struct attribute *imon_sysfs_entries[] = {
	&dev_attr_associate_remote.attr,
	NULL
};

static struct attribute_group imon_attribute_group = {
	.attrs = imon_sysfs_entries
};




/**
 * Writes data to the VFD.  The IMON VFD is 2x16 characters
 * and requires data in 5 consecutive USB interrupt packets,
 * each packet but the last carrying 7 bytes.
 *
 * I don't know if the VFD board supports features such as
 * scrolling, clearing rows, blanking, etc. so at
 * the caller must provide a full screen of data.  If fewer
 * than 32 bytes are provided spaces will be appended to
 * generate a full screen.
 */
static ssize_t vfd_write(struct file *file, const char *buf,
			 size_t n_bytes, loff_t *pos)
{
	int i;
	int offset;
	int seq;
	int retval = 0;
	struct imon_context *context;

	context = (struct imon_context *) file->private_data;
	if (!context) {
		err("%s: no context for device", __func__);
		return -ENODEV;
	}

	mutex_lock(&context->lock);

	if (!context->dev_present) {
		err("%s: no iMON device present", __func__);
		retval = -ENODEV;
		goto exit;
	}

	if (n_bytes <= 0 || n_bytes > 32) {
		err("%s: invalid payload size", __func__);
		retval = -EINVAL;
		goto exit;
	}

	if (copy_from_user(context->tx.data_buf, buf, n_bytes)) {
		retval = -EFAULT;
		goto exit;
	}

	/* Pad with spaces */
	for (i = n_bytes; i < 32; ++i)
		context->tx.data_buf[i] = ' ';

	for (i = 32; i < 35; ++i)
		context->tx.data_buf[i] = 0xFF;

	offset = 0;
	seq = 0;

	do {
		memcpy(context->usb_tx_buf, context->tx.data_buf + offset, 7);
		context->usb_tx_buf[7] = (unsigned char) seq;

		retval = send_packet(context);
		if (retval) {
			err("%s: send packet failed for packet #%d",
					__func__, seq/2);
			goto exit;
		} else {
			seq += 2;
			offset += 7;
		}

	} while (offset < 35);

	if (context->vfd_proto_6p) {
		/* Send packet #6 */
		memcpy(context->usb_tx_buf, vfd_packet6, 7);
		context->usb_tx_buf[7] = (unsigned char) seq;
		retval = send_packet(context);
		if (retval)
			err("%s: send packet failed for packet #%d",
					__func__, seq/2);
	}

exit:
	mutex_unlock(&context->lock);

	return (!retval) ? n_bytes : retval;
}

/**
 * Writes data to the LCD.  The iMON OEM LCD screen excepts 8-byte
 * packets. We accept data as 16 hexadecimal digits, followed by a
 * newline (to make it easy to drive the device from a command-line
 * -- even though the actual binary data is a bit complicated).
 *
 * The device itself is not a "traditional" text-mode display. It's
 * actually a 16x96 pixel bitmap display. That means if you want to
 * display text, you've got to have your own "font" and translate the
 * text into bitmaps for display. This is really flexible (you can
 * display whatever diacritics you need, and so on), but it's also
 * a lot more complicated than most LCDs...
 */
static ssize_t lcd_write(struct file *file, const char *buf,
			 size_t n_bytes, loff_t *pos)
{
	int retval = 0;
	struct imon_context *context;

	context = (struct imon_context *) file->private_data;
	if (!context) {
		err("%s: no context for device", __func__);
		return -ENODEV;
	}

	mutex_lock(&context->lock);

	if (!context->dev_present) {
		err("%s: no iMON device present", __func__);
		retval = -ENODEV;
		goto exit;
	}

	if (n_bytes != 8) {
		err("%s: invalid payload size: %d (expecting 8)",
		    __func__, (int) n_bytes);
		retval = -EINVAL;
		goto exit;
	}

	if (copy_from_user(context->usb_tx_buf, buf, 8)) {
		retval = -EFAULT;
		goto exit;
	}

	retval = send_packet(context);
	if (retval) {
		err("%s: send packet failed!", __func__);
		goto exit;
	} else if (debug) {
		info("%s: write %d bytes to LCD", __func__, (int) n_bytes);
	}
exit:
	mutex_unlock(&context->lock);
	return (!retval) ? n_bytes : retval;
}

/**
 * Callback function for USB core API: transmit data
 */
static void usb_tx_callback(struct urb *urb)
{
	struct imon_context *context;

	if (!urb)
		return;
	context = (struct imon_context *) urb->context;
	if (!context)
		return;

	context->tx.status = urb->status;

	/* notify waiters that write has finished */
	atomic_set(&context->tx.busy, 0);
	complete(&context->tx.finished);

	return;
}

/**
 * Called by lirc_dev when the application opens /dev/lirc
 */
static int ir_open(void *data)
{
	int retval = 0;
	struct imon_context *context;

	/* prevent races with disconnect */
	mutex_lock(&disconnect_lock);

	context = (struct imon_context *) data;

	/* initial IR protocol decode variables */
	context->rx.count = 0;
	context->rx.initial_space = 1;
	context->rx.prev_bit = 0;

	usb_fill_int_urb(context->rx_urb, context->dev,
		usb_rcvintpipe(context->dev,
				context->rx_endpoint->bEndpointAddress),
		context->usb_rx_buf, sizeof(context->usb_rx_buf),
		usb_rx_callback, context, context->rx_endpoint->bInterval);

	retval = usb_submit_urb(context->rx_urb, GFP_KERNEL);

	if (retval)
		err("%s: usb_submit_urb failed for ir_open(%d)",
		    __func__, retval);
	else {
		context->ir_isopen = 1;
		info("IR port opened");
	}

	mutex_unlock(&disconnect_lock);
	return retval;
}

/**
 * Called by lirc_dev when the application closes /dev/lirc
 */
static void ir_close(void *data)
{
	struct imon_context *context;

	context = (struct imon_context *)data;
	if (!context) {
		err("%s: no context for device", __func__);
		return;
	}

	mutex_lock(&context->lock);

	usb_kill_urb(context->rx_urb);
	context->ir_isopen = 0;
	context->ir_isassociating = 0;
	info("IR port closed");

	if (!context->dev_present) {
		/*
		 * Device disconnected while IR port was still open. Driver
		 * was not deregistered at disconnect time, so do it now.
		 */
		deregister_from_lirc(context);

		if (!context->display_isopen) {
			mutex_unlock(&context->lock);
			delete_context(context);
			return;
		}
		/*
		 * If display port is open, context will be deleted by
		 * display_close
		 */
	}

	mutex_unlock(&context->lock);
	return;
}

/**
 * Convert bit count to time duration(in us) and submit
 * the value to lirc_dev.
 */
static void submit_data(struct imon_context *context)
{
	unsigned char buf[4];
	int value = context->rx.count;
	int i;

	if (debug)
		info("submitting data to LIRC\n");

	value *= BIT_DURATION;
	value &= PULSE_MASK;
	if (context->rx.prev_bit)
		value |= PULSE_BIT;

	for (i = 0; i < 4; ++i)
		buf[i] = value>>(i*8);

	lirc_buffer_write(context->driver->rbuf, buf);
	wake_up(&context->driver->rbuf->wait_poll);
	return;
}

/**
 * Process the incoming packet
 */
static void incoming_packet(struct imon_context *context,
				   struct urb *urb)
{
	int len = urb->actual_length;
	unsigned char *buf = urb->transfer_buffer;
	int octet, bit;
	unsigned char mask;
	int chunk_num;
#ifdef DEBUG
	int i;
#endif

	/* we need to add some special handling for the imon's IR mouse events */
	if ((len == 5) && (buf[0] == 0x01) && (buf[4] == 0x00)) {
		/* first, pad it to 8 bytes so it conforms with everything else */
		buf[5] = buf[6] = buf[7] = 0;
		len = 8;

		/*
		 * the imon directional pad functions more like a touchpad. Bytes 3 & 4
		 * contain a position coordinate (x,y), with each component ranging
		 * from -14 to 14. Since this doesn't cooperate well with the way lirc
		 * works (it would appear to lirc as more than 100 different buttons)
		 * we need to map it to 4 discrete values. Also, when you get too close
		 * to diagonals, it has a tendancy to jump back and forth, so lets try
		 * to ignore when they get too close
		 */
		if ((buf[1] == 0) && ((buf[2] != 0) || (buf[3] != 0))) {
			int y = (int)(char)buf[2];
			int x = (int)(char)buf[3];
			if (abs(abs(x) - abs(y)) < 3) {
				return;
			} else if (abs(y) > abs(x)) {
				buf[2] = 0x00;
				buf[3] = (y > 0) ? 0x7f : 0x80;
			} else {
				buf[3] = 0x00;
				buf[2] = (x > 0) ? 0x7f : 0x80;
			}
		}
	} 

	if (len != 8) {
		warn("%s: invalid incoming packet size(%d)",
		     __func__, len);
		return;
	}

	/* iMON 2.4G associate frame */
	if (buf[0] == 0x00 &&
	    buf[2] == 0xFF &&				/* REFID */
	    buf[3] == 0xFF &&
	    buf[4] == 0xFF &&
	    buf[5] == 0xFF &&				/* iMON 2.4G */
	   ((buf[6] == 0x4E && buf[7] == 0xDF) ||	/* LT */
	    (buf[6] == 0x5E && buf[7] == 0xDF))) {	/* DT */
		warn("%s: remote associated refid=%02X", __func__, buf[1]);
		context->ir_isassociating = 0;
	}

	chunk_num = buf[7];

	if (chunk_num == 0xFF)
		return;		/* filler frame, no data here */

	if (buf[0] == 0xFF &&
	    buf[1] == 0xFF &&
	    buf[2] == 0xFF &&
	    buf[3] == 0xFF &&
	    buf[4] == 0xFF &&
	    buf[5] == 0xFF &&				/* iMON 2.4G */
	    ((buf[6] == 0x4E && buf[7] == 0xAF) ||	/* LT */
	     (buf[6] == 0x5E && buf[7] == 0xAF)))	/* DT */
		return;		/* filler frame, no data here */

#ifdef DEBUG
	for (i = 0; i < 8; ++i)
		printk(KERN_INFO "%02x ", buf[i]);
	printk(KERN_INFO "\n");
#endif

	if (context->ir_onboard_decode) {
		/* The signals have been decoded onboard the iMON controller */
		lirc_buffer_write(context->driver->rbuf, buf);
		wake_up(&context->driver->rbuf->wait_poll);
		return;
	}

	/*
	 * Translate received data to pulse and space lengths.
	 * Received data is active low, i.e. pulses are 0 and
	 * spaces are 1.
	 *
	 * My original algorithm was essentially similar to
	 * Changwoo Ryu's with the exception that he switched
	 * the incoming bits to active high and also fed an
	 * initial space to LIRC at the start of a new sequence
	 * if the previous bit was a pulse.
	 *
	 * I've decided to adopt his algorithm.
	 */

	if (chunk_num == 1 && context->rx.initial_space) {
		/* LIRC requires a leading space */
		context->rx.prev_bit = 0;
		context->rx.count = 4;
		submit_data(context);
		context->rx.count = 0;
	}

	for (octet = 0; octet < 5; ++octet) {
		mask = 0x80;
		for (bit = 0; bit < 8; ++bit) {
			int curr_bit = !(buf[octet] & mask);
			if (curr_bit != context->rx.prev_bit) {
				if (context->rx.count) {
					submit_data(context);
					context->rx.count = 0;
				}
				context->rx.prev_bit = curr_bit;
			}
			++context->rx.count;
			mask >>= 1;
		}
	}

	if (chunk_num == 10) {
		if (context->rx.count) {
			submit_data(context);
			context->rx.count = 0;
		}
		context->rx.initial_space = context->rx.prev_bit;
	}
}

/**
 * Callback function for USB core API: receive data
 */
static void usb_rx_callback(struct urb *urb)
{
	struct imon_context *context;

	if (!urb)
		return;
	context = (struct imon_context *) urb->context;
	if (!context)
		return;

	switch (urb->status) {
	case -ENOENT:		/* usbcore unlink successful! */
		return;
	case 0:
		if (context->ir_isopen)
			incoming_packet(context, urb);
		break;
	default:
		warn("%s: status(%d): ignored", __func__, urb->status);
		break;
	}

	usb_submit_urb(context->rx_urb, GFP_ATOMIC);
	return;
}



/**
 * Callback function for USB core API: Probe
 */
static int imon_probe(struct usb_interface *interface,
		      const struct usb_device_id *id)
{
	struct usb_device *dev = NULL;
	struct usb_host_interface *iface_desc = NULL;
	struct usb_endpoint_descriptor *rx_endpoint = NULL;
	struct usb_endpoint_descriptor *tx_endpoint = NULL;
	struct urb *rx_urb = NULL;
	struct urb *tx_urb = NULL;
	struct lirc_driver *driver = NULL;
	struct lirc_buffer *rbuf = NULL;
	int lirc_minor = 0;
	int num_endpts;
	int retval = 0;
	int display_ep_found = 0;
	int ir_ep_found = 0;
	int alloc_status;
	int vfd_proto_6p = 0;
	int ir_onboard_decode = 0;
	int tx_control = 0;
	int is_lcd = 0;
	int buf_chunk_size = BUF_CHUNK_SIZE;
	struct imon_context *context = NULL;
	int i;

	info("%s: found IMON device", __func__);

	/*
	 * If it's the LCD, as opposed to the VFD, we just need to replace
	 * the "write" file op.
	 */
	if (usb_match_id(interface, lcd_device_list) || display_type == 2) {
		display_fops.write = &lcd_write;
		is_lcd = 1;
	}

	dev = usb_get_dev(interface_to_usbdev(interface));
	iface_desc = interface->cur_altsetting;
	num_endpts = iface_desc->desc.bNumEndpoints;

	/*
	 * Scan the endpoint list and set:
	 *	first input endpoint = IR endpoint
	 *	first output endpoint = Display endpoint
	 */
	for (i = 0; i < num_endpts && !(ir_ep_found && display_ep_found); ++i) {
		struct usb_endpoint_descriptor *ep;
		int ep_dir;
		int ep_type;
		ep = &iface_desc->endpoint[i].desc;
		ep_dir = ep->bEndpointAddress & USB_ENDPOINT_DIR_MASK;
		ep_type = ep->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

		if (!ir_ep_found &&
			ep_dir == USB_DIR_IN &&
			ep_type == USB_ENDPOINT_XFER_INT) {

			rx_endpoint = ep;
			ir_ep_found = 1;
			if (debug)
				info("%s: found IR endpoint", __func__);

		} else if (!display_ep_found &&
			   ep_dir == USB_DIR_OUT &&
			   ep_type == USB_ENDPOINT_XFER_INT) {
			tx_endpoint = ep;
			display_ep_found = 1;
			if (debug)
				info("%s: found display endpoint", __func__);
		}
	}

	/*
	 * If we didn't find a display endpoint, and we have a next-gen LCD,
	 * use control urb instead of interrupt
	 */
	if (!display_ep_found) {
		if (is_lcd) {
			tx_control = 1;
			display_ep_found = 1;
			if (debug)
				info("%s: LCD device uses control endpoint, "
				     "not interface OUT endpoint", __func__);
		}
	}

	/*
	 * Some iMon receivers have no display. Unfortunately, it seems
	 * that SoundGraph recycles device IDs between devices both with
	 * and without... :\
	 */
	if (usb_match_id(interface, ir_only_list) || display_type == 3) {
		tx_control = 0;
		display_ep_found = 0;
		if (debug)
			info("%s: device has no display", __func__);
	}

	/* Input endpoint is mandatory */
	if (!ir_ep_found) {
		err("%s: no valid input (IR) endpoint found.", __func__);
		retval = -ENODEV;
		goto exit;
	} else {
		/* Determine if the IR signals are decoded onboard */
		if (usb_match_id(interface, ir_onboard_decode_list))
			ir_onboard_decode = 1;

		if (debug)
			info("ir_onboard_decode: %d", ir_onboard_decode);
	}

	/* Determine if Display requires 6 packets */
	if (display_ep_found) {
		if (usb_match_id(interface, vfd_proto_6p_list))
			vfd_proto_6p = 1;

		if (debug)
			info("vfd_proto_6p: %d", vfd_proto_6p);
	}

	/* Determine if this device requires a larger buffer */
	if (usb_match_id(interface, large_buffer_list))
		buf_chunk_size = 8;

	alloc_status = 0;

	context = kzalloc(sizeof(struct imon_context), GFP_KERNEL);
	if (!context) {
		err("%s: kmalloc failed for context", __func__);
		alloc_status = 1;
		goto alloc_status_switch;
	}
	driver = kzalloc(sizeof(struct lirc_driver), GFP_KERNEL);
	if (!driver) {
		err("%s: kmalloc failed for lirc_driver", __func__);
		alloc_status = 2;
		goto alloc_status_switch;
	}
	rbuf = kmalloc(sizeof(struct lirc_buffer), GFP_KERNEL);
	if (!rbuf) {
		err("%s: kmalloc failed for lirc_buffer", __func__);
		alloc_status = 3;
		goto alloc_status_switch;
	}
	if (lirc_buffer_init(rbuf, buf_chunk_size, BUF_SIZE)) {
		err("%s: lirc_buffer_init failed", __func__);
		alloc_status = 4;
		goto alloc_status_switch;
	}
	rx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!rx_urb) {
		err("%s: usb_alloc_urb failed for IR urb", __func__);
		alloc_status = 5;
		goto alloc_status_switch;
	}
	if (display_ep_found) {
		tx_urb = usb_alloc_urb(0, GFP_KERNEL);
		if (!tx_urb) {
			err("%s: usb_alloc_urb failed for Display urb",
			    __func__);
			alloc_status = 6;
			goto alloc_status_switch;
		}
	}

	mutex_init(&context->lock);
	context->vfd_proto_6p = vfd_proto_6p;
	context->ir_onboard_decode = ir_onboard_decode;

	strcpy(driver->name, MOD_NAME);
	driver->minor = -1;
	driver->code_length = (ir_onboard_decode) ? 32 : sizeof(int) * 8;
	driver->sample_rate = 0;
	driver->features = (ir_onboard_decode) ?
		LIRC_CAN_REC_LIRCCODE : LIRC_CAN_REC_MODE2;
	driver->data = context;
	driver->rbuf = rbuf;
	driver->set_use_inc = ir_open;
	driver->set_use_dec = ir_close;
	driver->dev = &dev->dev;
	driver->owner = THIS_MODULE;

	mutex_lock(&context->lock);

	lirc_minor = lirc_register_driver(driver);
	if (lirc_minor < 0) {
		err("%s: lirc_register_driver failed", __func__);
		alloc_status = 7;
		mutex_unlock(&context->lock);
		goto alloc_status_switch;
	} else
		info("%s: Registered iMON driver(minor:%d)",
		     __func__, lirc_minor);

	/* Needed while unregistering! */
	driver->minor = lirc_minor;

	context->dev = dev;
	context->dev_present = 1;
	context->rx_endpoint = rx_endpoint;
	context->rx_urb = rx_urb;
	if (display_ep_found) {
		context->display_supported = 1;
		context->tx_endpoint = tx_endpoint;
		context->tx_urb = tx_urb;
		context->tx_control = tx_control;
	}
	context->driver = driver;

	usb_set_intfdata(interface, context);

	if (cpu_to_le16(dev->descriptor.idProduct) == 0xffdc) {
		int err;

		err = sysfs_create_group(&interface->dev.kobj,
					 &imon_attribute_group);
		if (err)
			err("%s: Could not create sysfs entries(%d)",
			    __func__, err);
	}

	if (display_ep_found && context->display_supported) {
		if (debug)
			info("Registering Display with sysfs");
		if (usb_register_dev(interface, &imon_class)) {
			/* Not a fatal error, so ignore */
			info("%s: could not get a minor number for Display",
				__func__);
		}
	}

	info("%s: iMON device on usb<%d:%d> initialized",
			__func__, dev->bus->busnum, dev->devnum);

	mutex_unlock(&context->lock);

alloc_status_switch:

	switch (alloc_status) {
	case 7:
		if (display_ep_found)
			usb_free_urb(tx_urb);
	case 6:
		usb_free_urb(rx_urb);
	case 5:
		lirc_buffer_free(rbuf);
	case 4:
		kfree(rbuf);
	case 3:
		kfree(driver);
	case 2:
		kfree(context);
		context = NULL;
	case 1:
		retval = -ENOMEM;
	case 0:
		;
	}

exit:
	return retval;
}

/**
 * Callback function for USB core API: disconnect
 */
static void imon_disconnect(struct usb_interface *interface)
{
	struct imon_context *context;

	/* prevent races with ir_open()/display_open() */
	mutex_lock(&disconnect_lock);

	context = usb_get_intfdata(interface);
	mutex_lock(&context->lock);

	/*
	 * sysfs_remove_group is safe to call even if sysfs_create_group
	 * hasn't been called
	 */
	sysfs_remove_group(&interface->dev.kobj,
			   &imon_attribute_group);
	usb_set_intfdata(interface, NULL);
	context->dev_present = 0;

	/* Stop reception */
	usb_kill_urb(context->rx_urb);

	/* Abort ongoing write */
	if (atomic_read(&context->tx.busy)) {
		usb_kill_urb(context->tx_urb);
		complete_all(&context->tx.finished);
	}

	/* De-register from lirc_dev if IR port is not open */
	if (!context->ir_isopen)
		deregister_from_lirc(context);

	if (context->display_supported)
		usb_deregister_dev(interface, &imon_class);

	mutex_unlock(&context->lock);

	if (!context->ir_isopen && !context->display_isopen)
		delete_context(context);

	mutex_unlock(&disconnect_lock);

	info("%s: iMON device disconnected", __func__);
}

static int __init imon_init(void)
{
	int rc;

	info(MOD_DESC ", v" MOD_VERSION);
	info(MOD_AUTHOR);

	rc = usb_register(&imon_driver);
	if (rc) {
		err("%s: usb register failed(%d)", __func__, rc);
		return -ENODEV;
	}
	return 0;
}

static void __exit imon_exit(void)
{
	usb_deregister(&imon_driver);
	info("module removed. Goodbye!");
}

module_init(imon_init);
module_exit(imon_exit);
