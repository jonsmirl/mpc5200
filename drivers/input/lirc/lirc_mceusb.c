/*
 * USB Microsoft IR Transceiver driver - 0.2
 *
 * Copyright (c) 2003-2004 Dan Conti (dconti@acm.wwu.edu)
 *
 * This driver is based on the USB skeleton driver packaged with the
 * kernel, and the notice from that package has been retained below.
 *
 * The Microsoft IR Transceiver is a neat little IR receiver with two
 * emitters on it designed for Windows Media Center. This driver might
 * work for all media center remotes, but I have only tested it with
 * the philips model. The first revision of this driver only supports
 * the receive function - the transmit function will be much more
 * tricky due to the nature of the hardware. Microsoft chose to build
 * this device inexpensively, therefore making it extra dumb.
 * There is no interrupt endpoint on this device; all usb traffic
 * happens over two bulk endpoints. As a result of this, poll() for
 * this device is an actual hardware poll (instead of a receive queue
 * check) and is rather expensive.
 *
 * All trademarks property of their respective owners.
 *
 * TODO
 *   - Fix up minor number, registration of major/minor with usb subsystem
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/completion.h>
#include <linux/uaccess.h>

#ifdef CONFIG_USB_DEBUG
static int debug = 1;
#else
static int debug;
#endif

#include "lirc.h"
#include "lirc_dev.h"

/* Use our own dbg macro */
#define dprintk(fmt, args...)				\
	do {						\
		if (debug)				\
			printk(KERN_DEBUG __FILE__ ": "	\
			       fmt "\n", ## args);	\
	} while (0)

/* Version Information */
#define DRIVER_VERSION "v0.2"
#define DRIVER_AUTHOR "Dan Conti, dconti@acm.wwu.edu"
#define DRIVER_DESC "USB Microsoft IR Transceiver Driver"
#define DRIVER_NAME "lirc_mceusb"

/* Define these values to match your device */
#define USB_MCEUSB_VENDOR_ID	0x045e
#define USB_MCEUSB_PRODUCT_ID	0x006d

/* table of devices that work with this driver */
static struct usb_device_id mceusb_table[] = {
	{ USB_DEVICE(USB_MCEUSB_VENDOR_ID, USB_MCEUSB_PRODUCT_ID) },
	{ }	/* Terminating entry */
};

/* we can have up to this number of device plugged in at once */
#define MAX_DEVICES		16

/* Structure to hold all of our device specific stuff */
struct mceusb_device {
	struct usb_device *udev; /* save off the usb device pointer */
	struct usb_interface *interface; /* the interface for this device */
	unsigned char minor;	 /* the starting minor number for this device */
	unsigned char num_ports; /* the number of ports this device has */
	char num_interrupt_in;	 /* number of interrupt in endpoints */
	char num_bulk_in;	 /* number of bulk in endpoints */
	char num_bulk_out;	 /* number of bulk out endpoints */

	unsigned char *bulk_in_buffer;	/* the buffer to receive data */
	int bulk_in_size;		/* the size of the receive buffer */
	__u8 bulk_in_endpointAddr;	/* the address of bulk in endpoint */

	unsigned char *bulk_out_buffer;	/* the buffer to send data */
	int bulk_out_size;		/* the size of the send buffer */
	struct urb *write_urb;		/* the urb used to send data */
	__u8 bulk_out_endpointAddr;	/* the address of bulk out endpoint */

	atomic_t write_busy;		/* true iff write urb is busy */
	struct completion write_finished; /* wait for the write to finish */

	wait_queue_head_t wait_q; /* for timeouts */
	int open_count;		/* number of times this port has been opened */
	struct mutex lock;	/* locks this structure */

	int present;		/* if the device is not disconnected */

	struct lirc_driver *driver;

	int lircdata[256]; /* place to store data until lirc processes it */
	int lircidx;		/* current index */
	int lirccnt;		/* remaining values */

	int usb_valid_bytes_in_bulk_buffer; /* leftover data from prior read */
	int mce_bytes_left_in_packet;	/* for packets split across reads */

	/* Value to hold the last received space; 0 if last value
	 * received was a pulse */
	int last_space;

	dma_addr_t dma_in;
	dma_addr_t dma_out;
};

#define MCE_TIME_UNIT 50

/* driver api */
static int mceusb_probe(struct usb_interface *interface,
			const struct usb_device_id *id);
static void mceusb_disconnect(struct usb_interface *interface);
static void mceusb_write_bulk_callback(struct urb *urb);

/* read data from the usb bus; convert to mode2 */
static int msir_fetch_more_data(struct mceusb_device *dev, int dont_block);

/* helper functions */
static void msir_cleanup(struct mceusb_device *dev);
static void set_use_dec(void *data);
static int set_use_inc(void *data);

/* array of pointers to our devices that are currently connected */
static struct mceusb_device *minor_table[MAX_DEVICES];

/* lock to protect the minor_table structure */
static DEFINE_MUTEX(minor_table_mutex);
static void mceusb_setup(struct usb_device *udev);

/* usb specific object needed to register this driver with the usb subsystem */
static struct usb_driver mceusb_driver = {
	.name		= DRIVER_NAME,
	.probe		= mceusb_probe,
	.disconnect	= mceusb_disconnect,
	.id_table	= mceusb_table,
};


static void usb_mceusb_debug_data(const char *function, int size,
					  const unsigned char *data)
{
	int i;
	if (!debug)
		return;

	printk(KERN_DEBUG __FILE__": %s - length = %d, data = ",
	       function, size);
	for (i = 0; i < size; ++i)
		printk(KERN_DEBUG "%.2x ", data[i]);
	printk(KERN_DEBUG "\n");
}

static void mceusb_delete(struct mceusb_device *dev)
{
	dprintk("%s", __func__);
	minor_table[dev->minor] = NULL;
	usb_buffer_free(dev->udev, dev->bulk_in_size,
			dev->bulk_in_buffer, dev->dma_in);
	usb_buffer_free(dev->udev, dev->bulk_out_size,
			dev->bulk_out_buffer, dev->dma_out);
	if (dev->write_urb != NULL)
		usb_free_urb(dev->write_urb);
	kfree(dev);
}

static void mceusb_setup(struct usb_device *udev)
{
	char data[8];
	int res;

	memset(data, 0, 8);

	/* Get Status */
	res = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
			      USB_REQ_GET_STATUS, USB_DIR_IN,
			      0, 0, data, 2, HZ * 3);

	/*    res = usb_get_status( udev, 0, 0, data ); */
	dprintk("%s - res = %d status = 0x%x 0x%x", __func__,
		res, data[0], data[1]);

	/*
	 * This is a strange one. They issue a set address to the device
	 * on the receive control pipe and expect a certain value pair back
	 */
	memset(data, 0, 8);

	res = usb_control_msg(udev, usb_rcvctrlpipe(udev, 0),
			      5, USB_TYPE_VENDOR, 0, 0,
			      data, 2, HZ * 3);
	dprintk("%s - res = %d, devnum = %d", __func__, res, udev->devnum);
	dprintk("%s - data[0] = %d, data[1] = %d", __func__,
		data[0], data[1]);


	/* set feature */
	res = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			      USB_REQ_SET_FEATURE, USB_TYPE_VENDOR,
			      0xc04e, 0x0000, NULL, 0, HZ * 3);

	dprintk("%s - res = %d", __func__, res);

	/*
	 * These two are sent by the windows driver, but stall for
	 * me. I don't have an analyzer on the Linux side so I can't
	 * see what is actually different and why the device takes
	 * issue with them
	 */
#if 0
	/* this is some custom control message they send */
	res = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			      0x04, USB_TYPE_VENDOR,
			      0x0808, 0x0000, NULL, 0, HZ * 3);

	dprintk("%s - res = %d", __func__, res);

	/* this is another custom control message they send */
	res = usb_control_msg(udev, usb_sndctrlpipe(udev, 0),
			      0x02, USB_TYPE_VENDOR,
			      0x0000, 0x0100, NULL, 0, HZ * 3);

	dprintk("%s - res = %d", __func__, res);
#endif
}

static void msir_cleanup(struct mceusb_device *dev)
{
	memset(dev->bulk_in_buffer, 0, dev->bulk_in_size);

	dev->usb_valid_bytes_in_bulk_buffer = 0;

	dev->last_space = PULSE_MASK;

	dev->mce_bytes_left_in_packet = 0;
	dev->lircidx = 0;
	dev->lirccnt = 0;
	memset(dev->lircdata, 0, sizeof(dev->lircdata));
}

static int set_use_inc(void *data)
{
	return 0;
}

static void set_use_dec(void *data)
{
}

/*
 * msir_fetch_more_data
 *
 * The goal here is to read in more remote codes from the remote. In
 * the event that the remote isn't sending us anything, the caller
 * will block until a key is pressed (i.e. this performs phys read,
 * filtering, and queueing of data) unless dont_block is set to 1; in
 * this situation, it will perform a few reads and will exit out if it
 * does not see any appropriate data
 *
 * dev->lock should be locked when this function is called - fine grain
 * locking isn't really important here anyways
 *
 * This routine always returns the number of words available
 *
 */
static int msir_fetch_more_data(struct mceusb_device *dev, int dont_block)
{
	int retries = 0;
	int words_to_read =
		(sizeof(dev->lircdata)/sizeof(int)) - dev->lirccnt;
	int partial, this_read = 0;
	int bulkidx = 0;
	int bytes_left_in_packet = 0;
	signed char *signedp = (signed char *)dev->bulk_in_buffer;

	if (words_to_read == 0)
		return dev->lirccnt;

	/*
	 * this forces all existing data to be read by lirc before we
	 * issue another usb command. this is the only form of
	 * throttling we have
	 */
	if (dev->lirccnt)
		return dev->lirccnt;

	/* reserve room for our leading space */
	if (dev->last_space)
		words_to_read--;

	while (words_to_read) {
		/* handle signals and USB disconnects */
		if (signal_pending(current))
			return dev->lirccnt ? dev->lirccnt : -EINTR;
		if (!dev->udev)
			return -ENODEV;

		bulkidx = 0;

		/* perform data read (phys or from previous buffer) */

		/* use leftovers if present, otherwise perform a read */
		if (dev->usb_valid_bytes_in_bulk_buffer) {
			this_read = dev->usb_valid_bytes_in_bulk_buffer;
			partial = this_read;
			dev->usb_valid_bytes_in_bulk_buffer = 0;
		} else {
			int retval;

			this_read = dev->bulk_in_size;
			partial = 0;
			retval = usb_bulk_msg(dev->udev,
					usb_rcvbulkpipe(dev->udev,
						dev->bulk_in_endpointAddr),
					(unsigned char *)dev->bulk_in_buffer,
					this_read, &partial, HZ*10);

			/*
			 * retry a few times on overruns; map all
			 * other errors to -EIO
			 */
			if (retval) {
				if (retval == -EOVERFLOW && retries < 5) {
					retries++;
					interruptible_sleep_on_timeout(
						&dev->wait_q, HZ);
					continue;
				} else
					return -EIO;
			}

			retries = 0;
			if (partial)
				this_read = partial;

			/* skip the header */
			bulkidx += 2;

			/* check for empty reads (header only) */
			if (this_read == 2) {
				/* assume no data */
				if (dont_block)
					break;

				/*
				 * sleep for a bit before performing
				 * another read
				 */
				interruptible_sleep_on_timeout(&dev->wait_q, 1);
				continue;
			}
		}

		/* process data */

		/* at this point this_read is > 0 */
		while (bulkidx < this_read &&
		       (words_to_read > (dev->last_space ? 1 : 0))) {
			/* while( bulkidx < this_read && words_to_read) */
			int keycode;
			int pulse = 0;

			/* read packet length if needed */
			if (!bytes_left_in_packet) {
				/*
				 * we assume we are on a packet length
				 * value. it is possible, in some
				 * cases, to get a packet that does
				 * not start with a length, apparently
				 * due to some sort of fragmenting,
				 * but occasionally we do not receive
				 * the second half of a fragment
				 */
				bytes_left_in_packet =
					128 + signedp[bulkidx++];

				/*
				 * unfortunately rather than keep all
				 * the data in the packetized format,
				 * the transceiver sends a trailing 8
				 * bytes that aren't part of the
				 * transmission from the remote,
				 * aren't packetized, and don't really
				 * have any value. we can basically
				 * tell we have hit them if 1) we have
				 * a loooong space currently stored
				 * up, and 2) the bytes_left value for
				 * this packet is obviously wrong
				 */
				if (bytes_left_in_packet > 4) {
					if (dev->mce_bytes_left_in_packet) {
						bytes_left_in_packet =
						  dev->mce_bytes_left_in_packet;
						bulkidx--;
					}
					bytes_left_in_packet = 0;
					bulkidx = this_read;
				}

				/*
				 * always clear this if we have a
				 * valid packet
				 */
				dev->mce_bytes_left_in_packet = 0;

				/*
				 * continue here to verify we haven't
				 * hit the end of the bulk_in
				 */
				continue;

			}

			/* generate mode2 */

			keycode = signedp[bulkidx++];
			if (keycode < 0) {
				pulse = 1;
				keycode += 128;
			}
			keycode *= MCE_TIME_UNIT;

			bytes_left_in_packet--;

			if (pulse) {
				if (dev->last_space) {
					dev->lircdata[dev->lirccnt++] =
						dev->last_space;
					dev->last_space = 0;
					words_to_read--;

					/* clear for the pulse */
					dev->lircdata[dev->lirccnt] = 0;
				}
				dev->lircdata[dev->lirccnt] += keycode;
				dev->lircdata[dev->lirccnt] |= PULSE_BIT;
			} else {
				/*
				 * on pulse->space transition, add one
				 * for the existing pulse
				 */
				if (dev->lircdata[dev->lirccnt] &&
				    !dev->last_space) {
					dev->lirccnt++;
					words_to_read--;
				}

				dev->last_space += keycode;
			}
		}
	}

	/* save off some info if we're exiting mid-packet, or with leftovers */
	if (bytes_left_in_packet)
		dev->mce_bytes_left_in_packet = bytes_left_in_packet;
	if (bulkidx < this_read) {
		dev->usb_valid_bytes_in_bulk_buffer = (this_read - bulkidx);
		memcpy(dev->bulk_in_buffer, &(dev->bulk_in_buffer[bulkidx]),
		       dev->usb_valid_bytes_in_bulk_buffer);
	}
	return dev->lirccnt;
}

/**
 * mceusb_add_to_buf: called by lirc_dev to fetch all available keys
 * this is used as a polling interface for us: since we set
 * driver->sample_rate we will periodically get the below call to
 * check for new data returns 0 on success, or -ENODATA if nothing is
 * available
 */
static int mceusb_add_to_buf(void *data, struct lirc_buffer *buf)
{
	struct mceusb_device *dev = (struct mceusb_device *) data;

	mutex_lock(&dev->lock);

	/* verify device still present */
	if (dev->udev == NULL) {
		mutex_unlock(&dev->lock);
		return -ENODEV;
	}

	if (!dev->lirccnt) {
		int res;
		dev->lircidx = 0;

		res = msir_fetch_more_data(dev, 1);

		if (res == 0)
			res = -ENODATA;
		if (res < 0) {
			mutex_unlock(&dev->lock);
			return res;
		}
	}

	if (dev->lirccnt) {
		while (dev->lirccnt && lirc_buffer_available(buf)) {
			lirc_buffer_write(buf,
			    (unsigned char *) &(dev->lircdata[dev->lircidx]));
			dev->lircidx++;
			dev->lirccnt--;
		}
		mutex_unlock(&dev->lock);
		return 0;
	}

	mutex_unlock(&dev->lock);
	return -ENODATA;
}

static void mceusb_write_bulk_callback(struct urb *urb)
{
	struct mceusb_device *dev = (struct mceusb_device *)urb->context;

	dprintk("%s - minor %d", __func__, dev->minor);

	if ((urb->status != -ENOENT) &&
	    (urb->status != -ECONNRESET)) {
		dprintk("%s - nonzero write buld status received: %d",
			__func__, urb->status);
		return;
	}

	return;
}

/**
 * mceusb_probe
 *
 * Called by the usb core when a new device is connected that it
 * thinks this driver might be interested in.
 */
static int mceusb_probe(struct usb_interface *interface,
			const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(interface);
	struct usb_host_interface *iface_desc;
	struct mceusb_device *dev = NULL;
	struct usb_endpoint_descriptor *endpoint;

	struct lirc_driver *driver;

	int minor;
	size_t buffer_size;
	int i;
	int retval = -ENOMEM;
	char junk[64];
	int partial = 0;

	/* See if the device offered us matches what we can accept */
	if (cpu_to_le16(udev->descriptor.idVendor) != USB_MCEUSB_VENDOR_ID ||
	    cpu_to_le16(udev->descriptor.idProduct) != USB_MCEUSB_PRODUCT_ID) {
		dprintk("Wrong Vendor/Product IDs");
		return -ENODEV;
	}

	/* select a "subminor" number (part of a minor number) */
	mutex_lock(&minor_table_mutex);
	for (minor = 0; minor < MAX_DEVICES; ++minor) {
		if (minor_table[minor] == NULL)
			break;
	}
	if (minor >= MAX_DEVICES) {
		info("Too many devices plugged in, "
		     "can not handle this device.");
		goto error;
	}

	/* allocate memory for our device state and initialize it */
	dev = kzalloc(sizeof(struct mceusb_device), GFP_KERNEL);
	if (dev == NULL) {
		err("Out of memory");
		retval = -ENOMEM;
		goto error;
	}
	minor_table[minor] = dev;

	mutex_init(&dev->lock);
	dev->udev = udev;
	dev->interface = interface;
	dev->minor = minor;

	/*
	 * set up the endpoint information, check out the endpoints.
	 * use only the first bulk-in and bulk-out endpoints
	 */
	iface_desc = interface->cur_altsetting;

	for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i) {
		endpoint = &iface_desc->endpoint[i].desc;
		if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) &&
		    ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		     USB_ENDPOINT_XFER_BULK)) {
			dprintk("we found a bulk in endpoint");
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_in_size = buffer_size;
			dev->bulk_in_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_in_buffer =
				usb_buffer_alloc(udev, buffer_size,
						 GFP_ATOMIC, &dev->dma_in);
			if (!dev->bulk_in_buffer) {
				err("Couldn't allocate bulk_in_buffer");
				goto error;
			}
		}

		if (((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK)
		    == 0x00)
		    && ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) ==
		    USB_ENDPOINT_XFER_BULK)) {
			dprintk("we found a bulk out endpoint");
			dev->write_urb = usb_alloc_urb(0, GFP_KERNEL);
			if (!dev->write_urb) {
				err("No free urbs available");
				goto error;
			}
			buffer_size = endpoint->wMaxPacketSize;
			dev->bulk_out_size = buffer_size;
			dev->bulk_out_endpointAddr = endpoint->bEndpointAddress;
			dev->bulk_out_buffer =
				usb_buffer_alloc(udev, buffer_size,
						 GFP_ATOMIC, &dev->dma_out);
			if (!dev->bulk_out_buffer) {
				err("Couldn't allocate bulk_out_buffer");
				goto error;
			}
			usb_fill_bulk_urb(dev->write_urb, udev,
				      usb_sndbulkpipe
				      (udev, endpoint->bEndpointAddress),
				      dev->bulk_out_buffer, buffer_size,
				      mceusb_write_bulk_callback, dev);
		}
	}

	if (!(dev->bulk_in_endpointAddr && dev->bulk_out_endpointAddr)) {
		err("Couldn't find both bulk-in and bulk-out endpoints");
		goto error;
	}

	/* init the waitq */
	init_waitqueue_head(&dev->wait_q);


	/* Set up our lirc driver */
	driver = kzalloc(sizeof(struct lirc_driver), GFP_KERNEL);
	if (!driver) {
		err("out of memory");
		goto error;
	}

	strcpy(driver->name, DRIVER_NAME " ");
	driver->minor = minor;
	driver->code_length = sizeof(int) * 8;
	driver->features = LIRC_CAN_REC_MODE2; /* | LIRC_CAN_SEND_MODE2; */
	driver->data = dev;
	driver->chunk_size = sizeof(int);
	driver->buffer_size = 128;
	driver->set_use_inc = &set_use_inc;
	driver->set_use_dec = &set_use_dec;
	driver->sample_rate = 80;   /* sample at 100hz (10ms) */
	driver->add_to_buf = &mceusb_add_to_buf;
	driver->dev = &udev->dev;
	driver->owner = THIS_MODULE;
	if (lirc_register_driver(driver) < 0) {
		kfree(driver);
		goto error;
	}
	dev->driver = driver;

	/*
	 * clear off the first few messages. these look like
	 * calibration or test data, i can't really tell
	 * this also flushes in case we have random ir data queued up
	 */
	for (i = 0; i < 40; i++)
		(void) usb_bulk_msg(udev,
				    usb_rcvbulkpipe(udev,
						    dev->bulk_in_endpointAddr),
				    junk, 64, &partial, HZ*10);

	msir_cleanup(dev);
	mceusb_setup(udev);

	/* we can register the device now, as it is ready */
	usb_set_intfdata(interface, dev);
	/* let the user know what node this device is now attached to */
	mutex_unlock(&minor_table_mutex);
	return 0;
error:
	if (likely(dev))
		mceusb_delete(dev);

	dev = NULL;
	dprintk("%s: retval = %x", __func__, retval);
	mutex_unlock(&minor_table_mutex);
	return retval;
}

/**
 * mceusb_disconnect
 *
 * Called by the usb core when the device is removed from the system.
 *
 * This routine guarantees that the driver will not submit any more urbs
 * by clearing dev->udev.  It is also supposed to terminate any currently
 * active urbs.  Unfortunately, usb_bulk_msg(), used in skel_read(), does
 * not provide any way to do this.  But at least we can cancel an active
 * write.
 */
static void mceusb_disconnect(struct usb_interface *interface)
{
	struct mceusb_device *dev;
	int minor;
	dev = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);

	mutex_lock(&minor_table_mutex);
	mutex_lock(&dev->lock);
	minor = dev->minor;

	/* unhook lirc things */
	lirc_unregister_driver(minor);
	kfree(dev->driver);
	/* terminate an ongoing write */
	if (atomic_read(&dev->write_busy)) {
		usb_kill_urb(dev->write_urb);
		wait_for_completion(&dev->write_finished);
	}

	/* prevent device read, write and ioctl */
	dev->present = 0;

	mutex_unlock(&dev->lock);
	mceusb_delete(dev);

	info("Microsoft IR Transceiver #%d now disconnected", minor);
	mutex_unlock(&minor_table_mutex);
}



static int __init usb_mceusb_init(void)
{
	int result;

	/* register this driver with the USB subsystem */
	result = usb_register(&mceusb_driver);
	if (result) {
		err("usb_register failed for the " DRIVER_NAME
		    " driver. error number %d", result);
		return result;
	}

	info(DRIVER_DESC " " DRIVER_VERSION);
	return 0;
}


static void __exit usb_mceusb_exit(void)
{
	usb_deregister(&mceusb_driver);
}

module_init(usb_mceusb_init);
module_exit(usb_mceusb_exit);

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_LICENSE("GPL");
MODULE_DEVICE_TABLE(usb, mceusb_table);

module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "Debug enabled or not");
