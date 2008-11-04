/*
 * Private defines for IR support
 *
 * Copyright (C) 2008 Jon Smirl <jonsmirl@gmail.com>
 */

#include <linux/configfs.h>

#undef IR_PROTOCOL_DEBUG
#ifdef IR_PROTOCOL_DEBUG
#define PDEBUG( format, arg... ) \
	printk(KERN_DEBUG format , ## arg);
#else
#define PDEBUG(format, arg...) \
	({ if (0) printk(KERN_DEBUG format , ## arg); 0; })
#endif

struct ir_protocol {
	unsigned int state, code, good, count, bits, mode;
};

struct ir_device {
	struct ir_protocol sony;
	struct ir_protocol jvc;
	struct ir_protocol nec;
	struct ir_protocol rc5;
	struct ir_protocol rc6;
	struct mutex lock;
	void *private;
	send_func xmit;
	struct {
		unsigned int buffer[200];
		unsigned int count;
	} send;
	struct {
		int buffer[200];
		unsigned int head;
		unsigned int tail;
		unsigned int carrier;
		unsigned int xmitter;
	} raw;
};

extern struct configfs_subsystem input_ir_remotes;
void input_ir_translate(struct input_dev *dev, int protocol, int device, int command);



