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

#define MAX_SAMPLES 200

struct ir_device {
	struct ir_protocol sony;
	struct ir_protocol jvc;
	struct ir_protocol nec;
	struct ir_protocol rc5;
	struct ir_protocol rc6;
	struct mutex lock;
	void *private;
	send_func xmit;
	struct input_dev *input;
	struct {
		unsigned int buffer[MAX_SAMPLES];
		unsigned int count;
	} send;
	struct {
		int buffer[MAX_SAMPLES];
		unsigned int head;
		unsigned int tail;
		unsigned int carrier;
		unsigned int xmitter;
	} raw;
	struct {
		spinlock_t lock;
		int head, tail;
		unsigned int samples[MAX_SAMPLES];
	} queue;
	struct work_struct work;
};

extern struct configfs_subsystem input_ir_remotes;
void input_ir_translate(struct input_dev *dev, int protocol, int device, int command);

#ifdef CONFIG_INPUT_IR
void input_ir_destroy(struct input_dev *dev);
#else
static inline void input_ir_destroy(struct input_dev *dev) {}
#endif



