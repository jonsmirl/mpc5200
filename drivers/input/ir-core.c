/*
 * Core routines for IR support
 *
 * Copyright (C) 2008 Jon Smirl <jonsmirl@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/input.h>
#include <linux/configfs.h>

#undef IR_PROTOCOL_DEBUG
#ifdef IR_PROTOCOL_DEBUG
#define PDEBUG( format, arg... ) \
	printk(KERN_DEBUG format , ## arg);
#else
#define PDEBUG(format, arg...) \
	({ if (0) printk(KERN_DEBUG format , ## arg); 0; })
#endif

static int encode_sony(struct ir_device *ir, struct ir_command *command)
{
	/* Sony SIRC IR code */
	/* http://www.sbprojects.com/knowledge/ir/sirc.htm */
	int i, bit, dev, cmd, total;

	ir->send.count = 0;
	switch (command->protocol) {
	case IR_PROTOCOL_SONY_20:
		dev = 10; cmd = 10; break;
	case IR_PROTOCOL_SONY_15:
		dev = 8; cmd = 7; break;
	default:
	case IR_PROTOCOL_SONY_12:
		dev = 5; cmd = 7; break;
	}
	ir->send.buffer[ir->send.count++] = 2400;
	ir->send.buffer[ir->send.count++] = 600;

	for (i = 0; i < cmd; i++) {
		bit = command->command & 1;
		command->command >>= 1;
		ir->send.buffer[ir->send.count++] = (bit ? 1200 : 600);
		ir->send.buffer[ir->send.count++] = 600;
	}
	for (i = 0; i < dev; i++) {
		bit = command->device & 1;
		command->device >>= 1;
		ir->send.buffer[ir->send.count++] = (bit ? 1200 : 600);
		ir->send.buffer[ir->send.count++] = 600;
	}
	total = 0;
	for (i = 0; i < ir->send.count; i++)
		total += ir->send.buffer[i];
	ir->send.buffer[ir->send.count++] = 45000 - total;

	memcpy(&ir->send.buffer[ir->send.count], &ir->send.buffer[0], ir->send.count * sizeof ir->send.buffer[0]);
	ir->send.count += ir->send.count;
	memcpy(&ir->send.buffer[ir->send.count], &ir->send.buffer[0], ir->send.count * sizeof ir->send.buffer[0]);
	ir->send.count += ir->send.count;

	return 0;
}

static int decode_sony(struct input_dev *dev, struct ir_protocol *sony, unsigned int d, unsigned int bit)
{
	/* Sony SIRC IR code */
	/* http://www.sbprojects.com/knowledge/ir/sirc.htm */
	/* based on a 600us cadence */
	int ret = 0, delta = d;

	delta = (delta + 300) / 600;

	if ((bit == 0) && (delta > 22)) {
		PDEBUG("SIRC state 1\n");
		if ((sony->state == 26) || (sony->state == 32) || (sony->state == 42)) {
			if (sony->good && (sony->good == sony->code)) {

				input_report_ir(dev, IR_PROTOCOL, (sony->state == 26) ? IR_PROTOCOL_SONY_12 :
						(sony->state == 32) ? IR_PROTOCOL_SONY_15 : IR_PROTOCOL_SONY_20);

				if (sony->state == 26) {
					input_report_ir(dev, IR_DEVICE, sony->code & 0x1F);
					input_report_ir(dev, IR_COMMAND, sony->code >> 5);
				} else {
					input_report_ir(dev, IR_DEVICE, sony->code & 0xFF);
					input_report_ir(dev, IR_COMMAND, sony->code >> 8);
				}
				input_sync(dev);

				sony->good = 0;
				ret = 1;
			} else {
				PDEBUG("SIRC - Saving %d bit %05x\n", (sony->state - 2) / 2, sony->code);
				sony->good = sony->code;
			}
		}
		sony->state = 1;
		sony->code = 0;
		return ret;
	}
	if ((sony->state == 1) && (bit == 1) && (delta == 4)) {
		sony->state = 2;
		PDEBUG("SIRC state 2\n");
		return 0;
	}
	if ((sony->state == 2) && (bit == 0) && (delta == 1)) {
		sony->state = 3;
		PDEBUG("SIRC state 3\n");
		return 0;
	}
	if ((sony->state >= 3) && (sony->state & 1) && (bit == 1) && ((delta == 1) || (delta == 2))) {
		sony->state++;
		sony->code |= ((delta - 1) << ((sony->state - 4) / 2));
		PDEBUG("SIRC state %d bit %d\n", sony->state, delta - 1);
		return 0;
	}
	if ((sony->state >= 3) && !(sony->state & 1) && (bit == 0) && (delta == 1)) {
		sony->state++;
		PDEBUG("SIRC state %d\n", sony-> state);
		return 0;
	}
	sony->state = 0;
	return 0;
}


static int encode_jvc(struct ir_device *ir, struct ir_command *command)
{
	/* JVC IR code */
	/* http://www.sbprojects.com/knowledge/ir/jvc.htm */
	int i, bit, total;

	ir->send.count = 0;

	ir->send.buffer[ir->send.count++] = 8400;
	ir->send.buffer[ir->send.count++] = 4200;

	for (i = 0; i < 8; i++) {
		bit = command->device & 1;
		command->device >>= 1;
		ir->send.buffer[ir->send.count++] = 525;
		ir->send.buffer[ir->send.count++] = (bit ? 1575 : 525);
	}
	for (i = 0; i < 8; i++) {
		bit = command->command & 1;
		command->command >>= 1;
		ir->send.buffer[ir->send.count++] = 525;
		ir->send.buffer[ir->send.count++] = (bit ? 1575 : 525);
	}
	ir->send.buffer[ir->send.count++] = 525;

	total = 0;
	for (i = 0; i < ir->send.count; i++)
		total += ir->send.buffer[i];
	ir->send.buffer[ir->send.count] = 55000 - total;

	return 0;
}

static int decode_jvc(struct input_dev *dev, struct ir_protocol *jvc, unsigned int d, unsigned int bit)
{
	/* JVC IR code */
	/* http://www.sbprojects.com/knowledge/ir/jvc.htm */
	/* based on a 525us cadence */
	int ret = 0, delta = d;

	delta = (delta + 263) / 525;

	if ((bit == 0) && (delta > 22)) {
		PDEBUG("JVC state 1\n");
		jvc->state = 1;
		jvc->code = 0;
		return ret;
	}
	if ((jvc->state == 1) && (bit == 1) && (delta == 16)) {
		jvc->state = 2;
		PDEBUG("JVC state 2\n");
		return 0;
	}
	if ((jvc->state == 2) && (bit == 0) && (delta == 8)) {
		jvc->state = 3;
		PDEBUG("JVC state 3\n");
		return 0;
	}
	if ((jvc->state >= 3) && (jvc->state & 1) && (bit == 1) && (delta == 1)) {
		jvc->state++;
		PDEBUG("JVC state %d\n", jvc-> state);
		return 0;
	}
	if ((jvc->state >= 3) && !(jvc->state & 1) && (bit == 0) && ((delta == 1) || (delta == 3))) {
		if (delta == 3)
			jvc->code |= 1 << ((jvc->state - 4) / 2);
		jvc->state++;
		PDEBUG("JVC state %d bit %d\n", jvc->state, delta - 1);
		if (jvc->state == 34) {
			jvc->state = 3;
			if (jvc->good && (jvc->good == jvc->code)) {
				input_report_ir(dev, IR_PROTOCOL, IR_PROTOCOL_JVC);
				input_report_ir(dev, IR_DEVICE, jvc->code >> 8);
				input_report_ir(dev, IR_COMMAND, jvc->code & 0xFF);
				input_sync(dev);
				jvc->good = 0;
				ret = 1;
			} else {
				PDEBUG("JVC - Saving 16 bit %05x\n", jvc->code);
				jvc->good = jvc->code;
			}
			jvc->code = 0;
		}
		return 0;
	}
	jvc->state = 0;
	return 0;
}


static int encode_nec(struct ir_device *ir, struct ir_command *command)
{
	/* NEC IR code */
	/* http://www.sbprojects.com/knowledge/ir/nec.htm */
	int i, bit, total;

	ir->send.count = 0;

	ir->send.buffer[ir->send.count++] = 9000;
	ir->send.buffer[ir->send.count++] = 4500;

	for (i = 0; i < 8; i++) {
		bit = command->device & 1;
		command->device >>= 1;
		ir->send.buffer[ir->send.count++] = 563;
		ir->send.buffer[ir->send.count++] = (bit ? 1687 : 562);
	}
	for (i = 0; i < 8; i++) {
		bit = command->command & 1;
		command->command >>= 1;
		ir->send.buffer[ir->send.count++] = 563;
		ir->send.buffer[ir->send.count++] = (bit ? 1687 : 562);
	}
	ir->send.buffer[ir->send.count++] = 562;

	total = 0;
	for (i = 0; i < ir->send.count; i++)
		total += ir->send.buffer[i];
	ir->send.buffer[ir->send.count] = 110000 - total;

	return 0;
}

static int decode_nec(struct input_dev *dev, struct ir_protocol *nec, unsigned int d, unsigned int bit)
{
	/* NEC IR code */
	/* http://www.sbprojects.com/knowledge/ir/nec.htm */
	/* based on a 560us cadence */
	int delta = d;

	delta = (delta + 280) / 560;

	if ((bit == 0) && (delta > 22)) {
		PDEBUG("nec state 1\n");
		nec->state = 1;
		nec->code = 0;
		return 0;
	}
	if ((nec->state == 1) && (bit == 1) && (delta == 16)) {
		nec->state = 2;
		PDEBUG("nec state 2\n");
		return 0;
	}
	if ((nec->state == 2) && (bit == 0) && (delta == 8)) {
		nec->state = 3;
		PDEBUG("nec state 3\n");
		return 0;
	}
	if ((nec->state >= 3) && (nec->state & 1) && (bit == 1) && (delta == 1)) {
		nec->state++;
		PDEBUG("nec state %d\n", nec-> state);
		if (nec->state == 68) {
			input_report_ir(dev, IR_PROTOCOL, IR_PROTOCOL_NEC);
			input_report_ir(dev, IR_DEVICE, nec->code >> 16);
			input_report_ir(dev, IR_COMMAND, nec->code & 0xFFFF);
			input_sync(dev);
			return 1;
		}
		return 0;
	}
	if ((nec->state >= 3) && !(nec->state & 1) && (bit == 0) && ((delta == 1) || (delta == 3))) {
		if (delta == 3)
			nec->code |= 1 << ((nec->state - 4) / 2);
		nec->state++;
		PDEBUG("nec state %d bit %d\n", nec->state, delta - 1);
		return 0;
	}
	nec->state = 0;
	nec->code = 0;
	return 0;
}


static int encode_rc5(struct ir_device *ir, struct ir_command *command)
{
	/* Philips RC-5 IR code */
	/* http://www.sbprojects.com/knowledge/ir/rc5.htm */
	return 0;
}

static int decode_rc5(struct input_dev *dev, struct ir_protocol *rc5, unsigned int d, unsigned int bit)
{
	/* Philips RC-5 IR code */
	/* http://www.sbprojects.com/knowledge/ir/rc5.htm */
	/* based on a 889us cadence */
	int delta = d;

	delta = (delta + 444) / 889;

	return 0;
}


static int encode_rc6(struct ir_device *ir, struct ir_command *command)
{
	/* Philips RC-6 IR code */
	/* http://www.sbprojects.com/knowledge/ir/rc6.htm */
	int i, bit, last;

	ir->send.count = 0;

	ir->send.buffer[ir->send.count++] = 2666;
	ir->send.buffer[ir->send.count++] = 889;

	ir->send.buffer[ir->send.count++] = 444;
	ir->send.buffer[ir->send.count++] = 444;

	last = 1;
	for (i = 0; i < 8; i++) {
		bit = command->device & 1;
		command->device >>= 1;

		if (last != bit)
			ir->send.buffer[ir->send.count - 1] += 444;
		else
			ir->send.buffer[ir->send.count++] = 444;
		ir->send.buffer[ir->send.count++] = 444;
		last = bit;
	}
	for (i = 0; i < 8; i++) {
		bit = command->command & 1;
		command->command >>= 1;

		if (last != bit)
			ir->send.buffer[ir->send.count - 1] += 444;
		else
			ir->send.buffer[ir->send.count++] = 444;
		ir->send.buffer[ir->send.count++] = 444;
		last = bit;
	}
	ir->send.buffer[ir->send.count] = 2666;

	return 0;
}

static void decode_rc6_bit(struct input_dev *dev, struct ir_protocol *rc6, unsigned int bit)
{
	/* bits come in one at a time */
	/* when two are collected look for a symbol */
	/* rc6->bits == 1 is a zero symbol */
	/* rc6->bits == 2 is a one symbol */
	rc6->count++;
	rc6->bits <<= 1;
	rc6->bits |= bit;
	if (rc6->count == 2) {
		if ((rc6->bits == 0) || (rc6->bits == 3)) {
			rc6->mode = rc6->code;
			rc6->code = 0;
		} else {
			rc6->code <<= 1;
			if (rc6->bits == 2)
				rc6->code |= 1;
		}
		rc6->count = 0;
		if (rc6->state == 23) {
			input_report_ir(dev, IR_PROTOCOL, IR_PROTOCOL_PHILIPS_RC6);
			input_report_ir(dev, IR_DEVICE, rc6->code >> 8);
			input_report_ir(dev, IR_COMMAND, rc6->code & 0xFF);
			input_sync(dev);
			rc6->state = 0;
		} else
			rc6->state++;
		PDEBUG("rc6 state %d bit %d\n", rc6->state, rc6->bits == 2);
		rc6->bits = 0;
	}
}

static int decode_rc6(struct input_dev *dev, struct ir_protocol *rc6, unsigned int d, unsigned int bit)
{
	/* Philips RC-6 IR code */
	/* http://www.sbprojects.com/knowledge/ir/rc6.htm */
	/* based on a 444us cadence */

	int delta = d;

	delta = (delta + 222) / 444;

	if ((bit == 0) && (delta > 19)) {
		rc6->count = 0;
		rc6->bits = 0;
		rc6->state = 1;
		rc6->code = 0;
		PDEBUG("rc6 state 1\n");
		return 0;
	}
	if ((rc6->state == 1) && (bit == 1) && (delta == 6)) {
		rc6->state = 2;
		PDEBUG("rc6 state 2\n");
		return 0;
	}
	if ((rc6->state == 2) && (bit == 0) && (delta == 2)) {
		rc6->state = 3;
		PDEBUG("rc6 state 3\n");
		return 0;
	}
	if (rc6->state >= 3) {
		if ((delta >= 1) || (delta <= 3)) {
			while (delta-- >= 1)
				decode_rc6_bit(dev, rc6, bit);
			return 0;
		}
	}
	rc6->state = 0;
	rc6->code = 0;
	return 0;
}

static void record_raw(struct input_dev *dev, unsigned int delta, unsigned int bit)
{
	int head = dev->ir->raw.head;
	if (bit)
		delta = -delta;

	head += 1;
	if (head > ARRAY_SIZE(dev->ir->raw.buffer))
		head = 0;

	if (head != dev->ir->raw.tail) {
		dev->ir->raw.buffer[dev->ir->raw.head] = delta;
		dev->ir->raw.head = head;
	}
}

void input_ir_decode(struct input_dev *dev, unsigned int delta, unsigned int bit)
{
	PDEBUG("IR bit %d %d\n", delta, bit);
	record_raw(dev, delta, bit);
	decode_sony(dev, &dev->ir->sony, delta, bit);
	decode_jvc(dev, &dev->ir->jvc, delta, bit);
	decode_nec(dev, &dev->ir->nec, delta, bit);
	decode_rc5(dev, &dev->ir->rc5, delta, bit);
	decode_rc6(dev, &dev->ir->rc6, delta, bit);
}
EXPORT_SYMBOL_GPL(input_ir_decode);

struct mapping {
	struct config_item item;
	int protocol;
	int device;
	int command;
};

static inline struct mapping *to_mapping(struct config_item *item)
{
	return item ? container_of(item, struct mapping, item) : NULL;
}

static void mapping_release(struct config_item *item)
{
	kfree(to_mapping(item));
}

static ssize_t mapping_attr_show(struct config_item *item,
				      struct configfs_attribute *attr,
				      char *page)
{
	ssize_t count;
	struct mapping *mapping = to_mapping(item);

	count = sprintf(page, "%d\n", mapping->protocol);

	return count;
}

static ssize_t mapping_attr_store(struct config_item *item,
				       struct configfs_attribute *attr,
				       const char *page, size_t count)
{
	struct mapping *mapping = to_mapping(item);
	unsigned long tmp;
	char *p = (char *) page;

	tmp = simple_strtoul(p, &p, 10);
	if (!p || (*p && (*p != '\n')))
		return -EINVAL;

	if (tmp > INT_MAX)
		return -ERANGE;

	mapping->protocol = tmp;

	return count;
}

static struct configfs_item_operations mapping_item_ops = {
	.release		= mapping_release,
	.show_attribute		= mapping_attr_show,
	.store_attribute	= mapping_attr_store,
};

static struct configfs_attribute mapping_attr_protocol = {
	.ca_owner = THIS_MODULE,
	.ca_name = "protocol",
	.ca_mode = S_IRUGO | S_IWUSR,
};

static struct configfs_attribute *mapping_attrs[] = {
	&mapping_attr_protocol,
	NULL,
};

static struct config_item_type mapping_type = {
	.ct_item_ops	= &mapping_item_ops,
	.ct_attrs	= mapping_attrs,
	.ct_owner	= THIS_MODULE,
};

struct remote_map {
	struct config_group group;
};

static inline struct remote_map *to_remote_map(struct config_item *item)
{
	return item ? container_of(to_config_group(item), struct remote_map, group) : NULL;
}

static struct config_item *remote_map_make_item(struct config_group *group, const char *name)
{
	struct mapping *mapping;

	mapping = kzalloc(sizeof(struct mapping), GFP_KERNEL);
	if (!mapping)
		return ERR_PTR(-ENOMEM);

	config_item_init_type_name(&mapping->item, name,
				   &mapping_type);

	mapping->protocol = 0;

	return &mapping->item;
}

static void remote_map_release(struct config_item *item)
{
	kfree(to_remote_map(item));
}

static ssize_t remote_map_attr_show(struct config_item *item,
					 struct configfs_attribute *attr,
					 char *page)
{
	return sprintf(page,
"Remote map\n"
"\n"
"Map for a specific application\n"
"Remote signals matching this map will be translated into keyboard/mouse events\n");
}

static struct configfs_item_operations remote_map_item_ops = {
	.release	= remote_map_release,
	.show_attribute	= remote_map_attr_show,
};

/*
 * Note that, since no extra work is required on ->drop_item(),
 * no ->drop_item() is provided.
 */
static struct configfs_group_operations remote_map_group_ops = {
	.make_item	= remote_map_make_item,
};

static struct configfs_attribute remote_map_attr_description = {
	.ca_owner = THIS_MODULE,
	.ca_name = "description",
	.ca_mode = S_IRUGO,
};

static struct configfs_attribute *remote_map_attrs[] = {
	&remote_map_attr_description,
	NULL,
};

static struct config_item_type remote_map_type = {
	.ct_item_ops	= &remote_map_item_ops,
	.ct_group_ops	= &remote_map_group_ops,
	.ct_attrs	= remote_map_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group *remotes_make_group(struct config_group *group, const char *name)
{
	struct remote_map *remote_map;

	remote_map = kzalloc(sizeof(*remote_map), GFP_KERNEL);
	if (!remote_map)
		return ERR_PTR(-ENOMEM);

	config_group_init_type_name(&remote_map->group, name, &remote_map_type);

	return &remote_map->group;
}

static struct configfs_attribute remotes_attr_description = {
	.ca_owner = THIS_MODULE,
	.ca_name = "description",
	.ca_mode = S_IRUGO,
};

static struct configfs_attribute *remotes_attrs[] = {
	&remotes_attr_description,
	NULL,
};

static ssize_t remotes_attr_show(struct config_item *item,
					struct configfs_attribute *attr,
					char *page)
{
	return sprintf(page,
"IR Remotes\n"
"\n"
"This subsystem allows the creation of IR remote control maps.\n"
"Maps allow IR signals to be mapped into key strokes or mouse events.\n");
}

static struct configfs_item_operations remotes_item_ops = {
	.show_attribute	= remotes_attr_show,
};

/*
 * Note that, since no extra work is required on ->drop_item(),
 * no ->drop_item() is provided.
 */
static struct configfs_group_operations remotes_group_ops = {
	.make_group	= remotes_make_group,
};

static struct config_item_type remotes_type = {
	.ct_item_ops	= &remotes_item_ops,
	.ct_group_ops	= &remotes_group_ops,
	.ct_attrs	= remotes_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct configfs_subsystem remotes = {
	.su_group = {
		.cg_item = {
			.ci_namebuf = "IR-remotes",
			.ci_type = &remotes_type,
		},
	},
};

static ssize_t ir_raw_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct input_dev *input_dev = to_input_dev(dev);
	unsigned int i, count = 0;

	for (i = input_dev->ir->raw.tail; i != input_dev->ir->raw.head; ) {

		count += snprintf(&buf[count], PAGE_SIZE - 1, "%i\n", input_dev->ir->raw.buffer[i++]);
		if (i > ARRAY_SIZE(input_dev->ir->raw.buffer))
			i = 0;
		if (count >= PAGE_SIZE - 1) {
			input_dev->ir->raw.tail = i;
			return PAGE_SIZE - 1;
		}
	}
	input_dev->ir->raw.tail = i;
	return count;
}

static ssize_t ir_raw_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	struct ir_device *ir = to_input_dev(dev)->ir;
	long delta;
	int i = count;
	int first = 0;

	if (!ir->xmit)
		return count;
	ir->send.count = 0;

	while (i > 0) {
		i -= strict_strtoul(&buf[i], i, &delta);
		while ((buf[i] != '\n') && (i > 0))
			i--;
		i--;
		/* skip leading zeros */
		if ((delta > 0) && !first)
			continue;

		ir->send.buffer[ir->send.count++] = abs(delta);
	}

	ir->xmit(ir->private, ir->send.buffer, ir->send.count, ir->raw.carrier, ir->raw.xmitter);

	return count;
}

static ssize_t ir_carrier_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct ir_device *ir = to_input_dev(dev)->ir;

	return sprintf(buf, "%i\n", ir->raw.carrier);
}

static ssize_t ir_carrier_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	struct ir_device *ir = to_input_dev(dev)->ir;

	ir->raw.carrier = simple_strtoul(buf, NULL, 0);
	return count;
}

static ssize_t ir_xmitter_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct ir_device *ir = to_input_dev(dev)->ir;

	return sprintf(buf, "%i\n", ir->raw.xmitter);
}

static ssize_t ir_xmitter_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	struct ir_device *ir = to_input_dev(dev)->ir;

	ir->raw.xmitter = simple_strtoul(buf, NULL, 0);
	return count;
}

static ssize_t ir_debug_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct ir_device *ir = to_input_dev(dev)->ir;
	struct config_item *i;

    mutex_lock(&remotes.su_mutex);

    list_for_each_entry(i, &remotes.su_group.cg_children, ci_entry) {
    	printk("item %s\n", i->ci_name);
    }
    mutex_unlock(&remotes.su_mutex);

	return sprintf(buf, "%i\n", ir->raw.xmitter);
}

static ssize_t ir_debug_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	struct ir_device *ir = to_input_dev(dev)->ir;

	ir->raw.xmitter = simple_strtoul(buf, NULL, 0);
	return count;
}

static DEVICE_ATTR(raw, S_IRUGO | S_IWUSR, ir_raw_show, ir_raw_store);
static DEVICE_ATTR(carrier, S_IRUGO | S_IWUSR, ir_carrier_show, ir_carrier_store);
static DEVICE_ATTR(xmitter, S_IRUGO | S_IWUSR, ir_xmitter_show, ir_xmitter_store);
static DEVICE_ATTR(debug, S_IRUGO | S_IWUSR, ir_debug_show, ir_debug_store);

static struct attribute *input_ir_attrs[] = {
		&dev_attr_raw.attr,
		&dev_attr_carrier.attr,
		&dev_attr_xmitter.attr,
		&dev_attr_debug.attr,
		NULL
	};

static struct attribute_group input_ir_group = {
	.name	= "ir",
	.attrs	= input_ir_attrs,
};

int input_ir_register(struct input_dev *dev)
{
	int ret;

	if (!dev->ir)
		return 0;

	config_group_init(&remotes.su_group);
	mutex_init(&remotes.su_mutex);
	ret = configfs_register_subsystem(&remotes);
	if (ret) {
		printk(KERN_ERR "Error %d while registering configfs %s\n",
			   ret, remotes.su_group.cg_item.ci_namebuf);
		goto out_unregister;
	}

	return sysfs_create_group(&dev->dev.kobj, &input_ir_group);

out_unregister:
	configfs_unregister_subsystem(&remotes);
	return ret;
}

int input_ir_create(struct input_dev *dev, void *private, send_func xmit)
{
	dev->ir = kzalloc(sizeof(struct ir_device), GFP_KERNEL);
	if (!dev->ir)
		return -ENOMEM;

	dev->evbit[0] = BIT_MASK(EV_IR);
	dev->ir->private = private;
	dev->ir->xmit = xmit;

	return 0;
}
EXPORT_SYMBOL_GPL(input_ir_create);


void input_ir_destroy(struct input_dev *dev)
{
	if (dev->ir) {
		kfree(dev->ir);
		dev->ir = NULL;
	}
	configfs_unregister_subsystem(&remotes);
}
EXPORT_SYMBOL_GPL(input_ir_destroy);

int input_ir_send(struct input_dev *dev, struct ir_command *ir_command, struct file *file)
{
	unsigned freq, xmit = 0;
	int ret;

	mutex_lock(&dev->ir->lock);

	switch (ir_command->protocol) {
	case IR_PROTOCOL_PHILIPS_RC5:
		freq = 36000;
		encode_rc5(dev->ir, ir_command);
		break;
	case IR_PROTOCOL_PHILIPS_RC6:
		freq = 36000;
		encode_rc6(dev->ir, ir_command);
		break;
	case IR_PROTOCOL_PHILIPS_RCMM:
		freq = 36000;
		encode_rc5(dev->ir, ir_command);
		break;
	case IR_PROTOCOL_JVC:
		freq = 38000;
		encode_jvc(dev->ir, ir_command);
		break;
	case IR_PROTOCOL_NEC:
		freq = 38000;
		encode_nec(dev->ir, ir_command);
		break;
	case IR_PROTOCOL_NOKIA:
	case IR_PROTOCOL_SHARP:
	case IR_PROTOCOL_PHILIPS_RECS80:
		freq = 38000;
		break;
	case IR_PROTOCOL_SONY_12:
	case IR_PROTOCOL_SONY_15:
	case IR_PROTOCOL_SONY_20:
		encode_sony(dev->ir, ir_command);
		freq = 40000;
		break;
	case IR_PROTOCOL_RCA:
		freq = 56000;
		break;
	case IR_PROTOCOL_ITT:
		freq = 0;
		break;
	default:
		ret = -ENODEV;
		goto exit;
	}

	if (dev->ir && dev->ir->xmit)
		ret = dev->ir->xmit(dev->ir->private, dev->ir->send.buffer, dev->ir->send.count, freq, xmit);
	else
		ret = -ENODEV;

exit:
	mutex_unlock(&dev->ir->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(input_ir_send);

