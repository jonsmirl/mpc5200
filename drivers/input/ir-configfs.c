/*
 * Configfs routines for IR support
 *
 * Copyright (C) 2008 Jon Smirl <jonsmirl@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/input.h>

#include "ir.h"

/* each 'struct item' represents a file in the configfs directory */
/* the four files are protocol, device, command, keycode */
struct item {
	struct config_item citem;
	int value;
};

static inline struct item *to_item(struct config_item *citem)
{
	return citem ? container_of(citem, struct item, citem) : NULL;
}

static ssize_t item_show(struct config_item *citem,
				      struct configfs_attribute *attr,
				      char *page)
{
	struct item *item = to_item(citem);
	return sprintf(page, "%d\n", item->value);
}

static ssize_t item_store(struct config_item *citem,
				       struct configfs_attribute *attr,
				       const char *page, size_t count)
{
	struct item *item = to_item(citem);
	unsigned long tmp;
	char *p = (char *) page;

	tmp = simple_strtoul(p, &p, 10);
	if (!p || (*p && (*p != '\n')))
		return -EINVAL;

	if (tmp > INT_MAX)
		return -ERANGE;

	item->value = tmp;
	return count;
}

static struct configfs_item_operations item_ops = {
	.show_attribute = item_show,
	.store_attribute = item_store,
};

static struct configfs_attribute item_protocol = {
	.ca_owner = THIS_MODULE,
	.ca_name = "protocol",
	.ca_mode = S_IRUGO | S_IWUSR,
};

static struct configfs_attribute item_device = {
	.ca_owner = THIS_MODULE,
	.ca_name = "device",
	.ca_mode = S_IRUGO | S_IWUSR,
};

static struct configfs_attribute item_command = {
	.ca_owner = THIS_MODULE,
	.ca_name = "command",
	.ca_mode = S_IRUGO | S_IWUSR,
};

static struct configfs_attribute item_keycode = {
	.ca_owner = THIS_MODULE,
	.ca_name = "keycode",
	.ca_mode = S_IRUGO | S_IWUSR,
};


/* Start the definition of the all of the attributes
 * in a single mapping directory
 */
static struct configfs_attribute *mapping_attrs[] = {
	&item_protocol,
	&item_device,
	&item_command,
	&item_keycode,
	NULL,
};

static struct config_item_type mapping_type = {
	.ct_item_ops = &item_ops,
	.ct_attrs	= mapping_attrs,
	.ct_owner	= THIS_MODULE,
};

/* named directory containing the four attribute files that constitute a mapping */
struct mapping_dir {
	struct config_group group;
	struct item protocol;
	struct item device;
	struct item command;
	struct item keycode;
};

static inline struct mapping_dir *to_mapping_dir(struct config_group *group)
{
	return group ? container_of(group, struct mapping_dir, group) : NULL;
}

static struct config_item *mapping_make_item(struct config_group *group, const char *name)
{
	struct mapping_dir *mapping = to_mapping_dir(group);
	struct item *item;

	if (strcmp(name, item_protocol.ca_name) == 0)
		item = &mapping->protocol;
	else if (strcmp(name, item_device.ca_name) == 0)
		item = &mapping->device;
	else if (strcmp(name, item_command.ca_name) == 0)
		item = &mapping->command;
	else if (strcmp(name, item_keycode.ca_name) == 0)
		item = &mapping->keycode;
	else {
		printk("No match %s\n", name);
		return ERR_PTR(-EINVAL);
	}

	config_item_init_type_name(&item->citem, name, &mapping_type);
	item->value = 0;

	return &item->citem;
}

static void mapping_release(struct config_item *item)
{
	kfree(to_mapping_dir(to_config_group(item)));
}

static ssize_t mapping_description_show(struct config_item *item,
					 struct configfs_attribute *attr,
					 char *page)
{
	return sprintf(page,
"Remote map\n"
"\n"
"Map for a specific application\n"
"Remote signals matching this map will be translated into keyboard/mouse events\n");
}

static struct configfs_item_operations remote_item_ops = {
	.release	= mapping_release,
	.show_attribute	= mapping_description_show,
};

/*
 * Note that, since no extra work is required on ->drop_item(),
 * no ->drop_item() is provided.
 */
static struct configfs_group_operations remote_group_ops = {
	.make_item	= mapping_make_item,
};

static struct configfs_attribute remote_attr_description = {
	.ca_owner = THIS_MODULE,
	.ca_name = "description",
	.ca_mode = S_IRUGO,
};

static struct configfs_attribute *remote_attrs[] = {
	&remote_attr_description,
	NULL,
};

static struct config_item_type remote_type = {
	.ct_item_ops	= &remote_item_ops,
	.ct_group_ops	= &remote_group_ops,
	.ct_attrs	= remote_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_group *remotes_make_group(struct config_group *group, const char *name)
{
	struct mapping_dir *mapping;

	mapping = kzalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping)
		return ERR_PTR(-ENOMEM);

	config_group_init_type_name(&mapping->group, name, &remote_type);

	return &mapping->group;
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

struct configfs_subsystem remotes = {
	.su_group = {
		.cg_item = {
			.ci_namebuf = "remotes",
			.ci_type = &remotes_type,
		},
	},
};
