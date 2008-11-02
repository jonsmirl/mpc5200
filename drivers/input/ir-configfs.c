/*
 * Configfs routines for IR support
 *
 *   configfs root
 *   --remotes
 *   ----specific remote
 *   ------keymap
 *   --------protocol
 *   --------device
 *   --------command
 *   --------keycode
 *   ------repeat keymaps
 *   --------....
 *   ----another remote
 *   ------more keymaps
 *   --------....
 *
 * Copyright (C) 2008 Jon Smirl <jonsmirl@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/input.h>

#include "ir.h"

static void remote_release(struct config_item *remote);

struct keymap {
	struct config_item item;
	int protocol;
	int device;
	int command;
	int keycode;
};

static inline struct keymap *to_keymap(struct config_item *item)
{
	return item ? container_of(item, struct keymap, item) : NULL;
}

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

static ssize_t item_show(struct config_item *item,
				      struct configfs_attribute *attr,
				      char *page)
{
	struct keymap *keymap = to_keymap(item);

	if (attr == &item_protocol)
		return sprintf(page, "%d\n", keymap->protocol);
	if (attr == &item_device)
		return sprintf(page, "%d\n", keymap->device);
	if (attr == &item_command)
		return sprintf(page, "%d\n", keymap->command);
	return sprintf(page, "%d\n", keymap->keycode);
}

static ssize_t item_store(struct config_item *item,
				       struct configfs_attribute *attr,
				       const char *page, size_t count)
{
	struct keymap *keymap = to_keymap(item);
	unsigned long tmp;
	char *p = (char *) page;

	tmp = simple_strtoul(p, &p, 10);
	if (!p || (*p && (*p != '\n')))
		return -EINVAL;

	if (tmp > INT_MAX)
		return -ERANGE;

	if (attr == &item_protocol)
		keymap->protocol = tmp;
	else if (attr == &item_device)
		keymap->device = tmp;
	else if (attr == &item_command)
		keymap->command = tmp;
	else
		keymap->keycode = tmp;

	return count;
}

static void keymap_release(struct config_item *item)
{
	kfree(to_keymap(item));
}

static struct configfs_item_operations keymap_ops = {
	.release = keymap_release,
	.show_attribute = item_show,
	.store_attribute = item_store,
};

/* Start the definition of the all of the attributes
 * in a single keymap directory
 */
static struct configfs_attribute *keymap_attrs[] = {
	&item_protocol,
	&item_device,
	&item_command,
	&item_keycode,
	NULL,
};

static struct config_item_type keymap_type = {
	.ct_item_ops = &keymap_ops,
	.ct_attrs	= keymap_attrs,
	.ct_owner	= THIS_MODULE,
};

static struct config_item *make_keymap(struct config_group *group, const char *name)
{
	struct keymap *keymap;

	keymap = kzalloc(sizeof(*keymap), GFP_KERNEL);
	if (!keymap)
		return ERR_PTR(-ENOMEM);

	config_item_init_type_name(&keymap->item, name, &keymap_type);
	return &keymap->item;
}

/*
 * Note that, since no extra work is required on ->drop_item(),
 * no ->drop_item() is provided.
 */
static struct configfs_group_operations remote_group_ops = {
	.make_item = make_keymap,
};

static ssize_t remote_show_description(struct config_item *item,
					 struct configfs_attribute *attr,
					 char *page)
{
	return sprintf(page,
"Map for a specific remote\n"
"Remote signals matching this map will be translated into keyboard/mouse events\n");
}

static struct configfs_item_operations remote_item_ops = {
	.release	= remote_release,
	.show_attribute	= remote_show_description,
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


/* Top level remotes directory for all remotes */

/* Create a new remote group */
static struct config_group *remote_make(struct config_group *parent, const char *name)
{
	struct config_group *remote;

	remote = kzalloc(sizeof(*remote), GFP_KERNEL);
	if (!remote)
		return ERR_PTR(-ENOMEM);

	config_group_init_type_name(remote, name, &remote_type);

	return remote;
}

static void remote_release(struct config_item *remote)
{
	kfree(to_config_group(remote));
}

static ssize_t remotes_show_description(struct config_item *item,
					struct configfs_attribute *attr,
					char *page)
{
	return sprintf(page,
"This subsystem allows the creation of IR remote control maps.\n"
"Maps allow IR signals to be mapped into key strokes or mouse events.\n");
}

static struct configfs_item_operations remotes_item_ops = {
	.show_attribute	= remotes_show_description,
	.release = remote_release,
};

static struct configfs_attribute remotes_attr_description = {
	.ca_owner = THIS_MODULE,
	.ca_name = "description",
	.ca_mode = S_IRUGO,
};

static struct configfs_attribute *remotes_attrs[] = {
	&remotes_attr_description,
	NULL,
};

/*
 * Note that, since no extra work is required on ->drop_item(),
 * no ->drop_item() is provided.
 */
static struct configfs_group_operations remotes_group_ops = {
	.make_group	= remote_make,
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
