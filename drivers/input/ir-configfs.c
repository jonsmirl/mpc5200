/*
 * Configfs routines for IR support
 *
 * Copyright (C) 2008 Jon Smirl <jonsmirl@gmail.com>
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/input.h>

#include "ir.h"

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

struct configfs_subsystem remotes = {
	.su_group = {
		.cg_item = {
			.ci_namebuf = "IR-remotes",
			.ci_type = &remotes_type,
		},
	},
};
