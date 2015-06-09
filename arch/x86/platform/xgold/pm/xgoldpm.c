/*
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <linux/of.h>
#include <linux/device_state_pm.h>
#include <sofia/vpower.h>
#include "xgoldpm.h"
#if defined(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#endif

/* Final call for giving control to the platform specific PM API */
int xgold_dev_pm_set_state(struct device *dev,
		struct device_state_pm_state *new_state)
{
	int ret = 0;
	unsigned *mi, sz = 0;
	struct xgold_user_info *ui;

	if (!new_state)
		return -1; /* TODO: Return proper error */

	if (!strcmp(dev->pm_data.pm_user->name, "chg") ||
		!strcmp(dev->pm_data.pm_user->name, "bat") ||
		!strcmp(dev->pm_data.pm_user->name, "ccd") ||
		!strcmp(dev->pm_data.pm_user->name, "bnt"))
			return ret;

	mi = (uint32_t *)(new_state->mode_info);
	sz = new_state->mode_info_size;
	ui = (struct xgold_user_info *)(dev->pm_data.pm_user->user_config);

	pr_debug("#PM_DBG:Set STATE Call-> dev:%s,class:%s",
			dev->pm_data.pm_user->name,
			dev->pm_data.pm_class->name);
	pr_debug("#PM_DBG: user:%d, per:%d, newstate:%s\n",
			ui->user_id, ui->per_id, new_state->name);
	ret = vpower_call_prh(ui->user_id, ui->per_id, mi, sz);
	if (ret)
		pr_err("#PM_DBG: Error->Power State change %d\n", ret);
	pr_debug("#PM_DBG: Ret:%d\n", ret);
	return ret;
}

struct device_state_pm_state *xgold_dev_pm_get_initial_state(struct device *dev)
{
	/* The initial state of a device must be known here
	 * For now we just take the first state
	 */
	if ((!dev) || (!dev->pm_data.pm_class) ||
			(!dev->pm_data.pm_class->states))
		return NULL;

	return &(dev->pm_data.pm_class->states[0]);
}

static struct device_state_pm_ops xgold_pm_ops = {
	.set_state = xgold_dev_pm_set_state,
	.get_initial_state = xgold_dev_pm_get_initial_state,
};

#if defined(CONFIG_DEBUG_FS)
static int devpm_open(struct inode *inode, struct file *file)
{
	file->private_data = inode->i_private;
	return 0;
}

ssize_t devpm_read(struct file *file, char __user *buf,
					size_t size, loff_t *ppos)
{
	struct device_state_pm_dev *user;
	struct device_state_pm_class *class;
	char buffer[32];
	int len = 0;
	int i;

	if (*ppos)
		return 0;

	user = (struct device_state_pm_dev *)file->private_data;
	if (!user) {
		pr_err("%s:unknown user\n", __func__);
		return -EINVAL;
	}

	class = device_state_pm_find_class(user->reqd_class);
	if (!class) {
		pr_err("%s:no class for %s\n", __func__, user->name);
		return -EINVAL;
	}

	for (i = 0; i < class->num_states; ++i) {
		int l = snprintf(buffer, sizeof(buffer), "%s\n", class->states[i].name);

		if (copy_to_user(buf + len, buffer, l))
			return -EINVAL;
		len += l;
	}
	*ppos += len;

	return len;
}

static ssize_t devpm_write(struct file *file, const char __user *in,
						size_t count, loff_t *pos)
{
	struct device_state_pm_dev *user;
	struct device_state_pm_class *class;
	struct xgold_user_info *ui;
	char buffer[32];
	int ret;
	int i;

	user = (struct device_state_pm_dev *)file->private_data;
	if (!user) {
		pr_err("%s:unknown user\n", __func__);
		return -EINVAL;
	}

	class = device_state_pm_find_class(user->reqd_class);
	if (!class) {
		pr_err("%s:no class for %s\n", __func__, user->name);
		return -EINVAL;
	}

	if (copy_from_user(buffer, in, count > 32 ? 32 : count))
		return -EFAULT;

	for (i = 0; i < class->num_states; ++i) {
		if (!strncmp(class->states[i].name, buffer,
			strlen(class->states[i].name))) {
			unsigned *mi, sz = 0;
			ui = (struct xgold_user_info *)(user->user_config);

			mi = class->states[i].mode_info;
			sz = class->states[i].mode_info_size;
			ret = vpower_call_prh(ui->user_id, ui->per_id, mi, sz);
			pr_info("vpower_call_prh, uid: %d, pid: %d, status: %s, ret: %d\n",
					ui->user_id, ui->per_id,
					class->states[i].name, ret);
		}
	}

	return count;
}


static const struct file_operations devpm_fops = {
	.open = devpm_open,
	.read = devpm_read,
	.write = devpm_write,
};
#endif

#define USER_COMP "intel,xgold_pm_user"
#define CLASS_COMP "intel,xgold_pm_class"

#define USER_NAME_PROP "intel,name"
#define USER_ID_PROP "intel,user-id"
#define PER_ID_PROP "intel,per-id"
#define CLASS_PROP "intel,class"

#define CLASS_NAME_PROP "intel,name"
#define STATES_NAMES_PROP "intel,states-names"
#define CLASS_STATES_PROP "intel,states"
#define SYNC_PROP "intel,sync"

static int __init xgold_pm_of_init(void)
{
	struct device_node *np;
	int ret = 0, i, count = 0, field_nr, params_nr;
	struct device_state_pm_dev *user = NULL;
	struct xgold_user_info *user_info = NULL;
	struct device_state_pm_class *class = NULL;
	uint32_t *state_config = NULL;
	uint32_t *mode_info = NULL, mode_info_size = 0;
	struct device_node *class_node;
	uint32_t *sync_config = NULL;
	int32_t sync_len = 0;
#if defined(CONFIG_DEBUG_FS)
	struct dentry *rootdir = debugfs_create_dir("devpm", NULL);
#endif

	/* PM_USER */
	count = 0;
	for_each_compatible_node(np, NULL, USER_COMP) {
		pr_debug("%s: %s found!\n", __func__, np->full_name);
		/* Alloc pm_user */
		user = kzalloc(sizeof(struct device_state_pm_dev), GFP_KERNEL);
		if (!user) {
			pr_err("%s: alloc failed\n", __func__);
			goto free_them_all;
		}
		user_info = kzalloc(sizeof(struct xgold_user_info), GFP_KERNEL);
		if (!user_info) {
			pr_err("%s: alloc failed\n", __func__);
			goto free_them_all;
		}

		/* Fill pm_user with dts */
		of_property_read_string(np, USER_NAME_PROP, &user->name);
		class_node = of_parse_phandle(np, CLASS_PROP, 0);
		of_property_read_string(class_node, CLASS_NAME_PROP,
				&user->reqd_class);
		of_node_put(class_node);
		of_property_read_u32(np, USER_ID_PROP, &user_info->user_id);
		of_property_read_u32(np, PER_ID_PROP, &user_info->per_id);
		user->user_config = user_info;
		/* Add pm_user */
		device_state_pm_add_user(user);
		pr_debug("%s: PM_USER %s added with class %s user-id:%d per-id:%d\n",
				__func__, user->name, user->reqd_class,
				user_info->user_id, user_info->per_id);
		count++;
#if defined(CONFIG_DEBUG_FS)
		if (!IS_ERR_OR_NULL(rootdir))
			debugfs_create_file(user->name,
					S_IRUGO | S_IWUSR,
					rootdir, user, &devpm_fops);
#endif
	}
	pr_info("%s: %d pm users added%s\n",
			__func__, count, count ? "" : " => FAIL");

	/* PM_CLASS */
	count = 0;
	for_each_compatible_node(np, NULL, CLASS_COMP) {
		pr_debug("%s: %s found!\n", __func__, np->full_name);
		/* Alloc pm_class */
		class = kzalloc(sizeof(struct device_state_pm_class),
				GFP_KERNEL);
		if (!class) {
			pr_err("%s: alloc failed\n", __func__);
			goto free_them_all;
		}
		/* Fill pm_class with dts */
		of_property_read_string(np, CLASS_NAME_PROP, &class->name);
		class->num_states =
			of_property_count_strings(np, STATES_NAMES_PROP);
		if (class->num_states <= 0) {
			pr_err("%s: Property '%s's length is not correct\n",
				__func__, STATES_NAMES_PROP);
			goto free_them_all;
		}
		class->states = kzalloc(class->num_states *
				sizeof(struct device_state_pm_state),
				GFP_KERNEL);
		if (!class->states) {
			pr_err("%s: alloc failed\n", __func__);
			goto free_them_all;
		}
		of_find_property(np, CLASS_STATES_PROP, &params_nr);
		params_nr /= 4; /* 32bits values */
		if (params_nr % class->num_states) {
			pr_err("%s: must have same nb of params per state\n",
					__func__);
			goto free_them_all;
		}
		field_nr = params_nr / class->num_states;
		pr_debug("%s: field_nr %d, params_nr %d, num_states %d\n",
				__func__, field_nr, params_nr,
				class->num_states);

		/* Building mode_info: sync (if any) + config */
		of_get_property(np, SYNC_PROP, &sync_len);
		sync_len /= sizeof(uint32_t);
		mode_info_size = (sync_len + params_nr) * sizeof(uint32_t);
		mode_info = kzalloc(mode_info_size, GFP_KERNEL);
		if (!mode_info) {
			pr_err("%s: alloc failed\n", __func__);
			goto free_them_all;
		}
		if (!sync_len) {
			sync_config = NULL;
			state_config = mode_info;
		} else {
			sync_config = mode_info;
			state_config = mode_info + sync_len;
		}

		pr_debug("%s: sync_len:%d\n", __func__, sync_len);
		pr_debug("%s: params_nr:%d\n", __func__, params_nr);
		pr_debug("%s: mode_info_size:%d\n", __func__, mode_info_size);
		pr_debug("%s: sync_config:%p\n", __func__, sync_config);
		pr_debug("%s: state_config:%p\n", __func__, state_config);
		pr_debug("%s: mode_info:%p\n", __func__, mode_info);

		/* Get sync property if any */
		if (sync_len) {
			of_property_read_u32_array(np, SYNC_PROP,
					sync_config, sync_len);
			pr_debug("%s: sync property found: %d words\n",
					__func__, sync_len);
			for (i = 0; i < sync_len; i++)
				pr_debug("%s:sync_config[%d]=%#x\n",
					__func__, i, sync_config[i]);
		}
		/* Get state property */
		for (i = 0; i < params_nr; i++) {
			if (!(i % field_nr)) {
				of_property_read_string_index(np,
					STATES_NAMES_PROP, i / field_nr,
					&class->states[i / field_nr].name);
				class->states[i / field_nr].mode_info =
					(void *)&mode_info[i];
				class->states[i / field_nr].mode_info_size =
					mode_info_size;
			}
			of_property_read_u32_index(np, CLASS_STATES_PROP,
					i, &state_config[i]);
		}
		class->ops = &xgold_pm_ops;
		/* Add pm_class */
		device_state_pm_add_class(class);
		pr_debug("%s: PM_CLASS %s added with %d states:\n", __func__,
				class->name, class->num_states);
		for (i = 0; i < params_nr; i++) {
			if (!(i % field_nr))
				pr_debug("%s: state[%d]: %s\n", __func__,
					i / field_nr,
					class->states[i / field_nr].name);
			pr_debug("config[%d]: %d\n", i % field_nr,
					state_config[i]);
		}
		count++;
	}
	pr_info("%s: %d pm class added%s\n",
			__func__, count, count ? "" : " => FAIL");
	return ret;

free_them_all:
	pr_err("%s: failed: free them all\n", __func__);
	kfree(mode_info);
	kfree(class);
	kfree(user);
	kfree(user_info);
	return -1;
}

void __init xgold_pm_init(void)
{
	xgold_pm_of_init();
}

arch_initcall(xgold_pm_of_init);
