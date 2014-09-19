/*
 ****************************************************************
 *
 *  Intel CIF ISP 2.0 driver - Virtualized platform implementation
 *
 *  Copyright (C) 2014 Intel Mobile GmbH
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 *  You should have received a copy of the GNU General Public License Version 2
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * Note:
 *     07/07/2014: initial version.
 *
 ****************************************************************
 */

#ifndef CONFIG_OF
#error "this driver requires a kernel with device tree support"
#endif

#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/platform_device_pm.h>
#include <linux/interrupt.h>
#include <sofia/vmm_platform_service.h>
#include "cif_isp20.h"
#include <linux/list.h>
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/fs.h>
#ifdef CONFIG_CIF_ISP20_REG_TRACE
#include <stdarg.h>

static struct {
	char *reg_trace;
	loff_t reg_trace_read_pos;
	loff_t reg_trace_write_pos;
	size_t reg_trace_max_size;
	void __iomem *base_addr;
} cif_isp20_reg_trace;
#endif
#endif

struct cif_isp20_pltfrm_csi_config {
	struct list_head list;
	u32 pps;
	struct cif_isp20_csi_config csi_config;
};

struct cif_isp20_pltfrm_data {
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct device_pm_platdata *pm_platdata;
	struct {
		int irq;
		int (*isr)(void *cntxt);
	} irq_handlers[4];
	struct list_head csi0_configs;
	struct list_head csi1_configs;
#ifdef CONFIG_DEBUG_FS
	struct {
		struct dentry *dir;
		struct dentry *csi0_file;
		struct dentry *csi1_file;
#ifdef CONFIG_CIF_ISP20_REG_TRACE
		struct dentry *reg_trace_file;
#endif
	} dbgfs;
#endif
};

static int cif_isp20_pltfrm_fill_csi_config_from_node(
	struct device *dev,
	struct cif_isp20_csi_config *csi_config,
	struct device_node *csi_config_node
	)
{
	int ret = 0;

	ret = of_property_read_u32(csi_config_node,
		"intel,csi-vc",
		&csi_config->vc);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev,
			"reading property 'intel,csi-vc'\n");
		goto err;
	}

	ret = of_property_read_u32(csi_config_node,
		"intel,csi-lanes",
		&csi_config->nb_lanes);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev,
			"reading property 'intel,csi-lanes'\n");
		goto err;
	}

	ret = of_property_read_u32(csi_config_node,
		"intel,csi-dphy1",
		&csi_config->dphy1);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev,
			"reading property 'intel,csi-dphy1'\n");
		goto err;
	}

	ret = of_property_read_u32(csi_config_node,
		"intel,csi-dphy2",
		&csi_config->dphy2);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev,
			"reading property 'intel,csi-dphy2'\n");
		goto err;
	}

	ret = of_property_read_u32(csi_config_node,
		"intel,csi-ana-bandgap-bias",
		&csi_config->ana_bandgab_bias);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev,
			"reading property 'intel,csi-ana-bandgap-bias'\n");
		goto err;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev,
		"failed with error %d", ret);
	return ret;
}

static struct cif_isp20_pltfrm_csi_config
	*cif_isp20_g_csi_config_from_list(
	u32 pps,
	struct list_head *csi_configs)
{
	struct list_head *pos;
	struct cif_isp20_pltfrm_csi_config *cfg;
	struct cif_isp20_pltfrm_csi_config *best_cfg = NULL;

	if (list_empty(csi_configs))
		return ERR_PTR(-EINVAL);

	list_for_each(pos, csi_configs) {
		cfg = list_entry(pos, struct cif_isp20_pltfrm_csi_config, list);
		if ((cfg->pps <= pps) &&
			((best_cfg == NULL) || (cfg->pps > best_cfg->pps)))
			best_cfg = cfg;
	}

	if (best_cfg != NULL)
		return best_cfg;
	return ERR_PTR(-EINVAL);
}

static int cif_isp20_pltfrm_l_g_csi_config(
	struct device *dev,
	enum cif_isp20_inp inp,
	u32 *pps,
	struct cif_isp20_csi_config *csi_config)
{
	int ret = 0;
	u32 best_pps = 0;
	struct cif_isp20_pltfrm_data *pdata = dev->platform_data;
	struct device_node *best_csi_config = NULL;
	struct device *img_src_dev = NULL;
	struct device_node *parent_node = NULL;
	struct device_node *child_node = NULL, *prev_node = NULL;
	u32 of_value;
	struct cif_isp20_pltfrm_csi_config *cfg = NULL;

	if (inp == CIF_ISP20_INP_CSI_0)
		cfg = cif_isp20_g_csi_config_from_list(
			*pps, &pdata->csi0_configs);
	else if (inp == CIF_ISP20_INP_CSI_1) {
		cfg = cif_isp20_g_csi_config_from_list(
			*pps, &pdata->csi1_configs);
	}
	if (!IS_ERR_OR_NULL(cfg)) {
		cif_isp20_pltfrm_pr_dbg(dev,
			"found CSI config vc = %d, nb_lanes = %d, dphy1 = 0x%08x, dphy2 = 0x%02x, ana_bandgap_bias = %d\n",
			cfg->csi_config.vc,
			cfg->csi_config.nb_lanes,
			cfg->csi_config.dphy1,
			cfg->csi_config.dphy2,
			cfg->csi_config.ana_bandgab_bias);
		*pps = cfg->pps;
		*csi_config = cfg->csi_config;
		return ret;
	}

	img_src_dev = cif_isp20_pltfrm_get_img_src_device(dev, inp);
	if (IS_ERR_OR_NULL(img_src_dev)) {
		if (IS_ERR(img_src_dev)) {
			ret = PTR_ERR(img_src_dev);
			goto err;
		} else {
			ret = -ENODEV;
			goto err;
		}
	}
	parent_node = of_node_get(img_src_dev->of_node);
	put_device(img_src_dev);

	while (!IS_ERR_OR_NULL(child_node =
		of_get_next_child(parent_node, prev_node))) {
		if (!strncasecmp(child_node->name,
			"intel,camera-module-csi-config",
			strlen("intel,camera-module-csi-config"))) {
			ret = of_property_read_u32(child_node,
				"intel,csi-pixels-per-second", &of_value);
			if (IS_ERR_VALUE(ret)) {
				cif_isp20_pltfrm_pr_err(dev,
					"reading property 'intel,csi-pixels-per-second'\n");
				goto err;
			}
			if ((of_value <= *pps) && (of_value >= best_pps)) {
				of_node_put(best_csi_config);
				best_csi_config = of_node_get(child_node);
				best_pps = of_value;
			}
		}
		of_node_put(prev_node);
		prev_node = child_node;
	}
	of_node_put(prev_node);
	of_node_put(parent_node);

	if (IS_ERR_OR_NULL(best_csi_config)) {
		cif_isp20_pltfrm_pr_err(dev,
			"no matching CSI config (%d pps) found\n", *pps);
		ret = -EEXIST;
		goto err;
	}

	ret = cif_isp20_pltfrm_fill_csi_config_from_node(dev,
		csi_config, best_csi_config);
	if (IS_ERR_VALUE(ret))
		goto err;
	of_node_put(best_csi_config);

	*pps = best_pps;
	cif_isp20_pltfrm_pr_dbg(dev,
		"found CSI config vc = %d, nb_lanes = %d, dphy1 = 0x%08x, dphy2 = 0x%02x, ana_bandgap_bias = %d\n",
		csi_config->vc,
		csi_config->nb_lanes,
		csi_config->dphy1,
		csi_config->dphy2,
		csi_config->ana_bandgab_bias);

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev, "failed with error %d\n", ret);
	of_node_put(prev_node);
	of_node_put(child_node);
	of_node_put(parent_node);
	of_node_put(best_csi_config);
	if (!IS_ERR_OR_NULL(img_src_dev))
		put_device(img_src_dev);
	return ret;
}

int cif_isp20_pltfrm_l_s_csi_config(
	struct device *dev,
	enum cif_isp20_inp inp,
	u32 pps,
	struct cif_isp20_csi_config *csi_config)
{
	int ret = 0;
	struct cif_isp20_pltfrm_data *pdata = dev->platform_data;
	struct cif_isp20_pltfrm_csi_config *cfg = NULL;
	struct list_head *csi_configs;

	if (inp == CIF_ISP20_INP_CSI_0) {
		csi_configs = &pdata->csi0_configs;
	} else if (inp == CIF_ISP20_INP_CSI_1) {
		csi_configs = &pdata->csi1_configs;
	} else {
		cif_isp20_pltfrm_pr_err(dev, "wrong input\n");
		ret = -EINVAL;
		goto err;
	}

	cfg = cif_isp20_g_csi_config_from_list(pps, csi_configs);
	if (!IS_ERR_OR_NULL(cfg))
		cfg->csi_config = *csi_config;
	else {
		struct cif_isp20_csi_config of_csi_config;
		ret = cif_isp20_pltfrm_l_g_csi_config(
			dev, inp, &pps, &of_csi_config);
		if (IS_ERR_VALUE(ret))
			goto err;
		cfg = kmalloc(
			sizeof(struct cif_isp20_pltfrm_csi_config),
		GFP_KERNEL);
		if (cfg == NULL) {
			cif_isp20_pltfrm_pr_err(dev,
				"memory allocation failed\n");
			ret = -ENOMEM;
			goto err;
		}
		cfg->pps = pps;
		cfg->csi_config = *csi_config;
		list_add_tail(&cfg->list, csi_configs);
	}

	cif_isp20_pltfrm_pr_dbg(dev,
		"updated CSI config:\n"
		"   pps = %d\n"
		"   vc = %d\n"
		"   nb_lanes = %d\n"
		"   dphy1 = 0x%08x\n"
		"   dphy2 = 0x%08x\n"
		"   ana_bandgap_bias = %d\n",
		cfg->pps, cfg->csi_config.vc, cfg->csi_config.nb_lanes,
		cfg->csi_config.dphy1, cfg->csi_config.dphy2,
		cfg->csi_config.ana_bandgab_bias);

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev, "failed with error %d\n", ret);
	return ret;
}


#ifdef CONFIG_DEBUG_FS
#define CIF_ISP20_DBGFS_BUF_SIZE 256
static char cif_isp20_dbgfs_buf[CIF_ISP20_DBGFS_BUF_SIZE];

static int cif_isp20_dbgfs_fill_csi_config_from_string(
	struct device *dev,
	struct cif_isp20_csi_config *csi_config,
	char *strp)
{
	char *token;

	token = strsep(&strp, " ");
	if (IS_ERR_OR_NULL(token))
		goto missing_token;
	if (IS_ERR_VALUE(kstrtou32(token, 10,
			&csi_config->vc)))
		goto wrong_token_format;
	token = strsep(&strp, " ");
	if (IS_ERR_OR_NULL(token))
		goto missing_token;
	if (IS_ERR_VALUE(kstrtou32(token, 10,
			&csi_config->nb_lanes)))
		goto wrong_token_format;
	token = strsep(&strp, " ");
	if (IS_ERR_OR_NULL(token))
		goto missing_token;
	if (IS_ERR_VALUE(kstrtou32(token, 16,
			&csi_config->dphy1)))
		goto wrong_token_format;
	token = strsep(&strp, " ");
	if (IS_ERR_OR_NULL(token))
		goto missing_token;
	if (IS_ERR_VALUE(kstrtou32(token, 16,
			&csi_config->dphy2)))
		goto wrong_token_format;
	token = strsep(&strp, " ");
	if (IS_ERR_OR_NULL(token))
		goto missing_token;
	if (IS_ERR_VALUE(kstrtou32(token, 10,
			&csi_config->ana_bandgab_bias)))
		goto wrong_token_format;

	return 0;
missing_token:
	cif_isp20_pltfrm_pr_err(dev,
		"missing token, command format is 'push <pps> <vc> <#lanes> <HEX dphy1> <HEX dphy2> <analog bandgap bias>'\n");
	return -EINVAL;
wrong_token_format:
	cif_isp20_pltfrm_pr_err(dev,
		"wrong token format, command format is 'push <pps> <vc> <#lanes> <HEX dphy1> <HEX dphy2> <analog bandgap bias>'\n");
	return -EINVAL;
}

static int cif_isp20_dbgfs_csi_configs_init(
	struct device *dev,
	enum cif_isp20_inp inp,
	struct list_head *csi_configs)
{
	int ret = 0;
	struct device *img_src_dev = NULL;
	struct device_node *parent_node = NULL;
	struct device_node *child_node = NULL, *prev_node = NULL;
	struct cif_isp20_pltfrm_csi_config *cfg = NULL;
	u32 pps;

	img_src_dev = cif_isp20_pltfrm_get_img_src_device(dev, inp);
	if (IS_ERR_OR_NULL(img_src_dev)) {
		ret = -EFAULT;
		goto err;
	}
	parent_node = of_node_get(img_src_dev->of_node);
	put_device(img_src_dev);
	img_src_dev = NULL;

	while (!IS_ERR_OR_NULL(child_node =
		of_get_next_child(parent_node, prev_node))) {
		if (!strncasecmp(child_node->name,
			"intel,camera-module-csi-config",
			strlen("intel,camera-module-csi-config"))) {
			ret = of_property_read_u32(child_node,
				"intel,csi-pixels-per-second", &pps);
			if (IS_ERR_VALUE(ret)) {
				cif_isp20_pltfrm_pr_err(dev,
					"reading property 'intel,csi-pixels-per-second'\n");
				goto err;
			}
			cfg = kmalloc(
				sizeof(struct cif_isp20_pltfrm_csi_config),
			GFP_KERNEL);
			if (cfg == NULL) {
				cif_isp20_pltfrm_pr_err(dev,
					"memory allocation failed\n");
				ret = -ENOMEM;
				goto err;
			}
			cfg->pps = pps;
			ret = cif_isp20_pltfrm_fill_csi_config_from_node(
					dev, &cfg->csi_config, child_node);
			if (IS_ERR_VALUE(ret))
				goto err;
			list_add_tail(&cfg->list, csi_configs);
			cfg = NULL;
		}
		of_node_put(prev_node);
		prev_node = child_node;
	}
	of_node_put(prev_node);
	of_node_put(parent_node);

	return 0;
err:
	of_node_put(prev_node);
	of_node_put(child_node);
	of_node_put(parent_node);
	kfree(cfg);
	if (!IS_ERR_OR_NULL(img_src_dev))
		put_device(img_src_dev);
	return ret;
}

static ssize_t cif_isp20_dbgfs_read(
	struct file *f,
	char __user *out,
	size_t count,
	loff_t *pos)
{
	u32 out_size = 0;
	u32 str_len;
	struct cif_isp20_pltfrm_csi_config *cfg;
	u32 index = 0;
	struct list_head *list_pos;
	struct device *dev = f->f_inode->i_private;
	struct cif_isp20_pltfrm_data *pdata = dev->platform_data;
	struct list_head *csi_configs;
	enum cif_isp20_inp inp;

	if (f->f_inode == pdata->dbgfs.csi0_file->d_inode) {
		csi_configs = &pdata->csi0_configs;
		inp = CIF_ISP20_INP_CSI_0;
	} else if (f->f_inode == pdata->dbgfs.csi1_file->d_inode) {
		csi_configs = &pdata->csi1_configs;
		inp = CIF_ISP20_INP_CSI_1;
	} else {
		cif_isp20_pltfrm_pr_err(dev, "wrong file handle\n");
		return -EINVAL;
	}

	if (list_empty(csi_configs))
		if (IS_ERR_VALUE(cif_isp20_dbgfs_csi_configs_init(
				dev, inp, csi_configs)))
				return -EFAULT;

	if (*pos)
		return 0;

	list_for_each(list_pos, csi_configs) {
		cfg = list_entry(list_pos,
			struct cif_isp20_pltfrm_csi_config, list);
		sprintf(cif_isp20_dbgfs_buf,
			"csi-config-%d:\n"
			"   pps = %d\n"
			"   vc = %d\n"
			"   nb_lanes = %d\n"
			"   dphy1 = 0x%08x\n"
			"   dphy2 = 0x%08x\n"
			"   ana_bandgap_bias = %d\n",
			index,
			cfg->pps, cfg->csi_config.vc, cfg->csi_config.nb_lanes,
			cfg->csi_config.dphy1, cfg->csi_config.dphy2,
			cfg->csi_config.ana_bandgab_bias);
		index++;
		str_len = strnlen(cif_isp20_dbgfs_buf,
			CIF_ISP20_DBGFS_BUF_SIZE);
		if (str_len > count) {
			*pos += out_size;
			return 0;
		}
		*pos = 0;
		if (IS_ERR_VALUE(simple_read_from_buffer(
			out + out_size, str_len, pos,
			cif_isp20_dbgfs_buf, str_len)))
			break;
		out_size += strnlen(cif_isp20_dbgfs_buf,
			CIF_ISP20_DBGFS_BUF_SIZE);
		count -= str_len;
	}

	*pos += out_size;
	return out_size;
}

static ssize_t cif_isp20_dbgfs_write(
	struct file *f,
	const char __user *in,
	size_t count,
	loff_t *pos)
{
	ssize_t ret;
	char *strp = cif_isp20_dbgfs_buf;
	char *token;
	struct device *dev = f->f_inode->i_private;
	struct cif_isp20_pltfrm_data *pdata = dev->platform_data;
	struct list_head *csi_configs;
	enum cif_isp20_inp inp;

	if (count > CIF_ISP20_DBGFS_BUF_SIZE) {
		cif_isp20_pltfrm_pr_err(dev, "command line too large\n");
		return -EINVAL;
	}

	if (f->f_inode == pdata->dbgfs.csi0_file->d_inode) {
		csi_configs = &pdata->csi0_configs;
		inp = CIF_ISP20_INP_CSI_0;
	} else if (f->f_inode == pdata->dbgfs.csi1_file->d_inode) {
		csi_configs = &pdata->csi1_configs;
		inp = CIF_ISP20_INP_CSI_1;
	} else {
		cif_isp20_pltfrm_pr_err(dev, "wrong file handle\n");
		return -EINVAL;
	}

	if (list_empty(csi_configs))
		if (IS_ERR_VALUE(cif_isp20_dbgfs_csi_configs_init(
				dev, inp, csi_configs)))
				return -EFAULT;

	memset(cif_isp20_dbgfs_buf, 0, CIF_ISP20_DBGFS_BUF_SIZE);
	ret = simple_write_to_buffer(strp,
		CIF_ISP20_DBGFS_BUF_SIZE, pos, in, count);
	if (IS_ERR_VALUE(ret))
		return ret;

	token = strsep(&strp, " ");
	if (!strcmp(token, "push")) {
		struct cif_isp20_pltfrm_csi_config cfg;
		token = strsep(&strp, " ");
		if (IS_ERR_OR_NULL(token)) {
			cif_isp20_pltfrm_pr_err(dev,
				"missing token, command format is 'push <pps> <vc> <#lanes> <HEX dphy1> <HEX dphy2> <analog bandgap bias>'\n");
			return -EINVAL;
		}
		if (IS_ERR_VALUE(kstrtou32(token, 10,
				&cfg.pps))) {
			cif_isp20_pltfrm_pr_err(dev,
				"missing token, command format is 'push <pps> <vc> <#lanes> <HEX dphy1> <HEX dphy2> <analog bandgap bias>'\n");
			return -EINVAL;
		}
		ret = cif_isp20_dbgfs_fill_csi_config_from_string(
			dev, &cfg.csi_config, strp);
		if (IS_ERR_VALUE(ret))
			return ret;
		ret = cif_isp20_pltfrm_l_s_csi_config(
			dev, inp, cfg.pps, &cfg.csi_config);
		if (IS_ERR_VALUE(ret))
			return ret;
	} else if (!strncmp(token, "reset", 5)) {
		cif_isp20_reset_csi_configs(dev, inp);
	} else {
		cif_isp20_pltfrm_pr_err(dev, "unkown command %s\n", token);
		return -EINVAL;
	}

	return count;
}

static const struct file_operations cif_isp20_dbgfs_fops = {
	.read = cif_isp20_dbgfs_read,
	.write = cif_isp20_dbgfs_write
};

#ifdef CONFIG_CIF_ISP20_REG_TRACE

int
cif_isp20_pltfrm_reg_trace_printf(
	struct device *dev,
	const char *fmt,
	...)
{
	va_list args;
	int i;
	u32 rem_size;

	rem_size = cif_isp20_reg_trace.reg_trace_max_size -
		cif_isp20_reg_trace.reg_trace_write_pos - 1;

	va_start(args, fmt);
	i = vsnprintf(cif_isp20_reg_trace.reg_trace +
		cif_isp20_reg_trace.reg_trace_write_pos,
		rem_size,
		fmt, args);
	va_end(args);
	if (i == rem_size) /* buffer full */
		return 0;
	cif_isp20_reg_trace.reg_trace_write_pos += i;
	return i;
}

static void cif_isp20_dbgfs_reg_trace_clear(
	struct device *dev)
{
	cif_isp20_reg_trace.reg_trace_write_pos = 0;
	cif_isp20_reg_trace.reg_trace_read_pos = 0;
}

static ssize_t cif_isp20_dbgfs_reg_trace_read(
	struct file *f,
	char __user *out,
	size_t count,
	loff_t *pos)
{
	ssize_t bytes;

	bytes = simple_read_from_buffer(
		out, count, &cif_isp20_reg_trace.reg_trace_read_pos,
		cif_isp20_reg_trace.reg_trace,
		cif_isp20_reg_trace.reg_trace_write_pos -
		cif_isp20_reg_trace.reg_trace_read_pos);

	if (!bytes)
		cif_isp20_reg_trace.reg_trace_read_pos = 0;

	return bytes;
}

static ssize_t cif_isp20_dbgfs_reg_trace_write(
	struct file *f,
	const char __user *in,
	size_t count,
	loff_t *pos)
{
	ssize_t ret;
	char *strp = cif_isp20_dbgfs_buf;
	char *token;
	struct device *dev = f->f_inode->i_private;
	u32 max_size;

	if (count > CIF_ISP20_DBGFS_BUF_SIZE) {
		cif_isp20_pltfrm_pr_err(dev, "command line too long\n");
		return -EINVAL;
	}

	memset(cif_isp20_dbgfs_buf, 0, CIF_ISP20_DBGFS_BUF_SIZE);
	ret = simple_write_to_buffer(strp,
		CIF_ISP20_DBGFS_BUF_SIZE, pos, in, count);
	if (IS_ERR_VALUE(ret))
		return ret;

	token = strsep(&strp, " ");
	if (!strncmp(token, "clear", 5)) {
		cif_isp20_dbgfs_reg_trace_clear(dev);
		cif_isp20_pltfrm_pr_info(dev,
			"register trace buffer cleared\n");
	} else if (!strcmp(token, "size")) {
		token = strsep(&strp, " ");
		if (IS_ERR_OR_NULL(token)) {
			cif_isp20_pltfrm_pr_err(dev,
				"missing token, command format is 'size <num entries>'\n");
			return -EINVAL;
		}
		if (IS_ERR_VALUE(kstrtou32(token, 10,
				&max_size))) {
			cif_isp20_pltfrm_pr_err(dev,
				"wrong token format, <num entries> must be positive integer>'\n");
			return -EINVAL;
		}
		if (cif_isp20_reg_trace.reg_trace) {
			devm_kfree(dev, cif_isp20_reg_trace.reg_trace);
			cif_isp20_reg_trace.reg_trace = NULL;
		}
		cif_isp20_dbgfs_reg_trace_clear(dev);
		if (max_size > 0) {
			cif_isp20_reg_trace.reg_trace = devm_kzalloc(dev,
				max_size, GFP_KERNEL);
			if (!cif_isp20_reg_trace.reg_trace) {
				cif_isp20_pltfrm_pr_err(dev,
					"memory allocation failed\n");
				return -ENOMEM;
			}
			cif_isp20_reg_trace.reg_trace_max_size = max_size;
			cif_isp20_pltfrm_pr_info(dev,
				"register trace buffer size set to %d Byte\n",
				max_size);
		}
	} else {
		cif_isp20_pltfrm_pr_err(dev, "unkown command %s\n", token);
		return -EINVAL;
	}
	return count;
}

static const struct file_operations
cif_isp20_dbgfs_reg_trace_fops = {
	.read = cif_isp20_dbgfs_reg_trace_read,
	.write = cif_isp20_dbgfs_reg_trace_write
};

#endif
#endif

static irqreturn_t cif_isp20_pltfrm_irq_handler(int irq, void *cntxt)
{
	unsigned int i;
	int ret;
	struct device *dev = cntxt;
	struct cif_isp20_pltfrm_data *pdata =
		dev_get_platdata(dev);
	void *cif_isp20_dev = dev_get_drvdata(dev);

	for (i = 0; i < ARRAY_SIZE(pdata->irq_handlers); i++) {
		if (pdata->irq_handlers[i].irq == irq) {
			if (IS_ERR_OR_NULL(pdata->irq_handlers[i].isr)) {
				cif_isp20_pltfrm_pr_err(NULL,
					"ISR for IRQ #%d not set\n", irq);
				break;
			}
			ret = pdata->irq_handlers[i].isr(cif_isp20_dev);
			if (IS_ERR_VALUE(ret)) {
				cif_isp20_pltfrm_pr_err(NULL,
					"ISR for IRQ #%d failed with error %d\n",
					irq, ret);
			}
			return IRQ_HANDLED;
		}
	}

	return IRQ_NONE;
}

const char *cif_isp20_pltfrm_pm_state_string(
	enum device_pm_state pm_state)
{
	switch (pm_state) {
	case PM_STATE_D0:
		return "PM_STATE_D0";
	case PM_STATE_D1:
		return "PM_STATE_D1";
	case PM_STATE_D2:
		return "PM_STATE_D2";
	case PM_STATE_D3:
		return "PM_STATE_D3";
	case PM_STATE_D0i0:
		return "PM_STATE_D0i0";
	case PM_STATE_D0i1:
		return "PM_STATE_D0i1";
	case PM_STATE_D0i2:
		return "PM_STATE_D0i2";
	case PM_STATE_D0i3:
		return "PM_STATE_D0i3";
	case PM_STATE_D0i4:
		return "PM_STATE_D0i4";
	case PM_STATE_D0i5:
		return "PM_STATE_D0i5";
	case PM_STATE_D0i6:
		return "PM_STATE_D0i6";
	default:
		return "PM_STATE_UNKNOWN";
	}
}

void cif_isp20_pltfrm_write_reg(
	struct device *dev,
	u32 data,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr)
{
	iowrite32(data, addr);
#ifdef CONFIG_CIF_ISP20_REG_TRACE
	if ((cif_isp20_reg_trace.reg_trace_write_pos + (24 * sizeof(char))) <
		cif_isp20_reg_trace.reg_trace_max_size)
		cif_isp20_reg_trace.reg_trace_write_pos +=
			sprintf(cif_isp20_reg_trace.reg_trace +
				cif_isp20_reg_trace.reg_trace_write_pos,
				"%08x %08x\n",
				addr - cif_isp20_reg_trace.base_addr,
				data);
#endif
}

void cif_isp20_pltfrm_write_reg_OR(
	struct device *dev,
	u32 data,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr)
{
	cif_isp20_pltfrm_write_reg(dev,
		(ioread32(addr) | data), addr);
}

void cif_isp20_pltfrm_write_reg_AND(
	struct device *dev,
	u32 data,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr)
{
	cif_isp20_pltfrm_write_reg(dev,
		(ioread32(addr) & data), addr);
}

u32 cif_isp20_pltfrm_read_reg(
	struct device *dev,
	CIF_ISP20_PLTFRM_MEM_IO_ADDR addr)
{
	return ioread32(addr);
}

int cif_isp20_pltfrm_dev_init(
	struct cif_isp20_device *cif_isp20_dev,
	struct device **_dev,
	void __iomem **reg_base_addr)
{
	int ret;
	struct cif_isp20_pltfrm_data *pdata;
	struct device *dev = *_dev;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct device_node *np = dev->of_node;
	struct resource *res;
	void __iomem *base_addr;
	unsigned int i;

	if (IS_ERR_VALUE(dev_set_drvdata(dev, cif_isp20_dev))) {
		cif_isp20_pltfrm_pr_err(dev,
			"could not set driver data\n");
		return -ENODEV;
	}

	cif_isp20_dev->dev = dev;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (NULL == pdata) {
		cif_isp20_pltfrm_pr_err(dev,
			"could not allocate memory for platform data\n");
		ret = -ENOMEM;
		goto err;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "register");
	if (res == NULL) {
		cif_isp20_pltfrm_pr_err(NULL,
			"platform_get_resource_byname failed\n");
		ret = -ENODEV;
		goto err;
	}
	base_addr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR_OR_NULL(base_addr)) {
		cif_isp20_pltfrm_pr_err(NULL, "devm_ioremap_resource failed\n");
		if (IS_ERR(base_addr))
			ret = PTR_ERR(base_addr);
		else
			ret = -ENODEV;
	}
	*reg_base_addr = base_addr;

	/* FIXME:
	 * In case of CSI platform, no need for pinctrl
	 * but since parallel interfaces are possible we need to keep it
	 * Need to get rid of these "dummy" warning messages
	 * devm_pinctrl_get should return an error as no pinctrl node is
	 * associated ...
	 * */
	pdata->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR_OR_NULL(pdata->pinctrl)) {
		pdata->pins_default = pinctrl_lookup_state(pdata->pinctrl,
			PINCTRL_STATE_DEFAULT);

		pdata->pins_sleep = pinctrl_lookup_state(pdata->pinctrl,
			PINCTRL_STATE_SLEEP);

		pdata->pins_inactive = pinctrl_lookup_state(pdata->pinctrl,
			"inactive");

		if (!IS_ERR_OR_NULL(pdata->pins_default))
			pinctrl_select_state(pdata->pinctrl,
				pdata->pins_default);
	}
	pdata->pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pdata->pm_platdata)) {
		cif_isp20_pltfrm_pr_err(dev,
			"error during device state pm init\n");
		ret = PTR_ERR(pdata->pm_platdata);
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(pdata->irq_handlers); i++)
		pdata->irq_handlers[i].irq = -EINVAL;

	dev->platform_data = pdata;

#ifdef CONFIG_PLATFORM_DEVICE_PM
	{
		struct platform_device *pdev =
			container_of(dev, struct platform_device, dev);
		if (IS_ERR_VALUE(platform_device_pm_set_class(pdev,
				pdata->pm_platdata->pm_user_name))) {
			cif_isp20_pltfrm_pr_err(dev,
				"Registering to PM class failed\n");
			ret = -ENODEV;
			goto err;
		}

		if (IS_ERR_VALUE(device_pm_get_states_handlers(dev,
			pdata->pm_platdata))) {
			cif_isp20_pltfrm_pr_err(dev,
				"Errors while retrieving PM states\n");
			ret = -ENODEV;
			goto err;
		}
	}
#endif
	INIT_LIST_HEAD(&pdata->csi0_configs);
	INIT_LIST_HEAD(&pdata->csi1_configs);

#ifdef CONFIG_DEBUG_FS
	pdata->dbgfs.dir = debugfs_create_dir("cif_isp20", NULL);
	pdata->dbgfs.csi0_file = debugfs_create_file(
		"csi-0",
		0644,
		pdata->dbgfs.dir,
		dev,
		&cif_isp20_dbgfs_fops);
	pdata->dbgfs.csi1_file = debugfs_create_file(
		"csi-1",
		0644,
		pdata->dbgfs.dir,
		dev,
		&cif_isp20_dbgfs_fops);
#ifdef CONFIG_CIF_ISP20_REG_TRACE
	pdata->dbgfs.reg_trace_file = debugfs_create_file(
		"reg_trace",
		0644,
		pdata->dbgfs.dir,
		dev,
		&cif_isp20_dbgfs_reg_trace_fops);
	cif_isp20_reg_trace.reg_trace = NULL;
	cif_isp20_dbgfs_reg_trace_clear(dev);
	cif_isp20_reg_trace.reg_trace_max_size = 0;
	cif_isp20_reg_trace.base_addr = base_addr;
#endif
#endif

	return 0;
err:
	cif_isp20_pltfrm_pr_err(NULL, "failed with error %d\n", ret);
	if (!IS_ERR_OR_NULL(pdata))
		devm_kfree(dev, pdata);
	return ret;
}

int cif_isp20_pltfrm_pm_set_state(
	struct device *dev,
	enum cif_isp20_pm_state pm_state,
	struct cif_isp20_strm_fmt *strm_fmt)
{
	int ret;
	enum device_pm_state pltfrm_pm_state;
	struct cif_isp20_pltfrm_data *pdata = dev_get_platdata(dev);

	switch (pm_state) {
	case CIF_ISP20_PM_STATE_OFF:
	case CIF_ISP20_PM_STATE_SUSPENDED:
		pltfrm_pm_state = PM_STATE_D3;
		break;
	case CIF_ISP20_PM_STATE_SW_STNDBY:
	case CIF_ISP20_PM_STATE_STREAMING:
		pltfrm_pm_state = PM_STATE_D0;
		break;
	default:
		cif_isp20_pltfrm_pr_err(dev,
			"unknown or unsupported PM state %d\n", pm_state);
		return -EINVAL;
	}

	cif_isp20_pltfrm_pr_dbg(dev,
		"set pm state to %s\n",
		cif_isp20_pltfrm_pm_state_string(pltfrm_pm_state));

	ret = device_state_pm_set_state(dev,
		get_device_pm_state(pdata->pm_platdata, pltfrm_pm_state));

	if (IS_ERR_VALUE(ret))
		cif_isp20_pltfrm_pr_err(dev,
			"setting pm state to %s failed with error %d\n",
			cif_isp20_pltfrm_pm_state_string(pltfrm_pm_state), ret);
	else
		cif_isp20_pltfrm_pr_dbg(dev,
			"successfully changed pm state to %s\n",
			cif_isp20_pltfrm_pm_state_string(pltfrm_pm_state));
	return ret;
}

int cif_isp20_pltfrm_write_cif_ana_bandgap_bias(
	struct device *dev,
	u32 val)
{
	struct device_node *np;
	int ret = 0;
	u32 scu_base_addr;
	u32 offset;
	u32 mask;
	u32 shift;

	np = of_find_compatible_node(NULL, NULL, "intel,scu");
	if (IS_ERR_OR_NULL(np)) {
		cif_isp20_pltfrm_pr_err(dev,
			"cannot find node 'intel,scu' (SCU) in device tree\n");
		ret = -EEXIST;
		goto err;
	}

	ret = of_property_read_u32_index(np,
			"reg", 0, &scu_base_addr);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev,
			"reading property 'reg'\n");
		goto err;
	}
	ret = of_property_read_u32(np,
			"intel,cif-ana-bandgap-bias-mask", &mask);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev,
			"reading property 'intel,cif-ana-bandgap-bias-mask'\n");
		goto err;
	}

	ret = of_property_read_u32(np,
			"intel,cif-ana-bandgap-bias-reg-offset", &offset);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev,
			"reading property 'intel,cif-ana-bandgap-bias-reg-offset'\n");
		goto err;
	}

	for (shift = mask; !(shift & 0x1); val <<= 1, shift >>= 1)
		;

	ret = vmm_reg_write(scu_base_addr + offset, val, mask);
	if (IS_ERR_VALUE(ret)) {
		cif_isp20_pltfrm_pr_err(dev,
			"register write @0x%08x := 0x%08x (mask 0x%08x) failed\n",
			scu_base_addr + offset, val, mask);
		goto err;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_warn(dev, "failed with error %d\n", ret);
	/* Failing to set the bandgap is not critial error. Just notify with
		the error message. */
	return 0;
}

int cif_isp20_pltfrm_s_csi_config(
	struct device *dev,
	enum cif_isp20_inp inp,
	struct cif_isp20_strm_fmt *strm_fmt,
	struct cif_isp20_csi_config *csi_config)
{
	u32 pps = (
		strm_fmt->frm_fmt.width * strm_fmt->frm_fmt.height
		*
		strm_fmt->frm_intrvl.denominator
		+
		(strm_fmt->frm_intrvl.numerator - 1))
		/
		strm_fmt->frm_intrvl.numerator;

	cif_isp20_pltfrm_pr_dbg(dev, "%s, pps = %d",
		(inp == CIF_ISP20_INP_CSI_0) ? "CSI-0" :
			((inp == CIF_ISP20_INP_CSI_1) ? "CSI-1" : "?"),
		pps);

	return cif_isp20_pltfrm_l_s_csi_config(
		dev, inp, pps, csi_config);
}

int cif_isp20_pltfrm_g_csi_config(
	struct device *dev,
	enum cif_isp20_inp inp,
	struct cif_isp20_strm_fmt *strm_fmt,
	struct cif_isp20_csi_config *csi_config)
{
	u32 pps = (
		strm_fmt->frm_fmt.width * strm_fmt->frm_fmt.height
		*
		strm_fmt->frm_intrvl.denominator
		+
		(strm_fmt->frm_intrvl.numerator - 1))
		/
		strm_fmt->frm_intrvl.numerator;

	cif_isp20_pltfrm_pr_dbg(dev, "%s, pps = %d",
		(inp == CIF_ISP20_INP_CSI_0) ? "CSI-0" :
			((inp == CIF_ISP20_INP_CSI_1) ? "CSI-1" : "?"),
		pps);

	return cif_isp20_pltfrm_l_g_csi_config(
		dev, inp, &pps, csi_config);
}

void cif_isp20_reset_csi_configs(
	struct device *dev,
	enum cif_isp20_inp inp)
{
	struct cif_isp20_pltfrm_data *pdata = dev->platform_data;
	struct cif_isp20_pltfrm_csi_config *cfg = NULL;
	struct list_head *csi_configs;

	if (inp == CIF_ISP20_INP_CSI_0)
		csi_configs = &pdata->csi0_configs;
	else if (inp == CIF_ISP20_INP_CSI_1)
		csi_configs = &pdata->csi1_configs;
	else
		return;

	while (!list_empty(csi_configs)) {
		cfg = list_first_entry(csi_configs,
			struct cif_isp20_pltfrm_csi_config,
			list);
		list_del(&cfg->list);
		kfree(cfg);
	}
}

int cif_isp20_pltfrm_pinctrl_set_state(
	struct device *dev,
	enum cif_isp20_pinctrl_state pinctrl_state)
{
	int ret = 0;
	struct cif_isp20_pltfrm_data *pdata = dev_get_platdata(dev);

	if (IS_ERR_OR_NULL(pdata->pinctrl))
		return 0;

	cif_isp20_pltfrm_pr_dbg(dev,
		"set pinctrl state to %d\n", pinctrl_state);

	if (NULL == pdata) {
		cif_isp20_pltfrm_pr_err(dev,
			"unable to retrieve CIF platform data\n");
		ret = -EINVAL;
		goto err;
	}

	switch (pinctrl_state) {
	case CIF_ISP20_PINCTRL_STATE_SLEEP:
		if (!IS_ERR_OR_NULL(pdata->pins_sleep))
			ret = pinctrl_select_state(pdata->pinctrl,
				pdata->pins_sleep);
		break;
	case CIF_ISP20_PINCTRL_STATE_ACTIVE:
	case CIF_ISP20_PINCTRL_STATE_DEFAULT:
		if (!IS_ERR_OR_NULL(pdata->pins_default))
			ret = pinctrl_select_state(pdata->pinctrl,
				pdata->pins_default);
		break;
	case CIF_ISP20_PINCTRL_STATE_INACTIVE:
		if (!IS_ERR_OR_NULL(pdata->pins_inactive))
			ret = pinctrl_select_state(pdata->pinctrl,
				pdata->pins_inactive);
		break;
	default:
		cif_isp20_pltfrm_pr_err(dev,
			"unknown or unsupported pinctrl state %d\n",
			pinctrl_state);
		ret = -EINVAL;
		goto err;
	}

	if (IS_ERR_VALUE(ret))
		goto err;

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev, "failed with error %d\n", ret);
	return ret;
}

int (*cif_isp20_pltfrm_irq_g_isr(
	struct device *dev,
	enum cif_isp20_irq irq))(void *)
{
	int ret = 0;
	const char *irq_name;
	int pltfrm_irq;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct cif_isp20_pltfrm_data *pdata =
		dev_get_platdata(&pdev->dev);
	u32 i;

	switch (irq) {
	case CIF_ISP20_IRQ_MIPI:
		irq_name = "CIF_ISP20_MIPI_IRQ";
		break;
	case CIF_ISP20_IRQ_ISP:
		irq_name = "CIF_ISP20_ISP_IRQ";
		break;
	case CIF_ISP20_IRQ_MI:
		irq_name = "CIF_ISP20_MI_IRQ";
		break;
	case CIF_ISP20_IRQ_JPE_STATUS:
		irq_name = "CIF_ISP20_JPE_STATUS_IRQ";
		break;
	case CIF_ISP20_IRQ_JPE_ERROR:
		irq_name = "CIF_ISP20_JPE_ERROR_IRQ";
		break;
	default:
		cif_isp20_pltfrm_pr_err(dev,
			"unknown or unsupported IRQ %d\n", irq);
		ret = -EINVAL;
		goto err;
	}

	pltfrm_irq = platform_get_irq_byname(pdev, irq_name);
	if (IS_ERR_VALUE(pltfrm_irq)) {
		ret = pltfrm_irq;
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(pdata->irq_handlers); i++) {
		if (pdata->irq_handlers[i].irq == pltfrm_irq)
			return pdata->irq_handlers[i].isr;
	}
	ret = -EEXIST;

err:
	cif_isp20_pltfrm_pr_err(dev, "failed with error %d\n", ret);
	return ERR_PTR(ret);
}

int cif_isp20_pltfrm_irq_register_isr(
	struct device *dev,
	enum cif_isp20_irq irq,
	int (*isr)(void *cntxt),
	void *cntxt)
{
	int ret = 0;
	unsigned int i;
	int slot = -EINVAL;
	int pltfrm_irq;
	struct platform_device *pdev =
		container_of(dev, struct platform_device, dev);
	struct cif_isp20_pltfrm_data *pdata =
		dev_get_platdata(&pdev->dev);
	bool skip_request_irq = false;
	const char *irq_name;

	switch (irq) {
	case CIF_ISP20_IRQ_MIPI:
		irq_name = "CIF_ISP20_MIPI_IRQ";
		break;
	case CIF_ISP20_IRQ_ISP:
		irq_name = "CIF_ISP20_ISP_IRQ";
		break;
	case CIF_ISP20_IRQ_MI:
		irq_name = "CIF_ISP20_MI_IRQ";
		break;
	case CIF_ISP20_IRQ_JPE_STATUS:
		irq_name = "CIF_ISP20_JPE_STATUS_IRQ";
		break;
	case CIF_ISP20_IRQ_JPE_ERROR:
		irq_name = "CIF_ISP20_JPE_ERROR_IRQ";
		break;
	default:
		cif_isp20_pltfrm_pr_err(dev,
			"unknown or unsupported IRQ %d\n", irq);
		ret = -EINVAL;
		goto err;
	}

	cif_isp20_pltfrm_pr_dbg(dev,
		"registering ISR for IRQ %s\n", irq_name);

	pltfrm_irq = platform_get_irq_byname(pdev, irq_name);
	if (IS_ERR_VALUE(pltfrm_irq)) {
		ret = pltfrm_irq;
		goto err;
	}

	for (i = 0; i < ARRAY_SIZE(pdata->irq_handlers); i++) {
		if (IS_ERR_VALUE(slot) &&
			IS_ERR_VALUE(pdata->irq_handlers[i].irq))
			slot = (int)i;
		if (pdata->irq_handlers[i].irq == pltfrm_irq) {
			cif_isp20_pltfrm_pr_dbg(dev,
				"overwriting ISR for IRQ %s\n", irq_name);
			slot = (int)i;
			skip_request_irq = true;
			break;
		}
	}
	if (IS_ERR_VALUE(slot)) {
		if (NULL == isr)
			return 0;
		cif_isp20_pltfrm_pr_err(dev,
			"cannot register ISR for IRQ %s, too many ISRs already registered\n",
			irq_name);
		ret = -EFAULT;
		goto err;
	}
	pdata->irq_handlers[slot].isr = isr;
	if (NULL == isr) {
		devm_free_irq(dev, pltfrm_irq, pdev);
		pdata->irq_handlers[slot].irq = -EINVAL;
		skip_request_irq = true;
	} else
		pdata->irq_handlers[slot].irq = pltfrm_irq;

	if (!skip_request_irq) {
		ret = devm_request_threaded_irq(dev,
			pltfrm_irq,
			cif_isp20_pltfrm_irq_handler,
			NULL,
			IRQF_DISABLED,
			dev_driver_string(dev),
			dev);
		if (IS_ERR_VALUE(ret))
			goto err;
	}

	return 0;
err:
	cif_isp20_pltfrm_pr_err(dev, "failed with error %d\n", ret);
	return ret;
}

const char *cif_isp20_pltfrm_get_device_type(
	struct device *dev)
{
	return dev->of_node->type;
}

const char *cif_isp20_pltfrm_dev_string(
	struct device *dev)
{
	return dev_driver_string(dev);
}

struct device *cif_isp20_pltfrm_get_img_src_device(
	struct device *dev,
	enum cif_isp20_inp inp)
{
	struct device_node *node = NULL;
	struct device_node *camera_list_node = NULL;
	struct i2c_client *client = NULL;
	int ret = 0;
	u32 index;

	node = of_node_get(dev->of_node);
	if (IS_ERR_OR_NULL(node)) {
		dev_err(dev, "Unable to obtain CIF device node\n");
		ret = -EEXIST;
		goto err;
	}

	if ((inp == CIF_ISP20_INP_CSI_0) ||
		(inp == CIF_ISP20_INP_CSI_1)) {
		if (inp == CIF_ISP20_INP_CSI_0)
			index = 0;
		else
			index = 1;

		camera_list_node = of_parse_phandle(node,
			"intel,camera-modules-attached", index);
		of_node_put(node);
		if (IS_ERR_OR_NULL(camera_list_node)) {
			cif_isp20_pltfrm_pr_err(dev,
				"invalid index %d for property 'intel,camera-modules-attached'\n",
				index);
				ret = -EINVAL;
				goto err;
		}

		if (!strcmp(camera_list_node->type, "v4l2-i2c-subdev")) {
			client = of_find_i2c_device_by_node(camera_list_node);
			of_node_put(camera_list_node);
			if (IS_ERR_OR_NULL(client)) {
				cif_isp20_pltfrm_pr_warn(dev,
					"could not get camera i2c client, maybe not yet created, deferring device probing...\n");
				ret = -EPROBE_DEFER;
				goto err;
			}
		} else {
			cif_isp20_pltfrm_pr_err(dev,
				"device of type %s not supported\n",
				camera_list_node->type);
				ret = -EINVAL;
				goto err;
		}
	} else {
		cif_isp20_pltfrm_pr_err(dev,
			"CIF parallel input is currently not supported\n");
			ret = -EINVAL;
			goto err;
	}

	cif_isp20_pltfrm_pr_dbg(dev,
		"found device %s attached to CIF CSI-%d",
		dev_driver_string(&client->dev), index);

	return &client->dev;
err:
	dev_err(dev, "failed with error %d\n", ret);
	if (!IS_ERR_OR_NULL(client))
		put_device(&client->dev);
	if (!IS_ERR_OR_NULL(camera_list_node))
		of_node_put(camera_list_node);
	return ERR_PTR(ret);
}


void cif_isp20_pltfrm_dev_release(
	struct device *dev)
{
	struct cif_isp20_pltfrm_data *pdata = dev->platform_data;

	cif_isp20_reset_csi_configs(dev, CIF_ISP20_INP_CSI_0);
	cif_isp20_reset_csi_configs(dev, CIF_ISP20_INP_CSI_1);
#ifdef CONFIG_DEBUG_FS
	{
		debugfs_remove(pdata->dbgfs.dir);
		debugfs_remove(pdata->dbgfs.csi0_file);
		debugfs_remove(pdata->dbgfs.csi1_file);
	}
#endif
}

