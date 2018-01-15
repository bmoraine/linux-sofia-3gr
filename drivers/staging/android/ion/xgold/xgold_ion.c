/*
 *
 * Copyright (C) 2014 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "../ion_priv.h"
#include <linux/module.h>
#include <linux/of.h>
#include <linux/compat.h>

#include <linux/xgold_ion.h>
#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/vvpu_vbpipe.h>
#endif

struct ion_mapper *xgold_user_mapper;
struct ion_heap **heaps;

static int xgold_ion_get_param(struct ion_client *client,
					unsigned int cmd,
					unsigned long arg)
{
	struct xgold_ion_get_params_data *data;
	struct ion_handle *handle;
	ion_phys_addr_t paddr;
	size_t size;
	struct ion_buffer *buffer;
	int ret = 0;

	if (is_compat_task()) {
		data = compat_xgold_ion_get_param(arg);
		if (IS_ERR_OR_NULL(data))
			return -EFAULT;
	} else {
		data = kmalloc(sizeof(struct xgold_ion_get_params_data),
				GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			ret = -EFAULT;
			goto free_data;
		}
	}

	handle = ion_handle_get_by_id(client, data->handle);
	buffer = ion_handle_buffer(handle);
	ion_phys(client, handle, &paddr, &size);
	ion_free(client, handle);
	data->addr = paddr;
	data->size = size;

	if (is_compat_task()) {
		return compat_put_xgold_ion_custom_data(arg, data);

	} else {
		if (copy_to_user((void __user *)arg, data, sizeof(*data))) {
			ret = -EFAULT;
			goto free_data;
		}
		kfree(data);
	}

	return 0;
free_data:
	kfree(data);
	return ret;
}


#ifdef CONFIG_X86_INTEL_SOFIA
static int xgold_ion_alloc_secure(struct ion_client *client,
	unsigned int cmd,
	unsigned long arg)
{
	struct xgold_ion_get_params_data *data = NULL;
	struct device *dev;
	struct vvpu_secvm_cmd vvpu_cmd;
	int vvpu_ret;
	int ret = 0;

	dev = ion_struct_device_from_client(client);
	if (is_compat_task()) {
		data = compat_xgold_ion_get_param(arg);
		if (IS_ERR_OR_NULL(data))
			return -EFAULT;
	} else {
		data = kmalloc(sizeof(struct xgold_ion_get_params_data),
				GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		if (copy_from_user(data, (void __user *)arg,
					sizeof(*data))) {
			ret = -EFAULT;
			goto free_data;
		}
	}

	/* call into secure VM to allocate a secure video buffer */
	memset(&vvpu_cmd, 0, sizeof(cmd));

	vvpu_cmd.payload[0] = VVPU_VTYPE_MEM;
	vvpu_cmd.payload[1] = VVPU_VOP_MEM_ALLOC;
	vvpu_cmd.payload[2] = 0;
	vvpu_cmd.payload[3] = 0;
	vvpu_cmd.payload[4] = data->size;
	vvpu_cmd.payload[5] = 0;

	/* execute command */
	vvpu_ret = vvpu_call(dev, &vvpu_cmd);

	if (vvpu_ret == 0)
		dev_err(dev, "error allocating secure memory");
	else {
		data->size = (unsigned int) vvpu_cmd.payload[4];
		data->addr = (unsigned int) vvpu_cmd.payload[5];

		dev_info(dev, "ion_alloc_secure() 0x%lx / %zu",
			data->addr, data->size);
	}

	if (is_compat_task()) {
		return compat_put_xgold_ion_custom_data(arg, data);
	} else {
		if (copy_to_user((void __user *) arg, data, sizeof(*data))) {
			ret = -EFAULT;
			goto free_data;
		}

		kfree(data);
	}

	return 0;

free_data:
	kfree(data);
	return ret;
}

static int xgold_ion_free_secure(struct ion_client *client,
	unsigned int cmd,
	unsigned long arg)
{
	struct xgold_ion_get_params_data *data;
	struct device *dev;
	struct vvpu_secvm_cmd vvpu_cmd;
	int vvpu_ret;
	int ret = 0;

	dev = ion_struct_device_from_client(client);
	if (is_compat_task()) {
		data = compat_xgold_ion_get_param(arg);
		if (IS_ERR_OR_NULL(data))
			return -EFAULT;
	} else {
		data = kmalloc(sizeof(struct xgold_ion_get_params_data),
				GFP_KERNEL);
		if (!data)
			return -ENOMEM;

		if (copy_from_user(data, (void __user *)arg, sizeof(*data))) {
			ret = -EFAULT;
			goto free_data;
		}
	}

	/* call into secure VM to allocate a secure video buffer */
	memset(&vvpu_cmd, 0, sizeof(cmd));

	vvpu_cmd.payload[0] = VVPU_VTYPE_MEM;
	vvpu_cmd.payload[1] = VVPU_VOP_MEM_FREE;
	vvpu_cmd.payload[2] = 0;
	vvpu_cmd.payload[3] = 0;
	vvpu_cmd.payload[4] = data->size;
	vvpu_cmd.payload[5] = data->addr;

	/* execute command */
	vvpu_ret = vvpu_call(dev, &vvpu_cmd);

	if (vvpu_ret == 0)
		dev_err(dev, "error freeing secure memory");

	/* leave data.size and data.addr alone */
	if (is_compat_task()) {
		return compat_put_xgold_ion_custom_data(arg, data);

	} else {
		if (copy_to_user((void __user *) arg, data, sizeof(*data))) {
			ret = -EFAULT;
			goto free_data;
		}
		kfree(data);
	}

	return 0;
free_data:
	kfree(data);
	return ret;
}
#endif

static long xgold_ion_ioctl(struct ion_client *client,
				   unsigned int cmd,
				   unsigned long arg)
{
	int ret = -ENOTTY;

	switch (cmd) {
	case XGOLD_ION_GET_PARAM:
		ret = xgold_ion_get_param(client, cmd, arg);
		break;
#ifdef CONFIG_X86_INTEL_SOFIA
	case XGOLD_ION_ALLOC_SECURE:
		ret = xgold_ion_alloc_secure(client, cmd, arg);
		break;
	case XGOLD_ION_FREE_SECURE:
		ret = xgold_ion_free_secure(client, cmd, arg);
		break;
#endif
	default:
		pr_err("Unknown custom ioctl\n");
		return -ENOTTY;
	}
	return ret;
}

struct ion_type_name {
	const char *name;
	unsigned int id;
	unsigned int type;
};

static struct ion_type_name ion_heap_type_name[] = {
	{
		.name = "system-heap",
		.id = ION_HEAP_TYPE_SYSTEM,
		.type = ION_HEAP_TYPE_SYSTEM
	}, {
		.name = "carveout-heap",
		.id = ION_HEAP_TYPE_CARVEOUT,
		.type = ION_HEAP_TYPE_CARVEOUT
	}, {
		.name = "cma-heap",
		.id = ION_HEAP_TYPE_DMA,
		.type = ION_HEAP_TYPE_DMA
	}, {
		.name = "display-heap",
		.id = ION_HEAP_TYPE_DISPLAY_CARVEOUT,
		.type = ION_HEAP_TYPE_CARVEOUT
	}, {
		.name = "video-heap",
		.id = ION_HEAP_TYPE_VIDEO_CARVEOUT,
		.type = ION_HEAP_TYPE_CARVEOUT
	},
};

#include <linux/of.h>
#define ION_PROP_HEAP_MEM	"intel,heapmem"

static struct ion_platform_data *xgold_ion_of(struct device_node *node)
{
	struct ion_platform_data *pdata;
	int ret = 0;
	int itype = 0;
	int iheap = 0;
	struct device_node *child;
	struct ion_platform_heap *myheap;
	u32 array[2];
	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);

	pdata->nr = of_get_child_count(node);
again:
	pdata->heaps = kcalloc(pdata->nr, sizeof(*myheap), GFP_KERNEL);
	for_each_child_of_node(node, child) {
		for (itype = 0;
				itype < ARRAY_SIZE(ion_heap_type_name);
				itype++) {
			if (!strcmp(ion_heap_type_name[itype].name,
							child->name)) {
				myheap = &pdata->heaps[iheap];
				pr_debug("%s: heap %p\n", __func__, myheap);
				myheap->name = child->name;
				myheap->type = ion_heap_type_name[itype].type;
				myheap->align = 0x100000;
				myheap->id = ion_heap_type_name[itype].id;
				ret = of_property_read_u32_array(child,
						ION_PROP_HEAP_MEM, array, 2);
				if (ret) {
					pr_err("xg_ion Can't read property:%s, skipping node %s\n",
						ION_PROP_HEAP_MEM, child->name);
					continue;
				}
				myheap->base = array[0];
				myheap->size = array[1] - 1;
				pr_info("xg_ion heap %d %s base:0x%08x length:0x%08x\n",
							iheap, child->name,
							array[0], array[1]);
				iheap++;
			}
		}
	}

	if (pdata->nr != iheap) {
		pdata->nr = iheap;
		iheap = 0;
		kfree(pdata->heaps);
		pr_err("%s: mismatch, repeating\n", __func__);
		goto again;
	}

	return pdata;
}

int xgold_ion_probe(struct platform_device *pdev)
{
	int err;
	int i;
	struct ion_device *idev;
	struct ion_platform_data *pdata = pdev->dev.platform_data;

	if (pdata == NULL) {
		pdata = xgold_ion_of(pdev->dev.of_node);
		pdev->dev.platform_data = pdata;
	}

	heaps = kzalloc(sizeof(struct ion_heap *) * pdata->nr, GFP_KERNEL);

	idev = ion_device_create(xgold_ion_ioctl);
	if (IS_ERR_OR_NULL(idev)) {
		kfree(heaps);
		return PTR_ERR(idev);
	}

	/* create the heaps as specified in the board file */
	for (i = 0; i < pdata->nr; i++) {
		struct ion_platform_heap *heap_data =  &pdata->heaps[i];

		heaps[i] = ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			goto err;
		}
		ion_device_add_heap(idev, heaps[i]);
	}
	platform_set_drvdata(pdev, idev);

	xgold_ion_handler_init(pdev->dev.of_node, idev);

	return 0;
err:
	for (i = 0; i < pdata->nr; i++) {
		if (heaps[i])
			ion_heap_destroy(heaps[i]);
	}
	kfree(heaps);
	return err;
}

int xgold_ion_remove(struct platform_device *pdev)
{
	struct ion_platform_data *pdata = pdev->dev.platform_data;
	struct ion_device *idev = platform_get_drvdata(pdev);
	int i;

	xgold_ion_handler_exit();

	ion_device_destroy(idev);
	for (i = 0; i < pdata->nr; i++)
		ion_heap_destroy(heaps[i]);
	kfree(heaps);
	return 0;
}
static const struct of_device_id xgold_ion[] = {
	{
		.compatible = "intel,ion",
	},
	{}
};

MODULE_DEVICE_TABLE(of, xgold_ion);

static struct platform_driver ion_driver = {
	.probe = xgold_ion_probe,
	.remove = xgold_ion_remove,
	.driver = {
		.name = "xgold-ion",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(xgold_ion),
	}
};

static int ion_init(void)
{
	return platform_driver_register(&ion_driver);
}

static void ion_exit(void)
{
	platform_driver_unregister(&ion_driver);
}

module_init(ion_init);
module_exit(ion_exit);

