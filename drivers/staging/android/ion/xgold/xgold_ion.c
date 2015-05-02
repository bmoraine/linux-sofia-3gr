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
#include <linux/of_reserved_mem.h>

#include <linux/xgold_ion.h>
#include <linux/dma-buf.h>
#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/vvpu_vbpipe.h>
#endif
static struct ion_device *xgold_ion_idev;
struct ion_mapper *xgold_user_mapper;
struct ion_heap **heaps;

struct ion_client *xgold_ion_client_create(const char *name)
{
	struct ion_device *dev = xgold_ion_idev;

	if (dev == NULL) {
		pr_err("%s: no ion device found\n", __func__);
		return NULL;
	}

	return ion_client_create(dev, name);
}
EXPORT_SYMBOL(xgold_ion_client_create);


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
	struct ion_handle *handle;
	struct ion_phys_data pdata;
	struct ion_share_id_data sdata;
	struct dma_buf *dmabuf;
	int fd = 0;

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
	case ION_IOC_GET_PHYS:
		if (copy_from_user(&pdata, (void __user *)arg,
				   sizeof(struct ion_phys_data)))
			return -EFAULT;
		handle = ion_handle_get_by_id(client, pdata.handle);
		if (IS_ERR(handle))
			return PTR_ERR(handle);
		ret = ion_phys(client, handle, &pdata.phys,
			       (size_t *)&pdata.size);
		ion_handle_put(handle);
		if (ret < 0)
			return ret;
		if (copy_to_user((void __user *)arg, &pdata,
				 sizeof(struct ion_phys_data)))
			return -EFAULT;
		break;
	case ION_IOC_GET_SHARE_ID:
		if (copy_from_user(&sdata, (void __user *)arg,
				   sizeof(struct ion_share_id_data)))
			return -EFAULT;
		dmabuf = dma_buf_get(sdata.fd);
		if (IS_ERR(dmabuf))
			return PTR_ERR(dmabuf);
		sdata.id = (unsigned int)dmabuf;
		if (copy_to_user((void __user *)arg, &sdata,
				 sizeof(struct ion_share_id_data)))
			return -EFAULT;
		break;
	case ION_IOC_SHARE_BY_ID:
		if (copy_from_user(&sdata, (void __user *)arg,
				   sizeof(struct ion_share_id_data)))
			return -EFAULT;
		fd = dma_buf_fd((struct dma_buf *)sdata.id, O_CLOEXEC);
		if (fd < 0)
			return fd;
		sdata.fd = fd;
		if (copy_to_user((void __user *)arg, &sdata,
				 sizeof(struct ion_share_id_data)))
			return -EFAULT;
		break;
	case ION_IOC_CLEAN_CACHES:
	case ION_IOC_INV_CACHES:
	case ION_IOC_CLEAN_INV_CACHES:
		break;
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
	}, {
		.name = "carveout-heap",
		.id = ION_HEAP_TYPE_CARVEOUT,
	}, {
		.name = "cma-heap",
		.id = ION_HEAP_TYPE_DMA,
	}, {
		.name = "secured-heap",
		.id = ION_HEAP_TYPE_SECURE,
	},
};

#include <linux/of.h>

int xgold_ion_of_heap(struct ion_platform_heap *myheap,
		struct device_node *node)
{
	int ret = 0;
	int itype = 0;

	for (itype = 0;	itype < ARRAY_SIZE(ion_heap_type_name);	itype++) {
		if (!strcmp(ion_heap_type_name[itype].name, node->name)) {
			struct cma *cma_area;
			phys_addr_t base = 0;
			phys_addr_t size = 0;

			myheap->name = node->name;

			myheap->align = 0x100000;
			myheap->id = ion_heap_type_name[itype].id;

			if (!strcmp("system-heap", node->name)) {
				myheap->type = ION_HEAP_TYPE_SYSTEM;
				return 1;
			}

			ret = of_get_reserved_memory_region(node,
					&size, &base, &cma_area);
			if (ret) {
				pr_err("xg_ion Can't find memory def! Skip %s\n",
						node->name);
				continue;
			}

			myheap->base = base;
			myheap->size = size;

			if (cma_area) {
				myheap->priv2 = cma_area;
				myheap->type = ION_HEAP_TYPE_DMA;
			} else if (myheap->base && !cma_area)
				myheap->type = ION_HEAP_TYPE_CARVEOUT;
			else if (size)
				myheap->type = ION_HEAP_TYPE_SYSTEM;
			else {
				pr_err("xg_ion unknow memory type! Skip %s\n",
						node->name);
				continue;
			}

			pr_info("xg_ion heap %s base:%pa length:0x%08x type %d\n",
					node->name, &myheap->base,
					myheap->size, myheap->type);
			return 1;
		}
	}
	return 0;
}


static struct ion_platform_data *xgold_ion_of(struct device_node *node)
{
	struct ion_platform_data *pdata;
	int iheap = 0;
	struct device_node *child;
	struct ion_platform_heap *myheap;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return NULL;

	pdata->nr = of_get_child_count(node);
again:
	pdata->heaps = kcalloc(pdata->nr, sizeof(*myheap), GFP_KERNEL);
	for_each_child_of_node(node, child) {
		iheap += xgold_ion_of_heap(&pdata->heaps[iheap], child);
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
		if (pdata == NULL)
			return -EINVAL;
		pdev->dev.platform_data = pdata;
	}

	heaps = kzalloc(sizeof(struct ion_heap *) * pdata->nr, GFP_KERNEL);
	if (!heaps)
		return -ENOMEM;

	idev = ion_device_create(xgold_ion_ioctl);
	if (IS_ERR_OR_NULL(idev)) {
		kfree(heaps);
		return PTR_ERR(idev);
	}
	xgold_ion_idev = idev;
	/* create the heaps as specified in the board file */
	for (i = 0; i < pdata->nr; i++) {
		struct ion_platform_heap *heap_data =  &pdata->heaps[i];
		heap_data->priv = &pdev->dev;
		heaps[i] = ion_heap_create(heap_data);
		if (IS_ERR_OR_NULL(heaps[i])) {
			err = PTR_ERR(heaps[i]);
			goto err;
		}
		ion_device_add_heap(idev, heaps[i]);
	}
	platform_set_drvdata(pdev, idev);

	xgold_ion_handler_init(pdev->dev.of_node, idev, pdata);

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

subsys_initcall(ion_init);
module_exit(ion_exit);

