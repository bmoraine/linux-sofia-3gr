/* Copyright (C) 2013 Intel Mobile Communications GmbH
 * *
 * * This software is licensed under the terms of the GNU General Public
 * * License version 2, as published by the Free Software Foundation, and
 * * may be copied, distributed, and modified under those terms.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * * GNU General Public License for more details.
 * */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/notifier.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/io.h>

#include <sofia/vmcalls.h>
#include <sofia/mv_svc_hypercalls.h>
#include <linux/proc_fs.h>
#include <sofia/mv_gal.h>

#include "vdump.h"

#define NVM_PARTITION_NO	3
#define NVM_ADDR_SHIFT		(0x3D100000 - 0x3C000000)
#define SEC_PACK_SIZE 		2048

struct vmodem_drvdata {
	struct device *dev;
	void *hirq_info;
	int modem_state;
	unsigned int running_guest;
	struct miscdevice devfile;
	struct work_struct work;
	struct workqueue_struct *wq;
	struct resource *res;
};

struct shared_secure_data {
	unsigned int vm_id;
	unsigned int image_startaddr;
	char secpack[SEC_PACK_SIZE];
};

struct nvm_partition_info {
	char* partition;
	size_t partition_size;
};

static struct nvm_partition_info nvm_partition[] = {
	{"/dev/block/platform/soc0/e0000000.noc/by-name/ImcPartID022", 262144},
	{"/dev/block/platform/soc0/e0000000.noc/by-name/ImcPartID023", 262144},
	{"/dev/block/platform/soc0/e0000000.noc/by-name/ImcPartID024", 524288},
};

struct shared_secure_data *p_shared_mem_addr;
static unsigned int mex_loadaddr;
static unsigned int mex_loadsize;
vmm_paddr_t p_shared_mem_phy_addr;
static struct vmodem_drvdata *g_vm_data;

static void modem_state_work(struct work_struct *ws)
{
	struct vmodem_drvdata *pdata =
		container_of(ws, struct vmodem_drvdata, work);
	int new_state = pdata->modem_state;
	struct device *misc_dev = pdata->devfile.this_device;
	char *on[2] = { "MODEM_STATE=ON", NULL };
	char *off[2] = { "MODEM_STATE=OFF", NULL };
	char *trap[2] = { "MODEM_STATE=TRAP", NULL };
	char **uevent_envp = NULL;

	pdata->running_guest = mv_get_running_guests();
	if ((pdata->running_guest & 0x2) == 0x2) {
		new_state = 1;
		uevent_envp = on;
	} else if ((pdata->running_guest & 0x2) == 0) {
		if(vdump_modem()) {
			new_state = 3;
			uevent_envp = trap;
		}
		else {
			new_state = 0;
			uevent_envp = off;
		}
	}

	if (new_state != pdata->modem_state) {
		pdata->modem_state = new_state;
		kobject_uevent_env(&misc_dev->kobj,
				KOBJ_CHANGE, uevent_envp);
		/*sysfs_notify(&(pdata->dev->kobj), NULL, "vmodem");*/
	}
}

static irqreturn_t modem_state_sysconf_hdl(int xirq, void *arg)
{
	struct vmodem_drvdata *p = (struct vmodem_drvdata *)arg;
	queue_work(p->wq, &p->work);
	return IRQ_HANDLED;
}

void schedule_vmodem_workqueue(void)
{
	queue_work(g_vm_data->wq, &g_vm_data->work);
}
EXPORT_SYMBOL(schedule_vmodem_workqueue);

static ssize_t modem_sys_state_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int ret = 0;
	struct vmodem_drvdata *p = dev_get_drvdata(dev);
	switch(p->modem_state)
	{
		case 0:
			ret = sprintf(buf, "%s\n", "Off");
			break;
		case 1:
			ret = sprintf(buf, "%s\n", "On");
			break;
		case 2:
			ret = sprintf(buf, "%s\n", "Trap");
			break;
	}
	return ret;
}
/*
 * This function will be revmoved once the vvfs driver is ready
*/
static int reload_nvm(struct vmodem_drvdata *p)
{
	int i = 0;
	void * __iomem mex_buffer;
	mex_buffer = ioremap(p->res->start + NVM_ADDR_SHIFT, 0x100000);
	if (IS_ERR_OR_NULL(mex_buffer)) {
		dev_err(p->dev, "ioremap failed\n");
		return -ENOMEM;
	}
	for(i = 0; i < NVM_PARTITION_NO; i++) {
		mm_segment_t fs = get_fs();
		struct file *fd;
		set_fs(get_fs());
		fd = filp_open(nvm_partition[i].partition,
				(O_RDONLY | O_SYNC), 0);
		set_fs(fs);
		if (unlikely(IS_ERR(fd))) {
			/* Better use system events to get notified
			 * on device removal/creation */
			return -EINVAL;
		}
		fs = get_fs();
		set_fs(get_ds());

		fd->f_op->read(fd, mex_buffer,
				nvm_partition[i].partition_size,
				&(fd->f_pos));
		mex_buffer += nvm_partition[i].partition_size;
		set_fs(fs);
		filp_close(fd, NULL);
	}
	iounmap(mex_buffer);
	return 1;
}


static ssize_t modem_sys_state_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t count)
{
	int en, ret;
	struct vmodem_drvdata *p = dev_get_drvdata(dev);
	ret = sscanf(buf, "%d", &en);
	if (ret != 1) {
		dev_err(dev, "invalid input\n");
		return -EINVAL;
	}
	dev_dbg(dev, "%s sets modem %s\n", __func__, en ? "On" : "Off");
	if (en != 0) {
		reload_nvm(p);
		mv_vm_start(1);
	} else {
		mv_vm_stop(1);
	}

	return count;
}

static DEVICE_ATTR(modem_state, 0664,
		modem_sys_state_show, modem_sys_state_store);


static ssize_t modem_write(struct file *file, const char __user *buf,
		size_t nbytes, loff_t *ppos)
{
	struct vmodem_drvdata *p = file->private_data;
	void * __iomem mex_buffer;
	int ret =  0;
	mex_buffer = ioremap(p->res->start + *ppos, nbytes);
	if (IS_ERR_OR_NULL(mex_buffer)) {
		dev_err(p->dev, "ioremap failed\n");
		return -ENOMEM;
	}

	if (copy_from_user(mex_buffer, buf, nbytes))
		ret = -EFAULT;
	else {
		*ppos += nbytes;
		ret = nbytes;
	}
	iounmap(mex_buffer);
	return ret;
}

static int modem_open(struct inode *inode, struct file *file)
{
	struct vmodem_drvdata *p = container_of(file->private_data, struct
			vmodem_drvdata, devfile);
	file->private_data = p;
	return 0;
}

static const struct file_operations modem_fops = {
	.owner   = THIS_MODULE,
	.open	 = modem_open,
	.write   = modem_write,
};

int vmodem_probe(struct platform_device *pdev)
{
	struct vmodem_drvdata *pdata;
	struct device *dev = &pdev->dev;
	int ret;
	/* Allocate driver data record */
	pdata = devm_kzalloc(dev,
			sizeof(struct vmodem_drvdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Couldn't allocate driver data record\n");
		return -ENOMEM;
	}
	g_vm_data = pdata;
	pdata->modem_state = 0;
	pdata->dev = &pdev->dev;
	platform_set_drvdata(pdev, pdata);

	/* Register device */
	pdata->devfile.minor = MISC_DYNAMIC_MINOR;
	pdata->devfile.name = "vmodem";
	pdata->devfile.fops = &modem_fops;
	pdata->devfile.parent = NULL;

	ret = misc_register(&pdata->devfile);
	if (ret) {
		dev_err(dev, "failed to register misc device.\n");
		return -EINVAL;
	}

	dev_set_drvdata(pdata->devfile.this_device, pdata);

	/* Allocate resource */
	pdata->res = devm_kzalloc(dev, sizeof(struct resource), GFP_KERNEL);
	if (!pdata->res) {
		dev_err(dev, "Couldn't allocate resource\n");
		return -ENOMEM;
	}

	/*
	 * Get the loading vm load info
	 */
	mv_svc_security_getvm_loadinfo(1, &mex_loadaddr, &mex_loadsize);
	pdata->res->start = mex_loadaddr;
	pdata->res->end = mex_loadaddr + mex_loadsize;
	pdata->res->flags = IORESOURCE_MEM;

	/*
	 * /sys/class/misc/vmodem/modem_state
	 */
	if (device_create_file(pdata->devfile.this_device,
				&dev_attr_modem_state))
		return -ENOMEM;

	INIT_WORK(&pdata->work, modem_state_work);

	/* Create workqueues */
	pdata->wq = alloc_ordered_workqueue(
		"vmodem_wq", WQ_NON_REENTRANT | WQ_HIGHPRI);
	if (!pdata->wq) {
		dev_err(dev, "unable to allocate workqueue\n");
		return -ENOMEM;
	}

	pdata->hirq_info = mv_gal_register_hirq_callback(512,
		modem_state_sysconf_hdl, pdata);
	if (!pdata->hirq_info) {
		dev_err(dev, "unable to register hirq\n");
		destroy_workqueue(pdata->wq);
		return -EBUSY;
	}

	/*
	  linux2secvm_vlink_init();
	  p_shared_mem_addr->vm_id = 1;
	  p_shared_mem_addr->image_startaddr = mex_loadaddr;
	*/
	dev_dbg(dev, "modem device initialized");
	return 0;
}

static int vmodem_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct vmodem_drvdata *pdata =
		(struct vmodem_drvdata *)platform_get_drvdata(pdev);

	mv_gal_hirq_detach(pdata->hirq_info);
	flush_workqueue(pdata->wq);
	destroy_workqueue(pdata->wq);
	misc_deregister(&pdata->devfile);
	devm_kfree(&pdev->dev, pdata);
	return ret;
}

static struct of_device_id vmodem_of_match[] = {
	{ .compatible = "intel,vmodem", },
	{ },
};

static struct platform_driver vmodem_driver = {
	.probe = vmodem_probe,
	.remove = vmodem_remove,
	.driver = {
		.name = "vmodem",
		.owner = THIS_MODULE,
		.of_match_table = vmodem_of_match,
	},
};

int __init modem_init(void)
{
	return platform_driver_register(&vmodem_driver);
}

static void __exit modem_exit(void)
{
	platform_driver_unregister(&vmodem_driver);
}

module_init(modem_init);
module_exit(modem_exit);

MODULE_DESCRIPTION("Modem Runtime Loading");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL");


