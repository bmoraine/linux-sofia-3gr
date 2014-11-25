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
#include <sofia/nk_sofia_bridge.h>
#include <sofia/mv_gal.h>

#define NVM_ADDR_SHIFT	(0x3D100000 - 0x3C000000)
#define SEC_PACK_SIZE 2048

struct vmodem_drvdata {
	struct device *dev;
	int modem_state;
	unsigned int running_guest;
	struct miscdevice devfile;
	struct work_struct work;
	struct workqueue_struct *wq;
	void __iomem *mex_buffer;
};

struct shared_secure_data {
	unsigned int vm_id;
	unsigned int image_startaddr;
	char secpack[SEC_PACK_SIZE];
};


struct shared_secure_data *p_shared_mem_addr;
static unsigned int mex_loadaddr;
static unsigned int mex_loadsize;
vmm_paddr_t p_shared_mem_phy_addr;


static void linux2secvm_vlink_init(void)
{
	vmm_paddr_t paddr;
	vmm_id_t vm_id;
	struct vmm_vlink *vlink;
	char *vlink_name = "vlinux2secvm";
	vm_id = mv_gal_os_id();
	paddr = 0;
	do {
		paddr = mv_gal_vlink_lookup(vlink_name, paddr);
		mv_gal_printk("vlink lookup returns: 0x%X\n", paddr);
		vlink = (struct vmm_vlink *)mv_gal_ptov(paddr);
		if (vlink->s_id == vm_id)
			break;
	} while (paddr);

	if (!paddr) {
		mv_gal_printk("Cannot find %s\n", vlink_name);
		return;
	}
	p_shared_mem_phy_addr = mv_shared_mem_alloc(paddr, 0,
			sizeof(struct shared_secure_data));
	p_shared_mem_addr =
		(struct shared_secure_data *)mv_gal_ptov(p_shared_mem_phy_addr);
}

static void modem_state_work(struct work_struct *ws)
{
	struct vmodem_drvdata *pdata =
		container_of(ws, struct vmodem_drvdata, work);
	int new_state = pdata->modem_state;
	char *on[2] = { "MODEM_STATE=ON", NULL };
	char *off[2] = { "MODEM_STATE=OFF", NULL };
	char **uevent_envp = NULL;

	pdata->running_guest = mv_get_running_guests();
	if ((pdata->running_guest & 0x2) == 0x2) {
		new_state = 1;
		uevent_envp = on;
	} else if ((pdata->running_guest & 0x2) == 0) {
		new_state = 0;
		uevent_envp = off;
	}

	if (new_state != pdata->modem_state) {
		pdata->modem_state = new_state;
		kobject_uevent_env(&pdata->dev->kobj, KOBJ_CHANGE, uevent_envp);
		/*sysfs_notify(&(pdata->dev->kobj), NULL, "vmodem");*/
	}
}

static void modem_state_sysconf_hdl(void *arg, NkXIrq xirq)
{
	struct vmodem_drvdata *p = (struct vmodem_drvdata *)arg;
	queue_work(p->wq, &p->work);
}

static ssize_t modem_sys_state_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct vmodem_drvdata *p = dev_get_drvdata(dev);
	return sprintf(buf, "%s\n",
			p->modem_state ? "On" : "Off");
}
/*
 * This function will be revmoved once the vvfs driver is ready
*/
static int reload_nvm(struct vmodem_drvdata *p)
{
	mm_segment_t fs = get_fs();
	struct file *fd;
	set_fs(get_fs());
	fd = filp_open(
		"/dev/block/platform/soc0/e0000000.noc/by-name/ImcPartID022",
		(O_RDONLY | O_SYNC), 0);
	set_fs(fs);
	if (unlikely(IS_ERR(fd))) {
		/* Better use system events to get notified
		 * on device removal/creation */
		return -EINVAL;
	}
	fs = get_fs();
	set_fs(get_ds());
	fd->f_op->read(fd, (p->mex_buffer + NVM_ADDR_SHIFT),
			0x180000, &(fd->f_pos));
	set_fs(fs);
	filp_close(fd, NULL);
	return 1;
}

static int load_secpack(unsigned char *buf)
{
	mm_segment_t fs = get_fs();
	struct file *fd;
	set_fs(get_fs());
	fd = filp_open(
		"/data/modem.fls_ID0_CUST_SecureBlock.bin",
		(O_RDONLY | O_SYNC), 0);
	set_fs(fs);
	if (unlikely(IS_ERR(fd))) {
		/* Better use system events to get notified
		 * on device removal/creation */
		return -EINVAL;
	}
	fs = get_fs();
	set_fs(get_ds());
	fd->f_op->read(fd, buf,
			SEC_PACK_SIZE, &(fd->f_pos));
	set_fs(fs);
	filp_close(fd, NULL);
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
		load_secpack(p_shared_mem_addr->secpack);
		mv_start_vm(1);
	} else {
		mv_stop_vm(1);
	}

	return count;
}

static DEVICE_ATTR(modem_state, 0664,
		modem_sys_state_show, modem_sys_state_store);


static ssize_t modem_write(struct file *file, const char __user *buf,
		size_t nbytes, loff_t *ppos)
{
	struct vmodem_drvdata *p = file->private_data;

	if (copy_from_user(p->mex_buffer + *ppos, buf, nbytes))
		return -EFAULT;
	else {
		*ppos += nbytes;
		return nbytes;
	}
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
	struct resource *res;
	NkXIrqId sysconf_id;
	int ret;
	/* Allocate driver data record */
	pdata = devm_kzalloc(dev,
			sizeof(struct vmodem_drvdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "Couldn't allocate driver data record\n");
		return -ENOMEM;
	}
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

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "could not determine modem address\n");
		return -EINVAL;
	}

	pdata->mex_buffer = devm_ioremap_resource(dev, res);
	if (IS_ERR_OR_NULL(pdata->mex_buffer)) {
		dev_err(dev, "can't ioremap modem address\n");
		return -ENOMEM;
	}

	/*
	 * /sys/bus/platform/drivers/vmodem/vmodem.173/modem_state
	 */
	if (device_create_file(&pdev->dev, &dev_attr_modem_state))
		return -ENOMEM;

	INIT_WORK(&pdata->work, modem_state_work);

	/* Create workqueues */
	pdata->wq = alloc_ordered_workqueue(
		"vmodem_wq", WQ_NON_REENTRANT | WQ_HIGHPRI);

	sysconf_id =
		nkops.nk_xirq_attach(NK_XIRQ_SYSCONF,
				modem_state_sysconf_hdl, pdata);
	if (!sysconf_id) {
		dev_err(dev, "nk_xirq_attach failed\n");
		return -EINVAL;
	}
	mv_svc_security_getvm_loadinfo(1, &mex_loadaddr, &mex_loadsize);
	linux2secvm_vlink_init();
	p_shared_mem_addr->vm_id = 1;
	p_shared_mem_addr->image_startaddr = mex_loadaddr;
	dev_dbg(dev, "modem device initialized");
	return 0;
}

static int vmodem_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct vmodem_drvdata *pdata =
		(struct vmodem_drvdata *)platform_get_drvdata(pdev);

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


