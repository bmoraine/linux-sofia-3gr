/* ----------------------------------------------------------------------------
 *  Copyright (C) 2014 Intel Mobile Communications GmbH

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

  ---------------------------------------------------------------------------*/
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/irqchip.h>
#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <sofia/mv_svc_hypercalls.h>
#include <sofia/mv_gal.h>
#include <sofia/pal_shared_data.h>
#include <sofia/vmm_pmic.h>

static struct mutex pmic_access_mutex;
static DECLARE_WAIT_QUEUE_HEAD(pmic_wait_queue);
static int32_t vmm_pmic_reg_access_initialised;
static int32_t vmm_pmic_access_done;

static struct pmic_access_shared_data *vmm_pmic_get_shared_data(void)
{
	struct vmm_shared_data *vmm_shared_data;
	struct pal_shared_data *pal_shared_data;
	vmm_shared_data = mv_gal_get_shared_data();
	pal_shared_data = (struct pal_shared_data *)
			(&vmm_shared_data->pal_shared_mem_data);
	return &(pal_shared_data->pmic_access_shared_data);
}

int32_t vmm_pmic_reg_write_by_range(uint32_t reg_address,
				uint8_t *data, uint32_t size_in_byte)
{
	struct pmic_access_shared_data *pmic_access_shared_data;
	int32_t result = 0;

	mutex_lock(&pmic_access_mutex);

	if (size_in_byte + 1 > PMIC_ACCESS_MAX_SIZE) {
		pr_err("%s: transfer size not supported (%d > %d)\n", __func__,
				size_in_byte + 1, PMIC_ACCESS_MAX_SIZE);
		result = -1;
		goto pmic_write_end;
	}

	pmic_access_shared_data = vmm_pmic_get_shared_data();
	pmic_access_shared_data->data[0] = (uint8_t)(reg_address & 0xFF);
	memcpy(&(pmic_access_shared_data->data[1]), data, size_in_byte);
	vmm_pmic_access_done = 0;
	if (mv_svc_pmic_reg_access(PMIC_REG_WRITE, reg_address,
						size_in_byte + 1)) {
		pr_err("%s: mv_svc_pmic_reg_access failed\n", __func__);
		result = -1;
		goto pmic_write_end;
	}
	if (wait_event_interruptible(pmic_wait_queue, vmm_pmic_access_done)) {
		pr_err("%s: wait_event interrupted by signal\n", __func__);
		result = -ERESTARTSYS;
		goto pmic_write_end;
	}

	if (pmic_access_shared_data->status) {
		pr_err("%s: pmic shared data status is wrong\n", __func__);
		result = -1;
		goto pmic_write_end;
	}
pmic_write_end:
	pr_debug("%s %s on CPU%d\n", __func__, result < 0 ? "FAILS" : "PASS",
			raw_smp_processor_id());
	mutex_unlock(&pmic_access_mutex);
	return result;
}
EXPORT_SYMBOL(vmm_pmic_reg_write_by_range);

int32_t vmm_pmic_reg_write(uint32_t reg_address, uint32_t reg_val)
{
	return vmm_pmic_reg_write_by_range(reg_address, (uint8_t *)&reg_val, 1);
}
EXPORT_SYMBOL(vmm_pmic_reg_write);

int32_t vmm_pmic_reg_read_by_range(uint32_t reg_address,
					uint8_t *data, uint32_t size_in_byte)
{
	struct pmic_access_shared_data *pmic_access_shared_data;
	int32_t result = 0;

	mutex_lock(&pmic_access_mutex);
	if (size_in_byte > PMIC_ACCESS_MAX_SIZE) {
		result = -1;
		pr_err("%s: transfer size not supported (%d > %d)\n", __func__,
				size_in_byte + 1, PMIC_ACCESS_MAX_SIZE);
		goto pmic_read_end;
	}
	pmic_access_shared_data = vmm_pmic_get_shared_data();
	pmic_access_shared_data->data[0] = (uint8_t)(reg_address & 0xFF);
	vmm_pmic_access_done = 0;
	if (mv_svc_pmic_reg_access(PMIC_REG_READ, reg_address, size_in_byte)) {
		result = -1;
		pr_err("%s: mv_svc_pmic_reg_access failed\n", __func__);
		goto pmic_read_end;
	}
	if (wait_event_interruptible(pmic_wait_queue, vmm_pmic_access_done)) {
		result = -ERESTARTSYS;
		pr_err("%s: wait_event interrupted by signal\n", __func__);
		goto pmic_read_end;
	}
	if (pmic_access_shared_data->status) {
		result = -1;
		pr_err("%s: pmic shared data status is wrong\n", __func__);
		goto pmic_read_end;
	}
	memcpy(data, pmic_access_shared_data->data, size_in_byte);

pmic_read_end:
	pr_debug("%s %s on CPU%d\n", __func__, result < 0 ? "FAILS" : "PASS",
			raw_smp_processor_id());
	mutex_unlock(&pmic_access_mutex);
	return result;
}
EXPORT_SYMBOL(vmm_pmic_reg_read_by_range);

int32_t vmm_pmic_reg_read(uint32_t reg_address, uint32_t *p_reg_val)
{
	return vmm_pmic_reg_read_by_range(reg_address, (uint8_t *)p_reg_val, 1);
}
EXPORT_SYMBOL(vmm_pmic_reg_read);

static irqreturn_t vmm_pmic_reg_access_irq_hdl(int irq, void *dev)
{
	if (vmm_pmic_access_done != 1) {
		vmm_pmic_access_done = 1;
		wake_up(&pmic_wait_queue);
	}
	return IRQ_HANDLED;
}
u8 regaddr;
u8 val;
u8 i2cdev;

static ssize_t vmm_pmic_val_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	if (kstrtou8_from_user(ubuf, count, 0, &val))
		return -EFAULT;

	vmm_pmic_reg_write((i2cdev << 24) | regaddr, val);

	return count;

}
static int vmm_pmic_val_show(struct seq_file *s, void *unused)
{
	int read_val = 0;
	vmm_pmic_reg_read((i2cdev << 24) | regaddr, &read_val);
	seq_printf(s, "0x%x\n", read_val);
	return 0;
}

static int vmm_pmic_val_open(struct inode *inode, struct file *file)
{
	return single_open(file, vmm_pmic_val_show, inode->i_private);
}

const struct file_operations vmm_pmic_val_ops = {
	.open = vmm_pmic_val_open,
	.read = seq_read,
	.write = vmm_pmic_val_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t vmm_pmic_regaddr_write(struct file *file,
		const char __user *ubuf, size_t count, loff_t *ppos)
{
	if (kstrtou8_from_user(ubuf, count, 0, &regaddr))
		return -EFAULT;
	return count;

}
static int vmm_pmic_regaddr_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "0x%x\n", regaddr);
	return 0;
}

static int vmm_pmic_regaddr_open(struct inode *inode, struct file *file)
{
	return single_open(file, vmm_pmic_regaddr_show, inode->i_private);
}

const struct file_operations vmm_pmic_regaddr_ops = {
	.open = vmm_pmic_regaddr_open,
	.read = seq_read,
	.write = vmm_pmic_regaddr_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t vmm_pmic_i2cdev_write(struct file *file, const char __user *ubuf,
				size_t count, loff_t *ppos)
{
	if (kstrtou8_from_user(ubuf, count, 0, &i2cdev))
		return -EFAULT;
	return count;

}
static int vmm_pmic_i2cdev_show(struct seq_file *s, void *unused)
{
	seq_printf(s, "0x%x\n", i2cdev);
	return 0;
}

static int vmm_pmic_i2cdev_open(struct inode *inode, struct file *file)
{
	return single_open(file, vmm_pmic_i2cdev_show, inode->i_private);
}

const struct file_operations vmm_pmic_i2cdev_ops = {
	.open = vmm_pmic_i2cdev_open,
	.read = seq_read,
	.write = vmm_pmic_i2cdev_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static int32_t vmm_pmic_probe(struct platform_device *pdev)
{
	uint32_t irq_number;
	struct dentry *vmm_pmic_dentry;
	struct dentry *vmm_pmic_dbg_root;
	int ret = 0;

	/* mutex/wait queue init */
	mutex_init(&pmic_access_mutex);

	/* irq init */
	irq_number = platform_get_irq_byname(pdev, "PMIC_ACCESS_HIRQ");
	if (!IS_ERR_VALUE(irq_number)) {
		ret = devm_request_irq(&pdev->dev, irq_number,
				vmm_pmic_reg_access_irq_hdl,
				IRQF_NO_SUSPEND, "pmic hirq", NULL);
		if (ret != 0) {
			dev_err(&pdev->dev, "setup irq %d failed: %d\n",
					irq_number, ret);
			return -EINVAL;
		}
	}
	vmm_pmic_dbg_root = debugfs_create_dir("vmm_pmic", NULL);

	if (!vmm_pmic_dbg_root || IS_ERR(vmm_pmic_dbg_root))
		return -ENODEV;

	vmm_pmic_dentry = debugfs_create_file("i2cdev", S_IRUGO | S_IWUSR,
		vmm_pmic_dbg_root, NULL,
		&vmm_pmic_i2cdev_ops);

	if (!vmm_pmic_dentry) {
		debugfs_remove(vmm_pmic_dbg_root);
		vmm_pmic_dbg_root = NULL;
		return -ENODEV;
	}

	vmm_pmic_dentry = debugfs_create_file("regaddr", S_IRUGO | S_IWUSR,
		vmm_pmic_dbg_root, NULL,
		&vmm_pmic_regaddr_ops);

	if (!vmm_pmic_dentry) {
		debugfs_remove(vmm_pmic_dbg_root);
		vmm_pmic_dbg_root = NULL;
		return -ENODEV;
	}

	vmm_pmic_dentry = debugfs_create_file("val", S_IRUGO | S_IWUSR,
		vmm_pmic_dbg_root, NULL,
		&vmm_pmic_val_ops);

	if (!vmm_pmic_dentry) {
		debugfs_remove(vmm_pmic_dbg_root);
		vmm_pmic_dbg_root = NULL;
		return -ENODEV;
	}

	/* init done */
	vmm_pmic_reg_access_initialised = 1;
	return 0;
}

static const struct of_device_id xgold_vmm_pmic_of_match[] = {
	{
		.compatible = "intel,xgold_vmm_pmic",
	},
	{},
};

MODULE_DEVICE_TABLE(of, xgold_vmm_pmic_of_match);

static struct platform_driver xgold_vmm_pmic_driver = {
	.probe = vmm_pmic_probe,
	.driver = {
		.name = "xgold_vmm_pmic",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(xgold_vmm_pmic_of_match),
	}
};

static int32_t __init vmm_pmic_reg_access_init(void)
{
	return platform_driver_register(&xgold_vmm_pmic_driver);
}

static void __exit vmm_pmic_reg_access_exit(void)
{
	platform_driver_unregister(&xgold_vmm_pmic_driver);
}

subsys_initcall(vmm_pmic_reg_access_init);
module_exit(vmm_pmic_reg_access_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("vmm pmic access driver");
