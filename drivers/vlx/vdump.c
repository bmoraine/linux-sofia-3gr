/*
* Copyright (C) 2013 Intel Mobile Communications GmbH
*
* This program is free software; you can redistribute it and/or modify it
* under the terms and conditions of the GNU General Public License,
* version 2, as published by the Free Software Foundation.
*
* This program is distributed in the hope it will be useful, but WITHOUT
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
* FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
* more details.
*
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
*
*/

#include <linux/device.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/kmsg_dump.h>
#include <asm/page.h>

#include <sofia/vmm_al.h>
#include <sofia/pal_shared_data.h>
#include <sofia/vmm_platform_service.h>
#include <sofia/pal_sys_exception.h>
#include <sofia/pal_coredump.h>

#define VDUMP_MODULE_NAME "vdump"

#if 1
#define VDUMP_DEBUG
#endif

#define VD_LOG(format, args...)	printk(VDUMP_MODULE_NAME": " format, ## args)

#ifdef VDUMP_DEBUG
#define VD_DEBUG(format, args...) printk(VDUMP_MODULE_NAME": " format, ## args)
#else
#define VD_DEBUG(format, args...)
#endif

#define VDUMP_BUF_LEN 8196
/*
 * This is a global struct for storing data use by vdump
 */
struct vdump_data {
	/* Used for registering chardevs */
	int major;
	struct class *vd_class;
	char buffer[VDUMP_BUF_LEN];
};

static struct vdump_data vdump_data;

static int vdump_control_open(struct inode *p_inode, struct file *p_file);
static int vdump_control_release(struct inode *p_inode, struct file *p_file);
static ssize_t vdump_control_write(struct file *p_file,
		const char __user *user_buffer, size_t count, loff_t *position);
static ssize_t vdump_control_read(struct file *file_ptr,
		char __user *user_buffer, size_t count, loff_t *position);

const struct file_operations vdump_control_fops = {
	.read = vdump_control_read,
	.write = vdump_control_write,
	.open = vdump_control_open,
	.release = vdump_control_release
};

static void vdump_panic(struct kmsg_dumper *dumper,
		enum kmsg_dump_reason reason);

static struct kmsg_dumper vdumper = {
	.dump = vdump_panic,
};

static int vdump_control_open(struct inode *p_inode, struct file *p_file)
{
	VD_LOG("vdump_control_open\n");
	return 0;
}

static int vdump_control_release(struct inode *p_inode, struct file *p_file)
{
	VD_LOG("vdump_control_release\n");
	return 0;
}

static ssize_t vdump_control_write(struct file *p_file,
		const char __user *user_buffer, size_t count, loff_t *position)
{
	VD_LOG("vdump_control_write\n");
	return 0;
}

static ssize_t vdump_control_read(struct file *file_ptr,
		char __user *user_buffer, size_t count, loff_t *position)
{
	VD_LOG("vdump_control_read\n");
	return 0;
}

static int vdump_init(void)
{
	struct device *vdump_dev;

	VD_LOG("init");

	vdump_data.major = register_chrdev(0, VDUMP_MODULE_NAME,
			&vdump_control_fops);
	if (vdump_data.major < 0) {
		VD_LOG("cannot register dev errorcode=%i", vdump_data.major);
		return -1;
	}

	vdump_data.vd_class = class_create(THIS_MODULE, VDUMP_MODULE_NAME);
	if (IS_ERR(vdump_data.vd_class)) {
		unregister_chrdev(vdump_data.major, VDUMP_MODULE_NAME);
		VD_LOG("error creating class");
		return -1;
	}

	vdump_dev = device_create(vdump_data.vd_class, NULL,
			MKDEV(vdump_data.major, 0), NULL, VDUMP_MODULE_NAME);

	if (IS_ERR(vdump_dev)) {
			class_destroy(vdump_data.vd_class);
			unregister_chrdev(vdump_data.major, VDUMP_MODULE_NAME);
			VD_LOG("error creating device");
			return -1;
	}

	/* register linux kernal space for dumping */
	kmsg_dump_register(&vdumper);

	return 0;
}

static void vdump_exit(void)
{
	VD_LOG("exit");
}

/*
 * callback from kmsg_dump. (s2,l2) has the most recently
 * written bytes, older bytes are in (s1,l1).
 */
static void vdump_panic(struct kmsg_dumper *dumper,
		enum kmsg_dump_reason reason)
{
	struct vmm_shared_data *vmm_shared_data;
	struct sys_trap *trap_data;
	struct cd_ram *cd_area;
	size_t len;

	if (reason != KMSG_DUMP_PANIC)
		return;

	/* Get share data */
	vmm_shared_data = get_vmm_shared_data();

	cd_area = (struct cd_ram *)vmm_shared_data->vm_log_str;
	cd_area->logical_start = (void *)__start_rodata;
	cd_area->physical_start = (void *)__pa(__start_rodata);
	cd_area->length = (unsigned int)_end - (unsigned int)__start_rodata;

	vmm_cd_service(CD_ADD_REGION, (void *)__pa(cd_area));

	/* Entire low memory range */
	cd_area->logical_start = (void *)_end;
	cd_area->physical_start = (void *)__pa(_end);
	cd_area->length = (unsigned int)high_memory - (unsigned int)_end;
	vmm_cd_service(CD_ADD_REGION, (void *)__pa(cd_area));

	/* Format trap data */
	trap_data = (struct sys_trap *)vmm_shared_data->vm_log_str;

	memset(trap_data, 0, sizeof(*trap_data));
	trap_data->exception_type = SYS_EXCEPTION_LINUX;

	kmsg_dump_get_buffer(dumper, true,
			vdump_data.buffer, VDUMP_BUF_LEN, &len);
	trap_data->os.linux_log.kmsg = (char *)__pa(vdump_data.buffer);
	trap_data->os.linux_log.kmsg_len = len;
	vmm_sys_exception(SYS_EXCEPTION_DUMP, (void *)__pa(trap_data));

	/* Program should never return back here */
	unreachable();
}

module_init(vdump_init);
module_exit(vdump_exit);

MODULE_DESCRIPTION("Virtual Dump");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL");
