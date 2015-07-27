/*
* Copyright (C) 2015 Intel Mobile Communications GmbH
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
#include <linux/kthread.h>
#include <linux/delay.h>  /* for msleep*/
#include <asm/page.h>

#include <sofia/mv_gal.h>
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>
#include "sofia/mv_hypercalls.h"
#include <sofia/pal_sys_exception_types.h>
#include <sofia/pal_coredump_types.h>

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

#define VDUMP_CONFIG_FILE "/system/etc/vdump.conf"
#define VDUMP_COREDUMP_FILE "/storage/sdcard0/coredump.istp"
#define VDUMP_CONFIG_FILE_SIZE 500
#define VDUMP_CONFIG_MAX_VALUE_LENGTH 50
#define VDUMP_CONFIG_PATH_LEN 255

#define DEFAULT_COREDUMP_PATH "/storage/sdcard0/"
#define DEFAULT_COREDUMP_NAME "coredump"

#define VDUMP_CONFIG_TAG_PATH "save_path"
#define VDUMP_CONFIG_TAG_ENABLE "enable"
#define VDUMP_CONFIG_TAG_SHMEM "shmem"
#define VDUMP_CONFIG_TAG_DEVICE "device"
#define VDUMP_CONFIG_TAG_DEVICE_BAUD "device_baud"
#define VDUMP_CONFIG_TAG_DEBUG_DEVICE "debug_device"
#define VDUMP_CONFIG_TAG_DEBUG_DEVICE_BAUD "debug_device_baud"
#define VDUMP_CONFIG_TAG_USB_CONFIG "usb_config"

static bool vdump_get_config(char *cfg_path);
static void vdump_send_config(void);
static void vdump_detect(char *gadget_name);
static bool get_value_by_tag(char *config, char *tag, char *value, int size);
static int str2int(char *str);

struct vdump_vmm_settings {
	int vdump_enable;
	int shmem_enable;
	struct cd_config cd_config_linux;
};
static struct vdump_vmm_settings vdump_vmm_setting;
static char coredump_path[VDUMP_CONFIG_PATH_LEN+1] = {0};
static struct file *coredump_file_ptr;
static loff_t coredump_file_pos;

void vdump_setting_init(void)
{
	memset(&vdump_vmm_setting, 0, sizeof(vdump_vmm_setting));
}

int vdump_get_shmem_config(void)
{
	return vdump_vmm_setting.shmem_enable;
}

void vdump_set_linux_config(void)
{
	int i = 20;
	vdump_detect(VDUMP_CONFIG_FILE);
	while (i--) {
		if (true == vdump_get_config(VDUMP_CONFIG_FILE)) {
			vdump_send_config();
			break;
		}
		msleep(2000);
	}
	if (i == 0)
		VD_LOG("vdump config not exist\n");
}

void vdump_open_coredump(void)
{
	int i;
	char filename[VDUMP_CONFIG_PATH_LEN+1];
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	/* scan for new coredump file to be saved */
	for (i = 0; i <= 99; i++) {
		snprintf(filename,
			VDUMP_CONFIG_PATH_LEN,
			"%s_%02d.istp",
			coredump_path,
			i);
		coredump_file_ptr = filp_open(filename,
			O_RDWR | O_CREAT | O_EXCL, 0);
		if (IS_ERR_OR_NULL(coredump_file_ptr)) {
			if (i == 99)
				VD_LOG("ERROR: Cannot save coredump file:%s.\n",
					filename);
		} else {
			coredump_file_pos = 0;
			VD_LOG("File opened:%s\n", filename);
			break;
		}
	}
	set_fs(old_fs);
}

void vdump_close_coredump(void)
{
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (!IS_ERR_OR_NULL(coredump_file_ptr)) {
		filp_close(coredump_file_ptr, NULL);
		coredump_file_ptr = NULL;
		coredump_file_pos = 0;
		VD_LOG("File closed.\n");
	}
	set_fs(old_fs);
}

void vdump_save_coredump(void *ptr, int num_bytes, int end)
{
	int written_bytes = 0;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (!IS_ERR_OR_NULL(coredump_file_ptr)) {
		/* try a first time write */
		written_bytes = coredump_file_ptr->f_op->write(
			coredump_file_ptr, (char *)ptr,
			num_bytes, &coredump_file_pos);
		if (written_bytes <= 0) {
			VD_LOG(
				"Error: file write Error with valid fp=%p, %d\n",
				coredump_file_ptr, written_bytes);
		}
	}

	set_fs(old_fs);
}

static bool vdump_get_config(char *cfg_path)
{
	char cfg_context[VDUMP_CONFIG_FILE_SIZE+1] = {0};
	char value_enable[VDUMP_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	char value_shmem[VDUMP_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	char value_device[VDUMP_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	char value_device_baud[VDUMP_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	char value_debug_device[VDUMP_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	char value_debug_device_baud[VDUMP_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	char value_usb_config[VDUMP_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	char value_path[VDUMP_CONFIG_MAX_VALUE_LENGTH+1] = {0};
	struct file *fp_cfg;
	loff_t pos_cfg = 0;
	/* loff_t pos_dir = 0; */
	mm_segment_t old_fs;
	int file_len = 0;
	bool valid = false;

	/* VD_LOG("vdump_get_config\n"); */

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp_cfg = filp_open(cfg_path, O_RDONLY, 0);
	/* VD_LOG("fp_cfg:%p\n", (void *)fp_cfg); */

	if (!IS_ERR_OR_NULL(fp_cfg)) {
		file_len = fp_cfg->f_op->llseek(fp_cfg, 0, SEEK_END);
		if (file_len > 0)
			fp_cfg->f_op->read(fp_cfg, cfg_context,
					file_len, &pos_cfg);
		if (strlen(cfg_context) != 0) {
			get_value_by_tag(cfg_context,
				VDUMP_CONFIG_TAG_PATH,
				value_path,
				VDUMP_CONFIG_MAX_VALUE_LENGTH);
			get_value_by_tag(cfg_context,
				VDUMP_CONFIG_TAG_ENABLE,
				value_enable,
				VDUMP_CONFIG_MAX_VALUE_LENGTH);
			get_value_by_tag(cfg_context,
				VDUMP_CONFIG_TAG_SHMEM,
				value_shmem,
				VDUMP_CONFIG_MAX_VALUE_LENGTH);
			get_value_by_tag(cfg_context,
				VDUMP_CONFIG_TAG_DEVICE,
				value_device,
				VDUMP_CONFIG_MAX_VALUE_LENGTH);
			get_value_by_tag(cfg_context,
				VDUMP_CONFIG_TAG_DEVICE_BAUD,
				value_device_baud,
				VDUMP_CONFIG_MAX_VALUE_LENGTH);
			get_value_by_tag(cfg_context,
				VDUMP_CONFIG_TAG_DEBUG_DEVICE,
				value_debug_device,
				VDUMP_CONFIG_MAX_VALUE_LENGTH);
			get_value_by_tag(cfg_context,
				VDUMP_CONFIG_TAG_DEBUG_DEVICE_BAUD,
				value_debug_device_baud,
				VDUMP_CONFIG_MAX_VALUE_LENGTH);
			get_value_by_tag(cfg_context,
				VDUMP_CONFIG_TAG_USB_CONFIG,
				value_usb_config,
				VDUMP_CONFIG_MAX_VALUE_LENGTH);
		}
		filp_close(fp_cfg, NULL);
		fp_cfg = NULL;
		valid = true;
	} else {
		/* VD_LOG("vdump config not exist\n"); */
		valid = false;
	}
	set_fs(old_fs);

	if (true == valid) {
		/* config path */
		memset(coredump_path, 0, VDUMP_CONFIG_PATH_LEN + 1);
		if (value_path == NULL || strlen(value_path) == 0)
			snprintf(coredump_path, VDUMP_CONFIG_PATH_LEN, "%s%s",
				DEFAULT_COREDUMP_PATH, DEFAULT_COREDUMP_NAME);
		else
			snprintf(coredump_path, VDUMP_CONFIG_PATH_LEN, "%s%s",
				value_path, DEFAULT_COREDUMP_NAME);
		/* config enable */
		vdump_vmm_setting.vdump_enable =
			(value_enable == NULL
				|| strlen(value_enable) == 0) ?
			0 : str2int(value_enable);
		vdump_vmm_setting.shmem_enable =
			(value_shmem == NULL
				|| strlen(value_shmem) == 0) ?
			0 : str2int(value_shmem);
		vdump_vmm_setting.cd_config_linux.device =
			(value_device == NULL
				|| strlen(value_device) == 0) ?
			0 : str2int(value_device);
		vdump_vmm_setting.cd_config_linux.device_baud =
			(value_device_baud == NULL
				|| strlen(value_device_baud) == 0) ?
			0 : str2int(value_device_baud);
		vdump_vmm_setting.cd_config_linux.debug_device =
			(value_debug_device == NULL
				|| strlen(value_debug_device) == 0) ?
			0 : str2int(value_debug_device);
		vdump_vmm_setting.cd_config_linux.debug_device_baud =
			(value_debug_device_baud == NULL
				|| strlen(value_debug_device_baud) == 0) ?
			0 : str2int(value_debug_device_baud);
		vdump_vmm_setting.cd_config_linux.usb_config =
			(value_usb_config == NULL
				|| strlen(value_usb_config) == 0) ?
			0 : str2int(value_usb_config);
	}

	return valid;
}

static bool get_value_by_tag(char *config, char *tag, char *value, int size)
{
	int i = 0;
	char *tv = strstr(config, tag);
	if (value == NULL || size == 0)
		return false;
	if (tv != NULL && strlen(tv) != 0) {
		bool b_equal_symbol = false;
		while (*(++tv) != '\n' && *tv != '\0' && i < size) {
			if (!((*tv >= 'a' && *tv <= 'z') ||
				(*tv >= 'A' && *tv <= 'Z') ||
				(*tv >= '0' && *tv <= '9') ||
				*tv == '_' || *tv == '.' ||
				*tv == '=' || *tv == '/' ||
				*tv == '\\'))
				continue;

		if (b_equal_symbol)
			value[i++] = *tv;
		if (*tv == '=')
			b_equal_symbol = true;
		}
		value[i++] = '\0';
	}
	return true;
}

static int str2int(char *str)
{
	int value = 0;
	char *s = str;
	if (NULL == str || strlen(str) == 0)
		return 0;
	do {
		if (*s > '9' || *s < '0')
			continue;
		value *= 10;
		value += *s - '0';
	} while (*(++s) != 0);
	return value;
}

static void vdump_send_config(void)
{
	/* struct vmm_shared_data *shared_data; */
	struct vmm_shared_data *vmm_shared_data;
	struct cd_config *cd_config_linux;

	/* Get shared struct */
	/* shared_data = mv_vcpu_get_data(); */
	vmm_shared_data = mv_gal_get_shared_data();
	cd_config_linux = (struct cd_config *)vmm_shared_data->vm_log_str;

	cd_config_linux->device =
		vdump_vmm_setting.cd_config_linux.device;
	cd_config_linux->device_baud =
		vdump_vmm_setting.cd_config_linux.device_baud;
	cd_config_linux->debug_device =
		vdump_vmm_setting.cd_config_linux.debug_device;
	cd_config_linux->debug_device_baud =
		vdump_vmm_setting.cd_config_linux.debug_device_baud;
	cd_config_linux->usb_config =
		vdump_vmm_setting.cd_config_linux.usb_config;

	VD_LOG("save_path:%s\n", coredump_path);
	VD_LOG("cd_config_linux->device=%d\n",
		cd_config_linux->device);
	VD_LOG("cd_config_linux->device_baud=%d\n",
		cd_config_linux->device_baud);
	VD_LOG("cd_config_linux->debug_device=%d\n",
		cd_config_linux->debug_device);
	VD_LOG("cd_config_linux->debug_device_baud=%d\n",
		cd_config_linux->debug_device_baud);
	VD_LOG("cd_config_linux->usb_config=%d\n",
		cd_config_linux->usb_config);

	/* Inform VMM of coredump settings */
	mv_svc_cd_service(CD_SET_CONFIG, (void *)__pa(cd_config_linux));
	/* Disable coredump configuration to
	prevent modem from overwriting the settings */
	mv_svc_cd_service(CD_ALLOW_CONFIG, (void *)0);
}

static void vdump_detect(char *gadget_name)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	int retry = 3;
	VD_LOG("vdump_detect[%s]\n", gadget_name);

	while (!kthread_should_stop() && retry >= 0) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fp = filp_open(gadget_name, O_RDONLY, 0);
		if (NULL == fp) {
			VD_LOG("NULL Gadget File Handle");
			msleep(2000);
			continue;
		} else if (IS_ERR(fp)) {
			/* VD_LOG("Gadget fp=%x open failed!\n",
			(unsigned int)fp); */
			retry--;
			msleep(2000);
			continue;
		} else {
			VD_LOG("Gadget %s open succeeded. fp=%x\n",
					gadget_name, (unsigned int)fp);
			filp_close(fp, NULL);
			break;
		}
		set_fs(old_fs);
		set_current_state(TASK_INTERRUPTIBLE);
	}
	return;
}
