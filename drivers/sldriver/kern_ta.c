/*
 * The file implements ioctl interface for vIDT driver.
 * Copyright (c) 2015, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#include "include/vmm_hsym_common.h"
#include "include/vmm_hsym.h"
#include "include/vidt_ioctl.h"
#include "include/vidt.h"
#include <linux/uaccess.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/printk.h>
#include <asm/siginfo.h>	/*siginfo */
#include <linux/rcupdate.h>	/*rcu_read_lock */
#include <linux/sched.h>	/*find_task_by_pid_type */
#include <linux/vmalloc.h>
#include <linux/seq_file.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/cpu.h>
#include <linux/types.h>

#ifdef __x86_64__
extern inline int cpuid_asm64(uint32_t leaf, uint32_t b_val, uint64_t c,
			      uint64_t d, uint64_t S, uint64_t D);
extern int request_sl_service(uint32_t leaf, uint64_t param_a,
			      uint64_t param_b, uint64_t S, uint64_t D);
#else
extern inline int info_cpuid(sl_info_t *info_ptr);
extern int request_sl_service(uint32_t leaf, uint32_t param_a,
			      uint32_t param_b, uint32_t S, uint32_t D);
#endif

/* we choose 44 as our signal number
 * (real-time signals are in the range of 33 to 64)*/
#define SIG_CLEAN 44
#define DEBUG_LEAF 42
#define EPTP_SWITCH_LEAF 0
#define D_VIEW           0
unsigned int g_view = 0;
#define ENABLE_EPT 1
#define DISABLE_EPT 0

#define VMM_PROT_ACTION 0
#define VMM_UNPROT_ACTION 1
static int hspid;

inline void vmfunc_emul(uint32_t a, uint32_t c, uint32_t d)
{
	asm volatile ("vmcall" : : "a" (a), "c"(c), "d"(d));
}

inline void vmfunc(uint32_t a, uint32_t c)
{

	asm volatile ("nop" : : "a" (a), "c"(c));
	asm volatile (".byte 0x0f");
	asm volatile (".byte 0x01");
	asm volatile (".byte 0xd4");
}

static int chk_hsim_version(void)
{
	sl_info_t *info = kmalloc(sizeof(sl_info_t), GFP_KERNEL);
	int ret;

	memset(info, 0, sizeof(sl_info_t));

#ifdef __x86_64__
	ret =
	    cpuid_asm64(SL_CMD_HSEC_GET_INFO, CMD_GET, (uint64_t) info,
			sizeof(sl_info_t), 0, 0);
#else
	ret = info_cpuid(info);
#endif
	if (ret) {
		kfree(info);
		return SL_EUNKNOWN;
	}

	if (info->major_version != MAJOR_VERSION
	    || info->minor_version != MINOR_VERSION
	    || info->vendor_id[0] != 0x534c
	    || info->vendor_id[1] != 0x01
	    || info->vendor_id[2] != 0x20) {
		pr_err("Error: Hsim Version mismatch\n");
		kfree(info);
		return SL_EVERSION_MISMATCH;
	}
	kfree(info);
	return SL_SUCCESS;
}

static long vidt_ioctl(struct file *file, unsigned int vidt_command,
		       unsigned long vidt_param)
{
	struct entry_pid_viewid param_entry;
	int err = -1;
	static uint32_t setup_vidt_done;
	switch (vidt_command) {
	case VIDT_GET_TA_TYPE:{
			struct vmm_view_type *c_param;
			uint32_t retVal = 0;
			uint32_t size = _IOC_SIZE(vidt_command);
			void __user *ubuf = (void __user *)vidt_param;
			uint64_t param_addr;

			if (size != sizeof(vmm_view_type_t)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(c_param, ubuf,
					   sizeof(vmm_view_type_t));
			if (retVal != 0) {
				pr_err
				    ("%s %d Failed to read param from ubuf:%p\n",
				     __func__, __LINE__, ubuf);
				kfree(c_param);
				return -EFAULT;
			}

			param_addr = virt_to_phys(&c_param->ta_type);

#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_GET_TA_TYPE,
					       c_param->view_id,
					       (unsigned long)param_addr, 0, 0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_GET_TA_TYPE,
					       (uint32_t) c_param->view_id,
					       (uint32_t) param_addr, 0, 0);
#endif

			if (retVal != 0) {
				pr_err("GET TA TYPE hypercall failed:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}

			retVal =
			    copy_to_user(ubuf, c_param,
					 sizeof(vmm_view_type_t));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}

			kfree(c_param);
			return 0;
		}

	case VIDT_PID_VIEW_MAP:
		if ((struct entry_pid_viewid __user *)vidt_param != NULL) {
			err = copy_from_user(&param_entry,
					     (struct entry_pid_viewid __user *)
					     vidt_param,
					     sizeof(struct entry_pid_viewid));
			if (err) {
				pr_err("copy_from_user err = %d\n", err);
				return -EFAULT;
			}
		}
		return map_pid_viewId(file->private_data, param_entry);

	case VIDT_PID_VIEW_UNMAP:
		if ((struct entry_pid_viewid __user *)vidt_param != NULL) {
			err = copy_from_user(&param_entry,
					     (struct entry_pid_viewid __user *)
					     vidt_param,
					     sizeof(struct entry_pid_viewid));
			if (err) {
				pr_err("copy_from_user err = %d\n", err);
				return -EFAULT;
			}
		}
		return unmap_pid_viewId(file->private_data, param_entry);

	case VIDT_REGISTER:
		if (!setup_vidt_done) {
			err = setup_vidt();
			if (err == 0)
				setup_vidt_done = 1;
		} else {
			return 0;
		}
		return err;

	case VIDT_REGISTER_HP_PID:
		if ((pid_t *) vidt_param != NULL) {
			err =
			    copy_from_user(&hspid, (pid_t *) vidt_param,
					   sizeof(pid_t));
			pr_err("copy_from_user hspid = %d\n", (int)hspid);
			if (err) {
				pr_err("copy_from_user err = %d\n", err);
				return -EFAULT;
			}
		}
		return 0;

	case VIDT_CREATE_VIEW:{
			struct enclave_create_param *c_param;
			uint32_t size = _IOC_SIZE(vidt_command);
			int retVal = 0;
			void __user *ubuf = (void __user *)vidt_param;
			uint64_t param_addr;

			if (size != sizeof(struct enclave_create_param)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(c_param, ubuf,
					   sizeof(struct enclave_create_param));
			if (retVal != 0) {
				pr_err("Failed to read param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return -EFAULT;
			}

			param_addr = virt_to_phys(c_param);
			/*Call the hypercall to create the view. */
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_CREATE_VIEW,
					       (uint64_t) param_addr,
					       sizeof(struct
						      enclave_create_param), 0,
					       0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_CREATE_VIEW,
					       (uint32_t) param_addr,
					       sizeof(struct
						      enclave_create_param), 0,
					       0);
#endif
			if (retVal != 0) {
				pr_err
				    ("Create view hypercall failed. Retval:%x\n",
				     retVal);
				kfree(c_param);
				return retVal;
			}
			if (c_param->view_handle == 0) {
				pr_err
				    ("Create view hypercall to create view\n");
				kfree(c_param);
				return retVal;
			}
			retVal =
			    copy_to_user(ubuf, c_param,
					 sizeof(struct enclave_create_param));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}
			kfree(c_param);
			return 0;
		}

	case VIDT_ADD_PAGE:{
			struct enclave_add_param *c_param;
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			void __user *ubuf = (void __user *)vidt_param;
			uint64_t param_addr;

			if (size != sizeof(struct enclave_add_param)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(c_param, ubuf,
					   sizeof(struct enclave_add_param));
			if (retVal != 0) {
				pr_err("Failed to read param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return -EFAULT;
			}
			param_addr = virt_to_phys(c_param);
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_ADD_PAGE,
					       (uint64_t) param_addr,
					       sizeof(struct enclave_add_param),
					       0, 0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_ADD_PAGE,
					       (uint32_t) param_addr,
					       sizeof(struct enclave_add_param),
					       0, 0);
#endif

			if (retVal != 0) {
				pr_err
				    ("%s %d Add page hypercall failed. Retval:%x \
					 view_handle is %x param_addr is %x\n",
				     __func__, __LINE__, retVal,
				     c_param->view_handle, (uint32_t)
				     vidt_param);
				kfree(c_param);
				return retVal;
			}
			retVal =
			    copy_to_user(ubuf, c_param,
					 sizeof(struct enclave_add_param));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}
			kfree(c_param);
			return 0;
		}

	case VIDT_INIT_VIEW:{
			struct enclave_init_param *c_param;
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			void __user *ubuf = (void __user *)vidt_param;
			uint64_t param_addr;

			if (size != sizeof(struct enclave_init_param)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(c_param, ubuf,
					   sizeof(struct enclave_init_param));
			if (retVal != 0) {
				pr_err("Failed to read param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return -EFAULT;
			}
			param_addr = virt_to_phys(c_param);
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_INIT_VIEW,
					       param_addr,
					       sizeof(struct
						      enclave_init_param), 0,
					       0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_INIT_VIEW,
					       (uint32_t) param_addr,
					       sizeof(struct
						      enclave_init_param), 0,
					       0);
#endif

			if (retVal != 0) {
				pr_err
				    ("Init view hypercall failed. Retval:%x\n",
				     retVal);
				kfree(c_param);
				return retVal;
			}
			retVal =
			    copy_to_user(ubuf, c_param,
					 sizeof(struct enclave_init_param));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}
			kfree(c_param);
			return 0;
		}

	case VIDT_GET_TA_PROPERTIES:{
			struct view_prop_t *c_param;
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			void __user *ubuf = (void __user *)vidt_param;
			uint64_t param_addr;

			if (size != sizeof(struct view_prop_t)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(c_param, ubuf,
					   sizeof(struct view_prop_t));
			if (retVal != 0) {
				pr_err("Failed to read param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return -EFAULT;
			}
			param_addr = virt_to_phys(c_param);
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_GET_TA_PROPERTIES,
					       param_addr,
					       sizeof(struct view_prop_t), 0,
					       0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_GET_TA_PROPERTIES,
					       (uint32_t) param_addr,
					       sizeof(struct view_prop_t), 0,
					       0);
#endif

			if (retVal != 0) {
				pr_err
				    ("%s %d SL_CMD_HSEC_GET_TA_PROPERTIES hypercall failed.\
					 Retval:%x \n", __func__, __LINE__,
				     retVal);
				kfree(c_param);
				return retVal;
			}
			retVal =
			    copy_to_user(ubuf, c_param,
					 sizeof(struct view_prop_t));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}
			kfree(c_param);
			return 0;
		}

	case VIDT_VERIFY_STATUS:{
			uint32_t retVal = 0;
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_VIDT_VERIFY_STATUS,
					       0, 0, 0, 0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_VIDT_VERIFY_STATUS,
					       0, 0, 0, 0);
#endif

			if (retVal != 0) {
				pr_err
				    ("%s %d  SL_CMD_HSEC_VIDT_VERIFY_STATUS hypercall failed.\
					 Retval:%x\n", __func__, __LINE__,
				     retVal);
				return retVal;
			}
			return retVal;
		}

	case VIDT_VERIFY:{
			struct vIDT_sign *c_param;
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			void __user *ubuf = (void __user *)vidt_param;
			uint64_t param_addr;

			if (size != sizeof(struct vIDT_sign)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(c_param, ubuf,
					   sizeof(struct vIDT_sign));
			if (retVal != 0) {
				pr_err("Failed to read param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return -EFAULT;
			}
			param_addr = virt_to_phys(c_param);
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_VERIFY_VIDT,
					       param_addr,
					       sizeof(struct vIDT_sign), 0, 0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_VERIFY_VIDT,
					       (uint32_t) param_addr,
					       sizeof(struct vIDT_sign), 0, 0);
#endif

			if (retVal != 0) {
				pr_err
				    ("%s %d  SL_CMD_HSEC_VERIFY_VIDT hypercall failed.\
					 Retval:%x\n", __func__, __LINE__,
				     retVal);
				kfree(c_param);
				return retVal;
			}
			retVal =
			    copy_to_user(ubuf, c_param,
					 sizeof(struct vIDT_sign));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}
			kfree(c_param);
			return retVal;
		}

	case VIDT_MAP_SHM:{
			struct shm_map_info *c_param;
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			void __user *ubuf = (void __user *)vidt_param;
			const void __user *view_list_addr;
			uint64_t param_addr;
			uint32_t num_entries;
			uint32_t view_list_size;
			v_node *view_list;

			if (size != sizeof(struct shm_map_info)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(c_param, ubuf,
					   sizeof(struct shm_map_info));
			if (retVal != 0) {
				pr_err
				    ("%s %d Failed to read param from ubuf:%p\n",
				     __func__, __LINE__, ubuf);
				kfree(c_param);
				return -EFAULT;
			}
			view_list_addr =
			    (const void __user *)(uintptr_t) c_param->view_list;

			param_addr = virt_to_phys(c_param);
			num_entries = c_param->num_entries;
			view_list_size = num_entries * sizeof(v_node);
			view_list = kmalloc(view_list_size, GFP_KERNEL);
			if (view_list == NULL) {
				pr_err("[%s:%d] Failed to allocate v_node\n",
				       __func__, __LINE__);
				kfree(c_param);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(view_list,
					   view_list_addr, view_list_size);
			if (retVal != 0) {
				pr_err
				    ("%s %d Failed to read param from ubuf:%p\n",
				     __func__, __LINE__, view_list_addr);
				kfree(c_param);
				kfree(view_list);
				return -EFAULT;
			}
			c_param->view_list = virt_to_phys(view_list);
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_MAP_SHM, param_addr,
					       sizeof(struct shm_map_info), 0,
					       0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_MAP_SHM,
					       (uint32_t) param_addr,
					       sizeof(struct shm_map_info), 0,
					       0);
#endif

			if (retVal != 0) {
				pr_err
				    ("%s %d SL_CMD_HSEC_MAP_SHM hypercall failed.\
					 Retval:%x \n", __func__, __LINE__,
				     retVal);
				kfree(view_list);
				kfree(c_param);
				return retVal;
			}
			retVal =
			    copy_to_user(ubuf, c_param,
					 sizeof(struct shm_map_info));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				kfree(view_list);
				kfree(c_param);
				return retVal;
			}
			kfree(view_list);
			kfree(c_param);
			return 0;
		}

	case VIDT_CHANGE_MEM_PERM:{
			struct mem_info *c_param;
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			void __user *ubuf = (void __user *)vidt_param;
			const void __user *view_list_addr;
			uint64_t param_addr;
			uint32_t num_entries;
			uint32_t view_list_size;
			v_node *view_list;

			if (size != sizeof(struct mem_info)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(c_param, ubuf,
					   sizeof(struct mem_info));
			if (retVal != 0) {
				pr_err("Failed to read param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return -EFAULT;
			}

			view_list_addr =
			    (const void __user *)(uintptr_t) c_param->view_list;
			param_addr = virt_to_phys(c_param);
			num_entries = c_param->num_entries;
			view_list_size = num_entries * sizeof(v_node);
			view_list = kmalloc(view_list_size, GFP_KERNEL);
			if (view_list == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				kfree(c_param);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(view_list,
					   view_list_addr, view_list_size);
			if (retVal != 0) {
				pr_err("Failed to read param from ubuf:%p\n",
				       view_list_addr);
				kfree(view_list);
				kfree(c_param);
				return -EFAULT;
			}
			c_param->view_list = virt_to_phys(view_list);

#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_CHANGE_MEM_PERM,
					       param_addr,
					       sizeof(struct mem_info), 0, 0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_CHANGE_MEM_PERM,
					       (uint32_t) param_addr,
					       sizeof(struct mem_info), 0, 0);
#endif

			if (retVal != 0) {
				pr_err
				    ("%s %d SL_CMD_HSEC_CHANGE_MEM_PERM hypercall failed.\
					 Retval:%x \n", __func__, __LINE__,
				     retVal);
				kfree(view_list);
				kfree(c_param);
				return retVal;
			}
			retVal =
			    copy_to_user(ubuf, c_param,
					 sizeof(struct mem_info));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				kfree(view_list);
				kfree(c_param);
				return retVal;
			}
			kfree(view_list);
			kfree(c_param);
			return 0;
		}

	case VIDT_UPDATE_PERM:{
			struct enclave_code_page_param *c_param;
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			void __user *ubuf = (void __user *)vidt_param;
			uint64_t param_addr;

			if (size != sizeof(struct enclave_code_page_param)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(c_param, ubuf,
					   sizeof(struct
						  enclave_code_page_param));
			if (retVal != 0) {
				pr_err("Failed to read param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return -EFAULT;
			}
			param_addr = virt_to_phys(c_param);
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_UPDATE_PERM,
					       param_addr,
					       sizeof(struct
						      enclave_code_page_param),
					       0, 0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_UPDATE_PERM,
					       (uint32_t) param_addr,
					       sizeof(struct
						      enclave_code_page_param),
					       0, 0);
#endif

			if (retVal != 0) {
				pr_err
				    ("%s %d SL_CMD_HSEC_CHANGE_MEM_PERM hypercall failed.\
					 Retval:%x \n", __func__, __LINE__,
				     retVal);
				kfree(c_param);
				return retVal;
			}
			retVal =
			    copy_to_user(ubuf, c_param,
					 sizeof(struct
						enclave_code_page_param));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}
			kfree(c_param);
			return 0;
		}

	case VIDT_GET_VMM_VIEWID:{
			vmm_view_info_t *c_param;
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			uint64_t vmm_view;
			void __user *ubuf = (void __user *)vidt_param;
			uint64_t param_addr;

			if (size != sizeof(vmm_view_info_t)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(c_param, ubuf,
					   sizeof(vmm_view_info_t));
			if (retVal != 0) {
				pr_err
				    ("%s %d Failed to read param from ubuf:%p\n",
				     __func__, __LINE__, ubuf);
				kfree(c_param);
				return -EFAULT;
			}

			param_addr = virt_to_phys(&vmm_view);
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_GET_VMM_VIEWID,
					       c_param->enclave_id,
					       (unsigned long)param_addr);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_GET_VMM_VIEWID,
					       c_param->enclave_id,
					       (uint32_t) param_addr, 0, 0);
#endif
			if (retVal != 0) {
				pr_err("VMM VIEWID hypercall failed:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}

			c_param->vmm_view =
			    *(uint64_t *) phys_to_virt(param_addr);
			retVal =
			    copy_to_user(ubuf, c_param,
					 sizeof(vmm_view_info_t));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}
			kfree(c_param);
			return 0;
		}

	case VIDT_GET_INFO:{
			sl_info_t *c_param;
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			void __user *ubuf = (void __user *)vidt_param;

			if (size != sizeof(sl_info_t)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param:%u\n",
				       __func__, __LINE__, vidt_command);
				return -ENOMEM;
			}
			retVal =
			    copy_from_user(c_param, ubuf, sizeof(sl_info_t));
			if (retVal != 0) {
				pr_err
				    ("%s %d Failed to read param from ubuf:%p\n",
				     __func__, __LINE__, ubuf);
				kfree(c_param);
				return -EFAULT;
			}
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_GET_INFO,
					       (unsigned long)param_addr,
					       sizeof(sl_info_t), 0, 0);
#else
			retVal = info_cpuid(c_param);
#endif
			if (retVal != 0) {
				pr_err("GET INFO hypercall is failed:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}

			retVal = copy_to_user(ubuf, c_param, sizeof(sl_info_t));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				kfree(c_param);
				return retVal;
			}
			kfree(c_param);
			return 0;
		}

	case VIDT_REMOVE_VIEW:{
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			uint64_t enclave_id;
			void __user *ubuf = (void __user *)vidt_param;
			if (size != sizeof(uint64_t)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(&enclave_id, ubuf,
					   sizeof(enclave_id));
			if (retVal != 0) {
				pr_err
				    ("%s %d Failed to read param from ubuf:%p\n",
				     __func__, __LINE__, ubuf);
				return -EFAULT;
			}
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_REMOVE_VIEW, 0,
					       enclave_id, 0, 0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_REMOVE_VIEW, 0,
					       enclave_id, 0, 0);
#endif
			if (retVal != 0) {
				pr_err("REMOVE VIEW hypercall failed:%p\n",
				       ubuf);
				return retVal;
			}
			return 0;
		}
	case VIDT_GET_SL_AFFINITY:{
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			uint32_t affinity;
			void __user *ubuf = (void __user *)vidt_param;
			if (size != sizeof(uint32_t)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_GET_AFFINITY_MASK,
					       virt_to_phys(&affinity),
					       sizeof(affinity), 0, 0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_GET_AFFINITY_MASK,
					       (uint32_t)
					       virt_to_phys(&affinity),
					       sizeof(affinity), 0, 0);
#endif
			if (retVal != 0) {
				pr_err("REMOVE VIEW hypercall failed:%p\n",
				       ubuf);
				return retVal;
			}
			retVal =
			    copy_to_user(ubuf, &affinity, sizeof(affinity));
			if (retVal != 0) {
				pr_err
				    ("%s %d Failed to read param from ubuf:%p\n",
				     __func__, __LINE__, ubuf);
				return -EFAULT;
			}
			return 0;
		}

	case VIDT_GET_AVAIL_HEAP:{
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			uint64_t vmm_heap_size;
			void __user *ubuf = (void __user *)vidt_param;
			uint64_t param_addr;

			if (size != sizeof(uint64_t)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}

			param_addr = virt_to_phys(&vmm_heap_size);
#ifdef __x86_64__
			retVal =
			    request_sl_service(SL_CMD_HSEC_GET_AVAIL_HEAP,
					       param_addr,
					       sizeof(vmm_heap_size), 0, 0);
#else
			retVal =
			    request_sl_service(SL_CMD_HSEC_GET_AVAIL_HEAP,
					       (uint32_t) param_addr,
					       sizeof(vmm_heap_size), 0, 0);
#endif
			if (retVal != 0) {
				pr_err("AVAIL HEAP hypercall failed:%p\n",
				       ubuf);
				return retVal;
			}
			retVal =
			    copy_to_user(ubuf, &vmm_heap_size,
					 sizeof(vmm_heap_size));
			if (retVal != 0) {
				pr_err("Failed to write param from ubuf:%p\n",
				       ubuf);
				return retVal;
			}
			return 0;
		}

	case VIDT_ACTIVATE_KEEPALIVE_VIEW:{
			struct view_prop_t *c_param;
			uint32_t size = _IOC_SIZE(vidt_command);
			uint32_t retVal = 0;
			uint64_t param_addr = 0;
			void __user *ubuf = (void __user *)vidt_param;
			if (size != sizeof(struct vmm_keepalive_info)) {
				pr_err("[%s:%d] Invalid command passed:%u\n",
				       __func__, __LINE__, vidt_command);
				return -EFAULT;
			}
			c_param = kmalloc(size, GFP_KERNEL);
			if (c_param == NULL) {
				pr_err("[%s:%d] Failed to allocate c_param\n",
				       __func__, __LINE__);
				return -EFAULT;
			}
			retVal =
			    copy_from_user(c_param, ubuf,
					   sizeof(struct vmm_keepalive_info));
			if (retVal != 0) {
				pr_err("Failed to read param from ubuf\n");
				kfree(c_param);
				return -EFAULT;
			}
			param_addr = virt_to_phys(c_param);
#ifdef __x86_64__
			retVal =
			    request_sl_service
			    (SL_CMD_HSEC_ACTIVATE_KEEPALIVE_VIEW, param_addr,
			     sizeof(struct vmm_keepalive_info), 0, 0);
#else
			retVal =
			    request_sl_service
			    (SL_CMD_HSEC_ACTIVATE_KEEPALIVE_VIEW,
			     (uint32_t) param_addr,
			     sizeof(struct vmm_keepalive_info), 0, 0);
#endif

			if (retVal != 0) {
				pr_err
				    ("%s %d ACTIVATE_KEEPALIVE failed %x",
				     __func__, __LINE__, retVal);
				kfree(c_param);
				return retVal;
			}
			kfree(c_param);
			return 0;
		}

	default:
		pr_err("%s %d INVALID %x\n", __func__, __LINE__, vidt_command);
		break;
	}

	return 0;
}

static int vidt_open(struct inode *inode, struct file *file)
{
	int status = 0;
	file->private_data = NULL;
	status = init_view_map_list(file);
	if (status) {
		pr_err("view map list initialization failed\n");
		return -1;
	}
	return 0;
}

static int vidt_close(struct inode *inode, struct file *file)
{
	struct siginfo info;
	struct task_struct *t;
	int ret;

	clean_ta_view(file->private_data);
	clean_view_map_list(file);

	/* send the signal */
	memset(&info, 0, sizeof(struct siginfo));
	info.si_signo = SIG_CLEAN;
	info.si_code = SI_QUEUE;
	/* this is bit of a trickery: SI_QUEUE is normally used by sigqueue from
	 * user space,and kernel space should use SI_KERNEL. But if SI_KERNEL is
	 * used the real_time data is not delivered to the user space signal handler
	 * function. */
	/*real time signals may have 32 bits of data, so send pid */
	info.si_pid = current->pid;
	info.si_int = 0xDEAD;

	rcu_read_lock();
	/*find the task_struct associated with this pid */
	t = pid_task(find_pid_ns(hspid, &init_pid_ns), PIDTYPE_PID);
	if (t == NULL) {
		pr_err("no such pid\n");
		rcu_read_unlock();
		return -ENODEV;
	}
	rcu_read_unlock();
	ret = send_sig_info(SIG_CLEAN, &info, t);	/*send the signal */
	if (ret < 0) {
		pr_err("error sending signal\n");
		return ret;
	}
	return 0;
}

static const struct file_operations vidt_fops = {
	.owner = THIS_MODULE,
	.open = vidt_open,
	.release = vidt_close,
	.unlocked_ioctl = vidt_ioctl,
};

static struct miscdevice vidt_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vidt",
	.fops = &vidt_fops,

};

int __init vmm_mod_init(void)
{
	int ret = 0;

	ret = chk_hsim_version();
	if (ret != 0)
		return SL_EVERSION_MISMATCH;

	ret = misc_register(&vidt_device);
	if (ret)
		pr_err("errot ret = %d\n ", ret);

	return ret;
}

void __exit vmm_mod_exit(void)
{
	misc_deregister(&vidt_device);
	restore_os_idt();
}

module_init(vmm_mod_init);
module_exit(vmm_mod_exit);

MODULE_LICENSE("GPL v2");
