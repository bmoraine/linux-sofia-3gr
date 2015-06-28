/*
 ****************************************************************
 *
 *  Component: Virtual OEMCrypto frontend driver
 *
 *  Copyright (C) 2011 - 2014 Intel Mobile Communications GmbH
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
 *  Contributor(s):
 *
 ****************************************************************
 */

#include <linux/module.h>
#include <linux/stddef.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/proc_fs.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/sched.h>
#include <linux/wait.h>

#include <sofia/mv_gal.h>
#include <sofia/mv_ipc.h>

#include "voemcrypto_common.h"

#if 0
#define VOEMCRYPTO_DEBUG
#endif

/*----- Tracing -----*/
#ifdef VOEMCRYPTO_DEBUG
#define DTRACE(format, args...)	\
	pr_debug("%s: " format, __func__, ##args)
#else
#define DTRACE(x...)
#endif

#define TRACE(x...)	pr_debug("VOEMCRYPTO-FE: " x)
#define WTRACE(x...)	pr_warn("VOEMCRYPTO-FE: " x)
#define ETRACE(x...)	pr_err("VOEMCRYPTO-FE: " x)


/**** Data/structure definitions *****/
enum voemcrypto_devstate_t {
	VOEMCRYPTO_DEVICE_IDLE,
	VOEMCRYPTO_DEVICE_BUSY,
	VOEMCRYPTO_DEVICE_WAITING_FOR_READ,
};

enum voec_ctl_status {
	VOEC_CTL_STATUS_CONNECT = 0,
	VOEC_CTL_STATUS_DISCONNECT
};

enum voec_ctl_event {
	VOEC_CTL_EVENT_CALL = 0,
	VOEC_CTL_EVENT_RESERVED
};

struct voemcrypto_t {
	/* Internal */
	uint32_t token;
	enum voec_ctl_status status;
	uint32_t share_data_size;
	unsigned char *share_data;
	char *cmdline;
	uint32_t  respond;
	wait_queue_head_t client_wait;
	wait_queue_head_t open_wait;
	struct mutex call_mutex;
};

/**** Data declarations *****/
static struct voemcrypto_t *voemcrypto_data = (struct voemcrypto_t *) NULL;
static enum voemcrypto_devstate_t device_state = VOEMCRYPTO_DEVICE_IDLE;

/* structs for data channel */
static struct voemcrypto_t voemcrypto_data_in;
static struct voemcrypto_t voemcrypto_data_out;


#define SIZE_OF_KCTL 16
#define SIZE_OF_IV 16
#define SIZE_OF_MACKEY 64


/****** Basic VRPC call function *******/
/* Need to add support for multiple arguments */
/* Calls need to be atomic to prevent corrupting the shared mem */
uint32_t
voemcrypto_call_ext(struct voemcrypto_t *p_voemcrypto, const uint32_t size0)
{
	uint32_t ret = size0;
	/* check for invalid size */
	if (size0 > p_voemcrypto->share_data_size)
		return 0;

	/* Acquire mutex */
	if (mutex_lock_interruptible(&p_voemcrypto->call_mutex))
		return 0;

	/* wait for open */
	if (wait_event_interruptible(p_voemcrypto->open_wait,
			p_voemcrypto->status == VOEC_CTL_STATUS_CONNECT)) {
		ret = 0;
		goto exit;
	}

	/* cmd+data is formatted beforehand to shared mem. */
	/* Up to 11 arguments in OEMCrypto interfaces. */

	p_voemcrypto->respond = 0;

	/* inform server */
	mv_ipc_mbox_post(p_voemcrypto->token, VOEC_CTL_EVENT_CALL);

	/* wait for server respond */
	if (wait_event_interruptible(p_voemcrypto->client_wait,
			p_voemcrypto->respond == 1)) {
		ret = 0;
		goto exit;
	}

	/* at this stage, vrpc_call has returned with data in pmem */
exit:
	mutex_unlock(&p_voemcrypto->call_mutex);
	return ret;
}


/*******************************************************************************
* Function:... voemc_get_pa
*******************************************************************************/
static inline phys_addr_t voemc_get_pa(uint8_t *v_addr)
{
	return page_to_phys(virt_to_page(v_addr)) + offset_in_page(v_addr);
}

static int get_phys_addr_from_user(struct voem_input_buffer *ib)
{
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;
	unsigned long prev_pfn, this_pfn;
	unsigned long pages_done, user_address;
	unsigned int offset;
	int err;

	offset = ib->baddr & ~PAGE_MASK;
	ib->phys_size = PAGE_ALIGN(ib->size + offset);
	err = -EINVAL;

	down_read(&mm->mmap_sem);

	vma = find_vma(mm, ib->baddr);
	if (!vma)
		goto out_up;

	if ((ib->baddr + ib->phys_size) > vma->vm_end)
		goto out_up;

	pages_done = 0;
	prev_pfn = 0; /* kill warning */
	user_address = ib->baddr;

	while (pages_done < (ib->phys_size >> PAGE_SHIFT)) {
		err = follow_pfn(vma, user_address, &this_pfn);
		if (err)
			break;

		if (pages_done == 0)
			ib->phys = (this_pfn << PAGE_SHIFT) + offset;
		else if (this_pfn != (prev_pfn + 1))
			err = -EFAULT;

		if (err)
			break;

		prev_pfn = this_pfn;
		user_address += PAGE_SIZE;
		pages_done++;
	}

out_up:
	up_read(&current->mm->mmap_sem);

	return err;
}

/*
 * Description:
 *		OEMCrypto API called in user space.
 *		This function reads raw data from the shared memory.
 *		It must be called after write op to vrpc device to read
 *		return value and set the state machine.
 */
static ssize_t dev_read(struct file *fl,
				char __user *buf, size_t size, loff_t *off)
{
	if (device_state == VOEMCRYPTO_DEVICE_IDLE)
		return -EAGAIN;

	if (device_state == VOEMCRYPTO_DEVICE_BUSY)
		return -EBUSY;

	/* For now we assume only shared mem is used.
	 * If not sufficient, we may need to kalloc extra */
	if (size > voemcrypto_data->share_data_size)
		return -EINVAL;

	/* Data is waiting in pmem and waiting to be read */
	copy_to_user(buf, voemcrypto_data->share_data, size);

	/* device is ready for next command */
	device_state = VOEMCRYPTO_DEVICE_IDLE;

	TRACE("Read %d success.\n", size);

	return size;
}


/* OEMCrypto API called in user space */
static ssize_t dev_write(struct file *fl,
			const char __user *buf, size_t size, loff_t *off)
{
	if (device_state != VOEMCRYPTO_DEVICE_IDLE)
		return -EBUSY; /* Shared mem is still being used!*/
	/* For now we assume only shared mem is used.
	 * If not sufficient, we may need to kalloc extra */
	if (size > voemcrypto_data->share_data_size)
		return -EINVAL;

	copy_from_user(voemcrypto_data->share_data, buf, size);

	TRACE("Write %d success.\n", size);

	return size;
}


/*******************************************************************************
* Function:... dev_ioctl
*******************************************************************************/
static long dev_ioctl(struct file *file, unsigned int cmd, unsigned long input)
{
	size_t ret_size = 0;
	void *data = (void *)voemcrypto_data->share_data;

	/*===================================================================*/
	/* this ioctl should always be executed, as it does not use
	 * any device shared resource */
	if (cmd == VOEMC_IOCTL_GETADDR) {
		int err;
		struct voem_input_buffer ib;
		if (copy_from_user(&ib, (void __user *)input, sizeof(ib)))
			return -EFAULT;

		err = get_phys_addr_from_user(&ib);

		/* return to user space */
		if (copy_to_user((void __user *)input, &ib, sizeof(ib)))
			return -EFAULT;

		return err;
	}
	/*===================================================================*/

	if (device_state != VOEMCRYPTO_DEVICE_IDLE)
		return -EBUSY;
		/* Shared mem is still being used!*/
		/* Should probably be waiting then. */

	device_state = VOEMCRYPTO_DEVICE_BUSY;

	DTRACE("Dispatch IOCTL cmd 0x%08X\n", (uint32_t)cmd&0x000000FF);

	switch (cmd) {
	/*===================================================================*/
	case VOEMC_IOCTL_DECRYPTCTR:
	/*===================================================================*/
	{
		struct voemc_decryptctr_transfer_t *req =
					(struct voemc_decryptctr_transfer_t *)input;

		if (copy_from_user(data, (void __user *)input,
			sizeof(struct voemc_decryptctr_transfer_t)))
			return -EFAULT;

		/* copy data to data in channel */
		copy_from_user(voemcrypto_data_in.share_data,
				(void __user *)req->req.data_addr,
				req->req.data_length);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_decryptctr_transfer_t));

		copy_to_user((void __user *)input, voemcrypto_data->share_data, 2*sizeof(uint32_t));  // only need to return result

		device_state = VOEMCRYPTO_DEVICE_IDLE;  // free device

		/* check if need to read output data */
		if (req->dest_buffer_desc.type == voemc_buffer_type_clear &&
			req->req.is_encrypted == true) {
			/* Read from shared out mem */
			if (copy_to_user((void __user *)req->dest_buffer_desc.buffer.clear.address,
							voemcrypto_data_out.share_data,
							req->req.data_length)) {
				pr_err("copy failed");
				return -EFAULT;
			}
		}

		return ret_size;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_WVC_DECRYPT_VIDEO:
	/*===================================================================*/
	{
		struct voem_wvc_decrypt_video_t *req =
			(struct voem_wvc_decrypt_video_t *)input;

		if (copy_from_user(data, (void __user *)input,
			sizeof(struct voem_wvc_decrypt_video_t)))
			return -EFAULT;

		/* copy data to data in channel */
		copy_from_user(voemcrypto_data_in.share_data,
				(void __user *)req->input,
				req->input_length);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voem_wvc_decrypt_video_t));

		copy_to_user((void __user *)input, voemcrypto_data->share_data,
					sizeof(struct voem_wvc_decrypt_video_t));

		device_state = VOEMCRYPTO_DEVICE_IDLE;  // free device

		return ret_size;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_WVC_DECRYPT_AUDIO:
	/*===================================================================*/
	{
		struct voem_wvc_decrypt_audio_t *out;
		struct voem_wvc_decrypt_audio_t *req =
			(struct voem_wvc_decrypt_audio_t *)input;

		if (copy_from_user(data, (void __user *)input,
			sizeof(struct voem_wvc_decrypt_audio_t)))
			return -EFAULT;

		/* copy data to data in channel */
		copy_from_user(voemcrypto_data_in.share_data,
				(void __user *)req->input,
				req->input_length);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voem_wvc_decrypt_audio_t));

		copy_to_user((void __user *)input, voemcrypto_data->share_data, sizeof(struct voem_wvc_decrypt_audio_t));  // only need to return result

		device_state = VOEMCRYPTO_DEVICE_IDLE;  // free device

		out = (struct voem_wvc_decrypt_audio_t *)voemcrypto_data->share_data;
		copy_to_user((void __user *)req->output,
					voemcrypto_data_out.share_data,
					out->output_length);

		return ret_size;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_INIT:
	/*===================================================================*/
	{
		struct voemc_init_t *req =
					(struct voemc_init_t *)data;

		req->cmd = VOEMCRYPTO_CMD_INITIALIZE;
		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_init_t));
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_TERM:
	/*===================================================================*/
	{
		struct voemc_term_t *req =
					(struct voemc_term_t *)data;

		req->cmd = VOEMCRYPTO_CMD_TERMINATE;
		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_term_t));
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_OPEN:
	/*===================================================================*/
	{
		struct voemc_open_t *req =
					(struct voemc_open_t *)data;

		req->cmd = VOEMCRYPTO_CMD_OPEN_SESSION;
		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_open_t));
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_CLOSE:
	/*===================================================================*/
	{
		struct voemc_close_t *req =
					(struct voemc_close_t *)data;

		req->cmd = VOEMCRYPTO_CMD_CLOSE_SESSION;
		req->session = (uint32_t)input;
		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_close_t));
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_GENNONCE:
	/*===================================================================*/
	{
		struct voemc_gennonce_t *req =
					(struct voemc_gennonce_t *)data;

		req->cmd = VOEMCRYPTO_CMD_GEN_NONCE;
		req->session = (uint32_t)input;
		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_gennonce_t));
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_GENDERIVEDKEYS:
	/*===================================================================*/
	{
		struct voemc_genderivedkeys_t *req =
					(struct voemc_genderivedkeys_t *)data;
		uint8_t *mac;
		uint8_t *enc;

		req->cmd = VOEMCRYPTO_CMD_GEN_DERIVED_KEYS;

		/* MAC */
		mac = kmalloc(req->mac_key_context_length, GFP_KERNEL);
		if (mac == NULL)
			return -EAGAIN;

		copy_from_user(mac, req->mac_key_context,
						req->mac_key_context_length);
		/* tell backend where to find data */
		req->mac_key_context = (uint8_t *)voemc_get_pa(mac);


		/* ENC */
		enc = kmalloc(req->enc_key_context_length, GFP_KERNEL);
		if (enc == NULL) {
			kfree(mac);
			return -EAGAIN;
		}
		copy_from_user(enc, req->enc_key_context,
						req->enc_key_context_length);
		/* tell backend where to find data */
		req->enc_key_context = (uint8_t *)voemc_get_pa(enc);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_genderivedkeys_t));

		/* clean up */
		kfree(mac);
		kfree(enc);

		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_GENSIGN:
	/*===================================================================*/
	{
		struct voemc_gensign_t *req =
					(struct voemc_gensign_t *)data;
		uint8_t *msg;

		uint8_t *signature = NULL;
		size_t signature_length = 0;
		uint8_t __user *virt_signature = NULL;
		size_t __user *virt_signature_length = NULL;

		req->cmd = VOEMCRYPTO_CMD_GEN_SIGN;

		msg = kmalloc(req->message_length, GFP_KERNEL);
		if (msg == NULL)
			return -EAGAIN;
		copy_from_user(msg, req->message, req->message_length);
		/* tell backend where to find data */
		req->message = (uint8_t *)voemc_get_pa(msg);


		if (NULL != req->signature) {
			virt_signature = req->signature;

			signature = kmalloc(*req->signature_length, GFP_KERNEL);
			if (NULL == signature) {
				kfree(msg);
				return -EAGAIN;
			}
			/* Update shared pointer with physical address */
			req->signature = (uint8_t *)voemc_get_pa(signature);
		}

		virt_signature_length = req->signature_length;

		signature_length = *virt_signature_length;
		/* Update shared pointer with physical address reference */
		req->signature_length =
				(size_t *)voemc_get_pa(
				(uint8_t *)&signature_length);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_gensign_t));

		/* Update shared pointer with virtual address reference */
		req->signature_length = virt_signature_length;
		*virt_signature_length = signature_length;


		if (NULL != req->signature) {
			copy_to_user(virt_signature, signature,
						signature_length);
			req->signature = virt_signature;
		}

		/* clean up */
		kfree(msg);
		kfree(signature);

		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_LOADKEYS:
	/*===================================================================*/
	{
		struct voemc_loadkeys_t *req =
					(struct voemc_loadkeys_t *)data;
		uint8_t *message = NULL, *phys_message = NULL;
		uint8_t *signature = NULL;
		struct voemc_key_object_t *key_array = NULL;
		uint32_t i = 0;

		req->cmd = VOEMCRYPTO_CMD_LOAD_KEYS;

		/* message */
		message = kmalloc(req->message_length, GFP_KERNEL);
		if (NULL == message)
			return -EAGAIN;
		copy_from_user(message, req->message, req->message_length);
		phys_message = (uint8_t *)voemc_get_pa(message);

		/* signature */
		signature = kmalloc(req->signature_length, GFP_KERNEL);
		if (NULL == signature) {
			kfree(message);
			return -EAGAIN;
		}
		copy_from_user(signature, req->signature,
						req->signature_length);

		/* enc_mac_key_iv */
		if (NULL != req->enc_mac_key_iv) {
			req->enc_mac_key_iv =
				phys_message + ((uint32_t)req->enc_mac_key_iv -
						(uint32_t)req->message);
		}

		/* enc_mac_keys */
		if (NULL != req->enc_mac_keys) {
			req->enc_mac_keys =
				phys_message + ((uint32_t)req->enc_mac_keys -
						(uint32_t)req->message);
		}

		/* key_array */
		key_array = kmalloc(
			sizeof(struct voemc_key_object_t)*req->num_keys,
								GFP_KERNEL);
		if (NULL == key_array) {
			kfree(message);
			kfree(signature);
			return -EAGAIN;
		}
		if (NULL != req->pst) {
			req->pst = phys_message +
				(((uint32_t) req->pst) -
				(uint32_t) req->message);
				}

		for (i = 0; i < req->num_keys; i++) {
			key_array[i].key_id_length =
				req->key_array[i].key_id_length;
			key_array[i].key_id =
				(NULL == req->key_array[i].key_id) ? 0 :
				phys_message +
				((uint32_t)req->key_array[i].key_id -
				(uint32_t) req->message);
			key_array[i].key_data_iv =
				(NULL == req->key_array[i].key_data_iv) ? 0 :
				phys_message +
				((uint32_t) req->key_array[i].key_data_iv -
				(uint32_t) req->message);
			key_array[i].key_data_length =
				req->key_array[i].key_data_length;
			key_array[i].key_data =
				(NULL == req->key_array[i].key_data) ? 0 :
				phys_message +
				((uint32_t) req->key_array[i].key_data -
				(uint32_t) req->message);
			key_array[i].key_control_iv =
				(NULL == req->key_array[i].key_control_iv) ? 0 :
				phys_message +
				((uint32_t) req->key_array[i].key_control_iv -
				(uint32_t) req->message);
			key_array[i].key_control =
				(NULL == req->key_array[i].key_control) ? 0 :
				phys_message +
				((uint32_t) req->key_array[i].key_control -
				(uint32_t) req->message);
		}

		/* Update shared pointer with physical address reference */
		req->message = phys_message;
		req->signature = (uint8_t *)voemc_get_pa(signature);
		req->key_array =
			(struct voemc_key_object_t *)voemc_get_pa(
			(uint8_t *)key_array);


		/* Invoke RPC call */
		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_loadkeys_t));


		/* Cleanup */
		kfree(message);
		kfree(signature);
		kfree(key_array);
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_REFRESHKEYS:
	/*===================================================================*/
	{
		struct voemc_refreshkeys_t *req =
					(struct voemc_refreshkeys_t *)data;
		uint8_t *message = NULL, *phys_message = NULL;
		uint8_t *signature = NULL;
		struct voemc_key_refresh_object_t *key_array = NULL;
		uint32_t i = 0;


		req->cmd = VOEMCRYPTO_CMD_REFRESH_KEYS;

		/* message */
		message = kmalloc(req->message_length, GFP_KERNEL);
		if (NULL == message)
			return -EAGAIN;

		copy_from_user(message, req->message, req->message_length);
		phys_message = (uint8_t *)voemc_get_pa(message);

		/* signature */
		signature = kmalloc(req->signature_length, GFP_KERNEL);
		if (NULL == signature) {
			kfree(message);
			return -EAGAIN;
		}
		copy_from_user(signature, req->signature,
						req->signature_length);

		/* key_array */
		key_array = kmalloc(
			sizeof(struct voemc_key_refresh_object_t)*req->num_keys,
								GFP_KERNEL);

		if (NULL == key_array) {
			kfree(message);
			kfree(signature);
			return -EAGAIN;
		}
		for (i = 0; i < req->num_keys; i++) {
			key_array[i].key_id_length =
				req->key_array[i].key_id_length;
			key_array[i].key_id =
				(NULL == req->key_array[i].key_id) ? 0 :
				phys_message +
				((uint32_t) req->key_array[i].key_id -
				(uint32_t)req->message);
			key_array[i].key_control_iv =
				(NULL == req->key_array[i].key_control_iv) ? 0 :
				phys_message +
				((uint32_t) req->key_array[i].key_control_iv -
				(uint32_t)req->message);
			key_array[i].key_control =
				(NULL == req->key_array[i].key_control) ? 0 :
				phys_message +
				((uint32_t) req->key_array[i].key_control -
				(uint32_t) req->message);
		}

		/* Update shared pointer with physical address reference */
		req->message = phys_message;
		req->signature = (uint8_t *)voemc_get_pa(signature);
		req->key_array =
			(struct voemc_key_refresh_object_t *)voemc_get_pa(
							(uint8_t *)key_array);

		/* Invoke RPC call */
		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_loadkeys_t));

		/* Cleanup */
		kfree(message);
		kfree(signature);
		kfree(key_array);
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_INSTALLKEYBOX:
	/*===================================================================*/
	{
		struct voemc_installkeybox_t *req =
			(struct voemc_installkeybox_t *)data;
		uint8_t *keybox;

		req->cmd = VOEMCRYPTO_CMD_INSTALL_KEYBOX;

		keybox = kmalloc(req->keybox_length, GFP_KERNEL);
		if (keybox == NULL)
			return -EAGAIN;
		copy_from_user(keybox, req->keybox, req->keybox_length);
		/* Tell backend where to find data */
		req->keybox = (uint8_t *)voemc_get_pa(keybox);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_installkeybox_t));

		kfree(keybox);
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_ISKEYBOXVALID:
	/*===================================================================*/
	{
		struct voemc_iskeyboxvalid_t *req =
			(struct voemc_iskeyboxvalid_t *)data;
		req->cmd = VOEMCRYPTO_CMD_IS_KEYBOX_VALID;
		ret_size = voemcrypto_call_ext(voemcrypto_data,
			sizeof(struct voemc_iskeyboxvalid_t));
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_GETDEVICEID:
	/*===================================================================*/
	{
		struct voemc_getdevid_t *req = (struct voemc_getdevid_t *)data;
		uint8_t *devid;
		uint8_t __user *user;

		req->cmd = VOEMCRYPTO_CMD_GET_DEV_ID;

		/* Allocate contiguous memory */
		devid = kmalloc(req->id_length, GFP_KERNEL);
		if (devid == NULL)
			return -EAGAIN;

		/* Backup virtual address from user space */
		user = req->device_id;

		/* Translate and store physical address in shared memory */
		req->device_id = (uint8_t *)voemc_get_pa(devid);

		/* Execute RPC command */
		ret_size = voemcrypto_call_ext(voemcrypto_data,
			sizeof(struct voemc_getdevid_t));

		/* Copy device ID from allocated memory to user space */
		copy_to_user(user, devid, req->id_length);

		kfree(devid);
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_GETKEYDATA:
	/*===================================================================*/
	{
		struct voemc_getkeydata_t *req =
					(struct voemc_getkeydata_t *)data;
		uint8_t *key;
		uint8_t __user *user;

		req->cmd = VOEMCRYPTO_CMD_GET_KEYDATA;

		key = kmalloc(req->key_data_length, GFP_KERNEL);

		if (key == NULL)
			return -EAGAIN;

		user = req->key_data;
		/* Tell backend where to put output */
		req->key_data = (uint8_t *)voemc_get_pa(key);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_getkeydata_t));

		copy_to_user(user, key, req->key_data_length);

		kfree(key);
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_GETRANDOM:
	/*===================================================================*/
	{
		struct voemc_getrandom_t *req =
					(struct voemc_getrandom_t *)data;
		uint8_t *random;
		uint8_t __user *user;

		req->cmd = VOEMCRYPTO_CMD_GET_RANDOM;

		random = kmalloc(req->data_length, GFP_KERNEL);
		if (random == NULL)
			return -EAGAIN;

		user = req->random_data;
		/* Tell backend where to put output */
		req->random_data = (uint8_t *)voemc_get_pa(random);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_getrandom_t));

		copy_to_user(user, random, req->data_length);

		kfree(random);
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_WRAPKEYBOX:
	/*===================================================================*/
	{
		struct voemc_wrapkeybox_t *req =
					(struct voemc_wrapkeybox_t *)data;
		uint8_t *key = NULL, *wrapped = NULL, *transkey = NULL;
		uint8_t __user *user;

		req->cmd = VOEMCRYPTO_CMD_WRAP_KEYBOX;

		key = kmalloc(req->keybox_length, GFP_KERNEL);
		wrapped = kmalloc(req->wrapped_keybox_length, GFP_KERNEL);
		if (key == NULL || wrapped == NULL) {
			kfree(key);
			kfree(wrapped);
			return -EAGAIN;
		}

		/* Transfer to physical memory for backend to use */
		copy_from_user(key, req->keybox, req->keybox_length);
		req->keybox = (uint8_t *)voemc_get_pa(key);

		/* save user buffer for later use */
		user = req->wrapped_keybox;
		/* tell backend where to store output */
		req->wrapped_keybox = (uint8_t *)voemc_get_pa(wrapped);

		if ((req->transport_key) && (req->transport_key_length)) {
			transkey = kmalloc(req->transport_key_length,
								GFP_KERNEL);

			if (transkey == NULL) {
				kfree(key);
				kfree(wrapped);
				return -EAGAIN;
			}
			copy_from_user(transkey, req->transport_key,
						req->transport_key_length);
			req->transport_key = (uint8_t *)voemc_get_pa(transkey);
		}


		ret_size = voemcrypto_call_ext(voemcrypto_data,
			sizeof(struct voemc_wrapkeybox_t));

		copy_to_user(user, wrapped, req->wrapped_keybox_length);

		/* clean up */
		kfree(key);
		kfree(wrapped);
		kfree(transkey);
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_REWRAPDEVRSAKEY:
	/*===================================================================*/
	{
		struct voemc_rewrapdevrsakey_t *req =
					(struct voemc_rewrapdevrsakey_t *)data;

		uint8_t *message = NULL, *phys_message = NULL;
		uint8_t *signature = NULL;
		uint8_t *wrapped_rsa_key = NULL;
		size_t wrapped_rsa_key_length = 0;
		uint8_t __user *virt_wrapped_rsa_key = NULL;
		size_t __user *virt_wrapped_rsa_key_length = 0;

		req->cmd = VOEMCRYPTO_CMD_REWRAP_DEV_RSA_KEY;

		message = kmalloc(req->message_length, GFP_KERNEL);

		if (NULL == message)
			return -EAGAIN;


		signature = kmalloc(req->signature_length, GFP_KERNEL);

		if (NULL == signature) {
			kfree(message);
			return -EAGAIN;
		}

		if (req->wrapped_rsa_key) {
			wrapped_rsa_key = kmalloc(*req->wrapped_rsa_key_length,
								GFP_KERNEL);

			if (NULL == wrapped_rsa_key) {
				kfree(message);
				kfree(signature);
				return -EAGAIN;
			}
		}

		/* transfer to physical memory for backend to use */
		copy_from_user(message, req->message, req->message_length);
		phys_message = (uint8_t *)voemc_get_pa(message);
		copy_from_user(signature, req->signature,
						req->signature_length);
		wrapped_rsa_key_length = *req->wrapped_rsa_key_length;

		if (req->wrapped_rsa_key)
			virt_wrapped_rsa_key = req->wrapped_rsa_key;

		virt_wrapped_rsa_key_length = req->wrapped_rsa_key_length;

		req->nonce =
			(uint32_t *)(phys_message + ((uint32_t) req->nonce -
						(uint32_t)req->message));
		req->enc_rsa_key =
			phys_message + ((uint32_t) req->enc_rsa_key -
						(uint32_t)req->message);
		req->enc_rsa_key_iv =
			phys_message + ((uint32_t) req->enc_rsa_key_iv -
						(uint32_t)req->message);
		req->message = phys_message;
		req->signature = (uint8_t *)voemc_get_pa(signature);

		if (req->wrapped_rsa_key)
			req->wrapped_rsa_key =
				(uint8_t *)voemc_get_pa(wrapped_rsa_key);

		req->wrapped_rsa_key_length =
			(size_t *)voemc_get_pa(
			(uint8_t *)&wrapped_rsa_key_length);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
				sizeof(struct voemc_rewrapdevrsakey_t));

		*virt_wrapped_rsa_key_length = wrapped_rsa_key_length;
		if (req->wrapped_rsa_key)
			copy_to_user(virt_wrapped_rsa_key, wrapped_rsa_key,
					wrapped_rsa_key_length);

		/* Cleanup */
		kfree(message);
		kfree(signature);
		kfree(wrapped_rsa_key);
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_LOADDEVRSAKEY:
	/*===================================================================*/
	{
		struct voemc_loaddevrsakey_t *req =
			(struct voemc_loaddevrsakey_t *)data;
		uint8_t *key = NULL;

		req->cmd = VOEMCRYPTO_CMD_LOAD_DEV_RSA_KEY;

		key = kmalloc(req->wrapped_rsa_key_length, GFP_KERNEL);
		if (key == NULL)
			return -EAGAIN;

		/* transfer to physical memory for backend to use */
		copy_from_user(key, req->wrapped_rsa_key,
						req->wrapped_rsa_key_length);
		req->wrapped_rsa_key = (uint8_t *)voemc_get_pa(key);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_loaddevrsakey_t));

		kfree(key);
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_GENRSASIGN:
	/*===================================================================*/
	{
		struct voemc_genrsasign_t *req =
			(struct voemc_genrsasign_t *)data;
		uint8_t *key = NULL, *sign = NULL;
		uint8_t __user *user = NULL;

		req->cmd = VOEMCRYPTO_CMD_GEN_RSA_SIGN;

		key = kmalloc(req->message_length, GFP_KERNEL);

		if (key == NULL)
			return -EAGAIN;

		if (NULL != req->signature) {
			sign = kmalloc(req->signature_length, GFP_KERNEL);
			if (NULL == sign) {
				kfree(key);
				return -EAGAIN;
			}
		}

		/* transfer to physical memory for backend to use */
		copy_from_user(key, req->message, req->message_length);
		req->message = (uint8_t *)voemc_get_pa(key);

		if (NULL != req->signature) {
			user = req->signature;
			req->signature = (uint8_t *)voemc_get_pa(sign);
		}

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_genrsasign_t));

		if (NULL != req->signature && NULL != user)
			copy_to_user(user, sign, req->signature_length);

		kfree(sign);
		kfree(key);
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_DERKEYSFROMSESSIONKEY:
	/*===================================================================*/
	{
		struct voemc_derkeysfrsessionkey_t *req =
			(struct voemc_derkeysfrsessionkey_t *)data;
		uint8_t *sessionkey = NULL, *mackey = NULL, *enckey = NULL;

		req->cmd = VOEMCRYPTO_CMD_DERIVE_KEYS_FROM_SESSION_KEY;

		sessionkey = kmalloc(req->enc_session_key_length, GFP_KERNEL);
		mackey = kmalloc(req->mac_key_context_length, GFP_KERNEL);
		enckey = kmalloc(req->enc_key_context_length, GFP_KERNEL);
		if (sessionkey == NULL || mackey == NULL || enckey == NULL) {
			kfree(sessionkey);
			kfree(mackey);
			kfree(enckey);
			return -EAGAIN;
		}

		/* transfer to physical memory for backend to use */
		copy_from_user(sessionkey, req->enc_session_key,
						req->enc_session_key_length);
		req->enc_session_key = (uint8_t *)voemc_get_pa(sessionkey);

		copy_from_user(mackey, req->mac_key_context,
						req->mac_key_context_length);
		req->mac_key_context = (uint8_t *)voemc_get_pa(mackey);

		copy_from_user(enckey, req->enc_key_context,
						req->enc_key_context_length);
		req->enc_key_context = (uint8_t *)voemc_get_pa(enckey);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
				sizeof(struct voemc_derkeysfrsessionkey_t));

		/* clean up */
		kfree(sessionkey);
		kfree(mackey);
		kfree(enckey);

		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_GENENCRYPT:
	/*===================================================================*/
	{
		struct voemc_genencrypt_t *req =
			(struct voemc_genencrypt_t *)data;
		uint8_t *in = NULL, *out = NULL;
		uint8_t __user *user;

		req->cmd = VOEMCRYPTO_CMD_GENERIC_ENCRYPT;

		in = kmalloc(req->buffer_length, GFP_KERNEL);
		out = kmalloc(req->buffer_length, GFP_KERNEL);
		if (in == NULL || out == NULL) {
			kfree(in);
			kfree(out);
			return -EAGAIN;
		}

		copy_from_user(in, req->in_buffer, req->buffer_length);
		req->in_buffer = (uint8_t *)voemc_get_pa(in);
		user = req->out_buffer;
		req->out_buffer = (uint8_t *)voemc_get_pa(out);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_genencrypt_t));

		copy_to_user(user, out, req->buffer_length);

		/* clean up */
		kfree(in);
		kfree(out);

		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_GENDECRYPT:
	/*===================================================================*/
	{
		struct voemc_gendecrypt_t *req =
			(struct voemc_gendecrypt_t *)data;
		uint8_t *in = NULL, *out = NULL;
		uint8_t __user *user;

		req->cmd = VOEMCRYPTO_CMD_GENERIC_DECRYPT;

		in = kmalloc(req->buffer_length, GFP_KERNEL);
		out = kmalloc(req->buffer_length, GFP_KERNEL);

		if (in == NULL || out == NULL) {
			kfree(in);
			kfree(out);
			return -EAGAIN;
		}

		copy_from_user(in, req->in_buffer, req->buffer_length);
		req->in_buffer = (uint8_t *)voemc_get_pa(in);
		user = req->out_buffer;
		req->out_buffer = (uint8_t *)voemc_get_pa(out);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
			sizeof(struct voemc_gendecrypt_t));

		copy_to_user(user, out, req->buffer_length);

		/* clean up */
		kfree(in);
		kfree(out);

		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_GENERICSIGN:
	/*===================================================================*/
	{
		struct voemc_genericsign_t *req =
			(struct voemc_genericsign_t *)data;
		uint8_t *in = NULL, *out = NULL;
		uint8_t __user *user;

		req->cmd = VOEMCRYPTO_CMD_GENERIC_SIGN;

		in = kmalloc(req->buffer_length, GFP_KERNEL);
		out = kmalloc(req->signature_length, GFP_KERNEL);

		if (in == NULL || out == NULL) {
			kfree(in);
			kfree(out);
			return -EAGAIN;
		}

		copy_from_user(in, req->in_buffer, req->buffer_length);
		req->in_buffer = (uint8_t *)voemc_get_pa(in);
		user = req->signature;
		req->signature = (uint8_t *)voemc_get_pa(out);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
					sizeof(struct voemc_genericsign_t));

		copy_to_user(user, out, req->signature_length);

		/* clean up */
		kfree(in);
		kfree(out);

		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_GENVERIFY:
	/*===================================================================*/
	{
		struct voemc_genverify_t *req =
			(struct voemc_genverify_t *)data;
		uint8_t *in = NULL, *sign = NULL;

		req->cmd = VOEMCRYPTO_CMD_GENERIC_VERIFY;

		in = kmalloc(req->buffer_length, GFP_KERNEL);
		sign = kmalloc(req->signature_length, GFP_KERNEL);

		if (in == NULL || sign == NULL) {
			kfree(in);
			kfree(sign);
			return -EAGAIN;
		}

		copy_from_user(in, req->in_buffer, req->buffer_length);
		req->in_buffer = (uint8_t *)voemc_get_pa(in);
		copy_from_user(sign, req->signature, req->signature_length);
		req->signature = (uint8_t *)voemc_get_pa(sign);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
			sizeof(struct voemc_genverify_t));

		/* clean up */
		kfree(in);
		kfree(sign);

		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_UPDATEUSAGETABLE:
	/*===================================================================*/
	{
		struct voemc_update_usage_table_t *req =
			(struct voemc_update_usage_table_t *)data;
		req->cmd = VOEMCRYPTO_CMD_UPDATEUSAGETABLE;
		ret_size = voemcrypto_call_ext(voemcrypto_data,
			sizeof(struct voemc_update_usage_table_t));
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_DEACTIVEUSAGEENTRY:
	/*===================================================================*/
	{
		struct voemc_deactive_usage_entry_t *req =
			(struct voemc_deactive_usage_entry_t *)data;
		uint8_t *pst = NULL;

		req->cmd = VOEMCRYPTO_CMD_DEACTIVEUSAGEENTRY;

		pst = kmalloc(req->pst_length, GFP_KERNEL);

		if (pst == NULL)
			return -EAGAIN;

		copy_from_user(pst, req->pst, req->pst_length);
		req->pst = (uint8_t *)voemc_get_pa(pst);
		ret_size = voemcrypto_call_ext(voemcrypto_data,
			sizeof(struct voemc_deactive_usage_entry_t));

		/* clean up */
		kfree(pst);
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_REPORTUSAGE:
	/*===================================================================*/
	{
		struct voemc_report_usage_t *req =
			(struct voemc_report_usage_t *)data;

		uint8_t *pst = NULL, *rptbuffer = NULL;
		uint8_t __user *rptbuffer_old = NULL;

		rptbuffer_old = req->rpt_buffer;
		req->cmd = VOEMCRYPTO_CMD_REPORTUSAGE;
		pst = kmalloc(req->pst_length, GFP_KERNEL);

		if (pst == NULL)
			return -EAGAIN;

		rptbuffer = kmalloc((size_t)req->rpt_buffer_length, GFP_KERNEL);

		if (rptbuffer == NULL) {
			kfree(pst);
			return -EAGAIN;
		}

		copy_from_user(pst, req->pst, req->pst_length);
		req->pst = (uint8_t *)voemc_get_pa(pst);
		req->rpt_buffer = (uint8_t *)voemc_get_pa(rptbuffer);
		ret_size = voemcrypto_call_ext(voemcrypto_data,
			sizeof(struct voemc_report_usage_t));
		copy_to_user(rptbuffer_old, rptbuffer,
			(size_t)req->rpt_buffer_length);

		req->rpt_buffer = rptbuffer_old;

		kfree(pst);
		kfree(rptbuffer);

		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_DELETEUSAGEENTRY:
	/*===================================================================*/
	{
		struct voemc_delete_usage_entry_t *req =
			(struct voemc_delete_usage_entry_t *)data;
		uint8_t *message = NULL, *signature = NULL;
		uint8_t *phy_message = NULL;

		req->cmd = VOEMCRYPTO_CMD_DELETEUSAGEENTRY;
		message = kmalloc(req->message_length, GFP_KERNEL);

		if (message == NULL)
			return -EAGAIN;

		signature = kmalloc(req->signature_length, GFP_KERNEL);

		if (signature == NULL) {
			kfree(message);
			return -EAGAIN;
		}

		copy_from_user(message,
				req->message,
				req->message_length);
		phy_message = (uint8_t *)voemc_get_pa(message);
		req->pst = phy_message + (req->pst - req->message);
		req->message = phy_message;
		copy_from_user(signature,
			req->signature,
			req->signature_length);
		req->signature = (uint8_t *)voemc_get_pa(signature);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
				sizeof(struct voemc_delete_usage_entry_t));

		/* clean up */
		kfree(message);
		kfree(signature);

		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_DELETEUSAGETABLE:
	/*===================================================================*/
	{
		struct voemc_delete_usage_table_t *req =
			(struct voemc_delete_usage_table_t *)data;
		req->cmd = VOEMCRYPTO_CMD_DELETEUSAGETABLE;
		ret_size = voemcrypto_call_ext(voemcrypto_data,
			sizeof(struct voemc_delete_usage_table_t));

		/* clean up */
		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_WVC_SET_ENTITLEMENT_KEY:
	/*===================================================================*/
	{
		struct voem_wvc_set_entitlement_key_t *req =
			(struct voem_wvc_set_entitlement_key_t *)data;
		uint8_t *in = NULL;

		req->cmd = VOEMCRYPTO_CMD_WVC_SET_ENTITLEMENT_KEY;
		in = kmalloc(req->emm_key_length, GFP_KERNEL);

		if (in == NULL) {
			kfree(in);
			return -EAGAIN;
		}

		copy_from_user(in, req->emm_key, req->emm_key_length);
		req->emm_key = (uint8_t *)virt_to_phys(in);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
			sizeof(struct voem_wvc_set_entitlement_key_t));

		/* clean up */
		kfree(in);

		break;
	}

	/*===================================================================*/
	case VOEMC_IOCTL_WVC_DERIVE_CONTROL_WORD:
	/*===================================================================*/
	{
		struct voem_wvc_derive_control_word_t *req =
			(struct voem_wvc_derive_control_word_t *)data;
		uint8_t *in = NULL;
		uint32_t *out = NULL;
		uint8_t __user *user;

		req->cmd = VOEMCRYPTO_CMD_WVC_DERIVE_CONTROL_WORD;
		in = kmalloc(req->length, GFP_KERNEL);
		out = kmalloc(sizeof(uint32_t), GFP_KERNEL);

		if (NULL == in || NULL == out) {
			kfree(in);
			kfree(out);
			return -EAGAIN;
		}

		copy_from_user(in, req->ecm, req->length);
		req->ecm = (uint8_t *)virt_to_phys(in);
		user = (uint8_t *)req->flags;
		req->flags = (uint32_t *)virt_to_phys(out);

		ret_size = voemcrypto_call_ext(voemcrypto_data,
			sizeof(struct voem_wvc_derive_control_word_t));

		copy_to_user(user, out, sizeof(uint32_t));

		/* clean up */
		kfree(in);
		kfree(out);

		 break;
	}
	/*===================================================================*/
	default:
	/*===================================================================*/
	{
		ETRACE("Bad command.\n");
		device_state = VOEMCRYPTO_DEVICE_IDLE;
		return -EINVAL;
	}
	}
	device_state = VOEMCRYPTO_DEVICE_WAITING_FOR_READ;
	TRACE("ioctl: success %d.\n", ret_size);

	return ret_size;
}

loff_t dev_llseek(struct file *file, loff_t pos, int whence)
{
	loff_t newpos = -1;

	switch (whence) {
	case SEEK_SET:
		newpos = pos;
		break;
	case SEEK_CUR:
		newpos = file->f_pos + pos;
		break;
	case SEEK_END:
		/* should probably never use this */
		break;
	}
	if (newpos < 0)
		return -EINVAL;

	file->f_pos = newpos;
	return newpos;
}

static ssize_t dev_voemclose(struct inode *an, struct file *ff)
{
	if (!an) {
		int i = 0;
		i = (int)(an);
	};

	if (!ff) {
		int jj = 0;
		jj = (int)ff;
	}

	device_state = VOEMCRYPTO_DEVICE_IDLE;
	return 0;
}

/**** Shared Memory Device creation (interface to Android space) *****/
static const struct file_operations voemcrypto_fops = {
	.owner	= THIS_MODULE,
	/*.open	= dev_open,*/
	.llseek	= dev_llseek,
	.read	= dev_read,
	.write = dev_write,
	.unlocked_ioctl = dev_ioctl,
	.release = dev_voemclose
};

static struct miscdevice voemcrypto_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "voemcrypto",
	.fops	= &voemcrypto_fops,
};

static void voemcrypto_on_connect(uint32_t token, void *cookie)
{
	struct voemcrypto_t *p_voemcrypto = (struct voemcrypto_t *)cookie;
	p_voemcrypto->status = VOEC_CTL_STATUS_CONNECT;
	wake_up_interruptible(&p_voemcrypto->open_wait);
}

static void voemcrypto_on_disconnect(uint32_t token, void *cookie)
{
	struct voemcrypto_t *p_voemcrypto = (struct voemcrypto_t *)cookie;
	p_voemcrypto->status = VOEC_CTL_STATUS_DISCONNECT;
}

static void voemcrypto_on_event(uint32_t token, uint32_t event_id, void *cookie)
{
	struct voemcrypto_t *p_voemcrypto = (struct voemcrypto_t *)cookie;
	switch (event_id) {
	case VOEC_CTL_EVENT_CALL:
		p_voemcrypto->respond = 1;
		wake_up_interruptible(&p_voemcrypto->client_wait);
	break;
	default:
	break;
	}
}

static struct mbox_ops voemcrypto_ops = {
	.on_connect    = voemcrypto_on_connect,
	.on_disconnect = voemcrypto_on_disconnect,
	.on_event      = voemcrypto_on_event
};

static int __init voemcrypto_init(void)
{
	int ret;
	struct voemcrypto_t *p_voemcrypto;


	TRACE("Initializing.\n");

	p_voemcrypto = kzalloc(sizeof(struct voemcrypto_t), GFP_KERNEL);
	if (!p_voemcrypto) {
		ETRACE("Out of memory.\n");
		return -ENOMEM;
	}

	mutex_init(&p_voemcrypto->call_mutex);
	init_waitqueue_head(&p_voemcrypto->client_wait);
	init_waitqueue_head(&p_voemcrypto->open_wait);

	p_voemcrypto->token = mv_ipc_mbox_get_info("security",
					"oec_ctl",
					&voemcrypto_ops,
					&(p_voemcrypto->share_data),
					&(p_voemcrypto->share_data_size),
					&(p_voemcrypto->cmdline),
					(void *)p_voemcrypto);

	/* compare with largest struct */
	if (p_voemcrypto->share_data_size < sizeof(struct voemc_loadkeys_t)) {
		ETRACE("share data size too small.\n");
		ret = -EINVAL;
		goto error;
	}

	/* store the new allocated struct for later use. */
	voemcrypto_data = p_voemcrypto;

#if 0
	/* create procfs device */
	proc_file_entry = proc_create("voemcrypto", 0666,
				NULL, &voemcrypto_fops);

	if (proc_file_entry == NULL) {
		ETRACE("Could not create proc device.\n");
		ret = -ENOMEM;
		goto error;
	}
#endif

	/* create device /dev/voemcrypto */
	ret = misc_register(&voemcrypto_miscdev);
	if (ret) {
		ETRACE("Could not create misc device.\n");
		ret = -ENOMEM;
		goto error;
	}
	p_voemcrypto->status = VOEC_CTL_STATUS_DISCONNECT;
	mv_mbox_set_online(p_voemcrypto->token);

	p_voemcrypto = &voemcrypto_data_in;
	p_voemcrypto->token = mv_ipc_mbox_get_info("security",
					"oec_dat_in",
					&voemcrypto_ops,
					&(p_voemcrypto->share_data),
					&(p_voemcrypto->share_data_size),
					&(p_voemcrypto->cmdline),
					(void *)p_voemcrypto);

	p_voemcrypto = &voemcrypto_data_out;
	p_voemcrypto->token = mv_ipc_mbox_get_info("security",
					"oec_dat_out",
					&voemcrypto_ops,
					&(p_voemcrypto->share_data),
					&(p_voemcrypto->share_data_size),
					&(p_voemcrypto->cmdline),
					(void *)p_voemcrypto);

	TRACE("Initialized.\n");
	return 0;
error:
	kfree(p_voemcrypto);
	return ret;
}

#if 0
static void __exit voemcrypto_exit(void)
{
	kfree(voemcrypto_data);
	if (proc_file_entry)
		proc_remove(proc_file_entry);
}
#endif

device_initcall(voemcrypto_init);


/*----- Module description -----*/
MODULE_AUTHOR("Chee Leong, Ng <chee.leong.ng@intel.com>");
MODULE_DESCRIPTION("VLX OEMCrypto driver (VOEMCRYPTO-FE)");
MODULE_LICENSE("GPL");
MODULE_ALIAS("voemcrypto-fe");

/*----- End of file -----*/
