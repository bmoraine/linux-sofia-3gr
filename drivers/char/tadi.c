/* ----------------------------------------------------------------------
 * Copyright (C) 2011-2013 Intel Mobile Communications GmbH
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Revision Information:
 * File name:  tadi.c $
 * Date:	    2013-07-18	13:52:09 UTC $
 * ----------------------------------------------------------------------
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>		   /*for kmalloc, kfree */
#include <linux/spinlock_types.h>
#include <linux/spinlock.h>
#include <linux/aio.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/tadi.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/console.h>
#include <linux/of.h>
#include <linux/sizes.h>
/* this linux tadi char driver to be used for kernel >= 2.6.36 */

/* constants */
/*#define TADI_DEV_PHASE 1
#define TADI_DEBUG_ON 1*/
#define TADI_MODULE_NAME "tadi"
#define TADI_DRV_VERSION 2
#define INVALID_MSG_TYPE 0xFF
#define INVALID_CMD 0xFF

#define TADO_CHID_GENERAL_PURPOSE_START 192
#define TADO_CHID_GENERAL_PURPOSE_END 215
#define TADO_CHID_OVERLOAD 254
#define TADO_CHID_UNDEFINED_CHANNEL 255
#define TADO_FIFO_NUMBER_OF_CHIDS (256)
#define TADO_FIFO_BASE_RANGE (0x01000000)
#define TADO_FIFO_BASE_ADDR fifo_start
#define FIFO_LAST_DATA_OFFSET (0x8000)

#define TADO_FIFO_CHID_RANGE \
	(TADO_FIFO_BASE_RANGE / TADO_FIFO_NUMBER_OF_CHIDS)

#define TADO_FIFO_GET_ADDR(ch_id) \
	((ch_id * TADO_FIFO_CHID_RANGE) + TADO_FIFO_BASE_ADDR)

#define TADO_HANDLE_TO_CHID(_h) \
	(((((uintptr_t)_h)) - TADO_FIFO_BASE_ADDR) / TADO_FIFO_CHID_RANGE)

#define TRCDBG_TADO_FILL(x_) \
	((x_ < TADO_CHID_GENERAL_PURPOSE_START) \
		? TADO_CHID_UNDEFINED_CHANNEL : \
	((x_ == TADO_CHID_GENERAL_PURPOSE_END) \
		? TADO_CHID_UNDEFINED_CHANNEL : \
	((x_ > TADO_CHID_GENERAL_PURPOSE_END) \
		? TADO_CHID_UNDEFINED_CHANNEL : \
	((x_ + 1)))))

/* low level fifo write macros */
#ifdef TADI_DEV_PHASE
	#ifdef TADI_DEBUG_ON
		#define TADI_DBG(fmt, arg...) \
			pr_info(TADI_MODULE_NAME": " fmt, ##arg)
	#else
		#define TADI_DBG(fmt, arg...)
	#endif
	#define FIFO_W32(handle, data) {\
		iowrite32(data, handle);\
		TADI_DBG("W32: %p @ %x\n", handle, data); }
	#define FIFO_W16(handle, data) {\
		iowrite16(data, handle);\
		TADI_DBG("W16: %p @ %x\n", handle, data); }
	#define FIFO_W8(handle, data) {\
		iowrite8(data, handle);\
		TADI_DBG("W8: %p @ %x\n", handle, data); }
#else /* TADI_DEV_PHASE */
	#ifdef TADI_DEBUG_ON
		#define TADI_DBG(fmt, arg...) \
			pr_debug(TADI_MODULE_NAME": " fmt, ##arg)
	#else
		#define TADI_DBG(fmt, arg...)
	#endif
	#define FIFO_W32(handle, data) iowrite32(data, handle);
	#define FIFO_W16(handle, data) iowrite16(data, handle);
	#define FIFO_W8(handle, data) iowrite8(data, handle);
#endif /* TADI_DEV_PHASE */

static int tadi_driver_probe(struct platform_device *_dev);
static int tadi_driver_remove(struct platform_device *_dev);

struct s_tadi_priv {
	void  *handler;
	int   mt;
};

/* Global variables */
static int rev = TADI_DRV_VERSION;
static uintptr_t fifo_start;
static int tadi_major_number;
static struct class *class_tadi;
static unsigned char tado_free_ch_id_count =
	TADO_CHID_GENERAL_PURPOSE_END - TADO_CHID_GENERAL_PURPOSE_START;
static unsigned char tado_free_ch_id_root   = TADO_CHID_GENERAL_PURPOSE_START;
static unsigned char tado_free_ch_id_last   = TADO_CHID_GENERAL_PURPOSE_END;
static unsigned char tado_ch_id_array[TADO_FIFO_NUMBER_OF_CHIDS] = {
	TRCDBG_TADO_FILL(0x00), TRCDBG_TADO_FILL(0x01), TRCDBG_TADO_FILL(0x02),
	TRCDBG_TADO_FILL(0x03), TRCDBG_TADO_FILL(0x04), TRCDBG_TADO_FILL(0x05),
	TRCDBG_TADO_FILL(0x06), TRCDBG_TADO_FILL(0x07), TRCDBG_TADO_FILL(0x08),
	TRCDBG_TADO_FILL(0x09), TRCDBG_TADO_FILL(0x0a), TRCDBG_TADO_FILL(0x0b),
	TRCDBG_TADO_FILL(0x0c), TRCDBG_TADO_FILL(0x0d), TRCDBG_TADO_FILL(0x0e),
	TRCDBG_TADO_FILL(0x0f), TRCDBG_TADO_FILL(0x10), TRCDBG_TADO_FILL(0x11),
	TRCDBG_TADO_FILL(0x12), TRCDBG_TADO_FILL(0x13), TRCDBG_TADO_FILL(0x14),
	TRCDBG_TADO_FILL(0x15), TRCDBG_TADO_FILL(0x16), TRCDBG_TADO_FILL(0x17),
	TRCDBG_TADO_FILL(0x18), TRCDBG_TADO_FILL(0x19), TRCDBG_TADO_FILL(0x1a),
	TRCDBG_TADO_FILL(0x1b), TRCDBG_TADO_FILL(0x1c), TRCDBG_TADO_FILL(0x1d),
	TRCDBG_TADO_FILL(0x1e), TRCDBG_TADO_FILL(0x1f), TRCDBG_TADO_FILL(0x20),
	TRCDBG_TADO_FILL(0x21), TRCDBG_TADO_FILL(0x22), TRCDBG_TADO_FILL(0x23),
	TRCDBG_TADO_FILL(0x24), TRCDBG_TADO_FILL(0x25), TRCDBG_TADO_FILL(0x26),
	TRCDBG_TADO_FILL(0x27), TRCDBG_TADO_FILL(0x28), TRCDBG_TADO_FILL(0x29),
	TRCDBG_TADO_FILL(0x2a), TRCDBG_TADO_FILL(0x2b), TRCDBG_TADO_FILL(0x2c),
	TRCDBG_TADO_FILL(0x2d), TRCDBG_TADO_FILL(0x2e), TRCDBG_TADO_FILL(0x2f),
	TRCDBG_TADO_FILL(0x30), TRCDBG_TADO_FILL(0x31), TRCDBG_TADO_FILL(0x32),
	TRCDBG_TADO_FILL(0x33), TRCDBG_TADO_FILL(0x34), TRCDBG_TADO_FILL(0x35),
	TRCDBG_TADO_FILL(0x36), TRCDBG_TADO_FILL(0x37), TRCDBG_TADO_FILL(0x38),
	TRCDBG_TADO_FILL(0x39), TRCDBG_TADO_FILL(0x3a), TRCDBG_TADO_FILL(0x3b),
	TRCDBG_TADO_FILL(0x3c), TRCDBG_TADO_FILL(0x3d), TRCDBG_TADO_FILL(0x3e),
	TRCDBG_TADO_FILL(0x3f), TRCDBG_TADO_FILL(0x40), TRCDBG_TADO_FILL(0x41),
	TRCDBG_TADO_FILL(0x42), TRCDBG_TADO_FILL(0x43), TRCDBG_TADO_FILL(0x44),
	TRCDBG_TADO_FILL(0x45), TRCDBG_TADO_FILL(0x46), TRCDBG_TADO_FILL(0x47),
	TRCDBG_TADO_FILL(0x48), TRCDBG_TADO_FILL(0x49), TRCDBG_TADO_FILL(0x4a),
	TRCDBG_TADO_FILL(0x4b), TRCDBG_TADO_FILL(0x4c), TRCDBG_TADO_FILL(0x4d),
	TRCDBG_TADO_FILL(0x4e), TRCDBG_TADO_FILL(0x4f), TRCDBG_TADO_FILL(0x50),
	TRCDBG_TADO_FILL(0x51), TRCDBG_TADO_FILL(0x52), TRCDBG_TADO_FILL(0x53),
	TRCDBG_TADO_FILL(0x54), TRCDBG_TADO_FILL(0x55), TRCDBG_TADO_FILL(0x56),
	TRCDBG_TADO_FILL(0x57), TRCDBG_TADO_FILL(0x58), TRCDBG_TADO_FILL(0x59),
	TRCDBG_TADO_FILL(0x5a), TRCDBG_TADO_FILL(0x5b), TRCDBG_TADO_FILL(0x5c),
	TRCDBG_TADO_FILL(0x5d), TRCDBG_TADO_FILL(0x5e), TRCDBG_TADO_FILL(0x5f),
	TRCDBG_TADO_FILL(0x60), TRCDBG_TADO_FILL(0x61), TRCDBG_TADO_FILL(0x62),
	TRCDBG_TADO_FILL(0x63), TRCDBG_TADO_FILL(0x64), TRCDBG_TADO_FILL(0x65),
	TRCDBG_TADO_FILL(0x66), TRCDBG_TADO_FILL(0x67), TRCDBG_TADO_FILL(0x68),
	TRCDBG_TADO_FILL(0x69), TRCDBG_TADO_FILL(0x6a), TRCDBG_TADO_FILL(0x6b),
	TRCDBG_TADO_FILL(0x6c), TRCDBG_TADO_FILL(0x6d), TRCDBG_TADO_FILL(0x6e),
	TRCDBG_TADO_FILL(0x6f), TRCDBG_TADO_FILL(0x70), TRCDBG_TADO_FILL(0x71),
	TRCDBG_TADO_FILL(0x72), TRCDBG_TADO_FILL(0x73), TRCDBG_TADO_FILL(0x74),
	TRCDBG_TADO_FILL(0x75), TRCDBG_TADO_FILL(0x76), TRCDBG_TADO_FILL(0x77),
	TRCDBG_TADO_FILL(0x78), TRCDBG_TADO_FILL(0x79), TRCDBG_TADO_FILL(0x7a),
	TRCDBG_TADO_FILL(0x7b), TRCDBG_TADO_FILL(0x7c), TRCDBG_TADO_FILL(0x7d),
	TRCDBG_TADO_FILL(0x7e), TRCDBG_TADO_FILL(0x7f), TRCDBG_TADO_FILL(0x80),
	TRCDBG_TADO_FILL(0x81), TRCDBG_TADO_FILL(0x82), TRCDBG_TADO_FILL(0x83),
	TRCDBG_TADO_FILL(0x84), TRCDBG_TADO_FILL(0x85), TRCDBG_TADO_FILL(0x86),
	TRCDBG_TADO_FILL(0x87), TRCDBG_TADO_FILL(0x88), TRCDBG_TADO_FILL(0x89),
	TRCDBG_TADO_FILL(0x8a), TRCDBG_TADO_FILL(0x8b), TRCDBG_TADO_FILL(0x8c),
	TRCDBG_TADO_FILL(0x8d), TRCDBG_TADO_FILL(0x8e), TRCDBG_TADO_FILL(0x8f),
	TRCDBG_TADO_FILL(0x90), TRCDBG_TADO_FILL(0x91), TRCDBG_TADO_FILL(0x92),
	TRCDBG_TADO_FILL(0x93), TRCDBG_TADO_FILL(0x94), TRCDBG_TADO_FILL(0x95),
	TRCDBG_TADO_FILL(0x96), TRCDBG_TADO_FILL(0x97), TRCDBG_TADO_FILL(0x98),
	TRCDBG_TADO_FILL(0x99), TRCDBG_TADO_FILL(0x9a), TRCDBG_TADO_FILL(0x9b),
	TRCDBG_TADO_FILL(0x9c), TRCDBG_TADO_FILL(0x9d), TRCDBG_TADO_FILL(0x9e),
	TRCDBG_TADO_FILL(0x9f), TRCDBG_TADO_FILL(0xa0), TRCDBG_TADO_FILL(0xa1),
	TRCDBG_TADO_FILL(0xa2), TRCDBG_TADO_FILL(0xa3), TRCDBG_TADO_FILL(0xa4),
	TRCDBG_TADO_FILL(0xa5), TRCDBG_TADO_FILL(0xa6), TRCDBG_TADO_FILL(0xa7),
	TRCDBG_TADO_FILL(0xa8), TRCDBG_TADO_FILL(0xa9), TRCDBG_TADO_FILL(0xaa),
	TRCDBG_TADO_FILL(0xab), TRCDBG_TADO_FILL(0xac), TRCDBG_TADO_FILL(0xad),
	TRCDBG_TADO_FILL(0xae), TRCDBG_TADO_FILL(0xaf), TRCDBG_TADO_FILL(0xb0),
	TRCDBG_TADO_FILL(0xb1), TRCDBG_TADO_FILL(0xb2), TRCDBG_TADO_FILL(0xb3),
	TRCDBG_TADO_FILL(0xb4), TRCDBG_TADO_FILL(0xb5), TRCDBG_TADO_FILL(0xb6),
	TRCDBG_TADO_FILL(0xb7), TRCDBG_TADO_FILL(0xb8), TRCDBG_TADO_FILL(0xb9),
	TRCDBG_TADO_FILL(0xba), TRCDBG_TADO_FILL(0xbb), TRCDBG_TADO_FILL(0xbc),
	TRCDBG_TADO_FILL(0xbd), TRCDBG_TADO_FILL(0xbe), TRCDBG_TADO_FILL(0xbf),
	TRCDBG_TADO_FILL(0xc0), TRCDBG_TADO_FILL(0xc1), TRCDBG_TADO_FILL(0xc2),
	TRCDBG_TADO_FILL(0xc3), TRCDBG_TADO_FILL(0xc4), TRCDBG_TADO_FILL(0xc5),
	TRCDBG_TADO_FILL(0xc6), TRCDBG_TADO_FILL(0xc7), TRCDBG_TADO_FILL(0xc8),
	TRCDBG_TADO_FILL(0xc9), TRCDBG_TADO_FILL(0xca), TRCDBG_TADO_FILL(0xcb),
	TRCDBG_TADO_FILL(0xcc), TRCDBG_TADO_FILL(0xcd), TRCDBG_TADO_FILL(0xce),
	TRCDBG_TADO_FILL(0xcf), TRCDBG_TADO_FILL(0xd0), TRCDBG_TADO_FILL(0xd1),
	TRCDBG_TADO_FILL(0xd2), TRCDBG_TADO_FILL(0xd3), TRCDBG_TADO_FILL(0xd4),
	TRCDBG_TADO_FILL(0xd5), TRCDBG_TADO_FILL(0xd6), TRCDBG_TADO_FILL(0xd7),
	TRCDBG_TADO_FILL(0xd8), TRCDBG_TADO_FILL(0xd9), TRCDBG_TADO_FILL(0xda),
	TRCDBG_TADO_FILL(0xdb), TRCDBG_TADO_FILL(0xdc), TRCDBG_TADO_FILL(0xdd),
	TRCDBG_TADO_FILL(0xde), TRCDBG_TADO_FILL(0xdf), TRCDBG_TADO_FILL(0xe0),
	TRCDBG_TADO_FILL(0xe1), TRCDBG_TADO_FILL(0xe2), TRCDBG_TADO_FILL(0xe3),
	TRCDBG_TADO_FILL(0xe4), TRCDBG_TADO_FILL(0xe5), TRCDBG_TADO_FILL(0xe6),
	TRCDBG_TADO_FILL(0xe7), TRCDBG_TADO_FILL(0xe8), TRCDBG_TADO_FILL(0xe9),
	TRCDBG_TADO_FILL(0xea), TRCDBG_TADO_FILL(0xeb), TRCDBG_TADO_FILL(0xec),
	TRCDBG_TADO_FILL(0xed), TRCDBG_TADO_FILL(0xee), TRCDBG_TADO_FILL(0xef),
	TRCDBG_TADO_FILL(0xf0), TRCDBG_TADO_FILL(0xf1), TRCDBG_TADO_FILL(0xf2),
	TRCDBG_TADO_FILL(0xf3), TRCDBG_TADO_FILL(0xf4), TRCDBG_TADO_FILL(0xf5),
	TRCDBG_TADO_FILL(0xf6), TRCDBG_TADO_FILL(0xf7), TRCDBG_TADO_FILL(0xf8),
	TRCDBG_TADO_FILL(0xf9), TRCDBG_TADO_FILL(0xfa), TRCDBG_TADO_FILL(0xfb),
	TRCDBG_TADO_FILL(0xfc), TRCDBG_TADO_FILL(0xfd), TRCDBG_TADO_FILL(0xfe),
	TRCDBG_TADO_FILL(0xff)
};

int tado_alloc_channel_id(void)
{
	unsigned long flags;
	int	 ch_id;
	DEFINE_SPINLOCK(chid_lock);

	spin_lock_irqsave(&chid_lock, flags);
	TADI_DBG("Alloc channel %d: Free:%d\n",
		tado_free_ch_id_root, tado_free_ch_id_count);
	/* if still free IDs available return an unused ID */
	if (tado_free_ch_id_count) {
		tado_free_ch_id_count--;
		ch_id = tado_free_ch_id_root;
		tado_free_ch_id_root = tado_ch_id_array[ch_id];
		tado_ch_id_array[ch_id] = ch_id; /* mark ch_id as 'in use' */
	/* else, if no more IDs available return general/error ID  */
	} else
		ch_id = TADO_CHID_OVERLOAD;

	spin_unlock_irqrestore(&chid_lock, flags);

	return ch_id;
}

void tado_free_channel_id(int ch_id)
{
	unsigned long flags;
	DEFINE_SPINLOCK(chid_lock);
	TADI_DBG("Release channel %d:\n", ch_id);
	/* if returned ID is general purpose (2..237) (only managed here) */
	if ((ch_id >= TADO_CHID_GENERAL_PURPOSE_START) &&
		(ch_id <= TADO_CHID_GENERAL_PURPOSE_END)) {
		spin_lock_irqsave(&chid_lock, flags); /*Enter CS */

		/* if ch_id is really 'in use' */
	    if ((tado_ch_id_array[ch_id] == 0) ||
				(tado_ch_id_array[ch_id] == ch_id)) {
			/* give to chain as last free */
			tado_ch_id_array[tado_free_ch_id_last] =
				(unsigned char)ch_id;
			/* set this as last free */
			tado_free_ch_id_last = (unsigned char)ch_id;
			/* mark ch_id as 'free' */
			tado_ch_id_array[ch_id]	= TADO_CHID_UNDEFINED_CHANNEL;
			tado_free_ch_id_count++;
		}
		spin_unlock_irqrestore(&chid_lock, flags); /*Leave CS */
	}
}

void tadi_write_end_fifo(void *handle, void *ptr, int count)
{
	int i, remaining_len;
	char *last_data;
	unsigned int ch_id;

	for (i = 0; i < (count-1)/4; i++)
		FIFO_W32(handle, ((unsigned int *)ptr)[i]);

	last_data = (char *)&((unsigned int *)ptr)[i];
	remaining_len = count - i*sizeof(int);
	switch (remaining_len) {
	case 4:
		FIFO_W32((char *)handle+FIFO_LAST_DATA_OFFSET,
				*(unsigned int *)last_data);
		break;
	case 3:
		FIFO_W16(handle, *(unsigned short *)last_data);
		last_data +=  2;
		FIFO_W8((char *)handle+FIFO_LAST_DATA_OFFSET,
				*(unsigned char *)last_data);
		break;
	case 2:
		FIFO_W16((char *)handle+FIFO_LAST_DATA_OFFSET,
				*(unsigned short *)last_data);
		break;
	case 1:
		FIFO_W8((char *)handle+FIFO_LAST_DATA_OFFSET,
				*(unsigned char *)last_data);
		break;
	default:
		TADI_DBG("Error FIFO End length");
		FIFO_W8((char *)handle+FIFO_LAST_DATA_OFFSET, 0);
		break;
	}
	/*free handle */
	ch_id = TADO_HANDLE_TO_CHID(handle);
	tado_free_channel_id(ch_id);
}

void tadi_write_cont_fifo(void *handle, void *ptr, int count)
{
	int i;
	int remaining_len;
	char *last_data;

	for (i = 0; i < count/4; i++)
		FIFO_W32(handle, ((unsigned int *)ptr)[i]);

	last_data = (char *)&((unsigned int *)ptr)[i];
	remaining_len = count - i*sizeof(int);
	switch (remaining_len) {
	case 3:
		FIFO_W16(handle, *(unsigned short *)last_data);
		last_data += 2;
		FIFO_W8(handle, *(unsigned char *)last_data);
		break;
	case 2:
		FIFO_W16(handle, *(unsigned short *)last_data);
		break;
	case 1:
		FIFO_W8(handle, *(unsigned char *)last_data);
		break;
	default:
		break;
	}
}

void *tadi_write_first_fifo(unsigned char mt)
{
	void *handle;
	int ch_id;
	unsigned char ftsc_value = 0; /*TODO: FTS support?*/

	ch_id = tado_alloc_channel_id();
	handle = (void *)TADO_FIFO_GET_ADDR(ch_id);
	TADI_DBG("Open channel %d with MT: %d\n", ch_id, mt);

	FIFO_W16(handle, mt|(ftsc_value << 8));
	return handle;
}

void *trc_tadi_open(unsigned char mt)
{
	return tadi_write_first_fifo(mt);
}
EXPORT_SYMBOL(trc_tadi_open);

void trc_tadi_write(void *handle, void *ptr, int count)
{
	tadi_write_cont_fifo(handle, ptr, count);
}
EXPORT_SYMBOL(trc_tadi_write);

void trc_tadi_close(void *handle)
{
	tadi_write_end_fifo(handle, "\0", 1);
}
EXPORT_SYMBOL(trc_tadi_close);

static ssize_t tadi_read(struct file *file_ptr, char __user *user_buffer,
		size_t count, loff_t *position)
{
	unsigned ret = 0;
	#define TADI_ERROR_MSG "Error: read op from tadi not allowed\n"
	TADI_DBG("read at offset = %i, bytes count = %u",
			(int)*position,
			(unsigned int)count);

	ret = copy_to_user(user_buffer, TADI_ERROR_MSG, sizeof(TADI_ERROR_MSG));
	if (ret)
		pr_err("%s: %d have not been copied by copy_from_user\n",
				__func__, ret);
	return count;
}

static ssize_t tadi_write(struct file *p_file, const char __user *user_buffer,
		size_t count, loff_t *position)
{
	struct s_tadi_priv *p_tadi_priv;
	unsigned char *buffer;
	unsigned int ret = 0;

	p_tadi_priv = (struct s_tadi_priv *)p_file->private_data;
	buffer = kmalloc(count, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	ret = copy_from_user(buffer, user_buffer, count);
	if (ret)
		pr_err("%s: %d have not been copied by copy_from_user\n",
				__func__, ret);
	/*check if previous ioct for setting mt has been done */
	if (p_tadi_priv->mt == INVALID_MSG_TYPE)
		TADI_DBG("Error - Invalid message type\n");

	/*write a single packet */
	p_tadi_priv->handler = tadi_write_first_fifo(p_tadi_priv->mt);
	tadi_write_end_fifo(p_tadi_priv->handler, buffer, count);

	kfree(buffer);
	return count;
}

int tadi_open(struct inode *p_inode, struct file *p_file)
{
	/*pr_debug("Open fd: %x\n", (unsigned int) p_file);*/
	/* allocate memory for .private_data */
	p_file->private_data = kmalloc(sizeof(struct s_tadi_priv), GFP_KERNEL);
	if (!p_file->private_data)
		return -ENOMEM;

	((struct s_tadi_priv *)p_file->private_data)->mt = INVALID_MSG_TYPE;
	/*((struct s_tadi_priv *)p_file->private_data)->cmd = INVALID_CMD; */
	((struct s_tadi_priv *)p_file->private_data)->handler = NULL;

	/* allocate channel ID */

	return 0;
}

int tadi_release(struct inode *p_inode, struct file *p_file)
{
	/*pr_debug("Close fd: %x\n", (unsigned int) p_file);*/
	/* TODO: If the chid is not closed, close-it with 0 */
	kfree(p_file->private_data);
	return 0;
}

static long tadi_ioctl(struct file *p_file, unsigned int cmnd,
		unsigned long param)
{
	struct s_tadi_multi_write tadi_multiwrite;
	struct s_tadi_priv   *p_tadi_priv;
	unsigned char *buffer;
	unsigned ret = 0;

	p_tadi_priv = (struct s_tadi_priv *)p_file->private_data;
	TADI_DBG("IOCTL FD:%p CMD:%d, MT/param:%lx\n",
			p_file, cmnd, param);
	switch (cmnd) {
	case CMD_SET_MT:
		p_tadi_priv->mt = param;
		break;
	case CMD_WRITE_MUL_FIRST:
		if (p_tadi_priv->mt == INVALID_MSG_TYPE) {
			TADI_DBG(" Error - MT not set.\n");
			break;
		}
		ret = copy_from_user(
			&tadi_multiwrite,
			(const void *)param,
			sizeof(struct s_tadi_multi_write));
		tadi_multiwrite.handle = tadi_write_first_fifo(p_tadi_priv->mt);
		ret += copy_to_user(
			&((struct s_tadi_multi_write *)param)->handle,
			&tadi_multiwrite.handle,
			sizeof(tadi_multiwrite.handle));
		buffer = kmalloc(tadi_multiwrite.len, GFP_KERNEL);
		if (!buffer)
			return -ENOMEM;
		ret += copy_from_user(
			buffer,
			tadi_multiwrite.buff,
			tadi_multiwrite.len);
		tadi_write_cont_fifo(
			tadi_multiwrite.handle,
			buffer,
			tadi_multiwrite.len);
		if (ret)
			pr_err("%s: %d have not been copied by copy_from_user\n",
					__func__, ret);
		kfree(buffer);
		break;
	case CMD_WRITE_MUL_CONT:
		ret = copy_from_user(
			&tadi_multiwrite,
			(const void *)param,
			sizeof(struct s_tadi_multi_write));
		if (tadi_multiwrite.handle == NULL) {
			TADI_DBG("Error - handler null.\n");
			break;
		}

		buffer = kmalloc(tadi_multiwrite.len, GFP_KERNEL);
		if (!buffer)
			return -ENOMEM;
		ret += copy_from_user(buffer,
				tadi_multiwrite.buff, tadi_multiwrite.len);
		tadi_write_cont_fifo(tadi_multiwrite.handle,
						buffer,
						tadi_multiwrite.len);
		if (ret)
			pr_err("%s: %d have not been copied by copy_from_user\n",
					__func__, ret);
		kfree(buffer);
		break;
	case CMD_WRITE_MUL_LAST:
		ret = copy_from_user(
			&tadi_multiwrite,
			(const void *)param,
			sizeof(struct s_tadi_multi_write));
		if (tadi_multiwrite.handle == NULL)	{
			TADI_DBG(" Error - handler null.\n");
			break;
		}
		buffer = kmalloc(tadi_multiwrite.len, GFP_KERNEL);
		ret += copy_from_user(buffer,
				tadi_multiwrite.buff, tadi_multiwrite.len);
		tadi_write_end_fifo(tadi_multiwrite.handle,
						buffer,
						tadi_multiwrite.len);
		if (ret)
			pr_err("%s: %d have not been copied by copy_from_user\n",
					__func__, ret);
		kfree(buffer);
		break;
	case CMD_GET_REV:
		ret = copy_to_user((unsigned int *)param, &rev, sizeof(int));
		if (ret)
			pr_err("%s: %d have not been copied by copy_from_user\n",
					__func__, ret);
		break;
	default:
		TADI_DBG(" Error - Invalid command\n");
		break;
	}
	return 0;
}

ssize_t tadi_aio_write(struct kiocb *iocb, const struct iovec *iov,
		unsigned long list_len, loff_t offset)
{
	struct file *p_file;
	struct s_tadi_priv   *p_tadi_priv;
	unsigned char *buffer;
	int i;
	unsigned ret = 0;

	p_file = iocb->ki_filp;
	p_tadi_priv = (struct s_tadi_priv *)p_file->private_data;

	if (p_tadi_priv->mt == INVALID_MSG_TYPE) {
		TADI_DBG(" Error - MT not set.\n");
		return -1;
	}

	p_tadi_priv->handler = tadi_write_first_fifo(p_tadi_priv->mt);

	i = 0;
	do {
		if (iov[i].iov_base == NULL) {
			TADI_DBG(" Error writev null buffer %d, size %d\n",
					i, (int)iov[i].iov_len);
			/*release the channel */
			tadi_write_end_fifo(p_tadi_priv->handler, "\0", 1);
			return -1;
		}
		buffer = kmalloc(iov[i].iov_len, GFP_KERNEL);
		if (!buffer)
			return -ENOMEM;
		ret = copy_from_user(buffer, iov[i].iov_base, iov[i].iov_len);
		if (ret)
			pr_err("%s: %d have not been copied by copy_from_user\n",
					__func__, ret);

		if (i < list_len - 1) {
			tadi_write_cont_fifo(
				p_tadi_priv->handler,
				buffer,
				iov[i].iov_len);
		} else {
		    tadi_write_end_fifo(
				p_tadi_priv->handler,
				buffer,
				iov[i].iov_len);
			kfree(buffer);
			break;
		}
		kfree(buffer);
		i++;
	} while (i < TADI_MAX_LIST_SIZE);

	if ((i == 0) || (i == TADI_MAX_LIST_SIZE)) {
		TADI_DBG("Error Invalid list!\n");
		return -1;
	}
	return 0;
}

static const struct file_operations tadi_fops = {
	.owner			= THIS_MODULE,
	.aio_write		= tadi_aio_write,
	.read			= tadi_read,
	.write			= tadi_write,
	.open			= tadi_open,
	.release		= tadi_release,
	.unlocked_ioctl	= tadi_ioctl
};

/**
 * td_cons_write()-  Write to the console that has been acquired.
 *
 * @c:   Not used in this implementaiton.
 * @buf: Data to be written.
 * @len: Length of buf.
 */
static void td_cons_write(struct console *c, const char *buf,
		unsigned len)
{
	void  *handler;
	int   mt = 0x10;

	/* write a single packet */
	handler = tadi_write_first_fifo(mt);
	tadi_write_end_fifo(handler, (void *)buf, (int)len);
}

static struct console td_cons = {
	.name		= "tdcons",
	.write		= td_cons_write,
	.flags		= CON_PRINTBUFFER,
	/*.flags		= CON_PRINTBUFFER | CON_ENABLED,*/
	.index		= 0
};

static const struct of_device_id xgold_tadi_of_match[] = {
	{
		.compatible = "intel,tadi",
	},
	{}
};

MODULE_DEVICE_TABLE(of, xgold_tadi_of_match);

static struct platform_driver tadi_driver = {
	.driver = {
		   .name = (char *)TADI_MODULE_NAME,
		   .of_match_table = of_match_ptr(xgold_tadi_of_match),
		   },
	.probe = tadi_driver_probe,
	.remove = tadi_driver_remove,
};

static int tadi_init(void)
{
	struct device *dev_tadi;
	int result = 0;
	TADI_DBG("device registration\n");

	result = register_chrdev(0, TADI_MODULE_NAME, &tadi_fops);
	if (result < 0) {
		TADI_DBG("can\'t register char dev errorcode=%i", result);
		return result;
	}
	tadi_major_number = result;

	class_tadi = class_create(THIS_MODULE, TADI_MODULE_NAME);

	if (IS_ERR(class_tadi)) {
		unregister_chrdev(tadi_major_number, TADI_MODULE_NAME);
		TADI_DBG("error creating class\n");
		return 0;
	}

	dev_tadi = device_create(class_tadi, NULL,
					MKDEV(tadi_major_number, 0),
					NULL, TADI_MODULE_NAME);
	if (IS_ERR(dev_tadi)) {
		class_destroy(class_tadi);
		unregister_chrdev(tadi_major_number, TADI_MODULE_NAME);
		TADI_DBG("Error creating device\n");
		return 0;
	}

	TADI_DBG("registered char device, major number=%i\n"
			 , tadi_major_number);

	result = platform_driver_register(&tadi_driver);
	if (result < 0) {
		pr_err("%s retval=%d\n", __func__, result);
		return result;
	}

	return 0;
}

static void tadi_exit(void)
{
	unregister_console(&td_cons);

	device_destroy(class_tadi, MKDEV(tadi_major_number, 0));
	class_destroy(class_tadi);
	iounmap((void __iomem *)fifo_start);
	unregister_chrdev(tadi_major_number, TADI_MODULE_NAME);

	TADI_DBG("device unregistration\n");
}

module_init(tadi_init);
module_exit(tadi_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Tadi character driver");

static int tadi_driver_probe(struct platform_device *_dev)
{
	struct resource *mem;
	int size;
	mem = platform_get_resource_byname(_dev,
			IORESOURCE_MEM, "trace-port");
	if (!mem) {
		TADI_DBG(" no trace port resource?\n");
		return 0;
	}
	size = (mem->end - mem->start) + 1;
	fifo_start = (uintptr_t)ioremap(mem->start, size);
	pr_info("%s: trace port:%pR - ioremap:%lx\n",
			__func__, mem, fifo_start);
	if (!fifo_start) {
		pr_err("%s: unable to remap memory region\n", __func__);
		release_mem_region(mem->start, size);
		return 0;
	}
	pr_info("%s: Register %s console\n", __func__, td_cons.name);
	register_console(&td_cons);
	return 0;
}

static int tadi_driver_remove(struct platform_device *_dev)
{
	iounmap((void __iomem *)fifo_start);
	return 0;
}
