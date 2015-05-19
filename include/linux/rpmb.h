/*
 * Copyright (C) 2015 Intel Corp. All rights reserved
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 */
#ifndef __RPMB_H__
#define __RPMB_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/kref.h>
#include <linux/genhd.h>
#include <linux/workqueue.h>


struct rpmb_frame {
	u8     stuff[196];
	u8     key_mac[32];
	u8     data[256];
	u8     nonce[16];
	__be32 write_counter;
	__be16 addr;
	__be16 block_count;
	__be16 result;
	__be16 req_resp;
} __packed;

#define RPMB_PROGRAM_KEY       0x1    /* Program RPMB Authentication Key */
#define RPMB_GET_WRITE_COUNTER 0x2    /* Read RPMB write counter */
#define RPMB_WRITE_DATA        0x3    /* Write data to RPMB partition */
#define RPMB_READ_DATA         0x4    /* Read data from RPMB partition */
#define RPMB_RESULT_READ       0x5    /* Read result request  (Internal) */

#define RPMB_RESPONSE_MSK      0x80   /* response BIT */


struct rpmb_data {
	struct rpmb_frame *in_frames;
	struct rpmb_frame *out_frames;
	u32 in_frames_cnt;
	u32 out_frames_cnt;
	u16 req_type;
	u16 block_count;
};

struct rpmb_ops {
	int (*send_rpmb_req)(struct gendisk *disk, struct rpmb_data *req);
};

/*
 * rpmb_dev: the device which can support RPMB partition
 */
struct rpmb_dev {
	struct list_head dev_list;
	struct kref refcnt;
	struct device *parent;
	struct gendisk *disk;
	struct rpmb_ops *ops;
	const char *name;
	struct mutex lock;
	struct work_struct work;
};


/* Events from the rpmb partition */
#define RPMB_PART_ADD         0x0001
#define RPMB_PART_REMOVE      0x0002
#define RPMB_PART_SUSPEND     0x0003
#define RPMB_PART_RESUME      0x0004

#ifdef CONFIG_RPMB
extern struct rpmb_dev *rpmb_dev_get(struct rpmb_dev *dev);
extern void rpmb_dev_put(struct rpmb_dev *dev);
extern struct rpmb_dev *rpmb_dev_find_by_disk(struct gendisk *disk);
extern struct rpmb_dev *rpmb_dev_get_default(void);
extern struct rpmb_dev *rpmb_dev_register(struct gendisk *disk,
					  struct rpmb_ops *ops);
extern int rpmb_dev_unregister(struct gendisk *disk);
extern int rpmb_dev_suspend(struct gendisk *disk);
extern int rpmb_dev_resume(struct gendisk *disk);
extern int rpmb_send_req(struct rpmb_dev *dev, struct rpmb_data *data);

extern int rpmb_register_notify(struct notifier_block *nb);
extern int rpmb_unregister_notify(struct notifier_block *nb);

#else
static inline struct rpmb_dev *rpmb_dev_get(struct rpmb_dev *dev)
{
	return NULL;
}
static inline void rpmb_dev_put(struct rpmb_dev *dev) { }
static inline struct rpmb_dev *
rpmb_dev_find_by_disk(struct gendisk *disk)
{
	return NULL;
}
static inline struct rpmb_dev *rpmb_dev_get_default(void)
{
	return NULL;
}
static inline struct rpmb_dev *
rpmb_dev_register(struct gendisk *disk, struct rpmb_ops *ops)
{
	return NULL;
}
static inline int rpmb_dev_unregister(struct gendisk *disk)
{
	return 0;
}
static inline int rpmb_dev_suspend(struct gendisk *disk)
{
	return 0;
}
static inline int rpmb_dev_resume(struct gendisk *disk)
{
	return 0;
}
static inline int rpmb_send_req(struct rpmb_dev *dev, struct rpmb_data *data)
{
	return 0;
}
static inline int rpmb_register_notify(struct notifier_block *nb)
{
	return 0;
}
static inline int rpmb_unregister_notify(struct notifier_block *nb)
{
	return 0;
}
#endif /* CONFIG_RPMB */

#endif /* __RPMB_H__ */
