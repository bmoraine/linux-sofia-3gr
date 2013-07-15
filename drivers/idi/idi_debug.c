/*
 * Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/device.h>

#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_debug.h>

#define DEBUG_MAX_INDEX  (256)

struct idi_debug_entry {
	enum idi_events evnt;
	enum idi_peripheral_type peripheral;
	u32 chan;
	u32 direction;
	void *data;
	dma_addr_t dma;
	u32 next;
	u32 size;
	u64 timestamp;
};

struct idi_debug_info {
	struct idi_debug_entry debug_array[DEBUG_MAX_INDEX];
	u32 index;
	spinlock_t lock;
	struct dentry *dir;
};

static struct idi_debug_info idi_debug;

static int idi_debug_show(struct seq_file *m, void *p)
{
	int i;
	char ch;

	pr_debug("%s:\n", __func__);

	for (i = 0; i < DEBUG_MAX_INDEX; i++) {

		if (i == idi_debug.index)
			ch = '*';
		else
			ch = ' ';
		seq_printf(m, "%c: %lld %2d %2d %2d %2d %p %x 0x%08x %d\n", ch,
			   idi_debug.debug_array[i].timestamp,
			   idi_debug.debug_array[i].evnt,
			   idi_debug.debug_array[i].peripheral,
			   idi_debug.debug_array[i].chan,
			   idi_debug.debug_array[i].direction,
			   idi_debug.debug_array[i].data,
			   idi_debug.debug_array[i].dma,
			   idi_debug.debug_array[i].next,
			   idi_debug.debug_array[i].size);
	}

	return 0;

}

static int idi_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, idi_debug_show, inode->i_private);
}

static const struct file_operations idi_debug_fops = {
	.open = idi_debug_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init idi_debug_add_ctrl(void)
{
	struct dentry *dir;

	pr_debug("%s:\n", __func__);

	/*
	 * IDI controller
	 */
	dir = debugfs_create_dir("idi_debug", NULL);
	if (IS_ERR(dir))
		return PTR_ERR(dir);

	debugfs_create_file("entries", S_IRUGO, dir, &idi_debug,
			    &idi_debug_fops);

	idi_debug.dir = dir;

	return 0;

}

void __idi_debug_init(void)
{
	idi_debug.dir = NULL;
	idi_debug_add_ctrl();

	spin_lock_init(&idi_debug.lock);
}
EXPORT_SYMBOL_GPL(__idi_debug_init);

void __idi_debug_remove(void)
{
	debugfs_remove_recursive(idi_debug.dir);

}
EXPORT_SYMBOL_GPL(__idi_debug_remove);

void __idi_debug_add_event(enum idi_events event,
			   enum idi_peripheral_type p_type,
			   struct idi_transaction *trans)
{
	struct idi_debug_entry *idi_event;
	unsigned long flags;
	u32 index;

	spin_lock_irqsave(&idi_debug.lock, flags);

	index = idi_debug.index;
	if (index >= DEBUG_MAX_INDEX)
		index = 0;

	idi_event = &idi_debug.debug_array[index];

	idi_event->evnt = event;
	idi_event->peripheral = p_type;

	if (trans) {
		idi_event->chan = trans->channel;

		if (trans->idi_xfer.base) {
			idi_event->data = trans->idi_xfer.cpu_base;
			idi_event->dma = trans->idi_xfer.base;
			idi_event->next = 0;
		} else {
			idi_event->data = trans->idi_xfer.desc;
			idi_event->next = trans->idi_xfer.desc->next;
		}
		idi_event->size = trans->idi_xfer.size;
	} else {
		idi_event->chan = IDI_MAX_CHANNEL;
		idi_event->data = NULL;
		idi_event->dma = 0;
		idi_event->next = 0;
		idi_event->size = 0;
	}

	idi_event->timestamp = get_jiffies_64();

	index++;
	idi_debug.index = index;

	spin_unlock_irqrestore(&idi_debug.lock, flags);

}
EXPORT_SYMBOL_GPL(__idi_debug_add_event);
