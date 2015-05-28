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

/*
 * NOTES:
 * 1) This source file is included in guests including Linux and purposely
 * kept in line with Linux kernel coding guidelines for ease of portability and
 * maintainability. See linux/Documentation/CodingStyle for coding guidelines.
 * Please check all further changes are compliant with Linux coding standard
 * and clear all commands reported by Linux checkpatch.pl
 * Use command: linux/scripts/checkpatch.pl -f <filename>
 * Clear ALL warnings and errors reported.
 *
 */

#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/smp.h>
#include <linux/sysprofile.h>
#include <linux/memblock.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <sofia/mv_hypercalls.h>
#include <sofia/mv_gal.h>
#include <sofia/mv_ipc.h>

struct mbox_directory *mbox_dir;
extern struct irq_domain *hirq_domain;

#define MBOX_IRQ_HANDLER_NAME_SIZE (MAX_MBOX_NAME_SIZE + 32)
#define HIRQ_OFFSET 512

struct mbox_event_info {
	struct mbox_db_guest_entry *info;
	uint32_t event_id;
};

struct mbox_database_entry {
	void *cookie;
	struct mbox_ops *ops;
	struct mbox_db_guest_entry *info;
	struct mbox_event_info *event_info_array;
};

struct mbox_database_entry *mbox_database;

struct mbox_dir_entry
*mbox_lookup_directory_entry(char *name)
{
	int i;

	for (i = 0; i < mbox_dir->num_of_entries; i++) {
		struct mbox_dir_entry *entry = &mbox_dir->entry[i];
		if (!strncmp(entry->name, name, MAX_MBOX_NAME_SIZE))
			return entry;
	}

	return NULL;
}

struct mbox_instance_dir_entry
*mbox_lookup_instance_directory_entry(struct mbox_instance_directory
				      *instance_dir, char *instance_name)
{
	int i;

	for (i = 0; i < instance_dir->num_of_instances; i++) {
		struct mbox_instance_dir_entry *entry = &instance_dir->entry[i];
		if (!strncmp(entry->name, instance_name,
			     MAX_MBOX_INSTANCE_NAME_SIZE))
			return entry;
	}

	return NULL;
}

void mbox_get_shmem_for_all_instances(char *name,
		uint32_t *vaddr,
		uint32_t *size)
{
	int i;
	for (i = 0; i < mbox_dir->num_of_entries; i++) {
		struct mbox_dir_entry *entry = &mbox_dir->entry[i];
		if (!strncmp(entry->name, name, MAX_MBOX_NAME_SIZE)) {
			*vaddr = (uint32_t) mv_gal_ptov((vmm_paddr_t)
			    (entry->addr_of_shmem_for_all_instances));
			*size = entry->size_of_shmem_for_all_instances;
			return;
		}
	}
}

uint32_t mbox_lookup_id_by_name(char *name,
				char *instance_name,
				uint32_t *mbox_id, uint32_t *instance_number)
{
	struct mbox_dir_entry *entry = mbox_lookup_directory_entry(name);

	if (entry) {
		struct mbox_instance_dir_entry *instance_dir_entry = NULL;
		struct mbox_instance_directory *instance_directory = NULL;

		instance_directory = (struct mbox_instance_directory *)
			  mv_gal_ptov((vmm_paddr_t) entry->p_instance_dir);
		if (!instance_directory) {
			pr_err("Could not get virt address\n");
			return MBOX_ERROR;
		}

		instance_dir_entry = mbox_lookup_instance_directory_entry(
				instance_directory, instance_name);
		if (instance_dir_entry) {
			*mbox_id = entry->id;
			*instance_number = instance_dir_entry->id;
			return MBOX_SUCCESS;
		}
	}

	return MBOX_ERROR;
}

irqreturn_t mbox_connect_handler(int irq, void *data)
{
	struct mbox_db_guest_entry *info = (struct mbox_db_guest_entry *)data;
	struct mbox_database_entry *db_entry =
				&mbox_database[info->token & 0xFF];
	db_entry->ops->on_connect(info->token, db_entry->cookie);
	return IRQ_HANDLED;
}

irqreturn_t mbox_disconnect_handler(int irq, void *data)
{
	struct mbox_db_guest_entry *info = (struct mbox_db_guest_entry *)data;
	struct mbox_database_entry *db_entry =
				&mbox_database[info->token & 0xFF];
	db_entry->ops->on_disconnect(info->token, db_entry->cookie);
	return IRQ_HANDLED;
}

irqreturn_t mbox_event_handler(int irq, void *data)
{
	struct mbox_event_info *event_info = (struct mbox_event_info *)data;
	struct mbox_db_guest_entry *info = event_info->info;
	struct mbox_database_entry *db_entry =
				&mbox_database[info->token & 0xFF];
	db_entry->ops->on_event(info->token,
				event_info->event_id, db_entry->cookie);
	return IRQ_HANDLED;
}

uint32_t mv_ipc_init(void)
{
	mbox_dir =
		(struct mbox_directory *)mv_gal_ptov(mv_mbox_get_directory());
	if (!mbox_dir) {
		pr_err("Could not get mbox virt address\n");
		return -1;
	}
	mbox_database =
		kmalloc(mbox_dir->num_of_instances *
			sizeof(struct mbox_database_entry), GFP_KERNEL);
	return 0;
}

uint32_t mv_ipc_mbox_get_info(char *name,
			      char *instance_name,
			      struct mbox_ops *ops,
			      unsigned char **shared_mem_start,
			      unsigned int *shared_mem_size,
			      char **cmdline, void *cookie)
{
	uint32_t pinfo, token, mid, instance_no, res;
	struct mbox_database_entry *db_entry;
	int irq_on_connect, irq_on_disconnect, irq_event;
	char *connect_name = NULL, *disc_name = NULL;
	char **event_names = NULL;
	struct mbox_db_guest_entry *info;
	int i, req = 0;

	res = mbox_lookup_id_by_name(name, instance_name, &mid, &instance_no);
	if (res == MBOX_ERROR)
		return MBOX_INIT_ERR;

	token = mv_mbox_get_info(mid, instance_no, &pinfo);
	if (token == MBOX_INIT_ERR)
		return MBOX_INIT_ERR;

	info = (struct mbox_db_guest_entry *)mv_gal_ptov(pinfo);
	if (!info) {
		pr_err("Could not get virt address\n");
		goto exit1;
	}

	*cmdline = info->cmd_line;
	*shared_mem_start = info->shared_mem;
	*shared_mem_size = info->size_of_shared_memory;

	irq_on_connect = irq_create_mapping(hirq_domain,
					    info->on_connect_hirq -
					    HIRQ_OFFSET);
	connect_name = kmalloc(MBOX_IRQ_HANDLER_NAME_SIZE, GFP_KERNEL);
	if (!connect_name) {
		pr_err("Could not allocate connect name\n");
		goto exit1;
	}
	snprintf(connect_name, MBOX_IRQ_HANDLER_NAME_SIZE,
		"mb-%s-%s-conn", name, instance_name);
	req = request_irq(irq_on_connect, mbox_connect_handler,
		    IRQF_NO_SUSPEND, connect_name, info);
	if (req) {
		pr_err("Could not request_irq (%s - %d)\n", connect_name,
				irq_on_connect);
		goto exit2;
	}

	irq_on_disconnect = irq_create_mapping(hirq_domain,
					       info->on_disconnect_hirq -
					       HIRQ_OFFSET);
	disc_name = kmalloc(MBOX_IRQ_HANDLER_NAME_SIZE, GFP_KERNEL);
	if (!disc_name) {
		pr_err("Could not allocate disconnect name\n");
		goto exit3;
	}
	snprintf(disc_name, MBOX_IRQ_HANDLER_NAME_SIZE,
		 "mb-%s-%s-disc", name, instance_name);

	req = request_irq(irq_on_disconnect, mbox_disconnect_handler,
		IRQF_NO_SUSPEND, disc_name, info);
	if (req) {
		pr_err("Could not request_irq (%s - %d)\n", disc_name,
				irq_on_disconnect);
		goto exit4;
	}

	db_entry = &mbox_database[token & 0xFF];
	db_entry->info = info;
	db_entry->ops = ops;
	db_entry->cookie = cookie;

	db_entry->event_info_array = kmalloc(sizeof(struct mbox_event_info) *
					     info->num_of_events, GFP_KERNEL);
	if (!db_entry->event_info_array) {
		pr_err("Could not allocate event_info_array\n");
		goto exit5;
	}

	event_names = kmalloc(sizeof(char *) * info->num_of_events, GFP_KERNEL);
	if (!event_names) {
		pr_err("Could not allocate event names array\n");
		goto exit6;
	}

	for (i = 0; i < info->num_of_events; i++) {
		irq_event = irq_create_mapping(hirq_domain,
					       info->first_event_hirq + i -
					       HIRQ_OFFSET);
		db_entry->event_info_array[i].info = info;
		db_entry->event_info_array[i].event_id = i;

		event_names[i] = kmalloc(MBOX_IRQ_HANDLER_NAME_SIZE,
				GFP_KERNEL);
		if (!event_names[i]) {
			pr_err("Could not allocate event name\n");
			goto exit7;
		}
		snprintf(event_names[i], MBOX_IRQ_HANDLER_NAME_SIZE,
			"mb-%s-%s-%d", name, instance_name, i);
		req = request_irq(irq_event, mbox_event_handler,
				IRQF_NO_SUSPEND, event_names[i],
				&(db_entry->event_info_array[i]));
		if (req) {
			pr_err("Could not request_irq (%s - %d)\n",
					event_names[i], irq_event);
			goto exit8;
		}
	}
	kfree(event_names);
	return token;
exit8:
	kfree(event_names[i]);
exit7:
	while (--i >= 0) {
		irq_event = irq_find_mapping(hirq_domain,
					       info->first_event_hirq + i -
					       HIRQ_OFFSET);
		free_irq(irq_event, &(db_entry->event_info_array[i]));
		kfree(event_names[i]);
	}
	kfree(event_names);
exit6:
	kfree(db_entry->event_info_array);
exit5:
	free_irq(irq_on_disconnect, info);
exit4:
	kfree(disc_name);
exit3:
	free_irq(irq_on_connect, info);
exit2:
	kfree(connect_name);
exit1:
	return MBOX_INIT_ERR;
}

uint32_t mv_ipc_mbox_for_all_instances(char *name,
				       void (*on_instance)(char *,
						       uint32_t, uint32_t))
{
	int i;
	struct mbox_instance_directory *instance_dir;
	struct mbox_dir_entry *entry = mbox_lookup_directory_entry(name);

	if (!entry)
		return MBOX_ERROR;

	instance_dir = (struct mbox_instance_directory *)
		       mv_gal_ptov((uint32_t) entry->p_instance_dir);
	if (!instance_dir) {
		pr_err("Could not get virt address\n");
		return MBOX_ERROR;
	}

	for (i = 0; i < instance_dir->num_of_instances; i++) {
		on_instance(instance_dir->entry[i].name, i,
			    instance_dir->num_of_instances);
	}

	return MBOX_SUCCESS;
}

uint32_t mv_ipc_mbox_post(uint32_t token, uint32_t event_id)
{
	struct mbox_database_entry *db_entry;
	db_entry = &mbox_database[token & 0xFF];
	mv_mbox_post(token, db_entry->info->first_event_hirq + event_id);
	return 0;
}

uint32_t mv_ipc_mbox_set_online(uint32_t token)
{
	mv_mbox_set_online(token);
	return 0;
}

uint32_t mv_ipc_mbox_set_offline(uint32_t token)
{
	mv_mbox_set_offline(token);
	return 0;
}
