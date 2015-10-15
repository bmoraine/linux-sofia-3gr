/*
 * Copyright (c) 2015, Intel Corporation
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
 */

#ifndef _VIDT_H_
#define _VIDT_H_

#include <linux/seq_file.h>
#include "vidt_ioctl.h"

int setup_vidt(void);
void restore_os_idt(void);
int map_pid_viewId(void *priv_data, struct entry_pid_viewid view_new);
int unmap_pid_viewId(void *priv_data, struct entry_pid_viewid view_entry);
int clean_ta_view(void *priv_data);
void clean_view_map_list(struct file *file);
int init_view_map_list(struct file *file);

#endif
