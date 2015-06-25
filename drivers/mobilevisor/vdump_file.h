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

#ifndef _VDUMP_FILE_H_
#define _VDUMP_FILE_H_

void vdump_set_linux_config(void);
void vdump_open_coredump(void);
void vdump_close_coredump(void);
void vdump_save_coredump(void *ptr, int num_bytes);
int vdump_get_shmem_config(void);
void vdump_setting_init(void);

#endif /* _VDUMP_H_ */
