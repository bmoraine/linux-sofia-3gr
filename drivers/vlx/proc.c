/*
 ****************************************************************
 *
 *  Component: VLX /proc/nk directory
 *
 *  Copyright (C) 2011-2012, Red Bend Ltd.
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
 *    Vladimir Grouzdev (vladimir.grouzdev@redbend.com)
 *    Adam Mirowski (adam.mirowski@redbend.com)
 *
 ****************************************************************
 */

#include <linux/proc_fs.h>

    static int
proc_nk_module_init (void)
{
    if (!proc_mkdir ("nk", NULL)) {
	printk (KERN_ERR "Error: could not create /proc/nk\n");
	return -EAGAIN;
    }
    return 0;
}

    static void
proc_nk_module_exit (void)
{
    remove_proc_entry ("nk", NULL);
}

arch_initcall(proc_nk_module_init);
module_exit(proc_nk_module_exit);
