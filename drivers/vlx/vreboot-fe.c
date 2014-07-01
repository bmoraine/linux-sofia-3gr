/*
 ****************************************************************
 *
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
 *  Copyright (C) 2011, Red Bend Ltd.
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
 *
 ****************************************************************
 */

#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/reboot.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/nk_sofia_bridge.h>
#else
#include <nk/nkern.h>
#include <nk/nkdev.h>
#include <nk/nk.h>
#endif

typedef enum
{
  FALSE,
  TRUE
} BOOL;


#define TRACE(x...)   printk (KERN_NOTICE "VREBOOT-FE: " x)
#define ETRACE(x...)  printk (KERN_ERR    "VREBOOT-FE: Error: " x)

static NkDevVlink* vreboot_vlink_server;
static NkXIrq vreboot_xirq_server;        /*xirq from vreboot be to vreboot fe*/

static NkDevVlink* vreboot_vlink_client;
static NkXIrq vreboot_xirq_client;        /*xirq from vreboot fe to vreboot be*/

static BOOL vreboot_fe_init = FALSE;
static BOOL is_linux_reboot = TRUE;
static BOOL is_blocking = FALSE;

static DECLARE_WAIT_QUEUE_HEAD(vreboot_wait_queue);


static void vreboot_xirq_hdl(void* dev, NkXIrq xirq)
{
  (void) dev;
  (void) xirq;
  if(is_blocking)
  {
    TRACE("Received BE response\n");
    is_blocking = FALSE;
#if !defined (VREBOOT_HOSTTEST_ENABLE)
    wake_up(&vreboot_wait_queue);
#endif
  }
  else
  {
    TRACE("This is a MEX Reboot\n");
    is_linux_reboot = FALSE;
#if !defined (VREBOOT_HOSTTEST_ENABLE)
    kernel_restart(NULL);
#endif
  }
}

void vreboot_inform_be_and_block_until_be_responds(void)
{
  if(is_linux_reboot)
  {
    TRACE("This is a Linux Reboot\n");
    is_blocking = TRUE;
#if !defined (VREBOOT_HOSTTEST_ENABLE)
    nkops.nk_xirq_trigger(vreboot_xirq_client, vreboot_vlink_client->s_id);
#endif
    TRACE("Waiting for BE response\n");
#if !defined (VREBOOT_HOSTTEST_ENABLE)
    wait_event_interruptible(vreboot_wait_queue, !is_blocking);
#endif
    TRACE("BE responded - Linux reboot unblocked\n");
  }
}

/*************************************************
  * init vreboot XIRQ
  *************************************************/
static BOOL vreboot_xirq_init(char* name, NkDevVlink** vlink, int vlink_id, BOOL is_server, NkXIrq* xirq)
{
  NkPhAddr plink;
  BOOL found = FALSE;
  plink = 0;

  while((plink = nkops.nk_vlink_lookup(name, plink)) != 0)
  {
    *vlink = nkops.nk_ptov(plink);
    if((*vlink)->link == vlink_id)
    {
      if(is_server && (*vlink)->s_id == nkops.nk_id_get())
      {
        found = TRUE;
        break;
      }
      else if(!is_server && (*vlink)->c_id == nkops.nk_id_get())
      {
        found = TRUE;
        break;
      }
      else
      {
        continue;
      }
    }
  }

  if(!found)
  {
    ETRACE("vlink lookup fail\n");
    return FALSE;
  }

  *xirq = nkops.nk_pxirq_alloc(plink, 1, (*vlink)->s_id, 1);

  if(*xirq == 0)
  {
    ETRACE("xirq alloc fail\n");
    return FALSE;
  }

  return TRUE;

}

#if defined(VREBOOT_HOSTTEST_ENABLE)
extern void hosttest_add_test(char* test_name, void (*test_func)(void));
static void vreboot_exit(void);

void vreboot_hosttest_func(void)
{
	is_blocking = TRUE;
	vreboot_xirq_hdl(NULL,NULL);
	is_blocking = FALSE;
	vreboot_xirq_hdl(NULL,NULL);

	is_linux_reboot = TRUE;
	vreboot_inform_be_and_block_until_be_responds();

	is_linux_reboot = FALSE;
	vreboot_inform_be_and_block_until_be_responds();

	vreboot_xirq_init("dummy1", &vreboot_vlink_server, 0, TRUE, &vreboot_xirq_server);

	vreboot_exit();
  printk("test vreboot done\n");
}
#endif

static int __init vreboot_init(void)
{
  if(!vreboot_xirq_init("vreboot", &vreboot_vlink_server, 0, TRUE, &vreboot_xirq_server))
  {
    ETRACE("server xirq create fail\n");
    return 0;
  }

  nkops.nk_xirq_attach(vreboot_xirq_server, vreboot_xirq_hdl, 0);

  if(!vreboot_xirq_init("vreboot", &vreboot_vlink_client, 0, FALSE, &vreboot_xirq_client))
  {
    ETRACE("client xirq create fail\n");
    return 0;
  }

  vreboot_fe_init = TRUE;

#if defined(VREBOOT_HOSTTEST_ENABLE)
	hosttest_add_test("vreboot", vreboot_hosttest_func);
#endif

  TRACE("Initialized.\n");
  return 0;
}

#if defined(VREBOOT_HOSTTEST_ENABLE)
static void vreboot_exit(void)
#else
static void __exit vreboot_exit(void)
#endif
{
  TRACE("Module unloaded.\n");
}


module_init(vreboot_init);
#if !defined(VREBOOT_HOSTTEST_ENABLE)
module_exit(vreboot_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("virtual reboot");
