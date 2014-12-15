/*
 ****************************************************************
 *
 *  Component: VirtualLogix VVFS-BE driver
 *
 *  Copyright (C) 2012 - 2013 Intel Mobile Communications GmbH
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
 ****************************************************************
 */

/*----- System header files -----*/

#include <linux/module.h>    /* __exit, __init */
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/ctype.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION (2,6,7)
/* This include exists in 2.6.6 but functions are not yet exported */
#include <linux/kthread.h>
#endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
#include <linux/freezer.h>
#endif
#include <linux/proc_fs.h>
#include <asm/io.h>        /* ioremap */
#if LINUX_VERSION_CODE >= KERNEL_VERSION (2,6,27)
#include <linux/semaphore.h>
#endif
#include <linux/init.h>        /* module_init() in 2.6.0 and before */
#include <linux/major.h>
#include <linux/wakelock.h>
#include <linux/seq_file.h>


#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/nk_sofia_bridge.h>
#include "vvfs_common.h"
#else
#include <vlx/vvfs_common.h>
#include <nk/nkern.h>
#endif

/*----- Local header files -----*/

#include "vlx-vmq.h"
#include "vlx-vipc.h"

#define VLX_SERVICES_THREADS
#include "vlx-services.c"

/*----- Local configuration -----*/

#if 0
#define VVFS_DEBUG
#endif

#if 0
#define VVFS_XDEBUG  /* Activates old/noisy debug traces */
#endif

/*----- Tracing -----*/

#define TRACE(_f, _a...)  printk (KERN_INFO    "VVFS-BE: " _f, ## _a)
#define ETRACE(_f, _a...) printk (KERN_ERR     "VVFS-BE: Error: " _f, ## _a)

#ifdef VVFS_DEBUG
#define DTRACE(_f, _a...) \
        do { printk (KERN_ALERT "VVFS-BE: %s: " _f, __func__, ## _a); } while (0)
#else
#define DTRACE(_f, _a...) ((void)0)
#endif

#ifdef VVFS_XDEBUG
#define XTRACE(_f, _a...) \
        do { printk (KERN_ALERT "VVFS-BE: %s: " _f, __func__, ## _a); } while (0)
#else
#define XTRACE(_f, _a...)
#endif

#ifdef VVFS_ASSERTS
#define VVFS_ASSERT(c) do { if (!(c)) BUG(); } while (0)
#else
#define VVFS_ASSERT(c)
#endif

/*----- Data types: vvfs_be descriptor -----*/

typedef struct {
    struct semaphore    thread_sem;
    _Bool           is_thread_aborted;
    _Bool           is_sysconf;
    _Bool           is_receive;
    _Bool           is_devinit;
    vlx_thread_t    thread_desc;
    struct proc_dir_entry*  proc;
    struct proc_dir_entry*  fe_proc;
    vmq_links_t*    links;  /* global */
    vmq_link_t*     link;
    vipc_ctx_t*     vipc_ctx;
    char*           data_area;    /* for link */
    _Bool           link_active;
    unsigned        calls [NKDEV_VFS_OP_MAX];
    /* Device details */
    _Bool           dev_active;
    char            devname [NKDEV_VFS_NAME_LIMIT];
    unsigned        devid;
    unsigned        devsize;
    struct file*    fd;
} vvfs_be_t;

typedef struct {
    vvfs_be_t*      vvfs;
    vipc_ctx_t      vipc_ctx;
} vvfs_link_t;

/*----- Global Variables -----*/

static vvfs_be_t    vvfs_vfs_be;
static const char*  vvfs_op_names[] = {NKDEV_VFS_OP_NAMES};
static char         vvfs_erase_buf[32*1024];  /* 32kb */
static struct wake_lock vvfs_be_suspend_lock;

/* Local Defines */

#define VVFS_CALL(vvfs_be,name)    ++(vvfs_be)->calls [name]

#define vvfs_kfree_and_clear(ptr) \
    do {kfree (ptr); (ptr) = NULL;} while (0)

#define VVFS_LINK(link) \
        (*(vvfs_link_t**) &((vmq_link_public_t*) (link))->priv)
#define VVFS_LINKS(links) \
        ((vvfs_be_t*) ((vmq_links_public*) (links))->priv)
#undef VVFS_LINKS
#define VVFS_LINKS(links) (*(vvfs_be_t**) (links))

/* Local Function Prototypes */

static void vvfs_init_device_notify (vvfs_be_t* vvfs_be);
static void vvfs_exit_device (vvfs_be_t* vvfs_be);
static _Bool vvfs_flush_newreq (vmq_link_t* link, void* cookie);

/*----- Version compatibility functions -----*/

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,7)
    static inline void*
kzalloc (size_t size, unsigned flags)
{
    void* ptr = kmalloc (size, flags);
    if (ptr) {
        memset (ptr, 0, size);
    }
    return ptr;
}
#endif

int vvfs_device_read(vvfs_be_t* vvfs_be, loff_t offset, char* buf, size_t count)
{
    ssize_t cnt;
	loff_t pos;
    mm_segment_t old_fs = get_fs();

    XTRACE("off %lld buf 0x%08x cnt %d\n", offset, (nku32_f)buf, count);
   if((pos = vfs_llseek(vvfs_be->fd, offset, SEEK_SET)) < 0){
        DTRACE(" Seeking in file failed \n");
	    return 0;
	}
	DTRACE ("pos = %d\n", pos);
    set_fs(KERNEL_DS);
    cnt = vfs_read(vvfs_be->fd, buf, count, &pos);
    set_fs(old_fs);
    return cnt;
}

int vvfs_device_write(vvfs_be_t* vvfs_be, loff_t offset, char* buf, size_t count)
{
    ssize_t cnt;
	loff_t pos;
    mm_segment_t old_fs = get_fs();

    XTRACE("off %lld buf 0x%08x cnt %d\n", offset, (nku32_f)buf, count);
   if((pos = vfs_llseek(vvfs_be->fd, offset, SEEK_SET)) < 0){
        DTRACE(" Seeking in file failed \n");
	    return 0;
	}
	DTRACE("pos = %d\n", pos);
    set_fs(KERNEL_DS);
    cnt = vfs_write(vvfs_be->fd, buf, count, &pos);
    set_fs(old_fs);
    return cnt;
}

int vvfs_device_erase(vvfs_be_t* vvfs_be, loff_t offset, size_t count)
{
    size_t count0 = 0;
    ssize_t cnt;
	loff_t pos;
    mm_segment_t old_fs = get_fs();

   XTRACE("off %lld cnt %d\n", offset, count);
   if((pos = vfs_llseek(vvfs_be->fd, offset, SEEK_SET)) < 0){
        DTRACE(" Seeking in file failed \n");
	    return count0;
	}
	DTRACE("pos = %d\n", pos);

   /* Better use the IOCTL exported by driver for erase */
    while(count > 0) {
        size_t my_count = count > sizeof vvfs_erase_buf ? sizeof vvfs_erase_buf : count;
        set_fs(KERNEL_DS);
        cnt = vfs_write(vvfs_be->fd, vvfs_erase_buf, my_count, &pos);
        set_fs(old_fs);
        if(cnt < 0) return cnt;
        if(cnt != my_count){
            DTRACE("Erase with multiple-fills failed\n");
            return (count0 + cnt);
        }
        /* write sets offset to current fp position  */
        count0 += my_count;
		pos += my_count;
        count  -= my_count;
    }
    return count0;
}

/*----- Common call management -----*/

    static nkdev_vfs_request*
vvfs_alloc_async (vmq_link_t* link, unsigned op,
        unsigned data_len, unsigned* data_offset)
{
    nkdev_vfs_msg*    msg;
    signed    diag;

    /* Data freed by allocator only */
    diag = vmq_msg_allocate (link, data_len, (void**) &msg, data_offset);
    if (diag) {
    ETRACE ("vmq msg alloc (%d)\n", diag);
    return NULL;
    }
    XTRACE ("msg 0x%x\n", (unsigned)msg);
    msg->req.op       = op;
    msg->req.flags    = NKDEV_VFS_FLAGS_NOTIFICATION;
    msg->req.cookie   = 0;  /* Async msgs have null cookie */
    msg->req.dataOffset = data_offset ? *data_offset : 0;
    msg->req.isData = (data_offset != NULL);
    return &msg->req;
}

    /* Can sleep for place in FIFO */

    static inline signed
vvfs_alloc_req (vvfs_be_t* vvfs_be, unsigned op, unsigned data_len,
        nkdev_vfs_request** preq, unsigned* data_offset)
{
    nkdev_vfs_msg*    msg;
    signed        diag;

    XTRACE ("\n");
    if (unlikely (!vvfs_be->link_active)) return -EAGAIN;

    /* Data freed by allocator only */
    diag = vmq_msg_allocate (vvfs_be->link, data_len, (void**) &msg, data_offset);
    if (unlikely (diag)) {
        ETRACE ("vmq msg alloc error, %d\n", diag);
        return diag;
    }
    msg->req.op = op;
    msg->req.flags = NKDEV_VFS_FLAGS_REQUEST;
    msg->req.dataOffset = data_offset ? *data_offset : 0;
    msg->req.isData = (data_offset != NULL);
    *preq = &msg->req;
    return 0;
}

    static inline void
vvfs_send_reply (vmq_link_t* link, signed retcode, nkdev_vfs_request* req)
{
    nkdev_vfs_reply* reply = (nkdev_vfs_reply*) req;

    XTRACE ("%s msg 0x%p retcode %d retvalue %d\n", vvfs_op_names[reply->op % NKDEV_VFS_OP_MAX],
            reply, retcode, reply->retvalue);
    DTRACE ("retcode %d cookie 0x%llx\n", retcode, reply->cookie);
    reply->retcode = (unsigned) retcode;
    reply->flags = NKDEV_VFS_FLAGS_REPLY;
    vmq_msg_return (link, reply);
}

    static inline void
vvfs_free_reply (vmq_link_t* link, nkdev_vfs_reply* reply)
{
    reply->cookie = 0;    /* For security versus stale requests */
    if (reply->isData)
        vmq_data_free (link, reply->dataOffset);
    vmq_return_msg_free (link, reply);
}

    /* Executes in interrupt context */

    static void
vvfs_process_reply (vmq_link_t* link, nkdev_vfs_msg* msg)
{
    vvfs_link_t* be_link = VVFS_LINK (link);
    nkdev_vfs_reply* reply = (nkdev_vfs_reply*) msg;

    DTRACE ("%s link %d cookie 0x%llx retcode %d\n",
                vvfs_op_names[reply->op % NKDEV_VFS_OP_MAX],
                vmq_peer_osid (link), reply->cookie, reply->retcode);
    if (reply->flags != NKDEV_VFS_FLAGS_REPLY ||
        vipc_ctx_process_reply (&be_link->vipc_ctx,
                    &reply->cookie)) {
        /* Freed / replied async req's are handled here */
        vvfs_free_reply (link, reply);
    }
}

    /*
     *   IN: have req
     *  OUT: req released, may have reply
     */

    static nkdev_vfs_reply*
vvfs_call_be (vvfs_be_t* vvfs_be, nkdev_vfs_request* req)
{
    nku64_f* reply_cookie;

    XTRACE ("%s\n", vvfs_op_names[req->op % NKDEV_VFS_OP_MAX]);
    VVFS_CALL (vvfs_be, req->op);
    reply_cookie = vipc_ctx_call (vvfs_be->vipc_ctx, &req->cookie);
    if(reply_cookie) {
        nkdev_vfs_reply* reply = container_of (reply_cookie, nkdev_vfs_reply, cookie);

        if(((signed)reply->retcode) < 0) {
            ETRACE ("%s reply, error %d\n",
                vvfs_op_names[reply->op % NKDEV_VFS_OP_MAX],
                reply->retcode);
        }
        return reply;
    }
    else {
        ETRACE ("%s: %s error\n", __func__,
                vvfs_op_names[req->op % NKDEV_VFS_OP_MAX]);
        return NULL;
    }
}

/*----- VVFS device handling -----*/

    /*
     * For convenience we distinguish between ide, scsi and 'other' (i.e.
     * potentially combinations of the two) in the naming scheme and in a few
     * other places (like default readahead, etc).
     */
#define VVFS_NUM_IDE_MAJORS     10
#define VVFS_NUM_SCSI_MAJORS    9
#define VVFS_NUM_VVFS_MAJORS    1
#define VVFS_NUM_FD_MAJORS      1
#ifdef MMC_BLOCK_MAJOR
#define VVFS_NUM_MMC_MAJORS     1
#else
#define VVFS_NUM_MMC_MAJORS     0
#endif

typedef struct {
    const char* name;
    int     minors_per_dev;
    int     devs_per_major;
} vvfs_type_t;

static const vvfs_type_t vvfs_type_ide  = {"hd",  64,  2};
static const vvfs_type_t vvfs_type_scsi = {"sd",  16, 16};
static const vvfs_type_t vvfs_type_vvfs  = {"xvd", 16,  0};
static const vvfs_type_t vvfs_type_fd   = {"fd",  1,  1};
#ifdef MMC_BLOCK_MAJOR
#ifdef CONFIG_MMC_BLOCK_MINORS
static const vvfs_type_t vvfs_type_mmc  = {"mmc",
        CONFIG_MMC_BLOCK_MINORS, 256 / CONFIG_MMC_BLOCK_MINORS};
#else
static const vvfs_type_t vvfs_type_mmc  = {"mmc", 8, 256 / 8};
#endif
#endif

    static void
vvfs_be_get_major (vvfs_be_t* vvfs_be)
{
    const int       major = NKDEV_VFS_DEVID_MAJOR (vvfs_be->devid);
    const int       minor = NKDEV_VFS_DEVID_MINOR (vvfs_be->devid);
    int             major_idx, new_major, part;
    int             index;
    const vvfs_type_t*  type;

    switch (major) {
    case IDE0_MAJOR: major_idx = 0; new_major = IDE0_MAJOR; break;
    case IDE1_MAJOR: major_idx = 1; new_major = IDE1_MAJOR; break;
    case IDE2_MAJOR: major_idx = 2; new_major = IDE2_MAJOR; break;
    case IDE3_MAJOR: major_idx = 3; new_major = IDE3_MAJOR; break;
    case IDE4_MAJOR: major_idx = 4; new_major = IDE4_MAJOR; break;
    case IDE5_MAJOR: major_idx = 5; new_major = IDE5_MAJOR; break;
    case IDE6_MAJOR: major_idx = 6; new_major = IDE6_MAJOR; break;
    case IDE7_MAJOR: major_idx = 7; new_major = IDE7_MAJOR; break;
    case IDE8_MAJOR: major_idx = 8; new_major = IDE8_MAJOR; break;
    case IDE9_MAJOR: major_idx = 9; new_major = IDE9_MAJOR; break;
    case SCSI_DISK0_MAJOR: major_idx = 10; new_major = SCSI_DISK0_MAJOR; break;
    case SCSI_DISK1_MAJOR ... SCSI_DISK7_MAJOR:
        major_idx = 11 + major - SCSI_DISK1_MAJOR;
        new_major = SCSI_DISK1_MAJOR + major - SCSI_DISK1_MAJOR;
        break;
    case SCSI_CDROM_MAJOR: major_idx = 18; new_major = SCSI_CDROM_MAJOR; break;
    case FLOPPY_MAJOR: major_idx = 19; new_major = FLOPPY_MAJOR; break;
#ifdef MMC_BLOCK_MAJOR
    case MMC_BLOCK_MAJOR: major_idx = 20; new_major = MMC_BLOCK_MAJOR; break;
    default: major_idx = 21; new_major = 0; break;
#else
    default: major_idx = 20; new_major = 0; break;
#endif
    }
    switch (major_idx) {
    case 0 ... VVFS_NUM_IDE_MAJORS - 1:
    type = &vvfs_type_ide;
    index = major_idx;
    break;

    case VVFS_NUM_IDE_MAJORS ...  VVFS_NUM_IDE_MAJORS + VVFS_NUM_SCSI_MAJORS - 1:
    type = &vvfs_type_scsi;
    index = major_idx - VVFS_NUM_IDE_MAJORS;
    break;

    case VVFS_NUM_IDE_MAJORS + VVFS_NUM_SCSI_MAJORS ...
        VVFS_NUM_IDE_MAJORS + VVFS_NUM_SCSI_MAJORS + VVFS_NUM_FD_MAJORS - 1:
    type = &vvfs_type_fd;
    index = major_idx - (VVFS_NUM_IDE_MAJORS + VVFS_NUM_SCSI_MAJORS);
    break;

#ifdef MMC_BLOCK_MAJOR
    case VVFS_NUM_IDE_MAJORS + VVFS_NUM_SCSI_MAJORS + VVFS_NUM_FD_MAJORS ...
        VVFS_NUM_IDE_MAJORS + VVFS_NUM_SCSI_MAJORS + VVFS_NUM_FD_MAJORS +
        VVFS_NUM_MMC_MAJORS - 1:
    type = &vvfs_type_mmc;
    index = major_idx -
        (VVFS_NUM_IDE_MAJORS + VVFS_NUM_SCSI_MAJORS + VVFS_NUM_FD_MAJORS);
    break;
#endif

    case VVFS_NUM_IDE_MAJORS + VVFS_NUM_SCSI_MAJORS + VVFS_NUM_FD_MAJORS +
        VVFS_NUM_MMC_MAJORS ...
        VVFS_NUM_IDE_MAJORS + VVFS_NUM_SCSI_MAJORS + VVFS_NUM_FD_MAJORS +
        VVFS_NUM_MMC_MAJORS + VVFS_NUM_VVFS_MAJORS - 1:
    type = &vvfs_type_vvfs;
    index = major_idx -
        (VVFS_NUM_IDE_MAJORS + VVFS_NUM_SCSI_MAJORS +
         VVFS_NUM_FD_MAJORS + VVFS_NUM_MMC_MAJORS);
    break;
    default:
	ETRACE("Unsupported major index %d\n", major_idx);
	return;
    }

    /* Construct an appropriate device name */
    part = minor % type->minors_per_dev;

    if (part) {
#ifdef MMC_BLOCK_MAJOR
        if (new_major == MMC_BLOCK_MAJOR) {
            snprintf (vvfs_be->devname, sizeof vvfs_be->devname, "mmcblk%dp%d",
                  minor / type->minors_per_dev, part);
        } else
#endif
        {
            snprintf (vvfs_be->devname, sizeof vvfs_be->devname,
                  "%s%c%d", type->name,
                  'a' + (index << 1) +
                  (minor / type->minors_per_dev), part);
        }
    } else {
        /* Floppy disk special naming rules */
        if (!strcmp (type->name, "fd")) {
            snprintf (vvfs_be->devname, sizeof vvfs_be->devname,
                  "%s%c", type->name, '0');
#ifdef MMC_BLOCK_MAJOR
        } else if (new_major == MMC_BLOCK_MAJOR) {
            snprintf (vvfs_be->devname, sizeof vvfs_be->devname, "mmcblk%d",
                  minor / type->minors_per_dev);
#endif
        } else {
            snprintf (vvfs_be->devname, sizeof vvfs_be->devname, "%s%c",
                  type->name, 'a' + (index << 1) +
                  (minor / type->minors_per_dev));
        }
    }
}

    /* Called from vvfs_probe_devid() only */

    static signed
vvfs_get_devid (vvfs_be_t* vvfs_be)
{
    nkdev_vfs_request*    req;
    nkdev_vfs_reply*    reply;
    signed        diag;

    DTRACE ("\n");
    diag = vvfs_alloc_req (vvfs_be, NKDEV_VFS_OP_GET_DEVICE_ID, 0, &req, NULL);
    if (diag) return diag;

    req->size = 0x100;    /* Version 1.0 */
    reply = vvfs_call_be (vvfs_be, req);
    if (!reply) {
        diag = -EBUSY;
        goto error;
    }
    diag = reply->retcode;
    if (!diag) {
        vvfs_be->devid = reply->retvalue;
    }

error:
    vvfs_free_reply (vvfs_be->link, reply);
    return diag;
}

    static signed
vvfs_probe_devid (vvfs_be_t* vvfs_be)
{
    DTRACE ("\n");
    if (vvfs_get_devid (vvfs_be)) {
        return -ENODEV;
    }
    if (vvfs_be->devid == 0) {
        ETRACE ("Could not proceed further as no device configured\n");
        return -ENODEV;
    }
    vvfs_be_get_major(vvfs_be);
    DTRACE ("device id (%u,%u) name %s\n", NKDEV_VFS_DEVID_MAJOR (vvfs_be->devid),
                NKDEV_VFS_DEVID_MINOR (vvfs_be->devid), vvfs_be->devname);
    vvfs_init_device_notify(vvfs_be);
    return 0;
}

    static signed
vvfs_set_devinfo (vvfs_be_t* vvfs_be)
{
    nkdev_vfs_request*    req;
    nkdev_vfs_reply*    reply;
    signed        diag;

    DTRACE ("\n");
    diag = vvfs_alloc_req (vvfs_be, NKDEV_VFS_OP_DEVICE_INFO, 0, &req, NULL);
    if (diag) return diag;

    req->addr = vvfs_be->devid;
    req->size = vvfs_be->devsize;
    reply = vvfs_call_be (vvfs_be, req);
    if (!reply) {
        diag = -EBUSY;
        goto error;
    }
    diag = reply->retcode;

error:
    vvfs_free_reply (vvfs_be->link, reply);
    return diag;
}

    static signed
vvfs_set_devdown (vvfs_be_t* vvfs_be)
{
    nkdev_vfs_request*    async;

    DTRACE ("\n");
    vvfs_exit_device(vvfs_be);

    if((async = vvfs_alloc_async(vvfs_be->link, NKDEV_VFS_OP_DEVICE_DOWN, 0, NULL)) == NULL)
        return -EAGAIN;

    VVFS_CALL (vvfs_be, async->op);
    vmq_msg_send (vvfs_be->link, async);
    /* May got new req by the time device down is informed */
    vmq_links_iterate (vvfs_be->links, vvfs_flush_newreq, NULL);
    return 0;
}

    static void
vvfs_exit_device (vvfs_be_t* vvfs_be)
{
    mm_segment_t old_fs;

    XTRACE ("\n");
    if(vvfs_be->dev_active == false) return;

    vvfs_be->dev_active = false;
    old_fs = get_fs();
    set_fs(KERNEL_DS);
    filp_close(vvfs_be->fd, NULL);
    set_fs(old_fs);
    vvfs_be->fd = NULL;
    vvfs_be->devsize = 0;
    return;
}

    /* Called only from vvfs_thread() */

    static int
vvfs_init_device (vvfs_be_t* vvfs_be, _Bool is_retry)
{
    signed diag;
	char dev_name[75];
    loff_t pos;
    struct file *fd;
    mm_segment_t old_fs = get_fs();

    XTRACE ("\n");
    snprintf(dev_name, sizeof dev_name, "/dev/block/%s", vvfs_be->devname);
    /*snprintf(dev_name, sizeof dev_name, "/dev/block/mmcblk0p3");*/
	snprintf(dev_name, sizeof(dev_name),
				"/dev/block/platform/soc0/e0000000.noc/by-name/ImcPartID022");
    if(vvfs_be->dev_active == true) {
      TRACE ("device %s size %u bytes already opened\n", dev_name, vvfs_be->devsize);
      if((diag = vvfs_set_devinfo(vvfs_be)) != 0) {
          ETRACE("could not send device info\n");
          goto dev_error;
      }
      return 0;
    }

    set_fs(KERNEL_DS);
    fd = filp_open(dev_name, (O_RDWR | O_SYNC), 0);
    set_fs(old_fs);

    if(unlikely(IS_ERR(fd))){
        /* Better use system events to get notified on device removal/creation */
        ETRACE("open device %s failed, retrying\n", dev_name);
        goto dev_retry;
    }
    vvfs_be->fd = fd;
    if((pos = vfs_llseek(fd, 0,SEEK_END))< 0){
        ETRACE("Seek to end of device failed\n");
        diag = -ESPIPE;
        goto dev_error;
    }
    if(pos == 0) {
        ETRACE("device %s size 0 bytes\n", dev_name);
        diag = -ESPIPE;
        goto dev_error;
    }
    /* VFS assumes that after erase flash content is 0xFF */
    memset(vvfs_erase_buf, 0xFF, sizeof vvfs_erase_buf);
    vvfs_be->devsize = pos;
    vvfs_be->dev_active = true;
    TRACE ("opened device %s size %lld bytes\n", dev_name, pos);
    if((diag = vvfs_set_devinfo(vvfs_be)) != 0) {
        ETRACE("could not send device info\n");
        goto dev_error;
    }
    return 0;

dev_error:
    /* No retry */
    vvfs_exit_device(vvfs_be);
    return diag;

dev_retry:
    if (is_retry) {
      /* Waiting for the device before retry */
      set_current_state (TASK_INTERRUPTIBLE);
      schedule_timeout (1000);
      vvfs_init_device_notify(vvfs_be);
    }
    return -EAGAIN;
}

    static int
vvfs_testsuite_ext_helper (vvfs_be_t* vvfs_be, nkdev_vfs_request* req)
{
  nkdev_vfs_reply* reply = (nkdev_vfs_reply*)req;
  signed result = 0;

  DTRACE ("testsuite sub-op %d\n", req->addr);
  switch (req->addr) {
  case NKDEV_VFS_TEST_GET_STATUS:
  reply->retvalue = vvfs_be->dev_active;
  break;

  case NKDEV_VFS_TEST_DEV_PROBE:
  result = vvfs_probe_devid(vvfs_be);
  break;

  case NKDEV_VFS_TEST_DEV_INIT:
  result = vvfs_init_device(vvfs_be, false);
  break;

  case NKDEV_VFS_TEST_DEV_DOWN:
  result = vvfs_set_devdown(vvfs_be);
  break;

  default:
  result = -EINVAL;
  break;
  }

  return result;
}

    static void
vvfs_process_msg (vmq_link_t* link, nkdev_vfs_msg* msg, char* data_area,
           NkOsId osid)
{
    vvfs_be_t* vvfs_be = VVFS_LINK (link)->vvfs;
    nkdev_vfs_request* req = &msg->req;
    unsigned data_offset = req->dataOffset;

    /* Just stopping processing messages will soon block sender */
    if (vvfs_be->is_thread_aborted) return;
    DTRACE ("op %s flag 0x%x\n", vvfs_op_names [req->op % NKDEV_VFS_OP_MAX], req->flags);
    XTRACE ("msg 0x%p isData %d data 0x%x\n", msg, req->isData, req->dataOffset);
    if (req->flags != NKDEV_VFS_FLAGS_REQUEST) {
    DTRACE ("Ignoring request with invalid flags 0x%x\n", req->flags);
    vvfs_send_reply (link, -EINVAL, req);
    return;
    }
    if (req->isData && !vmq_data_offset_ok (link, data_offset)) {
    DTRACE ("Ignoring request with invalid dataOffset 0x%x\n", data_offset);
    vvfs_send_reply (link, -EINVAL, req);
    return;
    }
    VVFS_CALL (vvfs_be, req->op % NKDEV_VFS_OP_MAX);

    switch (req->op) {
    case NKDEV_VFS_OP_READ_DATA: {
    char* buf = (char*) (data_area + data_offset);
    ssize_t cnt;

    if(vvfs_be->dev_active == false)
        cnt = -ENODEV;
    else
        cnt = vvfs_device_read(vvfs_be, req->addr, buf, req->size);
    vvfs_send_reply (link, cnt, req);
    /* Initiate recovery for certain errors */
    if (cnt == -EBADF) {
        vvfs_set_devdown(vvfs_be);
        vvfs_init_device_notify(vvfs_be);
    }
    break;
    }
    case NKDEV_VFS_OP_WRITE_DATA: {
    char* buf = (char*) (data_area + data_offset);
    ssize_t cnt;

    if(vvfs_be->dev_active == false)
        cnt = -ENODEV;
    else
        cnt = vvfs_device_write(vvfs_be, req->addr, buf, req->size);
    vvfs_send_reply (link, cnt, req);
    /* Initiate recovery for certain errors */
    if (cnt == -EBADF) {
        vvfs_set_devdown(vvfs_be);
        vvfs_init_device_notify(vvfs_be);
    }
    break;
    }
    case NKDEV_VFS_OP_ERASE_DATA: {
    ssize_t cnt;

    if(vvfs_be->dev_active == false)
        cnt = -ENODEV;
    else
        cnt = vvfs_device_erase(vvfs_be, req->addr, req->size);
    vvfs_send_reply (link, cnt, req);
    /* Initiate recovery for certain errors */
    if (cnt == -EBADF) {
        vvfs_set_devdown(vvfs_be);
        vvfs_init_device_notify(vvfs_be);
    }
    break;
    }
    case NKDEV_VFS_OP_TESTSUITE: {
    signed retcode;

    retcode = vvfs_testsuite_ext_helper (vvfs_be, req);
    vvfs_send_reply (link, retcode, req);
    break;
    }

    default:
    DTRACE ("op %d invalid request\n", req->op);
    vvfs_send_reply (link, -EINVAL, req);
    break;
    }
}

/*----- Module thread -----*/

    static _Bool
vvfs_flush_newreq (vmq_link_t* link, void* cookie)
{
    nkdev_vfs_msg* msg;

    (void) cookie;
    XTRACE ("\n");
    while (!vmq_msg_receive (link, (void**)&msg)) {
    nkdev_vfs_request* req = &msg->req;

    if (req->flags != NKDEV_VFS_FLAGS_NOTIFICATION) {
        vvfs_send_reply (link, -ENODEV, req);
    } else {
        req->cookie = 0;    /* For security versus stale requests */
        vmq_msg_free (link, req);
    }
    }
    return false;
}

    static _Bool
vvfs_receive_link (vmq_link_t* link, void* cookie)
{
    const NkOsId    osid = vmq_peer_osid (link);
    char*        data_area = vmq_rx_data_area (link);
    void*        msg;

    (void) cookie;
    while (!vmq_msg_receive (link, &msg)) {
	vvfs_process_msg(link, (nkdev_vfs_msg *) msg, data_area, osid);
	wake_unlock(&vvfs_be_suspend_lock);
    }
    return false;
}

    static void
vvfs_thread_aborted_notify (vvfs_be_t* vvfs_be)
{
    vvfs_be->is_thread_aborted = true;
    up (&vvfs_be->thread_sem);
}

    static void
vvfs_init_device_notify (vvfs_be_t* vvfs_be)
{
    vvfs_be->is_devinit = true;
    up (&vvfs_be->thread_sem);
}

    /* All CallBacks Executes in interrupt context */

    static void
vvfs_sysconf_notify (vmq_links_t* links)
{
    vvfs_be_t* vvfs_be = VVFS_LINKS (links);

    DTRACE ("\n");
    vvfs_be->is_sysconf = true;
    up (&vvfs_be->thread_sem);
}

    static void
vvfs_return_notify (vmq_link_t* link)
{
    void* msg;

    XTRACE ("\n");
    while (!vmq_return_msg_receive (link, &msg)) {
        vvfs_process_reply (link, (nkdev_vfs_msg*) msg);
    }
}

    static void
vvfs_receive_notify (vmq_link_t* link)
{
    vvfs_be_t* vvfs_be = VVFS_LINK (link)->vvfs;

	XTRACE("\n");
	wake_lock(&vvfs_be_suspend_lock);
    vvfs_be->is_receive = true;
    up (&vvfs_be->thread_sem);
}

    static int
vvfs_thread (void* arg)
{
    vvfs_be_t* vvfs_be = arg;

    DTRACE ("started\n");
    while (!vvfs_be->is_thread_aborted) {
    int diag = down_interruptible (&vvfs_be->thread_sem);

    (void) diag;
    DTRACE ("wakeup%s%s%s%s\n",
        vvfs_be->is_thread_aborted ? " thread-aborted"  : "",
        vvfs_be->is_receive        ? " receive"         : "",
        vvfs_be->is_devinit        ? " devinit"         : "",
        vvfs_be->is_sysconf        ? " sysconf"         : "");

    if (vvfs_be->is_thread_aborted)
        break;
    if (vvfs_be->is_sysconf) {
        vvfs_be->is_sysconf = false;
        vmq_links_sysconf (vvfs_be->links);
    }
    if (vvfs_be->is_receive) {
        vvfs_be->is_receive = false;
        vmq_links_iterate (vvfs_be->links, vvfs_receive_link, NULL);
    }
    if (vvfs_be->is_devinit) {
        vvfs_be->is_devinit = false;
        vvfs_init_device (vvfs_be, true);
    }
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,11)
        /* Check if freeze signal has been received */
    try_to_freeze();
#endif
    }
    DTRACE ("exiting\n");
    return 0;
}

/*----- Support for /proc/nk/vvfs-fe -----*/

    static int
vvfs_fe_proc_show  (struct seq_file* seq, void* v)
{
    vvfs_be_t*          vvfs_be = seq->private;
    nkdev_vfs_request*  req;
    nkdev_vfs_reply*    reply;
    signed              diag;

    DTRACE ("\n");
    diag = vvfs_alloc_req (vvfs_be, NKDEV_VFS_OP_PRINT_STATE, 0, &req, NULL);
    if (diag) {
      ETRACE ("msg alloc failed, err %d\n", diag);
      return 0;
    }
    reply = vvfs_call_be (vvfs_be, req);
    if (!reply) {
      ETRACE ("call_be failed\n");
      return 0;
    }
    vvfs_free_reply (vvfs_be->link, reply);
    return 0;
}

/*----- Support for /proc/nk/vvfs_be -----*/

    static int
vvfs_proc_show (struct seq_file* seq, void* v)
{
    vvfs_be_t*  vvfs_be = seq->private;
    unsigned    i;
    _Bool       had_any = false;

    seq_printf (seq,
            "OS %2d Flags %c%c\n",
            (vvfs_be->link ? vmq_peer_osid (vvfs_be->link) : ~0),
            vvfs_be->link_active   ? 'A' : '.',
            vvfs_be->dev_active    ? 'A' : '.');

    seq_printf(seq,
            "Device id (%u,%u) name '%s' size %u bytes fd 0x%p\n",
            NKDEV_VFS_DEVID_MAJOR (vvfs_be->devid),
	    NKDEV_VFS_DEVID_MINOR (vvfs_be->devid),
            vvfs_be->devname, vvfs_be->devsize, vvfs_be->fd);

    for (i = 0; i < NKDEV_VFS_OP_MAX; ++i) {
        if (!vvfs_be->calls [i])
		continue;
        if (!had_any)
            seq_printf(seq, "  ");

        seq_printf (seq, " %s:%u\n", vvfs_op_names [i],
                vvfs_be->calls [i]);
        had_any = true;
    }

    if (had_any)
        seq_printf (seq, "\n");

    seq_printf (seq, "\n");

    return 0;
}

    /*----- Interface with vlx-vmq.c -----*/

    static void
vvfs_link_on (vmq_link_t* link)
{
    vvfs_be_t* vvfs_be = VVFS_LINK (link)->vvfs;

    DTRACE ("link on (local <-> OS %d)\n", vmq_peer_osid (link));
    vvfs_be->link_active = true;
    /* To do: Probe Yes & Init Yes & Set info Yes
     *        Probe No & Init Yes & Set info Yes
     *        Probe No & Init No & Set info Yes
     */
    if (vvfs_be->devid == 0)
        vvfs_probe_devid (vvfs_be);
    else if(vvfs_be->dev_active == false)
        vvfs_init_device_notify(vvfs_be);
    else
        vvfs_set_devinfo(vvfs_be);
}

    static void
vvfs_link_off (vmq_link_t* link)
{
    vvfs_link_t* be_link = VVFS_LINK (link);
    vvfs_be_t* vvfs_be = be_link->vvfs;

    DTRACE ("link off (OS %d)\n", vmq_peer_osid (link));
    if (vvfs_be->link != link) return;

    /* Only abort requests; Will not bring device down,
     * may help in immediate link ON */
    vvfs_be->link_active = false;
    vipc_ctx_abort_calls (&be_link->vipc_ctx);
}

    static void
vvfs_link_off_completed (vmq_link_t* link)
{
    DTRACE ("link off completed (OS %d)\n", vmq_peer_osid (link));
}

/*----- Initialization and exit entry points -----*/

    /* Only called from vvfs_exit() */

    static _Bool
vvfs_link_free (vmq_link_t* link, void* cookie)
{
    vvfs_link_t* be_link = VVFS_LINK (link);

    (void) cookie;
    vvfs_kfree_and_clear (be_link);
    return false;
}

    /* Only called from vvfs_init() */

    static _Bool
vvfs_link_init (vmq_link_t* link, void* cookie)
{
    vvfs_be_t* vvfs_be = cookie;
    vvfs_link_t* be_link = (vvfs_link_t*) kzalloc (sizeof *be_link, GFP_KERNEL);

    if (!be_link) {
    ETRACE ("out of memory for link\n");
    return true;
    }
    VVFS_LINK (link) = be_link;
    be_link->vvfs = vvfs_be;
    vipc_ctx_init (&be_link->vipc_ctx, link);
    vvfs_be->link = link;
    vvfs_be->vipc_ctx = &be_link->vipc_ctx;
    vvfs_be->data_area = vmq_tx_data_area (link);
    return false;
}

#define VVFS_FIELD(name,value)    value

    /* All CallBacks Executes in interrupt context */

static const vmq_callbacks_t vvfs_callbacks = {
    VVFS_FIELD (link_on,            vvfs_link_on),
    VVFS_FIELD (link_off,           vvfs_link_off),
    VVFS_FIELD (link_off_completed, vvfs_link_off_completed),
    VVFS_FIELD (sysconf_notify,     vvfs_sysconf_notify),
    VVFS_FIELD (receive_notify,     vvfs_receive_notify),
    VVFS_FIELD (return_notify,      vvfs_return_notify)
};

static const vmq_xx_config_t vvfs_tx_config = {
    VVFS_FIELD (msg_count,  NKDEV_VFS_TX_MSG_COUNT),
    VVFS_FIELD (msg_max,    sizeof (nkdev_vfs_msg)),
    VVFS_FIELD (data_count, NKDEV_VFS_TX_DATA_COUNT),
    VVFS_FIELD (data_max,   NKDEV_VFS_TX_DATA_MAX)
};

static const vmq_xx_config_t vvfs_rx_config = {
    VVFS_FIELD (msg_count,  NKDEV_VFS_RX_MSG_COUNT),
    VVFS_FIELD (msg_max,    sizeof (nkdev_vfs_msg)),
    VVFS_FIELD (data_count, NKDEV_VFS_RX_DATA_COUNT),
    VVFS_FIELD (data_max,   NKDEV_VFS_RX_DATA_MAX)
};

#undef VVFS_FIELD

    static void
vvfs_exit (void)
{
    vvfs_be_t*  be = &vvfs_vfs_be;

    DTRACE ("exiting\n");
    vvfs_exit_device (be);
    if (be->links) {
    vmq_links_iterate (be->links, vvfs_flush_newreq, NULL);
    vmq_links_abort (be->links);
    }
    vvfs_thread_aborted_notify (be);
    vlx_thread_join (&be->thread_desc);
    if (be->links) {
    vmq_links_iterate (be->links, vvfs_link_free, NULL);
    vmq_links_finish (be->links);
    be->links = NULL;
    }
    if (be->proc) {
    remove_proc_entry ("nk/vvfs-be", NULL);
    }
    if (be->fe_proc) {
    remove_proc_entry ("nk/vvfs-fe", NULL);
    }
	wake_lock_destroy(&vvfs_be_suspend_lock);
}

    /* NOTE:
     * 1. Never abort pending req other that link off. Aborting elsewhere makes
     *    vmq link unstable. Let backend gracefully flush.
     * 2. Data freed by allocator only.
     * 3. Data Offset of '0' is also valid.
     * 4. Read /proc/nk/vmq.vvfs-xx to get vmq status.
     * 5. Follow the follwing req flow always. Skipping /interchanging procedure makes
     *    vmq link unstable.

                    OS (1)           |        OS (2)
                                     |
            vmq_msg_allocate         |
             (optional data)         |
            vmq_msg_send        -------->  receive_notify
             (Wait for response,     |     vmq_msg_receive
                if sync)             |
            -------------------------|
            return_notify       <--------  vmq_msg_return (if sync) /
            vmq_return_msg_receive   |        vmq_msg_free (if async)
             (Release wait, if sync) |
            vmq_data_free (if data)  |
            vvfs_free_reply          |
    */

static int vvfs_proc_open (struct inode* inode, struct file* file)
{
    return single_open (file, vvfs_proc_show, PDE_DATA(inode));
}

static int vvfs_fe_proc_open (struct inode* inode, struct file* file)
{
    return single_open (file, vvfs_fe_proc_show, PDE_DATA(inode));
}

static const struct file_operations vvfs_proc_fops = {
    .owner	= THIS_MODULE,
    .open	= vvfs_proc_open,
    .read	= seq_read,
    .llseek	= seq_lseek,
    .release	= single_release,

};

static const struct file_operations vvfs_fe_proc_fops = {
    .owner	= THIS_MODULE,
    .open	= vvfs_fe_proc_open,
    .read	= seq_read,
    .llseek	= seq_lseek,
    .release	= single_release,

};


    static int __init
vvfs_init (void)
{
    vvfs_be_t*  be = &vvfs_vfs_be;
    int         diag;

    DTRACE ("initializing\n");
    be->proc = proc_create_data ("nk/vvfs-be", 0, NULL,
                       &vvfs_proc_fops, be);
    be->fe_proc = proc_create_data ("nk/vvfs-fe", 0, NULL,
                       &vvfs_fe_proc_fops, be);
    sema_init (&be->thread_sem, 0);    /* Before it is signaled */
	wake_lock_init(&vvfs_be_suspend_lock,
					WAKE_LOCK_SUSPEND,
					"VVFS_BE_EMMC_ACCESS_WAKE_LOCK");
    diag = vmq_links_init_ex (&be->links, "vvfs", &vvfs_callbacks,
                  &vvfs_tx_config, &vvfs_rx_config, be, false);
    if (diag) goto error;
    if (vmq_links_iterate (be->links, vvfs_link_init, be)) {
    diag = -ENOMEM;
    goto error;
    }
    diag = vmq_links_start (be->links);
    if (diag) goto error;
    diag = vlx_thread_start (&be->thread_desc, vvfs_thread, be, "vvfs-be");
    if (diag) {
    ETRACE ("thread start failure\n");
    goto error;
    }
    TRACE ("initialized\n");
    return 0;

error:
    ETRACE ("init failed (%d)\n", diag);
    vvfs_exit();
    return diag;
}
    /*
    *  module_init(vvfs_init) executes even before the block device
    *  is registered. So open fails and goes for a retry which misleads
    *  as if there is actual problem.
    */

module_init (vvfs_init);
module_exit (vvfs_exit);

/*----- Module description -----*/

MODULE_LICENSE ("GPL");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION ("Virtual VFS backend driver");

/*----- End of file -----*/

