/*
 ****************************************************************
 *
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
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

#include <linux/init.h>
#include <linux/module.h>   /* for kernel module struct */
#include <linux/fs.h>       /* for struct file_operation */
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/slab.h>     /* for kmalloc */
#include <linux/wait.h>     /* Required for the wait queues */
#include <linux/sched.h>    /* Required for task states (TASK_INTERRUPTIBLE etc ) */
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/kthread.h>  /* for threads */
#include <linux/sched.h>    /* for task_struct */

#include <asm/uaccess.h>    /* for get_user and put_user */
#include <nvm/nvm.h>

#include "nvmdev.h"
#include "nvm_cfg.h"

#define NVM_MASK_REMOVE_GROUP          (0xF0000000)

/* Declaration of RAM mirrors */
NVM_DECLARE_MIRROR_STA_CAL
#ifdef NVM_DECLARE_MIRROR_STA_CAL_TEST
  NVM_DECLARE_MIRROR_STA_CAL_TEST
#endif
#ifdef NVM_DECLARE_MIRROR_STA_CAL_EXTERN
  NVM_DECLARE_MIRROR_STA_CAL_EXTERN
#endif

NVM_DECLARE_MIRROR_STA_FIX
#ifdef NVM_DECLARE_MIRROR_STA_FIX_TEST
  NVM_DECLARE_MIRROR_STA_FIX_TEST
#endif
#ifdef NVM_DECLARE_MIRROR_STA_FIX_EXTERN
  NVM_DECLARE_MIRROR_STA_FIX_EXTERN
#endif

NVM_DECLARE_MIRROR_DYN
#ifdef NVM_DECLARE_MIRROR_DYN_TEST
  NVM_DECLARE_MIRROR_DYN_TEST
#endif
#ifdef NVM_DECLARE_MIRROR_DYN_EXTERN
  NVM_DECLARE_MIRROR_DYN_EXTERN
#endif

/* Properties for the groups - Calib */
static T_GROUP_PROPERTIES group_prop_cal[] = {
#ifdef NVM_INIT_PROP_STA_CAL_TEST
  NVM_INIT_PROP_STA_CAL_TEST,
#endif
#ifdef NVM_INIT_PROP_STA_CAL
  NVM_INIT_PROP_STA_CAL,
#endif
};
/* Properties for the groups - Fixed */
static T_GROUP_PROPERTIES group_prop_fix[] = {
#ifdef NVM_INIT_PROP_STA_FIX_TEST
  NVM_INIT_PROP_STA_FIX_TEST,
#endif
#ifdef NVM_INIT_PROP_STA_FIX
  NVM_INIT_PROP_STA_FIX,
#endif
};
/* Properties for the groups - Dynamic */
static T_GROUP_PROPERTIES group_prop_dyn[] = {
#ifdef NVM_INIT_PROP_DYN_TEST
  NVM_INIT_PROP_DYN_TEST,
#endif
#ifdef NVM_INIT_PROP_DYN_EXTERN
  NVM_INIT_PROP_DYN_EXTERN,
#endif
#ifdef NVM_INIT_PROP_DYN
  NVM_INIT_PROP_DYN,
#endif
};

/* Array of pointers to group properties */
static T_GROUP_PROPERTIES  *group_prop[NOF_NVM_TYPES] = {&group_prop_cal[0], &group_prop_fix[0], &group_prop_dyn[0]};

struct nvmdev_t {
    struct class        *p_class;
    struct cdev         *p_cdev;
	struct mutex	nmutex;
    wait_queue_head_t   queue;
    unsigned long       flags;
    int                 major;
    dev_t               dev;
};

static struct nvmdev_t *p_nvmdev;

/* nvmdev related stuff */
static T_NVMDEV_GROUP_INFO nvmdev_struct[TOT_NOF_NVM_GROUPS];
static volatile int         nvmdev_update =  0;
static volatile T_NVM_STATE nvmdev_state  =  NVM_IDLE;

/* Device specific global declarations end here. */

/* nvmdev fops */
static struct file_operations fops = {
    .read           = nvmdev_read,                /* Not supported */
    .write          = nvmdev_write,               /* Not supported */
    .open           = nvmdev_open,
#ifdef CONFIG_COMPAT
    .compat_ioctl   = nvmdev_ioctl,
#endif
    .unlocked_ioctl = nvmdev_ioctl,
    .release        = nvmdev_release,
    .owner          = THIS_MODULE
};

/* Definition of Callback function */
typedef void (*T_NVM_CALLBACK_DEFAULT)(T_NVM_SET_FACTORY nvm_set_factory);

/* Array that holds the NVM callback functions for setting default dynamic values */
static T_NVM_CALLBACK_DEFAULT nvm_callback_default_dyn[NOF_NVM_GROUPS_ALL_DYN];

static T_NVM_STATE_NOTIFY nvm_state_notify[TOT_NOF_NVM_GROUPS];
/* Variable used to store the status of notifier thread */
static bool is_notifier_running;

typedef struct
{
  unsigned int nof_int_ext_groups;       /* internal + external groups */
  unsigned int nof_int_ext_found_groups; /* internal + external + unknown groups (unknown means not included in nvm_cfg.h */
} T_NOF_GROUPS;

/* Array that holds the number of groups */
static T_NOF_GROUPS nof_groups_arr[NOF_NVM_TYPES] =    \
                                            {   \
                                              {NOF_NVM_GROUPS_ALL_STA_CAL, 0},  \
                                              {NOF_NVM_GROUPS_ALL_STA_FIX, 0},  \
                                              {NOF_NVM_GROUPS_ALL_DYN,     0}   \
                                            };

/* Caluculate index from group and type index */
static T_NVM_RETURNCODE get_index_from_type_and_group(unsigned int *index, unsigned int type_index, unsigned int group_index)
{
    if (type_index == NVM_TYPE_INDEX_STA_CAL) {
        *index = group_index;
    } else if (type_index == NVM_TYPE_INDEX_STA_FIX) {
        *index = NOF_NVM_GROUPS_ALL_STA_CAL + group_index;
    } else if (type_index == NVM_TYPE_INDEX_DYN) {
        *index = NOF_NVM_GROUPS_ALL_STA_CAL + NOF_NVM_GROUPS_ALL_STA_FIX + group_index;
    } else {
        INFO("Invalid group type %u\n", type_index);
        return NVM_ILLEGAL_GROUP;
    }

    return NVM_OK;
}

/* Get type and group from index */
static T_NVM_RETURNCODE get_type_and_group_from_index(unsigned int index, unsigned int *type_index, unsigned int *group_index)
{
#if NOF_NVM_GROUPS_ALL_STA_CAL
        if (index < NOF_NVM_GROUPS_ALL_STA_CAL) {
            *type_index  = NVM_TYPE_INDEX_STA_CAL;
            *group_index = index;
            goto out;
        }
#endif
#if NOF_NVM_GROUPS_ALL_STA_FIX
        if (index < NOF_NVM_GROUPS_ALL_STA_CAL + NOF_NVM_GROUPS_ALL_STA_FIX) {
            *type_index  = NVM_TYPE_INDEX_STA_FIX;
            *group_index = index - NOF_NVM_GROUPS_ALL_STA_CAL;
            goto out;
        }
#endif
#if NOF_NVM_GROUPS_ALL_DYN
        if (index < NOF_NVM_GROUPS_ALL_DYN + NOF_NVM_GROUPS_ALL_STA_FIX + NOF_NVM_GROUPS_ALL_STA_CAL) {
            *type_index  = NVM_TYPE_INDEX_DYN;
            *group_index = index - (NOF_NVM_GROUPS_ALL_STA_CAL + NOF_NVM_GROUPS_ALL_STA_FIX);
            goto out;
        }
#endif
        ALERT("Invalid Group Index\n");
        return NVM_ILLEGAL_GROUP;

out:
        return NVM_OK;
}

/* Get the index of structure from property table */
static T_NVM_RETURNCODE get_type_and_group_from_groupid(T_NVM_GROUP group_id, unsigned int *p_type_index, unsigned int *p_group_index)
{
    unsigned int i;
    T_GROUP_PROPERTIES *p_group_prop;

    *p_type_index = ((T_NVM_GROUP)(group_id & NVM_MASK_REMOVE_GROUP) >> NVM_TYPE_POS) - 1;
    if (*p_type_index >= NOF_NVM_TYPES)
    {
        ALERT("Invalid group\n");
        return NVM_ILLEGAL_GROUP;
    }

    p_group_prop = group_prop[*p_type_index];
    for (i = 0; i < nof_groups_arr[*p_type_index].nof_int_ext_groups; i++)
    {
        if (group_id == p_group_prop[i].group_id)
        {
            *p_group_index = i;
            return NVM_OK;
        }
    }

    return NVM_ILLEGAL_GROUP;
}

/* API implementation starts here */

static int nvm_read_common(T_NVM_GROUP group_id, unsigned char *data_ptr, int offset, unsigned int no_of_bytes)
{
    int index = 0;
    unsigned int type_index, group_index;

    INFO("-------------------- NVM READ -------------------------\n");
    if (nvmdev_state == NVM_IDLE) {
        INFO(" NVMDEV NOT READY\n");
        INFO("-------------------------------------------------------\n");
        return -EAGAIN;
    }

    offset = offset + sizeof(T_NVM_VER_REV_LGT);

    /*
     * Identify the group index and check ready_to_read flag.
     * If not ready_to_ready; return and ask to retry.
     * Note : ready_to_read flag will set only for one group at a time; the one currently updating (writing).
     */
    if (get_type_and_group_from_groupid(group_id, &type_index, &group_index) == NVM_OK) {
        get_index_from_type_and_group(&index, type_index, group_index);
        if (!(nvmdev_struct[index].ready_to_read)) {
            /* Not ready to read, unlock and return -eagain */
            INFO("  not ready to read\n");
            INFO("  group_id ready_to_read [0x%x][%d]\n", group_id, (nvmdev_struct[index].ready_to_read));
            INFO("-------------------------------------------------------\n");
            return -EAGAIN;
        }

        if (((no_of_bytes + offset) <= ((*(group_prop + type_index) + group_index)->nof_bytes_mirror)) && (offset >= 0)) {
            memcpy(data_ptr, &(*(group_prop + type_index) + group_index)->p_data[offset], no_of_bytes);
        } else {
            /* Invalid parameteres, return error */
            ALERT("  Invalid args offset nof_bytes [%05d] [%05u]\n", offset, no_of_bytes);
            return -EINVAL;
        }
    } else {
        /* Invalid group id, return error */
        ALERT("  Invalid group id [0x%x]\n", group_id);
        return -EINVAL;
    }

    INFO("  group_id nofbytes [0x%x][%05u][%05u]\n", group_id, offset, no_of_bytes);
    INFO("  type_index group_index index nvmdev_update [%02u][%02u][%02u][%d]\n", \
                    type_index, group_index, index, nvmdev_update);
    INFO("-------------------------------------------------------\n");

    return NVM_OK;
}

static int nvm_write_common(T_NVM_GROUP group_id,                       \
                                        unsigned char *data_ptr,        \
                                        int offset,                     \
                                        unsigned int no_of_bytes,       \
                                        T_VERSION version,              \
                                        T_REVISION revision,            \
                                        BOOL write_ver_rev_lgt,         \
                                        BOOL update_flash_from_mirror,  \
                                        BOOL device_initialing)
{
    int index = 0;
    unsigned int type_index, group_index;

    if ((nvmdev_state != NVM_ACTIVE) && (!device_initialing)) {
        return -EAGAIN;
    }

    /* Acquire mutex */
	mutex_lock(&p_nvmdev->nmutex);
    INFO("-------------------- NVM WRITE ------------------------\n");


	/* Waiting for mutex, but state changed.
		release the mutex and return -EAGAIN */
    if ((nvmdev_state != NVM_ACTIVE) && (!device_initialing)) {
        INFO ("  Spinlock going to release, as the state changed\n");
        INFO("-------------------------------------------------------\n");
		mutex_unlock(&p_nvmdev->nmutex);
        return -EAGAIN;
    }

    /* The first part of each group is version + revision */
    offset = offset + sizeof(T_NVM_VER_REV_LGT);

    if ((get_type_and_group_from_groupid(group_id, &type_index, &group_index) == NVM_OK) &&
            (offset >= 0) && ((no_of_bytes + offset) <= ((*(group_prop + type_index) + group_index)->nof_bytes_mirror))) {

            if ((S32)no_of_bytes < 0) {
			mutex_unlock(&p_nvmdev->nmutex);
                return -EINVAL;
            }


            /* TODO: Check for return errors */
            get_index_from_type_and_group(&index, type_index, group_index);
            /* Clear the ready_to_read flag, will block read operation. */
            nvmdev_struct[index].ready_to_read = FALSE;
            /* data update */
            memcpy(&(*(group_prop + type_index) + group_index)->p_data[offset], data_ptr, no_of_bytes);
            /* Write version + revision to the RAM mirror */
            if (write_ver_rev_lgt) {
                INFO("  version revision length [%05u][%05u][%05u]\n", version, revision, \
                    ((*(group_prop + type_index) + group_index)->nof_bytes_mirror) - sizeof(T_NVM_VER_REV_LGT));

                ((T_NVM_VER_REV_LGT*)(*(group_prop + type_index) + group_index)->p_data)->version   = version;
                ((T_NVM_VER_REV_LGT*)(*(group_prop + type_index) + group_index)->p_data)->revision  = revision;
                ((T_NVM_VER_REV_LGT*)(*(group_prop + type_index) + group_index)->p_data)->nof_bytes = \
                                                    ((*(group_prop + type_index) + group_index)->nof_bytes_mirror) - sizeof(T_NVM_VER_REV_LGT);
            }

            /* Set ready to read flag; will unblock read operation. */
            nvmdev_struct[index].ready_to_read = TRUE;
            if (update_flash_from_mirror) {
                nvmdev_struct[index].update_count++;        /* Individual group update count */
            }
    } else {
        /* Invalid group id or length */
        INFO("  Invalid args gid off nof_bytes [0x%x][%05u][%05u]\n", \
                                                            group_id,
                                                            offset,
                                                            no_of_bytes);
        INFO("-------------------------------------------------------\n");
		mutex_unlock(&p_nvmdev->nmutex);
        return -EINVAL;
    }

    if (update_flash_from_mirror) {
        /* will wake up nvm user space daemon and inform total no. of updates */
        nvmdev_update++;
		mutex_unlock(&p_nvmdev->nmutex);
        wake_up_interruptible(&p_nvmdev->queue);
    } else {
            INFO("  group_id offset nofbytes [0x%x][%05u][%05u]\n", group_id, offset, no_of_bytes);
            INFO("  type_index group_index index nvmdev_update [%02u][%02u][%02u][%d]\n", \
                            type_index, group_index, index, nvmdev_update);
            INFO("-------------------------------------------------------\n");

			mutex_unlock(&p_nvmdev->nmutex);
    }

    return NVM_OK;
}

static bool do_notify(unsigned char *flags, bool is_nvm_thread)
{
    unsigned char state_notified = 0;

    /* Check current state notification required. */
    switch(nvmdev_state)
    {
        case NVM_ACTIVE:
            if (!(*flags & NVM_ACTIVE_NOTIFY))
                goto dont_notify;

            state_notified |= NVM_ACTIVE_NOTIFIED;
        break;
        case NVM_DEACTIVATE:
            if (!(*flags & NVM_DEACTIVE_NOTIFY))
                goto dont_notify;

            state_notified |= NVM_DEACTIVE_NOTIFIED;
        break;
        case NVM_IDLE:
        default:
            goto dont_notify;
        break;
    }

    /* If control reaches here, notification required if already not notified */

    /* Check notify always is set */
    if (*flags & NVM_NOTIFY_ALWAYS) {
        if (is_nvm_thread) {
            /* Check state change already notified during registration; if so clear and dont notify */
            if (*flags & state_notified) {
                *flags &= ~(state_notified);
                goto dont_notify;
            }
        } else if (is_notifier_running) {
            /* during registration and nvm_notifier thread is alive */
            *flags |= state_notified;
        }
    } else {
        /* If 'notify always' is not set; clear the current notification flag */
        state_notified >>= NVM_NOTIFIED_SHIFT_LOC;
        *flags &= ~(state_notified);
    }

    return true;
dont_notify:
    return false;
}

static int nvm_notify_users(void *nothing)
{
    int              index;
    unsigned char    flags = 0;

    /* At this moment state is changed to NVM_IDLE; then do nothing */
    if (nvmdev_state == NVM_IDLE)
        goto out;

    is_notifier_running = true;

    for (index = 0; index < TOT_NOF_NVM_GROUPS; index++) {
        if (!(nvm_state_notify[index].p_func) || (!(nvm_state_notify[index].flags)))
            continue;

        flags = nvm_state_notify[index].flags;

        if (do_notify(&flags, true) == false)
            continue;

        nvm_state_notify[index].p_func(nvmdev_state);
        nvm_state_notify[index].flags = flags;
    }

    is_notifier_running = false;

    /* Clearing all the notified bits */
    for (index = 0; index < TOT_NOF_NVM_GROUPS; index++) {
        if (nvm_state_notify[index].flags)
            nvm_state_notify[index].flags &= ~(NVM_ACTIVE_NOTIFIED | NVM_DEACTIVE_NOTIFIED);
    }

out:
    /* Stop the thread and exit */
    do_exit(0);
    return 0;
}


#define NVM_READ_RETRY_COUNT            5
#define NVM_WRITE_RETRY_COUNT           5

#define NVM_READ_RETRY_INTERVAL_MSEC    50
#define NVM_WRITE_RETRY_INTERVAL_MSEC   50

/* Exposed API for NVM-users */
T_NVM_RETURNCODE nvm_read(T_NVM_GROUP groupId, unsigned char *dst, unsigned int offset, unsigned int nofBytes)
{
    int retval          = NVM_OK;
    int retry_count     = NVM_READ_RETRY_COUNT;

    do {
        retval = nvm_read_common(groupId, dst, (int)offset, nofBytes);
        if (retval == -EAGAIN) {
            msleep(NVM_READ_RETRY_INTERVAL_MSEC);
        }
        retry_count--;
    /* Device not yet initialized or on going write to same group (come after some time) */
    } while ((retval == -EAGAIN) && (retry_count));

    if (retval < 0) {
        retval = NVM_ERROR;
    }

    return retval;
}
EXPORT_SYMBOL(nvm_read);

T_NVM_RETURNCODE nvm_write(T_NVM_GROUP groupId, unsigned char *src, unsigned int offset, unsigned int nofBytes)
{
    int retval          = NVM_OK;
    int retry_count     = NVM_WRITE_RETRY_COUNT;

    do {
        retval = nvm_write_common(groupId, src, (int)offset, nofBytes, 0, 0, FALSE, TRUE, FALSE);
        if (retval == -EAGAIN) {
            msleep(NVM_WRITE_RETRY_INTERVAL_MSEC);
        }
        retry_count--;
    /* Device is not yet initialized or on going write to same group (come after some time) */
    } while ((retval == -EAGAIN) && (retry_count));

    if (retval < 0) {
        /* retval contains the error code */
        retval = NVM_ERROR;
    }

    return retval;
}
EXPORT_SYMBOL(nvm_write);

T_NVM_RETURNCODE nvm_init(T_NVM_GROUP groupId, unsigned char *src, unsigned int offset, unsigned int nofBytes, T_VERSION version, T_REVISION revision)
{
    int retval          = NVM_OK;
    int retry_count     = NVM_WRITE_RETRY_COUNT;

    do {
        retval = nvm_write_common(groupId, src, (int)offset, nofBytes, version, revision, TRUE, TRUE, FALSE);
        if (retval == -EAGAIN) {
            msleep(NVM_WRITE_RETRY_INTERVAL_MSEC);
        }
        retry_count--;
    /* Device is not yet initialized or on going write to same group (come after some time) */
    } while ((retval == -EAGAIN) && (retry_count));

    if (retval < 0) {
        retval = NVM_ERROR;
    }

    return retval;
}
EXPORT_SYMBOL(nvm_init);

T_NVM_RETURNCODE nvm_get_ver_rev_lgt(T_NVM_GROUP group_id, T_VERSION *p_version, T_REVISION *p_revision, unsigned int *p_nof_bytes)
{
    T_NVM_VER_REV_LGT     ver_rev_lgt;

    int retval          = NVM_OK;
    int retry_count     = NVM_READ_RETRY_COUNT;

    do {
        /* read the version header from mirror (offset = (0 - sizeof(T_NVM_VER_REV_LGT))) */
        retval = nvm_read_common(group_id, (unsigned char *) &ver_rev_lgt, (0 - sizeof(T_NVM_VER_REV_LGT)), sizeof(T_NVM_VER_REV_LGT));
        if (retval == -EAGAIN) {
            msleep(NVM_READ_RETRY_INTERVAL_MSEC);
        }
        retry_count--;
    } while ((retval == -EAGAIN) && (retry_count));

    if (retval < 0) {
        retval = NVM_ERROR;
    } else {
        *p_version   = ver_rev_lgt.version;
        *p_revision  = ver_rev_lgt.revision;
        *p_nof_bytes = ver_rev_lgt.nof_bytes;
    }

    return retval;
}
EXPORT_SYMBOL(nvm_get_ver_rev_lgt);


T_NVM_RETURNCODE nvm_register_callback_set_default_dynamic(T_NVM_GROUP group_id, void (*p_func)(T_NVM_SET_FACTORY nvm_set_factory))
{
    unsigned int                 type_index;
    unsigned int                 group_index;

    type_index = ((T_NVM_GROUP)(group_id & NVM_MASK_REMOVE_GROUP) >> NVM_TYPE_POS) - 1;

    if (type_index < NVM_TYPE_INDEX_DYN) {
        return NVM_ILLEGAL_GROUP;
    }

    get_type_and_group_from_groupid(group_id, &type_index, &group_index);
    nvm_callback_default_dyn[group_index] = p_func;

    return NVM_OK;
}
EXPORT_SYMBOL(nvm_register_callback_set_default_dynamic);

T_NVM_RETURNCODE nvm_register_state_change_notify(T_NVM_GROUP group_id, void (*p_func)(T_NVM_STATE nvmdev_state), unsigned char flags)
{
    unsigned int        type_index, group_index, index;

    /* Nothing to be done */
    if ((!p_func) || (!flags))
        goto out;

    if (get_type_and_group_from_groupid(group_id, &type_index, &group_index) != NVM_OK)
        return NVM_ILLEGAL_GROUP;

    if (get_index_from_type_and_group(&index, type_index, group_index) != NVM_OK)
        return NVM_ILLEGAL_GROUP;

    nvm_state_notify[index].flags     = 0;
    nvm_state_notify[index].p_func    = p_func;

    if (nvmdev_state != NVM_IDLE) {
        /* Check notification required */
        if (do_notify(&flags, false)) {
            p_func(nvmdev_state);
        }
    }

    nvm_state_notify[index].flags = flags;
out:
    return NVM_OK;
}
EXPORT_SYMBOL(nvm_register_state_change_notify);

T_NVM_RETURNCODE nvm_unregister_state_change_notify(T_NVM_GROUP group_id)
{
    unsigned int        type_index, group_index, index;

    if (get_type_and_group_from_groupid(group_id, &type_index, &group_index) != NVM_OK)
        return NVM_ILLEGAL_GROUP;

    if (get_index_from_type_and_group(&index, type_index, group_index) != NVM_OK)
        return NVM_ILLEGAL_GROUP;

    nvm_state_notify[index].p_func   = NULL;
    nvm_state_notify[index].flags    = 0;

    return NVM_OK;
}
EXPORT_SYMBOL(nvm_unregister_state_change_notify);
/* API implementation ends here */

/* fops definitions */

static long nvmdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    unsigned int group_index        = 0;
    unsigned int type_index         = 0;
    int          index              = 0;
    int          ua_index           = 0;    /* user-agent index */
    int          retval             = 0;
    int          retry_count        = NVM_WRITE_RETRY_COUNT;
    BOOL         write_ver_rev_lgt  = 0;

    struct task_struct      *nvm_notifier;

    T_NVMDEV_IOCTL          *nvmdev_ioctl;
    T_NVM_SET_FACTORY       nvm_set_factory;

    switch(cmd)
    {
        /***************************** NVM_UA_WRITE_TO_FLASH **************************************/
        /*
         * NVM User Agent block until atleast one nvm-user write.
         *  1. Check the nvm device state. If the state is not ready return error.
         *  2. Wait for event. untill (nvmdev_update > 0) becomes true.
			3. Acquire mutex.
         *      1. Typecast the argument received (pointer to nvmdev_ioctl expected).
         *      2. Traverse through the nvm groups, until update count is zero.
         *      3. Check for update count of each group. If the group is updated copy it to user space.
			4. Release mutex.
         */
        case NVM_UA_WRITE_TO_FLASH:
            if (nvmdev_state != NVM_ACTIVE) {
                INFO("--------------- NVM_UA_WRITE_TO_FLASH -----------------\n");
                INFO("  device not initialized\n");
                INFO("-------------------------------------------------------\n");
                return -EAGAIN;
            }

            /* Only block if no more pending updates; if updates are pending go and process it */
            if (nvmdev_update == 0) {
                /* Will send event, once 'nvmdev_update' is updated */
                wait_event_interruptible(p_nvmdev->queue, (nvmdev_update > 0));
            }

			/* Acquire mutex */
			mutex_lock(&p_nvmdev->nmutex);
            INFO("---------------- NVM_UA_WRITE_TO_FLASH -----------------\n");
            /* critical region (semaphore acquired) ... */
            nvmdev_ioctl = (T_NVMDEV_IOCTL *)arg;
            index    = 0;
            ua_index = 0;
            /* identify the updated groups and read it */
            do {
                INFO("  index [%02d] nvmdev_update [%02d] ua_index[%d]\n", index, nvmdev_update, ua_index);
                if (nvmdev_struct[index].update_count != 0) {
			int ret;

                    /* Identify the type index and group index from nvmdev_struct index */
                    get_type_and_group_from_index(index, &type_index, &group_index);

                    /* Copy the group id and data to user-agent */
                    (nvmdev_ioctl + ua_index)->group_id    = ((*(group_prop + type_index) + group_index)->group_id);
                    (nvmdev_ioctl + ua_index)->no_of_bytes = ((*(group_prop + type_index) + group_index)->nof_bytes_mirror) - sizeof(T_NVM_VER_REV_LGT);

                    ret = copy_to_user((nvmdev_ioctl + ua_index)->data_ptr,                                                         \
                                        (&((*(group_prop + type_index) + group_index)->p_data[sizeof(T_NVM_VER_REV_LGT)])),   \
                                                (((*(group_prop + type_index) + group_index)->nof_bytes_mirror) - sizeof(T_NVM_VER_REV_LGT)));
			BUG_ON(ret);
                    (nvmdev_ioctl + ua_index)->version  = ((T_NVM_VER_REV_LGT*)(*(group_prop + type_index) + group_index)->p_data)->version;
                    (nvmdev_ioctl + ua_index)->revision = ((T_NVM_VER_REV_LGT*)(*(group_prop + type_index) + group_index)->p_data)->revision;
                    /* length can be ignore (as we have nof_bytes) */
                    (nvmdev_ioctl + ua_index)->length   = ((T_NVM_VER_REV_LGT*)(*(group_prop + type_index) + group_index)->p_data)->nof_bytes - \
                                                                                                                    sizeof(T_NVM_VER_REV_LGT);
                    (nvmdev_ioctl + ua_index)->valid = 1;
                    /* Update count processing */
                    INFO("  group_id update_count [0x%x][%05u]\n", ((*(group_prop + type_index) + group_index)->group_id), \
                                                                   nvmdev_struct[index].update_count);
                    nvmdev_update = nvmdev_update - (nvmdev_struct[index].update_count);
                    nvmdev_struct[index].update_count = 0; /* update cleared */
                    ua_index++;
                }
                index++;
            /* Until nvmdev_update == 0 OR it reaches the end of group. or reaches update_limit */
            } while ((nvmdev_update != 0) && (index < TOT_NOF_NVM_GROUPS) && (ua_index < UPDATE_LIMIT));
            INFO("-------------------------------------------------------\n");
			mutex_unlock(&p_nvmdev->nmutex);
            break;
        /***************************** NVM_KERNEL_MIRROR_UPDATE **********************************/
        /* Which will be triggered from user-agent.
         * and update the required groups at runtime.
         */
         /* 1. Check the device state is initialized
			2. Acquire the mutex.
          *     1. Copy the argument to ioctl structure.
          *     2. Identify the type_index and group index.
          *     3. Identift the index. ready_to_read is based on this index.
          *     4. Update the data from user space to kernel mirror.
          *     5. Change ready to read flag (TODO)
			3. Release the mutex
          */
        /* Dont touch the update count as this can only happen from User space */
        case NVM_COPY_TO_KERNEL_MIRROR:
        case NVM_KERNEL_MIRROR_UPDATE:
            INFO("-------------- NVM_COPY_TO_KERNEL_MIRROR --------------\n");

            nvmdev_ioctl = (T_NVMDEV_IOCTL *)arg;

            INFO("gid off nof ver rev len [0x%x][%05u][%05u][%05u][%05u][%05u]\n",      \
                                                (nvmdev_ioctl->group_id),               \
                                                (int)(nvmdev_ioctl->offset),            \
                                                (nvmdev_ioctl->no_of_bytes),            \
                                                (nvmdev_ioctl->version),                \
                                                (nvmdev_ioctl->revision),               \
                                                (nvmdev_ioctl->length));

            /* Check version, revision has to update or not */
            /* For normal GTI write; version, revision and length will be 0xFF */
            if (((nvmdev_ioctl->version) != (unsigned int)0xFFFFFFFF) || \
                    ((nvmdev_ioctl->revision) != (unsigned int)0xFFFFFFFF)) {
                write_ver_rev_lgt = TRUE;
            } else {
                write_ver_rev_lgt = FALSE;
            }

            do {
                retval = nvm_write_common((nvmdev_ioctl->group_id),         \
                                          (nvmdev_ioctl->data_ptr),         \
                                          (int)(nvmdev_ioctl->offset),      \
                                          (nvmdev_ioctl->no_of_bytes),      \
                                          (nvmdev_ioctl->version),          \
                                          (nvmdev_ioctl->revision),         \
                                          write_ver_rev_lgt,                \
                                          FALSE,                            \
                                          nvmdev_ioctl->device_initialing);
                if (retval == -EAGAIN) {
                    msleep(NVM_WRITE_RETRY_INTERVAL_MSEC);
                }
                retry_count--;
                /* Device is not yet initialized or on going write to same group (come after some time) */
            } while ((retval == -EAGAIN) && (retry_count));

            INFO("-------------------------------------------------------\n");
            return retval;
            break;
        case NVM_FACTORY_DEFAULT:
            nvm_set_factory = (T_NVM_SET_FACTORY)arg;
            /* Will be called when a factory reset trigered in user-space */
            for (index = 0; index < NOF_NVM_GROUPS_ALL_DYN; index++) {
                if (nvm_callback_default_dyn[index] != 0) {
                    nvm_callback_default_dyn[index](nvm_set_factory);
                }
            }

            return NVM_OK;
            break;
        /* This will be called from User space to change the kernel driver state */
        case NVM_CHANGE_DEV_STATE:
            INFO("----------------- NVM_CHANGE_DEV_STATE ----------------\n");
            if (nvmdev_state == (T_NVM_STATE)arg) {
                /* No need to change the state */
                INFO("  state changed to [%05d]\n", nvmdev_state);
                INFO("-------------------------------------------------------\n");
                return NVM_OK;
            }

            /* Before changing the state to deactivate or idle; Make sure no pending updates */
            if (((T_NVM_STATE)arg == NVM_DEACTIVATE) || ((T_NVM_STATE)arg == NVM_IDLE)) {
                if (nvmdev_update > 0) {
                    /* Pending updates */
                    wake_up_interruptible(&p_nvmdev->queue);
                }
                INFO("  pending updates  [%05d]\n", nvmdev_update);
            }

			mutex_lock(&p_nvmdev->nmutex);
            /* Change the state */
            nvmdev_state = (T_NVM_STATE)arg;
            INFO("  state changed to [%05d]\n", nvmdev_state);
            INFO("-------------------------------------------------------\n");
			mutex_unlock(&p_nvmdev->nmutex);

            if (nvmdev_state != NVM_IDLE) {
                /* Spawn a thread to notify the users about state change */
                nvm_notifier = kthread_run(nvm_notify_users, NULL, "nvm-notifier");
                if (IS_ERR(nvm_notifier)) {
                    printk(KERN_ERR "Error during thread creation : %ld\n", PTR_ERR(nvm_notifier));
                    nvm_notifier = NULL;
                }
            }

            break;
        /***************************** NVM_DEBUG ****************************************/
        case NVM_DEBUG:
            if (nvmdev_state != NVM_ACTIVE) {
                INFO("NVMDEV NOT READY %d\n", -EAGAIN);
                return -EAGAIN; /* or ENOTREADY */
            }

			mutex_lock(&p_nvmdev->nmutex);
            if (nvmdev_state != NVM_ACTIVE) {
                INFO("Spinlock going to release, as the state changed\n");
				mutex_unlock(&p_nvmdev->nmutex);
                return -EAGAIN;
            }

            INFO("----------------------------------------\n");
            for (index = 0; index < TOT_NOF_NVM_GROUPS; index++) {
                get_type_and_group_from_index(index, &type_index, &group_index);
                INFO("Group ID       : 0x%x\n", ((*(group_prop + type_index) + group_index)->group_id));
                INFO("No. of bytes   : %u\n"  , ((*(group_prop + type_index) + group_index)->nof_bytes_mirror));
                INFO("Data ptr       : %p\n"  , ((*(group_prop + type_index) + group_index)->p_data));
                INFO("Ready to read  : %d\n"  , nvmdev_struct[index].ready_to_read);
                INFO("----------------------------------------\n");
            }

			mutex_unlock(&p_nvmdev->nmutex);
        default:
            return -EINVAL;
            break;
    }

    return NVM_OK;
}

/* Initialize kernel mirrors and fill it with 0xFF */
static int nvm_kernel_mirror_fill_default(void)
{
    int                      index       = 0;
    unsigned int             type_index  = 0;
    unsigned int             group_index = 0;

#if defined (NVM_LINUX_DEBUG)
    unsigned int             max_limit   = 0;
#endif

    for (index = 0; index < TOT_NOF_NVM_GROUPS; index++) {
        get_type_and_group_from_index(index, &type_index, &group_index);

        /* Check the ioctl structure has enough room to accomadate any group */
        if (((*(group_prop + type_index) + group_index)->nof_bytes_mirror) > NVM_MAX_GROUP_SIZE) {
            /* trap */
            ALERT("  group_id [0x%x] size is greater than transfer buffer size\n", \
                                            (*(group_prop + type_index) + group_index)->group_id);
            return -ENOMEM;
        }
        memset(((*(group_prop + type_index) + group_index)->p_data), 0xFF, ((*(group_prop + type_index) + group_index)->nof_bytes_mirror));
    }

    /* Flush the factory default callback pointer array */
    memset(nvm_callback_default_dyn, 0, sizeof(nvm_callback_default_dyn));
    /* Flush the state change notify callback pointer array */
    memset(nvm_state_notify, 0, sizeof(nvm_state_notify));

#if defined (NVM_LINUX_DEBUG)
    INFO("----------------------------------------------------------------------\n");
    INFO("Total no of groups for test : %d\n", TOT_NOF_NVM_GROUPS);
    INFO("Total no of cal groups      : %d\n", NOF_NVM_GROUPS_ALL_STA_CAL);
    INFO("Total no of fix groups      : %d\n", NOF_NVM_GROUPS_ALL_STA_FIX);
    INFO("Total no of dyn groups      : %d\n", NOF_NVM_GROUPS_ALL_DYN);

    INFO("Total no of types           : %d\n", NOF_NVM_TYPES);
    /* Cal, Fix, Dyn groups all need to display */

    for (type_index = 0; type_index < NOF_NVM_TYPES; type_index++) {
        if      (type_index  == NVM_TYPE_INDEX_STA_CAL)  max_limit = NOF_NVM_GROUPS_ALL_STA_CAL;
        else if (type_index  == NVM_TYPE_INDEX_STA_FIX)  max_limit = NOF_NVM_GROUPS_ALL_STA_FIX;
        else if (type_index  == NVM_TYPE_INDEX_DYN)      max_limit = NOF_NVM_GROUPS_ALL_DYN;
        else INFO ("Unknown type %u\n", type_index);

        INFO("----------------------------------------------------------------------\n");
        INFO("Testing get_index_from_type_and_group\n");
        for (group_index = 0; group_index < max_limit; group_index++) {
            get_index_from_type_and_group(&index, type_index, group_index);
            INFO(" [type][group] [%d][%d] max [%u] -> Group ID : 0x%x\n", type_index,  \
                                                                          group_index, \
                                                                          max_limit,   \
                                                                         ((*(group_prop + type_index) + group_index)->group_id));
        }
    }

    INFO("----------------------------------------------------------------------\n");
    INFO("Testing get_type_and_group_from_index\n");
    for (index = 0; index < TOT_NOF_NVM_GROUPS; index++) {
        get_type_and_group_from_index(index, &type_index, &group_index);
        INFO(" Index [%02d] [%u][%u] -> Group ID : 0x%x\n", index,        \
                                                            type_index,   \
                                                            group_index,  \
                                                            ((*(group_prop + type_index) + group_index)->group_id));
    }

    INFO("----------------------------------------------------------------------\n");
#endif /* NVM_LINUX_DEBUG */
    return 0;
}

static ssize_t nvmdev_read(struct file *filp, char *buf, size_t count, loff_t *offp)
{
    return -EINVAL;
}

static ssize_t nvmdev_write(struct file *filp, const char *buf, size_t count, loff_t *offp)
{
    return -EINVAL;
}

static int nvmdev_release(struct inode *inodp, struct file *filp)
{
    module_put(THIS_MODULE);
    return NVM_OK;
}

static int nvmdev_open(struct inode *inodp, struct file *filp)
{
    if (!try_module_get(THIS_MODULE))
        return -EBUSY;

	mutex_lock(&p_nvmdev->nmutex);
    /* critical region ... */
	mutex_unlock(&p_nvmdev->nmutex);

    return NVM_OK;
}

/* Module init and exit functions */
static int __init nvmdev_init(void)
{
    struct device *dev = NULL;
    int ret;

    p_nvmdev = (struct nvmdev_t *)kzalloc(sizeof(*p_nvmdev), GFP_KERNEL);
    if (!p_nvmdev) {
        ret = -ENOMEM;
        goto err;
    }

    p_nvmdev->dev = MKDEV(0, 0);
    p_nvmdev->p_cdev = cdev_alloc();
    if (!p_nvmdev->p_cdev) {
        ret = -ENOMEM;
        goto err;
    }

    /* Register a range of char device numbers */
    ret = alloc_chrdev_region(&p_nvmdev->dev, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        INFO("alloc_chrdev_region failed\n");
        goto err_cdev;
    }

    p_nvmdev->major = MAJOR(p_nvmdev->dev);

    INFO("Registered %d \n", p_nvmdev->major);

    /* Initialize a cdev structure */
    cdev_init(p_nvmdev->p_cdev, &fops);
    p_nvmdev->p_cdev->ops   = &fops;
    p_nvmdev->p_cdev->owner = THIS_MODULE;

    ret = cdev_add(p_nvmdev->p_cdev, p_nvmdev->dev, 1);
    if (ret < 0) {
        INFO("Unable to allocate cdev\n");
        goto err_chrdev;
    }

    /* Create a struct class structure */
    p_nvmdev->p_class = class_create(THIS_MODULE, DEVICE_NAME);
    if (!p_nvmdev->p_class) {
        INFO("Unable to create class struct\n");
        goto err_chrdev;
    }

    /* Creates a device and registers it with sysfs */
    dev = device_create(p_nvmdev->p_class, NULL, MKDEV(p_nvmdev->major, 0), "%s", DEVICE_NAME);
    if (IS_ERR(dev)) {
        INFO("Unable to create device\n");
        ret = PTR_ERR(dev);
        goto err_class;
    }

    init_waitqueue_head(&p_nvmdev->queue);
	mutex_init(&p_nvmdev->nmutex);

    nvmdev_state = NVM_IDLE;
    /* initialize nvmdev list */
    ret = nvm_kernel_mirror_fill_default();
    if (ret < 0) {
        INFO("NVM mirror fill with default failed!\n");
        goto err_class;
    }

    return 0;

err_class:
    if (!IS_ERR(dev))
        device_destroy(p_nvmdev->p_class, MKDEV(p_nvmdev->major, 0));

    class_unregister(p_nvmdev->p_class);
    class_destroy(p_nvmdev->p_class);
err_chrdev:
    unregister_chrdev_region(p_nvmdev->dev, 1);
err_cdev:
    cdev_del(p_nvmdev->p_cdev);
err:
    if (p_nvmdev)
        kfree(p_nvmdev);

    return ret;
}

static void __exit nvmdev_exit(void)
{
    INFO("Un Registered\n");

    cdev_del(p_nvmdev->p_cdev);
    unregister_chrdev_region(p_nvmdev->dev, 1);
    device_destroy(p_nvmdev->p_class, MKDEV(p_nvmdev->major, 0));
    class_unregister(p_nvmdev->p_class);
    class_destroy(p_nvmdev->p_class);
    kfree(p_nvmdev);
}

module_init(nvmdev_init);
module_exit(nvmdev_exit);

MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
