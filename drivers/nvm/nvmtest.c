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

#include <linux/kernel.h>       // Needed for KERN_INFO
#include <linux/init.h>         // Needed for macros
#include <linux/module.h>       // Needed by all the modules
#include <linux/syscalls.h>
#include <linux/fcntl.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>       // for msleep
#include <linux/spinlock.h>

#include <asm/uaccess.h>

#include <nvm/nvm.h>

#define INFO(...)       printk(KERN_INFO  "[nvmtest driver] " __VA_ARGS__);

#define IOC_MAGIC       't'
/* ioctl commands */
#define NVM_TEST_READ                       _IOW(IOC_MAGIC, 1, int)
#define NVM_TEST_WRITE                      _IOW(IOC_MAGIC, 2, int)
#define NVM_TEST_GET_VRL                    _IOW(IOC_MAGIC, 3, int)
#define NVM_TEST_INIT                       _IOW(IOC_MAGIC, 4, int)
#define NVM_TEST_OFFSET_WRITE               _IOW(IOC_MAGIC, 5, int)
#define NVM_TEST_OFFSET_READ                _IOW(IOC_MAGIC, 6, int)
#define NVM_TEST_STATE_NOTIFY_CB            _IOW(IOC_MAGIC, 7, int)


#define DEVICE_NAME         "nvmgtest"
#define DATA_MAX_SIZ        20032

#define DECLARE_NVM_STATE_CHANGE_NOTIFCATION_CB(_GROUP_ID_) \
    void nvm_state_change_notification_##_GROUP_ID_ (T_NVM_STATE nvmdev_state)\
    {\
        switch (nvmdev_state) { \
            case NVM_ACTIVE: \
                /* do something */ \
            break; \
            case NVM_DEACTIVATE : \
                /* do something */ \
            break; \
            default : \
                /* Unknown state, do nothing */ \
            break; \
        } \
        INFO("Group ID 0x%x state : %d\n", _GROUP_ID_, nvmdev_state);\
        return ;\
    }

DECLARE_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_CAL_GROUP_5)
DECLARE_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_CAL_GROUP_7)
DECLARE_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_FIX_GROUP_4)
DECLARE_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_FIX_GROUP_2)
DECLARE_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_DYN_GROUP_4)
DECLARE_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_DYN_GROUP_5)
DECLARE_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_FIX_GROUP_6)
DECLARE_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_CAL_GROUP_0)
DECLARE_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_CAL_GROUP_4)

#define P_FUNC_NVM_STATE_CHANGE_NOTIFCATION_CB(_GROUP_ID_) nvm_state_change_notification_##_GROUP_ID_

/* nvmtest device related stuff */
static struct cdev  *nvmtest_cdev;
static struct class *nvmtest_class;

static int major;
static dev_t dev = MKDEV(0, 0);

typedef struct {
    T_NVM_GROUP               group_id;
    U8                        data[DATA_MAX_SIZ];
    U32                       offset;
    U32                       nof_bytes;
    U32                       version;
    U32                       revision;
    U32                       test_offset;
    U32                       test_nof_bytes;
    U8                        w_pattern;            /* ASCII value of the pattern to write */
    U8                        i_pattern;            /* Used for init */
    U8                        g_pattern;            /* Used for gti */
    U8                        b_pattern;
} T_NVM_TEST_IOCTL;


static ssize_t nvmtest_read(struct file *filp, char *buf, size_t count, loff_t *offp)
{   return -EINVAL; }

static ssize_t nvmtest_write(struct file *filp, const char *buf, size_t count, loff_t *offp)
{   return -EINVAL; }

static int nvmtest_release(struct inode *inodp, struct file *filp)
{
    module_put(THIS_MODULE);
    return 0;
}

static int nvmtest_open(struct inode *inodp, struct file *filp)
{
    if (!try_module_get(THIS_MODULE))
        return -EBUSY;
    return 0;
}

unsigned char temp[DATA_MAX_SIZ];

/* NVM state change notify cb */

static long nvmtest_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{

    T_NVM_TEST_IOCTL          *p_nvm_test_ioctl;
    int                       index;
    unsigned int              nof_bytes;
    T_NVM_RETURNCODE          retval = NVM_ERROR;

    switch(cmd) {
        case NVM_TEST_READ:
            p_nvm_test_ioctl = (T_NVM_TEST_IOCTL *)arg;
            /* Call the nvm_read api */
            retval = nvm_read(p_nvm_test_ioctl->group_id,           \
                                p_nvm_test_ioctl->data,             \
                                p_nvm_test_ioctl->offset,           \
                                p_nvm_test_ioctl->nof_bytes);

            break;
        case NVM_TEST_WRITE:
            p_nvm_test_ioctl = (T_NVM_TEST_IOCTL *)arg;
            for (index = 0; index < p_nvm_test_ioctl->nof_bytes; index++) {
                p_nvm_test_ioctl->data[index] = (U8)((p_nvm_test_ioctl->w_pattern) + index);
            }
            /* Call the nvm_write api */
            retval = nvm_write(p_nvm_test_ioctl->group_id,              \
                                            p_nvm_test_ioctl->data,     \
                                            p_nvm_test_ioctl->offset,   \
                                            p_nvm_test_ioctl->nof_bytes);
            break;
        case NVM_TEST_GET_VRL:
            p_nvm_test_ioctl = (T_NVM_TEST_IOCTL *)arg;
            /* Call the nvm_get_ver_rev_lgt */
            retval = nvm_get_ver_rev_lgt(p_nvm_test_ioctl->group_id,     \
                                            &p_nvm_test_ioctl->version,  \
                                            &p_nvm_test_ioctl->revision, \
                                            &p_nvm_test_ioctl->nof_bytes);

            break;
        case NVM_TEST_INIT:
            p_nvm_test_ioctl = (T_NVM_TEST_IOCTL *)arg;
            /* Prepare data packet */
            for (index = 0; index < p_nvm_test_ioctl->nof_bytes; index++) {
                p_nvm_test_ioctl->data[index] = (U8)((p_nvm_test_ioctl->i_pattern) + index);
            }

            /* Invoke nvm_init */
            retval = nvm_init(p_nvm_test_ioctl->group_id,           \
                                    p_nvm_test_ioctl->data,         \
                                    p_nvm_test_ioctl->offset,       \
                                    p_nvm_test_ioctl->nof_bytes,    \
                                    p_nvm_test_ioctl->version,      \
                                    p_nvm_test_ioctl->revision);
            break;
        case NVM_TEST_OFFSET_WRITE:
            INFO("NVM_TEST_OFFSET_WRITE");
            p_nvm_test_ioctl = (T_NVM_TEST_IOCTL *)arg;

            /* Flush the mirror with 0xFF */
            memset(p_nvm_test_ioctl->data, 0xFF, p_nvm_test_ioctl->nof_bytes);
            retval = nvm_write(p_nvm_test_ioctl->group_id,  \
                                p_nvm_test_ioctl->data,     \
                                0,                          \
                                p_nvm_test_ioctl->nof_bytes);

            INFO(" gid offset nof_bytes [0x%x][%05u][%05u]\n",  \
                                p_nvm_test_ioctl->group_id,     \
                                p_nvm_test_ioctl->test_offset,  \
                                p_nvm_test_ioctl->test_nof_bytes);

            if (p_nvm_test_ioctl->test_nof_bytes > DATA_MAX_SIZ) {
                nof_bytes = p_nvm_test_ioctl->nof_bytes;
            } else {
                nof_bytes = p_nvm_test_ioctl->test_nof_bytes;
            }
            /* Prepare data packet */
            for (index = 0; index < nof_bytes; index++) {
                    p_nvm_test_ioctl->data[index] = (U8)((p_nvm_test_ioctl->w_pattern) + index);
            }

            /* Write the offset data to mirror */
            retval = nvm_write(p_nvm_test_ioctl->group_id,      \
                                p_nvm_test_ioctl->data,         \
                                p_nvm_test_ioctl->test_offset,  \
                                p_nvm_test_ioctl->test_nof_bytes);
            INFO(" retval [%d]\n", retval);
            break;
        case NVM_TEST_OFFSET_READ:
            INFO("NVM_TEST_OFFSET_READ");
            break;
        case NVM_TEST_STATE_NOTIFY_CB:
            INFO("NVM_TEST_STATE_NOTIFY");
            break;
        default:
            break;
    }

    return retval;
}

static struct file_operations fops = {
    .read           = nvmtest_read,
    .write          = nvmtest_write,
    .unlocked_ioctl = nvmtest_ioctl,
    .open           = nvmtest_open,
    .release        = nvmtest_release,
    .owner          = THIS_MODULE
};

/* Register nvm state change nofity callback for all the test groups */
static int register_nvm_state_change_notify_cb(void)
{
    T_NVM_RETURNCODE retval = NVM_OK;

    retval  = nvm_register_state_change_notify(NVM_STA_CAL_GROUP_5, P_FUNC_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_CAL_GROUP_5), NVM_NOTIFY_ALL);
    retval |= nvm_register_state_change_notify(NVM_STA_CAL_GROUP_7, P_FUNC_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_CAL_GROUP_7), NVM_NOTIFY_ALL);
    retval |= nvm_register_state_change_notify(NVM_STA_FIX_GROUP_4, P_FUNC_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_FIX_GROUP_4), NVM_NOTIFY_ALL);
    retval |= nvm_register_state_change_notify(NVM_STA_FIX_GROUP_2, P_FUNC_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_FIX_GROUP_2), NVM_NOTIFY_ALL);
    retval |= nvm_register_state_change_notify(NVM_DYN_GROUP_4,     P_FUNC_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_DYN_GROUP_4),     NVM_NOTIFY_ALL);
    retval |= nvm_register_state_change_notify(NVM_DYN_GROUP_5,     P_FUNC_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_DYN_GROUP_5),     NVM_NOTIFY_ALL);
    retval |= nvm_register_state_change_notify(NVM_STA_FIX_GROUP_6, P_FUNC_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_FIX_GROUP_6), NVM_NOTIFY_ALL);
    retval |= nvm_register_state_change_notify(NVM_STA_CAL_GROUP_0, P_FUNC_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_CAL_GROUP_0), NVM_NOTIFY_ALL);
    retval |= nvm_register_state_change_notify(NVM_STA_CAL_GROUP_4, P_FUNC_NVM_STATE_CHANGE_NOTIFCATION_CB(NVM_STA_CAL_GROUP_4), NVM_NOTIFY_ALL);

    if (retval != NVM_OK)
        INFO("nvm_register_state_change_notify() failed retval = %d\n", retval);

    return retval;
}

static int __init nvmtest_init(void)
{
    int ret;

    nvmtest_cdev = cdev_alloc();

    alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
    major = MAJOR(dev);

    INFO("Registered %d \n", major);

    cdev_init(nvmtest_cdev, &fops);
    nvmtest_cdev->ops     = &fops;
    nvmtest_cdev->owner   = THIS_MODULE;
    ret = cdev_add(nvmtest_cdev, dev, 1);
    if (ret < 0) {
        printk(KERN_INFO "Unable to allocate cdev\n");
        return ret;
    }

    /* create a struct class structure */
    nvmtest_class = class_create(THIS_MODULE, DEVICE_NAME);
    /* creates a device and registers it with sysfs */
    device_create(nvmtest_class, NULL, MKDEV(major,0), "%s", DEVICE_NAME);

    /* Register nvm state change notify callbacks */
    if (register_nvm_state_change_notify_cb() == NVM_OK)
        INFO("Successfully registered the state change notification callbacks\n");

    return 0;
}

static void __exit nvmtest_exit(void)
{
    cdev_del(nvmtest_cdev);
    unregister_chrdev_region(dev, 1);
    device_destroy(nvmtest_class, MKDEV(major, 0));
    class_unregister(nvmtest_class);
    class_destroy(nvmtest_class);

    return;
}

MODULE_LICENSE("GPL");
module_init(nvmtest_init);
module_exit(nvmtest_exit);
