/*
 * Copyright (C) 2013-2014 Intel Mobile Communications GmbH
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/kfifo.h>
#include <linux/uaccess.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/uio.h>

MODULE_LICENSE("GPL v2");

#ifdef CONFIG_FMDEV_VERBOSE
#define FMDEV_DEBUG(...) FMDEV_DEBUG(__VA_ARGS__)
#else /* CONFIG_FMDEV_VERBOSE */
#define FMDEV_DEBUG(...)
#endif /* CONFIG_FMDEV_VERBOSE */

/* Structure to store information for one kfifo inside fmdev. */
struct fmdev_fifo_t {
	wait_queue_head_t inq;
	wait_queue_head_t outq;
	struct kfifo fifo;
	struct semaphore sem;
};

/**
 * Structure type to keep track internal state of fmdev.
 */
struct fmdev_info_t {
	struct mutex fmdev_mutex; /**< Mutex to protect the rest of this
		structure. */
	const struct file_operations fops;  /**< File operations structure.
		Populated with pointers to the system call
		function implementations for fmdev. */
	dev_t dev;                    /**< Device number of the fmdev
		character device. */
	struct cdev fmdev_cdev;       /**< Kernel character device structure. */
	struct class *fmdev_class;    /**< Class structure of the fmdev
		character device. */
	struct device *fmdev_device;  /**< Device structure of the fmdev
		character device. */
	mode_t f_mode;                /**< Mode bitfield to keep track of
		the openings of the fmdev character device file. */
	struct fmdev_fifo_t u2k_fifo;        /**< Fifo for transferring data
		from User Space to Kernel Space. */
	struct fmdev_fifo_t k2u_fifo;        /**< Fifo for transferring data
		from Kernel Space to User Space. */
};


static int fmdev_open(struct inode *inode, struct file *filp);
static int fmdev_close(struct inode *inode, struct file *filp);
static ssize_t fmdev_read(struct file *filp, char __user *buffer,
	size_t length, loff_t *offset);
static ssize_t fmdev_write(struct file *filp, const char __user *buffer,
	size_t length, loff_t *offset);

/*
 * Global State of the fmdev module
 */
static struct fmdev_info_t fmdev_info = {
	.fops.owner = THIS_MODULE,
	.fops.read = fmdev_read,
	.fops.write = fmdev_write,
	.fops.open = fmdev_open,
	.fops.release = fmdev_close,
};
/**
 * Initialize a struct fmdev_fifo_t structure.
 */
static int fmdev_fifo_init(struct fmdev_fifo_t *fifo, size_t size)
{
	int retval = 0;

	if (IS_ERR(fifo))
		retval = PTR_ERR(fifo);
	else if (size == 0)
		retval = -EINVAL;
	else {
		init_waitqueue_head(&(fifo->inq));
		init_waitqueue_head(&(fifo->outq));
		retval = kfifo_alloc(&(fifo->fifo), size, GFP_KERNEL);
		sema_init(&(fifo->sem), 1);
	}

	return retval;
}

/**
 * Free any kernel resources associated with a struct fmdev_fifo_t structure.
 */
static void fmdev_fifo_destroy(struct fmdev_fifo_t *fifo)
{
	if (!IS_ERR(fifo))
		kfifo_free(&(fifo->fifo));
}

/**
 * Common function for all fifo IO. Provides blocking functionality.
 */
static int fmdev_fifo_access(struct fmdev_fifo_t *fifo, char *buf,
	size_t count, unsigned int *copied, bool user,
	bool dir_in, bool block)
{
	int retval;
	bool cond;

	FMDEV_DEBUG("fmdev_fifo_access(user: %u, dir_in: %u)\n", user, dir_in);

	if (IS_ERR(fifo))
		return PTR_ERR(fifo);
	else if (count > 0 && IS_ERR(buf))
		return PTR_ERR(fifo);
	else if (count == 0)
		return 0;
	else if (IS_ERR(copied))
		return PTR_ERR(copied);
	else if (down_interruptible(&(fifo->sem)))
		return -ERESTARTSYS;

	cond = (dir_in) ?
		kfifo_is_full(&(fifo->fifo)) :
		kfifo_is_empty(&(fifo->fifo));
	while (cond) { /* nothing to read */
		up(&(fifo->sem)); /* release the lock */

		/* Break out if we were called in non-blocking mode. */
		if (!block)
			return -EAGAIN;

		FMDEV_DEBUG(
		"fmdev_fifo_access(user: %u, dir_in: %u):Going to sleep.\n",
		user, dir_in);

		/* Put current thread to sleep until kfifo is not full
			or empty (based on direction). */
		if (dir_in)
			retval = wait_event_interruptible(fifo->inq,
				!kfifo_is_full(&(fifo->fifo)));
		else
			retval = wait_event_interruptible(fifo->outq,
				!kfifo_is_empty(&(fifo->fifo)));

		FMDEV_DEBUG(
		"fmdev_fifo_access(user: %u, dir_in: %u): WaitIntRet: %i\n",
			user, dir_in, retval);

		if (retval != 0)
			/* signal: tell the fs layer to handle it */
			return retval;

		/* otherwise loop, but first reacquire the lock */
		if (down_interruptible(&(fifo->sem)))
			return -ERESTARTSYS;

		cond = (dir_in) ? kfifo_is_full(&(fifo->fifo)) :
				kfifo_is_empty(&(fifo->fifo));
	}

	/* No longer waiting for data/space in the kfifo.
		We still hold the semaphore. */
	if (user) {
		if (dir_in)
			retval = kfifo_from_user(&(fifo->fifo),
				buf, count, copied);
		else
			retval = kfifo_to_user(&(fifo->fifo),
				buf, count, copied);
	} else {
		retval = 0;
		if (dir_in)
			*copied = kfifo_in(&(fifo->fifo), buf, count);
		else
			*copied = kfifo_out(&(fifo->fifo), buf, count);
	}

	up(&(fifo->sem)); /* release the lock */

	/* Wake up any thread(s) waiting on the other end of the fifo. */
	if (retval == 0) {
		FMDEV_DEBUG(
		"fmdev_fifo_access(user: %u, dir_in: %u): copied %u bytes\n",
				user, dir_in, (*copied));
		if (dir_in)
			wake_up_interruptible(&(fifo->outq));
		else
			wake_up_interruptible(&(fifo->inq));
	}

	FMDEV_DEBUG("fmdev_fifo_access(user: %u, dir_in: %u): %i\n",
				user, dir_in, retval);
	return retval;

}

/**
 * Lock the mutex that protects fmdev state information.
 *
 * @param blocking Indicates behavior if another thread currently holds
 * the mutex. If "false", return an error immediately in this case,
 * otherwise block until the other thread releases the mutex.
 * @return 0 if mutex was successfully locked, an error code otherwise.
 */
static int fmdev_mutex_lock(bool blocking)
{
	int status = 0;

	if (blocking)
		mutex_lock(&(fmdev_info.fmdev_mutex));
	else
		status = !mutex_trylock(&(fmdev_info.fmdev_mutex));

	return status;
}

/**
 * Unlock the mutex that protects fmdev state information.
 *
 * @return 0 if mutex was successfully unlocked, an error code otherwise.
 */
static void fmdev_mutex_unlock(void)
{
	mutex_unlock(&(fmdev_info.fmdev_mutex));
}

/**
 * fops open
 */
static int fmdev_open(struct inode *inode, struct file *filp)
{
	int status = 0;
	FMDEV_DEBUG("fmdev_open(inode: 0x%08X, fiop: 0x%08X)\n",
				(unsigned int) inode, (unsigned int) filp);

	if (filp == NULL)
		return -EINVAL;

	status = fmdev_mutex_lock(!(filp->f_flags & O_NONBLOCK));
	if (status == 0) {
		/* Only allow one opening at a time for read and write. */
		if (filp->f_mode & fmdev_info.f_mode &
			(FMODE_READ | FMODE_WRITE))
			status = -EBUSY;
		else
			fmdev_info.f_mode |=
				(filp->f_mode & (FMODE_READ | FMODE_WRITE));

		fmdev_mutex_unlock();
	}

	return status;
}

/**
 * fops close
 */
static int fmdev_close(struct inode *inode, struct file *filp)
{
	int status = 0;
	FMDEV_DEBUG("fmdev_close()\n");

	if (filp != NULL) {
		status = fmdev_mutex_lock(!(filp->f_flags & O_NONBLOCK));
		if (status == 0) {
			fmdev_info.f_mode &=
				~(filp->f_mode & (FMODE_READ | FMODE_WRITE));
			fmdev_mutex_unlock();
		}
	} else
		status = -EINVAL;

	return status;
}

/**
 * Called when a program in userspace reads the fmdev device file.
 */
static ssize_t fmdev_read(struct file *filp, char __user *buffer,
	size_t length, loff_t *offset)
{
	FMDEV_DEBUG("fmdev_read()\n");

	if (IS_ERR(filp))
		return PTR_ERR(filp);
	else {
		unsigned int count = 0;
		ssize_t retval;

		/* Read from the Kernel to User (K2U) fifo. */
		retval = fmdev_fifo_access(&(fmdev_info.k2u_fifo),
			buffer, length, &count, true, false,
			!(filp->f_flags & O_NONBLOCK));

		if (retval == 0)
			retval = (ssize_t) count;

		return retval;
	}
}

/**
 * Called when a program in userspace writes to the fmdev device file.
 */
static ssize_t fmdev_write(struct file *filp,
	const char __user *buffer, size_t length, loff_t *offset)
{
	FMDEV_DEBUG("fmdev_write()\n");

	if (IS_ERR(filp))
		return PTR_ERR(filp);
	else {
		unsigned int count = 0;
		ssize_t retval;

		/* Read from the Kernel to User (K2U) fifo. */
		retval = fmdev_fifo_access(&(fmdev_info.u2k_fifo),
			(char *) buffer, length,
			&count, true, true,
			!(filp->f_flags & O_NONBLOCK));

		if (retval == 0)
			retval = (ssize_t) count;

		return retval;
	}
}

/* Receive bytes from the fmdev character device in the kernel side. */
ssize_t fmdev_receive(char *buffer, size_t length, bool block)
{
	unsigned int count = 0;
	ssize_t retval;

	FMDEV_DEBUG("fmdev_receive()\n");

	/* Read from the User to Kernel (U2K) fifo. */
	retval = fmdev_fifo_access(&(fmdev_info.u2k_fifo),
		buffer, length, &count, false, false, true);

	if (retval == 0)
		retval = (ssize_t) count;

	return retval;
}
EXPORT_SYMBOL(fmdev_receive);

/* Send bytes through the fmdev character device from the kernel side. */
static ssize_t fmdev_send(const char *buffer, size_t length, bool block)
{
	unsigned int count = 0;
	ssize_t retval;

	FMDEV_DEBUG("fmdev_send(buffer: %p, length: %u, block: %u)\n",
			buffer, length, block);

	if (!(fmdev_info.f_mode & FMODE_READ))
		return length;

	/* Write to the Kernel to User (K2U) fifo. */
	retval = fmdev_fifo_access(&(fmdev_info.k2u_fifo), (char *) buffer,
		length, &count, false, true, true);

	if (retval == 0)
		retval = (ssize_t) count;

	return retval;
}

/* Send bytes represented by an IO Vector structure through the fmdev character
	 device from the kernel side. */
int fmdev_send_vector(struct iovec *vector, const size_t count)
{
	size_t i;
	size_t total_size = 0;
	int retval = 0;

	FMDEV_DEBUG("fmdev_send_vector(vector: %p, count: %u)\n",
				vector, count);

	if (!(fmdev_info.f_mode & FMODE_READ))
                return retval;

	/* Lock the mutex in blocking mode to ensure the vector blocks from
		 different threads are written in the correct order. */
	if (fmdev_mutex_lock(true)) {
		pr_crit("fmdev_send_vector(): mutex_lock failed!\n");
		return -1;
	}

	/* Add up the total number of bytes in the vector to check if there is
		 enough space available in the kfifo to hold it. */
	for (i = 0; i < count; i++)
		total_size += vector[i].iov_len;

	if (kfifo_avail(&(fmdev_info.k2u_fifo.fifo)) < total_size) {
		if (!fmdev_info.f_mode)
			goto out;

		pr_crit_ratelimited("fmdev_send_vector(count: %u): Not enough space in fifo!\n",
					count);
		retval = -EAGAIN;
          	goto out;
	}

	for (i = 0; !retval && i < count; i++) {
		if (vector[i].iov_len > 0) {
			retval = fmdev_send((char *) vector[i].iov_base,
						vector[i].iov_len, false);
			if (retval == (int) vector[i].iov_len)
				retval = 0;
			else if (retval > 0 && retval < (int) vector[i].iov_len)
				retval = -1;
		}
	}

out:
	fmdev_mutex_unlock();
	return retval;
}
EXPORT_SYMBOL(fmdev_send_vector);

/**
 * Initialize the fmdev module.
 */
static int __init fmdev_init(void)
{
	int retval;
/*
	fmdev_info.fops.owner = THIS_MODULE;
	fmdev_info.fops.read = fmdev_read;
	fmdev_info.fops.write = fmdev_write;
	fmdev_info.fops.open = fmdev_open;
	fmdev_info.fops.release = fmdev_close;
*/
	pr_info("fmdev_init()\n");

	mutex_init(&(fmdev_info.fmdev_mutex));

	fmdev_info.f_mode = 0;

	/* Todo: make loop */
	retval = fmdev_fifo_init(&(fmdev_info.u2k_fifo), 4096);
	if (retval != 0) {
		pr_crit("fmdev_init(): Failed to allocate u2k fifo: %i!\n",
			retval);
		goto error_u2k_fifo_init;
	}

	retval = fmdev_fifo_init(&(fmdev_info.k2u_fifo), 4096);
	if (retval != 0) {
		pr_crit("fmdev_init(): Failed to allocate k2u fifo: %i!\n",
			retval);
		goto error_k2u_fifo_init;
	}

	/* Allocate a region of 1 character devices for "fmdev". */
	retval = alloc_chrdev_region(&(fmdev_info.dev), 0, 1, "fmdev");
	if (retval != 0) {
		pr_crit("fmdev_init(): alloc_chrdev_region() returned: %i!\n",
			retval);
		goto error_alloc_chrdev_region;
	}

	cdev_init(&(fmdev_info.fmdev_cdev), &(fmdev_info.fops));
	fmdev_info.fmdev_cdev.owner = THIS_MODULE;
	fmdev_info.fmdev_cdev.ops = &(fmdev_info.fops);

	retval = cdev_add(&(fmdev_info.fmdev_cdev), fmdev_info.dev, 1);
	if (retval != 0) {
		pr_crit("fmdev_init(): cdev_add() returned: %i!\n",
			retval);
		goto error_cdev_add;
	}

	fmdev_info.fmdev_class = class_create(THIS_MODULE, "fmdev");
	if (IS_ERR(fmdev_info.fmdev_class)) {
		pr_crit("fmdev_init(): class_create() returned: %li!\n",
			PTR_ERR(fmdev_info.fmdev_class));
		retval = PTR_ERR(fmdev_info.fmdev_class);
		goto error_class_create;
	}

	fmdev_info.fmdev_device = device_create(fmdev_info.fmdev_class,
		NULL, fmdev_info.dev, NULL, "fmdev_device");
	if (IS_ERR(fmdev_info.fmdev_device)) {
		pr_crit("fmdev_init(): device_create() returned: %li!\n",
			PTR_ERR(fmdev_info.fmdev_device));
		retval = PTR_ERR(fmdev_info.fmdev_device);
		goto error_device_create;
	}

	pr_info("fmdev_init(): Success! retval: %i\n", retval);
	return retval;

/* Error handling: We must undo everything that was successful before
	the failure.  Unfortunately goto's are the cleanest/easiest way of
	doing this. */
error_device_create:
	class_unregister(fmdev_info.fmdev_class);
	class_destroy(fmdev_info.fmdev_class);

error_class_create:
	cdev_del(&(fmdev_info.fmdev_cdev));

error_cdev_add:
	unregister_chrdev_region(fmdev_info.dev, 1);

error_alloc_chrdev_region:
	fmdev_fifo_destroy(&(fmdev_info.k2u_fifo));

error_k2u_fifo_init:
	fmdev_fifo_destroy(&(fmdev_info.u2k_fifo));

error_u2k_fifo_init:

	return retval;
}

/**
 * Called when unloading the "fmdev" module to release any kernel resources still allocated in fm_dev.
 */
static void __exit fmdev_exit(void)
{
	pr_info("fmdev_exit()\n");

	device_destroy(fmdev_info.fmdev_class, fmdev_info.dev);
	class_unregister(fmdev_info.fmdev_class);
	class_destroy(fmdev_info.fmdev_class);

	cdev_del(&(fmdev_info.fmdev_cdev));
	unregister_chrdev_region(fmdev_info.dev, 1);

	fmdev_fifo_destroy(&(fmdev_info.k2u_fifo));
	fmdev_fifo_destroy(&(fmdev_info.u2k_fifo));
}

arch_initcall(fmdev_init);
module_exit(fmdev_exit);
