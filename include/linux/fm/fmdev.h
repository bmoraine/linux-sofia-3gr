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

#include <linux/types.h>

/* Kernel-side interface of fmdev character device driver, exposed to allow
IuiFm interface to interact with FM in userspace/mex. */

/* Receive bytes from the fmdev character device in the kernel side. */
ssize_t fmdev_receive(char *buffer, size_t length, bool block);

/* Send bytes represented by an IO Vector structure through the fmdev character
   device from the kernel side. */
int fmdev_send_vector(struct iovec *vector, const size_t count);
