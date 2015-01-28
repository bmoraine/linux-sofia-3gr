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
*/
#ifndef OCT_DEV_H
#define OCT_DEV_H

/* oct driver default settings */
#define OCT_EXT_RING_BUFF_SIZE 0x80000 /* 2MB buffer size */
#define DEFAULT_OCT_MODE        OCT_MODE_STALL
#define DEFAULT_OCT_PATH        OCT_PATH_TTY
/* page timeout settings */
#define OCT2_PG_TIMEOUT_RUN     1000 /* 1000 msec */
#define OCT2_PG_TIMEOUT_SLEEP  10000 /* 10 sec */


enum e_oct_ioctl {
	OCT_IOCTL_SET_PATH = 10,   /* tty or read-pool if */
	OCT_IOCTL_SET_MODE,        /* stall or overwrite */
	OCT_IOCTL_CONF_TRIG_CYCLE, /* config trig cycle timeout value */
	OCT_IOCTL_ENTER_CD,        /* enter coredump mode */
	OCT_IOCTL_FLUSH,           /* flush data into external ring buffer */
	OCT_IOCTL_GET_INFO         /* get OCT info like rd/wr ptr */
};

enum e_oct_mode {
	OCT_MODE_OFF = 0,
	OCT_MODE_STALL,
	OCT_MODE_OW
};

enum e_oct_path {
	OCT_PATH_NONE = 0, /* for background traces */
	OCT_PATH_APP_POLL,
	OCT_PATH_TTY
};

struct s_oct_info {
	unsigned int rd_ptr;
	unsigned int wr_ptr;
	unsigned char is_full;
	unsigned char irq_stat;
};

#endif
