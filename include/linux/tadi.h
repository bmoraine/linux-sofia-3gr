/*
 *  Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License Version 2
 *  as published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 */
#ifndef TADI_CDEV_H
#define TADI_CDEV_H

#define TADI_MAX_LIST_SIZE 5

struct s_tadi_multi_write {
	void *handle;
	unsigned char *buff;
	int len;
};

enum e_tadi_cmd {
	CMD_GET_REV = 10,
	CMD_SET_MT,
	CMD_WRITE_MUL_FIRST,
	CMD_WRITE_MUL_CONT,
	CMD_WRITE_MUL_LAST
};

enum e_tadi_msg_type {
	MT_PRINTF = 16,
	MT_ASCII_UFMT,
	MT_ASCII_PCOD,
	MT_FTRACE = 128,
	MT_WLAN,
	MT_GNSS,
	MT_BT,
	MT_CUST_TRACE00 = 224,
	MT_CUST_TRACE01,
	MT_CUST_TRACE02,
	MT_CUST_TRACE03,
	MT_CUST_TRACE04,
	MT_CUST_TRACE05,
	MT_CUST_TRACE06,
	MT_CUST_TRACE07,
	MT_CUST_TRACE08,
	MT_CUST_TRACE09,
	MT_CUST_TRACE10,
	MT_CUST_TRACE11,
	MT_CUST_TRACE12,
	MT_CUST_TRACE13,
	MT_CUST_TRACE14,
	MT_CUST_TRACE15,
	MT_CUST_TRACE16,
	MT_CUST_TRACE17,
	MT_CUST_TRACE18,
	MT_CUST_TRACE19,
	MT_CUST_TRACE20,
	MT_CUST_TRACE21,
	MT_CUST_TRACE22,
	MT_CUST_TRACE23,
	MT_CUST_TRACE24,
	MT_CUST_TRACE25,
	MT_CUST_TRACE26,
	MT_CUST_TRACE27,
	MT_CUST_TRACE28,
	MT_CUST_TRACE29,
	MT_CUST_TRACE30,
	MT_CUST_TRACE31
};

/* API to use tadi interface from Kernel space.
	Routines are used in the following sequence:
	open->write(single or multiple)->close */
void *trc_tadi_open(unsigned char mt);
void trc_tadi_write(void *handle, void *ptr, int count);
void trc_tadi_close(void *handle);

#endif
