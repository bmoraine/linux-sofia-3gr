/*
 *******************************************************************************
 *
 *  Component: VLX Virtual USB SIO Extender Front End (Kernel Mode)
 *
 *  Copyright (C) 2011 - 2012 Intel Mobile Communications GmbH
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
 *******************************************************************************
 */

/*****************************************************************************/
/*****************************************************************************/
/**                                                                         **/
/** This module is the Front-End half of a link that handles commands from  **/
/** a matching Back-End, that are used to initialise and run a set of data  **/
/** link transports that connect MEX-side SIO channels onto Linux-side      **/
/** serial gadgets.                                                         **/
/**                                                                         **/
/** The number and mapping of the links is sent in a set of setup commands  **/
/**                                                                         **/
/** 'modem' events from each individual link are converted to/from text     **/
/** messages and multiplexed over the control channel.                      **/
/**                                                                         **/
/*****************************************************************************/
/*****************************************************************************/

/*****************************************************************************/
/* INCLUDES                                                                  */
/*****************************************************************************/

/*----- Standard header files -----*/

#include <linux/version.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/err.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <asm/segment.h>
#include <linux/buffer_head.h>
#include <asm/termios.h>
#include <asm/ioctls.h>

#include <linux/ctype.h>

/*----- Module's Own Headers -----*/

#include "vusb_secmd_defs.h"

#include "vusb_se_fe.h"

/*****************************************************************************/
/* DEFINES                                                                   */
/*****************************************************************************/

/* ====================================================================== */
/* This flag needs to be enabled only when for testing GCOV code coverage */
/* ====================================================================== */

/* #define GCOV_CODE_COVERAGE */

/************************/
/* VERSION TAG          */
/************************/

#define VUSB_SE_FE_VERSION_MAJOR   1
#define VUSB_SE_FE_VERSION_MINOR   2

/************************/
/* Explicit MVPIPE !    */
/************************/

#define VUSB_SE_FE_COMMAND_MVPIPE_NUM 8

/************************/
/* MVPIPE Setup         */
/************************/
#define VUSB_SE_FE_MVPIPE_CMD_NAME "/dev/mvpipe-secmd"
#define VUSB_SE_FE_MVPIPE_GENERIC_DATA_NAME  "/dev/mvpipe-sedat%d"

/************************/
/* Gadget Setup         */
/************************/

#define VUSB_SE_FE_GADGET_GENERIC_NAME  "/dev/ttyGS%d"

/************************/
/* TXFR Status          */
/************************/

#define VUSB_SE_FE_TXFR_OK       0
#define VUSB_SE_FE_TXFR_RX_NUL  -1
#define VUSB_SE_FE_TXFR_RX_ERR  -2
#define VUSB_SE_FE_TXFR_TX_NUL  -3
#define VUSB_SE_FE_TXFR_TX_ERR  -4

/*****************************************************************************/
/* MACRO DEFINITIONS                                                         */
/*****************************************************************************/

/************************/
/* Generic Link Check   */
/************************/

#define VUSB_SE_FE_CHECK_AND_SET_LINK(data, link, tag) {\
	link = (unsigned int) data;\
	VUSB_SE_FE_LOG("Thread %s[%d] Start", tag, link);\
	if (link > (VUSB_SECMD_MAX_LINKS - 1)) {\
		VUSB_SE_FE_ERR("Bad %s Link Number %d", tag, link);\
		return -1;\
	} \
}

/************************/
/* T/F from Bit Value   */
/************************/

#define VUSB_SE_FE_IS_BIT_SET(mask, value)                                \
	((mask == (value & mask)) ? VUSB_SE_FE_TRUE : VUSB_SE_FE_FALSE)

/*****************************************************************************/
/* TYPE DEFINITIONS                                                          */
/*****************************************************************************/

enum vusb_se_fe_modem_signal {
	VUSB_SE_FE_MODEM_SIGNAL_UNKNOWN,
	VUSB_SE_FE_MODEM_SIGNAL_DTR,
	VUSB_SE_FE_MODEM_SIGNAL_DSR,
	VUSB_SE_FE_MODEM_SIGNAL_RTS,
	VUSB_SE_FE_MODEM_SIGNAL_CTS,
	VUSB_SE_FE_MODEM_SIGNAL_DCD
};

enum vusb_se_fe_modem_state {
	VUSB_SE_FE_MODEM_STATE_UNKNOWN,
	VUSB_SE_FE_MODEM_STATE_ON,
	VUSB_SE_FE_MODEM_STATE_OFF
};

struct vusb_se_fe_scan {
	char *text;
	unsigned int eval;
};

struct vusb_se_fe_map_link_id {
	int link_id;
	int gadget;
	int data_pipe_id;
};

struct vusb_se_data {
	unsigned uint_val;
} *vusb_data;

/*****************************************************************************/
/* LOCAL DATA OBJECTS                                                        */
/*****************************************************************************/

/************************/
/* System State         */
/************************/

static bool vusb_se_fe_started = VUSB_SE_FE_FALSE;
static bool vusb_se_fe_started_threads = VUSB_SE_FE_FALSE;


/************************/
/* Linux->MEX Tx Mutex  */
/************************/
struct mutex vusb_se_fe_tx_lock;

struct mutex vusb_dat_reopen_lock;

/************************/
/* MVPIPE HANDLES       */
/************************/

static struct file *vusb_se_fe_command_mvpipe_fp;

static struct file *vusb_se_fe_link_mvpipe_fp[VUSB_SECMD_MAX_LINKS];

/************************/
/* GADGET HANDLES       */
/************************/

static struct file *vusb_se_fe_link_gadget_fp[VUSB_SECMD_MAX_LINKS];

/************************/
/* MAX Link IDs         */
/************************/

static unsigned int vusb_se_fe_num_links;

/************************/
/* MAPPING TABLES       */
/************************/

static struct vusb_se_fe_scan vusb_se_fe_modem_signaltable[] = {

	{VUSB_SECMD_TXT_SIGNAL_DTR, (unsigned int)VUSB_SE_FE_MODEM_SIGNAL_DTR},
	{VUSB_SECMD_TXT_SIGNAL_DSR, (unsigned int)VUSB_SE_FE_MODEM_SIGNAL_DSR},
	{VUSB_SECMD_TXT_SIGNAL_RTS, (unsigned int)VUSB_SE_FE_MODEM_SIGNAL_RTS},
	{VUSB_SECMD_TXT_SIGNAL_CTS, (unsigned int)VUSB_SE_FE_MODEM_SIGNAL_CTS},
	{VUSB_SECMD_TXT_SIGNAL_DCD, (unsigned int)VUSB_SE_FE_MODEM_SIGNAL_DCD},
};

static struct vusb_se_fe_scan vusb_se_fe_modem_statetable[] = {

	{VUSB_SECMD_TXT_STATE_ON, (unsigned int)VUSB_SE_FE_MODEM_STATE_ON},
	{VUSB_SECMD_TXT_STATE_OFF, (unsigned int)VUSB_SE_FE_MODEM_STATE_OFF},
};

static struct vusb_se_fe_map_link_id
	vusb_se_fe_map_link_idtable[VUSB_SECMD_MAX_LINKS];

/*****************************************************************************/
/* LOCAL FUNCTION PROTOTYPES                                                 */
/*****************************************************************************/

/************************/
/* Module Entry/Exit    */
/************************/

static int __init vusb_se_fe_init(void);

#ifdef GCOV_CODE_COVERAGE

static void vusb_se_fe_exit(void);

#endif

/************************/
/* Utilities            */
/************************/

static void vusb_se_fe_log_ktermios(int gadget, struct ktermios *pattr);

static char *vusb_se_fe_ersatz_strtok(char *text, char *seps);
static unsigned int vusb_se_fe_ersatz_strtoi(char *text);

static struct file *vusb_se_fe_open_mvpipe(char *name);
static struct file *vusb_se_fe_open_gadget(int gadget);
static void vusb_se_fe_close_gadget(struct file *fp, int gadget);

/************************/
/* Message Handlers     */
/************************/

static void vusb_se_fe_parse_message(char *message);

static void vusb_se_fe_handle_initialise(void);
static void vusb_se_fe_handle_start_threads(void);
static void vusb_se_fe_handle_modem_signal(void);

/************************/
/* Response Handler     */
/************************/

static void vusb_se_fe_send_message(int channel, char *item, char *signal,
				    char *state);

/************************/
/* Message RX Thread    */
/************************/

static int vusb_se_fe_command_thread(void *data);

/************************/
/* Generic Data Links   */
/************************/

static int vusb_se_fe_do_link_txfr(unsigned int link, char *tag,
				   struct file **rx_fps, struct file **tx_fps);

static int vusb_se_fe_link_rx_thread(void *data);
static int vusb_se_fe_link_tx_thread(void *data);

static int vusb_se_fe_modem_event_thread(void *data);

/************************/
/* Initialisation       */
/************************/

static int __init vusb_se_fe_init(void);

/*****************************************************************************/
/* LOCAL FUNCTION IMPLEMENTATIONS                                            */
/*****************************************************************************/

/*****************************************************************************/
/* Function:... vusb_se_fe_log_ktermios                                      */
/*                                                                           */
/* Description: This function logs the content of the 'ktermios; struct that */
/*              is used to change the behaviour of the ttyGS channels        */
/*****************************************************************************/

static void vusb_se_fe_log_ktermios(int gadget, struct ktermios *pattr)
{
	int n;

	char *c_cc_txt[] = {
		"VINTR   ",
		"VQUIT   ",
		"VERASE  ",
		"VKILL   ",
		"VEOF    ",
		"VTIME   ",
		"VMIN    ",
		"VSWTC   ",
		"VSTART  ",
		"VSTOP   ",
		"VSUSP   ",
		"VEOL    ",
		"VREPRINT",
		"VDISCARD",
		"VWERASE ",
		"VLNEXT  ",
		"VEOL2   ",
		"XXX-17  ",
		"XXX-18  ",
		"XXX-19  "
	};

	VUSB_SE_FE_LOG("[%d] .... c_iflag = Octal(%07o)", gadget,
		       pattr->c_iflag);
	VUSB_SE_FE_LOG("[%d] .... c_oflag = Octal(%07o)", gadget,
		       pattr->c_oflag);
	VUSB_SE_FE_LOG("[%d] .... c_cflag = Octal(%07o)", gadget,
		       pattr->c_cflag);
	VUSB_SE_FE_LOG("[%d] .... c_lflag = Octal(%07o)", gadget,
		       pattr->c_lflag);

	for (n = 0; n < NCCS; n++) {

		VUSB_SE_FE_LOG("[%d] .... c_cc[%02d][%s] = 0x%02X", gadget, n,
			       c_cc_txt[n], pattr->c_cc[n]);
	}
}

/*****************************************************************************/
/* Function:... vusb_se_fe_ersatz_strtok                                     */
/*                                                                           */
/* Description: This is an unashamed version of strtok() needed since there  */
/*              is no implementation in the VLX linux library                */
/*                                                                           */
/*      It is : *NOT* thread-safe but this is accepted since the incoming    */
/*              messages only come from the single command reader thread     */
/*   So it is :                                                              */
/*            : Trusting that the inputs are NUL-terminated strings          */
/*            : Not optimised                                                */
/*****************************************************************************/

static char *vusb_se_fe_ersatz_strtok(char *text, char *seps)
{
	static char *start;
	static char *end;

	char *tptr;
	char *sptr;

	int token;

	/* The buffer has to be declared static so that the content is loaded
	 * when 'text' is non-NULL, but on subsequent calls when 'text' is NULL
	 * (as per standard strtok operation) then the content must be still
	 * available
	 */
	static char buff[VUSB_SECMD_BUFFER_SIZE];

	/* Although 'text' can be a pointer or NULL, 'seps' must be a pointer
	 */
	if (NULL == seps) {

		VUSB_SE_FE_ERR("STRTOK Separator String is NULL");

		return text;
	}

	/* If the entry is non-NULL then this is a first-time use where
	 * the local ( static ) variables are initialised and the delimiters
	 * are knocked out
	 */
	if (text) {

		/*
		 * Put the string into the local buffer
		 * so that the NUL-embedding doesn't
		 * corrupt the input string
		 */
		memset(buff, 0, VUSB_SECMD_BUFFER_SIZE);

		strncpy(buff, text, VUSB_SECMD_BUFFER_SIZE);

		buff[VUSB_SECMD_BUFFER_SIZE - 1] = 0;

		/* Now process the string, overwrite all separators with NUL
		 */
		start = &buff[0];
		end = start + strlen(buff);

		token = 0;

		for (tptr = start; tptr < end; tptr++) {

			for (sptr = seps; *sptr != 0; sptr++) {

				if (*tptr == *sptr) {

					token++;

					VUSB_SE_FE_INF
					    ("Token %d: Sep[%c] at [%zu]",
					     token, *sptr, (tptr - start));

					*tptr = 0;

					break;
				}
			}
		}

		/* For debugging, log the individual token values
		 */
		token = 0;

		for (tptr = start; tptr < end; tptr++) {

			if (*tptr) {

				token++;

				VUSB_SE_FE_INF("Token %d: %s", token, tptr);

				while ((*tptr)
				       && (tptr < end)
				    ) {

					tptr++;
				}
			}
		}
	}

	/* If repeated calls have exhausted the input string clear the return
	 * pointer ( avoid multiple exit points )
	 */
	if (NULL == start) {

		tptr = NULL;
	} else {

		/*
		 * Wherever we are, advance along the string
		 * ignoring NUL characters
		 * ( which are where the delimiters were )
		 */
		while ((0 == *start)
		       && (start < end)
		    ) {

			start++;
		}

		tptr = start;

		/*
		 * If we are at the end of the string then
		 * we simply set NULL to clear it all down
		 * to prevent any further char ptr increment
		 */
		if (start == end)
			tptr = start = end = NULL;

		/* Now march the start pointer along the string looking for the
		 * next delimiter ( NUL ) - unless we're at the very end
		 */
		if (start) {

			while ((*start != 0)
			       && (start < end)
			    ) {

				start++;
			}
		}
	}

	if (tptr) {
		VUSB_SE_FE_INF("Returning [%s]", tptr);
	} else {
		VUSB_SE_FE_INF("Returning NULL [No Tokens Left]");
	}

	return tptr;
}

/*****************************************************************************/
/* Function:... vusb_se_fe_ersatz_strtoi                                     */
/*                                                                           */
/* Description: This is an unashamed version of strtoi() needed since there  */
/*              is no implementation of strtol() in the VLX linux library    */
/*                                                                           */
/*      It is : *NOT* thread-safe but this is accepted since the incoming    */
/*              messages only come from the single command reader thread     */
/*   So it is :                                                              */
/*            : Trusting that the inputs are NUL-terminated strings          */
/*            : Not optimised                                                */
/*****************************************************************************/

static unsigned int vusb_se_fe_ersatz_strtoi(char *text)
{
	char *dptr;

	unsigned int result = 0;

	int digits = 0;

	dptr = text;

	if (NULL == dptr) {
		VUSB_SE_FE_ERR("Bad Pointer for STRTOI");
	} else {
		VUSB_SE_FE_LOG("STRTOI Converting %s", dptr);
	}

	/* Simply step along the string until encountering a NUL (end of string)
	 * or alternatively, any character that is not a digit
	 */
	while (VUSB_SE_FE_TRUE) {

		char cval = 0;

		if (NULL != dptr)
			cval = *dptr;

		if (0 == cval) {

			VUSB_SE_FE_LOG("STRTOI hits NUL, returning %d", result);

			return result;
		}

		if ((cval < '0')
		    || (cval > '9')
		    ) {

			VUSB_SE_FE_LOG("STRTOI hits Non-Digit %c, returning %d",
				       cval, result);

			return result;
		}

		result = (10 * result) + (cval - '0');

		digits++;

		if (result > 99999) {

			VUSB_SE_FE_ERR("STRTOI result is too big!");

			return 0;
		}

		if (digits > 5) {

			VUSB_SE_FE_ERR("STRTOI finds too many digits!");

			return 0;
		}

		/* If not returned a value yet, step on to the next digit
		 */
		dptr++;
	}

	return result;
}


/*****************************************************************************/
/* Function:... vusb_se_fe_open_mvpipe                                       */
/* Description: Opens the mvpipe nominated by the pipe name                  */
/*****************************************************************************/
static struct file *vusb_se_fe_open_mvpipe(char *name)
{

	struct file *fp = NULL;

	mm_segment_t old_fs;

	VUSB_SE_FE_LOG("Opening MVPIPE[%s]", name);

	while (!kthread_should_stop()) {

		old_fs = get_fs();

		set_fs(KERNEL_DS);
		{
			fp = filp_open(name, O_RDWR, 0);
		}
		set_fs(old_fs);

		if (NULL == fp) {

			VUSB_SE_FE_ERR("NULL MVPIPE File Handle");

			msleep(1000);

			continue;
		} else if (IS_ERR(fp)) {

			VUSB_SE_FE_ERR("MVPIPE open failed!");

			msleep(1000);

			continue;
		} else {

			VUSB_SE_FE_INF("MVPIPE open succeeded.");

			break;
		}

		set_current_state(TASK_INTERRUPTIBLE);
	}

	/* Purely for klocwork, check file operation pointers look
	 * valid...
	 */
	if ((NULL == fp)
	    || (NULL == fp->f_op)
	    || (NULL == fp->f_op->open)
	    || (NULL == fp->f_op->release)
	    || (NULL == fp->f_op->read)
	    || (NULL == fp->f_op->write)
	    ) {

		VUSB_SE_FE_ERR("Fatal MVPIPE function error!");

		msleep(1000);

		return NULL;
	}

	return fp;
}

/*****************************************************************************/
/* Function:... vusb_se_fe_close_mvpipe                                       */
/* Description: Closes the mvpipe                  */
/*****************************************************************************/
static void vusb_se_fe_close_mvpipe(struct file *fp)
{
	/* Unused argument inode to function release */
	mm_segment_t oldfs;

	oldfs = get_fs();

	set_fs(KERNEL_DS);

	filp_close(fp, 0);

	set_fs(oldfs);

	return;
}

/*****************************************************************************/
/* Function:... vusb_se_fe_open_gadget                                       */
/* Description: Opens the gadget nominated by the pipe number                */
/*****************************************************************************/

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

static void vusb_se_fe_clr_bit(tcflag_t *pflags, tcflag_t mask)
{
	VUSB_SE_FE_INF("CLR Bit 0x%07o", mask);
	VUSB_SE_FE_INF("---- Before 0x%07o", *pflags);
	*pflags &= ~mask;
	VUSB_SE_FE_INF("---- After  0x%07o", *pflags);
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

static void vusb_se_fe_set_bit(tcflag_t *pflags, tcflag_t mask)
{
	VUSB_SE_FE_INF("SET Bit 0x%07o", mask);
	VUSB_SE_FE_INF("---- Before 0x%07o", *pflags);
	*pflags |= mask;
	VUSB_SE_FE_INF("---- After  0x%07o", *pflags);
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

static void vusb_se_fe_set_ccc(struct ktermios *attr, unsigned int item,
			       int val)
{
	if (item < NCCS) {

		VUSB_SE_FE_INF("Change c_cc value [%d]", item);
		VUSB_SE_FE_INF("---- Before %d", attr->c_cc[item]);
		attr->c_cc[item] = val;
		VUSB_SE_FE_INF("---- After  %d", attr->c_cc[item]);
	} else {

		VUSB_SE_FE_ERR("Bad Item [%d] for Change C-CC", item);
	}
}

/*---------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------*/

static struct file *vusb_se_fe_open_gadget(int gadget)
{
	char gadget_name[32];

	struct file *fp = NULL;

	mm_segment_t old_fs;

	long diagnostic;

	sprintf(gadget_name, VUSB_SE_FE_GADGET_GENERIC_NAME, gadget);

	VUSB_SE_FE_LOG("Opening Gadget[%s]", gadget_name);

	while (!kthread_should_stop()) {

		old_fs = get_fs();

		set_fs(KERNEL_DS);
		{
			fp = filp_open(gadget_name, O_RDWR, 0);
		}
		set_fs(old_fs);

		if (NULL == fp) {

			VUSB_SE_FE_ERR("NULL Gadget File Handle");

			msleep(2000);

			continue;
		} else if (IS_ERR(fp)) {

			VUSB_SE_FE_ERR("Gadget open failed!");

			msleep(2000);

			continue;
		} else {

			VUSB_SE_FE_INF("Gadget open succeeded.");

			break;
		}

		set_current_state(TASK_INTERRUPTIBLE);
	}

	/* Purely for klocwork, check file operation pointers look
	 * valid...
	 */
	if ((NULL == fp)
	    || (NULL == fp->f_op)
	    || (NULL == fp->f_op->open)
	    || (NULL == fp->f_op->release)
	    || (NULL == fp->f_op->read)
	    || (NULL == fp->f_op->write)
	    || (NULL == fp->f_op->unlocked_ioctl)
	    ) {

		VUSB_SE_FE_ERR("Fatal Gadget function error!");

		msleep(1000);

		return NULL;
	}

	old_fs = get_fs();

	set_fs(KERNEL_DS);
	{
		struct ktermios myattr;

		/*
		 * Read the current set of terminal
		 * attribute flags and dump the result
		 */
		diagnostic =
		    fp->f_op->unlocked_ioctl(fp, TCGETS,
					     (unsigned long)&myattr);

		VUSB_SE_FE_LOG("[ttyGS%d] Get Attributes %ld", gadget,
			       diagnostic);

		vusb_se_fe_log_ktermios(gadget, &myattr);

		/*
		 * Modify the attributes to be
		 * what we want instead of the default
		 */
		vusb_se_fe_clr_bit(&myattr.c_iflag, IGNBRK);
		vusb_se_fe_clr_bit(&myattr.c_iflag, BRKINT);
		vusb_se_fe_clr_bit(&myattr.c_iflag, PARMRK);
		vusb_se_fe_clr_bit(&myattr.c_iflag, ISTRIP);
		vusb_se_fe_clr_bit(&myattr.c_iflag, INLCR);
		vusb_se_fe_clr_bit(&myattr.c_iflag, IGNCR);
		vusb_se_fe_clr_bit(&myattr.c_iflag, ICRNL);
		vusb_se_fe_clr_bit(&myattr.c_iflag, IXON);

		vusb_se_fe_clr_bit(&myattr.c_oflag, OPOST);

		vusb_se_fe_clr_bit(&myattr.c_lflag, ECHO);
		vusb_se_fe_clr_bit(&myattr.c_lflag, ECHONL);
		vusb_se_fe_clr_bit(&myattr.c_lflag, ICANON);
		vusb_se_fe_clr_bit(&myattr.c_lflag, ISIG);
		vusb_se_fe_clr_bit(&myattr.c_lflag, IEXTEN);

		vusb_se_fe_clr_bit(&myattr.c_cflag, CSIZE);
		vusb_se_fe_clr_bit(&myattr.c_cflag, PARENB);
		vusb_se_fe_set_bit(&myattr.c_cflag, CS8);

		vusb_se_fe_set_ccc(&myattr, VTIME, 0);
		vusb_se_fe_set_ccc(&myattr, VMIN, 1);

		/*
		 * Log what it is that is about
		 * to be sent and then write back the new
		 * attributes that we want
		 */
		VUSB_SE_FE_LOG("Changed attribute settings (about to send)");

		vusb_se_fe_log_ktermios(gadget, &myattr);

		diagnostic =
		    fp->f_op->unlocked_ioctl(fp, TCSETS,
					     (unsigned long)&myattr);

		VUSB_SE_FE_LOG("[ttyGS%d] Set Attributes %ld", gadget,
			       diagnostic);

		/*
		 * Read the attributes back again just
		 * to prove that what we sent has been set
		 */
		diagnostic =
		    fp->f_op->unlocked_ioctl(fp, TCGETS,
					     (unsigned long)&myattr);

		VUSB_SE_FE_LOG("[ttyGS%d] Get Attributes %ld", gadget,
			       diagnostic);

		vusb_se_fe_log_ktermios(gadget, &myattr);
	}
	set_fs(old_fs);

	return fp;
}

/*****************************************************************************/
/* Function:... vusb_se_fe_close_gadget                                      */
/* Description: Releases the gadget file pointer nominated by the pipe number*/
/*****************************************************************************/

static void vusb_se_fe_close_gadget(struct file *fp, int gadget)
{
	/* Unused argument inode to function release */
	mm_segment_t oldfs;

	/* In order to avoid compiler warning set unused variable to void */
	(void)gadget;

	oldfs = get_fs();

	set_fs(KERNEL_DS);

	filp_close(fp, 0);
	VUSB_SE_FE_LOG("Gadget Closed");

	set_fs(oldfs);

	return;
}

/*****************************************************************************/
/* Function:... vusb_se_fe_parse_message                                     */
/*                                                                           */
/* Description: This is the worker function that handles dispatch to the     */
/*              various operational handlers in the SE CMD system            */
/*                                                                           */
/*              'strtok' is used to strip out the various parts of the       */
/*              incoming message, while 'strcmp' is used to identify each    */
/*                                                                           */
/* SETUP        Two messages are handled, one to show how many channels are  */
/* -----        to be set up, the second to provide startup parameters about */
/*              the link and its associated SIO and gadget channels          */
/*                                                                           */
/*              NUM_LINKS:<links>:                                           */
/*                                                                           */
/*              links    specifies how many LINK_DATA will follow, each one  */
/*                       will require a read/write pair of threads to be     */
/*                       created and started                                 */
/*                                                                           */
/*              LINK_DATA:<link>:<mvpipe>:<bufsiz>:<sioNum>:<linNum>         */
/*                                                                           */
/*              link     identifies which one of the links is to be set up   */
/*              mvpipe   gives the mvpipe number that is to be opened        */
/*              bufsiz   is the size of the mvpipe ring buffer               */
/*              sioNum   is for information/logging here, shows USBCDC       */
/*              linNum   is the number of the serial gadget to be opened     */
/*                                                                           */
/* MODEM        On each channel, MEX can send some modem signals that must   */
/* -----        be reflected into the individual serial gadget               */
/*                                                                           */
/*              MODEM:<link>:<signal>:<state>:                               */
/*                                                                           */
/*              link    identifies which link is to receive the modem signal */
/*              signal  is the particular modem signal                       */
/*                      CTS or DTR are handled                               */
/*              state   is the state of the signal                           */
/*                      ON or OFF are handled                                */
/*                                                                           */
/*****************************************************************************/

static void vusb_se_fe_parse_message(char *message)
{
	char *mptr;

	if (!message) {

		VUSB_SE_FE_ERR("Empty incoming message!");

		return;
	}

	VUSB_SE_FE_LOG("Incoming Message To Parse [%s]", message);

	mptr = vusb_se_fe_ersatz_strtok(message, VUSB_SECMD_TXT_SEPARATOR);

	if (!mptr) {

		VUSB_SE_FE_ERR("Badly Formatted, Can't Extract Command!");

		return;
	} else if (strcmp(mptr, VUSB_SECMD_TXT_NUM_LINKS) == 0) {

		vusb_se_fe_handle_initialise();
	} else if (strcmp(mptr, VUSB_SECMD_TXT_LINK_DATA) == 0) {

		vusb_se_fe_handle_start_threads();
	} else if (strcmp(mptr, VUSB_SECMD_TXT_MODEM) == 0) {

		vusb_se_fe_handle_modem_signal();
	} else {

		VUSB_SE_FE_ERR("No Handler for Message [%s]", mptr);
	}
}

/* ------------------------------------------------------------------------- */

static void vusb_se_fe_handle_initialise(void)
{
	char *mptr;

	unsigned int numlinks;

	/* There is one numerical parameter, this identifies the number
	 * of links that are to be set up
	 */
	mptr = vusb_se_fe_ersatz_strtok(NULL, VUSB_SECMD_TXT_SEPARATOR);

	if (!mptr) {

		VUSB_SE_FE_ERR("Empty NumLinks Parameter!");

		return;
	}

	numlinks = vusb_se_fe_ersatz_strtoi(mptr);

	if (numlinks > VUSB_SECMD_MAX_LINKS) {

		VUSB_SE_FE_ERR("Unexpected Number of Links [%d]", numlinks);

		return;
	}

	if (vusb_se_fe_started != VUSB_SE_FE_FALSE) {

		VUSB_SE_FE_ERR("Already Started!");

		return;
	}

	vusb_se_fe_started = VUSB_SE_FE_TRUE;

	vusb_se_fe_num_links = numlinks;
}

/* ------------------------------------------------------------------------- */

static void vusb_se_fe_handle_start_threads(void)
{
	char *mptr;

	int par;

	unsigned int pval[5] = { 0, 0, 0, 0, 0 };

	char thread_name[32];
	char mvpipe_name[32];
	/* Use a 'while' loop to allow break-out on failure instead of
	 * successive indentation
	 */
	while (1) {

		/* Confirm we already had the number of links verified
		 */
		if (vusb_se_fe_started != VUSB_SE_FE_TRUE) {

			VUSB_SE_FE_ERR("System Not Yet Started!");

			break;
		}

		if (vusb_se_fe_started_threads == VUSB_SE_FE_TRUE) {
			vusb_se_fe_send_message(0, VUSB_SECMD_TXT_LINK,
					VUSB_SECMD_TXT_START,
					VUSB_SECMD_TXT_GOOD);
			return;
		}

		vusb_se_fe_started_threads = VUSB_SE_FE_TRUE;

		/*
		 * There are five numerical parameters,
		 * this identifies the link that is
		 * to be started, and the parameters for it
		 */

		for (par = 0; par < 5; par++) {

			mptr =
			    vusb_se_fe_ersatz_strtok(NULL,
						     VUSB_SECMD_TXT_SEPARATOR);

			if (!mptr) {

				VUSB_SE_FE_ERR("Empty Parameter[%d]!", par);

				break;
			}

			pval[par] = vusb_se_fe_ersatz_strtoi(mptr);
		}

		VUSB_SE_FE_LOG("Start ...");
		VUSB_SE_FE_LOG("...... Link ID %d", pval[0]);
		VUSB_SE_FE_LOG(".... .. DATA PIPE ID %d", pval[1]);
		VUSB_SE_FE_LOG(".... .. BufSiz %d", pval[2]);
		VUSB_SE_FE_LOG(".... .. SIO ID %d", pval[3]);
		VUSB_SE_FE_LOG(".... Gadget ID %d", pval[4]);

		if (pval[0] > vusb_se_fe_num_links) {

			VUSB_SE_FE_ERR("Bad Link Number!");

			break;
		}

		if ((vusb_se_fe_link_mvpipe_fp[pval[0]] != NULL)
		    || (vusb_se_fe_link_gadget_fp[pval[0]] != NULL)
		    ) {

			VUSB_SE_FE_ERR("File Pointers Already Set!");

			break;
		}

		/* store the values of the gadget and mvpipe
		 */
		vusb_se_fe_map_link_idtable[pval[0]].link_id = pval[0];
		vusb_se_fe_map_link_idtable[pval[0]].gadget = pval[4];
		vusb_se_fe_map_link_idtable[pval[0]].data_pipe_id = pval[1];

		/* Open the data source/sink as file pointers
		 * - note that the MVPIPE one will open directly
		 * (we can assume that mvpipe operation has started,
		 * since we have received a command over a mvpipe!)
		 * but that the Linux gadget pipe may not open directly
		 * - after all, there may be no ACM ports in the current USB
		 * configuration!
		 */
		sprintf(mvpipe_name, VUSB_SE_FE_MVPIPE_GENERIC_DATA_NAME, pval[1]);
		vusb_se_fe_link_mvpipe_fp[pval[0]] =
		    vusb_se_fe_open_mvpipe(mvpipe_name);

		vusb_se_fe_link_gadget_fp[pval[0]] =
		    vusb_se_fe_open_gadget(pval[4]);

		/* Start RX ( MEX-> Linux ) and TX ( Linux -> MEX ) threads
		 */
		sprintf(thread_name, "VUSB_SE_FE_LINK_RX_%d", pval[0]);

		VUSB_SE_FE_LOG("Starting Thread [%s]", thread_name);
		vusb_data->uint_val = pval[0];
		kthread_run(vusb_se_fe_link_rx_thread, (void *)vusb_data,
			    thread_name);

		sprintf(thread_name, "VUSB_SE_FE_LINK_TX_%d", pval[0]);

		VUSB_SE_FE_LOG("Starting Thread [%s]", thread_name);

		kthread_run(vusb_se_fe_link_tx_thread, (void *)vusb_data,
			    thread_name);

		/* Start Modem event watcher thread
		 */
		sprintf(thread_name, "VUSB_SE_FE_MODEM_%d", pval[0]);

		VUSB_SE_FE_LOG("Starting Thread [%s]", thread_name);

		kthread_run(vusb_se_fe_modem_event_thread, (void *)vusb_data,
			    thread_name);
		/* And notify the back end that things are running
		 */
		vusb_se_fe_send_message(pval[0], VUSB_SECMD_TXT_LINK,
					VUSB_SECMD_TXT_START,
					VUSB_SECMD_TXT_GOOD);

		return;
	}

	/*
	 * If the 'return' above was not reached,
	 * it is because of an earlier 'break'
	 * indicating a failure, so send a message
	 * to let the back end know
	 */
	vusb_se_fe_send_message(pval[0], VUSB_SECMD_TXT_LINK,
				VUSB_SECMD_TXT_START, VUSB_SECMD_TXT_FAIL);

	return;
}

/* ------------------------------------------------------------------------- */

#define VUSB_SE_FE_HANDLE_SIGNAL(msig, signal, state) {\
	if (VUSB_SE_FE_MODEM_STATE_ON == state)\
		msig |= signal;\
	else\
		msig &= (~signal);\
}

static void vusb_se_fe_handle_modem_signal(void)
{
	char *mptr;

	int par;

	unsigned int link = VUSB_SECMD_MAX_LINKS;

	char *signal = NULL;
	char *state = NULL;

	enum vusb_se_fe_modem_signal modemsignal =
		VUSB_SE_FE_MODEM_SIGNAL_UNKNOWN;
	enum vusb_se_fe_modem_state modemstate =
		VUSB_SE_FE_MODEM_STATE_UNKNOWN;

	int msig = 0;
	int old_msig = 0;

	int scan;

	mm_segment_t old_fs;

	struct file *fp;

	long diagnostic;

	/* There is one numerical parameter that identifies the link (and hence
	 * the serial gadget) and two that give the modem signal and it's state
	 */
	for (par = 0; par < 3; par++) {

		mptr = vusb_se_fe_ersatz_strtok(NULL, VUSB_SECMD_TXT_SEPARATOR);

		if (!mptr) {

			VUSB_SE_FE_ERR("Empty Parameter[%d]!", par);

			return;
		}

		VUSB_SE_FE_LOG("Param[%d] = %s", par, mptr);

		switch (par) {

		case 0:

			link = vusb_se_fe_ersatz_strtoi(mptr);
			break;

		case 1:

			signal = mptr;
			break;

		case 2:

			state = mptr;
			break;

		default:

			VUSB_SE_FE_ERR("Keep klocwork quiet!");
			break;
		}
	}

	/* We ought to have all three items now, error if not
	 */
	if ((link > (VUSB_SECMD_MAX_LINKS - 1))
	    || (NULL == signal)
	    || (NULL == state)
	    ) {

		VUSB_SE_FE_ERR("Bad Params!");

		return;
	}

	VUSB_SE_FE_LOG("Modem[%d] %s : %s", link, signal, state);

	/*
	 * Having collected the strings out
	 * of the command, convert to defined (enum)
	 * values (by table search) radsy to send the
	 * modem event off to the apt channel
	 */
	for (scan = 0; scan <
			ARRAY_SIZE(vusb_se_fe_modem_signaltable); scan++) {

		struct vusb_se_fe_scan *eptr =
			&vusb_se_fe_modem_signaltable[scan];

		if (strcmp(signal, eptr->text) == 0) {

			modemsignal = (enum vusb_se_fe_modem_signal) eptr->eval;

			break;
		}
	}

	for (scan = 0; scan < ARRAY_SIZE(vusb_se_fe_modem_statetable); scan++) {

		struct vusb_se_fe_scan *eptr =
			&vusb_se_fe_modem_statetable[scan];

		if (strcmp(state, eptr->text) == 0) {

			modemstate = (enum vusb_se_fe_modem_state) eptr->eval;

			break;
		}
	}

	if ((modemsignal == VUSB_SE_FE_MODEM_SIGNAL_UNKNOWN)
	    || (modemstate == VUSB_SE_FE_MODEM_STATE_UNKNOWN)
	    ) {

		VUSB_SE_FE_ERR("Bad Unknown Signal/State!");

		return;
	}

	/* Temporary disable handling modem signals
	 */
	VUSB_SE_FE_LOG("TODO: Handle Signal");
	return;

	/* We now know the link and the signal and state, send it into
	 * Linux for the apt file
	 */
	fp = vusb_se_fe_link_gadget_fp[link];

	old_fs = get_fs();

	set_fs(KERNEL_DS);
	{
		diagnostic =
		    fp->f_op->unlocked_ioctl(fp, TIOCMGET,
					     (unsigned long)&msig);
	}
	set_fs(old_fs);

	if (-1 == diagnostic) {

		VUSB_SE_FE_ERR("Failed to get Modem State for Link %d", link);

		return;
	}

	old_msig = msig;

	switch (modemsignal) {

	case VUSB_SE_FE_MODEM_SIGNAL_DTR:
		VUSB_SE_FE_HANDLE_SIGNAL(msig, modemstate, TIOCM_DTR);
		break;
	case VUSB_SE_FE_MODEM_SIGNAL_DSR:
		VUSB_SE_FE_HANDLE_SIGNAL(msig, modemstate, TIOCM_DSR);
		break;
	case VUSB_SE_FE_MODEM_SIGNAL_RTS:
		VUSB_SE_FE_HANDLE_SIGNAL(msig, modemstate, TIOCM_RTS);
		break;
	case VUSB_SE_FE_MODEM_SIGNAL_CTS:
		VUSB_SE_FE_HANDLE_SIGNAL(msig, modemstate, TIOCM_CTS);
		break;
	case VUSB_SE_FE_MODEM_SIGNAL_DCD:
		VUSB_SE_FE_HANDLE_SIGNAL(msig, modemstate, TIOCM_CAR);
		break;

	default:

		VUSB_SE_FE_LOG("Modem Signal %s Not Handled", signal);

		break;
	}

	if (old_msig == msig) {

		VUSB_SE_FE_LOG("Modem Signals Unchanged");
	} else {

		old_fs = get_fs();

		set_fs(KERNEL_DS);
		{
			diagnostic =
			    fp->f_op->unlocked_ioctl(fp, TIOCMSET,
						     (unsigned long)&msig);
		}
		set_fs(old_fs);

		if (-1 == diagnostic) {

			VUSB_SE_FE_ERR("Failed to set Modem State for Link %d",
				       link);
		}
	}
}

/*****************************************************************************/
/* Function:... vusb_se_fe_send_message                                      */
/* Description: This function handles all messages to the other side         */
/*                                                                           */
/*              Semaphore protection on pipe writer is needed since there    */
/*              is an individual serial gadget reader thread for each of     */
/*              the SIO extender links as well as the single command link    */
/*****************************************************************************/

static void vusb_se_fe_send_message(int channel, char *item, char *signal,
				    char *state)
{
	char text[VUSB_SECMD_MAX_MESSAGE];

	int diagnostic;

	int must_write;
	int have_written;
	int previously_written;

	mm_segment_t old_fs;

	/* Ensure only one thread executes this operation
	 */
	diagnostic = mutex_lock_interruptible(&vusb_se_fe_tx_lock);

	if (diagnostic) {

		VUSB_SE_FE_ERR("Could Not process Tx Lock (Err=%d)",
			       diagnostic);

		return;
	}

	/* Note use of string concatenation here in the format string
	 */
	must_write = snprintf(text, sizeof(text),
			      "%d" VUSB_SECMD_TXT_SEPARATOR
			      "%s" VUSB_SECMD_TXT_SEPARATOR
			      "%s" VUSB_SECMD_TXT_SEPARATOR
			      "%s", channel, item, signal, state);

	VUSB_SE_FE_LOG("Sending [%s]", text);

	/* Make sure the trailing NUL is sent as well as the text
	 */
	must_write++;

	VUSB_SE_FE_INF("Need to Write %d chars", must_write);

	/* Try to write to the mvpipe.
	 */
	previously_written = 0;

	while (must_write > 0) {

		old_fs = get_fs();

		set_fs(KERNEL_DS);
		{
			have_written =
			    vusb_se_fe_command_mvpipe_fp->f_op->
			    write(vusb_se_fe_command_mvpipe_fp,
				  text + previously_written, must_write, 0);
		}
		set_fs(old_fs);

		if (have_written < 0) {

			VUSB_SE_FE_ERR("File Write Problem");

			break;
		}

		VUSB_SE_FE_INF("Wrote %d chars", have_written);

		if (have_written != must_write) {

			VUSB_SE_FE_LOG("MVPIPE Short Write %d:%d", have_written,
				       must_write);

			must_write -= have_written;

			previously_written += have_written;

			continue;
		}

		break;
	}

	mutex_unlock(&vusb_se_fe_tx_lock);
}

/*****************************************************************************/
/* Function:... vusb_se_fe_command_thread                                    */
/* Description: This function is called as the reader daemon and sits in a   */
/*              forever-loop collecting messages from the back end and then  */
/*              parsing them.                                                */
/*                                                                           */
/*              Incoming messages are either for initialisation and setup    */
/*              or are run-time modem state changes to be passed to links    */
/*****************************************************************************/

static int vusb_se_fe_command_thread(void *d)
{
	/* Use a local buffer in the thread's stack since Linux Kernel advice
	 * is not to attempt to use 'malloc' functionality
	 */
	char read_buff[VUSB_SECMD_BUFFER_SIZE];

	unsigned int pipe;

	int count;
	unsigned data = ((struct vusb_se_data *)d)->uint_val;

	if (!data) {

		VUSB_SE_FE_ERR("Bad thread parameter");

		return -1;
	} else {

		pipe = data;
	}

	/* Wait a while for Linux start to stabilise...
	 */
	for (count = 0; count < 5; count++) {

		msleep(1000);

		VUSB_SE_FE_LOG("Command Thread Waiting [%d]", count);
	}

	/* Set up the single mutex that protects write-back to MEX
	 */
	mutex_init(&vusb_se_fe_tx_lock);
	mutex_init(&vusb_dat_reopen_lock);

	/* open the mvpipe
	 */
	VUSB_SE_FE_LOG("Opening Command MVPIPE: %s",
		VUSB_SE_FE_MVPIPE_CMD_NAME);

	vusb_se_fe_command_mvpipe_fp =
		vusb_se_fe_open_mvpipe(VUSB_SE_FE_MVPIPE_CMD_NAME);

	/* check for the NULL pointer return. purely for klocwork.
	 */
	if (NULL == vusb_se_fe_command_mvpipe_fp)
		return 0;

	/* open is successful, read from pipe
	 */
	while (!kthread_should_stop()) {

		int have_read = 0;
		int this_read = 0;
		int have_used = 0;

		mm_segment_t old_fs;

		VUSB_SE_FE_INF("Command MVPIPE about to read.");

		old_fs = get_fs();

		set_fs(KERNEL_DS);
		{
			this_read =
			    vusb_se_fe_command_mvpipe_fp->f_op->
			    read(vusb_se_fe_command_mvpipe_fp,
				 &read_buff[have_read],
				 VUSB_SECMD_BUFFER_SIZE - have_read, 0);
		}
		set_fs(old_fs);

		if (this_read <= 0) {

			if (0 == this_read) {

				VUSB_SE_FE_INF
				("Command MVPIPE has nothing to read from MEX");
			} else {

				VUSB_SE_FE_ERR("Command MVPIPE MEX Read Error");
				/* re-open it */
				if (mutex_lock_interruptible
					(&vusb_se_fe_tx_lock))
					return 0;
				vusb_se_fe_close_mvpipe(vusb_se_fe_command_mvpipe_fp);
				msleep(1000);
				vusb_se_fe_command_mvpipe_fp =
					vusb_se_fe_open_mvpipe
						(VUSB_SE_FE_MVPIPE_CMD_NAME);

				mutex_unlock(&vusb_se_fe_tx_lock);

				/* check for the NULL pointer return. purely for klocwork.
				 */
				if (NULL == vusb_se_fe_command_mvpipe_fp)
					return 0;
			}

			msleep(1000);
		} else {

			/* Read something, but does it have the terminator at
			 * the end?
			 */
			have_read += this_read;

			VUSB_SE_FE_LOG("Command MVPIPE Read %d bytes, total %d",
				       this_read, have_read);

			read_buff[have_read] = 0;

			VUSB_SE_FE_INF("Read [%s]", read_buff);

			if (0 == read_buff[have_read - 1]) {

				VUSB_SE_FE_INF
				("Command MVPIPE Complete Message of %d Bytes",
				     have_read);

				/*
				 * The incoming might contain more
				 * than one message, so since each
				 * is NUL-terminated, advance along
				 * the buffer until the messages
				 * are all parsed
				 */
				have_used = 0;

				while (have_used < have_read) {

					char *parse_this =
					    &read_buff[have_used];

					vusb_se_fe_parse_message(parse_this);

					have_used += (1 + strlen(parse_this));
				}

				have_read = 0;
			} else {

				VUSB_SE_FE_LOG
				("Command MVPIPE No terminator, read again");
			}
		}
	}

	return 0;
}

/*****************************************************************************/
/* Function:... vusb_se_fe_do_link_txfr                                      */
/* Description: Generic function to read from one pipe and write to another  */
/*****************************************************************************/

static int vusb_se_fe_do_link_txfr(unsigned int link, char *tag,
				   struct file **rx_fps, struct file **tx_fps)
{
	int have_read;
	int have_used;
	int rx_offset;
	struct file *rx_fp = NULL;
	struct file *tx_fp = NULL;

	/* Use a local buffer in the thread's stack since Linux Kernel advice
	 * is not to attempt to use 'malloc' functionality - this does reduce
	 * the maximum size for each read, but it should be OK since these are
	 * stream operations...
	 */
	char read_buf[VUSB_SE_FE_DATA_BUFSIZ];

	mm_segment_t old_fs;

	VUSB_SE_FE_INF("%s[%d] about to read", tag, link);

	if (NULL == rx_fps || NULL == tx_fps)
		return 0;

	old_fs = get_fs();

	set_fs(KERNEL_DS);
	{
		/* get file pointer to the specific link
		 */
		rx_fp = rx_fps[link];

		if (NULL != rx_fp) {
			have_read =
			    rx_fp->f_op->read(rx_fp, read_buf,
					      VUSB_SE_FE_DATA_BUFSIZ, 0);
		} else {
			have_read = 0;
		}

	}
	set_fs(old_fs);

	if (have_read <= 0) {

		if (have_read < 0) {

			VUSB_SE_FE_ERR("%s[%d] Read Error %d", tag, link,
				       have_read);
		}

		msleep(1000);

		return (0 ==
			have_read) ? VUSB_SE_FE_TXFR_RX_NUL :
		    VUSB_SE_FE_TXFR_RX_ERR;
	}

	if (have_read == 1) {

		char c = read_buf[0];

		VUSB_SE_FE_INF("%s[%d] Sees [0x%02X] [%c]", tag, link, c,
			       isprint(c) ? c : '-');
	} else {

		VUSB_SE_FE_INF("%s[%d] Read %d", tag, link, have_read);
	}

	rx_offset = 0;

	while (have_read) {

		old_fs = get_fs();

		set_fs(KERNEL_DS);
		{
			if (mutex_lock_interruptible(&vusb_dat_reopen_lock))
				return 0;

			/* get file pointer to the specific link
			 */
			tx_fp = tx_fps[link];

			if (NULL != tx_fp) {
				have_used =
				    tx_fp->f_op->write(tx_fp,
						       &read_buf[rx_offset],
						       have_read, 0);
			} else {
				have_used = 0;
			}

			mutex_unlock(&vusb_dat_reopen_lock);
		}
		set_fs(old_fs);

		if (have_used <= 0) {

			if (have_used < 0) {

				VUSB_SE_FE_ERR("%s[%d] Write Error %d", tag,
					       link, have_used);
			}

			msleep(1000);

			return (0 ==
				have_used) ? VUSB_SE_FE_TXFR_TX_NUL :
			    VUSB_SE_FE_TXFR_TX_ERR;
		}

		VUSB_SE_FE_INF("%s[%d] Wrote %d", tag, link, have_used);

		rx_offset += have_used;

		have_read -= have_used;
	}

	return VUSB_SE_FE_TXFR_OK;
}

/*****************************************************************************/
/* Function:... vusb_se_fe_link_rx_thread                                    */
/* Description: This function is called as the reader daemon for mvpipe and  */
/*              runs a forever-loop collecting data and sends it off to the  */
/*              matching linux gadget                                        */
/*              This function supports multiple instances of the link as     */
/*              defined by the parameter 'data'                              */
/*****************************************************************************/

static int vusb_se_fe_link_rx_thread(void *d)
{
	unsigned int link;
	char mvpipe_name[32];

	int diagnostic = 1;
	unsigned data = ((struct vusb_se_data *)d)->uint_val;
	struct file *fp;

	VUSB_SE_FE_CHECK_AND_SET_LINK(data, link, "BE-to-FE");

	/* Now read/write forever
	 */
	while (!kthread_should_stop()) {

		diagnostic =
		    vusb_se_fe_do_link_txfr(link, "BE-to-FE",
					    vusb_se_fe_link_mvpipe_fp,
					    vusb_se_fe_link_gadget_fp);

		if (VUSB_SE_FE_TXFR_RX_ERR == diagnostic) {
			/* re-open it */
			if (mutex_lock_interruptible(&vusb_dat_reopen_lock))
				return 0;

			if (vusb_se_fe_link_mvpipe_fp[link] != NULL) {
				fp = vusb_se_fe_link_mvpipe_fp[link];
				vusb_se_fe_close_mvpipe(fp);
			}
			msleep(1000);
			sprintf(mvpipe_name,
				VUSB_SE_FE_MVPIPE_GENERIC_DATA_NAME, link);

			vusb_se_fe_link_mvpipe_fp[link] =
				vusb_se_fe_open_mvpipe(mvpipe_name);

			mutex_unlock(&vusb_dat_reopen_lock);
		}

	}

	return (int)diagnostic;
}

/*****************************************************************************/
/* Function:... vusb_se_fe_link_tx_thread                                    */
/* Description: This function is called as the reader daemon for the linux   */
/*              gadget and runs a forever-loop collecting data and sends it  */
/*              off to the matching mvpipe                                   */
/*              This function supports multiple instances of the link as     */
/*              defined by the parameter 'data'                              */
/*****************************************************************************/

static int vusb_se_fe_link_tx_thread(void *d)
{
	unsigned int link;

	int diagnostic = 1;

	unsigned int index = 0;
	unsigned data = ((struct vusb_se_data *)d)->uint_val;

	VUSB_SE_FE_CHECK_AND_SET_LINK(data, link, "FE-to-BE");

	/* Now read/write forever
	 */
	while (!kthread_should_stop()) {

		diagnostic =
		    vusb_se_fe_do_link_txfr(link, "FE-to-BE",
					    vusb_se_fe_link_gadget_fp,
					    vusb_se_fe_link_mvpipe_fp);

		if (VUSB_SE_FE_TXFR_RX_NUL == diagnostic) {

			/* find out the gadget
			 */
			while (vusb_se_fe_map_link_idtable[index].link_id !=
			       link) {

				index++;
				if (index >= vusb_se_fe_num_links) {
					VUSB_SE_FE_ERR
					("index %d exceed number of links: %d",
					     index, vusb_se_fe_num_links);
					return 0;
				}
			}

			/* close the gadget filp
			 */
			if (NULL != vusb_se_fe_link_gadget_fp[link]) {

				vusb_se_fe_close_gadget
				    (vusb_se_fe_link_gadget_fp[link],
				     vusb_se_fe_map_link_idtable[index].gadget);

			}

			/* reopen the gadget and update filp
			 */
			vusb_se_fe_link_gadget_fp[link] =
			    vusb_se_fe_open_gadget(vusb_se_fe_map_link_idtable
						   [index].gadget);

			/* reset index
			 */
			index = 0;

		}

	}

	return (int)diagnostic;
}

/*****************************************************************************/
/* Function:... vusb_se_fe_modem_event_thread                                */
/* Description: This function is called as the handler for Modem events      */
/*              originating from the gadget (i.e. from the connected host)   */
/*              and runs forever, waiting for events                         */
/*              This function supports multiple instances of the link as     */
/*              defined by the parameter 'data'                              */
/*****************************************************************************/

static int vusb_se_fe_modem_event_thread(void *d)
{
	unsigned int link;
	unsigned data = ((struct vusb_se_data *)d)->uint_val;

	mm_segment_t old_fs;

	int msig = 0;

	struct file *fp;

	bool new_dsr = VUSB_SE_FE_FALSE;
	bool new_cts = VUSB_SE_FE_FALSE;

	bool old_dsr = VUSB_SE_FE_FALSE;
	bool old_cts = VUSB_SE_FE_FALSE;

	long diagnostic;

	VUSB_SE_FE_CHECK_AND_SET_LINK(data, link, "MDM");

	fp = vusb_se_fe_link_gadget_fp[link];

	/* Wait on modem events forever
	 */
	while (!kthread_should_stop()) {

		/* Temporary disable
		 */
		VUSB_SE_FE_LOG("Modem Event Thread [%d] Sleeps", link);
		msleep(5000);
		continue;
		VUSB_SE_FE_ERR("Modem Event Thread Should Not Reach Here!");

		/* Check to see what state the modem is in
		 */
		old_fs = get_fs();

		set_fs(KERNEL_DS);
		{
			diagnostic =
			    fp->f_op->unlocked_ioctl(fp, TIOCMGET,
						     (unsigned long)&msig);
		}
		set_fs(old_fs);

		if (-1 == diagnostic) {

			VUSB_SE_FE_ERR("Failed to get Modem State for Link %d",
				       link);

			continue;
		}

		/*
		 * Look for changes
		 * (also handles start-up where 'old' signals are false)
		 */
		new_dsr = VUSB_SE_FE_IS_BIT_SET(TIOCM_DSR, msig);
		new_cts = VUSB_SE_FE_IS_BIT_SET(TIOCM_CTS, msig);

		if (new_dsr != old_dsr) {

			vusb_se_fe_send_message(link, VUSB_SECMD_TXT_MODEM,
						VUSB_SECMD_TXT_SIGNAL_DTR,
						(new_dsr) ?
						VUSB_SECMD_TXT_STATE_ON :
						VUSB_SECMD_TXT_STATE_OFF);

			old_dsr = new_dsr;
		}

		if (new_cts != old_cts) {

			vusb_se_fe_send_message(link, VUSB_SECMD_TXT_MODEM,
						VUSB_SECMD_TXT_SIGNAL_CTS,
						(new_cts) ?
						VUSB_SECMD_TXT_STATE_ON :
						VUSB_SECMD_TXT_STATE_OFF);

			old_cts = new_cts;
		}

		/* Now block until modem state changes again
		 */
		old_fs = get_fs();

		set_fs(KERNEL_DS);
		{
			diagnostic =
			    fp->f_op->unlocked_ioctl(fp, TIOCMIWAIT,
						     (unsigned long)(TIOCM_DSR |
								    TIOCM_CTS));
		}
		set_fs(old_fs);

		if (-1 == diagnostic) {

			VUSB_SE_FE_ERR
			    ("Failed waiting for Modem State for Link %d",
			     link);

			continue;
		}
	}

	return 0;
}

/*****************************************************************************/
/* Function:... vusb_se_fe_init / vusb_se_fe_exit                            */
/* Description: Local entry and exit functions used solely to manage the     */
/*              internals of this module                                     */
/*              These functions are used to register and unregister drivers  */
/*              using the composite driver framework.                        */
/*****************************************************************************/

static int __init vusb_se_fe_init(void)
{
	int link;
	vusb_data = kzalloc(sizeof(struct vusb_se_data), GFP_KERNEL);
	if (!vusb_data) {
		VUSB_SE_FE_ERR("Alloc failed - out of memory");
		return -ENOMEM;
	}

	VUSB_SE_FE_LOG("INIT VUSB_SE_FE called");

	/* Initially there are no file pointers
	 */
	vusb_se_fe_command_mvpipe_fp = NULL;

	for (link = 0; link < VUSB_SECMD_MAX_LINKS; link++) {

		vusb_se_fe_link_mvpipe_fp[link] = NULL;
		vusb_se_fe_link_gadget_fp[link] = NULL;
	}

	/* Now start the command-handler thread that will open the specified
	 * mvpipe command channel and then wait for incoming setup commands
	 * from the back end, and after that, possible modem state changes
	 */
	vusb_data->uint_val = VUSB_SE_FE_COMMAND_MVPIPE_NUM;
	kthread_run(vusb_se_fe_command_thread,
			(void *)vusb_data,
		    "VUSB_SE_FE_MEX_MESSAGES");

	VUSB_SE_FE_LOG("INIT OK");

	return 0;
}

/* ------------------------------------------------------------------------- */

#ifdef GCOV_CODE_COVERAGE
static void vusb_se_fe_exit(void)
#else
static void __exit vusb_se_fe_exit(void)
#endif
{
#ifdef GCOV_CODE_COVERAGE
#endif
	kfree(vusb_data);
	VUSB_SE_FE_LOG("EXIT OK");
}

/*****************************************************************************/
/* Main Module Initialisation, simply maps to local functions                */
/*****************************************************************************/

module_init(vusb_se_fe_init);
module_exit(vusb_se_fe_exit);

/*****************************************************************************/
/* Licence Description                                                       */
/*****************************************************************************/

MODULE_DESCRIPTION("VLX Virtual USB SIO Extender Front End Driver");
MODULE_AUTHOR("Intel Mobile Communications GmbH");
MODULE_LICENSE("GPL");
