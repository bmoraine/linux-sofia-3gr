/*
* Copyright (C) 2013 Intel Mobile Communications GmbH
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

#include <linux/kernel.h>	/* printk() */
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>		/* character device */
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/workqueue.h>	/*for events*/
#include <linux/delay.h>	/* for msleep*/
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/device.h>	/*for class & device */
#include <linux/slab.h>		/*for kmalloc, kfree */
#include <linux/interrupt.h>	/*for irq */
#include <linux/delay.h>	/*for events */
#include <linux/io.h>		/*for ioremap */
#include <asm/termios.h>	/* for file attribute */
#include <linux/dma-mapping.h>	/* for DMA alloc */
#include <linux/poll.h>
#include <linux/device_pm_data.h>
#include <linux/platform_device.h>/*register platform for PM callbacks */
#include <linux/pm_runtime.h>	/*PM runtime*/
#include <linux/of.h>
#include <linux/timer.h>	/*for page timeout */

#include "oct2_io.h"
#include "oct2.h"

#include <linux/tadi.h>
#include <linux/time.h>
/*constants */

#define OCT_MODULE_NAME "oct"
#define PLATFORM_DRIVER_NAME "oct2_drv"

#define OCT_DRIVER_VER 1

#define TRACE_DEBUG_OCT_FULL_MANAGEMENT 0
/*oct hw constants */

#define TRACE_DEBUG_OCT_TIMEOUT  0x80
#define OCT_REG_ADDRESS_BASE oct2_reg
#define OCT_REG_SIZE sizeof(struct _sOct)
#define OCT_INT oct2_int

#define USB_CH_NAME "/dev/ttyGS1"
#define USB_DEBUG_CH "/dev/ttyGS2"

#ifdef OCT_HOST_TEST
/*these macros will be replaced */
#define USB_CH_NAME "/dev/ttyS0"
#define OCT_INT 1 /*keyboard int*/
#endif

enum oct2_onOff_enum {
	TRACE_DEBUG_OFF = 0,
	TRACE_DEBUG_ON
};

enum oct2_onOff_enum oct2_on = TRACE_DEBUG_ON;

#define TRACE_DEBUG_OCT_DATA_RX_EVENT 1
#define TRACE_DEBUG_OCT_DATA_ACK_EVENT 2
#define TRACE_DEBUG_OCT_DATA_FULL_EVENT 4
#define TRACE_DEBUG_OCT_RELOAD_PG_TAB_EVENT  8
#define TRACE_DEBUG_OCT_WRAP_EVENT 16
#define TRACE_DEBUG_OCT_DATA_ALL_EVENTS (1 | 2 | 4 | 8 | 16)

/*common macros */
#define OCT2_GET_RINGBUF_BASE_ADR \
	(oct2_page_table[0].buff_desc.base << 2)
#define OCT2_GET_RINGBUFF_WRITE_PTR(index) \
	((oct2_page_table[index].buff_desc.base << 2)\
	+ OCT2_GET_PAGE_WPTR)

/* !!! debug stuff !!!*/
#define OCT_DEBUG_OVER_USB_CH 0

static unsigned int debug_on;
static unsigned int debug_over_usb;

static void USB_printk(const char *fmt, ...);
#define OCT_DBG(fmt, arg...) {\
		if (debug_over_usb)\
			USB_printk(fmt, ##arg);\
		if (debug_on)\
			pr_info(OCT_MODULE_NAME": " fmt"\n", ##arg); }

#define OCT_LOG(fmt, arg...) do {\
	if (debug_over_usb)\
		USB_printk(fmt, ##arg);\
	else\
		pr_info(OCT_MODULE_NAME": " fmt"\n", ##arg);\
	} while (0)

enum oct2_info_enum {
	OCT2_INFO_SEND = 1, /* val1 = offset; val2 = size */
	OCT2_INFO_ACKN,
	OCT2_INFO_LISR,
	OCT2_INFO_EVNT,
	OCT2_INFO_EMPT,
	OCT2_INFO_FULL,
	OCT2_INFO_REST,
	OCT2_INFO_WRAP,
	OCT2_INFO_MODE,
	OCT2_INFO_SETU,
	OCT2_INFO_UNKN = 0
};
struct oct2_debug_struct {
	unsigned int gts;
	enum oct2_info_enum info;
	unsigned short page_idx;
	unsigned int val1;
	unsigned int val2;
};
static struct oct2_debug_struct oct2_dbg[0x100];
static unsigned char oct2_dbg_idx;
#define OCT2_LOG(_info, _index, _val1, _val2) \
		oct2_dbg_idx++;\
		oct2_dbg[oct2_dbg_idx].gts = oct2_get_time_ms();\
		oct2_dbg[oct2_dbg_idx].info = _info;\
		oct2_dbg[oct2_dbg_idx].val1 = _val1;\
		oct2_dbg[oct2_dbg_idx].val2 = (_val2);\
		oct2_dbg[oct2_dbg_idx].page_idx = _index;

/*--------------------- buffer descriptor -----------------------------------*/

#define OCT_PAGE_SIZE_4K	0x00
#define OCT_PAGE_SIZE_8K	0x01
#define OCT_PAGE_SIZE_16K	0x02
#define OCT_PAGE_SIZE_32K	0x03
#define OCT_PAGE_SIZE_64K	0x04
#define OCT_PAGE_SIZE_128K	0x05
#define OCT_PAGE_SIZE_256K	0x06
#define OCT_PAGE_SIZE_512K	0x07
#define OCT_PAGE_SIZE_1MB	0x08
#define OCT_PAGE_SIZE_2MB	0x09
#define OCT_PAGE_SIZE_4MB	0x0A
#define OCT_PAGE_SIZE_8MB	0x0B
#define OCT_PAGE_SIZE_16MB	0x0C
#define OCT_PAGE_SIZE_32MB	0x0D
#define OCT_PAGE_SIZE_64MB	0x0E
#define OCT_PAGE_SIZE_128MB	0x0F

#define OCT_PAGE_SIZE OCT_PAGE_SIZE_32K
#define OCT2_DMA_PAGE_SIZE (0x1000<<OCT_PAGE_SIZE)
#define OCT_NUM_OF_PAGES (OCT_EXT_RING_BUFF_SIZE / OCT2_DMA_PAGE_SIZE)
#define OCT_EXT_RING_BUFF_SIZE_ADJ (OCT2_DMA_PAGE_SIZE * OCT_NUM_OF_PAGES)


struct oct2_buff_desc_struct {
	unsigned int index:16;
	unsigned int size:4;
	unsigned int reserved:10;
	unsigned int refill:1; /* swap with full*/
	unsigned int restart:1;
	unsigned int forward:1;
	unsigned int full:1;   /* swap with refill */
	unsigned int base:30;
	unsigned char *base_virt;
	struct oct2_buff_desc_struct *next;
	struct oct2_buff_desc_struct *prev;
};

union oct2_buff_desc_union {
	struct {
		unsigned int buff_desc_lo;
		unsigned int buff_desc_hi;
	} content;
	struct oct2_buff_desc_struct buff_desc;
};

static union oct2_buff_desc_union oct2_page_table[OCT_NUM_OF_PAGES];

static unsigned char pg_curr_idx;
static unsigned char pg_read_idx;
static unsigned char open_num_pages;
static unsigned int open_num_bytes;
static bool TraceDebugOctBgFULL;
static bool TraceDebugOctBuffEmpty = 1;
static unsigned int TraceDebugOctBuffWrap;
static unsigned int TraceDebugOctExtMemFull;


/* Local variables */
static void * __iomem oct2_reg;
static unsigned int oct2_int;
static int major;
static int minor;
static struct class *class_oct;
static struct file *fp, *fp_debug;
static struct task_struct *task;
static wait_queue_head_t oct2_wq, oct2_poll_wq;
static wait_queue_head_t oct2_read_wq;
static dma_addr_t phy_base_addr;

static int oct2_ext_ring_buff_len = OCT_EXT_RING_BUFF_SIZE_ADJ;
static unsigned int oct2_mode = DEFAULT_OCT_MODE;
static unsigned int current_timeout = OCT2_PG_TIMEOUT_RUN;
static unsigned int oct2_out_path = DEFAULT_OCT_PATH;

static void *oct2_ext_rbuff_ptr;
static unsigned char (*oct2_buff)[OCT_NUM_OF_PAGES][OCT2_DMA_PAGE_SIZE];
static unsigned int oct2_ext_mem_full;
static unsigned int oct2_irq;
static unsigned int oct2_events;
static unsigned int oct2_read_ptr;
static unsigned int oct2_write_ptr;
static unsigned int data_available;
static unsigned int count_read_time_outs;
static unsigned int count_too_many_bytes;
static unsigned int count_error_transfers;
static unsigned int count_transfers;
static unsigned int count_read_event_wakeup;
static unsigned int count_oct2_tasklet_usb;
static unsigned int oct2_disable_irq_during_sleep;
static struct timer_list oct2_page_timer;

/*Settings comming from MEX */
#define OCT_FCS_SIZE_16 1
#define OCT_FCS_SIZE_32 0
static unsigned int oct2_fcs_size = OCT_FCS_SIZE_32;


/*static struct platform_device *my_platform_device;*/
static struct device_pm_platdata *oct2_pm_platdata;


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void oct2_trap(int value)
{
	OCT_DBG("TRAP");
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void oct2_generate_page_table(unsigned int mode)
{
	unsigned char i;
	struct oct2_buff_desc_struct *p_page_descriptor;
	/*set first page */
	p_page_descriptor = &oct2_page_table[0].buff_desc;
	p_page_descriptor->base = phy_base_addr>>2 & 0x3FFFFFFF;
	p_page_descriptor->base_virt = &(*oct2_buff)[0][0];
	p_page_descriptor->index = 0;
	p_page_descriptor->forward = (mode == OCT_MODE_STALL)?1:0;
	p_page_descriptor->refill = 0;
	p_page_descriptor->restart = 0;
	p_page_descriptor->full = 0;
	p_page_descriptor->size = OCT_PAGE_SIZE;
	p_page_descriptor->next = &oct2_page_table[1].buff_desc;
	p_page_descriptor->prev =
		&oct2_page_table[OCT_NUM_OF_PAGES-1].buff_desc;
	/*set pages between 2 -> (n-1)*/
	for (i = 1; i < OCT_NUM_OF_PAGES - 1; i++) {
		p_page_descriptor = &oct2_page_table[i].buff_desc;
		p_page_descriptor->base =
		  ((phy_base_addr + i*OCT2_DMA_PAGE_SIZE)>>2) & 0x3FFFFFFF;
		p_page_descriptor->base_virt = &(*oct2_buff)[i][0];
		p_page_descriptor->index = i;
		p_page_descriptor->forward = (mode == OCT_MODE_STALL)?1:0;
		p_page_descriptor->refill = 0;
		p_page_descriptor->restart = 0;
		p_page_descriptor->full = 0;
		p_page_descriptor->size = OCT_PAGE_SIZE;
		p_page_descriptor->next = &oct2_page_table[i+1].buff_desc;
		p_page_descriptor->prev = &oct2_page_table[i-1].buff_desc;
	}
	/*set the last page*/
	p_page_descriptor = &oct2_page_table[OCT_NUM_OF_PAGES-1].buff_desc;
	p_page_descriptor->base = ((phy_base_addr
		+ OCT_EXT_RING_BUFF_SIZE_ADJ - OCT2_DMA_PAGE_SIZE)>>2)
		& 0x3FFFFFFF;
	p_page_descriptor->base_virt = &(*oct2_buff)[i][0];
	p_page_descriptor->index = i;
	p_page_descriptor->forward = (mode == OCT_MODE_STALL)?1:0;
	p_page_descriptor->refill = 0;
	p_page_descriptor->restart = 1;
#if TRACE_DEBUG_OCT_FULL_MANAGEMENT
	p_page_descriptor->full = (mode == OCT_MODE_STALL)?1:0;
#else
	p_page_descriptor->full = 0;
#endif
	p_page_descriptor->size = OCT_PAGE_SIZE;
	p_page_descriptor->next =
		&oct2_page_table[0].buff_desc;
	p_page_descriptor->prev =
		&oct2_page_table[OCT_NUM_OF_PAGES-2].buff_desc;
}

void oct2_load_page_table(void)
{
	unsigned char i;
	OCT2_CLR_CNF_PTBY;
	/* copy OCT page table to registers*/
	for (i = 0; i < OCT_NUM_OF_PAGES; i++) {
		OCT2_SET_PGT_MEM0(i, oct2_page_table[i].content.buff_desc_lo);
		OCT2_SET_PGT_MEM1(i, oct2_page_table[i].content.buff_desc_hi);
	}
	OCT2_SET_CNF_PTBY;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

#define GET_TIME_MS(timp) ((unsigned int)\
	(timp.tv_sec * 1000 + timp.tv_usec/1000))
static unsigned int oct2_get_time_ms(void)
{
	static unsigned int ref_time;
	static unsigned int currTime;
	struct timeval time;
	do_gettimeofday(&time);
	if (ref_time == 0)
		ref_time = GET_TIME_MS(time);
	currTime = GET_TIME_MS(time) - ref_time;
	return currTime;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void USB_printk(const char *fmt, ...)
{
	mm_segment_t old_fs;
	int written_bytes;
	int num_bytes;
	char oct2_print_buffer[0x100];
	va_list args;

	num_bytes = sprintf(oct2_print_buffer, "[%d]: ", oct2_get_time_ms());

	va_start(args, fmt);
	num_bytes += vsprintf(&oct2_print_buffer[num_bytes], fmt, args);
	va_end(args);

	oct2_print_buffer[num_bytes++] = '\r';
	oct2_print_buffer[num_bytes++] = '\n';

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	written_bytes =
	  fp_debug->f_op->write(fp_debug, oct2_print_buffer, num_bytes, 0);

	set_fs(old_fs);
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void oct2_print_page_table(void)
{
	char buff[OCT_NUM_OF_PAGES+1];
	int i;
	for (i = 0; i < OCT_NUM_OF_PAGES; i++)
		if (oct2_page_table[i].buff_desc.full)
			buff[i] = '1';
		else
			buff[i] = '0';
	buff[i] = 0;
	USB_printk("pg_table: %s", buff);
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void oct2_enable_all_irqs(void)
{
	OCT2_ENABLE_ALL_IRQS;
	OCT_DBG("IRQs have been enabled");
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void oct2_disable_all_irqs(void)
{
	OCT2_DISABLE_ALL_IRQS;
	OCT_DBG("IRQs have been disabled");
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void oct2_reset_timeout_cnt(unsigned int val)
{
	mod_timer(&oct2_page_timer, jiffies + msecs_to_jiffies(val));
	OCT2_SET_PG_TIMEOUT(0x1FF);
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static ssize_t oct2_get_available_bytes_number(void)
{
	/*get page index & write Ptr */
	pg_curr_idx = OCT2_GET_PAGE_INDEX;
	oct2_write_ptr = OCT2_GET_PAGE_WPTR;
	if (TraceDebugOctBgFULL)
		open_num_pages = OCT_NUM_OF_PAGES;
	else if (pg_curr_idx >= pg_read_idx)
		open_num_pages = pg_curr_idx - pg_read_idx;
	else
		open_num_pages = OCT_NUM_OF_PAGES - (pg_read_idx - pg_curr_idx);
	OCT2_LOG(OCT2_INFO_SEND, pg_curr_idx, oct2_write_ptr, open_num_pages);
	if (open_num_pages) {
		open_num_bytes = OCT2_DMA_PAGE_SIZE - oct2_read_ptr;
	} else {
		/* always octWritePtr => octReadPtr in the same page */
		if (oct2_write_ptr < oct2_read_ptr)
			oct2_trap(oct2_write_ptr);
		open_num_bytes = oct2_write_ptr - oct2_read_ptr;
	}
	return (open_num_pages * OCT2_DMA_PAGE_SIZE + open_num_bytes);
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

/* power management section */

static int oct2_pm_suspend(struct device *dev)
{
	int irq;
	/*steps to move subsystem to system suspend safe state*/
	irq = OCT2_GET_IRQ_STAT;

	OCT_LOG("suspend: %d avail data; IRQ Stat: %d",
		oct2_get_available_bytes_number(), irq);
	/* adjust sleep TIMEOUT */
	oct2_reset_timeout_cnt(OCT2_PG_TIMEOUT_SLEEP);

	/* reset the interrupts */
	OCT2_CLEAR_IRQS(irq);
	if (oct2_disable_irq_during_sleep)
		oct2_disable_all_irqs();
	else
		oct2_enable_all_irqs();
	/* trace_debug_oct2_dma_off(); */
	return 0;
}

static int oct2_pm_resume(struct device *dev)
{
	int irq;

	/*steps to resume from suspend*/
	irq = OCT2_GET_IRQ_STAT;
	OCT_LOG("Resume: %d avail data; IRQ Stat: %x b2..0=(TO|L2|L1)",
		oct2_get_available_bytes_number(), irq);
	/* set CYCLE interrupt for running mode*/
	oct2_reset_timeout_cnt(current_timeout);

	if (oct2_disable_irq_during_sleep)
		oct2_enable_all_irqs();
	return 0;
}

static int oct2_pm_runtime_suspend(struct device *dev)
{
	OCT_DBG("Runtime suspend");
	/* steps to move subsystem to runtime suspend safe state */
	return 0;
}

static int oct2_pm_runtime_resume(struct device *dev)
{
	/* steps to resume from suspend */
	OCT_DBG("Runtime resume");
	return 0;
}

static int oct2_pm_runtime_idle(struct device *dev)
{
	/* This function is the entry point when runtime counter is 0
	 * This function must check if the condition(s)
	 * for suspend is/are fulfiled.
	 * Before allowing it to enter suspend
	 * This function will indirect reach runtime_suspend
	*/
	OCT_DBG("Runtime Idle");
	/* If abc conditions are met.. */
	pm_runtime_suspend(dev);
	return 0;
}

static const struct dev_pm_ops my_pm_ops = {
	/* For system suspend/resume */
	.suspend = oct2_pm_suspend,
	.resume = oct2_pm_resume,
	/* For runtime PM */
	.runtime_suspend = oct2_pm_runtime_suspend,
	.runtime_resume = oct2_pm_runtime_resume,
	.runtime_idle = oct2_pm_runtime_idle,
};

/* Tasklets for BH */
static void oct2_tasklet_bh(unsigned long data);
DECLARE_TASKLET(oct2_tasklet, oct2_tasklet_bh, 0);

static int __init oct2_init(void);
static void __exit oct2_exit(void);

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

irq_handler_t oct2_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	/*get IRQ status */
	oct2_irq = OCT2_GET_IRQ_STAT;
	/* disable interrupts, will be re-enabled if ext mem is empty*/
	OCT2_DISABLE_ALL_IRQS;
	OCT2_CLEAR_IRQS(oct2_irq);
	/*activate bottom half task */
	tasklet_schedule(&oct2_tasklet);
	return (irq_handler_t)IRQ_HANDLED;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void oct2_clr_bit(tcflag_t *p_flags, tcflag_t mask)
{
	OCT_DBG("CLR Bit 0x%07o", mask);
	OCT_DBG("---- Before 0x%07o", *p_flags);
	*p_flags &= ~mask;
	OCT_DBG("---- After  0x%07o", *p_flags);
}

static void oct2_set_bit(tcflag_t *p_flags, tcflag_t mask)
{
	OCT_DBG("SET Bit 0x%07o", mask);
	OCT_DBG("---- Before 0x%07o", *p_flags);
	*p_flags |= mask;
	OCT_DBG("---- After  0x%07o", *p_flags);
}

static void oct2_set_ccc(struct ktermios *attr,
					 unsigned int item, int val)
{
	if (item < NCCS) {
		OCT_DBG("Change c_cc value [%d]", item);
		OCT_DBG("---- Before %d", attr->c_cc[item]);
		attr->c_cc[item] = val;
		OCT_DBG("---- After  %d", attr->c_cc[item]);
	} else {
		OCT_DBG("Bad Item [%d] for Change C-CC", item);
	}
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void oct2_set_raw_gadget(struct file *fp)
{
	struct ktermios my_attr;

	/* Read the current set of terminal attribute flags & dump the result */
	fp->f_op->unlocked_ioctl(fp, TCGETS, (unsigned long) &my_attr);

	/* Modify the attributes to be what we want instead of the default */
	oct2_clr_bit(&my_attr.c_iflag, IGNBRK);
	oct2_clr_bit(&my_attr.c_iflag, BRKINT);
	oct2_clr_bit(&my_attr.c_iflag, PARMRK);
	oct2_clr_bit(&my_attr.c_iflag, ISTRIP);
	oct2_clr_bit(&my_attr.c_iflag, INLCR);
	oct2_clr_bit(&my_attr.c_iflag, IGNCR);
	oct2_clr_bit(&my_attr.c_iflag, ICRNL);
	oct2_clr_bit(&my_attr.c_iflag, IXON);

	oct2_clr_bit(&my_attr.c_oflag, OPOST);

	oct2_clr_bit(&my_attr.c_lflag, ECHO);
	oct2_clr_bit(&my_attr.c_lflag, ECHONL);
	oct2_clr_bit(&my_attr.c_lflag, ICANON);
	oct2_clr_bit(&my_attr.c_lflag, ISIG);
	oct2_clr_bit(&my_attr.c_lflag, IEXTEN);

	oct2_clr_bit(&my_attr.c_cflag, CSIZE);
	oct2_clr_bit(&my_attr.c_cflag, PARENB);
	oct2_set_bit(&my_attr.c_cflag, CS8);

	oct2_set_ccc(&my_attr, VTIME, 0);
	oct2_set_ccc(&my_attr, VMIN,  1);

	fp->f_op->unlocked_ioctl(fp, TCSETS, (unsigned long)&my_attr);
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static struct file *oct2_open_gadget(char *gadget_name)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	OCT_DBG("Opening Gadget[%s]", gadget_name);

	while (!kthread_should_stop()) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		{
			fp = filp_open(gadget_name, O_RDWR, 0);
		}
		set_fs(old_fs);
		if (NULL == fp) {
			OCT_LOG("NULL Gadget File Handle");
			msleep(2000);
			continue;
		} else if (IS_ERR(fp)) {
			OCT_LOG("Gadget %s open failed.", gadget_name);
			msleep(2000);
			continue;
		} else {
			OCT_LOG("Gadget %s open succeeded.", gadget_name);
			oct2_set_raw_gadget(fp);
			break;
		}
		/*set_current_state(TASK_INTERRUPTIBLE);*/
	}
	return fp;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void oct2_mode_off(void){}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void trace_debug_oct2_define_ringbuff(void)
{
	oct2_ext_rbuff_ptr = kzalloc(OCT_EXT_RING_BUFF_SIZE_ADJ, GFP_KERNEL);
	phy_base_addr = dma_map_single(NULL, oct2_ext_rbuff_ptr,
			OCT_EXT_RING_BUFF_SIZE_ADJ, DMA_FROM_DEVICE);
	/* oct2_buff will be handled as a matrix */
	oct2_buff = (unsigned char (*)[OCT_NUM_OF_PAGES][OCT2_DMA_PAGE_SIZE])
		oct2_ext_rbuff_ptr;

	OCT_DBG("DMA addr:%#x; KERN Addr:%p", (unsigned int)phy_base_addr,
			oct2_ext_rbuff_ptr);

	/* if NULL ptr or zero size defined -> Error
	 * or bad alignment */
	if (!(oct2_ext_rbuff_ptr && oct2_ext_ring_buff_len) ||
		((uintptr_t)oct2_ext_rbuff_ptr & 15) ||
		(oct2_ext_ring_buff_len & 15) ||
		(oct2_ext_ring_buff_len < 0x4000) ||
		(oct2_ext_ring_buff_len > 0x04000000)) {
			OCT_DBG("Define Ringbuff: Fatal error!");
			oct2_trap(0);
	}
	oct2_generate_page_table(OCT_MODE_STALL);
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static int oct2_check_power(void)
{
	if (getOct_OCT2_CLC_STAT(oct2_reg) != 0)
		return 1;
	else
		return 0;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void oct2_hw_init(void)
{
	unsigned int poly;

	/* switch to config mode */
	OCT2_SET_CONF_MODE;
	/* while configuring OCTM - switch to stall mode */
	OCT2_CONF_MANU_STALL(TRACE_DEBUG_ON);
	/* set frame timeout */
	OCT2_SET_FRAME_TIMEOUT(1);
	/* config OCTM to forward data to DMA controller */
	OCT2_CONF_OCTM_RINGBUFF(TRACE_DEBUG_OFF);
	/*start clearing OCTM */
	OCT_CLR_OCT_ON;
	/*reset the trace formatter drop status */
	OCT_CLR_TRFORM_DROP_STATUS;
	/* exit from OCTM config */
	OCT2_CONF_MANU_STALL(TRACE_DEBUG_OFF);
	/* it seems that OCTM clear must be done after */
	/* stall mode is off (different from old OCT) */
	OCT_CLR_OCT_OFF;
	/* allocate & generate DMA page table */
	trace_debug_oct2_define_ringbuff();
	/*DMA related settings */
	oct2_load_page_table();
	TraceDebugOctBgFULL = 0;
	oct2_read_ptr = 0;
	oct2_write_ptr = 0;
	pg_curr_idx = 0;
	pg_read_idx = 0;
	/* set page timeout to maximum. It will be triggered by sw timer*/
	OCT2_SET_PG_TIMEOUT(0x1FF); /* 0x1FF *128ms */
	OCT2_SET_PG_TIMEOUT_FACTOR_128(TRACE_DEBUG_ON);
	/* set CRC settings*/
	OCT2_SET_CRC_LEN_16(oct2_fcs_size);
	if (oct2_fcs_size == OCT_FCS_SIZE_16) /*check if FCS is 16 bit */
		poly = 0x1021;/*polynom for 16 bit FCS value */
	else
		poly = 0x04C11DB7;/*polynom for 32 bit FCS value */
	OCT2_SET_CRC_POLY(poly);
	OCT2_SET_CRC_INIT_ONES(TRACE_DEBUG_ON);
	/* exit from OCT config mode */
	OCT2_SET_RUN_MODE;
	OCT2_CLEAR_IRQS(OCT2_ALL_IRQS);
	OCT2_ENABLE_ALL_IRQS;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void oct2_acknowledge(int num_bytes)
{
	/*acknowledge section */
	OCT2_LOG(OCT2_INFO_ACKN, pg_read_idx, oct2_read_ptr, 0);
	if (!open_num_pages) {
		/* rptr and wptr are on the same page */
		oct2_read_ptr += num_bytes;
	} else {
#if TRACE_DEBUG_OCT_FULL_MANAGEMENT
		unsigned short idx;
		idx = oct2_page_table[pg_read_idx].buff_desc.prev->index;
		/* page has been completely sent */
		/*increment the read index & update End indicator*/
		oct2_page_table[idx].buff_desc.full = 0;
		OCT2_CLR_CNF_PTBY;
		/* E bit placed in LSB page table*/
		OCT2_SET_PGT_MEM0(idx,
			oct2_page_table[idx].content.buff_desc_lo);
		OCT2_SET_PGT_MEM1(idx,
			oct2_page_table[idx].content.buff_desc_hi);
		idx = (oct2_page_table[idx].buff_desc.next)->index;
		oct2_page_table[idx].buff_desc.full = 1;
		/* E bit placed in LSB page table*/
		OCT2_SET_PGT_MEM0(idx,
			oct2_page_table[idx].content.buff_desc_lo);
		OCT2_SET_PGT_MEM1(idx,
			oct2_page_table[idx].content.buff_desc_hi);
		OCT2_SET_CNF_PTBY;
#endif
		pg_read_idx =
		  (oct2_page_table[pg_read_idx].buff_desc.next)->index;
		oct2_read_ptr = 0;
		oct2_write_ptr = 0;
		open_num_pages--;
		oct2_reset_timeout_cnt(current_timeout);
	}
	if (open_num_pages) {
		/*more pages to send */
		oct2_events |= TRACE_DEBUG_OCT_DATA_RX_EVENT
			| TRACE_DEBUG_OCT_DATA_ACK_EVENT;
	} else {
		/*send complete - buffer emty at this point */
		TraceDebugOctBuffEmpty = 1;
	}
	oct2_reset_timeout_cnt(current_timeout);
	data_available = 0;
	OCT_DBG("	[ack] %2d/%2d (curr/rd idx); Wptr %5x; Rptr %5x",
		pg_curr_idx, pg_read_idx, oct2_write_ptr, oct2_read_ptr);
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void oct2_write_data_to_usb(void *ptr, int num_bytes)
{
	mm_segment_t old_fs;
	int written_bytes;
	int verify_fp = 2;

	OCT_DBG("Sending to USB @0x%x:0x%x", (unsigned int)ptr, num_bytes);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	while (verify_fp > 0) {
		if (IS_ERR_OR_NULL(fp)) {
			OCT_LOG("Gadget fp=%p is not valid=%ld!",
					fp, PTR_ERR(fp));
			/* re-open the tty gadget */
			fp = filp_open(USB_CH_NAME, O_RDWR, 0);
		}
		if (IS_ERR_OR_NULL(fp)) {
			OCT_LOG("Gadget fp=%p re-open failed with error=%ld!",
						fp, PTR_ERR(fp));
			/* skip bytes in buffer and leave loop */
			written_bytes = num_bytes;
			verify_fp = -1;
		} else {
			/* try a first time write */
			written_bytes =
				fp->f_op->write(fp, (char *)ptr, num_bytes, 0);
			if (written_bytes <= 0) {
				OCT_LOG(
					"Error: USB write Error with valid fp=%p, %d",
					fp, written_bytes);
				/* come back and force the loop to re-open fp */
				fp = NULL;
				/* avoid infinity loop doing only
				 * one time the loop */
				verify_fp--;
				/* skip bytes in buffer and leave loop */
				written_bytes = num_bytes;
			} else {
				/* write sucessfuly to USB and leave the loop */
				verify_fp = -1;
			}
		}
	}
	set_fs(old_fs);
	OCT_DBG("Written bytes 0x%x, oct2_read_ptr 0x%x",
		written_bytes, oct2_read_ptr);

}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void oct2_send_page(unsigned int index, unsigned int offset, unsigned int size)
{
	dma_addr_t phy_addr;
	/* get 4 byte alligned address of the page to be sent */
	phy_addr = (oct2_page_table[index].buff_desc.base<<2) & 0xFFFFFFFC;
	OCT2_LOG(OCT2_INFO_SEND, index, offset, size);
	dma_sync_single_for_cpu(NULL,
			phy_addr + offset, size, DMA_FROM_DEVICE);
	/*to do: lock the page */
	if (debug_over_usb == 1) {
		OCT_DBG("  -[snd]write-> Sart Addr(VT/PH): %8x/%8x, size = %d",
		(unsigned int)
		((oct2_page_table[index].buff_desc.base_virt) + offset),
		phy_addr + offset, size);
		/*msleep(size/50);*/
	}
	oct2_write_data_to_usb(
		oct2_page_table[index].buff_desc.base_virt + offset,
		size);
	oct2_acknowledge(size);

}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static int oct2_thread(void *param)
{
	OCT_DBG("Enter thread");
	fp = oct2_open_gadget(USB_CH_NAME);
	fp_debug = oct2_open_gadget(USB_DEBUG_CH);
#if OCT_DEBUG_OVER_USB_CH
	if (!IS_ERR_OR_NULL(fp_debug))
		debug_over_usb = 1;
	else
		OCT_LOG("debug_over_usb disabled due to open failed\n");
#endif
	while (!kthread_should_stop()) {
		wait_event_interruptible(oct2_wq, oct2_events);
		/*oct2_print_page_table();*/

		OCT_DBG("---[evt] %d; IRQ: %x; FULL: %d EMPTY: %d",
			oct2_events, oct2_irq, TraceDebugOctBgFULL,
			TraceDebugOctBuffEmpty);
		OCT2_LOG(OCT2_INFO_EVNT, 0,
			0, oct2_events);

		if (oct2_events & TRACE_DEBUG_OCT_DATA_FULL_EVENT)
			oct2_events &= ~TRACE_DEBUG_OCT_DATA_FULL_EVENT;
		if (oct2_events & TRACE_DEBUG_OCT_DATA_ACK_EVENT)
			oct2_events &= ~TRACE_DEBUG_OCT_DATA_ACK_EVENT;
		if (oct2_events & TRACE_DEBUG_OCT_RELOAD_PG_TAB_EVENT) {
			oct2_load_page_table();
			oct2_events &= ~TRACE_DEBUG_OCT_RELOAD_PG_TAB_EVENT;
		}
		/* buffer wraparound detection */
		if (oct2_events & TRACE_DEBUG_OCT_WRAP_EVENT) {
			TraceDebugOctBuffWrap++;
			OCT2_LOG(OCT2_INFO_WRAP, pg_curr_idx,
				TraceDebugOctBuffWrap, 0);
			OCT_DBG("Buffer Wrap around %d", TraceDebugOctBuffWrap);
			oct2_events &= ~TRACE_DEBUG_OCT_WRAP_EVENT;
		}
		if (!(oct2_events & TRACE_DEBUG_OCT_DATA_RX_EVENT)
			|| TraceDebugOctBuffEmpty) {
			/* in case no data available */
			OCT2_LOG(OCT2_INFO_EMPT, pg_curr_idx, 0, 0);
			/* reload timeout counter */
			oct2_reset_timeout_cnt(current_timeout);
			oct2_events &= ~TRACE_DEBUG_OCT_DATA_RX_EVENT;
			OCT2_ENABLE_ALL_IRQS;
			continue;
		}
		oct2_events &= ~TRACE_DEBUG_OCT_DATA_RX_EVENT;
		/* if we are here, start the transfer */
		/* get page index & write Ptr */
		pg_curr_idx = OCT2_GET_PAGE_INDEX;
		oct2_write_ptr = OCT2_GET_PAGE_WPTR;
		if (TraceDebugOctBgFULL)
			open_num_pages = OCT_NUM_OF_PAGES;
		else if (pg_curr_idx >= pg_read_idx)
			open_num_pages = pg_curr_idx - pg_read_idx;
		else
			open_num_pages = OCT_NUM_OF_PAGES
					- (pg_read_idx - pg_curr_idx);
		OCT2_LOG(OCT2_INFO_SEND, pg_curr_idx,
				oct2_write_ptr, open_num_pages);
		OCT_DBG(" --[tra] %2d/%2d (curr/rd idx); Wptr %5x; Rptr %5x",
					pg_curr_idx, pg_read_idx,
					oct2_write_ptr, oct2_read_ptr);
		/* forward data */
		if (open_num_pages == 1) {
			/*if there is only one page to be sent */
			oct2_send_page(pg_read_idx, 0, open_num_bytes);
		} else if (open_num_pages > 1) {
			/*if more pages to be sent then send them completely */
			oct2_send_page(pg_read_idx, 0, OCT2_DMA_PAGE_SIZE);
		}
		/*detect Buffer FULL */
		if (TraceDebugOctBgFULL) {
			OCT2_LOG(OCT2_INFO_FULL, pg_curr_idx,
					oct2_write_ptr, oct2_read_ptr);
			TraceDebugOctExtMemFull++;
		}
		/*schedule_timeout(100);*/
		/*msleep(10); - wait some time till next thread run*/
		/*set_current_state(TASK_INTERRUPTIBLE);*/

		OCT2_ENABLE_ALL_IRQS;
	}
	OCT_DBG("WHILE exited");
	return 0;
}



/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void oct2_tasklet_bh(unsigned long data)
{
	unsigned int prev_idx;
	unsigned int wrPtr;
	unsigned int pg_idx;

	if (!debug_over_usb)
		OCT_DBG("IRQ: %d", oct2_irq);
	/* get page index */
	pg_idx = OCT2_GET_PAGE_INDEX;
	wrPtr = OCT2_GET_PAGE_WPTR;
	OCT2_LOG(OCT2_INFO_LISR, pg_idx, oct2_irq, wrPtr);

	if ((oct2_irq & OCT2_IRQ_TMO) && (wrPtr != 0)) {
		/* if there is only timeout irq - forward data */
		/* write pointer is in the current page idx */
		/* wPtr should be always != 0 */
		open_num_bytes = wrPtr;
		OCT2_SET_PG_TIMEOUT(0x1FF);
		oct2_events = TRACE_DEBUG_OCT_DATA_RX_EVENT;
	} else {
		/* write pointer should be alread in the next page */
		/* take the idx from previous page & check the descriptor */
		/* if HW behaves different ->following line to be removed */
		prev_idx = oct2_page_table[pg_idx].buff_desc.prev->index;
		/*check the descriptor from previous page */
		if (oct2_irq  & OCT2_IRQ_FW) {
			open_num_bytes = OCT2_DMA_PAGE_SIZE;
			oct2_events = TRACE_DEBUG_OCT_DATA_RX_EVENT;
		}
		if (oct2_irq & OCT2_IRQ_REFILL)
			oct2_events |= TRACE_DEBUG_OCT_RELOAD_PG_TAB_EVENT;
		if (oct2_irq & OCT2_IRQ_END) {
			oct2_events |= TRACE_DEBUG_OCT_DATA_FULL_EVENT;
			/* TraceDebugOctBgFULL = 1; */
			OCT2_LOG(OCT2_INFO_FULL, pg_idx, oct2_irq, wrPtr);
		}
		/*detect buffer wraparound */
		if (oct2_irq & OCT2_IRQ_RES)
			oct2_events |= TRACE_DEBUG_OCT_WRAP_EVENT;
	}
	/* detect if there is data in buffer */
	if ((pg_read_idx != pg_idx)
		|| (oct2_read_ptr != wrPtr)
		|| (TraceDebugOctBgFULL)) {
		TraceDebugOctBuffEmpty = 0;
	}
	if (oct2_events) {
		count_oct2_tasklet_usb++;
		wake_up(&oct2_wq);
	}
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

void oct2_page_timer_callback(unsigned long data)
{
	unsigned int pg_idx;
	unsigned int wrPtr;

	/* get page index */
	pg_idx = OCT2_GET_PAGE_INDEX;
	wrPtr = OCT2_GET_PAGE_WPTR;

	/* detect if there is data in buffer */
	if (pg_read_idx == pg_idx) {
		/*if there are no pending pages to be send, trigger timeout */
		/* trigger HW timeout */
		OCT2_SET_PG_TIMEOUT(1);
		OCT2_SET_PG_TIMEOUT_FACTOR_128(TRACE_DEBUG_OFF);
	} else
		oct2_reset_timeout_cnt(OCT2_PG_TIMEOUT_RUN);
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static ssize_t oct2_read(struct file *file_ptr,
			    char __user *user_buffer,
			    size_t count,
			    loff_t *position)
{
#if 0
	unsigned int nb_of_tx_bytes = 0;
	unsigned int nb_of_avail_bytes = 0;
	unsigned int result = 0;
	int retval = 0;

	if (oct2_out_path != OCT_PATH_APP_POLL)
		return -1;

	/*check the amount of data available in the oct ext ring buffer*/
	nb_of_tx_bytes = oct2_get_available_bytes_number();
	nb_of_avail_bytes = nb_of_tx_bytes;

	/*if requested size exists ->send data imediatelly*/
	if (nb_of_tx_bytes < count) {
		oct2_reset_timeout_cnt(current_timeout);
		oct2_enable_all_irqs();
		retval = wait_event_interruptible_timeout(oct2_read_wq,
							oct2_events, 10*HZ);
		if (retval < 0) {
			OCT_DBG("waitM: -ERESTARTSYS");
		} else if (oct2_events == 0 && retval == 0) {
			OCT_DBG("waitM: Inform APP of time out");
			count_read_time_outs++;
		} else if (oct2_events == 1 && retval > 0) {
			OCT_DBG("waitM: Interrupt Event received");
			count_read_event_wakeup++;
		} else {
			OCT_DBG("waitM: Known Event received");
		}
		oct2_events = 0;
		nb_of_tx_bytes = oct2_get_available_bytes_number();
		OCT_DBG("DATA before/after event: %d --> %d",
				nb_of_avail_bytes, nb_of_tx_bytes);
		nb_of_avail_bytes = nb_of_tx_bytes;
	}
	if (nb_of_tx_bytes) {
		/*if requested less bytes than available*/
		if (nb_of_tx_bytes >= count) {
			/*send only the requested size*/
			nb_of_tx_bytes = count;
		}
		/* Sync cache */
		dma_sync_single_for_cpu(NULL, phy_base_addr +
			oct2_read_ptr, nb_of_tx_bytes, DMA_FROM_DEVICE);
		/* Transfer data */
		result = copy_to_user((void __user *)user_buffer,
				&((char *)oct2_ext_rbuff_ptr)[oct2_read_ptr],
				nb_of_tx_bytes);
		if (result) {
			OCT_DBG("Error copy_to_user");
			count_error_transfers++;
		} else {
			count_transfers++;
		}
		oct2_read_ptr += nb_of_tx_bytes;
		/* check if result is consistent*/
		if (oct2_read_ptr > oct2_ext_ring_buff_len) {
			/* too many bytes*/
			OCT_DBG("Error: ReadPtr exceeds the buffer length");
			OCT_DBG("num bytes 0x%x, oct2_read_ptr 0x%x",
					nb_of_tx_bytes, oct2_read_ptr);
			oct2_read_ptr = 0;
			count_too_many_bytes++;
		} else if (oct2_read_ptr == oct2_ext_ring_buff_len) {
			oct2_read_ptr = 0;	 /* wrap around*/
			OCT_DBG("ReadPtr wrap around");
			OCT_DBG("num bytes 0x%x, oct2_read_ptr 0x%x",
					nb_of_tx_bytes, oct2_read_ptr);
		}
		setOct_trform_OCT_SW_RPTR(oct2_reg, oct2_read_ptr);
	}
	OCT_DBG("%d out of %d bytes sent", nb_of_tx_bytes, nb_of_avail_bytes);
	return nb_of_tx_bytes;
#else
	return 0;
#endif
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static unsigned int oct2_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;
	OCT_DBG("Poll-%d", data_available);
	if (data_available) {
		ret |= POLLIN | POLLRDNORM;
		return ret;
	}
	poll_wait(file, &oct2_poll_wq, wait);
	if (data_available)
		ret |= POLLIN | POLLRDNORM;
	return ret;
}

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static ssize_t oct2_write(struct file *p_file,
			     const char __user *user_buffer,
			     size_t count,
			     loff_t *position)
{
	unsigned char *buffer;
	mm_segment_t old_fs;
	int result;

	buffer = kmalloc(count+1, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;
	result = copy_from_user(buffer, user_buffer, count);
	if (result)
		OCT_DBG("Error copy_from_user");
	buffer[count] = '\0';
	OCT_DBG("Write ->Count: %d Buffer: %s", count, buffer);

	if (!strncmp(buffer, "stop", 4)) {
		free_irq(OCT_INT, (void *)(oct2_irq_handler));
		OCT_DBG("Oct Stopped");
	} else if (!strncmp(buffer, "start1", 6)) {
		result = request_irq(OCT_INT, (irq_handler_t)oct2_irq_handler,
					IRQF_SHARED, "OCT_Drv",
					(void *)(oct2_irq_handler));
		if (result)
			OCT_LOG("Oct can't start");
		else
			OCT_LOG("Oct Started");
	} else if (!strncmp(buffer, "start2", 6)) {
		result = request_irq(OCT_INT, (irq_handler_t)oct2_irq_handler,
					IRQF_DISABLED, "OCT_Drv",
					(void *)(oct2_irq_handler));
		if (result)
			OCT_LOG("Oct can't start");
		else
			OCT_LOG("Oct Started");
	} else if (!strncmp(buffer, "init", 4)) {
			oct2_hw_init();
			OCT_LOG("HW initialized");
	} else if (!strncmp(buffer, "info", 4)) {
		void *tadihandle = NULL;
		int page_index, page_wptr, ringbuf_ptr;
		page_index = OCT2_GET_PAGE_INDEX;
		page_wptr = OCT2_GET_PAGE_WPTR;
		ringbuf_ptr = OCT2_GET_RINGBUFF_WRITE_PTR(page_index);
		OCT_LOG("OCT driver version is: %d", OCT_DRIVER_VER);
		OCT_LOG("OCT Power (1-ON/0-OFF) %d", oct2_check_power());
		OCT_LOG("OCT_CNF:  0x%8x", OCT2_GET_CNF_VAL);
		OCT_LOG("oct2_reg @ 0x%8x", (unsigned int)oct2_reg);
		OCT_LOG("oct2_buff @ 0x%8x (size: %dbytes)",
				(unsigned int)oct2_ext_rbuff_ptr,
				oct2_ext_ring_buff_len);
		OCT_LOG("Page idx %d", page_index);
		OCT_LOG("Page wptr %x", page_wptr);
		OCT_LOG("Base Address %x", OCT2_GET_RINGBUF_BASE_ADR);
		OCT_LOG("Ringbuf wr Ptr %x", ringbuf_ptr);
		OCT_LOG("IRQ Stat:0x%x", OCT2_GET_IRQ_STAT);
		OCT_LOG("SW-Rdp/Wrp %d/%d", oct2_read_ptr, oct2_write_ptr);
		OCT_LOG("Data_avail: %d", oct2_get_available_bytes_number());
		OCT_LOG("Pg. timeout: %x", OCT2_GET_PG_TIMEOUT);
		OCT_LOG("oct2_events: %d", oct2_events);
		OCT_LOG("oct2_ext_mem_full: %d", oct2_ext_mem_full);
		OCT_LOG("oct2_out_path: %d", oct2_out_path);
		OCT_LOG("count_too_many_bytes: %d",
				count_too_many_bytes);
		OCT_LOG("count_error_transfers: %d",
				count_error_transfers);
		OCT_LOG("count_read_event_wakeup: %d",
				count_read_event_wakeup);
		OCT_LOG("count_read_time_outs: %d",
				count_read_time_outs);
		OCT_LOG("count_oct2_tasklet_usb: %d",
				count_oct2_tasklet_usb);
		OCT_LOG("count_transfers: %d",
			count_transfers);

		tadihandle = trc_tadi_open(MT_PRINTF);
		if (NULL != tadihandle) {
			trc_tadi_write(tadihandle,
				"Including Tadi Kernel Interface", 31);
			trc_tadi_write(tadihandle, "Ending", 6);
			trc_tadi_close(tadihandle);
			OCT_LOG("Tadi Kernel Interface Open/Close");
		} else {
			OCT_LOG("Tadi Kernel Interface Unable to Open");
		}
	} else if (!strncmp(buffer, "off", 5)) {
		oct2_mode_off();
	} else if (!strncmp(buffer, "debug", 5)) {
		debug_on ^= 1;
	} else if (!strncmp(buffer, "usbdebug", 8)) {
		debug_over_usb ^= 1;
	} else if (!strncmp(buffer, "size", 4)) {
		int size;
		size = oct2_get_available_bytes_number();
		OCT_DBG("Nb of bytes: %d", size);
	} else if (!strncmp(buffer, "timeout1", 8)) {
		current_timeout = OCT2_PG_TIMEOUT_RUN;
		oct2_reset_timeout_cnt(current_timeout);
	} else if (!strncmp(buffer, "timeout2", 8)) {
		current_timeout = OCT2_PG_TIMEOUT_SLEEP;
		oct2_reset_timeout_cnt(current_timeout);
	} else if (!strncmp(buffer, "irq_en", 6)) {
		oct2_disable_irq_during_sleep = 0;
		oct2_enable_all_irqs();
	} else if (!strncmp(buffer, "irq_dis", 7)) {
		static int oct_all_irqs;
		oct_all_irqs = OCT2_ALL_IRQS;
		oct2_disable_irq_during_sleep = 1;
		oct2_disable_all_irqs();
		OCT2_CLEAR_IRQS(oct_all_irqs);
	} else if (!strncmp(buffer, "sleep", 5)) {
		oct2_disable_irq_during_sleep ^= 1;
		OCT_LOG(" IRQs disabled in sleep?: %d",
			oct2_disable_irq_during_sleep);
	} else if (!strncmp(buffer, "usb", 3)) {
		if (oct2_out_path == OCT_PATH_TTY)
			oct2_out_path = OCT_PATH_NONE;
		else
			oct2_out_path = OCT_PATH_TTY;
	} else if (!strncmp(buffer, "sdcard", 6)) {
		if (oct2_out_path == OCT_PATH_APP_POLL)
			oct2_out_path = OCT_PATH_TTY;
		else
			oct2_out_path = OCT_PATH_APP_POLL;
	} else if (!strncmp(buffer, "wakeup", 6)) {
		oct2_events = 1;
		if (oct2_out_path == OCT_PATH_APP_POLL)
			wake_up(&oct2_read_wq);
		else
			wake_up(&oct2_wq);
	} else if (!strncmp(buffer, "open", 4)) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		fp = filp_open("/dev/ttyGS1", O_RDWR, 0);
		OCT_LOG("Opening Gadget returned %ld", PTR_ERR(fp));
		set_fs(old_fs);
	} else {
		int nb_of_bytes;
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		nb_of_bytes =  fp->f_op->write(fp, buffer, count, 0);
		OCT_LOG("Bytes avail/written: %d/%d", count, nb_of_bytes);
		set_fs(old_fs);
	}
	if (count <= 0)
		OCT_LOG("OCT to USB write Error");
	kfree(buffer);
	return count;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static int oct2_open(struct inode *p_inode, struct file *p_file)
{
	return 0;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static int oct2_release(struct inode *p_inode, struct file *p_file)
{
	return 0;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static long oct2_ioctl(struct file *p_file,
			 unsigned int cmnd,
			 unsigned long param)
{
#if 0
	uOCT_MASTER_WPTR wptr_un;
	struct s_oct_info oct2_info;
	int result;

	OCT_DBG("Ioctl cmd %d, param: %ld", cmnd, param);

	switch (cmnd) {
	case OCT_IOCTL_SET_PATH:
		oct2_out_path = param;
		break;
	case OCT_IOCTL_SET_MODE:
		oct2_mode = param;
		if (param == OCT_MODE_OFF)
			oct2_mode_off();
		else
			oct2_hw_init();
		if (param == OCT_MODE_OW)
			oct2_out_path = OCT_PATH_NONE;
		break;
	case OCT_IOCTL_CONF_TRIG_CYCLE:
		/* todo: timeout not supported, a sw one must be created */
		break;
	case OCT_IOCTL_GET_INFO:
		wptr_un.OCT_MASTER_WPTR_Content =
		getOct_trform_OCT_MASTER_WPTR(oct2_reg);
		oct2_info.wr_ptr = (unsigned int)
		wptr_un.OCT_MASTER_WPTR_Structure.MASTER_WPTR << 2;
		oct2_info.rd_ptr =
		(unsigned int)getOct_trform_OCT_SW_RPTR(oct2_reg);
		oct2_info.is_full =
		wptr_un.OCT_MASTER_WPTR_Structure.EXT_MEM_FULL;
		oct2_info.irq_stat =
			getOct_trform_OCT_MASTER_RXIRQ_STAT(oct2_reg) & 0x07;
		result = copy_to_user((struct s_oct2_info *)param,
				&oct2_info,
				sizeof(oct2_info));
		if (result)
			OCT_LOG("Error in Ioctl Get info");
		break;
	case OCT_IOCTL_ENTER_CD:
		/* todo: - for coredump*/
		break;
	case OCT_IOCTL_FLUSH:
		/*todo: - for coredump*/
		break;
	default:
		OCT_DBG("Ioctl command out of range");
	}
#endif
	return 0;
}

static const struct file_operations oct2_fops = {
	.owner = THIS_MODULE,
	.aio_write = NULL,
	.read = oct2_read,
	.poll = oct2_poll,
	.write = oct2_write,
	.open = oct2_open,
	.release = oct2_release,
	.unlocked_ioctl = oct2_ioctl
};

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static int oct2_driver_probe(struct platform_device *pdev)
{
	int ret = 0;
	int result = 0;
	int data = 0;
	struct device *dev_oct;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *mem;

	dev_dbg(dev, "%s", __func__);

	result = register_chrdev(0, OCT_MODULE_NAME, &oct2_fops);
	if (result < 0) {
		dev_err(dev, "can\'t register dev errorcode=%i", result);
		return -1;
	}

	major = result;
	class_oct = class_create(THIS_MODULE, OCT_MODULE_NAME);

	if (IS_ERR(class_oct)) {
		dev_err(dev, "error creating class");
		goto unregister_chrdev;
	}

	dev_oct = device_create(class_oct, NULL,
			MKDEV(major, 0),
			NULL, OCT_MODULE_NAME);

	if (IS_ERR(dev_oct)) {
		dev_err(dev, "Error creating device");
		goto unregister_class;
	}

	dev_dbg(dev, "registered char device, major number=%i", major);

	mem = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "oct-registers");
	if (!mem) {
		dev_err(dev, "no oct-registers resource?\n");
		goto unregister_class;
	}
	oct2_reg = devm_ioremap_resource(dev, mem);
	dev_info(dev, "oct resource: %pR - ioremap: %p\n", mem, oct2_reg);
	if (!oct2_reg) {
		pr_err("%s: unable to remap memory region\n", __func__);
		goto unregister_class;
	}
	/* pm */
	oct2_pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(oct2_pm_platdata)) {
		pr_err("%s: Error during device state pm init\n", __func__);
		goto unregister_class;
	}
	if (oct2_pm_platdata) {
		ret = device_state_pm_set_class(dev,
					oct2_pm_platdata->pm_user_name);
		if (ret) {
			pr_err("%s: Error while setting the pm class for user %s\n",
				__func__, oct2_pm_platdata->pm_user_name);
			goto unregister_class;
		}
		ret = device_state_pm_set_state_by_name(dev,
				oct2_pm_platdata->pm_state_D0_name);
		if (ret) {
			pr_err("%s: Error while setting the pm state: %s\n",
				__func__, oct2_pm_platdata->pm_state_D0_name);
			goto unregister_class;
		} else
			pr_debug("%s: %s requested for user %s\n", __func__,
					oct2_pm_platdata->pm_state_D0_name,
					oct2_pm_platdata->pm_user_name);
	} else
		pr_debug("oct2_pm_platdata is NULL\n");

	oct2_mode = OCT_MODE_STALL;
	dev_dbg(dev, "get_oct2_mode returns: %x", oct2_mode);
	oct2_hw_init();
	setup_timer(&oct2_page_timer, oct2_page_timer_callback, 0);
	oct2_reset_timeout_cnt(current_timeout);
	if (oct2_mode == OCT_MODE_OW)
		oct2_out_path = OCT_PATH_NONE;
	oct2_ext_mem_full = 0;
	init_waitqueue_head(&oct2_wq);
	init_waitqueue_head(&oct2_poll_wq);
	init_waitqueue_head(&oct2_read_wq);
	task = kthread_run(oct2_thread, (void *)&data, "OCT Thread");
	dev_dbg(dev, "Kernel OCT Thread: %s", task->comm);


	oct2_int = platform_get_irq_byname(pdev, "OCT_INT");
	if (oct2_mode != OCT_MODE_OW && !IS_ERR_VALUE(oct2_int)) {
		ret = request_irq(OCT_INT,
				(irq_handler_t) oct2_irq_handler,
				IRQF_SHARED,
				"OCT_Drv",
				(void *)(oct2_irq_handler));
		if (ret)
			dev_err(dev, "can't get shared interrupt");
		else
			dev_dbg(dev, "Interrupt %d installed", oct2_int);
	}

	/* Runtime PM initialization */
	pm_runtime_enable(dev);
	dev_dbg(dev, "PM runtime enabled");
	pm_runtime_irq_safe(dev);
	pm_runtime_get_sync(dev);
	dev_dbg(dev, "PM runtime get");

	/* Do some other initialization.. if any.
	 * And put when no longer needed */
	pm_runtime_put_sync(dev);
	dev_dbg(dev, "PM runtime put");

	return 0;
unregister_class:
	class_destroy(class_oct);
unregister_chrdev:
	unregister_chrdev(major, OCT_MODULE_NAME);
	return -1;
}

static int oct2_driver_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;

	OCT_LOG("Removing...");

	del_timer(&oct2_page_timer);
	/* Deregister IRQ */
	free_irq(OCT_INT, (void *)(oct2_irq_handler));
	oct2_events = 1;
	wake_up(&oct2_wq);
	kthread_stop(task);
	iounmap(OCT_REG_ADDRESS_BASE);
	dma_unmap_single(NULL, phy_base_addr,
			OCT_EXT_RING_BUFF_SIZE_ADJ, DMA_FROM_DEVICE);
	kfree(oct2_ext_rbuff_ptr);

	device_destroy(class_oct, MKDEV(major, minor));
	class_destroy(class_oct);
	unregister_chrdev(major, OCT_MODULE_NAME);
	pm_runtime_disable(dev);
	OCT_DBG("device unregistration");
	if (oct2_pm_platdata) {
		err = device_state_pm_set_state_by_name(dev,
				oct2_pm_platdata->pm_state_D3_name);
		if (err) {
			pr_err("%s: Error while setting the pm state: %s\n",
				__func__, oct2_pm_platdata->pm_state_D3_name);
			return -1;
		}
	}

	return 0;
}

static const struct of_device_id xgold_oct2_of_match[] = {
	{
		.compatible = "intel,oct2",
	},
	{}
};

MODULE_DEVICE_TABLE(of, xgold_oct2_of_match);

static struct platform_driver oct2_driver = {
	.probe = oct2_driver_probe,
	.remove = oct2_driver_remove,
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.pm = &my_pm_ops,
		.of_match_table = of_match_ptr(xgold_oct2_of_match),
	}
};

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static int __init oct2_init(void)
{
	int err;
	OCT_DBG("Module loading...ver %d", OCT_DRIVER_VER);

	err = platform_driver_register(&oct2_driver);
	if (err) {
		OCT_LOG("Unable to register platform driver");
		return -1;
	}
	OCT_DBG("Platform driver registration ok...");
	return 0;
}


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - */

static void __exit oct2_exit(void)
{
	/* Deregister Driver */
	platform_driver_unregister(&oct2_driver);
	OCT_DBG("module removed");
}

module_init(oct2_init);
module_exit(oct2_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OCT driver");
