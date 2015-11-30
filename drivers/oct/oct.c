/*
* Copyright (C) 2013 Intel Mobile Communications GmbH
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
* You should have received a copy of the GNU General Public License along with
* this program; if not, write to the Free Software Foundation, Inc.,
* 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
*
*/
#include <linux/kernel.h>		/* printk() */
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/cdev.h>			/* character device */
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/workqueue.h>	/*for events*/
#include <linux/delay.h>		/* for msleep*/
#include <linux/kthread.h>
#include <linux/uaccess.h>
#include <linux/device.h>		/*for class & device */
#include <linux/slab.h>			/*for kmalloc, kfree */
#include <linux/interrupt.h>		/*for irq */
#include <linux/delay.h>		/*for events */
#include <linux/io.h>			/*for ioremap */
#include <asm/termios.h>		/* for file attribute */
#include <linux/dma-mapping.h>		/* for DMA alloc */
#include <linux/poll.h>
#include <linux/device_pm_data.h>
#include <linux/platform_device.h>	/*register platform for PM callbacks */
#include <linux/pm_runtime.h>		/*PM runtime*/
#include <linux/of.h>

#include "oct_io.h"
#include "oct.h"
#include <linux/tadi.h>

/* Constants */
#define OCT_MODULE_NAME "oct"
#define PLATFORM_DRIVER_NAME "oct_drv"

#define OCT_DRIVER_VER 5

#ifndef OCT_HOST_TEST
	#define USB_CH_NAME "/dev/ttyGS1"
	#define OCT_INT oct_int
#else
	#define USB_CH_NAME "/dev/ttyS0"
	#define OCT_INT 1 /* keyboard int */
#endif

#define OCT_EV_UNKNOWN 0  /* if the reason for event is unknown*/
#define OCT_EV_RINGBUFF_TIMEOUT_WITH_DATA    1
#define OCT_EV_RINGBUFF_FILL_LEVEL_1_REACHED 2
#define OCT_EV_RINGBUFF_FILL_LEVEL_2_REACHED 4

/* OCT hw constants */
#define TRACE_DEBUG_OCT_TIMEOUT  0x80
#define OCT_TRIG_CYCLE_SLEEP   (5 * 1000 * 1000 * 26) /* 5s */
#define OCT_TRIG_LEVEL_2 ((OCT_EXT_RING_BUFF_SIZE / 4) * 3)
#define OCT_REG_ADDRESS_BASE oct_base
#define OCT_REG_SIZE sizeof(struct _soct_trform)

/* useful macros */
#define OCT_DBG(fmt, arg...) {\
	if (debug_on) \
		pr_info(OCT_MODULE_NAME": " fmt"\n", ##arg); }

#define OCT_LOG(fmt, arg...) \
	pr_info(OCT_MODULE_NAME": " fmt"\n", ##arg)

#define OCT_CLR_OCT_ON	SET_OCT_OCT_CNF_CLR_OCTM(oct_trform, 1)
#define OCT_CLR_OCT_OFF	SET_OCT_OCT_CNF_CLR_OCTM(oct_trform, 0)

/* Global variables */
void * __iomem oct_base;
unsigned int oct_int;
static int major;
static int minor;
static struct class *class_oct;
static struct file *fp;
static struct task_struct *task;
static wait_queue_head_t oct_wq, oct_poll_wq;
static dma_addr_t phy_base_addr;

static int oct_ext_ring_buff_len = OCT_EXT_RING_BUFF_SIZE;
static unsigned int oct_mode = DEFAULT_OCT_MODE;
static unsigned int current_timeout = DEFAULT_OCT_TRIG_CYCLE;
static unsigned int oct_trig_level1 = DEFAULT_OCT_TRIG_LEVEL;
static unsigned int oct_out_path = DEFAULT_OCT_PATH;
static unsigned int debug_on;

#define oct_trform oct_base
static void *oct_ext_rbuff_ptr;
static unsigned int oct_ext_mem_full;
static unsigned int oct_irq;
static char oct_events;
static unsigned int oct_read_ptr;
static unsigned int data_available;
struct device_pm_platdata *pm_platdata;

static struct file *oct_open_gadget(char *gadget_name);

#define OCT_OFFLOG
#ifdef OCT_OFFLOG
#include "oct_offlog.c"
#endif

/* power management section */

void disable_otc_irq(void) {
	/* disable timer interrupt, will be re-enabled if ext mem is empty */
	SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
			OCT_CNF_ENABLE_TRIG_CYCLE_INT_DISABLED);
}
void enable_otc_irq(void) {
	/* disable timer interrupt, will be re-enabled if ext mem is empty */
	SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
			OCT_CNF_ENABLE_TRIG_CYCLE_INT_ENABLED);
}

static int pm_suspend_fct(struct device *dev)
{
	int irq;
	/* steps to move subsystem to system suspend safe state */
	OCT_DBG("suspend");

	irq = GET_OCT_OCT_MASTER_RXIRQ_STAT(oct_trform);
	/* reset the interrupts */
	SET_OCT_OCT_MASTER_RXCON_EN_CH(oct_trform, 0);
	disable_otc_irq();
	SET_OCT_OCT_MASTER_RXIRQ_CON(oct_trform, irq);
	SET_OCT_OCT_MASTER_RXIRQ_CON(oct_trform, 0);
	/* trace_debug_oct_dma_off(); */
	return 0;
}

static int pm_resume_fct(struct device *dev)
{
	/* steps to resume from suspend */
	OCT_DBG("resume");
	/* reset & enable CYCLE interrupt */
	SET_OCT_OCT_MASTER_RXCON_EN_CH(oct_trform, 1);
	enable_otc_irq();
	return 0;
}

static int pm_runtime_suspend_fct(struct device *dev)
{
	OCT_DBG("Runtime suspend");
	/* steps to move subsystem to runtime suspend safe state */
	return 0;
}

static int pm_runtime_resume_fct(struct device *dev)
{
	/* steps to resume from suspend */
	OCT_DBG("Runtime resume");
	return 0;
}

static int pm_runtime_idle_fct(struct device *dev)
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

int oct_get_buffer_info(
	void **start_address,
	void **start_address_phy,
	unsigned int *size,
	void **read_ptr,
	void **write_ptr)
{
	void *ring_buf_start = NULL;
	unsigned int ring_buf_size = 0;
	unsigned int ring_read_ptr = 0;
	unsigned int ring_write_ptr = 0;

	int oct_enabled = 0;

	/* Check if OCT is enabled */
	if (0 != (GET_OCT_CLC(oct_trform) && 0x0000FF00)) {
		oct_enabled = 1;
		ring_buf_start =
			(void *)(GET_OCT_OCT_MASTER_RXCH0_BASE(oct_trform));
		ring_buf_size =
			GET_OCT_OCT_MASTER_RXCH0_SIZE(oct_trform);
		ring_read_ptr =
			GET_OCT_OCT_SW_RPTR(oct_trform) & 0xFFFFFFFC;
		ring_write_ptr =
			GET_OCT_OCT_MASTER_WPTR(oct_trform) & 0xFFFFFFFC;
	}

	if (start_address)
		*start_address = oct_ext_rbuff_ptr;

	if (start_address_phy)
		*start_address_phy = (void *)phy_base_addr;

	if (size)
		*size = ring_buf_size;

	if (read_ptr)
		*read_ptr = oct_ext_rbuff_ptr + ring_read_ptr;

	if (write_ptr)
		*write_ptr = oct_ext_rbuff_ptr + ring_write_ptr;

	OCT_LOG(
		"start=0x%x, start_phy=0x%x, size=0x%x, read=0x%x write=0x%x\n",
		(unsigned int)*start_address,
		(unsigned int)*start_address_phy,
		(unsigned int)*size,
		(unsigned int)*read_ptr,
		(unsigned int)*write_ptr);

	return oct_enabled;
}




static const struct dev_pm_ops my_pm_ops = {
	/* For system suspend/resume */
	.suspend = pm_suspend_fct,
	.resume = pm_resume_fct,
	/* For runtime PM */
	.runtime_suspend = pm_runtime_suspend_fct,
	.runtime_resume = pm_runtime_resume_fct,
	.runtime_idle = pm_runtime_idle_fct,
};

/* Tasklets for BH */
static void oct_tasklet_bh(unsigned long data);
DECLARE_TASKLET(oct_tasklet, oct_tasklet_bh, 0);

static int  __init oct_init(void);
static void __exit oct_exit(void);

irq_handler_t oct_irq_handler(int irq, void *dev_id, struct pt_regs *regs)
{
	/* clear OCT external buffer interrupt for LEVEL1, LEVEL2 and CYCLE*/
	irq = GET_OCT_OCT_MASTER_RXIRQ_STAT(oct_trform);

	/* disable timer interrupt, will be re-enabled if ext mem is empty */
	SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
			OCT_CNF_ENABLE_TRIG_CYCLE_INT_DISABLED);
	SET_OCT_OCT_CNF_ENABLE_TRIG_LEVEL1_INT(oct_trform,
			OCT_CNF_ENABLE_TRIG_LEVEL1_INT_DISABLED);
	SET_OCT_OCT_CNF_ENABLE_TRIG_LEVEL2_INT(oct_trform,
			OCT_CNF_ENABLE_TRIG_LEVEL2_INT_DISABLED);
	/* SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
				  OCT_CNF_ENABLE_TRIG_CYCLE_INT_DISABLED); */

	oct_irq = OCT_EV_UNKNOWN;

	if (irq & 1<<OCT_MASTER_RXIRQ_STAT_LSB_IRQ_CYCLE)
		oct_irq |= OCT_EV_RINGBUFF_TIMEOUT_WITH_DATA;
	if (irq & 1<<OCT_MASTER_RXIRQ_STAT_LSB_IRQ_LEVEL1)
		oct_irq |= OCT_EV_RINGBUFF_FILL_LEVEL_1_REACHED;
	if (irq & 1<<OCT_MASTER_RXIRQ_STAT_LSB_IRQ_LEVEL2)
		oct_irq |= OCT_EV_RINGBUFF_FILL_LEVEL_2_REACHED;

	/* reset the interrupts */
	SET_OCT_OCT_MASTER_RXIRQ_CON(oct_trform, irq);
	SET_OCT_OCT_MASTER_RXIRQ_CON(oct_trform, 0);

	/* activate bottom half task */
	tasklet_schedule(&oct_tasklet);
	OCT_DBG("IRQ: %d", oct_irq);

	return (irq_handler_t)IRQ_HANDLED;
}

static void oct_clr_bit(tcflag_t *p_flags, tcflag_t mask)
{
	OCT_DBG("CLR Bit 0x%07o", mask);
	OCT_DBG("---- Before 0x%07o", *p_flags);
	*p_flags &= ~mask;
	OCT_DBG("---- After  0x%07o", *p_flags);
}

static void oct_set_bit(tcflag_t *p_flags, tcflag_t mask)
{
	OCT_DBG("SET Bit 0x%07o", mask);
	OCT_DBG("---- Before 0x%07o", *p_flags);
	*p_flags |= mask;
	OCT_DBG("---- After  0x%07o", *p_flags);
}

static void oct_set_ccc(struct ktermios *attr,
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

void oct_set_raw_gadget(struct file *fp)
{
	struct ktermios my_attr;

	/* Read the current set of terminal attribute flags & dump the result */
	fp->f_op->unlocked_ioctl(fp, TCGETS, (unsigned long) &my_attr);

	/* Modify the attributes to be what we want instead of the default */
	oct_clr_bit(&my_attr.c_iflag, IGNBRK);
	oct_clr_bit(&my_attr.c_iflag, BRKINT);
	oct_clr_bit(&my_attr.c_iflag, PARMRK);
	oct_clr_bit(&my_attr.c_iflag, ISTRIP);
	oct_clr_bit(&my_attr.c_iflag, INLCR);
	oct_clr_bit(&my_attr.c_iflag, IGNCR);
	oct_clr_bit(&my_attr.c_iflag, ICRNL);
	oct_clr_bit(&my_attr.c_iflag, IXON);

	oct_clr_bit(&my_attr.c_oflag, OPOST);

	oct_clr_bit(&my_attr.c_lflag, ECHO);
	oct_clr_bit(&my_attr.c_lflag, ECHONL);
	oct_clr_bit(&my_attr.c_lflag, ICANON);
	oct_clr_bit(&my_attr.c_lflag, ISIG);
	oct_clr_bit(&my_attr.c_lflag, IEXTEN);

	oct_clr_bit(&my_attr.c_cflag, CSIZE);
	oct_clr_bit(&my_attr.c_cflag, PARENB);
	oct_set_bit(&my_attr.c_cflag, CS8);

	oct_set_ccc(&my_attr, VTIME, 0);
	oct_set_ccc(&my_attr, VMIN,  1);

	fp->f_op->unlocked_ioctl(fp, TCSETS, (unsigned long)&my_attr);
}

static struct file *oct_open_gadget(char *gadget_name)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	OCT_DBG("Opening Gadget[%s]", gadget_name);

	while (!kthread_should_stop()) {
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if (oct_out_path == OCT_PATH_TTY)
			fp = filp_open(gadget_name, O_RDWR, 0);
		else if (oct_out_path == OCT_PATH_FILE)
			fp = filp_open(gadget_name, O_RDWR | O_CREAT |
					O_TRUNC, 0666);
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
			if (oct_out_path == OCT_PATH_TTY)
				oct_set_raw_gadget(fp);
			break;
		}
		set_current_state(TASK_INTERRUPTIBLE);
	}
	return fp;
}

static void oct_mode_off(void)
{
	/* flush remaining data from internal OCT memory*/
	SET_OCT_OCT_CNF_OCTM_TIMEOUT(oct_trform,
					     TRACE_DEBUG_OCT_TIMEOUT);
	OCT_CLR_OCT_ON;
	/* flush DMA machine */
	SET_OCT_OCT_CNF_FRAME_TIMEOUT(oct_trform, 1);
	udelay(1);	 /* wait for 1 ms that remaining data can be flushed */
	SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 1);
	/* stop writing to external memory */
	SET_OCT_OCT_CNF_ENABLE_MANU_STALL(oct_trform,
				  OCT_CNF_ENABLE_MANU_STALL_ENABLED);
	/* disable Arm11 interrupt for OCT HW block */
	free_irq(OCT_INT, (void *)(oct_irq_handler));

	/* set read pointer to write pointer to clear memory buffer */
	SET_OCT_OCT_SW_RPTR_SW_RPTR(oct_trform,
		GET_OCT_OCT_MASTER_WPTR_MASTER_WPTR(oct_trform));
	/* reset TIMEOUT setting, clear DMA channel and disable channel */
	SET_OCT_OCT_MASTER_RXCON_TIMEOUT(oct_trform, 0);
	SET_OCT_OCT_MASTER_RXCON_CLR_CH(oct_trform,
				    OCT_MASTER_RXCON_CLR_CH_CLEAR);
	SET_OCT_OCT_MASTER_RXCON_EN_CH(oct_trform, 0);
	SET_OCT_CLC(oct_trform, 0x03);
}

static void trace_debug_oct_define_ringbuff(void *ring_buff_start,
		unsigned int ring_buff_size)
{
	/* if '-1' the set our default values */
	if (ring_buff_start != (void *)-1)
		oct_ext_rbuff_ptr = ring_buff_start;
	if (ring_buff_size != -1)
		oct_ext_ring_buff_len = ring_buff_size;

	oct_ext_rbuff_ptr = kzalloc(OCT_EXT_RING_BUFF_SIZE, GFP_KERNEL);
	phy_base_addr = dma_map_single(NULL, oct_ext_rbuff_ptr,
			OCT_EXT_RING_BUFF_SIZE, DMA_FROM_DEVICE);

	OCT_DBG("DMA addr:%#x; KERN Addr:%p", (unsigned int)phy_base_addr,
			oct_ext_rbuff_ptr);

	/* if NULL ptr or zero size defined -> Error
	 * or bad alignment */
	if (!(oct_ext_rbuff_ptr && oct_ext_ring_buff_len) ||
		((uintptr_t)oct_ext_rbuff_ptr & 15) ||
		(oct_ext_ring_buff_len & 15) ||
		(oct_ext_ring_buff_len < 0x4000) ||
		(oct_ext_ring_buff_len > 0x04000000))
		OCT_DBG("Define Ringbuff: Fatal error!");

	/* set pointer and size now in hardware */
	SET_OCT_OCT_MASTER_RXCH0_BASE_BASE(oct_trform,
			(unsigned int)(phy_base_addr >> 4));
	SET_OCT_OCT_MASTER_RXCH0_SIZE_SIZE(oct_trform,
			oct_ext_ring_buff_len >> 2);

	/* reset old write ptr and read ptr */
	SET_OCT_OCT_SW_RPTR_RST(oct_trform, OCT_SW_RPTR_RST_RESET);
	SET_OCT_OCT_SW_RPTR_RST(oct_trform, OCT_SW_RPTR_RST_NORMAL);
	return;
}

static int oct_check_power(void)
{
	SET_OCT_CLC(oct_trform, 0x100);
	if (GET_OCT_CLC(oct_trform) == 0x100)
		return 1;
	else
		return 0;
}

static void oct_hw_init(void)
{
	SET_OCT_CLC(oct_trform, 0x100);
	SET_OCT_OCT_CNF2_TRFORM_STOP(oct_trform,
			OCT_CNF2_TRFORM_STOP_DISABLED);
	SET_OCT_OCT_MASTER_RXCON_EN_CH(oct_trform, 0);
	SET_OCT_OCT_CNF_OCTM_TIMEOUT(oct_trform,
			TRACE_DEBUG_OCT_TIMEOUT);
	SET_OCT_OCT_CNF_FRAME_TIMEOUT(oct_trform, 1);

	/* flush / clear */
	SET_OCT_OCT_CNF_ENABLE_MANU_STALL(oct_trform,
			OCT_CNF_ENABLE_MANU_STALL_ENABLED);
	SET_OCT_OCT_CNF_ENABLE_OCTM_RINGBF(oct_trform,
			OCT_CNF_ENABLE_OCTM_RINGBF_DISABLED);
	if (oct_mode == OCT_MODE_OW)
		SET_OCT_OCT_CNF_ENABLE_EXTM_RINGBF(oct_trform,
				OCT_CNF_ENABLE_EXTM_RINGBF_ENABLED);
	else if (oct_mode == OCT_MODE_STALL)
		SET_OCT_OCT_CNF_ENABLE_EXTM_RINGBF(oct_trform,
				OCT_CNF_ENABLE_EXTM_RINGBF_DISABLED);

	OCT_CLR_OCT_ON;
	SET_OCT_OCT_MASTER_RXCON_CLR_CH(oct_trform,
			OCT_MASTER_RXCON_CLR_CH_CLEAR);
	SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 1);
	SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 0);
	SET_OCT_OCT_MASTER_RXCON_CLR_CH(oct_trform,
			OCT_MASTER_RXCON_CLR_CH_DEFAULT);
	OCT_CLR_OCT_OFF;

	SET_OCT_OCT_CNF_ENABLE_MANU_STALL(oct_trform,
			OCT_CNF_ENABLE_MANU_STALL_DISABLED);

	trace_debug_oct_define_ringbuff((void *)-1, -1);

	/* set fill levels */
	SET_OCT_OCT_MASTER_TRIG_LEVEL1_DMA_TRIG_LEVEL(oct_trform,
			oct_trig_level1 >> 2);
	SET_OCT_OCT_MASTER_TRIG_LEVEL2_DMA_TRIG_LEVEL(oct_trform,
			OCT_TRIG_LEVEL_2 >> 2);

	/* reset & enable CYCLE interrupt */
	SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(oct_trform,
			current_timeout);
	SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
			OCT_CNF_ENABLE_TRIG_CYCLE_INT_ENABLED);
	SET_OCT_OCT_MASTER_RXCON_TIMEOUT(oct_trform, 1024);
	SET_OCT_OCT_MASTER_RXCON_EN_CH(oct_trform, 1);
	return;
}

static void oct_write_data_to_usb(void *ptr, int num_bytes)
{
	mm_segment_t old_fs;
	int written_bytes;
	int verify_fp = 2;

	OCT_DBG("Sending to USB @%p:0x%x", ptr, num_bytes);
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

	/* increment the read ptr anyhow to re-start timeout */
	oct_read_ptr += written_bytes;
	/* check if result is consistent */
	if (oct_read_ptr > oct_ext_ring_buff_len) {
		/* too many bytes */
		OCT_LOG("Error: ReadPtr exceeds the buffer length");
		OCT_LOG("written bytes 0x%x, oct_read_ptr 0x%x",
				written_bytes, oct_read_ptr);
		oct_read_ptr = 0;
	} else if (oct_read_ptr == oct_ext_ring_buff_len) {
		oct_read_ptr = 0; /*wrap around*/
		OCT_DBG("ReadPtr wrap around:");
		OCT_DBG(" written bytes 0x%x, oct_read_ptr 0x%x",
				written_bytes, oct_read_ptr);
	}
	SET_OCT_OCT_SW_RPTR(oct_trform, oct_read_ptr);
	data_available = 0;
	/* reset & enable CYCLE interrupt */
	SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(oct_trform,
							   current_timeout);
	SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
				    OCT_CNF_ENABLE_TRIG_CYCLE_INT_ENABLED);
}

static int oct_thread(void *param)
{
	unsigned int num_bytes = 0, wptr_un = 0;
	struct device_node *np = (struct device_node *)param;
	OCT_DBG("Enter thread");

	#ifdef OCT_OFFLOG
	/* do it config file is found */
	oct_offlog_config(np);
	#endif

	if (oct_out_path == OCT_PATH_TTY)
		fp = oct_open_gadget(USB_CH_NAME);

	while (!kthread_should_stop()) {
		wait_event_interruptible(oct_wq, oct_events);
		oct_events = 0;
		OCT_DBG("Event received");
		/* read the whole WPTR register including flags */
		wptr_un = GET_OCT_OCT_MASTER_WPTR(oct_trform);

		if (OCT_MASTER_WPTR_DAT(wptr_un)) { /* if data */
			/* check ext mem and get
			 * nb of bytes available to be sent */
			unsigned int read_ptr, write_ptr;
			OCT_DBG("DATA available to be sent out");
			data_available = 1;
			wake_up(&oct_poll_wq);
			read_ptr = oct_read_ptr;
			write_ptr = OCT_MASTER_WPTR_MASTER_WPTR(wptr_un) << 2;

			/* not sure if we have to flush -but I did it */
			SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 1);
			SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 0);

			if (write_ptr > read_ptr) {
				num_bytes = write_ptr - read_ptr;
			} else {
				num_bytes = oct_ext_ring_buff_len - read_ptr;
				if (write_ptr == read_ptr)
					oct_ext_mem_full++;
			}
			/* Sync cache */
			dma_sync_single_for_cpu(NULL, phy_base_addr +
				oct_read_ptr, num_bytes, DMA_FROM_DEVICE);
			if (oct_out_path == OCT_PATH_TTY) {
				/* call subscribed function to forward data */
				oct_write_data_to_usb((char *)
				(&((char *)oct_ext_rbuff_ptr)[oct_read_ptr]),
					num_bytes);
			}
			#ifdef OCT_OFFLOG
			else if (oct_out_path == OCT_PATH_FILE) {
				/* call subscribed function to forward data */
				oct_write_data_to_file((char *)
				(&((char *)oct_ext_rbuff_ptr)[oct_read_ptr]),
					num_bytes);
			}
			#endif
			OCT_DBG("Sent out %d bytes", num_bytes);
		} else /* if no data available*/
			OCT_DBG("NO data available to be sent out");

		/*schedule_timeout(100);*/
		/*msleep(10); - wait some time till next thread run*/
		set_current_state(TASK_INTERRUPTIBLE);
		SET_OCT_OCT_CNF_ENABLE_TRIG_LEVEL1_INT(oct_trform,
				OCT_CNF_ENABLE_TRIG_LEVEL1_INT_ENABLED);
		SET_OCT_OCT_CNF_ENABLE_TRIG_LEVEL2_INT(oct_trform,
				OCT_CNF_ENABLE_TRIG_LEVEL2_INT_ENABLED);
	}
	OCT_DBG("WHILE exited");
	return 0;
}

static void oct_tasklet_bh(unsigned long data)
{
	/*__set_current_state(TASK_RUNNING);*/
	oct_events = 1;
	wake_up(&oct_wq);
	/*wake_up_process(task);*/
}

static ssize_t oct_read(struct file *file_ptr, char __user *user_buffer,
		size_t count, loff_t *position)
{
	unsigned int read_ptr, write_ptr;
	unsigned int num_bytes = 0;
	unsigned  wptr_un = 0;
	unsigned int result = 0;

	OCT_DBG("read at offset = %i, bytes count = %u",
			(int)*position,
			(unsigned int)count);
	if (oct_out_path != OCT_PATH_APP_POLL)
		return 0;
	/* read the whole WPTR register including flags */
	wptr_un  =
		GET_OCT_OCT_MASTER_WPTR(oct_trform);

	if (OCT_MASTER_WPTR_DAT(wptr_un)) { /* if data */
		write_ptr = OCT_MASTER_WPTR_MASTER_WPTR(wptr_un) << 2;
		read_ptr  = oct_read_ptr;

		if (write_ptr > read_ptr) {
			num_bytes = write_ptr - read_ptr;
		} else {
		    num_bytes = oct_ext_ring_buff_len - read_ptr;
		    if (write_ptr == read_ptr)
				oct_ext_mem_full++;
		}
		if (num_bytes > count)/*if requested less bytes than available*/
		    num_bytes = count;/*send only the requested size*/
		else {
		    data_available = 0;/*all the bytes will be sent */
		    /* reset & enable CYCLE interrupt */
		    SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(
								oct_trform,
							    current_timeout);
		    SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
				    OCT_CNF_ENABLE_TRIG_CYCLE_INT_ENABLED);
		}
		/* Sync cache */
		dma_sync_single_for_cpu(NULL, phy_base_addr +
			oct_read_ptr, num_bytes, DMA_FROM_DEVICE);

		result = copy_to_user(user_buffer,
			    &((char *)oct_ext_rbuff_ptr)[oct_read_ptr],
			    num_bytes);
		if (result)
			OCT_LOG("Error copy_to_user");

		oct_read_ptr += num_bytes;
		/* check if result is consistent*/
		if (oct_read_ptr > oct_ext_ring_buff_len) {
			/* too many bytes */
			OCT_DBG("Error: ReadPtr exceeds the buffer length");
			oct_read_ptr = 0;
		} else if (oct_read_ptr == oct_ext_ring_buff_len)
			oct_read_ptr = 0; /* wrap around*/

		SET_OCT_OCT_SW_RPTR(oct_trform, oct_read_ptr);
	}
	OCT_DBG("%x bytes sent out", num_bytes);
	return num_bytes;
}

static unsigned int oct_poll(struct file *file, poll_table *wait)
{
	unsigned int ret = 0;
	OCT_DBG("Poll-%d", data_available);
	if (data_available) {
		ret |= POLLIN | POLLRDNORM;
		return ret;
	}
	poll_wait(file, &oct_poll_wq, wait);
	if (data_available)
		ret |= POLLIN | POLLRDNORM;
	return ret;
}

static unsigned int get_oct_mode(void)
{
	/* OCT_CNF_ENABLE_EXTM_RINGBF_ENABLED */
	if (GET_OCT_OCT_CNF_ENABLE_EXTM_RINGBF(oct_trform))
		return OCT_MODE_OW;
	/* OCT_CNF_ENABLE_EXTM_RINGBF_DISABLED */
	else
		return OCT_MODE_STALL;
}

static ssize_t oct_write(struct file *p_file, const char __user *user_buffer,
		size_t count, loff_t *position)
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
	OCT_DBG("Buffer: %s Count: %d", (char *)buffer, (uint32_t)count);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	{
	if (!strncmp(buffer, "stop", 4)) {
		free_irq(OCT_INT, (void *)(oct_irq_handler));
		OCT_DBG("Oct Stopped");
	} else if (!strncmp(buffer, "start", 5)) {
		result = request_irq(OCT_INT, (irq_handler_t)oct_irq_handler,
			     IRQF_SHARED, "OCT_Drv", (void *)(oct_irq_handler));
		if (result)
			OCT_LOG("Oct can't start");
		else
			OCT_LOG("Oct Started");
	} else if (!strncmp(buffer, "init", 4)) {
		/*oct_open_port(USB_CH_NAME);*/
		oct_mode = get_oct_mode();
		OCT_DBG("get_oct_mode returns:  %x", oct_mode);
		oct_hw_init();
		if (oct_mode == OCT_MODE_OW)
			oct_out_path = OCT_PATH_NONE;
	} else if (!strncmp(buffer, "info", 4)) {
		void *tadihandle;

		OCT_LOG("OCT driver version: %d", OCT_DRIVER_VER);
		OCT_LOG("OCT Power (1-ON/0-OFF) %d", oct_check_power());
		OCT_LOG("oct_trform=%p",
				oct_trform);
		OCT_LOG("Ptr: %p Len: %d",
				oct_ext_rbuff_ptr, oct_ext_ring_buff_len);
		OCT_LOG("Wptr: %d",
				GET_OCT_OCT_MASTER_WPTR(oct_trform));
		OCT_LOG("Rptr: %d",
				GET_OCT_OCT_SW_RPTR(oct_trform));
		OCT_LOG("Base %#x",
				GET_OCT_OCT_MASTER_RXCH0_BASE(oct_trform));
		OCT_LOG("Size:%#x",
				GET_OCT_OCT_MASTER_RXCH0_SIZE(oct_trform));
		OCT_LOG("IRQ Stat:%#x",
				GET_OCT_OCT_MASTER_RXIRQ_STAT(oct_trform));
		OCT_LOG("SW-Rptr: %#x", oct_read_ptr);
		OCT_LOG("Data_avail: %d", data_available);
		tadihandle = trc_tadi_open(MT_PRINTF);
		trc_tadi_write(tadihandle,
				"Including Tadi Kernel Interface", 31);
		trc_tadi_write(tadihandle, "Ending", 6);
		trc_tadi_close(tadihandle);

	} else if (!strncmp(buffer, "off", 5)) {
		oct_mode_off();
	} else if (!strncmp(buffer, "debug", 5)) {
		debug_on ^= 1;
	} else if (!strncmp(buffer, "flush", 5)) {
		unsigned int num_bytes = 0;
		unsigned  wptr_un = 0;
		OCT_LOG("Flush command received");
		/* read the whole WPTR register including flags*/
		wptr_un  =
			GET_OCT_OCT_MASTER_WPTR(oct_trform);

		if (OCT_MASTER_WPTR_DAT(wptr_un)) {/* if data */
		    /*check ext mem and get nb of bytes available to be sent*/
		    unsigned int read_ptr, write_ptr;
		    OCT_LOG("flush: DATA available to be sent out");

		    read_ptr  = oct_read_ptr;
		    write_ptr =
			OCT_MASTER_WPTR_MASTER_WPTR(wptr_un) << 2;

		    /*not sure if we have to flush -but I did it */
		    SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 1);
		    SET_OCT_OCT_MASTER_RXCON_FLUSH(oct_trform, 0);

		    if (write_ptr > read_ptr) {
			num_bytes = write_ptr - read_ptr;
		    } else {
			num_bytes = oct_ext_ring_buff_len - read_ptr;
			if (write_ptr == read_ptr)
				oct_ext_mem_full++;
		    }

			OCT_LOG("flush: %x write_ptr_B", write_ptr);
			OCT_LOG("flush: %x read_ptr_B", read_ptr);
			OCT_LOG("flush: %x oct_read_ptr_B", oct_read_ptr);
			OCT_LOG("flush: %x num_bytes_B", num_bytes);

			/* Sync cache */
			dma_sync_single_for_cpu(NULL, phy_base_addr +
				oct_read_ptr, num_bytes, DMA_FROM_DEVICE);

			/* call subscribed function to forward data */
			oct_write_data_to_usb((char *)
			    (&((char *)oct_ext_rbuff_ptr)[oct_read_ptr]),
			    num_bytes);
			OCT_LOG("flush: %x write_ptr_A", write_ptr);
			OCT_LOG("flush: %x read_ptr_A", read_ptr);
			OCT_LOG("flush: %x oct_read_ptr_A", oct_read_ptr);
			OCT_LOG("flush: %x num_bytes_A", num_bytes);
		} else { /* if no data available */
		    OCT_DBG("flush: NO data available to be sent out");
		}
	} else if (!strncmp(buffer, "usb", 3)) {
		if (oct_out_path == OCT_PATH_TTY)
			oct_out_path = OCT_PATH_NONE;
		else
			oct_out_path = OCT_PATH_TTY;
	} else if (!strncmp(buffer, "open", 4)) {
		fp = filp_open("/dev/ttyGS1", O_RDWR, 0);
		OCT_LOG("Opening Gadget returned %ld", PTR_ERR(fp));
	} else {
		int nb_of_bytes;
		nb_of_bytes =  fp->f_op->write(fp, buffer, count, 0);
		OCT_LOG("Bytes to write %d, Bytes written %d",
				(uint32_t)count, nb_of_bytes);
	}
	if (count <= 0)
		OCT_LOG("OCT to USB write Error");
	}
	set_fs(old_fs);
	kfree(buffer);
	return count;
}

static int oct_open(struct inode *p_inode, struct file *p_file)
{
	return 0;
}

static int oct_release(struct inode *p_inode, struct file *p_file)
{
	return 0;
}

static long oct_ioctl(struct file *p_file,
		unsigned int cmnd, unsigned long param)
{
	unsigned wptr_un = 0;
	struct s_oct_info oct_info;
	int result;

	OCT_DBG("Ioctl cmd %d, param: %ld", cmnd, param);

	switch (cmnd) {
	case OCT_IOCTL_SET_PATH:
	    oct_out_path = param;
	    break;
	case OCT_IOCTL_SET_MODE:
	    oct_mode = param;
	    if (param == OCT_MODE_OFF)
		oct_mode_off();
	    else
		oct_hw_init();
		if (param == OCT_MODE_OW)
			oct_out_path = OCT_PATH_NONE;
	    break;
	case OCT_IOCTL_CONF_TRIG_CYCLE:
	    current_timeout = param * 1000 * 26;
	    /* reset & enable CYCLE interrupt */
	    SET_OCT_OCT_MASTER_TRIG_CYCLE_DMA_TRIG_CYCLE(oct_trform,
							    current_timeout);
	    SET_OCT_OCT_CNF_ENABLE_TRIG_CYCLE_INT(oct_trform,
				OCT_CNF_ENABLE_TRIG_CYCLE_INT_ENABLED);
	    break;
	case OCT_IOCTL_CONF_TRIG_L1:
	    /* configuration in procentage 0-100% */
	    oct_trig_level1 = (param * OCT_EXT_RING_BUFF_SIZE) / 100;
	    SET_OCT_OCT_MASTER_TRIG_LEVEL1_DMA_TRIG_LEVEL(oct_trform,
							oct_trig_level1 >> 2);
	    break;
	case OCT_IOCTL_GET_INFO:
	    wptr_un  =
		GET_OCT_OCT_MASTER_WPTR(oct_trform);
	    oct_info.wr_ptr = (unsigned int)
		OCT_MASTER_WPTR_MASTER_WPTR(wptr_un) << 2;
	    oct_info.rd_ptr =
		(unsigned int)GET_OCT_OCT_SW_RPTR(oct_trform);
	    oct_info.is_full =
		OCT_MASTER_WPTR_EXT_MEM_FULL(wptr_un);
	    result = copy_to_user((struct s_oct_info *)param,
			    &oct_info,
			    sizeof(oct_info));
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
	return 0;
}

static const struct file_operations oct_fops = {
	.owner			= THIS_MODULE,
	.aio_write		= NULL,
	.read			= oct_read,
	.poll			= oct_poll,
	.write			= oct_write,
	.open			= oct_open,
	.release		= oct_release,
	.unlocked_ioctl	= oct_ioctl
};

static int oct_driver_probe(struct platform_device *pdev)
{
	int result = 0;
	struct device *dev_oct;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct resource *mem;
	int size, err;

	OCT_DBG("%s", __func__);

	result = register_chrdev(0, OCT_MODULE_NAME, &oct_fops);
	if (result < 0) {
		OCT_LOG("can\'t register dev errorcode=%i", result);
		return -1;
	}

	major = result;
	class_oct = class_create(THIS_MODULE, OCT_MODULE_NAME);

	if (IS_ERR(class_oct)) {
		unregister_chrdev(major, OCT_MODULE_NAME);
		OCT_LOG("error creating class");
		return -1;
	}

	dev_oct = device_create(class_oct, NULL,
			MKDEV(major, 0),
			NULL, OCT_MODULE_NAME);
	if (IS_ERR(dev_oct)) {
		class_destroy(class_oct);
		unregister_chrdev(major, OCT_MODULE_NAME);
		OCT_LOG("Error creating device");
		return -1;
	}
	OCT_DBG("registered char device, major number=%i", major);

	mem = platform_get_resource_byname(pdev,
			IORESOURCE_MEM, "oct-registers");
	if (!mem) {
		OCT_LOG("no oct-registers resource?\n");
		return -1;
	}
	size = (mem->end - mem->start) + 1;
	oct_base = ioremap(mem->start, size);
	OCT_LOG("oct resource: %pR - ioremap: %p\n", mem, oct_base);
	if (!oct_base) {
		pr_err("%s: unable to remap memory region\n", __func__);
		release_mem_region(mem->start, size);
		return -1;
	}
	/* pm */
	pm_platdata = of_device_state_pm_setup(np);
	if (IS_ERR(pm_platdata)) {
		pr_err("%s: Error during device state pm init\n", __func__);
		return -1;
	}
	if (pm_platdata) {
		err = device_state_pm_set_class(dev, pm_platdata->pm_user_name);
		if (err) {
			pr_err("%s: Error while setting the pm class for user %s\n",
				__func__, pm_platdata->pm_user_name);
			return -1;
		}
		err = device_state_pm_set_state_by_name(dev,
				pm_platdata->pm_state_D0_name);
		if (err) {
			pr_err("%s: Error while setting the pm state: %s\n",
				__func__, pm_platdata->pm_state_D0_name);
			return -1;
		} else
			pr_debug("%s: %s requested for user %s\n", __func__,
					pm_platdata->pm_state_D0_name,
					pm_platdata->pm_user_name);
	} else
		pr_debug("pm_platdata is NULL\n");

	oct_mode = get_oct_mode();
	OCT_DBG("get_oct_mode returns: %x", oct_mode);
	oct_hw_init();
	if (oct_mode == OCT_MODE_OW)
		oct_out_path = OCT_PATH_NONE;
	oct_ext_mem_full = 0;
	init_waitqueue_head(&oct_wq);
	init_waitqueue_head(&oct_poll_wq);
	task = kthread_run(oct_thread, (void *)np, "OCT Thread");
	OCT_DBG("Kernel OCT Thread: %s", task->comm);

	oct_int = platform_get_irq_byname(pdev, "OCT_INT");
	if (oct_mode != OCT_MODE_OW && !IS_ERR_VALUE(oct_int)) {
		result = request_irq(OCT_INT,
				(irq_handler_t) oct_irq_handler,
				IRQF_SHARED,
				"OCT_Drv",
				(void *)(oct_irq_handler));
		if (result)
			OCT_LOG("can't get shared interrupt");
		else
			OCT_DBG("Interrupt %d installed", oct_int);
	}

	/* Runtime PM initialization */
	pm_runtime_enable(dev);
	OCT_DBG("PM runtime enabled");
	pm_runtime_irq_safe(dev);
	pm_runtime_get_sync(dev);
	OCT_DBG("PM runtime get");

	/* Do some other initialization.. if any.
	 * And put when no longer needed */
	pm_runtime_put_sync(dev);
	OCT_DBG("PM runtime put");

	return 0;
}

static int oct_driver_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int err;

	OCT_LOG("Removing...");

	/* Deregister IRQ */
	free_irq(OCT_INT, (void *)(oct_irq_handler));
	oct_events = 1;
	wake_up(&oct_wq);
	kthread_stop(task);
	iounmap(OCT_REG_ADDRESS_BASE);
	dma_unmap_single(NULL, phy_base_addr,
			OCT_EXT_RING_BUFF_SIZE, DMA_FROM_DEVICE);
	kfree(oct_ext_rbuff_ptr);

	device_destroy(class_oct, MKDEV(major, minor));
	class_destroy(class_oct);
	unregister_chrdev(major, OCT_MODULE_NAME);
	pm_runtime_disable(dev);
	OCT_DBG("device unregistration");
	if (pm_platdata) {
		err = device_state_pm_set_state_by_name(dev,
				pm_platdata->pm_state_D3_name);
		if (err) {
			pr_err("%s: Error while setting the pm state: %s\n",
				__func__, pm_platdata->pm_state_D3_name);
			return -1;
		}
	}

	return 0;
}

static const struct of_device_id xgold_oct_of_match[] = {
	{
		.compatible = "intel,oct",
	},
	{}
};

MODULE_DEVICE_TABLE(of, xgold_oct_of_match);

static struct platform_driver oct_driver = {
	.probe = oct_driver_probe,
	.remove = oct_driver_remove,
	.driver = {
		.name = PLATFORM_DRIVER_NAME,
		.pm	= &my_pm_ops,
		.of_match_table = of_match_ptr(xgold_oct_of_match),
	}
};

static int __init oct_init(void)
{
	int err;
	OCT_DBG("Module loading...ver %d", OCT_DRIVER_VER);

	err = platform_driver_register(&oct_driver);
	if (err) {
		OCT_LOG("Unable to register platform driver");
		return -1;
	}
	OCT_DBG("Platform driver registration ok...");
	return 0;
}

static void __exit oct_exit(void)
{
	/* Deregister Driver */
	platform_driver_unregister(&oct_driver);
	OCT_DBG("module removed");
}

module_init(oct_init);
module_exit(oct_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("OCT driver");
