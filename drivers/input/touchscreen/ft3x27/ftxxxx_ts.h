#ifndef __LINUX_ftxxxx_TS_H__
#define __LINUX_ftxxxx_TS_H__

#include <linux/i2c.h>
#include <linux/input.h>
//#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include <linux/switch.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>

/* -- dirver configure -- */
#define CFG_MAX_TOUCH_POINTS	10

#define PRESS_MAX	0xFF
#define FT_PRESS	0x08

#define FTXXXX_NAME	"ft3x27"
#define Focal_input_dev_name	"focal-touchscreen"


#define FT_MAX_ID	0x0F
#define FT_TOUCH_STEP	6
#define FT_TOUCH_X_H_POS		3
#define FT_TOUCH_X_L_POS		4
#define FT_TOUCH_Y_H_POS		5
#define FT_TOUCH_Y_L_POS		6
#define FT_TOUCH_XY_POS			7
#define FT_TOUCH_MISC			8
#define FT_TOUCH_EVENT_POS		3
#define FT_TOUCH_ID_POS			5

#define POINT_READ_BUF	(3 + FT_TOUCH_STEP * CFG_MAX_TOUCH_POINTS)

#define TPID_1	96 + 16		/* pcb_id  3 */
#define TPID_2	96 + 15		/* pcb_id 12 */
#define TPID_3	96 +  7		/* pcb_id 10 */

/*register address*/
#define FTXXXX_REG_FW_VER		0xA6
#define FTXXXX_REG_POINT_RATE	0x88
#define FTXXXX_REG_THGROUP	0x80
#define FTXXXX_REG_VENDOR_ID	0xA8
#define FTXXXX_REG_PROJECT_ID	0xA1

#define FTXXXX_ENABLE_IRQ	1
#define FTXXXX_DISABLE_IRQ	0

#define FT_COORDS_ARR_SIZE	4

int ftxxxx_i2c_Read(struct i2c_client *client, char *writebuf, int writelen, char *readbuf, int readlen);
int ftxxxx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen);

void ftxxxx_reset_tp(int HighOrLow);
int ftxxxx_read_tp_id(void);
u8 get_focal_tp_fw(void);

void ftxxxx_irq_switch(int enable);

void ftxxxx_enable(int enable);

void set_event_printable(int on);

/* The platform data for the Focaltech ftxxxx touchscreen driver */
struct ftxxxx_platform_data {
	uint32_t gpio_irq;			/* IRQ port*/
	uint32_t irq_cfg;

	uint32_t gpio_wakeup;		/* Wakeup support*/
	uint32_t wakeup_cfg;

	uint32_t gpio_reset;		/* Reset support*/
	uint32_t reset_cfg;

	int screen_max_x;
	int screen_max_y;
	int pressure_max;
};

struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:0 -- down; 1-- contact; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u8 au8_finger_weight[CFG_MAX_TOUCH_POINTS];	/*touch weight */
	u8 pressure[CFG_MAX_TOUCH_POINTS];
	u8 area[CFG_MAX_TOUCH_POINTS];
	u8 touch_point;
};

struct ftxxxx_ts_data {
	unsigned int irq;
	short irq_count;
	unsigned int x_max;
	unsigned int y_max;
	unsigned int init_success;
	bool suspend_flag;
	bool usb_status;
	bool reset_pin_status;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct ts_event event;
	struct switch_dev touch_sdev;
	struct focal_i2c_platform_data *pdata;
	struct mutex g_device_mutex;
	struct workqueue_struct *usb_wq;
	struct work_struct usb_detect_work;
	struct workqueue_struct *reset_wq;
	struct delayed_work reset_ic_work;
	struct workqueue_struct *resume_wq;
	struct work_struct resume_work;
#ifdef CONFIG_PM
	struct early_suspend *early_suspend;
#endif
	/* Wakelock Protect */
	struct wake_lock wake_lock;
	/* Wakelock Protect */
	struct mutex irq_mutex;
};

#endif
