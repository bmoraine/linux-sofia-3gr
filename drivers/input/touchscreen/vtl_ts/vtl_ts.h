/*
 * VTL CTP driver
 *
 * Copyright (C) 2013 VTL Corporation.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program;
 *
 */
#ifndef	_TS_CORE_H_
#define	_TS_CORE_H_

#include <linux/gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif


/*vtl touch IC define*/
#define CT36X			0x01/*(CT36X:ct362,ct363,ct365)*/
#define	CT360			0x02/*(CT360:ct360)*/

/*xy data protocol*/
#define OLD_PROTOCOL		0x01
#define	NEW_PROTOCOL		0x02



/***********************vtl ts driver config *******************/

/*vtl chip ID*/
#define	CHIP_ID			CT36X/*CT360*/

#define	XY_DATA_PROTOCOL	NEW_PROTOCOL /*OLD_PROTOCOL*/

#define TS_I2C_SPEED		400000	    /*for rockchip*/


#define		XY_SWAP_ENABLE		1

#define		X_REVERSE_ENABLE        0

#define		Y_REVERSE_ENABLE        1

#define		CHIP_UPDATE_ENABLE	1

#define		DEBUG_ENABLE		0


/***********************vtl ts driver config  end*****************/




/*vtl ts driver name*/
#define DRIVER_NAME		"vtl_ts"

#if (DEBUG_ENABLE)
#define	DEBUG() pr_debug("___%s___\n", __func__);
#else
#define		DEBUG()

#endif

#define	TOUCH_POINT_NUM		5

/*priate define and declare*/
#if (CHIP_ID == CT360)
struct xy_data {
	#if (XY_DATA_PROTOCOL == OLD_PROTOCOL)
	/* Action information, 1: Down; 2: Move; 3: Up */
	unsigned char	status:4;
	/* ID information, from 1 to CFG_MAX_POINT_NUM */
	unsigned char	id:4;
	#endif
	unsigned char	xhi;			/* X coordinate Hi */
	unsigned char	yhi;			/* Y coordinate Hi */
	unsigned char	ylo:4;		/* Y coordinate Lo */
	unsigned char	xlo:4;		/* X coordinate Lo */
	#if (XY_DATA_PROTOCOL == NEW_PROTOCOL)
	/* Action information, 1: Down; 2: Move; 3: Up */
	unsigned char	status:4;
	/* ID information, from 1 to CFG_MAX_POINT_NUM */
	unsigned char	id:4;
	#endif
};
#else
struct xy_data {
	#if (XY_DATA_PROTOCOL == OLD_PROTOCOL)
	/* Action information, 1: Down; 2: Move; 3: Up */
	unsigned char	status:3;
	/* ID information, from 1 to CFG_MAX_POINT_NUM */
	unsigned char	id:5;
	#endif
	unsigned char	xhi;			/* X coordinate Hi */
	unsigned char	yhi;			/* Y coordinate Hi */
	unsigned char	ylo:4;		/* Y coordinate Lo */
	unsigned char	xlo:4;		/* X coordinate Lo */
	#if (XY_DATA_PROTOCOL == NEW_PROTOCOL)
	/* Action information, 1: Down; 2: Move; 3: Up */
	unsigned char	status:3;
	/* ID information, from 1 to CFG_MAX_POINT_NUM */
	unsigned char	id:5;
	#endif
	unsigned char	area;			/* Touch area */
	unsigned char	pressure;		/* Touch Pressure */
};
#endif


union ts_xy_data {
	struct xy_data	point[TOUCH_POINT_NUM];
	unsigned char	buf[TOUCH_POINT_NUM * sizeof(struct xy_data)];
};


struct ts_driver {

	struct i2c_client		*client;

	/* input devices */
	struct input_dev		*input_dev;

	struct proc_dir_entry		*proc_entry;

	struct task_struct		*ts_thread;

	#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend		early_suspend;
	#endif
};

struct ts_config_info {
	unsigned int	screen_max_x;
	unsigned int	screen_max_y;
	unsigned int irq_gpio_number;
	unsigned int irq_number;
	unsigned int rst_gpio_number;
	unsigned char touch_point_number;
	unsigned char ctp_used;

	struct regulator *power;
	struct regulator *power2;

	struct device_pm_platdata *pm_platdata;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	int irq_pin;
	int reset_pin;
	bool polling_mode;

};

struct	ts_info {
	struct ts_driver	*driver;
	struct ts_config_info	*config_info;
	union ts_xy_data	xy_data;
	unsigned char		debug;
};



extern struct ts_info *vtl_ts_get_object(void);
extern void vtl_ts_hw_reset(void);

#endif
