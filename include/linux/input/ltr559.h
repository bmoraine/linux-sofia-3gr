/*
 *  Adaptations were made to original apds990x.h with
 *  ltr559.h - Linux kernel modules for ambient light + proximity sensor
 *
 *  Copyright (C) 2012-2013 Intel Mobile Communications GmbH
 *  Copyright (C) 2010 Lee Kai Koon <kai-koon.lee@avagotech.com>
 *  Copyright (C) 2010 Avago Technologies
 */

#ifndef __LTR_ltr559_H__
#define __LTR_ltr559_H__

#define LTR559_DEV_NAME		"ltr559"

struct als_platform_data {
	int (*power_on)(struct device *dev);
	int (*power_off)(struct device *dev);
	int (*init)(struct device *dev);
	void (*exit)(void);
	/* gpio ports for interrupt pads */
	int gpio_int;
};
#endif  /* __LTR_ltr559_H__ */
