
#define Focal_X_MAX 		2240 //picasso 2624	2240  asus 2240 3008 2240
#define Focal_Y_MAX		1408 //picasso 1728	1408	asus 1280 1856 1408
#define Focal_X_MAX_370T	2112
#define Focal_Y_MAX_370T	1280
#define Focal_X_MAX_202T	2944
#define Focal_Y_MAX_202T	1856


#ifndef _LINUX_FT3X17_H
#define _LINUX_FT3X17_H

/*************IO control setting***************/
#define TOUCH_TP_ON			1
#define TOUCH_TP_OFF			0
#define TOUCH_EC_ON			1
#define TOUCH_EC_OFF			0
#define TOUCH_FW_UPGRADE_ON		1
#define TOUCH_FW_UPGRADE_OFF		0
#define TOUCH_WIFI_ON			1
#define TOUCH_WIFI_OFF			0
#define TOUCH_IOC_MAGIC			0xF4
#define TOUCH_IOC_MAXNR			7
#define TOUCH_INIT			_IOR(TOUCH_IOC_MAGIC,	1,	int)
#define TOUCH_FW_UPDATE_FLAG		_IOR(TOUCH_IOC_MAGIC,	2,	int)
#define TOUCH_FW_UPDATE_PROCESS		_IOR(TOUCH_IOC_MAGIC,	3,	int)
#define TOUCH_TP_FW_check		_IOR(TOUCH_IOC_MAGIC,	4,	int)

#define TOUCH_FW_UPGRADE_SUCCESS	"0"
#define TOUCH_FW_UPGRADE_FAIL		"1"
#define TOUCH_FW_UPGRADE_PROCESS	"2"
#define TOUCH_FW_UPGRADE_INIT		"3"
/*************IO control setting***************/

#ifdef CONFIG_TOUCHSCREEN_FT3X27
#define FOCAL_TS_NAME	"ft3x27"
#endif
#define TOUCH_SDEV_NAME	"touch"

struct focal_i2c_platform_data
{
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int rst_gpio;
	int tp_en_gpio;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_sleep;
	struct pinctrl_state *pins_inactive;
	struct device_pm_platdata *pm_platdata;
};

#endif /* _LINUX_FT3x17_H */
