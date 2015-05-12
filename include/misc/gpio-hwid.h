#ifndef _GPIO_HWID_H_
#define _GPIO_HWID_H_

enum hwid {
	GPIO_HWID_SR = 0, /* SR */
	GPIO_HWID_ER = 1, /* ER */
	GPIO_HWID_ER2 = 2, /* ER2*/
	GPIO_HWID_PR = 3, /* PR */
	HWID_COUNT,
};

enum lcmid {
	GPIO_LCMID_KD,
	GPIO_LCMID_BOE,
	LCMID_COUNT,
};

int platform_gpio_hwid_get(void);
int platform_gpio_lcmid_get(void);

#endif /* _GPIO_HWID_H_ */
