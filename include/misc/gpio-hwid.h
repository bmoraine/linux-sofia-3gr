#ifndef _GPIO_HWID_H_
#define _GPIO_HWID_H_

typedef enum hwid {
	GPIO_HWID_STAGE0 = 0,
	GPIO_HWID_STAGE1 = 1,
	GPIO_HWID_STAGE2 = 2,
	HWID_COUNT,
} hwid_t;

hwid_t platform_gpio_hwid_get(void);

#endif /* _GPIO_HWID_H_ */
