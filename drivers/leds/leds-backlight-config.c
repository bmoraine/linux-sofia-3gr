#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/string.h>

static char backlight_config[10];

static __init int get_backlight_config(char *str)
{
	int ret;

	if (!str)
		return -1;

	ret = sscanf(str, "%s", backlight_config);
	if (ret != 1) {
		pr_info("backlight config parse failed\n");
		return -1;
	}
	pr_info("backlight:%s\n", backlight_config);

	return 0;
}
__setup("sf3gr_mrd_version=", get_backlight_config);

int leds_backlight_config(char *str)
{
	return strcmp(backlight_config, str);
}
