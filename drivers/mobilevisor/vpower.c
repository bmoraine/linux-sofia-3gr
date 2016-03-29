/*
 ****************************************************************
 *
 *  Copyright (C) 2014 Intel Mobile Communications GmbH
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
 *
 ****************************************************************
 */

#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/stddef.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/spinlock_types.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/clk.h>
#include <linux/clocksource.h>
#include <linux/of_irq.h>
#include <linux/device_state_pm.h>
#include <linux/debugfs.h>

#include <sofia/vpower.h>
#include <sofia/mv_hypercalls.h>
#include <sofia/pal_shared_data.h>
#include <sofia/mv_svc_hypercalls.h>
#include <sofia/mv_gal.h>

extern struct vmm_shared_data *vmm_shared_data[];
static DEFINE_MUTEX(call_mutex);

struct vpower_data {
	/* Internal */
	struct pal_shared_data *shared_data;
};


static DEFINE_PER_CPU(struct vpower_data, percpu_vpower);
static DECLARE_COMPLETION(prh_sync_complete);

static irqreturn_t vmm_prh_irq(int irq, void *dev_id)
{
	complete(&prh_sync_complete);
	return IRQ_HANDLED;
}
#define INTEL_PM_VPOWER "intel,vpower"
#define INTEL_PM_PRH_MODE "intel,prh_mode"
/* TODO: This parameter should be coming from the caller */
enum vmm_pm_opcode pm_opcode = PM_PRH_SET_PER_MODE;


static int of_vpower_parse(void)
{
	const char *vm_power_mode;
	int hirq_prh;
	struct device_node *dn;
	int ret;

	dn = of_find_compatible_node(NULL, NULL, INTEL_PM_VPOWER);
	if (!dn)
		return -EINVAL;

	hirq_prh = irq_of_parse_and_map(dn, 0);
	if (hirq_prh == 0) {
		pr_err("No PRH HIRQ found - going to die !\n");
		BUG();
	}

	ret = request_irq(hirq_prh, vmm_prh_irq,
			IRQF_NO_SUSPEND, "vmm hirq prh", NULL);
	if (ret) {
		pr_err("Installing HIRQ PRQ (%d) handler failed (%d)\n",
				hirq_prh, ret);
		BUG();
	}

	ret = of_property_read_string(dn, INTEL_PM_PRH_MODE, &vm_power_mode);
	if (!ret) {
		if (strcmp(vm_power_mode, "async") == 0)
			pm_opcode = PM_PRH_SET_PER_MODE_ASYNC;
		else if (strcmp(vm_power_mode, "sync") == 0)
			pm_opcode = PM_PRH_SET_PER_MODE;
		else
			pr_err("%s: Invalid (%s) vpower mode\n",
					__func__, vm_power_mode);

		pr_info("VPOWER: Running in %s mode\n", vm_power_mode);
	} else {
		pr_info("%s: %s is not defined - fallback to sync mode\n",
				__func__, INTEL_PM_PRH_MODE);
	}

	return 0;
}

static ePRH_RETURN_T vpower_init_prh(void)
{
	int i;
	int hirq_prh = 0;
	ePRH_RETURN_T retval;

	for (i = 0; i < num_possible_cpus(); i++) {
		struct vpower_data *vpower;
		struct vmm_shared_data *vmmdata = vmm_shared_data[i];
		vpower = &per_cpu(percpu_vpower, i);
		vpower->shared_data =
			(struct pal_shared_data *) vmmdata->pal_shared_mem_data;
	}

	of_vpower_parse();
	/* FIXME: We likely want to question VMM about the mode to be used
	 * a simple 'prh_init' vmcall service should be enough */
	retval = mv_svc_pm_control(PM_PRH_INIT_SET_MODE, 4, 8, 0);
	if (retval != PRH_OK)
		return retval;

	retval = mv_svc_pm_control(PM_PRH_PRE_INIT_SET_MODE, 4, 8, 0);
	if (retval != PRH_OK)
		return retval;

	if (!hirq_prh)
		return retval;


	return retval;
}

ePRH_RETURN_T vpower_call_prh(uint32_t user_id,
			uint32_t per_id,
			uint32_t * const p_per_mode_info,
			uint32_t size)
{
	ePRH_RETURN_T retval;
	struct vpower_data *vpower;

#ifdef CONFIG_VPOWER_STUB
	return PRH_OK;
#endif

/* FIXME: No sense to check on hard coded int there
 * Find a smarter way
   if (size >= 20)
		return PRH_ERR_INV_MODE_INFO;
*/
	mutex_lock(&call_mutex);

	vpower = &get_cpu_var(percpu_vpower);

	if ((vpower == NULL) || (vpower->shared_data == NULL)) {
		retval = PRH_ERR_INTERNAL;
		goto fail;
	}

	memset((&vpower->shared_data->pm_control_shared_data.prh_param), 0,
			PRH_MODE_INFO_SZ);
	memcpy((&vpower->shared_data->pm_control_shared_data.prh_param),
		  (void *) p_per_mode_info, size);

	reinit_completion(&prh_sync_complete);

	retval = mv_svc_pm_control(pm_opcode , user_id, per_id, 0);
	if (retval)
		pr_err("%s: Error(%i) arguments (%#x, %#x, %#x)\n",
			__func__, retval, PM_PRH_SET_PER_MODE, user_id, per_id);

fail:
	put_cpu_var(percpu_vpower);
	if ((retval == 0) && (pm_opcode == PM_PRH_SET_PER_MODE_ASYNC)) {
		retval = wait_for_completion_timeout(&prh_sync_complete,
				msecs_to_jiffies(30000));
		if (retval == 0) {
			pr_err("%s: Timeout waiting for PRH async interrupt\n", __func__);
			BUG();
		} else {
			retval = (ePRH_RETURN_T) vpower->shared_data
			->pm_control_shared_data.prh_request_return_value;
		}
	}

	mutex_unlock(&call_mutex);

	/* return the actual prh ret value from backend */
	return retval;
}

static char *power_id_str[] = {
	"NOT_AVAIL",
	"USIF1",
	"USIF2",
	"I2C1",
	"I2C2",
	"I2C3",
	"I2C4",
	"PWM",
	"RGA",
	"SDMMC1",
	"EMMC",
	"SDIO",
	"CIF",
	"DCC",
	"KEYPAD",
	"CEU",
	"CEU2",
	"USB_HS",
	"CAPCOM0",
	"CAPCOM1",
	"STM",
	"GPTU0",
	"GPTU1",
	"PCL",
	"RTC",
	"USIM",
	"USIM2",
	"PLL",
	"CAM_PRIM",
	"CAM_SEC",
	"PRIM_DISPLAY",
	"PRIM_DISP_BACKLIGHT",
	"TOUCHSCREEN",
	"TOUCH_SENSOR",
	"PROXIMITY_SENSOR",
	"ACCELEROMETER",
	"MAGNETOMETER",
	"GYROSCOPE",
	"GSI",
	"SHMEM",
	"GUCIPH",
	"ST_ARB",
	"ST_OCT",
	"ST_MON",
	"ST_MTM1",
	"ST_MTM2",
	"DMA4_CH",
	"DMA8_CH",
	"PS_CPU",
	"DSP_2G",
	"DSP_AUDIO",
	"3G_COMRAM_CPHY",
	"3G_COMRAM_PHY",
	"MACPHY",
	"DIG_RF",
	"ST_MON_SB_1",
	"ST_MON_SB_2",
	"ST_MON_SB_3",
	"ST_MON_SB_4",
	"ST_MON_SB_5",
	"ST_MON_SB_6",
	"ST_MON_SB_7",
	"ST_MON_SB_8",
	"ST_MON_SB_9",
	"ST_MON_SB_10",
	"ST_MON_SB_11",
	"ST_MON_SB_12",
	"NANDCTRL",
	"CST",
	"ETMA5",
	"AUDIO",
	"EMIC",
	"USB_PLL_E_480M",
	"GSER",
	"IDI",
	"OUT0",
	"OUT1",
	"ATCPTEST",
	"PMU_IF",
	"TSMU",
	"GPU",
	"VIDEO_DECODER",
	"VIDEO_ENCODER",
	"ABB_BT_IP",
	"ABB_BT_IF",
	"ABB_BT_AUD",
	"ABB_FMR",
	"ABB_AFE",
	"ABB_IDI",
	"ABB_RTC",
	"ABB_PCL",
	"ABB_VIBRATOR",
	"ABB_BACKLIGHT",
	"ABB_DIG_MIC",
	"ABB_MTM",
	"ABB_ST_ARB",
	"ABB_ST_MON",
	"ABB_DCDC",
	"ABB_PMU_CHP",
	"ABB_MS_CHP",
	"ABB_I2C",
	"ABB_WLAN",
	"ABB_USIF",
	"ABB_GNSS",
	"ABB_AUD_SYNC",
	"ABB_ST_MON_SB_1",
	"ABB_ST_MON_SB_2",
	"ABB_ST_MON_SB_3"
};

static int devices_state_show(struct seq_file *s, void *unused)
{
	struct vpower_data *vpower;
	int ret;
	unsigned int s3_count;
	unsigned long long s3_total_res, uptime, t, time;
	unsigned long nanosec_rem, remainder;
	unsigned long total_sec, total_msec;
	char *typestr = "s3               ";
	unsigned int pal_power_sleep_disabled[4];
	int i;

	vpower = &get_cpu_var(percpu_vpower);

	if ((vpower == NULL) || (vpower->shared_data == NULL))
		goto fail;

	ret = mv_svc_pm_control(PM_S3_COUNTER_UPDATE, 0, 0, 0);
	s3_count = vpower->shared_data->pm_state_shared_data.s3_count;
	s3_total_res = vpower->shared_data->pm_state_shared_data.s3_total_res;

	memcpy(pal_power_sleep_disabled,
	vpower->shared_data->pm_state_shared_data.pal_power_sleep_disabled,
	4*sizeof(unsigned int));

	uptime = cpu_clock(0);
	seq_printf(s, "\t\t\ttime(secs)\tresidency(%%)\tcount\tAvg.Res(Sec)\n");

	/* S3 total time */
	nanosec_rem = do_div(s3_total_res, NSEC_PER_SEC);
	seq_printf(s, "%s\t%5lu.%03lu\t",
		typestr,  (unsigned long)s3_total_res, nanosec_rem / 1000000);

	/* Residency(%) */
	nanosec_rem = do_div(uptime, NSEC_PER_SEC);
	total_sec = uptime;
	total_msec = nanosec_rem / 1000000;
	time = s3_total_res * 100;
	remainder = uptime ? do_div(time, uptime) : (time = 0);

	/* for getting 3 digit precision after
	 * decimal dot */
	t = remainder * 1000;
	uptime ? do_div(t, uptime) : (t = 0);

	seq_printf(s, "%5lu.%03lu\t",  (unsigned long)time,  (unsigned long)t);

	/* Count */
	seq_printf(s, "%d\t", s3_count);

	/* Avg.Res(Sec) */
	if (s3_count != 0) {
		time = s3_total_res;
		remainder = do_div(time, s3_count);
		t = remainder * 1000;
		do_div(t, s3_count);
	} else {
		time = 0;
		t = 0;
	}
	seq_printf(s, "%5lu.%03lu\n",  (unsigned long)time,  (unsigned long)t);

	/* Uptime */
	seq_printf(s, "\nTotal time: %5lu.%03lu Sec\n", total_sec, total_msec);

	/* Device pm enable/disable */
	seq_puts(s, "\nDevices                        status\n");
	for (i = 1;  i < POW_CONTROL_DEPRECATED_ID; i++)
		seq_printf(s, "%-30s  [%s]\n",
		power_id_str[i],
		(pal_power_sleep_disabled[i/32] & (1 << (i%32)))?"D0":"D0i3");

fail:
	put_cpu_var(percpu_vpower);

	return 0;
}


static int devices_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, devices_state_show, NULL);
}

static const struct file_operations devices_state_operations = {
	.open		= devices_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static char *wakeup_id_str[] = {
	"NOT_AVAIL",
	"DBB_KPD",         /**< wakeup by Keypad */
	"DBB_RES",         /**< Reserved bit */
	"DBB_SIM1",        /**<  SIM1 */
	"DBB_SIM2",        /**<  SIM2 */
	"DBB_DAP",         /**<  DAP */
	"DBB_OCT",         /**<  On-Chip Trace Module */
	"DBB_CPM",         /**<  CAPCOMs */
	"DBB_DSP",         /**<  DSP and FMRadio */
	"DBB_3G",          /**<  3G */
	"DBB_EXT0",        /**<  EXTINT 0 */
	"DBB_EXT1",        /**<  EXTINT 1 */
	"DBB_EXT2",        /**<  EXTINT 2 */
	"DBB_EXT3",        /**<  EXTINT 3 */
	"DBB_EXT4",        /**<  EXTINT 4 */
	"DBB_EXT5",        /**<  EXTINT 5 */
	"DBB_EXT6",        /**<  EXTINT 6 */
	"DBB_EXT7",        /**<  EXTINT 7 */
	"DBB_EXT8",        /**<  EXTINT 8 */
	"DBB_EXT9",        /**<  EXTINT 9 */
	"DBB_EXT10",       /**<  EXTINT 10 */
	"DBB_EXT11",       /**<  EXTINT 11 */
	"DBB_EXT12",       /**<  EXTINT 12 */
	"DBB_EXT13",       /**<  EXTINT 13 */
	"DBB_EXT14",       /**<  EXTINT 14 */
	"DBB_EXT15",       /**<  EXTINT 15 */
	"DBB_EXT16",       /**<  EXTINT 16 : USB_HS */
	"DBB_EXT17",       /**<  EXTINT 17 : SDMMC */
	"DBB_EXT18",       /**<  EXTINT 18 : SDIO */
	"DBB_EXT19",       /**<  EXTINT 19 : SDIO */
	"DBB_EXT20",       /**<  EXTINT 20 : SDIO */
	"DBB_EXT21",       /**<  EXTINT 21 : USIF1 */
	"DBB_EXT22",       /**<  EXTINT 22 : USIF2 */
	"DBB_GST_WKUP",    /**<  GST_WKUP */
	"DBB_nIRQOUT0",    /**<  nIRQOUT0 */
	"DBB_nIRQOUT1",    /**<  nIRQOUT1 */
	"DBB_nFIQOUT2",    /**<  nFIQOUT2 */
	"DBB_nFIQOUT3",    /**<  nFIQOUT3 */
	"DBB_USB_ID",      /**<  USB_ID */
	"NOT_AVAIL",
	"ABB_WLAN",        /**<  ABB WLAN  */
	"ABB_DAP",         /**<  ABB DAP  */
	"ABB_FMR",         /**<  FMR */
	"ABB_PMU",         /**<  PMU */
	"ABB_FSYS1",       /**<  FSYS1_EN */
	"ABB_RTC",         /**<  ABB RTC */
	"ABB_BT",          /**<  Bluetooth */
	"ABB_GLDO",        /**<  GLDO */
	"ABB_ACI_EN",      /**<  Accessory In */
	"DBB_REF_CLK_EN",  /**<  DBB REF CLK EN */
	"ABB_PEN_IRQ",     /**<  PEN IRQ */
	"ABB_FSYS2",       /**<  FSYS2_EN */
	"ABB_G2ARM",       /**<  G2ARM_EN */
	"ABB_GWDG",        /**<  GWDG_EN */
	"ABB_GGPIO",       /**<  GGPIO_EN */
	"WAKEUP_NOT_SLEPT",
};

static int devices_wakeup_state_show(struct seq_file *s, void *unused)
{
	int i;
	struct vmm_shared_data *data = mv_gal_get_system_shared_data();
	struct pal_shared_data *shared_mem =
			(struct pal_shared_data *)data->pal_shared_mem_data;
	seq_printf(s, "last_wakeup_source:\t\t\t%s\n",
	wakeup_id_str[shared_mem->pm_state_shared_data.last_wakeup_src]);

	seq_puts(s, "\nDevices wakeup counts:\n");
	for (i = 1;  i < SPCU_HWWUP_END-2; i++) {
		if (shared_mem->pm_state_shared_data.wakeup_counts[i] > 0)
			seq_printf(s, "%-30s \t\t\t[%d]\n",
			wakeup_id_str[i],
			shared_mem->pm_state_shared_data.wakeup_counts[i]);
	}

	return 0;
}

static int devices_wakeup_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, devices_wakeup_state_show, NULL);
}

static const struct file_operations devices_wakeup_state_operations = {
	.open		= devices_wakeup_state_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int __init vpower_init(void)
{
	vpower_init_prh();

	/* /sys/kernel/debug/mid_pmu_states */
	(void) debugfs_create_file("mid_pmu_states", S_IFREG | S_IRUGO,
				NULL, NULL, &devices_state_operations);
	(void) debugfs_create_file("mid_wakeup_states", S_IFREG | S_IRUGO,
				NULL, NULL, &devices_wakeup_state_operations);

	return 0;
}
arch_initcall(vpower_init);

