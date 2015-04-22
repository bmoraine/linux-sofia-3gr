/**
 * -------------------------------------------------------------------------
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; If not, see <http://www.gnu.org/licenses/>.
 *
 */

#define DRIVER_NAME					"pmic_bat_drv"
#define pr_fmt(fmt) DRIVER_NAME": "fmt

#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/time.h>
#include <linux/power/battery_id.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>
#include <sofia/vmm_pmic.h>
#include <sofia/vmm_pmic-ext.h>

#include <linux/slab.h>
#include <linux/string.h>

#include <linux/iio/iio.h>
#include <linux/iio/consumer.h>

#include <linux/platform_device.h>
#include "bprofile_dts_parser.h"

#define BAT_DRV_DEBUG_DATA_SIZE (2<<6)

/* Size of array of function payloads to implement work FIFO. */
#define BAT_DRV_HAL_WORK_FIFO_LENGTH (32)

#define BASE_ADDRESS_DEV1 (0x4E)
#define BASE_ADDRESS_DEV1_VMM (BASE_ADDRESS_DEV1 << 24)

/* Charger Interrupt Register 1 */
#define CHGRIRQ1_REG (BASE_ADDRESS_DEV1_VMM | 0xA)

/* Battery presence detection interrupt */
#define BATTDET_M 1
#define BATTDET_O 2

/* MCHGRIRQ1 Interrupt Mask Register */
#define MCHGRIRQ1_REG (BASE_ADDRESS_DEV1_VMM | 0x18)

/* Battery presence detection interrupt mask */
#define MBATTDET_M 1
#define MBATTDET_O 2

#define MIRQLVL1_REG (BASE_ADDRESS_DEV1_VMM | 0x0E)

/* mask/unmask CHGR IRQ */
#define MCHGR_M 1
#define MCHGR_O 5

/* Power Source Interrupt Status Register */
#define SPWRSRC_REG (BASE_ADDRESS_DEV1_VMM | 0x20)

#define SBATTDET_M 1
#define SBATTDET_O 2

/* Valid Battery Detection Register 0 */
#define LOWBATTDET0_REG (BASE_ADDRESS_DEV1_VMM | 0x21)

#define LOWBATT_M (0xf)
#define LOWBATT_O 0

/* Valid Battery Detection Register 1 */
#define LOWBATTDET1_REG (BASE_ADDRESS_DEV1_VMM | 0x22)

#define LOWBATTDCP_M (0xf)
#define LOWBATTDCP_O 4

#define LOWBATTSDP_M (0xf)
#define LOWBATTSDP_O 0

/* Power Source Detect Configuration Register */
#define PSDETCTRL_REG (BASE_ADDRESS_DEV1_VMM | 0x23)

#define BATTRMSRC_M (0x3)
#define BATTRMSRC_O 4

#define BATTRMPDEN_M 1
#define BATTRMPDEN_O 3

#define NTC 2
#define BATTID 1
#define VBATT 0

/* Battery Removal Control Register 0 */
#define BATTDETCTRL0_REG (BASE_ADDRESS_DEV1_VMM | 0x24)

#define BATTYP_M (0x7)
#define BATTYP_O 5

#define BATTDBEN_M 1
#define BATTDBEN_O 0

/* Battery Removal Control Register 1 */
#define BATTDETCTRL1_REG (BASE_ADDRESS_DEV1_VMM | 0x25)

#define BRMDBC_M (0x1f)
#define BRMDBC_O 0

/* TIME = (BRMDBC * 2 +1) * RTC cycles.
Max time achievable is ~2ms */
#define BAT_RM_DBC_MAX 0x1f

/* Threshold in ohm for whether NTC will be detected as dummy battery */
#define TBATID_DUMMY_THRESHOLD_OHM 300000

#define BATID_DUMMY_STR "dummy"

#define BATID_STANDARD_STR "standrd"

#define BATTID_STR_LEN 8

#define reg_set_field(_reg, _field, _val)\
do {\
	_reg &= ~(_field##_M << _field##_O);\
	_reg |= (_val & _field##_M) << _field##_O;\
} while (0)

#define reg_read_field(_reg, _field)\
({\
	u32 _data;\
	_data = (_reg >> _field##_O) & _field##_M;\
	_data;\
})

enum {
	DISCONNECTED,
	CONNECTED,

	DISABLE = 0,
	ENABLE,

	UNMASK = 0,
	MASK,

	NOT_PENDING = 0,
	PENDING,

	CLEAR = 1,
};

/* Macro to trace and log debug data internally. Jiffy resolution is adequate
for Bat Drv HAL */
#define BAT_DRV_DEBUG(_array, _event, _param) \
{ \
	unsigned long flags; \
	spin_lock_irqsave(&_array.lock, flags); \
	_array.log_array[_array.index].time_stamp = jiffies; \
	_array.log_array[_array.index].event = (_event); \
	_array.log_array[_array.index].param = (long)(_param); \
	_array.index++; \
	_array.index &= (BAT_DRV_DEBUG_DATA_SIZE-1); \
	spin_unlock_irqrestore(&_array.lock, flags); \
	pr_debug("%s 0x%lx  dec=%ld\n", #_event, \
				(unsigned long)_param, (long)_param); \
}

/* Macro to trace and log debug event and data. */
#define BAT_DRV_DEBUG_PARAM(_event, _param) \
		BAT_DRV_DEBUG(bat_drv_debug_info, _event, _param)

/* Macro to trace and log debug event without a parameter. */
#define BAT_DRV_DEBUG_NO_PARAM(_event) \
		BAT_DRV_DEBUG(bat_drv_debug_info, _event, 0)

#define BAT_DRV_HAL_ENQUEUE(pbat, p_func, param) \
	bat_drv_enqueue_function(pbat, (fp_scheduled_function)(p_func), \
								(long)(param))

struct bat_drv_data;

/* Function type for scheduled execution by work. */
typedef void (*fp_scheduled_function) (struct bat_drv_data *, long);
static int bat_drv_get_batid_ohm(struct bat_drv_data *pbat);
static int pmic_set_bat_type(struct bat_drv_data *pbat, enum battery_type_id);

/* Message payload for work scheduler queue. */
struct bat_drv_fifo_payload {
	fp_scheduled_function p_func;
	long param;
};

/*
 * Message queue for scheduled work.
 * This is a ring buffer protected by a spinlock,
 * as access is required from interrupt and timer callback context.
 */
struct bat_drv_work_fifo {
	DECLARE_KFIFO(fifo, struct bat_drv_fifo_payload,
		BAT_DRV_HAL_WORK_FIFO_LENGTH);
	spinlock_t lock;

	struct wake_lock kfifo_wakelock;

	struct workqueue_struct *p_work_queue;
	struct work_struct work;
	bool removal_pending;
};

/** Events for use in debug and tracing. */
enum bat_drv_debug_event {
	BAT_DRV_DEBUG_EVENT_INIT,
	BAT_DRV_DEBUG_EVENT_EXIT,
	BAT_DRV_DEBUG_EVENT_PROBE,
	BAT_DRV_DEBUG_EVENT_REMOVE,
	BAT_DRV_DEBUG_EVENT_SUSPEND,
	BAT_DRV_DEBUG_EVENT_RESUME,

	BAT_DRV_DEBUG_ENQUEUE_FUNCTION,
	BAT_DRV_DEBUG_ENQUEUE_PARAM,
	BAT_DRV_DEBUG_EXECUTE_FUNCTION,
	BAT_DRV_DEBUG_EXECUTE_PARAM,

	BAT_DRV_DEBUG_BAT_CONNECTED,
	BAT_DRV_DEBUG_BAT_DISCONNECTED,

	BAT_DRV_DEBUG_TICK_STM,
	BAT_DRV_DEBUG_INIT_PROCESS_EVT,
	BAT_DRV_DEBUG_CONN_PROCESS_EVT,
	BAT_DRV_DEBUG_DISCONN_PROCESS_EVT,

	BAT_DRV_DEBUG_HW_INIT,
	BAT_DRV_DEBUG_PRESENCE_CHANGE_TH_IRQ,
	BAT_DRV_DEBUG_BATID_IN_OHMS,
	BAT_DRV_DEBUG_TBATID_IN_OHMS,
	BAT_DRV_DEBUG_REPORTED_BATID_IN_OHMS,

	BAT_DRV_DEBUG_EVENT_BAT_CHANGED,
};

/**
 * struct bat_drv_debug_data - Structure to collect debug data
 * @lock		Spinlock for atomic access
 * @index		Index of logging array
 * @log_array		Debug data logging array
 *	@time_stamp	System Time Stamp in Jiffies
 *	@event		Event which occurred
 *	@param		General purpose parameter
 */
struct bat_drv_debug_data {
	spinlock_t lock;
	u32 index;
	struct {
		u32 time_stamp;
		enum bat_drv_debug_event event;
		long param;
	} log_array[BAT_DRV_DEBUG_DATA_SIZE];
};

/**  Presence state machine events */
enum bat_drv_presence_stm_event {
	BAT_DRV_STM_EVENT_IRQ_PRESENCE_CHANGED,
	BAT_DRV_STM_EVENT_DET_BAT_PRESENCE,
};

/**  Presence state machine states */
enum bat_drv_presence_stm_state {
	BAT_DRV_STM_STATE_INIT,
	BAT_DRV_STM_STATE_BATTERY_CONNECTED,
	BAT_DRV_STM_STATE_BATTERY_DISCONNECTED,
};

/* Presence state machine data */
struct bat_drv_presence_stm {
	enum bat_drv_presence_stm_state current_state;
};


struct bat_drv_platform_data {
	struct battery_type *supported_batteries;
	size_t supported_batteries_len;

	struct ps_pse_mod_prof *bprofiles;
	size_t bprofiles_len;
};


/**
 * struct bat_drv_data		Battery Driver Hal control structure
 * @initialised			Driver initialisation state
 * @pdev			Pointer to platform device
 * @use_tbatid			whether to use NTC as battery ID
 * @batid_iio_chan		iio channel to read Battery ID from
 * @tbatid_iio_chan		iio channel to read NTC resistance
 * @presence_stm		presence state machine
 * @work			messages FIFO
 * @irq				associated battery removal/detection irq
 * @supported_batteries		array of supported battery ids
 * @supported_batteries_len	the lenght of above array
 * @bprofiles			array of battery profiles
 * @bprofiles_len		the lenght of above array
 */
struct bat_drv_data {
	bool initialised;
	struct platform_device *pdev;

	bool use_tbatid;
	struct iio_channel *batid_iio_chan;
	struct iio_channel *tbatid_iio_chan;

	struct bat_drv_presence_stm presence_stm;
	struct bat_drv_work_fifo work;
	int irq;

	struct bat_drv_platform_data pdata;
};

/* Bat Driver Hal instance */
static struct bat_drv_data bat_drv_instance = {

	.presence_stm = {
		.current_state =
			BAT_DRV_STM_STATE_INIT,
	},

	.initialised = false,
	.use_tbatid = false,
};

/* Array to collect debug data */
static struct bat_drv_debug_data bat_drv_debug_info;

static struct battery_type *get_bat_type(struct bat_drv_data *pbat,
					unsigned int batid_ohms)
{
	unsigned int ibatid_min_th, ibatid_max_th;
	int i, len = pbat->pdata.supported_batteries_len;

	if (!pbat)
		return NULL;

	if (pbat->use_tbatid) {
		char *id_str;
		int id_len;
		struct ps_pse_mod_prof *p_model;

		if (batid_ohms >= TBATID_DUMMY_THRESHOLD_OHM)
			id_str = BATID_DUMMY_STR;
		else
			id_str = BATID_STANDARD_STR;

		id_len = strnlen(id_str, BATTID_STR_LEN);

		for (i = 0; i < len; i++) {
			p_model = (struct ps_pse_mod_prof *) pbat->pdata.
				supported_batteries[i].profile.batt_prof;

			if (!p_model)
				continue;

			if (0 == strncmp(p_model->batt_id, id_str, id_len)) {
				pr_info("Profile found for NTC %dohm %s type battery\n",
					batid_ohms, id_str);
				return &pbat->pdata.supported_batteries[i];
			}
		}

		pr_err("%s - No profile found for NTC %dohm %s type battery",
			__func__, batid_ohms, id_str);

		return NULL;
	}

	if (batid_ohms > pbat->pdata.supported_batteries[len-1].batid_ohms)
		return NULL;

	if (batid_ohms < pbat->pdata.supported_batteries[0].batid_ohms)
		return NULL;

	if (1 == len)
		return &pbat->pdata.supported_batteries[0];

	for (i = 1; i < len; ++i) {
		if (batid_ohms <= pbat->pdata.
				supported_batteries[i].batid_ohms)
			break;
	}

	ibatid_min_th = pbat->pdata.supported_batteries[i-1].batid_ohms;
	ibatid_max_th = pbat->pdata.supported_batteries[i].batid_ohms;

	if (batid_ohms >= (ibatid_min_th + ibatid_max_th)/2)
		return &pbat->pdata.supported_batteries[i];
	else
		return &pbat->pdata.supported_batteries[i-1];
}


/**
 * bat_drv_enqueue_function - Adds a function to the message queue for the
 * serialisation work. This function is supplied to the HAL to allow processing
 * in a single thread work for static data. Since it may be called from
 * interrupt or timer callback context, the locking mechanism is spinlock.
 *
 * @pbat		[in] pointer to driver structure
 * @p_function		[in] Function to be added to the execution FIFO.
 * @param		[in] Parameter value for the function.
 */
static void bat_drv_enqueue_function(struct bat_drv_data *pbat,
				fp_scheduled_function p_function,
							long param)
{
	unsigned long flags;

	/* Functions may not be scheduled while the device is being removed. */
	if (!pbat->work.removal_pending) {
		struct bat_drv_fifo_payload payload = {
			.p_func = p_function,
			.param = param
		};

		spin_lock_irqsave(&pbat->work.lock, flags);

		/*
		* Message queue is a critical region.
		* kfifo() needs explicit locking when there
		* are multiple consumers or producers.
		*/
		BUG_ON(0 == kfifo_in(&pbat->work.fifo,
				&payload, 1));

		BAT_DRV_DEBUG_PARAM(BAT_DRV_DEBUG_ENQUEUE_FUNCTION,
					p_function);
		BAT_DRV_DEBUG_PARAM(BAT_DRV_DEBUG_ENQUEUE_PARAM, param);

		wake_lock(&pbat->work.kfifo_wakelock);

		/* Schedule the work queue to process the message. */
		(void)queue_work(pbat->work.p_work_queue,
				&pbat->work.work);

		spin_unlock_irqrestore(&pbat->work.lock, flags);
	}
}

/**
 * bat_drv_execute_function - Remove from queue and execute all scheduled
 * functions in a work.
 *
 * @work		[in] work to be executed
 */
static void bat_drv_execute_function(struct work_struct *work)
{
	struct bat_drv_fifo_payload payload;
	unsigned long flags;

	struct bat_drv_data *pbat =
			container_of(work, struct bat_drv_data, work.work);

	/* Repeatedly fetch one entry from the fifo and process it until the
	fifo is empty. */
	while (0 != kfifo_out_spinlocked(&pbat->work.fifo,
					&payload,
					1, &pbat->work.lock)) {

		BAT_DRV_DEBUG_PARAM(BAT_DRV_DEBUG_EXECUTE_FUNCTION,
					payload.p_func);
		BAT_DRV_DEBUG_PARAM(BAT_DRV_DEBUG_EXECUTE_PARAM,
					payload.param);
		/* Execute the function. */
		payload.p_func(pbat, payload.param);

		/* wakelock acquisition must be in sync with kfifo operation */
		spin_lock_irqsave(&pbat->work.lock, flags);

		/* If work fifo is empty the wakelock can be released */
		if (kfifo_is_empty(&pbat->work.fifo))
			wake_unlock(&pbat->work.kfifo_wakelock);

		spin_unlock_irqrestore(&pbat->work.lock, flags);
	}
}


/**
 * bat_drv_batrm_det_irq_en - Enable or disable the battery removal
 * detection IRQ If the state has not changed since the last call,
 * no propgagation to the HW is made.
 *
 * @pbat	[in] pointer to driver's structure
 * @new_state	[in] new state of irq: enable or disable
 */
static void bat_drv_batrm_det_irq_en(struct bat_drv_data *pbat, bool new_state)
{
	static int batrm_det_irq_enabled = -1;

	if (new_state != batrm_det_irq_enabled) {
		if (new_state)
			enable_irq(pbat->irq);
		else
			disable_irq(pbat->irq);

		batrm_det_irq_enabled = new_state;
	}
}


/**
 * bat_drv_update_presence_status - Update subscribers with battery
 * info if needed.
 *
 * @pbat	[in] pointer to battery driver's structure
 * @connected	[in] TRUE if battery is fitted, else FALSE.
 * @bat_type	[in] associated battery_type pointer
 */
static void bat_drv_update_presence_status(struct bat_drv_data *pbat,
			bool connected, struct battery_type *bat_type)
{
	if (connected) {
		battery_prop_changed(POWER_SUPPLY_BATTERY_INSERTED,
				&bat_type->profile);

		BAT_DRV_DEBUG_PARAM(BAT_DRV_DEBUG_REPORTED_BATID_IN_OHMS,
						bat_type->batid_ohms);
	}
	else
		battery_prop_changed(POWER_SUPPLY_BATTERY_REMOVED,
				NULL);
}

static bool bat_drv_get_bat_detection_status(struct bat_drv_data *pbat)
{
	u32 spwrsrc_reg = 0;

	vmm_pmic_reg_read(SPWRSRC_REG, &spwrsrc_reg);

	return CONNECTED == reg_read_field(spwrsrc_reg, SBATTDET);
}


static void stm_init_state_process_evt(struct bat_drv_data *pbat,
				enum bat_drv_presence_stm_event event)
{
	struct battery_type *bat_type = NULL;

	BAT_DRV_DEBUG_PARAM(BAT_DRV_DEBUG_INIT_PROCESS_EVT, event);

	switch (event) {

	case BAT_DRV_STM_EVENT_DET_BAT_PRESENCE:
		bat_drv_batrm_det_irq_en(pbat, true);

		if (bat_drv_get_bat_detection_status(pbat)) {
			int batid_ohm;

			batid_ohm = bat_drv_get_batid_ohm(pbat);
			if (batid_ohm >= 0)
				bat_type = get_bat_type(pbat, batid_ohm);
		}

		if (NULL != bat_type) {

			pbat->presence_stm.current_state =
				BAT_DRV_STM_STATE_BATTERY_CONNECTED;

			pmic_set_bat_type(pbat, bat_type->type);

			bat_drv_update_presence_status(pbat, true, bat_type);

			BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_BAT_CONNECTED);
		} else {
			pbat->presence_stm.current_state =
				BAT_DRV_STM_STATE_BATTERY_DISCONNECTED;

			pmic_set_bat_type(pbat, BAT_TYPE_MIPI);

			bat_drv_update_presence_status(pbat, false, NULL);

			BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_BAT_DISCONNECTED);
		}
		break;

	case BAT_DRV_STM_EVENT_IRQ_PRESENCE_CHANGED:
	default:
		pr_err("%s: unexpected event! (event=%d)\n", __func__, event);
		break;
	}
}

static void stm_bat_connected_state_process_evt(struct bat_drv_data *pbat,
				enum bat_drv_presence_stm_event event)
{
	BAT_DRV_DEBUG_PARAM(BAT_DRV_DEBUG_CONN_PROCESS_EVT, event);

	switch (event) {

	case BAT_DRV_STM_EVENT_IRQ_PRESENCE_CHANGED:
		if (!bat_drv_get_bat_detection_status(pbat)) {
			pbat->presence_stm.current_state =
				BAT_DRV_STM_STATE_BATTERY_DISCONNECTED;

			pmic_set_bat_type(pbat, BAT_TYPE_MIPI);

			bat_drv_update_presence_status(pbat, false, NULL);

			BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_BAT_DISCONNECTED);
		} else {
			pr_err("%s: consecutive connected updates!\n",
								__func__);
		}
		break;


	case BAT_DRV_STM_EVENT_DET_BAT_PRESENCE:
	default:
		pr_err("%s: unexpected event! (event=%d)\n", __func__, event);
		break;
	}
}

static void stm_bat_disconnected_state_process_evt(struct bat_drv_data *pbat,
				enum bat_drv_presence_stm_event event)
{
	struct battery_type *bat_type = NULL;

	BAT_DRV_DEBUG_PARAM(BAT_DRV_DEBUG_DISCONN_PROCESS_EVT, event);

	switch (event) {

	case BAT_DRV_STM_EVENT_IRQ_PRESENCE_CHANGED:
		if (bat_drv_get_bat_detection_status(pbat)) {
			int batid_ohm;

			batid_ohm = bat_drv_get_batid_ohm(pbat);
			if (batid_ohm >= 0)
				bat_type = get_bat_type(pbat, batid_ohm);
		}

		if (NULL != bat_type) {
			pbat->presence_stm.current_state =
				BAT_DRV_STM_STATE_BATTERY_CONNECTED;

			pmic_set_bat_type(pbat, bat_type->type);

			bat_drv_update_presence_status(pbat, true, bat_type);

			BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_BAT_CONNECTED);

		} else {
			pr_err("%s: consecutive disconnected updates!\n",
								__func__);
		}
		break;

	case BAT_DRV_STM_EVENT_DET_BAT_PRESENCE:
	default:
		pr_err("%s: unexpected event! (event=%d)\n", __func__, event);
		break;
	}
}


/**
 * bat_drv_presence_stm_tick - tick PMU hal presence state machine with new
 * event.
 *
 * @pbat	[in] pointer to battery driver's structure
 * @event	[in] State machine event
*/
static void bat_drv_presence_stm_tick(struct bat_drv_data *pbat,
				enum bat_drv_presence_stm_event event)
{
	enum bat_drv_presence_stm_state current_state =
				pbat->presence_stm.current_state;

	BAT_DRV_DEBUG_PARAM(BAT_DRV_DEBUG_TICK_STM, event);

	switch (current_state) {
	case BAT_DRV_STM_STATE_INIT:
		stm_init_state_process_evt(pbat, event);
		break;

	case BAT_DRV_STM_STATE_BATTERY_CONNECTED:
		stm_bat_connected_state_process_evt(pbat, event);
		break;

	case BAT_DRV_STM_STATE_BATTERY_DISCONNECTED:
		stm_bat_disconnected_state_process_evt(pbat, event);
		break;

	default:
		break;
	}
}

/**
 * bat_presence_change_th - irq thread for battery presence irq
 * state change. Schedules a work to signal the state machine, This function is
 * triggered by the battery removal detection IRQ.
 *
 * @irq		[in] (not used)
 * @dev		[in] conveys the pointer to battery driver's structure
 */

static irqreturn_t bat_presence_change_th(int irq, void *dev)
{
	struct bat_drv_data *pbat = (struct bat_drv_data *)dev;

	BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_PRESENCE_CHANGE_TH_IRQ);

	BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_EVENT_BAT_CHANGED);

	BAT_DRV_HAL_ENQUEUE(pbat, bat_drv_presence_stm_tick,
			BAT_DRV_STM_EVENT_IRQ_PRESENCE_CHANGED);

	return IRQ_HANDLED;
}


static int bat_drv_get_batid_ohm(struct bat_drv_data *pbat)
{
	int adc_ret, adc_batid_ohms;
	struct iio_channel *chan;
	enum bat_drv_debug_event dgb_evt;

	if (!pbat)
		return -EINVAL;

	chan = pbat->batid_iio_chan;

	if (pbat->use_tbatid) {
		chan = pbat->tbatid_iio_chan;
		dgb_evt = BAT_DRV_DEBUG_TBATID_IN_OHMS;
	} else {
		chan = pbat->batid_iio_chan;
		dgb_evt = BAT_DRV_DEBUG_BATID_IN_OHMS;
	}

	adc_ret = iio_read_channel_processed(chan, &adc_batid_ohms);

	if (adc_ret != IIO_VAL_INT) {
		pr_err("failed reading battery ID from iio\n");
		return adc_ret;
	}

	BAT_DRV_DEBUG_PARAM(dgb_evt, adc_batid_ohms);

	return adc_batid_ohms;
}

static int pmic_set_bat_type(struct bat_drv_data *pbat,
				enum battery_type_id type_id)
{
	u32 batdetctrl0_reg = 0;

	WARN_ON(vmm_pmic_reg_read(BATTDETCTRL0_REG, &batdetctrl0_reg));
	reg_set_field(batdetctrl0_reg, BATTYP, type_id);
	WARN_ON(vmm_pmic_reg_write(BATTDETCTRL0_REG, batdetctrl0_reg));

	return 0;
}

static int bat_drv_hw_initialize(struct bat_drv_data *pbat)
{
	u32 batdetctrl0_reg = 0, batdetctrl1_reg = 0, chgrirq1_reg = 0, rmsrc;
	int ret;

	if (!pbat)
		return -EINVAL;

	rmsrc = pbat->use_tbatid ? NTC : BATTID;

	BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_HW_INIT);

	ret = pmic_reg_set_field(PSDETCTRL_REG,
		(BATTRMSRC_M << BATTRMSRC_O) | (BATTRMPDEN_M << BATTRMPDEN_O),
		(rmsrc << BATTRMSRC_O) | (ENABLE << BATTRMPDEN_O));
	if (ret)
		return ret;


	ret = vmm_pmic_reg_read(BATTDETCTRL0_REG, &batdetctrl0_reg);
	if (ret)
		return ret;

	reg_set_field(batdetctrl0_reg, BATTYP, BAT_TYPE_MIPI);
	reg_set_field(batdetctrl0_reg, BATTDBEN, ENABLE);

	ret = vmm_pmic_reg_write(BATTDETCTRL0_REG, batdetctrl0_reg);
	if (ret)
		return ret;


	ret = vmm_pmic_reg_read(BATTDETCTRL1_REG, &batdetctrl1_reg);
	if (ret)
		return ret;

	reg_set_field(batdetctrl1_reg, BRMDBC, BAT_RM_DBC_MAX);

	ret = vmm_pmic_reg_write(BATTDETCTRL1_REG, batdetctrl1_reg);
	if (ret)
		return ret;


	ret = pmic_reg_set_field(MCHGRIRQ1_REG,
		MBATTDET_M << MBATTDET_O, UNMASK << MBATTDET_O);
	if (ret)
		return ret;

	ret = pmic_reg_set_field(MIRQLVL1_REG,
			MCHGR_M << MCHGR_O, UNMASK << MCHGR_O);
	if (ret)
		return ret;

	/* clear any pending interrupt so far */
	reg_set_field(chgrirq1_reg, BATTDET, CLEAR);
	ret = vmm_pmic_reg_write(CHGRIRQ1_REG, chgrirq1_reg);
	if (ret)
		return ret;

	return 0;
}

static inline int bat_drv_get_pdata(struct bat_drv_data *hal)
{
	struct device_node *np = hal->pdev->dev.of_node;
	struct bat_drv_platform_data *pdata;

	if (IS_ENABLED(CONFIG_OF))
		return bprofile_parse_dt(&hal->pdata.supported_batteries,
				&hal->pdata.supported_batteries_len,
				&hal->pdata.bprofiles,
				&hal->pdata.bprofiles_len,
				np);


	pdata = (struct bat_drv_platform_data *)
			hal->pdev->dev.platform_data;

	if (!pdata || !pdata->supported_batteries)
		return -ENODATA;

	hal->pdata.supported_batteries = pdata->supported_batteries;
	hal->pdata.supported_batteries_len = pdata->supported_batteries_len;

	return 0;
}

static inline void bat_drv_release_pdata(struct bat_drv_data *hal)
{
	if (IS_ENABLED(CONFIG_OF)) {
		kfree(hal->pdata.supported_batteries);
		kfree(hal->pdata.bprofiles);
	}
}


/**
 * bat_drv_probe - Initialises the driver, when the device has been found.
 */
static int __init bat_drv_probe(struct platform_device *p_platform_dev)
{
	int batid_ohm = 0, ret = 0;
	struct bat_drv_data *pbat = &bat_drv_instance;

	struct iio_channel *batid_iio_chan;
	struct iio_channel *tbatid_iio_chan;

	BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_EVENT_PROBE);

	pbat->pdev = p_platform_dev;

	pbat->irq = platform_get_irq_byname(p_platform_dev,
					"PMIC_CHARGER_HIRQ");

	if (IS_ERR_VALUE(pbat->irq)) {
		pr_err("(%s) failed to get irq no\n", __func__);
		return -ENXIO;
	}

	ret = request_threaded_irq(pbat->irq, NULL, bat_presence_change_th,
		IRQF_SHARED | IRQF_ONESHOT,
		"pmic_battery_irq", pbat);

	if (ret) {
		pr_err("(%s) failed requesting interrupt\n", __func__);
		return -EINVAL;
	}

	ret = bat_drv_get_pdata(pbat);

	if (ret) {
		pr_err("obtaining platform data failed!\n");
		return ret;
	}

	spin_lock_init(&pbat->work.lock);

	/* Initialise work queue. Must be single thread to ensure
	serialisation. */
	INIT_WORK(&pbat->work.work,
			bat_drv_execute_function);

	wake_lock_init(&pbat->work.kfifo_wakelock,
			WAKE_LOCK_SUSPEND, "batdrv_kfifo_wakelock");

	/*
	* Create private, single-threaded workqueue instead of using one of
	* the system predefined workqueues to reduce latency
	*/
	pbat->work.p_work_queue =
		create_singlethread_workqueue(dev_name(&pbat->pdev->dev));

	BUG_ON(NULL == pbat->work.p_work_queue);
	INIT_KFIFO(pbat->work.fifo);

	batid_iio_chan = iio_channel_get(NULL, "BATID_SENSOR");

	if (IS_ERR(batid_iio_chan)) {
		pr_info("%s - batid iio_channel_get failed\n", __func__);
		ret = PTR_ERR(batid_iio_chan);
		goto iio_ch_get_failed;
	}

	tbatid_iio_chan = iio_channel_get(NULL, "TBATID_SENSOR");

	if (IS_ERR(tbatid_iio_chan)) {
		pr_info("%s - tbatid iio_channel_get failed\n", __func__);
		ret = PTR_ERR(tbatid_iio_chan);
		goto iio_ch_get_failed;
	}

	/* Check if the BATTID pin is floating or not. If it is then the
	BPTHERM0 pin will be used for both bat id detection and battery
	temperature, else use the BATID pin to determine the battery type. */
	ret = iio_read_channel_processed(batid_iio_chan, &batid_ohm);

	if (IIO_VAL_INT == ret) {
		pr_info("BATID is fitted\n");
		pbat->use_tbatid = false;
	} else if (-ERANGE == ret) {
		pr_info("BATID is not fitted, use NTC as BATID\n");
		pbat->use_tbatid = true;
	} else {
		pr_err("%s - fail to measure battery ID %d\n", __func__, ret);
		goto batid_meas_failed;
	}

	/* Disable the presence detect interrupt before enabling presence
	detection hardware. */
	bat_drv_batrm_det_irq_en(pbat, false);

	ret = bat_drv_hw_initialize(pbat);
	if (ret) {
		pr_err("%s hw_init failed\n", __func__);
		goto hw_init_failed;
	}

	pbat->batid_iio_chan = batid_iio_chan;
	pbat->tbatid_iio_chan = tbatid_iio_chan;

	pbat->initialised = true;

	/* Determine battery presence following interrupt */
	BAT_DRV_HAL_ENQUEUE(pbat, bat_drv_presence_stm_tick,
				BAT_DRV_STM_EVENT_DET_BAT_PRESENCE);

	pr_info("%s OK\n", __func__);

	return 0;

hw_init_failed:
batid_meas_failed:
iio_ch_get_failed:
	destroy_workqueue(pbat->work.p_work_queue);
	wake_lock_destroy(&pbat->work.kfifo_wakelock);
	bat_drv_release_pdata(pbat);
	free_irq(pbat->irq, pbat);

	return ret;
}

/**
 * bat_drv_remove - Release allocated resources.
 */
static int __exit bat_drv_remove(struct platform_device *p_platform_dev)
{
	struct bat_drv_data *pbat = &bat_drv_instance;

	BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_EVENT_REMOVE);

	/* Delete allocated resources and mark driver as uninitialised. */
	if (pbat->initialised) {
		pbat->initialised = false;

		iio_channel_release(pbat->batid_iio_chan);
		iio_channel_release(pbat->tbatid_iio_chan);
		bat_drv_batrm_det_irq_en(pbat, false);
		destroy_workqueue(pbat->work.p_work_queue);
		wake_lock_destroy(&pbat->work.kfifo_wakelock);
		bat_drv_release_pdata(pbat);
		/* Deregister interrupt handler */
		free_irq(pbat->irq, pbat);
	}
	return 0;
}

/**
 * bat_drv_suspend() - Called when the system is attempting to suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int bat_drv_suspend(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Nothing to do here except logging */
	BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_EVENT_SUSPEND);
	return 0;
}

/**
 * bat_drv_resume() - Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int bat_drv_resume(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Nothing to do here */
	BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_EVENT_RESUME);
	return 0;
}

const struct dev_pm_ops bat_drv_pm = {
	.suspend = bat_drv_suspend,
	.resume = bat_drv_resume,
};


static const struct of_device_id bat_id_of_match[] = {
	{
		.compatible = "intel,pmic_bat",
	},
	{}
};

MODULE_DEVICE_TABLE(of, bat_id_of_match);

static struct platform_driver battery_driver = {
	.probe          = bat_drv_probe,
	.remove         = bat_drv_remove,
	.driver = {
		.name   = DRIVER_NAME,
		.owner  = THIS_MODULE,
		.of_match_table = of_match_ptr(bat_id_of_match),
		.pm = &bat_drv_pm,
	},
};

static int __init bat_drv_init(void)
{
	int ret;

	/* NOTE: Must initialise debug data spinlock before any logging. */
	spin_lock_init(&bat_drv_debug_info.lock);
	BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_EVENT_INIT);

	ret = platform_driver_register(&battery_driver);

	return ret;
}

static void __exit bat_drv_exit(void)
{
	BAT_DRV_DEBUG_NO_PARAM(BAT_DRV_DEBUG_EVENT_EXIT);
	return platform_driver_unregister(&battery_driver);
}

device_initcall_sync(bat_drv_init);
module_exit(bat_drv_exit);

MODULE_DESCRIPTION("SofiaLTE Battery Driver");
MODULE_LICENSE("GPL v2");
