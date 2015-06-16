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
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 */

#define DRIVER_NAME					"ag6x0_bat_hal"
#define pr_fmt(fmt) DRIVER_NAME": "fmt

#include <linux/device.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/idi/idi_interface.h>
#include <linux/idi/idi_ids.h>
#include <linux/idi/idi_device_pm.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/power/battery_id.h>
#include <linux/wakelock.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include "bprofile_dts_parser.h"

#define PMU_AG6X0_BAT_DET	0xC00
#define PMU_AG6X0_C0_FS		0x1038

/* BIT position in C0_FS register for BatRemovalDet (read only) */
#define PMU_AG6X0_C0_FS_BIT				(0x4)
/* Mask for C0_FS register (read only) */
#define PMU_AG6X0_C0_FS_MASK \
		(1 << PMU_AG6X0_C0_FS_BIT)

/* BIT position for BATRM_DET BREG register field */
#define PMU_AG6X0_BATRM_DET_BREG_BIT			(0x8)
/* BIT position for BATRM_DET EBRM register field */
#define PMU_AG6X0_BATRM_DET_EBRM_BIT			(0x9)
/* Mask for BATRM_DET BREG register */
#define PMU_AG6X0_BATRM_DET_BREG_SET \
		(1 << PMU_AG6X0_BATRM_DET_BREG_BIT)

/* Mask for BATRM_DET EBRM register */
#define PMU_AG6X0_BATRM_DET_EBRM_SET \
		(1 << PMU_AG6X0_BATRM_DET_EBRM_BIT)

/* Mask for BATRM_DET DBLN register (default value) */
#define PMU_AG6X0_BATRM_DET_DBLN_VAL			(0x03)

/**
 * Debounce time for battery presence state input that detects
 * battery insertion/removal (ms)
 */
#define BATTERY_CONNECTOR_DEBOUNCE_TIME_MS		(10)
/**
 * Debounce count: number of battery presence state reads with same
 * state before determining GPIO is stable
 */
#define BATTERY_CONNECTOR_DEBOUNCE_STABLE_COUNT		(3)
/**
 * Debounce count: number of consecutive battery presence state
 * reads before it settles. Used to detect a defective contact
 */
#define BATTERY_CONNECTOR_DEBOUNCE_UNSTABLE_COUNT	(20)

/**  Size of stm state change debug data array (has to be power of 2!!!) */
#define STM_STATE_CHANGE_DEBUG_DATA_SIZE		(1<<4)

/* Size of debug data array (has to be power of 2!!!) */
#define BAT_DRV_HAL_DEBUG_DATA_SIZE			(1<<6)

/* Size of array of function payloads to implement work FIFO. */
#define BAT_DRV_HAL_WORK_FIFO_LENGTH			(32)

/**  Macro for getting state machine input status */
#define BAT_DRV_HAL_PRESENCE_STM_GET_STATUS(status_bit) \
	((bat_drv_hal_instance.presence_stm.inputs) & (1UL << (status_bit)))
#define BAT_DRV_HAL_PRESENCE_STM_SET_INPUT(status_bit) \
	((bat_drv_hal_instance.presence_stm.inputs) |= (1UL << (status_bit)))
#define BAT_DRV_HAL_PRESENCE_STM_CLEAR_INPUT(status_bit) \
	((bat_drv_hal_instance.presence_stm.inputs) &= ~(1UL << (status_bit)))

/* Macro to trace and log debug data internally. Jiffy resolution is adequate
for Bat Drv HAL */
#define BAT_DRV_HAL_DEBUG(_array, _event, _param) \
{ \
	unsigned long flags; \
	spin_lock_irqsave(&_array.lock, flags); \
	_array.log_array[_array.index].time_stamp = jiffies; \
	_array.log_array[_array.index].event = (_event); \
	_array.log_array[_array.index].param = (long)(_param); \
	_array.index++; \
	_array.index &= (BAT_DRV_HAL_DEBUG_DATA_SIZE-1); \
	spin_unlock_irqrestore(&_array.lock, flags); \
	pr_debug("%s 0x%lx  dec=%ld\n", #_event, \
			(unsigned long)_param, (long)_param); \
}

/* Macro to trace and log debug event and data. */
#define BAT_DRV_HAL_DEBUG_PARAM(_event, _param) \
		BAT_DRV_HAL_DEBUG(bat_drv_hal_debug_info, _event, _param)

/* Macro to trace and log debug event without a parameter. */
#define BAT_DRV_HAL_DEBUG_NO_PARAM(_event) \
		BAT_DRV_HAL_DEBUG(bat_drv_hal_debug_info, _event, 0)

#define BAT_DRV_HAL_ENQUEUE(p_func, param) \
	bat_drv_hal_enqueue_function((fp_scheduled_function)(p_func), \
								(long)(param))

#define bat_hal_set_pm_state(_idi_dev, _pm_state, _en) \
do {\
	int __ret;\
	__ret = idi_set_power_state(_idi_dev, _pm_state, _en);\
\
	if (__ret) {\
		pr_err("%s: setting PM state '%s' failed!\n",\
			__FILE__, _pm_state->name);\
		if (_en)\
			BUG();\
	} \
} while (0)

/* Function type for scheduled execution by work. */
typedef void (*fp_scheduled_function) (long param);

/* Message payload for work scheduler queue. */
struct bat_drv_hal_fifo_payload {
	fp_scheduled_function p_func;
	long param;
};

/*
 * Message queue for scheduled work.
 * This is a ring buffer protected by a spinlock,
 * as access is required from interrupt and timer callback context.
 */
struct bat_drv_hal_work_fifo {
	DECLARE_KFIFO(fifo, struct bat_drv_hal_fifo_payload,
		BAT_DRV_HAL_WORK_FIFO_LENGTH);
	spinlock_t lock;

	struct wake_lock kfifo_wakelock;

	struct workqueue_struct *p_work_queue;
	struct work_struct work;
	bool removal_pending;
};

/** Events for use in debug and tracing. */
enum bat_drv_hal_debug_event {
	BAT_DRV_HAL_DEBUG_EVENT_INIT,
	BAT_DRV_HAL_DEBUG_EVENT_EXIT,
	BAT_DRV_HAL_DEBUG_EVENT_PROBE,
	BAT_DRV_HAL_DEBUG_EVENT_REMOVE,
	BAT_DRV_HAL_DEBUG_EVENT_SUSPEND,
	BAT_DRV_HAL_DEBUG_EVENT_RESUME,

	BAT_DRV_HAL_DEBUG_ENQUEUE_FUNCTION,
	BAT_DRV_HAL_DEBUG_EXECUTE_FUNCTION,
	BAT_DRV_HAL_DEBUG_ENQUEUE_PARAM,
	BAT_DRV_HAL_DEBUG_EXECUTE_PARAM,

	BAT_DRV_HAL_DEBUG_GET_PRESENCE_STATE,
	BAT_DRV_HAL_DEBUG_GET_PRESENCE_STATE_BRD_PENDING,
	BAT_DRV_HAL_DEBUG_GET_PRESENCE_STATE_BREG_STATE,
	BAT_DRV_HAL_DEBUG_ENABLE_PRESENCE_DETECT,
	BAT_DRV_HAL_DEBUG_CHECK_STATUS_UPDATE,
	BAT_DRV_HAL_DEBUG_DETECTION_DEBOUNCE_START,
	BAT_DRV_HAL_DEBUG_DETECTION_DEBOUNCING_WORK,

	BAT_DRV_HAL_DEBUG_PRESENCE_STM_DO_INPUTS,
	BAT_DRV_HAL_DEBUG_PRESENCE_STM_DO_TRANSITION,
	BAT_DRV_HAL_DEBUG_PRESENCE_STM_DO_OUTPUTS,

	BAT_DRV_HAL_DEBUG_EVENT_REGISTER_HAL,
	BAT_DRV_HAL_DEBUG_EVENT_BAT_NOTIFICATION,
	BAT_DRV_HAL_DEBUG_EVENT_EXTERNAL_SUPPLY_CHANGED,

	BAT_DRV_HAL_DEBUG_EVENT_BAT_NOT_FITTED,
	BAT_DRV_HAL_DEBUG_EVENT_BAT_ID_TYPE,
	BAT_DRV_HAL_DEBUG_EVENT_BAT_REPORT_REPEATED,
	BAT_DRV_HAL_DEBUG_EVENT_BAT_CHANGED,
};

/**
 * struct bat_drv_hal_debug_data - Structure to collect debug data
 * @lock		Spinlock for atomic access
 * @index		Index of logging array
 * @log_array		Debug data logging array
 *	@time_stamp	System Time Stamp in Jiffies
 *	@event		Event which occurred
 *	@param		General purpose parameter
 */
struct bat_drv_hal_debug_data {
	spinlock_t lock;
	u32 index;
	struct {
		u32 time_stamp;
		enum bat_drv_hal_debug_event event;
		long param;
	} log_array[BAT_DRV_HAL_DEBUG_DATA_SIZE];
};

/**  Presence state machine events */
enum bat_drv_hal_presence_stm_event {
	BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_PRESENCE_ATTAIN_REQUEST,
	BAT_DRV_HAL_PRESENCE_STM_EVENT_PMU_CB_PRESENCE_CHANGED,
	BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_PRESENT,
	BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_NOT_PRESENT,
	BAT_DRV_HAL_PRESENCE_STM_EVENT_PULLUP_FIRST_DISABLE_REQUEST,
	BAT_DRV_HAL_PRESENCE_STM_EVENT_LAST_PULLUP_ENABLE_ALLOW,
	BAT_DRV_HAL_PRESENCE_STM_EVENT_BATID_READ_DONE,
	BAT_DRV_HAL_PRESENCE_STM_EVENT_START,
};

/*
 * Presence state machine states
 *
 * BAT_DRV_HAL_PRESENCE_STM_STATE_PRE_INIT
 *				Preinitialised state.
 * BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAIN
 *				Determining whether battery id fitted or not in
 *				a debouncing sequence
 *
 * BAT_DRV_HAL_PRESENCE_STM_STATE_READ_ONGOING
 *				Determining whether battery id fitted or not in
 *				a debouncing sequence
 *
 * BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_READ_ONGOING
 *				Waiting for pullup to be reenabled, after an
 *				battery id measurement has been taken
 *
 * BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_CONFIRM
 *				Confirming that the batteryi s still present
 *				after having read the battery id during which
 *				time the pullup would have been disabled,
 *				therefore creating a blackout period when a
 *				battery removal would not have generated an
 *				interrupt
 *
 * BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_NO_READ_ONGOING	Waiting for the pullup
 *				to be reenabled after a request to disable the
 *				pullup that was not coincident with a read of
 *				the battery id resistor invoked by this
 *				statemachine, e.g. a read invoked externally for
 *				calibrating the battery id sensor
 *
 * BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAINED Battery presence state
 *				determined whether it be fitted (with or without
 *				known id value) or removed.
 */
enum bat_drv_hal_presence_stm_state {
	BAT_DRV_HAL_PRESENCE_STM_STATE_PRE_INIT,
	BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAIN,
	BAT_DRV_HAL_PRESENCE_STM_STATE_READ_ONGOING,
	BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_READ_ONGOING,
	BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_CONFIRM,
	BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_NO_READ_ONGOING,
	BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAINED,
};

/* Presence state machine data */
struct bat_drv_hal_presence_stm {
	u32 inputs;
	enum bat_drv_hal_presence_stm_state current_state;
	bool client_initial_update_done;
};

/* Presence state machine debug data */
struct bat_drv_hal_presence_stm_debug {
	u32 index;
	u32 stm_state_change_seq_num;

	struct {
		u32 timestamp;
		u32 stm_state_change_seq_num;
		enum bat_drv_hal_presence_stm_event event;
		enum bat_drv_hal_presence_stm_state new_state;
	} data[STM_STATE_CHANGE_DEBUG_DATA_SIZE];
};

/**
 * struct debounce_data - Data for battery contact debouncing.
 *
 * @brd_interrupt_pending		RD interrupt pending report from PMU
					driver
 * @last_bat_presence_state		last detected battery presence state
 * @presence_stable_counter		number of consecutive battery presence
 *					state reads found with same value. Used
 *					for debouncing
 * @presence_unstable_counter		number of consecutive battery presence
 *					state reads with unstable value.
 *					Used to detect a defective battery
 *					contact
 * @currently_checking_debouncing	Indicates if currently checking a
 *					battery presence pin debounce
 * @timer				Timer to process debouncing
.* @lock				Guards access to
 *						currently_checking_debouncing
 */
struct debounce_data {
	bool brd_interrupt_pending;
	bool last_bat_presence_state;
	u32 presence_stable_counter;
	u32 presence_unstable_counter;
	bool currently_checking_debouncing;
	struct timer_list timer;
	spinlock_t lock;
};

struct bat_drv_hal_platform_data {
	struct battery_type *supported_batteries;
	size_t supported_batteries_len;

	struct ps_pse_mod_prof *bprofiles;
	size_t bprofiles_len;
};


/**
 * struct bat_drv_hal_data - Battery Driver Hal control structure
 * @initialised		Driver initialisation state
 * @p_idi_device	Pointer to platform device
 * @battery_id		Battery ID data
 */
struct bat_drv_hal_data {
	bool initialised;
	struct idi_peripheral_device *p_idi_device;

	struct device_state_pm_state *pm_state_en;
	struct device_state_pm_state *pm_state_dis;

	bool last_reported_presence_state;
	struct bat_drv_hal_presence_stm presence_stm;
	struct bat_drv_hal_presence_stm_debug presence_stm_debug;
	struct debounce_data debouncing;
	struct bat_drv_hal_work_fifo work;
	struct resource *pmu_res;
	bool breg_state;
	int irq;

	struct bat_drv_hal_platform_data pdata;
};

/* Bat Driver Hal instance */
static struct bat_drv_hal_data bat_drv_hal_instance = {
	.presence_stm = {
		.current_state =
			BAT_DRV_HAL_PRESENCE_STM_STATE_PRE_INIT,
	},
	.initialised = false,
};

/* Array to collect debug data */
static struct bat_drv_hal_debug_data bat_drv_hal_debug_info;

static unsigned bat_drv_pmu_ioread(struct bat_drv_hal_data *bat_drv_hal,
					unsigned offset)
{
	int ret = 0;
	unsigned reg;
	unsigned addr;

	if (unlikely(bat_drv_hal == NULL))
		BUG();

	addr = bat_drv_hal->pmu_res->start + offset;

	ret = idi_client_ioread(bat_drv_hal->p_idi_device, addr, &reg);
	if (ret)
		BUG();

	return reg;
}

static void bat_drv_pmu_iowrite(struct bat_drv_hal_data *bat_drv_hal,
					unsigned offset,
					unsigned data)
{
	int ret = 0;
	unsigned addr;

	if (unlikely(bat_drv_hal == NULL))
		BUG();

	addr = bat_drv_hal->pmu_res->start + offset;

	ret = idi_client_iowrite(bat_drv_hal->p_idi_device, addr, data);
	if (ret)
		BUG();
}


/* Forwward declaration */
static irqreturn_t bat_drv_hal_presence_change_cb(int irq, void *dev);


static struct battery_type *get_bat_type(struct bat_drv_hal_data *pbat,
					unsigned int batid_ohms)
{
	unsigned int ibatid_min_th, ibatid_max_th;
	int i, len = pbat->pdata.supported_batteries_len;

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
 * bat_drv_hal_enqueue_function - Adds a function to the message queue for the
 * serialisation work. This function is supplied to the HAL to allow processing
 * in a single thread work for static data. Since it may be called from
 * interrupt or timer callback context, the locking mechanism is spinlock.
 *
 * @p_function		[in] Function to be added to the execution FIFO.
 * @param		[in] Parameter value for the function.
 */
static void bat_drv_hal_enqueue_function(fp_scheduled_function p_function,
					long param)
{
	unsigned long flags;

	/* Functions may not be scheduled while the device is being removed. */
	if (!bat_drv_hal_instance.work.removal_pending) {
		struct bat_drv_hal_fifo_payload payload = {
			.p_func = p_function,
			.param = param
		};

		spin_lock_irqsave(&bat_drv_hal_instance.work.lock, flags);

		/*
		* Message queue is a critical region.
		* kfifo() needs explicit locking when there
		* are multiple consumers or producers.
		*/
		BUG_ON(0 == kfifo_in(&bat_drv_hal_instance.work.fifo,
				&payload, 1));

		BAT_DRV_HAL_DEBUG_PARAM(BAT_DRV_HAL_DEBUG_ENQUEUE_FUNCTION,
					p_function);
		BAT_DRV_HAL_DEBUG_PARAM(BAT_DRV_HAL_DEBUG_ENQUEUE_PARAM, param);

		wake_lock(&bat_drv_hal_instance.work.kfifo_wakelock);

		/* Schedule the work queue to process the message. */
		(void)queue_work(bat_drv_hal_instance.work.p_work_queue,
				&bat_drv_hal_instance.work.work);

		spin_unlock_irqrestore(&bat_drv_hal_instance.work.lock, flags);
	}
}

/**
 * bat_drv_hal_execute_function - Remove from queue and execute all scheduled
 * functions in a work.
 *
 * @p_function		[in] Function to be added to the execution FIFO.
 * @param		[in] Parameter value for the function.
 */
static void bat_drv_hal_execute_function(struct work_struct *work)
{
	struct bat_drv_hal_fifo_payload payload;
	unsigned long flags;

	/* Repeatedly fetch one entry from the fifo and process it until the
	fifo is empty. */
	while (0 != kfifo_out_spinlocked(&bat_drv_hal_instance.work.fifo,
					&payload,
					1, &bat_drv_hal_instance.work.lock)) {

		BAT_DRV_HAL_DEBUG_PARAM(BAT_DRV_HAL_DEBUG_EXECUTE_FUNCTION,
					payload.p_func);
		BAT_DRV_HAL_DEBUG_PARAM(BAT_DRV_HAL_DEBUG_EXECUTE_PARAM,
					payload.param);
		/* Execute the function. */
		payload.p_func(payload.param);

		/* wakelock acquisition must be in sync with kfifo operation */
		spin_lock_irqsave(&bat_drv_hal_instance.work.lock, flags);

		/* If work fifo is empty the wakelock can be released */
		if (kfifo_is_empty(&bat_drv_hal_instance.work.fifo))
			wake_unlock(&bat_drv_hal_instance.work.kfifo_wakelock);

		spin_unlock_irqrestore(&bat_drv_hal_instance.work.lock, flags);
	}
}

/**
 * bat_drv_hal_get_presence_state - Gets the battery presence state
 *
 * @return  Battery presence state (true if present, else false)
 */
static bool bat_drv_hal_get_presence_state(void)
{
	bool presence;
	u32 c0_fs, batrm_det;
	struct bat_drv_hal_data *hal_data = &bat_drv_hal_instance;

	/* Read C0_FS and BATRM_DET registers */
	c0_fs = bat_drv_pmu_ioread(hal_data, PMU_AG6X0_C0_FS);
	batrm_det = bat_drv_pmu_ioread(hal_data, PMU_AG6X0_BAT_DET);

	/*
	* Read from the C0_FS Register (floating status) and not the
	* C0_IRQSS_Register (interrupt pending status), even though the type
	* returned is named pending/not pending.
	* Floating status is required for debouncing.
	*/

	bat_hal_set_pm_state(bat_drv_hal_instance.p_idi_device,
			bat_drv_hal_instance.pm_state_en, true);

	bat_drv_hal_instance.debouncing.brd_interrupt_pending
			= (c0_fs & PMU_AG6X0_C0_FS_MASK);
	BAT_DRV_HAL_DEBUG_PARAM
		(BAT_DRV_HAL_DEBUG_GET_PRESENCE_STATE_BRD_PENDING,
			bat_drv_hal_instance.debouncing.brd_interrupt_pending);

	bat_drv_hal_instance.breg_state
			= (batrm_det & PMU_AG6X0_BATRM_DET_BREG_SET);
	BAT_DRV_HAL_DEBUG_PARAM
		(BAT_DRV_HAL_DEBUG_GET_PRESENCE_STATE_BREG_STATE,
			bat_drv_hal_instance.breg_state);

	bat_hal_set_pm_state(bat_drv_hal_instance.p_idi_device,
			bat_drv_hal_instance.pm_state_dis, false);

	/*
	* The BREG bit changes the meaning of the polarity of the BRD bit.
	* The following table describes how to determine the battery presence:
	*
	* BREG == 0 == Rising Edge
	* BRD 0 == PMU_IRQ_NOT_PENDING == battery present
	* BRD 1 == PMU_IRQ_PENDING == battery not present

	* BREG == 1 == Falling edge
	* BRD 0 == PMU_IRQ_NOT_PENDING == battery not present
	* BRD 1 == PMU_IRQ_PENDING == battery present
	*
	*/
	presence = (bat_drv_hal_instance.breg_state ==
		bat_drv_hal_instance.debouncing.brd_interrupt_pending);

	BAT_DRV_HAL_DEBUG_PARAM(BAT_DRV_HAL_DEBUG_GET_PRESENCE_STATE, presence);

	return presence;
}

/**
 * bat_drv_hal_batrm_det_enable - Enable or disable the battery removal
 * detection IRQ If the state has not changed since the last call,
 * no propgagation to the HW is made.
 *
 * @new_enabled_state	[in]
 */
void bat_drv_hal_batrm_det_enable(bool new_enabled_state)
{
	/* The interrupt is disabled by default at boot. */
	static bool batrm_det_irq_enabled;

	if (new_enabled_state != batrm_det_irq_enabled) {
		if (new_enabled_state) {
			BUG_ON(0 != request_irq(bat_drv_hal_instance.irq,
						bat_drv_hal_presence_change_cb,
						0, DRIVER_NAME, NULL));
		} else {
			free_irq(bat_drv_hal_instance.irq, NULL);
		}

		batrm_det_irq_enabled = new_enabled_state;
	}
}

/**
 * bat_drv_hal_enable_presence_detect_interrupt - Enables the presence detect
 * interrupt. The PMU battery removal/insertion detect interrupt is edge based.
 *
 * @bat_present		[in] bat_present Indicates if battery is determined as
 *			present or not
 */
static void bat_drv_hal_enable_presence_detect_interrupt(bool bat_present)
{
	/* Re-enable checking of battery presence changes */
	bat_drv_hal_instance.debouncing.currently_checking_debouncing = false;

	BAT_DRV_HAL_DEBUG_PARAM(BAT_DRV_HAL_DEBUG_ENABLE_PRESENCE_DETECT,
				bat_present);

	/*
	* Set BREG to detect next battery presence state change:
	* BREG = 0 = Rising edge, detect removal.
	* BREG = 1 = Falling edge, detect insertion.
	*/

	bat_hal_set_pm_state(bat_drv_hal_instance.p_idi_device,
			bat_drv_hal_instance.pm_state_en, true);

	{
		u32 batrm_det = bat_drv_pmu_ioread(&bat_drv_hal_instance,
				PMU_AG6X0_BAT_DET);
		batrm_det &= ~PMU_AG6X0_BATRM_DET_BREG_SET;
		batrm_det |= (false == bat_present) ?
				PMU_AG6X0_BATRM_DET_BREG_SET : 0;

		bat_drv_pmu_iowrite(&bat_drv_hal_instance,
					PMU_AG6X0_BAT_DET, batrm_det);
	}

	bat_hal_set_pm_state(bat_drv_hal_instance.p_idi_device,
			bat_drv_hal_instance.pm_state_dis, false);

	/* Enable interrupt handler */
	bat_drv_hal_batrm_det_enable(true);

	/* Due to edge based triggering the state of the pin must be checked
	after enabling the trigger in case it has changed state since the last
	debounce check read. */
	if (bat_present != bat_drv_hal_get_presence_state()) {
		/*
		* The state of the pin has changed since the last state check.
		* Act now as if a new presence change event has occurred
		*/
		(void)bat_drv_hal_presence_change_cb(0, NULL);
	}
}

/**
 * bat_drv_hal_check_battery_status_update - Update subscribers with battery
 * info if needed.
 *
 * @new_presence_state	[in] TRUE if battery is fitted, else FALSE.
 */
static void bat_drv_hal_check_battery_status_update(bool new_presence_state)
{
	/* Determine whether battery presence has changed */
	struct battery_type *bat_type = NULL;
	const bool presence_changed =
	(bat_drv_hal_instance.last_reported_presence_state !=
	new_presence_state);
	/*define a static variable to make battery status
	update once only when starting up*/
	static int bat_status = 0;
	BAT_DRV_HAL_DEBUG_PARAM(BAT_DRV_HAL_DEBUG_CHECK_STATUS_UPDATE,
				new_presence_state);
	if(bat_status == 1)
		return;
	if (new_presence_state) {
		/* Send an update even if the last detected presence state was
		fitted.
		This could occur if:
		1) a battery was removed and a different one was refitted whilst
			the pullup was disabled or
		2) when the battery was temporarily disconnected, for example if
			the phone was dropped. */
		/* Signal the presence of the default battery. */
		pr_info("battery is fitted\n");
		bat_type = get_bat_type(&bat_drv_hal_instance, 0);
		battery_prop_changed(POWER_SUPPLY_BATTERY_INSERTED,
				&bat_type->profile);
	} else {
		/* The battery is not fitted. Send update only if the presence
		state has changed or this is the first notification report */
		pr_err("no battery, PSU is worked\n");
		if (presence_changed || !bat_drv_hal_instance.presence_stm.
						client_initial_update_done) {

			battery_prop_changed(
				POWER_SUPPLY_BATTERY_REMOVED, NULL);
		}
	}
	bat_drv_hal_instance.presence_stm.client_initial_update_done = true;
	bat_drv_hal_instance.last_reported_presence_state = new_presence_state;
	bat_status = 1;
}

/**
 * bat_drv_hal_presence_detection_debouncing_start - Start off battery presence
 * detection process. Starts a debouncing timer, waiting for any possible
 * mechanical switch oscillations to cease before determining whether battery is
 * fitted or not.
 */
static void bat_drv_hal_presence_detection_debouncing_start(void)
{
	BAT_DRV_HAL_DEBUG_NO_PARAM(BAT_DRV_HAL_DEBUG_DETECTION_DEBOUNCE_START);

	/* Read initial presence state for debouncing sequence */
	bat_drv_hal_instance.debouncing.last_bat_presence_state =
					bat_drv_hal_get_presence_state();

	/* Reset contact stability counters */
	bat_drv_hal_instance.debouncing.presence_stable_counter = 0;
	bat_drv_hal_instance.debouncing.presence_unstable_counter = 0;

	(void)mod_timer(&bat_drv_hal_instance.debouncing.timer, jiffies +
			msecs_to_jiffies(BATTERY_CONNECTOR_DEBOUNCE_TIME_MS));
}

/**
 * bat_drv_hal_presence_stm_do_inputs - Process state machine inputs
 *
 * @event	[in] State machine event
 */
static void bat_drv_hal_presence_stm_do_inputs(enum
					bat_drv_hal_presence_stm_event
					event)
{
	BAT_DRV_HAL_DEBUG_PARAM(BAT_DRV_HAL_DEBUG_PRESENCE_STM_DO_INPUTS,
				event);

	/* Extract status info from events */
	switch (event) {
	case BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_PRESENCE_ATTAIN_REQUEST:
	case BAT_DRV_HAL_PRESENCE_STM_EVENT_BATID_READ_DONE:
		/* Do nothing */
		/* This is fine as some events do not change input status */
		break;

	case BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_PRESENT:
		/* Record this event, because a client update may not happen
		immediately if hal not started */
		BAT_DRV_HAL_PRESENCE_STM_SET_INPUT(
			BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_PRESENT);
		break;

	case BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_NOT_PRESENT:
		BAT_DRV_HAL_PRESENCE_STM_CLEAR_INPUT(
			BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_PRESENT);
		break;

	case BAT_DRV_HAL_PRESENCE_STM_EVENT_PULLUP_FIRST_DISABLE_REQUEST:
		/* Record this event, because transition on id read done event
		will depend on if pullup is still disabled */
		BAT_DRV_HAL_PRESENCE_STM_SET_INPUT(
		 BAT_DRV_HAL_PRESENCE_STM_EVENT_PULLUP_FIRST_DISABLE_REQUEST);
		break;

	case BAT_DRV_HAL_PRESENCE_STM_EVENT_LAST_PULLUP_ENABLE_ALLOW:
		/* Record this event, because transition on id read done event
		will depend on if pullup is still disabled */
		BAT_DRV_HAL_PRESENCE_STM_CLEAR_INPUT
		 (BAT_DRV_HAL_PRESENCE_STM_EVENT_PULLUP_FIRST_DISABLE_REQUEST);
		break;

	case BAT_DRV_HAL_PRESENCE_STM_EVENT_START:
		/* Record asynchronous start event */
		BAT_DRV_HAL_PRESENCE_STM_SET_INPUT(
				BAT_DRV_HAL_PRESENCE_STM_EVENT_START);
		break;

	case BAT_DRV_HAL_PRESENCE_STM_EVENT_PMU_CB_PRESENCE_CHANGED:
		/* Record asynchronous pmu callback event */
		BAT_DRV_HAL_PRESENCE_STM_SET_INPUT(
		 BAT_DRV_HAL_PRESENCE_STM_EVENT_PMU_CB_PRESENCE_CHANGED);
		break;

	default:
		/* Unknown event - STM is corrupt. */
		BUG();
		break;
	}
}

/**
 * bat_drv_hal_presence_stm_do_state_transition - Make a state machine state
 * change if required
 *
 * @current_state	[in] Current state machine state
 * @event		[in] State machine event
 * @return		new state machine state (could be the same as old state)
 */
static enum bat_drv_hal_presence_stm_state
bat_drv_hal_presence_stm_do_state_transition(enum bat_drv_hal_presence_stm_state
					current_state,
					 enum bat_drv_hal_presence_stm_event
					  event)
{
	/* Default is not to change state */
	enum bat_drv_hal_presence_stm_state new_state_transition =
	current_state;

	BAT_DRV_HAL_DEBUG_PARAM(BAT_DRV_HAL_DEBUG_PRESENCE_STM_DO_TRANSITION,
				current_state);

	switch (current_state) {
	case BAT_DRV_HAL_PRESENCE_STM_STATE_PRE_INIT:
		if (BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_PRESENCE_ATTAIN_REQUEST
		== event) {
			/* Determine presence state of battery at boot */
			new_state_transition =
			BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAIN;
		} else
		if (BAT_DRV_HAL_PRESENCE_STM_EVENT_PULLUP_FIRST_DISABLE_REQUEST
		== event) {
			/* Driver clients should only be initialised after
			drivers in the boot sequence */
			BUG();
		}
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAIN:
		/* If the battery is fitted */
		if (BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_PRESENT ==
		event) {
			/* If the id bat sensor driver is available read
			the battery id */
			/* Battery state attainment is complete */
			new_state_transition =
			 BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAINED;
		} else
		if
		(BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_NOT_PRESENT
		== event) {
			/*
			* Battery is not fitted
			* Battery state attainment is complete
			*/
			new_state_transition =
			 BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAINED;
		} else
		if
		(BAT_DRV_HAL_PRESENCE_STM_EVENT_PULLUP_FIRST_DISABLE_REQUEST
		== event) {
			/* If there was an external request read the battery id,
			wait until the pullup can be enabled again before
			trying to determine if a battery is fitted */
			new_state_transition =
			 BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_NO_READ_ONGOING;
		}
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_READ_ONGOING:
		if (BAT_DRV_HAL_PRESENCE_STM_EVENT_BATID_READ_DONE == event) {

			/* If read is done, but pullup still disabled then wait
			until it is enabled before performing a check to ensure
			that the battery is still present */
			if (BAT_DRV_HAL_PRESENCE_STM_GET_STATUS(
		BAT_DRV_HAL_PRESENCE_STM_EVENT_PULLUP_FIRST_DISABLE_REQUEST)) {
				new_state_transition =
			BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_READ_ONGOING;
			} else {

				/* Check to confirm that the battery wasn't
				removed whilst the pullup was disabled during
				the id read */
				new_state_transition =
			 BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_CONFIRM;
			}
		}
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_READ_ONGOING:
		if (BAT_DRV_HAL_PRESENCE_STM_EVENT_LAST_PULLUP_ENABLE_ALLOW ==
		event) {
			/* The pullup has now been reenabled, so now check if
			the battery is still present */
			new_state_transition =
			 BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_CONFIRM;
		}
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_CONFIRM:
		if ((BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_PRESENT ==
		 event) ||
		  (BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_NOT_PRESENT
		   == event)) {

			/* If either the battery is still present, or it has
			been removed report it to the notifier as appropriate */
			new_state_transition =
			BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAINED;
		} else
		if
		(BAT_DRV_HAL_PRESENCE_STM_EVENT_PULLUP_FIRST_DISABLE_REQUEST
		== event) {

			/* Wait until the pullup can be enabled again before
			trying to determine if a battery is fitted */
			new_state_transition =
			 BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_NO_READ_ONGOING;
		}
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_NO_READ_ONGOING:
		if (BAT_DRV_HAL_PRESENCE_STM_EVENT_LAST_PULLUP_ENABLE_ALLOW ==
		 event) {
			/* Check if the battery presence state changed whilst
			the pullup was disabled */
			new_state_transition =
			 BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAIN;
		}
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAINED:
		/* If there was a battery presence change callback */
		if ((BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_PRESENCE_ATTAIN_REQUEST
		 == event) ||
		  (BAT_DRV_HAL_PRESENCE_STM_EVENT_PMU_CB_PRESENCE_CHANGED
		   == event)) {

			/* Determine whether the battery was inserted or
			removed, or if it was just a temporary false removal
			due e.g. the phone being dropped */
			new_state_transition =
			 BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAIN;
		}
		/* If there was an external request to disable the pullup */
		else if
		 (BAT_DRV_HAL_PRESENCE_STM_EVENT_PULLUP_FIRST_DISABLE_REQUEST
		  == event) {
			/* Wait until the pullup can be enabled again */
			new_state_transition =
			 BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_NO_READ_ONGOING;
		}
		break;

	default:
		/* Unknown state - STM is corrupt. */
		BUG();
		break;
	}
	return new_state_transition;
}

/**
 * bat_drv_hal_presence_stm_do_outputs - Handle state machine outputs and
 * transition to new state.
 * @new_state		[in] State machine state that will be transitioned to
 */
static void bat_drv_hal_presence_stm_do_outputs(
					enum bat_drv_hal_presence_stm_state
								new_state)
{
	BAT_DRV_HAL_DEBUG_PARAM(BAT_DRV_HAL_DEBUG_PRESENCE_STM_DO_OUTPUTS,
				new_state);

	switch (new_state) {
	case BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAIN:
		/*
		* Disable the presence detect interrupt whilst
		* the presence state is being debounced
		*/
		bat_drv_hal_batrm_det_enable(false);
		/*
		* Detect whether the battery presence has really changed
		* or if there was just a momentary disconnection.
		* Software debouncing needs to be done, because hardware
		* debouncing in the PMU is not long enough.
		*/
		bat_drv_hal_presence_detection_debouncing_start();
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_READ_ONGOING:
		/*
		 * Start measurement of battery id - sensor driver will
		 * request pullup disable if needed
		 */
		BUG();
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_READ_ONGOING:
		/* Nothing to do - wait until pullup reenabled */
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_CONFIRM:
		/*
		* Check if the battery was inserted or removed whilst id
		* read was ongoing. Software debouncing needs to be done,
		* because hardware deboucing in the PMU is not long enough.
		*/
		bat_drv_hal_presence_detection_debouncing_start();
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_IDLE_NO_READ_ONGOING:
		/*
		* Stop possible running debounce timer - if a timer callback
		* is running the process will be stopped when the timer callback
		* work runs.
		*/
		(void)del_timer(&bat_drv_hal_instance.debouncing.timer);

		/* Disable the presence detect interrupt because the pullup will
		be disabled */
		bat_drv_hal_batrm_det_enable(false);
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAINED:
		/*
		* Reenable presence detect interrupt.
		* Subscriber report is done in the STM tick function
		*/
		bat_drv_hal_enable_presence_detect_interrupt(
		 BAT_DRV_HAL_PRESENCE_STM_GET_STATUS(
		  BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_PRESENT));
		break;

	case BAT_DRV_HAL_PRESENCE_STM_STATE_PRE_INIT:
		/* Should never transition to this state, only start on it */
	default:
		BUG();
		break;
	}
}

/**
 * bat_drv_hal_presence_stm_tickTick - PMU hal presence state machine with new
 * event.
 *
 * This state machine handles determining whether the battery is fitted or not.
 * If it is fitted, the reading its id resistor.
 *
 * It receives events indicating if a battery was fitted or removed from the PMU
 * driver and then invokes a debouncing sequence before finally deeming the
 * battery present or not present.
 *
 * This state machine also handles events from the pullup state machine in this
 * file. If the pullup resistor was disabled this consititutes a blackout period
 * where battery insertion/removal will not be detected. After the blackout
 * period is over (i.e. the pullup is reenabled), this statemachine will enter a
 * debouncing sequence to check if the battery presence state has changed.
 *
 * An exception is when a pullup disable request occurs after a read of the
 * battery id has been invoked by this state machine. A battery id read request
 * to the ADC sensor driver will cause it to make a pullup disable request back
 * to this HAL via the lock interface. In this case the best this state machine
 * can do is to confirm if the battery is still fitted after the id was read and
 * the pullup was reenabled. There is a small chance that the battery was
 * swopped undetected whilst the pullup was disabled, however to restart another
 * id read would result in an endless loop.
 *
 * @event	[in] State machine event
*/
static void bat_drv_hal_presence_stm_tick(enum bat_drv_hal_presence_stm_event
					event)
{
	enum bat_drv_hal_presence_stm_state new_state;

	/* Firstly take care of inputs */
	bat_drv_hal_presence_stm_do_inputs(event);

	/* Do required state transition */
	new_state =
		bat_drv_hal_presence_stm_do_state_transition(
			bat_drv_hal_instance.presence_stm.current_state,
								event);

	/* If there was a state change do the outputs for entering the new
	state */
	if (new_state != bat_drv_hal_instance.presence_stm.current_state) {

		u32 debug_index = bat_drv_hal_instance.presence_stm_debug.index;
		bat_drv_hal_instance.presence_stm_debug.
					stm_state_change_seq_num++;

		/* Log STM debug data */
		bat_drv_hal_instance.presence_stm_debug.data[debug_index].
							timestamp = jiffies;

		bat_drv_hal_instance.presence_stm_debug.data[debug_index].
								event = event;

		bat_drv_hal_instance.presence_stm_debug.data[debug_index].
					stm_state_change_seq_num =
			bat_drv_hal_instance.presence_stm_debug.
					stm_state_change_seq_num;

		bat_drv_hal_instance.presence_stm_debug.data[debug_index].
							new_state = new_state;
		bat_drv_hal_instance.presence_stm_debug.index++;
		bat_drv_hal_instance.presence_stm_debug.index &=
				STM_STATE_CHANGE_DEBUG_DATA_SIZE - 1;
		/* End of debug logging */

		bat_drv_hal_instance.presence_stm.current_state = new_state;
		bat_drv_hal_presence_stm_do_outputs(new_state);
	}
	/*
	* Notify subscribers if the battery status is known,
	* the HAL instance has been started and either:
	*	(A) The first client update has not been done,
	*	(B) The battery presence status has changed,
	*	(C) There has been a pmu callback, battery was previously
	*	present and the new battery presence status is "present". This
	*	is a "debounce" event and the status update function will take
	*	care of reporting it. In this case, clear the PMU event before
	*	calling the status update.
	*/
	if ((BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAINED ==
		new_state && BAT_DRV_HAL_PRESENCE_STM_GET_STATUS(
				BAT_DRV_HAL_PRESENCE_STM_EVENT_START))) {

		bool battery_present = BAT_DRV_HAL_PRESENCE_STM_GET_STATUS(
			BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_PRESENT);
		bool presence_changed = (battery_present !=
			bat_drv_hal_instance.last_reported_presence_state);

		bool pmu_cb_triggered_presence_attain =
		 BAT_DRV_HAL_PRESENCE_STM_GET_STATUS(
		  BAT_DRV_HAL_PRESENCE_STM_EVENT_PMU_CB_PRESENCE_CHANGED);

		bool do_status_update =
			((!bat_drv_hal_instance.presence_stm.
				client_initial_update_done) ||
				presence_changed || (battery_present &&
					pmu_cb_triggered_presence_attain));

		if (pmu_cb_triggered_presence_attain) {
			BAT_DRV_HAL_PRESENCE_STM_CLEAR_INPUT
		 (BAT_DRV_HAL_PRESENCE_STM_EVENT_PMU_CB_PRESENCE_CHANGED);
		}
		if (do_status_update) {
			bat_drv_hal_check_battery_status_update
			 (BAT_DRV_HAL_PRESENCE_STM_GET_STATUS
		 (BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_PRESENT));
		}
	}
}

/**
 * bat_drv_hal_presence_change_cb - PMU driver callback for battery presence
 * state change. Schedules a work to signal the state machine, This function is
 * triggered by the battery removal detection IRQ.
 *
 * @irq		[in] (not used)
 * @dev		[in] (not used)
 */
static irqreturn_t bat_drv_hal_presence_change_cb(int irq, void *dev)
{
	/* Unused */
	(void)irq;
	(void)dev;

	/* Entering critical section */
	spin_lock(&bat_drv_hal_instance.debouncing.lock);

	/* If currently in a debouncing check don't re-disable the battery
	presence detection interrupt or start a second simultaneous debounce
	check sequence. */
	if (bat_drv_hal_instance.debouncing.currently_checking_debouncing) {
		/* Done updating critical section. */
		spin_unlock(&bat_drv_hal_instance.debouncing.lock);
	} else {
		/* Enter debouncing check sequence */
		bat_drv_hal_instance.debouncing.currently_checking_debouncing =
									true;

		/* Done updating critical section. */
		spin_unlock(&bat_drv_hal_instance.debouncing.lock);
		/* Determine battery presence following interrupt */
		BAT_DRV_HAL_ENQUEUE(bat_drv_hal_presence_stm_tick,
		 BAT_DRV_HAL_PRESENCE_STM_EVENT_PMU_CB_PRESENCE_CHANGED);
	}
	return IRQ_HANDLED;
}

/**
 * bat_drv_hal_presence_detection_debouncing_work - Work to perform deboucing
 * actions in thread context. Restarts a debouncing timer, waiting for
 * mechanical switch oscillations to cease before determining whether battery is
 * fitted or not.
 *
 * @dummy		[in] Unused parameter
*/
static void bat_drv_hal_presence_detection_debouncing_work(void *dummy)
{
	bool new_bat_presence_state;

	/* Unused parameter */
	(void)dummy;

	BAT_DRV_HAL_DEBUG_NO_PARAM(BAT_DRV_HAL_DEBUG_DETECTION_DEBOUNCING_WORK);

	/* The timer will be stopped if a pullup disable request occurs whilst
	debouncing is ongoing. If the LISR timer callback was already running
	when the timer was stopped then the process is stopped here by not
	processing the timer callback and restarting the timer. */
	if ((BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_ATTAIN
		== bat_drv_hal_instance.presence_stm.current_state)
		 || (BAT_DRV_HAL_PRESENCE_STM_STATE_PRESENCE_STATE_CONFIRM
		  == bat_drv_hal_instance.presence_stm.current_state)) {

		/* Get the battery presence state */
		new_bat_presence_state = bat_drv_hal_get_presence_state();

		/* Check whether battery presence state has changed */
		if (bat_drv_hal_instance.debouncing.last_bat_presence_state !=
		 new_bat_presence_state) {

			/* Store new state, reset stable counter and increment
			unstable counter */
			bat_drv_hal_instance.debouncing.
			 last_bat_presence_state = new_bat_presence_state;

			bat_drv_hal_instance.debouncing.
				presence_stable_counter = 0;

			bat_drv_hal_instance.debouncing.
				presence_unstable_counter++;

			(void)mod_timer(&bat_drv_hal_instance.debouncing.timer,
					jiffies + msecs_to_jiffies(
					 BATTERY_CONNECTOR_DEBOUNCE_TIME_MS));

			/* Contact has bounced too long. It is most likely
			defective! */
			BUG_ON(BATTERY_CONNECTOR_DEBOUNCE_UNSTABLE_COUNT <=
				bat_drv_hal_instance.debouncing.
					presence_unstable_counter);
		} else {

			/* Increment stable counter and check whether it has
			been stable long enough to report battery status
			change. */
			bat_drv_hal_instance.debouncing.
				presence_stable_counter++;

			if (BATTERY_CONNECTOR_DEBOUNCE_STABLE_COUNT <=
					bat_drv_hal_instance.debouncing.
						presence_stable_counter) {
				/* Inform presence stm of presence detection
				event */
				bat_drv_hal_presence_stm_tick((
				 bat_drv_hal_instance.debouncing.
					last_bat_presence_state ?
			BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_PRESENT :
		BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_DETERMINED_NOT_PRESENT));

			} else {
				/* Continue reading pin until it is deemed
				stable */
				(void)mod_timer(&bat_drv_hal_instance.
				 debouncing.timer, jiffies + msecs_to_jiffies(
					BATTERY_CONNECTOR_DEBOUNCE_TIME_MS));
			}
		}
	}
}

/**
 * bat_drv_hal_presence_detection_debouncing_timer_cb - Timer callback for
 * battery contact debouncing.
 *
 * @param	[in] Unused parameter
 */
static void bat_drv_hal_presence_detection_debouncing_timer_cb(unsigned long
							param)
{
	/* Unused parameter */
	(void)param;

	/* Perform PMU driver accesses in work context */
	BAT_DRV_HAL_ENQUEUE(bat_drv_hal_presence_detection_debouncing_work,
			NULL);
}

static inline int bat_drv_hal_get_pdata(struct bat_drv_hal_data *hal)
{
	struct device_node *np = hal->p_idi_device->device.of_node;
	struct bat_drv_hal_platform_data *pdata;

	if (IS_ENABLED(CONFIG_OF))
		return bprofile_parse_dt(&hal->pdata.supported_batteries,
				&hal->pdata.supported_batteries_len,
				&hal->pdata.bprofiles,
				&hal->pdata.bprofiles_len,
				np);


	pdata = (struct bat_drv_hal_platform_data *)
			hal->p_idi_device->device.platform_data;

	if (!pdata || !pdata->supported_batteries)
		return -ENODATA;

	hal->pdata.supported_batteries = pdata->supported_batteries;
	hal->pdata.supported_batteries_len = pdata->supported_batteries_len;

	return 0;
}

static inline void bat_drv_hal_release_pdata(struct bat_drv_hal_data *hal)
{
	if (IS_ENABLED(CONFIG_OF)) {
		kfree(hal->pdata.supported_batteries);
		kfree(hal->pdata.bprofiles);
	}
}


/**
 * bat_drv_hal_probe - Initialises the driver, when the device has been found.
 */
static int __init bat_drv_hal_probe(struct idi_peripheral_device *ididev,
				const struct idi_device_id *id)
{
	struct device *dev = &ididev->device;
	struct resource *res;
	int ret = 0;
	struct bat_drv_hal_data *hal = &bat_drv_hal_instance;

	BAT_DRV_HAL_DEBUG_NO_PARAM(BAT_DRV_HAL_DEBUG_EVENT_PROBE);

	/* Store platform device in static instance. */
	bat_drv_hal_instance.p_idi_device = ididev;

	ret =  idi_device_pm_set_class(ididev);
	if (ret) {
		pr_err("%s: Unable to register for generic pm class\n",
								__func__);
		return -EINVAL;
	}

	bat_drv_hal_instance.pm_state_en =
		idi_peripheral_device_pm_get_state_handler(ididev, "enable");

	if (bat_drv_hal_instance.pm_state_en == NULL) {
		pr_err("%s: Unable to get handler for PM state 'enable'!\n",
								__func__);
		return -EINVAL;
	}

	bat_drv_hal_instance.pm_state_dis =
		idi_peripheral_device_pm_get_state_handler(ididev, "disable");

	if (bat_drv_hal_instance.pm_state_dis == NULL) {
		pr_err("%s: Unable to get handler for PM state 'disable'!\n",
								__func__);
		return -EINVAL;
	}

	pr_info("%s: Getting PM state handlers: OK\n", __func__);

	ret = bat_drv_hal_get_pdata(hal);
	if (ret)
		return ret;

	res =
		idi_get_resource_byname(&ididev->resources, IORESOURCE_MEM,
								"pmu");
	if (res == NULL) {
		pr_err("getting PMU's 'BATRM_DET' resource failed!\n");
		ret = -EINVAL;
		goto res_fail;
	}

	bat_drv_hal_instance.pmu_res = res;

	res = idi_get_resource_byname(&ididev->resources,
					IORESOURCE_IRQ, "brd");
	if (res == NULL) {
		pr_err("getting brd resource failed!\n");
		ret = -EINVAL;
		goto res_fail;
	}

	bat_drv_hal_instance.irq = res->start;
	if (!bat_drv_hal_instance.irq) {
		dev_err(dev, "could not get interrupt\n");
		ret = -EINVAL;
		goto res_fail;
	}

	/* Initialise the spinlock and timer used for debouncing the battery
	connector. */
	spin_lock_init(&bat_drv_hal_instance.debouncing.lock);
	spin_lock_init(&bat_drv_hal_instance.work.lock);
	setup_timer(&bat_drv_hal_instance.debouncing.timer,
		bat_drv_hal_presence_detection_debouncing_timer_cb, 0);

	/* Initialise work queue. Must be single thread to ensure
	serialisation. */
	INIT_WORK(&bat_drv_hal_instance.work.work,
			bat_drv_hal_execute_function);

	wake_lock_init(&bat_drv_hal_instance.work.kfifo_wakelock,
			WAKE_LOCK_SUSPEND, "batdrv_kfifo_wakelock");

	/*
	* Create private, single-threaded workqueue instead of using one of
	* the system predefined workqueues to reduce latency
	*/
	bat_drv_hal_instance.work.p_work_queue =
		create_singlethread_workqueue(dev_name(&ididev->device));

	BUG_ON(NULL == bat_drv_hal_instance.work.p_work_queue);
	INIT_KFIFO(bat_drv_hal_instance.work.fifo);

	/* Disable the presence detect interrupt before enabling presence
	detection hardware. */
	bat_drv_hal_batrm_det_enable(false);
	/*
	* (1)
	* Initialise BREG bit for battery fitted case, i.e. to detect a rising
	* edge on the M0 pin
	* if a battery is removed and also to disable the SIM via the SAPD bit
	* functionality,
	* if the SAPD bit is set and the battery is not fitted. The SAPD bit is
	* controlled in a different driver that should come later in the boot
	* sequence than the battery driver.
	*
	* This change from the default PMU hardware setting of the BREG bit
	* doesn't actually affect the operation of the battery driver code,
	* because the interrupt is disabled whilst determining the presence
	* state of the battery and also the changing polarity of the BRD bit
	* according to the BREG bit setting is taken into account.
	*
	* (2)
	* Enable battery removal detection. Setting the EBRM bit enables the
	* internal pull-up resistor for a correct presence state reading.
	*
	* EBRM = 1
	* BREG = 0
	* NOTE: Only these two bit are important to this driver.
	* Other bits will not be changed.
	*/

	ret = idi_set_power_state(bat_drv_hal_instance.p_idi_device,
				bat_drv_hal_instance.pm_state_en, true);

	if (ret) {
		pr_err("%s: setting PM state '%s' failed!\n", __FILE__,
				bat_drv_hal_instance.pm_state_en->name);
		ret = -EIO;
		goto set_pm_state_fail;
	}

	{

		u32 batrm_det = bat_drv_pmu_ioread(&bat_drv_hal_instance,
				PMU_AG6X0_BAT_DET);
		batrm_det = batrm_det | PMU_AG6X0_BATRM_DET_EBRM_SET;
		bat_drv_pmu_iowrite(&bat_drv_hal_instance,
					PMU_AG6X0_BAT_DET, batrm_det);
	}
	bat_hal_set_pm_state(bat_drv_hal_instance.p_idi_device,
			bat_drv_hal_instance.pm_state_dis, false);

	/* Start battery presence attainment process - starting it from here
	guarantees state machine is not in preinit state when the first message
	to disable pullup is processed */
	bat_drv_hal_presence_stm_tick
		(BAT_DRV_HAL_PRESENCE_STM_EVENT_BAT_PRESENCE_ATTAIN_REQUEST);
	/* Determine battery presence following interrupt */
	BAT_DRV_HAL_ENQUEUE(bat_drv_hal_presence_stm_tick,
			BAT_DRV_HAL_PRESENCE_STM_EVENT_START);

	bat_drv_hal_instance.initialised = true;

	return 0;

set_pm_state_fail:
	destroy_workqueue(bat_drv_hal_instance.work.p_work_queue);
res_fail:
	bat_drv_hal_release_pdata(hal);
	return ret;
}

/**
 * bat_drv_hal_remove - Release allocated resources.
 */
static int __exit bat_drv_hal_remove(struct idi_peripheral_device *ididev)
{
	BAT_DRV_HAL_DEBUG_NO_PARAM(BAT_DRV_HAL_DEBUG_EVENT_REMOVE);

	/* Delete allocated resources and mark driver as uninitialised. */
	if (bat_drv_hal_instance.initialised) {

		bat_drv_hal_instance.initialised = false;
		/* Deregister interrupt handler */
		bat_drv_hal_batrm_det_enable(false);

		(void)del_timer(&bat_drv_hal_instance.debouncing.timer);

		bat_drv_hal_release_pdata(&bat_drv_hal_instance);
	}
	return 0;
}

/**
 * bat_drv_hal_suspend() - Called when the system is attempting to suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int bat_drv_hal_suspend(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Nothing to do here except logging */
	BAT_DRV_HAL_DEBUG_NO_PARAM(BAT_DRV_HAL_DEBUG_EVENT_SUSPEND);
	return 0;
}

/**
 * bat_drv_hal_resume() - Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int bat_drv_hal_resume(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Nothing to do here */
	BAT_DRV_HAL_DEBUG_NO_PARAM(BAT_DRV_HAL_DEBUG_EVENT_RESUME);
	return 0;
}

const struct dev_pm_ops bat_drv_hal_pm = {
	.suspend = bat_drv_hal_suspend,
	.resume = bat_drv_hal_resume,
};

static const struct idi_device_id idi_ids[] = {
	{
		.vendor = IDI_ANY_ID,
		.device = IDI_DEVICE_ID_INTEL_AG620,
		.subdevice = IDI_SUBDEVICE_ID_INTEL_BAT,
	},

	{ /* end: all zeroes */},
};


static struct idi_peripheral_driver bat_hal_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.pm = &bat_drv_hal_pm,
	},

	.p_type = IDI_BAT,
	.id_table = idi_ids,
	.probe = bat_drv_hal_probe,
	.remove = bat_drv_hal_remove,
};

static int __init bat_drv_hal_init(void)
{
	int ret;

	/* NOTE: Must initialise debug data spinlock before any logging. */
	spin_lock_init(&bat_drv_hal_debug_info.lock);
	BAT_DRV_HAL_DEBUG_NO_PARAM(BAT_DRV_HAL_DEBUG_EVENT_INIT);

	ret = idi_register_peripheral_driver(&bat_hal_driver);
	return ret;
}

static void __exit bat_drv_hal_exit(void)
{
	BAT_DRV_HAL_DEBUG_NO_PARAM(BAT_DRV_HAL_DEBUG_EVENT_EXIT);
	idi_unregister_peripheral_driver(&bat_hal_driver);
}

late_initcall(bat_drv_hal_init);
module_exit(bat_drv_hal_exit);

MODULE_DESCRIPTION("AGOLD6x0 Battery Driver HAL");
MODULE_LICENSE("GPL v2");
