/*
* intel_adc.c - generic ADC driver.
*
* Copyright (C) 2013 Intel Corporation
*
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
* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*/

/* Device name definition for Intel ADC */
#define INTEL_ADC_DEV_NAME	"intel_adc"
#define pr_fmt(fmt) INTEL_ADC_DEV_NAME": "fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/consumer.h>
#include <linux/iio/intel_adc_hal_interface.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/debugfs.h>
#include <linux/time.h>
#include <linux/spinlock.h>
#include <linux/kfifo.h>
#include <linux/completion.h>
#include <linux/iio/driver.h> /* IIO consumer map */

#include <linux/wakelock.h>

/* This is used as the timeout for stuck conversion supervisory timer in ms.
   Timeout must take into account worst-case scenario and timer tolerance */
#define ADC_MAX_MEAS_CONVERSION_LATENCY_MS (150000)

/* ADC STM FIFO
   Queue depth. It must be a power of 2! */
#define ADC_STM_FIFO_DEPTH (64)

/* Number of measurement queues.
   Three queues are supported. */
#define ADC_NUMBER_OF_QUEUES (2)

/* Supported queue indexes.
   Queue[0] has normal priority and is meant for all general measurements.
   Queue[1] has low priority and meant for Open Circuit Voltage Measurements.*/
#define ADC_NORMAL_PRIO_QUEUE_INDEX (0)
#define ADC_LOW_PRIO_QUEUE_INDEX  (1)

/* High priority measurement request queue.
   Queue depth. It must be a power of 2! */
#define ADC_NORMAL_PRIO_QDEPTH (1<<5)
/* Low priority measurement request queue.
   Queue depth. It must be a power of 2! */
#define ADC_LOW_PRIO_QDEPTH (1<<2)

/* Queue empty element */
#define ADC_QUEUE_EMPTY_ELEMENT (NULL)

/* Macros for queue management: Next element.  */
#define ADC_QNEXT(headtail, qdepth)     (((headtail) + 1) & ((qdepth) - 1))
/* Macros for queue management: Empty condition. */
#define ADC_QEMPTY(head, tail, qdepth)   ((head) == (tail))
/* Macros for queue management: Full condition. */
#define ADC_QFULL(head, tail, qdepth)    ((head) == ADC_QNEXT(tail, qdepth))
/* Macros for queue management: Advance Queue Head. */
#define ADC_QADVANCE_HEAD(head, qdepth) (head = ((head) + 1) & ((qdepth) - 1))
/* Macros for queue management: Advance Queue Tail. */
#define ADC_QADVANCE_TAIL(tail, qdepth) (tail = ((tail) + 1) & ((qdepth) - 1))

/* Size of debug data array (has to be power of 2!!!) */
#define ADC_DEBUG_DATA_SIZE (1<<6)

#define DRIVER_NAME "intel_adc"

#define SYSFS_INPUT_VAL_LEN (1)

/* Macro to log debug data */
#define ADC_DEBUG_DATA_LOG(_event, _state, _hmeas, _context, _context2, \
					_context3, _context4, _context5) \
{ \
	spin_lock(&adc_debug_data.lock); \
	adc_debug_data.log_array[adc_debug_data.index].time_stamp = jiffies; \
	adc_debug_data.log_array[adc_debug_data.index].event = (_event); \
	adc_debug_data.log_array[adc_debug_data.index].adc_state = (_state); \
	\
	adc_debug_data.log_array[adc_debug_data.index].hmeas = \
						(unsigned long)(_hmeas); \
	adc_debug_data.log_array[adc_debug_data.index].context = \
						(unsigned long)(_context); \
	adc_debug_data.log_array[adc_debug_data.index].context2 = \
						(unsigned long)(_context2); \
	adc_debug_data.log_array[adc_debug_data.index].context3 = \
						(unsigned long)(_context3); \
	adc_debug_data.log_array[adc_debug_data.index].context4 = \
						(unsigned long)(_context4); \
	adc_debug_data.log_array[adc_debug_data.index].context5 = \
						(unsigned long)(_context5); \
	adc_debug_data.index++; \
	adc_debug_data.index &= (ADC_DEBUG_DATA_SIZE-1); \
	spin_unlock(&adc_debug_data.lock); \
	pr_debug("%s->(%d):0x%lu,%lu,%lu,%lu,%lu,%lu\n", #_event, \
			(unsigned int)_state, (unsigned long)_hmeas, \
			(unsigned long)_context, (unsigned long)_context2, \
			(unsigned long)_context3, (unsigned long)_context4, \
			(unsigned long)_context5); \
}

#define intel_adc_dbg_printk(fmt, ...) \
		pr_debug(fmt, ##__VA_ARGS__);

/**
 * adc_meas_instance - ADC measurement handle: it contains all information
 * pertining a measurement instance.
 *
 * @dequeue_timestamp:		Timestamp in jiffies done when measurement is
 *				dequeued. This is used to prevent settling
 *				measurements getting stuck.
 * @channel:			The channel a read is requested on.
 * @latest_result:		Measurement result data structure.
 * @uv:				Measurement result in uv.
 * @na:				Current level used for the bias circuit.
 * @error:			Measurement error code value.
 * @settling:			Measurement  settling feature data structure.
 * @max_signal_settling_time_ms:Maximum time in ms to allow measurement signal
 *				to settle. Zero if settling is disabled.
 * @not_settled_value_uv:	Transient measurement value obtained before
 *				signal is settled.
 * @done:			Completion structure. This is used to
 *				synchronise the client context with the
 *				measurement, i.e. client context waits on
 *				completion.
 */
struct adc_meas_instance {
	long long dequeue_timestamp;
	int channel;
	struct {
		int uv;
		int na;
		int error;
	} latest_result;
	struct {
		uint max_signal_settling_time_ms;
		int not_settled_value_uv;
	} settling;
	bool autoscaling_on;
	struct completion done;
};

/**
 * Queue handling type
 *@head:			Queue Head index.
 *@tail:			Queue Tail index.
 *@size:			Queue data size.
 *@*p_data:			Pointer to queue data.
 */
struct adc_meas_queue {
	uint head;
	uint tail;
	uint size;
	struct adc_meas_instance **p_data;
};

/* Enum for ADC STM events */
enum adc_stm_events {
	ADC_STM_REQUEST,
	ADC_STM_POWER_UP_DONE,
	ADC_STM_MEAS_SET_UP_DONE,
	ADC_STM_MEAS_DONE,
	ADC_STM_SETTLING_RETRY,
	ADC_STM_PENDING_CHECK
};

/**
 * adc_stm_event_payload - Union that contains the message payload of event.
 * @power_mode:	Power mode reported by HAL.
 * @p_meas_req:	Pointer to measurement request.
 * @adc_result:	ADC result value.
 */
union adc_stm_event_payload {
	enum adc_hal_power_mode power_mode;
	struct adc_meas_instance *p_meas_req;
	int adc_result;
};

/**
 * adc_stm_mess - Structure for STM message payload.
 * @event:	Event that ticks ADC STM.
 * @payload:	Union that contains the message payload of event.
 */
struct adc_stm_mess {
	enum adc_stm_events event;
	union adc_stm_event_payload payload;

};

/* Enum for ADC STM states */
enum adc_states {
	ADC_NOT_INITIALISED,
	ADC_READY,
	ADC_BUSY
};

/**
 * adc_manager_data - Status of the ADC driver.
 *
 * @keep_running:		Flag to stop ADC STM thread.
 * @state:			STM state.
 * @supported_channels:		List of supported channels.
 * @p_hal_if:			Pointer to HAL interface for each of the
 *				supported channels.
 * @p_channel_data:		Pointer to channel data of each of the supported
 *				channels.
 * @latest_adc_bias_na:		Latest adc bias current for supported channel
 *				in na
 * @registered_hals:		List of supported channels.
 * @p_hal_if:			Pointer to each registered HAL interface.
 * @num:			Number of registered HAL interfaces.
 * @adc_stm_fifo:		FIFO used to pass events to ADC STM.
 * @qlock:			adc_stm_fifo lock.
 * @supervisory_timer:		Supervisory timer for stuck measurements.
 * @active_meas:		Currently active measurement handle.
 * @queue:			Array of measurement queues.
 * @settling:			Measurement  settling feature data structure.
 * @accuracy_uv:		Settling accuracy in uv.
 * @polling_ms:			Size of the polling interval for settling.
 * @polling_timer:		Timer used to poll for settling.
 * @stats:			ADC measurement statistics.
 * @meas_scheduled_count:	Total number of measurements scheduled so far.
 * @meas_performed_count:	Total number of measurements performed so far.
 * @ocv_performed_count:	Number of OCV measurements performed so far.
 * @meas_again_error_cnt:	Number of measurements EAGAIN errors so far.
 * @meas_io_error_cnt:		Number of measurements EIO errors so far.
 * @lock:			Spin lock used to protect critical sections
 *				when modifying device state data.
 * @suspended:			TRUE=Device is suspended, otherwise not.
 */
struct adc_manager_data {
	bool keep_running;
	enum adc_states state;
	struct {
		struct adc_hal_interface *p_hal_if;
		struct intel_adc_hal_channel_data *p_channel_data;
		int latest_adc_bias_na;
	} supported_channels[ADC_MAX_NO_OF_CHANNELS];
	struct {
		struct adc_hal_interface *p_hal_if[ADC_MAX_NO_OF_CHANNELS];
		int num;
	} registered_hals;
	DECLARE_KFIFO_PTR(adc_stm_fifo, struct adc_stm_mess);
	spinlock_t qlock;
	struct timer_list supervisory_timer;

	struct wake_lock measurement_wakelock;

	struct adc_meas_instance *p_active_meas;
	struct adc_meas_queue queue[ADC_NUMBER_OF_QUEUES];
	struct adc_meas_instance *low_prio_queue[ADC_LOW_PRIO_QDEPTH];
	struct adc_meas_instance *normal_prio_queue[ADC_NORMAL_PRIO_QDEPTH];
	struct task_struct *adc_thread;
	struct {
		int accuracy_uv;
		int polling_ms;
		struct timer_list polling_timer;
	} settling;
	struct {
		/* Number of measurements scheduled so far */
		uint meas_scheduled_count;
		/* Number of non-triggered measurements performed so far */
		uint meas_performed_count;
		/** @rief Number of triggered measurements performed so far */
		uint ocv_performed_count;
		/* Number of measurements that ended up in EAGAIN error
		so far */
		uint meas_again_error_cnt;
		/* Number of measurements that ended up in EIO error so far */
		uint meas_io_error_cnt;
	} stats;
	spinlock_t lock;
	bool suspended;
};

/*  Enum for ADC debug events */
enum adc_debug_event {
	ADC_INIT,
	ADC_DE_INIT,
	ADC_STM_TICK,
	ADC_PWR_UP,
	ADC_PWR_UP_DONE,
	ADC_PWR_UP_FAILED,
	ADC_PWR_DN,
	ADC_ENQUEUE,
	ADC_DEQUEUE,
	ADC_START_SET_UP,
	ADC_START_SET_UP_DONE,
	ADC_START_MEAS,
	ADC_MEAS_DONE,
	ADC_MEAS_SETTLING_CHECK,
	ADC_MEAS_SETTLING_RETRY,
	ADC_MEAS_SETTLING_DONE,
	ADC_MEAS_SETTLING_FAILED,
	ADC_MEAS_AUTOSCALING_CHECK,
	ADC_MEAS_AUTOSCALING_RETRY,
	ADC_MEAS_AUTOSCALING_DONE,
	ADC_TIMEOUT,
	ADC_OCV_RESCHEDULE,
	ADC_OCV_MEASUREMENT_FAILED,
	ADC_SUSPEND,
	ADC_RESUME
};


/**
 * adc_debug_data - Structure to collect debug data.
 * @index:		Index of log array.
 * @lock:			Spin lock for atomic access.
 * @log_array:		Debug data logging array.
 * @time_stamp:		System time stamp in jiffies.
 * @event:		Event that caused logging.
 * @adc_state:		Current ADC State Machine state.
 * @hmeas:		Measurement handle (if available).
 * @context-5:		Event context parameters.
 */
struct adc_debug_data {
	uint index;
	spinlock_t lock;
	struct {
		uint time_stamp;
		enum adc_debug_event event;
		enum adc_states adc_state;
		unsigned long hmeas;
		unsigned long context;
		unsigned long context2;
		unsigned long context3;
		unsigned long context4;
		unsigned long context5;
	} log_array[ADC_DEBUG_DATA_SIZE];
};

/* Measurement Unit management structure */
static struct adc_manager_data adc_manager = {
	.keep_running = true,
	.state = ADC_NOT_INITIALISED,
	{[0 ... (ADC_MAX_NO_OF_CHANNELS - 1)] = {NULL, NULL} },
	{ {[0 ... (ADC_MAX_NO_OF_CHANNELS - 1)] = NULL}, 0 },
	.suspended = false,
};

/* Array to collect debug data */
static struct adc_debug_data adc_debug_data;

/*---------------------------- Function prototypes: -------------------------*/

/**
 * intel_adc_tick_stm() - It implements ADC high level state machine.
 *  Always runs in same context to guarantee serialisation.
 *
 * @event:	Event that drives the state machine tick.
 *
*/
static void intel_adc_tick_stm(enum adc_stm_events event,
				union adc_stm_event_payload payload);

/*---------------------------- Local Functions: --------------------------*/

/**
 * convert_to_iio_node_name() - Extract extended name of ADC.
 * e.g. ADC NAME - "VBAT_ADC", Extended name - "vbat"
 * @src:	Name of the ADC.
 * @return:	Extended name of ADC.
 */
static const char *convert_to_iio_node_name(const char *src)
{
	char *p, *dst;
	int i, len = 0;

	len	= strlen(src);
	p = kmalloc(len, GFP_KERNEL);
	BUG_ON(p == NULL);
	strcpy(p, src);
	dst = p;
	len = len - strlen("_ADC");
	for (i = 0; i < len; i++) {
		*p = tolower(*p);
		p++;
	}
	*p = '\0';
	return dst;
}

/**
 * intel_adc_add_mess() - Add a message to the ADC STM FIFO.
 * @p_payload:	Pointer to message payload.
 * @payload_size:	Message payload size.
 */
static inline void intel_adc_add_mess(struct adc_stm_mess *p_payload)
{
	struct adc_manager_data *st = &adc_manager;
	/* Make sure that FIFO will not be overflown */
	BUG_ON(kfifo_is_full(&st->adc_stm_fifo));

	/* Add ADC STM message to the queue. */
	kfifo_in_spinlocked(&st->adc_stm_fifo, p_payload, 1, &st->qlock);
	wake_up_process(st->adc_thread);
}

/**
 * intel_adc_fetch_mess() - Fetches a message from the ADC STM FIFO.
 * @p_payload:	Pointer to message payload.
 * @payload_size:	Message payload size.
 */
static inline int intel_adc_fetch_mess(struct adc_stm_mess *p_payload)
{
	struct adc_manager_data *st = &adc_manager;
	/* Fetch ADC STM message from the queue. */
	return kfifo_out_spinlocked(&st->adc_stm_fifo, p_payload, 1,
					&st->qlock);
}

/**
 * intel_adc_hal_event_callback() - Generic HAL callback function.
 * @adc_hal_cb_event:	Event that originates the callback.
 * @adc_hal_cb_param:	Event specific callback parameters.
 */
static void intel_adc_hal_event_callback(enum adc_hal_cb_event adc_hal_cb_event,
					union adc_hal_cb_param
					adc_hal_cb_param)
{
	struct adc_stm_mess mess;

	/* Set Event and payload according to type of callback */
	switch (adc_hal_cb_event) {
	case ADC_HAL_CB_EVENT_POWER_UP_DONE:
		mess.event = ADC_STM_POWER_UP_DONE;
		mess.payload.power_mode = adc_hal_cb_param.power_mode;
		break;
	case ADC_HAL_CB_EVENT_MEAS_SET_UP_DONE:
		mess.event = ADC_STM_MEAS_SET_UP_DONE;
		/* There is no payload */
		break;
	case ADC_HAL_CB_EVENT_MEAS_DONE:
		mess.event = ADC_STM_MEAS_DONE;
		mess.payload.adc_result = adc_hal_cb_param.adc_result;
		break;
	default:
		/* We should never get here */
		BUG();
		break;
	};
	/* Add ADC STM message to the queue. */
	intel_adc_add_mess(&mess);
}

/**
 * intel_adc_enqueue_meas() - Enqueques measurement request in the right queue
 * according to measurement type.
 *
 * @p_meas:	Pointer to measurement request to enqueue.
 */
static void intel_adc_enqueue_meas(struct adc_meas_instance *p_meas)
{
	struct adc_meas_queue *p_queue = NULL;

	/* Use low priority queue for OCV measurements. */
	if (p_meas->channel == ADC_V_BAT_OCV)
		p_queue = &adc_manager.queue[ADC_LOW_PRIO_QUEUE_INDEX];
	else
		p_queue = &adc_manager.queue[ADC_NORMAL_PRIO_QUEUE_INDEX];

	/* Do debug data logging */
	ADC_DEBUG_DATA_LOG(ADC_ENQUEUE,
				adc_manager.state,
				p_meas,
				p_meas->channel,
				p_meas->latest_result.uv,
				p_meas->latest_result.na,
				p_meas->settling.max_signal_settling_time_ms,
				p_meas->autoscaling_on);

	/* Check whether queue is full  */
	if (ADC_QFULL(p_queue->head, p_queue->tail, p_queue->size)) {
		/* Report error */
		pr_err("%s ADC Queue is full\n", __func__);
		p_meas->latest_result.error = -EAGAIN;
		/* Signal to release client waiting on completion */
		complete(&p_meas->done);
	} else {
		/* Check whether queue is corrupted  */
		BUG_ON(ADC_QUEUE_EMPTY_ELEMENT !=
			*(p_queue->p_data + p_queue->tail));

		/* Account for total measurements scheduled */
		adc_manager.stats.meas_scheduled_count++;

		/* Enqueue measurement handle to be serviced as soon as ADC is
		available */
		/* Put handle in measurement queue */
		*(p_queue->p_data + p_queue->tail) = p_meas;
		/* Advance tail index */
		ADC_QADVANCE_TAIL(p_queue->tail, p_queue->size);
	}
}

/**
 * intel_adc_set_power_mode() - Set  new power mode through HAL if the new power
 *			mode is different from its current mode.
 *
 * @p_hal_if:		Pointer to specific HAL to set power on.
 * @new_power_mode:	New power mode to set.
 *			Returns: 0= mode was changed and callback is
 *			required, WNOTREQ= no mode change or callback
 *			is not required, -X= error.
 */
static int intel_adc_set_power_mode(struct adc_hal_interface *p_hal_if,
					enum adc_hal_power_mode new_power_mode)
{
	int halret = WNOTREQ;
	enum adc_hal_power_mode current_power_mode;

	/* Protect critical section when testing and modifying device
	state data */
	spin_lock(&adc_manager.lock);
	if (adc_manager.suspended == true) {
		spin_unlock(&adc_manager.lock);
		intel_adc_dbg_printk("%s ADC in suspend\n", __func__);
		return -EIO;
	}

	/* End of critical section */
	spin_unlock(&adc_manager.lock);

	/* Get current power mode */
	BUG_ON((p_hal_if->get) (ADC_HAL_GET_POWER_MODE,
				((union adc_hal_get_params) {
				.p_power_mode = &current_power_mode})));

	/* Check whether power mode needs to be changed, i.e. if current and
	new modes are different */
	if (current_power_mode != new_power_mode) {
		switch (new_power_mode) {
		case ADC_HAL_POWER_MODE_ON:

			wake_lock(&adc_manager.measurement_wakelock);

			/* Start supervisory timer prior for potential
			calibration measurement */
			mod_timer(&adc_manager.supervisory_timer,
					jiffies +
					msecs_to_jiffies
					(ADC_MAX_MEAS_CONVERSION_LATENCY_MS));

			/* Do debug data logging */
			ADC_DEBUG_DATA_LOG(ADC_PWR_UP,
					adc_manager.state,
					0,
					p_hal_if, new_power_mode, 0, 0, 0);

			/* Set new mode */
			halret =
				(p_hal_if->set) (ADC_HAL_SET_POWER_MODE,
						((union adc_hal_set_params) {
						.power_mode =
						ADC_HAL_POWER_MODE_ON}));

			if (WNOTREQ == halret) {
				/* Do debug data logging */
				ADC_DEBUG_DATA_LOG(ADC_PWR_UP_DONE,
						adc_manager.state,
						0,
						p_hal_if,
						new_power_mode, 0, 0, 0);
			} else if (0 > halret) {
				(void)del_timer_sync(&adc_manager.
							supervisory_timer);

				wake_unlock(&adc_manager.measurement_wakelock);

				/* Do debug data logging */
				ADC_DEBUG_DATA_LOG(ADC_PWR_UP_FAILED,
						adc_manager.state,
						0,
						p_hal_if,
						new_power_mode, 0, 0, 0);
			}
			break;
		case ADC_HAL_POWER_MODE_OFF:

			/* Do debug data logging */
			ADC_DEBUG_DATA_LOG(ADC_PWR_DN,
						adc_manager.state,
						0,
						p_hal_if, new_power_mode,
						0, 0, 0);

			/* Set new mode */
			halret =
				(p_hal_if->set) (ADC_HAL_SET_POWER_MODE,
						((union adc_hal_set_params) {
						.power_mode =
						ADC_HAL_POWER_MODE_OFF}));

			wake_unlock(&adc_manager.measurement_wakelock);

			break;
		default:
			BUG();
			break;
		}
	}

	return halret;
}

/**
 * intel_adc_step_up_meas() - Set up HAL and supervisory timer (if available) in
 * order to start a measurement.
 *
 * @p_meas:	Pointer to measurement instance.
 *		Returns: true= Set up is not required, false= Set up is required
 *		and when done it will be reported via callback.
 */
static bool intel_adc_step_up_meas(struct adc_meas_instance *p_meas)
{
	int ret = false;

	/* Do debug data logging only if Set up is required */
	ADC_DEBUG_DATA_LOG(ADC_START_SET_UP,
				adc_manager.state,
				p_meas,
				p_meas->channel,
				p_meas->latest_result.uv,
				p_meas->latest_result.na,
				p_meas->settling.max_signal_settling_time_ms,
				p_meas->autoscaling_on);

	/* Start measurement on through the relevant HAL */
	ret =
		(adc_manager.supported_channels[p_meas->channel].p_hal_if->
		set) (ADC_HAL_SET_UP_MEAS, ((union adc_hal_set_params) {
					.channel = p_meas->channel}));
	BUG_ON(ret && (WNOTREQ != ret));

	if (WNOTREQ == ret) {
		/* Do debug data logging only if Set up is not required */
		ADC_DEBUG_DATA_LOG(ADC_START_SET_UP_DONE,
				adc_manager.state,
				p_meas,
				p_meas->channel,
				p_meas->latest_result.uv,
				p_meas->latest_result.na,
				p_meas->settling.max_signal_settling_time_ms,
				p_meas->autoscaling_on);
		ret = true;
	}

	return ret;
}

/**
 * intel_adc_start_meas() - Get the ADC measurement result through the
 * relevant HAL.
 *
 * The result can be available immediately or be reported via callback
 * depending on the HAL HW capabiliy.
 */
static void intel_adc_start_meas(struct adc_meas_instance *p_meas)
{
	int ret;

	/* Do debug data logging */
	ADC_DEBUG_DATA_LOG(ADC_START_MEAS,
				adc_manager.state,
				p_meas,
				p_meas->channel,
				p_meas->latest_result.uv,
				p_meas->latest_result.na,
				p_meas->settling.max_signal_settling_time_ms,
				p_meas->autoscaling_on);

	/* Start supervisory timer prior to starting any measurement */
	mod_timer(&adc_manager.supervisory_timer,
			jiffies +
			msecs_to_jiffies(ADC_MAX_MEAS_CONVERSION_LATENCY_MS));

	/* Start measurement on through the relevant HAL */
	ret =
		(adc_manager.supported_channels[p_meas->channel].p_hal_if->
		get) (ADC_HAL_GET_ADC_MEAS, ((union adc_hal_get_params) {
						.adc_meas = {
						p_meas->channel,
						&p_meas->latest_result.na,
						&p_meas->latest_result.uv} }));
	BUG_ON(ret && (ENODATA != ret) && (-ERANGE != ret));

	/* If measurement result is already available or saturated, tick stm
	with meas_done event */
	if (!ret) {
		intel_adc_tick_stm(ADC_STM_MEAS_DONE,
			((union adc_stm_event_payload) {
				.adc_result = p_meas->latest_result.uv}));
	} else if (-ERANGE == ret) {
		intel_adc_tick_stm(ADC_STM_MEAS_DONE,
			((union adc_stm_event_payload) {
				.adc_result = -ERANGE}));

	}
}

/**
 * intel_adc_report_meas() - Provides measurement result back, whether
 * successful or not, to client by releasing relevant wait on completion.
 *
 * @p_meas:	Pointer to measurement instance.
 */
static void intel_adc_report_meas(struct adc_meas_instance *p_meas)
{
	/* Do measurment result stats */
	adc_manager.stats.meas_performed_count++;
	if (-EAGAIN == p_meas->latest_result.error)
		adc_manager.stats.meas_again_error_cnt++;
	else if (-EIO == p_meas->latest_result.error)
		adc_manager.stats.meas_io_error_cnt++;

	if (ADC_V_BAT_OCV == p_meas->channel)
		adc_manager.stats.ocv_performed_count++;

	/* Signal to release client waiting on completion */
	complete(&p_meas->done);
}

/**
 * intel_adc_start_settling_meas() - Settling polling timer callback. Start a
 * measurement in a delayed fashion.
 */
static void intel_adc_start_settling_meas(unsigned long dummy)
{
	struct adc_stm_mess mess;

	/* Unsed parameter */
	(void)dummy;

	/* Send message to ADC STM to re-try measurement settling  */
	/* Add ADC STM message to the queue. */
	mess.event = ADC_STM_SETTLING_RETRY;
	/* There is no payload */
	intel_adc_add_mess(&mess);
}

/**
 * intel_adc_stuck_measurement - Stuck measurement supervisory timer callback.
 */
static void intel_adc_stuck_measurement(unsigned long dummy)
{
	struct adc_meas_queue *p_queue;
	int i;

	/* Unsed parameter */
	(void)dummy;

	/* Do debug data logging */
	ADC_DEBUG_DATA_LOG(ADC_TIMEOUT,
				adc_manager.state,
				adc_manager.p_active_meas,
				adc_manager.p_active_meas->channel,
				adc_manager.p_active_meas->latest_result.uv,
				adc_manager.p_active_meas->latest_result.na,
				adc_manager.p_active_meas->settling.
				max_signal_settling_time_ms,
				adc_manager.p_active_meas->autoscaling_on);

	pr_err("intel_adc_stuck_measurement: adc stm state->%d, active_meas->handle:0x%lu, channel id:%d\n",
				adc_manager.state,
				(unsigned long)adc_manager.p_active_meas,
				adc_manager.p_active_meas->channel);

	/* dump HW registers */
	(adc_manager.supported_channels[adc_manager.p_active_meas->channel].
	p_hal_if->set)
		(ADC_HAL_DUMP_REGISTER, ((union adc_hal_set_params) {
					.channel =
					adc_manager.p_active_meas->channel}));

	for (i = ADC_NORMAL_PRIO_QUEUE_INDEX, p_queue =
		&adc_manager.queue[ADC_NORMAL_PRIO_QUEUE_INDEX];
		i < ADC_NUMBER_OF_QUEUES; i++, p_queue++) {
		struct adc_meas_instance *hmeas = NULL;

		if (ADC_QEMPTY(p_queue->head, p_queue->tail, p_queue->size))
			pr_err("\t No more measurement request(s) in queue->%d\n",
					i);

		while (!ADC_QEMPTY(p_queue->head, p_queue->tail,
							p_queue->size)) {
			/* Dequeue measurement request */
			hmeas = *(p_queue->p_data + p_queue->head);
			/* Advance head index */
			ADC_QADVANCE_HEAD(p_queue->head, p_queue->size);
			pr_err("\t Remaining measurement in queue->%d, meas->handle:0x%lu, channel id:%u\n",
					i, (unsigned long)hmeas,
					hmeas->channel);
		}
	}

	/* Then throw an assert */
	BUG();
}

/**
 * intel_adc_tick_stm() - It implements ADC high level state machine.
 *  Always runs in same context to guarantee serialisation.
 *
 * @event:	Event that drives the state machine tick.
 *
*/
static void intel_adc_tick_stm(enum adc_stm_events event,
				union adc_stm_event_payload payload)
{
	struct adc_meas_queue *p_queue;
	struct adc_meas_instance *hmeas;
	int i;
	bool do_conversion_done;

	/* Log any STM tick */
	ADC_DEBUG_DATA_LOG(ADC_STM_TICK,
				adc_manager.state,
				0, event, payload.adc_result, 0, 0, 0);

	switch (adc_manager.state) {
	case ADC_READY:
		switch (event) {
		case ADC_STM_REQUEST:
			/* Enqueue new measurement request */
			intel_adc_enqueue_meas(payload.p_meas_req);

			/* fall through is intentional */
		case ADC_STM_PENDING_CHECK:
			hmeas = NULL;

			/* Measurement Unit now free for new measurement
			 * If there is any queued and not to be closed
			 * measurement, do it now
			 * Start first with the high priority queue,
			 * then check low priority queue */
			for (i = ADC_NORMAL_PRIO_QUEUE_INDEX, p_queue =
			 &adc_manager.queue[ADC_NORMAL_PRIO_QUEUE_INDEX];
			  i < ADC_NUMBER_OF_QUEUES; i++, p_queue++) {

				if (!ADC_QEMPTY(p_queue->head, p_queue->tail,
							p_queue->size)) {

					/* Point to current head of the queue */
					BUG_ON(ADC_QUEUE_EMPTY_ELEMENT ==
					 *(p_queue->p_data + p_queue->head));

					/* Dequeue measurement request */
					hmeas = *(p_queue->p_data +
								p_queue->head);

					/* Clear queued measurement */
					*(p_queue->p_data + p_queue->head) =
							ADC_QUEUE_EMPTY_ELEMENT;

					/* Advance head index */
					ADC_QADVANCE_HEAD(
						p_queue->head, p_queue->size);
					/* Don't look into other queue if
					active handle was found already */
					break;
				};
			}

			/* Check whether a handle to be processed was found in
			the queue */
			if (NULL != hmeas) {
				int ret;

				/* Declare Measurement Unit busy prior to
				starting conversion */
				adc_manager.state = ADC_BUSY;
				/* Set active handle */
				adc_manager.p_active_meas = hmeas;
				/* Store timestamp in instance. This is used to
				prevent settling measurements for getting
				stuck */
				hmeas->dequeue_timestamp = jiffies;

				/* Do debug data logging */
				ADC_DEBUG_DATA_LOG(ADC_DEQUEUE,
						adc_manager.state,
						hmeas,
						hmeas->channel,
						hmeas->latest_result.
						uv,
						hmeas->latest_result.
						na,
						hmeas->settling.
						max_signal_settling_time_ms,
						hmeas->
						dequeue_timestamp);

				/* Set power mode on. If power mode was not
				changed, i.e. power mode was already on, start
				measurement immediately. The measurement will be
				started from the power-up callback otherwise */
				ret = intel_adc_set_power_mode(
				 adc_manager.supported_channels[hmeas->channel].
				  p_hal_if, ADC_HAL_POWER_MODE_ON);

				if (WNOTREQ == ret) {
					/* First try to set up the measurement.
					If set up is not required, start
					measurement immediately.
					Measurement will be started on the
					set-up done callback otherwise */
					if (intel_adc_step_up_meas(hmeas)) {
						/* Start measurement */
						intel_adc_start_meas(hmeas);
					}

				} else if (0 > ret) {
					/* Assign error and return 0uv */
					hmeas->latest_result.error = ret;
					hmeas->latest_result.uv = 0;
					/* Reset active handle */
					adc_manager.p_active_meas = NULL;
					/* ADC is now free for new measurement*/
					/* Declare Measurement Unit ready to do
					another measurement */
					adc_manager.state = ADC_READY;
					/* Provide measurement result back */
					intel_adc_report_meas(hmeas);
					/* Tick STM again to check whether there
					are measurements pending */
					intel_adc_tick_stm(
						ADC_STM_PENDING_CHECK,
						((union adc_stm_event_payload) {
							.p_meas_req = hmeas}));
				}

			} else {
				/* Power down all registered HALs */
				for (i = 0;
					i <
					adc_manager.registered_hals.num;
					i++) {
					(void)
						intel_adc_set_power_mode
						(adc_manager.
						registered_hals.
						p_hal_if[i],
						ADC_HAL_POWER_MODE_OFF);
				}
			}
			break;
		case ADC_STM_MEAS_SET_UP_DONE:
		case ADC_STM_MEAS_DONE:
			/* This event is discarded as it could happen due to
			race condiiton with stopping a measurement */
			break;
		case ADC_STM_SETTLING_RETRY:
		case ADC_STM_POWER_UP_DONE:
		default:
			/* We should never be here. If we are something went
			really wrong! */
			BUG();
		}
		break;
	case ADC_BUSY:
		switch (event) {
		case ADC_STM_REQUEST:
			hmeas = adc_manager.p_active_meas;

			/* Enqueue new measurement request */
			intel_adc_enqueue_meas(payload.p_meas_req);

			/* An OCV (OPEN_CIRCUIT_VOLTAGE) measurement takes
			significantly longer than a vsys_min or vsys_max
			measurement, whose peak detection period is 100ms.
			For an OCV measurement a peak detection time of 500ms is
			foreseen. Since this duration is significant from the
			measurement system point of view, the OCV measurements
			are subject to fail if during peak detection another,
			higher priority measurement request is enqueued. */
			if ((NULL != hmeas) &&
					(ADC_V_BAT_OCV == hmeas->channel)) {
				/* Abort OCV measurement and report error to
				client */

				/* First of all, stop supervisor timer */
				(void)del_timer_sync(
						&adc_manager.supervisory_timer);

				BUG_ON((adc_manager.
					supported_channels[hmeas->channel].
					p_hal_if->set) (ADC_HAL_STOP_MEAS,
					((union adc_hal_set_params) {
						.channel = hmeas->channel})));
				intel_adc_dbg_printk(
					"%s Info: OCV measurement preempted\n",
								__func__);
				/* Assign error and return 0uv */
				hmeas->latest_result.error = -EAGAIN;
				hmeas->latest_result.uv = 0;
				/* Do debug data logging */
				ADC_DEBUG_DATA_LOG
						(ADC_OCV_MEASUREMENT_FAILED,
						adc_manager.state, hmeas,
						hmeas->channel,
						hmeas->latest_result.uv,
						hmeas->latest_result.na,
						hmeas->settling.
						max_signal_settling_time_ms,
						hmeas->autoscaling_on);
				/* Reset active handle */
				adc_manager.p_active_meas = NULL;
				/* ADC is now free for new measurement */
				/* Declare Measurement Unit ready to do another
				measurement */
				adc_manager.state = ADC_READY;
				/* Provide measurement result back */
				intel_adc_report_meas(hmeas);
				/* Tick STM again to check whether there are
				measurements pending */
				intel_adc_tick_stm
					(ADC_STM_PENDING_CHECK,
					((union adc_stm_event_payload) {
							.p_meas_req = hmeas}));
			}
			break;
		case ADC_STM_POWER_UP_DONE:
			hmeas = adc_manager.p_active_meas;
			/* Do debug data logging */
			ADC_DEBUG_DATA_LOG(ADC_PWR_UP_DONE,
						adc_manager.state,
						hmeas,
						hmeas->channel,
						hmeas->latest_result.uv,
						hmeas->latest_result.na,
						hmeas->settling.
						max_signal_settling_time_ms,
						hmeas->autoscaling_on);
			/* Reset active handle */
			/* First try to set up the measurement. If set up is not
			required, start measurement immediately. Measurement
			will be started on the set-up done callback otherwise */
			if (intel_adc_step_up_meas(hmeas)) {
				/* Start measurement */
				intel_adc_start_meas(hmeas);
			}
			break;
		case ADC_STM_MEAS_SET_UP_DONE:
			hmeas = adc_manager.p_active_meas;
			/* Do debug data logging */
			ADC_DEBUG_DATA_LOG(ADC_START_SET_UP_DONE,
						adc_manager.state,
						hmeas,
						hmeas->channel,
						hmeas->latest_result.uv,
						hmeas->latest_result.na,
						hmeas->settling.
						max_signal_settling_time_ms,
						hmeas->autoscaling_on);
			/* Start measurement */
			intel_adc_start_meas(hmeas);
			break;
		case ADC_STM_SETTLING_RETRY:
			hmeas = adc_manager.p_active_meas;
			/* Do debug data logging */
			ADC_DEBUG_DATA_LOG(ADC_MEAS_SETTLING_RETRY,
						adc_manager.state,
						hmeas,
						hmeas->channel,
						hmeas->latest_result.uv,
						hmeas->latest_result.na,
						hmeas->settling.
						max_signal_settling_time_ms,
						hmeas->autoscaling_on);
			/* Re-start active measurement */
			/* First try to set up the measurement. If set up is not
			required, start measurement immediately. Measurement
			will be started on the set-up done callback otherwise */
			if (intel_adc_step_up_meas(hmeas)) {
				/* Start measurement */
				intel_adc_start_meas(hmeas);
			}
			break;
		case ADC_STM_MEAS_DONE:
			hmeas = adc_manager.p_active_meas;
			do_conversion_done = true;

			/* First of all, stop supervisor timer as the
			measurement was done */
			(void)del_timer_sync(&adc_manager.supervisory_timer);

			if (-ERANGE != payload.adc_result) {

				/* Store measurement result in instance for
				later use */
				hmeas->latest_result.uv = payload.adc_result;
				/* In principle the measurement was sucessful.
				Error code might get updated below */
				hmeas->latest_result.error = 0;
			} else {
				/* ADC measurement saturated */
				hmeas->latest_result.error = -ERANGE;
				hmeas->latest_result.uv = 0;
			}

			/* Check whether signal settling feature is enabled for
			this measurement */
			if (ADC_HAL_SIGNAL_SETTLING_DISABLED !=
				hmeas->settling.max_signal_settling_time_ms) {

				int signal_delta;

				/* Need to continue to sample until signal is
				stable */
				/* Calculate signal delta and its absolute
				value */
				signal_delta = hmeas->latest_result.uv -
					hmeas->settling.not_settled_value_uv;

				signal_delta = (signal_delta >= 0 ?
						signal_delta : -signal_delta);
				/* If absolute value is greater then ADC
				accuracy the measurement is not settled */
				if (signal_delta >
					adc_manager.settling.accuracy_uv) {

					/* Calculated elapsed time */
					int elapsed_time_ms =
						jiffies_to_msecs(jiffies -
						 hmeas->dequeue_timestamp);
					/* Do debug data logging */
					ADC_DEBUG_DATA_LOG(
						ADC_MEAS_SETTLING_CHECK,
						adc_manager.state, hmeas,
						hmeas->channel,
						hmeas->latest_result.uv,
						hmeas->latest_result.na,
						hmeas->settling.
						max_signal_settling_time_ms,
						elapsed_time_ms);

					/* Check whether signal is not settling
					after configured maximum settling time
					and throw an error if so */
					if (hmeas->settling.
						max_signal_settling_time_ms <
							elapsed_time_ms) {

						/* Assign error and return 0uv*/
						pr_err("%s ADC settling has failed for channel %d\n",
							__func__,
							(int) hmeas->channel);

						hmeas->latest_result.error =
									-EAGAIN;
						hmeas->latest_result.uv = 0;
						/* Do debug data logging */
						ADC_DEBUG_DATA_LOG(
						 ADC_MEAS_SETTLING_FAILED,
						 adc_manager.state,
						 hmeas,
						 hmeas->channel,
						 hmeas->latest_result.uv,
						 hmeas->latest_result.na,
						 hmeas->settling.
						  max_signal_settling_time_ms,
						 elapsed_time_ms);
					} else {
						/* Prevent doing conversion done
						part */
						do_conversion_done =
							false;
						/* Store unsettled measurement
						value for comparison with next
						measurement */
						hmeas->settling.
						 not_settled_value_uv =
						  hmeas->latest_result.uv;

						/* Need to start another
						measurement as signal is not
						settled yet. Do next measurement
						with some delay to allow for
						signal to settle */
						mod_timer(&adc_manager.
							settling.polling_timer,
							jiffies +
							msecs_to_jiffies(
							 adc_manager.settling.
								polling_ms));
					}

				} else {
					/* Do debug data logging */
					ADC_DEBUG_DATA_LOG
						(ADC_MEAS_SETTLING_DONE,
						adc_manager.state, hmeas,
						hmeas->channel,
						hmeas->latest_result.uv,
						hmeas->latest_result.na,
						hmeas->settling.
						max_signal_settling_time_ms,
						hmeas->autoscaling_on);
				}
			}
			/* Check whether autoscaling is required */
			if ((do_conversion_done)
				&& (hmeas->autoscaling_on)) {
				struct adc_hal_interface *p_hal_if =
				 adc_manager.supported_channels[hmeas->channel].
				  p_hal_if;

				struct intel_adc_hal_channel_data
					*p_channel_data =
					adc_manager.
					supported_channels[hmeas->channel].
					p_channel_data;

				struct adc_hal_current_scaling_info
					*p_current_scaling_info =
					p_hal_if->hw_info->
					p_current_scaling_info;

				bool scaling_required = false;
				int idx;

				/* Do debug data logging */
				ADC_DEBUG_DATA_LOG(ADC_MEAS_AUTOSCALING_CHECK,
						adc_manager.state,
						hmeas,
						hmeas->channel,
						hmeas->latest_result.uv,
						hmeas->latest_result.na,
						hmeas->settling.
						 max_signal_settling_time_ms,
						hmeas->autoscaling_on);

				/* Check whether bias current needs scaling up*/
				if ((hmeas->latest_result.uv <
					p_hal_if->hw_info->
					p_current_scaling_info->
					low_threshold_uv)
					&& (hmeas->latest_result.na <
					p_current_scaling_info->
					p_current_table
					[p_current_scaling_info->
					table_size - 1])) {
					/* Trying to scale up */
					/* Walk up the scaling table. */
				    for (idx = 0; idx <
					p_current_scaling_info->
						table_size - 1; idx++) {

					if ((p_current_scaling_info->
						p_current_table[idx] >=
						p_channel_data->adc_bias_na) &&
						(p_current_scaling_info->
						p_current_table[idx] >
						hmeas->latest_result.na)) {

						scaling_required = true;
						break;
					}
				    }
				} else
				/* Check whether bias current needs scaling
				down */
				if ((hmeas->latest_result.uv >
						p_hal_if->hw_info->
						p_current_scaling_info->
						high_threshold_uv)
						&& (hmeas->
							latest_result.
							na >
							p_channel_data->
							adc_bias_na)) {
					/* Trying to scale down */
					/* Walk down the scaling table. */
				    for (idx = p_current_scaling_info->
						table_size - 1; idx >= 0;
									idx--) {
					if ((p_current_scaling_info->
						p_current_table[idx] >=
						 p_channel_data->adc_bias_na) &&
						 (p_current_scaling_info->
						 p_current_table[idx] <
						 hmeas->latest_result.na)) {

						scaling_required = true;
						break;
					}
				    }
				}
				if (scaling_required) {
					/* Prevent doing conversion done part */
					do_conversion_done = false;
					/* Refresh dequeue timestampt as
					settling would have to be restarted */
					hmeas->dequeue_timestamp =
						jiffies;
					/* Scaled bias current */
					hmeas->latest_result.na =
						p_current_scaling_info->
						p_current_table[idx];
					/* Do debug data logging */
					ADC_DEBUG_DATA_LOG
						(ADC_MEAS_AUTOSCALING_RETRY,
						adc_manager.state, hmeas,
						hmeas->channel,
						hmeas->latest_result.uv,
						hmeas->latest_result.na,
						hmeas->settling.
						max_signal_settling_time_ms,
						hmeas->autoscaling_on);
					/* Re-start active measurement */
					/* First try to set up the measurement.
					If set up is not required, start
					measurement immediately.
					Measurement will be started on the
					set-up done callback otherwise */
					if (intel_adc_step_up_meas
						(hmeas)) {
						/* Start measurement */
						intel_adc_start_meas
							(hmeas);
					}
				} else {
					/* Do debug data logging */
					ADC_DEBUG_DATA_LOG
						(ADC_MEAS_AUTOSCALING_DONE,
						adc_manager.state, hmeas,
						hmeas->channel,
						hmeas->latest_result.uv,
						hmeas->latest_result.na,
						hmeas->settling.
						max_signal_settling_time_ms,
						hmeas->autoscaling_on);
				}

			}

			if (do_conversion_done) {
				/* Do debug data logging */
				ADC_DEBUG_DATA_LOG(ADC_MEAS_DONE,
						adc_manager.state,
						hmeas,
						hmeas->channel,
						hmeas->latest_result.uv,
						hmeas->latest_result.na,
						hmeas->settling.
						 max_signal_settling_time_ms,
						hmeas->autoscaling_on);

				/* Reset active handle */
				adc_manager.p_active_meas = NULL;
				/* ADC is now free for new measurement */
				/* Declare Measurement Unit ready to do another
				measurement */
				adc_manager.state = ADC_READY;
				/* Provide measurement result back */
				intel_adc_report_meas(hmeas);
				/* Tick STM machine again to check whether there
				measurements pending */
				intel_adc_tick_stm
					(ADC_STM_PENDING_CHECK,
					((union adc_stm_event_payload) {
					.p_meas_req = hmeas}));
			}
			break;
		case ADC_STM_PENDING_CHECK:
		default:
			/* We should never be here. If we are something went
			really wrong! */
			BUG();
		}
		break;
	case ADC_NOT_INITIALISED:
		/* This ADC STM tick was called before its thread. This should
		never happen */
	default:
		/* We should never be here. If we are something went really
		wrong! */
		BUG();
	}
}

/**
 * intel_adc_thread() - It implements the kernel thread from where ADC high
 * level state machine is run.
 *
 */
static int intel_adc_thread(void *dummy)
{
	int i;
	struct adc_meas_queue *p_queue;
	struct adc_meas_instance **p_qdata;

	/* Unsed parameter */
	(void)dummy;

	/* Initialise ADC driver static resources */
	/* Set up supervisory timer callback */
	setup_timer(&adc_manager.supervisory_timer, intel_adc_stuck_measurement,
			0);
	/* Set up settling timer callback */
	setup_timer(&adc_manager.settling.polling_timer,
			intel_adc_start_settling_meas, 0);

	wake_lock_init(&adc_manager.measurement_wakelock,
			WAKE_LOCK_SUSPEND, "adc_meas_lock");

	/* Initialise both measurement queues */
	p_queue = &adc_manager.queue[ADC_NORMAL_PRIO_QUEUE_INDEX];
	p_queue->head = 0;
	p_queue->tail = 0;
	p_queue->size = ARRAY_SIZE(adc_manager.normal_prio_queue);
	p_queue->p_data = adc_manager.normal_prio_queue;
	for (i = 0, p_qdata = p_queue->p_data; i < p_queue->size;
		i++, p_qdata++) {
		*p_qdata = ADC_QUEUE_EMPTY_ELEMENT;
	}
	p_queue = &adc_manager.queue[ADC_LOW_PRIO_QUEUE_INDEX];
	p_queue->head = 0;
	p_queue->tail = 0;
	p_queue->size = ARRAY_SIZE(adc_manager.low_prio_queue);
	p_queue->p_data = adc_manager.low_prio_queue;
	for (i = 0, p_qdata = p_queue->p_data; i < p_queue->size;
		i++, p_qdata++) {
		*p_qdata = ADC_QUEUE_EMPTY_ELEMENT;
	}

	/* ADC driver is not ready to start processing measurements */
	adc_manager.state = ADC_READY;

	while (adc_manager.keep_running) {
		struct adc_stm_mess mess;

		/* Try to fetch a message. If no data is fetched, retry later */
		if (0 == intel_adc_fetch_mess(&mess)) {
			set_current_state(TASK_INTERRUPTIBLE);
			if (0 == intel_adc_fetch_mess(&mess)) {
				schedule();
				continue;
			} else {
				set_current_state(TASK_RUNNING);
			}
		}
		/* Call ADC STM to message payload */
		intel_adc_tick_stm(mess.event, mess.payload);
	}			/* while - dequeuing requests */

	/* Release ADC driver static resources */
	(void)del_timer_sync(&adc_manager.supervisory_timer);
	(void)del_timer_sync(&adc_manager.settling.polling_timer);

	return 0;
}

/**
 * intel_adc_read_raw - Entry point to start doing a measurement. This function
 * is exported through the IIO bus.
 *
 * @iiodev:	IIO device structure.
 * @chan:	Channel to measure.
 * @p_val:	Pointer to pass back measured value.
 * @p_val2:	Pointer to pass back bias current used for measured value.
 * @mask:	Channel attributes. "0" means read raw.
 */
static int intel_adc_read_raw(struct iio_dev *iiodev,
				struct iio_chan_spec const *chan,
				int *p_val, int *p_val2, long mask)
{
	int ret = -EINVAL;

	BUG_ON(chan == NULL);
	WARN_ON(chan->channel >= ADC_MAX_NO_OF_CHANNELS);

	if (chan->channel >= ADC_MAX_NO_OF_CHANNELS)
		return -ECHRNG;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:	/* Channel read */{
		struct adc_meas_instance meas_req;
		struct adc_stm_mess mess;

		/* Initialise measurement request data structure */
		meas_req.channel = chan->channel;
		meas_req.settling.max_signal_settling_time_ms =
				adc_manager.supported_channels[chan->channel]
				.p_channel_data->max_signal_settling_time_ms;

		meas_req.autoscaling_on =
				adc_manager.supported_channels[chan->channel]
				.p_channel_data->autoscaling_on;

		meas_req.latest_result.na =
				adc_manager.supported_channels[chan->channel]
				.latest_adc_bias_na;

		init_completion(&meas_req.done);

		/* Add ADC STM message to the queue. */
		mess.event = ADC_STM_REQUEST;
		mess.payload.p_meas_req = &meas_req;
		intel_adc_add_mess(&mess);

		/* Wait for the measurement to complete */
		wait_for_completion(&meas_req.done);

		/* Measurement is done now. Copy measurement results */
		*p_val = meas_req.latest_result.uv;
		*p_val2 = meas_req.latest_result.na;
		adc_manager.supported_channels[chan->channel].
				latest_adc_bias_na = meas_req.latest_result.na;
		ret = meas_req.latest_result.error;
		if (0 == ret)
			ret = IIO_VAL_INT;

			intel_adc_dbg_printk(
				"Returning uv=%d, na=%d, error=%d\n",
						*p_val, *p_val2, ret);
	}
		break;
	default:
		/* This is not supposed to happen */
		BUG();
		break;
	}

	return ret;
}

static const struct iio_info iio_info_intel_adc = {
	.driver_module = THIS_MODULE,
	.read_raw = &intel_adc_read_raw,
};

/**
 * intel_adc_iio_registration() - Register the relevant IIO devices
 * for a given HAL.
 *
 * @pdev			[in] HAL platform data.
 *
 * Returns Error number; may be tested as boolean with 0=success, other=fail.
 * Some errors may need re-trial.
 */
static int intel_adc_iio_registration(struct device *pdev)
{
	struct intel_adc_hal_pdata *pdata =
		(struct intel_adc_hal_pdata *)pdev->platform_data;
	struct adc_hal_channel_info *p_channel_info =
		(struct adc_hal_channel_info *)&pdata->channel_info;
	struct intel_adc_hal_channel_data *p_channel_data;
	int ret = -EINVAL;
	enum adc_channel chan;
	struct iio_chan_spec *p_adc_chan_spec;
	struct iio_map *p_iio_map;

	WARN_ON(!pdata);

	dev_err(pdev, "Probing the ADC\n");
	dev_set_drvdata(pdev, &adc_manager);

	pdata->p_adc_iio_dev = iio_device_alloc(0);
	if (pdata->p_adc_iio_dev == NULL) {
		ret = PTR_ERR(pdata->p_adc_iio_dev);
		dev_err(pdev, "Error allocating IIO device: %d\n", ret);
		goto error_device_alloc;
	}

	/* Dynamically create the list of channels. Allocate memory to hold
	channel specs and mapping */
	pdata->p_adc_chan_spec = kcalloc(p_channel_info->nchan,
						sizeof(struct iio_chan_spec),
						GFP_KERNEL);
	if (pdata->p_adc_chan_spec == NULL) {
		ret = PTR_ERR(pdata->p_adc_chan_spec);
		dev_err(pdev, "Error allocating IIO channel specs: %d\n", ret);
		goto error_adc_chan_alloc;
	}
	/* The IIO map is a NULL terminated list, allocate 1 extra structure
	element. The last element is automatically set to NULL by kcalloc() */
	pdata->p_iio_map = kcalloc(p_channel_info->nchan + 1,
					sizeof(struct iio_map), GFP_KERNEL);
	if (pdata->p_iio_map == NULL) {
		ret = PTR_ERR(pdata->p_iio_map);
		dev_err(pdev, "Error allocating IIO channel map: %d\n", ret);
		goto error_iio_map_alloc;
	}
	/* Finally initialise the list of channels */
	p_adc_chan_spec = &pdata->p_adc_chan_spec[0];
	p_iio_map = &pdata->p_iio_map[0];
	p_channel_data = &p_channel_info->p_data[0];
	for (chan = 0; chan < p_channel_info->nchan; chan++) {
		/* Initialise channel specs */
		p_adc_chan_spec->type = p_channel_data->iio_type;
		p_adc_chan_spec->indexed = false;
		p_adc_chan_spec->output  = true;
		p_adc_chan_spec->channel = p_channel_data->log_channel_id;
		p_adc_chan_spec->extend_name = convert_to_iio_node_name(
				p_channel_data->consumer_channel);
		p_adc_chan_spec->datasheet_name	=
					p_channel_data->datasheet_name;
		p_adc_chan_spec->info_mask_separate = BIT(IIO_CHAN_INFO_RAW);
		p_adc_chan_spec++;
		/* Initialise channel map */
		p_iio_map->consumer_dev_name =
			p_channel_data->consumer_dev_name;
		p_iio_map->consumer_channel = p_channel_data->consumer_channel;
		p_iio_map->adc_channel_label = p_channel_data->datasheet_name;
		p_iio_map++;
		/* Increment device channel data pointer */
		p_channel_data++;
	}

	pdata->p_adc_iio_dev->name = INTEL_ADC_DEV_NAME;
	pdata->p_adc_iio_dev->channels = pdata->p_adc_chan_spec;
	pdata->p_adc_iio_dev->num_channels = p_channel_info->nchan;
	pdata->p_adc_iio_dev->info = &iio_info_intel_adc;
	pdata->p_adc_iio_dev->modes = INDIO_DIRECT_MODE;
	pdata->p_adc_iio_dev->dev.parent = pdev;

	/* Register IIO devices */
	ret = iio_device_register(pdata->p_adc_iio_dev);
	if (0 != ret)
		goto error_iio_dev_reg;
	/* Map IIO devices to inkernel channels */
	ret = iio_map_array_register(pdata->p_adc_iio_dev, pdata->p_iio_map);
	if (0 == ret)
		return 0;

	iio_device_unregister(pdata->p_adc_iio_dev);

error_iio_dev_reg:
	kfree(pdata->p_iio_map);

error_iio_map_alloc:
	kfree(pdata->p_adc_chan_spec);

error_adc_chan_alloc:
	iio_device_free(pdata->p_adc_iio_dev);

error_device_alloc:
	pr_err("ADC iio registration failed with error code %d\n", ret);
	return ret;
}

/**
 * intel_adc_iio_registration() - De-register the relevant IIO devices
 * for a given HAL.
 *
 * @pdev			[in] HAL platform data.
 *
 * Returns Error number; may be tested as boolean with 0=success, other=fail.
 * Some errors may need re-trial.
 */
static void intel_adc_iio_deregistration(struct device *pdev)
{
	struct intel_adc_hal_pdata *pdata =
		(struct intel_adc_hal_pdata *)pdev->platform_data;
	iio_map_array_unregister(pdata->p_adc_iio_dev);
	iio_device_unregister(pdata->p_adc_iio_dev);
	kfree(pdata->p_iio_map);
	kfree(pdata->p_adc_chan_spec);
	iio_device_free(pdata->p_adc_iio_dev);
}

static DEFINE_SPINLOCK(hal_list_lock);

/**
 * adc_register_hal() - This is the function that the HALs call to register
 * themselves and declare what channels they support. When all required channels
 * are supported, this function will register the adc and sensor device to have
 * them probed by their respective drivers.
 *
 * @p_adc_hal_interface:	A pointer to the HAL interface descriptor.
 * @hal_device:			The device that the HAL is built around.
 * @p_adc_driver_cb_function:	A function that is called when measurements are
 *				done.
 */
int adc_register_hal(struct adc_hal_interface *p_adc_hal_interface,
			struct device *hal_device,
			adc_hal_cb_t *p_adc_driver_cb_function)
{
	int ret = -EINVAL, chan;
	struct intel_adc_hal_pdata *pdata =
		(struct intel_adc_hal_pdata *)hal_device->platform_data;
	struct adc_hal_channel_info *p_channel_info =
		(struct adc_hal_channel_info *)&pdata->channel_info;
	struct adc_manager_data *st = &adc_manager;

	spin_lock(&hal_list_lock);

	pr_info("Registering %s, %d channels available.\n",
		p_adc_hal_interface->hw_info->p_hw_name, p_channel_info->nchan);

	/* Check that provided properties aren't already registered. */
	for (chan = 0; chan < p_channel_info->nchan; chan++) {
		pr_info("\tchan=%d, id=%d\n", chan,
			p_channel_info->p_data[chan].log_channel_id);

		if (st->supported_channels[p_channel_info->p_data[chan].
					log_channel_id].p_hal_if) {
			pr_err("\tChannel %d is already registered! Aborting...\n",
				 p_channel_info->p_data[chan].log_channel_id);

			ret = -EEXIST;
			goto error_free_channels;
		}
		st->supported_channels[p_channel_info->p_data[chan].
					log_channel_id].p_hal_if =
						p_adc_hal_interface;

		st->supported_channels[p_channel_info->p_data[chan].
					log_channel_id].p_channel_data =
						&p_channel_info->p_data[chan];

		st->supported_channels[p_channel_info->p_data[chan].
					log_channel_id].latest_adc_bias_na =
						p_channel_info->p_data[chan].
								adc_bias_na;
	}

	/*
	* If all HALs have been registered, the ADC layer can be activated
	* by registering its platform data. The sensor device is also added
	* to enforce proper ordering.
	*/
	pr_info(
		"All HALs were registered, probing the abstraction layer...\n");

	/* Update the state with relevant info from the platform data */
	st->settling.polling_ms = pdata->settling_interval_ms;
	st->settling.accuracy_uv = pdata->settling_accuracy_uv;

	*p_adc_driver_cb_function = intel_adc_hal_event_callback;

	/* Account for new registered HAL */
	st->registered_hals.p_hal_if[st->registered_hals.num] =
		p_adc_hal_interface;
	st->registered_hals.num++;

	spin_unlock(&hal_list_lock);

	ret = intel_adc_iio_registration(hal_device);

	if (0 == ret)
		return 0;

	spin_lock(&hal_list_lock);
error_free_channels:
	while (chan--) {
		st->supported_channels[p_channel_info->p_data[chan].
					log_channel_id].p_hal_if = NULL;
		st->supported_channels[p_channel_info->p_data[chan].
					log_channel_id].p_channel_data = NULL;
	}
	spin_unlock(&hal_list_lock);

	dev_err(hal_device, "Hal registration failed with error code %d\n",
		ret);
	return ret;
}
EXPORT_SYMBOL(adc_register_hal);

/**
 * adc_unregister_hal() - Deregister the supported operations and capabilities
 * of a HAL with the ADC Driver.
 *
 * @p_adc_hal_interface		[in] Funtions, channels and HW specific
 *				information.
 * @pdata			[in] HAL platform data.
 */
void adc_unregister_hal(struct adc_hal_interface *p_adc_hal_interface,
			struct device *hal_device)
{
	int i;
	struct adc_manager_data *st = &adc_manager;

	spin_lock(&hal_list_lock);

	for (i = 0; i < ARRAY_SIZE(st->supported_channels); i++) {
		/* The assumption here is that p_adc_hal_interface is a static
		alloc. */
		if ((st->supported_channels[i].p_hal_if) ==
							p_adc_hal_interface) {
			st->supported_channels[i].p_hal_if = NULL;
			st->supported_channels[i].p_channel_data = NULL;
		}
	}

	/* Account for HAL unregistered */
	for (i = 0; i < st->registered_hals.num; i++) {
		if (st->registered_hals.p_hal_if[i] == p_adc_hal_interface)
			break;
	}
	st->registered_hals.num--;
	if (st->registered_hals.num != i) {
		st->registered_hals.p_hal_if[i] =
			st->registered_hals.p_hal_if[st->registered_hals.num];
		st->registered_hals.p_hal_if[st->registered_hals.num] = NULL;
	} else {
		st->registered_hals.p_hal_if[i] = NULL;
	}

	spin_unlock(&hal_list_lock);

	intel_adc_iio_deregistration(hal_device);
}
EXPORT_SYMBOL(adc_unregister_hal);

static int __init intel_adc_probe(struct platform_device *p_platform_dev)
{
	struct task_struct *thread;
	struct adc_manager_data *st = &adc_manager;
	struct dentry *dir = debugfs_create_dir("meas", NULL);
	struct dentry *d;

	int ret = -EINVAL;
	spin_lock_init(&adc_debug_data.lock);
	spin_lock_init(&adc_manager.lock);

	ret = kfifo_alloc(&st->adc_stm_fifo, ADC_STM_FIFO_DEPTH, GFP_KERNEL);
	if (ret)
		goto error_exit;

	thread = kthread_create(intel_adc_thread, NULL, "adc-thread");

	WARN_ON(!thread);

	/* Create debugfs entries for all state fields */
	if (!IS_ERR(dir)) {
		d = debugfs_create_u32("meas_scheduled_count", S_IRUGO, dir,
				&adc_manager.stats.meas_scheduled_count);
		d = debugfs_create_u32("meas_performed_count", S_IRUGO, dir,
				&adc_manager.stats.meas_performed_count);
		d = debugfs_create_u32("ocv_performed_count", S_IRUGO, dir,
				&adc_manager.stats.ocv_performed_count);
		d = debugfs_create_u32("meas_again_error_cnt", S_IRUGO, dir,
				&adc_manager.stats.meas_again_error_cnt);
		d = debugfs_create_u32("meas_io_error_cnt", S_IRUGO, dir,
				&adc_manager.stats.meas_io_error_cnt);
	}

	if (thread) {
		spin_lock_init(&st->qlock);
		st->adc_thread = thread;
		wake_up_process(thread);
	} else {
		kfifo_free(&st->adc_stm_fifo);
	}

	/* Do debug data logging */
	ADC_DEBUG_DATA_LOG(ADC_INIT, adc_manager.state, 0, ret, 0, 0, 0, 0);

error_exit:
	return ret;
}

static int __exit intel_adc_remove(struct platform_device *p_platform_dev)
{
	struct adc_manager_data *st = &adc_manager;

	st->keep_running = false;
	kfifo_free(&st->adc_stm_fifo);

	/* Do debug data logging */
	ADC_DEBUG_DATA_LOG(ADC_DE_INIT, adc_manager.state, 0, 0, 0, 0, 0, 0);

	return 0;

}

/**
 * intel_adc_suspend() - Called when the system is attempting to suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int intel_adc_suspend(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Protect critical section when testing and modifying device state
	data */
	spin_lock(&adc_manager.lock);

	/* No measurement ongoing - allow suspend. */
	adc_manager.suspended = true;

	/* End of critical section */
	spin_unlock(&adc_manager.lock);

	/* Do debug data logging */
	ADC_DEBUG_DATA_LOG(ADC_SUSPEND,
				adc_manager.state,
				0,
				NULL, 0,
				0, 0, 0);

	return 0;
}

/**
 * intel_adc_resume() - Called when the system is resuming from suspend.
 * @dev		[in] Pointer to the device.(not used)
 * returns	0
 */
static int intel_adc_resume(struct device *dev)
{
	/* Unused parameter */
	(void)dev;

	/* Update suspend flag used to tell whether operations on device are
	allowed */
	adc_manager.suspended = false;

	/* Do debug data logging */
	ADC_DEBUG_DATA_LOG(ADC_RESUME,
				adc_manager.state,
				0,
				NULL, 0,
				0, 0, 0);

	return 0;
}


const struct dev_pm_ops intel_adc_pm = {
	.suspend = intel_adc_suspend,
	.resume = intel_adc_resume,
};

static const struct of_device_id adc_of_match[] = {
	{
	 .compatible = "intel,adc",
	 },
	{}
};

MODULE_DEVICE_TABLE(of, adc_of_match);

static struct platform_driver intel_adc_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(adc_of_match),
		.pm = &intel_adc_pm,
	},
	.probe = intel_adc_probe,
	.remove = intel_adc_remove,
};

/*
 * intel_adc_init - Module intialisation function.
 */
static int __init intel_adc_init(void)
{
	return platform_driver_register(&intel_adc_driver);
}

/*
 * intel_adc_exit - Module cleanup function.
 */
static void __exit intel_adc_exit(void)
{
	return platform_driver_unregister(&intel_adc_driver);
}

module_init(intel_adc_init);
module_exit(intel_adc_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ADC abstraction layer for Intel family.");
