/* ----------------------------------------------------------------------------
   Copyright (C) 2014 Intel Mobile Communications GmbH

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

  ---------------------------------------------------------------------------*/
/*
 * NOTES:
 * 1) This source file is included in guests including Linux and purposely
 * kept in line with Linux kernel coding guidelines for ease of portability and
 * maintainability. See linux/Documentation/CodingStyle for coding guidelines.
 * Please check all further changes are compliant with Linux coding standard
 * and clear all commands reported by Linux checkpatch.pl
 * Use command: linux/scripts/checkpatch.pl -f <filename>
 * Clear ALL warnings and errors reported.
 *
 * 2) Use only C99 fixed width types for definition as this header needs to be
 * both 32bit/64bit portable.
 * Avoid the use of pointers/enum in structure as that make the structure
 * variable size based on 32/64bit toolchain used.
*/

/* For inclusion by Guest VMs only! */

#ifndef _MV_SVC_HYPERCALLS_H
#define _MV_SVC_HYPERCALLS_H

#include "pal_shared_data.h"

#define VMM_PAL_REBOOT_ACTION_NONE      0
#define VMM_PAL_REBOOT_ACTION_HALT      1
#define VMM_PAL_REBOOT_ACTION_RESET     2

#define PINCTRL_NULL 0
#define PINCTRL_ABB_IDX(x) (x | (0x8000))

/** @typedef vmm_platform_service_type
 *
 *  @brief enumeration platform dependent service type
 *  which is used in vmcall handlers
*/
enum mv_service_type {
	DUMMY_SERIVCE   = 0,
	PINCTRL_SERVICE,
	PM_SERVICE,
	SYS_EXCEPTION_SERVICE,
	CD_SERVICE,
	SOCWATCH_SERVICE,
	SEP_SERVICE,
	WATCHDOG_SERVICE,
	SYSPROF_SERVICE,
	VMM_REG_RD_SERVICE,
	VMM_REG_WR_SERVICE,
	VMM_REG_WR_ONLY_SERVICE,
	VMM_PMIC_REG_ACCESS_SERVICE,
	VMM_VTIMER_START_SERVICE,
	VMM_VTIMER_STOP_SERVICE,
	VMM_VTIMER_GET_FREQ_SERVICE,
	RTC_SERVICE,
	VMM_SPCU_THERMAL_SERVICE,
	VMM_PWM_SERVICE
};


/**
  @typedef pinctrl_service_op_code
  @brief   enumeration containing the operation of pinctrl service
**/
enum pinctrl_service_op_code {
	PINCTRL_OPEN = 0,
	PINCTRL_CONTROL,
	PINCTRL_GET_PCL,
	PINCTRL_SET_PCL,
	PINCTRL_GET_PIN,
	PINCTRL_CLOSE
};

/**
  @typedef cd_op_code
  @brief   enumeration containing the operation of core dump service
**/
enum cd_op_code {
	CD_SET_CONFIG = 0,
	CD_ADD_REGION
};

/**
  @typedef cd_op_code
  @brief   enumeration containing the operation of system exception service
**/
enum sys_exception_op_code {
	SYS_EXCEPTION_STOP = 0,
	SYS_EXCEPTION_DUMP,
};

/**
  @typedef socwatch_op_code
  @brief   enumeration containing the operation of socwatch service
**/
enum socwatch_op_code {
	SOCWATCH_SET_CONFIG = 0,
	SOCWATCH_RUN_CONTROL
};

/**
  @typedef socwatch_buffer_status
  @brief   defines the buffer status
**/
enum socwatch_buffer_status {
	SOCWATCH_BUFFER_INVALID = 0,
	SOCWATCH_BUFFER_VALID,
	SOCWATCH_BUFFER_CONSUMED,
};

/**
  @brief socwatch buffer information
**/
struct socwatch_buffer_info {
	uint32_t num_buffers_allocated;
	/** buffer size in Kbytes (1024)
	 * 16 means a 16384 byte buffer */
	uint32_t buffer_length;
	/** struct user must check for NULL values
	 * when num_buffers_allocated is less than 4,
	 * some array values will be NULL
	 * buffer_start values must be 64bit physical addresses
	 * pointing to contiguous memory;
	 * use the low 32 bits if 32 bit addresses are  used */
	uint64_t buffer_start[8];
	/** 64bit physical address of buffer to be processed
	 * use the current processor index
	 * to determine which array index to use */
	uint64_t buffer_delivered[4];
	/** the size of the data contained within the buffer
	 * when it is delivered from the VMM to the Linux kernel driver */
	uint32_t buffer_data_size[4];
	/**
	 0: buffer is invalid
	 1: buffer is valid and not yet consumed
	 2: buffer has been consumed
	*/
	uint32_t buffer_status[4];
};

/*
 * We are setting a 2 byte boundary for PWCollector_msg because
 * that is how it is implemented in SoCWatch for all the other systems
 */
#pragma pack(push)	/* Store current alignment */
#pragma pack(2)		/* Set new alignment -- 2 byte boundaries */

/**
 * The main PWCollector_msg structure.
 * This struct is used by SoCWatch.
 * Changes to this struct without corresponging changes
 * in the SoCWatch driver/binary will break SoCWatch.
 */
struct PWCollector_msg {
	/** STM based (26MHz) count. TSC of message. */
	uint64_t tsc;
	/** length of payload message in bytes (not including this header)
	 * represented by p_data. */
	uint16_t data_len;
	 /** physical core number (0 or 1) */
	uint16_t cpuidx;
	/** type of payload encoded by 'p_data': one of 'sofia_msg_t' enum */
	uint8_t data_type;
	/** The compiler would have inserted it anyway! */
	uint8_t padding;
	/** For SW1 file, this is the payload: one of *_msg_t corresponding to
	 * data_type (inline memory). For SoCWatch internal data, this field is
	 * a pointer to the non-contiguous payload memory (not inline). */
	uint64_t p_data;
};

#define PW_MSG_HEADER_SIZE (sizeof(struct PWCollector_msg) - sizeof(u64))

#pragma pack(pop)	/* Restore previous alignment */

/**
  @typedef sep_op_code
  @brief   enumeration containing the operation of sep service
**/
enum sep_op_code {
	SEP_CONFIG = 0,
	SEP_SET_GUEST_CONTEXT,
	SEP_RUN_CONTROL,
	SEP_READ_COUNTER_LIST,
	SEP_WRITE_COUNTER_LIST
};

struct sep_packet {
	/** timestamp */
	uint64_t timestamp;
	/** context where PMI is triggered */
	uint32_t os_id;
	/** instruction pointer */
	uint32_t rip;
	/** the task id */
	uint32_t task_id;
	/** the task name */
	char task[16];
} __packed;

struct sep_counter {
	/** counter to read/write; last entry will have value of -1 */
	int32_t msr_id;
	/** value to write or location to write into */
	uint64_t value;
} __packed;


/**
  @typedef watchdog_op_code
  @brief   enumeration containing the operation of watchdog service
**/
enum watchdog_op_code {
	WATCHDOG_ENABLE = 0,
	WATCHDOG_DISABLE,
	WATCHDOG_PET
};

/**
  @typedef rtc_opcode
  @brief   enumeration containing the operation of watchdog service
**/
enum rtc_opcode {
	RTC_GET_DATETIME = 0,
	RTC_SET_DATETIME,
	RTC_GET_DATETIME_US,
	RTC_GET_ALARM,
	RTC_SET_ALARM,
	RTC_CLEAR_ALARM,
};

/**
Data structure for keeping date and time\n
*/
typedef struct rtc_datetime_shared_data pal_rtc_datetime;

uint32_t mv_svc_rtc_set_datetime(pal_rtc_datetime *rtc_datetime);
uint32_t mv_svc_rtc_get_datetime(pal_rtc_datetime *rtc_datetime);
uint32_t mv_svc_rtc_get_time_us(uint64_t *rtc_us_time);
uint32_t mv_svc_rtc_clear_alarm(void);
uint32_t mv_svc_rtc_set_alarm(pal_rtc_datetime *rtc_datetime);
uint32_t mv_svc_rtc_get_alarm(pal_rtc_datetime *rtc_datetime);

/**
  @typedef vmm_pinctrl_result
  @brief   enumeration containing return result types
**/
enum vmm_pinctrl_result {
	VMM_PINCTRL_RESULT_SUCCESS = 0,
	VMM_PINCTRL_RESULT_FAILURE = -1
};

/**
  @typedef port configuration operation
  @brief   enumeration containing return result types
**/
enum pinctrl_oper {
	VMM_PINCTRL_OPER_ACTIVATE          = 0,
	VMM_PINCTRL_OPER_SLEEP             = 1,
	VMM_PINCTRL_OPER_DEACTIVATE        = 2
};

/**
  @typedef sysprof_op_code
  @brief   enumeration containing the operation of System Profiling service
**/
enum sysprof_op_code {
	SYSPROF_TRACE_START = 0,
	SYSPROF_TRACE_STOP = 1,
	SYSPROF_TASK_LIST_REQ = 2,
	SYSPROF_ENTITY_INFO = 3,
	SYSPROF_ENTITY_SENT = 4,
	SYSPROF_IRQ_LIST_REQ = 5
};

/**
  @typedef pmic_reg_access_op_code
  @brief   enumeration containing the operation of PMIC register access service
**/
enum pmic_reg_access_op_code {
	PMIC_REG_WRITE = 0,
	PMIC_REG_READ
};

/**
  @typedef spcu_therml_service_op_code
  @brief   enumeration containing the operation of spcu thermal service
**/
enum spcu_thermal_service_op_code {
	SPCU_THERMAL_REQUEST = 0,
	SPCU_THERMAL_ENABLE_INTR,
	SPCU_THERMAL_DISABLE_INTR,
};

/**
  @typedef pwm_op_code
  @brief   enumeration containing the operation of PWM service
**/
enum pwm_op_code {
	PWM_CONFIG = 0,
	PWM_ENABLE,
	PWM_DISABLE,
};

/**
 @brief  MobileVisor platform pin control service
 @brief  Use PINCTRL_ABB_IDX(agr1) for AGOLD PCL
 @param  pinctrl_opcode  enumerate operation service
 @param  arg1  physical index number of PCL (GET/ SET)
 @param  arg1  symbolic pad names (CONTROL)
 @param  arg2  register value to be written (SET)
 @param  arg2  operation mode (CONTROL)
 @param  arg3  return register value read (GET)
 @return Return 0 if success, -1 otherwise
**/
uint32_t mv_svc_pinctrl_service(
			uint32_t pinctrl_opcode,
			uint32_t arg1, uint32_t arg2, uint32_t *arg3);

/** @brief PM control
 *
 */
uint32_t mv_svc_pm_control(uint32_t pm_opcode, uint32_t arg1,
			uint32_t arg2, uint32_t arg3);
void mv_svc_modem_2g_sleep_time(uint32_t *pal_shared_mem_p, uint32_t duration);
void mv_svc_modem_3g_sleep_time(uint32_t *pal_shared_mem_p, uint32_t duration);
void mv_svc_modem_next_timeout(uint32_t *pal_shared_mem_p, uint32_t duration);
void mv_svc_vm_enter_idle(uint32_t *pal_shared_mem_p, uint32_t target_power_state);

/**
 @brief  MobileVisor    core dump service
 @param  cd_opcode core dump operation code
 @param  cd_data        physical address of share data
**/
void mv_svc_cd_service(uint32_t cd_opcode, void *cd_data);

/**
 @brief  MobileVisor system exception service
 @param  opcode  system exception operation code
 @param  trap_data      share data
**/
void mv_svc_sys_exception(uint32_t opcode, void *trap_data);

/**
 @brief  MobileVisor socwatch configuration
 @param  events socwatch events to be enabled
 @param  buffer_info phy address to buffer information
 @return 0 if success
**/
uint32_t mv_svc_socwatch_config(uint32_t events,
			struct socwatch_buffer_info *buffer_info);

/**
 @brief  MobileVisor socwatch run control
 @param  run_control 0 to stop, 1 to start
 @return for start, 0 if success. For stop, returns number of dropped packets
**/
uint32_t mv_svc_socwatch_run_control(uint32_t run_control);

/**
 @brief  MobileVisor sep configuration
 @param  p_packet
 @return 0 if success
**/
uint32_t mv_svc_sep_config(struct sep_packet *p_packet);

/**
 @brief  MobileVisor sep config guest context
 @param  task_pointer linear address of the guest's current task pointer
 @param  task_name_offset offset of the current task name from the task pointer
 @return 0 if success.
**/
uint32_t mv_svc_sep_guest_context(uint32_t task_pointer,
				uint32_t task_name_offset);

/**
 @brief  MobileVisor sep run control
 @param  run_control 0 to stop, 1 to start
 @return 0 if success.
**/
uint32_t mv_svc_sep_run_control(uint32_t run_control);

/**
 @brief  MobileVisor sep read counter control
 @param  buffer_pointer linear address of the guest's current list of
			counters to read
 @return 0 if success.
**/
uint32_t mv_svc_sep_read_counters(struct sep_counter *buffer_pointer);

/**
 @brief  MobileVisor sep write counter control
 @param  buffer_pointer linear address of the guest's current list of
			counters and values to write
 @return 0 if success.
**/
uint32_t mv_svc_sep_write_counters(struct sep_counter *buffer_pointer);

/**
 @brief  Enable watchdog with the timeout specified
 @param  timeout period (in seconds) that the watchdog must be serviced
 @return 0 if success.
**/
uint32_t mv_svc_watchdog_enable(uint32_t timeout);

/**
 @brief  Pet the watchdog
 @return 0 if success.
**/
uint32_t mv_svc_watchdog_pet(void);

/**
 @brief  Disable the watchdog
 @return 0 if success.
**/
uint32_t mv_svc_watchdog_disable(void);

/**
 @brief  MobileVisor System Profiling service
 @param  opcode  system profiling operation code (enum sysprof_op_code)
 @param  swt_paddr	physical address for software trace channels
			(one per physical core)
 @param  mask		mask for the classes of events
**/
void mv_svc_sysprof_service(uint32_t opcode, uint32_t *swt_paddr,
			uint32_t mask);

/**
 @brief  MobileVisor System Profiling service to start trace
 @param  swt_paddr	physical address for software trace channels
			(one per physical core)
 @param  mask		mask for the classes of events
**/
void mv_svc_sysprof_start_trace(uint32_t *swt_paddr, uint32_t mask);

/**
 @brief  MobileVisor System Profiling service to stop trace
**/
void mv_svc_sysprof_stop_trace(void);

/**
 @return struct pal_shared_data * physical address to per VCPU-Mobilevisor
				  platform shared data structure
**/
struct pal_shared_data *mv_svc_get_shared_data(void);

/**
 @brief  MobileVisor platform 32bit register access service
 @param  address physical address map
 @param  p_reg_val read value (-1 if access disallowed)
 @param  mask bit to modify
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_reg_read(uint32_t address, uint32_t *p_reg_val, uint32_t mask);

/**
 @brief  MobileVisor platform 32bit register access service
 @param  address physical address map
 @param  reg_val value to be written
 @param  mask bit to modify
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_reg_write(uint32_t address, uint32_t reg_val, uint32_t mask);

/**
 @brief  MobileVisor platform 32bit register access service
 @param  address physical address map
 @param  reg_val value to be written
 @param  mask bit to modify
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_reg_write_only(uint32_t address, uint32_t reg_val, uint32_t mask);


/**
 @brief  VMM PMIC register access service
 @param  operation read/write
 @param  reg_address, bit 24-31 slave address, bit 0-23 reg address to be access
 @param  size to read/write in bytes
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_pmic_reg_access(enum pmic_reg_access_op_code op,
			uint32_t reg_address, uint32_t size_in_byte);


/**
 @brief  Vtimer start service
 @param  num_of_vcycles
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_vtimer_start(uint32_t num_of_vcycles);

/**
 @brief  Vtimer stop service
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_vtimer_stop(void);

/**
 @brief  Vtimer get frequency service
 @return virtual frequency value
**/
int32_t mv_svc_vtimer_get_freq(void);

/**
 @brief  MobileVisor platform spcu thermal service
 @param  spcu thermal platform opcode, see enum spcu_thermal_service_op_code
 @param  which thermal are you operating, 0, 1, 2
 @param  which threshold are you operating, low or high
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_spcu_thermal_service(uint32_t opcode, uint32_t dev_id,
			 uint32_t thres);
/**
 @brief  VMM PWM access service
 @param  operation config/enable/disable
 @param  duty_ns duty time in ns
 @param  period_ns total period time in ns
 @return 0 if success, -1 if access disallowed
**/
int32_t mv_svc_pwm_access(enum pwm_op_code op,
			uint32_t duty_ns, uint32_t period_ns);

#endif /* _VMM_PLATFORM_SERVICE_H */
