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
/*
 * WARNING:
 * If this file is  modified,
 * ensure the regenerated lib_mobilevisor_platform_service.a into
 * pal/release/$(PROJECTFOLDER)/lib_guest/ folder for commit
 * Use command: "make release"
*/

#ifndef PAL_SHARED_DATA_H
#define PAL_SHARED_DATA_H

#ifdef __KERNEL__
#include "linux/types.h"
#else
#include "stdint.h"
#endif

struct pm_control_shared_data {
       volatile uint32_t emif_curr_freq;
       volatile uint32_t emif_clk_src;
       volatile uint32_t prh_user_id;
       volatile uint32_t prh_per_id;
       volatile uint32_t prh_param[20];
       volatile uint32_t reserved0[4];
       volatile uint32_t vm_pow_state_param[20];
       volatile uint32_t vm_cpu_freq_param[20];
       volatile uint32_t prh_request_control_flag;
       volatile uint32_t prh_request_return_value;
       volatile uint32_t target_power_state;
       volatile uint32_t actual_power_state;
       volatile uint32_t modem_state;
       volatile uint32_t exit_latency;
       volatile uint32_t vm_blocker_id;
       volatile uint32_t vm_blocking_reason;
       volatile uint32_t calibration_state;
       volatile uint32_t gsm_sleep_timer_frames_in;
       volatile uint32_t gsm_sleep_timer_frames_out;
       volatile uint32_t gsm_sleep_timer_stopped;
       volatile uint32_t cpu_drv_param;
       volatile uint32_t cpu_scaling_states[7];
       volatile uint32_t cpu_clk;
       volatile uint32_t vcpu_c0[12];
       volatile uint32_t num_dev_sleep_blockers;
};

#define PMIC_ACCESS_MAX_SIZE 64

struct pmic_access_shared_data {
	uint32_t status;
	uint8_t  data[PMIC_ACCESS_MAX_SIZE];
};

/**
Data structure for keeping date and time\n
*/
struct rtc_datetime_shared_data {
	uint16_t   m_year;
	uint8_t    m_month;
	uint8_t    m_day;
	uint8_t    m_hour;
	uint8_t    m_minute;
	uint8_t    m_second;
	uint16_t   m_msecond;
};

#define MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP 64
/**
  @brief pinctrl group information
**/
struct pinctrl_group_info {
	uint32_t pinctrl_pad; /*!< PCL pad index */
	uint32_t pinctrl_pad_value; /*!< pad value */
};

struct pal_shared_data {
	struct pm_control_shared_data pm_control_shared_data;
	struct pmic_access_shared_data pmic_access_shared_data;
	struct rtc_datetime_shared_data rtc_shared_data;
	struct pinctrl_group_info
	    pincontrol_group_shared_data[MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP];
};

enum VMM_POWER_STATE_E {
	PM_S0,
	PM_S1,
	PM_S0i3 = 4,
	PM_S3 = 4,
	PM_S5 = 5,
};

enum vmm_pm_opcode {
	PM_OPCODE_START = 0,
	ENTER_IDLE = 1,
	ENTER_SLEEP = 2,
	SYSTEM_READY = 3,
	EMIC_INIT = 4,
	PM_PRH_PRE_INIT_SET_MODE = 5,
	PM_PRH_INIT_SET_MODE = 6,
	PM_PRH_LATE_INIT_SET_MODE = 7,
	PM_PRH_SET_PER_MODE = 8,
	PM_WAKEUP_CONTROL = 9,
	PM_CALIB_CONTROL = 10,
	PM_PRH_SET_PER_MODE_SYNC = 11,
	PM_PRH_SET_PER_MODE_ASYNC = 12,
	PM_CPU_DIS_PRF_INT = 13,
	PM_CPU_EN_PRF_INT = 14,
	PM_CPU_SCALING_START = 15,
	PM_CPU_LOAD2 = 16,
	PM_OMP_SET_POLICY = 17,
	PM_GET_VCPU_C0 = 18,
	PM_REQ_FREQ_CHNG = 19,
	PM_GSM_SLEEP_TIMER_START = 20,
	PM_GSM_SLEEP_TIMER_STOP = 21,
	PM_GSM_SLEEP_TIMER_GET_SLEPT_FRAMES = 22,
	PM_GSM_SLEEP_TIMER_IS_STOPPED = 23,
	PM_CPU_GET_NUM_FREQ_STEPS = 24,
	PM_CPU_GET_FREQ_STEPS = 25,
	PM_OPCODE_END = 26,
};


enum boot_mode {
	ENUM_BOOT_MODE_POWER_OFF = 1,
	ENUM_BOOT_MODE_NORMAL,
	ENUM_BOOT_MODE_WEAK_BATTERY_CHARGING,
	ENUM_BOOT_MODE_WEAK_BATTERY_CHARGING_STEP2,
	ENUM_BOOT_MODE_RECOVERY,
	ENUM_BOOT_MODE_RECOVERY_CLEAR,
	ENUM_BOOT_MODE_FASTBOOT,
	ENUM_BOOT_MODE_PTEST,
	ENUM_BOOT_MODE_PTEST_CLEAR,
	ENUM_BOOT_MODE_CHARGEONLY,
	ENUM_BOOT_MODE_SILENT_RESET,
	ENUM_BOOT_MODE_KDUMP,
	ENUM_BOOT_MODE_SHUT_DOWN
};

struct slb_iram_mode_info {
	uint32_t magic;
	uint32_t version;
	uint32_t next_mode; /* T_ENUM_BOOT_MODE */
	uint32_t check_sum;
};

#define IRAM_MODE_INFO_MAGIC 0x9876abcd
#define IRAM_MODE_INFO_VERSION 1

/*#################################################################*/
#endif
