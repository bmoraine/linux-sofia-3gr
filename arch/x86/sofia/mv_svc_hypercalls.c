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
 * If any files in pal/platforms/<platform>/service are modified,
 * ensure the regenerated lib_mobilevisor_platform_service.a into
 * pal/release/$(PROJECTFOLDER)/lib_guest/ folder for commit
 * Use command: "make release"
*/

/*
 * WARNING:
  * Always use portable C99 fixed width types for 64bit/32bit compatibility.
*/
#ifdef __KERNEL__
#include <linux/kernel.h>
#include <linux/string.h>
#include <sofia/pal_shared_data.h>
#include <sofia/mv_hypercalls.h>
#include <sofia/mv_svc_hypercalls.h>
#include <sofia/mv_gal.h>
#else
#include <mv_gal.h>
#include <pal_shared_data.h>
#include <mv_svc_hypercalls.h>
#include <mv_hypercalls.h>
#endif

void mv_svc_dummy_service(uint32_t *ret1, uint32_t *ret2,
				uint32_t *ret3, uint32_t *ret4)
{
	mv_platform_service(DUMMY_SERIVCE, 0, 0, 0, 0, ret1, ret2, ret3, ret4);
}

uint32_t mv_svc_pinctrl_service(
				uint32_t opcode,
				uint32_t arg1,
				uint32_t arg2,
				uint32_t *arg3)
{
	struct pal_shared_data *pal_shared_data_ptr;
	uint32_t result = 0;

	switch (opcode) {
	case PINCTRL_OPEN:
	case PINCTRL_SET:
	case PINCTRL_CONFIG_FUNC:
	case PINCTRL_CLOSE:
	case PINCTRL_SET_CONTROL:
		return mv_platform_service(PINCTRL_SERVICE,
					   opcode,
					   arg1, arg2, 0, 0, 0, 0, 0);
		break;

	case PINCTRL_GET:
	case PINCTRL_GET_CONTROL:
	case PINCTRL_GET_PIN:
		return mv_platform_service(PINCTRL_SERVICE, opcode,
					arg1,
					0, 0, 0, 0, 0, arg3);
		break;

	case PINCTRL_SET_GROUP:
			pal_shared_data_ptr = (struct pal_shared_data *)
				mv_svc_get_shared_data();
			if (arg2 == 0)
				return 0xFFFFFFFF;

			/* check is required to
			 * avoid memory corruption
			*/
			if (arg2 > MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP)
				arg2 =
				MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP;

			memcpy((struct pinctrl_group_info *)
				&pal_shared_data_ptr->
				pincontrol_group_shared_data,
				(void *)arg1,
				(sizeof(struct pinctrl_group_info) * arg2));
			return mv_platform_service(PINCTRL_SERVICE,
				opcode, 0,
				arg2, 0, 0, 0, 0, 0);
		break;

	case PINCTRL_GET_GROUP:
			pal_shared_data_ptr = (struct pal_shared_data *)
				mv_svc_get_shared_data();
			if (arg2 == 0)
				return 0xFFFFFFFF;

			/* check is required to avoid
			 * memory corruption
			*/

			if (arg2 > MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP)
				arg2 =
				MAX_NUMBER_OF_PADS_IN_PINCTRL_GROUP;
			memcpy((struct pinctrl_group_info *)
				&pal_shared_data_ptr->
				pincontrol_group_shared_data,
				(void *)arg1,
				(sizeof(struct pinctrl_group_info)
				*arg2));
			result = mv_platform_service(PINCTRL_SERVICE,
				     opcode, 0,
				     arg2, 0, 0, 0, 0, 0);
			memcpy((void *)arg1,
			(struct pinctrl_group_info *)
			&pal_shared_data_ptr->
			pincontrol_group_shared_data,
			(sizeof(struct pinctrl_group_info) * arg2));
		break;

	default:
		break;
	}
	return result;
}

uint32_t mv_svc_pm_control(uint32_t pm_opcode,
				uint32_t arg1,
				uint32_t arg2,
				uint32_t arg3)
{
	return mv_platform_service(PM_SERVICE, pm_opcode, arg1, arg2,
				(uint32_t *)arg3,  0, 0, 0, 0);
}

void mv_svc_vm_enter_idle(uint32_t *pal_mem, uint32_t state)
{
	struct pal_shared_data *shared_mem = (struct pal_shared_data *)pal_mem;
	shared_mem->pm_control_shared_data.target_power_state = state;
}

void mv_svc_modem_2g_sleep_time(uint32_t *pal_mem, uint32_t duration)
{
	struct pal_shared_data *shared_mem = (struct pal_shared_data *)pal_mem;
	shared_mem->pm_control_shared_data.vm_pow_state_param[2] = duration;

}

void mv_svc_modem_3g_sleep_time(uint32_t *pal_mem, uint32_t duration)
{
	struct pal_shared_data *shared_mem = (struct pal_shared_data *)pal_mem;
	shared_mem->pm_control_shared_data.vm_pow_state_param[3] = duration;
}

void mv_svc_modem_next_timeout(uint32_t *pal_mem, uint32_t duration)
{
	struct pal_shared_data *shared_mem = (struct pal_shared_data *)pal_mem;
	shared_mem->pm_control_shared_data.vm_pow_state_param[4] = duration;
}

void mv_svc_cd_service(uint32_t cd_opcode, void *cd_data)
{
	mv_platform_service(CD_SERVICE, cd_opcode, (uint32_t)cd_data,
					0, 0, 0, 0, 0, 0);
}

void mv_svc_sys_exception(uint32_t opcode, void *trap_data)
{
	mv_platform_service(SYS_EXCEPTION_SERVICE, opcode,
				(uint32_t)trap_data, 0, 0, 0, 0, 0, 0);
}

uint32_t mv_svc_socwatch_config(uint32_t events,
			struct socwatch_buffer_info *buffer_info)
{
	mv_platform_service(SOCWATCH_SERVICE, SOCWATCH_SET_CONFIG, events,
				(uint32_t)buffer_info, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_socwatch_config);
#endif

uint32_t mv_svc_socwatch_run_control(uint32_t run_control)
{
	mv_platform_service(SOCWATCH_SERVICE, SOCWATCH_RUN_CONTROL,
					run_control, 0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_socwatch_run_control);
#endif

uint32_t mv_svc_sep_config(struct sep_buffer_info *p_buffer_info)
{
	mv_platform_service(SEP_SERVICE, SEP_CONFIG,
			(uint32_t)p_buffer_info, 0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_config);
#endif

uint32_t mv_svc_sep_guest_context(uint32_t task_pointer,
					uint32_t task_name_offset)
{
	mv_platform_service(SEP_SERVICE, SEP_SET_GUEST_CONTEXT, task_pointer,
					task_name_offset, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_guest_context);
#endif

uint32_t mv_svc_sep_run_control(uint32_t run_control)
{
	mv_platform_service(SEP_SERVICE, SEP_RUN_CONTROL, run_control,
					0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_run_control);
#endif

uint32_t mv_svc_sep_read_counters(struct sep_counter *buffer_pointer)
{
	mv_platform_service(SEP_SERVICE, SEP_READ_COUNTER_LIST,
				(uint32_t)buffer_pointer, 0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_read_counters);
#endif

uint32_t mv_svc_sep_write_counters(struct sep_counter *buffer_pointer)
{
	mv_platform_service(SEP_SERVICE, SEP_WRITE_COUNTER_LIST,
				(uint32_t)buffer_pointer, 0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_write_counters);
#endif

uint32_t mv_svc_sep_pmi_msr_list(struct sep_msr_control *entry_list,
					struct sep_msr_control *exit_list)
{
	mv_platform_service(SEP_SERVICE, SEP_CONTROL_PMI_MSR_LIST,
				(uint32_t)entry_list, (uint32_t)exit_list,
				0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_pmi_msr_list);
#endif

uint32_t mv_svc_sep_vmswitch_msr_list(struct sep_msr_control *vmentry_list,
					struct sep_msr_control *vmexit_list)
{
	mv_platform_service(SEP_SERVICE, SEP_CONTROL_VMSW_MSR_LIST,
				(uint32_t)vmentry_list, (uint32_t)vmexit_list,
				0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_vmswitch_msr_list);
#endif

uint32_t mv_svc_sep_version_number(void)
{
	uint32_t ver_num;
	if (mv_platform_service(SEP_SERVICE, SEP_VERSION_NUMBER,
				0, 0, 0, &ver_num, 0, 0, 0) != 0)
		return ver_num;
	else
		return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_sep_version_number);
#endif

uint32_t mv_svc_watchdog_enable(uint32_t timeout)
{
	mv_platform_service(WATCHDOG_SERVICE, WATCHDOG_ENABLE, timeout,
				0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_watchdog_enable);
#endif

uint32_t mv_svc_watchdog_pet(void)
{
	mv_platform_service(WATCHDOG_SERVICE, WATCHDOG_PET, 0,
			0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_watchdog_pet);
#endif

uint32_t mv_svc_watchdog_disable(void)
{
	mv_platform_service(WATCHDOG_SERVICE, WATCHDOG_DISABLE, 0,
			0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_watchdog_disable);
#endif


uint32_t mv_svc_scu_watchdog_switch(int on)
{
	mv_platform_service(WATCHDOG_SERVICE, VMM_SCU_WATCHDOG, on,
			0, 0, 0, 0, 0, 0);
	return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_scu_watchdog_switch);
#endif

uint32_t mv_svc_rtc_set_datetime(pal_rtc_datetime *rtc_datetime)
{
	struct pal_shared_data *pal_shared_data_ptr =
		(struct pal_shared_data *)mv_svc_get_shared_data();
	memcpy((pal_rtc_datetime *)&pal_shared_data_ptr->rtc_shared_data,
			rtc_datetime,
			sizeof(pal_shared_data_ptr->rtc_shared_data));
	mv_platform_service(RTC_SERVICE, RTC_SET_DATETIME,
			0, 0, 0, 0, 0, 0, 0);
	return 0;
}

uint32_t mv_svc_rtc_get_datetime(pal_rtc_datetime *rtc_datetime)
{
	struct pal_shared_data *pal_shared_data_ptr =
		(struct pal_shared_data *)mv_svc_get_shared_data();
	mv_platform_service(RTC_SERVICE, RTC_GET_DATETIME,
			0, 0, 0, 0, 0, 0, 0);
	memcpy(rtc_datetime,
		(pal_rtc_datetime *)&pal_shared_data_ptr->rtc_shared_data,
		sizeof(pal_shared_data_ptr->rtc_shared_data));
	return 0;
}

uint32_t mv_svc_rtc_get_datetime_async(void)
{
	mv_platform_service(RTC_SERVICE, RTC_GET_DATETIME,
			0, 0, 0, 0, 0, 0, 0);
	return 0;
}

uint32_t mv_svc_rtc_get_time_us(uint64_t *rtc_us_time)
{
	uint32_t rtc_time[2] = {0, 0};
	mv_platform_service(RTC_SERVICE, RTC_GET_DATETIME_US,
			0, 0, 0, 0, 0, &rtc_time[1], &rtc_time[0]);
	*rtc_us_time = (uint64_t)((((uint64_t)rtc_time[1]) << 0x20)
			| (uint64_t)(rtc_time[0]));
	return 0;
}

uint32_t mv_svc_rtc_clear_alarm(void)
{
	return mv_platform_service(RTC_SERVICE, RTC_CLEAR_ALARM,
			0, 0, 0, 0, 0, 0, 0);
}

uint32_t mv_svc_rtc_set_alarm(pal_rtc_datetime *rtc_alarm)
{
	struct pal_shared_data *pal_shared_data_ptr =
		(struct pal_shared_data *)mv_svc_get_shared_data();
	memcpy((pal_rtc_datetime *)&pal_shared_data_ptr->rtc_shared_data,
			rtc_alarm,
			sizeof(pal_shared_data_ptr->rtc_shared_data));
	return mv_platform_service(RTC_SERVICE, RTC_SET_ALARM,
			0, 0, 0, 0, 0, 0, 0);
}

uint32_t mv_svc_rtc_get_alarm(pal_rtc_datetime *rtc_alarm)
{
	struct pal_shared_data *pal_shared_data_ptr =
		(struct pal_shared_data *)mv_svc_get_shared_data();
	mv_platform_service(RTC_SERVICE, RTC_GET_ALARM, 0, 0, 0, 0, 0, 0, 0);
	memcpy(rtc_alarm,
		(pal_rtc_datetime *)&pal_shared_data_ptr->rtc_shared_data,
		sizeof(pal_shared_data_ptr->rtc_shared_data));
	return 0;
}

uint32_t mv_svc_rtc_get_alarm_async(void)
{
	return mv_platform_service(RTC_SERVICE, RTC_GET_ALARM,
			0, 0, 0, 0, 0, 0, 0);
}

void mv_svc_sysprof_trace_start(uint32_t *swt_paddr, uint32_t mask)
{
	mv_platform_service(SYSPROF_SERVICE, SYSPROF_TRACE_START,
				(uint32_t)swt_paddr, mask, 0, 0, 0, 0, 0);
}

void mv_svc_sysprof_trace_stop(void)
{
	mv_platform_service(SYSPROF_SERVICE, SYSPROF_TRACE_STOP,
				0, 0, 0, 0, 0, 0, 0);
}

void mv_svc_sysprof_get_vcore_map(struct sysprof_vcore_info *vcore_map)
{
	mv_platform_service(SYSPROF_SERVICE, SYSPROF_VCORE_MAP_REQ,
				(uint32_t)vcore_map, 0, 0, 0, 0, 0, 0);
}

void mv_svc_sysprof_perf_count_enable(uint32_t cnt_config)
{
	mv_platform_service(SYSPROF_SERVICE, SYSPROF_PERF_COUNT_ENABLE,
				cnt_config, 0, 0, 0, 0, 0, 0);
}

struct pal_shared_data *mv_svc_get_shared_data(void)
{
	struct vmm_shared_data *shared_mem = mv_gal_get_shared_data();
	return (struct pal_shared_data *)(shared_mem->pal_shared_mem_data);
}

struct pal_shared_data *mv_svc_get_system_shared_data(void)
{
	struct vmm_shared_data *shared_mem = mv_gal_get_system_shared_data();
	return (struct pal_shared_data *)(shared_mem->pal_shared_mem_data);
}


int32_t mv_svc_reg_read(uint32_t address, uint32_t *p_reg_val, uint32_t mask)
{
	return mv_platform_service(VMM_REG_RD_SERVICE, address, mask,
				0, 0, 0,
				0, 0, p_reg_val);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_reg_read);
#endif

int32_t mv_svc_reg_write(uint32_t address, uint32_t reg_val, uint32_t mask)
{
	return mv_platform_service(VMM_REG_WR_SERVICE, address, reg_val, mask,
				0, 0, 0, 0,
				0);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_reg_write);
#endif

int32_t mv_svc_reg_write_only(uint32_t address, uint32_t reg_val, uint32_t mask)
{
	return mv_platform_service(VMM_REG_WR_ONLY_SERVICE,
				address, reg_val, mask,
				0, 0, 0, 0,
				0);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_reg_write_only);
#endif

int32_t mv_svc_pmic_reg_access(enum pmic_reg_access_op_code op,
		uint32_t reg_address, uint32_t size_in_byte)
{
	return mv_platform_service(VMM_PMIC_REG_ACCESS_SERVICE,
		op, reg_address, size_in_byte,
		0, 0, 0, 0, 0);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_pmic_reg_access);
#endif

int32_t mv_svc_pwm_access(enum pwm_op_code op,
		uint32_t duty_ns, uint32_t period_ns)
{
	return mv_platform_service(VMM_PWM_SERVICE,
		op, duty_ns, period_ns,
		0, 0, 0, 0, 0);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_pwm_access);
#endif

int32_t mv_svc_vtimer_start(uint32_t num_of_vcycles)
{
	return mv_platform_service(VMM_VTIMER_START_SERVICE, num_of_vcycles,
		0, 0, 0, 0, 0, 0, 0);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_vtimer_start);
#endif

int32_t mv_svc_vtimer_stop(void)
{
	return mv_platform_service(VMM_VTIMER_STOP_SERVICE, 0,
		0, 0, 0, 0, 0, 0, 0);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_vtimer_stop);
#endif

int32_t mv_svc_vtimer_get_freq(void)
{
	int32_t freq;
	if (mv_platform_service(VMM_VTIMER_GET_FREQ_SERVICE, 0,
		0, 0, 0, &freq, 0, 0, 0) == 0)
		return freq;
	else
		return -1;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_vtimer_get_freq);
#endif

int32_t mv_svc_spcu_thermal_service(uint32_t opcode, uint32_t dev_id,
		 uint32_t threshold)
{
	return mv_platform_service(VMM_SPCU_THERMAL_SERVICE,
				opcode, dev_id, threshold,
				0, 0, 0, 0, 0);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_spcu_thermal_service);
#endif

int32_t mv_svc_security_verify_vm_completed(uint32_t verify_result,
						uint32_t vm_id)
{
	return mv_platform_service(VMM_SECURITY_VERIFY_VM_COMPLETED_SERVICE,
			verify_result, vm_id, 0, 0, 0, 0, 0, 0);
}

#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_security_verify_vm_completed);
#endif

int32_t mv_svc_security_getvm_loadinfo(uint32_t vm_id, uint32_t *vm_loadaddr,
					uint32_t *vm_loadsize)
{
	return mv_platform_service(VMM_SECURITY_GETVM_LOADINFO_SERVICE,
			vm_id, 0, 0, 0,	0, 0, vm_loadaddr, vm_loadsize);
}

#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_security_getvm_loadinfo);
#endif

uint32_t mv_svc_timestamp_counter_frequency(void)
{
	uint32_t freq;
	if (mv_platform_service(VMM_TIMESTAMP_COUNTER_INFO_SERVICE,
				TIMESTAMP_COUNTER_FREQ,
				0, 0, 0, &freq, 0, 0, 0) != 0)
		return freq;
	else
		return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_timestamp_counter_frequency);
#endif

uint32_t mv_svc_timestamp_counter_size(void)
{
	uint32_t size;
	if (mv_platform_service(VMM_TIMESTAMP_COUNTER_INFO_SERVICE,
				TIMESTAMP_COUNTER_SIZE,
				0, 0, 0, &size, 0, 0, 0) != 0)
		return size;
	else
		return 0;
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_timestamp_counter_size);
#endif

uint32_t mv_svc_silentlake_op(uint32_t op_leaf, uint32_t param_a, uint32_t param_b)
{
	return mv_platform_service(VMM_SILENTLAKE_SERVICES, op_leaf,
				param_a, param_b, 0, 0, 0, 0, 0);
}
#ifdef __KERNEL__
EXPORT_SYMBOL(mv_svc_silentlake_op);
#endif

