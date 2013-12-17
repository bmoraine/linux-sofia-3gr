/* Copyright (C) 2013 Intel Mobile Communications GmbH
 * *
 * * This software is licensed under the terms of the GNU General Public
 * * License version 2, as published by the Free Software Foundation, and
 * * may be copied, distributed, and modified under those terms.
 * *
 * * This program is distributed in the hope that it will be useful,
 * * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * * GNU General Public License for more details.
 * */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/reboot.h>
#include <linux/notifier.h>

#include <linux/io.h>
#include <asm/reboot.h>
#include <sofia/nk_sofia_bridge.h>
#include <sofia/vmm_al.h>
#include <sofia/pal_shared_data.h>


#define TRACE(x...) pr_notice("REBOOT driver: " x)
#define ETRACE(x...) pr_err("REBOOT driver: Error: " x)


static volatile unsigned int running_guest;
static NkXIrqId reboot_sysconf_id;
static bool is_linux_reboot = true;
static bool is_blocking;

static void next_mode_transition(enum boot_mode *next_mode, char *reason)
{

	if (!reason)
		*next_mode = ENUM_BOOT_MODE_NORMAL;
	else if (strcmp(reason, "halt") == 0)
		*next_mode = ENUM_BOOT_MODE_SHUT_DOWN;
	else if (strcmp(reason, "poweroff") == 0)
		*next_mode = ENUM_BOOT_MODE_SHUT_DOWN;
	else if (strcmp(reason, "silent") == 0)
		*next_mode = ENUM_BOOT_MODE_SILENT_RESET;
	else if (strcmp(reason, "recovery") == 0)
		*next_mode = ENUM_BOOT_MODE_RECOVERY;
	else if (strcmp(reason, "recovery_clear") == 0)
		*next_mode = ENUM_BOOT_MODE_RECOVERY_CLEAR;
	else if (strcmp(reason, "fastboot") == 0)
		*next_mode = ENUM_BOOT_MODE_FASTBOOT;
	else if (strcmp(reason, "ptest") == 0)
		*next_mode = ENUM_BOOT_MODE_PTEST;
	else if (strcmp(reason, "ptest_clear") == 0)
		*next_mode = ENUM_BOOT_MODE_PTEST_CLEAR;
	else
		*next_mode = ENUM_BOOT_MODE_NORMAL;
	TRACE("next_mode_transition 0x%x, reboot cmd: %s\n",
			*next_mode, reason);
}

static void reboot_sysconf_hdl(void *dev, NkXIrq xirq)
{
	struct vmm_shared_data *data;
	data = get_vmm_shared_data();
	/* TRACE("system_reboot_action: %d\n", data->system_reboot_action); */
	if (data->system_reboot_action > 0) {
		if (!is_blocking) {
			TRACE("This reboot triggered by other OS\n");
			is_linux_reboot = false;
			kernel_restart(NULL);
		}
	}
}

static int reboot(struct notifier_block *notifier,
		unsigned long event, void *data)
{
	enum boot_mode next_mode = 0;
	struct vmm_shared_data *share_data;
	TRACE("Event code: %li!  reboot cmd: %s\n", event, (char *)data);
	if ((event == SYS_HALT) || (event == SYS_POWER_OFF))
		next_mode_transition(&next_mode, "poweroff");
	else
		next_mode_transition(&next_mode, (char *)data);
	is_blocking = true;
	if (is_linux_reboot)
		vmm_initiate_reboot(next_mode);
	/*
	 * Block reboot process and waiting for other os finish the reboot
	 * since we have NVM depency
	 */
	while (1) {
		share_data = get_vmm_shared_data();
		running_guest = vmm_get_running_guests();
		if (running_guest == (1 << share_data->os_id))
			break;
	}
	TRACE("all other guest os reboot successfully!\n");
	return NOTIFY_OK;
}

static struct notifier_block reboot_notifier = {
	.notifier_call = reboot
};

void vmm_machine_crash_shutdown(struct pt_regs *regs)
{
	vmm_stop_vcpu(vmm_vcpu_id());
}

static void vmm_machine_emergency_restart(void)
{
	vmm_stop_vcpu(vmm_vcpu_id());
}

void vmm_machine_shutdown(void)
{
	vmm_stop_vcpu(vmm_vcpu_id());
}

static void vmm_machine_restart(char *__unused)
{
	vmm_stop_vcpu(vmm_vcpu_id());
}

static void vmm_machine_halt(void)
{
	vmm_stop_vcpu(vmm_vcpu_id());
}

static void vmm_machine_power_off(void)
{
	vmm_stop_vcpu(vmm_vcpu_id());
}

#ifdef CONFIG_KEXEC
void vmm_crash_shutdown(struct pt_regs *regs)
{
}
#endif

struct machine_ops vmm_machine_ops = {
	.power_off = vmm_machine_power_off,
	.shutdown = vmm_machine_shutdown,
	.emergency_restart = vmm_machine_emergency_restart,
	.restart = vmm_machine_restart,
	.halt = vmm_machine_halt,
#ifdef CONFIG_KEXEC
	.crash_shutdown = vmm_machine_crash_shutdown,
#endif
};

static int __init reboot_init(void)
{
	running_guest = vmm_get_running_guests();
	reboot_sysconf_id =
		nkops.nk_xirq_attach(NK_XIRQ_SYSCONF, reboot_sysconf_hdl, 0);
	/* vmm_register_xirq_callback(VMM_XIRQ_SYSCONF,
			reboot_sysconf_hdl, NULL); */
	if (reboot_sysconf_id == 0) {
		TRACE("nk_xirq_attach failed\n");
		return -1;
	}
	register_reboot_notifier(&reboot_notifier);
	machine_ops = vmm_machine_ops;
	return 0;
}

static void __exit reboot_exit(void)
{
	unregister_reboot_notifier(&reboot_notifier);
}

module_init(reboot_init);
module_exit(reboot_exit);

MODULE_LICENSE("GPL");


