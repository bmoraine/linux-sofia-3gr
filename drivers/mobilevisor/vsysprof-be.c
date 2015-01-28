/*
 ****************************************************************
 *
 *  Copyright (C) 2013 Intel Mobile Communications GmbH
 *  Copyright (C) 2011, Red Bend Ltd.
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

#include <linux/version.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/reboot.h>
#include <linux/completion.h>
#include <linux/sysprofile.h>
#include <linux/uaccess.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>

#include <sofia/mv_gal.h>
#include <sofia/mv_ipc.h>
#include <sofia/mv_svc_hypercalls.h>

#define TRACE(x...)   pr_notice("VSYSPROF-BE: " x)
#define ETRACE(x...)  pr_err("VSYSPROF-BE: Error: " x)

#define MAX_TASK_NAME_SZ            80          /* inclusive of '\0' */
#define MAX_IRQ_NAME_SZ             20		/* inclusive of '\0' */
#define MAX_ENTITY_NAME_SZ          MAX_TASK_NAME_SZ
#define SYSPROF_LAST_ENTITY_INFO    0x80000000

/* Initial pointer values in sys_prof_if[][] for a physical core.
   Number of entries here is equivalent to SYSPROF_NOF_EVENT_CLASSES. */
#define	SYS_PROF_IF_INIT_VALUES		\
		{ (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy, \
		  (uint32_t __iomem *) &sys_prof_if_dummy }


struct interrupt_info {
	uint32_t    id;
	char        name[MAX_IRQ_NAME_SZ];
};

struct vsysprof_shared {
	uint32_t    opcode;
	union {
		struct {
			uint32_t    trace_mask;
			uint32_t    trace_paddr[CONFIG_NR_CPUS];
		} setup;
		struct {
			uint32_t    id;
			char        name[MAX_ENTITY_NAME_SZ];
		} entity_info;
	} u;
};

enum vsysprof_status {
	VSYSPROF_STATUS_CONNECT = 0,
	VSYSPROF_STATUS_DISCONNECT
};

struct vsysprof_struct {
	uint32_t token;
	char *cmdline;
	uint32_t share_data_size;
	struct vsysprof_shared  *vshared;
	enum vsysprof_status status;

	struct work_struct      cmd_work;
	struct completion       entity_completion;
};

static struct vsysprof_struct vsysprof;

static uint32_t sys_prof_if_dummy;

static uint32_t *sys_prof_trace_addr[CONFIG_NR_CPUS];

/* Array with write pointers per core and per event class. */
uint32_t __iomem
	*sys_prof_if[CONFIG_NR_CPUS][SYSPROF_NOF_EVENT_CLASSES] = {
#ifdef CONFIG_SMP
#if CONFIG_NR_CPUS >= 8
		SYS_PROF_IF_INIT_VALUES,
		SYS_PROF_IF_INIT_VALUES,
		SYS_PROF_IF_INIT_VALUES,
		SYS_PROF_IF_INIT_VALUES,
#endif
#if CONFIG_NR_CPUS >= 4
		SYS_PROF_IF_INIT_VALUES,
		SYS_PROF_IF_INIT_VALUES,
#endif
#if CONFIG_NR_CPUS >= 2
		SYS_PROF_IF_INIT_VALUES,
#endif
#endif
		SYS_PROF_IF_INIT_VALUES
	};
EXPORT_SYMBOL(sys_prof_if);

/*
 * List of interrupts not handled by common_interrupt handler
 */
static struct interrupt_info interrupt_list[] = {
#ifdef CONFIG_X86_LOCAL_APIC
	{LOCAL_TIMER_VECTOR, "apic_timer"},
	{ERROR_APIC_VECTOR, "error_interrupt"},
	{SPURIOUS_APIC_VECTOR, "spurious_interrupt"},
#ifdef CONFIG_IRQ_WORK
	{IRQ_WORK_VECTOR, "irq_work_interrupt"},
#endif
#endif
#ifdef CONFIG_SMP
	{RESCHEDULE_VECTOR , "reschedule_intr"},
	{CALL_FUNCTION_VECTOR, "call_function"},
	{CALL_FUNCTION_SINGLE_VECTOR, "call_func_single"},
	{REBOOT_VECTOR, "reboot_interrupt"},
#endif
#ifdef CONFIG_HAVE_KVM
	{POSTED_INTR_VECTOR, "kvm_posted_intr_ipi"},
#endif
	{X86_PLATFORM_IPI_VECTOR, "x86_platform_ipi"}
};

#define NUMOF_INTERRUPT_INFO \
			(sizeof(interrupt_list)/sizeof(struct interrupt_info))

static void vsysprof_set_trace_address(uint32_t *trace_addr)
{
	int i;

	for (i = 0; i < num_online_cpus(); i++) {
		if (!sys_prof_trace_addr[i])
			sys_prof_trace_addr[i] = (uint32_t *)
						ioremap_nocache(
							trace_addr[i],
							sizeof(uint32_t));
	}
}

static void vsysprof_set_trace_mask(unsigned int mask)
{
	int i, j;
	uint32_t *trace_addr;

	for (i = 0; i < num_online_cpus(); i++) {
		for (j = 0; j < SYSPROF_NOF_EVENT_CLASSES; j++) {
			if ((mask & (1 << j)) && sys_prof_trace_addr[i])
				trace_addr = sys_prof_trace_addr[i];
			else
				trace_addr = &sys_prof_if_dummy;
			sys_prof_if[i][j] = (uint32_t __iomem *) trace_addr;
		}
	}
}

/*
 * The following function has been extracted from proc_pid_cmdline()
 * in fs/proc/base.c
 */
static void vsysprof_get_task_info(struct task_struct *tsk, uint32_t *pid,
					char **pname)
{
	int res, len;
	static char buffer[PAGE_SIZE+1];
	struct mm_struct *mm;

	res = 0;
	mm = get_task_mm(tsk);
	if (!mm)
		goto out;

	if (!mm->arg_end)
		goto out_mm;

	len = mm->arg_end - mm->arg_start;
	if (len > PAGE_SIZE)
		len = PAGE_SIZE;

	res = access_process_vm(tsk, mm->arg_start, buffer, len, 0);

	/* If the nul at the end of args has been overwritten, then
	   assume application is using setproctitle(3). */
	if (res > 0 && buffer[res-1] != '\0' && len < PAGE_SIZE) {
		len = strnlen(buffer, res);
		if (len < res) {
			res = len;
		} else {
			len = mm->env_end - mm->env_start;
			if (len > PAGE_SIZE - res)
				len = PAGE_SIZE - res;
			res += access_process_vm(tsk, mm->env_start, buffer+res,
								len, 0);
			res = strnlen(buffer, res);
		}
	}
out_mm:
	mmput(mm);
out:
	if (*(buffer+res) != '\0')
		*(buffer+res) = '\0';

	*pid = task_pid_nr(tsk);

	if (res == 0) {
		buffer[0] = '[';
		for (len = 0; len < TASK_COMM_LEN; len++) {
			if (tsk->comm[len] == '\0')
				break;
			buffer[len+1] = tsk->comm[len];
		}
		buffer[len+1] = ']';
		buffer[len+2] = '\0';
		*pname = buffer;
	} else {
		len = ((res + 1) > MAX_TASK_NAME_SZ) ?
					(res + 1) - MAX_TASK_NAME_SZ : 0;
		*pname = &buffer[len];
	}
}

static void vsysprof_get_task_list(struct vsysprof_struct *pvsysprof)
{
	uint32_t pid;
	char *pname;
	struct task_struct *grp, *tsk;

	/* rcu_read_lock(); -- needed? */

	/*
	 * The following within the for loop has been extracted from
	 * proc_pid_cmdline() in fs/proc/base.c
	 */
	do_each_thread(grp, tsk) {
		/* Trigger XIRQ after filling shared memory with entity info */
		if (pvsysprof->vshared->opcode == SYSPROF_ENTITY_INFO) {
			mv_ipc_mbox_post(pvsysprof->token, 0);

			/* wait for front-end driver to finisg reading entity */
			wait_for_completion(&pvsysprof->entity_completion);
		}

		vsysprof_get_task_info(tsk, &pid, &pname);

		/*
		 * Fill in the shared memory here, but only trigger XIRQ at
		 * beginning of next loop so that there is a chance to append
		 * an indicator for the last task.
		 */
		pvsysprof->vshared->u.entity_info.id = pid;
		strncpy(pvsysprof->vshared->u.entity_info.name, pname,
					MAX_TASK_NAME_SZ);
		pvsysprof->vshared->opcode = SYSPROF_ENTITY_INFO;
		/* pr_warn("Task %04d: %s\n",
			pvsysprof->vshared->u.entity_info.id,
			pvsysprof->vshared->u.entity_info.name); */
	} while_each_thread(grp, tsk);

	/* rcu_read_unlock(); -- needed? */

	/* Trigger XIRQ to send the info of the last task */
	pvsysprof->vshared->u.entity_info.id |= SYSPROF_LAST_ENTITY_INFO;
	mv_ipc_mbox_post(pvsysprof->token, 0);
}

static void vsysprof_get_irq_list(struct vsysprof_struct *pvsysprof)
{
	unsigned int irq;
	struct irq_desc *desc;

	/*
	 * Send list of ID and name of interrupt not handled by common_interrupt
	 */
	for (irq = 0; irq < NUMOF_INTERRUPT_INFO; irq++) {
		pvsysprof->vshared->u.entity_info.id = interrupt_list[irq].id;
		strncpy(pvsysprof->vshared->u.entity_info.name,
				interrupt_list[irq].name, MAX_IRQ_NAME_SZ);
		pvsysprof->vshared->opcode = SYSPROF_ENTITY_INFO;
		mv_ipc_mbox_post(pvsysprof->token, 0);

		/* wait for front-end driver to finish reading entity */
		wait_for_completion(&pvsysprof->entity_completion);
	}

	/*
	 * Send list of ID and name of interrupt handled by common_interrupt
	 */
	for (irq = 0; irq < nr_irqs; irq++) {
		desc = irq_to_desc(irq);
		if (desc == NULL || desc->action == NULL)
			continue;

		/* Trigger XIRQ after filling shared memory with IRQ info */
		if (pvsysprof->vshared->opcode == SYSPROF_ENTITY_INFO) {
			mv_ipc_mbox_post(pvsysprof->token, 0);

			/* wait for front-end driver to finish reading entity */
			wait_for_completion(&pvsysprof->entity_completion);
		}

		/*
		 * Fill in the shared memory here, but only trigger XIRQ at
		 * beginning of next loop so that there is a chance to append
		 * an indicator for the last task.
		 */
		pvsysprof->vshared->u.entity_info.id = irq;
		if (desc->action->name && *(desc->action->name) != '\0') {
			strncpy(pvsysprof->vshared->u.entity_info.name,
					desc->action->name, MAX_IRQ_NAME_SZ);
			*(pvsysprof->vshared->u.entity_info.name
						+ MAX_IRQ_NAME_SZ - 1) = '\0';
		} else {
			sprintf(pvsysprof->vshared->u.entity_info.name,
							"IRQ_%04d", irq);
			*(pvsysprof->vshared->u.entity_info.name + 8) = '\0';
		}
		pvsysprof->vshared->opcode = SYSPROF_ENTITY_INFO;
	}

	/* Trigger XIRQ to send the info of the last task */
	pvsysprof->vshared->u.entity_info.id |= SYSPROF_LAST_ENTITY_INFO;
	mv_ipc_mbox_post(pvsysprof->token, 0);
}

static void vsysprof_cmd_work(struct work_struct *work)
{
	struct vsysprof_struct *pvsysprof;
	struct vsysprof_shared *pcmd;

	pvsysprof = container_of(work, struct vsysprof_struct, cmd_work);
	pcmd = pvsysprof->vshared;

	if (pcmd->opcode == SYSPROF_TRACE_START) {
		vsysprof_set_trace_address(pcmd->u.setup.trace_paddr);
		vsysprof_set_trace_mask(pcmd->u.setup.trace_mask);
		mv_ipc_mbox_post(pvsysprof->token, 0);
	} else if (pcmd->opcode == SYSPROF_TRACE_STOP) {
		vsysprof_set_trace_mask(0);
		mv_ipc_mbox_post(pvsysprof->token, 0);
	} else if (pcmd->opcode == SYSPROF_TASK_LIST_REQ) {
		vsysprof_get_task_list(pvsysprof);
	} else if (pcmd->opcode == SYSPROF_IRQ_LIST_REQ) {
		vsysprof_get_irq_list(pvsysprof);
	}
}

static void vsysprof_on_connect(uint32_t token, void *cookie)
{
	struct vsysprof_struct *pvsysprof;
	pvsysprof = (struct vsysprof_struct *) cookie;
	pvsysprof->status = VSYSPROF_STATUS_CONNECT;
}

static void vsysprof_on_disconnect(uint32_t token, void *cookie)
{
	struct vsysprof_struct *pvsysprof;
	pvsysprof = (struct vsysprof_struct *) cookie;
	pvsysprof->status = VSYSPROF_STATUS_DISCONNECT;
}

static void vsysprof_on_event(uint32_t token, uint32_t event_id, void *cookie)
{
	struct vsysprof_struct *pvsysprof;
	pvsysprof = (struct vsysprof_struct *) cookie;

	switch (pvsysprof->vshared->opcode) {
	case SYSPROF_TRACE_START:
	case SYSPROF_TRACE_STOP:
	case SYSPROF_TASK_LIST_REQ:
	case SYSPROF_IRQ_LIST_REQ:
		schedule_work(&pvsysprof->cmd_work);
		break;

	case SYSPROF_ENTITY_SENT:
		complete(&pvsysprof->entity_completion);
		break;

	default:
		break;
	}
}

static struct mbox_ops vsysprof_ops = {
	.on_connect    = vsysprof_on_connect,
	.on_disconnect = vsysprof_on_disconnect,
	.on_event      = vsysprof_on_event
};

static int __init vsysprof_init(void)
{
	unsigned char *pshare_mem;
	struct vsysprof_struct *p_vsysprof = &vsysprof;

	p_vsysprof->token = mv_ipc_mbox_get_info("sysprof",
						 "m-l",
						 &vsysprof_ops,
						 &pshare_mem,
						 &(p_vsysprof->share_data_size),
						 &(p_vsysprof->cmdline),
						 (void *)p_vsysprof);

	if (p_vsysprof->share_data_size < sizeof(struct vsysprof_shared))
		return -EFAULT;

	p_vsysprof->vshared = (struct vsysprof_shared *)pshare_mem;

	INIT_WORK(&vsysprof.cmd_work, vsysprof_cmd_work);
	init_completion(&vsysprof.entity_completion);

	/* Set on line */
	p_vsysprof->status = VSYSPROF_STATUS_DISCONNECT;
	mv_mbox_set_online(p_vsysprof->token);

	TRACE("Initialized.\n");
	return 0;
}

static void __exit vsysprof_exit(void)
{
	TRACE("Module unloaded.\n");
}


module_init(vsysprof_init);
module_exit(vsysprof_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("virtual sysprof");
