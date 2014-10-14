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

#ifdef CONFIG_X86_INTEL_SOFIA
#include <sofia/nk_sofia_bridge.h>
#include <sofia/mv_svc_hypercalls.h>
#else
#include <nk/nkern.h>
#include <nk/nkdev.h>
#include <nk/nk.h>
#endif


#define TRACE(x...)   pr_notice("VSYSPROF-BE: " x)
#define ETRACE(x...)  pr_err("VSYSPROF-BE: Error: " x)


#define VSYSPROF_NAME               "vsysprof"

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


struct vsysprof_shared {
	uint32_t    opcode;
	union {
		struct {
			uint32_t    trace_paddr[SYSPROF_NOF_PHYSICAL_CORES];
			uint32_t    trace_mask;
		} setup;
		struct {
			uint32_t    id;
			char        name[MAX_ENTITY_NAME_SZ];
		} entity_info;
	} u;
};

struct vsysprof_struct {
	NkDevVlink              *vlink;
	NkXIrq                  sxirq;
	NkXIrq                  cxirq;
	NkXIrqId                sxid;
	NkPhAddr                pshared;
	struct vsysprof_shared  *vshared;
	struct work_struct      cmd_work;
	struct completion       entity_completion;
};

static struct vsysprof_struct vsysprof;

static uint32_t sys_prof_if_dummy;

static uint32_t *sys_prof_trace_addr[SYSPROF_NOF_PHYSICAL_CORES];

/* Array with write pointers per core and per event class. */
uint32_t __iomem
	*sys_prof_if[SYSPROF_NOF_PHYSICAL_CORES][SYSPROF_NOF_EVENT_CLASSES] = {
		SYS_PROF_IF_INIT_VALUES,
		SYS_PROF_IF_INIT_VALUES
	};
EXPORT_SYMBOL(sys_prof_if);


static void vsysprof_set_trace_address(uint32_t *trace_addr)
{
	int i;

	for (i = 0; i < SYSPROF_NOF_PHYSICAL_CORES; i++) {
		if (!sys_prof_trace_addr[i])
			sys_prof_trace_addr[i] = (uint32_t *)
						ioremap_nocache(
							trace_addr[i],
							sizeof(uint32_t));
	}
}

static void vsysprof_set_trace_mask(unsigned int mask)
{
	int i, j, k;
	uint32_t *trace_addr;

	for (i = 0; i < SYSPROF_NOF_PHYSICAL_CORES; i++) {
		for (j = 0; j < SYSPROF_NOF_EVENT_CLASSES; j++) {
			/*
			 * For SMP, re-arrange the trace address, so they can be
			 * directly indexed by raw_smp_processor_id().
			 * The assumption of the relation between this ID and
			 * the physical core ID may be valid only for a system
			 * with dual core CPU only.
			 */
			if ((mask & (1 << j)) && sys_prof_trace_addr[i]) {
#ifdef CONFIG_SMP
				k = SYSPROF_NOF_PHYSICAL_CORES - i - 1;
#else
				k = i;
#endif
				trace_addr = sys_prof_trace_addr[k];
			} else
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
			nkops.nk_xirq_trigger(pvsysprof->cxirq,
							pvsysprof->vlink->c_id);

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
	nkops.nk_xirq_trigger(pvsysprof->cxirq, pvsysprof->vlink->c_id);
}

static void vsysprof_get_irq_list(struct vsysprof_struct *pvsysprof)
{
	unsigned int irq;
	struct irq_desc *desc;

	for (irq = 0; irq < nr_irqs; irq++) {
		desc = irq_to_desc(irq);
		if (desc == NULL || desc->action == NULL)
			continue;

		/* Trigger XIRQ after filling shared memory with IRQ info */
		if (pvsysprof->vshared->opcode == SYSPROF_ENTITY_INFO) {
			nkops.nk_xirq_trigger(pvsysprof->cxirq,
							pvsysprof->vlink->c_id);

			/* wait for front-end driver to finisg reading entity */
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
	nkops.nk_xirq_trigger(pvsysprof->cxirq, pvsysprof->vlink->c_id);
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
		nkops.nk_xirq_trigger(pvsysprof->cxirq, pvsysprof->vlink->c_id);
	} else if (pcmd->opcode == SYSPROF_TRACE_STOP) {
		vsysprof_set_trace_mask(0);
		nkops.nk_xirq_trigger(pvsysprof->cxirq, pvsysprof->vlink->c_id);
	} else if (pcmd->opcode == SYSPROF_TASK_LIST_REQ) {
		vsysprof_get_task_list(pvsysprof);
	} else if (pcmd->opcode == SYSPROF_IRQ_LIST_REQ) {
		vsysprof_get_irq_list(pvsysprof);
	}
}

static void vsysprof_sxirq_hdlr(void *dev, NkXIrq xirq)
{
	struct vsysprof_struct *pvsysprof;

	(void) xirq;

	pvsysprof = (struct vsysprof_struct *) dev;

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

static int __init vsysprof_init(void)
{
	NkPhAddr plink = 0;
	bool found = false;
	int i, j;

	while ((plink = nkops.nk_vlink_lookup(VSYSPROF_NAME, plink)) != 0) {
		vsysprof.vlink = nkops.nk_ptov(plink);
		if (vsysprof.vlink->link == 0) {
			if (vsysprof.vlink->s_id == nkops.nk_id_get()) {
				found = true;
				break;
			}
		}
	}

	if (!found) {
		ETRACE("vlink lookup fail\n");
		return -1;
	}

	vsysprof.pshared = nkops.nk_pmem_alloc(plink, 1,
						sizeof(struct vsysprof_shared));

	if (!vsysprof.pshared) {
		ETRACE("pmem alloc fail\n");
		return -1;
	}

	vsysprof.vshared = (void *) nkops.nk_mem_map(vsysprof.pshared,
						sizeof(struct vsysprof_shared));
	if (!vsysprof.vshared) {
		ETRACE("pmem mapping fail\n");
		return -1;
	}

	vsysprof.sxirq = nkops.nk_pxirq_alloc(plink, 0,
						vsysprof.vlink->s_id, 1);
	vsysprof.cxirq = nkops.nk_pxirq_alloc(plink, 1,
						vsysprof.vlink->c_id, 1);

	if (vsysprof.sxirq == 0) {
		ETRACE("server xirq alloc fail\n");
		return -1;
	}

	if (vsysprof.cxirq == 0) {
		ETRACE("client xirq alloc fail\n");
		return -1;
	}

	for (i = 0; i < SYSPROF_NOF_PHYSICAL_CORES; i++) {
		sys_prof_trace_addr[i] = 0;

		for (j = 0; j < SYSPROF_NOF_EVENT_CLASSES; j++)
			sys_prof_if[i][j] = (uint32_t __iomem *)
							&sys_prof_if_dummy;
	}

	INIT_WORK(&vsysprof.cmd_work, vsysprof_cmd_work);
	init_completion(&vsysprof.entity_completion);

	vsysprof.sxid = nkops.nk_xirq_attach(vsysprof.sxirq,
					vsysprof_sxirq_hdlr, &vsysprof);

	TRACE("Initialized.\n");
	return 0;
}

static void __exit vsysprof_exit(void)
{
	nkops.nk_xirq_detach(vsysprof.sxid);

	TRACE("Module unloaded.\n");
}


module_init(vsysprof_init);
module_exit(vsysprof_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("virtual sysprof");
