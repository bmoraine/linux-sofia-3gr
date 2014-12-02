#ifndef _HIRQ_IRQ_H
#define _HIRQ_IRQ_H

/* HIRQ nodes from HIRQ domain point of view */
#define HIRQ_VMM_BROADCAST 0
#define HIRQ_PMIC_TEST 1
#define HIRQ_HIRQ_TEST 2
#ifdef SOFIA3G_ES2_TAB_SVB
#define HIRQ_PMIC_PRH_SIGNALING 3
#else
#define HIRQ_PRH_SIGNALING 3
#endif

#define HIRQ_VTIMER 4
/* 5 */
/* 6 */
/* 7 */
/* 8 */
#define HIRQ_RTC_ALARM 9
#define HIRQ_CPU_CLK_CHANGE 10
#define HIRQ_MODEM_DVFS 11
#endif /* _HIRQ_IRQ_H */
