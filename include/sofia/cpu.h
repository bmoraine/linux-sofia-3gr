/* Copyright (C) 2014 Intel Mobile Communications GmbH
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
#include "linux/types.h"

extern struct static_key paravirt_steal_enabled;
extern struct static_key paravirt_steal_rq_enabled;


struct sofia_cpu_freq_t {
	unsigned curfreq;
	unsigned minfreq;
	unsigned maxfreq;
};

int sofia_get_steal_time(uint64_t *total_time, uint64_t *active_stolen_time,
						uint64_t *idle_stolen_time);

void sofia_set_cpu_frequency(const int cpufreq);

void sofia_set_cpu_policy(const int freqmin, const int freqmax);

void sofia_thermal_set_cpu_policy(const int freqmin, const int freqmax);

void sofia_get_cpu_frequency(struct sofia_cpu_freq_t *freq);

u32 sofia_get_cpu_freq_table(int *freq_table, const int nb_freq);

u32 sofia_get_cpu_nb_freq(void);

bool sofia_is_cpu_shared(unsigned);

bool sofia_is_cpu_exclusive(unsigned);

unsigned sofia_get_nr_vcpus(void);

int sofia_cpu_mapping_init(void);

unsigned sofia_cpu_get_apicid(unsigned);

static inline u64 paravirt_steal_clock(int cpu)
{
	u64 active_steal, timetotal, idle_steal;

	sofia_get_steal_time(&timetotal, &active_steal, &idle_steal);

	return active_steal;
}
