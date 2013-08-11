/*
 *  drivers/cpufreq/cpufreq_darkness.c
 *
 *  Copyright (C)  2011 Samsung Electronics co. ltd
 *    ByungChang Cha <bc.cha@samsung.com>
 *
 *  Based on ondemand governor
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>.
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 * Created by Alucard_24@xda
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cpufreq.h>
#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/hrtimer.h>
#include <linux/tick.h>
#include <linux/ktime.h>
#include <linux/sched.h>
#include <linux/slab.h>
/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define MAX_HOTPLUG_RATE		(40)
#define HOTPLUG_DOWN_INDEX		(0)
#define HOTPLUG_UP_INDEX		(1)

#ifndef CONFIG_CPU_EXYNOS4210
static atomic_t hotplug_freq[4][2] = {
	{ATOMIC_INIT(0), ATOMIC_INIT(500000)},
	{ATOMIC_INIT(200000), ATOMIC_INIT(500000)},
	{ATOMIC_INIT(200000), ATOMIC_INIT(500000)},
	{ATOMIC_INIT(200000), ATOMIC_INIT(0)}
};
#else
static atomic_t hotplug_freq[2][2] = {
	{ATOMIC_INIT(0), ATOMIC_INIT(500000)},
	{ATOMIC_INIT(200000), ATOMIC_INIT(0)}
};
#endif

static void do_darkness_timer(struct work_struct *work);
static int cpufreq_governor_darkness(struct cpufreq_policy *policy,
				unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_DARKNESS
static
#endif
struct cpufreq_governor cpufreq_gov_darkness = {
	.name                   = "darkness",
	.governor               = cpufreq_governor_darkness,
	.owner                  = THIS_MODULE,
};

struct cpufreq_darkness_cpuinfo {
	u64 prev_cpu_busy;
	u64 prev_cpu_idle;
	struct delayed_work work;
	int cpu;
};
/*
 * mutex that serializes governor limit change with
 * do_darkness_timer invocation. We do not want do_darkness_timer to run
 * when user is changing the governor or limits.
 */
static struct mutex timer_mutex;
static struct workqueue_struct *dvfs_workqueue;

static DEFINE_PER_CPU(struct cpufreq_darkness_cpuinfo, od_darkness_cpuinfo);
static DEFINE_PER_CPU(struct cpufreq_policy *, cpufreq_cpu_data);

static unsigned int darkness_enable;	/* number of CPUs using this policy */
/*
 * darkness_mutex protects darkness_enable in governor start/stop.
 */
static DEFINE_MUTEX(darkness_mutex);

/* darkness tuners */
static struct darkness_tuners {
	atomic_t sampling_rate;
	atomic_t hotplug_enable;
	atomic_t cpu_up_rate;
	atomic_t cpu_down_rate;
	atomic_t up_load;
	atomic_t down_load;
	atomic_t up_sf_step;
	atomic_t down_sf_step;
	atomic_t force_freqs_step;
	atomic_t onecoresuspend;
	atomic_t min_freq_limit;
	atomic_t max_freq_limit;
} darkness_tuners_ins = {
	.sampling_rate = ATOMIC_INIT(60000),
	.hotplug_enable = ATOMIC_INIT(0),
	.cpu_up_rate = ATOMIC_INIT(10),
	.cpu_down_rate = ATOMIC_INIT(5),
	.up_load = ATOMIC_INIT(65),
	.up_sf_step = ATOMIC_INIT(0),
	.down_sf_step = ATOMIC_INIT(0),
	.force_freqs_step = ATOMIC_INIT(0),
	.onecoresuspend = ATOMIC_INIT(0),
};

static int num_rate;

static int freqs_step[16][4]={
    {1600000,1500000,1500000,1500000},
    {1500000,1400000,1300000,1300000},
    {1400000,1300000,1200000,1200000},
    {1300000,1200000,1000000,1000000},
    {1200000,1100000, 800000, 800000},
    {1100000,1000000, 600000, 500000},
    {1000000, 800000, 500000, 200000},
    { 900000, 600000, 400000, 100000},
    { 800000, 500000, 200000, 100000},
    { 700000, 400000, 100000, 100000},
    { 600000, 200000, 100000, 100000},
    { 500000, 100000, 100000, 100000},
    { 400000, 100000, 100000, 100000},
    { 300000, 100000, 100000, 100000},
    { 200000, 100000, 100000, 100000},
	{ 100000, 100000, 100000, 100000}
};

/************************** sysfs interface ************************/

/* cpufreq_darkness Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%d\n", atomic_read(&darkness_tuners_ins.object));		\
}
show_one(sampling_rate, sampling_rate);
show_one(hotplug_enable, hotplug_enable);
show_one(cpu_up_rate, cpu_up_rate);
show_one(cpu_down_rate, cpu_down_rate);
show_one(up_load, up_load);
show_one(down_load, down_load);
show_one(up_sf_step, up_sf_step);
show_one(down_sf_step, down_sf_step);
show_one(force_freqs_step, force_freqs_step);
show_one(onecoresuspend, onecoresuspend);
show_one(min_freq_limit, min_freq_limit);
show_one(max_freq_limit, max_freq_limit);

#define show_hotplug_param(file_name, num_core, up_down)		\
static ssize_t show_##file_name##_##num_core##_##up_down		\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%d\n", atomic_read(&file_name[num_core - 1][up_down]));	\
}

#define store_hotplug_param(file_name, num_core, up_down)		\
static ssize_t store_##file_name##_##num_core##_##up_down		\
(struct kobject *kobj, struct attribute *attr,				\
	const char *buf, size_t count)					\
{									\
	unsigned int input;						\
	int ret;							\
	ret = sscanf(buf, "%u", &input);				\
	if (ret != 1)							\
		return -EINVAL;						\
	if (input == atomic_read(&file_name[num_core - 1][up_down])) {		\
		return count;	\
	}	\
	atomic_set(&file_name[num_core - 1][up_down], input);			\
	return count;							\
}

show_hotplug_param(hotplug_freq, 1, 1);
show_hotplug_param(hotplug_freq, 2, 0);
#ifndef CONFIG_CPU_EXYNOS4210
show_hotplug_param(hotplug_freq, 2, 1);
show_hotplug_param(hotplug_freq, 3, 0);
show_hotplug_param(hotplug_freq, 3, 1);
show_hotplug_param(hotplug_freq, 4, 0);
#endif

store_hotplug_param(hotplug_freq, 1, 1);
store_hotplug_param(hotplug_freq, 2, 0);
#ifndef CONFIG_CPU_EXYNOS4210
store_hotplug_param(hotplug_freq, 2, 1);
store_hotplug_param(hotplug_freq, 3, 0);
store_hotplug_param(hotplug_freq, 3, 1);
store_hotplug_param(hotplug_freq, 4, 0);
#endif

define_one_global_rw(hotplug_freq_1_1);
define_one_global_rw(hotplug_freq_2_0);
#ifndef CONFIG_CPU_EXYNOS4210
define_one_global_rw(hotplug_freq_2_1);
define_one_global_rw(hotplug_freq_3_0);
define_one_global_rw(hotplug_freq_3_1);
define_one_global_rw(hotplug_freq_4_0);
#endif

/* sampling_rate */
static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(input,10000);
	
	if (input == atomic_read(&darkness_tuners_ins.sampling_rate))
		return count;

	atomic_set(&darkness_tuners_ins.sampling_rate,input);

	return count;
}

/* hotplug_enable */
static ssize_t store_hotplug_enable(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = input > 0; 

	if (atomic_read(&darkness_tuners_ins.hotplug_enable) == input)
		return count;

	atomic_set(&darkness_tuners_ins.hotplug_enable, input);

	return count;
}

/* cpu_up_rate */
static ssize_t store_cpu_up_rate(struct kobject *a, struct attribute *b,
				 const char *buf, size_t count)
{
	int input;
	int ret;
	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,MAX_HOTPLUG_RATE),1);

	if (input == atomic_read(&darkness_tuners_ins.cpu_up_rate))
		return count;

	atomic_set(&darkness_tuners_ins.cpu_up_rate,input);

	return count;
}

/* cpu_down_rate */
static ssize_t store_cpu_down_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,MAX_HOTPLUG_RATE),1);

	if (input == atomic_read(&darkness_tuners_ins.cpu_down_rate))
		return count;

	atomic_set(&darkness_tuners_ins.cpu_down_rate,input);
	return count;
}

/* up_load */
static ssize_t store_up_load(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,101),0);

	if (input == atomic_read(&darkness_tuners_ins.up_load))
		return count;

	atomic_set(&darkness_tuners_ins.up_load,input);

	return count;
}

/* down_load */
static ssize_t store_down_load(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;
	
	input = max(min(input,101),0);

	if (input == atomic_read(&darkness_tuners_ins.down_load))
		return count;

	atomic_set(&darkness_tuners_ins.down_load,input);

	return count;
}

/* up_sf_step */
static ssize_t store_up_sf_step(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,99),0);

	if (input == atomic_read(&darkness_tuners_ins.up_sf_step))
		return count;

	 atomic_set(&darkness_tuners_ins.up_sf_step,input);

	return count;
}

/* down_sf_step */
static ssize_t store_down_sf_step(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,99),0);

	if (input == atomic_read(&darkness_tuners_ins.down_sf_step))
		return count;

	atomic_set(&darkness_tuners_ins.down_sf_step,input);

	return count;
}

/* force_freqs_step */
static ssize_t store_force_freqs_step(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;
	
	input = max(min(input,3),0);

	if (input == atomic_read(&darkness_tuners_ins.force_freqs_step))
		return count;

	atomic_set(&darkness_tuners_ins.force_freqs_step,input);

	return count;
}

/* onecoresuspend */
static ssize_t store_onecoresuspend(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = input > 0; 

	if (atomic_read(&darkness_tuners_ins.onecoresuspend) == input)
		return count;

	atomic_set(&darkness_tuners_ins.onecoresuspend, input);

	return count;
}

/* min_freq_limit */
static ssize_t store_min_freq_limit(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;
	
	input = max(min(input,atomic_read(&darkness_tuners_ins.max_freq_limit)),0);

	if (input == atomic_read(&darkness_tuners_ins.min_freq_limit))
		return count;

	atomic_set(&darkness_tuners_ins.min_freq_limit,input);

	return count;
}

/* max_freq_limit */
static ssize_t store_max_freq_limit(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;
	
	input = max(min(input,1600000),atomic_read(&darkness_tuners_ins.min_freq_limit));

	if (input == atomic_read(&darkness_tuners_ins.max_freq_limit))
		return count;

	atomic_set(&darkness_tuners_ins.max_freq_limit,input);

	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(hotplug_enable);
define_one_global_rw(cpu_up_rate);
define_one_global_rw(cpu_down_rate);
define_one_global_rw(up_load);
define_one_global_rw(down_load);
define_one_global_rw(up_sf_step);
define_one_global_rw(down_sf_step);
define_one_global_rw(force_freqs_step);
define_one_global_rw(onecoresuspend);
define_one_global_rw(min_freq_limit);
define_one_global_rw(max_freq_limit);

static struct attribute *darkness_attributes[] = {
	&sampling_rate.attr,
	&hotplug_enable.attr,
	&hotplug_freq_1_1.attr,
	&hotplug_freq_2_0.attr,
#ifndef CONFIG_CPU_EXYNOS4210
	&hotplug_freq_2_1.attr,
	&hotplug_freq_3_0.attr,
	&hotplug_freq_3_1.attr,
	&hotplug_freq_4_0.attr,
#endif
	&cpu_up_rate.attr,
	&cpu_down_rate.attr,
	&up_load.attr,
	&down_load.attr,
	&up_sf_step.attr,
	&down_sf_step.attr,
	&force_freqs_step.attr,
	&onecoresuspend.attr,
	&min_freq_limit.attr,
	&max_freq_limit.attr,
	NULL
};

static struct attribute_group darkness_attr_group = {
	.attrs = darkness_attributes,
	.name = "darkness",
};

/************************** sysfs end ************************/

static void darkness_check_cpu(struct cpufreq_darkness_cpuinfo *this_darkness_cpuinfo)
{
	int up_rate = atomic_read(&darkness_tuners_ins.cpu_up_rate);
	int down_rate = atomic_read(&darkness_tuners_ins.cpu_down_rate);
	bool onecoresuspend = atomic_read(&darkness_tuners_ins.onecoresuspend) > 0;
	bool hotplug_enable = atomic_read(&darkness_tuners_ins.hotplug_enable) > 0;
	int force_freq_steps = atomic_read(&darkness_tuners_ins.force_freqs_step);
	unsigned int min_freq = atomic_read(&darkness_tuners_ins.min_freq_limit);
	unsigned int max_freq = atomic_read(&darkness_tuners_ins.max_freq_limit);
	int up_sf_step = atomic_read(&darkness_tuners_ins.up_sf_step);
	int down_sf_step = atomic_read(&darkness_tuners_ins.down_sf_step);
	unsigned int next_freq[NR_CPUS];
	int cur_load[NR_CPUS];
	int num_core = num_online_cpus();
	unsigned int j,i;

	for_each_online_cpu(j) {
		struct cpufreq_darkness_cpuinfo *j_darkness_cpuinfo = &per_cpu(od_darkness_cpuinfo, j);
		struct cpufreq_policy *cpu_policy = per_cpu(cpufreq_cpu_data, j);
		u64 cur_busy_time, cur_idle_time;
		unsigned int busy_time, idle_time;
		unsigned int tmp_freq;

		cur_busy_time = cputime_to_usecs(kcpustat_cpu(j).cpustat[CPUTIME_USER] + kcpustat_cpu(j).cpustat[CPUTIME_SYSTEM]
						+ kcpustat_cpu(j).cpustat[CPUTIME_IRQ] + kcpustat_cpu(j).cpustat[CPUTIME_SOFTIRQ]
						+ kcpustat_cpu(j).cpustat[CPUTIME_STEAL] + kcpustat_cpu(j).cpustat[CPUTIME_NICE]);

		cur_idle_time = cputime_to_usecs(kcpustat_cpu(j).cpustat[CPUTIME_IDLE] + kcpustat_cpu(j).cpustat[CPUTIME_IOWAIT]);

		busy_time = (unsigned int)
				(cur_busy_time - j_darkness_cpuinfo->prev_cpu_busy);
		j_darkness_cpuinfo->prev_cpu_busy = cur_busy_time;

		idle_time = (unsigned int)
				(cur_idle_time - j_darkness_cpuinfo->prev_cpu_idle);
		j_darkness_cpuinfo->prev_cpu_idle = cur_idle_time;

		/*printk(KERN_ERR "TIMER CPU[%u], wall[%u], idle[%u]\n",j, busy_time + idle_time, idle_time);*/
		if (!cpu_policy || busy_time + idle_time == 0) { /*if busy_time and idle_time are 0, evaluate cpu load next time*/
			hotplug_enable = false;
			continue;
		}
		cur_load[j] = busy_time ? (100 * busy_time) / (busy_time + idle_time) : 1;/*if busy_time is 0 cpu_load is equal to 1*/
		/* Checking Frequency Limit */
		if (max_freq > cpu_policy->max || max_freq < cpu_policy->min)
			max_freq = cpu_policy->max;
		if (min_freq < cpu_policy->min || min_freq > cpu_policy->max)
			min_freq = cpu_policy->min;
		/* CPUs Online Scale Frequency*/
		tmp_freq = max(min(cur_load[j] * (max_freq / 100), max_freq), min_freq);
		if (force_freq_steps == 0) {
			next_freq[j] = (tmp_freq / 100000) * 100000;
			if ((next_freq[j] > cpu_policy->cur
				&& (tmp_freq % 100000 > up_sf_step * 1000))
				|| (next_freq[j] < cpu_policy->cur 
				&& (tmp_freq % 100000 > down_sf_step * 1000))) {
					next_freq[j] += 100000;
			}
		} else {
			for (i = 0; i < 16; i++) {
				if (tmp_freq >= freqs_step[i][force_freq_steps]) {
					next_freq[j] = freqs_step[i][force_freq_steps];
					break;
				}
			}
		}		
		/*printk(KERN_ERR "FREQ CALC.: CPU[%u], load[%d], target freq[%u], cur freq[%u], min freq[%u], max_freq[%u]\n",j, cur_load[j], next_freq[j], cpu_policy->cur, cpu_policy->min, max_freq); */
		if (next_freq[j] != cpu_policy->cur) {
			__cpufreq_driver_target(cpu_policy, next_freq[j], CPUFREQ_RELATION_L);
		}
	}

	/* set num_rate used */
	++num_rate;

	if (hotplug_enable) {
		/*Check for CPU hotplug*/
		if (!onecoresuspend && num_rate % up_rate == 0 && num_core < NR_CPUS) {
#ifndef CONFIG_CPU_EXYNOS4210
			if (cur_load[num_core - 1] >= atomic_read(&darkness_tuners_ins.up_load)
				&& next_freq[num_core - 1] >= atomic_read(&hotplug_freq[num_core - 1][HOTPLUG_UP_INDEX])) {
				/* printk(KERN_ERR "[HOTPLUG IN] %s %u>=%u\n",
					__func__, cur_freq, up_freq); */
				if (!cpu_online(num_core)) {
					cpu_up(num_core);
					num_rate = 0;
				}
			}
#else
			if (cur_load[0] >= atomic_read(&darkness_tuners_ins.up_load)
				&& next_freq[0] >= atomic_read(&hotplug_freq[0][HOTPLUG_UP_INDEX])) {
				/* printk(KERN_ERR "[HOTPLUG IN] %s %u>=%u\n",
					__func__, cur_freq, up_freq); */
				if (!cpu_online(1)) {
					cpu_up(1);
					num_rate = 0;
				}
			}
#endif
		} else if (num_rate % down_rate == 0 && num_core > 1) {
#ifndef CONFIG_CPU_EXYNOS4210	
			if (onecoresuspend 
				|| cur_load[num_core - 1] < atomic_read(&darkness_tuners_ins.down_load) 
				|| next_freq[num_core - 1] <= atomic_read(&hotplug_freq[num_core - 1][HOTPLUG_DOWN_INDEX])) {
				/* printk(KERN_ERR "[HOTPLUG OUT] %s %u<=%u\n",
					__func__, cur_freq, down_freq); */
				if (cpu_online(num_core - 1)) {
					cpu_down(num_core - 1);
					num_rate = 0;
				}
			}
#else
			if (onecoresuspend 
				|| cur_load[1] < atomic_read(&darkness_tuners_ins.down_load)
				|| next_freq[1] <= atomic_read(&hotplug_freq[1][HOTPLUG_DOWN_INDEX])) {
				/* printk(KERN_ERR "[HOTPLUG OUT] %s %u<=%u\n",
					__func__, cur_freq, down_freq); */
				if (cpu_online(1)) {
					cpu_down(1);
					num_rate = 0;
				}
			}
#endif
		}
	}
	if (num_rate == max(up_rate, down_rate)) {
		num_rate = 0;
	}
}

static void do_darkness_timer(struct work_struct *work)
{
	struct cpufreq_darkness_cpuinfo *darkness_cpuinfo =
		container_of(work, struct cpufreq_darkness_cpuinfo, work.work);
	int delay;

	mutex_lock(&timer_mutex);
	darkness_check_cpu(darkness_cpuinfo);
	/* We want all CPUs to do sampling nearly on
	 * same jiffy
	 */
	delay = usecs_to_jiffies(atomic_read(&darkness_tuners_ins.sampling_rate));
	if (num_online_cpus() > 1) {
		delay -= jiffies % delay;
	}

	mod_delayed_work_on(darkness_cpuinfo->cpu, dvfs_workqueue, &darkness_cpuinfo->work, delay);
	mutex_unlock(&timer_mutex);
}

static int cpufreq_governor_darkness(struct cpufreq_policy *policy,
				unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpufreq_darkness_cpuinfo *this_darkness_cpuinfo = &per_cpu(od_darkness_cpuinfo, cpu);
	struct cpufreq_policy *cpu_policy = per_cpu(cpufreq_cpu_data, cpu);
	unsigned int j;
	int rc;

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&darkness_mutex);
		num_rate = 0;

		darkness_enable++;
		for_each_possible_cpu(j) {
			struct cpufreq_darkness_cpuinfo *j_darkness_cpuinfo = &per_cpu(od_darkness_cpuinfo, j);
			per_cpu(cpufreq_cpu_data, j) = policy;
			j_darkness_cpuinfo->prev_cpu_busy = cputime_to_usecs(kcpustat_cpu(j).cpustat[CPUTIME_USER] + kcpustat_cpu(j).cpustat[CPUTIME_SYSTEM]
						+ kcpustat_cpu(j).cpustat[CPUTIME_IRQ] + kcpustat_cpu(j).cpustat[CPUTIME_SOFTIRQ]
						+ kcpustat_cpu(j).cpustat[CPUTIME_STEAL] + kcpustat_cpu(j).cpustat[CPUTIME_NICE]);

			j_darkness_cpuinfo->prev_cpu_idle = cputime_to_usecs(kcpustat_cpu(j).cpustat[CPUTIME_IDLE] + kcpustat_cpu(j).cpustat[CPUTIME_IOWAIT]);
		}
		this_darkness_cpuinfo->cpu = cpu;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (darkness_enable == 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&darkness_attr_group);
			if (rc) {
				mutex_unlock(&darkness_mutex);
				return rc;
			}
			atomic_set(&darkness_tuners_ins.min_freq_limit,policy->min);
			atomic_set(&darkness_tuners_ins.max_freq_limit,policy->max);
		}
		mutex_unlock(&darkness_mutex);

		mutex_init(&timer_mutex);
		INIT_DEFERRABLE_WORK(&this_darkness_cpuinfo->work, do_darkness_timer);
		mod_delayed_work_on(this_darkness_cpuinfo->cpu, dvfs_workqueue, &this_darkness_cpuinfo->work, 0);

		break;

	case CPUFREQ_GOV_STOP:
		cancel_delayed_work(&this_darkness_cpuinfo->work);
		mutex_destroy(&timer_mutex);

		mutex_lock(&darkness_mutex);
		darkness_enable--;
		for_each_possible_cpu(j) {
			per_cpu(cpufreq_cpu_data, j) = NULL;
		}

		if (!darkness_enable) {
			sysfs_remove_group(cpufreq_global_kobject,
					   &darkness_attr_group);
		}
		mutex_unlock(&darkness_mutex);
		
		break;

	case CPUFREQ_GOV_LIMITS:
		if(!cpu_policy) {
			break;
		}
		mutex_lock(&timer_mutex);
		if (policy->max < cpu_policy->cur)
			__cpufreq_driver_target(cpu_policy,
				policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > cpu_policy->cur)
			__cpufreq_driver_target(cpu_policy,
				policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&timer_mutex);

		break;
	}
	return 0;
}

static int __init cpufreq_gov_darkness_init(void)
{
	int ret;

	dvfs_workqueue = create_singlethread_workqueue("kdarkness");
	if (!dvfs_workqueue) {
		pr_err("%s cannot create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_free;
	}

	ret = cpufreq_register_governor(&cpufreq_gov_darkness);
	if (ret)
		goto err_reg;

	return ret;

err_reg:
	destroy_workqueue(dvfs_workqueue);
err_free:
	kfree(&darkness_tuners_ins);
	kfree(&hotplug_freq);
	return ret;
}

static void __exit cpufreq_gov_darkness_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_darkness);
	destroy_workqueue(dvfs_workqueue);
	kfree(&darkness_tuners_ins);
	kfree(&hotplug_freq);
}

MODULE_AUTHOR("Alucard24@XDA");
MODULE_DESCRIPTION("'cpufreq_darkness' - A dynamic cpufreq/cpuhotplug governor v.1.5");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_darkness
fs_initcall(cpufreq_gov_darkness_init);
#else
module_init(cpufreq_gov_darkness_init);
#endif
module_exit(cpufreq_gov_darkness_exit);
