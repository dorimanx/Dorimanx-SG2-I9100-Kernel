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
	unsigned long prev_cpu_user;
	unsigned long prev_cpu_system;
	unsigned long prev_cpu_others;
	unsigned long prev_cpu_idle;
	unsigned long prev_cpu_iowait;
	struct cpufreq_frequency_table *freq_table;
	struct delayed_work work;
	struct cpufreq_policy *cur_policy;
	int cpu;
	unsigned int enable:1;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
/*
 * mutex that serializes governor limit change with
 * do_darkness_timer invocation. We do not want do_darkness_timer to run
 * when user is changing the governor or limits.
 */
static DEFINE_PER_CPU(struct cpufreq_darkness_cpuinfo, od_darkness_cpuinfo);

static unsigned int darkness_enable;	/* number of CPUs using this policy */
/*
 * darkness_mutex protects darkness_enable in governor start/stop.
 */
static DEFINE_MUTEX(darkness_mutex);

static atomic_t min_freq_limit[NR_CPUS];
static atomic_t max_freq_limit[NR_CPUS];
#ifndef CONFIG_CPU_EXYNOS4210
static atomic_t min_freq_limit_sleep[NR_CPUS];
static atomic_t max_freq_limit_sleep[NR_CPUS];
#endif

/* darkness tuners */
static struct darkness_tuners {
	atomic_t sampling_rate;
#ifdef CONFIG_CPU_EXYNOS4210
	atomic_t up_sf_step;
	atomic_t down_sf_step;
	atomic_t force_freqs_step;
#endif
} darkness_tuners_ins = {
	.sampling_rate = ATOMIC_INIT(60000),
#ifdef CONFIG_CPU_EXYNOS4210
	.up_sf_step = ATOMIC_INIT(0),
	.down_sf_step = ATOMIC_INIT(0),
	.force_freqs_step = ATOMIC_INIT(0),
#endif
};

#ifdef CONFIG_CPU_EXYNOS4210
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
#endif

#ifndef CONFIG_CPU_EXYNOS4210
extern bool apget_if_suspended(void);
#endif

/************************** sysfs interface ************************/

/* cpufreq_darkness Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%d\n", atomic_read(&darkness_tuners_ins.object));		\
}
show_one(sampling_rate, sampling_rate);
#ifdef CONFIG_CPU_EXYNOS4210
show_one(up_sf_step, up_sf_step);
show_one(down_sf_step, down_sf_step);
show_one(force_freqs_step, force_freqs_step);
#endif

#define show_freqlimit_param(file_name, cpu)		\
static ssize_t show_##file_name##_##cpu		\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%d\n", atomic_read(&file_name[cpu]));	\
}

#define store_freqlimit_param(file_name, cpu)		\
static ssize_t store_##file_name##_##cpu		\
(struct kobject *kobj, struct attribute *attr,				\
	const char *buf, size_t count)					\
{									\
	unsigned int input;						\
	int ret;							\
	ret = sscanf(buf, "%d", &input);				\
	if (ret != 1)							\
		return -EINVAL;						\
	if (input == atomic_read(&file_name[cpu])) {		\
		return count;	\
	}	\
	atomic_set(&file_name[cpu], input);			\
	return count;							\
}

/* min freq limit for awaking */
show_freqlimit_param(min_freq_limit, 0);
show_freqlimit_param(min_freq_limit, 1);
#if NR_CPUS >= 4
show_freqlimit_param(min_freq_limit, 2);
show_freqlimit_param(min_freq_limit, 3);
#endif
#ifndef CONFIG_CPU_EXYNOS4210
/* min freq limit for sleeping */
show_freqlimit_param(min_freq_limit_sleep, 0);
show_freqlimit_param(min_freq_limit_sleep, 1);
#if NR_CPUS >= 4
show_freqlimit_param(min_freq_limit_sleep, 2);
show_freqlimit_param(min_freq_limit_sleep, 3);
#endif
#endif
/* max freq limit for awaking */
show_freqlimit_param(max_freq_limit, 0);
show_freqlimit_param(max_freq_limit, 1);
#if NR_CPUS >= 4
show_freqlimit_param(max_freq_limit, 2);
show_freqlimit_param(max_freq_limit, 3);
#endif
#ifndef CONFIG_CPU_EXYNOS4210
/* max freq limit for sleeping */
show_freqlimit_param(max_freq_limit_sleep, 0);
show_freqlimit_param(max_freq_limit_sleep, 1);
#if NR_CPUS >= 4
show_freqlimit_param(max_freq_limit_sleep, 2);
show_freqlimit_param(max_freq_limit_sleep, 3);
#endif
#endif
/* min freq limit for awaking */
store_freqlimit_param(min_freq_limit, 0);
store_freqlimit_param(min_freq_limit, 1);
#if NR_CPUS >= 4
store_freqlimit_param(min_freq_limit, 2);
store_freqlimit_param(min_freq_limit, 3);
#endif
#ifndef CONFIG_CPU_EXYNOS4210
/* min freq limit for sleeping */
store_freqlimit_param(min_freq_limit_sleep, 0);
store_freqlimit_param(min_freq_limit_sleep, 1);
#if NR_CPUS >= 4
store_freqlimit_param(min_freq_limit_sleep, 2);
store_freqlimit_param(min_freq_limit_sleep, 3);
#endif
#endif
/* max freq limit for awaking */
store_freqlimit_param(max_freq_limit, 0);
store_freqlimit_param(max_freq_limit, 1);
#if NR_CPUS >= 4
store_freqlimit_param(max_freq_limit, 2);
store_freqlimit_param(max_freq_limit, 3);
#endif
#ifndef CONFIG_CPU_EXYNOS4210
/* max freq limit for sleeping */
store_freqlimit_param(max_freq_limit_sleep, 0);
store_freqlimit_param(max_freq_limit_sleep, 1);
#if NR_CPUS >= 4
store_freqlimit_param(max_freq_limit_sleep, 2);
store_freqlimit_param(max_freq_limit_sleep, 3);
#endif
#endif

define_one_global_rw(min_freq_limit_0);
define_one_global_rw(min_freq_limit_1);
#if NR_CPUS >= 4
define_one_global_rw(min_freq_limit_2);
define_one_global_rw(min_freq_limit_3);
#endif
#ifndef CONFIG_CPU_EXYNOS4210
define_one_global_rw(min_freq_limit_sleep_0);
define_one_global_rw(min_freq_limit_sleep_1);
#if NR_CPUS >= 4
define_one_global_rw(min_freq_limit_sleep_2);
define_one_global_rw(min_freq_limit_sleep_3);
#endif
#endif
define_one_global_rw(max_freq_limit_0);
define_one_global_rw(max_freq_limit_1);
#if NR_CPUS >= 4
define_one_global_rw(max_freq_limit_2);
define_one_global_rw(max_freq_limit_3);
#endif
#ifndef CONFIG_CPU_EXYNOS4210
define_one_global_rw(max_freq_limit_sleep_0);
define_one_global_rw(max_freq_limit_sleep_1);
#if NR_CPUS >= 4
define_one_global_rw(max_freq_limit_sleep_2);
define_one_global_rw(max_freq_limit_sleep_3);
#endif
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
#ifdef CONFIG_CPU_EXYNOS4210
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
#endif

define_one_global_rw(sampling_rate);
#ifdef CONFIG_CPU_EXYNOS4210
define_one_global_rw(up_sf_step);
define_one_global_rw(down_sf_step);
define_one_global_rw(force_freqs_step);
#endif

static struct attribute *darkness_attributes[] = {
	&sampling_rate.attr,
#ifdef CONFIG_CPU_EXYNOS4210
	&up_sf_step.attr,
	&down_sf_step.attr,
	&force_freqs_step.attr,
#endif
	&min_freq_limit_0.attr,
	&min_freq_limit_1.attr,
#if NR_CPUS >= 4
	&min_freq_limit_2.attr,
	&min_freq_limit_3.attr,
#endif
#ifndef CONFIG_CPU_EXYNOS4210
	&min_freq_limit_sleep_0.attr,
	&min_freq_limit_sleep_1.attr,
#if NR_CPUS >= 4
	&min_freq_limit_sleep_2.attr,
	&min_freq_limit_sleep_3.attr,
#endif
#endif
	&max_freq_limit_0.attr,
	&max_freq_limit_1.attr,
#if NR_CPUS >= 4
	&max_freq_limit_2.attr,
	&max_freq_limit_3.attr,
#endif
#ifndef CONFIG_CPU_EXYNOS4210
	&max_freq_limit_sleep_0.attr,
	&max_freq_limit_sleep_1.attr,
#if NR_CPUS >= 4
	&max_freq_limit_sleep_2.attr,
	&max_freq_limit_sleep_3.attr,
#endif
#endif
	NULL
};

static struct attribute_group darkness_attr_group = {
	.attrs = darkness_attributes,
	.name = "darkness",
};

/************************** sysfs end ************************/

static void darkness_check_cpu(struct cpufreq_darkness_cpuinfo *this_darkness_cpuinfo)
{
	struct cpufreq_policy *cpu_policy;
#ifndef CONFIG_CPU_EXYNOS4210
	bool earlysuspend = apget_if_suspended();
#endif
	unsigned int cpu = this_darkness_cpuinfo->cpu;
#ifdef CONFIG_CPU_EXYNOS4210
	unsigned int min_freq = atomic_read(&min_freq_limit[cpu]);
	unsigned int max_freq = atomic_read(&max_freq_limit[cpu]);
	int up_sf_step = atomic_read(&darkness_tuners_ins.up_sf_step);
	int down_sf_step = atomic_read(&darkness_tuners_ins.down_sf_step);
	int force_freq_steps = atomic_read(&darkness_tuners_ins.force_freqs_step);
	unsigned int tmp_freq = 0;
	unsigned int i;
#else
	unsigned int min_freq = !earlysuspend ? atomic_read(&min_freq_limit[cpu]) : atomic_read(&min_freq_limit_sleep[cpu]);
	unsigned int max_freq = !earlysuspend ? atomic_read(&max_freq_limit[cpu]) : atomic_read(&max_freq_limit_sleep[cpu]);
#endif
	unsigned long cur_user_time, cur_system_time, cur_others_time, cur_idle_time, cur_iowait_time;
	unsigned int busy_time, idle_time;
	unsigned int index = 0;
	unsigned int next_freq = 0;
	int cur_load = -1;

	cpu_policy = this_darkness_cpuinfo->cur_policy;

	cur_user_time = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_USER]);
	cur_system_time = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM]);
	cur_others_time = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ] + kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ]
																	+ kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL] + kcpustat_cpu(cpu).cpustat[CPUTIME_NICE]);

	cur_idle_time = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE]);
	cur_iowait_time = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT]);

	busy_time = (unsigned int)
			((cur_user_time - this_darkness_cpuinfo->prev_cpu_user) +
			 (cur_system_time - this_darkness_cpuinfo->prev_cpu_system) +
			 (cur_others_time - this_darkness_cpuinfo->prev_cpu_others));
	this_darkness_cpuinfo->prev_cpu_user = cur_user_time;
	this_darkness_cpuinfo->prev_cpu_system = cur_system_time;
	this_darkness_cpuinfo->prev_cpu_others = cur_others_time;

	idle_time = (unsigned int)
			((cur_idle_time - this_darkness_cpuinfo->prev_cpu_idle) + 
			 (cur_iowait_time - this_darkness_cpuinfo->prev_cpu_iowait));
	this_darkness_cpuinfo->prev_cpu_idle = cur_idle_time;
	this_darkness_cpuinfo->prev_cpu_iowait = cur_iowait_time;

	/*printk(KERN_ERR "TIMER CPU[%u], wall[%u], idle[%u]\n",cpu, busy_time + idle_time, idle_time);*/
	if (busy_time + idle_time > 0) { /*if busy_time and idle_time are 0, evaluate cpu load next time*/
		cur_load = busy_time ? (100 * busy_time) / (busy_time + idle_time) : 1;/*if busy_time is 0 cpu_load is equal to 1*/
		/* Checking Frequency Limit */
		if (max_freq > cpu_policy->max || max_freq < cpu_policy->min)
			max_freq = cpu_policy->max;
		if (min_freq < cpu_policy->min || min_freq > cpu_policy->max)
			min_freq = cpu_policy->min;
		/* CPUs Online Scale Frequency*/
#ifdef CONFIG_CPU_EXYNOS4210
		tmp_freq = max(min(cur_load * (max_freq / 100), max_freq), min_freq);
		if (force_freq_steps == 0) {
			next_freq = (tmp_freq / 100000) * 100000;
			if ((next_freq > cpu_policy->cur
				&& (tmp_freq % 100000 > up_sf_step * 1000))
				|| (next_freq < cpu_policy->cur
				&& (tmp_freq % 100000 > down_sf_step * 1000))) {
					next_freq += 100000;
			}
		} else {
			for (i = 0; i < 16; i++) {
				if (tmp_freq >= freqs_step[i][force_freq_steps]) {
					next_freq = freqs_step[i][force_freq_steps];
					break;
				}
			}
		}
#else
		next_freq = max(min(cur_load * (max_freq / 100), max_freq), min_freq);
		cpufreq_frequency_table_target(cpu_policy, this_darkness_cpuinfo->freq_table, next_freq,
			CPUFREQ_RELATION_H, &index);
		next_freq = this_darkness_cpuinfo->freq_table[index].frequency;
#endif
		/*printk(KERN_ERR "FREQ CALC.: CPU[%u], load[%d], target freq[%u], cur freq[%u], min freq[%u], max_freq[%u]\n",cpu, cur_load, next_freq, cpu_policy->cur, cpu_policy->min, max_freq);*/
		if (next_freq != cpu_policy->cur) {
			__cpufreq_driver_target(cpu_policy, next_freq, CPUFREQ_RELATION_L);
		}
	}

}

static void do_darkness_timer(struct work_struct *work)
{
	struct cpufreq_darkness_cpuinfo *darkness_cpuinfo =
		container_of(work, struct cpufreq_darkness_cpuinfo, work.work);
	int delay;
	unsigned int cpu = darkness_cpuinfo->cpu;

	mutex_lock(&darkness_cpuinfo->timer_mutex);
	darkness_check_cpu(darkness_cpuinfo);
	/* We want all CPUs to do sampling nearly on
	 * same jiffy
	 */
	delay = usecs_to_jiffies(atomic_read(&darkness_tuners_ins.sampling_rate));
	if (num_online_cpus() > 1) {
		delay -= jiffies % delay;
	}

#ifdef CONFIG_CPU_EXYNOS4210
	mod_delayed_work_on(cpu, system_wq, &darkness_cpuinfo->work, delay);
#else
	queue_delayed_work_on(cpu, system_wq, &darkness_cpuinfo->work, delay);
#endif
	mutex_unlock(&darkness_cpuinfo->timer_mutex);
}

static int cpufreq_governor_darkness(struct cpufreq_policy *policy,
				unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpufreq_darkness_cpuinfo *this_darkness_cpuinfo;
	int rc, delay;

	this_darkness_cpuinfo = &per_cpu(od_darkness_cpuinfo, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&darkness_mutex);

		this_darkness_cpuinfo->cur_policy = policy;

		this_darkness_cpuinfo->prev_cpu_user = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_USER]);
		this_darkness_cpuinfo->prev_cpu_system = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM]);
		this_darkness_cpuinfo->prev_cpu_others = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ] + kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ]
																	+ kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL] + kcpustat_cpu(cpu).cpustat[CPUTIME_NICE]);

		this_darkness_cpuinfo->prev_cpu_idle = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE]);
		this_darkness_cpuinfo->prev_cpu_iowait = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT]);

		this_darkness_cpuinfo->freq_table = cpufreq_frequency_get_table(cpu);
		this_darkness_cpuinfo->cpu = cpu;

		mutex_init(&this_darkness_cpuinfo->timer_mutex);

		darkness_enable++;
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
		}

		if (atomic_read(&min_freq_limit[cpu]) == 0)
			atomic_set(&min_freq_limit[cpu], policy->min);

		if (atomic_read(&max_freq_limit[cpu]) == 0)
			atomic_set(&max_freq_limit[cpu], policy->max);
#ifndef CONFIG_CPU_EXYNOS4210
		if (atomic_read(&min_freq_limit_sleep[cpu]) == 0)
			atomic_set(&min_freq_limit_sleep[cpu], policy->min);

		if (atomic_read(&max_freq_limit_sleep[cpu]) == 0)
			atomic_set(&max_freq_limit_sleep[cpu], 702000);
#endif
		mutex_unlock(&darkness_mutex);

		delay=usecs_to_jiffies(atomic_read(&darkness_tuners_ins.sampling_rate));
		if (num_online_cpus() > 1) {
			delay -= jiffies % delay;
		}

		this_darkness_cpuinfo->enable = 1;
#ifdef CONFIG_CPU_EXYNOS4210
		INIT_DEFERRABLE_WORK(&this_darkness_cpuinfo->work, do_darkness_timer);
		mod_delayed_work_on(this_darkness_cpuinfo->cpu, system_wq, &this_darkness_cpuinfo->work, delay);
#else
		INIT_DELAYED_WORK_DEFERRABLE(&this_darkness_cpuinfo->work, do_darkness_timer);
		queue_delayed_work_on(this_darkness_cpuinfo->cpu, system_wq, &this_darkness_cpuinfo->work, delay);
#endif

		break;

	case CPUFREQ_GOV_STOP:
		this_darkness_cpuinfo->enable = 0;
		cancel_delayed_work_sync(&this_darkness_cpuinfo->work);

		mutex_lock(&darkness_mutex);
		darkness_enable--;
		mutex_destroy(&this_darkness_cpuinfo->timer_mutex);

		if (!darkness_enable) {
			sysfs_remove_group(cpufreq_global_kobject,
					   &darkness_attr_group);			
		}
		mutex_unlock(&darkness_mutex);
		
		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_darkness_cpuinfo->timer_mutex);
		if (policy->max < this_darkness_cpuinfo->cur_policy->cur)
			__cpufreq_driver_target(this_darkness_cpuinfo->cur_policy,
				policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > this_darkness_cpuinfo->cur_policy->cur)
			__cpufreq_driver_target(this_darkness_cpuinfo->cur_policy,
				policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&this_darkness_cpuinfo->timer_mutex);

		break;
	}
	return 0;
}

static int __init cpufreq_gov_darkness_init(void)
{
	return cpufreq_register_governor(&cpufreq_gov_darkness);
}

static void __exit cpufreq_gov_darkness_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_darkness);
}

MODULE_AUTHOR("Alucard24@XDA");
MODULE_DESCRIPTION("'cpufreq_darkness' - A dynamic cpufreq/cpuhotplug governor v3.5 (SnapDragon)");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_DARKNESS
fs_initcall(cpufreq_gov_darkness_init);
#else
module_init(cpufreq_gov_darkness_init);
#endif
module_exit(cpufreq_gov_darkness_exit);
