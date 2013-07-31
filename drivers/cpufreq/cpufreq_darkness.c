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
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define MAX_HOTPLUG_RATE		(40)
#define HOTPLUG_DOWN_INDEX		(0)
#define HOTPLUG_UP_INDEX		(1)

static unsigned int hotplug_freq[4][2] = {
	{0, 500000},
	{200000, 500000},
	{200000, 500000},
	{200000, 0}
};

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
	atomic_t earlysuspend;
} darkness_tuners_ins = {
	.sampling_rate = ATOMIC_INIT(60000),
	.hotplug_enable = ATOMIC_INIT(0),
	.cpu_up_rate = ATOMIC_INIT(10),
	.cpu_down_rate = ATOMIC_INIT(5),
	.up_load = ATOMIC_INIT(65),
	.up_sf_step = ATOMIC_INIT(0),
	.down_sf_step = ATOMIC_INIT(0),
	.force_freqs_step = ATOMIC_INIT(0),
	.earlysuspend = ATOMIC_INIT(0),
};

/*
 * History of CPU usage
 */
struct darkness_cpu_usage {
	unsigned int freq[NR_CPUS];
	int load[NR_CPUS];
};

struct darkness_cpu_usage_history {
	struct darkness_cpu_usage usage[MAX_HOTPLUG_RATE];
	int num_hist;
};

static struct darkness_cpu_usage_history *hotplug_history;

static int freqs_step[16][4]={
    {1600000,1500000,1500000,1500000},
    {1500000,1400000,1300000,1300000},
    {1400000,1300000,1300000,1300000},
    {1300000,1200000,1200000,1200000},
    {1200000,1100000,1200000,1200000},
    {1100000,1000000,1000000,1000000},
    {1000000, 800000, 800000,1000000},
    { 900000, 800000, 800000, 800000},
    { 800000, 600000, 600000, 800000},
    { 700000, 600000, 600000, 500000},
    { 600000, 500000, 500000, 500000},
    { 500000, 500000, 500000, 500000},
    { 400000, 400000, 400000, 200000},
    { 300000, 200000, 200000, 200000},
    { 200000, 200000, 200000, 200000},
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

#define show_hotplug_param(file_name, num_core, up_down)		\
static ssize_t show_##file_name##_##num_core##_##up_down		\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", file_name[num_core - 1][up_down]);	\
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
	file_name[num_core - 1][up_down] = input;			\
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

define_one_global_rw(sampling_rate);
define_one_global_rw(hotplug_enable);
define_one_global_rw(cpu_up_rate);
define_one_global_rw(cpu_down_rate);
define_one_global_rw(up_load);
define_one_global_rw(down_load);
define_one_global_rw(up_sf_step);
define_one_global_rw(down_sf_step);
define_one_global_rw(force_freqs_step);

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
	NULL
};

static struct attribute_group darkness_attr_group = {
	.attrs = darkness_attributes,
	.name = "darkness",
};

/************************** sysfs end ************************/

static int check_up(bool earlysuspend)
{
	int up_rate = atomic_read(&darkness_tuners_ins.cpu_up_rate);
	int up_load = atomic_read(&darkness_tuners_ins.up_load);
	int online = num_online_cpus();
	int num_hist = hotplug_history->num_hist;
	unsigned int up_freq = hotplug_freq[online - 1][HOTPLUG_UP_INDEX];
	unsigned int cur_freq = hotplug_history->usage[num_hist - 1].freq[0];
	int cur_load = hotplug_history->usage[num_hist - 1].load[0];
	int i;

	if (online == num_possible_cpus() || earlysuspend)
		return 0;

	if (num_hist == 0 || num_hist % up_rate)
		return 0;

	if (cur_freq >= up_freq
		&& cur_load >= up_load) {
		/* printk(KERN_ERR "[HOTPLUG IN] %s %u>=%u\n",
			__func__, cur_freq, up_freq); */
		hotplug_history->num_hist = 0;
		return 1;
	}
	return 0;
}

static int check_down(bool earlysuspend)
{
	int down_rate = atomic_read(&darkness_tuners_ins.cpu_down_rate);
	int down_load = atomic_read(&darkness_tuners_ins.down_load);
	int online = num_online_cpus();
	int num_hist = hotplug_history->num_hist;
	unsigned int down_freq = hotplug_freq[online - 1][HOTPLUG_DOWN_INDEX];
	unsigned int cur_freq = hotplug_history->usage[num_hist - 1].freq[1];
	int cur_load = hotplug_history->usage[num_hist - 1].load[1];
	int i;

	if (online == 1)
		return 0;

	if (earlysuspend)
		return 1;

	if (num_hist == 0 || num_hist % down_rate)
		return 0;

	if (cur_freq <= down_freq
		|| cur_load < down_load) {
		/* printk(KERN_ERR "[HOTPLUG OUT] %s %u<=%u\n",
			__func__, cur_freq, down_freq); */
		hotplug_history->num_hist = 0;
		return 1;
	}
	return 0;
}

static unsigned int darkness_frequency_adjust(int next_freq, unsigned int min_freq, unsigned int cur_freq, unsigned int max_freq, int ffs)
{	
	unsigned int scaling_freq_step = next_freq >= cur_freq ? atomic_read(&darkness_tuners_ins.up_sf_step) : atomic_read(&darkness_tuners_ins.down_sf_step);
	unsigned int adjust_freq = (next_freq / 100000) * 100000;
	int i=0;

	if (adjust_freq >= max_freq) {
		return max_freq;
	} else if (adjust_freq <= min_freq) {
		return min_freq;
	} else if (adjust_freq == cur_freq) {
		return cur_freq;
	} else if (ffs == 0 && (next_freq % 100000 > scaling_freq_step * 1000)) {
		adjust_freq += 100000;
	} else if (ffs > 0) {
		for (i = 0; i < 16; i++) {
			if (next_freq >= freqs_step[i][ffs]) {
				return freqs_step[i][ffs];
			}
		}
	}
	return adjust_freq;		
}

static void darkness_check_cpu(struct cpufreq_darkness_cpuinfo *this_darkness_cpuinfo)
{
	int max_hotplug_rate = max(atomic_read(&darkness_tuners_ins.cpu_up_rate),atomic_read(&darkness_tuners_ins.cpu_down_rate));
	bool earlysuspend = atomic_read(&darkness_tuners_ins.earlysuspend) > 0;
	bool hotplug_enable = atomic_read(&darkness_tuners_ins.hotplug_enable) > 0;
	int force_freq_steps = atomic_read(&darkness_tuners_ins.force_freqs_step);
	int num_hist = hotplug_history->num_hist;
	unsigned int j;

	for_each_online_cpu(j) {
		struct cpufreq_darkness_cpuinfo *j_darkness_cpuinfo;
		struct cpufreq_policy *cpu_policy = per_cpu(cpufreq_cpu_data, j);
		u64 cur_busy_time, cur_idle_time;
		unsigned int busy_time, idle_time;
		/* Current load across this CPU */
		int cur_load;
		unsigned int next_freq;
		unsigned int max_freq=(!earlysuspend) ? cpu_policy->max : min(cpu_policy->max_suspend,cpu_policy->max);

		j_darkness_cpuinfo = &per_cpu(od_darkness_cpuinfo, j);

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
		if (unlikely(!cpu_policy || busy_time + idle_time == 0)) { /*if busy_time and idle_time are 0, evaluate cpu load next time*/
			continue;
		}
		cur_load = busy_time ? (100 * busy_time) / (busy_time + idle_time) : 1;/*if busy_time is 0 cpu_load is equal to 1*/
		/* CPUs Online Scale Frequency*/
		next_freq = darkness_frequency_adjust((cur_load * (max_freq / 100)), cpu_policy->min, cpu_policy->cur, max_freq, force_freq_steps);
		/* SET CPUs Online history*/
		hotplug_history->usage[num_hist].freq[j] = next_freq;
		hotplug_history->usage[num_hist].load[j] = cur_load;
		/*printk(KERN_ERR "FREQ CALC.: CPU[%u], load[%d], target freq[%u], cur freq[%u], min freq[%u], max_freq[%u]\n",j, cur_load, next_freq, cpu_policy->cur, cpu_policy->min, max_freq); */
		if (next_freq != cpu_policy->cur) {
			__cpufreq_driver_target(cpu_policy, next_freq, CPUFREQ_RELATION_L);
		}
	}

	/* set num_hist used */
	++hotplug_history->num_hist;

	if (hotplug_enable) {
		/*Check for CPU hotplug*/
		if (check_up(earlysuspend)) {
			cpu_up(1);
		} else if (check_down(earlysuspend)) {
			cpu_down(1);
		}
	}
	if (hotplug_history->num_hist == max_hotplug_rate)
		hotplug_history->num_hist = 0;
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

#ifdef CONFIG_HAS_EARLYSUSPEND
static inline void cpufreq_darkness_early_suspend(struct early_suspend *h)
{
	atomic_set(&darkness_tuners_ins.earlysuspend, 1);
}

static inline void cpufreq_darkness_late_resume(struct early_suspend *h)
{
	atomic_set(&darkness_tuners_ins.earlysuspend, 0);
}

static struct early_suspend darkness_early_suspend = {
	.suspend = cpufreq_darkness_early_suspend,
	.resume = cpufreq_darkness_late_resume,
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
};
#endif

static int cpufreq_governor_darkness(struct cpufreq_policy *policy,
				unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpufreq_darkness_cpuinfo *this_darkness_cpuinfo;
	struct cpufreq_policy *cpu_policy;
	unsigned int j;
	int rc;

	this_darkness_cpuinfo = &per_cpu(od_darkness_cpuinfo, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		/* SET POLICY SHARED TYPE AND APPLY MASK TO ALL CPUS */
		policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
		cpumask_setall(policy->cpus);

		hotplug_history->num_hist = 0;

		mutex_lock(&darkness_mutex);

		darkness_enable++;
		for_each_possible_cpu(j) {
			struct cpufreq_darkness_cpuinfo *j_darkness_cpuinfo;
			per_cpu(cpufreq_cpu_data, j) = policy;
			j_darkness_cpuinfo = &per_cpu(od_darkness_cpuinfo, j);
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
			atomic_set(&darkness_tuners_ins.earlysuspend,0);
		}
		mutex_init(&timer_mutex);
		INIT_DEFERRABLE_WORK(&this_darkness_cpuinfo->work, do_darkness_timer);
		mutex_unlock(&darkness_mutex);

		mod_delayed_work_on(this_darkness_cpuinfo->cpu, dvfs_workqueue, &this_darkness_cpuinfo->work, 0);

#ifdef CONFIG_HAS_EARLYSUSPEND
		register_early_suspend(&darkness_early_suspend);
#endif
		break;

	case CPUFREQ_GOV_STOP:
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&darkness_early_suspend);
#endif
		mutex_lock(&darkness_mutex);
		cancel_delayed_work(&this_darkness_cpuinfo->work);
		mutex_destroy(&timer_mutex);
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
		get_online_cpus();
		mutex_lock(&timer_mutex);
		/* NOTHING TO DO JUST WATT */
		cpu_policy = per_cpu(cpufreq_cpu_data, cpu);
		if(!cpu_policy) {
			mutex_unlock(&timer_mutex);
			break;
		}
		if (policy->max < cpu_policy->cur)
			__cpufreq_driver_target(cpu_policy,
				policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > cpu_policy->cur)
			__cpufreq_driver_target(cpu_policy,
				policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&timer_mutex);
		put_online_cpus();

		break;
	}
	return 0;
}

static int __init cpufreq_gov_darkness_init(void)
{
	int ret;

	hotplug_history = kzalloc(sizeof(struct darkness_cpu_usage_history), GFP_KERNEL);
	if (!hotplug_history) {
		pr_err("%s cannot create hotplug history array\n", __func__);
		ret = -ENOMEM;
		goto err_free;
	}

	dvfs_workqueue = create_singlethread_workqueue("kdarkness");
	if (!dvfs_workqueue) {
		pr_err("%s cannot create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_queue;
	}

	ret = cpufreq_register_governor(&cpufreq_gov_darkness);
	if (ret)
		goto err_reg;

	return ret;

err_reg:
	destroy_workqueue(dvfs_workqueue);
err_queue:
	kfree(hotplug_history);
err_free:
	kfree(&darkness_tuners_ins);
	kfree(&hotplug_freq);
	return ret;
}

static void __exit cpufreq_gov_darkness_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_darkness);
	destroy_workqueue(dvfs_workqueue);
	kfree(hotplug_history);
	kfree(&darkness_tuners_ins);
	kfree(&hotplug_freq);
}

MODULE_AUTHOR("Alucard24@XDA");
MODULE_DESCRIPTION("'cpufreq_darkness' - A dynamic cpufreq/cpuhotplug governor v.1.3");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_darkness
fs_initcall(cpufreq_gov_darkness_init);
#else
module_init(cpufreq_gov_darkness_init);
#endif
module_exit(cpufreq_gov_darkness_exit);
