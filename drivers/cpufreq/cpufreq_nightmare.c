/*
 *  drivers/cpufreq/cpufreq_nightmare.c
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

static void do_nightmare_timer(struct work_struct *work);
static int cpufreq_governor_nightmare(struct cpufreq_policy *policy,
				unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_NIGHTMARE
static
#endif
struct cpufreq_governor cpufreq_gov_nightmare = {
	.name                   = "nightmare",
	.governor               = cpufreq_governor_nightmare,
	.owner                  = THIS_MODULE,
};

struct cpufreq_nightmare_cpuinfo {
	u64 prev_cpu_busy;
	u64 prev_cpu_idle;
	struct delayed_work work;
	int cpu;
};
/*
 * mutex that serializes governor limit change with
 * do_nightmare_timer invocation. We do not want do_nightmare_timer to run
 * when user is changing the governor or limits.
 */
static struct mutex timer_mutex;
static struct workqueue_struct *dvfs_workqueue;

static DEFINE_PER_CPU(struct cpufreq_nightmare_cpuinfo, od_nightmare_cpuinfo);
static DEFINE_PER_CPU(struct cpufreq_policy *, cpufreq_cpu_data);

static unsigned int nightmare_enable;	/* number of CPUs using this policy */
/*
 * nightmare_mutex protects nightmare_enable in governor start/stop.
 */
static DEFINE_MUTEX(nightmare_mutex);

/* nightmare tuners */
static struct nightmare_tuners {
	atomic_t sampling_rate;
	atomic_t hotplug_enable;
	atomic_t cpu_up_rate;
	atomic_t cpu_down_rate;
	atomic_t up_load;
	atomic_t down_load;
	atomic_t inc_cpu_load_at_min_freq;
	atomic_t inc_cpu_load;
	atomic_t dec_cpu_load;
	atomic_t freq_for_responsiveness;
	atomic_t freq_for_responsiveness_max;
	atomic_t freq_up_brake_at_min_freq;
	atomic_t freq_up_brake;
	atomic_t freq_step_at_min_freq;
	atomic_t freq_step;
	atomic_t freq_step_dec;
	atomic_t freq_step_dec_at_max_freq;
	atomic_t up_sf_step;
	atomic_t down_sf_step;
	atomic_t onecoresuspend;
	atomic_t min_freq_limit;
	atomic_t max_freq_limit;
} nightmare_tuners_ins = {
	.sampling_rate = ATOMIC_INIT(60000),
	.hotplug_enable = ATOMIC_INIT(0),
	.cpu_up_rate = ATOMIC_INIT(10),
	.cpu_down_rate = ATOMIC_INIT(5),
	.up_load = ATOMIC_INIT(65),
	.down_load = ATOMIC_INIT(30),
	.inc_cpu_load_at_min_freq = ATOMIC_INIT(60),
	.inc_cpu_load = ATOMIC_INIT(70),
	.dec_cpu_load = ATOMIC_INIT(50),
	.freq_for_responsiveness = ATOMIC_INIT(400000),
	.freq_for_responsiveness_max = ATOMIC_INIT(1200000),
	.freq_step_at_min_freq = ATOMIC_INIT(20),
	.freq_step = ATOMIC_INIT(20),
	.freq_up_brake_at_min_freq = ATOMIC_INIT(30),
	.freq_up_brake = ATOMIC_INIT(30),
	.freq_step_dec = ATOMIC_INIT(10),
	.freq_step_dec_at_max_freq = ATOMIC_INIT(10),
	.up_sf_step = ATOMIC_INIT(0),
	.down_sf_step = ATOMIC_INIT(0),
	.onecoresuspend = ATOMIC_INIT(0),
};

static int num_rate;

/************************** sysfs interface ************************/

/* cpufreq_nightmare Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%d\n", atomic_read(&nightmare_tuners_ins.object));		\
}
show_one(sampling_rate, sampling_rate);
show_one(hotplug_enable, hotplug_enable);
show_one(cpu_up_rate, cpu_up_rate);
show_one(cpu_down_rate, cpu_down_rate);
show_one(up_load, up_load);
show_one(down_load, down_load);
show_one(inc_cpu_load_at_min_freq, inc_cpu_load_at_min_freq);
show_one(inc_cpu_load, inc_cpu_load);
show_one(dec_cpu_load, dec_cpu_load);
show_one(freq_for_responsiveness, freq_for_responsiveness);
show_one(freq_for_responsiveness_max, freq_for_responsiveness_max);
show_one(freq_step_at_min_freq, freq_step_at_min_freq);
show_one(freq_step, freq_step);
show_one(freq_up_brake_at_min_freq, freq_up_brake_at_min_freq);
show_one(freq_up_brake, freq_up_brake);
show_one(freq_step_dec, freq_step_dec);
show_one(freq_step_dec_at_max_freq, freq_step_dec_at_max_freq);
show_one(up_sf_step, up_sf_step);
show_one(down_sf_step, down_sf_step);
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
	
	if (input == atomic_read(&nightmare_tuners_ins.sampling_rate))
		return count;

	atomic_set(&nightmare_tuners_ins.sampling_rate,input);

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

	if (atomic_read(&nightmare_tuners_ins.hotplug_enable) == input)
		return count;

	atomic_set(&nightmare_tuners_ins.hotplug_enable, input);

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

	if (input == atomic_read(&nightmare_tuners_ins.cpu_up_rate))
		return count;

	atomic_set(&nightmare_tuners_ins.cpu_up_rate,input);

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

	if (input == atomic_read(&nightmare_tuners_ins.cpu_down_rate))
		return count;

	atomic_set(&nightmare_tuners_ins.cpu_down_rate,input);
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

	if (input == atomic_read(&nightmare_tuners_ins.up_load))
		return count;

	atomic_set(&nightmare_tuners_ins.up_load,input);

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

	if (input == atomic_read(&nightmare_tuners_ins.down_load))
		return count;

	atomic_set(&nightmare_tuners_ins.down_load,input);

	return count;
}

/* inc_cpu_load_at_min_freq */
static ssize_t store_inc_cpu_load_at_min_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1) {
		return -EINVAL;
	}

	input = min(input,atomic_read(&nightmare_tuners_ins.inc_cpu_load));

	if (input == atomic_read(&nightmare_tuners_ins.inc_cpu_load_at_min_freq))
		return count;

	atomic_set(&nightmare_tuners_ins.inc_cpu_load_at_min_freq,input);

	return count;
}

/* inc_cpu_load */
static ssize_t store_inc_cpu_load(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,100),0);

	if (input == atomic_read(&nightmare_tuners_ins.inc_cpu_load))
		return count;

	atomic_set(&nightmare_tuners_ins.inc_cpu_load,input);

	return count;
}

/* dec_cpu_load */
static ssize_t store_dec_cpu_load(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,95),5);

	if (input == atomic_read(&nightmare_tuners_ins.dec_cpu_load))
		return count;

	atomic_set(&nightmare_tuners_ins.dec_cpu_load,input);

	return count;
}

/* freq_for_responsiveness */
static ssize_t store_freq_for_responsiveness(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	if (input == atomic_read(&nightmare_tuners_ins.freq_for_responsiveness))
		return count;

	atomic_set(&nightmare_tuners_ins.freq_for_responsiveness,input);

	return count;
}

/* freq_for_responsiveness_max */
static ssize_t store_freq_for_responsiveness_max(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	if (input == atomic_read(&nightmare_tuners_ins.freq_for_responsiveness_max))
		return count;

	atomic_set(&nightmare_tuners_ins.freq_for_responsiveness_max,input);

	return count;
}

/* freq_step_at_min_freq */
static ssize_t store_freq_step_at_min_freq(struct kobject *a, struct attribute *b,
			       const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,100),0);

	if (input == atomic_read(&nightmare_tuners_ins.freq_step_at_min_freq))
		return count;

	atomic_set(&nightmare_tuners_ins.freq_step_at_min_freq,input);

	return count;
}

/* freq_step */
static ssize_t store_freq_step(struct kobject *a, struct attribute *b,
			       const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,100),0);

	if (input == atomic_read(&nightmare_tuners_ins.freq_step))
		return count;

	atomic_set(&nightmare_tuners_ins.freq_step,input);

	return count;
}

/* freq_up_brake_at_min_freq */
static ssize_t store_freq_up_brake_at_min_freq(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,100),0);

	if (input == atomic_read(&nightmare_tuners_ins.freq_up_brake_at_min_freq)) {/* nothing to do */
		return count;
	}

	atomic_set(&nightmare_tuners_ins.freq_up_brake_at_min_freq,input);

	return count;
}

/* freq_up_brake */
static ssize_t store_freq_up_brake(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,100),0);

	if (input == atomic_read(&nightmare_tuners_ins.freq_up_brake)) {/* nothing to do */
		return count;
	}

	atomic_set(&nightmare_tuners_ins.freq_up_brake,input);

	return count;
}

/* freq_step_dec */
static ssize_t store_freq_step_dec(struct kobject *a, struct attribute *b,
				       const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,100),0);

	if (input == atomic_read(&nightmare_tuners_ins.freq_step_dec)) {/* nothing to do */
		return count;
	}

	atomic_set(&nightmare_tuners_ins.freq_step_dec,input);

	return count;
}

/* freq_step_dec_at_max_freq */
static ssize_t store_freq_step_dec_at_max_freq(struct kobject *a, struct attribute *b,
				       const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(min(input,100),0);

	if (input == atomic_read(&nightmare_tuners_ins.freq_step_dec_at_max_freq)) {/* nothing to do */
		return count;
	}

	atomic_set(&nightmare_tuners_ins.freq_step_dec_at_max_freq,input);

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

	if (input == atomic_read(&nightmare_tuners_ins.up_sf_step))
		return count;

	 atomic_set(&nightmare_tuners_ins.up_sf_step,input);

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

	if (input == atomic_read(&nightmare_tuners_ins.down_sf_step))
		return count;

	atomic_set(&nightmare_tuners_ins.down_sf_step,input);

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

	if (atomic_read(&nightmare_tuners_ins.onecoresuspend) == input)
		return count;

	atomic_set(&nightmare_tuners_ins.onecoresuspend, input);

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
	
	input = max(min(input,atomic_read(&nightmare_tuners_ins.max_freq_limit)),0);

	if (input == atomic_read(&nightmare_tuners_ins.min_freq_limit))
		return count;

	atomic_set(&nightmare_tuners_ins.min_freq_limit,input);

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
	
	input = max(min(input,1600000),atomic_read(&nightmare_tuners_ins.min_freq_limit));

	if (input == atomic_read(&nightmare_tuners_ins.max_freq_limit))
		return count;

	atomic_set(&nightmare_tuners_ins.max_freq_limit,input);

	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(hotplug_enable);
define_one_global_rw(cpu_up_rate);
define_one_global_rw(cpu_down_rate);
define_one_global_rw(up_load);
define_one_global_rw(down_load);
define_one_global_rw(inc_cpu_load_at_min_freq);
define_one_global_rw(inc_cpu_load);
define_one_global_rw(dec_cpu_load);
define_one_global_rw(freq_for_responsiveness);
define_one_global_rw(freq_for_responsiveness_max);
define_one_global_rw(freq_step_at_min_freq);
define_one_global_rw(freq_step);
define_one_global_rw(freq_up_brake_at_min_freq);
define_one_global_rw(freq_up_brake);
define_one_global_rw(freq_step_dec);
define_one_global_rw(freq_step_dec_at_max_freq);
define_one_global_rw(up_sf_step);
define_one_global_rw(down_sf_step);
define_one_global_rw(onecoresuspend);
define_one_global_rw(min_freq_limit);
define_one_global_rw(max_freq_limit);

static struct attribute *nightmare_attributes[] = {
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
	&inc_cpu_load_at_min_freq.attr,
	&inc_cpu_load.attr,
	&dec_cpu_load.attr,
	&freq_for_responsiveness.attr,
	&freq_for_responsiveness_max.attr,
	&freq_step_at_min_freq.attr,
	&freq_step.attr,
	&freq_up_brake_at_min_freq.attr,
	&freq_up_brake.attr,
	&freq_step_dec.attr,
	&freq_step_dec_at_max_freq.attr,
	&up_sf_step.attr,
	&down_sf_step.attr,
	&onecoresuspend.attr,
	&min_freq_limit.attr,
	&max_freq_limit.attr,
	NULL
};

static struct attribute_group nightmare_attr_group = {
	.attrs = nightmare_attributes,
	.name = "nightmare",
};

/************************** sysfs end ************************/

static void nightmare_check_cpu(struct cpufreq_nightmare_cpuinfo *this_nightmare_cpuinfo)
{
	int up_rate = atomic_read(&nightmare_tuners_ins.cpu_up_rate);
	int down_rate = atomic_read(&nightmare_tuners_ins.cpu_down_rate);
	bool onecoresuspend = atomic_read(&nightmare_tuners_ins.onecoresuspend) > 0;
	bool hotplug_enable = atomic_read(&nightmare_tuners_ins.hotplug_enable) > 0;
	unsigned int freq_for_responsiveness = atomic_read(&nightmare_tuners_ins.freq_for_responsiveness);
	unsigned int freq_for_responsiveness_max = atomic_read(&nightmare_tuners_ins.freq_for_responsiveness_max);
	int up_sf_step = atomic_read(&nightmare_tuners_ins.up_sf_step);
	int down_sf_step = atomic_read(&nightmare_tuners_ins.down_sf_step);
	int dec_cpu_load = atomic_read(&nightmare_tuners_ins.dec_cpu_load);
	unsigned int min_freq = atomic_read(&nightmare_tuners_ins.min_freq_limit);
	unsigned int max_freq = atomic_read(&nightmare_tuners_ins.max_freq_limit);
	unsigned int next_freq[NR_CPUS];
	int cur_load[NR_CPUS];
	int num_core = num_online_cpus();
	unsigned int j;

	for_each_online_cpu(j) {
		struct cpufreq_nightmare_cpuinfo *j_nightmare_cpuinfo = &per_cpu(od_nightmare_cpuinfo, j);
		struct cpufreq_policy *cpu_policy = per_cpu(cpufreq_cpu_data, j);
		int inc_cpu_load = atomic_read(&nightmare_tuners_ins.inc_cpu_load);
		int freq_step = atomic_read(&nightmare_tuners_ins.freq_step);
		int freq_up_brake = atomic_read(&nightmare_tuners_ins.freq_up_brake);
		int freq_step_dec = atomic_read(&nightmare_tuners_ins.freq_step_dec);
		u64 cur_busy_time, cur_idle_time;
		unsigned int busy_time, idle_time;
		unsigned int tmp_freq;

		cur_busy_time = cputime_to_usecs(kcpustat_cpu(j).cpustat[CPUTIME_USER] + kcpustat_cpu(j).cpustat[CPUTIME_SYSTEM]
						+ kcpustat_cpu(j).cpustat[CPUTIME_IRQ] + kcpustat_cpu(j).cpustat[CPUTIME_SOFTIRQ]
						+ kcpustat_cpu(j).cpustat[CPUTIME_STEAL] + kcpustat_cpu(j).cpustat[CPUTIME_NICE]);

		cur_idle_time = cputime_to_usecs(kcpustat_cpu(j).cpustat[CPUTIME_IDLE] + kcpustat_cpu(j).cpustat[CPUTIME_IOWAIT]);

		busy_time = (unsigned int)
				(cur_busy_time - j_nightmare_cpuinfo->prev_cpu_busy);
		j_nightmare_cpuinfo->prev_cpu_busy = cur_busy_time;

		idle_time = (unsigned int)
				(cur_idle_time - j_nightmare_cpuinfo->prev_cpu_idle);
		j_nightmare_cpuinfo->prev_cpu_idle = cur_idle_time;

		/*printk(KERN_ERR "TIMER CPU[%u], wall[%u], idle[%u]\n",j, busy_time + idle_time, idle_time);*/
		if (!cpu_policy || busy_time + idle_time == 0) { /*if busy_time and idle_time are 0, evaluate cpu load next time*/
			hotplug_enable = false;
			continue;
		}
		cur_load[j] = busy_time ? (100 * busy_time) / (busy_time + idle_time) : 1;/*if busy_time is 0 cpu_load is equal to 1*/
		tmp_freq = cpu_policy->cur;
		/* Checking Frequency Limit */
		if (max_freq > cpu_policy->max || max_freq < cpu_policy->min)
			max_freq = cpu_policy->max;
		if (min_freq < cpu_policy->min || min_freq > cpu_policy->max)
			min_freq = cpu_policy->min;		
		/* CPUs Online Scale Frequency*/
		if (cpu_policy->cur < freq_for_responsiveness) {
			inc_cpu_load = atomic_read(&nightmare_tuners_ins.inc_cpu_load_at_min_freq);
			freq_step = atomic_read(&nightmare_tuners_ins.freq_step_at_min_freq);
			freq_up_brake = atomic_read(&nightmare_tuners_ins.freq_up_brake_at_min_freq);
		} else if (cpu_policy->cur > freq_for_responsiveness_max) {
			freq_step_dec = atomic_read(&nightmare_tuners_ins.freq_step_dec_at_max_freq);
		}		
		/* Check for frequency increase or for frequency decrease */
		if (cur_load[j] >= inc_cpu_load && cpu_policy->cur < max_freq) {
			tmp_freq = max(min((cpu_policy->cur + ((cur_load[j] + freq_step - freq_up_brake == 0 ? 1 : cur_load[j] + freq_step - freq_up_brake) * 2000)), max_freq), min_freq);
		} else if (cur_load[j] < dec_cpu_load && cpu_policy->cur > min_freq) {
			tmp_freq = max(min((cpu_policy->cur - ((100 - cur_load[j] + freq_step_dec == 0 ? 1 : 100 - cur_load[j] + freq_step_dec) * 2000)), max_freq), min_freq);
		}
		next_freq[j] = (tmp_freq / 100000) * 100000;
		if ((next_freq[j] > cpu_policy->cur
			&& (tmp_freq % 100000 > up_sf_step * 1000))
			|| (next_freq[j] < cpu_policy->cur 
			&& (tmp_freq % 100000 > down_sf_step * 1000))) {
				next_freq[j] += 100000;
		}
		/* printk(KERN_ERR "UP FREQ CALC.: CPU[%u], load[%d], target freq[%u], cur freq[%u], min freq[%u], max_freq[%u]\n",j, cur_load[j], next_freq[j], cpu_policy->cur, cpu_policy->min, max_freq); */
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
			if (cur_load[num_core - 1] >= atomic_read(&nightmare_tuners_ins.up_load)
				&& next_freq[num_core - 1] >= atomic_read(&hotplug_freq[num_core - 1][HOTPLUG_UP_INDEX])) {
				/* printk(KERN_ERR "[HOTPLUG IN] %s %u>=%u\n",
					__func__, cur_freq, up_freq); */
				if (!cpu_online(num_core)) {
					cpu_up(num_core);
					num_rate = 0;
				}
			}
#else
			if (cur_load[0] >= atomic_read(&nightmare_tuners_ins.up_load)
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
				|| cur_load[num_core - 1] < atomic_read(&nightmare_tuners_ins.down_load)
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
				|| cur_load[1] < atomic_read(&nightmare_tuners_ins.down_load)
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

static void do_nightmare_timer(struct work_struct *work)
{
	struct cpufreq_nightmare_cpuinfo *nightmare_cpuinfo =
		container_of(work, struct cpufreq_nightmare_cpuinfo, work.work);
	int delay;

	mutex_lock(&timer_mutex);
	nightmare_check_cpu(nightmare_cpuinfo);
	/* We want all CPUs to do sampling nearly on
	 * same jiffy
	 */
	delay = usecs_to_jiffies(atomic_read(&nightmare_tuners_ins.sampling_rate));
	if (num_online_cpus() > 1) {
		delay -= jiffies % delay;
	}	

	mod_delayed_work_on(nightmare_cpuinfo->cpu, dvfs_workqueue, &nightmare_cpuinfo->work, delay);
	mutex_unlock(&timer_mutex);
}

static int cpufreq_governor_nightmare(struct cpufreq_policy *policy,
				unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpufreq_nightmare_cpuinfo *this_nightmare_cpuinfo = &per_cpu(od_nightmare_cpuinfo, cpu);
	struct cpufreq_policy *cpu_policy = per_cpu(cpufreq_cpu_data, cpu);
	unsigned int j;
	int rc;

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&nightmare_mutex);
		num_rate = 0;

		nightmare_enable++;
		for_each_possible_cpu(j) {
			struct cpufreq_nightmare_cpuinfo *j_nightmare_cpuinfo = &per_cpu(od_nightmare_cpuinfo, j);
			per_cpu(cpufreq_cpu_data, j) = policy;
			j_nightmare_cpuinfo->prev_cpu_busy = cputime_to_usecs(kcpustat_cpu(j).cpustat[CPUTIME_USER] + kcpustat_cpu(j).cpustat[CPUTIME_SYSTEM]
						+ kcpustat_cpu(j).cpustat[CPUTIME_IRQ] + kcpustat_cpu(j).cpustat[CPUTIME_SOFTIRQ]
						+ kcpustat_cpu(j).cpustat[CPUTIME_STEAL] + kcpustat_cpu(j).cpustat[CPUTIME_NICE]);

			j_nightmare_cpuinfo->prev_cpu_idle = cputime_to_usecs(kcpustat_cpu(j).cpustat[CPUTIME_IDLE] + kcpustat_cpu(j).cpustat[CPUTIME_IOWAIT]);
		}
		this_nightmare_cpuinfo->cpu = cpu;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (nightmare_enable == 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&nightmare_attr_group);
			if (rc) {
				mutex_unlock(&nightmare_mutex);
				return rc;
			}
			atomic_set(&nightmare_tuners_ins.min_freq_limit,policy->min);
			atomic_set(&nightmare_tuners_ins.max_freq_limit,policy->max);
		}
		mutex_unlock(&nightmare_mutex);

		mutex_init(&timer_mutex);
		INIT_DEFERRABLE_WORK(&this_nightmare_cpuinfo->work, do_nightmare_timer);
		mod_delayed_work_on(this_nightmare_cpuinfo->cpu, dvfs_workqueue, &this_nightmare_cpuinfo->work, 0);

		break;

	case CPUFREQ_GOV_STOP:
		cancel_delayed_work(&this_nightmare_cpuinfo->work);
		mutex_destroy(&timer_mutex);

		mutex_lock(&nightmare_mutex);
		nightmare_enable--;
		for_each_possible_cpu(j) {
			per_cpu(cpufreq_cpu_data, j) = NULL;
		}

		if (!nightmare_enable) {
			sysfs_remove_group(cpufreq_global_kobject,
					   &nightmare_attr_group);
		}
		mutex_unlock(&nightmare_mutex);
		
		break;

	case CPUFREQ_GOV_LIMITS:
		if(!cpu_policy) {
			break;
		}
		mutex_lock(&timer_mutex);
		/* NOTHING TO DO JUST WATT */
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

static int __init cpufreq_gov_nightmare_init(void)
{
	int ret;

	dvfs_workqueue = create_singlethread_workqueue("knightmare");
	if (!dvfs_workqueue) {
		pr_err("%s cannot create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_free;
	}

	ret = cpufreq_register_governor(&cpufreq_gov_nightmare);
	if (ret)
		goto err_reg;

	return ret;

err_reg:
	destroy_workqueue(dvfs_workqueue);
err_free:
	kfree(&nightmare_tuners_ins);
	kfree(&hotplug_freq);
	return ret;
}

static void __exit cpufreq_gov_nightmare_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_nightmare);
	destroy_workqueue(dvfs_workqueue);
	kfree(&nightmare_tuners_ins);
	kfree(&hotplug_freq);
}

MODULE_AUTHOR("ByungChang Cha <bc.cha@samsung.com> | Alucard24@XDA");
MODULE_DESCRIPTION("'cpufreq_nightmare' - A dynamic cpufreq/cpuhotplug governor");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_NIGHTMARE
fs_initcall(cpufreq_gov_nightmare_init);
#else
module_init(cpufreq_gov_nightmare_init);
#endif
module_exit(cpufreq_gov_nightmare_exit);
