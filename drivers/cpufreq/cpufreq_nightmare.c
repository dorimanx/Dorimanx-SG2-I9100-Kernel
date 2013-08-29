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
#ifndef CONFIG_CPU_EXYNOS4210
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#endif
/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define MAX_HOTPLUG_RATE		(40)
#define HOTPLUG_DOWN_INDEX		(0)
#define HOTPLUG_UP_INDEX		(1)

#ifndef CONFIG_CPU_EXYNOS4210
static atomic_t hotplug_freq[4][2] = {
	{ATOMIC_INIT(0), ATOMIC_INIT(702000)},
	{ATOMIC_INIT(486000), ATOMIC_INIT(702000)},
	{ATOMIC_INIT(486000), ATOMIC_INIT(702000)},
	{ATOMIC_INIT(486000), ATOMIC_INIT(0)}
};
static atomic_t hotplug_load[4][2] = {
	{ATOMIC_INIT(0), ATOMIC_INIT(65)},
	{ATOMIC_INIT(30), ATOMIC_INIT(65)},
	{ATOMIC_INIT(30), ATOMIC_INIT(65)},
	{ATOMIC_INIT(30), ATOMIC_INIT(0)}
};
#else
static atomic_t hotplug_freq[2][2] = {
	{ATOMIC_INIT(0), ATOMIC_INIT(500000)},
	{ATOMIC_INIT(200000), ATOMIC_INIT(0)}
};
static atomic_t hotplug_load[4][2] = {
	{ATOMIC_INIT(0), ATOMIC_INIT(65)},
	{ATOMIC_INIT(30), ATOMIC_INIT(0)}
};
#endif

#ifndef CONFIG_CPU_EXYNOS4210
extern void apenable_intelli_plug(unsigned int state);
extern unsigned int apget_intelli_plug_active(void);
static unsigned int prev_apenable;
#endif

static void do_nightmare_timer(struct work_struct *work);
static int cpufreq_governor_nightmare(struct cpufreq_policy *policy,
				unsigned int event);

static struct work_struct hotplug_offline_work;
static struct work_struct hotplug_online_work;

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_NIGHTMARE
static
#endif
struct cpufreq_governor cpufreq_gov_nightmare = {
	.name                   = "nightmare",
	.governor               = cpufreq_governor_nightmare,
	.owner                  = THIS_MODULE,
};

struct cpufreq_nightmare_cpuinfo {
	unsigned long prev_cpu_user;
	unsigned long prev_cpu_system;
	unsigned long prev_cpu_others;
	unsigned long prev_cpu_idle;
	unsigned long prev_cpu_iowait;
	struct delayed_work work;
	int cpu;
};
/*
 * mutex that serializes governor limit change with
 * do_nightmare_timer invocation. We do not want do_nightmare_timer to run
 * when user is changing the governor or limits.
 */
static DEFINE_PER_CPU(struct cpufreq_nightmare_cpuinfo, od_nightmare_cpuinfo);

static unsigned int nightmare_enable;	/* number of CPUs using this policy */
static struct mutex timer_mutex;

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
#ifdef CONFIG_CPU_EXYNOS4210
	atomic_t up_sf_step;
	atomic_t down_sf_step;
#endif
	atomic_t maxcoreslimit;
#ifndef CONFIG_CPU_EXYNOS4210
	atomic_t maxcoreslimitsleep;
#endif
	atomic_t min_freq_limit;
	atomic_t max_freq_limit;
} nightmare_tuners_ins = {
	.sampling_rate = ATOMIC_INIT(60000),
	.hotplug_enable = ATOMIC_INIT(0),
	.cpu_up_rate = ATOMIC_INIT(10),
	.cpu_down_rate = ATOMIC_INIT(5),
	.inc_cpu_load_at_min_freq = ATOMIC_INIT(60),
	.inc_cpu_load = ATOMIC_INIT(70),
	.dec_cpu_load = ATOMIC_INIT(50),
#ifndef CONFIG_CPU_EXYNOS4210
	.freq_for_responsiveness = ATOMIC_INIT(540000),
	.freq_for_responsiveness_max = ATOMIC_INIT(1890000),
#else
	.freq_for_responsiveness = ATOMIC_INIT(300000),
	.freq_for_responsiveness_max = ATOMIC_INIT(1200000),
#endif
	.freq_step_at_min_freq = ATOMIC_INIT(20),
	.freq_step = ATOMIC_INIT(20),
	.freq_up_brake_at_min_freq = ATOMIC_INIT(30),
	.freq_up_brake = ATOMIC_INIT(30),
	.freq_step_dec = ATOMIC_INIT(10),
	.freq_step_dec_at_max_freq = ATOMIC_INIT(10),
#ifdef CONFIG_CPU_EXYNOS4210
	.up_sf_step = ATOMIC_INIT(0),
	.down_sf_step = ATOMIC_INIT(0),
#endif
	.maxcoreslimit = ATOMIC_INIT(NR_CPUS),
#ifndef CONFIG_CPU_EXYNOS4210
	.maxcoreslimitsleep = ATOMIC_INIT(2),
#endif
};
#ifndef CONFIG_CPU_EXYNOS4210
static bool earlysuspend;
#endif
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
#ifdef CONFIG_CPU_EXYNOS4210
show_one(up_sf_step, up_sf_step);
show_one(down_sf_step, down_sf_step);
#endif
show_one(maxcoreslimit, maxcoreslimit);
#ifndef CONFIG_CPU_EXYNOS4210
show_one(maxcoreslimitsleep, maxcoreslimitsleep);
#endif
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
show_hotplug_param(hotplug_load, 1, 1);
show_hotplug_param(hotplug_load, 2, 0);
#ifndef CONFIG_CPU_EXYNOS4210
show_hotplug_param(hotplug_freq, 2, 1);
show_hotplug_param(hotplug_freq, 3, 0);
show_hotplug_param(hotplug_freq, 3, 1);
show_hotplug_param(hotplug_freq, 4, 0);
show_hotplug_param(hotplug_load, 2, 1);
show_hotplug_param(hotplug_load, 3, 0);
show_hotplug_param(hotplug_load, 3, 1);
show_hotplug_param(hotplug_load, 4, 0);
#endif

store_hotplug_param(hotplug_freq, 1, 1);
store_hotplug_param(hotplug_freq, 2, 0);
store_hotplug_param(hotplug_load, 1, 1);
store_hotplug_param(hotplug_load, 2, 0);
#ifndef CONFIG_CPU_EXYNOS4210
store_hotplug_param(hotplug_freq, 2, 1);
store_hotplug_param(hotplug_freq, 3, 0);
store_hotplug_param(hotplug_freq, 3, 1);
store_hotplug_param(hotplug_freq, 4, 0);
store_hotplug_param(hotplug_load, 2, 1);
store_hotplug_param(hotplug_load, 3, 0);
store_hotplug_param(hotplug_load, 3, 1);
store_hotplug_param(hotplug_load, 4, 0);
#endif

define_one_global_rw(hotplug_freq_1_1);
define_one_global_rw(hotplug_freq_2_0);
define_one_global_rw(hotplug_load_1_1);
define_one_global_rw(hotplug_load_2_0);
#ifndef CONFIG_CPU_EXYNOS4210
define_one_global_rw(hotplug_freq_2_1);
define_one_global_rw(hotplug_freq_3_0);
define_one_global_rw(hotplug_freq_3_1);
define_one_global_rw(hotplug_freq_4_0);
define_one_global_rw(hotplug_load_2_1);
define_one_global_rw(hotplug_load_3_0);
define_one_global_rw(hotplug_load_3_1);
define_one_global_rw(hotplug_load_4_0);
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

#ifndef CONFIG_CPU_EXYNOS4210
	if (input == 0) {
		apenable_intelli_plug(prev_apenable);
	} else {
		prev_apenable = apget_intelli_plug_active();
		apenable_intelli_plug(0);
	}
#endif

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
#endif
/* maxcoreslimit */
static ssize_t store_maxcoreslimit(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(input > NR_CPUS ? NR_CPUS : input, 1);

	if (atomic_read(&nightmare_tuners_ins.maxcoreslimit) == input)
		return count;

	atomic_set(&nightmare_tuners_ins.maxcoreslimit, input);

	return count;
}
#ifndef CONFIG_CPU_EXYNOS4210
/* maxcoreslimitsleep */
static ssize_t store_maxcoreslimitsleep(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(input > NR_CPUS ? NR_CPUS : input, 1); 

	if (atomic_read(&nightmare_tuners_ins.maxcoreslimitsleep) == input)
		return count;

	atomic_set(&nightmare_tuners_ins.maxcoreslimitsleep, input);

	return count;
}
#endif
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
	
	input = max(min(input,1890000),atomic_read(&nightmare_tuners_ins.min_freq_limit));

	if (input == atomic_read(&nightmare_tuners_ins.max_freq_limit))
		return count;

	atomic_set(&nightmare_tuners_ins.max_freq_limit,input);

	return count;
}

define_one_global_rw(sampling_rate);
define_one_global_rw(hotplug_enable);
define_one_global_rw(cpu_up_rate);
define_one_global_rw(cpu_down_rate);
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
#ifdef CONFIG_CPU_EXYNOS4210
define_one_global_rw(up_sf_step);
define_one_global_rw(down_sf_step);
#endif
define_one_global_rw(maxcoreslimit);
#ifndef CONFIG_CPU_EXYNOS4210
define_one_global_rw(maxcoreslimitsleep);
#endif
define_one_global_rw(min_freq_limit);
define_one_global_rw(max_freq_limit);

static struct attribute *nightmare_attributes[] = {
	&sampling_rate.attr,
	&hotplug_enable.attr,
	&hotplug_freq_1_1.attr,
	&hotplug_freq_2_0.attr,
	&hotplug_load_1_1.attr,
	&hotplug_load_2_0.attr,
#ifndef CONFIG_CPU_EXYNOS4210
	&hotplug_freq_2_1.attr,
	&hotplug_freq_3_0.attr,
	&hotplug_freq_3_1.attr,
	&hotplug_freq_4_0.attr,
	&hotplug_load_2_1.attr,
	&hotplug_load_3_0.attr,
	&hotplug_load_3_1.attr,
	&hotplug_load_4_0.attr,
#endif
	&cpu_up_rate.attr,
	&cpu_down_rate.attr,
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
#ifdef CONFIG_CPU_EXYNOS4210
	&up_sf_step.attr,
	&down_sf_step.attr,
#endif
	&maxcoreslimit.attr,
#ifndef CONFIG_CPU_EXYNOS4210
	&maxcoreslimitsleep.attr,
#endif
	&min_freq_limit.attr,
	&max_freq_limit.attr,
	NULL
};

static struct attribute_group nightmare_attr_group = {
	.attrs = nightmare_attributes,
	.name = "nightmare",
};

/************************** sysfs end ************************/

static void __ref hp_offline_work_fn(struct work_struct *work)
{
	int cpu;
	int online = num_online_cpus();

	for_each_online_cpu(cpu) {
		if (cpu == (online - 1) && cpu) {
			cpu_down(cpu);			
			//pr_info("auto_hotplug: CPU%d down.\n", cpu);
			break;
		}
	}
}

static void __ref hp_online_work_fn(struct work_struct *work)
{
	int cpu;
	for_each_cpu_not(cpu, cpu_online_mask) {
		if (cpu) {
			cpu_up(cpu);			
			//pr_info("auto_hotplug: CPU%d up.\n", cpu);
			break;
		}
	}
}

static void nightmare_check_cpu(struct cpufreq_nightmare_cpuinfo *this_nightmare_cpuinfo)
{
	int up_rate = atomic_read(&nightmare_tuners_ins.cpu_up_rate);
	int down_rate = atomic_read(&nightmare_tuners_ins.cpu_down_rate);
#ifdef CONFIG_CPU_EXYNOS4210
	int lmaxcoreslimit = atomic_read(&nightmare_tuners_ins.maxcoreslimit);
#else
	int lmaxcoreslimit = !earlysuspend ? atomic_read(&nightmare_tuners_ins.maxcoreslimit) : atomic_read(&nightmare_tuners_ins.maxcoreslimitsleep);
#endif
	bool hotplug_enable = atomic_read(&nightmare_tuners_ins.hotplug_enable) > 0;
	unsigned int freq_for_responsiveness = atomic_read(&nightmare_tuners_ins.freq_for_responsiveness);
	unsigned int freq_for_responsiveness_max = atomic_read(&nightmare_tuners_ins.freq_for_responsiveness_max);
#ifdef CONFIG_CPU_EXYNOS4210
	int up_sf_step = atomic_read(&nightmare_tuners_ins.up_sf_step);
	int down_sf_step = atomic_read(&nightmare_tuners_ins.down_sf_step);
#endif
	int dec_cpu_load = atomic_read(&nightmare_tuners_ins.dec_cpu_load);
	unsigned int min_freq = atomic_read(&nightmare_tuners_ins.min_freq_limit);
	unsigned int max_freq = atomic_read(&nightmare_tuners_ins.max_freq_limit);
	unsigned int next_freq[NR_CPUS];
	int cur_load[NR_CPUS];
	int num_core;
	unsigned int j;

	for_each_cpu(j, cpu_online_mask) {
		struct cpufreq_nightmare_cpuinfo *j_nightmare_cpuinfo = &per_cpu(od_nightmare_cpuinfo, j);
		struct cpufreq_policy cpu_policy;
		int inc_cpu_load = atomic_read(&nightmare_tuners_ins.inc_cpu_load);
		int freq_step = atomic_read(&nightmare_tuners_ins.freq_step);
		int freq_up_brake = atomic_read(&nightmare_tuners_ins.freq_up_brake);
		int freq_step_dec = atomic_read(&nightmare_tuners_ins.freq_step_dec);
		unsigned long cur_user_time, cur_system_time, cur_others_time, cur_idle_time, cur_iowait_time;
		unsigned int busy_time, idle_time;
		unsigned int tmp_freq;
		unsigned long flags;

		local_irq_save(flags);
		cur_user_time = (__force unsigned long)(kcpustat_cpu(j).cpustat[CPUTIME_USER]);
		cur_system_time = (__force unsigned long)(kcpustat_cpu(j).cpustat[CPUTIME_SYSTEM]);
		cur_others_time = (__force unsigned long)(kcpustat_cpu(j).cpustat[CPUTIME_IRQ] + kcpustat_cpu(j).cpustat[CPUTIME_SOFTIRQ]
																		+ kcpustat_cpu(j).cpustat[CPUTIME_STEAL] + kcpustat_cpu(j).cpustat[CPUTIME_NICE]);

		cur_idle_time = (__force unsigned long)(kcpustat_cpu(j).cpustat[CPUTIME_IDLE]);
		cur_iowait_time = (__force unsigned long)(kcpustat_cpu(j).cpustat[CPUTIME_IOWAIT]);
		local_irq_restore(flags);

		busy_time = (unsigned int)
				((cur_user_time - j_nightmare_cpuinfo->prev_cpu_user) +
				 (cur_system_time - j_nightmare_cpuinfo->prev_cpu_system) +
				 (cur_others_time - j_nightmare_cpuinfo->prev_cpu_others));
		j_nightmare_cpuinfo->prev_cpu_user = cur_user_time;
		j_nightmare_cpuinfo->prev_cpu_system = cur_system_time;
		j_nightmare_cpuinfo->prev_cpu_others = cur_others_time;

		idle_time = (unsigned int)
				((cur_idle_time - j_nightmare_cpuinfo->prev_cpu_idle) + 
				 (cur_iowait_time - j_nightmare_cpuinfo->prev_cpu_iowait));
		j_nightmare_cpuinfo->prev_cpu_idle = cur_idle_time;
		j_nightmare_cpuinfo->prev_cpu_iowait = cur_iowait_time;

		cpufreq_get_policy(&cpu_policy, j);
		/*printk(KERN_ERR "TIMER CPU[%u], wall[%u], idle[%u]\n",j, busy_time + idle_time, idle_time);*/
		if (!cpu_policy.cur || busy_time + idle_time == 0) { /*if busy_time and idle_time are 0, evaluate cpu load next time*/
			hotplug_enable = false;
			continue;
		}
		cur_load[j] = busy_time ? (100 * busy_time) / (busy_time + idle_time) : 1;/*if busy_time is 0 cpu_load is equal to 1*/
		tmp_freq = cpu_policy.cur;
		/* Checking Frequency Limit */
		if (max_freq > cpu_policy.max || max_freq < cpu_policy.min)
			max_freq = cpu_policy.max;
		if (min_freq < cpu_policy.min || min_freq > cpu_policy.max)
			min_freq = cpu_policy.min;		
		/* CPUs Online Scale Frequency*/
		if (cpu_policy.cur < freq_for_responsiveness) {
			inc_cpu_load = atomic_read(&nightmare_tuners_ins.inc_cpu_load_at_min_freq);
			freq_step = atomic_read(&nightmare_tuners_ins.freq_step_at_min_freq);
			freq_up_brake = atomic_read(&nightmare_tuners_ins.freq_up_brake_at_min_freq);
		} else if (cpu_policy.cur > freq_for_responsiveness_max) {
			freq_step_dec = atomic_read(&nightmare_tuners_ins.freq_step_dec_at_max_freq);
		}		
		/* Check for frequency increase or for frequency decrease */
#ifndef CONFIG_CPU_EXYNOS4210
		if (cur_load[j] >= inc_cpu_load && cpu_policy.cur < max_freq) {
			tmp_freq = max(min((cpu_policy.cur + ((cur_load[j] + freq_step - freq_up_brake == 0 ? 1 : cur_load[j] + freq_step - freq_up_brake) * 3840)), max_freq), min_freq);
		} else if (cur_load[j] < dec_cpu_load && cpu_policy.cur > min_freq) {
			tmp_freq = max(min((cpu_policy.cur - ((100 - cur_load[j] + freq_step_dec == 0 ? 1 : 100 - cur_load[j] + freq_step_dec) * 3840)), max_freq), min_freq);
		}
		next_freq[j] = tmp_freq;
#else
		if (cur_load[j] >= inc_cpu_load && cpu_policy.cur < max_freq) {
			tmp_freq = max(min((cpu_policy.cur + ((cur_load[j] + freq_step - freq_up_brake == 0 ? 1 : cur_load[j] + freq_step - freq_up_brake) * 2000)), max_freq), min_freq);
		} else if (cur_load[j] < dec_cpu_load && cpu_policy.cur > min_freq) {
			tmp_freq = max(min((cpu_policy.cur - ((100 - cur_load[j] + freq_step_dec == 0 ? 1 : 100 - cur_load[j] + freq_step_dec) * 2000)), max_freq), min_freq);
		}
		next_freq[j] = (tmp_freq / 100000) * 100000;
		if ((next_freq[j] > cpu_policy.cur
			&& (tmp_freq % 100000 > up_sf_step * 1000))
			|| (next_freq[j] < cpu_policy.cur 
			&& (tmp_freq % 100000 > down_sf_step * 1000))) {
				next_freq[j] += 100000;
		}
#endif
		/* printk(KERN_ERR "UP FREQ CALC.: CPU[%u], load[%d], target freq[%u], cur freq[%u], min freq[%u], max_freq[%u]\n",j, cur_load[j], next_freq[j], cpu_policy.cur, cpu_policy.min, max_freq); */
		if (next_freq[j] != cpu_policy.cur) {
			__cpufreq_driver_target(&cpu_policy, next_freq[j], CPUFREQ_RELATION_L);
		}
	}

	/* set num_rate used */
	++num_rate;

	if (hotplug_enable) {
		num_core = num_online_cpus();
		/*Check for CPU hotplug*/
		if (num_rate % up_rate == 0 && num_core < lmaxcoreslimit) {
#ifndef CONFIG_CPU_EXYNOS4210
			if (cur_load[num_core - 1] >= atomic_read(&hotplug_load[num_core - 1][HOTPLUG_UP_INDEX])
				&& next_freq[num_core - 1] >= atomic_read(&hotplug_freq[num_core - 1][HOTPLUG_UP_INDEX])) {
				/* printk(KERN_ERR "[HOTPLUGGING IN] %s %u>=%u\n",
					__func__, cur_freq, up_freq); */
				schedule_work_on(this_nightmare_cpuinfo->cpu, &hotplug_online_work);
				num_rate = 0;
			}
#else
			if (cur_load[0] >= atomic_read(&hotplug_load[0][HOTPLUG_UP_INDEX])
				&& next_freq[0] >= atomic_read(&hotplug_freq[0][HOTPLUG_UP_INDEX])) {
				/* printk(KERN_ERR "[HOTPLUGGING IN] %s %u>=%u\n",
					__func__, cur_freq, up_freq); */
				schedule_work_on(this_nightmare_cpuinfo->cpu, &hotplug_online_work);
				num_rate = 0;
			}
#endif
		} else if (num_rate % down_rate == 0 && num_core > (lmaxcoreslimit == NR_CPUS ? 1 : lmaxcoreslimit)) {
#ifndef CONFIG_CPU_EXYNOS4210	
			if (cur_load[num_core - 1] < atomic_read(&hotplug_load[num_core - 1][HOTPLUG_DOWN_INDEX])
				|| next_freq[num_core - 1] <= atomic_read(&hotplug_freq[num_core - 1][HOTPLUG_DOWN_INDEX])) {
				/* printk(KERN_ERR "[HOTPLUGGING OUT] %s %u<=%u\n",
					__func__, cur_freq, down_freq); */
				schedule_work_on(this_nightmare_cpuinfo->cpu, &hotplug_offline_work);
				num_rate = 0;
			}
#else
			if (cur_load[1] < atomic_read(&hotplug_load[1][HOTPLUG_DOWN_INDEX])
				|| next_freq[1] <= atomic_read(&hotplug_freq[1][HOTPLUG_DOWN_INDEX])) {
				/* printk(KERN_ERR "[HOTPLUGGING OUT] %s %u<=%u\n",
					__func__, cur_freq, down_freq); */
				schedule_work_on(this_nightmare_cpuinfo->cpu, &hotplug_offline_work);
				num_rate = 0;
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

	schedule_delayed_work_on(nightmare_cpuinfo->cpu, &nightmare_cpuinfo->work, delay);
	mutex_unlock(&timer_mutex);
}

#ifndef CONFIG_CPU_EXYNOS4210
#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend early_suspend;
static inline void cpufreq_nightmare_early_suspend(struct early_suspend *h)
{
	earlysuspend = true;
}
static inline void cpufreq_nightmare_late_resume(struct early_suspend *h)
{
	earlysuspend = false;
}
#endif
#endif

static int cpufreq_governor_nightmare(struct cpufreq_policy *policy,
				unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpufreq_nightmare_cpuinfo *this_nightmare_cpuinfo = &per_cpu(od_nightmare_cpuinfo, cpu);
	struct cpufreq_policy cpu_policy;
	unsigned int j;
	int rc, delay;

	cpufreq_get_policy(&cpu_policy, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
#ifndef CONFIG_CPU_EXYNOS4210
		prev_apenable = apget_intelli_plug_active();
		apenable_intelli_plug(0);
#endif

		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		mutex_lock(&nightmare_mutex);
		num_rate = 0;
		nightmare_enable=1;
		for_each_cpu(j, cpu_possible_mask) {
			struct cpufreq_nightmare_cpuinfo *j_nightmare_cpuinfo = &per_cpu(od_nightmare_cpuinfo, j);
			unsigned long flags;
			local_irq_save(flags);
			j_nightmare_cpuinfo->prev_cpu_user = (__force unsigned long)(kcpustat_cpu(j).cpustat[CPUTIME_USER]);
			j_nightmare_cpuinfo->prev_cpu_system = (__force unsigned long)(kcpustat_cpu(j).cpustat[CPUTIME_SYSTEM]);
			j_nightmare_cpuinfo->prev_cpu_others = (__force unsigned long)(kcpustat_cpu(j).cpustat[CPUTIME_IRQ] + kcpustat_cpu(j).cpustat[CPUTIME_SOFTIRQ]
																		+ kcpustat_cpu(j).cpustat[CPUTIME_STEAL] + kcpustat_cpu(j).cpustat[CPUTIME_NICE]);

			j_nightmare_cpuinfo->prev_cpu_idle = (__force unsigned long)(kcpustat_cpu(j).cpustat[CPUTIME_IDLE]);
			j_nightmare_cpuinfo->prev_cpu_iowait = (__force unsigned long)(kcpustat_cpu(j).cpustat[CPUTIME_IOWAIT]);
			local_irq_restore(flags);
		}
		this_nightmare_cpuinfo->cpu = cpu;
		mutex_init(&timer_mutex);
		INIT_DELAYED_WORK_DEFERRABLE(&this_nightmare_cpuinfo->work, do_nightmare_timer);
		INIT_WORK(&hotplug_offline_work, hp_offline_work_fn);
		INIT_WORK(&hotplug_online_work, hp_online_work_fn);
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
			atomic_set(&nightmare_tuners_ins.hotplug_enable, 1);
#ifndef CONFIG_CPU_EXYNOS4210
			earlysuspend = false;
#endif
		}
		delay=usecs_to_jiffies(atomic_read(&nightmare_tuners_ins.sampling_rate));
		if (num_online_cpus() > 1) {
			delay -= jiffies % delay;
		}
		mutex_unlock(&nightmare_mutex);
		schedule_delayed_work_on(this_nightmare_cpuinfo->cpu, &this_nightmare_cpuinfo->work, delay);
#ifndef CONFIG_CPU_EXYNOS4210
#ifdef CONFIG_HAS_EARLYSUSPEND
		register_early_suspend(&early_suspend);
#endif
#endif
		break;

	case CPUFREQ_GOV_STOP:
#ifndef CONFIG_CPU_EXYNOS4210
		apenable_intelli_plug(prev_apenable);
#endif

#ifndef CONFIG_CPU_EXYNOS4210
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&early_suspend);
#endif
#endif
		cancel_delayed_work_sync(&this_nightmare_cpuinfo->work);
		cancel_work_sync(&hotplug_offline_work);
		cancel_work_sync(&hotplug_online_work);
		mutex_destroy(&timer_mutex);

		mutex_lock(&nightmare_mutex);
		nightmare_enable=0;

		if (!nightmare_enable) {
			sysfs_remove_group(cpufreq_global_kobject,
					   &nightmare_attr_group);
		}
		mutex_unlock(&nightmare_mutex);
		
		break;

	case CPUFREQ_GOV_LIMITS:
		if(!cpu_policy.cur) {
			break;
		}
		mutex_lock(&timer_mutex);				
		/* NOTHING TO DO JUST WATT */
		if (policy->max < cpu_policy.cur)
			__cpufreq_driver_target(&cpu_policy,
				policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > cpu_policy.cur)
			__cpufreq_driver_target(&cpu_policy,
				policy->min, CPUFREQ_RELATION_L);
		mutex_unlock(&timer_mutex);

		break;
	}
	return 0;
}

static int __init cpufreq_gov_nightmare_init(void)
{
	int ret;

	ret = cpufreq_register_governor(&cpufreq_gov_nightmare);
	if (ret)
		goto err_free;
#ifndef CONFIG_CPU_EXYNOS4210
#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.suspend = cpufreq_nightmare_early_suspend;
	early_suspend.resume = cpufreq_nightmare_late_resume;
	early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
#endif
#endif
	return ret;

err_free:
	kfree(&nightmare_tuners_ins);
	kfree(&hotplug_freq);
	kfree(&hotplug_load);
	return ret;
}

static void __exit cpufreq_gov_nightmare_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_nightmare);
	kfree(&nightmare_tuners_ins);
	kfree(&hotplug_freq);
	kfree(&hotplug_load);
}

MODULE_AUTHOR("Alucard24@XDA");
MODULE_DESCRIPTION("'cpufreq_nightmare' - A dynamic cpufreq/cpuhotplug governor v1.8 (SnapDragon)");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_NIGHTMARE
fs_initcall(cpufreq_gov_nightmare_init);
#else
module_init(cpufreq_gov_nightmare_init);
#endif
module_exit(cpufreq_gov_nightmare_exit);
