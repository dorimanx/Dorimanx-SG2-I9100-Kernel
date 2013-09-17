/*
 * Author: Alucard_24@XDA
 *
 * Copyright 2012 Alucard_24@XDA
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/jiffies.h>
#include <linux/kernel_stat.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/module.h>
#ifndef CONFIG_CPU_EXYNOS4210
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#endif

#ifndef CONFIG_CPU_EXYNOS4210
static DEFINE_MUTEX(alucard_hotplug_mutex);
#endif
static struct mutex timer_mutex;

static struct delayed_work alucard_hotplug_work;

struct hotplug_cpuinfo {
	unsigned long prev_cpu_user;
	unsigned long prev_cpu_system;
	unsigned long prev_cpu_others;
	unsigned long prev_cpu_idle;
	unsigned long prev_cpu_iowait;
};

static DEFINE_PER_CPU(struct hotplug_cpuinfo, od_hotplug_cpuinfo);

static struct hotplug_tuners {
	atomic_t hotplug_sampling_rate;
	atomic_t hotplug_enable;
	atomic_t cpu_up_rate;
	atomic_t cpu_down_rate;
	atomic_t maxcoreslimit;
#ifndef CONFIG_CPU_EXYNOS4210
	atomic_t maxcoreslimitsleep;
#endif
} hotplug_tuners_ins = {
	.hotplug_sampling_rate = ATOMIC_INIT(50000),
	.hotplug_enable = ATOMIC_INIT(0),
	.cpu_up_rate = ATOMIC_INIT(10),
	.cpu_down_rate = ATOMIC_INIT(10),
	.maxcoreslimit = ATOMIC_INIT(NR_CPUS),
#ifndef CONFIG_CPU_EXYNOS4210
	.maxcoreslimitsleep = ATOMIC_INIT(1),
#endif
};

#define MAX_HOTPLUG_RATE		(40)
#define DOWN_INDEX		(0)
#define UP_INDEX		(1)

static atomic_t hotplugging_rate = ATOMIC_INIT(0);
#ifndef CONFIG_CPU_EXYNOS4210
static bool earlysuspend = false;
#endif

#ifdef CONFIG_CPU_EXYNOS4210
static atomic_t hotplug_freq[2][2] = {
	{ATOMIC_INIT(0), ATOMIC_INIT(800000)},
	{ATOMIC_INIT(500000), ATOMIC_INIT(0)}
};
static atomic_t hotplug_load[2][2] = {
	{ATOMIC_INIT(0), ATOMIC_INIT(65)},
	{ATOMIC_INIT(30), ATOMIC_INIT(0)}
};
#else
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
#endif

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%d\n", atomic_read(&hotplug_tuners_ins.object));		\
}
show_one(hotplug_sampling_rate, hotplug_sampling_rate);
show_one(hotplug_enable, hotplug_enable);
show_one(cpu_up_rate, cpu_up_rate);
show_one(cpu_down_rate, cpu_down_rate);
show_one(maxcoreslimit, maxcoreslimit);
#ifndef CONFIG_CPU_EXYNOS4210
show_one(maxcoreslimitsleep, maxcoreslimitsleep);
#endif

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
	ret = sscanf(buf, "%d", &input);				\
	if (ret != 1)							\
		return -EINVAL;						\
	if (input == atomic_read(&file_name[num_core - 1][up_down])) {		\
		return count;	\
	}	\
	atomic_set(&file_name[num_core - 1][up_down], input);			\
	return count;							\
}

/* hotplug freq */
show_hotplug_param(hotplug_freq, 1, 1);
show_hotplug_param(hotplug_freq, 2, 0);
#if NR_CPUS >= 4
show_hotplug_param(hotplug_freq, 2, 1);
show_hotplug_param(hotplug_freq, 3, 0);
show_hotplug_param(hotplug_freq, 3, 1);
show_hotplug_param(hotplug_freq, 4, 0);
#endif
/* hotplug load */
show_hotplug_param(hotplug_load, 1, 1);
show_hotplug_param(hotplug_load, 2, 0);
#if NR_CPUS >= 4
show_hotplug_param(hotplug_load, 2, 1);
show_hotplug_param(hotplug_load, 3, 0);
show_hotplug_param(hotplug_load, 3, 1);
show_hotplug_param(hotplug_load, 4, 0);
#endif

/* hotplug freq */
store_hotplug_param(hotplug_freq, 1, 1);
store_hotplug_param(hotplug_freq, 2, 0);
#if NR_CPUS >= 4
store_hotplug_param(hotplug_freq, 2, 1);
store_hotplug_param(hotplug_freq, 3, 0);
store_hotplug_param(hotplug_freq, 3, 1);
store_hotplug_param(hotplug_freq, 4, 0);
#endif
/* hotplug load */
store_hotplug_param(hotplug_load, 1, 1);
store_hotplug_param(hotplug_load, 2, 0);
#if NR_CPUS >= 4
store_hotplug_param(hotplug_load, 2, 1);
store_hotplug_param(hotplug_load, 3, 0);
store_hotplug_param(hotplug_load, 3, 1);
store_hotplug_param(hotplug_load, 4, 0);
#endif
define_one_global_rw(hotplug_freq_1_1);
define_one_global_rw(hotplug_freq_2_0);
#if NR_CPUS >= 4
define_one_global_rw(hotplug_freq_2_1);
define_one_global_rw(hotplug_freq_3_0);
define_one_global_rw(hotplug_freq_3_1);
define_one_global_rw(hotplug_freq_4_0);
#endif
define_one_global_rw(hotplug_load_1_1);
define_one_global_rw(hotplug_load_2_0);
#if NR_CPUS >= 4
define_one_global_rw(hotplug_load_2_1);
define_one_global_rw(hotplug_load_3_0);
define_one_global_rw(hotplug_load_3_1);
define_one_global_rw(hotplug_load_4_0);
#endif

static void __ref cpus_hotplugging(bool state) {
	unsigned int cpu=0;
	int delay = usecs_to_jiffies(atomic_read(&hotplug_tuners_ins.hotplug_sampling_rate));

	mutex_lock(&timer_mutex);

	if (state) {
		for_each_possible_cpu(cpu) {
			struct hotplug_cpuinfo *cpu_hotplug_cpuinfo = &per_cpu(od_hotplug_cpuinfo, cpu);
			cpu_hotplug_cpuinfo->prev_cpu_user = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_USER]);
			cpu_hotplug_cpuinfo->prev_cpu_system = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM]);
			cpu_hotplug_cpuinfo->prev_cpu_others = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ] + kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ]
																			+ kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL] + kcpustat_cpu(cpu).cpustat[CPUTIME_NICE]);

			cpu_hotplug_cpuinfo->prev_cpu_idle = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE]);
			cpu_hotplug_cpuinfo->prev_cpu_iowait = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT]);
		}
		atomic_set(&hotplugging_rate, 0);
		schedule_delayed_work_on(0, &alucard_hotplug_work, delay);
	} else {
		for_each_online_cpu(cpu) {
			if (cpu == 0)
				continue;
			cpu_down(cpu);
		}
	}

	mutex_unlock(&timer_mutex);
}

/* hotplug_sampling_rate */
static ssize_t store_hotplug_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	int input;
	int ret;

	ret = sscanf(buf, "%d", &input);
	if (ret != 1)
		return -EINVAL;

	input = max(input,10000);
	
	if (input == atomic_read(&hotplug_tuners_ins.hotplug_sampling_rate))
		return count;

	atomic_set(&hotplug_tuners_ins.hotplug_sampling_rate,input);

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

	if (atomic_read(&hotplug_tuners_ins.hotplug_enable) == input)
		return count;

	if (input > 0)
		cpus_hotplugging(true);
	else
		cpus_hotplugging(false);

	atomic_set(&hotplug_tuners_ins.hotplug_enable, input);

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

	if (input == atomic_read(&hotplug_tuners_ins.cpu_up_rate))
		return count;

	atomic_set(&hotplug_tuners_ins.cpu_up_rate,input);

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

	if (input == atomic_read(&hotplug_tuners_ins.cpu_down_rate))
		return count;

	atomic_set(&hotplug_tuners_ins.cpu_down_rate,input);
	return count;
}

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

	if (atomic_read(&hotplug_tuners_ins.maxcoreslimit) == input)
		return count;

	atomic_set(&hotplug_tuners_ins.maxcoreslimit, input);

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

	if (atomic_read(&hotplug_tuners_ins.maxcoreslimitsleep) == input)
		return count;

	atomic_set(&hotplug_tuners_ins.maxcoreslimitsleep, input);

	return count;
}
#endif
define_one_global_rw(hotplug_sampling_rate);
define_one_global_rw(hotplug_enable);
define_one_global_rw(cpu_up_rate);
define_one_global_rw(cpu_down_rate);
define_one_global_rw(maxcoreslimit);
#ifndef CONFIG_CPU_EXYNOS4210
define_one_global_rw(maxcoreslimitsleep);
#endif

static struct attribute *alucard_hotplug_attributes[] = {
	&hotplug_sampling_rate.attr,
	&hotplug_enable.attr,
	&hotplug_freq_1_1.attr,
	&hotplug_freq_2_0.attr,
#if NR_CPUS >= 4
	&hotplug_freq_2_1.attr,
	&hotplug_freq_3_0.attr,
	&hotplug_freq_3_1.attr,
	&hotplug_freq_4_0.attr,
#endif
	&hotplug_load_1_1.attr,
	&hotplug_load_2_0.attr,
#if NR_CPUS >= 4
	&hotplug_load_2_1.attr,
	&hotplug_load_3_0.attr,
	&hotplug_load_3_1.attr,
	&hotplug_load_4_0.attr,
#endif
	&cpu_up_rate.attr,
	&cpu_down_rate.attr,
	&maxcoreslimit.attr,
#ifndef CONFIG_CPU_EXYNOS4210
	&maxcoreslimitsleep.attr,
#endif
	NULL
};

static struct attribute_group alucard_hotplug_attr_group = {
	.attrs = alucard_hotplug_attributes,
	.name = "alucard_hotplug",
};

static void __cpuinit hotplug_work_fn(struct work_struct *work)
{
	int delay;
	int up_rate = atomic_read(&hotplug_tuners_ins.cpu_up_rate);
	int down_rate = atomic_read(&hotplug_tuners_ins.cpu_down_rate);
#ifdef CONFIG_CPU_EXYNOS4210
	int maxcoreslimit = atomic_read(&hotplug_tuners_ins.maxcoreslimit);
#else
	int maxcoreslimit = !earlysuspend ? atomic_read(&hotplug_tuners_ins.maxcoreslimit) : atomic_read(&hotplug_tuners_ins.maxcoreslimitsleep);
#endif
	bool hotplug_enable = atomic_read(&hotplug_tuners_ins.hotplug_enable) > 0;
#ifdef CONFIG_CPU_EXYNOS4210
	unsigned int cur_freq[NR_CPUS] = {0, 0};
	int cur_load[NR_CPUS] = {-1, -1};
#else
	unsigned int cur_freq[NR_CPUS] = {0, 0, 0, 0};
	int cur_load[NR_CPUS] = {-1, -1, -1, -1};
#endif
	unsigned int cpu = 0;
	bool hotplugged = false;

	mutex_lock(&timer_mutex);

	if (hotplug_enable) { 
		for_each_online_cpu(cpu) {
			struct hotplug_cpuinfo *cpu_hotplug_cpuinfo = &per_cpu(od_hotplug_cpuinfo, cpu);
			struct cpufreq_policy cpu_policy;
			unsigned long cur_user_time, cur_system_time, cur_others_time, cur_idle_time, cur_iowait_time;
			unsigned int busy_time, idle_time;
			unsigned int ret = -EINVAL;

			cur_user_time = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_USER]);
			cur_system_time = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM]);
			cur_others_time = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ] + kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ]
																			+ kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL] + kcpustat_cpu(cpu).cpustat[CPUTIME_NICE]);

			cur_idle_time = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE]);
			cur_iowait_time = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT]);

			busy_time = (unsigned int)
					((cur_user_time - cpu_hotplug_cpuinfo->prev_cpu_user) +
					 (cur_system_time - cpu_hotplug_cpuinfo->prev_cpu_system) +
					 (cur_others_time - cpu_hotplug_cpuinfo->prev_cpu_others));
			cpu_hotplug_cpuinfo->prev_cpu_user = cur_user_time;
			cpu_hotplug_cpuinfo->prev_cpu_system = cur_system_time;
			cpu_hotplug_cpuinfo->prev_cpu_others = cur_others_time;

			idle_time = (unsigned int)
					((cur_idle_time - cpu_hotplug_cpuinfo->prev_cpu_idle) + 
					 (cur_iowait_time - cpu_hotplug_cpuinfo->prev_cpu_iowait));
			cpu_hotplug_cpuinfo->prev_cpu_idle = cur_idle_time;
			cpu_hotplug_cpuinfo->prev_cpu_iowait = cur_iowait_time;

			ret = cpufreq_get_policy(&cpu_policy, cpu);
			/*printk(KERN_ERR "TIMER CPU[%u], wall[%u], idle[%u]\n",j, busy_time + idle_time, idle_time);*/
			if (ret || busy_time + idle_time == 0) { /*if busy_time and idle_time are 0, evaluate cpu load next time*/
				continue;
			}

			cur_load[cpu] = busy_time ? (100 * busy_time) / (busy_time + idle_time) : 1;/*if busy_time is 0 cpu_load is equal to 1*/
			cur_freq[cpu] = cpu_policy.cur;
		}

		/* set hotplugging_rate used */
		atomic_inc(&hotplugging_rate);

		/*Check for CPU hotplug*/
		if (atomic_read(&hotplugging_rate) % up_rate == 0) {
			for_each_cpu_not(cpu, cpu_online_mask) {
				if (cpu == 0 || cpu > maxcoreslimit - 1 || hotplugged == true) {
					continue;
				}
				if (cur_load[cpu - 1] >= atomic_read(&hotplug_load[cpu - 1][UP_INDEX])
					&& cur_freq[cpu - 1] >= atomic_read(&hotplug_freq[cpu - 1][UP_INDEX])) {
						cpu_up(cpu);
						atomic_set(&hotplugging_rate, 0);
						hotplugged = true;
						break;
				}
			}
			if (hotplugged == true)
				goto schedule_cpu;
		}

		if (atomic_read(&hotplugging_rate) % down_rate == 0) {
			maxcoreslimit = (maxcoreslimit == NR_CPUS ? 0 : maxcoreslimit - 1);
			for_each_online_cpu(cpu) {
				if (cur_load[cpu] < 0 || cpu <= maxcoreslimit || hotplugged == true) {
					continue;
				}
				if (cur_load[cpu] < atomic_read(&hotplug_load[cpu][DOWN_INDEX])
					|| cur_freq[cpu] <= atomic_read(&hotplug_freq[cpu][DOWN_INDEX])) {
						cpu_down(cpu);
						atomic_set(&hotplugging_rate, 0);
						hotplugged = true;
						break;
				}
			}
			if (hotplugged == true)
				goto schedule_cpu;
		}

		if (atomic_read(&hotplugging_rate) >= max(up_rate, down_rate)) {
			atomic_set(&hotplugging_rate, 0);
		}
	}

schedule_cpu:
	delay = usecs_to_jiffies(atomic_read(&hotplug_tuners_ins.hotplug_sampling_rate));
	if (num_online_cpus() > 1)
		delay -= jiffies % delay;
	schedule_delayed_work_on(0, &alucard_hotplug_work, delay);
	mutex_unlock(&timer_mutex);
}

#ifndef CONFIG_CPU_EXYNOS4210
#ifdef CONFIG_HAS_EARLYSUSPEND
static void alucard_hotplug_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&alucard_hotplug_mutex);
	earlysuspend = true;
	mutex_unlock(&alucard_hotplug_mutex);
}

static void __cpuinit alucard_hotplug_late_resume(struct early_suspend *handler)
{
	mutex_lock(&alucard_hotplug_mutex);
	earlysuspend = false;
	mutex_unlock(&alucard_hotplug_mutex);
}

static struct early_suspend alucard_hotplug_early_suspend_struct_driver = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 10,
	.suspend = alucard_hotplug_early_suspend,
	.resume = alucard_hotplug_late_resume,
};
#endif	/* CONFIG_HAS_EARLYSUSPEND */
#endif

int __init alucard_hotplug_init(void)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay;
	unsigned int cpu;
	int ret;

	ret = sysfs_create_group(cpufreq_global_kobject, &alucard_hotplug_attr_group);
	if (ret) {
		printk(KERN_ERR "failed at(%d)\n", __LINE__);
		return ret;
	}

	mutex_lock(&alucard_hotplug_mutex);
	atomic_set(&hotplugging_rate, 0);
	for_each_possible_cpu(cpu) {
		struct hotplug_cpuinfo *cpu_hotplug_cpuinfo = &per_cpu(od_hotplug_cpuinfo, cpu);
		cpu_hotplug_cpuinfo->prev_cpu_user = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_USER]);
		cpu_hotplug_cpuinfo->prev_cpu_system = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM]);
		cpu_hotplug_cpuinfo->prev_cpu_others = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ] + kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ]
																		+ kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL] + kcpustat_cpu(cpu).cpustat[CPUTIME_NICE]);

		cpu_hotplug_cpuinfo->prev_cpu_idle = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IDLE]);
		cpu_hotplug_cpuinfo->prev_cpu_iowait = (__force unsigned long)(kcpustat_cpu(cpu).cpustat[CPUTIME_IOWAIT]);
	}
	mutex_init(&timer_mutex);
	mutex_unlock(&alucard_hotplug_mutex);

	delay = usecs_to_jiffies(atomic_read(&hotplug_tuners_ins.hotplug_sampling_rate));
	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	INIT_DELAYED_WORK(&alucard_hotplug_work, hotplug_work_fn);
	schedule_delayed_work_on(0, &alucard_hotplug_work, delay);

#ifndef CONFIG_CPU_EXYNOS4210
#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&alucard_hotplug_early_suspend_struct_driver);
#endif
#endif

	return ret;
}

static void __exit alucard_hotplug_exit(void)
{
	cancel_delayed_work_sync(&alucard_hotplug_work);
	mutex_destroy(&timer_mutex);

#ifndef CONFIG_CPU_EXYNOS4210
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&alucard_hotplug_early_suspend_struct_driver);
#endif
#endif
}
MODULE_AUTHOR("Alucard_24@XDA");
MODULE_DESCRIPTION("'alucard_hotplug' - A cpu hotplug driver for "
	"capable processors");
MODULE_LICENSE("GPL");
module_init(alucard_hotplug_init);
module_exit(alucard_hotplug_exit);
