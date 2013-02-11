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
#include <linux/suspend.h>
#include <linux/reboot.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#define EARLYSUSPEND_HOTPLUGLOCK 1

/*
 * runqueue average
 */

#define RQ_AVG_TIMER_RATE	10

struct runqueue_data {
	unsigned int nr_run_avg;
	unsigned int update_rate;
	int64_t last_time;
	int64_t total_time;
	struct delayed_work work;
	struct workqueue_struct *nr_run_wq;
	spinlock_t lock;
};

static struct runqueue_data *rq_data;
static void rq_work_fn(struct work_struct *work);

static void start_rq_work(void)
{
	rq_data->nr_run_avg = 0;
	rq_data->last_time = 0;
	rq_data->total_time = 0;
	if (rq_data->nr_run_wq == NULL)
		rq_data->nr_run_wq =
			create_singlethread_workqueue("nr_run_avg");

	queue_delayed_work(rq_data->nr_run_wq, &rq_data->work,
			   msecs_to_jiffies(rq_data->update_rate));
	return;
}

static void stop_rq_work(void)
{
	if (rq_data->nr_run_wq)
		cancel_delayed_work(&rq_data->work);
	return;
}

static int __init init_rq_avg(void)
{
	rq_data = kzalloc(sizeof(struct runqueue_data), GFP_KERNEL);
	if (rq_data == NULL) {
		pr_err("%s cannot allocate memory\n", __func__);
		return -ENOMEM;
	}
	spin_lock_init(&rq_data->lock);
	rq_data->update_rate = RQ_AVG_TIMER_RATE;
	INIT_DEFERRABLE_WORK(&rq_data->work, rq_work_fn);

	return 0;
}

static void rq_work_fn(struct work_struct *work)
{
	int64_t time_diff = 0;
	int64_t nr_run = 0;
	unsigned long flags = 0;
	int64_t cur_time = ktime_to_ns(ktime_get());

	spin_lock_irqsave(&rq_data->lock, flags);

	if (rq_data->last_time == 0)
		rq_data->last_time = cur_time;
	if (rq_data->nr_run_avg == 0)
		rq_data->total_time = 0;

	nr_run = nr_running() * 100;
	time_diff = cur_time - rq_data->last_time;
	do_div(time_diff, 1000 * 1000);

	if (time_diff != 0 && rq_data->total_time != 0) {
		nr_run = (nr_run * time_diff) +
			(rq_data->nr_run_avg * rq_data->total_time);
		do_div(nr_run, rq_data->total_time + time_diff);
	}
	rq_data->nr_run_avg = nr_run;
	rq_data->total_time += time_diff;
	rq_data->last_time = cur_time;

	if (rq_data->update_rate != 0)
		queue_delayed_work(rq_data->nr_run_wq, &rq_data->work,
				   msecs_to_jiffies(rq_data->update_rate));

	spin_unlock_irqrestore(&rq_data->lock, flags);
}

static unsigned int get_nr_run_avg(void)
{
	unsigned int nr_run_avg;
	unsigned long flags = 0;

	spin_lock_irqsave(&rq_data->lock, flags);
	nr_run_avg = rq_data->nr_run_avg;
	rq_data->nr_run_avg = 0;
	spin_unlock_irqrestore(&rq_data->lock, flags);

	return nr_run_avg;
}


/*
 * dbs is used in this file as a shortform for demandbased switching
 * It helps to keep variable names smaller, simpler
 */

#define DEF_SAMPLING_UP_FACTOR			(1)
#define MAX_SAMPLING_UP_FACTOR		(100000)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define DEF_FREQ_STEP_DEC		(5)

#define DEF_SAMPLING_RATE			(60000)
#define MIN_SAMPLING_RATE			(10000)
#define MAX_HOTPLUG_RATE			(40u)

#define DEF_MAX_CPU_LOCK			(0)
#define DEF_MIN_CPU_LOCK			(0)
#define DEF_UP_NR_CPUS				(1)
#define DEF_FREQ_STEP				(30)

#define DEF_START_DELAY				(0)

#define FREQ_FOR_RESPONSIVENESS			(400000)
#define FIRST_CORE_FREQ_LIMIT			(0)
#define SECOND_CORE_FREQ_LIMIT			(0)

#define HOTPLUG_DOWN_INDEX			(0)
#define HOTPLUG_UP_INDEX			(1)

/* CPU freq will be increased if measured load > inc_cpu_load;*/
#define DEF_INC_CPU_LOAD (80)
#define INC_CPU_LOAD_AT_MIN_FREQ		(40)
/* CPU freq will be decreased if measured load < dec_cpu_load;*/
#define DEF_DEC_CPU_LOAD (60)
#define DEF_FREQ_UP_BRAKE				(5u)
/* HOTPLUG FROM STANDALONE */
#define CPU1_ON_FREQ 800000
#define CPU1_OFF_FREQ 800000
#define TRANS_LOAD_H0 20
#define TRANS_LOAD_L1 20
#define TRANS_LOAD_H1 100
#define TRANS_LOAD_H0_SCROFF 20
#define TRANS_LOAD_L1_SCROFF 20
#define TRANS_LOAD_H1_SCROFF 100
#define TRANS_RQ 2
#define TRANS_LOAD_RQ 20
#define CPU_OFF 0
#define CPU_ON  1

#define NUM_CPUS num_possible_cpus()
#define CPULOAD_TABLE (NR_CPUS + 1)
/* HOTPLUG FROM STANDALONE */

static unsigned int min_sampling_rate;
static void do_dbs_timer(struct work_struct *work);
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

/* Sampling types */
enum {DBS_NORMAL_SAMPLE, DBS_SUB_SAMPLE};

static bool screen_off;

enum flag{
	HOTPLUG_NOP,
	HOTPLUG_IN,
	HOTPLUG_OUT
};

struct cpu_hotplug_info {
	unsigned long nr_running;
	pid_t tgid;
};

struct cpufreq_nightmare_cpuinfo {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_iowait;
	cputime64_t prev_cpu_wall;
	cputime64_t prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	struct work_struct up_work;
	struct work_struct down_work;
	struct cpufreq_frequency_table *freq_table;
	unsigned int rate_mult;
	int cpu;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpufreq_nightmare_cpuinfo, od_cpu_dbs_info);

struct workqueue_struct *dvfs_workqueues;

static unsigned int dbs_enable;	/* number of CPUs using this policy */


/*
 * dbs_mutex protects dbs_enable in governor start/stop.
 */
static DEFINE_MUTEX(dbs_mutex);

static struct dbs_tuners {
	unsigned int sampling_rate;
	unsigned int freq_step_dec;
	unsigned int ignore_nice;
	unsigned int sampling_down_factor;
	unsigned int io_is_busy;
	/* nightmare tuners */
	unsigned int freq_step;
	unsigned int up_nr_cpus;
	unsigned int max_cpu_lock;
	unsigned int min_cpu_lock;
	atomic_t hotplug_lock;
	unsigned int dvfs_debug;
	unsigned int max_freq;
	unsigned int min_freq;
#ifdef CONFIG_HAS_EARLYSUSPEND
	int early_suspend;
#endif
	unsigned int inc_cpu_load_at_min_freq;
	unsigned int freq_for_responsiveness;
	unsigned int first_core_freq_limit;
	unsigned int second_core_freq_limit;
	unsigned int inc_cpu_load;
	unsigned int dec_cpu_load;
	unsigned int sampling_up_factor;
	unsigned int freq_up_brake;
	unsigned int freq_cpu1on;
	unsigned int freq_cpu1off;
	unsigned int trans_rq;
	unsigned int trans_load_rq;
	unsigned int trans_load_h0;
	unsigned int trans_load_l1;
	unsigned int trans_load_h1;
	unsigned int trans_load_h0_scroff;
	unsigned int trans_load_l1_scroff;
	unsigned int trans_load_h1_scroff;
#if (NR_CPUS > 2)
	unsigned int trans_load_l2;
	unsigned int trans_load_h2;
	unsigned int trans_load_l3;
#endif

} dbs_tuners_ins = {
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.freq_step_dec = DEF_FREQ_STEP_DEC,
	.ignore_nice = 0,
	.freq_step = DEF_FREQ_STEP,
	.up_nr_cpus = DEF_UP_NR_CPUS,
	.max_cpu_lock = DEF_MAX_CPU_LOCK,
	.min_cpu_lock = DEF_MIN_CPU_LOCK,
	.hotplug_lock = ATOMIC_INIT(0),
	.dvfs_debug = 0,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.early_suspend = -1,
#endif
	.inc_cpu_load_at_min_freq = INC_CPU_LOAD_AT_MIN_FREQ,
	.freq_for_responsiveness = FREQ_FOR_RESPONSIVENESS,
	.first_core_freq_limit = FIRST_CORE_FREQ_LIMIT,
	.second_core_freq_limit = SECOND_CORE_FREQ_LIMIT,
	.inc_cpu_load = DEF_INC_CPU_LOAD,
	.dec_cpu_load = DEF_DEC_CPU_LOAD,
	.sampling_up_factor = DEF_SAMPLING_UP_FACTOR,
	.freq_up_brake = DEF_FREQ_UP_BRAKE,
	.freq_cpu1on = CPU1_ON_FREQ,
	.freq_cpu1off = CPU1_OFF_FREQ,
	.trans_rq = TRANS_RQ,
	.trans_load_rq = TRANS_LOAD_RQ,
	.trans_load_h0 = TRANS_LOAD_H0,
	.trans_load_l1 = TRANS_LOAD_L1,
	.trans_load_h1 = TRANS_LOAD_H1,
	.trans_load_h0_scroff = TRANS_LOAD_H0_SCROFF,
	.trans_load_l1_scroff = TRANS_LOAD_L1_SCROFF,
	.trans_load_h1_scroff = TRANS_LOAD_H1_SCROFF,
#if (NR_CPUS > 2)
	.trans_load_l2 = TRANS_LOAD_L2,
	.trans_load_h2 = TRANS_LOAD_H2,
	.trans_load_l3 = TRANS_LOAD_L3,
#endif
};


/*
 * CPU hotplug lock interface
 */

static atomic_t g_hotplug_count = ATOMIC_INIT(0);
static atomic_t g_hotplug_lock = ATOMIC_INIT(0);

static void apply_hotplug_lock(void)
{
	int online, possible, lock, flag;
	struct work_struct *work;
	struct cpufreq_nightmare_cpuinfo *dbs_info;

	/* do turn_on/off cpus */
	dbs_info = &per_cpu(od_cpu_dbs_info, 0); /* from CPU0 */
	online = num_online_cpus();
	possible = num_possible_cpus();
	lock = atomic_read(&g_hotplug_lock);
	flag = lock - online;

	if (lock == 0 || flag == 0)
		return;

	work = flag > 0 ? &dbs_info->up_work : &dbs_info->down_work;

	pr_debug("%s online %d possible %d lock %d flag %d %d\n",
		 __func__, online, possible, lock, flag, (int)abs(flag));

	queue_work_on(dbs_info->cpu, dvfs_workqueues, work);
}

int cpufreq_nightmare_cpu_lock(int num_core)
{
	int prev_lock;

	if (num_core < 1 || num_core > num_possible_cpus())
		return -EINVAL;

	prev_lock = atomic_read(&g_hotplug_lock);

	if (prev_lock != 0 && prev_lock < num_core)
		return -EINVAL;
	else if (prev_lock == num_core)
		atomic_inc(&g_hotplug_count);

	atomic_set(&g_hotplug_lock, num_core);
	atomic_set(&g_hotplug_count, 1);
	apply_hotplug_lock();

	return 0;
}

int cpufreq_nightmare_cpu_unlock(int num_core)
{
	int prev_lock = atomic_read(&g_hotplug_lock);

	if (prev_lock < num_core)
		return 0;
	else if (prev_lock == num_core && atomic_read(&g_hotplug_count) > 0)
		atomic_dec(&g_hotplug_count);

	if (atomic_read(&g_hotplug_count) == 0 && atomic_read(&g_hotplug_lock) > 0)
		atomic_set(&g_hotplug_lock, 0);

	return 0;
}

void cpufreq_nightmare_min_cpu_lock(unsigned int num_core)
{
	int online, flag;
	struct cpufreq_nightmare_cpuinfo *dbs_info;

	dbs_tuners_ins.min_cpu_lock = min(num_core, num_possible_cpus());

	dbs_info = &per_cpu(od_cpu_dbs_info, 0); /* from CPU0 */
	online = num_online_cpus();
	flag = (int)num_core - online;
	if (flag <= 0)
		return;
	queue_work_on(dbs_info->cpu, dvfs_workqueues, &dbs_info->up_work);
}

void cpufreq_nightmare_min_cpu_unlock(void)
{
	int online, lock, flag;
	struct cpufreq_nightmare_cpuinfo *dbs_info;

	dbs_tuners_ins.min_cpu_lock = 0;

	dbs_info = &per_cpu(od_cpu_dbs_info, 0); /* from CPU0 */
	online = num_online_cpus();
	lock = atomic_read(&g_hotplug_lock);
	if (lock == 0)
		return;
	flag = lock - online;
	if (flag >= 0)
		return;
	queue_work_on(dbs_info->cpu, dvfs_workqueues, &dbs_info->down_work);
}

/*
 * History of CPU usage
 */
struct cpu_usage {
	unsigned int freq[NR_CPUS];
	int load[NR_CPUS];
	unsigned int rq_avg;
	unsigned int avg_load;
	unsigned long nr_rq_min;
	unsigned int cpu_rq_min;
	unsigned int avg_freq;
};

struct cpu_usage_history {
	struct cpu_usage usage[MAX_HOTPLUG_RATE];
	unsigned int num_hist;
	unsigned int last_num_hist;
};

struct cpu_usage_history *hotplug_histories;

static inline u64 get_cpu_idle_time_jiffy(unsigned int cpu, u64 *wall)
{
	u64 idle_time;
	u64 cur_wall_time;
	u64 busy_time;

	cur_wall_time = jiffies64_to_cputime64(get_jiffies_64());

	busy_time  = kcpustat_cpu(cpu).cpustat[CPUTIME_USER];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SYSTEM];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_IRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_SOFTIRQ];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_STEAL];
	busy_time += kcpustat_cpu(cpu).cpustat[CPUTIME_NICE];

	idle_time = cur_wall_time - busy_time;
	if (wall)
		*wall = jiffies_to_usecs(cur_wall_time);

	return jiffies_to_usecs(idle_time);
}

static inline cputime64_t get_cpu_idle_time(unsigned int cpu, cputime64_t *wall)
{
	u64 idle_time = get_cpu_idle_time_us(cpu, NULL);

	if (idle_time == -1ULL)
		return get_cpu_idle_time_jiffy(cpu, wall);
	else
		idle_time += get_cpu_iowait_time_us(cpu, wall);

	return idle_time;
}

static inline cputime64_t get_cpu_iowait_time(unsigned int cpu,
					      cputime64_t *wall)
{
	u64 iowait_time = get_cpu_iowait_time_us(cpu, wall);

	if (iowait_time == -1ULL)
		return 0;

	return iowait_time;
}

/************************** sysfs interface ************************/

static ssize_t show_sampling_rate_min(struct kobject *kobj,
				      struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", min_sampling_rate);
}

define_one_global_ro(sampling_rate_min);

/* cpufreq_nightmare Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_rate, sampling_rate);
show_one(io_is_busy, io_is_busy);
show_one(sampling_down_factor, sampling_down_factor);
show_one(ignore_nice_load, ignore_nice);
show_one(freq_step_dec, freq_step_dec);
show_one(freq_step, freq_step);
show_one(up_nr_cpus, up_nr_cpus);
show_one(max_cpu_lock, max_cpu_lock);
show_one(min_cpu_lock, min_cpu_lock);
show_one(dvfs_debug, dvfs_debug);
show_one(inc_cpu_load_at_min_freq, inc_cpu_load_at_min_freq);
show_one(freq_for_responsiveness, freq_for_responsiveness);
show_one(first_core_freq_limit, first_core_freq_limit);
show_one(second_core_freq_limit, second_core_freq_limit);
show_one(inc_cpu_load, inc_cpu_load);
show_one(dec_cpu_load, dec_cpu_load);
show_one(sampling_up_factor, sampling_up_factor);
show_one(freq_up_brake, freq_up_brake);
show_one(freq_cpu1on, freq_cpu1on);
show_one(freq_cpu1off, freq_cpu1off);
show_one(trans_rq, trans_rq);
show_one(trans_load_rq, trans_load_rq);
show_one(trans_load_h0, trans_load_h0);
show_one(trans_load_l1, trans_load_l1);
show_one(trans_load_h1, trans_load_h1);
show_one(trans_load_h0_scroff, trans_load_h0_scroff);
show_one(trans_load_l1_scroff, trans_load_l1_scroff);
show_one(trans_load_h1_scroff, trans_load_h1_scroff);
#if (NR_CPUS > 2)
show_one(trans_load_l2, trans_load_l2);
show_one(trans_load_h2, trans_load_h2);
show_one(trans_load_l3, trans_load_l3);
#endif

static ssize_t show_hotplug_lock(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", atomic_read(&g_hotplug_lock));
}

static ssize_t store_sampling_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_rate = max(input, min_sampling_rate);
	return count;
}

static ssize_t store_io_is_busy(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.io_is_busy = !!input;
	return count;
}

static ssize_t store_sampling_down_factor(struct kobject *a,
					  struct attribute *b,
					  const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_DOWN_FACTOR || input < 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_down_factor = input;

	return count;
}

static ssize_t store_ignore_nice_load(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	unsigned int j;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1)
		input = 1;

	if (input == dbs_tuners_ins.ignore_nice) { /* nothing to do */
		return count;
	}
	dbs_tuners_ins.ignore_nice = input;

	/* we need to re-evaluate prev_cpu_idle */
	for_each_online_cpu(j) {
		struct cpufreq_nightmare_cpuinfo *dbs_info;
		dbs_info = &per_cpu(od_cpu_dbs_info, j);
		dbs_info->prev_cpu_idle =
			get_cpu_idle_time(j, &dbs_info->prev_cpu_wall);
		if (dbs_tuners_ins.ignore_nice)
			dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
	}
	return count;
}

static ssize_t store_freq_step_dec(struct kobject *a, struct attribute *b,
				       const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.freq_step_dec = min(input, 100u);
	return count;
}

static ssize_t store_freq_step(struct kobject *a, struct attribute *b,
			       const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.freq_step = min(input, 100u);
	return count;
}

static ssize_t store_up_nr_cpus(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.up_nr_cpus = min(input, num_possible_cpus());
	return count;
}

static ssize_t store_max_cpu_lock(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.max_cpu_lock = min(input, num_possible_cpus());
	return count;
}

static ssize_t store_min_cpu_lock(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	if (input == 0)
		cpufreq_nightmare_min_cpu_unlock();
	else
		cpufreq_nightmare_min_cpu_lock(input);
	return count;
}

static ssize_t store_hotplug_lock(struct kobject *a, struct attribute *b,
				  const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	int prev_lock;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	input = min(input, num_possible_cpus());
	prev_lock = atomic_read(&dbs_tuners_ins.hotplug_lock);

	/* Fix: If input = 0 set all parameters to 0 and go out */
	if (input == 0) {
		atomic_set(&g_hotplug_lock, 0);
		atomic_set(&g_hotplug_count, 0);
		atomic_set(&dbs_tuners_ins.hotplug_lock, 0);
		return count;
	}

	if (prev_lock)
		cpufreq_nightmare_cpu_unlock(prev_lock);

	ret = cpufreq_nightmare_cpu_lock(input);
	if (ret) {
		printk(KERN_ERR "[HOTPLUG] already locked with smaller value %d < %d\n",
			atomic_read(&g_hotplug_lock), input);
		return ret;
	}

	atomic_set(&dbs_tuners_ins.hotplug_lock, input);

	return count;
}

static ssize_t store_dvfs_debug(struct kobject *a, struct attribute *b,
				const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.dvfs_debug = input > 0;
	return count;
}

static ssize_t store_inc_cpu_load_at_min_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100) {
		return -EINVAL;
	}
	dbs_tuners_ins.inc_cpu_load_at_min_freq = min(input,dbs_tuners_ins.inc_cpu_load);
	return count;
}

static ssize_t store_freq_for_responsiveness(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.freq_for_responsiveness = input;
	return count;
}

/* first_core_freq_limit */
static ssize_t store_first_core_freq_limit(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1200000)	
		dbs_tuners_ins.first_core_freq_limit = 1200000;
	else
		dbs_tuners_ins.first_core_freq_limit = input;

	return count;
}

/* second_core_freq_limit */
static ssize_t store_second_core_freq_limit(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	if (input > 1200000)	
		dbs_tuners_ins.second_core_freq_limit = 1200000;
	else
		dbs_tuners_ins.second_core_freq_limit = input;
	return count;
}


/* inc_cpu_load */
static ssize_t store_inc_cpu_load(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.inc_cpu_load = max(min(input,100u),10u);
	return count;
}

/* dec_cpu_load */
static ssize_t store_dec_cpu_load(struct kobject *a, struct attribute *b,
					const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.dec_cpu_load = max(min(input,95u),5u);
	return count;
}

/* sampling_up_factor */
static ssize_t store_sampling_up_factor(struct kobject *a,
					  struct attribute *b,
					  const char *buf, size_t count)
{
	unsigned int input, j;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > MAX_SAMPLING_UP_FACTOR || input < 1)
		return -EINVAL;
	dbs_tuners_ins.sampling_up_factor = input;
	
	return count;
}

/* freq_up_brake */
static ssize_t store_freq_up_brake(struct kobject *a, struct attribute *b,
				      const char *buf, size_t count)
{
	unsigned int input;
	int ret;

	ret = sscanf(buf, "%u", &input);
	if (ret != 1 || input < 0 || input > 100)
		return -EINVAL;

	if (input == dbs_tuners_ins.freq_up_brake) { /* nothing to do */
		return count;
	}

	dbs_tuners_ins.freq_up_brake = input;

	return count;
}

/* freq_cpu1on */
static ssize_t store_freq_cpu1on(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.freq_cpu1on = input;
	return count;
}
/* freq_cpu1off */
static ssize_t store_freq_cpu1off(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.freq_cpu1off = input;
	return count;
}

/* trans_rq */
static ssize_t store_trans_rq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.trans_rq = input;
	return count;
}
/* trans_load_rq */
static ssize_t store_trans_load_rq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.trans_load_rq = input;
	return count;
}
/* trans_load_h0 */
static ssize_t store_trans_load_h0(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.trans_load_h0 = input;
	return count;
}
/* trans_load_l1 */
static ssize_t store_trans_load_l1(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.trans_load_l1 = input;
	return count;
}
/* trans_load_h1 */
static ssize_t store_trans_load_h1(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.trans_load_h1 = input;
	return count;
}
/* trans_load_h0_scroff */
static ssize_t store_trans_load_h0_scroff(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.trans_load_h0_scroff = input;
	return count;
}
/* trans_load_l1_scroff */
static ssize_t store_trans_load_l1_scroff(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.trans_load_l1_scroff = input;
	return count;
}
/* trans_load_h1_scroff */
static ssize_t store_trans_load_h1_scroff(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.trans_load_h1_scroff = input;
	return count;
}

#if (NR_CPUS > 2)
	/* trans_load_l2 */
	static ssize_t store_trans_load_l2(struct kobject *a, struct attribute *b,
					   const char *buf, size_t count)
	{
		unsigned int input;
		int ret;
		ret = sscanf(buf, "%u", &input);
		if (ret != 1)
			return -EINVAL;
		dbs_tuners_ins.trans_load_l2 = input;
		return count;
	}
	/* trans_load_h2 */
	static ssize_t store_trans_load_h2(struct kobject *a, struct attribute *b,
					   const char *buf, size_t count)
	{
		unsigned int input;
		int ret;
		ret = sscanf(buf, "%u", &input);
		if (ret != 1)
			return -EINVAL;
		dbs_tuners_ins.trans_load_h2 = input;
		return count;
	}
	/* trans_load_l3 */
	static ssize_t store_trans_load_l3(struct kobject *a, struct attribute *b,
					   const char *buf, size_t count)
	{
		unsigned int input;
		int ret;
		ret = sscanf(buf, "%u", &input);
		if (ret != 1)
			return -EINVAL;
		dbs_tuners_ins.trans_load_l3 = input;
		return count;
	}
#endif



define_one_global_rw(sampling_rate);
define_one_global_rw(io_is_busy);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(ignore_nice_load);
define_one_global_rw(freq_step_dec);
define_one_global_rw(freq_step);
define_one_global_rw(up_nr_cpus);
define_one_global_rw(max_cpu_lock);
define_one_global_rw(min_cpu_lock);
define_one_global_rw(hotplug_lock);
define_one_global_rw(dvfs_debug);
define_one_global_rw(inc_cpu_load_at_min_freq);
define_one_global_rw(freq_for_responsiveness);
define_one_global_rw(first_core_freq_limit);
define_one_global_rw(second_core_freq_limit);
define_one_global_rw(inc_cpu_load);
define_one_global_rw(dec_cpu_load);
define_one_global_rw(sampling_up_factor);
define_one_global_rw(freq_up_brake);
define_one_global_rw(freq_cpu1on);
define_one_global_rw(freq_cpu1off);
define_one_global_rw(trans_rq);
define_one_global_rw(trans_load_rq);
define_one_global_rw(trans_load_h0);
define_one_global_rw(trans_load_l1);
define_one_global_rw(trans_load_h1);
define_one_global_rw(trans_load_h0_scroff);
define_one_global_rw(trans_load_l1_scroff);
define_one_global_rw(trans_load_h1_scroff);
#if (NR_CPUS > 2)
define_one_global_rw(trans_load_l2);
define_one_global_rw(trans_load_h2);
define_one_global_rw(trans_load_l3);
#endif

static struct attribute *dbs_attributes[] = {
	&sampling_rate_min.attr,
	&sampling_rate.attr,
	&sampling_down_factor.attr,
	&ignore_nice_load.attr,
	&io_is_busy.attr,
	&freq_step_dec.attr,
	&freq_step.attr,
	&up_nr_cpus.attr,
	/* priority: hotplug_lock > max_cpu_lock > min_cpu_lock
	   Exception: hotplug_lock on early_suspend uses min_cpu_lock */
	&max_cpu_lock.attr,
	&min_cpu_lock.attr,
	&hotplug_lock.attr,
	&dvfs_debug.attr,
	&inc_cpu_load_at_min_freq.attr,
	&freq_for_responsiveness.attr,
	&first_core_freq_limit.attr,
	&second_core_freq_limit.attr,
	&inc_cpu_load.attr,
	&dec_cpu_load.attr,
	&sampling_up_factor.attr,
	&freq_up_brake.attr,
	&freq_cpu1on.attr,
	&freq_cpu1off.attr,
	&trans_rq.attr,
	&trans_load_rq.attr,
	&trans_load_h0.attr,
	&trans_load_l1.attr,
	&trans_load_h1.attr,
	&trans_load_h0_scroff.attr,
	&trans_load_l1_scroff.attr,
	&trans_load_h1_scroff.attr,
#if (NR_CPUS > 2)
	&trans_load_l2.attr,
	&trans_load_h2.attr,
	&trans_load_l3.attr,
#endif
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "nightmare",
};

/************************** sysfs end ************************/

static bool nightmare_hotplug_out_check(unsigned int nr_online_cpu, unsigned int threshold_up,
		unsigned int avg_load, unsigned int cur_freq)
{
	return ((nr_online_cpu > 1) &&
		(avg_load < threshold_up ||
		cur_freq < dbs_tuners_ins.freq_cpu1off));
}

static inline enum flag
standalone_hotplug(struct cpufreq_nightmare_cpuinfo *this_dbs_info)
{
	struct cpufreq_policy *policy;
	int num_hist = hotplug_histories->last_num_hist;
	unsigned int cur_freq;
	unsigned int nr_online_cpu;
	int load = 0;
	/*load threshold*/

	unsigned int threshold[CPULOAD_TABLE][2] = {
		{0, dbs_tuners_ins.trans_load_h0},
		{dbs_tuners_ins.trans_load_l1, dbs_tuners_ins.trans_load_h1},
#if (NR_CPUS > 2)
		{dbs_tuners_ins.trans_load_l2, dbs_tuners_ins.trans_load_h2},
		{dbs_tuners_ins.trans_load_l3, 100},
#endif
		{0, 0}
	};

	unsigned int threshold_scroff[CPULOAD_TABLE][2] = {
		{0, dbs_tuners_ins.trans_load_h0_scroff},
		{dbs_tuners_ins.trans_load_l1_scroff, dbs_tuners_ins.trans_load_h1_scroff},
#if (NR_CPUS > 2)
		{dbs_tuners_ins.trans_load_l2_scroff, dbs_tuners_ins.trans_load_h2_scroff},
		{dbs_tuners_ins.trans_load_l3_scroff, 100},
#endif
		{0, 0}
	};

	unsigned int avg_load;
	unsigned long nr_rq_min;
	unsigned int cpu_rq_min;

	static void __iomem *clk_fimc;
	unsigned char fimc_stat;

	policy = this_dbs_info->cur_policy;

	avg_load = hotplug_histories->usage[num_hist].avg_load;
	nr_rq_min = hotplug_histories->usage[num_hist].nr_rq_min;
	cpu_rq_min = hotplug_histories->usage[num_hist].cpu_rq_min;

	cur_freq = policy->cur;
	//cur_freq = hotplug_histories->usage[num_hist].avg_freq;

	nr_online_cpu = num_online_cpus();

	clk_fimc = ioremap(0x10020000, SZ_4K);
	fimc_stat = __raw_readl(clk_fimc + 0x0920);
	iounmap(clk_fimc);

	if ((fimc_stat>>4 & 0x1) == 1)
		return HOTPLUG_IN;

	if (nightmare_hotplug_out_check(nr_online_cpu, (screen_off ? threshold_scroff[nr_online_cpu-1][0] : threshold[nr_online_cpu - 1][0] ),
			    avg_load, cur_freq)) {
		return HOTPLUG_OUT;
		/* If total nr_running is less than cpu(on-state) number, hotplug do not hotplug-in */
	} else if (nr_running() > nr_online_cpu &&
		   avg_load > (screen_off ? threshold_scroff[nr_online_cpu-1][1] : threshold[nr_online_cpu - 1][1] )
		   && cur_freq >= dbs_tuners_ins.freq_cpu1on) {

		return HOTPLUG_IN;
	} else if (nr_online_cpu > 1 && nr_rq_min < dbs_tuners_ins.trans_rq) {
		
		load = hotplug_histories->usage[num_hist].load[cpu_rq_min];
		/*If CPU(cpu_rq_min) load is less than trans_load_rq, hotplug-out*/
		if (load < dbs_tuners_ins.trans_load_rq)
			return HOTPLUG_OUT;
	}

	return HOTPLUG_NOP;
}

static void cpu_up_work(struct work_struct *work)
{
	int cpu;
	int online = num_online_cpus();
	int nr_up = dbs_tuners_ins.up_nr_cpus;
	int min_cpu_lock = dbs_tuners_ins.min_cpu_lock;
	int hotplug_lock = atomic_read(&g_hotplug_lock);

	if (hotplug_lock && min_cpu_lock)
		nr_up = max(hotplug_lock, min_cpu_lock) - online;
	else if (hotplug_lock)
		nr_up = hotplug_lock - online;
	else if (min_cpu_lock)
		nr_up = max(nr_up, min_cpu_lock - online);

	if (online == 1) {
		printk(KERN_ERR "CPU_UP 3\n");
		cpu_up(num_possible_cpus() - 1);
		nr_up -= 1;
	}

	for_each_cpu_not(cpu, cpu_online_mask) {
		if (nr_up-- == 0)
			break;
		if (cpu == 0)
			continue;
		printk(KERN_ERR "CPU_UP %d\n", cpu);
		cpu_up(cpu);
	}
}

static void cpu_down_work(struct work_struct *work)
{
	int cpu;
	int online = num_online_cpus();
	int nr_down = 1;
	int hotplug_lock = atomic_read(&g_hotplug_lock);

	if (hotplug_lock)
		nr_down = online - hotplug_lock;

	for_each_online_cpu(cpu) {
		if (cpu == 0)
			continue;
		printk(KERN_ERR "CPU_DOWN %d\n", cpu);
		cpu_down(cpu);
		if (--nr_down == 0)
			break;
	}
}

static void debug_hotplug_check(int which, int rq_avg, int freq,
			 struct cpu_usage *usage)
{
	int cpu;
	printk(KERN_ERR "CHECK %s rq %d.%02d freq %d [", which ? "up" : "down",
	       rq_avg / 100, rq_avg % 100, freq);
	for_each_online_cpu(cpu) {
		printk(KERN_ERR "(%d, %d), ", cpu, usage->load[cpu]);
	}
	printk(KERN_ERR "]\n");
}

static void dbs_check_cpu(struct cpufreq_nightmare_cpuinfo *this_dbs_info)
{
	struct cpu_hotplug_info tmp_hotplug_info[4];
	struct cpufreq_policy *policy;
	unsigned int j;
	int num_hist = hotplug_histories->num_hist;
	int max_hotplug_rate = 20;
	/* add total_load, avg_load to get average load */
	unsigned int total_load = 0;
	unsigned int total_freq = 0;
	unsigned int avg_load = 0;
	unsigned int avg_freq = 0;
	unsigned int cpu_rq_min=0;
	unsigned long nr_rq_min = -1UL;
	unsigned int select_off_cpu = 0;
	unsigned int ncores = 0;
	int hotplug_lock = atomic_read(&g_hotplug_lock);

	enum flag flag_hotplug;

	policy = this_dbs_info->cur_policy;

	/* get last num_hist used */
	hotplug_histories->last_num_hist = num_hist;
	++hotplug_histories->num_hist;

	for_each_cpu(j, policy->cpus) {
		struct cpufreq_nightmare_cpuinfo *j_dbs_info;
		cputime64_t cur_wall_time, cur_idle_time, cur_iowait_time;
		cputime64_t prev_wall_time, prev_idle_time, prev_iowait_time;
		unsigned int idle_time, wall_time, iowait_time;
		int load = 0;
		unsigned int freq = 0;

		/* reset variables percpu*/
		hotplug_histories->usage[num_hist].load[j] = 0;
		hotplug_histories->usage[num_hist].freq[j] = 0;

		j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
		
		prev_wall_time = j_dbs_info->prev_cpu_wall;
		prev_idle_time = j_dbs_info->prev_cpu_idle;
		prev_iowait_time = j_dbs_info->prev_cpu_iowait;

		cur_idle_time = get_cpu_idle_time(j, &cur_wall_time);
		cur_iowait_time = get_cpu_iowait_time(j, &cur_wall_time);

		wall_time = (unsigned int) cputime64_sub(cur_wall_time,
							 prev_wall_time);
		j_dbs_info->prev_cpu_wall = cur_wall_time;

		idle_time = (unsigned int) cputime64_sub(cur_idle_time,
							 prev_idle_time);
		j_dbs_info->prev_cpu_idle = cur_idle_time;

		iowait_time = (unsigned int) cputime64_sub(cur_iowait_time,
							   prev_iowait_time);
		j_dbs_info->prev_cpu_iowait = cur_iowait_time;

		if (dbs_tuners_ins.ignore_nice) {
			u64 cur_nice;
			unsigned long cur_nice_jiffies;

			cur_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE] -
						 j_dbs_info->prev_cpu_nice;
			/*
			 * Assumption: nice time between sampling periods will
			 * be less than 2^32 jiffies for 32 bit sys
			 */
			cur_nice_jiffies = (unsigned long)
				cputime64_to_jiffies64(cur_nice);

			j_dbs_info->prev_cpu_nice = kcpustat_cpu(j).cpustat[CPUTIME_NICE];
			idle_time += jiffies_to_usecs(cur_nice_jiffies);
		}
		
		if (dbs_tuners_ins.io_is_busy && idle_time >= iowait_time)
			idle_time -= iowait_time;

		if (unlikely(!wall_time || wall_time < idle_time))
			continue;

		ncores++;

		// LOAD
		load = 100 * (wall_time - idle_time) / wall_time;
		hotplug_histories->usage[num_hist].load[j] = load;
		total_load += load;

		// FREQUENCY
		freq = j_dbs_info->cur_policy->cur;	
		total_freq += freq;
		hotplug_histories->usage[num_hist].freq[j] = freq;

		/*find minimum runqueue length*/
		tmp_hotplug_info[j].nr_running = get_cpu_nr_running(j);

		if (j && nr_rq_min > tmp_hotplug_info[j].nr_running) {
			nr_rq_min = tmp_hotplug_info[j].nr_running;
			cpu_rq_min = j;
		}
	}
	/* calculate the average load across all related CPUs */
	avg_load = total_load / ncores;
	avg_freq = total_freq / ncores;

	hotplug_histories->usage[num_hist].avg_load = avg_load;	
	hotplug_histories->usage[num_hist].avg_freq = avg_freq;
	hotplug_histories->usage[num_hist].nr_rq_min = nr_rq_min;
	hotplug_histories->usage[num_hist].cpu_rq_min = cpu_rq_min;

	if (hotplug_lock > 0)
		return;

	if (dbs_tuners_ins.max_cpu_lock != 0
		&& num_online_cpus() == dbs_tuners_ins.max_cpu_lock)
		return;

	if (dbs_tuners_ins.min_cpu_lock != 0
		&& num_online_cpus() == dbs_tuners_ins.min_cpu_lock)
		return;

	for (j = NUM_CPUS - 1; j > 0; --j) {
		if (cpu_online(j) == 0) {
			select_off_cpu = j;
			break;
		}
	}

	/*standallone hotplug*/
	flag_hotplug = standalone_hotplug(this_dbs_info);

	/*do not ever hotplug out CPU 0*/
	if((cpu_rq_min == 0) && (flag_hotplug == HOTPLUG_OUT))
		return;

	/*cpu hotplug*/
	if (flag_hotplug == HOTPLUG_IN && cpu_online(select_off_cpu) == CPU_OFF) {
		queue_work_on(this_dbs_info->cpu, dvfs_workqueues,&this_dbs_info->up_work);
		//DBG_PRINT("cpu%d on\n", select_off_cpu);
	} else if (flag_hotplug == HOTPLUG_OUT && cpu_online(cpu_rq_min) == CPU_ON) {
		queue_work_on(this_dbs_info->cpu, dvfs_workqueues,&this_dbs_info->down_work);
		//DBG_PRINT("cpu%d off!\n", cpu_rq_min);
	} 
	if (hotplug_histories->num_hist == max_hotplug_rate)
		hotplug_histories->num_hist = 0;
}


static void dbs_check_frequency(struct cpufreq_nightmare_cpuinfo *this_dbs_info)
{
	int j;
	int num_hist = hotplug_histories->last_num_hist;
	int inc_cpu_load = dbs_tuners_ins.inc_cpu_load;
	int dec_cpu_load = dbs_tuners_ins.dec_cpu_load;
	unsigned int freq_step = dbs_tuners_ins.freq_step;
	unsigned int freq_up_brake = dbs_tuners_ins.freq_up_brake;
	unsigned int freq_step_dec = dbs_tuners_ins.freq_step_dec;
	unsigned int inc_load=0;
	unsigned int inc_brake=0;
	unsigned int freq_up = 0;
	unsigned int dec_load = 0;
	unsigned int freq_down = 0;
	unsigned int first_core_freq_limit = dbs_tuners_ins.first_core_freq_limit;
	unsigned int second_core_freq_limit = dbs_tuners_ins.second_core_freq_limit;
	unsigned int ccore = 0;

	for_each_online_cpu(j) {
		struct cpufreq_policy *policy;
		unsigned int load = 0;

		load = hotplug_histories->usage[num_hist].load[j];

		ccore++;

		policy = cpufreq_cpu_get(j);
		if (!policy)
			continue;

		policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
		cpumask_setall(policy->related_cpus);
		cpumask_setall(policy->cpus);

		/* I need to integrate into exynos_cpufreq.c */
		/*policy->cpu = j;*/

		/* CPUs Online Scale Frequency*/
		if (policy->cur < dbs_tuners_ins.freq_for_responsiveness)
			inc_cpu_load = dbs_tuners_ins.inc_cpu_load_at_min_freq;
		else
			inc_cpu_load = dbs_tuners_ins.inc_cpu_load;

		// Check for frequency increase or for frequency decrease
		if (load >= inc_cpu_load) {
			this_dbs_info->rate_mult = dbs_tuners_ins.sampling_up_factor;

			// if we cannot increment the frequency anymore, break out early
			if (policy->cur == policy->max) {
				cpufreq_cpu_put(policy);
				continue;
			}

			inc_load = ((load * policy->min) / 100) + ((freq_step * policy->min) / 100);
			inc_brake = (freq_up_brake * policy->min) / 100;

			if (inc_brake > inc_load) {
				cpufreq_cpu_put(policy);
				continue;
			} else {
				freq_up = policy->cur + (inc_load - inc_brake);
			}			

			if (ccore == 1 && first_core_freq_limit > 0) {
				if (freq_up > first_core_freq_limit) {
					freq_up = min(first_core_freq_limit,policy->max);
				}
			} else if (ccore == 2 && second_core_freq_limit > 0) {
				if (freq_up > second_core_freq_limit) {
					freq_up = min(second_core_freq_limit,policy->max);
				}
			}

			if (freq_up != policy->cur && freq_up <= policy->max) {
				__cpufreq_driver_target(policy, freq_up, CPUFREQ_RELATION_L);
			}

		} else if (load <	 dec_cpu_load && load > 0) {
			this_dbs_info->rate_mult = dbs_tuners_ins.sampling_down_factor;

			// if we cannot reduce the frequency anymore, break out early
			if (policy->cur == policy->min) {
				cpufreq_cpu_put(policy);
				continue;
			}
	
			dec_load = (((100 - load) * policy->min) / 100) + ((freq_step_dec * policy->min) / 100);

			if (policy->cur > dec_load + policy->min) {
				freq_down = policy->cur - dec_load;
			} else {
				freq_down = policy->min;
			}

			if (ccore == 1 && first_core_freq_limit > 0) {
				if (freq_down > first_core_freq_limit) {
					freq_down = max(first_core_freq_limit,policy->min);
				}
			} else if (ccore == 2 && second_core_freq_limit > 0) {
				if (freq_down > second_core_freq_limit) {
					freq_down = max(second_core_freq_limit,policy->min);
				}
			}

			if (freq_down != policy->cur) {
				__cpufreq_driver_target(policy, freq_down, CPUFREQ_RELATION_L);
			}
		}
		cpufreq_cpu_put(policy);
	}
	return;
}

static void do_dbs_timer(struct work_struct *work)
{
	struct cpufreq_nightmare_cpuinfo *dbs_info =
		container_of(work, struct cpufreq_nightmare_cpuinfo, work.work);
	unsigned int cpu = dbs_info->cpu;
	int delay;

	mutex_lock(&dbs_info->timer_mutex);

	dbs_check_cpu(dbs_info);
	dbs_check_frequency(dbs_info);
	/* We want all CPUs to do sampling nearly on
	 * same jiffy
	 */
	delay = usecs_to_jiffies(dbs_tuners_ins.sampling_rate * (dbs_info->rate_mult < 1 ? 1 : dbs_info->rate_mult));

	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	queue_delayed_work_on(cpu, dvfs_workqueues, &dbs_info->work, delay);
	mutex_unlock(&dbs_info->timer_mutex);
}

static inline void dbs_timer_init(struct cpufreq_nightmare_cpuinfo *dbs_info)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(DEF_START_DELAY * 1000 * 1000
				     + dbs_tuners_ins.sampling_rate);

	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	INIT_DEFERRABLE_WORK(&dbs_info->work, do_dbs_timer);
	INIT_WORK(&dbs_info->up_work, cpu_up_work);
	INIT_WORK(&dbs_info->down_work, cpu_down_work);

	queue_delayed_work_on(dbs_info->cpu, dvfs_workqueues,
			      &dbs_info->work, delay + 2 * HZ);
}

static inline void dbs_timer_exit(struct cpufreq_nightmare_cpuinfo *dbs_info)
{
	cancel_delayed_work_sync(&dbs_info->work);
	cancel_work_sync(&dbs_info->up_work);
	cancel_work_sync(&dbs_info->down_work);
}

static int pm_notifier_call(struct notifier_block *this,
			    unsigned long event, void *ptr)
{
	static unsigned int prev_hotplug_lock;
	switch (event) {
	case PM_SUSPEND_PREPARE:
		prev_hotplug_lock = atomic_read(&g_hotplug_lock);
		atomic_set(&g_hotplug_lock, 1);
		apply_hotplug_lock();
		pr_debug("%s enter suspend\n", __func__);
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		atomic_set(&g_hotplug_lock, prev_hotplug_lock);
		if (prev_hotplug_lock)
			apply_hotplug_lock();
		prev_hotplug_lock = 0;
		pr_debug("%s exit suspend\n", __func__);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block pm_notifier = {
	.notifier_call = pm_notifier_call,
};

static int reboot_notifier_call(struct notifier_block *this,
				unsigned long code, void *_cmd)
{
	atomic_set(&g_hotplug_lock, 1);
	return NOTIFY_DONE;
}

static struct notifier_block reboot_notifier = {
	.notifier_call = reboot_notifier_call,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend early_suspend;
unsigned int previous_freq_step;
unsigned int previous_sampling_rate;
static void cpufreq_nightmare_early_suspend(struct early_suspend *h)
{
#if EARLYSUSPEND_HOTPLUGLOCK
	dbs_tuners_ins.early_suspend =
		atomic_read(&g_hotplug_lock);
#endif
	screen_off = true;
	previous_freq_step = dbs_tuners_ins.freq_step;
	previous_sampling_rate = dbs_tuners_ins.sampling_rate;
	dbs_tuners_ins.freq_step = 10;
	dbs_tuners_ins.sampling_rate = 200000;
#if EARLYSUSPEND_HOTPLUGLOCK
	atomic_set(&g_hotplug_lock,
	    (dbs_tuners_ins.min_cpu_lock) ? dbs_tuners_ins.min_cpu_lock : 1);
	apply_hotplug_lock();
	stop_rq_work();
#endif
}
static void cpufreq_nightmare_late_resume(struct early_suspend *h)
{
#if EARLYSUSPEND_HOTPLUGLOCK
	atomic_set(&g_hotplug_lock, dbs_tuners_ins.early_suspend);
#endif
	screen_off = false;
	dbs_tuners_ins.early_suspend = -1;
	dbs_tuners_ins.freq_step = previous_freq_step;
	dbs_tuners_ins.sampling_rate = previous_sampling_rate;
#if EARLYSUSPEND_HOTPLUGLOCK
	apply_hotplug_lock();
	start_rq_work();
#endif
}
#endif

static int cpufreq_governor_nightmare(struct cpufreq_policy *policy,
				unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpufreq_nightmare_cpuinfo *this_dbs_info;
	struct cpufreq_frequency_table *freq_table;
	unsigned int j;
	int rc;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(cpu)) || (!policy->cur))
			return -EINVAL;

		policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
		cpumask_setall(policy->related_cpus);
		cpumask_setall(policy->cpus);

		// FIX HOTPLUG_LOCK AT GOV START
		atomic_set(&dbs_tuners_ins.hotplug_lock, 0);

		dbs_tuners_ins.max_freq = policy->max;
		dbs_tuners_ins.min_freq = policy->min;
		hotplug_histories->num_hist = 0;
		hotplug_histories->last_num_hist = 0;
		start_rq_work();

		mutex_lock(&dbs_mutex);

		dbs_enable++;
		for_each_cpu(j, policy->cpus) {
			struct cpufreq_nightmare_cpuinfo *j_dbs_info;
			j_dbs_info = &per_cpu(od_cpu_dbs_info, j);
			j_dbs_info->cur_policy = policy;

			j_dbs_info->prev_cpu_idle = get_cpu_idle_time(j,
				&j_dbs_info->prev_cpu_wall);
			if (dbs_tuners_ins.ignore_nice)
				j_dbs_info->prev_cpu_nice =
					kcpustat_cpu(j).cpustat[CPUTIME_NICE];
		}
		this_dbs_info->cpu = cpu;
		this_dbs_info->rate_mult = 1;
		/*
		 * Start the timerschedule work, when this governor
		 * is used for first time
		 */
		if (dbs_enable == 1) {
			rc = sysfs_create_group(cpufreq_global_kobject,
						&dbs_attr_group);
			if (rc) {
				mutex_unlock(&dbs_mutex);
				return rc;
			}

			min_sampling_rate = MIN_SAMPLING_RATE;
			dbs_tuners_ins.sampling_rate = DEF_SAMPLING_RATE;
			dbs_tuners_ins.io_is_busy = 0;
		}
		mutex_unlock(&dbs_mutex);

		register_reboot_notifier(&reboot_notifier);

		mutex_init(&this_dbs_info->timer_mutex);
		dbs_timer_init(this_dbs_info);

#if !EARLYSUSPEND_HOTPLUGLOCK
		register_pm_notifier(&pm_notifier);
#endif
#ifdef CONFIG_HAS_EARLYSUSPEND
		register_early_suspend(&early_suspend);
#endif
		break;

	case CPUFREQ_GOV_STOP:
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&early_suspend);
#endif
#if !EARLYSUSPEND_HOTPLUGLOCK
		unregister_pm_notifier(&pm_notifier);
#endif

		dbs_timer_exit(this_dbs_info);

		mutex_lock(&dbs_mutex);
		mutex_destroy(&this_dbs_info->timer_mutex);

		unregister_reboot_notifier(&reboot_notifier);

		dbs_enable--;
		mutex_unlock(&dbs_mutex);

		stop_rq_work();

		if (!dbs_enable)
			sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		mutex_lock(&this_dbs_info->timer_mutex);

		/*if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
						policy->max,
						CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
						policy->min,
						CPUFREQ_RELATION_L);*/

		for_each_online_cpu(j) {
			struct cpufreq_policy *cpu_policy;
			struct cpufreq_nightmare_cpuinfo *cpu_dbs_info;

			cpu_policy = cpufreq_cpu_get(j);
			if (!cpu_policy)
				continue;

			cpu_policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
			cpumask_setall(cpu_policy->related_cpus);
			cpumask_setall(cpu_policy->cpus);

			if (policy->max < cpu_policy->cur)
				__cpufreq_driver_target(cpu_policy,policy->max,CPUFREQ_RELATION_H);
			else if (policy->min > cpu_policy->cur)
				__cpufreq_driver_target(cpu_policy,policy->min,CPUFREQ_RELATION_L);

			cpu_dbs_info = &per_cpu(od_cpu_dbs_info, j);
			cpu_dbs_info->cur_policy = cpu_policy;

			cpufreq_cpu_put(policy);

		}
		mutex_unlock(&this_dbs_info->timer_mutex);
		break;
	}
	return 0;
}

static int __init cpufreq_gov_nightmare_init(void)
{
	int ret;

	ret = init_rq_avg();
	if (ret)
		return ret;

	hotplug_histories = kzalloc(sizeof(struct cpu_usage_history), GFP_KERNEL);
	if (!hotplug_histories) {
		pr_err("%s cannot create hotplug history array\n", __func__);
		ret = -ENOMEM;
		goto err_hist;
	}

	dvfs_workqueues = create_workqueue("knightmare");
	if (!dvfs_workqueues) {
		pr_err("%s cannot create workqueue\n", __func__);
		ret = -ENOMEM;
		goto err_queue;
	}

	ret = cpufreq_register_governor(&cpufreq_gov_nightmare);
	if (ret)
		goto err_reg;

#ifdef CONFIG_HAS_EARLYSUSPEND
	early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
	early_suspend.suspend = cpufreq_nightmare_early_suspend;
	early_suspend.resume = cpufreq_nightmare_late_resume;
#endif

	return ret;

err_reg:
	destroy_workqueue(dvfs_workqueues);
err_queue:
	kfree(hotplug_histories);
err_hist:
	kfree(rq_data);
	return ret;
}

static void __exit cpufreq_gov_nightmare_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_nightmare);
	destroy_workqueue(dvfs_workqueues);
	kfree(hotplug_histories);
	kfree(rq_data);
}

MODULE_AUTHOR("ByungChang Cha <bc.cha@samsung.com>");
MODULE_DESCRIPTION("'cpufreq_nightmare' - A dynamic cpufreq/cpuhotplug governor");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_NIGHTMARE
fs_initcall(cpufreq_gov_nightmare_init);
#else
module_init(cpufreq_gov_nightmare_init);
#endif
module_exit(cpufreq_gov_nightmare_exit);
