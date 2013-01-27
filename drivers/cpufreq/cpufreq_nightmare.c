/* linux/arch/arm/mach-exynos/stand-hotplug.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS - Nightmare Governor - Dynamic CPU hotpluging
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/percpu.h>
#include <linux/kernel_stat.h>
#include <linux/mutex.h>
#include <linux/ktime.h>
#include <linux/tick.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/suspend.h>
#include <linux/reboot.h>

#include <linux/err.h>
#include <linux/clk.h>
#include <linux/regulator/consumer.h>
#include <mach/cpufreq.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#define EARLYSUSPEND_HOTPLUGLOCK 1

#include <plat/s5p-clock.h>

#if defined(CONFIG_MACH_P10)
#define TRANS_LOAD_H0 5
#define TRANS_LOAD_L1 2
#define TRANS_LOAD_H1 100

#define BOOT_DELAY	30
#define CHECK_DELAY_ON	(.5*HZ * 8)
#define CHECK_DELAY_OFF	(.5*HZ)

#endif

#if defined(CONFIG_MACH_U1) || defined(CONFIG_MACH_PX) || \
	defined(CONFIG_MACH_TRATS)
//#define TRANS_LOAD_H0 50
#define TRANS_LOAD_H0 20
//#define TRANS_LOAD_L1 40
#define TRANS_LOAD_L1 20
#define TRANS_LOAD_H1 100

//#define TRANS_LOAD_H0_SCROFF 100
//#define TRANS_LOAD_L1_SCROFF 100
#define TRANS_LOAD_H0_SCROFF 20
#define TRANS_LOAD_L1_SCROFF 20
#define TRANS_LOAD_H1_SCROFF 100

#define BOOT_DELAY	60
#define CHECK_DELAY_ON	HZ << 1
#define CHECK_DELAY_OFF	HZ >> 1
#define CHECK_RATE 100
#define CHECK_RATE_CPUON 200

#define CPU1_ON_FREQ 300000
#endif

#if defined(CONFIG_MACH_MIDAS) || defined(CONFIG_MACH_SMDK4X12) \
	|| defined(CONFIG_MACH_SLP_PQ)
#define TRANS_LOAD_H0 20
#define TRANS_LOAD_L1 10
#define TRANS_LOAD_H1 35
#define TRANS_LOAD_L2 15
#define TRANS_LOAD_H2 45
#define TRANS_LOAD_L3 20

#define BOOT_DELAY	60

#if defined(CONFIG_MACH_SLP_PQ)
#define CHECK_DELAY_ON	(.3*HZ * 4)
#define CHECK_DELAY_OFF	(.3*HZ)
#else
#define CHECK_DELAY_ON	(.5*HZ * 4)
#define CHECK_DELAY_OFF	(.5*HZ)
#endif
#endif

#define TRANS_RQ 2
#define TRANS_LOAD_RQ 20

#define CPU_OFF 0
#define CPU_ON  1

#define HOTPLUG_UNLOCKED 0
#define HOTPLUG_LOCKED 1
#define PM_HOTPLUG_DEBUG 0
#define NUM_CPUS num_possible_cpus()
#define CPULOAD_TABLE (NR_CPUS + 1)

#define DEF_FREQ_STEP				(35)
#define DEF_FREQ_UP_BRAKE			(5u)
#define DEF_FREQ_STEP_DEC			(20)
/* CPU freq will be increased if measured load > inc_cpu_load;*/
#define INC_CPU_LOAD_AT_MIN_FREQ		(40)
#define DEF_INC_CPU_LOAD			(60)
/* CPU freq will be decreased if measured load < dec_cpu_load;*/
#define DEF_DEC_CPU_LOAD			(60)
#define FREQ_FOR_RESPONSIVENESS			(400000)

#define DBG_PRINT(fmt, ...)\
	if(PM_HOTPLUG_DEBUG)			\
		printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)

#define NIGHTMARE_SECOND_VERSION (1)

static unsigned int n_second_core_on = 1;
static unsigned int n_hotplug_on = 1;
static unsigned int hotplug_sampling_rate = CHECK_DELAY_OFF;

static struct workqueue_struct *dvfs_workqueues;

struct runqueue_data {
	unsigned int nr_run_avg;
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

	queue_delayed_work(rq_data->nr_run_wq, &rq_data->work,hotplug_sampling_rate);
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

	if (hotplug_sampling_rate != 0)
		queue_delayed_work(rq_data->nr_run_wq, &rq_data->work,hotplug_sampling_rate);

	spin_unlock_irqrestore(&rq_data->lock, flags);
}

static unsigned int max_performance;

enum flag{
	HOTPLUG_NOP,
	HOTPLUG_IN,
	HOTPLUG_OUT
};

struct cpu_hotplug_info {
	unsigned long nr_running;
	pid_t tgid;
};

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

struct cpufreq_nightmare_cpuinfo {
	struct cpufreq_policy *cur_policy;
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	unsigned int load;
	int cpu;
	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct delayed_work work;
	struct mutex timer_mutex;
};
static DEFINE_PER_CPU(struct cpufreq_nightmare_cpuinfo, od_cpu_dbs_info);
static unsigned int dbs_enable;	/* number of CPUs using this policy */
static bool screen_off;

/* mutex can be used since do_dbs_timer does not run in
   timer(softirq) context but in process context */
static DEFINE_MUTEX(dbs_mutex);
/* Second core values by tegrak */

#if (NR_CPUS > 2)
static unsigned int load_each[4];
#else
static unsigned int load_each[2];
#endif
static unsigned int freq_min = -1UL;

static struct dbs_tuners {
	/* nightmare tuners */
	unsigned int freq_step;
	unsigned int freq_up_brake;
	unsigned int freq_step_dec;
#ifdef CONFIG_HAS_EARLYSUSPEND
	int early_suspend;
#endif
	unsigned int inc_cpu_load_at_min_freq;
	unsigned int inc_cpu_load;
	unsigned int dec_cpu_load;
	unsigned int freq_for_responsiveness;
	unsigned int check_rate;
	unsigned int check_rate_cpuon;
	unsigned int check_rate_scroff;
	unsigned int freq_cpu1on;
	unsigned int user_lock;
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
	char *hotplug_on_s;
	char *second_core_on_s;

} dbs_tuners_ins = {
	.freq_step = DEF_FREQ_STEP,
	.freq_up_brake = DEF_FREQ_UP_BRAKE,
	.freq_step_dec = DEF_FREQ_STEP_DEC,
#ifdef CONFIG_HAS_EARLYSUSPEND
	.early_suspend = -1,
#endif
	.inc_cpu_load_at_min_freq = INC_CPU_LOAD_AT_MIN_FREQ,
	.inc_cpu_load = DEF_INC_CPU_LOAD,
	.dec_cpu_load = DEF_DEC_CPU_LOAD,
	.freq_for_responsiveness = FREQ_FOR_RESPONSIVENESS,
	.check_rate = CHECK_RATE,
	.check_rate_cpuon = CHECK_RATE_CPUON,
	.check_rate_scroff = CHECK_DELAY_ON << 2,
	.freq_cpu1on = CPU1_ON_FREQ,
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
	.hotplug_on_s = "on",
	.second_core_on_s = "off",
};

/************************** sysfs interface ************************/

#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(freq_step, freq_step);
show_one(freq_step_dec, freq_step_dec);
show_one(inc_cpu_load_at_min_freq, inc_cpu_load_at_min_freq);
show_one(inc_cpu_load, inc_cpu_load);
show_one(dec_cpu_load, dec_cpu_load);
show_one(freq_for_responsiveness, freq_for_responsiveness);
show_one(freq_up_brake, freq_up_brake);
show_one(check_rate, check_rate);
show_one(check_rate_cpuon, check_rate_cpuon);
show_one(check_rate_scroff, check_rate_scroff);
show_one(freq_cpu1on, freq_cpu1on);
show_one(user_lock, user_lock);
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

static ssize_t show_version(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", NIGHTMARE_SECOND_VERSION);
}

static ssize_t show_author(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "Alucard24@XDA\n");
}

static ssize_t show_hotplug_on_s(struct kobject *kobj,
				     struct attribute *attr, char *buf) {
	return sprintf(buf, "%s\n", (n_hotplug_on) ? ("on") : ("off"));
}

static ssize_t show_second_core_on_s(struct kobject *kobj,
				     struct attribute *attr, char *buf) {
	return sprintf(buf, "%s\n", (n_second_core_on) ? ("on") : ("off"));
}

/* freq_step */
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

/* freq_step_dec */
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

/* inc_cpu_load_at_min_freq */
static ssize_t store_inc_cpu_load_at_min_freq(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);

	if (ret != 1 || input > 100u) {
		return -EINVAL;
	}
	dbs_tuners_ins.inc_cpu_load_at_min_freq = min(input,dbs_tuners_ins.inc_cpu_load);
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

/* freq_for_responsiveness */
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

/* check_rate */
static ssize_t store_check_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.check_rate = input;
	return count;
}
/* check_rate_cpuon */
static ssize_t store_check_rate_cpuon(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.check_rate_cpuon = input;
	return count;
}
/* check_rate_scroff */
static ssize_t store_check_rate_scroff(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.check_rate_scroff = input;
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
/* user_lock */
static ssize_t store_user_lock(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	dbs_tuners_ins.user_lock = input;
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

static ssize_t store_hotplug_on_s(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	struct cpufreq_nightmare_cpuinfo *dbs_info;
	unsigned int user_lock = 0;
	mutex_lock(&dbs_mutex);
	user_lock = dbs_tuners_ins.user_lock;

	if (user_lock) {
		goto finish;
	}
	/* do turn_on/off cpus */
	dbs_info = &per_cpu(od_cpu_dbs_info, 0); /* from CPU0 */

	if (!n_hotplug_on && strcmp(buf, "on\n") == 0) {
		n_hotplug_on = 1;
		// restart worker thread.
		hotplug_sampling_rate = CHECK_DELAY_ON;
		queue_delayed_work_on(0, dvfs_workqueues, &dbs_info->work, hotplug_sampling_rate);
		printk("second_core: hotplug is on!\n");
	}
	else if (n_hotplug_on && strcmp(buf, "off\n") == 0) {
		n_hotplug_on = 0;
		n_second_core_on = 1;
		if (cpu_online(1) == 0) {
			cpu_up(1);
		}
		printk("second_core: hotplug is off!\n");
	}
	
finish:
	mutex_unlock(&dbs_mutex);
	return count;
}

static ssize_t store_second_core_on_s(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int user_lock = 0;	

	mutex_lock(&dbs_mutex);
	user_lock = dbs_tuners_ins.user_lock;
	
	if (n_hotplug_on || user_lock) {
		goto finish;
	}
	
	if (!n_second_core_on && strcmp(buf, "on\n") == 0) {
		n_second_core_on = 1;
		if (cpu_online(1) == 0) {
			cpu_up(1);
		}
		printk("second_core: 2nd core is always on!\n");
	}
	else if (n_second_core_on && strcmp(buf, "off\n") == 0) {
		n_second_core_on = 0;
		if (cpu_online(1) == 1) {
			cpu_down(1);
		}
		printk("second_core: 2nd core is always off!\n");
	}
	
finish:
	mutex_unlock(&dbs_mutex);
	return count;
}

define_one_global_rw(freq_step);
define_one_global_rw(freq_up_brake);
define_one_global_rw(freq_step_dec);
define_one_global_rw(inc_cpu_load_at_min_freq);
define_one_global_rw(inc_cpu_load);
define_one_global_rw(dec_cpu_load);
define_one_global_rw(freq_for_responsiveness);
define_one_global_rw(check_rate);
define_one_global_rw(check_rate_cpuon);
define_one_global_rw(check_rate_scroff);
define_one_global_rw(freq_cpu1on);
define_one_global_rw(user_lock);
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
define_one_global_ro(version);
define_one_global_ro(author);
define_one_global_rw(hotplug_on_s);
define_one_global_rw(second_core_on_s);


static struct attribute *dbs_attributes[] = {
	&freq_step.attr,
	&freq_up_brake.attr,
	&freq_step_dec.attr,
	&inc_cpu_load_at_min_freq.attr,
	&inc_cpu_load.attr,
	&dec_cpu_load.attr,
	&freq_for_responsiveness.attr,
	&check_rate.attr,
	&check_rate_cpuon.attr,
	&check_rate_scroff.attr,
	&freq_cpu1on.attr,
	&user_lock.attr,
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
	&hotplug_on_s.attr,
	&second_core_on_s.attr,
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
#if defined(CONFIG_MACH_P10)
	return ((nr_online_cpu > 1) &&
		(avg_load < threshold_up &&
		cur_freq < dbs_tuners_ins.freq_cpu1on));
#else
	return ((nr_online_cpu > 1) &&
		(avg_load < threshold_up ||
		cur_freq < dbs_tuners_ins.freq_cpu1on));
#endif
}

static inline enum flag
standalone_hotplug(unsigned int load, unsigned long nr_rq_min, unsigned int cpu_rq_min)
{
	unsigned int cur_freq;
	unsigned int nr_online_cpu;
	unsigned int avg_load;
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

	static void __iomem *clk_fimc;
	unsigned char fimc_stat;

	cur_freq = clk_get_rate(clk_get(NULL, "armclk")) / 1000;

	nr_online_cpu = num_online_cpus();

	avg_load = (unsigned int)((cur_freq * load) / max_performance);

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
#if defined(CONFIG_MACH_P10)
#else
	} else if (nr_online_cpu > 1 && nr_rq_min < dbs_tuners_ins.trans_rq) {

		struct cpufreq_nightmare_cpuinfo *tmp_info;

		tmp_info = &per_cpu(od_cpu_dbs_info, cpu_rq_min);

		/*If CPU(cpu_rq_min) load is less than trans_load_rq, hotplug-out*/
		if (tmp_info->load < dbs_tuners_ins.trans_load_rq)
			return HOTPLUG_OUT;
#endif
	}

	return HOTPLUG_NOP;
}

static void dbs_check_cpu(struct cpufreq_nightmare_cpuinfo *this_dbs_info)
{
	struct cpu_hotplug_info tmp_hotplug_info[4];
	struct cpufreq_policy *policy;
	int i;
	unsigned int load = 0;
	unsigned int cpu_rq_min=0;
	unsigned long nr_rq_min = -1UL;
	unsigned int select_off_cpu = 0;

	enum flag flag_hotplug;

	policy = this_dbs_info->cur_policy;

	// exit if we turned off dynamic hotplug by tegrak
	// cancel the timer
	if (!n_hotplug_on) {
		if (!n_second_core_on && cpu_online(1) == 1)
			cpu_down(1);
	}

	for_each_cpu(i,policy->cpus) {
		struct cpufreq_nightmare_cpuinfo *tmp_info;
		cputime64_t cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;

		tmp_info = &per_cpu(od_cpu_dbs_info, i);

		// RESET LOAD_EACH
		load_each[i] = -1;

		cur_idle_time = get_cpu_idle_time_us(i, &cur_wall_time);

		idle_time = (unsigned int)cputime64_sub(cur_idle_time,
							tmp_info->prev_cpu_idle);
		tmp_info->prev_cpu_idle = cur_idle_time;

		wall_time = (unsigned int)cputime64_sub(cur_wall_time,
							tmp_info->prev_cpu_wall);
		tmp_info->prev_cpu_wall = cur_wall_time;

		if (wall_time < idle_time)
			continue;

#ifdef CONFIG_TARGET_LOCALE_P2TMO_TEMP
		/*For once Divide-by-Zero issue*/
		if (wall_time == 0)
			wall_time++;
#endif
		tmp_info->load = 100 * (wall_time - idle_time) / wall_time;

		load += tmp_info->load;

		// GET CORE LOAD used in dbs_check_frequency
		load_each[i] = tmp_info->load;

		/*find minimum runqueue length*/
		tmp_hotplug_info[i].nr_running = get_cpu_nr_running(i);

		if (i && nr_rq_min > tmp_hotplug_info[i].nr_running) {
			nr_rq_min = tmp_hotplug_info[i].nr_running;
			cpu_rq_min = i;
		}
	}

	if (dbs_tuners_ins.user_lock == 1 || !n_hotplug_on)
		return;

	for (i = NUM_CPUS - 1; i > 0; --i) {
		if (cpu_online(i) == 0) {
			select_off_cpu = i;
			break;
		}
	}

	/*standallone hotplug*/
	flag_hotplug = standalone_hotplug(load, nr_rq_min, cpu_rq_min);

	/*do not ever hotplug out CPU 0*/
	if((cpu_rq_min == 0) && (flag_hotplug == HOTPLUG_OUT))
		return;

	/*cpu hotplug*/
	if (flag_hotplug == HOTPLUG_IN && cpu_online(select_off_cpu) == CPU_OFF) {
		DBG_PRINT("cpu%d turning on!\n", select_off_cpu);
		cpu_up(select_off_cpu);
		DBG_PRINT("cpu%d on\n", select_off_cpu);
		hotplug_sampling_rate = dbs_tuners_ins.check_rate_cpuon;
	} else if (flag_hotplug == HOTPLUG_OUT && cpu_online(cpu_rq_min) == CPU_ON) {
		DBG_PRINT("cpu%d turnning off!\n", cpu_rq_min);
		cpu_down(cpu_rq_min);
		DBG_PRINT("cpu%d off!\n", cpu_rq_min);
		if(!screen_off) hotplug_sampling_rate = dbs_tuners_ins.check_rate;
		else hotplug_sampling_rate = dbs_tuners_ins.check_rate_scroff;
	} 

	return;
}

static void dbs_check_frequency(struct cpufreq_nightmare_cpuinfo *this_dbs_info)
{
	int j;
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
	struct cpufreq_policy *policy;

	for_each_online_cpu(j) {
		struct cpufreq_policy *policy;
		int load = 0;

		// GET LOAD_EACH
		load = load_each[j];

		policy = cpufreq_cpu_get(j);
		if (!policy)
			continue;

		// CPUs Online Scale Frequency
		if (policy->cur < dbs_tuners_ins.freq_for_responsiveness)
			inc_cpu_load = dbs_tuners_ins.inc_cpu_load_at_min_freq;
		else
			inc_cpu_load = dbs_tuners_ins.inc_cpu_load;

		// Check for frequency increase or for frequency decrease
		if (load >= inc_cpu_load) {

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
		
			if (freq_up != policy->cur && freq_up <= policy->max) {				
				__cpufreq_driver_target(policy, freq_up, CPUFREQ_RELATION_L);
			}

		} else if (load <	 dec_cpu_load && load > -1) {

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

	mutex_lock(&dbs_mutex);
	mutex_lock(&dbs_info->timer_mutex);

	// CHECK FOR HOTPLUGING
	dbs_check_cpu(dbs_info);
	// CHECK TO INCREASE/DECREASE FREQUENCY.....
	dbs_check_frequency(dbs_info);

	queue_delayed_work_on(0, dvfs_workqueues, &dbs_info->work, hotplug_sampling_rate);

	mutex_unlock(&dbs_info->timer_mutex);
	mutex_unlock(&dbs_mutex);
}

static inline void dbs_timer_init(struct cpufreq_nightmare_cpuinfo *dbs_info)
{
	INIT_DEFERRABLE_WORK(&dbs_info->work, do_dbs_timer);
	queue_delayed_work_on(0, dvfs_workqueues, &dbs_info->work, BOOT_DELAY * HZ);
}

static inline void do_dbs_timer_exit(struct cpufreq_nightmare_cpuinfo *dbs_info)
{
	cancel_delayed_work_sync(&dbs_info->work);
}

static int pm_notifier_call(struct notifier_block *this,
					     unsigned long event, void *ptr)
{
	static unsigned user_lock_saved;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		mutex_lock(&dbs_mutex);
		user_lock_saved = dbs_tuners_ins.user_lock;
		dbs_tuners_ins.user_lock = 1;
		pr_info("%s: saving pm_hotplug lock %x\n",
			__func__, user_lock_saved);
		mutex_unlock(&dbs_mutex);
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		mutex_lock(&dbs_mutex);
		pr_info("%s: restoring pm_hotplug lock %x\n",
			__func__, user_lock_saved);
		dbs_tuners_ins.user_lock = user_lock_saved;
		mutex_unlock(&dbs_mutex);
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
	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.user_lock = 1;
	mutex_unlock(&dbs_mutex);

	return NOTIFY_DONE;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend early_suspend;
static void cpufreq_nightmare_early_suspend(struct early_suspend *handler)
{
	unsigned int check_rate_scroff = dbs_tuners_ins.check_rate_scroff;
#if EARLYSUSPEND_HOTPLUGLOCK
	mutex_lock(&dbs_mutex);
	dbs_tuners_ins.early_suspend = dbs_tuners_ins.user_lock;
#endif
	screen_off = true;
	//Hotplug out all extra CPUs
	while(num_online_cpus() > 1)
	  cpu_down(num_online_cpus()-1);

	hotplug_sampling_rate = check_rate_scroff;

#if EARLYSUSPEND_HOTPLUGLOCK
	stop_rq_work();
	mutex_unlock(&dbs_mutex);
#endif
}

static void cpufreq_nightmare_late_resume(struct early_suspend *handler)
{
	unsigned int check_rate = dbs_tuners_ins.check_rate;
	//printk(KERN_INFO "pm-hotplug: enable cpu auto-hotplug\n");
#if EARLYSUSPEND_HOTPLUGLOCK
	mutex_lock(&dbs_mutex);
#endif
	screen_off = false;
	dbs_tuners_ins.early_suspend = -1;
	hotplug_sampling_rate = check_rate;

#if EARLYSUSPEND_HOTPLUGLOCK
	start_rq_work();
	mutex_unlock(&dbs_mutex);
#endif
}
#endif

static struct notifier_block reboot_notifier = {
	.notifier_call = reboot_notifier_call,
};

static int cpufreq_governor_nightmare(struct cpufreq_policy *policy,
				unsigned int event)
{
	unsigned int cpu = policy->cpu;
	struct cpufreq_nightmare_cpuinfo *this_dbs_info;
	int ret;
	unsigned int i;
	unsigned int freq;
	unsigned int freq_max = 0;
	struct cpufreq_frequency_table *table;

	this_dbs_info = &per_cpu(od_cpu_dbs_info, cpu);

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(0)) || (!policy->cur))
			return -EINVAL;

		start_rq_work();

		mutex_lock(&dbs_mutex);

		dbs_enable++;

		for_each_cpu(i,policy->cpus) {
			struct cpufreq_nightmare_cpuinfo *tmp_info;
			tmp_info = &per_cpu(od_cpu_dbs_info, i);
			tmp_info->cur_policy = policy;
		}
		
		this_dbs_info->cpu = cpu;

#ifdef CONFIG_CPU_FREQ
		table = cpufreq_frequency_get_table(0);

		for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
			freq = table[i].frequency;

			if (freq != CPUFREQ_ENTRY_INVALID && freq > freq_max)
				freq_max = freq;
			else if (freq != CPUFREQ_ENTRY_INVALID && freq_min > freq)
				freq_min = freq;
		}
		/*get max frequence*/
		max_performance = freq_max * NUM_CPUS;
#else
		max_performance = clk_get_rate(clk_get(NULL, "armclk")) / 1000 * NUM_CPUS;
		freq_min = clk_get_rate(clk_get(NULL, "armclk")) / 1000;
#endif

		if (dbs_enable == 1) {
			ret = sysfs_create_group(cpufreq_global_kobject,&dbs_attr_group);

			if (ret) {
				mutex_unlock(&dbs_mutex);
				return ret;
			}

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
		do_dbs_timer_exit(this_dbs_info);

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

		if (policy->max < this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
						policy->max,CPUFREQ_RELATION_H);
		else if (policy->min > this_dbs_info->cur_policy->cur)
			__cpufreq_driver_target(this_dbs_info->cur_policy,
						policy->min,CPUFREQ_RELATION_L);
		
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

	dvfs_workqueues = create_workqueue("Nightmare dynamic hotplug");
	if (!dvfs_workqueues) {
		pr_err("%s Creation of Nightmare hotplug work failed\n", __func__);
		ret = -ENOMEM;
		goto err_rq_data;
	}

	printk(KERN_INFO "cpufreq_gov_nightmare_init: %d\n", ret); 

	ret = cpufreq_register_governor(&cpufreq_gov_nightmare);

	if (ret) {
		printk(KERN_ERR "Registering Nightmare governor failed at(%d)\n", __LINE__);
		goto err_reg;
	}

	#ifdef CONFIG_HAS_EARLYSUSPEND
		early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB;
		early_suspend.suspend = cpufreq_nightmare_early_suspend;
		early_suspend.resume = cpufreq_nightmare_late_resume;
	#endif

	return ret;

err_reg:
	destroy_workqueue(dvfs_workqueues);	
err_rq_data:
	kfree(rq_data);
	pr_debug("%s: failed initialization\n", __func__);
	return ret;
}

static void __exit cpufreq_gov_nightmare_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_nightmare);
	destroy_workqueue(dvfs_workqueues);
	kfree(rq_data);
}

MODULE_AUTHOR("Alucard24@XDA <dmbaoh2@gmail.com>");
MODULE_DESCRIPTION("'cpufreq_nightmare' - A dynamic cpufreq/cpuhotplug governor");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_NIGHTMARE
fs_initcall(cpufreq_gov_nightmare_init);
#else
module_init(cpufreq_gov_nightmare_init);
#endif
module_exit(cpufreq_gov_nightmare_exit);

