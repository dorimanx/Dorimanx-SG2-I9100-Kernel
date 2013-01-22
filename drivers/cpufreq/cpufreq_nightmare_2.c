/* linux/arch/arm/mach-exynos/stand-hotplug.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * EXYNOS - Dynamic CPU hotpluging
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/serial_core.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/cpu.h>
#include <linux/percpu.h>
#include <linux/ktime.h>
#include <linux/tick.h>
#include <linux/kernel_stat.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/reboot.h>
#include <linux/gpio.h>
#include <linux/cpufreq.h>
#include <linux/device.h>       //for second_core by tegrak
#include <linux/miscdevice.h>   //for second_core by tegrak
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#include <plat/map-base.h>
#include <plat/gpio-cfg.h>
#include <plat/s5p-clock.h>
#include <plat/clock.h>

#include <mach/regs-gpio.h>
#include <mach/regs-irq.h>

#define EARLYSUSPEND_HOTPLUGLOCK 1

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
#define TRANS_LOAD_H0 50
#define TRANS_LOAD_L1 40
#define TRANS_LOAD_H1 100

#define TRANS_LOAD_H0_SCROFF 100
#define TRANS_LOAD_L1_SCROFF 100
#define TRANS_LOAD_H1_SCROFF 100

#define BOOT_DELAY	60
#define CHECK_DELAY_ON	HZ << 1
#define CHECK_DELAY_OFF	HZ >> 1

#define CPU1_ON_FREQ 800000
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

#define DEF_SAMPLING_UP_FACTOR			(1)
#define MAX_SAMPLING_UP_FACTOR		(100000)
#define DEF_SAMPLING_DOWN_FACTOR		(1)
#define MAX_SAMPLING_DOWN_FACTOR		(100000)
#define DEF_FREQ_STEP_DEC		(5)

#define DEF_FREQ_STEP				(30)

#define FREQ_FOR_RESPONSIVENESS			(400000)
#define FIRST_CORE_FREQ_LIMIT			(1200000)
#define SECOND_CORE_FREQ_LIMIT			(1200000)

/* CPU freq will be increased if measured load > inc_cpu_load;*/
#define DEF_INC_CPU_LOAD (80)
#define INC_CPU_LOAD_AT_MIN_FREQ		(40)
/* CPU freq will be decreased if measured load < dec_cpu_load;*/
#define DEF_DEC_CPU_LOAD (60)
#define DEF_FREQ_UP_BRAKE				(5u)


#define DBG_PRINT(fmt, ...)\
	if(PM_HOTPLUG_DEBUG)			\
		printk(KERN_INFO pr_fmt(fmt), ##__VA_ARGS__)

static struct workqueue_struct *hotplug_wq;
static struct delayed_work hotplug_work;

static unsigned int max_performance;

enum flag{
	HOTPLUG_NOP,
	HOTPLUG_IN,
	HOTPLUG_OUT
};

struct cpu_time_info {
	cputime64_t prev_cpu_idle;
	cputime64_t prev_cpu_wall;
	unsigned int load;
};

struct cpu_hotplug_info {
	unsigned long nr_running;
	pid_t tgid;
};

static void hotplug_timer(struct work_struct *work);
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

static DEFINE_PER_CPU(struct cpu_time_info, hotplug_cpu_time);

static bool standhotplug_enabled = true;
static bool screen_off;

/* mutex can be used since hotplug_timer does not run in
   timer(softirq) context but in process context */
static DEFINE_MUTEX(hotplug_lock);
/* Second core values by tegrak */
#define SECOND_CORE_VERSION (1)
int second_core_on = 1;
int hotplug_on = 1;

static struct dbs_tuners {
	unsigned int sampling_up_factor;
	unsigned int sampling_down_factor;
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
	unsigned int first_core_freq_limit;
	unsigned int second_core_freq_limit;
	unsigned int freq_min;
	unsigned int hotpluging_rate;
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
	char *hotplug_on;
	char *second_core_on;

} dbs_tuners_ins = {
	.sampling_up_factor = DEF_SAMPLING_UP_FACTOR,
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
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
	.first_core_freq_limit = FIRST_CORE_FREQ_LIMIT,
	.second_core_freq_limit = SECOND_CORE_FREQ_LIMIT,
	.freq_min = -1UL,
	.hotpluging_rate = CHECK_DELAY_OFF,
	.check_rate = CHECK_DELAY_OFF,
	.check_rate_cpuon = CHECK_DELAY_ON,
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
	.hotplug_on = "on",
	.second_core_on = "off",
};

bool hotplug_out_chk(unsigned int nr_online_cpu, unsigned int threshold_up,
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

	if (hotplug_out_chk(nr_online_cpu, (screen_off ? threshold_scroff[nr_online_cpu-1][0] : threshold[nr_online_cpu - 1][0] ),
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

		struct cpu_time_info *tmp_info;

		tmp_info = &per_cpu(hotplug_cpu_time, cpu_rq_min);
		/*If CPU(cpu_rq_min) load is less than trans_load_rq, hotplug-out*/
		if (tmp_info->load < dbs_tuners_ins.trans_load_rq)
			return HOTPLUG_OUT;
#endif
	}

	return HOTPLUG_NOP;
}

static void hotplug_timer(struct work_struct *work)
{
	struct cpu_hotplug_info tmp_hotplug_info[4];
	int i;
	unsigned int load = 0;
	unsigned int cpu_rq_min=0;
	unsigned long nr_rq_min = -1UL;
	unsigned int select_off_cpu = 0;
	enum flag flag_hotplug;

	mutex_lock(&hotplug_lock);

	if(!standhotplug_enabled) {
		printk(KERN_INFO "pm-hotplug: disable cpu auto-hotplug\n");
		goto off_hotplug;
	}

	// exit if we turned off dynamic hotplug by tegrak
	// cancel the timer
	if (!hotplug_on) {
		if (!second_core_on && cpu_online(1) == 1)
			cpu_down(1);
		goto off_hotplug;
	}

	if (dbs_tuners_ins.user_lock == 1)
		goto no_hotplug;

	for_each_online_cpu(i) {
		struct cpu_time_info *tmp_info;
		cputime64_t cur_wall_time, cur_idle_time;
		unsigned int idle_time, wall_time;

		tmp_info = &per_cpu(hotplug_cpu_time, i);

		cur_idle_time = get_cpu_idle_time_us(i, &cur_wall_time);

		idle_time = (unsigned int)cputime64_sub(cur_idle_time,
							tmp_info->prev_cpu_idle);
		tmp_info->prev_cpu_idle = cur_idle_time;

		wall_time = (unsigned int)cputime64_sub(cur_wall_time,
							tmp_info->prev_cpu_wall);
		tmp_info->prev_cpu_wall = cur_wall_time;

		if (wall_time < idle_time)
			goto no_hotplug;

#ifdef CONFIG_TARGET_LOCALE_P2TMO_TEMP
		/*For once Divide-by-Zero issue*/
		if (wall_time == 0)
			wall_time++;
#endif
		tmp_info->load = 100 * (wall_time - idle_time) / wall_time;

		load += tmp_info->load;
		/*find minimum runqueue length*/
		tmp_hotplug_info[i].nr_running = get_cpu_nr_running(i);

		if (i && nr_rq_min > tmp_hotplug_info[i].nr_running) {
			nr_rq_min = tmp_hotplug_info[i].nr_running;

			cpu_rq_min = i;
		}
	}

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
		goto no_hotplug;

	/*cpu hotplug*/
	if (flag_hotplug == HOTPLUG_IN && cpu_online(select_off_cpu) == CPU_OFF) {
		DBG_PRINT("cpu%d turning on!\n", select_off_cpu);
		cpu_up(select_off_cpu);
		DBG_PRINT("cpu%d on\n", select_off_cpu);
		dbs_tuners_ins.hotpluging_rate = dbs_tuners_ins.check_rate_cpuon;
	} else if (flag_hotplug == HOTPLUG_OUT && cpu_online(cpu_rq_min) == CPU_ON) {
		DBG_PRINT("cpu%d turnning off!\n", cpu_rq_min);
		cpu_down(cpu_rq_min);
		DBG_PRINT("cpu%d off!\n", cpu_rq_min);
		if(!screen_off) dbs_tuners_ins.hotpluging_rate = dbs_tuners_ins.check_rate;
		else dbs_tuners_ins.hotpluging_rate = dbs_tuners_ins.check_rate_scroff;
	} 

no_hotplug:
	//printk("hotplug_timer done.\n");

	queue_delayed_work_on(0, hotplug_wq, &hotplug_work, dbs_tuners_ins.hotpluging_rate);
off_hotplug:

	mutex_unlock(&hotplug_lock);
}


//static inline void hotplug_timer_init(struct cpufreq_nightmare_cpuinfo *dbs_info)
static inline void hotplug_timer_init(struct cpufreq_policy *policy)
{

	//hotplug_wq = create_workqueue("dynamic hotplug");
	hotplug_wq = alloc_workqueue("dynamic hotplug", 0, 0);
	if (!hotplug_wq) {
		printk(KERN_ERR "Creation of hotplug work failed\n");
		//return -EFAULT;
		return;
	}
	INIT_DELAYED_WORK(&hotplug_work, hotplug_timer);
	queue_delayed_work_on(0, hotplug_wq, &hotplug_work, BOOT_DELAY * HZ);

	/* We want all CPUs to do sampling nearly on same jiffy */

	/*INIT_DEFERRABLE_WORK(&dbs_info->work, do_dbs_timer);
	INIT_WORK(&dbs_info->up_work, cpu_up_work);
	INIT_WORK(&dbs_info->down_work, cpu_down_work);*/

}

//static inline void hotplug_timer_exit(struct cpufreq_nightmare_cpuinfo *dbs_info)
static inline void hotplug_timer_exit(struct cpufreq_policy *policy)
{
	cancel_delayed_work_sync(&hotplug_work);
}

static int nightmare_pm_hotplug_notifier_event(struct notifier_block *this,
					     unsigned long event, void *ptr)
{
	static unsigned user_lock_saved;

	switch (event) {
	case PM_SUSPEND_PREPARE:
		mutex_lock(&hotplug_lock);
		user_lock_saved = dbs_tuners_ins.user_lock;
		dbs_tuners_ins.user_lock = 1;
		pr_info("%s: saving pm_hotplug lock %x\n",
			__func__, user_lock_saved);
		mutex_unlock(&hotplug_lock);
		return NOTIFY_OK;
	case PM_POST_RESTORE:
	case PM_POST_SUSPEND:
		mutex_lock(&hotplug_lock);
		pr_info("%s: restoring pm_hotplug lock %x\n",
			__func__, user_lock_saved);
		dbs_tuners_ins.user_lock = user_lock_saved;
		mutex_unlock(&hotplug_lock);
		return NOTIFY_OK;
	}
	return NOTIFY_DONE;
}

static struct notifier_block nightmare_pm_hotplug_notifier = {
	.notifier_call = nightmare_pm_hotplug_notifier_event,
};

static void hotplug_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&hotplug_lock);
	screen_off = true;
	//Hotplug out all extra CPUs
	while(num_online_cpus() > 1)
	  cpu_down(num_online_cpus()-1);
	dbs_tuners_ins.hotpluging_rate = dbs_tuners_ins.check_rate_scroff;
	mutex_unlock(&hotplug_lock);
}

static void hotplug_late_resume(struct early_suspend *handler)
{
	printk(KERN_INFO "pm-hotplug: enable cpu auto-hotplug\n");

	mutex_lock(&hotplug_lock);
	screen_off = false;
	dbs_tuners_ins.hotpluging_rate = dbs_tuners_ins.check_rate;
	queue_delayed_work_on(0, hotplug_wq, &hotplug_work, dbs_tuners_ins.hotpluging_rate);
	mutex_unlock(&hotplug_lock);
}

static struct early_suspend hotplug_early_suspend_notifier = {
	.suspend = hotplug_early_suspend,
	.resume = hotplug_late_resume,
	.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN,
};

static int hotplug_reboot_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	mutex_lock(&hotplug_lock);
	pr_err("%s: disabling pm hotplug\n", __func__);
	dbs_tuners_ins.user_lock = 1;
	mutex_unlock(&hotplug_lock);

	return NOTIFY_DONE;
}

static struct notifier_block hotplug_reboot_notifier = {
	.notifier_call = hotplug_reboot_notifier_call,
};

/****************************************
 * DEVICE ATTRIBUTES FUNCTION by tegrak
****************************************/
static ssize_t show_version(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", SECOND_CORE_VERSION);
}

static ssize_t show_author(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "Alucard24@XDA\n");
}

static ssize_t show_hotplug_on(struct kobject *kobj,
				     struct attribute *attr, char *buf) {
	return sprintf(buf, "%s\n", (hotplug_on) ? ("on") : ("off"));
}

/****************************************
 * second_core attributes function by tegrak
 ****************************************/

static ssize_t store_hotplug_on(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int hotpluging_rate = dbs_tuners_ins.hotpluging_rate;
	unsigned int user_lock = dbs_tuners_ins.user_lock;
	
	mutex_lock(&hotplug_lock);
	if (user_lock) {
		goto finish;
	}
	
	if (!hotplug_on && strcmp(buf, "on\n") == 0) {
		hotplug_on = 1;
		// restart worker thread.
		//hotpluging_rate = CHECK_DELAY_ON;
		queue_delayed_work_on(0, hotplug_wq, &hotplug_work, hotpluging_rate);
		printk("second_core: hotplug is on!\n");
	}
	else if (hotplug_on && strcmp(buf, "off\n") == 0) {
		hotplug_on = 0;
		second_core_on = 1;
		if (cpu_online(1) == 0) {
			cpu_up(1);
		}
		printk("second_core: hotplug is off!\n");
	}
	
finish:
	mutex_unlock(&hotplug_lock);
	return count;
}

static ssize_t show_second_core_on(struct kobject *kobj,
				     struct attribute *attr, char *buf) {
	return sprintf(buf, "%s\n", (second_core_on) ? ("on") : ("off"));
}

static ssize_t store_second_core_on(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int user_lock = dbs_tuners_ins.user_lock;

	mutex_lock(&hotplug_lock);
	
	if (hotplug_on || user_lock) {
		goto finish;
	}
	
	if (!second_core_on && strcmp(buf, "on\n") == 0) {
		second_core_on = 1;
		if (cpu_online(1) == 0) {
			cpu_up(1);
		}
		printk("second_core: 2nd core is always on!\n");
	}
	else if (second_core_on && strcmp(buf, "off\n") == 0) {
		second_core_on = 0;
		if (cpu_online(1) == 1) {
			cpu_down(1);
		}
		printk("second_core: 2nd core is always off!\n");
	}
	
finish:
	mutex_unlock(&hotplug_lock);
	return count;
}

define_one_global_ro(version);
define_one_global_ro(author);
//define_one_global_rw(hotplug_on);
//define_one_global_rw(second_core_on);

/* cpufreq_nightmare Governor Tunables */
#define show_one(file_name, object)					\
static ssize_t show_##file_name						\
(struct kobject *kobj, struct attribute *attr, char *buf)		\
{									\
	return sprintf(buf, "%u\n", dbs_tuners_ins.object);		\
}
show_one(sampling_up_factor, sampling_up_factor);
show_one(sampling_down_factor, sampling_down_factor);
show_one(freq_step, freq_step);
show_one(freq_step_dec, freq_step_dec);
show_one(inc_cpu_load_at_min_freq, inc_cpu_load_at_min_freq);
show_one(inc_cpu_load, inc_cpu_load);
show_one(dec_cpu_load, dec_cpu_load);
show_one(freq_for_responsiveness, freq_for_responsiveness);
show_one(freq_up_brake, freq_up_brake);
show_one(first_core_freq_limit, first_core_freq_limit);
show_one(second_core_freq_limit, second_core_freq_limit);
show_one(freq_min, freq_min);
show_one(hotpluging_rate, hotpluging_rate);
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

/* sampling_down_factor */
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

	if (ret != 1 || input > 100) {
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

/* first_core_freq_limit */
static ssize_t store_first_core_freq_limit(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;

	if (input > 1600000)	
		dbs_tuners_ins.first_core_freq_limit = 1600000;
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
	if (input > 1600000)	
		dbs_tuners_ins.second_core_freq_limit = 1600000;
	else
		dbs_tuners_ins.second_core_freq_limit = input;
	return count;
}

/* freq_min */
static ssize_t store_freq_min(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.freq_min = input;
	return count;
}
/* hotpluging_rate */
static ssize_t store_hotpluging_rate(struct kobject *a, struct attribute *b,
				   const char *buf, size_t count)
{
	unsigned int input;
	int ret;
	ret = sscanf(buf, "%u", &input);
	if (ret != 1)
		return -EINVAL;
	dbs_tuners_ins.hotpluging_rate = input;
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

define_one_global_rw(sampling_up_factor);
define_one_global_rw(sampling_down_factor);
define_one_global_rw(freq_step);
define_one_global_rw(freq_up_brake);
define_one_global_rw(freq_step_dec);
define_one_global_rw(inc_cpu_load_at_min_freq);
define_one_global_rw(inc_cpu_load);
define_one_global_rw(dec_cpu_load);
define_one_global_rw(freq_for_responsiveness);
define_one_global_rw(first_core_freq_limit);
define_one_global_rw(second_core_freq_limit);
define_one_global_rw(freq_min);
define_one_global_rw(hotpluging_rate);
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

static struct attribute *dbs_attributes[] = {
	&sampling_up_factor.attr,
	&sampling_down_factor.attr,
	&freq_step.attr,
	&freq_up_brake.attr,
	&freq_step_dec.attr,
	&inc_cpu_load_at_min_freq.attr,
	&inc_cpu_load.attr,
	&dec_cpu_load.attr,
	&freq_for_responsiveness.attr,
	&first_core_freq_limit.attr,
	&second_core_freq_limit.attr,
	&freq_min.attr,
	&hotpluging_rate.attr,
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
/*	&hotplug_on.attr,
	&second_core_on.attr,*/
	NULL
};

static struct attribute_group dbs_attr_group = {
	.attrs = dbs_attributes,
	.name = "nightmare",
};

static struct miscdevice second_core_device = {
		.minor = MISC_DYNAMIC_MINOR,
		.name = "second_core",
};


/************************** sysfs end ************************/


static int cpufreq_gov_nightmare(struct cpufreq_policy *policy,
				unsigned int event)
{
	int ret;
	unsigned int i;
	unsigned int freq;
	unsigned int freq_max = 0;
	struct cpufreq_frequency_table *table;

	policy->shared_type = CPUFREQ_SHARED_TYPE_ANY;
	cpumask_setall(policy->cpus);

	printk(KERN_INFO "NIGHTMARE PM-hotplug init function\n");

	switch (event) {
	case CPUFREQ_GOV_START:
		if ((!cpu_online(0)) || (!policy->cur))
			return -EINVAL;

		#ifdef CONFIG_CPU_FREQ
			table = cpufreq_frequency_get_table(0);

			for (i = 0; table[i].frequency != CPUFREQ_TABLE_END; i++) {
				freq = table[i].frequency;

				if (freq != CPUFREQ_ENTRY_INVALID && freq > freq_max)
					freq_max = freq;
				else if (freq != CPUFREQ_ENTRY_INVALID && dbs_tuners_ins.freq_min > freq)
					dbs_tuners_ins.freq_min = freq;
			}
			/*get max frequence*/
			max_performance = freq_max * NUM_CPUS;
		#else
			max_performance = clk_get_rate(clk_get(NULL, "armclk")) / 1000 * NUM_CPUS;
			dbs_tuners_ins.freq_min = clk_get_rate(clk_get(NULL, "armclk")) / 1000;
		#endif

		//#if !EARLYSUSPEND_HOTPLUGLOCK
			register_pm_notifier(&nightmare_pm_hotplug_notifier);
		//#endif

			register_reboot_notifier(&hotplug_reboot_notifier);		
			// register second_core device by tegrak
			ret = misc_register(&second_core_device);
			if (ret) {
				printk(KERN_ERR "failed at(%d)\n", __LINE__);
				return ret;
			}
	
			//ret = sysfs_create_group(&second_core_device.this_device->kobj, &second_core_group);
			ret = sysfs_create_group(cpufreq_global_kobject,&dbs_attr_group);
			if (ret) {
				printk(KERN_ERR "failed at(%d)\n", __LINE__);
				return ret;
			}
		#ifdef CONFIG_HAS_EARLYSUSPEND
			register_early_suspend(&hotplug_early_suspend_notifier);
		#endif	

		break;

	case CPUFREQ_GOV_STOP:
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&hotplug_early_suspend_notifier);
#endif
//#if !EARLYSUSPEND_HOTPLUGLOCK
		unregister_pm_notifier(&nightmare_pm_hotplug_notifier);
//#endif

		hotplug_timer_exit(policy);

		unregister_reboot_notifier(&hotplug_reboot_notifier);

		sysfs_remove_group(cpufreq_global_kobject,
					   &dbs_attr_group);
		break;

	case CPUFREQ_GOV_LIMITS:

		break;
	}
	return 0;
}

static struct platform_device nightmare_pm_hotplug_device = {
	.name = "nightmare-dynamic-cpu-hotplug",
	.id = -1,
};

static int nghthotplug_cpufreq_policy_notifier_call(struct notifier_block *this,
				unsigned long code, void *data)
{
	struct cpufreq_policy *policy = data;
	unsigned int hotpluging_rate = dbs_tuners_ins.hotpluging_rate;

	switch (code) {
	case CPUFREQ_ADJUST:
		DBG_PRINT("Stand-hotplug is enabled: governor=%s\n",policy->governor->name);
		mutex_lock(&hotplug_lock);
		standhotplug_enabled = true;
		queue_delayed_work_on(0, hotplug_wq, &hotplug_work, hotpluging_rate);
		mutex_unlock(&hotplug_lock);
		break;
	case CPUFREQ_INCOMPATIBLE:
	case CPUFREQ_NOTIFY:
	default:
		break;
	}

	return NOTIFY_DONE;
}
static struct notifier_block nghthotplug_cpufreq_policy_notifier = {
	.notifier_call = nghthotplug_cpufreq_policy_notifier_call,
};

static int __init nightmare_pm_hotplug_device_init(void)
{
	int ret;

	standhotplug_enabled = 1;

	ret = platform_device_register(&nightmare_pm_hotplug_device);

	if (ret) {
		printk(KERN_ERR "failed at(%d)\n", __LINE__);
		return ret;
	}

	printk(KERN_INFO "nightmare_pm_hotplug_device_init: %d\n", ret);

	cpufreq_register_notifier(&nghthotplug_cpufreq_policy_notifier,
						CPUFREQ_POLICY_NOTIFIER);

	ret = cpufreq_register_governor(&cpufreq_gov_nightmare);

	if (ret) {
		printk(KERN_ERR "Registering Nightmare governor failed at(%d)\n", __LINE__);
		return ret;
	}

	return ret;
}

static void __exit nightmare_pm_hotplug_device_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_nightmare);
	cpufreq_unregister_notifier(&nghthotplug_cpufreq_policy_notifier);
}

MODULE_AUTHOR("Alucard24@XDA <dmbaoh2@gmail.com>");
MODULE_DESCRIPTION("'cpufreq_nightmare' - A dynamic cpufreq/cpuhotplug governor");
MODULE_LICENSE("GPL");

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_NIGHTMARE
fs_initcall(nightmare_pm_hotplug_device_init);
#else
module_init(nightmare_pm_hotplug_device_init);
#endif
module_exit(nightmare_pm_hotplug_device_exit);

