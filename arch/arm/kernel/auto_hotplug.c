/* Copyright (c) 2012, Will Tisdale <willtisdale@gmail.com>. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

/*
 * Generic auto hotplug driver for ARM SoCs. Targeted at current generation
 * SoCs with dual and quad core applications processors.
 * Automatically hotplugs online and offline CPUs based on system load.
 * It is also capable of immediately onlining a core based on an external
 * event by calling void hotplug_boostpulse(void)
 *
 * Not recommended for use with OMAP4460 due to the potential for lockups
 * whilst hotplugging.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/workqueue.h>
#include <linux/sched.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

/*
 * Enable debug output to dump the average
 * calculations and ring buffer array values
 * WARNING: Enabling this causes a ton of overhead
 *
 * FIXME: Turn it into debugfs stats (somehow)
 * because currently it is a sack of shit.
 */
#define DEBUG 0

#define CPUS_AVAILABLE		num_possible_cpus()
/*
 * SAMPLING_PERIODS * MIN_SAMPLING_RATE is the minimum
 * load history which will be averaged
 */
#define SAMPLING_PERIODS	15
#define INDEX_MAX_VALUE		(SAMPLING_PERIODS - 1)
/*
 * MIN_SAMPLING_RATE is scaled based on num_online_cpus()
 */
#define MIN_SAMPLING_RATE	msecs_to_jiffies(20)

/*
 * Load defines:
 * ENABLE_ALL is a high watermark to rapidly online all CPUs
 *
 * ENABLE is the load which is required to enable 1 extra CPU
 * DISABLE is the load at which a CPU is disabled
 * These two are scaled based on num_online_cpus()
 */
#define ENABLE_ALL_LOAD_THRESHOLD	440
#define ENABLE_LOAD_THRESHOLD		290
#define DISABLE_LOAD_THRESHOLD		250
static int enable_load[] = { 0, 290, 340, 390 };
static int hotplug_cpu_single_on[] = { 0, 0, 0, 0 };
static int hotplug_cpu_single_off[] = { 0, 0, 0, 0 };

/* Control flags */
unsigned char flags;
#define HOTPLUG_DISABLED	(1 << 0)
#define HOTPLUG_PAUSED		(1 << 1)
#define BOOSTPULSE_ACTIVE	(1 << 2)
#define EARLYSUSPEND_ACTIVE	(1 << 3)

struct delayed_work hotplug_decision_work;
struct delayed_work hotplug_unpause_work;
struct work_struct hotplug_online_all_work;
struct work_struct hotplug_online_single_work;
struct delayed_work aphotplug_offline_work;
struct work_struct hotplug_offline_all_work;
struct work_struct hotplug_boost_online_work;

static unsigned int history[SAMPLING_PERIODS];
static unsigned int index;

static unsigned int min_online_cpus = 1;

static bool isEnabled = false;
module_param(isEnabled, bool, 0644);

static bool EnableOverride = false;

static int min_online_cpus_fn_set(const char *arg, const struct kernel_param *kp)
{
    int ret; 
    
    ret = param_set_int(arg, kp);
    
    ///at least 1 core must run even if set value is out of range
    if ((min_online_cpus < 1) || (min_online_cpus > CPUS_AVAILABLE))
    {
        min_online_cpus = 1;
    }
    
    //online all cores and offline them based on set value
    schedule_work_on(0, &hotplug_online_all_work);
        
    return ret;
}

static int min_online_cpus_fn_get(char *buffer, const struct kernel_param *kp)
{
    return param_get_int(buffer, kp);
}

static struct kernel_param_ops min_online_cpus_ops = 
{
    .set = min_online_cpus_fn_set,
    .get = min_online_cpus_fn_get,
};

module_param_cb(min_online_cpus, &min_online_cpus_ops, &min_online_cpus, 0755);


static void hotplug_decision_work_fn(struct work_struct *work)
{
	unsigned int running, disable_load, sampling_rate, avg_running = 0;
	unsigned int online_cpus, available_cpus, i, j;
	bool hotplug_flag_on = false;
	bool hotplug_flag_off = false;
#if DEBUG
	unsigned int k;
#endif
	if (!isEnabled)
		return;
		
	online_cpus = num_online_cpus();
	available_cpus = CPUS_AVAILABLE;
	disable_load = DISABLE_LOAD_THRESHOLD; // * online_cpus;
	//enable_load = ENABLE_LOAD_THRESHOLD; // * online_cpus;
	/*
	 * Multiply nr_running() by 100 so we don't have to
	 * use fp division to get the average.
	 */
	running = nr_running() * 100;

	history[index] = running;

#if DEBUG
	pr_info("online_cpus is: %d\n", online_cpus);
	//pr_info("enable_load is: %d\n", enable_load);
	pr_info("disable_load is: %d\n", disable_load);
	pr_info("index is: %d\n", index);
	pr_info("running is: %d\n", running);
#endif

	/*
	 * Use a circular buffer to calculate the average load
	 * over the sampling periods.
	 * This will absorb load spikes of short duration where
	 * we don't want additional cores to be onlined because
	 * the cpufreq driver should take care of those load spikes.
	 */
	for (i = 0, j = index; i < SAMPLING_PERIODS; i++, j--) {
		avg_running += history[j];
		if (unlikely(j == 0))
			j = INDEX_MAX_VALUE;
	}

	/*
	 * If we are at the end of the buffer, return to the beginning.
	 */
	if (unlikely(index++ == INDEX_MAX_VALUE))
		index = 0;

#if DEBUG
	pr_info("array contents: ");
	for (k = 0; k < SAMPLING_PERIODS; k++) {
		 pr_info("%d: %d\t",k, history[k]);
	}
	pr_info("\n");
	pr_info("avg_running before division: %d\n", avg_running);
#endif

	avg_running = avg_running / SAMPLING_PERIODS;

#if DEBUG
	pr_info("average_running is: %d\n", avg_running);
#endif

	if (likely(!(flags & HOTPLUG_DISABLED))) {
		int cpu;
		for (cpu = 1; cpu < CPUS_AVAILABLE; cpu++)
		{
			if (avg_running >= enable_load[cpu] && (!cpu_online(cpu)))
			{
				hotplug_cpu_single_on[cpu] = 1;
				hotplug_flag_on = true;
			}
			else if (avg_running < enable_load[cpu] && (cpu_online(cpu)))
			{
				hotplug_cpu_single_off[cpu] = 1;
				hotplug_flag_off = true;
			}
		}
	
		if (unlikely((avg_running >= ENABLE_ALL_LOAD_THRESHOLD) && (online_cpus < available_cpus))) {
			pr_info("auto_hotplug: Onlining all CPUs, avg running: %d\n", avg_running);
			/*
			 * Flush any delayed offlining work from the workqueue.
			 * No point in having expensive unnecessary hotplug transitions.
			 * We still online after flushing, because load is high enough to
			 * warrant it.
			 * We set the paused flag so the sampling can continue but no more
			 * hotplug events will occur.
			 */
			flags |= HOTPLUG_PAUSED;
			if (delayed_work_pending(&aphotplug_offline_work))
				cancel_delayed_work(&aphotplug_offline_work);
			schedule_work_on(0, &hotplug_online_all_work);
			return;
		} else if (flags & HOTPLUG_PAUSED) {
			schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
			return;
		} else if (hotplug_flag_on) {
#if DEBUG
			pr_info("auto_hotplug: Onlining single CPU, avg running: %d\n", avg_running);
#endif
			if (delayed_work_pending(&aphotplug_offline_work))
				cancel_delayed_work(&aphotplug_offline_work);
			hotplug_flag_on = false;
			schedule_work_on(0, &hotplug_online_single_work);
			return;
		} else if (hotplug_flag_off) {
			/* Only queue a cpu_down() if there isn't one already pending */
			if (!(delayed_work_pending(&aphotplug_offline_work))) {
#if DEBUG
				pr_info("auto_hotplug: Offlining CPU, avg running: %d\n", avg_running);
#endif
				hotplug_flag_off = false;
				schedule_delayed_work_on(0, &aphotplug_offline_work, HZ);
			}
			/* If boostpulse is active, clear the flags */
			if (flags & BOOSTPULSE_ACTIVE) {
				flags &= ~BOOSTPULSE_ACTIVE;
#if DEBUG
				pr_info("auto_hotplug: Clearing boostpulse flags\n");
#endif
			}
		}
	}

	/*
	 * Reduce the sampling rate dynamically based on online cpus.
	 */
	sampling_rate = MIN_SAMPLING_RATE * (online_cpus * online_cpus);
#if DEBUG
	pr_info("sampling_rate is: %d\n", jiffies_to_msecs(sampling_rate));
#endif
	schedule_delayed_work_on(0, &hotplug_decision_work, sampling_rate);

}

bool apget_enable_auto_hotplug(void)
{
	return isEnabled;
}

void apenable_auto_hotplug(bool state)
{
	unsigned int sampling_rate;
	unsigned int online_cpus = num_online_cpus();
	
	isEnabled = state;
	if (isEnabled)
	{
		sampling_rate = msecs_to_jiffies(1000);
#if DEBUG
		pr_info("sampling_rate is: %d\n", jiffies_to_msecs(sampling_rate));
#endif
		schedule_delayed_work_on(0, &hotplug_decision_work, sampling_rate);
	}
	else
	{
		sampling_rate = MIN_SAMPLING_RATE * (online_cpus * online_cpus);
#if DEBUG
		pr_info("sampling_rate is: %d\n", jiffies_to_msecs(sampling_rate));
#endif
		schedule_delayed_work_on(0, &hotplug_decision_work, sampling_rate);
		EnableOverride = true;
		schedule_work_on(0, &hotplug_online_all_work);
	}
}

static void __cpuinit hotplug_online_all_work_fn(struct work_struct *work)
{
	int cpu;
	if (!isEnabled && !EnableOverride)
		return;
	EnableOverride = false;
	
	for_each_possible_cpu(cpu) {
		if (likely(!cpu_online(cpu)) && (cpu)) {
			cpu_up(cpu);
#if DEBUG
			pr_info("auto_hotplug: CPU%d up.\n", cpu);
#endif
		}
	}
	/*
	 * Pause for 2 seconds before even considering offlining a CPU
	 */
	schedule_delayed_work(&hotplug_unpause_work, HZ * 2);
	schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
}

static void __cpuinit hotplug_offline_all_work_fn(struct work_struct *work)
{
	int cpu;
	if (!isEnabled)
		return;

	for_each_possible_cpu(cpu) {
		if (likely(cpu_online(cpu) && (cpu))) {
			cpu_down(cpu);
#if DEBUG
			pr_info("auto_hotplug: CPU%d down.\n", cpu);
#endif
		}
	}
}

static void __cpuinit hotplug_online_single_work_fn(struct work_struct *work)
{
	int cpu;
	if (!isEnabled)
		return;

	for_each_possible_cpu(cpu) {
		if (likely(!cpu_online(cpu) && (cpu))) {
			if (hotplug_cpu_single_on[cpu])
			{
				hotplug_cpu_single_on[cpu] = 0;
				cpu_up(cpu);
			}
#if DEBUG
			pr_info("auto_hotplug: CPU%d up.\n", cpu);
#endif
			break;
		}
	}
	schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
}

static void aphotplug_offline_work_fn(struct work_struct *work)
{
	int cpu;
	if (!isEnabled)
		return;

	for_each_online_cpu(cpu) {
		if (likely(cpu_online(cpu) && (cpu))) {
			if (hotplug_cpu_single_off[cpu])
			{
				hotplug_cpu_single_off[cpu] = 0;
				cpu_down(cpu);
			}
#if DEBUG
			pr_info("auto_hotplug: CPU%d down.\n", cpu);
#endif
			break;
		}
	}
	schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
}

static void hotplug_unpause_work_fn(struct work_struct *work)
{
#if DEBUG
	pr_info("auto_hotplug: Clearing pause flag\n");
#endif
	flags &= ~HOTPLUG_PAUSED;
}

void hotplug_disable(bool flag)
{
	if (flags & HOTPLUG_DISABLED && !flag) {
		flags &= ~HOTPLUG_DISABLED;
		flags &= ~HOTPLUG_PAUSED;
#if DEBUG
		pr_info("auto_hotplug: Clearing disable flag\n");
#endif
		schedule_delayed_work_on(0, &hotplug_decision_work, 0);
	} else if (flag && (!(flags & HOTPLUG_DISABLED))) {
		flags |= HOTPLUG_DISABLED;
#if DEBUG
		pr_info("auto_hotplug: Setting disable flag\n");
#endif
		cancel_delayed_work_sync(&aphotplug_offline_work);
		cancel_delayed_work_sync(&hotplug_decision_work);
		cancel_delayed_work_sync(&hotplug_unpause_work);
	}
}

inline void hotplugap_boostpulse(void)
{
	if (!isEnabled)
		return;

	if (unlikely(flags & (EARLYSUSPEND_ACTIVE
		| HOTPLUG_DISABLED)))
		return;

	if (!(flags & BOOSTPULSE_ACTIVE)) {
		flags |= BOOSTPULSE_ACTIVE;
		/*
		 * If there are less than 2 CPUs online, then online
		 * an additional CPU, otherwise check for any pending
		 * offlines, cancel them and pause for 2 seconds.
		 * Either way, we don't allow any cpu_down()
		 * whilst the user is interacting with the device.
		 */
		if (likely(num_online_cpus() < 2)) {
			cancel_delayed_work_sync(&aphotplug_offline_work);
			flags |= HOTPLUG_PAUSED;
			schedule_work_on(0, &hotplug_online_single_work);
			schedule_delayed_work(&hotplug_unpause_work, HZ );
		} else {
#if DEBUG
			pr_info("auto_hotplug: %s: %d CPUs online\n", __func__, num_online_cpus());
#endif
			if (delayed_work_pending(&aphotplug_offline_work)) {
#if DEBUG
				pr_info("auto_hotplug: %s: Cancelling aphotplug_offline_work\n", __func__);
#endif
				cancel_delayed_work(&aphotplug_offline_work);
				flags |= HOTPLUG_PAUSED;
				schedule_delayed_work(&hotplug_unpause_work, HZ );
				schedule_delayed_work_on(0, &hotplug_decision_work, MIN_SAMPLING_RATE);
			}
		}
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void auto_hotplug_early_suspend(struct early_suspend *handler)
{
	if (!isEnabled)
		return;

	pr_info("auto_hotplug: early suspend handler\n");
	flags |= EARLYSUSPEND_ACTIVE;

	/* Cancel all scheduled delayed work to avoid races */
	cancel_delayed_work_sync(&aphotplug_offline_work);
	cancel_delayed_work_sync(&hotplug_decision_work);
	if (num_online_cpus() > 1) {
		pr_info("auto_hotplug: Offlining CPUs for early suspend\n");
		schedule_work_on(0, &hotplug_offline_all_work);
	}
}

static void auto_hotplug_late_resume(struct early_suspend *handler)
{
	if (!isEnabled)
		return;

	pr_info("auto_hotplug: late resume handler\n");
	flags &= ~EARLYSUSPEND_ACTIVE;

	schedule_delayed_work_on(0, &hotplug_decision_work, HZ);
}

static struct early_suspend auto_hotplug_suspend = {
	.suspend = auto_hotplug_early_suspend,
	.resume = auto_hotplug_late_resume,
};
#endif /* CONFIG_HAS_EARLYSUSPEND */

static int __init auto_hotplug_init(void)
{
	pr_info("auto_hotplug: v0.220 by _thalamus modded by ktoonsez for KTSGS4\n");
	pr_info("auto_hotplug: %d CPUs detected\n", CPUS_AVAILABLE);

	INIT_DELAYED_WORK(&hotplug_decision_work, hotplug_decision_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&hotplug_unpause_work, hotplug_unpause_work_fn);
	INIT_WORK(&hotplug_online_all_work, hotplug_online_all_work_fn);
	INIT_WORK(&hotplug_online_single_work, hotplug_online_single_work_fn);
	INIT_WORK(&hotplug_offline_all_work, hotplug_offline_all_work_fn);
	INIT_DELAYED_WORK_DEFERRABLE(&aphotplug_offline_work, aphotplug_offline_work_fn);

	/*
	 * Give the system time to boot before fiddling with hotplugging.
	 */
	flags |= HOTPLUG_PAUSED;
	schedule_delayed_work_on(0, &hotplug_decision_work, HZ * 5);
	schedule_delayed_work(&hotplug_unpause_work, HZ * 10);

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&auto_hotplug_suspend);
#endif
	return 0;
}
late_initcall(auto_hotplug_init);
