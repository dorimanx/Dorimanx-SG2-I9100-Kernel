/*
 * drivers/cpufreq/cpufreq_zenx.c
 *
 * Copyright (C) 2010 Google, Inc.
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
 * Author: Mike Chan (mike@android.com)
 * Author: Brandon Berhent (bbedward@androiddeveloperalliance.org)
 *
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/rwsem.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/earlysuspend.h>
#include <linux/slab.h>

#include <asm/cputime.h>

static atomic_t active_count = ATOMIC_INIT(0);

struct cpufreq_zenx_cpuinfo {
	struct timer_list cpu_timer;
	struct timer_list cpu_slack_timer;
	spinlock_t load_lock; /* protects the next 4 fields */
	u64 time_in_idle;
	u64 time_in_idle_timestamp;
	u64 cputime_speedadj;
	u64 cputime_speedadj_timestamp;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	unsigned int target_freq;
	unsigned int floor_freq;
	unsigned int last_cpu_load;
	unsigned int nr_periods_add;
	unsigned int nr_periods_remove;
	unsigned int add_avg_load;
	unsigned int remove_avg_load;
	u64 floor_validate_time;
	u64 hispeed_validate_time;
	struct rw_semaphore enable_sem;
	int governor_enabled;
};

static DEFINE_PER_CPU(struct cpufreq_zenx_cpuinfo, cpuinfo);

/* realtime thread handles frequency scaling */
static struct task_struct *speedchange_task;
static cpumask_t speedchange_cpumask;
static spinlock_t speedchange_cpumask_lock;
static struct mutex set_speed_lock;

/* workqueues handle hotplugging */
static struct workqueue_struct *hotplug_add_wq;
static struct work_struct hotplug_add_work;
static cpumask_t hotplug_add_cpumask;
static spinlock_t hotplug_add_cpumask_lock;
static struct workqueue_struct *hotplug_remove_wq;
static struct work_struct hotplug_remove_work;
static cpumask_t hotplug_remove_cpumask;
static spinlock_t hotplug_remove_cpumask_lock;

/* Hi speed to bump to from lo speed when load burst (default max) */
static unsigned int hispeed_freq;

/* Go to hi speed when CPU load at or above this value. */
#define DEFAULT_GO_HISPEED_LOAD 99
static unsigned long go_hispeed_load = DEFAULT_GO_HISPEED_LOAD;

/* Unplug auxillary CPUs below these values. */
#define DEFAULT_UNPLUG_LOAD_CPU1 35
#define DEFAULT_UNPLUG_LOAD_CPU2 50
#define DEFAULT_UNPLUG_LOAD_CPUMORE 50

static unsigned int unplug_load[] =
	{ DEFAULT_UNPLUG_LOAD_CPU1,
	  DEFAULT_UNPLUG_LOAD_CPU2,
	  DEFAULT_UNPLUG_LOAD_CPUMORE
	};

/* Target load.  Lower values result in higher CPU speeds. */
#define DEFAULT_TARGET_LOAD 90
static unsigned int default_target_loads[] = {DEFAULT_TARGET_LOAD};
static spinlock_t target_loads_lock;
static unsigned int *target_loads = default_target_loads;
static int ntarget_loads = ARRAY_SIZE(default_target_loads);

/*
 * Number of sampling periods to take average CPU load across
 * for CPU removal.
 */
#define DEFAULT_NR_REMOVE_PERIODS (100)
#define DEFAULT_NR_REMOVE_PERIODS_SUSPENDED (50)
static unsigned int curr_hot_remove_sampling_periods = DEFAULT_NR_REMOVE_PERIODS;
static unsigned int hot_remove_sampling_periods = DEFAULT_NR_REMOVE_PERIODS;
static unsigned int hot_remove_sampling_periods_suspended = DEFAULT_NR_REMOVE_PERIODS_SUSPENDED;

/*
 * Number of sampling periods to take average CPU load across
 * for CPU add.
 */
#define DEFAULT_NR_ADD_PERIODS (30)
#define DEFAULT_NR_ADD_PERIODS_SUSPENDED (100)
static unsigned int curr_hot_add_sampling_periods = DEFAULT_NR_ADD_PERIODS;
static unsigned int hot_add_sampling_periods = DEFAULT_NR_ADD_PERIODS;
static unsigned int hot_add_sampling_periods_suspended = DEFAULT_NR_ADD_PERIODS_SUSPENDED;

/*
 * The minimum amount of time to spend at a frequency before we can ramp down.
 */
#define DEFAULT_MIN_SAMPLE_TIME (40 * USEC_PER_MSEC)
static unsigned long min_sample_time = DEFAULT_MIN_SAMPLE_TIME;

/*
 * The sample rate of the timer used to increase frequency
 */
#define DEFAULT_TIMER_RATE (20 * USEC_PER_MSEC)
#define DEFAULT_TIMER_RATE_SUSPENDED (30 * USEC_PER_MSEC)
static unsigned long curr_timer_rate = DEFAULT_TIMER_RATE;
static unsigned long timer_rate = DEFAULT_TIMER_RATE;
static unsigned long timer_rate_suspended = DEFAULT_TIMER_RATE_SUSPENDED;

/*
 * Wait this long before raising speed above hispeed, by default a single
 * timer interval.
 */
#define DEFAULT_ABOVE_HISPEED_DELAY DEFAULT_TIMER_RATE
static unsigned long above_hispeed_delay_val = DEFAULT_ABOVE_HISPEED_DELAY;

/* Non-zero means indefinite speed boost active */
static int boost_val;
/* Duration of a boot pulse in usecs */
static int boostpulse_duration_val = DEFAULT_MIN_SAMPLE_TIME;
/* End time of boost pulse in ktime converted to usecs */
static u64 boostpulse_endtime;

/*
 * Max additional time to wait in idle, beyond timer_rate, at speeds above
 * minimum before wakeup to reduce speed, or -1 if unnecessary.
 */
#define DEFAULT_TIMER_SLACK (4 * DEFAULT_TIMER_RATE)
static int timer_slack_val = DEFAULT_TIMER_SLACK;

static int cpufreq_governor_zenx(struct cpufreq_policy *policy,
		unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_ZENX
static
#endif
struct cpufreq_governor cpufreq_gov_zenx = {
	.name = "ZenX",
	.governor = cpufreq_governor_zenx,
	.max_transition_latency = 10000000,
	.owner = THIS_MODULE,
};

static void cpufreq_zenx_timer_resched(
	struct cpufreq_zenx_cpuinfo *pcpu)
{
	unsigned long expires = jiffies + usecs_to_jiffies(curr_timer_rate);
	unsigned long flags;

	mod_timer_pinned(&pcpu->cpu_timer, expires);
	if (timer_slack_val >= 0 && pcpu->target_freq > pcpu->policy->min) {
		expires += usecs_to_jiffies(timer_slack_val);
		mod_timer_pinned(&pcpu->cpu_slack_timer, expires);
	}

	spin_lock_irqsave(&pcpu->load_lock, flags);
	pcpu->time_in_idle =
		get_cpu_idle_time_us(smp_processor_id(),
				     &pcpu->time_in_idle_timestamp);
	pcpu->cputime_speedadj = 0;
	pcpu->cputime_speedadj_timestamp = pcpu->time_in_idle_timestamp;
	spin_unlock_irqrestore(&pcpu->load_lock, flags);
}

static unsigned int freq_to_targetload(unsigned int freq)
{
	int i;
	unsigned int ret;
	unsigned long flags;

	spin_lock_irqsave(&target_loads_lock, flags);

	for (i = 0; i < ntarget_loads - 1 && freq >= target_loads[i+1]; i += 2)
		;

	ret = target_loads[i];
	spin_unlock_irqrestore(&target_loads_lock, flags);
	return ret;
}

/*
 * If increasing frequencies never map to a lower target load then
 * choose_freq() will find the minimum frequency that does not exceed its
 * target load given the current load.
 */

static unsigned int choose_freq(
	struct cpufreq_zenx_cpuinfo *pcpu, unsigned int loadadjfreq)
{
	unsigned int freq = pcpu->policy->cur;
	unsigned int prevfreq, freqmin, freqmax;
	unsigned int tl;
	int index;
	freqmin = 0;
	freqmax = UINT_MAX;

	do {
		prevfreq = freq;
		tl = freq_to_targetload(freq);

		/*
		 * Find the lowest frequency where the computed load is less
		 * than or equal to the target load.
		 */

		cpufreq_frequency_table_target(
			pcpu->policy, pcpu->freq_table, loadadjfreq / tl,
			CPUFREQ_RELATION_L, &index);
		freq = pcpu->freq_table[index].frequency;

		if (freq > prevfreq) {
			/* The previous frequency is too low. */
			freqmin = prevfreq;

			if (freq >= freqmax) {
				/*
				 * Find the highest frequency that is less
				 * than freqmax.
				 */
				cpufreq_frequency_table_target(
					pcpu->policy, pcpu->freq_table,
					freqmax - 1, CPUFREQ_RELATION_H,
					&index);
				freq = pcpu->freq_table[index].frequency;

				if (freq == freqmin) {
					/*
					 * The first frequency below freqmax
					 * has already been found to be too
					 * low.  freqmax is the lowest speed
					 * we found that is fast enough.
					 */
					freq = freqmax;
					break;
				}
			}
		} else if (freq < prevfreq) {
			/* The previous frequency is high enough. */
			freqmax = prevfreq;

			if (freq <= freqmin) {
				/*
				 * Find the lowest frequency that is higher
				 * than freqmin.
				 */
				cpufreq_frequency_table_target(
					pcpu->policy, pcpu->freq_table,
					freqmin + 1, CPUFREQ_RELATION_L,
					&index);
				freq = pcpu->freq_table[index].frequency;

				/*
				 * If freqmax is the first frequency above
				 * freqmin then we have already found that
				 * this speed is fast enough.
				 */
				if (freq == freqmax)
					break;
			}
		}

		/* If same frequency chosen as previous then done. */
	} while (freq != prevfreq);

	return freq;
}

static u64 update_load(int cpu)
{
	struct cpufreq_zenx_cpuinfo *pcpu = &per_cpu(cpuinfo, cpu);
	u64 now;
	u64 now_idle;
	unsigned int delta_idle;
	unsigned int delta_time;
	u64 active_time;

	now_idle = get_cpu_idle_time_us(cpu, &now);
	delta_idle = (unsigned int)(now_idle - pcpu->time_in_idle);
	delta_time = (unsigned int)(now - pcpu->time_in_idle_timestamp);
	active_time = delta_time - delta_idle;
	pcpu->cputime_speedadj += active_time * pcpu->policy->cur;

	pcpu->time_in_idle = now_idle;
	pcpu->time_in_idle_timestamp = now;

	return now;
}

static void cpufreq_zenx_timer(unsigned long data)
{
	u64 now;
	unsigned int delta_time;
	u64 cputime_speedadj;
	unsigned int cpu, cpu_load, avg_load;
	struct cpufreq_zenx_cpuinfo *pcpu =
		&per_cpu(cpuinfo, data);
	unsigned int new_freq;
	unsigned int loadadjfreq;
	unsigned int index, up_load_index;
	unsigned int total_load = 0;
	unsigned int rearm_if_notmax = 0;
	unsigned long flags;
	bool boosted;

	if (!down_read_trylock(&pcpu->enable_sem))
		return;

	if (!pcpu->governor_enabled)
		goto exit;

	/*
	 * Skip load calculation and frequency logic for this CPU
	 * if it is offline.
	 */
	if (data > 0 && !cpu_online(data))
		goto call_hp_work;

	spin_lock_irqsave(&pcpu->load_lock, flags);
	now = update_load(data);
	delta_time = (unsigned int)(now - pcpu->cputime_speedadj_timestamp);
	cputime_speedadj = pcpu->cputime_speedadj;
	spin_unlock_irqrestore(&pcpu->load_lock, flags);

	if (WARN_ON_ONCE(!delta_time))
		goto rearm;

	do_div(cputime_speedadj, delta_time);
	loadadjfreq = (unsigned int)cputime_speedadj * 100;
	cpu_load = loadadjfreq / pcpu->target_freq;
	pcpu->last_cpu_load = cpu_load;
	boosted = boost_val || now < boostpulse_endtime;

	if (cpu_load >= go_hispeed_load || boosted) {
		if (pcpu->target_freq < hispeed_freq) {
			new_freq = hispeed_freq;
		} else {
			new_freq = choose_freq(pcpu, loadadjfreq);

			if (new_freq < hispeed_freq)
				new_freq = hispeed_freq;
		}
	} else {
		new_freq = choose_freq(pcpu, loadadjfreq);
	}

	if (pcpu->target_freq >= hispeed_freq &&
	    new_freq > pcpu->target_freq &&
	    now - pcpu->hispeed_validate_time < above_hispeed_delay_val) {
		goto call_hp_work;
	}

	pcpu->hispeed_validate_time = now;

	if (cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table,
					   new_freq, CPUFREQ_RELATION_L,
					   &index)) {
		pr_warn_once("timer %d: cpufreq_frequency_table_target error\n",
			     (int) data);
		goto call_hp_work;
	}

	new_freq = pcpu->freq_table[index].frequency;

	/*
	 * Do not scale below floor_freq unless we have been at or above the
	 * floor frequency for the minimum sample time since last validated.
	 */
	if (new_freq < pcpu->floor_freq) {
		if (now - pcpu->floor_validate_time < min_sample_time) {
			goto call_hp_work;
		}
	}

	/*
	 * Update the timestamp for checking whether speed has been held at
	 * or above the selected frequency for a minimum of min_sample_time,
	 * if not boosted to hispeed_freq.  If boosted to hispeed_freq then we
	 * allow the speed to drop as soon as the boostpulse duration expires
	 * (or the indefinite boost is turned off).
	 */

	if (!boosted || new_freq > hispeed_freq) {
		pcpu->floor_freq = new_freq;
		pcpu->floor_validate_time = now;
	}

	if (pcpu->target_freq == new_freq) {
		rearm_if_notmax = 1;
		goto call_hp_work;
	}

	pcpu->target_freq = new_freq;
	spin_lock_irqsave(&speedchange_cpumask_lock, flags);
	cpumask_set_cpu(data, &speedchange_cpumask);
	spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);
	wake_up_process(speedchange_task);

call_hp_work:
	/* Skip hot-add/remove calculations for CPU 0 */
	if (data > 0) {
		if (data > 2)
			up_load_index = 2;
		else
			up_load_index = data - 1;
	        /*
	         * Compute average load across all online CPUs
        	 */
        	for_each_cpu(cpu, cpu_online_mask) {
			struct cpufreq_zenx_cpuinfo *pjcpu =
				&per_cpu(cpuinfo, cpu);

			total_load += pjcpu->last_cpu_load;
		}
		avg_load = total_load / num_online_cpus();

		/* Increment nr_periods_add/remove and load */
		pcpu->nr_periods_add++;
		pcpu->nr_periods_remove++;
		pcpu->add_avg_load+=avg_load;
		pcpu->remove_avg_load+=avg_load;

		/*
		 * If we've been here for at least hot_add/remove
		 * sampling periods reset load+period counter and
		 * compare average load to unplug_load. If it exceeds
		 * add the CPU, if it is lower than remove the CPU.
		 */
		if (pcpu->nr_periods_add >= curr_hot_add_sampling_periods) {
			if (pcpu->add_avg_load / pcpu->nr_periods_add
			    > unplug_load[up_load_index]) {
				spin_lock_irqsave(&hotplug_add_cpumask_lock, flags);
				cpumask_set_cpu(data, &hotplug_add_cpumask);
				spin_unlock_irqrestore(&hotplug_add_cpumask_lock, flags);
				queue_work(hotplug_add_wq, &hotplug_add_work);
				/* Reset remove counters so we don't instantly do more work. */
				pcpu->remove_avg_load = 0;
				pcpu->nr_periods_remove = 0;
			}
			/* Reset add period counters */
			pcpu->add_avg_load = 0;
			pcpu->nr_periods_add = 0;
		} else if (pcpu->nr_periods_remove >= curr_hot_remove_sampling_periods) {
			if (pcpu->remove_avg_load / pcpu->nr_periods_remove
			    <= unplug_load[up_load_index]) {
				spin_lock_irqsave(&hotplug_remove_cpumask_lock, flags);
				cpumask_set_cpu(data, &hotplug_remove_cpumask);
				spin_unlock_irqrestore(&hotplug_remove_cpumask_lock, flags);
				queue_work(hotplug_remove_wq, &hotplug_remove_work);
				/* Reset add counters so we don't instantly do more work. */
				pcpu->add_avg_load = 0;
				pcpu->nr_periods_add = 0;
			}
			/* Reset remove period counters */
			pcpu->remove_avg_load = 0;
			pcpu->nr_periods_remove = 0;
		}
	}

rearm:
	if (rearm_if_notmax) {
		/*
		 * Already set max speed and don't see a need to change that,
		 * wait until next idle to re-evaluate, don't need timer.
		 */
		if (pcpu->target_freq == pcpu->policy->max)
			goto exit;
	}

	if (!timer_pending(&pcpu->cpu_timer))
		cpufreq_zenx_timer_resched(pcpu);

exit:
	up_read(&pcpu->enable_sem);
	return;
}

static void cpufreq_zenx_idle_start(void)
{
	struct cpufreq_zenx_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());
	int pending;

	if (!down_read_trylock(&pcpu->enable_sem))
		return;
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->enable_sem);
		return;
	}

	pending = timer_pending(&pcpu->cpu_timer);

	if (pcpu->target_freq != pcpu->policy->min) {
		/*
		 * Entering idle while not at lowest speed.  On some
		 * platforms this can hold the other CPU(s) at that speed
		 * even though the CPU is idle. Set a timer to re-evaluate
		 * speed so this idle CPU doesn't hold the other CPUs above
		 * min indefinitely.  This should probably be a quirk of
		 * the CPUFreq driver.
		 */
		if (!pending)
			cpufreq_zenx_timer_resched(pcpu);
	}

	up_read(&pcpu->enable_sem);
}

static void cpufreq_zenx_idle_end(void)
{
	struct cpufreq_zenx_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());

	if (!down_read_trylock(&pcpu->enable_sem))
		return;
	if (!pcpu->governor_enabled) {
		up_read(&pcpu->enable_sem);
		return;
	}

	/* Arm the timer for 1-2 ticks later if not already. */
	if (!timer_pending(&pcpu->cpu_timer)) {
		cpufreq_zenx_timer_resched(pcpu);
	} else if (time_after_eq(jiffies, pcpu->cpu_timer.expires)) {
		del_timer(&pcpu->cpu_timer);
		del_timer(&pcpu->cpu_slack_timer);
		cpufreq_zenx_timer(smp_processor_id());
	}

	up_read(&pcpu->enable_sem);
}

static void cpufreq_zenx_hotplug_add_cpu_work(struct work_struct *work)
{
	cpumask_t tmp_mask;
	unsigned int cpu;
	unsigned long flags;
	struct cpufreq_zenx_cpuinfo *pcpu;

	spin_lock_irqsave(&hotplug_add_cpumask_lock, flags);
	tmp_mask = hotplug_add_cpumask;
	cpumask_clear(&hotplug_add_cpumask);
	spin_unlock_irqrestore(&hotplug_add_cpumask_lock, flags);

	for_each_cpu(cpu, &tmp_mask) {
		pcpu = &per_cpu(cpuinfo, cpu);

		if (!down_read_trylock(&pcpu->enable_sem))
			continue;
		if (!pcpu->governor_enabled) {
			up_read(&pcpu->enable_sem);
			continue;
		}

		if (cpu > 0 && !cpu_online(cpu)) {
			mutex_lock(&set_speed_lock);
			cpu_up(cpu);
			mutex_unlock(&set_speed_lock);
		}
		up_read(&pcpu->enable_sem);
	}
}

static void cpufreq_zenx_hotplug_remove_cpu_work(struct work_struct *work)
{
	cpumask_t tmp_mask;
	unsigned int cpu;
	unsigned long flags;
	struct cpufreq_zenx_cpuinfo *pcpu;

	spin_lock_irqsave(&hotplug_remove_cpumask_lock, flags);
	tmp_mask = hotplug_remove_cpumask;
	cpumask_clear(&hotplug_remove_cpumask);
	spin_unlock_irqrestore(&hotplug_remove_cpumask_lock, flags);

	for_each_cpu(cpu, &tmp_mask) {
		pcpu = &per_cpu(cpuinfo, cpu);

		if (!down_read_trylock(&pcpu->enable_sem))
			continue;
		if (!pcpu->governor_enabled) {
			up_read(&pcpu->enable_sem);
			continue;
		}

		if (cpu > 0 && cpu_online(cpu)) {
			mutex_lock(&set_speed_lock);
			cpu_down(cpu);
			mutex_unlock(&set_speed_lock);
		}

		up_read(&pcpu->enable_sem);
	}
}

static int cpufreq_zenx_speedchange_task(void *data)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_zenx_cpuinfo *pcpu;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&speedchange_cpumask_lock, flags);

		if (cpumask_empty(&speedchange_cpumask)) {
			spin_unlock_irqrestore(&speedchange_cpumask_lock,
					       flags);
			schedule();

			if (kthread_should_stop())
				break;

			spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		}

		set_current_state(TASK_RUNNING);
		tmp_mask = speedchange_cpumask;
		cpumask_clear(&speedchange_cpumask);
		spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

		for_each_cpu(cpu, &tmp_mask) {
			unsigned int j;
			unsigned int max_freq = 0;

			pcpu = &per_cpu(cpuinfo, cpu);

			if (!down_read_trylock(&pcpu->enable_sem))
				continue;
			if (!pcpu->governor_enabled) {
				up_read(&pcpu->enable_sem);
				continue;
			}

			mutex_lock(&set_speed_lock);

			for_each_cpu(j, pcpu->policy->cpus) {
				struct cpufreq_zenx_cpuinfo *pjcpu =
					&per_cpu(cpuinfo, j);

				if (pjcpu->target_freq > max_freq)
					max_freq = pjcpu->target_freq;
			}

			if (max_freq != pcpu->policy->cur)
				__cpufreq_driver_target(pcpu->policy,
							max_freq,
							CPUFREQ_RELATION_H);
			mutex_unlock(&set_speed_lock);

			up_read(&pcpu->enable_sem);
		}
	}

	return 0;
}

static void cpufreq_zenx_boost(void)
{
	int i;
	int anyboost = 0;
	unsigned long flags;
	struct cpufreq_zenx_cpuinfo *pcpu;

	spin_lock_irqsave(&speedchange_cpumask_lock, flags);

	for_each_online_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);

		if (pcpu->target_freq < hispeed_freq) {
			pcpu->target_freq = hispeed_freq;
			cpumask_set_cpu(i, &speedchange_cpumask);
			pcpu->hispeed_validate_time =
				ktime_to_us(ktime_get());
			anyboost = 1;
		}

		/*
		 * Set floor freq and (re)start timer for when last
		 * validated.
		 */

		pcpu->floor_freq = hispeed_freq;
		pcpu->floor_validate_time = ktime_to_us(ktime_get());
	}

	spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

	if (anyboost)
		wake_up_process(speedchange_task);
}

static int cpufreq_zenx_notifier(
	struct notifier_block *nb, unsigned long val, void *data)
{
	struct cpufreq_freqs *freq = data;
	struct cpufreq_zenx_cpuinfo *pcpu;
	int cpu;
	unsigned long flags;

	if (val == CPUFREQ_POSTCHANGE) {
		pcpu = &per_cpu(cpuinfo, freq->cpu);
		if (!down_read_trylock(&pcpu->enable_sem))
			return 0;
		if (!pcpu->governor_enabled) {
			up_read(&pcpu->enable_sem);
			return 0;
		}

		for_each_cpu(cpu, pcpu->policy->cpus) {
			struct cpufreq_zenx_cpuinfo *pjcpu =
				&per_cpu(cpuinfo, cpu);
			spin_lock_irqsave(&pjcpu->load_lock, flags);
			update_load(cpu);
			spin_unlock_irqrestore(&pjcpu->load_lock, flags);
		}

		up_read(&pcpu->enable_sem);
	}
	return 0;
}

static struct notifier_block cpufreq_notifier_block = {
	.notifier_call = cpufreq_zenx_notifier,
};

static ssize_t show_target_loads(
	struct kobject *kobj, struct attribute *attr, char *buf)
{
	int i;
	ssize_t ret = 0;
	unsigned long flags;

	spin_lock_irqsave(&target_loads_lock, flags);

	for (i = 0; i < ntarget_loads; i++)
		ret += sprintf(buf + ret, "%u%s", target_loads[i],
			       i & 0x1 ? ":" : " ");

	ret += sprintf(buf + ret, "\n");
	spin_unlock_irqrestore(&target_loads_lock, flags);
	return ret;
}

static ssize_t store_target_loads(
	struct kobject *kobj, struct attribute *attr, const char *buf,
	size_t count)
{
	int ret;
	const char *cp;
	unsigned int *new_target_loads = NULL;
	int ntokens = 1;
	int i;
	unsigned long flags;

	cp = buf;
	while ((cp = strpbrk(cp + 1, " :")))
		ntokens++;

	if (!(ntokens & 0x1))
		goto err_inval;

	new_target_loads = kmalloc(ntokens * sizeof(unsigned int), GFP_KERNEL);
	if (!new_target_loads) {
		ret = -ENOMEM;
		goto err;
	}

	cp = buf;
	i = 0;
	while (i < ntokens) {
		if (sscanf(cp, "%u", &new_target_loads[i++]) != 1)
			goto err_inval;

		cp = strpbrk(cp, " :");
		if (!cp)
			break;
		cp++;
	}

	if (i != ntokens)
		goto err_inval;

	spin_lock_irqsave(&target_loads_lock, flags);
	if (target_loads != default_target_loads)
		kfree(target_loads);
	target_loads = new_target_loads;
	ntarget_loads = ntokens;
	spin_unlock_irqrestore(&target_loads_lock, flags);
	return count;

err_inval:
	ret = -EINVAL;
err:
	kfree(new_target_loads);
	return ret;
}

static struct global_attr target_loads_attr =
	__ATTR(target_loads, S_IRUGO | S_IWUSR,
		show_target_loads, store_target_loads);

static ssize_t show_hispeed_freq(struct kobject *kobj,
				 struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", hispeed_freq);
}

static ssize_t store_hispeed_freq(struct kobject *kobj,
				  struct attribute *attr, const char *buf,
				  size_t count)
{
	int ret;
	long unsigned int val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	hispeed_freq = val;
	return count;
}

static struct global_attr hispeed_freq_attr = __ATTR(hispeed_freq, 0644,
		show_hispeed_freq, store_hispeed_freq);


static ssize_t show_go_hispeed_load(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", go_hispeed_load);
}

static ssize_t store_go_hispeed_load(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	go_hispeed_load = val;
	return count;
}

static struct global_attr go_hispeed_load_attr = __ATTR(go_hispeed_load, 0644,
		show_go_hispeed_load, store_go_hispeed_load);

static ssize_t show_unplug_load_cpu1(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", unplug_load[0]);
}

static ssize_t store_unplug_load_cpu1(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u\n", &val);
	if (ret < 0)
		return ret;
	unplug_load[0] = val;
        return count;
}

static struct global_attr unplug_load_cpu1_attr = __ATTR(unplug_load_cpu1, 0644,
	show_unplug_load_cpu1, store_unplug_load_cpu1);

static ssize_t show_unplug_load_cpu2(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", unplug_load[0]);
}

static ssize_t store_unplug_load_cpu2(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u\n", &val);
	if (ret < 0)
		return ret;
	unplug_load[1] = val;
        return count;
}

static struct global_attr unplug_load_cpu2_attr = __ATTR(unplug_load_cpu2, 0644,
	show_unplug_load_cpu2, store_unplug_load_cpu2);

static ssize_t show_unplug_load_cpumore(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", unplug_load[2]);
}

static ssize_t store_unplug_load_cpumore(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u\n", &val);
	if (ret < 0)
		return ret;
	unplug_load[2] = val;
        return count;
}

static struct global_attr unplug_load_cpumore_attr = __ATTR(unplug_load_cpumore, 0644,
	show_unplug_load_cpumore, store_unplug_load_cpumore);

static ssize_t show_hot_remove_sampling_periods(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", hot_remove_sampling_periods);
}

static ssize_t store_hot_remove_sampling_periods(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u\n", &val);
	if (ret < 0)
		return ret;
	hot_remove_sampling_periods = val;
	return count;
}

static struct global_attr hot_remove_sampling_periods_attr = __ATTR(hot_remove_sampling_periods, 0644,
		show_hot_remove_sampling_periods, store_hot_remove_sampling_periods);

static ssize_t show_hot_remove_sampling_periods_suspended(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", hot_remove_sampling_periods_suspended);
}

static ssize_t store_hot_remove_sampling_periods_suspended(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u\n", &val);
	if (ret < 0)
		return ret;
	hot_remove_sampling_periods_suspended = val;
	return count;
}

static struct global_attr hot_remove_sampling_periods_suspended_attr = __ATTR(hot_remove_sampling_periods_suspended, 0644,
		show_hot_remove_sampling_periods_suspended, store_hot_remove_sampling_periods_suspended);

static ssize_t show_hot_add_sampling_periods(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", hot_add_sampling_periods);
}

static ssize_t store_hot_add_sampling_periods(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u\n", &val);
	if (ret < 0)
		return ret;
	hot_add_sampling_periods = val;
	return count;
}

static struct global_attr hot_add_sampling_periods_attr = __ATTR(hot_add_sampling_periods, 0644,
		show_hot_add_sampling_periods, store_hot_add_sampling_periods);

static ssize_t show_hot_add_sampling_periods_suspended(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", hot_add_sampling_periods_suspended);
}

static ssize_t store_hot_add_sampling_periods_suspended(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val;

	ret = sscanf(buf, "%u\n", &val);
	if (ret < 0)
		return ret;
	hot_add_sampling_periods_suspended = val;
	return count;
}

static struct global_attr hot_add_sampling_periods_suspended_attr = __ATTR(hot_add_sampling_periods_suspended, 0644,
		show_hot_add_sampling_periods_suspended, store_hot_add_sampling_periods_suspended);

static ssize_t show_min_sample_time(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", min_sample_time);
}

static ssize_t store_min_sample_time(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	min_sample_time = val;
	return count;
}

static struct global_attr min_sample_time_attr = __ATTR(min_sample_time, 0644,
		show_min_sample_time, store_min_sample_time);

static ssize_t show_above_hispeed_delay(struct kobject *kobj,
					struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", above_hispeed_delay_val);
}

static ssize_t store_above_hispeed_delay(struct kobject *kobj,
					 struct attribute *attr,
					 const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	above_hispeed_delay_val = val;
	return count;
}

define_one_global_rw(above_hispeed_delay);

static ssize_t show_timer_rate(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", timer_rate);
}

static ssize_t store_timer_rate(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	timer_rate = val;
	return count;
}

static struct global_attr timer_rate_attr = __ATTR(timer_rate, 0644,
		show_timer_rate, store_timer_rate);

static ssize_t show_timer_rate_suspended(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", timer_rate_suspended);
}

static ssize_t store_timer_rate_suspended(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = strict_strtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	timer_rate_suspended = val;
	return count;
}

static struct global_attr timer_rate_suspended_attr = __ATTR(timer_rate_suspended, 0644,
		show_timer_rate_suspended, store_timer_rate_suspended);

static ssize_t show_timer_slack(
	struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", timer_slack_val);
}

static ssize_t store_timer_slack(
	struct kobject *kobj, struct attribute *attr, const char *buf,
	size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtol(buf, 10, &val);
	if (ret < 0)
		return ret;

	timer_slack_val = val;
	return count;
}

define_one_global_rw(timer_slack);

static ssize_t show_boost(struct kobject *kobj, struct attribute *attr,
			  char *buf)
{
	return sprintf(buf, "%d\n", boost_val);
}

static ssize_t store_boost(struct kobject *kobj, struct attribute *attr,
			   const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	boost_val = val;

	if (boost_val) 
		cpufreq_zenx_boost();

	return count;
}

define_one_global_rw(boost);

static ssize_t store_boostpulse(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	boostpulse_endtime = ktime_to_us(ktime_get()) + boostpulse_duration_val;
	cpufreq_zenx_boost();
	return count;
}

static struct global_attr boostpulse =
	__ATTR(boostpulse, 0200, NULL, store_boostpulse);

static ssize_t show_boostpulse_duration(
	struct kobject *kobj, struct attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", boostpulse_duration_val);
}

static ssize_t store_boostpulse_duration(
	struct kobject *kobj, struct attribute *attr, const char *buf,
	size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	boostpulse_duration_val = val;
	return count;
}

define_one_global_rw(boostpulse_duration);

static struct attribute *zenx_attributes[] = {
	&target_loads_attr.attr,
	&hispeed_freq_attr.attr,
	&go_hispeed_load_attr.attr,
	&above_hispeed_delay.attr,
	&unplug_load_cpu1_attr.attr,
	&unplug_load_cpu2_attr.attr,
	&unplug_load_cpumore_attr.attr,
	&hot_remove_sampling_periods_attr.attr,
	&hot_remove_sampling_periods_suspended_attr.attr,
	&hot_add_sampling_periods_attr.attr,
	&hot_add_sampling_periods_suspended_attr.attr,
	&min_sample_time_attr.attr,
	&timer_rate_attr.attr,
	&timer_rate_suspended_attr.attr,
	&timer_slack.attr,
	&boost.attr,
	&boostpulse.attr,
	&boostpulse_duration.attr,
	NULL,
};

static struct attribute_group zenx_attr_group = {
	.attrs = zenx_attributes,
	.name = "zenx",
};

#ifdef CONFIG_EARLYSUSPEND
static void zenx_early_suspend(struct early_suspend *handler) {
	/*
	 * Adjust parameters of the governor on suspend for
	 * power savings
	 */
	curr_hot_remove_sampling_periods = hot_remove_sampling_periods_suspended;
	curr_hot_add_sampling_periods = hot_add_sampling_periods_suspended;
	curr_timer_rate = timer_rate_suspended;
}

static void zenx_late_resume(struct early_suspend *handler) {
	/*
	 * Adjust parameters of the governor back to defaults.
	 * Using offsets allows us not to overwrite user values.
	 */
	curr_hot_remove_sampling_periods = hot_remove_sampling_periods;
	curr_hot_add_sampling_periods = hot_add_sampling_periods;
	curr_timer_rate = timer_rate;
	cpufreq_zenx_boost();
}

static struct early_suspend zenx_power_suspend = {
	.suspend = zenx_early_suspend,
	.resume = zenx_late_resume,
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1,
};
#endif

static int cpufreq_zenx_idle_notifier(struct notifier_block *nb,
					     unsigned long val,
					     void *data)
{
	switch (val) {
	case IDLE_START:
		cpufreq_zenx_idle_start();
		break;
	case IDLE_END:
		cpufreq_zenx_idle_end();
		break;
	}

	return 0;
}

static struct notifier_block cpufreq_zenx_idle_nb = {
	.notifier_call = cpufreq_zenx_idle_notifier,
};

static int cpufreq_governor_zenx(struct cpufreq_policy *policy,
		unsigned int event)
{
	int rc;
	unsigned int j;
	struct cpufreq_zenx_cpuinfo *pcpu;
	struct cpufreq_frequency_table *freq_table;

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(policy->cpu))
			return -EINVAL;

		freq_table =
			cpufreq_frequency_get_table(policy->cpu);
		if (!hispeed_freq)
			hispeed_freq = policy->max;

		for_each_cpu(j, policy->cpus) {
			unsigned long expires;

			pcpu = &per_cpu(cpuinfo, j);
			pcpu->policy = policy;
			pcpu->target_freq = policy->cur;
			pcpu->freq_table = freq_table;
			pcpu->floor_freq = pcpu->target_freq;
			pcpu->floor_validate_time =
				ktime_to_us(ktime_get());
			pcpu->hispeed_validate_time =
				pcpu->floor_validate_time;
			pcpu->nr_periods_add = 0;
			pcpu->nr_periods_remove = 0;
			pcpu->add_avg_load = 0;
			pcpu->remove_avg_load =  0;
			pcpu->last_cpu_load = 0;
			down_write(&pcpu->enable_sem);
			expires = jiffies + usecs_to_jiffies(curr_timer_rate);
			pcpu->cpu_timer.expires = expires;
			add_timer_on(&pcpu->cpu_timer, j);
			if (timer_slack_val >= 0) {
				expires += usecs_to_jiffies(timer_slack_val);
				pcpu->cpu_slack_timer.expires = expires;
				add_timer_on(&pcpu->cpu_slack_timer, j);
			}
			pcpu->governor_enabled = 1;
			up_write(&pcpu->enable_sem);
		}

		/*
		 * Do not register the idle hook and create sysfs
		 * entries if we have already done so.
		 */
		if (atomic_inc_return(&active_count) > 1)
			return 0;

		rc = sysfs_create_group(cpufreq_global_kobject,
				&zenx_attr_group);
		if (rc)
			return rc;

#ifdef CONFIG_EARLYSUSPEND
		register_early_suspend(&zenx_power_suspend);
#endif
		idle_notifier_register(&cpufreq_zenx_idle_nb);
		cpufreq_register_notifier(
			&cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
		break;

	case CPUFREQ_GOV_STOP:
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			down_write(&pcpu->enable_sem);
			pcpu->governor_enabled = 0;
			del_timer_sync(&pcpu->cpu_timer);
			del_timer_sync(&pcpu->cpu_slack_timer);
			up_write(&pcpu->enable_sem);
		}

		if (atomic_dec_return(&active_count) > 0)
			return 0;

		cpufreq_unregister_notifier(
			&cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
		idle_notifier_unregister(&cpufreq_zenx_idle_nb);
		sysfs_remove_group(cpufreq_global_kobject,
				&zenx_attr_group);

#ifdef CONFIG_EARLYSUSPEND
		unregister_early_suspend(&zenx_power_suspend);
#endif
		/*
		 * XXX BIG CAVEAT: Stopping the governor with CPU1 offline
		 * will result in it remaining offline until the user onlines
		 * it again.  It is up to the user to do this (for now).
		 */
		break;

	case CPUFREQ_GOV_LIMITS:
		if (policy->max < policy->cur)
			__cpufreq_driver_target(policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > policy->cur)
			__cpufreq_driver_target(policy,
					policy->min, CPUFREQ_RELATION_L);
		break;
	}
	return 0;
}

static void cpufreq_zenx_nop_timer(unsigned long data)
{
}

static int __init cpufreq_zenx_init(void)
{
	unsigned int i;
	struct cpufreq_zenx_cpuinfo *pcpu;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };

	/* Initalize per-cpu timers */
	for_each_possible_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);
		init_timer_deferrable(&pcpu->cpu_timer);
		pcpu->cpu_timer.function = cpufreq_zenx_timer;
		pcpu->cpu_timer.data = i;
		init_timer(&pcpu->cpu_slack_timer);
		pcpu->cpu_slack_timer.function = cpufreq_zenx_nop_timer;
		spin_lock_init(&pcpu->load_lock);
		init_rwsem(&pcpu->enable_sem);
	}

	spin_lock_init(&target_loads_lock);
	spin_lock_init(&speedchange_cpumask_lock);
	mutex_init(&set_speed_lock);

	speedchange_task =
		kthread_create(cpufreq_zenx_speedchange_task, NULL,
			       "cfzenx");
	if (IS_ERR(speedchange_task))
		return PTR_ERR(speedchange_task);

	sched_setscheduler_nocheck(speedchange_task, SCHED_FIFO, &param);
	get_task_struct(speedchange_task);

	/* NB: wake up so the thread does not look hung to the freezer */
	wake_up_process(speedchange_task);

	hotplug_add_wq = alloc_workqueue("hpaddzenx", 0, 1);
	if (!hotplug_add_wq) {
		put_task_struct(speedchange_task);
		return -ENOMEM;
	}

	INIT_WORK(&hotplug_add_work, cpufreq_zenx_hotplug_add_cpu_work);

	spin_lock_init(&hotplug_remove_cpumask_lock);

	hotplug_remove_wq = alloc_workqueue("hpremovezenx", 0, 1);
	if (!hotplug_remove_wq) {
		put_task_struct(speedchange_task);
		return -ENOMEM;
	}

	INIT_WORK(&hotplug_remove_work, cpufreq_zenx_hotplug_remove_cpu_work);

	spin_lock_init(&hotplug_remove_cpumask_lock);

	return cpufreq_register_governor(&cpufreq_gov_zenx);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_ZENX
fs_initcall(cpufreq_zenx_init);
#else
module_init(cpufreq_zenx_init);
#endif

static void __exit cpufreq_zenx_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_zenx);
	kthread_stop(speedchange_task);
	put_task_struct(speedchange_task);
	destroy_workqueue(hotplug_add_wq);
	destroy_workqueue(hotplug_remove_wq);
}

module_exit(cpufreq_zenx_exit);

MODULE_AUTHOR("Mike Chan <mike@android.com>");
MODULE_AUTHOR("Brandon Berhent <bbedward@androiddeveloperalliance.org>");
MODULE_DESCRIPTION("'cpufreq_zenx' - A cpufreq governor for "
	"Latency sensitive workloads with load-based hotplugging.");
MODULE_VERSION("2.1");
MODULE_LICENSE("GPL");
