/* Copyright (c) 2010-2011, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/rtc.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include<linux/spinlock.h>

#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/rtc.h>

#ifdef CONFIG_RTC_AUTO_PWRON
#include <linux/reboot.h>
extern int poweroff_charging;
#ifdef CONFIG_RTC_AUTO_PWRON_PARAM
#include <linux/sec_param.h>
#include <linux/param.h>
#include <linux/wakelock.h>

/* for alarm mode */
#define ALARM_MODE_NOMAL			(0x6A)
#define ALARM_MODE_BOOT_RTC			(0x7B)
#endif
#endif

/* RTC Register offsets from RTC CTRL REG */
#define PM8XXX_ALARM_CTRL_OFFSET 0x01
#define PM8XXX_RTC_WRITE_OFFSET 0x02
#define PM8XXX_RTC_READ_OFFSET 0x06
#define PM8XXX_ALARM_RW_OFFSET 0x0A

/* RTC_CTRL register bit fields */
#define PM8xxx_RTC_ENABLE	BIT(7)
#define PM8xxx_RTC_ALARM_ENABLE	BIT(1)
#define PM8xxx_RTC_ABORT_ENABLE BIT(0)

#define PM8xxx_RTC_ALARM_CLEAR  BIT(0)

#define NUM_8_BIT_RTC_REGS	0x4

/**
 * struct pm8xxx_rtc - rtc driver internal structure
 * @rtc: rtc device for this driver
 * @rtc_alarm_irq: rtc alarm irq number
 */
struct pm8xxx_rtc {
	struct rtc_device *rtc;
	int rtc_alarm_irq;
	int rtc_base;
	int rtc_read_base;
	int rtc_write_base;
	int alarm_rw_base;
	u8  ctrl_reg;
	struct device *rtc_dev;
	spinlock_t ctrl_reg_lock;
};

#if defined(CONFIG_RTC_AUTO_PWRON)
#ifdef CONFIG_RTC_AUTO_PWRON_PARAM
static struct workqueue_struct*	sapa_workq;
static struct delayed_work		sapa_load_param;
static struct wake_lock			sapa_wakelock;
#endif
static struct rtc_wkalrm		sapa_saved_time;
static int						sapa_dev_suspend;


static void print_time(char* str, struct rtc_wkalrm *alarm)
{
	pr_info( "%s: [%d] %d-%d-%d %d:%d:%d\n", str, alarm->enabled,
		alarm->time.tm_year, alarm->time.tm_mon, alarm->time.tm_mday,
		alarm->time.tm_hour, alarm->time.tm_min, alarm->time.tm_sec);
}
#endif

/*
 * The RTC registers need to be read/written one byte at a time. This is a
 * hardware limitation.
 */

static int pm8xxx_read_wrapper(struct pm8xxx_rtc *rtc_dd, u8 *rtc_val,
			int base, int count)
{
	int i, rc;
	struct device *parent = rtc_dd->rtc_dev->parent;

	for (i = 0; i < count; i++) {
		rc = pm8xxx_readb(parent, base + i, &rtc_val[i]);
		if (rc < 0) {
			dev_err(rtc_dd->rtc_dev, "PM8xxx read failed\n");
			return rc;
		}
	}

	return 0;
}

static int pm8xxx_write_wrapper(struct pm8xxx_rtc *rtc_dd, u8 *rtc_val,
			int base, int count)
{
	int i, rc;
	struct device *parent = rtc_dd->rtc_dev->parent;

	for (i = 0; i < count; i++) {
		rc = pm8xxx_writeb(parent, base + i, rtc_val[i]);
		if (rc < 0) {
			dev_err(rtc_dd->rtc_dev, "PM8xxx write failed\n");
			return rc;
		}
	}

	return 0;
}


/*
 * Steps to write the RTC registers.
 * 1. Disable alarm if enabled.
 * 2. Write 0x00 to LSB.
 * 3. Write Byte[1], Byte[2], Byte[3] then Byte[0].
 * 4. Enable alarm if disabled in step 1.
 */
static int
pm8xxx_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	int rc;
	unsigned long secs, irq_flags;
	u8 value[4], reg = 0, alarm_enabled = 0, ctrl_reg;
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);

	rtc_tm_to_time(tm, &secs);

	value[0] = secs & 0xFF;
	value[1] = (secs >> 8) & 0xFF;
	value[2] = (secs >> 16) & 0xFF;
	value[3] = (secs >> 24) & 0xFF;

	dev_info(dev, "Seconds value to be written to RTC = %lu\n", secs);

	spin_lock_irqsave(&rtc_dd->ctrl_reg_lock, irq_flags);
	ctrl_reg = rtc_dd->ctrl_reg;

	if (ctrl_reg & PM8xxx_RTC_ALARM_ENABLE) {
		alarm_enabled = 1;
		ctrl_reg &= ~PM8xxx_RTC_ALARM_ENABLE;
		rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base,
									1);
		if (rc < 0) {
			dev_err(dev, "PM8xxx write failed\n");
			goto rtc_rw_fail;
		}
	} else
		spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);

	/* Write Byte[1], Byte[2], Byte[3], Byte[0] */
	/* Write 0 to Byte[0] */
	reg = 0;
	rc = pm8xxx_write_wrapper(rtc_dd, &reg, rtc_dd->rtc_write_base, 1);
	if (rc < 0) {
		dev_err(dev, "PM8xxx write failed\n");
		goto rtc_rw_fail;
	}

	/* Write Byte[1], Byte[2], Byte[3] */
	rc = pm8xxx_write_wrapper(rtc_dd, value + 1,
					rtc_dd->rtc_write_base + 1, 3);
	if (rc < 0) {
		dev_err(dev, "Write to RTC registers failed\n");
		goto rtc_rw_fail;
	}

	/* Write Byte[0] */
	rc = pm8xxx_write_wrapper(rtc_dd, value, rtc_dd->rtc_write_base, 1);
	if (rc < 0) {
		dev_err(dev, "Write to RTC register failed\n");
		goto rtc_rw_fail;
	}

	if (alarm_enabled) {
		ctrl_reg |= PM8xxx_RTC_ALARM_ENABLE;
		rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base,
									1);
		if (rc < 0) {
			dev_err(dev, "PM8xxx write failed\n");
			goto rtc_rw_fail;
		}
	}

	rtc_dd->ctrl_reg = ctrl_reg;

#ifdef CONFIG_RTC_AUTO_PWRON
	pr_info("%s : secs = %lu, h:m:s == %d:%d:%d, d/m/y = %d/%d/%d\n", __func__,
			secs, tm->tm_hour, tm->tm_min, tm->tm_sec,
			tm->tm_mday, tm->tm_mon, tm->tm_year);
#endif
			
rtc_rw_fail:
	if (alarm_enabled)
		spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);

	return rc;
}

static int
pm8xxx_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	int rc;
	u8 value[4], reg;
	unsigned long secs;
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);

	rc = pm8xxx_read_wrapper(rtc_dd, value, rtc_dd->rtc_read_base,
							NUM_8_BIT_RTC_REGS);
	if (rc < 0) {
		dev_err(dev, "RTC time read failed\n");
		return rc;
	}

	/*
	 * Read the LSB again and check if there has been a carry over.
	 * If there is, redo the read operation.
	 */
	rc = pm8xxx_read_wrapper(rtc_dd, &reg, rtc_dd->rtc_read_base, 1);
	if (rc < 0) {
		dev_err(dev, "PM8xxx read failed\n");
		return rc;
	}

	if (unlikely(reg < value[0])) {
		rc = pm8xxx_read_wrapper(rtc_dd, value,
				rtc_dd->rtc_read_base, NUM_8_BIT_RTC_REGS);
		if (rc < 0) {
			dev_err(dev, "RTC time read failed\n");
			return rc;
		}
	}

	secs = value[0] | (value[1] << 8) | (value[2] << 16) \
						| (value[3] << 24);

	rtc_time_to_tm(secs, tm);

	rc = rtc_valid_tm(tm);
	if (rc < 0) {
		dev_err(dev, "Invalid time read from PM8xxx\n");
		return rc;
	}

	dev_dbg(dev, "secs = %lu, h:m:s == %d:%d:%d, d/m/y = %d/%d/%d\n",
			secs, tm->tm_hour, tm->tm_min, tm->tm_sec,
			tm->tm_mday, tm->tm_mon, tm->tm_year);

	return 0;
}

static int
pm8xxx_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	int rc;
	u8 value[4], ctrl_reg;
	unsigned long secs, secs_rtc, irq_flags;
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);
	struct rtc_time rtc_tm;

	rtc_tm_to_time(&alarm->time, &secs);

	/*
	 * Read the current RTC time and verify if the alarm time is in the
	 * past. If yes, return invalid.
	 */
	rc = pm8xxx_rtc_read_time(dev, &rtc_tm);
	if (rc < 0) {
		dev_err(dev, "Unamble to read RTC time\n");
		return -EINVAL;
	}

	rtc_tm_to_time(&rtc_tm, &secs_rtc);
	if (secs < secs_rtc) {
		dev_err(dev, "Trying to set alarm in the past\n");
		return -EINVAL;
	}

#ifdef CONFIG_RTC_AUTO_PWRON
	if ( sapa_saved_time.enabled ) {
		unsigned long secs_pwron;

		/* If there are power on alarm before alarm time, ignore alarm */
		rtc_tm_to_time(&sapa_saved_time.time, &secs_pwron);
		pr_info("secs_pwron=%lu, secs=%lu, rtc=%lu\n", secs_pwron, secs, secs_rtc);
		if ( secs_rtc < secs_pwron && secs_pwron < secs ) {
			pr_info("Override with SAPA\n");
			memcpy(alarm, &sapa_saved_time, sizeof(struct rtc_wkalrm));
			secs = secs_pwron;
		}
		if (  secs_pwron < secs_rtc ) {
			pr_info("SAPA was expired. Clear\n");
			sapa_saved_time.enabled = 0;
		}
	}
#endif

	value[0] = secs & 0xFF;
	value[1] = (secs >> 8) & 0xFF;
	value[2] = (secs >> 16) & 0xFF;
	value[3] = (secs >> 24) & 0xFF;

	spin_lock_irqsave(&rtc_dd->ctrl_reg_lock, irq_flags);

	rc = pm8xxx_write_wrapper(rtc_dd, value, rtc_dd->alarm_rw_base,
							NUM_8_BIT_RTC_REGS);
	if (rc < 0) {
		dev_err(dev, "Write to RTC ALARM registers failed\n");
		goto rtc_rw_fail;
	}

	ctrl_reg = rtc_dd->ctrl_reg;
	ctrl_reg = (alarm->enabled) ? (ctrl_reg | PM8xxx_RTC_ALARM_ENABLE) :
					(ctrl_reg & ~PM8xxx_RTC_ALARM_ENABLE);

	rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base, 1);
	if (rc < 0) {
		dev_err(dev, "PM8xxx write failed\n");
		goto rtc_rw_fail;
	}

	rtc_dd->ctrl_reg = ctrl_reg;

	dev_info(dev, "Alarm Set for h:r:s=%d:%d:%d, d/m/y=%d/%d/%d\n",
			alarm->time.tm_hour, alarm->time.tm_min,
			alarm->time.tm_sec, alarm->time.tm_mday,
			alarm->time.tm_mon, alarm->time.tm_year);
rtc_rw_fail:
	spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);
	return rc;
}

static int
pm8xxx_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	int rc;
	u8 value[4];
	unsigned long secs;
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);

	rc = pm8xxx_read_wrapper(rtc_dd, value, rtc_dd->alarm_rw_base,
			NUM_8_BIT_RTC_REGS);
	if (rc < 0) {
		dev_err(dev, "RTC alarm time read failed\n");
		return rc;
	}

	secs = value[0] | (value[1] << 8) | (value[2] << 16) | \
						 (value[3] << 24);

	rtc_time_to_tm(secs, &alarm->time);

	rc = rtc_valid_tm(&alarm->time);
	if (rc < 0) {
		dev_err(dev, "Invalid time read from PM8xxx\n");
		return rc;
	}

	dev_dbg(dev, "Alarm set for - h:r:s=%d:%d:%d, d/m/y=%d/%d/%d\n",
		alarm->time.tm_hour, alarm->time.tm_min,
				alarm->time.tm_sec, alarm->time.tm_mday,
				alarm->time.tm_mon, alarm->time.tm_year);

	return 0;
}


static int
pm8xxx_rtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	int rc;
	unsigned long irq_flags;
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);
	u8 ctrl_reg;

	spin_lock_irqsave(&rtc_dd->ctrl_reg_lock, irq_flags);
	ctrl_reg = rtc_dd->ctrl_reg;
	ctrl_reg = (enabled) ? (ctrl_reg | PM8xxx_RTC_ALARM_ENABLE) :
				(ctrl_reg & ~PM8xxx_RTC_ALARM_ENABLE);

	rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base, 1);
	if (rc < 0) {
		dev_err(dev, "PM8xxx write failed\n");
		goto rtc_rw_fail;
	}

	rtc_dd->ctrl_reg = ctrl_reg;

rtc_rw_fail:
	spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);
	return rc;
}

#ifdef CONFIG_RTC_AUTO_PWRON
#ifdef CONFIG_RTC_AUTO_PWRON_PARAM
static void sapa_load_kparam(struct work_struct *work)
{
	int temp1, temp2, temp3;
	unsigned long pwron_time=(unsigned long)0;

	if ( sapa_saved_time.enabled == 0 ) {
		sec_get_param(param_index_boot_alarm_set, &temp1);
		sec_get_param(param_index_boot_alarm_value_l, &temp2);
		sec_get_param(param_index_boot_alarm_value_h, &temp3);
		pwron_time = temp3<<4 | temp2;

		pr_info("%s: %x %lu\n", __func__, temp1, pwron_time);
		if ( temp1 == ALARM_MODE_BOOT_RTC ) {
			rtc_time_to_tm( pwron_time, &sapa_saved_time.time );
			sapa_saved_time.enabled = 1;
			print_time("sapa_saved_time = ", &sapa_saved_time);
		}
	}
}
#endif

static void sapa_store_kparam(struct rtc_wkalrm *alarm)
{
#ifdef CONFIG_RTC_AUTO_PWRON_PARAM
	//int temp1, temp2, temp3;
	int MSB=0, LSB=0;
	int alarm_mode = 0;
	unsigned long secs;

	if ( !sapa_workq ) {
		pr_err("%s: pwron alarm work_queue not exist\n", __func__);
		return ;
	}

	if ( alarm == &sapa_saved_time ) {
		pr_err("%s: pwr on alarm param already was written\n", __func__);
		return ;
	}

	if ( alarm->enabled ) {
		rtc_tm_to_time(&alarm->time, &secs);
		LSB = (int)secs;
		MSB = (int)(secs>>4);

		alarm_mode = ALARM_MODE_BOOT_RTC;
		sec_set_param(param_index_boot_alarm_set, &alarm_mode);
		sec_set_param(param_index_boot_alarm_value_l, &LSB);
		sec_set_param(param_index_boot_alarm_value_h, &MSB);
		pr_info("%s: [%x] [%x] [%x]\n", __func__, alarm_mode, LSB, MSB);

		#if 0 // for debugging
		sec_get_param(param_index_boot_alarm_set,&temp1);
		sec_get_param(param_index_boot_alarm_value_l, &temp2);
		sec_get_param(param_index_boot_alarm_value_h, &temp3);
		pr_info( "sec_set_param [%x] [%x] [%x] -- feedback\n", temp1, temp2, temp3);
		#endif
	}
	else {
		alarm_mode = ALARM_MODE_NOMAL;
		sec_set_param(param_index_boot_alarm_set, &alarm_mode);
		pr_info("%s: clear\n", __func__);
	}
#endif
}

static int
sapa_rtc_getalarm(struct device *dev, struct rtc_wkalrm *a)
{
	struct rtc_time b;
	int ret = 0;

	/* read boot alarm */
	if ( pm8xxx_rtc_read_time(dev, &b) ) {
		pr_err("%s: read time failed.\n", __func__);
		ret = -EINVAL;
	}
	if ( pm8xxx_rtc_read_time(dev, &(a->time)) ) {
		pr_err("%s: read alarm failed.\n", __func__);
		ret = -EINVAL;
	}

	if ( !ret ) {
		pr_info("%s: [PMIC ALARM] %d-%d-%d %d:%d:%d \n", __func__,
			a->time.tm_year, a->time.tm_mon, a->time.tm_mday,
			a->time.tm_hour, a->time.tm_min, a->time.tm_sec);
		pr_info("%s: [PMIC RTC] %d-%d-%d %d:%d:%d \n", __func__,
			b.tm_year, b.tm_mon, b.tm_mday,
			b.tm_hour, b.tm_min, b.tm_sec);
	}

	return ret;
}

static int
sapa_rtc_setalarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	//unsigned long now = get_seconds();
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);
	u8 value[4], ctrl_reg;
	unsigned long secs, secs_rtc;//, irq_flags;
	struct rtc_time rtc_tm;
	int rc;

	if (!alarm->enabled) {
		pr_info("%s: try to clear\n", __func__);

		sapa_saved_time.enabled = 0;  // disable pwr on alarm to prevent retrying
		sapa_store_kparam(alarm);

		ctrl_reg = (rtc_dd->ctrl_reg & ~PM8xxx_RTC_ALARM_ENABLE);

		rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base, 1);
		if (rc < 0) {
			pr_err("%s: PM8xxx write failed!\n", __func__);
			return rc;
		}
		rtc_dd->ctrl_reg = ctrl_reg;
	}
	else
	{
		pr_info("%s: <- %4d-%02d-%02d %02d:%02d:%02d\n", __func__,
			alarm->time.tm_year, alarm->time.tm_mon, alarm->time.tm_mday,
			alarm->time.tm_hour, alarm->time.tm_min, alarm->time.tm_sec);

		rtc_tm_to_time(&alarm->time, &secs);

		/*
		 * Read the current RTC time and verify if the alarm time is in the
		 * past. If yes, return invalid.
		 */
		rc = pm8xxx_rtc_read_time(dev, &rtc_tm);
		if (rc < 0) {
			pr_err("%s: Unamble to read RTC time\n", __func__);
			return -EINVAL;
		}

		rtc_tm_to_time(&rtc_tm, &secs_rtc);
		if (secs < secs_rtc) {
			pr_err("%s: Trying to set alarm in the past\n", __func__);
			sapa_saved_time.enabled = 0;  // disable pwr on alarm to prevent retrying
			sapa_store_kparam(alarm);
			return -EINVAL;
		}

		value[0] = secs & 0xFF;
		value[1] = (secs >> 8) & 0xFF;
		value[2] = (secs >> 16) & 0xFF;
		value[3] = (secs >> 24) & 0xFF;

		//spin_lock_irqsave(&rtc_dd->ctrl_reg_lock, irq_flags);

		rc = pm8xxx_write_wrapper(rtc_dd, value, rtc_dd->alarm_rw_base,
								NUM_8_BIT_RTC_REGS);
		if (rc < 0) {
			pr_err("%s: Write to RTC ALARM registers failed\n", __func__);
			goto rtc_rw_fail;
		}

		ctrl_reg = rtc_dd->ctrl_reg;
		ctrl_reg = (alarm->enabled) ? (ctrl_reg | PM8xxx_RTC_ALARM_ENABLE) :
						(ctrl_reg & ~PM8xxx_RTC_ALARM_ENABLE);

		rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base, 1);
		if (rc < 0) {
			pr_err("%s: PM8xxx write failed\n", __func__);
			goto rtc_rw_fail;
		}

		rtc_dd->ctrl_reg = ctrl_reg;

		if ( alarm != &sapa_saved_time ) {
			pr_info("%s: pwron alarm was updated.\n", __func__);
			memcpy(&sapa_saved_time, alarm, sizeof(struct rtc_wkalrm));
			sapa_store_kparam(alarm);
		}
	}

	/* read boot alarm */
	rc = pm8xxx_rtc_read_alarm(dev, alarm);
	if ( rc < 0 ) {
		pr_err("%s: write failed.\n", __func__);
			return rc;
	}
	pr_info("%s: -> %4d-%02d-%02d %02d:%02d:%02d\n", __func__,
			alarm->time.tm_year, alarm->time.tm_mon, alarm->time.tm_mday,
			alarm->time.tm_hour, alarm->time.tm_min, alarm->time.tm_sec);
	if ( alarm != &sapa_saved_time )
		pm8xxx_rtc_read_time(dev,&(alarm->time));

rtc_rw_fail:
	//spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);
	return rc;
}

static int pm8xxx_rtc0_resetbootalarm(struct device *dev)
{
	pr_info("%s: enable=%d\n", __func__, sapa_saved_time.enabled);
	return sapa_rtc_setalarm(dev, &sapa_saved_time);
}
#endif /*CONFIG_RTC_AUTO_PWRON*/

static struct rtc_class_ops pm8xxx_rtc_ops = {
	.read_time	= pm8xxx_rtc_read_time,
	.set_alarm	= pm8xxx_rtc_set_alarm,
	.read_alarm	= pm8xxx_rtc_read_alarm,
#ifdef CONFIG_RTC_AUTO_PWRON
	.read_bootalarm = sapa_rtc_getalarm,
	.set_bootalarm  = sapa_rtc_setalarm,
#endif /*CONFIG_RTC_AUTO_PWRON*/
	.alarm_irq_enable = pm8xxx_rtc_alarm_irq_enable,
};

static irqreturn_t pm8xxx_alarm_trigger(int irq, void *dev_id)
{
	struct pm8xxx_rtc *rtc_dd = dev_id;
	u8 ctrl_reg;
	int rc;
	unsigned long irq_flags;

#ifdef CONFIG_RTC_AUTO_PWRON
	pr_info("##### ALARM TRIGGER #####\n");
#endif
	rtc_update_irq(rtc_dd->rtc, 1, RTC_IRQF | RTC_AF);

	spin_lock_irqsave(&rtc_dd->ctrl_reg_lock, irq_flags);

	/* Clear the alarm enable bit */
	ctrl_reg = rtc_dd->ctrl_reg;
	ctrl_reg &= ~PM8xxx_RTC_ALARM_ENABLE;

	rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base, 1);
	if (rc < 0) {
		spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);
		dev_err(rtc_dd->rtc_dev, "PM8xxx write failed!\n");
		goto rtc_alarm_handled;
	}

	rtc_dd->ctrl_reg = ctrl_reg;
	spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);

	/* Clear RTC alarm register */
	rc = pm8xxx_read_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base +
						PM8XXX_ALARM_CTRL_OFFSET, 1);
	if (rc < 0) {
		dev_err(rtc_dd->rtc_dev, "PM8xxx write failed!\n");
		goto rtc_alarm_handled;
	}

	ctrl_reg &= ~PM8xxx_RTC_ALARM_CLEAR;
	rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base +
						PM8XXX_ALARM_CTRL_OFFSET, 1);
	if (rc < 0)
		dev_err(rtc_dd->rtc_dev, "PM8xxx write failed!\n");

#ifdef CONFIG_RTC_AUTO_PWRON
#ifdef CONFIG_RTC_AUTO_PWRON_PARAM
	if ( poweroff_charging && sapa_saved_time.enabled) {
		struct rtc_time now;
		struct rtc_wkalrm alarm;
		unsigned long curr_time, alarm_time, pwron_time;

		/* To wake up rtc device */
		wake_lock_timeout(&sapa_wakelock, HZ/2 );

		pm8xxx_rtc_read_time(rtc_dd->rtc_dev, &now);
		rtc_tm_to_time(&now, &curr_time);

		pm8xxx_rtc_read_alarm(rtc_dd->rtc_dev, &alarm);
		rtc_tm_to_time(&alarm.time, &alarm_time);

		rtc_tm_to_time(&sapa_saved_time.time, &pwron_time);

		dev_info(rtc_dd->rtc_dev, "curr_time: %lu\n", curr_time);
		dev_info(rtc_dd->rtc_dev, "pmic_time: %lu\n", alarm_time);
		dev_info(rtc_dd->rtc_dev, "pwrontime: %lu [%d]\n", pwron_time, sapa_saved_time.enabled);

		if ( pwron_time <= curr_time && curr_time < pwron_time+120 )  {
			dev_info(rtc_dd->rtc_dev, "%s: Restart since RTC \n", __func__);
			//machine_restart(NULL);
			kernel_restart(NULL);
			//panic("Test panic");
		}
		else {
			dev_info(rtc_dd->rtc_dev, "%s: Not power on alarm.\n", __func__);
			if (!sapa_dev_suspend)
				pm8xxx_rtc0_resetbootalarm(rtc_dd->rtc_dev);
		}
	}
#else
	if ( poweroff_charging ) {
		dev_info(rtc_dd->rtc_dev, "%s: Restart since RTC \n", __func__);
		//machine_restart(NULL);
		kernel_restart(NULL);
	}
#endif
#endif
rtc_alarm_handled:
	return IRQ_HANDLED;
}

static int __devinit pm8xxx_rtc_probe(struct platform_device *pdev)
{
	int rc;
	u8 ctrl_reg;
	bool rtc_write_enable = false;

	struct pm8xxx_rtc *rtc_dd;
	struct resource *rtc_resource;
	const struct pm8xxx_rtc_platform_data *pdata =
		pdev->dev.platform_data;

	if (pdata != NULL)
		rtc_write_enable = pdata->rtc_write_enable;

	rtc_dd = kzalloc(sizeof(*rtc_dd), GFP_KERNEL);
	if (rtc_dd == NULL) {
		dev_err(&pdev->dev, "Unable to allocate memory!\n");
		return -ENOMEM;
	}

	/* Initialise spinlock to protect RTC cntrol register */
	spin_lock_init(&rtc_dd->ctrl_reg_lock);

	rtc_dd->rtc_alarm_irq = platform_get_irq(pdev, 0);
	if (rtc_dd->rtc_alarm_irq < 0) {
		dev_err(&pdev->dev, "Alarm IRQ resource absent!\n");
		rc = -ENXIO;
		goto fail_rtc_enable;
	}

	rtc_resource = platform_get_resource_byname(pdev, IORESOURCE_IO,
							"pmic_rtc_base");
	if (!(rtc_resource && rtc_resource->start)) {
		dev_err(&pdev->dev, "RTC IO resource absent!\n");
		rc = -ENXIO;
		goto fail_rtc_enable;
	}

	rtc_dd->rtc_base = rtc_resource->start;

	/* Setup RTC register addresses */
	rtc_dd->rtc_write_base = rtc_dd->rtc_base + PM8XXX_RTC_WRITE_OFFSET;
	rtc_dd->rtc_read_base = rtc_dd->rtc_base + PM8XXX_RTC_READ_OFFSET;
	rtc_dd->alarm_rw_base = rtc_dd->rtc_base + PM8XXX_ALARM_RW_OFFSET;

	rtc_dd->rtc_dev = &(pdev->dev);

	/* Check if the RTC is on, else turn it on */
	rc = pm8xxx_read_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base, 1);
	if (rc < 0) {
		dev_err(&pdev->dev, "PM8xxx read failed!\n");
		goto fail_rtc_enable;
	}

	if (!(ctrl_reg & PM8xxx_RTC_ENABLE)) {
		ctrl_reg |= PM8xxx_RTC_ENABLE;
		rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base,
									1);
		if (rc < 0) {
			dev_err(&pdev->dev, "PM8xxx write failed!\n");
			goto fail_rtc_enable;
		}
	}

	/* Enable abort enable feature */
	ctrl_reg |= PM8xxx_RTC_ABORT_ENABLE;
	rc = pm8xxx_write_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base, 1);
	if (rc < 0) {
		dev_err(&pdev->dev, "PM8xxx write failed!\n");
		goto fail_rtc_enable;
	}

	rtc_dd->ctrl_reg = ctrl_reg;
	if (rtc_write_enable == true)
		pm8xxx_rtc_ops.set_time = pm8xxx_rtc_set_time;

	platform_set_drvdata(pdev, rtc_dd);

	/* Register the RTC device */
	rtc_dd->rtc = rtc_device_register("pm8xxx_rtc", &pdev->dev,
				&pm8xxx_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc_dd->rtc)) {
		dev_err(&pdev->dev, "%s: RTC registration failed (%ld)\n",
					__func__, PTR_ERR(rtc_dd->rtc));
		rc = PTR_ERR(rtc_dd->rtc);
		goto fail_rtc_enable;
	}

	/* Request the alarm IRQ */
	rc = request_any_context_irq(rtc_dd->rtc_alarm_irq,
				 pm8xxx_alarm_trigger, IRQF_TRIGGER_RISING,
				 "pm8xxx_rtc_alarm", rtc_dd);
	if (rc < 0) {
		dev_err(&pdev->dev, "Request IRQ failed (%d)\n", rc);
		goto fail_req_irq;
	}

#ifdef CONFIG_RTC_AUTO_PWRON_PARAM
	sapa_workq = create_singlethread_workqueue("pwron_alarm_resume");
	if (sapa_workq == NULL) {
		dev_err(&pdev->dev, "pwron_alarm work creating failed (%d)\n", rc);
	}

	wake_lock_init(&sapa_wakelock, WAKE_LOCK_SUSPEND, "alarm_trigger");
#endif

	device_init_wakeup(&pdev->dev, 1);

	dev_dbg(&pdev->dev, "Probe success !!\n");

#ifdef CONFIG_RTC_AUTO_PWRON_PARAM
	/* To read saved power on alarm time */
	if ( poweroff_charging ) {
		INIT_DELAYED_WORK(&sapa_load_param, sapa_load_kparam);
		queue_delayed_work(sapa_workq, &sapa_load_param, (10*HZ));
	}
#endif

	return 0;

fail_req_irq:
	rtc_device_unregister(rtc_dd->rtc);
fail_rtc_enable:
	platform_set_drvdata(pdev, NULL);
	kfree(rtc_dd);
	return rc;
}

#ifdef CONFIG_PM
static int pm8xxx_rtc_resume(struct device *dev)
{
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		disable_irq_wake(rtc_dd->rtc_alarm_irq);
#if defined(CONFIG_RTC_AUTO_PWRON)
	sapa_dev_suspend = 0;
	pm8xxx_rtc0_resetbootalarm(dev);
#endif

	return 0;
}

static int pm8xxx_rtc_suspend(struct device *dev)
{
	struct pm8xxx_rtc *rtc_dd = dev_get_drvdata(dev);

	if (device_may_wakeup(dev))
		enable_irq_wake(rtc_dd->rtc_alarm_irq);
#ifdef CONFIG_RTC_AUTO_PWRON
	sapa_dev_suspend = 1;
#endif

	return 0;
}

static const struct dev_pm_ops pm8xxx_rtc_pm_ops = {
	.suspend = pm8xxx_rtc_suspend,
	.resume = pm8xxx_rtc_resume,
};
#endif
static int __devexit pm8xxx_rtc_remove(struct platform_device *pdev)
{
	struct pm8xxx_rtc *rtc_dd = platform_get_drvdata(pdev);

#ifdef CONFIG_RTC_AUTO_PWRON_PARAM
	destroy_workqueue(sapa_workq);
#endif
	device_init_wakeup(&pdev->dev, 0);
	free_irq(rtc_dd->rtc_alarm_irq, rtc_dd);
	rtc_device_unregister(rtc_dd->rtc);
	platform_set_drvdata(pdev, NULL);
	kfree(rtc_dd);

	return 0;
}

#ifdef CONFIG_RTC_AUTO_PWRON
static void pm8xxx_rtc_shutdown(struct platform_device *pdev)
{
	u8 value[4] = {0, 0, 0, 0};
	struct pm8xxx_rtc *rtc_dd = platform_get_drvdata(pdev);
	unsigned long secs;
	u8 ctrl_reg;
	int rc;

	pm8xxx_rtc0_resetbootalarm(&pdev->dev);

	/* Check if the RTC is on, else turn it on */
	rc = pm8xxx_read_wrapper(rtc_dd, &ctrl_reg, rtc_dd->rtc_base, 1);
	if (rc < 0) {
		dev_err(&pdev->dev, "PM8xxx read failed!\n");
	}

	rc = pm8xxx_read_wrapper(rtc_dd, value, rtc_dd->rtc_read_base,
							NUM_8_BIT_RTC_REGS);

	secs = value[0] | (value[1] << 8) | (value[2] << 16) \
						| (value[3] << 24);
	pr_info("%s : secs = %lu\n", __func__,secs);
	pr_info("%s RTC Register : %d \n", __func__, ctrl_reg);

#ifdef CONFIG_RTC_AUTO_PWRON_PARAM
	wake_lock_destroy(&sapa_wakelock);
#endif
}
#else
static void pm8xxx_rtc_shutdown(struct platform_device *pdev)
{
	u8 value[4] = {0, 0, 0, 0};
	u8 reg;
	int rc;
	unsigned long irq_flags;
	bool rtc_alarm_powerup = false;
	struct pm8xxx_rtc *rtc_dd = platform_get_drvdata(pdev);
	struct pm8xxx_rtc_platform_data *pdata = pdev->dev.platform_data;

	if (pdata != NULL)
		rtc_alarm_powerup =  pdata->rtc_alarm_powerup;

	if (!rtc_alarm_powerup) {

		spin_lock_irqsave(&rtc_dd->ctrl_reg_lock, irq_flags);
		dev_dbg(&pdev->dev, "Disabling alarm interrupts\n");

		/* Disable RTC alarms */
		reg = rtc_dd->ctrl_reg;
		reg &= ~PM8xxx_RTC_ALARM_ENABLE;
		rc = pm8xxx_write_wrapper(rtc_dd, &reg, rtc_dd->rtc_base, 1);
		if (rc < 0) {
			dev_err(rtc_dd->rtc_dev, "PM8xxx write failed\n");
			goto fail_alarm_disable;
		}

		/* Clear Alarm register */
		rc = pm8xxx_write_wrapper(rtc_dd, value,
				rtc_dd->alarm_rw_base, NUM_8_BIT_RTC_REGS);
		if (rc < 0)
			dev_err(rtc_dd->rtc_dev, "PM8xxx write failed\n");
		
fail_alarm_disable:
		spin_unlock_irqrestore(&rtc_dd->ctrl_reg_lock, irq_flags);
	}
}
#endif /* CONFIG_RTC_AUTO_PWRON */

static struct platform_driver pm8xxx_rtc_driver = {
	.probe		= pm8xxx_rtc_probe,
	.remove		= __devexit_p(pm8xxx_rtc_remove),
	.shutdown	= pm8xxx_rtc_shutdown,
	.driver	= {
		.name	= PM8XXX_RTC_DEV_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
		.pm	= &pm8xxx_rtc_pm_ops,
#endif
	},
};

static int __init pm8xxx_rtc_init(void)
{
	return platform_driver_register(&pm8xxx_rtc_driver);
}
module_init(pm8xxx_rtc_init);

static void __exit pm8xxx_rtc_exit(void)
{
	platform_driver_unregister(&pm8xxx_rtc_driver);
}
module_exit(pm8xxx_rtc_exit);

MODULE_ALIAS("platform:rtc-pm8xxx");
MODULE_DESCRIPTION("PMIC8xxx RTC driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Anirudh Ghayal <aghayal@codeaurora.org>");
