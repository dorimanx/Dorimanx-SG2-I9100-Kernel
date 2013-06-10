/*
 * cypress_touchkey.c - Platform data for cypress touchkey driver
 *
 * Copyright (C) 2011 Samsung Electronics
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#define SEC_TOUCHKEY_DEBUG

#include <linux/module.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/earlysuspend.h>
#include <linux/i2c/cypress_touchkey.h>
#include "cypress_tkey_fw.h"
#include <linux/regulator/consumer.h>
#include <linux/mutex.h>
#include <linux/workqueue.h>
#include <linux/leds.h>
#include <asm/mach-types.h>
#include "issp_extern.h"
#include <linux/mfd/pm8xxx/pm8921.h>
#include "../../../../arch/arm/mach-msm/board-8064.h"

#define CYPRESS_GEN		0X00
#define CYPRESS_FW_VER		0X01
#define CYPRESS_MODULE_VER	0X02
#define CYPRESS_2ND_HOST	0X03
#define CYPRESS_THRESHOLD	0X04
#define CYPRESS_AUTO_CAL_FLG	0X05

#define CYPRESS_IDAC_MENU	0X07
#define CYPRESS_IDAC_BACK	0X06
#define CYPRESS_IDAC_HOME	0X08

#define CYPRESS_DIFF_MENU	0x0C
#define CYPRESS_DIFF_BACK	0x0A
#define CYPRESS_DIFF_HOME	0x0E

#define CYPRESS_RAW_DATA_MENU	0x10
#define CYPRESS_RAW_DATA_BACK	0x0E
#define CYPRESS_RAW_DATA_HOME	0x12
#define CYPRESS_RAW_DATA_BACK_GOGH	0x14

#define CYPRESS_LED_ON		0X10
#define CYPRESS_LED_OFF		0X20
#define CYPRESS_DATA_UPDATE	0X40
#define CYPRESS_AUTO_CAL	0X50
#define CYPRESS_LED_CONTROL_ON	0X60
#define CYPRESS_LED_CONTROL_OFF	0X70
#define CYPRESS_SLEEP		0X80

#define CYPRESS_FW_ID_REG	0X05
/*
#define CYPRESS_55_IC_MASK	0x20
#define CYPRESS_65_IC_MASK	0x04
*/

u32 ic_fw_id;
EXPORT_SYMBOL(ic_fw_id);

#if defined(CONFIG_GLOVE_TOUCH)
static int glove_value;
#endif

static int vol_mv_level = 33;

#define KEYCODE_REG		0x00


#define TOUCHKEY_BACKLIGHT	"button-backlight"


/* bit masks*/
#define PRESS_BIT_MASK		0X08
#define KEYCODE_BIT_MASK	0X07

#define TOUCHKEY_LOG(k, v) dev_notice(&info->client->dev, "key[%d] %d\n", k, v);
#define FUNC_CALLED dev_notice(&info->client->dev, "%s: called.\n", __func__);

#define NUM_OF_RETRY_UPDATE	3
#define NUM_OF_KEY		4

struct cypress_touchkey_info {
	struct i2c_client			*client;
	struct cypress_touchkey_platform_data	*pdata;
	struct input_dev			*input_dev;
	struct early_suspend			early_suspend;
	char			phys[32];
	unsigned char			keycode[NUM_OF_KEY];
	u8			sensitivity[NUM_OF_KEY];
	int			irq;
	u8			fw_ver;
	void (*power_onoff)(int);
	int			touchkey_update_status;
	struct led_classdev			leds;
	enum led_brightness			brightness;
	struct mutex			touchkey_led_mutex;
	struct workqueue_struct			*led_wq;
	struct work_struct			led_work;

#if defined(CONFIG_GLOVE_TOUCH)
	struct workqueue_struct		*glove_wq;
	struct work_struct		glove_work;
#endif

	bool is_powering_on;
	bool enabled;
	bool done_ta_setting;

#ifdef TKEY_FLIP_MODE
	bool enabled_flip;
#endif

#ifdef TSP_BOOSTER
	struct delayed_work	work_dvfs_off;
	struct delayed_work	work_dvfs_chg;
	bool dvfs_lock_status;
	struct mutex		dvfs_lock;
#endif

#ifdef TK_INFORM_CHARGER
	struct touchkey_callbacks callbacks;
	bool charging_mode;
#endif


};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cypress_touchkey_early_suspend(struct early_suspend *h);
static void cypress_touchkey_late_resume(struct early_suspend *h);
#endif


static int touchkey_led_status;
static int touchled_cmd_reversed;

static struct pm_gpio tkey_int = {
	.direction	= PM_GPIO_DIR_IN,
	.pull		= PM_GPIO_PULL_NO,
	.vin_sel	= 2,
	.function	= PM_GPIO_FUNC_NORMAL,
	.inv_int_pol	= 0,
};

static struct pm_gpio tkey_sleep_int = {
	.direction	= PM_GPIO_DIR_IN,
	.pull		= PM_GPIO_PULL_UP_30,
	.vin_sel	= 2,
	.function	= PM_GPIO_FUNC_NORMAL,
	.inv_int_pol	= 0,
};

static void cypress_int_gpio_setting(bool value)
{
	int ret;

	if (value) {
		ret = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_TKEY_INT),
			&tkey_int);
		if (ret)
			printk(KERN_ERR "%s PMIC_GPIO_TKEY_INT config failed\n", __func__);

		printk(KERN_DEBUG "%s in resume \n", __func__);
		}
	else {
		ret = pm8xxx_gpio_config(PM8921_GPIO_PM_TO_SYS(PMIC_GPIO_TKEY_INT),
			&tkey_sleep_int);
		if (ret)
			printk(KERN_ERR "%s PMIC_GPIO_TKEY_INT_SLEEP config failed\n", __func__);

		printk(KERN_DEBUG "%s in suspend \n", __func__);
		}

}

static void cypress_gpio_setting(bool value)
{
	int ret;

#if defined(CONFIG_MACH_JF_ATT) || defined(CONFIG_MACH_JF_TMO) || defined(CONFIG_MACH_JF_EUR)
	if(value) {
		if (system_rev < 9) {
			ret = gpio_request(GPIO_TOUCHKEY_SCL, "TKEY_SCL");
			if (ret)
				printk(KERN_ERR "%s: request GPIO %s err %d.",\
						__func__, "TKEY_SCL", ret);
			}
		else {
			ret = gpio_request(GPIO_TOUCHKEY_SCL_2, "TKEY_SCL");
			if (ret)
				printk(KERN_ERR "%s: request GPIO %s err %d.",\
						__func__, "TKEY_SCL_2", ret);
			}

			ret = gpio_request(GPIO_TOUCHKEY_SDA, "TKEY_SDA");
			if (ret)
				printk(KERN_ERR "%s: request GPIO %s err %d.",\
						__func__, "TKEY_SDA", ret);

			printk(KERN_DEBUG "%s in resume \n",__func__);
		}
	else {
		if (system_rev < 9) {
			gpio_free(GPIO_TOUCHKEY_SCL);
			}
		else {
			gpio_free(GPIO_TOUCHKEY_SCL_2);
			}

			gpio_free(GPIO_TOUCHKEY_SDA);

			printk(KERN_DEBUG "%s in suspend \n",__func__);
		}
#else /*VZW, SPR, USC, CRI*/
	if(value) {
		if (system_rev < 10) {
			ret = gpio_request(GPIO_TOUCHKEY_SCL, "TKEY_SCL");
			if (ret)
				printk(KERN_ERR "%s: request GPIO %s err %d.",\
						__func__, "TKEY_SCL", ret);
			}
		else {
			ret = gpio_request(GPIO_TOUCHKEY_SCL_2, "TKEY_SCL");
			if (ret)
				printk(KERN_ERR "%s: request GPIO %s err %d.",\
						__func__, "TKEY_SCL_2", ret);
			}

			ret = gpio_request(GPIO_TOUCHKEY_SDA, "TKEY_SDA");
			if (ret)
				printk(KERN_ERR "%s: request GPIO %s err %d.",\
						__func__, "TKEY_SDA", ret);

			printk(KERN_DEBUG "%s in resume \n",__func__);
		}
	else {
		if (system_rev < 10) {
			gpio_free(GPIO_TOUCHKEY_SCL);
			}
		else {
			gpio_free(GPIO_TOUCHKEY_SCL_2);
			}

			gpio_free(GPIO_TOUCHKEY_SDA);

			printk(KERN_DEBUG "%s in suspend \n",__func__);
		}
#endif
}


#ifdef TSP_BOOSTER
static void cypress_change_dvfs_lock(struct work_struct *work)
{
	struct cypress_touchkey_info *info =
		container_of(work,
			struct cypress_touchkey_info, work_dvfs_chg.work);
	int retval;

	mutex_lock(&info->dvfs_lock);
	retval = set_freq_limit(DVFS_TOUCH_ID,
			MIN_TOUCH_LIMIT_SECOND);
	if (retval < 0)
		dev_err(&info->client->dev,
			"%s: booster change failed(%d).\n",
			__func__, retval);
	mutex_unlock(&info->dvfs_lock);
}

static void cypress_set_dvfs_off(struct work_struct *work)
{
	struct cypress_touchkey_info *info =
		container_of(work,
			struct cypress_touchkey_info, work_dvfs_off.work);
	int retval;

	mutex_lock(&info->dvfs_lock);
	retval = set_freq_limit(DVFS_TOUCH_ID, -1);
	if (retval < 0)
		dev_err(&info->client->dev,
			"%s: booster stop failed(%d).\n",
			__func__, retval);

	info->dvfs_lock_status = false;
	mutex_unlock(&info->dvfs_lock);
}

static void cypress_set_dvfs_lock(struct cypress_touchkey_info *info,
					uint32_t on)
{
	int ret = 0;

	mutex_lock(&info->dvfs_lock);
	if (on == 0) {
		if (info->dvfs_lock_status) {
			schedule_delayed_work(&info->work_dvfs_off,
				msecs_to_jiffies(TOUCH_BOOSTER_OFF_TIME));
		}
	} else if (on == 1) {
		cancel_delayed_work(&info->work_dvfs_off);
		if (!info->dvfs_lock_status) {
			ret = set_freq_limit(DVFS_TOUCH_ID,
					MIN_TOUCH_LIMIT);
			if (ret < 0)
				dev_err(&info->client->dev,
					"%s: cpu first lock failed(%d)\n",
					__func__, ret);

			schedule_delayed_work(&info->work_dvfs_chg,
				msecs_to_jiffies(TOUCH_BOOSTER_CHG_TIME));
			info->dvfs_lock_status = true;
		}
	} else if (on == 2) {
		if (info->dvfs_lock_status) {
			cancel_delayed_work(&info->work_dvfs_off);
			cancel_delayed_work(&info->work_dvfs_chg);
			schedule_work(&info->work_dvfs_off.work);
		}
	}
	mutex_unlock(&info->dvfs_lock);
}


static void cypress_init_dvfs(struct cypress_touchkey_info *info)
{
	mutex_init(&info->dvfs_lock);

	INIT_DELAYED_WORK(&info->work_dvfs_off, cypress_set_dvfs_off);
	INIT_DELAYED_WORK(&info->work_dvfs_chg, cypress_change_dvfs_lock);

	info->dvfs_lock_status = false;
}
#endif

static int i2c_touchkey_read(struct i2c_client *client,
		u8 reg, u8 *val, unsigned int len)
{
	int err = 0;
	int retry = 3;

	while (retry--) {
		err = i2c_smbus_read_i2c_block_data(client,
				KEYCODE_REG, len, val);
		if (err >= 0)
			return 0;
		printk(KERN_ERR "[TouchKey] %s %d i2c transfer error\n",
		       __func__, __LINE__);
		msleep(20);
	}
	return err;

}


static int i2c_touchkey_write(struct i2c_client *client,
		u8 *val, unsigned int len)
{
	int err = 0;
	int retry = 3;

	while (retry--) {
		err = i2c_smbus_write_i2c_block_data(client,
				KEYCODE_REG, len, val);
		if (err >= 0)
			return 0;

		printk(KERN_DEBUG "[TouchKey] %s %d i2c transfer error\n",
		       __func__, __LINE__);
		msleep(20);
	}
	return err;
}

static void cypress_touchkey_led_work(struct work_struct *work)
{
	struct cypress_touchkey_info *info =
		container_of(work, struct cypress_touchkey_info, led_work);
	u8 buf;
	int ret;

	if(info->is_powering_on == true)
		touchled_cmd_reversed = 1;
	msleep(30);

	if (info->brightness == LED_OFF)
		buf = CYPRESS_LED_OFF;
	else
		buf = CYPRESS_LED_ON;

	printk(KERN_DEBUG "[TouchKey] %s:0x%2X \n",__func__,buf);

	if ((!info->enabled)) {
		touchled_cmd_reversed = 1;
		touchkey_led_status = buf;
		printk(KERN_ERR "[TouchKey] %s %d Touchkey is not enabled.\n",
			__func__, __LINE__);
		return;
		}

	mutex_lock(&info->touchkey_led_mutex);

	ret = i2c_smbus_write_byte_data(info->client, CYPRESS_GEN, buf);
	if (ret < 0)
		touchled_cmd_reversed = 1;

	touchkey_led_status = buf;

	mutex_unlock(&info->touchkey_led_mutex);
}

static void cypress_touchkey_brightness_set(struct led_classdev *led_cdev,
			enum led_brightness brightness)
{
	/* Must not sleep, use a workqueue if needed */
	struct cypress_touchkey_info *info =
		container_of(led_cdev, struct cypress_touchkey_info, leds);

	info->brightness = brightness;

	queue_work(info->led_wq, &info->led_work);
}

static void change_touch_key_led_voltage(int vol_mv)
{
	struct regulator *tled_regulator;
	int ret;
	vol_mv_level = vol_mv;

	tled_regulator = regulator_get(NULL, "8921_l10");
	if (IS_ERR(tled_regulator)) {
		pr_err("%s: failed to get resource %s\n", __func__,
			"touch_led");
		return;
	}
	ret = regulator_set_voltage(tled_regulator,
		vol_mv * 100000, vol_mv * 100000);
	if (ret)
		printk(KERN_ERR"error setting voltage\n");
	regulator_put(tled_regulator);
}

static ssize_t brightness_control(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	int data;

	if (sscanf(buf, "%d\n", &data) == 1) {
		printk(KERN_ERR "[TouchKey] touch_led_brightness: %d\n", data);
		change_touch_key_led_voltage(data);
	} else {
		printk(KERN_ERR "[TouchKey] touch_led_brightness Error\n");
	}
	return size;
}

static ssize_t brightness_level_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int count;

	count = snprintf(buf, sizeof(buf), "%d\n", vol_mv_level);

	printk(KERN_DEBUG "[TouchKey] Touch LED voltage = %d\n", vol_mv_level);
	return count;
}

#if defined(TK_INFORM_CHARGER)
static int touchkey_ta_setting(struct cypress_touchkey_info *info)
{
	u8 data[6] = { 0, };
	int count = 0;
	int ret = 0;
	unsigned short retry = 0;

	dev_info(&info->client->dev, "%s\n", __func__);

	while (retry < 3) {
		ret = i2c_touchkey_read(info->client, KEYCODE_REG, data, 4);
		if (ret < 0) {
			dev_err(&info->client->dev, "Failed to read Keycode_reg.\n");
			return ret;
		}

		/* Send autocal Command */

		if (info->charging_mode) {
			dev_info(&info->client->dev, "TA connect!!!\n");
			data[0] = 0x90;
			data[3] = 0x10;
		} else {
			dev_info(&info->client->dev, "TA disconnect!!!\n");
			data[0] = 0x90;
			data[3] = 0x20;
		}

		count = i2c_touchkey_write(info->client, data, 4);

		msleep(100);

		/* Check autocal status */
		ret = i2c_touchkey_read(info->client, KEYCODE_REG, data, 6);

		if (info->charging_mode) {
			if (data[5] & TK_BIT_TA_ON) {
				dev_info(&info->client->dev, "TA Enabled\n");
				break;
			} else {
				dev_dbg(&info->client->dev, "TA Enabled error, retry %d\n",
					retry);
			}
		} else {
			if (!(data[5] & TK_BIT_TA_ON)) {
				dev_info(&info->client->dev, "TA Disabled\n");
				break;
			} else {
				dev_dbg(&info->client->dev, "TA Disabled error, retry %d\n",
					retry);
			}
		}
		retry = retry + 1;
	}

	if (retry == 3)
		dev_info(&info->client->dev, "TA mode setting failed\n");

	return count;

}

static void touchkey_ta_cb(struct touchkey_callbacks *cb, bool ta_status)
{
	struct cypress_touchkey_info *info =
			container_of(cb, struct cypress_touchkey_info, callbacks);
	struct i2c_client *client = info->client;

	dev_info(&client->dev, "%s : 0x%02x\n", __func__, ic_fw_id);

	info->charging_mode = ta_status;

	if (!(info->enabled)) {
		printk(KERN_ERR "[TouchKey] %s %d Touchkey is not enabled.\n",
			__func__, __LINE__);
		info->done_ta_setting = false;
		return;
		}

	if (ic_fw_id & CYPRESS_55_IC_MASK) {
		printk(KERN_INFO "[Touchkey] IC id 20055\n");
		touchkey_ta_setting(info);
		}
	else if (ic_fw_id & CYPRESS_65_IC_MASK) {
		printk(KERN_INFO "[Touchkey] IC id 20065\n");
		touchkey_ta_setting(info);
		}
	else {
		printk(KERN_INFO "[Touchkey] IC id 20045\n");
		printk(KERN_INFO "[TouchKey] TA mode does not support!\n");
		}
}
#endif

#if defined(CONFIG_GLOVE_TOUCH)
static void cypress_touchkey_glove_work(struct work_struct *work)
{
	struct cypress_touchkey_info *info =
		container_of(work, struct cypress_touchkey_info, glove_work);

	u8 data[6] = { 0, };
	int count = 0;
	int ret = 0;
	unsigned short retry = 0;

#ifdef TKEY_FLIP_MODE
	if (info->enabled_flip) {
		dev_info(&info->client->dev,"[Touchkey] Enabled flip cover mode.\n");
		return;
	}
#endif
	if(ic_fw_id & CYPRESS_55_IC_MASK)
		printk(KERN_INFO "[Touchkey] IC id 20055\n");
	else if (ic_fw_id & CYPRESS_65_IC_MASK)
		printk(KERN_INFO "[Touchkey] IC id 20065\n");
	else {
		printk(KERN_INFO "[Touchkey] IC id 20045\n");
		printk(KERN_INFO "[TouchKey] Glovemode does not support!\n");
		return;
		}

	while (retry < 3) {
		ret = i2c_touchkey_read(info->client, CYPRESS_GEN, data, 4);
		if (ret < 0) {
			dev_err(&info->client->dev, "[TouchKey] Failed to read Keycode_reg.\n");
			return;
		}

		if (glove_value == 1) {
		/* Send glove Command */
			data[0] = 0xA0;
			data[3] = 0x30;
		} else {
			data[0] = 0xA0;
			data[3] = 0x40;
		}

		count = i2c_touchkey_write(info->client, data, 4);

		msleep(50);

		/* Check autocal status */
		ret = i2c_touchkey_read(info->client, CYPRESS_GEN, data, 6);

		if (glove_value == 1) {
			if (data[5] & TK_BIT_GLOVE) {
				printk(KERN_DEBUG "[TouchKey] glove_mode Enabled\n");
				break;
			} else {
				printk(KERN_DEBUG "[TouchKey] glove_mode Error, retry %d\n",
					retry);
			}
		} else {
			if (!(data[5] & TK_BIT_GLOVE)) {
				printk(KERN_DEBUG "[TouchKey] normal_mode Enabled\n");
				break;
			} else {
				printk(KERN_DEBUG "[TouchKey] normal_mode Error, retry %d\n",
					retry);
			}
		}
		retry = retry + 1;
	}

	if (retry == 3)
		printk(KERN_ERR "[TouchKey] glove_mode failed\n");

	return;
}

struct cypress_touchkey_info *tkey_info;

int touchkey_glovemode(int value)
{

	struct cypress_touchkey_info *info = tkey_info;

	if (!(info->enabled)) {
		printk(KERN_ERR "[TouchKey] %s %d Touchkey is not enabled.\n",
			__func__, __LINE__);
		return 0;
		}

	glove_value = value;
	queue_work(info->glove_wq, &info->glove_work);

	return 0;
}
EXPORT_SYMBOL(touchkey_glovemode);
#endif

#ifdef TKEY_FLIP_MODE
void touchkey_flip_cover(int value)
{
	struct cypress_touchkey_info *info = tkey_info;
	u8 data[6] = { 0, };
	int count = 0;
	int ret = 0;
	unsigned short retry = 0;

	if (!(info->enabled)) {
		printk(KERN_ERR "[TouchKey] %s %d Touchkey is not enabled.\n",
			__func__, __LINE__);
		return ;
		}

	if(ic_fw_id & CYPRESS_65_IC_MASK)
		printk(KERN_INFO "[Touchkey] IC id 20065\n");
	else if (ic_fw_id & CYPRESS_55_IC_MASK) {
		printk(KERN_INFO "[Touchkey] IC id 20055\n");
		printk(KERN_INFO "[TouchKey] flipmode does not support!\n");
		return;
		}
	else {
		printk(KERN_INFO "[Touchkey] IC id 20045\n");
		printk(KERN_INFO "[TouchKey] flipmode does not support!\n");
		return;
		}

	while (retry < 3) {
		ret = i2c_touchkey_read(info->client, KEYCODE_REG, data, 4);
		if (ret < 0) {
			dev_err(&info->client->dev, "[Touchkey] Failed to read Keycode_reg %d times.\n",
				retry);
			return;
		}

		if (value == 1) {
			/* Send filp mode Command */
				data[0] = 0xA0;
				data[3] = 0x50;
		} else {
				data[0] = 0xA0;
				data[3] = 0x40;
		}

		count = i2c_touchkey_write(info->client, data, 4);

		msleep(100);

		/* Check autocal status */
		ret = i2c_touchkey_read(info->client, KEYCODE_REG, data, 6);

		if (value == 1){
			if (data[5] & TK_BIT_FLIP) {
				dev_info(&info->client->dev, "[Touchkey] flip_mode Enabled\n");
				info->enabled_flip = true;
				break;
			} else {
				dev_info(&info->client->dev, "[Touchkey] flip_mode Enabled, retry %d \n",
					retry);
			}
		} else {
			if (!(data[5] & TK_BIT_FLIP)) {
				dev_info(&info->client->dev, "[Touchkey] normal_mode Enabled from flipmode\n");
				info->enabled_flip = false;
				break;
			} else {
				dev_info(&info->client->dev, "[Touchkey] normal_mode Enabled from flipmode, retry %d \n",
					retry);
			}
		}
		retry = retry + 1;
	}

	if (retry == 3)
		dev_info(&info->client->dev, "[Touchkey] flip cover failed\n");

	return;
}
#endif

static irqreturn_t cypress_touchkey_interrupt(int irq, void *dev_id)
{
	struct cypress_touchkey_info *info = dev_id;
	s8 buf[10] = {0,};
	int code;
	int press;
	int ret;

	ret = gpio_get_value(info->pdata->gpio_int);
	if (ret) {
		printk(KERN_ERR "not real interrupt (%d).\n", ret);
		goto out;
	}

	if (info->is_powering_on) {
		printk(KERN_ERR "%s: ignoring spurious boot "\
					"interrupt\n", __func__);
		return IRQ_HANDLED;
	}

	buf[0] = i2c_smbus_read_byte_data(info->client, CYPRESS_GEN);
	if (buf[0] < 0) {
		printk(KERN_ERR  "interrupt failed with %d.\n", ret);
		goto out;
	}

	press = !(buf[0] & PRESS_BIT_MASK);
	code = (int)(buf[0] & KEYCODE_BIT_MASK) - 1;
	printk(KERN_ERR
		"[TouchKey]press=%d, code=%d\n", press, code);

	if (code < 0) {
		dev_err(&info->client->dev,
				"not profer interrupt 0x%2X.\n", buf[0]);
		goto out;
	}

#if defined(SEC_TOUCHKEY_DEBUG)
	TOUCHKEY_LOG(info->keycode[code], press);
#endif

	input_report_key(info->input_dev, info->keycode[code], press);
	input_sync(info->input_dev);

#ifdef TSP_BOOSTER
	cypress_set_dvfs_lock(info, !!press);
#endif

out:
	return IRQ_HANDLED;
}

static void cypress_touchkey_con_hw(struct cypress_touchkey_info *dev_info,
								bool flag)
{
	struct cypress_touchkey_info *info =  dev_info;

	gpio_set_value(info->pdata->gpio_led_en, flag ? 1 : 0);

#if defined(SEC_TOUCHKEY_DEBUG)
	dev_info(&info->client->dev,
			"%s : called with flag %d.\n", __func__, flag);
#endif
}


static int cypress_touchkey_auto_cal(struct cypress_touchkey_info *dev_info)
{
	struct cypress_touchkey_info *info = dev_info;
	u8 data[6] = { 0, };
	int count = 0;
	int ret = 0;
	unsigned short retry = 0;
	while (retry < 3) {

		ret = i2c_touchkey_read(info->client, CYPRESS_GEN, data, 4);
		if (ret < 0) {
			printk(KERN_ERR "[TouchKey]i2c read fail.\n");
			return ret;
		}
		data[0] = 0x50;
		data[3] = 0x01;

		count = i2c_touchkey_write(info->client, data, 4);
		printk(KERN_DEBUG
				"[TouchKey] data[0]=%x data[1]=%x data[2]=%x data[3]=%x\n",
				data[0], data[1], data[2], data[3]);

		msleep(130);

		ret = i2c_touchkey_read(info->client, CYPRESS_GEN, data, 6);

		if ((data[5] & 0x80)) {
			printk(KERN_DEBUG "[Touchkey] autocal Enabled\n");
			break;
		} else {
			printk(KERN_DEBUG "[Touchkey] autocal disabled, retry %d\n",
					retry);
		}
		retry = retry + 1;
	}

	if (retry == 3)
		printk(KERN_DEBUG "[Touchkey] autocal failed\n");

	return count;
}

static int cypress_touchkey_led_on(struct cypress_touchkey_info *dev_info)
{
	struct cypress_touchkey_info *info = dev_info;
	u8 buf = CYPRESS_LED_ON;

	int ret;
	ret = i2c_smbus_write_byte_data(info->client, CYPRESS_GEN, buf);
	if (ret < 0) {
		dev_err(&info->client->dev,
				"[Touchkey] i2c write error [%d]\n", ret);
	}
	return ret;
}

static int cypress_touchkey_led_off(struct cypress_touchkey_info *dev_info)
{
	struct cypress_touchkey_info *info = dev_info;
	u8 buf = CYPRESS_LED_OFF;

	int ret;
	ret = i2c_smbus_write_byte_data(info->client, CYPRESS_GEN, buf);
	if (ret < 0) {
		dev_err(&info->client->dev,
				"[Touchkey] i2c write error [%d]\n", ret);
	}
	return ret;
}

static ssize_t touch_version_read(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	u8 data;
	int count;

	data = i2c_smbus_read_byte_data(info->client, CYPRESS_FW_VER);
	count = snprintf(buf, 20, "0x%02x\n", data);

	dev_info(&info->client->dev,
		"[TouchKey] %s : FW Ver 0x%02x\n", __func__, data);

	return count;
}

static ssize_t touch_version_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int count;

	count = snprintf(buf, 20, "0x%02x\n", BIN_FW_VERSION);
	return count;
}

static ssize_t touchkey_firm_status_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	int count = 0;
	char buff[16] = {0};
	dev_info(&info->client->dev, "[TouchKey] touchkey_update_status: %d\n",
						info->touchkey_update_status);
	if (info->touchkey_update_status == 0)
		count = snprintf(buff, sizeof(buff), "PASS\n");
	else if (info->touchkey_update_status == 1)
		count = snprintf(buff, sizeof(buff), "Downloading\n");
	else if (info->touchkey_update_status == -1)
		count = snprintf(buff, sizeof(buff), "Fail\n");
	return count;
}

static ssize_t touch_update_read(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	int count = 0;
	char buff[16] = {0};

	dev_info(&info->client->dev, "[TouchKey] touchkey_update_read: %d\n",
						info->touchkey_update_status);
	if (info->touchkey_update_status == 0)
		count = snprintf(buff, sizeof(buff), "PASS\n");
	else if (info->touchkey_update_status == 1)
		count = snprintf(buff, sizeof(buff), "Downloading\n");
	else if (info->touchkey_update_status == -1)
		count = snprintf(buff, sizeof(buff), "Fail\n");
	return count;
}


static ssize_t touch_update_write(struct device *dev,
			 struct device_attribute *attr,
			 const char *buf, size_t size)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	int count = 0;
	int retry = NUM_OF_RETRY_UPDATE;
	char buff[16] = {0};
	u8 data;

	info->touchkey_update_status = 1;
	dev_info(dev, "[TouchKey] touch_update_write!\n");

	disable_irq(info->irq);

	if (ic_fw_id & CYPRESS_55_IC_MASK) {
		printk(KERN_INFO "[Touchkey] IC id 20055\n");
		return 1;
		}
	else if (ic_fw_id & CYPRESS_65_IC_MASK)
		printk(KERN_INFO "[Touchkey] IC id 20065\n");
	else {
		printk(KERN_INFO "[Touchkey] IC id 20045\n");
		printk(KERN_INFO "[TouchKey] FW update does not support!\n");
		enable_irq(info->irq);
		return 1;
		}

	while (retry--) {
		if (ISSP_main() == 0) {
			dev_info(&info->client->dev,
				"[TouchKey] Update success!\n");
			msleep(50);
			cypress_touchkey_auto_cal(info);
			info->touchkey_update_status = 0;
			count = 1;
			enable_irq(info->irq);
			break;
		}
		dev_err(&info->client->dev,
			"[TouchKey] Touchkey_update failed... retry...\n");
	}

	if (retry <= 0) {
		if (info->pdata->gpio_led_en)
			cypress_touchkey_con_hw(info, false);
		msleep(300);
		count = 0;
		dev_err(&info->client->dev, "[TouchKey]Touchkey_update fail\n");
		info->touchkey_update_status = -1;
		return count;
	}

	msleep(500);

	data = i2c_smbus_read_byte_data(info->client, CYPRESS_FW_VER);
	count = snprintf(buff, sizeof(buff), "0x%02x\n", data);
	dev_info(&info->client->dev,
		"[TouchKey] %s : FW Ver 0x%02x\n", __func__, data);

	return count;
}

static ssize_t touch_led_control(struct device *dev,
				 struct device_attribute *attr, const char *buf,
				 size_t size)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);

	dev_info(&info->client->dev, "called %s\n", __func__);
	if (buf != NULL) {
		if (buf[0] == '1')
			cypress_touchkey_led_on(info);
		else
			cypress_touchkey_led_off(info);
#if defined(SEC_TOUCHKEY_DEBUG)
		dev_info(&info->client->dev,
			"[TouchKey] touch_led_control : %d\n", buf[0]);
#endif
	} else
		dev_info(&info->client->dev, "[TouchKey] touch_led_control Error\n");

	return size;
}

static ssize_t touch_sensitivity_control(struct device *dev,
				struct device_attribute *attr, const char *buf,
				size_t size)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	int ret;

	ret = i2c_smbus_write_byte_data(info->client,
			CYPRESS_GEN, CYPRESS_DATA_UPDATE);
	if (ret < 0) {
		dev_err(&info->client->dev,
			"[Touchkey] fail to CYPRESS_DATA_UPDATE.\n");
		return ret;
	}
	msleep(150);
	return size;
}

static ssize_t touchkey_menu_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	static u16 menu_sensitivity;
	u8 data[14] = { 0, };
	int ret;

	unsigned char sensitivity_control = 0x40;
	i2c_touchkey_write(info->client, &sensitivity_control, 1);

	ret = i2c_touchkey_read(info->client, KEYCODE_REG, data, 14);

	printk(KERN_DEBUG "called %s data[10] = %d, data[11] =%d\n", __func__,
			data[10], data[11]);
	menu_sensitivity = ((0x00FF & data[10]) << 8) | data[11];

	return snprintf(buf, 20, "%d\n", menu_sensitivity);
}

static ssize_t touchkey_back_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	static u16 back_sensitivity;
	u8 data[14] = { 0, };
	int ret;

	unsigned char sensitivity_control = 0x40;
	i2c_touchkey_write(info->client, &sensitivity_control, 1);

	ret = i2c_touchkey_read(info->client, KEYCODE_REG, data, 14);

	printk(KERN_DEBUG "called %s data[12] = %d, data[13] =%d\n", __func__,
			data[12], data[13]);
	back_sensitivity = ((0x00FF & data[12]) << 8) | data[13];

	return snprintf(buf, 20, "%d\n", back_sensitivity);
}

static ssize_t touchkey_raw_data0_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	static u16 raw_data0;
	u8 data[2] = {0,};
	int ret;
	bool touchkey;

	touchkey = info->pdata->touchkey_order;
	if (0) {
		ret = i2c_smbus_read_i2c_block_data(info->client,
			CYPRESS_RAW_DATA_MENU, ARRAY_SIZE(data), data);

	} else {
		ret = i2c_smbus_read_i2c_block_data(info->client,
		touchkey ? CYPRESS_RAW_DATA_MENU : CYPRESS_RAW_DATA_BACK,
		ARRAY_SIZE(data), data);
	}
	if (ret != ARRAY_SIZE(data)) {
		dev_err(&info->client->dev,
			"[TouchKey] fail to read MENU raw data.\n");
		return ret;
	}

	raw_data0 = ((0x00FF & data[0])<<8) | data[1];

	dev_info(&info->client->dev, "called %s , data : %d %d\n",
			__func__, data[0], data[1]);
	return snprintf(buf, 20, "%d\n", raw_data0);

}

static ssize_t touchkey_raw_data1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	static u16 raw_data1;
	u8 data[2] = {0,};
	int ret;
	bool touchkey;

	touchkey = info->pdata->touchkey_order;
	if (0) {
		ret = i2c_smbus_read_i2c_block_data(info->client,
			CYPRESS_RAW_DATA_BACK_GOGH, ARRAY_SIZE(data), data);
	} else {
		ret = i2c_smbus_read_i2c_block_data(info->client,
		touchkey ? CYPRESS_RAW_DATA_BACK : CYPRESS_RAW_DATA_MENU,
		ARRAY_SIZE(data), data);
	}
	if (ret != ARRAY_SIZE(data)) {
		dev_err(&info->client->dev,
			"[TouchKey] fail to read HOME raw data.\n");
		return ret;
	}

	raw_data1 = ((0x00FF & data[0])<<8) | data[1];

	dev_info(&info->client->dev, "called %s , data : %d %d\n",
			__func__, data[0], data[1]);
	return snprintf(buf, 20, "%d\n", raw_data1);

}

static ssize_t touchkey_idac0_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	static u8 idac0;
	u8 data = 0;
	bool touchkey;

	touchkey = info->pdata->touchkey_order;
	if (0) {
		data = i2c_smbus_read_byte_data(info->client,
					CYPRESS_IDAC_MENU);
	} else {
		data = i2c_smbus_read_byte_data(info->client,
			touchkey ? CYPRESS_IDAC_MENU : CYPRESS_IDAC_BACK);
	}
	dev_info(&info->client->dev, "called %s , data : %d\n", __func__, data);
	idac0 = data;
	return snprintf(buf, 20, "%d\n", idac0);

}

static ssize_t touchkey_idac1_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	static u8 idac1;
	u8 data = 0;
	bool touchkey;

	touchkey = info->pdata->touchkey_order;
	if (0) {
		data = i2c_smbus_read_byte_data(info->client,
					CYPRESS_IDAC_BACK);
	} else {
		data = i2c_smbus_read_byte_data(info->client,
			touchkey ? CYPRESS_IDAC_BACK : CYPRESS_IDAC_MENU);
	}
	dev_info(&info->client->dev, "called %s , data : %d\n", __func__, data);
	idac1 = data;
	return snprintf(buf, 20, "%d\n", idac1);

}

#if defined(TK_HOME_ENABLE)
static ssize_t touchkey_home_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	static u8 home_sensitivity;
	u8 data[2] = {0,};
	int ret;

	ret = i2c_smbus_read_i2c_block_data(info->client,
		CYPRESS_DIFF_MENU, ARRAY_SIZE(data), data);
	if (ret != ARRAY_SIZE(data)) {
		dev_err(&info->client->dev,
			"[TouchKey] fail to read home sensitivity.\n");
		return ret;
	}

	home_sensitivity = ((0x00FF & data[0])<<8) | data[1];

	dev_info(&info->client->dev, "called %s , data : %d %d\n",
			__func__, data[0], data[1]);
	return snprintf(buf, 20, "%d\n", home_sensitivity);

}
static ssize_t touchkey_raw_data2_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	static u16 raw_data1;
	u8 data[2] = {0,};
	int ret;

	ret = i2c_smbus_read_i2c_block_data(info->client,
		CYPRESS_RAW_DATA_HOME, ARRAY_SIZE(data), data);

	if (ret != ARRAY_SIZE(data)) {
		dev_err(&info->client->dev,
			"[TouchKey] fail to read HOME raw data.\n");
		return ret;
	}

	raw_data1 = ((0x00FF & data[0])<<8) | data[1];

	dev_info(&info->client->dev, "called %s , data : %d %d\n",
			__func__, data[0], data[1]);
	return snprintf(buf, 20, "%d\n", raw_data1);

}
static ssize_t touchkey_idac2_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	static u8 idac1;
	u8 data = 0;

	data = i2c_smbus_read_byte_data(info->client, CYPRESS_IDAC_HOME);

	dev_info(&info->client->dev, "called %s , data : %d\n", __func__, data);
	idac1 = data;
	return snprintf(buf, 20, "%d\n", idac1);

}
#endif
static ssize_t touchkey_threshold_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	static u8 touchkey_threshold;
	u8 data = 0;

	data = i2c_smbus_read_byte_data(info->client, CYPRESS_THRESHOLD);

	dev_info(&info->client->dev, "called %s , data : %d\n", __func__, data);
	touchkey_threshold = data;
	return snprintf(buf, 20, "%d\n", touchkey_threshold);
}

static ssize_t touch_autocal_testmode(struct device *dev,
		struct device_attribute *attr, const char *buf,
		size_t size)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	int count = 0;
	int on_off;
	if (sscanf(buf, "%d\n", &on_off) == 1) {
		printk(KERN_ERR "[TouchKey] Test Mode : %d\n", on_off);
		if (on_off == 1) {
			count = i2c_smbus_write_byte_data(info->client,
					CYPRESS_GEN, CYPRESS_DATA_UPDATE);
		}
	} else {
		if (info->pdata->gpio_led_en) {
			cypress_touchkey_con_hw(info, false);
			msleep(50);
			cypress_touchkey_con_hw(info, true);
			msleep(50);
		}
		cypress_touchkey_auto_cal(info);
	}

	return count;
}

static ssize_t autocalibration_enable(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	int data;

	sscanf(buf, "%d\n", &data);

	if (data == 1)
		cypress_touchkey_auto_cal(info);
	return 0;
}

static ssize_t autocalibration_status(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	u8 data[6];
	int ret;
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);

	printk(KERN_DEBUG "[Touchkey] %s\n", __func__);

	ret = i2c_smbus_read_i2c_block_data(info->client,
				CYPRESS_GEN, 6, data);
	if ((data[5] & 0x80))
		return snprintf(buf, 20, "Enabled\n");
	else
		return snprintf(buf, 20, "Disabled\n");
}

#if defined(CONFIG_GLOVE_TOUCH)
static ssize_t glove_mode_enable(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	int data;

	sscanf(buf, "%d\n", &data);
	dev_info(&info->client->dev, "%s %d\n", __func__, data);

	touchkey_glovemode(data);

	return size;
}
#endif

#ifdef TKEY_FLIP_MODE
static ssize_t flip_cover_mode_enable(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t size)
{
	struct cypress_touchkey_info *info = dev_get_drvdata(dev);
	int data;

	sscanf(buf, "%d\n", &data);
	dev_info(&info->client->dev, "%s %d\n", __func__, data);

	touchkey_flip_cover(data);

	return size;
}
#endif


static DEVICE_ATTR(touchkey_firm_update_status,
		S_IRUGO | S_IWUSR | S_IWGRP, touchkey_firm_status_show, NULL);
static DEVICE_ATTR(touchkey_firm_version_panel, S_IRUGO,
				touch_version_read, NULL);
static DEVICE_ATTR(touchkey_firm_version_phone, S_IRUGO,
				touch_version_show, NULL);
static DEVICE_ATTR(touchkey_firm_update, S_IRUGO | S_IWUSR | S_IWGRP,
				touch_update_read, touch_update_write);
static DEVICE_ATTR(brightness, S_IRUGO | S_IWUSR | S_IWGRP,
				NULL, touch_led_control);
static DEVICE_ATTR(touch_sensitivity, S_IRUGO | S_IWUSR | S_IWGRP,
				NULL, touch_sensitivity_control);
static DEVICE_ATTR(touchkey_menu, S_IRUGO, touchkey_menu_show, NULL);
static DEVICE_ATTR(touchkey_raw_data0, S_IRUGO, touchkey_raw_data0_show, NULL);
static DEVICE_ATTR(touchkey_idac0, S_IRUGO, touchkey_idac0_show, NULL);
static DEVICE_ATTR(touchkey_back, S_IRUGO, touchkey_back_show, NULL);
static DEVICE_ATTR(touchkey_raw_data1, S_IRUGO, touchkey_raw_data1_show, NULL);
static DEVICE_ATTR(touchkey_idac1, S_IRUGO, touchkey_idac1_show, NULL);
#if defined(TK_HOME_ENABLE)
static DEVICE_ATTR(touchkey_home, S_IRUGO, touchkey_home_show, NULL);
static DEVICE_ATTR(touchkey_raw_data2, S_IRUGO, touchkey_raw_data2_show, NULL);
static DEVICE_ATTR(touchkey_idac2, S_IRUGO, touchkey_idac2_show, NULL);
#endif
static DEVICE_ATTR(touchkey_threshold, S_IRUGO, touchkey_threshold_show, NULL);
static DEVICE_ATTR(touchkey_autocal_start, S_IRUGO | S_IWUSR | S_IWGRP,
				NULL, touch_autocal_testmode);
static DEVICE_ATTR(autocal_enable, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   autocalibration_enable);
static DEVICE_ATTR(autocal_stat, S_IRUGO | S_IWUSR | S_IWGRP,
		   autocalibration_status, NULL);
static DEVICE_ATTR(touchkey_brightness_level, S_IRUGO | S_IWUSR | S_IWGRP,
				brightness_level_show, brightness_control);
#if defined(CONFIG_GLOVE_TOUCH)
static DEVICE_ATTR(glove_mode, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   glove_mode_enable);
#endif

#ifdef TKEY_FLIP_MODE
static DEVICE_ATTR(flip_mode, S_IRUGO | S_IWUSR | S_IWGRP, NULL,
		   flip_cover_mode_enable);
#endif


static int __devinit cypress_touchkey_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct cypress_touchkey_platform_data *pdata =
					client->dev.platform_data;
	struct cypress_touchkey_info *info;
	struct input_dev *input_dev;
	int ret = 0;
	int i;
	int retry = NUM_OF_RETRY_UPDATE;
	u32 ic_fw_ver;
	u8 data[6] = { 0, };
	struct device *sec_touchkey;

	printk(KERN_INFO "[TouchKey] START(%s)!\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&client->dev, "fail to memory allocation.\n");
		goto err_mem_alloc;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "fail to allocate input device.\n");
		goto err_input_dev_alloc;
	}

	info->client = client;
	info->input_dev = input_dev;
	info->pdata = client->dev.platform_data;
	info->irq = client->irq;
	info->power_onoff = pdata->power_onoff;
	info->touchkey_update_status = 0;
	memcpy(info->keycode, pdata->touchkey_keycode,
			sizeof(pdata->touchkey_keycode));
	snprintf(info->phys, sizeof(info->phys),
			"%s/input0", dev_name(&client->dev));
	input_dev->name = "sec_touchkey";
	input_dev->phys = info->phys;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;

	info->is_powering_on = true;

	info->power_onoff(1);
	msleep(50);
	info->enabled = true;

	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_LED, input_dev->evbit);
	set_bit(LED_MISC, input_dev->ledbit);
	for (i = 0; i < ARRAY_SIZE(info->keycode); i++)
		set_bit(info->keycode[i], input_dev->keybit);

	input_set_drvdata(input_dev, info);
	mutex_init(&info->touchkey_led_mutex);

	ret = input_register_device(input_dev);
	if (ret) {
		dev_err(&client->dev, "[TouchKey] failed to register input dev (%d).\n",
			ret);
		goto err_reg_input_dev;
	}

	i2c_set_clientdata(client, info);

	if (info->pdata->gpio_led_en) {
		ret = gpio_request(info->pdata->gpio_led_en,
						"gpio_touchkey_led");
		if (ret < 0) {
			dev_err(&client->dev,
				"gpio_touchkey_led gpio_request is failed\n");
			goto err_gpio_request;
		}
		gpio_tlmm_config(GPIO_CFG(info->pdata->gpio_led_en, 0,
			GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);

		cypress_touchkey_con_hw(info, true);
	}

#if defined(CONFIG_MACH_JF_DCM)
	msleep(50);
#endif
	ret = i2c_touchkey_read(info->client, KEYCODE_REG, data, 6);

#ifdef CONFIG_TOUCHSCREEN_FACTORY_PLATFORM
	if (ret < 0) {
		printk(KERN_ERR "[TouchKey] %s %d i2c transfer error\n",
			__func__, __LINE__);
		goto err_i2c_check;
		}
#else
	if (ret < 0) {
		disable_irq(client->irq);
		if (ISSP_main() == 0) {
			dev_info(&client->dev, "[TouchKey] Update success!\n");
			enable_irq(client->irq);
			}
		else {
			printk(KERN_ERR "[TouchKey] %s %d i2c transfer error\n",
				__func__, __LINE__);
			goto err_i2c_check;
			}
		}
#endif

#ifdef TSP_BOOSTER
	cypress_init_dvfs(info);
#endif

	ret = request_threaded_irq(client->irq, NULL,
			cypress_touchkey_interrupt,
			IRQF_TRIGGER_FALLING, client->dev.driver->name, info);
	if (ret < 0) {
		dev_err(&client->dev, "Failed to request IRQ %d (err: %d).\n",
				client->irq, ret);
		goto err_req_irq;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	info->early_suspend.suspend = cypress_touchkey_early_suspend;
	info->early_suspend.resume = cypress_touchkey_late_resume;
	register_early_suspend(&info->early_suspend);
#endif /* CONFIG_HAS_EARLYSUSPEND */

#if defined(CONFIG_GLOVE_TOUCH)
	info->glove_wq = create_singlethread_workqueue("cypress_touchkey");
	if (!info->glove_wq)
		dev_err(&client->dev, "fail to create glove workquewe.\n");
	else
		INIT_WORK(&info->glove_work, cypress_touchkey_glove_work);
#endif

	info->led_wq = create_singlethread_workqueue("cypress_touchkey");
	if (!info->led_wq)
		dev_err(&client->dev, "fail to create led workquewe.\n");
	else
		INIT_WORK(&info->led_work, cypress_touchkey_led_work);

	info->leds.name = TOUCHKEY_BACKLIGHT;
	info->leds.brightness = LED_FULL;
	info->leds.max_brightness = LED_FULL;
	info->leds.brightness_set = cypress_touchkey_brightness_set;

	ret = led_classdev_register(&client->dev, &info->leds);
	if (ret)
		goto err_led_class_dev;

	ic_fw_ver = data[1];
	printk(KERN_INFO "[Touchkey] IC FW Version: 0x%02x\n", ic_fw_ver);

	ic_fw_id = data[5];
	printk(KERN_INFO "[Touchkey] IC ID Version: 0x%02x\n", ic_fw_id);

	if ((ic_fw_id & CYPRESS_65_IC_MASK) && (ic_fw_ver >= BASE_FW_VERSION) && (ic_fw_ver < BIN_FW_VERSION)){
		printk(KERN_INFO "[Touchkey] IC id 20065\n");
		printk(KERN_INFO "[TouchKey] touchkey_update Start!!\n");
		disable_irq(client->irq);

		while (retry--) {
			if (ISSP_main() == 0) {
				dev_info(&client->dev, "[TouchKey] Update success!\n");
				enable_irq(client->irq);
				break;
			}
			dev_err(&client->dev,
				"[TouchKey] Touchkey_update failed... retry...\n");
		}

		if (retry <= 0) {
			if (info->pdata->gpio_led_en)
				cypress_touchkey_con_hw(info, false);
			msleep(300);
			dev_err(&client->dev, "[TouchKey]Touchkey_update fail\n");
		}

		msleep(500);

		ic_fw_ver = i2c_smbus_read_byte_data(info->client,
				CYPRESS_FW_VER);
		dev_info(&client->dev,
			"[TouchKey] %s : FW Ver 0x%02x\n", __func__, ic_fw_ver);
		}
	else if ((ic_fw_id & CYPRESS_55_IC_MASK) && (ic_fw_ver < BIN_FW_VERSION_20055)){
		printk(KERN_INFO "[Touchkey] IC id 20055\n");
		printk(KERN_INFO "[TouchKey] touchkey_update Start!!\n");
		disable_irq(client->irq);

		while (retry--) {
			if (ISSP_main() == 0) {
				dev_info(&client->dev, "[TouchKey] Update success!\n");
				enable_irq(client->irq);
				break;
			}
			dev_err(&client->dev,
				"[TouchKey] Touchkey_update failed... retry...\n");
		}

		if (retry <= 0) {
			if (info->pdata->gpio_led_en)
				cypress_touchkey_con_hw(info, false);
			msleep(300);
			dev_err(&client->dev, "[TouchKey]Touchkey_update fail\n");
		}

		msleep(500);

		ic_fw_ver = i2c_smbus_read_byte_data(info->client,
				CYPRESS_FW_VER);
		dev_info(&client->dev,
			"[TouchKey] %s : FW Ver 0x%02x\n", __func__, ic_fw_ver);

	} else {
		dev_info(&client->dev, "[TouchKey] FW update does not need!\n");
	}

	cypress_touchkey_auto_cal(info);
	sec_touchkey = device_create(sec_class, NULL, 0, NULL, "sec_touchkey");
	if (IS_ERR(sec_touchkey)) {
		pr_err("Failed to create device(sec_touchkey)!\n");
		goto err_sysfs;
	}
	dev_set_drvdata(sec_touchkey, info);

#if defined(TK_INFORM_CHARGER)
	info->callbacks.inform_charger = touchkey_ta_cb;
	if (info->pdata->register_cb) {
		dev_info(&client->dev, "[Touchkey] Register TA Callback\n");
		info->pdata->register_cb(&info->callbacks);
	}
#endif

	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_firm_update_status) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_firm_update.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_firm_update) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_firm_update.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_firm_version_panel) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_firm_version_panel.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_firm_version_phone) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_firm_version_phone.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_brightness) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_brightness.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_touch_sensitivity) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touch_sensitivity.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_menu) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_menu.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_back) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_back.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_raw_data0) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_raw_data0.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_raw_data1) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_raw_data1.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_idac0) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_idac0.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_idac1) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_idac1.attr.name);
		goto err_sysfs;
	}
#if defined(TK_HOME_ENABLE)
	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_home) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_home.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_raw_data2) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_raw_data2.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey, &dev_attr_touchkey_idac2) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_idac2.attr.name);
		goto err_sysfs;
	}
#endif
	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_threshold) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_threshold.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_touchkey_autocal_start) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_touchkey_autocal_start.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_autocal_enable) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_autocal_enable.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
			&dev_attr_autocal_stat) < 0) {
		pr_err("Failed to create device file(%s)!\n",
			dev_attr_autocal_stat.attr.name);
		goto err_sysfs;
	}

	if (device_create_file(sec_touchkey,
		&dev_attr_touchkey_brightness_level) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n",
		dev_attr_touchkey_brightness_level.attr.name);
		goto err_sysfs;
	}
#if defined(CONFIG_GLOVE_TOUCH)
	if (device_create_file(sec_touchkey,
		&dev_attr_glove_mode) < 0) {
		printk(KERN_ERR "Failed to create device file(%s)!\n",
		dev_attr_glove_mode.attr.name);
		goto err_sysfs;
	}
#endif
#ifdef TKEY_FLIP_MODE
		if (device_create_file(sec_touchkey,
			&dev_attr_flip_mode) < 0) {
			printk(KERN_ERR "Failed to create device file(%s)!\n",
			dev_attr_flip_mode.attr.name);
			goto err_sysfs;
		}
#endif


	info->is_powering_on = false;

#if defined(CONFIG_GLOVE_TOUCH)
		tkey_info = info;
#endif

	printk(KERN_INFO "[Touchkey] DONE(%s)!\n", __func__);
	return 0;


err_sysfs:
err_led_class_dev:
	if (info->led_wq)
		destroy_workqueue(info->led_wq);
err_req_irq:
err_i2c_check:
err_gpio_request:
	input_unregister_device(input_dev);
err_reg_input_dev:
	input_free_device(input_dev);
	input_dev = NULL;
	mutex_destroy(&info->touchkey_led_mutex);
err_input_dev_alloc:
	kfree(info);
	return -ENXIO;
err_mem_alloc:
	return -ENOMEM;

}

static int __devexit cypress_touchkey_remove(struct i2c_client *client)
{
	struct cypress_touchkey_info *info = i2c_get_clientdata(client);
	if (info->irq >= 0)
		free_irq(info->irq, info);
	mutex_destroy(&info->touchkey_led_mutex);
	led_classdev_unregister(&info->leds);
	input_unregister_device(info->input_dev);
	input_free_device(info->input_dev);
	kfree(info);
	return 0;
}

#if defined(CONFIG_PM) || defined(CONFIG_HAS_EARLYSUSPEND)
static int cypress_touchkey_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cypress_touchkey_info *info = i2c_get_clientdata(client);
	int ret = 0;

	info->is_powering_on = true;
	disable_irq(info->irq);
	if (info->pdata->gpio_led_en)
		cypress_touchkey_con_hw(info, false);
	info->power_onoff(0);
	cypress_int_gpio_setting(false);
	cypress_gpio_setting(false);
	info->enabled = false;
	info->done_ta_setting = true;

#ifdef TSP_BOOSTER
	cypress_set_dvfs_lock(info, 2);
	dev_info(&info->client->dev,
			"%s: dvfs_lock free.\n", __func__);
#endif
	return ret;
}

static int cypress_touchkey_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct cypress_touchkey_info *info = i2c_get_clientdata(client);
	int ret = 0;

	info->power_onoff(1);
	cypress_int_gpio_setting(true);
	cypress_gpio_setting(true);
	if (info->pdata->gpio_led_en)
		cypress_touchkey_con_hw(info, true);
	msleep(100);

	if (touchled_cmd_reversed) {
			touchled_cmd_reversed = 0;
			i2c_smbus_write_byte_data(info->client,
					CYPRESS_GEN, touchkey_led_status);
			printk(KERN_INFO "%s: LED returned on, %d\n", __func__, ret);

			msleep(30);
			i2c_smbus_write_byte_data(info->client,
					CYPRESS_GEN, touchkey_led_status);
			printk(KERN_INFO "%s: LED returned on, %d\n", __func__, ret);
	}

	info->enabled = true;

	if(!(info->done_ta_setting)) {
		printk(KERN_DEBUG "[Touchkey] Enter the TA setting\n");
		touchkey_ta_setting(info);
		}

	cypress_touchkey_auto_cal(info);
/*
	if (touchled_cmd_reversed) {
			msleep(30);
			touchled_cmd_reversed = 0;
			i2c_smbus_write_byte_data(info->client,
					CYPRESS_GEN, touchkey_led_status);
			printk(KERN_DEBUG "LED returned on\n");
		}
*/
	enable_irq(info->irq);

	info->is_powering_on = false;
	return ret;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cypress_touchkey_early_suspend(struct early_suspend *h)
{
	struct cypress_touchkey_info *info;
	info = container_of(h, struct cypress_touchkey_info, early_suspend);
	cypress_touchkey_suspend(&info->client->dev);
}

static void cypress_touchkey_late_resume(struct early_suspend *h)
{
	struct cypress_touchkey_info *info;
	info = container_of(h, struct cypress_touchkey_info, early_suspend);
	cypress_touchkey_resume(&info->client->dev);
}
#endif

static const struct i2c_device_id cypress_touchkey_id[] = {
	{"cypress_touchkey", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, cypress_touchkey_id);

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static const struct dev_pm_ops cypress_touchkey_pm_ops = {
	.suspend	= cypress_touchkey_suspend,
	.resume		= cypress_touchkey_resume,
};
#endif

struct i2c_driver cypress_touchkey_driver = {
	.probe = cypress_touchkey_probe,
	.remove = cypress_touchkey_remove,
	.driver = {
		.name = "cypress_touchkey",
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
		.pm	= &cypress_touchkey_pm_ops,
#endif
		   },
	.id_table = cypress_touchkey_id,
};

static int __init cypress_touchkey_init(void)
{
	int ret = 0;
	ret = i2c_add_driver(&cypress_touchkey_driver);
	if (ret) {
		printk(KERN_ERR"[TouchKey] cypress touch keypad registration failed. ret= %d\n",
			ret);
	}

	return ret;
}

static void __exit cypress_touchkey_exit(void)
{
	i2c_del_driver(&cypress_touchkey_driver);
}

late_initcall(cypress_touchkey_init);
module_exit(cypress_touchkey_exit);

MODULE_DESCRIPTION("Touchkey driver for Cypress touchkey controller ");
MODULE_LICENSE("GPL");
