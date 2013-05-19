/*
** =========================================================================
** File:
**     tspdrv.c
**
** Description:
**     TouchSense Kernel Module main entry-point.
**
** Portions Copyright (c) 2008-2010 Immersion Corporation. All Rights Reserved.
**
** This file contains Original Code and/or Modifications of Original Code
** as defined in and that are subject to the GNU Public License v2 -
** (the 'License'). You may not use this file except in compliance with the
** License. You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES,
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see
** the License for the specific language governing rights and limitations
** under the License.
** =========================================================================
*/

#ifndef __KERNEL__
#define __KERNEL__
#endif

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#include <linux/io.h>
#include "../staging/android/timed_output.h"

#include "tspdrv.h"
#include <linux/vibrator.h>
#include "immvibespi.c"

#if defined(VIBE_DEBUG) && defined(VIBE_RECORD)
#include <tspdrvRecorder.c>
#endif
#include <mach/msm_xo.h>
/* Device name and version information */
/* DO NOT CHANGE - this is auto-generated */
#define VERSION_STR " v3.4.55.7\n"
/* account extra space for future extra digits in version number */
#define VERSION_STR_LEN 16
/* initialized in tspdrv_probe */
static char g_szdevice_name[(VIBE_MAX_DEVICE_NAME_LENGTH
							+ VERSION_STR_LEN)
							* NUM_ACTUATORS];
static size_t g_cchdevice_name;

static struct wake_lock vib_wake_lock;

/* Flag indicating whether the driver is in use */
static char g_bisplaying;

/* Buffer to store data sent to SPI */
#define SPI_BUFFER_SIZE \
		(NUM_ACTUATORS * (VIBE_OUTPUT_SAMPLE_SIZE + SPI_HEADER_SIZE))
static int g_bstoprequested;
static actuator_samples_buffer g_samples_buffer[NUM_ACTUATORS] = {{0} };
static char g_cwrite_buffer[SPI_BUFFER_SIZE];

#define VIBE_TUNING
/* #define VIBE_ENABLE_SYSTEM_TIMER */

/* For QA purposes */
#ifdef QA_TEST
#define FORCE_LOG_BUFFER_SIZE   128
#define TIME_INCREMENT          5
static int g_ntime;
static int g_nforcelog_index;
static int8_t g_nforcelog[FORCE_LOG_BUFFER_SIZE];
#endif

#ifdef IMPLEMENT_AS_CHAR_DRIVER
static int g_nmajor;
#endif

/* Needs to be included after the global variables because it uses them */
#ifdef CONFIG_HIGH_RES_TIMERS
    #include "VibeOSKernelLinuxHRTime.c"
#else
    #include "VibeOSKernelLinuxTime.c"
#endif

/* timed_output */
static void _set_vibetonz_work(struct work_struct *unused);

static DECLARE_WORK(vibetonz_work, _set_vibetonz_work);


static struct hrtimer timer;
static int max_timeout = 10000;

static int vibrator_value = -1;
static int vibrator_work;

#define TEST_MODE_TIME 10000

struct vibrator_platform_data vibrator_drvdata;

static int set_vibetonz(int timeout)
{
	int8_t strength;
	if (!timeout) {
		if (vibrator_drvdata.vib_model == HAPTIC_PWM) {
			strength = 0;
			ImmVibeSPI_ForceOut_SetSamples(0, 8, 1, &strength);
		} else { /* HAPTIC_MOTOR */
			ImmVibeSPI_ForceOut_AmpDisable(0);
		}
	} else {
		DbgOut((KERN_INFO "tspdrv: ENABLE\n"));
		if (vibrator_drvdata.vib_model == HAPTIC_PWM) {
			strength = 119;
			/* 90% duty cycle */
			ImmVibeSPI_ForceOut_SetSamples(0, 8, 1, &strength);
		} else { /* HAPTIC_MOTOR */
			ImmVibeSPI_ForceOut_AmpEnable(0);
		}
	}

	vibrator_value = timeout;
	return 0;
}

static void _set_vibetonz_work(struct work_struct *unused)
{
	set_vibetonz(vibrator_work);

	return;
}

static enum hrtimer_restart vibetonz_timer_func(struct hrtimer *timer)
{
	/* set_vibetonz(0); */
	vibrator_work = 0;
	schedule_work(&vibetonz_work);

	return HRTIMER_NORESTART;
}

static int get_time_for_vibetonz(struct timed_output_dev *dev)
{
	int remaining;

	if (hrtimer_active(&timer))	{
		ktime_t r = hrtimer_get_remaining(&timer);
		remaining = ktime_to_ms(r);/*returning time in ms*/
	} else {
		remaining = 0;
	}

	if (vibrator_value == -1)
		remaining = -1;

	return remaining;

}

static void enable_vibetonz_from_user(struct timed_output_dev *dev, int value)
{
	//printk(KERN_DEBUG "tspdrv: Enable time = %d msec\n", value);
	hrtimer_cancel(&timer);

	/* set_vibetonz(value); */
	vibrator_work = value;
	schedule_work(&vibetonz_work);

	if (value > 0 && (value != TEST_MODE_TIME)) {
		if (value > max_timeout)
			value = max_timeout;

		hrtimer_start(&timer,
			ktime_set(value / 1000, (value % 1000) * 1000000),
			HRTIMER_MODE_REL);
		vibrator_value = 0;
	}
}

static struct timed_output_dev timed_output_vt = {
	.name     = "vibrator",
	.get_time = get_time_for_vibetonz,
	.enable   = enable_vibetonz_from_user,
};

static void vibetonz_start(void)
{
	int ret = 0;

	hrtimer_init(&timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	timer.function = vibetonz_timer_func;

	ret = timed_output_dev_register(&timed_output_vt);

	if (ret)
		DbgOut((KERN_ERR
		"tspdrv: timed_output_dev_register is fail\n"));
}

/* File IO */
static int open(struct inode *inode, struct file *file);
static int release(struct inode *inode, struct file *file);
static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos);
static ssize_t write(struct file *file, const char *buf, size_t count,
					 loff_t *ppos);
#if HAVE_UNLOCKED_IOCTL
static long ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
#endif
static const struct file_operations fops = {
	.owner =    THIS_MODULE,
	.read =     read,
	.write =    write,
	.unlocked_ioctl =    ioctl,
	.open =     open,
	.release =  release ,
	.llseek =   default_llseek
};

#ifndef IMPLEMENT_AS_CHAR_DRIVER
static struct miscdevice miscdev = {
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     MODULE_NAME,
	.fops =     &fops
};
#endif

#ifdef VIBE_ENABLE_SYSTEM_TIMER
int vibetonz_clk_on(struct device *dev)
{
	struct clk *vibetonz_clk = NULL;

	vibetonz_clk = clk_get(dev, "timers");
	if (IS_ERR(vibetonz_clk)) {
		DbgOut((KERN_ERR "tspdrv: failed to get clock for vibetonz\n"));
		goto err_clk0;
	}
	clk_enable(vibetonz_clk);
	clk_put(vibetonz_clk);

	return 0;

err_clk0:
	clk_put(vibetonz_clk);
	return -EINVAL;
}

int vibetonz_clk_off(struct device *dev)
{
	struct clk *vibetonz_clk = NULL;

	vibetonz_clk = clk_get(dev, "timers");
	if (IS_ERR(vibetonz_clk)) {
		DbgOut((KERN_ERR "tspdrv: failed to get clock for vibetonz\n"));
		goto err_clk0;
	}
	clk_disable(vibetonz_clk);
	clk_put(vibetonz_clk);

	return 0;

err_clk0:
	clk_put(vibetonz_clk);

	return -EINVAL;
}
#else
int vibetonz_clk_on(struct device *dev)
{
	return -EINVAL;
}

int vibetonz_clk_off(struct device *dev)
{
	return -EINVAL;
}
#endif	/* VIBE_ENABLE_SYSTEM_TIMER */

static __devinit int tspdrv_probe(struct platform_device *pdev)
{
	struct vibrator_platform_data *pdata;
	int ret, i;   /* initialized below */

	create_vibrator_sysfs();

	DbgOut((KERN_INFO "tspdrv: tspdrv_probe.\n"));

	/* This condition will be removed,after all board files changes done */
	if (pdev->dev.platform_data == NULL) {
		DbgOut(KERN_ERR "tspdrv: tspdrv probe failed, pdata is NULL");
		return -EINVAL;
	} else {
		pdata = pdev->dev.platform_data;
		vibrator_drvdata.vib_model = pdata->vib_model;
		vibrator_drvdata.is_pmic_haptic_pwr_en = \
				pdata->is_pmic_haptic_pwr_en;
		if (pdata->vib_model == HAPTIC_PWM) {
			if (pdata->is_pmic_vib_pwm)
				vibrator_drvdata.vib_pwm_gpio = \
				PM8921_GPIO_PM_TO_SYS(pdata->vib_pwm_gpio);
			else
				vibrator_drvdata.vib_pwm_gpio =
					pdata->vib_pwm_gpio;
		}
		vibrator_drvdata.power_onoff = pdata->power_onoff;
	}
#ifdef IMPLEMENT_AS_CHAR_DRIVER
	g_nmajor = register_chrdev(0, MODULE_NAME, &fops);
	if (g_nmajor < 0) {
		DbgOut((KERN_ERR "tspdrv: can't get major number.\n"));
		ret = g_nmajor;
		return ret;
	}
#else
	ret = misc_register(&miscdev);
	if (ret) {
		DbgOut((KERN_ERR "tspdrv: misc_register failed.\n"));
		return ret;
	}
#endif

	DbgRecorderInit(());

	vibetonz_clk_on(&pdev->dev);

	ImmVibeSPI_ForceOut_Initialize();
	VibeOSKernelLinuxInitTimer();

	/* Get and concatenate device name and initialize data buffer */
	g_cchdevice_name = 0;
	for (i = 0; i < NUM_ACTUATORS; i++) {
		char *szName = g_szdevice_name + g_cchdevice_name;
		ImmVibeSPI_Device_GetName(i,
				szName, VIBE_MAX_DEVICE_NAME_LENGTH);

		/* Append version information and get buffer length */
		strlcat(szName, VERSION_STR, sizeof(VERSION_STR));
		g_cchdevice_name +=
			strnlen(szName, (VIBE_MAX_DEVICE_NAME_LENGTH+VERSION_STR_LEN)*NUM_ACTUATORS);

		g_samples_buffer[i].nindex_playing_buffer = -1;/* Not playing */
		g_samples_buffer[i].actuator_samples[0].nbuffer_size = 0;
		g_samples_buffer[i].actuator_samples[1].nbuffer_size = 0;
	}
	wake_lock_init(&vib_wake_lock, WAKE_LOCK_SUSPEND, "vib_present");

	vibetonz_start();

	return 0;
}

static int __devexit tspdrv_remove(struct platform_device *pdev)
{
	DbgOut((KERN_INFO "tspdrv: tspdrv_remove.\n"));

	DbgRecorderTerminate(());

	VibeOSKernelLinuxTerminateTimer();
	ImmVibeSPI_ForceOut_Terminate();

	wake_lock_destroy(&vib_wake_lock);

	return 0;
}

static int open(struct inode *inode, struct file *file)
{
	DbgOut((KERN_INFO "tspdrv: open.\n"));

	if (!try_module_get(THIS_MODULE))
		return -ENODEV;

	return 0;
}

static int release(struct inode *inode, struct file *file)
{
	DbgOut((KERN_INFO "tspdrv: release.\n"));

	/*
	** Reset force and stop timer when the driver is closed, to make sure
	** no dangling semaphore remains in the system, especially when the
	** driver is run outside of immvibed for testing purposes.
	*/
	VibeOSKernelLinuxStopTimer();

	/*
	** Clear the variable used to store the magic number to prevent
	** unauthorized caller to write data. TouchSense service is the only
	** valid caller.
	*/
	file->private_data = (void *)NULL;

	module_put(THIS_MODULE);

	return 0;
}

static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
	const size_t nbufsize =
			(g_cchdevice_name > (size_t)(*ppos)) ?
			min(count, g_cchdevice_name - (size_t)(*ppos)) : 0;

	/* End of buffer, exit */
	if (0 == nbufsize)
		return 0;

	if (0 != copy_to_user(buf, g_szdevice_name + (*ppos), nbufsize)) {
		/* Failed to copy all the data, exit */
		DbgOut((KERN_ERR "tspdrv: copy_to_user failed.\n"));
		return 0;
	}

	/* Update file position and return copied buffer size */
	*ppos += nbufsize;
	return nbufsize;
}

static ssize_t write(struct file *file, const char *buf, size_t count,
					 loff_t *ppos)
{
	int i = 0;

	*ppos = 0;  /* file position not used, always set to 0 */
	/* DbgOut((KERN_ERR "tspdrv: write....\n")); */

	/*
	** Prevent unauthorized caller to write data.
	** TouchSense service is the only valid caller.
	*/
	if (file->private_data != (void *)TSPDRV_MAGIC_NUMBER) {
		DbgOut((KERN_ERR "tspdrv: unauthorized write.\n"));
		return 0;
	}

	/* Check buffer size */
	if ((count <= SPI_HEADER_SIZE) || (count > SPI_BUFFER_SIZE)) {
		DbgOut((KERN_ERR "tspdrv: invalid write buffer size.\n"));
		return 0;
	}

	/* Copy immediately the input buffer */
	if (0 != copy_from_user(g_cwrite_buffer, buf, count)) {
		/* Failed to copy all the data, exit */
		DbgOut((KERN_ERR "tspdrv: copy_from_user failed.\n"));
		return 0;
	}

	while (i < count) {
		int nindex_free_buffer;   /* initialized below */

		samples_buffer *pinput_buffer =
			(samples_buffer *)(&g_cwrite_buffer[i]);

		if ((i + SPI_HEADER_SIZE) >= count) {
			/*
			** Index is about to go beyond the buffer size.
			** (Should never happen).
			*/
			DbgOut((KERN_EMERG "tspdrv: invalid buffer index.\n"));
		}

		/* Check bit depth */
		if (8 != pinput_buffer->nbit_depth)
			DbgOut((KERN_WARNING
			"tspdrv: invalid bit depth.Use default value(8).\n"));

		/* The above code not valid if SPI header size is not 3 */
#if (SPI_HEADER_SIZE != 3)
#error "SPI_HEADER_SIZE expected to be 3"
#endif

		/* Check buffer size */
		if ((i + SPI_HEADER_SIZE + pinput_buffer->nbuffer_size)
			> count) {
			/*
			** Index is about to go beyond the buffer size.
			** (Should never happen).
			*/
			DbgOut((KERN_EMERG "tspdrv: invalid data size.\n"));
		}

		/* Check actuator index */
		if (NUM_ACTUATORS <= pinput_buffer->nactuator_index) {
			DbgOut((KERN_ERR "tspdrv: invalid actuator index.\n"));
			i += (SPI_HEADER_SIZE + pinput_buffer->nbuffer_size);
			continue;
		}

		if (0 == g_samples_buffer[pinput_buffer->nactuator_index]
			.actuator_samples[0].nbuffer_size) {
			nindex_free_buffer = 0;
		} else if (0 == g_samples_buffer[pinput_buffer->nactuator_index]
			.actuator_samples[1].nbuffer_size) {
			nindex_free_buffer = 1;
		} else {
			/* No room to store new samples  */
			DbgOut((KERN_ERR
			 "tspdrv: no room to store new samples.\n"));
			return 0;
		}

		/* Store the data in the free buffer of the given actuator */
		memcpy(
			   &(g_samples_buffer[pinput_buffer->nactuator_index]
			   .actuator_samples[nindex_free_buffer]),
			   &g_cwrite_buffer[i],
			   (SPI_HEADER_SIZE + pinput_buffer->nbuffer_size));

		/*  If the no buffer is playing, prepare to play
		 ** g_samples_buffer[pinput_buffer->nactuator_index].
		 ** actuator_samples[nindex_free_buffer]
		 */
		if (-1 == g_samples_buffer[pinput_buffer->nactuator_index]
			.nindex_playing_buffer) {
			g_samples_buffer[pinput_buffer->nactuator_index]
				.nindex_playing_buffer = nindex_free_buffer;
			g_samples_buffer[pinput_buffer->nactuator_index]
				.nindex_output_value = 0;
		}

		/* Increment buffer index */
		i += (SPI_HEADER_SIZE + pinput_buffer->nbuffer_size);
	}

#ifdef QA_TEST
	g_nforcelog[g_nforcelog_index++] = g_cSPIBuffer[0];
	if (g_nforcelog_index >= FORCE_LOG_BUFFER_SIZE) {
		for (i = 0; i < FORCE_LOG_BUFFER_SIZE; i++) {
			printk(KERN_INFO "%d\t%d\n", g_ntime, g_nforcelog[i]);
			g_ntime += TIME_INCREMENT;
		}
		g_nforcelog_index = 0;
	}
#endif

	/* Start the timer after receiving new output force */
	g_bisplaying = true;
	VibeOSKernelLinuxStartTimer();

	return count;
}

static long ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
#ifdef QA_TEST
	int i;
#endif

	/* DbgOut(KERN_INFO "tspdrv: ioctl cmd[0x%x].\n", cmd); */
	switch (cmd) {
	case TSPDRV_STOP_KERNEL_TIMER:
		/*
		** As we send one sample ahead of time, we need to finish
		** playing the last sample before stopping the timer.
		** So we just set a flag here.
		*/
		if (true == g_bisplaying)
			g_bstoprequested = true;

#ifdef VIBEOSKERNELPROCESSDATA
		/* Last data processing to disable amp and stop timer */
		VibeOSKernelProcessData(NULL);
#endif

#ifdef QA_TEST
		if (g_nforcelog_index) {
			for (i = 0; i < g_nforcelog_index; i++) {
				printk(KERN_INFO "%d\t%d\n"
					   , g_ntime, g_nforcelog[i]);
				g_ntime += TIME_INCREMENT;
			}
		}
		g_ntime = 0;
		g_nforcelog_index = 0;
#endif
		break;

	case TSPDRV_MAGIC_NUMBER:
		filp->private_data = (void *)TSPDRV_MAGIC_NUMBER;
		break;

	case TSPDRV_ENABLE_AMP:
		wake_lock(&vib_wake_lock);
		ImmVibeSPI_ForceOut_AmpEnable(arg);
		DbgRecorderReset((arg));
		DbgRecord((arg, ";------- TSPDRV_ENABLE_AMP ---------\n"));
		break;

	case TSPDRV_DISABLE_AMP:
		/*
		** Small fix for now to handle proper combination of
		** TSPDRV_STOP_KERNEL_TIMER and TSPDRV_DISABLE_AMP together
		** If a stop was requested, ignore the request as the amp
		** will be disabled by the timer proc when it's ready
		*/
		if (!g_bstoprequested)
			ImmVibeSPI_ForceOut_AmpDisable(arg);
		wake_unlock(&vib_wake_lock);
		break;

	case TSPDRV_GET_NUM_ACTUATORS:
		return NUM_ACTUATORS;
	}

	return 0;
}

static int suspend(struct platform_device *pdev, pm_message_t state)
{
	int ret;

	if (g_bisplaying) {
		ret = -EBUSY;
	} else {
		/* Disable system timers */
		vibetonz_clk_off(&pdev->dev);

		ret = 0;
	}
	DbgOut(KERN_DEBUG "tspdrv: %s (%d).\n", __func__, ret);
	return ret;
}

static int resume(struct platform_device *pdev)
{
	/* Restart system timers */
	DbgOut(KERN_DEBUG "tspdrv: %s.\n", __func__);
	return 0;
}

static struct platform_driver tspdrv_driver = {
	.probe = tspdrv_probe,
	.remove = __devexit_p(tspdrv_remove),
	.suspend = suspend,
	.resume = resume,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init tspdrv_init(void)
{
	return platform_driver_register(&tspdrv_driver);
}

static void __exit tspdrv_exit(void)
{
	platform_driver_unregister(&tspdrv_driver);
}

late_initcall(tspdrv_init);
module_exit(tspdrv_exit);

/* Module info */
MODULE_AUTHOR("Immersion Corporation");
MODULE_DESCRIPTION("TouchSense Kernel Module");
MODULE_LICENSE("GPL v2");
