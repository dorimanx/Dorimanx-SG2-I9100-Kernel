/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef _FELICA_H
#define _FELICA_H

#include <linux/module.h>
#include <linux/fs.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/ioctl.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/tty.h>

/* jmodel */
#if defined(CONFIG_ARCH_EXYNOS)
#include <mach/gpio-exynos.h>
#include <plat/gpio-cfg.h>
#elif defined(CONFIG_ARCH_APQ8064)
#include "../../arch/arm/mach-msm/board-8064.h"
#include <mach/apq8064-gpio.h>
#include <mach/gpiomux.h>
#include <linux/barcode_emul.h>
#endif


/******************************************************************************
 * log
 ******************************************************************************/

/*#define FELICA_DEBUG*/





/******************************************************************************
 * config option
 ******************************************************************************/
#define P2P_FPGA_ALWAYS_ON




/******************************************************************************
 * common
 ******************************************************************************/

/* constant definition */
#define FELICA_MAJOR			10 /* same value as MISC_MAJOR */
#define FELICA_MINOR				0
/* first of the requested range of minor numbers */
#define FELICA_BASEMINOR		0
/* the number of minor numbers required */
#define FELICA_MINOR_COUNT		1
#define GPIO_VALUE_HIGH			1
#define GPIO_VALUE_LOW			0

/* function prototype */
static void felica_initialize_pin(void);
static void felica_finalize_pin(void);
static void felica_register_device(void);
static void felica_deregister_device(void);
static int __init felica_init(void);
static void __exit felica_exit(void);

/******************************************************************************
 * /dev/felica
 ******************************************************************************/

/* constant definition */
#define FELICA_UART_NAME				"felica"
#define FELICA_NL_NETLINK_USER			31
#define FELICA_NL_SUCCESS				1
#define FELICA_NL_EFAILED				2
#define FELICA_NL_REQ_OPEN				0x01
#define FELICA_NL_REQ_CLOSE				0x02
#define FELICA_NL_REQ_READ				0x03
#define FELICA_NL_REQ_WRITE				0x04
#define FELICA_NL_REQ_AVAIABLE			0x05
#define FELICA_NL_REQ_SYNC				0x06
#define FELICA_NL_RESPONCE				0xFE
#define FELICA_NL_CONNECT_MSG			0xFF
/* jmodel */
/*#define FELICA_NL_MSG_DATA_SIZE			4096*/
#define FELICA_NL_MSG_DATA_SIZE			N_TTY_BUF_SIZE
/*#define FELICA_NL_MSG_DATA_SIZE			4096*4*/
#define FELICA_NL_MSG_SIZE			(FELICA_NL_MSG_DATA_SIZE+4)

#define MSG_READ1_FLAGS_OFFSET			1
#define MSG_READ1_LEN_OFFSET			2
#define MSG_READ2_FLAGS_OFFSET			3
#define MSG_READ2_LEN_OFFSET			4
#define MSG_WRITE_FLAGS_OFFSET			5
#define MSG_WRITE_LEN_OFFSET			6
#define MSG_LOCK_ADDR_OFFSET			7
#define MSG_I2C_ADDR_OFFSET			8
#define MSG_DIAG_NAME_OFFSET			9
#define MSG_MFC_UID_FRONT_OFFSET		24
#define MSG_MFC_UID_BACK_OFFSET			25
#define MSG_MFL_UID_FRONT_OFFSET		26
#define MSG_MFL_UID_BACK_OFFSET			27
#define MSG_RWM_UID_FRONT_OFFSET		28
#define MSG_RWM_UID_BACK_OFFSET			29
#define MSG_ANT_ADDR_OFFSET			30
#define MSG_ANT_UID_FRONT_OFFSET		31
#define MSG_ANT_UID_BACK_OFFSET			32
#ifdef CONFIG_NFC_FELICA
#define MSG_NFC_UID_FRONT_OFFSET		33
#define MSG_NFC_UID_BACK_OFFSET			34
#endif /* CONFIG_NFC_FELICA */

#define FELICA_HLOS_IMG_TAMPER_FUSE    0
#define FELICA_SCM_SVC_FUSE            0x08
#define FELICA_SCM_BLOW_SW_FUSE_ID     0x01
#define FELICA_SCM_IS_SW_FUSE_BLOWN_ID 0x02

/* function prototype */
static void felica_uart_init(void);
static void felica_uart_exit(void);
static int felica_uart_open(struct inode *inode, struct file *file);
static int felica_uart_close(struct inode *inode, struct file *file);
static ssize_t felica_uart_read(struct file *file, char __user *buf,\
			size_t len, loff_t *ppos);
static ssize_t felica_uart_write(struct file *file, const char __user *data,\
			size_t len, loff_t *ppos);
static int felica_uart_sync(struct file *file, loff_t start, loff_t len,\
			int datasync);
static long felica_uart_ioctl(struct file *file, unsigned int cmd,\
			unsigned long arg);
static void felica_nl_init(void);
static void felica_nl_exit(void);
static int  felica_nl_send_msg(int len);
static void felica_nl_recv_msg(struct sk_buff *skb);
static void felica_nl_wait_ret_msg(void);
static void felica_set_felica_info(void);

#if defined(CONFIG_ARCH_EXYNOS)
static int felica_smc_read_oemflag(u32 ctrl_word, u32 *val);
static int felica_Cpu0(void);
static int felica_CpuAll(void);
#endif
static uint8_t felica_get_tamper_fuse_cmd(void);
#ifdef CONFIG_NFC_FELICA
static int felica_uart_open_wait_for_polling(void);
#endif /* CONFIG_NFC_FELICA */





/******************************************************************************
 * /dev/felica_pon
 ******************************************************************************/

/* constant definition */
#define FELICA_PON_NAME					"felica_pon"
/* jmodel */
#if defined(CONFIG_ARCH_EXYNOS)
#define GPIO_PINID_FELICA_PON			EXYNOS5410_GPJ2(7)
#elif defined(CONFIG_ARCH_APQ8064)
#define GPIO_PINID_FELICA_PON			FPGA_GPIO_FELICA_PON
#endif
#define FELICA_PON_DATA_LEN				1
#define FELICA_PON_WIRELESS				0
#define FELICA_PON_WIRED				1

/* function prototype */
static void felica_pon_init(void);
static void felica_pon_exit(void);
static int felica_pon_open(struct inode *inode, struct file *file);
static int felica_pon_close(struct inode *inode, struct file *file);
static ssize_t felica_pon_read(struct file *file, char __user *buf,\
			size_t len, loff_t *ppos);
static ssize_t felica_pon_write(struct file *file, const char __user *data, \
			size_t len, loff_t *ppos);





/******************************************************************************
 * felica_i2c_driver
 ******************************************************************************/

/* constant definition */
#define FELICA_I2C_NAME					"felica_i2c"

/* function prototype */
static void felica_i2c_init(void);
static void felica_i2c_exit(void);
static int felica_i2c_probe(struct i2c_client *client,\
				const struct i2c_device_id *devid);
static int felica_i2c_remove(struct i2c_client *client);

/******************************************************************************
 * /dev/felica_cen
 ******************************************************************************/

/* constant definition */
#define FELICA_CEN_NAME					"felica_cen"
#define FELICA_CEN_DATA_LEN				1
#define FELICA_CONTROL_LOCK_MASK			1
#define FELICA_CEN_LOCK					0
#define FELICA_CEN_UNLOCK				1
#define FELICA_CEN_SET_LOCK				0x80
#define FELICA_CEN_SET_UNLOCK				0x81

/* function prototype */
static void felica_cen_init(void);
static void felica_cen_exit(void);
static int felica_cen_open(struct inode *inode, struct file *file);
static int felica_cen_close(struct inode *inode, struct file *file);
static ssize_t felica_cen_read(struct file *file, char __user *buf,\
				size_t len, loff_t *ppos);
static ssize_t felica_cen_write(struct file *file, const char __user *data,\
				size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/felica_rfs
 ******************************************************************************/

/* constant definition */
#define FELICA_RFS_NAME					"felica_rfs"
/* jmodel */
#if defined(CONFIG_ARCH_EXYNOS)
#define GPIO_PINID_FELICA_RFS			EXYNOS5410_GPJ3(0)
#elif defined(CONFIG_ARCH_APQ8064)
#define GPIO_PINID_FELICA_RFS			GPIO_FELICA_RFS
#endif
#define FELICA_RFS_DATA_LEN				1
#define FELICA_RFS_STANDBY				0
#define FELICA_RFS_DETECTED				1

/* function prototype */
static void felica_rfs_init(void);
static void felica_rfs_exit(void);
static int felica_rfs_open(struct inode *inode, struct file *file);
static int felica_rfs_close(struct inode *inode, struct file *file);
static ssize_t felica_rfs_read(struct file *file, char __user *buf, \
			size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/felica_rws
 ******************************************************************************/

/* constant definition */
#define FELICA_RWS_NAME					"felica_rws"
#define FELICA_RWS_DATA_LEN				1
#define FELICA_RW_STATUS_INIT			0
#define FELICA_RW_STATUS_ENABLE			0
#define FELICA_RW_STATUS_DISABLE		1

/* function prototype */
static void felica_rws_init(void);
static void felica_rws_exit(void);
static int felica_rws_open(struct inode *inode, struct file *file);
static int felica_rws_close(struct inode *inode, struct file *file);
static ssize_t felica_rws_read(struct file *file, char __user *buf,\
			size_t len, loff_t *ppos);
static ssize_t felica_rws_write(struct file *file, const char __user *data, \
			size_t len, loff_t *ppos);





/******************************************************************************
 * /dev/felica_int_poll
 ******************************************************************************/

/* constant definition */
#define FELICA_INT_POLL_NAME			"felica_int_poll"
/* jmodel */
#if defined(CONFIG_ARCH_EXYNOS)
#define GPIO_PINID_FELICA_INT_REV03			EXYNOS5410_GPX1(3) /* rev0.3 */
#define GPIO_PINID_FELICA_INT_REV00			EXYNOS5410_GPX1(6) /* rev0.0 */
#elif defined(CONFIG_ARCH_APQ8064)
#define GPIO_PINID_FELICA_INT_REV03			GPIO_FELICA_INT
#endif
#define FELICA_INT_DATA_LEN				1
#define FELICA_INT_DELAY_TIME			3
#define FELICA_INT_LOW					0
#define FELICA_INT_HIGH					1

/* function prototype */
static irqreturn_t felica_int_irq_handler(int irq, void *dev_id);
static void felica_int_irq_work(struct work_struct *work);
static void felica_int_poll_init(void);
static void felica_int_poll_exit(void);
static int felica_int_poll_open(struct inode *inode, struct file *file);
static int felica_int_poll_close(struct inode *inode, struct file *file);
static ssize_t felica_int_poll_read(struct file *file, \
			char __user *buf, size_t len, loff_t *ppos);
static unsigned int felica_int_poll_poll(struct file *file, poll_table *wait);


/******************************************************************************
 * /dev/felica_uid
 ******************************************************************************/

/* constant definition */
#define FELICA_UID_NAME					"felica_uid"
#define DIAG_NAME_MAXSIZE				15
#define FELICA_MAGIC					0xF8
#define SET_FELICA_UID_DIAG	_IOW(FELICA_MAGIC, 4, void *)

/* function prototype */
static void felica_uid_init(void);
static void felica_uid_exit(void);
static int felica_uid_open(struct inode *inode, struct file *file);
static int felica_uid_close(struct inode *inode, struct file *file);
static long felica_uid_ioctl(struct file *file, unsigned int cmd, \
						unsigned long arg);
/******************************************************************************
 * /dev/felica_ant
 ******************************************************************************/

/* constant definition */
#define FELICA_ANT_NAME					"felica_ant"
#define FELICA_ANT_DATA_LEN				1

/* function prototype */
static void felica_ant_init(void);
static void felica_ant_exit(void);
static int felica_ant_open(struct inode *inode, struct file *file);
static int felica_ant_close(struct inode *inode, struct file *file);
static ssize_t felica_ant_read(struct file *file, char __user *buf,\
				size_t len, loff_t *ppos);
static ssize_t felica_ant_write(struct file *file, const char __user *data,\
				size_t len, loff_t *ppos);


/******************************************************************************
 *
 *
 *
 *
 * Add driver code for CXD2235AGG NFC/Felica IC
 *
 *
 *
 *
 ******************************************************************************/
#ifdef CONFIG_NFC_FELICA
/******************************************************************************
 * common
 ******************************************************************************/
/* function prototype */
static void snfc_register_device(void);
static void snfc_deregister_device(void);
static ssize_t snfc_cen_sts_init(void);
static int snfc_uid_check(void);

/******************************************************************************
 * /dev/snfc_hsel
 ******************************************************************************/

/* constant definition */
#if defined(CONFIG_ARCH_EXYNOS)
#define GPIO_PINID_NFC_HSEL				EXYNOS5410_GPX0(4)
#elif defined(CONFIG_ARCH_APQ8064)
#define GPIO_PINID_NFC_HSEL				FPGA_GPIO_FELICA_HSEL
#endif
#define HSEL_DEV_NAME					"snfc_hsel"
#define HSEL_DEV_COUNT					1

/* function prototype */
static int hsel_init(void);
static void hsel_exit(void);
static ssize_t hsel_read(struct file *file, char __user * buf, size_t len,\
                loff_t * ppos);
static ssize_t hsel_write(struct file *file, const char __user *data,\
                size_t len, loff_t *ppos);
static int hsel_open(struct inode *inode, struct file *file);
static int hsel_release(struct inode *inode, struct file *file);


/******************************************************************************
 * /dev/snfc_intu_poll
 ******************************************************************************/

/* constant definition */
#if defined(CONFIG_ARCH_EXYNOS)
#define GPIO_PINID_NFC_INTU_REV03				EXYNOS5410_GPX2(7) /* rev0.3 */
#define GPIO_PINID_NFC_INTU_REV00				EXYNOS5410_GPX1(5) /* rev0.0 */
#elif defined(CONFIG_ARCH_APQ8064)
#define GPIO_PINID_NFC_INTU_REV03				GPIO_FELICA_INTU
#define GPIO_PINID_NFC_INTU_REV06				GPIO_FELICA_INTU_REV06
#endif

#define INTU_POLL_DEV_NAME				"snfc_intu_poll"
#define INTU_POLL_DEV_COUNT				1
#define INTU_POLL_DELAY					3
#define INTU_POLL_DEV_LOW				0
#define INTU_POLL_DEV_HIGH				1

/* function prototype */
static irqreturn_t intu_poll_irq_handler(int irq, void *dev_id);
static void intu_poll_work_func(struct work_struct *work);
static int intu_poll_init(void);
static void intu_poll_exit(void);
static ssize_t intu_poll_read(struct file *file, char __user * buf,\
                size_t len, loff_t * ppos);
static int intu_poll_open(struct inode *inode, struct file *file);
static int intu_poll_release(struct inode *inode, struct file *file);


/******************************************************************************
 * /dev/snfc_available_poll
 ******************************************************************************/

/* constant definition */
#define AVAILABLE_POLL_DEV_NAME			"snfc_available_poll"
#define AVAILABLE_POLL_DEV_COUNT		1
#define AVAILABLE_POLL_DEV_MAGIC		0xFC
#define AVAILABLE_POLL_DEV_SET_PRE_LOCK	_IO(AVAILABLE_POLL_DEV_MAGIC, 8)

/* function prototype */
static int available_poll_init(void);
static void available_poll_exit(void);
static long available_poll_ioctl(struct file *file, unsigned int cmd,\
                unsigned long arg);
static ssize_t available_poll_read(struct file *file, char __user * buf,\
                size_t len, loff_t * ppos);
static int available_poll_open(struct inode *inode, struct file *file);
static int available_poll_release(struct inode *inode, struct file *file);


/******************************************************************************
 * /dev/snfc_rfs_poll
 ******************************************************************************/

/* constant definition */
#define GPIO_PINID_NFC_RFS_POLL			GPIO_PINID_FELICA_RFS
#define RFS_POLL_DEV_NAME				"snfc_rfs_poll"
#define RFS_POLL_DEV_COUNT				1
#define RFS_POLL_RETRY_CNT				3
#define RFS_POLL_SLEEP					1
#define RFS_POLL_DELAY					3
#define RFS_POLL_DEV_LOW				0
#define RFS_POLL_DEV_HIGH				1
#define RFS_POLL_RET_STS_LOW			0
#define RFS_POLL_RET_STS_HIGH			1

/* function prototype */
static irqreturn_t rfs_poll_irq_handler(int irq, void *dev_id);
static void rfs_poll_work_func(struct work_struct *work);
static int rfs_poll_init(void);
static void rfs_poll_exit(void);
static int rfs_poll_open(struct inode *inode, struct file *file);
static int rfs_poll_release(struct inode *inode, struct file *file);
static unsigned int rfs_poll_poll(struct file *file, poll_table *wait);
static ssize_t rfs_poll_read(struct file *file, char __user * buf, size_t len,
				loff_t * ppos);


/******************************************************************************
 * /dev/snfc_pon
 ******************************************************************************/

/* constant definition */
/* CXD2235POWER device	*/
#define GPIO_PINID_NFC_PON				GPIO_PINID_FELICA_PON
#define CXD2235_POWER_DEV_NAME			"snfc_pon"
#define CXD2235_POWER_DEV_COUNT			1

/* function prototype */
static int cxd2235power_init(void);
static void cxd2235power_exit(void);
static int cxd2235power_open(struct inode *inode, struct file *file);
static int cxd2235power_release(struct inode *inode, struct file *file);
static ssize_t cxd2235power_write(struct file *file, const char __user *data,\
				size_t len, loff_t *ppos);


/******************************************************************************
 * /dev/snfc_rfs
 ******************************************************************************/

/* constant definition */
#define SNFC_RFS_NAME					"snfc_rfs"
#if defined(CONFIG_ARCH_EXYNOS)
#define GPIO_PINID_SNFC_RFS				EXYNOS5410_GPJ3(0)
#elif defined(CONFIG_ARCH_APQ8064)
#define GPIO_PINID_SNFC_RFS				GPIO_PINID_FELICA_RFS
#endif
#define SNFC_RFS_DATA_LEN				1
#define SNFC_RFS_STANDBY				0
#define SNFC_RFS_DETECTED				1

/* function prototype */
static void snfc_rfs_init(void);
static void snfc_rfs_exit(void);
static int snfc_rfs_open(struct inode *inode, struct file *file);
static int snfc_rfs_close(struct inode *inode, struct file *file);
static ssize_t snfc_rfs_read(struct file *file, char __user *buf, \
			size_t len, loff_t *ppos);

/******************************************************************************
 * /dev/snfc_uart
 ******************************************************************************/

/* constant definition */
#define SNFC_UART_NAME					"snfc_uart"
#if defined(CONFIG_ARCH_EXYNOS)
#define UART_DEV_NAME					"/dev/ttySAC1"
#elif defined(CONFIG_ARCH_APQ8064)
#define UART_DEV_NAME					"/dev/ttyHSL2"
#endif

/* function prototype */
static void snfc_uart_init(void);
static void snfc_uart_exit(void);
static int snfc_uart_open(struct inode *inode, struct file *file);
static int snfc_uart_close(struct inode *inode, struct file *file);
static ssize_t snfc_uart_read(struct file *file, char __user *buf,\
			size_t len, loff_t *ppos);
static ssize_t snfc_uart_write(struct file *file, const char __user *data, \
			size_t len, loff_t *ppos);
static long snfc_uart_ioctl(struct file *file, unsigned int cmd, \
						unsigned long arg);
static unsigned int snfc_uart_poll(struct file *file, \
						struct poll_table_struct *poll_table);

/******************************************************************************
 * /dev/uartcc
 ******************************************************************************/
 
/* constant definition */
#define UARTCC_MAJOR					FELICA_MAJOR				/* 10 */
#define UARTCC_MINOR					FELICA_MINOR				/*  0 */
#define UARTCC_BASEMINOR				FELICA_BASEMINOR			/*  0 */
#define UARTCC_MINOR_COUNT				FELICA_MINOR_COUNT			/*  1 */
#define UARTCC_NAME						"uartcc"
#define UARTCC_MAGIC					FELICA_MAGIC 				/* 0xF8 */
#define UARTCC_SET_FELICA_STATUS		_IOW(UARTCC_MAGIC, 1, int)
#define UARTCC_SET_NFC_START_REQ		_IOW(UARTCC_MAGIC, 2, int)
#define UARTCC_GET_FELICA_STATUS		_IOR(UARTCC_MAGIC, 3, int)
#define UARTCC_GET_NFC_START_REQ		_IOR(UARTCC_MAGIC, 4, int)

/* felica usage state */
enum {
    UARTCC_FELICA_STATAUS_IDLE = 0,			/* idle			*/
    UARTCC_FELICA_STATAUS_WAIT_POLLING,		/* wait pollig	*/
    UARTCC_FELICA_STATAUS_IN_USE,			/* in use		*/
};

/* nfc start request */
enum {
    UARTCC_NFC_START_AUTOPOLLING = 1,
    UARTCC_NFC_START_ENDPROC = 0xFF,
};

#ifdef P2P_FPGA_ALWAYS_ON
enum {
	SNFC_START_SETTING = 0,
	SNFC_START_AUTOPOLL,
	SNFC_START_RW,
	SNFC_START_TARGET,
	SNFC_START_INTU,
	SNFC_START_WAITSIMBOOT,
};
#endif

/* function prototype */
static void uartcc_init(void);
static void uartcc_exit(void);
static int uartcc_open(struct inode *inode, struct file *file);
static int uartcc_close(struct inode *inode, struct file *file);
static long uartcc_ioctl(struct file *file, unsigned int cmd,\
			unsigned long arg);
static void uartcc_felica_start_request(void);
static void uartcc_set_felica_status(int status);
static int uartcc_get_felica_status(void);
static unsigned int uartcc_is_idle_status(void);



#endif /* CONFIG_NFC_FELICA */

#endif /* _FELICA_H */
