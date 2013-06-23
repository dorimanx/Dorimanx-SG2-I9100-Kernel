/* Synaptics Register Mapped Interface (RMI4) I2C Physical Layer Driver.
 * Copyright (c) 2007-2012, Synaptics Incorporated
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <asm/unaligned.h>
#include <mach/cpufreq.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/firmware.h>
#include "synaptics_i2c_rmi.h"
#include <linux/regulator/consumer.h>

#define F01_DEVICE_STATUS	0X0004

#define CHECKSUM_OFFSET 0x00
#define BOOTLOADER_VERSION_OFFSET 0x07
#define IMAGE_SIZE_OFFSET 0x08
#define CONFIG_SIZE_OFFSET 0x0C
#define PRODUCT_ID_OFFSET 0x10
#define PRODUCT_INFO_OFFSET 0x1E
#define FW_IMAGE_OFFSET 0x100
#define PRODUCT_ID_SIZE 10

#define BOOTLOADER_ID_OFFSET 0
#define FLASH_PROPERTIES_OFFSET 1
#define BLOCK_SIZE_OFFSET 2
#define BLOCK_COUNT_OFFSET 3

#define REG_MAP (1 << 0)
#define UNLOCKED (1 << 1)
#define HAS_CONFIG_ID (1 << 2)
#define HAS_PERM_CONFIG (1 << 3)
#define HAS_BL_CONFIG (1 << 4)
#define HAS_DISP_CONFIG (1 << 5)
#define HAS_CTRL1 (1 << 6)

#define BLOCK_NUMBER_OFFSET 0
#define BLOCK_DATA_OFFSET 1
#define FLASH_COMMAND_OFFSET 2
#define FLASH_STATUS_OFFSET 3

#define UI_CONFIG_AREA 0x00
#define PERM_CONFIG_AREA 0x01
#define BL_CONFIG_AREA 0x02
#define DISP_CONFIG_AREA 0x03

#define CMD_WRITE_FW_BLOCK 0x2
#define CMD_ERASE_ALL 0x3
#define CMD_READ_CONFIG_BLOCK 0x5
#define CMD_WRITE_CONFIG_BLOCK 0x6
#define CMD_ERASE_CONFIG 0x7
#define CMD_ERASE_BL_CONFIG 0x9
#define CMD_ERASE_DISP_CONFIG 0xa
#define CMD_ENABLE_FLASH_PROG 0xf

#define SLEEP_MODE_NORMAL (0x00)
#define SLEEP_MODE_SENSOR_SLEEP (0x01)
#define SLEEP_MODE_RESERVED0 (0x02)
#define SLEEP_MODE_RESERVED1 (0x03)

#define ENABLE_WAIT_MS (1 * 1000)
#define WRITE_WAIT_MS (3 * 1000)
#define ERASE_WAIT_MS (5 * 1000)

#define MIN_SLEEP_TIME_US 50
#define MAX_SLEEP_TIME_US 100

#if defined(CONFIG_MACH_JACTIVE_EUR) || defined(CONFIG_MACH_JACTIVE_ATT)
#define FW_SUPPORT_HSYNC03(x)	 (strncmp(x->product_id, "SY 03", 5) == 0)
#define FW_SUPPORT_HSYNC04(x)	 (strncmp(x->product_id, "SY 04", 5) == 0)
#define FW_NOT_SUPPORT_HSYNC(x)	 ((strncmp(x->product_id, "SY 01", 5) == 0) || (strncmp(x->product_id, "S5000B", 6) == 0) || (strncmp(x->product_id, "SY 02", 5) == 0))
#endif

static ssize_t fwu_sysfs_show_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t fwu_sysfs_store_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count);

static ssize_t fwu_sysfs_do_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_write_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_read_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_config_area_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_image_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count);

static ssize_t fwu_sysfs_block_size_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_firmware_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_configuration_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_perm_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_bl_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static ssize_t fwu_sysfs_disp_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf);

struct image_header {
	unsigned int checksum;
	unsigned int image_size;
	unsigned int config_size;
	unsigned char options;
	unsigned char bootloader_version;
	unsigned char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	unsigned char product_info[SYNAPTICS_RMI4_PRODUCT_INFO_SIZE];
};

struct pdt_properties {
	union {
		struct {
			unsigned char reserved_1:6;
			unsigned char has_bsr:1;
			unsigned char reserved_2:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f01_device_status {
	union {
		struct {
			unsigned char status_code:4;
			unsigned char reserved:2;
			unsigned char flash_prog:1;
			unsigned char unconfigured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f01_device_control {
	union {
		struct {
			unsigned char sleep_mode:2;
			unsigned char nosleep:1;
			unsigned char reserved:2;
			unsigned char charger_connected:1;
			unsigned char report_rate:1;
			unsigned char configured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_flash_status {
	union {
		struct {
			unsigned char status:6;
			unsigned char reserved:1;
			unsigned char program_enabled:1;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_fwu_handle {
	bool initialized;
	bool has_perm_config;
	bool has_bl_config;
	bool has_disp_config;
	unsigned int image_size;
	unsigned int data_pos;
	unsigned char *ext_data_source;
	unsigned char *read_config_buf;
	unsigned char intr_mask;
	unsigned char command;
	unsigned char bootloader_id[2];
	unsigned char flash_properties;
	unsigned char productinfo1;
	unsigned char productinfo2;
	unsigned short block_size;
	unsigned short fw_block_count;
	unsigned short config_block_count;
	unsigned short perm_config_block_count;
	unsigned short bl_config_block_count;
	unsigned short disp_config_block_count;
	unsigned short config_size;
	unsigned short config_area;
	char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];
	const unsigned char *firmware_data;
	const unsigned char *config_data;
	struct f34_flash_status flash_status;
	struct synaptics_rmi4_fn_desc f01_fd;
	struct synaptics_rmi4_fn_desc f34_fd;
	struct synaptics_rmi4_exp_fn_ptr *fn_ptr;
	struct synaptics_rmi4_data *rmi4_data;
};

static struct bin_attribute dev_attr_data = {
	.attr = {
		.name = "data",
		.mode = (S_IRUGO | S_IWUSR | S_IWGRP),
	},
	.size = 0,
	.read = fwu_sysfs_show_image,
	.write = fwu_sysfs_store_image,
};

static struct device_attribute attrs[] = {
	__ATTR(doreflash, S_IWUSR | S_IWGRP,
			synaptics_rmi4_show_error,
			fwu_sysfs_do_reflash_store),
	__ATTR(writeconfig, S_IWUSR | S_IWGRP,
			synaptics_rmi4_show_error,
			fwu_sysfs_write_config_store),
	__ATTR(readconfig, S_IWUSR | S_IWGRP,
			synaptics_rmi4_show_error,
			fwu_sysfs_read_config_store),
	__ATTR(configarea, S_IWUSR | S_IWGRP,
			synaptics_rmi4_show_error,
			fwu_sysfs_config_area_store),
	__ATTR(imagesize, S_IWUSR | S_IWGRP,
			synaptics_rmi4_show_error,
			fwu_sysfs_image_size_store),
	__ATTR(blocksize, S_IRUGO,
			fwu_sysfs_block_size_show,
			synaptics_rmi4_store_error),
	__ATTR(fwblockcount, S_IRUGO,
			fwu_sysfs_firmware_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(configblockcount, S_IRUGO,
			fwu_sysfs_configuration_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(permconfigblockcount, S_IRUGO,
			fwu_sysfs_perm_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(blconfigblockcount, S_IRUGO,
			fwu_sysfs_bl_config_block_count_show,
			synaptics_rmi4_store_error),
	__ATTR(dispconfigblockcount, S_IRUGO,
			fwu_sysfs_disp_config_block_count_show,
			synaptics_rmi4_store_error),
};

static struct synaptics_rmi4_fwu_handle *fwu;

static unsigned int extract_uint(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
			(unsigned int)ptr[1] * 0x100 +
			(unsigned int)ptr[2] * 0x10000 +
			(unsigned int)ptr[3] * 0x1000000;
}

static void parse_header(struct image_header *header,
		const unsigned char *fw_image)
{
	header->checksum = extract_uint(&fw_image[CHECKSUM_OFFSET]);
	header->bootloader_version = fw_image[BOOTLOADER_VERSION_OFFSET];
	header->image_size = extract_uint(&fw_image[IMAGE_SIZE_OFFSET]);
	header->config_size = extract_uint(&fw_image[CONFIG_SIZE_OFFSET]);
	memcpy(header->product_id, &fw_image[PRODUCT_ID_OFFSET],
			SYNAPTICS_RMI4_PRODUCT_ID_SIZE);
	header->product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE] = 0;
	memcpy(header->product_info, &fw_image[PRODUCT_INFO_OFFSET],
			SYNAPTICS_RMI4_PRODUCT_INFO_SIZE);

	return;
}

static int fwu_read_f01_device_status(struct f01_device_status *status)
{
	int retval;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f01_fd.data_base_addr,
			status->data,
			sizeof(status->data));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read F01 device status\n",
				__func__);
		return retval;
	}

	return 0;
}

static int fwu_read_f34_queries(void)
{
	int retval;
	unsigned char count;
	unsigned char buf[10];

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + BOOTLOADER_ID_OFFSET,
			fwu->bootloader_id,
			sizeof(fwu->bootloader_id));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read bootloader ID\n",
				__func__);
		return retval;
	}

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + FLASH_PROPERTIES_OFFSET,
			&fwu->flash_properties,
			sizeof(fwu->flash_properties));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read flash properties\n",
				__func__);
		return retval;
	}

	count = 4;

	if (fwu->flash_properties & HAS_PERM_CONFIG) {
		fwu->has_perm_config = 1;
		count += 2;
	}

	if (fwu->flash_properties & HAS_BL_CONFIG) {
		fwu->has_bl_config = 1;
		count += 2;
	}

	if (fwu->flash_properties & HAS_DISP_CONFIG) {
		fwu->has_disp_config = 1;
		count += 2;
	}

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + BLOCK_SIZE_OFFSET,
			buf,
			2);
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read block size info\n",
				__func__);
		return retval;
	}

	batohs(&fwu->block_size, &(buf[0]));

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.query_base_addr + BLOCK_COUNT_OFFSET,
			buf,
			count);
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read block count info\n",
				__func__);
		return retval;
	}

	batohs(&fwu->fw_block_count, &(buf[0]));
	batohs(&fwu->config_block_count, &(buf[2]));

	count = 4;

	if (fwu->has_perm_config) {
		batohs(&fwu->perm_config_block_count, &(buf[count]));
		count += 2;
	}

	if (fwu->has_bl_config) {
		batohs(&fwu->bl_config_block_count, &(buf[count]));
		count += 2;
	}

	if (fwu->has_disp_config)
		batohs(&fwu->disp_config_block_count, &(buf[count]));

	return 0;
}

static int fwu_read_f34_flash_status(void)
{
	int retval;
	unsigned char status;
	unsigned char command;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + FLASH_STATUS_OFFSET,
			&status,
			sizeof(status));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read flash status\n",
				__func__);
		return retval;
	}

	/* Program enabled bit not available - force bit to be set */
	fwu->flash_status.program_enabled = 1;
	fwu->flash_status.status = status & MASK_3BIT;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + FLASH_COMMAND_OFFSET,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read flash command\n",
				__func__);
		return retval;
	}

	fwu->command = command & MASK_4BIT;

	return 0;
}

static int fwu_write_f34_command(unsigned char cmd)
{
	int retval;
	unsigned char command = cmd;

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + FLASH_COMMAND_OFFSET,
			&command,
			sizeof(command));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write command 0x%02x\n",
				__func__, command);
		return retval;
	}

	fwu->command = cmd;

	return 0;
}

static int fwu_wait_for_idle(int timeout_ms)
{
	int count = 0;
	int timeout_count = ((timeout_ms * 1000) / MAX_SLEEP_TIME_US) + 1;

	do {
		usleep_range(MIN_SLEEP_TIME_US, MAX_SLEEP_TIME_US);

		count++;
		if (count == timeout_count)
			fwu_read_f34_flash_status();

		if ((fwu->command == 0x00) &&
				(fwu->flash_status.status == 0x00))
			return 0;
	} while (count < timeout_count);

	dev_err(&fwu->rmi4_data->i2c_client->dev,
			"%s: Timed out waiting for idle status\n",
			__func__);

	return -ETIMEDOUT;
}

static int fwu_scan_pdt(void)
{
	int retval;
	unsigned char ii;
	unsigned char intr_count = 0;
	unsigned char intr_off;
	unsigned char intr_src;
	unsigned short addr;
	bool f01found = false;
	bool f34found = false;
	struct synaptics_rmi4_fn_desc rmi_fd;

	for (addr = PDT_START; addr > PDT_END; addr -= PDT_ENTRY_SIZE) {
		retval = fwu->fn_ptr->read(fwu->rmi4_data,
				addr,
				(unsigned char *)&rmi_fd,
				sizeof(rmi_fd));
		if (retval < 0)
			return retval;

		if (rmi_fd.fn_number) {
			dev_dbg(&fwu->rmi4_data->i2c_client->dev,
					"%s: Found F%02x\n",
					__func__, rmi_fd.fn_number);
			switch (rmi_fd.fn_number) {
			case SYNAPTICS_RMI4_F01:
				f01found = true;
				fwu->f01_fd.query_base_addr =
						rmi_fd.query_base_addr;
				fwu->f01_fd.ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				fwu->f01_fd.data_base_addr =
						rmi_fd.data_base_addr;
				fwu->f01_fd.cmd_base_addr =
						rmi_fd.cmd_base_addr;
				break;
			case SYNAPTICS_RMI4_F34:
				f34found = true;
				fwu->f34_fd.query_base_addr =
						rmi_fd.query_base_addr;
				fwu->f34_fd.ctrl_base_addr =
						rmi_fd.ctrl_base_addr;
				fwu->f34_fd.data_base_addr =
						rmi_fd.data_base_addr;

				fwu->intr_mask = 0;
				intr_src = rmi_fd.intr_src_count;
				intr_off = intr_count % 8;
				for (ii = intr_off;
						ii < ((intr_src & MASK_3BIT) +
						intr_off);
						ii++) {
					fwu->intr_mask |= 1 << ii;
				}
				break;
			}
		} else {
			break;
		}

		intr_count += (rmi_fd.intr_src_count & MASK_3BIT);
	}

	if (!f01found || !f34found) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to find both F01 and F34\n",
				__func__);
		return -EINVAL;
	}

	return 0;
}

static int fwu_write_blocks(unsigned char *block_ptr, unsigned short block_cnt,
		unsigned char command)
{
	int retval;
	unsigned char block_offset[] = {0, 0};
	unsigned short block_num;

	block_offset[1] |= (fwu->config_area << 5);

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_NUMBER_OFFSET,
			block_offset,
			sizeof(block_offset));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write to block number registers\n",
				__func__);
		return retval;
	}

	for (block_num = 0; block_num < block_cnt; block_num++) {
		retval = fwu->fn_ptr->write(fwu->rmi4_data,
				fwu->f34_fd.data_base_addr + BLOCK_DATA_OFFSET,
				block_ptr,
				fwu->block_size);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to write block data (block %d)\n",
					__func__, block_num);
			return retval;
		}

		retval = fwu_write_f34_command(command);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to write command for block %d\n",
					__func__, block_num);
			return retval;
		}

		retval = fwu_wait_for_idle(WRITE_WAIT_MS);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to wait for idle status (block %d)\n",
					__func__, block_num);
			return retval;
		}

		block_ptr += fwu->block_size;
	}

	return 0;
}

static int fwu_write_firmware(void)
{
	return fwu_write_blocks((unsigned char *)fwu->firmware_data,
		fwu->fw_block_count, CMD_WRITE_FW_BLOCK);
}

static int fwu_write_configuration(void)
{
	return fwu_write_blocks((unsigned char *)fwu->config_data,
		fwu->config_block_count, CMD_WRITE_CONFIG_BLOCK);
}

static int fwu_write_bootloader_id(void)
{
	int retval;

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_DATA_OFFSET,
			fwu->bootloader_id,
			sizeof(fwu->bootloader_id));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write bootloader ID\n",
				__func__);
		return retval;
	}

	return 0;
}

static int fwu_enter_flash_prog(void)
{
	int retval;
	struct f01_device_status f01_device_status;
	struct f01_device_control f01_device_control;

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	retval = fwu_write_f34_command(CMD_ENABLE_FLASH_PROG);
	if (retval < 0)
		return retval;

	retval = fwu_wait_for_idle(ENABLE_WAIT_MS);
	if (retval < 0)
		return retval;

	if (!fwu->flash_status.program_enabled) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Program enabled bit not set\n",
				__func__);
		return -EINVAL;
	}

	retval = fwu_scan_pdt();
	if (retval < 0)
		return retval;

	retval = fwu_read_f01_device_status(&f01_device_status);
	if (retval < 0)
		return retval;

	if (!f01_device_status.flash_prog) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Not in flash prog mode\n",
				__func__);
		return -EINVAL;
	}

	retval = fwu_read_f34_queries();
	if (retval < 0)
		return retval;

	retval = fwu->fn_ptr->read(fwu->rmi4_data,
			fwu->f01_fd.ctrl_base_addr,
			f01_device_control.data,
			sizeof(f01_device_control.data));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to read F01 device control\n",
				__func__);
		return retval;
	}

	f01_device_control.nosleep = true;
	f01_device_control.sleep_mode = SLEEP_MODE_NORMAL;

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f01_fd.ctrl_base_addr,
			f01_device_control.data,
			sizeof(f01_device_control.data));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write F01 device control\n",
				__func__);
		return retval;
	}

	return retval;
}

static int fwu_do_reflash(void)
{
	int retval;

	retval = set_freq_limit(DVFS_TOUCH_ID,
				MIN_TOUCH_LIMIT);
	if (retval < 0)
		dev_err(&fwu->rmi4_data->i2c_client->dev,
			"%s: dvfs failed at fw update.\n",
			__func__);

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		return retval;

	dev_info(&fwu->rmi4_data->i2c_client->dev,
			"%s: Entered flash prog mode\n",
			__func__);

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	dev_info(&fwu->rmi4_data->i2c_client->dev,
			"%s: Bootloader ID written\n",
			__func__);

	retval = fwu_write_f34_command(CMD_ERASE_ALL);
	if (retval < 0)
		return retval;

	dev_info(&fwu->rmi4_data->i2c_client->dev,
			"%s: Erase all command written\n",
			__func__);

	retval = fwu_wait_for_idle(ERASE_WAIT_MS);
	if (retval < 0)
		return retval;

	dev_info(&fwu->rmi4_data->i2c_client->dev,
			"%s: Idle status detected\n",
			__func__);

	if (fwu->firmware_data) {
		retval = fwu_write_firmware();
		if (retval < 0)
			return retval;
		dev_info(&fwu->rmi4_data->i2c_client->dev, "%s: Firmware programmed\n",
			__func__);
	}

	if (fwu->config_data) {
		retval = fwu_write_configuration();
		if (retval < 0)
			return retval;
		dev_info(&fwu->rmi4_data->i2c_client->dev, "%s: Configuration programmed\n",
			__func__);
	}

	return retval;
}

/* TODO: Below function is added to check that firmware update is needed or not.
 * During development period, we need to support test firmware and various H/W
 * type such as A1/B0.... So Below conditions are very compex, maybe we need to
 * simplify this function not so far.
 * Synaptics's test firmware binary doesn't have Ic and firmware version.
 * in that case we skip update on booting time.
 * otherwise we forced run the update during UMS update..
 */
static bool fwu_check_skip_reflash(bool mode, bool factory_fw,
	const struct firmware *fw_entry)
{
	if (fwu->ext_data_source) {
		/* UMS case */
		int ic_revision_of_bin =
			(int)fwu->ext_data_source[IC_REVISION_BIN_OFFSET];
		int fw_version_of_bin =
			(int)fwu->ext_data_source[FW_VERSION_BIN_OFFSET];
		int fw_release_date_of_bin =
			(int)(fwu->ext_data_source[DATE_OF_FIRMWARE_BIN_OFFSET] << 8
				| fwu->ext_data_source[DATE_OF_FIRMWARE_BIN_OFFSET + 1]);

		/* A1 revision does not have revision info in firmware */
		if ((ic_revision_of_bin >> 4) != 0xB) {
#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
			/* Test firmware file does not have version infomation */
			if (!ic_revision_of_bin && !fw_version_of_bin
				&& !fw_release_date_of_bin) {
				dev_info(&fwu->rmi4_data->i2c_client->dev,
					"%s [UMS] : Firmware is Test firmware\n",
					__func__);
				ic_revision_of_bin = 0xB0;
			} else {
				ic_revision_of_bin = 0xA1;
			}
#else
			ic_revision_of_bin = 0xA1;
#endif
		}

		if (ic_revision_of_bin == 0xBF)
			dev_info(&fwu->rmi4_data->i2c_client->dev,
				"%s [UMS] : Firmware is Factory Test firmware\n",
				__func__);

		/* To prevent writing B0 firmware into A1 board */
		if ((fwu->rmi4_data->ic_revision_of_ic >> 4) != (ic_revision_of_bin >> 4)) {

			if ((!fwu->rmi4_data->fw_release_date_of_ic) && (!fwu->rmi4_data->fw_version_of_ic)) {
					dev_err(&fwu->rmi4_data->i2c_client->dev,
						"%s:[UMS] update: Revision is not macthed 0x%X(BIN) != 0x%X(IC), but IC FW is TEST FW, run force update\n",
						__func__, ic_revision_of_bin,
						fwu->rmi4_data->ic_revision_of_ic);
				return false;
			} else {
				dev_err(&fwu->rmi4_data->i2c_client->dev,
						"%s:[UMS] update: Revision is not macthed 0x%X(BIN) != 0x%X(IC)\n",
						__func__, ic_revision_of_bin,
						fwu->rmi4_data->ic_revision_of_ic);
				return true;
			}
		}
	} else {
		/* In kernel case */
		/* Read revision and firmware info from binary */
		fwu->rmi4_data->ic_revision_of_bin =
			(int)fw_entry->data[IC_REVISION_BIN_OFFSET];
		/* A1 revision does not have revision info in firmware */
		if ((fwu->rmi4_data->ic_revision_of_bin  >> 4) != 0xB)
			fwu->rmi4_data->ic_revision_of_bin = 0xA1;

		fwu->rmi4_data->fw_version_of_bin =
			(int)fw_entry->data[FW_VERSION_BIN_OFFSET];

		dev_info(&fwu->rmi4_data->i2c_client->dev,
			"%s: [FW size. revision, version] [%d, 0x%02X/0x%02X(BIN/IC), 0x%02X/0x%02X(BIN/IC)]\n",
			__func__, fw_entry->size,
			fwu->rmi4_data->ic_revision_of_bin,
			fwu->rmi4_data->ic_revision_of_ic,
			(int)fw_entry->data[FW_VERSION_BIN_OFFSET],
			fwu->rmi4_data->fw_version_of_ic);
		dev_info(&fwu->rmi4_data->i2c_client->dev,
			"%s: [Panel, mode, prog] [0x%02X, 0x%02X, 0x%02X]\n",
			__func__, fwu->rmi4_data->panel_revision,
			mode, fwu->rmi4_data->flash_prog_mode);

#if !defined(CONFIG_SAMSUNG_PRODUCT_SHIP)
		/* Test firmware file does not have version infomation */
		if ((!mode) && (!fwu->rmi4_data->flash_prog_mode)
			&& !fwu->rmi4_data->fw_version_of_ic
			&& !fwu->rmi4_data->fw_release_date_of_ic){
			dev_info(&fwu->rmi4_data->i2c_client->dev,
					"%s:[TEST] Firmware is Test firmware so do not need to update fw\n",
					__func__);
			return true;
		}
#endif
/*
		if (((fwu->rmi4_data->ic_revision_of_ic >> 4) != (fwu->rmi4_data->ic_revision_of_bin >> 4))) {
			dev_info(&fwu->rmi4_data->i2c_client->dev,
				"%s: IC revision is not equal. IC: 0x%x / BIN 0x%x\n",
				__func__,
				fwu->rmi4_data->ic_revision_of_ic,
				fwu->rmi4_data->ic_revision_of_bin);
			return true;
		}
*/
		if ((!mode) && (factory_fw) && (fwu->rmi4_data->ic_revision_of_ic == 0xB0) &&
			(fwu->rmi4_data->fw_version_of_bin <= fwu->rmi4_data->fw_version_of_ic)) {
			if (fwu->rmi4_data->board->recovery_mode) {
				dev_info(&fwu->rmi4_data->i2c_client->dev,
					"%s: recovery mode && factory bianry, run fw update to Factory FW\n",
					__func__);
			} else {
				dev_info(&fwu->rmi4_data->i2c_client->dev,
					"%s:[%s] %s BIN: 0x%02X /  %s FW: 0x%02X, do not update factory fw\n",
					__func__, mode ? "Force" : "Booting",
					factory_fw ? "FACTORY" : "USER", fwu->rmi4_data->fw_version_of_bin,
					fwu->rmi4_data->ic_revision_of_ic == 0xB0 ? "USER FW" : "FACTORY FW",
					fwu->rmi4_data->fw_version_of_ic);
				return true;
			}
		}

		if ((fwu->rmi4_data->fw_version_of_bin <= fwu->rmi4_data->fw_version_of_ic) &&
			(fwu->rmi4_data->ic_revision_of_ic == fwu->rmi4_data->ic_revision_of_bin) &&
			(!mode) && (!fwu->rmi4_data->flash_prog_mode)) {
			dev_info(&fwu->rmi4_data->i2c_client->dev,
				"%s: do not need to update fw\n", __func__);
			return true;
		} else {
			dev_info(&fwu->rmi4_data->i2c_client->dev,
				"%s: fw_ver (BIN:0x%x/IC:0x%x) ic_ver(BIN:0x%x/IC:0x%x) mode(%s) frog(%x)\n",
				__func__, fwu->rmi4_data->fw_version_of_bin,
				fwu->rmi4_data->fw_version_of_ic,
				fwu->rmi4_data->ic_revision_of_ic,
				fwu->rmi4_data->ic_revision_of_bin,
				mode ? "forced" : "booting",
				fwu->rmi4_data->flash_prog_mode);
		}
	}

	return false;
}

static int fwu_start_reflash(bool mode, bool factory_fw)
{
	int retval = 0, retry = 3;
	unsigned char device_status;
	struct image_header header;
	const unsigned char *fw_image;
	const struct firmware *fw_entry = NULL;

	if (fwu->rmi4_data->sensor_sleep || fwu->rmi4_data->touch_stopped) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Sensor sleeping or stopped\n",
				__func__);
		return -ENODEV;
	}

	fwu->rmi4_data->stay_awake = true;

	if (fwu->ext_data_source) {
		if (fwu_check_skip_reflash(mode, factory_fw, NULL)) {
			retval = -EINVAL;
			goto done;
		}
		fw_image = fwu->ext_data_source;
	} else {
		char fw_path[SYNAPTICS_MAX_FW_PATH];

		memset(&fw_path, 0, SYNAPTICS_MAX_FW_PATH);

		/* use factory test FW */
#if defined(CONFIG_MACH_JACTIVE_EUR) || defined(CONFIG_MACH_JACTIVE_ATT)
		if (factory_fw) {
			dev_info(&fwu->rmi4_data->i2c_client->dev,
				"%s: run fw update for FACTORY FIRMWARE\n",
				__func__);
			if (FW_NOT_SUPPORT_HSYNC(fwu))
				snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
					"%s", FW_IMAGE_NAME_B0_NON_HSYNC_FAC);
			else if (FW_SUPPORT_HSYNC03(fwu))
				snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
					"%s", FW_IMAGE_NAME_B0_HSYNC_FAC);
			else // FW_SUPPORT_HSYNC04(fwu)
				snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
					"%s", FW_IMAGE_NAME_B0_HSYNC04_FAC);
		} else {
		/* Read firmware according to ic revision */
			if ((fwu->rmi4_data->ic_revision_of_ic >> 4) == 0xB) {
				/* Read firmware according to panel ID */
				switch (fwu->rmi4_data->panel_revision) {
				case OCTA_PANEL_REVISION_34:
					if (FW_NOT_SUPPORT_HSYNC(fwu))
						snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
							"%s", FW_IMAGE_NAME_B0_NON_HSYNC);
					else if (FW_SUPPORT_HSYNC03(fwu)){
						snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
							"%s", FW_IMAGE_NAME_B0_HSYNC);
						}
					else // FW_SUPPORT_HSYNC04(fwu)
						snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
							"%s", FW_IMAGE_NAME_B0_HSYNC04);
					break;
				default:
					dev_info(&fwu->rmi4_data->i2c_client->dev,
						"%s: Do not request, not matched revision and FW.\n",
						__func__);
					goto out;
				}
			} else if ((fwu->rmi4_data->ic_revision_of_ic >> 4) == 0xA) {
				dev_info(&fwu->rmi4_data->i2c_client->dev,
					"%s: Do not request, not matched revision and FW.\n",
					__func__);
				goto out;
			} else { // force update when ic_revision_of_ic is NULL
				snprintf(fw_path, SYNAPTICS_MAX_FW_PATH, "%s", FW_IMAGE_NAME_B0_HSYNC);
				printk(KERN_ERR "%s, force update when ic_revision_of_ic is NULL\n", __func__);
				mode = true;
			}
		}
#else
		if (factory_fw) {
			dev_info(&fwu->rmi4_data->i2c_client->dev,
				"%s: run fw update for FACTORY FIRMWARE\n",
				__func__);
			if (fwu->rmi4_data->panel_revision < OCTA_PANEL_REVISION_51)
				snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
						"%s", FW_IMAGE_NAME_B0_FAC);
			else
				snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
						"%s", FW_IMAGE_NAME_B0_5_1_FAC);

		} else {
		/* Read firmware according to ic revision */
			if ((fwu->rmi4_data->ic_revision_of_ic >> 4) == 0xB) {
					/* Read firmware according to panel ID */
					switch (fwu->rmi4_data->panel_revision) {
					case OCTA_PANEL_REVISION_43:
						snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
							"%s", FW_IMAGE_NAME_B0_43);
						break;
					case OCTA_PANEL_REVISION_40:
						snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
							"%s", FW_IMAGE_NAME_B0_40);
						break;
					case OCTA_PANEL_REVISION_34:
						snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
							"%s", FW_IMAGE_NAME_B0_34);
						break;
					case OCTA_PANEL_REVISION_51:
						snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
							"%s", FW_IMAGE_NAME_B0_51);
						break;
					default:
						dev_info(&fwu->rmi4_data->i2c_client->dev,
							"%s: Do not request, not matched revision and FW.\n",
							__func__);
						goto out;
				}
			} else if ((fwu->rmi4_data->ic_revision_of_ic >> 4) == 0xA) {
				snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
					"%s", FW_IMAGE_NAME_A1);
			} else if (fwu->rmi4_data->ic_revision_of_ic == 0x00) {
				if ((strncmp(fwu->product_id, SYNAPTICS_PRODUCT_ID_B0, 5) == 0)
					 ||	(strncmp(fwu->product_id, SYNAPTICS_PRODUCT_ID_B0_SPAIR, 6) == 0))
					snprintf(fw_path, SYNAPTICS_MAX_FW_PATH,
						"%s", FW_IMAGE_NAME_B0_43);
			}
		}
#endif
		dev_info(&fwu->rmi4_data->i2c_client->dev,
				"%s: Requesting firmware image %s\n",
				__func__, fw_path);

		retval = request_firmware(&fw_entry, fw_path,
				&fwu->rmi4_data->i2c_client->dev);
		if (retval != 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Firmware image %s not available\n",
					__func__, fw_path);
			goto done;
		}

		if (fwu_check_skip_reflash(mode, factory_fw, fw_entry))
			goto done;
		fw_image = fw_entry->data;
	}

	parse_header(&header, fw_image);

	if (header.image_size)
		fwu->firmware_data = fw_image + FW_IMAGE_OFFSET;
	if (header.config_size) {
		fwu->config_data = fw_image + FW_IMAGE_OFFSET +
				header.image_size;
	}

	while (retry--) {

		mutex_lock(&(fwu->rmi4_data->rmi4_reflash_mutex));
		fwu->rmi4_data->doing_reflash = true;
		retval = fwu_do_reflash();
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to do reflash\n",
					__func__);
		}

		fwu->rmi4_data->reset_device(fwu->rmi4_data);
		fwu->rmi4_data->doing_reflash = false;
		mutex_unlock(&(fwu->rmi4_data->rmi4_reflash_mutex));

		fwu->fn_ptr->read(fwu->rmi4_data,
				F01_DEVICE_STATUS,
				&device_status,
				sizeof(device_status));
		if (!(device_status & (1 << 6)))
			break;
		else
			dev_info(&fwu->rmi4_data->i2c_client->dev,
				"%s: flash prog bit = %x, retry = %d\n",
				__func__, device_status & (1 << 6), retry);

	}
done:
	if (fw_entry)
		release_firmware(fw_entry);
out:
	retval = set_freq_limit(DVFS_TOUCH_ID, -1);
	if (retval < 0)
		dev_err(&fwu->rmi4_data->i2c_client->dev,
			"%s: in fw update, failed booster stop.\n",
			__func__);

	dev_info(&fwu->rmi4_data->i2c_client->dev, "%s: End of reflash process\n",
		 __func__);

	fwu->rmi4_data->stay_awake = false;

	return retval;
}

static int fwu_do_write_config(void)
{
	int retval;

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Entered flash prog mode\n",
			__func__);

	if (fwu->config_area == PERM_CONFIG_AREA) {
		fwu->config_block_count = fwu->perm_config_block_count;
		goto write_config;
	}

	retval = fwu_write_bootloader_id();
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Bootloader ID written\n",
			__func__);

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_CONFIG);
		break;
	case BL_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_BL_CONFIG);
		fwu->config_block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		retval = fwu_write_f34_command(CMD_ERASE_DISP_CONFIG);
		fwu->config_block_count = fwu->disp_config_block_count;
		break;
	}
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Erase command written\n",
			__func__);

	retval = fwu_wait_for_idle(ERASE_WAIT_MS);
	if (retval < 0)
		return retval;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Idle status detected\n",
			__func__);

write_config:
	retval = fwu_write_configuration();
	if (retval < 0)
		return retval;

	dev_info(&fwu->rmi4_data->i2c_client->dev, "%s: Config written\n",
		__func__);

	return retval;
}

static int fwu_start_write_config(void)
{
	int retval;
	struct image_header header;

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		break;
	case PERM_CONFIG_AREA:
		if (!fwu->has_perm_config)
			return -EINVAL;
		break;
	case BL_CONFIG_AREA:
		if (!fwu->has_bl_config)
			return -EINVAL;
		break;
	case DISP_CONFIG_AREA:
		if (!fwu->has_disp_config)
			return -EINVAL;
		break;
	default:
		return -EINVAL;
	}

	if (fwu->ext_data_source)
		fwu->config_data = fwu->ext_data_source;
	else
		return -EINVAL;

	if (fwu->config_area == UI_CONFIG_AREA) {
		parse_header(&header, fwu->ext_data_source);

		if (header.config_size) {
			fwu->config_data = fwu->ext_data_source +
					FW_IMAGE_OFFSET +
					header.image_size;
		} else {
			return -EINVAL;
		}
	}

	dev_info(&fwu->rmi4_data->i2c_client->dev, "%s: Start of write config process\n",
		__func__);

	fwu->rmi4_data->doing_reflash = true;
	retval = fwu_do_write_config();
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write config\n",
				__func__);
	}

	fwu->rmi4_data->reset_device(fwu->rmi4_data);
	fwu->rmi4_data->doing_reflash = false;

	dev_info(&fwu->rmi4_data->i2c_client->dev, "%s: End of write config process\n",
		__func__);

	return retval;
}

static int fwu_do_read_config(void)
{
	int retval;
	unsigned char block_offset[] = {0, 0};
	unsigned short block_num;
	unsigned short block_count;
	unsigned short index = 0;

	retval = fwu_enter_flash_prog();
	if (retval < 0)
		goto exit;

	dev_dbg(&fwu->rmi4_data->i2c_client->dev,
			"%s: Entered flash prog mode\n",
			__func__);

	switch (fwu->config_area) {
	case UI_CONFIG_AREA:
		block_count = fwu->config_block_count;
		break;
	case PERM_CONFIG_AREA:
		if (!fwu->has_perm_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->perm_config_block_count;
		break;
	case BL_CONFIG_AREA:
		if (!fwu->has_bl_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->bl_config_block_count;
		break;
	case DISP_CONFIG_AREA:
		if (!fwu->has_disp_config) {
			retval = -EINVAL;
			goto exit;
		}
		block_count = fwu->disp_config_block_count;
		break;
	default:
		retval = -EINVAL;
		goto exit;
	}

	fwu->config_size = fwu->block_size * block_count;

	kfree(fwu->read_config_buf);
	fwu->read_config_buf = kzalloc(fwu->config_size, GFP_KERNEL);

	block_offset[1] |= (fwu->config_area << 5);

	retval = fwu->fn_ptr->write(fwu->rmi4_data,
			fwu->f34_fd.data_base_addr + BLOCK_NUMBER_OFFSET,
			block_offset,
			sizeof(block_offset));
	if (retval < 0) {
		dev_err(&fwu->rmi4_data->i2c_client->dev,
				"%s: Failed to write to block number registers\n",
				__func__);
		goto exit;
	}

	for (block_num = 0; block_num < block_count; block_num++) {
		retval = fwu_write_f34_command(CMD_READ_CONFIG_BLOCK);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to write read config command\n",
					__func__);
			goto exit;
		}

		retval = fwu_wait_for_idle(WRITE_WAIT_MS);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to wait for idle status\n",
					__func__);
			goto exit;
		}

		retval = fwu->fn_ptr->read(fwu->rmi4_data,
				fwu->f34_fd.data_base_addr + BLOCK_DATA_OFFSET,
				&fwu->read_config_buf[index],
				fwu->block_size);
		if (retval < 0) {
			dev_err(&fwu->rmi4_data->i2c_client->dev,
					"%s: Failed to read block data (block %d)\n",
					__func__, block_num);
			goto exit;
		}

		index += fwu->block_size;
	}

exit:
	fwu->rmi4_data->reset_device(fwu->rmi4_data);

	return retval;
}

/*
 * fw_data : if exist external FW data.
 * mode : TRUE : force update, FALSE : auto update
 * factory_mode : TRUE : FACTORY FW update, FALSE : normal FW update
 */
int synaptics_fw_updater(unsigned char *fw_data, bool mode, bool factory_fw)
{
	int retval;

	if (!fwu)
		return -ENODEV;

	if (!fwu->initialized)
		return -ENODEV;

	fwu->ext_data_source = fw_data;
	fwu->config_area = UI_CONFIG_AREA;

	dev_info(&fwu->rmi4_data->i2c_client->dev,
			"%s: %s, %s_FW\n", __func__, mode ? "forced" : "booting",
			factory_fw ? "factory" : "user");
	retval = fwu_start_reflash(mode, factory_fw);

	return retval;
}
EXPORT_SYMBOL(synaptics_fw_updater);

static ssize_t fwu_sysfs_show_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (count < fwu->config_size) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Not enough space (%d bytes) in buffer\n",
				__func__, count);
		return -EINVAL;
	}

	memcpy(buf, fwu->read_config_buf, fwu->config_size);

	return fwu->config_size;
}

static ssize_t fwu_sysfs_store_image(struct file *data_file,
		struct kobject *kobj, struct bin_attribute *attributes,
		char *buf, loff_t pos, size_t count)
{
	memcpy((void *)(&fwu->ext_data_source[fwu->data_pos]),
			(const void *)buf,
			count);

	fwu->data_pos += count;

	return count;
}

static ssize_t fwu_sysfs_do_reflash_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (sscanf(buf, "%u", &input) != 1) {
		retval = -EINVAL;
		goto exit;
	}

	if (input != 1) {
		retval = -EINVAL;
		goto exit;
	}

	retval = synaptics_fw_updater(fwu->ext_data_source, true, false);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to do reflash\n",
				__func__);
		goto exit;
	}

	retval = count;

exit:
	kfree(fwu->ext_data_source);
	fwu->ext_data_source = NULL;
	return retval;
}

static ssize_t fwu_sysfs_write_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (sscanf(buf, "%u", &input) != 1) {
		retval = -EINVAL;
		goto exit;
	}

	if (input != 1) {
		retval = -EINVAL;
		goto exit;
	}

	retval = fwu_start_write_config();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to write config\n",
				__func__);
		goto exit;
	}

	retval = count;

exit:
	kfree(fwu->ext_data_source);
	fwu->ext_data_source = NULL;
	return retval;
}

static ssize_t fwu_sysfs_read_config_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned int input;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	if (sscanf(buf, "%u", &input) != 1)
		return -EINVAL;

	if (input != 1)
		return -EINVAL;

	retval = fwu_do_read_config();
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to read config\n",
				__func__);
		return retval;
	}

	return count;
}

static ssize_t fwu_sysfs_config_area_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long config_area;

	retval = kstrtoul(buf, 10, &config_area);
	if (retval)
		return retval;

	fwu->config_area = config_area;

	return count;
}

static ssize_t fwu_sysfs_image_size_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	int retval;
	unsigned long size;
	struct synaptics_rmi4_data *rmi4_data = fwu->rmi4_data;

	retval = kstrtoul(buf, 10, &size);
	if (retval)
		return retval;

	fwu->image_size = size;
	fwu->data_pos = 0;

	kfree(fwu->ext_data_source);
	fwu->ext_data_source = kzalloc(fwu->image_size, GFP_KERNEL);
	if (!fwu->ext_data_source) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for image data\n",
				__func__);
		return -ENOMEM;
	}

	return count;
}

static ssize_t fwu_sysfs_block_size_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->block_size);
}

static ssize_t fwu_sysfs_firmware_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->fw_block_count);
}

static ssize_t fwu_sysfs_configuration_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->config_block_count);
}

static ssize_t fwu_sysfs_perm_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->perm_config_block_count);
}

static ssize_t fwu_sysfs_bl_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->bl_config_block_count);
}

static ssize_t fwu_sysfs_disp_config_block_count_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", fwu->disp_config_block_count);
}

static void synaptics_rmi4_fwu_attn(struct synaptics_rmi4_data *rmi4_data,
		unsigned char intr_mask)
{
	if (fwu->intr_mask & intr_mask)
		fwu_read_f34_flash_status();

	return;
}

static int synaptics_rmi4_fwu_init(struct synaptics_rmi4_data *rmi4_data)
{
	int retval;
	unsigned char attr_count;
	int attr_count_num;
	struct pdt_properties pdt_props;

	fwu = kzalloc(sizeof(*fwu), GFP_KERNEL);
	if (!fwu) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for fwu\n",
				__func__);
		goto exit;
	}

	fwu->fn_ptr = kzalloc(sizeof(*(fwu->fn_ptr)), GFP_KERNEL);
	if (!fwu->fn_ptr) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to alloc mem for fn_ptr\n",
				__func__);
		retval = -ENOMEM;
		goto exit_free_fwu;
	}

	fwu->rmi4_data = rmi4_data;
	fwu->fn_ptr->read = rmi4_data->i2c_read;
	fwu->fn_ptr->write = rmi4_data->i2c_write;
	fwu->fn_ptr->enable = rmi4_data->irq_enable;

	retval = fwu->fn_ptr->read(rmi4_data,
			PDT_PROPS,
			pdt_props.data,
			sizeof(pdt_props.data));
	if (retval < 0) {
		dev_info(&rmi4_data->i2c_client->dev,
				"%s: Failed to read PDT properties, assuming 0x00\n",
				__func__);
	} else if (pdt_props.has_bsr) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Reflash for LTS not currently supported\n",
				__func__);
		goto exit_free_mem;
	}

	retval = fwu_scan_pdt();
	if (retval < 0)
		goto exit_free_mem;

	fwu->productinfo1 = rmi4_data->rmi4_mod_info.product_info[0];
	fwu->productinfo2 = rmi4_data->rmi4_mod_info.product_info[1];
	memcpy(fwu->product_id, rmi4_data->rmi4_mod_info.product_id_string,
			SYNAPTICS_RMI4_PRODUCT_ID_SIZE);
	fwu->product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE] = 0;

#if defined(CONFIG_MACH_JACTIVE_EUR) || defined(CONFIG_MACH_JACTIVE_ATT)
	/* Fortius Use only B Type. So read and use ic_revision_of_ic from IC */
#else
	/* Check the IC revision from product ID value
	 * we can check the ic revision with f34_ctrl_3 but to read production
	 * ID is more safity. because it is non-user writerble area.
	 */

	if ((strncmp(fwu->product_id, SYNAPTICS_PRODUCT_ID_B0, 5) == 0)
		 || (strncmp(fwu->product_id, SYNAPTICS_PRODUCT_ID_B0_SPAIR, 6) == 0))
			 dev_info(&rmi4_data->i2c_client->dev,
					"%s: ic revision is B0 type.\n",
					__func__);
	else
		rmi4_data->ic_revision_of_ic = 0xA1;
#endif

	dev_info(&rmi4_data->i2c_client->dev,
			"%s: [IC] [F01 product info, ID(revision)] [0x%04X 0x%04X, %s(0X%X)], PANEL : 0x%02X\n",
			__func__, fwu->productinfo1,
			fwu->productinfo2, fwu->product_id,
			rmi4_data->ic_revision_of_ic,
			rmi4_data->panel_revision);

	retval = fwu_read_f34_queries();
	if (retval < 0)
		goto exit_free_mem;

	fwu->initialized = true;

	retval = sysfs_create_bin_file(&rmi4_data->input_dev->dev.kobj,
			&dev_attr_data);
	if (retval < 0) {
		dev_err(&rmi4_data->i2c_client->dev,
				"%s: Failed to create sysfs bin file\n",
				__func__);
		goto exit_free_mem;
	}

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		retval = sysfs_create_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
		if (retval < 0) {
			dev_err(&rmi4_data->i2c_client->dev,
					"%s: Failed to create sysfs attributes\n",
					__func__);
			retval = -ENODEV;
			goto exit_remove_attrs;
		}
	}

	return 0;

exit_remove_attrs:
	attr_count_num = (int)attr_count;
	for (attr_count_num--; attr_count_num >= 0; attr_count_num--) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
			&attrs[attr_count].attr);
}

sysfs_remove_bin_file(&rmi4_data->input_dev->dev.kobj, &dev_attr_data);

exit_free_mem:
	kfree(fwu->fn_ptr);

exit_free_fwu:
	kfree(fwu);

exit:
	return 0;
}

static void synaptics_rmi4_fwu_remove(struct synaptics_rmi4_data *rmi4_data)
{
	unsigned char attr_count;

	sysfs_remove_bin_file(&rmi4_data->input_dev->dev.kobj, &dev_attr_data);

	for (attr_count = 0; attr_count < ARRAY_SIZE(attrs); attr_count++) {
		sysfs_remove_file(&rmi4_data->input_dev->dev.kobj,
				&attrs[attr_count].attr);
	}

	kfree(fwu->fn_ptr);
	kfree(fwu);

	return;
}

int rmi4_fw_update_module_register(void)
{
	int retval;

	retval = synaptics_rmi4_new_function(RMI_FW_UPDATER,
			synaptics_rmi4_fwu_init,
			synaptics_rmi4_fwu_remove,
			synaptics_rmi4_fwu_attn);

	return retval;
}
