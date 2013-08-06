/*
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#include <linux/jiffies.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/slab.h>

#include <scsi/scsi.h>
#include <scsi/scsi_cmnd.h>

#include <linux/firmware.h>

#include "usb.h"
#include "transport.h"
#include "protocol.h"
#include "debug.h"

MODULE_DESCRIPTION("Driver for ENE UB6250 reader");
MODULE_LICENSE("GPL");


/*
 * The table of devices
 */
#define UNUSUAL_DEV(id_vendor, id_product, bcdDeviceMin, bcdDeviceMax, \
		    vendorName, productName, useProtocol, useTransport, \
		    initFunction, flags) \
{ USB_DEVICE_VER(id_vendor, id_product, bcdDeviceMin, bcdDeviceMax), \
	.driver_info = (flags)|(USB_US_TYPE_STOR<<24) }

struct usb_device_id ene_ub6250_usb_ids[] = {
#	include "unusual_ene_ub6250.h"
	{ }		/* Terminating entry */
};
MODULE_DEVICE_TABLE(usb, ene_ub6250_usb_ids);

#undef UNUSUAL_DEV

/*
 * The flags table
 */
#define UNUSUAL_DEV(idVendor, idProduct, bcdDeviceMin, bcdDeviceMax, \
		    vendor_name, product_name, use_protocol, use_transport, \
		    init_function, Flags) \
{ \
	.vendorName = vendor_name,	\
	.productName = product_name,	\
	.useProtocol = use_protocol,	\
	.useTransport = use_transport,	\
	.initFunction = init_function,	\
}

static struct us_unusual_dev ene_ub6250_unusual_dev_list[] = {
#	include "unusual_ene_ub6250.h"
	{ }		/* Terminating entry */
};

#undef UNUSUAL_DEV



/* ENE bin code len */
#define ENE_BIN_CODE_LEN    0x800
/* EnE HW Register */
#define REG_CARD_STATUS     0xFF83
#define REG_HW_TRAP1        0xFF89

/* SRB Status */
#define SS_SUCCESS                  0x00      /* No Sense */
#define SS_NOT_READY                0x02
#define SS_MEDIUM_ERR               0x03
#define SS_HW_ERR                   0x04
#define SS_ILLEGAL_REQUEST          0x05
#define SS_UNIT_ATTENTION           0x06

/* ENE Load FW Pattern */
#define SD_INIT1_PATTERN   1
#define SD_INIT2_PATTERN   2
#define SD_RW_PATTERN      3
#define MS_INIT_PATTERN    4
#define MSP_RW_PATTERN     5
#define MS_RW_PATTERN      6
#define SM_INIT_PATTERN    7
#define SM_RW_PATTERN      8

#define FDIR_WRITE         0
#define FDIR_READ          1


struct SD_STATUS {
	u8    Insert:1;
	u8    Ready:1;
	u8    MediaChange:1;
	u8    IsMMC:1;
	u8    HiCapacity:1;
	u8    HiSpeed:1;
	u8    WtP:1;
	u8    Reserved:1;
};

struct MS_STATUS {
	u8    Insert:1;
	u8    Ready:1;
	u8    MediaChange:1;
	u8    IsMSPro:1;
	u8    IsMSPHG:1;
	u8    Reserved1:1;
	u8    WtP:1;
	u8    Reserved2:1;
};

struct SM_STATUS {
	u8    Insert:1;
	u8    Ready:1;
	u8    MediaChange:1;
	u8    Reserved:3;
	u8    WtP:1;
	u8    IsMS:1;
};


/* SD Block Length */
/* 2^9 = 512 Bytes, The HW maximum read/write data length */
#define SD_BLOCK_LEN  9

struct ene_ub6250_info {
	/* for 6250 code */
	struct SD_STATUS	SD_Status;
	struct MS_STATUS	MS_Status;
	struct SM_STATUS	SM_Status;

	/* ----- SD Control Data ---------------- */
	/*SD_REGISTER SD_Regs; */
	u16		SD_Block_Mult;
	u8		SD_READ_BL_LEN;
	u16		SD_C_SIZE;
	u8		SD_C_SIZE_MULT;

	/* SD/MMC New spec. */
	u8		SD_SPEC_VER;
	u8		SD_CSD_VER;
	u8		SD20_HIGH_CAPACITY;
	u32		HC_C_SIZE;
	u8		MMC_SPEC_VER;
	u8		MMC_BusWidth;
	u8		MMC_HIGH_CAPACITY;

	/*----- MS Control Data ---------------- */
	bool		MS_SWWP;
	u32		MSP_TotalBlock;
	/*MS_LibControl       MS_Lib;*/
	bool		MS_IsRWPage;
	u16		MS_Model;

	/*----- SM Control Data ---------------- */
	u8		SM_DeviceID;
	u8		SM_CardID;

	unsigned char	*testbuf;
	u8		BIN_FLAG;
	u32		bl_num;
	int		SrbStatus;

	/*------Power Managerment ---------------*/
	bool		Power_IsResum;
};

static int ene_sd_init(struct us_data *us);
static int ene_load_bincode(struct us_data *us, unsigned char flag);

static void ene_ub6250_info_destructor(void *extra)
{
	if (!extra)
		return;
}

static int ene_send_scsi_cmd(struct us_data *us, u8 fDir, void *buf, int use_sg)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	struct bulk_cs_wrap *bcs = (struct bulk_cs_wrap *) us->iobuf;

	int result;
	unsigned int residue;
	unsigned int cswlen = 0, partial = 0;
	unsigned int transfer_length = bcb->DataTransferLength;

	/* usb_stor_dbg(us, "transport --- ene_send_scsi_cmd\n"); */
	/* send cmd to out endpoint */
	result = usb_stor_bulk_transfer_buf(us, us->send_bulk_pipe,
					    bcb, US_BULK_CB_WRAP_LEN, NULL);
	if (result != USB_STOR_XFER_GOOD) {
		usb_stor_dbg(us, "send cmd to out endpoint fail ---\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	if (buf) {
		unsigned int pipe = fDir;

		if (fDir  == FDIR_READ)
			pipe = us->recv_bulk_pipe;
		else
			pipe = us->send_bulk_pipe;

		/* Bulk */
		if (use_sg) {
			result = usb_stor_bulk_srb(us, pipe, us->srb);
		} else {
			result = usb_stor_bulk_transfer_sg(us, pipe, buf,
						transfer_length, 0, &partial);
		}
		if (result != USB_STOR_XFER_GOOD) {
			usb_stor_dbg(us, "data transfer fail ---\n");
			return USB_STOR_TRANSPORT_ERROR;
		}
	}

	/* Get CSW for device status */
	result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe, bcs,
					    US_BULK_CS_WRAP_LEN, &cswlen);

	if (result == USB_STOR_XFER_SHORT && cswlen == 0) {
		usb_stor_dbg(us, "Received 0-length CSW; retrying...\n");
		result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe,
					    bcs, US_BULK_CS_WRAP_LEN, &cswlen);
	}

	if (result == USB_STOR_XFER_STALLED) {
		/* get the status again */
		usb_stor_dbg(us, "Attempting to get CSW (2nd try)...\n");
		result = usb_stor_bulk_transfer_buf(us, us->recv_bulk_pipe,
						bcs, US_BULK_CS_WRAP_LEN, NULL);
	}

	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	/* check bulk status */
	residue = le32_to_cpu(bcs->Residue);

	/* try to compute the actual residue, based on how much data
	 * was really transferred and what the device tells us */
	if (residue && !(us->fflags & US_FL_IGNORE_RESIDUE)) {
		residue = min(residue, transfer_length);
		if (us->srb != NULL)
			scsi_set_resid(us->srb, max(scsi_get_resid(us->srb),
								(int)residue));
	}

	if (bcs->Status != US_BULK_STAT_OK)
		return USB_STOR_TRANSPORT_ERROR;

	return USB_STOR_TRANSPORT_GOOD;
}

static int sd_scsi_test_unit_ready(struct us_data *us, struct scsi_cmnd *srb)
{
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	if (info->SD_Status.Insert && info->SD_Status.Ready)
		return USB_STOR_TRANSPORT_GOOD;
	else {
		ene_sd_init(us);
		return USB_STOR_TRANSPORT_GOOD;
	}

	return USB_STOR_TRANSPORT_GOOD;
}

static int sd_scsi_inquiry(struct us_data *us, struct scsi_cmnd *srb)
{
	unsigned char data_ptr[36] = {
		0x00, 0x80, 0x02, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x55,
		0x53, 0x42, 0x32, 0x2E, 0x30, 0x20, 0x20, 0x43, 0x61,
		0x72, 0x64, 0x52, 0x65, 0x61, 0x64, 0x65, 0x72, 0x20,
		0x20, 0x20, 0x20, 0x20, 0x20, 0x30, 0x31, 0x30, 0x30 };

	usb_stor_set_xfer_buf(data_ptr, 36, srb);
	return USB_STOR_TRANSPORT_GOOD;
}

static int sd_scsi_mode_sense(struct us_data *us, struct scsi_cmnd *srb)
{
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;
	unsigned char mediaNoWP[12] = {
		0x0b, 0x00, 0x00, 0x08, 0x00, 0x00,
		0x71, 0xc0, 0x00, 0x00, 0x02, 0x00 };
	unsigned char mediaWP[12]   = {
		0x0b, 0x00, 0x80, 0x08, 0x00, 0x00,
		0x71, 0xc0, 0x00, 0x00, 0x02, 0x00 };

	if (info->SD_Status.WtP)
		usb_stor_set_xfer_buf(mediaWP, 12, srb);
	else
		usb_stor_set_xfer_buf(mediaNoWP, 12, srb);


	return USB_STOR_TRANSPORT_GOOD;
}

static int sd_scsi_read_capacity(struct us_data *us, struct scsi_cmnd *srb)
{
	u32   bl_num;
	u16    bl_len;
	unsigned int offset = 0;
	unsigned char    buf[8];
	struct scatterlist *sg = NULL;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	usb_stor_dbg(us, "sd_scsi_read_capacity\n");
	if (info->SD_Status.HiCapacity) {
		bl_len = 0x200;
		if (info->SD_Status.IsMMC)
			bl_num = info->HC_C_SIZE-1;
		else
			bl_num = (info->HC_C_SIZE + 1) * 1024 - 1;
	} else {
		bl_len = 1<<(info->SD_READ_BL_LEN);
		bl_num = info->SD_Block_Mult * (info->SD_C_SIZE + 1)
				* (1 << (info->SD_C_SIZE_MULT + 2)) - 1;
	}
	info->bl_num = bl_num;
	usb_stor_dbg(us, "bl_len = %x\n", bl_len);
	usb_stor_dbg(us, "bl_num = %x\n", bl_num);

	/*srb->request_bufflen = 8; */
	buf[0] = (bl_num >> 24) & 0xff;
	buf[1] = (bl_num >> 16) & 0xff;
	buf[2] = (bl_num >> 8) & 0xff;
	buf[3] = (bl_num >> 0) & 0xff;
	buf[4] = (bl_len >> 24) & 0xff;
	buf[5] = (bl_len >> 16) & 0xff;
	buf[6] = (bl_len >> 8) & 0xff;
	buf[7] = (bl_len >> 0) & 0xff;

	usb_stor_access_xfer_buf(buf, 8, srb, &sg, &offset, TO_XFER_BUF);

	return USB_STOR_TRANSPORT_GOOD;
}

static int sd_scsi_read(struct us_data *us, struct scsi_cmnd *srb)
{
	int result;
	unsigned char *cdb = srb->cmnd;
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	u32 bn = ((cdb[2] << 24) & 0xff000000) | ((cdb[3] << 16) & 0x00ff0000) |
		 ((cdb[4] << 8) & 0x0000ff00) | ((cdb[5] << 0) & 0x000000ff);
	u16 blen = ((cdb[7] << 8) & 0xff00) | ((cdb[8] << 0) & 0x00ff);
	u32 bnByte = bn * 0x200;
	u32 blenByte = blen * 0x200;

	if (bn > info->bl_num)
		return USB_STOR_TRANSPORT_ERROR;

	result = ene_load_bincode(us, SD_RW_PATTERN);
	if (result != USB_STOR_XFER_GOOD) {
		usb_stor_dbg(us, "Load SD RW pattern Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	if (info->SD_Status.HiCapacity)
		bnByte = bn;

	/* set up the command wrapper */
	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = blenByte;
	bcb->Flags  = 0x80;
	bcb->CDB[0] = 0xF1;
	bcb->CDB[5] = (unsigned char)(bnByte);
	bcb->CDB[4] = (unsigned char)(bnByte>>8);
	bcb->CDB[3] = (unsigned char)(bnByte>>16);
	bcb->CDB[2] = (unsigned char)(bnByte>>24);

	result = ene_send_scsi_cmd(us, FDIR_READ, scsi_sglist(srb), 1);
	return result;
}

static int sd_scsi_write(struct us_data *us, struct scsi_cmnd *srb)
{
	int result;
	unsigned char *cdb = srb->cmnd;
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	u32 bn = ((cdb[2] << 24) & 0xff000000) | ((cdb[3] << 16) & 0x00ff0000) |
		 ((cdb[4] << 8) & 0x0000ff00) | ((cdb[5] << 0) & 0x000000ff);
	u16 blen = ((cdb[7] << 8) & 0xff00) | ((cdb[8] << 0) & 0x00ff);
	u32 bnByte = bn * 0x200;
	u32 blenByte = blen * 0x200;

	if (bn > info->bl_num)
		return USB_STOR_TRANSPORT_ERROR;

	result = ene_load_bincode(us, SD_RW_PATTERN);
	if (result != USB_STOR_XFER_GOOD) {
		usb_stor_dbg(us, "Load SD RW pattern Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	if (info->SD_Status.HiCapacity)
		bnByte = bn;

	/* set up the command wrapper */
	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = blenByte;
	bcb->Flags  = 0x00;
	bcb->CDB[0] = 0xF0;
	bcb->CDB[5] = (unsigned char)(bnByte);
	bcb->CDB[4] = (unsigned char)(bnByte>>8);
	bcb->CDB[3] = (unsigned char)(bnByte>>16);
	bcb->CDB[2] = (unsigned char)(bnByte>>24);

	result = ene_send_scsi_cmd(us, FDIR_WRITE, scsi_sglist(srb), 1);
	return result;
}

static int ene_get_card_type(struct us_data *us, u16 index, void *buf)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	int result;

	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength	= 0x01;
	bcb->Flags			= 0x80;
	bcb->CDB[0]			= 0xED;
	bcb->CDB[2]			= (unsigned char)(index>>8);
	bcb->CDB[3]			= (unsigned char)index;

	result = ene_send_scsi_cmd(us, FDIR_READ, buf, 0);
	return result;
}

static int ene_get_card_status(struct us_data *us, u8 *buf)
{
	u16 tmpreg;
	u32 reg4b;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	/*US_DEBUGP("transport --- ENE_ReadSDReg\n");*/
	reg4b = *(u32 *)&buf[0x18];
	info->SD_READ_BL_LEN = (u8)((reg4b >> 8) & 0x0f);

	tmpreg = (u16) reg4b;
	reg4b = *(u32 *)(&buf[0x14]);
	if (info->SD_Status.HiCapacity && !info->SD_Status.IsMMC)
		info->HC_C_SIZE = (reg4b >> 8) & 0x3fffff;

	info->SD_C_SIZE = ((tmpreg & 0x03) << 10) | (u16)(reg4b >> 22);
	info->SD_C_SIZE_MULT = (u8)(reg4b >> 7)  & 0x07;
	if (info->SD_Status.HiCapacity && info->SD_Status.IsMMC)
		info->HC_C_SIZE = *(u32 *)(&buf[0x100]);

	if (info->SD_READ_BL_LEN > SD_BLOCK_LEN) {
		info->SD_Block_Mult = 1 << (info->SD_READ_BL_LEN-SD_BLOCK_LEN);
		info->SD_READ_BL_LEN = SD_BLOCK_LEN;
	} else {
		info->SD_Block_Mult = 1;
	}

	return USB_STOR_TRANSPORT_GOOD;
}

static int ene_load_bincode(struct us_data *us, unsigned char flag)
{
	int err;
	char *fw_name = NULL;
	unsigned char *buf = NULL;
	const struct firmware *sd_fw = NULL;
	int result = USB_STOR_TRANSPORT_ERROR;
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	if (info->BIN_FLAG == flag)
		return USB_STOR_TRANSPORT_GOOD;

	switch (flag) {
	/* For SD */
	case SD_INIT1_PATTERN:
		US_DEBUGP("SD_INIT1_PATTERN\n");
		fw_name = "ene-ub6250/sd_init1.bin";
		break;
	case SD_INIT2_PATTERN:
		US_DEBUGP("SD_INIT2_PATTERN\n");
		fw_name = "ene-ub6250/sd_init2.bin";
		break;
	case SD_RW_PATTERN:
		US_DEBUGP("SD_RDWR_PATTERN\n");
		fw_name = "ene-ub6250/sd_rdwr.bin";
		break;
	default:
		US_DEBUGP("----------- Unknown PATTERN ----------\n");
		goto nofw;
	}

	err = request_firmware(&sd_fw, fw_name, &us->pusb_dev->dev);
	if (err) {
		US_DEBUGP("load firmware %s failed\n", fw_name);
		goto nofw;
	}
	buf = kmalloc(sd_fw->size, GFP_KERNEL);
	if (buf == NULL) {
		US_DEBUGP("Malloc memory for fireware failed!\n");
		goto nofw;
	}
	memcpy(buf, sd_fw->data, sd_fw->size);
	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = sd_fw->size;
	bcb->Flags = 0x00;
	bcb->CDB[0] = 0xEF;

	result = ene_send_scsi_cmd(us, FDIR_WRITE, buf, 0);
	info->BIN_FLAG = flag;
	kfree(buf);

nofw:
	if (sd_fw != NULL) {
		release_firmware(sd_fw);
		sd_fw = NULL;
	}

	return result;
}

static int ene_sd_init(struct us_data *us)
{
	int result;
	u8  buf[0x200];
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	US_DEBUGP("transport --- ENE_SDInit\n");
	/* SD Init Part-1 */
	result = ene_load_bincode(us, SD_INIT1_PATTERN);
	if (result != USB_STOR_XFER_GOOD) {
		US_DEBUGP("Load SD Init Code Part-1 Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->Flags = 0x80;
	bcb->CDB[0] = 0xF2;

	result = ene_send_scsi_cmd(us, FDIR_READ, NULL, 0);
	if (result != USB_STOR_XFER_GOOD) {
		US_DEBUGP("Execution SD Init Code Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	/* SD Init Part-2 */
	result = ene_load_bincode(us, SD_INIT2_PATTERN);
	if (result != USB_STOR_XFER_GOOD) {
		US_DEBUGP("Load SD Init Code Part-2 Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = 0x200;
	bcb->Flags              = 0x80;
	bcb->CDB[0]             = 0xF1;

	result = ene_send_scsi_cmd(us, FDIR_READ, &buf, 0);
	if (result != USB_STOR_XFER_GOOD) {
		US_DEBUGP("Execution SD Init Code Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	info->SD_Status =  *(struct SD_STATUS *)&buf[0];
	if (info->SD_Status.Insert && info->SD_Status.Ready) {
		ene_get_card_status(us, (unsigned char *)&buf);
		US_DEBUGP("Insert     = %x\n", info->SD_Status.Insert);
		US_DEBUGP("Ready      = %x\n", info->SD_Status.Ready);
		US_DEBUGP("IsMMC      = %x\n", info->SD_Status.IsMMC);
		US_DEBUGP("HiCapacity = %x\n", info->SD_Status.HiCapacity);
		US_DEBUGP("HiSpeed    = %x\n", info->SD_Status.HiSpeed);
		US_DEBUGP("WtP        = %x\n", info->SD_Status.WtP);
	} else {
		US_DEBUGP("SD Card Not Ready --- %x\n", buf[0]);
		return USB_STOR_TRANSPORT_ERROR;
	}
	return USB_STOR_TRANSPORT_GOOD;
}


static int ene_init(struct us_data *us)
{
	int result;
	u8  misc_reg03 = 0;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *)(us->extra);

	result = ene_get_card_type(us, REG_CARD_STATUS, &misc_reg03);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	if (misc_reg03 & 0x01) {
		if (!info->SD_Status.Ready) {
			result = ene_sd_init(us);
			if (result != USB_STOR_XFER_GOOD)
				return USB_STOR_TRANSPORT_ERROR;
		}
	}

	return result;
}

/*----- sd_scsi_irp() ---------*/
static int sd_scsi_irp(struct us_data *us, struct scsi_cmnd *srb)
{
	int    result;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *)us->extra;

	info->SrbStatus = SS_SUCCESS;
	switch (srb->cmnd[0]) {
	case TEST_UNIT_READY:
		result = sd_scsi_test_unit_ready(us, srb);
		break; /* 0x00 */
	case INQUIRY:
		result = sd_scsi_inquiry(us, srb);
		break; /* 0x12 */
	case MODE_SENSE:
		result = sd_scsi_mode_sense(us, srb);
		break; /* 0x1A */
	/*
	case START_STOP:
		result = SD_SCSI_Start_Stop(us, srb);
		break; //0x1B
	*/
	case READ_CAPACITY:
		result = sd_scsi_read_capacity(us, srb);
		break; /* 0x25 */
	case READ_10:
		result = sd_scsi_read(us, srb);
		break; /* 0x28 */
	case WRITE_10:
		result = sd_scsi_write(us, srb);
		break; /* 0x2A */
	default:
		info->SrbStatus = SS_ILLEGAL_REQUEST;
		result = USB_STOR_TRANSPORT_FAILED;
		break;
	}
	return result;
}

static int ene_transport(struct scsi_cmnd *srb, struct us_data *us)
{
	int result = 0;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *)(us->extra);

	/*US_DEBUG(usb_stor_show_command(srb)); */
	scsi_set_resid(srb, 0);
	if (unlikely(!info->SD_Status.Ready))
		result = ene_init(us);
	else
		result = sd_scsi_irp(us, srb);

	return 0;
}


<<<<<<< HEAD
=======
static void ms_lib_free_allocatedarea(struct us_data *us)
{
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	ms_lib_free_writebuf(us); /* Free MS_Lib.pagemap */
	ms_lib_free_logicalmap(us); /* kfree MS_Lib.Phy2LogMap and MS_Lib.Log2PhyMap */

	/* set struct us point flag to 0 */
	info->MS_Lib.flags = 0;
	info->MS_Lib.BytesPerSector = 0;
	info->MS_Lib.SectorsPerCylinder = 0;

	info->MS_Lib.cardType = 0;
	info->MS_Lib.blockSize = 0;
	info->MS_Lib.PagesPerBlock = 0;

	info->MS_Lib.NumberOfPhyBlock = 0;
	info->MS_Lib.NumberOfLogBlock = 0;
}


static int ms_lib_alloc_writebuf(struct us_data *us)
{
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	info->MS_Lib.wrtblk = (u16)-1;

	info->MS_Lib.blkpag = kmalloc(info->MS_Lib.PagesPerBlock * info->MS_Lib.BytesPerSector, GFP_KERNEL);
	info->MS_Lib.blkext = kmalloc(info->MS_Lib.PagesPerBlock * sizeof(struct ms_lib_type_extdat), GFP_KERNEL);

	if ((info->MS_Lib.blkpag == NULL) || (info->MS_Lib.blkext == NULL)) {
		ms_lib_free_writebuf(us);
		return (u32)-1;
	}

	ms_lib_clear_writebuf(us);

return 0;
}

static int ms_lib_force_setlogical_pair(struct us_data *us, u16 logblk, u16 phyblk)
{
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	if (logblk == MS_LB_NOT_USED)
		return 0;

	if ((logblk >= info->MS_Lib.NumberOfLogBlock) ||
		(phyblk >= info->MS_Lib.NumberOfPhyBlock))
		return (u32)-1;

	info->MS_Lib.Phy2LogMap[phyblk] = logblk;
	info->MS_Lib.Log2PhyMap[logblk] = phyblk;

	return 0;
}

static int ms_read_copyblock(struct us_data *us, u16 oldphy, u16 newphy,
			u16 PhyBlockAddr, u8 PageNum, unsigned char *buf, u16 len)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	int result;

	/* printk(KERN_INFO "MS_ReaderCopyBlock --- PhyBlockAddr = %x,
		PageNum = %x\n", PhyBlockAddr, PageNum); */
	result = ene_load_bincode(us, MS_RW_PATTERN);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = 0x200*len;
	bcb->Flags = 0x00;
	bcb->CDB[0] = 0xF0;
	bcb->CDB[1] = 0x08;
	bcb->CDB[4] = (unsigned char)(oldphy);
	bcb->CDB[3] = (unsigned char)(oldphy>>8);
	bcb->CDB[2] = 0; /* (BYTE)(oldphy>>16) */
	bcb->CDB[7] = (unsigned char)(newphy);
	bcb->CDB[6] = (unsigned char)(newphy>>8);
	bcb->CDB[5] = 0; /* (BYTE)(newphy>>16) */
	bcb->CDB[9] = (unsigned char)(PhyBlockAddr);
	bcb->CDB[8] = (unsigned char)(PhyBlockAddr>>8);
	bcb->CDB[10] = PageNum;

	result = ene_send_scsi_cmd(us, FDIR_WRITE, buf, 0);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	return USB_STOR_TRANSPORT_GOOD;
}

static int ms_read_eraseblock(struct us_data *us, u32 PhyBlockAddr)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	int result;
	u32 bn = PhyBlockAddr;

	/* printk(KERN_INFO "MS --- ms_read_eraseblock,
			PhyBlockAddr = %x\n", PhyBlockAddr); */
	result = ene_load_bincode(us, MS_RW_PATTERN);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = 0x200;
	bcb->Flags = US_BULK_FLAG_IN;
	bcb->CDB[0] = 0xF2;
	bcb->CDB[1] = 0x06;
	bcb->CDB[4] = (unsigned char)(bn);
	bcb->CDB[3] = (unsigned char)(bn>>8);
	bcb->CDB[2] = (unsigned char)(bn>>16);

	result = ene_send_scsi_cmd(us, FDIR_READ, NULL, 0);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	return USB_STOR_TRANSPORT_GOOD;
}

static int ms_lib_check_disableblock(struct us_data *us, u16 PhyBlock)
{
	unsigned char *PageBuf = NULL;
	u16 result = MS_STATUS_SUCCESS;
	u16 blk, index = 0;
	struct ms_lib_type_extdat extdat;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	PageBuf = kmalloc(MS_BYTES_PER_PAGE, GFP_KERNEL);
	if (PageBuf == NULL) {
		result = MS_NO_MEMORY_ERROR;
		goto exit;
	}

	ms_read_readpage(us, PhyBlock, 1, (u32 *)PageBuf, &extdat);
	do {
		blk = be16_to_cpu(PageBuf[index]);
		if (blk == MS_LB_NOT_USED)
			break;
		if (blk == info->MS_Lib.Log2PhyMap[0]) {
			result = MS_ERROR_FLASH_READ;
			break;
		}
		index++;
	} while (1);

exit:
	kfree(PageBuf);
	return result;
}

static int ms_lib_setacquired_errorblock(struct us_data *us, u16 phyblk)
{
	u16 log;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	if (phyblk >= info->MS_Lib.NumberOfPhyBlock)
		return (u32)-1;

	log = info->MS_Lib.Phy2LogMap[phyblk];

	if (log < info->MS_Lib.NumberOfLogBlock)
		info->MS_Lib.Log2PhyMap[log] = MS_LB_NOT_USED;

	if (info->MS_Lib.Phy2LogMap[phyblk] != MS_LB_INITIAL_ERROR)
		info->MS_Lib.Phy2LogMap[phyblk] = MS_LB_ACQUIRED_ERROR;

	return 0;
}

static int ms_lib_overwrite_extra(struct us_data *us, u32 PhyBlockAddr,
				u8 PageNum, u8 OverwriteFlag)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	int result;

	/* printk("MS --- MS_LibOverwriteExtra,
		PhyBlockAddr = %x, PageNum = %x\n", PhyBlockAddr, PageNum); */
	result = ene_load_bincode(us, MS_RW_PATTERN);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = 0x4;
	bcb->Flags = US_BULK_FLAG_IN;
	bcb->CDB[0] = 0xF2;
	bcb->CDB[1] = 0x05;
	bcb->CDB[5] = (unsigned char)(PageNum);
	bcb->CDB[4] = (unsigned char)(PhyBlockAddr);
	bcb->CDB[3] = (unsigned char)(PhyBlockAddr>>8);
	bcb->CDB[2] = (unsigned char)(PhyBlockAddr>>16);
	bcb->CDB[6] = OverwriteFlag;
	bcb->CDB[7] = 0xFF;
	bcb->CDB[8] = 0xFF;
	bcb->CDB[9] = 0xFF;

	result = ene_send_scsi_cmd(us, FDIR_READ, NULL, 0);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	return USB_STOR_TRANSPORT_GOOD;
}

static int ms_lib_error_phyblock(struct us_data *us, u16 phyblk)
{
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	if (phyblk >= info->MS_Lib.NumberOfPhyBlock)
		return MS_STATUS_ERROR;

	ms_lib_setacquired_errorblock(us, phyblk);

	if (ms_lib_iswritable(info))
		return ms_lib_overwrite_extra(us, phyblk, 0, (u8)(~MS_REG_OVR_BKST & BYTE_MASK));

	return MS_STATUS_SUCCESS;
}

static int ms_lib_erase_phyblock(struct us_data *us, u16 phyblk)
{
	u16 log;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	if (phyblk >= info->MS_Lib.NumberOfPhyBlock)
		return MS_STATUS_ERROR;

	log = info->MS_Lib.Phy2LogMap[phyblk];

	if (log < info->MS_Lib.NumberOfLogBlock)
		info->MS_Lib.Log2PhyMap[log] = MS_LB_NOT_USED;

	info->MS_Lib.Phy2LogMap[phyblk] = MS_LB_NOT_USED;

	if (ms_lib_iswritable(info)) {
		switch (ms_read_eraseblock(us, phyblk)) {
		case MS_STATUS_SUCCESS:
			info->MS_Lib.Phy2LogMap[phyblk] = MS_LB_NOT_USED_ERASED;
			return MS_STATUS_SUCCESS;
		case MS_ERROR_FLASH_ERASE:
		case MS_STATUS_INT_ERROR:
			ms_lib_error_phyblock(us, phyblk);
			return MS_ERROR_FLASH_ERASE;
		case MS_STATUS_ERROR:
		default:
			ms_lib_ctrl_set(info, MS_LIB_CTRL_RDONLY); /* MS_LibCtrlSet will used by ENE_MSInit ,need check, and why us to info*/
			ms_lib_setacquired_errorblock(us, phyblk);
			return MS_STATUS_ERROR;
		}
	}

	ms_lib_setacquired_errorblock(us, phyblk);

	return MS_STATUS_SUCCESS;
}

static int ms_lib_read_extra(struct us_data *us, u32 PhyBlock,
				u8 PageNum, struct ms_lib_type_extdat *ExtraDat)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	int result;
	u8 ExtBuf[4];

	/* printk("MS_LibReadExtra --- PhyBlock = %x, PageNum = %x\n", PhyBlock, PageNum); */
	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = 0x4;
	bcb->Flags      = US_BULK_FLAG_IN;
	bcb->CDB[0]     = 0xF1;
	bcb->CDB[1]     = 0x03;
	bcb->CDB[5]     = (unsigned char)(PageNum);
	bcb->CDB[4]     = (unsigned char)(PhyBlock);
	bcb->CDB[3]     = (unsigned char)(PhyBlock>>8);
	bcb->CDB[2]     = (unsigned char)(PhyBlock>>16);
	bcb->CDB[6]     = 0x01;

	result = ene_send_scsi_cmd(us, FDIR_READ, &ExtBuf, 0);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	ExtraDat->reserved = 0;
	ExtraDat->intr     = 0x80;  /* Not yet, waiting for fireware support */
	ExtraDat->status0  = 0x10;  /* Not yet, waiting for fireware support */
	ExtraDat->status1  = 0x00;  /* Not yet, waiting for fireware support */
	ExtraDat->ovrflg   = ExtBuf[0];
	ExtraDat->mngflg   = ExtBuf[1];
	ExtraDat->logadr   = memstick_logaddr(ExtBuf[2], ExtBuf[3]);

	return USB_STOR_TRANSPORT_GOOD;
}

static int ms_libsearch_block_from_physical(struct us_data *us, u16 phyblk)
{
	u16 Newblk;
	u16 blk;
	struct ms_lib_type_extdat extdat; /* need check */
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;


	if (phyblk >= info->MS_Lib.NumberOfPhyBlock)
		return MS_LB_ERROR;

	for (blk = phyblk + 1; blk != phyblk; blk++) {
		if ((blk & MS_PHYSICAL_BLOCKS_PER_SEGMENT_MASK) == 0)
			blk -= MS_PHYSICAL_BLOCKS_PER_SEGMENT;

		Newblk = info->MS_Lib.Phy2LogMap[blk];
		if (info->MS_Lib.Phy2LogMap[blk] == MS_LB_NOT_USED_ERASED) {
			return blk;
		} else if (info->MS_Lib.Phy2LogMap[blk] == MS_LB_NOT_USED) {
			switch (ms_lib_read_extra(us, blk, 0, &extdat)) {
			case MS_STATUS_SUCCESS:
			case MS_STATUS_SUCCESS_WITH_ECC:
				break;
			case MS_NOCARD_ERROR:
				return MS_NOCARD_ERROR;
			case MS_STATUS_INT_ERROR:
				return MS_LB_ERROR;
			case MS_ERROR_FLASH_READ:
			default:
				ms_lib_setacquired_errorblock(us, blk);
				continue;
			} /* End switch */

			if ((extdat.ovrflg & MS_REG_OVR_BKST) != MS_REG_OVR_BKST_OK) {
				ms_lib_setacquired_errorblock(us, blk);
				continue;
			}

			switch (ms_lib_erase_phyblock(us, blk)) {
			case MS_STATUS_SUCCESS:
				return blk;
			case MS_STATUS_ERROR:
				return MS_LB_ERROR;
			case MS_ERROR_FLASH_ERASE:
			default:
				ms_lib_error_phyblock(us, blk);
				break;
			}
		}
	} /* End for */

	return MS_LB_ERROR;
}
static int ms_libsearch_block_from_logical(struct us_data *us, u16 logblk)
{
	u16 phyblk;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	phyblk = ms_libconv_to_physical(info, logblk);
	if (phyblk >= MS_LB_ERROR) {
		if (logblk >= info->MS_Lib.NumberOfLogBlock)
			return MS_LB_ERROR;

		phyblk = (logblk + MS_NUMBER_OF_BOOT_BLOCK) / MS_LOGICAL_BLOCKS_PER_SEGMENT;
		phyblk *= MS_PHYSICAL_BLOCKS_PER_SEGMENT;
		phyblk += MS_PHYSICAL_BLOCKS_PER_SEGMENT - 1;
	}

	return ms_libsearch_block_from_physical(us, phyblk);
}

static int ms_scsi_test_unit_ready(struct us_data *us, struct scsi_cmnd *srb)
{
	struct ene_ub6250_info *info = (struct ene_ub6250_info *)(us->extra);

	/* pr_info("MS_SCSI_Test_Unit_Ready\n"); */
	if (info->MS_Status.Insert && info->MS_Status.Ready) {
		return USB_STOR_TRANSPORT_GOOD;
	} else {
		ene_ms_init(us);
		return USB_STOR_TRANSPORT_GOOD;
	}

	return USB_STOR_TRANSPORT_GOOD;
}

static int ms_scsi_inquiry(struct us_data *us, struct scsi_cmnd *srb)
{
	/* pr_info("MS_SCSI_Inquiry\n"); */
	unsigned char data_ptr[36] = {
		0x00, 0x80, 0x02, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x55,
		0x53, 0x42, 0x32, 0x2E, 0x30, 0x20, 0x20, 0x43, 0x61,
		0x72, 0x64, 0x52, 0x65, 0x61, 0x64, 0x65, 0x72, 0x20,
		0x20, 0x20, 0x20, 0x20, 0x20, 0x30, 0x31, 0x30, 0x30};

	usb_stor_set_xfer_buf(data_ptr, 36, srb);
	return USB_STOR_TRANSPORT_GOOD;
}

static int ms_scsi_mode_sense(struct us_data *us, struct scsi_cmnd *srb)
{
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;
	unsigned char mediaNoWP[12] = {
		0x0b, 0x00, 0x00, 0x08, 0x00, 0x00,
		0x71, 0xc0, 0x00, 0x00, 0x02, 0x00 };
	unsigned char mediaWP[12]   = {
		0x0b, 0x00, 0x80, 0x08, 0x00, 0x00,
		0x71, 0xc0, 0x00, 0x00, 0x02, 0x00 };

	if (info->MS_Status.WtP)
		usb_stor_set_xfer_buf(mediaWP, 12, srb);
	else
		usb_stor_set_xfer_buf(mediaNoWP, 12, srb);

	return USB_STOR_TRANSPORT_GOOD;
}

static int ms_scsi_read_capacity(struct us_data *us, struct scsi_cmnd *srb)
{
	u32   bl_num;
	u16    bl_len;
	unsigned int offset = 0;
	unsigned char    buf[8];
	struct scatterlist *sg = NULL;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	usb_stor_dbg(us, "ms_scsi_read_capacity\n");
	bl_len = 0x200;
	if (info->MS_Status.IsMSPro)
		bl_num = info->MSP_TotalBlock - 1;
	else
		bl_num = info->MS_Lib.NumberOfLogBlock * info->MS_Lib.blockSize * 2 - 1;

	info->bl_num = bl_num;
	usb_stor_dbg(us, "bl_len = %x\n", bl_len);
	usb_stor_dbg(us, "bl_num = %x\n", bl_num);

	/*srb->request_bufflen = 8; */
	buf[0] = (bl_num >> 24) & 0xff;
	buf[1] = (bl_num >> 16) & 0xff;
	buf[2] = (bl_num >> 8) & 0xff;
	buf[3] = (bl_num >> 0) & 0xff;
	buf[4] = (bl_len >> 24) & 0xff;
	buf[5] = (bl_len >> 16) & 0xff;
	buf[6] = (bl_len >> 8) & 0xff;
	buf[7] = (bl_len >> 0) & 0xff;

	usb_stor_access_xfer_buf(buf, 8, srb, &sg, &offset, TO_XFER_BUF);

	return USB_STOR_TRANSPORT_GOOD;
}

static void ms_lib_phy_to_log_range(u16 PhyBlock, u16 *LogStart, u16 *LogEnde)
{
	PhyBlock /= MS_PHYSICAL_BLOCKS_PER_SEGMENT;

	if (PhyBlock) {
		*LogStart = MS_LOGICAL_BLOCKS_IN_1ST_SEGMENT + (PhyBlock - 1) * MS_LOGICAL_BLOCKS_PER_SEGMENT;/*496*/
		*LogEnde = *LogStart + MS_LOGICAL_BLOCKS_PER_SEGMENT;/*496*/
	} else {
		*LogStart = 0;
		*LogEnde = MS_LOGICAL_BLOCKS_IN_1ST_SEGMENT;/*494*/
	}
}

static int ms_lib_read_extrablock(struct us_data *us, u32 PhyBlock,
	u8 PageNum, u8 blen, void *buf)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	int     result;

	/* printk("MS_LibReadExtraBlock --- PhyBlock = %x,
		PageNum = %x, blen = %x\n", PhyBlock, PageNum, blen); */

	/* Read Extra Data */
	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = 0x4 * blen;
	bcb->Flags      = US_BULK_FLAG_IN;
	bcb->CDB[0]     = 0xF1;
	bcb->CDB[1]     = 0x03;
	bcb->CDB[5]     = (unsigned char)(PageNum);
	bcb->CDB[4]     = (unsigned char)(PhyBlock);
	bcb->CDB[3]     = (unsigned char)(PhyBlock>>8);
	bcb->CDB[2]     = (unsigned char)(PhyBlock>>16);
	bcb->CDB[6]     = blen;

	result = ene_send_scsi_cmd(us, FDIR_READ, buf, 0);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	return USB_STOR_TRANSPORT_GOOD;
}

static int ms_lib_scan_logicalblocknumber(struct us_data *us, u16 btBlk1st)
{
	u16 PhyBlock, newblk, i;
	u16 LogStart, LogEnde;
	struct ms_lib_type_extdat extdat;
	u8 buf[0x200];
	u32 count = 0, index = 0;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	for (PhyBlock = 0; PhyBlock < info->MS_Lib.NumberOfPhyBlock;) {
		ms_lib_phy_to_log_range(PhyBlock, &LogStart, &LogEnde);

		for (i = 0; i < MS_PHYSICAL_BLOCKS_PER_SEGMENT; i++, PhyBlock++) {
			switch (ms_libconv_to_logical(info, PhyBlock)) {
			case MS_STATUS_ERROR:
				continue;
			default:
				break;
			}

			if (count == PhyBlock) {
				ms_lib_read_extrablock(us, PhyBlock, 0, 0x80, &buf);
				count += 0x80;
			}
			index = (PhyBlock % 0x80) * 4;

			extdat.ovrflg = buf[index];
			extdat.mngflg = buf[index+1];
			extdat.logadr = memstick_logaddr(buf[index+2], buf[index+3]);

			if ((extdat.ovrflg & MS_REG_OVR_BKST) != MS_REG_OVR_BKST_OK) {
				ms_lib_setacquired_errorblock(us, PhyBlock);
				continue;
			}

			if ((extdat.mngflg & MS_REG_MNG_ATFLG) == MS_REG_MNG_ATFLG_ATTBL) {
				ms_lib_erase_phyblock(us, PhyBlock);
				continue;
			}

			if (extdat.logadr != MS_LB_NOT_USED) {
				if ((extdat.logadr < LogStart) || (LogEnde <= extdat.logadr)) {
					ms_lib_erase_phyblock(us, PhyBlock);
					continue;
				}

				newblk = ms_libconv_to_physical(info, extdat.logadr);

				if (newblk != MS_LB_NOT_USED) {
					if (extdat.logadr == 0) {
						ms_lib_set_logicalpair(us, extdat.logadr, PhyBlock);
						if (ms_lib_check_disableblock(us, btBlk1st)) {
							ms_lib_set_logicalpair(us, extdat.logadr, newblk);
							continue;
						}
					}

					ms_lib_read_extra(us, newblk, 0, &extdat);
					if ((extdat.ovrflg & MS_REG_OVR_UDST) == MS_REG_OVR_UDST_UPDATING) {
						ms_lib_erase_phyblock(us, PhyBlock);
						continue;
					} else {
						ms_lib_erase_phyblock(us, newblk);
					}
				}

				ms_lib_set_logicalpair(us, extdat.logadr, PhyBlock);
			}
		}
	} /* End for ... */

	return MS_STATUS_SUCCESS;
}


static int ms_scsi_read(struct us_data *us, struct scsi_cmnd *srb)
{
	int result;
	unsigned char *cdb = srb->cmnd;
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	u32 bn = ((cdb[2] << 24) & 0xff000000) | ((cdb[3] << 16) & 0x00ff0000) |
		((cdb[4] << 8) & 0x0000ff00) | ((cdb[5] << 0) & 0x000000ff);
	u16 blen = ((cdb[7] << 8) & 0xff00) | ((cdb[8] << 0) & 0x00ff);
	u32 blenByte = blen * 0x200;

	if (bn > info->bl_num)
		return USB_STOR_TRANSPORT_ERROR;

	if (info->MS_Status.IsMSPro) {
		result = ene_load_bincode(us, MSP_RW_PATTERN);
		if (result != USB_STOR_XFER_GOOD) {
			usb_stor_dbg(us, "Load MPS RW pattern Fail !!\n");
			return USB_STOR_TRANSPORT_ERROR;
		}

		/* set up the command wrapper */
		memset(bcb, 0, sizeof(struct bulk_cb_wrap));
		bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
		bcb->DataTransferLength = blenByte;
		bcb->Flags  = US_BULK_FLAG_IN;
		bcb->CDB[0] = 0xF1;
		bcb->CDB[1] = 0x02;
		bcb->CDB[5] = (unsigned char)(bn);
		bcb->CDB[4] = (unsigned char)(bn>>8);
		bcb->CDB[3] = (unsigned char)(bn>>16);
		bcb->CDB[2] = (unsigned char)(bn>>24);

		result = ene_send_scsi_cmd(us, FDIR_READ, scsi_sglist(srb), 1);
	} else {
		void *buf;
		int offset = 0;
		u16 phyblk, logblk;
		u8 PageNum;
		u16 len;
		u32 blkno;

		buf = kmalloc(blenByte, GFP_KERNEL);
		if (buf == NULL)
			return USB_STOR_TRANSPORT_ERROR;

		result = ene_load_bincode(us, MS_RW_PATTERN);
		if (result != USB_STOR_XFER_GOOD) {
			pr_info("Load MS RW pattern Fail !!\n");
			result = USB_STOR_TRANSPORT_ERROR;
			goto exit;
		}

		logblk  = (u16)(bn / info->MS_Lib.PagesPerBlock);
		PageNum = (u8)(bn % info->MS_Lib.PagesPerBlock);

		while (1) {
			if (blen > (info->MS_Lib.PagesPerBlock-PageNum))
				len = info->MS_Lib.PagesPerBlock-PageNum;
			else
				len = blen;

			phyblk = ms_libconv_to_physical(info, logblk);
			blkno  = phyblk * 0x20 + PageNum;

			/* set up the command wrapper */
			memset(bcb, 0, sizeof(struct bulk_cb_wrap));
			bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
			bcb->DataTransferLength = 0x200 * len;
			bcb->Flags  = US_BULK_FLAG_IN;
			bcb->CDB[0] = 0xF1;
			bcb->CDB[1] = 0x02;
			bcb->CDB[5] = (unsigned char)(blkno);
			bcb->CDB[4] = (unsigned char)(blkno>>8);
			bcb->CDB[3] = (unsigned char)(blkno>>16);
			bcb->CDB[2] = (unsigned char)(blkno>>24);

			result = ene_send_scsi_cmd(us, FDIR_READ, buf+offset, 0);
			if (result != USB_STOR_XFER_GOOD) {
				pr_info("MS_SCSI_Read --- result = %x\n", result);
				result = USB_STOR_TRANSPORT_ERROR;
				goto exit;
			}

			blen -= len;
			if (blen <= 0)
				break;
			logblk++;
			PageNum = 0;
			offset += MS_BYTES_PER_PAGE*len;
		}
		usb_stor_set_xfer_buf(buf, blenByte, srb);
exit:
		kfree(buf);
	}
	return result;
}

static int ms_scsi_write(struct us_data *us, struct scsi_cmnd *srb)
{
	int result;
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	unsigned char *cdb = srb->cmnd;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	u32 bn = ((cdb[2] << 24) & 0xff000000) |
			((cdb[3] << 16) & 0x00ff0000) |
			((cdb[4] << 8) & 0x0000ff00) |
			((cdb[5] << 0) & 0x000000ff);
	u16 blen = ((cdb[7] << 8) & 0xff00) | ((cdb[8] << 0) & 0x00ff);
	u32 blenByte = blen * 0x200;

	if (bn > info->bl_num)
		return USB_STOR_TRANSPORT_ERROR;

	if (info->MS_Status.IsMSPro) {
		result = ene_load_bincode(us, MSP_RW_PATTERN);
		if (result != USB_STOR_XFER_GOOD) {
			pr_info("Load MSP RW pattern Fail !!\n");
			return USB_STOR_TRANSPORT_ERROR;
		}

		/* set up the command wrapper */
		memset(bcb, 0, sizeof(struct bulk_cb_wrap));
		bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
		bcb->DataTransferLength = blenByte;
		bcb->Flags  = 0x00;
		bcb->CDB[0] = 0xF0;
		bcb->CDB[1] = 0x04;
		bcb->CDB[5] = (unsigned char)(bn);
		bcb->CDB[4] = (unsigned char)(bn>>8);
		bcb->CDB[3] = (unsigned char)(bn>>16);
		bcb->CDB[2] = (unsigned char)(bn>>24);

		result = ene_send_scsi_cmd(us, FDIR_WRITE, scsi_sglist(srb), 1);
	} else {
		void *buf;
		int offset = 0;
		u16 PhyBlockAddr;
		u8 PageNum;
		u16 len, oldphy, newphy;

		buf = kmalloc(blenByte, GFP_KERNEL);
		if (buf == NULL)
			return USB_STOR_TRANSPORT_ERROR;
		usb_stor_set_xfer_buf(buf, blenByte, srb);

		result = ene_load_bincode(us, MS_RW_PATTERN);
		if (result != USB_STOR_XFER_GOOD) {
			pr_info("Load MS RW pattern Fail !!\n");
			result = USB_STOR_TRANSPORT_ERROR;
			goto exit;
		}

		PhyBlockAddr = (u16)(bn / info->MS_Lib.PagesPerBlock);
		PageNum      = (u8)(bn % info->MS_Lib.PagesPerBlock);

		while (1) {
			if (blen > (info->MS_Lib.PagesPerBlock-PageNum))
				len = info->MS_Lib.PagesPerBlock-PageNum;
			else
				len = blen;

			oldphy = ms_libconv_to_physical(info, PhyBlockAddr); /* need check us <-> info */
			newphy = ms_libsearch_block_from_logical(us, PhyBlockAddr);

			result = ms_read_copyblock(us, oldphy, newphy, PhyBlockAddr, PageNum, buf+offset, len);

			if (result != USB_STOR_XFER_GOOD) {
				pr_info("MS_SCSI_Write --- result = %x\n", result);
				result =  USB_STOR_TRANSPORT_ERROR;
				goto exit;
			}

			info->MS_Lib.Phy2LogMap[oldphy] = MS_LB_NOT_USED_ERASED;
			ms_lib_force_setlogical_pair(us, PhyBlockAddr, newphy);

			blen -= len;
			if (blen <= 0)
				break;
			PhyBlockAddr++;
			PageNum = 0;
			offset += MS_BYTES_PER_PAGE*len;
		}
exit:
		kfree(buf);
	}
	return result;
}

/*
 * ENE MS Card
 */

static int ene_get_card_type(struct us_data *us, u16 index, void *buf)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	int result;

	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength	= 0x01;
	bcb->Flags			= US_BULK_FLAG_IN;
	bcb->CDB[0]			= 0xED;
	bcb->CDB[2]			= (unsigned char)(index>>8);
	bcb->CDB[3]			= (unsigned char)index;

	result = ene_send_scsi_cmd(us, FDIR_READ, buf, 0);
	return result;
}

static int ene_get_card_status(struct us_data *us, u8 *buf)
{
	u16 tmpreg;
	u32 reg4b;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	/*usb_stor_dbg(us, "transport --- ENE_ReadSDReg\n");*/
	reg4b = *(u32 *)&buf[0x18];
	info->SD_READ_BL_LEN = (u8)((reg4b >> 8) & 0x0f);

	tmpreg = (u16) reg4b;
	reg4b = *(u32 *)(&buf[0x14]);
	if (info->SD_Status.HiCapacity && !info->SD_Status.IsMMC)
		info->HC_C_SIZE = (reg4b >> 8) & 0x3fffff;

	info->SD_C_SIZE = ((tmpreg & 0x03) << 10) | (u16)(reg4b >> 22);
	info->SD_C_SIZE_MULT = (u8)(reg4b >> 7)  & 0x07;
	if (info->SD_Status.HiCapacity && info->SD_Status.IsMMC)
		info->HC_C_SIZE = *(u32 *)(&buf[0x100]);

	if (info->SD_READ_BL_LEN > SD_BLOCK_LEN) {
		info->SD_Block_Mult = 1 << (info->SD_READ_BL_LEN-SD_BLOCK_LEN);
		info->SD_READ_BL_LEN = SD_BLOCK_LEN;
	} else {
		info->SD_Block_Mult = 1;
	}

	return USB_STOR_TRANSPORT_GOOD;
}

static int ene_load_bincode(struct us_data *us, unsigned char flag)
{
	int err;
	char *fw_name = NULL;
	unsigned char *buf = NULL;
	const struct firmware *sd_fw = NULL;
	int result = USB_STOR_TRANSPORT_ERROR;
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	if (info->BIN_FLAG == flag)
		return USB_STOR_TRANSPORT_GOOD;

	switch (flag) {
	/* For SD */
	case SD_INIT1_PATTERN:
		usb_stor_dbg(us, "SD_INIT1_PATTERN\n");
		fw_name = SD_INIT1_FIRMWARE;
		break;
	case SD_INIT2_PATTERN:
		usb_stor_dbg(us, "SD_INIT2_PATTERN\n");
		fw_name = SD_INIT2_FIRMWARE;
		break;
	case SD_RW_PATTERN:
		usb_stor_dbg(us, "SD_RW_PATTERN\n");
		fw_name = SD_RW_FIRMWARE;
		break;
	/* For MS */
	case MS_INIT_PATTERN:
		usb_stor_dbg(us, "MS_INIT_PATTERN\n");
		fw_name = MS_INIT_FIRMWARE;
		break;
	case MSP_RW_PATTERN:
		usb_stor_dbg(us, "MSP_RW_PATTERN\n");
		fw_name = MSP_RW_FIRMWARE;
		break;
	case MS_RW_PATTERN:
		usb_stor_dbg(us, "MS_RW_PATTERN\n");
		fw_name = MS_RW_FIRMWARE;
		break;
	default:
		usb_stor_dbg(us, "----------- Unknown PATTERN ----------\n");
		goto nofw;
	}

	err = request_firmware(&sd_fw, fw_name, &us->pusb_dev->dev);
	if (err) {
		usb_stor_dbg(us, "load firmware %s failed\n", fw_name);
		goto nofw;
	}
	buf = kmalloc(sd_fw->size, GFP_KERNEL);
	if (buf == NULL)
		goto nofw;

	memcpy(buf, sd_fw->data, sd_fw->size);
	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = sd_fw->size;
	bcb->Flags = 0x00;
	bcb->CDB[0] = 0xEF;

	result = ene_send_scsi_cmd(us, FDIR_WRITE, buf, 0);
	info->BIN_FLAG = flag;
	kfree(buf);

nofw:
	release_firmware(sd_fw);
	return result;
}

static int ms_card_init(struct us_data *us)
{
	u32 result;
	u16 TmpBlock;
	unsigned char *PageBuffer0 = NULL, *PageBuffer1 = NULL;
	struct ms_lib_type_extdat extdat;
	u16 btBlk1st, btBlk2nd;
	u32 btBlk1stErred;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	printk(KERN_INFO "MS_CardInit start\n");

	ms_lib_free_allocatedarea(us); /* Clean buffer and set struct us_data flag to 0 */

	/* get two PageBuffer */
	PageBuffer0 = kmalloc(MS_BYTES_PER_PAGE, GFP_KERNEL);
	PageBuffer1 = kmalloc(MS_BYTES_PER_PAGE, GFP_KERNEL);
	if ((PageBuffer0 == NULL) || (PageBuffer1 == NULL)) {
		result = MS_NO_MEMORY_ERROR;
		goto exit;
	}

	btBlk1st = btBlk2nd = MS_LB_NOT_USED;
	btBlk1stErred = 0;

	for (TmpBlock = 0; TmpBlock < MS_MAX_INITIAL_ERROR_BLOCKS+2; TmpBlock++) {

		switch (ms_read_readpage(us, TmpBlock, 0, (u32 *)PageBuffer0, &extdat)) {
		case MS_STATUS_SUCCESS:
			break;
		case MS_STATUS_INT_ERROR:
			break;
		case MS_STATUS_ERROR:
		default:
			continue;
		}

		if ((extdat.ovrflg & MS_REG_OVR_BKST) == MS_REG_OVR_BKST_NG)
			continue;

		if (((extdat.mngflg & MS_REG_MNG_SYSFLG) == MS_REG_MNG_SYSFLG_USER) ||
			(be16_to_cpu(((struct ms_bootblock_page0 *)PageBuffer0)->header.wBlockID) != MS_BOOT_BLOCK_ID) ||
			(be16_to_cpu(((struct ms_bootblock_page0 *)PageBuffer0)->header.wFormatVersion) != MS_BOOT_BLOCK_FORMAT_VERSION) ||
			(((struct ms_bootblock_page0 *)PageBuffer0)->header.bNumberOfDataEntry != MS_BOOT_BLOCK_DATA_ENTRIES))
				continue;

		if (btBlk1st != MS_LB_NOT_USED) {
			btBlk2nd = TmpBlock;
			break;
		}

		btBlk1st = TmpBlock;
		memcpy(PageBuffer1, PageBuffer0, MS_BYTES_PER_PAGE);
		if (extdat.status1 & (MS_REG_ST1_DTER | MS_REG_ST1_EXER | MS_REG_ST1_FGER))
			btBlk1stErred = 1;
	}

	if (btBlk1st == MS_LB_NOT_USED) {
		result = MS_STATUS_ERROR;
		goto exit;
	}

	/* write protect */
	if ((extdat.status0 & MS_REG_ST0_WP) == MS_REG_ST0_WP_ON)
		ms_lib_ctrl_set(info, MS_LIB_CTRL_WRPROTECT);

	result = MS_STATUS_ERROR;
	/* 1st Boot Block */
	if (btBlk1stErred == 0)
		result = ms_lib_process_bootblock(us, btBlk1st, PageBuffer1);
		/* 1st */
	/* 2nd Boot Block */
	if (result && (btBlk2nd != MS_LB_NOT_USED))
		result = ms_lib_process_bootblock(us, btBlk2nd, PageBuffer0);

	if (result) {
		result = MS_STATUS_ERROR;
		goto exit;
	}

	for (TmpBlock = 0; TmpBlock < btBlk1st; TmpBlock++)
		info->MS_Lib.Phy2LogMap[TmpBlock] = MS_LB_INITIAL_ERROR;

	info->MS_Lib.Phy2LogMap[btBlk1st] = MS_LB_BOOT_BLOCK;

	if (btBlk2nd != MS_LB_NOT_USED) {
		for (TmpBlock = btBlk1st + 1; TmpBlock < btBlk2nd; TmpBlock++)
			info->MS_Lib.Phy2LogMap[TmpBlock] = MS_LB_INITIAL_ERROR;

		info->MS_Lib.Phy2LogMap[btBlk2nd] = MS_LB_BOOT_BLOCK;
	}

	result = ms_lib_scan_logicalblocknumber(us, btBlk1st);
	if (result)
		goto exit;

	for (TmpBlock = MS_PHYSICAL_BLOCKS_PER_SEGMENT;
		TmpBlock < info->MS_Lib.NumberOfPhyBlock;
		TmpBlock += MS_PHYSICAL_BLOCKS_PER_SEGMENT) {
		if (ms_count_freeblock(us, TmpBlock) == 0) {
			ms_lib_ctrl_set(info, MS_LIB_CTRL_WRPROTECT);
			break;
		}
	}

	/* write */
	if (ms_lib_alloc_writebuf(us)) {
		result = MS_NO_MEMORY_ERROR;
		goto exit;
	}

	result = MS_STATUS_SUCCESS;

exit:
	kfree(PageBuffer1);
	kfree(PageBuffer0);

	printk(KERN_INFO "MS_CardInit end\n");
	return result;
}

static int ene_ms_init(struct us_data *us)
{
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	int result;
	u8 buf[0x200];
	u16 MSP_BlockSize, MSP_UserAreaBlocks;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	printk(KERN_INFO "transport --- ENE_MSInit\n");

	/* the same part to test ENE */

	result = ene_load_bincode(us, MS_INIT_PATTERN);
	if (result != USB_STOR_XFER_GOOD) {
		printk(KERN_ERR "Load MS Init Code Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = 0x200;
	bcb->Flags      = US_BULK_FLAG_IN;
	bcb->CDB[0]     = 0xF1;
	bcb->CDB[1]     = 0x01;

	result = ene_send_scsi_cmd(us, FDIR_READ, &buf, 0);
	if (result != USB_STOR_XFER_GOOD) {
		printk(KERN_ERR "Execution MS Init Code Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}
	/* the same part to test ENE */
	info->MS_Status = *(struct MS_STATUS *)&buf[0];

	if (info->MS_Status.Insert && info->MS_Status.Ready) {
		printk(KERN_INFO "Insert     = %x\n", info->MS_Status.Insert);
		printk(KERN_INFO "Ready      = %x\n", info->MS_Status.Ready);
		printk(KERN_INFO "IsMSPro    = %x\n", info->MS_Status.IsMSPro);
		printk(KERN_INFO "IsMSPHG    = %x\n", info->MS_Status.IsMSPHG);
		printk(KERN_INFO "WtP= %x\n", info->MS_Status.WtP);
		if (info->MS_Status.IsMSPro) {
			MSP_BlockSize      = (buf[6] << 8) | buf[7];
			MSP_UserAreaBlocks = (buf[10] << 8) | buf[11];
			info->MSP_TotalBlock = MSP_BlockSize * MSP_UserAreaBlocks;
		} else {
			ms_card_init(us); /* Card is MS (to ms.c)*/
		}
		usb_stor_dbg(us, "MS Init Code OK !!\n");
	} else {
		usb_stor_dbg(us, "MS Card Not Ready --- %x\n", buf[0]);
		return USB_STOR_TRANSPORT_ERROR;
	}

	return USB_STOR_TRANSPORT_GOOD;
}

static int ene_sd_init(struct us_data *us)
{
	int result;
	u8  buf[0x200];
	struct bulk_cb_wrap *bcb = (struct bulk_cb_wrap *) us->iobuf;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *) us->extra;

	usb_stor_dbg(us, "transport --- ENE_SDInit\n");
	/* SD Init Part-1 */
	result = ene_load_bincode(us, SD_INIT1_PATTERN);
	if (result != USB_STOR_XFER_GOOD) {
		usb_stor_dbg(us, "Load SD Init Code Part-1 Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->Flags = US_BULK_FLAG_IN;
	bcb->CDB[0] = 0xF2;

	result = ene_send_scsi_cmd(us, FDIR_READ, NULL, 0);
	if (result != USB_STOR_XFER_GOOD) {
		usb_stor_dbg(us, "Execution SD Init Code Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	/* SD Init Part-2 */
	result = ene_load_bincode(us, SD_INIT2_PATTERN);
	if (result != USB_STOR_XFER_GOOD) {
		usb_stor_dbg(us, "Load SD Init Code Part-2 Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	memset(bcb, 0, sizeof(struct bulk_cb_wrap));
	bcb->Signature = cpu_to_le32(US_BULK_CB_SIGN);
	bcb->DataTransferLength = 0x200;
	bcb->Flags              = US_BULK_FLAG_IN;
	bcb->CDB[0]             = 0xF1;

	result = ene_send_scsi_cmd(us, FDIR_READ, &buf, 0);
	if (result != USB_STOR_XFER_GOOD) {
		usb_stor_dbg(us, "Execution SD Init Code Fail !!\n");
		return USB_STOR_TRANSPORT_ERROR;
	}

	info->SD_Status =  *(struct SD_STATUS *)&buf[0];
	if (info->SD_Status.Insert && info->SD_Status.Ready) {
		struct SD_STATUS *s = &info->SD_Status;

		ene_get_card_status(us, (unsigned char *)&buf);
		usb_stor_dbg(us, "Insert     = %x\n", s->Insert);
		usb_stor_dbg(us, "Ready      = %x\n", s->Ready);
		usb_stor_dbg(us, "IsMMC      = %x\n", s->IsMMC);
		usb_stor_dbg(us, "HiCapacity = %x\n", s->HiCapacity);
		usb_stor_dbg(us, "HiSpeed    = %x\n", s->HiSpeed);
		usb_stor_dbg(us, "WtP        = %x\n", s->WtP);
	} else {
		usb_stor_dbg(us, "SD Card Not Ready --- %x\n", buf[0]);
		return USB_STOR_TRANSPORT_ERROR;
	}
	return USB_STOR_TRANSPORT_GOOD;
}


static int ene_init(struct us_data *us)
{
	int result;
	u8  misc_reg03 = 0;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *)(us->extra);

	result = ene_get_card_type(us, REG_CARD_STATUS, &misc_reg03);
	if (result != USB_STOR_XFER_GOOD)
		return USB_STOR_TRANSPORT_ERROR;

	if (misc_reg03 & 0x01) {
		if (!info->SD_Status.Ready) {
			result = ene_sd_init(us);
			if (result != USB_STOR_XFER_GOOD)
				return USB_STOR_TRANSPORT_ERROR;
		}
	}
	if (misc_reg03 & 0x02) {
		if (!info->MS_Status.Ready) {
			result = ene_ms_init(us);
			if (result != USB_STOR_XFER_GOOD)
				return USB_STOR_TRANSPORT_ERROR;
		}
	}
	return result;
}

/*----- sd_scsi_irp() ---------*/
static int sd_scsi_irp(struct us_data *us, struct scsi_cmnd *srb)
{
	int    result;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *)us->extra;

	info->SrbStatus = SS_SUCCESS;
	switch (srb->cmnd[0]) {
	case TEST_UNIT_READY:
		result = sd_scsi_test_unit_ready(us, srb);
		break; /* 0x00 */
	case INQUIRY:
		result = sd_scsi_inquiry(us, srb);
		break; /* 0x12 */
	case MODE_SENSE:
		result = sd_scsi_mode_sense(us, srb);
		break; /* 0x1A */
	/*
	case START_STOP:
		result = SD_SCSI_Start_Stop(us, srb);
		break; //0x1B
	*/
	case READ_CAPACITY:
		result = sd_scsi_read_capacity(us, srb);
		break; /* 0x25 */
	case READ_10:
		result = sd_scsi_read(us, srb);
		break; /* 0x28 */
	case WRITE_10:
		result = sd_scsi_write(us, srb);
		break; /* 0x2A */
	default:
		info->SrbStatus = SS_ILLEGAL_REQUEST;
		result = USB_STOR_TRANSPORT_FAILED;
		break;
	}
	return result;
}

/*
 * ms_scsi_irp()
 */
static int ms_scsi_irp(struct us_data *us, struct scsi_cmnd *srb)
{
	int result;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *)us->extra;
	info->SrbStatus = SS_SUCCESS;
	switch (srb->cmnd[0]) {
	case TEST_UNIT_READY:
		result = ms_scsi_test_unit_ready(us, srb);
		break; /* 0x00 */
	case INQUIRY:
		result = ms_scsi_inquiry(us, srb);
		break; /* 0x12 */
	case MODE_SENSE:
		result = ms_scsi_mode_sense(us, srb);
		break; /* 0x1A */
	case READ_CAPACITY:
		result = ms_scsi_read_capacity(us, srb);
		break; /* 0x25 */
	case READ_10:
		result = ms_scsi_read(us, srb);
		break; /* 0x28 */
	case WRITE_10:
		result = ms_scsi_write(us, srb);
		break;  /* 0x2A */
	default:
		info->SrbStatus = SS_ILLEGAL_REQUEST;
		result = USB_STOR_TRANSPORT_FAILED;
		break;
	}
	return result;
}

static int ene_transport(struct scsi_cmnd *srb, struct us_data *us)
{
	int result = 0;
	struct ene_ub6250_info *info = (struct ene_ub6250_info *)(us->extra);

	/*US_DEBUG(usb_stor_show_command(us, srb)); */
	scsi_set_resid(srb, 0);
	if (unlikely(!(info->SD_Status.Ready || info->MS_Status.Ready))) {
		result = ene_init(us);
	} else {
		if (info->SD_Status.Ready)
			result = sd_scsi_irp(us, srb);

		if (info->MS_Status.Ready)
			result = ms_scsi_irp(us, srb);
	}
	return 0;
}


>>>>>>> 191648d... usb: storage: Convert US_DEBUGP to usb_stor_dbg
static int ene_ub6250_probe(struct usb_interface *intf,
			 const struct usb_device_id *id)
{
	int result;
	u8  misc_reg03 = 0;
	struct us_data *us;

	result = usb_stor_probe1(&us, intf, id,
		   (id - ene_ub6250_usb_ids) + ene_ub6250_unusual_dev_list);
	if (result)
		return result;

	/* FIXME: where should the code alloc extra buf ? */
	if (!us->extra) {
		us->extra = kzalloc(sizeof(struct ene_ub6250_info), GFP_KERNEL);
		if (!us->extra)
			return -ENOMEM;
		us->extra_destructor = ene_ub6250_info_destructor;
	}

	us->transport_name = "ene_ub6250";
	us->transport = ene_transport;
	us->max_lun = 0;

	result = usb_stor_probe2(us);
	if (result)
		return result;

	/* probe card type */
	result = ene_get_card_type(us, REG_CARD_STATUS, &misc_reg03);
	if (result != USB_STOR_XFER_GOOD) {
		usb_stor_disconnect(intf);
		return USB_STOR_TRANSPORT_ERROR;
	}

	if (!(misc_reg03 & 0x01)) {
		result = -ENODEV;
		printk(KERN_NOTICE "ums_eneub6250: The driver only supports SD card. "
		       "To use SM/MS card, please build driver/staging/keucr\n");
		usb_stor_disconnect(intf);
	}

	return result;
}


#ifdef CONFIG_PM

static int ene_ub6250_resume(struct usb_interface *iface)
{
	u8 tmp = 0;
	struct us_data *us = usb_get_intfdata(iface);
	struct ene_ub6250_info *info = (struct ene_ub6250_info *)(us->extra);

	mutex_lock(&us->dev_mutex);

	if (us->suspend_resume_hook)
		(us->suspend_resume_hook)(us, US_RESUME);

	mutex_unlock(&us->dev_mutex);

	info->Power_IsResum = true;
	/*info->SD_Status.Ready = 0; */
	info->SD_Status = *(struct SD_STATUS *)&tmp;
	info->MS_Status = *(struct MS_STATUS *)&tmp;
	info->SM_Status = *(struct SM_STATUS *)&tmp;

	return 0;
}

static int ene_ub6250_reset_resume(struct usb_interface *iface)
{
	u8 tmp = 0;
	struct us_data *us = usb_get_intfdata(iface);
	struct ene_ub6250_info *info = (struct ene_ub6250_info *)(us->extra);

	/* Report the reset to the SCSI core */
	usb_stor_reset_resume(iface);

	/* FIXME: Notify the subdrivers that they need to reinitialize
	 * the device */
	info->Power_IsResum = true;
	/*info->SD_Status.Ready = 0; */
	info->SD_Status = *(struct SD_STATUS *)&tmp;
	info->MS_Status = *(struct MS_STATUS *)&tmp;
	info->SM_Status = *(struct SM_STATUS *)&tmp;

	return 0;
}

#else

#define ene_ub6250_resume		NULL
#define ene_ub6250_reset_resume		NULL

#endif

static struct usb_driver ene_ub6250_driver = {
	.name =		"ums_eneub6250",
	.probe =	ene_ub6250_probe,
	.disconnect =	usb_stor_disconnect,
	.suspend =	usb_stor_suspend,
	.resume =	ene_ub6250_resume,
	.reset_resume =	ene_ub6250_reset_resume,
	.pre_reset =	usb_stor_pre_reset,
	.post_reset =	usb_stor_post_reset,
	.id_table =	ene_ub6250_usb_ids,
	.soft_unbind =	1,
};

module_usb_driver(ene_ub6250_driver);
