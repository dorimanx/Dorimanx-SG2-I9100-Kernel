/*
   EDS loop encryption enabling module

   Copyright (C)  2012 Oleg <oleg@sovworks.com>

   This module is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This module is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this module; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/init.h>
#include <linux/module.h>
//#include <linux/blkdev.h>
#include <linux/loop.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/uaccess.h>
#include <linux/jiffies.h>
#include <linux/proc_fs.h>

#include "xts.h"
#include "eds_km.h"

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("EDS kernel module");
MODULE_AUTHOR("Oleg <oleg@sovworks.com>");

//#define EDSDEBUG

#ifdef EDSDEBUG
#define DEBUGLOG(...) printk(KERN_ALERT __VA_ARGS__);
#else
#define DEBUGLOG(...)
#endif


struct dev_crypto_context
{
	struct xts_encrypt_ctx encrypt_ctx;
	struct xts_decrypt_ctx decrypt_ctx;
	unsigned long last_access_ticks;
};

static struct proc_dir_entry *proc_dir = NULL;

static unsigned long get_current_ticks(void)
{
	return jiffies*1000/HZ;
}

static int proc_entry_read( char *buf, char **start, off_t offset, int count, int *eof, void *data)
{
	struct loop_device *lo = (struct loop_device *)data;
	struct dev_crypto_context *ctx = (struct dev_crypto_context *)lo->key_data;
    int len = 0;

    len += sprintf(buf + len,"Backing file: %s\n",lo->lo_file_name);
    len += sprintf( buf + len, "Last access ticks: %ld\n",ctx->last_access_ticks);

    /* If our data length is smaller than what
       the application requested, mark End-Of-File */
    if( len <= count + offset )
        *eof = 1;    /* Initialize the start pointer */
    *start = buf + offset;

    /* Reduce the offset from the total length */
    len -= offset;

    /* Normalize the len variable */
    if( len > count )
        len = count;
    if( len < 0 )
        len = 0;

    return len;
}

static int init_loop_proc_entry(struct loop_device *lo)
{
	struct proc_dir_entry *proc_file = NULL;
	char buf[10];

	 /* Create a proc file */
	snprintf(buf,sizeof(buf),"loop%d",lo->lo_number);


	/* create jiffies using convenience function */
	proc_file = create_proc_read_entry(buf,
	0444, proc_dir,
	proc_entry_read,
	lo);

	if(proc_file == NULL)
		return -ENOMEM;

	return 0;
}

static int init_dev_crypto_context(struct loop_device *lo, const struct loop_info64 *info)
{
	struct dev_crypto_context *ctx = NULL;

	if (info->lo_offset % EDS_SECTOR_SIZE)
		goto out;

	ctx = kzalloc(sizeof (*ctx), GFP_KERNEL);
	if (!ctx)
		goto out;
	ctx->last_access_ticks=get_current_ticks();
	lo->key_data = ctx;
	if(init_loop_proc_entry(lo))
		goto out;

	return 0;
out:
	lo->key_data = NULL;
	if(ctx)
		kfree(ctx);
	return -EINVAL;
}

static int init_keys(struct dev_crypto_context *ctx, unsigned char *key_data, int key_size)
{
	DEBUGLOG("eds_km decrypt key byte before = %u\n",ctx->decrypt_ctx.dec_ctx[0].ks[1])

	if (xts_encrypt_key(key_data, key_size, &ctx->encrypt_ctx))
		return -EINVAL;
	
	if (xts_decrypt_key(key_data, key_size, &ctx->decrypt_ctx))
		return -EINVAL;
	
	DEBUGLOG("eds_km decrypt key byte after = %u\n",ctx->decrypt_ctx.dec_ctx[0].ks[1])

	return 0;
}

static int crypto_transfer(struct loop_device *lo, int cmd,
	struct page *raw_page, unsigned raw_off,
	struct page *loop_page, unsigned loop_off,
	int size, sector_t IV)
{
	unsigned char *buf;
	struct dev_crypto_context *ctx = lo->key_data;	
	char *raw_buf,*loop_buf;
	int res = 0;
	if (!ctx)
	{
		DEBUGLOG("eds_km crypto_transfer keys not set\n");
		return -EINVAL;
	}
	
	DEBUGLOG("eds_km crypto_transfer size=%d raw_off=%d loop_off=%d\n",size,raw_off,loop_off)

	buf = kzalloc(EDS_SECTOR_SIZE,GFP_KERNEL);
	raw_buf = kmap(raw_page) + raw_off;
	loop_buf = kmap(loop_page) + loop_off;

	//raw_buf = (unsigned char *)kmap_atomic(raw_page, KM_USER0) + raw_off;
	//loop_buf = (unsigned char *)kmap_atomic(loop_page, KM_USER1) + loop_off;

	if (cmd == READ) 
	{
		while (size > 0)
		{
			const int sz = min(size, EDS_SECTOR_SIZE);
			u32 iv = cpu_to_le32(IV & 0xffffffff);
			memcpy(buf, raw_buf, sz);
			DEBUGLOG("crypto_transfer decrypting size=%d IV=%u iv=%d raw=%p loop=%p\n",sz,IV,iv,raw_buf,loop_buf)
			DEBUGLOG("crypto_transfer decrypting buf[0]=%d before\n",buf[0])
			res = xts_decrypt_sector(buf, iv, EDS_SECTOR_SIZE, &(ctx->decrypt_ctx));
			if (res)
			{
				DEBUGLOG("eds_km crypto_transfer xts_decrypt_sector error=%d\n",res)
				res = -EINVAL;
				goto out;
			}
			memcpy(loop_buf, buf, sz);
			DEBUGLOG("crypto_transfer decrypting loop_buf[0]=%d IV=%u after\n",(int)loop_buf[0],IV)
			IV++;
			DEBUGLOG("crypto_transfer IV incremented = %u\n",IV)
			size -= sz;
			raw_buf += sz;
			loop_buf += sz;
		}

	} 
	else 
	{
		while (size > 0)
		{
			const int sz = min(size, EDS_SECTOR_SIZE);
			u32 iv = cpu_to_le32(IV & 0xffffffff);
			memcpy(buf, loop_buf, sz);
			DEBUGLOG("crypto_transfer encrypting size=%d IV=%u iv=%d raw=%p loop=%p\n",sz,IV,iv,raw_buf,loop_buf)
			DEBUGLOG("crypto_transfer encrypting buf[0]=%d before\n",buf[0])
			res = xts_encrypt_sector(buf, iv, EDS_SECTOR_SIZE, &(ctx->encrypt_ctx));
			if (res)
			{
				DEBUGLOG("eds_km crypto_transfer xts_encrypt_sector error=%d\n",res)
				res = -EINVAL;
				goto out;
			}
			memcpy(raw_buf, buf, sz);
			DEBUGLOG("crypto_transfer encrypting raw_buf[0]=%d IV=%u after\n",(int)raw_buf[0],IV)
			IV++;
			DEBUGLOG("crypto_transfer IV incremented = %u\n",IV)
			size -= sz;
			loop_buf += sz;
			raw_buf += sz;
		}
	}
	ctx->last_access_ticks = get_current_ticks();
	
out:
	kfree(buf);
	kunmap(loop_page);
	kunmap(raw_page);
	//kunmap_atomic(loop_buf, KM_USER1);
	//kunmap_atomic(raw_buf, KM_USER0);
	return res;	
}

static int ioctl_set_key(struct dev_crypto_context *ctx,unsigned long arg)
{
	struct eds_key_info key_info;
	unsigned char key_data[EDS_KEY_SIZE];
	int size = EDS_KEY_SIZE;

	if(copy_from_user(&key_info,(const void *)arg,sizeof(key_info)))
	{
		DEBUGLOG("EDS_SET_KEY failed getting key_info\n")
		return -EFAULT;
	}

	DEBUGLOG("EDS_SET_KEY ioctl key size = %d\n",key_info.key_size);

	if(key_info.key_size<size)
		size = key_info.key_size;
	memset(key_data,0,EDS_KEY_SIZE);
	if(copy_from_user(key_data,key_info.key,size))
	{
		DEBUGLOG("EDS_SET_KEY failed getting key\n")
		return -EFAULT;
	}
	if(init_keys(ctx,key_data,size))
	{
		DEBUGLOG("EDS_SET_KEY key init failed\n")
		return -EFAULT;
	}
	return 0;
}


static int eds_ioctl(struct loop_device *lo, int cmd, unsigned long arg)
{	
	struct dev_crypto_context *ctx = lo->key_data;
	

	if(ctx == NULL)
	{
		DEBUGLOG("eds_ioctl: lo->key_data is NULL\n")
		return -EINVAL;
	}
	
	DEBUGLOG("in eds_ioctl\n")
		
	if (_IOC_TYPE(cmd) != EDS_CTL_CODE)
	{
		DEBUGLOG("eds_ioctl: wrong magic code\n")
		return -ENOTTY;
	}
	
	if (_IOC_DIR(cmd) & _IOC_WRITE)
	{
		if (_IOC_NR(cmd) == EDS_CMD_SET_KEY)
		{
			DEBUGLOG("EDS_SET_KEY ioctl\n")
			return ioctl_set_key(ctx,arg);
		}
		else
		{
			DEBUGLOG("eds_ioctl: wrong command code\n")
			return -ENOTTY;
		}
	}
	else
	{
		DEBUGLOG("eds_ioctl: wrong command code\n")
		return -ENOTTY;
	}
	return 0;
}

static int free_dev_crypto_context(struct loop_device *lo)
{
	struct dev_crypto_context *ctx = lo->key_data;
	char buf[10];

	snprintf(buf,sizeof(buf),"loop%d",lo->lo_number);
	remove_proc_entry(buf, proc_dir);
	if (ctx != NULL)
	{
		memset(ctx,0,sizeof(*ctx));
		kfree(ctx);
		lo->key_data = NULL;
		return 0;
	}
	printk(KERN_ERR "free_dev_crypto_context(): tfm == NULL?\n");
	return -EINVAL;
}

static struct loop_func_table eds_km_funcs = {
	.number = LO_CRYPT_EDS,
	.init = init_dev_crypto_context,
	.ioctl = eds_ioctl,
	.transfer = crypto_transfer,
	.release = free_dev_crypto_context,
	.owner = THIS_MODULE
};



static int __init
init_eds_km(void)
{
	int rc;
	
	DEBUGLOG("Initializing eds kernel module.\n")
	proc_dir = proc_mkdir(EDS_PROC_ENTRY,NULL);
	//proc_dir = create_proc_entry(EDS_PROC_ENTRY,0555, NULL);
	proc_dir->mode |=  (S_IRUSR | S_IRGRP | S_IROTH | S_IXUSR | S_IXGRP | S_IXOTH);
	if(proc_dir == NULL)
	{
		printk(KERN_ERR "eds: failed registering proc directory\n");
		return -ENOMEM;
	}
	rc = loop_register_transfer(&eds_km_funcs);
	if (rc)
		printk(KERN_ERR "eds: loop_register_transfer failed\n");


	return rc;
}

static void __exit
cleanup_eds_km(void)
{
	DEBUGLOG("Cleaning eds kernel module.\n")
	if(proc_dir!=NULL)
	{
		remove_proc_entry(EDS_PROC_ENTRY, NULL);
		proc_dir=NULL;
	}

	if (loop_unregister_transfer(LO_CRYPT_EDS))
		printk(KERN_ERR
		"eds: loop_unregister_transfer failed\n");
}

module_init(init_eds_km);
module_exit(cleanup_eds_km);
