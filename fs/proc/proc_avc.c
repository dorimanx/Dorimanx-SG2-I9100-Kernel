/*
 *  linux/fs/proc/proc_avc.c
 *
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/syslog.h>
#include <linux/bootmem.h>
#include <linux/export.h>

#include <asm/uaccess.h>
#include <asm/io.h>

#ifdef CONFIG_SEC_DEBUG
#include <mach/sec_debug.h>
#endif

#define LOG_MAGIC 0x4d474f4c	/* "LOGM" */

static unsigned *sec_avc_log_ptr;
static char *sec_avc_log_buf;
static unsigned sec_avc_log_size;

int __init sec_avc_log_init(void)
{
	unsigned size = SZ_256K;
	unsigned *sec_avc_log_mag;

#ifdef CONFIG_SEC_DEBUG
	if (kernel_sec_get_debug_level() == KERNEL_SEC_DEBUG_LEVEL_LOW) {
		pr_info("AVC_MSG:debug level is low.\n");
		return 1;
	}
#endif

	sec_avc_log_size = size + 8;
	sec_avc_log_mag = alloc_bootmem(sec_avc_log_size);
	pr_info("allocating %u bytes at %p (%lx physical) for avc log\n",
		sec_avc_log_size, sec_avc_log_mag, __pa(sec_avc_log_buf));

	sec_avc_log_ptr = sec_avc_log_mag + 4;
	sec_avc_log_buf = (char *)(sec_avc_log_mag + 8);
	sec_avc_log_size = size;

	if (*sec_avc_log_mag != LOG_MAGIC) {
		pr_info("%s: no old log found\n", __func__);
		*sec_avc_log_ptr = 0;
		*sec_avc_log_mag = LOG_MAGIC;
	}

	return 1;
}

#define BUF_SIZE 256
void sec_avc_log(char *fmt, ...)
{
	va_list args;
	char buf[BUF_SIZE];
	int len = 0;
	unsigned int idx;
	unsigned int size;

	/* In case of sec_avc_log_setup is failed */
	if (!sec_avc_log_size)
		return;

	va_start(args, fmt);
	vsnprintf(buf, sizeof(buf), fmt, args);
	va_end(args);

	idx = *sec_avc_log_ptr;
	size = strlen(buf);

	if (idx + size > sec_avc_log_size - 1) {
		len = scnprintf(&sec_avc_log_buf[0],
				size + 1, "%s", buf);
		*sec_avc_log_ptr = len;
	} else {
		len = scnprintf(&sec_avc_log_buf[idx], size + 1, "%s", buf);
		*sec_avc_log_ptr += len;
	}
}

static ssize_t sec_avc_log_write(struct file *file,
					     const char __user *buf,
					     size_t count, loff_t *ppos)
{
	char *page = NULL;
	ssize_t ret;
	int new_value;

	if (!sec_avc_log_buf)
		return 0;

	ret = -ENOMEM;
	if (count >= PAGE_SIZE)
		return ret;

	ret = -ENOMEM;
	page = (char *)get_zeroed_page(GFP_KERNEL);
	if (!page)
		return ret;;

	ret = -EFAULT;
	if (copy_from_user(page, buf, count))
		goto out;

	ret = -EINVAL;
	if (sscanf(page, "%u", &new_value) != 1) {
		pr_info("%s\n", page);
		/* print avc_log to sec_avc_log_buf */
		sec_avc_log("%s", page);
	} 
	ret = count;
out:
	free_page((unsigned long)page);
	return ret;
}
static ssize_t sec_avc_log_read(struct file *file, char __user *buf,
				size_t len, loff_t *offset)
{
	loff_t pos = *offset;
	ssize_t count;

	if (pos >= (*sec_avc_log_ptr & (sec_avc_log_size - 1)))
		return 0;

	count = min(len,
		(size_t)((*sec_avc_log_ptr & (sec_avc_log_size - 1)) - pos));
	if (copy_to_user(buf, sec_avc_log_buf + pos, count))
		return -EFAULT;

	*offset += count;
	return count;
}

static const struct file_operations avc_msg_file_ops = {
	.owner = THIS_MODULE,
	.read = sec_avc_log_read,
	.write = sec_avc_log_write,
	.llseek = generic_file_llseek,
};

static int __init sec_avc_log_late_init(void)
{
	struct proc_dir_entry *entry;

	if (sec_avc_log_buf == NULL) {
		pr_err("%s: sec_avc_log_buf not initialized.\n", __func__);
		return 0;
	}

	entry = create_proc_entry("avc_msg", S_IFREG | S_IRUGO, NULL);
	if (!entry) {
		pr_err("%s: failed to create proc entry\n", __func__);
		return 0;
	}

	entry->proc_fops = &avc_msg_file_ops;
	entry->size = sec_avc_log_size;
	return 0;
}

late_initcall(sec_avc_log_late_init);
