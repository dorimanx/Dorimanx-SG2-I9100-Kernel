/*
 * Wrapper functions for accessing the file_struct fd array.
 */

#ifndef __LINUX_FILE_H
#define __LINUX_FILE_H

#include <linux/compiler.h>
#include <linux/types.h>
#include <linux/posix_types.h>

struct file;

extern void fput(struct file *);
extern void drop_file_write_access(struct file *file);

struct file_operations;
struct vfsmount;
struct dentry;
struct path;
extern struct file *alloc_file(struct path *, fmode_t mode,
	const struct file_operations *fop);

static inline void fput_light(struct file *file, int fput_needed)
{
	if (fput_needed)
		fput(file);
}

struct fd {
	struct file *file;
	int need_put;
};

static inline void fdput(struct fd fd)
{
	if (fd.need_put)
		fput(fd.file);
}

extern struct file *fget(unsigned int fd);
extern struct file *fget_light(unsigned int fd, int *fput_needed);

static inline struct fd fdget(unsigned int fd)
{
	int b;
	struct file *f = fget_light(fd, &b);
	return (struct fd){f,b};
}

extern struct file *fget_raw(unsigned int fd);
extern struct file *fget_raw_light(unsigned int fd, int *fput_needed);

static inline struct fd fdget_raw(unsigned int fd)
{
	int b;
	struct file *f = fget_raw_light(fd, &b);
	return (struct fd){f,b};
}

extern int f_dupfd(unsigned int from, struct file *file, unsigned flags);
extern int replace_fd(unsigned fd, struct file *file, unsigned flags);
extern void set_close_on_exec(unsigned int fd, int flag);
extern void put_filp(struct file *);
extern int alloc_fd(unsigned start, unsigned flags);
extern int get_unused_fd(void);
#define get_unused_fd_flags(flags) alloc_fd(0, (flags))
extern void put_unused_fd(unsigned int fd);

extern void fd_install(unsigned int fd, struct file *file);

#endif /* __LINUX_FILE_H */
