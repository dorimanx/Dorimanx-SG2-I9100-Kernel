/*
 * vsyscall_emu_64.S: Vsyscall emulation page
 *
 * Copyright (c) 2011 Andy Lutomirski
 *
 * Subject to the GNU General Public License, version 2
 */

#include <linux/linkage.h>
<<<<<<< HEAD

#include <asm/irq_vectors.h>
#include <asm/page_types.h>
#include <asm/unistd_64.h>

__PAGE_ALIGNED_DATA
	.globl __vsyscall_page
	.balign PAGE_SIZE, 0xcc
	.type __vsyscall_page, @object
__vsyscall_page:

	mov $__NR_gettimeofday, %rax
	syscall
	ret

	.balign 1024, 0xcc
	mov $__NR_time, %rax
	syscall
	ret

	.balign 1024, 0xcc
	mov $__NR_getcpu, %rax
	syscall
	ret

	.balign 4096, 0xcc

	.size __vsyscall_page, 4096
=======
#include <asm/irq_vectors.h>

/* The unused parts of the page are filled with 0xcc by the linker script. */

.section .vsyscall_0, "a"
ENTRY(vsyscall_0)
	int $VSYSCALL_EMU_VECTOR
END(vsyscall_0)

.section .vsyscall_1, "a"
ENTRY(vsyscall_1)
	int $VSYSCALL_EMU_VECTOR
END(vsyscall_1)

.section .vsyscall_2, "a"
ENTRY(vsyscall_2)
	int $VSYSCALL_EMU_VECTOR
END(vsyscall_2)
>>>>>>> 5cec93c... x86-64: Emulate legacy vsyscalls
