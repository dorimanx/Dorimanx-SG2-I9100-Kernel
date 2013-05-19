/*
 * Author: Paul Reioux aka Faux123 <reioux@gmail.com>
 *
 * krait-defines
 * Copyright 2013 Paul Reioux
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
#ifdef CONFIG_CPU_OVERCLOCK
#define OVERCLOCK_EXTRA_FREQS	7
#else
#define OVERCLOCK_EXTRA_FREQS	0
#endif

#ifdef CONFIG_LOW_CPUCLOCKS
#define FREQ_TABLE_SIZE		(39 + OVERCLOCK_EXTRA_FREQS)
#else
#define FREQ_TABLE_SIZE		(35 + OVERCLOCK_EXTRA_FREQS)
#endif
