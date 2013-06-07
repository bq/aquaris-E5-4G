/*
 * Copyright (C) 2012 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef __ASM_SIGINFO_H
#define __ASM_SIGINFO_H

#ifdef __LP64__
#define __ARCH_SI_PREAMBLE_SIZE	(4 * sizeof(int))
#else /* ILP32 */
typedef long long __kernel_si_clock_t __attribute__((aligned(4)));
#define __ARCH_SI_CLOCK_T	__kernel_si_clock_t
#define __ARCH_SI_ATTRIBUTES	__attribute__((aligned(8)))
#endif

#include <asm-generic/siginfo.h>

#endif
