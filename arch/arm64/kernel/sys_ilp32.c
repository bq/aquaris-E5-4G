/*
 * AArch64 ILP32 specific system calls implementation
 *
 * Copyright (C) 2013 ARM Ltd.
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

#include <linux/compiler.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/syscalls.h>
#include <linux/compat.h>

asmlinkage long compat_sys_rt_sigreturn_wrapper(void);

/*
 * ILP32 system calls routed via the compat layer.
 */
#define sys_ioctl		compat_sys_ioctl
#define sys_readv		compat_sys_readv
#define sys_writev		compat_sys_writev
#define sys_preadv		compat_sys_preadv64
#define sys_pwritev		compat_sys_pwritev64
#define sys_vmsplice		compat_sys_vmsplice
#define sys_waitid		compat_sys_waitid
#define sys_set_robust_list	compat_sys_set_robust_list
#define sys_get_robust_list	compat_sys_get_robust_list
#define sys_kexec_load		compat_sys_kexec_load
#define sys_timer_create	compat_sys_timer_create
#define sys_ptrace		compat_sys_ptrace
#define sys_sigaltstack		compat_sys_sigaltstack
#define sys_rt_sigaction	compat_sys_rt_sigaction
#define sys_rt_sigpending	compat_sys_rt_sigpending
#define sys_rt_sigtimedwait	compat_sys_rt_sigtimedwait
#define sys_rt_sigqueueinfo	compat_sys_rt_sigqueueinfo
#define sys_rt_sigreturn	compat_sys_rt_sigreturn_wrapper
#define sys_mq_notify		compat_sys_mq_notify
#define sys_recvfrom		compat_sys_recvfrom
#define sys_setsockopt		compat_sys_setsockopt
#define sys_getsockopt		compat_sys_getsockopt
#define sys_sendmsg		compat_sys_sendmsg
#define sys_recvmsg		compat_sys_recvmsg
#define sys_execve		compat_sys_execve
#define sys_move_pages		compat_sys_move_pages
#define sys_rt_tgsigqueueinfo	compat_sys_rt_tgsigqueueinfo
#define sys_recvmmsg		compat_sys_recvmmsg
#define sys_sendmmsg		compat_sys_sendmmsg
#define sys_process_vm_readv	compat_sys_process_vm_readv
#define sys_process_vm_writev	compat_sys_process_vm_writev

#include <asm/syscalls.h>

#undef __SYSCALL
#define __SYSCALL(nr, sym)	[nr] = sym,

/*
 * The sys_call_ilp32_table array must be 4K aligned to be accessible from
 * kernel/entry.S.
 */
void *sys_call_ilp32_table[__NR_syscalls] __aligned(4096) = {
	[0 ... __NR_syscalls - 1] = sys_ni_syscall,
#include <asm/unistd.h>
};
