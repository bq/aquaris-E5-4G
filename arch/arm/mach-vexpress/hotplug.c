/*
 *  linux/arch/arm/mach-vexpress/hotplug.c
 *
 *  Copyright (C) 2010 ARM Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>
#include <linux/completion.h>

#include <mach/motherboard.h>

#include <plat/hotplug.h>

extern volatile int pen_release;

void __ref platform_do_lowpower(unsigned int cpu)
{
	ct_desc->enter_lowpower();

	for (;;) {
		if (ct_desc->do_lowpower() == -ENODEV)
			asm volatile("wfi" : : : "memory");

		if (pen_release == cpu)
			break;

#ifdef DEBUG
		printk("CPU%u: spurious wakeup call\n", cpu);
#endif
	}

	ct_desc->leave_lowpower();
}
