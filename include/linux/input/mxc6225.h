/*
 * Copyright (C) 2011 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
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
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#ifndef __MXC6225_H__
#define __MXC6225_H__

#ifndef DEBUG
#define DEBUG
#endif

#define	MXC622X_ACC_IOCTL_BASE 'B'
/** The following define the IOCTL command values via the ioctl macros */
#define	MXC622X_ACC_IOCTL_SET_DELAY		_IOW(MXC622X_ACC_IOCTL_BASE, 0, int)
#define	MXC622X_ACC_IOCTL_GET_DELAY		_IOR(MXC622X_ACC_IOCTL_BASE, 1, int)
#define	MXC622X_ACC_IOCTL_SET_ENABLE		_IOW(MXC622X_ACC_IOCTL_BASE, 2, int)
#define	MXC622X_ACC_IOCTL_GET_ENABLE		_IOR(MXC622X_ACC_IOCTL_BASE, 3, int)
#define	MXC622X_ACC_IOCTL_GET_COOR_XYZ       _IOW(MXC622X_ACC_IOCTL_BASE, 22, int)
#define	MXC622X_ACC_IOCTL_GET_CHIP_ID        _IOR(MXC622X_ACC_IOCTL_BASE, 255, char[32])

/************************************************/
/* 	Accelerometer defines section	 	*/
/************************************************/
#define MXC622X_ACC_DEV_NAME		"mxc6225"
#define MXC622X_ACC_INPUT_NAME		"accelerometer" 
#define MXC622X_ACC_I2C_ADDR     	0x15
#define MXC622X_ACC_I2C_NAME     	MXC622X_ACC_DEV_NAME

/* MXC622X register address */
#define MXC622X_REG_CTRL		0x04
#define MXC622X_REG_DATA		0x00

/* MXC622X control bit */
#define MXC622X_CTRL_PWRON		0x00	/* power on */
#define MXC622X_CTRL_PWRDN		0x80	/* power donw */

#define MXC622X_ACCEL_MIN_POLL_INTERVAL_MS	1
#define MXC622X_ACCEL_MAX_POLL_INTERVAL_MS	5000

//#if defined(CONFIG_MACH_SP6810A)
#define I2C_BUS_NUM_STATIC_ALLOC
#define I2C_STATIC_BUS_NUM        ( 0)	// Need to be modified according to actual setting
//#endif

#ifdef	__KERNEL__
struct mxc622x_acc_platform_data {
	int poll_interval;
	int min_interval;
	int max_interval;

	int vdd_max_uv;
	int vdd_min_uv;
	int vio_max_uv;
	int vio_min_uv;
};
#endif	/* __KERNEL__ */

#endif  /* __MXC6225_H__ */
