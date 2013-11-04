Use ADB (Android Debug Bridge) to do register access
- Power on device.
- Connect device to host via USB.
- Open command prompt on host and go to directory where adb and synaptics_reg_access
  reside.
- Run "adb devices" to ensure connection with device.
- Run "adb root" to have root privileges.
- Run "adb push synaptics_reg_access /data" to copy synaptics_reg_access to /data directory on
  device.
- Run "adb shell chmod 777 /data/synaptics_reg_access" to make synaptics_reg_access executable.
- Follow instructions below to run synaptics_reg_access.

Parameters
[-a {address in hex}] - Start address (16-bit) for reading/writing
[-l {length to read}] - Length in bytes to read from start address
[-d {data to write}] - Data (MSB = first byte to write) to write to start address
[-r] - Read from start address for number of bytes set with -l parameter
[-w] - Write data set with -d parameter to start address

Usage examples
- Read five bytes of data starting from address 0x048a
   adb shell /data/synaptics_reg_access -a 0x048a -l 5 -r
- Write 0x11 0x22 0x33 to address 0x048a starting with 0x11
   adb shell /data/synaptics_reg_access -a 0x048a -d 0x112233 -w

/* ----------------------------------------------------------------------------
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
