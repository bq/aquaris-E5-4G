/*
 * synaptics_reg_access.c
 *
 * Copyright (C) 2012 Synaptics Incorporated
 *
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * Description: command line register access implimentation using command
 * line args. This file should not be OS dependant and should build and
 * run under any Linux based OS that utilizes the Synaptice rmi dev
 * built into the kernel
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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>

#define VERSION "1.4"

#define INPUT_PATH "/sys/class/input/input"
#define NUMBER_OF_INPUTS_TO_SCAN 20

#define OPEN_FILENAME "rmidev/open"
#define DATA_FILENAME "rmidev/data"
#define RELEASE_FILENAME "rmidev/release"
#define DETECT_FILENAME "buildid"

#define MAX_BUFFER_LEN 256
#define MAX_STRING_LEN 256

char mySensor[MAX_STRING_LEN];
char input_detect[MAX_STRING_LEN];

char rmidev_open[MAX_STRING_LEN];
char rmidev_data[MAX_STRING_LEN];
char rmidev_release[MAX_STRING_LEN];

unsigned char *r_data;

unsigned char read_write = 0; /* 0 = read, 1 = write */
unsigned short address = 0;
unsigned int length = 1;

static void usage(char *name)
{
	printf("Version %s\n", VERSION);
	printf("Usage: %s [-a {address in hex}] [-l {length to read}] [-d {data to write}] [-r] [-w]\n", name);

	return;
}
static void error_exit(error_code)
{
	if (r_data)
		free(r_data);

	exit(error_code);

	return;
}
/*
static void writevaluetofd(int fd, unsigned int value)
{
	int numBytesWritten = 0;
	char buf[MAX_BUFFER_LEN];

	snprintf(buf, MAX_BUFFER_LEN, "%u\n", value);

	lseek(fd, 0, SEEK_SET);
	numBytesWritten = write(fd, buf, strlen(buf));
	if (numBytesWritten != strlen(buf)) {
		printf("error: failed to write all bytes to file\n");
		close(fd);
		error_exit(EIO);
	}

	return;
}
*/
static int CheckOneFile(char* filename)
{
	int retval;
	struct stat st;

	retval = stat(filename, &st);

	if (retval)
		printf("error: %s does not appear to exist\n", filename);

	return retval;
}

static int CheckFiles(void)
{
	int retval;

	retval = CheckOneFile(rmidev_open);
	if (retval)
		return retval;

	retval = CheckOneFile(rmidev_data);
	if (retval)
		return retval;

	retval = CheckOneFile(rmidev_release);
	if (retval)
		return retval;

	return 0;
}

int main(int argc, char* argv[])
{
	int retval;
	int this_arg = 1;
	int found = 0;
	int fd;
	unsigned long temp;
	unsigned char bytes_to_write;
	unsigned char index = 0;
	unsigned char w_data[MAX_BUFFER_LEN] = {0};
	char data_input[MAX_STRING_LEN] = {0};
	char next_value[3] = {0};
	char *strptr;
	struct stat st;

	if (argc == 1) {
		usage(argv[0]);
		error_exit(EINVAL);
	}

	for (temp = 0; temp < NUMBER_OF_INPUTS_TO_SCAN; temp++) {
		memset(input_detect, 0x00, MAX_STRING_LEN);
		snprintf(input_detect, MAX_STRING_LEN, "%s%d/%s", INPUT_PATH,
				(unsigned int)temp, DETECT_FILENAME);
		retval = stat(input_detect, &st);
		if (retval == 0) {
			snprintf(mySensor, MAX_STRING_LEN, "%s%d", INPUT_PATH,
					(unsigned int)temp);
			found = 1;
			break;
		}
	}

	if (!found) {
		printf("error: input driver not found\n");
		error_exit(ENODEV);
	}

	while (this_arg < argc) {
		if (!strcmp((const char *)argv[this_arg], "-r")) {
			read_write = 0;
		} else if (!strcmp((const char *)argv[this_arg], "-w")) {
			read_write = 1;
		} else if (!strcmp((const char *)argv[this_arg], "-a")) {
			this_arg++;
			temp = strtoul(argv[this_arg], NULL, 0);
			address = (unsigned short)temp;
		} else if (!strcmp((const char *)argv[this_arg], "-l")) {
			this_arg++;
			temp = strtoul(argv[this_arg], NULL, 0);
			length = (unsigned short)temp;
		} else if (!strcmp((const char *)argv[this_arg], "-d")) {
			this_arg++;
			memcpy(data_input, argv[this_arg], strlen(argv[this_arg]));
		} else {
			usage(argv[0]);
			printf("error: invalid parameter %s\n", argv[this_arg]);
			error_exit(EINVAL);
		}
		this_arg++;
	}

	snprintf(rmidev_open, MAX_STRING_LEN, "%s/%s", mySensor,
			OPEN_FILENAME);
	snprintf(rmidev_data, MAX_STRING_LEN, "%s/%s", mySensor,
			DATA_FILENAME);
	snprintf(rmidev_release, MAX_STRING_LEN, "%s/%s", mySensor,
			RELEASE_FILENAME);

	if (CheckFiles())
		error_exit(ENODEV);
/*
	fd = open(rmidev_open, O_WRONLY);
	if (fd < 0) {
		printf("error: failed to open %s\n", rmidev_open);
		error_exit(EIO);
	}
	writevaluetofd(fd, 1);
	close(fd);
*/
	if (read_write == 1) {
		if ((strlen(data_input) == 0) || (strlen(data_input) % 2)) {
			printf("error: invalid data format\n");
			error_exit(EIO);
		}

		fd = open(rmidev_data, O_WRONLY);
		if (fd < 0) {
			printf("error: failed to open %s\n", rmidev_data);
			error_exit(EIO);
		}

		lseek(fd, address, SEEK_SET);

		strptr = strstr(data_input, "0x");
		if (!strptr) {
			strptr = &data_input[0];
			bytes_to_write = strlen(data_input) / 2;
		} else {
			strptr += 2;
			bytes_to_write = (strlen(data_input) / 2) - 1;
		}

		temp = bytes_to_write;
		while (temp) {
			memcpy(next_value, strptr, 2);
			w_data[index] = (unsigned char)strtoul(next_value, NULL, 16);
			strptr += 2;
			index++;
			temp--;
		}

		retval = write(fd, &w_data, bytes_to_write);
		if (retval != bytes_to_write) {
			printf("error: failed to write data\n");
			close(fd);
			error_exit(EIO);
		}

		close(fd);
	} else {
		r_data = malloc(length);
		if (!r_data) {
			printf("error: failed to allocate r_data buffer\n");
			error_exit(ENOMEM);
		}

		fd = open(rmidev_data, O_RDONLY);

		if (fd < 0) {
			printf("error: failed to open %s\n", rmidev_data);
			error_exit(EIO);
		}

		lseek(fd, address, SEEK_SET);

		retval = read(fd, r_data, length);
		if (retval != length) {
			printf("error: failed to read data\n");
			close(fd);
			error_exit(EIO);
		}

		close(fd);

		for(temp = 0; temp < length; temp++)
			printf("data %d = 0x%02x\n", (unsigned int)temp, r_data[temp]);
	}
/*
	fd = open(rmidev_release, O_WRONLY);
	if (fd < 0) {
		printf("error: failed to open %s\n", rmidev_release);
		error_exit(EIO);
	}
	writevaluetofd(fd, 1);
	close(fd);
*/
	if (r_data)
		free(r_data);

	return 0;
}
