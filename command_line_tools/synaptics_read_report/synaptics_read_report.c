/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Copyright © 2011, 2012 Synaptics Incorporated. All rights reserved.
 *
 * The information in this file is confidential under the terms
 * of a non-disclosure agreement with Synaptics and is provided
 * AS IS without warranties or guarantees of any kind.
 *
 * The information in this file shall remain the exclusive property
 * of Synaptics and may be the subject of Synaptics patents, in
 * whole or part. Synaptics intellectual property rights in the
 * information in this file are not expressly or implicitly licensed
 * or otherwise transferred to you as a result of such information
 * being made available to you.
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <sys/time.h>

#define VERSION "1.7"

#define INPUT_PATH "/sys/class/input/input"
#define NUMBER_OF_INPUTS_TO_SCAN 20

#define MAX_STRING_LEN 256
#define MAX_INT_LEN 33
#define PATIENCE 10000

#define DATA_FILENAME "f54/report_data"
#define DO_PREPARATION_FILENAME "f54/do_preparation"
#define RESUME_TOUCH_FILENAME "f54/resume_touch"
#define REPORT_TYPE_FILENAME "f54/report_type"
#define GET_REPORT_FILENAME "f54/get_report"
#define REPORT_SIZE_FILENAME "f54/report_size"
#define STATUS_FILENAME "f54/status"
#define RX_ELECTRODES_FILENAME "f54/num_of_mapped_rx"
#define TX_ELECTRODES_FILENAME "f54/num_of_mapped_tx"
#define RESET_FILENAME "reset"
#define DETECT_FILENAME "buildid"

char mySensor[MAX_STRING_LEN];
char input_detect[MAX_STRING_LEN];

unsigned char *data_buffer;

enum report_types {
	F54_8BIT_IMAGE = 1,
	F54_16BIT_IMAGE = 2,
	F54_RAW_16BIT_IMAGE = 3,
	F54_HIGH_RESISTANCE = 4,
	F54_TX_TO_TX_SHORT = 5,
	F54_RX_TO_RX1 = 7,
	F54_TRUE_BASELINE = 9,
	F54_FULL_RAW_CAP_MIN_MAX = 13,
	F54_RX_OPENS1 = 14,
	F54_TX_OPEN = 15,
	F54_TX_TO_GROUND = 16,
	F54_RX_TO_RX2 = 17,
	F54_RX_OPENS2 = 18,
	F54_FULL_RAW_CAP = 19,
	F54_FULL_RAW_CAP_RX_COUPLING_COMP = 20,
	F54_SENSOR_SPEED = 22,
	F54_ADC_RANGE = 23,
	F54_TREX_OPENS = 24,
	F54_TREX_TO_GND = 25,
	F54_TREX_SHORTS = 26,
	F54_ABS_CAP = 38,
	F54_ABS_DELTA = 40,
	F54_ABS_ADC = 42,
	INVALID_REPORT_TYPE = -1,
};

static void usage(char *name)
{
	printf("Version %s\n", VERSION);
	printf("Usage: %s {report_type} [-n {number of readings}] [-c] [-s]\n", name);
	printf("\t[-n] - number of report readings to take\n");
	printf("\t[-c] - display output in 2D format\n");

	return;
}

static void error_exit(error_code)
{
	if (data_buffer)
		free(data_buffer);

	exit(error_code);

	return;
}

static void ReadBinData(char *fname, unsigned char *buf, int len)
{
	int numBytesRead;
	FILE *fp;

	fp = fopen(fname, "r");
	if (!fp) {
		printf("ERROR: failed to open %s for reading data\n", fname);
		error_exit(EIO);
	}

	numBytesRead = fread(buf, 1, len, fp);

	if (numBytesRead != len) {
		printf("ERROR: failed to read all data from bin file\n");
		fclose(fp);
		error_exit(EIO);
	}

	fclose(fp);

	return;
}

static void WriteValueToFp(FILE *fp, unsigned int value)
{
	int numBytesWritten;
	char buf[MAX_INT_LEN];

	snprintf(buf, MAX_INT_LEN, "%u", value);

	fseek(fp, 0, 0);

	numBytesWritten = fwrite(buf, 1, strlen(buf) + 1, fp);
	if (numBytesWritten != ((int)(strlen(buf) + 1))) {
		printf("ERROR: failed to write value to file pointer\n");
		fclose(fp);
		error_exit(EIO);
	}

	return;
}

static void WriteValueToSysfsFile(char *fname, unsigned int value)
{
	FILE *fp;

	fp = fopen(fname, "w");
	if (!fp) {
		printf("ERROR: failed to open %s for writing value\n", fname);
		error_exit(EIO);
	}

	WriteValueToFp(fp, value);

	fclose(fp);

	return;
}

static void ReadValueFromFp(FILE *fp, unsigned int *value)
{
	int retVal;
	char buf[MAX_INT_LEN];

	fseek(fp, 0, 0);

	retVal = fread(buf, 1, sizeof(buf), fp);
	if (retVal == -1) {
		printf("ERROR: failed to read value from file pointer\n");
		error_exit(EIO);
	}

	*value = strtoul(buf, NULL, 0);

	return;
}

static void ReadValueFromSysfsFile(char *fname, unsigned int *value)
{
	FILE *fp;

	fp = fopen(fname, "r");
	if (!fp) {
		printf("ERROR: failed to open %s for reading value\n", fname);
		error_exit(EIO);
	}

	ReadValueFromFp(fp, value);

	fclose(fp);

	return;
}

static void ReadBlockData(char *buf, int len)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, DATA_FILENAME);

	ReadBinData(tmpfname, (unsigned char *)buf, len);

	return;
}

static void DoPreparation(int value)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, DO_PREPARATION_FILENAME);

	WriteValueToSysfsFile(tmpfname, value);

	return;
}

static void ResumeTouch(int value)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, RESUME_TOUCH_FILENAME);

	WriteValueToSysfsFile(tmpfname, value);

	return;
}

static void SetReportType(int value)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, REPORT_TYPE_FILENAME);

	WriteValueToSysfsFile(tmpfname, value);

	return;
}

static void GetReport(int value)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, GET_REPORT_FILENAME);

	WriteValueToSysfsFile(tmpfname, value);

	return;
}

static int ReadReportSize(void)
{
	unsigned int reportSize;
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, REPORT_SIZE_FILENAME);

	ReadValueFromSysfsFile(tmpfname, &reportSize);

	return reportSize;
}

static int GetStatus(void)
{
	unsigned int status;
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, STATUS_FILENAME);

	ReadValueFromSysfsFile(tmpfname, &status);

	return status;
}

static int GetRxElectrodes(void)
{
	unsigned int rx_electrodes;
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, RX_ELECTRODES_FILENAME);

	ReadValueFromSysfsFile(tmpfname, &rx_electrodes);

	return rx_electrodes;
}

static int GetTxElectrodes(void)
{
	unsigned int tx_electrodes;
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, TX_ELECTRODES_FILENAME);

	ReadValueFromSysfsFile(tmpfname, &tx_electrodes);

	return tx_electrodes;
}

static void DoReset(int value)
{
	char tmpfname[MAX_STRING_LEN];

	snprintf(tmpfname, MAX_STRING_LEN, "%s/%s", mySensor, RESET_FILENAME);

	WriteValueToSysfsFile(tmpfname, value);

	return;
}

int main(int argc, char* argv[])
{
	int retval;
	int ii;
	int jj;
	int this_arg = 1;
	int found = 0;
	int readings = 1;
	int reading;;
	int cartesian = 0;
	int patience = PATIENCE;
	int report_type;
	int report_size;
	int rx_num;
	int tx_num;
	char *report_data_8;
	short *report_data_16;
	int *report_data_32;
	unsigned char data_8;
	struct stat st;

	if (argc == 1) {
		usage(argv[0]);
		error_exit(EINVAL);
	}

	for (ii = 0; ii < NUMBER_OF_INPUTS_TO_SCAN; ii++) {
		memset(input_detect, 0x00, MAX_STRING_LEN);
		snprintf(input_detect, MAX_STRING_LEN, "%s%d/%s", INPUT_PATH,
				(unsigned int)ii, DETECT_FILENAME);
		retval = stat(input_detect, &st);
		if (retval == 0) {
			snprintf(mySensor, MAX_STRING_LEN, "%s%d", INPUT_PATH,
					(unsigned int)ii);
			found = 1;
			break;
		}
	}

	if (!found) {
		printf("ERROR: input driver not found\n");
		error_exit(ENODEV);
	}

	while (this_arg < argc) {
		if (!strcmp((const char *)argv[this_arg], "-n")) {
			this_arg++;
			readings = (unsigned int)strtoul(argv[this_arg], NULL, 0);
		} else if (!strcmp((const char *)argv[this_arg], "-c")) {
			cartesian = 1;
		} else {
			report_type = strtoul(argv[this_arg], NULL, 0);
		}
		this_arg++;
	}

	if (cartesian) {
		rx_num = GetRxElectrodes();
		tx_num = GetTxElectrodes();
	}

	switch (report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_SENSOR_SPEED:
	case F54_ADC_RANGE:
	case F54_ABS_CAP:
	case F54_ABS_DELTA:
		break;
	default:
		DoPreparation(1);
		break;
	}

	for (reading = 0; reading < readings; reading++) {
		patience = PATIENCE;
		SetReportType(report_type);
		GetReport(1);
		do {
			if (GetStatus() == 0)
				break;
		} while (--patience > 0);

		report_size = ReadReportSize();
		if (report_size == 0) {
			printf("ERROR: unable to read report\n");
			DoReset(1);
			error_exit(EINVAL);
		}

		if (!data_buffer) {
			data_buffer = malloc(report_size);
			if (!data_buffer) {
				printf("ERROR: failed to allocate report data buffer\n");
				DoReset(1);
				error_exit(ENOMEM);
			}
		}
		ReadBlockData((char *)&data_buffer[0], report_size);

		printf("Reading %d\r\n", reading + 1);

		switch (report_type) {
		case F54_8BIT_IMAGE:
			report_data_8 = (char *)data_buffer;
			for (ii = 0; ii < report_size; ii++) {
				printf("%03d: %d\r\n", ii, *report_data_8);
				report_data_8++;
			}
			break;
		case F54_16BIT_IMAGE:
		case F54_RAW_16BIT_IMAGE:
		case F54_TRUE_BASELINE:
		case F54_FULL_RAW_CAP:
		case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
			report_data_16 = (short *)data_buffer;
			if (cartesian) {
				printf("   ");
				for (ii = 0; ii < rx_num; ii++)
					printf("     %2d", ii);
				printf("\r\n");

				for (ii = 0; ii < tx_num; ii++) {
					printf("%2d ", ii);
					for (jj = 0; jj < rx_num; jj++) {
						printf("  %5d", *report_data_16);
						report_data_16++;
					}
					printf("\r\n");
				}
			} else {
				for (ii = 0; ii < report_size; ii += 2) {
					printf("%03d: %d\r\n", ii / 2, *report_data_16);
					report_data_16++;
				}
			}
			break;
		case F54_HIGH_RESISTANCE:
		case F54_FULL_RAW_CAP_MIN_MAX:
		case F54_SENSOR_SPEED:
		case F54_ADC_RANGE:
			report_data_16 = (short *)data_buffer;
			for (ii = 0; ii < report_size; ii += 2) {
				printf("%03d: %d\r\n", ii / 2, *report_data_16);
				report_data_16++;
			}
			break;
		case F54_ABS_CAP:
		case F54_ABS_DELTA:
			report_data_32 = (int *)data_buffer;
			if (cartesian) {
				printf("Rx ");
				for (ii = 0; ii < rx_num; ii++)
					printf("     %2d", ii);
				printf("\r\n");

				printf("   ");
				for (ii = 0; ii < rx_num; ii++) {
					printf("  %5d", *report_data_32);
					report_data_32++;
				}
				printf("\r\n");

				printf("Tx ");
				for (ii = 0; ii < tx_num; ii++)
					printf("     %2d", ii);
				printf("\r\n");

				printf("   ");
				for (ii = 0; ii < tx_num; ii++) {
					printf("  %5d", *report_data_32);
					report_data_32++;
				}
				printf("\r\n");
			} else {
				for (ii = 0; ii < report_size; ii += 4) {
					printf("%03d: %d\r\n", ii / 4, *report_data_32);
					report_data_32++;
				}
			}
			break;
		case F54_ABS_ADC:
			report_data_16 = (short *)data_buffer;
			for (ii = 0; ii < report_size; ii += 2) {
				data_8 = (unsigned char)*report_data_16;
				printf("%03d: %d\r\n", ii / 2, data_8);
				report_data_16++;
			}
			break;
		default:
			for (ii = 0; ii < report_size; ii++)
				printf("%03d: 0x%02x\r\n", ii, data_buffer[ii]);
			break;
		}
	}

	switch (report_type) {
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_SENSOR_SPEED:
	case F54_ADC_RANGE:
	case F54_ABS_CAP:
	case F54_ABS_DELTA:
		ResumeTouch(1);
		break;
	default:
		DoReset(1);
		break;
	}

	if (data_buffer)
		free(data_buffer);

	return 0;
}
