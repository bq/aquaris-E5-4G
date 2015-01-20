
#ifndef _CPS121_H
#define _CPS121_H
#include <linux/sensors.h>

#define CPS121_NAME			"pressure"
#define CPS121_DRV_NAME		"cps121"

/* POWER SUPPLY VOLTAGE RANGE */
#define CPS121_VDD_MIN_UV  2000000
#define CPS121_VDD_MAX_UV  3300000


#define ABS_MIN_PRESSURE	30000
#define ABS_MAX_PRESSURE	120000

#define CPS121_REG_MEASUREREQUEST		0x30
#define CPS121_CMD_MEASUREREQUEST		0x0A
#define CPS121_REG_GETDATA				0x06

#define CPS121_GETDATA_P_LENGTH		3
#define CPS121_GETDATA_T_LENGTH		5

struct cps121_raw_data {
	union {
		struct {
			unsigned char pressure[3];
			unsigned char temperature[2];
		} __packed;
		unsigned char value[5];
	};
};

/* Each client has this additional data */
struct cps121_data {
	struct i2c_client *client;
	struct device *dev;
	struct input_dev	*input;
	struct sensors_classdev cdev;
	struct delayed_work work;
	struct cps121_platform_data *platform_data;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	/* regulator data */
	bool power_state;
	struct regulator *vdd;
	struct regulator *vio;

	struct mutex lock;
	u32	delay;
	u32	enable;
};

struct cps121_platform_data {
	u16	polling_interval;
	u16	measure_delay;
};

#endif
