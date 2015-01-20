////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2012 MStar Semiconductor, Inc.
// All rights reserved.
//
// Unless otherwise stipulated in writing, any and all information contained
// herein regardless in any format shall remain the sole proprietary of
// MStar Semiconductor Inc. and be kept in strict confidence
// (??MStar Confidential Information??) by the recipient.
// Any unauthorized act including without limitation unauthorized disclosure,
// copying, use, reproduction, sale, distribution, modification, disassembling,
// reverse engineering and compiling of the contents of MStar Confidential
// Information is unlawful and strictly prohibited. MStar hereby reserves the
// rights to any and all damages, losses, costs and expenses resulting therefrom.
//
// DEVICE DRIVER RELEASE VERSION : v1.4.0.0
//
////////////////////////////////////////////////////////////////////////////////

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/gpio.h>

#include <linux/sysfs.h>
#include <linux/init.h>
#include <linux/mutex.h>
#include <mach/gpio.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include <linux/syscalls.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <asm/unistd.h>
#include <linux/cdev.h>
#include <asm/uaccess.h>
#include <linux/input.h>
//#include <linux/i2c/common_ts.h>
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#else
#include <linux/earlysuspend.h>
#endif
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "lct_tp_fm_info.h"
#include "lct_ctp_upgrade.h"

/*=============================================================*/
// Macro Definition
/*=============================================================*/

#define TOUCH_DRIVER_DEBUG 0
#if (TOUCH_DRIVER_DEBUG == 1)
//#define DBG(fmt, arg...) pr_err(fmt, ##arg) //pr_info(fmt, ##arg)
#define DBG(fmt, arg...)	printk("<MSG2633>"fmt,##arg);
#else
#define DBG(fmt, arg...)
#endif

/*=============================================================*/
// Constant Value & Variable Definition
/*=============================================================*/

#define U8         unsigned char
#define U16        unsigned short
#define U32        unsigned int
#define S8         signed char
#define S16        signed short
#define S32        signed int

#define TOUCH_SCREEN_X_MIN   (0)
#define TOUCH_SCREEN_Y_MIN   (0)
/*
 * Note.
 * Please change the below touch screen resolution according to the touch panel that you are using.
 */
//#define TOUCH_SCREEN_X_MAX   (480) //LCD_WIDTH
//#define TOUCH_SCREEN_Y_MAX   (800) //LCD_HEIGHT
/*
 * Note.
 * Please do not change the below setting.
 */
#define TPD_WIDTH   (2048)
#define TPD_HEIGHT  (2048)

/*
 * Note.
 * Please change the below GPIO pin setting to follow the platform that you are using(EX. MediaTek, Spreadtrum, Qualcomm).
 */
static int int_gpio = -1;
static int reset_gpio = -1;
#define MS_TS_MSG26XX_GPIO_RST   reset_gpio
#define MS_TS_MSG26XX_GPIO_INT   int_gpio
//---------------------------------------------------------------------//
/*
 * Note.
 * If this compile option is not defined, the SW ID mechanism for updating firmware will be disabled.
 * By default, this compile option is disabled.
 */
#define CONFIG_UPDATE_FIRMWARE_BY_SW_ID

/*
 * Note.
 * If this compile option is defined, the update firmware bin file shall be stored in a two dimensional array format.
 * Else, the update firmware bin file shall be stored in an one dimensional array format.
 * By default, this compile option is enabled.
 */
#define CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY

/*
 * Note.
 * The following is a case list for the integrity check of the firmware on e-flash.
 * Case 1(main block : complete, info block : complete)
 * Case 2(main block : broken, info block : complete)
 * Case 3(main block : complete, info block : broken)
 *
 * If this compile option is defined, for the above three cases, the SW ID mechanism will compare the firmware version for the update firmware bin file and the firmware on the e-flash.
 * Only when the version of the firmware on the e-flash is older than the version of the update firmware bin file, the firmware on the e-flash will be updated.
 *
 * If this compile option is not defined, for the above case 2 and case 3, the SW ID mechanism will not compare the firmware version for the update firmware bin file and the firmware on the e-flash.
 * The firmware on the e-flash will be updated directly if main block or info block is broken.
 * But for the above case 1, the SW ID mechanism will compare the firmware version for the update firmware bin file and the firmware on the e-flash.
 * Only when the version of the firmware on the e-flash is older than the version of the update firmware bin file, the firmware on the e-flash will be updated.
 *
 * By default, this compile option is disabled.
 */
//#define CONFIG_UPDATE_FIRMWARE_WITH_CHECK_VERSION

/* Support to upgrade firmware from LCT engineer mode. */
#define LCT_UPGRADE_FIRMWARE

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
/*
 * Note.
 * Please modify the name of the below .h depends on the vendor TP that you are using.
 */
#include "msg26xx_digjing_update_bin.h"		// xuke @ 20140829	Used for project L6300.
#include "msg26xx_tianma_update_bin.h"

#define UPDATE_FIRMWARE_RETRY_COUNT (2)
#define FIRMWARE_MAIN_BLOCK_SIZE (32) //32K
#define FIRMWARE_INFO_BLOCK_SIZE (8) //8K
#define FIRMWARE_WHOLE_SIZE (FIRMWARE_MAIN_BLOCK_SIZE+FIRMWARE_INFO_BLOCK_SIZE) //40K
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID
//---------------------------------------------------------------------//

// xuke @ 20140922	Correct the sys authority for CTS test.
#if defined(CONFIG_L6140_COMMON) || defined(CONFIG_L6300_COMMON)
#define SYSFS_AUTHORITY_CHANGE_FOR_CTS_TEST
#endif

#ifdef SYSFS_AUTHORITY_CHANGE_FOR_CTS_TEST
#define SYSFS_AUTHORITY (0644)
#else
#define SYSFS_AUTHORITY (0777)
#endif

#define CONFIG_TP_HAVE_KEY

/*
 * Note.
 * If the below virtual key value definition are not consistent with those that defined in key layout file of platform(EX. MediaTek, Spreadtrum, Qualcomm),
 * please change the below virtual key value to follow the platform that you are using.
 */
#ifdef CONFIG_TP_HAVE_KEY
#define TOUCH_KEY_MENU (139) //229
#define TOUCH_KEY_HOME (172) //102
#define TOUCH_KEY_BACK (158)
#define TOUCH_KEY_SEARCH (217)

const U16 tp_key_array[] = {TOUCH_KEY_MENU, TOUCH_KEY_HOME, TOUCH_KEY_BACK, TOUCH_KEY_SEARCH};
#define MAX_KEY_NUM (sizeof(tp_key_array)/sizeof(tp_key_array[0]))
#endif

#define COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define VTG_MIN_UV		2850000
#define VTG_MAX_UV		2850000
#define I2C_VTG_MIN_UV	1800000
#define I2C_VTG_MAX_UV	1800000

#define SLAVE_I2C_ID_DBBUS         (0xC4>>1) //0x62 FW_ADDR_MSG26XX
#define SLAVE_I2C_ID_DWI2C      (0x4C>>1) //0x26 FW_ADDR_MSG26XX_TP

/*
 * Note.
 * The below compile option is used to enable ito open test.
 * By default, this compile option is disabled.
 */
//#define CONFIG_ENABLE_ITO_MP_TEST

#ifdef CONFIG_ENABLE_ITO_MP_TEST
#define BIT0  0x0001
#define BIT1  0x0002
#define BIT2  0x0004
#define BIT3  0x0008
#define BIT4  0x0010
#define BIT5  0x0020
#define BIT6  0x0040
#define BIT7  0x0080
#define BIT8  0x0100
#define BIT9  0x0200
#define BIT10  0x0400
#define BIT11  0x0800
#define BIT12  0x1000
#define BIT13  0x2000
#define BIT14  0x4000
#define BIT15  0x8000

#define MAX_CHANNEL_NUM  38
#define MAX_CHANNEL_DRV  28
#define MAX_CHANNEL_SEN  14
#define MAX_MUTUAL_NUM  (MAX_CHANNEL_DRV * MAX_CHANNEL_SEN)
#define ANA3_MUTUAL_CSUB_NUMBER (192) //192 = 14 * 13 + 10
#define ANA4_MUTUAL_CSUB_NUMBER (MAX_MUTUAL_NUM - ANA3_MUTUAL_CSUB_NUMBER) //200 = 392 - 192
#define FILTER1_MUTUAL_DELTA_C_NUMBER (190) //190 = (6 * 14 + 11) * 2
#define FILTER2_MUTUAL_DELTA_C_NUMBER (594) //594 = (MAX_MUTUAL_NUM - (6 * 14 + 11)) * 2
#define FIR_THRESHOLD    6553
#define FIR_RATIO    25

#define CTP_SELF_TEST_RETRY_COUNT (3)
#endif //CONFIG_ENABLE_ITO_MP_TEST

#define DEMO_MODE_PACKET_LENGTH    (43) //(23)
#define MAX_TOUCH_NUM           (5)     //10

#define MAX_DEBUG_REGISTER_NUM     (10)

/*
 * Note.
 * The below compile option is used to enable debug mode data log for firmware.
 * By default, this compile option is disabled.
 */
//#define CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
#define FIRMWARE_MODE_DEMO_MODE    (0x0005)
#define FIRMWARE_MODE_DEBUG_MODE   (0x0105)

static U16 firmware_mode = 0;
static U16 sense_line_number = 0;
static U16 drive_line_number = 0;
static U16 debug_mode_packet_length = 0;
static U16 debug_mode_packet_header = 0xA5;
static U8 demo_mode_packet[DEMO_MODE_PACKET_LENGTH] = {0};
static U8 *debug_mode_packet = NULL;

static struct kset *example_kset = NULL;
static struct kobject *example_kobj = NULL;
static U16 get_firmware_mode(void);
static int get_firmware_info(void);
static void get_debug_mode_packet_length(void);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

extern int is_tp_driver_loaded;

#define TP_PRINT
#ifdef TP_PRINT
static void tp_print_create_entry(void);
#endif

//#define TP_DEBUG_ON
#ifdef TP_DEBUG_ON
static int tp_debug_on=0;
static void tp_debug_create_entry(void);
#endif

struct msg26xx_platform_data {
	const char *name;
	const char *fw_name;
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
	u32 power_ldo_gpio;
	u32 power_ldo_gpio_flags;
	u32 family_id;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 hard_rst_dly;
	u32 soft_rst_dly;
	u32 num_max_touches;
	bool fw_vkey_support;
	bool no_force_update;
	bool i2c_pull_up;
	bool ignore_id_check;
	int (*power_init) (bool);
	int (*power_on) (bool);
    struct regulator *vdd;
	struct regulator *vcc_i2c;
    struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};

///#define __UPDATE_INFO__

#ifdef __UPDATE_INFO__
static U8 g_dwiic_info_data[8*1024];
#endif

static unsigned char update_complete_flag = 1;

static char *ctp_module_name = NULL;
static char *platform_fw_version = NULL; // internal use firmware version for MStar
static char *fw_version = NULL; // customer firmware version
static U16 fw_version_major = 0;
static U16 fw_version_minor = 0;
static U8 temp[94][1024];
static U32 crc32_table[256];
static int FwDataCnt = 0;
static struct class *firmware_class = NULL;
static struct device *firmware_cmd_dev = NULL;

static U16 debug_reg[MAX_DEBUG_REGISTER_NUM] = {0};
static U32 debug_reg_count = 0;

static struct i2c_client *i2c_client = NULL;

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data);
static struct notifier_block msg26xx_fb_notif;
#elif defined (CONFIG_HAS_EARLYSUSPEND)
static struct early_suspend mstar_ts_early_suspend;
#endif

static int irq_msg26xx = -1;
static struct work_struct msg26xx_wk;
static struct mutex msg26xx_mutex;
static struct input_dev *input_dev = NULL;
struct msg26xx_platform_data *tp_platform_data = NULL;

#ifdef CONFIG_ENABLE_ITO_MP_TEST
s32 deltaC[MAX_MUTUAL_NUM] = {0};
s32 g_result[MAX_MUTUAL_NUM] = {0};
uint8_t g_mode[MAX_MUTUAL_NUM] = {0};

static int is_in_self_test = 0;
static int retry_count = CTP_SELF_TEST_RETRY_COUNT;
static char ctp_self_test_status[10] = {0};
static struct work_struct ctp_self_test_wk;
static struct workqueue_struct *ctp_self_test_wq = NULL;
#endif //CONFIG_ENABLE_ITO_MP_TEST


#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
static int update_retry_count = UPDATE_FIRMWARE_RETRY_COUNT;
static int is_update_info_block_first = 0;
static struct work_struct update_firmware_by_sw_id_wk;
static struct workqueue_struct *update_firmware_by_sw_id_wq = NULL;
static U8 is_update_firmware = 0x00;
static U8 g_temp_data[1024];
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

/*
 * Note.
 * The below compile option is used to enable gesture wakeup feature.
 * By default, this compile option is disabled.
 */
//#define CONFIG_ENABLE_GESTURE_WAKEUP

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
#define GESTURE_WAKEUP_MODE_1     0x01    //0000 0001 (m)
#define GESTURE_WAKEUP_MODE_2     0x02    //0000 0010 (o)
#define GESTURE_WAKEUP_MODE_3     0x04    //0000 0100 (w)
#define GESTURE_WAKEUP_MODE_4     0x08    //0000 1000 (c)
#define GESTURE_WAKEUP_MODE_5     0x10    //0001 0000 (e)
#define GESTURE_WAKEUP_MODE_6     0x20    //0010 0000 (L)
#define GESTURE_WAKEUP_MODE_7     0x40    //0100 0000 (-)(Right Slide)
#define GESTURE_WAKEUP_MODE_8     0x80    //1000 0000 (|)(Down Slide)

static u8 gesture_wakeup_mode = 0x00;
static u8 gesture_wakeup_value = 0;
static u8 gesture_wakeup_flag = 0;
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

/*=============================================================*/
// Data Type Definition
/*=============================================================*/

typedef struct
{
    U16 id;
    U16 x;
    U16 y;
    U16 p;
}   touchPoint_t;

/// max 80+1+1 = 82 bytes
typedef struct
{
    touchPoint_t point[MAX_TOUCH_NUM];
    U8 count;
    U8 keycode;
}   touchInfo_t;

enum i2c_speed
{
    I2C_SLOW = 0,
    I2C_NORMAL = 1, /* Enable erasing/writing for 10 msec. */
    I2C_FAST = 2,   /* Disable EWENB before 10 msec timeout. */
};

typedef enum
{
    EMEM_ALL = 0,
    EMEM_MAIN,
    EMEM_INFO,
} EMEM_TYPE_t;

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
/*
 * Note.
 * 0x0000 and 0xFFFF are not allowed to be defined as SW ID.
 * SW_ID_UNDEFINED is a reserved enum value, do not delete it or modify it.
 * Please modify the SW ID of the below enum value depends on the TP vendor that you are using.
 */
typedef enum {
    SW_ID_DIJING = 0x0001,
    SW_ID_TIANMA = 0x0002,
    SW_ID_UNDEFINED
} SW_ID_ENUM_t;
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

/*=============================================================*/
// Function Declaration
/*=============================================================*/

static int get_customer_firmware_version(void);
static ssize_t firmware_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t size);

/*=============================================================*/
// Function Definition
/*=============================================================*/

/// CRC
static U32 _CRC_doReflect(U32 ref, S8 ch)
{
    U32 value = 0;
    U32 i = 0;

    for (i = 1; i < (ch + 1); i ++)
    {
        if (ref & 1)
        {
            value |= 1 << (ch - i);
        }
        ref >>= 1;
    }

    return value;
}

U32 _CRC_getValue(U32 text, U32 prevCRC)
{
    U32 ulCRC = prevCRC;

    ulCRC = (ulCRC >> 8) ^ crc32_table[(ulCRC & 0xFF) ^ text];

    return ulCRC;
}

static void _CRC_initTable(void)
{
    U32 magic_number = 0x04c11db7;
    U32 i, j;

    for (i = 0; i <= 0xFF; i ++)
    {
        crc32_table[i] = _CRC_doReflect (i, 8) << 24;
        for (j = 0; j < 8; j ++)
        {
            crc32_table[i] = (crc32_table[i] << 1) ^ (crc32_table[i] & (0x80000000L) ? magic_number : 0);
        }
        crc32_table[i] = _CRC_doReflect(crc32_table[i], 32);
    }
}

static void reset_hw(void)
{
    DBG("reset_hw()\n");

    gpio_direction_output(MS_TS_MSG26XX_GPIO_RST, 1); //gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 1);
    gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 0);
    mdelay(100);     /* Note that the RST must be in LOW 10ms at least */
    gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 1);
    mdelay(100);     /* Enable the interrupt service thread/routine for INT after 50ms */
}

static int read_i2c_seq(U8 addr, U8* buf, U16 size)
{
    int rc = 0;
    struct i2c_msg msgs[] =
    {
        {
            .addr = addr,
            .flags = I2C_M_RD, // read flag
            .len = size,
            .buf = buf,
        },
    };

    /* If everything went ok (i.e. 1 msg transmitted), return #bytes
       transmitted, else error code. */
    if (i2c_client != NULL)
    {
        rc = i2c_transfer(i2c_client->adapter, msgs, 1);
        if (rc < 0)
        {
            DBG("read_i2c_seq() error %d\n", rc);
        }
    }
    else
    {
        DBG("i2c_client is NULL\n");
    }

    return rc;
}

static int write_i2c_seq(U8 addr, U8* buf, U16 size)
{
    int rc = 0;
    struct i2c_msg msgs[] =
    {
        {
            .addr = addr,
            .flags = 0, // if read flag is undefined, then it means write flag.
            .len = size,
            .buf = buf,
        },
    };

    /* If everything went ok (i.e. 1 msg transmitted), return #bytes
       transmitted, else error code. */
    if (i2c_client != NULL)
    {
        rc = i2c_transfer(i2c_client->adapter, msgs, 1);
        if ( rc < 0 )
        {
            DBG("write_i2c_seq() error %d\n", rc);
        }
    }
    else
    {
        DBG("i2c_client is NULL\n");
    }

    return rc;
}

static U16 read_reg(U8 bank, U8 addr)
{
    U8 tx_data[3] = {0x10, bank, addr};
    U8 rx_data[2] = {0};

    write_i2c_seq(SLAVE_I2C_ID_DBBUS, &tx_data[0], 3);
    read_i2c_seq(SLAVE_I2C_ID_DBBUS, &rx_data[0], 2);

    return (rx_data[1] << 8 | rx_data[0]);
}

static void write_reg(U8 bank, U8 addr, U16 data)
{
    U8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    write_i2c_seq(SLAVE_I2C_ID_DBBUS, &tx_data[0], 5);
}

static void write_reg_8bit(U8 bank, U8 addr, U8 data)
{
    U8 tx_data[4] = {0x10, bank, addr, data};
    write_i2c_seq(SLAVE_I2C_ID_DBBUS, &tx_data[0], 4);
}

void dbbusDWIICEnterSerialDebugMode(void)
{
    U8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;

    write_i2c_seq(SLAVE_I2C_ID_DBBUS, data, 5);
}

void dbbusDWIICStopMCU(void)
{
    U8 data[1];

    // Stop the MCU
    data[0] = 0x37;

    write_i2c_seq(SLAVE_I2C_ID_DBBUS, data, 1);
}

void dbbusDWIICIICUseBus(void)
{
    U8 data[1];

    // IIC Use Bus
    data[0] = 0x35;

    write_i2c_seq(SLAVE_I2C_ID_DBBUS, data, 1);
}

void dbbusDWIICIICReshape(void)
{
    U8 data[1];

    // IIC Re-shape
    data[0] = 0x71;

    write_i2c_seq(SLAVE_I2C_ID_DBBUS, data, 1);
}

void dbbusDWIICIICNotUseBus(void)
{
    U8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;

    write_i2c_seq(SLAVE_I2C_ID_DBBUS, data, 1);
}

void dbbusDWIICNotStopMCU(void)
{
    U8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;

    write_i2c_seq(SLAVE_I2C_ID_DBBUS, data, 1);
}

void dbbusDWIICExitSerialDebugMode(void)
{
    U8 data[1];

    // Exit the Serial Debug Mode
    data[0] = 0x45;

    write_i2c_seq(SLAVE_I2C_ID_DBBUS, data, 1);

    // Delay some interval to guard the next transaction
    //udelay ( 200 );        // delay about 0.2ms
}

//---------------------------------------------------------------------//

uint16_t AnaGetMutualChannelNum(void)
{
    uint16_t numOfSensor = 0;
    uint16_t u16RegData = 0;

    u16RegData = read_reg(0x10, 0x2E); //bank:ana3, addr:h0017
    numOfSensor = u16RegData & 0x000F;

    DBG("numOfSensor=%d\n", numOfSensor);

    return numOfSensor;
}

uint16_t AnaGetMutualSubframeNum(void)
{
    uint16_t numOfDriver = 0;
    uint16_t u16RegData = 0;

    u16RegData = read_reg(0x12, 0x16); //bank:ana2, addr:h000b
    numOfDriver = ((u16RegData & 0xFF00) >> 8) + 1; //Since we only retrieve 8th~12th bit of reg_m_sf_num, 0xFF00 shall be changed to 0x1F00.

    DBG("numOfDriver=%d\n", numOfDriver);

    return numOfDriver;
}

static u8 get_ic_type(void)
{
    u8 ic_type = 0;

    reset_hw();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // stop mcu
    write_reg_8bit ( 0x0F, 0xE6, 0x01 );
    // disable watch dog
    write_reg ( 0x3C, 0x60, 0xAA55 );
    // get ic type
    ic_type = (0xff)&(read_reg(0x1E, 0xCC));
    printk("%s,ic_type=%d", __func__, ic_type);

    if (ic_type != 1        //msg2133
        && ic_type != 2     //msg21xxA
        && ic_type !=  3)   //msg26xxM
    {
        ic_type = 0;
    }

    reset_hw();

    return ic_type;
}
//---------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_ITO_MP_TEST

uint16_t FW_GetState(void)
{
    uint16_t checkState = 0;

    checkState = read_reg(0x3C, 0xDE); //bank:reg_PIU_MISC_0, addr:h006f

    return checkState;
}

void MCU_Stop(void)
{
    write_reg(0x0F, 0xE6, 0x0001); //bank:mheg5, addr:h0073
}

void AnaSwitch2Mutual(void)
{
    uint16_t u16Temp;

    u16Temp = read_reg(0x11, 0x4A); //bank:ana, addr:h0025
    u16Temp |= BIT0;
    write_reg(0x11, 0x4A, u16Temp);
    u16Temp = read_reg(0x11, 0x16); //bank:ana, addr:h000b
    u16Temp |= (BIT2 | BIT0);
    write_reg(0x11, 0x16, u16Temp);
}

void AnaGetMutualCsub(uint8_t *mode)
{
    uint16_t i, j;
    uint16_t sensorNum;
    uint16_t driverNum;
    uint16_t totalNum;
    uint8_t u8dataAna4[3];
    uint8_t u8dataAna3[3];
    uint8_t u8dataAna41[ANA4_MUTUAL_CSUB_NUMBER]; //200 = 392 - 192
    uint8_t u8dataAna31[ANA3_MUTUAL_CSUB_NUMBER]; //192 = 14 * 13 + 10
    uint8_t mode_Temp[MAX_MUTUAL_NUM];

    totalNum = MAX_MUTUAL_NUM;
    sensorNum = AnaGetMutualChannelNum();
    driverNum = AnaGetMutualSubframeNum();

    if (ANA4_MUTUAL_CSUB_NUMBER > 0)
    {
        mdelay(100);
        for (i = 0; i < ANA4_MUTUAL_CSUB_NUMBER; i ++)
        {
            u8dataAna41[i] = 0;
        }

        u8dataAna4[0] = 0x10;
        u8dataAna4[1] = 0x15; //bank:ana4, addr:h0000
        u8dataAna4[2] = 0x00;

        write_i2c_seq(SLAVE_I2C_ID_DBBUS, &u8dataAna4[0], 3);
        read_i2c_seq(SLAVE_I2C_ID_DBBUS, &u8dataAna41[0], ANA4_MUTUAL_CSUB_NUMBER); //200

        totalNum -= (uint16_t)ANA4_MUTUAL_CSUB_NUMBER;
    }

    for (i = 0; i < totalNum; i ++)
    {
        u8dataAna31[i] = 0;
    }

    mdelay(100);

    u8dataAna3[0] = 0x10;
    u8dataAna3[1] = 0x10; //bank:ana3, addr:h0020
    u8dataAna3[2] = 0x40;

    write_i2c_seq(SLAVE_I2C_ID_DBBUS, &u8dataAna3[0], 3);
    read_i2c_seq(SLAVE_I2C_ID_DBBUS, &u8dataAna31[0], ANA3_MUTUAL_CSUB_NUMBER); //192

    for (i = 0; i < ANA3_MUTUAL_CSUB_NUMBER; i ++)
    {
        mode_Temp[i] = u8dataAna31[i];
    }

    for (i = ANA3_MUTUAL_CSUB_NUMBER; i < (ANA3_MUTUAL_CSUB_NUMBER + ANA4_MUTUAL_CSUB_NUMBER); i ++)
    {
        mode_Temp[i] = u8dataAna41[i - ANA3_MUTUAL_CSUB_NUMBER];
    }

    for (i = 0; i < driverNum; i ++)
    {
        for (j = 0; j < sensorNum; j ++)
        {
            mode[j * driverNum + i] = mode_Temp[i * MAX_CHANNEL_SEN + j];

//            DBG("mode[%d] = %d\n", j * driverNum + i, mode[j * driverNum + i]);
        }
    }
}

void DisableFilterNoiseDetect(void)
{
    uint16_t u16Temp;

    u16Temp = read_reg(0x13, 0x02); //bank:fir, addr:h0001
    u16Temp &= (~(BIT2 | BIT1 | BIT0));
    write_reg(0x13, 0x02, u16Temp);
}

void AnaSWReset(void)
{
    write_reg(0x11, 0x00, 0xFFFF); //bank:ana, addr:h0000
    write_reg(0x11, 0x00, 0x0000);
    mdelay(100);
}

void enableADCOneShot(void)
{
    uint16_t u16Temp;

    write_reg(0x13, 0x0C, BIT15); //bank:fir, addr:h0006
    u16Temp = read_reg(0x12, 0x14); //bank:ana2, addr:h000a
    u16Temp |= BIT0;
    write_reg(0x12, 0x14, u16Temp);
}

void GetMutualOneShotRawIIR(uint16_t resultData[][MAX_CHANNEL_DRV], uint16_t driverNum, uint16_t sensorNum)
{
    uint16_t u16RegData;
    uint16_t i, j;
    uint16_t u16Temp;
    uint8_t tx_data[3];
    uint8_t u8ShotData1[FILTER1_MUTUAL_DELTA_C_NUMBER]; //190 = (6 * 14 + 11) * 2
    uint8_t u8ShotData2[FILTER2_MUTUAL_DELTA_C_NUMBER]; //594 = (MAX_MUTUAL_NUM - (6 * 14 + 11)) * 2

    u16Temp = read_reg(0x3D, 0x08); //bank:intr_ctrl, addr:h0004
    u16Temp &= (~(BIT8 | BIT4));
    write_reg(0x3D, 0x08, u16Temp);

    enableADCOneShot();
    u16RegData = 0;
    while (0x0000 == (u16RegData & BIT8))
    {
        u16RegData = read_reg(0x3D, 0x18); //bank:intr_ctrl, addr:h000c
    }

    for (i = 0; i < FILTER1_MUTUAL_DELTA_C_NUMBER; i ++)
    {
        u8ShotData1[i] = 0;
    }

    for (i = 0; i < FILTER2_MUTUAL_DELTA_C_NUMBER; i ++)
    {
        u8ShotData2[i] = 0;
    }

    mdelay(100);

    tx_data[0] = 0x10;
    tx_data[1] = 0x13; //bank:fir, addr:h0021
    tx_data[2] = 0x42;
    write_i2c_seq(SLAVE_I2C_ID_DBBUS, &tx_data[0], 3);
    read_i2c_seq(SLAVE_I2C_ID_DBBUS, &u8ShotData1[0], FILTER1_MUTUAL_DELTA_C_NUMBER); //190

    mdelay(100);

    tx_data[0] = 0x10;
    tx_data[1] = 0x20; //bank:fir2, addr:h0000
    tx_data[2] = 0x00;
    write_i2c_seq(SLAVE_I2C_ID_DBBUS, &tx_data[0], 3);
    read_i2c_seq(SLAVE_I2C_ID_DBBUS, &u8ShotData2[0], FILTER2_MUTUAL_DELTA_C_NUMBER); //594

    for (j = 0; j < driverNum; j ++)
    {
        for (i = 0; i < sensorNum; i ++)
        {
            // FILTER1 : SF0~SF5, AFE0~AFE13; SF6, AFE0~AFE10
            if ((j <= 5) || ((j == 6) && (i <= 10)))
            {
                u16RegData = (uint16_t)(u8ShotData1[(j * 14 + i) * 2] | u8ShotData1[(j * 14 + i) * 2 + 1] << 8);
                resultData[i][ j] = (short)u16RegData;
            }
            else
            {
                // FILTER2 : SF6, AFE11~AFE13
                if ((j == 6) && (i > 10))
                {
                    u16RegData = (uint16_t)(u8ShotData2[((j - 6) * 14 + (i - 11)) * 2] | u8ShotData2[((j - 6) * 14 + (i - 11)) * 2 + 1] << 8);
                    resultData[i][j] = (short)u16RegData;
                }
                else
                {
                    u16RegData = (uint16_t)(u8ShotData2[6 + ((j - 7) * 14 + i) * 2] | u8ShotData2[6 + ((j - 7) * 14 + i) * 2 + 1] << 8);
                    resultData[i][j] = (short)u16RegData;
                }
            }
        }
    }

    u16Temp = read_reg(0x3D, 0x08); //bank:intr_ctrl, addr:h0004
    u16Temp |= (BIT8 | BIT4);
    write_reg(0x3D, 0x08, u16Temp);
}

void getDeltaC(s32 *target)
{
    s16 sTmp;
    uint16_t rawData[MAX_CHANNEL_SEN][MAX_CHANNEL_DRV];
    uint16_t i, j;
    uint16_t drvNum = 0, senNum = 0, shift = 0;

    senNum = AnaGetMutualChannelNum();
    drvNum = AnaGetMutualSubframeNum();
    GetMutualOneShotRawIIR(rawData, drvNum, senNum);

    for (i = 0; i < senNum; i ++)
    {
        for (j = 0; j < drvNum; j ++)
        {
            shift = (uint16_t)(i * drvNum + j);
            sTmp = (s16)rawData[i][j];
            target[shift] = sTmp;

//            DBG("rawData[%d][%d] = %d\n", i, j, sTmp);
        }
    }
}

int ITO_Open_Test(void)
{
    int ret = 0;
    s32 s16Prev = 0, s16Delta = 0;
    uint16_t i = 0, j = 0;
    uint16_t checkState = 0;
    uint16_t drvNum = 0, senNum = 0;

//    disable_irq(MS_TS_MSG26XX_GPIO_INT);
    disable_irq(irq_msg26xx);
    reset_hw();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(300);

    /*
      0 : SYS_STATE_NULL
      1 : SYS_STATE_INIT
      4 : SYS_STATE_STRIKE
    */
    while ((checkState == 0 || checkState == 1 || checkState == 4) && (i < 10))
    {
        checkState = FW_GetState();
        mdelay(100);
        i ++;
    }

    if (i >= 10)
    {
        dbbusDWIICIICNotUseBus();
        dbbusDWIICNotStopMCU();
        dbbusDWIICExitSerialDebugMode();
        reset_hw();
        mdelay(300);
//        enable_irq(MS_TS_MSG26XX_GPIO_INT);
        enable_irq(irq_msg26xx);
        return -2;
    }
    MCU_Stop();
    mdelay(10);

    senNum = AnaGetMutualChannelNum();
    drvNum = AnaGetMutualSubframeNum();
    AnaSwitch2Mutual();
    AnaGetMutualCsub(g_mode);
    DisableFilterNoiseDetect();
    AnaSWReset();
    getDeltaC(deltaC);

    for (i = 0; i < senNum; i ++)
    {
        DBG("\nSen[%02d]\t", i);

        for (j = 0; j < drvNum; j ++)
        {
            g_result[i * drvNum + j] = (4464*g_mode[i * drvNum + j] - deltaC[i * drvNum + j]);
//            DBG("%d\t", g_result[i * drvNum + j]);
            DBG("%d  %d  %d\t", g_result[i * drvNum + j], 4464*g_mode[i * drvNum + j], deltaC[i * drvNum + j]);
        }
    }

    DBG("\n\n\n");

    for (j = 0; (j < drvNum) && (ret == 0); j ++)
    {
        for (i = 0; (i < senNum) && (ret == 0); i ++)
        {
            if (g_result[i * drvNum + j] < FIR_THRESHOLD)
            {
                ret = -1;
            	  DBG("\nsen%d, drv%d, MIN_Thres=%d\t", i, j, g_result[i * drvNum + j]);
            }

            if (i > 0)
            {
                s16Delta = g_result[i * drvNum + j] > s16Prev ? (g_result[i * drvNum + j] - s16Prev) : (s16Prev - g_result[i * drvNum + j]);
                if (s16Delta > s16Prev*FIR_RATIO/100)
                {
                    ret = -1;
                    DBG("\nsen%d, drv%d, MAX_Ratio=%d,%d\t", i, j, s16Delta, s16Prev);
                }
            }
            s16Prev = g_result[i * drvNum + j];
        }
    }

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();
    reset_hw();
    mdelay(300);
//    enable_irq(MS_TS_MSG26XX_GPIO_INT);
    enable_irq(irq_msg26xx);

    return ret;
}

static void ctp_self_test_do_work(struct work_struct *work)
{
    int ret = 0;

    DBG("ctp_self_test_do_work() is_in_self_test=%d, retry_count=%d\n", is_in_self_test, retry_count);

    ret = ITO_Open_Test();

    DBG("*** ctp self test result = %d ***\n", ret);

    if (ret == 0)
    {
        strcpy(ctp_self_test_status, "PASS");
        is_in_self_test = 0;
        DBG("ctp_self_test_do_work() self test success\n");

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
        restore_firmware_mode_to_debug_mode();
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
    }
    else
    {
        retry_count --;
        if (retry_count > 0)
        {
            DBG("ctp_self_test_do_work() self test retry retry_count=%d\n", retry_count);
            queue_work(ctp_self_test_wq, &ctp_self_test_wk);
        }
        else
        {
            strcpy(ctp_self_test_status, "FAIL");
            is_in_self_test = 0;
            DBG("ctp_self_test_do_work() self test failed\n");

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
            restore_firmware_mode_to_debug_mode();
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
        }
    }
}

#endif //CONFIG_ENABLE_ITO_MP_TEST

//------------------------------------------------------------------------------//
#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID

static U32 calculate_firmware_CRC_from_eflash(EMEM_TYPE_t emem_type)
{
    U32 retval = 0;
    U16 reg_data = 0;

    DBG("calculate_firmware_CRC_from_eflash() emem_type = %d\n", emem_type);

    reset_hw();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(100);

    // Stop mcu
    write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

    // Stop Watchdog
    write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

    // cmd
    write_reg(0x3C, 0xE4, 0xDF4C); //bank:reg_PIU_MISC_0, addr:h0072

    // TP SW reset
    write_reg(0x1E, 0x04, 0x7d60); //bank:chip, addr:h0002
    write_reg(0x1E, 0x04, 0x829F);

    // Start mcu
    write_reg_8bit(0x0F, 0xE6, 0x00); //bank:mheg5, addr:h0073

    mdelay(100);

    // Polling 0x3CE4 is 0x9432
    do
    {
        reg_data = read_reg(0x3C, 0xE4); //bank:reg_PIU_MISC_0, addr:h0072
    } while (reg_data != 0x9432);

    if (emem_type == EMEM_MAIN) // Read calculated main block CRC(32K-8) from register
    {
        retval = read_reg(0x3C, 0x80);
        retval = (retval << 16) | read_reg(0x3C, 0x82);

        DBG("Main Block CRC = 0x%x\n", retval);
    }
    else if (emem_type == EMEM_INFO) // Read calculated info block CRC(8K) from register
    {
        retval = read_reg(0x3C, 0xA0);
        retval = (retval << 16) | read_reg(0x3C, 0xA2);

        DBG("Info Block CRC = 0x%x\n", retval);
    }

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    return retval;
}

static U32 retrieve_firmware_CRC_from_main_block(EMEM_TYPE_t emem_type)
{
    U32 retval = 0;
    U16 reg_data = 0;
    U8 dbbus_tx_data[5] = {0};
    U8 dbbus_rx_data[4] = {0};

    DBG("retrieve_firmware_CRC_from_main_block() emem_type = %d\n", emem_type);

    reset_hw();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(100);

    // Stop mcu
    write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

    // Stop Watchdog
    write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

    // cmd
    write_reg(0x3C, 0xE4, 0xA4AB); //bank:reg_PIU_MISC_0, addr:h0072

    // TP SW reset
    write_reg(0x1E, 0x04, 0x7d60); //bank:chip, addr:h0002
    write_reg(0x1E, 0x04, 0x829F);

    // Start mcu
    write_reg_8bit(0x0F, 0xE6, 0x00); //bank:mheg5, addr:h0073

    mdelay(100);

    // Polling 0x3CE4 is 0x5B58
    do
    {
        reg_data = read_reg(0x3C, 0xE4); //bank:reg_PIU_MISC_0, addr:h0072
    } while (reg_data != 0x5B58);

    dbbus_tx_data[0] = 0x72;
    if (emem_type == EMEM_MAIN) // Read main block CRC(32K-8) from main block
    {
        dbbus_tx_data[1] = 0x7F;
        dbbus_tx_data[2] = 0xF8;
    }
    else if (emem_type == EMEM_INFO) // Read info block CRC(8K) from main block
    {
        dbbus_tx_data[1] = 0x7F;
        dbbus_tx_data[2] = 0xFC;
    }
    dbbus_tx_data[3] = 0x00;
    dbbus_tx_data[4] = 0x04;

    write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 5);
    read_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 4);

    /*
      The order of 4 bytes [ 0 : 1 : 2 : 3 ]
      Ex. CRC32 = 0x12345678
          0x7FF8 = 0x78, 0x7FF9 = 0x56,
          0x7FFA = 0x34, 0x7FFB = 0x12
    */

    retval = dbbus_rx_data[3];
    retval = (retval << 8) | dbbus_rx_data[2];
    retval = (retval << 8) | dbbus_rx_data[1];
    retval = (retval << 8) | dbbus_rx_data[0];

    DBG("CRC = 0x%x\n", retval);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    return retval;
}

static U32 retrieve_info_CRC_from_info_block(void)
{
    U32 retval = 0;
    U16 reg_data = 0;
    U8 dbbus_tx_data[5] = {0};
    U8 dbbus_rx_data[4] = {0};

    DBG("retrieve_info_CRC_from_info_block()\n");

    reset_hw();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(100);

    // Stop mcu
    write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

    // Stop Watchdog
    write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

    // cmd
    write_reg(0x3C, 0xE4, 0xA4AB); //bank:reg_PIU_MISC_0, addr:h0072

    // TP SW reset
    write_reg(0x1E, 0x04, 0x7d60); //bank:chip, addr:h0002
    write_reg(0x1E, 0x04, 0x829F);

    // Start mcu
    write_reg_8bit(0x0F, 0xE6, 0x00); //bank:mheg5, addr:h0073

    mdelay(100);

    // Polling 0x3CE4 is 0x5B58
    do
    {
        reg_data = read_reg(0x3C, 0xE4); //bank:reg_PIU_MISC_0, addr:h0072
    } while (reg_data != 0x5B58);


    // Read info CRC(8K-4) from info block
    dbbus_tx_data[0] = 0x72;
    dbbus_tx_data[1] = 0x80;
    dbbus_tx_data[2] = 0x00;
    dbbus_tx_data[3] = 0x00;
    dbbus_tx_data[4] = 0x04;

    write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 5);
    read_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 4);

    retval = dbbus_rx_data[3];
    retval = (retval << 8) | dbbus_rx_data[2];
    retval = (retval << 8) | dbbus_rx_data[1];
    retval = (retval << 8) | dbbus_rx_data[0];

    DBG("CRC = 0x%x\n", retval);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    return retval;
}

static U32 retrieve_firmware_CRC_from_bin_file(U8 tmpbuf[][1024], EMEM_TYPE_t emem_type)
{
    U32 retval = 0;

    DBG("retrieve_firmware_CRC_from_bin_file() emem_type = %d\n", emem_type);

    if (tmpbuf != NULL)
    {
        if (emem_type == EMEM_MAIN) // Read main block CRC(32K-8) from bin file
        {
            retval = tmpbuf[31][1019];
            retval = (retval << 8) | tmpbuf[31][1018];
            retval = (retval << 8) | tmpbuf[31][1017];
            retval = (retval << 8) | tmpbuf[31][1016];
        }
        else if (emem_type == EMEM_INFO) // Read info block CRC(8K) from bin file
        {
            retval = tmpbuf[31][1023];
            retval = (retval << 8) | tmpbuf[31][1022];
            retval = (retval << 8) | tmpbuf[31][1021];
            retval = (retval << 8) | tmpbuf[31][1020];
        }
    }

    return retval;
}

static U32 calculate_info_CRC_by_device_driver(void)
{
    U32 retval = 0xffffffff;
    U32 i, j;
    U16 reg_data = 0;
    U8 dbbus_tx_data[5] = {0};

    DBG("calculate_info_CRC_by_device_driver()\n");

    reset_hw();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(300);

    // Stop mcu
    write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

    // Stop Watchdog
    write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

    // cmd
    write_reg(0x3C, 0xE4, 0xA4AB); //bank:reg_PIU_MISC_0, addr:h0072

    // TP SW reset
    write_reg(0x1E, 0x04, 0x7d60); //bank:chip, addr:h0002
    write_reg(0x1E, 0x04, 0x829F);

    // Start mcu
    write_reg_8bit(0x0F, 0xE6, 0x00); //bank:mheg5, addr:h0073

    mdelay(100);

    // Polling 0x3CE4 is 0x5B58
    do
    {
        reg_data = read_reg(0x3C, 0xE4); //bank:reg_PIU_MISC_0, addr:h0072
    } while (reg_data != 0x5B58);

    _CRC_initTable();

    // Read info data(8K) from info block
    dbbus_tx_data[0] = 0x72;
    dbbus_tx_data[3] = 0x00; // read 8 bytes
    dbbus_tx_data[4] = 0x08;

    for (i = 0; i < 8; i ++)
    {
        for (j = 0; j < 128; j ++)
        {
            dbbus_tx_data[1] = 0x80 + (i*0x04) + (((j*8)&0xff00)>>8);
            dbbus_tx_data[2] = (j*8)&0x00ff;

            write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 5);

            mdelay(50);

            // Receive info data
            read_i2c_seq(SLAVE_I2C_ID_DWI2C, &g_temp_data[j*8], 8);
        }

        if (i == 0)
        {
            for (j = 4; j < 1024; j ++)
            {
                retval = _CRC_getValue(g_temp_data[j], retval);
            }
        }
        else
        {
            for (j = 0; j < 1024; j ++)
            {
                retval = _CRC_getValue(g_temp_data[j], retval);
            }
        }
    }

    retval = retval ^ 0xffffffff;

    DBG("Info(8K-4) CRC = 0x%x\n", retval);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    return retval;
}

static int compare_8bytes_for_CRC(U8 tmpbuf[][1024])
{
    int retval = -1;
    U16 reg_data = 0;
    U8 dbbus_tx_data[5] = {0};
    U8 dbbus_rx_data[8] = {0};
    U8 crc[8] = {0};

    DBG("compare_8bytes_for_CRC()\n");

    // Read 8 bytes from bin file
    if (tmpbuf != NULL)
    {
        crc[0] = tmpbuf[31][1016];
        crc[1] = tmpbuf[31][1017];
        crc[2] = tmpbuf[31][1018];
        crc[3] = tmpbuf[31][1019];
        crc[4] = tmpbuf[31][1020];
        crc[5] = tmpbuf[31][1021];
        crc[6] = tmpbuf[31][1022];
        crc[7] = tmpbuf[31][1023];
    }

    // Read 8 bytes from the firmware on e-flash
    //reset_hw();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(100);

    // Stop mcu
    write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

    // Stop Watchdog
    write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

    // cmd
    write_reg(0x3C, 0xE4, 0xA4AB); //bank:reg_PIU_MISC_0, addr:h0072

    // TP SW reset
    write_reg(0x1E, 0x04, 0x7d60); //bank:chip, addr:h0002
    write_reg(0x1E, 0x04, 0x829F);

    // Start mcu
    write_reg_8bit(0x0F, 0xE6, 0x00); //bank:mheg5, addr:h0073

    mdelay(100);

    // Polling 0x3CE4 is 0x5B58
    do
    {
        reg_data = read_reg(0x3C, 0xE4); //bank:reg_PIU_MISC_0, addr:h0072
    } while (reg_data != 0x5B58);

    dbbus_tx_data[0] = 0x72;
    dbbus_tx_data[1] = 0x7F;
    dbbus_tx_data[2] = 0xF8;
    dbbus_tx_data[3] = 0x00;
    dbbus_tx_data[4] = 0x08;

    write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 5);
    read_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 8);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    if (crc[0] == dbbus_rx_data[0]
        && crc[1] == dbbus_rx_data[1]
        && crc[2] == dbbus_rx_data[2]
        && crc[3] == dbbus_rx_data[3]
        && crc[4] == dbbus_rx_data[4]
        && crc[5] == dbbus_rx_data[5]
        && crc[6] == dbbus_rx_data[6]
        && crc[7] == dbbus_rx_data[7])
    {
        retval = 0;
    }
    else
    {
        retval = -1;
    }

    DBG("compare 8bytes for CRC = %d\n", retval);

    return retval;
}

static U16 get_SW_ID(EMEM_TYPE_t emem_type)
{
    U16 retval = 0;
    U16 reg_data = 0;
    U8 dbbus_tx_data[5] = {0};
    U8 dbbus_rx_data[4] = {0};

    DBG("get_SW_ID() emem_type = %d\n", emem_type);

    //reset_hw();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(100);

    // Stop mcu
    write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

    // Stop Watchdog
    write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

    // cmd
    write_reg(0x3C, 0xE4, 0xA4AB); //bank:reg_PIU_MISC_0, addr:h0072

    // TP SW reset
    write_reg(0x1E, 0x04, 0x7d60); //bank:chip, addr:h0002
    write_reg(0x1E, 0x04, 0x829F);

    // Start mcu
    write_reg_8bit(0x0F, 0xE6, 0x00); //bank:mheg5, addr:h0073

    mdelay(100);

    // Polling 0x3CE4 is 0x5B58
    do
    {
        reg_data = read_reg(0x3C, 0xE4); //bank:reg_PIU_MISC_0, addr:h0072
    } while (reg_data != 0x5B58);

    dbbus_tx_data[0] = 0x72;
    if (emem_type == EMEM_MAIN) // Read SW ID from main block
    {
        dbbus_tx_data[1] = 0x00;
        dbbus_tx_data[2] = 0x2A;
    }
    else if (emem_type == EMEM_INFO) // Read SW ID from info block
    {
        dbbus_tx_data[1] = 0x80;
        dbbus_tx_data[2] = 0x04;
    }
    dbbus_tx_data[3] = 0x00;
    dbbus_tx_data[4] = 0x04;

    write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 5);
    read_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 4);

    /*
      Ex. SW ID in Main Block :
          Major low byte at address 0x002A
          Major high byte at address 0x002B

          SW ID in Info Block :
          Major low byte at address 0x8004
          Major high byte at address 0x8005
    */

    retval = dbbus_rx_data[1];
    retval = (retval << 8) | dbbus_rx_data[0];

    DBG("SW ID = 0x%x\n", retval);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    return retval;
}

static void erase_firmware_on_eflash(EMEM_TYPE_t emem_type)
{
    DBG("erase_firmware_on_eflash() emem_type = %d\n", emem_type);

    DBG("erase 0\n");

    //reset_hw();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(100);
    // Stop mcu
    write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

    // Stop Watchdog
    write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

    // cmd
    write_reg(0x3C, 0xE4, 0x78C5); //bank:reg_PIU_MISC_0, addr:h0072

    // TP SW reset
    write_reg(0x1E, 0x04, 0x7d60); //bank:chip, addr:h0002
    write_reg(0x1E, 0x04, 0x829F);

    // Start mcu
    write_reg_8bit(0x0F, 0xE6, 0x00); //bank:mheg5, addr:h0073

    mdelay(100);
    DBG("erase 1\n");

    // Stop mcu
    write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

    // Disable watch dog
    write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

    // Set PROGRAM password
    write_reg(0x16, 0x1A, 0xABBA); //bank:emem, addr:h000D

    if (emem_type == EMEM_INFO)
    {
        write_reg(0x16, 0x00, 0x8000); //bank:emem, addr:h0000
    }

    // Clear pce
    write_reg_8bit(0x16, 0x18, 0x80); //bank:emem, addr:h000C

    DBG("erase 2\n");

    // Clear setting
    write_reg_8bit(0x16, 0x18, 0x40); //bank:emem, addr:h000C

    mdelay(10);

    // Clear pce
    write_reg_8bit(0x16, 0x18, 0x80); //bank:emem, addr:h000C

    DBG("erase 3\n");

    // Trigger erase
    if (emem_type == EMEM_ALL)
    {
        write_reg_8bit(0x16, 0x0E, 0x08); //all chip //bank:emem, addr:h0007
    }
    else if (emem_type == EMEM_MAIN || emem_type == EMEM_INFO)
    {
        write_reg_8bit(0x16, 0x0E, 0x04); //sector //bank:emem, addr:h0007
    }

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    mdelay(1000);

    DBG("erase OK\n");
}

static void program_firmware_on_eflash(EMEM_TYPE_t emem_type)
{
    U32 start = 0, end = 0;
    U32 i, j;
    U16 temp1 = 0;
    U16 reg_data = 0;
    U16 reg_data2 = 0, reg_data3 = 0;
    DBG("program_firmware_on_eflash() emem_type = %d\n", emem_type);

    DBG("program 0\n");

    //reset_hw();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(100);

    if (emem_type == EMEM_INFO || emem_type == EMEM_MAIN)
    {
        // Stop mcu
        write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

        // Stop Watchdog
        write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

        // cmd
        write_reg(0x3C, 0xE4, 0x78C5); //bank:reg_PIU_MISC_0, addr:h0072

        // TP SW reset
        write_reg(0x1E, 0x04, 0x7d60); //bank:chip, addr:h0002
        write_reg(0x1E, 0x04, 0x829F);

	temp1 = read_reg(0x16, 0x18);
		
	DBG("*** reg  temp  = 0x%x ***\n", temp1);
	temp1 |= 0x40;
		
	DBG("*** reg  temp  = 0x%x ***\n", temp1);
	write_reg_8bit(0x16,0x18,temp1);

        // Start mcu
        write_reg_8bit(0x0F, 0xE6, 0x00); //bank:mheg5, addr:h0073

        mdelay(100);
    }

    DBG("program 1\n");
    write_reg(0x0F, 0x52, 0xDB00); // Add for analysis

    // Check_Loader_Ready: Polling 0x3CE4 is 0x1C70
    do
    {
        reg_data = read_reg(0x3C, 0xE4); //bank:reg_PIU_MISC_0, addr:h0072
        DBG("*** reg(0x3C, 0xE4) = 0x%x ***\n", reg_data);

        reg_data2 = read_reg(0x0F, 0x00); // Add for analysis

        DBG("*** reg(0x0F, 0x00) = 0x%x ***\n", reg_data2);

        reg_data3 = read_reg(0x1E, 0x04); // Add for analysis

        DBG("*** reg(0x1E, 0x04) = 0x%x ***\n", reg_data3);
        
    } while (reg_data != 0x1C70);

    DBG("program 2\n");

    if (emem_type == EMEM_ALL)
    {
        write_reg(0x3C, 0xE4, 0xE38F);  //all chip

        start = 0;
        end = FIRMWARE_WHOLE_SIZE; //32K + 8K
    }
    else if (emem_type == EMEM_MAIN)
    {
        write_reg(0x3C, 0xE4, 0x7731);  //main block

        start = 0;
        end = FIRMWARE_MAIN_BLOCK_SIZE; //32K
    }
    else if (emem_type == EMEM_INFO)
    {
        write_reg(0x3C, 0xE4, 0xB9D6);  //info block

        start = FIRMWARE_MAIN_BLOCK_SIZE;
        end = FIRMWARE_MAIN_BLOCK_SIZE + FIRMWARE_INFO_BLOCK_SIZE;
    }

    // Check_Loader_Ready2Program: Polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = read_reg(0x3C, 0xE4);
    } while (reg_data != 0x2F43);

    DBG("program 3\n");

    for (i = start; i < end; i ++)
    {
        for (j = 0; j < 8; j ++)
        {
            write_i2c_seq(SLAVE_I2C_ID_DWI2C, &temp[i][j*128], 128);
        }

        mdelay(100);

        // Check_Program_Done: Polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = read_reg(0x3C, 0xE4);
        } while (reg_data != 0xD0BC);

        // Continue_Program
        write_reg(0x3C, 0xE4, 0x2F43);
    }

    DBG("program 4\n");

    // Notify_Write_Done
    write_reg(0x3C, 0xE4, 0x1380);
    mdelay(100);

    DBG("program 5\n");

    // Check_CRC_Done: Polling 0x3CE4 is 0x9432
    do
    {
       reg_data = read_reg(0x3C, 0xE4);
    } while (reg_data != 0x9432);

    DBG("program 6\n");

    FwDataCnt = 0; // Reset FwDataCnt to 0 after update firmware

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    //reset_hw();
    mdelay(300);

    DBG("program OK\n");
}

static int update_firmware_by_SW_ID(void)
{
    int ret = -1;
    U32 crc_info_a = 0, crc_info_b = 0, crc_main_a = 0, crc_main_b = 0;

    DBG("update_firmware_by_SW_ID()\n");

    DBG("is_update_info_block_first=%d, is_update_firmware=0x%x\n", is_update_info_block_first, is_update_firmware);

    if (is_update_info_block_first == 1)
    {
        if ((is_update_firmware & 0x10) == 0x10)
        {
            erase_firmware_on_eflash(EMEM_INFO);
            program_firmware_on_eflash(EMEM_INFO);

            crc_info_a = retrieve_firmware_CRC_from_bin_file(temp, EMEM_INFO);
            crc_info_b = calculate_firmware_CRC_from_eflash(EMEM_INFO);

            DBG("crc_info_a=0x%x, crc_info_b=0x%x\n", crc_info_a, crc_info_b);

            if (crc_info_a == crc_info_b)
            {
                erase_firmware_on_eflash(EMEM_MAIN);
                program_firmware_on_eflash(EMEM_MAIN);

                crc_main_a = retrieve_firmware_CRC_from_bin_file(temp, EMEM_MAIN);
                crc_main_b = calculate_firmware_CRC_from_eflash(EMEM_MAIN);

                DBG("crc_main_a=0x%x, crc_main_b=0x%x\n", crc_main_a, crc_main_b);

                if (crc_main_a == crc_main_b)
                {
                    ret = compare_8bytes_for_CRC(temp);

                    if (ret == 0)
                    {
                        is_update_firmware = 0x00;
                    }
                    else
                    {
                        is_update_firmware = 0x11;
                    }
                }
                else
                {
                    is_update_firmware = 0x01;
                }
            }
            else
            {
                is_update_firmware = 0x11;
            }
        }
        else if ((is_update_firmware & 0x01) == 0x01)
        {
            erase_firmware_on_eflash(EMEM_MAIN);
            program_firmware_on_eflash(EMEM_MAIN);

            crc_main_a = retrieve_firmware_CRC_from_bin_file(temp, EMEM_MAIN);
            crc_main_b = calculate_firmware_CRC_from_eflash(EMEM_MAIN);

            DBG("crc_main_a=0x%x, crc_main_b=0x%x\n", crc_main_a, crc_main_b);

            if (crc_main_a == crc_main_b)
            {
                ret = compare_8bytes_for_CRC(temp);

                if (ret == 0)
                {
                    is_update_firmware = 0x00;
                }
                else
                {
                    is_update_firmware = 0x11;
                }
            }
            else
            {
                is_update_firmware = 0x01;
            }
        }
    }
    else //is_update_info_block_first == 0
    {
        if ((is_update_firmware & 0x10) == 0x10)
        {
            erase_firmware_on_eflash(EMEM_MAIN);
            program_firmware_on_eflash(EMEM_MAIN);

            crc_main_a = retrieve_firmware_CRC_from_bin_file(temp, EMEM_MAIN);
            crc_main_b = calculate_firmware_CRC_from_eflash(EMEM_MAIN);

            DBG("crc_main_a=0x%x, crc_main_b=0x%x\n", crc_main_a, crc_main_b);

            if (crc_main_a == crc_main_b)
            {
                erase_firmware_on_eflash(EMEM_INFO);
                program_firmware_on_eflash(EMEM_INFO);

                crc_info_a = retrieve_firmware_CRC_from_bin_file(temp, EMEM_INFO);
                crc_info_b = calculate_firmware_CRC_from_eflash(EMEM_INFO);

                DBG("crc_info_a=0x%x, crc_info_b=0x%x\n", crc_info_a, crc_info_b);

                if (crc_info_a == crc_info_b)
                {
                    ret = compare_8bytes_for_CRC(temp);

                    if (ret == 0)
                    {
                        is_update_firmware = 0x00;
                    }
                    else
                    {
                        is_update_firmware = 0x11;
                    }
                }
                else
                {
                    is_update_firmware = 0x01;
                }
            }
            else
            {
                is_update_firmware = 0x11;
            }
        }
        else if ((is_update_firmware & 0x01) == 0x01)
        {
            erase_firmware_on_eflash(EMEM_INFO);
            program_firmware_on_eflash(EMEM_INFO);

            crc_info_a = retrieve_firmware_CRC_from_bin_file(temp, EMEM_INFO);
            crc_info_b = calculate_firmware_CRC_from_eflash(EMEM_INFO);

            DBG("crc_info_a=0x%x, crc_info_b=0x%x\n", crc_info_a, crc_info_b);

            if (crc_info_a == crc_info_b)
            {
                ret = compare_8bytes_for_CRC(temp);

                if (ret == 0)
                {
                    is_update_firmware = 0x00;
                }
                else
                {
                    is_update_firmware = 0x11;
                }
            }
            else
            {
                is_update_firmware = 0x01;
            }
        }
    }

    return ret;
}

static void update_firmware_by_SW_ID_do_work(struct work_struct *work)
{
    int ret = 0;

    DBG("update_firmware_by_SW_ID_do_work() update_retry_count=%d\n", update_retry_count);

    ret = update_firmware_by_SW_ID();

    DBG("*** update firmware by software id result = %d ***\n", ret);

    if (ret == 0)
    {
        DBG("update firmware by software id success\n");
        is_update_info_block_first = 0;
        is_update_firmware = 0x00;
		reset_hw();  	
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
	   // get firmware mode for parsing packet judgement.
		firmware_mode = get_firmware_mode();
		get_firmware_info(); // get_firmware_info() must be called first before get_debug_mode_packet_length()
		get_debug_mode_packet_length();
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
		reset_hw(); 
		enable_irq(irq_msg26xx);
    }
    else //ret == -1
    {
        update_retry_count --;
        if (update_retry_count > 0)
        {
            DBG("update_firmware_by_SW_ID_do_work() update_retry_count=%d\n", update_retry_count);
            queue_work(update_firmware_by_sw_id_wq, &update_firmware_by_sw_id_wk);
        }
        else
        {
            DBG("update firmware by software id failed\n");
            is_update_info_block_first = 0;
            is_update_firmware = 0x00;
            reset_hw();
			enable_irq(irq_msg26xx);
        }
    }
}

static void check_firmware_update_by_SW_ID(void)
{
    U32 crc_main_a, crc_info_a, crc_main_b, crc_info_b;
    U32 i;
    U16 update_bin_major = 0;
    U16 update_bin_minor = 0;
    SW_ID_ENUM_t sw_id = SW_ID_UNDEFINED;

    DBG("check_firmware_update_by_SW_ID()\n");

//    disable_irq(MS_TS_MSG26XX_GPIO_INT);
    disable_irq(irq_msg26xx);

    get_customer_firmware_version();

    crc_main_a = calculate_firmware_CRC_from_eflash(EMEM_MAIN);
    crc_main_b = retrieve_firmware_CRC_from_main_block(EMEM_MAIN);

    crc_info_a = calculate_firmware_CRC_from_eflash(EMEM_INFO);
    crc_info_b = retrieve_firmware_CRC_from_main_block(EMEM_INFO);

    update_firmware_by_sw_id_wq = create_singlethread_workqueue("update_firmware_by_sw_id");
    INIT_WORK(&update_firmware_by_sw_id_wk, update_firmware_by_SW_ID_do_work);

    DBG("crc_main_a=0x%x, crc_info_a=0x%x, crc_main_b=0x%x, crc_info_b=0x%x\n", crc_main_a, crc_info_a, crc_main_b, crc_info_b);

    if (crc_main_a == crc_main_b && crc_info_a == crc_info_b) // Case 1. Main Block:OK, Info Block:OK
    {
        sw_id = get_SW_ID(EMEM_MAIN);

        if (sw_id == SW_ID_DIJING)
        {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
            update_bin_major = msg26xx_dijing_update_bin[0][0x2B]<<8 | msg26xx_dijing_update_bin[0][0x2A];
            update_bin_minor = msg26xx_dijing_update_bin[0][0x2D]<<8 | msg26xx_dijing_update_bin[0][0x2C];
#else // By one dimensional array
            update_bin_major = msg26xx_dijing_update_bin[0x002B]<<8 | msg26xx_dijing_update_bin[0x002A];
            update_bin_minor = msg26xx_dijing_update_bin[0x002D]<<8 | msg26xx_dijing_update_bin[0x002C];
#endif
			strcpy(ctp_module_name, "DIJING");
        }
        else if (sw_id == SW_ID_TIANMA)
        {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
            update_bin_major = msg26xx_tianma_update_bin[0][0x2B]<<8 | msg26xx_tianma_update_bin[0][0x2A];
            update_bin_minor = msg26xx_tianma_update_bin[0][0x2D]<<8 | msg26xx_tianma_update_bin[0][0x2C];
#else // By one dimensional array
            update_bin_major = msg26xx_tianma_update_bin[0x002B]<<8 | msg26xx_tianma_update_bin[0x002A];
            update_bin_minor = msg26xx_tianma_update_bin[0x002D]<<8 | msg26xx_tianma_update_bin[0x002C];
#endif
			strcpy(ctp_module_name, "TIANMA");
        }
        else //sw_id == SW_ID_UNDEFINED
        {
            DBG("sw_id = %d is an undefined SW ID.\n", sw_id);

            sw_id = SW_ID_UNDEFINED;
            update_bin_major = 0;
            update_bin_minor = 0;
			strcpy(ctp_module_name, "UNKNOWN");
        }

        DBG("sw_id=%d, fw_version_major=%d, fw_version_minor=%d, update_bin_major=%d, update_bin_minor=%d\n", sw_id, fw_version_major, fw_version_minor, update_bin_major, update_bin_minor);

        if (update_bin_minor > fw_version_minor || 0xffff == fw_version_minor)
        {
            if (sw_id < SW_ID_UNDEFINED && sw_id != 0x0000 && sw_id != 0xFFFF)
            {
                if (sw_id == SW_ID_DIJING)
                {
                    for (i = 0; i < FIRMWARE_WHOLE_SIZE; i ++)
                    {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
                        firmware_data_store(NULL, NULL, msg26xx_dijing_update_bin[i], 1024);
#else // By one dimensional array
                        firmware_data_store(NULL, NULL, &(msg26xx_dijing_update_bin[i*1024]), 1024);
#endif
                    }
                }
                else if (sw_id == SW_ID_TIANMA)
                {
                    for (i = 0; i < FIRMWARE_WHOLE_SIZE; i ++)
                    {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
                        firmware_data_store(NULL, NULL, msg26xx_tianma_update_bin[i], 1024);
#else // By one dimensional array
                        firmware_data_store(NULL, NULL, &(msg26xx_tianma_update_bin[i*1024]), 1024);
#endif
                    }
                }

                FwDataCnt = 0; // Reset FwDataCnt to 0 after copying update firmware data to temp buffer

                update_retry_count = UPDATE_FIRMWARE_RETRY_COUNT;
                is_update_info_block_first = 1; // Set 1 for indicating main block is complete
                is_update_firmware = 0x11;
                queue_work(update_firmware_by_sw_id_wq, &update_firmware_by_sw_id_wk);
				return;
            }
            else
            {
                DBG("The sw id is invalid.\n");
                DBG("Go to normal boot up process.\n");
            }
        }
        else
        {
            DBG("The update bin version is older than or equal to the current firmware version on e-flash.\n");
            DBG("Go to normal boot up process.\n");
        }
    }
    else if (crc_main_a == crc_main_b && crc_info_a != crc_info_b) // Case 2. Main Block:OK, Info Block:FAIL
    {
        sw_id = get_SW_ID(EMEM_MAIN);

        if (sw_id == SW_ID_DIJING)
        {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
            update_bin_major = msg26xx_dijing_update_bin[0][0x2B]<<8 | msg26xx_dijing_update_bin[0][0x2A];
            update_bin_minor = msg26xx_dijing_update_bin[0][0x2D]<<8 | msg26xx_dijing_update_bin[0][0x2C];
#else // By one dimensional array
            update_bin_major = msg26xx_dijing_update_bin[0x002B]<<8 | msg26xx_dijing_update_bin[0x002A];
            update_bin_minor = msg26xx_dijing_update_bin[0x002D]<<8 | msg26xx_dijing_update_bin[0x002C];
#endif
        }
        else if (sw_id == SW_ID_TIANMA)
        {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
            update_bin_major = msg26xx_tianma_update_bin[0][0x2B]<<8 | msg26xx_tianma_update_bin[0][0x2A];
            update_bin_minor = msg26xx_tianma_update_bin[0][0x2D]<<8 | msg26xx_tianma_update_bin[0][0x2C];
#else // By one dimensional array
            update_bin_major = msg26xx_tianma_update_bin[0x002B]<<8 | msg26xx_tianma_update_bin[0x002A];
            update_bin_minor = msg26xx_tianma_update_bin[0x002D]<<8 | msg26xx_tianma_update_bin[0x002C];
#endif
        }
        else //sw_id == SW_ID_UNDEFINED
        {
            DBG("sw_id = %d is an undefined SW ID.\n", sw_id);

            sw_id = SW_ID_UNDEFINED;
            update_bin_major = 0;
            update_bin_minor = 0;
        }

        DBG("sw_id=%d, fw_version_major=%d, fw_version_minor=%d, update_bin_major=%d, update_bin_minor=%d\n", sw_id, fw_version_major, fw_version_minor, update_bin_major, update_bin_minor);

#ifdef CONFIG_UPDATE_FIRMWARE_WITH_CHECK_VERSION
        if (update_bin_minor > fw_version_minor)
        {
#endif //CONFIG_UPDATE_FIRMWARE_WITH_CHECK_VERSION
            if (sw_id < SW_ID_UNDEFINED && sw_id != 0x0000 && sw_id != 0xFFFF)
            {
                if (sw_id == SW_ID_DIJING)
                {
                    for (i = 0; i < FIRMWARE_WHOLE_SIZE; i ++)
                    {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
                        firmware_data_store(NULL, NULL, msg26xx_dijing_update_bin[i], 1024);
#else // By one dimensional array
                        firmware_data_store(NULL, NULL, &(msg26xx_dijing_update_bin[i*1024]), 1024);
#endif
                    }
                }
                else if (sw_id == SW_ID_TIANMA)
                {
                    for (i = 0; i < FIRMWARE_WHOLE_SIZE; i ++)
                    {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
                        firmware_data_store(NULL, NULL, msg26xx_tianma_update_bin[i], 1024);
#else // By one dimensional array
                        firmware_data_store(NULL, NULL, &(msg26xx_tianma_update_bin[i*1024]), 1024);
#endif
                    }
                }

                FwDataCnt = 0; // Reset FwDataCnt to 0 after copying update firmware data to temp buffer

                update_retry_count = UPDATE_FIRMWARE_RETRY_COUNT;
                is_update_info_block_first = 1; // Set 1 for indicating main block is complete
                is_update_firmware = 0x11;
                queue_work(update_firmware_by_sw_id_wq, &update_firmware_by_sw_id_wk);
				return;
			}
            else
            {
                DBG("The sw id is invalid.\n");
                DBG("Go to normal boot up process.\n");
            }
#ifdef CONFIG_UPDATE_FIRMWARE_WITH_CHECK_VERSION
        }
        else
        {
            DBG("The update bin version is older than or equal to the current firmware version on e-flash.\n");
            DBG("Go to normal boot up process.\n");
        }
#endif  //CONFIG_UPDATE_FIRMWARE_WITH_CHECK_VERSION
    }
    else // Case 3. Main Block:FAIL, Info Block:FAIL/OK
    {
        crc_info_a = retrieve_info_CRC_from_info_block();
        crc_info_b = calculate_info_CRC_by_device_driver();

        DBG("8K-4 : crc_info_a=0x%x, crc_info_b=0x%x\n", crc_info_a, crc_info_b);

        if (crc_info_a == crc_info_b) // Check if info block is actually OK.
        {
            sw_id = get_SW_ID(EMEM_INFO);

            if (sw_id == SW_ID_DIJING)
            {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
                update_bin_major = msg26xx_dijing_update_bin[32][0x05]<<8 | msg26xx_dijing_update_bin[32][0x04];
                update_bin_minor = msg26xx_dijing_update_bin[32][0x07]<<8 | msg26xx_dijing_update_bin[32][0x06];
#else // By one dimensional array
                update_bin_major = msg26xx_dijing_update_bin[0x8005]<<8 | msg26xx_dijing_update_bin[0x8004];
                update_bin_minor = msg26xx_dijing_update_bin[0x8007]<<8 | msg26xx_dijing_update_bin[0x8006];
#endif
            }
            else if (sw_id == SW_ID_TIANMA)
            {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
                update_bin_major = msg26xx_tianma_update_bin[32][0x05]<<8 | msg26xx_tianma_update_bin[0][0x04];
                update_bin_minor = msg26xx_tianma_update_bin[32][0x07]<<8 | msg26xx_tianma_update_bin[0][0x06];
#else // By one dimensional array
                update_bin_major = msg26xx_tianma_update_bin[0x8005]<<8 | msg26xx_tianma_update_bin[0x8004];
                update_bin_minor = msg26xx_tianma_update_bin[0x8007]<<8 | msg26xx_tianma_update_bin[0x8006];
#endif
            }
            else //sw_id == SW_ID_UNDEFINED
            {
                DBG("sw_id = %d is an undefined SW ID.\n", sw_id);

                sw_id = SW_ID_UNDEFINED;
                update_bin_major = 0;
                update_bin_minor = 0;
            }

            DBG("sw_id=%d, fw_version_major=%d, fw_version_minor=%d, update_bin_major=%d, update_bin_minor=%d\n", sw_id, fw_version_major, fw_version_minor, update_bin_major, update_bin_minor);

#ifdef CONFIG_UPDATE_FIRMWARE_WITH_CHECK_VERSION
            if (update_bin_minor > fw_version_minor)
            {
#endif //CONFIG_UPDATE_FIRMWARE_WITH_CHECK_VERSION

                if (sw_id < SW_ID_UNDEFINED && sw_id != 0x0000 && sw_id != 0xFFFF)
                {
                    if (sw_id == SW_ID_DIJING)
                    {
                        for (i = 0; i < FIRMWARE_WHOLE_SIZE; i ++)
                        {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
                            firmware_data_store(NULL, NULL, msg26xx_dijing_update_bin[i], 1024);
#else // By one dimensional array
                            firmware_data_store(NULL, NULL, &(msg26xx_dijing_update_bin[i*1024]), 1024);
#endif
                        }
                    }
                    else if (sw_id == SW_ID_TIANMA)
                    {
                        for (i = 0; i < FIRMWARE_WHOLE_SIZE; i ++)
                        {
#ifdef CONFIG_UPDATE_FIRMWARE_BY_TWO_DIMENSIONAL_ARRAY // By two dimensional array
                            firmware_data_store(NULL, NULL, msg26xx_tianma_update_bin[i], 1024);
#else // By one dimensional array
                            firmware_data_store(NULL, NULL, &(msg26xx_tianma_update_bin[i*1024]), 1024);
#endif
                        }
                    }

                    FwDataCnt = 0; // Reset FwDataCnt to 0 after copying update firmware data to temp buffer

                    update_retry_count = UPDATE_FIRMWARE_RETRY_COUNT;
                    is_update_info_block_first = 0; // Set 0 for indicating main block is broken
                    is_update_firmware = 0x11;
                    queue_work(update_firmware_by_sw_id_wq, &update_firmware_by_sw_id_wk);
					return;
				}
                else
                {
                    DBG("The sw id is invalid.\n");
                    DBG("Go to normal boot up process.\n");
                }
#ifdef CONFIG_UPDATE_FIRMWARE_WITH_CHECK_VERSION
            }
            else
            {
                DBG("The update bin version is older than or equal to the current firmware version on e-flash.\n");
                DBG("Go to normal boot up process.\n");
            }
#endif  //CONFIG_UPDATE_FIRMWARE_WITH_CHECK_VERSION
        }
        else
        {
            DBG("Info block is broken.\n");
        }
    }

//    enable_irq(MS_TS_MSG26XX_GPIO_INT);
    reset_hw();
    enable_irq(irq_msg26xx);
}

#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

//------------------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP

static void open_gesture_wakeup(U8 mode)
{
    U8 dbbus_tx_data[3] = {0};
    int rc;

    DBG("*** %s() ***\n", __func__);

    DBG("wakeup mode = 0x%x\n", mode);

    dbbus_tx_data[0] = 0x58;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = (0xFF & mode);

    rc = write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
    if (rc < 0)
    {
        DBG("Enable gesture wakeup failed\n");
    }
    else
    {
        DBG("Enable gesture wakeup success\n");
    }

    gesture_wakeup_flag = 1; // gesture wakeup is enabled
}

static void close_gesture_wakeup(void)
{
    U8 dbbus_tx_data[3] = {0};
    int rc;

    DBG("*** %s() ***\n", __func__);

    dbbus_tx_data[0] = 0x58;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x00;

    rc = write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
    if (rc < 0)
    {
        DBG("Disable gesture wakeup failed\n");
    }
    else
    {
        DBG("Disable gesture wakeup success\n");
    }

    gesture_wakeup_flag = 0; // gesture wakeup is disabled
}

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

//------------------------------------------------------------------------------//

static int convert_char_to_hex_digit(char *s, int length)
{
    int retval = 0;
    int i;

    for (i = 0; i < length; i ++)
    {
        char c = *s++;
        int n = 0;

        if ((i == 0 && c == '0') || (i == 1 && c == 'x'))
        {
            continue;
        }

        if ('0' <= c && c <= '9')
        {
            n = c-'0';
        }
        else if ('a' <= c && c <= 'f')
        {
            n = 10 + c-'a';
        }
        else if ('A' <= c && c <= 'F')
        {
            n = 10 + c-'A';
        }

        retval = n + retval*16;
    }

    return retval;
}

//------------------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG

static U16 get_firmware_mode(void)
{
    U16 mode = 0;

    DBG("get_firmware_mode()\n");

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(300);

    mode = read_reg(0x3C, 0xF4); //bank:reg_PIU_MISC0, addr:h007a

    DBG("firmware_mode=0x%x\n", mode);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();
    mdelay(300);

    return mode;
}

static U16 change_firmware_mode(U16 mode)
{
    U16 fw_mode = 0;

    DBG("change_firmware_mode()\n");

    reset_hw();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(300);

    write_reg(0x3C, 0xF4, mode); //bank:reg_PIU_MISC0, addr:h007a
    fw_mode = read_reg(0x3C, 0xF4);

    DBG("firmware_mode=0x%x\n", firmware_mode);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();
    mdelay(300);

    return fw_mode;
}

static int get_firmware_info(void)
{
    U8 dbbus_tx_data[3] = {0};
    U8 dbbus_rx_data[3] = {0};
    U16 nx = 0, ny = 0, header = 0;
    int ret = 0;

    DBG("get_firmware_info()\n");

    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x48;

    write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
    read_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 3);

    header = dbbus_rx_data[0];
    ny = dbbus_rx_data[1];
    nx = dbbus_rx_data[2];

    DBG("*** header = 0x%x ***\n", header);
    DBG("*** ny = %d ***\n", ny);
    DBG("*** nx = %d ***\n", nx);

//    if (header != 0xA5 && header != 0xAB)
    if (header == 8 && ny == 0 && nx == 9)
    {
        DBG("*** Firmware is old version ***\n");

        dbbusDWIICEnterSerialDebugMode();
        dbbusDWIICStopMCU();
        dbbusDWIICIICUseBus();
        dbbusDWIICIICReshape();
        mdelay(300);

        drive_line_number = AnaGetMutualSubframeNum();
        sense_line_number = AnaGetMutualChannelNum();
        debug_mode_packet_header = 0xA5;

        dbbusDWIICIICNotUseBus();
        dbbusDWIICNotStopMCU();
        dbbusDWIICExitSerialDebugMode();
        mdelay(300);
    }
    else
    {
        DBG("*** Firmware is new version ***\n");

        debug_mode_packet_header = header;
        drive_line_number = ny;
        sense_line_number = nx;
    }

    DBG("*** debug_mode_packet_header = 0x%x ***\n", debug_mode_packet_header);
    DBG("*** drive_line_number = %d ***\n", drive_line_number);
    DBG("*** sense_line_number = %d ***\n", sense_line_number);

    return ret;
}

static void get_debug_mode_packet_length(void)
{
    DBG("get_debug_mode_packet_length()\n");

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(300);

    if (sense_line_number != 0 && drive_line_number != 0)
    {
        if (debug_mode_packet_header == 0xA5)
        {
            // for parsing old debug mode packet
            debug_mode_packet_length = 1+1+1+1+10*3+sense_line_number*drive_line_number*2+1;
            if (debug_mode_packet == NULL)
            {
                debug_mode_packet = kzalloc(sizeof(U8)*debug_mode_packet_length, GFP_KERNEL);
            }
        }
        else if (debug_mode_packet_header == 0xAB)
        {
            // for parsing new debug mode packet
            debug_mode_packet_length = 1+1+1+1+10*3+sense_line_number*drive_line_number*2+sense_line_number*2+drive_line_number*2+2*2+8*2+1;
            if (debug_mode_packet == NULL)
            {
                debug_mode_packet = kzalloc(sizeof(U8)*debug_mode_packet_length, GFP_KERNEL);
            }
        }
        else
        {
            DBG("Undefined debug mode packet header=0x%x\n", debug_mode_packet_header);
        }
        DBG("debug_mode_packet_length=%d\n", debug_mode_packet_length);
    }
    else
    {
        DBG("Failed to retrieve channel number or subframe number.\n");
    }

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();
    mdelay(300);
}

static void restore_firmware_mode_to_debug_mode(void)
{
    U16 mode = get_firmware_mode();

    DBG("*** %s() ***\n", __func__);

    DBG("firmware_mode = 0x%x, mode = 0x%x\n", firmware_mode, mode);

    // Since reset_hw() will reset the value of dummy register(bank:reg_PIU_MISC0, addr:h007a), we must reset the firmware mode again after reset_hw().
    if (firmware_mode == FIRMWARE_MODE_DEBUG_MODE && FIRMWARE_MODE_DEMO_MODE == mode)
    {
        firmware_mode = change_firmware_mode(FIRMWARE_MODE_DEBUG_MODE);
    }
    else
    {
        DBG("firmware mode is not restored\n");
    }
}

#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

//------------------------------------------------------------------------------//

static int get_customer_firmware_version(void)
{
    U8 dbbus_tx_data[3] = {0};
    U8 dbbus_rx_data[4] = {0};
    int ret = 0;
    int rc_w = 0, rc_r = 0;

    DBG("get_customer_firmware_version()\n");

    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x2A;

    mutex_lock(&msg26xx_mutex);
    rc_w = write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
    rc_r = read_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 4);
    mutex_unlock(&msg26xx_mutex);

    fw_version_major = (dbbus_rx_data[1]<<8) + dbbus_rx_data[0];
    fw_version_minor = (dbbus_rx_data[3]<<8) + dbbus_rx_data[2];

    DBG("*** major = %d ***\n", fw_version_major);
    DBG("*** minor = %d ***\n", fw_version_minor);
    if(rc_w<0||rc_r<0)
    {
        fw_version_major = 0xffff;
        fw_version_minor = 0xffff;
    }

    if (fw_version == NULL)
    {
        fw_version = kzalloc(sizeof(char), GFP_KERNEL);
    }

    sprintf(fw_version, "%03d%03d", fw_version_major, fw_version_minor);

    return ret;
}

static int get_platform_firmware_version(void)
{
    int ret = 0;
    U32 i;
    U16 reg_data = 0;
    U8 dbbus_tx_data[5] = {0};
    U8 dbbus_rx_data[16] = {0};

    DBG("get_platform_firmware_version()\n");

    reset_hw();

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(100);

    // Stop mcu
    write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

    // Stop Watchdog
    write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

    // cmd
    write_reg(0x3C, 0xE4, 0xA4AB); //bank:reg_PIU_MISC_0, addr:h0072

    // TP SW reset
    write_reg(0x1E, 0x04, 0x7d60); //bank:chip, addr:h0002
    write_reg(0x1E, 0x04, 0x829F);

    // Start mcu
    write_reg_8bit(0x0F, 0xE6, 0x00); //bank:mheg5, addr:h0073

    mdelay(100);

    // Polling 0x3CE4 is 0x5B58
    do
    {
        reg_data = read_reg(0x3C, 0xE4); //bank:reg_PIU_MISC_0, addr:h0072
    } while (reg_data != 0x5B58);

    // Read platform firmware version from info block
    dbbus_tx_data[0] = 0x72;
    dbbus_tx_data[3] = 0x00;
    dbbus_tx_data[4] = 0x08;

    for (i = 0; i < 2; i ++)
    {
        dbbus_tx_data[1] = 0x80;
        dbbus_tx_data[2] = 0x10 + ((i*8)&0x00ff);

        write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 5);

        mdelay(50);

        read_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[i*8], 8);
    }

    if (platform_fw_version == NULL)
    {
        platform_fw_version = kzalloc(sizeof(char), GFP_KERNEL);
    }

    sprintf(platform_fw_version, "%.16s", dbbus_rx_data);

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    reset_hw();
    mdelay(300);

    DBG("*** platform_fw_version = %s ***\n", platform_fw_version);

    return ret;
}

static S32 update_firmware_cash(EMEM_TYPE_t emem_type)
{
#ifdef __UPDATE_INFO__
    U8 life_counter[2] = {0};
#endif
    U32 i, j;
    U32 crc_main, crc_main_tp;
    U32 crc_info, crc_info_tp;
    U16 reg_data = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

#ifdef __UPDATE_INFO__

    /////////////////////////
    // Update info
    /////////////////////////

    DBG("read info 0\n");

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    mdelay(300);

    DBG("read info 1\n");

    // Stop Watchdog
    write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

    // Stop mcu
    write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

    // Read
    write_reg(0x3C, 0xE4, 0xA4AB); //bank:reg_PIU_MISC_0, addr:h0072

    // TP SW reset
    write_reg(0x1E, 0x04, 0x7d60); //bank:chip, addr:h0002
    write_reg(0x1E, 0x04, 0x829F);

    // Start mcu
    write_reg_8bit(0x0F, 0xE6, 0x00); //bank:mheg5, addr:h0073

    mdelay(100);

    DBG("read info 2\n");

    // Polling 0x3CE4 is 0x5B58
    do
    {
        reg_data = read_reg(0x3C, 0xE4); //bank:reg_PIU_MISC_0, addr:h0072
    } while (reg_data != 0x5B58);

    DBG("read info 3\n");

    // Get 128*8 bytes
    tx_data[0] = 0x72;
    tx_data[3] = 0x00;  //0x04
    tx_data[4] = 0x80;  //0x00

    for (i = 0; i < 8; i ++)
    {
        tx_data[1] = 0x80+(((i*128)&0xff00)>>8);
        tx_data[2] = (i*128)&0x00ff;
        write_i2c_seq(SLAVE_I2C_ID_DWI2C, &tx_data[0], 5);

        mdelay(50);

        // Receive info data
        read_i2c_seq(SLAVE_I2C_ID_DWI2C, &g_dwiic_info_data[i*128], 128);
    }

    DBG("read info 4\n");

    DBG("g_dwiic_info_data = %d%d%d%d%d%d%d%d\n", g_dwiic_info_data[0],
                                                     g_dwiic_info_data[1],
                                                     g_dwiic_info_data[2],
                                                     g_dwiic_info_data[3],
                                                     g_dwiic_info_data[4],
                                                     g_dwiic_info_data[5],
                                                     g_dwiic_info_data[6],
                                                     g_dwiic_info_data[7]);

    if (g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        g_dwiic_info_data[8] = temp[32][8];
        g_dwiic_info_data[9] = temp[32][9];
        g_dwiic_info_data[10] = temp[32][10];
        g_dwiic_info_data[11] = temp[32][11];

        // Updata life counter
        life_counter[1] = ((((g_dwiic_info_data[13] << 8) | g_dwiic_info_data[12]) + 1) >> 8) & 0xFF;
        life_counter[0] = (((g_dwiic_info_data[13] << 8) | g_dwiic_info_data[12]) + 1) & 0xFF;
        g_dwiic_info_data[12] = life_counter[0];
        g_dwiic_info_data[13] = life_counter[1];

        write_reg(0x3C, 0xE4, 0x78C5);
        write_reg(0x1E, 0x04, 0x7d60);

        // SW reset
        write_reg(0x1E, 0x04, 0x829F);
        mdelay(100);

        // Check_Loader_Ready2Program: Polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = read_reg(0x3C, 0xE4);
        } while (reg_data != 0x2F43);

        // transmit 8k info data
        for (i = 0; i < 8*8; i ++)
        {
            write_i2c_seq(SLAVE_I2C_ID_DWI2C, &g_dwiic_info_data[i*128], 128);
        }

        // Check_Program_Done: Polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = read_reg(0x3C, 0xE4);
        } while (reg_data != 0xD0BC);
    }

    DBG("read info 5\n");

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    DBG("read info OK\n");
#endif

    /////////////////////////
    // Erase
    /////////////////////////

    DBG("erase 0\n");
    reset_hw();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(300);

    DBG("erase 1\n");

    // Stop mcu
    write_reg_8bit(0x0F, 0xE6, 0x01); //bank:mheg5, addr:h0073

    // Disable watch dog
    write_reg(0x3C, 0x60, 0xAA55); //bank:reg_PIU_MISC_0, addr:h0030

    // Set PROGRAM password
    write_reg(0x16, 0x1A, 0xABBA); //bank:emem, addr:h000D

    // Clear pce
    write_reg_8bit(0x16, 0x18, 0x80); //bank:emem, addr:h000C

    DBG("erase 2\n");
    // Clear setting
    write_reg_8bit(0x16, 0x18, 0x40); //bank:emem, addr:h000C

    mdelay(10);

    // Clear pce
    write_reg_8bit(0x16, 0x18, 0x80); //bank:emem, addr:h000C

    DBG("erase 3\n");
    // trigger erase
    if (emem_type == EMEM_ALL)
    {
        write_reg_8bit(0x16, 0x0E, 0x08); //all chip //bank:emem, addr:h0007
    }
    else
    {
        write_reg_8bit(0x16, 0x0E, 0x04); //sector //bank:emem, addr:h0007
    }

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    mdelay(1000);
    DBG("erase OK\n");

    /////////////////////////
    // Program
    /////////////////////////

    DBG("program 0\n");

    reset_hw();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(300);

    DBG("program 1\n");

    // Check_Loader_Ready: Polling 0x3CE4 is 0x1C70
    do
    {
        reg_data = read_reg(0x3C, 0xE4); //bank:reg_PIU_MISC_0, addr:h0072
    } while (reg_data != 0x1C70);

    DBG("program 2\n");

    write_reg(0x3C, 0xE4, 0xE38F);  //all chip
    mdelay(100);

    // Check_Loader_Ready2Program: Polling 0x3CE4 is 0x2F43
    do
    {
        reg_data = read_reg(0x3C, 0xE4);
    } while (reg_data != 0x2F43);

    DBG("program 3\n");

    // prepare CRC & send data
    _CRC_initTable();

    for (i = 0; i < (32+8); i ++) // main 32 KB  + info 8KB
    {
        if (i > 31)
        {
            for (j = 0; j < 1024; j ++)
            {
                crc_info = _CRC_getValue(temp[i][j], crc_info);
            }
        }
        else if (i < 31)
        {
            for (j = 0; j < 1024; j ++)
            {
                crc_main = _CRC_getValue(temp[i][j], crc_main);
            }
        }
        else ///if (i == 31)
        {
            temp[i][1014] = 0x5A;
            temp[i][1015] = 0xA5;

            for (j = 0; j < 1016; j ++)
            {
                crc_main = _CRC_getValue(temp[i][j], crc_main);
            }
        }

        for (j = 0; j < 8; j ++)
        {
            write_i2c_seq(SLAVE_I2C_ID_DWI2C, &temp[i][j*128], 128);
        }
        mdelay(100);

        // Check_Program_Done: Polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = read_reg(0x3C, 0xE4);
        } while (reg_data != 0xD0BC);

        // Continue_Program
        write_reg(0x3C, 0xE4, 0x2F43);
    }

    DBG("program 4\n");

    // Notify_Write_Done
    write_reg(0x3C, 0xE4, 0x1380);
    mdelay(100);

    DBG("program 5\n");

    // Check_CRC_Done: Polling 0x3CE4 is 0x9432
    do
    {
       reg_data = read_reg(0x3C, 0xE4);
    } while (reg_data != 0x9432);

    DBG("program 6\n");

    // check CRC
    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    // read CRC from TP
    crc_main_tp = read_reg(0x3C, 0x80);
    crc_main_tp = (crc_main_tp << 16) | read_reg(0x3C, 0x82);
    crc_info_tp = read_reg(0x3C, 0xA0);
    crc_info_tp = (crc_info_tp << 16) | read_reg(0x3C, 0xA2);

    DBG("crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp);

    FwDataCnt = 0;

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    reset_hw();
    mdelay(300);

    if ((crc_main_tp != crc_main) || (crc_info_tp != crc_info))
    {
        DBG("update FAILED\n");

        return -1;
    }

    DBG("update SUCCESS\n");

    return 0;
}

//------------------------------------------------------------------------------//

static ssize_t firmware_debug_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    U32 i;
    U8 bank, addr;
    U16 reg_data[MAX_DEBUG_REGISTER_NUM] = {0};
    U8 out[MAX_DEBUG_REGISTER_NUM*25] = {0}, value[10] = {0};

    DBG("*** firmware_debug_show() ***\n");

    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay(300);

    for (i = 0; i < debug_reg_count; i ++)
    {
        reg_data[i] = read_reg((debug_reg[i] >> 8) & 0xFF, debug_reg[i] & 0xFF);
    }

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    for (i = 0; i < debug_reg_count; i ++)
    {
        bank = (debug_reg[i] >> 8) & 0xFF;
        addr = debug_reg[i] & 0xFF;

        DBG("reg(0x%X,0x%X)=0x%04X\n", bank, addr, reg_data[i]);

        strcat(out, "reg(");
        sprintf(value, "0x%X", bank);
        strcat(out, value);
        strcat(out, ",");
        sprintf(value, "0x%X", addr);
        strcat(out, value);
        strcat(out, ")=");
        sprintf(value, "0x%04X", reg_data[i]);
        strcat(out, value);
        strcat(out, "\n");
    }

    return sprintf(buf, "%s\n", out);
}

static ssize_t firmware_debug_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size)
{
    U32 i;
    char *ch;

    DBG("*** firmware_debug_store() ***\n");

    if (buf != NULL)
    {
        DBG("*** %s() buf[0] = %c ***\n", __func__, buf[0]);
        DBG("*** %s() buf[1] = %c ***\n", __func__, buf[1]);
        DBG("*** %s() buf[2] = %c ***\n", __func__, buf[2]);
        DBG("*** %s() buf[3] = %c ***\n", __func__, buf[3]);
        DBG("*** %s() buf[4] = %c ***\n", __func__, buf[4]);
        DBG("*** %s() buf[5] = %c ***\n", __func__, buf[5]);

        DBG("size = %d\n", size);

        i = 0;
        while ((ch = strsep((char **)&buf, " ,")) && (i < MAX_DEBUG_REGISTER_NUM))
        {
            DBG("ch=%s\n", ch);

            debug_reg[i] = convert_char_to_hex_digit(ch, strlen(ch));

            DBG("debug_reg[%d]=0x%04X\n", i, debug_reg[i]);
            i ++;
        }
        debug_reg_count = i;

        DBG("debug_reg_count = %d\n", debug_reg_count);
    }

    return size;
}

static DEVICE_ATTR(debug, SYSFS_AUTHORITY, firmware_debug_show, firmware_debug_store);

static ssize_t firmware_update_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
    DBG("*** firmware_update_show() fw_version = %s ***\n", fw_version);

    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_update_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size)
{
//    disable_irq(MS_TS_MSG26XX_GPIO_INT);
    disable_irq(irq_msg26xx);

    DBG("*** update fw size = %d ***\n", FwDataCnt);

    if (0 != update_firmware_cash(EMEM_ALL))
    {
        update_complete_flag = 0;
        DBG("update failed\n");
    }

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    restore_firmware_mode_to_debug_mode();
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

//    enable_irq(MS_TS_MSG26XX_GPIO_INT);
    enable_irq(irq_msg26xx);

    return size;
}

static DEVICE_ATTR(update, SYSFS_AUTHORITY, firmware_update_show, firmware_update_store);

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    DBG("*** firmware_version_show() fw_version = %s ***\n", fw_version);

    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    get_customer_firmware_version();

    DBG("*** firmware_version_store() fw_version = %s ***\n", fw_version);

    return size;
}

static DEVICE_ATTR(version, SYSFS_AUTHORITY, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    DBG("*** firmware_data_show() FwDataCnt = %d ***\n", FwDataCnt);

    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
    int count = size / 1024;
    int i;

    for (i = 0; i < count; i ++)
    {
        memcpy(temp[FwDataCnt], buf+(i*1024), 1024);

        FwDataCnt ++;
    }

    DBG("***FwDataCnt = %d ***\n", FwDataCnt);

    if (buf != NULL)
    {
        DBG("*** buf[0] = %c ***\n", buf[0]);
    }

    return size;
}

static DEVICE_ATTR(data, SYSFS_AUTHORITY, firmware_data_show, firmware_data_store);

//------------------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_ITO_MP_TEST

static ssize_t firmware_test_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    DBG("*** ctp_self_test_status = %s ***\n", ctp_self_test_status);

    return sprintf(buf, "%s", ctp_self_test_status);
}

static ssize_t firmware_test_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    if (is_in_self_test == 0)
    {
        DBG("ctp self test start\n");

        strcpy(ctp_self_test_status, "Testing...");
        is_in_self_test = 1;
        retry_count = CTP_SELF_TEST_RETRY_COUNT;
        queue_work(ctp_self_test_wq, &ctp_self_test_wk);
    }

    return size;
}

static DEVICE_ATTR(test, SYSFS_AUTHORITY, firmware_test_show, firmware_test_store);

#endif //CONFIG_ENABLE_ITO_MP_TEST

//------------------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG

static ssize_t firmware_mode_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    U16 mode = get_firmware_mode();

    DBG("*** firmware_mode_show() firmware mode = 0x%x ***\n", mode);

    return sprintf(buf, "%x", mode);
}

static ssize_t firmware_mode_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    U32 mode;

    if (buf != NULL)
    {
        sscanf(buf, "%x", &mode);
        DBG("firmware mode = 0x%x\n", mode);

        if (mode == FIRMWARE_MODE_DEMO_MODE) //demo mode
        {
            firmware_mode = change_firmware_mode(FIRMWARE_MODE_DEMO_MODE);
        }
        else if (mode == FIRMWARE_MODE_DEBUG_MODE) //debug mode
        {
            firmware_mode = change_firmware_mode(FIRMWARE_MODE_DEBUG_MODE);
        }
        else
        {
            DBG("*** Undefined Firmware Mode ***\n");
        }
    }

    DBG("*** firmware_mode_store() firmware mode = 0x%x ***\n", firmware_mode);

    return size;
}

static DEVICE_ATTR(mode, SYSFS_AUTHORITY, firmware_mode_show, firmware_mode_store);

static ssize_t firmware_packet_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    U32 i = 0;
    U32 length = 0;

    DBG("firmware_packet_show()\n");

    if (debug_mode_packet != NULL)
    {
        DBG("firmware_mode=0x%x, debug_mode_packet[0]=%x, debug_mode_packet[1]=%x \n", firmware_mode, debug_mode_packet[0], debug_mode_packet[1]);
        DBG("debug_mode_packet[2]=%x, debug_mode_packet[3]=%x \n", debug_mode_packet[2], debug_mode_packet[3]);
        DBG("debug_mode_packet[4]=%x, debug_mode_packet[5]=%x \n", debug_mode_packet[4], debug_mode_packet[5]);

        if ((firmware_mode == FIRMWARE_MODE_DEBUG_MODE) && (debug_mode_packet[0] == 0xA5 || debug_mode_packet[0] == 0xAB))
        {
            for (i = 0; i < debug_mode_packet_length; i ++)
            {
                buf[i] = debug_mode_packet[i];
            }

            length = debug_mode_packet_length;
            DBG("length = %d \n", length);
        }
        else
        {
            DBG("CURRENT MODE IS NOT DEBUG MODE/WRONG DEBUG MODE HEADER\n");
        }
    }
    else
    {
        DBG("debug_mode_packet is NULL\n");
    }

    return length;
}

static ssize_t firmware_packet_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    DBG("firmware_packet_store()\n");

    return size;
}

static DEVICE_ATTR(packet, SYSFS_AUTHORITY, firmware_packet_show, firmware_packet_store);

static ssize_t firmware_sensor_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    DBG("firmware_sensor_show()\n");

    return sprintf(buf, "%d,%d", sense_line_number, drive_line_number);
}

static ssize_t firmware_sensor_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    DBG("firmware_sensor_store()\n");

    get_firmware_info();

    return size;
}

static DEVICE_ATTR(sensor, SYSFS_AUTHORITY, firmware_sensor_show, firmware_sensor_store);

static ssize_t firmware_header_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    DBG("firmware_header_show()\n");

    return sprintf(buf, "%d", debug_mode_packet_header);
}

static ssize_t firmware_header_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    DBG("firmware_header_store()\n");

    get_firmware_info();

    return size;
}

static DEVICE_ATTR(header, SYSFS_AUTHORITY, firmware_header_show, firmware_header_store);

#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

//------------------------------------------------------------------------------//

static ssize_t firmware_platform_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    DBG("*** firmware_platform_version_show() platform_fw_version = %s ***\n", platform_fw_version);

    return sprintf(buf, "%s\n", platform_fw_version);
}

static ssize_t firmware_platform_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    get_platform_firmware_version();

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    restore_firmware_mode_to_debug_mode();
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    DBG("*** firmware_platform_version_store() platform_fw_version = %s ***\n", platform_fw_version);

    return size;
}

static DEVICE_ATTR(platform_version, SYSFS_AUTHORITY, firmware_platform_version_show, firmware_platform_version_store);

//------------------------------------------------------------------------------//

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP

static ssize_t firmware_gesture_wakeup_mode_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    DBG("*** %s() ***\n", __func__);
    DBG("gesture_wakeup_mode = 0x%x\n", gesture_wakeup_mode);

    return sprintf(buf, "%x", gesture_wakeup_mode);
}

static ssize_t firmware_gesture_wakeup_mode_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    U32 length, wakeup_mode;

    DBG("*** %s() ***\n", __func__);

    if (buf != NULL)
    {
        sscanf(buf, "%x", &wakeup_mode);
        DBG("wakeup_mode = 0x%x\n", wakeup_mode);

        length = size;
        DBG("length = %d\n", length);

        if ((wakeup_mode & GESTURE_WAKEUP_MODE_1) == GESTURE_WAKEUP_MODE_1)
        {
            gesture_wakeup_mode = gesture_wakeup_mode | GESTURE_WAKEUP_MODE_1;
        }
        else
        {
            gesture_wakeup_mode = gesture_wakeup_mode & (~GESTURE_WAKEUP_MODE_1);
        }

        if ((wakeup_mode & GESTURE_WAKEUP_MODE_2) == GESTURE_WAKEUP_MODE_2)
        {
            gesture_wakeup_mode = gesture_wakeup_mode | GESTURE_WAKEUP_MODE_2;
        }
        else
        {
            gesture_wakeup_mode = gesture_wakeup_mode & (~GESTURE_WAKEUP_MODE_2);
        }

        if ((wakeup_mode & GESTURE_WAKEUP_MODE_3) == GESTURE_WAKEUP_MODE_3)
        {
            gesture_wakeup_mode = gesture_wakeup_mode | GESTURE_WAKEUP_MODE_3;
        }
        else
        {
            gesture_wakeup_mode = gesture_wakeup_mode & (~GESTURE_WAKEUP_MODE_3);
        }

        if ((wakeup_mode & GESTURE_WAKEUP_MODE_4) == GESTURE_WAKEUP_MODE_4)
        {
            gesture_wakeup_mode = gesture_wakeup_mode | GESTURE_WAKEUP_MODE_4;
        }
        else
        {
            gesture_wakeup_mode = gesture_wakeup_mode & (~GESTURE_WAKEUP_MODE_4);
        }

        if ((wakeup_mode & GESTURE_WAKEUP_MODE_5) == GESTURE_WAKEUP_MODE_5)
        {
            gesture_wakeup_mode = gesture_wakeup_mode | GESTURE_WAKEUP_MODE_5;
        }
        else
        {
            gesture_wakeup_mode = gesture_wakeup_mode & (~GESTURE_WAKEUP_MODE_5);
        }

        if ((wakeup_mode & GESTURE_WAKEUP_MODE_6) == GESTURE_WAKEUP_MODE_6)
        {
            gesture_wakeup_mode = gesture_wakeup_mode | GESTURE_WAKEUP_MODE_6;
        }
        else
        {
            gesture_wakeup_mode = gesture_wakeup_mode & (~GESTURE_WAKEUP_MODE_6);
        }

        if ((wakeup_mode & GESTURE_WAKEUP_MODE_7) == GESTURE_WAKEUP_MODE_7)
        {
            gesture_wakeup_mode = gesture_wakeup_mode | GESTURE_WAKEUP_MODE_7;
        }
        else
        {
            gesture_wakeup_mode = gesture_wakeup_mode & (~GESTURE_WAKEUP_MODE_7);
        }

        if ((wakeup_mode & GESTURE_WAKEUP_MODE_8) == GESTURE_WAKEUP_MODE_8)
        {
            gesture_wakeup_mode = gesture_wakeup_mode | GESTURE_WAKEUP_MODE_8;
        }
        else
        {
            gesture_wakeup_mode = gesture_wakeup_mode & (~GESTURE_WAKEUP_MODE_8);
        }

        DBG("gesture_wakeup_mode = 0x%x\n", gesture_wakeup_mode);
    }

    return size;
}

static DEVICE_ATTR(gesture_wakeup_mode, SYSFS_AUTHORITY, firmware_gesture_wakeup_mode_show, firmware_gesture_wakeup_mode_store);

#endif //CONFIG_ENABLE_GESTURE_WAKEUP

//------------------------------------------------------------------------------//

#ifdef LCT_UPGRADE_FIRMWARE
static int ctp_upgrade_from_engineermode(struct i2c_client *client)
{
	struct file *pfile = NULL;
	struct inode *inode;
	int fsize = 0;
	u8 *pbt_buf = NULL;
	mm_segment_t old_fs;
	loff_t pos;
	char realfilepath[125] = {0};
	char *filepath = "/mnt/sdcard/CTP_FW.bin";
	char *filepath1 = "/storage/sdcard1/CTP_FW.bin";
	int ret= 1; 
	
	strcpy(realfilepath,filepath);
	pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		printk("error occured while opening file %s.\n",filepath);
		pfile = filp_open(filepath1, O_RDONLY, 0);
		if(IS_ERR(pfile))
		{
			printk("error occured while opening file1 %s.\n",filepath1);
			//strcpy(ctp_upgrade_status,"File no exist");
			lct_set_ctp_upgrade_status("File no exist");
			return -1;
		}
		strcpy(realfilepath,filepath1);
	}
	inode = pfile->f_dentry->d_inode;
	fsize = inode->i_size;

	printk("ctp_upgrade_from_engineermode fsize = %d\n",fsize);

	if(fsize <= 0)
	{
		//strcpy(ctp_upgrade_status,"File size err");
		lct_set_ctp_upgrade_status("File size err");
		return -1;
	}
	filp_close(pfile,NULL);

	//read firmware

	//pbt_buf = kmalloc(fsize + 1, GFP_ATOMIC);
	pbt_buf = &temp[0][0];
	
	if(pbt_buf == NULL)
	{
		lct_set_ctp_upgrade_status("Malloc fail");
		return -1;
	}

	pfile = filp_open(realfilepath, O_RDONLY, 0);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, pbt_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);
	
	firmware_data_store(NULL, NULL, pbt_buf, fsize);
	update_complete_flag = 1;
	firmware_update_store(NULL, NULL, NULL, 0);
	if(update_complete_flag == 1)
	{
		//strcpy(ctp_upgrade_status,"Success");
		lct_set_ctp_upgrade_status("Success");
	}
	else
	{
		//strcpy(ctp_upgrade_status,"Failed");
		lct_set_ctp_upgrade_status("Failed");
	}
	//kfree(pbt_buf);
	return ret; 
}

static int ctp_upgrade_func(void)
{
	return ctp_upgrade_from_engineermode(i2c_client);
}

static void ctp_upgrade_read_ver_func(char *ver)
{
	int cnt= 0;
	
	if(ver == NULL)
		return;

	get_customer_firmware_version();
	
	cnt = sprintf(ver, "vid:%s,fw:%s,ic:%s\n", 
		(ctp_module_name ? ctp_module_name : "Unknown"), 
		(fw_version ? fw_version : "Unknown"), 
		"MSG2633");
	return ;
}
#endif


#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG

static ssize_t kobject_packet_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
    U32 i = 0;
    U32 length = 0;

    DBG("kobject_packet_show()\n");

    if (strcmp(attr->attr.name, "packet") == 0)
    {
        if (debug_mode_packet != NULL)
        {
            DBG("firmware_mode=0x%x, debug_mode_packet[0]=%x, debug_mode_packet[1]=%x \n", firmware_mode, debug_mode_packet[0], debug_mode_packet[1]);
            DBG("debug_mode_packet[2]=%x, debug_mode_packet[3]=%x \n", debug_mode_packet[2], debug_mode_packet[3]);
            DBG("debug_mode_packet[4]=%x, debug_mode_packet[5]=%x \n", debug_mode_packet[4], debug_mode_packet[5]);

            if ((firmware_mode == FIRMWARE_MODE_DEBUG_MODE) && (debug_mode_packet[0] == 0xA5 || debug_mode_packet[0] == 0xAB))
            {
                for (i = 0; i < debug_mode_packet_length; i ++)
                {
                    buf[i] = debug_mode_packet[i];
                }

                length = debug_mode_packet_length;
                DBG("length = %d \n", length);
            }
            else
            {
                DBG("CURRENT MODE IS NOT DEBUG MODE/WRONG DEBUG MODE HEADER\n");
            }
        }
        else
        {
            DBG("debug_mode_packet is NULL\n");
        }
    }
    else
    {
        DBG("attr->attr.name = %s \n", attr->attr.name);
    }

    return length;
}

static ssize_t kobejct_packet_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
    DBG("kobejct_packet_store()\n");
/*
    if (strcmp(attr->attr.name, "packet") == 0)
    {

    }
*/
    return count;
}

static struct kobj_attribute packet_attr = __ATTR(packet, 0666, kobject_packet_show, kobejct_packet_store);

/* Create a group of attributes so that we can create and destroy them all at once. */
static struct attribute *attrs[] = {
    &packet_attr.attr,
    NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * An unnamed attribute group will put all of the attributes directly in
 * the kobject directory. If we specify a name, a subdirectory will be
 * created for the attributes with the directory being the name of the
 * attribute group.
 */
static struct attribute_group attr_group = {
    .attrs = attrs,
};

#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

//------------------------------------------------------------------------------//

static U8 calculate_checksum(U8 *msg, S32 length)
{
    S32 Checksum = 0;
    S32 i;

    for (i = 0; i < length; i ++)
    {
        Checksum += msg[i];
    }

    return (U8)((-Checksum) & 0xFF);
}

static S32 parse_info(touchInfo_t *info)
{
    U32 i, j;
    U8 checksum = 0;
    U32 x = 0, y = 0;
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    U8 *data = NULL;
    U16 report_packet_length = 0;
#else
    U8 data[DEMO_MODE_PACKET_LENGTH] = {0};
    U16 report_packet_length = DEMO_MODE_PACKET_LENGTH;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
	struct msg26xx_platform_data *pdata = dev_get_drvdata(&input_dev->dev);

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    if (firmware_mode == FIRMWARE_MODE_DEMO_MODE)
    {
        DBG("FIRMWARE_MODE_DEMO_MODE\n");

        report_packet_length = DEMO_MODE_PACKET_LENGTH;
        data = demo_mode_packet;
    }
    else if (firmware_mode == FIRMWARE_MODE_DEBUG_MODE)
    {
        DBG("FIRMWARE_MODE_DEBUG_MODE\n");

        if (debug_mode_packet_header != 0xA5 && debug_mode_packet_header != 0xAB)
        {
            DBG("WRONG DEBUG MODE HEADER : 0x%x\n", debug_mode_packet_header);
            return -1;
        }

        if (debug_mode_packet == NULL)
        {
            DBG("debug_mode_packet is NULL\n");
            return -1;
        }

        report_packet_length = debug_mode_packet_length;
        data = debug_mode_packet;
    }
    else
    {
        DBG("WRONG FIRMWARE MODE\n");
        return -1;
    }
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    mutex_lock(&msg26xx_mutex);
#define PKG_SIZE	128
#if 1
	j = report_packet_length/PKG_SIZE;
	for (i=0; i<j; ++i)
	{
		read_i2c_seq(SLAVE_I2C_ID_DWI2C, &data[i*PKG_SIZE], PKG_SIZE);
	}
	read_i2c_seq(SLAVE_I2C_ID_DWI2C, &data[i*PKG_SIZE], report_packet_length%PKG_SIZE);
#else
	for (i=0, j=0; i<report_packet_length; i+=PKG_SIZE, ++j)
	{
		read_i2c_seq(SLAVE_I2C_ID_DWI2C, &data[j*PKG_SIZE], (report_packet_length - i)<PKG_SIZE ? (report_packet_length - i) : PKG_SIZE);
	}
#endif
//    read_i2c_seq(SLAVE_I2C_ID_DWI2C, &data[0], report_packet_length);
    mutex_unlock(&msg26xx_mutex);

    DBG("received raw data from touch panel as following:\n");
    DBG("data[0]=%x \n data[1]=%x data[2]=%x data[3]=%x data[4]=%x \n data[5]=%x data[6]=%x data[7]=%x data[8]=%x \n", \
                data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], data[8]);
    checksum = calculate_checksum(&data[0], (report_packet_length-1));
    DBG("check sum : [%x] == [%x]? \n", data[report_packet_length-1], checksum);
#ifdef TP_DEBUG_ON
    if (tp_debug_on)
    {
        printk(KERN_ERR "check sum : [%x] == [%x]? \n", data[report_packet_length-1], checksum);
    }
#endif

    if (data[report_packet_length-1] != checksum)
    {
        DBG("WRONG CHECKSUM\n");
    #ifdef TP_DEBUG_ON
        if (tp_debug_on)
        {
            printk(KERN_ERR "WRONG CHECKSUM\n");
        }
    #endif
        return -1;
    }

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    if (firmware_mode == FIRMWARE_MODE_DEMO_MODE && data[0] != 0x5A)
    {
        DBG("WRONG DEMO MODE HEADER\n");
        return -1;
    }
    else if (firmware_mode == FIRMWARE_MODE_DEBUG_MODE && data[0] != 0xA5 && data[0] != 0xAB)
    {
        DBG("WRONG DEBUG MODE HEADER\n");
        return -1;
    }
#else
    if (data[0] != 0x5A)
    {
        DBG("WRONG DEMO MODE HEADER\n");
    #ifdef TP_DEBUG_ON
        if (tp_debug_on)
        {
            printk(KERN_ERR "WRONG DEMO MODE HEADER\n");
        }
    #endif
        return -1;
    }
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    if (gesture_wakeup_flag == 1)
    {
        if (data[0] == 0x5A) //gesture wakeup is only supported for demo mode
        {
            U8 wakeup_mode = data[report_packet_length-2] & 0x0F; //Since the gesture wakeup mode is stored in 0th~3th bit of variable "wakeup_mode", we can only retrieve 0th~3th bit of it.

            DBG("wakeup_mode = 0x%x\n", wakeup_mode);

            switch (wakeup_mode)
            {
                case 1:
                    gesture_wakeup_value = GESTURE_WAKEUP_MODE_1;
                    break;
                case 2:
                    gesture_wakeup_value = GESTURE_WAKEUP_MODE_2;
                    break;
                case 3:
                    gesture_wakeup_value = GESTURE_WAKEUP_MODE_3;
                    break;
                case 4:
                    gesture_wakeup_value = GESTURE_WAKEUP_MODE_4;
                    break;
                case 5:
                    gesture_wakeup_value = GESTURE_WAKEUP_MODE_5;
                    break;
                case 6:
                    gesture_wakeup_value = GESTURE_WAKEUP_MODE_6;
                    break;
                case 7:
                    gesture_wakeup_value = GESTURE_WAKEUP_MODE_7;
                    break;
                case 8:
                    gesture_wakeup_value = GESTURE_WAKEUP_MODE_8;
                    break;
                default:
                    gesture_wakeup_value = 0;
                    break;
            }

            DBG("gesture_wakeup_value = 0x%x\n", gesture_wakeup_value);

            if ((gesture_wakeup_mode & gesture_wakeup_value) == GESTURE_WAKEUP_MODE_1 ||
                (gesture_wakeup_mode & gesture_wakeup_value) == GESTURE_WAKEUP_MODE_2 ||
                (gesture_wakeup_mode & gesture_wakeup_value) == GESTURE_WAKEUP_MODE_3 ||
                (gesture_wakeup_mode & gesture_wakeup_value) == GESTURE_WAKEUP_MODE_4 ||
                (gesture_wakeup_mode & gesture_wakeup_value) == GESTURE_WAKEUP_MODE_5 ||
                (gesture_wakeup_mode & gesture_wakeup_value) == GESTURE_WAKEUP_MODE_6 ||
                (gesture_wakeup_mode & gesture_wakeup_value) == GESTURE_WAKEUP_MODE_7 ||
                (gesture_wakeup_mode & gesture_wakeup_value) == GESTURE_WAKEUP_MODE_8
                )
            {
                DBG("Light up screen by gesture wakeup.\n");

                input_report_key(input_dev, KEY_POWER, 1);
                input_sync(input_dev);
                input_report_key(input_dev, KEY_POWER, 0);
                input_sync(input_dev);
            }
        }
        else
        {
            DBG("gesture wakeup is only supported for demo mode.\n");
        }

        return -1;
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

    // Process raw data...
    if (data[0] == 0x5A)
    {
        for (i = 0; i < MAX_TOUCH_NUM; i ++)
        {
            if ((data[(4*i)+1] == 0xFF) && (data[(4*i)+2] == 0xFF) && (data[(4*i)+3] == 0xFF))
            {
                continue;
            }

            x = (((data[(4*i)+1] & 0xF0) << 4) | (data[(4*i)+2]));
            y = (((data[(4*i)+1] & 0x0F) << 8) | (data[(4*i)+3]));

//			printk("%s, x_max=%d, y_max=%d\n", __func__, pdata->x_max, pdata->y_max);
            info->point[info->count].x = x * pdata->x_max / TPD_WIDTH;
            info->point[info->count].y = y * pdata->y_max / TPD_HEIGHT;
            info->point[info->count].p = data[4*(i+1)];
            info->point[info->count].id = i;

            DBG("[x,y]=[%d,%d]\n", x, y);
            DBG("point[%d] : (%d,%d) = %d\n", info->point[info->count].id, info->point[info->count].x, info->point[info->count].y, info->point[info->count].p);

            info->count ++;
        }
    }
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    else if (data[0] == 0xA5 || data[0] == 0xAB)
    {
        for (i = 0; i < MAX_TOUCH_NUM; i ++)
        {
            if ((data[(3*i)+4] == 0xFF) && (data[(3*i)+5] == 0xFF) && (data[(3*i)+6] == 0xFF))
            {
                continue;
            }

            x = (((data[(3*i)+4] & 0xF0) << 4) | (data[(3*i)+5]));
            y = (((data[(3*i)+4] & 0x0F) << 8) | (data[(3*i)+6]));

            info->point[info->count].x = x * pdata->x_max / TPD_WIDTH;
            info->point[info->count].y = y * pdata->y_max / TPD_HEIGHT;
            info->point[info->count].p = 1;
            info->point[info->count].id = i;

            DBG("[x,y]=[%d,%d]\n", x, y);
            DBG("point[%d] : (%d,%d) = %d\n", info->point[info->count].id, info->point[info->count].x, info->point[info->count].y, info->point[info->count].p);

            info->count ++;
        }

        // Notify android application to retrieve debug mode packet from device driver by sysfs.
        if (example_kobj != NULL)
        {
            char *envp[2];
            int ret = 0;

            envp[0] = "STATUS=GET_DEBUG_MODE_PACKET";
            envp[1] = NULL;

            ret = kobject_uevent_env(example_kobj, KOBJ_CHANGE, envp);
            DBG("kobject_uevent_env() ret = %d\n", ret);
        }
    }
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_TP_HAVE_KEY
    if (data[0] == 0x5A)
    {
        U8 button = data[report_packet_length-2]; //Since the key value is stored in 0th~3th bit of variable "button", we can only retrieve 0th~3th bit of it.

//        if (button)
        if (button != 0xFF)
        {
            DBG("button=%x\n", button);

            for (i = 0; i < MAX_KEY_NUM; i ++)
            {
                if ((button & (1<<i)) == (1<<i))
                {
                    if (info->keycode == 0)
                    {
                        info->keycode = i;

                        DBG("key[%d]=%d ...\n", i, tp_key_array[i]);
                    }
                    else
                    {
                        /// if pressing multi-key => no report
                        info->keycode = 0xFF;
                    }
                }
            }
        }
        else
        {
            info->keycode = 0xFF;
        }
    }
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    else if (data[0] == 0xA5 || data[0] == 0xAB)
    {
    		// TODO : waiting for firmware define the virtual key

        if (data[0] == 0xA5)
        {
        	  // Do nothing	because of 0xA5 not define virtual key in the packet
        }
        else if (data[0] == 0xAB)
        {
            U8 button = data[3]; // The pressed virtual key is stored in TimeStamp byte(4th byte) for debug mode packet.

//        if (button)
            if (button != 0xFF)
            {
                DBG("button=%x\n", button);

                for (i = 0; i < MAX_KEY_NUM; i ++)
                {
                    if ((button & (1<<i)) == (1<<i))
                    {
                        if (info->keycode == 0)
                        {
                            info->keycode = i;

                            DBG("key[%d]=%d ...\n", i, tp_key_array[i]);
                        }
                        else
                        {
                            /// if pressing multi-key => no report
                            info->keycode = 0xFF;
                        }
                    }
                }
            }
            else
            {
                info->keycode = 0xFF;
            }
        }
    }
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
#endif //CONFIG_TP_HAVE_KEY

    return 0;
}

static void touch_driver_touch_pressed(int x, int y, int p, int id)
{
    DBG("point touch pressed");
#ifdef TP_DEBUG_ON
    if(tp_debug_on)
    {
        printk(KERN_ERR "point touch pressed (%d,%d)\n", x, y);
    }
#endif

    input_report_key(input_dev, BTN_TOUCH, 1);
    input_report_abs(input_dev, ABS_MT_TRACKING_ID, id);
    input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 1);
    input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 1);
    input_report_abs(input_dev, ABS_MT_POSITION_X, x);
    input_report_abs(input_dev, ABS_MT_POSITION_Y, y);

    input_mt_sync(input_dev);
}

static void touch_driver_touch_released(int x, int y)
{
    DBG("point touch released");
#ifdef TP_DEBUG_ON
    if(tp_debug_on)
    {
        printk(KERN_ERR "point touch released");
    }
#endif

    input_report_key(input_dev, BTN_TOUCH, 0);
    /*
    input_report_key(input_dev, TOUCH_KEY_MENU, 0);
    input_report_key(input_dev, TOUCH_KEY_HOME, 0);
    input_report_key(input_dev, TOUCH_KEY_BACK, 0);
    input_report_key(input_dev, TOUCH_KEY_SEARCH, 0);
    input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, 0);
    input_report_abs(input_dev, ABS_MT_WIDTH_MAJOR, 0);
    */
    input_mt_sync(input_dev);
}

/* read data through I2C then report data to input sub-system when interrupt occurred */
void touch_driver_do_work(struct work_struct *work)
{
    touchInfo_t info;
    int i = 0;
    static int last_keycode = 0xFF;
    static int last_count = 0;

    DBG("touch_driver_do_work()\n");

    memset(&info, 0x0, sizeof(info));

    if (0 == parse_info(&info))
    {
#ifdef CONFIG_TP_HAVE_KEY
        if (info.keycode != 0xFF)   //key touch pressed
        {
            DBG("touch_driver_do_work() info.keycode=%x, last_keycode=%x, tp_key_array[%d]=%d\n", info.keycode, last_keycode, info.keycode, tp_key_array[info.keycode]);
        #ifdef TP_DEBUG_ON
            if (tp_debug_on)
            {
                printk(KERN_ERR "keycode=%x, last_keycode=%x, key_array[%d]=%d\n", info.keycode, last_keycode, info.keycode, tp_key_array[info.keycode]);
            }
        #endif
            if (info.keycode < MAX_KEY_NUM)
            {
                if (info.keycode != last_keycode)
                {
                    DBG("key touch pressed");

                    input_report_key(input_dev, BTN_TOUCH, 1);
                    input_report_key(input_dev, tp_key_array[info.keycode], 1);

                    last_keycode = info.keycode;
                }
                else
                {
                    /// pass duplicate key-pressing
                    DBG("REPEATED KEY\n");
                }
            }
            else
            {
                DBG("WRONG KEY\n");
            }
        }
        else                        //key touch released
        {
            if (last_keycode != 0xFF)
            {
                DBG("key touch released");
            #ifdef TP_DEBUG_ON
                if (tp_debug_on)
                {
                    printk(KERN_ERR "key touch released");
                }
            #endif

                input_report_key(input_dev, BTN_TOUCH, 0);
                input_report_key(input_dev, tp_key_array[last_keycode], 0);

                last_keycode = 0xFF;
            }
        }
#endif //CONFIG_TP_HAVE_KEY

        if (info.count > 0)          //point touch pressed
        {
            for (i = 0; i < info.count; i ++)
            {
                touch_driver_touch_pressed(info.point[i].x, info.point[i].y, info.point[i].p, info.point[i].id);
            }
            last_count = info.count;
        }
        else if (last_count > 0)                        //point touch released
        {
            touch_driver_touch_released(info.point[0].x, info.point[0].y);
            last_count = 0;
        }

        input_sync(input_dev);
    }

//    enable_irq(MS_TS_MSG26XX_GPIO_INT);
    enable_irq(irq_msg26xx);
}

/* The interrupt service routine will be triggered when interrupt occurred */
irqreturn_t touch_driver_isr(int irq, void *dev_id)
{
    DBG("touch_driver_isr()\n");

//    disable_irq_nosync(MS_TS_MSG26XX_GPIO_INT);
    disable_irq_nosync(irq_msg26xx);
    schedule_work(&msg26xx_wk);

    return IRQ_HANDLED;
}

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self, unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;
    printk("fb_notifier_callback, is_update_firmware=%d\n", is_update_firmware);

    if (evdata && evdata->data && event == FB_EVENT_BLANK )
    {
        blank = evdata->data;
        printk("blank = %d\n",*blank);

		if (is_update_firmware)
		{
			return 0;
		}
		
        if (*blank == FB_BLANK_UNBLANK)
        {
            printk("FB_BLANK_UNBLANK\n");
        #ifdef CONFIG_ENABLE_GESTURE_WAKEUP
            if (gesture_wakeup_flag == 1)
            {
                close_gesture_wakeup();
            }
            else
            {
                enable_irq(irq_msg26xx);
            }
        #endif

            gpio_direction_output(MS_TS_MSG26XX_GPIO_RST, 1);
            mdelay(10);
            gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 0);
            mdelay(50);
            gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 1);
            mdelay(20);

            touch_driver_touch_released(0, 0);
            input_sync(input_dev);

        #ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
            restore_firmware_mode_to_debug_mode();
        #endif

        #ifndef CONFIG_ENABLE_GESTURE_WAKEUP
            enable_irq(irq_msg26xx);
        #endif
        }
        else if (*blank == FB_BLANK_POWERDOWN)
        {
            printk("FB_BLANK_POWERDOWN\n");
        #ifdef CONFIG_ENABLE_GESTURE_WAKEUP
            if (gesture_wakeup_mode != 0x00)
            {
                open_gesture_wakeup(gesture_wakeup_mode);
                return 0;
            }
        #endif
            disable_irq(irq_msg26xx);
            gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 0);
        }
    }

    return 0;
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
void touch_driver_early_suspend(struct early_suspend *p)
{
    DBG("touch_driver_early_suspend()\n");

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    if (gesture_wakeup_mode != 0x00)
    {
        open_gesture_wakeup(gesture_wakeup_mode);
        return;
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

//    disable_irq(MS_TS_MSG26XX_GPIO_INT);
    disable_irq(irq_msg26xx);

    gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 0);
}

void touch_driver_early_resume(struct early_suspend *p)
{
    DBG("touch_driver_early_resume()\n");

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    if (gesture_wakeup_flag == 1)
    {
        close_gesture_wakeup();
    }
    else
    {
//    enable_irq(MS_TS_MSG26XX_GPIO_INT);
        enable_irq(irq_msg26xx);
    }
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

    gpio_direction_output(MS_TS_MSG26XX_GPIO_RST, 1); //gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 1);
    mdelay(10);
    gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 0);
    mdelay(10);
    gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 1);
    mdelay(200);

    touch_driver_touch_released(0, 0);
    input_sync(input_dev);

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    restore_firmware_mode_to_debug_mode();
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifndef CONFIG_ENABLE_GESTURE_WAKEUP
//    enable_irq(MS_TS_MSG26XX_GPIO_INT);
    enable_irq(irq_msg26xx);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP
}
#endif

static int touch_driver_power_on(struct i2c_client *client, bool on)
{
	struct msg26xx_platform_data *data = i2c_get_clientdata(client);
	int rc;

	if (!on)
		goto power_off;

	if (gpio_is_valid(data->power_ldo_gpio))
	{
		printk("%s, power_ldo_gpio\n", __func__);
		gpio_set_value(data->power_ldo_gpio, 1);
	}
	else
	{
		printk("%s, regulator\n", __func__);
		rc = regulator_enable(data->vdd);
		if (rc)
		{
			dev_err(&client->dev, "Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc)
	{
		dev_err(&client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	if (gpio_is_valid(data->power_ldo_gpio))
	{
		gpio_set_value(data->power_ldo_gpio, 0);
	}
	else
	{
		rc = regulator_disable(data->vdd);
		if (rc)
		{
			dev_err(&client->dev, "Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
	}
	rc = regulator_disable(data->vcc_i2c);
	if (rc)
	{
		dev_err(&client->dev, "Regulator vcc_i2c disable failed rc=%d\n", rc);
	}

	return rc;
}

static int touch_driver_power_init(struct i2c_client *client, bool on)
{
	struct msg26xx_platform_data *data = i2c_get_clientdata(client);
	int rc;

	if (!on)
		goto pwr_deinit;
	
	if (gpio_is_valid(data->power_ldo_gpio))
	{
		printk("%s, power_ldo_gpio\n", __func__);
		rc = gpio_request(data->power_ldo_gpio, "msg21xx_ldo_gpio");
		if (rc)
		{
			printk("irq gpio request failed\n");
			return rc;
		}
		
		rc = gpio_direction_output(data->power_ldo_gpio, 1);
		if (rc)
		{
			printk("set_direction for irq gpio failed\n");
			goto free_ldo_gpio;
		}
	}
	else
	{
		printk("%s, regulator\n", __func__);
		data->vdd = regulator_get(&client->dev, "vdd");
		if (IS_ERR(data->vdd))
		{
			rc = PTR_ERR(data->vdd);
			dev_err(&client->dev, "Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0)
		{
			rc = regulator_set_voltage(data->vdd, VTG_MIN_UV, VTG_MAX_UV);
			if (rc)
			{
				dev_err(&client->dev, "Regulator set_vtg failed vdd rc=%d\n", rc);
				goto reg_vdd_put;
			}
		}
	}
	
	data->vcc_i2c = regulator_get(&client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c))
	{
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&client->dev, "Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0)
	{
		rc = regulator_set_voltage(data->vcc_i2c, I2C_VTG_MIN_UV, I2C_VTG_MAX_UV);
		if (rc)
		{
			dev_err(&client->dev, "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (!gpio_is_valid(data->power_ldo_gpio))
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, VTG_MAX_UV);
reg_vdd_put:
free_ldo_gpio:	
	if (gpio_is_valid(data->power_ldo_gpio))
		gpio_free(data->power_ldo_gpio);
	else
		regulator_put(data->vdd);
	return rc;
pwr_deinit:
	if (gpio_is_valid(data->power_ldo_gpio))
		gpio_free(data->power_ldo_gpio);
	else
	{
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, VTG_MAX_UV);

		regulator_put(data->vdd);
	}
	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}

static int touch_driver_pinctrl_init(struct i2c_client *client)
{
	int retval;
	struct msg26xx_platform_data *data = i2c_get_clientdata(client);
	
	/* Get pinctrl if target uses pinctrl */
	data->ts_pinctrl = devm_pinctrl_get(&client->dev);
	if (IS_ERR_OR_NULL(data->ts_pinctrl))
	{
		dev_dbg(&client->dev, "Target does not use pinctrl\n");
		retval = PTR_ERR(data->ts_pinctrl);
		data->ts_pinctrl = NULL;
		return retval;
	}
    
	data->gpio_state_active	= pinctrl_lookup_state(data->ts_pinctrl, "pmx_ts_active");
	if (IS_ERR_OR_NULL(data->gpio_state_active))
	{
		printk("%s Can not get ts default pinstate\n", __func__);
		retval = PTR_ERR(data->gpio_state_active);
		data->ts_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_suspend = pinctrl_lookup_state(data->ts_pinctrl, "pmx_ts_suspend");
	if (IS_ERR_OR_NULL(data->gpio_state_suspend))
	{
		dev_err(&client->dev,	"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(data->gpio_state_suspend);
		data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int touch_driver_pinctrl_select(struct i2c_client *client, bool on)
{
	struct pinctrl_state *pins_state;
	struct msg26xx_platform_data *data = i2c_get_clientdata(client);
	int ret;
	
	pins_state = on ? data->gpio_state_active : data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state))
	{
		ret = pinctrl_select_state(data->ts_pinctrl, pins_state);
		if (ret)
		{
			dev_err(&client->dev, "can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	}
	else
	{
		dev_err(&client->dev,	"not a valid '%s' pinstate\n",
			on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return 0;
}

#ifdef CONFIG_OF
static int touch_driver_get_dt_coords(struct device *dev, char *name, struct msg26xx_platform_data *pdata)
{
	u32 coords[COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != COORDS_ARR_SIZE)
	{
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL))
	{
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "mstar,panel-coords"))
	{
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	}
	else if (!strcmp(name, "mstar,display-coords"))
	{
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	}
	else
	{
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int touch_driver_parse_dt(struct device *dev, struct msg26xx_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];

//	printk("%s\n", __func__);
	pdata->name = "mstar";
	rc = of_property_read_string(np, "mstar,name", &pdata->name);
	if (rc && (rc != -EINVAL))
	{
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = touch_driver_get_dt_coords(dev, "mstar,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = touch_driver_get_dt_coords(dev, "mstar,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np, "mstar,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np, "mstar,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "mstar,reset-gpio", 0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;
	else
		printk("%s, reset_gpio=%d\n", __func__, pdata->reset_gpio);

	pdata->irq_gpio = of_get_named_gpio_flags(np, "mstar,irq-gpio", 0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;
	else
		printk("%s, irq_gpio=%d\n", __func__, pdata->irq_gpio);

	/* power ldo gpio info*/
	pdata->power_ldo_gpio = of_get_named_gpio_flags(np, "mstar,power_ldo-gpio", 0, &pdata->power_ldo_gpio_flags);
	if (pdata->power_ldo_gpio < 0)
#if 0
		return pdata->power_ldo_gpio;
#else
		printk("%s, power_ldo_gpio=%d\n", __func__, pdata->power_ldo_gpio);
#endif
	
	rc = of_property_read_u32(np, "mstar,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "mstar,button-map", NULL);
	if (prop)
	{
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np, "mstar,button-map", button_map, num_buttons);
		if (rc)
		{
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int touch_driver_parse_dt(struct device *dev, struct msg21xx_platform_data *pdata)
{
	return -ENODEV;
}
#endif

/* probe function is used for matching and initializing input device */
static int touch_driver_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret = 0;
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    char *devpath = NULL;
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

    DBG("*** %s ***\n", __FUNCTION__);

	if(is_tp_driver_loaded == 1)
	{
		printk(KERN_ERR"msg21xx_ts_probe other driver has been loaded\n");
		return ENODEV;
	}
	
    if (input_dev != NULL)
    {
        DBG("input device has found\n");
        return -1;
    }
	
    if (client->dev.of_node)
	{
		tp_platform_data = devm_kzalloc(&client->dev, sizeof(struct msg26xx_platform_data), GFP_KERNEL);
		if (!tp_platform_data)
		{
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		ret = touch_driver_parse_dt(&client->dev, tp_platform_data);
		if (ret) {
			dev_err(&client->dev, "DT parsing failed\n");
			goto Err_parse_dt;
		}
	}
	else
	{
		tp_platform_data = client->dev.platform_data;
		if (!tp_platform_data)
		{
			dev_err(&client->dev, "Invalid tp_platform_data\n");
			return -EINVAL;
		}
	}

    i2c_client = client;
	i2c_set_clientdata(client, tp_platform_data);
	reset_gpio = tp_platform_data->reset_gpio;
	int_gpio = tp_platform_data->irq_gpio;
	
	ret = touch_driver_pinctrl_init(client);
	if (!ret && tp_platform_data->ts_pinctrl)
	{
		ret = touch_driver_pinctrl_select(client, true);
		if (ret < 0)
			goto Err_pinctrl_init;
	}
	
	ret = touch_driver_power_init(client, true);
	if (ret) {
		dev_err(&client->dev, "power init failed");
		goto Err_power_init;
	}

	ret = touch_driver_power_on(client, true);
	if (ret) {
		dev_err(&client->dev, "power on failed");
		goto Err_power_on;
	}

	if (gpio_is_valid(MS_TS_MSG26XX_GPIO_RST))
	{
	    ret = gpio_request(MS_TS_MSG26XX_GPIO_RST, "reset");
	    if (ret)
	    {
	        pr_err("*** Failed to request GPIO %d, error %d ***\n", MS_TS_MSG26XX_GPIO_RST, ret);
	        goto Err_gpio_request_reset;
	    }

	    // power on TP
	    ret = gpio_direction_output(MS_TS_MSG26XX_GPIO_RST, 1);
		if (ret)
		{
			dev_err(&client->dev, "set_direction for reset gpio failed\n");
			goto Err_reset_direction_output;
		}

	    mdelay(20);
	    gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 0);
	    mdelay(10);
	    gpio_set_value(MS_TS_MSG26XX_GPIO_RST, 1);
	    mdelay(20);
	}
	
    if (0 == get_ic_type())
    {
        pr_err("the currnet ic is not Mstar\n");
        ret = -1;
        goto err0;
    }

    mutex_init(&msg26xx_mutex);

    /* allocate an input device */
    input_dev = input_allocate_device();
    if (!input_dev)
    {
        ret = -ENOMEM;
        pr_err("*** input device allocation failed ***\n");
        goto err1;
    }

    input_dev->name = client->name;
    input_dev->phys = "I2C";
    input_dev->dev.parent = &client->dev;
    input_dev->id.bustype = BUS_I2C;

	dev_set_drvdata(&input_dev->dev, tp_platform_data);
	
    /* set the supported event type for input device */
    set_bit(EV_ABS, input_dev->evbit);
    set_bit(EV_SYN, input_dev->evbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(BTN_TOUCH, input_dev->keybit);
    set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

#ifdef CONFIG_TP_HAVE_KEY
    {
        int i;
        for (i = 0; i < MAX_KEY_NUM; i ++)
        {
            input_set_capability(input_dev, EV_KEY, tp_key_array[i]);
        }
    }
#endif
/*
#ifdef CONFIG_TP_HAVE_KEY
    set_bit(TOUCH_KEY_MENU, input_dev->keybit); //menu
    set_bit(TOUCH_KEY_HOME, input_dev->keybit); //home
    set_bit(TOUCH_KEY_BACK, input_dev->keybit); //back
    set_bit(TOUCH_KEY_SEARCH, input_dev->keybit); //search
#endif
*/

#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    input_set_capability(input_dev, EV_KEY, KEY_POWER);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

    input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_X, TOUCH_SCREEN_X_MIN, tp_platform_data->x_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_POSITION_Y, TOUCH_SCREEN_Y_MIN, tp_platform_data->y_max, 0, 0);
    input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, 255, 0, 0);

    /* register the input device to input sub-system */
    ret = input_register_device(input_dev);
    if (ret < 0)
    {
        pr_err("*** Unable to register ms-touchscreen input device ***\n");
        goto err1;
    }

    /* set sysfs for firmware */
    firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx"); //client->name
    if (IS_ERR(firmware_class))
        pr_err("Failed to create class(firmware)!\n");

    firmware_cmd_dev = device_create(firmware_class, NULL, 0, NULL, "device");
    if (IS_ERR(firmware_cmd_dev))
        pr_err("Failed to create device(firmware_cmd_dev)!\n");

    // debug
    if (device_create_file(firmware_cmd_dev, &dev_attr_debug) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_debug.attr.name);
    // version
    if (device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
    if (device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
    if (device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);
#ifdef CONFIG_ENABLE_ITO_MP_TEST
    // test
    if (device_create_file(firmware_cmd_dev, &dev_attr_test) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_test.attr.name);
#endif //CONFIG_ENABLE_ITO_MP_TEST
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    // mode
    if (device_create_file(firmware_cmd_dev, &dev_attr_mode) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_mode.attr.name);
    // packet
    if (device_create_file(firmware_cmd_dev, &dev_attr_packet) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_packet.attr.name);
    // sensor
    if (device_create_file(firmware_cmd_dev, &dev_attr_sensor) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_sensor.attr.name);
    // header
    if (device_create_file(firmware_cmd_dev, &dev_attr_header) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_header.attr.name);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
    // platform version
    if (device_create_file(firmware_cmd_dev, &dev_attr_platform_version) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_platform_version.attr.name);
#ifdef CONFIG_ENABLE_GESTURE_WAKEUP
    // gesture wakeup mode
    if (device_create_file(firmware_cmd_dev, &dev_attr_gesture_wakeup_mode) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_gesture_wakeup_mode.attr.name);
#endif //CONFIG_ENABLE_GESTURE_WAKEUP

    dev_set_drvdata(firmware_cmd_dev, NULL);

#ifdef TP_PRINT
    tp_print_create_entry();
#endif

#ifdef TP_DEBUG_ON
    tp_debug_create_entry();
#endif

    /* initialize the work queue */
    INIT_WORK(&msg26xx_wk, touch_driver_do_work);

#ifdef CONFIG_ENABLE_ITO_MP_TEST
    ctp_self_test_wq = create_singlethread_workqueue("ctp_self_test");
    INIT_WORK(&ctp_self_test_wk, ctp_self_test_do_work);
#endif //CONFIG_ENABLE_ITO_MP_TEST

	if (gpio_is_valid(MS_TS_MSG26XX_GPIO_INT))
	{
	    ret = gpio_request(MS_TS_MSG26XX_GPIO_INT, "interrupt");
	    if (ret < 0)
	    {
	        pr_err("*** Failed to request GPIO %d, error %d ***\n", MS_TS_MSG26XX_GPIO_INT, ret);
	        goto Err_gpio_request_int;
	    }
		
	    ret = gpio_direction_input(MS_TS_MSG26XX_GPIO_INT);
		if (ret)
		{
			dev_err(&client->dev, "set_direction for irq gpio failed\n");
			goto Err_irq_direction_input;
		}
	}
	
    irq_msg26xx = gpio_to_irq(MS_TS_MSG26XX_GPIO_INT);

    /* request an irq and register the isr */
    ret = request_irq(irq_msg26xx/*MS_TS_MSG26XX_GPIO_INT*/, touch_driver_isr,
                      IRQF_TRIGGER_RISING /* | IRQF_TRIGGER_FALLING */,
                      "msg26xx", NULL);
    if (ret != 0)
    {
        pr_err("*** Unable to claim irq %d; error %d ***\n", MS_TS_MSG26XX_GPIO_INT, ret);
        goto err3;
    }
	get_customer_firmware_version();
	
#ifdef SUPPORT_READ_TP_VERSION
	init_tp_fm_info(fw_version_minor, fw_version, "MSG26XX");
#endif
	
#ifdef LCT_UPGRADE_FIRMWARE
	lct_ctp_upgrade_int(ctp_upgrade_func, ctp_upgrade_read_ver_func);
#endif

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    /* create a kset with the name of "kset_example" which is located under /sys/kernel/ */
    example_kset = kset_create_and_add("kset_example", NULL, kernel_kobj);
    if (!example_kset)
    {
        pr_err("*** kset_create_and_add() failed, ret = %d ***\n", ret);

        ret = -ENOMEM;
        goto err3;
    }

    example_kobj = kobject_create();
    if (!example_kobj)
    {
        pr_err("*** kobject_create() failed, ret = %d ***\n", ret);

        ret = -ENOMEM;
		    kset_unregister(example_kset);
		    example_kset = NULL;
        goto err3;
    }

    example_kobj->kset = example_kset;

    ret = kobject_add(example_kobj, NULL, "%s", "kobject_example");
    if (ret != 0)
    {
        pr_err("*** kobject_add() failed, ret = %d ***\n", ret);

		    kobject_put(example_kobj);
		    example_kobj = NULL;
		    kset_unregister(example_kset);
		    example_kset = NULL;
        goto err3;
    }

    /* create the files associated with this kobject */
    ret = sysfs_create_group(example_kobj, &attr_group);
    if (ret != 0)
    {
        pr_err("*** sysfs_create_file() failed, ret = %d ***\n", ret);

        kobject_put(example_kobj);
		    example_kobj = NULL;
		    kset_unregister(example_kset);
		    example_kset = NULL;
        goto err3;
    }

    devpath = kobject_get_path(example_kobj, GFP_KERNEL);
    DBG("DEVPATH = %s\n", devpath);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
   // get firmware mode for parsing packet judgement.
    firmware_mode = get_firmware_mode();
    get_firmware_info(); // get_firmware_info() must be called first before get_debug_mode_packet_length()
    get_debug_mode_packet_length();
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
	ctp_module_name = kzalloc(64*sizeof(char), GFP_KERNEL);
	if (!ctp_module_name)
	{
		dev_err(&client->dev, "Not enough memory\n");
		goto Err_kzalloc_name;
	}
    check_firmware_update_by_SW_ID();
#endif //CONFIG_UPDATE_FIRMWARE_BY_SW_ID

#if defined(CONFIG_FB)
    msg26xx_fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&msg26xx_fb_notif);
    if (ret)
    {
        printk("Unable to register fb_notifier: %d\n", ret);
    }
#elif defined (CONFIG_HAS_EARLYSUSPEND)
    mstar_ts_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
    mstar_ts_early_suspend.suspend = touch_driver_early_suspend;
    mstar_ts_early_suspend.resume = touch_driver_early_resume;
    register_early_suspend(&mstar_ts_early_suspend);
#endif

    DBG("*** mstar touch screen registered ***\n");

	is_tp_driver_loaded = 1;
    return 0;

#ifdef CONFIG_UPDATE_FIRMWARE_BY_SW_ID
Err_kzalloc_name:
#endif
err3:
//    free_irq(MS_TS_MSG26XX_GPIO_INT, input_dev);
    free_irq(irq_msg26xx, input_dev);

Err_irq_direction_input:
	if (gpio_is_valid(MS_TS_MSG26XX_GPIO_INT))
		gpio_free(MS_TS_MSG26XX_GPIO_INT);

Err_gpio_request_int:
err1:
    mutex_destroy(&msg26xx_mutex);
    input_unregister_device(input_dev);
    input_free_device(input_dev);
    input_dev = NULL;

err0:
Err_reset_direction_output:
	if (gpio_is_valid(MS_TS_MSG26XX_GPIO_RST))
		gpio_free(MS_TS_MSG26XX_GPIO_RST);

Err_gpio_request_reset:
	touch_driver_power_on(client, false);

Err_power_on:
	touch_driver_power_init(client, false);

Err_power_init:
	if (tp_platform_data->ts_pinctrl)
	{
		ret = touch_driver_pinctrl_select(client, false);
		if (ret < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}

Err_pinctrl_init:
	if (tp_platform_data->ts_pinctrl)
		pinctrl_put(tp_platform_data->ts_pinctrl);
	i2c_set_clientdata(client, NULL);
	
Err_parse_dt:
	kfree(tp_platform_data);
	printk("%s err, ret=%d\n", __func__, ret);
    return ret;
}

/* remove function is triggered when the input device is removed from input sub-system */
static int __exit touch_driver_remove(struct i2c_client *client)
{
    DBG("touch_driver_remove()\n");

	kfree(ctp_module_name);
//    free_irq(MS_TS_MSG26XX_GPIO_INT, input_dev);
    free_irq(irq_msg26xx, input_dev);
    gpio_free(MS_TS_MSG26XX_GPIO_INT);
    gpio_free(MS_TS_MSG26XX_GPIO_RST);
    input_unregister_device(input_dev);
#ifdef CONFIG_ENABLE_FIRMWARE_DATA_LOG
    kset_unregister(example_kset);
    kobject_put(example_kobj);
#endif //CONFIG_ENABLE_FIRMWARE_DATA_LOG
    mutex_destroy(&msg26xx_mutex);

    return 0;
}

/* The I2C device list is used for matching I2C device and I2C device driver. */
static const struct i2c_device_id touch_device_id[] =
{
    {"msg2633", 0},
    {}, /* should not omitted */
};

static struct of_device_id msg26xx_match_table[] = {
	{ .compatible = "mstar,msg2633", },
	{ },
};


MODULE_DEVICE_TABLE(i2c, touch_device_id);

static struct i2c_driver touch_device_driver =
{
    .driver = {
        .name = "msg2633",
        .owner = THIS_MODULE,
        .of_match_table = msg26xx_match_table,
    },
    .probe = touch_driver_probe,
    .remove = __exit_p(touch_driver_remove),
    .id_table = touch_device_id,
};

static int __init touch_driver_init(void)
{
    int ret;

    /* register driver */
    ret = i2c_add_driver(&touch_device_driver);
    if (ret < 0)
    {
        DBG("add touch_device_driver i2c driver failed.\n");
        return -ENODEV;
    }
    DBG("add touch_device_driver i2c driver.\n");

    return ret;
}

static void __exit touch_driver_exit(void)
{
    DBG("remove touch_device_driver i2c driver.\n");

    i2c_del_driver(&touch_device_driver);
}

#ifdef TP_PRINT
#include <linux/proc_fs.h>

#define TP_PRINT_AUTHORITY 0777
#define PROC_TP_PRINT_MSG   "msgtpp"
#define PROC_TP_PRINT_NODE  "tpp"

static struct proc_dir_entry *msg_tpp = NULL;
static struct proc_dir_entry *tpp = NULL;
static u16 InfoAddr = 0x0F, PoolAddr = 0x10, TransLen = 256;
static u16 cnt, head, tail;
static u8 row, units, update;

static int tp_print_proc_read(struct file *file, char __user *buffer, size_t len, loff_t *ppos)
{
    u16 i, j;
    u16 left, offset;
    u8 dbbus_tx_data[3] = {0};
    u8 dbbus_rx_data[8] = {0};
    u8 u8Data;
    s16 s16Data;
    s32 s32Data;
    char *buf = NULL;
    int rc = 0;

    if ((InfoAddr != 0x0F) && (PoolAddr != 0x10))
    {
        if (update)
        {
            dbbus_tx_data[0] = 0x53;
            dbbus_tx_data[1] = (InfoAddr >> 8) & 0xFF;
            dbbus_tx_data[2] = InfoAddr & 0xFF;
            mutex_lock(&msg26xx_mutex);
            write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
            read_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 8);
            mutex_unlock(&msg26xx_mutex);

            units = dbbus_rx_data[0];
            row = dbbus_rx_data[1];
            cnt = dbbus_rx_data[2];
            head = dbbus_rx_data[3];
            tail = dbbus_rx_data[4];
            TransLen = (dbbus_rx_data[7]<<8) + dbbus_rx_data[6];
            printk("tpp: row=%d, units=%d\n", row, units);
            printk("tpp: cnt=%d, head=%d, tail=%d\n", cnt, head, tail);
        }
        else
        {
            printk("tpp: \n");
        }

        offset = 0;
        left = cnt*row*units;
        buf = kmalloc(left, GFP_KERNEL);
        if (buf != NULL)
        {
            while (left > 0)
            {
                dbbus_tx_data[0] = 0x53;
                dbbus_tx_data[1] = ((PoolAddr + offset) >> 8) & 0xFF;
                dbbus_tx_data[2] = (PoolAddr + offset) & 0xFF;
                mutex_lock(&msg26xx_mutex);
                write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
                read_i2c_seq(SLAVE_I2C_ID_DWI2C, &buf[offset], left > TransLen ? TransLen : left);
                mutex_unlock(&msg26xx_mutex);

                if (left > TransLen)
                {
                    left -= TransLen;
                    offset += TransLen;
                }
                else
                {
                    left = 0;
                }
            }

            i = head;
            while ((cnt*row*units > 0) && (rc >= 0))
            {
                if (i < cnt)
                {
                    if (i == tail)
                    {
                        rc = -1;
                    }
                }
                else
                {
                    if (tail >= cnt)
                    {
                        rc = -1;
                    }
                    else
                    {
                        i = 0;
                    }
                }

                if (rc >= 0)
                {
                    printk("tpp: ");
                    for (j = 0; j < row; j++)
                    {
                        if (units == 1)
                        {
                            u8Data = buf[i*row*units + j*units];
                            printk("%d\t", u8Data);
                        }
                        else if (units == 2)
                        {
                            s16Data = buf[i*row*units + j*units] + (buf[i*row*units + j*units + 1] << 8);
                            printk("%d\t", s16Data);
                        }
                        else if (units == 4)
                        {
                            s32Data = buf[i*row*units + j*units] + (buf[i*row*units + j*units + 1] << 8) + (buf[i*row*units + j*units + 2] << 16) + (buf[i*row*units + j*units + 3] << 24);
                            printk("%d\t", s32Data);
                        }
                    }
                    printk("\n");
                    i++;
                }
            }

            kfree(buf);
        }
    }

    return 0;
}

static const struct file_operations tp_print_proc_fops = {
	.read		= tp_print_proc_read,
};

static void tp_print_create_entry(void)
{
    u8 dbbus_tx_data[3] = {0};
    u8 dbbus_rx_data[8] = {0};

    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
    dbbus_tx_data[2] = 0x56;
    mutex_lock(&msg26xx_mutex);
    write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
    read_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 4);
    mutex_unlock(&msg26xx_mutex);
    InfoAddr = (dbbus_rx_data[1]<<8) + dbbus_rx_data[0];
    PoolAddr = (dbbus_rx_data[3]<<8) + dbbus_rx_data[2];
    printk("InfoAddr=0x%X\n", InfoAddr);
    printk("PoolAddr=0x%X\n", PoolAddr);

    if ((InfoAddr != 0x0F) && (PoolAddr != 0x10))
    {
		msg_tpp = proc_mkdir(PROC_TP_PRINT_MSG, NULL);
		tpp = proc_create_data(PROC_TP_PRINT_NODE, TP_PRINT_AUTHORITY, msg_tpp, &tp_print_proc_fops, NULL);
		if (IS_ERR_OR_NULL(tpp))
		{
			pr_err("tp_print_create_entry failed\n");
		}

        msleep(10);
        dbbus_tx_data[0] = 0x53;
        dbbus_tx_data[1] = (InfoAddr >> 8) & 0xFF;
        dbbus_tx_data[2] = InfoAddr & 0xFF;
        mutex_lock(&msg26xx_mutex);
        write_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_tx_data[0], 3);
        read_i2c_seq(SLAVE_I2C_ID_DWI2C, &dbbus_rx_data[0], 8);
        mutex_unlock(&msg26xx_mutex);

        units = dbbus_rx_data[0];
        row = dbbus_rx_data[1];
        cnt = dbbus_rx_data[2];
        head = dbbus_rx_data[3];
        tail = dbbus_rx_data[4];
        update = dbbus_rx_data[5];
        TransLen = (dbbus_rx_data[7]<<8) + dbbus_rx_data[6];
        printk("tpp: row=%d, units=%d\n", row, units);
        printk("tpp: cnt=%d, head=%d, tail=%d\n", cnt, head, tail);
        printk("tpp: update=%d, TransLen=%d\n", update, TransLen);
    }
}
#endif

#ifdef TP_DEBUG_ON
#include <linux/proc_fs.h>

#define DEBUG_AUTHORITY 0777
#define PROC_TP_DEBUG      "tp_debug"
#define PROC_DEBUG_ON      "debug_on"

static struct proc_dir_entry *tp_debug = NULL;
static struct proc_dir_entry *debug_on = NULL;

static int tp_debug_proc_write(struct file *file, const char __user *buffer, size_t len, loff_t *ppos)
{
    char *buf;

    if (len < 1)
    return -EINVAL;

    buf = kmalloc(len, GFP_KERNEL);
    if (!buf)
    return -ENOMEM;

    if (copy_from_user(buf, buffer, len))
    {
        kfree(buf);
        return -EFAULT;
    }

    if (buf[0] >= '0' && buf[0] <= '9')
    {
        tp_debug_on = buf[0] - '0';
    }
    else
    {
        kfree(buf);
        return -EINVAL;
    }

    kfree(buf);
    return len;
}

static int tp_debug_proc_read(struct file *file, char __user *buffer, size_t len, loff_t *ppos)
{
    printk("tp_debug_on=%d\n",tp_debug_on);
    return sprintf(buffer, "%d\n", tp_debug_on);
}

static const struct file_operations tp_debug_proc_fops = {
	.read		= tp_debug_proc_read,
	.write		= tp_debug_proc_write,
};

static void tp_debug_create_entry(void)
{
    tp_debug = proc_mkdir(PROC_TP_DEBUG, NULL);
	debug_on = proc_create_data(PROC_DEBUG_ON, DEBUG_AUTHORITY, tp_debug, &tp_debug_proc_fops, NULL);
	if (IS_ERR_OR_NULL(debug_on))
	{
		pr_err("tp_debug_create_entry failed\n");
	}
}
#endif

late_initcall(touch_driver_init);
module_exit(touch_driver_exit);
MODULE_LICENSE("GPL");

