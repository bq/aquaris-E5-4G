#include <linux/regulator/consumer.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/timer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>

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
#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#include <mach/vreg.h>

#include <linux/miscdevice.h>
#include <asm/uaccess.h>

#include <linux/poll.h>
#include <linux/irq.h>
//#include <mach/rpc_pmapp.h>
#include <linux/wakelock.h>
#include <linux/input-polldev.h>

#include <linux/kthread.h>
#include "lct_ctp_selftest.h"

#include "lct_ctp_upgrade.h"
#include <linux/proc_fs.h>          
#include "lct_tp_fm_info.h"

#define SWAP_X_Y	(1)
#define REVERSE_X	(1)
//#define REVERSE_Y 	(1)
//#define MSTAR_USE_VIRTUALKEY
//int tp_resume_save_time = 0;
#ifdef CONFIG_LCT_AE515
extern int  light_lcd_leve;
#endif //for lcd boyi backlight

#define TP_PRINT		// fae @ 20140711		For tools debug.
//#define TP_DIFF
#define LOG_TAG "msg2133"
//#define DEBUG
#ifdef DEBUG
#define TP_DEBUG(format, args...)   printk(KERN_INFO "TP_:%s( )_%d_: " format, \
	__FUNCTION__ , __LINE__, ## args);
#define DBG() printk("[%s]:%d => \n",__FUNCTION__,__LINE__);
#else
#define TP_DEBUG(format, args...);
#define DBG()
#define TPD_DMESG(a,arg...) printk(LOG_TAG ": " a,##arg)

#endif

#define __MSG2133A_FIRMWARE_UPDATE__        1

#define __FIRMWARE_AUTO_UPDATE__ 1
#define __FIRMWARE_FORCE_UPDATE__	0

#define LCT_UPGRADE_MSG2133

// xuke @ 20140922	Correct the sys authority for CTS test.
#if defined(CONFIG_L6140_COMMON) || defined(CONFIG_L6300_COMMON)
#define SYSFS_AUTHORITY_CHANGE_FOR_CTS_TEST
#endif

#ifdef SYSFS_AUTHORITY_CHANGE_FOR_CTS_TEST
#define SYSFS_AUTHORITY (S_IRUGO|S_IWUSR|S_IWGRP)
#else
#define SYSFS_AUTHORITY (S_IRUGO|S_IWUGO)
#endif

//#define MSG_GESTURE_FUNCTION
#define MSG_SOFT_VERSION_ID
#define MSG_ITO_TEST

#define TP_OF_YUSHUN		(0x01)		// 宇顺
#define TP_OF_BOYI		(0x02)			// 博一
#define TP_OF_YEJI		(0x03)			// 业继

//#define CYTTSP_SUPPORT_READ_TP_VERSION

#define VTG_MIN_UV		2850000
#define VTG_MAX_UV		2850000
#define I2C_VTG_MIN_UV	1800000
#define I2C_VTG_MAX_UV	1800000

#ifdef TP_PRINT
struct mutex tpp_lock;
static void tp_print_create_entry(void);
#endif

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
#include <linux/poll.h>
#include <linux/wakelock.h>
#include <linux/input-polldev.h>
//#define DEBUG_TP_SENSOR
#define PROXIMITY_INPUT_DEV_NAME 	"proximity"
#define SENSOR_PROX_TP_USE_WAKELOCK
static struct i2c_client *i2c_prox_client=NULL;
static DEFINE_MUTEX(tp_prox_sensor_mutex);
static DECLARE_WAIT_QUEUE_HEAD(tp_sensor_waitqueue);

#ifdef SENSOR_PROX_TP_USE_WAKELOCK
static	struct wake_lock sensor_prox_tp_wake_lock;
#endif


static int tp_prox_sensor_opened;
static char tp_prox_sensor_data = 1;  // 0 near 1 far 
static char tp_pre_sensor_data = 1; 
static int tp_prox_sensor_data_changed = 0;
static void tp_prox_sensor_enable(int enable);


static int is_need_report_pointer = 1;

#endif//CONFIG_TOUCHPANEL_PROXIMITY_SENSOR

void lct_qup_i2c_use_custom_clk(struct i2c_adapter *adap, u32 iicRate);
#if defined(MSG_GESTURE_FUNCTION)
extern struct device_attribute dev_attr_doubleclick;
extern struct device_attribute dev_attr_updirect;
extern struct device_attribute dev_attr_downdirect;
extern struct device_attribute dev_attr_leftdirect;
extern struct device_attribute dev_attr_rightdirect;

extern int msg_GetGestureFlag(void);
extern void msg_SetGestureFlag(int val);
extern void msg_SetDoubleClickModeFlage(int val);
extern void msg_SetUpDirectModeFlage(int val);
extern void msg_SetDownDirectModeFlage(int val);
extern void msg_SetLeftDirectModeFlage(int val);
extern void msg_SetRightDirectModeFlage(int val);
extern int msg_CloseGestureFunction( void );
extern u8 msg_GetGestureModeValue( void );
extern int msg_OpenGestureFunction( int g_Mode );
#endif

static int is_suspend = 0;

#define u8 unsigned char
#define U8 unsigned char
#define u32 unsigned int
#define U32 unsigned int
#define s32 signed int
#define U16 ushort
#define REPORT_PACKET_LENGTH  8
#define MSG21XX_INT_GPIO  13
#define MSG21XX_RESET_GPIO 12     //高电平时候正常工作
#define FLAG_GPIO    122
#define ST_KEY_SEARCH 	217
#define MSG21XX_RETRY_COUNT 2
#define MS_TS_MSG20XX_X_MIN   0
#define MS_TS_MSG20XX_Y_MIN   0
#define MS_TS_MSG21XX_X_MAX   540
#define MS_TS_MSG21XX_Y_MAX   960
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000
#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4
static int msg21xx_irq = 0;
static struct i2c_client *msg21xx_i2c_client;
static struct work_struct msg21xx_wq;
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
struct input_dev  *input_prox_dev;
struct input_polled_dev *input_poll_dev;
#endif
#if defined(CONFIG_FB)
static	struct notifier_block fb_notif;
#endif
//static struct early_suspend early_suspend;
static struct mutex msg21xx_mutex;
//static struct input_dev *msg21xx_input = NULL;
struct msg21xx_ts_data *msg2133_ts = NULL;
struct msg21xx_platform_data *pdata = NULL;
//#ifndef MSTAR_USE_VIRTUALKEY
#if 0
static struct input_dev *simulate_key_input = NULL;
#endif
static 	struct input_dev *input=NULL;
static  int after0 = 0;//filter the continous nFingerNum==0 


const U16 tpd_key_array[] = { KEY_MENU, KEY_HOMEPAGE, KEY_BACK, KEY_SEARCH };
#define MAX_KEY_NUM ( sizeof( tpd_key_array )/sizeof( tpd_key_array[0] ) )


#if __MSG2133A_FIRMWARE_UPDATE__
static struct task_struct		*update_firmware_tsk;
static bool fw_in_updating_flag;		// xuke @ 20140708		Can not enable irq during updating fw.
#endif
#if __MSG2133A_FIRMWARE_UPDATE__
//unsigned char *temp=NULL;
static unsigned char FW_YuShun[]=
{
#include "MSG213X_CM610_YuShun_Vid01.i"
};

static unsigned char FW_BoYi[]=
{
#include "MSG213X_CM610_BoYi_Vid02.i"
};

static unsigned char FW_YeJi[]=
{
#include "MSG213X_L6140_YeJi_Vid03.i"
};

//funayuangchuang
static unsigned char FW_FuNa[]=
{
123,123
};

#endif
//lct.songzhou add ctp module info

typedef struct {
int id;
char * module_name;
}tp_info_t;

tp_info_t  tp_info[] = {
{0x04,"Funa"},
{0x01,"CS"},
};


//static u8 temp_key_value;

#ifdef MSTAR_USE_VIRTUALKEY
int virtualkey_x;
int virtualkey_y;
#endif

//static      struct timer_list my_timer;

static  u16 mstar_module_name = 0;
static  u16 mstar_firmware_version = 0;

static int is_msg2133A = 1;



#define TOUCH_ADDR_MSG21XX	0x26
#define FW_ADDR_MSG20XX   		0x62

#if __MSG2133A_FIRMWARE_UPDATE__
static int is_msg2133A_funa = 0;
#endif


struct device *firmware_cmd_dev;


struct msg21xx_platform_data {
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
};

struct msg21xx_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
    const struct msg21xx_platform_data *pdata;
    struct regulator *vdd;
	struct regulator *vcc_i2c;
	int use_irq;
	struct hrtimer timer;
	struct work_struct  work;
	int (*power)(int on);
	int (*get_int_status)(void);
	void (*reset_ic)(void);
	
    struct pinctrl *ts_pinctrl;
	struct pinctrl_state *gpio_state_active;
	struct pinctrl_state *gpio_state_suspend;
};
//static struct msg21xx_ts_data *msg21xx_ts_gpts = NULL;

#define MAX_TOUCH_FINGER 2
typedef struct
{
    u16 X;
    u16 Y;
} TouchPoint_t;

typedef struct
{
    u8 nTouchKeyMode;
    u8 nTouchKeyCode;
    u8 nFingerNum;
    TouchPoint_t Point[MAX_TOUCH_FINGER];
} TouchScreenInfo_t;


#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#endif



#if !__MSG2133A_FIRMWARE_UPDATE__
void HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
    //according to your platform.
   	int rc;
 
	struct i2c_msg msgs[] = {
		{
			.addr = addr,
			.flags = 0,
			.len = size,
			.buf = data,
		},
	};
	rc = i2c_transfer(msg21xx_i2c_client->adapter, msgs, 1);
	if(rc < 0){
		printk("HalTscrCDevWriteI2CSeq error %d\n", rc);
	}
}
#endif
#if 0
static int _msg_WriteI2CSeqReturn(U8 addr, U8* data, U16 size)
{
    int rc;
    struct i2c_msg msgs[] =
    {
        {
            .addr = addr,
            .flags = 0,
            .len = size,
            .buf = data,
        },
    };

    rc = i2c_transfer(msg21xx_i2c_client->adapter, msgs, 1);
    if( rc < 0 )
    {
        printk("_msg_WriteI2CSeq error %d,addr = %d\n", rc, addr);
    }
	return rc;

}
#endif
#if __MSG2133A_FIRMWARE_UPDATE__
#define FW_ADDR_MSG21XX   (0xC4>>1)
#define FW_ADDR_MSG21XX_TP   (0x4C>>1)
#define FW_UPDATE_ADDR_MSG21XX   (0x92>>1)
//#define TP_DEBUG	printk//(x)		//x
#define DBUG	printk//(x) //x
static  char *fw_version;
static u8 temp[94][1024];
 u8  Fmr_Loader[1024];
     u32 crc_tab[256];

static int FwDataCnt;
struct class *firmware_class;
struct device *firmware_cmd_dev;

static u8 g_dwiic_info_data[1024];   // Buffer for info data



static int HalTscrCReadI2CSeq(u8 addr, u8* read_data, u16 size)
{
   //according to your platform.
   	int rc;

	struct i2c_msg msgs[] =
    {
		{
			.addr = addr,
			.flags = I2C_M_RD,
			.len = size,
			.buf = read_data,
		},
	};

	rc = i2c_transfer(msg21xx_i2c_client->adapter, msgs, 1);
	if( rc < 0 )
    {
//		printk("HalTscrCReadI2CSeq error %d\n", rc);
	}
	
	return rc;
}

static int HalTscrCDevWriteI2CSeq(u8 addr, u8* data, u16 size)
{
    //according to your platform.
   	int rc;
	struct i2c_msg msgs[] =
    {
		{
			.addr = addr,
			.flags = 0,
			.len = size,
			.buf = data,
		},
	};
	rc = i2c_transfer(msg21xx_i2c_client->adapter, msgs, 1);
	if( rc < 0 )
    {
//		printk("HalTscrCDevWriteI2CSeq error %d,addr = %d\n", rc,addr);
	}
	
	return rc;
}
/*
static void Get_Chip_Version(void)
{
    printk("[%s]: Enter!\n", __func__);
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[2];

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCE;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
    if (dbbus_rx_data[1] == 0)
    {
        // it is Catch2
        TP_DEBUG(printk("*** Catch2 ***\n");)
        //FwVersion  = 2;// 2 means Catch2
    }
    else
    {
        // it is catch1
        TP_DEBUG(printk("*** Catch1 ***\n");)
        //FwVersion  = 1;// 1 means Catch1
    }

}
*/

static int dbbusDWIICEnterSerialDebugMode(void)
{
    u8 data[5];

    // Enter the Serial Debug Mode
    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
	return HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 5);
}

static void dbbusDWIICStopMCU(void)
{
    u8 data[1];

    // Stop the MCU
    data[0] = 0x37;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICUseBus(void)
{
    u8 data[1];

    // IIC Use Bus
    data[0] = 0x35;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICReshape(void)
{
    u8 data[1];

    // IIC Re-shape
    data[0] = 0x71;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICIICNotUseBus(void)
{
    u8 data[1];

    // IIC Not Use Bus
    data[0] = 0x34;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICNotStopMCU(void)
{
    u8 data[1];

    // Not Stop the MCU
    data[0] = 0x36;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);
}

static void dbbusDWIICExitSerialDebugMode(void)
{
    u8 data[1];

    // Exit the Serial Debug Mode
    data[0] = 0x45;
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, data, 1);

    // Delay some interval to guard the next transaction
    udelay ( 150);//200 );        // delay about 0.2ms
}

static void drvISP_EntryIspMode(void)
{
    u8 bWriteData[5] =
    {
        0x4D, 0x53, 0x54, 0x41, 0x52
    };
	printk("\n******%s come in*******\n",__FUNCTION__);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);
    udelay ( 150 );//200 );        // delay about 0.1ms
}

static u8 drvISP_Read(u8 n, u8* pDataToRead)    //First it needs send 0x11 to notify we want to get flash data back.
{
    u8 Read_cmd = 0x11;
    unsigned char dbbus_rx_data[2] = {0};
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &Read_cmd, 1);
    //msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay( 800 );//200);
    if (n == 1)
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
        *pDataToRead = dbbus_rx_data[0];
        TP_DEBUG("dbbus=%d,%d===drvISP_Read=====\n",dbbus_rx_data[0],dbbus_rx_data[1]);
  	}
    else
    {
        HalTscrCReadI2CSeq(FW_UPDATE_ADDR_MSG21XX, pDataToRead, n);
    }

    return 0;
}

static void drvISP_WriteEnable(void)
{
    u8 bWriteData[2] =
    {
        0x10, 0x06
    };
    u8 bWriteData1 = 0x12;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    udelay(150);//1.16
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
}


static void drvISP_ExitIspMode(void)
{
    u8 bWriteData = 0x24;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 1);
    udelay( 150 );//200);
}

static u8 drvISP_ReadStatus(void)
{
    u8 bReadData = 0;
    u8 bWriteData[2] =
    {
        0x10, 0x05
    };
    u8 bWriteData1 = 0x12;

    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    //msctpc_LoopDelay ( 1 );        // delay about 100us*****
    udelay(150);//200);
    drvISP_Read(1, &bReadData);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    return bReadData;
}

#if 0
static void drvISP_BlockErase(u32 addr)
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
	u32 timeOutCount=0;
	printk("\n******%s come in*******\n",__FUNCTION__);
	
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 3);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
	//msctpc_LoopDelay ( 1 );        // delay about 100us*****
	udelay(150);//200);
    timeOutCount=0;
	while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
	{
		timeOutCount++;
		if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
	}
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;//0xD8;        //Block Erase
    //bWriteData[2] = ((addr >> 16) & 0xFF) ;
    //bWriteData[3] = ((addr >> 8) & 0xFF) ;
    //bWriteData[4] = (addr & 0xFF) ;
	HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 2);
    //HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData, 5);
    HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
		//msctpc_LoopDelay ( 1 );        // delay about 100us*****
	udelay(150);//200);
	timeOutCount=0;
	while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
	{
		timeOutCount++;
		if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
	}
}
#endif
static void drvISP_Program(u16 k, u8* pDataToWrite)
{
    u16 i = 0;
    u16 j = 0;
    //u16 n = 0;
    u8 TX_data[133];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
		u32 timeOutCount=0;
    for (j = 0; j < 8; j++)   //128*8 cycle
    {
        TX_data[0] = 0x10;
        TX_data[1] = 0x02;// Page Program CMD
        TX_data[2] = (addr + 128 * j) >> 16;
        TX_data[3] = (addr + 128 * j) >> 8;
        TX_data[4] = (addr + 128 * j);
        for (i = 0; i < 128; i++)
        {
            TX_data[5 + i] = pDataToWrite[j * 128 + i];
        }
        //msctpc_LoopDelay ( 1 );        // delay about 100us*****
        udelay(150);//200);
       
        timeOutCount=0;
		while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
		{
			timeOutCount++;
			if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
		}
  
        drvISP_WriteEnable();
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, TX_data, 133);   //write 133 byte per cycle
        HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);
    }
}

static ssize_t firmware_update_show ( struct device *dev,
                                      struct device_attribute *attr, char *buf )
{
    return sprintf ( buf, "%s\n", fw_version );
}
/*reset the chip*/
static void _HalTscrHWReset(void)
{
	gpio_direction_output(pdata->reset_gpio, 1);
	gpio_set_value(pdata->reset_gpio, 1);
	gpio_set_value(pdata->reset_gpio, 0);
	
	mdelay(10);  /* Note that the RST must be in LOW 10ms at least */
	gpio_set_value(pdata->reset_gpio, 1);
	/* Enable the interrupt service thread/routine for INT after 50ms */
	mdelay(50);
}


static void drvISP_Verify ( u16 k, u8* pDataToVerify )
{
    u16 i = 0, j = 0;
    u8 bWriteData[5] ={ 0x10, 0x03, 0, 0, 0 };
    u8 RX_data[256];
    u8 bWriteData1 = 0x12;
    u32 addr = k * 1024;
    u8 index = 0;
    u32 timeOutCount;
    for ( j = 0; j < 8; j++ ) //128*8 cycle
    {
        bWriteData[2] = ( u8 ) ( ( addr + j * 128 ) >> 16 );
        bWriteData[3] = ( u8 ) ( ( addr + j * 128 ) >> 8 );
        bWriteData[4] = ( u8 ) ( addr + j * 128 );
        udelay ( 100 );        // delay about 100us*****

        timeOutCount = 0;
        while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
        {
            timeOutCount++;
            if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
        }

        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 5 ); //write read flash addr
        udelay ( 100 );        // delay about 100us*****
        drvISP_Read ( 128, RX_data );
        HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 ); //cmd end
        for ( i = 0; i < 128; i++ ) //log out if verify error
        {
            if ( ( RX_data[i] != 0 ) && index < 10 )
            {
                //TP_DEBUG("j=%d,RX_data[%d]=0x%x\n",j,i,RX_data[i]);
                index++;
            }
            if ( RX_data[i] != pDataToVerify[128 * j + i] )
            {
                TP_DEBUG ( "k=%d,j=%d,i=%d===============Update Firmware Error================", k, j, i );
            }
        }
    }
}

static void drvISP_ChipErase(void)
{
    u8 bWriteData[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
    u8 bWriteData1 = 0x12;
    u32 timeOutCount = 0;
    drvISP_WriteEnable();

    //Enable write status register
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x50;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write Status
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x01;
    bWriteData[2] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 3 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );

    //Write disable
    bWriteData[0] = 0x10;
    bWriteData[1] = 0x04;
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    udelay ( 100 );        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 100000 ) break; /* around 1 sec timeout */
    }
    drvISP_WriteEnable();

    bWriteData[0] = 0x10;
    bWriteData[1] = 0xC7;

    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, bWriteData, 2 );
    HalTscrCDevWriteI2CSeq ( FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1 );
    udelay ( 100 );        // delay about 100us*****
    timeOutCount = 0;
    while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
    {
        timeOutCount++;
        if ( timeOutCount >= 500000 ) break; /* around 5 sec timeout */
    }
}

/* update the firmware part, used by apk*/
/*show the fw version*/

static ssize_t firmware_update_c2 ( struct device *dev,
                                    struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay ( 300 );

    // Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    //Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    TP_DEBUG ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    return size;
}

static u32 Reflect ( u32 ref, char ch ) //unsigned int Reflect(unsigned int ref, char ch)
{
    u32 value = 0;
    u32 i = 0;

    for ( i = 1; i < ( ch + 1 ); i++ )
    {
        if ( ref & 1 )
        {
            value |= 1 << ( ch - i );
        }
        ref >>= 1;
    }
    return value;
}

u32 Get_CRC ( u32 text, u32 prevCRC, u32 *crc32_table )
{
    u32  ulCRC = prevCRC;
	ulCRC = ( ulCRC >> 8 ) ^ crc32_table[ ( ulCRC & 0xFF ) ^ text];
    return ulCRC ;
}
static void Init_CRC32_Table ( u32 *crc32_table )
{
    u32 magicnumber = 0x04c11db7;
    u32 i = 0, j;

    for ( i = 0; i <= 0xFF; i++ )
    {
        crc32_table[i] = Reflect ( i, 8 ) << 24;
        for ( j = 0; j < 8; j++ )
        {
            crc32_table[i] = ( crc32_table[i] << 1 ) ^ ( crc32_table[i] & ( 0x80000000L ) ? magicnumber : 0 );
        }
        crc32_table[i] = Reflect ( crc32_table[i], 32 );
    }
}

typedef enum
{
	EMEM_ALL = 0,
	EMEM_MAIN,
	EMEM_INFO,
} EMEM_TYPE_t;

static void drvDB_WriteReg8Bit ( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 4 );
}

static void drvDB_WriteReg ( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 5 );
}

static unsigned short drvDB_ReadReg ( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}

#ifdef LCT_UPGRADE_MSG2133
static unsigned char update_complete_flag = 1;
#endif
static int drvTP_erase_emem_c32 ( void )
{
    /////////////////////////
    //Erase  all
    /////////////////////////
    
    //enter gpio mode
    drvDB_WriteReg ( 0x16, 0x1E, 0xBEAF );

    // before gpio mode, set the control pin as the orginal status
    drvDB_WriteReg ( 0x16, 0x08, 0x0000 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptrim = 1, h'04[2]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x04 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    // ptm = 6, h'04[12:14] = b'110
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x60 );
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    // pmasi = 1, h'04[6]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0x44 );
    // pce = 1, h'04[11]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x68 );
    // perase = 1, h'04[7]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xC4 );
    // pnvstr = 1, h'04[5]
    drvDB_WriteReg8Bit ( 0x16, 0x08, 0xE4 );
    // pwe = 1, h'04[9]
    drvDB_WriteReg8Bit ( 0x16, 0x09, 0x6A );
    // trigger gpio load
    drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x10 );

    return ( 1 );
}

static ssize_t firmware_update_c32 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size,  EMEM_TYPE_t emem_type )
{
//    u8  dbbus_tx_data[4];
    //u8  dbbus_rx_data[2] = {0};
      // Buffer for slave's firmware

    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
    u16 reg_data = 0;
	unsigned long timeout = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;

#if 1
    /////////////////////////
    // Erase  all
    /////////////////////////
    drvTP_erase_emem_c32();
    mdelay ( 1000 ); //MCR_CLBK_DEBUG_DELAY ( 1000, MCU_LOOP_DELAY_COUNT_MS );

    //ResetSlave();
    _HalTscrHWReset();
    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Reset Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    timeout = jiffies + HZ/2;		// xuke @ 20140711	Add timeout mechanism.
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while (( reg_data != 0x1C70 ) && (time_before(jiffies, timeout)));


    drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks

    //polling 0x3CE4 is 0x2F43
    timeout = jiffies + HZ/2;		// xuke @ 20140711	Add timeout mechanism.
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while (( reg_data != 0x2F43 ) && (time_before(jiffies, timeout)));


    //calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  // emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( temp[i][j], crc_info, &crc_tab[0] );
            }
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
#if 0
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );
#else
		for (j=0; j<8; ++j)
		{
			TP_DEBUG("i=%d, j=%d\n", i, j);
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &temp[i][j*128], 128);
		}
#endif

        // polling 0x3CE4 is 0xD0BC
		timeout = jiffies + HZ/2; 	// xuke @ 20140711	Add timeout mechanism.
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while (( reg_data != 0xD0BC ) && (time_before(jiffies, timeout)));

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    //write file done
    drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );

    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );
    // polling 0x3CE4 is 0x9432
	timeout = jiffies + HZ/2; 	// xuke @ 20140711	Add timeout mechanism.
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while (( reg_data != 0x9432 ) && (time_before(jiffies, timeout)));

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    // CRC Main from TP
    crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
    crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );
 
    //CRC Info from TP
    crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
    crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );

    TP_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    if ( ( crc_main_tp != crc_main ) || ( crc_info_tp != crc_info ) )
    {
        printk ( "update FAILED\n" );
	#ifdef LCT_UPGRADE_MSG2133
		update_complete_flag = 0;
	#endif
		_HalTscrHWReset();
        FwDataCnt = 0;
		enable_irq(msg21xx_irq);		
        return ( 0 );
    }

    printk ( "update OK\n" );
	_HalTscrHWReset();
    FwDataCnt = 0;
	enable_irq(msg21xx_irq);

    return size;
#endif
}

static int drvTP_erase_emem_c33 ( EMEM_TYPE_t emem_type )
{
    // stop mcu
    drvDB_WriteReg ( 0x0F, 0xE6, 0x0001 );

    //disable watch dog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    // set PROGRAM password
    drvDB_WriteReg8Bit ( 0x16, 0x1A, 0xBA );
    drvDB_WriteReg8Bit ( 0x16, 0x1B, 0xAB );

    //proto.MstarWriteReg(F1.loopDevice, 0x1618, 0x80);
    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    if ( emem_type == EMEM_ALL )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x08, 0x10 ); //mark
    }

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x40 );
    mdelay ( 10 );

    drvDB_WriteReg8Bit ( 0x16, 0x18, 0x80 );

    // erase trigger
    if ( emem_type == EMEM_MAIN )
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x04 ); //erase main
    }
    else
    {
        drvDB_WriteReg8Bit ( 0x16, 0x0E, 0x08 ); //erase all block
    }

    return ( 1 );
}
#if 0
static int drvTP_read_emem_dbbus_c33 ( EMEM_TYPE_t emem_type, u16 addr, size_t size, u8 *p, size_t set_pce_high )
{
    u32 i;

    // Set the starting address ( must before enabling burst mode and enter riu mode )
    drvDB_WriteReg ( 0x16, 0x00, addr );

    // Enable the burst mode ( must before enter riu mode )
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) | 0x0001 );

    // Set the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0xABBA );

    // Enable the information block if pifren is HIGH
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );

        // Set the PIFREN to be HIGH
        drvDB_WriteReg ( 0x16, 0x08, 0x0010 );
    }

    // Set the PCE to be HIGH
    drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
    mdelay ( 10 );

    // Wait pce becomes 1 ( read data ready )
    while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );

    for ( i = 0; i < size; i += 4 )
    {
        // Fire the FASTREAD command
        drvDB_WriteReg ( 0x16, 0x0E, drvDB_ReadReg ( 0x16, 0x0E ) | 0x0001 );

        // Wait the operation is done
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0001 ) != 0x0001 );

        p[i + 0] = drvDB_ReadReg ( 0x16, 0x04 ) & 0xFF;
        p[i + 1] = ( drvDB_ReadReg ( 0x16, 0x04 ) >> 8 ) & 0xFF;
        p[i + 2] = drvDB_ReadReg ( 0x16, 0x06 ) & 0xFF;
        p[i + 3] = ( drvDB_ReadReg ( 0x16, 0x06 ) >> 8 ) & 0xFF;
    }

    // Disable the burst mode
    drvDB_WriteReg ( 0x16, 0x0C, drvDB_ReadReg ( 0x16, 0x0C ) & ( ~0x0001 ) );

    // Clear the starting address
    drvDB_WriteReg ( 0x16, 0x00, 0x0000 );

    //Always return to main block
    if ( emem_type == EMEM_INFO )
    {
        // Clear the PCE before change block
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0080 );
        mdelay ( 10 );
        // Set the PIFREN to be LOW
        drvDB_WriteReg ( 0x16, 0x08, drvDB_ReadReg ( 0x16, 0x08 ) & ( ~0x0010 ) );

        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    // Clear the RIU password
    drvDB_WriteReg ( 0x16, 0x1A, 0x0000 );

    if ( set_pce_high )
    {
        // Set the PCE to be HIGH before jumping back to e-flash codes
        drvDB_WriteReg ( 0x16, 0x18, drvDB_ReadReg ( 0x16, 0x18 ) | 0x0040 );
        while ( ( drvDB_ReadReg ( 0x16, 0x10 ) & 0x0004 ) != 0x0004 );
    }

    return ( 1 );
}
#endif

#if 0
static int drvTP_read_info_dwiic_c33 ( void )
{
    u8  dwiic_tx_data[5];
//    u8  dwiic_rx_data[4];
    u16 reg_data=0;
    mdelay ( 300 );
	TP_DEBUG ( "drvTP_read_info_dwiic_c33 start" );
    // Stop Watchdog
    drvDB_WriteReg8Bit ( 0x3C, 0x60, 0x55 );
    drvDB_WriteReg8Bit ( 0x3C, 0x61, 0xAA );

    drvDB_WriteReg ( 0x3C, 0xE4, 0xA4AB );

	drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );

    // TP SW reset
    drvDB_WriteReg ( 0x1E, 0x04, 0x829F );
	mdelay ( 1 );
    dwiic_tx_data[0] = 0x10;
    dwiic_tx_data[1] = 0x0F;
    dwiic_tx_data[2] = 0xE6;
    dwiic_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dwiic_tx_data, 4 );	
    mdelay ( 100 );
	TP_DEBUG ( "drvTP_read_info_dwiic_c33 11111" );
    do{
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
		TP_DEBUG ( "drvTP_read_info_dwiic_c33 22222" );
    }
    while ( reg_data != 0x5B58 );
	TP_DEBUG ( "drvTP_read_info_dwiic_c33 33333333" );
    dwiic_tx_data[0] = 0x72;
    dwiic_tx_data[1] = 0x80;
    dwiic_tx_data[2] = 0x00;
    dwiic_tx_data[3] = 0x04;
    dwiic_tx_data[4] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , dwiic_tx_data, 5 );
	TP_DEBUG ( "drvTP_read_info_dwiic_c33 44444" );
    mdelay ( 50 );

    // recive info data
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX_TP, &g_dwiic_info_data[0], 8 );
	TP_DEBUG ( "drvTP_read_info_dwiic_c33 55555" );

    return ( 1 );
}
#endif
#if 0
static int drvTP_info_updata_C33 ( u16 start_index, u8 *data, u16 size )
{
    // size != 0, start_index+size !> 1024
    u16 i;
    for ( i = 0; i < size; i++ )
    {
        g_dwiic_info_data[start_index] = * ( data + i );
        start_index++;
    }
    return ( 1 );
}
#endif
static ssize_t firmware_update_c33 ( struct device *dev, struct device_attribute *attr,
                                     const char *buf, size_t size, EMEM_TYPE_t emem_type )
{
//    u8  dbbus_tx_data[4];
//    u8  dbbus_rx_data[2] = {0};
 //   u8  life_counter[2];
    u32 i, j;
    u32 crc_main, crc_main_tp;
    u32 crc_info, crc_info_tp;
  
    int update_pass = 1;
    u16 reg_data = 0;
	unsigned long timeout = 0;

    crc_main = 0xffffffff;
    crc_info = 0xffffffff;
	TP_DEBUG ( "firmware_update_c33 start" );
	//printk("p_update start");

	lct_qup_i2c_use_custom_clk(msg21xx_i2c_client->adapter, 50000);
	
	#if 0
    drvTP_read_info_dwiic_c33();
	TP_DEBUG ( "firmware_update_c33 end" );
	printk("p_update 1111");
    if ( g_dwiic_info_data[0] == 'M' && g_dwiic_info_data[1] == 'S' && g_dwiic_info_data[2] == 'T' && g_dwiic_info_data[3] == 'A' && g_dwiic_info_data[4] == 'R' && g_dwiic_info_data[5] == 'T' && g_dwiic_info_data[6] == 'P' && g_dwiic_info_data[7] == 'C' )
    {
        // updata FW Version
        //drvTP_info_updata_C33 ( 8, &temp[32][8], 5 );

		g_dwiic_info_data[8]=temp[32][8];
		g_dwiic_info_data[9]=temp[32][9];
		g_dwiic_info_data[10]=temp[32][10];
		g_dwiic_info_data[11]=temp[32][11];
        // updata life counter
        life_counter[1] = (( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) >> 8 ) & 0xFF;
        life_counter[0] = ( ( (g_dwiic_info_data[13] << 8 ) | g_dwiic_info_data[12]) + 1 ) & 0xFF;
		g_dwiic_info_data[12]=life_counter[0];
		g_dwiic_info_data[13]=life_counter[1];
        //drvTP_info_updata_C33 ( 10, &life_counter[0], 3 );
        drvDB_WriteReg ( 0x3C, 0xE4, 0x78C5 );
		drvDB_WriteReg ( 0x1E, 0x04, 0x7d60 );
        // TP SW reset
        drvDB_WriteReg ( 0x1E, 0x04, 0x829F );

        mdelay ( 50 );

        //polling 0x3CE4 is 0x2F43
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );

        }
        while ( reg_data != 0x2F43 );

        // transmit lk info data
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &g_dwiic_info_data[0], 1024 );
		printk("p_update 2222");
        //polling 0x3CE4 is 0xD0BC
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while ( reg_data != 0xD0BC );
		printk("p_update 33333");
    }
	#endif
	

    //erase main
    drvTP_erase_emem_c33 ( EMEM_MAIN );
    mdelay ( 1000 );

    //ResetSlave();
    _HalTscrHWReset();

    //drvDB_EnterDBBUS();
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    /////////////////////////
    // Program
    /////////////////////////

    //polling 0x3CE4 is 0x1C70
    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
		timeout = jiffies + HZ/2; 	// xuke @ 20140711	Add timeout mechanism.
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while (( reg_data != 0x1C70 ) && (time_before(jiffies, timeout)));
    }

    switch ( emem_type )
    {
        case EMEM_ALL:
            drvDB_WriteReg ( 0x3C, 0xE4, 0xE38F );  // for all-blocks
            break;
        case EMEM_MAIN:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for main block
            break;
        case EMEM_INFO:
            drvDB_WriteReg ( 0x3C, 0xE4, 0x7731 );  // for info block

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );

            drvDB_WriteReg8Bit ( 0x3C, 0xE4, 0xC5 ); //
            drvDB_WriteReg8Bit ( 0x3C, 0xE5, 0x78 ); //

            drvDB_WriteReg8Bit ( 0x1E, 0x04, 0x9F );
            drvDB_WriteReg8Bit ( 0x1E, 0x05, 0x82 );

            drvDB_WriteReg8Bit ( 0x0F, 0xE6, 0x00 );
            mdelay ( 100 );
            break;
    }

    // polling 0x3CE4 is 0x2F43
	timeout = jiffies + HZ/2; 	// xuke @ 20140711	Add timeout mechanism.
    do
    {
        reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
    }
    while (( reg_data != 0x2F43 ) && (time_before(jiffies, timeout)));

    // calculate CRC 32
    Init_CRC32_Table ( &crc_tab[0] );

    for ( i = 0; i < 33; i++ ) // total  33 KB : 2 byte per R/W
    {
        if ( emem_type == EMEM_INFO )
			i = 32;

        if ( i < 32 )   //emem_main
        {
            if ( i == 31 )
            {
                temp[i][1014] = 0x5A; //Fmr_Loader[1014]=0x5A;
                temp[i][1015] = 0xA5; //Fmr_Loader[1015]=0xA5;

                for ( j = 0; j < 1016; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
            else
            {
                for ( j = 0; j < 1024; j++ )
                {
                    //crc_main=Get_CRC(Fmr_Loader[j],crc_main,&crc_tab[0]);
                    crc_main = Get_CRC ( temp[i][j], crc_main, &crc_tab[0] );
                }
            }
        }
        else  //emem_info
        {
            for ( j = 0; j < 1024; j++ )
            {
                //crc_info=Get_CRC(Fmr_Loader[j],crc_info,&crc_tab[0]);
                crc_info = Get_CRC ( g_dwiic_info_data[j], crc_info, &crc_tab[0] );
            }
            if ( emem_type == EMEM_MAIN ) break;
        }

        //drvDWIIC_MasterTransmit( DWIIC_MODE_DWIIC_ID, 1024, Fmr_Loader );
#if 0
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP, temp[i], 1024 );
#else
		for (j=0; j<8; ++j)
		{
			TP_DEBUG("i=%d, j=%d, FwDataCnt=%d\n", i, j, FwDataCnt);
			HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &temp[i][j*128], 128);
		}
#endif
        // polling 0x3CE4 is 0xD0BC
		timeout = jiffies + HZ/2; 	// xuke @ 20140711	Add timeout mechanism.
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }
        while (( reg_data != 0xD0BC ) && (time_before(jiffies, timeout)));

        drvDB_WriteReg ( 0x3C, 0xE4, 0x2F43 );
    }

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // write file done and check crc
        drvDB_WriteReg ( 0x3C, 0xE4, 0x1380 );
    }
    mdelay ( 10 ); //MCR_CLBK_DEBUG_DELAY ( 10, MCU_LOOP_DELAY_COUNT_MS );

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // polling 0x3CE4 is 0x9432
		timeout = jiffies + HZ/2; 	// xuke @ 20140711	Add timeout mechanism.
        do
        {
            reg_data = drvDB_ReadReg ( 0x3C, 0xE4 );
        }while (( reg_data != 0x9432 ) && (time_before(jiffies, timeout)));
    }

    crc_main = crc_main ^ 0xffffffff;
    crc_info = crc_info ^ 0xffffffff;

    if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        // CRC Main from TP
        crc_main_tp = drvDB_ReadReg ( 0x3C, 0x80 );
        crc_main_tp = ( crc_main_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0x82 );

        // CRC Info from TP
        crc_info_tp = drvDB_ReadReg ( 0x3C, 0xA0 );
        crc_info_tp = ( crc_info_tp << 16 ) | drvDB_ReadReg ( 0x3C, 0xA2 );
    }
    TP_DEBUG ( "crc_main=0x%x, crc_info=0x%x, crc_main_tp=0x%x, crc_info_tp=0x%x\n",
               crc_main, crc_info, crc_main_tp, crc_info_tp );

    //drvDB_ExitDBBUS();
    
	lct_qup_i2c_use_custom_clk(msg21xx_i2c_client->adapter, 100000);

    update_pass = 1;
	fw_in_updating_flag = 0;
	if ( ( emem_type == EMEM_ALL ) || ( emem_type == EMEM_MAIN ) )
    {
        if ( crc_main_tp != crc_main )
            update_pass = 0;

        /*if ( crc_info_tp != crc_info )
            update_pass = 0;*/
    }

    if ( !update_pass )
    {
    #ifdef LCT_UPGRADE_MSG2133
    update_complete_flag = 0;
	#endif
        pr_err( "update FAILED\n" );
		_HalTscrHWReset();
        FwDataCnt = 0;
    	enable_irq(msg21xx_irq);
        if(is_msg2133A_funa)
            return ( size );
        else
            return ( 0 );
    }

    printk ( "update OK\n" );
	_HalTscrHWReset();
    FwDataCnt = 0;
    enable_irq(msg21xx_irq);
    return size;
}

#define _FW_UPDATE_C3_
#ifdef _FW_UPDATE_C3_
static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
//    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
	disable_irq_nosync(msg21xx_irq);

    pr_err("msg2133 enter %s\n",__func__);

	fw_in_updating_flag = 1;
    _HalTscrHWReset();

    // Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
	// c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
	TP_DEBUG ( "mstar dbbus_rx version[0]=0x%x\n", dbbus_rx_data[0] );
    if ( dbbus_rx_data[0] == 2 )
    {
        // check version
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3C;
        dbbus_tx_data[2] = 0xEA;
        HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
        HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
        TP_DEBUG ( "dbbus_rx version[0]=0x%x\n", dbbus_rx_data[0] );

        if ( dbbus_rx_data[0] == 3 ){
            return firmware_update_c33 ( dev, attr, buf, size, EMEM_MAIN );
		}
        else{

            return firmware_update_c32 ( dev, attr, buf, size, EMEM_ALL );
        }
    }
    else
    {
        return firmware_update_c2 ( dev, attr, buf, size );
    } 
}
#else
static ssize_t firmware_update_store ( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
    u8 i;
    u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};

    _HalTscrHWReset();

    // 1. Erase TP Flash first
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    drvISP_EntryIspMode();
    drvISP_ChipErase();
    _HalTscrHWReset();
    mdelay ( 300 );

    // 2.Program and Verify
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set FRO to 50M
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0xE2;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    dbbus_rx_data[0] = 0;
    dbbus_rx_data[1] = 0;
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 3
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set MCU clock,SPI clock =FRO
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x22;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x23;
    dbbus_tx_data[3] = 0x00;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable slave's ISP ECO mode
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x08;
    dbbus_tx_data[2] = 0x0c;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // Enable SPI Pad
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
    TP_DEBUG ( "dbbus_rx_data[0]=0x%x", dbbus_rx_data[0] );
    dbbus_tx_data[3] = ( dbbus_rx_data[0] | 0x20 ); //Set Bit 5
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // WP overwrite
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x0E;
    dbbus_tx_data[3] = 0x02;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    // set pin high
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0x10;
    dbbus_tx_data[3] = 0x08;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );

    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();

    for ( i = 0; i < 94; i++ ) // total  94 KB : 1 byte per R/W
    {
        drvISP_Program ( i, temp[i] ); // program to slave's flash
        drvISP_Verify ( i, temp[i] ); //verify data
    }
    TP_DEBUG ( "update OK\n" );
    drvISP_ExitIspMode();
    FwDataCnt = 0;
    
    return size;
}
#endif
static DEVICE_ATTR(update, SYSFS_AUTHORITY, firmware_update_show, firmware_update_store);
#if 0
/*test=================*/
static ssize_t firmware_clear_show(struct device *dev,
                                    struct device_attribute *attr, char *buf)
{
	printk(" +++++++ [%s] Enter!++++++\n", __func__);
	u16 k=0,i = 0, j = 0;
	u8 bWriteData[5] =
	{
        0x10, 0x03, 0, 0, 0
	};
	u8 RX_data[256];
	u8 bWriteData1 = 0x12;
	u32 addr = 0;
	u32 timeOutCount=0;
	for (k = 0; k < 94; i++)   // total  94 KB : 1 byte per R/W
	{
		addr = k * 1024;
		for (j = 0; j < 8; j++)   //128*8 cycle
		{
			bWriteData[2] = (u8)((addr + j * 128) >> 16);
			bWriteData[3] = (u8)((addr + j * 128) >> 8);
			bWriteData[4] = (u8)(addr + j * 128);
			//msctpc_LoopDelay ( 1 );        // delay about 100us*****
			udelay(150);//200);

			timeOutCount=0;
			while ( ( drvISP_ReadStatus() & 0x01 ) == 0x01 )
			{
				timeOutCount++;
				if ( timeOutCount >= 100000 ) 
					break; /* around 1 sec timeout */
	  		}
        
			HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, bWriteData, 5);    //write read flash addr
			//msctpc_LoopDelay ( 1 );        // delay about 100us*****
			udelay(150);//200);
			drvISP_Read(128, RX_data);
			HalTscrCDevWriteI2CSeq(FW_UPDATE_ADDR_MSG21XX, &bWriteData1, 1);    //cmd end
			for (i = 0; i < 128; i++)   //log out if verify error
			{
				if (RX_data[i] != 0xFF)
				{
					//TP_DEBUG(printk("k=%d,j=%d,i=%d===============erase not clean================",k,j,i);)
					printk("k=%d,j=%d,i=%d  erase not clean !!",k,j,i);
				}
			}
		}
	}
	TP_DEBUG("read finish\n");
	return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_clear_store(struct device *dev,
                                     struct device_attribute *attr, const char *buf, size_t size)
{

	u8 dbbus_tx_data[4];
	unsigned char dbbus_rx_data[2] = {0};
	printk(" +++++++ [%s] Enter!++++++\n", __func__);
	//msctpc_LoopDelay ( 100 ); 	   // delay about 100ms*****

	// Enable slave's ISP ECO mode

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x08;
	dbbus_tx_data[2] = 0x0c;
	dbbus_tx_data[3] = 0x08;
	
	// Disable the Watchdog
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	dbbus_tx_data[3] = 0x00;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x60;
	dbbus_tx_data[3] = 0x55;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x3C;
	dbbus_tx_data[2] = 0x61;
	dbbus_tx_data[3] = 0xAA;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//Stop MCU
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x0F;
	dbbus_tx_data[2] = 0xE6;
	dbbus_tx_data[3] = 0x01;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//Enable SPI Pad
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x02;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	TP_DEBUG(printk("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = (dbbus_rx_data[0] | 0x20);  //Set Bit 5
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x25;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);

	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	TP_DEBUG(printk("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xFC;  //Clear Bit 1,0
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	//WP overwrite
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x0E;
	dbbus_tx_data[3] = 0x02;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);


	//set pin high
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x1E;
	dbbus_tx_data[2] = 0x10;
	dbbus_tx_data[3] = 0x08;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);
	//set FRO to 50M
	dbbus_tx_data[0] = 0x10;
	dbbus_tx_data[1] = 0x11;
	dbbus_tx_data[2] = 0xE2;
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 3);
	dbbus_rx_data[0] = 0;
	dbbus_rx_data[1] = 0;
	HalTscrCReadI2CSeq(FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2);
	TP_DEBUG(printk("dbbus_rx_data[0]=0x%x", dbbus_rx_data[0]);)
	dbbus_tx_data[3] = dbbus_rx_data[0] & 0xF7;  //Clear Bit 1,0
	HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX, dbbus_tx_data, 4);

	dbbusDWIICIICNotUseBus();
	dbbusDWIICNotStopMCU();
	dbbusDWIICExitSerialDebugMode();

    ///////////////////////////////////////
    // Start to load firmware
    ///////////////////////////////////////
    drvISP_EntryIspMode();
	TP_DEBUG(printk("chip erase+\n");)
    drvISP_BlockErase(0x00000);
	TP_DEBUG(printk("chip erase-\n");)
    drvISP_ExitIspMode();
    return size;
}
static DEVICE_ATTR(clear, SYSFS_AUTHORITY, firmware_clear_show, firmware_clear_store);
#endif //0
/*test=================*/
/*Add by Tracy.Lin for update touch panel firmware and get fw version*/

static ssize_t firmware_version_show(struct device *dev,
                                     struct device_attribute *attr, char *buf)
{
    TP_DEBUG("*** firmware_version_show fw_version = %s***\n", fw_version);
    return sprintf(buf, "%s\n", fw_version);
}

static ssize_t firmware_version_store(struct device *dev,
                                      struct device_attribute *attr, const char *buf, size_t size)
{
    unsigned char dbbus_tx_data[3];
    unsigned char dbbus_rx_data[4] ;
    unsigned short major=0, minor=0;
/*
    dbbusDWIICEnterSerialDebugMode();
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();

*/
    fw_version = kzalloc(sizeof(char), GFP_KERNEL);

    //Get_Chip_Version();
    
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0x00;
	if(is_msg2133A_funa == 1)
	{
		dbbus_tx_data[2] = 0x74;
	}
	else
	{
    	dbbus_tx_data[2] = 0x2A;
	}
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
	if(is_msg2133A_funa == 1)
	{
		msleep(50);
	}
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4);

    major = (dbbus_rx_data[1]<<8)+dbbus_rx_data[0];
    minor = (dbbus_rx_data[3]<<8)+dbbus_rx_data[2];

	if(is_msg2133A_funa == 1)
	{
		major = dbbus_rx_data[3];
    	minor = dbbus_rx_data[2];
	}

    TP_DEBUG("***major = %d ***\n", major);
    TP_DEBUG("***minor = %d ***\n", minor);
    sprintf(fw_version,"%03d%03d", major, minor);
    //TP_DEBUG(printk("***fw_version = %s ***\n", fw_version);)


    return size;
}
static DEVICE_ATTR(version, SYSFS_AUTHORITY, firmware_version_show, firmware_version_store);

static ssize_t firmware_data_show(struct device *dev,
                                  struct device_attribute *attr, char *buf)
{
    return FwDataCnt;
}

static ssize_t firmware_data_store(struct device *dev,
                                   struct device_attribute *attr, const char *buf, size_t size)
{
    //int i;
	TP_DEBUG("***FwDataCnt = %d ***\n", FwDataCnt);
	//printk("p_update FwDataCnt=%d\n",FwDataCnt);
    //for (i = 0; i < 1024; i++)
    {
        memcpy(temp[FwDataCnt], buf, 1024);
    }
	FwDataCnt++;
	return size;
}
static DEVICE_ATTR(data, SYSFS_AUTHORITY, firmware_data_show, firmware_data_store);
#endif



int msg21xx_i2c_rx_data(char *buf, int len)
{
	uint8_t i;
	struct i2c_msg msg[] = {
		{
			.addr	= msg21xx_i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= len,
			.buf	= buf,
		}
	};
	
	for (i = 0; i < MSG21XX_RETRY_COUNT; i++) {
		if (i2c_transfer(msg21xx_i2c_client->adapter, msg, 1) >= 0) {
			break;
		}
		mdelay(10);
	}

	if (i >= MSG21XX_RETRY_COUNT) {
		pr_err("%s: retry over %d\n", __FUNCTION__, MSG21XX_RETRY_COUNT);
		return -EIO;
	}
	return 0;
}

void msg21xx_i2c_wr_data(u8 addr, u8* data, u16 size)
{
	//according to your platform.
	int rc;
 
	struct i2c_msg msgs[] = {
		{
			.addr = addr,
			.flags = 0,
			.len = size,
			.buf = data,
		},
	};
	rc = i2c_transfer(msg21xx_i2c_client->adapter, msgs, 1);
	if(rc < 0){
		printk("HalTscrCDevWriteI2CSeq error %d\n", rc);
	}
}

int msg21xx_i2c_wr_data_ext(u8 addr, u8* data, u16 size)
{
	//according to your platform.
	int rc;
 
	struct i2c_msg msgs[] = {
		{
			.addr = addr,
			.flags = 0,
			.len = size,
			.buf = data,
		},
	};
	rc = i2c_transfer(msg21xx_i2c_client->adapter, msgs, 1);
	if(rc < 0){
		printk("HalTscrCDevWriteI2CSeq error %d\n", rc);
		return rc;
	}
	return 0;
}

#if 0
static void msg21xx_chip_init(void)
{
    /* After the LCD is on, power on the TP controller */
   // printk("wax->%s\n",__FUNCTION__);    
	//gpio_direction_output(MSG21XX_RESET_GPIO, 1);
	gpio_direction_output(pdata->reset_gpio, 1);
	printk(" msg21xx_chip_init before set gpio\n");
	gpio_set_value(pdata->reset_gpio, 0);	
    printk(" msg21xx_chip_init after set gpio\n");
	mdelay(200); 
	gpio_set_value(pdata->reset_gpio, 1);
    mdelay(500);/* Note that the RST must be in LOW 10ms at least */
	//gpio_set_value(MSG21XX_RESET_GPIO, 1);
    /* Enable the interrupt service thread/routine for INT after 50ms */
    //mdelay(50);

}

static void msg21xx_ts_early_suspend(struct early_suspend *h)
{
	//printk("wax->%s\n",__FUNCTION__);
	printk("%s: enter\n", __func__);
	#ifdef CONFIG_LCT_AE515
	 pmapp_disp_backlight_set_brightness(0);
	printk(KERN_ERR"pmapp_disp_backlight_set_brightness");
       #endif //for lcdc boyi backlight
#if __MSG2133A_FIRMWARE_UPDATE__
	if(FwDataCnt)
	{
		return;
	}
#endif


	gpio_set_value(MSG21XX_RESET_GPIO, 0);
	disable_irq_nosync(msg21xx_irq);

	
}

static void msg21xx_ts_late_resume(struct early_suspend *h)
{
	struct irq_desc *desc = irq_to_desc(msg21xx_irq);
	//printk("wax->%s\n",__FUNCTION__);
	//u8 buf[4] = {0x52,0x00,0x64,0xA0};
	printk("%s: enter\n", __func__);


	printk("%s: desc->depth:%d\n", __func__,desc->depth);

	

	//msleep(100);
	if(1)  // the action is in lcd_panel on
		{
		gpio_set_value(MSG21XX_RESET_GPIO, 1);
		msleep(300);
		}
		
	enable_irq(msg21xx_irq);
	//msleep(100);


}


#endif





u8 Calculate_8BitsChecksum( u8 *msg, s32 s32Length )
{
    s32 s32Checksum = 0;
    s32 i;

    for ( i = 0 ; i < s32Length; i++ )
    {
        s32Checksum += msg[i];
    }

    return (u8)( ( -s32Checksum ) & 0xFF );
}


static void msg21xx_do_work(struct work_struct *work)
{
	u8 val[8] = {0};
	u8 Checksum = 0;
	u8 i;
	u32 delta_x = 0, delta_y = 0;
	u32 u32X = 0;
	u32 u32Y = 0;
	u8 touchkeycode = 0;
	TouchScreenInfo_t touchData;
	static u32 preKeyStatus=0;
	//static u32 preFingerNum=0;
    #ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	uint8_t is_pointer_event = 0;
    //U8 data[REPORT_PACKET_LENGTH] = {0};
	#endif
#ifdef SWAP_X_Y
	int tempx;
	int tempy;
#endif
#ifdef MSG_GESTURE_FUNCTION
	int closeGesturnRetval = 0;
	unsigned long timeout = 0;
#endif

	mutex_lock(&msg21xx_mutex);
	msg21xx_i2c_rx_data( &val[0], REPORT_PACKET_LENGTH );
	Checksum = Calculate_8BitsChecksum( val, (REPORT_PACKET_LENGTH-1) );
    #ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	#ifdef DEBUG_TP_SENSOR
	 if(tp_prox_sensor_opened)
	 {
	 	printk("[mstar]:%s:  val[5]=%x\n", __func__, val[5]);
		printk("[mstar]:%s:  tp_sensor_opened=%d,tp_pre_sensor_data=%d\n", __func__,tp_prox_sensor_opened,tp_pre_sensor_data);
	 }
    #endif
	if(tp_prox_sensor_opened)
	{
		if(val[5] == 0x80) // close Panel
		{	
			tp_prox_sensor_data = 0;
            printk(" tp_prox_sensor_data = %d,close\n",tp_prox_sensor_data);
            
		}
		else if(val[5] == 0x40) // Open Panel
		{			
			tp_prox_sensor_data = 1;
            printk(" tp_prox_sensor_data = %d,leave\n",tp_prox_sensor_data);
		}
		else
		{
			is_pointer_event = 1;
		}
		#ifdef DEBUG_TP_SENSOR
		if(tp_prox_sensor_opened)
		{
			printk("[mstar]:%s:  is_pointer_event=%d,tp_sensor_data=%d,tp_pre_sensor_data=%d\n", __func__,is_pointer_event,tp_prox_sensor_data,tp_pre_sensor_data);
		}
		#endif
		 if((is_pointer_event != 1) && (tp_prox_sensor_opened == 1) && (tp_pre_sensor_data != tp_prox_sensor_data))
		 {  
		    printk("[mstar]:%s:  sensor data changed\n", __func__);
			tp_pre_sensor_data = tp_prox_sensor_data;
			mutex_lock(&tp_prox_sensor_mutex);
	        tp_prox_sensor_data_changed = 1;
	        mutex_unlock(&tp_prox_sensor_mutex);
	        wake_up_interruptible(&tp_sensor_waitqueue);
			//return 1;
			
		 }
	}
	if(is_need_report_pointer == 0)
	{
		printk("[mstar]:%s:  we don not report pointer when sleep in call\n", __func__);
		//return 1;
	}
	#endif

#ifdef MSG_GESTURE_FUNCTION
	if (msg_GetGestureFlag())
	{							
		if(( val[0] == 0x52 ) && ( val[1] == 0xFF ) && ( val[2] == 0xFF ) && ( val[3] == 0xFF ) && ( val[4] == 0xFF ) && ( val[6] == 0xFF ) && ( Checksum == val[7] ))
		{
			printk("tpd_touchinfo gesture function mode read data--%0x \n",val[5]);
			if(val[5] == 0x58)
			{
				msg_SetDoubleClickModeFlage(1);
			}
			else if(val[5] == 0x60)
			{
				msg_SetUpDirectModeFlage(1);
			}
			else if(val[5] == 0x61)
			{
				msg_SetDownDirectModeFlage(1);
			}
			else if(val[5] == 0x62)
			{
				msg_SetLeftDirectModeFlage(1);
			}
			else if(val[5] == 0x63)
			{
				msg_SetRightDirectModeFlage(1);
			}
			 if((val[5] == 0x58)||(val[5] == 0x60)||(val[5] == 0x61)||
				(val[5] == 0x62)||(val[5] == 0x63))
			{
				timeout = jiffies + HZ/2;	 // xuke @ 20140711  Add timeout mechanism.
				while ((closeGesturnRetval == 0) && (time_before(jiffies, timeout)))
				{
					closeGesturnRetval = msg_CloseGestureFunction();
				}/////be sure close this function
			}
			  /// app need to do something
		 }
	}  
#endif

    if ((Checksum == val[7]) && (val[0] == 0x52))   //check the checksum  of packet
    {
        u32X = (((val[1] & 0xF0) << 4) | val[2]);         //parse the packet to coordinates
        u32Y = (((val[1] & 0x0F) << 8) | val[3]);

        delta_x = (((val[4] & 0xF0) << 4) | val[5]);
        delta_y = (((val[4] & 0x0F) << 8) | val[6]);

#ifdef SWAP_X_Y
        tempy = u32X;
        tempx = u32Y;
        u32X = tempx;
        u32Y = tempy;

        tempy = delta_x;
        tempx = delta_y;
        delta_x = tempx;
        delta_y = tempy;
#endif
#ifdef REVERSE_X
		u32X = 2047 - u32X;
		delta_x = 4095 - delta_x;
#endif
#ifdef REVERSE_Y
		u32Y = 2047 - u32Y;
		delta_y = 4095 - delta_y;
#endif
		//DBG("[HAL] u32X = %x, u32Y = %x", u32X, u32Y);
		//DBG("[HAL] delta_x = %x, delta_y = %x", delta_x, delta_y);

		if ((val[1] == 0xFF) && (val[2] == 0xFF) && (val[3] == 0xFF) && (val[4] == 0xFF) && (val[6] == 0xFF))
		{
			touchData.Point[0].X = 0; // final X coordinate
			touchData.Point[0].Y = 0; // final Y coordinate

			if((val[5]==0x0)||(val[5]==0xFF))
			{
				touchData.nFingerNum = 0; //touch end
				touchData.nTouchKeyCode = 0; //TouchKeyMode
				touchData.nTouchKeyMode = 0; //TouchKeyMode
			}
			else
			{
				touchData.nTouchKeyMode = 1; //TouchKeyMode
				touchData.nTouchKeyCode = val[5]; //TouchKeyCode
				touchData.nFingerNum = 1;
			}
		}
		else
		{
			touchData.nTouchKeyMode = 0; //Touch on screen...

			if(
#ifdef REVERSE_X
			(delta_x == 4095)
#else
			(delta_x == 0) 
#endif
			&& 
#ifdef REVERSE_Y
			(delta_y == 4095)
#else
			(delta_y == 0)
#endif
			)
			{
				if(u32X >=0&&u32X <=2047&&u32Y >=0&&u32Y<=2047)
				{
					touchData.nFingerNum = 1; //one touch
					touchData.Point[0].X = (u32X * pdata->x_max) / 2048;
					touchData.Point[0].Y = (u32Y * pdata->y_max) / 2048;
				}
				else
				{
					enable_irq(msg21xx_irq);
					mutex_unlock(&msg21xx_mutex);
					return;
				}
			}
			else
			{
				u32 x2, y2;
				/* Finger 2 */
				if (delta_x > 2048)     //transform the unsigh value to sign value
				{
				delta_x -= 4096;
				}
				if (delta_y > 2048)
				{
				delta_y -= 4096;
				}

				x2 = (u32)(u32X + delta_x);
				y2 = (u32)(u32Y + delta_y);

				if(u32X >=0&&u32X <=2047&&u32Y >=0&&u32Y<=2047
				&&x2 >=0&&x2 <=2047&&y2 >=0&&y2<=2047)
				{
					touchData.nFingerNum = 2; //two touch
					/* Finger 1 */
					touchData.Point[0].X = (u32X * pdata->x_max) / 2048;
					touchData.Point[0].Y = (u32Y * pdata->y_max) / 2048;
					
					touchData.Point[1].X = (x2 * pdata->x_max) / 2048;
					touchData.Point[1].Y = (y2 * pdata->y_max) / 2048;
				}
				else
				{
					enable_irq(msg21xx_irq);
					mutex_unlock(&msg21xx_mutex);
					return;
				}
			}
		}

		//report...
		if(touchData.nTouchKeyMode)
		{
			if(is_msg2133A == 1)
			{
				if (touchData.nTouchKeyCode == 4)
				{
					touchkeycode = KEY_BACK;
#ifdef MSTAR_USE_VIRTUALKEY
					virtualkey_x = 540;
					virtualkey_y = 1060;
#endif
				}
				if (touchData.nTouchKeyCode == 2)
				{
					touchkeycode = KEY_HOMEPAGE;
#ifdef MSTAR_USE_VIRTUALKEY
					virtualkey_x = 270;
					virtualkey_y = 1060;
#endif
				}
				if (touchData.nTouchKeyCode == 1)
				{
					touchkeycode = KEY_MENU;
#ifdef MSTAR_USE_VIRTUALKEY
					virtualkey_x = 90;
					virtualkey_y = 1060;
#endif
				}
			}
			else
			{
				if (touchData.nTouchKeyCode == 1)
				{
					touchkeycode = KEY_MENU;
#ifdef MSTAR_USE_VIRTUALKEY
					virtualkey_x = 270;
					virtualkey_y = 496;
#endif
				}
				if (touchData.nTouchKeyCode == 4)
				{
					touchkeycode = KEY_BACK;
#ifdef MSTAR_USE_VIRTUALKEY
					virtualkey_x = 60;
					virtualkey_y = 496;
#endif
				}
				if (touchData.nTouchKeyCode == 2)
				{
					touchkeycode = KEY_HOME;
#ifdef MSTAR_USE_VIRTUALKEY
					virtualkey_x = 160;
					virtualkey_y = 496;
#endif
				}
			}
			//if (touchData.nTouchKeyCode == 8)
			//	touchkeycode = KEY_SEARCH;

			if(preKeyStatus!=touchkeycode)
			{
                after0 = 0;
				preKeyStatus=touchkeycode;

#ifdef MSTAR_USE_VIRTUALKEY
				input_report_abs(input, ABS_MT_TOUCH_MAJOR, 1);
				input_report_abs(input, ABS_MT_WIDTH_MAJOR, 1);
				input_report_abs(input, ABS_MT_POSITION_X, virtualkey_x);
				input_report_abs(input, ABS_MT_POSITION_Y, virtualkey_y);
				input_mt_sync(input);	
                input_report_key(input, BTN_TOUCH, 1);
				input_sync(input);
#else
				//input_report_key(input, touchkeycode, 1);
				//input_sync(input);
                input_report_key(input, BTN_TOUCH, 1);
	            input_report_key(input, touchkeycode, 1); 
                input_sync(input);
#endif
				//printk("&&&&&&&&useful key code report touch key code = %d\n",touchkeycode);
			}
		}
		else
		{
			preKeyStatus=0; //clear key status..

            if((touchData.nFingerNum) == 0&&after0==0)   //touch end
			{
                after0 = 1;
                //preFingerNum=0;
				input_report_key(input, KEY_MENU, 0);
				input_report_key(input, KEY_HOMEPAGE, 0);
				input_report_key(input, KEY_BACK, 0);
				input_report_key(input, KEY_SEARCH, 0);

				input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
				input_mt_sync(input);
                input_report_key(input, BTN_TOUCH, 0);
				input_sync(input);
			}
			else //touch on screen
            {
                after0 = 0; 
				/*
				if(preFingerNum!=touchData.nFingerNum)   //for one touch <--> two touch issue
				{
				printk("langwenlong number has changed\n");
				preFingerNum=touchData.nFingerNum;
				input_report_abs(input, ABS_MT_TOUCH_MAJOR, 0);
				input_mt_sync(input);
				input_sync(input);
				}*/

				for(i = 0;i < (touchData.nFingerNum);i++)
				{
//					printk("jacky: i=%d, x=%d, y=%d\n", i, touchData.Point[i].X, touchData.Point[i].Y);
					input_report_abs(input, ABS_MT_PRESSURE, 15);
					input_report_abs(input, ABS_MT_TOUCH_MAJOR, 15);
					input_report_abs(input, ABS_MT_POSITION_X, touchData.Point[i].X);
					input_report_abs(input, ABS_MT_POSITION_Y, touchData.Point[i].Y);
					input_mt_sync(input);
                    input_report_key(input, BTN_TOUCH, 1);
                }

				input_sync(input);
			}
		}
	}
	else
	{
		//DBG("Packet error 0x%x, 0x%x, 0x%x", val[0], val[1], val[2]);
		//DBG("             0x%x, 0x%x, 0x%x", val[3], val[4], val[5]);
		//DBG("             0x%x, 0x%x, 0x%x", val[6], val[7], Checksum);
		printk(KERN_ERR "err status in tp\n");
	}

	enable_irq(msg21xx_irq);
	mutex_unlock(&msg21xx_mutex);
}

static int msg21xx_ts_open(struct input_dev *dev)
{
	return 0;
}

static void msg21xx_ts_close(struct input_dev *dev)
{
	printk("msg21xx_ts_close\n");

}


static int msg21xx_init_input(void)
{//Dèòaìí?ó


	//int ret;
	int err;

	//printk("wax->%s\n",__FUNCTION__);

	printk("%s: msg21xx_i2c_client->name:%s\n", __func__,msg21xx_i2c_client->name);
	input = input_allocate_device();
	input->name = msg21xx_i2c_client->name;
	input->phys = "I2C";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &msg21xx_i2c_client->dev;
	input->open = msg21xx_ts_open;
	input->close = msg21xx_ts_close;

	set_bit(EV_ABS, input->evbit);
	set_bit(EV_SYN, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);   //open by dingliang
	set_bit(INPUT_PROP_DIRECT, input->propbit);
	#ifdef MSTAR_USE_VIRTUALKEY
    set_bit(EV_KEY, input->evbit);
    set_bit(BTN_MISC, input->keybit);
	set_bit(KEY_OK, input->keybit);
    set_bit(KEY_MENU, input->keybit);
	set_bit(KEY_BACK, input->keybit);
    #else
    {
        int i;
        for(i = 0; i < MAX_KEY_NUM; i++)
        {   
			input_set_capability(input, EV_KEY, tpd_key_array[i]);			
        }
    }
    #endif
	
  
  
  	input_set_abs_params(input, ABS_MT_TRACKING_ID, 0, 2, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 2, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_X, 0, pdata->x_max, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, pdata->y_max, 0, 0);

	err = input_register_device(input);
	if (err)
		goto fail_alloc_input; 

	//create simulate key input
	//#ifndef MSTAR_USE_VIRTUALKEY//danyu
	#if 0
	simulate_key_input = input_allocate_device();
	if (simulate_key_input == NULL) {
		printk("%s: not enough memory for input device\n", __func__);
		err = -ENOMEM;
		goto fail_alloc_input;
	}
	simulate_key_input->name = "7x27a_kp";//"7x27a_kp";
	__set_bit(EV_KEY, simulate_key_input->evbit);
	__set_bit(KEY_BACK, simulate_key_input->keybit);
	__set_bit(KEY_MENU, simulate_key_input->keybit);
	__set_bit(KEY_HOME, simulate_key_input->keybit);
	__set_bit(ST_KEY_SEARCH, simulate_key_input->keybit);
	err = input_register_device(simulate_key_input);
	if (err != 0) {
		printk("%s: failed to register input device\n", __func__);
		goto fail_alloc_input;
	}
	#endif

	

	
fail_alloc_input:	

	return 0;
}
static irqreturn_t msg21xx_interrupt(int irq, void *dev_id)
{	
	//printk("wax->%s\n",__FUNCTION__);
	//printk("%s: enter\n", __func__);

	disable_irq_nosync(msg21xx_irq);
	schedule_work(&msg21xx_wq);
	return IRQ_HANDLED;
}



/// @param[in]	addr		 is the SM-bus address
/// @param[in]	numOfBytes	 is the number of bytes to be read
/// @param[out] pDataToRead[0]~pDataToRead[numOfBytes-1] will be the read data.
/// @pre		pDataToRead[0]~pDataToRead[numOfBytes-1] is allocated.
int SMBus_Read(U16 addr, U32 numOfBytes,  U8 *pDataToRead)
{
	int ret = -1;
	U8 dbbus_tx_data[4];
	
	dbbus_tx_data[0] = 0x53;
	dbbus_tx_data[1] = (addr>>8)&0xFF;		  
	dbbus_tx_data[2] = addr&0xFF;		 

	msg21xx_i2c_wr_data(  TOUCH_ADDR_MSG21XX, dbbus_tx_data,3 );
	ret = msg21xx_i2c_rx_data((char*)pDataToRead,4);
	return ret;
}

int Read_FW_Version(void)
{
	int ret = -1;
	U8 dbbus_rx_data[4] = {0};
	U16 Major,Minor;

//	printk("xuke: %s, is_msg2133A=%d\n", __func__, is_msg2133A);
	if(is_msg2133A == 1)
	{
		ret = SMBus_Read(0x002a, 4, &dbbus_rx_data[0]);
	}
	else
	{
		ret = SMBus_Read(0x0074, 4, &dbbus_rx_data[0]);
	}
	
	if(ret == 0)
	{
		Major = ((U16)dbbus_rx_data[1]<<8)| (dbbus_rx_data[0]); // the major version number infact it is module 
		Minor = ((U16)dbbus_rx_data[3]<<8)| (dbbus_rx_data[2]); // the minor version number infact it is firmware version
		mstar_module_name = Major;
		mstar_firmware_version = Minor;
		printk("%s: mstar_module_name:%d,mstar_firmware_version:%d\n", __func__,mstar_module_name,mstar_firmware_version);
	}
	return ret;
}

#ifdef CYTTSP_SUPPORT_READ_TP_VERSION

static int tp_version_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int tp_version_release(struct inode *inode, struct file *file)
{
	return 0;
}

static ssize_t tp_version_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{

	int i;
	
	char tp_version[30] = {0};
	char ic_type[10] = {0};
	printk("%s Enter,mstar_vendor_id=%d,mstar_firmware_version=%d\n",__func__,mstar_module_name,mstar_firmware_version);
	if(mstar_module_name < 0)
	{
		strcpy(tp_version,"no tp");
	}
	else
	{
		for (i=0;i< sizeof(tp_info)/sizeof(tp_info_t);i++)
		{
			if (mstar_module_name == tp_info[i].id)
				break;
		}
		if(is_msg2133A == 1)
		{
			strcpy(ic_type,"msg2133A");
		}
		else
		{
			strcpy(ic_type,"msg2133");
		}
		
		if(i == sizeof(tp_info)/sizeof(tp_info_t))
			snprintf(tp_version, 30,"M%xV%x-%s-%s\n",mstar_module_name,mstar_firmware_version,"unknown",ic_type);
		else
		{
			#if __FIRMWARE_AUTO_UPDATE__
			snprintf(tp_version, 30,"M%xV%x-%s-%s-%s\n", mstar_module_name,mstar_firmware_version,tp_info[i].module_name,ic_type,"open");
			#else
			snprintf(tp_version, 30,"M%xV%x-%s-%s\n", mstar_module_name,mstar_firmware_version,tp_info[i].module_name,ic_type);
			#endif
		}
	}
	if(copy_to_user(buf, tp_version, strlen(tp_version)))
		return -EFAULT;

	return strlen(tp_version);
}


static struct file_operations tp_version_fops = {
	.owner = THIS_MODULE,
	.open = tp_version_open,
	.release = tp_version_release,
	.read = tp_version_read,
};


static struct miscdevice tp_version_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "tp_version",
	.fops = &tp_version_fops,
};
#endif

#if __MSG2133A_FIRMWARE_UPDATE__
static int update_func(void *pointer)
{
		firmware_update_store(NULL,NULL,NULL,0);
		return 0;
}
#endif

#if 1
static int msg21xx_setic_flag(void)
{
	int ret = 0;
  	u8 dbbus_tx_data[4];
    unsigned char dbbus_rx_data[2] = {0};
	//disable_irq(msg21xx_irq);
	//printk("%s: enter\n", __func__);

    _HalTscrHWReset();

    // Erase TP Flash first
    ret = dbbusDWIICEnterSerialDebugMode();
	if (ret < 0)
	{
//		printk("%s, error=%d\n", __func__, ret);
		goto Err_DebugMode;
	}
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();
    mdelay ( 300 );

    // Disable the Watchdog
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x60;
    dbbus_tx_data[3] = 0x55;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x3C;
    dbbus_tx_data[2] = 0x61;
    dbbus_tx_data[3] = 0xAA;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    // Stop MCU
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x0F;
    dbbus_tx_data[2] = 0xE6;
    dbbus_tx_data[3] = 0x01;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 4 );
    /////////////////////////
    // Difference between C2 and C3
    /////////////////////////
	// c2:2133 c32:2133a(2) c33:2138
    //check id
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x1E;
    dbbus_tx_data[2] = 0xCC;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX, dbbus_tx_data, 3 );
    HalTscrCReadI2CSeq ( FW_ADDR_MSG21XX, &dbbus_rx_data[0], 2 );
	printk("msg21xx_setic_flag dbbus_rx_data[0]=0x%x\n",dbbus_rx_data[0]);
    if(dbbus_rx_data[0]!=1//msg2133
        &&dbbus_rx_data[0]!=2//msg21xxA
        &&dbbus_rx_data[0]!=3)//msg26xx
    {
        is_msg2133A = 0;
    }
    else
    {
        is_msg2133A = 1;
    }
    /*
	if(dbbus_rx_data[0] ==1)
	{
		printk("msg21xx_setic_flag it is msg2133 ic\n");
		is_msg2133A = 0;
	}
	else
	{	
		printk("msg21xx_setic_flag it is msg2133A ic\n");
		is_msg2133A = 1;
	}
	*/
Err_DebugMode:
	return ret;
}
#endif
static int msg21xx_power_on(struct msg21xx_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	if (gpio_is_valid(data->pdata->power_ldo_gpio))
	{
		printk("%s, power_ldo_gpio\n", __func__);
		gpio_set_value(data->pdata->power_ldo_gpio, 1);
	}
	else
	{
		printk("%s, regulator\n", __func__);
		rc = regulator_enable(data->vdd);
		if (rc)
		{
			dev_err(&data->client->dev, "Regulator vdd enable failed rc=%d\n", rc);
			return rc;
		}
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc)
	{
		dev_err(&data->client->dev, "Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	if (gpio_is_valid(data->pdata->power_ldo_gpio))
	{
		gpio_set_value(data->pdata->power_ldo_gpio, 0);
	}
	else
	{
		rc = regulator_disable(data->vdd);
		if (rc)
		{
			dev_err(&data->client->dev, "Regulator vdd disable failed rc=%d\n", rc);
			return rc;
		}
	}
	rc = regulator_disable(data->vcc_i2c);
	if (rc)
	{
		dev_err(&data->client->dev, "Regulator vcc_i2c disable failed rc=%d\n", rc);
	}

	return rc;
}



static int msg21xx_power_init(struct msg21xx_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;
	
	if (gpio_is_valid(data->pdata->power_ldo_gpio))
	{
		printk("%s, power_ldo_gpio\n", __func__);
		rc = gpio_request(data->pdata->power_ldo_gpio, "msg21xx_ldo_gpio");
		if (rc)
		{
			printk("irq gpio request failed\n");
			return rc;
		}
		
		rc = gpio_direction_output(data->pdata->power_ldo_gpio, 1);
		if (rc)
		{
			printk("set_direction for irq gpio failed\n");
			goto free_ldo_gpio;
		}
	}
	else
	{
		printk("%s, regulator\n", __func__);
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd))
		{
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev, "Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0)
		{
			rc = regulator_set_voltage(data->vdd, VTG_MIN_UV, VTG_MAX_UV);
			if (rc)
			{
				dev_err(&data->client->dev, "Regulator set_vtg failed vdd rc=%d\n", rc);
				goto reg_vdd_put;
			}
		}
	}
	
	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c))
	{
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev, "Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0)
	{
		rc = regulator_set_voltage(data->vcc_i2c, I2C_VTG_MIN_UV, I2C_VTG_MAX_UV);
		if (rc)
		{
			dev_err(&data->client->dev, "Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (!gpio_is_valid(data->pdata->power_ldo_gpio))
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0, VTG_MAX_UV);
reg_vdd_put:
free_ldo_gpio:	
	if (gpio_is_valid(data->pdata->power_ldo_gpio))
		gpio_free(data->pdata->power_ldo_gpio);
	else
		regulator_put(data->vdd);
	return rc;
pwr_deinit:
	if (gpio_is_valid(data->pdata->power_ldo_gpio))
		gpio_free(data->pdata->power_ldo_gpio);
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

static int msg21xx_pinctrl_init(struct msg21xx_ts_data *data)
{
	int retval;
	/* Get pinctrl if target uses pinctrl */
	data->ts_pinctrl = devm_pinctrl_get(&(data->client->dev));
	if (IS_ERR_OR_NULL(data->ts_pinctrl)) {
		dev_dbg(&data->client->dev,
			"Target does not use pinctrl\n");
		retval = PTR_ERR(data->ts_pinctrl);
		data->ts_pinctrl = NULL;
		return retval;
	}
    
	data->gpio_state_active
		= pinctrl_lookup_state(data->ts_pinctrl,
			"pmx_ts_active");
	if (IS_ERR_OR_NULL(data->gpio_state_active)) {
		printk("%s Can not get ts default pinstate\n", __func__);
		retval = PTR_ERR(data->gpio_state_active);
		data->ts_pinctrl = NULL;
		return retval;
	}

	data->gpio_state_suspend
		= pinctrl_lookup_state(data->ts_pinctrl,
			"pmx_ts_suspend");
	if (IS_ERR_OR_NULL(data->gpio_state_suspend)) {
		dev_err(&data->client->dev,
			"Can not get ts sleep pinstate\n");
		retval = PTR_ERR(data->gpio_state_suspend);
		data->ts_pinctrl = NULL;
		return retval;
	}

	return 0;
}

static int msg21xx_pinctrl_select(struct msg21xx_ts_data *data,
						bool on)
{
	struct pinctrl_state *pins_state;
	int ret;
	pins_state = on ? data->gpio_state_active
		: data->gpio_state_suspend;
	if (!IS_ERR_OR_NULL(pins_state)) {
		ret = pinctrl_select_state(data->ts_pinctrl, pins_state);
		if (ret) {
			dev_err(&data->client->dev,
				"can not set %s pins\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
			return ret;
		}
	} else {
		dev_err(&data->client->dev,
			"not a valid '%s' pinstate\n",
				on ? "pmx_ts_active" : "pmx_ts_suspend");
	}

	return 0;
}


#ifdef CONFIG_OF
static int msg21xx_get_dt_coords(struct device *dev, char *name,
				struct msg21xx_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "mstar,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "mstar,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int msg21xx_parse_dt(struct device *dev,
			struct msg21xx_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];

	pdata->name = "mstar";
	rc = of_property_read_string(np, "mstar,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = msg21xx_get_dt_coords(dev, "mstar,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = msg21xx_get_dt_coords(dev, "mstar,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"mstar,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"mstar,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "mstar,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "mstar,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;

	/* power ldo gpio info*/
	pdata->power_ldo_gpio = of_get_named_gpio_flags(np, "mstar,power_ldo-gpio",
				0, &pdata->power_ldo_gpio_flags);
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
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"mstar,button-map", button_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int msg21xx_parse_dt(struct device *dev,
			struct msg21xx_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#ifdef LCT_UPGRADE_MSG2133
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

static int msg2133_ctp_upgrade_func(void)
{
	return ctp_upgrade_from_engineermode(msg21xx_i2c_client);
}



static void msg2133_ctp_upgrade_read_ver_func(char *ver)
{
	int cnt= 0;
	
	if(ver == NULL)
	{
		return;
	}
	
	Read_FW_Version();
		
	cnt = sprintf(ver, "vid:0x%04x,fw:0x%04x,ic:%s\n",mstar_module_name,mstar_firmware_version,"msg2133");
	return ;

	

}
#endif

#ifdef MSG_ITO_TEST
static void ito_test_create_entry(void);

typedef enum
{
	ITO_TEST_OK = 0,
	ITO_TEST_FAIL,
	ITO_TEST_GET_TP_TYPE_ERROR,
} ITO_TEST_RET;

static ITO_TEST_RET ito_test_interface(void);

static int msg2133_chip_self_test(void)
{
	return (int)ito_test_interface();
}
#endif

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR

	static void tp_prox_sensor_enable(int enable)
	{
		u8 ps_store_data[4];
		int ret = 0;
	   printk(KERN_ERR"tp_sensor_enable  enable = %d\n",enable );
	    if(i2c_prox_client==NULL)
	        return;

		// xuke @ 20140515	Fix the issue that prox enabled before tp resume would cause i2c error.
		if (!gpio_get_value(pdata->reset_gpio))
		{
			gpio_set_value(pdata->reset_gpio, 1);
			msleep(30);
			printk("xuke: pull up reset gpio\n");
		}
		
		if(enable == 1)
		{
			ps_store_data[0] = 0x52;
			ps_store_data[1] = 0x00;
            ps_store_data[2] = 0x4A;
			ps_store_data[3] = 0xa0;
			
			ret = msg21xx_i2c_wr_data_ext(TOUCH_ADDR_MSG21XX, &ps_store_data[0], 4);
			if(ret < 0)
			{
				printk(KERN_ERR "tp_sensor_enable on i2c_master_send failed \n");
			}
            else
            {
                printk(KERN_ERR "tp_sensor_enable on i2c_master_send success \n");
            }

		}
		else
		{		
			ps_store_data[0] = 0x52;
			ps_store_data[1] = 0x00;		
	        ps_store_data[2] = 0x4A; 
			ps_store_data[3] = 0xa1;
			ret = msg21xx_i2c_wr_data_ext(TOUCH_ADDR_MSG21XX, &ps_store_data[0], 4);
			if(ret < 0) 
			{
				printk(KERN_ERR "tp_sensor_enable off i2c_master_send failed \n");
			}
		}	
	
		
	}

static ssize_t tp_prox_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 4, "%d\n", tp_prox_sensor_opened);
}

static ssize_t tp_prox_enable_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct input_dev *input_dev = input_prox_dev;
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	pr_info("%s, data=%ld\n",__func__,data);
	mutex_lock(&input_dev->mutex);
	tp_prox_sensor_enable((int)data);
	if(data){
		#ifdef SENSOR_PROX_TP_USE_WAKELOCK
		wake_lock(&sensor_prox_tp_wake_lock);
		#endif
		tp_prox_sensor_opened = 1;
		tp_prox_sensor_data = 1;
		tp_prox_sensor_data_changed = 1;
	}else{
		tp_prox_sensor_opened = 0;
		#ifdef SENSOR_PROX_TP_USE_WAKELOCK
		wake_unlock(&sensor_prox_tp_wake_lock);
		#endif

	}
	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(enable, SYSFS_AUTHORITY,
			tp_prox_enable_show, tp_prox_enable_store);

/* Returns currently selected poll interval (in ms) */
static ssize_t tp_prox_get_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{

	return sprintf(buf, "%d\n", input_poll_dev->poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t tp_prox_set_poll_delay(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct input_dev *input_dev = input_prox_dev;
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	/*
	 * Set current interval to the greater of the minimum interval or
	 * the requested interval
	 */
	input_poll_dev->poll_interval = max((int)interval,(int)input_poll_dev->poll_interval_min);

	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(poll_delay, SYSFS_AUTHORITY,
			tp_prox_get_poll_delay, tp_prox_set_poll_delay);

static struct attribute *tp_prox_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group tp_prox_attribute_group = {
	.attrs = tp_prox_attributes,
};

static void __init tp_prox_init_input_device(struct input_polled_dev * poll_dev,
		struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	input_dev->name = PROXIMITY_INPUT_DEV_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &i2c_prox_client->dev;
}

static void tp_prox_poll(struct input_polled_dev *dev)
{

	if (tp_prox_sensor_data_changed){
		mutex_lock(&tp_prox_sensor_mutex);
		
		tp_prox_sensor_data_changed = 0;
		mutex_unlock(&tp_prox_sensor_mutex);
		pr_info("%s poll tp_prox_sensor_data=%d\n",__func__,tp_prox_sensor_data);
		input_report_abs(input_prox_dev, ABS_DISTANCE, tp_prox_sensor_data);
		input_sync(input_prox_dev);
	}
}

static int __init tp_prox_setup_polled_device(void)
{
	int err;
	struct input_polled_dev *poll_dev;

	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		TP_DEBUG(" Failed to allocate polled device\n");
		return -ENOMEM;
	}

	input_poll_dev = poll_dev;
	input_prox_dev = poll_dev->input;

	poll_dev->private = input_poll_dev;
	poll_dev->poll = tp_prox_poll;
	poll_dev->poll_interval = 100;
	poll_dev->poll_interval_min= 0;

	tp_prox_init_input_device(input_poll_dev, poll_dev->input);
	err = input_register_polled_device(poll_dev);
	pr_info("%s, err=%d, poll-interval=%d\n",__func__,err,poll_dev->poll_interval);
	if (err) {

		TP_DEBUG(" Unable to register polled device, err=%d\n",err);
		input_free_polled_device(poll_dev);
		return -ENOMEM;
	}

	return 0;
}

static void tp_prox_teardown_polled_device(void)
{
	input_unregister_polled_device(input_poll_dev);
	input_free_polled_device(input_poll_dev);
}
#endif//CONFIG_TOUCHPANEL_PROXIMITY_SENSOR

#ifdef MSG_SOFT_VERSION_ID //20140704

static unsigned short curr_ic_major=0;
static unsigned short curr_ic_minor=0;

static u16 _msg_GetVersion_MoreTime(void)
{
    int version_check_time = 0,ret = 0;
    for(version_check_time=0;version_check_time<2;version_check_time++)
    {
		ret = Read_FW_Version();
		if(ret == 0)
	    {
	    	curr_ic_major = mstar_module_name;
	    	curr_ic_minor = mstar_firmware_version;
	    }
	    else
	    {
	    	curr_ic_major = 0xffff;
	    	curr_ic_minor = 0xffff;
	    }
	    TP_DEBUG("version_check_time=%d;curr_ic_major=%d;curr_ic_minor=%d \n",version_check_time,curr_ic_major,curr_ic_minor);
	    if(TP_OF_YUSHUN==curr_ic_major
			|| TP_OF_BOYI==curr_ic_major
			|| TP_OF_YEJI==curr_ic_major
	       )
	    {
	        break;
	    }
	    else if(version_check_time<3)
	    {
	        msleep(100);
	    }
	    else
	    {
            _HalTscrHWReset();
        }
    }
    return curr_ic_major;
}
static u16 _msg_GetVendorID_fromIC(void)
{
    u8 i;
    u16 vendor_id=0;
    u8  dbbus_tx_data[5]={0};
    u8  dbbus_rx_data[4]={0};
    u16 reg_data=0;
    u16 addr = 0;
	unsigned long timeout = 0;
	int ret = 0;

	lct_qup_i2c_use_custom_clk(msg21xx_i2c_client->adapter, 50000);
	
    _HalTscrHWReset();
    ret = dbbusDWIICEnterSerialDebugMode();
	if (ret < 0)
	{
		goto Err_DebugMode;
	}
    dbbusDWIICStopMCU();
    dbbusDWIICIICUseBus();
    dbbusDWIICIICReshape();     
    msleep ( 100 );
	
    //Stop MCU
    drvDB_WriteReg(0x0F, 0xE6, 0x0001);
	
    // Stop Watchdog
    drvDB_WriteReg8Bit( 0x3C, 0x60, 0x55);
    drvDB_WriteReg8Bit( 0x3C, 0x61, 0xAA);

    //cmd	
    drvDB_WriteReg( 0x3C, 0xE4, 0xA4AB);
    drvDB_WriteReg(0x1E, 0x04, 0x7d60);

    // TP SW reset
    drvDB_WriteReg( 0x1E, 0x04, 0x829F );

    //MCU run	
    drvDB_WriteReg(0x0F, 0xE6, 0x0000 );

   //polling 0x3CE4
    timeout = jiffies + HZ/2;	// xuke @ 20140711	Add timeout mechanism.
    do
    {
        reg_data = drvDB_ReadReg( 0x3C, 0xE4);
		msleep(10);
    }
    while (( reg_data != 0x5B58 ) && (time_before(jiffies, timeout)));

    addr = 0x8300;

    dbbus_tx_data[0] = 0x72;
    dbbus_tx_data[1] = (addr>>8)&0xFF;
    dbbus_tx_data[2] = addr&0xFF;
    dbbus_tx_data[3] = 0x00;
    dbbus_tx_data[4] = 0x04;
    HalTscrCDevWriteI2CSeq ( FW_ADDR_MSG21XX_TP , &dbbus_tx_data[0], 5 );
    // recive info data
    HalTscrCReadI2CSeq( FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0], 4 );

    for(i=0;i<4;i++)
    {
		TP_DEBUG("[21xxA]:Vendor id dbbus_rx_data[%d]=0x%x,%d\n",i,dbbus_rx_data[i],(dbbus_rx_data[i]-0x30));
    }

    if((dbbus_rx_data[0]>=0x30 && dbbus_rx_data[0]<=0x39)
    &&(dbbus_rx_data[1]>=0x30 && dbbus_rx_data[1]<=0x39)
    &&(dbbus_rx_data[2]>=0x31 && dbbus_rx_data[2]<=0x39))  
    {
    	vendor_id=(dbbus_rx_data[0]-0x30)*100+(dbbus_rx_data[1]-0x30)*10+(dbbus_rx_data[2]-0x30);
    }
    else 
    {
        vendor_id = 0xffff;
    }
    TP_DEBUG("vendor_id = %d\n",vendor_id);
    dbbusDWIICIICNotUseBus();
    dbbusDWIICNotStopMCU();
    dbbusDWIICExitSerialDebugMode();
    _HalTscrHWReset();

	lct_qup_i2c_use_custom_clk(msg21xx_i2c_client->adapter, 100000);

Err_DebugMode:
    return vendor_id;
}

unsigned char *update_bin = NULL;
static int _msg_auto_updateFirmware(void)
{
    U16 vendor_id = 0;
    unsigned short update_bin_major=0;
    unsigned short update_bin_minor=0;
	U16 VendorID_OF_YUSHUN = FW_YuShun[0x7f4f]<<8|FW_YuShun[0x7f4e];
	U16 TpType_OF_YUSHUN = FW_YuShun[0x7f4f]<<8|FW_YuShun[0x7f4e];
	U16 VendorID_OF_BOYI = FW_BoYi[0x7f4f]<<8|FW_BoYi[0x7f4e];
	U16 TpType_OF_BOYI = FW_BoYi[0x7f4f]<<8|FW_BoYi[0x7f4e];
	U16 VendorID_OF_YEJI = FW_YeJi[0x7f4f]<<8|FW_YeJi[0x7f4e];
	U16 TpType_OF_YEJI = FW_YeJi[0x7f4f]<<8|FW_YeJi[0x7f4e];

    _msg_GetVersion_MoreTime();

	TP_DEBUG("curr_ic_major=0x%x, curr_ic_minor=0x%x\n", curr_ic_major, curr_ic_minor);
    if(curr_ic_major==0xffff && curr_ic_minor==0xffff)
    {
        vendor_id = _msg_GetVendorID_fromIC();
        //modify :当前项目使用各家tp厂VendorID，必须根据项目实际情况增减修改
        TP_DEBUG("vendor_id=%d, VendorID_OF_YUSHUN=%d, VendorID_OF_BOYI=%d, VendorID_OF_YEJI=%d\n", 
        	vendor_id, VendorID_OF_YUSHUN, VendorID_OF_BOYI, VendorID_OF_YEJI);
        if(VendorID_OF_YUSHUN==vendor_id)
        {
            update_bin = FW_YuShun;
            return 1;
        }
        else if(VendorID_OF_BOYI==vendor_id)
        {
            update_bin = FW_BoYi;
            return 1;
        }
        else if(VendorID_OF_YEJI==vendor_id)
        {
            update_bin = FW_YeJi;
            return 1;
        }
        else
        {
            TP_DEBUG("AUTO_UPDATE choose VendorID failed,VendorID=%d\n",vendor_id);
            return 0;
        }
    }
    else 
    {
         //modify :当前项目使用各家tp厂，必须根据项目实际情况增减修改
       TP_DEBUG("curr_ic_major=%d, TpType_OF_YEJI=%d, TpType_OF_LIHE=%d\n", 
       		curr_ic_major, TpType_OF_YUSHUN, TpType_OF_BOYI);

        if(TpType_OF_YUSHUN==curr_ic_major)
        {
            update_bin = FW_YuShun;
        }
        else if(TpType_OF_BOYI==curr_ic_major)
        {
            update_bin = FW_BoYi;
        }
        else if(TpType_OF_YEJI==curr_ic_major)
        {
            update_bin = FW_YeJi;
        }
        else
        {
            TP_DEBUG("AUTO_UPDATE choose tptype failed,curr_ic_major=%d\n",curr_ic_major);
            return 0;
        }

        update_bin_major = update_bin[0x7f4f]<<8|update_bin[0x7f4e];
        update_bin_minor = update_bin[0x7f51]<<8|update_bin[0x7f50];

        if(update_bin_major==curr_ic_major && update_bin_minor>curr_ic_minor)
        {
            TP_DEBUG("need auto update curr_ic_major=%d;\n curr_ic_minor=%d;\n update_bin_major=%d;\n update_bin_minor=%d;\n",curr_ic_major,curr_ic_minor,update_bin_major,update_bin_minor);
            return 1;
        }
        else
        {
            TP_DEBUG("do not auto update curr_ic_major=%d;\n curr_ic_minor=%d;\n update_bin_major=%d;\n update_bin_minor=%d;\n",curr_ic_major,curr_ic_minor,update_bin_major,update_bin_minor);
            return 0;
        }
    }
}
#endif

extern int is_tp_driver_loaded;
extern char android_boot_mode[];
//extern int msg2133_virtualkey_init(void); 
static int msg21xx_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned int irq;
	int  err = 0;
#if __MSG2133A_FIRMWARE_UPDATE__
	int isNeedupdate = 0;
	int nK = 0;
	int i = 0;
	int fwversion = 0;
#endif
#ifdef SUPPORT_READ_TP_VERSION
    char tp_version[60] = {0};
	char tp_vendor[8];
#endif

	printk("%s, is_tp_driver_loaded=%d\n", __func__, is_tp_driver_loaded);
	if(is_tp_driver_loaded == 1)
	{
		printk(KERN_ERR"msg21xx_ts_probe other driver has been loaded\n");
		return ENODEV;
	}

	// xuke @ 20140911	Do not need to enter probe if power on by charging.
	if (!strncmp(android_boot_mode, "charger", strlen("charger"))||!strcmp(android_boot_mode,"recovery"))
	{
		printk("%s, boot mode %s\n", __func__,android_boot_mode);
		return ENODEV;
	}
	
    if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct msg21xx_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = msg21xx_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			goto Err_parse_dt;
		}
	} else
		pdata = client->dev.platform_data;

	msg21xx_i2c_client = client;
    msg2133_ts = devm_kzalloc(&client->dev,
			sizeof(struct msg21xx_ts_data), GFP_KERNEL);
	if (!msg2133_ts) {
		dev_err(&client->dev, "Not enough memory\n");
		goto Err_kzalloc_msg2133_ts;
	}

    msg2133_ts->client = client;
	msg2133_ts->pdata = pdata;
    i2c_set_clientdata(client, msg2133_ts);
	
	err = msg21xx_pinctrl_init(msg2133_ts);
	if (!err && msg2133_ts->ts_pinctrl) {
		err = msg21xx_pinctrl_select(msg2133_ts, true);
		if (err < 0)
			goto free_pinctrl;
	}
	
	err = msg21xx_power_init(msg2133_ts, true);
	if (err) {
		dev_err(&client->dev, "power init failed");
		goto exit_irq_request_failed;
	}

	err = msg21xx_power_on(msg2133_ts, true);
	if (err) {
		dev_err(&client->dev, "power on failed");
		goto pwr_deinit;
	}
	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio, "msg21xx_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto pwr_off;
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}

	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, "msg21xx_reset_gpio");
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 1);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		gpio_set_value(pdata->reset_gpio, 1);
	}

	/* make sure CTP already finish startup process */
	msleep(100);
    
//	msg21xx_chip_init();
	
#if 1
	err = msg21xx_setic_flag();
	if (err < 0)
	{
		printk("%s, msg21xx_setic_flag ERROR\n", __func__);
	}
	else
	{
		_HalTscrHWReset();
		err = Read_FW_Version(); 
	}
#if __MSG2133A_FIRMWARE_UPDATE__
	if(is_msg2133A == 1)
	{
		if(is_msg2133A_funa == 1)
		{
//			printk("---TP module is msg2133a Funa\n");
			
			nK = sizeof(FW_FuNa)/1024;
			//printk("---FN tp fw nK =%d\n",nK);
			
			for(i = 0;i <nK;i++)
			{
				memcpy(temp[i], FW_FuNa+i*1024, 1024);
			}
			//printk("%s: msg21xx_ts_update,FW_FuNa[0x800a] = %d, FW_FuNa[0x800b] = %d\n", __func__,FW_FuNa[0x800a],FW_FuNa[0x800b]);
			//fwversion = FW_FuNa[0x800a]<<8|FW_FuNa[0x800b];
			fwversion = FW_FuNa[0x800a];
			printk("---TP module is msg2133a Funa, fwversion=%d\n",fwversion);
			
		}
		else if(mstar_module_name == TP_OF_YUSHUN)		// YuShun
		{
			nK = sizeof(FW_YuShun)/1024;
			//printk("---cs tp fw nK =%d\n",nK);
			
			for(i = 0;i <nK;i++)
			{
				memcpy(temp[i], FW_YuShun+i*1024, 1024);
			}
//			printk("%s: msg21xx_ts_update,FW_YuShun[0x7f51] = %d, FW_YuShun[0x7f50] = %d\n", __func__,FW_YuShun[0x7f51],FW_YuShun[0x7f50]);
			fwversion = FW_YuShun[0x7f51]<<8|FW_YuShun[0x7f50];
			printk("---TP module is msg2133a YuShun, fwversion=%03d\n",fwversion);
			
		}
        else if(mstar_module_name == TP_OF_BOYI)		// BoYi
        {
			nK = sizeof(FW_BoYi)/1024;
			//printk("---cs tp fw nK =%d\n",nK);
			
			for(i = 0;i <nK;i++)
			{
				memcpy(temp[i], FW_BoYi+i*1024, 1024);
			}
//			printk("%s: msg21xx_ts_update,FW_BoYi[0x7f51] = %d, FW_BoYi[0x7f50] = %d\n", __func__,FW_BoYi[0x7f51],FW_BoYi[0x7f50]);
			fwversion = FW_BoYi[0x7f51]<<8|FW_BoYi[0x7f50];
			printk("---TP module is msg2133a BoYi, fwversion=%03d\n",fwversion);
        }
		else if(mstar_module_name == TP_OF_YEJI) 	// YeJi
		{
			nK = sizeof(FW_YeJi)/1024;
			//printk("---cs tp fw nK =%d\n",nK);
			
			for(i = 0;i <nK;i++)
			{
				memcpy(temp[i], FW_YeJi+i*1024, 1024);
			}
//			printk("%s: msg21xx_ts_update,FW_YeJi[0x7f51] = %d, FW_YeJi[0x7f50] = %d\n", __func__,FW_YeJi[0x7f51],FW_YeJi[0x7f50]);
			fwversion = FW_YeJi[0x7f51]<<8|FW_YeJi[0x7f50];
			printk("---TP module is msg2133a YeJi, fwversion=%03d\n",fwversion);
		}
	}
	
#endif

	if(err != 0)
	{
#ifdef MSG_SOFT_VERSION_ID   //add 20140704
		isNeedupdate = _msg_auto_updateFirmware();
        if(isNeedupdate)
        {
			for (FwDataCnt=0; FwDataCnt<sizeof(FW_YuShun)/1024; FwDataCnt++)
			{
				memcpy(temp[FwDataCnt], update_bin + FwDataCnt*1024, 1024);
				printk("%s, FwDataCnt=%d, temp=%d\n", __func__, FwDataCnt, temp[FwDataCnt][0]);
			}
        }
		else
#endif
		goto Err_Read_FW_Version;
	}
#endif

#ifdef SUPPORT_READ_TP_VERSION
	memset(tp_version, 0, sizeof(tp_version));
	memset(tp_vendor, 0, sizeof(tp_vendor));
	switch (mstar_module_name)
	{
		case TP_OF_YUSHUN:
			sprintf(tp_vendor, "YuShun");
			break;
		case TP_OF_BOYI:
			sprintf(tp_vendor, "BoYi");
			break;
		case TP_OF_YEJI:
			sprintf(tp_vendor, "YeJi");
			break;
		default:
			sprintf(tp_vendor, "Unknown");
			break;
	}
#if __FIRMWARE_AUTO_UPDATE__ || __FIRMWARE_FORCE_UPDATE__
    sprintf(tp_version, "vid:0x%04x,fw:0x%04x,ic:%s\n",mstar_module_name,mstar_firmware_version,"msg2133,open");
#else
    sprintf(tp_version, "vid:0x%04x,fw:0x%04x,ic:%s\n",mstar_module_name,mstar_firmware_version,"msg2133");
#endif
    init_tp_fm_info(0, tp_version, tp_vendor);
#endif
#if __FIRMWARE_AUTO_UPDATE__
	 if(is_msg2133A && ((mstar_module_name==TP_OF_YUSHUN) || (mstar_module_name==TP_OF_BOYI) || (mstar_module_name==TP_OF_YEJI))
	 	&& ((mstar_firmware_version<fwversion) || (mstar_firmware_version == 0x8)))
	 {
		
		//firmware_update_store(NULL,NULL,NULL,0);
		isNeedupdate = 1; 	
	 }
	 else
	 {
		printk(KERN_ERR "TP do not need update\n");
	 }
	#endif	 
#if __FIRMWARE_FORCE_UPDATE__
	// firmware_update_store(NULL,NULL,NULL,0);
	if(is_msg2133A == 1)
	{
		isNeedupdate = 1;
	}
	
#endif	 

	INIT_WORK(&msg21xx_wq, msg21xx_do_work);
	 
#ifdef MSTAR_USE_VIRTUALKEY
	 //msg2133_virtualkey_init(); 
#endif
	msg21xx_init_input();

	irq = gpio_to_irq(pdata->irq_gpio);
	msg21xx_irq = irq;
	printk("[ PORTING MSG]%s irq =%d\n",__func__, irq);

	err = request_irq(irq, msg21xx_interrupt, 
								IRQF_TRIGGER_FALLING, "msg21xx", NULL);
	if (err != 0) {
		printk("%s: cannot register irq\n", __func__);
		goto Err_request_irq;
	}

#if __MSG2133A_FIRMWARE_UPDATE__

	  firmware_class = class_create(THIS_MODULE, "ms-touchscreen-msg20xx");
    if (IS_ERR(firmware_class))
     pr_err("Failed to create class(firmware)!\n");
     
    firmware_cmd_dev = device_create(firmware_class,
                                     NULL, 0, NULL, "device");
    if (IS_ERR(firmware_cmd_dev))
     pr_err("Failed to create device(firmware_cmd_dev)!\n");
    // version
    if (device_create_file(firmware_cmd_dev, &dev_attr_version) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_version.attr.name);
    // update
    if (device_create_file(firmware_cmd_dev, &dev_attr_update) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_update.attr.name);
    // data
    if (device_create_file(firmware_cmd_dev, &dev_attr_data) < 0)
        pr_err("Failed to create device file(%s)!\n", dev_attr_data.attr.name);

//    if (device_create_file(firmware_cmd_dev, &dev_attr_clear) < 0)
  //      pr_err("Failed to create device file(%s)!\n", dev_attr_clear.attr.name);
  
    dev_set_drvdata(firmware_cmd_dev, NULL);
#endif
#ifdef  MSG_GESTURE_FUNCTION
	// DoubleClick
	if (device_create_file(firmware_cmd_dev, &dev_attr_doubleclick) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_doubleclick.attr.name);
	
	// UpDirect
	if (device_create_file(firmware_cmd_dev, &dev_attr_updirect) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_updirect.attr.name);

	// DownDirect
	if (device_create_file(firmware_cmd_dev, &dev_attr_downdirect) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_downdirect.attr.name);

	// LeftDirect
	if (device_create_file(firmware_cmd_dev, &dev_attr_leftdirect) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_leftdirect.attr.name);

	// RightDirect
	if (device_create_file(firmware_cmd_dev, &dev_attr_rightdirect) < 0)
		pr_err("Failed to create device file(%s)!\n", dev_attr_rightdirect.attr.name);
#endif

	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	#ifdef SENSOR_PROX_TP_USE_WAKELOCK
	wake_lock_init(&sensor_prox_tp_wake_lock, WAKE_LOCK_SUSPEND, "tp_sensor");
	#endif

	i2c_prox_client = client;

	err = sysfs_create_group(&client->dev.kobj, &tp_prox_attribute_group);
	if (err) {
		TP_DEBUG( " sysfs create failed\n");  
	}

	err = tp_prox_setup_polled_device();
	if (err < 0)
	{   
       TP_DEBUG( " failed to setup polled device\n");  
	}
	#endif

#ifdef MSG_ITO_TEST
	ito_test_create_entry();
	lct_ctp_selftest_int(msg2133_chip_self_test);
#endif

#ifdef TP_PRINT
    mutex_init(&tpp_lock);
    tp_print_create_entry();
#endif

#ifdef LCT_UPGRADE_MSG2133
	lct_ctp_upgrade_int(msg2133_ctp_upgrade_func,msg2133_ctp_upgrade_read_ver_func);
#endif



#if defined(CONFIG_FB)
	fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&fb_notif);
	if (err)
		TP_DEBUG("Unable to register fb_notifier: %d\n",err);
#endif

	//early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN;
	//early_suspend.suspend = msg21xx_ts_early_suspend;
	//early_suspend.resume = msg21xx_ts_late_resume;
	//register_early_suspend(&early_suspend);
	
	
	#ifdef CYTTSP_SUPPORT_READ_TP_VERSION
	misc_register(&tp_version_device);
	#endif



#if 0
	gpio_set_value(pdata->reset_gpio, 0);
    pr_err(" MSG21XX_RESET_GPIO after set gpio");
	msleep(200);
	gpio_set_value(pdata->reset_gpio, 1);
	msleep(500);
#else
	_HalTscrHWReset();
#endif
   

	#if __MSG2133A_FIRMWARE_UPDATE__
	update_firmware_tsk = kthread_create( update_func, update_firmware_tsk, "update_firmware");
	if (IS_ERR(update_firmware_tsk)) 
	{
		kfree(update_firmware_tsk);
#if 0
		return -EINVAL;
#else
		goto Err_update_firmware_tsk;
#endif
	}
	if(isNeedupdate)//close tp auto update
	{
		wake_up_process(update_firmware_tsk);
	}
	#endif
    
	is_tp_driver_loaded = 1;
	return 0;

Err_update_firmware_tsk:
Err_request_irq:
Err_Read_FW_Version:
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
	{
		printk("%s, free_irq_gpio, irq_gpio=%d\n", __func__, pdata->irq_gpio);
		gpio_free(pdata->irq_gpio);
	}
pwr_off:
	msg21xx_power_on(msg2133_ts, false);
pwr_deinit:
	msg21xx_power_init(msg2133_ts, false);
exit_irq_request_failed:
	if (msg2133_ts->ts_pinctrl) {
		msg21xx_pinctrl_select(msg2133_ts, false);
	}
free_pinctrl:
	if (msg2133_ts->ts_pinctrl)
		pinctrl_put(msg2133_ts->ts_pinctrl);
	i2c_set_clientdata(client, NULL);
	devm_kfree(&client->dev, msg2133_ts);
Err_kzalloc_msg2133_ts:
Err_parse_dt:
	devm_kfree(&client->dev, pdata);
	printk("%s done, err=%d\n", __func__, err);
	return err;
}

#if defined(CONFIG_FB)
static void msg2133_ts_suspend(void)
{
#ifdef MSG_GESTURE_FUNCTION
	 int msg_gesturemoderetval = 0, gesture_mode = 0, timeout = 3;
#endif

#if __MSG2133A_FIRMWARE_UPDATE__
	if(FwDataCnt)
	{
		return;
	}
#endif
#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	if(tp_prox_sensor_opened == 1)
	{
		is_need_report_pointer = 0;
		printk(KERN_INFO "[mstar] %s:  tp can not sleep in call\n",__func__);
		return;
	}
#endif	
#ifdef MSG_GESTURE_FUNCTION
	gesture_mode = msg_GetGestureModeValue();
	if(gesture_mode)
	{
		
		if((gesture_mode >= 0x01)&&(gesture_mode <= 0x1F))
		{
			while((!msg_gesturemoderetval) && (--timeout))
			{
				msg_gesturemoderetval = msg_OpenGestureFunction(gesture_mode);
			} ////be sure enter this function

			if (!timeout && !msg_gesturemoderetval)
			{
				printk("xuke: %s, failed to enter gesture mode!\n", __func__);
			}
			else
			{
				printk("xuke: %s, enter gesture mode 0x%x\n", __func__, gesture_mode);
			}
			return;
		}
		else
		{
			////command wrong!
			printk("%s, gesture_mode wrong!\n", __func__);
		}
	}
#endif

	gpio_set_value(pdata->reset_gpio, 0);
	disable_irq_nosync(msg21xx_irq);
    is_suspend = 1;
}

static void msg2133_ts_resume(void)
{
	//printk("%s: enter\n", __func__);
#ifdef MSG_GESTURE_FUNCTION
	msg_SetGestureFlag(0);
#endif
	//msleep(100);
	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	is_need_report_pointer = 1;
	printk(KERN_INFO "[mstar] %s:  tp_sensor_opened=%d,is_suspend=%d\n",__func__,tp_prox_sensor_opened,is_suspend);
	if((tp_prox_sensor_opened == 1) && (is_suspend == 0))
	{
		printk(KERN_INFO "[mstar] %s:  tp no need to wake up in call\n",__func__);
		return ;
	}
	#endif
	if(1)  // the action is in lcd_panel on
	{
		gpio_set_value(pdata->reset_gpio, 1);
		msleep(30);
	}
#if defined(__MSG2133A_FIRMWARE_UPDATE__)		// xuke @ 20140708		Can not enable irq during updating fw.
	if (!fw_in_updating_flag)
#endif
	enable_irq(msg21xx_irq);
	//msleep(100); 
	is_suspend = 0;

}
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	
	if (evdata && evdata->data && event == FB_EVENT_BLANK ){
		blank = evdata->data;
		TP_DEBUG("fb_notifier_callback blank=%d\n",*blank);
		if (*blank == FB_BLANK_UNBLANK)
			msg2133_ts_resume();
		else if (*blank == FB_BLANK_POWERDOWN)
			msg2133_ts_suspend();
	}

	return 0;
}

#endif


static int msg21xx_ts_remove(struct i2c_client *client)
{
	#ifdef CYTTSP_SUPPORT_READ_TP_VERSION
	misc_deregister(&tp_version_device);
	#endif
	
	#ifdef MSTAR_TP_USE_WAKELOCK
		wake_lock_destroy(&mstar_tp_wake_lock);
	#endif
    #ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
    sysfs_remove_group(&client->dev.kobj, &tp_prox_attribute_group);
    tp_prox_teardown_polled_device();
    #ifdef SENSOR_PROX_TP_USE_WAKELOCK
    wake_lock_destroy(&sensor_prox_tp_wake_lock);
    #endif
#endif

	#if defined(CONFIG_FB)
		if (fb_unregister_client(&fb_notif))
			TP_DEBUG("Error occurred while unregistering fb_notifier.\n");
	#endif


	
	return 0;
}

static struct of_device_id msg2133_match_table[] = {
	{ .compatible = "mstar,msg2133",},
	{ },
};


static const struct i2c_device_id msg21xx_ts_id[] = {
	{ "ms-msg20xx", TOUCH_ADDR_MSG21XX },
	{ }
};
MODULE_DEVICE_TABLE(i2c, msg21xx_ts_id);


static struct i2c_driver msg21xx_ts_driver = {
	.driver = {
		.name = "ms-msg20xx",
		.owner = THIS_MODULE,
		.of_match_table = msg2133_match_table,
	},
	.probe = msg21xx_ts_probe,
	.remove = msg21xx_ts_remove,
	//.suspend = msg21xx_ts_suspend,
	//.resume = msg21xx_ts_resume,
	.id_table = msg21xx_ts_id,
};
/*
void timer_func(unsigned long data)
{
	printk("time[%d]\n",(int)jiffies);
	mod_timer(&my_timer,jiffies+2*HZ);
}

static int test_init(void)
{
	init_timer(&my_timer);
	my_timer.expires = jiffies + 5*HZ;
	my_timer.function= timer_func;
	my_timer.data = 99;

	printk("time[%d]\n",(int)jiffies);
	mdelay(1000);
	printk("time[%d]\n",(int)jiffies);
	add_timer(&my_timer);
	printk("start\n");
	printk("hello kernel\n");
	return 0;
}
*/





static int __init msg21xx_init(void)
{
#ifdef   TP_DIFF
	int err;
	int ret;
	gpio_request(FLAG_GPIO, "flag_msg21xx");
	gpio_direction_input(FLAG_GPIO);
	udelay(10);
	ret = gpio_get_value(FLAG_GPIO);
	//printk("flag_gpio sitronix init ret=%d\n", ret);
	if(0 == ret)
	{
		udelay(10);
		ret = gpio_get_value(FLAG_GPIO);
		ret = 1;
		//printk("flag_gpio sitronix init ret=%d\n", ret);
		if (0 == ret)
		{
			gpio_free(FLAG_GPIO);

			mutex_init(&msg21xx_mutex);
			err = i2c_add_driver(&msg21xx_ts_driver);
			if (err) {
				printk(KERN_WARNING "msg21xx  driver failed "
				       "(errno = %d)\n", err);
			} else {
				printk( "Successfully added driver %s\n",
				          msg21xx_ts_driver.driver.name);
			}
			return err;
		}
		else 
		{
			gpio_free(FLAG_GPIO); 
		}
	}
	else
	{
		gpio_free(FLAG_GPIO); 
		return -1;
	}
#else
	int err;
	mutex_init(&msg21xx_mutex);
	err = i2c_add_driver(&msg21xx_ts_driver);
	if (err) {
		printk(KERN_WARNING "msg21xx  driver failed "
		       "(errno = %d)\n", err);
	} else {
		printk( "Successfully added driver %s\n",
		          msg21xx_ts_driver.driver.name);
	}

	
	return err;
#endif
}

static void __exit msg21xx_cleanup(void)
{
	i2c_del_driver(&msg21xx_ts_driver);
	mutex_destroy(&msg21xx_mutex);
}


module_init(msg21xx_init);
module_exit(msg21xx_cleanup);

#ifdef MSG_ITO_TEST
//modify:根据项目修改
#include "./msg2133_selftest/open_test_ANA1_yushun.h"
#include "./msg2133_selftest/open_test_ANA2_yushun.h"
#include "./msg2133_selftest/open_test_ANA1_B_yushun.h"
#include "./msg2133_selftest/open_test_ANA2_B_yushun.h"
#include "./msg2133_selftest/open_test_ANA3_yushun.h"

#include "./msg2133_selftest/open_test_ANA1_boyi.h"
#include "./msg2133_selftest/open_test_ANA2_boyi.h"
#include "./msg2133_selftest/open_test_ANA1_B_boyi.h"
#include "./msg2133_selftest/open_test_ANA2_B_boyi.h"
#include "./msg2133_selftest/open_test_ANA3_boyi.h"

///////////////////////////////////////////////////////////////////////////
u8 bItoTestDebug = 1;
#define ITO_TEST_DEBUG(format, ...) \
{ \
    if(bItoTestDebug) \
    { \
        printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__); \
        mdelay(5); \
    } \
}
#define ITO_TEST_DEBUG_MUST(format, ...)	printk(KERN_ERR "ito_test ***" format "\n", ## __VA_ARGS__);mdelay(5)


u16  Th_Tri = 25;        
u16  Th_bor = 25;    

s16  s16_raw_data_1[48] = {0};
s16  s16_raw_data_2[48] = {0};
s16  s16_raw_data_3[48] = {0};
u8 ito_test_keynum = 0;
u8 ito_test_dummynum = 0;
u8 ito_test_trianglenum = 0;
u8 ito_test_2r = 0;
u8 g_LTP = 1;	
uint16_t *open_1 = NULL;
uint16_t *open_1B = NULL;
uint16_t *open_2 = NULL;
uint16_t *open_2B = NULL;
uint16_t *open_3 = NULL;
u8 *MAP1 = NULL;
u8 *MAP2=NULL;
u8 *MAP3=NULL;
u8 *MAP40_1 = NULL;
u8 *MAP40_2 = NULL;
u8 *MAP40_3 = NULL;
u8 *MAP40_4 = NULL;
u8 *MAP41_1 = NULL;
u8 *MAP41_2 = NULL;
u8 *MAP41_3 = NULL;
u8 *MAP41_4 = NULL;

#define ITO_TEST_ADDR_TP  (0x4C>>1)
#define ITO_TEST_ADDR_REG (0xC4>>1)
#define REG_INTR_FIQ_MASK           0x04
#define FIQ_E_FRAME_READY_MASK      ( 1 << 8 )
#define MAX_CHNL_NUM (48)
#define BIT0  (1<<0)
#define BIT1  (1<<1)
#define BIT5  (1<<5)
#define BIT11 (1<<11)
#define BIT15 (1<<15)

static int ito_test_i2c_read(U8 addr, U8* read_data, U16 size)//modify : 根据项目修改 msg21xx_i2c_client
{
    int rc;
    U8 addr_before = msg21xx_i2c_client->addr;
    msg21xx_i2c_client->addr = addr;

    #ifdef DMA_IIC
    if(size>8&&NULL!=I2CDMABuf_va)
    {
        int i = 0;
        msg21xx_i2c_client->ext_flag = msg21xx_i2c_client->ext_flag | I2C_DMA_FLAG ;
        rc = i2c_master_recv(msg21xx_i2c_client, (unsigned char *)I2CDMABuf_pa, size);
        for(i = 0; i < size; i++)
   		{
        	read_data[i] = I2CDMABuf_va[i];
    	}
    }
    else
    {
        rc = i2c_master_recv(msg21xx_i2c_client, read_data, size);
    }
    msg21xx_i2c_client->ext_flag = msg21xx_i2c_client->ext_flag & (~I2C_DMA_FLAG);	
    #else
    rc = i2c_master_recv(msg21xx_i2c_client, read_data, size);
    #endif

    msg21xx_i2c_client->addr = addr_before;
    if( rc < 0 )
    {
        ITO_TEST_DEBUG_MUST("ito_test_i2c_read error %d,addr=%d\n", rc,addr);
    }
    return rc;
}

static int ito_test_i2c_write(U8 addr, U8* data, U16 size)//modify : 根据项目修改 msg21xx_i2c_client
{
    int rc;
    U8 addr_before = msg21xx_i2c_client->addr;
    msg21xx_i2c_client->addr = addr;

#ifdef DMA_IIC
    if(size>8&&NULL!=I2CDMABuf_va)
	{
	    int i = 0;
	    for(i=0;i<size;i++)
    	{
    		 I2CDMABuf_va[i]=data[i];
    	}
		msg21xx_i2c_client->ext_flag = msg21xx_i2c_client->ext_flag | I2C_DMA_FLAG ;
		rc = i2c_master_send(msg21xx_i2c_client, (unsigned char *)I2CDMABuf_pa, size);
	}
	else
	{
		rc = i2c_master_send(msg21xx_i2c_client, data, size);
	}
    msg21xx_i2c_client->ext_flag = msg21xx_i2c_client->ext_flag & (~I2C_DMA_FLAG);	
#else
    rc = i2c_master_send(msg21xx_i2c_client, data, size);
#endif

    msg21xx_i2c_client->addr = addr_before;
    if( rc < 0 )
    {
        ITO_TEST_DEBUG_MUST("ito_test_i2c_write error %d,addr = %d,data[0]=%d\n", rc, addr,data[0]);
    }
    return rc;
}

static void ito_test_reset(void)//modify:根据项目修改
{

	gpio_direction_output(pdata->reset_gpio, 1);
  gpio_set_value(pdata->reset_gpio, 1);
	gpio_set_value(pdata->reset_gpio, 0);
	mdelay(100); 
    ITO_TEST_DEBUG("reset tp\n");
	gpio_set_value(pdata->reset_gpio, 1);
	mdelay(200);
}
static void ito_test_disable_irq(void)//modify:根据项目修改
{
	disable_irq_nosync(msg21xx_irq);
}
static void ito_test_enable_irq(void)//modify:根据项目修改
{
	enable_irq(msg21xx_irq);
}

static void ito_test_set_iic_rate(bool enable, u32 iicRate)//modify:根据平台修改,iic速率要求50K
{

	ITO_TEST_DEBUG("ito_test_set_iic_rate \n");

	#if 1	// qc
		lct_qup_i2c_use_custom_clk(msg21xx_i2c_client->adapter, iicRate);
	#endif
	
	#ifdef SPRD//展讯平台
        sprd_i2c_ctl_chg_clk(msg21xx_i2c_client->adapter->nr, iicRate);
        mdelay(100);
	#endif
    #ifdef MTK//MTK平台
        msg21xx_i2c_client->timing = iicRate/1000;
    #endif
}

static void ito_test_WriteReg( u8 bank, u8 addr, u16 data )
{
    u8 tx_data[5] = {0x10, bank, addr, data & 0xFF, data >> 8};
    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 5 );
}
static void ito_test_WriteReg8Bit( u8 bank, u8 addr, u8 data )
{
    u8 tx_data[4] = {0x10, bank, addr, data};
    ito_test_i2c_write ( ITO_TEST_ADDR_REG, &tx_data[0], 4 );
}
static unsigned short ito_test_ReadReg( u8 bank, u8 addr )
{
    u8 tx_data[3] = {0x10, bank, addr};
    u8 rx_data[2] = {0};

    ito_test_i2c_write( ITO_TEST_ADDR_REG, &tx_data[0], 3 );
	mdelay(20);
    ito_test_i2c_read ( ITO_TEST_ADDR_REG, &rx_data[0], 2 );
    return ( rx_data[1] << 8 | rx_data[0] );
}
static u32 ito_test_get_TpType(void)
{
    u8 tx_data[3] = {0};
    u8 rx_data[4] = {0};
    u32 Major = 0, Minor = 0;

    ITO_TEST_DEBUG("GetTpType\n");
        
    tx_data[0] = 0x53;
    tx_data[1] = 0x00;
    tx_data[2] = 0x2A;
    ito_test_i2c_write(ITO_TEST_ADDR_TP, &tx_data[0], 3);
    mdelay(50);
    ito_test_i2c_read(ITO_TEST_ADDR_TP, &rx_data[0], 4);
    Major = (rx_data[1]<<8) + rx_data[0];
    Minor = (rx_data[3]<<8) + rx_data[2];

    ITO_TEST_DEBUG("***TpTypeMajor = %d ***\n", Major);
    ITO_TEST_DEBUG("***TpTypeMinor = %d ***\n", Minor);
    
    return Major;
    
}

//modify:注意该项目tp数目
static u32 ito_test_choose_TpType(void)
{
    u32 tpType = 0;
    u8 i = 0;
    open_1 = NULL;
    open_1B = NULL;
    open_2 = NULL;
    open_2B = NULL;
    open_3 = NULL;
    MAP1 = NULL;
    MAP2 = NULL;
    MAP3 = NULL;
    MAP40_1 = NULL;
    MAP40_2 = NULL;
    MAP40_3 = NULL;
    MAP40_4 = NULL;
    MAP41_1 = NULL;
    MAP41_2 = NULL;
    MAP41_3 = NULL;
    MAP41_4 = NULL;
    ito_test_keynum = 0;
    ito_test_dummynum = 0;
    ito_test_trianglenum = 0;
    ito_test_2r = 0;

    for(i=0;i<10;i++)
    {
        tpType = ito_test_get_TpType();
        ITO_TEST_DEBUG("tpType=%d;i=%d;\n",tpType,i);
        if(TP_OF_YUSHUN==tpType)//modify:注意该项目tp数目
        {
            break;
        }
		else if(TP_OF_BOYI==tpType)
        {
            break;
        }
        else if(i<5)
        {
            mdelay(100);  
        }
        else
        {
            ito_test_reset();
        }
    }
    
    if(TP_OF_YUSHUN==tpType)//modify:注意该项目tp数目
    {
        open_1 = open_1_yushun;
        open_1B = open_1B_yushun;
        open_2 = open_2_yushun;
        open_2B = open_2B_yushun;
        open_3 = open_3_yushun;
        MAP1 = MAP1_yushun;
        MAP2 = MAP2_yushun;
        MAP3 = MAP3_yushun;
        MAP40_1 = MAP40_1_yushun;
        MAP40_2 = MAP40_2_yushun;
        MAP40_3 = MAP40_3_yushun;
        MAP40_4 = MAP40_4_yushun;
        MAP41_1 = MAP41_1_yushun;
        MAP41_2 = MAP41_2_yushun;
        MAP41_3 = MAP41_3_yushun;
        MAP41_4 = MAP41_4_yushun;
        ito_test_keynum = NUM_KEY_YUSHUN;
        ito_test_dummynum = NUM_DUMMY_YUSHUN;
        ito_test_trianglenum = NUM_SENSOR_YUSHUN;
        ito_test_2r = ENABLE_2R_YUSHUN;
    }
	else if(TP_OF_BOYI==tpType)
    {
        open_1 = open_1_boyi;
        open_1B = open_1B_boyi;
        open_2 = open_2_boyi;
        open_2B = open_2B_boyi;
        open_3 = open_3_boyi;
        MAP1 = MAP1_boyi;
        MAP2 = MAP2_boyi;
        MAP3 = MAP3_boyi;
        MAP40_1 = MAP40_1_boyi;
        MAP40_2 = MAP40_2_boyi;
        MAP40_3 = MAP40_3_boyi;
        MAP40_4 = MAP40_4_boyi;
        MAP41_1 = MAP41_1_boyi;
        MAP41_2 = MAP41_2_boyi;
        MAP41_3 = MAP41_3_boyi;
        MAP41_4 = MAP41_4_boyi;
        ito_test_keynum = NUM_KEY_BOYI;
        ito_test_dummynum = NUM_DUMMY_BOYI;
        ito_test_trianglenum = NUM_SENSOR_BOYI;
        ito_test_2r = ENABLE_2R_BOYI;
    }
    else
    {
        tpType = 0;
    }
    return tpType;
}
static void ito_test_EnterSerialDebugMode(void)
{
    u8 data[5];

    data[0] = 0x53;
    data[1] = 0x45;
    data[2] = 0x52;
    data[3] = 0x44;
    data[4] = 0x42;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 5);

    data[0] = 0x37;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x35;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);

    data[0] = 0x71;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, &data[0], 1);
}
static uint16_t ito_test_get_num( void )
{
    uint16_t    num_of_sensor,i;
    uint16_t 	RegValue1,RegValue2;
 
    num_of_sensor = 0;
        
    RegValue1 = ito_test_ReadReg( 0x11, 0x4A);
    ITO_TEST_DEBUG("ito_test_get_num,RegValue1=%d\n",RegValue1);
    if ( ( RegValue1 & BIT1) == BIT1 )
    {
    	RegValue1 = ito_test_ReadReg( 0x12, 0x0A);			
    	RegValue1 = RegValue1 & 0x0F;
    	
    	RegValue2 = ito_test_ReadReg( 0x12, 0x16);    		
    	RegValue2 = (( RegValue2 >> 1 ) & 0x0F) + 1;
    	
    	num_of_sensor = RegValue1 * RegValue2;
    }
	else
	{
	    for(i=0;i<4;i++)
	    {
	        num_of_sensor+=(ito_test_ReadReg( 0x12, 0x0A)>>(4*i))&0x0F;
	    }
	}
    ITO_TEST_DEBUG("ito_test_get_num,num_of_sensor=%d\n",num_of_sensor);
    return num_of_sensor;        
}
static void ito_test_polling( void )
{
    uint16_t    reg_int = 0x0000;
    uint8_t     dbbus_tx_data[5];
    uint8_t     dbbus_rx_data[4];
    uint16_t    reg_value;
	unsigned long timeout = 0;

    reg_int = 0;

    ito_test_WriteReg( 0x13, 0x0C, BIT15 );       
    ito_test_WriteReg( 0x12, 0x14, (ito_test_ReadReg(0x12,0x14) | BIT0) );         
            
    ITO_TEST_DEBUG("polling start\n");
	timeout = jiffies + HZ;	// xuke @ 20140711	Add timeout mechanism.
    while( (( reg_int & BIT0 ) == 0x0000 ) && (time_before(jiffies, timeout)))
    {
        dbbus_tx_data[0] = 0x10;
        dbbus_tx_data[1] = 0x3D;
        dbbus_tx_data[2] = 0x18;
        ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3);
		mdelay(20);
        ito_test_i2c_read(ITO_TEST_ADDR_REG,  dbbus_rx_data, 2);
        reg_int = dbbus_rx_data[1];
    }
    ITO_TEST_DEBUG("polling end\n");
    reg_value = ito_test_ReadReg( 0x3D, 0x18 ); 
    ito_test_WriteReg( 0x3D, 0x18, reg_value & (~BIT0) );      
}
static uint16_t ito_test_get_data_out( int16_t* s16_raw_data )
{
    uint8_t     i,dbbus_tx_data[8];
    uint16_t    raw_data[48]={0};
    uint16_t    num_of_sensor;
    uint16_t    reg_int;
    uint8_t		dbbus_rx_data[96]={0};
  
    num_of_sensor = ito_test_get_num();
    if(num_of_sensor*2>96)
    {
        ITO_TEST_DEBUG("danger,num_of_sensor=%d\n",num_of_sensor);
        return num_of_sensor;
    }

    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 ); 
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int & (uint16_t)(~FIQ_E_FRAME_READY_MASK) ) ); 
    ito_test_polling();
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x13;
    dbbus_tx_data[2] = 0x40;
	ITO_TEST_DEBUG("ito_test_get_data_out,ITO_TEST_ADDR_REG\n");
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3);
    mdelay(20);
    ito_test_i2c_read(ITO_TEST_ADDR_REG, &dbbus_rx_data[0], (num_of_sensor * 2));
    mdelay(100);
    for(i=0;i<num_of_sensor * 2;i++)
    {
        ITO_TEST_DEBUG("dbbus_rx_data[%d]=%d\n",i,dbbus_rx_data[i]);
    }
 
    reg_int = ito_test_ReadReg( 0x3d, REG_INTR_FIQ_MASK<<1 ); 
    ito_test_WriteReg( 0x3d, REG_INTR_FIQ_MASK<<1, (reg_int | (uint16_t)FIQ_E_FRAME_READY_MASK ) ); 


    for( i = 0; i < num_of_sensor; i++ )
    {
        raw_data[i] = ( dbbus_rx_data[ 2 * i + 1] << 8 ) | ( dbbus_rx_data[2 * i] );
        s16_raw_data[i] = ( int16_t )raw_data[i];
    }
    
    return(num_of_sensor);
}


static void ito_test_send_data_in( uint8_t step )
{
    uint16_t	i;
    uint8_t 	dbbus_tx_data[512];
    uint16_t 	*Type1=NULL;        

    ITO_TEST_DEBUG("ito_test_send_data_in step=%d\n",step);
	if( step == 4 )
    {
        Type1 = &open_1[0];        
    }
    else if( step == 5 )
    {
        Type1 = &open_2[0];      	
    }
    else if( step == 6 )
    {
        Type1 = &open_3[0];      	
    }
    else if( step == 9 )
    {
        Type1 = &open_1B[0];        
    }
    else if( step == 10 )
    {
        Type1 = &open_2B[0];      	
    } 
     
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;
    dbbus_tx_data[2] = 0x00;    
    for( i = 0; i <= 0x3E ; i++ )
    {
        dbbus_tx_data[3+2*i] = Type1[i] & 0xFF;
        dbbus_tx_data[4+2*i] = ( Type1[i] >> 8 ) & 0xFF;    	
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+0x3F*2);
 
    dbbus_tx_data[2] = 0x7A * 2;
    for( i = 0x7A; i <= 0x7D ; i++ )
    {
        dbbus_tx_data[3+2*(i-0x7A)] = 0;
        dbbus_tx_data[4+2*(i-0x7A)] = 0;    	    	
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 3+8);  
    
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x12;
      
    dbbus_tx_data[2] = 5 * 2;
    dbbus_tx_data[3] = Type1[128+5] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+5] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x0B * 2;
    dbbus_tx_data[3] = Type1[128+0x0B] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x0B] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x12 * 2;
    dbbus_tx_data[3] = Type1[128+0x12] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x12] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);
    
    dbbus_tx_data[2] = 0x15 * 2;
    dbbus_tx_data[3] = Type1[128+0x15] & 0xFF;
    dbbus_tx_data[4] = ( Type1[128+0x15] >> 8 ) & 0xFF;
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, 5);        
}

static void ito_test_set_v( uint8_t Enable, uint8_t Prs)	
{
    uint16_t    u16RegValue;        
    
    
    u16RegValue = ito_test_ReadReg( 0x12, 0x08);   
    u16RegValue = u16RegValue & 0xF1; 							
    if ( Prs == 0 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0C); 		
    }
    else if ( Prs == 1 )
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x0E); 		     	
    }
    else
    {
    	ito_test_WriteReg( 0x12, 0x08, u16RegValue| 0x02); 			
    }    
    
    if ( Enable )
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06);    
        ito_test_WriteReg( 0x11, 0x06, u16RegValue| 0x03);   	
    }
    else
    {
        u16RegValue = ito_test_ReadReg( 0x11, 0x06);    
        u16RegValue = u16RegValue & 0xFC;					
        ito_test_WriteReg( 0x11, 0x06, u16RegValue);         
    }

}

static void ito_test_set_c( uint8_t Csub_Step )
{
    uint8_t i;
    uint8_t dbbus_tx_data[MAX_CHNL_NUM+3];
    uint8_t HighLevel_Csub = false;
    uint8_t Csub_new;
     
    dbbus_tx_data[0] = 0x10;
    dbbus_tx_data[1] = 0x11;        
    dbbus_tx_data[2] = 0x84;        
    for( i = 0; i < MAX_CHNL_NUM; i++ )
    {
		Csub_new = Csub_Step;        
        HighLevel_Csub = false;   
        if( Csub_new > 0x1F )
        {
            Csub_new = Csub_new - 0x14;
            HighLevel_Csub = true;
        }
           
        dbbus_tx_data[3+i] =    Csub_new & 0x1F;        
        if( HighLevel_Csub == true )
        {
            dbbus_tx_data[3+i] |= BIT5;
        }
    }
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);

    dbbus_tx_data[2] = 0xB4;        
    ito_test_i2c_write(ITO_TEST_ADDR_REG, dbbus_tx_data, MAX_CHNL_NUM+3);
}

static void ito_test_sw( void )
{
    ito_test_WriteReg( 0x11, 0x00, 0xFFFF );
    ito_test_WriteReg( 0x11, 0x00, 0x0000 );
    mdelay( 50 );
}



static void ito_test_first( uint8_t item_id , int16_t* s16_raw_data)		
{
//	uint8_t     result = 0;
	uint8_t		loop = 0;
//	uint8_t     dbbus_tx_data[9];
	uint8_t     i,j;
    int16_t     s16_raw_data_tmp[48]={0};
	uint8_t     num_of_sensor, num_of_sensor2;
	uint8_t		total_sensor = 0;
	uint16_t	u16RegValue;
    uint8_t 	*pMapping=NULL;
    
    
	num_of_sensor = 0;
	num_of_sensor2 = 0;	
	
    ITO_TEST_DEBUG("ito_test_first item_id=%d\n",item_id);
	ito_test_WriteReg( 0x0F, 0xE6, 0x01 );

	ito_test_WriteReg( 0x1E, 0x24, 0x0500 );
	ito_test_WriteReg( 0x1E, 0x2A, 0x0000 );
	ito_test_WriteReg( 0x1E, 0xE6, 0x6E00 );
	ito_test_WriteReg( 0x1E, 0xE8, 0x0071 );
	    
    if ( item_id == 40 )    			
    {
        pMapping = &MAP1[0];
        if ( ito_test_2r )
		{
			total_sensor = ito_test_trianglenum/2; 
		}
		else
		{
		    total_sensor = ito_test_trianglenum/2 + ito_test_keynum + ito_test_dummynum;
		}
    }
    else if( item_id == 41 )    		
    {
        pMapping = &MAP2[0];
        if ( ito_test_2r )
		{
			total_sensor = ito_test_trianglenum/2; 
		}
		else
		{
		    total_sensor = ito_test_trianglenum/2 + ito_test_keynum + ito_test_dummynum;
		}
    }
    else if( item_id == 42 )    		
    {
        pMapping = &MAP3[0];      
        total_sensor =  ito_test_trianglenum + ito_test_keynum+ ito_test_dummynum; 
    }
        	    
	    
	loop = 1;
	if ( item_id != 42 )
	{
	    if(total_sensor>11)
        {
            loop = 2;
        }
	}	
    ITO_TEST_DEBUG("loop=%d\n",loop);
	for ( i = 0; i < loop; i++ )
	{
		if ( i == 0 )
		{
			ito_test_send_data_in( item_id - 36 );
		}
		else
		{ 
			if ( item_id == 40 ) 
				ito_test_send_data_in( 9 );
			else 		
				ito_test_send_data_in( 10 );
		}
	
		ito_test_set_v(1,0);    
		u16RegValue = ito_test_ReadReg( 0x11, 0x0E);    			
		ito_test_WriteReg( 0x11, 0x0E, u16RegValue | BIT11 );				 		
	
		if ( g_LTP == 1 )
	    	ito_test_set_c( 32 );	    	
		else	    	
	    	ito_test_set_c( 0 );
	    
		ito_test_sw();
		
		if ( i == 0 )	 
        {      
            num_of_sensor=ito_test_get_data_out(  s16_raw_data_tmp );
            ITO_TEST_DEBUG("num_of_sensor=%d;\n",num_of_sensor);
        }
		else	
        {      
            num_of_sensor2=ito_test_get_data_out(  &s16_raw_data_tmp[num_of_sensor] );
            ITO_TEST_DEBUG("num_of_sensor=%d;num_of_sensor2=%d\n",num_of_sensor,num_of_sensor2);
        }
	}
    for ( j = 0; j < total_sensor ; j ++ )
	{
		if ( g_LTP == 1 )
			s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j] + 4096;
		else
			s16_raw_data[pMapping[j]] = s16_raw_data_tmp[j];	
	}	

	return;
}

ITO_TEST_RET ito_test_second (u8 item_id)
{
	u8 i = 0;
    
	s32  s16_raw_data_jg_tmp1 = 0;
	s32  s16_raw_data_jg_tmp2 = 0;
	s32  jg_tmp1_avg_Th_max =0;
	s32  jg_tmp1_avg_Th_min =0;
	s32  jg_tmp2_avg_Th_max =0;
	s32  jg_tmp2_avg_Th_min =0;

	//u8  Th_Tri = 30;        
	//u8  Th_bor = 40;        

	if ( item_id == 40 )    			
    {
        for (i=0; i<(ito_test_trianglenum/2)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_1[MAP40_1[i]];
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_1[MAP40_2[i]];
		}
    }
    else if( item_id == 41 )    		
    {
        for (i=0; i<(ito_test_trianglenum/2)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_2[MAP41_1[i]];
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_2[MAP41_2[i]];
		}
    }

	    jg_tmp1_avg_Th_max = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/2)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp1_avg_Th_min = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/2)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp2_avg_Th_max = (s16_raw_data_jg_tmp2 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp2_avg_Th_min = (s16_raw_data_jg_tmp2 / 2 ) * ( 100 - Th_bor) / 100 ;
	
        ITO_TEST_DEBUG("item_id=%d;sum1=%d;max1=%d;min1=%d;sum2=%d;max2=%d;min2=%d\n",item_id,s16_raw_data_jg_tmp1,jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min,s16_raw_data_jg_tmp2,jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min);

	if ( item_id == 40 ) 
	{
		for (i=0; i<(ito_test_trianglenum/2)-2; i++)
	    {
			if (s16_raw_data_1[MAP40_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_1[MAP40_1[i]] < jg_tmp1_avg_Th_min) 
			{
				//ITO_TEST_DEBUG("i=%d;MAP40_1[i]=%d;s16_raw_data_1[MAP40_1[i]]=%d;jg_tmp1_avg_Th_max=%d;jg_tmp1_avg_Th_min=%d\n",i,MAP40_1[i],s16_raw_data_1[MAP40_1[i]],jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min);
				return ITO_TEST_FAIL;
			}
		}
		for (i=0; i<(ito_test_trianglenum/2)-3; i++)//modify: 注意sensor次序
        {
            if (s16_raw_data_1[MAP40_1[i+1]] < s16_raw_data_1[MAP40_1[i]]*4/5 ) 
            {
				//ITO_TEST_DEBUG("i=%d;MAP40_1[i]=%d;s16_raw_data_1[MAP40_1[i]]*4/5=%d\n",i,MAP40_1[i],s16_raw_data_1[MAP40_1[i]]*4/5);
				//ITO_TEST_DEBUG("i+1=%d;MAP40_1[i+1]=%d;s16_raw_data_1[MAP40_1[i+1]]=%d\n",i+1,MAP40_1[i+1],s16_raw_data_1[MAP40_1[i+1]]);
				return ITO_TEST_FAIL;
			}
        }
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_1[MAP40_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_1[MAP40_2[i]] < jg_tmp2_avg_Th_min) 
			{
			//	ITO_TEST_DEBUG("i=%d;MAP40_2[i]=%d;s16_raw_data_1[MAP40_2[i]]=%d;jg_tmp2_avg_Th_max=%d;jg_tmp2_avg_Th_min=%d\n",i,MAP40_2[i],s16_raw_data_1[MAP40_2[i]],jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min);
				return ITO_TEST_FAIL;
			}
		} 
	}

	if ( item_id == 41 ) 
	{
		for (i=0; i<(ito_test_trianglenum/2)-2; i++)
	    {
			if (s16_raw_data_2[MAP41_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_2[MAP41_1[i]] < jg_tmp1_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
        for (i=0; i<(ito_test_trianglenum/2)-3; i++)//modify: 注意sensor次序
        {
            if (s16_raw_data_2[MAP41_1[i+1]] < s16_raw_data_2[MAP41_1[i]]*4/5 ) 
                return ITO_TEST_FAIL;
        }

		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_2[MAP41_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_2[MAP41_2[i]] < jg_tmp2_avg_Th_min) 
				return ITO_TEST_FAIL;
		} 
	}

	return ITO_TEST_OK;
	
}
ITO_TEST_RET ito_test_second_2r (u8 item_id)
{
	u8 i = 0;
    
	s32  s16_raw_data_jg_tmp1 = 0;
	s32  s16_raw_data_jg_tmp2 = 0;
	s32  s16_raw_data_jg_tmp3 = 0;
	s32  s16_raw_data_jg_tmp4 = 0;
	
	s32  jg_tmp1_avg_Th_max =0;
	s32  jg_tmp1_avg_Th_min =0;
	s32  jg_tmp2_avg_Th_max =0;
	s32  jg_tmp2_avg_Th_min =0;
	s32  jg_tmp3_avg_Th_max =0;
	s32  jg_tmp3_avg_Th_min =0;
	s32  jg_tmp4_avg_Th_max =0;
	s32  jg_tmp4_avg_Th_min =0;

	//u8  Th_Tri = 25;    // non-border threshold    
	//u8  Th_bor = 25;    // border threshold    

	if ( item_id == 40 )    			
    {
        for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_1[MAP40_1[i]];  //first region: non-border 
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_1[MAP40_2[i]];  //first region: border
		}

		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp3 += s16_raw_data_1[MAP40_3[i]];  //second region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp4 += s16_raw_data_1[MAP40_4[i]];  //second region: border
		}
    }



	
    else if( item_id == 41 )    		
    {
        for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp1 += s16_raw_data_2[MAP41_1[i]];  //first region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp2 += s16_raw_data_2[MAP41_2[i]];  //first region: border
		}
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
        {
			s16_raw_data_jg_tmp3 += s16_raw_data_2[MAP41_3[i]];  //second region: non-border
		}
		for (i=0; i<2; i++)
        {
			s16_raw_data_jg_tmp4 += s16_raw_data_2[MAP41_4[i]];  //second region: border
		}
    }

	    jg_tmp1_avg_Th_max = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp1_avg_Th_min = (s16_raw_data_jg_tmp1 / ((ito_test_trianglenum/4)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp2_avg_Th_max = (s16_raw_data_jg_tmp2 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp2_avg_Th_min = (s16_raw_data_jg_tmp2 / 2) * ( 100 - Th_bor) / 100 ;
		jg_tmp3_avg_Th_max = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 + Th_Tri) / 100 ;
	    jg_tmp3_avg_Th_min = (s16_raw_data_jg_tmp3 / ((ito_test_trianglenum/4)-2)) * ( 100 - Th_Tri) / 100 ;
        jg_tmp4_avg_Th_max = (s16_raw_data_jg_tmp4 / 2) * ( 100 + Th_bor) / 100 ;
	    jg_tmp4_avg_Th_min = (s16_raw_data_jg_tmp4 / 2) * ( 100 - Th_bor) / 100 ;
		
	
        ITO_TEST_DEBUG("item_id=%d;sum1=%d;max1=%d;min1=%d;sum2=%d;max2=%d;min2=%d;sum3=%d;max3=%d;min3=%d;sum4=%d;max4=%d;min4=%d;\n",item_id,s16_raw_data_jg_tmp1,jg_tmp1_avg_Th_max,jg_tmp1_avg_Th_min,s16_raw_data_jg_tmp2,jg_tmp2_avg_Th_max,jg_tmp2_avg_Th_min,s16_raw_data_jg_tmp3,jg_tmp3_avg_Th_max,jg_tmp3_avg_Th_min,s16_raw_data_jg_tmp4,jg_tmp4_avg_Th_max,jg_tmp4_avg_Th_min);




	if ( item_id == 40 ) 
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_1[MAP40_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_1[MAP40_1[i]] < jg_tmp1_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_1[MAP40_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_1[MAP40_2[i]] < jg_tmp2_avg_Th_min) 
				return ITO_TEST_FAIL;
		} 
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_1[MAP40_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_1[MAP40_3[i]] < jg_tmp3_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_1[MAP40_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_1[MAP40_4[i]] < jg_tmp4_avg_Th_min) 
				return ITO_TEST_FAIL;
		} 
	}

	if ( item_id == 41 ) 
	{
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_2[MAP41_1[i]] > jg_tmp1_avg_Th_max || s16_raw_data_2[MAP41_1[i]] < jg_tmp1_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_2[MAP41_2[i]] > jg_tmp2_avg_Th_max || s16_raw_data_2[MAP41_2[i]] < jg_tmp2_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
		
		for (i=0; i<(ito_test_trianglenum/4)-2; i++)
	    {
			if (s16_raw_data_2[MAP41_3[i]] > jg_tmp3_avg_Th_max || s16_raw_data_2[MAP41_3[i]] < jg_tmp3_avg_Th_min) 
				return ITO_TEST_FAIL;
		}
		
		for (i=0; i<2; i++)
	    {
			if (s16_raw_data_2[MAP41_4[i]] > jg_tmp4_avg_Th_max || s16_raw_data_2[MAP41_4[i]] < jg_tmp4_avg_Th_min) 
				return ITO_TEST_FAIL;
		} 

	}

	return ITO_TEST_OK;
	
}

static void ito_test_get_test_range(void)
{
    u8 tx_data[3] = {0};
    u8 rx_data[4] = {0};
    u16 triBin = 0, borBin = 0;

    ITO_TEST_DEBUG("Get test range\n");
        
    tx_data[0] = 0x53;
    tx_data[1] = 0x00;
    tx_data[2] = 0x50;//modify:默认使用固件端u16Reserved3和u16Reserved4
    ito_test_i2c_write(ITO_TEST_ADDR_TP, &tx_data[0], 3);
    mdelay(50);
    ito_test_i2c_read(ITO_TEST_ADDR_TP, &rx_data[0], 4);
    triBin = (rx_data[1]<<8) + rx_data[0];
    borBin = (rx_data[3]<<8) + rx_data[2];

    ITO_TEST_DEBUG("***triBin = %d ***\n", triBin);
    ITO_TEST_DEBUG("***borBin = %d ***\n", borBin);
    if(triBin>0
        &&triBin<100
        &&borBin>0
        &&borBin<100)
    {
        Th_Tri = Th_Tri>triBin?Th_Tri:triBin;
        Th_bor = Th_bor>borBin?Th_bor:borBin;
    }
    ITO_TEST_DEBUG("***Th_Tri = %d ***\n", Th_Tri);
    ITO_TEST_DEBUG("***Th_bor = %d ***\n", Th_bor);
}


static ITO_TEST_RET ito_test_interface(void)
{
    ITO_TEST_RET ret = ITO_TEST_OK;
    uint16_t i = 0;
#ifdef DMA_IIC
    _msg_dma_alloc();
#endif
    ito_test_set_iic_rate(true, 50000);
	msleep(100);

    ITO_TEST_DEBUG("start\n");
    ito_test_disable_irq();
	ito_test_reset();
    if(!ito_test_choose_TpType())
    {
        ITO_TEST_DEBUG("choose tpType fail\n");
        ret = ITO_TEST_GET_TP_TYPE_ERROR;
        goto ITO_TEST_END;
    }
    ito_test_get_test_range();
    ito_test_EnterSerialDebugMode();
    mdelay(100);
    ITO_TEST_DEBUG("EnterSerialDebugMode\n");
    ito_test_WriteReg8Bit ( 0x0F, 0xE6, 0x01 );
    ito_test_WriteReg ( 0x3C, 0x60, 0xAA55 );
    ITO_TEST_DEBUG("stop mcu and disable watchdog V.005\n");   
    mdelay(50);
    
	for(i = 0;i < 48;i++)
	{
		s16_raw_data_1[i] = 0;
		s16_raw_data_2[i] = 0;
		s16_raw_data_3[i] = 0;
	}	
	
    ito_test_first(40, s16_raw_data_1);
    ITO_TEST_DEBUG("40 get s16_raw_data_1\n");
    if(ito_test_2r)
    {
        ret=ito_test_second_2r(40);
    }
    else
    {
        ret=ito_test_second(40);
    }
    if(ITO_TEST_FAIL==ret)
    {
        goto ITO_TEST_END;
    }
    
    ito_test_first(41, s16_raw_data_2);
    ITO_TEST_DEBUG("41 get s16_raw_data_2\n");
    if(ito_test_2r)
    {
        ret=ito_test_second_2r(41);
    }
    else
    {
        ret=ito_test_second(41);
    }
    if(ITO_TEST_FAIL==ret)
    {
        goto ITO_TEST_END;
    }
    
    //ito_test_first(42, s16_raw_data_3);
    //ITO_TEST_DEBUG("42 get s16_raw_data_3\n");
    
    ITO_TEST_END:
#ifdef DMA_IIC
    _msg_dma_free();
#endif
    ito_test_set_iic_rate(false, 100000);
	ito_test_reset();
    ito_test_enable_irq();
    ITO_TEST_DEBUG("end\n");
    return ret;
}

#include <linux/proc_fs.h>
#define ITO_TEST_AUTHORITY 0777 
static struct proc_dir_entry *msg_ito_test = NULL;
static struct proc_dir_entry *debug = NULL;
static struct proc_dir_entry *debug_on_off = NULL;
#define PROC_MSG_ITO_TESE      "msg-ito-test"
#define PROC_ITO_TEST_DEBUG      "debug"
#define PROC_ITO_TEST_DEBUG_ON_OFF     "debug-on-off"
ITO_TEST_RET g_ito_test_ret = ITO_TEST_OK;
static int ito_test_proc_read_debug(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
    g_ito_test_ret = ito_test_interface();
    if(ITO_TEST_OK==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_OK");
    }
    else if(ITO_TEST_FAIL==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_FAIL");
    }
    else if(ITO_TEST_GET_TP_TYPE_ERROR==g_ito_test_ret)
    {
        ITO_TEST_DEBUG_MUST("ITO_TEST_GET_TP_TYPE_ERROR");
    }
    
    return size;
}

static int ito_test_proc_write_debug(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{    
    u16 i = 0;
    mdelay(5);
    ITO_TEST_DEBUG_MUST("ito_test_ret = %d",g_ito_test_ret);
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_1[%d]=%d;\n",i,s16_raw_data_1[i]);
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_2[%d]=%d;\n",i,s16_raw_data_2[i]);
    }
    mdelay(5);
    for(i=0;i<48;i++)
    {
        ITO_TEST_DEBUG_MUST("data_3[%d]=%d;\n",i,s16_raw_data_3[i]);
    }
    mdelay(5);
    return size;
}
static int ito_test_proc_read_debug_on_off(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
    bItoTestDebug = 1;
    ITO_TEST_DEBUG_MUST("on debug bItoTestDebug = %d",bItoTestDebug);
    return size;
}

static int ito_test_proc_write_debug_on_off(struct file *file, const char __user *buf, size_t size, loff_t *ppos)
{    
    bItoTestDebug = 0;
    ITO_TEST_DEBUG_MUST("off debug bItoTestDebug = %d",bItoTestDebug);
    return size;
}

static const struct file_operations debug_fops = {
	.read		= ito_test_proc_read_debug,
	.write		= ito_test_proc_write_debug,
};

static const struct file_operations debug_on_off_fops = {
	.read		= ito_test_proc_read_debug_on_off,
	.write		= ito_test_proc_write_debug_on_off,
};

static void ito_test_create_entry(void)
{
    msg_ito_test = proc_mkdir(PROC_MSG_ITO_TESE, NULL);
#if 1		// xuke @ 20140707
	debug = proc_create_data(PROC_ITO_TEST_DEBUG, ITO_TEST_AUTHORITY, msg_ito_test, &debug_fops, NULL);
	if (IS_ERR_OR_NULL(debug))
	{
		pr_err("add /proc/msg-ito-test/debug error \n");
	}
	
	debug_on_off = proc_create_data(PROC_ITO_TEST_DEBUG_ON_OFF, ITO_TEST_AUTHORITY, msg_ito_test, &debug_on_off_fops, NULL);
	if (IS_ERR_OR_NULL(debug_on_off))
	{
		pr_err("add /proc/msg-ito-test/debug_on_off error \n");
	}
#else
    debug = create_proc_entry(PROC_ITO_TEST_DEBUG, ITO_TEST_AUTHORITY, msg_ito_test);
    debug_on_off= create_proc_entry(PROC_ITO_TEST_DEBUG_ON_OFF, ITO_TEST_AUTHORITY, msg_ito_test);

    if (NULL==debug) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DEBUG failed\n");
    } 
    else 
    {
        debug->read_proc = ito_test_proc_read_debug;
        debug->write_proc = ito_test_proc_write_debug;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST DEBUG OK\n");
    }
    if (NULL==debug_on_off) 
    {
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST ON OFF failed\n");
    } 
    else 
    {
        debug_on_off->read_proc = ito_test_proc_read_debug_on_off;
        debug_on_off->write_proc = ito_test_proc_write_debug_on_off;
        ITO_TEST_DEBUG_MUST("create_proc_entry ITO TEST ON OFF OK\n");
    }
#endif
}
#endif

#ifdef TP_PRINT
#include <linux/proc_fs.h>

#define MAX_IIC_LENGTH 256
#define TP_PRINT_AUTHORITY 0777
#define PROC_TP_PRINT_MSG   "msgtpp"
#define PROC_TP_PRINT_NODE  "tpp"

static struct proc_dir_entry *msg_tpp = NULL;
static struct proc_dir_entry *tpp = NULL;

static int tp_print_proc_read(struct file *file, char __user *buffer, size_t len, loff_t *ppos)
{
    u16 i, address;
    u8  units,size;
    u8 dbbus_tx_data[3] = {0};
    u8 dbbus_rx_data[80] = {0};
    s16 s16Data[40]={0};
//    char *buf = NULL;
//    int rc = 0;

    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = 0;
    dbbus_tx_data[2] = 0x4c;
    mutex_lock(&tpp_lock);
    msleep(10);
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0],2);
    msleep(10);
    mutex_unlock(&tpp_lock);

    units = dbbus_rx_data[0]<<8||dbbus_rx_data[1];
    printk("test   units=%d\n",  units);
  	if(units == 1)
  	{
		address = 0x0800;
		size = 48;
  	}
  	else if(units == 2)
  	{
		address = 0x0830;
		size = 48;
  	}
  	else if(units == 3)
  	{
		address = 0x0964;
		size = 80;
  	}
  	else if(units == 4)
  	{
		address = 0x09fc;
		size = 76;
  	}
  	else if(units == 5)
  	{
		address = 0x0a48;
		size = 76;
  	}
  	else if(units == 6)
  	{
		address = 0x0b2c;
		size = 76;
  	}
    dbbus_tx_data[0] = 0x53;
    dbbus_tx_data[1] = (address >> 8) & 0xFF;
    dbbus_tx_data[2] = address & 0xFF;
	
    mutex_lock(&tpp_lock);
    msleep(10);
    HalTscrCDevWriteI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_tx_data[0], 3);
    HalTscrCReadI2CSeq(FW_ADDR_MSG21XX_TP, &dbbus_rx_data[0],size);
    msleep(10);
    mutex_unlock(&tpp_lock);
  	for(i = 0 ;i <size/2 ;i++)
  	{
		s16Data[i]= dbbus_rx_data[2*i]<<8 ||dbbus_rx_data[2*i+1];
		printk("test s16Data[%d] = %d \n",i,s16Data[i]);
  	}

    return 0;
}

static const struct file_operations tp_print_proc_fops = {
	.read		= tp_print_proc_read,
};

static void tp_print_create_entry(void)
{
#if 1
    msg_tpp = proc_mkdir(PROC_TP_PRINT_MSG, NULL);
	tpp = proc_create_data(PROC_TP_PRINT_NODE, TP_PRINT_AUTHORITY, msg_tpp, &tp_print_proc_fops, NULL);
	if (IS_ERR_OR_NULL(tpp))
	{
		pr_err("tp_print_create_entry failed\n");
	}
#else
    tpp = create_proc_entry(PROC_TP_PRINT_NODE, TP_PRINT_AUTHORITY, msg_tpp);

    if (NULL == tpp)
    {
        printk("tp_print_create_entry failed\n");
    }
    else
    {
        tpp->read_proc = tp_print_proc_read;
        tpp->write_proc = NULL;
        printk("tp_print_create_entry OK\n");
    }
#endif
}
#endif

MODULE_AUTHOR("wax.wang cellon");
MODULE_DESCRIPTION("Driver for msg21xx Touchscreen Controller");
MODULE_LICENSE("GPL");
