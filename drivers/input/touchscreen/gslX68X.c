/* drivers/input/touchscreen/gslX68X.h
 * 
 * 2010 - 2013 SLIEAD Technology.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be a reference 
 * to you, when you are integrating the SLIEAD's CTP IC into your system, 
 * but WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
 * General Public License for more details.
 * 
 */
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/byteorder/generic.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/platform_device.h>

#include <linux/firmware.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>

#include "lct_tp_fm_info.h"
#include "lct_ctp_selftest.h"

#include "gslX68X.h"

#ifdef GSL_REPORT_POINT_SLOT
    #include <linux/input/mt.h>
#endif

#define GSL9XX_VDDIO_1800 1

#define COORDS_ARR_SIZE	4

#define VTG_MIN_UV		2850000
#define VTG_MAX_UV		2850000
#define I2C_VTG_MIN_UV	1800000
#define I2C_VTG_MAX_UV	1800000

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR

#define DEBUG_TP_SENSOR
#define PROXIMITY_INPUT_DEV_NAME 	"proximity"
#define SENSOR_PROX_TP_USE_WAKELOCK
static struct i2c_client *i2c_prox_client=NULL;
static DEFINE_MUTEX(tp_prox_sensor_mutex);
static DECLARE_WAIT_QUEUE_HEAD(tp_sensor_waitqueue);

// xuke @ 20140922	Correct the sys authority for CTS test.
#if defined(CONFIG_L6140_COMMON) || defined(CONFIG_L6300_COMMON)
#define SYS_AUTHORITY_FOR_CTS
#endif

#ifdef SYS_AUTHORITY_FOR_CTS
#define SYS_AUTHORITY		(S_IRUGO|S_IWUSR|S_IWGRP)
#else
#define SYS_AUTHORITY		(S_IRUGO|S_IWUGO)
#endif

#ifdef SENSOR_PROX_TP_USE_WAKELOCK
static	struct wake_lock sensor_prox_tp_wake_lock;
#endif

static int tp_prox_sensor_opened;
static char tp_prox_sensor_data = 1; 
static char tp_pre_sensor_data = 1; 
static int tp_prox_sensor_data_changed = 0;
#define STATUS_SUSPENDED 	1
#define STATUS_RESUMED	0
//static int tp_system_status = STATUS_RESUMED;
//static int tp_unset_status = 0;
static void tp_prox_sensor_enable(int enable);
static int is_need_report_pointer = 1;
#endif//CONFIG_TOUCHPANEL_PROXIMITY_SENSOR

static int			is_suspend		= 0;



static struct mutex gsl_i2c_lock;

#ifdef GSL_DEBUG 
#define print_info(fmt, args...)   \
        do{                              \
                printk("[tp-gsl][%s]"fmt,__func__, ##args);     \
        }while(0)
#else
#define print_info(fmt, args...)   //
#endif

#define TPD_PROC_DEBUG
#ifdef TPD_PROC_DEBUG
static struct proc_dir_entry *gsl_config_proc = NULL;
#define GSL_CONFIG_PROC_FILE "gsl_config"
#define CONFIG_LEN 31
static char gsl_read[CONFIG_LEN];
static u8 gsl_data_proc[8] = {0};
static u8 gsl_proc_flag = 0;
#endif



#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void gsl_early_suspend(struct early_suspend *handler);
static void gsl_early_resume(struct early_suspend *handler);
#endif

#ifdef TOUCH_VIRTUAL_KEYS
static ssize_t virtual_keys_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{

	return sprintf(buf,
         __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":120:900:40:40"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":240:900:40:40"
	 ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":360:900:40:40"
	 "\n");
}

static struct kobj_attribute virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.GSL_TP",
		.mode = S_IRUGO,
	},
	.show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
	&virtual_keys_attr.attr,
	NULL
};

static struct attribute_group properties_attr_group = {
	.attrs = properties_attrs,
};

static void gsl_ts_virtual_keys_init(void)
{
	int ret;
	struct kobject *properties_kobj;

	print_info("%s\n",__func__);

	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
			&properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");
}

#endif


/*define golbal variable*/
static struct gsl_ts_data *ddata=NULL;

static int gsl_read_interface(struct i2c_client *client, u8 reg, u8 *buf, u32 num)
{
	int err = 0;
	u8 temp = reg;
	mutex_lock(&gsl_i2c_lock);
	if(temp < 0x80)
	{
		temp = (temp+8)&0x5c;
		i2c_master_send(client,&temp,1);
		err = i2c_master_recv(client,&buf[0],4);

		temp = reg;
		i2c_master_send(client,&temp,1);
		err = i2c_master_recv(client,&buf[0],4);
	}
	i2c_master_send(client,&reg,1);
	err = i2c_master_recv(client,&buf[0],num);
	mutex_unlock(&gsl_i2c_lock);
	return err;
}

static int gsl_write_interface(struct i2c_client *client, const u8 reg, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];
	int err;
	u8 tmp_buf[num+1];
	tmp_buf[0] = reg;
	memcpy(tmp_buf + 1, buf, num);
	xfer_msg[0].addr = client->addr;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = tmp_buf;
	//xfer_msg[0].timing = 400;

	mutex_lock(&gsl_i2c_lock);
	err= i2c_transfer(client->adapter, xfer_msg, 1);
	mutex_unlock(&gsl_i2c_lock);
	return err;	
//	return i2c_transfer(client->adapter, xfer_msg, 1) == 1 ? 0 : -EFAULT;
}
static void gsl_io_control(struct i2c_client *client)
{ 
	u8 buf[4] = {0};
	int i;
#if GSL9XX_VDDIO_1800
	for(i=0;i<5;i++){
		buf[0] = 0;
		buf[1] = 0;
		buf[2] = 0xfe;
		buf[3] = 0x1;
		gsl_write_interface(client,0xf0,buf,4);
		buf[0] = 0x5;
		buf[1] = 0;
		buf[2] = 0;
		buf[3] = 0x80;
		gsl_write_interface(client,0x78,buf,4);
		msleep(5);
	}
	msleep(50);
#endif
}
#if 0
#define DMA_TRANS_LEN 0x20
static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[DMA_TRANS_LEN*4] = {0};
	u8 send_flag = 1;
	u8 addr=0;
	u32 source_line = 0;
	u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

	printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
		if (0xf0 == GSL_DOWNLOAD_DATA[source_line].offset)
		{
			memcpy(buf,&GSL_DOWNLOAD_DATA[source_line].val,4);	
			gsl_write_interface(client, 0xf0, buf, 4);
			send_flag = 1;
		}
		else 
		{
			if (1 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20))
	    			addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;

			memcpy((buf+send_flag*4 -4),&GSL_DOWNLOAD_DATA[source_line].val,4);	

			if (0 == send_flag % (DMA_TRANS_LEN < 0x20 ? DMA_TRANS_LEN : 0x20)) 
			{
	    		gsl_write_interface(client, addr, buf, DMA_TRANS_LEN * 4);
				send_flag = 0;
			}

			send_flag++;
		}
	}

	printk("=============gsl_load_fw end==============\n");

}
#else 
static void gsl_load_fw(struct i2c_client *client,const struct fw_data *GSL_DOWNLOAD_DATA,int data_len)
{
	u8 buf[4] = {0};
	//u8 send_flag = 1;
	u8 addr=0;
	u32 source_line = 0;
	u32 source_len = data_len;//ARRAY_SIZE(GSL_DOWNLOAD_DATA);

	printk("=============gsl_load_fw start==============\n");

	for (source_line = 0; source_line < source_len; source_line++) 
	{
		/* init page trans, set the page val */
		addr = (u8)GSL_DOWNLOAD_DATA[source_line].offset;
		memcpy(buf,&GSL_DOWNLOAD_DATA[source_line].val,4);
		gsl_write_interface(client, addr, buf, 4);	
	}
	printk("=============gsl_load_fw end==============\n");
}
#endif
static void gsl_start_core(struct i2c_client *client)
{
	//u8 tmp = 0x00;
	u8 buf[4] = {0};
#if 0
	buf[0]=0;
	buf[1]=0x10;
	buf[2]=0xfe;
	buf[3]=0x1;
	gsl_write_interface(client,0xf0,buf,4);
	buf[0]=0xf;
	buf[1]=0;
	buf[2]=0;
	buf[3]=0;
	gsl_write_interface(client,0x4,buf,4);
	msleep(20);
#endif
	buf[0]=0;
	gsl_write_interface(client,0xe0,buf,4);
#ifdef GSL_ALG_ID
	{
		gsl_DataInit(gsl_config_data_id);
	}
#endif	
}

static void gsl_reset_core(struct i2c_client *client)
{
	u8 buf[4] = {0x00};
	
	buf[0] = 0x88;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(5);

	buf[0] = 0x04;
	gsl_write_interface(client,0xe4,buf,4);
	msleep(5);
	
	buf[0] = 0;
	gsl_write_interface(client,0xbc,buf,4);
	msleep(5);

			
	buf[3] = 0x01;
	buf[2] = 0xfe;
	buf[1] = 0;
	buf[0] = 0;
	gsl_write_interface(client,0xf0,buf,4);
	buf[3] = 0x80;
	buf[2] = 0;
	buf[1] = 0;
	buf[0] = 0x5;
	gsl_write_interface(client,0x78,buf,4);
	msleep(20);
	gsl_io_control(client);
}

static void gsl_clear_reg(struct i2c_client *client)
{
	u8 buf[4]={0};
	//clear reg
	buf[0]=0x88;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(20);
	buf[0]=0x3;
	gsl_write_interface(client,0x80,buf,4);
	msleep(5);
	buf[0]=0x4;
	gsl_write_interface(client,0xe4,buf,4);
	msleep(5);
	buf[0]=0x0;
	gsl_write_interface(client,0xe0,buf,4);
	msleep(20);
	//clear reg

}
#define GSL_TEST_TP
#ifdef GSL_TEST_TP
extern void gsl_write_test_config(unsigned int cmd,int value);
extern unsigned int gsl_read_test_config(unsigned int cmd);
extern int gsl_obtain_array_data_ogv(unsigned short *ogv,int i_max,int j_max);
extern int gsl_obtain_array_data_dac(unsigned int *dac,int i_max,int j_max);
extern int gsl_tp_module_test(char *buf,int size);
void gsl_I2C_ROnePage(unsigned int addr, char *buf)
{
	u8 tmp_buf[4]={0};
	tmp_buf[3]=(u8)(addr>>24);
	tmp_buf[2]=(u8)(addr>>16);
	tmp_buf[1]=(u8)(addr>>8);
	tmp_buf[0]=(u8)(addr);
	gsl_write_interface(ddata->client,0xf0,tmp_buf,4);
	gsl_read_interface(ddata->client,0,buf,128);
}
EXPORT_SYMBOL(gsl_I2C_ROnePage);
void gsl_I2C_RTotal_Address(unsigned int addr,unsigned int *data)
{
	u8 tmp_buf[4]={0};	
	tmp_buf[3]=(u8)((addr/0x80)>>24);
	tmp_buf[2]=(u8)((addr/0x80)>>16);
	tmp_buf[1]=(u8)((addr/0x80)>>8);
	tmp_buf[0]=(u8)((addr/0x80));
	gsl_write_interface(ddata->client,0xf0,tmp_buf,4);
	gsl_read_interface(ddata->client,addr%0x80,tmp_buf,4);
	*data = tmp_buf[0]|(tmp_buf[1]<<8)|(tmp_buf[2]<<16)|(tmp_buf[3]<<24);
}
EXPORT_SYMBOL(gsl_I2C_RTotal_Address);
#endif
static int gsl_compatible_id(struct i2c_client *client)
{
	u8 buf[4];
	int i,err;

	printk("%s\n", __func__);
	for(i=0;i<5;i++)
	{
		err = gsl_read_interface(client,0xfc,buf,4);
		printk("[tp-gsl] 0xfc = {0x%02x%02x%02x%02x}\n",buf[3],buf[2],
			buf[1],buf[0]);
		if(!(err<0))
		{
			err = 1;		
			break;	
		}
	}
	return err;	
}
static void gsl_hw_init(struct gsl_ts_data *data)
{
	struct gsl_ts_data *pdata = data;
	//add power
	GSL_POWER_ON();	
	//
	if (gpio_is_valid(pdata->irq_gpio))
	{
		gpio_request(pdata->irq_gpio, GSL_IRQ_NAME);
		gpio_direction_input(pdata->irq_gpio);
	}
	
	if (gpio_is_valid(pdata->reset_gpio))
	{
		gpio_request(pdata->reset_gpio,GSL_RST_NAME);
		gpio_direction_output(pdata->reset_gpio,1);	
		
		msleep(5);
		gpio_set_value(pdata->reset_gpio,0);
		msleep(10); 
		gpio_set_value(pdata->reset_gpio,1);
		msleep(20);
	}
}
static void gsl_sw_init(struct i2c_client *client)
{
	struct gsl_ts_data *pdata = i2c_get_clientdata(client);
	int temp;
	
	//struct fw_data *fw = GSLx68x_FW;
	if(1==ddata->gsl_sw_flag)
		return;
	ddata->gsl_sw_flag = 1;
	
	gpio_set_value(pdata->reset_gpio, 0);
	msleep(20);
	gpio_set_value(pdata->reset_gpio, 1);
	msleep(20);	

	gsl_clear_reg(client);
	gsl_reset_core(client);
	gsl_io_control(client);

	{
		temp = ARRAY_SIZE(GSLX68X_FW);
		gsl_load_fw(client,GSLX68X_FW,temp);	
	}
	gsl_io_control(client);

	gsl_start_core(client);

	ddata->gsl_sw_flag = 0;
}

static void check_mem_data(struct i2c_client *client)
{

	u8 read_buf[4]  = {0};
	msleep(30);
	gsl_read_interface(client,0xb0,read_buf,4);
	if (read_buf[3] != 0x5a || read_buf[2] != 0x5a 
		|| read_buf[1] != 0x5a || read_buf[0] != 0x5a)
	{
		printk("0xb4 ={0x%02x%02x%02x%02x}\n",
			read_buf[3], read_buf[2], read_buf[1], read_buf[0]);
		gsl_sw_init(client);
	}
}
static int char_to_int(char ch)
{
	if(ch>='0' && ch<='9')
		return (ch-'0');
	else
		return (ch-'a'+10);
}
//
#ifdef TPD_PROC_DEBUG
static int gsl_ctp_selftest(void)
{
	int ret = -1;
	unsigned short *gsl_ogv;
	unsigned int *dac;
	char *test_buf;
	int i,j,tmp;

	//return 0;
	gsl_ogv = (unsigned short *)kzalloc(15*8*2,GFP_KERNEL);
	if(!gsl_ogv){
		return -1;
	}
	dac = (unsigned int *)kzalloc(4*8*4,GFP_KERNEL);
	if(!dac){
		kfree(gsl_ogv);
		return -1;
	}
	test_buf = kzalloc(1024*6,GFP_KERNEL);
	if(!test_buf){
		kfree(gsl_ogv);
		kfree(dac);
		return -1;
	}
	tmp = gsl_tp_module_test(test_buf,1024*6);
	if(tmp>0)
	{
		ret = 0;
		printk("gsl tp module test is pass!\n");
	}
	tmp = gsl_obtain_array_data_ogv(gsl_ogv,15,8);
	tmp = gsl_obtain_array_data_dac(dac,4,8);
	printk("gsl tp module test ogv resutle:\n %s \n",test_buf);
	printk("gsl tp module test dac resutle:\n %s \n",&test_buf[1024*2]);
	printk("gsl tp module test rate resutle:\n %s \n",&test_buf[1024*4]);
	printk("gsl original value:\n");
	for(i=0;i<15;i++){
		for(j=0;j<8;j++){
			printk("%4d ",gsl_ogv[i*8+j]);
		}
		printk("\n");
	}
	printk("dac value:\n");
	for(i=0;i<4;i++){
		for(j=0;j<8;j++){
			printk("%4d ",dac[i*8+j]);
		}
		printk("\n");
	}
	kfree(gsl_ogv);
	kfree(dac);
	kfree(test_buf);

	return ret;
	
}
static int gsl_config_read_proc(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
	char *ptr = buf;
	char temp_data[5] = {0};
	unsigned int tmp=0;
	if('v'==gsl_read[0]&&'s'==gsl_read[1])
	{
#ifdef GSL_ALG_ID
		tmp=gsl_version_id();
#else 
		tmp=0x20121215;
#endif
		ptr += sprintf(ptr,"version:%x\n",tmp);
	}
	else if('r'==gsl_read[0]&&'e'==gsl_read[1])
	{
		if('i'==gsl_read[3])
		{
#ifdef GSL_ALG_ID 
			tmp=(gsl_data_proc[5]<<8) | gsl_data_proc[4];
			ptr +=sprintf(ptr,"gsl_config_data_id[%d] = ",tmp);
			if(tmp>=0&&tmp<ARRAY_SIZE(gsl_config_data_id))
				ptr +=sprintf(ptr,"%d\n",gsl_config_data_id[tmp]); 
#endif
		}
		else 
		{
			gsl_write_interface(ddata->client,0xf0,&gsl_data_proc[4],4);
			gsl_read_interface(ddata->client,gsl_data_proc[0],temp_data,4);
			ptr +=sprintf(ptr,"offset : {0x%02x,0x",gsl_data_proc[0]);
			ptr +=sprintf(ptr,"%02x",temp_data[3]);
			ptr +=sprintf(ptr,"%02x",temp_data[2]);
			ptr +=sprintf(ptr,"%02x",temp_data[1]);
			ptr +=sprintf(ptr,"%02x};\n",temp_data[0]);
		}
	}
#ifdef GSL_TEST_TP
	else if('t'==gsl_read[0]&&'r'==gsl_read[1]){
		unsigned int value;
		value = gsl_read_test_config((unsigned int)gsl_data_proc[4]);
		ptr +=sprintf(ptr,"tp test config [%d]:",gsl_data_proc[4]);
		ptr +=sprintf(ptr,"%4d\n",value);
	}else if('t'==gsl_read[0]&&'t'==gsl_read[1]){
		unsigned short *gsl_ogv;
		unsigned int *dac;
		char *test_buf;
		int i,j,tmp;
		gsl_ogv = (unsigned short *)kzalloc(15*8*2,GFP_KERNEL);
		if(!gsl_ogv){
			return -1;
		}
		dac = (unsigned int *)kzalloc(4*8*4,GFP_KERNEL);
		if(!dac){
			kfree(gsl_ogv);
			return -1;
		}
		test_buf = kzalloc(1024*6,GFP_KERNEL);
		if(!test_buf){
			kfree(gsl_ogv);
			kfree(dac);
			return -1;
		}
		tmp = gsl_tp_module_test(test_buf,1024*6);
		if(tmp>0)
			printk("tp module test is pass!\n");
		tmp = gsl_obtain_array_data_ogv(gsl_ogv,15,8);
		tmp = gsl_obtain_array_data_dac(dac,4,8);
		printk("tp module test ogv resutle:\n %s \n",test_buf);
		printk("tp module test dac resutle:\n %s \n",&test_buf[1024*2]);
		printk("tp module test rate resutle:\n %s \n",&test_buf[1024*4]);
		printk("original value:\n");
		for(i=0;i<15;i++){
			for(j=0;j<8;j++){
				printk("%4d ",gsl_ogv[i*8+j]);
			}
			printk("\n");
		}
		printk("dac value:\n");
		for(i=0;i<4;i++){
			for(j=0;j<8;j++){
				printk("%4d ",dac[i*8+j]);
			}
			printk("\n");
		}
		kfree(gsl_ogv);
		kfree(dac);
		kfree(test_buf);

	}
#endif

	return (ptr - buf);
}
static int gsl_config_write_proc(struct file *file, const char __user *buffer, size_t size, loff_t *ppos)
{
	u8 buf[8] = {0};
	char temp_buf[CONFIG_LEN];
	char *path_buf;
	int tmp = 0;
	int tmp1 = 0;
	print_info("[tp-gsl][%s] \n",__func__);
	if(size > 512)
	{
		print_info("size not match [%d:%d]\n", CONFIG_LEN, size);
        	return -EFAULT;
	}
	path_buf=kzalloc(size,GFP_KERNEL);
	if(!path_buf)
	{
		printk("alloc path_buf memory error \n");
		return -1;
	}	
	if(copy_from_user(path_buf, buffer, size))
	{
		print_info("copy from user fail\n");
		goto exit_write_proc_out;
	}
	memcpy(temp_buf,path_buf,(size<CONFIG_LEN?size:CONFIG_LEN));
	print_info("[tp-gsl][%s][%s]\n",__func__,temp_buf);
	
	buf[3]=char_to_int(temp_buf[14])<<4 | char_to_int(temp_buf[15]);	
	buf[2]=char_to_int(temp_buf[16])<<4 | char_to_int(temp_buf[17]);
	buf[1]=char_to_int(temp_buf[18])<<4 | char_to_int(temp_buf[19]);
	buf[0]=char_to_int(temp_buf[20])<<4 | char_to_int(temp_buf[21]);
	
	buf[7]=char_to_int(temp_buf[5])<<4 | char_to_int(temp_buf[6]);
	buf[6]=char_to_int(temp_buf[7])<<4 | char_to_int(temp_buf[8]);
	buf[5]=char_to_int(temp_buf[9])<<4 | char_to_int(temp_buf[10]);
	buf[4]=char_to_int(temp_buf[11])<<4 | char_to_int(temp_buf[12]);
	if('v'==temp_buf[0]&& 's'==temp_buf[1])//version //vs
	{
		memcpy(gsl_read,temp_buf,4);
		printk("gsl version\n");
	}
	else if('s'==temp_buf[0]&& 't'==temp_buf[1])//start //st
	{
		gsl_proc_flag = 1;
		gsl_reset_core(ddata->client);
	}
	else if('e'==temp_buf[0]&&'n'==temp_buf[1])//end //en
	{
		msleep(20);
		gsl_reset_core(ddata->client);
		gsl_start_core(ddata->client);
		gsl_proc_flag = 0;
	}
	else if('r'==temp_buf[0]&&'e'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('w'==temp_buf[0]&&'r'==temp_buf[1])//write buf
	{
		gsl_write_interface(ddata->client,buf[4],buf,4);
	}
	
#ifdef GSL_ALG_ID
	else if('i'==temp_buf[0]&&'d'==temp_buf[1])//write id config //
	{
		tmp1=(buf[7]<<24)|(buf[6]<<16)|(buf[5]<<8)|buf[4];
		tmp=(buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		if(tmp1>=0 && tmp1<ARRAY_SIZE(gsl_config_data_id))
		{
			gsl_config_data_id[tmp1] = tmp;
		}
	}
#endif
#ifdef GSL_TEST_TP
	else if('t'==temp_buf[0]&&'w'==temp_buf[1]){
		unsigned int value = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|buf[0];
		gsl_write_test_config((unsigned int)buf[4],value);
	}
	else if('t'==temp_buf[0]&&'r'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
	else if('t'==temp_buf[0]&&'t'==temp_buf[1])//read buf //
	{
		memcpy(gsl_read,temp_buf,4);
		memcpy(gsl_data_proc,buf,8);
	}
#endif	
exit_write_proc_out:
	kfree(path_buf);
	return size;
}

static const struct file_operations gsl_config_proc_fops = {
	.read		= gsl_config_read_proc,
	.write		= gsl_config_write_proc,
};

#endif

static void gsl_read_version(char* tp_version)
{
#if defined(CONFIG_L6300_COMMON)
   sprintf(tp_version, "vid:%s,fw:%s,ic:%s\n","YeJi","Unknown","GXL968E");
#else
   sprintf(tp_version, "vid:%s,fw:%s,ic:%s\n","RongNa","Unknown","gsl968");
#endif
}

#ifdef GSL_TIMER
static void gsl_timer_check_func(struct work_struct *work)
{	
	u8 buf[4] = {0};
	u32 tmp;
	int i,flag=0;
	static int timer_count;
	if(1==ddata->gsl_halt_flag){
		return;
	}
	gsl_read_interface(ddata->client, 0xb4, buf, 4);
	tmp = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);

	print_info("[pre] 0xb4 = %x \n",ddata->gsl_timer_data);
	print_info("[cur] 0xb4 = %x \n",tmp);
	print_info("gsl_timer_flag=%d\n",ddata->gsl_timer_flag);
	if(0 == ddata->gsl_timer_flag)
	{
		if(tmp== ddata->gsl_timer_data)
		{
			ddata->gsl_timer_flag = 1;
			if(0==ddata->gsl_halt_flag)
			{
				queue_delayed_work(ddata->timer_wq, &ddata->timer_work, 25);
			}
		}
		else
		{
			for(i=0;i<5;i++){
				gsl_read_interface(ddata->client,0xbc,buf,4);
				if(buf[0]==0&&buf[1]==0&&buf[2]==0&&buf[3]==0)
				{
					flag = 1;
					break;
				}
				flag =0;
			}
			if(flag == 0){
				gsl_reset_core(ddata->client);
				gsl_start_core(ddata->client);
			}
			ddata->gsl_timer_flag = 0;
			timer_count = 0;
			if(0 == ddata->gsl_halt_flag)
			{
				queue_delayed_work(ddata->timer_wq, &ddata->timer_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
	}
	else if(1==ddata->gsl_timer_flag){
		if(tmp==ddata->gsl_timer_data)
		{
			if(0==ddata->gsl_halt_flag)
			{
				timer_count++;
				ddata->gsl_timer_flag = 2;
				gsl_sw_init(ddata->client);
				ddata->gsl_timer_flag = 1;
			}
			if(0 == ddata->gsl_halt_flag && timer_count < 20)
			{
				queue_delayed_work(ddata->timer_wq, &ddata->timer_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
		else{
			timer_count = 0;
			if(0 == ddata->gsl_halt_flag && timer_count < 20)
			{
				queue_delayed_work(ddata->timer_wq, &ddata->timer_work, GSL_TIMER_CHECK_CIRCLE);
			}
		}
		ddata->gsl_timer_flag = 0;
	}
	ddata->gsl_timer_data = tmp;
}
#endif

//
#if GSL_HAVE_TOUCH_KEY
static int gsl_report_key(struct input_dev *idev,int x,int y)
{
	int i;

	for(i=0;i<ddata->num_buttons;i++)
	{
		if(x > ddata->gsl_key_data[i].x_min &&
			x < ddata->gsl_key_data[i].x_max &&
			y > ddata->gsl_key_data[i].y_min &&
			y < ddata->gsl_key_data[i].y_max)
		{
			ddata->gsl_key_state = i+1;
			input_report_key(idev, BTN_TOUCH, 1);
			input_report_key(idev,ddata->gsl_key_data[i].key,1);
			input_sync(idev);
			return 1;
		}
	}
	return 0;
}
#endif
static void gsl_report_point(struct input_dev *idev, struct gsl_touch_info *cinfo)
{
	int i; 
	u32 gsl_point_state = 0;
	u32 temp=0;

	print_info("finger_num=%d\n", cinfo->finger_num);
	if(cinfo->finger_num>0 && cinfo->finger_num<6)
	{
		ddata->gsl_up_flag = 0;
		gsl_point_state = 0;
#if GSL_HAVE_TOUCH_KEY
		if(1==cinfo->finger_num)
		{
			if(cinfo->x[0] > ddata->x_max|| cinfo->y[0] > ddata->y_max)
			{
				print_info("x=%d, y=%d, x_max=%d, y_max=%d\n", cinfo->x[0], cinfo->y[0], ddata->x_max, ddata->y_max);
				gsl_report_key(idev,cinfo->x[0],cinfo->y[0]);
				return;		
			}
		}
#endif
		for(i=0;i<cinfo->finger_num;i++)
		{
			gsl_point_state |= (0x1<<cinfo->id[i]);	
			print_info("id = %d, x = %d, y = %d \n",cinfo->id[i], 
				cinfo->x[i],cinfo->y[i]);
		#ifdef GSL_REPORT_POINT_SLOT
			input_mt_slot(idev, cinfo->id[i] - 1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, cinfo->id[i]-1);
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, GSL_PRESSURE);
			input_report_abs(idev, ABS_MT_POSITION_X, cinfo->x[i]);
			input_report_abs(idev, ABS_MT_POSITION_Y, cinfo->y[i]);	
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 1);
		
		#else 
			input_report_key(idev, BTN_TOUCH, 1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, cinfo->id[i]-1);
			input_report_abs(idev, ABS_MT_TOUCH_MAJOR, GSL_PRESSURE);
			input_report_abs(idev, ABS_MT_POSITION_X, cinfo->x[i]);
			input_report_abs(idev, ABS_MT_POSITION_Y, cinfo->y[i]);	
			input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 1);
			input_mt_sync(idev);		
		#endif
		}
	}
	else if(cinfo->finger_num == 0)
	{
		gsl_point_state = 0;
		ddata->gsl_point_state = 0;
		if(1 == ddata->gsl_up_flag)
		{
			return;
		}
		ddata->gsl_up_flag = 1;
#if GSL_HAVE_TOUCH_KEY
		if(ddata->gsl_key_state > 0)
		{
			if(ddata->gsl_key_state < ddata->num_buttons+1)
			{
				input_report_key(idev,ddata->gsl_key_data[ddata->gsl_key_state - 1].key,0);
				input_sync(idev);
			}
		}
#endif
#ifndef GSL_REPORT_POINT_SLOT
		input_report_key(idev, BTN_TOUCH, 0);
		input_report_abs(idev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(idev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(idev);
#endif
	}

	temp = gsl_point_state & ddata->gsl_point_state;
	temp = (~temp) & ddata->gsl_point_state;
#ifdef GSL_REPORT_POINT_SLOT
	for(i=1;i<6;i++)
	{
		if(temp & (0x1<<i))
		{
			input_mt_slot(idev, i-1);
			input_report_abs(idev, ABS_MT_TRACKING_ID, -1);
			input_mt_report_slot_state(idev, MT_TOOL_FINGER, false);
		}
	}
#endif	
	ddata->gsl_point_state = gsl_point_state;
	input_sync(idev);
}


static void gsl_report_work(struct work_struct *work)
{
	int rc,tmp;
	u8 buf[44] = {0};
	int tmp1=0;


	struct gsl_touch_info *cinfo = ddata->cinfo;
	struct i2c_client *client = ddata->client;
	struct input_dev *idev = ddata->idev;
	
	if(1 == ddata->gsl_sw_flag)
		goto schedule;

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		goto schedule;
	}
#endif
#ifdef GSL_TIMER 
	if(2==ddata->gsl_timer_flag){
		goto schedule;
	}
#endif

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	if(tp_prox_sensor_opened == 1)
	{
		gsl_read_interface(ddata->client,0xac,buf,4);
		#ifdef DEBUG_TP_SENSOR
		print_info( "%s buf[0]=%x,buf[1]=%x,buf[2]=%x,buf[3]=%x\n", __func__,buf[0],buf[1],buf[2],buf[3]);
		#endif

		if (buf[0] == 1 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0)
		{
			tp_prox_sensor_data = 0;//near
		}
		else
		{
			tp_prox_sensor_data = 1;//far
		}

		if( tp_pre_sensor_data != tp_prox_sensor_data)
		{  
			print_info( "%s:  sensor data changed\n", __func__);
			tp_pre_sensor_data = tp_prox_sensor_data;
			mutex_lock(&tp_prox_sensor_mutex);
			tp_prox_sensor_data_changed = 1;
			mutex_unlock(&tp_prox_sensor_mutex);
			enable_irq(client->irq);
			return;
		}

		if(is_need_report_pointer == 0)
		{
		 	print_info( "%s: we don not report pointer when sleep in call\n", __func__);
			enable_irq(client->irq);
			return;
		}
	}
#endif

	/* read data from DATA_REG */
	rc = gsl_read_interface(client, 0x80, buf, 44);
	if (rc < 0) 
	{
		dev_err(&client->dev, "[gsl] I2C read failed\n");
		goto schedule;
	}

	if (buf[0] == 0xff) {
		goto schedule;
	}

	cinfo->finger_num = buf[0];
	for(tmp=0;tmp<(cinfo->finger_num>10 ? 10:cinfo->finger_num);tmp++)
	{
		cinfo->y[tmp] = (buf[tmp*4+4] | ((buf[tmp*4+5])<<8));
		cinfo->x[tmp] = (buf[tmp*4+6] | ((buf[tmp*4+7] & 0x0f)<<8));
		cinfo->id[tmp] = buf[tmp*4+7] >> 4;
		print_info("tp-gsl tmp=%d, x = %d y = %d \n", tmp, cinfo->x[tmp],cinfo->y[tmp]);
	}
	
	print_info("111 finger_num= %d\n",cinfo->finger_num);
#ifdef GSL_ALG_ID
	cinfo->finger_num = (buf[3]<<24)|(buf[2]<<16)|(buf[1]<<8)|(buf[0]);
	gsl_alg_id_main(cinfo);
	tmp1=gsl_mask_tiaoping();
	print_info("[tp-gsl] tmp1=%x\n",tmp1);
	if(tmp1>0&&tmp1<0xffffffff)
	{
		buf[0]=0xa;
		gsl_write_interface(client,0xf0,buf,4);
		buf[0]=(u8)(tmp1 & 0xff);
		buf[1]=(u8)((tmp1>>8) & 0xff);
		buf[2]=(u8)((tmp1>>16) & 0xff);
		buf[3]=(u8)((tmp1>>24) & 0xff);
		print_info("tmp1=%08x,buf[0]=%02x,buf[1]=%02x,buf[2]=%02x,buf[3]=%02x\n", tmp1,buf[0],buf[1],buf[2],buf[3]);
		gsl_write_interface(client,0x8,buf,4);
	}
#endif


	print_info("222 finger_num= %d\n",cinfo->finger_num);
	gsl_report_point(idev,cinfo);
	
schedule:
	enable_irq(client->irq);

}

static int gsl_request_input_dev(struct gsl_ts_data *ddata)

{
	struct input_dev *input_dev;
	int err;
#if GSL_HAVE_TOUCH_KEY
	int i;
#endif
	/*allocate input device*/
	print_info("==input_allocate_device=\n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		goto err_allocate_input_device_fail;
	}
	ddata->idev = input_dev;
	/*set input parameter*/	
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT,input_dev->propbit);
#ifdef GSL_REPORT_POINT_SLOT
	__set_bit(EV_REP, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT,input_dev->propbit);
	input_mt_init_slots(input_dev,5);
#else 
	__set_bit(BTN_TOUCH, input_dev->keybit);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID,0,5,0,0);
#endif
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);
#ifdef TOUCH_VIRTUAL_KEYS
	__set_bit(KEY_MENU,  input_dev->keybit);
	__set_bit(KEY_BACK,  input_dev->keybit);
	__set_bit(KEY_HOME,  input_dev->keybit);    
#endif
#if GSL_HAVE_TOUCH_KEY
	for(i=0;i<ddata->num_buttons;i++)
	{
		__set_bit(ddata->gsl_key_data[i].key, input_dev->keybit);
	}
#endif
	
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, ddata->x_max, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, ddata->y_max, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	input_dev->name = GSL_TS_NAME;		//dev_name(&client->dev)

	/*register input device*/
	err = input_register_device(input_dev);
	if (err) {
		goto err_register_input_device_fail;
	}
	return 0;
err_register_input_device_fail:
	input_free_device(input_dev);			
err_allocate_input_device_fail:
	return err;
}

static irqreturn_t gsl_ts_interrupt(int irq, void *dev_id)
{
	struct i2c_client *client = ddata->client;
	print_info("gslX68X_ts_interrupt\n");
	
	disable_irq_nosync(client->irq);
	if (!work_pending(&ddata->work)) {
		queue_work(ddata->wq, &ddata->work);
	}
	
	return IRQ_HANDLED;
}
#if defined(CONFIG_FB)
static int gsl_ts_suspend(void)
{
	u32 tmp;
	struct i2c_client *client = ddata->client;
	print_info("==gslX68X_ts_suspend=\n");
	//version info
	printk("[tp-gsl]the last time of debug:%x\n",TPD_DEBUG_TIME);

	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	if(tp_prox_sensor_opened == 1)
	{
		is_need_report_pointer = 0;
		printk("[tp-gsl] tp can not sleep in call\n");
		return 0;
	}
	#endif
	

	if(1==ddata->gsl_sw_flag)
		return 0;

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return 0;
	}
#endif
is_suspend = 0;
#ifdef GSL_ALG_ID
	tmp = gsl_version_id();	
	printk("[tp-gsl]the version of alg_id:%x\n",tmp);
#endif
	//version info
	ddata->gsl_halt_flag = 1;
#ifdef GSL_TIMER	
	cancel_delayed_work_sync(&ddata->timer_work);
	if(2==ddata->gsl_timer_flag){
		return 0 ;
	}
#endif
	disable_irq_nosync(client->irq);
	gpio_set_value(ddata->reset_gpio, 0);
	is_suspend = 1;
	return 0;
}

static int gsl_ts_resume(void)
{	
	struct i2c_client *client = ddata->client;
	print_info("==gslX68X_ts_resume=\n");
	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
	is_need_report_pointer = 1;
	if((tp_prox_sensor_opened == 1) && (is_suspend == 0))
	{
		printk("[tp-gsl] tp no need to wake up in call\n");
		return 0;
	}
	#endif

	if(1==ddata->gsl_sw_flag){
		ddata->gsl_halt_flag = 0;
		return 0;
	}

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		ddata->gsl_halt_flag = 0;
		return 0;
	}
#endif

#ifdef GSL_TIMER
	if(2==ddata->gsl_timer_flag)
	{
		ddata->gsl_halt_flag=0;
		enable_irq(client->irq);
		return 0;
	}
#endif
	gpio_set_value(ddata->reset_gpio, 1);
	msleep(20);
	gsl_reset_core(client);
	gsl_start_core(client);
	msleep(20);
	check_mem_data(client);
	enable_irq(client->irq);
#ifdef GSL_TIMER
	queue_delayed_work(ddata->timer_wq, &ddata->timer_work, GSL_TIMER_CHECK_CIRCLE);
	ddata->gsl_timer_flag = 0;
#endif
	ddata->gsl_halt_flag = 0;
	is_suspend = 0;
	return 0;

}
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	
	if (evdata && evdata->data && event == FB_EVENT_BLANK ){
		blank = evdata->data;
		print_info("fb_notifier_callback blank=%d\n",*blank);
		if (*blank == FB_BLANK_UNBLANK)
			gsl_ts_resume();
		else if (*blank == FB_BLANK_POWERDOWN)
			gsl_ts_suspend();
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void gsl_early_suspend(struct early_suspend *handler)
{
	u32 tmp;
	struct i2c_client *client = ddata->client;
	print_info("==gslX68X_ts_suspend=\n");
	//version info
	printk("[tp-gsl]the last time of debug:%x\n",TPD_DEBUG_TIME);

	if(1==ddata->gsl_sw_flag)
		return;

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		return;
	}
#endif
#ifdef GSL_ALG_ID
	tmp = gsl_version_id();	
	printk("[tp-gsl]the version of alg_id:%x\n",tmp);
#endif
	//version info
	ddata->gsl_halt_flag = 1;
#ifdef GSL_TIMER	
	cancel_delayed_work_sync(&ddata->timer_work);
	if(2==ddata->gsl_timer_flag){
		return;
	}
#endif
	disable_irq_nosync(client->irq);
	gpio_set_value(ddata->reset_gpio, 0);
}

static void gsl_early_resume(struct early_suspend *handler)
{	
	struct i2c_client *client = ddata->client;
	print_info("==gslX68X_ts_resume=\n");
	if(1==ddata->gsl_sw_flag){
		ddata->gsl_halt_flag = 0;
		return;
	}

#ifdef TPD_PROC_DEBUG
	if(gsl_proc_flag == 1){
		ddata->gsl_halt_flag = 0;
		return;
	}
#endif

#ifdef GSL_TIMER
	if(2==ddata->gsl_timer_flag)
	{
		ddata->gsl_halt_flag=0;
		enable_irq(client->irq);
		return;
	}
#endif
	gpio_set_value(ddata->reset_gpio, 1);
	msleep(20);
	gsl_reset_core(client);
	gsl_start_core(client);
	msleep(20);
	check_mem_data(client);
	enable_irq(client->irq);
#ifdef GSL_TIMER
	queue_delayed_work(ddata->timer_wq, &ddata->timer_work, GSL_TIMER_CHECK_CIRCLE);
	ddata->gsl_timer_flag = 0;
#endif
	ddata->gsl_halt_flag = 0;

}
#endif

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR

static void tp_prox_sensor_enable(int enable)
{
	uint8_t buf[4];
	int ret = -1;
	printk(KERN_ERR"%s  enable=%d\n",__func__,enable);
	if(i2c_prox_client==NULL) return;

	if(enable)
	{	
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		ret = gsl_write_interface(ddata->client, 0xf0, buf, 4);
		buf[3] = 0x0;
		buf[2] = 0x0;
		buf[1] = 0x0;
		buf[0] = 0x2;
		ret = gsl_write_interface(ddata->client, 0, buf, 4);
		
		if(ret < 0) 
		{
			printk(KERN_ERR "tp_sensor_enable on i2c_master_send failed \n");
		}

	}
	else
	{
		buf[3] = 0x00;
		buf[2] = 0x00;
		buf[1] = 0x00;
		buf[0] = 0x4;
		ret = gsl_write_interface(ddata->client, 0xf0, buf, 4);
		buf[3] = 0x0;
		buf[2] = 0x0;
		buf[1] = 0x0;
		buf[0] = 0x0;
		ret = gsl_write_interface(ddata->client, 0, buf, 4);

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
	struct i2c_client *client = to_i2c_client(dev);
	struct gsl_ts_data *prox = i2c_get_clientdata(client);
	struct input_dev *input_dev = prox->input_prox_dev;
	unsigned long data;
	int error;

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	pr_info("%s, data=%ld\n",__func__,data);
	#if 0
	if(data){
		if(tp_system_status == STATUS_SUSPENDED){
			tp_unset_status  = 1;
			pr_info("%s, should perform after touchpanel resumed!\n",__func__);
			return 0;
		}
	}
	#endif
	mutex_lock(&input_dev->mutex);
	disable_irq(client->irq);

	mutex_lock(&tp_prox_sensor_mutex);
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
	mutex_unlock(&tp_prox_sensor_mutex);
  
	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(enable, SYS_AUTHORITY,
			tp_prox_enable_show, tp_prox_enable_store);

/* Returns currently selected poll interval (in ms) */
static ssize_t tp_prox_get_poll_delay(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gsl_ts_data *ps = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", ps->poll_dev->poll_interval);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t tp_prox_set_poll_delay(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct gsl_ts_data *ps = i2c_get_clientdata(client);
	struct input_dev *input_dev = ps->input_prox_dev;
	unsigned int interval;
	int error;

	error = kstrtouint(buf, 10, &interval);
	if (error < 0)
		return error;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&input_dev->mutex);

	disable_irq(client->irq);

	/*
	 * Set current interval to the greater of the minimum interval or
	 * the requested interval
	 */
	ps->poll_dev->poll_interval = max((int)interval,(int)ps->poll_dev->poll_interval_min);

	enable_irq(client->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}

static DEVICE_ATTR(poll_delay, SYS_AUTHORITY,
			tp_prox_get_poll_delay, tp_prox_set_poll_delay);

static struct attribute *tp_prox_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static struct attribute_group tp_prox_attribute_group = {
	.attrs = tp_prox_attributes
};

static void __devinit tp_prox_init_input_device(struct gsl_ts_data *prox,
		struct input_dev *input_dev)
{
	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	input_dev->name = PROXIMITY_INPUT_DEV_NAME;
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &prox->client->dev;
}

static void tp_prox_poll(struct input_polled_dev *dev)
{
	struct gsl_ts_data *prox = dev->private;

	if (tp_prox_sensor_data_changed){
		mutex_lock(&tp_prox_sensor_mutex);
		//----[ENABLE_CHIP_RESET_MACHINE]------------------------------------------------------------------start	
			#ifdef ENABLE_CHIP_RESET_MACHINE	
			//queue_delayed_work(private_ts->himax_wq, &private_ts->himax_chip_reset_work, 0);
			#endif
		//----[ENABLE_CHIP_RESET_MACHINE]--------------------------------------------------------------------end
		tp_prox_sensor_data_changed = 0;
		mutex_unlock(&tp_prox_sensor_mutex);
		pr_info("%s poll tp_prox_sensor_data=%d\n",__func__,tp_prox_sensor_data);
		input_report_abs(prox->input_prox_dev, ABS_DISTANCE, tp_prox_sensor_data);
		input_sync(prox->input_prox_dev);
	}
}

static int __devinit tp_prox_setup_polled_device(struct gsl_ts_data *ps)
{
	int err;
	struct input_polled_dev *poll_dev;

	poll_dev = input_allocate_polled_device();
	if (!poll_dev) {
		dev_err(&ps->client->dev,
			"Failed to allocate polled device\n");
		return -ENOMEM;
	}

	ps->poll_dev = poll_dev;
	ps->input_prox_dev = poll_dev->input;

	poll_dev->private = ps;
	poll_dev->poll = tp_prox_poll;
	poll_dev->poll_interval = 100;
	poll_dev->poll_interval_min= 0;

	tp_prox_init_input_device(ps, poll_dev->input);
	err = input_register_polled_device(poll_dev);
	pr_info("%s, err=%d, poll-interval=%d\n",__func__,err,poll_dev->poll_interval);
	if (err) {
		dev_err(&ps->client->dev,
			"Unable to register polled device, err=%d\n", err);
		input_free_polled_device(poll_dev);
		return -ENOMEM;
	}

	return 0;
}

static void tp_prox_teardown_polled_device(struct gsl_ts_data *ps)
{
	input_unregister_polled_device(ps->poll_dev);
	input_free_polled_device(ps->poll_dev);
}
#endif//CONFIG_TP_PROXIMITY_SENSOR_CYTTSP

static int touch_driver_power_on(struct i2c_client *client, bool on)
{
	struct gsl_ts_data *data = i2c_get_clientdata(client);
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
	struct gsl_ts_data *data = i2c_get_clientdata(client);
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
	struct gsl_ts_data *data = i2c_get_clientdata(client);
	
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
	struct gsl_ts_data *data = i2c_get_clientdata(client);
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
static int touch_driver_get_dt_coords(struct device *dev, char *name, struct gsl_ts_data *pdata)
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

	if (!strcmp(name, "gsl,panel-coords"))
	{
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	}
	else if (!strcmp(name, "gsl,display-coords"))
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

static int touch_driver_parse_dt(struct device *dev, struct gsl_ts_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val;

	printk("%s\n", __func__);
	pdata->name = "gsl";
	rc = of_property_read_string(np, "gsl,name", &pdata->name);
	if (rc && (rc != -EINVAL))
	{
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = touch_driver_get_dt_coords(dev, "gsl,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = touch_driver_get_dt_coords(dev, "gsl,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np, "gsl,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np, "gsl,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "gsl,reset-gpio", 0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;
	else
		printk("%s, reset_gpio=%d\n", __func__, pdata->reset_gpio);

	pdata->irq_gpio = of_get_named_gpio_flags(np, "gsl,irq-gpio", 0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;
	else
		printk("%s, irq_gpio=%d\n", __func__, pdata->irq_gpio);

	/* power ldo gpio info*/
	pdata->power_ldo_gpio = of_get_named_gpio_flags(np, "gsl,power_ldo-gpio", 0, &pdata->power_ldo_gpio_flags);
	if (pdata->power_ldo_gpio < 0)
#if 0
		return pdata->power_ldo_gpio;
#else
		printk("%s, power_ldo_gpio=%d\n", __func__, pdata->power_ldo_gpio);
#endif
	
	rc = of_property_read_u32(np, "gsl,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "gsl,button-map", NULL);
	if (prop)
	{
		pdata->num_buttons = prop->length / sizeof(temp_val);
		if (pdata->num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np, "gsl,button-map", pdata->button_map, pdata->num_buttons);
		if (rc)
		{
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int touch_driver_parse_dt(struct device *dev, struct gsl_ts_data *pdata)
{
	return -ENODEV;
}
#endif

extern int is_tp_driver_loaded;
static int gsl_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0, i;
	char tp_version[50] = {0};
	
	printk("%s, is_tp_driver_loaded=%d\n", __func__, is_tp_driver_loaded);

	if(is_tp_driver_loaded == 1)
	{
		print_info(" other driver has been loaded\n");
		return ENODEV;
	}
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
        printk("%s: I2c doesn't work\n", __func__);
		goto exit_check_functionality_failed;
	}

    if (client->dev.of_node)
	{
		print_info("==kzalloc=");
		ddata = kzalloc(sizeof(struct gsl_ts_data), GFP_KERNEL);
		if (ddata==NULL)
		{
			err = -ENOMEM;
			goto exit_alloc_data_failed;
		}
	
		err = touch_driver_parse_dt(&client->dev, ddata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			goto Err_parse_dt;
		}
    }
	
	ddata->gsl_halt_flag = 0;
	ddata->gsl_sw_flag = 0;
	ddata->gsl_up_flag = 0;
	ddata->gsl_point_state = 0;
#if GSL_HAVE_TOUCH_KEY
	ddata->gsl_key_state = 0;
//++++>
	if (ddata->num_buttons)
	{
		ddata->gsl_key_data = kzalloc(sizeof(struct key_data) * ddata->num_buttons, GFP_KERNEL);
		if (!ddata->gsl_key_data)
		{
			err = -ENOMEM;
			goto Error_kzalloc_gsl_key_data;
		}

		for (i=0; i<ddata->num_buttons; ++i)
		{
			ddata->gsl_key_data[i].key = ddata->button_map[i];
			ddata->gsl_key_data[i].x_min = ddata->x_min + (ddata->x_max - ddata->x_min)/ddata->num_buttons * i;
			ddata->gsl_key_data[i].x_max = ddata->x_min + (ddata->x_max - ddata->x_min)/ddata->num_buttons * (i + 1);
			ddata->gsl_key_data[i].y_min = ddata->y_max + 1;
			ddata->gsl_key_data[i].y_max = ddata->panel_maxy;
		}
	}
//<++++		xuke @ 20140812		Parse virtual key values from DTS.
#endif	
	ddata->cinfo = kzalloc(sizeof(struct gsl_touch_info),GFP_KERNEL);
	if(ddata->cinfo == NULL)
	{
		err = -ENOMEM;
		goto exit_alloc_cinfo_failed;
	}
	mutex_init(&gsl_i2c_lock);
	
	ddata->client = client;
	i2c_set_clientdata(client, ddata);
	print_info("I2C addr=%x\n", client->addr);	
	
	gsl_hw_init(ddata);

	err = touch_driver_pinctrl_init(client);
	if (!err && ddata->ts_pinctrl)
	{
		err = touch_driver_pinctrl_select(client, true);
		if (err < 0)
			goto Err_pinctrl_init;
	}
	
	err = touch_driver_power_init(client, true);
	if (err) {
		dev_err(&client->dev, "power init failed");
		goto Err_power_init;
	}

	err = touch_driver_power_on(client, true);
	if (err) {
		dev_err(&client->dev, "power on failed");
		goto Err_power_on;
	}

	err = gsl_compatible_id(client);
	if(err < 0)
	{
		goto exit_i2c_transfer_fail; 
	}
	/*request input system*/	
	err = gsl_request_input_dev(ddata);
	if(err < 0)
	{
		goto exit_i2c_transfer_fail;	
	}

	
	/*register early suspend*/
	print_info("==register_early_suspend =\n");
	#if defined(CONFIG_FB)
	ddata->fb_notif.notifier_call = fb_notifier_callback;
	err = fb_register_client(&ddata->fb_notif);
	if (err)
		dev_err(&ddata->client->dev,
			"Unable to register fb_notifier: %d\n",
			err);
	#elif defined(CONFIG_HAS_EARLYSUSPEND)
	ddata->pm.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ddata->pm.suspend = gsl_early_suspend;
	ddata->pm.resume	= gsl_early_resume;
	register_early_suspend(&ddata->pm);
	#endif
	

	/*init work queue*/
	INIT_WORK(&ddata->work, gsl_report_work);
	ddata->wq = create_singlethread_workqueue(dev_name(&client->dev));
	if (!ddata->wq) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}

	/*request irq */
	client->irq = gpio_to_irq(ddata->irq_gpio);
	print_info("%s: ==request_irq=\n",__func__);
	print_info("%s IRQ number is %d\n", client->name, client->irq);
	err = request_irq(client->irq, gsl_ts_interrupt, IRQF_TRIGGER_RISING, client->name, ddata);
	if (err < 0) {
		dev_err(&client->dev, "gslX68X_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq_nosync(client->irq);
	/*gsl of software init*/
	gsl_sw_init(client);
	msleep(20);
	check_mem_data(client);
#ifdef TPD_PROC_DEBUG
	gsl_config_proc = proc_create_data(GSL_CONFIG_PROC_FILE, 0666, NULL, &gsl_config_proc_fops, NULL);
	if (IS_ERR_OR_NULL(gsl_config_proc))
	{
		pr_err("proc_create_data %s failed\n", GSL_CONFIG_PROC_FILE);
	}
	gsl_proc_flag = 0;
#endif

#ifdef GSL_TIMER
	INIT_DELAYED_WORK(&ddata->timer_work, gsl_timer_check_func);
	ddata->timer_wq = create_workqueue("gsl_timer_wq");
	queue_delayed_work(ddata->timer_wq, &ddata->timer_work, GSL_TIMER_CHECK_CIRCLE);
#endif

#ifdef TOUCH_VIRTUAL_KEYS
	gsl_ts_virtual_keys_init();
#endif

    gsl_read_version(tp_version);
    init_tp_fm_info(0, tp_version, "gsl968");
   

#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
#ifdef SENSOR_PROX_TP_USE_WAKELOCK
	wake_lock_init(&sensor_prox_tp_wake_lock, WAKE_LOCK_SUSPEND, "tp_sensor");
#endif

	i2c_prox_client = client;

	err = sysfs_create_group(&client->dev.kobj, &tp_prox_attribute_group);
	if (err) {
		dev_err(&client->dev, "sysfs create failed: %d\n", err);
	}

	err = tp_prox_setup_polled_device(ddata);
#endif

	lct_ctp_selftest_int(gsl_ctp_selftest);

	enable_irq(client->irq);
	is_tp_driver_loaded = 1;
	print_info("%s: ==probe over =\n",__func__);
	return 0;


exit_irq_request_failed:
	cancel_work_sync(&ddata->work);
	destroy_workqueue(ddata->wq);
exit_create_singlethread:
	#if defined(CONFIG_FB)
	if (fb_unregister_client(&ddata->fb_notif))
		dev_err(&client->dev,
			"Error occurred while unregistering fb_notifier.\n");
	#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ddata->pm);
	#endif
 
	input_unregister_device(ddata->idev);
	input_free_device(ddata->idev);
exit_i2c_transfer_fail:
Err_power_on:
	touch_driver_power_init(client, false);
Err_power_init:
	if (ddata->ts_pinctrl)
	{
		err = touch_driver_pinctrl_select(client, false);
		if (err < 0)
			pr_err("Cannot get idle pinctrl state\n");
	}
Err_pinctrl_init:
	if (ddata->ts_pinctrl)
		pinctrl_put(ddata->ts_pinctrl);

	gpio_free(ddata->reset_gpio);
	gpio_free(ddata->irq_gpio);
	i2c_set_clientdata(client, NULL);
	kfree(ddata->cinfo);
exit_alloc_cinfo_failed:
	kfree(ddata->gsl_key_data);
Error_kzalloc_gsl_key_data:
Err_parse_dt:
	kfree(ddata);
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __exit gsl_ts_remove(struct i2c_client *client)
{

	print_info("==gslX68X_ts_remove=\n");
	
#ifdef TPD_PROC_DEBUG
	if(gsl_config_proc!=NULL)
		remove_proc_entry(GSL_CONFIG_PROC_FILE, NULL);
#endif

#ifdef GSL_TIMER
	cancel_delayed_work_sync(&ddata->timer_work);
	destroy_workqueue(ddata->timer_wq);
#endif
	#if defined(CONFIG_FB)
	if (fb_unregister_client(&ddata->fb_notif))
		dev_err(&client->dev,
			"Error occurred while unregistering fb_notifier.\n");
	#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&ddata->pm);
	#endif

	#ifdef CONFIG_TOUCHPANEL_PROXIMITY_SENSOR
    sysfs_remove_group(&client->dev.kobj, &tp_prox_attribute_group);
    tp_prox_teardown_polled_device(ddata);
    #ifdef SENSOR_PROX_TP_USE_WAKELOCK
    wake_lock_destroy(&sensor_prox_tp_wake_lock);
    #endif
	#endif

	
	free_irq(client->irq,ddata);
	input_unregister_device(ddata->idev);
	input_free_device(ddata->idev);
	gpio_free(ddata->reset_gpio);
	gpio_free(ddata->irq_gpio);
	
	cancel_work_sync(&ddata->work);
	destroy_workqueue(ddata->wq);
	i2c_set_clientdata(client, NULL);
	//sprd_free_gpio_irq(client->irq);
	kfree(ddata->cinfo);
	kfree(ddata->gsl_key_data);
	kfree(ddata);

	return 0;
}


static const struct i2c_device_id gsl968_ts_id[] = {
	{ GSL_TS_NAME, GSL_TS_ADDR },
	{ }
};
static struct of_device_id gsl968_match_table[] = {
	{ .compatible = "gsl,gsl968",},
	{ },
};
static struct i2c_driver gsl968_ts_driver = {
	.driver = {
		.name = GSL_TS_NAME,
        .owner    = THIS_MODULE,
		.of_match_table = gsl968_match_table,
	},
	.probe = gsl_ts_probe,
	.remove = __exit_p(gsl_ts_remove),
	.id_table = gsl968_ts_id,
};
 

#if I2C_BOARD_INFO_METHOD
static int __init gsl_ts_init(void)
{
	print_info();	
	i2c_add_driver(&gsl968_ts_driver);
	return 0;
}

static void __exit gsl_ts_exit(void)
{
	print_info();
	i2c_del_driver(&gsl968_ts_driver);	
}
#else
static struct i2c_board_info gsl_i2c_info = {
	.type = GSL_TS_NAME,
	.addr = GSL_TS_ADDR,
}; 
static int __init gsl_ts_init(void)
{
	int ret;
	struct i2c_client *client;
	struct i2c_adapter *adapter;
	adapter = i2c_get_adapter(GSL_ADAPTER_INDEX);
	if(!adapter)
	{
		return -1;
	}
	client = i2c_new_device(adapter,&gsl_i2c_info);
	if(!client)
	{
		ret = -1;
		goto err_put_adapter;
	}
	i2c_put_adapter(adapter);

	ret = i2c_add_driver(&gsl968_ts_driver);
	if(ret < 0 )
	{
		goto err_add_driver;
	}
	return 0;

err_add_driver:
	kfree(client);
err_put_adapter:
	kfree(adapter);
	return ret;
}
static void __exit gsl_ts_exit(void)
{
	print_info();
	i2c_unregister_device(ddata->client);
	i2c_del_driver(&gsl968_ts_driver);	
}
#endif
module_init(gsl_ts_init);
module_exit(gsl_ts_exit);

MODULE_AUTHOR("sileadinc");
MODULE_DESCRIPTION("GSL Series Driver");
MODULE_LICENSE("GPL");

