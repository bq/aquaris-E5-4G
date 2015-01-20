/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <mach/gpiomux.h>
#include "msm_sensor.h"
#include "msm_cci.h"
#include "msm_camera_io_util.h"
#include "msm_camera_i2c_mux.h"


#define SP0A20_SENSOR_NAME "sp0a20"
DEFINE_MSM_MUTEX(sp0a20_mut);

#define CONFIG_MSMB_CAMERA_DEBUG
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

//sharpness                         
#if 1
//heq
  #define SP0A20_P1_0x10		0x7a//0x80		 //ku_outdoor
  #define SP0A20_P1_0x11		0x7a//0x80		//ku_nr
  #define SP0A20_P1_0x12		0x7a//0x80		 //ku_dummy
  #define SP0A20_P1_0x13		0x7a//0x80		 //ku_low  
  #define SP0A20_P1_0x14		0x80//0x88		//c4 //kl_outdoor 
  #define SP0A20_P1_0x15		0x80//0x88		//c4 //kl_nr      
  #define SP0A20_P1_0x16		0x80//0x88		//c4 //kl_dummy    
  #define SP0A20_P1_0x17		0x80//0x88		//c4 //kl_low   

//sharpness  
#define SP0A20_P2_0xe8         	0x50 //0x10//10//;sharp_fac_pos_outdoor
#define SP0A20_P2_0xec         	0x50 //0x20//20//;sharp_fac_neg_outdoor
#define SP0A20_P2_0xe9        	0x50 //0x0a//0a//;sharp_fac_pos_nr
#define SP0A20_P2_0xed         	0x50 //0x20//20//;sharp_fac_neg_nr
#define SP0A20_P2_0xea         	0x40 //0x08//08//;sharp_fac_pos_dummy
#define SP0A20_P2_0xee            0x40 //0x18//18//;sharp_fac_neg_dummy
#define SP0A20_P2_0xeb         	0x40 //0x08//08//;sharp_fac_pos_low
#define SP0A20_P2_0xef            0x40 //0x08//18//;sharp_fac_neg_low


//saturation

#define SP0A20_P1_0xd3		0x54//0x77
#define SP0A20_P1_0xd4		0x66//0x78
#define SP0A20_P1_0xd5  	    0x58//0x70
#define SP0A20_P1_0xd6  	    0x58//0x70
#define SP0A20_P1_0xd7		0x54//0x77
#define SP0A20_P1_0xd8		0x66//0x78
#define SP0A20_P1_0xd9   	    0x58//0x70
#define SP0A20_P1_0xda   	    0x58//0x70
#endif

//#define DEBUG_SENSOR_SP

#ifdef DEBUG_SENSOR_SP
#define SP_OP_CODE_INI		    0x00		/* Initial value. */
#define SP_OP_CODE_REG		0x01		/* Register */
#define SP_OP_CODE_DLY		0x02		/* Delay */
#define SP_OP_CODE_END		0x03		/* End of initial setting. */
uint16_t fromsd;

struct msm_camera_i2c_reg_conf SP_Init_Reg[1000];
struct msm_camera_i2c_reg_conf SP_Start_Reg[1000];
struct msm_camera_i2c_reg_conf SP_Stop_Reg[1000];


u32 strtol(const char *nptr, u8 base)
{
	u8 ret;
	if(!nptr || (base!=16 && base!=10 && base!=8))
	{
		printk("%s(): NULL pointer input\n", __FUNCTION__);
		return -1;
	}
	for(ret=0; *nptr; nptr++)
	{
		if((base==16 && *nptr>='A' && *nptr<='F') || 
				(base==16 && *nptr>='a' && *nptr<='f') || 
				(base>=10 && *nptr>='0' && *nptr<='9') ||
				(base>=8 && *nptr>='0' && *nptr<='7') )
		{
			ret *= base;
			if(base==16 && *nptr>='A' && *nptr<='F')
				ret += *nptr-'A'+10;
			else if(base==16 && *nptr>='a' && *nptr<='f')
				ret += *nptr-'a'+10;
			else if(base>=10 && *nptr>='0' && *nptr<='9')
				ret += *nptr-'0';
			else if(base>=8 && *nptr>='0' && *nptr<='7')
				ret += *nptr-'0';
		}
		else
			return ret;
	}
	return ret;
}

u32 reg_num = 0;
u32 reg_start_num = 0;
u32 reg_stop_num = 0;

u8 SP_Initialize_from_T_Flash(void)
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */


	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static u8 data_buff[10*1024] ;

	fp = filp_open("/system/lib/sp_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) { 
		printk("create file error %x \n", (unsigned int)fp); 
		return 0; 
	} 
	fs = get_fs(); 
	set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	//printk("%s %d %d\n", buf,iFileLen,pos); 
	filp_close(fp, NULL); 
	set_fs(fs);

	reg_num = 0;

	/* Start parse the setting witch read from t-flash. */
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size))
	{
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
		{
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
			{
				curr_ptr++;		/* Skip block comment code. */
			}

			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}

		if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
		{
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}
		/* This just content one enter line. */
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		//printk(" curr_ptr1 = %s\n",curr_ptr);
		memcpy(func_ind, curr_ptr, 3);


		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */

			SP_Init_Reg[i].reg_addr = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */

			SP_Init_Reg[i].reg_data = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */

			reg_num = i;
			//printk("i %d, reg_num %d \n", i, reg_num);
		}
		
		i++;


		/* Skip to next line directly. */
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}

	
	return 1;	
}
u8 SP_Start_Initialize_from_T_Flash(void)
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */


	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static u8 data_buff[10*1024] ;

	fp = filp_open("/system/lib/sp_start_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) { 
		printk("create file error %x \n", (unsigned int)fp); 
		return 0; 
	} 
	fs = get_fs(); 
	set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	//printk("%s %d %d\n", buf,iFileLen,pos); 
	filp_close(fp, NULL); 
	set_fs(fs);

	reg_start_num = 0;

	/* Start parse the setting witch read from t-flash. */
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size))
	{
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
		{
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
			{
				curr_ptr++;		/* Skip block comment code. */
			}

			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}

		if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
		{
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}
		/* This just content one enter line. */
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		//printk(" curr_ptr1 = %s\n",curr_ptr);
		memcpy(func_ind, curr_ptr, 3);


		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */

			SP_Start_Reg[i].reg_addr = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */

			SP_Start_Reg[i].reg_data = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */

			reg_start_num = i;
			//printk("i %d, reg_sgva_num %d \n", i, reg_sgva_num);
		}
		
		i++;


		/* Skip to next line directly. */
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}

	
	return 1;	
}
u8 SP_Stop_Initialize_from_T_Flash(void)
{
	//FS_HANDLE fp = -1;				/* Default, no file opened. */
	//u8 *data_buff = NULL;
	u8 *curr_ptr = NULL;
	u32 file_size = 0;
	//u32 bytes_read = 0;
	u32 i = 0;
	u8 func_ind[4] = {0};	/* REG or DLY */


	struct file *fp; 
	mm_segment_t fs; 
	loff_t pos = 0; 
	static u8 data_buff[10*1024] ;

	fp = filp_open("/system/lib/sp_stop_sd", O_RDONLY , 0); 
	if (IS_ERR(fp)) { 
		printk("create file error %x \n", (unsigned int)fp); 
		return 0; 
	} 
	fs = get_fs(); 
	set_fs(KERNEL_DS); 

	file_size = vfs_llseek(fp, 0, SEEK_END);
	vfs_read(fp, data_buff, file_size, &pos); 
	//printk("%s %d %d\n", buf,iFileLen,pos); 
	filp_close(fp, NULL); 
	set_fs(fs);

	reg_stop_num = 0;

	/* Start parse the setting witch read from t-flash. */
	curr_ptr = data_buff;
	while (curr_ptr < (data_buff + file_size))
	{
		while ((*curr_ptr == ' ') || (*curr_ptr == '\t'))/* Skip the Space & TAB */
			curr_ptr++;				

		if (((*curr_ptr) == '/') && ((*(curr_ptr + 1)) == '*'))
		{
			while (!(((*curr_ptr) == '*') && ((*(curr_ptr + 1)) == '/')))
			{
				curr_ptr++;		/* Skip block comment code. */
			}

			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}

		if (((*curr_ptr) == '/') || ((*curr_ptr) == '{') || ((*curr_ptr) == '}'))		/* Comment line, skip it. */
		{
			while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
			{
				curr_ptr++;
			}

			curr_ptr += 2;						/* Skip the enter line */

			continue ;
		}
		/* This just content one enter line. */
		if (((*curr_ptr) == 0x0D) && ((*(curr_ptr + 1)) == 0x0A))
		{
			curr_ptr += 2;
			continue ;
		}
		//printk(" curr_ptr1 = %s\n",curr_ptr);
		memcpy(func_ind, curr_ptr, 3);


		if (strcmp((const char *)func_ind, "REG") == 0)		/* REG */
		{
			curr_ptr += 6;				/* Skip "REG(0x" or "DLY(" */

			SP_Stop_Reg[i].reg_addr = strtol((const char *)curr_ptr, 16);
			curr_ptr += 5;	/* Skip "00, 0x" */

			SP_Stop_Reg[i].reg_data = strtol((const char *)curr_ptr, 16);
			curr_ptr += 4;	/* Skip "00);" */

			reg_stop_num = i;
			//printk("i %d, reg_sgva_num %d \n", i, reg_sgva_num);
		}
		
		i++;


		/* Skip to next line directly. */
		while (!((*curr_ptr == 0x0D) && (*(curr_ptr+1) == 0x0A)))
		{
			curr_ptr++;
		}
		curr_ptr += 2;
	}

	
	return 1;	
}
#endif
static struct msm_sensor_ctrl_t sp0a20_s_ctrl;

static struct msm_sensor_power_setting sp0a20_power_setting[] = {
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_VANA,
        .config_val = GPIO_OUT_LOW,
        .delay = 1,
    },
    {
        .seq_type = SENSOR_GPIO,
        .seq_val = SENSOR_GPIO_VANA,
        .config_val = GPIO_OUT_HIGH,
        .delay = 1,
    },
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_VDIG,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 24000000,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_STANDBY,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 1,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 1,
	},
};

static struct msm_camera_i2c_reg_conf sp0a20_start_settings[] = {
	{0xfd, 0x00},
	{0x0c, 0x00},
	{0x92, 0x11},//01 
	//{0x99, 0x05},
	{0xfd, 0x01},
	{0x36, 0x00},	
	{0xfd, 0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_stop_settings[] = {
	{0xfd, 0x00},
	{0x0c, 0x01},
	{0x92, 0x00},
	//{0x99, 0x00},
	{0xfd, 0x01},
	{0x36, 0x02},
	{0xfd, 0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_recommend_settings[] = {
      #if 1            //2014-9-28 from e5255 change ccm
		{0xfd,0x01},
		{0x36,0x02},
		{0xfd,0x00},
		{0x0c,0x00},
		{0x12,0x02},
		{0x13,0x2f},
		{0x6d,0x32},
		{0x6c,0x32},
		{0x6f,0x33},
		{0x6e,0x34},
		{0x99,0x04},
		{0x16,0x38},
		{0x17,0x38},
		{0x70,0x3a},
		{0x14,0x02},
		{0x15,0x20},
		{0x71,0x23},
		{0x69,0x25},
		{0x6a,0x1a},
		{0x72,0x1c},
		{0x75,0x1e},
		{0x73,0x3c},
		{0x74,0x21},
		{0x79,0x00},
		{0x77,0x10},
		{0x1a,0x4d},
		{0x1b,0x27},
		{0x1c,0x07},
		{0x1e,0x15},
		{0x21,0x0e},
		{0x22,0x28},
		{0x26,0x66},
		{0x28,0x0b},
		{0x37,0x5a},
		{0xfd,0x02},
		{0x01,0x80},
		{0x52,0x10},
		{0x54,0x00},
		{0x56,0x03},
		{0xfd,0x01},
		{0x41,0x00},
		{0x42,0x00},
		{0x43,0x00},
		{0x44,0x00},
		{0xfd,0x00},
		{0x03,0x02},
		{0x04,0x58},
		{0x05,0x00},
		{0x06,0x00},
		{0x07,0x00},
		{0x08,0x00},
		{0x09,0x01},
		{0x0a,0x64},
		{0xfd,0x01},
		{0xf0,0x00},
		{0xf7,0x64},
		{0x02,0x0e},
		{0x03,0x01},
		{0x06,0x64},
		{0x07,0x00},
		{0x08,0x01},
		{0x09,0x00},
		{0xfd,0x02},
		{0xbe,0x78},
		{0xbf,0x05},
		{0xd0,0x78},
		{0xd1,0x05},
		{0xfd,0x01},
		{0x5a,0x40},
		{0xfd,0x02},
		{0xbc,0x70},
		{0xbd,0x50},
		{0xb8,0x70},
		{0xb9,0x90},
		{0xba,0x30},
		{0xbb,0x45},
		{0xfd,0x01},
		{0xe0,0x44},
		{0xe1,0x36},
		{0xe2,0x30},
		{0xe3,0x2a},
		{0xe4,0x2a},
		{0xe5,0x28},
		{0xe6,0x28},
		{0xe7,0x26},
		{0xe8,0x26},
		{0xe9,0x26},
		{0xea,0x24},
		{0xf3,0x24},
		{0xf4,0x24},
		{0xfd,0x01},
		{0x04,0xa0},
		{0x05,0x24},
		{0x0a,0xa0},
		{0x0b,0x24},
		{0xfd,0x01},
		{0xeb,0x6c},
		{0xec,0x68},
		{0xed,0x05},
		{0xee,0x0a},
		{0xfd,0x01},
		{0xf2,0x4d},
		{0xfd,0x02},
		{0x54,0x0a},
		{0x5b,0x05},
		{0x5c,0xa0},
		{0xfd,0x01},
		{0x26,0x80},
		{0x27,0x4f},
		{0x28,0x00},
		{0x29,0x20},
		{0x2a,0x00},
		{0x2b,0x03},
		{0x2c,0x00},
		{0x2d,0x20},
		{0x30,0x00},
		{0x31,0x00},
		{0xfd,0x01},
		{0xa1,0x4c},
		{0xa2,0x4c},
		{0xa3,0x4c},
		{0xa4,0x4c},
		{0xa5,0x4c},
		{0xa6,0x4c},
		{0xa7,0x4c},
		{0xa8,0x4c},
		{0xa9,0x4c},
		{0xaa,0x4c},
		{0xab,0x4c},
		{0xac,0x4c},
		{0xad,0x0e},
		{0xae,0x08},
		{0xaf,0x0a},
		{0xb0,0x08},
		{0xb1,0x06},
		{0xb2,0x01},
		{0xb3,0x04},
		{0xb4,0x01},
		{0xb5,0x07},
		{0xb6,0x00},
		{0xb7,0x00},
		{0xb8,0x00},
		{0xfd,0x02},
		{0x1d,0x03},
		{0x1f,0x04},
		{0x08,0x00},
		{0x09,0x06},
		{0x0b,0x2f},
		{0x0d,0x0e},
		{0xfd,0x01},
		{0x32,0x00},
		{0xfd,0x02},
		{0x26,0xa0},
		{0x27,0xa8},
		{0x10,0x00},
		{0x11,0x00},
		{0x1b,0x80},
		{0x1a,0x80},
		{0x18,0x27},
		{0x19,0x26},
		{0x2a,0x01},
		{0x2b,0x10},
		{0x28,0xe0},
		{0x29,0x08},
		{0xe7,0x03},
		{0xe7,0x00},
		{0x66,0x4b},
		{0x67,0x6b},
		{0x68,0xD1},
		{0x69,0x10},
		{0x6a,0xa6},
		{0xe7,0x03},
		{0xe7,0x00},
		{0x7c,0x30},
		{0x7d,0x70},
		{0x7e,0xf0},
		{0x7f,0x10},
		{0x80,0xa6},
		{0xe7,0x03},
		{0xe7,0x00},
		{0x70,0x2d},
		{0x71,0x4f},
		{0x72,0x23},
		{0x73,0x41},
		{0x74,0xaa},
		{0xe7,0x03},
		{0xe7,0x00},
		{0x6b,0x10},
		{0x6c,0x2B},
		{0x6d,0x20},
		{0x6e,0x40},
		{0x6f,0xaa},
		{0xe7,0x03},
		{0xe7,0x00},
		{0x61,0xe5},
		{0x62,0x05},
		{0x63,0x4f},
		{0x64,0x6f},
		{0x65,0x6A},
		{0xe7,0x03},
		{0xe7,0x00},
		{0x75,0x80},
		{0x76,0x09},
		{0x77,0x02},
		{0x24,0x25},
		{0x0e,0x16},
		{0x3b,0x09},
		{0xfd,0x02},
		{0xde,0x0f},
		{0xd7,0x0c},
		{0xd8,0x10},
		{0xd9,0x12},
		{0xda,0x12},
		{0xe8,SP0A20_P2_0xe8},
		{0xe9,SP0A20_P2_0xe9},
		{0xea,SP0A20_P2_0xea},
		{0xeb,SP0A20_P2_0xeb},
		{0xec,SP0A20_P2_0xec},
		{0xed,SP0A20_P2_0xed},
		{0xee,SP0A20_P2_0xee},
		{0xef,SP0A20_P2_0xef},
		{0xd3,0x30},
		{0xd4,0x50},
		{0xd5,0x30},
		{0xd6,0x18},
		{0xfd,0x01},
		{0xd1,0x20},
		{0xfd,0x02},
		{0xdc,0x06},
		{0x05,0x20},
		{0xfd,0x02},
		{0x81,0x00},
		{0xfd,0x01},
		{0xfc,0x00},
		{0x7d,0x05},
		{0x7e,0x05},
		{0x7f,0x09},
		{0x80,0x08},
		{0xfd,0x02},
		{0xdd,0x07},
		{0xfd,0x01},
		{0x86,0x20},
		{0x6d,0x08},
		{0x6e,0x08},
		{0x6f,0x10},
		{0x70,0x18},
		{0x86,0x18},
		{0x71,0x0a},
		{0x72,0x0a},
		{0x73,0x14},
		{0x74,0x14},
		{0x75,0x08},
		{0x76,0x0a},
		{0x77,0x06},
		{0x78,0x06},
		{0x79,0x25},
		{0x7a,0x23},
		{0x7b,0x22},
		{0x7c,0x00},
		{0x81,0x0d},
		{0x82,0x18},
		{0x83,0x20},
		{0x84,0x24},
		{0xfd,0x02},
		{0x83,0x10},
		{0x83,0x12},
		{0x84,0x14},
		{0x86,0x04},
		{0xfd,0x01},
		{0x61,0x60},
		{0x62,0x28},
		{0x8a,0x10},
		{0xfd,0x01},
		{0x8b,0x00},
		{0x8c,0x09},
		{0x8d,0x13},
		{0x8e,0x1b},
		{0x8f,0x23},
		{0x90,0x32},
		{0x91,0x41},
		{0x92,0x4d},
		{0x93,0x5c},
		{0x94,0x70},
		{0x95,0x84},
		{0x96,0x94},
		{0x97,0xa3},
		{0x98,0xb5},
		{0x99,0xc2},
		{0x9a,0xcf},
		{0x9b,0xd9},
		{0x9c,0xe3},
		{0x9d,0xec},
		{0x9e,0xf4},
		{0x9f,0xfa},
		{0xa0,0xff},
		{0xfd,0x02},
#ifdef CONFIG_L6140_COMMON

		{0x15,0xca},
		{0x16,0x8a},
		{0xa0,0xa6},
		{0xa1,0xf4},
		{0xa2,0xe6},
		{0xa3,0xf6},
		{0xa4,0xac},
		{0xa5,0xdf},
		{0xa6,0xf4},
		{0xa7,0xcd},
		{0xa8,0xbf},
		{0xa9,0x3c},
		{0xaa,0x33},
		{0xab,0x0f},
		{0xac,0x99},
		{0xad,0x0c},
		{0xae,0xdb},
		{0xaf,0xed},
		{0xb0,0xb9},
		{0xb1,0xda},
		{0xb2,0xd7},
		{0xb3,0xd1},
		{0xb4,0xd8},
		{0xb5,0x30},
		{0xb6,0x33},
#else
		{0x15,0xc4},
		{0x16,0x8c},
		{0xa0,0x80},
		{0xa1,0xed},
		{0xa2,0x13},
		{0xa3,0xf4},
		{0xa4,0x80},
		{0xa5,0x0c},
		{0xa6,0x00},
		{0xa7,0xcd},
		{0xa8,0xb3},
		{0xa9,0x0c},
		{0xaa,0x03},
		{0xab,0x0c},
		{0xac,0x80},
		{0xad,0xf4},
		{0xae,0x0c},
		{0xaf,0xe7},
		{0xb0,0x80},
		{0xb1,0x18},
		{0xb2,0xe0},
		{0xb3,0xb4},
		{0xb4,0xec},
		{0xb5,0x0c},
		{0xb6,0x03},
#endif
		{0xb7,0x0f},
		{0xfd,0x01},
		{0xd3,SP0A20_P1_0xd3},
		{0xd4,SP0A20_P1_0xd4},
		{0xd5,SP0A20_P1_0xd5},
		{0xd6,SP0A20_P1_0xd6},
		{0xd7,SP0A20_P1_0xd7},
		{0xd8,SP0A20_P1_0xd8},
		{0xd9,SP0A20_P1_0xd9},
		{0xda,SP0A20_P1_0xda},
		{0xfd,0x01},
		{0xdd,0x30},
		{0xde,0x10},
		{0xdf,0xff},
		{0x00,0x00},
		{0xfd,0x01},
		{0xc2,0xaa},
		{0xc3,0x88},
		{0xc4,0x77},
		{0xc5,0x66},
		{0xfd,0x01},
		{0xcd,0x10},
		{0xce,0x1f},
		{0xcf,0x30},
		{0xd0,0x45},
		{0xfd,0x02},
		{0x31,0x60},
		{0x32,0x60},
		{0x33,0xc0},
		{0x35,0x60},
		{0x36,0x30},
		{0x37,0x13},
		{0xfd,0x01},
		{0x0e,0x80},
		{0x0f,0x20},
		{0x10,SP0A20_P1_0x10},
		{0x11,SP0A20_P1_0x11},
		{0x12,SP0A20_P1_0x12},
		{0x13,SP0A20_P1_0x13},
		{0x14,SP0A20_P1_0x14},
		{0x15,SP0A20_P1_0x15},
		{0x16,SP0A20_P1_0x16},
		{0x17,SP0A20_P1_0x17},
		{0xfd,0x00},
		{0x31,0x07},
		{0xfd,0x01},
		{0x32,0x15},
		{0x33,0xef},
		{0x34,0x07},
		{0xd2,0x01},
		{0xfb,0x25},
		{0xf2,0x49},
		{0x35,0x40},
		{0x5d,0x11},
		{0xfd,0x01},
		{0x36,0x01},
    #else
{0xfd,0x01},
{0x36,0x02},
{0xfd,0x00},
{0x0c,0x00},
{0x12,0x02},
{0x13,0x2f},
{0x6d,0x32},
{0x6c,0x32},
{0x6f,0x33},
{0x6e,0x34},
{0x99,0x04},
{0x16,0x38},
{0x17,0x38},
{0x70,0x3a},
{0x14,0x02},
{0x15,0x20},
{0x71,0x23},
{0x69,0x25},
{0x6a,0x1a},
{0x72,0x1c},
{0x75,0x1e},
{0x73,0x3c},
{0x74,0x21},
{0x79,0x00},
{0x77,0x10},
{0x1a,0x4d},
{0x1b,0x27},
{0x1c,0x07},
{0x1e,0x15},
{0x21,0x0e},
{0x22,0x28},
{0x26,0x66},
{0x28,0x0b},
{0x37,0x5a},
{0xfd,0x02},
{0x01,0x80},
{0x52,0x10},
{0x54,0x00},
{0x56,0x03},
{0xfd,0x01},
{0x41,0x00},
{0x42,0x00},
{0x43,0x00},
{0x44,0x00},
{0xfd,0x00},
{0x03,0x01},
{0x04,0x0e},
{0x05,0x00},
{0x06,0x00},
{0x07,0x00},
{0x08,0x00},
{0x09,0x07},
{0x0a,0x1f},
{0xfd,0x01},
{0xf0,0x00},
{0xf7,0x2d},
{0x02,0x0b},
{0x03,0x01},
{0x06,0x2d},
{0x07,0x00},
{0x08,0x01},
{0x09,0x00},
{0xfd,0x02},
{0xbe,0xef},
{0xbf,0x01},
{0xd0,0xef},
{0xd1,0x01},
{0xfd,0x01},
{0x5a,0x40},
{0xfd,0x02},
{0xbc,0x70},
{0xbd,0x50},
{0xb8,0x66},
{0xb9,0x88},
{0xba,0x30},
{0xbb,0x45},
{0xfd,0x01},
{0xe0,0x44},
{0xe1,0x36},
{0xe2,0x30},
{0xe3,0x2a},
{0xe4,0x2a},
{0xe5,0x28},
{0xe6,0x28},
{0xe7,0x26},
{0xe8,0x26},
{0xe9,0x26},
{0xea,0x24},
{0xf3,0x24},
{0xf4,0x24},
{0xfd,0x01},
{0x04,0xa0},
{0x05,0x24},
{0x0a,0xa0},
{0x0b,0x24},
{0xfd,0x01},
{0xeb,0x78},
{0xec,0x78},
{0xed,0x05},
{0xee,0x0c},
{0xfd,0x01},
{0xf2,0x4d},
{0xfd,0x02},
{0x54,0x0a},
{0x5b,0x05},
{0x5c,0xa0},
{0xfd,0x01},
{0x26,0x80},
{0x27,0x4f},
{0x28,0x00},
{0x29,0x20},
{0x2a,0x00},
{0x2b,0x03},
{0x2c,0x00},
{0x2d,0x20},
{0x30,0x00},
{0x31,0x00},
{0xfd,0x01},
{0xa1,0x31},
{0xa2,0x33},
{0xa3,0x2c},
{0xa4,0x37},
{0xa5,0x26},
{0xa6,0x25},
{0xa7,0x24},
{0xa8,0x30},
{0xa9,0x23},
{0xaa,0x23},
{0xab,0x20},
{0xac,0x2c},
{0xad,0x06},
{0xae,0x01},
{0xaf,0x2c},
{0xb0,0x08},
{0xb1,0x00},
{0xb2,0x00},
{0xb3,0x20},
{0xb4,0x00},
{0xb5,0x00},
{0xb6,0x00},
{0xb7,0x20},
{0xb8,0x00},
{0xfd,0x02},
{0x1d,0x00},
{0x1f,0x05},
{0x08,0x03},
{0x09,0x05},
{0x0b,0x2f},
{0x0d,0x0e},
{0xfd,0x01},
{0x32,0x00},
{0xfd,0x02},
{0x26,0xa0},
{0x27,0xa8},
{0x10,0x00},
{0x11,0x00},
{0x1b,0x80},
{0x1a,0x80},
{0x18,0x27},
{0x19,0x26},
{0x2a,0x01},
{0x2b,0x10},
{0x28,0xf8},
{0x29,0x08},
{0x66,0x4a},
{0x67,0x6c},
{0x68,0xd0},
{0x69,0xf4},
{0x6a,0xa5},
{0x7c,0x30},
{0x7d,0x4b},
{0x7e,0xf7},
{0x7f,0x13},
{0x80,0xa6},
{0x70,0x26},
{0x71,0x4a},
{0x72,0x25},
{0x73,0x46},
{0x74,0xaa},
{0x6b,0x0a},
{0x6c,0x31},
{0x6d,0x2e},
{0x6e,0x52},
{0x6f,0xaa},
{0x61,0xe8},
{0x62,0x18},
{0x63,0x40},
{0x64,0x6e},
{0x65,0x6a},
{0x75,0x80},
{0x76,0x09},
{0x77,0x02},
{0x24,0x25},
{0x0e,0x16},
{0x3b,0x09},
{0xfd,0x02},
{0xde,0x0f},
{0xd7,0x08},
{0xd8,0x08},
{0xd9,0x10},
{0xda,0x14},
  
{0xe8,SP0A20_P2_0xe8},   
{0xe9,SP0A20_P2_0xec},   
{0xea,SP0A20_P2_0xe9},   
{0xeb,SP0A20_P2_0xed},   
{0xec,SP0A20_P2_0xea},   
{0xed,SP0A20_P2_0xee},   
{0xee,SP0A20_P2_0xeb},   
{0xef,SP0A20_P2_0xef},   
{0xe8,0x20},
{0xe9,0x20},
{0xea,0x20},
{0xeb,0x20},
{0xec,0x20},
{0xed,0x20},
{0xee,0x20},
{0xef,0x20},
{0xd3,0x20},
{0xd4,0x48},
{0xd5,0x20},
{0xd6,0x08},
{0xfd,0x01},
{0xd1,0x20},
{0xfd,0x02},
{0xdc,0x05},
{0x05,0x20},
{0xfd,0x02},
{0x81,0x00},
{0xfd,0x01},
{0xfc,0x00},
{0x7d,0x05},
{0x7e,0x05},
{0x7f,0x09},
{0x80,0x08},
{0xfd,0x02},
{0xdd,0x0f},
{0xfd,0x01},
{0x86,0x20},
{0x6d,0x08},
{0x6e,0x08},
{0x6f,0x10},
{0x70,0x18},
{0x86,0x18},
{0x71,0x0a},
{0x72,0x0a},
{0x73,0x14},
{0x74,0x14},
{0x75,0x0c},
{0x76,0x0d},
{0x77,0x0a},
{0x78,0x0a},
{0x79,0x29},
{0x7a,0x27},
{0x7b,0x26},
{0x7c,0x04},
{0x81,0x0d},
{0x82,0x18},
{0x83,0x20},
{0x84,0x24},
{0xfd,0x02},
{0x83,0x10},
{0x83,0x12},
{0x84,0x14},
{0x86,0x04},
{0xfd,0x01},
{0x61,0x60},
{0x62,0x28},
{0x8a,0x10},
{0xfd,0x01},
{0x8b,0x00},
{0x8c,0x0a},
{0x8d,0x18},
{0x8e,0x2b},
{0x8f,0x3b},
{0x90,0x4b},
{0x91,0x57},
{0x92,0x64},
{0x93,0x73},
{0x94,0x84},
{0x95,0x95},
{0x96,0xa5},
{0x97,0xb3},
{0x98,0xc0},
{0x99,0xcc},
{0x9a,0xd6},
{0x9b,0xdf},
{0x9c,0xe7},
{0x9d,0xee},
{0x9e,0xf4},
{0x9f,0xfa},
{0xa0,0xff},
{0xfd,0x02},
{0x15,0xc0},
{0x16,0x8c},
{0xa0,0xa0},
{0xa1,0xfa},
{0xa2,0xe7},
{0xa3,0xe7},
{0xa4,0xc0},
{0xa5,0xda},
{0xa6,0xed},
{0xa7,0xda},
{0xa8,0xb9},
{0xa9,0x3c},
{0xaa,0x33},
{0xab,0x0f},
{0xac,0x80},
{0xad,0x19},
{0xae,0xe7},
{0xaf,0xcd},
{0xb0,0xd9},
{0xb1,0xda},
{0xb2,0xda},
{0xb3,0x80},
{0xb4,0x26},
{0xb5,0x30},
{0xb6,0x33},
{0xb7,0x1f},
  
{0xfd,0x01},   
{0xd3,SP0A20_P1_0xd3},   
{0xd4,SP0A20_P1_0xd4},   
{0xd5,SP0A20_P1_0xd5},   
{0xd6,SP0A20_P1_0xd6},   
{0xd7,SP0A20_P1_0xd7},   
{0xd8,SP0A20_P1_0xd8},   
{0xd9,SP0A20_P1_0xd9},   
{0xda,SP0A20_P1_0xda},   
{0xfd,0x01},
{0xdd,0x30},
{0xde,0x10},
{0xdf,0xff},
{0x00,0x00},
{0xfd,0x01},
{0xc2,0xaa},
{0xc3,0x88},
{0xc4,0x77},
{0xc5,0x66},
{0xfd,0x01},
{0xcd,0x10},
{0xce,0x1f},
{0xcf,0x30},
{0xd0,0x45},
{0xfd,0x02},
{0x31,0x60},
{0x32,0x60},
{0x33,0xc0},
{0x35,0x60},
{0x36,0x30},
{0x37,0x13},
{0xfd,0x01},
{0x0e,0x80},
{0x0f,0x20},

{0x10,SP0A20_P1_0x10},   
{0x11,SP0A20_P1_0x11},   
{0x12,SP0A20_P1_0x12},   
{0x13,SP0A20_P1_0x13},   
{0x14,SP0A20_P1_0x14},   
{0x15,SP0A20_P1_0x15},   
{0x16,SP0A20_P1_0x16},   
{0x17,SP0A20_P1_0x17},   
{0xfd,0x00},
{0x31,0x06},
{0xfd,0x01},
{0x32,0x15},
{0x33,0xef},
{0x34,0x07},
{0xd2,0x01},
{0xfb,0x25},
{0xf2,0x49},
{0x35,0x40},
{0x5d,0x11},
{0xfd,0x01},
{0x36,0x01},
{0xfd,0x01},
{0xfd,0x01},
{0xfd,0x01},
{0xfd,0x01},
{0xfd,0x01},
{0xfd,0x01},
{0xfd,0x01},
{0xfd,0x01},
 

   #endif 

};

static struct v4l2_subdev_info sp0a20_subdev_info[] = {
	{
		.code   = V4L2_MBUS_FMT_YUYV8_2X8,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt    = 1,
		.order    = 0,
	},
};

static const struct i2c_device_id sp0a20_i2c_id[] = {
	{SP0A20_SENSOR_NAME, (kernel_ulong_t)&sp0a20_s_ctrl},
	{ }
};

static int32_t msm_sp0a20_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	CDBG("%s, E. ", __func__);

	return msm_sensor_i2c_probe(client, id, &sp0a20_s_ctrl);
}

static struct i2c_driver sp0a20_i2c_driver = {
	.id_table = sp0a20_i2c_id,
	.probe  = msm_sp0a20_i2c_probe,
	.driver = {
		.name = SP0A20_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client sp0a20_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_BYTE_ADDR,
};

static const struct of_device_id sp0a20_dt_match[] = {
	{.compatible = "qcom,sp0a20", .data = &sp0a20_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, sp0a20_dt_match);

static struct platform_driver sp0a20_platform_driver = {
	.driver = {
		.name = "qcom,sp0a20",
		.owner = THIS_MODULE,
		.of_match_table = sp0a20_dt_match,
	},
};

static void sp0a20_i2c_write_table(struct msm_sensor_ctrl_t *s_ctrl,
		struct msm_camera_i2c_reg_conf *table,
		int num)
{
	int i = 0;
	int rc = 0;
	for (i = 0; i < num; ++i) {
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
			s_ctrl->sensor_i2c_client, table->reg_addr,
			table->reg_data,
			MSM_CAMERA_I2C_BYTE_DATA);
		/*
		if (rc < 0) {
			msleep(100);
			rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write(
				s_ctrl->sensor_i2c_client, table->reg_addr,
				table->reg_data,
				MSM_CAMERA_I2C_BYTE_DATA);
		}
		*/
		table++;
	}

}

static int32_t sp0a20_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	return msm_sensor_power_down(s_ctrl);
}
#ifndef DEBUG_SENSOR_SP
static struct msm_camera_i2c_reg_conf sp0a20_saturation[11][10] = {
	{
		//Saturation level 0
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 - 0x3c},
		{0xd4, SP0A20_P1_0xd4 - 0x3c},
		{0xd5, SP0A20_P1_0xd5 - 0x3c},
		{0xd6, SP0A20_P1_0xd6 - 0x3c},
		{0xd7, SP0A20_P1_0xd7 - 0x3c},
		{0xd8, SP0A20_P1_0xd8 - 0x3c},
		{0xd9, SP0A20_P1_0xd9 - 0x3c},
		{0xda, SP0A20_P1_0xda - 0x3c},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL0*/

	{
			//Saturation level 1
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 - 0x38},
		{0xd4, SP0A20_P1_0xd4 - 0x38},
		{0xd5, SP0A20_P1_0xd5 - 0x38},
		{0xd6, SP0A20_P1_0xd6 - 0x38},
		{0xd7, SP0A20_P1_0xd7 - 0x38},
		{0xd8, SP0A20_P1_0xd8 - 0x38},
		{0xd9, SP0A20_P1_0xd9 - 0x38},
		{0xda, SP0A20_P1_0xda - 0x38},
		{0xfd, 0x00},  
	}, /* SATURATION LEVEL1*/

	{
				//Saturation level 2
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 - 0x30},
		{0xd4, SP0A20_P1_0xd4 - 0x30},
		{0xd5, SP0A20_P1_0xd5 - 0x30},
		{0xd6, SP0A20_P1_0xd6 - 0x30},
		{0xd7, SP0A20_P1_0xd7 - 0x30},
		{0xd8, SP0A20_P1_0xd8 - 0x30}, 
		{0xd9, SP0A20_P1_0xd9 - 0x30},
		{0xda, SP0A20_P1_0xda - 0x30},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL2*/

	{
	//Saturation level 3
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 - 0x20},
		{0xd4, SP0A20_P1_0xd4 - 0x20},
		{0xd5, SP0A20_P1_0xd5 - 0x20},
		{0xd6, SP0A20_P1_0xd6 - 0x20},
		{0xd7, SP0A20_P1_0xd7 - 0x20},
		{0xd8, SP0A20_P1_0xd8 - 0x20},
		{0xd9, SP0A20_P1_0xd9 - 0x20},
		{0xda, SP0A20_P1_0xda - 0x20},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL3*/

	{
			//Saturation level 4
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 - 0x10},
		{0xd4, SP0A20_P1_0xd4 - 0x10},
		{0xd5, SP0A20_P1_0xd5 - 0x10},
		{0xd6, SP0A20_P1_0xd6 - 0x10},
		{0xd7, SP0A20_P1_0xd7 - 0x10},
		{0xd8, SP0A20_P1_0xd8 - 0x10},
		{0xd9, SP0A20_P1_0xd9 - 0x10},
		{0xda, SP0A20_P1_0xda - 0x10},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL4*/

	{
			//Saturation level 5 (default)
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3},
		{0xd4, SP0A20_P1_0xd4},
		{0xd5, SP0A20_P1_0xd5},
		{0xd6, SP0A20_P1_0xd6},
		{0xd7, SP0A20_P1_0xd7},
		{0xd8, SP0A20_P1_0xd8},
		{0xd9, SP0A20_P1_0xd9},
		{0xda, SP0A20_P1_0xda},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL5*/

	{
		//Saturation level 6
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 + 0x10},
		{0xd4, SP0A20_P1_0xd4 + 0x10},
		{0xd5, SP0A20_P1_0xd5 + 0x10},
		{0xd6, SP0A20_P1_0xd6 + 0x10},
		{0xd7, SP0A20_P1_0xd7 + 0x10},
		{0xd8, SP0A20_P1_0xd8 + 0x10},
		{0xd9, SP0A20_P1_0xd9 + 0x10},
		{0xda, SP0A20_P1_0xda + 0x10},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL6*/

	{
		//Saturation level 7
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 + 0x20},
		{0xd4, SP0A20_P1_0xd4 + 0x20},
		{0xd5, SP0A20_P1_0xd5 + 0x20},
		{0xd6, SP0A20_P1_0xd6 + 0x20},
		{0xd7, SP0A20_P1_0xd7 + 0x20},
		{0xd8, SP0A20_P1_0xd8 + 0x20},
		{0xd9, SP0A20_P1_0xd9 + 0x20},
		{0xda, SP0A20_P1_0xda + 0x20},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL7*/

	{
		//Saturation level 8
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 + 0x30},
		{0xd4, SP0A20_P1_0xd4 + 0x30},
		{0xd5, SP0A20_P1_0xd5 + 0x30},
		{0xd6, SP0A20_P1_0xd6 + 0x30},
		{0xd7, SP0A20_P1_0xd7 + 0x30},
		{0xd8, SP0A20_P1_0xd8 + 0x30},
		{0xd9, SP0A20_P1_0xd9 + 0x30},
		{0xda, SP0A20_P1_0xda + 0x30},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL8*/

	{
		//Saturation level 9
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 + 0x40},
		{0xd4, SP0A20_P1_0xd4 + 0x40},
		{0xd5, SP0A20_P1_0xd5 + 0x40},
		{0xd6, SP0A20_P1_0xd6 + 0x40},
		{0xd7, SP0A20_P1_0xd7 + 0x40},
		{0xd8, SP0A20_P1_0xd8 + 0x40},
		{0xd9, SP0A20_P1_0xd9 + 0x40},
		{0xda, SP0A20_P1_0xda + 0x40},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL9*/

	{
			//Saturation level 10
		{0xfd, 0x01},
		{0xd3, SP0A20_P1_0xd3 + 0x50},
		{0xd4, SP0A20_P1_0xd4 + 0x50},
		{0xd5, SP0A20_P1_0xd5 + 0x50},
		{0xd6, SP0A20_P1_0xd6 + 0x50},
		{0xd7, SP0A20_P1_0xd7 + 0x50},
		{0xd8, SP0A20_P1_0xd8 + 0x50},
		{0xd9, SP0A20_P1_0xd9 + 0x50},
		{0xda, SP0A20_P1_0xda + 0x50},
		{0xfd, 0x00},
	}, /* SATURATION LEVEL10*/
};

static struct msm_camera_i2c_reg_conf sp0a20_contrast[11][10] = {
	{
		//standard curve   3.0
		//Contrast -5
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 - 0x28},
		{0x11, SP0A20_P1_0x11 - 0x28},
		{0x12, SP0A20_P1_0x12 - 0x28},
		{0x13, SP0A20_P1_0x13 - 0x28},
		{0x14, SP0A20_P1_0x14 - 0x28},
		{0x15, SP0A20_P1_0x15 - 0x28},
		{0x16, SP0A20_P1_0x16 - 0x28},
		{0x17, SP0A20_P1_0x17 - 0x28},
		{0xfd, 0x00},
	}, /* CONTRAST L0*/
	{
		//standard curve   3.5
		//Contrast -4
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 - 0x20},
		{0x11, SP0A20_P1_0x11 - 0x20},
		{0x12, SP0A20_P1_0x12 - 0x20},
		{0x13, SP0A20_P1_0x13 - 0x20},
		{0x14, SP0A20_P1_0x14 - 0x20},
		{0x15, SP0A20_P1_0x15 - 0x20},
		{0x16, SP0A20_P1_0x16 - 0x20},
		{0x17, SP0A20_P1_0x17 - 0x20},
		{0xfd, 0x00},
	}, /* CONTRAST L1*/
	{
		//standard curve   4.0
		//Contrast -3
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 - 0x18},
		{0x11, SP0A20_P1_0x11 - 0x18},
		{0x12, SP0A20_P1_0x12 - 0x18},
		{0x13, SP0A20_P1_0x13 - 0x18},
		{0x14, SP0A20_P1_0x14 - 0x18},
		{0x15, SP0A20_P1_0x15 - 0x18},
		{0x16, SP0A20_P1_0x16 - 0x18},
		{0x17, SP0A20_P1_0x17 - 0x18},
		{0xfd, 0x00},
	}, /* CONTRAST L2*/
	{
		//standard curve   4.5
		//Contrast -2
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 - 0x10},
		{0x11, SP0A20_P1_0x11 - 0x10},
		{0x12, SP0A20_P1_0x12 - 0x10},
		{0x13, SP0A20_P1_0x13 - 0x10},
		{0x14, SP0A20_P1_0x14 - 0x10},
		{0x15, SP0A20_P1_0x15 - 0x10},
		{0x16, SP0A20_P1_0x16 - 0x10},
		{0x17, SP0A20_P1_0x17 - 0x10},
		{0xfd, 0x00},
	}, /* CONTRAST L3*/
	{
		//standard curve   5.0
		//Contrast -1
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 - 0x08},
		{0x11, SP0A20_P1_0x11 - 0x08},
		{0x12, SP0A20_P1_0x12 - 0x08},
		{0x13, SP0A20_P1_0x13 - 0x08},
		{0x14, SP0A20_P1_0x14 - 0x08},
		{0x15, SP0A20_P1_0x15 - 0x08},
		{0x16, SP0A20_P1_0x16 - 0x08},
		{0x17, SP0A20_P1_0x17 - 0x08},
		{0xfd, 0x00},
	}, /* CONTRAST L4*/
	{
		//default
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10},
		{0x11, SP0A20_P1_0x11},
		{0x12, SP0A20_P1_0x12},
		{0x13, SP0A20_P1_0x13},
		{0x14, SP0A20_P1_0x14 },
		{0x15, SP0A20_P1_0x15},
		{0x16, SP0A20_P1_0x16},
		{0x17, SP0A20_P1_0x17},
		{0xfd, 0x00},
	}, /* CONTRAST L5*/
	{
		//standard curve   6.0
		//Contrast 1
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 + 0x08},
		{0x11, SP0A20_P1_0x11 + 0x08},
		{0x12, SP0A20_P1_0x12 + 0x08},
		{0x13, SP0A20_P1_0x13 + 0x08},
		{0x14, SP0A20_P1_0x14 + 0x08},
		{0x15, SP0A20_P1_0x15 + 0x08},
		{0x16, SP0A20_P1_0x16 + 0x08},
		{0x17, SP0A20_P1_0x17 + 0x08},
		{0xfd, 0x00},
	}, /* CONTRAST L6*/
	{
		//standard curve   6.5
		//Contrast 2
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 + 0x10},
		{0x11, SP0A20_P1_0x11 + 0x10},
		{0x12, SP0A20_P1_0x12 + 0x10},
		{0x13, SP0A20_P1_0x13 + 0x10},
		{0x14, SP0A20_P1_0x14 + 0x10},
		{0x15, SP0A20_P1_0x15 + 0x10},
		{0x16, SP0A20_P1_0x16 + 0x10},
		{0x17, SP0A20_P1_0x17 + 0x10},
		{0xfd, 0x00},
	}, /* CONTRAST L7*/
	{
		//standard curve   7.0
		//Contrast 3
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 + 0x18},
		{0x11, SP0A20_P1_0x11 + 0x18},
		{0x12, SP0A20_P1_0x12 + 0x18},
		{0x13, SP0A20_P1_0x13 + 0x18},
		{0x14, SP0A20_P1_0x14 + 0x18},
		{0x15, SP0A20_P1_0x15 + 0x18},
		{0x16, SP0A20_P1_0x16 + 0x18},
		{0x17, SP0A20_P1_0x17 + 0x18},
		{0xfd, 0x00},
	}, /* CONTRAST L8*/
	{
		//standard curve   7.5
		//Contrast 4
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 + 0x20},
		{0x11, SP0A20_P1_0x11 + 0x20},
		{0x12, SP0A20_P1_0x12 + 0x20},
		{0x13, SP0A20_P1_0x13 + 0x20},
		{0x14, SP0A20_P1_0x14 + 0x20},
		{0x15, SP0A20_P1_0x15 + 0x20},
		{0x16, SP0A20_P1_0x16 + 0x20},
		{0x17, SP0A20_P1_0x17 + 0x20},
		{0xfd, 0x00},
	}, /* CONTRAST L9*/
	{
		//standard curve   8.0
		//Contrast 5
		{0xfd, 0x01},
		{0x10, SP0A20_P1_0x10 + 0x28},
		{0x11, SP0A20_P1_0x11 + 0x28},
		{0x12, SP0A20_P1_0x12 + 0x28},
		{0x13, SP0A20_P1_0x13 + 0x28},
		{0x14, SP0A20_P1_0x14 + 0x28},
		{0x15, SP0A20_P1_0x15 + 0x28},
		{0x16, SP0A20_P1_0x16 + 0x28},
		{0x17, SP0A20_P1_0x17 + 0x28},
		{0xfd, 0x00},
	},/* CONTRAST L10*/
};

static struct msm_camera_i2c_reg_conf sp0a20_sharpness[7][10] = {
	{
		//Sharpness 0
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 - 0x0c},
		{0xec, SP0A20_P2_0xec - 0x0c},
		{0xe9, SP0A20_P2_0xe9 - 0x0c},
		{0xed, SP0A20_P2_0xed - 0x0c},
		{0xea, SP0A20_P2_0xea - 0x0c},
		{0xee, SP0A20_P2_0xee - 0x0c},
		{0xeb, SP0A20_P2_0xeb - 0x0c},
		{0xef, SP0A20_P2_0xef  - 0x0c},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 0*/
	{
		//Sharpness 1
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 - 0x08},
		{0xec, SP0A20_P2_0xec - 0x08},
		{0xe9, SP0A20_P2_0xe9 - 0x08},
		{0xed, SP0A20_P2_0xed - 0x08},
		{0xea, SP0A20_P2_0xea - 0x08},
		{0xee, SP0A20_P2_0xee - 0x08},
		{0xeb, SP0A20_P2_0xeb - 0x08},
		{0xef, SP0A20_P2_0xef  - 0x08},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 1*/
	{
		//Sharpness 2
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 - 0x04},
		{0xec, SP0A20_P2_0xec - 0x04},
		{0xe9, SP0A20_P2_0xe9 - 0x04},
		{0xed, SP0A20_P2_0xed - 0x04},
		{0xea, SP0A20_P2_0xea - 0x04},
		{0xee, SP0A20_P2_0xee - 0x04},
		{0xeb, SP0A20_P2_0xeb - 0x04},
		{0xef, SP0A20_P2_0xef  - 0x04},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 2*/
	{
		         //Sharpness Auto (Default)
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8},
		{0xec, SP0A20_P2_0xec},
		{0xe9, SP0A20_P2_0xe9},
		{0xed, SP0A20_P2_0xed},
		{0xea, SP0A20_P2_0xea},
		{0xee, SP0A20_P2_0xee},
		{0xeb, SP0A20_P2_0xeb},
		{0xef, SP0A20_P2_0xef}, 
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 3*/
	{
		//Sharpness 4
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 + 0x08},
		{0xec, SP0A20_P2_0xec + 0x08},
		{0xe9, SP0A20_P2_0xe9 + 0x08},
		{0xed, SP0A20_P2_0xed + 0x08},
		{0xea, SP0A20_P2_0xea + 0x08},
		{0xee, SP0A20_P2_0xee + 0x08},
		{0xeb, SP0A20_P2_0xeb + 0x08},
		{0xef, SP0A20_P2_0xef + 0x08},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 4*/
	{
		//Sharpness 5
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 + 0x10},
		{0xec, SP0A20_P2_0xec + 0x10},
		{0xe9, SP0A20_P2_0xe9 + 0x10},
		{0xed, SP0A20_P2_0xed + 0x10},
		{0xea, SP0A20_P2_0xea + 0x10},
		{0xee, SP0A20_P2_0xee + 0x10},
		{0xeb, SP0A20_P2_0xeb + 0x10},
		{0xef, SP0A20_P2_0xef + 0x10},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 5*/
	{
		//Sharpness 6
		{0xfd, 0x02},
		{0xe8, SP0A20_P2_0xe8 + 0x20},
		{0xec, SP0A20_P2_0xec + 0x20},
		{0xe9, SP0A20_P2_0xe9 + 0x20},
		{0xed, SP0A20_P2_0xed + 0x20},
		{0xea, SP0A20_P2_0xea + 0x20},
		{0xee, SP0A20_P2_0xee + 0x20},
		{0xeb, SP0A20_P2_0xeb + 0x20},
		{0xef, SP0A20_P2_0xef + 0x20},
		{0xfd, 0x00},
	}, /* SHARPNESS LEVEL 6*/
};

static struct msm_camera_i2c_reg_conf sp0a20_exposure[5][3] = {
	{
		                	//@@ +2EV
		{0xfd, 0x01},
		{0xdb, 0xe0},
		{0xfd, 0x00},
	}, /*EXPOSURECOMPENSATIONN2*/
	{
				       	//@@ +1EV
		{0xfd, 0x01},
		{0xdb, 0xf0},  
		{0xfd, 0x00},
	}, /*EXPOSURECOMPENSATIONN1*/
	{
			  	//@@ default
		{0xfd, 0x01},
		{0xdb, 0x00},  
		{0xfd, 0x00},
	}, /*EXPOSURECOMPENSATIOND*/
	{
			//@@ -1EV
		{0xfd, 0x01},
		{0xdb, 0x10},	 
		{0xfd, 0x00},
	}, /*EXPOSURECOMPENSATIONP1*/
	{
				//@@ -2EV
		{0xfd, 0x01},
		{0xdb, 0x20}, 
		{0xfd, 0x00},
	}, /*EXPOSURECOMPENSATIONP2*/
};

static struct msm_camera_i2c_reg_conf sp0a20_iso[7][3] = {
	/* auto */
	{
		{0xfd, 0x00},		
	},
	/* auto hjt */
	{
		{0xfd, 0x00},	
	},
	/* iso 100 */
	{
		{0xfd, 0x00},
		{0x24, 0x20},
		{0xfd, 0x00},
	},
	/* iso 200 */
	{
		{0xfd, 0x00},
		{0x24, 0x30},
		{0xfd, 0x00},
	},
	/* iso 400 */
	{
		{0xfd, 0x00},
		{0x24, 0x40},
		{0xfd, 0x00},
	},
	/* iso 800 */
	{
		{0xfd, 0x00},
		{0x24, 0x50},
		{0xfd, 0x00},
	},
	/* iso 1600 */
	{
		{0xfd, 0x00},
		{0x24, 0x60},
		{0xfd, 0x00},
	},
};




static struct msm_camera_i2c_reg_conf sp0a20_antibanding[][25] = {
    {
        // OFF
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},  
        {-1,-1},     
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
        {-1,-1},           
    },
    /*ANTIBANDING 50HZ*/
	{                                         
    //50Hz_24MHz_19~10fps           
    {0xfd,0x00},                        
    {0xfd,0x00},                     
    },
    /*ANTIBANDING 60HZ*/
    {
    //60Hz_24MHz_18.96~10fps
    {0xfd,0x00},           
    {0xfd,0x00},           
    },    
    /*ANTIBANDING AUTO*/
	//50Hz_24MHz_19~10fps 
    {
    {0xfd,0x00},                                   
    {0xfd,0x00},                     
    }, 
};

static struct msm_camera_i2c_reg_conf sp0a20_wb_oem[][6] = {
	//{{0xfd, 0x00},{0xfd, 0x00},{0xfd, 0x00},{0xfd, 0x00},{0xfd, 0x00},}, /*WHITEBALNACE OFF*/
	{
		/* Auto: */
		{0xfd, 0x02},
		{0x26, 0xa0},
		{0x27, 0xa8},
		{0xfd, 0x01},
		{0x32, 0x15},
		{0xfd, 0x00},
}, /*WHITEBALNACE AUTO*/
	{
		/* Auto: */
		{0xfd, 0x02},
		{0x26, 0xa0},
		{0x27, 0xa8},
		{0xfd, 0x01},
		{0x32, 0x15},
		{0xfd, 0x00},
}, /*WHITEBALNACE CUSTOM*/
	{
		/* Office: *//*INCANDISCENT*///°×³ãµÆ
		{0xfd, 0x01},
		{0x32, 0x05},
		{0xfd, 0x02},
		{0x26, 0x7b},
		{0x27, 0xd3},
		{0xfd, 0x00},
}, /*INCANDISCENT*/
	{
		/* Home: */
		{0xfd, 0x01},
		{0x32, 0x05},
		{0xfd, 0x02},
		{0x26, 0xb4},
		{0x27, 0xc4},
		{0xfd, 0x00},	
}, /*FLOURESECT*/
	{
		/* Home: */
		{0xfd, 0x01},
		{0x32, 0x05},
		{0xfd, 0x02},
		{0x26, 0xb4},
		{0x27, 0xc4},
		{0xfd, 0x00},	
}, /*FLOURESECT*/
	{
		/*DAYLIGHT*/
		{0xfd, 0x01},
		{0x32, 0x05},
		{0xfd, 0x02},
		{0x26, 0xc1},
		{0x27, 0x88},
		{0xfd, 0x00},
}, /*DAYLIGHT*/
	{
		/*CLOUDY*/
		{0xfd, 0x01},
		{0x32, 0x05},
		{0xfd, 0x02},
		{0x26, 0xe2},
		{0x27, 0x82},
		{0xfd, 0x00},
}, /*CLOUDY*/
};
#if 0
static struct msm_camera_i2c_reg_conf sp0a20_special_effect[8][6] = {
	{{0xfd, 0x01}, {0x66, 0x00}, {0x67, 0x80}, {0x68, 0x80},  {0xfd, 0x02}, {0x14, 0x00},}, /*emboss_en */
	{{0xfd, 0x01}, {0x66, 0x02}, {0x67, 0x80}, {0x68, 0x80},  {0xfd, 0x02}, {0x14, 0x00},}, /*enchase_en */
	{{0xfd, 0x01}, {0x66, 0x04}, {0x67, 0x80}, {0x68, 0x80},  {0xfd, 0x02}, {0x14, 0x00},}, /*color_neg_en */ 
	{{0xfd, 0x01}, {0x66, 0x08}, {0x67, 0x80}, {0x68, 0x80},  {0xfd, 0x02}, {0x14, 0x00},}, /*gray_neg_en*/
	{{0xfd, 0x01}, {0x66, 0x10}, {0x67, 0x80}, {0x68, 0x80},  {0xfd, 0x02}, {0x14, 0x00},}, /*sepia_en*/
	{{0xfd, 0x01}, {0x66, 0x20}, {0x67, 0x80}, {0x68, 0x80},  {0xfd, 0x02}, {0x14, 0x00},}, /*mono_en*/
	{{0xfd, 0x01}, {0x66, 0x40}, {0x67, 0x80}, {0x68, 0x80},  {0xfd, 0x02}, {0x14, 0x00},}, /*sketch_en */
	{{0xfd, 0x01}, {0x66, 0x80}, {0x67, 0x80}, {0x68, 0x80},  {0xfd, 0x02}, {0x14, 0x00},}, /*solarize_en  */

};
#endif
//begin effect
static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_normal[] = {
	/* normal: */
	{0xfd, 0x01}, 
	{0x66, 0x00}, 
	{0x67, 0x80}, 
	{0x68, 0x80}, 
	{0xfd, 0x02}, 
	{0x14, 0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_enchase[] = {
	/* enchase: */
	{0xfd, 0x01},
	{0x66, 0x10}, 
	{0x67, 0x80}, 
	{0x68, 0x80},  
	{0xfd, 0x02}, 
	{0x14, 0x00},
	
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_emboss[] = {
	/* Emboss: */
	{0xfd, 0x01}, 
	{0x66, 0x02}, 
	{0x67, 0x80}, 
	{0x68, 0x80}, 
	{0xfd, 0x02}, 
	{0x14, 0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_sketch[] = {
	/* sketch: */
	{0xfd, 0x01}, 
	{0x66, 0x40}, 
	{0x67, 0x80}, 
	{0x68, 0x80}, 
	{0xfd, 0x02}, 
	{0x14, 0x00},
};

static struct msm_camera_i2c_reg_conf sp0a20_reg_effect_solarize[] = {
	{0xfd, 0x01},
	{0x66, 0x04},
	{0x67, 0x80}, 
	{0x68, 0x80},  
	{0xfd, 0x02}, 
	{0x14, 0x00},
};
// end effect

static void sp0a20_set_stauration(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
    if (value >=  ARRAY_SIZE(sp0a20_saturation))
    {
        pr_err("%s %d too large for saturation", __func__, value);
        return;
    }      
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_saturation[value][0],
		ARRAY_SIZE(sp0a20_saturation[value]));
}

static void sp0a20_set_contrast(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
    if (value >=  ARRAY_SIZE(sp0a20_contrast))
    {
        pr_err("%s %d too large for contrast", __func__, value);
        return;
    }        
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_contrast[value][0],
		ARRAY_SIZE(sp0a20_contrast[value]));
}

static void sp0a20_set_sharpness(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	int val = value / 6;
	pr_debug("%s %d", __func__, value);
    if (val >=  ARRAY_SIZE(sp0a20_sharpness))
    {
        pr_err("%s %d too large for sharpness", __func__, val);
        return;
    }    
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_sharpness[val][0],
		ARRAY_SIZE(sp0a20_sharpness[val]));
}

static void sp0a20_set_iso(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_iso[value][0],
		ARRAY_SIZE(sp0a20_iso[value]));
}


static void sp0a20_set_exposure_compensation(struct msm_sensor_ctrl_t *s_ctrl,
	int value)
{
	int val = (value + 12) / 6;
	pr_debug("%s %d", __func__, val);
    if (val >=  ARRAY_SIZE(sp0a20_exposure))
    {
        pr_err("%s %d too large for exposure", __func__, val);
        return;
    }
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_exposure[val][0],
		ARRAY_SIZE(sp0a20_exposure[val]));
}
#if 0
static void sp0a20_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_err("%s %d", __func__, value);
    if (value >=  ARRAY_SIZE(sp0a20_special_effect))
    {
        pr_err("%s %d too large for effect", __func__, value);
        return;
    }        
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_special_effect[value][0],
		ARRAY_SIZE(sp0a20_special_effect[value]));
}
#endif
static void sp0a20_set_effect(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	CDBG("%s %d\n", __func__, value);
	switch (value) {
	case MSM_CAMERA_EFFECT_MODE_OFF: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_normal[0],
			ARRAY_SIZE(sp0a20_reg_effect_normal));
		break;
	}
	case 10: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_enchase[0],
			ARRAY_SIZE(sp0a20_reg_effect_enchase));
		break;
	}
	case 9: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_emboss[0],
			ARRAY_SIZE(sp0a20_reg_effect_emboss));
		break;
	}
	case 11: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_sketch[0],
			ARRAY_SIZE(sp0a20_reg_effect_sketch));
		break;
	}
	case MSM_CAMERA_EFFECT_MODE_SOLARIZE: {
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_solarize[0],
			ARRAY_SIZE(sp0a20_reg_effect_solarize));
		break;
	}
	default:
		sp0a20_i2c_write_table(s_ctrl, &sp0a20_reg_effect_normal[0],
			ARRAY_SIZE(sp0a20_reg_effect_normal));
	}
}


static void sp0a20_set_antibanding(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	pr_debug("%s %d", __func__, value);
    if (value >=  ARRAY_SIZE(sp0a20_antibanding))
    {
        pr_err("%s %d too large for antibanding", __func__, value);
        return;
    }    
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_antibanding[value][0],
		ARRAY_SIZE(sp0a20_antibanding[value]));
}

static void sp0a20_set_white_balance(struct msm_sensor_ctrl_t *s_ctrl, int value)
{
	//unsigned short temp = 0;
	pr_debug("%s %d", __func__, value);
    if (value >=  ARRAY_SIZE(sp0a20_wb_oem))
    {
        pr_err("%s %d too large for whitebalance", __func__, value);
        return;
    }
/*

    s_ctrl->sensor_i2c_client->i2c_func_tbl->
              i2c_write(s_ctrl->sensor_i2c_client, 0xfd, 0x00, MSM_CAMERA_I2C_BYTE_DATA);

    s_ctrl->sensor_i2c_client->i2c_func_tbl->
              i2c_read(s_ctrl->sensor_i2c_client, 0x32, &temp, MSM_CAMERA_I2C_BYTE_DATA);

    if (value == 0 || value == 1)
    {
        temp |= 0x08;
    }
    else
    {
        temp &= (~(0x08));
    }

    s_ctrl->sensor_i2c_client->i2c_func_tbl->
              i2c_write(s_ctrl->sensor_i2c_client, 0x32, temp, MSM_CAMERA_I2C_BYTE_DATA);
*/
    
	sp0a20_i2c_write_table(s_ctrl, &sp0a20_wb_oem[value][0],
		ARRAY_SIZE(sp0a20_wb_oem[value]));
}
#endif

static int32_t sp0a20_platform_probe(struct platform_device *pdev)
{
	int32_t rc;
	const struct of_device_id *match;
	CDBG("%s, E.", __func__);
	match = of_match_device(sp0a20_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init sp0a20_init_module(void)
{
	int32_t rc;
	pr_info("%s:%d\n", __func__, __LINE__);
	rc = platform_driver_probe(&sp0a20_platform_driver,
		sp0a20_platform_probe);
	if (!rc)
		return rc;
    pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&sp0a20_i2c_driver);
}

static void __exit sp0a20_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (sp0a20_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&sp0a20_s_ctrl);
		platform_driver_unregister(&sp0a20_platform_driver);
	} else
		i2c_del_driver(&sp0a20_i2c_driver);
	return;
}

int32_t sp0a20_match_id(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;
	uint16_t chipid = 0;
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			s_ctrl->sensordata->slave_info->sensor_id_reg_addr,
			&chipid, MSM_CAMERA_I2C_BYTE_DATA);
	if (rc < 0) {
		pr_err("%s: %s: fuyipeng read id failed\n", __func__,
			s_ctrl->sensordata->sensor_name);
		return rc;
	}

	printk("%s: read id: %x expected id %x:\n", __func__, chipid,
		s_ctrl->sensordata->slave_info->sensor_id);
	if (chipid != s_ctrl->sensordata->slave_info->sensor_id) {
		pr_err("msm_sensor_match_id chip id doesnot match\n");
		return -ENODEV;
	}
	return rc;
}


int32_t sp0a20_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	int32_t i = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_SENSOR_INFO:
		memcpy(cdata->cfg.sensor_info.sensor_name,
			s_ctrl->sensordata->sensor_name,
			sizeof(cdata->cfg.sensor_info.sensor_name));
		cdata->cfg.sensor_info.session_id =
			s_ctrl->sensordata->sensor_info->session_id;
		for (i = 0; i < SUB_MODULE_MAX; i++)
			cdata->cfg.sensor_info.subdev_id[i] =
				s_ctrl->sensordata->sensor_info->subdev_id[i];
		CDBG("%s:%d sensor name %s\n", __func__, __LINE__,
			cdata->cfg.sensor_info.sensor_name);
		CDBG("%s:%d session id %d\n", __func__, __LINE__,
			cdata->cfg.sensor_info.session_id);
		for (i = 0; i < SUB_MODULE_MAX; i++)
			CDBG("%s:%d subdev_id[%d] %d\n", __func__, __LINE__, i,
				cdata->cfg.sensor_info.subdev_id[i]);
		break;
	case CFG_SET_INIT_SETTING:
		CDBG("init setting");
		#ifdef DEBUG_SENSOR_SP
		if(1 == SP_Initialize_from_T_Flash())
		{
			sp0a20_i2c_write_table(s_ctrl,
				&SP_Init_Reg[0],
				reg_num);
		} 
		else
			{

				sp0a20_i2c_write_table(s_ctrl,
				&sp0a20_recommend_settings[0],
				ARRAY_SIZE(sp0a20_recommend_settings));
			}
		#else
				sp0a20_i2c_write_table(s_ctrl,
				&sp0a20_recommend_settings[0],
				ARRAY_SIZE(sp0a20_recommend_settings));
		#endif
		CDBG("init setting X");
        msleep(200);
		break;
	case CFG_SET_RESOLUTION: {
//		int val = 0;
	//	if (copy_from_user(&val,
	//		(void *)cdata->cfg.setting, sizeof(int))) {
	//		pr_err("%s:%d failed\n", __func__, __LINE__);
	//		rc = -EFAULT;
	//		break;
	//	}
	//	if (val == 0)
	//		sp0a20_i2c_write_table(s_ctrl, &sp0a20_uxga_settings[0],
	//			ARRAY_SIZE(sp0a20_uxga_settings));
	//	else if (val == 1)
	//		sp0a20_i2c_write_table(s_ctrl, &sp0a20_svga_settings[0],
	//			ARRAY_SIZE(sp0a20_svga_settings));
		break;
	}
	case CFG_SET_STOP_STREAM:
		#ifdef DEBUG_SENSOR_SP
			if(1 == SP_Stop_Initialize_from_T_Flash())
			{
				sp0a20_i2c_write_table(s_ctrl, &SP_Stop_Reg[0],
					reg_stop_num);

			} 
		else
			{
				sp0a20_i2c_write_table(s_ctrl,
				&sp0a20_stop_settings[0],
				ARRAY_SIZE(sp0a20_stop_settings));
			}
		#else
			sp0a20_i2c_write_table(s_ctrl,
				&sp0a20_stop_settings[0],
				ARRAY_SIZE(sp0a20_stop_settings));
		#endif
        msleep(20);
		break;
	case CFG_SET_START_STREAM:
		#ifdef DEBUG_SENSOR_SP
			if(1 == SP_Start_Initialize_from_T_Flash())
			{
				sp0a20_i2c_write_table(s_ctrl, &SP_Start_Reg[0],
					reg_start_num);

			}
		else
			{
		sp0a20_i2c_write_table(s_ctrl,
			&sp0a20_start_settings[0],
			ARRAY_SIZE(sp0a20_start_settings));
			}
		#else
			sp0a20_i2c_write_table(s_ctrl,
				&sp0a20_start_settings[0],
				ARRAY_SIZE(sp0a20_start_settings));
		#endif
        msleep(20);
		break;
	case CFG_GET_SENSOR_INIT_PARAMS:
		cdata->cfg.sensor_init_params.modes_supported =
			s_ctrl->sensordata->sensor_info->modes_supported;
		cdata->cfg.sensor_init_params.position =
			s_ctrl->sensordata->sensor_info->position;
		cdata->cfg.sensor_init_params.sensor_mount_angle =
			s_ctrl->sensordata->sensor_info->sensor_mount_angle;
		CDBG("%s:%d init params mode %d pos %d mount %d\n", __func__,
			__LINE__,
			cdata->cfg.sensor_init_params.modes_supported,
			cdata->cfg.sensor_init_params.position,
			cdata->cfg.sensor_init_params.sensor_mount_angle);
		break;
	case CFG_SET_SLAVE_INFO: {
		struct msm_camera_sensor_slave_info sensor_slave_info;
		struct msm_sensor_power_setting_array *power_setting_array;
		int slave_index = 0;
		if (copy_from_user(&sensor_slave_info,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_sensor_slave_info))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		/* Update sensor slave address */
		if (sensor_slave_info.slave_addr) {
			s_ctrl->sensor_i2c_client->cci_client->sid =
				sensor_slave_info.slave_addr >> 1;
		}

		/* Update sensor address type */
		s_ctrl->sensor_i2c_client->addr_type =
			sensor_slave_info.addr_type;

		/* Update power up / down sequence */
		s_ctrl->power_setting_array =
			sensor_slave_info.power_setting_array;
		power_setting_array = &s_ctrl->power_setting_array;
		power_setting_array->power_setting = kzalloc(
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting), GFP_KERNEL);
		if (!power_setting_array->power_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(power_setting_array->power_setting,
			(void *)
			sensor_slave_info.power_setting_array.power_setting,
			power_setting_array->size *
			sizeof(struct msm_sensor_power_setting))) {
			kfree(power_setting_array->power_setting);
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.slave_addr);
		CDBG("%s sensor addr type %d\n", __func__,
			sensor_slave_info.addr_type);
		CDBG("%s sensor reg %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id_reg_addr);
		CDBG("%s sensor id %x\n", __func__,
			sensor_slave_info.sensor_id_info.sensor_id);
		for (slave_index = 0; slave_index <
			power_setting_array->size; slave_index++) {
			CDBG("%s i %d power setting %d %d %ld %d\n", __func__,
				slave_index,
				power_setting_array->power_setting[slave_index].
				seq_type,
				power_setting_array->power_setting[slave_index].
				seq_val,
				power_setting_array->power_setting[slave_index].
				config_val,
				power_setting_array->power_setting[slave_index].
				delay);
		}
		kfree(power_setting_array->power_setting);
		break;
	}
	case CFG_WRITE_I2C_ARRAY: {
		struct msm_camera_i2c_reg_setting conf_array;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
		kfree(reg_setting);
		break;
	}
	case CFG_WRITE_I2C_SEQ_ARRAY: {
		struct msm_camera_i2c_seq_reg_setting conf_array;
		struct msm_camera_i2c_seq_reg_array *reg_setting = NULL;

		if (copy_from_user(&conf_array,
			(void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_seq_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = kzalloc(conf_array.size *
			(sizeof(struct msm_camera_i2c_seq_reg_array)),
			GFP_KERNEL);
		if (!reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
			conf_array.size *
			sizeof(struct msm_camera_i2c_seq_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(reg_setting);
			rc = -EFAULT;
			break;
		}

		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_array);
		kfree(reg_setting);
		break;
	}

	case CFG_POWER_UP:
		if (s_ctrl->func_tbl->sensor_power_up)
			rc = s_ctrl->func_tbl->sensor_power_up(s_ctrl);
		else
			rc = -EFAULT;
		#if KK_CAMERA_CTS
		front_sensor_is_poweruped = 1;
		#endif
		break;

	case CFG_POWER_DOWN:
		if (s_ctrl->func_tbl->sensor_power_down)
			rc = s_ctrl->func_tbl->sensor_power_down(s_ctrl);
		else
			rc = -EFAULT;
		#if KK_CAMERA_CTS
		front_sensor_is_poweruped = 0;
		#endif
		break;

	case CFG_SET_STOP_STREAM_SETTING: {
		struct msm_camera_i2c_reg_setting *stop_setting =
			&s_ctrl->stop_setting;
		struct msm_camera_i2c_reg_array *reg_setting = NULL;
		if (copy_from_user(stop_setting, (void *)cdata->cfg.setting,
			sizeof(struct msm_camera_i2c_reg_setting))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}

		reg_setting = stop_setting->reg_setting;
		stop_setting->reg_setting = kzalloc(stop_setting->size *
			(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
		if (!stop_setting->reg_setting) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -ENOMEM;
			break;
		}
		if (copy_from_user(stop_setting->reg_setting,
			(void *)reg_setting, stop_setting->size *
			sizeof(struct msm_camera_i2c_reg_array))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			kfree(stop_setting->reg_setting);
			stop_setting->reg_setting = NULL;
			stop_setting->size = 0;
			rc = -EFAULT;
			break;
		}
		break;
	}
#ifndef DEBUG_SENSOR_SP
	case CFG_SET_SATURATION: {
		int32_t sat_lev;
		if (copy_from_user(&sat_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_err("%s: Saturation Value is %d", __func__, sat_lev);
		sp0a20_set_stauration(s_ctrl, sat_lev);
		break;
	}
	case CFG_SET_CONTRAST: {
		int32_t con_lev;
		if (copy_from_user(&con_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_err("%s: Contrast Value is %d", __func__, con_lev);
		sp0a20_set_contrast(s_ctrl, con_lev);
		break;
	}
	case CFG_SET_SHARPNESS: {
		int32_t shp_lev;
		if (copy_from_user(&shp_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_err("%s: Sharpness Value is %d", __func__, shp_lev);
		sp0a20_set_sharpness(s_ctrl, shp_lev);
		break;
	}
	case CFG_SET_ISO: {
		int32_t iso_lev;
		if (copy_from_user(&iso_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_err("%s: ISO Value is %d", __func__, iso_lev);
		sp0a20_set_iso(s_ctrl, iso_lev);
		break;
	}
	case CFG_SET_EXPOSURE_COMPENSATION: {
		int32_t ec_lev;
		if (copy_from_user(&ec_lev, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_err("%s: Exposure compensation Value is %d",
			__func__, ec_lev);
		sp0a20_set_exposure_compensation(s_ctrl, ec_lev);
		break;
	}
	case CFG_SET_EFFECT: {
		int32_t effect_mode;
		if (copy_from_user(&effect_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_err("%s: Effect mode is %d", __func__, effect_mode);
		sp0a20_set_effect(s_ctrl, effect_mode);
		break;
	}
	case CFG_SET_ANTIBANDING: {
		int32_t antibanding_mode;
		if (copy_from_user(&antibanding_mode,
			(void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_err("%s: anti-banding mode is %d", __func__,
			antibanding_mode);
		sp0a20_set_antibanding(s_ctrl, antibanding_mode);
		break;
	}
	case CFG_SET_BESTSHOT_MODE: {
        /*
		int32_t bs_mode;
		if (copy_from_user(&bs_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_debug("%s: best shot mode is %d", __func__, bs_mode);
		hi256_set_scene_mode(s_ctrl, bs_mode);*/
		break;
	}
	case CFG_SET_WHITE_BALANCE: {
		int32_t wb_mode;
		if (copy_from_user(&wb_mode, (void *)cdata->cfg.setting,
			sizeof(int32_t))) {
			pr_err("%s:%d failed\n", __func__, __LINE__);
			rc = -EFAULT;
			break;
		}
		pr_err("%s: white balance is %d-", __func__, wb_mode);
		sp0a20_set_white_balance(s_ctrl, wb_mode);
		break;
	}
#endif
	default:
		rc = -EFAULT;
		break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);
	return rc;
}

static struct msm_sensor_fn_t sp0a20_sensor_fn_t = {
	.sensor_config = sp0a20_sensor_config,
	.sensor_power_up = msm_sensor_power_up,
	.sensor_power_down = sp0a20_sensor_power_down,
	.sensor_match_id = sp0a20_match_id,
	
};

static struct msm_sensor_ctrl_t sp0a20_s_ctrl = {
	.sensor_i2c_client = &sp0a20_sensor_i2c_client,
	.power_setting_array.power_setting = sp0a20_power_setting,
	.power_setting_array.size = ARRAY_SIZE(sp0a20_power_setting),
	.msm_sensor_mutex = &sp0a20_mut,
	.sensor_v4l2_subdev_info = sp0a20_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(sp0a20_subdev_info),
	.func_tbl = &sp0a20_sensor_fn_t,
};

module_init(sp0a20_init_module);
module_exit(sp0a20_exit_module);
MODULE_DESCRIPTION("sp0a20");
MODULE_LICENSE("GPL v2");
