/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifdef CONFIG_COMPAT

#include <linux/fs.h>
#include <linux/compat.h>

#endif
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_camera_typedef.h"
/*task672921 modify by zhanyong.yin at 20150928 begin*/
#include <mt_gpio.h>
#include <gpio_const.h>
/*task672921 modify by zhanyong.yin at 20150928 end*/
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>


#define USE_NEW_DRIVER //task672921 modify by zhanyong.yin at 20150928
/*
// flash current vs index
    0       1      2       3    4       5      6       7    8       9     10
93.74  140.63  187.5  281.25  375  468.75  562.5  656.25  750  843.75  937.5
     11    12       13      14       15    16
1031.25  1125  1218.75  1312.5  1406.25  1500mA
*/
/******************************************************************************
 * Debug configuration
******************************************************************************/
/* availible parameter */
/* ANDROID_LOG_ASSERT */
/* ANDROID_LOG_ERROR */
/* ANDROID_LOG_WARNING */
/* ANDROID_LOG_INFO */
/* ANDROID_LOG_DEBUG */
/* ANDROID_LOG_VERBOSE */

#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    pr_debug(TAG_NAME "%s: " fmt, __func__ , ##arg)


#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG(a, ...)
#endif

/******************************************************************************
 * local variables
******************************************************************************/

static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */

/*task672921 modify by zhanyong.yin at 20150928 begin*/
static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;


static int g_timeOutTimeMs=0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,37))
static DEFINE_MUTEX(g_strobeSem);
#else
static DECLARE_MUTEX(g_strobeSem);
#endif


#define STROBE_DEVICE_ID 0x63


static struct work_struct workTimeOut;
/*task672921 modify by zhanyong.yin at 20150928 begin*/
#define GPIO_LED_EN  		(GPIO80 | 0x80000000)
#define GPIO_LED_STROBE  	(GPIO9 | 0x80000000)
#define GPIO_LED_TORCH  		(GPIO11 | 0x80000000)
/*task672921 modify by zhanyong.yin at 20150928 end*/

#define LM3643_REG_ENABLE      0x01
#define LM3643_REG_LED1_FLASH  0x03
#define LM3643_REG_LED2_FLASH  0x04
#define LM3643_REG_LED1_TORCH  0x05
#define LM3643_REG_LED2_TORCH  0x06
#define LM3643_REG_TIMING	   0x08		


/*****************************************************************************
Functions
*****************************************************************************/
/*task672921 modify by zhanyong.yin at 20150928 begin*/
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
static void work_timeOutFunc(struct work_struct *data);

static struct i2c_client *LM3643_i2c_client = NULL;

struct LM3643_platform_data {
	u8 torch_pin_enable;    // 1:  TX1/TORCH pin isa hardware TORCH enable
	u8 pam_sync_pin_enable; // 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input
	u8 thermal_comp_mode_enable;// 1: LEDI/NTC pin in Thermal Comparator Mode
	u8 strobe_pin_disable;  // 1 : STROBE Input disabled
	u8 vout_mode_enable;  // 1 : Voltage Out Mode enable
};

struct LM3643_chip_data {
	struct i2c_client *client;

	//struct led_classdev cdev_flash;
	//struct led_classdev cdev_torch;
	//struct led_classdev cdev_indicator;

	struct LM3643_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};
/*task672921 modify by zhanyong.yin at 20150928 end*/
/* i2c access*/
/*
static int LM3643_read_reg(struct i2c_client *client, u8 reg,u8 *val)
{
	int ret;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret = i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);

	if (ret < 0) {
		PK_ERR("failed reading at 0x%02x error %d\n",reg, ret);
		return ret;
	}
	*val = ret&0xff;

	return 0;
}*/
/*task672921 modify by zhanyong.yin at 20150928 begin*/
static int LM3643_write_reg(struct i2c_client *client, u8 reg, u8 val)
{
	int ret=0;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	ret =  i2c_smbus_write_byte_data(client, reg, val);
	mutex_unlock(&chip->lock);

	if (ret < 0)
		printk("failed writting at 0x%02x\n", reg);
	return ret;
}

static int LM3643_read_reg(struct i2c_client *client, u8 reg)
{
	int val=0;
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

	mutex_lock(&chip->lock);
	val =  i2c_smbus_read_byte_data(client, reg);
	mutex_unlock(&chip->lock);


	return val;
}




static int LM3643_chip_init(struct LM3643_chip_data *chip)
{


	return 0;
}
static int LM3643_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct LM3643_chip_data *chip;
	struct LM3643_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("LM3643_probe start--->.\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		printk(KERN_ERR  "LM3643 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3643_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&chip->lock);
	i2c_set_clientdata(client, chip);

	if(pdata == NULL){ //values are set to Zero.
		printk("LM3643 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3643_platform_data),GFP_KERNEL);
		chip->pdata  = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata  = pdata;
	if(LM3643_chip_init(chip)<0)
		goto err_chip_init;

	LM3643_i2c_client = client;
	PK_DBG("LM3643 Initializing is done \n");

	return 0;

err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	printk("LM3643 probe is failed \n");
	return -ENODEV;
}

static int LM3643_remove(struct i2c_client *client)
{
	struct LM3643_chip_data *chip = i2c_get_clientdata(client);

    if(chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}


#define LM3643_NAME "leds-LM3643"
#define I2C_STROBE_MAIN_SLAVE_7_BIT_ADDR  0x63
static const struct i2c_device_id LM3643_id[] = {
	{LM3643_NAME, 0},
	{}
};
#ifdef CONFIG_OF
static const struct of_device_id LM3643_of_match[] = {
{.compatible = "mediatek,STROBE_MAIN"},
	{},
};
#endif
static struct i2c_driver LM3643_i2c_driver = {
	.driver = {
		.name  = LM3643_NAME,
		#ifdef CONFIG_OF
		.of_match_table = LM3643_of_match,
		#endif
	},
	.probe	= LM3643_probe,
	.remove   = LM3643_remove,
	.id_table = LM3643_id,
};

struct LM3643_platform_data LM3643_pdata = {0, 0, 0, 0, 0};


static int __init LM3643_init(void)
{
	printk("LM3643_init\n");
	//i2c_register_board_info(2, &i2c_LM3643, 1);
	//i2c_register_board_info(1, &i2c_LM3643, 1);


	return i2c_add_driver(&LM3643_i2c_driver);
}

static void __exit LM3643_exit(void)
{
	i2c_del_driver(&LM3643_i2c_driver);
}


module_init(LM3643_init);
module_exit(LM3643_exit);

MODULE_DESCRIPTION("Flash driver for LM3643");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");
int readReg(int reg)
{
#if 1
    int val;
    val = LM3643_read_reg(LM3643_i2c_client, reg);
    return (int)val;
#else	

    char buf[2];
    char bufR[2];
    buf[0]=reg;
    iReadRegI2C(buf , 1, bufR,1, STROBE_DEVICE_ID);
    PK_DBG("qq reg=%x val=%x qq\n", buf[0],bufR[0]);
    return (int)bufR[0];
#endif	
}

int writeReg(int reg, int val)
{
#if 1
    int err;
    //struct LM3643_chip_data *chip = i2c_get_clientdata(client);
    PK_DBG("writeReg  ---1--- reg=0x%x\n",reg);
    	    err = LM3643_write_reg(LM3643_i2c_client, reg, val);
	    return (int)val;

#else	

    char buf[2];
    buf[0]=reg;
    buf[1]=data;
	
    iWriteRegI2C(buf, 2, STROBE_DEVICE_ID);

   return 0;
#endif   
}

#define e_DutyNum 17
#define TORCHDUTYNUM 4
static int isMovieMode[e_DutyNum] = {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0};

static int torchLEDReg[e_DutyNum] = {35,71,106,127,0,0,0,0,0,0,0,0,0,0,0,0,0};
//50,100,150,179ma
static int flashLEDReg[e_DutyNum] = {3,8,12,14,16,20,25,29,33,37,42,46,50,55,59,63,67};
//200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500ma



int m_duty1=0;
int m_duty2=0;
int LED1Closeflag = 0;
int LED2Closeflag = 0;

int FlashIc_Enable(void)
{
    if(mt_set_gpio_out(GPIO_LED_EN,GPIO_OUT_ONE)){PK_DBG(" set gpio failed!! \n");}
	printk("FlashIc_Enable!\n");
	return 0;
}

int FlashIc_Disable(void)
{
    //if(mt_set_gpio_out(GPIO_LED_EN,GPIO_OUT_ZERO)){PK_DBG(" set gpio failed!! \n");}
	printk("FlashIc_Disable!\n");
	return 0;
}


int flashEnable_LM3643_1(void)
{
	//int temp;
	return 0;
}
int flashDisable_LM3643_1(void)
{
	//int temp;
    return 0;
}

int setDuty_LM3643_1(int duty)
{

	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty1=duty;
	
	return 0;
}



int flashEnable_LM3643_2(void)
{
	int temp;

	PK_DBG("flashEnable_LM3643_2\n");
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);

	temp = readReg(LM3643_REG_ENABLE);

	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		writeReg(LM3643_REG_ENABLE, temp & 0xF0);//close		
	}
	else if(LED1Closeflag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFA);//torch mode
		else
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFE);//flash mode
	}
	else if(LED2Closeflag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xF9);//torch mode
		else
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFD);//flash mode		
	}
	else
	{
		if((isMovieMode[m_duty1] == 1) & (isMovieMode[m_duty2] == 1))
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFB);//torch mode
		else
			writeReg(LM3643_REG_ENABLE, (temp&0xF0) | 0xFF);//flash mode
	}
	return 0;

}
int flashDisable_LM3643_2(void)
{
	flashEnable_LM3643_2();
	return 0;
}


int setDuty_LM3643_2(int duty)
{
	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty2=duty;

	PK_DBG("setDuty_LM3643_2:m_duty = %d, m_duty2 = %d!\n", m_duty1, m_duty2);
	PK_DBG("LED1Closeflag = %d, LED2Closeflag = %d\n", LED1Closeflag, LED2Closeflag);

	if((LED1Closeflag == 1) && (LED2Closeflag == 1))
	{
		
	}
	else if(LED1Closeflag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
		{
			writeReg(LM3643_REG_LED2_TORCH, torchLEDReg[m_duty2]);
		}
		else
		{
	 	    writeReg(LM3643_REG_LED2_FLASH, flashLEDReg[m_duty2]);
		}
	}
	else if(LED2Closeflag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
		{
			writeReg(LM3643_REG_LED1_TORCH, torchLEDReg[m_duty1]);
		}
		else
		{
			writeReg(LM3643_REG_LED1_FLASH, flashLEDReg[m_duty1]);	
		}		
	}
	else
	{
		if((isMovieMode[m_duty1] == 1) && ((isMovieMode[m_duty2] == 1)))
		{
			writeReg(LM3643_REG_LED1_TORCH, torchLEDReg[m_duty1]);
			writeReg(LM3643_REG_LED2_TORCH, torchLEDReg[m_duty2]);
		}
		else
		{
	 	    writeReg(LM3643_REG_LED1_FLASH, flashLEDReg[m_duty1]);
			writeReg(LM3643_REG_LED2_FLASH, flashLEDReg[m_duty2]);
		}
	}

	return 0;
}


int FL_Enable(void)
{

	PK_DBG(" FL_Enable line=%d\n",__LINE__);


    return 0;
}



int FL_Disable(void)
{
	PK_DBG(" FL_Disable line=%d\n",__LINE__);
    return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
    setDuty_LM3643_1(duty);

    PK_DBG(" FL_dim_duty line=%d\n",__LINE__);
    return 0;
}




int FL_Init(void)
{
	PK_DBG("LED1_FL_Init!\n");
    if(mt_set_gpio_mode(GPIO_LED_EN,GPIO_MODE_00)){PK_DBG(" set gpio mode failed!! \n");}
    if(mt_set_gpio_dir(GPIO_LED_EN,GPIO_DIR_OUT)){PK_DBG(" set gpio dir failed!! \n");}
    if(mt_set_gpio_out(GPIO_LED_EN,GPIO_OUT_ONE)){PK_DBG(" set gpio failed!! \n");}

	writeReg(LM3643_REG_TIMING, 0x1F);

    INIT_WORK(&workTimeOut, work_timeOutFunc);
    PK_DBG(" FL_Init line=%d\n",__LINE__);
    return 0;
}


int FL_Uninit(void)
{
	PK_DBG("LED1_FL_Uninit!\n");
    return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/

static void work_timeOutFunc(struct work_struct *data)
{
    FL_Disable();
    PK_DBG("LED1TimeOut_callback\n");
}
/*task672921 modify by zhanyong.yin at 20150928 end*/


enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs = 1000;
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;
}



static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
/*task672921 modify by zhanyong.yin at 20150928 begin*/
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC,0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC,0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC,0, int));
	PK_DBG("LM3643_LED1_constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, (int)arg);
    switch(cmd)
    {
/*task672921 modify by zhanyong.yin at 20150928 end*/
	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
		PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n", (int)arg);
		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
		PK_DBG("FLASHLIGHT_DUTY: %d\n", (int)arg);
		m_duty1 = arg; //task672921 modify by zhanyong.yin at 20150928
		break;


	case FLASH_IOC_SET_STEP:
		PK_DBG("FLASH_IOC_SET_STEP: %d\n", (int)arg);

		break;
/*task672921 modify by zhanyong.yin at 20150928 begin*/
    	case FLASH_IOC_SET_ONOFF :
    		PK_DBG("FLASHLIGHT_ONOFF: %d\n",(int)arg);
    		if(arg==1)
    		{
				if(g_timeOutTimeMs!=0)
	            {
	            	ktime_t ktime;
					ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
					hrtimer_start( &g_timeOutTimer, ktime, HRTIMER_MODE_REL );
	            }
				LED1Closeflag = 0;
    			FL_Enable();
    		}
    		else
    		{
    			LED1Closeflag = 1;
    			FL_Disable();
				hrtimer_cancel( &g_timeOutTimer );
    		}
    		break;
    	case FLASH_IOC_SET_REG_ADR:
    	    break;
    	case FLASH_IOC_SET_REG_VAL:
    	    break;
    	case FLASH_IOC_SET_REG:
    	    break;
    	case FLASH_IOC_GET_REG:
    	    break;



		default :
    		PK_DBG(" No such command \n");
    		i4RetValue = -EPERM;
    		break;
    }
    return i4RetValue;
}
/*task672921 modify by zhanyong.yin at 20150928 end*/



static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res) {
		FL_Init();
		timerInit();
	}
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/
	spin_lock_irq(&g_strobeSMPLock);


	if (strobe_Res) {
		printk(" busy!\n");
		i4RetValue = -EBUSY;
	} else {
		strobe_Res += 1;
	}


	spin_unlock_irq(&g_strobeSMPLock);
	/*PK_DBG("constant_flashlight_open line=%d\n", __LINE__);*/

	return i4RetValue;

}


static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res) {
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;

}


FLASHLIGHT_FUNCTION_STRUCT constantFlashlightFunc = {
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};


MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
		*pfFunc = &constantFlashlightFunc;
	return 0;
}



/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{

	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);
