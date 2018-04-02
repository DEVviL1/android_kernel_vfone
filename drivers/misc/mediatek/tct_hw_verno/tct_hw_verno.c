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
#include <linux/mtd/mtd.h>
//#include <linux/mtd/nand.h>
#include <asm/io.h>
#include <asm/uaccess.h>
//#include <mach/mt_boot.h>
#include <linux/proc_fs.h>
#include "lcm_drv.h"
#include "../../../drivers/input/touchscreen/mediatek/tpd.h"
#include "tct_hw_verno.h"

extern LCM_DRIVER  *g_lcm_drv; 
extern char *g_main_camera;
extern char *g_sub_camera;
extern struct tpd_device  *tpd;
extern struct tpd_driver_t *g_tpd_drv;
extern u8 *g_tct_tp_fw_verno;

char* lcd_source_info[HW_LCD_SOURCE_TOTAL][TCT_SOURCE_INFO_MAX]=
{
/*{1st, name, version}*/
{HW_FIRST_SOURCE, HW_1ST_LCD_NAME, HW_1ST_LCD_PARA_VERNO},
{HW_SECOND_SOURCE, HW_2ND_LCD_NAME, HW_2ND_LCD_PARA_VERNO}
};

char* main_camera_source_info[HW_MAIN_CAMERA_SOURCE_TOTAL][TCT_SOURCE_INFO_MAX]=
{
/*{1st, name, version}*/
{HW_FIRST_SOURCE, HW_1ST_MAIN_CAMERA_NAME, HW_1ST_MAIN_CAMERA_PARA_VERNO},
{HW_SECOND_SOURCE, HW_2ND_MAIN_CAMERA_NAME, HW_2ND_MAIN_CAMERA_PARA_VERNO}
};

char* sub_camera_source_info[HW_SUB_CAMERA_SOURCE_TOTAL][TCT_SOURCE_INFO_MAX]=
{
/*{1st, name, version}*/
{HW_FIRST_SOURCE, HW_1ST_SUB_CAMERA_NAME, HW_1ST_SUB_CAMERA_PARA_VERNO},
{HW_SECOND_SOURCE, HW_2ND_SUB_CAMERA_NAME, HW_2ND_SUB_CAMERA_PARA_VERNO}
};

char* tp_source_info[HW_TP_SOURCE_TOTAL][TCT_SOURCE_INFO_MAX-1]=
{
/*{1st, name}*/
{HW_FIRST_SOURCE, HW_1ST_TP_NAME},
{HW_SECOND_SOURCE, HW_2ND_TP_NAME}
};

/****************************************************************************** 
 * Function Configuration
******************************************************************************/


/****************************************************************************** 
 * Debug configuration
******************************************************************************/
static ssize_t show_lcm(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int i = 0, index = -1;

    if(NULL!=g_lcm_drv)
    {
	for(i=0; i<HW_LCD_SOURCE_TOTAL; i++)
	{
	    if (strcmp(g_lcm_drv->name, lcd_source_info[i][TCT_SOURCE_INFO_NAME]) == 0)
	    { 
		index = i;
                break;
	    }
	}    	
        if(-1==index)
        	ret_value = sprintf(buf, "[LCD]: %s %s\n", g_lcm_drv->name, HW_NOT_IN_PURCHARSE);
	else 
		ret_value = sprintf(buf, "[%s LCD]: %s [Version]: %s\n",
					lcd_source_info[index][TCT_SOURCE_INFO_INDEX],
					lcd_source_info[index][TCT_SOURCE_INFO_NAME],
					lcd_source_info[index][TCT_SOURCE_INFO_VERNO]);  
    }
    else
    {
    	ret_value = sprintf(buf, "[LCD]: %s\n", "not_found");  
    }
    return ret_value;
}
static ssize_t store_lcm(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}

static ssize_t show_ctp(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int i = 0, index = -1;
    printk("[%s]: qqq tp fwverno:%s\n",__func__,g_tct_tp_fw_verno); 
    if(NULL!=g_tpd_drv)
    {
	for(i=0; i<HW_TP_SOURCE_TOTAL; i++)
	{
	    if (strcmp(g_tpd_drv->tpd_device_name, tp_source_info[i][TCT_SOURCE_INFO_NAME]) == 0)
	    { 
		index = i;
                break;
	    }
	}    	
        if(-1==index)
        	ret_value = sprintf(buf, "[TP]: %s %s\n", g_tpd_drv->tpd_device_name, HW_NOT_IN_PURCHARSE);
	else 
		ret_value = sprintf(buf, "[%s TP]: %s [Version]: %s\n",
					tp_source_info[index][TCT_SOURCE_INFO_INDEX],
					tp_source_info[index][TCT_SOURCE_INFO_NAME],
					g_tct_tp_fw_verno);  
    }
    else
    {
    	ret_value = sprintf(buf, "[TP]: %s\n", "not_found");  
    }
    
    return ret_value;
}
static ssize_t store_ctp(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}


static ssize_t show_main_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int i = 0, index = -1;

    if(NULL!=g_main_camera)
    {
	for(i=0; i<HW_MAIN_CAMERA_SOURCE_TOTAL; i++)
	{
	    if (strcmp(g_main_camera, main_camera_source_info[i][TCT_SOURCE_INFO_NAME]) == 0)
	    { 
		index = i;
                break;
	    }
	}    	
        if(-1==index)
        	ret_value = sprintf(buf, "[Main Camera]: %s %s\n", g_main_camera, HW_NOT_IN_PURCHARSE);
	else 
		ret_value = sprintf(buf, "[%s Main Camera]: %s [Version]: %s\n",
					main_camera_source_info[index][TCT_SOURCE_INFO_INDEX],
					main_camera_source_info[index][TCT_SOURCE_INFO_NAME],
					main_camera_source_info[index][TCT_SOURCE_INFO_VERNO]);  
    }
    else
    {
    	ret_value = sprintf(buf, "[Main Camera]: %s\n", "not_found");  
    }
 
    return ret_value;
}
static ssize_t store__main_camera(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}


static ssize_t show_sub_camera(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
    int i = 0, index = -1;

    if(NULL!=g_sub_camera)
    {
	for(i=0; i<HW_SUB_CAMERA_SOURCE_TOTAL; i++)
	{
	    if (strcmp(g_sub_camera, sub_camera_source_info[i][TCT_SOURCE_INFO_NAME]) == 0)
	    { 
		index = i;
                break;
	    }
	}    	
        if(-1==index)
        	ret_value = sprintf(buf, "[Sub Camera]: %s %s\n", g_sub_camera, HW_NOT_IN_PURCHARSE);
	else 
		ret_value = sprintf(buf, "[%s Sub Camera]: %s [Version]: %s\n",
					sub_camera_source_info[index][TCT_SOURCE_INFO_INDEX],
					sub_camera_source_info[index][TCT_SOURCE_INFO_NAME],
					sub_camera_source_info[index][TCT_SOURCE_INFO_VERNO]);  
    }
    else
    {
    	ret_value = sprintf(buf, "[Sub Camera]: %s\n", "not_found");  
    }

    return ret_value;
}
static ssize_t store_sub_camera(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}

static ssize_t show_rf(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
        ret_value = sprintf(buf , "[RF Version]: %s\n", RF_PARA_VERNO);
    return ret_value;
}
static ssize_t store_rf(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}

static ssize_t show_audio(struct device *dev,struct device_attribute *attr, char *buf)
{
    int ret_value = 1;
        ret_value = sprintf(buf , "[Audio Version]: %s\n", AUDIO_PARA_VERNO);
    return ret_value;
}
static ssize_t store_audio(struct device *dev,struct device_attribute *attr, const char *buf, size_t size)
{
    printk("[%s]: Not Support Write Function\n",__func__);  
    return size;
}

static DEVICE_ATTR(lcm, 0644, show_lcm, store_lcm);
static DEVICE_ATTR(ctp, 0644, show_ctp, store_ctp);
static DEVICE_ATTR(main_camera, 0644, show_main_camera, store__main_camera);
static DEVICE_ATTR(sub_camera, 0644, show_sub_camera, store_sub_camera);
static DEVICE_ATTR(rf, 0644, show_rf, store_rf);
static DEVICE_ATTR(audio, 0644, show_audio, store_audio);
///////////////////////////////////////////////////////////////////////////////////////////
//// platform_driver API 
///////////////////////////////////////////////////////////////////////////////////////////
static int tct_hw_verno_driver_probe(struct platform_device *dev)   
{   
    int ret_device_file = 0;

    printk("** tct_hw_verno_driver_probe!! **\n" );
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_lcm)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_ctp)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_main_camera)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_sub_camera)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_rf)) != 0) goto exit_error;
    if((ret_device_file = device_create_file(&(dev->dev), &dev_attr_audio)) != 0) goto exit_error;    
exit_error: 
    return ret_device_file;
}

static int tct_hw_verno_driver_remove(struct platform_device *dev)
{
    printk("** tct_hw_verno_drvier_remove!! **");

    device_remove_file(&(dev->dev), &dev_attr_lcm);
    device_remove_file(&(dev->dev), &dev_attr_ctp);
    device_remove_file(&(dev->dev), &dev_attr_main_camera);
    device_remove_file(&(dev->dev), &dev_attr_sub_camera);    
    device_remove_file(&(dev->dev), &dev_attr_rf);  
    device_remove_file(&(dev->dev), &dev_attr_audio);  
    return 0;
}

static struct platform_driver tct_hw_verno_driver = {
    .probe      = tct_hw_verno_driver_probe,
    .remove     = tct_hw_verno_driver_remove,
    .driver     = {
        .name = "tct_hw_verno",
    },
};

static struct platform_device tct_hw_verno_device = {
    .name   = "tct_hw_verno",
    .id     = -1,
};

static int __init tct_hw_verno_mod_init(void)
{
    int ret = 0;


    ret = platform_device_register(&tct_hw_verno_device);
    if (ret) {
        printk("tct_hw_verno_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_2;
    }
    

    ret = platform_driver_register(&tct_hw_verno_driver);
    if (ret) {
        printk("tct_hw_verno_mod_init  Unable to driver register(%d)\n", ret);
        goto  fail_1;
    }

    goto ok_result;

    
fail_1:
    platform_driver_unregister(&tct_hw_verno_driver);
fail_2:
    platform_device_unregister(&tct_hw_verno_device);
ok_result:

    return ret;
}


/*****************************************************************************/
static void __exit tct_hw_verno_mod_exit(void)
{
    platform_driver_unregister(&tct_hw_verno_driver);
    platform_device_unregister(&tct_hw_verno_device);
}
/*****************************************************************************/
module_init(tct_hw_verno_mod_init);
module_exit(tct_hw_verno_mod_exit);
/*****************************************************************************/
MODULE_DESCRIPTION(" Hareware verno Info");
MODULE_LICENSE("GPL");



