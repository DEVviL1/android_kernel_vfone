#include "tpd.h"
#include <linux/interrupt.h>
//#include <cust_eint.h>
#include <linux/i2c.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/rtpm_prio.h>
#include <linux/wait.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>


#include <tpd_custom_fts.h>
#include "focaltech_ex_fun.h"

#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/input/mt.h>
//#include <pmic_drv.h>
//#include "cust_gpio_usage.h"
//dma
#include <linux/dma-mapping.h>

#include "mt_boot_common.h"





//#define MT_PROTOCOL_B
//#define TPD_PROXIMITY
//#define FTS_GESTRUE
#define TPD_AUTO_UPGRADE				// if need upgrade CTP FW when POWER ON,pls enable this MACRO

//#define FTS_CTL_IIC
#define SYSFS_DEBUG
#define FTS_APK_DEBUG

#define USB_CHARGE_DETECT


#ifndef TPD_CLOSE_POWER_IN_SLEEP
#define TPD_CLOSE_POWER_IN_SLEEP
#endif

#ifndef TPD_POWER_SOURCE_CUSTOM
#define TPD_POWER_SOURCE_CUSTOM
#endif


#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif
#ifdef SYSFS_DEBUG
#include "focaltech_ex_fun.h"
#endif

#ifdef TPD_PROXIMITY

#define APS_ERR(fmt,arg...)           	printk("<<proximity>> "fmt"\n",##arg)

#define TPD_PROXIMITY_DEBUG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)

#define TPD_PROXIMITY_DMESG(fmt,arg...) printk("<<proximity>> "fmt"\n",##arg)

static u8 tpd_proximity_flag 			= 0;

static u8 tpd_proximity_flag_one 		= 0; //add for tpd_proximity by wangdongfang

static u8 tpd_proximity_detect 		= 1;//0-->close ; 1--> far away

#endif

#ifdef FTS_GESTRUE
#define  KEY_GESTURE_U KEY_U
#define  KEY_GESTURE_UP KEY_UP
#define  KEY_GESTURE_DOWN KEY_DOWN
#define  KEY_GESTURE_LEFT KEY_LEFT 
#define  KEY_GESTURE_RIGHT KEY_RIGHT
#define  KEY_GESTURE_O KEY_O
#define  KEY_GESTURE_E KEY_E
#define  KEY_GESTURE_M KEY_M 
#define  KEY_GESTURE_L KEY_L
#define  KEY_GESTURE_W KEY_W
#define  KEY_GESTURE_S KEY_S 
#define  KEY_GESTURE_V KEY_V
#define  KEY_GESTURE_Z KEY_Z


#define GESTURE_LEFT		0x20
#define GESTURE_RIGHT		0x21
#define GESTURE_UP		    0x22
#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24
#define GESTURE_O		    0x30
#define GESTURE_W		    0x31
#define GESTURE_M		    0x32
#define GESTURE_E		    0x33
#define GESTURE_L		    0x44
#define GESTURE_S		    0x46
#define GESTURE_V		    0x54
#define GESTURE_Z		    0x41

#include "ft_gesture_lib.h"

#define FTS_GESTRUE_POINTS 255
#define FTS_GESTRUE_POINTS_ONETIME  62
#define FTS_GESTRUE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};
#endif
 
extern struct tpd_device *tpd;
 
struct i2c_client *i2c_client = NULL;
struct task_struct *thread = NULL;


#ifndef MT_PROTOCOL_B
#define key_coord_y         2000
#define key_coord_back_x    172
#define key_coord_menu_x    826


#endif


struct Upgrade_Info fts_updateinfo[] =
{
	{0x54,"FT5x46",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,2, 2, 0x54, 0x2c, 10, 2000},
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000},
	{0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 10, 0x79, 0x06, 100, 2000},
	{0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 10, 1500},
	{0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
	{0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,10, 10, 0x79, 0x18, 10, 2000},
	{0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 10, 2000},
	{0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	{0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
	
};
				
struct Upgrade_Info fts_updateinfo_curr;

static DECLARE_WAIT_QUEUE_HEAD(waiter);
static DEFINE_MUTEX(i2c_access); 
static DEFINE_MUTEX(i2c_rw_access);
 
 static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id);

extern int fts_create_sysfs(struct i2c_client * client);
extern void fts_release_sysfs(struct i2c_client * client);
extern int ft5x0x_create_apk_debug_channel(struct i2c_client *client);
extern void ft5x0x_release_apk_debug_channel(void);

static int  tpd_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info);
static int  tpd_remove(struct i2c_client *client);
static int touch_event_handler(void *unused);
#ifdef TPD_PROXIMITY
extern  int (*ps_enable_nodata)(int en);
extern  int (*ps_get_data)(int* value, int* status);
extern int ps_report_interrupt_data(int value);
#endif
static int tpd_flag = 0;
static int tpd_halt=0;
#ifndef MT_PROTOCOL_B
static int point_num = 0;
static int p_point_num = 0;
#endif
unsigned int ft_touch_irq = 0;

int up_flag=0,up_count=0;

#ifdef USB_CHARGE_DETECT
static u8 b_usb_plugin = 0;
#endif


#define __MSG_DMA_MODE__  //
#ifdef __MSG_DMA_MODE__
	static u8 *g_dma_buff_va = NULL;    //
	static dma_addr_t g_dma_buff_pa = 0;    // 
#endif

#ifdef __MSG_DMA_MODE__
static void msg_dma_alloct(void){

		tpd->dev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
		g_dma_buff_va = (u8 *) dma_alloc_coherent(&tpd->dev->dev, 255,
			&g_dma_buff_pa, GFP_KERNEL);
		if (!g_dma_buff_va) {
			TPD_DMESG("[DMA][Error] Allocate DMA I2C Buffer failed!\n");
			return ;
		}
		memset(g_dma_buff_va, 0, 255);
}
static void msg_dma_release(void){
	if(g_dma_buff_va){
     	dma_free_coherent(NULL, /*4096*/255, g_dma_buff_va, g_dma_buff_pa);
        g_dma_buff_va = NULL;
        g_dma_buff_pa = 0;
		TPD_DMESG("[DMA][release] Allocate DMA I2C Buffer release!\n");
    }
}
#endif


#define TPD_OK 0
//register define

#define DEVICE_MODE 0x00
#define GEST_ID 0x01
#define TD_STATUS 0x02

#define TOUCH1_XH 0x03
#define TOUCH1_XL 0x04
#define TOUCH1_YH 0x05
#define TOUCH1_YL 0x06

#define TOUCH2_XH 0x09
#define TOUCH2_XL 0x0A
#define TOUCH2_YH 0x0B
#define TOUCH2_YL 0x0C

#define TOUCH3_XH 0x0F
#define TOUCH3_XL 0x10
#define TOUCH3_YH 0x11
#define TOUCH3_YL 0x12
//register define

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_MAX_RESET_COUNT 3


struct ts_event {
	u16 au16_x[CFG_MAX_TOUCH_POINTS];	/*x coordinate */
	u16 au16_y[CFG_MAX_TOUCH_POINTS];	/*y coordinate */
	u8 au8_touch_event[CFG_MAX_TOUCH_POINTS];	/*touch event:
					0 -- down; 1-- up; 2 -- contact */
	u8 au8_finger_id[CFG_MAX_TOUCH_POINTS];	/*touch ID */
	u16 pressure;
	u8 touch_point;
};


//#ifdef TPD_HAVE_BUTTON 
//static int tpd_keys_local[TPD_KEY_COUNT] = TPD_KEYS;
//static int tpd_keys_dim_local[TPD_KEY_COUNT][4] = TPD_KEYS_DIM;
//#endif
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))
static int tpd_wb_start_local[TPD_WARP_CNT] = TPD_WARP_START;
static int tpd_wb_end_local[TPD_WARP_CNT]   = TPD_WARP_END;
#endif
#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
static int tpd_calmat_local[8]     = TPD_CALIBRATION_MATRIX;
static int tpd_def_calmat_local[8] = TPD_CALIBRATION_MATRIX;
#endif

//#define VELOCITY_CUSTOM_fts
#ifdef VELOCITY_CUSTOM_fts
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>

// for magnify velocity********************************************

#ifndef TPD_VELOCITY_CUSTOM_X
#define TPD_VELOCITY_CUSTOM_X 10
#endif
#ifndef TPD_VELOCITY_CUSTOM_Y
#define TPD_VELOCITY_CUSTOM_Y 10
#endif

#define TOUCH_IOC_MAGIC 'A'

#define TPD_GET_VELOCITY_CUSTOM_X _IO(TOUCH_IOC_MAGIC,0)
#define TPD_GET_VELOCITY_CUSTOM_Y _IO(TOUCH_IOC_MAGIC,1)

int g_v_magnify_x =TPD_VELOCITY_CUSTOM_X;
int g_v_magnify_y =TPD_VELOCITY_CUSTOM_Y;


static int tpd_misc_open(struct inode *inode, struct file *file)
{
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int tpd_misc_release(struct inode *inode, struct file *file)
{
	return 0;
}
/*----------------------------------------------------------------------------*/

static long tpd_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{

	void __user *data;
	
	long err = 0;
	
	if(_IOC_DIR(cmd) & _IOC_READ)
	{
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	}
	else if(_IOC_DIR(cmd) & _IOC_WRITE)
	{
		err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	}

	if(err)
	{
		printk("tpd: access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case TPD_GET_VELOCITY_CUSTOM_X:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_x, sizeof(g_v_magnify_x)))
			{
				err = -EFAULT;
				break;
			}				 
			break;

	   case TPD_GET_VELOCITY_CUSTOM_Y:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &g_v_magnify_y, sizeof(g_v_magnify_y)))
			{
				err = -EFAULT;
				break;
			}				 
			break;


		default:
			printk("tpd: unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}


static struct file_operations tpd_fops = {
//	.owner = THIS_MODULE,
	.open = tpd_misc_open,
	.release = tpd_misc_release,
	.unlocked_ioctl = tpd_unlocked_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice tpd_misc_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "touch",
	.fops = &tpd_fops,
};

//**********************************************
#endif

struct touch_info {
    int y[10];
    int x[10];
    int p[10];
    int id[10];
    int count;
};
 
 static const struct i2c_device_id fts_tpd_id[] = {{"fts",0},{}};
 static const struct of_device_id  ft5436_dt_match[] = {
	{.compatible = "mediatek,cap_touch"},
	{},
 	};
 static unsigned short force[] = { 0, 0x38, I2C_CLIENT_END, I2C_CLIENT_END };
static const unsigned short *const forces[] = { force, NULL };
//static struct i2c_board_info __initdata fts_i2c_tpd={ I2C_BOARD_INFO("fts", (0x70>>1))};
 
 
 static struct i2c_driver tpd_i2c_driver = {
  .driver = {
  .name = "fts",
  .of_match_table = of_match_ptr(ft5436_dt_match),
//.owner = THIS_MODULE,
  },
  .probe = tpd_probe,
  .remove = tpd_remove,
  .id_table = fts_tpd_id,
  .detect = tpd_detect,
   .id_table = fts_tpd_id,
   .address_list = (const unsigned short *)forces,
 };
 
#ifdef USB_CHARGE_DETECT
void tpd_usb_plugin(int plugin)
{	
    int ret = -1;		
	
	b_usb_plugin = plugin;	
	printk("Fts usb detect: %s %d %d.\n",__func__,b_usb_plugin,tpd_halt);	
	
	if ( tpd_halt ) 
		return;	
	
	ret = i2c_smbus_write_i2c_block_data(i2c_client, 0x8B, 1, &b_usb_plugin);	
	if ( ret < 0 )	
	{		
	    printk("Fts usb detect write err: %s %d.\n",__func__,b_usb_plugin);	
	}
}
EXPORT_SYMBOL(tpd_usb_plugin);
#endif


static int tpd_irq_registration(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	u32 ints[2] = { 0, 0 };

	TPD_DMESG("Device Tree Tpd_irq_registration!\n");

	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		ft_touch_irq = irq_of_parse_and_map(node, 0);
			ret =
			    request_irq(ft_touch_irq,  tpd_eint_interrupt_handler, IRQF_TRIGGER_FALLING,
					"TOUCH_PANEL-eint", NULL);
			if (ret > 0) {
				ret = -1;
				TPD_DMESG("tpd request_irq IRQ LINE NOT AVAILABLE!.");
			}
		
	} else {
		TPD_DMESG("tpd request_irq can not find touch eint device node!.");
		ret = -1;
	}
	TPD_DMESG("[%s]irq:%d, debounce:%d-%d:\n", __func__, ft_touch_irq, ints[0], ints[1]);
	return ret;
}
#if 1
int fts_i2c_Read(struct i2c_client *client, u8 *writebuf,int writelen, u8 *readbuf, int readlen)
{
	int ret=0;

	// for DMA I2c transfer

	mutex_lock(&i2c_rw_access);
       client->timing = 200000/1000;
		if(writelen!=0)
	{
			if (readlen <8)
		{
			ret = i2c_master_send(client, writebuf, writelen);
		}
		else
	{
		//DMA Write
		memcpy(g_dma_buff_va, writebuf, writelen);

		client->addr = (client->addr & I2C_MASK_FLAG) | (I2C_DMA_FLAG);
		//if((ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen))!=writelen)
		//	dev_err(&client->dev, "###%s i2c write len=%x,buffaddr=%d\n", __func__,ret,g_dma_buff_pa);
		ret=i2c_master_send(client,(unsigned char *) g_dma_buff_pa, writelen);

		client->addr =( client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG);
	}
			}
	//DMA Read 

	if(readlen!=0)

	{

	      if (readlen <8)
		{
			ret = i2c_master_recv(client, readbuf, readlen);
		}
		else
		{
		client->addr = (client->addr & I2C_MASK_FLAG )|( I2C_DMA_FLAG);
		ret = i2c_master_recv(client, (unsigned char *)g_dma_buff_pa, readlen);
		memcpy(readbuf, g_dma_buff_va, readlen);
		client->addr =( client->addr & I2C_MASK_FLAG) &(~ I2C_DMA_FLAG);
			}
	}   

	if(ret<=0)
	{
		TPD_DMESG("[fts]fts_i2c_Read error line = %d, ret = %d\n", __LINE__, ret);
		mutex_unlock(&i2c_rw_access);
		return -1;
	}

	mutex_unlock(&i2c_rw_access);
	return ret;

}

/*write data by i2c*/
int fts_i2c_Write(struct i2c_client *client, u8 *writebuf, int writelen)
{
	int ret=0;

	mutex_lock(&i2c_rw_access);
	
 	//client->addr = client->addr & I2C_MASK_FLAG;
	//ret = i2c_master_send(client, writebuf, writelen);

      client->timing = 100;
     if(writelen<= 8)
	{
		TPD_DMESG("[FT3407]writelen<= 8\n");

		ret = i2c_master_send(client, writebuf, writelen);
	}
	else
	{
		memcpy(g_dma_buff_va, writebuf, writelen);
	      client->addr = (client->addr & I2C_MASK_FLAG )|( I2C_DMA_FLAG);
		ret=i2c_master_send(client, (unsigned char *)g_dma_buff_pa, writelen);
		client->addr = (client->addr & I2C_MASK_FLAG )&(~ I2C_DMA_FLAG);
	}

	if(ret<=0)
	{
		TPD_DMESG("[FT6436]i2c_write_byte error line = %d, ret = %d\n", __LINE__, ret);
		mutex_unlock(&i2c_rw_access);
		return -1;
	}


	mutex_unlock(&i2c_rw_access);
	
	return ret;
}
#endif

int fts_write_reg(struct i2c_client *client, u8 regaddr, u8 regvalue)
{
	unsigned char buf[2] = {0};

	buf[0] = regaddr;
	buf[1] = regvalue;

	return fts_i2c_Write(client, buf, sizeof(buf));
}

int fts_read_reg(struct i2c_client *client, u8 regaddr, u8 *regvalue)
{

	return fts_i2c_Read(client, &regaddr, 1, regvalue, 1);

}

static  int fts_read_i2c(u8 addr, u8 *pdata)
{
        int ret;
        u8 buf[1];
        i2c_master_send(i2c_client, &addr, 1);
        ret=i2c_master_recv(i2c_client, buf, 1);
        if (ret < 0)
                pr_err("msg %s i2c read error: %d\n", __func__, ret);

        *pdata = buf[0];
        return ret;
}



void focaltech_get_upgrade_array(void)
{

	u8 chip_id;
	u32 i;

	//i2c_smbus_read_i2c_block_data(i2c_client,FTS_REG_CHIP_ID,1,&chip_id);
	fts_read_i2c(FTS_REG_CHIP_ID,&chip_id);

	printk("%s chip_id = %x\n", __func__, chip_id);

	for(i=0;i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info);i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			memcpy(&fts_updateinfo_curr, &fts_updateinfo[i], sizeof(struct Upgrade_Info));
			break;
		}
	}

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		memcpy(&fts_updateinfo_curr, &fts_updateinfo[0], sizeof(struct Upgrade_Info));
	}
}

#ifndef MT_PROTOCOL_B
static  void tpd_down(int x, int y, int p) {

	if(x > TPD_RES_X)
	{
		TPD_DEBUG("warning: IC have sampled wrong value.\n");;
		return;
	}

	#if 1
	 input_report_key(tpd->dev, BTN_TOUCH, 1);
	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
	 input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0x3f);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);
	 //printk("tpd:D[%4d %4d %4d] ", x, y, p);
	 /* track id Start 0 */
     //input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p);
	 input_mt_sync(tpd->dev);
     #else
	 if(y==key_coord_y)
	 {
	     if(p==0)
	     {
             if(x==key_coord_back_x)
    		     input_report_key(tpd->dev, KEY_BACK, 1);
    		 else if(x==key_coord_menu_x)
    		     input_report_key(tpd->dev, KEY_MENU, 1);
    		 //else if(x==0xf0)
    		 //	 input_report_key(tpd->dev, KEY_HOMEPAGE, 1);
    		 //else
    		 	//input_report_key(tpd->dev, KEY_UNKNOWN, 1);
	     }
	 }
	 else
	 {
	     input_report_key(tpd->dev, BTN_TOUCH, 1);
    	 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 20);
    	 input_report_abs(tpd->dev, ABS_MT_PRESSURE, 0x3f);
    	 input_report_abs(tpd->dev, ABS_MT_POSITION_X, x);
    	 input_report_abs(tpd->dev, ABS_MT_POSITION_Y, y);

	 //printk("tpd:D[%4d %4d %4d] ", x, y, p);
	 /* track id Start 0 */
     //input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, p);
	     input_mt_sync(tpd->dev);
	 }
	 #endif

     if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
     {
       tpd_button(x, y, 1);
     }
	 if(y > TPD_RES_Y) //virtual key debounce to avoid android ANR issue
	 {
         //msleep(50);
		 printk("D virtual key \n");
	 }
	 TPD_EM_PRINT(x, y, x, y, p-1, 1);
 }

static  void tpd_up(int x, int y,int *count)
{
	 input_report_key(tpd->dev, BTN_TOUCH, 0);
	 //printk("U[%4d %4d %4d] ", x, y, 0);


     #if 0
	 if((y==key_coord_y)&&(p==1))
	 {
         if(x==key_coord_back_x)
		     input_report_key(tpd->dev, KEY_BACK, 0);
		 else if(x==key_coord_menu_x)
		     input_report_key(tpd->dev, KEY_MENU, 0);
		 //else if(x==0xf0)
		 	// input_report_key(tpd->dev, KEY_HOMEPAGE, 0);
		 //else
		 //	input_report_key(tpd->dev, KEY_UNKNOWN, 0);
	 }
	 else
	     input_mt_sync(tpd->dev);
	 #endif
	 //input_mt_sync(tpd->dev);
	 TPD_EM_PRINT(x, y, x, y, 0, 0);

	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{
		tpd_button(x, y, 0);
	}
 }

#if 0
static  void tpd_key_up(int x, int y,int p)
{

	 //printk("U[%4d %4d %4d] ", x, y, 0);


     #if 1
	 if((y==key_coord_y)&&(p==1))
	 {
         if(x==key_coord_back_x)
		     input_report_key(tpd->dev, KEY_BACK, 0);
		 else if(x==key_coord_menu_x)
		     input_report_key(tpd->dev, KEY_MENU, 0);
		 //else if(x==0xf0)
		 //	 input_report_key(tpd->dev, KEY_HOMEPAGE, 0);
		 //else
		 //	input_report_key(tpd->dev, KEY_UNKNOWN, 0);

	 }

	 #endif
	 //input_mt_sync(tpd->dev);
	 //TPD_EM_PRINT(x, y, x, y, 0, 0);

	if (FACTORY_BOOT == get_boot_mode()|| RECOVERY_BOOT == get_boot_mode())
	{
		tpd_button(x, y, 0);
	}
}
#endif
#endif

#ifndef MT_PROTOCOL_B

#if 1
static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo,struct touch_info *ptest)
{
	int i = 0;
	char data[128] = {0};
       u16 high_byte,low_byte;
    u8 reg;
	   
	//u8 report_rate =0;
	p_point_num = point_num;
	if (tpd_halt)
	{
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}
	mutex_lock(&i2c_access);

       reg = 0x00;
	fts_i2c_Read(i2c_client, &reg, 1, data, 64);
	mutex_unlock(&i2c_access);

	/*get the number of the touch points*/

	point_num= data[2] & 0x0f;
	if(up_flag==2)
	{
		up_flag=0;
		for(i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++)  
		{
				cinfo->p[i] = data[3+6*i] >> 6; //event flag 
			cinfo->id[i] = data[3+6*i+2]>>4; //touch id
		   	/*get the X coordinate, 2 bytes*/
			high_byte = data[3+6*i];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i + 1];
			cinfo->x[i] = high_byte |low_byte;	
			high_byte = data[3+6*i+2];
			high_byte <<= 8;
			high_byte &= 0x0f00;
			low_byte = data[3+6*i+3];
			cinfo->y[i] = high_byte |low_byte;

			
			if(point_num>=i+1)
				continue;
			if(up_count==0)
				continue;
			cinfo->p[i] = ptest->p[i-point_num]; //event flag 
			
			
			cinfo->id[i] = ptest->id[i-point_num]; //touch id
	
			cinfo->x[i] = ptest->x[i-point_num];	
			
			cinfo->y[i] = ptest->y[i-point_num];
			//dev_err(&fts_i2c_client->dev," zax add two x = %d, y = %d, evt = %d,id=%d\n", cinfo->x[i], cinfo->y[i], cinfo->p[i], cinfo->id[i]);
			up_count--;
			
				
		}
		
		return true;
	}
	up_count=0;
	for(i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++)  
	{
		cinfo->p[i] = data[3+6*i] >> 6; //event flag 
		
		if(0==cinfo->p[i])
		{
			//dev_err(&fts_i2c_client->dev,"\n  zax enter add   \n");
			up_flag=1;
     		}
     		cinfo->id[i] = data[3+6*i+2]>>4; //touch id
	   	/*get the X coordinate, 2 bytes*/
		high_byte = data[3+6*i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i + 1];
		cinfo->x[i] = high_byte |low_byte;	
		high_byte = data[3+6*i+2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i+3];
		cinfo->y[i] = high_byte |low_byte;

		if(up_flag==1 && 1==cinfo->p[i])
		{
			up_flag=2;
			point_num++;
			ptest->x[up_count]=cinfo->x[i];
			ptest->y[up_count]=cinfo->y[i];
			ptest->id[up_count]=cinfo->id[i];
			ptest->p[up_count]=cinfo->p[i];
			//dev_err(&fts_i2c_client->dev," zax add x = %d, y = %d, evt = %d,id=%d\n", ptest->x[j], ptest->y[j], ptest->p[j], ptest->id[j]);
			cinfo->p[i]=2;
			up_count++;
		}
	}
	if(up_flag==1)
		up_flag=0;
	//printk(" tpd cinfo->x[0] = %d, cinfo->y[0] = %d, cinfo->p[0] = %d\n", cinfo->x[0], cinfo->y[0], cinfo->p[0]);
	return true;

}
#else

#define MAX_TOUCH_NUM   5

static int tpd_touchinfo(struct touch_info *cinfo, struct touch_info *pinfo)
{
	int i = 0;
	char data[128] = {0};
    u16 high_byte,low_byte;
	u8  reg;
	//u8 report_rate =0;

	int p_tmp;
	int j = 0;
	int k = 0;
	int *t;
	struct touch_info *info_tmp;



	p_point_num = point_num;
	if (tpd_halt)
	{
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}
	mutex_lock(&i2c_access);

       reg = 0x00;
	fts_i2c_Read(i2c_client, &reg, 1, data, 64);
	mutex_unlock(&i2c_access);


	point_num= data[2] & 0x0f;

	cinfo->count = 0;
	pinfo->count = 0;
	j = 0;
	k = 0;

	for(i = 0; i < MAX_TOUCH_NUM; i++)
	{
	    p_tmp = data[3+6*i] >> 6;
	    if((p_tmp==0)||(p_tmp==2))
	    {
	        info_tmp = cinfo;
			t = &j;
	    }
		else if(p_tmp==1)
		{
		    info_tmp = pinfo;
			t = &k;
		}
		else
			continue;

		info_tmp->p[*t] = data[3+6*i] >> 6; //event flag
     		info_tmp->id[*t] = data[3+6*i+2]>>4; //touch id
	   	/*get the X coordinate, 2 bytes*/
		high_byte = data[3+6*i];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i + 1];
		info_tmp->x[*t] = high_byte |low_byte;
		high_byte = data[3+6*i+2];
		high_byte <<= 8;
		high_byte &= 0x0f00;
		low_byte = data[3+6*i+3];
		info_tmp->y[*t] = high_byte |low_byte;

		info_tmp->count++;
		(*t)++;
	}

	if(point_num!=cinfo->count)
		return false;
	else
		return true;

}
#endif

#endif



#ifdef MT_PROTOCOL_B
/*
*report the point information
*/
static int fts_read_Touchdata(struct ts_event *pinfo)
{
       u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;
	u8 cnt_down;

	if (tpd_halt)
	{
		TPD_DMESG( "tpd_touchinfo return ..\n");
		return false;
	}

	mutex_lock(&i2c_access);
	ret = fts_i2c_Read(i2c_client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&i2c_client->dev, "%s read touchdata failed.\n",__func__);
		mutex_unlock(&i2c_access);
		return ret;
	}
	mutex_unlock(&i2c_access);
	memset(pinfo, 0, sizeof(struct ts_event));
	
	pinfo->touch_point = 0;
	//printk("tpd  fts_updateinfo_curr.TPD_MAX_POINTS=%d fts_updateinfo_curr.chihID=%d \n", fts_updateinfo_curr.TPD_MAX_POINTS,fts_updateinfo_curr.CHIP_ID);
	for (i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++)
	{
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			pinfo->touch_point++;
		pinfo->au16_x[i] =
		    (s16) (buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i];
		pinfo->au16_y[i] =
		    (s16) (buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i] & 0x0F) <<
		    8 | (s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i];
		pinfo->au8_touch_event[i] =
		    buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		pinfo->au8_finger_id[i] =
		    (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
	}
	
	return 0;
}

 /*
 *report the point information
 */
static void fts_report_value(struct ts_event *data)
 {
	 struct ts_event *event = data;
	 int i = 0;
	 int up_point = 0;

	 //printk("tpd fts_report_value  touch_point=%d!!!!!",event->touch_point);
 
	 for (i = 0; i < event->touch_point; i++) 
	 {
		 input_mt_slot(tpd->dev, event->au8_finger_id[i]);

		  //printk("tpd fts_report_value  au8_touch_event[%d]=%d!!!!!",i,event->au8_touch_event[i]);
 
		 if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
			 {
				 input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,true);
				 input_report_abs(tpd->dev, ABS_MT_PRESSURE,0x3f);
				 input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,0x05);
				 input_report_abs(tpd->dev, ABS_MT_POSITION_X,event->au16_x[i]);
				 input_report_abs(tpd->dev, ABS_MT_POSITION_Y,event->au16_y[i]);
              //printk("tpd D x[%d] =%d,y[%d]= %d",i,event->au16_x[i],i,event->au16_y[i]);
			 }
			 else
			 {
				 up_point++;
				 input_mt_report_slot_state(tpd->dev, MT_TOOL_FINGER,false);
			 }				 
		 
	 }
 
	 if(event->touch_point == up_point)
		 input_report_key(tpd->dev, BTN_TOUCH, 0);
	 else
		 input_report_key(tpd->dev, BTN_TOUCH, 1);
 
	 input_sync(tpd->dev);
    //printk("tpd D x =%d,y= %d",event->au16_x[0],event->au16_y[0]);
 }
#endif

#ifdef TPD_PROXIMITY


static int tpd_get_ps_value (int* value, int* status)
{
	return tpd_proximity_detect;
}

static int tpd_enable_ps(int enable)
{
	u8 state;
	int ret = -1;
	
	i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &state);
	printk("[proxi_fts]read: 999 0xb0's value is 0x%02X\n", state);

	if (enable){
		state |= 0x01;
		tpd_proximity_flag = 1;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is on\n");	
	}else{
		state &= 0x00;	
		tpd_proximity_flag = 0;
		TPD_PROXIMITY_DEBUG("[proxi_fts]ps function is off\n");
	}
	
	ret = i2c_smbus_write_i2c_block_data(i2c_client, 0xB0, 1, &state);
	TPD_PROXIMITY_DEBUG("[proxi_fts]write: 0xB0's value is 0x%02X\n", state);
	return 0;
}
#if 0
int tpd_ps_operate(void* self, uint32_t command, void* buff_in, int size_in,

		void* buff_out, int size_out, int* actualout)
{
	int err = 0;
	int value;
	hwm_sensor_data *sensor_data;
	TPD_DEBUG("[proxi_fts]command = 0x%02X\n", command);		
	
	switch (command)
	{
		case SENSOR_DELAY:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Set delay parameter error!\n");
				err = -EINVAL;
			}
			// Do nothing
			break;
		case SENSOR_ENABLE:
			if((buff_in == NULL) || (size_in < sizeof(int)))
			{
				APS_ERR("Enable sensor parameter error!\n");
				err = -EINVAL;
			}
			else
			{				
				value = *(int *)buff_in;
				if(value)
				{		
					if((tpd_enable_ps(1) != 0))
					{
						APS_ERR("enable ps fail: %d\n", err); 
						return -1;
					}
				}
				else
				{
					if((tpd_enable_ps(0) != 0))
					{
						APS_ERR("disable ps fail: %d\n", err); 
						return -1;
					}
				}
			}
			break;
		case SENSOR_GET_DATA:
			if((buff_out == NULL) || (size_out< sizeof(hwm_sensor_data)))
			{
				APS_ERR("get sensor data parameter error!\n");
				err = -EINVAL;
			}
			else
			{
				sensor_data = (hwm_sensor_data *)buff_out;				
				if((err = tpd_read_ps()))
				{
					err = -1;;
				}
				else
				{
					sensor_data->values[0] = tpd_get_ps_value();
					TPD_PROXIMITY_DEBUG("huang sensor_data->values[0] 1082 = %d\n", sensor_data->values[0]);
					sensor_data->value_divide = 1;
					sensor_data->status = SENSOR_STATUS_ACCURACY_MEDIUM;
				}					
			}
			break;
		default:
			APS_ERR("proxmy sensor operate function no this parameter %d!\n", command);
			err = -1;
			break;
	}
	return err;	
}
#endif
#endif

#ifdef FTS_GESTRUE
static void check_gesture(int gesture_id)
{
    printk("fts gesture_id==0x%x\n ",gesture_id);
	switch(gesture_id)
	{
		case GESTURE_LEFT:
				input_report_key(tpd->dev, KEY_GESTURE_LEFT, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_LEFT, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_RIGHT:
				input_report_key(tpd->dev, KEY_GESTURE_RIGHT, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_RIGHT, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_UP:
				input_report_key(tpd->dev, KEY_GESTURE_UP, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_UP, 0);
				input_sync(tpd->dev);			
			break;
		case GESTURE_DOWN:
				input_report_key(tpd->dev, KEY_GESTURE_DOWN, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_DOWN, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_DOUBLECLICK:
				input_report_key(tpd->dev, KEY_GESTURE_U, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_U, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_O:
				input_report_key(tpd->dev, KEY_GESTURE_O, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_O, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_W:
				input_report_key(tpd->dev, KEY_GESTURE_W, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_W, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_M:
				input_report_key(tpd->dev, KEY_GESTURE_M, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_M, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_E:
				input_report_key(tpd->dev, KEY_GESTURE_E, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_E, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_L:
				input_report_key(tpd->dev, KEY_GESTURE_L, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_L, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_S:
				input_report_key(tpd->dev, KEY_GESTURE_S, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_S, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_V:
				input_report_key(tpd->dev, KEY_GESTURE_V, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_V, 0);
				input_sync(tpd->dev);
			break;
		case GESTURE_Z:
				input_report_key(tpd->dev, KEY_GESTURE_Z, 1);
				input_sync(tpd->dev);
				input_report_key(tpd->dev, KEY_GESTURE_Z, 0);
				input_sync(tpd->dev);
			break;
		default:
			break;
	}
}

static int fts_read_Gestruedata(void)
{
    unsigned char buf[FTS_GESTRUE_POINTS * 3] = { 0 };
    int ret = -1;
    int i = 0;
    buf[0] = 0xd3;
    int gestrue_id = 0;
    short pointnum = 0;

    pointnum = 0;
    ret = fts_i2c_Read(i2c_client, buf, 1, buf, FTS_GESTRUE_POINTS_HEADER);
	//printk( "tpd read FTS_GESTRUE_POINTS_HEADER.\n");
    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }

    /* FW */
     if (fts_updateinfo_curr.CHIP_ID==0x54)
     {
     		 gestrue_id = buf[0];
		 pointnum = (short)(buf[1]) & 0xff;
	 	 buf[0] = 0xd3;
	 
	 	 if((pointnum * 4 + 8)<255)
	 	 {
	 	    	 ret = fts_i2c_Read(i2c_client, buf, 1, buf, (pointnum * 4 + 8));
	 	 }
	 	 else
	 	 {
	 	        ret = fts_i2c_Read(i2c_client, buf, 1, buf, 255);
	 	        ret = fts_i2c_Read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
	 	 }
	 	 if (ret < 0)
	 	 {
	 	       printk( "%s read touchdata failed.\n", __func__);
	 	       return ret;
	 	 }
        	 check_gesture(gestrue_id);
		 for(i = 0;i < pointnum;i++)
	        {
	        	coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
	            	8 | (((s16) buf[1 + (4 * i)])& 0xFF);
	        	coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
	            	8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	   	 }
        	 return -1;
     }

    if (0x24 == buf[0])

    {
        gestrue_id = 0x24;
        check_gesture(gestrue_id);
		printk( "tpd %d check_gesture gestrue_id.\n", gestrue_id);
        return -1;
    }
	
    pointnum = (short)(buf[1]) & 0xff;
    buf[0] = 0xd3;
    if((pointnum * 4 + 8)<255)
    {
    	ret = fts_i2c_Read(i2c_client, buf, 1, buf, (pointnum * 4 + 8));
    }
    else
    {
         ret = fts_i2c_Read(i2c_client, buf, 1, buf, 255);
         ret = fts_i2c_Read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
    }
    if (ret < 0)
    {
        printk( "%s read touchdata failed.\n", __func__);
        return ret;
    }

   gestrue_id = fetch_object_sample(buf, pointnum);
   check_gesture(gestrue_id);
   printk( "tpd %d read gestrue_id.\n", gestrue_id);

    for(i = 0;i < pointnum;i++)
    {
        coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
            8 | (((s16) buf[1 + (4 * i)])& 0xFF);
        coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
            8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
    }
    return -1;
}
#endif

 static int touch_event_handler(void *unused)
 {
    #ifndef MT_PROTOCOL_B
	struct touch_info cinfo, pinfo,ptest;
	#endif
	
	#ifdef MT_PROTOCOL_B
	struct ts_event pevent;
	#endif

	#ifndef MT_PROTOCOL_B
	int i=0;
	#endif
	
	#ifdef TPD_PROXIMITY
	int err;
	u8 proximity_status;
	u8 p_state;
	#endif
	#ifdef FTS_GESTRUE
	u8 state;
	#endif

	#ifdef MT_PROTOCOL_B
    int ret;
	#endif
	
	 struct sched_param param = { .sched_priority = RTPM_PRIO_TPD };
	 sched_setscheduler(current, SCHED_RR, &param);
	 do
	 {
		// mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM); 
		 set_current_state(TASK_INTERRUPTIBLE); 
		 wait_event_interruptible(waiter,tpd_flag!=0);
						 
		 tpd_flag = 0;
			 
		 set_current_state(TASK_RUNNING);
		 //printk("tpd touch_event_handler\n");
	 	 #ifdef FTS_GESTRUE
			i2c_smbus_read_i2c_block_data(i2c_client, 0xd0, 1, &state);
			//printk("tpd fts_read_Gestruedata state=%d\n",state);
		     if(state ==1)
		     {
		        fts_read_Gestruedata();
		        continue;
		    }
		 #endif

		 #ifdef TPD_PROXIMITY

		 if (tpd_proximity_flag == 1)
		 {

			i2c_smbus_read_i2c_block_data(i2c_client, 0xB0, 1, &p_state);
            TPD_PROXIMITY_DEBUG("proxi_fts 0xB0 state value is 1131 0x%02X\n", p_state);
			if(!(p_state&0x01))
			{
				tpd_enable_ps(1);
			}
			i2c_smbus_read_i2c_block_data(i2c_client, 0x01, 1, &proximity_status);
            TPD_PROXIMITY_DEBUG("proxi_fts 0x01 value is 1139 0x%02X\n", proximity_status);
			if (proximity_status == 0xC0)
			{
				tpd_proximity_detect = 0;	
			}
			else if(proximity_status == 0xE0)
			{
				tpd_proximity_detect = 1;
			}
			else
			{
                          tpd_proximity_detect = -1;
			}

			TPD_PROXIMITY_DEBUG("tpd_proximity_detect 1149 = %d\n", tpd_proximity_detect);
			if ((-1==tpd_proximity_detect))
			{
				TPD_PROXIMITY_DMESG("proxi_fts read ps data error!\n");	
			}

			if ((err = ps_report_interrupt_data(tpd_proximity_detect)))
			{
				TPD_PROXIMITY_DMESG(" proxi_5206 call ps_report_interrupt_data failed= %d\n", err);	
		      }
		}  

		#endif
                                
		#ifdef MT_PROTOCOL_B
		{
            ret = fts_read_Touchdata(&pevent);
			if (ret == 0)
				fts_report_value(&pevent);
		}
		#else
		{
		    #if 1
			if (tpd_touchinfo(&cinfo, &pinfo,&ptest)) 
			{
		    	//printk("tpd point_num = %d\n",point_num);
			    TPD_DEBUG_SET_TIME;
			    if(point_num >0)
			    {
					/*
					for(i =0; i<point_num; i++)//only support 3 point
					{
						tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
					}
					input_sync(tpd->dev);
					*/
					for(i = 0; i < fts_updateinfo_curr.TPD_MAX_POINTS; i++)  
					{
						if((0==cinfo.p[i]) || (2==cinfo.p[i]))
						{
				       		tpd_down(cinfo.x[i], cinfo.y[i], cinfo.id[i]);
						}
						
					}
				    	input_sync(tpd->dev);
				}
				else  
	    		{
	              		tpd_up(cinfo.x[0], cinfo.y[0],&cinfo.id[0]);
        	    //TPD_DEBUG("release --->\n");
        	    input_sync(tpd->dev);
        		}
        	}
			#else
			if (tpd_touchinfo(&cinfo, &pinfo))
			{

				if(!cinfo.count)
				{
				    for(i =0; i<pinfo.count; i++)
    			    {
    			        tpd_up(pinfo.x[i], pinfo.y[i], pinfo.p[i]);
    			    }
				}
				else
				{

    			    for(i =0; i<cinfo.count; i++)
    			    {
    			        tpd_down(cinfo.x[i], cinfo.y[i], cinfo.p[i]);
    			    }

    				for(i =0; i<pinfo.count; i++)
    			    {
    			        tpd_key_up(pinfo.x[i], pinfo.y[i], pinfo.p[i]);
    			    }

				}

				input_sync(tpd->dev);
			}
			#endif
		}
		#endif
 }while(!kthread_should_stop());
	 return 0;
 }
 
void fts_reset_tp(int HighOrLow)
{
	if(HighOrLow)
	{

		  tpd_gpio_output(GTP_RST_PORT,1);
	}
	else
	{

		  tpd_gpio_output(GTP_RST_PORT,0);
	}
}
 static int tpd_detect (struct i2c_client *client, struct i2c_board_info *info) 
 {
	 strcpy(info->type, TPD_DEVICE);	
	  return 0;
 }
 
 static irqreturn_t tpd_eint_interrupt_handler(int irq, void *dev_id)
 {
	 TPD_DEBUG("TPD interrupt has been triggered\n");
	 TPD_DEBUG_PRINT_INT;
	 tpd_flag = 1;
	 wake_up_interruptible(&waiter);
	 return IRQ_HANDLED;
 }
#if 0
 static int fts_init_gpio_hw(void)
{

	int ret = 0;
	int i = 0;
       tpd_gpio_as_int(GTP_INT_PORT)
	//mt_set_gpio_mode(GPIO_CTP_EN_PIN, GPIO_CTP_EN_PIN_M_GPIO);
	//mt_set_gpio_dir(GPIO_CTP_EN_PIN, GPIO_DIR_OUT);
	//mt_set_gpio_out(GPIO_CTP_EN_PIN, GPIO_OUT_ZERO);

	return ret;
}
#endif
 static int  tpd_probe(struct i2c_client *client, const struct i2c_device_id *id)
 {	 
	int retval = TPD_OK;
	//char data;
	//u8 report_rate=0;
	//int err=0;
	//int reset_count = 0;
	   s32 nRetVal = 0;
       u8 uc_reg_value=0;
       u8 uc_reg_addr=0;

//reset_proc:   

    
	

#ifdef TPD_CLOSE_POWER_IN_SLEEP	
    tpd_gpio_output(GTP_RST_PORT,0);
    msleep(20);

#ifdef TPD_POWER_SOURCE_CUSTOM
	if(tpd->reg==NULL)
		{
		printk(" TPD DrvPlatformLyrTouchDeviceRegulatorPowerOn tpd->reg==NULL!\n");
		}
		printk(" TPD DrvPlatformLyrTouchDeviceRegulatorPowerOn regulator_set_voltage!\n");
		   nRetVal = regulator_set_voltage(tpd->reg, 2800000, 2800000); // For specific SPRD BB chip(ex. SC7715) or QCOM BB chip(ex. MSM8610), need to enable this function call for correctly power on Touch IC.
		if (nRetVal)
		{
			DBG("Could not set to 2800mv.\n");
		}

	nRetVal = regulator_disable(tpd->reg);
			if(nRetVal)
			   printk("Could not set regulator_disable to 2800mv.\n");
#else
    hwPowerDown(TPD_POWER_SOURCE,"TP");
#endif
#endif

		//power on, need confirm with SA
#ifdef TPD_POWER_SOURCE_CUSTOM
    printk(" TPD DrvPlatformLyrTouchDeviceRegulatorPowerOn regulator_set_voltage ok !\n");
    nRetVal =regulator_enable(tpd->reg);

      if (nRetVal)
    {
        DBG("Could not set regulator_enable to 2800mv.\n");
    }
	 //mdelay(20);
#else
//	hwPowerOn(MT65XX_POWER_LDO_VGP2, VOL_2800, "TP");
    hwPowerOn(TPD_POWER_SOURCE_CUSTOM, VOL_2800, "TP");
#endif

#ifdef TPD_POWER_SOURCE_1800
	hwPowerOn(TPD_POWER_SOURCE_1800, VOL_1800, "TP");
#endif 

    msleep(100);

#ifndef TPD_CLOSE_POWER_IN_SLEEP	
     tpd_gpio_output(GTP_RST_PORT,0);
	msleep(20);
#endif

    tpd_gpio_output(GTP_RST_PORT,1);
	msleep(50);
	TPD_DMESG(" fts reset\n");


	msg_dma_alloct();
	i2c_client = client;
	TPD_DMESG(" fts begin to write\n");
	mdelay(200);

       uc_reg_addr = FTS_REG_CHIP_ID;
	//fts_i2c_Write(i2c_client, &uc_reg_addr, 1);
	retval=fts_i2c_Read(i2c_client, &uc_reg_addr, 1, &uc_reg_value, 1);
	//retval=i2c_smbus_read_i2c_block_data(i2c_client, 0x00, 1,&uc_reg_value);
	TPD_DMESG("mtk_tpd[FTS] chip id is %x.\n",uc_reg_value);	
     if(retval<0)
    {
        TPD_DMESG("mtk_tpd[FTS] Read I2C error! driver NOt load!! CTP chip id is %d.\n",uc_reg_value);
	//return 0;
	return -1; 
	}

	tpd_load_status = 1;
		TPD_DMESG(" fts is right\n");
#ifdef TPD_PROXIMITY
	{
	       ps_enable_nodata=tpd_enable_ps;
	       ps_get_data=tpd_get_ps_value;

	}
#endif

       tpd_gpio_as_int(GTP_INT_PORT);
	msleep(150);

  //  mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);
  tpd_irq_registration();

#if 1
       uc_reg_addr = FTS_REG_POINT_RATE;				
	//fts_i2c_Write(i2c_client, &uc_reg_addr, 1);
	fts_i2c_Read(i2c_client, &uc_reg_addr, 1, &uc_reg_value, 1);
	printk("mtk_tpd[FTS] report rate is %dKHz.\n",uc_reg_value);

	uc_reg_addr = FTS_REG_FW_VER;
	//fts_i2c_Write(i2c_client, &uc_reg_addr, 1);
	fts_i2c_Read(i2c_client, &uc_reg_addr, 1, &uc_reg_value, 1);
	printk("mtk_tpd[FTS] Firmware version = 0x%x\n", uc_reg_value);

#if defined (CONFIG_TCT_HW_VERNO)
    	if (g_tct_tp_fw_verno == NULL)
    	{
        	g_tct_tp_fw_verno = kzalloc(sizeof(u8)*6, GFP_KERNEL);
    	}
    	sprintf(g_tct_tp_fw_verno, "%x", uc_reg_value);
#endif
#endif
	
    #ifdef VELOCITY_CUSTOM_fts
	if((err = misc_register(&tpd_misc_device)))
	{
		printk("mtk_tpd: tpd_misc_device register failed\n");
		
	}
	#endif

	thread = kthread_run(touch_event_handler, 0, TPD_DEVICE);
	 if (IS_ERR(thread))
		 { 
		  retval = PTR_ERR(thread);
		  TPD_DMESG(TPD_DEVICE " failed to create kernel thread: %d\n", retval);
		}


	focaltech_get_upgrade_array();
#ifdef SYSFS_DEBUG
                fts_create_sysfs(i2c_client);
#endif
#ifdef FTS_CTL_IIC
		 if (ft_rw_iic_drv_init(i2c_client) < 0)
			 dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n",
					 __func__);
#endif
	 
#ifdef FTS_APK_DEBUG
	fts_create_apk_debug_channel(i2c_client);
#endif

#ifdef TPD_AUTO_UPGRADE
	printk("********************Enter CTP Auto Upgrade********************\n");
	fts_ctpm_auto_upgrade(i2c_client);
#endif
	 
#ifdef FTS_GESTRUE
	init_para(480,854,60,0,0);
 
	input_set_capability(tpd->dev, EV_KEY, KEY_POWER);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_U); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_UP); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_DOWN);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_LEFT); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_RIGHT); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_E); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_M); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_S); 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_Z);
		
	__set_bit(KEY_GESTURE_RIGHT, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_LEFT, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_UP, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_DOWN, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_U, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_O, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_E, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_M, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_W, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_L, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_S, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_V, tpd->dev->keybit);
	__set_bit(KEY_GESTURE_Z, tpd->dev->keybit);
	#endif

#ifdef MT_PROTOCOL_B
	input_mt_init_slots(tpd->dev, MT_MAX_TOUCH_POINTS,INPUT_MT_DIRECT);
	input_set_abs_params(tpd->dev, ABS_MT_TOUCH_MAJOR,0, 255, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_X, 0, TPD_RES_X, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_POSITION_Y, 0, TPD_RES_Y, 0, 0);
	input_set_abs_params(tpd->dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
#endif
	
   printk("fts Touch Panel Device Probe %s\n", (retval < TPD_OK) ? "FAIL" : "PASS");
   return 0;
   
 }

 static int  tpd_remove(struct i2c_client *client)
 
 {

     #ifdef FTS_CTL_IIC
     	ft_rw_iic_drv_exit();
     #endif
     #ifdef SYSFS_DEBUG
     	fts_release_sysfs(client);
     #endif
     #ifdef FTS_APK_DEBUG
     	fts_release_apk_debug_channel();
     #endif
     msg_dma_release();
	 TPD_DEBUG("TPD removed\n");
 
   return 0;
 }
 
 static int tpd_local_init(void)
 {
  TPD_DMESG("Focaltech fts I2C Touchscreen Driver \n");
  #if 0
  printk(" TPD FT6436 tpd_local_init regulator_get !\n");
	tpd->reg=regulator_get(tpd->tpd_dev,"vgp1"); // get pointer to regulator structure
	printk(" TPD FT6436 tpd_local_init regulator_get ok!\n");
	if (IS_ERR(tpd->reg)) 
	printk("regulator_get() failed!\n");
  #endif

	tpd->reg = regulator_get(tpd->tpd_dev, "vtouch");
  
   if(i2c_add_driver(&tpd_i2c_driver)!=0)
   	{
        printk("fts unable to add i2c driver.\n");
      	return -1;
    }
    if(tpd_load_status == 0) 
    {
        printk("fts add error touch panel driver.\n");
      // regulator_disable(tpd->reg);
    	i2c_del_driver(&tpd_i2c_driver);
    	return -1;
    }
	
//#ifdef TPD_HAVE_BUTTON     
 //   tpd_button_setting(TPD_KEY_COUNT, tpd_keys_local, tpd_keys_dim_local);// initialize tpd button data
//#endif   
	//printk("tpd tpd_local_init use_tpd_button=%d,tpd_key_num=%d\n",tpd_dts_data.use_tpd_button,tpd_dts_data.tpd_key_num);
    //printk("tpd tpd_local_init tpd_key_local=%d,%d,%d,%d\n",tpd_dts_data.tpd_key_local[0],tpd_dts_data.tpd_key_local[1],tpd_dts_data.tpd_key_local[2],tpd_dts_data.tpd_key_local[3]);
	//printk("tpd tpd_local_init tpd_key_dim_local=%d,%d,%d,%d\n",tpd_dts_data.tpd_key_local[0],tpd_dts_data.tpd_key_local[1],tpd_dts_data.tpd_key_local[2],tpd_dts_data.tpd_key_local[3]);
	
  	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
		tpd_dts_data.tpd_key_dim_local);
	}
#if (defined(TPD_WARP_START) && defined(TPD_WARP_END))    
    TPD_DO_WARP = 1;
    memcpy(tpd_wb_start, tpd_wb_start_local, TPD_WARP_CNT*4);
    memcpy(tpd_wb_end, tpd_wb_start_local, TPD_WARP_CNT*4);
#endif 

#if (defined(TPD_HAVE_CALIBRATION) && !defined(TPD_CUSTOM_CALIBRATION))
    memcpy(tpd_calmat, tpd_def_calmat_local, 8*4);
    memcpy(tpd_def_calmat, tpd_def_calmat_local, 8*4);	
#endif  
	TPD_DMESG("end %s, %d\n", __FUNCTION__, __LINE__);  
	tpd_type_cap = 1;
    return 0; 
 }

 static void tpd_resume( struct device *h )
 {
	int ret = 0;
 TPD_DMESG("TPD wake up\n");
  	#ifdef TPD_PROXIMITY	
		if (tpd_proximity_flag == 1)
		{
			if(tpd_proximity_flag_one == 1)
			{
				tpd_proximity_flag_one = 0;	
				TPD_DMESG(TPD_DEVICE " tpd_proximity_flag_one \n"); 
				return;
			}
		}
	#endif	

 	#ifdef FTS_GESTRUE
    fts_write_reg(i2c_client,0xD0,0x00);
	#endif
#ifdef TPD_CLOSE_POWER_IN_SLEEP	
	//hwPowerOn(TPD_POWER_SOURCE,VOL_3300,"TP");

	tpd_gpio_output(GTP_RST_PORT,0);
	msleep(1); 

    ret = regulator_enable(tpd->reg);		
	if(ret)		
		printk("Could not set regulator_enable to 2800mv.\n");	

      msleep(20);  
	  tpd_gpio_output(GTP_RST_PORT,1);
#else
	  tpd_gpio_output(GTP_RST_PORT,0);
    msleep(1);  
	  tpd_gpio_output(GTP_RST_PORT,1);
#endif
	//mt_eint_unmask(CUST_EINT_TOUCH_PANEL_NUM);  
    
	msleep(30);
	tpd_halt = 0;

	#ifdef USB_CHARGE_DETECT    
	msleep(30);    
	tpd_usb_plugin(b_usb_plugin);
	#endif
	TPD_DMESG("TPD wake up done\n");

 }

 static void tpd_suspend( struct device *h )
 {
	 //static char data = 0x3;
	int ret = 0;
	 TPD_DMESG("TPD enter sleep\n");
	#ifdef TPD_PROXIMITY
	if (tpd_proximity_flag == 1)
	{
		tpd_proximity_flag_one = 1;	
		return;
	}
	#endif

	#ifdef FTS_GESTRUE
        	fts_write_reg(i2c_client, 0xd0, 0x01);
		  if (fts_updateinfo_curr.CHIP_ID==0x54)
		  {
		  	fts_write_reg(i2c_client, 0xd1, 0xff);
			fts_write_reg(i2c_client, 0xd2, 0xff);
			fts_write_reg(i2c_client, 0xd5, 0xff);
			fts_write_reg(i2c_client, 0xd6, 0xff);
			fts_write_reg(i2c_client, 0xd7, 0xff);
			fts_write_reg(i2c_client, 0xd8, 0xff);
		  }
        return;
	#endif
 	 tpd_halt = 1;

	// mt_eint_mask(CUST_EINT_TOUCH_PANEL_NUM);
	 mutex_lock(&i2c_access);
#ifdef TPD_CLOSE_POWER_IN_SLEEP	

	tpd_gpio_output(GTP_RST_PORT,0);
	msleep(5); 

	//hwPowerDown(TPD_POWER_SOURCE,"TP");
    ret = regulator_disable(tpd->reg);
        if(ret)
	       printk("Could not set regulator_disable to 2800mv.\n");
#else
	i2c_smbus_write_i2c_block_data(i2c_client, 0xA5, 1, &data);  //TP enter sleep mode
     
#endif
	mutex_unlock(&i2c_access);
    TPD_DMESG("TPD enter sleep done\n");

 } 


 static struct tpd_driver_t tpd_device_driver = {
        	//old  .tpd_device_name = "fts",
        	.tpd_device_name = "ft5436",    //fangjie modify for MinisW TP Rawdata test 
		 .tpd_local_init = tpd_local_init,
		 .suspend = tpd_suspend,
		 .resume = tpd_resume,
#ifdef TPD_HAVE_BUTTON
		 .tpd_have_button = 1,
#else
       	 .tpd_have_button = 0,
#endif		
 };
 /* called when loaded into kernel */
 static int __init tpd_driver_init(void) {
                printk("MediaTek fts touch panel driver init\n");
		    tpd_get_dts_info();
		 if(tpd_driver_add(&tpd_device_driver) < 0)
                 TPD_DMESG("add fts driver failed\n");
	 return 0;
 }
 
 /* should never be called */
 static void __exit tpd_driver_exit(void) {
                TPD_DMESG("MediaTek fts touch panel driver exit\n");
	 //input_unregister_device(tpd->dev);
	 tpd_driver_remove(&tpd_device_driver);
 }
 
 module_init(tpd_driver_init);
 module_exit(tpd_driver_exit);


