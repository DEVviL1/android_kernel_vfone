/* BEGIN PN:DTS2013053103858 , Added by d00238048, 2013.05.31*/
#ifndef BUILD_LK
    #include <linux/string.h>
    #include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
    #include <platform/mt_gpio.h>	
#elif defined(BUILD_UBOOT)
    #include <asm/arch/mt_gpio.h>
    #include <platform/mt_pmic.h>
#else
    #include <mt-plat/mt_gpio.h>
    #include <mach/gpio_const.h>
#endif
#ifdef BUILD_LK
#define LCD_DEBUG(fmt, args...) printf(fmt, ##args)		
#else
#define LCD_DEBUG(fmt, args...) printk(fmt, ##args)
#endif

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/***************************************************************************** 
 * Define
 *****************************************************************************/
#ifndef CONFIG_FPGA_EARLY_PORTING
#define TPS_I2C_BUSNUM  0//for I2C channel 0
#define I2C_ID_NAME "tps65132"
#define TPS_ADDR 0x3E
/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata tps65132_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};
/*add by jane for LCM resume crash */
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
		{ .compatible = "mediatek,i2c_lcd_bias" },
		{},
};
#endif
static struct i2c_client *tps65132_i2c_client = NULL;


/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

 struct tps65132_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id tps65132_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif
/*add by jane for LCM resume crash */
static struct i2c_driver tps65132_iic_driver = {
	.id_table	= tps65132_id,
	.probe		= tps65132_probe,
	.remove		= tps65132_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tps65132",
#if !defined(CONFIG_MTK_LEGACY)
		.of_match_table = lcm_of_match,
#endif
	},
 
};
/***************************************************************************** 
 * Extern Area
 *****************************************************************************/ 
 
 

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	LCD_DEBUG( "*********hx8395a tps65132_iic_probe\n");
	LCD_DEBUG("*********hx8395a TPS: info==>name=%s addr=0x%x\n",client->name,client->addr);
	tps65132_i2c_client  = client;		
	return 0;      
}


static int tps65132_remove(struct i2c_client *client)
{  	
	LCD_DEBUG( "*********hx8395a tps65132_remove\n");
	tps65132_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}


 static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    	ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
	LCD_DEBUG("*********hx8395a tps65132 write data fail !!\n");	
	return ret ;
}



/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{

   LCD_DEBUG( "*********hx8395a tps65132_iic_init\n");
   i2c_register_board_info(TPS_I2C_BUSNUM, &tps65132_board_info, 1);
   LCD_DEBUG( "*********hx8395a tps65132_iic_init2\n");
   i2c_add_driver(&tps65132_iic_driver);
   LCD_DEBUG( "*********hx8395a tps65132_iic_init success\n");	
   return 0;
}

static void __exit tps65132_iic_exit(void)
{
  LCD_DEBUG( "*********hx8395a tps65132_iic_exit\n");
  i2c_del_driver(&tps65132_iic_driver);  
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Longfang.liu");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL"); 
#endif
#endif

#ifndef CONFIG_FPGA_EARLY_PORTING
#ifdef BUILD_LK

#define TPS65132_SLAVE_ADDR_WRITE  0x7C  //(0X3E<<1==0X7C)
#include <platform/mt_i2c.h>

static struct mt_i2c_t TPS65132_i2c;

static int tps65132_write_byte_lk(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    TPS65132_i2c.id = 0;//I2C0;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    TPS65132_i2c.addr = (TPS65132_SLAVE_ADDR_WRITE >> 1);
    TPS65132_i2c.mode = ST_MODE;
    TPS65132_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&TPS65132_i2c, write_data, len);
    //LCD_DEBUG("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

#endif
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define FRAME_WIDTH  										(1080)
#define FRAME_HEIGHT 										(1920)

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#define LCD_VCC_EN        			(GPIO13 | 0x80000000)
#define LCD_VCC_EP        			(GPIO12 | 0x80000000)
#define GPIO_LCM_ID1				(GPIO17 | 0x80000000)
#define GPIO_LCM_ID2				(GPIO19 | 0x80000000)

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
/*add by jane for LCM resume crash */
#define SET_RESET_PIN(v)    				(lcm_util.set_reset_pin((v)))
#define MDELAY(n)					(lcm_util.mdelay(n))
#define UDELAY(n)					(lcm_util.udelay(n))
#define dsi_set_cmdq_V3(para_tbl,size,force_update)        lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)										lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)   			lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)   
#define set_gpio_lcd_enp(cmd) \
	lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enn(cmd) \
	lcm_util.set_gpio_lcd_enn_bias(cmd)

struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};
//update initial param for IC hx8395ad 0.01
//static struct LCM_setting_table  lcm_initialization_setting_ty[] = {
static void init_lcm_registers(void)
{
	unsigned int data_array[16];

data_array[0] = 0x00043902;

data_array[1] = 0x9583ffb9;

dsi_set_cmdq(data_array, 2, 1);

 



data_array[0] = 0x00053902;

data_array[1] = 0xa80373ba;

data_array[2] = 0x0000007b;

dsi_set_cmdq(data_array, 3, 1);

 



data_array[0] = 0x00073902;

data_array[1] = 0x0cb400b2;

data_array[2] = 0x00212a0a;

dsi_set_cmdq(data_array, 3, 1);

 



data_array[0] = 0x000b3902;

data_array[1] = 0x11116cb1;

data_array[2] = 0xf1110424;

data_array[3] = 0x00e9e380;

dsi_set_cmdq(data_array, 4, 1);

 



data_array[0]= 0x00023902;//10-20 ver.C update
	
data_array[1]= 0x000088D2;
	
dsi_set_cmdq(data_array, 2, 1);

 



data_array[0] = 0x00173902;

data_array[1] = 0x0eff00b4;

data_array[2] = 0x0e4a0e4a;

data_array[3] = 0x014c014a;

data_array[4] = 0x013a014c;

data_array[5] = 0x013a013a;

data_array[6] = 0x004a014a;

dsi_set_cmdq(data_array, 7, 1);

 



data_array[0] = 0x00383902;

data_array[1] = 0x000100d3;

data_array[2] = 0x00100000;

data_array[3] = 0x00011032;

data_array[4] = 0xc0133201;

data_array[5] = 0x10320000;

data_array[6] = 0x37000008;

data_array[7] = 0x37030304;

data_array[8] = 0x0c470004;

data_array[9] = 0x0a000008;

data_array[10] = 0x15010100;

data_array[11] = 0x00000000;

data_array[12] = 0x00000000;

data_array[13] = 0x00000000;

data_array[14] = 0x15010000;

dsi_set_cmdq(data_array, 15, 1);

 



data_array[0] = 0x002d3902;

data_array[1] = 0x183939d5;

data_array[2] = 0x03000118;

data_array[3] = 0x07040502;

data_array[4] = 0x18181806;

data_array[5] = 0x38181818;

data_array[6] = 0x21191938;

data_array[7] = 0x18222320;

data_array[8] = 0x18181818;

data_array[9] = 0x18181818;

data_array[10] = 0x18181818;

data_array[11] = 0x18181818;

data_array[12] = 0x00000018;

dsi_set_cmdq(data_array, 13, 1);

 



data_array[0] = 0x002d3902;

data_array[1] = 0x193939d6;

data_array[2] = 0x04070619;

data_array[3] = 0x00030205;

data_array[4] = 0x18181801;

data_array[5] = 0x38181818;

data_array[6] = 0x22181838;

data_array[7] = 0x58212023;

data_array[8] = 0x58585858;

data_array[9] = 0x58585858;

data_array[10] = 0x58585858;

data_array[11] = 0x58585858;

data_array[12] = 0x00000058;

dsi_set_cmdq(data_array, 13, 1);

 



data_array[0] = 0x00033902;

data_array[1] = 0x006a6ab6;

dsi_set_cmdq(data_array, 2, 1);

 



data_array[0] = 0x002b3902;

data_array[1] = 0x0d0800e0;

data_array[2] = 0x1c3f322e;

data_array[3] = 0x0c0a063c;

data_array[4] = 0x15130e17;

data_array[5] = 0x11071514;

data_array[6] = 0x08001812;

data_array[7] = 0x3f312e0c;

data_array[8] = 0x0a073d1b;

data_array[9] = 0x130f170d;

data_array[10] = 0x06131315;

data_array[11] = 0x00171210;

dsi_set_cmdq(data_array, 12, 1);

 



data_array[0] = 0x00033902;

data_array[1] = 0x001530c0;

dsi_set_cmdq(data_array, 2, 1);

 

data_array[0]= 0x00023902;
	
data_array[1]= 0x000005CC;	//08
	
dsi_set_cmdq(data_array, 2, 1);

 

data_array[0]= 0x00023902;
	
data_array[1]= 0x000000BD;

dsi_set_cmdq(data_array, 2, 1);

 



data_array[0]= 0x00023902;
	
data_array[1]= 0x000001BD;
	
dsi_set_cmdq(data_array, 2, 1);

 



data_array[0] = 0x00043902;

data_array[1] = 0x010002ef;

dsi_set_cmdq(data_array, 2, 1);

 



data_array[0]= 0x00023902;
	
data_array[1]= 0x000000BD;

dsi_set_cmdq(data_array, 2, 1);

 



data_array[0]= 0x00023902;
	
data_array[1]= 0x00000035;

dsi_set_cmdq(data_array, 2, 1);

 



data_array[0] = 0x00073902;

data_array[1] = 0xf700b1ef;

data_array[2] = 0x00022f37;

dsi_set_cmdq(data_array, 3, 1);

 



data_array[0] = 0x00033902;

data_array[1] = 0x0071b2ef;

dsi_set_cmdq(data_array, 2, 1);

 



data_array[0] = 0x000d3902;

data_array[1] = 0x0000b5ef;

data_array[2] = 0xff000000;

data_array[3] = 0x00000003;

data_array[4] = 0x00000000;

dsi_set_cmdq(data_array, 5, 1);

 



data_array[0] = 0x00163902;

data_array[1] = 0x0040b6ef;

data_array[2] = 0x01000080;

data_array[3] = 0x38040080;

data_array[4] = 0x20200004;

data_array[5] = 0x00043804;

data_array[6] = 0x00002020;

dsi_set_cmdq(data_array, 7, 1);

 



data_array[0] = 0x000a3902;

data_array[1] = 0xa000b7ef;

data_array[2] = 0x00000400;

data_array[3] = 0x00000800;

dsi_set_cmdq(data_array, 4, 1);

 



data_array[0] = 0x00303902;

data_array[1] = 0x0000c0ef;

data_array[2] = 0x60026002;

data_array[3] = 0xc005c004;

data_array[4] = 0x800b8008;

data_array[5] = 0x00160010;

data_array[6] = 0x002d0020;

data_array[7] = 0x005a0040;

data_array[8] = 0x00b40080;

data_array[9] = 0x01680100;

data_array[10] = 0x02d10201;

data_array[11] = 0x88888888;

data_array[12] = 0xffffffff;

dsi_set_cmdq(data_array, 13, 1);

 



data_array[0] = 0x00303902;

data_array[1] = 0x0000c1ef;

data_array[2] = 0x60026002;

data_array[3] = 0xc005c004;

data_array[4] = 0x800b8008;

data_array[5] = 0x00160010;

data_array[6] = 0x002d0020;

data_array[7] = 0x005a0040;

data_array[8] = 0x00b40080;

data_array[9] = 0x01680100;

data_array[10] = 0x02d10201;

data_array[11] = 0x88888888;

data_array[12] = 0xffffffff;

dsi_set_cmdq(data_array, 13, 1);

 



data_array[0] = 0x00303902;

data_array[1] = 0x0001c2ef;

data_array[2] = 0x60996098;

data_array[3] = 0xc132c031;

data_array[4] = 0x82648062;

data_array[5] = 0x04c807c5;

data_array[6] = 0x09910d8b;

data_array[7] = 0x13231a16;

data_array[8] = 0x2647282c;

data_array[9] = 0x4c8e5f58;

data_array[10] = 0x991c80b1;

data_array[11] = 0x88888888;

data_array[12] = 0xffffffff;

dsi_set_cmdq(data_array, 13, 1);

 



data_array[0] = 0x00303902;

data_array[1] = 0x0001c3ef;

data_array[2] = 0x60996098;

data_array[3] = 0xc132c331;

data_array[4] = 0x82648662;

data_array[5] = 0x04c802c5;

data_array[6] = 0x0991018b;

data_array[7] = 0x13231f16;

data_array[8] = 0x2647362c;

data_array[9] = 0x4c8e5858;

data_array[10] = 0x991c80b1;

data_array[11] = 0x88888888;

data_array[12] = 0xffffffff;

dsi_set_cmdq(data_array, 13, 1);

 



data_array[0] = 0x00243902;

data_array[1] = 0x0002c4ef;

data_array[2] = 0x8e888e89;

data_array[3] = 0x1d101d12;

data_array[4] = 0x3a213a25;

data_array[5] = 0x7443744b;

data_array[6] = 0xe886e897;

data_array[7] = 0xd10dd12e;

data_array[8] = 0x88888888;

data_array[9] = 0xffffffff;

dsi_set_cmdq(data_array, 10, 1);

 



data_array[0] = 0x00283902;

data_array[1] = 0x0002c5ef;

data_array[2] = 0x8e888e89;

data_array[3] = 0x1d101d12;

data_array[4] = 0x3a213a25;

data_array[5] = 0x7443744b;

data_array[6] = 0xe886e897;

data_array[7] = 0xd10dd12e;

data_array[8] = 0xa21ba25c;

data_array[9] = 0x88888888;

data_array[10] = 0xffffffff;

dsi_set_cmdq(data_array, 11, 1);

 



data_array[0] = 0x00343902;

data_array[1] = 0x0003c6ef;

data_array[2] = 0x62db62db;

data_array[3] = 0xc5b6c5b7;

data_array[4] = 0x8b6c8b6f;

data_array[5] = 0x16d916de;

data_array[6] = 0x2db32dbc;

data_array[7] = 0x5b665b79;

data_array[8] = 0xb6cdb6f2;

data_array[9] = 0x6d9a6de5;

data_array[10] = 0xdb34dbca;

data_array[11] = 0xb668b795;

data_array[12] = 0x88888888;

data_array[13] = 0xffffffff;

dsi_set_cmdq(data_array, 14, 1);

 



data_array[0] = 0x00303902;

data_array[1] = 0x0005c7ef;

data_array[2] = 0x5c315c31;

data_array[3] = 0xb863b862;

data_array[4] = 0x70c670c5;

data_array[5] = 0xe18ce18b;

data_array[6] = 0xc319c317;

data_array[7] = 0x8633862f;

data_array[8] = 0x4c764c5f;

data_array[9] = 0x58ad588e;

data_array[10] = 0x7266703c;

data_array[11] = 0x88888888;

data_array[12] = 0xffffffff;

dsi_set_cmdq(data_array, 13, 1);

 



data_array[0] = 0x00303902;

data_array[1] = 0x0000c8ef;

data_array[2] = 0x00000000;

data_array[3] = 0x00000000;

data_array[4] = 0x00000000;

data_array[5] = 0x00000000;

data_array[6] = 0x00000000;

data_array[7] = 0x00000000;

data_array[8] = 0x00000000;

data_array[9] = 0x00000000;

data_array[10] = 0x00000000;

data_array[11] = 0x88888888;

data_array[12] = 0xffffffff;

dsi_set_cmdq(data_array, 13, 1);

 



data_array[0] = 0x00303902;

data_array[1] = 0x0000c9ef;

data_array[2] = 0x00000000;

data_array[3] = 0x00000000;

data_array[4] = 0x00000000;

data_array[5] = 0x00000000;

data_array[6] = 0x00000000;

data_array[7] = 0x00000000;

data_array[8] = 0x00000000;

data_array[9] = 0x00000000;

data_array[10] = 0x00000000;

data_array[11] = 0x88888888;

data_array[12] = 0xffffffff;

dsi_set_cmdq(data_array, 13, 1);

 



data_array[0] = 0x00303902;

data_array[1] = 0x0000caef;

data_array[2] = 0x00000000;

data_array[3] = 0x00000000;

data_array[4] = 0x00000000;

data_array[5] = 0x00000000;

data_array[6] = 0x00000000;

data_array[7] = 0x00000000;

data_array[8] = 0x00000000;

data_array[9] = 0x00000000;

data_array[10] = 0x00000000;

data_array[11] = 0x88888888;

data_array[12] = 0xffffffff;

dsi_set_cmdq(data_array, 13, 1);

 



data_array[0] = 0x00303902;

data_array[1] = 0x0000cbef;

data_array[2] = 0x00000000;

data_array[3] = 0x00000000;

data_array[4] = 0x00000000;

data_array[5] = 0x00000000;

data_array[6] = 0x00000000;

data_array[7] = 0x00000000;

data_array[8] = 0x00000000;

data_array[9] = 0x00000000;

data_array[10] = 0x00000000;

data_array[11] = 0x88888888;

data_array[12] = 0xffffffff;

dsi_set_cmdq(data_array, 13, 1);

 



data_array[0] = 0x00183902;

data_array[1] = 0x0004ccef;

data_array[2] = 0x300d300c;

data_array[3] = 0x601a6018;

data_array[4] = 0xc035c031;

data_array[5] = 0x88888888;

data_array[6] = 0xffffffff;

dsi_set_cmdq(data_array, 7, 1);

 



data_array[0] = 0x000c3902;

data_array[1] = 0x0000d0ef;

data_array[2] = 0x4527160b;

data_array[3] = 0x7fbf6a5f;

dsi_set_cmdq(data_array, 4, 1);

 



data_array[0] = 0x000c3902;

data_array[1] = 0x0000d1ef;

data_array[2] = 0x4527534e;

data_array[3] = 0xd0b14894;

dsi_set_cmdq(data_array, 4, 1);

 



data_array[0] = 0x000b3902;

data_array[1] = 0x0000d2ef;

data_array[2] = 0x62315345;

data_array[3] = 0x00000000;

dsi_set_cmdq(data_array, 4, 1);

 



data_array[0] = 0x00073902;

data_array[1] = 0x3210d3ef;

data_array[2] = 0x00107654;

dsi_set_cmdq(data_array, 3, 1);

 



data_array[0] = 0x00063902;

data_array[1] = 0x3210d4ef;

data_array[2] = 0x00007654;

dsi_set_cmdq(data_array, 3, 1);

 



data_array[0] = 0x00053902;

data_array[1] = 0x0101d5ef;

data_array[2] = 0x00000001;

dsi_set_cmdq(data_array, 3, 1);

 



data_array[0] = 0x00033902;

data_array[1] = 0x0012d6ef;

dsi_set_cmdq(data_array, 2, 1);

 



data_array[0] = 0x00063902;

data_array[1] = 0xc14cdeef;

data_array[2] = 0x00008d52;

dsi_set_cmdq(data_array, 3, 1);

 



data_array[0] = 0x00143902;

data_array[1] = 0xab00dfef;

data_array[2] = 0x0a0b0c0d;

data_array[3] = 0x0a0b0c0d;

data_array[4] = 0x00ff0000;

data_array[5] = 0x0a0b0c0d;

dsi_set_cmdq(data_array, 6, 1);

 



data_array[0] = 0x00110500;

dsi_set_cmdq(data_array, 1, 1);

 MDELAY(120);



data_array[0] = 0x00290500;

dsi_set_cmdq(data_array, 1, 1);
MDELAY(20);
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
    memset(params, 0, sizeof(LCM_PARAMS));
    params->type   			= LCM_TYPE_DSI;
    params->width  			= FRAME_WIDTH;
    params->height 			= FRAME_HEIGHT;

    params->dsi.mode   			= BURST_VDO_MODE;

    // DSI
    /* Command mode setting */
    params->dsi.LANE_NUM		= LCM_FOUR_LANE;
    //The following defined the fomat for data coming from LCD engine.
    params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
    params->dsi.data_format.trans_seq   = LCM_DSI_TRANS_SEQ_MSB_FIRST;
    params->dsi.data_format.padding     = LCM_DSI_PADDING_ON_LSB;
    params->dsi.data_format.format      = LCM_DSI_FORMAT_RGB888;

   // Highly depends on LCD driver capability.
   //video mode timing

    params->dsi.PS			= LCM_PACKED_PS_24BIT_RGB888;
    params->dsi.HS_TRAIL= 8;

    params->dsi.vertical_sync_active = 2;
    params->dsi.vertical_backporch = 12;
    params->dsi.vertical_frontporch = 18;
    params->dsi.vertical_active_line = FRAME_HEIGHT;

    params->dsi.horizontal_sync_active = 30;
    params->dsi.horizontal_backporch = 74;
    params->dsi.horizontal_frontporch = 74;
    params->dsi.horizontal_active_pixel = FRAME_WIDTH;

    //improve clk quality
    params->dsi.PLL_CLOCK = 442; //this value must be in MTK suggested table
    //params->dsi.compatibility_for_nvk = 1;
    params->dsi.ssc_disable = 1;

}

static void push_table(struct LCM_setting_table *table, unsigned int count,
		       unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

		switch (cmd) {

		case REGFLAG_DELAY_MS_V3:
			MDELAY(table[i].count);
			break;

		case REGFLAG_END_OF_TABLE:
			break;

		default:
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}

}
/*add by jane for LCM resume crash */
static void lcm_init(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x00;
	data=0x0E;
	//VPS=0x00;data=0x0A;VSP=5V,
	//         data=0x0E;VSP=5.4V,
	//VNG=0x01;data=0x0A;VNG=-5V,
	//         data=0x0E;VNG=-5.4V,
#ifndef CONFIG_FPGA_EARLY_PORTING
	//enable power
	//lcm_util.set_gpio_out(LCD_VCC_EP, GPIO_OUT_ZERO);
	//lcm_util.set_gpio_out(LCD_VCC_EN, GPIO_OUT_ZERO);
	set_gpio_lcd_enp(0);
	set_gpio_lcd_enn(0);
	MDELAY(50);

	//lcm_util.set_gpio_out(LCD_VCC_EP, GPIO_OUT_ONE);
	set_gpio_lcd_enp(1);
	MDELAY(10);
	//lcm_util.set_gpio_out(LCD_VCC_EN, GPIO_OUT_ONE);
	set_gpio_lcd_enn(1);
	MDELAY(10);

#ifdef BUILD_LK
	ret=tps65132_write_byte_lk(cmd,data);
	if(ret)    	
	LCD_DEBUG("[LK]LM3463-----tps65132----cmd=%0x--i2c write error----\n",cmd);    	
	else
	LCD_DEBUG("[LK]LM3463----tps65132----cmd=%0x--i2c write success----\n",cmd);    		
#else
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
	
	cmd=0x01;
	data=0x0E;
	//VPS=0x00;data=0x0A;VSP=5V,
	//         data=0x0E;VSP=5.4V,
	//VNG=0x01;data=0x0A;VNG=-5V,
	//         data=0x0E;VNG=-5.4V,
#ifdef BUILD_LK
	ret=tps65132_write_byte_lk(cmd,data);
    if(ret)    	
	LCD_DEBUG("[LK]LM3463-----tps65132----cmd=%0x--i2c write error----\n",cmd);    	
	else
	LCD_DEBUG("[LK]LM3463----tps65132----cmd=%0x--i2c write success----\n",cmd);   
#else
	ret=tps65132_write_bytes(cmd,data);
	if(ret<0)
	LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	LCD_DEBUG("[KERNEL]LM3463-----tps65132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
#endif
	//reset high to low to high
	SET_RESET_PIN(1);
	MDELAY(50);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(150);
	//push_table(lcm_initialization_setting_ty, sizeof(lcm_initialization_setting_ty) / sizeof(struct LCM_setting_table), 1);  
	init_lcm_registers();
	LCD_DEBUG("uboot:pop455_hx8395a_fhd_dsi_vdo_tdt_lcm_init\n");
}
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Sleep Mode On
	{0x28,1,{0x00}},
	{REGFLAG_DELAY_MS_V3,50,{0x00}},
	{0x10,1,{0x00}},
	{REGFLAG_DELAY_MS_V3,120,{0x00}},
	{REGFLAG_END_OF_TABLE,1,{0x00}},
};
/*add by jane for LCM resume crash */
static void lcm_suspend(void)
{
    push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);	
#if 0
    //reset low
    SET_RESET_PIN(0);
    MDELAY(20);
    //disable power
    mt_set_gpio_out(LCD_VCC_EN, GPIO_OUT_ONE);
    mt_set_gpio_out(LCD_VCC_EN, GPIO_OUT_ZERO);
    MDELAY(10);
    mt_set_gpio_out(LCD_VCC_EP, GPIO_OUT_ONE);
    mt_set_gpio_out(LCD_VCC_EP, GPIO_OUT_ZERO);
    MDELAY(50);	
#endif
    LCD_DEBUG("uboot:pop455_hx8395a_fhd_dsi_vdo_tdt_lcm_suspend\n");

}
/*add by jane for LCM resume crash */
static void lcm_resume(void)
{
#if 0
    //reset low to high
    SET_RESET_PIN(1);
    MDELAY(50);
    SET_RESET_PIN(0);
    MDELAY(50);
    SET_RESET_PIN(1);
    MDELAY(150);
    //enable power
    lcm_util.set_gpio_out(LCD_VCC_EP, GPIO_OUT_ZERO);
    lcm_util.set_gpio_out(LCD_VCC_EN, GPIO_OUT_ZERO);
    MDELAY(50);
    
    lcm_util.set_gpio_out(LCD_VCC_EP, GPIO_OUT_ONE);
    MDELAY(10);
    lcm_util.set_gpio_out(LCD_VCC_EN, GPIO_OUT_ONE);
    MDELAY(10);

    //push_table(lcm_initialization_setting_ty, sizeof(lcm_initialization_setting_ty) / sizeof(struct LCM_setting_table), 1);
    init_lcm_registers();
#endif
    //lcm_init();
	init_lcm_registers();
    LCD_DEBUG("uboot:pop455_hx8395a_fhd_dsi_vdo_tdt_lcm_resume\n");

}

static unsigned int lcm_compare_id(void)
{
    	int id_type=0;	

	mt_set_gpio_mode(GPIO_LCM_ID1,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_ID1, GPIO_DIR_IN);
	//mt_set_gpio_pull_select(GPIO_LCM_ID1,GPIO_PULL_DOWN);
	mt_set_gpio_pull_enable(GPIO_LCM_ID1, GPIO_PULL_DISABLE);// def 0
	
	mt_set_gpio_mode(GPIO_LCM_ID2,GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCM_ID2, GPIO_DIR_IN);
	//mt_set_gpio_pull_select(GPIO_LCM_ID2,GPIO_PULL_DOWN);
	mt_set_gpio_pull_enable(GPIO_LCM_ID2, GPIO_PULL_DISABLE);//def 0

	MDELAY(10);
	id_type = mt_get_gpio_in(GPIO_LCM_ID1)<<1 | mt_get_gpio_in(GPIO_LCM_ID2);
	LCD_DEBUG("[LLF_Debug] Tdt ID_type is: %d",id_type);
	if (id_type==3)
    		return 1;
	else 
		return 0; 
}
LCM_DRIVER pop455_hx8395a_fhd_dsi_vdo_tdt_lcm_drv =
{
    .name           = "pop455_hx8395a_fhd_dsi_vdo_tdt",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,/*kd init fun.*/
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
   
};
