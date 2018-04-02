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
/*add by zhuqiang for PR949448 */
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
/*add by zhuqiang for PR949448 */
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
/*add by zhuqiang for PR949448 */
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
#define set_gpio_lcd_enn(cmd) \
	lcm_util.set_gpio_lcd_enp_bias(cmd)
#define set_gpio_lcd_enp(cmd) \
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
	data_array[0]= 0x00043902;
	data_array[1]= 0x9583FFB9;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00053902;//10-20 ver.C update
	data_array[1]= 0xA80373BA;
	data_array[2]= 0x0000007B;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x000B3902;//10-20 ver.C update
	data_array[1]= 0x11116CB1;
	data_array[2]= 0xF1110424;//10-20 ver.C update
	data_array[3]= 0x00E9E380;
	dsi_set_cmdq(data_array, 4, 1);


  	data_array[0]= 0x00023902;//10-20 ver.C update
	data_array[1]= 0x000077D2;
	dsi_set_cmdq(data_array, 2, 1);

  	data_array[0]= 0x00043902;//10-20 ver.C update
	data_array[1]= 0x010E41BF;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00063902;//10-20 ver.C update
	data_array[1]= 0x0CB400B2;
	data_array[2]= 0x00002A0A;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x00383902;	//GIP;//10-20 ver.C update
	data_array[1]= 0x000000D3; 
	data_array[2]= 0x00100000;
	data_array[3]= 0x00011032;
	data_array[4]= 0xC0133201;
	data_array[5]= 0x10320000;
	data_array[6]= 0x37000008;
	data_array[7]= 0x37030304;
	data_array[8]= 0x0C470004;//10-20 ver.C update
	data_array[9]= 0x0A000008;//10-20 ver.C update
	data_array[10]= 0x15010300;//10-20 ver.C update
	data_array[11]= 0x00000000;
	data_array[12]= 0xC0030000;
	data_array[13]= 0x04020800;
	data_array[14]= 0x15010000;
	dsi_set_cmdq(data_array, 15, 1);


	data_array[0]= 0x00173902;//9.22 Update
	data_array[1]= 0x0CFF00B4;
	data_array[2]= 0x0c400c40;
	data_array[3]= 0x014C0140;
	data_array[4]= 0x013A014C; //10-24 ver.C update soff
	data_array[5]= 0x013A033A;
	data_array[6]= 0x004A014A;
	dsi_set_cmdq(data_array, 7, 1);


	data_array[0]= 0x002D3902;  // Forward
	data_array[1]= 0x183939D5;
	data_array[2]= 0x03000118;
	data_array[3]= 0x07040502;
	data_array[4]= 0x18181806;
	data_array[5]= 0x38181818;
	data_array[6]= 0x21191938;
	data_array[7]= 0x18222320;
	data_array[8]= 0x18181818;
	data_array[9]= 0x18181818;
	data_array[10]= 0x18181818;
	data_array[11]= 0x18181818;
	data_array[12]= 0x00000018;
	dsi_set_cmdq(data_array, 13, 1);

	data_array[0]= 0x002D3902;  // Backward
	data_array[1]= 0x193939D6;
	data_array[2]= 0x04070619;
	data_array[3]= 0x00030205;
	data_array[4]= 0x18181801;
	data_array[5]= 0x38181818;
	data_array[6]= 0x22181838;
	data_array[7]= 0x58212023;
	data_array[8]= 0x58585858;
	data_array[9]= 0x58585858;
	data_array[10]= 0x58585858;
	data_array[11]= 0x58585858;
	data_array[12]= 0x00000058;
	dsi_set_cmdq(data_array, 13, 1);
	
  	data_array[0]= 0x00023902;
	data_array[1]= 0x000005CC; //the panel direction// old is 0X08
	dsi_set_cmdq(data_array, 2, 1);

  	data_array[0]= 0x00033902;
	data_array[1]= 0x001530C0;
	dsi_set_cmdq(data_array, 2, 1);

	data_array[0]= 0x00063902;
	data_array[1]= 0x0C0800C7;
	data_array[2]= 0x0000D000;
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0]= 0x002B3902;
	data_array[1]= 0x0D0700E0;
	data_array[2]= 0x1B3F3830;
	data_array[3]= 0x0C0A063D;
	data_array[4]= 0x15130F18;
	data_array[5]= 0x12071514;
	data_array[6]= 0x07001713;
	data_array[7]= 0x3F38310D;
	data_array[8]= 0x0A063D1B;
	data_array[9]= 0x130F180C;
	data_array[10]= 0x07141315;
	data_array[11]= 0x00181312;
	dsi_set_cmdq(data_array, 12, 1);//GAMMA  

  //********************YYG*******************//
  	data_array[0]= 0x00023902;
	data_array[1]= 0x006E6EB6;
	dsi_set_cmdq(data_array, 2, 1);//Set VCOM
	
  	data_array[0]= 0x00023902;
	data_array[1]= 0x000001BD;
	dsi_set_cmdq(data_array, 2, 1);
	
	// Himax EF
  	data_array[0]= 0x00043902;
	data_array[1]= 0x000002EF;
	dsi_set_cmdq(data_array, 2, 1);

  	data_array[0]= 0x00023902;
	data_array[1]= 0x000001BD;
	dsi_set_cmdq(data_array, 2, 1);

  	data_array[0]= 0x00043902;
	data_array[1]= 0x010002EF;
	dsi_set_cmdq(data_array, 2, 1);

  	data_array[0]= 0x00023902;
	data_array[1]= 0x000000BD;
	dsi_set_cmdq(data_array, 2, 1);

  	data_array[0]= 0x00073902;
	data_array[1]= 0xF700B1EF;
	data_array[2]= 0x00022F37;
	dsi_set_cmdq(data_array, 3, 1);

  	data_array[0]= 0x00033902;
	data_array[1]= 0x0071B2EF;//the panel direction//OLD IS 0X60
	dsi_set_cmdq(data_array, 2, 1);

  	data_array[0]= 0x000E3902;
	data_array[1]= 0x0000B5EF;
  	data_array[2]= 0xFF000000;
	data_array[3]= 0x00000003;
	data_array[4]= 0x00000000;
	dsi_set_cmdq(data_array, 5, 1);

  	data_array[0]= 0x00163902;
	data_array[1]= 0x0040B6EF;
  	data_array[2]= 0x01000080;
	data_array[3]= 0x38040080;
	data_array[4]= 0x20200004;
	data_array[5]= 0x00043804;
	data_array[6]= 0x00002020;
	dsi_set_cmdq(data_array, 7, 1);

 	data_array[0]= 0x00093902;
	data_array[1]= 0xA000B7EF;
 	data_array[2]= 0x00000400;
	data_array[3]= 0x00000800;
	dsi_set_cmdq(data_array, 4, 1);

 	data_array[0]= 0x00303902;
	data_array[1]= 0x0000C0EF;
 	data_array[2]= 0x60026002;
	data_array[3]= 0xC005C004;
	data_array[4]= 0x800B8008;
	data_array[5]= 0x00160010;
	data_array[6]= 0x002D0020;
 	data_array[7]= 0x005A0040;
	data_array[8]= 0x00B40080;
 	data_array[9]= 0x01680100;
	data_array[10]= 0x02D10201;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0000C1EF;
  	data_array[2]= 0x60026002;
	data_array[3]= 0xC005C004;
	data_array[4]= 0x800B8008;
	data_array[5]= 0x00160010;
	data_array[6]= 0x002D0020;
  	data_array[7]= 0x005A0040;
	data_array[8]= 0x00B40080;
  	data_array[9]= 0x01680100;
	data_array[10]= 0x02D10201;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0001C2EF;
  	data_array[2]= 0x60996098;
	data_array[3]= 0xC132C031;
	data_array[4]= 0x82648062;
	data_array[5]= 0x04C807C5;
	data_array[6]= 0x09910D8B;
  	data_array[7]= 0x13231A16;
	data_array[8]= 0x2647282C;
  	data_array[9]= 0x4C8E5F58;
	data_array[10]= 0x991C80B1;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0001C3EF;
  	data_array[2]= 0x60996098;
	data_array[3]= 0xC132C331;
	data_array[4]= 0x82648662;
	data_array[5]= 0x04C802C5;
	data_array[6]= 0x0991018B;
  	data_array[7]= 0x13231F16;
	data_array[8]= 0x2647362C;
  	data_array[9]= 0x4C8E5858;
	data_array[10]= 0x991C80B1;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 13, 1);

  	data_array[0]= 0x00243902;
	data_array[1]= 0x0002C4EF;
  	data_array[2]= 0x8E888E89;
	data_array[3]= 0x1D101D12;
	data_array[4]= 0x3A213A25;
	data_array[5]= 0x7443744B;
	data_array[6]= 0xE886E897;
  	data_array[7]= 0xD10DD12E;
	data_array[8]= 0x88888888;
  	data_array[9]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 10, 1);

  	data_array[0]= 0x00283902;
	data_array[1]= 0x0002C5EF;
  	data_array[2]= 0x8E888E89;
	data_array[3]= 0x1D101D12;
	data_array[4]= 0x3A213A25;
	data_array[5]= 0x7443744B;
	data_array[6]= 0xE886E897;
  	data_array[7]= 0xD10DD12E;
	data_array[8]= 0xA21BA25C;
  	data_array[9]= 0x88888888;
	data_array[10]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 11, 1);

  	data_array[0]= 0x00343902;
	data_array[1]= 0x0003C6EF;
  	data_array[2]= 0x62DB62DB;
	data_array[3]= 0xC5B6C5B7;
	data_array[4]= 0x8B6C8B6F;
	data_array[5]= 0x16D916DE;
	data_array[6]= 0x2DB32DBC;
  	data_array[7]= 0x5B665B79;
	data_array[8]= 0xB6CDB6F2;
  	data_array[9]= 0x6D9A6DE5;
	data_array[10]= 0xDB34DBCA;
	data_array[11]= 0xB668B795;
	data_array[12]= 0x88888888;
	data_array[13]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 14, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0005C7EF;
  	data_array[2]= 0x5C315C31;
	data_array[3]= 0xB863B862;
	data_array[4]= 0x70C670C5;
	data_array[5]= 0xE18CE18B;
	data_array[6]= 0xC319C317;
  	data_array[7]= 0x8633862F;
	data_array[8]= 0x4C764C5F;
  	data_array[9]= 0x58AD588E;
	data_array[10]= 0x7266703C;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0000C8EF;
  	data_array[2]= 0x00000000;
	data_array[3]= 0x00000000;
	data_array[4]= 0x00000000;
	data_array[5]= 0x00000000;
	data_array[6]= 0x00000000;
  	data_array[7]= 0x00000000;
	data_array[8]= 0x00000000;
  	data_array[9]= 0x00000000;
	data_array[10]= 0x00000000;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0000C9EF;
  	data_array[2]= 0x00000000;
	data_array[3]= 0x00000000;
	data_array[4]= 0x00000000;
	data_array[5]= 0x00000000;
	data_array[6]= 0x00000000;
  	data_array[7]= 0x00000000;
	data_array[8]= 0x00000000;
  	data_array[9]= 0x00000000;
	data_array[10]= 0x00000000;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0000CAEF;
  	data_array[2]= 0x00000000;
	data_array[3]= 0x00000000;
	data_array[4]= 0x00000000;
	data_array[5]= 0x00000000;
	data_array[6]= 0x00000000;
  	data_array[7]= 0x00000000;
	data_array[8]= 0x00000000;
  	data_array[9]= 0x00000000;
	data_array[10]= 0x00000000;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 13, 1);

  	data_array[0]= 0x00303902;
	data_array[1]= 0x0000CBEF;
  	data_array[2]= 0x00000000;
	data_array[3]= 0x00000000;
	data_array[4]= 0x00000000;
	data_array[5]= 0x00000000;
	data_array[6]= 0x00000000;
  	data_array[7]= 0x00000000;
	data_array[8]= 0x00000000;
  	data_array[9]= 0x00000000;
	data_array[10]= 0x00000000;
	data_array[11]= 0x88888888;
	data_array[12]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 13, 1);

  	data_array[0]= 0x00183902;
	data_array[1]= 0x0004CCEF;
  	data_array[2]= 0x300D300C;
	data_array[3]= 0x601A6018;
	data_array[4]= 0xC035C031;
	data_array[5]= 0x88888888;
	data_array[6]= 0xFFFFFFFF;
	dsi_set_cmdq(data_array, 7, 1);

  	data_array[0]= 0x00073902;
	data_array[1]= 0x3210D3EF;
  	data_array[2]= 0x00107654;
	dsi_set_cmdq(data_array, 3, 1);

  	data_array[0]= 0x00063902;
	data_array[1]= 0x3210D4EF;
  	data_array[2]= 0x00007654;
	dsi_set_cmdq(data_array, 3, 1);

  	data_array[0]= 0x00053902;
	data_array[1]= 0x0101D5EF;
  	data_array[2]= 0x00000001;
	dsi_set_cmdq(data_array, 3, 1);

  	data_array[0]= 0x00033902;
	data_array[1]= 0x0012D6EF;
	dsi_set_cmdq(data_array, 2, 1);

  	data_array[0]= 0x00063902;
	data_array[1]= 0xC14CDEEF;
  	data_array[2]= 0x00008D52;
	dsi_set_cmdq(data_array, 3, 1);

  	data_array[0]= 0x00143902;
	data_array[1]= 0xAB00DFEF;
  	data_array[2]= 0x0A0B0C0D;
	data_array[3]= 0x0A0B0C0D;
	data_array[4]= 0x00FF0000;
	data_array[5]= 0x0A0B0C0D;
	dsi_set_cmdq(data_array, 6, 1);

  	data_array[0]= 0x00043902;
	data_array[1]= 0x0000080C;
	dsi_set_cmdq(data_array, 2, 1);//add for esd

	data_array[0] = 0x00110500;	
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(120);

	data_array[0] = 0x00290500;	
	dsi_set_cmdq(data_array, 1, 1);


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

    //jane added for Dynamic FPS function
    params->dsi.vertical_frontporch_for_low_power=408;

    params->dsi.esd_check_enable = 1;
    params->dsi.customization_esd_check_enable = 1;
    params->dsi.lcm_esd_check_table[0].cmd = 0xB0;
    params->dsi.lcm_esd_check_table[0].count = 4;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x00;
    params->dsi.lcm_esd_check_table[0].para_list[1] = 0x01;
    params->dsi.lcm_esd_check_table[0].para_list[2] = 0x7C;
    params->dsi.lcm_esd_check_table[0].para_list[3] = 0x0F;
    params->dsi.lcm_esd_check_table[1].cmd = 0x9D;
    params->dsi.lcm_esd_check_table[1].count = 1;
    params->dsi.lcm_esd_check_table[1].para_list[0] = 0x00;
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
/*add by zhuqiang for PR949448 */
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
	MDELAY(10);
	//lcm_util.set_gpio_out(LCD_VCC_EN, GPIO_OUT_ONE);
	set_gpio_lcd_enn(1);
	MDELAY(10);
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
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(60);
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
/*add by zhuqiang for PR949448 */
static void lcm_suspend(void)
{
	//disable power
	//mt_set_gpio_out(LCD_VCC_EN, GPIO_OUT_ONE);
	//mt_set_gpio_out(LCD_VCC_EN, GPIO_OUT_ZERO);
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	//reset low
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);
	MDELAY(20);
	SET_RESET_PIN(1);
	MDELAY(10);
        set_gpio_lcd_enn(1);
        set_gpio_lcd_enn(0);
        MDELAY(10);
        set_gpio_lcd_enp(1);
        set_gpio_lcd_enp(0);
        MDELAY(50);
	LCD_DEBUG("uboot:pop455_hx8395a_fhd_dsi_vdo_tdt_lcm_suspend\n");

}
/*add by zhuqiang for PR949448 */
static void lcm_resume(void)
{
/*
	//enable power
	// lcm_util.set_gpio_out(LCD_VCC_EP, GPIO_OUT_ZERO);
	// lcm_util.set_gpio_out(LCD_VCC_EN, GPIO_OUT_ZERO);
	set_gpio_lcd_enp(0);
	set_gpio_lcd_enn(0);
	MDELAY(50);

	//lcm_util.set_gpio_out(LCD_VCC_EP, GPIO_OUT_ONE);
	set_gpio_lcd_enp(1);
	MDELAY(10);
	//lcm_util.set_gpio_out(LCD_VCC_EN, GPIO_OUT_ONE);
	set_gpio_lcd_enn(1);
	MDELAY(10);

	//reset low to high
	SET_RESET_PIN(1);
	MDELAY(50);
	SET_RESET_PIN(0);
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(150);
*/
	//push_table(lcm_initialization_setting_ty, sizeof(lcm_initialization_setting_ty) / sizeof(struct LCM_setting_table), 1);
	lcm_init();
	LCD_DEBUG("kernel:pop455_hx8395a_fhd_dsi_vdo_tdt_lcm_resume\n");

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
LCM_DRIVER shineplus_hx8395a_fhd_dsi_vdo_tdt_lcm_drv =
{
    .name           = "shineplus_hx8395a_fhd_dsi_vdo_tdt",
    .set_util_funcs = lcm_set_util_funcs,
    .get_params     = lcm_get_params,
    .init           = lcm_init,/*kd init fun.*/
    .suspend        = lcm_suspend,
    .resume         = lcm_resume,
    .compare_id     = lcm_compare_id,
   
};
