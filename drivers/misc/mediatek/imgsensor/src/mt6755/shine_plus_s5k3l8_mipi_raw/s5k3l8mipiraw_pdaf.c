#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>

//#include <asm/system.h>

#include <linux/proc_fs.h> 


#include <linux/dma-mapping.h>

#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"


#include "s5k3l8mipiraw_Sensor.h"
/****************************Modify Following Strings for Debug****************************/
#define PFX "S5K3L8_camera_pdaf"

#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

struct otp_pdaf_struct {
unsigned char pdaf_flag; //bit[7]--0:empty; 1:Valid
unsigned char data1[496];//output data1
unsigned char data2[806];//output data2
unsigned char data3[102];//output data3
unsigned char pdaf_checksum;//checksum of pd, SUM(0x0801~0x0D7C)%255+1

};




extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);
extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
extern void kdSetI2CSpeed(u16 i2cSpeed);
#define EEPROM_READ_ID  0xA1
#define EEPROM_WRITE_ID   0xA0
#define I2C_SPEED        400  //CAT24C512 can support 1Mhz

#define START_OFFSET     0x800

#define Delay(ms)  mdelay(ms)

#define MAX_OFFSET       0xd7b
#define DATA_SIZE 4096
//BYTE eeprom_data[DATA_SIZE]= {0};
static bool get_done = false;
static int last_size = 0;
static int last_offset = 0;


bool S5K3L8_selective_read_eeprom(kal_uint16 addr, BYTE* data)
{
	char pu_send_cmd[2] = {(char)(addr >> 8) , (char)(addr & 0xFF) };
    if(addr > MAX_OFFSET)
        return false;
	kdSetI2CSpeed(I2C_SPEED);

	if(iReadRegI2C(pu_send_cmd, 2, (u8*)data, 1, EEPROM_WRITE_ID)<0)
		return false;
    return true;
}


bool S5K3L8_read_eeprom(kal_uint16 addr, BYTE* data, kal_uint32 size ){
	int i = 0;
	//int offset = addr;
	int offset = 0x79b;
	//for(i = 0; i < 1404; i++) {
	for(i = 0; i < 1404; i++) {
		if(!S5K3L8_selective_read_eeprom(offset, &data[i])){
			LOG_INF("read_eeprom 0x%0x %d fail \n",offset, data[i]);
			return false;
		}
		LOG_INF("read_eeprom 0x%0x 0x%x\n",offset, data[i]);
		offset++;
	}
	get_done = true;
	last_size = size;
	last_offset = addr;
    return true;
}

bool s5k3l8_read_otp_pdaf_data( kal_uint16 addr, BYTE* data, kal_uint32 size){
	
	LOG_INF("read_otp_pdaf_data enter");
	if(!get_done || last_size != size || last_offset != addr) {
		//if(!_read_eeprom(addr, eeprom_data, size)){
		if(!S5K3L8_read_eeprom(addr, data, size)){
			get_done = 0;
            last_size = 0;
            last_offset = 0;
			LOG_INF("read_otp_pdaf_data fail");
			return false;
		}
	}
	//memcpy(data, eeprom_data, size);
	LOG_INF("read_otp_pdaf_data end");
    return true;
}

//
