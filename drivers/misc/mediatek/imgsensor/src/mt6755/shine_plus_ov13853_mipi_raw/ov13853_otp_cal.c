#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>


#include <linux/proc_fs.h> 


#include <linux/dma-mapping.h>


#include <linux/types.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"
#include "kd_camera_feature.h"

#include "ov13853mipiraw_Sensor.h"
#include "ov13853_otp_cal.h"

#if 1
extern int iReadReg(u16 a_u2Addr , u8 * a_puBuff , u16 i2cId);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);

//#define OV13853_R2A_write_i2c(addr, para) iWriteReg((u16) addr , (u32) para , 1, OV13853MIPI_WRITE_ID)
#define PFX "OV13853_R2A_OTP"
//#define LOG_INF(format, args...)	//xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)
#define LOG_INF(format, args...)    pr_debug(PFX "[%s] " format, __FUNCTION__, ##args)

#define Delay(ms)  mdelay(ms)
static unsigned char OV13853MIPI_WRITE_ID = 0XA0;
int OV13853_OTP_Flag=0;
int OV13853_pdaf_Flag=0;
static unsigned char lenc[1868];
kal_uint16 OV13853_R2A_read_i2c(kal_uint32 addr)
{
    kal_uint16 get_byte=0;
    iReadReg((u16) addr ,(u8*)&get_byte,OV13853MIPI_WRITE_ID);
    return get_byte;
}

kal_uint16 OV13853_R2A_write_i2c(u16 addr, u32 para)
{
		iWriteReg((u16) addr , (u32) para , 1, OV13853MIPI_WRITE_ID);
		return 1;
}
#endif

void otp_cali(void)
{
	struct otp_struct current_otp;
	//struct pdaf_struct current_pdaf;
	//OV13853MIPI_WRITE_ID = writeid;
	memset(&current_otp, 0, sizeof(struct otp_struct));
	//memset(&current_pdaf, 0, sizeof(struct pdaf_struct));
	read_otp(&current_otp);
	//read_pdaf(&current_pdaf);
	//apply_otp(&current_otp);
}
void pdaf_cali(void)
{
	
	struct pdaf_struct current_pdaf;
	memset(&current_pdaf, 0, sizeof(struct pdaf_struct));
	read_pdaf(&current_pdaf);
}
int read_pdaf(struct pdaf_struct *pdaf_ptr)
{
	int pdaf_flag=0;
	int checksumpdaf=0;
	int pdaf_offset = 0x79b;
	int i=0;
	
	//set 0x5002[1] to "0"
	int temp1=0;
	temp1 = OV13853_R2A_read_i2c(0x5002);
	OV13853_R2A_write_i2c(0x5002, (0x00 & 0x02) | (temp1 & (~0x02)));
	// read OTP into buffer
	OV13853_R2A_write_i2c(0x3d84, 0xC0);
	OV13853_R2A_write_i2c(0x3d88, 0x00); // OTP start address
	OV13853_R2A_write_i2c(0x3d89, 0x00);
	OV13853_R2A_write_i2c(0x3d8A, 0x01); // OTP end address
	OV13853_R2A_write_i2c(0x3d8B, 0xE5);
	OV13853_R2A_write_i2c(0x3d81, 0x01); // load otp into buffer
	Delay(10);
	  //pdaf
	  pdaf_flag=OV13853_R2A_read_i2c(0x079A);
	  printk("pdaf_flag : 0x%x \n", pdaf_flag);
	  if(pdaf_flag==0x03)
	  {
		for(i = 0; i < 1372; i++) {
		(* pdaf_ptr).pdaf[i]= OV13853_R2A_read_i2c(pdaf_offset + i);
			checksumpdaf += (* pdaf_ptr).pdaf[i];
			printk(" Lenc (* otp_ptr).pdaf[%d] : %x \n", i, (* pdaf_ptr).pdaf[i]);	
		}
		checksumpdaf = (checksumpdaf)%256;
		(* pdaf_ptr).checksumpdaf=OV13853_R2A_read_i2c(0x0D17);
		printk(" checksumpdaf:%d, (* otp_ptr).checksumpdaf :%d\n", checksumpdaf, (* pdaf_ptr).checksumpdaf);
	  }
	  else{
		printk("no pdaf data\n");
	  }
      if((pdaf_flag==0x03)&&(checksumpdaf==(* pdaf_ptr).checksumpdaf))
      	{
		OV13853_pdaf_Flag=1;
	  }
	  else{
		OV13853_pdaf_Flag=0;
	  }
	//set 0x5002[1] to "1"
	temp1 = OV13853_R2A_read_i2c(0x5002);
	OV13853_R2A_write_i2c(0x5002, (0x02 & 0x02) | (temp1 & (~0x02)));

	//return (*otp_ptr).flag;
	return 0;
}
int read_otp(struct otp_struct *otp_ptr)
{
	int otp_flag=0;
	int awb_flag=0;
	int vcm_flag=0;
	int lsc_flag=0;
	int addr=0;
	int lsc_addr=0;
	int af_addr=0;
	int i=0;
	int checksum=0;
	int checksumAWB=0;
	int checksumAF=0;
	int checksumLSC = 0;
	//int pdaf_flag=0;
	//int checksumpdaf=0;
	//int pdaf_offset = 0x79b;
	
	
	//set 0x5002[1] to "0"
	int temp1=0;
	temp1 = OV13853_R2A_read_i2c(0x5002);
	OV13853_R2A_write_i2c(0x5002, (0x00 & 0x02) | (temp1 & (~0x02)));
	// read OTP into buffer
	OV13853_R2A_write_i2c(0x3d84, 0xC0);
	OV13853_R2A_write_i2c(0x3d88, 0x00); // OTP start address
	OV13853_R2A_write_i2c(0x3d89, 0x00);
	OV13853_R2A_write_i2c(0x3d8A, 0x01); // OTP end address
	OV13853_R2A_write_i2c(0x3d8B, 0xE5);
	OV13853_R2A_write_i2c(0x3d81, 0x01); // load otp into buffer
	Delay(10);
	// OTP base information 
	otp_flag = OV13853_R2A_read_i2c(0x0000);
	LOG_INF(" module information : %x \n", otp_flag);
	addr = 0x0000;
	
	if(otp_flag == 0x01) {
		//(*otp_ptr).flag = 0x01; // valid info  in OTP
		(*otp_ptr).module_integrator_id = OV13853_R2A_read_i2c(addr+1);
		(*otp_ptr).lens_id = OV13853_R2A_read_i2c( addr + 5);
		(*otp_ptr).production_year = OV13853_R2A_read_i2c( addr + 2);
		(*otp_ptr).production_month = OV13853_R2A_read_i2c( addr + 3);
		(*otp_ptr).production_day = OV13853_R2A_read_i2c(addr + 4);
		(*otp_ptr).vcm_id = OV13853_R2A_read_i2c(addr + 6);
		(*otp_ptr).vcm_ic_id = OV13853_R2A_read_i2c(addr + 7);
		(*otp_ptr).checksumOTP = OV13853_R2A_read_i2c(addr + 8);
	}
	else {
	       //(*otp_ptr).flag = 0x00; // not info in OTP
		(*otp_ptr).module_integrator_id = 0;
		(*otp_ptr).lens_id = 0;
		(*otp_ptr).production_year = 0;
		(*otp_ptr).production_month = 0;
		(*otp_ptr).production_day = 0;
		(*otp_ptr).vcm_id=0;
		(*otp_ptr).vcm_ic_id=0;
		(*otp_ptr).checksumOTP = 0;
	}
	for(i=1;i<=7;i++){
	checksum+=OV13853_R2A_read_i2c(addr+i);
	}
	checksum = (checksum)%256;
	 printk("[OV13853 OTP]otp_flag=%d,module_integrator_id=0x%x,lens_id=0x%x,data=%d-%d-%d\n",otp_flag,(*otp_ptr).module_integrator_id,(*otp_ptr).lens_id,(*otp_ptr).production_year,(*otp_ptr).production_month,(*otp_ptr).production_day);
	printk("[OV13853 OTP]vcm_id=0x%x,vcm_ic_id=0x%x",(*otp_ptr).vcm_id,(*otp_ptr).vcm_ic_id);
	printk("[OV13853 OTP]checksum =%d",checksum);
	 printk("[OV13853 OTP](*otp_ptr).checksumOTP =%d",(*otp_ptr).checksumOTP);
	//OTP AWB information
	awb_flag=OV13853_R2A_read_i2c(0x0009);
	if(awb_flag==0x01){
		//(*otp_ptr).flag = 0x01;
		(*otp_ptr).R_Gr_ratio = ((OV13853_R2A_read_i2c(0x000A)<<8)&0xff00) | (OV13853_R2A_read_i2c(0x000B));
		(*otp_ptr).B_Gr_ratio = ((OV13853_R2A_read_i2c(0x000C)<<8)&0xff00) | (OV13853_R2A_read_i2c(0x000D));
		(*otp_ptr).Gb_Gr_ratio =((OV13853_R2A_read_i2c(0x000E)<<8)&0xff00) | (OV13853_R2A_read_i2c(0x000F));
		(*otp_ptr).g_R_Gr_ratio = ((OV13853_R2A_read_i2c(0x0010)<<8)&0xff00) | (OV13853_R2A_read_i2c(0x0011));
		(*otp_ptr).g_B_Gr_ratio = ((OV13853_R2A_read_i2c(0x0012)<<8)&0xff00) | (OV13853_R2A_read_i2c(0x0013));
		(*otp_ptr).g_Gb_Gr_ratio = ((OV13853_R2A_read_i2c(0x0014)<<8)&0xff00) | (OV13853_R2A_read_i2c(0x0015));
		(*otp_ptr).checksumAWB=OV13853_R2A_read_i2c(0x0016);
	}
	else{
		(*otp_ptr).R_Gr_ratio = 0;
		(*otp_ptr).B_Gr_ratio = 0;
		(*otp_ptr).Gb_Gr_ratio =0;
		(*otp_ptr).g_R_Gr_ratio = 0;
		(*otp_ptr).g_B_Gr_ratio = 0;
		(*otp_ptr).g_Gb_Gr_ratio = 0;
		(*otp_ptr).checksumAWB=0;
	}
	for(i=0;i<12;i++){
	checksumAWB+=OV13853_R2A_read_i2c(0x000A+i);
	}
	checksumAWB = (checksumAWB)%256;
	printk("[OV13853 OTP]AWB_flag=%d,R_Gr_ratio=0x%x,B_Gr_ratio=0x%x,Gb_Gr_ratio=0x%x\n",awb_flag,(*otp_ptr).R_Gr_ratio,(*otp_ptr).B_Gr_ratio,(*otp_ptr).Gb_Gr_ratio);
	printk("[OV13853 OTP]g_R_Gr_ratio=0x%x,g_B_Gr_ratio=0x%x,g_Gb_Gr_ratio=0x%x\n",(*otp_ptr).g_R_Gr_ratio,(*otp_ptr).g_B_Gr_ratio,(*otp_ptr).g_Gb_Gr_ratio);
	printk("[OV13853 OTP](*otp_ptr).checksumAWB=%d\n",(*otp_ptr).checksumAWB);
	printk("[OV13853 OTP]checksumAWB=%d\n",checksumAWB);
	// OTP VCM Calibration
	vcm_flag = OV13853_R2A_read_i2c(0x0765);
	printk(" [OV13853 OTP]VCM calibration flag : %x \n", vcm_flag);

	if(vcm_flag == 0x01) {
		//(*otp_ptr).flag == 0x01;
		(* otp_ptr).VCM_macro = ((OV13853_R2A_read_i2c(0x0769)<<8)&0xff00) | (OV13853_R2A_read_i2c(0x076A));
		(* otp_ptr).VCM_inf = ((OV13853_R2A_read_i2c(0x0767)<<8)&0xff00) | (OV13853_R2A_read_i2c(0x0768));
		(* otp_ptr).VCM_dir = OV13853_R2A_read_i2c(0x0766);
		(* otp_ptr).VCM_start = ((OV13853_R2A_read_i2c(0x076B)<<8)&0xff00) | (OV13853_R2A_read_i2c(0x076C));
		(* otp_ptr).checksumAF = OV13853_R2A_read_i2c(0x076D);
	}
	else {
		(* otp_ptr).VCM_macro = 0;
		(* otp_ptr).VCM_inf = 0;
		(* otp_ptr).VCM_dir = 0;
		(* otp_ptr).VCM_start = 0;
		(* otp_ptr).checksumAF = 0;
	}
	af_addr=0x0766;
	for(i=0;i<7;i++){
	checksumAF+=OV13853_R2A_read_i2c(af_addr+i);
	}
	checksumAF = (checksumAF)%256;
	printk(" [OV13853 OTP]VCM_macro: %x ,VCM_inf: %x,VCM_dir: %x,VCM_start: %x,(* otp_ptr).checksumAF: %d\n", (* otp_ptr).VCM_macro,(* otp_ptr).VCM_inf,(* otp_ptr).VCM_dir,(* otp_ptr).VCM_start,(* otp_ptr).checksumAF);
	printk(" [OV13853 OTP]checksumAF:%d\n",checksumAF);
	// OTP Lenc Calibration
	lsc_flag = OV13853_R2A_read_i2c(0x0017);
	printk(" Lenc calibration flag : %x \n", lsc_flag);
	lsc_addr=0x0018;
      if(lsc_flag == 0x01) {
		for(i=0;i<1868;i++) {
			lenc[i]= OV13853_R2A_read_i2c(lsc_addr + i);
			checksumLSC += lenc[i];
			printk(" Lenc (* otp_ptr).lenc[%d] : %x \n", i, lenc[i]);
		}
		//Decode the lenc buffer from OTP , from 186 bytes to 360 bytes
		//int lenc_out[360];

		checksumLSC = (checksumLSC)%256;
		(* otp_ptr).checksumLSC=OV13853_R2A_read_i2c(0x0764);
		printk(" checksumLSC:%d, (* otp_ptr).checksumLSC :%d\n", checksumLSC, (* otp_ptr).checksumLSC);
	}
	  else{
			for(i=0;i<1868;i++) {
			lenc[i]= 0;
			printk(" Lenc (* otp_ptr).lenc[%d] : %x \n", i, lenc[i]);
			}

	  }
	
	if((otp_flag==0x01)&&(awb_flag==0x01)&&(vcm_flag==0x01)&&(lsc_flag==0x01)&&((*otp_ptr).module_integrator_id==0x6)&&((*otp_ptr).lens_id==0x12)&&((*otp_ptr).vcm_id==0x62)&&((*otp_ptr).vcm_ic_id==0xe)&&(checksum==(*otp_ptr).checksumOTP)&&(checksumAWB==(*otp_ptr).checksumAWB)&&(checksumAF==(* otp_ptr).checksumAF)&&(checksumLSC==(* otp_ptr).checksumLSC))
	{
			OV13853_OTP_Flag=1;
	}
	else{
			OV13853_OTP_Flag=0;
		}

	//set 0x5002[1] to "1"
	temp1 = OV13853_R2A_read_i2c(0x5002);
	OV13853_R2A_write_i2c(0x5002, (0x02 & 0x02) | (temp1 & (~0x02)));

	//return (*otp_ptr).flag;
	return 0;
}


