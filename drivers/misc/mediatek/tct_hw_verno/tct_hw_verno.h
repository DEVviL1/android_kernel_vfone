#ifndef __TCT_HW_VERNO_H__
#define __TCT_HW_VERNO_H__

#include <linux/types.h>

#define HW_FIRST_SOURCE "1st"
#define HW_SECOND_SOURCE "2nd"
#define HW_NOT_IN_PURCHARSE		"found but not in purchasing plan"
/* ************************************ */
/* LCD */
/* ************************************ */
#define HW_LCD_SOURCE_TOTAL 		(2)
#define HW_1ST_LCD_NAME 		"pop455_hx8395a_fhd_dsi_vdo_tdt"
#define HW_1ST_LCD_PARA_VERNO		"V01"
#define HW_2ND_LCD_NAME 		"shineplus_r63350_fhd_dsi_vdo_tdt"
#define HW_2ND_LCD_PARA_VERNO		"V01"

/* ************************************ */
/* TP */
/* ************************************ */
#define HW_TP_SOURCE_TOTAL 		(2)
#define HW_1ST_TP_NAME 			"ft5436"
//#define HW_1ST_TP_PARA_VERNO		"V01"
#define HW_2ND_TP_NAME 			"msg2xxx"
//#define HW_2ND_TP_PARA_VERNO		"V01"

/* ************************************ */
/* MAIN CAMERA */
/* ************************************ */
#define HW_MAIN_CAMERA_SOURCE_TOTAL 	(2)
#define HW_1ST_MAIN_CAMERA_NAME 	"ov13853mipiraw"
#define HW_1ST_MAIN_CAMERA_PARA_VERNO	"V0.10"
#define HW_2ND_MAIN_CAMERA_NAME 	"s5k3l8mipiraw"
#define HW_2ND_MAIN_CAMERA_PARA_VERNO	"V0.6"

/* ************************************ */
/* SUB CAMERA */
/* ************************************ */
#define HW_SUB_CAMERA_SOURCE_TOTAL 	(2)
#define HW_1ST_SUB_CAMERA_NAME 		"s5k5e8yxmipiraw"
#define HW_1ST_SUB_CAMERA_PARA_VERNO	"V0.6"
#define HW_2ND_SUB_CAMERA_NAME 		"ov5675mipiraw"
#define HW_2ND_SUB_CAMERA_PARA_VERNO	"V0.4"

/* ************************************ */
/* RF */
/* ************************************ */
#define RF_PARA_VERNO			"V01"

/* ************************************ */
/* AUDIO */
/* ************************************ */
#define AUDIO_PARA_VERNO 		"V4.0"

enum tct_source_info {
        TCT_SOURCE_INFO_INDEX = 0,
        TCT_SOURCE_INFO_NAME,
        TCT_SOURCE_INFO_VERNO,
        TCT_SOURCE_INFO_MAX
};

#endif // __TCT_HW_VERNO_H__
