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

#if defined(CONFIG_MTK_HDMI_SUPPORT)
#include <linux/kernel.h>

/*#include <linux/xlog.h>*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/delay.h>
/*#include <mtk_kpd.h>*/

#include <mt-plat/upmu_common.h>
#include <mach/mt_pmic.h>

#include "hdmi_drv.h"
#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#include "smartbook.h"
#endif

#ifdef CONFIG_MTK_LEGACY
#include "hdmi_cust.h"
#endif
#include <linux/interrupt.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif

#include "siHdmiTx_902x_TPI.h"
//#include "dlpc2607_iic.h"
//#include "dpp2607.h"
//#include "mt8193.h"
/**
* MHL TX Chip Driver User Layer Interface
*/
extern struct mhl_dev_context *si_dev_context;
extern void ForceSwitchToD3( struct mhl_dev_context *dev_context);
extern void	ForceNotSwitchToD3(void);
extern int si_mhl_tx_post_initialize(struct mhl_dev_context *dev_context, bool bootup);
extern void siHdmiTx_VideoSel (byte vmode);
extern void siHdmiTx_AudioSel (byte AduioMode);
extern void set_platform_bitwidth(int bitWidth);
extern bool si_mhl_tx_set_path_en_I(struct mhl_dev_context *dev_context);
extern bool packed_pixel_available(struct mhl_dev_context *dev_context);
extern int dongle_dsc_dec_available(struct mhl_dev_context *dev_context);
extern void configure_and_send_audio_info(struct mhl_dev_context *dev_context, int audio_format);

//Should align to mhl_linux_tx.h
#define	MHL_TX_EVENT_DISCONNECTION	0x01
#define	MHL_TX_EVENT_CONNECTION		0x02
#define MHL_TX_EVENT_SMB_DATA		0x40
#define MHL_TX_EVENT_HPD_CLEAR 		0x41
#define MHL_TX_EVENT_HPD_GOT 		0x42
#define MHL_TX_EVENT_DEV_CAP_UPDATE 0x43
#define MHL_TX_EVENT_EDID_UPDATE 	0x44
#define MHL_TX_EVENT_EDID_DONE 		0x45
#define MHL_TX_EVENT_CALLBACK 		0x46

/**
* Platform Related Layer Interface
*/
extern int HalOpenI2cDevice(char const *DeviceName, char const *DriverName);
extern int32_t sii_8348_tx_init(void);    //Should  move to MHL TX Chip user layer
/**
* LOG For MHL TX Chip HAL
*/
static size_t hdmi_log_on = true;
static int txInitFlag = 0;
int	chip_device_id = 0;
bool need_reset_usb_switch = true;
int dlpc2607_status = 0;
static uint8_t ucHdmi_isr_en;

#define MHL_DBG(fmt, arg...)  \
	do { \
		if (1) printk("[HDMI_Chip_HAL]:"fmt, ##arg);  \
	}while (0)

#define MHL_FUNC()    \
	do { \
		if(hdmi_log_on) printk("[HDMI_Chip_HAL] %s\n", __func__); \
	}while (0)

void hdmi_drv_log_enable(bool enable)
{
//	hdmi_log_on = enable;
}
		
extern int sil9022_set_reset(int reset);	
void HDMI_reset(void)
{


	sil9022_set_reset(1);
	mdelay(20);
	sil9022_set_reset(0);
	mdelay(50);
	sil9022_set_reset(1);
	mdelay(20);
}
static int not_switch_to_d3 = 0;
//static int audio_enable = 0;

void hdmi_drv_force_on(int from_uart_drv )
{
    MHL_DBG("hdmi_drv_force_on %d\n", from_uart_drv);
}
 
/************************** Upper Layer To HAL*********************************/
struct HDMI_UTIL_FUNCS hdmi_util = {0};
static void hdmi_drv_set_util_funcs(const struct HDMI_UTIL_FUNCS *util)
{
	memcpy(&hdmi_util, util, sizeof(struct HDMI_UTIL_FUNCS));
}

//static unsigned int HDCP_Supported_Info = 0;
//bool MHL_3D_Support = false;
int MHL_3D_format=0x00;
static void hdmi_drv_get_params(struct HDMI_PARAMS *params)
{

#if 1
	//char* cable_str = "";
	enum HDMI_VIDEO_RESOLUTION input_resolution = params->init_config.vformat;
	memset(params, 0, sizeof(struct HDMI_PARAMS));

	printk("%s:===zhaolong debug Sil9022A===input_resolution=%d\n", __func__, input_resolution);
	
	input_resolution = HDMI_VIDEO_1920x1080p_60Hz;
	
	switch (input_resolution)
	{
		case HDMI_VIDEO_720x480p_60Hz:
			params->clk_pol   = HDMI_POLARITY_FALLING;
			params->de_pol    = HDMI_POLARITY_RISING;
			params->hsync_pol = HDMI_POLARITY_RISING;
			params->vsync_pol = HDMI_POLARITY_RISING;
			
			params->hsync_pulse_width = 62;
			params->hsync_back_porch  = 60;
			params->hsync_front_porch = 16;
			
			params->vsync_pulse_width = 6;
			params->vsync_back_porch  = 30;
			params->vsync_front_porch = 9;
			
			params->width       = 720;
			params->height      = 480;
			params->input_clock = 27027;

			params->init_config.vformat = HDMI_VIDEO_720x480p_60Hz;
			break;
		case HDMI_VIDEO_1280x720p_60Hz:
			params->clk_pol   = HDMI_POLARITY_RISING;
			params->de_pol    = HDMI_POLARITY_RISING;
			params->hsync_pol = HDMI_POLARITY_FALLING;
			params->vsync_pol = HDMI_POLARITY_FALLING;
			
			params->hsync_pulse_width = 40;
			params->hsync_back_porch  = 220;
			params->hsync_front_porch = 110;
			
			params->vsync_pulse_width = 5;
			params->vsync_back_porch  = 20;
			params->vsync_front_porch = 5;
		
			params->width       = 1280;
			params->height      = 720;
#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
			if (MHL_Connect_type == MHL_SMB_CABLE)
			{
				params->width  = 1366;
				params->height = 768;
			}
#endif
			params->input_clock = 74250;

			params->init_config.vformat = HDMI_VIDEO_1280x720p_60Hz;
			break;
		case HDMI_VIDEO_1920x1080p_30Hz:
			params->clk_pol   = HDMI_POLARITY_RISING;
			params->de_pol    = HDMI_POLARITY_RISING;
			params->hsync_pol = HDMI_POLARITY_FALLING;
			params->vsync_pol = HDMI_POLARITY_FALLING;
			
			params->hsync_pulse_width = 44;
			params->hsync_back_porch  = 148;
			params->hsync_front_porch = 88;
			
			params->vsync_pulse_width = 5;
			params->vsync_back_porch  = 36;
			params->vsync_front_porch = 4;
			
			params->width       = 1920;
			params->height      = 1080;
			params->input_clock = 74250;

			params->init_config.vformat = HDMI_VIDEO_1920x1080p_30Hz;
			break;
		case HDMI_VIDEO_1920x1080p_60Hz:
			params->clk_pol   = HDMI_POLARITY_RISING;
			params->de_pol    = HDMI_POLARITY_RISING;
			params->hsync_pol = HDMI_POLARITY_FALLING;
			params->vsync_pol = HDMI_POLARITY_FALLING;
			
			params->hsync_pulse_width = 44;
			params->hsync_back_porch  = 148;
			params->hsync_front_porch = 88;
			
			params->vsync_pulse_width = 5;
			params->vsync_back_porch  = 36;
			params->vsync_front_porch = 4;
			
			params->width       = 1920;
			params->height      = 1080;
			params->input_clock = 148500;

			params->init_config.vformat = HDMI_VIDEO_1920x1080p_60Hz;
			break;
		default:
			MHL_DBG("Unknow support resolution\n");
			break;
	}
	
#else
	memset(params, 0, sizeof(struct HDMI_PARAMS));

	pr_debug("720p\n");
	params->init_config.vformat = HDMI_VIDEO_1280x720p_60Hz;
	//params->init_config.aformat = HDMI_AUDIO_48K_2CH;
	
	params->clk_pol = HDMI_POLARITY_RISING;
	params->de_pol = HDMI_POLARITY_RISING;
	params->hsync_pol = HDMI_POLARITY_FALLING;
	params->vsync_pol = HDMI_POLARITY_FALLING;
	params->hsync_pulse_width = 40;
	params->hsync_back_porch  = 220;
	params->hsync_front_porch = 110;
	params->vsync_pulse_width = 5;
	params->vsync_back_porch  = 20;
	params->vsync_front_porch = 5;
	params->width = 1280;
	params->height = 720;
	params->input_clock = 74250;
#endif

	params->init_config.aformat         = HDMI_AUDIO_44K_2CH;
	params->rgb_order         			= HDMI_COLOR_ORDER_RGB;
	params->io_driving_current 			= IO_DRIVING_CURRENT_2MA;
	params->intermediat_buffer_num 		= 4;
	params->scaling_factor 				= 0;
	//params->HDCPSupported 				= HDCP_Supported_Info;

	params->cabletype 				= HDMI_CABLE;

	//params->is_3d_support 				= MHL_3D_Support;
	//params->output_mode = HDMI_OUTPUT_MODE_LCD_MIRROR;
	//params->is_force_awake = 1;
	//params->is_force_landscape = 1;

}

void hdmi_drv_suspend(void) {return ;}
void hdmi_drv_resume(void)  {return ;}
static int hdmi_drv_audio_config(enum HDMI_AUDIO_FORMAT aformat, int bitWidth)
{
	return 0;
}
static int hdmi_drv_video_enable(bool enable) 
{
    return 0;
}
/*----------------------------------------------------------------------------*/
extern int sil9022_set_iis_gpio(int onoff);
extern int sil9022_set_gpio(int onoff);


void Set_I2S_Pin(bool enable)
{

	if((1 == ucHdmi_isr_en) &&
		(enable == true))
	{
		sil9022_set_iis_gpio(0);

	}
	else
	{
		sil9022_set_iis_gpio(1);
	}

    return ;

}

static int hdmi_drv_audio_enable(bool enable)  
{
	bool flag = enable;

    MHL_DBG("[EXTD]Set_I2S_Pin, enable = %d\n", enable);   
	Set_I2S_Pin(flag);

    return 0;
}

static int hdmi_drv_enter(void)  {return 0;}
static int hdmi_drv_exit(void)  {return 0;}

static int hdmi_drv_video_config(enum HDMI_VIDEO_RESOLUTION vformat, enum HDMI_VIDEO_INPUT_FORMAT vin, int vout)
{

	byte sii9024_format;

	if (vformat == HDMI_VIDEO_720x480p_60Hz)
		sii9024_format = HDMI_480P60_4X3;
	else if (vformat == HDMI_VIDEO_1280x720p_60Hz)
		sii9024_format = HDMI_720P60;
	else if (vformat == HDMI_VIDEO_1920x1080p_30Hz)
		sii9024_format = HDMI_1080P30;
	else {
		printk("error:sii9024_video_config vformat=%d\n", vformat);
		sii9024_format = HDMI_720P60;
	}

	switch (sii9024_format) {
	case HDMI_480P60_4X3:
		siHdmiTx_VideoSel(HDMI_480P60_4X3);
		break;

	case HDMI_720P60:
		siHdmiTx_VideoSel(HDMI_720P60);
		break;

	case HDMI_1080P30:
		siHdmiTx_VideoSel(HDMI_1080P30);
		break;

	default:
		siHdmiTx_VideoSel(HDMI_720P60);
		break;
	}

	siHdmiTx_VideoSet();

	/* siHdmiTx_TPI_Init(); */
	/* siHdmiTx_PowerStateD3(); */
	return 0;


}

enum HDMI_STATE hdmi_drv_get_state(void)
{
	MHL_DBG("in\n");
//	return HDMI_STATE_ACTIVE;
	return HDMI_STATE_NO_DEVICE;
}

bool chip_inited = false;
static int hdmi_drv_init(void)
{
	MHL_DBG("hdmi_drv_init, not_switch_to_d3: %d, init-%d\n", not_switch_to_d3, chip_inited);
	if(chip_inited == true)
		return 0;

	txInitFlag = 0;
	chip_inited = true;

	//Should be enhanced
	chip_device_id = 0;
	need_reset_usb_switch = true;

	MHL_DBG("hdmi_drv_init -\n" );
	return 0;
}
extern int sil9022_1v2_onoff(int onoff);
extern  int hdmi_irq;
int hdmi_drv_power_on(void)
{

	MHL_FUNC();

	if (0 == ucHdmi_isr_en) {
 
		sil9022_1v2_onoff(1);	  
		ucHdmi_isr_en = 1;
		sil9022_set_gpio(0);
		siHdmiTx_VideoSel(HDMI_720P60);
		siHdmiTx_AudioSel(AFS_44K1);
		siHdmiTx_TPI_Init();
		
		enable_irq(hdmi_irq);
     
	}
	return 0;        

}

void hdmi_drv_power_off(void)
{
	MHL_FUNC();

	need_reset_usb_switch = false;
	
	if (ucHdmi_isr_en) {

		  disable_irq(hdmi_irq);
		  sil9022_1v2_onoff(0);
		  sil9022_set_reset(0);
			sil9022_set_gpio(1);
			ucHdmi_isr_en = 0;
	}

	

	chip_inited = false;

	return ;
}

int hdmi_drv_get_external_device_capablity(void)
{
	int capablity = 0;

	return capablity;
}

#define HDMI_MAX_INSERT_CALLBACK   10
static CABLE_INSERT_CALLBACK hdmi_callback_table[HDMI_MAX_INSERT_CALLBACK];
void hdmi_register_cable_insert_callback(CABLE_INSERT_CALLBACK cb)
{
    int i = 0;
    for (i = 0; i < HDMI_MAX_INSERT_CALLBACK; i++) {
        if (hdmi_callback_table[i] == cb)
            break;
    }
    if (i < HDMI_MAX_INSERT_CALLBACK)
        return;

    for (i = 0; i < HDMI_MAX_INSERT_CALLBACK; i++) {
        if (hdmi_callback_table[i] == NULL)
            break;
    }
    if (i == HDMI_MAX_INSERT_CALLBACK) {
        MHL_DBG("not enough mhl callback entries for module\n");
        return;
    }

    hdmi_callback_table[i] = cb;
	MHL_DBG("callback: %p,i: %d\n", hdmi_callback_table[i], i);
}

void hdmi_unregister_cable_insert_callback(CABLE_INSERT_CALLBACK cb)
{
    int i;
    for (i=0; i<HDMI_MAX_INSERT_CALLBACK; i++)
    {
        if (hdmi_callback_table[i] == cb)
        {
        	MHL_DBG("unregister cable insert callback: %p, i: %d\n", hdmi_callback_table[i], i);
            hdmi_callback_table[i] = NULL;
            break;
        }
    }
    if (i == HDMI_MAX_INSERT_CALLBACK)
    {
        MHL_DBG("Try to unregister callback function 0x%lx which was not registered\n",(unsigned long int)cb);
        return;
    }
}

void hdmi_invoke_cable_callbacks(enum HDMI_STATE state)
{
    int i = 0, j = 0;   
    for (i=0; i<HDMI_MAX_INSERT_CALLBACK; i++)   // 0 is for external display
    {
        if(hdmi_callback_table[i])
        {
			j = i;
        }
    }

	if (hdmi_callback_table[j])
	{
		MHL_DBG("callback: %p, state: %d, j: %d\n", hdmi_callback_table[j], state, j);
		hdmi_callback_table[j](state);
	}
}

void hdmi_GetEdidInfo(void *pv_get_info)
{
	//unsigned int ui4CEA_NTSC = SINK_480P | SINK_720P60 | SINK_1080P30 ;
	//pv_get_info->ui4_ntsc_resolution |= ui4CEA_NTSC;
}

/************************** ****************************************************/
const struct HDMI_DRIVER* HDMI_GetDriver(void)
{
	static const struct HDMI_DRIVER HDMI_DRV =
	{
		.set_util_funcs = hdmi_drv_set_util_funcs,
		.get_params     = hdmi_drv_get_params,
		.init           = hdmi_drv_init,
        .enter          = hdmi_drv_enter,
        .exit           = hdmi_drv_exit,
		.suspend        = hdmi_drv_suspend,
		.resume         = hdmi_drv_resume,
        .video_config   = hdmi_drv_video_config,
        .audio_config   = hdmi_drv_audio_config,
        .video_enable   = hdmi_drv_video_enable,
        .audio_enable   = hdmi_drv_audio_enable,
        .power_on       = hdmi_drv_power_on,
        .power_off      = hdmi_drv_power_off,
        .get_state      = hdmi_drv_get_state,
        .log_enable     = hdmi_drv_log_enable,
        .getedid        = hdmi_GetEdidInfo,
        .get_external_device_capablity = hdmi_drv_get_external_device_capablity,
		.register_callback   = hdmi_register_cable_insert_callback,
		.unregister_callback = hdmi_unregister_cable_insert_callback,
        .force_on = hdmi_drv_force_on,
	};

    MHL_FUNC();
	return &HDMI_DRV;
}
/************************** ****************************************************/

#endif
