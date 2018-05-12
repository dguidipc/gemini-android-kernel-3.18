/**************************************************************************
*  aw9120_led.c
* 
*  Create Date :
* 
*  Modify Date : 
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 1.0 , 2016/11/22
* 
**************************************************************************/
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <mach/mt_gpt.h>
#include <linux/wakelock.h>
#include <linux/atomic.h>
#include "aw9120_reg.h"

//////////////////////////////////////
// Marco
//////////////////////////////////////
#define AW9120_LED_NAME		"aw9120_led"

//////////////////////////////////////
// i2c client
//////////////////////////////////////
static struct i2c_client *aw9120_i2c_client;


/* The definition of each time described as shown in figure.
 *        /-----------\
 *       /      |      \
 *      /|      |      |\
 *     / |      |      | \-----------
 *       |hold_time_ms |      |
 *       |             |      |
 * rise_time_ms  fall_time_ms |
 *                       off_time_ms
 */
#define ROM_CODE_MAX 255

/*
 * rise_time_ms = 1500
 * hold_time_ms = 500
 * fall_time_ms = 1500
 * off_time_ms = 1500
 */
int led_code_len = 7;
int led_code[ROM_CODE_MAX] = {
	0xbf00,0x9f05,0xfffa,0x3c7d,0xdffa,0x3cbb,0x2,
};


//////////////////////////////////////////////////////////////////////////////////////////
// PDN power control
//////////////////////////////////////////////////////////////////////////////////////////
struct pinctrl *aw9120ctrl = NULL;
struct pinctrl_state *aw9120_pdn_high = NULL;
struct pinctrl_state *aw9120_pdn_low = NULL;

int aw9120_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	aw9120ctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(aw9120ctrl)) {
		dev_err(&pdev->dev, "Cannot find aw9120 pinctrl!");
		ret = PTR_ERR(aw9120ctrl);
		printk("%s devm_pinctrl_get fail!\n", __func__);
	}
	aw9120_pdn_high = pinctrl_lookup_state(aw9120ctrl, "aw9120_pdn_high");
	if (IS_ERR(aw9120_pdn_high)) {
		ret = PTR_ERR(aw9120_pdn_high);
		printk("%s : pinctrl err, aw9120_pdn_high\n", __func__);
	}

	aw9120_pdn_low = pinctrl_lookup_state(aw9120ctrl, "aw9120_pdn_low");
	if (IS_ERR(aw9120_pdn_low)) {
		ret = PTR_ERR(aw9120_pdn_low);
		printk("%s : pinctrl err, aw9120_pdn_low\n", __func__);
	}

	printk("%s success\n", __func__);
	return ret;
}

static void aw9120_hw_on(void)
{
	printk("%s enter\n", __func__);
	pinctrl_select_state(aw9120ctrl, aw9120_pdn_low);
	msleep(5);
	pinctrl_select_state(aw9120ctrl, aw9120_pdn_high);
	msleep(5);
	printk("%s out\n", __func__);
}

#if 0
static void aw9120_hw_off(void)
{
	printk("%s enter\n", __func__);
	pinctrl_select_state(aw9120ctrl, aw9120_pdn_low);
	msleep(5);
	printk("%s out\n", __func__);
}
#endif
//////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////////////////////
static unsigned int i2c_write_reg(unsigned char addr, unsigned int reg_data)
{
	int ret;
	u8 wdbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= aw9120_i2c_client->addr,
			.flags	= 0,
			.len	= 3,
			.buf	= wdbuf,
		},
	};

	if(NULL == aw9120_i2c_client) {
		pr_err("msg %s aw9120_i2c_client is NULL\n", __func__);
		return -1;
	}

	wdbuf[0] = addr;
	wdbuf[2] = (unsigned char)(reg_data & 0x00ff);
	wdbuf[1] = (unsigned char)((reg_data & 0xff00)>>8);

	ret = i2c_transfer(aw9120_i2c_client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	return ret;
}


static unsigned int i2c_read_reg(unsigned char addr)
{
	int ret;
	u8 rdbuf[512] = {0};
	unsigned int getdata;

	struct i2c_msg msgs[] = {
		{
			.addr	= aw9120_i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= aw9120_i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= 2,
			.buf	= rdbuf,
		},
	};

	if(NULL == aw9120_i2c_client) {
		pr_err("msg %s aw9120_i2c_client is NULL\n", __func__);
		return -1;
	}

	rdbuf[0] = addr;

	ret = i2c_transfer(aw9120_i2c_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

	getdata=rdbuf[0] & 0x00ff;
	getdata<<= 8;
	getdata |=rdbuf[1];

	return getdata;
}


//////////////////////////////////////////////////////////////////////////////////////////
// aw9120 led 
//////////////////////////////////////////////////////////////////////////////////////////
void aw9120_led_off(void)
{
	//Disable LED Module
	unsigned int reg;
	reg = i2c_read_reg(GCR);
	reg &= 0xFFFE;
	i2c_write_reg(GCR, reg); 		// GCR-Disable LED Module
}

void aw9120_led_on(void)
{
	//Disable LED Module
	unsigned int reg;
	reg = i2c_read_reg(GCR);
	reg &= 0xFFFE;
	i2c_write_reg(GCR, reg); 		// GCR-Disable LED Module

	//LED Config
	i2c_write_reg(IMAX1,0x1111); 	// IMAX1-LED1~LED4 Current
	i2c_write_reg(IMAX2,0x1111); 	// IMAX2-LED5~LED8 Current
	i2c_write_reg(IMAX3,0x1111); 	// IMAX3-LED9~LED12 Current
	i2c_write_reg(IMAX4,0x1111); 	// IMAX4-LED13~LED16 Current
	i2c_write_reg(IMAX5,0x1111); 	// IMAX5-LED17~LED20 Current
	i2c_write_reg(LER1,0x0FFF); 	// LER1-LED1~LED12 Enable
	i2c_write_reg(LER2,0x00FF); 	// LER2-LED13~LED20 Enable
	i2c_write_reg(CTRS1,0x0FFF); 	// CTRS1-LED1~LED12: i2c Control
	i2c_write_reg(CTRS2,0x00FF); 	// CTRS2-LED13~LED20: i2c Control
	//i2c_write_reg(CMDR,0xBFFF); 	// CMDR-LED1~LED20 PWM=0xFF

	//Enable LED Module
	reg |= 0x0001;
	i2c_write_reg(GCR,reg); 		// GCR-Enable LED Module

	i2c_write_reg(CMDR,0xBFFF); 	// CMDR-LED1~LED20 PWM=0xFF
}

void aw9120_led_breath(void)
{
	//Disable LED Module
	unsigned int reg;
	int i;
	reg = i2c_read_reg(GCR);
	reg &= 0xFFFE;
	i2c_write_reg(GCR, reg); 		// GCR-Disable LED Module

	//LED Config
	i2c_write_reg(IMAX1,0x1111); 	// IMAX1-LED1~LED4 Current
	i2c_write_reg(IMAX2,0x1111); 	// IMAX2-LED5~LED8 Current
	i2c_write_reg(IMAX3,0x1111); 	// IMAX3-LED9~LED12 Current
	i2c_write_reg(IMAX4,0x1111); 	// IMAX4-LED13~LED16 Current
	i2c_write_reg(IMAX5,0x1111); 	// IMAX5-LED17~LED20 Current
	i2c_write_reg(LER1,0x0FFF); 	// LER1-LED1~LED12 Enable
	i2c_write_reg(LER2,0x00FF); 	// LER2-LED13~LED20 Enable
	i2c_write_reg(CTRS1,0x0000); 	// CTRS1-LED1~LED12: SRAM Control
	i2c_write_reg(CTRS2,0x0000); 	// CTRS2-LED13~LED20: SRAM Control

	//Enable LED Module
	reg |= 0x0001;
	i2c_write_reg(GCR,reg); 		// GCR-Enable LED Module

	// LED SRAM Hold Mode
	i2c_write_reg(PMR,0x0000);		// PMR-Load SRAM with I2C
	i2c_write_reg(RMR,0x0000);		// RMR-Hold Mode

	// Load LED SRAM
	i2c_write_reg(WADDR,0x0000);	// WADDR-SRAM Load Addr
	for(i=0; i<led_code_len; i++)
	{
		i2c_write_reg(WDATA,led_code[i]);
	}

	// LED SRAM Run
	i2c_write_reg(SADDR,0x0000);	// SADDR-SRAM Run Start Addr:0
	i2c_write_reg(PMR,0x0001);		// PMR-Reload and Excute SRAM
	i2c_write_reg(RMR,0x0002);		// RMR-Run
}

static void aw9120_led1(unsigned int red, unsigned int green, unsigned int blue)
{
	unsigned int IMAX1Reg;
	unsigned int IMAX2Reg;
	unsigned int IMAX3Reg;
	unsigned int IMAX4Reg;
	unsigned int IMAX5Reg;
	unsigned int LER1reg;
	unsigned int LER2reg;
	unsigned int CTRS1reg;
	unsigned int CTRS2reg;

	IMAX1Reg = i2c_read_reg(IMAX1);
	IMAX2Reg = i2c_read_reg(IMAX2);
	IMAX3Reg = i2c_read_reg(IMAX3);
	IMAX4Reg = i2c_read_reg(IMAX4);
	IMAX5Reg = i2c_read_reg(IMAX5);
	LER1reg = i2c_read_reg(LER1);
	LER2reg = i2c_read_reg(LER2);
	CTRS1reg = i2c_read_reg(CTRS1);
	CTRS2reg = i2c_read_reg(CTRS2);

	if (red == 0 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX1,IMAX1Reg&0xF000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,LER1reg&0xFFF8); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,CTRS1reg&0xFFF8); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0200); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0004); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0004); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0400); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0004); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0004); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0600); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0004); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0004); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0020); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0002); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0002); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0220); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0006); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0006); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0420); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0006); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0006); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0620); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0006); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0006); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0040); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0002); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0002); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0240); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0006); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0006); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0440); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0006); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0006); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0640); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0006); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0006); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0060); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0002); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0002); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0260); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0006); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0006); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0460); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0006); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0006); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0660); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0006); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0006); 	// CTRS1-LED1~LED12: i2c Control
	}	
	else if (red == 1 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0002); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0001); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0001); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0202); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0005); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0005); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0402); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0005); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0005); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0602); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0005); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0005); 	// CTRS1-LED1~LED12: i2c Control
	}	
	else if (red == 1 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0022); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0003); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0003); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0222); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0422); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0622); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0042); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0003); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0003); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0242); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0442); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0642); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0062); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0003); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0003); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0262); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0462); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0662); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0004); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0001); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0001); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0204); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0005); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0005); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0404); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0005); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0005); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0604); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0005); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0005); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0024); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0003); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0003); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0224); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0424); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0624); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0044); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0003); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0003); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0244); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0444); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0644); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0064); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0003); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0003); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0264); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0464); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0664); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0006); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0001); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0001); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0206); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0005); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0005); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0406); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0005); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0005); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0606); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0005); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0005); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0026); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0003); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0003); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0226); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0426); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0626); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0046); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0003); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0003); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0246); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0446); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0646); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0066); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0003); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0003); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0266); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0466); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0xF000)|0x0666); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(LER1,(LER1reg&0xFFF8)|0x0007); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFF8)|0x0007); 	// CTRS1-LED1~LED12: i2c Control
	}
	
}

static void aw9120_led2(unsigned int red, unsigned int green, unsigned int blue)
{
	unsigned int IMAX1Reg;
	unsigned int IMAX2Reg;
	unsigned int IMAX3Reg;
	unsigned int IMAX4Reg;
	unsigned int IMAX5Reg;
	unsigned int LER1reg;
	unsigned int LER2reg;
	unsigned int CTRS1reg;
	unsigned int CTRS2reg;

	IMAX1Reg = i2c_read_reg(IMAX1);
	IMAX2Reg = i2c_read_reg(IMAX2);
	IMAX3Reg = i2c_read_reg(IMAX3);
	IMAX4Reg = i2c_read_reg(IMAX4);
	IMAX5Reg = i2c_read_reg(IMAX5);
	LER1reg = i2c_read_reg(LER1);
	LER2reg = i2c_read_reg(LER2);
	CTRS1reg = i2c_read_reg(CTRS1);
	CTRS2reg = i2c_read_reg(CTRS2);

	if (red == 0 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX1,IMAX1Reg&0x0FFF); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,IMAX2Reg&0xFF00); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,LER1reg&0xFFC7); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,CTRS1reg&0xFFC7); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0020); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0020); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0020); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0040); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0020); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0020); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0060); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0020); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0020); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0002); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0010); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0010); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0022); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0030); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0030); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0042); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0030); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0030); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0062); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0030); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0030); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0004); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0010); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0010); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0024); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0030); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0030); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0044); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0030); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0030); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0064); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0030); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0030); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0006); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0010); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0010); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0026); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0030); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0030); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0046); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0030); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0030); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x0000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0066); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0030); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0030); 	// CTRS1-LED1~LED12: i2c Control
	}	
	else if (red == 1 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0008); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0008); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0020); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0028); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0028); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0040); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0028); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0028); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0060); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0028); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0028); 	// CTRS1-LED1~LED12: i2c Control
	}	
	else if (red == 1 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0002); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0018); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0018); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0022); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0042); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0062); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0004); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0018); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0018); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0024); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0044); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0064); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0006); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0018); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0018); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0026); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0046); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x2000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0066); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0008); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0008); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0020); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0028); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0028); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0040); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0028); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0028); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0060); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0028); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0028); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0002); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0018); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0018); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0022); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0042); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0062); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0004); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0018); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0018); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0024); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0044); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0064); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0006); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0018); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0018); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0026); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0046); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x4000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0066); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0008); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0008); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0020); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0028); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0028); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0040); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0028); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0028); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0060); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0028); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0028); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0002); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0018); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0018); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0022); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0042); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0062); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0004); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0018); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0018); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0024); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0044); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0064); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0006); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0018); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0018); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0026); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0046); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX1,(IMAX1Reg&0x0FFF)|0x6000); 	// IMAX1-LED1~LED4 Current
		i2c_write_reg(IMAX2,(IMAX2Reg&0xFF00)|0x0066); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(LER1,(LER1reg&0xFFC7)|0x0038); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0038); 	// CTRS1-LED1~LED12: i2c Control
	}
	
}

static void aw9120_led3(unsigned int red, unsigned int green, unsigned int blue)
{
	unsigned int IMAX1Reg;
	unsigned int IMAX2Reg;
	unsigned int IMAX3Reg;
	unsigned int IMAX4Reg;
	unsigned int IMAX5Reg;
	unsigned int LER1reg;
	unsigned int LER2reg;
	unsigned int CTRS1reg;
	unsigned int CTRS2reg;

	IMAX1Reg = i2c_read_reg(IMAX1);
	IMAX2Reg = i2c_read_reg(IMAX2);
	IMAX3Reg = i2c_read_reg(IMAX3);
	IMAX4Reg = i2c_read_reg(IMAX4);
	IMAX5Reg = i2c_read_reg(IMAX5);
	LER1reg = i2c_read_reg(LER1);
	LER2reg = i2c_read_reg(LER2);
	CTRS1reg = i2c_read_reg(CTRS1);
	CTRS2reg = i2c_read_reg(CTRS2);

	if (red == 0 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0000); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFFC7)|0x0000); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0100); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0100); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0100); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0100); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0100); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0100); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0080); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0080); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0180); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0180); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0180); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0180); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0180); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0180); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0080); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0080); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0180); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0180); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0180); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0180); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0180); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0180); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0080); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0080); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0180); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0180); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0180); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0180); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6000); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0180); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0180); 	// CTRS1-LED1~LED12: i2c Control
	}	
	else if (red == 1 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0040); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0040); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0140); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0140); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0140); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0140); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0140); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0140); 	// CTRS1-LED1~LED12: i2c Control
	}	
	else if (red == 1 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x00C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x00C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x00C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x00C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x00C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x00C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6200); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0040); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0040); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0140); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0140); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0140); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0140); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0140); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0140); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x00C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x00C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x00C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x00C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x00C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x00C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6400); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0040); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0040); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0140); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0140); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0140); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0140); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x0600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x0140); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x0140); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x00C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x00C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x2600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x00C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x00C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x4600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x00C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x00C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0002); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0004); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX2,(IMAX2Reg&0x00FF)|0x6600); 	// IMAX2-LED5~LED8 Current
		i2c_write_reg(IMAX3,(IMAX3Reg&0xFFF0)|0x0006); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xFE3F)|0x01C0); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xFE3F)|0x01C0); 	// CTRS1-LED1~LED12: i2c Control
	}
	
}

static void aw9120_led4(unsigned int red, unsigned int green, unsigned int blue)
{
	unsigned int IMAX1Reg;
	unsigned int IMAX2Reg;
	unsigned int IMAX3Reg;
	unsigned int IMAX4Reg;
	unsigned int IMAX5Reg;
	unsigned int LER1reg;
	unsigned int LER2reg;
	unsigned int CTRS1reg;
	unsigned int CTRS2reg;

	IMAX1Reg = i2c_read_reg(IMAX1);
	IMAX2Reg = i2c_read_reg(IMAX2);
	IMAX3Reg = i2c_read_reg(IMAX3);
	IMAX4Reg = i2c_read_reg(IMAX4);
	IMAX5Reg = i2c_read_reg(IMAX5);
	LER1reg = i2c_read_reg(LER1);
	LER2reg = i2c_read_reg(LER2);
	CTRS1reg = i2c_read_reg(CTRS1);
	CTRS2reg = i2c_read_reg(CTRS2);

	if (red == 0 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0000); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0000); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0800); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0800); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0800); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0800); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6000); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0800); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0800); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0200); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0400); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0400); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2200); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0C00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0C00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4200); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0C00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0C00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6200); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0C00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0C00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0400); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0400); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0400); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2400); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0C00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0C00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4400); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0C00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0C00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6400); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0C00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0C00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0600); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0400); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0400); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2600); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0C00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0C00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4600); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0C00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0C00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6600); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0C00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0C00); 	// CTRS1-LED1~LED12: i2c Control
	}	
	else if (red == 1 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0020); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0200); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0200); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2020); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0A00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0A00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4020); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0A00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0A00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6020); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0A00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0A00); 	// CTRS1-LED1~LED12: i2c Control
	}	
	else if (red == 1 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0220); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0600); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0600); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2220); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4220); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6220); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0420); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0600); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0600); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2420); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4420); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6420); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0620); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0600); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0600); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2620); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4620); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6620); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0040); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0200); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0200); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2040); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0A00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0A00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4040); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0A00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0A00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6040); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0A00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0A00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0240); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0600); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0600); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2240); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4240); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6240); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0440); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0600); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0600); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2440); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4440); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6440); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0640); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0600); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0600); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2640); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4640); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6640); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0060); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0200); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0200); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2060); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0A00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0A00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4060); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0A00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0A00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6060); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0A00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0A00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0260); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0600); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0600); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2260); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4260); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6260); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0460); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0600); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0600); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2460); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4460); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6460); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x0660); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0600); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0600); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x2660); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x4660); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX3,(IMAX3Reg&0x000F)|0x6660); 	// IMAX3-LED9~LED12 Current 
		i2c_write_reg(LER1,(LER1reg&0xF1FF)|0x0E00); 	// LER1-LED1~LED12 Enable
		i2c_write_reg(CTRS1,(CTRS1reg&0xF1FF)|0x0E00); 	// CTRS1-LED1~LED12: i2c Control
	}
	
}

static void aw9120_led5(unsigned int red, unsigned int green, unsigned int blue)
{
	unsigned int IMAX1Reg;
	unsigned int IMAX2Reg;
	unsigned int IMAX3Reg;
	unsigned int IMAX4Reg;
	unsigned int IMAX5Reg;
	unsigned int LER1reg;
	unsigned int LER2reg;
	unsigned int CTRS1reg;
	unsigned int CTRS2reg;

	IMAX1Reg = i2c_read_reg(IMAX1);
	IMAX2Reg = i2c_read_reg(IMAX2);
	IMAX3Reg = i2c_read_reg(IMAX3);
	IMAX4Reg = i2c_read_reg(IMAX4);
	IMAX5Reg = i2c_read_reg(IMAX5);
	LER1reg = i2c_read_reg(LER1);
	LER2reg = i2c_read_reg(LER2);
	CTRS1reg = i2c_read_reg(CTRS1);
	CTRS2reg = i2c_read_reg(CTRS2);

	if (red == 0 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0000); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0000); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0200); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0004); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0004); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0400); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0004); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0004); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0600); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0004); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0004); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0020); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0002); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0002); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0220); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0006); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0006); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0420); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0006); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0006); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0620); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0006); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0006); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0040); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0002); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0002); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0240); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0006); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0006); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0440); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0006); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0006); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0640); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0006); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0006); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0060); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0002); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0002); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0260); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0006); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0006); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0460); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0006); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0006); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0660); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0006); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0006); 	// CTRS2-LED13~LED20: i2c Control
	}	
	else if (red == 1 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0002); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0001); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0001); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0202); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0005); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0005); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0402); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0005); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0005); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0602); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0005); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0005); 	// CTRS2-LED13~LED20: i2c Control
	}	
	else if (red == 1 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0022); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0003); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0003); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0222); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0422); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0622); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0042); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0003); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0003); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0242); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0442); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0642); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0062); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0003); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0003); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0262); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0462); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0662); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0004); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0001); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0001); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0204); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0005); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0005); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0404); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0005); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0005); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0604); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0005); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0005); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0024); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0003); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0003); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0224); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0424); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0624); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0044); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0003); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0003); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0244); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0444); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0644); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0064); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0003); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0003); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0264); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0464); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0664); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0006); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0001); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0001); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0206); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0005); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0005); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0406); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0005); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0005); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0606); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0005); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0005); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0026); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0003); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0003); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0226); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0426); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0626); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0046); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0003); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0003); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0246); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0446); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0646); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0066); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0003); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0003); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0266); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0466); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0xF000)|0x0666); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(LER2,(LER2reg&0xFFF8)|0x0007); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFF8)|0x0007); 	// CTRS2-LED13~LED20: i2c Control
	}
	
}

static void aw9120_led6(unsigned int red, unsigned int green, unsigned int blue)
{
	unsigned int IMAX1Reg;
	unsigned int IMAX2Reg;
	unsigned int IMAX3Reg;
	unsigned int IMAX4Reg;
	unsigned int IMAX5Reg;
	unsigned int LER1reg;
	unsigned int LER2reg;
	unsigned int CTRS1reg;
	unsigned int CTRS2reg;

	IMAX1Reg = i2c_read_reg(IMAX1);
	IMAX2Reg = i2c_read_reg(IMAX2);
	IMAX3Reg = i2c_read_reg(IMAX3);
	IMAX4Reg = i2c_read_reg(IMAX4);
	IMAX5Reg = i2c_read_reg(IMAX5);
	LER1reg = i2c_read_reg(LER1);
	LER2reg = i2c_read_reg(LER2);
	CTRS1reg = i2c_read_reg(CTRS1);
	CTRS2reg = i2c_read_reg(CTRS2);

	if (red == 0 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX4,IMAX4Reg&0x0FFF); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,IMAX5Reg&0xFF00); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,LER2reg&0xFFC7); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,CTRS2reg&0xFFC7); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0020); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0020); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0020); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0040); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0020); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0020); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0060); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0020); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0020); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0002); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0010); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0010); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0022); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0030); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0030); 	// CTRS2-LED13~LED20: i2c Control
	} 
	else if (red == 0 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0042); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0030); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0030); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0062); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0030); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0030); 	// CTRS2-LED13~LED20: i2c Control
	} 
	else if (red == 0 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0004); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0010); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0010); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0024); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0030); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0030); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0044); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0030); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0030); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0064); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0030); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0030); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0006); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0010); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0010); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0026); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0030); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0030); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0046); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0030); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0030); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x0000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0066); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0030); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0030); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0000); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0008); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0008); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0020); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0028); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0028); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0040); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0028); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0028); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0060); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0028); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0028); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0002); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0018); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0018); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0022); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0042); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0062); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0004); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0018); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0018); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0024); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0044); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0064); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0006); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0018); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0018); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0026); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0046); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x2000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0066); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0000); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0008); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0008); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0020); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0028); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0028); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0040); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0028); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0028); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0060); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0028); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0028); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0002); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0018); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0018); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0022); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0042); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0062); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0004); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0018); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0018); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0024); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0044); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0064); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0006); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0018); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0018); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0026); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0046); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x4000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0066); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0000); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0008); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0008); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0020); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0028); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0028); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0040); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0028); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0028); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 0 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0060); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0028); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0028); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0002); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0018); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0018); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0022); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0042); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 1 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0062); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0004); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0018); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0018); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0024); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0044); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 2 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0064); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 0)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0006); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0018); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0018); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 1)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0026); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 2)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0046); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 3 && blue == 3)
	{
		i2c_write_reg(IMAX4,(IMAX4Reg&0x0FFF)|0x6000); 	// IMAX4-LED13~LED16 Current
		i2c_write_reg(IMAX5,(IMAX5Reg&0xFF00)|0x0066); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFFC7)|0x0038); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFFC7)|0x0038); 	// CTRS2-LED13~LED20: i2c Control
	}
}
	
static void aw9120_led7(unsigned int red, unsigned int green, unsigned int blue)
{
	unsigned int IMAX1Reg;
	unsigned int IMAX2Reg;
	unsigned int IMAX3Reg;
	unsigned int IMAX4Reg;
	unsigned int IMAX5Reg;
	unsigned int LER1reg;
	unsigned int LER2reg;
	unsigned int CTRS1reg;
	unsigned int CTRS2reg;

	IMAX1Reg = i2c_read_reg(IMAX1);
	IMAX2Reg = i2c_read_reg(IMAX2);
	IMAX3Reg = i2c_read_reg(IMAX3);
	IMAX4Reg = i2c_read_reg(IMAX4);
	IMAX5Reg = i2c_read_reg(IMAX5);
	LER1reg = i2c_read_reg(LER1);
	LER2reg = i2c_read_reg(LER2);
	CTRS1reg = i2c_read_reg(CTRS1);
	CTRS2reg = i2c_read_reg(CTRS2);

	if (red == 0 && green == 0)
	{
		i2c_write_reg(IMAX5,IMAX5Reg&0x00FF); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,LER2reg&0xFF3F); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,CTRS2reg&0xFF3F); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 1)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x2000); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x0080); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x0080); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 2)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x4000); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x0080); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x0080); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 0 && green == 3)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x6000); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x0080); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x0080); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 0)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x0200); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x0040); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x0040); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 1)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x2200); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x00C0); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x00C0); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 2)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x4200); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x00C0); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x00C0); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 1 && green == 3)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x6200); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x00C0); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x00C0); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 0)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x0400); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x0040); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x0040); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 1)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x2400); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x00C0); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x00C0); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 2)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x4400); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x00C0); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x00C0); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 2 && green == 3)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x6400); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x00C0); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x00C0); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 0)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x0600); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x0040); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x0040); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 1)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x2600); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x00C0); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x00C0); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 2)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x4600); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x00C0); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x00C0); 	// CTRS2-LED13~LED20: i2c Control
	}
	else if (red == 3 && green == 3)
	{
		i2c_write_reg(IMAX5,(IMAX5Reg&0x00FF)|0x6600); 	// IMAX5-LED17~LED20 Current
		i2c_write_reg(LER2,(LER2reg&0xFF3F)|0x00C0); 	// LER2-LED13~LED20 Enable
		i2c_write_reg(CTRS2,(CTRS2reg&0xFF3F)|0x00C0); 	// CTRS2-LED13~LED20: i2c Control
	}
}

static int aw9120_led_work(unsigned int ledID, unsigned int red, unsigned int green, unsigned int blue)
{
	int val = 0;

	//Disable LED Module
	unsigned int reg;
	reg = i2c_read_reg(GCR);
	reg &= 0xFFFE;
	i2c_write_reg(GCR, reg); 		// GCR-Disable LED Module
	
	switch (ledID)
	{
		case 1:				//LED1、LED2、LED3
			aw9120_led1(red, green, blue);
			break;
		case 2:				//LED13、LED14、LED15
			aw9120_led5(red, green, blue);
			break;
		case 3:				//LED4、LED5、LED6
			aw9120_led2(red, green, blue);
			break;
		case 4:				//LED7、LED8、LED9
			aw9120_led3(red, green, blue);
			break;
		case 5:				//LED10、LED11、LED12
			aw9120_led4(red, green, blue);
			break;
		case 6:				//LED10、LED11、LED12
			aw9120_led6(red, green, blue);
			break;
		case 7:				//LED10、LED11、LED12
			aw9120_led7(red, green, blue);
			break;	
		default:
			printk("[aw9120]:cmd error!\n");
			break;
	}

	//Enable LED Module
	reg |= 0x0001;
	i2c_write_reg(GCR,reg); 		// GCR-Enable LED Module

	i2c_write_reg(CMDR,0xBFFF); 	// CMDR-LED1~LED20 PWM=0xFF
	
	return val;
}

/*************************************proc start***********************************************/
static ssize_t aw9120_value_proc_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	char str_buf[16] = {0};

  if (copy_from_user(str_buf, buff, count) ){
  	printk("copy_from_user---error\n");
    return -EFAULT;
  }

  if(str_buf[0]== '0') {// OFF
  	aw9120_led_off();
  }else if(str_buf[0]== '1') {// ON
  	aw9120_led_on();
  }else if(str_buf[0]== '2') {// Breath
  	aw9120_led_breath();
  }else {
	aw9120_led_off();
  }

  printk("%s end!\n", __func__);
  return count;
}

static const struct file_operations aw9120_value_proc_fops = { 
	.write = aw9120_value_proc_write,
};

static ssize_t aw9120_reg_proc_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
	unsigned int reg_val[1];
	ssize_t len = 0;
	unsigned char i;
	for(i=1;i<0x7F;i++) {
		reg_val[0] = i2c_read_reg(i);
		len += snprintf(buffer+len, PAGE_SIZE-len, "reg%2X = 0x%4X, ", i,reg_val[0]);
	}
	len += snprintf(buffer+len, PAGE_SIZE-len, "\n");
	return len;
}

static ssize_t aw9120_reg_proc_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned int databuf[2];
	if(2 == sscanf(buff,"%x %x",&databuf[0], &databuf[1])) {
		i2c_write_reg((u8)databuf[0],databuf[1]);
	}
	printk("%s enter, databuf[0]=0x%x, databuf[1]=0x%x\n", __func__, databuf[0], databuf[1]);
 	return count;
}

static const struct file_operations aw9120_reg_proc_fops = { 
	.read = aw9120_reg_proc_read,
	.write = aw9120_reg_proc_write,
};

static ssize_t aw9120_operation_proc_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned int databuf[4];
	if(4 == sscanf(buff,"%x %x %x %x",&databuf[0], &databuf[1], &databuf[2], &databuf[3])) {
		aw9120_led_work(databuf[0], databuf[1], databuf[2], databuf[3]);
	}
	printk("%s enter, databuf[0]=%d, databuf[1]=%d, databuf[2]=%d, databuf[3]=%d\n", __func__, databuf[0], databuf[1], databuf[2], databuf[3]);
 	return count;
}

static const struct file_operations aw9120_operation_proc_fops = { 
	.write = aw9120_operation_proc_write,
};

/*************************proc end*********************************/

//////////////////////////////////////////////////////////////////////////////////////////
// i2c driver
//////////////////////////////////////////////////////////////////////////////////////////
static int aw9120_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err = 0;
	int count =0;
	unsigned int reg_value; 
	unsigned int reg;

	printk("%s Enter\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	aw9120_i2c_client = client;
	printk("%s: i2c addr=%x\n", __func__, client->addr);

	aw9120_hw_on();

	for(count = 0; count < 5; count++){
		reg_value = i2c_read_reg(0x00);				//read chip ID
		printk("%s: aw9120 chip ID = 0x%4x\n", __func__, reg_value);
		if (reg_value == 0xb223)
			break;
		msleep(5);
		if(count == 4) {
			pr_err("msg %s read id error\n", __func__);
			err = -ENODEV;
			goto exit_create_singlethread;
		}
	}

	proc_create("aw9120_value", 0777, NULL, &aw9120_value_proc_fops);
    proc_create("aw9120_reg", 0777, NULL, &aw9120_reg_proc_fops);
	proc_create("aw9120_operation", 0777, NULL, &aw9120_operation_proc_fops);

	//add by wangys 开机打开大小写led蓝灯	
	reg = i2c_read_reg(GCR);
	reg &= 0xFFFE;
	i2c_write_reg(GCR, reg); 		// GCR-Disable LED Module

	aw9120_led7(0, 1, 0);

	//Enable LED Module
	reg |= 0x0001;
	i2c_write_reg(GCR,reg); 		// GCR-Enable LED Module
	i2c_write_reg(CMDR,0xBFFF); 	// CMDR-LED1~LED20 PWM=0xFF
    //end
    
	printk("%s Over\n", __func__);
	return 0;

exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	aw9120_i2c_client = NULL;
exit_check_functionality_failed:
	return err;
}

static int aw9120_i2c_remove(struct i2c_client *client)
{
	printk("%s enter\n", __func__);

	i2c_set_clientdata(client, NULL);
	aw9120_i2c_client = NULL;

	return 0;
}

static const struct i2c_device_id aw9120_i2c_id[] = {
	{ AW9120_LED_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, aw9120_ts_id);


#ifdef CONFIG_OF
static const struct of_device_id aw9120_i2c_of_match[] = {
	{.compatible = "mediatek,aw9120_led"},
	{},
};
#endif
static struct i2c_driver aw9120_i2c_driver = {
	.probe		= aw9120_i2c_probe,
	.remove		= aw9120_i2c_remove,
	.id_table	= aw9120_i2c_id,
	.driver	= {
		.name	= AW9120_LED_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = aw9120_i2c_of_match,
#endif	
	},
};

//////////////////////////////////////////////////////////////////////////////////////////
// platform driver
//////////////////////////////////////////////////////////////////////////////////////////
static int aw9120_led_remove(struct platform_device *pdev)
{
	printk("%s start!\n", __func__);
	i2c_del_driver(&aw9120_i2c_driver);
	return 0;
}

static int aw9120_led_probe(struct platform_device *pdev)
{
	int ret;

	printk("%s start!\n", __func__);
	
	ret = aw9120_gpio_init(pdev);
	if (ret != 0) {
		printk("[%s] failed to init aw9120 pinctrl.\n", __func__);
		return ret;
	} else {
		printk("[%s] Success to init aw9120 pinctrl.\n", __func__);
	}
	
	ret = i2c_add_driver(&aw9120_i2c_driver);
	if (ret != 0) {
		printk("[%s] failed to register aw9120 i2c driver.\n", __func__);
		return ret;
	} else {
		printk("[%s] Success to register aw9120 i2c driver.\n", __func__);
	}

	printk("%s exit!\n", __func__);
	
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw9120_led_of_match[] = {
	{.compatible = "mediatek,aw9120_led"},
	{},
};
#endif

static struct platform_driver aw9120_led_driver = {
	.probe	 = aw9120_led_probe,
	.remove	 = aw9120_led_remove,
	.driver = {
		.name   = "aw9120_led",
#ifdef CONFIG_OF
		.of_match_table = aw9120_led_of_match,
#endif
	}
};

static int __init aw9120_led_init(void)
{
	int ret;
	printk("%s Enter\n", __func__);

	ret = platform_driver_register(&aw9120_led_driver);
	if (ret) {
		printk("****[%s] Unable to register driver (%d)\n", __func__, ret);
		return ret;
	}

	printk("%s Exit\n", __func__);

	return ret;
}

static void __exit aw9120_led_exit(void)
{
	printk("%s Enter\n", __func__);
	platform_driver_unregister(&aw9120_led_driver);
	printk("%s Exit\n", __func__);
}

module_init(aw9120_led_init);
module_exit(aw9120_led_exit);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC AW9120 LED Driver");
MODULE_LICENSE("GPL v2");
