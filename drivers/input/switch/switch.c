/*
 * Copyright (C) 2010 MediaTek, Inc.
 *
 * Author: Terry Chang <terry.chang@mediatek.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include "switch.h"
#include <linux/wakelock.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/clk.h>
#include <linux/switch.h>
#include <linux/workqueue.h>
#include <linux/irqchip/mt-eic.h>

#define switch_NAME	"mtk-toggle-switch"
#define MTK_KP_WAKESOURCE	/* this is for auto set wake up source */

//#define SWITCH_AEON_DEBUG
#define SWITCH_AEON     "switch_aeon"
#if defined(SWITCH_AEON_DEBUG)
#define SWITCH_DEBUG(a, arg...)   printk(SWITCH_AEON ": " a, ##arg)
#else
#define SWITCH_DEBUG(a,arg...)	do {} while (0)
#endif

static unsigned int switch_irqnr;
unsigned int switchgpiopin, switchdebounce;
unsigned int switch_eint_type;
//struct input_dev *switch_input_dev;
static bool switch_suspend;
static char call_status;
struct wake_lock switch_suspend_lock;	/* For suspend usage */

static int switch_pdrv_probe(struct platform_device *pdev);
static int switch_pdrv_remove(struct platform_device *pdev);
#ifndef USE_EARLY_SUSPEND
static int switch_pdrv_suspend(struct platform_device *pdev, pm_message_t state);
static int switch_pdrv_resume(struct platform_device *pdev);
#endif
extern void mt_irq_set_polarity(unsigned int irq, unsigned int polarity);

//static void switch_switch_eint_handler(void);

/*zhaolong add for GPIO request*/

struct platform_device *switchPltFmDev;
struct pinctrl *switchctrl = NULL;
struct pinctrl_state *switch_pins_default;
struct pinctrl_state *switch_pins_cfg;

#if 0
int switch_gpio_init(void)
{
	int ret = 0;
	printk("zhaolong:======%s======.\n" , __func__);
	switchctrl = devm_pinctrl_get(&switchPltFmDev->dev);
	if (IS_ERR(switchctrl)) {
		ret = PTR_ERR(switchctrl);
		printk("Cannot find switch pinctrl!\n");
	}
	switch_pins_default = pinctrl_lookup_state(switchctrl, "pin_default");
	if (IS_ERR(switch_pins_default)) {
		ret = PTR_ERR(switch_pins_default);
		printk("Cannot find switch pinctrl default!\n");
	}

	switch_pins_cfg = pinctrl_lookup_state(switchctrl, "pin_cfg");
	if (IS_ERR(switch_pins_cfg)) {
		ret = PTR_ERR(switch_pins_cfg);
		printk("Cannot find switch pinctrl pin_cfg!\n");
	}
	pinctrl_select_state(switchctrl, switch_pins_cfg);
	return ret;	
}
#endif

#define SWITCH_RIGHT        (1)
#define SWITCH_LEFT       (0)

static struct switch_dev switch_data;
static struct work_struct switch_work;
static struct workqueue_struct *switch_workqueue = NULL;
static DEFINE_SPINLOCK(switch_lock);
volatile u8 new_switch = SWITCH_LEFT;
static int switch_status_flag = SWITCH_LEFT;
extern struct input_dev *kpd_accdet_dev;//kpd_accdet_dev;

static void switch_key_handler(struct work_struct *work)
{
	new_switch = gpio_get_value(switchgpiopin);
	SWITCH_DEBUG("switch_key_handler new_switch=%d , switch_status_flag=%d\n",new_switch, switch_status_flag);

	//if(switch_status_flag != new_switch)
	{
		spin_lock(&switch_lock);
		switch_status_flag = new_switch;
		spin_unlock(&switch_lock);
		
		if(switch_status_flag == SWITCH_LEFT)//left
		{
			input_report_key(kpd_accdet_dev, KEY_F9, 1);
			input_sync(kpd_accdet_dev);
			mdelay(10);
			input_report_key(kpd_accdet_dev, KEY_F9, 0);
      		input_sync(kpd_accdet_dev);
     		SWITCH_DEBUG("wys=======F9====\n");
		}
		else  // right
		{
		  input_report_key(kpd_accdet_dev, KEY_F10, 1);
	      input_sync(kpd_accdet_dev);
	      mdelay(10);
		  input_report_key(kpd_accdet_dev, KEY_F10, 0);
	      input_sync(kpd_accdet_dev);
	      SWITCH_DEBUG("wys=======F10====\n");
		}
		switch_set_state((struct switch_dev *)&switch_data, switch_status_flag);
	}

	irq_set_irq_type(switch_irqnr,new_switch ? IRQ_TYPE_LEVEL_LOW : IRQ_TYPE_LEVEL_HIGH);
	gpio_set_debounce(switchgpiopin, switchdebounce);	
	enable_irq(switch_irqnr);
}
#if 0
static void switch_switch_eint_handler(void)
{
	printk("switch_switch_eint_handler ..\n");
	queue_work(switch_workqueue, &switch_work);	
	disable_irq_nosync(switch_irqnr);
}
#else
static irqreturn_t switch_switch_eint_handler(int irq, void *dev_id)
{
	/* use _nosync to avoid deadlock */
	queue_work(switch_workqueue, &switch_work);	
	disable_irq_nosync(switch_irqnr);
	
	return IRQ_HANDLED;
}
#endif
static const struct of_device_id switch_of_match[] = {
	{.compatible = "mediatek, sw-eint"},
	{},
};

static struct platform_driver switch_pdrv = {
	.probe = switch_pdrv_probe,
	.remove = switch_pdrv_remove,
#ifndef USE_EARLY_SUSPEND
	.suspend = switch_pdrv_suspend,
	.resume = switch_pdrv_resume,
#endif
	.driver = {
		   .name = switch_NAME,
		   .owner = THIS_MODULE,
		   .of_match_table = switch_of_match,
		   },
};

static int switch_pdrv_probe(struct platform_device *pdev)
{
	int err = 0;
	int ret = 0;
	u32 ints[2] = { 0, 0 };
	u32 ints1[2] = { 0, 0 };
	struct device_node *node = NULL;

	printk("switch probe start!!!\n");
	
	switchPltFmDev = pdev;

	/* initialize and register input device (/dev/input/eventX) */
//	switch_input_dev = input_allocate_device();
//	if (!switch_input_dev) {
//		printk("input allocate device fail.\n");
//		return -ENOMEM;
//	}

	__set_bit(EV_KEY, kpd_accdet_dev->evbit);
	__set_bit(KEY_F9, kpd_accdet_dev->keybit);
	__set_bit(KEY_F10, kpd_accdet_dev->keybit);		

	switch_workqueue = create_singlethread_workqueue("switch");
	INIT_WORK(&switch_work, switch_key_handler);

	switch_status_flag = gpio_get_value(switchgpiopin);
	
	node = of_find_matching_node(node, switch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		switchgpiopin = ints[0];
		switchdebounce = ints[1];
		switch_eint_type = ints1[1];
		gpio_set_debounce(switchgpiopin, switchdebounce);
		switch_irqnr = irq_of_parse_and_map(node, 0);
		ret = request_irq(switch_irqnr, (irq_handler_t)switch_switch_eint_handler, IRQF_TRIGGER_NONE, "switch-eint", NULL);
		if (ret != 0) {
			printk("[switch]EINT IRQ LINE NOT AVAILABLE\n");
		} else {
			printk("[switch]switch set EINT finished, switch_irqnr=%d, switchgpiopin=%d, switchdebounce=%d, switch_eint_type=%d\n",
				     switch_irqnr, switchgpiopin, switchdebounce, switch_eint_type);
		}
	} else {
		printk("[switch]%s can't find compatible node\n", __func__);
	}

	printk("switch_switch_eint_handler done..\n");

	wake_lock_init(&switch_suspend_lock, WAKE_LOCK_SUSPEND, "switch wakelock");
	
	switch_data.name = "switch";
	switch_data.index = 0;
	switch_data.state = switch_status_flag;
	
	err = switch_dev_register(&switch_data);
	if(err)
	{
		printk("[Accdet]switch_dev_register returned:%d!\n", err);
//		return 1;
	}

	switch_set_state((struct switch_dev *)&switch_data, switch_status_flag);
	
	enable_irq(switch_irqnr);
	printk("====%s success=====.\n" , __func__);
	return 0;
}

/* should never be called */
static int switch_pdrv_remove(struct platform_device *pdev)
{
	return 0;
}

#ifndef USE_EARLY_SUSPEND
static int switch_pdrv_suspend(struct platform_device *pdev, pm_message_t state)
{
	switch_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("switch_early_suspend wake up source enable!! (%d)\n", switch_suspend);
	} else {
		kpd_wakeup_src_setting(0);
		printk("switch_early_suspend wake up source disable!! (%d)\n", switch_suspend);
	}
#endif
	printk("suspend!! (%d)\n", switch_suspend);
	return 0;
}

static int switch_pdrv_resume(struct platform_device *pdev)
{
	switch_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("switch_early_suspend wake up source enable!! (%d)\n", switch_suspend);
	} else {
		printk("switch_early_suspend wake up source resume!! (%d)\n", switch_suspend);
		kpd_wakeup_src_setting(1);
	}
#endif
	printk("resume!! (%d)\n", switch_suspend);
	return 0;
}
#else
#define switch_pdrv_suspend	NULL
#define switch_pdrv_resume		NULL
#endif

#ifdef USE_EARLY_SUSPEND
static void switch_early_suspend(struct early_suspend *h)
{
	switch_suspend = true;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("switch_early_suspend wake up source enable!! (%d)\n", switch_suspend);
	} else {
		/* switch_wakeup_src_setting(0); */
		printk("switch_early_suspend wake up source disable!! (%d)\n", switch_suspend);
	}
#endif
	printk("early suspend!! (%d)\n", switch_suspend);
}

static void switch_early_resume(struct early_suspend *h)
{
	switch_suspend = false;
#ifdef MTK_KP_WAKESOURCE
	if (call_status == 2) {
		printk("switch_early_resume wake up source resume!! (%d)\n", switch_suspend);
	} else {
		printk("switch_early_resume wake up source enable!! (%d)\n", switch_suspend);
		/* switch_wakeup_src_setting(1); */
	}
#endif
	printk("early resume!! (%d)\n", switch_suspend);
}

static struct early_suspend switch_early_suspend_desc = {
	//.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1,
	.suspend = switch_early_suspend,
	.resume = switch_early_resume,
};
#endif

static int __init switch_mod_init(void)
{
	int r;

	r = platform_driver_register(&switch_pdrv);
	if (r) {
		printk("register driver failed (%d)\n", r);
		return r;
	}
#ifdef USE_EARLY_SUSPEND
	register_early_suspend(&switch_early_suspend_desc);
#endif

#ifdef CONFIG_MTK_SMARTBOOK_SUPPORT
#ifdef CONFIG_HAS_SBSUSPEND
	register_sb_handler(&switch_sb_handler_desc);
#endif
#endif

	return 0;
}

/* should never be called */
static void __exit switch_mod_exit(void)
{
}

module_init(switch_mod_init);
module_exit(switch_mod_exit);
MODULE_AUTHOR("yucong.xiong <yucong.xiong@mediatek.com>");
MODULE_DESCRIPTION("MTK Keypad (toggle switch) Driver v0.4");
MODULE_LICENSE("GPL");
