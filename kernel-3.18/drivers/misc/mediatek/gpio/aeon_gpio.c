/******************************************************************************
 * aeon_gpio.c - MTKLinux GPIO Device Driver
 *
 * Copyright 2008-2009 MediaTek Co.,Ltd.
 *
 * DESCRIPTION:
 *     This file provid the other drivers GPIO relative functions
 *
 ******************************************************************************/
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <mt-plat/mt_gpio.h>
#include <linux/gpio.h>

#define AEON_TAG "[aeon_gpio]"
#define AEON_ERR(fmt, arg...)   printk(AEON_TAG fmt, ##arg)//pr_err(fmt, ##arg)

/* GPIO Pin control*/
struct platform_device *aeon_plt_dev = NULL;
struct pinctrl *aeonctrl = NULL;

#ifdef CONFIG_OF
static const struct of_device_id aeon_gpio_of_match[] = {
	{ .compatible = "mediatek,aeon_gpio", },
	{}
};
#endif

struct pinctrl_state * aeon_gpio_init(const char *name)
{
	int ret = 0;
	struct pinctrl_state *aeon_pinctrl = NULL;

/* 	if (aeonctrl == NULL)
	{
		aeonctrl = devm_pinctrl_get(&aeon_plt_dev->dev);
		if (IS_ERR(aeonctrl)) {
			aeonctrl = NULL;
			ret = PTR_ERR(aeonctrl);
			AEON_ERR("%s : Cannot find gpio pinctrl!\n", __func__);
		}
	} */

	if (aeonctrl == NULL)
	{
		AEON_ERR("%s : aeonctrl is NULL!\n", __func__);
		return NULL;
	}

	aeon_pinctrl = pinctrl_lookup_state(aeonctrl, name);
	if (IS_ERR(aeon_pinctrl)) {
		ret = PTR_ERR(aeon_pinctrl);
		AEON_ERR("%s : pinctrl err(%d), %s\n", __func__, ret, name);
		return NULL;
	}

	return aeon_pinctrl;
}

int aeon_gpio_set(const char *name)
{
	int ret = 0;
	struct pinctrl_state *aeon_pinctrl = NULL;

	aeon_pinctrl = aeon_gpio_init(name);

	if (aeon_pinctrl == NULL)
	{
		AEON_ERR("%s : aeon_pinctrl is null!\n", __func__);
		return -EIO;
	}

	pinctrl_select_state(aeonctrl, aeon_pinctrl);

	return ret;
}
EXPORT_SYMBOL_GPL(aeon_gpio_set);

unsigned int aeon_gpio_get(const char *name)
{
	u32 gpio_num[] = {0};
	unsigned int gpiopin = 0xff;
	struct pinctrl_state *aeon_pinctrl = NULL;
	struct device_node *node = NULL;

	aeon_pinctrl = aeon_gpio_init(name);
	if (aeon_pinctrl == NULL)
	{
		AEON_ERR("%s : aeon_pinctrl is null!\n", __func__);
		return -EIO;
	}
	pinctrl_select_state(aeonctrl, aeon_pinctrl);

	node = of_find_matching_node(node, aeon_gpio_of_match);
	if (node == NULL)
	{
		AEON_ERR("%s : node is null!\n", __func__);
		return -EIO;
	}
	of_property_read_u32_array(node, name, gpio_num, ARRAY_SIZE(gpio_num));

	gpiopin = gpio_num[0];
	return __gpio_get_value(gpiopin);
}
EXPORT_SYMBOL_GPL(aeon_gpio_get);

unsigned int aeon_get_gpio_pin(const char *name)
{
	u32 gpio_num[] = {0};
	unsigned int gpiopin = 0xff;
	struct device_node *node = NULL;

	node = of_find_matching_node(node, aeon_gpio_of_match);
	if (node == NULL)
	{
		AEON_ERR("%s : node is null!\n", __func__);
		return gpiopin;//return -EIO;
	}
	of_property_read_u32_array(node, name, gpio_num, ARRAY_SIZE(gpio_num));

	gpiopin = gpio_num[0];
	return gpiopin;
}
EXPORT_SYMBOL_GPL(aeon_get_gpio_pin);

unsigned int aeon_get_pwm_num(const char *name)
{
	u32 pwm_num[] = {0};
	unsigned int pwmnum = 0;
	struct device_node *node = NULL;

	node = of_find_matching_node(node, aeon_gpio_of_match);
	if (node == NULL)
	{
		AEON_ERR("%s : node is null!\n", __func__);
		return pwmnum;//return -EIO;
	}
	of_property_read_u32_array(node, name, pwm_num, ARRAY_SIZE(pwm_num));

	pwmnum = pwm_num[0];
	return pwmnum;
}
EXPORT_SYMBOL_GPL(aeon_get_pwm_num);

static int aeon_gpio_probe(struct platform_device *pdev)
{
	int ret = 0;

	aeon_plt_dev = pdev;

	aeonctrl = devm_pinctrl_get(&aeon_plt_dev->dev);
	if (IS_ERR(aeonctrl)) {
		aeonctrl = NULL;
		ret = PTR_ERR(aeonctrl);
		AEON_ERR("%s : Cannot find gpio pinctrl!\n", __func__);
	}
	aeon_gpio_set("usb_det_low");

	return ret;
}

static int aeon_gpio_remove(struct platform_device *pdev)
{
	aeonctrl = NULL;
	return 0;
}

#ifndef CONFIG_OF
static struct platform_device aeon_gpio_platform_device = {
    .name = "aeon_gpio",
    .id = 0,
    .dev = {
    }
};
#endif

static struct platform_driver aeon_gpio_driver = {
	.probe      = aeon_gpio_probe,
	.remove     = aeon_gpio_remove,
	.driver     = {
	.name   = "aeon_gpio",
	//.owner  = THIS_MODULE,
#ifdef CONFIG_OF
	.of_match_table = aeon_gpio_of_match,
#endif
	}
};

static int __init eastaeon_gpio_init(void)
{
	int ret = 0;
#ifndef CONFIG_OF
	ret = platform_device_register(&aeon_gpio_platform_device);
	if (ret) {
		AEON_ERR("%s : platform_device_register fail\n", __func__);
		return ret;
	}
#endif

	if (platform_driver_register(&aeon_gpio_driver)) {
		AEON_ERR("%s : failed to register aeon_gpio driver\n", __func__);
		return -ENODEV;
	}

	return ret;
}

static void __exit eastaeon_gpio_exit(void)
{
	platform_driver_unregister(&aeon_gpio_driver);
}

subsys_initcall(eastaeon_gpio_init);
module_exit(eastaeon_gpio_exit);

//MODULE_DESCRIPTION("eastaeon gpio driver");
//MODULE_AUTHOR("sanford lin <lin.xiufa@eastaeon.com>");
//MODULE_LICENSE("GPL");
