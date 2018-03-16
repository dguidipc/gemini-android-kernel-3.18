/**************************************************************************
*  aw9523_key.c
* 
*  Create Date : 
* 
*  Modify Date : 
*
*  Create by   : AWINIC Technology CO., LTD
*
*  Version     : 0.9, 2016/02/15
**************************************************************************/

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/dma-mapping.h>
#include <linux/gameport.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/miscdevice.h>
#include <linux/workqueue.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define AW9523_I2C_NAME		"AW9523_KEY" 
//reg list
#define P0_INPUT    0x00
#define P1_INPUT    0x01
#define P0_OUTPUT   0x02
#define P1_OUTPUT   0x03
#define P0_CONFIG   0x04
#define P1_CONFIG   0x05
#define P0_INT      0x06
#define P1_INT      0x07
#define ID_REG      0x10
#define CTL_REG     0x11
#define P0_LED_MODE 0x12
#define P1_LED_MODE 0x13
#define P1_0_DIM0   0x20
#define P1_1_DIM0   0x21
#define P1_2_DIM0   0x22
#define P1_3_DIM0   0x23
#define P0_0_DIM0   0x24
#define P0_1_DIM0   0x25
#define P0_2_DIM0   0x26
#define P0_3_DIM0   0x27
#define P0_4_DIM0   0x28
#define P0_5_DIM0   0x29
#define P0_6_DIM0   0x2A
#define P0_7_DIM0   0x2B
#define P1_4_DIM0   0x2C
#define P1_5_DIM0   0x2D
#define P1_6_DIM0   0x2E
#define P1_7_DIM0   0x2F
#define SW_RSTN     0x7F


#define KEY_P0_MASK		0x3F		// P0 Input
#define KEY_P1_MASK		0x3F		// P1 Output

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct aw9523_key_map {
	unsigned char row;
	unsigned char col;
	unsigned int name;
};

static struct aw9523_key_map aw9523_key_name[] = {
//    row      col           name
	{0,		0,		KEY_F1},
	{0,		1,		KEY_F2},
	{0,		2,		KEY_F3},
	{0,		3,		KEY_F4},
	{1,		0,		KEY_F5},
	{1,		1,		KEY_F6},
	{1,		2,		KEY_F7},
	{1,		3,		KEY_F8},
	{2,		0,		KEY_F9},
	{2,		1,		KEY_F10},
	{2,		2,		KEY_F11},
	{2,		3,		KEY_F12},
	{3,		0,		KEY_F13},
	{3,		1,		KEY_F14},
	{3,		2,		KEY_F15},
	{3,		3,		KEY_F16},
};

unsigned int aw9523_key_num = 0;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static unsigned char i2c_write_reg(unsigned char addr, unsigned char reg_data);
static unsigned char i2c_read_reg(unsigned char addr);

static ssize_t aw9523_get_reg(struct device* cd,struct device_attribute *attr, char* buf);
static ssize_t aw9523_set_reg(struct device* cd, struct device_attribute *attr,const char* buf, size_t len);

static DEVICE_ATTR(reg, 0660, aw9523_get_reg,  aw9523_set_reg);


struct aw9523_key_data {
	struct input_dev	*input_dev;
	struct work_struct 	eint_work;
	struct device_node *irq_node;
	int irq;
};

struct aw9523_pinctrl {
	struct pinctrl *pinctrl;
	struct pinctrl_state *shdn_high;
	struct pinctrl_state *shdn_low;
	struct pinctrl_state *int_pin;
};

//struct aw9523_pinctrl *aw9523_pin;
struct pinctrl *aw9523_pin;
struct pinctrl_state *shdn_high;
struct pinctrl_state *shdn_low;
struct pinctrl_state *int_pin;
struct aw9523_key_data *aw9523_key;
struct i2c_client *aw9523_i2c_client;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// GPIO Control
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int aw9523_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;
	aw9523_pin = devm_pinctrl_get(&pdev->dev);
    printk("%s : pinctrl init 00000000\n", __func__);
	if (IS_ERR(aw9523_pin)) {
		dev_err(&pdev->dev, "Cannot find aw9523 pinctrl!");
		ret = PTR_ERR(aw9523_pin);
		printk("%s devm_pinctrl_get fail!\n", __func__);
	}
	
    printk("%s : pinctrl init 11111111\n", __func__);
	shdn_high = pinctrl_lookup_state(aw9523_pin, "aw9523_shdn_high");
	if (IS_ERR(shdn_high)) {
		ret = PTR_ERR(shdn_high);
		printk("%s : pinctrl err, aw9523_shdn_high\n", __func__);
	}

	shdn_low = pinctrl_lookup_state(aw9523_pin, "aw9523_shdn_low");
	if (IS_ERR(shdn_low)) {
		ret = PTR_ERR(shdn_low);
		printk("%s : pinctrl err, aw9523_shdn_low\n", __func__);
	}

	return ret;
}

static void aw9523_hw_reset(void)
{
	printk("%s enter\n", __func__);
	pinctrl_select_state(aw9523_pin, shdn_low);
	msleep(5);
	pinctrl_select_state(aw9523_pin, shdn_high);
	msleep(5);
	printk("%s out\n", __func__);
}

#if 0
static void aw9523_hw_off(void)
{
	printk("%s enter\n", __func__);
	pinctrl_select_state(aw9523_pin, shdn_low);
	msleep(5);
	printk("%s out\n", __func__);
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Interrupt
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void aw9523_key_eint_work(struct work_struct *work)
{
	unsigned char reg = 0xff;
	unsigned char row = 0;
	unsigned char col = 0;
	unsigned char cnt = 0;
	unsigned char mask = 0;

  //unsigned int name;
	
	printk("%s enter\n", __func__);

	reg = i2c_read_reg(P0_INPUT);	// clear P0 Input Interrupt

	i2c_write_reg(P0_INT, KEY_P0_MASK);	// disbable p0 interrupt

	reg &= KEY_P0_MASK;			// MASK Unused Pin
	if(reg != KEY_P0_MASK) {		// Key Press
		cnt = 0;
		row = 0;
		reg = (~reg) & KEY_P0_MASK;

		// get row number
		while(reg) {
			if(reg & 0x01) {
				cnt ++;
			}
			reg = reg>>1;
			row ++;
		}
		if(cnt > 1) {
			//return;		// one more key press 
		}

		// get col number
		cnt = 0;
		col = 0;
		mask = KEY_P1_MASK;
		for(cnt=0; cnt<8; cnt++) {
			if(mask & 0x01) {
				reg = i2c_read_reg(P1_OUTPUT);
				reg &= (~KEY_P1_MASK);
				reg &= (~(1<<cnt));
				i2c_write_reg(P1_OUTPUT, reg);		// Set P1 Every Bit Output 0
				reg = i2c_read_reg(P0_INPUT);		// Get P0 Status
				reg &= KEY_P0_MASK;
				if(reg != KEY_P0_MASK) {
					col = cnt;
				}
			} else {
			}
			mask = mask >> 1;
		}
	

		// get key
		for(cnt=0; cnt<sizeof(aw9523_key_name); cnt++) {
			if((aw9523_key_name[cnt].col == col) && (aw9523_key_name[cnt].row == row)) {
				aw9523_key_num = cnt;
				break;
			}
		}

		// key report
		input_report_key(aw9523_key->input_dev, aw9523_key_name[aw9523_key_num].name, 1);
		input_sync(aw9523_key->input_dev);
	} else {
		// key report
		input_report_key(aw9523_key->input_dev, aw9523_key_name[aw9523_key_num].name, 0);
		input_sync(aw9523_key->input_dev);
	}
	
	i2c_write_reg(P1_OUTPUT, 0x00|(~KEY_P1_MASK));	// P1: 0000 0000
	i2c_write_reg(P0_INT, (~KEY_P0_MASK));	// enable p0 interrupt
	enable_irq(aw9523_key->irq);

	printk("%s exit\n", __func__);

}


static irqreturn_t aw9523_key_eint_func(int irq, void *desc)
{	
	disable_irq_nosync(aw9523_key->irq);
	printk("%s Enter\n", __func__);
	if(aw9523_key == NULL){
		printk("aw9523_key == NULL");
		return  IRQ_NONE;
	}

	schedule_work(&aw9523_key->eint_work);

	return IRQ_HANDLED;

}
int aw9523_key_setup_eint(void)
{
	int ret = 0;
	u32 ints[2] = {0, 0};

	int_pin = pinctrl_lookup_state(aw9523_pin, "aw9523_int_pin");
	if (IS_ERR(int_pin)) {
		ret = PTR_ERR(int_pin);
		pr_debug("%s : pinctrl err, aw9523_int_pin\n", __func__);
	}

	aw9523_key->irq_node = of_find_compatible_node(NULL, NULL, "mediatek,aw9523-eint");

	if (aw9523_key->irq_node) {
		of_property_read_u32_array(aw9523_key->irq_node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);
		pinctrl_select_state(aw9523_pin, int_pin);
		printk("%s ints[0] = %d, ints[1] = %d!!\n", __func__, ints[0], ints[1]);

		aw9523_key->irq = irq_of_parse_and_map(aw9523_key->irq_node, 0);
		printk("%s irq = %d\n", __func__, aw9523_key->irq);
		if (!aw9523_key->irq) {
			printk("%s irq_of_parse_and_map fail!!\n", __func__);
			return -EINVAL;
		}
		if (request_irq(aw9523_key->irq, aw9523_key_eint_func, IRQ_TYPE_LEVEL_LOW, "aw9523-eint", NULL)) {
			printk("%s IRQ LINE NOT AVAILABLE!!\n", __func__);
			return -EINVAL;
		}
	} else {
		printk("null irq node!!\n");
		return -EINVAL;
	}

    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// i2c write and read
//////////////////////////////////////////////////////////////////////////
////////////////////////////////////
static unsigned char i2c_write_reg(unsigned char addr, unsigned char reg_data)
{
	char ret;
	u8 wdbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= aw9523_i2c_client->addr,
			.flags	= 0,
			.len	= 2,
			.buf	= wdbuf,
		},
	};

	wdbuf[0] = addr;
	wdbuf[1] = reg_data;

	ret = i2c_transfer(aw9523_i2c_client->adapter, msgs, 1);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return ret;
}

static unsigned char i2c_read_reg(unsigned char addr)
{
	unsigned char ret;
	u8 rdbuf[512] = {0};

	struct i2c_msg msgs[] = {
		{
			.addr	= aw9523_i2c_client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rdbuf,
		},
		{
			.addr	= aw9523_i2c_client->addr,
			.flags	= I2C_M_RD,
			.len	= 1,
			.buf	= rdbuf,
		},
	};

	rdbuf[0] = addr;
	
	ret = i2c_transfer(aw9523_i2c_client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);

    return rdbuf[0];
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// aw9523 init
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void aw9523_init_keycfg(void)
{
	i2c_write_reg(SW_RSTN, 0x00);		// Software Reset
	
//	i2c_write_reg(P0_MODE, 0xFF&KEY_P0_MASK);		// P0: GPIO Mode
//	i2c_write_reg(P1_MODE, 0xFF&KEY_P1_MASK);		// P1: GPIO Mode

	i2c_write_reg(P0_CONFIG, 0xFF&KEY_P0_MASK);		// P0: Input Mode
	i2c_write_reg(P1_CONFIG, 0x00|(~KEY_P1_MASK));	// P1: Output Mode
	i2c_write_reg(P1_OUTPUT, 0x00|(~KEY_P1_MASK));	// P1: 0000 0000

	i2c_write_reg(P0_INT, 0x00|(~KEY_P0_MASK));		// P0: Enable Interrupt
	i2c_write_reg(P1_INT, 0xFF&KEY_P1_MASK);			// P1: Disable Interrupt
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Debug
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static ssize_t aw9523_get_reg(struct device* cd,struct device_attribute *attr, char* buf)
{
	unsigned char reg_val;
	ssize_t len = 0;
	u8 i;
	for(i=0;i<0x30;i++)
	{
		reg_val = i2c_read_reg(i);
		len += snprintf(buf+len, PAGE_SIZE-len, "reg%2X = 0x%2X, ", i,reg_val);
	}

	return len;
}

static ssize_t aw9523_set_reg(struct device* cd, struct device_attribute *attr, const char* buf, size_t len)
{
	unsigned int databuf[2];
	if(2 == sscanf(buf,"%x %x",&databuf[0], &databuf[1]))
	{
		i2c_write_reg(databuf[0],databuf[1]);
	}
	return len;
}

static int aw9523_create_sysfs(struct i2c_client *client)
{
	int err;
	struct device *dev = &(client->dev);

	err = device_create_file(dev, &dev_attr_reg);

	return err;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void aw9523_input_register(void)
{
	int err;
	unsigned int i = 0;
	struct input_dev *input_dev;
	struct aw9523_key_map *aw9523_key_map;

	input_dev = input_allocate_device();
	if (!input_dev){
		err = -ENOMEM;
//		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	aw9523_key->input_dev = input_dev;
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_SYN, input_dev->evbit);

	for(i=0; i<sizeof(aw9523_key_name)/sizeof(aw9523_key_map); i++) {
		__set_bit(aw9523_key_name[i].name, input_dev->keybit);
	}

	input_dev->name	= AW9523_I2C_NAME;
	err = input_register_device(input_dev);
	if (err) {
//		dev_err(&client->dev,
//		"aw9523_i2c_probe: failed to register input device: %s\n",
//		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}	
	
exit_input_dev_alloc_failed:
	cancel_work_sync(&aw9523_key->eint_work);
exit_input_register_device_failed:
	input_free_device(input_dev);
}


static int aw9523_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	unsigned char reg_value = 0;
	int err = 0;
	unsigned char cnt = 5;
	printk("%s start\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
	printk("%s: kzalloc\n", __func__);
	aw9523_key = kzalloc(sizeof(*aw9523_key), GFP_KERNEL);
	if (!aw9523_key)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
/*
	aw9523_pin = kzalloc(sizeof(*aw9523_pin), GFP_KERNEL);
	if (!aw9523_pin)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
*/
	aw9523_i2c_client = client;
	i2c_set_clientdata(client, aw9523_key);
	
	aw9523_hw_reset();

	// CHIP ID
	while((cnt>0)&&(reg_value != 0x23)){
		reg_value = i2c_read_reg(0x10);
		printk("aw9523 chipid=0x%2x\n", reg_value);
    		cnt --;
	    	msleep(10);
		
	}
	if(!cnt){
		err = -ENODEV;
		goto exit_create_singlethread;
	}

	//Interrupt
	INIT_WORK(&aw9523_key->eint_work, aw9523_key_eint_work);
	aw9523_key_setup_eint();

	aw9523_input_register();

	
	aw9523_create_sysfs(client);

	aw9523_init_keycfg();
	
	return 0;

exit_create_singlethread:
	aw9523_i2c_client = NULL;
exit_alloc_data_failed:	
exit_check_functionality_failed:
	return err;	
}

static int aw9523_i2c_remove(struct i2c_client *client)
{
	struct aw9523_key_data *aw9523_key = i2c_get_clientdata(client);

	printk("%s enter\n", __func__);
	
	input_unregister_device(aw9523_key->input_dev);
	
	kfree(aw9523_key);
	
	aw9523_i2c_client = NULL;
	i2c_set_clientdata(client, NULL);

	return 0;
}

static const struct i2c_device_id aw9523_i2c_id[] = {
	{ AW9523_I2C_NAME, 0 },
	{ }
};

#ifdef CONFIG_OF
static const struct of_device_id extgpio_of_match[] = {
	{.compatible = "mediatek,aw9523_key"},
	{},
};
#endif

static struct i2c_driver aw9523_i2c_driver = {
        .driver = {
                .name   = AW9523_I2C_NAME,
#ifdef CONFIG_OF
				.of_match_table = extgpio_of_match,
#endif
},

        .probe          = aw9523_i2c_probe,
        .remove         = aw9523_i2c_remove,
        .id_table       = aw9523_i2c_id,
};


static int aw9523_key_remove(struct platform_device *pdev)
{
	printk("aw9523 remove\n");
	i2c_del_driver(&aw9523_i2c_driver);
	return 0;
}

static int aw9523_key_probe(struct platform_device *pdev)
{
	int ret;

	printk("%s start!\n", __func__);

	ret = aw9523_pinctrl_init(pdev);
	if (ret != 0) {
		printk("[%s] failed to init aw9523 pinctrl.\n", __func__);
		return ret;
	} else {
		printk("[%s] Success to init aw9523 pinctrl.\n", __func__);
	}
	
	ret = i2c_add_driver(&aw9523_i2c_driver);
	if (ret != 0) {
		printk("[%s] failed to register aw9523 i2c driver.\n", __func__);
		return ret;
	} else {
		printk("[%s] Success to register aw9523 i2c driver.\n", __func__);
	}

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id aw9523plt_of_match[] = {
	{.compatible = "mediatek,aw9523_key"},
	{},
};
#endif

static struct platform_driver aw9523_key_driver = {
		.probe	 = aw9523_key_probe,
		.remove	 = aw9523_key_remove,
        .driver = {
                .name   = "aw9523_key",
#ifdef CONFIG_OF
				.of_match_table = aw9523plt_of_match,
#endif
        }
};

static int __init aw9523_key_init(void) {
	int ret;
	printk("%s start\n", __func__);
	
	ret = platform_driver_register(&aw9523_key_driver);
	if (ret) {
		printk("****[%s] Unable to register driver (%d)\n", __func__, ret);
		return ret;
	}		
	return 0;
}

static void __exit aw9523_key_exit(void) {
	printk("%s exit\n", __func__);
	platform_driver_unregister(&aw9523_key_driver);
}

module_init(aw9523_key_init);
module_exit(aw9523_key_exit);

MODULE_AUTHOR("<liweilei@awinic.com.cn>");
MODULE_DESCRIPTION("AWINIC aw9523 Key Driver");
MODULE_LICENSE("GPL");
