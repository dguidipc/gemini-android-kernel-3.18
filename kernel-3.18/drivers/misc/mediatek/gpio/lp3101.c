#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>



#define LP_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL    /* for I2C channel 0 */
#define I2C_ID_NAME "lp3101"
#define LP_ADDR 0x3E

#if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info lp3101_board_info __initdata = {I2C_BOARD_INFO(I2C_ID_NAME, LP_ADDR)};
#endif
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
		{.compatible = "mediatek,I2C_LCD_BIAS"},
		{},
};
#endif

static struct i2c_client *lp3101_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int lp3101_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lp3101_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct lp3101_dev	{	
	struct i2c_client *client;

};

static const struct i2c_device_id lp3101_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver lp3101_iic_driver = {
	.id_table	= lp3101_id,
	.probe		= lp3101_probe,
	.remove		= lp3101_remove,
	/* .detect               = mt6605_detect, */
	.driver = {
		.owner = THIS_MODULE,
		.name	= "lp3101",
#if !defined(CONFIG_MTK_LEGACY)
			.of_match_table = lcm_of_match,
#endif
	},
};

/*****************************************************************************
 * Function
 *****************************************************************************/
static int lp3101_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk("lp3101_iic_probe\n");
	printk("LP: info==>name=%s addr=0x%x\n",client->name,client->addr);
	lp3101_i2c_client  = client;		
	return 0;
}

static int lp3101_remove(struct i2c_client *client)
{
	printk("lp3101_remove\n");
	lp3101_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static int lp3101_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = lp3101_i2c_client;
	char write_data[2] = { 0 };
	if (client == NULL) {
		printk("i2c_client = NULL, skip lp3101_write_bytes\n");
		return 0;
	}
	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		printk("lp3101 write data fail !!\n");
	return ret;
}

void lp3101_poweron(void)
{
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret = 0;

	cmd = 0x00;
	data = 0x0f;
	ret=lp3101_write_bytes(cmd,data);
	if (ret < 0)
		printk("lp3101---cmd=%0x-- i2c write error-----\n",cmd);
	else
		printk("lp3101---cmd=%0x-- i2c write success-----\n",cmd);

	cmd = 0x01;
	data = 0x0f;
	ret=lp3101_write_bytes(cmd,data);
	if (ret < 0)
		printk("lp3101----cmd=%0x--i2c write error----\n", cmd);
	else
		printk("lp3101----cmd=%0x--i2c write success----\n", cmd);

}
EXPORT_SYMBOL(lp3101_poweron);

static int __init lp3101_iic_init(void)
{
	printk( "lp3101_iic_init\n");
#if defined(CONFIG_MTK_LEGACY)
	i2c_register_board_info(LP_I2C_BUSNUM, &lp3101_board_info, 1);
#endif
	printk( "lp3101_iic_init2\n");
	i2c_add_driver(&lp3101_iic_driver);
	printk( "lp3101_iic_init success\n");	
	return 0;
}

static void __exit lp3101_iic_exit(void)
{
  printk( "lp3101_iic_exit\n");
  i2c_del_driver(&lp3101_iic_driver);  
}

module_init(lp3101_iic_init);
module_exit(lp3101_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK lp3101 I2C Driver");
MODULE_LICENSE("GPL");
