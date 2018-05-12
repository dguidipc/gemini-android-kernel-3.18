#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <typec.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>

#include <linux/gpio.h>

#include "fusb301.h"

#ifndef USB_TYPE_C
#define USB_TYPE_C

#define K_EMERG	(1<<7)
#define K_QMU	(1<<7)
#define K_ALET		(1<<6)
#define K_CRIT		(1<<5)
#define K_ERR		(1<<4)
#define K_WARNIN	(1<<3)
#define K_NOTICE	(1<<2)
#define K_INFO		(1<<1)
#define K_DEBUG	(1<<0)

#define fusb_printk(level, fmt, args...) do { \
		if (debug_level & level) { \
			pr_err("[FUSB301]" fmt, ## args); \
		} \
	} while (0)

#define SKIP_TIMER

static u32 debug_level = (255);
static struct usbtypc *g_exttypec;
static struct i2c_client *typec_client;

/* /////////////////////////////////////////////////////////////////////////// */
/* Variables accessible outside of the FUSB300 state machine */
/* /////////////////////////////////////////////////////////////////////////// */
static FUSB301reg_t Registers;	/* Variable holding the current status of the FUSB300 registers */
void InitializeFUSB301(void)
{
	fusb_printk(K_DEBUG, "%s\n", __func__);
	FUSB301Read(regDeviceID, 1, &Registers.DeviceID.byte);	/* Read the device ID */
	fusb_printk(K_DEBUG, "%s DeviceID: 0x%x\n", __func__, Registers.DeviceID.byte);
	
	Registers.Mode.byte = 0x01;
	FUSB301Write(regMode, 1, &Registers.Mode.byte);
	
//	FUSB301Read(regSlice, 1, &Registers.Slice.byte);	/* Read the slice */
//	Registers.Mask.byte = 0x00;	/* Do not mask any interrupts */

	/* Clear all interrupt masks (we want to do something with them) */
//	FUSB301Write(regMask, 1, &Registers.Mask.byte);
}

/* /////////////////////////////////////////////////////////////////////////// */
/* FUSB300 I2C Routines */
/* /////////////////////////////////////////////////////////////////////////// */
/* BOOL FUSB300Write(struct usbtypc *typec, unsigned char regAddr, unsigned char length, unsigned char* data) */
BOOL FUSB301Write(unsigned char regAddr, unsigned char length, unsigned char *data)
{
	int i;

	for (i = 0; i < length; i++)
		fusb301_i2c_w_reg8(typec_client, regAddr + i, data[i]);

	return true;
}

/* BOOL FUSB300Read(struct usbtypc *typec, unsigned char regAddr, unsigned char length, unsigned char* data) */
BOOL FUSB301Read(unsigned char regAddr, unsigned char length, unsigned char *data)
{
	int i;

	for (i = 0; i < length; i++)
		data[i] = fusb301_i2c_r_reg(typec_client, regAddr + i);

	return true;
}



static DEFINE_MUTEX(typec_lock);
void fusb301_eint_work(struct work_struct *data)
{
//	struct usbtypc *typec = container_of(to_delayed_work(data), struct usbtypc, fsm_work);

	mutex_lock(&typec_lock);

	mutex_unlock(&typec_lock);
}

static irqreturn_t fusb301_eint_isr(int irqnum, void *data)
{
	int ret;
	struct usbtypc *typec = data;

	if (typec->en_irq) {
		fusb_printk(K_DEBUG, "Disable IRQ\n");
		disable_irq_nosync(irqnum);
		typec->en_irq = 0;
	}

	ret = schedule_delayed_work_on(WORK_CPU_UNBOUND, &typec->fsm_work, 0);

	return IRQ_HANDLED;
}

int fusb301_eint_init(struct usbtypc *typec)
{
	int retval = 0;
	u32 ints[2] = { 0, 0 };
	struct device_node *node;
	unsigned int debounce=0, gpiopin=0;

	node = of_find_compatible_node(NULL, NULL, "mediatek,fusb301-eint");
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		debounce = ints[1];
		gpiopin = ints[0];

		gpio_set_debounce(gpiopin, debounce);
	} else {
		fusb_printk(K_INFO, "request_irq node = NULL\n");
	}

	typec->irqnum = irq_of_parse_and_map(node, 0);
	typec->en_irq = 1;
	
	fusb_printk(K_INFO, "request_irq debounce=%d, gpiopin=%d\n", debounce, gpiopin);
	
	fusb_printk(K_INFO, "request_irq irqnum=0x%x\n", typec->irqnum);

	retval =
	    request_irq(typec->irqnum, fusb301_eint_isr, IRQF_TRIGGER_NONE, "fusb301-eint", typec);
	if (retval != 0) {
		fusb_printk(K_ERR, "request_irq fail, ret %d, irqnum %d!!!\n", retval,
			    typec->irqnum);
	}
	return retval;
}

void fusb301_i2c_w_reg8(struct i2c_client *client, u8 addr, u8 var)
{
	char buffer[2];

	buffer[0] = addr;
	buffer[1] = var;
	i2c_master_send(client, buffer, 2);
}

u8 fusb301_i2c_r_reg(struct i2c_client *client, u8 addr)
{
	u8 var;

	i2c_master_send(client, &addr, 1);
	i2c_master_recv(client, &var, 1);
	return var;
}

static int fusb301_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct usbtypc *typec;
//	unsigned char port_type;

	fusb_printk(K_INFO, "%s 0x%x\n", __func__, client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		fusb_printk(K_ERR, "fusb301 i2c functionality check fail.\n");
		return -ENODEV;
	}

	fusb_printk(K_DEBUG, "%s %s\n", __func__, client->dev.driver->name);

	if (!g_exttypec)
		g_exttypec = kzalloc(sizeof(struct usbtypc), GFP_KERNEL);

	typec = g_exttypec;

	typec->i2c_hd = client;
	typec_client = client;

	spin_lock_init(&typec->fsm_lock);
	mutex_init(&typec_lock);

	INIT_DELAYED_WORK(&typec->fsm_work, fusb301_eint_work);

//	fusb301_init_debugfs(typec);
	
	InitializeFUSB301();

//	usb_redriver_init(typec);
	fusb301_eint_init(typec);
	fusb_printk(K_INFO, "%s %x\n", __func__, fusb301_i2c_r_reg(client, 0x1));

	/*precheck status */
	/* StateMachineFUSB301(typec); */

	return 0;
}

/* /////////////////////////////////////////////////////////////////////////////// */
int register_typec0_switch_callback(struct typec_switch_data *new_driver)
{
	fusb_printk(K_INFO, "Register driver %s %d\n", new_driver->name, new_driver->type);
#if 0
	if (new_driver->type == DEVICE_TYPE) {
		g_exttypec->device_driver = new_driver;
		g_exttypec->device_driver->on = 0;
		return 0;
	}

	if (new_driver->type == HOST_TYPE) {
		g_exttypec->host_driver = new_driver;
		g_exttypec->host_driver->on = 0;
//		if (ConnState == AttachedSource)
//			trigger_driver(g_exttypec, HOST_TYPE, ENABLE, DONT_CARE);
		return 0;
	}

	return -1;
#else
        return 0;
#endif

}
EXPORT_SYMBOL_GPL(register_typec0_switch_callback);

int unregister_typec0_switch_callback(struct typec_switch_data *new_driver)
{
	fusb_printk(K_INFO, "Unregister driver %s %d\n", new_driver->name, new_driver->type);
#if 0
	if ((new_driver->type == DEVICE_TYPE) && (g_exttypec->device_driver == new_driver))
		g_exttypec->device_driver = NULL;

	if ((new_driver->type == HOST_TYPE) && (g_exttypec->host_driver == new_driver))
		g_exttypec->host_driver = NULL;
#else
	return 0;
#endif

}
EXPORT_SYMBOL_GPL(unregister_typec0_switch_callback);





#define FUSB301_NAME "FUSB301_0"

static const struct i2c_device_id usb_i2c_id[] = {
		{FUSB301_NAME, 0},
		{}
	};

#ifdef CONFIG_OF
static const struct of_device_id fusb301_of_match[] = {
		{.compatible = "mediatek,fusb301"},
		{},
	};
#endif

struct i2c_driver usb0_i2c_driver = {
	.probe = fusb301_i2c_probe,
	.driver = {
		.owner = THIS_MODULE,
		.name = FUSB301_NAME,
#ifdef CONFIG_OF
		.of_match_table = fusb301_of_match,
#endif
	},
	.id_table = usb_i2c_id,
};

static int __init fusb301_init(void)
{
	int ret = 0;

	if (i2c_add_driver(&usb0_i2c_driver) != 0) {
		fusb_printk(K_ERR, "fusb301_init initialization failed!!\n");
		ret = -1;
	} else {
		fusb_printk(K_DEBUG, "fusb301_init initialization succeed!!\n");
	}
	return ret;
}

static void __exit fusb301_exit(void)
{

}
fs_initcall(fusb301_init);
/* module_exit(fusb301_exit); */

#endif				/*USB_TYPE_C */
