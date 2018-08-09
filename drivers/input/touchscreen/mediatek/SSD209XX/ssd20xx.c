/*
 * Copyright 2017 Solomon Systech Ltd. All rights reserved.
 *
 * SSL SSD20xx Touch device driver
 *
 * Date: 2017.04.19
 */
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/irq.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>
#include <asm/uaccess.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>

#include "ssd20xx.h"
#include "tpd.h"

#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h> //add for IRQF_***

#ifdef SUPPORT_MTK_ESD_RECOVERY
extern int primary_display_suspend(void);
extern int primary_display_resume(void);
extern int primary_display_esd_recovery(void);
#endif
struct i2c_client *i2c_client = NULL;
unsigned int tpd_rst_gpio_number = 0;
unsigned int tpd_int_gpio_number = 1;
unsigned int touch_irq = 0;
unsigned tpd_intr_type = 0;

#ifdef CONFIG_MTK_I2C_EXTENSION
#if 0 //I2C_DMA_SUPPORT
#include <linux/dma-mapping.h>

uint8_t *gpDMABuf_va = NULL;
dma_addr_t gpDMABuf_pa = 0;
uint8_t *wrDMABuf_va = NULL;
dma_addr_t wrDMABuf_pa = 0;
#endif
#endif

int init_ds_flag = 0;
int init_tmc_flag = 0;
#if defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE)
int found_force_bin_file = 0;
#endif
#if (defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE) || defined(SUPPORT_BOOTUP_FW_UPGRADE_BINFILE))
static void solomon_fw_update_controller(const struct firmware *fw, void *context);
#endif

enum _ts_work_procedure {
	TS_NO_WORK = 0,
	TS_NORMAL_WORK,
	TS_ESD_TIMER_WORK,
	TS_IN_EALRY_SUSPEND,
	TS_IN_SUSPEND,
	TS_IN_RESUME,
	TS_IN_LATE_RESUME,
	TS_IN_UPGRADE,
	TS_REMOVE_WORK,
	TS_SET_MODE,
	TS_GET_RAW_DATA,
	TS_IN_INITIALIZE,
};

static int solomon_send_key_du(struct input_dev *input, unsigned int key);
static int solomon_report_release(struct solomon_device *);
static int solomon_init_config(struct solomon_device *ftdev);
static int solomon_pre_init(struct solomon_device *ftdev);
static int solomon_init(struct solomon_device *ftdev);
static int solomon_power_control(struct solomon_device *ftdev, u8 mode);
static int solomon_fw_update(struct solomon_device *ftdev);
static int solomon_hw_init(void);
static void solomon_hw_deint(void);

struct _raw_ioctl {
	int	sz;
	u8	*buf;
};

struct _reg_ioctl {
	int	addr;
	int	*val;
};

struct _down_ioctl {
	int	sz;
	char *file;
};

struct _mp_ioctl {
	int mode;
	int count;
	int calldelay;
	int needback;
};

#ifdef SUPPORT_GESTURE_DEMO
int m_gesture_value;
#endif	/* SUPPORT_GESTURE_DEMO */

static u32 m_key_map[MAX_DEVICE_KEYMAP_SIZE] = {
	KEY_BACK, KEY_MENU, KEY_HOMEPAGE, KEY_HOMEPAGE,
	KEY_BACK, KEY_BACK, KEY_BACK, KEY_BACK,
	KEY_BACK, KEY_BACK, KEY_BACK, KEY_BACK,
	KEY_BACK, KEY_BACK, KEY_BACK, KEY_BACK,
};
#ifdef SUPPORT_KEY_BUTTON
static int lpm_gesture_keys[] = {
	KEY_POWER, KEY_BACK, KEY_WAKEUP, KEY_MENU, KEY_HOMEPAGE
};

#define LPM_GESTURE_KEY_CNT	(sizeof(lpm_gesture_keys) /	\
		sizeof(lpm_gesture_keys[0]))
#endif
#ifdef SUPPORT_LPM
#define POWER_STATUS_NM		0x8000
#define POWER_STATUS_LPM	0x4000
#define LPM_RESUME		(POWER_STATUS_NM | 0x0001)
#define LPM_SUSPEND		(POWER_STATUS_LPM | 0x0001)
#define LPM_SUSPEND_DELAY	(POWER_STATUS_LPM | 0x0002)

volatile int m_power_status = LPM_RESUME;	/* 0:resume , 1:suspend */

static int solomon_read_points(struct solomon_device *ftdev,
	struct solomon_data *data);
#endif	/* SUPPORT_LPM */

int m_mp_total_count = -1;
int m_mp_cur_count;
int m_mp_skip_count;
int m_mp_calldealy;
int m_mp_needback;

unsigned long m_point_skip_time;

struct solomon_device *misc_dev;

static struct class *touchscreen_class;
u8 m_up_event[SOLOMON_MAX_POINT] = {0,};

/* rawdata queue process */
static char *get_rear_queue_buff(struct solomon_data *ftdata)
{
	char *ptr = NULL;

	if ((ftdata->queue_rear + 1) % SOLOMON_MAX_RAWDATA_QUEUE ==
			ftdata->queue_front)
		return NULL;

	ptr = (char *)&(ftdata->rawData[(ftdata->queue_rear + 1) %
			SOLOMON_MAX_RAWDATA_QUEUE][0]);
	return ptr;
}

static int put_queue(struct solomon_data *ftdata)
{
	if ((ftdata->queue_rear + 1) % SOLOMON_MAX_RAWDATA_QUEUE ==
			ftdata->queue_front)
		return -1;

	ftdata->queue_rear = (ftdata->queue_rear + 1) %
		SOLOMON_MAX_RAWDATA_QUEUE;

	return 0;
}

static u8 *get_front_queue_buff(struct solomon_data *ftdata)
{
	int tmp = ftdata->queue_front;

	if (ftdata->queue_front == ftdata->queue_rear)
		return NULL;

	tmp = (ftdata->queue_front + 1) % SOLOMON_MAX_RAWDATA_QUEUE;

	return (u8 *)&(ftdata->rawData[tmp][0]);
}

static int get_queue(struct solomon_data *ftdata)
{
	if (ftdata->queue_front == ftdata->queue_rear)
		return -1;

	ftdata->queue_front = (ftdata->queue_front + 1) %
		SOLOMON_MAX_RAWDATA_QUEUE;

	return 0;
}

/**************************************************************************/
// irq Interface
/**************************************************************************/
/**
 * ts_enable_irq - enable irq function.
 *
 */
void ts_enable_irq(void)
{
	GTP_GPIO_AS_INT(GTP_INT_PORT);
	enable_irq(misc_dev->irq);
}

/**
 * ts_irq_disable - disable irq function.
 *
 */
void ts_disable_irq(void)
{
	disable_irq(misc_dev->irq);
}

#ifdef SUPPORT_ESD_CHECKSUM
/* It calculate the checksum for ESD */
static int esd_checksum(u8 *data, int length, u8 *xor, u8 *sum)
{
	int ret = -1;
	int i = 0;

	if (data == NULL) {
		SOLOMON_WARNNING("data is null!!!");
		goto out;
	}

	if (length < 1) {
		SOLOMON_WARNNING("length(%d) is smaller than 1!!", length);
		goto out;
	}

	if (xor == NULL || sum == NULL) {
		SOLOMON_WARNNING(" xor or sum is null!!");
		goto out;
	}

	*xor = 0;
	*sum = 0;

	for (i = 0; i < length; i++) {
		*xor ^= data[i];
		*sum += data[i];
	}

	ret = 0;
out:
	return ret;
}
#endif	/* SUPPORT_ESD_CHECKSUM */


#if 0 //I2C_DMA_SUPPORT
int32_t i2c_read_bytes_dma(struct i2c_client *client, u16 addr, uint8_t offset, uint8_t *rxbuf, uint16_t len)
{
	uint8_t buf[2] = {offset,0};
	int32_t ret;
	int32_t retries = 0;

	struct i2c_msg msg[2] = {
		{
			.addr = (addr & I2C_MASK_FLAG),
			.flags = 0,
			.buf = buf,
			.len = 2,
			.timing = client->timing
		},
		{
			.addr = (addr & I2C_MASK_FLAG),
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = (uint8_t*)gpDMABuf_pa,
			.len = len,
			.timing = client->timing
		},
	};

	if (rxbuf == NULL) {
		SOLOMON_WARNNING("rxbuf is NULL!\n");
		return -ENOMEM;
	}

	for (retries = 0; retries < 20; ++retries) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0) {
			continue;
		}
		memcpy(rxbuf, gpDMABuf_va, len);
		return ret;
	}

	SOLOMON_WARNNING("Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", offset, len, ret);
	return ret;
}

int32_t i2c_read_bytes_dma_ex(struct i2c_client *client, u16 addr, uint8_t *offset, uint8_t count, uint8_t *rxbuf, uint16_t len)
{
	//uint8_t buf[2] = {offset,0};
	int32_t ret;
	int32_t retries = 0;

	struct i2c_msg msg[2] = {
		{
			.addr = (addr & I2C_MASK_FLAG),
			.flags = 0,
			.buf = offset,
			.len = count,
			.timing = client->timing
		},
		{
			.addr = (addr & I2C_MASK_FLAG),
			.ext_flag = (client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = (uint8_t*)gpDMABuf_pa,
			.len = len,
			.timing = client->timing
		},
	};

	if (rxbuf == NULL) {
		SOLOMON_WARNNING("rxbuf is NULL!\n");
		return -ENOMEM;
	}

	for (retries = 0; retries < 20; ++retries) {
		ret = i2c_transfer(client->adapter, &msg[0], 2);
		if (ret < 0) {
			continue;
		}
		memcpy(rxbuf, gpDMABuf_va, len);
		return ret;
	}

	SOLOMON_WARNNING("Dma I2C Read Error: 0x%04X, %d byte(s), err-code: %d", offset[0], len, ret);
	return ret;
}
#endif

/* I2C read function. */
int ts_read_data(struct i2c_client *client, u16 reg, u8 *values, u16 length)
{
	s32 ret;

#if 0 //I2C_DMA_SUPPORT
	uint8_t regAddr[2] = {(reg&0xff), (reg >> 8)&0xff};

	ret = i2c_read_bytes_dma_ex(client, SOLOMON_I2C_ADDR, regAddr, 2, values, length);

	return ret;
#else
	/* select register*/
	ret = i2c_master_send(client, (u8 *)&reg, 2);
	if (ret < 0) {
		SOLOMON_WARNNING("I2C READ CMD FAIL");
		return ret;
	}

	/* for setup tx transaction. */
	udelay(DELAY_FOR_TRANSCATION);
	ret = i2c_master_recv(client, values, length);

	if (ret < 0) {
		SOLOMON_WARNNING("I2C READ DATA FAIL (%d) (0x%04X)", ret, reg);
		return ret;
	}

	udelay(DELAY_FOR_POST_TRANSCATION);

	return length;
#endif
}

/* I2C read function. It support several byte for the write. */
int ts_read_data_ex(struct i2c_client *client, u8 *reg, u16 regLen,
		u8 *values, u16 length)
{
	s32 ret;

#if 0 //I2C_DMA_SUPPORT
	ret = i2c_read_bytes_dma_ex(client, SOLOMON_I2C_ADDR, reg, regLen, values, length);

	return ret;
#else
	/* select register */
	ret = i2c_master_send(client, reg, regLen);
	if (ret < 0) {
		SOLOMON_WARNNING("I2C READ CMD FAIL");
		return ret;
	}

	/* for setup tx transaction. */
	udelay(DELAY_FOR_TRANSCATION);
	ret = i2c_master_recv(client, values, length);

	if (ret < 0) {
		SOLOMON_WARNNING("I2C READ DATA FAIL (%d)", ret);
		return ret;
	}

	udelay(10);	/*DELAY_FOR_POST_TRANSCATION*/

	return length;
#endif
}

/* I2C write function. */
int ts_write_data(struct i2c_client *client, u16 reg, u8 *values, u16 length)
{
	s32 ret;
	s32 retry = 2;
	u8 pkt[64];				/* max packet */

	memset(pkt, 0, 64);

	pkt[0] = (reg)&0xff;			/* reg addr */
	pkt[1] = (reg >> 8)&0xff;
	memcpy((u8 *)&pkt[2], values, length);

again:
	ret = i2c_master_send(client, pkt, length + 2);
	if (ret < 0) {
		SOLOMON_WARNNING("I2C WRITE FAIL : 0x%x", reg);
		if ((retry--) > 0)
			goto again;

		return ret;
	}

	udelay(DELAY_FOR_POST_TRANSCATION);

	return length;
}

#ifdef SUPPORT_TMC_I2C_LENGTH
static inline s32 ts_read_tmc_i2c(struct i2c_client *client, u16 *i2c_len)
{
	int err = 0;

	err = ts_read_data(client, SOLOMON_TMC_I2C_LENGTH, (u8 *)(i2c_len), 2);

	if (err < 0) {
		SOLOMON_WARNNING("error : read TMC I2C Length");
		return -EAGAIN;
	}

	SOLOMON_DEBUG("read TMC I2C Length : 0x%04x", *i2c_len);
	return err;
}

static inline s32 ts_write_tmc_i2c(struct i2c_client *client, u16 i2c_len)
{
	int err = 0;

	err = ts_write_data(client, SOLOMON_TMC_I2C_LENGTH,
			(u8 *)(&i2c_len), 2);

	if (err < 0) {
		SOLOMON_WARNNING("error : write TMC I2C Length");
		return -EAGAIN;
	}

	return err;
}
#endif	/* SUPPORT_TMC_I2C_LENGTH */

/* get gesture */
static inline s32 ts_read_gesture(struct i2c_client *client,
		u16 *gesture, u16 length)
{
	int err = 0;

	err = ts_read_data(client,
			SOLOMON_GET_GESTURE, (u8 *)(gesture), length);

	if (err < 0) {
		SOLOMON_WARNNING("error : read Gesture");
		return -EAGAIN;
	}

	SOLOMON_DEBUG("read gesture : 0x%04x", *gesture);
	return err;
}
#ifdef SUPPORT_GESTURE_COORDINATE
static int solomon_read_gesture_coordinate(struct i2c_client *client,
		u16 length, u16 *coordinate)
{
	int err = 0;

	if (length == 0 || coordinate == NULL)
		return 0;

	err = ts_read_data(client,
			SOLOMON_GET_GESTURE_COORDINATE, (u8 *)(coordinate), length);

	if (err < 0) {
		SOLOMON_WARNNING("error : read Gesture coordinate!!");
		return -EAGAIN;
	}

	return err;
}
#endif	/* SUPPORT_GESTURE_COORDINATE */

/* get key touch */
static inline s32 ts_read_keydata(struct i2c_client *client, u8 *keydata)
{
	int err = 0;

	err = ts_read_data(client, SOLOMON_GET_KEYDATA, (u8 *)(keydata), 2);

	if (err < 0) {
		SOLOMON_WARNNING("error : read key data");
		return -EAGAIN;
	}

	SOLOMON_DEBUG("read key data : 0x%02x%02x", keydata[1], keydata[0]);

	return err;
}

#if 1	//Torr@20180117 start
static int solomon_initialize_input_device(struct solomon_device *ftdev)
{
	int error;
	struct input_dev *input_dev;
	input_dev = input_allocate_device();
	if (!input_dev) {
	printk("error : Failed to allocate memory for solomon input device!");
	return -ENOMEM;
	}
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &ftdev->client->dev;
	__set_bit(EV_SYN, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(BTN_TOOL_FINGER, input_dev->keybit);
	error = input_mt_init_slots(input_dev, ftdev->ftconfig->using_point,
				    INPUT_MT_DIRECT);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, ftdev->ftconfig->max_x, 0, 0);	
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, ftdev->ftconfig->max_y, 0, 0);	
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, ftdev->ftconfig->max_weight, 0, 0);
	error = input_register_device(input_dev);
	if (error) {
		printk("Error %d solomon registering input device!\n", error);
		input_free_device(input_dev);
		return error;
	}
	ftdev->input_dev = input_dev;
	return 0;
}
#endif	//Torr@20180117 end

int solomon_gesture_init(struct solomon_device *ftdev)
{
#ifdef KEY_GESTURE_UP 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_UP); 
#endif	/* KEY_GESTURE_UP */

#ifdef KEY_GESTURE_DOWN 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_DOWN); 
#endif	/* KEY_GESTURE_DOWN */

#ifdef KEY_GESTURE_LEFT 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_LEFT); 
#endif	/* KEY_GESTURE_LEFT */

#ifdef KEY_GESTURE_RIGHT 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_RIGHT); 
#endif	/* KEY_GESTURE_RIGHT */

#ifdef KEY_GESTURE_DOUBLECLICK 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_DOUBLECLICK); 
#endif	/* KEY_GESTURE_DOUBLECLICK */

#ifdef KEY_GESTURE_O 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_O); 
#endif	/* KEY_GESTURE_O */

#ifdef KEY_GESTURE_W 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_W); 
#endif	/* KEY_GESTURE_W */

#ifdef KEY_GESTURE_M 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_M); 
#endif	/* KEY_GESTURE_M */

#ifdef KEY_GESTURE_E 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_E); 
#endif	/* KEY_GESTURE_E */

#ifdef KEY_GESTURE_S 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_S); 
#endif	/* KEY_GESTURE_S */

#ifdef KEY_GESTURE_Z 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_Z); 
#endif	/* KEY_GESTURE_Z */

#ifdef KEY_GESTURE_C 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_C); 
#endif	/* KEY_GESTURE_C */

#ifdef KEY_GESTURE_U 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_U); 
#endif	/* KEY_GESTURE_U */

#ifdef KEY_GESTURE_U_DOWN 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_U_DOWN); 
#endif	/* KEY_GESTURE_U_DOWN */

#ifdef KEY_GESTURE_U_RIGHT 
	input_set_capability(tpd->dev, EV_KEY, KEY_GESTURE_U_RIGHT); 
#endif	/* KEY_GESTURE_U_RIGHT */

	return 0;
}

void solomon_gesture_report(struct solomon_device *ftdev, int gesture_code)
{  
	SOLOMON_WARNNING("---> gesture_code = 0x%x", gesture_code);

	switch(gesture_code)
	{
#ifdef KEY_GESTURE_UP 
		case GESTURE_UP:
			input_report_key(tpd->dev, KEY_GESTURE_UP, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_UP, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_UP");
			break;
#endif	/* KEY_GESTURE_UP */
#ifdef KEY_GESTURE_DOWN 
		case GESTURE_DOWN:
			input_report_key(tpd->dev, KEY_GESTURE_DOWN, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_DOWN, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_DOWN");
			break;
#endif	/* KEY_GESTURE_DOWN */
#ifdef KEY_GESTURE_LEFT 
		case GESTURE_LEFT:
			input_report_key(tpd->dev, KEY_GESTURE_LEFT, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_LEFT, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_LEFT");
			break;
#endif	/* KEY_GESTURE_LEFT */
#ifdef KEY_GESTURE_RIGHT 
		case GESTURE_RIGHT:
			input_report_key(tpd->dev, KEY_GESTURE_RIGHT, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_RIGHT, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_RIGHT");
			break;
#endif	/* KEY_GESTURE_RIGHT */
#ifdef KEY_GESTURE_DOUBLECLICK 
		case GESTURE_DOUBLECLICK:
			input_report_key(tpd->dev, KEY_GESTURE_DOUBLECLICK, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_DOUBLECLICK, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_DOUBLECLICK");
			break;
#endif	/* KEY_GESTURE_DOUBLECLICK */
#ifdef KEY_GESTURE_O 
		case GESTURE_O:
			input_report_key(tpd->dev, KEY_GESTURE_O, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_O, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_O");
			break;
#endif	/* KEY_GESTURE_O */
#ifdef KEY_GESTURE_W 
		case GESTURE_W:
			input_report_key(tpd->dev, KEY_GESTURE_W, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_W, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_W");
			break;
#endif	/* KEY_GESTURE_W */
#ifdef KEY_GESTURE_M 
		case GESTURE_M:
			input_report_key(tpd->dev, KEY_GESTURE_M, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_M, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_M");
			break;
#endif	/* KEY_GESTURE_M */
#ifdef KEY_GESTURE_E 
		case GESTURE_E:
			input_report_key(tpd->dev, KEY_GESTURE_E, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_E, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_E");
			break;
#endif	/* KEY_GESTURE_E */
#ifdef KEY_GESTURE_S 
		case GESTURE_S:
			input_report_key(tpd->dev, KEY_GESTURE_S, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_S, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_S");
			break;
#endif	/* KEY_GESTURE_S */
#ifdef KEY_GESTURE_Z 
		case GESTURE_Z:
			input_report_key(tpd->dev, KEY_GESTURE_Z, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_Z, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_Z");
			break;
#endif	/* KEY_GESTURE_Z */
#ifdef KEY_GESTURE_C 
		case GESTURE_C:
			input_report_key(tpd->dev, KEY_GESTURE_C, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_C, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_C");
			break;
#endif	/* KEY_GESTURE_C */
#ifdef KEY_GESTURE_U 
		case GESTURE_U:
			input_report_key(tpd->dev, KEY_GESTURE_U, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_U, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_U");
			break;
#endif	/* KEY_GESTURE_U */
#ifdef KEY_GESTURE_U_DOWN 
		case GESTURE_U_DOWN:
			input_report_key(tpd->dev, KEY_GESTURE_U_DOWN, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_U_DOWN, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_U_DOWN");
			break;
#endif	/* KEY_GESTURE_U_DOWN */
#ifdef KEY_GESTURE_U_RIGHT 
		case GESTURE_U_RIGHT:
			input_report_key(tpd->dev, KEY_GESTURE_U_RIGHT, 1);
			input_sync(tpd->dev);
			input_report_key(tpd->dev, KEY_GESTURE_U_RIGHT, 0);
			input_sync(tpd->dev);
			SOLOMON_WARNNING("---> reporting KEY_GESTURE_U_RIGHT");
			break;
#endif	/* KEY_GESTURE_U_RIGHT */
		default:
			break;
	}
}

/* ESD TIMER */
#if ESD_TIMER_ENABLE
static void esd_checktime_init(struct solomon_device *ftdev)
{
	ftdev->esd_check_time = jiffies;
}

static void esd_timeout_handler(unsigned long data)
{
	struct solomon_device *ftdev = (struct solomon_device *)data;

	if (ftdev == NULL)
		return;

	ftdev->p_esd_tmr = NULL;
	queue_work(ftdev->tmr_workqueue, &ftdev->tmr_work);
}

static void esd_timer_start(u16 sec, struct solomon_device *ftdev)
{
	if (ftdev == NULL)
		return;

	if (ftdev->p_esd_tmr != NULL)
		del_timer(ftdev->p_esd_tmr);

	ftdev->p_esd_tmr = NULL;

	init_timer(&(ftdev->esd_tmr));
	ftdev->esd_tmr.data = (unsigned long)(ftdev);
	ftdev->esd_tmr.function = esd_timeout_handler;
	ftdev->esd_tmr.expires = jiffies + HZ*sec;
	ftdev->p_esd_tmr = &ftdev->esd_tmr;
	add_timer(&ftdev->esd_tmr);
}

static void esd_timer_stop(struct solomon_device *ftdev)
{
	if (ftdev == NULL)
		return;

	if (ftdev->p_esd_tmr)
		del_timer(ftdev->p_esd_tmr);

	ftdev->p_esd_tmr = NULL;
}

//static void esd_timer_init(struct solomon_device *ftdev)
//{
//	if (ftdev == NULL)
//		return;
//
//	init_timer(&(ftdev->esd_tmr));
//	ftdev->esd_tmr.data = (unsigned long)(ftdev);
//	ftdev->esd_tmr.function = esd_timeout_handler;
//	ftdev->esd_check_time = jiffies;
//	ftdev->p_esd_tmr = NULL;
//}

static void touch_esd_tmr_work(struct work_struct *work)
{
	struct solomon_device *ftdev =
		container_of(work, struct solomon_device, tmr_work);

	SOLOMON_DEBUG("tmr queue work ++");
	if (ftdev == NULL) {
		SOLOMON_WARNNING("touch dev == NULL ?");
		goto fail_time_out_init;
	}

	if (down_trylock(&ftdev->work_procedure_lock)) {
		SOLOMON_WARNNING("fail to occupy sema");
		esd_timer_start(SOLOMON_CHECK_ESD_TIMER, ftdev);
		return;
	}

	if (ftdev->work_procedure != TS_NO_WORK) {
		SOLOMON_WARNNING("other process occupied (%d)",
				ftdev->work_procedure);
		up(&ftdev->work_procedure_lock);
		return;
	}

	ts_disable_irq();
	ftdev->work_procedure = TS_ESD_TIMER_WORK;

	SOLOMON_WARNNING("ESD TIMER(%d) %lu - %lu = %lu", HZ, jiffies,
			ftdev->esd_check_time, jiffies-ftdev->esd_check_time);

	if ((jiffies - ftdev->esd_check_time) > HZ * SOLOMON_CHECK_ESD_TIMER) {
		SOLOMON_WARNNING("ESD Time");

		solomon_report_release(ftdev);
		//solomon_power_control(ftdev, POWER_RESET);
#ifdef SUPPORT_MTK_ESD_RECOVERY		
		primary_display_esd_recovery();
		primary_display_suspend();
		primary_display_resume();
#endif
		/* solomon_reset(); */
	}

	ftdev->work_procedure = TS_NO_WORK;

	ts_enable_irq();
	up(&ftdev->work_procedure_lock);
	SOLOMON_DEBUG("tmr queue work ----");

	esd_timer_start(SOLOMON_CHECK_ESD_TIMER, ftdev);
	return;

fail_time_out_init:
	SOLOMON_DEBUG("tmr work : restart error");
	ftdev->work_procedure = TS_NO_WORK;
	ts_enable_irq();
	up(&ftdev->work_procedure_lock);
	esd_timer_start(SOLOMON_CHECK_ESD_TIMER, ftdev);
}
#endif	/* ESD_TIMER_ENABLE */

/* get TCI1 fail reason */
static inline s32 ts_read_TCI1_Fail_reason(struct solomon_device *dev)
{
	int err = 0;
	u16 buff[3] = {0,};
	u8 *tmp = NULL;
	int fail_count = 0;
	int i=0;

	err = ts_read_data(dev->client, SOLOMON_TCI1_FAIL_REASON, (u8*)buff, 6);

	if (err < 0) {
		SOLOMON_DEBUG("error : TCI1 Fail reason");
		return -EAGAIN;
	}

	tmp = (u8 *)buff;
	fail_count = tmp[0];
	SOLOMON_WARNNING("Fail Count : %d", fail_count);

	for (i = 0; i < fail_count; i++)
		SOLOMON_WARNNING("[%d] %d", i, tmp[i+1]);

	return err;
}

/* get TCI2 fail reason */
static inline s32 ts_read_TCI2_Fail_reason(struct solomon_device *dev)
{
	int err = 0;
	u16 buff[3] = {0,};
	u8 *tmp = NULL;
	int fail_count = 0;
	int i=0;

	err = ts_read_data(dev->client, SOLOMON_TCI2_FAIL_REASON, (u8*)buff, 6);

	if (err < 0) {
		SOLOMON_DEBUG("error : TCI2 Fail reason");
		return -EAGAIN;
	}

	tmp = (u8 *)buff;
	fail_count = tmp[0];
	SOLOMON_WARNNING("Fail Count : %d", fail_count);

	for (i = 0; i < fail_count; i++)
		SOLOMON_WARNNING("[%d] %d", i, tmp[i+1]);

	return err;
}

static ssize_t ssl_fail_reason(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct solomon_device *ftdev = i2c_get_clientdata(client);

	if (count > 0) {
		if (buf[0] == 0x31) {
			ts_read_TCI1_Fail_reason(ftdev);
		} else {
			ts_read_TCI2_Fail_reason(ftdev);
		}
	}

	return count;
}

static DEVICE_ATTR(fail_reason, S_IRUGO, NULL, ssl_fail_reason);

/* get the version info use kernel command. */
static ssize_t ssl_read_version(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	char tmp_str[128] = {0};
	char ssd_return_str[512] = {0};
	char hex_str1[5]={0};
	char hex_str2[5]={0};
	int ret;

	memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
	memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "################################\n");strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, " Display Version : 0x%08x\n",misc_dev->fw_version.display_version);strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "  Driver Version : %s\n",DRIVER_VRESION);strcat(ssd_return_str, tmp_str);
	hex_str1[0] = (misc_dev->fw_version.productID01&0xFF000000)>>24;
	hex_str1[1] = (misc_dev->fw_version.productID01&0x00FF0000)>>16;
	hex_str1[2] = (misc_dev->fw_version.productID01&0x0000FF00)>>8;
	hex_str1[3] = (misc_dev->fw_version.productID01&0x000000FF)>>0;
	hex_str2[0] = (misc_dev->fw_version.productID02&0xFF000000)>>24;
	hex_str2[1] = (misc_dev->fw_version.productID02&0x00FF0000)>>16;
	hex_str2[2] = (misc_dev->fw_version.productID02&0x0000FF00)>>8;
	hex_str2[3] = (misc_dev->fw_version.productID02&0x000000FF)>>0;
	memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "      Product ID : 0x%08x(%4s) 0x%08x(%4s)\n",misc_dev->fw_version.productID01, hex_str1, misc_dev->fw_version.productID02, hex_str2);strcat(ssd_return_str, tmp_str);
	hex_str1[0] = (misc_dev->fw_version.ICName01&0xFF000000)>>24;
	hex_str1[1] = (misc_dev->fw_version.ICName01&0x00FF0000)>>16;
	hex_str1[2] = (misc_dev->fw_version.ICName01&0x0000FF00)>>8;
	hex_str1[3] = (misc_dev->fw_version.ICName01&0x000000FF)>>0;
	hex_str2[0] = (misc_dev->fw_version.ICName02&0xFF000000)>>24;
	hex_str2[1] = (misc_dev->fw_version.ICName02&0x00FF0000)>>16;
	hex_str2[2] = (misc_dev->fw_version.ICName02&0x0000FF00)>>8;
	hex_str2[3] = (misc_dev->fw_version.ICName02&0x000000FF)>>0;
	memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "         IC Name : 0x%08x(%4s) 0x%08x(%4s)\n",misc_dev->fw_version.ICName01, hex_str1, misc_dev->fw_version.ICName02, hex_str2);strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "      Resolution : %d X %d\n", misc_dev->ftconfig->max_x, misc_dev->ftconfig->max_y);strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "       Node Info : %d X %d\n", misc_dev->ftconfig->x_node, misc_dev->ftconfig->y_node);strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "     Point Count : %d\n", misc_dev->ftconfig->using_point);strcat(ssd_return_str, tmp_str);
	memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "#################################\n");strcat(ssd_return_str, tmp_str);

	ret = snprintf(buf, PAGE_SIZE, "%s\n", ssd_return_str);
	return ret;
}
static DEVICE_ATTR(version, S_IRUGO, ssl_read_version, NULL);

static ssize_t class_ts_info_show(struct class *class,
		struct class_attribute *attr, char *buf)
{
	//    struct ssl_version *fw_version = fw_get_version();

	return sprintf(buf,"Solomon FW version: 0x%x\n", misc_dev->fw_version.display_version);
}

static CLASS_ATTR(ts_info, S_IRUSR, class_ts_info_show, NULL);

static ssize_t ssl_test_mode(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct solomon_device *ftdev = i2c_get_clientdata(client);

	if (count > 0) {
		if (buf[0] == 0x32) {
/* it is test for firmware upgrade.*/
			ts_disable_irq();
			ftdev->work_procedure = TS_IN_UPGRADE;
#if ESD_TIMER_ENABLE
			esd_checktime_init(ftdev);
#endif
			ftdev->work_procedure = TS_NO_WORK;
			ts_enable_irq();
			solomon_power_control(ftdev, POWER_RESET);
		} else if (buf[0] == 0x31) {
			ts_disable_irq();
			ftdev->work_procedure = TS_IN_UPGRADE;
			solomon_firmware_update_byfile(ftdev,
				FW_FORCE_FULL_PATH);
#if ESD_TIMER_ENABLE
			esd_checktime_init(ftdev);
#endif
			ftdev->work_procedure = TS_NO_WORK;
			ts_enable_irq();
			solomon_power_control(ftdev, POWER_RESET);
			/* solomon_reset(); */
		} else {
			solomon_power_control(ftdev, POWER_RESET);
			/* solomon_reset(); */
		}
	}

	return count;
}

static DEVICE_ATTR(testing, S_IRUGO, NULL, ssl_test_mode);

static ssize_t ssl_esd_time(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
#if ESD_TIMER_ENABLE
	struct i2c_client *client = to_i2c_client(dev);
	struct solomon_device *ftdev = i2c_get_clientdata(client);

	if (count > 0) {
		if (buf[0] == '0') {
			if (ftdev->use_esd_tmr) {
				SOLOMON_WARNNING("esd timer stop");
				esd_timer_stop(ftdev);
				ftdev->use_esd_tmr = 0;
			}
		} else {
			SOLOMON_WARNNING("esd timer start");
			ftdev->use_esd_tmr = 1;
			esd_checktime_init(ftdev);
			esd_timer_start(SOLOMON_CHECK_ESD_TIMER, ftdev);
		}
	}
#endif
	return count;
}

static DEVICE_ATTR(esdtime, S_IRUGO, NULL, ssl_esd_time);
/* get x, y node count */
static inline s32 ts_read_node(struct i2c_client *client, u8 *node_x,
		u8 *node_y)
{
	int err = 0;

	err = ts_read_data(client, SOLOMON_TOTAL_X_NODE, node_x, 2);

	if (err < 0) {
		SOLOMON_WARNNING("error : read x node");
		return -EAGAIN;
	}
	SOLOMON_DEBUG("x node : %d", *node_x);

	err = ts_read_data(client, SOLOMON_TOTAL_Y_NODE, node_y, 2);
	if (err < 0) {
		SOLOMON_WARNNING("error : read y node");
		return -EAGAIN;
	}
	SOLOMON_DEBUG("y node : %d", *node_y);

	return err;
}

/* set resolution */
static inline s32 ts_write_resolution(struct i2c_client *client, u16 resol_x,
		u16 resol_y)
{
	int err = 0;

	err = ts_write_data(client, SOLOMON_X_RESOLUTION, (u8 *)&(resol_x), 2);

	if (err < 0) {
		SOLOMON_WARNNING("error : write x resolution!!");
		return -EAGAIN;
	}

	err = ts_write_data(client, SOLOMON_Y_RESOLUTION, (u8 *)&(resol_y), 2);

	if (err < 0) {
		SOLOMON_WARNNING("error : write y resolution!!");
		return -EAGAIN;
	}

	return err;
}

/* set point num */
static inline s32 ts_write_point_num(struct i2c_client *client, u16 point)
{
	int err = 0;

	err = ts_write_data(client, SOLOMON_USING_POINT_NUM,
			(u8 *)&(point), 2);

	if (err < 0) {
		SOLOMON_WARNNING("error : write point num!!");
		return -EAGAIN;
	}

	return err;
}

int ds_read_boot_st(struct i2c_client *client, u16 *value)
{
	int ret = 0;
	u8 wd[10];
	u8 rd[10];

	wd[0] = 0x00;
	wd[1] = 0x00;
	wd[2] = 0x00;
	wd[3] = 0x00;

	ret = ts_read_data_ex(client, wd, 4, rd, 2);

	if (ret < 0) {
		SOLOMON_WARNNING("read boot st i2c read fail(1)!!");
		return ret;
	}

	*value = rd[0] | (rd[1] << 8);

	return ret;
}

int ds_clear_int(struct i2c_client *client)
{
	int ret = 0;
	u8 wd[10];

	wd[0] = 0x00;
	wd[1] = 0x00;

	ret = ts_write_data(client, DS_CLEAR_INT, wd, 2);

	if (ret < 0) {
		SOLOMON_WARNNING("0x%04X i2c write fail(2)!!", DS_CLEAR_INT);
		return ret;
	}

	return ret;
}

int ds_eflash_write(struct i2c_client *client, int addr, u16 data)
{
	int ret = 0;
	u8 wd[10];

	wd[0] = addr & 0xFF;
	wd[1] = (addr >> 8) & 0xFF;
	wd[2] = data & 0xFF;
	wd[3] = (data >> 8) & 0xFF;

	ret = ts_write_data(client, DS_EFLASH_WRITE, wd, 4);

	if (ret < 0) {
		SOLOMON_WARNNING("0x%04X i2c read fail(2)!!", DS_COMMAND_01);
		return ret;
	}

	return ret;
}

int ds_eflash_read(struct i2c_client *client, int addr, u8 *rd, int rLen)
{
	int ret = 0;
	u8 wd[10];

	wd[0] = DS_EFLASH_READ & 0xff;
	wd[1] = (DS_EFLASH_READ >> 8) & 0xff;
	wd[2] = addr & 0xff;
	wd[3] = (addr >> 8) & 0xff;

	ret = ts_read_data_ex(client, wd, 4, rd, rLen);

	if (ret < 0) {
		SOLOMON_WARNNING("0x%04X i2c read fail(1)!!", addr);
		return ret;
	}

	return ret;
}

#ifdef SUPPORT_ES2
/* read version */
static int ds_read_version(struct solomon_device *ftdev, u8 *version, int len)
{
	int ret = 0;
	u8 wd[10];

	wd[0] = DS_COMMAND_READ_VERSION & 0xff;
	wd[1] = (DS_COMMAND_READ_VERSION >> 8) & 0xff;
	wd[2] = 0x0080 & 0xff;
	wd[3] = (0x0080 >> 8) & 0xff;

	ret = ts_read_data_ex(ftdev->client, wd, 4, version, len);

	if (ret < 0) {
		SOLOMON_WARNNING("DS read version fail!!");
		return ret;
	}

	return 0;
}

static int ds_process_version(struct solomon_device *ftdev)
{
	int ret = 0;
	u16 version[5] = {0,};

	ret = ds_read_version(ftdev, (u8 *)version, 10);

	if (ret < 0)
		return ret;

	SOLOMON_WARNNING("[0] 0x%04x", version[0]);
	SOLOMON_WARNNING("[1] 0x%04x", version[1]);
	SOLOMON_WARNNING("[2] 0x%04x", version[2]);
	SOLOMON_WARNNING("[3] 0x%04x", version[3]);
	SOLOMON_WARNNING("[4] 0x%04x", version[4]);
	/* SSD2098 ES1 */
	if (version[0] == 0x055D && version[1] == 0x2098 &&
		version[2] == 0x0001 && version[3] == 0x2015 &&
		version[4] == 0x1012)
		ftdev->es_version = DS_VERSION_ES1;
	else if (version[0] == 0x2405 && version[1] == 0x0002 &&
		version[2] == 0x2016 && version[3] == 0x0504 &&
		version[4] == 0x1742)
		ftdev->es_version = DS_VERSION_ES2;
	else
		ftdev->es_version = DS_VERSION_ES1;

	SOLOMON_WARNNING("ES Version : %d", ftdev->es_version);
	return 0;
}
#endif
static int ds_init_code(struct i2c_client *client)
{
	int ret = 0;

	ret = ds_eflash_write(client, 0xE003, 0x0007);

	if (ret < 0)
		return ret;

	ret = ds_eflash_write(client, 0xE000, 0x0048);

	if (ret < 0)
		return ret;

	return ret;
}

static int sint_unstall(struct i2c_client *client)
{
	s32 ret = 0;
	u32 temp_flag = 0x00;

	temp_flag = 0x0000;

	ret = ts_write_data(client, DS_CUP_CONTROL, (u8 *)&temp_flag, 2);

	if (ret < 0) {
		SOLOMON_WARNNING("0x%04X i2c write fail!!", DS_CUP_CONTROL);
		return ret;
	}

	return ret;
}
/* send the clear command to TMC */
static inline s32 int_clear_cmd(struct i2c_client *client)
{
	int val = 0x01;

	return ts_write_data(client, SOLOMON_INT_CLEAR_CMD, (u8 *)&val, 2);
}

/* Check interrupt pin */
static s32 int_pin_check(struct solomon_device *ftdev, int retry)
{
	int ret = 0;

	if (ftdev == NULL)
		return 0;

	if (retry == 0)
		return 0;

	do {
		gpio_direction_input(tpd_int_gpio_number);
		if (gpio_get_value(ftdev->int_pin) == 0)
			break;

		mdelay(1);
	} while ((retry--) > 0);

	if (retry < 1)
		ret = -1;

	return ret;
}

#if ESD_TIMER_ENABLE
/* ESD Timer set to TMC
 * value : frame count. if you want to set 2sec, you should input 120(60x2).
 */
static s32 solomon_set_esdtime(struct solomon_device *ftdev, u16 value)
{
	int err;

	SOLOMON_WARNNING("ESD Time : %d", value);

	err = ts_write_data(ftdev->client,
			 SOLOMON_ESD_TIME, (u8 *)&(value), 2);

	if (err < 0)
		SOLOMON_WARNNING("Fail to set ESD Time.");

	return err;
}
#endif

#ifdef SUPPORT_LPM
static inline s32 lpm_end(struct solomon_device *ftdev)
{
	s32 ret = 0;
	u32 temp_flag = 0x0000;
	int retry1 = 5;

again:
	ret = ts_write_data(ftdev->client, DS_CUP_CONTROL,
			(u8 *)&temp_flag, 2);

	if (ret < 0)
		SOLOMON_WARNNING("0x%04X i2c write fail!!", DS_CUP_CONTROL);

	SOLOMON_WARNNING("retry = %d \t ret : 0x%04x", retry1, ret);

	retry1--;

	if (ret < 0 && retry1 > 0)
		goto again;

	return ret;
}

static inline s32 lpm_end_clear(struct solomon_device *ftdev)
{
	int err = 0;
	int retry = 500;

	err = int_pin_check(ftdev, retry);
	int_clear_cmd(ftdev->client);

	return err;
}

static inline s32 lpm_end2(struct solomon_device *ftdev)
{
	s32 ret = 0;
	u32 temp_flag = 0x0000;
	int retry = 5;

again:
	SOLOMON_WARNNING("lpm end2");
	ts_write_data(ftdev->client, DS_CUP_CONTROL, (u8 *)&temp_flag, 2);

	mdelay(1);

	ret = ts_write_data(ftdev->client, DS_CUP_CONTROL,
			(u8 *)&temp_flag, 2);

	if (ret < 0)
		SOLOMON_WARNNING("0x%04X i2c write fail!!", DS_CUP_CONTROL);

	retry--;

	if (ret < 0 && retry > 0)
		goto again;

	return ret;
}
#endif	/* SUPPORT_LPM */
static s32 solomon_set_mptest(struct solomon_device *ftdev, u16 value)
{
	int err;

	err = ts_write_data(ftdev->client,
			 SOLOMON_MP_TEST, (u8 *)&(value), 2);
	if (err < 0)
		SOLOMON_WARNNING("Fail to set MP_TEST %d."
			, ftdev->mptest_mode);

	ftdev->mptest_mode = value;
	ftdev->ftdata->queue_front = 0;
	ftdev->ftdata->queue_rear = 0;
#if 0
	err = ts_read_data(ftdev->client,
			 SOLOMON_MP_TEST, (u8 *)&(ftdev->mptest_mode), 2);
	if (err < 0)
		SOLOMON_WARNNING("err : read MP_TEST");

	SOLOMON_DEBUG("touch mode %d %d", value, ftdev->mptest_mode);
#endif

	return err;
}

static s32 solomon_set_mptest_alone(struct solomon_device *ftdev, u16 value)
{
	int err = 0;

	disable_irq(ftdev->irq);

	down(&ftdev->work_procedure_lock);

	if (ftdev->work_procedure != TS_NO_WORK) {
		SOLOMON_DEBUG("other process occupied.. (%d)\n",
			ftdev->work_procedure);
		enable_irq(ftdev->irq);
		up(&ftdev->work_procedure_lock);
		return -1;
	}

	ftdev->work_procedure = TS_SET_MODE;

	solomon_set_mptest(ftdev, value);

	ftdev->work_procedure = TS_NO_WORK;
	enable_irq(ftdev->irq);
	up(&ftdev->work_procedure_lock);

	return err;
}


static ssize_t ssl_mptest(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct solomon_device *ftdev = i2c_get_clientdata(client);
	u16 mode = 0;

	if (count > 0) {
		mode = buf[0] - '0';

		SOLOMON_WARNNING("MODE : %d", mode);

		if (mode == 0) {
			ftdev->mptest_mode = (ftdev->mptest_mode |
						MPTEST_READY_STOP);
			m_mp_total_count = -1;
			m_mp_cur_count = 0;
			m_mp_calldealy = 0;
		} else {
			ftdev->ftdata->queue_front = 0;
			ftdev->ftdata->queue_rear = 0;
			solomon_set_mptest_alone(ftdev, mode);

			m_mp_total_count = -1;
			m_mp_cur_count = 0;
			if (mode >= MPTEST_LPM_NC_RANGE ||
				mode <= MPTEST_LPM_LC_RANGE)
				m_mp_total_count = 10;

			if (mode >= MPTEST_LPM_NC_RANGE &&
				mode <= MPTEST_LPM_LC_JITTER)
				m_mp_skip_count = 100;
			else
				m_mp_skip_count = 0;
		}
	}

	return count;
}

static DEVICE_ATTR(mptest, S_IRUGO, NULL, ssl_mptest);

static s32 solomon_set_touchmode(u16 value)
{
	int err;
	int retry = 3;

	ts_disable_irq();

	down(&misc_dev->work_procedure_lock);

	if (misc_dev->work_procedure != TS_NO_WORK) {
		SOLOMON_WARNNING("other process occupied.. (%d)\n",
				misc_dev->work_procedure);
		ts_enable_irq();
		up(&misc_dev->work_procedure_lock);
		return -1;
	}

	misc_dev->work_procedure = TS_SET_MODE;
retry:
	err = ts_write_data(misc_dev->client,
			SOLOMON_TOUCH_MODE, (u8 *)&(value), 2);
	if (err < 0)
		SOLOMON_WARNNING("Fail to set TOUCH_MODE %d."
				, misc_dev->touch_mode);

	err = ts_read_data(misc_dev->client,
			SOLOMON_TOUCH_MODE, (u8 *)&(misc_dev->touch_mode), 2);
	if (err < 0)
		SOLOMON_WARNNING("err : read touch_mode");

	SOLOMON_DEBUG("touch mode %d %d", value, misc_dev->touch_mode);
	if (value != misc_dev->touch_mode) {
		SOLOMON_WARNNING("touch mode set fail!!!");
		if ((retry--) > 0) {
			SOLOMON_WARNNING("retry!!");
			goto retry;
		}
	}

	m_point_skip_time = jiffies;

	misc_dev->work_procedure = TS_NO_WORK;
	ts_enable_irq();
	up(&misc_dev->work_procedure_lock);

	return err;
}

static ssize_t ssl_touchmode(struct device *dev, struct device_attribute *attr,
	const char *buf, size_t count)
{
	u16 t = 0, i = 0;
	struct i2c_client *client = to_i2c_client(dev);
	struct solomon_device *ftdev = i2c_get_clientdata(client);

	if (count > 0) {
		for (i = 0; i < count-1; i++)
			t = t*10 + buf[i] - '0';

		SOLOMON_WARNNING("\ntouch mode : %d", t);
		ftdev->ftdata->queue_front = 0;
		ftdev->ftdata->queue_rear = 0;

		solomon_set_touchmode(t);
	}

	return count;
}

static DEVICE_ATTR(touchmode, S_IRUGO, NULL, ssl_touchmode);


/* send sleep-in command to TMC */
static s32 solomon_set_sleepin(struct solomon_device *ftdev)
{
	int err = 0x00;
#if 0
	u16 value = 0x00;
	int retry = 5;

	do {
		err = ts_write_data(misc_dev->client,
				SOLOMON_SLEEP_IN, (u8 *)&(value), 2);

		if (err >= 0)
			break;

		SOLOMON_WARNNING("Fail to set SOLOMON_SLEEP_IN.");
		mdelay(1);
	} while ((retry--) > 0);
#endif
	return err;
}

/* send sleep-out command to TMC */
static s32 solomon_set_sleepout(struct solomon_device *ftdev)
{
	int err = 0x00;
#if 0
	u16 value = 0x00;
	int retry = 5;

	do {
		err = ts_write_data(misc_dev->client,
				SOLOMON_SLEEP_OUT, (u8 *)&(value), 2);
		if (err >= 0)
			break;

		SOLOMON_WARNNING("Fail to set SOLOMON_SLEEP_OUT.");
		mdelay(1);
	} while ((retry--) > 0);
#endif
	return err;
}

/* IOCTL Driver */
static int ts_misc_fops_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int ts_misc_fops_release(struct inode *inode, struct file *file)
{
	return 0;
}

static long ts_misc_fops_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	struct _raw_ioctl raw_ioctl;
	struct _reg_ioctl reg_ioctl;
	struct _down_ioctl down_ioctl;
	struct _mp_ioctl mp_ioctl;

	u8 *u8Data;
	int ret = 0;
	u16 val;
	int nval = 0;
	char data[256];

	if (misc_dev == NULL)
		return -1;

	switch (cmd) {
		case TOUCH_IOCTL_GET_FW_VERSION:
			if (copy_to_user(argp, misc_dev->ftconfig->fw_ver,
						sizeof(misc_dev->ftconfig->fw_ver)))
				return -1;
			break;
#ifdef SUPPORT_GESTURE_DEMO
		case TOUCH_IOCTL_GET_GESTURE:
		ret = m_gesture_value;
			SOLOMON_WARNNING("(IOCTL) GESTURE VAL : 0x%02x",
			m_gesture_value);
			if (copy_to_user(argp, &ret, sizeof(ret)))
				return -1;
			break;
#endif	/* SUPPORT_GESTURE_DEMO */
		case TOUCH_IOCTL_GET_DATA_VERSION:
			ret = misc_dev->ftconfig->data_ver;
			if (copy_to_user(argp, &ret, sizeof(ret)))
				return -1;
			break;

		case TOUCH_IOCTL_GET_X_RESOLUTION:
			ret = misc_dev->ftconfig->max_x;
			if (copy_to_user(argp, &ret, sizeof(ret)))
				return -1;
			break;

		case TOUCH_IOCTL_GET_Y_RESOLUTION:
			ret = misc_dev->ftconfig->max_y;
			if (copy_to_user(argp, &ret, sizeof(ret)))
				return -1;
			break;

		case TOUCH_IOCTL_GET_X_NODE_NUM:
			ret = misc_dev->ftconfig->x_node;
			if (copy_to_user(argp, &ret, sizeof(ret)))
				return -1;
			break;

		case TOUCH_IOCTL_GET_Y_NODE_NUM:
			ret = misc_dev->ftconfig->y_node;
			if (copy_to_user(argp, &ret, sizeof(ret)))
				return -1;

			break;

		case TOUCH_IOCTL_SET_TOUCH_MODE:
			if (copy_from_user(&nval, argp, 2))
				return -1;
		SOLOMON_WARNNING("Touch Mode : %d", nval);
			return solomon_set_touchmode((u16)nval);

	case TOUCH_IOCTL_SW_RESET:
		sint_unstall(misc_dev->client);	/* TMC sw reset */
		SOLOMON_WARNNING("SW reset");
		break;

	case TOUCH_IOCTL_HW_RESET:
		/* TMC hw reset */
		solomon_power_control(misc_dev, POWER_RESET);
		SOLOMON_WARNNING("HW reset");
		break;

	case TOUCH_IOCTL_MP_TEST:
		ts_disable_irq();
		misc_dev->work_procedure = TS_SET_MODE;

		if (copy_from_user(&mp_ioctl,
			argp, sizeof(struct _mp_ioctl)))
			return -1;

		SOLOMON_WARNNING("MP TEST Mode : %d", mp_ioctl.mode);
		SOLOMON_WARNNING("MP TEST Count : %d", mp_ioctl.count);
		SOLOMON_WARNNING("MP TEST delay : %d", mp_ioctl.calldelay);
		SOLOMON_WARNNING("MP TEST needback : %d", mp_ioctl.needback);

		if (mp_ioctl.mode == 0) {
			SOLOMON_WARNNING("----------------- Stop mp mode");

			/* change MP mode */
			if (misc_dev->mptest_mode != MPTEST_STOP)
				solomon_set_mptest(misc_dev, MPTEST_STOP);

			m_mp_total_count = -1;
			m_mp_cur_count = 0;
			m_mp_calldealy = 0;
			m_mp_needback = 0;
			ret = 0;
		} else {
			misc_dev->mptest_mode = (mp_ioctl.mode |
						MPTEST_READY_START);
			m_mp_total_count = mp_ioctl.count;
			m_mp_calldealy = mp_ioctl.calldelay;
			m_mp_needback = mp_ioctl.needback;

			misc_dev->ftdata->queue_front = 0;
			misc_dev->ftdata->queue_rear = 0;

			m_mp_cur_count = m_mp_total_count;
			m_mp_skip_count = 1;

			sint_unstall(misc_dev->client);
			mdelay(m_mp_calldealy);

			SOLOMON_WARNNING("MP1 Mode:%d", misc_dev->mptest_mode);
			SOLOMON_WARNNING("MP1 Count : %d", m_mp_total_count);
			SOLOMON_WARNNING("MP1 delay : %d", m_mp_calldealy);
			SOLOMON_WARNNING("MP1 needback : %d", m_mp_needback);
		}

		misc_dev->work_procedure = TS_NO_WORK;
		ts_enable_irq();
		return ret;

	case TOUCH_IOCTL_GET_REG:
		down(&misc_dev->work_procedure_lock);
		if (misc_dev->work_procedure != TS_NO_WORK) {
			SOLOMON_DEBUG("other process occupied.. (%d)\n",
				misc_dev->work_procedure);
			up(&misc_dev->work_procedure_lock);
			return -1;
		}

			misc_dev->work_procedure = TS_SET_MODE;

			if (copy_from_user(&reg_ioctl,
						argp, sizeof(struct _reg_ioctl))) {
				misc_dev->work_procedure = TS_NO_WORK;
				up(&misc_dev->work_procedure_lock);
				SOLOMON_WARNNING("error : copy_from_user\n");
				return -1;
			}

			val = 0;
			if (ts_read_data(misc_dev->client,
						reg_ioctl.addr, (u8 *)&val, 2) < 0)
				ret = -1;

			nval = (int)val;

			if (copy_to_user(reg_ioctl.val, (u8 *)&nval, sizeof(nval))) {
				misc_dev->work_procedure = TS_NO_WORK;
				up(&misc_dev->work_procedure_lock);
				SOLOMON_WARNNING("error : copy_to_user\n");
				return -1;
			}

			SOLOMON_DEBUG("read : reg addr = 0x%x, val = 0x%x, ret = %d\n",
					reg_ioctl.addr, nval, ret);

			misc_dev->work_procedure = TS_NO_WORK;
			up(&misc_dev->work_procedure_lock);
			return ret;
		case TOUCH_IOCTL_SET_REG:
			down(&misc_dev->work_procedure_lock);
			if (misc_dev->work_procedure != TS_NO_WORK) {
				SOLOMON_WARNNING("other process occupied.. (%d)\n",
						misc_dev->work_procedure);
				up(&misc_dev->work_procedure_lock);
				return -1;
			}

			misc_dev->work_procedure = TS_SET_MODE;

			if (copy_from_user(&reg_ioctl,
						argp, sizeof(struct _reg_ioctl))) {
				misc_dev->work_procedure = TS_NO_WORK;
				up(&misc_dev->work_procedure_lock);
				SOLOMON_WARNNING("error : copy_from_user\n");
				return -1;
			}

			if (copy_from_user(&val, reg_ioctl.val, 2)) {
				misc_dev->work_procedure = TS_NO_WORK;
				up(&misc_dev->work_procedure_lock);
				SOLOMON_WARNNING("error : copy_from_user\n");
				return -1;
			}

			if (ts_write_data(misc_dev->client,
						reg_ioctl.addr, (u8 *)&val, 2) < 0)
				ret = -1;

			SOLOMON_DEBUG("write : reg addr = 0x%x, val = 0x%x",
					reg_ioctl.addr, val);

			misc_dev->work_procedure = TS_NO_WORK;
			up(&misc_dev->work_procedure_lock);

			return ret;
		case TOUCH_IOCTL_GET_GRAPH_DATA:
			down(&misc_dev->work_procedure_lock);
			if (misc_dev->work_procedure != TS_NO_WORK) {
				SOLOMON_WARNNING("other process occupied.. (%d)\n",
						misc_dev->work_procedure);
				up(&misc_dev->work_procedure_lock);
				return -1;
			}

			misc_dev->work_procedure = TS_SET_MODE;

			if (copy_from_user(&reg_ioctl,
						argp, sizeof(struct _reg_ioctl))) {
				SOLOMON_WARNNING("error : copy_from_user\n");
				ret = -1;
				goto out_graph;
			}

			val = 0;
			if (ts_read_data(misc_dev->client,
						reg_ioctl.addr, (u8 *)data, 64) < 0) {
				ret = -1;
				goto out_graph;
			}

			if (copy_to_user(reg_ioctl.val, (u8 *)&data, 64)) {
				SOLOMON_WARNNING("error : copy_to_user\n");
				ret = -1;
			}

			SOLOMON_DEBUG("read : reg addr = 0x%x", reg_ioctl.addr);
out_graph:
			misc_dev->work_procedure = TS_NO_WORK;
			up(&misc_dev->work_procedure_lock);
			return ret;

		case TOUCH_IOCTL_QUEUE_CLEAR:
			misc_dev->ftdata->queue_front = 0;
			misc_dev->ftdata->queue_rear = 0;
			SOLOMON_WARNNING("Rawdata queue clear!!!\n");
			return ret;

		case TOUCH_IOCTL_GET_RAW_DATA:
			if (misc_dev->touch_mode == TOUCH_POINT_MODE &&
				misc_dev->mptest_mode == MPTEST_STOP)
			return -1;

			if (copy_from_user(&raw_ioctl,argp, sizeof(raw_ioctl))) {
				SOLOMON_WARNNING("error : copy_from_user");
				return -1;
			}

			u8Data = get_front_queue_buff(misc_dev->ftdata);

			if (u8Data == NULL) {
				SOLOMON_WARNNING("rawdata queue is empty");
				return -3;
			}

			if (copy_to_user(raw_ioctl.buf,
						u8Data, raw_ioctl.sz)) {
				SOLOMON_WARNNING("error : copy_to_user");
				return -1;
			}
			get_queue(misc_dev->ftdata);

			return 0;

		case TOUCH_IOCTL_SET_DOWNLOAD:
			ts_disable_irq();
			down(&misc_dev->work_procedure_lock);
			misc_dev->work_procedure = TS_IN_UPGRADE;

#if ESD_TIMER_ENABLE
			if (misc_dev->use_esd_tmr)
				esd_timer_stop(misc_dev);
#endif

			memset(&down_ioctl, 0, sizeof(struct _down_ioctl));

			if (copy_from_user(&down_ioctl,
						argp, sizeof(struct _down_ioctl))) {
				misc_dev->work_procedure = TS_NO_WORK;
				up(&misc_dev->work_procedure_lock);
				ts_enable_irq();
				SOLOMON_WARNNING("error : copy_from_user\n");
				return -1;
			}

			if (copy_from_user(&data,
						down_ioctl.file, 256)) {
				misc_dev->work_procedure = TS_NO_WORK;
				up(&misc_dev->work_procedure_lock);
				ts_enable_irq();
				SOLOMON_WARNNING("error : copy_from_user\n");
				return -1;
			}

			SOLOMON_WARNNING("NAME : %s", down_ioctl.file);
			SOLOMON_WARNNING("SIZE : %d", down_ioctl.sz);

			ret = solomon_firmware_update_byfile(misc_dev,
					down_ioctl.file);

			if (ret < 0)
				SOLOMON_WARNNING("firmware update by file failed");

		/* solomon_reset(); */
		solomon_power_control(misc_dev, POWER_RESET);
#if ESD_TIMER_ENABLE
			if (misc_dev->use_esd_tmr) {
				esd_checktime_init(misc_dev);
				esd_timer_start(SOLOMON_CHECK_ESD_TIMER, misc_dev);
				SOLOMON_WARNNING("esd timer start");
			}
#endif	/* ESD_TIMER_ENABLE */
			misc_dev->work_procedure = TS_NO_WORK;
			up(&misc_dev->work_procedure_lock);
			ts_enable_irq();

			return ret;

		default:
			break;
	}

	return 0;
}

static const struct file_operations ts_misc_fops = {
	.open = ts_misc_fops_open,
	.release = ts_misc_fops_release,
	.unlocked_ioctl = ts_misc_fops_ioctl,
};

static struct miscdevice touch_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "sentron_touch_misc",
	.fops = &ts_misc_fops,
};

/* power control */
static int solomon_power_control(struct solomon_device *ftdev, u8 mode)
{
	int err = 0;

	if (mode == POWER_OFF) {
	} 
	else if (mode == POWER_ON) {
	} 
	else if (mode == POWER_ON_SEQUENCE) {
		SOLOMON_DEBUG("POWER_SEQUENCE");
#ifdef SUPPORT_ESD_CHECKSUM
		ftdev->state_flag = 0;
#endif
		solomon_reset();
	} else if (mode == POWER_RESET) {
#ifdef SUPPORT_ESD_CHECKSUM
		ftdev->state_flag = 0;
#endif
		solomon_reset();
	}

	return err;
}

int solomon_reset(void)
{
	int retry = 100;
#if defined(SUPPORT_TOUCH_RESET_PIN_CTL)
	msleep(10);
	gpio_set_value(misc_dev->reset_pin, 1);
	msleep(10);
	gpio_set_value(misc_dev->reset_pin, 0);
	msleep(5);
	gpio_set_value(misc_dev->reset_pin, 1);
	msleep(20);
#else
	msleep(10);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(10);
	tpd_gpio_output(tpd_rst_gpio_number, 0);
	msleep(20);
	tpd_gpio_output(tpd_rst_gpio_number, 1);
	msleep(80);
#endif 
	/* wait DS init; */
	if (misc_dev != NULL)
		int_pin_check(misc_dev, retry * 2);

	return 0;
}

/* Read HW info use I2C */
int solomon_read_hw_info(struct i2c_client *client, u16 *info)
{
	int err = 0;

	err = ts_read_data(client, SOLOMON_HW_CAL_INFO, (u8 *)(info), 2);

	if (err < 0)
		SOLOMON_WARNNING("error read HW cal info using i2c.-");

	return err;
}

/* Write HW info use I2c */
int solomon_write_hw_cal(struct i2c_client *client)
{
	int err = 0;
	u16 info = 0x01;

	err = ts_write_data(client, SOLOMON_HW_CALIBRATION, (u8 *)&(info), 2);

	if (err < 0)
		SOLOMON_WARNNING("error HW cal info write (0x%04x)!!", info);

	return err;
}

int solomon_hw_check(struct solomon_device *ftdev)
{
	int err = 0;
	int retry1 = 3;
	int limit = 0;
	u16 info = 0;

	err = solomon_read_hw_info(ftdev->client, &info);

	if (err >= 0) {
		SOLOMON_WARNNING("HW Cal info = %d", info);

		if (info == 0) {
			do {
				limit = 5;
				err = solomon_write_hw_cal(ftdev->client);

				if (err < 0)
					continue;

				do {
					solomon_read_hw_info(ftdev->client,
							&info);
					if (info != 0)
						break;

					mdelay(10);
				} while ((limit--) > 0);
			} while (info == 0 && retry1-- > 0);

			if (info == 1)
				SOLOMON_WARNNING("HW Cal Success!!");
			else
				SOLOMON_WARNNING("HW Cal Fail!!");
		}
	}

	return 0;
}

static int solomon_pre_init(struct solomon_device *ftdev)
{
	int ret = 0;
#if defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE)
	const char *fw_name = FW_BOOTUP_FORCE_FULL_PATH;
	int err = 0;
#elif defined(SUPPORT_BOOTUP_FW_UPGRADE_BINFILE)
	const char *fw_name = FW_FULL_PATH;
	int err = 0;
#endif	
	
	SOLOMON_TIME("pi s");
	if (ds_read_boot_st(ftdev->client, (u16 *)&(ftdev->boot_flag)) < 0)
		return -1;

	SOLOMON_WARNNING("read boot st read(2) : 0x%04x", ftdev->boot_flag);

	if (ds_init_code(ftdev->client) < 0)
		return -1;

	if (solomon_get_version_boot(ftdev) < 0)
		return -1;

	if(init_ds_flag == 1)
	{
#if (defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE) || defined(SUPPORT_BOOTUP_FW_UPGRADE_BINFILE))
		err = request_firmware_nowait(THIS_MODULE, true, fw_name,
			&ftdev->client->dev, GFP_KERNEL, ftdev,
			solomon_fw_update_controller);
		if (err)
		{
			SOLOMON_WARNNING("failed to schedule firmware update\n");
			return -1;
		}
#else
#if defined(SUPPORT_BOOTUP_FW_UPGRADE_HEADER)
		solomon_firmware_pre_boot_up_check_head(ftdev);
#endif
		if (ds_clear_int(ftdev->client) < 0)
			return -1;
		if (sint_unstall(ftdev->client) < 0)
			return -1;

#ifdef SUPPORT_ESD_CHECKSUM
		ftdev->state_flag = 1;
#endif
		ftdev->work_procedure = TS_NO_WORK;

		ts_enable_irq();
#endif
		init_ds_flag = 0;
	}
	else
	{
		if (ds_clear_int(ftdev->client) < 0)
			return -1;
		if (sint_unstall(ftdev->client) < 0)
			return -1;

		ret = int_pin_check(ftdev, 200);
#ifdef SUPPORT_ESD_CHECKSUM
		ftdev->state_flag = 1;
#endif
	}
	SOLOMON_TIME("pi e");
	return ret;
}

static int solomon_init(struct solomon_device *ftdev)
{
	int ret = 0;

	SOLOMON_TIME("i s");

	ret = solomon_pre_init(ftdev);

	if (ret < 0) {
		SOLOMON_WARNNING("Pre init failed");
		return ret;
	}

	ret = solomon_init_config(ftdev);
	if (ret < 0) {
		SOLOMON_WARNNING("Read config failed");
		return ret;
	}

	SOLOMON_TIME("i e");
	return ret;
}

static int solomon_hw_init(void)
{
	struct device_node *node = NULL;
	int ret = 0;
	int ints[2] = { 0, 0 };
	int ints1[2] = { 0, 0 };

	tpd_gpio_as_int(tpd_int_gpio_number);
	node = of_find_matching_node(node, touch_of_match);
	if (node) {
		of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_set_debounce(ints[0], ints[1]);

		of_property_read_u32_array(node, "interrupts", ints1, ARRAY_SIZE(ints1));
		//tpd_int_gpio_number = ints1[0];
		tpd_intr_type = ints1[1];

		touch_irq = irq_of_parse_and_map(node, 0);

		TPD_DMESG("Device tpd_intr_type = %d!\n", tpd_intr_type);
		TPD_DMESG("tpd_intr_type = %d!IRQF_TRIGGER_LOW\n", tpd_intr_type);
	} else {
		TPD_DMESG("tpd request_irq can not find touch eint device node!.\n");
		ret = -1;
	}
	TPD_DMESG("[%s]irq:%d, debounce:%d-%d:\n", __func__, touch_irq, ints[0], ints[1]);

#if 0 //defined(SUPPORT_TOUCH_RESET_PIN_CTL)	
	ret = gpio_request(P_GPIO_CTP_RST_PIN, "tpd_rst");
	if (ret)
		TPD_DMESG("[ssd20xx] solomon_hw_init : gpio_request (%d)fail\n", P_GPIO_CTP_RST_PIN);

	ret = gpio_direction_output(P_GPIO_CTP_RST_PIN, 0);
	if (ret)
		TPD_DMESG("[ssd20xx] solomon_hw_init : gpio_direction_output (%d)fail\n",
		       P_GPIO_CTP_RST_PIN);

	GTP_GPIO_OUTPUT(GTP_RST_PORT, 0);

	tpd_rst_gpio_number = P_GPIO_CTP_RST_PIN;

	TPD_DMESG("tpd_rst_gpio_number %d\n", tpd_rst_gpio_number);
	TPD_DMESG("tpd_int_gpio_number %d\n", tpd_int_gpio_number);
#endif
	return 0;
}

static void solomon_hw_deint(void)
{
#if defined(SUPPORT_TOUCH_RESET_PIN_CTL)
	if (gpio_is_valid(misc_dev->reset_pin))
		gpio_free(misc_dev->reset_pin);
#endif
	if (gpio_is_valid(misc_dev->int_pin))
		gpio_free(misc_dev->int_pin);
}
static int solomon_get_rawdata(struct solomon_device *ftdev, u8 *rawdata,
	int size)
{
	int err = 0;
	int sz = 0, len = 0;
	u8 *buffer = NULL;
	int max = 50;

	buffer = rawdata;
	sz = size;
	len = ftdev->ftconfig->i2c_length;	/* I2C length */

	while (sz > 0) {
		if (sz <= ftdev->ftconfig->i2c_length)
			len = sz;

		sz -= ftdev->ftconfig->i2c_length;
		err = ts_read_data(ftdev->client,
			SOLOMON_RAW_DATA, buffer, len);

		if (err < 0) {
			SOLOMON_DEBUG("error : read raw data");
			break;
		}

		buffer = buffer + len;
		if (--max < 0) {
			err = -EAGAIN;
			break;
		}
	}

	return err;
}

static int solomon_get_rawdata_by_queue(struct solomon_device *ftdev)
{
	int err = 0;
	u8 *buffer = NULL;
	int sz = 0;
	struct solomon_data *data = NULL;

	data = ftdev->ftdata;

	buffer = get_rear_queue_buff(data);

	if (buffer == NULL) {
		SOLOMON_WARNNING("rawdata queue is full");
		err = -EAGAIN;
		goto out;
	}

	sz = (ftdev->ftconfig->x_node * ftdev->ftconfig->y_node) * 2;

	err = solomon_get_rawdata(ftdev, buffer, sz);

	put_queue(data);

out:
	return err;
}

static int solomon_read_mptest(struct solomon_device *ftdev)
{
	int err = 0;

	solomon_get_rawdata_by_queue(ftdev);

	if (m_mp_skip_count > 0) {
		SOLOMON_WARNNING("rawdata skip!!");
		get_queue(ftdev->ftdata);
		if (m_mp_skip_count == 1) {
			ftdev->ftdata->queue_front = 0;
			ftdev->ftdata->queue_rear = 0;
		}
		m_mp_skip_count--;
		return err;
	}

	m_mp_cur_count--;

	if ((ftdev->mptest_mode & MPTEST_READY_STOP) == MPTEST_READY_STOP) {
		SOLOMON_WARNNING("STOP MPTEST");
		solomon_set_mptest(ftdev, MPTEST_STOP);	/* change MP mode */
		sint_unstall(ftdev->client);	/* TMC sw reset */
	} else {
		if (m_mp_cur_count <= 0) {
			SOLOMON_WARNNING("UP key (count:%d)", m_mp_cur_count);
			if (m_mp_needback > 0) {
				/* wake up AP */
				solomon_send_key_du(tpd->dev,
								KEY_BACK);
				mdelay(m_mp_needback);
			}
			/* send get rawdata event key to APK */
			solomon_send_key_du(tpd->dev, 98);

			m_mp_cur_count = m_mp_total_count;
		}
	}

	return err;
}

static int solomon_read_mpdata(struct solomon_device *ftdev,
	struct solomon_data *data)
{
	int err = 0;

	if (!ftdev || !data)
		return -EFAULT;

	if (gpio_get_value(ftdev->int_pin)) {
		/*interrupt pin is high, not valid data.*/
		SOLOMON_WARNNING("read rawdata... inturrpt pin is high");
		return 0;
	}

	if (down_trylock(&ftdev->raw_data_lock)) {
		SOLOMON_WARNNING("fail to occupy sema(%s)", __func__);
		goto out_2;
	}

#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		SOLOMON_DEBUG("esd timer stop");
		esd_timer_stop(ftdev);
	}
#endif	/* ESD_TIMER_ENABLE */

	err = ts_read_data(ftdev->client,
		SOLOMON_STATUS_LENGTH, (u8 *)(&data->point_info), 2);

	if (err < 0) {
		SOLOMON_WARNNING("error read mpdata info using i2c.-");
		goto out_2;
	}

	if ((ftdev->mptest_mode & MPTEST_READY_START) == MPTEST_READY_START) {
		if ((data->point_info & 0x8000) == 0) {
			sint_unstall(ftdev->client);
			mdelay(20);
			return 0;
		}

		err = solomon_set_mptest(ftdev,
			(ftdev->mptest_mode&(~MPTEST_READY_START)));

		if (err < 0) {
			sint_unstall(ftdev->client);
			goto out;
		} else {
			ftdev->mptest_mode &= (~MPTEST_READY_START);
		}

		goto out;
	}

	err = solomon_read_mptest(ftdev);

out:
#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		esd_checktime_init(ftdev);
		esd_timer_start(SOLOMON_CHECK_ESD_TIMER, ftdev);
		SOLOMON_DEBUG("esd timer start");
	}
#endif	/* ESD_TIMER_ENABLE */
out_2:
	int_clear_cmd(ftdev->client);
	udelay(DELAY_FOR_SIGNAL_DELAY);
	up(&ftdev->raw_data_lock);

	return err;
}

static int solomon_read_rawdata(struct solomon_device *ftdev,
		struct solomon_data *data, u8 *check)
{
	int err = 0;
	u32 total_node;
	int sz = 0;
	char *buffer, *buffer2;
	int len = MAX_RAWDATA_BUFFER_SIZE;
	int max = 10;
	*check = 0x00;

	if (!ftdev || !data)
		return -EFAULT;

	if (gpio_get_value(ftdev->int_pin)) {
		/*interrupt pin is high, not valid data.*/
		SOLOMON_DEBUG("read rawdata... inturrpt pin is high");
		return 0;
	}

	if (down_trylock(&ftdev->raw_data_lock)) {
		SOLOMON_WARNNING("fail to occupy sema(%s)", __func__);
		goto out_2;
	}

#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		SOLOMON_DEBUG("esd timer stop");
		esd_timer_stop(ftdev);
	}
#endif	/* ESD_TIMER_ENABLE */

	err = ts_read_data(ftdev->client,
			SOLOMON_STATUS_LENGTH, (u8 *)(&data->point_info), 2);

	if (err < 0) {
		SOLOMON_WARNNING("error read point info using i2c.-");
		goto out_2;
	}

	buffer = get_rear_queue_buff(data);

	if (buffer == NULL) {
		SOLOMON_WARNNING("rawdata queue is full");
		goto out;
	}

	buffer2 = buffer;
	total_node = ftdev->ftconfig->x_node * ftdev->ftconfig->y_node;
	sz = total_node*2;
	len = ftdev->ftconfig->i2c_length;	/* I2C length */

	while (sz > 0) {
		if (sz <= ftdev->ftconfig->i2c_length)
			len = sz;

		sz -= ftdev->ftconfig->i2c_length;
		err = ts_read_data(ftdev->client,
				SOLOMON_RAW_DATA, buffer, len);

		if (err < 0) {
			SOLOMON_WARNNING("error : read raw data");
			goto out;
		}

		buffer = buffer + len;
		if (--max < 0) {
			err = -EAGAIN;
			break;
		}
	}

	put_queue(data);
	ftdev->update = 1;

out:
#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		esd_checktime_init(ftdev);
		esd_timer_start(SOLOMON_CHECK_ESD_TIMER, ftdev);
		SOLOMON_DEBUG("esd timer start");
	}
#endif	/* ESD_TIMER_ENABLE */
out_2:
	int_clear_cmd(ftdev->client);
	udelay(DELAY_FOR_SIGNAL_DELAY);
	up(&ftdev->raw_data_lock);

	return err;
}

/* send android key event to OS */
static int solomon_send_key(struct input_dev *input, unsigned int key,
		int level)
{
	if (input == NULL)
		return -1;

	input_report_key(input, key, level);
	input_sync(input);

	return 0;
}

static int solomon_send_key_du(struct input_dev *input, unsigned int key)
{
	if (input == NULL)
		return -1;

	solomon_send_key(input, key, 1);
	solomon_send_key(input, key, 0);

	return 0;
}

#ifdef SUPPORT_KEY_BUTTON
static int ssd20xx_send_key(struct solomon_device *ftdev, u16 keydown,
	u16 keyup)
{
	int err = 0;
	int i = 0;
	u16 key = 0x00;
	struct input_dev *input = NULL;
	struct solomon_data *data = NULL;

	data = ftdev->ftdata;
	input = tpd->dev;

	SOLOMON_WARNNING("KEYDOWN : 0x%04x, KEYUP : 0x%04x", keydown, keyup);

	/* key down */
	key = keydown;

	for (i = 0; i < MAX_DEVICE_KEYMAP_SIZE; i++) {
		if ((key & 0x01)) {
			SOLOMON_WARNNING("down key = 0x%02x", m_key_map[i]);
			solomon_send_key(input, m_key_map[i], 1);
			data->keydata |= (key << i);
		}

		key >>= 1;
	}

	/* key up */
	key = keyup;

	for (i = 0; i < MAX_DEVICE_KEYMAP_SIZE; i++) {
		if ((key & 0x01)) {
			if ((data->keydata&(key << i)) == 0)
				SOLOMON_WARNNING("up[0x%02x] error!",
					m_key_map[i]);
			else {
				SOLOMON_WARNNING("up[0x%02x]", m_key_map[i]);
				solomon_send_key(input, m_key_map[i], 0);
				data->keydata &= ~(key << i);
			}
		}

		key >>= 1;
	}

	return err;
}
#ifndef SUPPORT_PROTOCOL_6BYTES
static int solomon_read_keydata_android(struct solomon_device *ftdev)
{
	int err = 0;
	u16 keydata = 0x00;

	SOLOMON_WARNNING("key start");
	if (!ftdev || !ftdev->ftdata)
		return -EFAULT;

	err = ts_read_keydata(ftdev->client, (u8 *)(&keydata));

	if (err < 0) {
		SOLOMON_WARNNING("Fail read keydata!!");
		return err;
	}

	err = ssd20xx_send_key(ftdev, (keydata&0xff), ((keydata>>8)&0xff));

	return err;
}
#endif	/* SUPPORT_PROTOCOL_6BYTES */

#endif	/* SUPPORT_KEY_BUTTON */

#ifdef SUPPORT_LPM
static ssize_t ssl_gesture_android(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{

	input_report_key(tpd->dev, KEY_BACK, 1);
	input_sync(tpd->dev);

	return count;
}

static DEVICE_ATTR(gesture, S_IRUGO, NULL, ssl_gesture_android);

#ifdef SUPPORT_GESTURE_COORDINATE
static int ssd20xx_read_gesture_coord(struct solomon_device *ftdev,
	int coordLen, u16 *pCoords)
{
	int err = 0;
	u16 sz = MAX_RAWDATA_BUFFER_SIZE;
	u16 *temp = NULL;
	int length = 0;

	if (length > MAX_GESTURE_COORDINATE_SIZE)
		length = MAX_GESTURE_COORDINATE_SIZE;

	temp = pCoords;
	sz = ftdev->ftconfig->i2c_length;
	length = coordLen;

	while (length > 0) {
		if (length <= ftdev->ftconfig->i2c_length)
			sz = length;

		err = solomon_read_gesture_coordinate(ftdev->client,
			sz, temp);

		if (err < 0) {
			SOLOMON_WARNNING("error : gesture");
			return -2;
		}

		length -= ftdev->ftconfig->i2c_length;
		temp = temp+(sz/2);
	}

	return 0;
}
#endif	/* SUPPORT_GESTURE_COORDINATE */

static int ssd20xx_gesture(struct solomon_device *ftdev, u16 gesture,
	int length)
{
	int err = 0;
#ifdef SUPPORT_GESTURE_COORDINATE
	int i = 0;
	u16 arrCoords[MAX_GESTURE_COORDINATE_SIZE] = {0,};
#endif
	unsigned int gesture_key = 0x00;
	unsigned int gesture_key2 = 0x00;

	SOLOMON_DEBUG("KNOCK CODE : 0x%04x\n", (gesture&0x00FF));
	SOLOMON_DEBUG("KNOCK STATUS : 0x%04x\n", ((gesture&0xFF00)>>8));

		if ((gesture & GESTURE_STATUS_KNOCK_ON) > 0)
			gesture_key = KEY_BACK;
		else if ((gesture &
					(GESTURE_STATUS_GESTURES |
					 GESTURE_STATUS_KNOCK_CODE)) > 0) {
			gesture_key = KEY_BACK;
			gesture_key2 = gesture & 0xFF;
#ifdef SUPPORT_GESTURE_COORDINATE
		if (length < 1)
			goto end_gesture;

			if (length > MAX_GESTURE_COORDINATE_SIZE)
				length = MAX_GESTURE_COORDINATE_SIZE;

		ssd20xx_read_gesture_coord(ftdev, length, arrCoords);
#if 0
			SOLOMON_WARNNING("= Gesture Coordinate =");
		for (i = 0; i < length/4; i++)
				SOLOMON_WARNNING("X : %d \t Y : %d",
				arrCoords[i * 2 + 0],
				arrCoords[i * 2 + 1]);

			SOLOMON_WARNNING("==========\n\n");
#endif
#endif	/* SUPPORT_GESTURE_COORDINATE */
	} else if ((gesture & GESTURE_STATUS_PALM_REJECT) > 0) {
		solomon_report_release(ftdev);
		goto out;
		} else {
			err = -1;
			goto out;
		}
#ifdef SUPPORT_GESTURE_COORDINATE
end_gesture:
#endif
		if (lpm_end(ftdev) < 0)
			goto out;

		m_power_status = LPM_SUSPEND_DELAY;

		solomon_send_key(tpd->dev, gesture_key, 1);
		solomon_send_key(tpd->dev, gesture_key, 0);

		if (gesture_key2 > 0) {
			/* It is just the demo. You must change keymap */
#ifdef SUPPORT_GESTURE_DEMO
		m_gesture_value = (int)gesture_key2;
		SOLOMON_DEBUG("GESTURE : 0x%02x", m_gesture_value);

			/* android do missing a event. So, need to delay */
			mdelay(100);

			solomon_send_key(tpd->dev, 97, 1);
			solomon_send_key(tpd->dev, 97, 0);
#endif	/* SUPPORT_GESTURE_DEMO */
		}
out:
	return err;
}

#ifndef SUPPORT_PROTOCOL_6BYTES
static int solomon_read_gesture(struct solomon_device *ftdev)
{
	int err = 0;
	u16 arrGesture[GESTURE_READ_SIZE] = {0,};

	err = ts_read_gesture(ftdev->client,
		arrGesture, GESTURE_READ_SIZE * 2);

	if (err > 0)
		err = ssd20xx_gesture(ftdev, arrGesture[0], arrGesture[1]);

	return err;
}
#endif
#endif	/* SUPPORT_LPM */

#ifdef SUPPORT_AUX
static int solomon_read_aux(struct i2c_client *client, u16 *aux)
{
	int err = 0;
	*aux = 0x00;

	err = ts_read_data(client, SOLOMON_AUX, (u8 *)(aux), 2);
	if (err < 0)
		SOLOMON_WARNNING("error read AUX info using i2c.-");

	SOLOMON_DEBUG("Read AUX info : 0x%04x", *aux);

	return err;

}
#endif	/* SUPPORT_AUX */

static int solomon_check_skiptime(unsigned long msecs)
{
	int err = 0;

	SOLOMON_DEBUG("jiffies : %lu, skip : %lu, msec : %d", jiffies,
		m_point_skip_time,
		jiffies_to_msecs(jiffies - m_point_skip_time));

	if (jiffies_to_msecs(jiffies - m_point_skip_time) < msecs) {
		SOLOMON_WARNNING("point1 data skip time(%lu)",
			m_point_skip_time);
		m_point_skip_time = 0;
		err = -1;
	} else
		m_point_skip_time = 0;

	return err;
}


static int solomon_touch_down_up(int id, int xpos, int ypos,
	int width, int isdown)
{
	SOLOMON_DEBUG("[%d] X=%d, Y=%d, W=%d", id, xpos, ypos, width);
	if (isdown) {
#if defined(SUPPORT_MT_PROTOCOL_B)
		input_mt_slot(tpd->dev, id);
		input_mt_report_slot_state(tpd->dev,
			MT_TOOL_FINGER, true);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, xpos);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, ypos);
		input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR,
			width);
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, width);
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR,
			width);
#else
		input_report_abs(tpd->dev, ABS_MT_TOUCH_MAJOR, 1);
		input_report_abs(tpd->dev, ABS_MT_POSITION_X, ypos);
		input_report_abs(tpd->dev, ABS_MT_POSITION_Y, (1080-xpos));
		input_report_abs(tpd->dev, ABS_MT_WIDTH_MAJOR,
			width);
		input_report_abs(tpd->dev, ABS_MT_PRESSURE, width);
		input_report_key(tpd->dev, BTN_TOUCH, 1);
		input_report_key(tpd->dev, BTN_TOOL_FINGER, 1);
		input_report_abs(tpd->dev, ABS_MT_TRACKING_ID, id);
		input_mt_sync(tpd->dev);
#endif
	} else {
#if defined(SUPPORT_MT_PROTOCOL_B)
		input_mt_slot(tpd->dev, id);
		input_mt_report_slot_state(tpd->dev,
			MT_TOOL_FINGER, false);
#else
		input_report_key(tpd->dev, BTN_TOUCH, 0);
		input_report_key(tpd->dev, BTN_TOOL_FINGER, 0);
		input_mt_sync(tpd->dev);
#endif
	}
	return 0;
}

#ifdef SUPPORT_PROTOCOL_6BYTES
static int ssd20xx_send_axis(struct solomon_device *ftdev, ssd_data axis)
{
	struct solomon_data *data = ftdev->ftdata;
	u8 id, w;
	u16 x, y;

	id = (axis.point.id);
	x = (((axis.point.x_y_msb & 0xf0) << 4) | axis.point.x_lsb);
	y = (((axis.point.x_y_msb & 0x0f) << 8) | axis.point.y_lsb);
	w = (axis.point.weight);

	/* force part */
	if ((axis.point.forceDeviceID & PASS_DATA_FORCE_DOWN_MASK))
		SOLOMON_WARNNING("Force down");

	if ((axis.point.forceDeviceID & PASS_DATA_FORCE_UP_MASK))
		SOLOMON_WARNNING("Force up");

	if (w) {
		solomon_touch_down_up(id, x, y, w, 1);
		data->lastValidCnt++;
		m_up_event[id] = 1;
	} else {
		solomon_touch_down_up(id, x, y, w, 0);
		m_up_event[id] = 0;
	}

	return 0;
}

static int solomon_report(struct solomon_device *ftdev)
{
	int i = 0;
	int count = 0, axisCnt = 0;
	ssd_data *points = NULL;
	struct solomon_data *data = ftdev->ftdata;
	struct input_dev *input = tpd->dev;

	if (!ftdev || !ftdev->ftdata)
		return -EFAULT;

	points = ftdev->ftdata->points;

	if (points == NULL) {
		SOLOMON_WARNNING("data for parse is null");
		return -1;
	}

	count = (ftdev->ftdata->point_info & 0x00ff) / sizeof(ssd_data);
	SOLOMON_DEBUG("union solomon_data size = %d", (int)sizeof(ssd_data));

	if (count == 0) {
		SOLOMON_DEBUG("length zero release");
		solomon_report_release(ftdev);
		return -1;
	}
	data->lastValidCnt = 0;

	for (i = 0; i < count; i++) {
		if (points[i].gesture.id == PASS_DATA_ID_GESTURE) {
			/* data ID is gesture */
			ssd20xx_gesture(ftdev, ((points[i].gesture.status<<8)
				| points[i].gesture.gesture),
				((points[i].gesture.coordLengthH<<8)
				| points[i].gesture.coordLengthL));
		} else if (points[i].key.id == PASS_DATA_ID_KEY) {
			/* data ID is key */
#ifdef SUPPORT_KEY_BUTTON
			ssd20xx_send_key(ftdev, ((points[i].key.keydownH << 8)
				| points[i].key.keydownL),
				((points[i].key.keyupH << 8)
				| points[i].key.keyupL));
#endif
		} else if (points[i].point.id >= PASS_DATA_ID_AXIS_FIRST
			&& points[i].point.id <= PASS_DATA_ID_AXIS_LAST) {
			/* data ID is axis */
			ssd20xx_send_axis(ftdev, points[i]);
			axisCnt++;
		} else if (points[i].aux.id == PASS_DATA_ID_AUX) {
			/* data ID is AUX. It doesn't support yet. */
			SOLOMON_WARNNING("AUX ID detected!!");
		} else {
			SOLOMON_WARNNING("%d doesn't supported ID(0x%02x)",
			(points[i].point.forceDeviceID &
			PASS_DATA_DEVICEID_MASK),
			points[i].point.id);
		}
	}

	if (axisCnt > 0) {
		input_mt_report_pointer_emulation(input, true);
		input_sync(input);
	}

	return 0;
}
#else
/* send a touch point to AP */
static int solomon_report(struct solomon_device *ftdev)
{
	int i = 0;
	u8 status, length, id, w;
	u16 x, y;
	struct solomon_data *data = ftdev->ftdata;
	struct input_dev *input = tpd->dev;

	if (!ftdev || !ftdev->ftdata)
		return -EFAULT;

	status = (data->point_info & 0xff00) >> 8;
	length = (data->point_info & 0x00ff) / 6;

	if (length == 0) {
		SOLOMON_DEBUG("length zero release");
		solomon_report_release(ftdev);
	} else {
		data->lastValidCnt = 0;

		for (i = 0; i < length; i++) {
			id = (data->points[i].point.id);
			x = (((data->points[i].point.x_y_msb & 0xf0) << 4) |
				data->points[i].point.x_lsb);
			y = (((data->points[i].point.x_y_msb & 0x0f) << 8) |
				data->points[i].point.y_lsb);
			w = (data->points[i].point.weight);

			SOLOMON_DEBUG("%d %dx%d %d", id, x, y, w);
			if (w) {
				solomon_touch_down_up(id, x, y, w, 1);
				data->lastValidCnt++;
				m_up_event[id] = 1;
			} else {
				if (m_up_event[id] == 1) {
					SOLOMON_DEBUG("[ID:%d] UP event", id);
					solomon_touch_down_up(id, x, y, w, 0);
				} else {
					SOLOMON_DEBUG("[ID:%d]unknown UP", id);
				}
				memset(&data->points[i], 0x0,
						sizeof(ssd_data));
			}
		}
		input_mt_report_pointer_emulation(input, true);
		input_sync(input);
		/* input_sync(input); */	/* sync again for test */
	}

	return 0;
}
#endif	/* SUPPORT_PROTOCOL_6BYTES */

static int solomon_report_release(struct solomon_device *ftdev)
{
	int i = 0;
	struct solomon_data *data = ftdev->ftdata;

	if (!ftdev || !ftdev->ftdata)
		return -EFAULT;

	if (data->lastValidCnt <= 0)
		return 0;

	SOLOMON_DEBUG("Point release(%d)", data->lastValidCnt);

	for (i = 0; i < SOLOMON_MAX_POINT; i++) {
		if (m_up_event[i] == 1) {
			SOLOMON_DEBUG("[ID:%d] UP event", i);
			solomon_touch_down_up(i, 0, 0, 0, 0);
			m_up_event[i] = 0;
		} else
			SOLOMON_DEBUG("[ID:%d] unknown UP event", i);
	}

	input_sync(tpd->dev);

	data->lastValidCnt = 0;

	return 0;
}

static int solomon_read_points(struct solomon_device *ftdev,
		struct solomon_data *data)
{
	int err = 0;
	u8 status, length;
#ifdef SUPPORT_AUX
	u16 aux = 0;
#endif	/* SUPPORT_AUX */

#ifdef SUPPORT_ESD_CHECKSUM
	u16 esd_snl[3 * SOLOMON_MAX_POINT + 2] = {0x00,};
	u8 xor = 0x00;
	u8 sum = 0x00;
	int retry = 3;
#endif	/* SUPPORT_ESD_CHECKSUM */

	if (!ftdev || !data)
		return -EFAULT;

	SOLOMON_DEBUG("solomon_read_points");

	if (gpio_get_value(ftdev->int_pin)) {
		/*interrupt pin is high, not valid data.*/
		SOLOMON_DEBUG("read points... interrupt pin is high");
		return -EAGAIN;
	}

#if ESD_TIMER_ENABLE
	ftdev->esd_check_time = jiffies;

	if (ftdev->use_esd_tmr) {
		SOLOMON_DEBUG("esd timer stop");
		esd_timer_stop(ftdev);
	}
#endif	/* ESD_TIMER_ENABLE */

#ifdef SUPPORT_ESD_CHECKSUM
retry_snl:
	if (retry-- < 1)
		goto out_reset;

	err = ts_read_data(ftdev->client,
		SOLOMON_STATUS_LENGTH, (u8 *)(esd_snl), 4);

	if (esd_snl[0] == 0x0AF0) {
		SOLOMON_WARNNING("SNL read again use 0x00F0!!");
		err = ts_read_data(ftdev->client,
		0x00F0, (u8 *)(esd_snl), 4);
	}
#else
	err = ts_read_data(ftdev->client,
			SOLOMON_STATUS_LENGTH, (u8 *)(&data->point_info), 2);
	/* for test */
	if (data->point_info == 0x0AF0) {
		SOLOMON_WARNNING("SNL read again use 0x00F0!!");
		err = ts_read_data(ftdev->client,
				0x00F0, (u8 *)(&data->point_info), 2);
	}
#endif	/* SUPPORT_ESD_CHECKSUM */

	if (err < 0) {
		SOLOMON_WARNNING("error read point info using i2c.-");
		goto out;
	}

#ifdef SUPPORT_ESD_CHECKSUM
	if (ftdev->state_flag > 0) {
		SOLOMON_DEBUG("ESD S&L : 0x%04x \t CHECKSUM : 0x%04x",
			esd_snl[0], esd_snl[1]);

	esd_checksum((u8 *)(esd_snl), 2, &xor, &sum);

		if (xor != ((esd_snl[1] >> 8) & 0xFF)) {
			SOLOMON_WARNNING("S&L CS XOR[0x%02x : 0x%02x] fail!!",
				xor, ((esd_snl[1]>>8)&0xFF));

		goto retry_snl;
	}

		if (sum != ((esd_snl[1]) & 0xFF)) {
			SOLOMON_WARNNING("S&L CS SUM[0x%02x : 0x%02x] fail!!",
				sum, ((esd_snl[1])&0xFF));

			goto retry_snl;
		}
	}

	data->point_info = esd_snl[0];
#endif	/* SUPPORT_ESD_CHECKSUM */

	if (data->point_info == 0x0000) {
#if ESD_TIMER_ENABLE
		esd_checktime_init(ftdev);
#endif	/* ESD_TIMER_ENABLE */
		err = -1;
		goto out;
	}

	status = (data->point_info & 0xff00) >> 8;
	length = (data->point_info & 0x00ff);

	if ((status&0xF0) > 0 && m_point_skip_time > 0) {

		err = solomon_check_skiptime(32);

		if (err < 0)
			goto out;
	}

#ifdef SUPPORT_ESD_CHECKSUM
	sum = sizeof(esd_snl) - 4;
#else
	sum = sizeof(data->points);
#endif
	SOLOMON_DEBUG("MAX points size: %d", sum);

	if (length > sum) {
		SOLOMON_WARNNING("state:0x%02x length(%d) over the MAX(%d)!!",
			status, length, sum);
		if (m_point_skip_time > 0) {
			err = solomon_check_skiptime(32);

			if (err < 0)
				goto out;
		}
		length = sum;
	}
	sum = 0;

	SOLOMON_DEBUG("status : 0x%x, %d", status, length);

	/* Gesture check */
#if defined(SUPPORT_LPM) && !defined(SUPPORT_PROTOCOL_6BYTES)
	if ((status&STATUS_CHECK_PALM_GESTURE) == STATUS_CHECK_PALM_GESTURE)
		err = solomon_read_gesture(ftdev);

	if (err < 0)
				goto out_go;
#endif	/* SUPPORT_LPM */

#ifdef SUPPORT_AUX
	if ((status&STATUS_CHECK_AUX) == STATUS_CHECK_AUX) {
		SOLOMON_DEBUG("AUX bit set!!");

		if (solomon_read_aux(ftdev->client, &aux) >= 0) {

			if ((aux&SOLOMON_AUX) == SOLOMON_AUX) {
#ifdef SUPPORT_ES2
				if (ftdev->es_version == DS_VERSION_ES2)
					goto out_reset;
				else if (ftdev->es_version == DS_VERSION_ES1) {
#endif
				/* TMC reseted. Need TMC init; */
				SOLOMON_WARNNING("Auxbit reseted");
				solomon_report_release(ftdev);
				err = solomon_pre_init(ftdev);

				if ((ftdev->boot_flag & 0x7FFF) > 0)
					goto checksum;

				if (err < 0)
					SOLOMON_WARNNING("Pre init failed");
#ifdef SUPPORT_ES2
				}
#endif
				return err;
#ifdef SUPPORT_ES2
			} else if (aux == AUX_BOOTUP_RESET) {
				SOLOMON_WARNNING("Aux bootup reset");
				solomon_report_release(ftdev);
				err = solomon_pre_init(ftdev);

				if ((ftdev->boot_flag & 0x7FFF) > 0)
					goto checksum;

				return err;
			} else if (aux == AUX_WDTDS_INT) {
				/* ftdev->checksum_flag = aux; */
				goto out_reset;
#endif
			} else if (aux == AUX_NEED_C1_TMC_RESET) {
				/* Need TMC reset */
				SOLOMON_WARNNING("Need TMC reset");
				goto out_reset;
			} else if (aux == AUX_NEED_C2_DIC_RESET) {
				/* TODO : for D-IC Reset */
				SOLOMON_WARNNING("Need D-IC Reset");
#ifdef SUPPORT_MTK_ESD_RECOVERY
				primary_display_esd_recovery();
				primary_display_suspend();
				primary_display_resume();
				goto out_reset;
#endif
			} else if (aux == AUX_NEED_C3_DIC_POWER) {
				/* TODO : for D-IC Power ON/OFF */
				SOLOMON_WARNNING("Need D-IC Power On/Off");
#ifdef SUPPORT_MTK_ESD_RECOVERY
				primary_display_esd_recovery();
				primary_display_suspend();
				primary_display_resume();
				goto out_reset;
#endif
			} else if (aux == AUX_NEED_C4_DSV_POWER_RESET) {
				/* TODO : for DSV power & reset */
				SOLOMON_WARNNING("Need DSV power & reset");
#ifdef SUPPORT_MTK_ESD_RECOVERY
				primary_display_esd_recovery();
				primary_display_suspend();
				primary_display_resume();
				goto out_reset;
#endif
			} else if ((aux & AUX_ESD_DETECT) == AUX_ESD_DETECT) {
				/* need ESD process */
				/* ftdev->checksum_flag = aux; */
				goto out_reset;
			} else if ((aux & AUX_CS_ERROR) == AUX_CS_ERROR) {
				/* cs error. same ESD process */
				SOLOMON_WARNNING("Aux CS error");
				ftdev->checksum_flag = aux;
				goto checksum;
			}
		}
	}
#endif	/* SUPPORT_AUX */

#if defined(SUPPORT_KEY_BUTTON) && !defined(SUPPORT_PROTOCOL_6BYTES)
	if ((status & STATUS_CHECK_KEY) == STATUS_CHECK_KEY) {
		SOLOMON_DEBUG("KEY bit set!!");
		solomon_read_keydata_android(ftdev);
	}
#endif	/* SUPPORT_KEY_BUTTON */

#ifdef STATUS_CHECK_BOOT_ST
	if ((status & STATUS_CHECK_BOOT_ST) == STATUS_CHECK_BOOT_ST) {
		SOLOMON_WARNNING("Need TMC config initialize.");
		solomon_report_release(ftdev);
		err = solomon_init_config(ftdev);

		if (err < 0)
			SOLOMON_WARNNING("TMC config initialize failed");
		else
		{
			if(init_tmc_flag == 1)
			{
#if 0				
				/* After TMC init */
				input_set_abs_params(misc_dev->input_dev, ABS_MT_POSITION_X, 0,
						ftdev->ftconfig->max_x, 0, 0);
				input_set_abs_params(misc_dev->input_dev, ABS_MT_POSITION_Y, 0,
						ftdev->ftconfig->max_y, 0, 0);
				input_set_abs_params(misc_dev->input_dev, ABS_MT_PRESSURE, 0,
						ftdev->ftconfig->max_weight, 0, 0);
				SOLOMON_WARNNING("max_weight : %d", ftdev->ftconfig->max_weight);
				err = input_register_device(tpd->dev);			
				if (err)
				{
					SOLOMON_WARNNING("Register input failed");
					err = -1;
				}
#endif
				init_tmc_flag = 0;
			} /*(init_tmc_flag == 1) */
		}

		goto out;
	}
#endif	/* STATUS_CHECK_BOOT_ST */

#ifdef SUPPORT_PROTOCOL_6BYTES
	if (length > 0 && ((status & 0x7F) > 0)) {
#else
	if (length > 0 && ((status & 0x0F) > 0)) {
#endif
#ifdef SUPPORT_ESD_CHECKSUM
		retry = 3;
retry_point:
		if (retry-- < 1)
			goto out_reset;

		err = ts_read_data(ftdev->client,
				SOLOMON_POINT_DATA, (u8 *)(esd_snl), length + 2);
#else
		err = ts_read_data(ftdev->client,
				SOLOMON_POINT_DATA, (u8 *)(&data->points), length);
#endif	/* SUPPORT_ESD_CHECKSUM */
		if (err < 0) {
			SOLOMON_WARNNING("error read point info using i2c.\n");
			goto out;
		}
#ifdef SUPPORT_ESD_CHECKSUM
		esd_checksum((u8 *)(esd_snl), length, &xor, &sum);
		SOLOMON_DEBUG("XOR C: 0x%02x, T: 0x%02x",
				xor, ((esd_snl[length / 2] >> 8) & 0xFF));
		SOLOMON_DEBUG("SUM C: 0x%02x, T: 0x%02x",
				sum, ((esd_snl[length / 2]) & 0xFF));

		if (xor != ((esd_snl[length / 2] >> 8) & 0xFF)) {
			SOLOMON_WARNNING("Check Sum XOR[0x%02x:0x%02x] fail!",
					xor, ((esd_snl[length/2]>>8)&0xFF));
			goto retry_point;
		}

		if (sum != (esd_snl[length / 2] & 0xFF)) {
			SOLOMON_WARNNING("Check Sum SUM[0x%02x:0x%02x] fail!",
					sum, (esd_snl[length/2]&0xFF));
			goto retry_point;
		}
		memcpy((u8 *)(&data->points), (u8 *)esd_snl, length);
#endif	/* SUPPORT_ESD_CHECKSUM */
#ifdef SOLOMON_POINT_DATA
		SOLOMON_DEBUG("point : 0x%x, 0x%x",
				data->points[0].point.x_lsb, data->points[0].point.y_lsb);
#else
		SOLOMON_DEBUG("point : 0x%x, 0x%x",
				data->points[0].id_x, data->points[0].w_y);
#endif
	}

	if (ftdev->touch_mode >= AGING_RAW_DATA_MODE)
		solomon_get_rawdata_by_queue(ftdev);
out:
#if defined(SUPPORT_LPM) && !defined(SUPPORT_PROTOCOL_6BYTES)
out_go:
#endif
#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		esd_checktime_init(ftdev);
		esd_timer_start(SOLOMON_CHECK_ESD_TIMER, ftdev);
		SOLOMON_DEBUG("esd timer start");
	}
#endif	/* ESD_TIMER_ENABLE */

#ifdef SUPPORT_LPM
	if (m_power_status != LPM_SUSPEND_DELAY)
#endif
		int_clear_cmd(ftdev->client);

	return err;
#if defined(SUPPORT_ESD_CHECKSUM) || defined(SUPPORT_AUX)
	/* checksum fail */
checksum:
	SOLOMON_WARNNING("Fail. So, TMC boot up check!!");
	solomon_report_release(ftdev);	/* all points release */
	/* implement update */
	if (ftdev->work_procedure == TS_IN_INITIALIZE)
		err = 0;
	else {
		solomon_fw_update(ftdev);
		err = -1;
	}

#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		esd_checktime_init(ftdev);
		esd_timer_start(SOLOMON_CHECK_ESD_TIMER, ftdev);
		SOLOMON_DEBUG("esd timer start");
	}
#endif	/* ESD_TIMER_ENABLE */
	return err;
#endif	/* SUPPORT_ESD_CHECKSUM || SUPPORT_AUX */
out_reset:
	solomon_power_control(ftdev, POWER_RESET);
	/* solomon_reset(); */
	return -1;
}

/* TMC initialize */
static int solomon_init_config(struct solomon_device *ftdev)
{
	int err = 0;
	u16 val = 0x0000;
	struct i2c_client *client = ftdev->client;
	struct solomon_config *ftconfig = ftdev->ftconfig;

	SOLOMON_TIME("ic s");
	if (!client || !ftconfig) {
		SOLOMON_WARNNING("error : I2C client fail!!!");
		return -EINVAL;
	}

	/* mp test mode */
/*	if (m_mp_total_count < 0)	*/
		misc_dev->mptest_mode = MPTEST_STOP;

	m_point_skip_time = 0;

	/* touch mode */
	val = TOUCH_POINT_MODE;
	err = ts_write_data(misc_dev->client, SOLOMON_TOUCH_MODE,
			(u8 *)&(val), 2);

	if (err < 0)
		SOLOMON_WARNNING("Fail to set TOUCH_MODE %d.", val);

	solomon_get_version(ftdev, ftconfig->fw_ver);

	err = ts_read_node(client, (u8 *)&ftconfig->x_node,
			(u8 *)&ftconfig->y_node);

	if (err < 0) {
		SOLOMON_WARNNING("Get node x/y failed");
		goto init_config_failed;
	}

	err = ts_read_data(client,
			SOLOMON_X_RESOLUTION, (u8 *)&(ftconfig->max_x), 2);

	if (err < 0) {
		err = -EAGAIN;
		SOLOMON_DEBUG("error : read x resolution");
	}
	SOLOMON_DEBUG("X resolution : %d", ftconfig->max_x);

	err = ts_read_data(client,
			SOLOMON_Y_RESOLUTION, (u8 *)&(ftconfig->max_y), 2);

	if (err < 0) {
		err = -EAGAIN;
		SOLOMON_DEBUG("error : read y resolution");
	}
	SOLOMON_DEBUG("Y resolution : %d", ftconfig->max_y);

	ftconfig->max_weight = (ftconfig->max_x * ftconfig->max_y) >> 1;

	err = ts_read_data(client,
			SOLOMON_USING_POINT_NUM, (u8 *)&(ftconfig->using_point), 2);

	if (err < 0) {
		err = -EAGAIN;
		SOLOMON_DEBUG("error : read using point");
	}
	SOLOMON_DEBUG("using point num : %d", ftconfig->using_point);

#ifdef SUPPORT_TMC_I2C_LENGTH
	err = ts_read_tmc_i2c(client, &val);
	if (err < 0) {
		err = -EAGAIN;
		SOLOMON_DEBUG("error : read I2C Length");
		ftconfig->i2c_length = MAX_RAWDATA_BUFFER_SIZE;
	} else {
		ftconfig->i2c_length = val * 2;
	}
	SOLOMON_DEBUG("I2C Length : %d", ftconfig->i2c_length);
#else
	ftconfig->i2c_length = MAX_RAWDATA_BUFFER_SIZE;
#endif	/* SUPPORT_TMC_I2C_LENGTH */

#if ESD_TIMER_ENABLE
	ftdev->use_esd_tmr = 0;
	if (SOLOMON_CHECK_ESD_TIMER > 0) {
		if (SOLOMON_CHECK_ESD_TIMER == 1)
			val = 30;
		else
			val = (SOLOMON_CHECK_ESD_TIMER - 1) * 60;

		err = solomon_set_esdtime(ftdev, val);

		if (err >= 0) {
			ftdev->use_esd_tmr = 1;
			esd_checktime_init(ftdev);
			esd_timer_start(SOLOMON_CHECK_ESD_TIMER, ftdev);
			SOLOMON_DEBUG("esd timer start");
		}

	}
#endif	/* ESD_TIMER_ENABLE */

init_config_failed:
	ftconfig->check = true;
	SOLOMON_TIME("ic e");
	return err;
}

static void solomon_work(struct work_struct *work)
{
	struct solomon_device *ftdevice =
		container_of(work, struct solomon_device, work);
	int ret = 0;
	u8 check = 0x00;

	SOLOMON_DEBUG("SOLOMON_WORK(%d)", ftdevice->touch_mode);

	if (ftdevice->work_procedure == TS_IN_UPGRADE) {
		SOLOMON_DEBUG("upgrade..");
		return;
	}

	//ts_disable_irq();
	if (down_trylock(&ftdevice->work_procedure_lock)) {
		SOLOMON_WARNNING("fail to occupy sema");
		do {
			udelay(10);
		} while (down_trylock(&ftdevice->work_procedure_lock));

		int_clear_cmd(ftdevice->client);
		udelay(DELAY_FOR_SIGNAL_DELAY);

		goto out;
	}

	if (ftdevice->work_procedure != TS_NO_WORK) {
		SOLOMON_DEBUG("other process occupied..");
		udelay(DELAY_FOR_SIGNAL_DELAY);
		int_clear_cmd(ftdevice->client);
		goto out;
	}

	ftdevice->work_procedure = TS_NORMAL_WORK;

	if (ftdevice->mptest_mode > MPTEST_STOP) {
		solomon_read_mpdata(ftdevice, ftdevice->ftdata);
	} else if (ftdevice->touch_mode > TOUCH_POINT_MODE &&
			ftdevice->touch_mode < AGING_RAW_DATA_MODE) {
		solomon_read_rawdata(ftdevice, ftdevice->ftdata, &check);

		if (check > 0)
			solomon_report(ftdevice);
	} else {
		ret = solomon_read_points(ftdevice, ftdevice->ftdata);

		if (ret > 0)
			solomon_report(ftdevice);
	}

out:
	if (ftdevice->work_procedure == TS_NORMAL_WORK)
		ftdevice->work_procedure = TS_NO_WORK;

	up(&ftdevice->work_procedure_lock);
	GTP_GPIO_AS_INT(GTP_INT_PORT);
	//ts_enable_irq();
}

static int tpd_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	strcpy(info->type, TPD_DEVICE);

	return 0;
}

static irqreturn_t solomon_interrupt(int irq, void *devid)
{
	struct solomon_device *ftdevice = (struct solomon_device *)devid;
#ifdef SUPPORT_LPM__
	/* This is not use more */
	if ((m_power_status & LPM_SUSPEND_DELAY) != LPM_SUSPEND_DELAY)
#endif	/* SUPPORT_LPM */
		//queue_work(ftdevice->workqueue, &ftdevice->work);
		solomon_work(&ftdevice->work);
	return IRQ_HANDLED;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
/*
 * In early suspend, it should set tp mode to idle.
 * Then in early resume, wakeup the panel. But here must reset it.
 * But there is no reset-pin in AT711.
 * So here just stop the work queue, and disable response of interrupting.
 */
static void solomon_early_suspend(struct early_suspend *handler)
{
	struct solomon_device *ftdev = container_of(handler,
			struct solomon_device, es);

	ts_disable_irq();
	flush_workqueue(ftdev->workqueue);

	solomon_power_control(ftdev, POWER_OFF);
}

static void solomon_early_resume(struct early_suspend *handler)
{
	struct solomon_device *ftdev = container_of(handler,
			struct solomon_device, es);

	ts_enable_irq();

	solomon_power_control(ftdev, POWER_ON);
}
#endif	/* CONFIG_HAS_EARLYSUSPEND */

//#####################################################################

static int solomon_get_rawdata_str(struct solomon_device *ftdev, u16 touchmode, char* buff)
{
	int err=0;
	int sz=0, total_node=0;
	unsigned long retry_time=0;
	int len = MAX_RAWDATA_BUFFER_SIZE;
	char *buffer = NULL;

	//solomon_irq_disable();

	down(&misc_dev->work_procedure_lock);
	if (misc_dev->work_procedure != TS_NO_WORK) {
		SOLOMON_DEBUG("other process occupied.. (%d)\n", misc_dev->work_procedure);
		err = -1;
		goto out;
	}

	misc_dev->work_procedure = TS_SET_MODE;
	err = ts_write_data(misc_dev->client, SOLOMON_TOUCH_MODE, (u8 *)&(touchmode), 2);		// change touch mode
	if (err < 0) {
		SOLOMON_DEBUG("Fail to set TOUCH_MODE %d.", misc_dev->touch_mode);
		goto out_work;
	}

	retry_time = 40;
	err = -1;
	do {
		gpio_direction_input(tpd_int_gpio_number);
		if( gpio_get_value(misc_dev->int_pin) == 0 ) {				// wait int pin low
			err = 0;
			break;
		}
		msleep(50);
	} while( retry_time-- > 0 );

	if( err < 0 ) goto out_clear;

	buffer=buff;
	total_node = misc_dev->ftconfig->x_node * misc_dev->ftconfig->y_node;
	sz = total_node*2 ;
	len = ftdev->ftconfig->i2c_length;	// I2C length

	while(sz > 0) {
		if(sz <= ftdev->ftconfig->i2c_length) len = sz;
		sz -= ftdev->ftconfig->i2c_length;
		err = ts_read_data(ftdev->client, SOLOMON_RAW_DATA, buffer, len);
		if (err < 0) {
			SOLOMON_DEBUG("error : read raw data");
			goto out_clear;
		}

		buffer = buffer+len;
	}

out_clear :
	int_clear_cmd(misc_dev->client);
	touchmode = TOUCH_POINT_MODE;
	err = ts_write_data(misc_dev->client, SOLOMON_TOUCH_MODE, (u8 *)&(touchmode), 2);		// change touch mode

	if (err < 0) {
		SOLOMON_DEBUG("Fail to set TOUCH_MODE %d.", misc_dev->touch_mode);
		goto out_work;
	}
out_work :
	misc_dev->work_procedure = TS_NO_WORK;
out :
	up(&misc_dev->work_procedure_lock);
	//solomon_irq_enable();

	return err;
}

static char ssd_return_str[256000]={0};

#define SSDTOUCH_WRITE					"write"
#define SSDTOUCH_READ					"read"

#define SSDTOUCH_DEBUG					"debug"
#define SSDTOUCH_RESET					"reset"
#define SSDTOUCH_UPDATE					"update"
#define SSDTOUCH_MODE					"mode"
#define SSDTOUCH_GESTURE				"gesture"
#define SSDTOUCH_VERSION				"version"

static ssize_t solomon_set_ssdtouch_attr(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *value=NULL;
	unsigned int reg=0;
	unsigned int data=0;

	unsigned short send_reg=0;
	unsigned short send_data=0;
	char tmp_str[64]={0};
	char hex_str1[5]={0};
	char hex_str2[5]={0};
	short *rawdata;
	int i,j;

	const char *ssd_receive_buf=buf;

	if ((value=strstr(ssd_receive_buf, SSDTOUCH_WRITE)) != NULL)//write
	{
		value += strlen(SSDTOUCH_WRITE);
		if (strlen(value)<14)
		{
			sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_WRITE);
			return -EFAULT;
		}
		value += strlen(" ");
		sscanf(value, "0x%x 0x%x", &reg, &data);
		send_reg = reg;
		send_data = data;

		ts_write_data(misc_dev->client, send_reg, (unsigned char *)&(send_data), 2);

		memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
		sprintf(ssd_return_str, "write [0x%04X]=0x%04X\n", reg, data);
	}
	else if ((value=strstr(ssd_receive_buf, SSDTOUCH_READ)) != NULL) //read
	{
		value += strlen(SSDTOUCH_READ);
		if (strlen(value)<4)
		{
			sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_READ);
			return -EFAULT;
		}

		value += strlen(" ");
		sscanf(value, "0x%x", &reg);
		send_reg = reg;

		ts_read_data(misc_dev->client, send_reg, (unsigned char *)&(send_data), 2);

		memset(ssd_return_str, 0 ,sizeof(ssd_return_str));

		sprintf(ssd_return_str, "0x%04X\n", send_data);
	}
	else if ((value=strstr(ssd_receive_buf, SSDTOUCH_UPDATE)) != NULL) //update
	{
		ts_disable_irq();
		misc_dev->work_procedure = TS_IN_UPGRADE;
		solomon_firmware_update_byfile(misc_dev, FW_FORCE_FULL_PATH);
		misc_dev->work_procedure = TS_NO_WORK;
		solomon_reset();
		ts_enable_irq();
		sprintf(ssd_return_str, "%s ok!\n",SSDTOUCH_UPDATE);
	}
	else if ((value=strstr(ssd_receive_buf, SSDTOUCH_RESET)) != NULL) //reset
	{
		solomon_init(misc_dev);
		sprintf(ssd_return_str, "%s ok!\n",SSDTOUCH_RESET);
	}
	else if ((value=strstr(ssd_receive_buf, SSDTOUCH_MODE)) != NULL) //mode
	{
		value += strlen(SSDTOUCH_MODE);
		if (strlen(value)<4)
		{
			sprintf(ssd_return_str, "%s error!\n",SSDTOUCH_MODE);
			return -EFAULT;
		}
		value += strlen(" ");
		sscanf(value, "0x%x", &reg);

		memset(ssd_return_str, 0 ,sizeof(ssd_return_str));

		rawdata = (u16 *)&(misc_dev->ftdata->rawData[0][0]);
		if (solomon_get_rawdata_str(misc_dev, reg, (char*)rawdata)>=0) {
			memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "	  ");strcat(ssd_return_str, tmp_str);
			for( j=0; j<misc_dev->ftconfig->y_node; j++ ) {
				memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "[%3d] ", j+1);strcat(ssd_return_str, tmp_str);
			}
			memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "\n");strcat(ssd_return_str, tmp_str);

			for( i=0; i<misc_dev->ftconfig->x_node; i++ ) {
				memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "[%3d] ", i+1);strcat(ssd_return_str, tmp_str);
				for( j=0; j<misc_dev->ftconfig->y_node; j++ ) {
					memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "%5d ", rawdata[i*misc_dev->ftconfig->y_node+j]);strcat(ssd_return_str, tmp_str);
				}
				memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "\n");strcat(ssd_return_str, tmp_str);
			}
			memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "\n");strcat(ssd_return_str, tmp_str);
		}
	}
	else if ((value=strstr(ssd_receive_buf, SSDTOUCH_VERSION)) != NULL) //version
	{
		memset(ssd_return_str, 0 ,sizeof(ssd_return_str));
		memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "################################\n");strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, " Display Version : 0x%08x\n",misc_dev->fw_version.display_version);strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "  Driver Version : %s\n",DRIVER_VRESION);strcat(ssd_return_str, tmp_str);
		hex_str1[0] = (misc_dev->fw_version.productID01&0xFF000000)>>24;
		hex_str1[1] = (misc_dev->fw_version.productID01&0x00FF0000)>>16;
		hex_str1[2] = (misc_dev->fw_version.productID01&0x0000FF00)>>8;
		hex_str1[3] = (misc_dev->fw_version.productID01&0x000000FF)>>0;
		hex_str2[0] = (misc_dev->fw_version.productID02&0xFF000000)>>24;
		hex_str2[1] = (misc_dev->fw_version.productID02&0x00FF0000)>>16;
		hex_str2[2] = (misc_dev->fw_version.productID02&0x0000FF00)>>8;
		hex_str2[3] = (misc_dev->fw_version.productID02&0x000000FF)>>0;
		memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "      Product ID : 0x%08x(%4s) 0x%08x(%4s)\n",misc_dev->fw_version.productID01, hex_str1, misc_dev->fw_version.productID02, hex_str2);strcat(ssd_return_str, tmp_str);
		hex_str1[0] = (misc_dev->fw_version.ICName01&0xFF000000)>>24;
		hex_str1[1] = (misc_dev->fw_version.ICName01&0x00FF0000)>>16;
		hex_str1[2] = (misc_dev->fw_version.ICName01&0x0000FF00)>>8;
		hex_str1[3] = (misc_dev->fw_version.ICName01&0x000000FF)>>0;
		hex_str2[0] = (misc_dev->fw_version.ICName02&0xFF000000)>>24;
		hex_str2[1] = (misc_dev->fw_version.ICName02&0x00FF0000)>>16;
		hex_str2[2] = (misc_dev->fw_version.ICName02&0x0000FF00)>>8;
		hex_str2[3] = (misc_dev->fw_version.ICName02&0x000000FF)>>0;
		memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "         IC Name : 0x%08x(%4s) 0x%08x(%4s)\n",misc_dev->fw_version.ICName01, hex_str1, misc_dev->fw_version.ICName02, hex_str2);strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "      Resolution : %d X %d\n", misc_dev->ftconfig->max_x, misc_dev->ftconfig->max_y);strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "       Node Info : %d X %d\n", misc_dev->ftconfig->x_node, misc_dev->ftconfig->y_node);strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "     Point Count : %d\n", misc_dev->ftconfig->using_point);strcat(ssd_return_str, tmp_str);
		memset(tmp_str, 0 ,sizeof(tmp_str));sprintf(tmp_str, "#################################\n");strcat(ssd_return_str, tmp_str);
	}
	return 1;
}

static ssize_t solomon_get_ssdtouch_attr(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", ssd_return_str);
}
static DEVICE_ATTR(ssdtouch, 0660, solomon_get_ssdtouch_attr, solomon_set_ssdtouch_attr);

//#####################################################################

static struct attribute *ssl_attrs[] = {
	&dev_attr_version.attr,
#ifdef SUPPORT_LPM
	&dev_attr_gesture.attr,
#endif	/* SUPPORT_LPM */
	&dev_attr_testing.attr,
	&dev_attr_ssdtouch.attr,
	&dev_attr_touchmode.attr,
	&dev_attr_mptest.attr,
	&dev_attr_esdtime.attr,
	&dev_attr_fail_reason.attr,
	NULL,
};

static const struct attribute_group ssl_attr_group = {
	.attrs = ssl_attrs,
};

static int solomon_boot_sequence(struct solomon_device *ftdev)
{
	int err = 0;

	ftdev->work_procedure = TS_IN_INITIALIZE;

	err = solomon_power_control(ftdev, POWER_ON_SEQUENCE);

	if (err < 0) {
		SOLOMON_WARNNING("Reset Fail!!");
		goto out;
	}
#ifdef SUPPORT_ES2
	err = ds_process_version(ftdev);

	if (err < 0) {
		SOLOMON_WARNNING("process version failed");
		goto out;
	}
#endif
	/* set flag */
	ftdev->boot_flag = 0x00;
	ftdev->checksum_flag = 0x00;
	ftdev->state_flag = 0;
	/* DS init */
	err = solomon_read_points(ftdev, ftdev->ftdata);

	if (err < 0) {
		SOLOMON_WARNNING("DS init failed");
		goto out;
	}

out:
	if (ftdev->boot_flag != 0x00 || ftdev->checksum_flag != 0x00)
		return 0;

	return err;
}

static int solomon_fw_update(struct solomon_device *ftdev)
{
	const struct firmware *fw;
	const char *fw_name = FW_FULL_PATH;

	solomon_power_control(ftdev, POWER_RESET);

	if (request_firmware(&fw, fw_name, &ftdev->client->dev) != 0) {
		SOLOMON_TIME("H");
		SOLOMON_WARNNING("Going to update using Header file.");
		solomon_firmware_pre_boot_up_check_head(ftdev);
	} else {
		SOLOMON_TIME("B");
		SOLOMON_WARNNING("Update using bin file(%s)", fw_name);
		solomon_firmware_pre_boot_up_check_bin(ftdev, fw->data,
				fw->size);
		SOLOMON_WARNNING("size : %lu Pre update end", fw->size);
		release_firmware(fw);
	}
	solomon_power_control(ftdev, POWER_RESET);
	/* solomon_reset(); */

	return 0;
}
#if (defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE) && defined(SUPPORT_BOOTUP_FW_UPGRADE_BINFILE))
static void solomon_repeat_fw_update_controller(const struct firmware *fw,
		void *context)
{
	struct solomon_device *dev = context;
	int err = 0;

	SOLOMON_WARNNING("Pre update start [repeat] >>>>> ");
	SOLOMON_TIME("S");

	if (dev != NULL && misc_dev != NULL) {
		dev->work_procedure = TS_IN_UPGRADE;

		if (!fw) {
#if defined(SUPPORT_BOOTUP_FW_UPGRADE_HEADER)			
			SOLOMON_TIME("H");
			SOLOMON_WARNNING("Going to update using Header file. [repeat]");
			err = solomon_firmware_pre_boot_up_check_head(dev);
#endif
		} else {
			SOLOMON_TIME("B");
			SOLOMON_WARNNING("Update using bin file [repeat]");
			err = solomon_firmware_pre_boot_up_check_bin(dev,
					fw->data, fw->size);
			SOLOMON_WARNNING("size : %lu Pre update end [repeat]", fw->size);
			release_firmware(fw);
		}
		if (err > 0) {
			SOLOMON_WARNNING("F/W updated. initialize sequence [repeat]");
		}

		if (ds_clear_int(misc_dev->client) < 0)
			return;
		if (sint_unstall(misc_dev->client) < 0)
			return;

		int_pin_check(misc_dev, 200);
#ifdef SUPPORT_ESD_CHECKSUM
		misc_dev->state_flag = 1;
#endif

		dev->work_procedure = TS_NO_WORK;

		ts_enable_irq();
	} else {
		SOLOMON_WARNNING("solomon device is null [repeat]");
	}
	SOLOMON_WARNNING("Update routine closed!! [repeat]");
	SOLOMON_TIME("E");
}
#endif
#if (defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE) || defined(SUPPORT_BOOTUP_FW_UPGRADE_BINFILE))
static void solomon_fw_update_controller(const struct firmware *fw,
		void *context)
{
	struct solomon_device *dev = context;
	int err = 0;
#if (defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE) && defined(SUPPORT_BOOTUP_FW_UPGRADE_BINFILE))
	const char *fw_name = FW_FULL_PATH;
#endif	

	SOLOMON_WARNNING("Pre update start >>>>> ");
	SOLOMON_TIME("S");

	if (dev != NULL && misc_dev != NULL) {
		dev->work_procedure = TS_IN_UPGRADE;
		ts_disable_irq();

		if (!fw) {
#if defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE)
			found_force_bin_file = 0;
#endif
#if (defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE) && defined(SUPPORT_BOOTUP_FW_UPGRADE_BINFILE))
			err = request_firmware_nowait(THIS_MODULE, true, fw_name,
				&dev->client->dev, GFP_KERNEL, dev,
				solomon_repeat_fw_update_controller);
			if (err)
			{
				SOLOMON_WARNNING("failed to schedule another firmware update\n");
				return;
			}
			return;
#elif defined(SUPPORT_BOOTUP_FW_UPGRADE_HEADER)
			SOLOMON_TIME("H");
			SOLOMON_WARNNING("Going to update using Header file.");
			err = solomon_firmware_pre_boot_up_check_head(dev);
#endif
		} else {
#if defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE)
			found_force_bin_file = 1;
#endif
			SOLOMON_TIME("B");
			SOLOMON_WARNNING("Update using bin file");
			err = solomon_firmware_pre_boot_up_check_bin(dev,
					fw->data, fw->size);
			SOLOMON_WARNNING("size : %lu Pre update end", fw->size);
			release_firmware(fw);
		}
		if (err > 0) {
			SOLOMON_WARNNING("F/W updated. initialize sequence");
		}

		if (ds_clear_int(misc_dev->client) < 0)
			return;
		if (sint_unstall(misc_dev->client) < 0)
			return;

		int_pin_check(misc_dev, 200);	
#ifdef SUPPORT_ESD_CHECKSUM
		misc_dev->state_flag = 1;
#endif

		dev->work_procedure = TS_NO_WORK;

		ts_enable_irq();
	} else {
		SOLOMON_WARNNING("solomon device is null");
	}
	SOLOMON_WARNNING("Update routine closed!!");
	SOLOMON_TIME("E");
}
#endif

static int solomon_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct solomon_device *ftdev = NULL;
	struct solomon_data *ftdata = NULL;
	struct solomon_config *ftconfig = NULL;
	struct workqueue_struct *wq = NULL;
	int err = 0;
#ifdef SUPPORT_KEY_BUTTON
	int i = 0;
#endif /* SUPPORT_KEY_BUTTON */

	SOLOMON_DEBUG("#### TOUCH DRIVER PROBE 1 ####");
	SOLOMON_TIME("s");
	if (client->addr != 0x53) {
        printk("[TPD]Change i2c addr 0x%02x to %x", client->addr, 0x53);
        client->addr = 0x53;
        printk("[TPD]i2c addr=0x%x\n", client->addr);
    	}
	ftdev = (struct solomon_device *)kzalloc(sizeof(struct solomon_device), GFP_KERNEL);

	if (!ftdev) {
		SOLOMON_WARNNING("Create solomon device failed");
		err = -ENOMEM;
		goto create_solomon_failed;
	}
	ftdev->client = client;	//Torr@20180117

	ftdata = (struct solomon_data *)kzalloc(sizeof(struct solomon_data), GFP_KERNEL);

	if (!ftdata) {
		SOLOMON_WARNNING("Create solomon data failed");
		err = -ENOMEM;
		goto create_data_failed;
	}
	/* initial rawdata queue */
	ftdata->queue_front = 0;
	ftdata->queue_rear = 0;
#ifdef SUPPORT_KEY_BUTTON
	ftdata->keydata = 0x00;
#endif	/* SUPPORT_KEY_BUTTON */

	ftconfig = (struct solomon_config *)kzalloc(sizeof(struct solomon_config), GFP_KERNEL);

	if (!ftconfig) {
		SOLOMON_WARNNING("Create solomon config failed");
		err = -ENOMEM;
		goto create_config_failed;
	}

	wq = create_singlethread_workqueue("solomon_touch");

	if (!wq) {
		SOLOMON_WARNNING("Create workqueue failed");
		goto create_workqueue_failed;
	}

	ftdev->touch_mode = TOUCH_POINT_MODE;
	misc_dev = ftdev;

	ftdev->workqueue = wq;
	ftdev->client = client;
	ftdev->ftconfig = ftconfig;
	ftdev->ftdata = ftdata;

	/* for I2C function */
	ftdev->i2c_func.ts_read_data = ts_read_data;
	ftdev->i2c_func.ts_read_data_ex = ts_read_data_ex;
	ftdev->i2c_func.ts_write_data = ts_write_data;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ftdev->es.level = 50;
	ftdev->es.resume = solomon_early_resume;
	ftdev->es.suspend = solomon_early_suspend;
	register_early_suspend(&ftdev->es);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

	i2c_client = client;
	err = regulator_enable(tpd->reg);
	if (err != 0)
		TPD_DMESG("Failed to enable reg-vgp6: %d\n", err);

	INIT_WORK(&ftdev->work, solomon_work);
	solomon_hw_init();
	ftdev->reset_pin = tpd_rst_gpio_number;
	ftdev->irq = touch_irq;
	ftdev->int_pin = tpd_int_gpio_number;

//#ifdef CONFIG_MTK_I2C_EXTENSION
#if 0 //I2C_DMA_SUPPORT
	gpDMABuf_va = (uint8_t *)dma_zalloc_coherent(&client->dev,
			DMA_MAX_TRANSACTION_LENGTH, &gpDMABuf_pa, GFP_KERNEL);
        
	if(!gpDMABuf_va){
        	dev_err(&client->dev,"Allocate DMA I2C Buffer failed, exit\n");
		return -ENOMEM;
    }

	wrDMABuf_va = (uint8_t *)dma_zalloc_coherent(&client->dev,
			DMA_MAX_TRANSACTION_LENGTH, &wrDMABuf_pa, GFP_KERNEL);
        
	if(!wrDMABuf_va){
        	dev_err(&client->dev,"Allocate DMA I2C Buffer failed, exit\n");
		return -ENOMEM;
    }
#endif
//#endif
	
	SOLOMON_DEBUG("#### TOUCH DRIVER PROBE 2 ####");
	init_ds_flag = 1;
	init_tmc_flag = 1;
	tpd_load_status = 1;
	if (solomon_boot_sequence(ftdev) < 0)
		goto init_config_failed;

	/* esd timer */
#if ESD_TIMER_ENABLE
	esd_checktime_init(misc_dev);
	INIT_WORK(&ftdev->tmr_work, touch_esd_tmr_work);
	ftdev->tmr_workqueue = create_singlethread_workqueue("tmr_workqueue");

	if (!ftdev->tmr_workqueue) {
		SOLOMON_DEBUG("unabled to create touch tmr work queue");
		goto init_config_failed;
	}
#endif	/* ESD_TIMER_ENABLE */

	SOLOMON_DEBUG("#### TOUCH DRIVER PROBE 3 ####");
	sema_init(&ftdev->work_procedure_lock, 1);

	device_enable_async_suspend(&client->dev);
	GTP_GPIO_AS_INT(GTP_INT_PORT);
	err = request_threaded_irq(ftdev->irq, NULL,
			solomon_interrupt,
			/*IRQF_DISABLED | */IRQF_ONESHOT | IRQF_TRIGGER_FALLING,
			SOLOMON_NAME, ftdev);
	if (err) {
		SOLOMON_WARNNING("Irq request failed");
		goto irq_request_failed;
	}

	sema_init(&ftdev->raw_data_lock, 1);
	err = misc_register(&touch_misc_dev);
	if (err)
	{
		SOLOMON_WARNNING("Fail to register touch misc device.");
		goto misc_register_failed;
	}

#ifdef SUPPORT_KEY_BUTTON
	for (i = 0; i < LPM_GESTURE_KEY_CNT; i++)
		input_set_capability(tpd->dev, EV_KEY, lpm_gesture_keys[i]);
#endif	/* SUPPORT_KEY_BUTTON */
	err= solomon_initialize_input_device(ftdev);	
	if(err){
		printk("solomon input device initialize failed!");
	}

#ifdef SUPPORT_LPM
	enable_irq_wake(misc_dev->client->irq);
	device_init_wakeup(&misc_dev->client->dev, true);
	solomon_gesture_init(misc_dev);
#ifdef SUPPORT_GESTURE_DEMO
	/* input_set_capability(input, EV_KEY, 30 ); */
	for (i = 1; i < 0x100; i++)
		input_set_capability(tpd->dev, EV_KEY, i);

#endif	/* SUPPORT_GESTURE_DEMO */
#endif	/* SUPPORT_LPM*/

	if (sysfs_create_group(&client->dev.kobj, &ssl_attr_group)) {
		dev_err(&client->dev, "failed to create sysfs group\n");
		err = -EAGAIN;
		goto sysfs_create_failed;
	}

	touchscreen_class = class_create(THIS_MODULE, "touchscreen");
	if (IS_ERR_OR_NULL(touchscreen_class)) {
		pr_err("%s: create class error!\n", __func__);
		err = -EAGAIN;
		goto class_create_failed;
	}

	err = class_create_file(touchscreen_class, &class_attr_ts_info);
	if (err < 0) {
		pr_err("%s class_create_file failed!\n", __func__);
		class_destroy(touchscreen_class);
		goto class_create_file_failed;
	}

	SOLOMON_TIME("e");
	SOLOMON_DEBUG("#### TOUCH DRIVER PROBE ok ####");
	return 0;

class_create_file_failed:
class_create_failed:
sysfs_create_failed:		
	misc_deregister(&touch_misc_dev);
misc_register_failed:
	free_irq(ftdev->irq, ftdev);
#ifdef CONFIG_MTK_I2C_EXTENSION
#if 0 //I2C_DMA_SUPPORT
	if (gpDMABuf_va)
		dma_free_coherent(NULL, DMA_MAX_TRANSACTION_LENGTH, gpDMABuf_va, gpDMABuf_pa);
	if (wrDMABuf_va)
		dma_free_coherent(NULL, DMA_MAX_TRANSACTION_LENGTH, wrDMABuf_va, wrDMABuf_pa);
#endif
#endif
irq_request_failed:
#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr != 0) {
		flush_work(&ftdev->tmr_work);
		esd_timer_stop(ftdev);
		destroy_workqueue(ftdev->tmr_workqueue);
	}
#endif
init_config_failed:
	flush_workqueue(ftdev->workqueue);
	destroy_workqueue(ftdev->workqueue);
	solomon_hw_deint();
create_workqueue_failed:
	kfree(ftconfig);
create_config_failed:
	kfree(ftdata);
create_data_failed:
	kfree(ftdev);
create_solomon_failed:
	misc_dev = NULL;
	return err;
}

static int solomon_remove(struct i2c_client *client)
{
#if ESD_TIMER_ENABLE
	int val;
#endif	/* ESD_TIMER_ENABLE */
	struct solomon_device *ftdev = i2c_get_clientdata(client);

	if (ftdev) {
		down(&ftdev->work_procedure_lock);
		ftdev->work_procedure = TS_REMOVE_WORK;

#if ESD_TIMER_ENABLE
		if (ftdev->use_esd_tmr != 0) {
			flush_work(&ftdev->tmr_work);
			val = 0;
			ts_write_data(client,
					SOLOMON_ESD_INT_INTERVAL, (u8 *)&val, 2);
			esd_timer_stop(ftdev);
			SOLOMON_DEBUG("esd timer stop");
			destroy_workqueue(ftdev->tmr_workqueue);
		}
#endif	/* ESD_TIMER_ENABLE */
		solomon_hw_deint();
		solomon_free_header(ftdev);
		free_irq(ftdev->irq, ftdev);
		misc_deregister(&touch_misc_dev);
		destroy_workqueue(ftdev->workqueue);
#ifdef CONFIG_HAS_EARLYSUSPEND
		unregister_early_suspend(&ftdev->es);
#endif	/* CONFIG_HAS_EARLYSUSPEND */
		kfree(ftdev->ftconfig);
		kfree(ftdev->ftdata);
		up(&ftdev->work_procedure_lock);
		kfree(ftdev);
	}

	i2c_set_clientdata(client, NULL);
	return 0;
}

#ifdef CONFIG_PM
static void solomon_suspend(struct device *dev)
{
	struct solomon_device *ftdev = NULL;

	ftdev = misc_dev;
	if(ftdev == NULL)
	{
		SOLOMON_WARNNING("ftdev is NULL, exit\n");
		return;
	}

#ifndef CONFIG_HAS_EARLYSUSPEND
#ifndef SUPPORT_LPM
	ts_disable_irq();
#endif	/* SUPPORT_LPM */
	flush_workqueue(ftdev->workqueue);
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		flush_work(&ftdev->tmr_work);
		esd_timer_stop(ftdev);
		SOLOMON_WARNNING("esd timer stop");
	}
#endif	/* ESD_TIMER_ENABLE */

	solomon_report_release(ftdev);
	solomon_power_control(ftdev, POWER_OFF);

	solomon_set_sleepin(ftdev);

#ifdef SUPPORT_LPM
	m_power_status = LPM_SUSPEND;
#endif	/* SUPPORT_LPM */

	return;
}

static void solomon_resume(struct device *dev)
{
	struct solomon_device *ftdev = NULL;
#ifdef SUPPORT_LPM
	int err = 0;
#endif	/* SUPPORT_LPM */

	if (dev != NULL)
		ftdev = dev_get_drvdata(dev);
	else
		ftdev = misc_dev;

	SOLOMON_WARNNING(">>>> %s solomon resume start!!!", __func__);

	solomon_set_sleepout(ftdev);
#ifdef SUPPORT_LPM
	if (m_power_status == LPM_SUSPEND_DELAY) {
		err = lpm_end_clear(ftdev);

		if (err < 0) {
			err = -EAGAIN;
			SOLOMON_WARNNING("fail to write reset command");
		}
	}
	m_power_status = LPM_RESUME;
#endif	/* SUPPORT_LPM */

#ifndef CONFIG_HAS_EARLYSUSPEND
#ifndef SUPPORT_LPM
	ts_enable_irq();
#endif	/* SUPPORT_LPM */
#endif	/* CONFIG_HAS_EARLYSUSPEND */

#if ESD_TIMER_ENABLE
	if (ftdev->use_esd_tmr) {
		esd_checktime_init(ftdev);
		esd_timer_start(SOLOMON_CHECK_ESD_TIMER, ftdev);
		SOLOMON_WARNNING("esd timer start");
	}
#endif	/* ESD_TIMER_ENABLE */

	SOLOMON_DEBUG("\n>>>> %s solomon resume end!!!", __func__);
	return;
}

int solomon_pre_on(void)
{
	SOLOMON_WARNNING("m_power_status=0x%04x!!", m_power_status);
#ifdef SUPPORT_LPM
	if (m_power_status == LPM_SUSPEND) {
		if (lpm_end2(misc_dev) >= 0)
			m_power_status = LPM_SUSPEND_DELAY;
	}
#endif	/* SUPPORT_LPM */
	return 0;
}
#else
int solomon_suspend_ex(void)
{
	return 0;
}
int solomon_resume_ex(void)
{
	return 0;
}
#endif	/* CONFIG_PM */

static const struct i2c_device_id solomon_id[] = {{TPD_DEVICE, 0}, {} };
static struct of_device_id ssd20xx_match_table[] = {
	{ .compatible = "mediatek,solomon_touch"},
	{ },
};

MODULE_DEVICE_TABLE(of, ssd20xx_match_table);

static struct i2c_driver solomon_driver = {
	.driver = {
		.of_match_table = of_match_ptr(ssd20xx_match_table),
		.owner = THIS_MODULE,
		.name = SOLOMON_NAME,
	},

	.probe = solomon_probe,
	.remove = solomon_remove,
	.id_table = solomon_id,
	.detect = tpd_i2c_detect,	
};

static int tpd_local_init(void)
{
	int retval = 0;

	TPD_DMESG("SSL SSD20xx I2C Touchscreen Driver...\n");
	tpd->reg = regulator_get(tpd->tpd_dev, "vio18");
	retval = regulator_set_voltage(tpd->reg, 1800000, 1800000);
	if (retval != 0) {
		TPD_DMESG("Failed to set reg-vgp6 voltage: %d\n", retval);
		return -1;
	}

	if (i2c_add_driver(&solomon_driver) != 0) {
		TPD_DMESG("unable to add i2c driver.\n");
		return -1;
	}
     /* tpd_load_status = 1; */
	if (tpd_dts_data.use_tpd_button) {
		tpd_button_setting(tpd_dts_data.tpd_key_num, tpd_dts_data.tpd_key_local,
		tpd_dts_data.tpd_key_dim_local);
	}
	TPD_DMESG("end %s, %d\n", __func__, __LINE__);
	tpd_type_cap = 1;

	return 0;
}

static struct tpd_driver_t tpd_device_driver = {
	.tpd_device_name = "SSD20xx",
	.tpd_local_init = tpd_local_init,
	.suspend = solomon_suspend,
	.resume = solomon_resume,
};

static int touch_solomon_init(void)
{
	TPD_DMESG("MediaTek SSD20xx touch panel driver init\n");
	tpd_get_dts_info();
	if (tpd_driver_add(&tpd_device_driver) < 0)
		TPD_DMESG("add SSD20xx driver failed\n");

	return 0;
}

static void __exit touch_solomon_exit(void)
{
	TPD_DMESG("MediaTek SSD20xx touch panel driver exit\n");
	tpd_driver_remove(&tpd_device_driver);
}

module_init(touch_solomon_init);
module_exit(touch_solomon_exit);

MODULE_AUTHOR("Solomon Systech (ShenZhen) Limited");
MODULE_DESCRIPTION("ssd20xx Touchscreen Driver "DRIVER_VRESION);
MODULE_LICENSE("GPL V2");
