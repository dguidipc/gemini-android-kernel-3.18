/*
 * Copyright 2017 Solomon Systech Ltd. All rights reserved.
 *
 * SSL SSD20xx Touch device driver
 *
 * Date: 2017.04.19
 */
#ifndef __SSD20XX_H
#define __SSD20XX_H

#include <linux/semaphore.h>

#define DRIVER_VRESION "1.10"
#define GTP_GPIO_AS_INT(pin) tpd_gpio_as_int(pin)
#define GTP_GPIO_OUTPUT(pin, level) tpd_gpio_output(pin, level)

//#define GPIO_CTP_EINT_PIN       (GPIO65|0x80000000)
//#define GPIO_CTP_RST_PIN        (GPIO66|0x80000000)

//#define P_GPIO_CTP_EINT_PIN       1
//#define P_GPIO_CTP_RST_PIN        10

extern struct tpd_device *tpd;

//--MTK I2C DMA info.---
//#define I2C_DMA_SUPPORT 1
#define DMA_MAX_TRANSACTION_LENGTH        255   // for DMA mode
#define DMA_MAX_I2C_TRANSFER_SIZE        (DMA_MAX_TRANSACTION_LENGTH - 1)

//#define SUPPORT_MT_PROTOCOL_B   //Del by ZGY

//#define SUPPORT_MTK_ESD_RECOVERY

//#define SUPPORT_TOUCH_RESET_PIN_CTL

#define SUPPORT_BOOTUP_FW_UPGRADE_HEADER
#define SUPPORT_BOOTUP_FW_UPGRADE_BINFILE
#define SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE

#define SUPPORT_ES2			/* for SSD2098 ES2 */
#define SUPPORT_PROTOCOL_6BYTES		/* for 6bytes point protocol */

/* for support gesture coordinate (2015/12/28) */
/* #define SUPPORT_GESTURE_COORDINATE */

/* for support TMC I2C set (2015/12/29) */
#define SUPPORT_TMC_I2C_LENGTH

/* #define TEST_FOR_LGE_LPWG */
/* #define FOR_TEST */			/* for test */
#define SUPPORT_LPM			/* for support of LPM */

/* For ESD checksum. only Status & Length and point info */
#define SUPPORT_ESD_CHECKSUM
#define SUPPORT_AUX			/* for support of AUX bit of SnL */
#define SUPPORT_KEY_BUTTON		/* for key H/W button */

//#define SUPPORT_GESTURE_DEMO	/* for gesture demo. relative MainTAG */

#define TMC_UNSTALL_DELAY	5
#define TMC_READ_DELAY		200		/* I2C packet delay */

#define TOUCH_POINT_MODE		0
#define RAW_DATA_MODE			1

#define AGING_RAW_DATA_MODE		31	/* for rawdata of aging */
#define AGING_BASELINED_DATA_MODE	32	/* for baselined of aging */
#define AGING_BASELINE_DATA_MODE	33	/* for baseline of aging */
#define AGING_MODIFY_DATA_MODE		34	/* for modify of aging */

/* for MP Test */
#define MPTEST_START		0x0001
#define MPTEST_STOP		0x0000
#define MPTEST_READY_STOP	0x8000
#define MPTEST_READY_START	0x4000
#define MPTEST_LPM_NC_RANGE	0x0006	/* Normal CELL */
#define MPTEST_LPM_NC_JITTER	0x0007	/* Normal CELL */
#define MPTEST_LPM_LC_RANGE	0x0008	/* Large CELL */
#define MPTEST_LPM_LC_JITTER	0x0009	/* Large CELL */

#define MAX_RAWDATA_BUFFER_SIZE	512
#define MAX_DEVICE_KEYMAP_SIZE	16

#define HIDDEN_VERSION_FACTORY		0x7F0000F7	/* for force update */

#define FW_FULL_PATH				("str_fw.img")
#define FW_BOOTUP_FORCE_FULL_PATH				("str_force_fw.img")
#define FW_FORCE_FULL_PATH			("/sdcard/str_force.img")

#define SOLOMON_NAME				("ssd20xx")	/* I2C device name */
#define SOLOMON_I2C_ADDR			(0x53)	/* I2C slave address */
#define SOLOMON_X_MAX				(1080)	/* resolution X */
#define SOLOMON_Y_MAX				(2160)	/* resolution Y */
#define SOLOMON_MAX_X_NODE			(18)			/* number of NODE X */
#define SOLOMON_MAX_Y_NODE			(32)			/* number of NODE Y */
#define SOLOMON_MAX_PRESSURE	    255
#define SOLOMON_MAX_MAJOR	        30

/* The maximum supported number of fingers. */
#define SOLOMON_MAX_POINT			(10)

#define SOLOMON_MAX_NODE			(SOLOMON_MAX_X_NODE * SOLOMON_MAX_Y_NODE)

#define SOLOMON_MAX_RAWDATA_QUEUE	11

//#define SOLOMON_RESET_PIN			(EXYNOS5410_GPF1(1))	/* reset */
//#define SOLOMON_INT_PIN				(EXYNOS5410_GPX0(2))	/* touch interrupt gpio */
//#define SOLOMON_IRQ					(IRQ_EINT(2)) /* touch irq */

#define DELAY_FOR_SIGNAL_DELAY		30	/* us */

#define DELAY_FOR_TRANSCATION		TMC_READ_DELAY
#define DELAY_FOR_POST_TRANSCATION	TMC_READ_DELAY

#define RETRY_TIME_INT_PIN			200
/* Gesture size
 * size = Gesture(2bytes)+Length(2bytes)+points(40bytes)
 * points = (pos_x(2bytes) + pos_y(2bytes))*Max_coord(10)
 */
#define LENGTH_GESTURE_DATA		(2+2+2*2*10)

#define PASS_DATA_ID_BYTE	0	/* array index */
#define PASS_DATA_DEVICEID_BYTE	5	/* array index */
#define PASS_DATA_DEVICEID_MASK	0x0F
#define PASS_DATA_FORCE_DOWN_MASK	0x80	/* force down */
#define PASS_DATA_FORCE_UP_MASK		0x40	/* force up */

#define PASS_DATA_ID_AXIS_FIRST	0x00
#define PASS_DATA_ID_AXIS_LAST	0x0E
#define PASS_DATA_ID_KEY	0xF5
#define PASS_DATA_ID_GESTURE	0xF6
#define PASS_DATA_ID_AUX	0xF8	/* aux */

#define I2C_SUCCESS				0
#define I2C_FAIL				1

#define POWER_ON				0
#define POWER_OFF				1
#define POWER_ON_SEQUENCE		2
#define POWER_RESET             3

//###############################################################################
// KEY_GESTURE 15 
//###############################################################################
#define  KEY_GESTURE_UP								KEY_UP
#define  KEY_GESTURE_DOWN							KEY_DOWN
#define  KEY_GESTURE_LEFT							KEY_LEFT 
#define  KEY_GESTURE_RIGHT							KEY_RIGHT
#define  KEY_GESTURE_DOUBLECLICK					KEY_D
#define  KEY_GESTURE_O								KEY_O
#define  KEY_GESTURE_W								KEY_W
#define  KEY_GESTURE_M								KEY_M
#define  KEY_GESTURE_E								KEY_E
#define  KEY_GESTURE_S								KEY_S
#define  KEY_GESTURE_Z								KEY_Z
#define  KEY_GESTURE_C								KEY_C
#define  KEY_GESTURE_U								KEY_U
//#define  KEY_GESTURE_U_DOWN							KEY_A
//#define  KEY_GESTURE_U_RIGHT						KEY_V


#define GESTURE_UP									0x54
#define GESTURE_DOWN								0x42
#define GESTURE_LEFT								0x52
#define GESTURE_RIGHT								0x4c

#define GESTURE_DOUBLECLICK							0x400
#define GESTURE_O									0x6F
#define GESTURE_W									0x77
#define GESTURE_M									0x6D
#define GESTURE_E									0x65
#define GESTURE_S									0x73
#define GESTURE_Z									0x7A
#define GESTURE_C									0x63
#define GESTURE_U									0x76
#define GESTURE_U_DOWN								0x5E
#define GESTURE_U_RIGHT								0x3E

/* for DS16 */
#define DS_EFLASH_WRITE				0x0004
#define DS_EFLASH_READ				0x0005
#define DS_COMMAND_01				0xF002
#define DS_COMMAND_02				0xE008
#define DS_COMMAND_031				0xA001
#define DS_COMMAND_03				0xA002
#define DS_CUP_CONTROL				0x0002
#define DS_CLEAR_INT				0x0001
#define DS_ERASE_MACRO				0x000B

#define DS_EFLASH_READ_01			0x0009
#define DS_EFLASH_WRITE_01			0x0008
#define DS_EFLASH_DOWNLOAD_ADDR1	0x7E00
#define DS_EFLASH_DOWNLOAD_ADDR2	0x0000

#define DS_COMMAND_READ_VERSION	    0x000F

#define DS_WRITE_PTR				0x9004
#define DS_READ_PTR					0x9000

/*-----------------------------------------------------
 *	Debug msg
 *-----------------------------------------------------
 */
#define solomon_debug		0
#define solomon_warnning	1
#define solomon_timecheck	0	/* only use check the boot time */

#if solomon_debug
#define SOLOMON_DEBUG(fmt, args...)	\
	pr_info("[SOLOMON-INFO : %-18s] "fmt"\n",	\
			__func__, ##args)
#else
#define SOLOMON_DEBUG(fmt, args...) \
	do {} while (0)
#endif

#if  solomon_warnning
#define SOLOMON_WARNNING(fmt, args...) \
	pr_info("[SOLOMON-WARN : %-18s] "fmt"\n",	\
			__func__, ##args)
#else
#define SOLOMON_WARNNING(fmt, args...) \
	do {} while (0)
#endif

#if  solomon_timecheck
#define SOLOMON_TIME(fmt, args...) \
	dev_info(""fmt"\n",	\
##args)
#else
#define SOLOMON_TIME(fmt, args...) \
	do {} while (0)
#endif
/*-----------------------------------------------------
 *	ESD TIMER
 *-----------------------------------------------------
 */
/* Support ESD Timer when value is 1. */
#define ESD_TIMER_ENABLE	 0	//1

#define	SOLOMON_ESD_INTERVAL		1
#define SOLOMON_SCAN_RATE_HZ		60
#define SOLOMON_CHECK_ESD_TIMER		3


/*-----------------------------------------------------
 *	CMD & Reg Addr
 *-----------------------------------------------------
 */
#define SOLOMON_POWER_MODE			0x002F

#define SOLOMON_SWRESET_CMD			0x0044

#define SOLOMON_INT_CLEAR_CMD		0x0043
#define SOLOMON_SW_CALIBRATION		0x0040
#define SOLOMON_TEST_MODE			0x0045
#define SOLOMON_HW_CALIBRATION		0x0046

#define SOLOMON_MP_TEST				0x004A

#define SOLOMON_TOUCH_MODE			0x0050

#define SOLOMON_INT_FLAG			0x0051
#define SOLOMON_DATA_VERSION		0x0052
#define SOLOMON_TOTAL_NODE			0x0053
#define SOLOMON_TOTAL_Y_NODE		0x0054
#define SOLOMON_TOTAL_X_NODE		0x0055
#define SOLOMON_X_RESOLUTION		0x0056
#define SOLOMON_Y_RESOLUTION		0x0057
#define SOLOMON_ORIENTATION			0x0058
#define SOLOMON_USING_POINT_NUM		0x0059
#define SOLOMON_SENSITIVITY			0x005c
#define SOLOMON_ESD_INT_INTERVAL	0x006a

#ifdef SUPPORT_TMC_I2C_LENGTH
#define SOLOMON_TMC_I2C_LENGTH		0x005A
#endif
#define SOLOMON_HW_CAL_INFO		(SOLOMON_TOUCH_MODE+0x12B)
#define SOLOMON_ESD_TIME		0x016E

#define SOLOMON_STATUS_LENGTH		0x0AF0
#define SOLOMON_POINT_DATA			0x0AF1
#define SOLOMON_RAW_DATA			0x0AF2
#define SOLOMON_DEBUG_MODE			0x0AF3
#define SOLOMON_GRAPH_MODE			0x0AF4
#define SOLOMON_GET_KEYDATA			0x0AF5
#define SOLOMON_GET_GESTURE			0x0AF6
#define SOLOMON_GET_GESTURE_COORDINATE	0x0AF7
#define SOLOMON_AUX					0x0AF8
#define SOLOMON_DOWNLOAD_MODE		0x0AF9
#define SOLOMON_FW_VER_WRITE		0x0AFA
#define SOLOMON_DATA_VER_WRITE		0x0AFB
#define SOLOMON_SAVE_REG			0x0AFD
#define SOLOMON_TCI1_FAIL_REASON	0x0B03
#define SOLOMON_TCI2_FAIL_REASON	0x0B04

#define SOLOMON_SLEEP_IN			0x0000
#define SOLOMON_SLEEP_OUT			0x0000

/*-----------------------------------------------------
 *	AUX mode
 *-----------------------------------------------------
 */
#define AUX_ESD_DETECT			0x00AA
#define AUX_CS_ERROR			0x00F0
#ifdef SUPPORT_ES2
#define AUX_BOOTUP_RESET		0x0001
#define AUX_WDTDS_INT			0x0002
#endif
#define AUX_NEED_C1_TMC_RESET		0x00C1
#define AUX_NEED_C2_DIC_RESET		0x00C2
#define AUX_NEED_C3_DIC_POWER		0x00C3
#define AUX_NEED_C4_DSV_POWER_RESET	0x00C4

/*-----------------------------------------------------
 *	POWER mode
 *-----------------------------------------------------
 */
#define POWER_MODE_NM				0x0001
#define POWER_MODE_LPM				0x0002

#define CHECK_GESTURE_KNOCK_ON		0x0400

#define STATUS_CHECK_PALM_GESTURE	0x10
#define STATUS_CHECK_KEY			0x20
#define STATUS_CHECK_AUX			0x40
#define STATUS_CHECK_BOOT_ST		0x80

#define MAX_GESTURE_COORDINATE_SIZE		(64*4)
/*	gesture status(1word) + legnth(1word); */
#define GESTURE_READ_SIZE				2

#define GESTURE_STATUS_KNOCK_CODE		0x0800
#define GESTURE_STATUS_KNOCK_ON			0x0400
#define GESTURE_STATUS_PALM_REJECT		0x0200
#define GESTURE_STATUS_LARGE_PALM		0x0100
#define GESTURE_STATUS_GESTURES			0x00FF

#define GESTURE_STATUS_KNOCK_ALL	(GESTURE_STATUS_KNOCK_CODE |	\
		GESTURE_STATUS_KNOCK_ON |	\
		GESTURE_STATUS_GESTURES)
/*-----------------------------------------------------
 *	boot status failure bit
 *-----------------------------------------------------
 */
#define BOOT_STATUS_ERR_CPUCFG_CPUINST_CHECKSUM_FAIL	0x0100
#define BOOT_STATUS_ERR_CPUCFG_CPUDM_CHECKSUM_FAIL		0x0200
#define BOOT_STATUS_ERR_CPUCFG_TABLE_CHECK_FAIL			0x0400
#define BOOT_STATUS_ERR_SYS_CFG_FAIL					0x0800
#define BOOT_STATUS_ERR_INF_LDORDAC_INVALID				0x1000
#define BOOT_STATUS_ERR_INF_IDCO32K_INVALID				0x2000
#define BOOT_STATUS_ERR_INF_OSC_TRIM_INVALID			0x4000

#define BOOT_STATUS_ERR_CPUCFG_ALL			\
	(BOOT_STATUS_ERR_CPUCFG_CPUINST_CHECKSUM_FAIL |	\
	 BOOT_STATUS_ERR_CPUCFG_CPUDM_CHECKSUM_FAIL	|	\
	 BOOT_STATUS_ERR_CPUCFG_TABLE_CHECK_FAIL)

#define BOOT_STATUS_ERR_INF_ALL				\
	(BOOT_STATUS_ERR_INF_LDORDAC_INVALID |		\
	 BOOT_STATUS_ERR_INF_IDCO32K_INVALID |		\
	 BOOT_STATUS_ERR_INF_OSC_TRIM_INVALID)

#define BOOT_STATUS_ERR_ALL			\
	(BOOT_STATUS_ERR_CPUCFG_ALL |		\
	 BOOT_STATUS_ERR_CPUCFG_SYS_CFG_FAIL |	\
	 BOOT_STATUS_ERR_INF_ALL)

/*-----------------------------------------------------
 *	DOWNLOAD
 *-----------------------------------------------------
 */
#define LDM_RUN				0x00
#define APM_RUN				0x02
#define	FW_LOAD_FINISH		0x04
#define	FW_LOAD_START		0x01
#define	FW_READ_START		0x08
#define DATA_LOAD_FINISH	0x07
#define DATA_LOAD_START		0x05
#define	DATA_READ_START		0x09

/*-----------------------------------------------------
 *	IOCTL
 *-----------------------------------------------------
 */
#define TOUCH_IOCTL_BASE				0xbc
#define TOUCH_IOCTL_GET_FW_VERSION		_IO(TOUCH_IOCTL_BASE, 0)
#define TOUCH_IOCTL_GET_DATA_VERSION	_IO(TOUCH_IOCTL_BASE, 1)
#define TOUCH_IOCTL_GET_X_NODE_NUM		_IO(TOUCH_IOCTL_BASE, 2)
#define TOUCH_IOCTL_GET_Y_NODE_NUM		_IO(TOUCH_IOCTL_BASE, 3)
#define TOUCH_IOCTL_GET_TOTAL_NODE_NUM	_IO(TOUCH_IOCTL_BASE, 4)
#define TOUCH_IOCTL_SET_TOUCH_MODE		_IO(TOUCH_IOCTL_BASE, 5)
#define TOUCH_IOCTL_GET_RAW_DATA		_IO(TOUCH_IOCTL_BASE, 6)
#define TOUCH_IOCTL_GET_X_RESOLUTION	_IO(TOUCH_IOCTL_BASE, 7)
#define TOUCH_IOCTL_GET_Y_RESOLUTION	_IO(TOUCH_IOCTL_BASE, 8)
#define TOUCH_IOCTL_GET_REG				_IO(TOUCH_IOCTL_BASE, 9)
#define TOUCH_IOCTL_SET_REG				_IO(TOUCH_IOCTL_BASE, 10)
#define TOUCH_IOCTL_SET_DOWNLOAD		_IO(TOUCH_IOCTL_BASE, 11)
#define TOUCH_IOCTL_GET_GRAPH_DATA		_IO(TOUCH_IOCTL_BASE, 12)
#define TOUCH_IOCTL_QUEUE_CLEAR			_IO(TOUCH_IOCTL_BASE, 13)
#define TOUCH_IOCTL_GET_GESTURE			_IO(TOUCH_IOCTL_BASE, 14)
#define TOUCH_IOCTL_MP_TEST				_IO(TOUCH_IOCTL_BASE, 15)
#define TOUCH_IOCTL_SW_RESET			_IO(TOUCH_IOCTL_BASE, 16)
#define TOUCH_IOCTL_HW_RESET			_IO(TOUCH_IOCTL_BASE, 17)

int solomon_reset(void);
int ts_read_data(struct i2c_client *client, u16 reg, u8 *values, u16 length);
int ts_read_data_ex(struct i2c_client *client, u8 *reg, u16 regLen,
		u8 *values, u16 length);
int ts_write_data(struct i2c_client *client,	u16 reg, u8 *values,
		u16 length);
int ds_read_boot_st(struct i2c_client *client, u16 *value);
int ds_clear_int(struct i2c_client *client);
int ds_eflash_write(struct i2c_client *client, int addr, u16 data);
int ds_eflash_read(struct i2c_client *client, int addr, u8 *rd, int rLen);

int solomon_suspend_ex(void);
int solomon_resume_ex(void);

/*-----------------------------------------------------
 *	firmware update
 *-----------------------------------------------------
 */
#define SUPPORT_TEST_MODE

#define CONTENT_HEADER_SIZE		12		/* bytes */

#define FW_MAX_RETRY_COUNT		3
#define FW_MAX_I2C_DATA_COUNT	256
#define FW_MAX_DATA_INFO_SIZE	8
#define FW_ERASE_ALL_PAGENUM	128

#define BOOT_UPDATE_ALL			1
#define BOOT_UPDATE_EACH		0

#define BOOT_UPDATE_OK			1
#define BOOT_UPDATE_NONE		0

/* ERROR NUMBER */
#define ERROR_TYPE_PARSING           0x81000000
#define ERROR_TYPE_UPDATE            0x82000000
#define ERROR_TYPE_VERSION           0x84000000
#define ERROR_TYPE_VERIFY            0x88000000
#define ERROR_TYPE_EFLASH            0x90000000
#define ERROR_TYPE_SYSTEM            0xA0000000

#define ERROR_SUCCESS			0
#define ERROR_PARSING_FILENAME_IS_NULL	(ERROR_TYPE_PARSING | 0x00000001)
#define ERROR_PARSING_FILE_OPEN_FAIL	(ERROR_TYPE_PARSING | 0x00000002)
#define ERROR_PARSING_FORMAT_INVALID	(ERROR_TYPE_PARSING | 0x00000003)
#define ERROR_PARSING_CHECKSUM_FAIL		(ERROR_TYPE_PARSING | 0x00000004)
#define ERROR_PARSING_MALLOC_FAIL		(ERROR_TYPE_PARSING | 0x00000005)
#define ERROR_PARSING_CONTENT_SIZE_FAIL	(ERROR_TYPE_PARSING | 0x00000006)
#define ERROR_PARSING_DATA_CNT_FAIL		(ERROR_TYPE_PARSING | 0x00000007)
#define ERROR_PARSING_HEADER_DATA_INVALID_LENGTH	\
	(ERROR_TYPE_PARSING | 0x00000008)

#define ERROR_PARSING_INVALID_DATATYPE	(ERROR_TYPE_PARSING | 0x00000009)

#define ERROR_UPDATE_INIT_FAIL		(ERROR_TYPE_UPDATE | 0x00000001)
#define ERROR_UPDATE_ERASE_FAIL		(ERROR_TYPE_UPDATE | 0x00000002)
#define ERROR_UPDATE_WRITE_FAIL		(ERROR_TYPE_UPDATE | 0x00000003)
#define ERROR_UPDATE_READ_FAIL		(ERROR_TYPE_UPDATE | 0x00000004)
#define ERROR_UPDATE_VERIFY_FAIL	(ERROR_TYPE_UPDATE | 0x00000005)

#define ERROR_EFLAH_ERASE_FAIL		(ERROR_TYPE_EFLASH | 0x00000001)
#define ERROR_EFLAH_WRITE_FAIL		(ERROR_TYPE_EFLASH | 0x00000002)
#define ERROR_EFLAH_READ_FAIL		(ERROR_TYPE_EFLASH | 0x00000003)
#define ERROR_EFLAH_VERIFY_FAIL		(ERROR_TYPE_EFLASH | 0x00000004)

#define ERROR_SYSTEM_FAIL			(ERROR_TYPE_SYSTEM | 0x00000001)

#define ERROR_VERSION_CHECK_FAIL	(ERROR_TYPE_VERSION | 0x00000001)

#define ERROR_VERIFY_VERIFY_FAIL	(ERROR_TYPE_VERIFY | 0x00000001)

/*-----------------------------------------------------
 *	key data
 *-----------------------------------------------------
 */
#ifdef SUPPORT_KEY_BUTTON
#define SSL_KEYDATA_BACK_DOWN		0x0001
#define SSL_KEYDATA_MENU_DOWN		0x0002
#define SSL_KEYDATA_HOME_DOWN		0x0004
#define SSL_KEYDATA_HOMEPAGE_DOWN	0x0008

#define SSL_KEYDATA_BACK_UP			0x0100
#define SSL_KEYDATA_MENU_UP			0x0200
#define SSL_KEYDATA_HOME_UP			0x0400
#define SSL_KEYDATA_HOMEPAGE_UP		0x0800
#endif	/* SUPPORT_KEY_BUTTON */

#ifdef SUPPORT_ES2
#define DS_VERSION_ES1		1
#define DS_VERSION_ES2		2
#endif
/* to support I2C read/write */
struct I2C_func {
	int (*ts_read_data)(struct i2c_client *, u16, u8 *, u16);
	int (*ts_read_data_ex)(struct i2c_client *, u8 *, u16, u8 *, u16);
	int (*ts_write_data)(struct i2c_client *, u16, u8 *, u16);
};

typedef union {
	/* point data structure */
	struct {
	u8 id;
	u8 x_lsb;
	u8 y_lsb;
	u8 x_y_msb;
	u8 weight;
		u8 forceDeviceID;
	} point;
	/* gesure data structure */
	struct {
		u8 id;
		u8 gesture;
		u8 status;
		u8 coordLengthL;
		u8 coordLengthH;
		u8 forceDeviceID;
	} gesture;
	/* key data structure */
	struct {
		u8 id;
		u8 keydownL;
		u8 keydownH;
		u8 keyupL;
		u8 keyupH;
		u8 forceDeviceID;
	} key;
	/* aux data structure */
	struct {
		u8 id;
		u8 codeL;
		u8 codeH;
		u8 reserved01;
		u8 reserved02;
		u8 reserved03;
	} aux;
} ssd_data;

struct solomon_data {
	u16 point_info;
	ssd_data points[SOLOMON_MAX_POINT];
	s16 rawData[SOLOMON_MAX_RAWDATA_QUEUE]
		[SOLOMON_MAX_NODE + 4 * SOLOMON_MAX_POINT + 2];
	int queue_front;
	int queue_rear;
	int validPointCnt;
	int lastValidCnt;
#ifdef SUPPORT_KEY_BUTTON
	u32	keydata;
#endif	/* SUPPORT_KEY_BUTTON */
};

struct solomon_config {
	u8 fw_ver[24];
	u16 data_ver;
	u16 max_x;		/* x resolution */
	u16 max_y;		/* y resolution */
	u16 max_weight;
	u16 using_point;
	u16 x_node;
	u16 y_node;
	bool check;
#ifdef SUPPORT_TMC_I2C_LENGTH
	u16 i2c_length;
#endif	/* SUPPORT_TMC_I2C_LENGTH */
};

struct solomon_version {
	u32 display_version;
	u32 hidden_version;
	u32 productID01;
	u32 productID02;
	u32 ICName01;
	u32 ICName02;
};

struct solomon_device {
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct atc260x_dev *atc260x;
	struct solomon_data *ftdata;
	struct solomon_config *ftconfig;
	struct work_struct work;
	struct workqueue_struct *workqueue;
	struct mutex lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend es;
#endif	/* CONFIG_HAS_EARLYSUSPEND */
	int irq;
	int int_pin;
	int reset_pin;

	u16 touch_mode;
	u16 mptest_mode;
	u8 update;
	struct semaphore	raw_data_lock;

	u8 work_procedure;
	struct semaphore work_procedure_lock;

	/* ESD TIMER */
#if ESD_TIMER_ENABLE
	struct work_struct tmr_work;
	struct workqueue_struct *tmr_workqueue;
	u8 use_esd_tmr;
	bool in_esd_tmr;
	struct timer_list esd_tmr;
	struct timer_list *p_esd_tmr;
	unsigned long esd_check_time;
#endif
	struct I2C_func i2c_func;
	struct solomon_version fw_version;
	u16		boot_flag;
	u16		checksum_flag;
#ifdef SUPPORT_ES2
	u16		es_version;
#endif
#ifdef SUPPORT_ESD_CHECKSUM
	u16		state_flag;
#endif
	unsigned long tmc_start_time;
};


struct solomon_fw {
	int address;
	int byte_cnt;
	int erase_page_cnt;
	int version;
	unsigned int checksum;
	int reserved_01;
	int reserved_02;
	int reserved_03;
	unsigned char *content;
};

/* structure for update */
struct solomon_fw_group {
	struct solomon_fw section;
	struct solomon_fw_group *next;
};

/* structure for header */
struct solomon_fw_group_header {
	struct solomon_version fw_version;
	struct solomon_fw_group *fw_group;
};

int solomon_firmware_update_byfile(struct solomon_device *dev, char *filename);
int solomon_firmware_pre_boot_up_check_bin(struct solomon_device *dev,
		const u8 *data, size_t data_size);
int solomon_firmware_pre_boot_up_check_head(struct solomon_device *dev);
int solomon_firmware_pre_boot_up_check(struct solomon_device *dev);
int solomon_get_version_boot(struct solomon_device *dev);
u8 *solomon_get_version(struct solomon_device *dev, u8 *ver_buff);
int solomon_free_header(struct solomon_device *dev);
#endif
