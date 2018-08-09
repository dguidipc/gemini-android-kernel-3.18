/*
 * Copyright 2017 Solomon Systech Ltd. All rights reserved.
 *
 * SSL SSD20xx MX25V100 device driver
 *
 * Date: 2017.04.19
 */
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include "ssd20xx.h"
#include "ssd20xx_mx25v100.h"

#define SEEPROM_ID_MX25V1006E	0x00C22011
#define SEEPROM_ID_MX25U1001E	0x00C22531

#define VERSION_START_ADDRESS	0x01C0
#define VERSION_DISPLAY_ADDR	(VERSION_START_ADDRESS+0x44)
#define VERSION_HIDDEN_ADDR		(VERSION_START_ADDRESS+0x48)
#define PRODUCT_ID_ADDR1		(VERSION_START_ADDRESS+0x4C)
#define PRODUCT_ID_ADDR2		(VERSION_START_ADDRESS+0x50)
#define ICNAME_ADDR1			(VERSION_START_ADDRESS+0x54)
#define ICNAME_ADDR2			(VERSION_START_ADDRESS+0x58)

#define DELAY_PRE_WAIT_BUSY_CLEAR	400
#define DELAY_WAIT_BEFORE_READ		10

int m_SEEPROM_ReadMax_Length	= 1024;
int m_SEEPROM_WriteMax_Length	= 256;
int m_SEEPROM_Align_Sector	= 4096;
int m_SEEPROM_ID;

#if 0//I2C_DMA_SUPPORT
#include <linux/dma-mapping.h>
extern uint8_t *gpDMABuf_va;
extern dma_addr_t gpDMABuf_pa;
extern uint8_t *wrDMABuf_va;
extern dma_addr_t wrDMABuf_pa;
#endif

int (*ds16_seeprom_erase_sector)(struct solomon_device *, u32);
int (*ds16_seeprom_erase_chip)(struct solomon_device *);
int (*ds16_seeprom_write_nbyte)(struct solomon_device *, u32, u8 *, int);

/*  calculate checksum
 *  parameter :
 *		- int len : tmpContent's length to calculate checksum
 *		- unsigned short tmpContent : To calculate checksum
 *		- unsigned int *checksum : result of calculate checksum
 *  return : 0 is success, else fail
 */
int ds16_seeprom_fw_calc_checksum(int len, unsigned short *tmpContent,
		unsigned int *checksum)
{
	int i = 0;
	int err = ERROR_SUCCESS;
	unsigned short sum = 0x00;
	unsigned short xor = 0x00;

	if (tmpContent == NULL)
		return ERROR_PARSING_CHECKSUM_FAIL;

	SOLOMON_WARNNING("\n\n");
	SOLOMON_WARNNING("0x%04x 0x%04x", tmpContent[0], tmpContent[1]);
	SOLOMON_WARNNING("\n\n");

	for (i = 0; i < len; i++) {
		sum += tmpContent[i];
		xor ^= tmpContent[i];
	}

	*checksum = (xor<<16)|sum;

	SOLOMON_WARNNING(">>>> sum:0x%04x, xor:0x%04x, checksum:0x%08x\n",
			sum, xor, *checksum);
	return err;
}

/*	Micro Command write
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u8 *dbuf : array for write to SEEPROM
 *		- int nByte : length of dbuf
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_general_ucmd_write(struct solomon_device *dev, u8 *dbuf,
		int nByte)
{
	int err = 0;
	u8 ws_wd[1024] = {0,};
	int ws_wd_nbyte = 0;
	int seeprom_ws_wd_ndelay = 0;

	/* SEEPROM Header */
	ws_wd[ws_wd_nbyte++] = 0x0A;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = (nByte & 0x00FF) >> 0;
	ws_wd[ws_wd_nbyte++] = (nByte & 0xFF00) >> 8;
	ws_wd[ws_wd_nbyte++] = (seeprom_ws_wd_ndelay & 0x00FF) >> 0;
	ws_wd[ws_wd_nbyte++] = (seeprom_ws_wd_ndelay & 0xFF00) >> 8;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = 0x00;

	memcpy(ws_wd+ws_wd_nbyte, dbuf, nByte);

	ws_wd_nbyte += nByte;

	err = i2c_master_send(dev->client, ws_wd, ws_wd_nbyte);

	if (err < 0) {
		SOLOMON_WARNNING("I2C write fail(0x%04x)!!", err);
		return err;
	}

	return 0;
}

/*	Micro Command read
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u8 *Wbuf : array for write to DS16
 *		- int WnByte : length of Wbuf
 *		- u8 *Rbuf : array for read data from DS16
 *		- int RnByte : length of Rbuf
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_general_ucmd_read(struct solomon_device *dev, u8 *Wbuf,
		int WnByte, u8 *Rbuf, int RnByte)
{
	int err = 0;

	unsigned char ws_wd[1024];
	int seeprom_ws_wd_ndelay = 0;
	int seeprom_rs_wd_ndelay = 0;
	int seeprom_rs_rd_ndelay = 0;
	int seeprom_rs_wd_nbyte = 0;
	int ws_wd_nbyte = 0;
	int ws_wd_ndelay;

#if 0//I2C_DMA_SUPPORT
	int32_t ret;
	int32_t retries = 0;
	unsigned char *buf_va = NULL;

	struct i2c_msg msg[2] = {
		{
			.addr = (SOLOMON_I2C_ADDR & I2C_MASK_FLAG),
			.ext_flag = (dev->client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = 0,
			.buf = (uint8_t*)wrDMABuf_pa,
			.len = WnByte,
			.timing = dev->client->timing
		},
		{
			.addr = (SOLOMON_I2C_ADDR & I2C_MASK_FLAG),
			.ext_flag = (dev->client->ext_flag | I2C_ENEXT_FLAG | I2C_DMA_FLAG),
			.flags = I2C_M_RD,
			.buf = (uint8_t*)gpDMABuf_pa,
			.len = RnByte,
			.timing = dev->client->timing
		},
	};
#endif

	/* SEEPROM, HEADER */
	ws_wd_nbyte = 0;
	ws_wd[ws_wd_nbyte++] = 0x0A;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = 0x00;
	ws_wd[ws_wd_nbyte++] = (WnByte & 0x00FF) >> 0;
	ws_wd[ws_wd_nbyte++] = (WnByte & 0xFF00)>>8;
	ws_wd[ws_wd_nbyte++] = (seeprom_ws_wd_ndelay & 0x00FF) >> 0;
	ws_wd[ws_wd_nbyte++] = (seeprom_ws_wd_ndelay & 0xFF00) >> 8;
	ws_wd[ws_wd_nbyte++] = (seeprom_rs_wd_nbyte & 0x00FF) >> 0;
	ws_wd[ws_wd_nbyte++] = (seeprom_rs_wd_nbyte & 0xFF00) >> 8;
	ws_wd[ws_wd_nbyte++] = (seeprom_rs_wd_ndelay & 0x00FF) >> 0;
	ws_wd[ws_wd_nbyte++] = (seeprom_rs_wd_ndelay & 0xFF00) >> 8;
	ws_wd[ws_wd_nbyte++] = (RnByte & 0x00FF) >> 0;
	ws_wd[ws_wd_nbyte++] = (RnByte & 0xFF00) >> 8;
	ws_wd[ws_wd_nbyte++] = (seeprom_rs_rd_ndelay & 0x00FF) >> 0;
	ws_wd[ws_wd_nbyte++] = (seeprom_rs_rd_ndelay & 0xFF00) >> 8;

	memcpy(ws_wd+ws_wd_nbyte, Wbuf, WnByte);	/* SEEPROM, WS_WD */
	ws_wd_nbyte += WnByte;
	ws_wd_ndelay = (RnByte*DELAY_WAIT_BEFORE_READ) + 10;

#if 0//I2C_DMA_SUPPORT
	if (Rbuf == NULL) {
		SOLOMON_WARNNING("Rbuf is NULL!\n");
		return -ENOMEM;
	}

	buf_va = wrDMABuf_va;
	memcpy(&buf_va[0], &ws_wd[0], ws_wd_nbyte);

	msg[0].len = ws_wd_nbyte;
	
	SOLOMON_DEBUG("buf_va0 (0x%02x), buf_va1 (0x%02x), buf_va16 (0x%02x), buf_va17 (0x%02x), buf_va18 (0x%02x), buf_va19 (0x%02x)", buf_va[0], buf_va[1], buf_va[16], buf_va[17], buf_va[18], buf_va[19]);
	SOLOMON_DEBUG("WnByte (%d), ws_wd_nbyte (%d), ws_wd_ndelay (%d), RnByte (%d)", WnByte, ws_wd_nbyte, ws_wd_ndelay, RnByte);

	for (retries = 0; retries < 20; ++retries) {
		ret = i2c_transfer(dev->client->adapter, &msg[0], 1);
		if (ret < 0) {
			SOLOMON_WARNNING("i2c_transfer DMA TX error!\n");
			continue;
		}
		//memcpy(Rbuf, gpDMABuf_va, RnByte);
		//return ret;
	}

	/* for setup tx transaction. */
	udelay(ws_wd_ndelay);

	for (retries = 0; retries < 20; ++retries) {
		ret = i2c_transfer(dev->client->adapter, &msg[1], 1);
		if (ret < 0) {
			SOLOMON_WARNNING("i2c_transfer DMA RX error!\n");
			continue;
		}
		memcpy(Rbuf, gpDMABuf_va, RnByte);
		//return ret;
	}

	return 0;
#else
	err = i2c_master_send(dev->client, ws_wd, ws_wd_nbyte);

	if (err < 0) {
		SOLOMON_WARNNING("I2C READ CMD FAIL");
		return err;
	}

	/* for setup tx transaction. */
	udelay(ws_wd_ndelay);

	err = i2c_master_recv(dev->client, Rbuf, RnByte);

	if (err < 0) {
		SOLOMON_WARNNING("I2C READ DATA FAIL (%d)", err);
		return err;
	}

	return 0;
#endif
}

/*	read SEEPROM ID
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- int *id : pointer to save read ID
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_readid(struct solomon_device *dev, int *id)
{
	int err = 0;
	unsigned char ws_wd[10];
	unsigned char rs_wd[10];
	int ws_wd_nbyte = 0;

	/* WS_WD */
	ws_wd[ws_wd_nbyte++] = 0x9F;	/* 0x9F : JEDEC, ID READ COMMAND */
	/* COMMENCE */
	err = ds16_seeprom_general_ucmd_read(dev, ws_wd, ws_wd_nbyte, rs_wd, 3);

	/* RDATA
	 * JEDEC ID : ManufacturerID2 ManufacturerID1  DeviceID2
	 */
	*id = (int)((rs_wd[0] & 0xFF) << 16);
	*id |= (int)((rs_wd[1] & 0xFF) << 8);
	*id |= (int)((rs_wd[2] & 0xFF) << 0);

	SOLOMON_WARNNING("SEEPROM ID = 0x%04x", *id);

	return err;
}

/*	read SEEPROM address
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u32 add : address to read
 *		- u8 * Rbuf : array to save read address
 *		- int nbyte : length of Rbuf
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_read(struct solomon_device *dev, u32 add, u8 *Rbuf,
		int nbyte)
{
	unsigned char ws_wd[4];
	int ws_wd_nbyte;
	int err = 0;

	ws_wd_nbyte = 0;
	ws_wd[ws_wd_nbyte++] = 0x03;			/* CMD(0x03) */
	ws_wd[ws_wd_nbyte++] = (add&0xFF0000) >> 16;	/* ADD1 */
	ws_wd[ws_wd_nbyte++] = (add&0x00FF00) >> 8;	/* ADD2 */
	ws_wd[ws_wd_nbyte++] = (add&0x0000FF) >> 0;	/* ADD3 */

	err = ds16_seeprom_general_ucmd_read(dev, ws_wd, ws_wd_nbyte, Rbuf,
			nbyte);

	return err;
}

/*	write SEEPROM write enable flag
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_wren(struct solomon_device *dev)
{
	unsigned char ws_wd[1];
	int ws_wd_nbyte;

	ws_wd_nbyte = 0;
	ws_wd[ws_wd_nbyte++] = 0x06;

	return ds16_seeprom_general_ucmd_write(dev, ws_wd, ws_wd_nbyte);
}

/*	for many SEEPROM, page program has address boundary limitation 256B
 *	 boundary. Writing over 256B boundary, those will be written to the
 *	 start of the page. Data is more than 256B, the last 256B will be
 *	 written.
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u32 add : address to write SEEPROM
 *		- u8 *Wbuf : array to write
 *		- int nbyte : length of Wbuf
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_pp(struct solomon_device *dev, u32 add, u8 *Wbuf, int nbyte)
{
	unsigned char ws_wd[1024];
	int ws_wd_nbyte;

	ws_wd_nbyte = 0;
	ws_wd[ws_wd_nbyte++] = 0x02;
	ws_wd[ws_wd_nbyte++] = (add & 0xFF0000) >> 16 ;	/* ADD1 */
	ws_wd[ws_wd_nbyte++] = (add & 0x00FF00) >> 8 ;	/* ADD2 */
	ws_wd[ws_wd_nbyte++] = (add & 0x0000FF) >> 0 ;	/* ADD3 */

	memcpy(ws_wd+ws_wd_nbyte, Wbuf, nbyte);

	ws_wd_nbyte += nbyte;

	return ds16_seeprom_general_ucmd_write(dev, ws_wd, ws_wd_nbyte);
}

/*	read status register?
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u8 *rdsr : array for read status register
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_rdsr(struct solomon_device *dev, u8 *rdsr)
{
	unsigned char ws_wd[1];
	int ws_wd_nbyte;

	ws_wd_nbyte = 0;
	ws_wd[ws_wd_nbyte++] = 0x05;

	return ds16_seeprom_general_ucmd_read(dev, ws_wd, ws_wd_nbyte, rdsr, 2);
}

/*	write status register?
 *	Write Enable (WREN) command: WREN command is required to set the Write
 *	Enable Latch bit (WEL) before other command to change data. The WEL
 *	bit will return to reset stage under following situation:
 *	 - Power-up
 *	 - WRDI : Write Disable (WRDI) command completion
 *	 - WRSR : Write Status Register (WRSR) command completion
 *	 - PP   : Page Program (PP) command completion
 *	 - SE   : Sector Erase (SE) command completion
 *	 - BE   : Block Erase (BE) command completion
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u8 data : array to write status register
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_wrsr(struct solomon_device *dev, u8 data)
{
	unsigned char ws_wd[2];
	int ws_wd_nbyte = 0;

	ws_wd[ws_wd_nbyte++] = 0x01;
	ws_wd[ws_wd_nbyte++] = (data & 0xFF);

	return ds16_seeprom_general_ucmd_write(dev, ws_wd, ws_wd_nbyte);
}

/*	Sector Erase (SE) command completion
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u32 add : address to Sector Erase
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_se(struct solomon_device *dev, u32 add)
{
	unsigned char ws_wd[4];
	int ws_wd_nbyte = 0;

	ws_wd[ws_wd_nbyte++] = 0x20;			/* CMD(0x03) */
	ws_wd[ws_wd_nbyte++] = (add & 0xFF0000) >> 16;	/* ADD1 */
	ws_wd[ws_wd_nbyte++] = (add & 0x00FF00) >> 8;	/* ADD2 */
	ws_wd[ws_wd_nbyte++] = (add & 0x0000FF) >> 0;	/* ADD3 */

	return ds16_seeprom_general_ucmd_write(dev, ws_wd, ws_wd_nbyte);
}

/*	Block Erase (BE) command completion
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u32 add : address to Block Erase
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_be(struct solomon_device *dev, u32 add)
{
	unsigned char ws_wd[4];
	int ws_wd_nbyte = 0;

	ws_wd[ws_wd_nbyte++] = 0x52;
	ws_wd[ws_wd_nbyte++] = (add & 0xFF0000) >> 16;	/* ADD1 */
	ws_wd[ws_wd_nbyte++] = (add & 0x00FF00) >> 8;	/* ADD2 */
	ws_wd[ws_wd_nbyte++] = (add & 0x0000FF) >> 0;	/* ADD3 */

	return ds16_seeprom_general_ucmd_write(dev, ws_wd, ws_wd_nbyte);
}

/*	Chip(?) Erase (CE) command completion
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u32 add : address to Chip(?) Erase
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_ce(struct solomon_device *dev)
{
	unsigned char ws_wd[1];
	int ws_wd_nbyte = 0;

	ws_wd[ws_wd_nbyte++] = 0x60;

	return ds16_seeprom_general_ucmd_write(dev, ws_wd, ws_wd_nbyte);
}

/*	wait Busy clear from SEEPROM
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_wait_bsyclr(struct solomon_device *dev)
{
	int err = 0;
	unsigned int n;
	unsigned int rd;

	/* BUSY check */
	for (n = 0; n < 5000; n++) {
		err = ds16_seeprom_rdsr(dev, (u8 *)&rd);

		if (err != 0)
			break;

		if ((rd&0x03) == 0x00)	/* [1]WREN  [0]BUSY */
			break;

		mdelay(1);
	}

	return err;
}

/*	erase sector on MX25U1001E
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u32 add : address to erase
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_erase_sector_mx25v1001e(struct solomon_device *dev, u32 add)
{
	int err = 0;

	err = ds16_seeprom_wren(dev);

	if (err != 0)
		return err;

	err = ds16_seeprom_se(dev, add);

	if (err != 0)
		return err;

	mdelay(1);

	return ds16_seeprom_wait_bsyclr(dev);
}

/*	erase sector on MX25U1006E
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u32 add : address to erase
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_erase_sector_mx25v1006e(struct solomon_device *dev, u32 add)
{
	int err = 0;

	err = ds16_seeprom_wren(dev);

	if (err != 0)
		return err;

	err = ds16_seeprom_se(dev, add);

	if (err != 0)
		return err;

	mdelay(1);

	return ds16_seeprom_wait_bsyclr(dev);
}

/*	erase chip on MX25U1001E
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_erase_chip_mx25v1001e(struct solomon_device *dev)
{
	int err = 0;

	SOLOMON_WARNNING("Erase chip MX25V1001E");

	err = ds16_seeprom_wren(dev);

	if (err != 0)
		return err;

	err = ds16_seeprom_ce(dev);

	if (err != 0)
		return err;

	mdelay(1);

	return ds16_seeprom_wait_bsyclr(dev);
}

/*	erase chip on MX25U1006E
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_erase_chip_mx25v1006e(struct solomon_device *dev)
{
	int err = 0;

	SOLOMON_WARNNING("Erase chip MX25V1006E");

	err = ds16_seeprom_wren(dev);

	if (err != 0)
		return err;

	err = ds16_seeprom_ce(dev);

	if (err != 0)
		return err;

	mdelay(1);

	return ds16_seeprom_wait_bsyclr(dev);
}

/*	erase SEEPROM at inputed address and sector_cnt
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u32 st : start address to erase
 *		- int sector_cnt : count of erase sector
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_erase_nsector(struct solomon_device *dev, u32 st,
		int sector_cnt)
{
	int err = 0;
	int i = 0;

	for (i = 0; i < sector_cnt; i++) {
		err = ds16_seeprom_erase_sector(dev, st +
				m_SEEPROM_Align_Sector * i);

		if (err < 0)
			break;
	}

	return err;
}

/*	disable the write protect. If executed this function, can't write
 *	to SEEPROM.
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_enable_protect(struct solomon_device *dev)
{
	int err = 0;
	unsigned int rd;

	SOLOMON_WARNNING("ENABLE WRITE PROTECT. Can't write.");

	err = ds16_seeprom_wren(dev);

	if (err != 0)
		return err;

	err = ds16_seeprom_wrsr(dev, 0x0C);

	if (err != 0)
		return err;

	err = ds16_seeprom_rdsr(dev, (u8 *)&rd);

	if (err != 0)
		return err;

	return err;
}

/*	enable the write protect. If executed this function, can write
 *	to SEEPROM.
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_disable_protect(struct solomon_device *dev)
{
	int err = 0;
	unsigned int rd;

	SOLOMON_WARNNING("DISABLE WRITE PROTECT. Can write.");

	err = ds16_seeprom_wren(dev);

	if (err != 0)
		return err;

	err = ds16_seeprom_wrsr(dev, 0x02);

	if (err != 0)
		return err;

	err = ds16_seeprom_rdsr(dev, (u8 *)&rd);

	if (err != 0)
		return err;

	return err;
}

/*	read SEEPROM at inputed address
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u32 st : start address to read
 *		- u8 *data_buf : array to save read data
 *		- int nbyte : length of data_buf
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_read_nbyte(struct solomon_device *dev, u32 st, u8 *data_buf,
		int nbyte)
{
	int err = 0;
	int data_length = 0;
	int read_length = 0;
	int add_index = 0;

	SOLOMON_WARNNING("read nbyte");
	data_length = nbyte;

	for (data_length = nbyte; data_length > 0;
			data_length -= m_SEEPROM_ReadMax_Length) {
		if (data_length >= m_SEEPROM_ReadMax_Length)
			read_length = m_SEEPROM_ReadMax_Length;
		else
			read_length = data_length;

		err = ds16_seeprom_read(dev, st + add_index *
				m_SEEPROM_ReadMax_Length, data_buf + add_index *
				m_SEEPROM_ReadMax_Length, read_length);

		if (err != 0)
			break;

		add_index++;
	}

	return err;
}

/*	write SEEPROM at inputed address on MX25U1001E
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u32 st : start address to read
 *		- u8 *data_buf : array to save read data
 *		- int nbyte : length of data_buf
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_write_nbyte_mx25v1001e(struct solomon_device *dev, u32 st,
		u8 *data_buf, int nbyte)
{
	int err = 0;
	int data_length = 0;
	int write_length = 0;
	int add_index = 0;
	unsigned int target_add = 0;
	unsigned int first_send_length = 0;

	/* In case Write to SEEPROM, type of Write is m_SEEPROM_WriteMax_Length
	 * ex) incase write 128 data to address 0x1c0,
	 * if m_SEEPROM_WriteMax_Length is 256, 0x1c0 = 448,  448%256 = 192,
	 * 256-192 = 64. So, the first write data count is 64.
	 */
	data_length = nbyte;
	SOLOMON_WARNNING("(st : 0x%04x, m_SEEPROM_WriteMax_Length = %d)", st,
			m_SEEPROM_WriteMax_Length);

	first_send_length = st%m_SEEPROM_WriteMax_Length;

	if (first_send_length != 0) {
		first_send_length = m_SEEPROM_WriteMax_Length -
			first_send_length;

		err = ds16_seeprom_wren(dev);

		if (err != 0)
			return err;

		err = ds16_seeprom_pp(dev, st, data_buf, first_send_length);

		if (err != 0)
			return err;

		udelay(DELAY_PRE_WAIT_BUSY_CLEAR);

		err = ds16_seeprom_wait_bsyclr(dev);

		if (err != 0)
			return err;
	}

	target_add = st + first_send_length;
	for (data_length = (nbyte-first_send_length); data_length > 0;
			data_length -= m_SEEPROM_WriteMax_Length) {

		if (data_length >=  m_SEEPROM_WriteMax_Length)
			write_length = m_SEEPROM_WriteMax_Length;
		else
			write_length = data_length;

		err = ds16_seeprom_wren(dev);

		if (err != 0)
			return err;

		err = ds16_seeprom_pp(dev, target_add+add_index*
				m_SEEPROM_WriteMax_Length, data_buf+first_send_length+
				add_index*m_SEEPROM_WriteMax_Length, write_length);

		if (err != 0)
			return err;

		udelay(DELAY_PRE_WAIT_BUSY_CLEAR);

		err = ds16_seeprom_wait_bsyclr(dev);

		if (err != 0)
			return err;

		add_index++;
	}

	return err;
}

/*	write SEEPROM at inputed address on MX25U1006E
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u32 st : start address to read
 *		- u8 *data_buf : array to save read data
 *		- int nbyte : length of data_buf
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_write_nbyte_mx25v1006e(struct solomon_device *dev,
		u32 st, u8 *data_buf, int nbyte)
{
	int err = 0;
	int data_length = 0;
	int write_length = 0;
	int add_index = 0;
	unsigned int target_add = 0;
	unsigned int first_send_length = 0;

	/* In case Write to SEEPROM, type of Write is m_SEEPROM_WriteMax_Length
	 * ex) incase write 128 data to address 0x1c0,
	 * if m_SEEPROM_WriteMax_Length is 256, 0x1c0 = 448,  448%256 = 192,
	 * 256-192 = 64. So, the first write data count is 64.
	 */
	data_length = nbyte;
	SOLOMON_WARNNING("write nbyte MX25V1006E");
	first_send_length = st%m_SEEPROM_WriteMax_Length;

	if (first_send_length != 0) {
		first_send_length = m_SEEPROM_WriteMax_Length -
			first_send_length;

		err = ds16_seeprom_wren(dev);

		if (err != 0)
			return err;

		err = ds16_seeprom_pp(dev, st, data_buf, first_send_length);

		if (err != 0)
			return err;

		udelay(DELAY_PRE_WAIT_BUSY_CLEAR);

		err = ds16_seeprom_wait_bsyclr(dev);

		if (err != 0)
			return err;
	}

	target_add = st + first_send_length;
	for (data_length = (nbyte-first_send_length); data_length > 0;
			data_length -= m_SEEPROM_WriteMax_Length) {

		if (data_length >=  m_SEEPROM_WriteMax_Length)
			write_length = m_SEEPROM_WriteMax_Length;
		else
			write_length = data_length;

		err = ds16_seeprom_wren(dev);

		if (err != 0)
			return err;

		err = ds16_seeprom_pp(dev, target_add+add_index*
				m_SEEPROM_WriteMax_Length, data_buf+first_send_length+
				add_index*m_SEEPROM_WriteMax_Length, write_length);

		if (err != 0)
			return err;

		udelay(DELAY_PRE_WAIT_BUSY_CLEAR);

		err = ds16_seeprom_wait_bsyclr(dev);

		if (err != 0)
			return err;

		add_index++;
	}

	return err;
}

/*	Verify after update
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- struct solomon_fw *fw : struct updated content
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_verify(struct solomon_device *dev, struct solomon_fw *fw)
{
	int err = 0;
	int i = 0;
	u8 *verify_data = NULL;

	SOLOMON_WARNNING("START VERIFY (address : 0x%08x) >>>>", fw->address);
	if (fw == NULL) {
		SOLOMON_WARNNING("pointer for verify is NULL!!");
		return -1;
	}

	verify_data = kmalloc(fw->byte_cnt+2, GFP_KERNEL);

	if (verify_data == NULL) {
		SOLOMON_WARNNING("malloc pointer for verify fail!!");
		return -2;
	}

	err = ds16_seeprom_read_nbyte(dev, fw->address, verify_data,
			fw->byte_cnt);

	if (err != 0) {
		SOLOMON_WARNNING("Read fail for Verify");
		goto out;
	}

	for (i = 0; i < fw->byte_cnt; i++) {
		if (fw->content[i] != verify_data[i]) {
			SOLOMON_WARNNING("Download Fail!");
			SOLOMON_WARNNING("miss match %04d W:0x%02x R:0x%02x!",
					i, fw->content[i], verify_data[i]);
			err = -3;
			goto out;
		}
	}

out:
	kfree(verify_data);
	SOLOMON_WARNNING("END VERIFY >>>>");
	return err;
}

/*	It is updateing SEEPROM by section
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- struct solomon_fw *fw : struct to update content
 *		- int all : section erase or not. if 1 then not, 0 erase.
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int ds16_seeprom_firmware_update_by_section(struct solomon_device *dev,
		struct solomon_fw *section, int all)
{
	int err = 0;
	int retry = FW_MAX_RETRY_COUNT;
	int retry2 = FW_MAX_RETRY_COUNT;

	SOLOMON_WARNNING("\n");
	SOLOMON_WARNNING("FLASH ADDRESS : 0x%04x", section->address);
	SOLOMON_WARNNING("FLASH ERASE PAGE : 0x%04x", section->erase_page_cnt);
	SOLOMON_WARNNING("FLASH DATA COUNT : 0x%04x", section->byte_cnt);

	if (section->byte_cnt > 0 && section->content != NULL) {
		if (all != BOOT_UPDATE_ALL && section->erase_page_cnt > 0) {
			retry2 = FW_MAX_RETRY_COUNT;
			do {
				err = ds16_seeprom_erase_nsector(dev,
						section->address, section->erase_page_cnt);

				if (err == 0)
					break;

				mdelay(1);
			} while ((retry2--) > 1 && err < 0);
			if (err < 0)
				goto out;
		}
		retry = FW_MAX_RETRY_COUNT;
		do {
			err = ds16_seeprom_write_nbyte(dev, section->address,
					section->content, section->byte_cnt);

			if (err >= 0)
				break;
			mdelay(1);
		} while ((retry--) > 1);

		if (err < 0)
			goto out;

		retry = FW_MAX_RETRY_COUNT;
		do {
			err = ds16_seeprom_verify(dev, section);

			if (err >= 0)
				break;
			mdelay(1);
		} while ((retry--) > 1);
	}
out:
	return err;
}

int ds16_seeprom_firmware_update(struct solomon_device *dev,
		struct solomon_fw_group *fw_group, int all)
{
	int err = 0;
	int retry1 = FW_MAX_RETRY_COUNT, retry2 = FW_MAX_RETRY_COUNT;
	struct solomon_fw_group *gPtr = NULL;
	struct solomon_fw *ptr = NULL;

retry_all:
	SOLOMON_WARNNING("update start(retry:%d >>>", retry1);
	if ((retry1--) > 0) {
		if (all == BOOT_UPDATE_ALL) {
			retry2 = FW_MAX_RETRY_COUNT;
			do {
				err = ds16_seeprom_erase_chip(dev);

				if (err >= 0)
					break;
				mdelay(1);
			} while ((retry2--) > 1);
		}

		if (err < ERROR_SUCCESS)
			goto out;

		gPtr = fw_group;

		while (gPtr != NULL) {
			ptr = &(gPtr->section);
			err = ds16_seeprom_firmware_update_by_section(dev, ptr,
					all);

			if (err < 0)
				break;

			gPtr = gPtr->next;
		}

		if (err < ERROR_SUCCESS)
			goto retry_all;
	}
out:
	SOLOMON_WARNNING("<<<<<<<<<<<< update end err=0x%08x", err);

	return err;
}

/*	It is updating used header file
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- unsigned short eflash_flag : section flasg
 *		- int all : update all or each section.
 *	return : zero or greater than zero if success, else otherwise
 */
static int ds16_seeprom_fw_update_by_header(struct solomon_device *dev,
		struct solomon_fw_group *fw, int all)
{
	int err = ERROR_SUCCESS;

	err = ds16_seeprom_firmware_update(dev, fw, all);

	return err;
}

static int ds16_seeprom_fw_update_by_header_all(struct solomon_device *dev,
		struct solomon_fw_group_header *fw_header)
{
	int err = 0;

	err = ds16_seeprom_fw_update_by_header(dev, fw_header->fw_group,
			BOOT_UPDATE_ALL);
	return err;
}
#if 0
/*	it is check boot-up sequence
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u16 boot_flag : read boot-up flag
 *		- int *updated : updated flag
 *	return : zero or greater than zero if success, else otherwise
 */
static int ds16_seeprom_boot_up_check(struct solomon_device *dev,
		u16 boot_flag, int *updated)
{
	int err = 0;

	SOLOMON_WARNNING(">>>>> boot_flag : 0x%04x START", boot_flag);
	*updated = BOOT_UPDATE_NONE;

	if ((boot_flag & BOOT_STATUS_ERR_CPUCFG_ALL) > 0) {
		SOLOMON_WARNNING(">>>>> Solomon DS16 CPU CFG error!! update");
		*updated = BOOT_UPDATE_OK;
	}

	if ((boot_flag & BOOT_STATUS_ERR_SYS_CFG_FAIL) > 0) {
		SOLOMON_WARNNING(">>>>> Solomon DS16 SYS CFG error!! update");
		*updated = BOOT_UPDATE_OK;
	}

	SOLOMON_WARNNING(">>>>> END <<<<<");
	return err;
}
#endif
/*
 *	read the version from eflash address.
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- u16 eflashAddr: the eflash address to read version
 *		- int *version	: be written value by read from the eflash
 *			address
 *	return : 0 or greater then zero if success, else otherwise
 */
static int ds16_seeprom_fw_ds_read_version(struct solomon_device *dev,
		int eflashAddr, int *version)
{
	int err = ERROR_EFLAH_READ_FAIL;
	int retry = FW_MAX_RETRY_COUNT;

	do {
		err = ds16_seeprom_read_nbyte(dev, eflashAddr,
				(u8 *)version, 4);

		if (err == 0)
			break;
	} while ((retry--) > 0);

	SOLOMON_WARNNING("err : 0x%08x \t VERSION : 0x%08x", err, *version);

	return err;
}
#if 0
/*	compare checkum on SEEPROM
 *	eFlash section = version(4bytes)+bytecount(4bytes)+checksum(4bytes)+
 *		contens(bytecount bytes). CPU/CPU_CFG/SYS_CFG is checked by DS.
 *	paramter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- int address : SEEPROM address for update
 *	return : zero or greater than zero if success, else otherwise
 */
static int ds16_seeprom_fw_verify_checksum(struct solomon_device *dev,
		int address)
{
	int err = 0;
	int nBlock_cnt;
	int byte_cnt = 0, tByte_cnt = 0;
	int len = 0;
	u8 *verify_data = NULL;
	unsigned short *tmpContent = NULL;
	unsigned int checksum = 0x00;
	unsigned int header[3] = {0,};
	unsigned int hChecksum = 0x00;
	int tAddress = 0;

	/* header read 12bytes */
	err = ds16_seeprom_read_nbyte(dev, address, (u8 *)header,
			CONTENT_HEADER_SIZE);

	if (err < 0) {
		err = ERROR_EFLAH_READ_FAIL;
		goto out;
	}

	SOLOMON_WARNNING("READ eFlash Header(12bytes) : 0x%08x 0x%08x 0x%08x",
			header[0], header[1], header[2]);
	tByte_cnt = byte_cnt = header[1];
	hChecksum = header[2];
	SOLOMON_WARNNING("Converted byte_cnt = %d(0x%08x), checksum : 0x%08x",
			byte_cnt, byte_cnt, hChecksum);

	nBlock_cnt = byte_cnt/FW_MAX_I2C_DATA_COUNT;
	verify_data = kmalloc(byte_cnt+2, GFP_KERNEL);
	if (verify_data == NULL) {
		SOLOMON_WARNNING("malloc fail!!");
		err = -1;
	} else {
		tAddress = address + CONTENT_HEADER_SIZE;
		SOLOMON_WARNNING("CHECKSUM Address : 0x%08x", tAddress);
		SOLOMON_WARNNING("CHECKSUM READ START ");
		err = ds16_seeprom_read_nbyte(dev, tAddress, verify_data,
				byte_cnt);
		SOLOMON_WARNNING("CHECKSUM READ END\n");
		if (err == 0) {
			len = byte_cnt/2+(byte_cnt&0x01);
			tmpContent = (unsigned short *)verify_data;
			err = ds16_seeprom_fw_calc_checksum(len, tmpContent,
					&checksum);

			if (err >= ERROR_SUCCESS) {
				if (hChecksum != checksum)
					err = ERROR_PARSING_CHECKSUM_FAIL;
			}
		}

		kfree(verify_data);
	}
out:
	if (err >= 0)
		SOLOMON_WARNNING("CHECKSUM SUCCESS ");
	else
		SOLOMON_WARNNING("CHECKSUM FAIL ");

	return err;
}
#endif
/*	read whole version ( display and hidden version, product id, icname )
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
static int ds16_seeprom_fw_ds_read_version_all(struct solomon_device *dev)
{
	int err = 0;

	err = ds16_seeprom_fw_ds_read_version(dev, PRODUCT_ID_ADDR1,
			&(dev->fw_version.productID01));

	if (err < ERROR_SUCCESS)
		goto out;

	err = ds16_seeprom_fw_ds_read_version(dev, PRODUCT_ID_ADDR2,
			&(dev->fw_version.productID02));

	if (err < ERROR_SUCCESS)
		goto out;

	err = ds16_seeprom_fw_ds_read_version(dev, ICNAME_ADDR1,
			&(dev->fw_version.ICName01));

	if (err < ERROR_SUCCESS)
		goto out;

	err = ds16_seeprom_fw_ds_read_version(dev, ICNAME_ADDR2,
			&(dev->fw_version.ICName02));

	if (err < ERROR_SUCCESS)
		goto out;

	err = ds16_seeprom_fw_ds_read_version(dev, VERSION_DISPLAY_ADDR,
			&(dev->fw_version.display_version));

	if (err < ERROR_SUCCESS)
		goto out;

	err = ds16_seeprom_fw_ds_read_version(dev, VERSION_HIDDEN_ADDR,
			&(dev->fw_version.hidden_version));

	if (err < ERROR_SUCCESS)
		goto out;

out:
	return err;
}

/*	initialize before update
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int seeprom_download_initialize(struct solomon_device *dev)
{
	int err = 0;

	m_SEEPROM_ID = 0;
	err = ds16_seeprom_readid(dev, &m_SEEPROM_ID);

	if (err < 0)
		return err;

	switch (m_SEEPROM_ID) {
		case SEEPROM_ID_MX25V1006E:
			SOLOMON_WARNNING("-SEEPROM Name : MX25V1006E");
			m_SEEPROM_ReadMax_Length = 1024;
			m_SEEPROM_WriteMax_Length = 256;
			m_SEEPROM_Align_Sector = 4096;
			ds16_seeprom_erase_sector =
				ds16_seeprom_erase_sector_mx25v1006e;
			ds16_seeprom_erase_chip = ds16_seeprom_erase_chip_mx25v1006e;
			ds16_seeprom_write_nbyte = ds16_seeprom_write_nbyte_mx25v1006e;

			break;

		case SEEPROM_ID_MX25U1001E:
			SOLOMON_WARNNING("-SEEPROM Name : MX25U1001E");
			m_SEEPROM_ReadMax_Length = 1024;
			m_SEEPROM_WriteMax_Length = 32;
			m_SEEPROM_Align_Sector = 4096;
			ds16_seeprom_erase_sector =
				ds16_seeprom_erase_sector_mx25v1001e;
			ds16_seeprom_erase_chip = ds16_seeprom_erase_chip_mx25v1001e;
			ds16_seeprom_write_nbyte = ds16_seeprom_write_nbyte_mx25v1001e;

			break;

		default:
			SOLOMON_WARNNING("-SEEPROM Name : None");
			m_SEEPROM_ReadMax_Length = 1024;
			m_SEEPROM_WriteMax_Length = 256;
			m_SEEPROM_Align_Sector = 4096;
			ds16_seeprom_erase_sector =
				ds16_seeprom_erase_sector_mx25v1006e;
			ds16_seeprom_erase_chip = ds16_seeprom_erase_chip_mx25v1006e;
			ds16_seeprom_write_nbyte = ds16_seeprom_write_nbyte_mx25v1006e;

			break;
	}

	return err;
}

/*	update SEEPROM use struct solomon_fw_group.
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- struct solomon_fw_group *fw_group : struct to update SEEPROM
 *		- int all : erase all or each flag
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */
int seeprom_firmware_update(struct solomon_device *dev,
		struct solomon_fw_group *fw_group, int all)
{
	int err = 0;

	SOLOMON_WARNNING("update start >>>>>>>>>>>>>>>");

	err = seeprom_download_initialize(dev);

	if (err < 0) {
		SOLOMON_WARNNING("initialize fail!!");
		return err;
	}

	err = ds16_seeprom_disable_protect(dev);

	if (err < 0)
		return err;	/* Disalbe write protect */

	err = ds16_seeprom_firmware_update(dev, fw_group, all);
	SOLOMON_WARNNING("<<<<<<<<<<<< update end err=0x%08x", err);

	ds16_seeprom_enable_protect(dev);	/* enalbe write protect */

	return err;
}

/*	update SEEPROM when do boot-up sequence.
 *
 *	parameter :
 *		- struct solomon_device *dev : solomon I2C device
 *		- struct solomon_fw_group_header *fw_header : header info
 *
 *	return : >= 0 thend success, else ( < 0 ) fail
 */

int seeprom_firmware_pre_boot_up_check(struct solomon_device *dev,
		struct solomon_fw_group_header *fw_header)
{
	int err = 0;

	SOLOMON_WARNNING("boot-up check start >>>>>>>>>>>>>>>");

	err = seeprom_download_initialize(dev);

	if (err < 0) {
		SOLOMON_WARNNING("initialize fail!!");
		return err;
	}

	err = ds16_seeprom_disable_protect(dev);

	if (err < 0)
		return err;	/* Disalbe write protect */

	err = ds16_seeprom_fw_update_by_header_all(dev, fw_header);

	ds16_seeprom_enable_protect(dev);	/* enalbe write protect	*/
	SOLOMON_WARNNING("<<<<<<<<<<<< boot-up check end err=0x%08x", err);
	return err;
}

int seeprom_get_version_boot(struct solomon_device *dev)
{
	return ds16_seeprom_fw_ds_read_version_all(dev);
}
