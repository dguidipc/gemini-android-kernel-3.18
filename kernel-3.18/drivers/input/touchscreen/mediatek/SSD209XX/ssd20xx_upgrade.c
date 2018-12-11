/*
 * Copyright 2017 Solomon Systech Ltd. All rights reserved.
 *
 * SSL SSD20xx Touch upgrade driver
 *
 * Date: 2017.04.19
 */
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/mm.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/input/mt.h>

#include "ssd20xx.h"
#include "ssd20xx_mx25v100.h"
#include "ssd20xx_upgrade.h"
#if defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE)
extern int found_force_bin_file;
#endif

/* level of state for parsing */
enum VAL_LEVEL {
	LEVEL_NONE = 0,
	LEVEL_ADDRESS,
	LEVEL_LENGTH,
	LEVEL_ERASE_SIZE,
	LEVEL_VERSION,
	LEVEL_CHECKSUM,
	LEVEL_RESERVE_01,
	LEVEL_RESERVE_02,
	LEVEL_RESERVE_03,
	LEVEL_CONTENT,
};

/* level of state for dollar */
enum VAL_DOLLAR_LEVEL {
	LEVEL_DOLLAR_NONE = 0,
	LEVEL_DOLLAR_ERASE_TYPE,
};

struct solomon_fw_group_header fw_header;

int m_fw_head_bin_flag;		/* 0 : header , 1:bin */

/* show section info */
void view_section(struct solomon_fw fw)
{
#if 0
	int i = 0;
	int len = 0;
	unsigned short *tmpContent = NULL;

	len = fw.byte_cnt;

	SOLOMON_WARNNING("\n\n");
	SOLOMON_WARNNING("\t address : 0x%08x", fw.address);
	SOLOMON_WARNNING("\t byte_cnt : 0x%08x", fw.byte_cnt);
	SOLOMON_WARNNING("\t erase_page_cnt : 0x%08x", fw.erase_page_cnt);
	SOLOMON_WARNNING("\t version : 0x%08x", fw.version);
	SOLOMON_WARNNING("\t checksum : 0x%08x", fw.checksum);
	SOLOMON_WARNNING("\t reserved_01 : 0x%08x", fw.reserved_01);
	SOLOMON_WARNNING("\t reserved_02 : 0x%08x", fw.reserved_02);
	SOLOMON_WARNNING("\t reserved_03 : 0x%08x", fw.reserved_03);

	tmpContent = (unsigned short *)fw.content;

	if (fw.content != NULL) {
		for (i = 0; i < 40; i += 4) {
			SOLOMON_WARNNING("\t0x%02x%02x%02x%02x \t 0x%04x%04x",
					fw.content[i], fw.content[i+1],
					fw.content[i+2], fw.content[i+3],
					tmpContent[i/2], tmpContent[i/2+1]);
		}
		SOLOMON_WARNNING("\t.");
		SOLOMON_WARNNING("\t.");
		SOLOMON_WARNNING("\t.");
		SOLOMON_WARNNING("\t.");
		for (i = len - 40; i < len; i += 4) {
			SOLOMON_WARNNING("\t0x%02x%02x%02x%02x \t 0x%04x%04x",
					fw.content[i], fw.content[i+1],
					fw.content[i+2], fw.content[i+3],
					tmpContent[i/2], tmpContent[i/2+1]);
		}
	}
#endif
}

/*	Convert hex string to int.
 *	parameter :
 *		unsigned char hex[] : hex string to convert int
 *		int hexLen : hex string length
 *		unsigned int *iRet : int value converted
 *	return : if 0 then success, else fail
 */
static int conv_hex_str2int(unsigned char hex[], int hexLen, unsigned int *iRet)
{
	int ret = 0;
	int hexIdx = 0;
	int temp = 0;
	int i = 0;

	if (hexLen < 0)
		return ERROR_PARSING_INVALID_DATATYPE;

	hexIdx = hexLen - 1;
	*iRet = 0;

	for (i = 0; i < hexLen; i++) {
		if (hex[i] >= 0x30 && hex[i] <= 0x39)
			temp = hex[i] - 0x30;
		else if (hex[i] >= 0x41 && hex[i] <= 0x46)
			temp = hex[i] - 0x37;
		else if (hex[i] >= 0x61 && hex[i] <= 0x66)
			temp = hex[i] - 0x57;
		else {
			ret = -2;
			break;
		}

		*iRet += (temp << (4 * hexIdx));
		hexIdx--;
	}

	return ret;
}

/*	calculate checksum
 *	parameter :
 *		- int len : tmpContent's length to calculate checksum
 *		- unsigned short tmpContent : To calculate checksum
 *		- unsigned int *checksum : result of calculate checksum
 *	return : 0 is success, else fail
 */
int fw_calc_checksum(int len, unsigned short *tmpContent,
		unsigned int *checksum)
{
	int i = 0;
	int ret = ERROR_SUCCESS;
	unsigned short sum = 0x00;
	unsigned short xor = 0x00;

	if (tmpContent == NULL)
		return ERROR_PARSING_CHECKSUM_FAIL;

	SOLOMON_WARNNING("\n");
	SOLOMON_WARNNING("0x%04x 0x%04x\n", tmpContent[0], tmpContent[1]);
	SOLOMON_WARNNING("\n");

	for (i = 0; i < len; i++) {
		sum += tmpContent[i];
		xor ^= tmpContent[i];
	}

	*checksum = (xor << 16) | sum;

	SOLOMON_WARNNING(">>>> sum:0x%04x, xor:0x%04x, checksum:0x%08x\n",
			sum, xor, *checksum);

	return ret;
}

/*	compare checksum
 *	parameter :
 *		- struct solomon_fw sec : section to calculate checksum
 *	return : If checksum is equal then 0, else fail
 */
int fw_checksum(struct solomon_fw sec)
{
	int ret = ERROR_SUCCESS;
	unsigned int checksum = 0x00;
	unsigned short *tmpContent = NULL;
	int len = 0;

	if (sec.address == 0x00000000 || sec.address == 0x000001c0 ||
			sec.address == 0x00001000) {

		len = sec.byte_cnt / 2 + (sec.byte_cnt & 0x01);
		tmpContent = (unsigned short *)sec.content;
	} else {
		len = (sec.byte_cnt - CONTENT_HEADER_SIZE) / 2 +
			((sec.byte_cnt - CONTENT_HEADER_SIZE) & 0x01);
		tmpContent = (u16 *)(sec.content + CONTENT_HEADER_SIZE);
	}

	ret = fw_calc_checksum(len, tmpContent, &checksum);

	if (ret == ERROR_SUCCESS) {
		if (sec.checksum != checksum)
			ret = ERROR_PARSING_CHECKSUM_FAIL;
	}

	return ret;
}

#ifdef SUPPORT_TEST_MODE
/* view error message */
void view_error_msg(int errnum)
{
	if ((errnum & ERROR_TYPE_PARSING) == ERROR_TYPE_PARSING)
		SOLOMON_WARNNING("[parsing");
	else if ((errnum & ERROR_TYPE_PARSING) == ERROR_TYPE_UPDATE)
		SOLOMON_WARNNING("[update]");

	SOLOMON_WARNNING("errnum : 0x%08x", errnum);

	if (errnum == ERROR_SUCCESS)
		SOLOMON_WARNNING("SUCCESS!!");
	else if (errnum == ERROR_PARSING_FILENAME_IS_NULL)
		SOLOMON_WARNNING("File nmae is fail!!");
	else if (errnum == ERROR_PARSING_FILE_OPEN_FAIL)
		SOLOMON_WARNNING("File open error!!");
	else if (errnum == ERROR_PARSING_FORMAT_INVALID)
		SOLOMON_WARNNING("Merge file format error!!");
	else if (errnum == ERROR_PARSING_CHECKSUM_FAIL)
		SOLOMON_WARNNING("Checksum fail!!");
	else if (errnum == ERROR_PARSING_MALLOC_FAIL)
		SOLOMON_WARNNING("Malloc fail!!");
	else if (errnum == ERROR_PARSING_CONTENT_SIZE_FAIL)
		SOLOMON_WARNNING("Content size fail!!");
	else if (errnum == ERROR_PARSING_DATA_CNT_FAIL)
		SOLOMON_WARNNING("Data count fail!!");
	else if (errnum == ERROR_PARSING_HEADER_DATA_INVALID_LENGTH)
		SOLOMON_WARNNING("The header data length invalid!!");
	else if (errnum == ERROR_PARSING_INVALID_DATATYPE)
		SOLOMON_WARNNING("The merge file have invalid data type!!");
	else if (errnum == ERROR_UPDATE_INIT_FAIL)
		SOLOMON_WARNNING("Update init fail!!");
	else if (errnum == ERROR_UPDATE_ERASE_FAIL)
		SOLOMON_WARNNING("Update erase fail!!");
	else if (errnum == ERROR_UPDATE_WRITE_FAIL)
		SOLOMON_WARNNING("Update write fail!!");
	else if (errnum == ERROR_UPDATE_READ_FAIL)
		SOLOMON_WARNNING("Update read fail!!");
	else if (errnum == ERROR_UPDATE_VERIFY_FAIL)
		SOLOMON_WARNNING("Update verify fail!!");
	else if (errnum == ERROR_EFLAH_ERASE_FAIL)
		SOLOMON_WARNNING("Eflash erase fail!!");
	else if (errnum == ERROR_EFLAH_WRITE_FAIL)
		SOLOMON_WARNNING("Eflash write fail!!");
	else if (errnum == ERROR_EFLAH_READ_FAIL)
		SOLOMON_WARNNING("Eflash read fail!!");
	else if (errnum == ERROR_EFLAH_VERIFY_FAIL)
		SOLOMON_WARNNING("Eflash verify fail!!");
	else if (errnum == ERROR_SYSTEM_FAIL)
		SOLOMON_WARNNING("Syste fail!!");
	else if (errnum == ERROR_VERSION_CHECK_FAIL)
		SOLOMON_WARNNING("Version check fail!!");
	else if (errnum == ERROR_VERIFY_VERIFY_FAIL)
		SOLOMON_WARNNING("The Verify verify fail!!");
	else
		SOLOMON_WARNNING("Unknown error!!");
}
#endif

/* It is free the solomon firmware struct to malloc */
int fw_free(struct solomon_fw_group *fw, int data_free)
{
	int i = 30;
	struct solomon_fw_group *ptr = fw;

	while (fw != NULL) {
		if ((i--) < 1)
			break;

		ptr = fw->next;

		if (data_free == 1 && fw->section.content != NULL)
			kfree(fw->section.content);

		kfree(fw);
		fw = ptr;
	}

	return 0;
}

/* make F/w linked list. */
static inline int solomon_fw_make_link(struct solomon_fw_group **head,
		struct solomon_fw_group **tail, struct solomon_fw *section)
{
	int err = 0;
	struct solomon_fw_group *ptr = NULL;

	ptr = kmalloc(sizeof(struct solomon_fw_group), GFP_KERNEL);

	if (ptr == NULL)
		return ERROR_PARSING_MALLOC_FAIL;

	ptr->section.address = section->address;
	ptr->section.byte_cnt = section->byte_cnt;
	ptr->section.erase_page_cnt = section->erase_page_cnt;
	ptr->section.version = section->version;
	ptr->section.checksum = section->checksum;
	ptr->section.reserved_01 = section->reserved_01;
	ptr->section.reserved_02 = section->reserved_02;
	ptr->section.reserved_03 = section->reserved_03;
	ptr->section.content = section->content;

	ptr->next = NULL;

	if (*head == NULL) {
		*head = *tail = ptr;
	} else {
		(*tail)->next = ptr;
		*tail = ptr;
	}

	return err;
}

/* Parsing F/W file for update */
static struct solomon_fw_group *parse_uchar2int_arr(const u8 *data,
		size_t data_size, struct solomon_fw_group_header *fw_header,
		int *errnum, int *all)
{
	struct solomon_fw_group *head = NULL, *tail = NULL, *ptr = NULL;
	unsigned char buff[128] = {0,};
	int buff_idx = 0;
	int ret = ERROR_SUCCESS;
	unsigned char ch;
	int bComment = 0;
	enum VAL_LEVEL level = LEVEL_NONE;
	enum VAL_DOLLAR_LEVEL dollar_level = LEVEL_DOLLAR_NONE;
	int iRet = 0;
	int dollar_cnt = 0;
	size_t data_idx = 0;

	do {
		memset(buff, 0, sizeof(buff));
		buff_idx = 0;
		bComment = 0;

		while (data_size > data_idx && buff_idx < 128) {
			ch = data[data_idx++];

			if (ch == '\n')
				break;
			if (ch == '\r' || ch == '\t' || ch == ' ' || ch == 0x09)
				continue;
			else if (ch == ';')
				bComment = 1;
			else if (bComment == 0)
				buff[buff_idx++] = ch;
		}

		if (data_size <= data_idx)
			break;

		if (strlen(buff) < 1)
			continue;

		if (buff[0] == '$') {
			dollar_cnt++;

			if (dollar_cnt == 1) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.display_version = iRet;
				SOLOMON_WARNNING("display version : 0x%08x",
						iRet);
			} else if (dollar_cnt == 2) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.hidden_version = iRet;
				SOLOMON_WARNNING("hidden version : 0x%08x",
						iRet);
			} else if (dollar_cnt == 3) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.productID01 = iRet;
				SOLOMON_WARNNING("productID01 : 0x%08x", iRet);
			} else if (dollar_cnt == 4) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.productID02 = iRet;
				SOLOMON_WARNNING("productID02 : 0x%08x", iRet);
			} else if (dollar_cnt == 5) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.ICName01 = iRet;
				SOLOMON_WARNNING("ICName01 : 0x%08x", iRet);
			} else if (dollar_cnt == 6) {
				ret = conv_hex_str2int((buff + 1),
						strlen(buff) - 1, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				fw_header->fw_version.ICName02 = iRet;
				SOLOMON_WARNNING("ICName02 : 0x%08x", iRet);
			}
			if (dollar_level == LEVEL_DOLLAR_NONE &&
					dollar_cnt == 9) {
				ret = conv_hex_str2int((buff + 1), 4, &iRet);

				if (ret < ERROR_SUCCESS)
					break;

				*all = iRet;
				SOLOMON_WARNNING("ALL Update flag : %d", *all);
				dollar_level = LEVEL_DOLLAR_ERASE_TYPE;
			}
			continue;
		}

		if (buff[0] == '#') {
			if (strlen(buff + 1) < 1) {
				ret = ERROR_PARSING_HEADER_DATA_INVALID_LENGTH;
				break;
			}

			if (level != LEVEL_NONE) {
				ret = ERROR_PARSING_FORMAT_INVALID;
				break;
			}

			ptr = kmalloc(sizeof(struct solomon_fw_group),
					GFP_KERNEL);

			if (ptr == NULL) {
				ret = ERROR_PARSING_MALLOC_FAIL;
				break;
			}

			memset(ptr, 0x00, sizeof(struct solomon_fw_group));

			ret = conv_hex_str2int((buff + 1), strlen(buff) - 1,
					&iRet);

			if (ret < ERROR_SUCCESS)
				break;

			ptr->section.address = iRet;
			SOLOMON_WARNNING("address : 0x%08x\n",
					ptr->section.address);

			if (head == NULL)
				head = tail = ptr;
			else {
				tail->next = ptr;
				tail = ptr;
			}

			level = LEVEL_ADDRESS;
		} else if (buff[0] == '*') {
			if (strlen(buff + 1) < 1) {
				ret = ERROR_PARSING_HEADER_DATA_INVALID_LENGTH;
				break;
			}

			if (level < LEVEL_ADDRESS) {
				ret = ERROR_PARSING_FORMAT_INVALID;
				break;
			}

			ret = conv_hex_str2int((buff + 1), strlen(buff) - 1,
					&iRet);

			if (ret < ERROR_SUCCESS)
				break;

			if (level == LEVEL_ADDRESS) {
				if (iRet < 1) {
					ret = ERROR_PARSING_DATA_CNT_FAIL;
					break;
				}
				ptr->section.byte_cnt = iRet;
				SOLOMON_WARNNING("byte_cnt : 0x%08x\n",
						ptr->section.byte_cnt);

				level = LEVEL_LENGTH;
			} else if (level == LEVEL_LENGTH) {
				/* is not used. Because CPU_CFG erase_page_cnt = 0. */
				/* if (iRet < 1) {
				 *	ret = ERROR_PARSING_DATA_CNT_FAIL;
				 *	break;
				 * }
				 */
				ptr->section.erase_page_cnt = iRet;
				SOLOMON_WARNNING("erase_page_cnt : 0x%08x\n",
						ptr->section.erase_page_cnt);
				level = LEVEL_ERASE_SIZE;
			} else if (level == LEVEL_ERASE_SIZE) {
				ptr->section.version = iRet;
				SOLOMON_WARNNING(">>> version : 0x%08x\n",
						ptr->section.version);
				level = LEVEL_VERSION;
			} else if (level == LEVEL_VERSION) {
				SOLOMON_WARNNING("%s", buff);
				SOLOMON_WARNNING("checksum before : 0x%08x",
						iRet);
				ptr->section.checksum = (unsigned int)iRet;
				SOLOMON_WARNNING("checksum : 0x%08x\n",
						ptr->section.checksum);
				level = LEVEL_CHECKSUM;
			} else if (level == LEVEL_CHECKSUM) {
				ptr->section.reserved_01 = iRet;
				SOLOMON_WARNNING("reserved_01 : 0x%08x\n",
						ptr->section.reserved_01);
				level = LEVEL_RESERVE_01;
			} else if (level == LEVEL_RESERVE_01) {
				ptr->section.reserved_02 = iRet;
				SOLOMON_WARNNING("reserved_02 : 0x%08x\n",
						ptr->section.reserved_03);
				level = LEVEL_RESERVE_02;
			} else if (level == LEVEL_RESERVE_02) {
				ptr->section.reserved_03 = iRet;
				SOLOMON_WARNNING(">>> reserved_03 : 0x%08x\n",
						ptr->section.reserved_03);
				ptr->section.content =
					kmalloc(ptr->section.byte_cnt,
							GFP_KERNEL);

				if (ptr->section.content == NULL) {
					ret = ERROR_PARSING_MALLOC_FAIL;
					break;
				}

				memcpy(ptr->section.content, data + data_idx,
						ptr->section.byte_cnt);

				data_idx += ptr->section.byte_cnt;

				SOLOMON_WARNNING("SIZE %d %d\n",
						ptr->section.byte_cnt, ret);
				/*if (ret != ptr->section.byte_cnt) {
				 *	ret = ERROR_PARSING_CONTENT_SIZE_FAIL;
				 *	break;
				 *}
				 */

				ret = fw_checksum(ptr->section);

				if (ret < ERROR_SUCCESS)
					break;

				level = LEVEL_NONE;
			}
		} else {
			ret = ERROR_PARSING_FORMAT_INVALID;
		}
	} while (ret >= 0);

	*errnum = ret;

	if (ret < 0) {
		fw_free(head, 1);
		head = NULL;
	}

	fw_header->fw_group = head;

	return head;
}

/* make FW linke list for checksum */
static int solomon_make_checksum_link(struct solomon_device *dev,
		struct solomon_fw_group_header *fw_header)
{
	int err = 0;
	int i = 0, len = 0, check = 0;
	int elseChecksum[] = {
		0x00000000
			, 0x000001C0
			, 0x00001000
	};
	struct solomon_fw_group *head = NULL, *chkHead = NULL, *chkTail = NULL;

	len = sizeof(elseChecksum) / sizeof(int);
	SOLOMON_WARNNING("size of else checksum : %d", len);

	head = fw_header->fw_group;

	while (head) {
		SOLOMON_WARNNING("address : 0x%08x", head->section.address);
		check = 0;
		for (i = 0; i < len; i++) {
			if (head->section.address == elseChecksum[i]) {
				SOLOMON_WARNNING("except address");
				check = 1;
				break;
			}
		}

		if (check == 0) {
			SOLOMON_WARNNING("add address");
			err = solomon_fw_make_link(&chkHead, &chkTail,
					&head->section);

			if (err < 0)
				return err;
		}
		head = head->next;
	}

	return err;
}

/* make FW linke list using FW hex file */
static int solomon_make_header_use_bin(struct solomon_device *dev,
		struct solomon_fw_group_header *fw_header, const u8 *data,
		size_t data_size)
{
	int err = 0;
	int errnum = 0;
	int all = BOOT_UPDATE_EACH;

	SOLOMON_TIME("S02");
	fw_header->fw_group = NULL;

	if (parse_uchar2int_arr(data, data_size, fw_header,
				&errnum, &all) != NULL)
		err = solomon_make_checksum_link(dev, fw_header);

	SOLOMON_TIME("E02");
	return err;
}

/* make FW linke list using FW Header file */
static int solomon_make_header_use_head(struct solomon_device *dev,
		struct solomon_fw_group_header *fw_header)
{
	int err = 0;
	struct solomon_fw_group *head = NULL, *tail = NULL;

	SOLOMON_TIME("S02");
	/* get version from header file */
	fw_header->fw_version.productID01 = SSL_FW_CFG.content[76] |
		(SSL_FW_CFG.content[77] << 8) |
		(SSL_FW_CFG.content[78] << 16) |
		(SSL_FW_CFG.content[79] << 24);

	fw_header->fw_version.productID02 = SSL_FW_CFG.content[80] |
		(SSL_FW_CFG.content[81] << 8) |
		(SSL_FW_CFG.content[82] << 16) |
		(SSL_FW_CFG.content[83] << 24);

	fw_header->fw_version.ICName01 = SSL_FW_CFG.content[84] |
		(SSL_FW_CFG.content[85] << 8) |
		(SSL_FW_CFG.content[86] << 16) |
		(SSL_FW_CFG.content[87] << 24);

	fw_header->fw_version.ICName02 = SSL_FW_CFG.content[88] |
		(SSL_FW_CFG.content[89] << 8) |
		(SSL_FW_CFG.content[90] << 16) |
		(SSL_FW_CFG.content[91] << 24);

	fw_header->fw_version.display_version = SSL_FW_CFG.content[68] |
		(SSL_FW_CFG.content[69] << 8) |
		(SSL_FW_CFG.content[70] << 16) |
		(SSL_FW_CFG.content[71] << 24);

	fw_header->fw_version.hidden_version = SSL_FW_CFG.content[72] |
		(SSL_FW_CFG.content[73] << 8) |
		(SSL_FW_CFG.content[74] << 16) |
		(SSL_FW_CFG.content[75] << 24);

	/* make firmware link */
	err = solomon_fw_make_link(&head, &tail, &SSL_SYS_CFG);

	if (err < 0)
		return err;

	err = solomon_fw_make_link(&head, &tail, &SSL_FW_CFG);

	if (err < 0)
		return err;

	err = solomon_fw_make_link(&head, &tail, &SSL_FW);

	if (err < 0)
		return err;

	err = solomon_fw_make_link(&head, &tail, &SSL_TMC_REG);

	if (err < 0)
		return err;

	err = solomon_fw_make_link(&head, &tail, &SSL_DCSW);

	if (err < 0)
		return err;

	err = solomon_fw_make_link(&head, &tail, &SSL_FDM);

	if (err < 0)
		return err;

	err = solomon_fw_make_link(&head, &tail, &SSL_MPFPM);

	if (err < 0)
		return err;

	err = solomon_fw_make_link(&head, &tail, &SSL_MPFDM);

	if (err < 0)
		return err;

	err = solomon_fw_make_link(&head, &tail, &SSL_FPM);

	if (err < 0)
		return err;

	fw_header->fw_group = head;

	SOLOMON_TIME("E02");
	return err;
}

/* DS init */
static int fw_solomon_init(struct solomon_device *dev)
{
	int ret = 0;

	SOLOMON_WARNNING("initialize start >>>>>>>>>>>>>>>");

	ret = ds_eflash_write(dev->client, 0xE003, 0x0007);

	if (ret < 0)
		return ERROR_SYSTEM_FAIL;

	ret = ds_eflash_write(dev->client, 0xE000, 0x0048);

	if (ret < 0)
		return ERROR_SYSTEM_FAIL;

	SOLOMON_WARNNING("<<<<<<<<<<<< initialize end(%d)", ret);
	return ret;
}

int solomon_free_header(struct solomon_device *dev)
{
	int err = 0;
	struct solomon_fw_group *head = NULL;

	if (dev == NULL)
		return 0;

	head = fw_header.fw_group;

	if (head != NULL)
		fw_free(head, m_fw_head_bin_flag);

	return err;
}

/* check version between SEEPROM and Header.
 * ( display and hidden version, product id, icname )
 */
static int solomon_firmware_version_check(struct solomon_device *dev,
		struct solomon_fw_group_header *fw_header)
{
	SOLOMON_WARNNING("PRODUCT ID E:0x%08x 0x%08x / H:0x%08x 0x%08x",
			dev->fw_version.productID01,
			dev->fw_version.productID02,
			fw_header->fw_version.productID01,
			fw_header->fw_version.productID02);

	SOLOMON_WARNNING("ICNAME E:0x%08x 0x%08x / H:0x%08x 0x%08x",
			dev->fw_version.ICName01,
			dev->fw_version.ICName02,
			fw_header->fw_version.ICName01,
			fw_header->fw_version.ICName02);

	SOLOMON_WARNNING("DISPLAY VERSION  EFLASH : 0x%08x\t0x%08x",
			dev->fw_version.display_version,
			fw_header->fw_version.display_version);

	SOLOMON_WARNNING("HIDDEN VERSION  EFLASH : 0x%08x\tHEADER : 0x%08x",
			dev->fw_version.hidden_version,
			fw_header->fw_version.hidden_version);

	if (dev->fw_version.display_version == 0xFFFFFFFF &&
			fw_header->fw_version.display_version != 0xFFFFFFFF)
		goto update;

	/* If hidden version of Header or seeprom is HIDDEN_VERSION_FACTORY,
	 * update the SEEPROM from header file.
	 */
	if ((dev->fw_version.hidden_version ==
				HIDDEN_VERSION_FACTORY) &&
		(fw_header->fw_version.hidden_version !=
			 HIDDEN_VERSION_FACTORY)) {
		SOLOMON_WARNNING("SEEPROM has the factory version in HV");
		goto update;
	}
	if ((dev->fw_version.hidden_version !=
				HIDDEN_VERSION_FACTORY) &&
			(fw_header->fw_version.hidden_version ==
			 HIDDEN_VERSION_FACTORY)) {
		SOLOMON_WARNNING("Header has the factory version in HV.");
		goto update;
	}

	/* The display version must over 1. If the display version is 0,
	 * then driver update the SEEPROM from header file.
	 */
	if (dev->fw_version.display_version == 0x00 &&
			fw_header->fw_version.display_version != 0x00) {
		SOLOMON_WARNNING("SEEPROM display version is 0. upgrade.");
		goto update;
	}

	if (dev->fw_version.productID01 !=
		fw_header->fw_version.productID01 ||
		dev->fw_version.productID02 !=
		fw_header->fw_version.productID02) {
		SOLOMON_WARNNING("Produect ID mismatch!!!");
		goto out;
	}

	if (dev->fw_version.ICName01 !=
		fw_header->fw_version.ICName01 ||
		dev->fw_version.ICName02 !=
		fw_header->fw_version.ICName02) {
		SOLOMON_WARNNING("IC Name mismatch!!!");
		goto out;
	}

	if (dev->fw_version.display_version <
			fw_header->fw_version.display_version) {
		SOLOMON_WARNNING("Detect new display version!!");
		goto update;
	} else if (dev->fw_version.display_version ==
			fw_header->fw_version.display_version) {
		if (dev->fw_version.hidden_version <
				fw_header->fw_version.hidden_version) {
			SOLOMON_WARNNING("Detect new hidden version!!");
			goto update;
		}
	}
out:
	return 0;

update:
	return -1;
}

/* Update using F/W file. */
int solomon_firmware_update_byfile(struct solomon_device *dev, char *filename)
{
	int errnum = 0;
	int retry = FW_MAX_RETRY_COUNT;
	struct solomon_fw_group_header fw_header_tmp;
	struct solomon_fw_group *head = NULL;
	int all = BOOT_UPDATE_EACH;
	struct file *src;
	mm_segment_t oldfs;
	size_t fw_size = 0, read_size = 0;
	u8 *fw_data = NULL;

	SOLOMON_TIME("S91");
	SOLOMON_TIME("S92");

	if (filename == NULL)
		return ERROR_PARSING_FILENAME_IS_NULL;

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	src = filp_open(filename, O_RDONLY, S_IRUSR|S_IRGRP|S_IROTH);
	if (IS_ERR(src)) {
		SOLOMON_WARNNING("[%s] file open error!!", filename);
		return ERROR_PARSING_FILE_OPEN_FAIL;
	}

	fw_size = src->f_path.dentry->d_inode->i_size;

	if (fw_size > 0) {
		fw_data = kzalloc(fw_size, GFP_KERNEL);
		read_size = vfs_read(src, (char __user *)fw_data, fw_size,
				&src->f_pos);

		SOLOMON_WARNNING("file path %s, size %lu Bytes\n",
				filename, fw_size);
		if (read_size != fw_size) {
			SOLOMON_WARNNING("File copy error (size : %lu : %lu)",
					fw_size, read_size);
			errnum = ERROR_PARSING_FILE_OPEN_FAIL;
		} else {
			head = parse_uchar2int_arr(fw_data, fw_size,
					&fw_header_tmp, &errnum, &all);
		}
		kfree(fw_data);
	}

	filp_close(src, NULL);
	set_fs(oldfs);
#ifdef SUPPORT_TEST_MODE
	view_error_msg(errnum);
#endif
	SOLOMON_TIME("E92");
	if (head != NULL) {
		solomon_reset();

		do {
			errnum = fw_solomon_init(dev);

			if (errnum >= 0)
				break;

			mdelay(10);
		} while ((retry--) > 1);

		if (errnum >= 0)
			errnum = seeprom_firmware_update(dev, head, all);
#ifdef SUPPORT_TEST_MODE
		view_error_msg(errnum);
#endif
		fw_free(head, 1);
	}
	SOLOMON_TIME("E91");
	return errnum;
}

int solomon_get_version_boot(struct solomon_device *dev)
{
	return seeprom_get_version_boot(dev);
}

u8 *solomon_get_version(struct solomon_device *dev, u8 *ver_buff)
{
	memcpy(ver_buff, (u8 *)&(dev->fw_version),
			sizeof(struct solomon_version));
	return ver_buff;
}

int solomon_firmware_pre_boot_up_check(struct solomon_device *dev)
{
	int err = 0;
	int retry = FW_MAX_RETRY_COUNT;

	if ((dev->boot_flag & BOOT_STATUS_ERR_CPUCFG_ALL) > 0) {
		SOLOMON_WARNNING("Solomon DS16 CPU CFG error!! update");
		goto update;
	}

	if ((dev->boot_flag & BOOT_STATUS_ERR_SYS_CFG_FAIL) > 0) {
		SOLOMON_WARNNING("Solomon DS16 SYS CFG error!!update");
		goto update;
	}

	if (dev->checksum_flag > 0) {
		SOLOMON_WARNNING("Checksum(0x%04x) fail!! update",
				dev->checksum_flag);
		dev->checksum_flag = 0;
		goto update;
	}
#if defined(SUPPORT_BOOTUP_FORCE_FW_UPGRADE_BINFILE)
	if(found_force_bin_file == 1)
		goto update;
#endif
	/* version check */
	if (solomon_firmware_version_check(dev, &fw_header) < 0)
		goto update;

	return -1;
update:
//	solomon_reset(); //should be BIOS mode here

	do {
		err = fw_solomon_init(dev);

		if (err >= 0)
			break;

		mdelay(10);
	} while ((retry--) > 1);

	if (err >= 0)
		err = seeprom_firmware_pre_boot_up_check(dev, &fw_header);

	view_error_msg(err);
	return 1;
}

int solomon_firmware_pre_boot_up_check_bin(struct solomon_device *dev,
		const u8 *data, size_t data_size)
{
	int err = 0;

	SOLOMON_TIME("S01");
	err = solomon_make_header_use_bin(dev, &fw_header, data, data_size);

	err = solomon_firmware_pre_boot_up_check(dev);
	m_fw_head_bin_flag = 1;
	solomon_free_header(dev);
	SOLOMON_TIME("E01");
	return err;
}

int solomon_firmware_pre_boot_up_check_head(struct solomon_device *dev)
{
	int err = 0;

	SOLOMON_TIME("S01");
	err = solomon_make_header_use_head(dev, &fw_header);

	err = solomon_firmware_pre_boot_up_check(dev);
	m_fw_head_bin_flag = 0;

	solomon_free_header(dev);
	SOLOMON_TIME("E01");
	return err;
}

/* for furture, It is not used yet. */
int solomon_firmware_check(struct solomon_device *dev, char *filename)
{
	int errnum = 0;
	struct file *src;
	mm_segment_t oldfs;
	size_t fw_size = 0, read_size = 0;
	u8 *fw_data = NULL;

	if (filename == NULL) {
		SOLOMON_WARNNING("Bin filename is null!!");
		goto header;
	}

	oldfs = get_fs();
	set_fs(KERNEL_DS);

	src = filp_open(filename, O_RDONLY, S_IRUSR|S_IRGRP|S_IROTH);
	if (IS_ERR(src)) {
		SOLOMON_WARNNING("[%s] file open error!!", filename);
		set_fs(oldfs);
		goto header;
	}

	fw_size = src->f_path.dentry->d_inode->i_size;
	if (fw_size > 0) {
		fw_data = kzalloc(fw_size, GFP_KERNEL);
		read_size = vfs_read(src, (char __user *)fw_data, fw_size,
				&src->f_pos);

		SOLOMON_WARNNING("file path %s, size %lu Bytes\n", filename,
				fw_size);
		if (read_size != fw_size) {
			SOLOMON_WARNNING("File copy error!!!(size : %lu : %lu)",
					fw_size, read_size);
			errnum = ERROR_PARSING_FILE_OPEN_FAIL;
			set_fs(oldfs);
			goto out;
		}
	}

	filp_close(src, NULL);
	set_fs(oldfs);

	goto out;
header:
	SOLOMON_WARNNING("Header file update");
	errnum = solomon_firmware_pre_boot_up_check_head(dev);
out:
	return errnum;
}
