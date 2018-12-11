/*
 * Copyright 2017 Solomon Systech Ltd. All rights reserved.
 *
 * SSL SSD20xx MX25V100 device driver
 *
 * Date: 2017.04.19
 */
#ifndef __MX25V100_H
#define __MX25V100_H

/* EFLASH FLAG */
#define FW_EFLASH_FLAG_CPU_ONLY				0x0001
#define FW_EFLASH_FLAG_MP_FPM				0x0002
#define FW_EFLASH_FLAG_MP_FDM				0x0004
#define FW_EFLASH_FLAG_HW_CAL				0x0008
#define FW_EFLASH_FLAG_FPM					0x0010
#define FW_EFLASH_FLAG_FDM					0x0020
#define FW_EFLASH_FLAG_TMC_REG				0x0040
#define FW_EFLASH_FLAG_SW_CAL				0x0080
#define FW_EFLASH_FLAG_CPU_CFG				0x0100
#define FW_EFLASH_FLAG_SYS_CFG				0x0200
#define FW_EFLASH_FLAG_INFO					0x0400
#define FW_EFLASH_FLAG_ALL	(FW_EFLASH_FLAG_CPU_ONLY|FW_EFLASH_FLAG_MP_FPM \
		|FW_EFLASH_FLAG_MP_FDM|FW_EFLASH_FLAG_HW_CAL|FW_EFLASH_FLAG_FPM \
		|FW_EFLASH_FLAG_FDM|FW_EFLASH_FLAG_TMC_REG|FW_EFLASH_FLAG_SW_CAL \
		|FW_EFLASH_FLAG_CPU_CFG|FW_EFLASH_FLAG_SYS_CFG|FW_EFLASH_FLAG_INFO)

int seeprom_firmware_update(struct solomon_device *dev,
		struct solomon_fw_group *fw_group, int all);
int seeprom_firmware_pre_boot_up_check(struct solomon_device *dev,
		struct solomon_fw_group_header *fw_header);
int seeprom_get_version_boot(struct solomon_device *dev);
#endif
