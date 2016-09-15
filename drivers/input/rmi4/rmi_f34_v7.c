/*
 * Copyright (c) 2016, Zodiac Inflight Innovations
 * Copyright (c) 2007-2016, Synaptics Incorporated
 * Copyright (C) 2012 Alexandra Chin <alexandra.chin@tw.synaptics.com>
 * Copyright (C) 2012 Scott Lin <scott.lin@tw.synaptics.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/rmi.h>
#include <linux/firmware.h>
#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/slab.h>

#include "rmi_driver.h"

#define V7_FLASH_STATUS_OFFSET 0
#define V7_PARTITION_ID_OFFSET 1
#define V7_BLOCK_NUMBER_OFFSET 2
#define V7_TRANSFER_LENGTH_OFFSET 3
#define V7_COMMAND_OFFSET 4
#define V7_PAYLOAD_OFFSET 5
#define BOOTLOADER_ID_OFFSET 1

#define V7_PARTITION_SUPPORT_BYTES 4

#define SLEEP_MODE_NORMAL (0x00)

#define IMAGE_HEADER_VERSION_10 0x10

#define MAX_IMAGE_NAME_LEN 256
#define SYNAPTICS_RMI4_PRODUCT_ID_SIZE 10
#define PRODUCT_ID_SIZE 10

#define MASK_8BIT 0xFF
#define MASK_5BIT 0x1F

#define ENABLE_WAIT_MS (1 * 1000)
#define WRITE_WAIT_MS (3 * 1000)
#define ERASE_WAIT_MS (5 * 1000)

#define MIN_SLEEP_TIME_US 50
#define MAX_SLEEP_TIME_US 100

#define FORCE_UPDATE false
#define DO_LOCKDOWN false

enum bl_version {
	V5 = 5,
	V6 = 6,
	BL_V5 = 5,
	BL_V6 = 6,
	BL_V7 = 7,
};

enum v7_flash_command2 {
	CMD_V7_IDLE = 0x00,
	CMD_V7_ENTER_BL,
	CMD_V7_READ,
	CMD_V7_WRITE,
	CMD_V7_ERASE,
	CMD_V7_ERASE_AP,
	CMD_V7_SENSOR_ID,
};

enum v7_flash_command {
	v7_CMD_IDLE = 0,
	v7_CMD_WRITE_FW,
	v7_CMD_WRITE_CONFIG,
	v7_CMD_WRITE_LOCKDOWN,
	v7_CMD_WRITE_GUEST_CODE,
	v7_CMD_READ_CONFIG,
	v7_CMD_ERASE_ALL,
	v7_CMD_ERASE_UI_FIRMWARE,
	v7_CMD_ERASE_UI_CONFIG,
	v7_CMD_ERASE_BL_CONFIG,
	v7_CMD_ERASE_DISP_CONFIG,
	v7_CMD_ERASE_FLASH_CONFIG,
	v7_CMD_ERASE_GUEST_CODE,
	v7_CMD_ENABLE_FLASH_PROG,
};

enum flash_area {
	NONE,
	UI_FIRMWARE,
	CONFIG_AREA,
};

enum update_mode {
	UPDATE_MODE_NORMAL = 1,
	UPDATE_MODE_FORCE = 2,
	UPDATE_MODE_LOCKDOWN = 8,
};

enum v7_config_area {
	v7_UI_CONFIG_AREA = 0,
	v7_PM_CONFIG_AREA,
	v7_BL_CONFIG_AREA,
	v7_DP_CONFIG_AREA,
	v7_FLASH_CONFIG_AREA,
};

enum v7_partition_id {
	BOOTLOADER_PARTITION = 0x01,
	DEVICE_CONFIG_PARTITION,
	FLASH_CONFIG_PARTITION,
	MANUFACTURING_BLOCK_PARTITION,
	GUEST_SERIALIZATION_PARTITION,
	GLOBAL_PARAMETERS_PARTITION,
	CORE_CODE_PARTITION,
	CORE_CONFIG_PARTITION,
	GUEST_CODE_PARTITION,
	DISPLAY_CONFIG_PARTITION,
};

struct f01_device_control {
	union {
		struct {
			unsigned char sleep_mode:2;
			unsigned char nosleep:1;
			unsigned char reserved:2;
			unsigned char charger_connected:1;
			unsigned char report_rate:1;
			unsigned char configured:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_v7_query_0 {
	union {
		struct {
			unsigned char subpacket_1_size:3;
			unsigned char has_config_id:1;
			unsigned char f34_query0_b4:1;
			unsigned char has_thqa:1;
			unsigned char f34_query0_b6__7:2;
		} __packed;
		unsigned char data[1];
	};
};

struct synaptics_rmi4_f34_query_01 {
	union {
		struct {
			unsigned char reg_map:1;
			unsigned char unlocked:1;
			unsigned char has_config_id:1;
			unsigned char has_perm_config:1;
			unsigned char has_bl_config:1;
			unsigned char has_disp_config:1;
			unsigned char has_ctrl1:1;
			unsigned char has_flash_query4:1;
		} __packed;
		unsigned char data[1];
	};
};

struct f34_v7_query_1_7 {
	union {
		struct {
			/* query 1 */
			unsigned char bl_minor_revision;
			unsigned char bl_major_revision;

			/* query 2 */
			unsigned char bl_fw_id_7_0;
			unsigned char bl_fw_id_15_8;
			unsigned char bl_fw_id_23_16;
			unsigned char bl_fw_id_31_24;

			/* query 3 */
			unsigned char minimum_write_size;
			unsigned char block_size_7_0;
			unsigned char block_size_15_8;
			unsigned char flash_page_size_7_0;
			unsigned char flash_page_size_15_8;

			/* query 4 */
			unsigned char adjustable_partition_area_size_7_0;
			unsigned char adjustable_partition_area_size_15_8;

			/* query 5 */
			unsigned char flash_config_length_7_0;
			unsigned char flash_config_length_15_8;

			/* query 6 */
			unsigned char payload_length_7_0;
			unsigned char payload_length_15_8;

			/* query 7 */
			unsigned char f34_query7_b0:1;
			unsigned char has_bootloader:1;
			unsigned char has_device_config:1;
			unsigned char has_flash_config:1;
			unsigned char has_manufacturing_block:1;
			unsigned char has_guest_serialization:1;
			unsigned char has_global_parameters:1;
			unsigned char has_core_code:1;
			unsigned char has_core_config:1;
			unsigned char has_guest_code:1;
			unsigned char has_display_config:1;
			unsigned char f34_query7_b11__15:5;
			unsigned char f34_query7_b16__23;
			unsigned char f34_query7_b24__31;
		} __packed;
		unsigned char data[21];
	};
};

struct f34_v7_data_1_5 {
	union {
		struct {
			unsigned char partition_id:5;
			unsigned char f34_data1_b5__7:3;
			unsigned char block_offset_7_0;
			unsigned char block_offset_15_8;
			unsigned char transfer_length_7_0;
			unsigned char transfer_length_15_8;
			unsigned char command;
			unsigned char payload_0;
			unsigned char payload_1;
		} __packed;
		unsigned char data[8];
	};
};

struct block_data {
	const unsigned char *data;
	int size;
};

struct partition_table {
	unsigned char partition_id:5;
	unsigned char byte_0_reserved:3;
	unsigned char byte_1_reserved;
	unsigned char partition_length_7_0;
	unsigned char partition_length_15_8;
	unsigned char start_physical_address_7_0;
	unsigned char start_physical_address_15_8;
	unsigned char partition_properties_7_0;
	unsigned char partition_properties_15_8;
} __packed;

struct physical_address {
	unsigned short ui_firmware;
	unsigned short ui_config;
	unsigned short dp_config;
	unsigned short guest_code;
};

struct container_descriptor {
	unsigned char content_checksum[4];
	unsigned char container_id[2];
	unsigned char minor_version;
	unsigned char major_version;
	unsigned char reserved_08;
	unsigned char reserved_09;
	unsigned char reserved_0a;
	unsigned char reserved_0b;
	unsigned char container_option_flags[4];
	unsigned char content_options_length[4];
	unsigned char content_options_address[4];
	unsigned char content_length[4];
	unsigned char content_address[4];
};

enum container_id {
	TOP_LEVEL_CONTAINER = 0,
	UI_CONTAINER,
	UI_CONFIG_CONTAINER,
	BL_CONTAINER,
	BL_IMAGE_CONTAINER,
	BL_CONFIG_CONTAINER,
	BL_LOCKDOWN_INFO_CONTAINER,
	PERMANENT_CONFIG_CONTAINER,
	GUEST_CODE_CONTAINER,
	BL_PROTOCOL_DESCRIPTOR_CONTAINER,
	UI_PROTOCOL_DESCRIPTOR_CONTAINER,
	RMI_SELF_DISCOVERY_CONTAINER,
	RMI_PAGE_CONTENT_CONTAINER,
	GENERAL_INFORMATION_CONTAINER,
	DEVICE_CONFIG_CONTAINER,
	FLASH_CONFIG_CONTAINER,
	GUEST_SERIALIZATION_CONTAINER,
	GLOBAL_PARAMETERS_CONTAINER,
	CORE_CODE_CONTAINER,
	CORE_CONFIG_CONTAINER,
	DISPLAY_CONFIG_CONTAINER,
};

struct block_count {
	unsigned short ui_firmware;
	unsigned short ui_config;
	unsigned short dp_config;
	unsigned short fl_config;
	unsigned short pm_config;
	unsigned short bl_config;
	unsigned short lockdown;
	unsigned short guest_code;
};

struct image_header_10 {
	unsigned char checksum[4];
	unsigned char reserved_04;
	unsigned char reserved_05;
	unsigned char minor_header_version;
	unsigned char major_header_version;
	unsigned char reserved_08;
	unsigned char reserved_09;
	unsigned char reserved_0a;
	unsigned char reserved_0b;
	unsigned char top_level_container_start_addr[4];
};

struct image_metadata {
	bool contains_firmware_id;
	bool contains_bootloader;
	bool contains_disp_config;
	bool contains_guest_code;
	bool contains_flash_config;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int bootloader_size;
	unsigned int disp_config_offset;
	unsigned char bl_version;
	unsigned char product_id[PRODUCT_ID_SIZE + 1];
	unsigned char cstmr_product_id[PRODUCT_ID_SIZE + 1];
	struct block_data bootloader;
	struct block_data ui_firmware;
	struct block_data ui_config;
	struct block_data dp_config;
	struct block_data fl_config;
	struct block_data bl_config;
	struct block_data guest_code;
	struct block_data lockdown;
	struct block_count blkcount;
	struct physical_address phyaddr;
};

struct register_offset {
	unsigned char properties;
	unsigned char properties_2;
	unsigned char block_size;
	unsigned char block_count;
	unsigned char gc_block_count;
	unsigned char flash_status;
	unsigned char partition_id;
	unsigned char block_number;
	unsigned char transfer_length;
	unsigned char flash_cmd;
	unsigned char payload;
};

struct img_file_content {
	unsigned char *fw_image;
	unsigned int image_size;
	unsigned char *image_name;
	unsigned char imageFileVersion;
	struct block_data uiFirmware;
	struct block_data uiConfig;
	struct block_data guestCode;
	struct block_data lockdown;
	struct block_data permanent;
	struct block_data bootloaderInfo;
	unsigned char blMajorVersion;
	unsigned char blMinorVersion;
	unsigned char *configId;	/* len 0x4 */
	unsigned char *firmwareId;	/* len 0x4 */
	unsigned char *packageId;	/* len 0x4 */
	unsigned char *dsFirmwareInfo;	/* len 0x10 */
};

struct f34_data {
	struct rmi_function *fn;

	enum bl_version bl_version;
	bool initialized;
	bool has_perm_config;
	bool has_bl_config;
	bool has_disp_config;
	bool has_guest_code;
	bool force_update;
	unsigned char *read_config_buf;
	unsigned char command;
	unsigned char bootloader_id[4];
	unsigned char bootloader_id_ic[2];
	unsigned char flash_status;
	unsigned char productinfo1;
	unsigned char productinfo2;
	unsigned char properties_off;
	unsigned char blk_size_off;
	unsigned char blk_count_off;
	unsigned char blk_data_off;
	unsigned char properties2_off;
	unsigned char guest_blk_count_off;
	unsigned char flash_cmd_off;
	unsigned char flash_status_off;
	unsigned short block_size;
	unsigned short fw_block_count;
	unsigned short config_block_count;
	unsigned short perm_config_block_count;
	unsigned short bl_config_block_count;
	unsigned short disp_config_block_count;
	unsigned short guest_code_block_count;
	unsigned short config_size;
	unsigned short config_area;
	char product_id[SYNAPTICS_RMI4_PRODUCT_ID_SIZE + 1];

	struct synaptics_rmi4_f34_query_01 flash_properties;
	struct workqueue_struct *fwu_workqueue;
	struct delayed_work fwu_work;
	struct img_file_content img;
	bool polling_mode;

	unsigned short flash_config_length;
	unsigned short payload_length;
	struct register_offset off;
	unsigned char partitions;
	unsigned short partition_table_bytes;
	unsigned short read_config_buf_size;
	struct block_count blkcount;
	struct physical_address phyaddr;
	struct image_metadata v7_img;
	bool new_partition_table;
	const unsigned char *config_data;
	const unsigned char *image;
	bool in_bl_mode;
	int f34_status;
	const char *config_id;
};

struct synaptics_fn {
	struct synaptics_rmi4_fwu_handle *fwu;
};

static unsigned int le_to_uint(const unsigned char *ptr)
{
	return (unsigned int)ptr[0] +
			(unsigned int)ptr[1] * 0x100 +
			(unsigned int)ptr[2] * 0x10000 +
			(unsigned int)ptr[3] * 0x1000000;
}

static int fwu_wait_for_idle(struct rmi_function *fn, int timeout_ms);

static int fwu_write_f34_command_single_transaction_v7(struct rmi_function *fn, unsigned char cmd)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);
	int ret;
	unsigned char base;
	struct f34_v7_data_1_5 data_1_5;

	base = fn->fd.data_base_addr;

	memset(data_1_5.data, 0x00, sizeof(data_1_5.data));

	switch (cmd) {
	case v7_CMD_ERASE_ALL:
		data_1_5.partition_id = CORE_CODE_PARTITION;
		data_1_5.command = CMD_V7_ERASE_AP;
		break;
	case v7_CMD_ERASE_UI_FIRMWARE:
		data_1_5.partition_id = CORE_CODE_PARTITION;
		data_1_5.command = CMD_V7_ERASE;
		break;
	case v7_CMD_ERASE_BL_CONFIG:
		data_1_5.partition_id = GLOBAL_PARAMETERS_PARTITION;
		data_1_5.command = CMD_V7_ERASE;
		break;
	case v7_CMD_ERASE_UI_CONFIG:
		data_1_5.partition_id = CORE_CONFIG_PARTITION;
		data_1_5.command = CMD_V7_ERASE;
		break;
	case v7_CMD_ERASE_DISP_CONFIG:
		data_1_5.partition_id = DISPLAY_CONFIG_PARTITION;
		data_1_5.command = CMD_V7_ERASE;
		break;
	case v7_CMD_ERASE_FLASH_CONFIG:
		data_1_5.partition_id = FLASH_CONFIG_PARTITION;
		data_1_5.command = CMD_V7_ERASE;
		break;
	case v7_CMD_ERASE_GUEST_CODE:
		data_1_5.partition_id = GUEST_CODE_PARTITION;
		data_1_5.command = CMD_V7_ERASE;
		break;
	case v7_CMD_ENABLE_FLASH_PROG:
		data_1_5.partition_id = BOOTLOADER_PARTITION;
		data_1_5.command = CMD_V7_ENTER_BL;
		break;
	};

	data_1_5.payload_0 = fwu->bootloader_id[0];
	data_1_5.payload_1 = fwu->bootloader_id[1];

	ret = rmi_write_block(fn->rmi_dev,
			base + fwu->off.partition_id,
			data_1_5.data,
			sizeof(data_1_5.data));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to write single transaction command\n",
			__func__);
		return ret;
	}

	return 0;
}

static int fwu_write_f34_command_v7(struct rmi_function *fn, unsigned char cmd)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);
	int ret;
	unsigned char base;
	unsigned char command;

	base = fn->fd.data_base_addr;

	switch (cmd) {
	case v7_CMD_WRITE_FW:
	case v7_CMD_WRITE_CONFIG:
	case v7_CMD_WRITE_GUEST_CODE:
		command = CMD_V7_WRITE;
		break;
	case v7_CMD_READ_CONFIG:
		command = CMD_V7_READ;
		break;
	case v7_CMD_ERASE_ALL:
		command = CMD_V7_ERASE_AP;
		break;
	case v7_CMD_ERASE_UI_FIRMWARE:
	case v7_CMD_ERASE_BL_CONFIG:
	case v7_CMD_ERASE_UI_CONFIG:
	case v7_CMD_ERASE_DISP_CONFIG:
	case v7_CMD_ERASE_FLASH_CONFIG:
	case v7_CMD_ERASE_GUEST_CODE:
		command = CMD_V7_ERASE;
		break;
	case v7_CMD_ENABLE_FLASH_PROG:
		command = CMD_V7_ENTER_BL;
		break;
	default:
		dev_err(&fn->dev, "%s: Invalid command 0x%02x\n",
			__func__, cmd);
		return -EINVAL;
	};

	fwu->command = command;

	switch (cmd) {
	case v7_CMD_ERASE_ALL:
	case v7_CMD_ERASE_UI_FIRMWARE:
	case v7_CMD_ERASE_BL_CONFIG:
	case v7_CMD_ERASE_UI_CONFIG:
	case v7_CMD_ERASE_DISP_CONFIG:
	case v7_CMD_ERASE_FLASH_CONFIG:
	case v7_CMD_ERASE_GUEST_CODE:
	case v7_CMD_ENABLE_FLASH_PROG:
		ret = fwu_write_f34_command_single_transaction_v7(fn, cmd);
		if (ret < 0)
			return ret;
		else
			return 0;
	default:
		break;
	};

	rmi_dbg(RMI_DEBUG_FLASH, &fn->dev, "%s: writing cmd %02X\n",
		__func__, command);

	ret = rmi_write_block(fn->rmi_dev,
			base + fwu->off.flash_cmd,
			&command,
			sizeof(command));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to write flash command\n",
			__func__);
		return ret;
	}

	return 0;
}

static int fwu_write_f34_v7_partition_id(struct rmi_function *fn, unsigned char cmd)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	int ret;
	unsigned char base;
	unsigned char partition;

	base = fn->fd.data_base_addr;

	switch (cmd) {
	case v7_CMD_WRITE_FW:
		partition = CORE_CODE_PARTITION;
		break;
	case v7_CMD_WRITE_CONFIG:
	case v7_CMD_READ_CONFIG:
		if (fwu->config_area == v7_UI_CONFIG_AREA)
			partition = CORE_CONFIG_PARTITION;
		else if (fwu->config_area == v7_DP_CONFIG_AREA)
			partition = DISPLAY_CONFIG_PARTITION;
		else if (fwu->config_area == v7_PM_CONFIG_AREA)
			partition = GUEST_SERIALIZATION_PARTITION;
		else if (fwu->config_area == v7_BL_CONFIG_AREA)
			partition = GLOBAL_PARAMETERS_PARTITION;
		else if (fwu->config_area == v7_FLASH_CONFIG_AREA)
			partition = FLASH_CONFIG_PARTITION;
		break;
	case v7_CMD_WRITE_GUEST_CODE:
		partition = GUEST_CODE_PARTITION;
		break;
	case v7_CMD_ERASE_ALL:
		partition = CORE_CODE_PARTITION;
		break;
	case v7_CMD_ERASE_BL_CONFIG:
		partition = GLOBAL_PARAMETERS_PARTITION;
		break;
	case v7_CMD_ERASE_UI_CONFIG:
		partition = CORE_CONFIG_PARTITION;
		break;
	case v7_CMD_ERASE_DISP_CONFIG:
		partition = DISPLAY_CONFIG_PARTITION;
		break;
	case v7_CMD_ERASE_FLASH_CONFIG:
		partition = FLASH_CONFIG_PARTITION;
		break;
	case v7_CMD_ERASE_GUEST_CODE:
		partition = GUEST_CODE_PARTITION;
		break;
	case v7_CMD_ENABLE_FLASH_PROG:
		partition = BOOTLOADER_PARTITION;
		break;
	default:
		dev_err(&fn->dev, "%s: Invalid command 0x%02x\n",
			__func__, cmd);
		return -EINVAL;
	};

	ret = rmi_write_block(fn->rmi_dev,
			base + fwu->off.partition_id,
			&partition,
			sizeof(partition));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to write partition ID\n",
			__func__);
		return ret;
	}

	return 0;
}

static int fwu_write_f34_partition_id(struct rmi_function *fn, unsigned char cmd)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);
	int ret;

	if (fwu->bl_version == BL_V7)
		ret = fwu_write_f34_v7_partition_id(fn, cmd);
	else
		ret = 0;

	return ret;
}

static int fwu_read_f34_v7_partition_table(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	int ret;
	unsigned char base;
	unsigned char length[2];
	unsigned short block_number = 0;

	base = fn->fd.data_base_addr;

	fwu->config_area = v7_FLASH_CONFIG_AREA;

	ret = fwu_write_f34_partition_id(fn, v7_CMD_READ_CONFIG);
	if (ret < 0)
		return ret;

	ret = rmi_write_block(fn->rmi_dev,
			base + fwu->off.block_number,
			(unsigned char *)&block_number,
			sizeof(block_number));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to write block number\n",
			__func__);
		return ret;
	}

	length[0] = (unsigned char)(fwu->flash_config_length & MASK_8BIT);
	length[1] = (unsigned char)(fwu->flash_config_length >> 8);

	ret = rmi_write_block(fn->rmi_dev,
			base + fwu->off.transfer_length,
			length,
			sizeof(length));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to write transfer length\n",
			__func__);
		return ret;
	}

	ret = fwu_write_f34_command_v7(fn, v7_CMD_READ_CONFIG);
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to write command\n",
			__func__);
		return ret;
	}

	fwu->polling_mode = true;
	ret = fwu_wait_for_idle(fn, WRITE_WAIT_MS);
	fwu->polling_mode = false;
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to wait for idle status\n",
			__func__);
		return ret;
	}

	ret = rmi_read_block(fn->rmi_dev,
			base + fwu->off.payload,
			fwu->read_config_buf,
			fwu->partition_table_bytes);
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to read block data\n",
			__func__);
		return ret;
	}

	return 0;
}

static void fwu_parse_partition_table(struct rmi_function *fn,
		const unsigned char *partition_table,
		struct block_count *blkcount, struct physical_address *phyaddr)
{
	unsigned char ii;
	unsigned char index;
	unsigned short partition_length;
	unsigned short physical_address;
	struct partition_table *ptable;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	for (ii = 0; ii < fwu->partitions; ii++) {
		index = ii * 8 + 2;
		ptable = (struct partition_table *)&partition_table[index];
		partition_length = ptable->partition_length_15_8 << 8 |
				ptable->partition_length_7_0;
		physical_address = ptable->start_physical_address_15_8 << 8 |
				ptable->start_physical_address_7_0;
		rmi_dbg(RMI_DEBUG_FLASH, &fn->dev,
			"%s: Partition entry %d: %*ph\n",
			__func__, ii, sizeof(struct partition_table), ptable);
		switch (ptable->partition_id) {
		case CORE_CODE_PARTITION:
			blkcount->ui_firmware = partition_length;
			phyaddr->ui_firmware = physical_address;
			rmi_dbg(RMI_DEBUG_FLASH, &fn->dev,
				"%s: Core code block count: %d\n",
				__func__, blkcount->ui_firmware);
			break;
		case CORE_CONFIG_PARTITION:
			blkcount->ui_config = partition_length;
			phyaddr->ui_config = physical_address;
			rmi_dbg(RMI_DEBUG_FLASH, &fn->dev,
				"%s: Core config block count: %d\n",
				__func__, blkcount->ui_config);
			break;
		case DISPLAY_CONFIG_PARTITION:
			blkcount->dp_config = partition_length;
			phyaddr->dp_config = physical_address;
			rmi_dbg(RMI_DEBUG_FLASH, &fn->dev,
				"%s: Display config block count: %d\n",
				__func__, blkcount->dp_config);
			break;
		case FLASH_CONFIG_PARTITION:
			blkcount->fl_config = partition_length;
			rmi_dbg(RMI_DEBUG_FLASH, &fn->dev,
				"%s: Flash config block count: %d\n",
				__func__, blkcount->fl_config);
			break;
		case GUEST_CODE_PARTITION:
			blkcount->guest_code = partition_length;
			phyaddr->guest_code = physical_address;
			rmi_dbg(RMI_DEBUG_FLASH, &fn->dev,
				"%s: Guest code block count: %d\n",
				__func__, blkcount->guest_code);
			break;
		case GUEST_SERIALIZATION_PARTITION:
			blkcount->pm_config = partition_length;
			rmi_dbg(RMI_DEBUG_FLASH, &fn->dev,
				"%s: Guest serialization block count: %d\n",
				__func__, blkcount->pm_config);
			break;
		case GLOBAL_PARAMETERS_PARTITION:
			blkcount->bl_config = partition_length;
			rmi_dbg(RMI_DEBUG_FLASH, &fn->dev,
				"%s: Global parameters block count: %d\n",
				__func__, blkcount->bl_config);
			break;
		case DEVICE_CONFIG_PARTITION:
			blkcount->lockdown = partition_length;
			rmi_dbg(RMI_DEBUG_FLASH, &fn->dev,
				"%s: Device config block count: %d\n",
				__func__, blkcount->lockdown);
			break;
		};
	}

	return;
}

static int fwu_read_f34_queries_bl_version(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	int ret;
	unsigned char base;
	unsigned char offset;
	struct f34_v7_query_0 query_0;
	struct f34_v7_query_1_7 query_1_7;

	base = fn->fd.query_base_addr;

	ret = rmi_read_block(fn->rmi_dev,
			base,
			query_0.data,
			sizeof(query_0.data));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to read query 0\n", __func__);
		return ret;
	}

	offset = query_0.subpacket_1_size + 1;

	ret = rmi_read_block(fn->rmi_dev,
			base + offset,
			query_1_7.data,
			sizeof(query_1_7.data));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to read queries 1 to 7\n",
			__func__);
		return ret;
	}

	fwu->bootloader_id[0] = query_1_7.bl_minor_revision;
	fwu->bootloader_id[1] = query_1_7.bl_major_revision;

	dev_info(&fn->dev, "%s: Bootloader V%d.%d\n", __func__,
		 fwu->bootloader_id[1],fwu->bootloader_id[0]);

	return 0;
}

static int fwu_read_f34_queries_v7(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	int ret;
	unsigned char ii;
	unsigned char base;
	unsigned char index;
	unsigned char offset;
	unsigned char *ptable;
	struct f34_v7_query_0 query_0;
	struct f34_v7_query_1_7 query_1_7;

	base = fn->fd.query_base_addr;

	ret = rmi_read_block(fn->rmi_dev,
			base,
			query_0.data,
			sizeof(query_0.data));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to read query 0\n", __func__);
		return ret;
	}

	offset = query_0.subpacket_1_size + 1;

	ret = rmi_read_block(fn->rmi_dev,
			base + offset,
			query_1_7.data,
			sizeof(query_1_7.data));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to read queries 1 to 7\n",
			__func__);
		return ret;
	}

	fwu->bootloader_id_ic[0] = fwu->bootloader_id[0] = query_1_7.bl_minor_revision;
	fwu->bootloader_id_ic[1] = fwu->bootloader_id[1] = query_1_7.bl_major_revision;

	fwu->block_size = query_1_7.block_size_15_8 << 8 |
			query_1_7.block_size_7_0;

	dev_info(&fn->dev, "%s: fwu->block_size = %d, [%d,%d]\n",
		 __func__, fwu->block_size, fwu->bootloader_id[0], fwu->bootloader_id[1]);

	fwu->flash_config_length = query_1_7.flash_config_length_15_8 << 8 |
			query_1_7.flash_config_length_7_0;

	fwu->payload_length = query_1_7.payload_length_15_8 << 8 |
			query_1_7.payload_length_7_0;

	fwu->off.flash_status = V7_FLASH_STATUS_OFFSET;
	fwu->off.partition_id = V7_PARTITION_ID_OFFSET;
	fwu->off.block_number = V7_BLOCK_NUMBER_OFFSET;
	fwu->off.transfer_length = V7_TRANSFER_LENGTH_OFFSET;
	fwu->off.flash_cmd = V7_COMMAND_OFFSET;
	fwu->off.payload = V7_PAYLOAD_OFFSET;

	fwu->flash_properties.has_disp_config = query_1_7.has_display_config;
	fwu->flash_properties.has_perm_config = query_1_7.has_guest_serialization;
	fwu->flash_properties.has_bl_config = query_1_7.has_global_parameters;

	fwu->has_guest_code = query_1_7.has_guest_code;
	fwu->flash_properties.has_config_id = query_0.has_config_id;

	index = sizeof(query_1_7.data) - V7_PARTITION_SUPPORT_BYTES;

	fwu->partitions = 0;
	for (offset = 0; offset < V7_PARTITION_SUPPORT_BYTES; offset++) {
		for (ii = 0; ii < 8; ii++) {
			if (query_1_7.data[index + offset] & (1 << ii))
				fwu->partitions++;
		}

		rmi_dbg(RMI_DEBUG_FLASH, &fn->dev,
			"%s: Supported partitions: 0x%02x\n",
			__func__, query_1_7.data[index + offset]);
	}

	fwu->partition_table_bytes = fwu->partitions * 8 + 2;

	fwu->read_config_buf = devm_kzalloc(&fn->dev,
					    fwu->partition_table_bytes,
					    GFP_KERNEL);
	if (!fwu->read_config_buf) {
		dev_err(&fn->dev,
			"%s: Failed to alloc mem for fwu->read_config_buf\n",
			__func__);
		fwu->read_config_buf_size = 0;
		return -ENOMEM;
	}
	fwu->read_config_buf_size = fwu->partition_table_bytes;
	ptable = fwu->read_config_buf;

	ret = fwu_read_f34_v7_partition_table(fn);
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to read partition table\n",
			__func__);
		return ret;
	}

	fwu_parse_partition_table(fn, ptable, &fwu->blkcount, &fwu->phyaddr);

	return 0;
}

static int fwu_read_flash_status_v7(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);
	unsigned char status;
	unsigned char command;
	int ret;

	ret = rmi_read_block(fn->rmi_dev,
			fn->fd.data_base_addr + fwu->off.flash_status,
			&status,
			sizeof(status));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to read flash status\n",
			__func__);
		return ret;
	}

	fwu->in_bl_mode = status >> 7;

	fwu->flash_status = status & MASK_5BIT;

	if (fwu->flash_status != 0x00) {
		dev_err(&fn->dev, "%s: Flash status = %d, command = 0x%02x\n",
			__func__, fwu->flash_status, fwu->command);
	}

	ret = rmi_read_block(fn->rmi_dev,
			fn->fd.data_base_addr + fwu->off.flash_cmd,
			&command,
			sizeof(command));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to read flash command\n",
			__func__);
		return ret;
	}

	fwu->command = command;

	return 0;
}

static int fwu_wait_for_idle(struct rmi_function *fn, int timeout_ms)
{
	int count = 0;
	int timeout_count = ((timeout_ms * 1000) / MAX_SLEEP_TIME_US) + 1;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	rmi_dbg(RMI_DEBUG_FLASH, &fn->dev,
		"%s: fwu->polling_mode = %d, timeout_count = %d\n",
		__func__, fwu->polling_mode, timeout_count );
#if 1
	do {
		usleep_range(MIN_SLEEP_TIME_US, MAX_SLEEP_TIME_US);

		count++;
		if (fwu->polling_mode || (count == timeout_count))
			fwu_read_flash_status_v7(fn);

		if ((fwu->command == v7_CMD_IDLE) && (fwu->flash_status == 0x00))
			return 0;
	} while (count < timeout_count);

#else
	do {
		if (fwu->polling_mode || count == timeout_count)
			fwu_read_f34_flash_status(fn);
			fwu_read_flash_status_v7(fn);
		if ((fwu->command == 0x00) && (fwu->flash_status == 0x00)) {
			if (count == timeout_count)
				fwu->polling_mode = true;
			return 0;
		}
		usleep_range(MIN_SLEEP_TIME_US, MAX_SLEEP_TIME_US);
		count++;

	} while (count <= timeout_count);
#endif
	dev_err(&fn->dev, "%s: Timed out waiting for idle status\n", __func__);

	return -ETIMEDOUT;
}

static int fwu_check_ui_firmware_size(struct rmi_function *fn)
{
	unsigned short block_count;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	block_count = fwu->v7_img.ui_firmware.size / fwu->block_size;

	if (block_count != fwu->blkcount.ui_firmware) {
		dev_err(&fn->dev, "%s: UI firmware size mismatch: "
			"block_count=%d,fwu->blkcount.ui_firmware=%d\n",
			__func__, block_count, fwu->blkcount.ui_firmware);
		return -EINVAL;
	}

	return 0;
}

static int fwu_check_ui_configuration_size(struct rmi_function *fn)
{
	unsigned short block_count;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	block_count = fwu->v7_img.ui_config.size / fwu->block_size;

	if (block_count != fwu->blkcount.ui_config) {
		dev_err(&fn->dev, "%s: UI configuration size mismatch\n",
			__func__);
		return -EINVAL;
	}
	return 0;
}

static int fwu_check_dp_configuration_size(struct rmi_function *fn)
{
	unsigned short block_count;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);


	block_count = fwu->v7_img.dp_config.size / fwu->block_size;

	if (block_count != fwu->blkcount.dp_config) {
		dev_err(&fn->dev, "%s: Display configuration size mismatch\n",
			__func__);
		return -EINVAL;
	}

	return 0;
}

static int fwu_check_guest_code_size(struct rmi_function *fn)
{
	unsigned short block_count;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);


	block_count = fwu->v7_img.guest_code.size / fwu->block_size;
	if (block_count != fwu->blkcount.guest_code) {
		dev_err(&fn->dev, "%s: Guest code size mismatch\n",
			__func__);
		return -EINVAL;
	}

	return 0;
}

static int fwu_check_bl_configuration_size(struct rmi_function *fn)
{
	unsigned short block_count;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);


	block_count = fwu->v7_img.bl_config.size / fwu->block_size;

	if (block_count != fwu->blkcount.bl_config) {
		dev_err(&fn->dev, "%s: Bootloader config size mismatch\n",
			__func__);
		return -EINVAL;
	}

	return 0;
}

static int fwu_erase_configuration(struct rmi_function *fn)
{
	int ret;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);


	switch (fwu->config_area) {
	case v7_UI_CONFIG_AREA:
		ret = fwu_write_f34_command_v7(fn, v7_CMD_ERASE_UI_CONFIG);
		if (ret < 0)
			return ret;
		break;
	case v7_DP_CONFIG_AREA:
		ret = fwu_write_f34_command_v7(fn, v7_CMD_ERASE_DISP_CONFIG);
		if (ret < 0)
			return ret;
		break;
	case v7_BL_CONFIG_AREA:
		ret = fwu_write_f34_command_v7(fn, v7_CMD_ERASE_BL_CONFIG);
		if (ret < 0)
			return ret;
		break;
	}

	dev_info(&fn->dev, "%s: Erase config command written\n", __func__);

	ret = fwu_wait_for_idle(fn, ENABLE_WAIT_MS);
	if (ret < 0)
		return ret;

	rmi_dbg(RMI_DEBUG_FLASH, &fn->dev, "%s: Idle status detected\n",
		__func__);

	return ret;
}

static int fwu_erase_guest_code(struct rmi_function *fn)
{
	int ret;

	ret = fwu_write_f34_command_v7(fn, v7_CMD_ERASE_GUEST_CODE);
	if (ret < 0)
		return ret;

	dev_info(&fn->dev, "%s: Erase command written\n", __func__);

	ret = fwu_wait_for_idle(fn, ENABLE_WAIT_MS);
	if (ret < 0)
		return ret;

	rmi_dbg(RMI_DEBUG_FLASH, &fn->dev, "%s: Idle status detected\n",
		__func__);

	return 0;
}

static int fwu_erase_all(struct rmi_function *fn)
{
	int ret;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	ret = fwu_write_f34_command_v7(fn, v7_CMD_ERASE_UI_FIRMWARE);
	if (ret < 0)
		return ret;

	dev_info(&fn->dev, "%s: Erase command written\n", __func__);

	ret = fwu_wait_for_idle(fn, ENABLE_WAIT_MS);
	if (ret < 0)
		return ret;

	rmi_dbg(RMI_DEBUG_FLASH, &fn->dev, "%s: Idle status detected\n",
		__func__);

	fwu->config_area = v7_UI_CONFIG_AREA;
	ret = fwu_erase_configuration(fn);
	if (ret < 0)
		return ret;

	if (fwu->flash_properties.has_disp_config) {
		fwu->config_area = v7_DP_CONFIG_AREA;
		ret = fwu_erase_configuration(fn);
		if (ret < 0)
			return ret;
	}

	if (fwu->new_partition_table && fwu->has_guest_code) {
		ret = fwu_erase_guest_code(fn);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int fwu_read_f34_v7_blocks(struct rmi_function *fn,
		unsigned short block_cnt,
		unsigned char command)
{
	int ret;
	unsigned char base;
	unsigned char length[2];
	unsigned short transfer;
	unsigned short max_transfer;
	unsigned short remaining = block_cnt;
	unsigned short block_number = 0;
	unsigned short index = 0;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	base = fn->fd.data_base_addr;

	ret = fwu_write_f34_partition_id(fn, command);
	if (ret < 0)
		return ret;

	ret = rmi_write_block(fn->rmi_dev,
			base + fwu->off.block_number,
			(unsigned char *)&block_number,
			sizeof(block_number));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to write block number\n",
			__func__);
		return ret;
	}

	if (fwu->payload_length > (PAGE_SIZE / fwu->block_size))
		max_transfer = PAGE_SIZE / fwu->block_size;
	else
		max_transfer = fwu->payload_length;

	do {
		if (remaining / max_transfer)
			transfer = max_transfer;
		else
			transfer = remaining;

		length[0] = (unsigned char)(transfer & MASK_8BIT);
		length[1] = (unsigned char)(transfer >> 8);

		ret = rmi_write_block(fn->rmi_dev,
				base + fwu->off.transfer_length,
				length,
				sizeof(length));
		if (ret < 0) {
			dev_err(&fn->dev,
				"%s: Failed to write transfer length "
				"(%d blocks remaining)\n",
				__func__, remaining);
			return ret;
		}

		ret = fwu_write_f34_command_v7(fn, command);
		if (ret < 0) {
			dev_err(&fn->dev, "%s: Failed to write command "
				"(%d blocks remaining)\n",
				__func__, remaining);
			return ret;
		}

		ret = fwu_wait_for_idle(fn, ENABLE_WAIT_MS);
		if (ret < 0) {
			dev_err(&fn->dev, "%s: Failed to wait for idle status "
				"(%d blocks remaining)\n",
				__func__, remaining);
			return ret;
		}

		ret = rmi_read_block(fn->rmi_dev,
				base + fwu->off.payload,
				&fwu->read_config_buf[index],
				transfer * fwu->block_size);
		if (ret < 0) {
			dev_err(&fn->dev, "%s: Failed to read block data "
				"(%d blocks remaining)\n",
				__func__, remaining);
			return ret;
		}

		index += (transfer * fwu->block_size);
		remaining -= transfer;
	} while (remaining);

	return 0;
}

static int fwu_write_f34_v7_blocks(struct rmi_function *fn,
		unsigned char *block_ptr,
		unsigned short block_cnt, unsigned char command)
{
	int ret;
	unsigned char base;
	unsigned char length[2];
	unsigned short transfer;
	unsigned short max_transfer;
	unsigned short remaining = block_cnt;
	unsigned short block_number = 0;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	base = fn->fd.data_base_addr;

	ret = fwu_write_f34_partition_id(fn, command);
	if (ret < 0)
		return ret;

	ret = rmi_write_block(fn->rmi_dev,
			base + fwu->off.block_number,
			(unsigned char *)&block_number,
			sizeof(block_number));
	if (ret < 0) {
		dev_err(&fn->dev, "%s: Failed to write block number\n",
			__func__);
		return ret;
	}

	if (fwu->payload_length > (PAGE_SIZE / fwu->block_size))
		max_transfer = PAGE_SIZE / fwu->block_size;
	else
		max_transfer = fwu->payload_length;

	do {
		if (remaining / max_transfer)
			transfer = max_transfer;
		else
			transfer = remaining;

		length[0] = (unsigned char)(transfer & MASK_8BIT);
		length[1] = (unsigned char)(transfer >> 8);

		ret = rmi_write_block(fn->rmi_dev,
				base + fwu->off.transfer_length,
				length,
				sizeof(length));
		if (ret < 0) {
			dev_err(&fn->dev,
				"%s: Failed to write transfer length (%d blocks remaining)\n",
				__func__, remaining);
			return ret;
		}

		ret = fwu_write_f34_command_v7(fn, command);
		if (ret < 0) {
			dev_err(&fn->dev,
				"%s: Failed to write command (%d blocks remaining)\n",
				__func__, remaining);
			return ret;
		}

		ret = rmi_write_block(fn->rmi_dev,
				base + fwu->off.payload,
				block_ptr,
				transfer * fwu->block_size);
		if (ret < 0) {
			dev_err(&fn->dev,
				"%s: Failed to write block data (%d blocks remaining)\n",
				__func__, remaining);
			return ret;
		}

		ret = fwu_wait_for_idle(fn, ENABLE_WAIT_MS);
		if (ret < 0) {
			dev_err(&fn->dev,
				"%s: Failed to wait for idle status (%d blocks remaining)\n",
				__func__, remaining);
			return ret;
		}

		block_ptr += (transfer * fwu->block_size);
		remaining -= transfer;

		if (command == v7_CMD_WRITE_FW)
			fwu->f34_status = 80 - 70*remaining/block_cnt;
		else if (command == v7_CMD_WRITE_CONFIG)
			fwu->f34_status = 90 - 10*remaining/block_cnt;
	} while (remaining);

	return 0;
}

static int fwu_write_f34_blocks(struct rmi_function *fn,
		unsigned char *block_ptr,
		unsigned short block_cnt, unsigned char cmd)
{
	int ret;

	ret = fwu_write_f34_v7_blocks(fn, block_ptr, block_cnt, cmd);

	return ret;
}

static int fwu_write_configuration(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	return fwu_write_f34_blocks(fn, (unsigned char *)fwu->config_data,
			fwu->config_block_count, v7_CMD_WRITE_CONFIG);
}

static int fwu_write_ui_configuration(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	fwu->config_area = v7_UI_CONFIG_AREA;
	fwu->config_data = fwu->v7_img.ui_config.data;
	fwu->config_size = fwu->v7_img.ui_config.size;
	fwu->config_block_count = fwu->config_size / fwu->block_size;

	return fwu_write_configuration(fn);
}

static int fwu_write_dp_configuration(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	fwu->config_area = v7_DP_CONFIG_AREA;
	fwu->config_data = fwu->v7_img.dp_config.data;
	fwu->config_size = fwu->v7_img.dp_config.size;
	fwu->config_block_count = fwu->config_size / fwu->block_size;

	return fwu_write_configuration(fn);
}

static int fwu_write_guest_code(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);
	int ret;
	unsigned short guest_code_block_count;

	guest_code_block_count = fwu->v7_img.guest_code.size / fwu->block_size;

	ret = fwu_write_f34_blocks(fn, (unsigned char *)fwu->v7_img.guest_code.data,
			guest_code_block_count, v7_CMD_WRITE_GUEST_CODE);
	if (ret < 0)
		return ret;

	return 0;
}

static int fwu_write_flash_configuration(struct rmi_function *fn)
{
	int ret;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	fwu->config_area = v7_FLASH_CONFIG_AREA;
	fwu->config_data = fwu->v7_img.fl_config.data;
	fwu->config_size = fwu->v7_img.fl_config.size;
	fwu->config_block_count = fwu->config_size / fwu->block_size;

	if (fwu->config_block_count != fwu->blkcount.fl_config) {
		dev_err(&fn->dev, "%s: Flash configuration size mismatch\n",
			__func__);
		return -EINVAL;
	}

	ret = fwu_write_f34_command_v7(fn, v7_CMD_ERASE_FLASH_CONFIG);
	if (ret < 0)
		return ret;

	dev_info(&fn->dev, "%s: Erase flash configuration command written\n",
		 __func__);

	ret = fwu_wait_for_idle(fn, ENABLE_WAIT_MS);
	if (ret < 0)
		return ret;

	rmi_dbg(RMI_DEBUG_FLASH, &fn->dev, "%s: Idle status detected\n",
		__func__);

	ret = fwu_write_configuration(fn);
	if (ret < 0)
		return ret;

	return 0;
}

static int fwu_write_partition_table(struct rmi_function *fn)
{
	int ret;
	unsigned short block_count;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	block_count = fwu->blkcount.bl_config;
	fwu->config_area = v7_BL_CONFIG_AREA;
	fwu->config_size = fwu->block_size * block_count;
	devm_kfree(&fn->dev, fwu->read_config_buf);
	fwu->read_config_buf = devm_kzalloc(&fn->dev, fwu->config_size,
					    GFP_KERNEL);
	if (!fwu->read_config_buf) {
		dev_err(&fn->dev, "%s: Failed to alloc mem for fwu->read_config_buf\n",
				__func__);
		fwu->read_config_buf_size = 0;
		return -ENOMEM;
	}
	fwu->read_config_buf_size = fwu->config_size;

	ret = fwu_read_f34_v7_blocks(fn, block_count, v7_CMD_READ_CONFIG);
	if (ret < 0)
		return ret;

	ret = fwu_erase_configuration(fn);
	if (ret < 0)
		return ret;

	ret = fwu_write_flash_configuration(fn);
	if (ret < 0)
		return ret;

	fwu->config_area = v7_BL_CONFIG_AREA;
	fwu->config_data = fwu->read_config_buf;
	fwu->config_size = fwu->v7_img.bl_config.size;
	fwu->config_block_count = fwu->config_size / fwu->block_size;

	ret = fwu_write_configuration(fn);
	if (ret < 0)
		return ret;

	return 0;
}

static int fwu_write_firmware(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);
	unsigned short firmware_block_count;

	firmware_block_count = fwu->v7_img.ui_firmware.size / fwu->block_size;

	return fwu_write_f34_blocks(fn, (unsigned char *)fwu->v7_img.ui_firmware.data,
			firmware_block_count, v7_CMD_WRITE_FW);
}

static int fwu_parse_image_info_v7(struct rmi_function *fn);

int fwu_do_reflash_v7(struct rmi_function *fn, const struct firmware *fw)
{
	int ret;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	fwu_read_f34_queries_bl_version(fn);

	fwu->image = fw->data;

	ret = fwu_parse_image_info_v7(fn);
	if (ret < 0)
		goto fail;

	fwu->f34_status = 5;

	if (!fwu->new_partition_table) {
		ret = fwu_check_ui_firmware_size(fn);
		if (ret < 0)
			goto fail;

		ret = fwu_check_ui_configuration_size(fn);
		if (ret < 0)
			goto fail;

		if (fwu->flash_properties.has_disp_config &&
				fwu->v7_img.contains_disp_config) {
			ret = fwu_check_dp_configuration_size(fn);
			if (ret < 0)
				goto fail;
		}

		if (fwu->has_guest_code && fwu->v7_img.contains_guest_code) {
			ret = fwu_check_guest_code_size(fn);
			if (ret < 0)
				goto fail;
		}
	} else {
		ret = fwu_check_bl_configuration_size(fn);
		if (ret < 0)
			goto fail;
	}

	ret = fwu_erase_all(fn);
	if (ret < 0)
		goto fail;

	if (fwu->new_partition_table) {
		ret = fwu_write_partition_table(fn);
		if (ret < 0)
			goto fail;
		dev_info(&fn->dev, "%s: Partition table programmed\n", __func__);
	}

	fwu->f34_status = 10;

	ret = fwu_write_firmware(fn);
	if (ret < 0)
		goto fail;
	dev_info(&fn->dev, "%s: Firmware programmed\n", __func__);

	fwu->config_area = v7_UI_CONFIG_AREA;
	ret = fwu_write_ui_configuration(fn);
	if (ret < 0)
		goto fail;
	dev_info(&fn->dev, "%s: Configuration programmed\n", __func__);

	if (fwu->flash_properties.has_disp_config &&
			fwu->v7_img.contains_disp_config) {
		ret = fwu_write_dp_configuration(fn);
		if (ret < 0)
			goto fail;
		dev_info(&fn->dev, "%s: Display configuration programmed\n", __func__);
	}

	fwu->f34_status = 95;

	if (fwu->new_partition_table) {
		if (fwu->has_guest_code && fwu->v7_img.contains_guest_code) {
			ret = fwu_write_guest_code(fn);
			if (ret < 0)
				goto fail;
			dev_info(&fn->dev, "%s: Guest code programmed\n", __func__);
		}
	}

	fwu->f34_status = 0;
	return ret;

fail:
	fwu->f34_status = ret;
	return ret;
}

static void fwu_parse_image_header_10_bl_container(struct rmi_function *fn,
						   const unsigned char *image)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	unsigned char ii;
	unsigned char num_of_containers;
	unsigned int addr;
	unsigned int container_id;
	unsigned int length;
	const unsigned char *content;
	struct container_descriptor *descriptor;

	num_of_containers = (fwu->v7_img.bootloader.size - 4) / 4;

	for (ii = 1; ii <= num_of_containers; ii++) {
		addr = le_to_uint(fwu->v7_img.bootloader.data + (ii * 4));
		descriptor = (struct container_descriptor *)(image + addr);
		container_id = descriptor->container_id[0] |
				descriptor->container_id[1] << 8;
		content = image + le_to_uint(descriptor->content_address);
		length = le_to_uint(descriptor->content_length);
		switch (container_id) {
		case BL_CONFIG_CONTAINER:
		case GLOBAL_PARAMETERS_CONTAINER:
			fwu->v7_img.bl_config.data = content;
			fwu->v7_img.bl_config.size = length;
			break;
		case BL_LOCKDOWN_INFO_CONTAINER:
		case DEVICE_CONFIG_CONTAINER:
			fwu->v7_img.lockdown.data = content;
			fwu->v7_img.lockdown.size = length;
			break;
		default:
			break;
		};
	}

	return;
}

static void fwu_parse_image_header_10(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	unsigned char ii;
	unsigned char num_of_containers;
	unsigned int addr;
	unsigned int offset;
	unsigned int container_id;
	unsigned int length;
	const unsigned char *image;
	const unsigned char *content;
	struct container_descriptor *descriptor;
	struct image_header_10 *header;

	image = fwu->image;
	header = (struct image_header_10 *)image;

	fwu->v7_img.checksum = le_to_uint(header->checksum);

	dev_info(&fn->dev, "%s: fwu->v7_img.checksum=%d\n",
		 __func__, fwu->v7_img.checksum);

	/* address of top level container */
	offset = le_to_uint(header->top_level_container_start_addr);
	descriptor = (struct container_descriptor *)(image + offset);

	/* address of top level container content */
	offset = le_to_uint(descriptor->content_address);
	num_of_containers = le_to_uint(descriptor->content_length) / 4;

	for (ii = 0; ii < num_of_containers; ii++) {
		addr = le_to_uint(image + offset);
		offset += 4;
		descriptor = (struct container_descriptor *)(image + addr);
		container_id = descriptor->container_id[0] |
				descriptor->container_id[1] << 8;
		content = image + le_to_uint(descriptor->content_address);
		length = le_to_uint(descriptor->content_length);

		dev_info(&fn->dev, "%s: container_id=%d, length=%d\n",
			 __func__, container_id, length);

		switch (container_id) {
		case UI_CONTAINER:
		case CORE_CODE_CONTAINER:
			fwu->v7_img.ui_firmware.data = content;
			fwu->v7_img.ui_firmware.size = length;
			break;
		case UI_CONFIG_CONTAINER:
		case CORE_CONFIG_CONTAINER:
			fwu->v7_img.ui_config.data = content;
			fwu->v7_img.ui_config.size = length;
			break;
		case BL_CONTAINER:
			fwu->v7_img.bl_version = *content;
			fwu->v7_img.bootloader.data = content;
			fwu->v7_img.bootloader.size = length;
			fwu_parse_image_header_10_bl_container(fn, image);
			break;
		case GUEST_CODE_CONTAINER:
			fwu->v7_img.contains_guest_code = true;
			fwu->v7_img.guest_code.data = content;
			fwu->v7_img.guest_code.size = length;
			break;
		case DISPLAY_CONFIG_CONTAINER:
			fwu->v7_img.contains_disp_config = true;
			fwu->v7_img.dp_config.data = content;
			fwu->v7_img.dp_config.size = length;
			break;
		case FLASH_CONFIG_CONTAINER:
			fwu->v7_img.contains_flash_config = true;
			fwu->v7_img.fl_config.data = content;
			fwu->v7_img.fl_config.size = length;
			break;
		case GENERAL_INFORMATION_CONTAINER:
			fwu->v7_img.contains_firmware_id = true;
			fwu->v7_img.firmware_id = le_to_uint(content + 4);
			break;
		default:
			break;
		}
	}

	return;
}


static void fwu_compare_partition_tables(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	if (fwu->phyaddr.ui_firmware != fwu->v7_img.phyaddr.ui_firmware) {
		fwu->new_partition_table = true;
		return;
	}

	if (fwu->phyaddr.ui_config != fwu->v7_img.phyaddr.ui_config) {
		fwu->new_partition_table = true;
		return;
	}

	if (fwu->flash_properties.has_disp_config) {
		if (fwu->phyaddr.dp_config != fwu->v7_img.phyaddr.dp_config) {
			fwu->new_partition_table = true;
			return;
		}
	}

	if (fwu->flash_properties.has_disp_config) {
		if (fwu->phyaddr.dp_config != fwu->v7_img.phyaddr.dp_config) {
			fwu->new_partition_table = true;
			return;
		}
	}

	if (fwu->has_guest_code) {
		if (fwu->phyaddr.guest_code != fwu->v7_img.phyaddr.guest_code) {
			fwu->new_partition_table = true;
			return;
		}
	}

	fwu->new_partition_table = false;

	return;
}

static int fwu_parse_image_info_v7(struct rmi_function *fn)
{
	struct image_header_10 *header;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	header = (struct image_header_10 *)fwu->image;

	memset(&fwu->v7_img, 0x00, sizeof(fwu->v7_img));

	dev_info(&fn->dev, "%s: header->major_header_version = %d\n",
		 __func__, header->major_header_version);

	switch (header->major_header_version) {
	case IMAGE_HEADER_VERSION_10:
		fwu_parse_image_header_10(fn);
		break;
	default:
		dev_err(&fn->dev, "%s: Unsupported image file format (0x%02x)\n",
			__func__, header->major_header_version);
		return -EINVAL;
	}

	if (fwu->bl_version == BL_V7) {
		if (!fwu->v7_img.contains_flash_config) {
			dev_err(&fn->dev, "%s: No flash config found in firmware image\n",
				__func__);
			return -EINVAL;
		}

		fwu_parse_partition_table(fn, fwu->v7_img.fl_config.data,
				&fwu->v7_img.blkcount, &fwu->v7_img.phyaddr);

		fwu_compare_partition_tables(fn);
	} else {
		fwu->new_partition_table = false;
	}

	return 0;
}

int fwu_enter_flash_prog_v7(struct rmi_function *fn)
{
	int ret;
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);
	/*unsigned char int_enable = 0x00;*/

	ret = fwu_read_flash_status_v7(fn);
	if (ret < 0)
		return ret;

	if (fwu->in_bl_mode)
		return 0;

	ret = fwu_write_f34_command_v7(fn, v7_CMD_ENABLE_FLASH_PROG);
	if (ret < 0)
		return ret;

	ret = fwu_wait_for_idle(fn, ENABLE_WAIT_MS);
	if (ret < 0)
		return ret;

	if (!fwu->in_bl_mode) {
		dev_err(&fn->dev, "%s: BL mode not entered\n", __func__);
		return -EINVAL;
	}

	return 0;
}

int fwu_start_reflash_v7(struct rmi_function *fn,
			 const struct firmware *fw)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);
	int ret = 0;

	dev_info(&fn->dev, "%s: start reflash process\n", __func__);

	if (fwu->bl_version != BL_V7) {
		dev_err(&fn->dev, "%s: Only support V7\n",
				__func__);
		ret = -EINVAL;
		goto exit;
	}

	fwu->config_area = v7_UI_CONFIG_AREA;
	fwu->image = fw->data;

	ret = fwu_parse_image_info_v7(fn);
	if (ret < 0)
		goto exit;

	if (fwu->bl_version != fwu->v7_img.bl_version) {
		dev_err(&fn->dev, "%s: Bootloader version mismatch\n",
				__func__);
		ret = -EINVAL;
		goto exit;
	}

	fwu->force_update = true;

	if (!fwu->force_update && fwu->new_partition_table) {
		dev_err(&fn->dev, "%s: Partition table mismatch\n",
				__func__);
		ret = -EINVAL;
		goto exit;
	}

	ret = fwu_read_flash_status_v7(fn);
	if (ret < 0)
		goto exit;

	if (fwu->in_bl_mode) {
		dev_info(&fn->dev, "%s: Device in bootloader mode\n",
				__func__);
	}

	fwu_enter_flash_prog_v7(fn);

	return 0;

exit:
	return ret;
}

const char *rmi_f34_get_bootloader_ID(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	return devm_kasprintf(&fn->dev, GFP_KERNEL, "%d%d",
			      fwu->bootloader_id[1],fwu->bootloader_id[0]);
}

const char *rmi_f34_get_configuration_ID(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);
	char f34_ctrl[4];
	int ret;

	if (!fwu->flash_properties.has_config_id)
		return NULL;

	if (!fwu->config_id) {
		ret = rmi_read_block(fn->rmi_dev,
				fn->fd.control_base_addr,
				f34_ctrl,
				sizeof(f34_ctrl));
		if (ret)
			return NULL;

		fwu->config_id = devm_kasprintf(&fn->dev, GFP_KERNEL,
						"%02x%02x%02x%02x",
						f34_ctrl[0], f34_ctrl[1],
						f34_ctrl[2], f34_ctrl[3]);
		if (!fwu->config_id)
			return NULL;
	}

	return fwu->config_id;
}

int rmi_f34_status(struct rmi_function *fn)
{
	struct f34_data *fwu = dev_get_drvdata(&fn->dev);

	return fwu->f34_status;
}

static int rmi_f34_attention(struct rmi_function *fn, unsigned long *irq_bits)
{
	rmi_dbg(RMI_DEBUG_FLASH, &fn->dev, "%s\n", __func__);
	return 0;
}

static int rmi_f34_probe(struct rmi_function *fn)
{
	struct f34_data *fwu;
	int ret;

	fwu = devm_kzalloc(&fn->dev, sizeof(struct f34_data), GFP_KERNEL);
	if (!fwu)
		return -ENOMEM;

	fwu->fn = fn;
	dev_set_drvdata(&fn->dev, fwu);

	memset(&fwu->img, 0, sizeof(struct img_file_content));

	fwu->img.image_name = devm_kzalloc(&fn->dev, MAX_IMAGE_NAME_LEN,
					   GFP_KERNEL);
	if (!fwu->img.image_name) {
		dev_err(&fn->dev, "%s: Failed to alloc mem for image name\n",
			__func__);
		ret = -ENOMEM;
		goto cleanup;
	}

	/* Read bootloader version */
	ret = fwu_read_f34_queries_bl_version(fn);

	if (fwu->bootloader_id[1] == '5') {
		fwu->bl_version = V5;
	} else if (fwu->bootloader_id[1] == '6') {
		fwu->bl_version = V6;
	} else if (fwu->bootloader_id[1] == 7) {
		fwu->bl_version = BL_V7;
	} else {
		dev_err(&fn->dev, "%s: Unrecognized bootloader version\n",
				__func__);
		return -EINVAL;
	}

	memset(&fwu->blkcount, 0x00, sizeof(fwu->blkcount));
	memset(&fwu->phyaddr, 0x00, sizeof(fwu->phyaddr));
	fwu_read_f34_queries_v7(fn);

	fwu->force_update = FORCE_UPDATE;
	fwu->initialized = true;
	return 0;

cleanup:
	return ret;
}

struct rmi_function_handler rmi_f34_handler = {
	.driver = {
		.name = "rmi4_f34",
	},
	.func = 0x34,
	.probe = rmi_f34_probe,
	.attention = rmi_f34_attention,
};
