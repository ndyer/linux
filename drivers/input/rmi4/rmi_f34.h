/*
 * Copyright (c) 2007-2016, Synaptics Incorporated
 * Copyright (C) 2016 Zodiac Inflight Innovations
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _RMI_F34_H
#define _RMI_F34_H

/* F34 image file offsets. */
#define F34_FW_IMAGE_OFFSET	0x100

/* F34 register offsets. */
#define F34_BLOCK_DATA_OFFSET	2

/* F34 commands */
#define F34_WRITE_FW_BLOCK	0x2
#define F34_ERASE_ALL		0x3
#define F34_READ_CONFIG_BLOCK	0x5
#define F34_WRITE_CONFIG_BLOCK	0x6
#define F34_ERASE_CONFIG	0x7
#define F34_ENABLE_FLASH_PROG	0xf

#define F34_STATUS_IN_PROGRESS	0xff
#define F34_STATUS_IDLE		0x80

#define F34_IDLE_WAIT_MS	500
#define F34_ENABLE_WAIT_MS	300
#define F34_ERASE_WAIT_MS	5000

#define F34_BOOTLOADER_ID_LEN	2

/* F34 V7 defines */
#define V7_FLASH_STATUS_OFFSET	0
#define V7_PARTITION_ID_OFFSET	1
#define V7_BLOCK_NUMBER_OFFSET	2
#define V7_TRANSFER_LENGTH_OFFSET 3
#define V7_COMMAND_OFFSET 4
#define V7_PAYLOAD_OFFSET 5
#define V7_BOOTLOADER_ID_OFFSET 1

#define V7_PARTITION_SUPPORT_BYTES 4

#define IMAGE_HEADER_VERSION_10 0x10

#define CONFIG_ID_SIZE 32
#define PRODUCT_ID_SIZE 10

#define ENABLE_WAIT_MS (1 * 1000)
#define WRITE_WAIT_MS (3 * 1000)

#define MIN_SLEEP_TIME_US 50
#define MAX_SLEEP_TIME_US 100

#define HAS_BSR		BIT(5)
#define HAS_CONFIG_ID	BIT(3)
#define HAS_GUEST_CODE	BIT(6)
#define HAS_DISP_CFG	BIT(5)

enum rmi_f34_bl_version {
	BL_V5 = 5,
	BL_V6 = 6,
	BL_V7 = 7,
};

enum rmi_f34v7_flash_command2 {
	CMD_V7_IDLE = 0x00,
	CMD_V7_ENTER_BL,
	CMD_V7_READ,
	CMD_V7_WRITE,
	CMD_V7_ERASE,
	CMD_V7_ERASE_AP,
	CMD_V7_SENSOR_ID,
};

enum rmi_f34v7_flash_command {
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

enum rmi_f34v7_config_area {
	v7_UI_CONFIG_AREA = 0,
	v7_PM_CONFIG_AREA,
	v7_BL_CONFIG_AREA,
	v7_DP_CONFIG_AREA,
	v7_FLASH_CONFIG_AREA,
};

enum rmi_f34v7_partition_id {
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

struct f34v7_query_1_7 {
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
			unsigned char partition_support_0;
			unsigned char partition_support_1;
			unsigned char partition_support_2;
			unsigned char partition_support_3;
		} __packed;
		unsigned char data[21];
	};
};

struct f34v7_data_1_5 {
	union {
		struct {
			unsigned char partition_id;
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
	unsigned char partition_id;
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
	bool contains_display_cfg;
	bool contains_guest_code;
	bool contains_flash_config;
	unsigned int firmware_id;
	unsigned int checksum;
	unsigned int bootloader_size;
	unsigned int display_cfg_offset;
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

struct rmi_f34_firmware {
	__le32 checksum;
	u8 pad1[3];
	u8 bootloader_version;
	__le32 image_size;
	__le32 config_size;
	u8 product_id[10];
	u8 product_info[2];
	u8 pad2[228];
	u8 data[];
};

struct f34v5_data {
	u16 block_size;
	u16 fw_blocks;
	u16 config_blocks;
	u16 ctrl_address;
	u8 status;

	struct completion cmd_done;
	struct mutex flash_mutex;
};

struct f34v7_data {
	bool has_display_cfg;
	bool has_guest_code;
	bool force_update;
	bool in_bl_mode;
	unsigned char *read_config_buf;
	unsigned short read_config_buf_size;
	unsigned char command;
	unsigned char flash_status;
	unsigned short block_size;
	unsigned short config_block_count;
	unsigned short config_size;
	unsigned short config_area;
	unsigned short flash_config_length;
	unsigned short payload_length;
	unsigned char partitions;
	unsigned short partition_table_bytes;
	bool new_partition_table;

	struct register_offset off;
	struct block_count blkcount;
	struct physical_address phyaddr;
	struct image_metadata img;

	const unsigned char *config_data;
	const unsigned char *image;
};

struct f34_data {
	struct rmi_function *fn;

	enum rmi_f34_bl_version bl_version;
	unsigned char bootloader_id[5];
	unsigned char configuration_id[CONFIG_ID_SIZE*2 + 1];

	int update_status;
	int update_progress;
	int update_size;

	union {
		struct f34v5_data v5;
		struct f34v7_data v7;
	};
};

int rmi_f34v7_start_reflash(struct f34_data *f34, const struct firmware *fw);
int rmi_f34v7_do_reflash(struct f34_data *f34, const struct firmware *fw);
int rmi_f34v7_probe(struct f34_data *f34);

#endif /* _RMI_F34_H */
