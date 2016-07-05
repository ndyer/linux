/*
 * Copyright (c) 2007-2016, Synaptics Incorporated
 * Copyright (C) 2016 Zodiac Inflight Innovations
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/rmi.h>
#include <linux/firmware.h>
#include <asm/unaligned.h>
#include <asm/unaligned.h>

#include "rmi_driver.h"

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

struct f34_data {
	struct rmi_function *fn;

	u16 block_size;
	u16 fw_blocks;
	u16 config_blocks;
	u16 ctrl_address;
	u8 status;
	struct completion cmd_done;

	struct mutex flash_mutex;

	int update_status;
	int update_progress;
	int update_size;
	struct completion async_firmware_done;

	unsigned char bootloader_id[5];
	unsigned char configuration_id[9];
};

static int rmi_f34_write_bootloader_id(struct f34_data *f34)
{
	struct rmi_function *fn = f34->fn;
        struct rmi_device *rmi_dev = fn->rmi_dev;
        u8 bootloader_id[F34_BOOTLOADER_ID_LEN];
        int ret;

        ret = rmi_read_block(rmi_dev, fn->fd.query_base_addr,
			     bootloader_id, sizeof(bootloader_id));
        if (ret) {
                dev_err(&fn->dev, "%s: Reading bootloader ID failed: %d\n",
                        __func__, ret);
                return ret;
        }

        rmi_dbg(RMI_DEBUG_FN, &fn->dev, "%s: writing bootloader id '%c%c'\n",
                __func__, bootloader_id[0], bootloader_id[1]);

        ret = rmi_write_block(rmi_dev,
			      fn->fd.data_base_addr + F34_BLOCK_DATA_OFFSET,
			      bootloader_id, sizeof(bootloader_id));
        if (ret) {
                dev_err(&fn->dev, "Failed to write bootloader ID: %d\n", ret);
                return ret;
        }

        return 0;
}

static int rmi_f34_command(struct f34_data *f34, u8 command,
			   unsigned int timeout, bool write_bl_id)
{
	struct rmi_function *fn = f34->fn;
        struct rmi_device *rmi_dev = fn->rmi_dev;
        int ret;

        if (write_bl_id) {
                ret = rmi_f34_write_bootloader_id(f34);
                if (ret)
                        return ret;
        }

        init_completion(&f34->cmd_done);

        ret = rmi_read(rmi_dev, f34->ctrl_address, &f34->status);
        if (ret) {
                dev_err(&f34->fn->dev,
                        "%s: Failed to read cmd register: %d (command %#02x)\n",
                        __func__, ret, command);
                return ret;
        }

        f34->status |= command & 0x0f;

        ret = rmi_write(rmi_dev, f34->ctrl_address, f34->status);
        if (ret < 0) {
                dev_err(&f34->fn->dev,
                        "Failed to write F34 command %#02x: %d\n",
                        command, ret);
                return ret;
        }

        if (!wait_for_completion_timeout(&f34->cmd_done,
                                         msecs_to_jiffies(timeout))) {

                ret = rmi_read(rmi_dev, f34->ctrl_address, &f34->status);
                if (ret) {
                        dev_err(&f34->fn->dev,
                                "%s: failed to read status after command %#02x timed out: %d\n",
                                __func__, command, ret);
                        return ret;
                }

                if (f34->status & 0x7f) {
                        dev_err(&f34->fn->dev,
                                "%s: command %#02x timed out, fw status: %#02x\n",
                                __func__, command, f34->status);
                        return -ETIMEDOUT;
                }
        }

        return 0;
}

static int rmi_f34_attention(struct rmi_function *fn, unsigned long *irq_bits)
{
	struct f34_data *f34 = dev_get_drvdata(&fn->dev);
        int ret;

        ret = rmi_read(f34->fn->rmi_dev, f34->ctrl_address,
                                             &f34->status);
        rmi_dbg(RMI_DEBUG_FN, &fn->dev, "%s: status: %#02x, ret: %d\n",
                __func__, f34->status, ret);

        if (!ret && !(f34->status & 0x7f))
                complete(&f34->cmd_done);

        return 0;
}

static int rmi_f34_write_blocks(struct f34_data *f34, const void *data,
				int block_count, u8 command)
{
	struct rmi_function *fn = f34->fn;
	struct rmi_device *rmi_dev = fn->rmi_dev;
        u16 address = fn->fd.data_base_addr + F34_BLOCK_DATA_OFFSET;
        u8 start_address[] = { 0, 0 };
        int i;
        int ret;

        ret = rmi_write_block(rmi_dev, fn->fd.data_base_addr,
			start_address, sizeof(start_address));
        if (ret) {
                dev_err(&fn->dev, "Failed to write initial zeros: %d\n", ret);
                return ret;
        }

        for (i = 0; i < block_count; i++) {
                ret = rmi_write_block(rmi_dev, address, data, f34->block_size);
                if (ret) {
                        dev_err(&fn->dev,
                                "failed to write block #%d: %d\n", i, ret);
                        return ret;
                }

                ret = rmi_f34_command(f34, command, F34_IDLE_WAIT_MS, false);
                if (ret) {
                        dev_err(&fn->dev,
                                "Failed to write command for block #%d: %d\n",
                                i, ret);
                        return ret;
                }

                rmi_dbg(RMI_DEBUG_FN, &fn->dev, "wrote block %d of %d\n",
                        i + 1, block_count);

                data += f34->block_size;
                f34->update_progress += f34->block_size;
                f34->update_status = (f34->update_progress * 100) /
				     f34->update_size;
        }

        return 0;
}

static int rmi_f34_write_firmware(struct f34_data *f34, const void *data)
{
        return rmi_f34_write_blocks(f34, data, f34->fw_blocks,
				F34_WRITE_FW_BLOCK);
}

static int rmi_f34_write_config(struct f34_data *f34, const void *data)
{
        return rmi_f34_write_blocks(f34, data, f34->config_blocks,
				F34_WRITE_CONFIG_BLOCK);
}

int rmi_f34_enable_flash(struct rmi_function *fn)
{
	struct f34_data *f34 = dev_get_drvdata(&fn->dev);

	return rmi_f34_command(f34, F34_ENABLE_FLASH_PROG,
			       F34_ENABLE_WAIT_MS, true);
}

static int rmi_f34_flash_firmware(struct f34_data *f34,
					 const struct rmi_f34_firmware *syn_fw)
{
	struct rmi_function *fn = f34->fn;
	int ret;

	f34->update_progress = 0;
	f34->update_size = syn_fw->image_size + syn_fw->config_size;
	if (syn_fw->image_size) {
		dev_info(&fn->dev, "Erasing FW...\n");
		ret = rmi_f34_command(f34, F34_ERASE_ALL,
				      F34_ERASE_WAIT_MS, true);
		if (ret)
			return ret;

		dev_info(&fn->dev, "Writing firmware data (%d bytes)...\n",
			 syn_fw->image_size);
		ret = rmi_f34_write_firmware(f34, syn_fw->data);
		if (ret)
			return ret;
	}

	if (syn_fw->config_size) {
		/*
		 * We only need to erase config if we haven't updated
		 * firmware.
		 */
		if (!syn_fw->image_size) {
			dev_info(&fn->dev, "%s: Erasing config data...\n",
					__func__);
			ret = rmi_f34_command(f34, F34_ERASE_CONFIG,
					      F34_ERASE_WAIT_MS, true);
			if (ret)
				return ret;
		}

		dev_info(&fn->dev, "%s: Writing config data (%d bytes)...\n",
				__func__, syn_fw->config_size);
		ret = rmi_f34_write_config(f34,
				&syn_fw->data[syn_fw->image_size]);
		if (ret)
			return ret;
	}

	dev_info(&fn->dev, "%s: Firmware update complete\n", __func__);
	return 0;
}

int rmi_f34_update_firmware(struct rmi_function *fn, const struct firmware *fw)
{
	struct f34_data *f34 = dev_get_drvdata(&fn->dev);
	const struct rmi_f34_firmware *syn_fw;
	int ret;

	syn_fw = (const struct rmi_f34_firmware *)fw->data;
        BUILD_BUG_ON(offsetof(struct rmi_f34_firmware, data) !=
                     F34_FW_IMAGE_OFFSET);

        rmi_dbg(RMI_DEBUG_FN, &fn->dev,
                 "FW size:%d, checksum:%08x, image_size:%d, config_size:%d\n",
                 (int)fw->size,
                 le32_to_cpu(syn_fw->checksum),
                 le32_to_cpu(syn_fw->image_size),
                 le32_to_cpu(syn_fw->config_size));

        dev_info(&fn->dev,
                 "FW bootloader_id:%02x, product_id:%.*s, info: %02x%02x\n",
                 syn_fw->bootloader_version,
                 (int)sizeof(syn_fw->product_id), syn_fw->product_id,
                 syn_fw->product_info[0], syn_fw->product_info[1]);

        if (syn_fw->image_size &&
            syn_fw->image_size != f34->fw_blocks * f34->block_size) {
                dev_err(&fn->dev,
                        "Bad firmware image: fw size %d, expected %d\n",
                        syn_fw->image_size,
                        f34->fw_blocks * f34->block_size);
                ret = -EILSEQ;
                goto out;
        }

        if (syn_fw->config_size &&
            syn_fw->config_size != f34->config_blocks * f34->block_size) {
                dev_err(&fn->dev,
                        "Bad firmware image: config size %d, expected %d\n",
                        syn_fw->config_size,
                        f34->config_blocks * f34->block_size);
                ret = -EILSEQ;
                goto out;
        }

        if (syn_fw->image_size && !syn_fw->config_size) {
                dev_err(&fn->dev,
                        "Bad firmware image: no config data\n");
                ret = -EILSEQ;
                goto out;
        }

	dev_info(&fn->dev, "Starting firmware update\n");
	mutex_lock(&f34->flash_mutex);

	ret = rmi_f34_flash_firmware(f34, syn_fw);
	dev_info(&fn->dev, "Firmware update complete, status:%d\n", ret);

	f34->update_status = ret;
	mutex_unlock(&f34->flash_mutex);

out:
	return ret;
}

int rmi_f34_status(struct rmi_function *fn)
{
	struct f34_data *f34 = dev_get_drvdata(&fn->dev);

	/*
	 * The status is the percentage complete, or once complete,
	 * zero for success or a negative return code.
	 */
	return f34->update_status;
}

int rmi_f34_check_supported(struct rmi_function *fn)
{
	u8 version = fn->fd.function_version;

	/* Only version 0 currently supported */
	if (version == 0) {
		return 0;
	} else {
		dev_warn(&fn->dev, "F34 V%d not supported!\n", version);
		return -ENODEV;
	}
}

static int rmi_f34_probe(struct rmi_function *fn)
{
	struct f34_data *f34;
	unsigned char f34_queries[9];
	bool has_config_id;
	int ret;

	ret = rmi_f34_check_supported(fn);
	if (ret)
		return ret;

	f34 = devm_kzalloc(&fn->dev, sizeof(struct f34_data), GFP_KERNEL);
	if (!f34)
		return -ENOMEM;

	f34->fn = fn;
	dev_set_drvdata(&fn->dev, f34);

	mutex_init(&f34->flash_mutex);
	init_completion(&f34->cmd_done);
	init_completion(&f34->async_firmware_done);

	ret = rmi_read_block(fn->rmi_dev, fn->fd.query_base_addr,
			     f34_queries, sizeof(f34_queries));
	if (ret) {
		dev_err(&fn->dev, "%s: Failed to query properties\n",
				__func__);
		return ret;
	}

	snprintf(f34->bootloader_id, sizeof(f34->bootloader_id),
		 "%c%c", f34_queries[0], f34_queries[1]);

	f34->block_size = get_unaligned_le16(&f34_queries[3]);
	f34->fw_blocks = get_unaligned_le16(&f34_queries[5]);
	f34->config_blocks = get_unaligned_le16(&f34_queries[7]);
	f34->ctrl_address = fn->fd.data_base_addr + F34_BLOCK_DATA_OFFSET +
		f34->block_size;
	has_config_id = f34_queries[2] & (1 << 2);

	dev_info(&fn->dev, "Bootloader ID: %s\n", f34->bootloader_id);
	rmi_dbg(RMI_DEBUG_FN, &fn->dev, "Block size: %d\n", f34->block_size);
	rmi_dbg(RMI_DEBUG_FN, &fn->dev, "FW blocks: %d\n", f34->fw_blocks);
	rmi_dbg(RMI_DEBUG_FN, &fn->dev, "CFG blocks: %d\n", f34->config_blocks);

	if (has_config_id) {
		ret = rmi_read_block(fn->rmi_dev, fn->fd.control_base_addr,
				     f34_queries, sizeof(f34_queries));
		if (ret) {
			dev_err(&fn->dev, "Failed to read F34 config ID\n");
			return ret;
		}

		snprintf(f34->configuration_id,
				sizeof(f34->configuration_id),
				"%02x%02x%02x%02x", f34_queries[0],
				f34_queries[1], f34_queries[2],
				f34_queries[3]);
		dev_info(&fn->dev, "Configuration ID: %s\n",
			 f34->configuration_id);
	}

	return 0;
}

struct rmi_function_handler rmi_f34_handler = {
	.driver = {
		.name = "rmi4_f34",
	},
	.func = 0x34,
	.probe = rmi_f34_probe,
	.attention = rmi_f34_attention,
};
