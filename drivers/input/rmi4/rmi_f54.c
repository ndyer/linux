/*
 * Copyright (c) 2012-2015 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/rmi.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include "rmi_driver.h"

#define F54_NAME		"rmi4_f54"

/* F54 data offsets */
#define F54_REPORT_DATA_OFFSET  3
#define F54_FIFO_OFFSET         1
#define F54_NUM_TX_OFFSET       1
#define F54_NUM_RX_OFFSET       0

/* F54 commands */
#define F54_GET_REPORT          1
#define F54_FORCE_CAL           2

/* Fixed sizes of reports */
#define F54_FULL_RAW_CAP_MIN_MAX_SIZE   4
#define F54_HIGH_RESISTANCE_SIZE(rx, tx) \
					(2 * ((rx) * (tx) + (rx) + (tx) + 3))
#define F54_MAX_REPORT_SIZE(rx, tx)	F54_HIGH_RESISTANCE_SIZE((rx), (tx))

/* definitions for F54 Query Registers in ultra-portable unionstruct form */
struct f54_ad_query {
        /* query 0 */
        u8 num_rx_electrodes;

        /* query 1 */
        u8 num_tx_electrodes;

        union {
                struct {
                        /* query2 */
                        u8 f54_ad_query2_b0__1:2;
                        u8 has_baseline:1;
                        u8 has_image8:1;
                        u8 f54_ad_query2_b4__5:2;
                        u8 has_image16:1;
                        u8 f54_ad_query2_b7:1;
                };
                u8 f54_ad_query2;
        };

        /* query 3.0 and 3.1 */
        u16 clock_rate;

        /* query 4 */
        u8 family;

        /* query 5 */
        union {
                struct {
                        u8 has_pixel_touch_threshold_adjustment:1;
                        u8 f54_ad_query5_b1__7:7;
                };
                u8 f54_ad_query5;
        };

        /* query 6 */
        union {
                struct {
                u8 has_sensor_assignment:1;
                u8 has_interference_metric:1;
                u8 has_sense_frequency_control:1;
                u8 has_firmware_noise_mitigation:1;
                u8 f54_ad_query6_b4:1;
                u8 has_two_byte_report_rate:1;
                u8 has_one_byte_report_rate:1;
                u8 has_relaxation_control:1;
                };
                u8 f54_ad_query6;
        };

        /* query 7 */
        union {
                struct {
                        u8 curve_compensation_mode:2;
                        u8 f54_ad_query7_b2__7:6;
                };
                u8 f54_ad_query7;
        };

        /* query 8 */
        union {
                struct {
                u8 f54_ad_query2_b0:1;
                u8 has_iir_filter:1;
                u8 has_cmn_removal:1;
                u8 has_cmn_maximum:1;
                u8 has_pixel_threshold_hysteresis:1;
                u8 has_edge_compensation:1;
                u8 has_perf_frequency_noisecontrol:1;
                u8 f54_ad_query8_b7:1;
                };
                u8 f54_ad_query8;
        };

        u8 f54_ad_query9;
        u8 f54_ad_query10;
        u8 f54_ad_query11;

        /* query 12 */
        union {
                struct {
                        u8 number_of_sensing_frequencies:4;
                        u8 f54_ad_query12_b4__7:4;
                };
                u8 f54_ad_query12;
        };

	u8 f54_ad_query13;
} __packed;

enum f54_report_type {
	F54_REPORT_NONE = 0,
	F54_8BIT_IMAGE = 1,
	F54_16BIT_IMAGE = 2,
	F54_RAW_16BIT_IMAGE = 3,
	F54_HIGH_RESISTANCE = 4,
	F54_TX_TO_TX_SHORT = 5,
	F54_RX_TO_RX1 = 7,
	F54_TRUE_BASELINE = 9,
	F54_FULL_RAW_CAP_MIN_MAX = 13,
	F54_RX_OPENS1 = 14,
	F54_TX_OPEN = 15,
	F54_TX_TO_GROUND = 16,
	F54_RX_TO_RX2 = 17,
	F54_RX_OPENS2 = 18,
	F54_FULL_RAW_CAP = 19,
	F54_FULL_RAW_CAP_RX_COUPLING_COMP = 20,
};

struct f54_data {
	struct rmi_function *fn;

	struct f54_ad_query qry;

	enum f54_report_type report_type;
	u8 *report_data;
	int report_size;

	bool is_busy;
	struct mutex status_mutex;
	struct mutex data_mutex;

	struct workqueue_struct *workqueue;
	struct delayed_work work;
	unsigned long timeout;

	struct completion cmd_done;

	/* Data used for debugging */
	u8 control_addr;
	u8 data_addr;
};

/*
 * Basic checks on report_type to ensure we write a valid type
 * to the sensor.
 */
static bool is_f54_report_type_valid(struct f54_data *f54,
                                     enum f54_report_type reptype)
{
	switch (reptype) {
	case F54_8BIT_IMAGE:
		return f54->qry.has_image8;
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
		return f54->qry.has_image16;
	case F54_TRUE_BASELINE:
		return f54->qry.has_baseline;
	case F54_HIGH_RESISTANCE:
	case F54_TX_TO_TX_SHORT:
	case F54_RX_TO_RX1:
	case F54_FULL_RAW_CAP_MIN_MAX:
	case F54_RX_OPENS1:
	case F54_TX_OPEN:
	case F54_TX_TO_GROUND:
	case F54_RX_TO_RX2:
	case F54_RX_OPENS2:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
		return true;
	default:
		return false;
	}
}

static int rmi_f54_request_report(struct rmi_function *fn, u8 report_type)
{
	struct f54_data *f54 = dev_get_drvdata(&fn->dev);
	struct rmi_device *rmi_dev = fn->rmi_dev;
	int error;

	/* Write Report Type into F54_AD_Data0 */
	if (f54->report_type != report_type) {
		error = rmi_write(rmi_dev, f54->fn->fd.data_base_addr,
				  report_type);
		if (error)
			return error;
		f54->report_type = report_type;
	}

	/*
	 * Small delay after disabling interrupts to avoid race condition
	 * in firmare. This value is a bit higher than absolutely necessary.
	 * Should be removed once issue is resolved in firmware.
	 */
	usleep_range(2000, 3000);

	mutex_lock(&f54->data_mutex);

	error = rmi_write(rmi_dev, fn->fd.command_base_addr, F54_GET_REPORT);
	if (error < 0)
		return error;

	init_completion(&f54->cmd_done);

	f54->is_busy = 1;
	f54->timeout = jiffies + msecs_to_jiffies(100);

	queue_delayed_work(f54->workqueue, &f54->work, 0);

	mutex_unlock(&f54->data_mutex);

	return 0;
}

static ssize_t rmi_f54_control_addr_show(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
	struct f54_data *f54 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", f54->control_addr);
}

static u8 f54_control_reg_size[107] = {
	1, 1, 2, 1, 1, 1, 1, 1, 2, 1, 1, 2, 1, 1, 1, 0,
	0, 0, 0, 0, 1, 2, 1, 2, 2, 1, 1, 1, 2, 1, 1, 1,
	1, 1, 1, 1, 0, 0, 0, 0, 0, 1, 2, 2, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 8, 2, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
	1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

static ssize_t rmi_f54_control_addr_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
	struct f54_data *f54 = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val >= ARRAY_SIZE(f54_control_reg_size))
		return -EINVAL;

	f54->control_addr = val;

	return count;
}

static DEVICE_ATTR(f54_control_addr, S_IRUGO | S_IWUSR,
                   rmi_f54_control_addr_show, rmi_f54_control_addr_store);

static ssize_t rmi_f54_control_data_show(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
	struct f54_data *f54 = dev_get_drvdata(dev);
	u8 data;
	int error;

	error = rmi_read(f54->fn->rmi_dev,
	                 f54->fn->fd.control_base_addr + f54->control_addr,
	                 &data);
	if (error)
		return error;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", data);
}

static ssize_t rmi_f54_control_data_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
	struct f54_data *f54 = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val > 255)
		return -EINVAL;

	error = rmi_write(f54->fn->rmi_dev,
	                  f54->fn->fd.control_base_addr + f54->control_addr,
	                  val);
	return error ? : count;
}

static DEVICE_ATTR(f54_control_data, S_IRUGO | S_IWUSR,
                   rmi_f54_control_data_show, rmi_f54_control_data_store);

static int f54_get_control_reg_size(struct f54_data *f54, int reg)
{
	if (reg < 0 || reg >= ARRAY_SIZE(f54_control_reg_size))
		return 0;

	switch (reg) {
	case 15:
		return f54->qry.num_rx_electrodes;
	case 16:
		return f54->qry.num_tx_electrodes;
	case 17:
	case 18:
	case 19:
	case 38:
	case 39:
	case 40:
	case 75:
		return f54->qry.f54_ad_query13 & 0x0f;
	case 36:
		switch (f54->qry.f54_ad_query8 & 0x03) {
		case 0x01:
			return max(f54->qry.num_tx_electrodes,
			           f54->qry.num_rx_electrodes);
		case 0x02:
			return f54->qry.num_rx_electrodes;
		}
		return 0;
	case 37:
		if ((f54->qry.f54_ad_query8 & 0x03) == 0x02)
			return f54->qry.num_rx_electrodes;
		return 0;
	case 64:
		return 7;       /* or 1 */
	case 87:
		return 13;
	case 89:
		return 11;
	case 91:
		return 5;
	case 93:
		return 2;
	case 94:
		return 4;
	case 95:
		return 11 * (f54->qry.f54_ad_query13 & 0x0f);
	case 96:
		return f54->qry.num_rx_electrodes;
	case 97:
		return f54->qry.num_rx_electrodes + f54->qry.num_tx_electrodes;
	case 98:
		return 10;
	case 99:
		if (f54->qry.family == 2)
			return 3;
		return 0;
	case 101:
		return 5;
	case 102:
		return 28;
	case 103:
		return 0;
	case 104:
		return 8;
	default:
		break;
	}
	return f54_control_reg_size[reg];
}

static ssize_t rmi_f54_control_data_read(struct file *data_file,
                                         struct kobject *kobj,
                                         struct bin_attribute *attr,
                                         char *buf, loff_t pos, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct f54_data *f54 = dev_get_drvdata(dev);
	size_t fsize = f54_get_control_reg_size(f54, f54->control_addr);
	int error;

	if (count == 0)
		return 0;

	if (pos > fsize)
		return -EFBIG;

	if (pos + count > fsize)
		count = fsize - pos;

	if (count == 0)
		return count;

	error = rmi_read_block(f54->fn->rmi_dev,
	                       f54->fn->fd.control_base_addr +
	                       f54->control_addr, buf, count);
	if (error < 0)
		return error;
	return count;
}

static struct bin_attribute f54_control_data = {
	.attr = {
		.name = "f54_control_data_bin",
		.mode = S_IRUGO
	},
	.size = 0,
	.read = rmi_f54_control_data_read,
};

static ssize_t rmi_f54_data_addr_show(struct device *dev,
                                      struct device_attribute *attr,
                                      char *buf)
{
	struct f54_data *f54 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", f54->data_addr);
}

static ssize_t rmi_f54_data_addr_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
	struct f54_data *f54 = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val > 19 + 6)
		return -EINVAL;

	f54->data_addr = val;

	return count;
}

static DEVICE_ATTR(f54_data_addr, S_IRUGO | S_IWUSR,
                   rmi_f54_data_addr_show, rmi_f54_data_addr_store);

static ssize_t rmi_f54_data_data_show(struct device *dev,
                                      struct device_attribute *attr, char *buf)
{
	struct f54_data *f54 = dev_get_drvdata(dev);
	u8 data;
	int error;

	error = rmi_read(f54->fn->rmi_dev,
	                 f54->fn->fd.data_base_addr + f54->data_addr,
	                 &data);
	if (error)
		return error;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", data);
}

static ssize_t rmi_f54_data_data_store(struct device *dev,
                                       struct device_attribute *attr,
                                       const char *buf, size_t count)
{
	struct f54_data *f54 = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val > 255)
		return -EINVAL;

	error = rmi_write(f54->fn->rmi_dev,
	                  f54->fn->fd.data_base_addr + f54->data_addr, val);
	return error ? : count;
}

static DEVICE_ATTR(f54_data_data, S_IRUGO | S_IWUSR,
                   rmi_f54_data_data_show, rmi_f54_data_data_store);

static ssize_t rmi_f54_command_show(struct device *dev,
                                    struct device_attribute *attr,
                                    char *buf)
{
	struct f54_data *f54 = dev_get_drvdata(dev);
	u8 data;
	int error;

	error = rmi_read(f54->fn->rmi_dev,
	                 f54->fn->fd.command_base_addr, &data);
	if (error)
		return error;

	return snprintf(buf, PAGE_SIZE, "0x%02x\n", data);
}

static ssize_t rmi_f54_command_store(struct device *dev,
                                     struct device_attribute *attr,
                                     const char *buf, size_t count)
{
	struct f54_data *f54 = dev_get_drvdata(dev);
	unsigned long val;
	int error;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val > 255)
		return -EINVAL;

	error = rmi_write(f54->fn->rmi_dev,
	                  f54->fn->fd.command_base_addr, val);
	return error ? : count;
}

static DEVICE_ATTR(f54_command, S_IRUGO | S_IWUSR,
                   rmi_f54_command_show, rmi_f54_command_store);

static ssize_t rmi_f54_report_type_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
	struct f54_data *f54 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%u\n", f54->report_type);
}

static ssize_t rmi_f54_report_type_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count)
{
	struct f54_data *f54 = dev_get_drvdata(dev);
	unsigned long val;
	int error = 0;

	if (kstrtoul(buf, 0, &val)) {
		dev_err(dev, "Not a digit\n");
		return -EINVAL;
	}

	if (val && !is_f54_report_type_valid(f54, val)) {
		dev_err(dev, "F54 report type %lu not valid\n", val);
		return -EINVAL;
	}

	mutex_lock(&f54->status_mutex);
	if (f54->is_busy) {
		error = -EBUSY;
		goto error;
	}

	if (val == 0) {
		f54->report_type = 0;
	} else {
		error = rmi_f54_request_report(f54->fn, val);
		if (error)
			dev_err(dev, "Error requesting F54 report %lu\n", val);

	}
error:
	mutex_unlock(&f54->status_mutex);
	return error < 0 ? error : count;
}

static DEVICE_ATTR(f54_report_type, S_IRUGO | S_IWUSR,
                   rmi_f54_report_type_show, rmi_f54_report_type_store);

static ssize_t rmi_f54_report_data_read(struct file *data_file,
                                        struct kobject *kobj,
                                        struct bin_attribute *attr,
                                        char *buf, loff_t pos, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct f54_data *f54 = dev_get_drvdata(dev);
	ssize_t ret;

	if (count == 0)
		return 0;

	mutex_lock(&f54->data_mutex);

	while (f54->is_busy) {
		mutex_unlock(&f54->data_mutex);
		/*
		 * transfer can take a long time, so wait for up to one second
		 */
		if (!wait_for_completion_timeout(&f54->cmd_done,
		                                 msecs_to_jiffies(1000)))
			return -ETIMEDOUT;
		mutex_lock(&f54->data_mutex);
	}
	if (f54->report_size == 0) {
		ret = -ENODATA;
		goto done;
	}

	if (pos + count > f54->report_size)
		count = f54->report_size - pos;

	memcpy(buf, f54->report_data + pos, count);
	ret = count;
done:
	mutex_unlock(&f54->data_mutex);
	return ret;
}

static struct bin_attribute f54_report_data = {
	.attr = {
		.name = "f54_report_data",
		.mode = S_IRUGO
	},
	.size = 0,
	.read = rmi_f54_report_data_read,
};

static ssize_t rmi_f54_query_data_read(struct file *data_file,
                                       struct kobject *kobj,
                                       struct bin_attribute *attr,
                                       char *buf, loff_t pos, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct f54_data *f54 = dev_get_drvdata(dev);

	if (count == 0)
		return 0;

	if (pos > sizeof(f54->qry))
		return -EFBIG;

	if (pos + count > sizeof(f54->qry))
		count = sizeof(f54->qry) - pos;

	memcpy(buf, ((char*)&f54->qry) + pos, count);
	return count;
}

static struct bin_attribute f54_query_data = {
	.attr = {
		.name = "f54_query_data",
		.mode = S_IRUGO
	},
	.size = 0,
	.read = rmi_f54_query_data_read,
};

static int rmi_f54_get_report_size(struct f54_data *f54)
{
	u8 rx = f54->qry.num_rx_electrodes ? : f54->qry.num_rx_electrodes;
	u8 tx = f54->qry.num_tx_electrodes ? : f54->qry.num_tx_electrodes;
	int size = 0;

	switch (f54->report_type) {
	case F54_8BIT_IMAGE:
		size = rx * tx;
		break;
	case F54_16BIT_IMAGE:
	case F54_RAW_16BIT_IMAGE:
	case F54_TRUE_BASELINE:
	case F54_FULL_RAW_CAP:
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
		size = 2 * rx * tx;
		break;
	case F54_HIGH_RESISTANCE:
		size = F54_HIGH_RESISTANCE_SIZE(rx, tx);
		break;
	case F54_FULL_RAW_CAP_MIN_MAX:
		size = F54_FULL_RAW_CAP_MIN_MAX_SIZE;
		break;
	case F54_TX_TO_TX_SHORT:
	case F54_TX_OPEN:
	case F54_TX_TO_GROUND:
		size =  (tx + 7) / 8;
		break;
	case F54_RX_TO_RX1:
	case F54_RX_OPENS1:
		if (rx < tx)
			size = 2 * rx * rx;
		else
			size = 2 * rx * tx;
		break;
	case F54_RX_TO_RX2:
	case F54_RX_OPENS2:
		if (rx <= tx)
			size = 0;
		else
			size = 2 * rx * (rx - tx);
		break;
	default:
		size = 0;
	}

	return size;
}

struct rmi_f54_reports {
	int start;
	int size;
};

static struct rmi_f54_reports rmi_f54_standard_report[] = {
	{ 0, 0 },           /* report size to be added in code */
	{ 0, 0 },
};

static void rmi_f54_work(struct work_struct *work)
{
	struct f54_data *f54 = container_of(work, struct f54_data, work.work);
	struct rmi_function *fn = f54->fn;
	u8 fifo[2];
	struct rmi_f54_reports *report;
	int report_size;
	u8 command;
	u8 *data;
	int error;

	data = f54->report_data;
	report_size = rmi_f54_get_report_size(f54);
	if (report_size == 0) {
		dev_err(&fn->dev, "Bad report size, report type=%d\n",
				f54->report_type);
		error = -EINVAL;
		goto error;     /* retry won't help */
	}
	rmi_f54_standard_report[0].size = report_size;
	report = rmi_f54_standard_report;

	mutex_lock(&f54->data_mutex);

	/*
	 * Need to check if command has completed.
	 * If not try again later.
	 */
	error = rmi_read(fn->rmi_dev, f54->fn->fd.command_base_addr,
	                 &command);
	if (error) {
		dev_err(&fn->dev, "Failed to read back command\n");
		goto error;
	}
	if (command & F54_GET_REPORT) {
		if (time_after(jiffies, f54->timeout)) {
			dev_err(&fn->dev, "Get report command timed out\n");
			error = -ETIMEDOUT;
		}
		report_size = 0;
		goto error;
	}

	rmi_dbg(RMI_DEBUG_FN, &fn->dev, "Get report command completed, reading data\n");

	report_size = 0;
	for (; report->size; report++) {
		fifo[0] = report->start & 0xff;
		fifo[1] = (report->start >> 8) & 0xff;
		error = rmi_write_block(fn->rmi_dev,
		                        fn->fd.data_base_addr + F54_FIFO_OFFSET,
		                        fifo, sizeof(fifo));
		if (error) {
			dev_err(&fn->dev, "Failed to set fifo start offset\n");
			goto abort;
		}

		error = rmi_read_block(fn->rmi_dev,
		                       fn->fd.data_base_addr + F54_REPORT_DATA_OFFSET,
		                       data, report->size);
		if (error) {
			dev_err(&fn->dev, "Report data read [%d bytes] returned %d\n",
			        report->size, error);
			goto abort;
		}
		data += report->size;
		report_size += report->size;
	}

abort:
	f54->report_size = error ? 0 : report_size;
error:
	if (error)
		report_size = 0;

	if (report_size == 0 && !error) {
		queue_delayed_work(f54->workqueue, &f54->work,
		                   msecs_to_jiffies(1));
	} else {
		f54->is_busy = false;
		complete(&f54->cmd_done);
	}

	mutex_unlock(&f54->data_mutex);
}

static int rmi_f54_attention(struct rmi_function *fn, unsigned long *irqbits)
{
	return 0;
}

static struct attribute *rmi_f54_attrs[] = {
	&dev_attr_f54_report_type.attr,
	&dev_attr_f54_control_addr.attr,
	&dev_attr_f54_control_data.attr,
	&dev_attr_f54_data_addr.attr,
	&dev_attr_f54_data_data.attr,
	&dev_attr_f54_command.attr,
	NULL
};

static struct attribute_group rmi_f54_attr_group = {
	.attrs          = rmi_f54_attrs,
};

static int rmi_f54_config(struct rmi_function *fn)
{
	struct rmi_driver *drv = fn->rmi_dev->driver;

	drv->set_irq_bits(fn->rmi_dev, fn->irq_mask);

	return 0;
}

static int rmi_f54_detect(struct rmi_function *fn)
{
	int error;
	struct f54_data *f54;

	f54 = dev_get_drvdata(&fn->dev);

	error = rmi_read_block(fn->rmi_dev, fn->fd.query_base_addr,
	                       &f54->qry, sizeof(f54->qry));
	if (error) {
		dev_err(&fn->dev, "%s: Failed to query F54 properties\n",
		        __func__);
		return error;
	}

	rmi_dbg(RMI_DEBUG_FN, &fn->dev, "F54 num_rx_electrodes: %d\n",
	        f54->qry.num_rx_electrodes);
	rmi_dbg(RMI_DEBUG_FN, &fn->dev, "F54 num_tx_electrodes: %d\n",
	        f54->qry.num_tx_electrodes);
	rmi_dbg(RMI_DEBUG_FN, &fn->dev, "F54 capabilities: 0x%x\n", f54->qry.f54_ad_query2);
	rmi_dbg(RMI_DEBUG_FN, &fn->dev, "F54 clock rate: 0x%x\n", f54->qry.clock_rate);
	rmi_dbg(RMI_DEBUG_FN, &fn->dev, "F54 family: 0x%x\n", f54->qry.family);

	f54->is_busy = false;

	return 0;
}

static int rmi_f54_probe(struct rmi_function *fn)
{
	struct f54_data *f54;
	int ret;
	u8 rx, tx;

	f54 = devm_kzalloc(&fn->dev, sizeof(struct f54_data), GFP_KERNEL);
	if (!f54)
		return -ENOMEM;

	f54->fn = fn;
	dev_set_drvdata(&fn->dev, f54);

	ret = rmi_f54_detect(fn);
	if (ret)
		return ret;

	mutex_init(&f54->data_mutex);
	mutex_init(&f54->status_mutex);

	rx = f54->qry.num_rx_electrodes;
	tx = f54->qry.num_tx_electrodes;
	f54->report_data = devm_kzalloc(&fn->dev,
	                                F54_MAX_REPORT_SIZE(rx, tx),
	                                GFP_KERNEL);
	if (f54->report_data == NULL)
		return -ENOMEM;

	INIT_DELAYED_WORK(&f54->work, rmi_f54_work);

	f54->workqueue = create_singlethread_workqueue("rmi4-poller");
	if (!f54->workqueue)
		return -ENOMEM;

	ret = sysfs_create_bin_file(&fn->dev.kobj, &f54_report_data);
	if (ret < 0)
		goto remove_wq;

	ret = sysfs_create_bin_file(&fn->dev.kobj, &f54_query_data);
	if (ret < 0)
		goto remove_report;

	ret = sysfs_create_bin_file(&fn->dev.kobj, &f54_control_data);
	if (ret < 0)
		goto remove_query;

	ret = sysfs_create_group(&fn->dev.kobj, &rmi_f54_attr_group);
	if (ret)
		goto remove_control;

	return 0;

remove_control:
	sysfs_remove_bin_file(&fn->dev.kobj, &f54_control_data);
remove_query:
	sysfs_remove_bin_file(&fn->dev.kobj, &f54_query_data);
remove_report:
	sysfs_remove_bin_file(&fn->dev.kobj, &f54_report_data);
remove_wq:
	cancel_delayed_work_sync(&f54->work);
	flush_workqueue(f54->workqueue);
	destroy_workqueue(f54->workqueue);
	return ret;
}

static void rmi_f54_remove(struct rmi_function *fn)
{
	sysfs_remove_group(&fn->dev.kobj, &rmi_f54_attr_group);
	sysfs_remove_bin_file(&fn->dev.kobj, &f54_control_data);
	sysfs_remove_bin_file(&fn->dev.kobj, &f54_report_data);
	sysfs_remove_bin_file(&fn->dev.kobj, &f54_query_data);
}

struct rmi_function_handler rmi_f54_handler = {
	.driver = {
		.name = F54_NAME,
	},
	.func = 0x54,
	.probe = rmi_f54_probe,
	.config = rmi_f54_config,
	.attention = rmi_f54_attention,
	.remove = rmi_f54_remove,
};
