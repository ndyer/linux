/*
 * Copyright (c) 2012-2015 Synaptics Incorporated
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#define DEBUG 1

#include <linux/kernel.h>
#include <linux/rmi.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>
#include "rmi_driver.h"

#define F54_NAME		"rmi4_f54"

#define F54_PERIODIC_SIZE       16
#define F54_PERIODIC_TIME       0       /* periodic timer, in seconds */
#define F54_RECAL_LIMIT         60      /* default recalibration limit */

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
	F54_MAX_REPORT_TYPE,
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

	/* periodic activity */
	struct mutex periodic_mutex; /* hold while periodic report pending */

	struct workqueue_struct *periodic;
	struct delayed_work periodic_work;

	bool is_periodic;	/* true while executing periodic report */
	u8 periodic_data[F54_PERIODIC_SIZE];
	int periodic_size;

	int worker_period;	/* rate of periodic activity, in seconds */
	int recal_limit;	/* limit to trigger recalibration */

	/* Data used for debugging */
	u8 control_addr;
	u8 data_addr;

	/* V4L2 support */
	struct v4l2_device v4l2;
	struct v4l2_pix_format format;
	struct video_device vdev;
	struct vb2_queue queue;
	struct mutex lock;
	int input;
	enum f54_report_type inputs[F54_MAX_REPORT_TYPE];
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

static enum f54_report_type rmi_f54_get_reptype(struct f54_data *f54,
						unsigned int i)
{
	if (i > F54_MAX_REPORT_TYPE)
		return F54_REPORT_NONE;

	return f54->inputs[i];
}

static void rmi_f54_create_input_map(struct f54_data *f54)
{
	int i = 0;
	enum f54_report_type reptype;

	for (reptype = 1; reptype < F54_MAX_REPORT_TYPE; reptype++) {
		if (!is_f54_report_type_valid(f54, reptype))
			continue;

		f54->inputs[i++] = reptype;
	}

	/* Remaining values are zero via kzalloc */
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

static ssize_t rmi_f54_recal_period_show(struct device *dev,
                                         struct device_attribute *attr,
                                         char *buf)
{
	struct f54_data *f54 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", f54->worker_period);
}

static ssize_t rmi_f54_recal_period_store(struct device *dev,
                                          struct device_attribute *attr,
                                          const char *buf, size_t count)
{
	struct f54_data *f54 = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val > 255)
		return -EINVAL;

	mutex_lock(&f54->periodic_mutex);
	f54->worker_period = val;

	cancel_delayed_work_sync(&f54->periodic_work);
	if (f54->worker_period)
		queue_delayed_work(f54->periodic, &f54->periodic_work,
		                   msecs_to_jiffies(f54->worker_period * 1000));

	mutex_unlock(&f54->periodic_mutex);

	return count;
}

static DEVICE_ATTR(f54_recal_period, S_IRUGO | S_IWUSR,
                   rmi_f54_recal_period_show, rmi_f54_recal_period_store);

static ssize_t rmi_f54_recal_limit_show(struct device *dev,
                                        struct device_attribute *attr,
                                        char *buf)
{
	struct f54_data *f54 = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", f54->recal_limit);
}

static ssize_t rmi_f54_recal_limit_store(struct device *dev,
                                         struct device_attribute *attr,
                                         const char *buf, size_t count)
{
	struct f54_data *f54 = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val > 65535)
		return -EINVAL;

	f54->recal_limit = val;

	return count;
}

static DEVICE_ATTR(f54_recal_limit, S_IRUGO | S_IWUSR,
                   rmi_f54_recal_limit_show, rmi_f54_recal_limit_store);

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

static const struct v4l2_file_operations rmi_f54_video_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,
};

struct rmi_f54_vb2_buffer {
	struct vb2_buffer       vb;
	struct list_head        list;
};

static int rmi_f54_queue_setup(struct vb2_queue *q,
                       unsigned int *nbuffers, unsigned int *nplanes,
                       unsigned int sizes[], void *alloc_ctxs[])
{
        struct f54_data *f54 = q->drv_priv;

        *nbuffers = 1;
        *nplanes = 1;
        sizes[0] = rmi_f54_get_report_size(f54);

        return 0;
}

static int rmi_f54_buffer_prepare(struct vb2_buffer *vb)
{
        return 0;
}

static void rmi_f54_buffer_queue(struct vb2_buffer *vb)
{
        struct f54_data *f54 = vb2_get_drv_priv(vb->vb2_queue);
        u16 *ptr;
	enum vb2_buffer_state state;
	enum f54_report_type reptype;
	int ret;

	mutex_lock(&f54->periodic_mutex);
	mutex_lock(&f54->status_mutex);

	reptype = rmi_f54_get_reptype(f54, f54->input);
	if (reptype == F54_REPORT_NONE) {
		state = VB2_BUF_STATE_ERROR;
		goto done;
	}

	if (f54->is_busy) {
		state = VB2_BUF_STATE_ERROR;
		goto done;
	}

	ret = rmi_f54_request_report(f54->fn, reptype);
	if (ret) {
		dev_err(&f54->fn->dev, "Error requesting F54 report\n");
		state = VB2_BUF_STATE_ERROR;
		goto done;
	}

	/* get frame data */
	mutex_lock(&f54->data_mutex);

	while (f54->is_busy) {
		mutex_unlock(&f54->data_mutex);
		if (!wait_for_completion_timeout(&f54->cmd_done,
						 msecs_to_jiffies(1000))) {
			dev_err(&f54->fn->dev, "Timed out\n");
			state = VB2_BUF_STATE_ERROR;
			goto done;
		}
		mutex_lock(&f54->data_mutex);
	}

	ptr = vb2_plane_vaddr(vb, 0);
	if (!ptr) {
		dev_err(&f54->fn->dev, "Error acquiring frame ptr\n");
		state = VB2_BUF_STATE_ERROR;
		goto data_done;
	}

	memcpy(ptr, f54->report_data, f54->report_size);
	vb2_set_plane_payload(vb, 0, rmi_f54_get_report_size(f54));
	state = VB2_BUF_STATE_DONE;

data_done:
	mutex_unlock(&f54->data_mutex);
done:
	vb2_buffer_done(vb, state);
	mutex_unlock(&f54->status_mutex);
	mutex_unlock(&f54->periodic_mutex);
}

/* V4L2 structures */
static const struct vb2_ops rmi_f54_queue_ops = {
	.queue_setup            = rmi_f54_queue_setup,
	.buf_prepare            = rmi_f54_buffer_prepare,
	.buf_queue              = rmi_f54_buffer_queue,
	.wait_prepare           = vb2_ops_wait_prepare,
	.wait_finish            = vb2_ops_wait_finish,
};

static const struct vb2_queue rmi_f54_queue = {
	.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
	.io_modes = VB2_MMAP,
	.buf_struct_size = sizeof(struct rmi_f54_vb2_buffer),
	.ops = &rmi_f54_queue_ops,
	.mem_ops = &vb2_vmalloc_memops,
	.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
	.min_buffers_needed = 1,
};

static int rmi_f54_vidioc_querycap(struct file *file, void *priv,
                                 struct v4l2_capability *cap)
{
	struct f54_data *f54 = video_drvdata(file);

	strlcpy(cap->driver, F54_NAME, sizeof(cap->driver));
	strlcpy(cap->card, SYNAPTICS_INPUT_DEVICE_NAME, sizeof(cap->card));
	strlcpy(cap->bus_info, dev_name(&f54->fn->dev), sizeof(cap->bus_info));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE |
		V4L2_CAP_READWRITE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static const char *rmi_f54_reptype_str(enum f54_report_type reptype)
{
	switch (reptype) {
	default:
	case F54_REPORT_NONE: return "No report";
	case F54_8BIT_IMAGE: return "8 bit image";
	case F54_16BIT_IMAGE: return "16 bit image";
	case F54_RAW_16BIT_IMAGE: return "Raw 16 bit image";
	case F54_HIGH_RESISTANCE: return "High resistance";
	case F54_TX_TO_TX_SHORT: return "TX to TX short";
	case F54_RX_TO_RX1: return "RX to RX1";
	case F54_TRUE_BASELINE: return "True baseline";
	case F54_FULL_RAW_CAP_MIN_MAX: return "Full raw cap min max";
	case F54_RX_OPENS1: return "RX open S1";
	case F54_TX_OPEN: return "TX open";
	case F54_TX_TO_GROUND: return "TX to ground";
	case F54_RX_TO_RX2: return "RX to RX2";
	case F54_RX_OPENS2: return "RX Open S2";
	case F54_FULL_RAW_CAP: return "Full raw cap";
	case F54_FULL_RAW_CAP_RX_COUPLING_COMP:
			       return "Full raw cap RX coupling comp";
	case F54_MAX_REPORT_TYPE: return "Max report type";
	}
}

static int rmi_f54_vidioc_enum_input(struct file *file, void *priv,
                                   struct v4l2_input *i)
{
	struct f54_data *f54 = video_drvdata(file);
	enum f54_report_type reptype;

	reptype = rmi_f54_get_reptype(f54, i->index);
	if (reptype == F54_REPORT_NONE)
		return -EINVAL;

	i->type = V4L2_INPUT_TYPE_CAMERA;
	i->std = V4L2_STD_UNKNOWN;
	i->capabilities = 0;
	strlcpy(i->name, rmi_f54_reptype_str(reptype), sizeof(i->name));
	return 0;
}

static int rmi_f54_set_input(struct f54_data *f54, unsigned int i)
{
	struct v4l2_pix_format *f = &f54->format;
	enum f54_report_type reptype;

	reptype = rmi_f54_get_reptype(f54, i);
	if (reptype == F54_REPORT_NONE)
		return -EINVAL;

	f54->input = i;

	f->width = f54->qry.num_rx_electrodes;
	f->height = f54->qry.num_tx_electrodes;
	f->pixelformat = V4L2_PIX_FMT_YS16;
	f->field = V4L2_FIELD_NONE;
	f->colorspace = V4L2_COLORSPACE_SRGB;
	f->bytesperline = f->width * sizeof(u16);
	f->sizeimage = f->width * f->height * sizeof(u16);

	return 0;
}

static int rmi_f54_vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	return rmi_f54_set_input(video_drvdata(file), i);
}

static int rmi_f54_vidioc_g_input(struct file *file, void *priv,
				  unsigned int *i)
{
	struct f54_data *f54 = video_drvdata(file);

	*i = f54->input;

	return 0;
}

static int rmi_f54_vidioc_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct f54_data *f54 = video_drvdata(file);

	f->fmt.pix = f54->format;

	return 0;
}

static int rmi_f54_vidioc_enum_fmt(struct file *file, void *priv,
                                 struct v4l2_fmtdesc *fmt)
{
	if (fmt->index > 0 || fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	fmt->pixelformat = V4L2_PIX_FMT_YS16;
	strlcpy(fmt->description, "16-bit raw debug data",
			sizeof(fmt->description));
	fmt->flags = 0;
	return 0;
}

static int rmi_f54_vidioc_enum_framesizes(struct file *file, void *priv,
                                        struct v4l2_frmsizeenum *f)
{
	struct f54_data *f54 = video_drvdata(file);

	if (f->index > 0)
		return -EINVAL;

	f->discrete.width = f54->qry.num_rx_electrodes;
	f->discrete.height = f54->qry.num_tx_electrodes;
	f->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	return 0;
}

static int rmi_f54_vidioc_enum_frameintervals(struct file *file, void *priv,
                                          struct v4l2_frmivalenum *f)
{
	if ((f->index > 0) || (f->pixel_format != V4L2_PIX_FMT_YS16))
		return -EINVAL;

	f->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	f->discrete.denominator  = 10;
	f->discrete.numerator = 1;
	return 0;
}

static const struct v4l2_ioctl_ops rmi_f54_video_ioctl_ops = {
	.vidioc_querycap        = rmi_f54_vidioc_querycap,

	.vidioc_enum_fmt_vid_cap = rmi_f54_vidioc_enum_fmt,
	.vidioc_s_fmt_vid_cap   = rmi_f54_vidioc_fmt,
	.vidioc_g_fmt_vid_cap   = rmi_f54_vidioc_fmt,

	.vidioc_enum_framesizes = rmi_f54_vidioc_enum_framesizes,
	.vidioc_enum_frameintervals = rmi_f54_vidioc_enum_frameintervals,

	.vidioc_enum_input      = rmi_f54_vidioc_enum_input,
	.vidioc_g_input         = rmi_f54_vidioc_g_input,
	.vidioc_s_input         = rmi_f54_vidioc_s_input,

	.vidioc_reqbufs         = vb2_ioctl_reqbufs,
	.vidioc_create_bufs     = vb2_ioctl_create_bufs,
	.vidioc_querybuf        = vb2_ioctl_querybuf,
	.vidioc_qbuf            = vb2_ioctl_qbuf,
	.vidioc_dqbuf           = vb2_ioctl_dqbuf,
	.vidioc_expbuf          = vb2_ioctl_expbuf,

	.vidioc_streamon        = vb2_ioctl_streamon,
	.vidioc_streamoff       = vb2_ioctl_streamoff,
};

static const struct video_device rmi_f54_video_device = {
	.name = "Synaptics RMI4",
	.fops = &rmi_f54_video_fops,
	.ioctl_ops = &rmi_f54_video_ioctl_ops,
	.release = video_device_release_empty,
};

/*
 *  rmi4_f54_periodic_work()
 */
static void rmi_f54_periodic_work(struct work_struct *work)
{
	struct f54_data *f54 =
	        container_of(work, struct f54_data,
	                     periodic_work.work);
	int error, i, sum;
	s16 *data;

	mutex_lock(&f54->periodic_mutex);
	f54->is_periodic = true;

	mutex_lock(&f54->status_mutex);
	if (f54->is_busy) {
		mutex_unlock(&f54->status_mutex);
		goto reschedule;
	}

	error = rmi_f54_request_report(f54->fn, F54_16BIT_IMAGE);
	if (error) {
		mutex_unlock(&f54->status_mutex);
		goto reschedule;
	}
	mutex_unlock(&f54->status_mutex);

	/* Now wait for results */

	mutex_lock(&f54->data_mutex);

	while (f54->is_busy) {
		mutex_unlock(&f54->data_mutex);
		/*
		 * If we did not receive a response within 200 ms,
		 * just try again later.
		 */
		if (!wait_for_completion_timeout(&f54->cmd_done,
		                                 msecs_to_jiffies(200))) {
			goto reschedule;
		}
		mutex_lock(&f54->data_mutex);
	}
	if (f54->periodic_size == 0)
		goto data_done;

	data = (s16 *)f54->periodic_data;
	sum = 0;
	for (i = 0; i < F54_PERIODIC_SIZE / 2; i++)
		sum += data[i];


	rmi_dbg(RMI_DEBUG_FN, &f54->fn->dev, "calibration data sum: %d\n", sum);
	if (sum > f54->recal_limit) {
		dev_info(&f54->fn->dev, "Touchscreen watchdog sum = %d Forcing recalibration.\n", sum);
		error = rmi_write(f54->fn->rmi_dev,
		                  f54->fn->fd.command_base_addr,
		                  F54_FORCE_CAL);
	}

data_done:
	mutex_unlock(&f54->data_mutex);
reschedule:
	queue_delayed_work(f54->periodic, &f54->periodic_work,
	                   msecs_to_jiffies(f54->worker_period * 1000));
	f54->is_periodic = false;
	mutex_unlock(&f54->periodic_mutex);
}

struct rmi_f54_reports {
	int start;
	int size;
};

static struct rmi_f54_reports rmi_f54_periodic_report_24[] = {
	{ 0   * 2, 4 },         /* four bytes each from four corners */
	{ 37  * 2, 4 },
	{ 896 * 2, 4 },
	{ 934 * 2, 4 },         /* This must be the last two blocks in the 936 block buffer */
	{ 0, 0 },
};

static struct rmi_f54_reports rmi_f54_periodic_report_30[] = {
	{ 0    * 2, 4 },        /* four bytes each from four corners */
	{ 44   * 2, 4 },
	{ 1421 * 2, 4 },
	{ 1378 * 2, 4 },        /* This must be the last two blocks in the 1380 block buffer */
	{ 0, 0 },
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

	if (f54->is_periodic) {
		data = f54->periodic_data;
		if (f54->qry.num_tx_electrodes <= 24)
			report = rmi_f54_periodic_report_24;
		else
			report = rmi_f54_periodic_report_30;
	} else {

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
	}

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
	if (!f54->is_periodic)
		f54->report_size = error ? 0 : report_size;
	else
		f54->periodic_size = error ? 0 : report_size;
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
	&dev_attr_f54_control_addr.attr,
	&dev_attr_f54_control_data.attr,
	&dev_attr_f54_data_addr.attr,
	&dev_attr_f54_data_data.attr,
	&dev_attr_f54_command.attr,
	&dev_attr_f54_recal_period.attr,
	&dev_attr_f54_recal_limit.attr,
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
	mutex_init(&f54->periodic_mutex);

	rx = f54->qry.num_rx_electrodes;
	tx = f54->qry.num_tx_electrodes;
	f54->report_data = devm_kzalloc(&fn->dev,
	                                F54_MAX_REPORT_SIZE(rx, tx),
	                                GFP_KERNEL);
	if (f54->report_data == NULL)
		return -ENOMEM;

	INIT_DELAYED_WORK(&f54->work, rmi_f54_work);
	INIT_DELAYED_WORK(&f54->periodic_work, rmi_f54_periodic_work);

	f54->worker_period = F54_PERIODIC_TIME;
	f54->recal_limit = F54_RECAL_LIMIT;

	f54->workqueue = create_singlethread_workqueue("rmi4-poller");
	if (!f54->workqueue)
		return -ENOMEM;

	f54->periodic = create_singlethread_workqueue("rmi4-periodic");
	if (!f54->periodic) {
		ret = -ENOMEM;
		goto remove_wq;
	}

	ret = sysfs_create_bin_file(&fn->dev.kobj, &f54_query_data);
	if (ret < 0)
		goto remove_periodic;

	ret = sysfs_create_bin_file(&fn->dev.kobj, &f54_control_data);
	if (ret < 0)
		goto remove_query;

	ret = sysfs_create_group(&fn->dev.kobj, &rmi_f54_attr_group);
	if (ret)
		goto remove_control;

	rmi_f54_create_input_map(f54);

        /* register video device */
        strlcpy(f54->v4l2.name, F54_NAME, sizeof(f54->v4l2.name));
        ret = v4l2_device_register(&fn->dev, &f54->v4l2);
        if (ret) {
                dev_err(&fn->dev, "Unable to register video dev.\n");
                goto remove_control;
        }

        /* initialize the queue */
        mutex_init(&f54->lock);
        f54->queue = rmi_f54_queue;
        f54->queue.drv_priv = f54;
        f54->queue.lock = &f54->lock;

        ret = vb2_queue_init(&f54->queue);
        if (ret)
                goto remove_v4l2;

        f54->vdev = rmi_f54_video_device;
        f54->vdev.v4l2_dev = &f54->v4l2;
        f54->vdev.lock = &f54->lock;
        f54->vdev.vfl_dir = VFL_DIR_RX;
        f54->vdev.queue = &f54->queue;
        video_set_drvdata(&f54->vdev, f54);

        ret = video_register_device(&f54->vdev, VFL_TYPE_TOUCH_SENSOR, -1);
        if (ret) {
                dev_err(&fn->dev, "Unable to register video subdevice.");
                goto remove_v4l2;
        }

	if (f54->worker_period)
		queue_delayed_work(f54->periodic, &f54->periodic_work,
		                   msecs_to_jiffies(f54->worker_period * 1000));

	return 0;

remove_v4l2:
	v4l2_device_unregister(&f54->v4l2);
remove_control:
	sysfs_remove_bin_file(&fn->dev.kobj, &f54_control_data);
remove_query:
	sysfs_remove_bin_file(&fn->dev.kobj, &f54_query_data);
remove_periodic:
	cancel_delayed_work_sync(&f54->periodic_work);
	flush_workqueue(f54->periodic);
	destroy_workqueue(f54->periodic);
remove_wq:
	cancel_delayed_work_sync(&f54->work);
	flush_workqueue(f54->workqueue);
	destroy_workqueue(f54->workqueue);
	return ret;
}

static void rmi_f54_remove(struct rmi_function *fn)
{
	struct f54_data *f54 = dev_get_drvdata(&fn->dev);

	video_unregister_device(&f54->vdev);
	v4l2_device_unregister(&f54->v4l2);

	sysfs_remove_group(&fn->dev.kobj, &rmi_f54_attr_group);
	sysfs_remove_bin_file(&fn->dev.kobj, &f54_control_data);
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
