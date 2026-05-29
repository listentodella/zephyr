/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
#include <zephyr/sys/check.h>
#include <zephyr/drivers/sensor_clock.h>

#include "qmi8660.h"
#include "qmi8660_decoder.h"
/* #include "qmi8660_reg.h" */
#include "qmi8660_rtio.h"
#include "qmi8660_bus.h"

LOG_MODULE_DECLARE(QMI8660_RTIO, CONFIG_SENSOR_LOG_LEVEL);

void qmi8660_submit_stream(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct qmi8660_dev_cfg *dev_cfg = dev->config;
	struct qmi8660_dev_data *dev_data = dev->data;

	struct qmi8660_runtime_cfg new_config = dev_data->rt_cfg;

	new_config.int1_ctl0.bits.drdy_acc_en = false;
	new_config.int1_ctl0.bits.drdy_gyro_en = false;
	new_config.int1_ctl0.bits.fifo_wtm = false;
	new_config.int1_ctl0.bits.fifo_full = false;
	for (int i = 0; i < cfg->count; ++i) {
		switch (cfg->triggers[i].trigger) {
		case SENSOR_TRIG_DATA_READY:
			/* LOG_ERR("drdy needed"); */
			new_config.int1_ctl0.bits.drdy_acc_en = true;
			new_config.int1_ctl0.bits.drdy_gyro_en = true;
			break;
		case SENSOR_TRIG_FIFO_WATERMARK:
			/* LOG_ERR("fifo WTM needed"); */
			new_config.fifo_mode = QMI8660_FIFO_FIFO_MODE;
			new_config.fifo_wtm = 1;
			new_config.int1_ctl0.bits.fifo_wtm = true;
			break;
		case SENSOR_TRIG_FIFO_FULL:
			/* LOG_ERR("fifo FUL needed"); */
			new_config.fifo_wtm = 1;
			new_config.fifo_mode = QMI8660_FIFO_FIFO_MODE;
			new_config.int1_ctl0.bits.fifo_full = true;
			break;
		case SENSOR_TRIG_MOTION:
			/* LOG_ERR("Motion trigger needed"); */
			new_config.int1_ctl1.bits.mot_b_en = true;
			break;
		default:
			LOG_ERR("Trigger (%d) not supported", cfg->triggers[i].trigger);
			rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
			return;
		}
	}

	/* How to determine accel_en or gyro_en ? */
	/* according to ODR ? */
	new_config.acc_en = true;
	new_config.gyr_en = true;

	/* if (new_config.interrupt1_drdy != data->cfg.interrupt1_drdy || */
	/* new_config.interrupt1_fifo_ths != data->cfg.interrupt1_fifo_ths || */
	/* new_config.interrupt1_fifo_full != data->cfg.interrupt1_fifo_full) { */
	int rc = qmi8660_safely_configure(dev, &new_config);

	if (rc != 0) {
		LOG_ERR("%p Failed to configure sensor", dev);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}
	/* uint8_t enctl = 0; */
	/* qmi8660_reg_read(dev, QMI8660_UI_ENCTL, &enctl, sizeof(enctl)); */
	/* LOG_ERR("enctl = 0x%02x", enctl); */

	/* #if DUMP_ENABLE */
	/* qmi8660_dump(dev); */
	/* #endif */
	/* } */

	(void)atomic_set(&dev_data->state, QMI8660_STREAM_ON);
	dev_data->streaming_sqe = iodev_sqe;
	(void)gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

static struct sensor_stream_trigger *
qmi8660_get_read_config_trigger(const struct sensor_read_config *cfg, enum sensor_trigger_type trig)
{
	for (int i = 0; i < cfg->count; ++i) {
		if (cfg->triggers[i].trigger == trig) {
			return &cfg->triggers[i];
		}
	}
	LOG_DBG("Unsupported trigger (%d)", trig);
	return NULL;
}

static inline void qmi8660_stream_result(const struct device *dev, int result)
{
	/* LOG_INF("%s", __func__); */
	struct qmi8660_dev_data *data = dev->data;
	struct rtio_iodev_sqe *streaming_sqe = data->streaming_sqe;

	data->streaming_sqe = NULL;
	if (streaming_sqe == NULL) {
		LOG_ERR("%p Stream completion has no active SQE (result=%d)", dev, result);
		return;
	}

	if (result < 0) {
		rtio_iodev_sqe_err(streaming_sqe, result);
	} else {
		rtio_iodev_sqe_ok(streaming_sqe, result);
	}
}

static void qmi8660_complete_cb(struct rtio *r, const struct rtio_sqe *sqe, int result, void *arg)
{
	/* LOG_INF("%s", __func__); */

	const struct device *dev = arg;
	const struct qmi8660_dev_cfg *dev_cfg = (const struct qmi8660_dev_cfg *)dev->config;

	qmi8660_stream_result(dev, result);
	gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}

static void qmi8660_int_status_cb(struct rtio *r, const struct rtio_sqe *sqe, int result, void *arg)
{
	ARG_UNUSED(result);
	int rc = 0;
	uint8_t *buf;
	uint32_t buf_len;
	size_t required_len = 0;
	struct rtio_sqe *nsqe;
	const struct device *dev = arg;
	struct qmi8660_dev_data *dev_data = dev->data;
	const struct qmi8660_dev_cfg *dev_cfg = (const struct qmi8660_dev_cfg *)dev->config;
	struct rtio_iodev_sqe *streaming_sqe = dev_data->streaming_sqe;
	struct qmi8660_fifo_data *fdata = NULL;

	if (dev_data->streaming_sqe == NULL ||
	    FIELD_GET(RTIO_SQE_CANCELED, dev_data->streaming_sqe->sqe.flags)) {
		LOG_ERR("%p Complete CB triggered with NULL handle. Disabling Interrupt", dev);
		(void)gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio, GPIO_INT_DISABLE);
		(void)atomic_set(&dev_data->state, QMI8660_STREAM_OFF);
		return;
	}

	struct sensor_read_config *read_config =
		(struct sensor_read_config *)dev_data->streaming_sqe->sqe.iodev->data;
	struct sensor_stream_trigger *fifo_wtm_cfg =
		qmi8660_get_read_config_trigger(read_config, SENSOR_TRIG_FIFO_WATERMARK);
	struct sensor_stream_trigger *fifo_full_cfg =
		qmi8660_get_read_config_trigger(read_config, SENSOR_TRIG_FIFO_FULL);

	bool has_fifo_wm_trig =
		fifo_wtm_cfg && FIELD_GET(BIT_FIFO_WTM_INT, dev_data->int_status[0]) == true;
	bool has_fifo_full_trig =
		fifo_full_cfg && FIELD_GET(BIT_FIFO_FULL_INT, dev_data->int_status[0]) == true;
	if (!has_fifo_wm_trig && !has_fifo_full_trig) {
		gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		return;
	}

	/* flush cqe */
	struct rtio_cqe *cqe;

	do {
		cqe = rtio_cqe_consume(dev_data->bus.rtio.ctx);
		if (cqe != NULL) {
			if (rc >= 0) {
				rc = cqe->result;
			}
			rtio_cqe_release(dev_data->bus.rtio.ctx, cqe);
		}
	} while (cqe != NULL);

	enum sensor_stream_data_opt data_opt;

	if (has_fifo_wm_trig && !has_fifo_full_trig) {
		data_opt = fifo_wtm_cfg->opt;
	} else if (has_fifo_full_trig && !has_fifo_wm_trig) {
		data_opt = fifo_full_cfg->opt;
	} else {
		data_opt = MAX(fifo_wtm_cfg->opt, fifo_full_cfg->opt);
	}

	if (data_opt == SENSOR_STREAM_DATA_NOP || data_opt == SENSOR_STREAM_DATA_DROP) {
		required_len = sizeof(struct qmi8660_fifo_data);
		rc = rtio_sqe_rx_buf(streaming_sqe, required_len, required_len, &buf, &buf_len);
		CHECKIF(rc < 0 || !buf) {
			LOG_ERR("%p Failed to obtain SQE buffer: %d", dev, rc);
			qmi8660_stream_result(dev, -ENOMEM);
			return;
		}
		fdata = (struct qmi8660_fifo_data *)buf;
		memset(fdata, 0, sizeof(*fdata));
		fdata->header.dev = dev;
		fdata->header.is_fifo = true;
		fdata->header.timestamp = dev_data->timestamp;
		fdata->fifo_bytes = 0; /* since we will drop it. */
		fdata->accel_odr_reg = dev_data->rt_cfg.acc_odr.reg_val;
		fdata->gyro_odr_reg = dev_data->rt_cfg.gyr_odr.reg_val;
		memcpy(fdata->int_status, dev_data->int_status, sizeof(dev_data->int_status));

		LOG_ERR("%p DATA_NOP/DROP?", dev);

		gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		if (data_opt == SENSOR_STREAM_DATA_DROP) {
			uint8_t off = 0x00;

			LOG_WRN("%p FIFO FULL! Flushing FIFO...", dev);
			rc = qmi8660_prep_reg_write_rtio_async(&dev_data->bus, QMI8660_UI_FIFO_CTL1,
							       &off, 1, &nsqe);
			CHECKIF(rc < 0 || !nsqe) {
				LOG_ERR("%p Failed to set FIFO OFF: %d", dev, rc);
				qmi8660_stream_result(dev, rc);
				return;
			}
			nsqe->flags |= RTIO_SQE_CHAINED;

			uint8_t on = 0x40 | 0x08;

			rc = qmi8660_prep_reg_write_rtio_async(&dev_data->bus, QMI8660_UI_FIFO_CTL1,
							       &on, 1, &nsqe);
			CHECKIF(rc < 0) {
				LOG_ERR("%p Failed to set FIFO ON: %d", dev, rc);
				qmi8660_stream_result(dev, rc);
				return;
			}
			if (rc < 0 || !nsqe) {
				LOG_ERR("%p Could not prepare async read: %d", dev, rc);
				qmi8660_stream_result(dev, -ENOMEM);
				return;
			}
			nsqe->flags |= RTIO_SQE_CHAINED;
		}
		(void)atomic_set(&dev_data->state, QMI8660_STREAM_OFF);
		goto out;
		/* rtio_submit(dev_data->bus.rtio.ctx, 0); */

		/* return qmi8660_stream_result(dev, rc); */
	}

	/* otherwise, we need the FIFO contents */
	/* TODO: 根据 a/g/t 实际状态来计算 payload_read_len,以及各自的帧数 */
	dev_data->fifo_count = sys_le16_to_cpu(dev_data->fifo_count) & 0x0FFF;

	uint16_t payload_read_len = dev_data->fifo_count; /* total Bytes */

	required_len = sizeof(struct qmi8660_fifo_data) + payload_read_len;
	rc = rtio_sqe_rx_buf(streaming_sqe, required_len, required_len, &buf, &buf_len);
	if (rc < 0 || !buf) {
		LOG_ERR("%p Failed to allocate buffer for the FIFO read: %d", dev, rc);
		qmi8660_stream_result(dev, rc);
		return;
	}

	if (buf_len < required_len) {
		LOG_WRN("buf_len %d < required_len %d!!!", buf_len, required_len);
	}

	fdata = (struct qmi8660_fifo_data *)buf;
	memset(fdata, 0, sizeof(*fdata));
	fdata->header.dev = dev;
	fdata->header.is_fifo = true;
	fdata->header.timestamp = dev_data->timestamp;
	fdata->fifo_bytes = payload_read_len;
	fdata->accel_odr_reg = dev_data->rt_cfg.acc_odr.reg_val;
	fdata->gyro_odr_reg = dev_data->rt_cfg.gyr_odr.reg_val;
	memcpy(fdata->int_status, dev_data->int_status, sizeof(dev_data->int_status));

	uint8_t *read_buf = buf + sizeof(struct qmi8660_fifo_data);

	rc = qmi8660_prep_reg_read_rtio_async(&dev_data->bus, QMI8660_UI_FIFO_DATA, read_buf,
					      payload_read_len, &nsqe);
	if (rc < 0 || !nsqe) {
		LOG_ERR("%p Could not prepare async read: %d", dev, rc);
		qmi8660_stream_result(dev, -ENOMEM);
		return;
	}
	nsqe->flags |= RTIO_SQE_CHAINED;

out:
	struct rtio_sqe *cb_sqe = rtio_sqe_acquire(dev_data->bus.rtio.ctx);

	if (cb_sqe == NULL) {
		LOG_ERR("%p Failed to acquire callback SQE", dev);
		qmi8660_stream_result(dev, -ENOMEM);
		return;
	}

	/* rtio_sqe_prep_callback_no_cqe(cb_sqe, qmi8660_complete_cb, (void *)dev, NULL); */
	rtio_sqe_prep_callback(cb_sqe, qmi8660_complete_cb, (void *)dev, NULL);

	rtio_submit(dev_data->bus.rtio.ctx, 0);
}

void qmi8660_fifo_event(const struct device *dev)
{
	/* LOG_INF("%s", __func__); */
	int rc = 0;
	struct qmi8660_dev_data *dev_data = dev->data;
	const struct qmi8660_dev_cfg *dev_cfg = (const struct qmi8660_dev_cfg *)dev->config;
	struct rtio_sqe *sqe;
	uint64_t cycles;

	if (dev_data->streaming_sqe == NULL ||
	    FIELD_GET(RTIO_SQE_CANCELED, dev_data->streaming_sqe->sqe.flags)) {
		LOG_ERR("%p FIFO event triggered with no stream submisssion. Disabling IRQ", dev);
		(void)gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio, GPIO_INT_DISABLE);
		(void)atomic_set(&dev_data->state, QMI8660_STREAM_OFF);
		return;
	}
	if (atomic_cas(&dev_data->state, QMI8660_STREAM_ON, QMI8660_STREAM_BUSY) == false) {
		LOG_WRN("%p Callback triggered while stream is busy. Ignoring request", dev);
		return;
	}

	rc = sensor_clock_get_cycles(&cycles);
	if (rc != 0) {
		LOG_ERR("%p Failed to get sensor clock cycles", dev);
		qmi8660_stream_result(dev, rc);
		return;
	}
	dev_data->timestamp = sensor_clock_cycles_to_ns(cycles);

	rc = qmi8660_prep_reg_read_rtio_async(&dev_data->bus, QMI8660_UI_INT_STATUS0,
					      dev_data->int_status, 4, &sqe);
	CHECKIF(rc < 0 || !sqe) {
		LOG_ERR("%p Could not prepare async read: %d", dev, rc);
		qmi8660_stream_result(dev, -ENOMEM);
		return;
	}
	sqe->flags |= RTIO_SQE_CHAINED;

	rc = qmi8660_prep_reg_read_rtio_async(&dev_data->bus, QMI8660_UI_FIFO_STATUS_L,
					      (uint8_t *)&dev_data->fifo_count, 2, &sqe);
	CHECKIF(rc < 0 || !sqe) {
		LOG_ERR("%p Could not prepare async read: %d", dev, rc);
		qmi8660_stream_result(dev, -ENOMEM);
		return;
	}
	sqe->flags |= RTIO_SQE_CHAINED;

	struct rtio_sqe *cb_sqe = rtio_sqe_acquire(dev_data->bus.rtio.ctx);

	if (cb_sqe == NULL) {
		LOG_ERR("%p Failed to acquire callback SQE", dev);
		qmi8660_stream_result(dev, -ENOMEM);
		return;
	}

	/* rtio_sqe_prep_callback_no_cqe(cb_sqe, qmi8660_int_status_cb, (void *)dev, NULL); */
	rtio_sqe_prep_callback(cb_sqe, qmi8660_int_status_cb, (void *)dev, NULL);

	rtio_submit(dev_data->bus.rtio.ctx, 0);
}
