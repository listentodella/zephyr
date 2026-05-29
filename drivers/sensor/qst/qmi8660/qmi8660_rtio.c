/*
 * Copyright (c) 2023 Google LLC
 * Copyright (c) 2024 Croxel Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* #include <zephyr/drivers/sensor.h> */
#include <zephyr/rtio/work.h>

#include "qmi8660.h"
#include "qmi8660_decoder.h"
/* #include "qmi8660_reg.h" */
#include "qmi8660_rtio.h"
/* #include "qmi8660_spi.h" */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(QMI8660_RTIO, CONFIG_SENSOR_LOG_LEVEL);

static int qmi8660_rtio_sample_fetch(const struct device *dev, int16_t readings[7])
{
	/* LOG_INF("%s", __func__); */
	uint8_t status;
	/* const struct qmi8660_dev_cfg *cfg = dev->config; */
	const struct qmi8660_dev_data *data = dev->data;
	const struct qmi8660_bus *bus = &data->bus;
	uint8_t *buffer = (uint8_t *)readings;

	qmi8660_reg_read_rtio(bus, QMI8660_UI_INT_STATUS0, &status, 1);

	qmi8660_read_raw(dev, buffer);

	for (int i = 0; i < 7; i++) {
		/* readings[i] = sys_le16_to_cpu((buffer[i * 2]) | buffer[i * 2 + 1] << 8); */
		readings[i] = (int16_t)sys_get_le16(&buffer[i * 2]);
	}

	return 0;
}

void qmi8660_submit_one_shot_sync(struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;
	const struct device *dev = cfg->sensor;
	const struct sensor_chan_spec *const channels = cfg->channels;
	const size_t num_channels = cfg->count;
	uint32_t min_buf_len = sizeof(struct qmi8660_encoded_data);
	int rc;
	uint8_t *buf;
	uint32_t buf_len;
	struct qmi8660_encoded_data *edata;
	/* LOG_INF("%s", __func__); */
	/* Get the buffer for the frame, it may be allocated dynamically by the rtio context */
	rc = rtio_sqe_rx_buf(iodev_sqe, min_buf_len, min_buf_len, &buf, &buf_len);
	if (rc != 0) {
		LOG_ERR("Failed to get a read buffer of size %u bytes", min_buf_len);
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	edata = (struct qmi8660_encoded_data *)buf;

	rc = qmi8660_encode(dev, channels, num_channels, buf);
	if (rc != 0) {
		LOG_ERR("Failed to encode sensor data");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	rc = qmi8660_rtio_sample_fetch(dev, edata->readings);
	/* Check that the fetch succeeded */
	if (rc != 0) {
		LOG_ERR("Failed to fetch samples");
		rtio_iodev_sqe_err(iodev_sqe, rc);
		return;
	}

	rtio_iodev_sqe_ok(iodev_sqe, 0);
}

static void qmi8660_submit_one_shot(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	struct rtio_work_req *req = rtio_work_req_alloc();
	/* LOG_INF("%s", __func__); */
	if (req == NULL) {
		LOG_ERR("RTIO work item allocation failed. Consider to increase "
			"CONFIG_RTIO_WORKQ_POOL_ITEMS.");
		rtio_iodev_sqe_err(iodev_sqe, -ENOMEM);
		return;
	}

	rtio_work_req_submit(req, iodev_sqe, qmi8660_submit_one_shot_sync);
}

void qmi8660_submit(const struct device *dev, struct rtio_iodev_sqe *iodev_sqe)
{
	const struct sensor_read_config *cfg = iodev_sqe->sqe.iodev->data;

	if (!cfg->is_streaming) {
		qmi8660_submit_one_shot(dev, iodev_sqe);
	} else if (IS_ENABLED(CONFIG_QMI8660_STREAM)) {
		qmi8660_submit_stream(dev, iodev_sqe);
	} else {
		rtio_iodev_sqe_err(iodev_sqe, -ENOTSUP);
	}
}
