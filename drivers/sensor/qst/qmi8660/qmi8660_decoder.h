/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_QMI8660_DECODER_H_
#define ZEPHYR_DRIVERS_SENSOR_QMI8660_DECODER_H_

#include <stdint.h>
#include <zephyr/drivers/sensor.h>
/* #include "qmi8660.h" */

struct qmi8660_decoder_header {
	const struct device *dev;
	uint64_t timestamp;
	uint8_t is_fifo: 1;
	uint8_t gyro_fs_reg: 3;
	uint8_t accel_fs_reg: 3;
	uint8_t variant: 1;
	/* struct alignment axis_align[3]; */
};

struct qmi8660_fifo_data {
	struct qmi8660_decoder_header header;
	uint8_t int_status[4];
	uint8_t gyro_odr_reg: 4;
	uint8_t accel_odr_reg: 4;
	uint16_t fifo_bytes: 12;
	uint16_t padding1: 4;
};

struct qmi8660_encoded_data {
	struct qmi8660_decoder_header header;
	struct {
		uint8_t channels: 7;
		uint8_t reserved: 1;
	} __attribute__((__packed__));
	int16_t readings[7];
};

int qmi8660_encode(const struct device *dev, const struct sensor_chan_spec *const channels,
		   const size_t num_channels, uint8_t *buf);

int qmi8660_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder);

#endif /* ZEPHYR_DRIVERS_SENSOR_QMI8660_DECODER_H_ */
