/*
 * Copyright (c) 2025 Croxel, Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_QMI8660_QMI8660_BUS_H_
#define ZEPHYR_DRIVERS_SENSOR_QMI8660_QMI8660_BUS_H_

#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>

struct qmi8660_bus {
	struct {
		struct rtio *ctx;
		struct rtio_iodev *iodev;
	} rtio;
	bool is_spi;
};

int qmi8660_prep_reg_read_rtio_async(const struct qmi8660_bus *bus, uint8_t reg, uint8_t *buf,
				     size_t size, struct rtio_sqe **out);

int qmi8660_prep_reg_write_rtio_async(const struct qmi8660_bus *bus, uint8_t reg,
				      const uint8_t *buf, size_t size, struct rtio_sqe **out);

int qmi8660_reg_read_rtio(const struct qmi8660_bus *bus, uint8_t start, uint8_t *buf, int size);

int qmi8660_reg_write_rtio(const struct qmi8660_bus *bus, uint8_t reg, const uint8_t *buf,
			   int size);

#endif /* ZEPHYR_DRIVERS_SENSOR_QMI8660_QMI8660_BUS_H_ */
