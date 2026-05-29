/*
 * Copyright (c) 2025 Croxel, Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "qmi8660_bus.h"

int qmi8660_prep_reg_read_rtio_async(const struct qmi8660_bus *bus, uint8_t reg, uint8_t *buf,
				     size_t size, struct rtio_sqe **out)
{
	struct rtio *ctx = bus->rtio.ctx;
	struct rtio_iodev *iodev = bus->rtio.iodev;
	struct rtio_sqe *write_reg_sqe = rtio_sqe_acquire(ctx);
	struct rtio_sqe *read_buf_sqe = rtio_sqe_acquire(ctx);

	if (!write_reg_sqe || !read_buf_sqe) {
		rtio_sqe_drop_all(ctx);
		return -ENOMEM;
	}
	if (bus->is_spi) {
		reg |= 0x80;
	}

	rtio_sqe_prep_tiny_write(write_reg_sqe, iodev, RTIO_PRIO_NORM, &reg, 1, NULL);
	write_reg_sqe->flags |= RTIO_SQE_TRANSACTION;
	rtio_sqe_prep_read(read_buf_sqe, iodev, RTIO_PRIO_NORM, buf, size, NULL);

	/** Send back last SQE so it can be concatenated later. */
	if (out) {
		if (!bus->is_spi) {
			/* For I2C/I3C, we need to send a STOP after the read.
			 * and I3C has same macro values with I2C
			 */
			read_buf_sqe->iodev_flags |= RTIO_IODEV_I2C_STOP | RTIO_IODEV_I2C_RESTART;
		}
		*out = read_buf_sqe;
	}

	return 2;
}

int qmi8660_prep_reg_write_rtio_async(const struct qmi8660_bus *bus, uint8_t reg,
				      const uint8_t *buf, size_t size, struct rtio_sqe **out)
{
	struct rtio *ctx = bus->rtio.ctx;
	struct rtio_iodev *iodev = bus->rtio.iodev;
	struct rtio_sqe *write_reg_sqe = rtio_sqe_acquire(ctx);
	struct rtio_sqe *write_buf_sqe = rtio_sqe_acquire(ctx);

	if (!write_reg_sqe || !write_buf_sqe) {
		rtio_sqe_drop_all(ctx);
		return -ENOMEM;
	}

	/** More than 7 won't work with tiny-write */
	if (size > 7) {
		return -EINVAL;
	}

	rtio_sqe_prep_tiny_write(write_reg_sqe, iodev, RTIO_PRIO_NORM, &reg, 1, NULL);
	write_reg_sqe->flags |= RTIO_SQE_TRANSACTION;
	rtio_sqe_prep_tiny_write(write_buf_sqe, iodev, RTIO_PRIO_NORM, buf, size, NULL);

	/** Send back last SQE so it can be concatenated later. */
	if (out) {
		*out = write_buf_sqe;
	}

	return 2;
}

int qmi8660_reg_read_rtio(const struct qmi8660_bus *bus, uint8_t start, uint8_t *buf, int size)
{
	struct rtio *ctx = bus->rtio.ctx;
	struct rtio_cqe *cqe;
	int ret;

	ret = qmi8660_prep_reg_read_rtio_async(bus, start, buf, size, NULL);
	if (ret < 0) {
		return ret;
	}

	ret = rtio_submit(ctx, ret);
	if (ret) {
		return ret;
	}

	do {
		cqe = rtio_cqe_consume(ctx);
		if (cqe != NULL) {
			ret = cqe->result;
			rtio_cqe_release(ctx, cqe);
		}
	} while (cqe != NULL);

	return ret;
}

int qmi8660_reg_write_rtio(const struct qmi8660_bus *bus, uint8_t reg, const uint8_t *buf, int size)
{
	struct rtio *ctx = bus->rtio.ctx;
	struct rtio_cqe *cqe;
	int ret;

	ret = qmi8660_prep_reg_write_rtio_async(bus, reg, buf, size, NULL);
	if (ret < 0) {
		return ret;
	}

	ret = rtio_submit(ctx, ret);
	if (ret) {
		return ret;
	}

	do {
		cqe = rtio_cqe_consume(ctx);
		if (cqe != NULL) {
			ret = cqe->result;
			rtio_cqe_release(ctx, cqe);
		}
	} while (cqe != NULL);

	return ret;
}
