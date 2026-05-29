/*
 * Copyright (c) 2023 Google LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/sensor_clock.h>

#include "qmi8660_decoder.h"
/* #include "qmi8660_reg.h" */
#include "qmi8660.h"

LOG_MODULE_REGISTER(QMI8660_DECODER, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT qst_qmi8660

static inline void qmi8660_accel_ms(const struct device *dev, int32_t in, int32_t *out_ms,
				    int32_t *out_ums)
{
	struct sensor_value val = {0};
	const struct qmi8660_dev_data *dev_data = dev->data;

	qmi8660_convert_accel(&val, in, dev_data->rt_cfg.acc_fs.val);
	*out_ms = val.val1;
	*out_ums = val.val2;
}

static inline void qmi8660_gyro_rads(const struct device *dev, int32_t in, int32_t *out_rads,
				     int32_t *out_urads)
{
	struct sensor_value val = {0};
	const struct qmi8660_dev_data *dev_data = dev->data;

	qmi8660_convert_gyro(&val, in, dev_data->rt_cfg.gyr_fs.val);
	*out_rads = val.val1;
	*out_urads = val.val2;
}

static inline void qmi8660_temp_c(int32_t in, int32_t *out_c, int32_t *out_uc)
{
	struct sensor_value val = {0};

	qmi8660_convert_temp(&val, in);
	*out_c = val.val1;
	*out_uc = val.val2;
}

int qmi8660_convert_raw_to_q31(const struct device *dev, enum sensor_channel chan, int32_t reading,
			       q31_t *out, int8_t *shift_out)
{
	int32_t whole;
	int32_t fraction;
	int64_t intermediate;
	int8_t shift;
	struct qmi8660_dev_data *dev_data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
		shift = dev_data->rt_cfg.acc_fs.shift;
		qmi8660_accel_ms(dev, reading, &whole, &fraction);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
		shift = dev_data->rt_cfg.gyr_fs.shift;
		qmi8660_gyro_rads(dev, reading, &whole, &fraction);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		shift = 9; /* fixed for temperature */
		qmi8660_temp_c(reading, &whole, &fraction);
		break;
	default:
		return -ENOTSUP;
	}

	if (shift_out != NULL) {
		*shift_out = shift;
	}

	intermediate = ((int64_t)whole * INT64_C(1000000) + fraction);
	if (shift < 0) {
		intermediate =
			intermediate * ((int64_t)INT32_MAX + 1) * (1 << -shift) / INT64_C(1000000);
	} else {
		intermediate =
			intermediate * ((int64_t)INT32_MAX + 1) / ((1 << shift) * INT64_C(1000000));
	}
	*out = CLAMP(intermediate, INT32_MIN, INT32_MAX);

	return 0;
}

static int qmi8660_get_channel_position(enum sensor_channel chan)
{
	/* LOG_INF("%s", __func__); */
	switch (chan) {
	case SENSOR_CHAN_GYRO_XYZ:
	case SENSOR_CHAN_GYRO_X:
		return 0;
	case SENSOR_CHAN_GYRO_Y:
		return 1;
	case SENSOR_CHAN_GYRO_Z:
		return 2;
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_ACCEL_X:
		return 3;
	case SENSOR_CHAN_ACCEL_Y:
		return 4;
	case SENSOR_CHAN_ACCEL_Z:
		return 5;
	case SENSOR_CHAN_DIE_TEMP:
		return 6;
	default:
		return 0;
	}
}

static uint8_t qmi8660_encode_channel(enum sensor_channel chan)
{
	uint8_t encode_bmask = 0;
	/* LOG_INF("%s", __func__); */
	switch (chan) {
	case SENSOR_CHAN_DIE_TEMP:
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
		encode_bmask = BIT(qmi8660_get_channel_position(chan));
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		encode_bmask = BIT(qmi8660_get_channel_position(SENSOR_CHAN_ACCEL_X)) |
			       BIT(qmi8660_get_channel_position(SENSOR_CHAN_ACCEL_Y)) |
			       BIT(qmi8660_get_channel_position(SENSOR_CHAN_ACCEL_Z));
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		encode_bmask = BIT(qmi8660_get_channel_position(SENSOR_CHAN_GYRO_X)) |
			       BIT(qmi8660_get_channel_position(SENSOR_CHAN_GYRO_Y)) |
			       BIT(qmi8660_get_channel_position(SENSOR_CHAN_GYRO_Z));
		break;
	default:
		break;
	}

	return encode_bmask;
}

int qmi8660_encode(const struct device *dev, const struct sensor_chan_spec *const channels,
		   const size_t num_channels, uint8_t *buf)
{
	struct qmi8660_dev_data *data = dev->data;
	struct qmi8660_encoded_data *edata = (struct qmi8660_encoded_data *)buf;
	uint64_t cycles;
	int rc;
	/* LOG_INF("%s", __func__); */
	edata->channels = 0;

	for (int i = 0; i < num_channels; i++) {
		edata->channels |= qmi8660_encode_channel(channels[i].chan_type);
	}

	rc = sensor_clock_get_cycles(&cycles);
	if (rc != 0) {
		return rc;
	}

	edata->header.dev = dev;
	edata->header.is_fifo = false;
	edata->header.accel_fs_reg = data->rt_cfg.acc_fs.reg_val;
	edata->header.gyro_fs_reg = data->rt_cfg.gyr_fs.reg_val;
	/*
	 *	edata->header.variant = data->cfg.variant;
	 *	edata->header.axis_align[0] = data->cfg.axis_align[0];
	 *	edata->header.axis_align[1] = data->cfg.axis_align[1];
	 *	edata->header.axis_align[2] = data->cfg.axis_align[2];
	 */

	edata->header.timestamp = sensor_clock_cycles_to_ns(cycles);

	return 0;
}

#define IS_ACCEL(chan) ((chan) >= SENSOR_CHAN_ACCEL_X && (chan) <= SENSOR_CHAN_ACCEL_XYZ)
#define IS_GYRO(chan)  ((chan) >= SENSOR_CHAN_GYRO_X && (chan) <= SENSOR_CHAN_GYRO_XYZ)

static const uint64_t accel_period_ns[] = {
	[QMI8660_ODR_0_78125] = UINT64_C(100000000000000) / 78125,
	[QMI8660_ODR_1_5625] = UINT64_C(10000000000000) / 15625,
	[QMI8660_ODR_3_125] = UINT64_C(10000000000000) / 31250,
	[QMI8660_ODR_6_25] = UINT64_C(10000000000000) / 62500,
	[QMI8660_ODR_12_5] = UINT64_C(1000000000000) / 12500,
	[QMI8660_ODR_25] = UINT64_C(1000000000) / 25,
	[QMI8660_ODR_50] = UINT64_C(1000000000) / 50,
	[QMI8660_ODR_100] = UINT64_C(1000000000) / 100,
	[QMI8660_ODR_200] = UINT64_C(1000000000) / 200,
	[QMI8660_ODR_400] = UINT64_C(1000000000) / 400,
	[QMI8660_ODR_800] = UINT64_C(1000000000) / 800,
	[QMI8660_ODR_1600] = UINT64_C(10000000) / 16,
	[QMI8660_ODR_3200] = UINT64_C(10000000) / 32,
	[QMI8660_ODR_6400] = UINT64_C(10000000) / 64,
	[QMI8660_ODR_12800] = UINT64_C(10000000) / 128,
};

static const uint64_t gyro_period_ns[] = {
	[QMI8660_ODR_12_5] = UINT64_C(1000000000000) / 12500,
	[QMI8660_ODR_25] = UINT64_C(1000000000) / 25,
	[QMI8660_ODR_50] = UINT64_C(1000000000) / 50,
	[QMI8660_ODR_100] = UINT64_C(1000000000) / 100,
	[QMI8660_ODR_200] = UINT64_C(1000000000) / 200,
	[QMI8660_ODR_400] = UINT64_C(1000000000) / 400,
	[QMI8660_ODR_800] = UINT64_C(1000000000) / 800,
	[QMI8660_ODR_1600] = UINT64_C(10000000) / 16,
	[QMI8660_ODR_3200] = UINT64_C(10000000) / 32,
	[QMI8660_ODR_6400] = UINT64_C(10000000) / 64,
};

static int qmi8660_calc_timestamp_delta(int chan_type, int dt_odr, int frame_count,
					uint64_t *out_delta)
{
	/* LOG_INF("%s", __func__); */
	uint64_t period;

	if (IS_ACCEL(chan_type)) {
		period = accel_period_ns[dt_odr];
	} else if (IS_GYRO(chan_type)) {
		period = gyro_period_ns[dt_odr];
	} else {
		return -EINVAL;
	}

	/* *out_delta = period; */
	*out_delta = (uint64_t)period * frame_count;

	return 0;
}

static inline uint8_t qmi8660_get_frame_size(const struct qmi8660_fifo_data *fdata)
{
	/* TODO: 是否有必要通过电源寄存器判断开关状态 */
	bool has_gyro = (fdata->gyro_odr_reg != 0);
	bool has_accel = (fdata->accel_odr_reg != 0);

	if (!has_gyro && !has_accel) {
		return 14;
	}
	return (has_gyro ? 6 : 0) + (has_accel ? 6 : 0) + 2;
}

static int qmi8660_fifo_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
			       uint32_t *fit, uint16_t max_count, void *data_out)
{
	const struct qmi8660_fifo_data *fdata = (const struct qmi8660_fifo_data *)buffer;
	const uint8_t *buffer_end = buffer + sizeof(struct qmi8660_fifo_data) + fdata->fifo_bytes;
	const struct device *dev = fdata->header.dev;
	int accel_frame_count = 0;
	int gyro_frame_count = 0;
	int count = 0;

	if ((uintptr_t)buffer_end <= *fit || chan_spec.chan_idx != 0) {
		return 0;
	}

	((struct sensor_data_header *)data_out)->base_timestamp_ns = fdata->header.timestamp;
	/* skip header info */
	buffer += sizeof(struct qmi8660_fifo_data);
	const uint8_t frame_size = qmi8660_get_frame_size(fdata);
	const bool has_gyro = (fdata->gyro_odr_reg != 0);
	const bool has_accel = (fdata->accel_odr_reg != 0);

	while (count < max_count && buffer + frame_size <= buffer_end) {
		const uint8_t *frame_start = buffer;
		const uint8_t *frame_end = buffer + frame_size;

		/* if ((uintptr_t)frame_start < *fit) {
		 *		buffer = frame_end;
		 *		if (has_accel) accel_frame_count++;
		 *		if (has_gyro) gyro_frame_count++;
		 *		continue;
		 *	}
		 */
		if (has_accel) {
			accel_frame_count++;
		}
		if (has_gyro) {
			gyro_frame_count++;
		}

		if (chan_spec.chan_type == SENSOR_CHAN_DIE_TEMP) {
			struct sensor_q31_data *data = (struct sensor_q31_data *)data_out;
			uint64_t ts_delta = 0;
			uint8_t temp_offset = frame_size - 2;

			(void)qmi8660_calc_timestamp_delta(
				has_accel ? SENSOR_CHAN_ACCEL_XYZ : SENSOR_CHAN_GYRO_XYZ,
				has_accel ? fdata->accel_odr_reg : fdata->gyro_odr_reg,
				has_accel ? accel_frame_count - 1 : gyro_frame_count - 1,
				&ts_delta);
			data->readings[count].timestamp_delta = ts_delta;
			qmi8660_convert_raw_to_q31(dev, chan_spec.chan_type,
						   sys_get_le16(&frame_start[temp_offset]),
						   &data->readings[count].value, &data->shift);
		} else if (IS_ACCEL(chan_spec.chan_type) && has_accel) {
			struct sensor_three_axis_data *data =
				(struct sensor_three_axis_data *)data_out;
			uint64_t ts_delta = 0;
			uint8_t accel_offset = (has_gyro) ? 6 : 0;

			(void)qmi8660_calc_timestamp_delta(SENSOR_CHAN_ACCEL_XYZ,
							   fdata->accel_odr_reg,
							   accel_frame_count - 1, &ts_delta);
			data->readings[count].timestamp_delta = ts_delta;
			for (int i = 0; i < 3; i++) {
				qmi8660_convert_raw_to_q31(
					dev, chan_spec.chan_type,
					sys_get_le16(&frame_start[accel_offset + i * 2]),
					&data->readings[count].values[i], &data->shift);
			}
		} else if (IS_GYRO(chan_spec.chan_type) && has_gyro) {
			struct sensor_three_axis_data *data =
				(struct sensor_three_axis_data *)data_out;
			uint64_t ts_delta = 0;

			(void)qmi8660_calc_timestamp_delta(SENSOR_CHAN_GYRO_XYZ,
							   fdata->gyro_odr_reg,
							   gyro_frame_count - 1, &ts_delta);
			data->readings[count].timestamp_delta = ts_delta;
			for (int i = 0; i < 3; i++) {
				qmi8660_convert_raw_to_q31(
					dev, chan_spec.chan_type, sys_get_le16(&frame_start[i * 2]),
					&data->readings[count].values[i], &data->shift);
			}
		} else {
			/*
			 *	buffer = frame_end;
			 *  continue;
			 */
			CODE_UNREACHABLE;
		}

		buffer = frame_end;
		*fit = (uintptr_t)frame_end;
		count++;
	}

	((struct sensor_data_header *)data_out)->reading_count = count;

	return count;
}

static int qmi8660_one_shot_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				   uint32_t *fit, uint16_t max_count, void *data_out)
{
	/* LOG_INF("%s", __func__); */
	const struct qmi8660_encoded_data *edata = (const struct qmi8660_encoded_data *)buffer;
	const struct qmi8660_decoder_header *header = &edata->header;
	const struct device *dev = header->dev;
	/* const struct qmi8660_dev_data *dev_data = dev->data; */

	uint8_t channel_request;
	/* int rc; */

	if (*fit != 0) {
		return 0;
	}
	if (max_count == 0 || chan_spec.chan_idx != 0) {
		return -EINVAL;
	}
	struct sensor_q31_data *q31_out = data_out;
	struct sensor_three_axis_data *three_axis_out = data_out;

	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_DIE_TEMP: {
		channel_request = qmi8660_encode_channel(chan_spec.chan_type);
		if ((channel_request & edata->channels) != channel_request) {
			return -ENODATA;
		}

		q31_out->header.base_timestamp_ns = edata->header.timestamp;
		q31_out->header.reading_count = 1;

		qmi8660_convert_raw_to_q31(
			dev, chan_spec.chan_type,
			edata->readings[qmi8660_get_channel_position(chan_spec.chan_type)],
			&q31_out->readings[0].value, &q31_out->shift);
		*fit = 1;
		return 1;
	}
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_GYRO_XYZ: {
		channel_request = qmi8660_encode_channel(chan_spec.chan_type);
		if ((channel_request & edata->channels) != channel_request) {
			return -ENODATA;
		}

		three_axis_out->header.base_timestamp_ns = edata->header.timestamp;
		three_axis_out->header.reading_count = 1;

		qmi8660_convert_raw_to_q31(
			dev, chan_spec.chan_type - 3,
			edata->readings[qmi8660_get_channel_position(chan_spec.chan_type - 3)],
			&three_axis_out->readings[0].x, &three_axis_out->shift);
		qmi8660_convert_raw_to_q31(
			dev, chan_spec.chan_type - 2,
			edata->readings[qmi8660_get_channel_position(chan_spec.chan_type - 2)],
			&three_axis_out->readings[0].y, &three_axis_out->shift);
		qmi8660_convert_raw_to_q31(
			dev, chan_spec.chan_type - 1,
			edata->readings[qmi8660_get_channel_position(chan_spec.chan_type - 1)],
			&three_axis_out->readings[0].z, &three_axis_out->shift);
		*fit = 1;
		return 1;
	}
	default:
		return -EINVAL;
	}
}

static int qmi8660_decoder_decode(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
				  uint32_t *fit, uint16_t max_count, void *data_out)
{
	/* LOG_INF("%s", __func__); */
	const struct qmi8660_decoder_header *header = (const struct qmi8660_decoder_header *)buffer;

	if (header->is_fifo) {
		return qmi8660_fifo_decode(buffer, chan_spec, fit, max_count, data_out);
	}
	return qmi8660_one_shot_decode(buffer, chan_spec, fit, max_count, data_out);
}

static int qmi8660_decoder_get_frame_count(const uint8_t *buffer, struct sensor_chan_spec chan_spec,
					   uint16_t *frame_count)
{
	/* LOG_INF("%s", __func__); */
	const struct qmi8660_fifo_data *data = (const struct qmi8660_fifo_data *)buffer;
	const struct qmi8660_encoded_data *enc_data = (const struct qmi8660_encoded_data *)buffer;
	const struct qmi8660_decoder_header *header = &data->header;

	if (chan_spec.chan_idx != 0) {
		return -ENOTSUP;
	}

	uint8_t channel_request = qmi8660_encode_channel(chan_spec.chan_type);

	if ((!enc_data->header.is_fifo) &&
	    (enc_data->channels & channel_request) != channel_request) {
		return -ENODATA;
	}

	if (!header->is_fifo) {
		switch (chan_spec.chan_type) {
		case SENSOR_CHAN_ACCEL_X:
		case SENSOR_CHAN_ACCEL_Y:
		case SENSOR_CHAN_ACCEL_Z:
		case SENSOR_CHAN_ACCEL_XYZ:
		case SENSOR_CHAN_GYRO_X:
		case SENSOR_CHAN_GYRO_Y:
		case SENSOR_CHAN_GYRO_Z:
		case SENSOR_CHAN_GYRO_XYZ:
		case SENSOR_CHAN_DIE_TEMP:
			*frame_count = 1;
			return 0;
		default:
			return -ENOTSUP;
		}
		return 0;
	}

	/* Skip the header */
	buffer += sizeof(struct qmi8660_fifo_data);

	uint16_t count = 0;
	const uint8_t *end = buffer + data->fifo_bytes;
	const uint8_t frame_size = qmi8660_get_frame_size(data);

	if (data->fifo_bytes % frame_size != 0) {
		LOG_ERR("fifo_count = %d, frame_size = %d!", data->fifo_bytes, frame_size);
		/* return -EINVAL; */
	}

	/* count = data->fifo_count / frame_size; */
	while (buffer + frame_size <= end) {
		buffer += frame_size;
		++count;
	}

	*frame_count = count;
	return 0;
}

static int qmi8660_decoder_get_size_info(struct sensor_chan_spec chan_spec, size_t *base_size,
					 size_t *frame_size)
{
	LOG_INF("%s", __func__);
	switch (chan_spec.chan_type) {
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_GYRO_XYZ:
		*base_size = sizeof(struct sensor_three_axis_data);
		*frame_size = sizeof(struct sensor_three_axis_sample_data);
		return 0;
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_DIE_TEMP:
		*base_size = sizeof(struct sensor_q31_data);
		*frame_size = sizeof(struct sensor_q31_sample_data);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static bool qmi8660_decoder_has_trigger(const uint8_t *buffer, enum sensor_trigger_type trigger)
{
	const struct qmi8660_fifo_data *edata = (const struct qmi8660_fifo_data *)buffer;
	/* LOG_INF("%s", __func__); */
	if (!edata->header.is_fifo) {
		return false;
	}

	switch (trigger) {
	case SENSOR_TRIG_DATA_READY:
		return FIELD_GET(BIT_ACC_RDY_INT, edata->int_status[0]) ||
		       FIELD_GET(BIT_GYR_RDY_INT, edata->int_status[0]);
	case SENSOR_TRIG_FIFO_WATERMARK:
		return FIELD_GET(BIT_FIFO_WTM_INT, edata->int_status[0]);
	case SENSOR_TRIG_FIFO_FULL:
		return FIELD_GET(BIT_FIFO_FULL_INT, edata->int_status[0]);
	default:
		return false;
	}
}

SENSOR_DECODER_API_DT_DEFINE() = {
	.get_frame_count = qmi8660_decoder_get_frame_count,
	.get_size_info = qmi8660_decoder_get_size_info,
	.decode = qmi8660_decoder_decode,
	.has_trigger = qmi8660_decoder_has_trigger,
};

int qmi8660_get_decoder(const struct device *dev, const struct sensor_decoder_api **decoder)
{
	ARG_UNUSED(dev);
	*decoder = &SENSOR_DECODER_NAME();
	/* LOG_INF("%s", __func__); */
	return 0;
}
