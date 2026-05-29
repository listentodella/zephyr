/*
 * Copyright (c) 2026 QST.Corp
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <SEGGER_RTT.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <stdio.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>

LOG_MODULE_REGISTER(myapp);

#define LED3_R_NODE DT_ALIAS(led3r)
#define LED3_G_NODE DT_ALIAS(led3g)
#define LED3_B_NODE DT_ALIAS(led3b)
#define LED4_R_NODE DT_ALIAS(led4r)
#define LED4_G_NODE DT_ALIAS(led4g)
#define LED4_B_NODE DT_ALIAS(led4b)

static const struct gpio_dt_spec leds[] = {
	GPIO_DT_SPEC_GET(LED3_R_NODE, gpios), GPIO_DT_SPEC_GET(LED3_G_NODE, gpios),
	GPIO_DT_SPEC_GET(LED3_B_NODE, gpios), GPIO_DT_SPEC_GET(LED4_R_NODE, gpios),
	GPIO_DT_SPEC_GET(LED4_G_NODE, gpios), GPIO_DT_SPEC_GET(LED4_B_NODE, gpios)};

#define STREAMDEV_ALIAS(i) DT_ALIAS(_CONCAT(stream, i))
#define STREAMDEV_DEVICE(i, _)                                                                     \
	IF_ENABLED(DT_NODE_EXISTS(STREAMDEV_ALIAS(i)), (DEVICE_DT_GET(STREAMDEV_ALIAS(i)),))
#define NUM_SENSORS 1

/* support up to 10 sensors */
static const struct device *const sensors[] = {LISTIFY(10, STREAMDEV_DEVICE, ()) };

#define STREAM_IODEV_SYM(id)    CONCAT(stream_iodev, id)
#define STREAM_IODEV_PTR(id, _) &STREAM_IODEV_SYM(id)
#define STREAM_TRIGGERS                                                                            \
	{SENSOR_TRIG_FIFO_FULL, SENSOR_STREAM_DATA_DROP},                                          \
	{                                                                                          \
		SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE                             \
	}

#define STREAM_DEFINE_IODEV(id, _)                                                                 \
	SENSOR_DT_STREAM_IODEV(STREAM_IODEV_SYM(id), STREAMDEV_ALIAS(id), STREAM_TRIGGERS);

LISTIFY(NUM_SENSORS, STREAM_DEFINE_IODEV, (;));

#define ONESHOT_IODEV_SYM(id)    CONCAT(oneshot_iodev, id)
#define ONESHOT_IODEV_PTR(id, _) &ONESHOT_IODEV_SYM(id)
#define ONESHOT_CHANNELS                                                                           \
	{SENSOR_CHAN_ACCEL_XYZ, 0}, {SENSOR_CHAN_GYRO_XYZ, 0}, {SENSOR_CHAN_DIE_TEMP, 0}

#define ONESHOT_DEFINE_IODEV(id, _)                                                                \
	SENSOR_DT_READ_IODEV(ONESHOT_IODEV_SYM(id), STREAMDEV_ALIAS(id), ONESHOT_CHANNELS);

LISTIFY(NUM_SENSORS, ONESHOT_DEFINE_IODEV, (;));
/* for stream */
struct rtio_iodev *iodevs[NUM_SENSORS] = {LISTIFY(NUM_SENSORS, STREAM_IODEV_PTR, (,)) };
/* for oneshot */
/* struct rtio_iodev *iodevs[NUM_SENSORS] = {LISTIFY(NUM_SENSORS, ONESHOT_IODEV_PTR, (,)) }; */

RTIO_DEFINE_WITH_MEMPOOL(stream_ctx, NUM_SENSORS * 4, NUM_SENSORS * 4, NUM_SENSORS * 20, 256,
			 sizeof(void *));

struct sensor_chan_spec accel_chan = {SENSOR_CHAN_ACCEL_XYZ, 0};
struct sensor_chan_spec gyro_chan = {SENSOR_CHAN_GYRO_XYZ, 0};
struct sensor_chan_spec temp_chan = {SENSOR_CHAN_DIE_TEMP, 0};
struct sensor_chan_spec rot_vector_chan = {SENSOR_CHAN_GAME_ROTATION_VECTOR, 0};
struct sensor_chan_spec gravity_chan = {SENSOR_CHAN_GRAVITY_VECTOR, 0};
struct sensor_chan_spec gbias_chan = {SENSOR_CHAN_GBIAS_XYZ, 0};

#define TASK_STACK_SIZE 4096ul
static K_THREAD_STACK_ARRAY_DEFINE(thread_stack, NUM_SENSORS, TASK_STACK_SIZE);
static struct k_thread thread_id[NUM_SENSORS];

uint8_t accel_buf[1024] = {0};
uint8_t gyro_buf[1024] = {0};
uint8_t temp_buf[512] = {0};

static void print_stream(void *p1, void *p2, void *p3)
{
	const struct device *dev = (const struct device *)p1;
	struct rtio_iodev *iodev = (struct rtio_iodev *)p2;
	int rc = 0;
	const struct sensor_decoder_api *decoder;
	struct rtio_cqe *cqe;
	uint8_t *buf;
	uint32_t buf_len;
	struct rtio_sqe *handle;

	struct sensor_three_axis_data *accel_data = (struct sensor_three_axis_data *)accel_buf;
	struct sensor_three_axis_data *gyro_data = (struct sensor_three_axis_data *)gyro_buf;
	struct sensor_q31_data *temp_data = (struct sensor_q31_data *)temp_buf;

	/* Start the stream */
	LOG_INF("sensor_stream start");
	sensor_stream(iodev, &stream_ctx, NULL, &handle);

	while (1) {
		/*
		 * for one-shot
		 * k_msleep(1000);
		 * sensor_read_async_mempool(iodev, &stream_ctx, NULL);
		 */
		cqe = rtio_cqe_consume_block(&stream_ctx);
		if (cqe->result != 0) {
			LOG_ERR("async read failed %d", cqe->result);
			return;
		}

		rc = rtio_cqe_get_mempool_buffer(&stream_ctx, cqe, &buf, &buf_len);

		if (rc != 0) {
			LOG_ERR("get mempool buffer failed %d", rc);
			return;
		}

		const struct device *sensor = dev;

		rtio_cqe_release(&stream_ctx, cqe);

		rc = sensor_get_decoder(sensor, &decoder);

		if (rc != 0) {
			LOG_ERR("sensor_get_decoder failed %d", rc);
			return;
		}

		/* Frame iterator values when data comes from a FIFO */
		uint32_t accel_fit = 0, gyro_fit = 0;
		uint32_t temp_fit = 0;

		/* Number of sensor data frames */
		uint16_t xl_count, gy_count, tp_count;
		uint16_t frame_count;

		rc = decoder->get_frame_count(buf, accel_chan, &xl_count);
		rc += decoder->get_frame_count(buf, gyro_chan, &gy_count);
		rc += decoder->get_frame_count(buf, temp_chan, &tp_count);
		if (rc != 0) {
			LOG_ERR("sensor_get_frame failed %d", rc);
			return;
		}

		frame_count = xl_count + gy_count + tp_count;

		/* If a tap has occurred lets print it out */
		if (decoder->has_trigger(buf, SENSOR_TRIG_TAP)) {
			LOG_WRN("Tap! Sensor %s", dev->name);
		}

		int i = 0;

		while (i < frame_count) {
			int8_t c = 0;
			/* decode and print Accelerometer FIFO frames */
			c = decoder->decode(buf, accel_chan, &accel_fit, 8, accel_data);

			for (int k = 0; k < c; k++) {
				LOG_INF("A %s %lluns (%" PRIq(6) ", %" PRIq(6) ", %" PRIq(6) ")",
					dev->name, PRIsensor_three_axis_data_arg(*accel_data, k));
			}
			i += c;

			/* decode and print Gyroscope FIFO frames */
			c = decoder->decode(buf, gyro_chan, &gyro_fit, 8, gyro_data);

			for (int k = 0; k < c; k++) {
				LOG_INF("G %s %lluns (%" PRIq(6) ", %" PRIq(6) ", %" PRIq(6) ")",
					dev->name, PRIsensor_three_axis_data_arg(*gyro_data, k));
			}
			i += c;

			/* decode and print Temperature FIFO frames */
			c = decoder->decode(buf, temp_chan, &temp_fit, 8, temp_data);

			for (int k = 0; k < c; k++) {
				LOG_INF("T %s %lluns %s%d.%d °C", dev->name,
					PRIsensor_q31_data_arg(*temp_data, k));
			}
			i += c;
		}

		rtio_release_buffer(&stream_ctx, buf, buf_len);
	}
}

static void check_sensor_is_off(const struct device *dev)
{
	int ret;
	struct sensor_value odr;

	ret = sensor_attr_get(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	/* Check if accel is off */
	if (ret != 0 || (odr.val1 == 0 && odr.val2 == 0)) {
		LOG_WRN("%s WRN : accelerometer device is off", dev->name);
	}

	ret = sensor_attr_get(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
	/* Check if gyro is off */
	if (ret != 0 || (odr.val1 == 0 && odr.val2 == 0)) {
		LOG_WRN("%s WRN : gyroscope device is off", dev->name);
	}
}

int main(void)
{
	int ret = 0;
	uint8_t idx = 0;
	struct sensor_value gbias[3];
	char name[CONFIG_THREAD_MAX_NAME_LEN] = {0};

	SEGGER_RTT_Init();

	for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
		if (!device_is_ready(sensors[i])) {
			LOG_ERR("sensor: device %s not ready", sensors[i]->name);
			return 0;
		}
		check_sensor_is_off(sensors[i]);

		/*
		 * Set GBIAS as 0.5 rad/s, -1 rad/s, 0.2 rad/s
		 *
		 * (here application should initialize gbias x/y/z with latest values
		 * calculated from previous run and probably saved to non volatile memory)
		 */
		gbias[0].val1 = 0;
		gbias[0].val2 = 500000;
		gbias[1].val1 = -1;
		gbias[1].val2 = 0;
		gbias[2].val1 = 0;
		gbias[2].val2 = 200000;
		sensor_attr_set(sensors[i], SENSOR_CHAN_GBIAS_XYZ, SENSOR_ATTR_OFFSET, gbias);

		snprintf(name, sizeof(name), "rtio-%s", sensors[i]->name);
		k_tid_t tid = k_thread_create(&thread_id[i], thread_stack[i], TASK_STACK_SIZE,
					      print_stream, (void *)sensors[i], (void *)iodevs[i],
					      NULL, K_PRIO_PREEMPT(5), K_INHERIT_PERMS, K_FOREVER);
		k_thread_name_set(tid, name);
		k_thread_start(&thread_id[i]);
	}

	for (int i = 0; i < ARRAY_SIZE(leds); i++) {
		if (!gpio_is_ready_dt(&leds[i])) {
			LOG_ERR("led %d gpio not ready!", i);
			return 0;
		}

		ret = gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_ACTIVE);
		if (ret < 0) {
			LOG_ERR("Error configuring led %d gpio!", i);
			return 0;
		}
	}
	while (1) {
		gpio_pin_toggle_dt(&leds[idx++ % ARRAY_SIZE(leds)]);
		k_msleep(100);
	}
	return 0;
}
