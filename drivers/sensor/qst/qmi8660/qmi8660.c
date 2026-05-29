/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <zephyr/kernel.h>
#if defined(CONFIG_SHELL)
#include <zephyr/shell/shell.h>
#endif
#include <zephyr/sys/util.h>

#include "qmi8660.h"
#include "qmi8660_rtio.h"
#include "qmi8660_decoder.h"

LOG_MODULE_REGISTER(QMI8660, CONFIG_SENSOR_LOG_LEVEL);

static const struct qmi8660_map qmi8660_odr_map[] = {
	{12, 0x05},  {25, 0x06},  {50, 0x07},   {100, 0x08},  {200, 0x09},
	{400, 0x0A}, {800, 0x0B}, {1600, 0x0C}, {3200, 0x0D}, {6400, 0x0E},
};

static const struct qmi8660_map qmi8660_accel_fs_map[] = {
	{4, 0x00, 6}, {8, 0x01, 7}, {16, 0x02, 8}, {32, 0x03, 9}};

static const struct qmi8660_map qmi8660_gyro_fs_map[] = {
	{128, 0x02, 2}, {256, 0x03, 3}, {512, 0x04, 4}, {1024, 0x05, 5}, {2048, 0x06, 6}};

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
static int qmi8660_i2c_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct qmi8660_dev_cfg *dev_cfg = dev->config;

	return i2c_burst_read(dev_cfg->bus_dev, dev_cfg->bus_cfg.i2c_addr, reg, data, len);
}
static int qmi8660_i2c_write(const struct device *dev, uint8_t reg, const uint8_t *data,
			     uint16_t len)
{
	const struct qmi8660_dev_cfg *dev_cfg = dev->config;

	return i2c_burst_write(dev_cfg->bus_dev, dev_cfg->bus_cfg.i2c_addr, reg, data, len);
}
const struct qmi8660_bus_api qmi8660_bus_api_i2c = {
	.read = qmi8660_i2c_read,
	.write = qmi8660_i2c_write,
};
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
static int qmi8660_spi_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct qmi8660_dev_cfg *dev_cfg = dev->config;
	uint8_t tx_buf[1] = {reg | 0x80};
	const struct spi_buf tx = {.buf = tx_buf, .len = 1};
	const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};

	struct spi_buf rx[2] = {{.buf = NULL, .len = 1}, /* 跳过寄存器/dummy字节 */
				{.buf = data, .len = len}};
	const struct spi_buf_set rx_set = {.buffers = rx, .count = 2};

	return spi_transceive_dt(&dev_cfg->bus_cfg.spi_spec, &tx_set, &rx_set);
}

static int qmi8660_spi_write(const struct device *dev, uint8_t reg, const uint8_t *data,
			     uint16_t len)
{
	const struct qmi8660_dev_cfg *dev_cfg = dev->config;
	uint8_t tx_buf[1] = {reg & ~BIT(7)}; /* 清除读位 */
	const struct spi_buf tx_spi_bufs[2] = {{.buf = tx_buf, .len = 1},
					       {.buf = (uint8_t *)data, .len = len}};
	const struct spi_buf_set tx_set = {.buffers = tx_spi_bufs, .count = 2};

	return spi_write_dt(&dev_cfg->bus_cfg.spi_spec, &tx_set);
}

const struct qmi8660_bus_api qmi8660_bus_api_spi = {
	.read = qmi8660_spi_read,
	.write = qmi8660_spi_write,
};
#endif

int qmi8660_reg_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
	const struct qmi8660_dev_cfg *dev_cfg = dev->config;
	const struct qmi8660_dev_data *dev_data = dev->data;

	if (dev_data->bus.is_spi) {
		reg |= 0x80;
	}
	return dev_cfg->bus_api->read(dev, reg, data, len);
}

int qmi8660_reg_write(const struct device *dev, uint8_t reg, uint8_t val, uint32_t delay_us)
{
	const struct qmi8660_dev_cfg *dev_cfg = dev->config;
	uint8_t txbuf[2] = {reg, val};
	int ret;

	ret = dev_cfg->bus_api->write(dev, txbuf[0], &txbuf[1], 1);
	if (ret < 0) {
		return ret;
	}

	if (delay_us > 0) {
		k_busy_wait(delay_us);
	}

	return 0;
}

#if DUMP_ENABLE
void qmi8660_dump(const struct device *dev)
{
	int ret = 0, i = 0;
	uint8_t rxbuf[128] = {0};
	char buf[128] = {0};

	ret = qmi8660_reg_read(dev, 0x00, rxbuf, 128);
	if (ret < 0) {
		LOG_ERR("%s: failed to read register (%d)", __func__, ret);
		return;
	}

	LOG_INF("----------- dump start ----------");
	LOG_INF(" A\\F| 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F");
	for (i = 0; i < 0x80; i += 16) {
		snprintf(buf, sizeof(buf), "0x%02x|", i);
		for (int j = 0; j < 16; j++) {
			snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf), " %02x",
				 rxbuf[i + j]);
		}
		LOG_INF("%s", buf);
	}
	LOG_INF("----------- dump end ----------");
}
#endif

static int qmi8660_detect(const struct device *dev)
{
	int ret;
	uint8_t chip_id;
	int i;

	for (i = 0; i < 3; i++) {
		ret = qmi8660_reg_read(dev, QMI8660_UI_WHOAMI, &chip_id, 1);
		if (ret < 0) {
			continue;
		}
		LOG_INF("detect chipid val: 0x%x", chip_id);
		if (chip_id == QMI8660_WAI_VALUE) {
			return 0;
		}
	}
	return -ENODEV;
}

int qmi8660_clear_fifo(const struct device *dev)
{
	int ret = 0;

	ret = qmi8660_reg_write(dev, QMI8660_UI_FIFO_CTL1, 0x00, 50);
	if (ret < 0) {
		return ret;
	}

	ret = qmi8660_reg_write(dev, QMI8660_UI_FIFO_CTL1, 0x40 | 0x08, 50);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

static inline int qmi8660_set_fifo_mode(const struct device *dev, uint8_t mode)
{
	/* struct qmi8660_dev_data *dev_data = dev->data; */
	/* dev_data->rt_cfg.fifo_mode = mode; */
	return qmi8660_reg_write(dev, QMI8660_UI_FIFO_CTL1, mode << 6 | 0x08, 50);
}

/* 中断配置 - 适配 qmi8660_interrupt_cfg */
static int qmi8660_interrupt_cfg(const struct device *dev, uint8_t int1_cfg, uint8_t int2_cfg)
{
	int ret;

	ret = qmi8660_reg_write(dev, QMI8660_UI_INT1_CFG, int1_cfg, 50);
	if (ret < 0) {
		return ret;
	}

	ret = qmi8660_reg_write(dev, QMI8660_UI_INT2_CFG, int2_cfg, 50);
	if (ret < 0) {
		return ret;
	}

	return 0;
}

/* 中断控制 - 适配 qmi8660_interrupt_ctrl */
static int qmi8660_interrupt_ctrl(const struct device *dev, uint8_t int1_ctrl0, uint8_t int1_ctrl1,
				  uint8_t int2_ctrl0, uint8_t int2_ctrl1)
{
	int ret;

	ret = qmi8660_reg_write(dev, QMI8660_UI_INT1_CTL0, int1_ctrl0, 5);
	if (ret < 0) {
		return ret;
	}

	ret = qmi8660_reg_write(dev, QMI8660_UI_INT1_CTL1, int1_ctrl1, 5);
	if (ret < 0) {
		return ret;
	}

	ret = qmi8660_reg_write(dev, QMI8660_UI_INT2_CTL0, int2_ctrl0, 5);
	if (ret < 0) {
		return ret;
	}

	ret = qmi8660_reg_write(dev, QMI8660_UI_INT2_CTL1, int2_ctrl1, 5);
	if (ret < 0) {
		return ret;
	}

	return 0;
}
#ifdef CONFIG_QMI8660_TRIGGER
static int qmi8660_trigger_set(const struct device *dev, const struct sensor_trigger *trig,
			       sensor_trigger_handler_t handler)
{
	struct qmi8660_dev_data *dev_data = dev->data;
	const struct qmi8660_dev_cfg *cfg = dev->config;

	if (!cfg->int_gpio.port) {
		return -ENOTSUP;
	}

	if (trig->type != SENSOR_TRIG_DATA_READY) {
		return -ENOTSUP;
	}

	if ((trig->chan != SENSOR_CHAN_ALL) && (trig->chan != SENSOR_CHAN_ACCEL_XYZ) &&
	    (trig->chan != SENSOR_CHAN_GYRO_XYZ)) {
		return -ENOTSUP;
	}

	if (trig->type == SENSOR_TRIG_DATA_READY) {
		/* qmi8660_reg_write(dev, QMI8660_UI_INT1_CTL0, 0x03, 5); */
		/* qmi8660_reg_write(dev, QMI8660_UI_ENCTL, 0x03, 5); */
	}
	switch (trig->type) {
	case SENSOR_TRIG_DATA_READY:
	case SENSOR_TRIG_FIFO_WATERMARK:
	case SENSOR_TRIG_FIFO_FULL:
		break;
	default:
		return -ENOTSUP;
	}

	(void)gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_DISABLE);

	dev_data->drdy_handler = handler;
	if (handler == NULL) { /* means off */
		return 0;
	}

	dev_data->drdy_trigger = trig;

	return gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}
#if defined(CONFIG_QMI8660_TRIGGER_OWN_THREAD) || defined(CONFIG_QMI8660_TRIGGER_GLOBAL_THREAD)
static void qmi8660_thread_cb(const struct device *dev)
{
	struct qmi8660_dev_data *dev_data = dev->data;
	const struct qmi8660_dev_cfg *cfg = dev->config;

	if (dev_data->drdy_handler != NULL) {
		dev_data->drdy_handler(dev, dev_data->drdy_trigger);
	}

	(void)gpio_pin_interrupt_configure_dt(&cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}
#endif

static void qmi8660_gpio_callback(const struct device *port, struct gpio_callback *cb,
				  uint32_t pins)
{
	struct qmi8660_dev_data *dev_data = CONTAINER_OF(cb, struct qmi8660_dev_data, gpio_cb);
	const struct qmi8660_dev_cfg *dev_cfg = dev_data->dev->config;

	ARG_UNUSED(port);
	ARG_UNUSED(pins);

	/* (void)gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio, GPIO_INT_DISABLE); */

	if (IS_ENABLED(CONFIG_QMI8660_STREAM)) {
		if (dev_data->streaming_sqe != NULL) {
			qmi8660_fifo_event(dev_data->dev);
			return;
		}
	}

	if (dev_data->drdy_handler == NULL) {
		(void)gpio_pin_interrupt_configure_dt(&dev_cfg->int_gpio, GPIO_INT_EDGE_TO_ACTIVE);
		return;
	}

#if defined(CONFIG_QMI8660_TRIGGER_OWN_THREAD)
	k_sem_give(&dev_data->gpio_sem);
#elif defined(CONFIG_QMI8660_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&dev_data->work);
#endif
}

#ifdef CONFIG_QMI8660_TRIGGER_OWN_THREAD
static void qmi8660_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct qmi8660_dev_data *data = p1;

	while (1) {
		k_sem_take(&data->gpio_sem, K_FOREVER);
		qmi8660_thread_cb(data->dev);
	}
}
#endif

#ifdef CONFIG_QMI8660_TRIGGER_GLOBAL_THREAD
static void qmi8660_work_cb(struct k_work *work)
{
	struct qmi8660_dev_data *data = CONTAINER_OF(work, struct qmi8660_dev_data, work);

	qmi8660_thread_cb(data->dev);
}
#endif

static int qmi8660_init_interrupt(const struct device *dev)
{
	struct qmi8660_dev_data *data = dev->data;
	const struct qmi8660_dev_cfg *cfg = dev->config;
	int rc;

	if (!cfg->int_gpio.port) {
		return 0;
	}

	if (!gpio_is_ready_dt(&cfg->int_gpio)) {
		LOG_ERR("Interrupt GPIO device is not ready");
		return -ENODEV;
	}

	data->dev = dev;

	rc = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
	if (rc < 0) {
		LOG_ERR("Failed to configure interrupt GPIO (%d)", rc);
		return rc;
	}

	gpio_init_callback(&data->gpio_cb, qmi8660_gpio_callback, BIT(cfg->int_gpio.pin));

	rc = gpio_add_callback(cfg->int_gpio.port, &data->gpio_cb);
	if (rc < 0) {
		LOG_ERR("Failed to add interrupt callback (%d)", rc);
		return rc;
	}

#if defined(CONFIG_QMI8660_TRIGGER_OWN_THREAD)
	k_sem_init(&data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&data->thread, data->thread_stack, CONFIG_QMI8660_THREAD_STACK_SIZE,
			qmi8660_thread, data, NULL, NULL,
			K_PRIO_COOP(CONFIG_QMI8660_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&data->thread, "qmi8660_trigger");
#elif defined(CONFIG_QMI8660_TRIGGER_GLOBAL_THREAD)
	k_work_init(&data->work, qmi8660_work_cb);
#endif

	return 0;
}
#endif /* CONFIG_QMI8660_TRIGGER */

int qmi8660_configure(const struct device *dev, struct qmi8660_runtime_cfg *new_cfg)
{
	int ret = 0;
	struct qmi8660_dev_data *dev_data = dev->data;
	struct qmi8660_runtime_cfg curr_cfg = dev_data->rt_cfg;
	bool acc_odr_changed =
		memcmp(&curr_cfg.acc_odr, &new_cfg->acc_odr, sizeof(new_cfg->acc_odr)) != 0;
	bool acc_fs_changed =
		memcmp(&curr_cfg.acc_fs, &new_cfg->acc_fs, sizeof(new_cfg->acc_fs)) != 0;
	bool gyro_odr_changed =
		memcmp(&curr_cfg.gyr_odr, &new_cfg->gyr_odr, sizeof(new_cfg->gyr_odr)) != 0;
	bool gyro_fs_changed =
		memcmp(&curr_cfg.gyr_fs, &new_cfg->gyr_fs, sizeof(new_cfg->gyr_fs)) != 0;
	bool fifo_wtm_changed = curr_cfg.fifo_wtm != new_cfg->fifo_wtm;
	bool fifo_mode_changed = curr_cfg.fifo_mode != new_cfg->fifo_mode;
	bool accel_changed = acc_odr_changed || acc_fs_changed;
	bool gyro_changed = gyro_odr_changed || gyro_fs_changed;
	bool fifo_changed = fifo_wtm_changed || fifo_mode_changed;
	bool reset_fifo = (curr_cfg.fifo_mode != QMI8660_FIFO_BYPASS_MODE) &&
			  (accel_changed || gyro_changed || fifo_changed);

	/* disable all interrupts, reconfig at end */
	if (reset_fifo) {
		ret = qmi8660_set_fifo_mode(dev, QMI8660_FIFO_BYPASS_MODE);
		if (ret < 0) {
			LOG_ERR("Failed to disable FIFO: %d", ret);
			return ret;
		}

		ret = qmi8660_clear_fifo(dev);
		if (ret < 0) {
			LOG_ERR("Failed to clear FIFO: %d", ret);
			return ret;
		}
	}

	/* acc config */
	if (acc_odr_changed) {
		ret = qmi8660_reg_write(dev, QMI8660_UI_ACTL0, new_cfg->acc_odr.reg_val, 50);
		if (ret < 0) {
			return ret;
		}
	}
	if (acc_fs_changed) {
		ret = qmi8660_reg_write(dev, QMI8660_UI_ACTL1, new_cfg->acc_fs.reg_val, 50);
		if (ret < 0) {
			return ret;
		}
	}

	/* gyro config */
	if (gyro_odr_changed) {
		ret = qmi8660_reg_write(dev, QMI8660_UI_GCTL0, new_cfg->gyr_odr.reg_val, 50);
		if (ret < 0) {
			return ret;
		}
	}
	if (gyro_fs_changed) {
		ret = qmi8660_reg_write(dev, QMI8660_UI_GCTL1, new_cfg->gyr_fs.reg_val, 50);
		if (ret < 0) {
			return ret;
		}
	}

	/* fifo config */
	if (fifo_wtm_changed) {
		ret = qmi8660_reg_write(dev, QMI8660_UI_FIFO_WTM_TH_L, new_cfg->fifo_wtm & 0xFF,
					50);
		if (ret < 0) {
			return ret;
		}

		ret = qmi8660_reg_write(dev, QMI8660_UI_FIFO_CTL0,
					0xFC | ((new_cfg->fifo_wtm & 0x0300) >> 8), 50);
		if (ret < 0) {
			return ret;
		}
	}

	if (fifo_mode_changed || reset_fifo) {
		ret = qmi8660_set_fifo_mode(dev, new_cfg->fifo_mode);
		if (ret < 0) {
			LOG_ERR("Failed to set FIFO mode: %d", ret);
			return ret;
		}
	}

	/* cfg interrupts */
	if (memcmp(&curr_cfg.int1_ctl0, &new_cfg->int1_ctl0, sizeof(new_cfg->int1_ctl0)) ||
	    memcmp(&curr_cfg.int1_ctl1, &new_cfg->int1_ctl1, sizeof(new_cfg->int1_ctl1)) ||
	    memcmp(&curr_cfg.int2_ctl0, &new_cfg->int2_ctl0, sizeof(new_cfg->int2_ctl0)) ||
	    memcmp(&curr_cfg.int2_ctl1, &new_cfg->int2_ctl1, sizeof(new_cfg->int2_ctl1))) {
		qmi8660_interrupt_ctrl(dev, new_cfg->int1_ctl0.val, new_cfg->int1_ctl1.val,
				       new_cfg->int2_ctl0.val, new_cfg->int2_ctl1.val);
	}

	if (memcmp(&curr_cfg.int1_cfg, &new_cfg->int1_cfg, sizeof(new_cfg->int1_cfg)) ||
	    memcmp(&curr_cfg.int2_cfg, &new_cfg->int2_cfg, sizeof(new_cfg->int2_cfg))) {
		qmi8660_interrupt_cfg(dev, new_cfg->int1_cfg.val, new_cfg->int2_cfg.val);
	}

	/* power mode */
	if (curr_cfg.acc_en != new_cfg->acc_en || curr_cfg.gyr_en != new_cfg->gyr_en) {
		uint8_t enctl = 0;

		if (new_cfg->acc_en) {
			enctl |= 0x01;
		}
		if (new_cfg->gyr_en) {
			enctl |= 0x02;
		}
		ret = qmi8660_reg_write(dev, QMI8660_UI_ENCTL, enctl, 0);
		if (ret < 0) {
			LOG_ERR("Failed to set power mode: %d", ret);
			return ret;
		}
	}

	return ret;
}

int qmi8660_safely_configure(const struct device *dev, struct qmi8660_runtime_cfg *cfg)
{
	struct qmi8660_dev_data *dev_data = dev->data;
	int ret = qmi8660_configure(dev, cfg);

	if (!ret) {
		dev_data->rt_cfg = *cfg;
	} else {
		ret = qmi8660_configure(dev, &dev_data->rt_cfg);
	}

	return ret;
}

/* 主初始化函数 - 适配 qmi8660_setup */
static int qmi8660_setup(const struct device *dev)
{
	int ret;
	uint8_t mot_detect_mode, motion_odr, mot_holdoff;
	uint16_t mot_thr_x, mot_thr_y, mot_thr_z, mot_dur_sel;
	uint8_t mot_ctl[13];
	uint8_t ectl0, i;

	/* 复位传感器 */
	ret = qmi8660_reg_write(dev, QMI8660_UI_RESET, QMI8660_SFT_SYSTEM_KEY, 50000);
	if (ret < 0) {
		return ret;
	}

	/* Motion 检测参数配置 */
	mot_detect_mode = 0x57;
	motion_odr = 8;  /* motion odr 100hz */
	mot_thr_x = 100; /* threshold 100mg */
	mot_thr_y = 100;
	mot_thr_z = 100;
	mot_dur_sel = 1;
	mot_holdoff = 1;

	ectl0 = (motion_odr << 4);
	mot_ctl[0] = mot_detect_mode;
	mot_ctl[1] = 0x00;
	mot_ctl[2] = 0x00;
	mot_ctl[3] = 0x00;
	mot_ctl[4] = 0x00;
	mot_ctl[5] = 0x00;
	mot_ctl[6] = mot_thr_x & 0x0ff;
	mot_ctl[7] = mot_thr_y & 0x0ff;
	mot_ctl[8] = mot_thr_z & 0x0ff;
	mot_ctl[9] = ((mot_thr_y & 0xf00) >> 4) | ((mot_thr_x & 0xf00) >> 8);
	mot_ctl[10] = (mot_thr_z & 0xf00) >> 8;
	mot_ctl[11] = mot_dur_sel & 0x0ff;
	mot_ctl[12] = (mot_holdoff << 4) | ((mot_dur_sel & 0x300) >> 8);

	/* 写入 Motion 配置 */
	for (i = 0; i < 13; i++) {
		ret = qmi8660_reg_write(dev, QMI8660_UI_MOT_B_CTL0 + i, mot_ctl[i], 25);
		if (ret < 0) {
			return ret;
		}
	}

	/* 写入 Motion ODR */
	ret = qmi8660_reg_write(dev, QMI8660_UI_ECTL0, ectl0, 25);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_QMI8660_TRIGGER
	ret = qmi8660_init_interrupt(dev);
	if (ret < 0) {
		return ret;
	}
#endif

	return 0;
}

static bool qmi8660_is_accel_channel(enum sensor_channel chan)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		return true;
	default:
		return false;
	}
}

static bool qmi8660_is_gyro_channel(enum sensor_channel chan)
{
	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		return true;
	default:
		return false;
	}
}

static int qmi8660_select_val(uint16_t requested_val, uint16_t *selected_val, uint8_t *reg_val,
			      uint8_t *shift_out, const struct qmi8660_map *map, size_t map_size)
{
	uint32_t min_delta = UINT32_MAX;
	size_t min_index = 0;

	if (requested_val == 0U) {
		return -EINVAL;
	}

	for (size_t i = 0; i < map_size; i++) {
		uint16_t candidate_val = map[i].val;
		uint32_t delta = (candidate_val > requested_val) ? (candidate_val - requested_val)
								 : (requested_val - candidate_val);

		if (delta < min_delta) {
			min_delta = delta;
			min_index = i;
		}
	}

	*selected_val = map[min_index].val;
	*reg_val = map[min_index].reg_val;
	if (shift_out != NULL) {
		*shift_out = map[min_index].shift;
	}

	return 0;
}

static int qmi8660_select_odr(uint16_t requested_hz, uint16_t *selected_hz, uint8_t *reg_val)
{
	return qmi8660_select_val(requested_hz, selected_hz, reg_val, NULL, qmi8660_odr_map,
				  ARRAY_SIZE(qmi8660_odr_map));
}

static int qmi8660_select_fs(uint16_t requested_fs, uint16_t *selected_fs, uint8_t *reg_val,
			     uint8_t *shift_out, const struct qmi8660_map *map, size_t map_size)
{
	return qmi8660_select_val(requested_fs, selected_fs, reg_val, shift_out, map, map_size);
}

int qmi8660_get_shift(enum sensor_channel channel, int accel_fs_reg, int gyro_fs_reg, int variant,
		      int8_t *shift)
{
	ARG_UNUSED(variant);

	if (shift == NULL) {
		return -EINVAL;
	}

	switch (channel) {
	case SENSOR_CHAN_ACCEL_XYZ:
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z: {
		unsigned int idx = (unsigned int)accel_fs_reg;

		if (accel_fs_reg < 0 || idx >= ARRAY_SIZE(qmi8660_accel_fs_map)) {
			return -EINVAL;
		}
		if (qmi8660_accel_fs_map[idx].reg_val != (uint8_t)accel_fs_reg) {
			return -EINVAL;
		}
		*shift = (int8_t)qmi8660_accel_fs_map[idx].shift;
		return 0;
	}
	case SENSOR_CHAN_GYRO_XYZ:
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z: {
		const unsigned int gyro_reg_min = 0x02U;
		unsigned int idx;

		if (gyro_fs_reg < 0x02 || gyro_fs_reg > 0x06) {
			return -EINVAL;
		}
		idx = (unsigned int)gyro_fs_reg - gyro_reg_min;
		if (idx >= ARRAY_SIZE(qmi8660_gyro_fs_map)) {
			return -EINVAL;
		}
		if (qmi8660_gyro_fs_map[idx].reg_val != (uint8_t)gyro_fs_reg) {
			return -EINVAL;
		}
		*shift = (int8_t)qmi8660_gyro_fs_map[idx].shift;
		return 0;
	}
	case SENSOR_CHAN_DIE_TEMP:
		*shift = 9;
		return 0;
	default:
		return -ENOTSUP;
	}
}

int qmi8660_read_raw(const struct device *dev, uint8_t data[14])
{
	return qmi8660_reg_read(dev, QMI8660_UI_GX_L, data, 14);
}

static int qmi8660_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct qmi8660_dev_data *data = dev->data;
	uint8_t sample[14];
	int rc;

	if ((chan != SENSOR_CHAN_ALL) && !qmi8660_is_accel_channel(chan) &&
	    !qmi8660_is_gyro_channel(chan) && (chan != SENSOR_CHAN_DIE_TEMP)) {
		return -ENOTSUP;
	}

	rc = qmi8660_read_raw(dev, sample);
	if (rc < 0) {
		LOG_ERR("Failed to read sample data (%d)", rc);
		return rc;
	}

	data->gyro_x_raw = (int16_t)sys_get_le16(&sample[0]);
	data->gyro_y_raw = (int16_t)sys_get_le16(&sample[2]);
	data->gyro_z_raw = (int16_t)sys_get_le16(&sample[4]);
	data->accel_x_raw = (int16_t)sys_get_le16(&sample[6]);
	data->accel_y_raw = (int16_t)sys_get_le16(&sample[8]);
	data->accel_z_raw = (int16_t)sys_get_le16(&sample[10]);
	data->temp_raw = (int16_t)sys_get_le16(&sample[12]);

	return 0;
}

static int qmi8660_channel_get(const struct device *dev, enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct qmi8660_dev_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		qmi8660_convert_accel(val, data->accel_x_raw, data->rt_cfg.acc_fs.val);
		return 0;
	case SENSOR_CHAN_ACCEL_Y:
		qmi8660_convert_accel(val, data->accel_y_raw, data->rt_cfg.acc_fs.val);
		return 0;
	case SENSOR_CHAN_ACCEL_Z:
		qmi8660_convert_accel(val, data->accel_z_raw, data->rt_cfg.acc_fs.val);
		return 0;
	case SENSOR_CHAN_ACCEL_XYZ:
		qmi8660_convert_accel(&val[0], data->accel_x_raw, data->rt_cfg.acc_fs.val);
		qmi8660_convert_accel(&val[1], data->accel_y_raw, data->rt_cfg.acc_fs.val);
		qmi8660_convert_accel(&val[2], data->accel_z_raw, data->rt_cfg.acc_fs.val);
		return 0;
	case SENSOR_CHAN_GYRO_X:
		qmi8660_convert_gyro(val, data->gyro_x_raw, data->rt_cfg.gyr_fs.val);
		return 0;
	case SENSOR_CHAN_GYRO_Y:
		qmi8660_convert_gyro(val, data->gyro_y_raw, data->rt_cfg.gyr_fs.val);
		return 0;
	case SENSOR_CHAN_GYRO_Z:
		qmi8660_convert_gyro(val, data->gyro_z_raw, data->rt_cfg.gyr_fs.val);
		return 0;
	case SENSOR_CHAN_GYRO_XYZ:
		qmi8660_convert_gyro(&val[0], data->gyro_x_raw, data->rt_cfg.gyr_fs.val);
		qmi8660_convert_gyro(&val[1], data->gyro_y_raw, data->rt_cfg.gyr_fs.val);
		qmi8660_convert_gyro(&val[2], data->gyro_z_raw, data->rt_cfg.gyr_fs.val);
		return 0;
	case SENSOR_CHAN_DIE_TEMP:
		qmi8660_convert_temp(val, data->temp_raw);
		return 0;
	default:
		return -ENOTSUP;
	}
}

static int qmi8660_attr_get(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, struct sensor_value *val)
{
	const struct qmi8660_dev_data *dev_data = dev->data;

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (qmi8660_is_accel_channel(chan)) {
			val->val1 = dev_data->rt_cfg.acc_odr.val;
			val->val2 = 0;
			return 0;
		}

		if (qmi8660_is_gyro_channel(chan)) {
			val->val1 = dev_data->rt_cfg.gyr_odr.val;
			val->val2 = 0;
			return 0;
		}
		break;
	case SENSOR_ATTR_FULL_SCALE:
		if (qmi8660_is_accel_channel(chan)) {
			sensor_g_to_ms2(dev_data->rt_cfg.acc_fs.val, val);
			return 0;
		}

		if (qmi8660_is_gyro_channel(chan)) {
			sensor_degrees_to_rad(dev_data->rt_cfg.gyr_fs.val, val);
			return 0;
		}
		break;
	case SENSOR_ATTR_CHIP_ID: {
		uint8_t chip_id = 0x00;

		qmi8660_reg_read_rtio(&dev_data->bus, QMI8660_UI_WHOAMI, &chip_id, 1);
		val->val1 = chip_id;
		val->val2 = 0;
		return 0;
	}
	default:
		break;
	}

	return -ENOTSUP;
}

static int qmi8660_attr_set(const struct device *dev, enum sensor_channel chan,
			    enum sensor_attribute attr, const struct sensor_value *val)
{
	struct qmi8660_dev_data *dev_data = dev->data;

	LOG_INF("%s:request attr %d, val1 = %d, val2 = %d", __func__, attr, val->val1, val->val2);
	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		if (qmi8660_is_accel_channel(chan)) {
			qmi8660_select_odr(val->val1, &dev_data->rt_cfg.acc_odr.val,
					   &dev_data->rt_cfg.acc_odr.reg_val);
			return 0;
		}

		if (qmi8660_is_gyro_channel(chan)) {
			qmi8660_select_odr(val->val1, &dev_data->rt_cfg.gyr_odr.val,
					   &dev_data->rt_cfg.gyr_odr.reg_val);
			return 0;
		}
		break;
	case SENSOR_ATTR_FULL_SCALE:
		if (qmi8660_is_accel_channel(chan)) {
			qmi8660_select_fs(val->val1, &dev_data->rt_cfg.acc_fs.val,
					  &dev_data->rt_cfg.acc_fs.reg_val,
					  &dev_data->rt_cfg.acc_fs.shift, qmi8660_accel_fs_map,
					  ARRAY_SIZE(qmi8660_accel_fs_map));
			return 0;
		}

		if (qmi8660_is_gyro_channel(chan)) {
			qmi8660_select_fs(val->val1, &dev_data->rt_cfg.gyr_fs.val,
					  &dev_data->rt_cfg.gyr_fs.reg_val,
					  &dev_data->rt_cfg.gyr_fs.shift, qmi8660_gyro_fs_map,
					  ARRAY_SIZE(qmi8660_gyro_fs_map));
			return 0;
		}
		break;
	default:
		break;
	}
	return -ENOTSUP;
}

static DEVICE_API(sensor, qmi8660_api) = {
#ifdef CONFIG_QMI8660_TRIGGER
	.trigger_set = qmi8660_trigger_set,
#endif
	.attr_get = qmi8660_attr_get,
	.attr_set = qmi8660_attr_set,
	.sample_fetch = qmi8660_sample_fetch,
	.channel_get = qmi8660_channel_get,
#ifdef CONFIG_SENSOR_ASYNC_API
	.submit = qmi8660_submit,
	.get_decoder = qmi8660_get_decoder,
#endif
};

#if defined(CONFIG_SHELL) && DUMP_ENABLE
static bool shell_device_is_qmi8660(const struct device *dev)
{
	return dev->api == &qmi8660_api;
}

static void shell_qmi8660_device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_filter(idx, shell_device_is_qmi8660);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(dsub_qmi8660_device_name, shell_qmi8660_device_name_get);

static int cmd_qmi8660_dump(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev;

	ARG_UNUSED(argc);

	dev = shell_device_get_binding(argv[1]);
	if (dev == NULL) {
		shell_error(sh, "QMI8660 device unknown (%s)", argv[1]);
		return -ENODEV;
	}

	if (!shell_device_is_qmi8660(dev)) {
		shell_error(sh, "Device is not QMI8660 (%s)", argv[1]);
		return -ENODEV;
	}

	qmi8660_dump(dev);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_qmi8660,
			       SHELL_CMD_ARG(dump, &dsub_qmi8660_device_name,
					     SHELL_HELP("Dump QMI8660 registers", "<device>"),
					     cmd_qmi8660_dump, 2, 0),
			       SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(qmi8660, &sub_qmi8660, "QMI8660 commands", NULL);
#endif

static int qmi8660_init(const struct device *dev)
{
	int ret = 0;
	const struct qmi8660_dev_cfg *dev_cfg = dev->config;
	struct qmi8660_dev_data *dev_data = dev->data;

	if (!device_is_ready(dev_cfg->bus_dev)) {
		LOG_ERR("Bus device not ready");
		return -ENODEV;
	}

	/* Add initialization code here */
	ret = qmi8660_detect(dev);
	if (ret < 0) {
		LOG_ERR("Failed to connect to QMI8660 sensor");
		return ret;
	}

	memset(&dev_data->rt_cfg, 0, sizeof(dev_data->rt_cfg));

	ret = qmi8660_setup(dev);
	if (ret < 0) {
		LOG_ERR("Failed to setup QMI8660 sensor");
		return ret;
	}

	struct qmi8660_runtime_cfg new_cfg = {0};
	/* acc & gyro cfg */
	new_cfg.gyr_en = false;
	new_cfg.acc_en = false;
	qmi8660_select_odr(dev_cfg->accel_odr_hz, &new_cfg.acc_odr.val, &new_cfg.acc_odr.reg_val);
	qmi8660_select_odr(dev_cfg->gyro_odr_hz, &new_cfg.gyr_odr.val, &new_cfg.gyr_odr.reg_val);
	qmi8660_select_fs(dev_cfg->accel_fs, &new_cfg.acc_fs.val, &new_cfg.acc_fs.reg_val,
			  &new_cfg.acc_fs.shift, qmi8660_accel_fs_map,
			  ARRAY_SIZE(qmi8660_accel_fs_map));
	qmi8660_select_fs(dev_cfg->gyro_fs, &new_cfg.gyr_fs.val, &new_cfg.gyr_fs.reg_val,
			  &new_cfg.gyr_fs.shift, qmi8660_gyro_fs_map,
			  ARRAY_SIZE(qmi8660_gyro_fs_map));
	/* fifo cfg */
	new_cfg.fifo_mode = QMI8660_FIFO_BYPASS_MODE;
	new_cfg.fifo_wtm = 0;
	/* int cfg */
	new_cfg.int1_cfg.bits.mode = QMI8660_INT_MODE_RAW;
	new_cfg.int1_cfg.bits.lvl = QMI8660_INT_ACTIVE_HIGH;
	new_cfg.int1_cfg.bits.od = QMI8660_INT_PUSH_PULL;
	new_cfg.int2_cfg.bits.pulse_sel = QMI8660_INT_PULSE_40US;

	new_cfg.int1_ctl0.bits.fifo_ovf = 0;
	new_cfg.int1_ctl0.bits.fifo_wtm = 1;
	new_cfg.int1_ctl0.bits.fifo_full = 1;
	/* new_cfg.int1_ctl0.bits.cmd_done = 1; */

	ret = qmi8660_safely_configure(dev, &new_cfg);
	if (ret) {
		LOG_ERR("%s:failed to set default params!", __func__);
	}

#if DUMP_ENABLE
	/* qmi8660_dump(dev); */
#endif

	return ret;
}

#define QMI8660_DEV_CFG_I2C(inst)                                                                  \
	.bus_api = &qmi8660_bus_api_i2c, .bus_cfg = {.i2c_addr = DT_INST_REG_ADDR(inst)}

#define QMI8660_SPI_MODE SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB

#define QMI8660_DEV_CFG_SPI(inst)                                                                  \
	.bus_api = &qmi8660_bus_api_spi,                                                           \
	.bus_cfg = {.spi_spec = SPI_DT_SPEC_INST_GET(inst, QMI8660_SPI_MODE)}

#define QMI8660_RTIO_SPI_DEFINE(inst)                                                              \
	SPI_DT_IODEV_DEFINE(qmi8660_iodev_##inst, DT_DRV_INST(inst), QMI8660_SPI_MODE)

#define QMI8660_RTIO_I2C_DEFINE(inst) I2C_DT_IODEV_DEFINE(qmi8660_iodev_##inst, DT_DRV_INST(inst))

#define QMI8660_RTIO_DEFINE(inst)                                                                  \
	COND_CODE_1(DT_INST_ON_BUS(inst, i2c),\
			    (QMI8660_RTIO_I2C_DEFINE(inst)),\
				(QMI8660_RTIO_SPI_DEFINE(inst)));        \
	RTIO_DEFINE(qmi8660_rtio_ctx_##inst, 32, 32)

/* clang-format off */
#define QMI8660_DEFINE(inst) \
	IF_ENABLED(CONFIG_QMI8660_STREAM, (QMI8660_RTIO_DEFINE(inst))); \
	static struct qmi8660_dev_data qmi8660_dev_data_##inst = { \
		IF_ENABLED(CONFIG_QMI8660_STREAM, \
		( \
			.bus = { \
				.rtio = { \
					.ctx = &qmi8660_rtio_ctx_##inst, \
					.iodev = &qmi8660_iodev_##inst, \
				}, \
				.is_spi = DT_INST_ON_BUS(inst, spi) ? true : false \
			})) };                    \
                                                                                                   \
	static const struct qmi8660_dev_cfg qmi8660_dev_cfg_##inst = {                             \
		.bus_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                       \
		COND_CODE_1(DT_INST_ON_BUS(inst, i2c),                                        \
			    (QMI8660_DEV_CFG_I2C(inst)),                                          \
			    (QMI8660_DEV_CFG_SPI(inst))),                  \
			 .accel_fs = DT_INST_PROP(inst, accel_fs),                                 \
			 .gyro_fs = DT_INST_PROP(inst, gyro_fs),                                   \
			 .accel_odr_hz = DT_INST_PROP(inst, accel_odr_hz),                         \
			 .gyro_odr_hz = DT_INST_PROP(inst, gyro_odr_hz),                           \
			 IF_ENABLED(CONFIG_QMI8660_TRIGGER, \
			   (.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, { 0 }),	\
			    .int_pin = DT_INST_PROP(inst, int_pin),)) };             \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(inst, qmi8660_init, NULL, &qmi8660_dev_data_##inst,           \
				     &qmi8660_dev_cfg_##inst, POST_KERNEL,                         \
				     CONFIG_SENSOR_INIT_PRIORITY, &qmi8660_api);

DT_INST_FOREACH_STATUS_OKAY(QMI8660_DEFINE)
/* clang-format on */
