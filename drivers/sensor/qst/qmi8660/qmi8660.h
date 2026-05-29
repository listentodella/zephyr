/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __QMI8660_H__
#define __QMI8660_H__

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>

#include "qmi8660_bus.h"

/* this macro is very important! */
/* it will determine the layouts of some structures related with DTS */
#define DT_DRV_COMPAT qst_qmi8660

#define DUMP_ENABLE 1

#define QMI8660_WAI_VALUE (0x06)
/* #define QMI8660_WAI_VALUE               (0x00) */

/* QMI8660 registers */
#define QMI8660_UI_MANID_L        0x00
#define QMI8660_UI_MANID_H        0x01
#define QMI8660_UI_WHOAMI         0x02
#define QMI8660_UI_PARTREV        0x03
#define QMI8660_UI_SN0            0x04
#define QMI8660_UI_SN1            0x05
#define QMI8660_UI_SN2            0x06
#define QMI8660_UI_SN3            0x07
#define QMI8660_UI_SN4            0x08
#define QMI8660_UI_SN5            0x09
#define QMI8660_UI_SN_CRC         0x0A
#define QMI8660_UI_COMM_CTL       0x0B
#define QMI8660_UI_MOT_A_CTL0     0x10
#define QMI8660_UI_MOT_A_CTL1     0x11
#define QMI8660_UI_MOT_A_CTL2     0x12
#define QMI8660_UI_MOT_A_CTL3     0x13
#define QMI8660_UI_MOT_A_CTL4     0x14
#define QMI8660_UI_MOT_A_CTL5     0x15
#define QMI8660_UI_MOT_A_CTL6     0x16
#define QMI8660_UI_MOT_A_CTL7     0x17
#define QMI8660_UI_MOT_A_CTL8     0x18
#define QMI8660_UI_MOT_A_CTL9     0x19
#define QMI8660_UI_MOT_A_CTL10    0x1A
#define QMI8660_UI_MOT_A_CTL11    0x1B
#define QMI8660_UI_MOT_A_CTL12    0x1C
#define QMI8660_UI_MOT_B_CTL0     0x1D
#define QMI8660_UI_MOT_B_CTL1     0x1E
#define QMI8660_UI_MOT_B_CTL2     0x1F
#define QMI8660_UI_MOT_B_CTL3     0x20
#define QMI8660_UI_MOT_B_CTL4     0x21
#define QMI8660_UI_MOT_B_CTL5     0x22
#define QMI8660_UI_MOT_B_CTL6     0x23
#define QMI8660_UI_MOT_B_CTL7     0x24
#define QMI8660_UI_MOT_B_CTL8     0x25
#define QMI8660_UI_MOT_B_CTL9     0x26
#define QMI8660_UI_MOT_B_CTL10    0x27
#define QMI8660_UI_MOT_B_CTL11    0x28
#define QMI8660_UI_MOT_B_CTL12    0x29
#define QMI8660_UI_A_OFF_X        0x2C
#define QMI8660_UI_A_OFF_MSBS0    0x2D
#define QMI8660_UI_A_OFF_Y        0x2E
#define QMI8660_UI_A_OFF_MSBS1    0x2F
#define QMI8660_UI_A_OFF_Z        0x30
#define QMI8660_UI_G_OFF_X        0x31
#define QMI8660_UI_G_OFF_MSBS0    0x32
#define QMI8660_UI_G_OFF_Y        0x33
#define QMI8660_UI_G_OFF_MSBS1    0x34
#define QMI8660_UI_G_OFF_Z        0x35
#define QMI8660_UI_ACTL0          0x36
#define QMI8660_UI_ACTL1          0x37
#define QMI8660_UI_GCTL0          0x38
#define QMI8660_UI_GCTL1          0x39
#define QMI8660_UI_ECTL0          0x3A
#define QMI8660_UI_ECTL1          0x3B
#define QMI8660_UI_ENGENCTL       0x3C
#define QMI8660_UI_ENCTL          0x3D
#define QMI8660_UI_ENCTL_OIS      0x3E
#define QMI8660_UI_ACTL_OIS0      0x3F
#define QMI8660_UI_ACTL_OIS1      0x40
#define QMI8660_UI_GCTL_OIS0      0x41
#define QMI8660_UI_GCTL_OIS1      0x42
#define QMI8660_UI_INT1_CFG       0x43
#define QMI8660_UI_INT1_CTL0      0x44
#define QMI8660_UI_INT1_CTL1      0x45
#define QMI8660_UI_INT1_CTL2      0x46
#define QMI8660_UI_INT2_CFG       0x47
#define QMI8660_UI_INT2_CTL0      0x48
#define QMI8660_UI_INT2_CTL1      0x49
#define QMI8660_UI_INT2_CTL2      0x4A
#define QMI8660_UI_INT2_CTL3      0x4B
#define QMI8660_UI_INTIBI_CTL0    0x4C
#define QMI8660_UI_INTIBI_CTL1    0x4D
#define QMI8660_UI_INTIBI_CTL2    0x4E
#define QMI8660_UI_FIFO_TRIG_SRC0 0x4F
#define QMI8660_UI_FIFO_TRIG_SRC1 0x50
#define QMI8660_UI_FIFO_WTM_TH_L  0x51
#define QMI8660_UI_FIFO_CTL0      0x52
#define QMI8660_UI_FIFO_CTL1      0x53
#define QMI8660_UI_FIFO_STATUS_L  0x54
#define QMI8660_UI_FIFO_STATUS_H  0x55
#define QMI8660_UI_FIFO_PAST      0x56
#define QMI8660_UI_FIFO_DATA      0x57
#define QMI8660_UI_INT_STATUS0    0x58
#define QMI8660_UI_INT_STATUS1    0x59
#define QMI8660_UI_INT_STATUS2    0x5A
#define QMI8660_UI_INT_STATUS3    0x5B
#define QMI8660_UI_GX_L           0x60
#define QMI8660_UI_GX_H           0x61
#define QMI8660_UI_GY_L           0x62
#define QMI8660_UI_GY_H           0x63
#define QMI8660_UI_GZ_L           0x64
#define QMI8660_UI_GZ_H           0x65
#define QMI8660_UI_AX_L           0x66
#define QMI8660_UI_AX_H           0x67
#define QMI8660_UI_AY_L           0x68
#define QMI8660_UI_AY_H           0x69
#define QMI8660_UI_AZ_L           0x6A
#define QMI8660_UI_AZ_H           0x6B
#define QMI8660_UI_TEMP_L         0x6C
#define QMI8660_UI_TEMP_H         0x6D
#define QMI8660_UI_TIMESTAMP_L    0x6E
#define QMI8660_UI_TIMESTAMP_M    0x6F
#define QMI8660_UI_TIMESTAMP_H    0x70
#define QMI8660_UI_STEP_CNT_L     0x72
#define QMI8660_UI_STEP_CNT_H     0x73
#define QMI8660_UI_TAP_STATUS     0x74
#define QMI8660_UI_ORIENT_STATUS  0x75
#define QMI8660_UI_ARM_RSVD_0     0x76
#define QMI8660_UI_ARM_RSVD_1     0x77
#define QMI8660_UI_ARM_RSVD_2     0x78
#define QMI8660_UI_STATUS0        0x79
#define QMI8660_UI_RESET          0x7B
#define QMI8660_UI_TM_LOCK_L      0x7C
#define QMI8660_UI_TM_LOCK_H      0x7D
#define PAGE_L                    0x7E
#define PAGE_H                    0x7F
#define QMI8660_UI_PED_CTL0       0x80
#define QMI8660_UI_PED_CTL1       0x81
#define QMI8660_UI_PED_CTL2       0x82
#define QMI8660_UI_PED_CTL3       0x83
#define QMI8660_UI_PED_CTL4       0x84
#define QMI8660_UI_PED_CTL5       0x85
#define QMI8660_UI_PED_CTL6       0x86
#define QMI8660_UI_PED_CTL7       0x87
#define QMI8660_UI_PED_CTL8       0x88
#define QMI8660_UI_PED_CTL9       0x89
#define QMI8660_UI_PED_CTL10      0x8A
#define QMI8660_UI_PED_CTL11      0x8B
#define QMI8660_UI_PED_CTL12      0x8C
#define QMI8660_UI_PED_CTL13      0x8D
#define QMI8660_UI_TAP_CTL0       0x90
#define QMI8660_UI_TAP_CTL1       0x91
#define QMI8660_UI_TAP_CTL2       0x92
#define QMI8660_UI_TAP_CTL3       0x93
#define QMI8660_UI_TAP_CTL4       0x94
#define QMI8660_UI_TAP_CTL5       0x95
#define QMI8660_UI_TAP_CTL6       0x96
#define QMI8660_UI_TAP_CTL7       0x97
#define QMI8660_UI_TAP_CTL8       0x98
#define QMI8660_UI_TAP_CTL9       0x99
#define QMI8660_UI_TAP_CTL10      0x9A
#define QMI8660_UI_TAP_CTL11      0x9B
#define QMI8660_UI_IIR_A0         0xA0
#define QMI8660_UI_IIR_A1         0xA1
#define QMI8660_UI_IIR_A2         0xA2
#define QMI8660_UI_IIR_A3         0xA3
#define QMI8660_UI_IIR_A4         0xA4
#define QMI8660_UI_IIR_A5         0xA5
#define QMI8660_UI_IIR_A6         0xA6
#define QMI8660_UI_IIR_A7         0xA7
#define QMI8660_UI_IIR_A8         0xA8
#define QMI8660_UI_IIR_A9         0xA9
#define QMI8660_UI_IIR_B0         0xB0
#define QMI8660_UI_IIR_B1         0xB1
#define QMI8660_UI_IIR_B2         0xB2
#define QMI8660_UI_IIR_B3         0xB3
#define QMI8660_UI_IIR_B4         0xB4
#define QMI8660_UI_IIR_B5         0xB5
#define QMI8660_UI_IIR_B6         0xB6
#define QMI8660_UI_IIR_B7         0xB7
#define QMI8660_UI_IIR_B8         0xB8
#define QMI8660_UI_IIR_B9         0xB9
#define QMI8660_UI_CMDDATA0       0xC0
#define QMI8660_UI_CMDDATA1       0xC1
#define QMI8660_UI_CMDDATA2       0xC2
#define QMI8660_UI_CMDDATA3       0xC3
#define QMI8660_UI_CMDDATA4       0xC4
#define QMI8660_UI_CMDDATA5       0xC5
#define QMI8660_UI_CMDDATA6       0xC6
#define QMI8660_UI_CMDDATA7       0xC7
#define QMI8660_UI_CMD            0xC8
#define QMI8660_UI_SPARE_0        0xCC
#define QMI8660_UI_SPARE_1        0xCD
#define QMI8660_UI_SPARE_2        0xCE
#define QMI8660_UI_SPARE_3        0xCF

#define QMI8660_SFT_SYSTEM_KEY  (0x98)
#define QMI8660_SFT_ARM_KEY     (0x92)
#define QMI8660_ONE_SAMPLE_BYTE 6

#define MAX_WATERMARK  100
#define FIFO_DATA_SIZE (MAX_WATERMARK * QMI8660_ONE_SAMPLE_BYTE)
#define SPI_BUF_SIZE   (FIFO_DATA_SIZE + 4)

#define ACCEL_CALI_GAIN 1000
#define GYRO_CALI_GAIN  1000000

#define ACCEL_RESOLUTION_VALUE       ((9.807f) * (16 * 2) / 65536)
/* #define ACCEL_RESOLUTION_VALUE             ((9.807f) * (8 * 2) / 65536) */
#define GYRO_RESOLUTION_VALUE        ((0.017453f) * (4096 * 2) / 65536)
#define TEMPERATURE_RESOLUTION_VALUE 0.00390625f

#define QMI8660_FIFO_FTH_MASK (0x0F)

/* QMI8660 fifo status */
#define QMI8660_FIFO_STATUS_FULL      (0x80) /* FIFO full */
#define QMI8660_FIFO_STATUS_WATERMARK (0x40) /* FIFO Watermark */
#define QMI8660_FIFO_STATUS_TRIGGER   (0x20) /* FIFO TRIGGER */
#define QMI8660_FIFO_STATUS_NOEMPTY   (0x10) /* FIFO not empty */
#define BIT_ACC_RDY_INT               BIT(0)
#define BIT_GYR_RDY_INT               BIT(1)
#define BIT_CMD_DONE_INT              BIT(4)
#define BIT_FIFO_OVF_INT              BIT(5)
#define BIT_FIFO_WTM_INT              BIT(6)
#define BIT_FIFO_FULL_INT             BIT(7)

/* QMI8660 fifo modes */
#define QMI8660_FIFO_BYPASS_MODE  (0x00)
#define QMI8660_FIFO_FIFO_MODE    (0x01)
#define QMI8660_FIFO_STREAM_MODE  (0x02)
#define QMI8660_FIFO_TRIGGER_MODE (0x03)

/* QMI8660 interrupt cfg */
#define QMI8660_INT_PUSH_PULL    (0x00)
#define QMI8660_INT_OPEN_DRAIN   (0x01)
#define QMI8660_INT_ACTIVE_LOW   (0x00)
#define QMI8660_INT_ACTIVE_HIGH  (0x01)
#define QMI8660_INT_MODE_DISABLE (0x00)
#define QMI8660_INT_MODE_RAW     (0x01)
#define QMI8660_INT_MODE_LATCH   (0x02)
#define QMI8660_INT_MODE_PULSE   (0x03)
#define QMI8660_INT_PULSE_40US   (0x00)
#define QMI8660_INT_PULSE_1MS    (0x01)

/* Output data rates (ODR) */
#define QMI8660_ODR_RESERVED (0x00)
#define QMI8660_ODR_0_78125  (0x01)
#define QMI8660_ODR_1_5625   (0x02)
#define QMI8660_ODR_3_125    (0x03)
#define QMI8660_ODR_6_25     (0x04)
#define QMI8660_ODR_12_5     (0x05)
#define QMI8660_ODR_25       (0x06)
#define QMI8660_ODR_50       (0x07)
#define QMI8660_ODR_100      (0x08)
#define QMI8660_ODR_200      (0x09)
#define QMI8660_ODR_400      (0x0a)
#define QMI8660_ODR_800      (0x0b)
#define QMI8660_ODR_1600     (0x0c)
#define QMI8660_ODR_3200     (0x0d)
#define QMI8660_ODR_6400     (0x0e)
#define QMI8660_ODR_12800    (0x0f)

/* QMI8660 basic any motion interrupts */
#define QMI8660_WAKEUP_MASK (0x02)

enum qmi8660_stream_state {
	QMI8660_STREAM_OFF = 0,
	QMI8660_STREAM_ON = 1,
	QMI8660_STREAM_BUSY = 2,
};

struct qmi8660_map {
	uint16_t val;
	uint8_t reg_val;
	uint8_t shift; /* optional */
};

union qmi8660_int_cfg {
	uint8_t val;
	struct {
		uint8_t od: 1;        /* bit0, 0 push-pull, 1 open-drain */
		uint8_t lvl: 1;       /* bit1, 0 active high, 1 active low */
		uint8_t mode: 2;      /* bit2~3, 0 disable, 1 raw, 2 latched, 3 pulse */
		uint8_t pulse_sel: 1; /* bit4, 0 40us pulse, 1 1ms pulse */
		uint8_t reserved: 3;  /* bit5~7 */
	} bits;
};

union qmi8660_int_ctl0 {
	uint8_t val;
	struct {
		uint8_t drdy_acc_en: 1;  /* bit0 */
		uint8_t drdy_gyro_en: 1; /* bit1 */
		uint8_t reserved: 2;     /* bit2~3 */
		uint8_t cmd_done: 1;     /* bit4 */
		uint8_t fifo_ovf: 1;     /* bit5 */
		uint8_t fifo_wtm: 1;     /* bit6 */
		uint8_t fifo_full: 1;    /* bit7 */
	} bits;
};
union qmi8660_int_ctl1 {
	uint8_t val;
	struct {
		uint8_t mot_a_en: 1;    /* bit0 */
		uint8_t mot_b_en: 1;    /* bit1 */
		uint8_t tap1_en: 1;     /* bit2 */
		uint8_t tap2_en: 1;     /* bit3 */
		uint8_t tap3_en: 1;     /* bit4 */
		uint8_t step_th_en: 1;  /* bit5 */
		uint8_t step_det_en: 1; /* bit6 */
		uint8_t orient_en: 1;   /* bit7 */
	} bits;
};

struct qmi8660_runtime_cfg {
	bool acc_en;
	bool gyr_en;

	struct qmi8660_map acc_fs;
	struct qmi8660_map gyr_fs;
	struct qmi8660_map acc_odr;
	struct qmi8660_map gyr_odr;

	uint8_t fifo_mode;
	uint16_t fifo_wtm;

	union qmi8660_int_cfg int1_cfg;
	union qmi8660_int_cfg int2_cfg;
	union qmi8660_int_ctl0 int1_ctl0;
	union qmi8660_int_ctl1 int1_ctl1;
	union qmi8660_int_ctl0 int2_ctl0;
	union qmi8660_int_ctl1 int2_ctl1;
	/* struct alignment axis_align[3]; */
};

struct qmi8660_dev_data {
	int16_t temp_raw;
	int16_t accel_x_raw;
	int16_t accel_y_raw;
	int16_t accel_z_raw;
	int16_t gyro_x_raw;
	int16_t gyro_y_raw;
	int16_t gyro_z_raw;
	struct qmi8660_runtime_cfg rt_cfg;
	/* struct qmi8660_map accel_fs; */
	/* struct qmi8660_map gyro_fs; */
	/* struct qmi8660_map accel_odr; */
	/* struct qmi8660_map gyro_odr; */

#ifdef CONFIG_QMI8660_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_cb;
	sensor_trigger_handler_t drdy_handler;
	const struct sensor_trigger *drdy_trigger;

#if defined(CONFIG_QMI8660_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_QMI8660_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_QMI8660_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#ifdef CONFIG_QMI8660_STREAM
	struct rtio_iodev_sqe *streaming_sqe;
	struct qmi8660_bus bus;
	uint8_t int_status[4];
	uint16_t fifo_count;
	uint64_t timestamp;
	atomic_t state;
#endif /* CONFIG_QMI8660_STREAM */

#endif
};

typedef int (*qmi8660_bus_read_t)(const struct device *dev, uint8_t reg, uint8_t *data,
				  uint16_t len);
typedef int (*qmi8660_bus_write_t)(const struct device *dev, uint8_t reg, const uint8_t *data,
				   uint16_t len);

struct qmi8660_bus_api {
	qmi8660_bus_read_t read;
	qmi8660_bus_write_t write;
};

struct qmi8660_dev_cfg {
	const struct device *bus_dev;
	const struct qmi8660_bus_api *bus_api;
	union {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
		uint16_t i2c_addr;
#endif
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
		struct spi_dt_spec spi_spec;
#endif
	} bus_cfg;

#ifdef CONFIG_QMI8660_TRIGGER
	struct gpio_dt_spec int_gpio;
	uint8_t int_pin;
#endif
	/* below are default configs from dts */
	uint16_t accel_fs;
	uint16_t gyro_fs;
	uint16_t accel_odr_hz;
	uint16_t gyro_odr_hz;
};
int qmi8660_reg_read(const struct device *dev, uint8_t reg, uint8_t *data, uint16_t len);
int qmi8660_reg_write(const struct device *dev, uint8_t reg, uint8_t val, uint32_t delay_us);
void qmi8660_dump(const struct device *dev);
int qmi8660_safely_configure(const struct device *dev, struct qmi8660_runtime_cfg *cfg);

int qmi8660_read_raw(const struct device *dev, uint8_t data[14]);

int qmi8660_get_shift(enum sensor_channel channel, int accel_fs, int gyro_fs, int variant,
		      int8_t *shift);

static inline void qmi8660_convert_accel(struct sensor_value *val, int16_t raw, uint16_t fs_g)
{
	int64_t micro_ms2 = ((int64_t)raw * SENSOR_G * fs_g) / 32768LL;

	(void)sensor_value_from_micro(val, micro_ms2);
}

static inline void qmi8660_convert_gyro(struct sensor_value *val, int16_t raw, uint16_t fs_dps)
{
	int64_t micro_rads = ((int64_t)raw * fs_dps * SENSOR_PI) / (32768LL * 180LL);

	(void)sensor_value_from_micro(val, micro_rads);
}

static inline void qmi8660_convert_temp(struct sensor_value *val, int16_t raw)
{
	int64_t micro_celsius = ((int64_t)raw * 1000000LL) / 256LL;

	(void)sensor_value_from_micro(val, micro_celsius);
}
#endif /* __QMI8660_H__ */
