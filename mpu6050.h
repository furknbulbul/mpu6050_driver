#ifndef MPU6050_H_
#define MPU6050_H_

#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>


#define OUTPUT_DATA_SIZE        24
#define REG_SELF_TEST_X         0x0D
#define REG_SELF_TEST_Y         0x0E
#define REG_SELF_TEST_Z         0x0F
#define REG_SELF_TEST_A         0x10

#define REG_SMPLRT_DIV          0x19   /* sample rate divider */
#define REG_CONFIG              0x1A   /* DLPF + FSYNC */
#define REG_GYRO_CONFIG         0x1B   /* gyro full-scale range */
#define REG_ACCEL_CONFIG        0x1C   /* accel full-scale range */
#define REG_MOT_THR				0x1F	
#define REG_FIFO_EN             0x23

#define REG_INT_PIN_CFG         0x37
#define REG_INT_ENABLE          0x38
#define REG_INT_STATUS          0x3A
#define REG_ACCEL_XOUT_H        0x3B   /* 6 bytes: XH XL YH YL ZH ZL */
#define REG_GYRO_XOUT_H         0x43   /* 6 bytes: XH XL YH YL ZH ZL */

#define REG_USER_CTRL         	0x6A
#define REG_PWR_MGMT_1          0x6B
#define REG_PWR_MGMT_2          0x6C
#define REG_FIFO_COUNT_H        0x72   /* 2 bytes big-endian */
#define REG_FIFO_R_W            0x74

#define REG_WHO_AM_I            0x75

#define BIT_XG_FIFO_EN          BIT(6)
#define BIT_YG_FIFO_EN          BIT(5)
#define BIT_ZG_FIFO_EN          BIT(4)
#define BIT_ACCEL_FIFO_EN       BIT(3)


#define BIT_INT_LEVEL           BIT(7)  /* 1 = active-low */
#define BIT_INT_OPEN            BIT(6)  /* 1 = open-drain */
#define BIT_LATCH_INT_EN        BIT(5)

#define BIT_FIFO_OFLOW_EN       BIT(4)
#define BIT_DATA_RDY_EN         BIT(0)

#define BIT_FIFO_OFLOW_INT      BIT(4)



/* Sensor data output registers */


#define BYTES_PER_3AXIS 6
#define FIFO_SIZE 1024



#define BIT_XG_FIFO_EN          BIT(6) 
#define BIT_ACCEL_FIFO_EN       BIT(3) 

/* Power management */

#define BIT_DEVICE_RESET        BIT(7)
#define BIT_SLEEP               BIT(6)
#define BIT_CYCLE               BIT(5)
#define CLKSEL_MASK             0x07
#define CLKSEL_INTERNAL         0x00
#define BIT_CLKSEL_0        BIT(0)   /* recommended */
#define BIT_CLKSEL_1        BIT(1)
#define BIT_CLKSEL_2        BIT(2)
#define BIT_TEMP_DIS        BIT(3)

#define WHOAMI_VALUE            0x68

#define POWER_UP_TIME_MS        100
#define GYRO_STARTUP_TIME_MS    60
#define ACCEL_STARTUP_TIME_MS   20



/**
 * struct mpu6050_reg_map - Register addresses for the MPU-6050.
 *
 * Keeping addresses in a struct (rather than scattered #defines) lets
 * the rest of the driver stay address-agnostic.  A single const instance
 * (mpu6050_reg) is defined in mpu6050_core.c and pointed to by
 * mpu6050_state.reg.
 *
 * @sample_rate_div:  SMPLRT_DIV  – sample-rate divider (0x19)
 * @lpf:              CONFIG      – DLPF_CFG / EXT_SYNC_SET  (0x1A)
 * @gyro_config:      GYRO_CONFIG – FS_SEL + self-test bits   (0x1B)
 * @accel_config:     ACCEL_CONFIG – AFS_SEL + self-test bits (0x1C)
 * @fifo_en:          FIFO_EN     – selects sensors → FIFO    (0x23)
 * @int_pin_cfg:      INT_PIN_CFG – interrupt pin behaviour   (0x37)
 * @int_enable:       INT_ENABLE  – interrupt source enables  (0x38)
 * @int_status:       INT_STATUS  – interrupt status flags    (0x3A)
 * @raw_accel:        ACCEL_XOUT_H – first accel output reg   (0x3B)
 * @temperature:      TEMP_OUT_H  – temperature output        (0x41)
 * @raw_gyro:         GYRO_XOUT_H – first gyro output reg     (0x43)
 * @user_ctrl:        USER_CTRL   – FIFO enable / reset       (0x6A)
 * @pwr_mgmt_1:       PWR_MGMT_1  – sleep, reset, clock       (0x6B)
 * @pwr_mgmt_2:       PWR_MGMT_2  – per-axis standby          (0x6C)
 * @fifo_count_h:     FIFO_COUNTH – FIFO byte count (MSB)     (0x72)
 * @fifo_r_w:         FIFO_R_W    – FIFO read/write port       (0x74)
 * @who_am_i:         WHO_AM_I    – device identity            (0x75)
 * @accel_offset:     XA_OFFS_H   – accel offset registers     (0x06)
 * @gyro_offset:      XG_OFFS_USRH – gyro offset registers    (0x13)
 */
struct mpu6050_reg_map {
    u8 SMPRT_DIV;
    u8 CONFIG;
    u8 GYRO_CONFIG;
    u8 ACCEL_CONFIG;
    u8 FIFO_EN;
    u8 INT_PIN_CFG;
    u8 INT_ENABLE;
    u8 INT_STATUS;
    u8 ACCEL_XOUT_H;
    u8 GYRO_XOUT_H;
    u8 USER_CONTROL;
    u8 PWR_MGMT_1;
    u8 PWR_MGMT_2;
    u8 FIFO_COUNT_H;
    u8 FIFO_R_W;
    u8 WHO_AM_I;
};

/**
 * struct mpu6050_chip_config - Runtime-configurable chip parameters.
 * @gyro_fsr:           Gyroscope full-scale range (enum mpu6050_gyro_fsr).
 * @accel_fsr:          Accelerometer full-scale range (enum mpu6050_accel_fsr).
 * @dlpf:               DLPF setting (enum mpu6050_dlpf).
 * @sample_rate_div:    Value written to SMPLRT_DIV register.
 * @accel_fifo_enable:  Accelerometer data routed to FIFO.
 * @gyro_fifo_enable:   Gyroscope data routed to FIFO.
 * @temp_fifo_enable:   Temperature data routed to FIFO.
 * @accel_en:           Accelerometer engine on.
 * @gyro_en:            Gyroscope engine on.
 * @temp_en:            Temperature sensor enabled.
 */
struct mpu6050_chip_config {
	u8 sample_rate_div;
	unsigned int accel_fifo_enable : 1;
	unsigned int gyro_fifo_enable  : 1;
	unsigned int accel_en : 1;
	unsigned int gyro_en  : 1;
};


/**
 * struct mpu6050_state - Per-device driver state.
 * @client:       Underlying I2C client.
 * @lock:         Serialises register access and config changes.
 * @trig:         IIO trigger (data-ready interrupt).
 * @irq:          GPIO interrupt line number.
 * @reg:          Pointer to the chip's register address map.
 * @config:       Current chip configuration (cached).
 * @vdd:          Optional VDD regulator.
 * @vddio:        Optional VDDIO regulator.
 * @data:         Scratch buffer for bulk I2C reads (FIFO drain etc.).
 * @it_timestamp: Timestamp captured in the hard-IRQ handler.
 */
struct mpu6050_state {
	struct i2c_client           *client;
	struct mutex                 lock;
	struct iio_trigger          *trig;
	int                          irq;
	const struct mpu6050_reg_map *reg;      /* ← register address map */
	struct mpu6050_chip_config   config;
	/* scratch buffer: accel(6) + temp(2) + gyro(6) + pad(2) + ts(8) */
	u8                        data[OUTPUT_DATA_SIZE] __aligned(8);
    s64 it_timestamp; 
};

/* Low-level I2C register access */
int  mpu6050_read_reg(struct mpu6050_state *st, u8 reg, u8 *val);
int  mpu6050_write_reg(struct mpu6050_state *st, u8 reg, u8 val);
int  mpu6050_read_burst(struct mpu6050_state *st, u8 reg, void *buf, size_t len);

/* Core init / tear-down */
int  mpu6050_chip_init(struct mpu6050_state *st);
int  mpu6050_set_power_mode(struct mpu6050_state *st, bool sleep);

int  mpu6050_prepare_fifo(struct mpu6050_state *st, bool enable);

/* Trigger / buffer functions (defined in mpu6050_trigger.c / _ring.c) */
int  mpu6050_probe_trigger(struct iio_dev *indio_dev, int irq_type);
irqreturn_t mpu6050_read_fifo(int irq, void *p);

enum {
	SCAN_ACCEL_X = 0,
	SCAN_ACCEL_Y,
	SCAN_ACCEL_Z,
	SCAN_GYRO_X,
	SCAN_GYRO_Y,
	SCAN_GYRO_Z,
	SCAN_TIMESTAMP,
};


#define SCAN_MASK_3AXIS_ACCEL	\
	(BIT(SCAN_ACCEL_X)		\
	| BIT(SCAN_ACCEL_Y)		\
	| BIT(SCAN_ACCEL_Z))

#define SCAN_MASK_3AXIS_GYRO	\
	(BIT(SCAN_GYRO_X)		\
	| BIT(SCAN_GYRO_Y)		\
	| BIT(SCAN_GYRO_Z))

static const unsigned long scan_masks[] = {
	/* 3-axis accel */
	SCAN_MASK_3AXIS_ACCEL,
	/* 3-axis gyro */
	SCAN_MASK_3AXIS_GYRO,
	/* accel + gyro */
	SCAN_MASK_3AXIS_ACCEL | SCAN_MASK_3AXIS_GYRO,
	0,
};

#endif /* MPU6050_H_ */

