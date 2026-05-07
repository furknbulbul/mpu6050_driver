// SPDX-License-Identifier: GPL-2.0-only
/*
 * mpu6050_core.c - MPU-6050 IIO driver core (I2C only, no aux bus)
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>

#include "mpu6050.h"

/* ------------------------------------------------------------------ */
/* Local constants not exposed in the header                           */
/* ------------------------------------------------------------------ */

/* USER_CTRL (0x6A) bits */
#define USER_CTRL_FIFO_EN    BIT(6)   /* enable FIFO operations     */
#define USER_CTRL_FIFO_RST   BIT(2)   /* reset FIFO (self-clearing) */

/* PWR_MGMT_2 (0x6C) per-axis standby bits */
#define STBY_XA   BIT(5)
#define STBY_YA   BIT(4)
#define STBY_ZA   BIT(3)
#define STBY_XG   BIT(2)
#define STBY_YG   BIT(1)
#define STBY_ZG   BIT(0)

/* INT_PIN_CFG: clear interrupt status on any register read */
#define INT_RD_CLEAR  BIT(4)

/* IIO scan indices (accel X/Y/Z, gyro X/Y/Z, timestamp) */

/* ------------------------------------------------------------------ */
/* Register map instance                                               */
/* ------------------------------------------------------------------ */

static const struct mpu6050_reg_map mpu6050_reg = {
	.SMPRT_DIV    = REG_SMPLRT_DIV,
	.CONFIG       = REG_CONFIG,
	.GYRO_CONFIG  = REG_GYRO_CONFIG,
	.ACCEL_CONFIG = REG_ACCEL_CONFIG,
	.FIFO_EN      = REG_FIFO_EN,
	.INT_PIN_CFG  = REG_INT_PIN_CFG,
	.INT_ENABLE   = REG_INT_ENABLE,
	.INT_STATUS   = REG_INT_STATUS,
	.ACCEL_XOUT_H = REG_ACCEL_XOUT_H,
	.GYRO_XOUT_H  = REG_GYRO_XOUT_H,
	.USER_CONTROL = REG_USER_CTRL,
	.PWR_MGMT_1   = REG_PWR_MGMT_1,
	.PWR_MGMT_2   = REG_PWR_MGMT_2,
	.FIFO_COUNT_H = REG_FIFO_COUNT_H,
	.FIFO_R_W     = REG_FIFO_R_W,
	.WHO_AM_I     = REG_WHO_AM_I,
};

static const struct mpu6050_chip_config chip_config = {
	.sample_rate_div = 9, // TODO: change
	.accel_en = true,
	.accel_fifo_enable = true,
	.gyro_fifo_enable  = true,
	.gyro_en = true
};


int mpu6050_read_reg(struct mpu6050_state *st, u8 reg, u8 *val)
{
	int ret = i2c_smbus_read_byte_data(st->client, reg);

	if (ret < 0)
		return ret;
	*val = (u8)ret;
	return 0;
}

int mpu6050_write_reg(struct mpu6050_state *st, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(st->client, reg, val);
}

int mpu6050_read_burst(struct mpu6050_state *st, u8 reg, void *buf, size_t len)
{
	struct i2c_msg msgs[2] = {
		{ .addr = st->client->addr, .flags = 0,        .len = 1,   .buf = &reg },
		{ .addr = st->client->addr, .flags = I2C_M_RD, .len = len, .buf = buf  },
	};

	return i2c_transfer(st->client->adapter, msgs, ARRAY_SIZE(msgs));
}

/* ------------------------------------------------------------------ */
/* Power mode                                                          */
/* ------------------------------------------------------------------ */

int mpu6050_set_power_mode(struct mpu6050_state *st, bool sleep)
{
	u8 val;
	int ret;

	ret = mpu6050_read_reg(st, st->reg->PWR_MGMT_1, &val);
	if (ret)
		return ret;

	if (sleep)
		val |=  BIT_SLEEP;
	else
		val &= ~BIT_SLEEP;

	return mpu6050_write_reg(st, st->reg->PWR_MGMT_1, val);
}

/* ------------------------------------------------------------------ */
/* FIFO control                                                        */
/* ------------------------------------------------------------------ */

int mpu6050_prepare_fifo(struct mpu6050_state *st, bool enable)
{
	u8 fifo_en_bits = 0;
	int ret;

	if (enable) {
		/* Reset the FIFO first */
		ret = mpu6050_write_reg(st, st->reg->USER_CONTROL,
					USER_CTRL_FIFO_RST);
		if (ret)
			return ret;

		if (st->config.accel_fifo_enable)
			fifo_en_bits |= BIT_ACCEL_FIFO_EN;
		if (st->config.gyro_fifo_enable)
			fifo_en_bits |= BIT_XG_FIFO_EN |
					BIT_YG_FIFO_EN |
					BIT_ZG_FIFO_EN;

		ret = mpu6050_write_reg(st, st->reg->FIFO_EN, fifo_en_bits);
		if (ret)
			return ret;

		ret = mpu6050_write_reg(st, st->reg->USER_CONTROL,
					USER_CTRL_FIFO_EN);
		if (ret)
			return ret;

		return mpu6050_write_reg(st, st->reg->INT_ENABLE, BIT_DATA_RDY_EN);
	}

	/* Disable path */
	ret = mpu6050_write_reg(st, st->reg->INT_ENABLE, 0);
	if (ret)
		return ret;

	ret = mpu6050_write_reg(st, st->reg->FIFO_EN, 0);
	if (ret)
		return ret;

	return mpu6050_write_reg(st, st->reg->USER_CONTROL, 0);
}

/* ------------------------------------------------------------------ */
/* Chip initialisation                                                 */
/* ------------------------------------------------------------------ */

int mpu6050_chip_init(struct mpu6050_state *st)
{
	u8 val;
	int ret;

	/* Hard reset: all registers return to power-on defaults */
	ret = mpu6050_write_reg(st, st->reg->PWR_MGMT_1, BIT_DEVICE_RESET);
	if (ret)
		return ret;
	msleep(POWER_UP_TIME_MS);

	/* Verify device identity */
	ret = mpu6050_read_reg(st, st->reg->WHO_AM_I, &val);
	if (ret)
		return ret;
	if ((val & 0x7E) != (WHOAMI_VALUE & 0x7E)) {
		dev_err(&st->client->dev,
			"unexpected WHO_AM_I 0x%02x (expected 0x%02x)\n",
			val, WHOAMI_VALUE);
		return -ENODEV;
	}

	/* Wake up and use PLL clock for better stability */
	ret = mpu6050_write_reg(st, st->reg->PWR_MGMT_1, CLKSEL_PLL_XGYRO);
	if (ret)
		return ret;

	/* Sample rate divider */
	ret = mpu6050_write_reg(st, st->reg->SMPRT_DIV, st->config.sample_rate_div);
	if (ret)
		return ret;

	/* DLPF = 188 Hz, gyro output rate = 1 kHz */
	ret = mpu6050_write_reg(st, st->reg->CONFIG, 0x01);
	if (ret)
		return ret;

	/* Gyro FSR = ±250 dps (FS_SEL = 0) */
	ret = mpu6050_write_reg(st, st->reg->GYRO_CONFIG, 0x00);
	if (ret)
		return ret;

	/* Accel FSR = ±2 g (AFS_SEL = 0) */
	ret = mpu6050_write_reg(st, st->reg->ACCEL_CONFIG, 0x00);
	if (ret)
		return ret;

	/* INT pin: active-high, push-pull, clear on any read */
	ret = mpu6050_write_reg(st, st->reg->INT_PIN_CFG, INT_RD_CLEAR | BIT_LATCH_INT_EN);
	if (ret)
		return ret;
	
	/* All axes active (no standby) */
	return mpu6050_write_reg(st, st->reg->PWR_MGMT_2, 0x00);
}

/* ------------------------------------------------------------------ */
/* IIO channel definitions (accel + gyro only, no temperature)        */
/* ------------------------------------------------------------------ */
// IIO_ANGL_VEL type of iio

#define GYRO_CHAN(_axis, _idx)						\
{									\
	.type		  = IIO_ANGL_VEL,				\
	.modified	  = 1,						\
	.channel2	  = IIO_MOD_##_axis,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index	  = _idx,					\
	.scan_type	  = {						\
		.sign		= 's',					\
		.realbits	= 16,					\
		.storagebits	= 16,					\
		.endianness	= IIO_BE,				\
	},								\
}

#define ACCEL_CHAN(_axis, _idx)						\
{									\
	.type		  = IIO_ACCEL,					\
	.modified	  = 1,						\
	.channel2	  = IIO_MOD_##_axis,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index	  = _idx,					\
	.scan_type	  = {						\
		.sign		= 's',					\
		.realbits	= 16,					\
		.storagebits	= 16,					\
		.endianness	= IIO_BE,				\
	},								\
}

static const struct iio_chan_spec mpu6050_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(SCAN_TIMESTAMP),
	ACCEL_CHAN(X, SCAN_ACCEL_X),
	ACCEL_CHAN(Y, SCAN_ACCEL_Y),
	ACCEL_CHAN(Z, SCAN_ACCEL_Z),
	GYRO_CHAN(X,  SCAN_GYRO_X),
	GYRO_CHAN(Y,  SCAN_GYRO_Y),
	GYRO_CHAN(Z,  SCAN_GYRO_Z),
};

/* ------------------------------------------------------------------ */
/* IIO read_raw / write_raw                                            */
/* ------------------------------------------------------------------ */

static int mpu6050_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct mpu6050_state *st = iio_priv(indio_dev);
	__be16 raw;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&st->lock);
		ret = pm_runtime_resume_and_get(&st->client->dev);
		if (ret < 0) {
			mutex_unlock(&st->lock);
			return ret;
		}

		if (chan->type == IIO_ACCEL)
			ret = mpu6050_read_burst(st,
				st->reg->ACCEL_XOUT_H + chan->scan_index * 2,
				&raw, sizeof(raw));
		else /* IIO_ANGL_VEL */
			ret = mpu6050_read_burst(st,
				st->reg->GYRO_XOUT_H +
					(chan->scan_index - SCAN_GYRO_X) * 2,
				&raw, sizeof(raw));

		pm_runtime_mark_last_busy(&st->client->dev);
		pm_runtime_put_autosuspend(&st->client->dev);
		mutex_unlock(&st->lock);

		if (ret < 0)
			return ret;
		*val  = (s16)be16_to_cpu(raw);
		*val2 = 0;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		/*
		 * Fixed default scales (hardware reset values):
		 *   Accel ±2g   → 16384 LSB/g  → 0.000598 m/s² per LSB
		 *   Gyro  ±250° → 131   LSB/°/s → 0.007629 rad/s per LSB
		 */
		*val  = 0;
		*val2 = (chan->type == IIO_ACCEL) ? 598 : 7629;
		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val  = 1000 / (st->config.sample_rate_div + 1);
		*val2 = 0;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int mpu6050_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct mpu6050_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&st->lock);

	if (mask == IIO_CHAN_INFO_SAMP_FREQ) {
		if (val < 4 || val > 1000)
			goto out;
		st->config.sample_rate_div = (1000 / val) - 1;
		ret = mpu6050_write_reg(st, st->reg->SMPRT_DIV,
					st->config.sample_rate_div);
	}

out:
	mutex_unlock(&st->lock);
	return ret;
}

static const struct iio_info mpu6050_info = {
	.read_raw  = mpu6050_read_raw,
	.write_raw = mpu6050_write_raw,
};

/* ------------------------------------------------------------------ */
/* Runtime power management                                            */
/* ------------------------------------------------------------------ */

static int __maybe_unused mpu6050_runtime_suspend(struct device *dev)
{
	struct mpu6050_state *st = iio_priv(dev_get_drvdata(dev));

	return mpu6050_set_power_mode(st, true);
}

static int __maybe_unused mpu6050_runtime_resume(struct device *dev)
{
	struct mpu6050_state *st = iio_priv(dev_get_drvdata(dev));
	int ret;

	ret = mpu6050_set_power_mode(st, false);
	if (ret)
		return ret;
	usleep_range(5000, 10000);
	return 0;
}

static const struct dev_pm_ops mpu6050_pm_ops = {
	SET_RUNTIME_PM_OPS(mpu6050_runtime_suspend, mpu6050_runtime_resume, NULL)
};

/* ------------------------------------------------------------------ */
/* Probe / remove                                                      */
/* ------------------------------------------------------------------ */

static int mpu6050_probe(struct i2c_client *client)
{
	struct iio_dev *indio_dev;
	struct mpu6050_state *st;
	int irq_type, ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_I2C))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->client = client;
	mutex_init(&st->lock);
	st->reg = &mpu6050_reg;
	i2c_set_clientdata(client, indio_dev);

	st->config = chip_config;

	ret = mpu6050_chip_init(st);
	if (ret) {
		dev_err(&client->dev, "chip init failed: %d\n", ret);
		return ret;
	}

	indio_dev->dev.parent   = &client->dev;
	indio_dev->name         = "mpu6050";
	indio_dev->info         = &mpu6050_info;
	indio_dev->channels     = mpu6050_channels;
	indio_dev->num_channels = ARRAY_SIZE(mpu6050_channels);
	indio_dev->modes        = INDIO_BUFFER_TRIGGERED | INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = scan_masks;


	ret = devm_iio_triggered_buffer_setup(&client->dev, indio_dev,
					      NULL,
					      mpu6050_read_fifo, NULL);
	
	if (ret)
		return ret;

	if (client->irq > 0) {
		st->irq  = client->irq;
		irq_type = irq_get_trigger_type(client->irq);
		if (!irq_type)
			irq_type = IRQF_TRIGGER_RISING;

		ret = mpu6050_probe_trigger(indio_dev, irq_type);
		if (ret)
			return ret;
	} else {
		dev_warn(&client->dev, "no IRQ; buffered mode unavailable\n");
	}

	pm_runtime_enable(&client->dev);
	//pm_runtime_set_autosuspend_delay(&client->dev, 2000);
	//pm_runtime_use_autosuspend(&client->dev);

	ret = devm_iio_device_register(&client->dev, indio_dev);
	if (ret) {
		pm_runtime_disable(&client->dev);
		return ret;
	}

	dev_info(&client->dev, "MPU-6050 registered\n");
	return 0;
}

static void mpu6050_remove(struct i2c_client *client)
{
	struct mpu6050_state *st =
		iio_priv(i2c_get_clientdata(client));

	pm_runtime_disable(&client->dev);
	pm_runtime_set_suspended(&client->dev);
	mpu6050_set_power_mode(st, true);
}

/* ------------------------------------------------------------------ */
/* Device / driver tables                                              */
/* ------------------------------------------------------------------ */

static const struct i2c_device_id mpu6050_id[] = {
	{ "mpu6050", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mpu6050_id);

static const struct of_device_id mpu6050_of_match[] = {
	{ .compatible = "invensense,mpu6050" },
	{ }
};
MODULE_DEVICE_TABLE(of, mpu6050_of_match);

static struct i2c_driver mpu6050_driver = {
	.probe    = mpu6050_probe,
	.remove   = mpu6050_remove,
	.id_table = mpu6050_id,
	.driver   = {
		.name           = "mpu6050",
		.of_match_table = mpu6050_of_match,
		.pm             = pm_ptr(&mpu6050_pm_ops),
	},
};
module_i2c_driver(mpu6050_driver);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("MPU-6050 IMU driver (I2C, no aux bus)");
MODULE_LICENSE("GPL");
