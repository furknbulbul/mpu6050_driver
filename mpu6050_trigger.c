// SPDX-License-Identifier: GPL-2.0-only
/*
 * mpu6050_trigger.c - MPU-6050 data-ready interrupt → IIO trigger
 *
 * The hard-IRQ handler captures a timestamp and wakes the threaded
 * handler which fires the IIO trigger.  The trigger state setter
 * enables/disables the FIFO and the data-ready interrupt together.
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>

#include <linux/iio/iio.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>

#include "mpu6050.h"

/* ------------------------------------------------------------------ */
/* Trigger state: enable / disable data acquisition                   */
/* ------------------------------------------------------------------ */

/**
 * mpu6050_set_enable() - Start or stop continuous data capture.
 * @indio_dev: IIO device.
 * @enable:    true = start, false = stop.
 *
 * When starting: bring the required sensor engines online, then arm
 * the hardware FIFO and the data-ready interrupt.
 * When stopping: disarm the FIFO/interrupt, then standby the engines.
 */
static int mpu6050_set_enable(struct iio_dev *indio_dev, bool enable)
{
	struct mpu6050_state *st = iio_priv(indio_dev);
	int ret;
	pr_info("mpu6050_set_enable: %s\n", enable ? "enable" : "disable");

	if (enable) {
		/* Determine which sensors are needed from the scan mask */
		st->config.accel_fifo_enable =
			test_bit(SCAN_ACCEL_X,
				 indio_dev->active_scan_mask) ||
			test_bit(SCAN_ACCEL_Y,
				 indio_dev->active_scan_mask) ||
			test_bit(SCAN_ACCEL_Z,
				 indio_dev->active_scan_mask);

		st->config.gyro_fifo_enable =
			test_bit(SCAN_GYRO_X,
				 indio_dev->active_scan_mask) ||
			test_bit(SCAN_GYRO_Y,
				 indio_dev->active_scan_mask) ||
			test_bit(SCAN_GYRO_Z,
				 indio_dev->active_scan_mask);

		ret = mpu6050_prepare_fifo(st, true);
		pr_info("FIFO enabled: accel %s, gyro %s\n",
			st->config.accel_fifo_enable ? "ON" : "OFF",
			st->config.gyro_fifo_enable  ? "ON" : "OFF");
		if (ret)
			return ret;
		return 0;
	}

	pr_info("disabling FIFO and interrupts\n");

	/* --- Disable path --- */
	ret = mpu6050_prepare_fifo(st, false);
	if (ret)
		return ret;

	return ret;
}

/**
 * mpu6050_trigger_set_state() - IIO trigger callback.
 * @trig:  The IIO trigger instance.
 * @state: Desired state (on/off).
 */
static int mpu6050_trigger_set_state(struct iio_trigger *trig, bool state)
{
	struct iio_dev       *indio_dev = iio_trigger_get_drvdata(trig);
	struct mpu6050_state *st        = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	ret = mpu6050_set_enable(indio_dev, state);
	mutex_unlock(&st->lock);

	return ret;
}

static const struct iio_trigger_ops mpu6050_trigger_ops = {
	.set_trigger_state = mpu6050_trigger_set_state,
};

/* ------------------------------------------------------------------ */
/* Interrupt handlers                                                  */
/* ------------------------------------------------------------------ */

/**
 * mpu6050_irq_timestamp() - Hard-IRQ: record timestamp, wake thread.
 *
 * Must be fast – just saves the time and returns IRQ_WAKE_THREAD.
 */
static irqreturn_t mpu6050_irq_timestamp(int irq, void *dev_id)
{
	struct iio_dev       *indio_dev = dev_id;
	struct mpu6050_state *st        = iio_priv(indio_dev);

	st->it_timestamp = iio_get_time_ns(indio_dev);
	return IRQ_WAKE_THREAD;
}

/**
 * mpu6050_irq_handler() - Threaded-IRQ: check status, fire trigger.
 *
 * Reads INT_STATUS to confirm a data-ready event, then notifies the
 * IIO trigger which will schedule mpu6050_read_fifo() via the poll
 * function.
 */
static irqreturn_t mpu6050_irq_handler(int irq, void *dev_id)
{
	struct iio_dev       *indio_dev = dev_id;
	struct mpu6050_state *st        = iio_priv(indio_dev);
	u8 int_status;
	int ret;
	pr_info("mpu6050: irq handler called.");

	ret = mpu6050_read_reg(st, REG_INT_STATUS, &int_status);
	if (ret) {
		dev_err(&st->client->dev, "failed to read INT_STATUS: %d\n",
			ret);
		return IRQ_HANDLED;
	}

	if (int_status & BIT_DATA_RDY_EN) {
		/* Pass the pre-captured timestamp to the poll function */
		pr_info("data ready, trigger poll");
		indio_dev->pollfunc->timestamp = st->it_timestamp;
		iio_trigger_poll_nested(st->trig);
	}

	if (int_status & BIT_FIFO_OFLOW_INT) {
		dev_warn(&st->client->dev, "FIFO overflow detected\n");
		/* Reset FIFO to clear overflow and resume data capture */
		mpu6050_prepare_fifo(st, false);
		mpu6050_prepare_fifo(st, true);
	}

	return IRQ_HANDLED;
}

/* ------------------------------------------------------------------ */
/* Trigger registration                                                */
/* ------------------------------------------------------------------ */

/**
 * mpu6050_probe_trigger() - Allocate and register the IIO trigger.
 * @indio_dev: IIO device.
 * @irq_type:  IRQF_TRIGGER_* flags (e.g. IRQF_TRIGGER_RISING).
 *
 * Returns 0 on success, negative errno on failure.
 */
int mpu6050_probe_trigger(struct iio_dev *indio_dev, int irq_type)
{
	struct mpu6050_state *st = iio_priv(indio_dev);
	int ret;

	pr_info("probing trigger with IRQ type 0x%x\n", irq_type);

	st->trig = devm_iio_trigger_alloc(&indio_dev->dev,
					  "%s-dev%d",
					  indio_dev->name,
					  iio_device_id(indio_dev));
	if (!st->trig)
		return -ENOMEM;

	irq_type |= IRQF_ONESHOT;

	ret = devm_request_threaded_irq(&indio_dev->dev,
					st->irq,
					mpu6050_irq_timestamp,
					mpu6050_irq_handler,
					irq_type,
					"mpu6050",
					indio_dev);
	if (ret) {
		dev_err(&indio_dev->dev, "IRQ request failed: %d\n", ret);
		return ret;
	}

	st->trig->dev.parent = &st->client->dev;
	st->trig->ops        = &mpu6050_trigger_ops;
	iio_trigger_set_drvdata(st->trig, indio_dev);

	ret = devm_iio_trigger_register(&indio_dev->dev, st->trig);
	if (ret)
		return ret;

	/* Use our own trigger by default */
	indio_dev->trig = iio_trigger_get(st->trig);

	return 0;
}
