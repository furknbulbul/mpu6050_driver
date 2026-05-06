// SPDX-License-Identifier: GPL-2.0-only
/*
 * mpu6050_ring.c - MPU-6050 hardware FIFO → IIO kfifo bridge
 *
 * Called from the trigger poll function when data-ready fires.
 * Drains the chip FIFO into the IIO buffer, adding a timestamp to
 * each sample.
 */

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/slab.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/trigger_consumer.h>

#include "mpu6050.h"

/**
 * mpu6050_read_fifo() - Bottom-half: drain the chip FIFO into the IIO buffer.
 * @irq: Not used (required by the irqreturn_t prototype).
 * @p:   Pointer to the IIO poll function (carries indio_dev + timestamp).
 *
 * Layout of one sample in the chip FIFO (with all three sensors enabled):
 *
 *   Bytes  0- 5  Accelerometer  XH XL YH YL ZH ZL  (big-endian 16-bit)
 *   Bytes  6- 7  Temperature    TH TL
 *   Bytes  8-13  Gyroscope      XH XL YH YL ZH ZL
 *
 * The same layout is pushed into the IIO kfifo, followed by a 64-bit
 * timestamp aligned to 8 bytes (see MPU6050_OUTPUT_DATA_SIZE).
 */

irqreturn_t mpu6050_read_fifo(int irq, void *p)
{
	struct iio_poll_func   *pf        = p;
	struct iio_dev         *indio_dev = pf->indio_dev;
	struct mpu6050_state   *st        = iio_priv(indio_dev);
	u8   fifo_count_buf[2];
	u16  fifo_count;
	size_t bytes_per_datum = 0;
	size_t nb_samples, i;
	int  ret;

	mutex_lock(&st->lock);

	/* ------------------------------------------------------------ */
	/* 1. Determine how many complete samples are waiting            */
	/* ------------------------------------------------------------ */

	if (!st->config.accel_fifo_enable &&
	    !st->config.gyro_fifo_enable)
		goto end_session;

	if (st->config.accel_fifo_enable)
		bytes_per_datum += BYTES_PER_3AXIS;
	if (st->config.gyro_fifo_enable)
		bytes_per_datum += BYTES_PER_3AXIS;

	ret = mpu6050_read_burst(st, REG_FIFO_COUNT_H,
				 fifo_count_buf, sizeof(fifo_count_buf));
	if (ret < 0)
		goto end_session;

	fifo_count = (u16)fifo_count_buf[0] << 8 | fifo_count_buf[1];

	/* Handle overflow: reset and bail */
	if (fifo_count >= FIFO_SIZE - 3 * bytes_per_datum) {
		dev_warn(&st->client->dev, "FIFO overflow – resetting\n");
		mpu6050_prepare_fifo(st, false);
		mpu6050_prepare_fifo(st, true);
		goto end_session;
	}

	nb_samples = fifo_count / bytes_per_datum;
	if (!nb_samples)
		goto end_session;

	/* ------------------------------------------------------------ */
	/* 2. Read all complete samples in one burst and push to kfifo   */
	/* ------------------------------------------------------------ */

	for (i = 0; i < nb_samples; i++) {
		/*
		 * Clear scratch buffer so there are no stale bytes near
		 * the timestamp slot (kernel data-leak prevention).
		 */
		memset(st->data, 0, sizeof(st->data));

		ret = mpu6050_read_burst(st, REG_FIFO_R_W,
					 st->data, bytes_per_datum);
		if (ret < 0)
			goto flush_fifo;

		/*
		 * iio_push_to_buffers_with_timestamp() appends the
		 * timestamp captured in the hard-IRQ handler
		 * (st->it_timestamp) after the sensor data.
		 */
		iio_push_to_buffers_with_timestamp(indio_dev,
						   st->data,
						   st->it_timestamp);
	}

end_session:
	mutex_unlock(&st->lock);
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;

flush_fifo:
	mpu6050_prepare_fifo(st, false);
	mpu6050_prepare_fifo(st, true);
	mutex_unlock(&st->lock);
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}
