echo "1-0068" | sudo tee /sys/bus/i2c/drivers/mpu6050/unbind && \
sudo rmmod mpu6050 && \
make && \
sudo insmod mpu6050.ko && \
echo 1 | sudo tee /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_x_en && \
echo 1 | sudo tee /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_y_en && \
echo 1 | sudo tee /sys/bus/iio/devices/iio:device0/scan_elements/in_accel_z_en && \
echo 1 | sudo tee /sys/bus/iio/devices/iio:device0/buffer0/enable