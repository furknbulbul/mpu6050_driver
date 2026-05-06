# 1. Modül adı (Çıktı mpu6050.ko olacak)
MODULE_NAME := mpu6050

# 2. Nesne dosyaları (Klasöründeki .c dosyalarıyla birebir aynı olmalı)
obj-m := $(MODULE_NAME).o
$(MODULE_NAME)-y := mpu6050_core.o mpu6050_ring.o mpu6050_trigger.o

# 3. Kernel dizini (Raspberry Pi ve genel Linux için dinamik yol)
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

# 4. Derleme kuralları
all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean

# Yardımcı komutlar
install:
	sudo insmod $(MODULE_NAME).ko

remove:
	sudo rmmod $(MODULE_NAME)