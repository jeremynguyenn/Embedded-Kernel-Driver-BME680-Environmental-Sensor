obj-m += bme680.o

KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
CC := gcc
DTC := dtc
APP := bme680_app
DTS_FILES := bme680.dts
DTBO_FILES := $(DTS_FILES:.dts=.dtbo)
PLATFORM ?= raspberry
VERSION := 3.2.0

CFLAGS := -g -O2 -Wall -pthread -lrt -lseccomp
KCFLAGS := -DCONFIG_BME680_DEBUG=$(BME680_DEBUG) -DCONFIG_VMALLOC=y -DCONFIG_NETLINK=y -DCONFIG_HWMON=y -DCONFIG_TRACEPOINTS=y -DCONFIG_SPI=y
BME680_DEBUG ?= 0

MAKEFLAGS += -j$(shell nproc)

all: check-tools dt app module

module:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules EXTRA_CFLAGS="$(KCFLAGS)"

dt: $(DTBO_FILES)
$(DTBO_FILES): %.dtbo: %.dts
	$(DTC) -@ -I dts -O dtb -o $@ $<

app: $(APP).c bme680.h
	$(CC) $(CFLAGS) -o $(APP) $(APP).c -lm

install: module dt
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
	mkdir -p /boot/overlays
	for dtbo in $(DTBO_FILES); do cp $$dtbo /boot/overlays/; done
	depmod -a
	echo "Add 'dtoverlay=bme680' to /boot/config.txt"

test: install
	if lsmod | grep -q bme680; then sudo rmmod bme680; fi
	sudo insmod $(PWD)/bme680.ko heater_temp=320
	echo "Run './$(APP) -i 1000' (check hwmon, netlink, IAQ)"

plot:
	gnuplot -e "set terminal png; plot 'data.log' with lines" > plot.png

backup:
	mkdir -p backup
	TIMESTAMP=$$(date +%F-%H%M%S); \
	for dtbo in $(DTBO_FILES); do [ -f /boot/overlays/$$dtbo ] && cp /boot/overlays/$$dtbo backup/$$dtbo-$$TIMESTAMP; done; \
	[ -f /lib/modules/$(uname -r)/kernel/drivers/iio/chemical/bme680.ko ] && cp /lib/modules/$(uname -r)/kernel/drivers/iio/chemical/bme680.ko backup/bme680.ko-$$TIMESTAMP

clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f $(DTBO_FILES) $(APP) plot.png
	rm -f *.o *.ko *.mod *.mod.c *.symvers *.order .*.cmd
	rm -rf .tmp_versions

check-tools:
	which $(CC) >/dev/null || (echo "Error: gcc not found"; exit 1)
	which $(DTC) >/dev/null || (echo "Error: dtc not found"; exit 1)
	pkg-config --exists pthread seccomp || echo "Warning: pthread/seccomp not found"
	[ -d $(KERNEL_DIR) ] || (echo "Error: Kernel dir not found"; exit 1)

version:
	echo "$(VERSION)" > version.txt

ifeq ($(PLATFORM),raspberry)
    DTS_FILES := bme680.dts
endif

.PHONY: all module dt app install test plot backup clean check-tools version