# Define kernel modules to build
obj-m += bme680.o

# Define variables
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)
CC := gcc
DTC := dtc
APP := bme680_app
DTS_FILES := bme680.dts
DTBO_FILES := $(DTS_FILES:.dts=.dtbo)
# Default platform: raspberry
PLATFORM ?= raspberry
# Driver version
VERSION := 3.0.0

# Compiler flags
CFLAGS := -g -O2 -Wall
# Kernel debug flag
KCFLAGS := -DCONFIG_BME680_DEBUG=$(BME680_DEBUG)
BME680_DEBUG ?= 0

# Parallel build support
MAKEFLAGS += -j$(shell nproc)

# Default target
all: check-tools dt app module

# Build kernel module
module:
	@echo "Building kernel module..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules EXTRA_CFLAGS="$(KCFLAGS)"

# Build Device Tree overlays
dt: $(DTBO_FILES)
$(DTBO_FILES): %.dtbo: %.dts
	@echo "Compiling Device Tree Overlay $<..."
	$(DTC) -@ -I dts -O dtb -o $@ $<

# Build user application (giả sử bme680_app.c tồn tại)
app: $(APP).c
	@echo "Building user application $(APP)..."
	$(CC) $(CFLAGS) -o $(APP) $(APP).c -lm

# Install kernel module and Device Tree overlays
install: module dt
	@echo "Installing kernel module and Device Tree overlays..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
	@mkdir -p /boot/overlays
	@for dtbo in $(DTBO_FILES); do \
		cp $$dtbo /boot/overlays/; \
		echo "Installed $$dtbo to /boot/overlays/"; \
	done
	@depmod -a
	@echo "Add 'dtoverlay=bme680' to /boot/config.txt for Raspberry Pi"

# Test module (load/unload)
test: install
	@echo "Testing kernel module..."
	@if lsmod | grep -q bme680; then \
		echo "Unloading existing bme680 module..."; \
		sudo rmmod bme680; \
	fi
	@echo "Loading bme680 module..."
	@sudo insmod $(PWD)/bme680.ko heater_temp=320
	@echo "Module loaded. Check 'dmesg' for logs."
	@echo "Run './$(APP)' to test user application."

# Backup existing files
backup:
	@echo "Backing up existing files..."
	@mkdir -p backup
	@for dtbo in $(DTBO_FILES); do \
		[ -f /boot/overlays/$$dtbo ] && cp /boot/overlays/$$dtbo backup/$$dtbo-$(shell date +%F-%H%M%S); \
	done
	@[ -f /lib/modules/$(shell uname -r)/kernel/drivers/iio/chemical/bme680.ko ] && \
		cp /lib/modules/$(shell uname -r)/kernel/drivers/iio/chemical/bme680.ko backup/bme680.ko-$(shell date +%F-%H%M%S)

# Clean up generated files
clean:
	@echo "Cleaning up..."
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f $(DTBO_FILES) $(APP)
	rm -f *.o *.ko *.mod *.mod.c *.symvers *.order .*.cmd
	rm -rf .tmp_versions

# Check for required tools
check-tools:
	@echo "Checking for required tools..."
	@which $(CC) > /dev/null || (echo "Error: gcc not found"; exit 1)
	@which $(DTC) > /dev/null || (echo "Error: dtc not found"; exit 1)
	@[ -d $(KERNEL_DIR) ] || (echo "Error: Kernel directory $(KERNEL_DIR) not found"; exit 1)

# Generate version file
version:
	@echo "Generating version file..."
	@echo "$(VERSION)" > version.txt
	@echo "Driver version: $(VERSION)"

# Platform-specific configuration
ifeq ($(PLATFORM),raspberry)
    DTS_FILES := bme680.dts
endif

# Phony targets
.PHONY: all module dt app install test backup clean check-tools version