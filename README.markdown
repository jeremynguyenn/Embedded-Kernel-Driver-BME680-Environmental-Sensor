# BME680 Kernel Driver

This is an advanced Linux kernel driver for the Bosch BME680 environmental sensor, integrated with the Industrial I/O (IIO) subsystem. It supports temperature, pressure, humidity, and gas resistance measurements over I2C, with advanced features like file operations, memory management, process management, POSIX-like threads, synchronization, and inter-process communication (IPC).

## Features
- **IIO Integration**: Exposes sensor data via `/sys/bus/iio/devices/iio:deviceX` (e.g., `in_temp_input` for temperature).
- **Character Device**: Provides `/dev/bme680` for user-space access with read, write, and ioctl operations.
- **File Operations**: Supports blocking/non-blocking reads, write for gas heater config, and poll for data-ready events.
- **Memory Management**: Uses `kmalloc`, `kzalloc`, and `kfifo` for efficient kernel memory allocation.
- **Process Management**: Kernel thread (`bme680_poll`) for periodic data polling.
- **Signals**: IIO events for gas measurement completion or errors.
- **Thread Synchronization**: Mutex and spinlock for thread-safe register and FIFO access.
- **IPC**: Kernel FIFO (`kfifo`) for buffered data and shared memory for calibration data.
- **Device Tree Support**: Configurable via `bme680.dts` for Raspberry Pi I2C1.
- **Debugfs**: Exposes stats (reads, errors, thread PID, uptime) at `/sys/kernel/debug/bme680/stats`.
- **Power Management**: Runtime PM for power efficiency.

## Requirements
- Linux kernel with headers (e.g., `raspberrypi-kernel-headers` for Raspberry Pi).
- Device Tree compiler (`dtc`).
- GCC compiler (`gcc`).
- Raspberry Pi with I2C enabled (or other platform with I2C support).
- BME680 sensor connected to I2C1 (address `0x76` or `0x77`).

## Installation

### 1. Prepare the Environment
Install required tools on Raspberry Pi:
```bash
sudo apt-get update
sudo apt-get install raspberrypi-kernel-headers device-tree-compiler gcc
```
Enable I2C:
```bash
sudo raspi-config
# Select Interface Options -> I2C -> Enable
sudo reboot
```

### 2. Copy Files
Place the following files in a directory (e.g., `bme680_driver`):
- `bme680.c`
- `bme680.h`
- `Makefile`
- `bme680.dts`
- `bme680_app.c`

### 3. Build
Build the kernel module, Device Tree overlay, and user application:
```bash
make
```
Output:
- `bme680.ko`: Kernel module.
- `bme680.dtbo`: Device Tree overlay.
- `bme680_app`: User-space application.

For debug mode:
```bash
make BME680_DEBUG=1
```

### 4. Install
Install the module and overlay:
```bash
sudo make install
```
Edit `/boot/config.txt` to add the overlay:
```text
dtoverlay=bme680
```
Reboot:
```bash
sudo reboot
```

### 5. Test
Load and test the module:
```bash
sudo make test
```
Check kernel logs:
```bash
dmesg | grep bme680
```
Run the user application:
```bash
./bme680_app
```
Read data via sysfs:
```bash
cat /sys/bus/iio/devices/iio:device0/in_temp_input
```
Check debug stats:
```bash
cat /sys/kernel/debug/bme680/stats
```

### 6. Cross-Compilation (Optional)
For cross-compiling on a different machine (e.g., for Raspberry Pi):
- Install cross-compiler:
  ```bash
  sudo apt-get install gcc-aarch64-linux-gnu
  ```
- Update `Makefile`:
  ```makefile
  CROSS_COMPILE=aarch64-linux-gnu-
  KERNEL_DIR=/path/to/raspberrypi-kernel-source
  ```
- Build:
  ```bash
  make
  ```

## Usage
- **Read Sensor Data**:
  - Via `/dev/bme680` (FIFO buffer):
    ```bash
    ./bme680_app
    ```
  - Via sysfs:
    ```bash
    cat /sys/bus/iio/devices/iio:device0/in_temp_input  # Temperature in micro °C
    cat /sys/bus/iio/devices/iio:device0/in_pressure_input  # Pressure in Pa
    ```
- **Configure Gas Heater**:
  - Write to `/dev/bme680` with `struct bme680_gas_config` (see `bme680_app.c` for example).
  - Or use module parameter:
    ```bash
    sudo modprobe bme680 heater_temp=320
    ```
- **Monitor Events**:
  - Use `iio_event_monitor` to capture gas measurement events:
    ```bash
    iio_event_monitor /dev/iio:device0
    ```
- **Debug Stats**:
  - Check read counts, errors, thread PID, and uptime:
    ```bash
    cat /sys/kernel/debug/bme680/stats
    ```

## Files
- `bme680.c`: Core kernel driver with IIO, file ops, and IPC.
- `bme680.h`: Header with register definitions, structs, and IIO channels.
- `bme680.dts`: Device Tree overlay for I2C1 on Raspberry Pi.
- `bme680_app.c`: Sample user-space application to read sensor data.
- `Makefile`: Builds module, overlay, and app.

## Notes
- Verify I2C address (`0x76` or `0x77`) with:
  ```bash
  sudo i2cdetect -y 1
  ```
- If using SPI, modify `bme680.dts` and driver for SPI support.
- For STM32 or other platforms, update `PLATFORM` in `Makefile` and adjust `bme680.dts`.
- Test on hardware to debug I2C errors (check `dmesg`).

## License
GPL-2.0

## Authors
- Himanshu Jha (original driver)
- Enhanced by Grok (xAI) for advanced features