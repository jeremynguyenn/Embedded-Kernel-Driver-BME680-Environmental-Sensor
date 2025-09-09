# BME680 Kernel Driver
<img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/2121961c-8947-4c0f-953d-a13099410ca5" />
<img width="225" height="225" alt="image" src="https://github.com/user-attachments/assets/32c2195e-acaf-4ee9-91bc-9dad5084613a" />

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
- BME680 sensor connected to I2C1 (address `0x76` or `0x77` depending on SDO pin).
- A working I2C bus (e.g., I2C1 on Raspberry Pi 4B) and GPIO 18 free for power control (as specified in `bme680.dts`).
- Sufficient permissions (e.g., `sudo` for module loading, or add user to `iio` group for sysfs access).

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
Verify I2C bus and sensor connectivity:
```bash
sudo i2cdetect -y 1
```
Expected output: `0x76` (or `0x77` if SDO pin is high) should appear in the I2C device map.

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
Expected output:
```bash
Checking for required tools...
gcc: command found
dtc: command found
Kernel directory /lib/modules/$(uname -r)/build exists
Compiling Device Tree Overlay bme680.dts...
Building user application bme680_app...
gcc -g -O2 -Wall -o bme680_app bme680_app.c -lm
Building kernel module...
make -C /lib/modules/$(uname -r)/build M=$(pwd) modules EXTRA_CFLAGS="-DCONFIG_BME680_DEBUG=0"
...
```
Output files:
- `bme680.ko`: Kernel module (~50-100 KB).
- `bme680.dtbo`: Device Tree overlay (~1-2 KB).
- `bme680_app`: User-space application (~10-20 KB).

For debug mode (enables verbose kernel logs):
```bash
make BME680_DEBUG=1
```
**Note**: If build fails, check for:
- Missing kernel headers (`sudo apt install raspberrypi-kernel-headers`).
- Incorrect kernel version (ensure `uname -r` matches `KERNEL_DIR`).
- Syntax errors in `bme680.c` (e.g., incomplete code due to truncation).

### 4. Install
Install the module and overlay:
```bash
sudo make install
```
This copies:
- `bme680.ko` to `/lib/modules/$(uname -r)/kernel/drivers/iio/chemical/`.
- `bme680.dtbo` to `/boot/overlays/`.

Edit `/boot/config.txt` to add the overlay:
```text
dtoverlay=bme680
```
Reboot to apply:
```bash
sudo reboot
```
**Verify Overlay**: After reboot, check if the overlay is applied:
```bash
sudo vcgencmd get_config arm
# Should show: dtoverlay=bme680
```
Check device tree:
```bash
ls /proc/device-tree/soc/i2c@7e804000/sensor@76
# Should list properties like compatible, reg, bosch,heater-temp
```

### 5. Test
Load and test the module:
```bash
sudo make test
```
Expected output:
```bash
Testing kernel module...
Loading bme680 module...
sudo insmod ./bme680.ko heater_temp=320
Module loaded. Check 'dmesg' for logs.
Run './bme680_app' to test user application.
```

Check kernel logs:
```bash
dmesg | grep bme680
```
Expected log:
```
[   5.123456] bme680_i2c 1-0076: BME680 Advanced IIO Driver Probed
```

Run the user application (default: read via `/dev/bme680` every 500ms):
```bash
./bme680_app
```
Example output:
```
Temperature: 25.42 °C
Pressure: 1013.25 hPa
Humidity: 45.67 %
Gas Resistance: 123456 Ohms
Timestamp: 1234567890 ns
```

Read data via sysfs (alternative):
```bash
cat /sys/bus/iio/devices/iio:device0/in_temp_input       # Temperature in micro °C
cat /sys/bus/iio/devices/iio:device0/in_pressure_input   # Pressure in kPa
cat /sys/bus/iio/devices/iio:device0/in_humidityrelative_input  # Humidity in micro %
cat /sys/bus/iio/devices/iio:device0/in_resistance_input # Gas resistance in Ohms
```

Check debug stats:
```bash
cat /sys/kernel/debug/bme680/stats
```
Example output:
```
Reads: 100
Errors: 0
Thread PID: 1234
Uptime: 3600 seconds
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
- Copy `bme680.ko`, `bme680.dtbo`, and `bme680_app` to the Raspberry Pi (e.g., via `scp`).
**Note**: Ensure the kernel source matches the target Raspberry Pi’s kernel version (`uname -r`).

## Usage
- **Read Sensor Data**:
  - Via `/dev/bme680` (FIFO buffer, recommended for real-time):
    ```bash
    ./bme680_app [-i interval_ms] [-n num_reads]  # e.g., ./bme680_app -i 1000 -n 10
    ```
  - Via sysfs (simpler, but no timestamp):
    ```bash
    ./bme680_app -s
    ```
  - Example sysfs output:
    ```
    Temperature: 25.42 °C
    Pressure: 1013.25 hPa
    Humidity: 45.67 %
    Gas Resistance: 123456 Ohms
    ```

- **Configure Gas Heater**:
  - Write to `/dev/bme680` with `struct bme680_gas_config` (see `bme680_app.c` for example).
    ```bash
    ./bme680_app  # Sets heater_temp=320°C, heater_dur=150ms, preheat_curr_ma=10mA
    ```
  - Or use module parameter at load time:
    ```bash
    sudo modprobe bme680 heater_temp=320
    ```
  - **Note**: Gas resistance readings require proper heater configuration (200–400°C, as per `bme680.h`).

- **Monitor Events**:
  - Use `iio_event_monitor` to capture gas measurement events:
    ```bash
    iio_event_monitor /dev/iio:device0
    ```
  - Example event output:
    ```
    Event: type=change, direction=either, channel=resistance, value=123456
    ```

- **Debug Stats**:
  - Check read counts, errors, thread PID, and uptime:
    ```bash
    cat /sys/kernel/debug/bme680/stats
    ```

- **Advanced Usage**:
  - Adjust oversampling for accuracy (via sysfs or driver code):
    ```bash
    echo 4 > /sys/bus/iio/devices/iio:device0/oversampling_temp  # Set to x8
    ```
  - Modify polling interval in `bme680_app` with `-i` (e.g., `-i 100` for 100ms).
  - Use `BME680_IOC_READ_FIFO` ioctl for timestamped FIFO data (see `bme680_app.c`).

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
- If using SPI, modify `bme680.dts` and driver for SPI support (not implemented in provided code).
- For STM32 or other platforms, update `PLATFORM` in `Makefile` and adjust `bme680.dts` for compatibility.
- Test on hardware to debug I2C errors (check `dmesg` for errors like `chip_id mismatch` or `I2C transfer failed`).
- **Hardware Notes**:
  - Ensure BME680 is powered correctly (GPIO 18 active-high in `bme680.dts`).
  - Check SDO pin (GND for 0x76, VCC for 0x77).
  - Gas sensor requires 20–30s warmup for stable readings.
- **Troubleshooting**:
  - No `/dev/bme680`: Check if module loaded (`lsmod | grep bme680`) and overlay applied.
  - No data: Verify I2C connection (`i2cdetect`) and sensor power.
  - Permission denied: Run `bme680_app` with `sudo` or add user to `iio` group (`sudo usermod -aG iio $USER`).
  - Check `dmesg` for errors like `i2c probe failed` or `invalid chip ID`.

## License
GPL-2.0

## Authors
- Himanshu Jha (original driver)
- Enhanced by Grok (xAI) for advanced features

## Version
- Driver Version: 3.0.0 (as defined in `Makefile`)

## Acknowledgments
- Bosch Sensortec for BME680 datasheet and reference algorithms.
- Linux IIO community for subsystem documentation.
- Raspberry Pi community for Device Tree and kernel module guides.
