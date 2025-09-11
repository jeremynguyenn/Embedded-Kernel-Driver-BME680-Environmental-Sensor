#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <getopt.h>
#include "bme680.h"

int main(int argc, char *argv[]) {
    int opt;
    int interval_ms = 500; // Mặc định 500ms
    int num_reads = 0;     // 0 = vô hạn
    int use_sysfs = 0;     // Mặc định dùng ioctl

    while ((opt = getopt(argc, argv, "i:n:s")) != -1) {
        switch (opt) {
            case 'i': interval_ms = atoi(optarg); break;
            case 'n': num_reads = atoi(optarg); break;
            case 's': use_sysfs = 1; break;
            default:
                fprintf(stderr, "Usage: %s [-i interval_ms] [-n num_reads] [-s]\n", argv[0]);
                return 1;
        }
    }

    if (use_sysfs) {
        while (num_reads == 0 || num_reads-- > 0) {
            FILE *temp_file = fopen("/sys/bus/iio/devices/iio:device0/in_temp_input", "r");
            FILE *press_file = fopen("/sys/bus/iio/devices/iio:device0/in_pressure_input", "r");
            FILE *humid_file = fopen("/sys/bus/iio/devices/iio:device0/in_humidityrelative_input", "r");
            FILE *gas_file = fopen("/sys/bus/iio/devices/iio:device0/in_resistance_input", "r");
            if (!temp_file || !press_file || !humid_file || !gas_file) {
                perror("Failed to open sysfs files");
                return 1;
            }

            float temp, press, humid, gas;
            fscanf(temp_file, "%f", &temp);
            fscanf(press_file, "%f", &press);
            fscanf(humid_file, "%f", &humid);
            fscanf(gas_file, "%f", &gas);
            printf("Temperature: %.2f °C\n", temp / 1000.0);
            printf("Pressure: %.2f hPa\n", press * 10.0); // IIO trả về kPa
            printf("Humidity: %.2f %%\n", humid / 1000.0);
            printf("Gas Resistance: %.0f Ohms\n", gas);
            fclose(temp_file);
            fclose(press_file);
            fclose(humid_file);
            fclose(gas_file);
            usleep(interval_ms * 1000);
        }
    } else {
        int fd = open("/dev/bme680", O_RDONLY);
        if (fd < 0) {
            perror("Failed to open /dev/bme680");
            return 1;
        }

        struct bme680_gas_config gas_config = {
            .heater_temp = 320,
            .heater_dur = 150,
            .preheat_curr_ma = 10
        };
        if (ioctl(fd, BME680_IOC_SET_GAS_CONFIG, &gas_config) < 0) {
            perror("Failed to set gas config");
            close(fd);
            return 1;
        }

        struct bme680_fifo_data data;
        while (num_reads == 0 || num_reads-- > 0) {
            int retries = 5;
            while (retries--) {
                if (ioctl(fd, BME680_IOC_READ_FIFO, &data) == 0) {
                    break;
                }
                if (errno == EAGAIN) {
                    usleep(100000); // Chờ 100ms nếu FIFO trống
                    continue;
                }
                perror("Failed to read FIFO");
                close(fd);
                return 1;
            }
            if (retries < 0) {
                fprintf(stderr, "Failed to read FIFO after retries\n");
                close(fd);
                return 1;
            }

            printf("Temperature: %.2f °C\n", data.temperature / 1000000.0);
            printf("Pressure: %.2f hPa\n", data.pressure / 100000.0);
            printf("Humidity: %.2f %%\n", data.humidity / 1000.0);
            printf("Gas Resistance: %u Ohms\n", data.gas_resistance);
            printf("Timestamp: %lld ns\n\n", data.timestamp);
            usleep(interval_ms * 1000);
        }
        close(fd);
    }

    return 0;
}