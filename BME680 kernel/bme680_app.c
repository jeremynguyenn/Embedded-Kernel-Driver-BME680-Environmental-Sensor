#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include "bme680.h"

int main(void) {
    int fd = open("/dev/bme680", O_RDONLY);
    if (fd < 0) {
        perror("Failed to open /dev/bme680");
        return 1;
    }

    struct bme680_fifo_data data;
    if (ioctl(fd, BME680_IOC_READ_FIFO, &data) < 0) {
        perror("Failed to read FIFO");
        close(fd);
        return 1;
    }

    printf("Temperature: %.2f °C\n", data.temperature / 1000000.0);
    printf("Pressure: %.2f Pa\n", data.pressure / 1000.0);
    printf("Humidity: %.2f %%\n", data.humidity / 1000.0);
    printf("Gas Resistance: %u Ohms\n", data.gas_resistance);
    printf("Timestamp: %lld ns\n", data.timestamp);

    close(fd);
    return 0;
}