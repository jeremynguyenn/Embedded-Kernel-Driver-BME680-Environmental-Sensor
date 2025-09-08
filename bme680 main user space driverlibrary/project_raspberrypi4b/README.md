### 1. Board

#### 1.1 Board Info

Board Name: Raspberry Pi 4B.

IIC Pin: SCL/SDA GPIO3/GPIO2.

SPI Pin: SCLK/MOSI/MISO/CS GPIO11/GPIO10/GPIO9/GPIO8.

### 2. Install

#### 2.1 Dependencies

Install the necessary dependencies.

```shell
sudo apt-get install libgpiod-dev pkg-config cmake -y
```

#### 2.2 Makefile

Build the project.

```shell
make
```

Install the project and this is optional.

```shell
sudo make install
```

Uninstall the project and this is optional.

```shell
sudo make uninstall
```

#### 2.3 CMake

Build the project.

```shell
mkdir build && cd build 
cmake .. 
make
```

Install the project and this is optional.

```shell
sudo make install
```

Uninstall the project and this is optional.

```shell
sudo make uninstall
```

Test the project and this is optional.

```shell
make test
```

Find the compiled library in CMake. 

```cmake
find_package(bme680 REQUIRED)
```

### 3. BME680

#### 3.1 Command Instruction

1. Show bme680 chip and driver information.

   ```shell
   bme680 (-i | --information)
   ```

2. Show bme680 help.

   ```shell
   bme680 (-h | --help)
   ```

3. Show bme680 pin connections of the current board.

   ```shell
   bme680 (-p | --port)
   ```

4. Run bme680 register test.

   ```shell
   bme680 (-t reg | --test=reg) [--addr=<0 | 1>] [--interface=<iic | spi>]
   ```

5. Run bme680 read test, num means the test times, index is the gas index. 

   ```shell
   bme680 (-t read | --test=read) [--addr=<0 | 1>] [--interface=<iic | spi>] [--index=<index>] [--times=<num>]
   ```

8. Run bme680 read function, num means the read times. 

   ```shell
   bme680 (-e read | --example=read) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
   ```

7. Run bme680 gas function, num means the read times, index is the gas index, ma the is idac ma, degree is the set degree, wait is the wait time in ms.

   ```shell
   bme680 (-e gas | --example=gas) [--addr=<0 | 1>] [--interface=<iic | spi>] [--index=<index>] [--idac=<ma>] [--degree=<degree>] [--wait=<ms>] [--times=<num>]
   ```

#### 3.2 Command Example

```shell
./bme680 -i

bme680: chip is Bosch BME680.
bme680: manufacturer is Bosch.
bme680: interface is IIC SPI.
bme680: driver version is 1.0.
bme680: min supply voltage is 1.7V.
bme680: max supply voltage is 3.6V.
bme680: max current is 0.85mA.
bme680: max temperature is 85.0C.
bme680: min temperature is -40.0C.
```

```shell
./bme680 -p

bme680: SPI interface SCK connected to GPIO11(BCM).
bme680: SPI interface MISO connected to GPIO9(BCM).
bme680: SPI interface MOSI connected to GPIO10(BCM).
bme680: SPI interface CS connected to GPIO8(BCM).
bme680: IIC interface SCL connected to GPIO3(BCM).
bme680: IIC interface SDA connected to GPIO2(BCM).
```

```shell
./bme680 -t reg --addr=0 --interface=iic

bme680: chip is Bosch BME680.
bme680: manufacturer is Bosch.
bme680: interface is IIC SPI.
bme680: driver version is 1.0.
bme680: min supply voltage is 1.7V.
bme680: max supply voltage is 3.6V.
bme680: max current is 0.85mA.
bme680: max temperature is 85.0C.
bme680: min temperature is -40.0C.
bme680: start register test.
bme680: bme680_set_interface/bme680_get_interface test.
bme680: set interface spi.
bme680: check interface ok.
bme680: set interface iic.
bme680: check interface ok.
bme680: bme680_set_addr_pin/bme680_get_addr_pin test.
bme680: set addr pin low.
bme680: check addr pin ok.
bme680: set addr pin high.
bme680: check addr pin ok.
bme680: bme680_set_temperature_oversampling/bme680_get_temperature_oversampling test.
bme680: set temperature oversampling skip.
bme680: check temperature oversampling ok.
bme680: set temperature oversampling x1.
bme680: check temperature oversampling ok.
bme680: set temperature oversampling x2.
bme680: check temperature oversampling ok.
bme680: set temperature oversampling x4.
bme680: check temperature oversampling ok.
bme680: set temperature oversampling x8.
bme680: check temperature oversampling ok.
bme680: set temperature oversampling x16.
bme680: check temperature oversampling ok.
bme680: bme680_set_pressure_oversampling/bme680_get_pressure_oversampling test.
bme680: set pressure oversampling skip.
bme680: check pressure oversampling ok.
bme680: set pressure oversampling x1.
bme680: check pressure oversampling ok.
bme680: set pressure oversampling x2.
bme680: check pressure oversampling ok.
bme680: set pressure oversampling x4.
bme680: check pressure oversampling ok.
bme680: set pressure oversampling x8.
bme680: check pressure oversampling ok.
bme680: set pressure oversampling x16.
bme680: check pressure oversampling ok.
bme680: bme680_set_humidity_oversampling/bme680_get_humidity_oversampling test.
bme680: set humidity oversampling skip.
bme680: check humidity oversampling ok.
bme680: set humidity oversampling x1.
bme680: check humidity oversampling ok.
bme680: set humidity oversampling x2.
bme680: check humidity oversampling ok.
bme680: set humidity oversampling x4.
bme680: check humidity oversampling ok.
bme680: set humidity oversampling x8.
bme680: check humidity oversampling ok.
bme680: set humidity oversampling x16.
bme680: check humidity oversampling ok.
bme680: bme680_set_mode/bme680_get_mode test.
bme680: set mode forced.
bme680: check mode ok.
bme680: set mode sleep.
bme680: check mode ok.
bme680: bme680_set_filter/bme680_get_filter test.
bme680: set filter off.
bme680: check filter ok.
bme680: set filter coeff 1.
bme680: check filter ok.
bme680: set filter coeff 3.
bme680: check filter ok.
bme680: set filter coeff 7.
bme680: check filter ok.
bme680: set filter coeff 15.
bme680: check filter ok.
bme680: set filter coeff 31.
bme680: check filter ok.
bme680: set filter coeff 63.
bme680: check filter ok.
bme680: set filter coeff 127.
bme680: check filter ok.
bme680: bme680_set_spi_wire/bme680_get_spi_wire test.
bme680: set spi wire 3.
bme680: check spi wire ok.
bme680: set spi wire 4.
bme680: check spi wire ok.
bme680: bme680_set_spi_wire_3_data_interrupt/bme680_get_spi_wire_3_data_interrupt test.
bme680: enable spi wire 3 data interrupt.
bme680: check spi wire 3 data interrupt ok.
bme680: disable spi wire 3 data interrupt.
bme680: check spi wire 3 data interrupt ok.
bme680: bme680_get_new_data_status test.
bme680: new data status is false.
bme680: bme680_get_gas_measuring_status test.
bme680: gas measuring status is false.
bme680: bme680_get_measuring_status test.
bme680: get measuring status is true.
bme680: bme680_get_gas_measuring_index test.
bme680: gas measuring index is 0x00.
bme680: bme680_get_gas_valid_status test.
bme680: gas valid status is false.
bme680: bme680_get_heater_stability_status test.
bme680: heater stability status is false.
bme680: bme680_set_heat_off/bme680_get_heat_off test.
bme680: disable heat off.
bme680: check heat off ok.
bme680: enable heat off.
bme680: check heat off ok.
bme680: bme680_set_run_gas/bme680_get_run_gas test.
bme680: enable run gas.
bme680: check run gas ok.
bme680: disable run gas.
bme680: check run gas ok.
bme680: bme680_set_convert_index/bme680_get_convert_index test.
bme680: set convert index 0.
bme680: check convert index ok.
bme680: set convert index 1.
bme680: check convert index ok.
bme680: set convert index 2.
bme680: check convert index ok.
bme680: set convert index 3.
bme680: check convert index ok.
bme680: set convert index 4.
bme680: check convert index ok.
bme680: set convert index 5.
bme680: check convert index ok.
bme680: set convert index 6.
bme680: check convert index ok.
bme680: set convert index 7.
bme680: check convert index ok.
bme680: set convert index 8.
bme680: check convert index ok.
bme680: set convert index 9.
bme680: check convert index ok.
bme680: bme680_set_idac_heat/bme680_get_idac_heat test.
bme680: set index 0 idac heat 0xA3.
bme680: check idac heat ok.
bme680: set index 1 idac heat 0x97.
bme680: check idac heat ok.
bme680: set index 2 idac heat 0xA2.
bme680: check idac heat ok.
bme680: set index 3 idac heat 0x55.
bme680: check idac heat ok.
bme680: set index 4 idac heat 0x53.
bme680: check idac heat ok.
bme680: set index 5 idac heat 0xBE.
bme680: check idac heat ok.
bme680: set index 6 idac heat 0xF1.
bme680: check idac heat ok.
bme680: set index 7 idac heat 0xFC.
bme680: check idac heat ok.
bme680: set index 8 idac heat 0xF9.
bme680: check idac heat ok.
bme680: set index 9 idac heat 0x79.
bme680: check idac heat ok.
bme680: bme680_set_resistance_heat/bme680_get_resistance_heat test.
bme680: set index 0 resistance heat 0x6B.
bme680: check resistance heat ok.
bme680: set index 1 resistance heat 0x52.
bme680: check resistance heat ok.
bme680: set index 2 resistance heat 0x14.
bme680: check resistance heat ok.
bme680: set index 3 resistance heat 0x13.
bme680: check resistance heat ok.
bme680: set index 4 resistance heat 0xE9.
bme680: check resistance heat ok.
bme680: set index 5 resistance heat 0xE2.
bme680: check resistance heat ok.
bme680: set index 6 resistance heat 0x2D.
bme680: check resistance heat ok.
bme680: set index 7 resistance heat 0x51.
bme680: check resistance heat ok.
bme680: set index 8 resistance heat 0x8E.
bme680: check resistance heat ok.
bme680: set index 9 resistance heat 0x1F.
bme680: check resistance heat ok.
bme680: bme680_set_gas_wait/bme680_get_gas_wait test.
bme680: set index 0 gas wait 0x56.
bme680: check gas wait ok.
bme680: set index 1 gas wait 0x08.
bme680: check gas wait ok.
bme680: set index 2 gas wait 0x57.
bme680: check gas wait ok.
bme680: set index 3 gas wait 0x27.
bme680: check gas wait ok.
bme680: set index 4 gas wait 0xA7.
bme680: check gas wait ok.
bme680: set index 5 gas wait 0x05.
bme680: check gas wait ok.
bme680: set index 6 gas wait 0xD4.
bme680: check gas wait ok.
bme680: set index 7 gas wait 0xD0.
bme680: check gas wait ok.
bme680: set index 8 gas wait 0x52.
bme680: check gas wait ok.
bme680: set index 9 gas wait 0x82.
bme680: check gas wait ok.
bme680: bme680_resistance_heat_convert_to_register test.
bme680: 39.29C convert to register 0x26.
bme680: bme680_gas_wait_convert_to_register test.
bme680: 52ms convert to register 0x34.
bme680: bme680_idac_heat_convert_to_register/bme680_idac_heat_convert_to_data test.
bme680: idac heat ma before is 2.20mA.
bme680: idac heat reg is 0x20.
bme680: idac heat ma after is 2.00mA.
bme680: bme680_soft_reset test.
bme680: check soft reset ok.
bme680: finish register test.
```

```shell
./bme680 -t read --addr=0 --interface=iic --times=3

bme680: chip is Bosch BME680.
bme680: manufacturer is Bosch.
bme680: interface is IIC SPI.
bme680: driver version is 1.0.
bme680: min supply voltage is 1.7V.
bme680: max supply voltage is 3.6V.
bme680: max current is 0.85mA.
bme680: max temperature is 85.0C.
bme680: min temperature is -40.0C.
bme680: start read test.
bme680: temperature is 28.92C.
bme680: pressure is 100012.87pa.
bme680: humidity is 57.52%.
bme680: gas index is 0.
bme680: gas resistance is 49387.66ohms.
bme680: temperature is 28.93C.
bme680: pressure is 100012.99pa.
bme680: humidity is 57.39%.
bme680: gas index is 0.
bme680: gas resistance is 34100.94ohms.
bme680: temperature is 28.94C.
bme680: pressure is 100013.12pa.
bme680: humidity is 57.24%.
bme680: gas index is 0.
bme680: gas resistance is 29746.65ohms.
bme680: finish read test.
```

```shell
./bme680 -e read --addr=0 --interface=iic --index=0 --times=3

bme680: 1/3.
bme680: temperature is 28.53C.
bme680: pressure is 100015.96Pa.
bme680: humidity is 58.21%.
bme680: 2/3.
bme680: temperature is 28.53C.
bme680: pressure is 100015.91Pa.
bme680: humidity is 58.22%.
bme680: 3/3.
bme680: temperature is 28.53C.
bme680: pressure is 100015.91Pa.
bme680: humidity is 58.24%.
```

```shell
./bme680 -e gas --addr=0 --interface=iic --index=0 --idac=5.0 --degree=200.0 --wait=150 --times=3

bme680: 1/3.
bme680: temperature is 28.31C.
bme680: pressure is 100015.09Pa.
bme680: humidity is 58.73%.
bme680: gas index is 0.
bme680: idac is 5.00mA.
bme680: degree celsius is 200.0C.
bme680: gas wait is 150 ms.
bme680: gas resistance is 28779.59ohms.
bme680: 2/3.
bme680: temperature is 28.31C.
bme680: pressure is 100015.09Pa.
bme680: humidity is 58.63%.
bme680: gas index is 0.
bme680: idac is 5.00mA.
bme680: degree celsius is 200.0C.
bme680: gas wait is 150 ms.
bme680: gas resistance is 27205.29ohms.
bme680: 3/3.
bme680: temperature is 28.32C.
bme680: pressure is 100015.17Pa.
bme680: humidity is 58.47%.
bme680: gas index is 0.
bme680: idac is 5.00mA.
bme680: degree celsius is 200.0C.
bme680: gas wait is 150 ms.
bme680: gas resistance is 26409.91ohms.
```

```shell
./bme680 -h

Usage:
  bme680 (-i | --information)
  bme680 (-h | --help)
  bme680 (-p | --port)
  bme680 (-t reg | --test=reg) [--addr=<0 | 1>] [--interface=<iic | spi>]
  bme680 (-t read | --test=read) [--addr=<0 | 1>] [--interface=<iic | spi>] [--index=<index>] [--times=<num>]
  bme680 (-e read | --example=read) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]
  bme680 (-e gas | --example=gas) [--addr=<0 | 1>] [--interface=<iic | spi>] [--index=<index>] [--idac=<ma>]
         [--degree=<degree>] [--wait=<ms>] [--times=<num>]

Options:
      --addr=<0 | 1>                 Set the chip iic address.([default: 0])
      --degree=<degree>              Set the heater expected temperature in degree celsius.([default: 200.0])
  -e <read | gas>, --example=<read | gas>
                                     Run the driver example.
  -h, --help                         Show the help.
  -i, --information                  Show the chip information.
      --idac=<ma>                    Set the heater idac current in mA.([default: 5.0])
      --index=<index>                Set the heater index and it range is 0 - 9.([default: 0])
      --interface=<iic | spi>        Set the chip interface.([default: iic])
  -p, --port                         Display the pin connections of the current board.
  -t <reg | read>, --test=<reg | read>
                                     Run the driver test.
      --times=<num>                  Set the running times.([default: 3])
      --wait=<ms>                    Set the heater wait time in ms.([default: 150])
```

