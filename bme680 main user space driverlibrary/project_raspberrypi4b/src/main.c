/**
 * @file      driver_bme680_basic.c
 * @brief     driver bme680 basic source file
 * @version   1.0.0
 * @author    Nguyen Nhan
 * @date      2025-07-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2025/07/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_bme680_read_test.h"
#include "driver_bme680_register_test.h"
#include "driver_bme680_basic.h"
#include "driver_bme680_gas.h"
#include <getopt.h>
#include <stdlib.h>

/**
 * @brief     bme680 full function
 * @param[in] argc arg numbers
 * @param[in] **argv arg address
 * @return    status code
 *            - 0 success
 *            - 1 run failed
 *            - 5 param is invalid
 * @note      none
 */
uint8_t bme680(uint8_t argc, char **argv)
{
    int c;
    int longindex = 0;
    const char short_options[] = "hipe:t:";
    const struct option long_options[] =
    {
        {"help", no_argument, NULL, 'h'},
        {"information", no_argument, NULL, 'i'},
        {"port", no_argument, NULL, 'p'},
        {"example", required_argument, NULL, 'e'},
        {"test", required_argument, NULL, 't'},
        {"addr", required_argument, NULL, 1},
        {"interface", required_argument, NULL, 2},
        {"times", required_argument, NULL, 3},
        {"index", required_argument, NULL, 4},
        {"idac", required_argument, NULL, 5},
        {"degree", required_argument, NULL, 6},
        {"wait", required_argument, NULL, 7},
        {NULL, 0, NULL, 0},
    };
    char type[33] = "unknown";
    uint8_t index = 0;
    uint32_t times = 3;
    float idac_ma = 5.0f;
    float degree_celsius = 200.0f;
    uint16_t gas_wait_ms = 150;
    bme680_interface_t interface = BME680_INTERFACE_IIC;
    bme680_address_t addr = BME680_ADDRESS_ADO_LOW;
    
    /* if no params */
    if (argc == 1)
    {
        /* goto the help */
        goto help;
    }
    
    /* init 0 */
    optind = 0;
    
    /* parse */
    do
    {
        /* parse the args */
        c = getopt_long(argc, argv, short_options, long_options, &longindex);
        
        /* judge the result */
        switch (c)
        {
            /* help */
            case 'h' :
            {
                /* set the type */
                memset(type, 0, sizeof(char) * 33);
                snprintf(type, 32, "h");
                
                break;
            }
            
            /* information */
            case 'i' :
            {
                /* set the type */
                memset(type, 0, sizeof(char) * 33);
                snprintf(type, 32, "i");
                
                break;
            }
            
            /* port */
            case 'p' :
            {
                /* set the type */
                memset(type, 0, sizeof(char) * 33);
                snprintf(type, 32, "p");
                
                break;
            }
            
            /* example */
            case 'e' :
            {
                /* set the type */
                memset(type, 0, sizeof(char) * 33);
                snprintf(type, 32, "e_%s", optarg);
                
                break;
            }
            
            /* test */
            case 't' :
            {
                /* set the type */
                memset(type, 0, sizeof(char) * 33);
                snprintf(type, 32, "t_%s", optarg);
                
                break;
            }
            
            /* running addr pin */
            case 1 :
            {
                if (strcmp("0", optarg) == 0)
                {
                    addr = BME680_ADDRESS_ADO_LOW;
                }
                else if (strcmp("1", optarg) == 0)
                {
                    addr = BME680_ADDRESS_ADO_HIGH;
                }
                else
                {
                    return 5;
                }
                
                break;
            } 
            
            /* running interface */
            case 2 :
            {
                if (strcmp("iic", optarg) == 0)
                {
                    interface = BME680_INTERFACE_IIC;
                }
                else if (strcmp("spi", optarg) == 0)
                {
                    interface = BME680_INTERFACE_SPI;
                }
                else
                {
                    return 5;
                }
                
                break;
            } 
            
            /* running times */
            case 3 :
            {
                /* set the times */
                times = atol(optarg);
                
                break;
            } 
            
            /* index */
            case 4 :
            {
                /* set the index */
                index = (uint8_t)atol(optarg);
                
                break;
            }
            
            /* idac */
            case 5 :
            {
                /* set idac */
                idac_ma = (float)atof(optarg);
                
                break;
            }
            
            /* degree */
            case 6 :
            {
                /* set degree */
                degree_celsius = (float)atof(optarg);
                
                break;
            }
            
            /* wait */
            case 7 :
            {
                /* set wait */
                gas_wait_ms = (uint16_t)atol(optarg);
                
                break;
            }
            
            /* the end */
            case -1 :
            {
                break;
            }
            
            /* others */
            default :
            {
                return 5;
            }
        }
    } while (c != -1);

    /* run the function */
    if (strcmp("t_reg", type) == 0)
    {
        uint8_t res;
        
        /* run the reg test */
        res = bme680_register_test(interface, addr);
        if (res != 0)
        {
            return 1;
        }
        
        return 0;
    }
    else if (strcmp("t_read", type) == 0)
    {
        uint8_t res;
        
        /* run the read test */
        res = bme680_read_test(interface, addr, index, times);
        if (res != 0)
        {
            return 1;
        }
        
        return 0;
    }
    else if (strcmp("e_read", type) == 0)
    {
        uint8_t res;
        uint32_t i;
        float temperature_c;
        float pressure_pa;
        float humidity_percentage;
        
        /* basic init */
        res = bme680_basic_init(interface, addr);
        if (res != 0)
        {
            return 1;
        }
        
        /* loop */
        for (i = 0; i < times; i++)
        {
            /* delay 1000ms */
            bme680_interface_delay_ms(1000);
            
            /* read data */
            res = bme680_basic_read((float *)&temperature_c, (float *)&pressure_pa, (float *)&humidity_percentage);
            if (res != 0)
            {
                (void)bme680_basic_deinit();
                
                return 1;
            }
            
            /* output */
            bme680_interface_debug_print("bme680: %d/%d.\n", i + 1, times);
            bme680_interface_debug_print("bme680: temperature is %0.2fC.\n", temperature_c);
            bme680_interface_debug_print("bme680: pressure is %0.2fPa.\n", pressure_pa);
            bme680_interface_debug_print("bme680: humidity is %0.2f%%.\n", humidity_percentage);
        }
        
        /* deinit */
        (void)bme680_basic_deinit();
        
        return 0;
    }
    else if (strcmp("e_gas", type) == 0)
    {
        uint8_t res;
        uint32_t i;
        float temperature_c;
        float pressure_pa;
        float humidity_percentage;
        float ohms;
        
        /* gas init */
        res = bme680_gas_init(interface, addr);
        if (res != 0)
        {
            return 1;
        }
        
        /* loop */
        for (i = 0; i < times; i++)
        {
            /* delay 1000ms */
            bme680_interface_delay_ms(1000);
            
            /* read data */
            res = bme680_gas_read(idac_ma, degree_celsius, gas_wait_ms, index,
                                 (float *)&temperature_c, (float *)&pressure_pa, (float *)&humidity_percentage, (float *)&ohms);
            if (res != 0)
            {
                (void)bme680_gas_deinit();
                
                return 1;
            }
            
            /* output */
            bme680_interface_debug_print("bme680: %d/%d.\n", i + 1, times);
            bme680_interface_debug_print("bme680: temperature is %0.2fC.\n", temperature_c);
            bme680_interface_debug_print("bme680: pressure is %0.2fPa.\n", pressure_pa);
            bme680_interface_debug_print("bme680: humidity is %0.2f%%.\n", humidity_percentage);
            bme680_interface_debug_print("bme680: gas index is %d.\n", index);
            bme680_interface_debug_print("bme680: idac is %0.2fmA.\n", idac_ma);
            bme680_interface_debug_print("bme680: degree celsius is %0.1fC.\n", degree_celsius);
            bme680_interface_debug_print("bme680: gas wait is %d ms.\n", gas_wait_ms);
            bme680_interface_debug_print("bme680: gas resistance is %0.2fohms.\n", ohms);
        }
        
        /* deinit */
        (void)bme680_gas_deinit();
        
        return 0;
    }
    else if (strcmp("h", type) == 0)
    {
        help:
        bme680_interface_debug_print("Usage:\n");
        bme680_interface_debug_print("  bme680 (-i | --information)\n");
        bme680_interface_debug_print("  bme680 (-h | --help)\n");
        bme680_interface_debug_print("  bme680 (-p | --port)\n");
        bme680_interface_debug_print("  bme680 (-t reg | --test=reg) [--addr=<0 | 1>] [--interface=<iic | spi>]\n");
        bme680_interface_debug_print("  bme680 (-t read | --test=read) [--addr=<0 | 1>] [--interface=<iic | spi>] [--index=<index>] [--times=<num>]\n");
        bme680_interface_debug_print("  bme680 (-e read | --example=read) [--addr=<0 | 1>] [--interface=<iic | spi>] [--times=<num>]\n");
        bme680_interface_debug_print("  bme680 (-e gas | --example=gas) [--addr=<0 | 1>] [--interface=<iic | spi>] [--index=<index>] [--idac=<ma>]\n");
        bme680_interface_debug_print("         [--degree=<degree>] [--wait=<ms>] [--times=<num>]\n");
        bme680_interface_debug_print("\n");
        bme680_interface_debug_print("Options:\n");
        bme680_interface_debug_print("      --addr=<0 | 1>                 Set the chip iic address.([default: 0])\n");
        bme680_interface_debug_print("      --degree=<degree>              Set the heater expected temperature in degree celsius.([default: 200.0])\n");
        bme680_interface_debug_print("  -e <read | gas>, --example=<read | gas>\n");
        bme680_interface_debug_print("                                     Run the driver example.\n");
        bme680_interface_debug_print("  -h, --help                         Show the help.\n");
        bme680_interface_debug_print("  -i, --information                  Show the chip information.\n");
        bme680_interface_debug_print("      --idac=<ma>                    Set the heater idac current in mA.([default: 5.0])\n");
        bme680_interface_debug_print("      --index=<index>                Set the heater index and it range is 0 - 9.([default: 0])\n");
        bme680_interface_debug_print("      --interface=<iic | spi>        Set the chip interface.([default: iic])\n");
        bme680_interface_debug_print("  -p, --port                         Display the pin connections of the current board.\n");
        bme680_interface_debug_print("  -t <reg | read>, --test=<reg | read>\n");
        bme680_interface_debug_print("                                     Run the driver test.\n");
        bme680_interface_debug_print("      --times=<num>                  Set the running times.([default: 3])\n");
        bme680_interface_debug_print("      --wait=<ms>                    Set the heater wait time in ms.([default: 150])\n");
        
        return 0;
    }
    else if (strcmp("i", type) == 0)
    {
        bme680_info_t info;
        
        /* print bme680 info */
        bme680_info(&info);
        bme680_interface_debug_print("bme680: chip is %s.\n", info.chip_name);
        bme680_interface_debug_print("bme680: manufacturer is %s.\n", info.manufacturer_name);
        bme680_interface_debug_print("bme680: interface is %s.\n", info.interface);
        bme680_interface_debug_print("bme680: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
        bme680_interface_debug_print("bme680: min supply voltage is %0.1fV.\n", info.supply_voltage_min_v);
        bme680_interface_debug_print("bme680: max supply voltage is %0.1fV.\n", info.supply_voltage_max_v);
        bme680_interface_debug_print("bme680: max current is %0.2fmA.\n", info.max_current_ma);
        bme680_interface_debug_print("bme680: max temperature is %0.1fC.\n", info.temperature_max);
        bme680_interface_debug_print("bme680: min temperature is %0.1fC.\n", info.temperature_min);
        
        return 0;
    }
    else if (strcmp("p", type) == 0)
    {
        /* print pin connection */
        bme680_interface_debug_print("bme680: SPI interface SCK connected to GPIO11(BCM).\n");
        bme680_interface_debug_print("bme680: SPI interface MISO connected to GPIO9(BCM).\n");
        bme680_interface_debug_print("bme680: SPI interface MOSI connected to GPIO10(BCM).\n");
        bme680_interface_debug_print("bme680: SPI interface CS connected to GPIO8(BCM).\n");
        bme680_interface_debug_print("bme680: IIC interface SCL connected to GPIO3(BCM).\n");
        bme680_interface_debug_print("bme680: IIC interface SDA connected to GPIO2(BCM).\n");
        
        return 0;
    }
    else
    {
        return 5;
    }
}

/**
 * @brief     main function
 * @param[in] argc arg numbers
 * @param[in] **argv arg address
 * @return    status code
 *             - 0 success
 * @note      none
 */
int main(uint8_t argc, char **argv)
{
    uint8_t res;

    res = bme680(argc, argv);
    if (res == 0)
    {
        /* run success */
    }
    else if (res == 1)
    {
        bme680_interface_debug_print("bme680: run failed.\n");
    }
    else if (res == 5)
    {
        bme680_interface_debug_print("bme680: param is invalid.\n");
    }
    else
    {
        bme680_interface_debug_print("bme680: unknown status code.\n");
    }

    return 0;
}
