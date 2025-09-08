/**
 * @file      driver_bme680_basic.c
 * @brief     driver bme680 basic source file
 * @version   1.0.0
 * @author    Nguyen Nhan
 * @date      2025-07-15
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2025/07/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_bme680_read_test.h"

static bme680_handle_t gs_handle;        /**< bme680 handle */

/**
 * @brief     read test
 * @param[in] interface chip interface
 * @param[in] addr_pin chip address pin
 * @param[in] index input index
 * @param[in] times test times
 * @return    status code
 *            - 0 success
 *            - 1 test failed
 * @note      none
 */
uint8_t bme680_read_test(bme680_interface_t interface, bme680_address_t addr_pin, uint8_t index, uint32_t times)
{
    uint8_t res;
    uint8_t reg;
    uint32_t i;
    uint32_t temperature_raw;
    uint32_t pressure_raw;
    uint32_t humidity_raw;
    float temperature_c;
    float pressure_pa;
    float humidity_percentage;
    uint16_t adc_raw;
    uint8_t adc_range;
    float ohms;
    uint8_t index_output;
    bme680_info_t info;
    
    /* link interface function */
    DRIVER_BME680_LINK_INIT(&gs_handle, bme680_handle_t); 
    DRIVER_BME680_LINK_IIC_INIT(&gs_handle, bme680_interface_iic_init);
    DRIVER_BME680_LINK_IIC_DEINIT(&gs_handle, bme680_interface_iic_deinit);
    DRIVER_BME680_LINK_IIC_READ(&gs_handle, bme680_interface_iic_read);
    DRIVER_BME680_LINK_IIC_WRITE(&gs_handle, bme680_interface_iic_write);
    DRIVER_BME680_LINK_SPI_INIT(&gs_handle, bme680_interface_spi_init);
    DRIVER_BME680_LINK_SPI_DEINIT(&gs_handle, bme680_interface_spi_deinit);
    DRIVER_BME680_LINK_SPI_READ(&gs_handle, bme680_interface_spi_read);
    DRIVER_BME680_LINK_SPI_WRITE(&gs_handle, bme680_interface_spi_write);
    DRIVER_BME680_LINK_DELAY_MS(&gs_handle, bme680_interface_delay_ms);
    DRIVER_BME680_LINK_DEBUG_PRINT(&gs_handle, bme680_interface_debug_print);
    
    /* get bme680 info */
    res = bme680_info(&info);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get info failed.\n");
       
        return 1;
    }
    else
    {
        /* print bme680 */
        bme680_interface_debug_print("bme680: chip is %s.\n", info.chip_name);
        bme680_interface_debug_print("bme680: manufacturer is %s.\n", info.manufacturer_name);
        bme680_interface_debug_print("bme680: interface is %s.\n", info.interface);
        bme680_interface_debug_print("bme680: driver version is %d.%d.\n", info.driver_version / 1000, (info.driver_version % 1000) / 100);
        bme680_interface_debug_print("bme680: min supply voltage is %0.1fV.\n", info.supply_voltage_min_v);
        bme680_interface_debug_print("bme680: max supply voltage is %0.1fV.\n", info.supply_voltage_max_v);
        bme680_interface_debug_print("bme680: max current is %0.2fmA.\n", info.max_current_ma);
        bme680_interface_debug_print("bme680: max temperature is %0.1fC.\n", info.temperature_max);
        bme680_interface_debug_print("bme680: min temperature is %0.1fC.\n", info.temperature_min);
    }
    
    /* start read test */
    bme680_interface_debug_print("bme680: start read test.\n");
    
    /* set interface */
    res = bme680_set_interface(&gs_handle, interface);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set interface failed.\n");
       
        return 1;
    }
    
    /* set addr pin */
    res = bme680_set_addr_pin(&gs_handle, addr_pin);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set addr pin failed.\n");
       
        return 1;
    }
    
    /* init */
    res = bme680_init(&gs_handle);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: init failed.\n");
       
        return 1;
    }
    
    /* set spi wire4 */
    res = bme680_set_spi_wire(&gs_handle, BME680_SPI_WIRE_4);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set spi wire failed.\n");
        (void)bme680_deinit(&gs_handle);
        
        return 1;
    }
    
    /* set humidity oversampling x1 */
    res = bme680_set_humidity_oversampling(&gs_handle, BME680_OVERSAMPLING_x1);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* set temperature oversampling x2 */
    res = bme680_set_temperature_oversampling(&gs_handle, BME680_OVERSAMPLING_x2);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* set pressure oversampling x16 */
    res = bme680_set_pressure_oversampling(&gs_handle, BME680_OVERSAMPLING_x16);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* set filter 15 */
    res = bme680_set_filter(&gs_handle, BME680_FILTER_COEFF_15);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* set sleep mode */
    res = bme680_set_mode(&gs_handle, BME680_MODE_SLEEP);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set mode failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* 5.0mA */
    res = bme680_idac_heat_convert_to_register(&gs_handle, 5.0f, &reg);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: idac heat convert to register failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* set idac heat */
    res = bme680_set_idac_heat(&gs_handle, index, reg);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set idac heat failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* 200C */
    res = bme680_resistance_heat_convert_to_register(&gs_handle, 200.0f, &reg);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: resistance heat convert to register failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* set resistance heat */
    res = bme680_set_resistance_heat(&gs_handle, index, reg);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set resistance heat failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* 150ms */
    res = bme680_gas_wait_convert_to_register(&gs_handle, 150, &reg);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: gas wait convert to register failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    res = bme680_set_gas_wait(&gs_handle, index, reg);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set gas wait failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* set convert index */
    res = bme680_set_convert_index(&gs_handle, index);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set convert index failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* disable heat off */
    res = bme680_set_heat_off(&gs_handle, BME680_BOOL_FALSE);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set heat off failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* enable run gas */
    res = bme680_set_run_gas(&gs_handle, BME680_BOOL_TRUE);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set run gas failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* delay 1000ms */
    bme680_interface_delay_ms(1000);
    
    /* loop */
    for (i = 0; i < times; i++)
    {
        /* read */
        res = bme680_read(&gs_handle, &temperature_raw, &temperature_c, 
                          &pressure_raw, &pressure_pa,
                          &humidity_raw, &humidity_percentage,
                          &adc_raw, &adc_range, &ohms, &index_output);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: read failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        
        /* delay 1000ms */
        bme680_interface_delay_ms(1000);
        
        /* output */
        bme680_interface_debug_print("bme680: temperature is %0.2fC.\n", temperature_c);
        bme680_interface_debug_print("bme680: pressure is %0.2fpa.\n", pressure_pa);
        bme680_interface_debug_print("bme680: humidity is %0.2f%%.\n", humidity_percentage);
        bme680_interface_debug_print("bme680: gas index is %d.\n", index_output);
        bme680_interface_debug_print("bme680: gas resistance is %0.2fohms.\n", ohms);
    }
    
    /* finish read test */
    bme680_interface_debug_print("bme680: finish read test.\n");
    (void)bme680_set_heat_off(&gs_handle, BME680_BOOL_TRUE);
    (void)bme680_deinit(&gs_handle); 
    
    return 0;
}
