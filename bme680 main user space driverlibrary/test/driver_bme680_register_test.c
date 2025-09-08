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

#include "driver_bme680_register_test.h"
#include "stdlib.h"

static bme680_handle_t gs_handle;        /**< bme680 handle */

/**
 * @brief     register test
 * @param[in] interface chip interface
 * @param[in] addr_pin chip address pin
 * @return    status code
 *            - 0 success
 *            - 1 test failed
 * @note      none
 */
uint8_t bme680_register_test(bme680_interface_t interface, bme680_address_t addr_pin)
{
    uint8_t res;
    uint8_t reg;
    uint8_t reg_check;
    uint8_t i;
    uint8_t index;
    uint8_t index_check;
    uint16_t ms;
    float degree_celsius;
    float ma;
    float ma_check;
    bme680_info_t info;
    bme680_address_t addr_pin_check;
    bme680_interface_t interface_check;
    bme680_oversampling_t oversampling;
    bme680_mode_t mode;
    bme680_filter_t filter;
    bme680_spi_wire_t spi;
    bme680_bool_t enable;
    
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
    
    /* start register test */
    bme680_interface_debug_print("bme680: start register test.\n");
    
    /* bme680_set_interface/bme680_get_interface test */
    bme680_interface_debug_print("bme680: bme680_set_interface/bme680_get_interface test.\n");
    
    /* set interface */
    res = bme680_set_interface(&gs_handle, BME680_INTERFACE_SPI);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set interface failed.\n");
       
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set interface spi.\n");
    
    /* get interface */
    res = bme680_get_interface(&gs_handle, &interface_check);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get interface failed.\n");
       
        return 1;
    }
    bme680_interface_debug_print("bme680: check interface %s.\n", interface_check == BME680_INTERFACE_SPI ? "ok" : "error");
    
    /* set interface */
    res = bme680_set_interface(&gs_handle, BME680_INTERFACE_IIC);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set interface failed.\n");
       
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set interface iic.\n");
    
    /* get interface */
    res = bme680_get_interface(&gs_handle, &interface_check);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get interface failed.\n");
       
        return 1;
    }
    bme680_interface_debug_print("bme680: check interface %s.\n", interface_check == BME680_INTERFACE_IIC ? "ok" : "error");
    
    /* bme680_set_addr_pin/bme680_get_addr_pin test */
    bme680_interface_debug_print("bme680: bme680_set_addr_pin/bme680_get_addr_pin test.\n");
    
    /* set addr pin low */
    res = bme680_set_addr_pin(&gs_handle, BME680_ADDRESS_ADO_LOW);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set addr pin failed.\n");
       
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set addr pin low.\n");
    
    /* get addr pin */
    res = bme680_get_addr_pin(&gs_handle, &addr_pin_check);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get addr pin failed.\n");
       
        return 1;
    }
    bme680_interface_debug_print("bme680: check addr pin %s.\n", addr_pin_check == BME680_ADDRESS_ADO_LOW ? "ok" : "error");
    
    /* set addr pin high */
    res = bme680_set_addr_pin(&gs_handle, BME680_ADDRESS_ADO_HIGH);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set addr pin failed.\n");
       
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set addr pin high.\n");
    
    /* get addr pin */
    res = bme680_get_addr_pin(&gs_handle, &addr_pin_check);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get addr pin failed.\n");
       
        return 1;
    }
    bme680_interface_debug_print("bme680: check addr pin %s.\n", addr_pin_check == BME680_ADDRESS_ADO_HIGH ? "ok" : "error");
    
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
    
    /* bme680_set_temperature_oversampling/bme680_get_temperature_oversampling test */
    bme680_interface_debug_print("bme680: bme680_set_temperature_oversampling/bme680_get_temperature_oversampling test.\n");
    
    /* set temperature oversampling skip */
    res = bme680_set_temperature_oversampling(&gs_handle, BME680_OVERSAMPLING_SKIP);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set temperature oversampling skip.\n");
    
    /* get temperature oversampling */
    res = bme680_get_temperature_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check temperature oversampling %s.\n", oversampling == BME680_OVERSAMPLING_SKIP ? "ok" : "error");
    
    /* set temperature oversampling x1 */
    res = bme680_set_temperature_oversampling(&gs_handle, BME680_OVERSAMPLING_x1);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set temperature oversampling x1.\n");
    
    /* get temperature oversampling */
    res = bme680_get_temperature_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check temperature oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x1 ? "ok" : "error");
    
    /* set temperature oversampling x2 */
    res = bme680_set_temperature_oversampling(&gs_handle, BME680_OVERSAMPLING_x2);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set temperature oversampling x2.\n");
    
    /* get temperature oversampling */
    res = bme680_get_temperature_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check temperature oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x2 ? "ok" : "error");
    
    /* set temperature oversampling x4 */
    res = bme680_set_temperature_oversampling(&gs_handle, BME680_OVERSAMPLING_x4);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set temperature oversampling x4.\n");
    
    /* get temperature oversampling */
    res = bme680_get_temperature_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check temperature oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x4 ? "ok" : "error");
    
    /* set temperature oversampling x8 */
    res = bme680_set_temperature_oversampling(&gs_handle, BME680_OVERSAMPLING_x8);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set temperature oversampling x8.\n");
    
    /* get temperature oversampling */
    res = bme680_get_temperature_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check temperature oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x8 ? "ok" : "error");
    
    /* set temperature oversampling x16 */
    res = bme680_set_temperature_oversampling(&gs_handle, BME680_OVERSAMPLING_x16);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set temperature oversampling x16.\n");
    
    /* get temperature oversampling */
    res = bme680_get_temperature_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get temperature oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check temperature oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x16 ? "ok" : "error");
    
    /* bme680_set_pressure_oversampling/bme680_get_pressure_oversampling test */
    bme680_interface_debug_print("bme680: bme680_set_pressure_oversampling/bme680_get_pressure_oversampling test.\n");
    
    /* set pressure oversampling skip */
    res = bme680_set_pressure_oversampling(&gs_handle, BME680_OVERSAMPLING_SKIP);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set pressure oversampling skip.\n");
    
    /* get pressure oversampling */
    res = bme680_get_pressure_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check pressure oversampling %s.\n", oversampling == BME680_OVERSAMPLING_SKIP ? "ok" : "error");
    
    /* set pressure oversampling x1 */
    res = bme680_set_pressure_oversampling(&gs_handle, BME680_OVERSAMPLING_x1);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set pressure oversampling x1.\n");
    
    /* get pressure oversampling */
    res = bme680_get_pressure_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check pressure oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x1 ? "ok" : "error");
    
    /* set pressure oversampling x2 */
    res = bme680_set_pressure_oversampling(&gs_handle, BME680_OVERSAMPLING_x2);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set pressure oversampling x2.\n");
    
    /* get pressure oversampling */
    res = bme680_get_pressure_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check pressure oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x2 ? "ok" : "error");
    
    /* set pressure oversampling x4 */
    res = bme680_set_pressure_oversampling(&gs_handle, BME680_OVERSAMPLING_x4);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set pressure oversampling x4.\n");
    
    /* get pressure oversampling */
    res = bme680_get_pressure_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check pressure oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x4 ? "ok" : "error");
    
    /* set pressure oversampling x8 */
    res = bme680_set_pressure_oversampling(&gs_handle, BME680_OVERSAMPLING_x8);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set pressure oversampling x8.\n");
    
    /* get pressure oversampling */
    res = bme680_get_pressure_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check pressure oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x8 ? "ok" : "error");
    
    /* set pressure oversampling x16 */
    res = bme680_set_pressure_oversampling(&gs_handle, BME680_OVERSAMPLING_x16);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set pressure oversampling x16.\n");
    
    /* get pressure oversampling */
    res = bme680_get_pressure_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get pressure oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check pressure oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x16 ? "ok" : "error");
    
    /* bme680_set_humidity_oversampling/bme680_get_humidity_oversampling test */
    bme680_interface_debug_print("bme680: bme680_set_humidity_oversampling/bme680_get_humidity_oversampling test.\n");
    
    /* set humidity oversampling skip */
    res = bme680_set_humidity_oversampling(&gs_handle, BME680_OVERSAMPLING_SKIP);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set humidity oversampling skip.\n");
    
    /* get humidity oversampling */
    res = bme680_get_humidity_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check humidity oversampling %s.\n", oversampling == BME680_OVERSAMPLING_SKIP ? "ok" : "error");
    
    /* set humidity oversampling x1 */
    res = bme680_set_humidity_oversampling(&gs_handle, BME680_OVERSAMPLING_x1);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set humidity oversampling x1.\n");
    
    /* get humidity oversampling */
    res = bme680_get_humidity_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check humidity oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x1 ? "ok" : "error");
    
    /* set humidity oversampling x2 */
    res = bme680_set_humidity_oversampling(&gs_handle, BME680_OVERSAMPLING_x2);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set humidity oversampling x2.\n");
    
    /* get humidity oversampling */
    res = bme680_get_humidity_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check humidity oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x2 ? "ok" : "error");
    
    /* set humidity oversampling x4 */
    res = bme680_set_humidity_oversampling(&gs_handle, BME680_OVERSAMPLING_x4);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set humidity oversampling x4.\n");
    
    /* get humidity oversampling */
    res = bme680_get_humidity_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check humidity oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x4 ? "ok" : "error");
    
    /* set humidity oversampling x8 */
    res = bme680_set_humidity_oversampling(&gs_handle, BME680_OVERSAMPLING_x8);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set humidity oversampling x8.\n");
    
    /* get humidity oversampling */
    res = bme680_get_humidity_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check humidity oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x8 ? "ok" : "error");
    
    /* set humidity oversampling x16 */
    res = bme680_set_humidity_oversampling(&gs_handle, BME680_OVERSAMPLING_x16);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set humidity oversampling x16.\n");
    
    /* get humidity oversampling */
    res = bme680_get_humidity_oversampling(&gs_handle, &oversampling);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get humidity oversampling failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check humidity oversampling %s.\n", oversampling == BME680_OVERSAMPLING_x16 ? "ok" : "error");
    
    /* bme680_set_mode/bme680_get_mode test */
    bme680_interface_debug_print("bme680: bme680_set_mode/bme680_get_mode test.\n");
    
    /* set mode forced */
    res = bme680_set_mode(&gs_handle, BME680_MODE_FORCED);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set mode failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set mode forced.\n");
    
    res = bme680_get_mode(&gs_handle, &mode);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get mode failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check mode %s.\n", mode == BME680_MODE_FORCED ? "ok" : "error");
    
    /* set mode sleep */
    res = bme680_set_mode(&gs_handle, BME680_MODE_SLEEP);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set mode failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set mode sleep.\n");
    
    res = bme680_get_mode(&gs_handle, &mode);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get mode failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check mode %s.\n", mode == BME680_MODE_SLEEP ? "ok" : "error");
    
    /* bme680_set_filter/bme680_get_filter test */
    bme680_interface_debug_print("bme680: bme680_set_filter/bme680_get_filter test.\n");
    
    /* set filter off */
    res = bme680_set_filter(&gs_handle, BME680_FILTER_OFF);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set filter off.\n");
    
    /* get filter */
    res = bme680_get_filter(&gs_handle, &filter);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check filter %s.\n", filter == BME680_FILTER_OFF ? "ok" : "error");
    
    /* set filter coeff 1 */
    res = bme680_set_filter(&gs_handle, BME680_FILTER_COEFF_1);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set filter coeff 1.\n");
    
    /* get filter */
    res = bme680_get_filter(&gs_handle, &filter);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check filter %s.\n", filter == BME680_FILTER_COEFF_1 ? "ok" : "error");
    
    /* set filter coeff 3 */
    res = bme680_set_filter(&gs_handle, BME680_FILTER_COEFF_3);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set filter coeff 3.\n");
    
    /* get filter */
    res = bme680_get_filter(&gs_handle, &filter);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check filter %s.\n", filter == BME680_FILTER_COEFF_3 ? "ok" : "error");
    
    /* set filter coeff 7 */
    res = bme680_set_filter(&gs_handle, BME680_FILTER_COEFF_7);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set filter coeff 7.\n");
    
    /* get filter */
    res = bme680_get_filter(&gs_handle, &filter);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check filter %s.\n", filter == BME680_FILTER_COEFF_7 ? "ok" : "error");
    
    /* set filter coeff 15 */
    res = bme680_set_filter(&gs_handle, BME680_FILTER_COEFF_15);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set filter coeff 15.\n");
    
    /* get filter */
    res = bme680_get_filter(&gs_handle, &filter);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check filter %s.\n", filter == BME680_FILTER_COEFF_15 ? "ok" : "error");
    
    /* set filter coeff 31 */
    res = bme680_set_filter(&gs_handle, BME680_FILTER_COEFF_31);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set filter coeff 31.\n");
    
    /* get filter */
    res = bme680_get_filter(&gs_handle, &filter);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check filter %s.\n", filter == BME680_FILTER_COEFF_31 ? "ok" : "error");
    
    /* set filter coeff 63 */
    res = bme680_set_filter(&gs_handle, BME680_FILTER_COEFF_63);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set filter coeff 63.\n");
    
    /* get filter */
    res = bme680_get_filter(&gs_handle, &filter);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check filter %s.\n", filter == BME680_FILTER_COEFF_63 ? "ok" : "error");
    
    /* set filter coeff 127 */
    res = bme680_set_filter(&gs_handle, BME680_FILTER_COEFF_127);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    
    /* output */
    bme680_interface_debug_print("bme680: set filter coeff 127.\n");
    
    /* get filter */
    res = bme680_get_filter(&gs_handle, &filter);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get filter failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check filter %s.\n", filter == BME680_FILTER_COEFF_127 ? "ok" : "error");
    
    /* if iic interface */
    if (interface == BME680_INTERFACE_IIC)
    {
        /* bme680_set_spi_wire/bme680_get_spi_wire test */
        bme680_interface_debug_print("bme680: bme680_set_spi_wire/bme680_get_spi_wire test.\n");
        
        /* set spi wire3 */
        res = bme680_set_spi_wire(&gs_handle, BME680_SPI_WIRE_3);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: set spi wire failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        
        /* output */
        bme680_interface_debug_print("bme680: set spi wire 3.\n");
        
        /* get spi wire */
        res = bme680_get_spi_wire(&gs_handle, &spi);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: get spi wire failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        bme680_interface_debug_print("bme680: check spi wire %s.\n", spi == BME680_SPI_WIRE_3 ? "ok" : "error");
        
        /* set spi wire4 */
        res = bme680_set_spi_wire(&gs_handle, BME680_SPI_WIRE_4);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: set spi wire failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        
        /* output */
        bme680_interface_debug_print("bme680: set spi wire 4.\n");
        
        /* get spi wire */
        res = bme680_get_spi_wire(&gs_handle, &spi);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: get spi wire failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        bme680_interface_debug_print("bme680: check spi wire %s.\n", spi == BME680_SPI_WIRE_4 ? "ok" : "error");
    }
    
    /* bme680_set_spi_wire_3_data_interrupt/bme680_get_spi_wire_3_data_interrupt test */
    bme680_interface_debug_print("bme680: bme680_set_spi_wire_3_data_interrupt/bme680_get_spi_wire_3_data_interrupt test.\n");
    
    /* enable spi wire 3 data interrupt */
    res = bme680_set_spi_wire_3_data_interrupt(&gs_handle, BME680_BOOL_TRUE);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set spi wire 3 data interrupt failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: enable spi wire 3 data interrupt.\n");
    res = bme680_get_spi_wire_3_data_interrupt(&gs_handle, &enable);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get spi wire 3 data interrupt failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check spi wire 3 data interrupt %s.\n", enable == BME680_BOOL_TRUE ? "ok" : "error");
    
    /* disable spi wire 3 data interrupt */
    res = bme680_set_spi_wire_3_data_interrupt(&gs_handle, BME680_BOOL_FALSE);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set spi wire 3 data interrupt failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: disable spi wire 3 data interrupt.\n");
    res = bme680_get_spi_wire_3_data_interrupt(&gs_handle, &enable);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get spi wire 3 data interrupt failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check spi wire 3 data interrupt %s.\n", enable == BME680_BOOL_FALSE ? "ok" : "error");
    
    /* bme680_get_new_data_status test */
    bme680_interface_debug_print("bme680: bme680_get_new_data_status test.\n");
    
    /* get new data status */
    res = bme680_get_new_data_status(&gs_handle, &enable);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get new data status failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: new data status is %s.\n", enable == BME680_BOOL_TRUE ? "true" : "false");
    
    /* bme680_get_gas_measuring_status test */
    bme680_interface_debug_print("bme680: bme680_get_gas_measuring_status test.\n");
    
    /* get gas measuring status */
    res = bme680_get_gas_measuring_status(&gs_handle, &enable);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get gas measuring status failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: gas measuring status is %s.\n", enable == BME680_BOOL_TRUE ? "true" : "false");
    
    /* bme680_get_measuring_status test */
    bme680_interface_debug_print("bme680: bme680_get_measuring_status test.\n");
    
    /* get measuring status */
    res = bme680_get_measuring_status(&gs_handle, &enable);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get measuring status failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: get measuring status is %s.\n", enable == BME680_BOOL_TRUE ? "true" : "false");
    
    /* bme680_get_gas_measuring_index test */
    bme680_interface_debug_print("bme680: bme680_get_gas_measuring_index test.\n");
    
    /* get gas measuring index */
    res = bme680_get_gas_measuring_index(&gs_handle, &index);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get gas measuring index failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: gas measuring index is 0x%02X.\n", index);
    
    /* bme680_get_gas_valid_status test */
    bme680_interface_debug_print("bme680: bme680_get_gas_valid_status test.\n");
    
    /* get gas valid status */
    res = bme680_get_gas_valid_status(&gs_handle, &enable);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get gas valid status failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: gas valid status is %s.\n", enable == BME680_BOOL_TRUE ? "true" : "false");
    
    /* bme680_get_heater_stability_status test */
    bme680_interface_debug_print("bme680: bme680_get_heater_stability_status test.\n");
    
    /* get heater stability status */
    res = bme680_get_heater_stability_status(&gs_handle, &enable);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get heater stability status failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: heater stability status is %s.\n", enable == BME680_BOOL_TRUE ? "true" : "false");
    
    /* bme680_set_heat_off/bme680_get_heat_off test */
    bme680_interface_debug_print("bme680: bme680_set_heat_off/bme680_get_heat_off test.\n");
    
    /* disable heat off */
    res = bme680_set_heat_off(&gs_handle, BME680_BOOL_FALSE);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set heat off failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: disable heat off.\n");
    res = bme680_get_heat_off(&gs_handle, &enable);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get heat off failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check heat off %s.\n", enable == BME680_BOOL_FALSE ? "ok" : "error");
    
    /* enable heat off */
    res = bme680_set_heat_off(&gs_handle, BME680_BOOL_TRUE);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set heat off failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: enable heat off.\n");
    res = bme680_get_heat_off(&gs_handle, &enable);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get heat off failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check heat off %s.\n", enable == BME680_BOOL_TRUE ? "ok" : "error");
    
    /* bme680_set_run_gas/bme680_get_run_gas test */
    bme680_interface_debug_print("bme680: bme680_set_run_gas/bme680_get_run_gas test.\n");
    
    /* enable run gas */
    res = bme680_set_run_gas(&gs_handle, BME680_BOOL_TRUE);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set run gas failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: enable run gas.\n");
    res = bme680_get_run_gas(&gs_handle, &enable);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get run gas failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check run gas %s.\n", enable == BME680_BOOL_TRUE ? "ok" : "error");
    
    /* disable run gas */
    res = bme680_set_run_gas(&gs_handle, BME680_BOOL_FALSE);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: set run gas failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: disable run gas.\n");
    res = bme680_get_run_gas(&gs_handle, &enable);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: get run gas failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check run gas %s.\n", enable == BME680_BOOL_FALSE ? "ok" : "error");
    
    /* bme680_set_convert_index/bme680_get_convert_index test */
    bme680_interface_debug_print("bme680: bme680_set_convert_index/bme680_get_convert_index test.\n");
    
    for (i = 0; i < 10; i++)
    {
        /* set convert index */
        res = bme680_set_convert_index(&gs_handle, i);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: set convert index failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        bme680_interface_debug_print("bme680: set convert index %d.\n", i);
        res = bme680_get_convert_index(&gs_handle, &index_check);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: get convert index failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        bme680_interface_debug_print("bme680: check convert index %s.\n", i == index_check ? "ok" : "error");
    }
    
    /* bme680_set_idac_heat/bme680_get_idac_heat test */
    bme680_interface_debug_print("bme680: bme680_set_idac_heat/bme680_get_idac_heat test.\n");
    
    for (i = 0; i < 10; i++)
    {
        reg = rand() % 0xFFU;
        res = bme680_set_idac_heat(&gs_handle, i, reg);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: set idac heat failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        bme680_interface_debug_print("bme680: set index %d idac heat 0x%02X.\n", i, reg);
        res = bme680_get_idac_heat(&gs_handle, i, &reg_check);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: get idac heat failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        bme680_interface_debug_print("bme680: check idac heat %s.\n", reg == reg_check ? "ok" : "error");
    }
    
    /* bme680_set_resistance_heat/bme680_get_resistance_heat test */
    bme680_interface_debug_print("bme680: bme680_set_resistance_heat/bme680_get_resistance_heat test.\n");
    
    for (i = 0; i < 10; i++)
    {
        reg = rand() % 0xFFU;
        res = bme680_set_resistance_heat(&gs_handle, i, reg);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: set resistance heat failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        bme680_interface_debug_print("bme680: set index %d resistance heat 0x%02X.\n", i, reg);
        res = bme680_get_resistance_heat(&gs_handle, i, &reg_check);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: get resistance heat failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        bme680_interface_debug_print("bme680: check resistance heat %s.\n", reg == reg_check ? "ok" : "error");
    }
    
    /* bme680_set_gas_wait/bme680_get_gas_wait test */
    bme680_interface_debug_print("bme680: bme680_set_gas_wait/bme680_get_gas_wait test.\n");
    
    for (i = 0; i < 10; i++)
    {
        reg = rand() % 0xFFU;
        res = bme680_set_gas_wait(&gs_handle, i, reg);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: set gas wait failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        bme680_interface_debug_print("bme680: set index %d gas wait 0x%02X.\n", i, reg);
        res = bme680_get_gas_wait(&gs_handle, i, &reg_check);
        if (res != 0)
        {
            bme680_interface_debug_print("bme680: get gas wait failed.\n");
            (void)bme680_deinit(&gs_handle); 
            
            return 1;
        }
        bme680_interface_debug_print("bme680: check gas wait %s.\n", reg == reg_check ? "ok" : "error");
    }
    
    /* bme680_resistance_heat_convert_to_register test */
    bme680_interface_debug_print("bme680: bme680_resistance_heat_convert_to_register test.\n");
    
    degree_celsius = (float)(rand() % 30000) / 100;
    res = bme680_resistance_heat_convert_to_register(&gs_handle, degree_celsius, &reg);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: resistance heat convert to register failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: %0.2fC convert to register 0x%02X.\n", degree_celsius, reg);
    
    /* bme680_gas_wait_convert_to_register test */
    bme680_interface_debug_print("bme680: bme680_gas_wait_convert_to_register test.\n");
    
    ms = rand() % 100 + 50;
    res = bme680_gas_wait_convert_to_register(&gs_handle, ms, &reg);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: gas wait convert to register failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: %dms convert to register 0x%02X.\n", ms, reg);
    
    /* bme680_idac_heat_convert_to_register/bme680_idac_heat_convert_to_data test */
    bme680_interface_debug_print("bme680: bme680_idac_heat_convert_to_register/bme680_idac_heat_convert_to_data test.\n");
    
    ma = (float)(rand() % 100) / 10.0f;
    bme680_interface_debug_print("bme680: idac heat ma before is %0.2fmA.\n", ma);
    res = bme680_idac_heat_convert_to_register(&gs_handle, ma, &reg);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: idac heat convert to register failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: idac heat reg is 0x%02X.\n", reg);
    res = bme680_idac_heat_convert_to_data(&gs_handle, reg, &ma_check);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: idac heat convert to data failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: idac heat ma after is %0.2fmA.\n", ma_check);
    
    /* bme680_soft_reset test */
    bme680_interface_debug_print("bme680: bme680_soft_reset test.\n");
    
    /* soft reset */
    res = bme680_soft_reset(&gs_handle);
    if (res != 0)
    {
        bme680_interface_debug_print("bme680: soft reset failed.\n");
        (void)bme680_deinit(&gs_handle); 
        
        return 1;
    }
    bme680_interface_debug_print("bme680: check soft reset %s.\n", res == 0 ? "ok" : "error");
    
    /* finish register test */
    bme680_interface_debug_print("bme680: finish register test.\n");
    (void)bme680_deinit(&gs_handle); 

    return 0;
}
