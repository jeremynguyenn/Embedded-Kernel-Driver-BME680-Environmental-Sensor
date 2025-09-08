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

#ifndef DRIVER_BME680_BASIC_H
#define DRIVER_BME680_BASIC_H

#include "driver_bme680_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup bme680_example_driver bme680 example driver function
 * @brief    bme680 example driver modules
 * @ingroup  bme680_driver
 * @{
 */

/**
 * @brief bme680 basic example default definition
 */
#define BME680_BASIC_DEFAULT_TEMPERATURE_OVERSAMPLING        BME680_OVERSAMPLING_x2            /**< temperature oversampling x2 */
#define BME680_BASIC_DEFAULT_PRESSURE_OVERSAMPLING           BME680_OVERSAMPLING_x16           /**< pressure oversampling x16 */
#define BME680_BASIC_DEFAULT_HUMIDITY_OVERSAMPLING           BME680_OVERSAMPLING_x1            /**< humidity oversampling x1 */
#define BME680_BASIC_DEFAULT_FILTER                          BME680_FILTER_COEFF_15            /**< filter coeff 15 */
#define BME680_BASIC_DEFAULT_SPI_WIRE                        BME680_SPI_WIRE_4                 /**< spi wire 4 */

/**
 * @brief     basic example init
 * @param[in] interface chip interface
 * @param[in] addr_pin chip address pin
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      none
 */
uint8_t bme680_basic_init(bme680_interface_t interface, bme680_address_t addr_pin);

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t bme680_basic_deinit(void);

/**
 * @brief      basic example read
 * @param[out] *temperature pointer to a converted temperature buffer
 * @param[out] *pressure pointer to a converted pressure buffer
 * @param[out] *humidity_percentage pointer to a converted humidity percentage buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t bme680_basic_read(float *temperature, float *pressure, float *humidity_percentage);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
