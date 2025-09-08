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

#ifndef DRIVER_BME680_READ_TEST_H
#define DRIVER_BME680_READ_TEST_H

#include "driver_bme680_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @addtogroup bme680_test_driver
 * @{
 */

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
uint8_t bme680_read_test(bme680_interface_t interface, bme680_address_t addr_pin, uint8_t index, uint32_t times);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
