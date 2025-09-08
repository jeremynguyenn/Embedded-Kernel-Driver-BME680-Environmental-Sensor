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
 * <tr><td>2022/11/11  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#ifndef IIC_H
#define IIC_H

#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup iic iic function
 * @brief    iic function modules
 * @{
 */

/**
 * @brief      iic bus init
 * @param[in]  *name pointer to an iic device name buffer
 * @param[out] *fd pointer to an iic device handle buffer
 * @return     status code
 *             - 0 success
 *             - 1 init failed
 * @note       none
 */
uint8_t iic_init(char *name, int *fd);

/**
 * @brief     iic bus deinit
 * @param[in] fd iic handle
 * @return    status code
 *            - 0 success
 *            - 1 deinit failed
 * @note      none
 */
uint8_t iic_deinit(int fd);

/**
 * @brief      iic bus read command
 * @param[in]  fd iic handle
 * @param[in]  addr iic device write address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       addr = device_address_7bits << 1
 */
uint8_t iic_read_cmd(int fd, uint8_t addr, uint8_t *buf, uint16_t len);

/**
 * @brief      iic bus read
 * @param[in]  fd iic handle
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       addr = device_address_7bits << 1
 */
uint8_t iic_read(int fd, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief      iic bus read with 16 bits register address
 * @param[in]  fd iic handle
 * @param[in]  addr iic device write address
 * @param[in]  reg iic register address
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       addr = device_address_7bits << 1
 */
uint8_t iic_read_address16(int fd, uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief     iic bus write command
 * @param[in] fd iic handle
 * @param[in] addr iic device write address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      addr = device_address_7bits << 1
 */
uint8_t iic_write_cmd(int fd, uint8_t addr, uint8_t *buf, uint16_t len);

/**
 * @brief     iic bus write
 * @param[in] fd iic handle
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      addr = device_address_7bits << 1
 */
uint8_t iic_write(int fd, uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);

/**
 * @brief     iic bus write with 16 bits register address
 * @param[in] fd iic handle
 * @param[in] addr iic device write address
 * @param[in] reg iic register address
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      addr = device_address_7bits << 1
 */
uint8_t iic_write_address16(int fd, uint8_t addr, uint16_t reg, uint8_t *buf, uint16_t len);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif 
