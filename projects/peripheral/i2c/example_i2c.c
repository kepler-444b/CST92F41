/* ----------------------------------------------------------------------------
 * Copyright (c) 2020-2030 chipsea Limited. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of chipseaelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * -------------------------------------------------------------------------- */

/**
 * @file     example_i2c.c
 * @brief    Example for using i2c
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup EXAMPLE_I2C I2C
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using i2c
 * @details
 * There is an example as follows: read and write eeprom
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */


/*******************************************************************************
 * INCLUDES
 */
#include "cs_driver.h"
#include "cs_common_utils.h"

/*******************************************************************************
 * MACROS
 */
/// Test pad i2c scl
#define PAD_I2C0_SCL                    18
/// Test pad mux i2c scl
#define MUX_I2C0_SCL                    PINMUX_PAD18_I2C0_SCK_CFG
/// Test pad i2c sda
#define PAD_I2C0_SDA                    19
/// Test pad mux i2c sda
#define MUX_I2C0_SDA                    PINMUX_PAD19_I2C0_SDA_CFG

#define CJMCU_1745 1

#if AT24C02

/* Use AT24C02 */
/// Address of eeprom
#define EEPROM_ADDR                     0x50U
/// Capacity of eeprom
#define EEPROM_CAPACITY                 256U
/// Test address in eeprom
#define TEST_ADDR                       0x40

#elif CJMCU_1745

/// Address of eeprom
#define EEPROM_ADDR                     0x38U
#define EEPROM_CAPACITY                 5U
#define TEST_ADDR                       0x62

#endif


/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * CONST & VARIABLES
 */
/// Buffer that stores data to be sent
static uint8_t eeprom_tx_buf[EEPROM_CAPACITY];
/// Buffer that stores data to be read
static uint8_t eeprom_rx_buf[EEPROM_CAPACITY];
/// I2C pin configuration
static const pin_config_t pin_cfg [] = {
        {PAD_I2C0_SCL, {MUX_I2C0_SCL}, PMU_PIN_MODE_OD, PMU_PIN_DRIVER_CURRENT_NORMAL},
        {PAD_I2C0_SDA, {MUX_I2C0_SDA}, PMU_PIN_MODE_OD, PMU_PIN_DRIVER_CURRENT_NORMAL},
};


/*******************************************************************************
 * LOCAL FUNCTIONS
 */


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of using i2c: read ,write eeprom
 *
 *******************************************************************************
 */
void example_i2c(void)
{
    drv_pin_init(pin_cfg, sizeof(pin_cfg) / sizeof(pin_cfg[0]));

    i2c_config_t cfg = {
        .mode  = I2C_MODE_MASTER,        // 7-bit addressing mode
        .speed = I2C_SPEED_100K,
    };
    drv_i2c_init(CS_I2C0, &cfg);

#if AT24C02
    for (uint8_t i = 1; i < 8; i++) {
        eeprom_tx_buf[i] = 0x80 + i;
    }
    eeprom_tx_buf[0] = TEST_ADDR;

    // write tx_buf[1:4] to address TEST_ADDR
    drv_i2c_master_write(CS_I2C0, EEPROM_ADDR, eeprom_tx_buf, 5, 0);
    CS_LOG_DEBUG("%s: eeprom_tx_buf %s\r\n", __func__, bt_hex(eeprom_tx_buf, 5));

    drv_dwt_delay_ms(20);  // eeprom internal write takes time

    // read data from address TEST_ADDR
    uint8_t rd_addr = TEST_ADDR;
    drv_i2c_master_read(CS_I2C0, EEPROM_ADDR, &rd_addr, 1, eeprom_rx_buf, 4, 0);
    CS_LOG_DEBUG("%s: eeprom_rx_buf %s\r\n", __func__, bt_hex(eeprom_rx_buf, 4));

#elif CJMCU_1745

    // write 0x81,0x82,0x83,0x84 to 62h,63h,64h,65h
    eeprom_tx_buf[0] = TEST_ADDR;
    eeprom_tx_buf[1] = 0x81;
    eeprom_tx_buf[2] = 0x82;
    eeprom_tx_buf[3] = 0x83;
    eeprom_tx_buf[4] = 0x84;
    drv_i2c_master_write(CS_I2C0, EEPROM_ADDR, eeprom_tx_buf, 5, 0);
    CS_LOG_DEBUG("%s: eeprom_tx_buf %s\r\n", __func__, bt_hex(eeprom_tx_buf + 1, 4));

    // read data from address TEST_ADDR
    uint8_t rd_addr = TEST_ADDR;
    drv_i2c_master_read(CS_I2C0, EEPROM_ADDR, &rd_addr, 1, eeprom_rx_buf, 4, 0);
    CS_LOG_DEBUG("%s: eeprom_rx_buf %s\r\n", __func__, bt_hex(eeprom_rx_buf, 4));
#endif

}

/** @} */
