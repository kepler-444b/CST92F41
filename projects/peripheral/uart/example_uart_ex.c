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
 * @file     example_uart_ex.c
 * @brief    example for extended uart
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup EXAMPLE_UART_EX UART EX
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using extended uart
 * @details
 * There is an example as follows: using extended uart to send data and process
 * the received data
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


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * TYPEDEFS
 */
/// Test tx pad for uart0
#define PAD_UART0_TXD               22
/// Test rx pad for uart0
#define PAD_UART0_RXD               21
/// Test extended uart
#define TEST_USART                  CS_USART0
/// Test uart baudrate
#define USART_BAUDRATE              115200


/*******************************************************************************
 * CONST & VARIABLES
 */
/// Test extended uart pin configuration
static const pin_config_t pin_config[] = {
    {PAD_UART0_TXD, {PINMUX_PAD5_UART0_TX_CFG}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    {PAD_UART0_RXD, {PINMUX_PAD6_UART0_RX_CFG}, PMU_PIN_MODE_PU, PMU_PIN_DRIVER_CURRENT_NORMAL},
};
/// Buffer that stores the data to be sent
static uint32_t send_data[4] = {0};


/*******************************************************************************
 * LOCAL FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief callback for extended uart
 *
 * @param[in] cs_usart    Pointer to extended uart
 * @param[in] event       Extended uart event
 *                        - DRV_EVENT_COMMON_READ_COMPLETED
 * @param[in] rx_buf      Pointer to receive buffer
 * @param[in] rx_cnt      The number of received data
 *
 *******************************************************************************
 */
static void test_usart_ex_cb(void *cs_usart, drv_event_t event, void *rx_buf, void *rx_cnt)
{
    /* Do what you want to do, but don't print anything here */
}


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of sending data
 *        1. send string "hello"
 *        2. when the data is received, the callback will be triggered, where user
 *           can do something with the received data but cannot print anything
 *
 *******************************************************************************
 */
void example_uart_ex(void)
{
    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));

    usart_ex_config_t usart_cfg = {
        .baudrate       = USART_BAUDRATE,
    };
    drv_usart_ex_init(TEST_USART, &usart_cfg);
    drv_usart_ex_register_isr_callback(TEST_USART, test_usart_ex_cb);

    char *str = "hello, uart0\r\n";
    uint8_t str_len = strlen(str);
    memcpy(send_data, str, str_len);
    drv_usart_ex_write(TEST_USART, (uint8_t *)send_data, str_len, 10);
}


/** @} */
