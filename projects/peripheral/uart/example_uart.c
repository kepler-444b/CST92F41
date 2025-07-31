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
 * @file     example_uart.c
 * @brief    example for using uart
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup EXAMPLE_UART UART
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using uart
 * @details
 * There are three examples to use uart as follows: blocking, interrupt, and DMA
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
/// Test tx pad for uart1
#define PAD_UART1_TXD               18
/// Test rx pad for uart1
#define PAD_UART1_RXD               17
/// Test cts pad for uart1
#define PAD_UART1_CTS               20
/// Test rts pad for uart1
#define PAD_UART1_RTS               19
/// Test uart
#define TEST_USART                  CS_USART1
/// Test uart baudrate
#define USART_BAUDRATE              115200
/// Wait flag defination
#define SEND_INT_IS_DONE            (1 << 0)
#define SEND_DMA_IS_DONE            (1 << 1)
#define READ_INT_IS_DONE            (1 << 2)
#define READ_DMA_IS_DONE            (1 << 3)


/*******************************************************************************
 * TYPEDEFS
 */
typedef struct {
    volatile uint8_t st;
} usart_dma_int_wait_flag_t;


/*******************************************************************************
 * CONST & VARIABLES
 */
/// Test uart pin configuration
static const pin_config_t pin_config[] = {
    {PAD_UART1_TXD, {PINMUX_PAD18_UART1_TX_CFG}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    {PAD_UART1_RXD, {PINMUX_PAD17_UART1_RX_CFG}, PMU_PIN_MODE_PU, PMU_PIN_DRIVER_CURRENT_NORMAL},
    {PAD_UART1_RTS, {PINMUX_PAD20_UART1_RTS_CFG}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    {PAD_UART1_CTS, {PINMUX_PAD19_UART1_CTS_CFG}, PMU_PIN_MODE_PU, PMU_PIN_DRIVER_CURRENT_NORMAL},
};
/// Buffer to store the received data
static uint32_t rec_data[4];
/// Buffer that stores the data to be sent
static uint32_t send_data[4];
/// DMA,INT wait flag
static usart_dma_int_wait_flag_t wait_flag;


/*******************************************************************************
 * LOCAL FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief callback for uart interrupt
 *
 * @param[in] cs_usart    Pointer to uart
 * @param[in] event       uart event
 *                        - DRV_EVENT_COMMON_WRITE_COMPLETED
 *                        - DRV_EVENT_COMMON_DMA2PERIPH_COMPLETED
 *                        - DRV_EVENT_COMMON_READ_COMPLETED
 *                        - DRV_EVENT_USART_RX_TIMEOUT
 *                        - DRV_EVENT_COMMON_RX_OVERFLOW
 *                        - DRV_EVENT_USART_RX_PARITY_ERROR
 *                        - DRV_EVENT_USART_RX_BREAK
 *                        - DRV_EVENT_USART_RX_FRAME_ERROR
 * @param[in] rxbuf      Pointer to receive buffer
 * @param[in] rx_cnt     The number of received data
 *
 *******************************************************************************
 */
static void test_usart_cb(void *cs_usart, drv_event_t event, void *rxbuf, void *rx_cnt)
{
    // Send what you receive
    if (event == DRV_EVENT_COMMON_READ_COMPLETED) {
        drv_usart_write(TEST_USART, (uint8_t *)rxbuf, (uint32_t)rx_cnt, 10);
        wait_flag.st |= READ_INT_IS_DONE;
        wait_flag.st |= READ_DMA_IS_DONE;
    } else if (event == DRV_EVENT_COMMON_WRITE_COMPLETED) {
        wait_flag.st |= SEND_INT_IS_DONE;
    } else if (event == DRV_EVENT_COMMON_DMA2PERIPH_COMPLETED) {
        wait_flag.st |= SEND_DMA_IS_DONE;
    }
}


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of sending and receiving in blocking mode
 *        1. send string "hello"
 *        2. block until 5 characters are received from user
 *        3. send the received data
 *******************************************************************************
 */
void example_uart_block(void)
{
    cs_error_t error;

    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));

    usart_config_t usart_cfg = {
        .baudrate       = USART_BAUDRATE,
        .flow_control   = USART_FLOW_CONTROL_NONE,
        .data_bit       = USART_DATA_BIT_8,
        .stop_bit       = USART_STOP_BIT_1,
        .parity         = USART_PARITY_NONE,
    };

    drv_usart_init(TEST_USART, &usart_cfg);

    memcpy(send_data, "hello\r\n", 7);
    drv_usart_write(TEST_USART, (uint8_t *)send_data, 7, 10);
    error = drv_usart_read(TEST_USART, (uint8_t *)rec_data, 5, 3000);
    if (error == CS_ERROR_TIMEOUT) {
        drv_usart_write(TEST_USART, (uint8_t *)"Read Timeout\n", 13, 10);
    }
    drv_usart_write(TEST_USART, (uint8_t *)rec_data, 5, 10);
}

/**
 *******************************************************************************
 * @brief example of sending and receiving in interrupt mode
 *        1. send string "hello"
 *        2. send what chip receives
 *
 *******************************************************************************
 */
void example_uart_int(void)
{
    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));

    usart_config_t usart_cfg = {
        .baudrate       = USART_BAUDRATE,
        .flow_control   = USART_FLOW_CONTROL_NONE,
        .data_bit       = USART_DATA_BIT_8,
        .stop_bit       = USART_STOP_BIT_1,
        .parity         = USART_PARITY_NONE,
    };

    drv_usart_init(TEST_USART, &usart_cfg);
    drv_usart_register_isr_callback(TEST_USART, test_usart_cb);

    char *str = "hello, uart1\r\n";
    uint8_t str_len = strlen(str);
    memcpy(send_data, str, str_len);
    drv_usart_write_int(TEST_USART, (uint8_t *)send_data, str_len);
    while (!(wait_flag.st & SEND_INT_IS_DONE));
    wait_flag.st &= ~SEND_INT_IS_DONE;

    drv_usart_read_int(TEST_USART, NULL, 0);
}

/**
 *******************************************************************************
 * @brief example of sending and receiving in dma mode
 *        1. send string "hello"
 *        2. when the user passes 5 characters, the chip will send those 5
 *           characters back
 *******************************************************************************
 */
void example_uart_dma(void)
{
    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));

    usart_config_t usart_cfg = {
        .baudrate       = USART_BAUDRATE,
        .flow_control   = USART_FLOW_CONTROL_NONE,
        .data_bit       = USART_DATA_BIT_8,
        .stop_bit       = USART_STOP_BIT_1,
        .parity         = USART_PARITY_NONE,
    };

    drv_usart_init(TEST_USART, &usart_cfg);
    drv_usart_register_isr_callback(TEST_USART, test_usart_cb);

    drv_usart_dma_channel_allocate(TEST_USART, USART_DMA_CHAN_ALL);

    memcpy(send_data, "hello\r\n", 7);
    drv_usart_write_dma(TEST_USART, (uint8_t *)send_data, 7);
    while (!(wait_flag.st & SEND_DMA_IS_DONE));
    wait_flag.st &= ~SEND_DMA_IS_DONE;

    drv_usart_read_dma(TEST_USART, (uint8_t *)rec_data, 5);
    while (!(wait_flag.st & READ_DMA_IS_DONE));
    wait_flag.st &= ~READ_DMA_IS_DONE;

    drv_usart_dma_channel_release(TEST_USART, USART_DMA_CHAN_ALL);
}

void example_uart_flow_control(void)
{
    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));

    usart_config_t usart_cfg = {
        .baudrate       = USART_BAUDRATE,
        .flow_control   = USART_FLOW_CONTROL_RTS_CTS,
        .data_bit       = USART_DATA_BIT_8,
        .stop_bit       = USART_STOP_BIT_1,
        .parity         = USART_PARITY_NONE,
    };

    drv_usart_init(TEST_USART, &usart_cfg);
    drv_usart_register_isr_callback(TEST_USART, test_usart_cb);

    memcpy(send_data, "hello\r\n", 7);
    drv_usart_write_int(TEST_USART, (uint8_t *)send_data, 7);
    while (!(wait_flag.st & SEND_INT_IS_DONE));
    wait_flag.st &= ~SEND_INT_IS_DONE;

    drv_usart_read_int(TEST_USART, NULL, 0);
}

/** @} */
