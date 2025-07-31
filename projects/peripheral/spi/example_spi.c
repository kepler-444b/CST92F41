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
 * @file     example_spi.c
 * @brief    example for using spi
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup EXAMPLE_SPI SPI
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using spi
 * @details
 * There are some examples to use uart as follows: wire4, wire3 transmission
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
#define PAD_SPI0_CS                     18
#define MUX_SPI0_CS                     PINMUX_PAD18_SPI0_CS_CFG
#define PAD_SPI0_CLK                    19
#define MUX_SPI0_CLK                    PINMUX_PAD19_SPI0_SCK_CFG
#define PAD_SPI0_DI                     24
#define MUX_SPI0_DI                     PINMUX_PAD24_SPI0_DI_CFG
#define PAD_SPI0_DO                     25
#define MUX_SPI0_DO                     PINMUX_PAD25_SPI0_DO_CFG

#define TEST_TRANS_SIZE                 100

#define PRINT_ARRAY(ARRAY,SIZE) \
{ \
    for (int i = 0; i < SIZE; i++) { \
        CS_LOG_DEBUG("0x%02x ", ARRAY[i]); \
        if (i % 32 == 31) { \
            CS_LOG_DEBUG("\r\n"); \
        } \
    } \
    CS_LOG_DEBUG("\r\n"); \
}


/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * CONST & VARIABLES
 */
/// Pin configuration
static const pin_config_t pin_cfg_cnt [] = {
    {PAD_SPI0_CS,  {MUX_SPI0_CS},  PMU_PIN_MODE_PU, PMU_PIN_DRIVER_CURRENT_NORMAL},
    {PAD_SPI0_CLK, {MUX_SPI0_CLK}, PMU_PIN_MODE_PU, PMU_PIN_DRIVER_CURRENT_NORMAL},
    {PAD_SPI0_DI,  {MUX_SPI0_DI},  PMU_PIN_MODE_PU, PMU_PIN_DRIVER_CURRENT_NORMAL},
    {PAD_SPI0_DO,  {MUX_SPI0_DO},  PMU_PIN_MODE_PU, PMU_PIN_DRIVER_CURRENT_NORMAL},
};
/// Buffer that stores the data to be sent
static uint8_t spi_tx_buf[TEST_TRANS_SIZE];
/// Buffer that stores the data to be received
static uint8_t spi_rx_buf[TEST_TRANS_SIZE];
/// Transfer finish flag
static volatile uint8_t int_transfer_is_done;
/// DMA Transfer finish flag
static volatile uint8_t dma_transfer_is_done;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief callback for spi interrupt
 *
 * @param[in] cs_spi    Pointer to spi
 * @param[in] event     SPI event
 *                      - DRV_EVENT_COMMON_TRANSFER_COMPLETED
 *                      - DRV_EVENT_COMMON_DMA2PERIPH_COMPLETED
 *                      - DRV_EVENT_COMMON_ABORT
 *                      - DRV_EVENT_COMMON_ERROR
 * @param[in] rx_buf    Pointer to rx_buf
 * @param[in] rx_cnt    Number of rx data
 *******************************************************************************
 */
static void spi_transfer_cb(void *cs_spi, drv_event_t event, void *rx_buf, void *rx_cnt)
{
    if (event == DRV_EVENT_COMMON_TRANSFER_COMPLETED) {
        int_transfer_is_done = 1;
    }
}

/**
 *******************************************************************************
 * @brief callback for spi DMA
 *
 * @param[in] cs_spi    Pointer to spi
 * @param[in] event     SPI event
 *                      - DRV_EVENT_COMMON_TRANSFER_COMPLETED
 *                      - DRV_EVENT_COMMON_DMA2PERIPH_COMPLETED
 *                      - DRV_EVENT_COMMON_ABORT
 *                      - DRV_EVENT_COMMON_ERROR
 * @param[in] rx_buf    Pointer to rx_buf
 * @param[in] rx_cnt    Number of rx data
 *******************************************************************************
 */
static void spi_transfer_dma_cb(void *cs_spi, drv_event_t event, void *rx_buf, void *rx_cnt)
{
    if ((event == DRV_EVENT_COMMON_DMA2PERIPH_COMPLETED)||(event == DRV_EVENT_COMMON_READ_COMPLETED)) {
        dma_transfer_is_done = 1;
    }
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of using spi wire4 to send and receive by int
 *
 *******************************************************************************
 */
void example_spi_wire4_trans_int(void)
{
    spi_config_t    spi_cfg;
    spi_cfg.freq        = 8*1000*1000;
    spi_cfg.role        = SPI_ROLE_MASTER;
    //spi_cfg.role        = SPI_ROLE_SLAVE;
    spi_cfg.mode        = SPI_MODE_0;
    spi_cfg.wire        = SPI_WIRE_4;
    spi_cfg.first_bit   = SPI_MSB_FIRST;
    spi_cfg.cs_valid    = SPI_CS_LOW;

    drv_pin_init(pin_cfg_cnt, sizeof(pin_cfg_cnt) / sizeof(pin_cfg_cnt[0]));
    drv_spi_init(CS_SPI0, &spi_cfg);
    drv_spi_register_isr_callback(CS_SPI0, spi_transfer_cb);

    for (uint16_t i = 0; i < TEST_TRANS_SIZE; i++) {
        spi_tx_buf[i] = i + 1;
    }
    memset(spi_rx_buf, 0xFF, sizeof(spi_rx_buf));

    drv_spi_transfer_int(CS_SPI0, spi_tx_buf, TEST_TRANS_SIZE, spi_rx_buf, TEST_TRANS_SIZE);
    while (!int_transfer_is_done);
    int_transfer_is_done = 0;

    if (spi_cfg.role == SPI_ROLE_MASTER) {
        CS_LOG_DEBUG("Spi master transfer:\r\n");
        PRINT_ARRAY(spi_tx_buf, TEST_TRANS_SIZE);
    } else {
        CS_LOG_DEBUG("Spi slave receive:\r\n");
        PRINT_ARRAY(spi_rx_buf, TEST_TRANS_SIZE);
    }
}

/**
 *******************************************************************************
 * @brief example of using spi wire4 to send and receive by DMA
 *
 *******************************************************************************
 */
void example_spi_wire4_trans_dma(void)
{
    spi_config_t    spi_cfg;
    spi_cfg.freq        = 8*1000*1000;
    spi_cfg.role        = SPI_ROLE_MASTER;
    //spi_cfg.role        = SPI_ROLE_SLAVE;
    spi_cfg.mode        = SPI_MODE_0;
    spi_cfg.wire        = SPI_WIRE_4;
    spi_cfg.first_bit   = SPI_MSB_FIRST;
    spi_cfg.cs_valid    = SPI_CS_LOW;

	drv_spi_dma_channel_allocate(CS_SPI0, SPI_DMA_CHAN_ALL);

    drv_pin_init(pin_cfg_cnt, sizeof(pin_cfg_cnt) / sizeof(pin_cfg_cnt[0]));
    drv_spi_init(CS_SPI0, &spi_cfg);
    drv_spi_register_isr_callback(CS_SPI0, spi_transfer_dma_cb);

    for (uint16_t i = 0; i < TEST_TRANS_SIZE; i++) {
        spi_tx_buf[i] = i + 1;
    }
    memset(spi_rx_buf, 0xFF, sizeof(spi_rx_buf));

    drv_spi_transfer_dma(CS_SPI0, spi_tx_buf, TEST_TRANS_SIZE, spi_rx_buf, TEST_TRANS_SIZE);
    while (!dma_transfer_is_done);
    dma_transfer_is_done = 0;

    drv_spi_dma_channel_release(CS_SPI0, SPI_DMA_CHAN_ALL);
}

/**
 *******************************************************************************
 * @brief example of using spi wire3 to send and receive by int
 * MUX_SPI0_CS, MUX_SPI0_CLK, MUX_SPI0_DO are used
 *
 *******************************************************************************
 */
void example_spi_wire3_trans_int(void)
{
    spi_config_t    spi_cfg;
    spi_cfg.freq        = 8*1000*1000;
    spi_cfg.role        = SPI_ROLE_MASTER;
	//spi_cfg.role        = SPI_ROLE_SLAVE;
    spi_cfg.mode        = SPI_MODE_0;
    spi_cfg.wire        = SPI_WIRE_3;
    spi_cfg.first_bit   = SPI_MSB_FIRST;
    spi_cfg.cs_valid    = SPI_CS_LOW;

    drv_pin_init(pin_cfg_cnt, sizeof(pin_cfg_cnt) / sizeof(pin_cfg_cnt[0]));
    drv_spi_init(CS_SPI0, &spi_cfg);
    drv_spi_register_isr_callback(CS_SPI0, spi_transfer_cb);

    for (uint16_t i = 0; i < TEST_TRANS_SIZE; i++) {
        spi_tx_buf[i] = i + 1;
    }
    memset(spi_rx_buf, 0xFF, sizeof(spi_rx_buf));

	if(spi_cfg.role == SPI_ROLE_MASTER){
        drv_spi_transfer_int(CS_SPI0, spi_tx_buf, TEST_TRANS_SIZE, spi_rx_buf, 0U);
	}
	else if(spi_cfg.role == SPI_ROLE_SLAVE){
		drv_spi_transfer_int(CS_SPI0, spi_tx_buf, 0U, spi_rx_buf, TEST_TRANS_SIZE);
	}
    while (!int_transfer_is_done);
    int_transfer_is_done = 0;

	if(spi_cfg.role == SPI_ROLE_MASTER){
        drv_dwt_delay_ms(5);   // leave the time for slave switch to transmit only
	}

	if(spi_cfg.role == SPI_ROLE_MASTER){
        drv_spi_transfer_int(CS_SPI0, spi_tx_buf, 0U, spi_rx_buf, TEST_TRANS_SIZE);
	}
	else if(spi_cfg.role == SPI_ROLE_SLAVE){
		drv_spi_transfer_int(CS_SPI0, spi_tx_buf, TEST_TRANS_SIZE, spi_rx_buf, 0U);
	}

    while (!int_transfer_is_done);
    int_transfer_is_done = 0;
}

/**
 *******************************************************************************
 * @brief example of using spi wire3 to send and receive by int
 * MUX_SPI0_CS, MUX_SPI0_CLK, MUX_SPI0_DO are used
 *
 *******************************************************************************
 */
void example_spi_wire3_trans_dma(void)
{
    spi_config_t    spi_cfg;
    spi_cfg.freq        = 8*1000*1000;
    spi_cfg.role        = SPI_ROLE_MASTER;
    //spi_cfg.role        = SPI_ROLE_SLAVE;
    spi_cfg.mode        = SPI_MODE_0;
    spi_cfg.wire        = SPI_WIRE_3;
    spi_cfg.first_bit   = SPI_MSB_FIRST;
    spi_cfg.cs_valid    = SPI_CS_LOW;

	drv_spi_dma_channel_allocate(CS_SPI0, SPI_DMA_CHAN_ALL);

    drv_pin_init(pin_cfg_cnt, sizeof(pin_cfg_cnt) / sizeof(pin_cfg_cnt[0]));
    drv_spi_init(CS_SPI0, &spi_cfg);
    drv_spi_register_isr_callback(CS_SPI0, spi_transfer_dma_cb);

    for (uint16_t i = 0; i < TEST_TRANS_SIZE; i++) {
        spi_tx_buf[i] = i + 1;
    }
    memset(spi_rx_buf, 0xFF, sizeof(spi_rx_buf));

	if(spi_cfg.role == SPI_ROLE_MASTER){
        drv_spi_transfer_dma(CS_SPI0, spi_tx_buf, TEST_TRANS_SIZE, spi_rx_buf, 0U);
	}
	else if(spi_cfg.role == SPI_ROLE_SLAVE){
		drv_spi_transfer_dma(CS_SPI0, spi_tx_buf, 0U, spi_rx_buf, TEST_TRANS_SIZE);
	}
    while (!dma_transfer_is_done);
    dma_transfer_is_done = 0;

	if(spi_cfg.role == SPI_ROLE_MASTER){
		drv_dwt_delay_ms(5);   // leave the time for slave switch to transmit only
	}

	if(spi_cfg.role == SPI_ROLE_MASTER){
		drv_spi_transfer_dma(CS_SPI0, spi_tx_buf, 0U, spi_rx_buf, TEST_TRANS_SIZE);
	}
	else if(spi_cfg.role == SPI_ROLE_SLAVE){
		drv_spi_transfer_dma(CS_SPI0, spi_tx_buf, TEST_TRANS_SIZE, spi_rx_buf, 0U);
	}

    while (!dma_transfer_is_done);
    dma_transfer_is_done = 0;

    drv_spi_dma_channel_release(CS_SPI0, SPI_DMA_CHAN_ALL);
}

/** @} */
