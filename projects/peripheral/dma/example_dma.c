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
 * @file     example_dma.c
 * @brief    example for using dma
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup EXAMPLE_DMA DMA
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using dma
 * @details
 * There are two examples to use dma as follows: transfer between memory
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
/// DMA test size
#define DMA_TEST_SIZE    256


/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * CONST & VARIABLES
 */
/// Source data buffer
static uint8_t src_buf[1024];

/// Destination data buffer
static uint8_t dst_buf[1024];

/// DMA initial configuration
static const dma_config_t dma_cfg_demo = {
    .channel_ctrl     = DMA_SET_CTRL(DMA_ADDR_CTRL_INC,  DMA_ADDR_CTRL_INC,
                                     DMA_TRANS_WIDTH_1B, DMA_TRANS_WIDTH_1B,
                                     DMA_BURST_SIZE_1T,  DMA_PRIORITY_LOW),
    .src_id           = DMA_ID_MEM,
    .dst_id           = DMA_ID_MEM,
    .chain_trans      = NULL,
    .chain_trans_num  = 0U,
};

/// DMA List Item
static dma_chain_trans_t chain_trans[3];


/*******************************************************************************
 * LOCAL FUNCTIONS
 */


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of transfering data using dma, move data from src_buf to dst_buf
 *
 *******************************************************************************
 */
void example_dma_ram2ram(void)
{
    uint8_t         chan_id;
    dma_config_t    dma_cfg;

    chan_id = drv_dma_channel_allocate();
    memcpy(&dma_cfg, &dma_cfg_demo, sizeof(dma_config_t));
    dma_cfg.chain_trans      = NULL;
    dma_cfg.chain_trans_num  = 0;
    dma_cfg.event_cb         = NULL;
    dma_cfg.cb_param         = "ram2ram";
    drv_dma_channel_config(chan_id, &dma_cfg);

    memset(dst_buf, 0x00, DMA_TEST_SIZE);
    for (uint32_t i = 0x00; i < DMA_TEST_SIZE; i++) {
        src_buf[i] = i % 256;
    }

    drv_dma_channel_enable(chan_id, (uint32_t)dst_buf, (uint32_t)src_buf, DMA_TEST_SIZE);

    DRV_DELAY_MS(10);

    // check
    for (uint32_t i = 0x00; i < DMA_TEST_SIZE; i++) {
        if (src_buf[i] != dst_buf[i]) {
            cs_printf("DMA ram2ram fail");
            CS_ASSERT(0);
        }
    }
    cs_printf("DMA ram2ram pass\r\n");

    drv_dma_channel_release(chan_id);
}

/**
 *******************************************************************************
 * @brief example of transfer data using dma list
 *        1. firstly, move 32B data from src_buf to dst_buf
 *        2. move 32B data from src_buf+32 to dst_buf+32 according to chain_trans[0]
 *        3. move 32B data from src_buf+64 to dst_buf+64 according to chain_trans[1]
 *        4. finally, the rest of data is moved from src_buf+96 to dst_buf+96 according to chain_trans[2]
 *
 *******************************************************************************
 */
void example_dma_ram2ram_chains(void)
{
    uint8_t              chan_id;
    dma_config_t         dma_cfg;

    chan_id = drv_dma_channel_allocate();
    memcpy(&dma_cfg, &dma_cfg_demo, sizeof(dma_config_t));
    dma_cfg.chain_trans      = chain_trans;
    dma_cfg.chain_trans_num  = 3;
    dma_cfg.event_cb         = NULL;
    dma_cfg.cb_param         = "ram2ram_chains";

    // 2. 2nd transfer 32 Byte
    chain_trans[0].src_addr   = (uint32_t) (src_buf + 32U);
    chain_trans[0].dst_addr   = (uint32_t) (dst_buf + 32U);
    chain_trans[0].size_byte = 32U;
    chain_trans[0].ll_ptr     = (void *)(&(chain_trans[1]));

    // 3. 3rd transfer 32 Byte
    chain_trans[1].src_addr   = (uint32_t)(src_buf + 32U + 32U);
    chain_trans[1].dst_addr   = (uint32_t)(dst_buf + 32U + 32U);
    chain_trans[1].size_byte = 32U;
    chain_trans[1].ll_ptr     = (void *)(&(chain_trans[2]));

    //4. finally transfer the last
    chain_trans[2].src_addr   = (uint32_t)(src_buf + 32U + 32U + 32U);
    chain_trans[2].dst_addr   = (uint32_t)(dst_buf + 32U + 32U + 32U);
    chain_trans[2].size_byte = DMA_TEST_SIZE - 3 * 32U;
    chain_trans[2].ll_ptr     = NULL;
    memset(dst_buf, 0x00, DMA_TEST_SIZE);
    for (uint32_t i = 0x00; i < DMA_TEST_SIZE; i++) {
        src_buf[i] = i % 256;
    }

    // 1. firstly transfer 32Byte and start GPDMA
    drv_dma_channel_config(chan_id, &dma_cfg);
    drv_dma_channel_enable(chan_id, (uint32_t)dst_buf, (uint32_t)src_buf, 32U);

    DRV_DELAY_MS(100);

    // 4. check
    for (uint32_t i = 0x00; i < DMA_TEST_SIZE; i++) {
        if (src_buf[i] != dst_buf[i]) {
            cs_printf("DMA ram2ram chains fail");
            CS_ASSERT(0);
        }
    }
    cs_printf("DMA ram2ram chains pass\r\n");

    drv_dma_channel_release(chan_id);
}


/** @} */