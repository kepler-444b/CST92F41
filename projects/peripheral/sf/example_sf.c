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
 * @file     example_sf.c
 * @brief    example for working with flash
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup EXAMPLE_SF SF
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using sf driver
 * @details
 * There is an example to use sf driver as follows: erase, read, write
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
#define TEST_ADDR   0xFA000


/*******************************************************************************
 * TYPEDEFS
 */
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
 * CONST & VARIABLES
 */
/// Buffer that stored the data to be written
static uint8_t write_buf[100];
/// Buffer that stored the data to be read
static uint8_t read_buf[100];


/*******************************************************************************
 * LOCAL FUNCTIONS
 */


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of working with flash: erase, write, read
 *
 *******************************************************************************
 */
void example_sf(void)
{
    CS_LOG_DEBUG("%s: begin\r\n", __func__);
    for (uint8_t i = 0; i < 100; i++) {
        write_buf[i] = i;
    }

    /* inside flash */
    // Enable Flash
    drv_sf_enable(CS_SF, 0);
    // Erase 4k in TEST_ADDR
    drv_sf_erase(CS_SF, 0, TEST_ADDR, 4 * 1024);
    // Read 100 bytes in TEST_ADDR, it should be all 0xFF
    drv_sf_read(CS_SF, 0, TEST_ADDR, read_buf, 100);
    CS_LOG_DEBUG("Contents before write:\r\n");
    PRINT_ARRAY(read_buf, 100);
    // Write 100 bytes to TEST_ADDR
    drv_sf_write(CS_SF, 0, TEST_ADDR, write_buf, 100);
    CS_LOG_DEBUG("Contents to write:\r\n");
    PRINT_ARRAY(write_buf, 100);
    // Read 100 bytes in TEST_ADDR, it should be same as write_buf
    drv_sf_read(CS_SF, 0, TEST_ADDR, read_buf, 100);
    CS_LOG_DEBUG("Contents after write:\r\n");
    PRINT_ARRAY(read_buf, 100);

    /* external flash */
#if 0
    // Enable Flash
    drv_sf_enable(CS_SF, 1);
    // Erase 4k in 128k
    drv_sf_erase(CS_SF, 1, 128 * 1024, 4 * 1024);
    // Read 100 bytes in 128k, it should be all 0xFF
    drv_sf_read(CS_SF, 1, 128 * 1024, read_buf, 100);
    // Write 100 bytes to 128k
    drv_sf_write(CS_SF, 1, 128 * 1024, write_buf, 100);
    // Read 100 bytes in 128k, it should be same as write_buf
    drv_sf_read(CS_SF, 1, 128 * 1024, read_buf, 100);
#endif
}


/** @} */
