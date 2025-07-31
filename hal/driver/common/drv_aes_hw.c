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
 * DISCLAIMED. IN NO EVENT AESLL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * -------------------------------------------------------------------------- */

/**
 * @file     aes256.c
 * @brief    aes256 driver
 * @date     15 December 2021
 * @author   chipsea
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
#include "RTE_cst92f4x.h"
#if (RTE_AES_HW)
#include <string.h>
#include "cs_device.h"
#include "cs_driver.h"


/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * CONST & VARIABLES
 */


/*******************************************************************************
 * LOCAL FUNCTIONS
 */

/**
 *******************************************************************************
 * @brief  aes encrypt
 *
 * @param[in] is_256  is 256
 * @param[in] key  key 32
 * @param[in] input  input 16
 * @param[in] output  output 16
 *******************************************************************************
 */
static void aes_hw_encrypt(bool is_256, const uint8_t key[32], const uint8_t input[16], uint8_t output[16])
{
    uint32_t ctrl_reg;
    uint32_t key_len;

    if (is_256) {
        ctrl_reg = AES_HW_CTRL_START_MASK | AES_HW_CTRL_MODE_MASK;
        key_len = 32;
    } else {
        ctrl_reg = AES_HW_CTRL_START_MASK;
        key_len = 16;
    }

    CS_CRITICAL_BEGIN();

    // check AES clock
    if (CS_CPM->BLE_CFG & CPM_BLE_CFG_BLE_AHB_GATE_EN_MASK)
        CS_CPM->AES_CFG = CCS_AES_CLK_SEL_MASK;

    // Digital support bytes access
    memcpy((void *)&CS_AES->KEY[is_256?0:4], key, key_len);
    memcpy((void *)CS_AES->DATA, input, 16);

    CS_AES->CTRL = ctrl_reg;
    while(!(CS_AES->CTRL & AES_HW_CTRL_DONE_MASK));
    CS_AES->CTRL |= AES_HW_CTRL_DONE_MASK;

    memcpy(output, (void *)CS_AES->DATA, 16);

    // check AES clock
    if (CS_CPM->BLE_CFG & CPM_BLE_CFG_BLE_AHB_GATE_EN_MASK)
        CS_CPM->AES_CFG = CPM_AES_CLK_EN_MASK;

    CS_CRITICAL_END();
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 *******************************************************************************
 * @brief  aes128 encrypt
 *
 * @param[in] key  key 16
 * @param[in] input  input 16
 * @param[in] output  output 16
 *******************************************************************************
 */
void drv_aes128_hw_encrypt(const uint8_t key[16], const uint8_t input[16], uint8_t output[16])
{
    aes_hw_encrypt(false/*is_256*/, key, input, output);
}

/**
 *******************************************************************************
 * @brief  aes256 encrypt
 *
 * @param[in] key  key 32
 * @param[in] input  input 16
 * @param[in] output  output 16
 *******************************************************************************
 */
void drv_aes256_hw_encrypt(const uint8_t key[32], const uint8_t input[16], uint8_t output[16])
{
    aes_hw_encrypt(true/*is_256*/, key, input, output);
}

#endif

/** @} */

