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
 * @file     cs_bms.c
 * @date     21. Aug. 2024
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
#include <string.h>
#include <math.h>
#include "cs_bms.h"

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint32_t bms_feature_support;
static bms_control_handler control_handler;
static void *cs_bms_user_data;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static uint8_t is_opcode_support(uint8_t opcode, uint8_t auth, uint32_t feature)
{
    return (opcode >= BMS_OPCODE_BR_EDR_LE_DEL_REQ_DEV && opcode <= BMS_OPCODE_LE_DEL_EXP_REQ_DEV) &&
           (((1 << (opcode - 1) * 2) & feature) || (auth && ((1 << (opcode * 2 - 1)) & feature)));
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
uint32_t cs_bms_write_ops_ctrl_adapt(const uint8_t *data, int len)
{
    if (len > 0 && control_handler) {
        uint8_t opcode = *data;
        uint8_t auth_len = len - 1;
        const uint8_t *auth = auth_len == 0 ? NULL : &data[1];
        if (is_opcode_support(opcode, auth != NULL, bms_feature_support)) {
            return control_handler(opcode, auth, auth_len, cs_bms_user_data);
        }
    }
    return BMS_ERR_OPCODE_NOT_SUPPORT;
}

void cs_bms_read_feature_adapt(uint8_t resp_data[3])
{
    memcpy(resp_data, &bms_feature_support, 3);
}

void cs_bms_init(uint32_t feature_bits, bms_control_handler callback, void *user_data)
{
    bms_feature_support = feature_bits;
    control_handler = callback;
    cs_bms_user_data = user_data;
}

/** @} */
