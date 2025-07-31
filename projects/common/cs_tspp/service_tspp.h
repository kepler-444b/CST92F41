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
 * @file     service_tspp.h
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */
#ifndef __SERVICE_TSPP_H__
#define __SERVICE_TSPP_H__

/*******************************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include "sdk_config.h"
#include "cs_ble.h"
#include "cs_fifo.h"

/*********************************************************************
 * MACROS
 */

#define TSPPS_CONN_MAX      CS_LE_MAX_MASTER_NUM
#define TSPPS_MTU_DEFAULT   20
#define TSPPS_BUF_SIZE      512
#define TSPPS_MAX_MTU_SIZE  CS_LE_HOST_ATT_MTU

/// TSPP Stream mode, data send in buffer maybe stick together
#define TSPPS_STREAM_MODE 1

/// type of tspps data size
typedef uint16_t tspps_size_t;

/*********************************************************************
 * TYPEDEFS
 */
enum {
    TSPPS_IDX_SVC,

    TSPPS_IDX_NOTIFY_CHAR,
    TSPPS_IDX_NOTIFY_VAL,
    TSPPS_IDX_NOTIFY_CFG,

    TSPPS_IDX_WRITE_CHAR,
    TSPPS_IDX_WRITE_VAL,

    TSPPS_IDX_AT_CHAR,
    TSPPS_IDX_AT_VAL,
    TSPPS_IDX_AT_CFG,

    TSPPS_IDX_MAX,

};

/// TSPPS error code
enum {
    TSPPS_ERR_NO_ERROR,
    TSPPS_ERR_DISABLED,
    TSPPS_ERR_EXCEED_MTU,
    TSPPS_ERR_FULL,
    TSPPS_ERR_CONN_INVALID,
};

enum {
    TSPPS_STATE_IDLE,
    TSPPS_STATE_BUSY,
};

typedef struct {
    uint16_t att_start_handle;
    uint16_t notify_enabled[TSPPS_CONN_MAX];
    uint16_t at_enabled[TSPPS_CONN_MAX];
    uint16_t state[TSPPS_CONN_MAX];
    uint16_t mtu[TSPPS_CONN_MAX];
    uint8_t  buf[TSPPS_CONN_MAX][TSPPS_BUF_SIZE];
    cs_fifo_t tspps_fifo[TSPPS_CONN_MAX];
} tspps_context_t;

/*******************************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief Init TSPP service
 *******************************************************************************
 */
void app_tspp_init(void);

/**
 *******************************************************************************
 * @brief tspps send data
 *
 * @param[in] data       Data to send
 * @param[in] len        Length of data
 * @param[in] conn_idx   Connection idx to send the data
 * @param[in] att_handle Att handle to send the data
 *
 * @return length of sent data
 *******************************************************************************
 */
uint8_t tspps_send(const uint8_t *data, tspps_size_t len, uint8_t conn_idx, uint16_t att_handle);

/**
 *******************************************************************************
 * @brief Get tspps fifo available
 *
 * @return The size of available buffer
 *******************************************************************************
 */
uint16_t tspps_avail(uint8_t conn_idx);

#endif /* __SERVICE_TSPP_H__ */

/** @} */
