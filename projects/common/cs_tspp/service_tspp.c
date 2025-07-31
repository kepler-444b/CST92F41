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
 * @file     service_tspp.c
 * @date     3 Feb 2023
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
#include <stdio.h>
#include <string.h>
#include "sdk_config.h"
#include "cs_ble.h"
#include "cs_common_utils.h"
#include "cs_log.h"
#include "uart_process.h"
#include "service_tspp.h"
#include "user_data.h"

//extern void app_tspp_data_sent_ind_handler(void);

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t service_tspps[CS_UUID_128BIT] = {0xF0, 0xFF};
static uint8_t tspps_notify[CS_UUID_128BIT] = {0xF1, 0xFF};
static uint8_t tspps_write[CS_UUID_128BIT] = {0xF2, 0xFF};
static uint8_t tspps_at[CS_UUID_128BIT] = {0xF3, 0xFF};
static cs_gatt_item_t atts_tspps[] = {
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { tspps_notify,    CS_UUID_16BIT, CS_ATT_PROP_NTF, 0},
    { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE},
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { tspps_write,     CS_UUID_16BIT, CS_ATT_PROP_WRITE},
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { tspps_at,        CS_UUID_16BIT, CS_ATT_PROP_WRITE_CMD | CS_ATT_PROP_NTF, 0},
    { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE},
};

static cs_gatt_item_t atts_tspps_128[] = {
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { tspps_notify,    CS_UUID_128BIT, CS_ATT_PROP_NTF, 0},
    { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE},
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { tspps_write,     CS_UUID_128BIT, CS_ATT_PROP_WRITE},
    { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
    { tspps_at,        CS_UUID_128BIT, CS_ATT_PROP_WRITE_CMD | CS_ATT_PROP_NTF, 0},
    { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE},
};

static tspps_context_t tspps_ctx = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void service_tspp_init(void)
{
    at_ble_service_t ble_service;

    memset(&tspps_ctx, 0, sizeof(tspps_ctx));
    for (int i = 0; i < TSPPS_CONN_MAX; i++) {
        tspps_ctx.mtu[i] = TSPPS_MTU_DEFAULT;
        tspps_ctx.state[i] = TSPPS_STATE_IDLE;
        cs_fifo_init(&tspps_ctx.tspps_fifo[i], tspps_ctx.buf[i], TSPPS_BUF_SIZE);
    }

#if AT_ENABLE
    user_data_get_ble_service(&ble_service);
    if (ble_service.uuid_type == UUID_16) {
        sys_memcpy_swap(service_tspps, ble_service.uuid_16, ATT_BT_UUID_SIZE);
        sys_memcpy_swap(tspps_notify, ble_service.char_notify, ATT_BT_UUID_SIZE);
        sys_memcpy_swap(tspps_write, ble_service.char_write, ATT_BT_UUID_SIZE);
        sys_memcpy_swap(tspps_at, ble_service.char_at, ATT_BT_UUID_SIZE);

        static const cs_gatt_serv_t gatt_service_tspps = {
            service_tspps, CS_UUID_16BIT,
            sizeof(atts_tspps) / sizeof(atts_tspps[0]), atts_tspps
        };

        cs_gatts_add_service(&gatt_service_tspps, &tspps_ctx.att_start_handle);
    } else {
        sys_memcpy_swap(service_tspps, ble_service.uuid_128, ATT_UUID_SIZE);
        sys_memcpy_swap(tspps_notify, ble_service.uuid_128, ATT_UUID_SIZE);
        sys_memcpy_swap(tspps_write, ble_service.uuid_128, ATT_UUID_SIZE);
        sys_memcpy_swap(tspps_at, ble_service.uuid_128, ATT_UUID_SIZE);

        sys_memcpy_swap(service_tspps + 12, ble_service.uuid_16, ATT_BT_UUID_SIZE);
        sys_memcpy_swap(tspps_notify + 12, ble_service.char_notify, ATT_BT_UUID_SIZE);
        sys_memcpy_swap(tspps_write + 12, ble_service.char_write, ATT_BT_UUID_SIZE);
        sys_memcpy_swap(tspps_at + 12, ble_service.char_at, ATT_BT_UUID_SIZE);

        static const cs_gatt_serv_t gatt_service_tspps = {
            service_tspps, CS_UUID_128BIT,
            sizeof(atts_tspps_128) / sizeof(atts_tspps_128[0]), atts_tspps_128
        };

        cs_gatts_add_service(&gatt_service_tspps, &tspps_ctx.att_start_handle);
    }
#else
    static const cs_gatt_serv_t gatt_service_tspps = {
        service_tspps, CS_UUID_16BIT,
        sizeof(atts_tspps) / sizeof(atts_tspps[0]), atts_tspps
    };

    cs_gatts_add_service(&gatt_service_tspps, &tspps_ctx.att_start_handle);
#endif


    //CS_LOG_DEBUG("TSPPS-INIT: tspps_ctx.att_start_handle %d\r\n", tspps_ctx.att_start_handle);

}

static bool inline tspps_is_notify_att(uint16_t att_handle)
{
    if (((tspps_ctx.att_start_handle + TSPPS_IDX_NOTIFY_VAL) == att_handle) ||
        ((tspps_ctx.att_start_handle + TSPPS_IDX_AT_VAL) == att_handle)) {
        return true;
    }

    return false;
}

static void tspps_transmit(uint8_t conn_idx, uint16_t att_handle);
static void tspps_event_cb(uint16_t evt_id, const cs_ble_evt_t *evt)
{
    if (evt_id == CS_GATTS_EVT_READ_REQ) {
        const cs_gatts_evt_read_req_t *req = &evt->gatt.read_req;

        CS_LOG_DEBUG("TSPPS-READ: att_handle(%d), conn_idx(%d)\r\n",
                req->att_hdl, evt->gatt.conn_idx);
        if (req->att_hdl == tspps_ctx.att_start_handle + TSPPS_IDX_AT_CFG) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR,
                                   (uint8_t *)&tspps_ctx.at_enabled[evt->gatt.conn_idx], sizeof(uint16_t));
        } else if (req->att_hdl == tspps_ctx.att_start_handle + TSPPS_IDX_NOTIFY_CFG) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR,
                                   (uint8_t *)&tspps_ctx.notify_enabled[evt->gatt.conn_idx], sizeof(uint16_t));
        }
    } else if (evt_id == CS_GATTS_EVT_WRITE_REQ) {
        const cs_gatts_evt_write_req_t *req = &evt->gatt.write_req;

        CS_LOG_DEBUG("TSPPS-WRITE: att_handle(%d), conn_idx(%d)\r\n",
                req->att_hdl, evt->gatt.conn_idx);
        if (req->att_hdl == tspps_ctx.att_start_handle + TSPPS_IDX_AT_CFG) {
            tspps_ctx.at_enabled[evt->gatt.conn_idx] = req->data[0];
        } else if (req->att_hdl == tspps_ctx.att_start_handle + TSPPS_IDX_NOTIFY_CFG) {
            tspps_ctx.notify_enabled[evt->gatt.conn_idx] = req->data[0];
        } else if ((req->att_hdl == tspps_ctx.att_start_handle + TSPPS_IDX_WRITE_VAL) ||
                   (req->att_hdl == tspps_ctx.att_start_handle + TSPPS_IDX_AT_VAL)) {
#ifdef AT_ENABLE
            uart_slave_rx_data(evt->gatt.conn_idx, req->att_hdl, req->data, req->len);
#endif
        }
    } else if (evt_id == CS_GAP_EVT_CONNECTED) {
        // default to enable.
        tspps_ctx.at_enabled[evt->gatt.conn_idx] = 1;
        tspps_ctx.notify_enabled[evt->gatt.conn_idx] = 1;
        tspps_ctx.mtu[evt->gatt.conn_idx] = TSPPS_MTU_DEFAULT;
    } else if (evt_id == CS_GAP_EVT_DISCONNECTED) {
    } else if (evt_id == CS_BLE_GATT_EVT_MTU_EXCHANGED) {
        tspps_ctx.mtu[evt->gatt.conn_idx] = evt->gatt.mtu_exchanged.mtu - 3;
    } else if (evt_id == CS_BLE_GATT_EVT_TX_COMPLETE) {
        if (tspps_is_notify_att(evt->gatt.tx_complete.id)) {
            tspps_ctx.state[evt->gatt.conn_idx] = TSPPS_STATE_IDLE;
            tspps_transmit(evt->gatt.conn_idx, evt->gatt.tx_complete.id);
        }
    }
}

static void tspps_transmit(uint8_t conn_idx, uint16_t att_handle)
{
    tspps_size_t data_len = cs_fifo_len(&tspps_ctx.tspps_fifo[conn_idx]);
    if (data_len) {
        if (tspps_ctx.state[conn_idx] == TSPPS_STATE_BUSY) {
            return;
        }

        uint16_t send_len;
#if TSPPS_STREAM_MODE
        send_len = tspps_ctx.mtu[conn_idx] < TSPPS_MAX_MTU_SIZE ? tspps_ctx.mtu[conn_idx] : TSPPS_MAX_MTU_SIZE;
        if (send_len > data_len) {
            send_len  = data_len;
        }
#else
        tspps_size_t len;
        cs_fifo_out(&tspps_ctx.tspps_fifo[conn_idx], &len, sizeof(len));
        send_len = len;
#endif
        uint8_t send_data[TSPPS_MAX_MTU_SIZE];
        send_len = cs_fifo_out(&tspps_ctx.tspps_fifo[conn_idx], send_data, send_len);

        cs_gatts_hvx_t hvx = {
            CS_HANDLE_VALUE_NTF,
            att_handle,
            send_data,
            send_len,
            att_handle,
        };

        uint32_t status = cs_gatts_send_hvx(conn_idx, &hvx);
        CS_LOG_DEBUG("%s: status %d\r\n", __func__, status);
        tspps_ctx.state[conn_idx] = TSPPS_STATE_BUSY;
    }
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */

uint8_t tspps_send(const uint8_t *data, tspps_size_t len, uint8_t conn_idx, uint16_t att_handle)
{
    uint8_t notify_enabled = 0;

    if (conn_idx >= TSPPS_CONN_MAX) {
        return TSPPS_ERR_CONN_INVALID;
    }

#if !TSPPS_STREAM_MODE
    if (len > tspps_ctx.mtu[conn_idx] || len > TSPPS_MAX_MTU_SIZE) {
        return TSPPS_ERR_EXCEED_MTU;
    }
#endif

    if ((tspps_ctx.att_start_handle + TSPPS_IDX_NOTIFY_VAL) == att_handle) {
        notify_enabled = tspps_ctx.notify_enabled[conn_idx];
    } else if ((tspps_ctx.att_start_handle + TSPPS_IDX_AT_VAL) == att_handle) {
        notify_enabled = tspps_ctx.at_enabled[conn_idx];
    }
    if (!notify_enabled) {
        return TSPPS_ERR_DISABLED;
    }

    tspps_size_t avail_len = tspps_avail(conn_idx);
    if (len > avail_len) {
        return TSPPS_ERR_FULL;
    }

#if TSPPS_STREAM_MODE
    cs_fifo_in(&tspps_ctx.tspps_fifo[conn_idx], data, len);
#else
    cs_fifo_in(&tspps_ctx.tspps_fifo[conn_idx], &len, sizeof(len));
    cs_fifo_in(&tspps_ctx.tspps_fifo[conn_idx], data, len);
#endif
    tspps_transmit(conn_idx, att_handle);
    
    return 0;
}

uint16_t tspps_avail(uint8_t conn_idx)
{
    tspps_size_t avail_len = cs_fifo_avail(&tspps_ctx.tspps_fifo[conn_idx]);
#if !TSPPS_STREAM_MODE
    if (avail_len < sizeof(avail_len)) {
        avail_len = 0;
    } else {
        avail_len -= sizeof(avail_len);
    }
#endif
    return avail_len;
}

void app_tspp_init(void)
{
    service_tspp_init();
    cs_event_callback_reg(tspps_event_cb);
}

/** @} */
