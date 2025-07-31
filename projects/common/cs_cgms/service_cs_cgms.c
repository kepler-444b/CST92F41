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
 * @file     service_cs_cgms.c
 * @date     30. Dec. 2023
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
#include "service_cs_cgms.h"
#include "cs_cgms.h"
#include "cs_cgms_type.h"
#include "cs_ble.h"

/*********************************************************************
 * MACROS
 */
#define CGMS_SERV_UUID                  0x181F
#define CGMS_MEASUREMENT_UUID           0x2AA7
#define CGMS_FEATURE_UUID               0x2AA8
#define CGMS_STATUS_UUID                0x2AA9
#define CGMS_SESSION_RUN_TIME_UUID      0x2AAB
#define CGMS_SESSION_START_TIME_UUID    0x2AAA
#define RECORD_ACCESS_POINT_UUID        0x2A52
#define CGMS_OPS_CONTROL_UUID           0x2AAC

#define CGMS_GAP_CONN_INVALID           0xFF
#define CGMS_NOTIFY_ID                  0xA7

/*********************************************************************
 * TYPEDEFS
 */
enum {
    IDX_CGMS_SVC,
    IDX_CGMS_MEASUREMENT_CHAR,
    IDX_CGMS_MEASUREMENT_VAL,
    IDX_CGMS_MEASUREMENT_DESC,
    IDX_CGMS_FEATURE_CHAR,
    IDX_CGMS_FEATURE_VAL,
    IDX_CGMS_STATUS_CHAR,
    IDX_CGMS_STATUS_VAL,
    IDX_CGMS_SESSION_START_TIME_CHAR,
    IDX_CGMS_SESSION_START_TIME_VAL,
    IDX_CGMS_SESSION_RUN_TIME_CHAR,
    IDX_CGMS_SESSION_RUN_TIME_VAL,
    IDX_RECORD_ACCESS_POINT_CHAR,
    IDX_RECORD_ACCESS_POINT_VAL,
    IDX_RECORD_ACCESS_POINT_DESC,
    IDX_CGMS_OPS_CONTROL_CHAR,
    IDX_CGMS_OPS_CONTROL_VAL,
    IDX_CGMS_OPS_CONTROL_DESC,
};

/*********************************************************************
 * LOCAL VARIABLES
 */
static uint8_t  m_conn_idx;
static uint16_t m_start_handle;
static uint16_t m_measurement_desc;
static uint16_t m_rec_acc_desc;
static uint16_t m_ops_control_desc;
static uint8_t m_pending_racp_data[64]; // len + data
static uint8_t m_pending_ops_data[64]; // len + data

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
void service_cs_cgms_evt_cb(uint16_t evt_id, const cs_ble_evt_t *evt)
{
    if (evt_id == CS_GAP_EVT_CONNECTED) {
        m_conn_idx = evt->gap.conn_idx;
        cs_cgms_connected_adapt();
    } else if (evt_id == CS_GAP_EVT_DISCONNECTED) {
        m_conn_idx = CGMS_GAP_CONN_INVALID;
        m_pending_racp_data[0] = 0;
    } else if (evt_id == CS_GATTS_EVT_READ_REQ) {
        const cs_gatts_evt_read_req_t *req = &evt->gatt.read_req;
        if (req->att_hdl == m_start_handle + IDX_CGMS_MEASUREMENT_DESC) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR,
                                   (uint8_t *)&m_measurement_desc, sizeof(m_measurement_desc));
        } else if (req->att_hdl == m_start_handle + IDX_RECORD_ACCESS_POINT_DESC) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR,
                                   (uint8_t *)&m_rec_acc_desc, sizeof(m_rec_acc_desc));
        } else if (req->att_hdl == m_start_handle + IDX_CGMS_OPS_CONTROL_DESC) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR,
                                   (uint8_t *)&m_ops_control_desc, sizeof(m_ops_control_desc));
        } else if (req->att_hdl == m_start_handle + IDX_CGMS_FEATURE_VAL) {
            uint8_t resp[6];
            uint8_t gatt_resp = cs_cgms_read_feature_adapt(resp);
            cs_gatts_read_response(evt->gatt.conn_idx, gatt_resp, resp, sizeof(cs_cgm_feature_t));
        } else if (req->att_hdl == m_start_handle + IDX_CGMS_STATUS_VAL) {
            uint8_t crc_valid, resp[7];
            uint8_t gatt_resp = cs_cgms_read_state_adapt(resp, &crc_valid);
            cs_gatts_read_response(evt->gatt.conn_idx, gatt_resp, (uint8_t *)resp,
                                   sizeof(uint16_t) + sizeof(cs_cgm_state_t) + !!crc_valid * sizeof(uint16_t));
        } else if (req->att_hdl == m_start_handle + IDX_CGMS_SESSION_START_TIME_VAL) {
            uint8_t crc_valid, resp[11];
            uint8_t gatt_resp = cs_cgms_read_session_start_time_adapt(resp, &crc_valid);
            cs_gatts_read_response(evt->gatt.conn_idx, gatt_resp, (uint8_t *)resp, 9 + !!crc_valid * sizeof(uint16_t));
        } else if (req->att_hdl == m_start_handle + IDX_CGMS_SESSION_RUN_TIME_VAL) {
            uint8_t crc_valid, resp[4];
            uint8_t gatt_resp = cs_cgms_read_session_run_time_adapt(resp, &crc_valid);
            cs_gatts_read_response(evt->gatt.conn_idx, gatt_resp, (uint8_t *)resp,
                                   sizeof(uint16_t) + !!crc_valid * sizeof(uint16_t));
        }
    } else if (evt_id == CS_GATTS_EVT_WRITE_REQ) {
        const cs_gatts_evt_write_req_t *req = &evt->gatt.write_req;
        if (req->att_hdl == m_start_handle + IDX_CGMS_MEASUREMENT_DESC) {
            cs_gatts_write_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR);
            m_measurement_desc = evt->gatt.write_req.data[0];
        } else if (req->att_hdl == m_start_handle + IDX_RECORD_ACCESS_POINT_DESC) {
            cs_gatts_write_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR);
            m_rec_acc_desc = evt->gatt.write_req.data[0];
        } else if (req->att_hdl == m_start_handle + IDX_CGMS_OPS_CONTROL_DESC) {
            cs_gatts_write_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR);
            m_ops_control_desc = evt->gatt.write_req.data[0];
        } else if (req->att_hdl == m_start_handle + IDX_CGMS_SESSION_START_TIME_VAL) {
            uint8_t gatt_resp = cs_cgms_write_session_start_time_adapt(req->data, req->len);
            cs_gatts_write_response(evt->gatt.conn_idx, gatt_resp);
        } else if (req->att_hdl == m_start_handle + IDX_RECORD_ACCESS_POINT_VAL) {
            if (m_measurement_desc == 0) {
                cs_gatts_write_response(evt->gatt.conn_idx, 0xFD);
                return;
            }
            cs_cgms_write_racp_adapt(req->data, req->len);
        } else if (req->att_hdl == m_start_handle + IDX_CGMS_OPS_CONTROL_VAL) {
            if (m_measurement_desc == 0) {
                cs_gatts_write_response(evt->gatt.conn_idx, 0xFD);
                return;
            }
            cs_cgms_write_ops_ctrl_adapt(req->data, req->len);
        }
    } else if (evt_id == CS_GATTS_EVT_INDICATE_CFM) {
        if ( m_pending_racp_data[0] > 0) {
            cs_gatts_hvx_t hvx = {
                CS_HANDLE_VALUE_IND,
                m_start_handle + IDX_RECORD_ACCESS_POINT_VAL, // handle
                &m_pending_racp_data[1],
                m_pending_racp_data[0],
            };
            uint8_t res = cs_gatts_send_hvx(m_conn_idx, &hvx);
            if (res == CS_GATT_ERR_NO_ERROR) {
                m_pending_racp_data[0] = 0;
            }
        }
        if ( m_pending_ops_data[0] > 0) {
            cs_gatts_hvx_t hvx = {
                CS_HANDLE_VALUE_IND,
                m_start_handle + IDX_CGMS_OPS_CONTROL_VAL, // handle
                &m_pending_ops_data[1],
                m_pending_ops_data[0],
            };
            uint8_t res = cs_gatts_send_hvx(m_conn_idx, &hvx);
            if (res == CS_GATT_ERR_NO_ERROR) {
                m_pending_ops_data[0] = 0;
            }
        }
    } else if (evt_id == CS_BLE_GATT_EVT_TX_COMPLETE) {
        if (evt->gatt.tx_complete.id == CGMS_NOTIFY_ID) {
            cs_cgms_measurement_send_done_adapt();
        }
    }
}

void service_cs_cgms_racp_indicate(const uint8_t *data, int len)
{
    if (m_conn_idx != CGMS_GAP_CONN_INVALID) {
        cs_gatts_hvx_t hvx = {
            CS_HANDLE_VALUE_IND,
            m_start_handle + IDX_RECORD_ACCESS_POINT_VAL, // handle
            data,
            len,
        };
        uint8_t res = cs_gatts_send_hvx(m_conn_idx, &hvx);
        if (res != CS_BLE_GATT_ERR_NO_ERROR) {
            len = len < (int)sizeof(m_pending_racp_data) - 1 ? len : (int)sizeof(m_pending_racp_data) - 1;
            m_pending_racp_data[0] = len;
            memcpy(&m_pending_racp_data[1], data, len);
        }
    }
}

void service_cs_cgms_ops_ctrl_indicate(const uint8_t *data, int len)
{
    if (m_conn_idx != CGMS_GAP_CONN_INVALID) {
        cs_gatts_hvx_t hvx = {
            CS_HANDLE_VALUE_IND,
            m_start_handle + IDX_CGMS_OPS_CONTROL_VAL, // handle
            data,
            len,
        };
        uint8_t res = cs_gatts_send_hvx(m_conn_idx, &hvx);
        if (res != CS_BLE_GATT_ERR_NO_ERROR) {
            len = len < (int)sizeof(m_pending_ops_data) - 1 ? len : (int)sizeof(m_pending_ops_data) - 1;
            m_pending_ops_data[0] = len;
            memcpy(&m_pending_ops_data[1], data, len);
        }
    }
}

void service_cs_cgms_measurement_notify(const uint8_t *data, int len)
{
    if (m_conn_idx != CGMS_GAP_CONN_INVALID) {
        cs_gatts_hvx_t hvx = {
            CS_HANDLE_VALUE_NTF,
            m_start_handle + IDX_CGMS_MEASUREMENT_VAL, // handle
            data,
            len,
            CGMS_NOTIFY_ID,
        };
        cs_gatts_send_hvx(m_conn_idx, &hvx);
    }
}

void service_cs_cgms_write_resp(uint8_t gatt_state)
{
    cs_gatts_write_response(m_conn_idx, gatt_state);
}

void cs_cgms_service_init(void)
{
    static const uint8_t serv_esls[2]  = {CGMS_SERV_UUID & 0xFF,            CGMS_SERV_UUID >> 8};
    static const uint8_t char1[2] = {CGMS_MEASUREMENT_UUID & 0xFF,          CGMS_MEASUREMENT_UUID >> 8};
    static const uint8_t char2[2] = {CGMS_FEATURE_UUID & 0xFF,              CGMS_FEATURE_UUID >> 8};
    static const uint8_t char3[2] = {CGMS_STATUS_UUID & 0xFF,               CGMS_STATUS_UUID >> 8};
    static const uint8_t char4[2] = {CGMS_SESSION_START_TIME_UUID & 0xFF,   CGMS_SESSION_START_TIME_UUID >> 8};
    static const uint8_t char5[2] = {CGMS_SESSION_RUN_TIME_UUID & 0xFF,     CGMS_SESSION_RUN_TIME_UUID >> 8};
    static const uint8_t char6[2] = {RECORD_ACCESS_POINT_UUID & 0xFF,       RECORD_ACCESS_POINT_UUID >> 8};
    static const uint8_t char7[2] = {CGMS_OPS_CONTROL_UUID & 0xFF,          CGMS_OPS_CONTROL_UUID >> 8};
    static const cs_gatt_item_t atts_dfu[] = {
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { char1,           CS_UUID_16BIT, CS_ATT_PROP_NTF },
        { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE },
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { char2,           CS_UUID_16BIT, CS_ATT_PROP_READ },
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { char3,           CS_UUID_16BIT, CS_ATT_PROP_READ },
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { char4,           CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE },
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { char5,           CS_UUID_16BIT, CS_ATT_PROP_READ },
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { char6,           CS_UUID_16BIT, CS_ATT_PROP_IND | CS_ATT_PROP_WRITE },
        { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE },
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { char7,           CS_UUID_16BIT, CS_ATT_PROP_IND | CS_ATT_PROP_WRITE },
        { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE },
    };
    static const cs_gatt_serv_t att_serv_dfu = {
        serv_esls, CS_UUID_16BIT,
        sizeof(atts_dfu) / sizeof(atts_dfu[0]), atts_dfu
    };
    cs_gatts_add_service(&att_serv_dfu, &m_start_handle);
}

void service_cs_cgms_init(void)
{
    cs_cgms_service_init();
    cs_event_callback_reg(service_cs_cgms_evt_cb);
}

/** @} */
