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
 * @file     service_chipsea_dfu.c
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @version
 * Version V20210119.1.0
 *  - Initial release
 *
 * @{
 */
/*******************************************************************************
 * INCLUDES
 */
#include <string.h>
#include "cs_ble.h"
#include "chipsea_dfu.h"
#include "service_chipsea_dfu.h"

/*********************************************************************
 * MACROS
 */
#define PROP_DFU_W_N  (ATT_PROP_WRITE | ATT_PROP_NTF)
#define PROP_DFU_WRN  (ATT_PROP_WRITE | ATT_PROP_READ  | ATT_PROP_NTF)
#define PROP_DFU_W_R  (ATT_PROP_WRITE | ATT_PROP_READ)
#define ATT_LEN_CCCD  sizeof(uint16_t)
#define DFU_VER_SIZE  (sizeof(DFU_APP_DESCRIBE)-1)
/*********************************************************************
 * LOCAL VARIABLES
 */
// Start handle
static uint16_t m_start_handle;

#define NOT_END 0xFF
#define END_RESET 0xFE
static uint8_t m_end_state = NOT_END;

enum {
    IDX_DFU_SVC,
    IDX_DFU_CTRL_CHAR,
    IDX_DFU_CTRL_VAL,
    IDX_DFU_CTRL_DESC,
    IDX_DFU_PKG_CHAR,
    IDX_DFU_PKG_VAL,
    IDX_DFU_VER_CHAR,
    IDX_DFU_VER_VAL,
    IDX_DFU_VER_DESC,
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void service_chipsea_dfu_init(void)
{
    static const uint8_t serv_dfu[16] = {0x59, 0xFE};
    static const uint8_t char1[16] = {0x50, 0xEA, 0xDA, 0x30, 0x88, 0x83, 0xB8, 0x9F, 0x60, 0x4F, 0x15, 0xF3, 0x01, 0x00, 0xC9, 0x8E};
    static const uint8_t char2[16] = {0x50, 0xEA, 0xDA, 0x30, 0x88, 0x83, 0xB8, 0x9F, 0x60, 0x4F, 0x15, 0xF3, 0x02, 0x00, 0xC9, 0x8E};
    static const uint8_t char3[16] = {0x03, 0x00};

    static const cs_gatt_item_t atts_dfu[] = {
        { ob_att_char_def, CS_UUID_16BIT,  CS_ATT_PROP_READ },
        { char1,           CS_UUID_128BIT, CS_ATT_PROP_NTF | CS_ATT_PROP_WRITE },
        { ob_att_cccd_def, CS_UUID_16BIT,  CS_ATT_PROP_READ | CS_ATT_PROP_WRITE },
        { ob_att_char_def, CS_UUID_16BIT,  CS_ATT_PROP_READ },
        { char2,           CS_UUID_128BIT, CS_ATT_PROP_WRITE_CMD },
        { ob_att_char_def, CS_UUID_16BIT,  CS_ATT_PROP_READ },
        { char3,           CS_UUID_16BIT,  CS_ATT_PROP_READ | CS_ATT_PROP_WRITE | CS_ATT_PROP_NTF },
        { ob_att_cccd_def, CS_UUID_16BIT,  CS_ATT_PROP_READ | CS_ATT_PROP_WRITE },
    };
    static const cs_gatt_serv_t att_serv_dfu = {
        serv_dfu, CS_UUID_16BIT,
        sizeof(atts_dfu) / sizeof(atts_dfu[0]), atts_dfu
    };
    cs_gatts_add_service(&att_serv_dfu, &m_start_handle);
}
static void dfu_response(dfu_response_t *dfu_rsp_data)
{
    uint16_t len = dfu_rsp_data->length;
    cs_gatts_hvx_t hvx = {
        CS_HANDLE_VALUE_NTF,
        m_start_handle + IDX_DFU_CTRL_VAL, // handle
        &dfu_rsp_data->rsp_code,
        len,
    };
    cs_gatts_send_hvx(0, &hvx);
}

static void dfu_response_version(dfu_version_t *dfu_version)
{
    uint16_t len = sizeof(struct dfu_version_data);
    cs_gatts_hvx_t hvx = {
        CS_HANDLE_VALUE_NTF,
        m_start_handle + IDX_DFU_VER_VAL, // handle
        (uint8_t *) &dfu_version->version_data,
        len,
    };
    cs_gatts_send_hvx(0, &hvx);
}

void service_chipsea_dfu_evt_cb(uint16_t evt_id, const cs_ble_evt_t *evt)
{
    if (evt_id == CS_GATTS_EVT_WRITE_REQ) {
        const uint8_t *data = evt->gatt.write_req.data;
        uint16_t len = evt->gatt.write_req.len;
        dfu_response_t dfu_rsp_data = {0};
        switch (evt->gatt.write_req.att_hdl - m_start_handle) {
            case IDX_DFU_CTRL_VAL:
                dfu_write_cmd((uint8_t *)data, len, &dfu_rsp_data);
                dfu_response(&dfu_rsp_data);
                break;
            case IDX_DFU_PKG_VAL:
                dfu_write_data(data, len, &dfu_rsp_data);
                if (dfu_rsp_data.length != DFU_RESP_SIZE_NO_DATA) {
                    dfu_response(&dfu_rsp_data);
                }
                break;
            case IDX_DFU_VER_VAL: {
                dfu_version_t version;
                uint32_t cmd = 0xFFFFFFFF;
                if (len == 4) {
                    cmd = (data[0] << 24) + (data[1] << 16) + (data[2] << 8) + (data[3]);
                } else if (len == 1) {
                    cmd = data[0];
                }
                dfu_write_version_char(cmd, &version);
                dfu_response_version(&version);
            }   break;
        }
    } else if (evt_id == CS_GAP_EVT_CONNECTED) {
        m_end_state = NOT_END;
    } else if (evt_id == CS_GAP_EVT_DISCONNECTED) {
        dfu_reset(DFU_UPDATE_ST_DISCONNECT);
        if (m_end_state == DFU_UPDATE_ST_SUCCESS) {
            drv_pmu_force_reboot();
        }
    } else if (evt_id == CS_BLE_GATT_EVT_TX_COMPLETE) {
        if (m_end_state != NOT_END) {
            app_chipsea_dfu_update_end_ind_handler(m_end_state, NULL);
        }
    }
}

static void dfu_begin_ind_handler(uint8_t status, void *p)
{
    app_chipsea_dfu_update_start_ind_handler(status, p);
}

static void dfu_prog_ind_handler(uint8_t status, void *p)
{
    app_chipsea_dfu_update_prog_ind_handler(status, p);
}

static void dfu_end_ind_handler(uint8_t status, void *p)
{
    m_end_state = status;
}

const dfu_cb_itf_t dfu_cb_itf = {
    dfu_begin_ind_handler,
    dfu_prog_ind_handler,
    dfu_end_ind_handler,
};


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
__WEAK void app_chipsea_dfu_update_start_ind_handler(uint8_t status, void *p)
{
    cs_gap_conn_params_t param = {0x0c, 0, 500};
    cs_gap_conn_param_update(0, &param);
}
__WEAK void app_chipsea_dfu_update_prog_ind_handler(uint8_t status, void *p) {}
__WEAK void app_chipsea_dfu_update_end_ind_handler(uint8_t status, void *p)
{
    cs_gap_disconnect(0, 0x13);
}

void app_chipsea_dfu_init(void)
{
    service_chipsea_dfu_init();
    cs_event_callback_reg(service_chipsea_dfu_evt_cb);
}

/** @} */
