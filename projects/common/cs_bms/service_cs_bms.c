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
 * @file     service_cs_bms.c
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
#include "service_cs_bms.h"
#include "cs_ble.h"
#include "cs_bms.h"

#define BMS_SERV_UUID                   0x181E
#define BMS_CTRL_POINT                  0x2AA4
#define BMS_FEATURE                     0x2AA5

enum {
    IDX_BMS_SVC,
    IDX_CTRL_POINT_CHAR,
    IDX_CTRL_POINT_VAL,
    IDX_FEATURE_CHAR,
    IDX_FEATURE_VAL,
};
/*********************************************************************
 * LOCAL VARIABLES
 */
static uint16_t m_start_handle;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void service_cs_bms_evt_cb(uint16_t evt_id, const cs_ble_evt_t *evt)
{
    if (evt_id == CS_GATTS_EVT_WRITE_REQ) {
        const cs_gatts_evt_write_req_t *req = &evt->gatt.write_req;
        if (req->att_hdl == m_start_handle + IDX_CTRL_POINT_VAL) {
            uint8_t res = cs_bms_write_ops_ctrl_adapt(req->data, req->len);
            cs_gatts_write_response(evt->gatt.conn_idx, res);
        }
    } else if (evt_id == CS_GATTS_EVT_READ_REQ) {
        const cs_gatts_evt_read_req_t *req = &evt->gatt.read_req;
        if (req->att_hdl == m_start_handle + IDX_FEATURE_VAL) {
            uint8_t resp_data[3];
            cs_bms_read_feature_adapt(resp_data);
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, (uint8_t *)resp_data, sizeof(resp_data));
        }
    }
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
void cs_bms_service_init(void)
{
    static const uint8_t serv_bms[2]  = {0x1e, 0x18};
    static const uint8_t char_ctrl_point[2] = {0xa4, 0x2a};
    static const uint8_t char_feature[2] = {0xa5, 0x2a};
    static const cs_gatt_item_t atts_dev[] = {
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { char_ctrl_point, CS_UUID_16BIT, CS_ATT_PROP_WRITE },
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { char_feature,     CS_UUID_16BIT, CS_ATT_PROP_READ },
    };
    static const cs_gatt_serv_t att_serv = {
        serv_bms, CS_UUID_16BIT,
        sizeof(atts_dev) / sizeof(atts_dev[0]), atts_dev
    };
    cs_gatts_add_service(&att_serv, &m_start_handle);
}

void service_cs_bms_init(void)
{
    cs_bms_service_init();
    cs_event_callback_reg(service_cs_bms_evt_cb);
}

/** @} */
