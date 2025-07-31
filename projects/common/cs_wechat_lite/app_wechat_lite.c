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
 * @file     app_wechat_lite.c
 * @date     27 Match 2024
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
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "cs_ble.h"
#include "app_wechat_lite.h"

#define WECHAT_LITE_UUID 0xFEE7
#define PROP_WCL_MEAS (ATT_PROP_READ | ATT_PROP_IND | ATT_PROP_NTF)
#define PROP_WCL_TARG (ATT_PROP_READ | ATT_PROP_WRITE | ATT_PROP_IND)
#define PROP_WCL_W_R  (ATT_PROP_WRITE | ATT_PROP_READ)
#define ATT_LEN_DESC  sizeof(uint16_t)

// Start handle
static uint16_t m_start_handle;
static uint8_t mac_address[6];
static uint8_t measurement[10]; // | Flag(1) | StepCount(3) | StepDistancer(3) | StepCalorie(3) |
static uint8_t target[4]; // | Flag(1) | StepCount(3) |
static uint8_t notify_measure;
static uint8_t notify_target;

enum {
    IDX_WCL_SVC,
    IDX_WCL_MEASURE_CHAR,
    IDX_WCL_MEASURE_VAL,
    IDX_WCL_MEASURE_DESC,
    IDX_WCL_TARGET_CHAR,
    IDX_WCL_TARGET_VAL,
    IDX_WCL_TARGET_DESC,
    IDX_WCL_ADDRESS_CHAR,
    IDX_WCL_ADDRESS_VAL,
};

static void set_address(void)
{
    uint8_t addr[CS_GAP_ADDR_LEN] = { 0x01, 0xA1, 0x26, 0x66, 0x5F, 0x01 };
    memcpy(mac_address, addr, 6); // Warnning: Address MUST same as advertise
}

static void wechat_lite_notify(void)
{
    if (notify_measure) {
        notify_measure = 0;
        uint16_t len = sizeof(measurement);
        cs_gatts_hvx_t hvx = {
            (notify_measure & 0x02) ? CS_HANDLE_VALUE_IND : CS_HANDLE_VALUE_NTF,
            m_start_handle + IDX_WCL_MEASURE_VAL, // handle
            measurement, len, WECHAT_LITE_UUID
        };
        cs_gatts_send_hvx(0, &hvx);
    } else if (notify_target) {
        notify_target = 0;
        uint16_t len = sizeof(target);
        cs_gatts_hvx_t hvx = {
            CS_HANDLE_VALUE_IND,
            m_start_handle + IDX_WCL_TARGET_VAL, // handle
            target, len, WECHAT_LITE_UUID
        };
        cs_gatts_send_hvx(0, &hvx);
    }
}

void wechat_lite_service_init(void)
{
    static const uint8_t serv_wechat[2]   = {WECHAT_LITE_UUID & 0xFF, WECHAT_LITE_UUID >> 8};
    static const uint8_t wechat_meas[2]   = {0xa1, 0xfe};
    static const uint8_t wechat_target[2] = {0xa2, 0xfe};
    static const uint8_t wechat_mac[2]    = {0xc9, 0xfe};
    static const cs_gatt_item_t atts_hid[] = {
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { wechat_meas,     CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_NTF | CS_ATT_PROP_IND },
        { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE },
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { wechat_target,   CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE | CS_ATT_PROP_IND },
        { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE },
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { wechat_mac,      CS_UUID_16BIT, CS_ATT_PROP_READ },
        { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE },
    };
    static const cs_gatt_serv_t att_serv_hid = {
        serv_wechat, CS_UUID_16BIT,
        sizeof(atts_hid) / sizeof(atts_hid[0]), atts_hid
    };
    cs_gatts_add_service(&att_serv_hid, &m_start_handle);
}

void service_wechat_lite_event_cb(uint16_t evt_id, const cs_ble_evt_t *evt)
{
    if (evt_id == CS_GAP_EVT_CONNECTED) {
        set_address();
    } else if (evt_id == CS_BLE_GATT_EVT_TX_COMPLETE) {
        if (evt->gatt.tx_complete.id == WECHAT_LITE_UUID) {
            if (notify_measure || notify_target) {
                wechat_lite_notify();
            }
        }
    } else if (evt_id == CS_GATTS_EVT_READ_REQ) {
        if (evt->gatt.read_req.att_hdl == m_start_handle + IDX_WCL_MEASURE_VAL) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, measurement, sizeof(measurement));
        } else if (evt->gatt.read_req.att_hdl == m_start_handle + IDX_WCL_TARGET_VAL) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, target, sizeof(target));
        } else if (evt->gatt.read_req.att_hdl == m_start_handle + IDX_WCL_ADDRESS_VAL) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, mac_address, sizeof(mac_address));
        } else if (evt->gatt.read_req.att_hdl == m_start_handle + IDX_WCL_MEASURE_DESC) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, (uint8_t *)"\x01\x00", sizeof(uint16_t));
        } else if (evt->gatt.read_req.att_hdl == m_start_handle + IDX_WCL_TARGET_DESC) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, (uint8_t *)"\x01\x00", sizeof(uint16_t));
        }
    }
    #if 1 // Just for test with AirSyncDebugger
    else if (evt_id == CS_GATTS_EVT_WRITE_REQ) {
        uint8_t measure[] = {0x07, 0x10, 0x27, 0x00, 0x70, 0x17, 0x00, 0x00, 0x01, 0x00};
        uint8_t target[] = {0x01, 0x10, 0x27, 0x00};
        if (evt->gatt.write_req.att_hdl == m_start_handle + IDX_WCL_MEASURE_DESC) {
            wechat_lite_set_measurement(measure);
        } else if (evt->gatt.write_req.att_hdl == m_start_handle + IDX_WCL_TARGET_DESC) {
            wechat_lite_set_target(target);
        } else if (evt->gatt.write_req.att_hdl == m_start_handle + IDX_WCL_TARGET_VAL) {
            wechat_lite_set_measurement(measure);
            wechat_lite_set_target(target);
        }
    }
    #endif
}

void app_wechat_lite_init(void)
{
    wechat_lite_service_init();
    cs_event_callback_reg(service_wechat_lite_event_cb);
}


void wechat_lite_set_measurement(uint8_t* new_measure)
{
    memcpy(measurement, new_measure, sizeof(measurement));
    notify_measure = true;
    wechat_lite_notify();
}

void wechat_lite_set_target(uint8_t* new_target)
{
    memcpy(target, new_target, sizeof(target));
    notify_target = true;
    wechat_lite_notify();
}

