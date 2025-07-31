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
 * @file     app_hid_media.c
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
#include <stdint.h>
#include <stdlib.h>
#include "cs_ble.h"
#include "app_hid_media.h"
#include "evt.h"

/*********************************************************************
 * MACROS
 */
#define APP_HID_MEDIA_TEST 1

#define PROP_HID_REP  (ATT_PROP_READ  | ATT_PROP_WRITE  | ATT_PROP_NTF)
#define PROP_HID_W_R  (ATT_PROP_WRITE | ATT_PROP_READ)
#define ATT_LEN_DESC  sizeof(uint16_t)

#define KEY_PENDING_UP       0x00
#define KEY_PENDING_NOTHING  0xFF
#define HID_MEDIA_NTF_SEQ    ATT_SVC_HID

/*********************************************************************
 * LOCAL VARIABLES
 */
// Start handle
static uint16_t start_handle_hid_media;
static uint8_t hid_media_pairing;
static uint8_t hid_media_encrypted;
static uint8_t hid_media_enabled;

/*********************************************************************
 * TYPEDEFS
 */
enum {
    IDX_HID_SVC,
    IDX_HID_REP_MAP_CHAR,
    IDX_HID_REP_MAP_VAL,
    IDX_HID_REP1_CHAR,
    IDX_HID_REP1_VAL,
    IDX_HID_REP1_DESC,
    IDX_HID_REP1_REF,
    IDX_HID_INFO_CHAR,
    IDX_HID_INFO_VAL,
    IDX_HID_NUM,
};

static const uint8_t hid_rep_map[] = {
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    //    0x85, 0x01,        //   Report ID (2)
    0x15, 0x00,        //   Logical Minimum (0)
    0x25, 0x01,        //   Logical Maximum (1)
    0x09, 0xE9,        //   Usage (Volume Increment)
    0x09, 0xEA,        //   Usage (Volume Decrement)
    0x09, 0xB1,        //   Usage (Pause)
    0x09, 0xB0,        //   Usage (Play)
    0x09, 0xB5,        //   Usage (Scan Next Track)
    0x09, 0xB6,        //   Usage (Scan Previous Track)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x06,        //   Report Count (6)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
#define HID_MEDIA_VOL_INC 0x01
#define HID_MEDIA_VOL_DEC 0x02
#define HID_MEDIA_PAUSE   0x04
#define HID_MEDIA_PLAY    0x08
#define HID_MEDIA_NEXT    0x10
#define HID_MEDIA_PREV    0x20
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void media_hid_service_init(void)
{
    static const uint8_t serv_hid[2]   = {0x12, 0x18};
    static const uint8_t hid_map[2]    = {0x4b, 0x2a};
    static const uint8_t hid_report[2] = {0x4d, 0x2a};
    static const uint8_t hid_info[2]   = {0x4a, 0x2a};
    static const cs_gatt_item_t atts_hid[] = {
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { hid_map,         CS_UUID_16BIT, CS_ATT_PROP_READ, true, true },
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { hid_report,      CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE | CS_ATT_PROP_NTF, true, true },
        { ob_att_cccd_def, CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE },
        { ob_att_rrd_def,  CS_UUID_16BIT, CS_ATT_PROP_READ | CS_ATT_PROP_WRITE },
        { ob_att_char_def, CS_UUID_16BIT, CS_ATT_PROP_READ },
        { hid_info,        CS_UUID_16BIT, CS_ATT_PROP_READ, true, true},
    };
    static const cs_gatt_serv_t att_serv_hid = {
        serv_hid, CS_UUID_16BIT,
        sizeof(atts_hid) / sizeof(atts_hid[0]), atts_hid
    };
    cs_gatts_add_service(&att_serv_hid, &start_handle_hid_media);
}

#if APP_HID_MEDIA_TEST
static int hid_send_count;
static evt_timer_t test_timer;
void test_timer_cb(evt_timer_t *timer, void *param)
{
    if (hid_send_count < 10) {
        hid_media_vol_inc();
    } else {
        hid_media_vol_dec();
    }
    if (hid_send_count++ < 20) {
        evt_timer_set(&test_timer, 25, EVT_TIMER_REPEAT, test_timer_cb, NULL);
    }
}
#endif

static void hid_event_cb(uint16_t evt_id, const cs_ble_evt_t *evt)
{
    if (evt_id == CS_GATTS_EVT_READ_REQ) {
        CS_LOG_DEBUG("CS_GATTS_EVT_READ_REQ \r\n");
        const cs_gatts_evt_read_req_t *req = &evt->gatt.read_req;
        if (req->att_hdl == start_handle_hid_media + IDX_HID_REP_MAP_VAL) {
            CS_LOG_DEBUG("REPORT MAP\r\n");
            const uint8_t *map = &hid_rep_map[evt->gatt.read_req.offset];
            int map_len = sizeof(hid_rep_map) - evt->gatt.read_req.offset;
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, map, map_len);
        } else if (req->att_hdl == start_handle_hid_media + IDX_HID_REP1_REF) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, (uint8_t *)HID_REF1, sizeof(HID_REF1) - 1);
        } else if (req->att_hdl == start_handle_hid_media + IDX_HID_INFO_VAL) {
            cs_gatts_read_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR, (uint8_t *)HID_INFO, sizeof(HID_INFO) - 1);
        }
    } else if (evt_id == CS_GATTS_EVT_WRITE_REQ) {
        CS_LOG_DEBUG("CS_GATTS_EVT_WRITE_REQ \r\n");
        const cs_gatts_evt_write_req_t *req = &evt->gatt.write_req;
        if (start_handle_hid_media < req->att_hdl && req->att_hdl < start_handle_hid_media + IDX_HID_NUM) {
            cs_gatts_write_response(evt->gatt.conn_idx, CS_BLE_GATT_ERR_NO_ERROR);
            if (req->att_hdl == start_handle_hid_media + IDX_HID_REP1_DESC) {
                if (hid_media_encrypted && !hid_media_enabled) {
                    hid_media_enabled = true;
                    #if APP_HID_MEDIA_TEST
                    hid_send_count = 0;
                    evt_timer_set(&test_timer, 25, EVT_TIMER_REPEAT, test_timer_cb, NULL);
                    #endif
                }
            }
        }
    } else if (evt_id == CS_GAP_EVT_CONNECTED) {
        hid_media_pairing = false;
        hid_media_encrypted = false;
    } else if (evt_id == CS_GAP_EVT_DISCONNECTED) {
        hid_media_enabled = false;
        #if APP_HID_MEDIA_TEST
        evt_timer_del(&test_timer);
        #endif
    } else if (evt_id == CS_GAP_EVT_PAIRING_REQUEST) {
        hid_media_pairing = true;
    } else if (evt_id == CS_GAP_EVT_ENCRYPT) {
        hid_media_encrypted = true;
        if (!hid_media_pairing) {
            hid_media_enabled = true;
            #if APP_HID_MEDIA_TEST
            hid_send_count = 0;
            evt_timer_set(&test_timer, 25, EVT_TIMER_REPEAT, test_timer_cb, NULL);
            #endif
        }
    }
}

static void hid_media_key_send(uint8_t key)
{
    if (!hid_media_enabled) {
        return;
    }

    cs_gatts_hvx_t hvx = {
        CS_HANDLE_VALUE_NTF,
        start_handle_hid_media + IDX_HID_REP1_VAL, // handle
        &key, sizeof(key)
    };
    cs_gatts_send_hvx(0, &hvx); // Key down
    key = 0;
    cs_gatts_send_hvx(0, &hvx); // Key up
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/// HID Increase volume
void hid_media_vol_inc(void)
{
    hid_media_key_send(HID_MEDIA_VOL_INC);
}

/// HID Decrease volume
void hid_media_vol_dec(void)
{
    hid_media_key_send(HID_MEDIA_VOL_DEC);
}

/// HID Pause media
void hid_media_pause(void)
{
    hid_media_key_send(HID_MEDIA_PAUSE);
}

/// HID Play media
void hid_media_play(void)
{
    hid_media_key_send(HID_MEDIA_PLAY);
}

/// HID Play next media
void hid_media_next(void)
{
    hid_media_key_send(HID_MEDIA_NEXT);
}

/// HID Play previous media
void hid_media_prev(void)
{
    hid_media_key_send(HID_MEDIA_PREV);
}

void app_media_hid_init(void)
{
    media_hid_service_init();
    cs_event_callback_reg(hid_event_cb);
}

/** @} */
