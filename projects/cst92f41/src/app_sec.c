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
 * @file     app_sec.c
 * @date     30. Jun. 2023
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
#include <stdlib.h>
#include "cs_ble.h"
#include "cs_driver.h"

/*********************************************************************
 * MACROS
 */
#define log_debug CS_LOG_DEBUG
#define hexdump(d, l) do{for(int i=0;i<l;i++)log_debug("%02X ", ((uint8_t*)d)[i]);log_debug("\n");}while(0);
#define PERIPHERAL_IOCAP  CS_BLE_SMP_IOCAP_NONE

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * EXTERN FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief  BLE event process callback
 * @param[in] evt_id  event id
 * @param[in] evt     event parameters
 *******************************************************************************
 */
static void app_sec_event_cb(uint16_t evt_id, const cs_ble_evt_t *evt)
{
    if (evt_id == CS_GAP_EVT_CONNECTED) {
        log_debug("CS_GAP_EVT_CONNECTED(%d): %d\n", evt->gap.conn_idx, evt->gap.connected.adv_idx);
    } else if (evt_id == CS_GAP_EVT_DISCONNECTED) {
        log_debug("CS_GAP_EVT_DISCONNECTED 0x%02X\n", evt->gap.disconnected.reason);
    } else if (evt_id == CS_GAP_EVT_PIN_REQUEST) {
        log_debug("CS_GAP_EVT_PIN_REQUEST:%d\n", evt->gap.pin_request.type);
        cs_smp_pin_t pin_info;
        switch (evt->gap.pin_request.type) {
            case CS_BLE_SMP_PIN_TYPE_DIS:
                log_debug("show pin:%06d\n", evt->gap.pin_request.pin_code);
            case CS_BLE_SMP_PIN_TYPE_NONE:
                cs_gap_pin_response(evt->gap.conn_idx, true, &pin_info);
                break;
            case CS_BLE_SMP_PIN_TYPE_PK_REQ:
            case CS_BLE_SMP_PIN_TYPE_OOB_REQ: {
                break;
            }
        }
    } else if (evt_id == CS_GAP_EVT_BONDED) {
        log_debug("CS_GAP_EVT_BONDED: 0x%04X\n", evt->gap.bonded.status);
    } else if (evt_id == CS_GAP_EVT_BOND_INFO_REQUEST) {
        log_debug("CS_GAP_EVT_BOND_INFO_REQUEST:%s\n", evt->gap.bond_info_request.type == CS_BOND_INFO_LTK ? "LTK" : "IRK");
        cs_bond_info_t bond_info = { evt->gap.bond_info_request.type };
        if (evt->gap.bond_info_request.type == CS_BOND_INFO_LTK) {
            memcpy(bond_info.enc_info.ltk, (uint8_t *)"LTKLTKLTKLTKLTK", 16);
            memcpy(bond_info.enc_info.random, (uint8_t *)"_RANDOM_", 8);
            bond_info.enc_info.ediv = 0x3344;
        } else {
            uint8_t addr[7] = {CS_ADV_ADDR_TYPE_RANDOM};
            cs_gap_addr_get(CS_ADV_ADDR_TYPE_RANDOM, &addr[1]);
            memcpy(bond_info.id_info.irk, (uint8_t *)"IRKIRKIRKIRKIRK", 16);
            memcpy(&bond_info.id_info.id_addr, addr, sizeof(addr));
        }
        cs_gap_bond_info_response(evt->gap.conn_idx, &bond_info);
    } else if (evt_id == CS_GAP_EVT_LTK_REQUEST) {
        log_debug("CS_GAP_EVT_LTK_REQUEST\n");
        cs_gap_ltk_response(evt->gap.conn_idx, (uint8_t *)"LTKLTKLTKLTKLTK");
    } else if (evt_id == CS_GAP_EVT_PAIRING_REQUEST) {
        log_debug("CS_GAP_EVT_PAIRING_REQUEST\n");
        cs_pairing_param_t response;
        response.authreq.bond_flags = 1;
        response.authreq.mitm = 1;
        response.authreq.sc = 0;
        response.oob_data_flag = 0;
        response.initiator_key_distribution = CS_BLE_SMP_DIST_BIT_ENC_KEY | CS_BLE_SMP_DIST_BIT_ID_KEY;
        response.responder_key_distribution = CS_BLE_SMP_DIST_BIT_ENC_KEY;
        response.io_capability = PERIPHERAL_IOCAP;
        cs_gap_pairing_response(evt->gap.conn_idx, &response);
    } else if (evt_id == CS_GAP_EVT_BOND_INFO) {
        log_debug("CS_GAP_EVT_BOND_INFO\n");
        if (evt->gap.bond_info.type == CS_BOND_INFO_LTK) {
            log_debug("Recv LTK: ");
            hexdump(evt->gap.bond_info.enc_info.ltk, 16);
            log_debug("Recv EDIV: 0x%04X\n", evt->gap.bond_info.enc_info.ediv);
            log_debug("Recv RANDOM: ");
            hexdump(evt->gap.bond_info.enc_info.random, 8);
        } else {
            log_debug("Recv IRK: ");
            hexdump(evt->gap.bond_info.id_info.irk, 16);
            log_debug("Recv ID: %01X ", evt->gap.bond_info.id_info.id_addr.addr_type);
            hexdump(&evt->gap.bond_info.id_info.id_addr.addr, 6);
        }
    } else if (evt_id == CS_GAP_EVT_ENCRYPT) {
        log_debug("CS_GAP_EVT_ENCRYPT: %d\n", evt->gap.encrypt.encrypted);
    } else {
    }
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief  Init advertising module
 *******************************************************************************
 */
void app_sec_init(void)
{
    cs_event_callback_reg(app_sec_event_cb);
}

/** @} */
