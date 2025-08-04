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
 * @file     app_adv.c
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
#include "app_adv.h"
#include "board_cst92f41_evb.h"
#include "sdk_config.h"

/*********************************************************************
 * MACROS
 */
#define log_debug CS_LOG_DEBUG
#define hexdump(d, l)                                                      \
    do {                                                                   \
        for (int i = 0; i < l; i++) log_debug("%02X ", ((uint8_t *)d)[i]); \
        log_debug("\n");                                                   \
    } while (0);

/// the adv data max len
#define APP_ADV_MAX_LEN (31)

/// advertising direct interval
#define APP_ADV_INTERVAL 0x80 /**<  dvertising interval (in units of 0.625 ms. This value corresponds to 100 ms.). */
#define APP_ADV_TIMEOUT  0    /**< The duration of the fast advertising period (in seconds). max 655, or 0 is no timeout*/

#define APP_ADV_NO_ERROR 0

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
/*const static int  app_gap_appearance = 0x03C2;*/
/// Advertise data
static uint8_t s_adv_data[APP_ADV_MAX_LEN] = {
    0x02, ///< length of this data
    0x01,
    0x06,
    0x03, // length of this data including the data type byte
    0xFF, // manufacturer specific adv data type
    0xA7, // Company ID - Fixed
    0x06, // Company ID - Fixed
};

/*const static int  app_gap_appearance = 0x03C2;*/
/// Advertise data
static uint8_t s_rsp_data[APP_ADV_MAX_LEN] = {

};

/// the adv env info
static app_adv_env_tag_t app_adv_env = {
    .adv_data = {s_adv_data, sizeof(s_adv_data)},
    .rsp_data = {s_rsp_data, sizeof(s_rsp_data)},
};

static cs_gap_addr_t last_conn;
static uint8_t s_local_addr[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};

static uint8_t app_connect_state = APP_DISCONNECT;

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
static void app_adv_event_cb(uint16_t evt_id, const cs_ble_evt_t *evt)
{
    if (evt_id == CS_GAP_EVT_CONNECTED) {
        log_debug("CS_GAP_EVT_CONNECTED(%d): %d\n", evt->gap.conn_idx, evt->gap.connected.adv_idx);
        memcpy(&last_conn, &evt->gap.connected.peer_addr, sizeof(cs_gap_addr_t));
        app_set_state(APP_CONNECTED);
        drv_gpio_write(CS_GPIO0, BITMASK(PAD_LED_0), GPIO_LEVEL_LOW);
    } else if (evt_id == CS_GAP_EVT_DISCONNECTED) {
        log_debug("CS_GAP_EVT_DISCONNECTED(%d): 0x%02X\n", evt->gap.conn_idx, evt->gap.disconnected.reason);

        app_adv_evt_disconnected_ind_handler();
        app_set_state(APP_DISCONNECT);
        drv_gpio_write(CS_GPIO0, BITMASK(PAD_LED_0), GPIO_LEVEL_HIGH);
    } else if (evt_id == CS_GAP_EVT_ADV_STATE_CHANGED) {
        log_debug("CS_GAP_EVT_ADV_STATE_CHANGED(%d), reason:%d\n", evt->gap.adv_state_changed.adv_idx,
                  evt->gap.adv_state_changed.state);
    } else {
    }
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
app_adv_env_tag_t *app_get_adv_env(void)
{
    return &app_adv_env;
}

void app_adv_set_adv_data(uint8_t *adv_data, uint8_t len)
{
    if (adv_data != NULL && len <= APP_ADV_MAX_LEN) {
        memcpy(app_adv_env.adv_data.data, adv_data, len);
        app_adv_env.adv_data.len = len;
    }
}

void app_adv_set_rsp_data(uint8_t *rsp_data, uint8_t len)
{
    if (rsp_data != NULL && len <= APP_ADV_MAX_LEN) {
        memcpy(app_adv_env.rsp_data.data, rsp_data, len);
        app_adv_env.rsp_data.len = len;
    }
}

void app_adv_set_adv_config(cs_adv_param_t *config)
{
    if (config != NULL) {
        memcpy(&app_adv_env.config, config, sizeof(cs_adv_param_t));
    }
}

void app_adv_set_env(app_adv_env_tag_t *p_adv_env)
{
    if (p_adv_env != NULL) {
        if (p_adv_env != &app_adv_env) {
            memcpy(&app_adv_env, p_adv_env, sizeof(app_adv_env_tag_t));
        }
    }
}

/**
 *******************************************************************************
 * @brief  Init advertising module
 *******************************************************************************
 */

void app_adv_init(void)
{
    cs_event_callback_reg(app_adv_event_cb);
    app_adv_env.config.own_addr_type = CS_ADV_ADDR_TYPE_RANDOM;
    app_adv_env.config.prim_phy      = CS_ADV_PHY_1M;
    app_adv_env.config.secd_phy      = CS_ADV_PHY_1M;
    app_adv_env.config.tx_pwr        = 0;
    app_adv_env.config.filter_policy = CS_ADV_FILTER_NONE;
    app_adv_env.config.prim_ch_map   = CS_ADV_CH_ALL;
    app_adv_env.config.prim_intv_min = APP_ADV_INTERVAL;
    app_adv_env.config.prim_intv_max = APP_ADV_INTERVAL;

    uint8_t addr[7] = {CS_ADV_ADDR_TYPE_PUBLIC};
    cs_gap_addr_get(CS_ADV_ADDR_TYPE_PUBLIC, &addr[1]);
    if (!memcmp(&addr[1], "\x00\x00\x00\x00\x00\x00", 6)) {
        app_adv_env.config.local_addr = s_local_addr;
    } else {
        memcpy(s_local_addr, &addr[1], 6);
        app_adv_env.config.local_addr = s_local_addr;
    }
    // app_adv_env.config.adv_properties = CS_ADV_PROP_EXT_CONN_NONSCAN;
    // app_adv_env.config.adv_properties = CS_ADV_PROP_LEGACY_DIRECT_IND_HIGH;
    // app_adv_env.config.adv_properties = CS_ADV_PROP_EXT_NONCONN_SCAN;
    app_adv_env.config.adv_properties = CS_ADV_PROP_LEGACY_IND;

    memcpy(&app_adv_env.adv_data.data[0], s_adv_data, strlen((const char *)s_adv_data));
    app_adv_env.adv_data.len     = strlen((const char *)s_adv_data);
    app_adv_env.rsp_data.data[0] = sizeof(DEFAULT_DEVICE_NAME);
    app_adv_env.rsp_data.data[1] = 0x09;
    memcpy(&app_adv_env.rsp_data.data[2], DEFAULT_DEVICE_NAME, sizeof(DEFAULT_DEVICE_NAME) - 1);
    app_adv_env.rsp_data.len = sizeof(DEFAULT_DEVICE_NAME) + 1;
    // 启动ble广播功能
    cs_gap_adv_start(0, &app_adv_env.config, &app_adv_env.adv_data, &app_adv_env.rsp_data);
}

void app_adv_restart(void)
{
    cs_gap_adv_start(0, &app_adv_env.config, &app_adv_env.adv_data, &app_adv_env.rsp_data);
}

void app_set_state(uint8_t state)
{
    app_connect_state = state;
}

uint8_t app_get_state(void)
{
    return app_connect_state;
}

__WEAK void app_adv_evt_disconnected_ind_handler(void)
{
    cs_gap_adv_start(0, &app_adv_env.config, &app_adv_env.adv_data, &app_adv_env.rsp_data);
}

/** @} */
