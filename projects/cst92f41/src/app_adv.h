/**
 * @file app_adv.h
 * @addtogroup BLE_APP_COMMON_app_adv App Advertise Module
 * @brief app common security module
 * @ingroup BLE_APP_COMMON
 *
 * @{
 */
#ifndef __APP_ADV_H__
#define __APP_ADV_H__

#include "cs_ble.h"

/*
 * MACROS
 ****************************************************************************************
 */
/// the adv data max len
#define APP_ADV_MAX_LEN         (31)

/// Advertising channel map - 37, 38, 39
#define APP_ADV_CHMAP           (~(0x07))
/// advertising direct interval
#define APP_ADV_INTERVAL        0x80   /**<  dvertising interval (in units of 0.625 ms. This value corresponds to 100 ms.). */
#define APP_ADV_TIMEOUT         0      /**< The duration of the fast advertising period (in seconds). max 655, or 0 is no timeout*/

#define APP_ADV_NO_ERROR        0

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/// States of APP task
enum app_state
{
    /// Initialization state
    APP_DISCONNECT,

    /// Connected state
    APP_CONNECTED,

    /// Number of defined states.
    APP_STATE_MAX
};


/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */
/**@struct app_adv_env_tag
 * @brief the advertising env information */
typedef struct 
{
    /// adv data
    cs_data_t adv_data;
    /// scan response data
    cs_data_t rsp_data;
    /// Advertising config
    cs_adv_param_t config;
}app_adv_env_tag_t;

/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */
app_adv_env_tag_t* app_get_adv_env(void);

void app_adv_set_adv_data(uint8_t* adv_data, uint8_t len);

void app_adv_set_rsp_data(uint8_t* rsp_data, uint8_t len);

void app_adv_set_adv_config(cs_adv_param_t *config);

void app_adv_init(void);

void app_adv_restart(void);

void app_set_state(uint8_t state);

uint8_t app_get_state(void);

void app_adv_evt_disconnected_ind_handler(void);

#endif /* __APP_ADV_H__ */

/** @} */
