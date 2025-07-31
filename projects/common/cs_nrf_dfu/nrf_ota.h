/**
 ****************************************************************************************
 *
 * @file nrf_ota.h
 *
 * @brief Header file - Service Server Role
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

#ifndef _NRF_OTA_H_
#define _NRF_OTA_H_

/**
 ****************************************************************************************
 * @addtogroup NRF_OTA 'Profile' Server
 * @ingroup NRF_OTA
 * @brief 'Profile' Server
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "rwip_config.h"
#include "prf_types.h"
#include "prf.h"

/*
 * DEFINES
 ****************************************************************************************
 */
 
/*********************************************************************
 * MACROS
 */

#define NRF_OTA_PKG_MAX_LEN       520
#define BLE_UUID_NRF_OTA_SERVICE  {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x30, 0x15, 0x00, 0x00}
#define NRF_OTA_CTRL_UUID     {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x31, 0x15, 0x00, 0x00}
#define NRF_OTA_PKG_UUID      {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x32, 0x15, 0x00, 0x00}
#define NRF_OTA_VERSION_UUID  {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, 0xDE, 0xEF, 0x12, 0x12, 0x34, 0x15, 0x00, 0x00}

///Maximum number of Server task instances
#define NRF_OTA_IDX_MAX     0x01

/*
 * ENUMERATIONS
 ****************************************************************************************
 */

/// NRF_OTA Service Attributes Indexes
enum
{
    NRF_OTA_IDX_SVC,

    NRF_OTA_IDX_CTRL_CHAR,
    NRF_OTA_IDX_CTRL_VAL,
    NRF_OTA_IDX_CTRL_CFG,

    NRF_OTA_IDX_PKG_CHAR,
    NRF_OTA_IDX_PKG_VAL,
    

    NRF_OTA_IDX_VERSION_CHAR,
    NRF_OTA_IDX_VERSION_VAL,

    NRF_OTA_IDX_NB,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// 'Profile' Server environment variable
struct nrf_ota_env_tag
{
    /// profile environment
    prf_env_t prf_env;
    /// On-going operation
    struct ke_msg * operation;
    /// Services Start Handle
    uint16_t start_hdl;
    /// NRF_OTA task state
    ke_state_t state[NRF_OTA_IDX_MAX];
    /// Notification configuration of peer devices.
    uint8_t ntf_cfg[BLE_CONNECTION_MAX];
};

/*
 * GLOBAL VARIABLE DECLARATIONS
 ****************************************************************************************
 */


/*
 * FUNCTION DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Retrieve service profile interface
 *
 * @return service profile interface
 ****************************************************************************************
 */
const struct prf_task_cbs* nrf_ota_prf_itf_get(void);

/**
 ****************************************************************************************
 * @brief Retrieve Attribute handle from service and attribute index
 *
 * @param[in] svc_idx NRF_OTA Service index
 * @param[in] att_idx Attribute index
 *
 * @return NRF_OTA attribute handle or INVALID HANDLE if nothing found
 ****************************************************************************************
 */
uint16_t nrf_ota_get_att_handle(uint8_t att_idx);

/**
 ****************************************************************************************
 * @brief Retrieve Service and attribute index form attribute handle
 *
 * @param[out] handle  Attribute handle
 * @param[out] svc_idx Service index
 * @param[out] att_idx Attribute index
 *
 * @return Success if attribute and service index found, else Application error
 ****************************************************************************************
 */
uint8_t nrf_ota_get_att_idx(uint16_t handle, uint8_t *att_idx);

/*
 * TASK DESCRIPTOR DECLARATIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * Initialize task handler
 *
 * @param task_desc Task descriptor to fill
 ****************************************************************************************
 */
void nrf_ota_task_init(struct ke_task_desc *task_desc);

/// @} NRF_OTA

#endif /* _NRF_OTA_H_ */
