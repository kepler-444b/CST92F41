/**
 ****************************************************************************************
 *
 * @file nrf_ota.c
 *
 * @brief Server Implementation.
 *
 * Copyright (C) RivieraWaves 2009-2016
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup NRF_OTA
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"

#if BLE_APP_NRF_OTA
#include "nrf_ota.h"
#include "nrf_ota_task.h"
#include "prf_utils.h"
#include "prf.h"

#include "ke_mem.h"

/*
 * NRF_OTA ATTRIBUTES DEFINITION
 ****************************************************************************************
 */
const uint8_t nrf_ota_service_uuid[ATT_UUID_128_LEN] = BLE_UUID_NRF_OTA_SERVICE;
/// Full NRF_OTA Database Description - Used to add attributes into the database
const struct attm_desc_128 nrf_ota_att_db[] =
{
    // Service Declaration
    [NRF_OTA_IDX_SVC]            =   {ATT_16_TO_128_ARRAY(ATT_DECL_PRIMARY_SERVICE),  PERM(RD, ENABLE), 0, 0},

    // Characteristic Declaration
    [NRF_OTA_IDX_CTRL_CHAR]      =   {ATT_16_TO_128_ARRAY(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    // Characteristic Value
    [NRF_OTA_IDX_CTRL_VAL]       =   {NRF_OTA_CTRL_UUID,    PERM(WRITE_REQ, ENABLE) | PERM(NTF, ENABLE), PERM(UUID_LEN, UUID_128), NRF_OTA_PKG_MAX_LEN},
    // Characteristic - Client Characteristic Configuration Descriptor
    [NRF_OTA_IDX_CTRL_CFG]       =   {ATT_16_TO_128_ARRAY(ATT_DESC_CLIENT_CHAR_CFG), PERM(RD, ENABLE) | PERM(WRITE_REQ, ENABLE), 0, 0},

    // Characteristic Declaration
    [NRF_OTA_IDX_PKG_CHAR]       =   {ATT_16_TO_128_ARRAY(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    // Characteristic Value
    [NRF_OTA_IDX_PKG_VAL]        =   {NRF_OTA_PKG_UUID,    PERM(WRITE_COMMAND, ENABLE), PERM(UUID_LEN, UUID_128), NRF_OTA_PKG_MAX_LEN},

    // Characteristic Declaration
    [NRF_OTA_IDX_VERSION_CHAR]   =   {ATT_16_TO_128_ARRAY(ATT_DECL_CHARACTERISTIC),   PERM(RD, ENABLE), 0, 0},
    // Characteristic Value
    [NRF_OTA_IDX_VERSION_VAL]    =   {NRF_OTA_VERSION_UUID,    PERM(RD, ENABLE) , PERM(RI, ENABLE)|PERM(UUID_LEN, UUID_128), 2},

};

/*
 * LOCAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @brief Initialization of the NRF_OTA module.
 * This function performs all the initializations of the Profile module.
 *  - Creation of database (if it's a service)
 *  - Allocation of profile required memory
 *  - Initialization of task descriptor to register application
 *      - Task State array
 *      - Number of tasks
 *      - Default task handler
 *
 * @param[out]    env        Collector or Service allocated environment data.
 * @param[in|out] start_hdl  Service start handle (0 - dynamically allocated), only applies for services.
 * @param[in]     app_task   Application task number.
 * @param[in]     sec_lvl    Security level (AUTH, EKS and MI field of @see enum attm_value_perm_mask)
 * @param[in]     param      Configuration parameters of profile collector or service (32 bits aligned)
 *
 * @return status code to know if profile initialization succeed or not.
 ****************************************************************************************
 */
static uint8_t nrf_ota_init (struct prf_task_env* env, uint16_t* start_hdl, uint16_t app_task, uint8_t sec_lvl,  void* params)
{
    uint16_t shdl;
    struct nrf_ota_env_tag* nrf_ota_env = NULL;
    // Status
    uint8_t status = GAP_ERR_NO_ERROR;

    //-------------------- allocate memory required for the profile  ---------------------
    nrf_ota_env = (struct nrf_ota_env_tag* ) ke_malloc(sizeof(struct nrf_ota_env_tag), KE_MEM_ATT_DB);
    memset(nrf_ota_env, 0 , sizeof(struct nrf_ota_env_tag));

    shdl = *start_hdl;

    //Create NRF_OTA in the DB
    //------------------ create the attribute database for the profile -------------------
    status = attm_svc_create_db_128(&shdl, nrf_ota_service_uuid, NULL,
            NRF_OTA_IDX_NB, NULL, env->task, nrf_ota_att_db,
            ((sec_lvl & (PERM_MASK_SVC_DIS | PERM_MASK_SVC_AUTH | PERM_MASK_SVC_EKS))| PERM(SVC_MI, DISABLE) | PERM(SVC_UUID_LEN, UUID_128)));

    //-------------------- Update profile task information  ---------------------
    if (status == ATT_ERR_NO_ERROR)
    {
        // allocate NRF_OTA required environment variable
        env->env = (prf_env_t*) nrf_ota_env;
        *start_hdl = shdl;
        nrf_ota_env->start_hdl = *start_hdl;
        nrf_ota_env->prf_env.app_task = app_task
                | (PERM_GET(sec_lvl, SVC_MI) ? PERM(PRF_MI, ENABLE) : PERM(PRF_MI, DISABLE));
        nrf_ota_env->prf_env.prf_task = env->task | PERM(PRF_MI, DISABLE);

        // initialize environment variable
        env->id                     = TASK_ID_NRF_OTA;
        nrf_ota_task_init(&(env->desc));
    }
    else if(nrf_ota_env != NULL)
    {
        ke_free(nrf_ota_env);
    }

    return (status);
}

/**
 ****************************************************************************************
 * @brief Destruction of the NRF_OTA module - due to a reset for instance.
 * This function clean-up allocated memory (attribute database is destroyed by another
 * procedure)
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 ****************************************************************************************
 */
static void nrf_ota_destroy(struct prf_task_env* env)
{
    struct nrf_ota_env_tag* nrf_ota_env = (struct nrf_ota_env_tag*) env->env;

    // clear on-going operation
    if(nrf_ota_env->operation != NULL)
    {
        ke_free(nrf_ota_env->operation);
    }

    // free profile environment variables
    env->env = NULL;
    ke_free(nrf_ota_env);
}

/**
 ****************************************************************************************
 * @brief Handles Connection creation
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 ****************************************************************************************
 */
static void nrf_ota_create(struct prf_task_env* env, uint8_t conidx)
{
    struct nrf_ota_env_tag* nrf_ota_env = (struct nrf_ota_env_tag*) env->env;
    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);

    // force notification config to zero when peer device is connected
    nrf_ota_env->ntf_cfg[conidx] = 0;
}

/**
 ****************************************************************************************
 * @brief Handles Disconnection
 *
 * @param[in|out]    env        Collector or Service allocated environment data.
 * @param[in]        conidx     Connection index
 * @param[in]        reason     Detach reason
 ****************************************************************************************
 */
static void nrf_ota_cleanup(struct prf_task_env* env, uint8_t conidx, uint8_t reason)
{
    struct nrf_ota_env_tag* nrf_ota_env = (struct nrf_ota_env_tag*) env->env;

    ASSERT_ERR(conidx < BLE_CONNECTION_MAX);
    // force notification config to zero when peer device is disconnected
    nrf_ota_env->ntf_cfg[conidx] = 0;
}

/*
 * GLOBAL VARIABLE DEFINITIONS
 ****************************************************************************************
 */

/// NRF_OTA Task interface required by profile manager
const struct prf_task_cbs nrf_ota_itf =
{
        (prf_init_fnct) nrf_ota_init,
        nrf_ota_destroy,
        nrf_ota_create,
        nrf_ota_cleanup,
};


/*
 * GLOBAL FUNCTIONS DEFINITIONS
 ****************************************************************************************
 */

const struct prf_task_cbs* nrf_ota_prf_itf_get(void)
{
   return &nrf_ota_itf;
}

uint16_t nrf_ota_get_att_handle(uint8_t att_idx)
{
    struct nrf_ota_env_tag* nrf_ota_env = PRF_ENV_GET(NRF_OTA, nrf_ota);
    uint16_t handle = nrf_ota_env->start_hdl + att_idx;
    return handle;
}

uint8_t nrf_ota_get_att_idx(uint16_t handle, uint8_t *att_idx)
{
    struct nrf_ota_env_tag* nrf_ota_env = PRF_ENV_GET(NRF_OTA, nrf_ota);
    *att_idx = handle - nrf_ota_env->start_hdl;
    return ATT_ERR_NO_ERROR;
}

#endif //BLE_APP_NRF_OTA

/// @} NRF_OTA
