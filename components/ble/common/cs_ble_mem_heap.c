/**
 * @file rwmem.c
 * @brief 
 * @date Thu, Jun 13, 2019 10:05:01 AM
 * @author liqiang
 *
 * @addtogroup 
 * @ingroup 
 * @details 
 *
 * @{
 */

/*********************************************************************
 * INCLUDES
 */
#include "sdk_config.h"

#include <string.h>
#include <stdio.h>

#include "cs_ble_mem.h"

/*********************************************************************
 * MACROS
 */

/// Fast heap size
#define CS_HEAP_FAST_SIZE  (   1000*CS_LE_OBSERVER \
                             + 1 * (  (16 + (CONFIG_LE_ACTIVITY_NB-1) * 56) \
                                    + (58 + (CONFIG_LE_ACTIVITY_NB-1) * 26) \
                                    + (CONFIG_LE_ACTIVITY_NB * 66) \
                                    + (CONFIG_LE_LL_HCI_CMD_PKTS_NB * 255) \
                                    + (CS_LE_OBSERVER * CS_LE_MAX_NB_ADV_REP_FRAG * 255 + CONFIG_LE_ACTIVITY_NB * 100) \
                                    + (CONFIG_LE_ACTIVITY_NB * 12) \
                                   ) )

/// Slow heap size
#define CS_HEAP_SLOW_SIZE       (330*CS_LE_CENTRAL + 270*CS_LE_OBSERVER + CONFIG_LE_ACTIVITY_NB*230)

/// Size of non-retention heap
#define CS_HEAP_NRET_SIZE    ((1980 + 336) * CS_LE_LL_SEC_CON)

/// Host heap size
#define CS_HEAP_HOST_SIZE      (480 \
                             + (360 + CS_LE_HOST_ATT_MTU + CS_LE_HOST_ATT_WRITE_CACHE_SIZE) * CS_LE_HOST_CONNECTION_NB \
                             + (132 * CS_LE_HOST_CONNECTION_NB * CS_LE_HOST_SC_PAIRING) \
                             + ( 20 * CS_LE_HOST_ADV_SET_NUM) \
                             + (  4 * CS_LE_HOST_MAX_GATT_SERV_NUM) \
                             + (CS_LE_HOST_MSG_SIZE))

/// ceil(len/4) + header_size
#define CS_HEAP_LEN_ALIGN(len)  ((((len)+3)/4) + (12/4))

/*********************************************************************
 * TYPEDEFS
 */


/*********************************************************************
 * CONSTANTS
 */


/*********************************************************************
 * LOCAL VARIABLES
 */

/// Memory allocated for environment variables
static uint32_t cs_heap_slow_env[CS_HEAP_LEN_ALIGN(CS_HEAP_SLOW_SIZE)];
/// Memory allocated for messages
static uint32_t cs_heap_fast_env[CS_HEAP_LEN_ALIGN(CS_HEAP_FAST_SIZE)];
/// Non Retention memory block
static uint32_t cs_heap_nret_env[CS_HEAP_LEN_ALIGN(CS_HEAP_NRET_SIZE)];

#ifdef CONFIG_BLE_HOST
/// Memory allocated for ble host
static uint32_t cs_heap_host_env[CS_HEAP_LEN_ALIGN(CS_HEAP_HOST_SIZE)];
#endif

/*********************************************************************
 * GLOBAL VARIABLES
 */


/*********************************************************************
 * LOCAL FUNCTIONS
 */


/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 *******************************************************************************
 * @brief  cs mem heap init
 *******************************************************************************
 */
void cs_ble_mem_heap_init(void)
{
    // Initialize memory heap used by kernel.
    cs_ble_mem_init();
    // slow alloc/free heap
    cs_ble_mem_heap_set(CS_MEM_SLOW, (uint8_t*)cs_heap_slow_env, sizeof(cs_heap_slow_env));
    // fast alloc/free heap
    cs_ble_mem_heap_set(CS_MEM_FAST, (uint8_t*)cs_heap_fast_env, sizeof(cs_heap_fast_env));
    // Non Retention memory block
    cs_ble_mem_heap_set(CS_MEM_NRET, (uint8_t*)cs_heap_nret_env, sizeof(cs_heap_nret_env));

#ifdef CONFIG_BLE_HOST
    // Memory allocated for ble host
    cs_ble_mem_heap_set(CS_MEM_HOST, (uint8_t*)cs_heap_host_env, sizeof(cs_heap_host_env));
#endif
}

/** @} */

