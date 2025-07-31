/**
 ****************************************************************************************
 *
 * @file cs_ble_mem.h
 *
 * @brief API for the heap management module.
 *
 ****************************************************************************************
 */

#ifndef _CS_BLE_MEM_H_
#define _CS_BLE_MEM_H_

#include "sdk_config.h"     // IP configuration
#include <stdint.h>          // standard integer
#include <stdbool.h>         // standard includes
#include "cs.h"

/*******************************************************************************
 * DEFINES
 */

/*
 * TYPE
 ****************************************************************************************
 */

/// memory heaps types.
enum CS_MEM_HEAP
{
    /// Environment variables
    CS_MEM_SLOW,
    /// Messages
    CS_MEM_FAST,
    /// Non Retention
    CS_MEM_NRET,

    /// Host
    CS_MEM_HOST,

    /// MAX
    CS_MEM_BLOCK_MAX,
};

/**
 *******************************************************************************
 * @brief  cs mem init
 *******************************************************************************
 */
void cs_ble_mem_init(void);

/**
 *******************************************************************************
 * @brief  cs mem heap set
 *
 * @param[in] type  type
 * @param[in] heap  heap
 * @param[in] heap_size  heap size
 *******************************************************************************
 */
void cs_ble_mem_heap_set(uint8_t type, uint8_t* heap, uint16_t heap_size);

/**
 *******************************************************************************
 * @brief  cs mem malloc
 *
 * @param[in] size  size
 * @param[in] type  type
 *******************************************************************************
 */
void *cs_ble_mem_malloc(uint32_t size, uint8_t type);

/**
 *******************************************************************************
 * @brief  cs mem malloc check
 *
 * @param[in] size  size
 * @param[in] type  type
 *
 * @return
 *******************************************************************************
 */
bool cs_ble_mem_malloc_check(uint32_t size, uint8_t type);

/**
 *******************************************************************************
 * @brief  cs mem free
 *
 * @param[in] mem_ptr  mem ptr
 *******************************************************************************
 */
void cs_ble_mem_free(void *mem_ptr);

/**
 *******************************************************************************
 * @brief  cs mem is empty
 *
 * @param[in] type  type
 *
 * @return
 *******************************************************************************
 */
bool cs_ble_mem_is_empty(uint8_t type);

/**
 ****************************************************************************************
 * @brief Check if current pointer is free or not
 *
 * @param[in] mem_ptr pointer to a memory block
 *
 * @return true if already free, false else.
 ****************************************************************************************
 */
bool cs_ble_mem_is_freed(void* mem_ptr);

/**
 ****************************************************************************************
 * @brief Retrieve memory usage of selected heap.
 *
 * @param[in] type Type of memory heap block
 *
 * @return current memory usage of current heap.
 ****************************************************************************************
 */
uint16_t cs_ble_mem_get_cur_usage(uint8_t type);


/**
 ****************************************************************************************
 * @brief Retrieve max memory usage of all heap.
 * This command also resets max measured value.
 *
 * @return max memory usage of all heap.
 ****************************************************************************************
 */
uint32_t cs_ble_mem_get_max_usage(void);

#endif // _CS_BLE_MEM_H_

