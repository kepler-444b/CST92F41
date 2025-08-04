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
 * @file     evt.h
 * @brief    event
 * @date     01. April 2020
 * @author   chipsea
 *
 * @defgroup EVT Event
 * @ingroup  COMPONENTS
 * @brief    event
 * @details  event
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __EVT_H
#define __EVT_H

/*******************************************************************************
 * INCLUDES
 */
#include "features.h"
#include "evt_timer.h"

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * MACROS
 */

#define CONFIG_EVT_RTOS_SUPPORT

/*******************************************************************************
 * TYPEDEFS
 */
/// Format of an event callback function
typedef void (*evt_callback_t)(void);

/// event type
typedef enum {
    EVT_TYPE_RESERVE, ///< Reserved
    EVT_TYPE_BLE,     ///< Event type for BLE
    EVT_TYPE_TIMER,   ///< Event type for Timer
    EVT_TYPE_SHELL,   ///< Event type for Shell

    EVT_TYPE_USR_FIRST = 16, ///< Event type user first
    EVT_TYPE_USR_LAST  = 31, ///< Event type user last

    EVT_TYPE_NUM, ///< Event number
} evt_type_t;

/*******************************************************************************
 * EXTERN FUNCTIONS
 */

/**
 ****************************************************************************************
 * @brief Initialize Kernel event module.
 ****************************************************************************************
 */
void evt_init(void);

/**
 ****************************************************************************************
 * @brief Register an event callback.
 *
 * @param[in]  evt_type       Event type.
 * @param[in]  callback         Pointer to callback function.
 ****************************************************************************************
 */
void evt_callback_set(evt_type_t evt_type, evt_callback_t callback);

/**
 ****************************************************************************************
 * @brief Get an event callback.
 *
 * @param[in]  evt_type       Event type.
 *
 * @return                      callback
 ****************************************************************************************
 */
evt_callback_t evt_callback_get(evt_type_t evt_type);

#ifdef CONFIG_EVT_RTOS_SUPPORT
/**
 ****************************************************************************************
 * @brief Register an event set callback.
 *
 * @param[in]  callback         Pointer to callback function.
 ****************************************************************************************
 */
void evt_schedule_trigger_callback_set(evt_callback_t callback);
#endif

/**
 ****************************************************************************************
 * @brief Set an event
 *
 * This primitive sets one event. It will trigger the call to the corresponding event
 * handler in the next scheduling call.
 *
 * @param[in]  evt_type      Event to be set.
 ****************************************************************************************
 */
void evt_set(evt_type_t evt_type);

/**
 ****************************************************************************************
 * @brief Clear an event
 *
 * @param[in]  evt_type      Event to be cleared.
 ****************************************************************************************
 */
void evt_clear(evt_type_t evt_type);

/**
 ****************************************************************************************
 * @brief Get the status of an event
 *
 * @param[in]  evt_type      Event to get.
 *
 * @return                     Event status (0: not set / 1: set)
 ****************************************************************************************
 */
uint8_t evt_get(evt_type_t evt_type);

/**
 ****************************************************************************************
 * @brief Get all event status
 *
 * @return                     Events bit field
 ****************************************************************************************
 */
uint32_t evt_get_all(void);

/**
 ****************************************************************************************
 * @brief Flush all pending events.
 ****************************************************************************************
 */
void evt_flush(void);

/**
 ****************************************************************************************
 * @brief Event scheduler entry point.
 *
 * This primitive is the entry point of Kernel event scheduling.
 ****************************************************************************************
 */
void evt_schedule(void);

/**
 ****************************************************************************************
 * @brief Event scheduler once entry point.
 *
 * This primitive is the entry point of Kernel event scheduling.
 ****************************************************************************************
 */
uint32_t evt_schedule_once(void);

#ifdef __cplusplus
}
#endif

#endif /* __EVT_H */

/** @} */
