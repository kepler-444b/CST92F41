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
 * @file     ancs_client.h
 * @date     22. Nov. 2023
 * @author   chipsea
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __ANCS_CLIENT_H__
#define __ANCS_CLIENT_H__

/*********************************************************************
 * MACROS
 */
#define log_debug(...)

#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif

#include <stdio.h>
#include <stdint.h>
#include "ancs_common.h"
/// Max Size of APP_ID
#define ANCS_APPID_CAP     32
/// Max Size of message Title
#define ANCS_TITLE_CAP     32
/// Max Size of message subtitle
#define ANCS_SUBTITLE_CAP  4
/// Max Size of date
#define ANCS_DATE_CAP      16
/// Max Size of message
#define ANCS_MSG_CAP       100

/*********************************************************************
 * TYPEDEFS
 */
/// ancs_notify_t
typedef struct {
    /// @ref ancs_eventID_values
    uint8_t evt_id;
    /// @ref ancs_event_flags
    uint8_t evt_flags;
    /// @ref ancs_categoryID_values
    uint8_t cate_id;
    /// category count
    uint8_t cate_count;
    /// message UID
    uint32_t notify_uid;
} ancs_notify_t;

/// ancs_data_t
typedef struct {
    /// Notify UID
    uint32_t notify_uid;
    /// Received length of date
    uint8_t date_len;
    /// Date in string format
    uint8_t date[ANCS_DATE_CAP];
    /// Received length of app_id
    uint8_t app_id_len;
    /// App id
    uint8_t app_id[ANCS_APPID_CAP];
    /// Recevied length of title
    uint8_t title_len;
    /// Message Title
    uint8_t title[ANCS_TITLE_CAP];
    /// Recevied length of subtitle
    uint8_t subtitle_len;
    /// Message Subtitle
    uint8_t subtitle[ANCS_SUBTITLE_CAP];
    /// Received length of message
    uint8_t msg_len;
    /// Message detail
    uint8_t msg[ANCS_MSG_CAP];
} ancs_data_t;


/*******************************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief Init ancs client
 *******************************************************************************
 */
void ancs_client_init(void);
/**
 *******************************************************************************
 * @brief Get message detail data by UID
 * @param[in] uid        UID
 *******************************************************************************
 */
void ancs_get_data(uint32_t uid);
/**
 *******************************************************************************
 * @brief Proform action by UID & action
 * @param[in] uid       UID
 * @param[in] act       action
 *******************************************************************************
 */
void ancs_perform_act(uint32_t uid, uint8_t act);
/**
 *******************************************************************************
 * @brief Callback for process notify
 * @param[in] notify       ancs notification
 *******************************************************************************
 */
extern void ancs_notify_evt(ancs_notify_t *notify);
/**
 *******************************************************************************
 * @brief Callback for process message detail data
 * @param[in] data       ancs data
 *******************************************************************************
 */
extern void ancs_data_evt(ancs_data_t *data);

#endif /* __ANCS_CLIENT_H__ */

/** @} */
