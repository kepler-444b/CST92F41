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
 * @file     cs_ble_error.h
 * @date     27. May 2021
 * @author   chipsea
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @addtogroup CS_BLE_ERROR
 * @brief 错误代码
 */
/// @{

#ifndef __CS_BLE_ERR_H__
#define __CS_BLE_ERR_H__

/// 错误定义
typedef enum cs_ble_error {
    CS_BLE_ERROR_NO_ERR                  = 0x00, ///< 执行成功
    CS_BLE_ERROR_NO_CONNECTION,                  ///< 无连接错误
    CS_BLE_ERROR_INVALID_STATE,                  ///< 状态错误
    CS_BLE_ERROR_INSUFFICIENT_RESOURCES,         ///< 无可用资源错误
    CS_BLE_ERROR_NOT_SUPPORT,                    ///< 操作或请求不支持
    CS_BLE_ERROR_INVALID_PARAM,                  ///< 参数错误
} cs_ble_error_t;

/// GATT 错误定义
typedef enum cs_ble_gatt_error {
    CS_BLE_GATT_ERR_NO_ERROR                       = 0x0000,
    CS_BLE_GATT_ERR_INVALID_HANDLE                 = 0x0401,
    CS_BLE_GATT_ERR_READ_NOT_PERMITTED             = 0x0402,
    CS_BLE_GATT_ERR_WRITE_NOT_PERMITTED            = 0x0403,
    CS_BLE_GATT_ERR_INVALID_PDU                    = 0x0404,
    CS_BLE_GATT_ERR_INSUFFICIENT_AUTHENTICATION    = 0x0405,
    CS_BLE_GATT_ERR_REQUEST_NOT_SUPPORTED          = 0x0406,
    CS_BLE_GATT_ERR_INVALID_OFFSET                 = 0x0407,
    CS_BLE_GATT_ERR_INSUFFICIENT_AUTHORIZATION     = 0x0408,
    CS_BLE_GATT_ERR_PREPARE_QUEUE_FULL             = 0x0409,
    CS_BLE_GATT_ERR_ATTRIBUTE_NOT_FOUND            = 0x040A,
    CS_BLE_GATT_ERR_ATTRIBUTE_NOT_LONG             = 0x040B,
    CS_BLE_GATT_ERR_ENCRYPTION_KEY_SIZE_TOO_SHORT  = 0x040C,
    CS_BLE_GATT_ERR_INVALID_ATTRIBUTE_VALUE_LENGTH = 0x040D,
    CS_BLE_GATT_ERR_UNLIKELY_ERROR                 = 0x040E,
    CS_BLE_GATT_ERR_INSUFFICIENT_ENCRYPTION        = 0x040F,
    CS_BLE_GATT_ERR_UNSUPPORTED_GROUP_TYPE         = 0x0410,
    CS_BLE_GATT_ERR_INSUFFICIENT_RESOURCES         = 0x0411,
    CS_BLE_GATT_ERR_DATABASE_OUT_OF_SYNC           = 0x0412,
    CS_BLE_GATT_ERR_VALUE_NOT_ALLOWED              = 0x0413,
} cs_ble_gatt_error_t;

/// SMP 错误定义
typedef enum cs_ble_smp_error {
    CS_BLE_SMP_ERR_NO_ERR                                 = 0x0000,
    CS_BLE_SMP_LOCAL_ERR_PASSKEY_ENTRY_FAILED             = 0x0601,
    CS_BLE_SMP_LOCAL_ERR_OOB_NOT_AVAILABLE                = 0x0602,
    CS_BLE_SMP_LOCAL_ERR_AUTHENTICATION_REQUIREMENTS      = 0x0603,
    CS_BLE_SMP_LOCAL_ERR_CONFIRM_VALUE_FAILED             = 0x0604,
    CS_BLE_SMP_LOCAL_ERR_PAIRING_NOT_SUPPORTED            = 0x0605,
    CS_BLE_SMP_LOCAL_ERR_ENCRYPTION_KEY_SIZE              = 0x0606,
    CS_BLE_SMP_LOCAL_ERR_COMMAND_NOT_SUPPORTED            = 0x0607,
    CS_BLE_SMP_LOCAL_ERR_UNSPECIFIED_REASON               = 0x0608,
    CS_BLE_SMP_LOCAL_ERR_REPEATED_ATTEMPTS                = 0x0609,
    CS_BLE_SMP_LOCAL_ERR_INVALID_PARAMETERS               = 0x060A,
    CS_BLE_SMP_LOCAL_ERR_DHKEY_CHECK_FAILED               = 0x060B,
    CS_BLE_SMP_LOCAL_ERR_NUMERIC_COMPARISON_FAILED        = 0x060C,
    CS_BLE_SMP_LOCAL_ERR_BR_EDR_PAIRING_IN_PROGRESS       = 0x060D,
    CS_BLE_SMP_LOCAL_ERR_CROSS_TRANSPORT_KEY_NOT_ALLOWED  = 0x060E,
    CS_BLE_SMP_LOCAL_ERR_KEY_REJECTED                     = 0x060F,
    CS_BLE_SMP_REMOTE_ERR_PASSKEY_ENTRY_FAILED            = 0x0701,
    CS_BLE_SMP_REMOTE_ERR_OOB_NOT_AVAILABLE               = 0x0702,
    CS_BLE_SMP_REMOTE_ERR_AUTHENTICATION_REQUIREMENTS     = 0x0703,
    CS_BLE_SMP_REMOTE_ERR_CONFIRM_VALUE_FAILED            = 0x0704,
    CS_BLE_SMP_REMOTE_ERR_PAIRING_NOT_SUPPORTED           = 0x0705,
    CS_BLE_SMP_REMOTE_ERR_ENCRYPTION_KEY_SIZE             = 0x0706,
    CS_BLE_SMP_REMOTE_ERR_COMMAND_NOT_SUPPORTED           = 0x0707,
    CS_BLE_SMP_REMOTE_ERR_UNSPECIFIED_REASON              = 0x0708,
    CS_BLE_SMP_REMOTE_ERR_REPEATED_ATTEMPTS               = 0x0709,
    CS_BLE_SMP_REMOTE_ERR_INVALID_PARAMETERS              = 0x070A,
    CS_BLE_SMP_REMOTE_ERR_DHKEY_CHECK_FAILED              = 0x070B,
    CS_BLE_SMP_REMOTE_ERR_NUMERIC_COMPARISON_FAILED       = 0x070C,
    CS_BLE_SMP_REMOTE_ERR_BR_EDR_PAIRING_IN_PROGRESS      = 0x070D,
    CS_BLE_SMP_REMOTE_ERR_CROSS_TRANSPORT_KEY_NOT_ALLOWED = 0x070E,
    CS_BLE_SMP_REMOTE_ERR_KEY_REJECTED                    = 0x070F,
} cs_ble_smp_error_t;

#endif /* __CS_BLE_ERR_H__ */

/// @}
