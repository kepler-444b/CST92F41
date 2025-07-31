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
 * @file     cs_ble_gatt.h
 * @date     27. May 2021
 * @author   chipsea
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @addtogroup CS_BLE_GATT
 * @brief GATT
 */
/// @{

#ifndef __CS_BLE_GATT_H__
#define __CS_BLE_GATT_H__
#include <stdint.h>
#include "cs_ble_range.h"

/*******************************************************************************
 * DEFINES
 */

/// UUID 16bit 长度值
#define CS_UUID_16BIT    2
/// UUID 128bit 长度值
#define CS_UUID_128BIT  16

// GATT

/// CS_BLE_GATT_EVENTS
enum CS_BLE_GATT_EVENTS {
    /// Notification 或 Write Command发送完成事件，参考 @ref cs_gatt_evt_tx_complete_t
    CS_BLE_GATT_EVT_TX_COMPLETE              = CS_BLE_GATT_EVTS_BASE + 0,
    /// MTU 更新完成事件，参考 @ref cs_gatt_evt_mtu_exchanged_t
    CS_BLE_GATT_EVT_MTU_EXCHANGED            = CS_BLE_GATT_EVTS_BASE + 1,
    /// 请求超时事件，参考 @ref cs_gatt_evt_timeout_t
    CS_BLE_GATT_EVT_TIMEOUT                  = CS_BLE_GATT_EVTS_BASE + 2,
    /// Indication响应事件，参考 @ref cs_gatts_evt_indicate_cfm_t
    CS_GATTS_EVT_INDICATE_CFM            = CS_BLE_GATT_EVTS_BASE + 3,
    /// 读请求事件，参考 @ref cs_gatts_evt_read_req_t
    CS_GATTS_EVT_READ_REQ                = CS_BLE_GATT_EVTS_BASE + 4,
    /// 写请求事件，参考 @ref cs_gatts_evt_write_req_t
    CS_GATTS_EVT_WRITE_REQ               = CS_BLE_GATT_EVTS_BASE + 5,
    /// 服务发现完成事件，参考 @ref cs_gattc_evt_find_serv_rsp_t
    CS_GATTC_EVT_FIND_SERV_RSP           = CS_BLE_GATT_EVTS_BASE + 6,
    /// 服务发现完成事件，参考 @ref cs_gattc_evt_find_serv_by_uuid_rsp_t
    CS_GATTC_EVT_FIND_SERV_BY_UUID_RSP   = CS_BLE_GATT_EVTS_BASE + 7,
    /// 特征发现完成事件，参考 @ref cs_gattc_evt_find_char_rsp_t
    CS_GATTC_EVT_FIND_CHAR_RSP           = CS_BLE_GATT_EVTS_BASE + 8,
    /// 描述发现完成事件，参考 @ref cs_gattc_evt_find_desc_rsp_t
    CS_GATTC_EVT_FIND_DESC_RSP           = CS_BLE_GATT_EVTS_BASE + 9,
    /// 读完成事件，参考 @ref cs_gattc_evt_read_rsp_t
    CS_GATTC_EVT_READ_RSP                = CS_BLE_GATT_EVTS_BASE + 10,
    /// 读完成事件，参考 @ref cs_gattc_evt_read_by_uuid_rsp_t
    CS_GATTC_EVT_READ_BY_UUID_RSP        = CS_BLE_GATT_EVTS_BASE + 11,
    /// 写完成事件，参考 @ref cs_gattc_evt_write_rsp_t
    CS_GATTC_EVT_WRITE_RSP               = CS_BLE_GATT_EVTS_BASE + 12,
    /// notification或indication接收事件，参考 @ref cs_gattc_evt_hvx_ind_t
    CS_GATTC_EVT_HVX_IND                 = CS_BLE_GATT_EVTS_BASE + 13,
};

/// att属性
enum cs_att_prop {
    CS_ATT_PROP_IND       = 1 << 5,
    CS_ATT_PROP_NTF       = 1 << 4,
    CS_ATT_PROP_WRITE     = 1 << 3,
    CS_ATT_PROP_WRITE_CMD = 1 << 2,
    CS_ATT_PROP_READ      = 1 << 1,
};

/// att属性结构体
typedef struct {
    const uint8_t *uuid;    ///< ATT UUID
    uint8_t uuid_len;       ///< ATT UUID长度
    uint8_t att_prop;       ///< ATT 属性，参考@ref cs_att_prop
    uint8_t att_perm_read;  ///< ATT 读权限
    uint8_t att_perm_write; ///< ATT 写权限
} cs_gatt_item_t;

/// gatt服务结构体
typedef struct {
    const uint8_t *uuid;        ///< Service UUID
    uint16_t uuid_len;          ///< Service UUID长度
    uint16_t att_num;           ///< item数量
    const cs_gatt_item_t *item; ///< ATT属性数组
} cs_gatt_serv_t;

/// CS_BLE_GATT_HVX_TYPE
enum cs_gatt_hvx_type {
    CS_HANDLE_VALUE_NTF,  ///< notification 类型
    CS_HANDLE_VALUE_IND,  ///< indication 类型
};

/// GATT写类型
enum cs_gattc_write_type {
    CS_GATTC_WRITE_REQ,  ///< write request 类型
    CS_GATTC_WRITE_CMD,  ///< write no response 类型
};

/// gatt handle value notify/indicate结构体
typedef struct {
    uint8_t type;           ///< CS_HANDLE_VALUE_NTF or CS_HANDLE_VALUE_IND, ref @ref cs_gatt_hvx_type
    uint16_t att_hdl;       ///< att handle
    const uint8_t *data;    ///< 数据，长度不应大于当前MTU值
    int len;                ///< 数据长度
    uint16_t id;             ///< 与CS_GATT_EVT_TX_COMPLETE事件中的id值对应，用于发送完成的消息
} cs_gatts_hvx_t;

/// Event structure for @ref CS_BLE_GATT_EVT_TX_COMPLETE.
typedef struct {
    uint16_t id;             ///< Notification 或 Write Command消息已经发送完成时数据对应id
} cs_gatt_evt_tx_complete_t;

/// Event structure for @ref CS_BLE_GATT_EVT_MTU_EXCHANGED.
typedef struct {
    uint16_t mtu;           ///< 更新后的MTU值
} cs_gatt_evt_mtu_exchanged_t;

/// Event structure for @ref CS_BLE_GATT_EVT_TIMEOUT.
typedef struct {
    uint8_t att_opcode;     ///< 超时的指令
} cs_gatt_evt_timeout_t;

/// Event structure for @ref CS_GATTS_EVT_INDICATE_CFM.
typedef struct {
    uint16_t att_hdl;       ///< att handle
} cs_gatts_evt_indicate_cfm_t;

/// Event structure for @ref CS_GATTS_EVT_READ_REQ.
typedef struct {
    uint16_t att_hdl;       ///< att handle
    uint16_t offset;        ///< data offset
} cs_gatts_evt_read_req_t;

/// Event structure for @ref CS_GATTS_EVT_WRITE_REQ.
typedef struct {
    uint16_t att_hdl;       ///< att handle
    const uint8_t *data;    ///< data
    uint16_t len;           ///< data length
} cs_gatts_evt_write_req_t;

/// Gatt Service结构体
typedef struct {
    uint16_t start_hdl;     ///< start handle
    uint16_t end_hdl;       ///< end handle
    const uint8_t *uuid;    ///< service UUID
    uint8_t uuid_len;       ///< length of UUID
} cs_gatt_service_t;

/// Event structure for @ref CS_GATTC_EVT_FIND_SERV_RSP.
typedef struct {
    uint32_t status;                ///< 状态
    cs_gatt_service_t *service;     ///< gatt service 数组
    uint8_t service_num;            ///< gatt service 数量
} cs_gattc_evt_find_serv_rsp_t;

/// Event structure for @ref CS_GATTC_EVT_FIND_SERV_BY_UUID_RSP.
typedef struct {
    uint32_t status;                ///< 状态
    cs_gatt_service_t *service;     ///< gatt service 数组
    uint8_t service_num;            ///< gatt service 数量
} cs_gattc_evt_find_serv_by_uuid_rsp_t;

/// Gatt Service结构体
typedef struct {
    uint16_t value_hdl;     ///< value handle
    uint8_t properties;     ///< properties
    const uint8_t *uuid;    ///< char UUID
    uint8_t uuid_len;       ///< length of UUID
} cs_gatt_characteristic_t;

/// Event structure for @ref CS_GATTC_EVT_FIND_CHAR_RSP.
typedef struct {
    uint32_t status;                          ///< 状态
    cs_gatt_characteristic_t *characteristic; ///< gatt char 数组
    uint8_t char_num;                         ///< gatt char 数量
} cs_gattc_evt_find_char_rsp_t;

/// Gatt Service结构体
typedef struct {
    uint16_t att_hdl;       ///< att handle
    const uint8_t *uuid;    ///< char UUID
    uint8_t uuid_len;       ///< length of UUID
} cs_gatt_descriptor_t;

/// Event structure for @ref CS_GATTC_EVT_FIND_DESC_RSP.
typedef struct {
    uint32_t status;                    ///< 状态
    cs_gatt_descriptor_t *descriptor;   ///< gatt descriptor 数组
    uint8_t desc_num;                   ///< gatt descriptor 数量
} cs_gattc_evt_find_desc_rsp_t;

/// Event structure for @ref CS_GATTC_EVT_READ_RSP.
typedef struct {
    uint32_t status;        ///< 状态
    uint16_t att_hdl;       ///< att handle
    uint16_t offset;        ///< read offset
    const uint8_t *data;    ///< data
    uint16_t len;           ///< data length
} cs_gattc_evt_read_rsp_t;

/// Event structure for @ref CS_GATTC_EVT_READ_BY_UUID_RSP.
typedef struct {
    uint32_t status;        ///< 状态
    uint16_t att_hdl;       ///< att handle
    const uint8_t *data;    ///< data
    uint16_t len;           ///< data length
} cs_gattc_evt_read_by_uuid_rsp_t;

/// Event structure for @ref CS_GATTC_EVT_WRITE_RSP.
typedef struct {
    uint32_t status;        ///< 写入状态
} cs_gattc_evt_write_rsp_t;

/// Event structure for @ref CS_GATTC_EVT_HVX_IND.
typedef struct {
    uint8_t type;           ///< @ref cs_gatt_hvx_type
    uint16_t att_hdl;       ///< att handle
    const uint8_t *data;    ///< data
    uint16_t len;           ///< data length
} cs_gattc_evt_hvx_ind_t;

/// GATT 消息结构体
typedef struct {
    uint8_t conn_idx; ///< 连接 index
    union {
        cs_gatt_evt_tx_complete_t             tx_complete;            ///< Notify 或 Write Cmd发送完成事件参数
        cs_gatt_evt_mtu_exchanged_t           mtu_exchanged;          ///< MTU 更新完成事件参数
        cs_gatt_evt_timeout_t                 timeout;                ///< 请求超时事件参数
        cs_gatts_evt_indicate_cfm_t           indicate_cfm;           ///< Indication响应事件参数
        cs_gatts_evt_read_req_t               read_req;               ///< 读请求事件参数
        cs_gatts_evt_write_req_t              write_req;              ///< 写请求事件参数
        cs_gattc_evt_find_serv_rsp_t          find_serv_rsp;          ///< 服务发现完成事件参数
        cs_gattc_evt_find_serv_by_uuid_rsp_t  find_serv_by_uuid_rsp;  ///< 服务发现完成事件参数
        cs_gattc_evt_find_char_rsp_t          find_char_rsp;          ///< 特征发现完成事件参数
        cs_gattc_evt_find_desc_rsp_t          find_desc_rsp;          ///< 描述发现完成事件参数
        cs_gattc_evt_read_rsp_t               read_rsp;               ///< 读完成事件参数
        cs_gattc_evt_read_by_uuid_rsp_t       read_by_uuid_rsp;       ///< 读完成事件参数
        cs_gattc_evt_write_rsp_t              write_rsp;              ///< 写完成事件参数
        cs_gattc_evt_hvx_ind_t                hvx_ind;                ///< notification或indication接收事件参数
    };
} cs_ble_gatt_evt_t;

/// ATT Primary Service定义
extern const uint8_t ob_att_serv_def[2];
/// ATT Primary Service定义
extern const uint8_t ob_att_secs_def[2];
/// ATT Include Service定义
extern const uint8_t ob_att_incl_def[2];
/// ATT Characteristic定义
extern const uint8_t ob_att_char_def[2];
/// ATT Characteristic User Description Descriptor定义
extern const uint8_t ob_att_cudd_def[2];
/// ATT Client Characteristic Configuration Descriptor定义
extern const uint8_t ob_att_cccd_def[2];
/// ATT Report Reference Descriptor定义
extern const uint8_t ob_att_rrd_def[2];

/**@brief 添加服务
 * @param[in]  att_serv       服务内容
 * @param[in]  start_handle   起始handle值
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gatts_add_service(const cs_gatt_serv_t *att_serv, uint16_t *start_handle);

/**@brief 设置服务可见性
 * @param[in]  serv_hdl       服务起始handle
 * @param[in]  visible        服务是否可见
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gatts_set_service_visibility(uint16_t serv_hdl, uint8_t visible);

/**@brief 发送notification或indication
 * @param[in]  conn_idx       连接index
 * @param[in]  hvx            数据参数
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gatts_send_hvx(uint8_t conn_idx, const cs_gatts_hvx_t *hvx);

/**@brief 响应读请求
 * @param[in]  conn_idx       连接index
 * @param[in]  att_state      读状态，参考@ref cs_ble_gatt_error，若不为success，则data参数无用
 * @param[in]  data           数据
 * @param[in]  len            长度，若超出当前MTU则数据内容自动截断
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gatts_read_response(uint8_t conn_idx, uint16_t att_state, const uint8_t *data,
                                uint16_t len);

/**@brief 响应写请求
 * @param[in]  conn_idx       连接index
 * @param[in]  att_state      写状态，参考@ref cs_ble_gatt_error
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gatts_write_response(uint8_t conn_idx, uint16_t att_state);

/**@brief 设置gatt server暂停响应client请求
 * @param[in]  conn_idx       连接index
 * @param[in]  pause_state    pending状态:1或0
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gatts_set_pause(uint8_t conn_idx, uint8_t pause_state);

/**@brief 请求MTU交换
 * @param[in]  conn_idx       连接index
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gattc_mtu_req(uint8_t conn_idx);

/**@brief 服务搜索 - 通过handle范围
 * @param[in]  conn_idx       连接index
 * @param[in]  start_handle    handle 范围
 * @param[in]  end_handle    handle 范围
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gattc_find_service_by_handle(uint8_t conn_idx, uint16_t start_handle,
                                         uint16_t end_handle);

/**@brief 服务搜索 - 通过handle范围
 * @param[in]  conn_idx       连接index
 * @param[in]  start_handle   handle 范围
 * @param[in]  end_handle    handle 范围
 * @param[in]  uuid          UUID
 * @param[in]  uuid_len      UUID长度
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gattc_find_service_by_uuid(uint8_t conn_idx, uint16_t start_handle,
                                       uint16_t end_handle, const uint8_t *uuid, uint8_t uuid_len);

/**@brief 特征搜索
 * @param[in]  conn_idx       连接index
 * @param[in]  start_handle   handle 范围
 * @param[in]  end_handle    handle 范围
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gattc_find_characteristic(uint8_t conn_idx,
                                      uint16_t start_handle, uint16_t end_handle);

/**@brief 描述搜索
 * @param[in]  conn_idx       连接index
 * @param[in]  start_handle   handle 范围
 * @param[in]  end_handle    handle 范围
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gattc_find_descriptor(uint8_t conn_idx, uint16_t start_handle, uint16_t end_handle);

/**@brief 读取请求
 * @param[in]  conn_idx       连接index
 * @param[in]  att_hdl        att handle
 * @param[in]  offset         数据偏移
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gattc_read(uint8_t conn_idx, uint16_t att_hdl, uint16_t offset);

/**@brief 读取请求
 * @param[in]  conn_idx       连接index
 * @param[in]  start_handle   handle 范围
 * @param[in]  end_handle    handle 范围
 * @param[in]  uuid          UUID
 * @param[in]  uuid_len      UUID长度
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gattc_read_by_uuid(uint8_t conn_idx, uint16_t start_handle, uint16_t end_handle, const uint8_t *uuid,
                               uint8_t uuid_len);

/**@brief 写入请求
 * @param[in]  conn_idx       连接index
 * @param[in]  att_hdl        att handle
 * @param[in]  type           write类型，参考@ref cs_gattc_write_type
 * @param[in]  data           数据
 * @param[in]  len            长度，超出当前MTU则自动截断或用prepare write模式
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gattc_write(uint8_t conn_idx, uint16_t att_hdl, uint8_t type,
                        const uint8_t *data, int len);

/**@brief indicate响应
 * @param[in]  conn_idx       连接index
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gattc_indicate_cfm(uint8_t conn_idx);

#endif /* __CS_BLE_GATT_H__ */

/// @}
