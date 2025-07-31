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
 * @file     cs_ble_gap.h
 * @date     27. May 2021
 * @author   chipsea
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @addtogroup CS_BLE_GAP GAP接口
 * @brief GAP
 * @details GAP相关接口主要控制蓝牙的广播、扫描、连接以及配对加密功能
 */
/// @{

#ifndef __CS_BLE_GAP_H__
#define __CS_BLE_GAP_H__
#include <stdint.h>
#include <stdbool.h>
#include "cs_ble_range.h"

/// CS_GAP_EVENTS
enum CS_GAP_EVENTS {
    /// 连接成功事件，参考 @ref cs_gap_evt_connected_t
    CS_GAP_EVT_CONNECTED              = CS_GAP_EVTS_BASE + 0x00,
    /// 连接断开事件，参考 @ref cs_gap_evt_disconnected_t
    CS_GAP_EVT_DISCONNECTED           = CS_GAP_EVTS_BASE + 0x01,
    /// 广播状态变化事件，参考 @ref cs_gap_evt_adv_state_changed_t
    CS_GAP_EVT_ADV_STATE_CHANGED      = CS_GAP_EVTS_BASE + 0x02,
    /// 参数更新事件，参考 @ref cs_gap_evt_conn_params_update_t
    CS_GAP_EVT_CONN_PARAMS_UPDATE     = CS_GAP_EVTS_BASE + 0x03,
    /// peripherial参数更新请求，参考 @ref cs_gap_evt_conn_params_request_t
    CS_GAP_EVT_CONN_PARAMS_REQUEST    = CS_GAP_EVTS_BASE + 0x04,
    /// 广播上报事件，参考 @ref cs_gap_evt_adv_report_t
    CS_GAP_EVT_ADV_REPORT             = CS_GAP_EVTS_BASE + 0x05,
    /// 链路加密状态改变事件，参考 @ref cs_gap_evt_encrypt_t
    CS_GAP_EVT_ENCRYPT                = CS_GAP_EVTS_BASE + 0x06,
    /// 加密请求事件，仅central角色支持，参考 @ref cs_gap_evt_sec_request_t
    CS_GAP_EVT_SEC_REQUEST            = CS_GAP_EVTS_BASE + 0x10,
    /// 配对请求事件，仅peripheral角色支持，参考 @ref cs_gap_evt_pairing_request_t
    CS_GAP_EVT_PAIRING_REQUEST        = CS_GAP_EVTS_BASE + 0x11,
    /// 配对PIN码请求事件，参考 @ref cs_gap_evt_pin_request_t
    CS_GAP_EVT_PIN_REQUEST            = CS_GAP_EVTS_BASE + 0x12,
    /// LTK请求，仅peripheral角色支持，参考 @ref cs_gap_evt_ltk_request_t
    CS_GAP_EVT_LTK_REQUEST            = CS_GAP_EVTS_BASE + 0x13,
    /// 绑定信息请求，参考 @ref cs_gap_evt_bond_info_request_t
    CS_GAP_EVT_BOND_INFO_REQUEST      = CS_GAP_EVTS_BASE + 0x14,
    /// 配对完成事件，参考 @ref cs_gap_evt_bonded_t
    CS_GAP_EVT_BONDED                 = CS_GAP_EVTS_BASE + 0x15,
    /// 配对信息上报事件（LTK/IRK），参考 @ref cs_gap_evt_bond_info_t
    CS_GAP_EVT_BOND_INFO              = CS_GAP_EVTS_BASE + 0x16,
    /// 超时事件（scan），参考 @ref cs_gap_evt_timeout_t
    CS_GAP_EVT_TIMEOUT                = CS_GAP_EVTS_BASE + 0x20,
    /// 功率变化事件，参考 @ref cs_gap_evt_power_changed_t
    CS_GAP_EVT_POWER_CHANGED          = CS_GAP_EVTS_BASE + 0x21,
    /// PHY变化事件，参考 @ref cs_gap_evt_phy_update_t
    CS_GAP_EVT_PHY_UPDATE             = CS_GAP_EVTS_BASE + 0x22,
    /// Data Length变化事件，参考 @ref cs_gap_evt_data_len_changed_t
    CS_GAP_EVT_DATA_LEN_CHANGED       = CS_GAP_EVTS_BASE + 0x23,
    /// 对端设备信息，参考 @ref cs_gap_evt_device_info_t
    CS_GAP_EVT_DEVICE_INFO            = CS_GAP_EVTS_BASE + 0x24,
    /// 收到Scan request事件，参考 @ref cs_gap_evt_scan_req_recv_t
    CS_GAP_EVT_SCAN_REQ_RECV          = CS_GAP_EVTS_BASE + 0x25,
    /// HCI指令错误，参考 @ref cs_gap_evt_hci_error_t
    CS_GAP_EVT_HCI_ERROR              = CS_GAP_EVTS_BASE + 0xFF,
};

/// @brief adv广播属性bit位
/// @details
/// 若使能了bit位CS_ADV_PROP_BIT4_LEGACY_ADV，则使用传统广播，广播数据长度最大为31字节，广播属性仅支持如下组合:\n
/// CS_ADV_PROP_BIT4_LEGACY_ADV | CS_ADV_PROP_BIT1_SCANNABLE | CS_ADV_PROP_BIT0_CONNECTALBE\n
/// CS_ADV_PROP_BIT4_LEGACY_ADV | CS_ADV_PROP_BIT2_DIRECTED_ADV | CS_ADV_PROP_BIT0_CONNECTALBE\n
/// CS_ADV_PROP_BIT4_LEGACY_ADV | CS_ADV_PROP_BIT3_HIGH_DUTY_CYCLE | CS_ADV_PROP_BIT0_CONNECTALBE\n
/// CS_ADV_PROP_BIT4_LEGACY_ADV | CS_ADV_PROP_BIT1_SCANNABLE\n
/// 若未使能bit位CS_ADV_PROP_BIT4_LEGACY_ADV，则使用扩展广播，广播数据长度最大为251字节\n
enum cs_adv_properties_bits {
    CS_ADV_PROP_BIT0_CONNECTALBE      = 1 << 0, ///< 可连接
    CS_ADV_PROP_BIT1_SCANNABLE        = 1 << 1, ///< 响应scan request
    CS_ADV_PROP_BIT2_DIRECTED_ADV     = 1 << 2, ///< 直接广播
    CS_ADV_PROP_BIT3_HIGH_DUTY_CYCLE  = 1 << 3, ///< 高速直接广播
    CS_ADV_PROP_BIT4_LEGACY_ADV       = 1 << 4, ///< 传统广播
    CS_ADV_PROP_BIT5_OMIT_ADV_ADDRESS = 1 << 5, ///< 匿名广播
    CS_ADV_PROP_BIT6_INCLUDE_TX_POWER = 1 << 6, ///< 包含Tx Power
};

/// 预定义的adv广播属性
enum cs_adv_properties {
    CS_ADV_PROP_LEGACY_IND               = 0x13, ///< 传统广播
    CS_ADV_PROP_LEGACY_DIRECT_IND_LOW    = 0x15, ///< 低速直连传统广播，广播间隔由参数定义
    CS_ADV_PROP_LEGACY_DIRECT_IND_HIGH   = 0x1D, ///< 高速直连传统广播，广播间隔由controller控制（小于3.75ms）
    CS_ADV_PROP_LEGACY_SCAN_IND          = 0x12, ///< 可扫描不可连接传统广播
    CS_ADV_PROP_LEGACY_NONCONN_IND       = 0x10, ///< 不可连接不可扫描传统广播
    CS_ADV_PROP_EXT_CONN_NONSCAN         = 0x01, ///< 可连接不可扫描扩展广播
    CS_ADV_PROP_EXT_NONCONN_SCAN         = 0x02, ///< 不可连接可扫描扩展广播
    CS_ADV_PROP_EXT_NONCONN_NONSCAN      = 0x00, ///< 不可连接不可扫描扩展广播
    CS_ADV_PROP_EXT_ANONYMOUS            = 0x20, ///< 匿名扩展广播（忽略广播地址）
};

/// 广播白名单设置
enum cs_adv_filter_policy {
    CS_ADV_FILTER_NONE,          ///< 不启用白名单功能
    CS_ADV_FILTER_SCAN,          ///< 响应所有设备的连接请求，但是只响应白名单中设备的scan request请求
    CS_ADV_FILTER_CONN,          ///< 响应所有设备的scan request请求，但是只响应白名单中设备的连接请求
    CS_ADV_FILTER_CONN_SCAN,     ///< 只响应白名单中设备的scan request请求和连接请求
};

/// 连接白名单设置
enum cs_conn_filter_policy {
    CS_CONN_FILTER_NOT_USE,      ///< 创建连接时不启用白名单功能
    CS_CONN_FILTER_USE,          ///< 创建连接时启用白名单功能，地址参数无效，只连接白名单中的地址
};

/// 扫描白名单设置
enum cs_scan_filter_policy {
    CS_SCAN_FILTER_BASIC_UNFILTER,
    CS_SCAN_FILTER_BASIC_FILTER,
    CS_SCAN_FILTER_EXT_UNFILTER,
    CS_SCAN_FILTER_EXT_FILTER,
};

/// 广播信道
enum cs_adv_channel {
    CS_ADV_CH_37  = 0x01,
    CS_ADV_CH_38  = 0x02,
    CS_ADV_CH_39  = 0x04,
    CS_ADV_CH_ALL = 0x07,
};

/// 广播地址类型
enum cs_adv_addr_type {
    CS_ADV_ADDR_TYPE_PUBLIC,
    CS_ADV_ADDR_TYPE_RANDOM,
};

/// 广播PHY
enum cs_adv_phy {
    CS_ADV_PHY_1M = 0x01,
    CS_ADV_PHY_2M = 0x02,
    CS_ADV_PHY_CODED = 0x03,
};

/// 绑定信息类型
enum cs_bond_info_type {
    CS_BOND_INFO_LTK,
    CS_BOND_INFO_IRK,
};

/// 配对IO能力类型
enum cs_smp_io_capability {
    CS_BLE_SMP_IOCAP_DIS_ONLY, // DisplayOnly
    CS_BLE_SMP_IOCAP_DISP_YN,  // DisplayYesNo
    CS_BLE_SMP_IOCAP_KBD_ONLY, // KeyboardOnly
    CS_BLE_SMP_IOCAP_NONE,     // NoInputNoOutput
    CS_BLE_SMP_IOCAP_KDB_DISP, // KeyboardDisplay
};

/// 配对pin 类型
enum cs_smp_pin_type {
    CS_BLE_SMP_PIN_TYPE_NONE,     // 无显示,无输入
    CS_BLE_SMP_PIN_TYPE_OOB_REQ,  // 请求oob输入
    CS_BLE_SMP_PIN_TYPE_DIS_YN,   // 显示pin码,并请求确认
    CS_BLE_SMP_PIN_TYPE_PK_REQ,   // 请求pin输入
    CS_BLE_SMP_PIN_TYPE_DIS,      // 显示pin码
};

/// 连接角色
enum cs_gap_role {
    CS_GAP_ROLE_CENTRAL,      ///< central
    CS_GAP_ROLE_PHERIPHERAL,  ///< peripheral
};

/// 分发密钥类型
enum cs_smp_distribution {
    CS_BLE_SMP_DIST_BIT_ENC_KEY = 0x01, ///< EncKey
    CS_BLE_SMP_DIST_BIT_ID_KEY  = 0x02, ///< IdKey
};

/// 超时原因
enum cs_gap_timeout_source_t {
    CS_GAP_TOUT_SCAN, ///< 扫描超时
    CS_GAP_TOUT_CONN_PARAM_UPDATE, ///< 参数更新超时
};

/// 设备信息类型
enum cs_dev_info_type {
    CS_DEV_INFO_PEER_VERSION,      ///< 对端设备版本信息
    CS_DEV_INFO_PEER_LE_FEATURE,   ///< 对端设备feature信息
    CS_DEV_INFO_LOCAL_VERSION,     ///< 当前设备版本信息
    CS_DEV_INFO_LOCAL_LE_FEATURE,  ///< 当前设备feature信息
    CS_DEV_INFO_RSSI,              ///< 指定连接的RSSI
    CS_DEV_INFO_CHANNEL_MAP,       ///< 指定连接的ChannelMap信息
};

/// PHY类型
enum cs_gap_phy {
    CS_GAP_PHY_1M = 0x01,
    CS_GAP_PHY_2M = 0x02,
    CS_GAP_PHY_CODED = 0x03,
};

#define CS_GAP_ADDR_LEN   6   ///< 地址长度
#define CS_GAP_RANDOM_LEN 8   ///< random值长度
#define CS_GAP_KEY_LEN    16  ///< key长度
#define CS_GAP_ADV_TX_POWER_NO_AVA 0x7F ///< 发射功率不可用

/// GAP地址
typedef struct cs_gap_addr {
    uint8_t addr_type;              ///< See @ref cs_adv_addr_type.
    uint8_t addr[CS_GAP_ADDR_LEN];  ///< 48-bit address, LSB format.
} cs_gap_addr_t;

/// 广播参数
typedef struct cs_adv_param {
    uint32_t prim_intv_min;   ///< unit of 0.625ms
    uint32_t prim_intv_max;   ///< unit of 0.625ms
    uint16_t adv_properties;  ///< @ref cs_adv_properties
    uint8_t own_addr_type;    ///< @ref cs_adv_addr_type
    uint8_t prim_ch_map;      ///< @ref cs_adv_channel
    cs_gap_addr_t *peer_addr; ///< peer address，若启用白名单，则该参数无效，否则该参数不能为NULL
    uint8_t filter_policy;    ///< @ref cs_adv_filter_policy
    uint8_t prim_phy;         ///< @ref cs_adv_phy, @note CS_ADV_PHY_2M not support in primary phy
    uint8_t secd_phy;         ///< @ref cs_adv_phy
    int8_t tx_pwr;            ///< -127 ~ +20, 0x7F: Host has no preference
    uint16_t timeout;         ///< 超时时间 单位 10ms，0表示无超时时间
    uint8_t *local_addr;      ///< 本地地址
} cs_adv_param_t;

/// 广播数据/扫描数据 @note 若类型为legacy广播，则len最大为31，若类型为Extended广播则len最大为251
typedef struct cs_data {
    uint8_t *data;       ///< 数据
    int len;             ///< 数据长度
} cs_data_t;

/// 连接参数
typedef struct {
    uint16_t conn_intv;      ///< 连接间隔（单位：1.25 ms）
    uint16_t latency_max;    ///< latency最大值
    uint16_t timeout;        ///< 连接超时时间（单位：10 ms）
} cs_gap_conn_params_t;

/// 连接参数更新参数
typedef struct {
    uint16_t conn_intv_min;  ///< 连接间隔最小值（单位：1.25 ms）
    uint16_t conn_intv_max;  ///< 连接间隔最大值（单位：1.25 ms）
    uint16_t latency_max;    ///< latency最大值
    uint16_t timeout;        ///< 连接超时时间（单位：10 ms）
} cs_gap_conn_parameter_t;

/// 绑定信息
typedef struct {
    uint8_t type; ///< @ref cs_bond_info_type
    union {
        /// Identical Address和IRK
        struct {
            cs_gap_addr_t id_addr;
            uint8_t irk[CS_GAP_KEY_LEN];
        } id_info;
        /// EDIV, Random和LTK
        struct {
            uint16_t ediv;
            uint8_t random[CS_GAP_RANDOM_LEN];
            uint8_t ltk[CS_GAP_KEY_LEN];
        } enc_info;
    };
} cs_bond_info_t;

/// 回复的pin码
typedef struct {
    uint8_t type;                      ///< @ref cs_smp_pin_type
    union {
        uint8_t oob[CS_GAP_KEY_LEN];   ///< OOB数据
        uint32_t passkey;              ///< 按键数据
    };
} cs_smp_pin_t;

/// 扩展广播状态
enum cs_gap_adv_state {
    CS_GAP_ADV_ST_STARTED,              ///< 广播已开启
    CS_GAP_ADV_ST_STOPPED_BY_USER,      ///< 用户手动关闭广播
    CS_GAP_ADV_ST_STOPPED_BY_CONNECTED, ///< 连接成功
    CS_GAP_ADV_ST_STOPPED_BY_TIMEOUT,   ///< 广播超时
    CS_GAP_ADV_ST_STOPPED_BY_EVENT,     ///< 达到了Max_Extended_Advertising_Events计数
    CS_GAP_ADV_ST_STOPPED_UNEXPECTED,   ///< 未知原因关闭了广播
};

/// 发射功率等级标记
enum cs_gap_tx_pwr_level_flag {
    CS_GAP_TX_PWR_LEVEL_MIN = 0x01,              ///< 发射功率达到最小值
    CS_GAP_TX_PWR_LEVEL_MAX = 0x02,              ///< 发射功率达到最大值
};

/// 广播数据状态
enum cs_gap_report_state {
    CS_GAP_REPORT_ST_COMPLETE,          ///< Complete
    CS_GAP_REPORT_ST_MORE_DATA,         ///< Incomplete, more data to come
    CS_GAP_REPORT_ST_TRUNCATED,         ///< Incomplete, data truncated, no more to come
};

/// Event structure for @ref CS_GAP_EVT_CONNECTED.
typedef struct {
    uint8_t role;                       ///< 连接角色，参考@ref cs_gap_role
    uint8_t adv_idx;                    ///< 当前连接对应的广播index，若role为Central则该参数无效
    cs_gap_addr_t peer_addr;            ///< 对方地址，参考@ref cs_gap_addr_t
    cs_gap_conn_params_t conn_params;   ///< 连接参数，参考@ref cs_gap_conn_params_t
} cs_gap_evt_connected_t;

/// Event structure for @ref CS_GAP_EVT_DISCONNECTED.
typedef struct {
    uint8_t reason;                     ///< 连接断开原因
} cs_gap_evt_disconnected_t;

/// Event structure for @ref CS_GAP_EVT_ADV_STATE_CHANGED.
typedef struct {
    uint8_t adv_idx;                   ///< 广播index
    uint8_t state;                     ///< 广播状态，参考@ref cs_gap_adv_state
} cs_gap_evt_adv_state_changed_t;

/// Event structure for @ref CS_GAP_EVT_CONN_PARAMS_UPDATE.
typedef struct {
    uint16_t conn_intv_min;  ///< 连接间隔最小值（单位：1.25 ms）
    uint16_t conn_intv_max;  ///< 连接间隔最大值（单位：1.25 ms）
    uint16_t latency_max;    ///< latency最大值
    uint16_t timeout;        ///< 连接超时时间（单位：10 ms）
} cs_gap_evt_conn_params_update_t;

/// Event structure for @ref CS_GAP_EVT_CONN_PARAMS_REQUEST.
typedef struct {
    uint16_t conn_intv_min;  ///< 连接间隔最小值（单位：1.25 ms）
    uint16_t conn_intv_max;  ///< 连接间隔最大值（单位：1.25 ms）
    uint16_t latency_max;    ///< latency最大值
    uint16_t timeout;        ///< 连接超时时间（单位：10 ms）
} cs_gap_evt_conn_params_request_t;

/// 广播report类型
typedef struct {
    uint8_t connectable : 1;    ///< 可连接广播标记
    uint8_t scannable   : 1;    ///< 可扫描广播标记
    uint8_t direct      : 1;    ///< 直连广播标记
    uint8_t scan_resp   : 1;    ///< scan response数据标记
    uint8_t legacy_adv  : 1;    ///< Legacy广播标记
    uint8_t data_state  : 2;    ///< 数据状态，参考 @ref cs_gap_report_state
    uint8_t rsv         : 1;    ///< reserved
} cs_gap_report_event_type_t;

/// Event structure for @ref CS_GAP_EVT_ADV_REPORT.
typedef struct {
    cs_gap_report_event_type_t report_type;     ///< 广播类型 @ref cs_gap_report_event_type_t
    cs_gap_addr_t addr;                         ///< 广播设备的地址
    cs_gap_addr_t direct_adv_addr;              ///< 直接广播目的地址，若report_type中direct为0则该参数无效
    uint8_t prim_phy;                           ///< primary广播信道phy，参考@ref cs_adv_phy
    uint8_t secd_phy;                           ///< secondary广播信道phy，参考@ref cs_adv_phy
    uint8_t adv_sid;                            ///< 0x00~0x0F, 0xFF means No ADI field provided
    uint16_t padv_intv;                         ///< 对应周期广播的间隔（单位：1.25 ms），如果为0x0000则表示无周期广播
    int8_t tx_pwr;                              ///< 发射功率，若值为@ref CS_GAP_ADV_TX_POWER_NO_AVA 则表示不可用
    int8_t rssi;                                ///< 若值为@ref CS_GAP_ADV_TX_POWER_NO_AVA 则表示不可用
    const uint8_t *data;                        ///< 广播数据
    uint8_t data_len;                           ///< 广播长度
} cs_gap_evt_adv_report_t;

/// Event structure for @ref CS_GAP_EVT_ENCRYPT.
typedef struct {
    bool encrypted;       ///< 链路当前加密状态
} cs_gap_evt_encrypt_t;

/// SMP Authentication requirements flags
typedef struct {
    uint8_t bond_flags : 2;  ///< 是否绑定
    uint8_t mitm       : 1;  ///< 是否支持MITM
    uint8_t sc         : 1;  ///< 是否支持安全配对
    uint8_t keypress   : 1;  ///< 是否支持按键通知
    uint8_t rsv        : 3;  ///< reserved
} cs_gap_auth_t;

/// Event structure for @ref CS_GAP_EVT_SEC_REQUEST.
typedef struct {
    cs_gap_auth_t auth; ///< 请求安全属性及绑定信息，参考 @ref cs_gap_auth_t
} cs_gap_evt_sec_request_t;

/// Event structure for @ref CS_GAP_EVT_PAIRING_REQUEST.
typedef struct {
    uint8_t io_capability;              ///< io_capability, @ref cs_smp_io_capability
    uint8_t oob_data_flag;              ///< 是否支持 oob_data
    cs_gap_auth_t authreq;              ///< 参考 @ref cs_gap_auth_t
    uint8_t initiator_key_distribution; ///< initiator_key_distribution @ref cs_smp_distribution
    uint8_t responder_key_distribution; ///< responder_key_distribution @ref cs_smp_distribution
} cs_gap_evt_pairing_request_t;

/// Event structure for @ref CS_GAP_EVT_PIN_REQUEST.
typedef struct {
    uint8_t type; ///< @ref cs_smp_pin_type
    uint32_t pin_code; ///< only valid if type == CS_BLE_SMP_PIN_TYPE_DIS
} cs_gap_evt_pin_request_t;

/// Event structure for @ref CS_GAP_EVT_LTK_REQUEST.
typedef struct {
    uint16_t ediv; ///< EDIV
    uint8_t random[CS_GAP_RANDOM_LEN]; ///< RANDOM
} cs_gap_evt_ltk_request_t;

/// Event structure for @ref CS_GAP_EVT_BOND_INFO_REQUEST.
typedef struct {
    uint8_t type; ///< @ref cs_bond_info_type
} cs_gap_evt_bond_info_request_t;

/// Event structure for @ref CS_GAP_EVT_BONDED.
typedef struct {
    uint32_t status;     ///< 绑定状态
    cs_gap_auth_t auth; ///< 绑定成功信息，参考@ref cs_gap_auth_t
} cs_gap_evt_bonded_t;

/// Event structure for @ref CS_GAP_EVT_BOND_INFO.
typedef struct {
    uint8_t type; ///< @ref cs_bond_info_type
    union {
        /// Identical Address和IRK
        struct {
            cs_gap_addr_t id_addr;
            uint8_t irk[CS_GAP_KEY_LEN];
        } id_info;
        /// EDIV, Random和LTK
        struct {
            uint16_t ediv;
            uint8_t random[CS_GAP_RANDOM_LEN];
            uint8_t ltk[CS_GAP_KEY_LEN];
        } enc_info;
    };
} cs_gap_evt_bond_info_t;

/// Event structure for @ref CS_GAP_EVT_TIMEOUT.
typedef struct {
    uint8_t source; ///< 超时原因，参考@ref cs_gap_timeout_source_t
} cs_gap_evt_timeout_t;

/// Event structure for @ref CS_GAP_EVT_POWER_CHANGED.
typedef struct {
    uint8_t side;                   ///< 本方或对方发射功率更新，0x00 本地发射功率更新，0x01 对方发射功率更新
    uint8_t phy;                    ///< 更新功率对应的PHY，0x01 1M Phy, 0x02 2M Phy, 0x03 coded Phy(S=8), 0x04 coded Phy(S=2)
    int8_t tx_pwr;                  ///< 更新后的功率，若值为@ref CS_GAP_ADV_TX_POWER_NO_AVA(0x7F) 则表示不可用
    uint8_t tx_pwr_level_flag;      ///< 是否达到最大或最小功率，参考 @ref cs_gap_tx_pwr_level_flag
    int8_t tx_pwr_delta;            ///< 功率变化，若值为@ref CS_GAP_ADV_TX_POWER_NO_AVA(0x7F) 则表示不可用
} cs_gap_evt_power_changed_t;

/// Event structure for @ref CS_GAP_EVT_PHY_UPDATE.
typedef struct {
    uint8_t status;                 ///< 更新状态，为0时表示更新成功，否则为更新失败
    uint8_t tx_phy;                 ///< TX PHY for this connection，参考 @ref cs_gap_phy
    uint8_t rx_phy;                 ///< RX PHY for this connection，参考 @ref cs_gap_phy
} cs_gap_evt_phy_update_t;

/// Event structure for @ref CS_GAP_EVT_DATA_LEN_CHANGED.
typedef struct {
    uint16_t max_tx_octets;   ///< 最大发送字节数
    uint16_t max_rx_octets;   ///< 最大接收字节数
    uint16_t max_tx_time_us;  ///< 最大发送时间（单位：us）
    uint16_t max_rx_time_us;  ///< 最大接收时间（单位：us）
} cs_gap_evt_data_len_changed_t;

/// Event structure for @ref CS_GAP_EVT_DEVICE_INFO.
typedef struct {
    uint8_t type;                     ///< 信息类型 @ref cs_dev_info_type
    union {
        /// bluetooth version
        struct {
            uint8_t version;              ///< 蓝牙version
            uint8_t subversion;           ///< 蓝牙subversion
            uint16_t company_identifier;  ///< company_identifier
        } version;
        /// device feature
        uint8_t features[8];              ///< 蓝牙feature，见core spec [Vol 6] Part B, Section 4.6
        /// RSSI
        int8_t rssi;                      ///< 信号强度
        /// channel map
        uint8_t channel_map[5];           ///< Channel map
    };
} cs_gap_evt_device_info_t;

/// Event structure for @ref CS_GAP_EVT_SCAN_REQ_RECV_T.
typedef struct {
    cs_gap_addr_t addr;                         ///< 广播设备的地址
    uint8_t adv_idx;                            ///< advertise index
} cs_gap_evt_scan_req_recv_t;

/// Event structure for @ref CS_GAP_EVT_HCI_ERROR.
typedef struct {
    uint16_t hci_opcode;     ///< 出错时对应的hci opcode
    uint8_t status;          ///< 出错时的HCI status状态
} cs_gap_evt_hci_error_t;

/// GAP 消息结构体
typedef struct {
    uint8_t conn_idx; ///< 连接 index
    union {
        cs_gap_evt_connected_t              connected;             ///< 连接成功事件参数
        cs_gap_evt_disconnected_t           disconnected;          ///< 连接断开事件参数
        cs_gap_evt_adv_state_changed_t      adv_state_changed;     ///< 广播状态变化事件参数
        cs_gap_evt_conn_params_update_t     conn_params_update;    ///< 参数更新完成事件参数
        cs_gap_evt_conn_params_request_t    conn_params_request;   ///< 参数更新请求参数
        cs_gap_evt_adv_report_t             adv_report;            ///< 广播上报事件参数
        cs_gap_evt_encrypt_t                encrypt;               ///< 链路加密状态改变事件参数
        cs_gap_evt_sec_request_t            sec_request;           ///< 加密请求事件参数
        cs_gap_evt_pairing_request_t        pairing_request;       ///< 配对请求事件参数
        cs_gap_evt_pin_request_t            pin_request;           ///< 配对PIN码请求事件参数
        cs_gap_evt_ltk_request_t            ltk_request;           ///< LTK请求参数
        cs_gap_evt_bond_info_request_t      bond_info_request;     ///< 绑定信息请求参数
        cs_gap_evt_bonded_t                 bonded;                ///< 配对完成事件参数
        cs_gap_evt_bond_info_t              bond_info;             ///< 配对信息上报事件（LTK/IRK）参数
        cs_gap_evt_timeout_t                timeout;               ///< 超时事件参数
        cs_gap_evt_power_changed_t          power_changed;         ///< 功率变化事件参数
        cs_gap_evt_phy_update_t             phy_updated;           ///< PHY变化事件参数
        cs_gap_evt_data_len_changed_t       data_len_changed;      ///< Data Length变化事件参数
        cs_gap_evt_device_info_t            device_info;           ///< 设备信息参数
        cs_gap_evt_scan_req_recv_t          scan_req_recv;         ///< 扫描请求事件参数
        cs_gap_evt_hci_error_t              hci_error;             ///< HCI指令错误参数
    };
} cs_ble_gap_evt_t;

/**@brief 设置默认地址
 * @note 地址必须在无广播和连接的状态下设置
 * @note 该地址影响设备的连接、扫描地址以及广播地址（仅当广播参数中local_addr为NULL时）
 * @param[in]  addr_type    地址类型，参考 @ref cs_adv_addr_type.
 * @param[in]  addr         设备地址
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_addr_set(uint8_t addr_type, const uint8_t addr[CS_GAP_ADDR_LEN]);

/**@brief 获取默认地址
 * @param[in]   addr_type  地址类型，参考 @ref cs_adv_addr_type.
 * @param[out]  addr       地址指针
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_addr_get(uint8_t addr_type, uint8_t addr[CS_GAP_ADDR_LEN]);
/**@brief 设置白名单
 * @details 只能在无广播使能的状态下进行调用，若fa_addrs不为NULL，则清除当前Filter Accept列表内容并将fa_addrs列表中的地址设置到白名单，
 * @details 若fa_addrs为NULL则仅清除当前Filter Accept列表内容
 * @param[in]  fa_addrs   filter accept地址指针组
 * @param[in]  len        数组长度
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_accept_list_set(const cs_gap_addr_t *fa_addrs, uint8_t len);

/**@brief 开启指定adv_idx的广播，或更新广播数据
 * @details 多广播场景下用参数adv_idx区分不同广播，不同广播支持配置成不同的广播地址\n
 *          若参数adv_param中的local_addr为NULL时，广播使用@ref cs_gap_addr_set 设置的地址，否则使用local_addr指定的地址
 * @param[in]  adv_idx        广播index
 * @param[in]  adv_param      广播参数参考@ref cs_adv_param_t, 若为NULL则只更新当前广播的广播数据
 * @param[in]  adv_data       广播数据
 * @param[in]  scan_rsp_data  扫描数据
 * @note 若需要更新广播参数，则该广播必须为已开启状态
 * @note 参数adv_idx值必须小于HOST支持的最大广播数，最大广播数在协议栈初始化时定义，见函数@ref cs_ble_init 的参数@ref cs_stack_param
 * @note 如果使用的是 ble_1peripheral 的蓝牙库，更新广播数据时，需要先停止广播，再调用此函数以更新广播数据
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_adv_start(uint8_t adv_idx, cs_adv_param_t *adv_param, cs_data_t *adv_data, cs_data_t *scan_rsp_data);

/**@brief 关闭指定index的广播
 * @param[in]  adv_idx        广播index
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_adv_stop(uint8_t adv_idx);

/**@brief 请求连接参数更新
 * @param[in]  conn_idx       连接index
 * @param[in]  p_conn_params  连接参数，参考 @ref cs_gap_conn_params_t
 * @return 执行结果，参考@ref cs_ble_error
 * @note 作为centrel角色时更新连接参数，作为peripheral角色时请求连接参数
 */
uint32_t cs_gap_conn_param_update(uint8_t conn_idx, cs_gap_conn_params_t const *p_conn_params);

/**@brief 请求连接参数更新
 * @param[in]  conn_idx       连接index
 * @param[in]  p_conn_params  连接参数，参考 @ref cs_gap_conn_parameter_t
 * @return 执行结果，参考@ref cs_ble_error
 * @note 该接口与@ref cs_gap_conn_param_update 的区别是请求的参数将最大连接间隔和最小连接间隔做了区分
 * @note 作为centrel角色时更新连接参数，作为peripheral角色时请求连接参数
 */
uint32_t cs_gap_conn_param_request(uint8_t conn_idx, cs_gap_conn_parameter_t const *p_conn_params);

/**@brief 断开连接
 * @param[in]  conn_idx        连接index
 * @param[in]  hci_status_code 断开原因
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_disconnect(uint8_t conn_idx, uint8_t hci_status_code);

/**@brief 配对/加密请求
 * @param[in]  conn_idx       连接index
 * @param[in]  auth           auth，参考@ref cs_gap_auth_t
 * @note 只有peripheral角色可以调用此接口
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_security_request(uint8_t conn_idx, cs_gap_auth_t auth);

/// GAP SMP配对参数结构体
typedef struct {
    uint8_t io_capability;              ///< io_capability, @ref cs_smp_io_capability
    uint8_t oob_data_flag;              ///< 是否支持OOB
    cs_gap_auth_t authreq;              ///< authreq，参考@ref cs_gap_auth_t
    uint8_t initiator_key_distribution; ///< initiator_key_distribution @ref cs_smp_distribution
    uint8_t responder_key_distribution; ///< responder_key_distribution @ref cs_smp_distribution
} cs_pairing_param_t;

/**@brief 配对/绑定请求
 * @param[in]  conn_idx       连接index
 * @param[in]  request        配对请求参数，参考 @ref cs_pairing_param_t
 * @note 只有central角色可以调用此接口
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_pairing_request(uint8_t conn_idx, cs_pairing_param_t *request);

/**@brief 配对/绑定请求
 * @param[in]  conn_idx       连接index
 * @param[in]  response       配对响应参数，参考 @ref cs_pairing_param_t
 * @note 只有peripheral角色在收到消息 @ref CS_GAP_EVT_PAIRING_REQUEST 后才可以调用此接口响应配对请求
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_pairing_response(uint8_t conn_idx, cs_pairing_param_t *response);

/**@brief 链路加密请求
 * @param[in]  conn_idx       连接index
 * @param[in]  ediv           EDIV
 * @param[in]  rand           random value
 * @param[in]  ltk            Long Term Key
 * @note 只有central角色可以调用此接口，用于发起链路加密流程
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_encrypt(uint8_t conn_idx, uint16_t ediv, uint8_t *rand, uint8_t *ltk);

/**@brief 链路加密响应
 * @param[in]  conn_idx       连接index
 * @param[in]  ltk            Long Term Key
 * @note 只有peripheral角色在收到消息 @ref CS_GAP_EVT_LTK_REQUEST 后才可以调用此接口响应加密请求
 * @note 若ltk为 NULL时表示LTK key missing
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_ltk_response(uint8_t conn_idx, uint8_t *ltk);

/**@brief 配对信息响应
 * @param[in]  conn_idx       连接index
 * @param[in]  bond_info      配对信息，参考@ref cs_bond_info_t
 * @note 该接口用于回复CS_GAP_EVT_BOND_INFO_REQUEST消息
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_bond_info_response(uint8_t conn_idx, cs_bond_info_t *bond_info);

/**@brief PIN码响应
 * @param[in]  conn_idx       连接index
 * @param[in]  accept         是否接受配对请求
 * @param[in]  pin_info       需要响应的PIN码，参考 @ref cs_smp_pin_t
 * @note 该接口用于响应 @ref CS_GAP_EVT_PIN_REQUEST 事件
 * @note 在pin_info::type为CS_SMP_PIN_TYPE_PK_REQ/CS_BLE_SMP_PIN_TYPE_OOB_REQ时用于回复相应的PIN码
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_pin_response(uint8_t conn_idx, uint8_t accept, cs_smp_pin_t *pin_info);

/**@brief 获取当前加密状态
 * @param[in]  conn_idx       连接index
 * @param[out] state          true/false，链路是否已加密
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_get_encrypt_state(uint8_t conn_idx, uint8_t *state);

/// cs_gap_phys
typedef struct {
    uint8_t phy_1m    : 1; ///< 1M phy
    uint8_t phy_2m    : 1; ///< 2M phy
    uint8_t phy_coded : 1; ///< Coded phy
    uint8_t rsv       : 5; ///< reserved
} cs_gap_phys_t;

/**@brief phy更新请求
 * @param[in]  conn_idx       连接index
 * @param[in]  tx_phys        链路发送PHY的建议值，参考@ref cs_gap_phys_t
 * @param[in]  rx_phys        链路接收PHY的建议值，参考@ref cs_gap_phys_t
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_phy_update(uint8_t conn_idx, cs_gap_phys_t tx_phys, cs_gap_phys_t rx_phys);

/**@brief 更新data length
 * @param[in]  conn_idx       连接index
 * @param[out] tx_octets      建议最大的数据字节长度
 * @param[out] tx_time        建议最大的数据传输时间（单位微秒）
 * @note tx_octets的范围是  0x001B ~ 0x00FB
 * @note tx_time的范围是  0x0148 ~ 0x4290
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_data_length_update(uint8_t conn_idx, uint16_t tx_octets, uint16_t tx_time);

/**@brief Set Host Channel Classification
 * @param[in]  channel_map  低37 bit对应通道号: field bad = 0, unknown = 1
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_host_channel_map_update(const uint8_t channel_map[5]);

/// cs_scanning_param
typedef struct {
    uint8_t scan_type;   ///< is Active Scanning
    uint16_t interval;   ///< 扫描间隔
    uint16_t window;     ///< 扫描窗口
} cs_scanning_param_t;

/// cs_scan_param
typedef struct {
    uint8_t addr_type;               ///< See @ref cs_adv_addr_type
    uint8_t filter_policy;           ///< @ref cs_scan_filter_policy
    uint16_t timeout;                ///< 超时时间，单位10毫秒
    cs_scanning_param_t *scan_1m;    ///< 1M phy 扫描参数
    cs_scanning_param_t *scan_coded; ///< Coded phy 扫描参数
} cs_scan_param_t;

/**@brief 开启扫描
 * @param[in]  param       扫描参数，参考@ref cs_scan_param_t
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_scan_start(cs_scan_param_t *param);

/**@brief 停止扫描
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_scan_stop(void);

/// cs_conn_phy_param
typedef struct {
    uint16_t scan_intv;      ///< 扫描间隔
    uint16_t scan_wind;      ///< 扫描窗口
    uint16_t conn_intv_min;  ///< 连接间隔最小色值
    uint16_t conn_intv_max;  ///< 连接间隔最大值
    uint16_t latency_max;    ///< latency最大值
    uint16_t timeout;        ///< 连接超时时间
} cs_conn_phy_param_t;

/// cs_conn_param
typedef struct {
    uint8_t addr_type;               ///< cs_adv_addr_type
    uint8_t filter_policy;           ///< @ref cs_conn_filter_policy
    cs_gap_addr_t peer_addr;         ///< 连接地址
    cs_conn_phy_param_t *scan_1m;    ///< 1M phy 连接参数
    cs_conn_phy_param_t *scan_2m;    ///< 2M phy 连接参数
    cs_conn_phy_param_t *scan_coded; ///< Coded phy 扫描参数
} cs_conn_param_t;
/**@brief 创建连接
 * @param[in]  param       连接参数
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_connect(cs_conn_param_t *param);

/**@brief 取消连接
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_gap_connect_cancel(void);

/**@brief 获取设备信息
 * @param[in]  type        信息类型，参考@ref cs_dev_info_type
 * @param[in]  conn_idx    若type为CS_DEV_INFO_LOCAL_XXX则该参数无效
 * @return 执行结果，参考@ref cs_ble_error
 * @note 返回的设备信息会通过@ref CS_GAP_EVT_DEVICE_INFO 事件上报
 */
uint32_t cs_gap_get_device_info(uint8_t type, uint8_t conn_idx);

/**@brief 根据指定IRK生成RPA
 * @param[in]   irk       用于生成RPA的IRK
 * @param[out]  rpa       生成的RPA
 */
void cs_gap_rpa_gen(const uint8_t irk[16], uint8_t rpa[6]);

/**@brief 检查指定的RPA和IRK是否匹配
 * @param[in]   irk       与RPA对应的IRK
 * @param[out]  rpa       需要检查的RPA
 * @return 是否匹配
 * @note 若给定的RPA类型错误则也返回false
 */
bool cs_gap_rpa_check(const uint8_t irk[16], const uint8_t rpa[6]);

#endif /* __CS_BLE_GAP_H__ */

/// @}
