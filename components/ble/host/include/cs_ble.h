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
 * @file     cs_ble.h
 * @date     27. May 2021
 * @author   chipsea
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @addtogroup CS_BLE_COMMON BLE_COMMON
 * @brief CS_BLE初始化接口
 */
/// @{
#ifndef __CS_BLE_API_H__
#define __CS_BLE_API_H__
#include <stdint.h>
#include <stdlib.h>
#include "cs_ble_gap.h"
#include "cs_ble_gatt.h"
#include "cs_ble_misc.h"
#include "cs_ble_error.h"

/*******************************************************************************
 * DEFINES
 */

/// BLE 消息结构体
typedef struct {
    union {
        cs_ble_gap_evt_t gap;   ///< gap 消息，参考 @ref cs_ble_gap_evt_t
        cs_ble_gatt_evt_t gatt; ///< gatt 消息，参考 @ref cs_ble_gatt_evt_t
    };
} cs_ble_evt_t;

/// 协议栈初始化参数
struct cs_stack_param {
    uint8_t max_connection;        ///< 最大连接数
    uint8_t max_ext_adv_set;       ///< 最大广播数
    uint16_t max_att_mtu;          ///< MAX ATT MTU
    uint16_t max_gatt_serv_num;    ///< MAX GATT service number
    uint16_t max_gatt_write_cache; ///< MAX GATT server prepare write cache length, Use MAX mtu if set to 0
    bool smp_sc_support;           ///< Secure Connection Pairing support
    uint8_t max_user_event_num;    ///< MAX user event number
    uint8_t max_gatt_buffer_num;   ///< 每个GATT连接最大数据缓存数量(1~63, 默认63)
};

/**@brief BLE 事件回调类型
 * @note 所有蓝牙相关消息均通过该结构体通知上层应用
 * @param[in] evt_id  事件id，参考 @ref CS_GAP_EVENTS, @ref CS_CLE_GATT_NTS
 * @param[in] evt  事件结构体，参考 @ref cs_ble_evt_t
 */
typedef void (*cs_ble_event_cb_t)(uint16_t evt_id, const cs_ble_evt_t *evt);

/**@brief 注册 BLE 事件回调函数
 * @note回调可以多次注册，各回调函数按照注册顺序依次调用
 * @param[in]  callback   回调函数，参考@ref cs_ble_event_cb_t
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_event_callback_reg(cs_ble_event_cb_t callback);

/**@brief 注销 BLE 事件回调函数
 * @param[in]  callback   回调函数，参考@ref cs_ble_event_cb_t
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_event_callback_unreg(cs_ble_event_cb_t callback);

/**@brief 终止 BLE 事件处理
 *
 * 当调用@ref cs_event_callback_reg 多次注册BLE事件回调后，在@ref cs_ble_cb_t
 * 函数中调用后将不再回调其它已注册的回调函数
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_event_abort(void);

/**@brief 不常用的参数配置
 * @param[in]  type  参数类型，参考@ref cs_ble_opt_cfg_type
 * @param[in]  cfg   需要配置的参数，参考@ref cs_ble_opt_cfg_t
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_opt_cfg_set(uint8_t type, cs_ble_opt_cfg_t *cfg);

/**@brief 获取不常用的参数
 * @param[in]  type  参数类型，参考@ref cs_ble_opt_cfg_type
 * @param[in]  cfg   需要获取的参数，参考@ref cs_ble_opt_cfg_t
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_opt_cfg_get(uint8_t type, cs_ble_opt_cfg_t *cfg);

/**@brief 初始化协议栈
 * @param[in]  param  协议栈初始化参数，参考 @ref cs_stack_param
 * @return 执行结果，参考@ref cs_ble_error
 * @note 该接口只能调用一次，并且必须在调用其它蓝牙接口前调用
 */
uint32_t ble_init(struct cs_stack_param *param);

/**@brief 设置用户事件回调函数
 * @param[in]  event_id  事件ID，id值应小于 @ref cs_stack_param 中的max_user_event_num值
 * @param[in]  callback  协议栈初始化参数，参考 @ref cs_stack_param
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_ble_set_user_event_callback(uint8_t event_id, void (*callback)(void));

/**@brief 触发用户事件回调函数
 * @param[in]  event_id  事件ID，id值应小于 @ref cs_stack_param 中的max_user_event_num值
 * @return 执行结果，参考@ref cs_ble_error
 */
uint32_t cs_ble_user_event_trigger(uint8_t event_id);

/**@brief 获取Time Tick
 * @return tick
 */
size_t cs_ble_tick_get(void);

/**@brief 计算tick2 - tick1的时间间隔，单位10ms
 * @param[in]  tick1  tick1
 * @param[in]  tick2  tick2
 * @return tick2 - tick1 的时间间隔，单位10ms
 */
int cs_ble_tick_diff_10ms(size_t tick1, size_t tick2);

/**@brief 调度协议栈
 */
void cs_ble_schedule(void);

#endif /* __CS_BLE_API_H__ */
/// @}
