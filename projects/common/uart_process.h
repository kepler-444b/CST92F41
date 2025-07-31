/**
 * @file    uart_process.h
 * @author  haobo
 * @brief
 * @version 0.1
 * @date    2024-02-04
 * @copyright Copyright (c) 2024, CHIPSEA Co., Ltd.
 * @note
 */
#ifndef __UART_PROCESS_H__
#define __UART_PROCESS_H__

#include <stdint.h>
#include "cs_log.h"
#include "board_cst92f41_evb.h"

#define AT_COMMAND      true
#define log_debug       CS_LOG_DEBUG

#define PIN_AT_WAKEUP   PAD_BUTTON_0

#define ATT_HANDLE_NOTIFY       0x17
#define ATT_HANDLE_WRITE        0x1A
#define ATT_HANDLE_AT           0x1C

/*********************************************************************
 * FUNCTIONS
 */
#ifdef __cplusplus
extern "C"
{
#endif

typedef void(*ble_data_send_interface)(uint8_t role, uint8_t channel, const uint8_t *data, uint16_t len, uint8_t att_handle);
typedef int8_t(*ble_bridge_handler_interface)(uint8_t in_role, uint8_t channel, const uint8_t *data, uint16_t len);

void at_uart_restore(void);
void at_wakeup_isr_handler(uint32_t int_status);
void at_uart_sleep(void);
void at_uart_init(uint32_t baud_rate);
void uart_send_block(const uint8_t *buf, unsigned length);
void uart_send_data(uint8_t from, uint8_t *data, uint16_t len);
void uart_slave_rx_data(uint8_t conn_idx, uint8_t handle, const uint8_t *data, uint16_t len);
void uart_master_rx_data(uint8_t conn_idx, uint8_t handle, const uint8_t *data, uint16_t len);
void set_ble_data_send_interface(ble_data_send_interface intf);
void set_ble_bridge_handler_interface(ble_bridge_handler_interface intf);

#ifdef __cplusplus
}
#endif
/*********************************************************************/

#endif /*__UART_PROCESS_H__*/
