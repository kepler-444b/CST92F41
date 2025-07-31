/**
 * @file    uart_process.c
 * @author  haobo
 * @brief
 * @version 0.1
 * @date    2024-02-04
 * @copyright Copyright (c) 2024, CHIPSEA Co., Ltd.
 * @note
 */

/*********************************************************************
 * INCLUDES
 */
#include "sdk_config.h"
#include "cs_common_utils.h"
#include "cs_driver.h"
#include "evt.h"

#include "app_at.h"
#include "at_command.h"
#include "uart_process.h"

#ifdef AT_ENABLE

#define UART_FIFO_BUF_SIZE      1024
#define PACKET_BUF_SIZE_MAX     498
#define RX_50_MS                50

#if ((CONFIG_AT_USART0) && (RTE_USART0))
    #define AT_USART CS_USART0
#elif ((CONFIG_AT_USART1) && (RTE_USART1))
    #define AT_USART CS_USART1
#endif

static uint8_t packet_buf[PACKET_BUF_SIZE_MAX];

static cs_kfifo_t uart_fifo;
static uint8_t uart_fifo_buf[UART_FIFO_BUF_SIZE];

static uint32_t g_baud_rate = 115200;
static uint32_t uart_timer_status = 0;
static uint32_t uart_last_byte_time = 0;
static evt_timer_t at_evt_timer;

static ble_data_send_interface ble_data_send_itf = NULL;
static ble_bridge_handler_interface ble_bridge_handler_intf = NULL;
static uint8_t at_cmd_ble_conn_idx = 0xFF;

static void tansparent_ble_data(uint8_t *buf, uint16_t buf_len)
{
    at_ble_data_session_t data_session = at_get_ble_data_session();
    at_ble_mode_t ble_mode = user_data_get_ble_mode();

    CS_LOG_DEBUG("%s: ble_mode %d, direction %d\r\n", __func__, ble_mode, data_session.direction);
    if (ble_mode == AT_BLE_MODE_MASTER_AND_SLAVE) {
        if (data_session.direction == AT_BLE_DATA_TO_MASTER) {
            if (ble_data_send_itf != NULL) {
                ble_data_send_itf(CS_GAP_ROLE_PHERIPHERAL, data_session.channel, buf, buf_len, ATT_HANDLE_NOTIFY);
            }
        } else if (data_session.direction == AT_BLE_DATA_TO_SLAVE){
            if (ble_data_send_itf != NULL) {
                ble_data_send_itf(CS_GAP_ROLE_CENTRAL, data_session.channel, buf, buf_len, 0xFF);
            }
        }
    } else if (ble_mode == AT_BLE_MODE_MASTER_ONLY) {
        if (ble_data_send_itf != NULL) {
            ble_data_send_itf(CS_GAP_ROLE_CENTRAL, data_session.channel, buf, buf_len, 0xFF);
        }
    } else if (ble_mode == AT_BLE_MODE_SLAVE_ONLY) {
        if (ble_data_send_itf != NULL) {
            ble_data_send_itf(CS_GAP_ROLE_PHERIPHERAL, data_session.channel, buf, buf_len, ATT_HANDLE_NOTIFY);
        }
    }
}

static void at_evt_handler(evt_timer_t *timer, void *param)
{
    uint16_t buff_len =0;

    if (cs_kfifo_is_empty(&uart_fifo)) {
        evt_timer_del(&at_evt_timer);
        uart_timer_status = 0;
        return;
    }

    if (drv_gpio_read(CS_GPIO0, GPIO_MASK(PIN_AT_WAKEUP))) {
        log_debug("Put pin %d's state to low to use at!\r\n", PIN_AT_WAKEUP);
        cs_kfifo_reset(&uart_fifo);
        evt_timer_del(&at_evt_timer);
        return;
    }

    if(cs_time_diff(cs_time(), uart_last_byte_time) < EVT_TIMER_MS2TICK(RX_50_MS)) {
        return;
    }

    buff_len = cs_kfifo_len(&uart_fifo);
    if ( buff_len > PACKET_BUF_SIZE_MAX) {
        buff_len = PACKET_BUF_SIZE_MAX;
    }

    memset(packet_buf, 0, sizeof(packet_buf));
    cs_kfifo_peek(&uart_fifo, packet_buf, buff_len);

    // AT test command
    if (memcmp(packet_buf, "AT\r\n", 4) == 0)
    {
        __cs_kfifo_add_out(&uart_fifo,4);
        uart_send_block((uint8_t *)"OK\r\n", 4);
    }
    // AT reply
    else if (memcmp(packet_buf, "OK\r\n", 4) == 0)
    {
        __cs_kfifo_add_out(&uart_fifo,4);
        uart_send_block((uint8_t *)"OK\r\n", 4);
    }
    // AT command
    else if (memcmp(packet_buf, "AT", 2) == 0)
    {
        at_cmd_process_status_t status = AT_PROCESS_OK;
        status = at_command_process(packet_buf, buff_len, AT_CMD_FROM_AT);
        if (status != AT_PROCESS_OK) {
            tansparent_ble_data(packet_buf, buff_len);
        }
        __cs_kfifo_add_out(&uart_fifo, buff_len);
    } else {
        tansparent_ble_data(packet_buf, buff_len);
        __cs_kfifo_add_out(&uart_fifo, buff_len);
    }
}

#if (RTE_PM)
static void at_pm_sleep_store_restore_handler(pm_sleep_state_t sleep_state, pm_status_t power_status)
{
    if (sleep_state == PM_SLEEP_RESTORE_HSI) {
        at_uart_init(g_baud_rate);
    }
}
#endif

static void __RAM_CODE at_usart_cb(void *cs_usart, drv_event_t event, void *rx_buf, void *rx_cnt)
{
    uart_last_byte_time=cs_time();

    cs_kfifo_in(&uart_fifo, rx_buf, (uint32_t)rx_cnt);

    if(uart_timer_status == 0) {
        if(evt_timer_set(&at_evt_timer, 10, EVT_TIMER_REPEAT, at_evt_handler, NULL) == true) {
            uart_timer_status = 1;
        }
    } else {
        evt_timer_set(&at_evt_timer, 5, EVT_TIMER_REPEAT, at_evt_handler, NULL);
    }
}

void at_wakeup_isr_handler(uint32_t int_status)
{
#if ((CONFIG_AT_USART0) && (RTE_USART0))
    uint16_t pm_id = (AT_USART == CS_USART0 ? PM_ID_USART0: PM_ID_USART1);
#elif ((CONFIG_AT_USART1) && (RTE_USART1))
    uint16_t pm_id = (AT_USART == CS_USART1 ? PM_ID_USART1: PM_ID_USART0);
#endif

    uint32_t gpio = __builtin_ctz(int_status);


    CS_LOG_DEBUG("%s: gpio: %d \r\n", __func__, gpio);
    if (gpio != PIN_AT_WAKEUP) return;

    if (drv_gpio_read(CS_GPIO0, GPIO_MASK(PIN_AT_WAKEUP))) {
        CS_LOG_DEBUG("%s: allow sleep\r\n", __func__);

        evt_timer_del(&at_evt_timer);
        pm_sleep_allow(pm_id);

    } else {
        CS_LOG_DEBUG("%s: prevent sleep\r\n", __func__);
        evt_timer_set(&at_evt_timer, 50, EVT_TIMER_REPEAT, at_evt_handler, NULL);
        pm_sleep_prevent(pm_id);
        app_at_restore();
    }
}

void at_uart_sleep(void)
{
#if ((CONFIG_AT_USART0) && (RTE_USART0))
    uint16_t pm_id = (AT_USART == CS_USART0 ? PM_ID_USART0: PM_ID_USART1);
#elif ((CONFIG_AT_USART1) && (RTE_USART1))
    uint16_t pm_id = (AT_USART == CS_USART1 ? PM_ID_USART1: PM_ID_USART0);
#endif
    evt_timer_del(&at_evt_timer);
    pm_sleep_allow(pm_id);
}

void at_uart_init(uint32_t baud_rate)
{
    g_baud_rate = baud_rate;

    cs_kfifo_init(&uart_fifo, uart_fifo_buf, UART_FIFO_BUF_SIZE);

    // Config UART
#if ((CONFIG_AT_USART0) && (RTE_USART0))
    usart_ex_config_t usart_cfg = {
        .baudrate = baud_rate,
    };
    drv_usart_ex_init(AT_USART, &usart_cfg);
    drv_usart_ex_register_isr_callback(AT_USART, at_usart_cb);
#elif ((CONFIG_AT_USART1) && (RTE_USART1))
    usart_config_t usart_cfg = {
        .baudrate = baud_rate,
        .flow_control = USART_FLOW_CONTROL_NONE,
        .data_bit = USART_DATA_BIT_8,
        .stop_bit = USART_STOP_BIT_1,
        .parity = USART_PARITY_NONE,
    };

    drv_usart_init(AT_USART, &usart_cfg);
    drv_usart_register_isr_callback(AT_USART, at_usart_cb);
    drv_usart_read_int(AT_USART, NULL, 0);
#endif

}

void uart_send_block(const uint8_t *buf, unsigned length)
{
#if ((CONFIG_AT_USART0) && (RTE_USART0))
    drv_usart_ex_write(AT_USART, buf, length, DRV_MAX_DELAY);
#elif ((CONFIG_AT_USART1) && (RTE_USART1))
    drv_usart_write(AT_USART, buf, length, DRV_MAX_DELAY);
#endif
}

void uart_reply_ok(at_cmd_source from, uint8_t conn_idx)
{
    if (from == AT_CMD_FROM_AT) {
        uart_send_block((uint8_t *)"OK\r\n", 4);
    } else if (from == AT_CMD_FROM_BLE) {
        uint8_t notify_data[4] = {'O', 'K', '\r', '\n'};

        if (ble_data_send_itf != NULL) {
            uint8_t channel = get_channel_by_conn_idx(CS_GAP_ROLE_PHERIPHERAL, conn_idx);
            if (channel != 0xFF) {
                ble_data_send_itf(CS_GAP_ROLE_PHERIPHERAL, channel, notify_data, 4, ATT_HANDLE_AT);
            }
        }
    }
}

void uart_send_data(uint8_t from, uint8_t *data, uint16_t len)
{
    //CS_LOG_DEBUG("%s: from %d, data %s\r\n", __func__, from, bt_hex(data, len));
    if (from == AT_CMD_FROM_AT) {
        uart_send_block(data, len);
    } else if (from == AT_CMD_FROM_BLE) {
        if (ble_data_send_itf != NULL) {
            uint8_t channel = get_channel_by_conn_idx(CS_GAP_ROLE_PHERIPHERAL, at_cmd_ble_conn_idx);
            if (channel != 0xFF) {
                ble_data_send_itf(CS_GAP_ROLE_PHERIPHERAL, channel, data, len, ATT_HANDLE_AT);
            }
        }
    }
}

void uart_slave_rx_data(uint8_t conn_idx, uint8_t handle, const uint8_t *data, uint16_t len)
{
    CS_LOG_DEBUG("%s: conn_idx %d, handle %d, len %d\r\n", __func__, conn_idx, handle, data, len);
    CS_LOG_DEBUG("%s: data(%s)\r\n", __func__, bt_hex(data, len));
    if (handle == ATT_HANDLE_AT) {
        if ((len >= 4) && (memcmp(data, "AT\r\n", 4) == 0)) {
            uart_reply_ok(AT_CMD_FROM_BLE, conn_idx);
        } else if ((len >= 3) && (memcmp(data, "AT+", 3)) == 0) {
            CS_LOG_DEBUG("%s: AT from ble\r\n", __func__);
            at_cmd_ble_conn_idx = conn_idx;
            at_command_process((uint8_t *)data, len, AT_CMD_FROM_BLE);
        } else if ((len == 2) && (memcmp(data, "AT", 2) == 0)) {
            uart_reply_ok(AT_CMD_FROM_BLE, conn_idx);
        } else {
            CS_LOG_DEBUG("%s: data too short\r\n", __func__);
        }
    } else if (handle == ATT_HANDLE_WRITE) {
        // TODO: at bridge function need restruct.
#if (AT_BRIDGE_MODE == 1)
        at_ble_mode_t ble_mode = user_data_get_ble_mode();
        if (ble_mode == AT_BLE_MODE_SLAVE_ONLY ) {
            uart_send_block(data, len);
        } else {
            uint8_t channel = get_channel_by_conn_idx(CS_GAP_ROLE_PHERIPHERAL, conn_idx);
            if ((ble_bridge_handler_intf == NULL) ||
                (ble_bridge_handler_intf(ROLE_MASTER, channel, data, len) < 0)) {
                uart_send_block(data, len);
            }
        }
#else
        uart_send_block(data, len);
#endif
    }
}

void uart_master_rx_data(uint8_t conn_idx, uint8_t handle, const uint8_t *data, uint16_t len)
{
    uint8_t channel = get_channel_by_conn_idx(CS_GAP_ROLE_CENTRAL, conn_idx);
    CS_LOG_DEBUG("%s: data(%s) from handle(%d) at channel(%d)\r\n",
                 __func__, bt_hex(data, len), handle, channel);
    if (channel != 0xFF) {
#if (AT_BRIDGE_MODE == 1)
        uint8_t channel = get_channel_by_conn_idx(CS_GAP_ROLE_CENTRAL, conn_idx);
        if ((ble_bridge_handler_intf == NULL) ||
                (ble_bridge_handler_intf(ROLE_SLAVE, channel, data, len) < 0)) {
            uart_send_block(data, len);
        }
#else
        uart_send_block(data, len);
#endif
    }
}

void set_ble_data_send_interface(ble_data_send_interface intf)
{
    ble_data_send_itf = intf;
}

void set_ble_bridge_handler_interface(ble_bridge_handler_interface intf)
{
    ble_bridge_handler_intf = intf;
}

#endif
