#ifndef _APP_24G_H_
#define _APP_24G_H_

typedef enum {
    RADIO_MODE_RX = 0, // 接收模式(默认)
    RADIO_MODE_TX      // 发送模式
} radio_mode_t;

static radio_mode_t current_radio_mode = RADIO_MODE_RX; // 当前模式

void switch_radio_mode(radio_mode_t new_mode);
static void app_24g_tx(uint8_t *data, uint8_t length);

#endif