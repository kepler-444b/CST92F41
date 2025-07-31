
#include <stddef.h>
#include <string.h>

#include "sdk_config.h"
#include "cs_driver.h"
#include "cs_log.h"
#include "cs_common_utils.h"
#include "nvds.h"
#include "uart_process.h"
#include "user_data.h"
#include "app_adv.h"
#include "app_at.h"
#include "app_conn.h"
#include "evt.h"

#define HI_UINT16(A) (((A) >> 8) & 0xFF)
#define LO_UINT16(A) ((A) & 0xFF)

static evt_timer_t uart_reset_timer;

/**
 * @brief ble uart service uuid
 */
static uint8_t ble_uart_service_uuid[ATT_UUID_SIZE] = {
    LO_UINT16(BLE_UART_SERVICE_UUID),
    HI_UINT16(BLE_UART_SERVICE_UUID),
};

/**
 * @brief ble uart characteristic receive uuid
 */
static uint8_t ble_uart_char_notify_uuid[ATT_UUID_SIZE] = {
    LO_UINT16(BLE_UART_NOTIFY_UUID),
    HI_UINT16(BLE_UART_NOTIFY_UUID),
};

/**
 * @brief ble uart characteristic write uuid
 */
static uint8_t ble_uart_char_write_uuid[ATT_UUID_SIZE] = {
    LO_UINT16(BLE_UART_WRITE_UUID),
    HI_UINT16(BLE_UART_WRITE_UUID),
};

/**
 * @brief ble uart characteristic at uuid
 */
static uint8_t ble_uart_char_at_uuid[ATT_UUID_SIZE] = {
    LO_UINT16(BLE_UART_AT_UUID),
    HI_UINT16(BLE_UART_AT_UUID),
};

/**
 * @brief baudrate of uart
 */
static const uint32_t baud_table[5] = {
    115200, 57600, 38400, 19200, 9600
};

/**
 * @brief advertising interval
 */
static const uint16_t adv_interval_table[9] = {
    160,    // 100ms
    320,    // 200ms
    480,    // 300ms
    640,    // 400ms
    800,    // 500ms
    1600,   // 1000ms
    3200,   // 2000ms
    4800,   // 3000ms
    6400,   // 4000ms
};

/**
 * @brief connect interval
 */
static const uint16_t conn_interval_table[7] = {
    40,     // 50ms
    80,     // 100ms
    160,    // 200ms
    240,    // 300ms
    400,    // 500ms
    800,    // 1000ms
    1200,   // 1500ms
};

/**
 * @brief tx power table
 */
static const uint16_t tx_power_table[5] = {-20, -5, -2, 0, 5};

/**
 * @brief scan window table
 */
static const uint16_t scan_window_table[5] = {
    10,     // 10ms
    20,     // 20ms
    30,     // 30ms
    40,     // 40ms
    50,     // 50ms
};

/**
 * @brief scan interval table. unit(ms).
 */
static const uint16_t scan_interval_table[10] = {
    0,
    100,
    200,
    300,
    400,
    500,
    1000,
    2000,
    3000,
    4000
};

/**
 * @brief current active user data bank index.
 */
static user_data_bank_index_t user_data_bank_index = BANK_INDEX_TOTOL;

/**
 * @brief address of user data bank0 and bank1.
 */
static uint32_t user_data_bank_addr[BANK_INDEX_TOTOL] = {
    0x7A000,
    0x7B000,
};

/**
 * @brief user settings data.
 */
static user_data_t user_data = {0};

static uint8_t user_data_crc32(uint8_t const *data, uint16_t data_len, uint32_t const *p_crc)
{
    uint32_t crc;

    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    for (uint32_t i = 0; i < data_len; i++){
        crc = crc ^ data[i];
        for (uint32_t j = 8; j > 0; j--) crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
    }
    return ~crc;
}

static bool is_user_data_band_index_valid(user_data_bank_index_t index)
{
    if ((index >= BANK_INDEX_A) && (index < BANK_INDEX_TOTOL)) {
        return true;
    }
    return false;
}

static bool user_data_load_from_flash(void)
{
    bool loaded = false;
    uint8_t buf[6];
    uint32_t save_index[BANK_INDEX_TOTOL] = {0xFFFFFFFF, 0xFFFFFFFF};

    CS_LOG_DEBUG("%s: sizeof(user_data): %d, OFFSET_OF(check_sum): %d\r\n",
            __func__, sizeof(user_data), OFFSET_OF(user_data_t, check_sum));

    for (int i = BANK_INDEX_A; i < BANK_INDEX_TOTOL; i++) {
        memset(buf, 0, ARRAY_SIZE(buf));

        drv_sf_read(CS_SF, 0, user_data_bank_addr[i], buf, ARRAY_SIZE(buf));
        CS_LOG_DEBUG("%d: %s\r\n", i, bt_hex(buf, 6));
        if ((buf[0] == 0xAA) && (buf[1] == 0x55)) {
            save_index[i] = (buf[5] << 24) | (buf[4] << 16) | (buf[3] << 8) | buf[2];
        }
        CS_LOG_DEBUG("%d: 0x%8X\r\n", i, save_index[i]);
    }

    if ((save_index[BANK_INDEX_A] != 0xFFFFFFFF) &&
            (save_index[BANK_INDEX_B] != 0xFFFFFFFF)) {
        if (save_index[BANK_INDEX_A] == (save_index[BANK_INDEX_B] + 1)) {
            user_data_bank_index = BANK_INDEX_A;
        } else if (save_index[BANK_INDEX_B] == (save_index[BANK_INDEX_A] + 1)) {
            user_data_bank_index = BANK_INDEX_B;
        }
    } else if (save_index[BANK_INDEX_A] != 0xFFFFFFFF) {
        user_data_bank_index = BANK_INDEX_A;
    } else if (save_index[BANK_INDEX_B] != 0xFFFFFFFF) {
        user_data_bank_index = BANK_INDEX_B;
    }
    CS_LOG_DEBUG("user_data_bank_index: 0x%08x\r\n", user_data_bank_index);

    if (is_user_data_band_index_valid(user_data_bank_index)) {
        CS_LOG_DEBUG("load user data from flash\r\n");
        drv_sf_read(CS_SF, 0, user_data_bank_addr[user_data_bank_index], (void *)&user_data, sizeof(user_data));
        loaded = true;

        if (user_data_crc32((uint8_t const *)&user_data, sizeof(user_data) - 1, NULL) != user_data.check_sum) {
            CS_LOG_DEBUG("check_sum error. expect %d, but %d\r\n",
                    user_data.check_sum, user_data_crc32((uint8_t const *)&user_data, sizeof(user_data) - 1, NULL));
            memset(&user_data, 0, sizeof(user_data));
            for (int i = BANK_INDEX_A; i < BANK_INDEX_TOTOL; i++) {
                drv_sf_erase(CS_SF, 0, user_data_bank_addr[i], 0x1000);
            }

            loaded = false;
        }
    }

    return loaded;
}

void user_data_load_default(void)
{
    user_data.flash_flag = 0x55AA;
    // save_index will plus 1 when save.
    user_data.save_index = 0;
    user_data.ble_mode = AT_BLE_MODE_SLAVE_ONLY;
    user_data.auto_link.auto_link_bits = 0;
    user_data.at_ble_service.uuid_type = UUID_16;
    user_data.scan_interval_index = 1;
    user_data.scan_window_index = 3;
    user_data.tx_power_index = 3;
    user_data.conn_interval_index = 1;
    user_data.adv_interval_index = 4;

    // Init at ble service
    sys_memcpy_swap(user_data.at_ble_service.uuid_16, ble_uart_service_uuid, ATT_BT_UUID_SIZE);
    sys_memcpy_swap(user_data.at_ble_service.char_notify, ble_uart_char_notify_uuid, ATT_BT_UUID_SIZE);
    sys_memcpy_swap(user_data.at_ble_service.char_write, ble_uart_char_write_uuid, ATT_BT_UUID_SIZE);
    sys_memcpy_swap(user_data.at_ble_service.char_at, ble_uart_char_at_uuid, ATT_BT_UUID_SIZE);

    if(nvds_get(NVDS_TAG_BD_ADDRESS,(nvds_tag_len_t *)BD_ADDR_LEN, user_data.mac) != NVDS_OK) {
        cs_gap_addr_get(CS_ADV_ADDR_TYPE_PUBLIC, user_data.mac);
    }

    memcpy(user_data.name, DEFAULT_DEVICE_NAME,
            (strlen((char *)DEFAULT_DEVICE_NAME) > 17) ? 17: strlen((char *)DEFAULT_DEVICE_NAME));

    user_data_save();

    CS_LOG_DEBUG("load user data default!\r\n");
}

void user_data_init(void)
{
    bool loaded = false;
    loaded = user_data_load_from_flash();

    if (!loaded) {
        user_data_load_default();
    } else {
        // we should set mac here.
        CS_LOG_DEBUG("Use user_data.mac as default!\r\n");
        cs_gap_addr_set(0, user_data.mac);

        // set user_data.name as default
        extern void app_adv_set_name(char *name);
        app_adv_set_name((char *)user_data.name);

        // set the default adv interval
        app_adv_set_interval(adv_interval_table[user_data.adv_interval_index]);

        // set the default conn interval
        app_conn_set_interval(conn_interval_table[user_data.conn_interval_index]);
    }

    CS_LOG_DEBUG("Re-init the uart with baudrate %d\r\n", baud_table[user_data.baud_index]);
    // Re-init the uart with the user data setting. 
    at_uart_init(baud_table[user_data.baud_index]);

    // When reset, send 'OK' to AT usart.
    uart_send_block((uint8_t *)"OK\r\n", 4);

    user_data_show();

    /**
     * The user_data_t is a packed struct. To avoid the unaligned pointer issue,
     * allocate an align variable on stack, and past the pointer to visit.
     */
    at_ble_auto_link_info_t __ALIGNED(4) auto_link;
    auto_link.auto_link_bits = user_data.auto_link.auto_link_bits;
    memcpy(auto_link.peer_addr, user_data.auto_link.peer_addr, sizeof(auto_link.peer_addr));
    auto_link_info_init(&auto_link);
}

void user_data_show(void)
{
    CS_LOG_DEBUG("Flash flag: 0x%04x\r\n", user_data.flash_flag);
    CS_LOG_DEBUG("Flash check_sum: 0x%04x\r\n", user_data.check_sum);
    CS_LOG_DEBUG("Scan window: %d\r\n", scan_window_table[user_data.scan_window_index]);
    CS_LOG_DEBUG("Scan interval: %d\r\n", scan_interval_table[user_data.scan_interval_index]);
    CS_LOG_DEBUG("Uart baudrate: %d\r\n", baud_table[user_data.baud_index]);
    CS_LOG_DEBUG("Advertise interval: %d\r\n", adv_interval_table[user_data.adv_interval_index]);
    CS_LOG_DEBUG("Connect interval: %d\r\n", conn_interval_table[user_data.conn_interval_index]);
    CS_LOG_DEBUG("Tx power: %d\r\n", tx_power_table[user_data.tx_power_index]);
    CS_LOG_DEBUG("Device name: %s\r\n", user_data.name);
    CS_LOG_DEBUG("Ble mode: %d\r\n", user_data.ble_mode);
    CS_LOG_DEBUG("Device mac: %02X%02X%02X%02X%02X%02X\r\n",
                    user_data.mac[5],
                    user_data.mac[4],
                    user_data.mac[3],
                    user_data.mac[2],
                    user_data.mac[1],
                    user_data.mac[0]);
    CS_LOG_DEBUG("Autolink: %d\r\n", user_data.auto_link.auto_link_bits);
    CS_LOG_DEBUG("Ble service uuid type: %d\r\n", user_data.at_ble_service.uuid_type);
    CS_LOG_DEBUG("Ble service uuid16: %s\r\n", bt_hex(user_data.at_ble_service.uuid_16, 2));
    CS_LOG_DEBUG("Ble characteristic notify: %s\r\n", bt_hex(user_data.at_ble_service.char_notify, 2));
    CS_LOG_DEBUG("Ble characteristic write: %s\r\n", bt_hex(user_data.at_ble_service.char_write, 2));
    CS_LOG_DEBUG("Ble characteristic at: %s\r\n", bt_hex(user_data.at_ble_service.char_at, 2));
    CS_LOG_DEBUG("Ble service uuid128: %s\r\n", bt_hex(user_data.at_ble_service.uuid_128, 16));
}

void user_data_save(void)
{
    uint8_t save_count = 0;
    user_data_t tmp_data;
    user_data_bank_index = (user_data_bank_index + 1) % BANK_INDEX_TOTOL;

    user_data.flash_flag = 0x55AA;

    user_data.save_index++;
    if (user_data.save_index == 0xFFFFFFFF) {
        user_data.save_index = 1;
    }

    user_data.check_sum = user_data_crc32((uint8_t const *)&user_data, sizeof(user_data) - 1, NULL);

    while (true) {
        save_count++;
        if (save_count > 3) break;

        drv_sf_erase(CS_SF, 0, user_data_bank_addr[user_data_bank_index], 0x1000);
        drv_sf_write(CS_SF, 0, user_data_bank_addr[user_data_bank_index], (uint8_t *)&user_data, sizeof(user_data));

        // Check.
        memset(&tmp_data, 0, sizeof(tmp_data));
        drv_sf_read(CS_SF, 0, user_data_bank_addr[user_data_bank_index], (void *)&tmp_data, sizeof(tmp_data));
        if (memcmp(&user_data, &tmp_data, sizeof(user_data_t)) == 0) {
            break;
        }
    }

    if (save_count > 3) {
        CS_LOG_DEBUG("User data save failed!\r\n");
    }
}

void user_data_restore(void)
{
    for (int i = BANK_INDEX_A; i < BANK_INDEX_TOTOL; i++) {
        drv_sf_erase(CS_SF, 0, user_data_bank_addr[i], 0x1000);
    }
}

static void uart_reset_handler(evt_timer_t *timer, void *param)
{
    // Re-init the uart with the user data setting.
    at_uart_init(baud_table[user_data.baud_index]);
}

bool user_data_baud_index_valid(uint8_t baud_index)
{
    if (baud_index < ARRAY_SIZE(baud_table)) {
        return true;
    }

    return false;
}

uint8_t user_data_get_baud_index(void)
{
    return user_data.baud_index;
}

void user_data_set_baud_index(uint8_t baud_index)
{
    user_data.baud_index = baud_index;

    user_data_save();

    evt_timer_set(&uart_reset_timer, 100, EVT_TIMER_ONE_SHOT, uart_reset_handler, NULL);
}

void user_data_get_mac(bd_addr_t *mac)
{
    memcpy(mac->addr, user_data.mac, BD_ADDR_LEN);
}

void user_data_set_mac(bd_addr_t *mac)
{
    memcpy(user_data.mac, mac->addr, BD_ADDR_LEN);
    cs_gap_addr_set(CS_ADV_ADDR_TYPE_PUBLIC, user_data.mac);

    user_data_save();
}

int8_t user_data_set_ble_mode(at_ble_mode_t mode)
{
    if ((mode >= AT_BLE_MODE_SLAVE_ONLY) &&
        (mode <= AT_BLE_MODE_MASTER_AND_SLAVE)) {
        user_data.ble_mode = mode;

        user_data_save();
        return 0;
    }

    return -1;
}

at_ble_mode_t user_data_get_ble_mode()
{
    return user_data.ble_mode;
}

int8_t user_data_set_adv_interval(uint8_t interval_index)
{
    if ((interval_index >=0) && (interval_index < ARRAY_SIZE(adv_interval_table))) {
        user_data.adv_interval_index = interval_index;
        user_data_save();

        return 0;
    }

    return -1;
}

uint8_t user_data_get_adv_interval(uint16_t *interval)
{
    if (interval != NULL) {
        *interval = adv_interval_table[user_data.adv_interval_index];
    }
    return user_data.adv_interval_index;
}

int8_t user_data_set_conn_interval(uint8_t interval_index)
{
    if ((interval_index >= 0) && (interval_index < ARRAY_SIZE(conn_interval_table))) {
        user_data.conn_interval_index = interval_index;
        user_data_save();

        return 0;
    }

    return -1;
}

uint8_t user_data_get_conn_interval(uint16_t *interval)
{
    if (interval != NULL) {
        *interval = conn_interval_table[user_data.conn_interval_index];
    }
    return user_data.conn_interval_index;
}

int8_t user_data_set_tx_power(uint8_t power_index)
{
    if ((power_index >= 0) && (power_index < ARRAY_SIZE(tx_power_table))) {
        user_data.tx_power_index = power_index;
        user_data_save();

        return 0;
    }

    return -1;
}

uint8_t user_data_get_tx_power(uint16_t *power)
{
    if (power != NULL) {
        *power = tx_power_table[user_data.tx_power_index];
    }

    return user_data.tx_power_index;
}

void user_data_set_name(char *name)
{
    uint8_t copy_len = (strlen(name) > sizeof(user_data.name)/sizeof(user_data.name[0]) - 1) ? 
                       sizeof(user_data.name)/sizeof(user_data.name[0]) : strlen(name);
    memcpy(user_data.name, name, copy_len);
    user_data.name[copy_len] = '\0';

    user_data_save();
}

uint8_t *user_data_get_name(void)
{
    return user_data.name;
}

bool user_data_scan_window_interval_valid(uint8_t scan_window_index, uint8_t scan_interval_index )
{
    if (((scan_window_index >= 0) && (scan_window_index < ARRAY_SIZE(scan_window_table))) &&
        ((scan_interval_index >= 0) && (scan_interval_index < ARRAY_SIZE(scan_interval_table)))) {
        return true;
    }

    return false;
}

int8_t user_data_set_scan_window(uint8_t scan_window_index)
{
    if ((scan_window_index >= 0) && (scan_window_index < ARRAY_SIZE(scan_window_table))) {
        user_data.scan_window_index = scan_window_index;

        user_data_save();
        return 0;
    }

    return -1;
}

uint8_t user_data_get_scan_window(uint16_t *scan_window)
{
    if (scan_window) {
        *scan_window = scan_window_table[user_data.scan_window_index];
    }

    return user_data.scan_window_index;
}

int8_t user_data_set_scan_interval(uint8_t scan_interval_index)
{
    if ((scan_interval_index >= 0) && (scan_interval_index < ARRAY_SIZE(scan_interval_table))) {
        user_data.scan_interval_index = scan_interval_index;

        user_data_save();

        return 0;
    }

    return -1;
}

uint8_t user_data_get_scan_interval(uint16_t *scan_interval)
{
    if (scan_interval) {
        *scan_interval = scan_window_table[user_data.scan_interval_index];
    }

    return user_data.scan_interval_index;
}

void user_data_update_autolink(uint8_t channel, uint8_t enable, cs_gap_addr_t *addr)
{
    if (enable) {
        user_data.auto_link.auto_link_bits |= (1 << channel);
    } else {
        user_data.auto_link.auto_link_bits &= ~(1 << channel);
    }
    memcpy(&user_data.auto_link.peer_addr[channel], addr, sizeof(cs_gap_addr_t));

    user_data_save();
}

uint32_t user_data_get_autolink_bits(void)
{
    return user_data.auto_link.auto_link_bits;
}

void user_data_get_ble_service(at_ble_service_t *service)
{
    if (service == NULL) {
        CS_LOG_DEBUG("NULL pointer\r\n");
        return;
    }

    service->uuid_type = user_data.at_ble_service.uuid_type;
    memcpy(service->uuid_16, user_data.at_ble_service.uuid_16, sizeof(service->uuid_16));
    memcpy(service->char_notify, user_data.at_ble_service.char_notify, sizeof(service->char_notify));
    memcpy(service->char_write, user_data.at_ble_service.char_write, sizeof(service->char_write));
    memcpy(service->char_at, user_data.at_ble_service.char_at, sizeof(service->char_at));
    if (user_data.at_ble_service.uuid_type == UUID_128) {
        memcpy(service->uuid_128, user_data.at_ble_service.uuid_128, sizeof(service->uuid_128));
    }
}

void user_data_set_ble_service(at_ble_service_t *service)
{
    if (service == NULL) {
        CS_LOG_DEBUG("NULL pointer\r\n");
        return;
    }

    user_data.at_ble_service.uuid_type = service->uuid_type; 
    memcpy(user_data.at_ble_service.uuid_16, service->uuid_16, sizeof(service->uuid_16));
    memcpy(user_data.at_ble_service.char_notify, service->char_notify, sizeof(service->char_notify));
    memcpy(user_data.at_ble_service.char_write, service->char_write, sizeof(service->char_write));
    memcpy(user_data.at_ble_service.char_at, service->char_at, sizeof(service->char_at));
    if (user_data.at_ble_service.uuid_type == UUID_128) {
        memcpy(user_data.at_ble_service.uuid_128, service->uuid_128, sizeof(service->uuid_128));
    }

    user_data_save();
}

int8_t user_data_get_bridge(uint8_t channel, at_bridge_t *p_bridge)
{
    if ((channel < AT_BRIDGE_NUM) && (p_bridge != NULL)) {
        p_bridge->bridge_in_role = user_data.bridge[channel].bridge_in_role;
        p_bridge->bridge_in_channel = user_data.bridge[channel].bridge_in_channel;
        p_bridge->bridge_out_role = user_data.bridge[channel].bridge_out_role;
        p_bridge->bridge_out_channel = user_data.bridge[channel].bridge_out_channel;
        p_bridge->bridge_enable = user_data.bridge[channel].bridge_enable;

        return 0;
    }

    return -1;
}

int8_t user_data_set_bridge(uint8_t channel, at_bridge_t *p_bridge)
{
    if ((channel < AT_BRIDGE_NUM) && (p_bridge != NULL)) {
        user_data.bridge[channel].bridge_in_role = p_bridge->bridge_in_role;
        user_data.bridge[channel].bridge_in_channel = p_bridge->bridge_in_channel;
        user_data.bridge[channel].bridge_out_role = p_bridge->bridge_out_role;
        user_data.bridge[channel].bridge_out_channel = p_bridge->bridge_out_channel;
        user_data.bridge[channel].bridge_enable = p_bridge->bridge_enable;

        user_data_save();

        return 0;
    }

    return -1;
}

bool user_data_check_bridge_path_exist(uint8_t dst_channel, at_bridge_t *p_bridge)
{
    if (p_bridge == NULL) {
        return false;
    }

    for (int channel = 0; channel < AT_BRIDGE_NUM; channel++) {
        if (channel == dst_channel) continue;

        if ((user_data.bridge[channel].bridge_in_role == p_bridge->bridge_in_role) && 
            (user_data.bridge[channel].bridge_in_channel == p_bridge->bridge_in_channel)) {
            return true;
        }

        if ((user_data.bridge[channel].bridge_out_role == p_bridge->bridge_out_role) && 
            (user_data.bridge[channel].bridge_out_channel == p_bridge->bridge_out_channel)) {
            return true;
        }

        if ((user_data.bridge[channel].bridge_in_role == p_bridge->bridge_out_role) && 
            (user_data.bridge[channel].bridge_in_channel == p_bridge->bridge_out_channel)) {
            return true;
        }

        if ((user_data.bridge[channel].bridge_out_role == p_bridge->bridge_in_role) && 
            (user_data.bridge[channel].bridge_out_channel == p_bridge->bridge_in_channel)) {
            return true;
        }
    }

    return false;
}

void at_uart_restore(void)
{
    at_uart_init(baud_table[user_data.baud_index]);
}

