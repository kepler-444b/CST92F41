#ifndef _USER_DATA
#define _USER_DATA

#include <stdint.h>
#include <stdbool.h>
#include "../chipsea/cst92f41.h"
#include "cs_ble_defines.h"
#include "cs_ble_gap.h"
#include "sdk_config.h"

#define AT_BRIDGE_NUM   5

/* 16bits uuid size*/
#define ATT_BT_UUID_SIZE    2

/* 128bits uuid size*/
#define ATT_UUID_SIZE       16

/* Simple Profile Service UUID*/
#define BLE_UART_SERVICE_UUID   0xFFF0

/* Ble uart receive uuid*/
#define BLE_UART_NOTIFY_UUID    0xFFF1

/* Ble uart write uuid*/
#define BLE_UART_WRITE_UUID     0xFFF2

/* Ble uart at receive uuid*/
#define BLE_UART_AT_UUID        0xFFF3

typedef enum {
    BANK_INDEX_A = 0,
    BANK_INDEX_B,
    BANK_INDEX_TOTOL,
} user_data_bank_index_t;

typedef enum {
    AT_BLE_MODE_SLAVE_ONLY = 0,
    AT_BLE_MODE_MASTER_ONLY,
    AT_BLE_MODE_MASTER_AND_SLAVE,
} at_ble_mode_t;

typedef enum {
    UUID_16 = 0,    // 16bits uuid
    UUID_128,       // 128bits uuid
} at_ble_uuid_type_t;

typedef enum {
    ROLE_SLAVE = 0x01,
    ROLE_MASTER = 0x02,
} at_ble_role_type_t;

typedef struct {
    uint8_t bridge_in_role;     //@ref at_ble_role_type_t
    uint8_t bridge_in_channel;  // channel of in role
    uint8_t bridge_out_role;    //@ref at_ble_role_type_t
    uint8_t bridge_out_channel; // channel of out role
    uint8_t bridge_enable;      // if bridge enabled
} at_bridge_t;

typedef struct {
    uint8_t uuid_type;
    uint8_t uuid_16[ATT_BT_UUID_SIZE];
    uint8_t uuid_128[ATT_UUID_SIZE];
    uint8_t char_notify[ATT_BT_UUID_SIZE];
    uint8_t char_write[ATT_BT_UUID_SIZE];
    uint8_t char_at[ATT_BT_UUID_SIZE];
} at_ble_service_t;

typedef struct {
    uint32_t  auto_link_bits;
    cs_gap_addr_t peer_addr[CS_LE_MAX_SLAVE_NUM];
} at_ble_auto_link_info_t;

typedef struct {
    uint16_t flash_flag;
    uint32_t save_index;
    uint8_t  baud_index;
    uint8_t  adv_interval_index;
    uint8_t  conn_interval_index;
    uint8_t  tx_power_index;
    uint8_t  scan_window_index;
    uint8_t  scan_interval_index;
    at_ble_service_t at_ble_service;
    at_ble_auto_link_info_t auto_link;
    at_bridge_t bridge[AT_BRIDGE_NUM];
    uint8_t  ble_mode;
    uint8_t  ble_data_direction; // only usable when ble_mode is AT_BLE_MODE_MASTER_AND_SLAVE
    uint8_t  name[BT_DEVICE_NAME_MAX_LEN];
    uint8_t  mac[6];
    uint8_t  check_sum;
} __PACKED user_data_t;

void user_data_init(void);
void user_data_save(void);
void user_data_show(void);
void user_data_restore(void);
bool user_data_baud_index_valid(uint8_t baud_index);
uint8_t user_data_get_baud_index(void);
void user_data_set_baud_index(uint8_t baud_index);
void user_data_get_mac(bd_addr_t *mac);
void user_data_set_mac(bd_addr_t *mac);
int8_t user_data_set_ble_mode(at_ble_mode_t mode);
at_ble_mode_t user_data_get_ble_mode();
int8_t user_data_set_adv_interval(uint8_t interval_index);
uint8_t user_data_get_adv_interval(uint16_t *interval);
int8_t user_data_set_conn_interval(uint8_t interval_index);
uint8_t user_data_get_conn_interval(uint16_t *interval);
int8_t user_data_set_tx_power(uint8_t power_index);
uint8_t user_data_get_tx_power(uint16_t *power);
void user_data_set_name(char *name);
uint8_t *user_data_get_name(void);
bool user_data_scan_window_interval_valid(uint8_t scan_window_index, uint8_t scan_interval_index );
int8_t user_data_set_scan_window(uint8_t scan_window_index);
uint8_t user_data_get_scan_window(uint16_t *scan_window);
int8_t user_data_set_scan_interval(uint8_t scan_interval_index);
uint8_t user_data_get_scan_interval(uint16_t *scan_interval);
void user_data_update_autolink(uint8_t channel, uint8_t enable, cs_gap_addr_t *addr);
uint32_t user_data_get_autolink_bits(void);
void user_data_get_ble_service(at_ble_service_t *service);
void user_data_set_ble_service(at_ble_service_t *service);
int8_t user_data_get_bridge(uint8_t channel, at_bridge_t *p_bridge);
int8_t user_data_set_bridge(uint8_t channel, at_bridge_t *p_bridge);
bool user_data_check_bridge_path_exist(uint8_t dst_channel, at_bridge_t *p_bridge);
void at_uart_restore(void);
#endif
