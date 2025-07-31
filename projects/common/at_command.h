/**
 * @file    at_command.h
 * @author  haobo
 * @brief
 * @version 0.1
 * @date    2024-02-04
 * @copyright Copyright (c) 2024, CHIPSEA Co., Ltd.
 * @note
 */
#ifndef AT_COMMAND_H
#define AT_COMMAND_H

#include "../chipsea/cst92f41.h"
#include "uart_process.h"

#define SECTION_AT __attribute((section(".AT_CMD_SECTION")))

#define AT_TABLE_CREATE(name, ...) \
    static const SECTION_AT __USED at_cmd_handle_t at_##name[] = { \
        __VA_ARGS__ \
    };

#define AT_CMD(at_cmd, at_from_ble_enabled, at_cmd_param_num, at_cmc_optional_param_num, at_handler) \
    { \
        .at = (uint8_t *)at_cmd, \
        .at_len = strlen(at_cmd), \
        .at_from_ble_enable = at_from_ble_enabled, \
        .param_num = at_cmd_param_num, \
        .optional_param_num = at_cmc_optional_param_num, \
        .cmd_handle = at_handler, \
    }

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif

// AT version
#define AT_VERSION       136

// AT query option
#define AT_CMD_FIND      0x7F
// AT invalid option
#define AT_CMD_ERROR     0xFF
// AT success option
#define AT_CMD_SUCCESS   0x80

#define ARG_VECTOR_STYPE 1

#if (ARG_VECTOR_STYPE == 1)
#define AT_MAX_ARGS      8
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#if (ARG_VECTOR_STYPE == 1)
typedef void (*at_func)(int argc, char *argv[]);
#else
typedef void (*at_func)(uint8_t* p,uint8_t len);
#endif

typedef void(*parse_callback)(uint8_t index, char *param, uint32_t result[], uint8_t result_len);

typedef enum {
    AT_PROCESS_OK,
    AT_PROCESS_FAILED,
} at_cmd_process_status_t;

typedef enum {
    AT_CMD_FROM_AT,
    AT_CMD_FROM_BLE,
} at_cmd_source;

typedef struct
{
    // AT command
    uint8_t *at;
    // AT command len
    uint8_t at_len;
    // AT command can be from ble.
    uint8_t at_from_ble_enable;
    // AT command param num
    uint8_t param_num;
    // AT command optional param num
    uint8_t optional_param_num;
    // AT command handle function
    at_func cmd_handle;
} at_cmd_handle_t;

uint32_t at_cmd_table_base(void);
uint32_t at_cmd_table_length(void);
at_cmd_process_status_t at_command_process(uint8_t *data, uint8_t len, at_cmd_source cmd_from);
void at_send_data(uint8_t *data, uint8_t len);
void reply_ok(void);
#if (ARG_VECTOR_STYPE == 1)
uint8_t check_and_parse_param(uint8_t *at_cmd, uint8_t at_cmd_len,
        const at_cmd_handle_t *at_handle, int *argc, char*argv[]);
#else
uint8_t check_cmd_param_valid(uint8_t *data, uint8_t len, const at_cmd_handle_t *at_handle);
#endif
char *strstr_wrap(const char *haystack, const char *needle, const char *haystack_end);
void at_parse_param(uint8_t *data, uint8_t len, uint32_t result[], uint16_t result_count);
void at_parse_param_callback(uint8_t *data, uint8_t len,
                             parse_callback callback, uint32_t result[], uint8_t result_len);
uint8_t at_get_cmd_source(void);
int at_parse_int(char *arg, int *val);
int at_parse_hex(char *arg, uint8_t buf[], uint16_t buf_len);

uint16_t uart_data_build_set_byte(uint8_t *buf, uint16_t buf_max_len,
                                  uint16_t dest, uint8_t byte);
uint16_t uart_data_build_set_buffer(uint8_t *buf, uint16_t buf_max_len,
                                    uint16_t dest, uint8_t *data, uint16_t data_len);

#ifdef __cplusplus
}
#endif

#endif /* AT_COMMAND_H */
