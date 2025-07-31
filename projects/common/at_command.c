/**
 * @file    at_command.c
 * @author  haobo
 * @brief
 * @version 0.1
 * @date    2024-02-04
 * @copyright Copyright (c) 2024, CHIPSEA Co., Ltd.
 * @note
 *
 * Param parse example 1:
 *   For case all params are integer, like the following cmd:
 *      AT+SAMPLE1:<12>,<32>,<56>
 *
 *   At command table item:
 *      AT_CMD("+SAMPLE1", 3, at_sample1),
 *
 *   void at_sample1(uint8_t *data,uint8_t len) {
 *      uint32_t param[3] = {0};
 *      uint8_t enable;
 *      uint16_t index;
 *      uint32_t adv_interval;
 *
 *      at_parse_param(data, len, param, ARRAY_SIZE(param));
 *      enable = param[0];
 *      index = param[1];
 *      adv_interval = param[2];
 *
 *      sample1_handle(enable, index, adv_interval);
 *   }
 *
 * Param parse example 2:
 *   For case exist special param, such as MAC addr, or UUID.
 *      AT+SAMPLE2:<111>,<112233445566778899AABBCCDDEEFFEE>
 *
 *   At command table item:
 *      AT_CMD("+SAMPLE2", 2, at_sample2),
 *
 *   void at_sample2_callback(uint8_t index, char *param, uint32_t result[], uint8_t result_len) {
 *      char *endptr;
 *      if (index == 0) {
 *          if ((param[0] == '0') && ((param[1] = 'x') || (param[1] = 'X'))) {
 *              result[index] = strtol(&param[2], &endptr, 16);
 *          } else {
 *              result[index] = strtol(param, &endptr, 10);
 *          }
 *      } else if (index == 1) {
 *          uint8_t len = strlen(param);
 *          uint8_t *uuid = (uint8_t *)&result[index];
 *          if (len != 32) {
 *              log_debug("Invalid UUID length!");
 *              return;
 *          }
 *
 *          for (int i = 0; i < (len >> 1); i++) {
 *              uuid[i] = (CHAR_TO_OCTECT(param[i * 2]) << 4) | (CHAR_TO_OCTECT(param[i * 2 + 1]));
 *          }
 *      }
 *   }
 *
 *   void at_sample2(uint8_t *data,uint8_t len) {
 *      // param[0] for "111"
 *      // param[1~4] for the 16 bytes UUID.
 *      uint32_t param[5] = {0};
 *      uint16_t tag;
 *      uint8_t *uuid;
 *
 *      at_parse_param_callback(data, len, at_sample2_callback, param, ARRAY_SIZE(param));
 *      tag = param[0]
 *      uuid = (uint8_t *)&param[1];
 *
 *      sample2_handle(tag, uuid);
 *   }
 */
/*********************************************************************
 * INCLUDES
 */
#include <stdlib.h>
#include <string.h>

#include "sdk_config.h"
#include "cs_common_utils.h"
#include "at_command.h"
#include "uart_process.h"

#ifdef AT_ENABLE

static at_cmd_source at_cmd_from = AT_CMD_FROM_AT;
/*
#define CHAR_TO_OCTECT(ch) \
(   \
    ((ch >= '0') && (ch <= '9')) ? (ch - '0') : \
    ((ch >= 'a') && (ch <= 'f')) ? (ch - 'a' + 10) : \
    ((ch >= 'A') && (ch <= 'F')) ? (ch - 'A' + 10) : 0x2A \
)
*/

uint32_t at_cmd_table_base(void)
{
#if defined ( __CC_ARM )
    extern uint32_t Image$$AT_CMD_SECTION$$Base[];
    return (uint32_t )(Image$$AT_CMD_SECTION$$Base);
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    extern uint32_t Image$$AT_CMD_SECTION$$Base[];
    return (uint32_t )(Image$$AT_CMD_SECTION$$Base);
#elif defined ( __GNUC__ )
    extern uint32_t __AT_TABLE_BASE[];
    return (uint32_t)__AT_TABLE_BASE;
#endif
}

uint32_t at_cmd_table_length(void)
{
    uint32_t table_length = 0;
#if defined ( __CC_ARM )
    extern uint32_t Image$$AT_CMD_SECTION$$Length[];
    table_length = (uint32_t )(Image$$AT_CMD_SECTION$$Length);
#elif defined (__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    extern uint32_t Image$$AT_CMD_SECTION$$Length[];
    table_length = (uint32_t )(Image$$AT_CMD_SECTION$$Length);
#elif defined ( __GNUC__ )
    extern uint32_t __AT_TABLE_BASE[];
    extern uint32_t __AT_TABLE_END[];
    table_length = (uint32_t)((uint32_t)__AT_TABLE_END - (uint32_t)__AT_TABLE_BASE);
#endif

    table_length = table_length / sizeof(at_cmd_handle_t);
    return table_length;
}

const at_cmd_handle_t *find_at_command_handle(uint8_t *at_cmd, uint8_t at_cmd_len)
{
    at_cmd_handle_t *at_table = (at_cmd_handle_t *)at_cmd_table_base();
    uint32_t at_table_len = at_cmd_table_length();

    for (int i = 0; i < at_table_len; i++) {
        if ((at_cmd_len == at_table[i].at_len)
                && (memcmp((at_cmd+2), at_table[i].at, at_table[i].at_len) == 0)) {
            //log_debug("%s: %d\r\n", __func__, i);
            return &at_table[i];
        }
    }
    return NULL;
}

void print_at_cmd(uint8_t *cmd, uint8_t cmd_len)
{
    for (int i = 0 ; i < cmd_len ; i++) {
        log_debug("%c",*(cmd+i));
    }
    log_debug("\r\n");
}

at_cmd_process_status_t at_command_process(uint8_t *data, uint8_t len, at_cmd_source cmd_from)
{
    const uint8_t ori_len = len;
    at_func at_cb;
    uint8_t ret = 0;

    if ((data[len - 2] == '\r')
            && (data[len - 1] == '\n')) {
        len -= 2;
    }

    // Print the at command.
    //print_at_cmd(data, len);

    char *p = (char *)data, *p_end = (char *)data + len, *q = NULL;
    char *colon = NULL;
    const at_cmd_handle_t *at_cmd_handle;
    uint16_t cmd_len = 0;
    uint16_t cmd_name_len = 0;
#if (ARG_VECTOR_STYPE == 1)
    int argc = 0;
    char *argv[AT_MAX_ARGS + 1] = {0};
#endif

    p = strstr(p, "AT+");
    while (p != NULL) {
        q = strstr(p + 1, "AT+");

        if (q != NULL) {
            cmd_len = q - p;

            colon = strstr_wrap(p + 1, ":", q);
            if (colon != NULL) {
                cmd_name_len = colon - p;
            } else {
                cmd_name_len = q - p;
            }
        } else {
            cmd_len = p_end - p;

            colon = strstr(p + 1, ":");
            if (colon != NULL) {
                cmd_name_len = colon - p;
            } else {
                cmd_name_len = p_end - p;
            }
        }

        cmd_name_len -= 2; // "AT"

        //log_debug("cmd_len: %d, cmd_name_len %d\r\n", cmd_len, cmd_name_len);
        print_at_cmd((uint8_t *)p, cmd_len);
        at_cmd_handle = find_at_command_handle((uint8_t *)p, cmd_name_len);
        //log_debug("%s: command handle %p\n", __func__, at_cmd_handle );
        if (at_cmd_handle != NULL) {
            // handle the cmd from p to p+cmd_len
#if (ARG_VECTOR_STYPE == 1)
            ret = check_and_parse_param((uint8_t *)p, cmd_len, at_cmd_handle, &argc, argv);
#else
            ret = check_cmd_param_valid((uint8_t *)p, cmd_len, at_cmd_handle);
#endif
            if (ret == AT_CMD_ERROR) {
                return AT_PROCESS_FAILED;
            }

            if ((cmd_from == AT_CMD_FROM_BLE) &&
                    (at_cmd_handle->at_from_ble_enable == false)) {
                log_debug("AT cmd can not from BLE\r\n");
                return AT_PROCESS_FAILED;
            }

            // Call the at command handle.
#if (ARG_VECTOR_STYPE == 1)
            //log_debug("----------\r\n");
            //for (int i = 0; i < argc; i++) {
            //    log_debug("%d: %s\r\n", i, argv[i]);
            //}
            //log_debug("----------\r\n");

            at_cmd_from = cmd_from;

            at_cmd_handle->cmd_handle(argc, argv);
#else
            at_cmd_handle->cmd_handle((uint8_t *)p, cmd_len);
#endif
        } else {
            return AT_PROCESS_FAILED;
        }

        p = q;
    }

    return AT_PROCESS_OK;
}

// Reply data
void at_send_data(uint8_t *data, uint8_t len)
{
    uart_send_data(at_cmd_from, data, len);
}

#if (ARG_VECTOR_STYPE == 1)
uint8_t check_and_parse_param(uint8_t *at_cmd, uint8_t at_cmd_len,
        const at_cmd_handle_t *at_handle, int *argc, char*argv[])
{
    char *p = (char *)at_cmd, *q = NULL;
    char *p_next = NULL, *p_end = (char *)at_cmd + at_cmd_len - 1;

    p = strstr_wrap(p, ":", p_end);
    if ((p == NULL)) {
        // No param, return success.
        *argc = 0;
        return AT_CMD_SUCCESS;
    }

    if ('<' != *(p+1)) {
        log_debug("%s: ':' NOT followed by '<'!\n", __func__);
        return AT_CMD_ERROR;
    }

    p = p + 1;
    //log_debug("p: %d\r\n", p - (char *)at_cmd);
    while ((p = strstr_wrap(p, "<", p_end)) != NULL) {
        p_next = strstr_wrap(p + 1, "<", p_end);
        q = strstr_wrap(p, ">", p_end);

        //log_debug("p: %p,  %d\r\n", p, p - (char *)at_cmd);
        //log_debug("q: %p, %d\r\n", q, q - (char *)at_cmd);
        if ((q != NULL) && (q > p) && ((p_next == NULL) || (p_next > q))) {
            // '>' should followd by ','
            if ((p_next != NULL) && (*(q + 1) != ',')) {
                log_debug("%s: '>' NOT followed by ','!\n", __func__);
                return AT_CMD_ERROR;
            }

            if (*argc > AT_MAX_ARGS) {
                log_debug("Param number (%d) exceed limit (%d)\r\n", (*argc + 1), AT_MAX_ARGS);
                return AT_CMD_ERROR;
            }

            argv[*argc] = p + 1;
            *argc = *argc + 1;

            /*
             * Change the '>' to '\0' to terminate the corresponding argv.
             */
            *q='\0';
            p = q + 1;
        } else {
            p++;
        }
    }

    //log_debug("*argc %d\r\n", *argc);
    if ((*argc != at_handle->param_num) &&
            (*argc != (at_handle->param_num + at_handle->optional_param_num))) {
        log_debug("%s: param num not matched for at command!\n", __func__);
        return AT_CMD_ERROR;
    }

    return AT_CMD_SUCCESS;
}

#else

// Check if the at command is valid
uint8_t check_cmd_param_valid(uint8_t *at_cmd, uint8_t at_cmd_len, const at_cmd_handle_t *at_handle)
{
    char *p = (char *)at_cmd, *q = NULL;
    char *p_next = NULL, *p_end = (char *)at_cmd + at_cmd_len - 1;
    int param_count = 0;

    p = strstr_wrap(p, ":", p_end);

    if ((p == NULL)) {
        // No param, return success.
        return AT_CMD_SUCCESS;
    }

    if ('<' != *(p+1)) {
        log_debug("%s: ':' NOT followed by '<'!\n", __func__);
        return AT_CMD_ERROR;
    }

    p = p + 1;
    while ( p!= NULL ) {
        p_next = strstr_wrap(p+1, "<", p_end);
        q = strstr_wrap(p, ">", p_end);
        if ((q != NULL) && (q > p) && ((p_next == NULL) || (p_next > q))) {
            param_count++;
        }
        p = p_next;
    }

    if ((param_count != at_handle->param_num) &&
            (param_count != (at_handle->param_num + at_handle->optional_param_num))) {
        log_debug("%s: param num not matched for at command!\n", __func__);
        return AT_CMD_ERROR;
    }

    return AT_CMD_SUCCESS;
}
#endif

char *strstr_wrap(const char *haystack, const char *needle, const char *haystack_end)
{
    char *p = strstr(haystack, needle);
    if (p > haystack_end) p = NULL;

    return p;
}

void at_parse_param(uint8_t *data, uint8_t len, uint32_t result[], uint16_t result_count)
{
    char *p = (char *)data, *q = NULL;
    char *endptr;
    char tmp[32];
    int param_index = 0;

    while ((p = strstr(p, "<")) != NULL) {
        q = strstr(p, ">");
        if ((q != NULL) && (q > p)) {
            memset(tmp, 0, sizeof(tmp)/sizeof(tmp[0]));
            memcpy(tmp, p+1, q - p - 1);
            if (param_index >= result_count) {
                log_debug("Insufficient result memory\r\n");
                return;
            }
            if ((tmp[0] == '0') && ((tmp[1] = 'x') || (tmp[1] = 'X'))) {
                result[param_index++] = strtol(&tmp[2], &endptr, 16);
            } else {
                result[param_index++] = strtol(tmp, &endptr, 10);
            }
        }
        p++;
    }
}

void at_parse_param_callback(uint8_t *data, uint8_t len,
        parse_callback callback, uint32_t result[], uint8_t result_len)
{
    char *p = (char *)data, *q = NULL;
    char tmp[256];
    int param_index = 0;

    while ((p = strstr(p, "<")) != NULL) {
        q = strstr(p, ">");
        if ((q != NULL) && (q > p)) {
            memset(tmp, 0, sizeof(tmp)/sizeof(tmp[0]));
            memcpy(tmp, p+1, q - p - 1);
            log_debug("%s: %s\r\n", __func__, tmp);
            callback(param_index++, tmp, result, result_len);
        }
        p++;
    }
}

uint8_t at_get_cmd_source(void)
{
    return at_cmd_from;
}

int at_parse_int(char *arg, int *val)
{
    char *endptr;
    int result = 0;

    if (val == NULL) {
        return -1;
    }

    if (strlen(arg) > 2) {
        if ((arg[0] == '0') && ((arg[1] = 'x') || (arg[1] = 'X'))) {
            result = strtol(&arg[2], &endptr, 16);
        } else {
            result = strtol(arg, &endptr, 10);
        }
    } else {
        result = strtol(arg, &endptr, 10);
    }

    if ((endptr - arg) != strlen(arg)) {
        return -1;
    }
    *val = result;
    return 0;
}

int at_parse_hex(char *arg, uint8_t buf[], uint16_t buf_len)
{
    uint8_t len = strlen(arg);
    if (len != 2 * buf_len) {
        log_debug("Invalid arg length!\r\n");
        return -1;
    }

    for (int i = 0; i < (len >> 1); i++) {
        buf[i] = (CHAR_TO_OCTECT(arg[i * 2]) << 4) | (CHAR_TO_OCTECT(arg[i * 2 + 1]));
    }
    return 0;
}

/**
 * Uart data build utilities.
 ******************************************************************************/
uint16_t uart_data_build_set_byte(uint8_t *buf, uint16_t buf_max_len,
                                  uint16_t dest, uint8_t byte)
{
    if (dest + 1 > buf_max_len) {
        log_debug("buf overflow!\r\n");
        return dest;
    }

    buf[dest] = byte;
    dest += 1;

    return dest;
}

uint16_t uart_data_build_set_buffer(uint8_t *buf, uint16_t buf_max_len,
                                    uint16_t dest, uint8_t *data, uint16_t data_len)
{
    if (dest + data_len > buf_max_len) {
        log_debug("buf overflow!\r\n");
        return dest;
    }

    memcpy(buf+dest, data, data_len);
    dest += data_len;

    return dest;
}

#if 0
///////////////////////////////////////// common AT command
#include "cs_driver.h"

// Reply ok
void reply_ok(void)
{
    at_send_data((uint8_t *)"OK\r\n", 4);
}

// Reset chip
void at_rst(uint8_t *data,uint8_t len)
{
    reply_ok();
    drv_pmu_force_reboot();
}

// Get version of at
void at_ver(uint8_t *data,uint8_t len)
{
    uint8_t temp_data[10];
    temp_data[0] = 'V';
    temp_data[1] = 'E';
    temp_data[2] = 'R';
    temp_data[3] = ':';
    temp_data[4] = 'V';
    temp_data[5] = (AT_VERSION / 100 % 10) + 0x30;
    temp_data[6] = (AT_VERSION / 10 % 10) + 0x30;
    temp_data[7] = (AT_VERSION / 1 % 10) + 0x30;
    temp_data[8] = '\r';
    temp_data[9] = '\n';

    at_send_data(temp_data, 10);
}

AT_TABLE_CREATE(base_at_table,
    /**
     * Reset chip
     * AT+RST
     * Example:
     *  AT+RST
     */
    AT_CMD("+RST", true, 0, at_rst),
    /**
     * Version of at command
     * AT+VER
     * Example:
     *  AT+VER
     */
    AT_CMD("+VER", true, 0, at_ver),
);
#endif

#endif
