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
 * @file     chipsea_dfu.h
 * @date     22. Nov. 2023
 * @author   chipsea
 *
 * @version
 * Version V20211229.1.0
 *  - Initial release
 *
 * @{
 */
 
#ifndef __CHIPSEA_DFU_H__
#define __CHIPSEA_DFU_H__
/*******************************************************************************
 * INCLUDES
 */
#include <stdint.h>
/*lint -save -e451 -e373*/
#include <stdlib.h>
#include <stdbool.h>
#include <assert.h>
#include "chipsea_dfu_nvds.h"
/*lint -restore */
/*
    0x01BFDF55 | Protocol Version(2 Bytes) | RFU(2Bytes) | Ctrl bitmap | Image Num | N × 『|Type|RFU|Version|SIZE|』| Reserved |
*/

extern void ebh_debug(char *fmt, ...);
/*********************************************************************
 * MACROS
 */
#define log_debug CS_LOG_DEBUG
#define DFU_APP_DESCRIBE "Version1.0"
#ifndef __ALIGNED
#define __ALIGNED(n)
#endif
#ifndef __WEAK
#define __WEAK __attribute__((weak))
#endif

/*********************************************************************
 * TYPEDEFS
 */
enum{
    DFU_UPDATE_ST_SUCCESS = 0,
    DFU_UPDATE_ST_DISCONNECT,
    DFU_UPDATE_ST_FAILED,
    DFU_UPDATE_ST_TIMEOUT,
};

enum{
    DFU_STATUS_ENABLED = 0,
    DFU_STATUS_DISABLED,
    DFU_STATUS_LOCKED,
};

enum{
    DFU_CTRL_BIT_SIGN     = 1<<0,
    DFU_CTRL_BIT_MORE_IMG = 1<<1,
};

#define DFU_DATE_VERSION             0x20211229
#define DFU_TYPE_VERSION             0x0000000F
#define DFU_PROTOCOL_VERSION         0x00000003
#define DFU_CTRL_SIGN_EN             0  // digital signature support
#define DFU_FORCE_CHECK_SHA256_EN    0  // check hash even sign not support, use @ref dfu_sha256_cmp to check
typedef void(*dfu_indicate_cb_t)(uint8_t status, void *p);
typedef uint8_t (*dfu_user_check_cb_t)(uint8_t img_type, uint32_t img_size, uint32_t img_version);
typedef void(*dfu_img_op_notify_cb_t)(uint8_t img_type, uint8_t state, uint32_t offset, uint32_t image_size); // state: 0-flashing prepare, 1-flashing done

typedef struct {
    const dfu_indicate_cb_t dfu_begin_cb;
    const dfu_indicate_cb_t dfu_prog_cb;
    const dfu_indicate_cb_t dfu_end_cb;
    const dfu_user_check_cb_t dfu_user_check_cb;
    const dfu_img_op_notify_cb_t dfu_img_op_notify_cb;
}dfu_cb_itf_t;

extern const dfu_cb_itf_t dfu_cb_itf;

/*********************************************************************
 * MACROS
 */
#define DFU_DATA_MAX_SIZE            0x10000000

#define DFU_COMMAND_OBJ_MAX_SIZE     128
#define DFU_CACHE_BUF_SIZE           128 // 作为写入flash的cache buffer，需要16字节对齐

#define DFU_CTRL_CREATE              0x01
#define DFU_CTRL_SET_PRN             0x02
#define DFU_CTRL_CAL_CHECKSUM        0x03
#define DFU_CTRL_EXECTUE             0x04
#define DFU_CTRL_RESERVE             0x05
#define DFU_CTRL_SELECT              0x06
#define DFU_CTRL_RESPONSE            0x60

#define DFU_INVALID_CODE             0x00
#define DFU_SUCCESS                  0x01
#define DFU_OPCODE_NOT_SUPPORT       0x02
#define DFU_INVALID_PARAMETER        0x03
#define DFU_INSUFFICIENT_RESOURCES   0x04
#define DFU_INVALID_OBJECT           0x05
#define DFU_UNSUPPORTED_TYPE         0x07
#define DFU_OPERATION_NOT_PERMITTED  0x08
#define DFU_OPERATION_FAILED         0x0A
#define DFU_VERSION_NOT_MATCH        0x41

// DFU_RESP_SIZE
#define DFU_RESP_SIZE_NO_DATA      0
#define DFU_RESP_SIZE_NO_EXT_DATA  3
#define DFU_RESP_SIZE_CHECKSUM     (3+4*2)
#define DFU_RESP_SIZE_SELECT       (3+4*3)

struct dfu_rsp_checksum
{
    uint32_t offset;
    uint32_t crc32;
};

struct dfu_rsp_select
{
    uint32_t max_size;
    uint32_t offset;
    uint32_t crc32;
};

union dfu_rsp_data
{
    struct dfu_rsp_checksum checksum;
    struct dfu_rsp_select select_data;
};

typedef struct
{
    uint8_t length; // @ref DFU_RESP_SIZE
    uint8_t rsp_code;
    uint8_t opcode;
    uint8_t result;
    union dfu_rsp_data data;
} dfu_response_t;


struct dfu_app_desc_data{
    uint32_t rsp_length;
    const char *app_describe;
};
struct dfu_version_data{
    uint32_t address;
    uint32_t size;
    uint32_t version;
};

typedef union
{
    struct dfu_app_desc_data desc_data; // response read verison
    struct dfu_version_data version_data; // notify version info
} dfu_version_t;

#define dfu_assert(p)    

/*******************************************************************************
 * EXTERN FUNCTIONS
 */
void dfu_reset(uint8_t reason); // reason: disconnect/timeout
void dfu_write_cmd(const uint8_t *data, uint32_t len, dfu_response_t *response);
void dfu_write_data(const uint8_t *data, uint32_t len, dfu_response_t *response);
void dfu_read_version_char(dfu_version_t *version);
void dfu_write_version_char(uint32_t cmd, dfu_version_t *version);
int dfu_set_enable(bool enabled); // default: DFU_STATUS_ENABLED
#if DFU_FORCE_CHECK_SHA256_EN
extern int dfu_sha256_cmp(uint8_t *sha256_resule, uint8_t sha256_len);
#endif

#endif /* __CHIPSEA_DFU_H__ */

/** @} */

