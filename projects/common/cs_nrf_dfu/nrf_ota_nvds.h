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
 * @file     chipsea_dfu_nvds.h
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @version
 * Version V20210119.1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __CHIPSEA_DFU_NVDS_H__
#define __CHIPSEA_DFU_NVDS_H__
/*******************************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include "cs_driver.h"
#include "mbr.h"

/*********************************************************************
 * MACROS
 */
#define CHIPSEA_DFU_NVDS_ST_SUCCESS 0
#define CHIPSEA_DFU_NVDS_ST_FAILED  0xFF
#define NRF_OTA_BIN_ADDR        0x00040000

/*********************************************************************
 * TYPEDEFS
 */
enum image_type{
    IMAGE_TYPE_APP,
    IMAGE_TYPE_PATCH,
    IMAGE_TYPE_CONFIG,
    IMAGE_TYPE_MBR_USR1,
    IMAGE_TYPE_MBR_USR2,
    IMAGE_TYPE_MBR_USR3,
    IMAGE_TYPE_MBR_USR4,
    IMAGE_TYPE_MBR_MAX,
    IMAGE_TYPE_DUMMY = IMAGE_TYPE_MBR_MAX,
    IMAGE_TYPE_CUSTOM = 0x10,
    IMAGE_TYPE_RAW    = 0x5F,
};

enum dfu_nvds_itf_type{
    DFU_NVDS_ITF_TYPE_MBR,
    DFU_NVDS_ITF_TYPE_FLASH,
    DFU_NVDS_ITF_TYPE_CFG,
    DFU_NVDS_ITF_TYPE_EXT_FLASH,
    DFU_NVDS_ITF_TYPE_DUMMY,
    DFU_NVDS_ITF_TYPE_MAX,
};

typedef struct {
    uint32_t address;
    uint32_t length;
    uint16_t crc16;
}dfu_image_mbr_info;

typedef struct {
    uint32_t address;
    uint32_t length;
    uint32_t version;
}dfu_image_info;

typedef struct {
    uint8_t (*enable)(void);
    uint8_t (*get)(uint32_t id, uint32_t *lengthPtr, void *buf);
    uint8_t (*put)(uint32_t id, uint32_t length, void *buf);
    uint8_t (*del)(uint32_t id, uint32_t length);
    uint8_t (*disable)(void);
}dfu_nvds_itf_t;

/*******************************************************************************
 * EXTERN VARIABLES
 */
extern const dfu_nvds_itf_t dfu_nvds_itf[DFU_NVDS_ITF_TYPE_MAX];

uint8_t nrf_ota_nvds_enable_flash(void);
uint8_t nrf_ota_nvds_get_flash(uint32_t addr, uint32_t *lengthPtr, void *buf);
uint8_t nrf_ota_nvds_put_flash(uint32_t addr, uint32_t length, void *buf);
uint8_t nrf_ota_nvds_erase_flash(uint32_t addr, uint32_t length);

uint8_t nrf_ota_nvds_enable_mbr(void);
uint8_t nrf_ota_nvds_get_mbr(uint32_t id, uint32_t *lengthPtr, void *buf);
uint8_t nrf_ota_nvds_put_mbr(uint32_t id, uint32_t length, void *buf);

#endif /* __CHIPSEA_DFU_NVDS_H__ */

/** @} */
