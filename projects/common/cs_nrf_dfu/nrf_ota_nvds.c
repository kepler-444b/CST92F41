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
 * @file     nrf_ota_nvds.c
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @version
 * Version V20210119.1.0
 *  - Initial release
 *
 * @{
 */

/*******************************************************************************
 * INCLUDES
 */
#include <string.h>
#include "stdio.h"
#include "stdint.h"
#include "stdlib.h"
#include "ota_protocol.h"
#include "nrf_ota_nvds.h"

/*********************************************************************
 * LOCAL VARIABLES
 */
const static int mbr_types[] = { // Correspond to enum image_type
    PART_TYPE_APP,
    PART_TYPE_PATCH,
    PART_TYPE_CFG,
    PART_TYPE_USR1,
    PART_TYPE_USR2,
    PART_TYPE_USR3,
    PART_TYPE_USR4
};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
uint8_t nrf_ota_nvds_enable_flash(void)
{
    drv_sfs_enable();
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
uint8_t nrf_ota_nvds_get_flash(uint32_t addr, uint32_t *lengthPtr, void *buf)
{
    drv_sfs_read(addr, buf, *lengthPtr);
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
uint8_t nrf_ota_nvds_put_flash(uint32_t addr, uint32_t length, void *buf)
{
    drv_sfs_write(addr, buf, length);
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
uint8_t nrf_ota_nvds_erase_flash(uint32_t addr, uint32_t length)
{
    drv_sfs_erase(addr, length);
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
uint8_t nrf_ota_nvds_disable_flash(void)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}

uint8_t nrf_ota_nvds_enable_mbr(void)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
uint8_t nrf_ota_nvds_get_mbr(uint32_t id, uint32_t *lengthPtr, void *buf)
{
    dfu_image_mbr_info *info = (dfu_image_mbr_info*)buf;
    if(id <= sizeof(mbr_types)/sizeof(mbr_types[0]) &&
        mbr_read_part(mbr_types[id], &info->address, &info->length, &info->crc16) == 0){
        return CHIPSEA_DFU_NVDS_ST_SUCCESS;
    }else{
        return CHIPSEA_DFU_NVDS_ST_FAILED;
    }
}
uint8_t nrf_ota_nvds_put_mbr(uint32_t id, uint32_t length, void *buf)
{
    dfu_image_mbr_info *info = (dfu_image_mbr_info*)buf;
    if(id < sizeof(mbr_types)/sizeof(mbr_types[0])){
        mbr_write_part(mbr_types[id], info->address, info->length, info->crc16);
        return CHIPSEA_DFU_NVDS_ST_SUCCESS;
    }else{
        return CHIPSEA_DFU_NVDS_ST_FAILED;
    }
}
uint8_t nrf_ota_nvds_del_mbr(uint32_t id, uint32_t length)
{
    return CHIPSEA_DFU_NVDS_ST_FAILED;
}
static uint8_t nrf_ota_nvds_disable_mbr(void)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}


uint8_t nrf_ota_nvds_enable_cfg(void)
{
    return CHIPSEA_DFU_NVDS_ST_FAILED;
}
uint8_t nrf_ota_nvds_get_cfg(uint32_t id, uint32_t *lengthPtr, void *buf)
{
    return CHIPSEA_DFU_NVDS_ST_FAILED;
}
uint8_t nrf_ota_nvds_put_cfg(uint32_t id, uint32_t length, void *buf)
{
    return CHIPSEA_DFU_NVDS_ST_FAILED;
}
uint8_t nrf_ota_nvds_del_cfg(uint32_t id, uint32_t length)
{
    return CHIPSEA_DFU_NVDS_ST_FAILED;
}
uint8_t nrf_ota_nvds_disable_cfg(void)
{
    return CHIPSEA_DFU_NVDS_ST_FAILED;
}

 uint8_t nrf_ota_nvds_enable_flash_ext(void)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
static uint8_t nrf_ota_nvds_get_flash_ext(uint32_t addr, uint32_t *lengthPtr, void *buf)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
 uint8_t nrf_ota_nvds_put_flash_ext(uint32_t addr, uint32_t length, void *buf)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
uint8_t nrf_ota_nvds_erase_flash_ext(uint32_t addr, uint32_t length)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
uint8_t nrf_ota_nvds_disable_flash_ext(void)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}


uint8_t nrf_ota_nvds_enable_dummy(void)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
uint8_t nrf_ota_nvds_get_dummy(uint32_t id, uint32_t *lengthPtr, void *buf)
{
    memset(buf, 0xFF, *lengthPtr);
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
uint8_t nrf_ota_nvds_put_dummy(uint32_t id, uint32_t length, void *buf)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
uint8_t nrf_ota_nvds_del_dummy(uint32_t id, uint32_t length)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}
uint8_t nrf_ota_nvds_disable_dummy(void)
{
    return CHIPSEA_DFU_NVDS_ST_SUCCESS;
}

const dfu_nvds_itf_t dfu_nvds_itf[] = {
    { //DFU_NVDS_ITF_TYPE_MBR,
        nrf_ota_nvds_enable_mbr,
        nrf_ota_nvds_get_mbr,
        nrf_ota_nvds_put_mbr,
        nrf_ota_nvds_del_mbr,
        nrf_ota_nvds_disable_mbr
    },
    { //DFU_NVDS_ITF_TYPE_FLASH,
        nrf_ota_nvds_enable_flash,
        nrf_ota_nvds_get_flash,
        nrf_ota_nvds_put_flash,
        nrf_ota_nvds_erase_flash,
        nrf_ota_nvds_disable_flash
    },
    { //DFU_NVDS_ITF_TYPE_CFG,
        nrf_ota_nvds_enable_cfg,
        nrf_ota_nvds_get_cfg,
        nrf_ota_nvds_put_cfg,
        nrf_ota_nvds_del_cfg,
        nrf_ota_nvds_disable_cfg
    },
    { //DFU_NVDS_ITF_TYPE_EXT_FLASH,
        nrf_ota_nvds_enable_flash_ext,
        nrf_ota_nvds_get_flash_ext,
        nrf_ota_nvds_put_flash_ext,
        nrf_ota_nvds_erase_flash_ext,
        nrf_ota_nvds_disable_flash_ext
    },
    { //DFU_NVDS_ITF_TYPE_DUMMY,
        nrf_ota_nvds_enable_dummy,
        nrf_ota_nvds_get_dummy,
        nrf_ota_nvds_put_dummy,
        nrf_ota_nvds_del_dummy,
        nrf_ota_nvds_disable_dummy
    },
};

/** @} */

