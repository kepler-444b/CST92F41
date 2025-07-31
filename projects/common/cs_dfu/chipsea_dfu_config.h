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
 * @file     chipsea_dfu_config.h
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @version
 * Version V20210119.1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __CHIPSEA_DFU_CONFIG_H__
#define __CHIPSEA_DFU_CONFIG_H__

/*******************************************************************************
 * INCLUDES
 */
#include "stdlib.h"
#include "chipsea_dfu_nvds.h"

//#define FLASH_ERASE_SIZE 0x1000   // Image写入地址必须以擦除大小对齐

/******* Warning: 不要在 onchipsea_dfu 以外的地方include该文件! *******/

// enum image_type{ //Defined in chipsea_dfu_nvds.h
//     IMAGE_TYPE_APP,
//     IMAGE_TYPE_PATCH,
//     IMAGE_TYPE_CONFIG,
//     IMAGE_TYPE_MBR_USR1,
//     IMAGE_TYPE_MBR_USR2,
//     IMAGE_TYPE_MBR_USR3,
//     IMAGE_TYPE_MBR_USR4,
//     IMAGE_TYPE_DUMMY,
//     IMAGE_TYPE_CUSTOM = 0x10,
//     IMAGE_TYPE_RAW    = 0x5F,
// };

typedef struct {
	uint16_t type; // @ref enum image_type
	uint32_t base_address1;
	uint32_t base_address2;
	uint32_t max_length;
	const char *describe; // 功能标识，仅用于Debug
    const dfu_nvds_itf_t *image_ops_itf; // Imgage 读写接口
    const dfu_nvds_itf_t *info_ops_itf;  // Image info 存储接口
	uint16_t info_id; //Image 信息在 NVDS 中存放的位置
}dfu_image_t;

const dfu_image_t dfu_image_types[] = {
//----------------------------------------------------------------------------------------------------------
 /* Image type               |  Base address1 |  Base address2 |  Max length |  Image describe  |
        Image ops API                                | Image info ops API                       |  Info ID */
//----------------------------------------------------------------------------------------------------------
	{IMAGE_TYPE_APP          ,  0x00040000    ,  0x00040000    ,   0x00040000,  "Application"   ,
        &dfu_nvds_itf[DFU_NVDS_ITF_TYPE_FLASH]       , NULL                                     , 0x00    },
//----------------------------------------------------------------------------------------------------------
	{IMAGE_TYPE_DUMMY        ,  0x00000000    ,  0x00000000    ,   0xFFFFFFFF,  "Dummy"     ,
        &dfu_nvds_itf[DFU_NVDS_ITF_TYPE_DUMMY]       , NULL                                     , 0x00    },
//----------------------------------------------------------------------------------------------------------
};
const uint8_t dfu_image_types_num = sizeof(dfu_image_types) / sizeof(dfu_image_types[0]);

#endif /* __CHIPSEA_DFU_CONFIG_H__ */

/** @} */

