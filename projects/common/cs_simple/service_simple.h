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
 * @file     service_tspp.h
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */
#ifndef __SERVICE_SIMPLE_H__
#define __SERVICE_SIMPLE_H__

/*******************************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include "sdk_config.h"
#include "cs_ble.h"
#include "cs_fifo.h"

/*********************************************************************
 * MACROS
 */
#define MAX_MTU_SIZE                        247 

#define DEFAULT_IBEACON_UUID                0x07,0x64,0x78,0x25, 0xFD, 0xA5, 0x06, 0x93, 0xA4, 0xE2, 0x4F, 0xB1, 0xAF, 0xCF, 0xC6, 0xEB
#define DEFAULT_IBEACON_MAJOR               0xFE23
#define DEFAULT_IBEACON_MINOR               0x4839
#define	DEFAULT_IBEACON_POWER               (-60)		//RSSI at 1m: -60dBm		

#define IBEACON_UUID_LEN                    16
#define IBEACON_MAJOR_LEN                   2
#define IBEACON_MINOR_LEN                   2
#define IBEACON_RSSI_LEN                    1
#define IBEACON_ATT_PKT_LEN                 251


/*********************************************************************
 * TYPEDEFS
 */
enum {
    IDX_SIMPLE_SVC,
    
    IDX_SIMPLE_CHAR1,
    IDX_SIMPLE_CHAR1_VAL,
    IDX_SIMPLE_CHAR1_DSP,
    
    IDX_SIMPLE_CHAR2,
    IDX_SIMPLE_CHAR2_VAL,
    IDX_SIMPLE_CHAR2_DSP,
    
    IDX_SIMPLE_CHAR3,
    IDX_SIMPLE_CHAR3_VAL,
    IDX_SIMPLE_CHAR3_DSP,
    
    IDX_SIMPLE_CHAR4,
    IDX_SIMPLE_CHAR4_VAL,
    IDX_SIMPLE_CHAR4_DSP,
    
    IDX_SIMPLE_CHAR5,
    IDX_SIMPLE_CHAR5_VAL,
    IDX_SIMPLE_CHAR5_DSP,
    
    IDX_SIMPLE_CHAR6,
    IDX_SIMPLE_CHAR6_VAL,
    IDX_SIMPLE_CHAR6_CCCD,
    IDX_SIMPLE_CHAR6_DSP,
    
    IDX_SIMPLE_CHAR7,
    IDX_SIMPLE_CHAR7_VAL,
    IDX_SIMPLE_CHAR7_DSP,
};

enum {
    PARAM_SUCCESS,
    PARAM_LEN_ERR,
    INVALIDPARAMETER,
    PARAM_ERR_DISABLED,
    PARAM_ERR_EXCEED_MTU,
    PARAM_ERR_FULL,
	PARAM_ERR_SEND
};

/*******************************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief Init service simple
 *******************************************************************************
 */
void app_service_simple_init(void);

void simple_server_char5_pro(uint16_t len, uint8_t *value);

void simple_server_char7_pro(uint16_t len, uint8_t *value);

uint8_t check_simple_notify_enable(void);

uint8_t simple_mtu_get(void);

uint8_t simple_notify_data_send(uint16_t idx, uint8_t* data, uint16_t len);

uint8_t simple_server_get_char_data(uint8_t idx, void *value, uint16_t *len);

void simple_server_tx_complete_pro(void);

void simple_server_disconnect_pro(void);

void simple_server_connect_pro(void);

#endif /* __SERVICE_SIMPLE_H__ */

/** @} */
