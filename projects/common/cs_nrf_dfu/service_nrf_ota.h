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
 * @file     service_chipsea_dfu.h
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @version
 * Version V20210119.1.0
 *  - Initial release
 *
 * @{
 */
 
#ifndef __SERVICE_CHIPSEA_DFU_H__
#define __SERVICE_CHIPSEA_DFU_H__

/*******************************************************************************
 * INCLUDES
 */
#include "ota_protocol.h"

/*******************************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief Init chipsea_dfu service
 *******************************************************************************
 */
void app_chipsea_dfu_init(void);

/**
 *******************************************************************************
 * @brief Callback for indicating dfu start
 * @param[in] status       Status
 * @param[in] p            Pointer of data
 *******************************************************************************
 */
extern void app_chipsea_dfu_update_start_ind_handler(uint8_t status, void *p);
/**
 *******************************************************************************
 * @brief Callback for indicating dfu in progress
 * @param[in] status       Status
 * @param[in] p            Pointer of data
 *******************************************************************************
 */
extern void app_chipsea_dfu_update_prog_ind_handler(uint8_t status, void *p);
/**
 *******************************************************************************
 * @brief Callback for indicating dfu end
 * @param[in] status       Status
 * @param[in] p            Pointer of data
 *******************************************************************************
 */
extern void app_chipsea_dfu_update_end_ind_handler(uint8_t status, void *p);

#endif /* __SERVICE_CHIPSEA_DFU_H__ */

/** @} */
