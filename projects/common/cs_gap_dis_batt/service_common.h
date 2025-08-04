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
 * @file     service_common.h
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */
/*******************************************************************************
 * INCLUDES
 */
#include <stdint.h>

#ifndef __SERVICE_COMMON_H__
#define __SERVICE_COMMON_H__

/*********************************************************************
 * MACROS
 */
/// GAP Device Name,
#define GAP_DEVICE_NAME "CST92F41 Simple"
/// GAP APPEARANCE
#define GAP_APPEARANCE "\xc2\x03"
/// DIS SYSTEM ID
#define DIS_SYSTEM_ID "\x00\x00\x00\x00\x00\x00\x00\x00"
/// DIS HARD VERSION
#define DIS_HARD_VERSION "1.0.0.0"
/// DIS SOFT VERSION
#define DIS_SOFT_VERSION "1.0.0.0"
/// DIS MANU NAME STR
#define DIS_MANU_NAME_STR "chipsea."

/// DIS PNP ID
#define DIS_PNP_ID ("\x01\xA7\x06\xB8\x32\x79\x49")
/// Battery service, set 1 to Enable
#define SERVICE_BATTARY 1

/*******************************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief Init common services, GAP, DIS, BATT included.
 *******************************************************************************
 */
void service_common_init(void);

#if SERVICE_BATTARY
/**
 *******************************************************************************
 * @brief Notify battery level changed.
 *
 * @param[in] val  The value of battary level
 *******************************************************************************
 */
void batt_level_change(uint8_t val);
#endif

#endif /* __SERVICE_COMMON_H__ */

/** @} */
