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
 * @file     app_hid_media.h
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
#ifndef __APP_HID_MEDIA_H__
#define __APP_HID_MEDIA_H__

/*********************************************************************
 * MACROS
 */
/// HID infomation
#define HID_INFO "\x11\x01\x00\x02"
/// Report ID, Report Type
#define HID_REF1 "\x00\x01"

/*******************************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief HID Increase volume
 *******************************************************************************
 */
void hid_media_vol_inc(void);
/**
 *******************************************************************************
 * @brief HID Decrease volume
 *******************************************************************************
 */
void hid_media_vol_dec(void);
/**
 *******************************************************************************
 * @brief HID Pause media
 *******************************************************************************
 */
void hid_media_pause(void);
/**
 *******************************************************************************
 * @brief HID Play media
 *******************************************************************************
 */
void hid_media_play(void);
/**
 *******************************************************************************
 * @brief HID Play next media
 *******************************************************************************
 */
void hid_media_next(void);
/**
 *******************************************************************************
 * @brief HID Play previous media
 *******************************************************************************
 */
void hid_media_prev(void);

/**
 *******************************************************************************
 * @brief Init common services, GAP, DIS, BATT included.
 *******************************************************************************
 */
void app_media_hid_init(void);

#endif /* __APP_HID_MEDIA_H__ */

/** @} */
