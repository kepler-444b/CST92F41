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
 * @file     cs_error.h
 * @brief    chipsea error codes define
 * @date     19. May 2020
 * @author   chipsea
 *
 * @defgroup CS_ERROR Error code
 * @ingroup  COMMON
 * @brief    Error code
 * @details  Error code
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __CS_ERROR_H
#define __CS_ERROR_H


/*******************************************************************************
 * TYPEDEFS
 */
/// error
typedef enum {
    CS_ERROR_OK                      =  0,      /**< OK */
    CS_ERROR_UNSPECIFIC              =  1,      /**< Unspecific */
    CS_ERROR_BUSY                    =  2,      /**< Busy */
    CS_ERROR_TIMEOUT                 =  3,      /**< Timeout */
    CS_ERROR_UNSUPPORTED             =  4,      /**< Unsupported */
    CS_ERROR_PARAMETER               =  5,      /**< Parameter error */
    CS_ERROR_RESOURCES               =  6,      /**< Resources error */
    CS_ERROR_PERMISSION              =  7,      /**< Permission error */
    CS_ERROR_OUT_OF_RANGE            =  8,      /**< Parameter out of range */
    CS_ERROR_ALIGN                   =  9,      /**< Memory align */
    CS_ERROR_STATUS                  =  10,     /**< Status issue */
    CS_ERROR_VERIFY                  =  11,     /**< Verify error */
    CS_ERROR_FAIL                    =  12,     /**< Fail */
    CS_ERROR_WRITE_FAIL              =  13,     /**< Write Fail */
    CS_ERROR_READ_FAIL               =  14,     /**< Read Fail */
} cs_error_t;

#endif  /* __CS_ERROR_H */


/** @} */


