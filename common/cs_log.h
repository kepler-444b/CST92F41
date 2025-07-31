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
 * @file     cs_log.h
 * @brief
 * @date     06. Aug 2020
 * @author   chipsea
 *
 * @defgroup CS_LOG Log
 * @ingroup  COMMON
 * @brief    Log
 * @details  Log
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __CS_LOG_H__
#define __CS_LOG_H__

#include "features.h"

#ifdef __cplusplus
extern "C"
{ /*}*/
#endif

/*********************************************************************
 * INCLUDES
 */
/// log level
typedef enum {
    CS_LOG_NONE,
    CS_LOG_ERROR,
    CS_LOG_WARN,
    CS_LOG_INFO,
} cs_log_level_t;

/*********************************************************************
 * MACROS
 */
#ifdef CONFIG_LOG
/// log
#define CS_LOG(level, format, ...)                 cs_log(level, format,  ## __VA_ARGS__)
/// log array
#define CS_LOG_ARRAY(level, array, len)            do{int __i; for(__i=0;__i<(len);++__i)CS_LOG(level, "%02X ",((uint8_t *)(array))[__i]);}while(0)
/// log array with show more
#define CS_LOG_ARRAY_EX(level, note, array, len)   do{CS_LOG(level, "%s: ",note); CS_LOG_ARRAY(level, array,len); CS_LOG(level, "[%dbytes]\n",len);}while(0)
/// log with debug
#define CS_LOG_DEBUG(format, ...)                  cs_log(CS_LOG_INFO, format,  ## __VA_ARGS__)
/// log debug array
#define CS_LOG_DEBUG_ARRAY(array, len)             do{int __i; for(__i=0;__i<(len);++__i)CS_LOG_DEBUG("%02X ",((uint8_t *)(array))[__i]);}while(0)
/// log debug array with show more
#define CS_LOG_DEBUG_ARRAY_EX(note, array, len)    do{CS_LOG_DEBUG("%s: ",note); CS_LOG_DEBUG_ARRAY(array,len); CS_LOG_DEBUG("[%dbytes]\n",len);}while(0)
#else
#define CS_LOG(level, format, ...)
#define CS_LOG_ARRAY(level, array, len)
#define CS_LOG_ARRAY_EX(level, note, array, len)
#define CS_LOG_DEBUG(format, ...)
#define CS_LOG_DEBUG_ARRAY(array, len)
#define CS_LOG_DEBUG_ARRAY_EX(note, array, len)
#endif


/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * EXTERN VARIABLES
 */


/*********************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief log output
 *
 * @param[in] level        Log level.
 * @param[in] fmt          String that contains the text to be writen to stdout
 *
 *******************************************************************************
 */
void cs_log(cs_log_level_t level, const char *fmt, ...);

/**
 *******************************************************************************
 * @brief  cs log level set
 *
 * @param[in] level  level
 *******************************************************************************
 **/
void cs_log_level_set(cs_log_level_t level);

#ifdef __cplusplus
/*{*/ }
#endif

#endif

/** @} */

