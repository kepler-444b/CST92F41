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
 * @file     cs_printf.h
 * @brief
 * @date     29. Sept 2021
 * @author   chipsea
 *
 * @defgroup DOC DOC
 * @ingroup  DOCUMENT
 * @brief    Tiny printf, sprintf and (v)snprintf implementation
 * @details  Optimized for speed on embedded systems with a very limited resources
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __CS_PRINTF_H
#define __CS_PRINTF_H


/*******************************************************************************
 * INCLUDES
 */
#include <stdarg.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 * MACROS
 */
// support for the floating point type (%f)
// default: activated
#define PRINTF_SUPPORT_FLOAT

// support for exponential floating point notation (%e/%g)
// default: activated
#define PRINTF_SUPPORT_EXPONENTIAL

// support for the long long types (%llu or %p)
// default: activated
#define PRINTF_SUPPORT_LONG_LONG

// support for the ptrdiff_t type (%t)
// ptrdiff_t is normally defined in <stddef.h> as long or long long type
// default: activated
#define PRINTF_SUPPORT_PTRDIFF_T            1



/*******************************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *
 * This function is declared here only. You have to write your custom implementation somewhere
 * \param character Character to output
 */
/**
 *******************************************************************************
 * @brief Output a character to a custom device like UART, used by the printf() function
 *
 * @param[in] character      Character to output
 *
 *******************************************************************************
 */
extern void cs_putchar(char character);

/**
 *******************************************************************************
 * @brief Tiny printf implementation
 *
 * @param[in] format      A string that specifies the format of the output
 *
 * @return: The number of characters that are written into the array, not counting the terminating null character
 *******************************************************************************
 */
extern int cs_printf(const char* format, ...);

/**
 *******************************************************************************
 * @brief Tiny sprintf implementation
 *
 * @param[in] buffer      A pointer to the buffer where to store the formatted string. MUST be big enough to store the output!
 * @param[in] format      A string that specifies the format of the output
 *
 * @return: The number of characters that are written into the buffer, not counting the terminating null character
 *******************************************************************************
 */
extern int cs_sprintf(char* buffer, const char* format, ...);

/**
 *******************************************************************************
 * @brief Tiny snprintf implementation
 *
 * @param[in] buffer      A pointer to the buffer where to store the formatted string.
 * @param[in] count       The maximum number of characters to store in the buffer, including a terminating null character
 * @param[in] format      A string that specifies the format of the output
 *
 * @return: The number of characters that COULD have been written into the buffer, not counting the terminating
 *          null character. A value equal or larger than count indicates truncation. Only when the returned value
 *          is non-negative and less than count, the string has been completely written.
 *******************************************************************************
 */
extern int cs_snprintf(char* buffer, size_t count, const char* format, ...);

/**
 *******************************************************************************
 * @brief Tiny vsnprintf implementation
 *
 * @param[in] buffer      A pointer to the buffer where to store the formatted string.
 * @param[in] count       The maximum number of characters to store in the buffer, including a terminating null character
 * @param[in] format      A string that specifies the format of the output
 * @param[in] va          A value identifying a variable arguments list
 *
 * @return: The number of characters that COULD have been written into the buffer, not counting the terminating
 *          null character. A value equal or larger than count indicates truncation. Only when the returned value
 *          is non-negative and less than count, the string has been completely written.
 *******************************************************************************
 */
extern int cs_vsnprintf(char* buffer, size_t count, const char* format, va_list va);

/**
 *******************************************************************************
 * @brief Tiny vsnprintf implementation
 *
 * @param[in] format      A string that specifies the format of the output
 * @param[in] va          A value identifying a variable arguments list
 *
 * @return: The number of characters that are WRITTEN into the buffer, not counting the terminating null character
 *******************************************************************************
 */
extern int cs_vprintf(const char* format, va_list va);

/**
 * printf with output function
 * You may use this as dynamic alternative to printf() with its fixed _putchar() output
 * \param out An output function which takes one character and an argument pointer
 * \param arg An argument pointer for user data passed to output function
 * \param format A string that specifies the format of the output
 * \return The number of characters that are sent to the output function, not counting the terminating null character
 */
/**
 *******************************************************************************
 * @brief Printf with output function
 *        You may use this as dynamic alternative to printf() with its fixed _putchar() output
 *
 * @param[in] out         An output function which takes one character and an argument pointer
 * @param[in] arg         An argument pointer for user data passed to output function
 * @param[in] format      A string that specifies the format of the output
 *
 * @return: The number of characters that are sent to the output function, not counting the terminating null character
 *******************************************************************************
 */
extern int cs_fctprintf(void (*out)(char character, void* arg), void* arg, const char* format, ...);


#ifdef __cplusplus
}
#endif

#endif  /* __CS_PRINTF_H */


/** @} */
