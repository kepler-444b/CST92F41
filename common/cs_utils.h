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
 * @file     cs_utils.h
 * @brief
 * @date     06. Aug 2020
 * @author   chipsea
 *
 * @defgroup CS_UTILS Utils
 * @ingroup  COMMON
 * @brief    Helper functions
 * @details  Helper functions
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __CS_UTIL_H__
#define __CS_UTIL_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "cs_device.h"
#include "cs_log.h"

/*********************************************************************
 * MACROS
 */
/// MAX
#define CS_MAX(x,y)                            (((x) > (y)) ? (x) : (y))
/// MIN
#define CS_MIN(x,y)                            (((x) < (y)) ? (x) : (y))

/// count of
#define CS_COUNT_OF(s)                         (sizeof(s)/sizeof((s)[0]))
/// offset of
#define CS_OFFSET_OF(type, member)             ((unsigned int) &((type *)0)->member)
/// container of
#define CS_CONTAINER_OF(ptr, type, member)     ((type *)((char *)(ptr) - CS_OFFSET_OF(type, member)))

/**
 *******************************************************************************
 * @brief  The CS_ASSERT macro is used for function's parameters check.
 * @param  expr: If expr is false, it calls drv_assert_failed function
 *         which reports the name of the source file and the source
 *         line number of the call that failed.
 *         If expr is true, it returns no value.
 * @return None
 *******************************************************************************
 */
/* Assert ------------------------------------------------------------------- */
#ifdef CONFIG_ASSERT
#define CS_ASSERT(expr)  do {                                                  \
        if(!(unsigned)(expr)) {                                                \
            while(1);                                                          \
        }                                                                      \
    } while(0)

/// assert with while
#define CS_ASSERT_WHILE(cond, expr)     do {                                   \
        if((unsigned)(cond)) {                                                 \
            if(!(unsigned)(expr)) {                                            \
                CS_LOG(CS_LOG_ERROR, "Error:%s:%d\n", __FILE__, __LINE__);     \
                while(1);                                                      \
            }                                                                  \
        }                                                                      \
    } while(0)

/// assert with show warning
#define CS_ASSERT_WARNING(expr)  do {                                          \
        if(!(unsigned)(expr)) {                                                \
            CS_LOG(CS_LOG_WARN, "WARNING:%s:%d\n", __FILE__, __LINE__);        \
        }                                                                      \
    } while(0)

#else
#define CS_ASSERT(expr)                 ((void)0U)
#define CS_ASSERT_WHILE(cond, expr)     ((void)0U)
#define CS_ASSERT_WARNING(expr)         ((void)0U)
#endif  /* CONFIG_ASSERT */

/// Compile Time Asserts
#define CS_STATIC_ASSERT(exp)           extern char cs_static_assert_failed[(exp)?1:-1]

/**
 *******************************************************************************
 *  @brief Align val on the multiple of align equal or nearest higher. align shall in [2, 4, 8, ... 2**n]
 *
 *  @param[in]   val          Value to align.
 *  @param[in]   align        Align
 *
 *  @return:                  Aligned Value
 *******************************************************************************
 */
#define CS_ALIGN_HI(val, align)    (((unsigned)((val) + ((align) - 1))) & (~((align) - 1)))

/**
 *******************************************************************************
 *  @brief Align val on the multiple of align equal or nearest lower. align shall in [2, 4, 8, ... 2**n]
 *
 *  @param[in]   val          Value to align.
 *  @param[in]   align        Align
 *
 *  @return                   Value aligned.
 *******************************************************************************
 */
#define CS_ALIGN_LO(val, align)    ((unsigned)(val) & (~((align) - 1)))

/**
 *******************************************************************************
 *  @brief Align val on the multiple of 32 equal or nearest higher.
 *
 *  @param[in]   val    Value to align.
 *
 *  @return             Value aligned.
 *******************************************************************************
 */
#define CS_ALIGN32_HI(val) (((unsigned)(val)+31)&(~31))

/**
 *******************************************************************************
 *  @brief Align val on the multiple of 32 equal or nearest lower.
 *
 *  @param[in]   val    Value to align.
 *
 *  @return             Value aligned.
 *******************************************************************************
 */
#define CS_ALIGN32_LO(val) ((unsigned)(val)&(~31))

/**
 *******************************************************************************
 *  @brief Align val on the multiple of 8 equal or nearest higher.
 *
 *  @param[in]   val    Value to align.
 *
 *  @return             Value aligned.
 *******************************************************************************
 */
#define CS_ALIGN8_HI(val) (((unsigned)(val)+7)&(~7))

/**
 *******************************************************************************
 *  @brief Align val on the multiple of 8 equal or nearest lower.
 *
 *  @param[in]   val    Value to align.
 *
 *  @return             Value aligned.
 *******************************************************************************
 */
#define CS_ALIGN8_LO(val) ((unsigned)(val)&(~7))

/**
 *******************************************************************************
 *  @brief Align val on the multiple of 4 equal or nearest higher.
 *
 *  @param[in]   val    Value to align.
 *
 *  @return             Value aligned.
 *******************************************************************************
 */
#define CS_ALIGN4_HI(val) (((unsigned)(val)+3)&(~3))

/**
 *******************************************************************************
 *  @brief Align val on the multiple of 4 equal or nearest lower.
 *
 *  @param[in]   val    Value to align.
 *
 *  @return             Value aligned.
 *******************************************************************************
 */
#define CS_ALIGN4_LO(val) ((unsigned)(val)&(~3))


/**
 *******************************************************************************
 *  @brief Align val on the multiple of 2 equal or nearest higher.
 *
 *  @param[in]   val    Value to align.
 *
 *  @return             Value aligned.
 *******************************************************************************
 */
#define CS_ALIGN2_HI(val) (((unsigned)(val)+1)&(~1))

/**
 *******************************************************************************
 *  @brief Align val on the multiple of 2 equal or nearest lower.
 *
 *  @param[in]   val    Value to align.
 *
 *  @return             Value aligned.
 *******************************************************************************
 */
#define CS_ALIGN2_LO(val) ((unsigned)(val)&(~1))


/**
 *******************************************************************************
 *  @brief Check val is align to the power of 2
 *
 *  @param[in]   val     Value to align.
 *  @param[in]   align   Align value, align range in [2, 4, 8, ...2**31, 0]
 *
 *  @return              is aligned.
 *******************************************************************************
 */
#define CS_IS_ALIGN(val, align)   (!((unsigned)(val) & ((unsigned)(align) - 1)))

/// Check val is align to 16 Bytes.
#define CS_IS_ALIGN16(val)   CS_IS_ALIGN(val, 16)

/// Check val is align to 8 Bytes.
#define CS_IS_ALIGN8(val)    CS_IS_ALIGN(val, 8)

/// Check val is align to 4 Bytes.
#define CS_IS_ALIGN4(val)    CS_IS_ALIGN(val, 4)

/// Check val is align to 2 Bytes.
#define CS_IS_ALIGN2(val)    CS_IS_ALIGN(val, 2)

/* attribute description in function/marcro define -------------------------- */
/* used for specify function/macro parameter as an IN parameter */
#ifdef __IN
#error "redefine __IN"
#else
#define __IN
#endif

/* used for specify function/macro parameter as an OUT parameter */
#ifdef __OUT
#error "redefine __OUT"
#else
#define __OUT
#endif

/* used for specify function/macro parameter as an INOUT parameter */
#ifdef __INOUT
#error "redefine __INOUT"
#else
#define __INOUT
#endif

/* used for specify function/macro parameter need ALIGN to 2 Bytes */
#ifdef __ALIGN2
#error "redefine __ALIGN2"
#else
#define __ALIGN2
#endif

/* used for specify function/macro parameter need ALIGN to 4 Bytes */
#ifdef __ALIGN4
#error "redefine __ALIGN4"
#else
#define __ALIGN4
#endif


/* Assert ------------------------------------------------------------------- */



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
 * @brief  cs crc16 ccitt
 *
 * @param[in] crcinit  crcinit
 * @param[in] buf  buf
 * @param[in] len  len
 *
 * @return crc16
 *******************************************************************************
 **/
extern uint16_t cs_crc16_ccitt(uint16_t crcinit, const void *buf, unsigned len);

/**
 *******************************************************************************
 * @brief  cs array reversal
 *
 * @param[in] array  array
 * @param[in] len  len
 *******************************************************************************
 */
extern void cs_array_reversal(void *array, unsigned len);


/**
 * @brief co_array_is_all_zero()
 *
 * @param[in] array  
 * @param[in] len  
 *
 * @return 
 **/
__STATIC_INLINE bool cs_array_is_all_zero(const void *array, unsigned len)
{
    while(len--)
        if(((uint8_t *)array)[len])
            return false;

    return true;
}

#ifdef __cplusplus
}
#endif

#endif

/** @} */

