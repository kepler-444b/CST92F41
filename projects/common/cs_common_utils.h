#ifndef _CS_UTILS_H
#define _CS_UTILS_H

#include <stdint.h>
#include <stdbool.h>
#include "cs_log.h"

#define log_debug CS_LOG_DEBUG

#ifndef MIN
/**
* @brief Obtain the minimum of two values.
*
* @note Arguments are evaluated twice. Use Z_MIN for a GCC-only, single
* evaluation version
*
* @param a First value.
* @param b Second value.
*
* @returns Minimum value of @p a and @p b.
*/
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#define OFFSET_OF(S, M) ((uint32_t)(&(((S *)0)->M)))

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif

#ifdef __cplusplus
extern "C"
{
#endif

static inline uint8_t CHAR_TO_OCTECT(uint8_t ch)
{
    return ((ch >= '0') && (ch <= '9')) ? (ch - '0') :
            ((ch >= 'a') && (ch <= 'f')) ? (ch - 'a' + 10) :
            ((ch >= 'A') && (ch <= 'F')) ? (ch - 'A' + 10) : 0x2A;
}


/**
 * @brief Swap one buffer content into another
 *
 * Copy the content of src buffer into dst buffer in reversed order,
 * i.e.: src[n] will be put in dst[end-n]
 * Where n is an index and 'end' the last index in both arrays.
 * The 2 memory pointers must be pointing to different areas, and have
 * a minimum size of given length.
 *
 * @param dst A valid pointer on a memory area where to copy the data in
 * @param src A valid pointer on a memory area where to copy the data from
 * @param length Size of both dst and src memory areas
 */
static inline void sys_memcpy_swap(void *dst, const void *src, uint32_t length)
{
    uint8_t *pdst = (uint8_t *)dst;
    const uint8_t *psrc = (const uint8_t *)src;

    if(!((psrc < pdst && (psrc + length) <= pdst) ||
            (psrc > pdst && (pdst + length) <= psrc))) {
        log_debug("Source and destination buffers must not overlap");
    }

    psrc += length - 1;

    for (; length > 0; length--) {
        *pdst++ = *psrc--;
    }
}

const char *bt_hex(const void *buf, uint16_t len);
bool str_is_valid_mac(const char *str);

#ifdef __cplusplus
}
#endif

#endif
