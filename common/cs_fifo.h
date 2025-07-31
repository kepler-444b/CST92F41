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
 * @file     cs_fifo.h
 * @brief    fifo with unlimited size but be slower than cs_kfifo
 * @date     05. June 2020
 * @author   chipsea
 *
 * @defgroup CS_FIFO Fifo
 * @ingroup  COMMON
 * @brief    Fifo
 * @details  Fifo
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */


#ifndef __CS_FIFO_H
#define __CS_FIFO_H


/*******************************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include <stddef.h>

#ifdef  __cplusplus
extern "C"
{
#endif


/*******************************************************************************
 * TYPEDEFS
 */
/// fifo struct
typedef struct {
    /// fifo's buffer
    unsigned char *buffer;
    /// buffer total size
    unsigned int size;
    /// in position
    volatile unsigned int in;
    /// out position
    volatile unsigned int out;
} cs_fifo_t;


/*******************************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief  cs nfifo size
 *
 * @param[in] fifo  fifo
 *
 * @return size
 *******************************************************************************
 **/
extern unsigned int cs_fifo_size(cs_fifo_t* fifo);

/**
 *******************************************************************************
 * @brief  cs nfifo avail
 *
 * @param[in] fifo  fifo
 *
 * @return valid length
 *******************************************************************************
 **/
extern unsigned int cs_fifo_avail(cs_fifo_t *fifo);

/**
 *******************************************************************************
 * @brief  cs nfifo len
 *
 * @param[in] fifo  fifo
 *
 * @return length
 *******************************************************************************
 **/
extern unsigned int cs_fifo_len(cs_fifo_t* fifo);

/**
 *******************************************************************************
 * @brief whether is fifo empty
 *
 * @param[in] fifo  fifo object
 *
 * @return empty?
 *******************************************************************************
 **/
extern int cs_fifo_is_empty(cs_fifo_t *fifo);

/**
 * @brief  cs fifo reset
 *
 * @param[in] fifo  fifo
 **/
extern void cs_fifo_reset(cs_fifo_t *fifo);

/**
 *******************************************************************************
 * @brief  cs nfifo init
 *
 * @param[in] fifo  fifo
 * @param[in] buffer  buffer
 * @param[in] size  size
 *******************************************************************************
 **/
extern void cs_fifo_init(cs_fifo_t *fifo, unsigned char *buffer, unsigned int size);

/**
 *******************************************************************************
 * @brief  cs nfifo in 1byte
 *
 * @param[in] fifo  fifo
 * @param[in] from  from
 *
 * @return in bytes
 *******************************************************************************
 **/
extern unsigned int cs_fifo_in_1byte(cs_fifo_t *fifo, const unsigned char *from);

/**
 *******************************************************************************
 * @brief  cs nfifo in
 *
 * @param[in] fifo  fifo
 * @param[in] from  from
 * @param[in] len  len
 *
 * @return in bytes
 *******************************************************************************
 **/
extern unsigned int cs_fifo_in(cs_fifo_t *fifo, const unsigned char *from, unsigned int len);

/**
 *******************************************************************************
 * @brief  cs nfifo out 1byte
 *
 * @param[in] fifo  fifo
 * @param[in] to  to
 *
 * @return  out bytes
 *******************************************************************************
 **/
extern unsigned int cs_fifo_out_1byte(cs_fifo_t *fifo, unsigned char *to);

/**
 *******************************************************************************
 * @brief  cs nfifo out
 *
 * @param[in] fifo  fifo
 * @param[in] to  to
 * @param[in] len  len
 *
 * @return out bytes
 *******************************************************************************
 **/
extern unsigned int cs_fifo_out(cs_fifo_t *fifo, unsigned char *to, unsigned int len);


#ifdef  __cplusplus
}
#endif

#endif  /* __CS_FIFO_H */

/** @} */
