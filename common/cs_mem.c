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
 * @file     cs_mem.c
 * @brief    memory heap manager
 * @date     14 Mar. 2023
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
#include <stddef.h>
#include <string.h>
#include "cs_mem.h"


/*******************************************************************************
 * TYPEDEFS
 */
typedef struct cs_mem_heap {            /* << Memory Pool management struct >>     */
    struct cs_mem_heap   *next;         /* Next Memory Block in the list           */
    uint32_t              len;          /* Length of data block                    */
} cs_mem_heap_t;

typedef struct {
    uint32_t base[CONFIG_MEM_NUM];
    uint32_t end[CONFIG_MEM_NUM];
} cs_mem_env_t;


/*******************************************************************************
 * CONST & VARIABLES
 */
static cs_mem_env_t cs_mem_env;


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
void cs_mem_init(void)
{
    memset((void *)(&cs_mem_env), 0, sizeof(cs_mem_env));
}

void cs_mem_register(cs_mem_type_t mem_type, __ALIGN4 void *pool, __ALIGN4 uint32_t size)
{
    cs_mem_heap_t *mem;

    CS_ASSERT(mem_type < CONFIG_MEM_NUM);
    CS_ASSERT(CS_IS_ALIGN4(pool) && (pool != NULL));
    CS_ASSERT(CS_IS_ALIGN4(size) && (size != 0U));

    if ((uint32_t)pool) {
        CS_ASSERT (size > sizeof(cs_mem_heap_t));

        mem = (cs_mem_heap_t *)pool;
        mem->next       = (cs_mem_heap_t *)((uint32_t)pool + size - sizeof(cs_mem_heap_t *));
        mem->len        = 0U;
        mem->next->next = NULL;
    }

    cs_mem_env.base[mem_type] = (uint32_t)pool;
    cs_mem_env.end[mem_type]  = (uint32_t)pool + size;
}

void *cs_mem_malloc(cs_mem_type_t mem_type, uint32_t size)
{
    cs_mem_heap_t *p, *p_search, *p_new;
    uint32_t hole_size;
    uint32_t pool;

    CS_ASSERT(mem_type < CONFIG_MEM_NUM);
    pool = cs_mem_env.base[mem_type];
    if(!pool) {
        return NULL;
    }

    p_search = (cs_mem_heap_t *)pool;
    /* Add header offset to 'size' */
    size += sizeof(cs_mem_heap_t);
    /* Make sure that block is 4-byte aligned  */
    size = CS_ALIGN4_HI(size);

    CS_CRITICAL_BEGIN();
    while (1) {
        hole_size  = (uint32_t)p_search->next - (uint32_t)p_search;
        hole_size -= p_search->len;
        /* Check if hole size is big enough */
        if (hole_size >= size) {
            break;
        }
        p_search = p_search->next;
        if (p_search->next == NULL) {
            /* Failed, we are at the end of the list */
            p = NULL;
            goto _exit;
        }
    }

    if (p_search->len == 0U) {
        /* No block is allocated, set the Length of the first element */
        p_search->len = size;
        p = (cs_mem_heap_t *)(((uint32_t)p_search) + sizeof(cs_mem_heap_t));
    } else {
        /* Insert new list element into the memory list */
        p_new       = (cs_mem_heap_t *)((uint32_t)p_search + p_search->len);
        p_new->next = p_search->next;
        p_new->len  = size;
        p_search->next = p_new;
        p = (cs_mem_heap_t *)(((uint32_t)p_new) + sizeof(cs_mem_heap_t));
    }

_exit:
    CS_CRITICAL_END();

    return (p);
}

void cs_mem_free(cs_mem_type_t mem_type, void *mem)
{
    cs_mem_heap_t *p_search, *p_prev, *p_return;
    uint32_t pool;

    CS_ASSERT(mem_type < CONFIG_MEM_NUM);

    if (((uint32_t)mem <= cs_mem_env.base[mem_type]) || ((uint32_t)mem >= cs_mem_env.end[mem_type])) {
        return;
    }
    pool = cs_mem_env.base[mem_type];
    CS_ASSERT(pool);

    CS_CRITICAL_BEGIN();
    p_return = (cs_mem_heap_t *)((uint32_t)mem - sizeof(cs_mem_heap_t));

    /* Set list header */
    p_prev = NULL;
    p_search = (cs_mem_heap_t *)pool;
    while (p_search != p_return) {
        p_prev   = p_search;
        p_search = p_search->next;
        if (p_search == NULL) {
            /* Valid Memory block not found */
            goto _exit;
        }
    }

    if (p_prev == NULL) {
        /* First block to be released, only set length to 0 */
        p_search->len = 0U;
    } else {
        /* Discard block from chain list */
        p_prev->next = p_search->next;
    }

_exit:
    CS_CRITICAL_END();
}

void *cs_mem_calloc(cs_mem_type_t mem_type, uint8_t num, uint32_t size)
{
    void *mem;
    mem = cs_mem_malloc(mem_type, num*size);
    if (mem != NULL) {
        memset(mem, 0, num*size);
    }

    return mem;
}

/** @} */

