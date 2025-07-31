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
 * @file     cs_q.h
 * @brief
 * @date     06. Aug 2020
 * @author   chipsea
 *
 * @defgroup CS_QUEUE Queue
 * @ingroup  COMMON
 * @brief    Queue
 * @details  Queue
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __CS_Q_H__
#define __CS_Q_H__


/*******************************************************************************
 * INCLUDES
 */
#include "cs_list.h"
#include "cs_device.h"

#ifdef __cplusplus
extern "C"
{
#endif


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * TYPEDEFS
 */
/// q is list
typedef cs_list_t cs_q_t;
/// qnode is list_node
typedef cs_list_node_t cs_q_node_t;


/*******************************************************************************
 * EXTERN VARIABLES
 */


/*******************************************************************************
 * INLINE FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief queue initialize
 *
 * @param[in] queue  queue
 *******************************************************************************
 */
__STATIC_INLINE void cs_q_init(cs_q_t *queue)
{
    cs_list_init((cs_list_t *)queue);
}

/**
 *******************************************************************************
 * @brief push a node to queue's front
 *
 * @param[in] new_node    new node
 * @param[in] queue       queue
 *******************************************************************************
 */
__STATIC_INLINE void cs_q_push_front(cs_q_node_t *new_node, cs_q_t *queue)
{
    cs_list_push_front((cs_list_node_t *)new_node, (cs_list_t *)queue);
}

/**
 *******************************************************************************
 * @brief push a node to queue's end
 *
 * @param[in] new_node    new node
 * @param[in] queue       queue
 *******************************************************************************
 */
__STATIC_INLINE void cs_q_push(cs_q_node_t *new_node, cs_q_t *queue)
{
    cs_list_push((cs_list_node_t *)new_node, (cs_list_t *)queue);
}

/**
 *******************************************************************************
 * @brief whether is empty
 *
 * @param[in] queue    queue
 *
 * @return
 *    - true:  queue is empty
 *    - false: queue is not empty
 *******************************************************************************
 */
__STATIC_INLINE bool cs_q_is_empty(const cs_q_t *queue)
{
    return cs_list_is_empty((cs_list_t *)queue);
}

/**
 *******************************************************************************
 * @brief peek node from front
 *
 * @param[in] queue    queue
 *
 * @return
 *    - NULL: no item
 *    - NOT NULL: the first item
 *******************************************************************************
 */
__STATIC_INLINE cs_q_node_t *cs_q_peek(cs_q_t *queue)
{
    return (cs_q_node_t *)cs_list_get_first_node((cs_list_t *)queue);
}

/**
 *******************************************************************************
 * @brief peek node from behind
 *
 * @param[in] queue    queue
 *
 * @return
 *    - NULL: no item
 *    - NOT NULL: the last item
 *******************************************************************************
 */
__STATIC_INLINE cs_q_node_t *cs_q_peek_behind(cs_q_t *queue)
{
    return (cs_q_node_t *)cs_list_get_last_node((cs_list_t *)queue);
}

/**
 *******************************************************************************
 * @brief pop item from the front
 *
 * @param[in] queue    queue
 *
 * @return
 *    - NULL: no item
 *    - NOT NULL: the item
 *******************************************************************************
 */
__STATIC_INLINE cs_q_node_t *cs_q_pop(cs_q_t *queue)
{
    return (cs_q_node_t *)cs_list_pop((cs_list_t *)queue);
}

/**
 *******************************************************************************
 * @brief pop item at the end
 *
 * @param[in] queue    queue
 *
 * @return
 *    - NULL: no item
 *    - NOT NULL: the item
 *******************************************************************************
 */
__STATIC_INLINE cs_q_node_t *cs_q_pop_behind(cs_q_t *queue)
{
    return (cs_q_node_t *)cs_list_pop_behind((cs_list_t *)queue);
}

/**
 *******************************************************************************
 * @brief move node from old_queue to new_queue
 *
 * @param[in] node         node
 * @param[in] old_queue    old queue
 * @param[in] new_queue    new queue
 *******************************************************************************
 */
__STATIC_INLINE void cs_q_move(cs_q_node_t *node, cs_q_t *old_queue, cs_q_t *new_queue)
{
    cs_list_move((cs_list_node_t *)node, (cs_list_t *)old_queue, (cs_list_t *)new_queue);
}


#ifdef __cplusplus
}
#endif

#endif  /* __CODE_TEMPLETE_H */


/** @} */
