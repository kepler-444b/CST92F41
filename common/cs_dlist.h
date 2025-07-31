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
 * @file cs_list.h
 * @brief list
 * @date Tue 09 Dec 2014 04:37:42 PM CST
 * @author chipsea
 *
 * @defgroup CS_DLIST Doubly linked List
 * @ingroup COMMON
 * @brief List Module
 * @details
 *
 * @{
 */

#ifndef __CS_DLIST_H__
#define __CS_DLIST_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "cs_utils.h"

/*********************************************************************
 * MACROS
 */
/// Magic poison1
#define CS_DLIST_POISON1    (cs_dlist_t *)0xFF123456
/// Magic poison2
#define CS_DLIST_POISON2    (cs_dlist_t *)0xFF234567

/**
 * @brief Traverse all list item
 *
 * @param[in] pos  list position
 * @param[in] head  list header
 **/
#define cs_dlist_for_each(pos, head) \
    for (pos = (head)->next; pos != (head); pos = pos->next)

/**
 * @brief Traverse all list item
 *
 * @param[in] pos  list position
 * @param[in] head  list header
 * @param[in] type  list item type struct
 **/
#define cs_dlist_for_each_ex(pos, head, type) \
    for (pos = (type *)((cs_dlist_t *)(head))->next; pos != (type *)(head); pos = (type *)((cs_dlist_t *)pos)->next)

/**
 * @brief Traverse all list item, can delete item in loop
 *
 * @param[in] pos  list position
 * @param[in] n  list next position (tmp value)
 * @param[in] head  list header
 **/
#define cs_dlist_for_each_safe(pos, n, head) \
    for (pos = (head)->next, n = pos->next; pos != (head); pos = n, n = pos->next)

/**
 * @brief Traverse all list item, can delete item in loop
 *
 * @param[in] pos  list position
 * @param[in] n  list next position (tmp value)
 * @param[in] head  list header
 * @param[in] type  list item type struct
 **/
#define cs_dlist_for_each_safe_ex(pos, n, head, type) \
    for (pos = (type *)((cs_dlist_t *)(head))->next, n = (type *)((cs_dlist_t *)pos)->next; pos != (type *)(head); pos = n, n = (type *)((cs_dlist_t *)pos)->next)


/*********************************************************************
 * TYPEDEFS
 */
/// list structure
typedef struct __cs_list
{
    /// The next pointer
    struct __cs_list * volatile next;
    /// The prev pointer
    struct __cs_list * volatile prev;
}cs_dlist_t;

/**
 * @brief list insert compare callback
 *
 * @param[in] me  new insert node
 * @param[in] qnode  current node
 * @param[in] param  param
 *
 * @return whether insert
 **/
typedef bool (*cmp_insert_callback_t)(const cs_dlist_t *me, const cs_dlist_t *qnode, void *param);

/**
 * @brief list extract compare callback
 *
 * @param[in] qnode  current node
 * @param[in] param  param
 *
 * @return whether extract
 **/
typedef bool (*cmp_extract_callback_t)(const cs_dlist_t *qnode, void *param);

/*********************************************************************
 * EXTERN VARIABLES
 */


/*********************************************************************
 * INLINE FUNCTIONS
 */
/**
 * @brief list initialize
 *
 * @param[in] head  list head
 **/
__STATIC_INLINE void cs_dlist_init(cs_dlist_t *head)
{
    head->next = head;
    head->prev = head;
}

/**
 *******************************************************************************
 * @brief  cs dlist add
 *
 * @param[in] new  new
 * @param[in] prev  prev
 * @param[in] next  next
 *******************************************************************************
 */
__STATIC_INLINE void __cs_dlist_add(cs_dlist_t *new, cs_dlist_t *prev, cs_dlist_t *next)
{
    CS_ASSERT(next->prev == prev);
    CS_ASSERT(prev->next == next);

    next->prev = new;
    new->next = next;
    new->prev = prev;
    prev->next = new;
}

/**
 * @brief add a item to front
 *
 * @param[in] new  new item
 * @param[in] head  list head
 **/
__STATIC_INLINE void cs_dlist_add_front(cs_dlist_t *new, cs_dlist_t *head)
{
    __cs_dlist_add(new, head, head->next);
}

/**
 * @brief add a item to tail
 *
 * @param[in] new  new item
 * @param[in] head  list head
 **/
__STATIC_INLINE void cs_dlist_add(cs_dlist_t *new, cs_dlist_t *head)
{
    __cs_dlist_add(new, head->prev, head);
}

/**
 * @brief insert a item to a specified place
 *
 * if cmp_cb() return true, insert the front of the cmp-node.
 * if no true return, insert the tail
 *
 * @param[in] new  new item
 * @param[in] head  list head
 * @param[in] cmp_cb  compare callback
 * @param[in] param  param
 **/
__STATIC_INLINE void cs_dlist_insert(cs_dlist_t *new, cs_dlist_t *head, cmp_insert_callback_t cmp_cb, void *param)
{
    cs_dlist_t *pos;

    cs_dlist_for_each(pos, head)
    {
        if(cmp_cb(new, pos, param))
        {
            __cs_dlist_add(new, pos->prev, pos);
            return;
        }
    }

    __cs_dlist_add(new, pos->prev, pos);
}

/**
 *******************************************************************************
 * @brief  cs dlist del
 *
 * @param[in] entry  entry
 *******************************************************************************
 */
__STATIC_INLINE void cs_dlist_del(cs_dlist_t *entry)
{
    CS_ASSERT(entry->prev->next == entry);
    CS_ASSERT(entry->next->prev == entry);

    entry->next->prev = entry->prev;
    entry->prev->next = entry->next;
    entry->next = CS_DLIST_POISON1;
    entry->prev = CS_DLIST_POISON2;
}

/**
 * @brief find a item
 *
 * @param[in] head  list head
 * @param[in] item  item
 *
 * @return if find, return it, otherwise return NULL
 **/
__STATIC_INLINE cs_dlist_t *cs_dlist_find(cs_dlist_t *head, void *item)
{
    cs_dlist_t *pos;

    cs_dlist_for_each(pos, head)
    {
        if(pos == item)
            return pos;
    }

    return NULL;
}

/**
 * @brief extract a item
 *
 * @param[in] head  list head
 * @param[in] item  item
 *
 * @return if find, extract it, otherwise extract NULL
 **/
__STATIC_INLINE cs_dlist_t *cs_dlist_extract(cs_dlist_t *head, void *item)
{
    cs_dlist_t *pos;

    pos = cs_dlist_find(head, item);

    if(pos)
        cs_dlist_del(pos);

    return pos;
}

/**
 * @brief find a item
 *
 * if cmp_cb() return true, extract it and delete it from q.
 * if no true return, extact NULL.
 *
 * @param[in] head  list head
 * @param[in] cmp_cb  compare callback
 * @param[in] param  param
 *
 * @return if find, return it, otherwise return NULL
 **/
__STATIC_INLINE cs_dlist_t *cs_dlist_find_ex(cs_dlist_t *head, cmp_extract_callback_t cmp_cb, void *param)
{
    cs_dlist_t *pos;

    cs_dlist_for_each(pos, head)
    {
        if(cmp_cb(pos, param))
            return pos;
    }

    return NULL;
}

/**
 * @brief extract a item
 *
 * if cmp_cb() return true, extract it and delete it from q.
 * if no true return, extact NULL.
 *
 * @param[in] head  list head
 * @param[in] cmp_cb  compare callback
 * @param[in] param  param
 *
 * @return if find, extract it, otherwise extract NULL
 **/
__STATIC_INLINE cs_dlist_t *cs_dlist_extract_ex(cs_dlist_t *head, cmp_extract_callback_t cmp_cb, void *param)
{
    cs_dlist_t *pos;

    pos = cs_dlist_find_ex(head, cmp_cb, param);

    if(pos)
        cs_dlist_del(pos);

    return pos;
}

/**
 * @brief whether is empty
 *
 * @param[in] head  list head
 *
 * @return empty?
 **/
__STATIC_INLINE bool cs_dlist_is_empty(const cs_dlist_t *head)
{
    return head->next == head;
}

/**
 * @brief whether is first
 *
 * @param[in] node  check item
 * @param[in] head  list head
 *
 * @return first?
 **/
__STATIC_INLINE bool cs_dlist_is_first(const cs_dlist_t *node, const cs_dlist_t *head)
{
    return head->next == node;
}

/**
 * @brief whether is invalid
 *
 * @param[in] entry  check item
 *
 * @return deleted
 **/
__STATIC_INLINE bool cs_dlist_is_invalid(const cs_dlist_t *entry)
{
    return (entry==CS_DLIST_POISON1 ||
            entry==CS_DLIST_POISON2 ||
            entry->next==CS_DLIST_POISON1 ||
            entry->prev==CS_DLIST_POISON2) ? true : false;
}

/**
 * @brief number of list
 *
 * @param[in] head  list head
 *
 * @return number
 **/
__STATIC_INLINE unsigned cs_dlist_num(const cs_dlist_t *head)
{
    cs_dlist_t *pos;
    unsigned num = 0;

    cs_dlist_for_each(pos, head)
    {
        ++num;
    }

    return num;
}

/**
 * @brief get first item from list
 *
 * @param[in] head  list head
 *
 * @retval NULL no item
 * @retval NOT_NULL first item
 **/
__STATIC_INLINE cs_dlist_t *cs_dlist_first(cs_dlist_t *head)
{
    return cs_dlist_is_empty(head) ? NULL : head->next;
}

/**
 * @brief get last item from list
 *
 * @param[in] head  list head
 *
 * @retval NULL no item
 * @retval NOT_NULL last item
 **/
__STATIC_INLINE cs_dlist_t *cs_dlist_last(cs_dlist_t *head)
{
    return cs_dlist_is_empty(head) ? NULL : head->prev;
}

/**
 * @brief move one item to another list
 *
 * @param[in] item  moved item
 * @param[in] head  list head
 **/
__STATIC_INLINE void cs_dlist_move(cs_dlist_t *item, cs_dlist_t *head)
{
    cs_dlist_del(item);
    cs_dlist_add(item, head);
}

/**
 * @brief  co list push
 *
 * @param[in] new  new
 * @param[in] head  head
 **/
__STATIC_INLINE void cs_dlist_push(cs_dlist_t *new, cs_dlist_t *head)
{
    cs_dlist_add(new, head);
}

/**
 * @brief  co list push front
 *
 * @param[in] new  new
 * @param[in] head  head
 **/
__STATIC_INLINE void cs_dlist_push_front(cs_dlist_t *new, cs_dlist_t *head)
{
    cs_dlist_add_front(new, head);
}

/**
 * @brief  co list pop
 *
 * @param[in] head  head
 **/
__STATIC_INLINE cs_dlist_t *cs_dlist_pop(cs_dlist_t *head)
{
    cs_dlist_t *node = cs_dlist_first(head);

    if (node)
        cs_dlist_del(node);

    return node;
}

/**
 * @brief  co list pop behind
 *
 * @param[in] head  head
 **/
__STATIC_INLINE cs_dlist_t *cs_dlist_pop_behind(cs_dlist_t *head)
{
    cs_dlist_t *node = cs_dlist_last(head);

    if (node)
        cs_dlist_del(node);

    return node;
}

/*********************************************************************
 * EXTERN FUNCTIONS
 */



#ifdef __cplusplus
}
#endif

#endif

/** @} */

