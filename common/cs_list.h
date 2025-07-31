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
 * @file     cs_list.h
 * @brief
 * @date     06. Aug 2020
 * @author   chipsea
 *
 * @defgroup CS_LIST Singly linked list
 * @ingroup  COMMON
 * @brief    Singly linked list
 * @details  Singly linked list
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __CS_LIST_H
#define __CS_LIST_H


/*******************************************************************************
 * INCLUDES
 */
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

/*******************************************************************************
 * MACROS
 */
/**
 *******************************************************************************
 * @brief traverse all node
 *
 * @param[in] pos       current node
 * @param[in] list      list
 *******************************************************************************
 */
#define cs_list_for_each(pos, list) \
    for (pos = (list)->head; pos != NULL; pos = pos->next)

/**
 *******************************************************************************
 * @brief traverse all node, can delete node
 *
 * @param[in] pos_prev  previous node
 * @param[in] pos       current node
 * @param[in] list      list
 *******************************************************************************
 */
#define cs_list_for_each_safe(pos_prev, pos, list) \
    for (pos_prev = NULL, pos = (list)->head; pos != NULL; pos_prev = pos, pos = pos->next)


/*******************************************************************************
 * TYPEDEFS
 */
/// list node define
typedef struct cs_list_node {
    /// next node
    struct cs_list_node * volatile next;
} cs_list_node_t;

/// list define
typedef struct {
    /// head node
    struct cs_list_node * volatile head;
    /// tail node
    struct cs_list_node * volatile tail;
} cs_list_t;

/**
 *******************************************************************************
 * @brief decide whether insert new node by comparision
 *
 * @param[in] new_node      new insert node
 * @param[in] cur_node      current node
 * @param[in] param         param
 *
 * @return whether insert
 *******************************************************************************
 */
typedef bool (*cs_list_cmp_insert_callback_t)(const cs_list_node_t *new_node, const cs_list_node_t *cur_node, void *param);

/**
 *******************************************************************************
 * @brief decide whether extract current node
 *
 * @param[in] cur_node      current node
 * @param[in] param         param
 *
 * @return whether extract
 *******************************************************************************
 */
typedef bool (*cs_list_cmp_extract_callback_t)(const cs_list_node_t *cur_node, void *param);

/*******************************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief cs list init
 *
 * @param[in] list    list
 *******************************************************************************
 */
extern void cs_list_init(cs_list_t *list);

/**
 *******************************************************************************
 * @brief whether is empty
 *
 * @param[in] list    list
 *
 * @return
 *    - true:    is empty
 *    - false:   not empty
 *******************************************************************************
 */
extern bool cs_list_is_empty(const cs_list_t *list);

/**
 *******************************************************************************
 * @brief whether node is the first one
 *
 * @param[in] node  check node
 * @param[in] list  list
 *
 * @return
 *    - true:  is first
 *    - false: not first
 *******************************************************************************
 */
extern bool cs_list_node_is_first(const cs_list_node_t *node, const cs_list_t *list);

/**
 *******************************************************************************
 * @brief get the first node of list
 *
 * @param[in] list    list
 *
 * @return
 *    - NULL : no node
 *    - the pointer to node
 *******************************************************************************
 */
extern cs_list_node_t *cs_list_get_first_node(cs_list_t *list);

/**
 *******************************************************************************
 * @brief get the last node of list
 *
 * @param[in] list    list
 *
 * @retval NULL  no item
 * @retval other  the pointer to node
 *******************************************************************************
 */
extern cs_list_node_t *cs_list_get_last_node(cs_list_t *list);

/**
 *******************************************************************************
 * @brief move one node from old list to new list's tail
 *
 * @param[in] node          the node to be moved
 * @param[in] old_list      the old list
 * @param[in] new_list      the new list
 *******************************************************************************
 */
extern void cs_list_move(cs_list_node_t *node, cs_list_t *old_list, cs_list_t *new_list);

/**
 *******************************************************************************
 * @brief del node
 *
 * @param[in] node     node to be deleted
 * @param[in] list     list
 *
 * @retval true  deleted successfully
 * @retval false  deleted faily
 *******************************************************************************
 */
extern bool cs_list_del_node(cs_list_node_t *node, cs_list_t *list);

/**
 * @brief  cs list add front
 *
 * @param[in] node  node
 * @param[in] list  list
 **/
extern void cs_list_add_front(cs_list_node_t *node, cs_list_t *list);

/**
 * @brief  cs list add
 *
 * @param[in] node  node
 * @param[in] list  list
 **/
extern void cs_list_add(cs_list_node_t *node, cs_list_t *list);

/**
 *******************************************************************************
 * @brief list push
 *
 * @param[in] node    the node to be push
 * @param[in] list    list
 *******************************************************************************
 */
extern void cs_list_push(cs_list_node_t *node, cs_list_t *list);

/**
 *******************************************************************************
 * @brief list push to front
 *
 * @param[in] node    the node to be push
 * @param[in] list    list
 *******************************************************************************
 */
extern void cs_list_push_front(cs_list_node_t *node, cs_list_t *list);

/**
 * @brief  cs list pop
 *
 * @param[in] list  list
 *
 * @return  the poped node
 **/
extern cs_list_node_t *cs_list_pop(cs_list_t *list);

/**
 *******************************************************************************
 * @brief list pop behind
 *
 * @param[in] list    list
 *
 * @retval NULL  no node
 * @retval OTHER  the pointer to node
 *******************************************************************************
 */
extern cs_list_node_t *cs_list_pop_behind(cs_list_t *list);

/**
 * @brief  cs list num
 *
 * @param[in] list  list
 *
 * @return number
 **/
extern unsigned cs_list_num(const cs_list_t *list);

/**
 *******************************************************************************
 * @brief  cs list extract
 *
 * @param[in] list  list
 * @param[in] list_hdr  list hdr
 *
 * @return extract ?
 *******************************************************************************
 **/
extern bool cs_list_extract(cs_list_t *list, cs_list_node_t *list_hdr);

/**
 *******************************************************************************
 * @brief extrac a node
 *
 * if extract_cb() return true, extract it and delete it
 * if no true return, extract NULL
 *
 * @param[in] list          list
 * @param[in] extract_cb    extract callback
 * @param[in] param         extract param
 *
 * @return if find, extract it, otherwise extract NULL
 *******************************************************************************
 */
extern cs_list_node_t *cs_list_extract_ex(cs_list_t *list, cs_list_cmp_extract_callback_t extract_cb, void *param);

/**
 *******************************************************************************
 * @brief  cs list find
 *
 * @param[in] list  list
 * @param[in] list_hdr  list hdr
 *
 * @return find?
 *******************************************************************************
 **/
extern bool cs_list_find(cs_list_t *list, cs_list_node_t *list_hdr);

/**
 *******************************************************************************
 * @brief find a node to be extracted
 *
 * @param[in] list              list
 * @param[in] extract_cb        extract callback
 * @param[in] param             param
 *
 *
 * @return
 *    - the node that was found
 *    - NULL
 *******************************************************************************
 */
extern cs_list_node_t *cs_list_find_ex(cs_list_t *list, cs_list_cmp_extract_callback_t extract_cb, void *param);

/**
 *******************************************************************************
 * @brief insert a item to a specified place, the place is decided by ins_cb
 *
 * if ins_cb() return true, insert the front of the cur_node.
 * if no true return ,insert the tail
 *
 * @param[in] list        list
 * @param[in] new_node    the node to be inserted
 * @param[in] ins_cb      insert callback
 * @param[in] param       param
 *******************************************************************************
 */
extern void cs_list_insert(cs_list_t *list, cs_list_node_t *new_node, cs_list_cmp_insert_callback_t ins_cb, void *param);

/**
 *******************************************************************************
 * @brief  cs list insert before
 *
 * @param[in] list  list
 * @param[in] elt_ref_hdr  elt ref hdr
 * @param[in] elt_to_add_hdr  elt to add hdr
 *******************************************************************************
 **/
extern void cs_list_insert_before(cs_list_t *list, cs_list_node_t *elt_ref_hdr, cs_list_node_t *elt_to_add_hdr);

#ifdef __cplusplus
}
#endif

#endif  /* __CS_LIST_H */


/** @} */

