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
 * @file     cs_list.c
 * @brief    Singly list
 * @date     06 Aug. 2020
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
#include "cs_list.h"
#include "cs_utils.h"


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * LOCAL FUNCTIONS
 */


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief cs list init
 *
 * @param[in] list    list
 *******************************************************************************
 */
void cs_list_init(cs_list_t *list)
{
    if (list) {
        list->head = NULL;
        list->tail = NULL;
    } else {
        CS_ASSERT(0);
    }
}

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
bool cs_list_is_empty(const cs_list_t *list)
{
    if ((list->head == NULL) && (list->tail == NULL)) {
        return true;
    } else if ((list->head != NULL) && (list->tail != NULL)) {
        return false;
    } else {
        CS_ASSERT(0);
        return false;
    }
}

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
bool cs_list_node_is_first(const cs_list_node_t *node, const cs_list_t *list)
{
    return list->head == node;
}

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
cs_list_node_t *cs_list_get_first_node(cs_list_t *list)
{
    return cs_list_is_empty(list) ? NULL : list->head;
}

/**
 *******************************************************************************
 * @brief get the last node of list
 *
 * @param[in] list    list
 *
 * @return
 *    - NULL : no item
 *    - the pointer to node
 *******************************************************************************
 */
cs_list_node_t *cs_list_get_last_node(cs_list_t *list)
{
    return cs_list_is_empty(list) ? NULL : list->tail;
}

/**
 *******************************************************************************
 * @brief move one node from old list to new list's tail
 *
 * @param[in] node          the node to be moved
 * @param[in] old_list      the old list
 * @param[in] new_list      the new list
 *
 * @return None
 *******************************************************************************
 */
void cs_list_move(cs_list_node_t *node, cs_list_t *old_list, cs_list_t *new_list)
{
    cs_list_del_node(node, old_list);
    cs_list_add(node, new_list);
}

/**
 *******************************************************************************
 * @brief del node
 *
 * @param[in] node     node to be deleted
 * @param[in] list     list
 *
 * @return
 *    - true:  deleted successfully
 *    - false: deleted faily
 *******************************************************************************
 */
bool cs_list_del_node(cs_list_node_t *node, cs_list_t *list)
{

    cs_list_node_t *prev_node, *cur_node;

    cs_list_for_each_safe(prev_node, cur_node, list) {
        if (node == cur_node) {
            if (prev_node == NULL) {        // the first node
                list->head = cur_node->next;
            } else {
                prev_node->next = cur_node->next;
            }
            if (cur_node == list->tail) {  // the last node
                list->tail = prev_node;
            }

            return true;
        }
    }

    return false;
}

/**
 * @brief  cs list add front
 *
 * @param[in] node  node
 * @param[in] list  list
 **/
void cs_list_add_front(cs_list_node_t *node, cs_list_t *list)
{
    if ((list != NULL) && (node != NULL)) {
        node->next = list->head;
        list->head = node;
        if (list->tail == NULL) {
            list->tail = node;
        }
    } else {
        CS_ASSERT(0);
    }
}

/**
 * @brief  cs list add
 *
 * @param[in] node  node
 * @param[in] list  list
 **/
void cs_list_add(cs_list_node_t *node, cs_list_t *list)
{
    if ((list != NULL) && (node != NULL)) {
        node->next = NULL;
        if (list->tail) {
            list->tail->next = node;
        } else {    // first node
            list->head = node;
        }
        list->tail = node;
    } else {
        CS_ASSERT(0);
    }
}

/**
 *******************************************************************************
 * @brief list push
 *
 * @param[in] node    the node to be push
 * @param[in] list    list
 *
 * @return None
 *******************************************************************************
 */
void cs_list_push(cs_list_node_t *node, cs_list_t *list)
{
    cs_list_add(node, list);
}

/**
 *******************************************************************************
 * @brief list push to front
 *
 * @param[in] node    the node to be push
 * @param[in] list    list
 *
 * @return None
 *******************************************************************************
 */
void cs_list_push_front(cs_list_node_t *node, cs_list_t *list)
{
    cs_list_add_front(node, list);
}

/**
 * @brief  cs list pop
 *
 * @param[in] list  list
 *
 * @return
 **/
cs_list_node_t *cs_list_pop(cs_list_t *list)
{
    cs_list_node_t *head_node;

    if (list) {
        head_node = list->head;
        if (list->head) {
            list->head = head_node->next;
            if (list->head == NULL) {
                list->tail = NULL;
            }
        }
        return head_node;
    }

    return NULL;
}

/**
 *******************************************************************************
 * @brief list pop behind
 *
 * @param[in] list    list
 *
 * @return
 *    - NULL : no node
 *    - the pointer to node
 *******************************************************************************
 */
cs_list_node_t *cs_list_pop_behind(cs_list_t *list)
{
    cs_list_node_t *tail_node;

    tail_node = cs_list_get_last_node(list);

    if (tail_node) {
        cs_list_del_node(tail_node, list);
    }

    return tail_node;
}

/**
 * @brief  cs list num
 *
 * @param[in] list  list
 *
 * @return
 **/
unsigned cs_list_num(const cs_list_t *list)
{
    cs_list_node_t *tmp_list_hdr;
    unsigned num = 0;

    // Go through the list to find the element
    tmp_list_hdr = list->head;

    while (tmp_list_hdr != NULL) {
        tmp_list_hdr = tmp_list_hdr->next;
        ++num;
    }

    return num;
}

/**
 ****************************************************************************************
 * @brief  cs list extract
 *
 * @param[in] list  list
 * @param[in] list_hdr  list hdr
 *
 * @return extract ?
 ****************************************************************************************
 **/
bool cs_list_extract(cs_list_t *list, cs_list_node_t *list_hdr)
{
    bool found = false;

    cs_list_node_t *prev = NULL;
    cs_list_node_t *curr = list->head;

    // Search for the element
    while(curr != NULL) {
        // Check element
        if(curr == list_hdr) {
            found = true;
            break;
        }

        // Move pointers
        prev = curr;
        curr = curr->next;
    }

    if(found) {
        // Check if the element is first
        if(prev == NULL) {
            // Extract element
            list->head = list_hdr->next;
        } else {
            // Extract element
            prev->next = list_hdr->next;
        }

        // Check if the element is last
        if(list_hdr == list->tail) {
            // Update last pointer
            list->tail = prev;
        }
    }

    return found;
}

/**
 *******************************************************************************
 * @brief extrac a node
 *
 * if extract_cb() return true, extract it and delete it
 * if no true return, extract NULL
 *
 * @param[in] list          list
 * @param[in] extract_cb    extract callback
 *
 * @return if find, extract it, otherwise extract NULL
 *******************************************************************************
 */
cs_list_node_t *cs_list_extract_ex(cs_list_t *list, cs_list_cmp_extract_callback_t extract_cb, void *param)
{
    cs_list_node_t *cur_node, *prev_node;

    cs_list_for_each_safe(prev_node, cur_node, list) {
        if (extract_cb(cur_node, param)) {
            if (prev_node == NULL) {
                list->head = cur_node->next;    // extract the first node
            } else {
                prev_node->next = cur_node->next;
            }

            // check is the node is last
            if (list->tail == cur_node) {
                list->tail = prev_node;
            }
            return cur_node;
        }
    }

    return NULL;
}

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
bool cs_list_find(cs_list_t *list, cs_list_node_t *list_hdr)
{
    cs_list_node_t *tmp_list_hdr;

    // Go through the list to find the element
    tmp_list_hdr = list->head;

    while ((tmp_list_hdr != list_hdr) && (tmp_list_hdr != NULL)) {
        tmp_list_hdr = tmp_list_hdr->next;
    }

    return (tmp_list_hdr == list_hdr);
}

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
cs_list_node_t *cs_list_find_ex(cs_list_t *list, cs_list_cmp_extract_callback_t extract_cb, void *param)
{
    cs_list_node_t *cur_node;

    cs_list_for_each(cur_node, list) {
        if (extract_cb(cur_node, param)) {
            return cur_node;
        }
    }

    return NULL;
}

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
 *
 * @return None
 *******************************************************************************
 */
void cs_list_insert(cs_list_t *list, cs_list_node_t *new_node, cs_list_cmp_insert_callback_t ins_cb, void *param)
{
    cs_list_node_t *cur_node, *prev_node;

    cs_list_for_each_safe(prev_node, cur_node, list) {
        if (ins_cb(new_node, cur_node, param)) {
            if (prev_node == NULL) {
                cs_list_add_front(new_node, list);             // list has only one node
            } else {
                new_node->next = cur_node;                     // found suitable cur_node to insert
                prev_node->next = new_node;
            }
            return;
        }
    }
    cs_list_add(new_node, list);    // list is NULL or no suitable cur_node to insert
}

/**
 *******************************************************************************
 * @brief  cs list insert before
 *
 * @param[in] list  list
 * @param[in] elt_ref_hdr  elt ref hdr
 * @param[in] elt_to_add_hdr  elt to add hdr
 *******************************************************************************
 **/
void cs_list_insert_before(cs_list_t *list, cs_list_node_t *elt_ref_hdr, cs_list_node_t *elt_to_add_hdr)
{
    // If no element referenced
    if(elt_ref_hdr == NULL) {
        cs_list_add_front(elt_to_add_hdr, list);
    } else {
        cs_list_node_t *tmp_list_prev_hdr = NULL;
        cs_list_node_t *tmp_list_curr_hdr;

        // Go through the list to find the element
        tmp_list_curr_hdr = list->head;

        while ((tmp_list_curr_hdr != elt_ref_hdr) && (tmp_list_curr_hdr != NULL)) {
            // Save previous element
            tmp_list_prev_hdr = tmp_list_curr_hdr;
            // Get the next element of the list
            tmp_list_curr_hdr = tmp_list_curr_hdr->next;
        }
        // If only one element is available
        if(tmp_list_prev_hdr == NULL) {
            cs_list_add_front(elt_to_add_hdr, list);
        } else {
            tmp_list_prev_hdr->next = elt_to_add_hdr;
            elt_to_add_hdr->next = tmp_list_curr_hdr;
        }
    }
}


/** @} */
