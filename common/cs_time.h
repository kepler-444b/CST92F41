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
 * @file     cs_time.h
 * @brief
 * @date     06. Aug 2020
 * @author   chipsea
 *
 * @defgroup CS_TIME Time
 * @ingroup  COMMON
 * @brief    Time
 * @details  Time
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __CS_TIME_H
#define __CS_TIME_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "cs_driver.h"

/*********************************************************************
 * MACROS
 */

/// Max timer tick
#define CS_TIMER_MAX_TICK       PMU_TIMER_MAX_TICK
/// Max timer delay
#define CS_TIMER_MAX_DELAY      (CS_TIMER_MAX_TICK/2 - 1)
/// Invalid time
#define CS_TIME_INVALID         0xFFFFFFFFU


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
 * @brief get current stack time(tick)
 *
 * @return current time, uint is 30.5us
 **/
__STATIC_FORCEINLINE uint32_t cs_time(void)
{
    return drv_pmu_timer_cnt_get();
}

/**
 * @brief convert time to safe
 *
 * @param[in] time  time
 *
 * @return safe time
 **/
__STATIC_FORCEINLINE uint32_t cs_time_to_safe(uint32_t time)
{
#if (CS_TIMER_MAX_TICK == 0xFFFFFFFF)
    return time;
#else
    return time & CS_TIMER_MAX_TICK;
#endif
}

/**
 * @brief time increase +
 *
 * @param[in] time  time
 * @param[in] increase  +value
 *
 * @return increase result
 **/
__STATIC_FORCEINLINE uint32_t cs_time_increase(uint32_t time, uint32_t increase)
{
#if (CS_TIMER_MAX_TICK == 0xFFFFFFFF)
    return time + increase;
#else
    CS_ASSERT(time <= CS_TIMER_MAX_TICK);
    return cs_time_to_safe(time + increase);
#endif
}

/**
 * @brief time decrease -
 *
 * @param[in] time  time value
 * @param[in] decrease  -value
 *
 * @return decrease result
 **/
__STATIC_FORCEINLINE uint32_t cs_time_decrease(uint32_t time, uint32_t decrease)
{
#if (CS_TIMER_MAX_TICK == 0xFFFFFFFF)
    return time - decrease;
#else
    CS_ASSERT(time <= CS_TIMER_MAX_TICK);
    return (time < decrease) ? (CS_TIMER_MAX_TICK + 1 - (decrease - time)) : (time - decrease);
#endif
}

/**
 * @brief different between with two time
 *
 * @param[in] me  one time
 * @param[in] he  another time
 *
 * @return different
 **/
__STATIC_FORCEINLINE uint32_t cs_time_diff(uint32_t me, uint32_t he)
{
#if (CS_TIMER_MAX_TICK == 0xFFFFFFFF)
    uint32_t diff1 = me - he;
    uint32_t diff2 = he - me;
#else
    uint32_t diff1 = (me > he) ? (me - he) : (he - me);
    uint32_t diff2 = CS_TIMER_MAX_TICK - diff1 + 1;
#endif

    return CS_MIN(diff1, diff2);
}

/**
 * @brief compare between with two time
 *
 * @param[in] me  one time
 * @param[in] he  another time
 *
 * @return if me is smaller or equeal, return true, otherwise return false.
 **/
__STATIC_FORCEINLINE bool cs_time_compare_equal_lesser(uint32_t me, uint32_t he)
{
#if (CS_TIMER_MAX_TICK == 0xFFFFFFFF)
    return (he - me < CS_TIMER_MAX_DELAY);
#else
    return (me > he) ? (me - he > CS_TIMER_MAX_DELAY) : (he - me < CS_TIMER_MAX_DELAY);
#endif
}

/**
 * @brief compare between with two time
 *
 * @param[in] me  one time
 * @param[in] he  another time
 *
 * @return if me is smaller, return true, otherwise return false.
 **/
__STATIC_FORCEINLINE bool cs_time_compare_lesser(uint32_t me, uint32_t he)
{
    return (me != he) && cs_time_compare_equal_lesser(me, he);
}

/**
 * @brief compare between with two time
 *
 * @param[in] me  one time
 * @param[in] he  another time
 *
 * @return if me is greater or equeal, return true, otherwise return false.
 **/
__STATIC_FORCEINLINE bool cs_time_compare_equal_greater(uint32_t me, uint32_t he)
{
    return (me == he) || !cs_time_compare_equal_lesser(me, he);
}

/**
 * @brief compare between with two time
 *
 * @param[in] me  one time
 * @param[in] he  another time
 *
 * @return if me is greater, return true, otherwise return false.
 **/
__STATIC_FORCEINLINE bool cs_time_compare_greater(uint32_t me, uint32_t he)
{
    return !cs_time_compare_equal_lesser(me, he);
}

/**
 * @brief whether time is pasted
 *
 * @param[in] time  time
 * @param[in] cur_time  current time
 *
 * @return whether pasted
 **/
__STATIC_FORCEINLINE bool cs_time_past(uint32_t time, uint32_t cur_time)
{
    return cs_time_compare_equal_lesser(time, cur_time);
}

/**
 * @brief time pasting time
 *
 * @param[in] time  time
 * @param[in] cur_time  current time
 *
 * @return 0:pasted, >0:will past time
 **/
__STATIC_FORCEINLINE uint32_t cs_time_pasting_time(uint32_t time, uint32_t cur_time)
{
    uint32_t delay;

#if (CS_TIMER_MAX_TICK == 0xFFFFFFFF)
    delay = time - cur_time;
#else
    if (time > cur_time)
        delay = time - cur_time;
    else
        delay = CS_TIMER_MAX_TICK - cur_time + time;
#endif

    return (delay>CS_TIMER_MAX_DELAY) ? 0 : delay;
}

/**
 * @brief time delay tick is pasted
 *
 * @param[in] delay  delay with tick
 * @param[in] prev_past_time  prev delay time, if delay pasted, the value will be modify to current time
 *
 * @return pasted
 **/
__STATIC_FORCEINLINE bool cs_time_delay_past(uint32_t delay, uint32_t *prev_past_time)
{
    bool pasted;
    uint32_t cur_time = cs_time();

    if(*prev_past_time == CS_TIMER_MAX_TICK)
        *prev_past_time = cur_time;

    pasted = cs_time_diff(cur_time, *prev_past_time) > delay;
    if(pasted)
        *prev_past_time = cur_time;

    return pasted;
}

/**
 * @brief time intersection
 *
 * @param[in] time1  time1
 * @param[in] time1_duration  time1_duration
 * @param[in] time2  time2
 * @param[in] time2_duration  time2_duration
 *
 * @return whether is intersection
 **/
__STATIC_FORCEINLINE bool cs_time_intersection(uint32_t time1, uint32_t time1_duration,
                                        uint32_t time2, uint32_t time2_duration)
{
    uint32_t time1_tail = cs_time_increase(time1, time1_duration);
    uint32_t time2_tail = cs_time_increase(time2, time2_duration);

    return !(cs_time_compare_equal_lesser(time2_tail, time1) || cs_time_compare_equal_lesser(time1_tail, time2));
}

#ifdef __cplusplus
}
#endif

#endif

/** @} */

