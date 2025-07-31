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
 * @file     example_rtc.c
 * @brief    example for rtc
 * @date     12 Feb 2024
 * @author   chipsea
 *
 * @defgroup EXAMPLE_RTC RTC
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using rtc
 * @details
 * There is an example to use rtc as follows: config second and alarm interrupt
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
#include "cs_driver.h"


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * CONST & VARIABLES
 */


/*******************************************************************************
 * LOCAL FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief second interrupt callback
 *
 * @param[in] tm    Pointer to current time
 *
 *******************************************************************************
 */
static void rtc_second_cb(const rtc_tm_t *tm)
{
    cs_printf("second cb\r\n");
}

/**
 *******************************************************************************
 * @brief alarm interrupt callback
 *
 * @param[in] event    Alarm event
 * @param[in] tm       Pointer to current time
 *
 *******************************************************************************
 */
static void rtc_alarm_cb(rtc_alarm_event_t event, const rtc_tm_t *tm)
{
    if (event == RTC_ALARM_EVENT0) {
        cs_printf("alarm  event0\r\n");
    } else if (event == RTC_ALARM_EVENT1) {
        cs_printf("alarm  event1\r\n");
    } else if (event == RTC_ALARM_EVENT2) {
        cs_printf("alarm  event2\r\n");
    }
}


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of using rtc second and alarm function
 *        1. "second cb" will be printed every second
 *        2. "alarm event[n]" will be printed after 2s, 5s, 9s from the beginning
 *
 *******************************************************************************
 */
void example_rtc(void)
{
    rtc_tm_t set_tm;
    rtc_tm_t alarm_tm;

    set_tm.tm_year = 2024 - 1900;   // years from 1900
    set_tm.tm_mon  = 12 - 1;         // months [0, 11]
    set_tm.tm_mday = 23;            // day [1, 31]
    set_tm.tm_hour = 9;            // hours [0, 23]
    set_tm.tm_min  = 15;            // min [0, 59]
    set_tm.tm_sec  = 0;            // sec [0, 59]
    set_tm.tm_wday = 1;             // wday [0, 6]

    memcpy(&alarm_tm, &set_tm, sizeof(rtc_tm_t));

    drv_rtc_init(CS_RTC);

    drv_rtc_register_second_isr_callback(CS_RTC, rtc_second_cb);
    drv_rtc_register_alarm_isr_callback(CS_RTC, rtc_alarm_cb);

    alarm_tm.tm_sec += 2;
    drv_rtc_alarm_set(CS_RTC, RTC_ALARM_EVENT0, &alarm_tm);
    alarm_tm.tm_sec += 3;
    drv_rtc_alarm_set(CS_RTC, RTC_ALARM_EVENT1, &alarm_tm);
    alarm_tm.tm_sec += 4;
    drv_rtc_alarm_set(CS_RTC, RTC_ALARM_EVENT2, &alarm_tm);

    // start signal
    drv_rtc_timer_set(CS_RTC, &set_tm);
}


/** @} */
