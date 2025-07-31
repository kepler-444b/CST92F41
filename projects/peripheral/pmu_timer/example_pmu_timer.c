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
 * @file     example_pmu_timer.c
 * @brief    example for using pmu timer
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup EXAMPLE_PMU_TIMER PMU TIMER
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using pmu timer
 * @details
 * There is an examples to use pmu timer as follows
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
static volatile uint32_t int0_cnt, int1_cnt;


/*******************************************************************************
 * LOCAL FUNCTIONS
 */
static void pmu_timer_val0_test_callback(void *cs_reg, drv_event_t drv_event, void *param0, void *param1)
{
    int0_cnt++;
    cs_printf("pmu timer, int0_cnt:%d\r\n", int0_cnt);
}

static void pmu_timer_val1_test_callback(void *cs_reg, drv_event_t drv_event, void *param0, void *param1)
{
    int1_cnt++;
    cs_printf("pmu timer, int1_cnt:%d\r\n", int1_cnt);
}


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of using pmu timer
 *
 *******************************************************************************
 */
void example_pmu_timer(void)
{
    uint32_t cnt_val;

    int0_cnt = 0;
    int1_cnt = 0;

    drv_pmu_timer_register_isr_callback(PMU_TIMER_TRIG_VAL0, pmu_timer_val0_test_callback);
    drv_pmu_timer_register_isr_callback(PMU_TIMER_TRIG_VAL1, pmu_timer_val1_test_callback);
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL0, PMU_TIMER_CONTROL_ENABLE, NULL);
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_ENABLE, NULL);
    drv_pmu_timer_init();

    cnt_val = drv_pmu_timer_cnt_get();
    cs_printf("cnt_val0:%ld\r\n", cnt_val);
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL0, PMU_TIMER_CONTROL_SET_TIMER_VAL, (void *)(cnt_val + 10 * 1000));
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_SET_TIMER_VAL, (void *)(cnt_val + 10 * 1000));
    while (int0_cnt < 1 || int1_cnt < 1);

    cnt_val = drv_pmu_timer_cnt_get();
    cs_printf("cnt_val1:%ld\r\n", cnt_val);
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL0, PMU_TIMER_CONTROL_SET_TIMER_VAL, (void *)(cnt_val + 20 * 1000));
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_SET_TIMER_VAL, (void *)(cnt_val + 20 * 1000));
    while (int0_cnt < 2 || int1_cnt < 2);

    cnt_val = drv_pmu_timer_cnt_get();
    cs_printf("cnt_val2:%ld\r\n", cnt_val);
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL0, PMU_TIMER_CONTROL_SET_TIMER_VAL, (void *)(cnt_val + 30 * 1000));
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_SET_TIMER_VAL, (void *)(cnt_val + 30 * 1000));
    while (int0_cnt < 3 || int1_cnt < 3);

    cnt_val = drv_pmu_timer_cnt_get();
    cs_printf("cnt_val3:%ld\r\n", cnt_val);

    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL0, PMU_TIMER_CONTROL_DISABLE, NULL);
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_DISABLE, NULL);
}


/** @} */
