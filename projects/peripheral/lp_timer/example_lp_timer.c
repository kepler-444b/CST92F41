/* ----------------------------------------------------------------------------
 * Copyright (c) 2020-2030 Chipsea Limited. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of Chipsea Microelectronics nor the names of its contributors
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
 * @file     example_lp_timer.c
 * @brief    example for low power timer
 * @date     3 Feb 2023
 * @author   Chipsea SW Team
 *
 * @defgroup EXAMPLE_LP_TIMER LP TIMER
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using lp timer
 * @details
 * There are some example as follows: various modes of using lp timer: free running mode,
 * buffered mode, one shot mode, double mode
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
/// Test pad for lp timer out0
#define PAD_LP_TIM_OUT0         18
/// Test pad for lp timer out1
#define PAD_LP_TIM_OUT1         15

/// Test pad for lp timer counter
#define PAD_LP_TIM_COUNT          14
/// Test mux for lp timer counter
#define MUX_LP_TIM_COUNT          PINMUX_PAD14_GPIO_MODE_CFG


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
 * @brief lp timer counter callback, when the lp timer generates a REP0 event 
 * or an UNDER_FLOW event, toggle the PAD_LP_TIM_COUNT
 *
 * @param[in] cs_lp_tim    Pointer to lp timer
 * @param[in] event        LP timer event
 * @param[in] buff         Pointer to data
 * @param[in] num          Number of data
 *
 *******************************************************************************
 */
static void lp_tim_count_callback(void *cs_lp_tim, drv_event_t event, void *buff, void *num)
{
    if ((event & DRV_EVENT_LP_TIMER_REP0) && (((CS_LP_TIM_Type *)cs_lp_tim)->INTE & LP_TIM_INTE_REP0_EN_MASK)){
        drv_gpio_toggle(CS_GPIO0, BITMASK(PAD_LP_TIM_COUNT));
        drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_STOP, NULL);
    }
    if ((event & DRV_EVENT_LP_TIMER_UNDER_FLOW) && (((CS_LP_TIM_Type *)cs_lp_tim)->INTE & LP_TIM_INTE_UF_EN_MASK)){
        drv_gpio_toggle(CS_GPIO0, BITMASK(PAD_LP_TIM_COUNT));
    }
}


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of using lp timer as counter with oneshot mode.
 *in this mode the count is stoped when sep0 is reduced to 0. 
 * toggle the pin level when the lp timer reaches 3 seconds
 *
 *******************************************************************************
 */
void example_lp_tim_oneshot_count(void)
{
    uint8_t i = 1;
    lp_tim_one_shot_config_t    mode_cfg;
    
    pin_config_t pin_cfg[] = {
        {PAD_LP_TIM_COUNT, {MUX_LP_TIM_COUNT}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    gpio_config_t gpio_cfg[] = {
        {CS_GPIO0, PAD_LP_TIM_COUNT, GPIO_DIR_OUTPUT, GPIO_LEVEL_LOW, GPIO_TRIG_NONE},
    };
    drv_pin_init(pin_cfg, sizeof(pin_cfg) / sizeof(pin_cfg[0]));
    drv_gpio_init(gpio_cfg, sizeof(gpio_cfg) / sizeof(gpio_cfg[0]));

    mode_cfg.presclar       = 0;
    mode_cfg.rep0_val       = 2;//8bits reg,max 0xFF(255).the value 0 and 255 represents a loop count of 1 and 256 rounds.so the final number needs to be subtracted by one.
    mode_cfg.top_en         = 1;
    mode_cfg.top_val        = 32*1000-1;//32*1000-1;//16bits reg,max 0xFFFF(65535).the value 32 represents one millisecond.the final number needs to be subtracted by one.
    mode_cfg.compare_val0   = 0;
    mode_cfg.compare_val1   = 0;
    drv_lp_tim_one_shot_init(CS_LP_TIM, &mode_cfg);
    drv_lp_tim_register_isr_callback(CS_LP_TIM, lp_tim_count_callback);

    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_INT_EN, (void *)(LP_TIM_INTE_REP0_EN_MASK | LP_TIM_INTE_UF_EN_MASK));
    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_POWER_IN_SLEEP, &i);
    drv_pmu_32k_enable_in_deep_sleep(true);

    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_START, NULL);
    
    drv_gpio_toggle(CS_GPIO0, BITMASK(PAD_LP_TIM_COUNT));
}
 
/**
 *******************************************************************************
 * @brief example of using lp timer as counter with free_running mode. 
 *in this mode the count is continuous until a stop command is sent. 
 *toggle the pin level when the lp timer reaches 2 seconds
 *
 *******************************************************************************
 */
void example_lp_tim_free_running_count(void)
{
    uint8_t i = 1;
    lp_tim_free_running_config_t    mode_cfg;
    
    pin_config_t pin_cfg[] = {
        {PAD_LP_TIM_COUNT, {MUX_LP_TIM_COUNT}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    gpio_config_t gpio_cfg[] = {
        {CS_GPIO0, PAD_LP_TIM_COUNT, GPIO_DIR_OUTPUT, GPIO_LEVEL_LOW, GPIO_TRIG_NONE},
    };
    drv_pin_init(pin_cfg, sizeof(pin_cfg) / sizeof(pin_cfg[0]));
    drv_gpio_init(gpio_cfg, sizeof(gpio_cfg) / sizeof(gpio_cfg[0]));

    mode_cfg.presclar       = 0;
    mode_cfg.top_en         = 1;
    mode_cfg.top_val        = 31;//32*2000-1;//16bits reg,max 0xFFFF(65535).the value 32 represents one millisecond.the final number needs to be subtracted by one.
    mode_cfg.compare_val0   = 0;
    mode_cfg.compare_val1   = 0;
    drv_lp_tim_free_running_init(CS_LP_TIM, &mode_cfg);
    drv_lp_tim_register_isr_callback(CS_LP_TIM, lp_tim_count_callback);

    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_INT_EN, (void *)LP_TIM_INTE_UF_EN_MASK);
    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_POWER_IN_SLEEP, &i);
    drv_pmu_32k_enable_in_deep_sleep(true);

    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_START, NULL);
    
    drv_gpio_toggle(CS_GPIO0, BITMASK(PAD_LP_TIM_COUNT));
}

/**
 *******************************************************************************
 * @brief example of continuous pulse generation using lp timer
 *
 *******************************************************************************
 */
void example_lp_tim_free_running_pwm(void)
{
    uint8_t i = 1;
    lp_tim_free_running_config_t    mode_cfg;
    lp_tim_out_config_t             out_cfg;

    mode_cfg.presclar       = 0;
    mode_cfg.top_en         = 1;
    mode_cfg.top_val        = 500;
    mode_cfg.compare_val0   = 0;
    mode_cfg.compare_val1   = 0;
    drv_lp_tim_free_running_init(CS_LP_TIM, &mode_cfg);

    pin_config_t pin_cfg_cnt [] = {
        {PAD_LP_TIM_OUT0,  {PINMUX_LP_TIMER_OUT0_CFG},  PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    drv_pin_init(pin_cfg_cnt, sizeof(pin_cfg_cnt) / sizeof(pin_cfg_cnt[0]));
    out_cfg.pol         = LP_TIM_POL_IDLE_LOW;
    out_cfg.action      = LP_TIM_UFOA_PULSE;
    drv_lp_tim_outx_config(CS_LP_TIM, LP_TIM_CHAN_OUT0, &out_cfg);
	
    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_POWER_IN_SLEEP, &i);
    drv_pmu_32k_enable_in_deep_sleep(true);

    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_START, NULL);
}

/**
 *******************************************************************************
 * @brief example of generating (rep0+1) pulses using lp time
 *
 *******************************************************************************
 */
void example_lp_tim_oneshot_pwm(void)
{
    uint8_t i = 1;
    lp_tim_one_shot_config_t    mode_cfg;
    lp_tim_out_config_t         out_cfg;

    mode_cfg.presclar       = 0;
    mode_cfg.rep0_val       = 3;
    mode_cfg.top_en         = 1;
    mode_cfg.top_val        = 500;
    mode_cfg.compare_val0   = 0;
    mode_cfg.compare_val1   = 0;
    drv_lp_tim_one_shot_init(CS_LP_TIM, &mode_cfg);

    pin_config_t pin_cfg_cnt [] = {
        {PAD_LP_TIM_OUT0,  {PINMUX_LP_TIMER_OUT0_CFG},  PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    drv_pin_init(pin_cfg_cnt, sizeof(pin_cfg_cnt) / sizeof(pin_cfg_cnt[0]));
    out_cfg.pol         = LP_TIM_POL_IDLE_LOW;
    out_cfg.action      = LP_TIM_UFOA_PULSE;
    drv_lp_tim_outx_config(CS_LP_TIM, LP_TIM_CHAN_OUT0, &out_cfg);

    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_POWER_IN_SLEEP, &i);
    drv_pmu_32k_enable_in_deep_sleep(true);
	
    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_START, NULL);
}

/**
 *******************************************************************************
 * @brief example of using lp timer to generate a specified number of pulses
 * with a specified period
 *
 *******************************************************************************
 */
void example_lp_tim_buffered_pwm(void)
{
    uint8_t i = 1;
    lp_tim_buffered_config_t    mode_cfg;
    lp_tim_out_config_t         out_cfg;

    mode_cfg.presclar       = 0;
    mode_cfg.rep0_val       = 3;
    mode_cfg.rep1_val       = 6;
    mode_cfg.top_en         = 1;
    mode_cfg.top_val        = 500;
    mode_cfg.buftop_en      = 1;
    mode_cfg.buftop_val     = 1000;
    mode_cfg.compare_val0   = 0;
    mode_cfg.compare_val1   = 0;
    drv_lp_tim_buffered_init(CS_LP_TIM, &mode_cfg);

    pin_config_t pin_cfg_cnt [] = {
        {PAD_LP_TIM_OUT0,  {PINMUX_LP_TIMER_OUT0_CFG},  PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    drv_pin_init(pin_cfg_cnt, sizeof(pin_cfg_cnt) / sizeof(pin_cfg_cnt[0]));
    out_cfg.pol         = LP_TIM_POL_IDLE_LOW;
    out_cfg.action      = LP_TIM_UFOA_PULSE;
    drv_lp_tim_outx_config(CS_LP_TIM, LP_TIM_CHAN_OUT0, &out_cfg);

    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_POWER_IN_SLEEP, &i);
    drv_pmu_32k_enable_in_deep_sleep(true);
	
    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_START, NULL);
}

/**
 *******************************************************************************
 * @brief example of using lp timer to generate two-way signals
 *
 *******************************************************************************
 */
void example_lp_tim_double_pwm(void)
{
    uint8_t i = 1;
    lp_tim_double_config_t      mode_cfg;
    lp_tim_out_config_t         out_cfg;

    mode_cfg.presclar       = 0;
    mode_cfg.rep0_val       = 3;
    mode_cfg.rep1_val       = 5;
    mode_cfg.top_en         = 1;
    mode_cfg.top_val        = 500;
    mode_cfg.compare_val0   = 0;
    mode_cfg.compare_val1   = 0;
    drv_lp_tim_double_init(CS_LP_TIM, &mode_cfg);

    pin_config_t pin_cfg_cnt [] = {
        {PAD_LP_TIM_OUT0,  {PINMUX_LP_TIMER_OUT0_CFG},  PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
        {PAD_LP_TIM_OUT1,  {PINMUX_LP_TIMER_OUT1_CFG},  PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    drv_pin_init(pin_cfg_cnt, sizeof(pin_cfg_cnt) / sizeof(pin_cfg_cnt[0]));
    out_cfg.pol         = LP_TIM_POL_IDLE_LOW;
    out_cfg.action      = LP_TIM_UFOA_PULSE;
    drv_lp_tim_outx_config(CS_LP_TIM, LP_TIM_CHAN_OUT0, &out_cfg);
    drv_lp_tim_outx_config(CS_LP_TIM, LP_TIM_CHAN_OUT1, &out_cfg);

    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_POWER_IN_SLEEP, &i);
    drv_pmu_32k_enable_in_deep_sleep(true);
	
    drv_lp_tim_control(CS_LP_TIM, LP_TIM_CONTROL_START, NULL);
}


/** @} */
