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
 * @file     example_timer.c
 * @brief    example for general purpose timer
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup EXAMPLE_TIMER TIMER
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using timer
 * @details
 * There are some example as follows: using timer to count, output the pwm, capture
 * the signal
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
#include <string.h>
#include "cs_driver.h"


/*******************************************************************************
 * MACROS
 */
/// Test pad for timer counter
#define PAD_TIM0_COUNT          17
/// Test mux for timer counter
#define MUX_TIM0_COUNT          PINMUX_PAD17_GPIO_MODE_CFG
/// Test pad for timer counter
#define PAD_TIM1_COUNT          18
/// Test mux for timer counter
#define MUX_TIM1_COUNT          PINMUX_PAD18_GPIO_MODE_CFG
/// Test pad for timer counter
#define PAD_TIM2_COUNT          19
/// Test mux for timer counter
#define MUX_TIM2_COUNT          PINMUX_PAD19_GPIO_MODE_CFG

/// Test pad for timer pwm
#define PAD_TIM1_PWM_CH1        12
#define PAD_TIM1_PWM_CH2        13
#define PAD_TIM1_PWM_CH3        14
#define PAD_TIM1_PWM_CH4        15
/// Test pad for timer pwm complementary
#define PAD_TIM1_PWM_CH1N       5
/// Test mux for timer pwm
#define MUX_TIM1_PWM_CH1        PINMUX_PAD12_TIMER1_OUT1_CFG
#define MUX_TIM1_PWM_CH2        PINMUX_PAD13_TIMER1_OUT2_CFG
#define MUX_TIM1_PWM_CH3        PINMUX_PAD14_TIMER1_OUT3_CFG
#define MUX_TIM1_PWM_CH4        PINMUX_PAD15_TIMER1_OUT4_CFG
/// Test mux for timer pwm complementary
#define MUX_TIM1_PWM_CH1N       PINMUX_PAD5_TIMER1_OUTN1_CFG
/// Test pad for timer capature
#define PAD_TIM0_CAP_CH1        16
/// Test mux for timer capature
#define MUX_TIM0_CAP_CH1        PINMUX_PAD16_TIMER0_IO1_CFG

/// Number of output compare val
#define PWM_DMA_OC_VAL_NUM      30
/// Number of capture
#define CAP_SAMPLE_NUM          30


/*******************************************************************************
 * TYPEDEFS
 */
typedef struct {
    uint8_t     cap_index;
    uint32_t    cap_overflow;
    uint32_t    cap_buf[CAP_SAMPLE_NUM];
} cap_env_t;

typedef struct {
    dma_chain_trans_t chain[3];
    uint16_t buf0[CAP_SAMPLE_NUM];
    uint16_t buf1[CAP_SAMPLE_NUM];
    uint16_t buf2[CAP_SAMPLE_NUM];
} cap_dma_env_t;

typedef struct {
    dma_chain_trans_t chain[3];
    uint16_t buf0[PWM_DMA_OC_VAL_NUM];
    uint16_t buf1[PWM_DMA_OC_VAL_NUM];
    uint16_t buf2[PWM_DMA_OC_VAL_NUM];
} pwm_dma_env_t;

typedef struct {
    uint32_t cap_overflow;
    uint32_t cap_cc1_index;
    uint32_t cap_cc2_index;
    uint32_t cap_cc1_buf[CAP_SAMPLE_NUM];
    uint32_t cap_cc2_buf[CAP_SAMPLE_NUM];
} pwm_input_env_t;


/*******************************************************************************
 * CONST & VARIABLES
 */
static cap_env_t        cap_env;
static cap_dma_env_t    cap_dma_env;
static pwm_dma_env_t    pwm_dma_env;
static pwm_input_env_t  pwm_input_env;


/*******************************************************************************
 * LOCAL FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief timer counter callback, when the timer generates an update event, toggle
 * the PAD_TIM0_COUNT
 *
 * @param[in] cs_tim    Pointer to timer
 * @param[in] event     Timer event
 * @param[in] buff      Pointer to data
 * @param[in] num       Number of data
 *
 *******************************************************************************
 */
static void tim_count_callback(void *cs_tim, drv_event_t event, void *buff, void *num)
{
    if (event == DRV_EVENT_TIMER_UPDATE) {
        drv_gpio_toggle(CS_GPIO0, BITMASK(PAD_TIM0_COUNT));
    }
}

/**
 *******************************************************************************
 * @brief timer capture callback, The captured data will be put into cap_buf[]
 *
 * @param[in] cs_tim    Pointer to timer
 * @param[in] event     Timer event
 * @param[in] buff      Pointer to data
 * @param[in] num       Number of data
 *
 *******************************************************************************
 */
static void tim_capture_callback(void *cs_tim, drv_event_t event, void *buff, void *num)
{
    if (event & DRV_EVENT_TIMER_UPDATE) {
        cap_env.cap_overflow++;
    }

    if (cap_env.cap_index < CAP_SAMPLE_NUM && event & DRV_EVENT_TIMER_CC1) {
        cap_env.cap_buf[cap_env.cap_index++] = (uint32_t)buff + cap_env.cap_overflow * 0xFFFF;
    }
}

/**
 *******************************************************************************
 * @brief timer pwm input callback, The captured data of the falling and rising
 * edges will be put into cap_cc1_buf, cap_cc2_buf respectively.
 *
 * @param[in] cs_tim    Pointer to timer
 * @param[in] event     Timer event
 * @param[in] buff      Pointer to data
 * @param[in] num       Number of data
 *
 *******************************************************************************
 */
static void tim_pwm_input_callback(void *cs_tim, drv_event_t event, void *buff, void *num)
{
    if (event & DRV_EVENT_TIMER_TRIGGER) {
        pwm_input_env.cap_overflow = 0;
    }

    if (event & DRV_EVENT_TIMER_UPDATE) {
        pwm_input_env.cap_overflow++;
    }

    if (pwm_input_env.cap_cc1_index < CAP_SAMPLE_NUM && (event & DRV_EVENT_TIMER_CC1)) {
        pwm_input_env.cap_cc1_buf[pwm_input_env.cap_cc1_index++] = (uint32_t)buff + pwm_input_env.cap_overflow * 0xFFFF;
    }

    if (pwm_input_env.cap_cc2_index < CAP_SAMPLE_NUM && (event & DRV_EVENT_TIMER_CC2)) {
        pwm_input_env.cap_cc2_buf[pwm_input_env.cap_cc2_index++] = (uint32_t)buff + pwm_input_env.cap_overflow * 0xFFFF;
    }
}


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of using timer as counter, toggle the pin level every 10ms
 *
 *******************************************************************************
 */
void example_tim_count(void)
{
    pin_config_t pin_cfg[] = {
        {PAD_TIM0_COUNT, {MUX_TIM0_COUNT}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    gpio_config_t gpio_cfg[] = {
        {CS_GPIO0, PAD_TIM0_COUNT, GPIO_DIR_OUTPUT, GPIO_LEVEL_LOW, GPIO_TRIG_NONE},
    };
    drv_pin_init(pin_cfg, sizeof(pin_cfg) / sizeof(pin_cfg[0]));
    drv_gpio_init(gpio_cfg, sizeof(gpio_cfg) / sizeof(gpio_cfg[0]));

    drv_tim_init(CS_TIM0);
    drv_tim_register_event_callback(CS_TIM0, tim_count_callback);

    tim_gp_config_t tim_cfg = {
        .period_us = 10 * 1000
    };

    drv_tim_gp_start(CS_TIM0, &tim_cfg);

    /* if you want to stop the timer, just use drv_tim_gp_stop(CS_TIMx) */
}

/**
 *******************************************************************************
 * @brief example of using timer output pwm, the timer1's channel1 will output pwm
 * waveform:period is 1ms, high level is 300us, low level is 700us
 *
 *******************************************************************************
 */
void example_tim_pwm(void)
{
    pin_config_t pin_cfg[] = {
        {PAD_TIM1_PWM_CH1, {MUX_TIM1_PWM_CH1}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    drv_pin_init(pin_cfg, sizeof(pin_cfg) / sizeof(pin_cfg[0]));

    tim_pwm_output_config_t pwm_config = {
        .cnt_freq = 1000000,            // 1Mhz
        .chan = {
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}},
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}},
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}},
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}}
        },
        .dma_cfg = {
            0, TIM_CHAN_ALL, NULL
        },
    };

    pwm_config.period_cnt                   = 1000;
    pwm_config.chan[TIM_CHAN_1].en          = 1;
    pwm_config.chan[TIM_CHAN_1].cfg.oc_val  = 300;
    pwm_config.chan[TIM_CHAN_1].cfg.pol     = TIM_PWM_POL_ACTIVE_HIGH;

    drv_tim_init(CS_TIM1);
    drv_tim_pwm_output_start(CS_TIM1, &pwm_config);

    /**
     * if you want to stop the timer, just use drv_tim_pwm_output_stop(...).
     * if you want to change the output compare value, just use drv_tim_pwm_change_oc_val(...)
     */
}

/**
 *******************************************************************************
 * @brief example of using timer output pwm with complementary and dead time, the timer1's channel1 will output pwm
 * waveform:period is 1ms, high level is 300us, low level is 700us,and output pwm complementary waveform:period is 1ms,
 * high level is 700us, low level is 300us,and the dead time is 3*(1/32M) = 93.75 ns(TIM_BDTR DTG=3,dead_time=DTG[7.0] * Tdtg,Tdtg=tDTS,TIM_CR1 CKD=0x0: tDTS =tCK_INT)
 *******************************************************************************
 */
void example_tim_pwm_complementary(void)
{
    pin_config_t pin_cfg[] = {
        {PAD_TIM1_PWM_CH1, {MUX_TIM1_PWM_CH1}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
		{PAD_TIM1_PWM_CH1N, {MUX_TIM1_PWM_CH1N}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    drv_pin_init(pin_cfg, sizeof(pin_cfg) / sizeof(pin_cfg[0]));

    tim_pwm_complementary_output_config_t pwm_config = {
        .cnt_freq = 1000000,            // 1Mhz
        .chan = {
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}},
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}},
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}},
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}}
        },
        .dma_cfg = {
            0, TIM_CHAN_ALL, NULL
        },
    };

    pwm_config.period_cnt                   = 1000;
    pwm_config.chan[TIM_CHAN_1].en          = 1;
    pwm_config.chan[TIM_CHAN_1].cfg.oc_val  = 300;
    pwm_config.chan[TIM_CHAN_1].cfg.pol     = TIM_PWM_POL_ACTIVE_HIGH;
	pwm_config.chan[TIM_CHAN_1].cfg.complementary_output_enable = true;
	pwm_config.dead_time = 3;
    drv_tim_init(CS_TIM1);
    drv_tim_pwm_complementary_output_start(CS_TIM1, &pwm_config);
    /**
     * if you want to stop the timer, just use drv_tim_pwm_output_stop(...).
     * if you want to change the output compare value, just use drv_tim_pwm_change_oc_val(...)
     */
}

/**
 *******************************************************************************
 * @brief example of using timer+dma ouput pwm with adjustable duty cycle. The
 * duty cycle is stored in pwm_dma_oc_val and gradually increased from 10% to 68%
 * at 2% intervals
 *
 *******************************************************************************
 */
void example_tim_pwm_dma(void)
{
    pin_config_t pin_cfg[] = {
        {PAD_TIM1_PWM_CH1, {MUX_TIM1_PWM_CH1}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    drv_pin_init(pin_cfg, sizeof(pin_cfg) / sizeof(pin_cfg[0]));

    pwm_dma_env.chain[0].src_addr   = (uint32_t)pwm_dma_env.buf0;
    pwm_dma_env.chain[0].size_byte  = PWM_DMA_OC_VAL_NUM * sizeof(uint16_t);
    pwm_dma_env.chain[0].ll_ptr     = &pwm_dma_env.chain[1];

    pwm_dma_env.chain[1].src_addr   = (uint32_t)pwm_dma_env.buf1;
    pwm_dma_env.chain[1].size_byte  = PWM_DMA_OC_VAL_NUM * sizeof(uint16_t);
    pwm_dma_env.chain[1].ll_ptr     = &pwm_dma_env.chain[2];

    pwm_dma_env.chain[2].src_addr   = (uint32_t)pwm_dma_env.buf2;
    pwm_dma_env.chain[2].size_byte  = PWM_DMA_OC_VAL_NUM * sizeof(uint16_t);
    pwm_dma_env.chain[2].ll_ptr     = &pwm_dma_env.chain[0];

    for (uint16_t i = 0; i < PWM_DMA_OC_VAL_NUM; i++) {
        pwm_dma_env.buf0[i] = 200 + i;
        pwm_dma_env.buf1[i] = 400 + i;
        pwm_dma_env.buf2[i] = 600 + i;
    }

    tim_pwm_output_config_t pwm_config = {
        .cnt_freq = 1000000,                // 1Mhz
        .period_cnt = 1000,
        .chan = {
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}},
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}},
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}},
            {0, {TIM_PWM_POL_ACTIVE_HIGH, 0}}
        },
        .dma_cfg = {
            .en = 1,
            .tim_chan = TIM_CHAN_1,
            .chain      = &pwm_dma_env.chain[0]
        },
    };

    pwm_config.chan[TIM_CHAN_1].en   = 1;

    drv_tim_init(CS_TIM1);
    drv_tim_dma_channel_allocate(CS_TIM1);
    drv_tim_pwm_output_start(CS_TIM1, &pwm_config);

    /* if you want to stop the timer, just use drv_tim_pwm_output_stop(...) and release dma channel. */
}

/**
 *******************************************************************************
 * @brief example of using timer capture rising edge. The capture value is stored
 * in cap_env.cap_buf[]
 *
 *******************************************************************************
 */
void example_tim_capture(void)
{
    pin_config_t pin_config[] = {
        {PAD_TIM0_CAP_CH1, {MUX_TIM0_CAP_CH1}, PMU_PIN_MODE_FLOAT, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));

    tim_capture_config_t capture_config = {
        .cnt_freq = 1000000,
        .chan = {
            {0, TIM_CAPTURE_POLARITY_RISING_EDGE},
            {0, TIM_CAPTURE_POLARITY_RISING_EDGE},
            {0, TIM_CAPTURE_POLARITY_RISING_EDGE},
            {0, TIM_CAPTURE_POLARITY_RISING_EDGE}
        },
        .dma_cfg = {
            .en       = 0,
            .tim_chan = TIM_CHAN_ALL,
            .chain    = NULL
        },
    };
    capture_config.chan[TIM_CHAN_1].en = 1;

    drv_tim_init(CS_TIM0);
    drv_tim_register_event_callback(CS_TIM0, tim_capture_callback);
    drv_tim_capture_start(CS_TIM0, &capture_config);

    /* if you want to stop the timer, just use drv_tim_capture_stop(...) */
}

/**
 *******************************************************************************
 * @brief example of using timer+dma capture rising edge. The captured data is
 * placed one by one into cap_dma_env.buf0->buf1->buf2->buf0......
 *
 *******************************************************************************
 */
void example_tim_dma_capture(void)
{
    pin_config_t pin_config[] = {
        {PAD_TIM0_CAP_CH1, {MUX_TIM0_CAP_CH1}, PMU_PIN_MODE_FLOAT, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));

    cap_dma_env.chain[0].dst_addr   = (uint32_t)cap_dma_env.buf0;
    cap_dma_env.chain[0].size_byte  = CAP_SAMPLE_NUM * sizeof(uint16_t);
    cap_dma_env.chain[0].ll_ptr     = &cap_dma_env.chain[1];

    cap_dma_env.chain[1].dst_addr   = (uint32_t)cap_dma_env.buf1;
    cap_dma_env.chain[1].size_byte  = CAP_SAMPLE_NUM * sizeof(uint16_t);
    cap_dma_env.chain[1].ll_ptr     = &cap_dma_env.chain[2];

    cap_dma_env.chain[2].dst_addr   = (uint32_t)cap_dma_env.buf2;
    cap_dma_env.chain[2].size_byte  = CAP_SAMPLE_NUM * sizeof(uint16_t);
    cap_dma_env.chain[2].ll_ptr     = &cap_dma_env.chain[0];

    tim_capture_config_t capture_config = {
        .cnt_freq = 1000000,
        .chan = {
            {0, TIM_CAPTURE_POLARITY_RISING_EDGE},
            {0, TIM_CAPTURE_POLARITY_RISING_EDGE},
            {0, TIM_CAPTURE_POLARITY_RISING_EDGE},
            {0, TIM_CAPTURE_POLARITY_RISING_EDGE}
        },
        .dma_cfg = {
            .en = 1,
            .tim_chan = TIM_CHAN_1,
            .chain = &cap_dma_env.chain[0]
        }
    };
    capture_config.chan[TIM_CHAN_1].en = 1;

    drv_tim_init(CS_TIM0);
    drv_tim_dma_channel_allocate(CS_TIM0);
    drv_tim_capture_start(CS_TIM0, &capture_config);

    /* if you want to stop the timer, just use drv_tim_capture_stop(...) and release dma channel*/
}

/**
 *******************************************************************************
 * @brief example of using timer pwm input mode. TI1FP1 captures the falling edge,
 * TI1FP2 captures the rising edge, when TI1FP1 captures, the tim counter is reset.
 *
 *******************************************************************************
 */
void example_tim_pwm_input(void)
{
    pin_config_t pin_config[] = {
        {PAD_TIM0_CAP_CH1, {MUX_TIM0_CAP_CH1}, PMU_PIN_MODE_FLOAT, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));

    tim_pwm_input_config_t cfg = {
        .cnt_freq = 1000000
    };

    drv_tim_init(CS_TIM0);
    drv_tim_register_event_callback(CS_TIM0, tim_pwm_input_callback);
    drv_tim_pwm_input_start(CS_TIM0, &cfg);

    /* if you want to stop the timer, just use drv_tim_pwm_input_stop(...) */
}

void tim_32bits_handler_cascade(void *cs_tim, drv_event_t drv_event, void *buff, void *num)
{
	CS_TIM_Type *cs_timer = (CS_TIM_Type *)cs_tim;
	CS_TIM_Type *cs_timer_slave = CS_TIM2;
    if (drv_event == DRV_EVENT_TIMER_CASCADE_UPDATE) {
		cs_printf("DRV_EVENT_TIMER_CASCADE_UPDATE\n");
	    cs_printf("TIM1 32BITS cs_timer->CNT= %d\n", cs_timer->CNT);
	    cs_printf("TIM1 32BITS cs_timer_slave->CNT= %d\n", cs_timer_slave->CNT);
		cs_timer_slave->SR = 0;	// Must clean slave_tim uif
    }
}

void example_timer_32bit_counter_start(void)
{

	drv_tim_init(CS_TIM1);
	drv_tim_init(CS_TIM2);

	tim_32bits_config_t tim_cas_config = {
		.slave_tim = CS_TIM2,
		.run_mode = TIMER_32BITS,
		.counter = 1,
		.time = 3000000,
		.delay_period = 0,
		.callback_cas_uif = (drv_isr_callback_t)tim_32bits_handler_cascade,
	};
    drv_tim_register_event_callback(CS_TIM1, tim_32bits_handler_cascade);
	// counter 32bit start
	drv_tim_gp_32bit_start(CS_TIM1,CS_TIM2, &tim_cas_config);
	
    /* if you want to stop the timer, just use drv_tim_gp_stop(CS_TIMx) */
}

void example_timer_32bit_pwm_start(void)
{
	drv_tim_init(CS_TIM1);
	drv_tim_init(CS_TIM2);

    pin_config_t pin_cfg[] = {
        {PAD_TIM1_PWM_CH1, {MUX_TIM1_PWM_CH1}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
        {PAD_TIM1_PWM_CH2, {MUX_TIM1_PWM_CH2}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
        {PAD_TIM1_PWM_CH3, {MUX_TIM1_PWM_CH3}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
        {PAD_TIM1_PWM_CH4, {MUX_TIM1_PWM_CH4}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    };
    drv_pin_init(pin_cfg, sizeof(pin_cfg) / sizeof(pin_cfg[0]));

	tim_32bits_config_t tim_cas_config = {
		.slave_tim = CS_TIM2,
		.run_mode = PWM_32BITS,
		.counter = 0,
		.freq = 10000,
		.duty = {15, 25, 35, 45},
	};
	// PWM 32bit start
	drv_tim_pwm_32bit_output_start(CS_TIM1,CS_TIM2, &tim_cas_config);
	
    /*if you want to stop the timer, just use drv_tim_pwm_output_stop(...)*/

}

/** @} */
