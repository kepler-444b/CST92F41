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
 * @file     drv_tim.h
 * @brief    Header file of TIM HAL module
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup TIM TIM
 * @ingroup  DRIVER
 * @brief    TIM Driver for cst92f41
 * @details  TIM Driver for cst92f41
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 *
 * @example example_timer.c
 * This is an example of how to use the timer
 *
 */

#ifndef __DRV_TIM_H
#define __DRV_TIM_H


/*******************************************************************************
 * INCLUDES
 */
#include "RTE_cst92f4x.h"
#if (RTE_TIM0 || RTE_TIM1 || RTE_TIM2)
#include <stdint.h>
#include "cs_driver.h"

#ifdef __cplusplus
extern "C"
{
#endif


/*******************************************************************************
 * MACROS
 */
#define TIM_ARR_MAX             0xFFFF
#define TIM_PSC_MAX             0xFFFF
#define TIM_32BITS_MAX			(0xFFFF + 0xFFFFU * 0xFFFFU)
#define TIM_1M					1000000

/*******************************************************************************
 * TYPEDEFS
 */
/// TIM Channel
typedef enum {
    /// TIM Channel1
    TIM_CHAN_1      = 0U,
    /// TIM Channel2
    TIM_CHAN_2      = 1U,
    /// TIM Channel3
    TIM_CHAN_3      = 2U,
    /// TIM Channel4
    TIM_CHAN_4      = 3U,

    TIM_CHAN_ALL    = 4U,
} tim_chan_t;

/// TIM Active Level
typedef enum {
    /// Specifies active level is high
    TIM_PWM_POL_ACTIVE_HIGH = 0U,
    /// Specifies active level is low
    TIM_PWM_POL_ACTIVE_LOW  = 1U,
} tim_pwm_pol_t;

/// TIM Force Level
typedef enum {
    /// Specifies force level is inactive
    TIM_FORCE_LEVEL_INACTIVE = 4U,
    /// Specifies force level is active
    TIM_FORCE_LEVEL_ACTIVE   = 5U,
} tim_force_level_t;

/// TIM Capture Polarity
typedef enum {
    /// Capture at rising edge
    TIM_CAPTURE_POLARITY_RISING_EDGE  = 0U,
    /// Capture at falling edge
    TIM_CAPTURE_POLARITY_FALLING_EDGE = 1U,
} tim_capture_polarity_t;

/// TIM General Purpose Configuration
typedef struct {
    /// period(us)
    uint32_t period_us;
} tim_gp_config_t;

/// TIM PWM Channel Configuration
typedef struct {
    /// PWM polarities
    tim_pwm_pol_t pol;
    /// Output compare value
    uint16_t      oc_val;
    //complementary_output_enable
    bool complementary_output_enable;
} tim_pwm_chan_config_t;

#if (RTE_DMA)
/// TIM PWM DMA Configuration
typedef struct {
    /// dma enable or not
    uint8_t en;
    /// tim channel
    tim_chan_t tim_chan;
    /// DMA Chain
    dma_chain_trans_t *chain;
} tim_pwm_dma_config_t;

/// TIM Capture DMA Configuration
typedef struct {
    /// dma enable or not
    uint8_t en;
    /// tim channel
    tim_chan_t tim_chan;
    /// DMA Chain
    dma_chain_trans_t *chain;
} tim_cap_dma_config_t;
#endif

/// TIM PWM Output Configuration
typedef struct {
    uint32_t cnt_freq;                      /*!< Frequency for every count */
    uint16_t period_cnt;                    /*!< Period counter */
    uint16_t dead_time;                     /*!< dead_time generate step */
    struct {
        uint8_t en;                         /*!< Channel enable or not */
        tim_pwm_chan_config_t cfg;          /*!< PWM channel configuration */
    } chan[TIM_CHAN_ALL];
    #if (RTE_DMA)
    tim_pwm_dma_config_t dma_cfg;           /*!< if use dma, pwm dma configuration, only one channel use dma */
    #endif
} tim_pwm_output_config_t;

/// TIM Capture Configuration
typedef struct {
    uint32_t cnt_freq;                      /*!< Frequency for every count */
    struct {
        uint8_t en;                         /*!< Channel enable or not */
        tim_capture_polarity_t pol;         /*!< Channel capture polarity configuration */
    } chan[TIM_CHAN_ALL];
    #if (RTE_DMA)
    tim_cap_dma_config_t dma_cfg;           /*!< if use dma, capture dama configuration, only one channel use dma  */
    #endif
} tim_capture_config_t;

/// TIM PWM Input Configuration
typedef struct {
    uint32_t cnt_freq;                      /*!< Frequency for every count */
} tim_pwm_input_config_t;

/// TIM mode
typedef enum
{
    /// Work as timer mode
    TIM_TIMER_MODE,
    /// Work as PWM mode
    TIM_PWM_MODE,
    /// Work as CAP mode
    TIM_CAP_MODE,
	// Worl as 32-bits mode
	TIM_32BITS_MODE,
} tim_mode_t;


typedef enum
{
    UP_COUNT   = 0,
    DOWN_COUNT = 1,
} tim_cnt_mode_t;
typedef enum
{
	TIMER_32BITS,
	PWM_32BITS,
} tim_32bits_run_mode_t;


/// TIM PWM Channel Configuration
typedef struct {
    /// PWM polarities
    tim_pwm_pol_t pol;
    /// Output compare value
    uint16_t      oc_val;
    //complementary_output_enable
    bool complementary_output_enable;
} tim_pwm_complementary_chan_config_t;



/// TIM PWM Output Configuration
typedef struct {
    uint32_t cnt_freq;                      /*!< Frequency for every count */
    uint16_t period_cnt;                    /*!< Period counter */
    uint16_t dead_time;                     /*!< dead_time generate step */
    struct {
        uint8_t en;                         /*!< Channel enable or not */
        tim_pwm_complementary_chan_config_t cfg;          /*!< PWM channel configuration */
    } chan[TIM_CHAN_ALL];
    #if (RTE_DMA)
    tim_pwm_dma_config_t dma_cfg;           /*!< if use dma, pwm dma configuration, only one channel use dma */
    #endif
} tim_pwm_complementary_output_config_t;



typedef struct
{
	CS_TIM_Type *slave_tim;
	tim_cnt_mode_t counter;
	tim_32bits_run_mode_t run_mode;
	uint32_t time;
	uint32_t delay_period;
	uint32_t freq;
	uint32_t duty[4];
	drv_isr_callback_t callback_uif;
	drv_isr_callback_t callback_cas_uif;
} tim_32bits_config_t;
/*******************************************************************************
 * EXTERN VARIABLES
 */


/*******************************************************************************
 * EXTERN FUNCTIONS
 */
/**
 * @brief Tim initialization
 *
 * @param[in] cs_tim      Pointer to TIM
 *
 * @return errno
 **/
extern cs_error_t drv_tim_init(CS_TIM_Type *cs_tim);

/**
 * @brief Tim uninitialization
 *
 * @param[in] cs_tim      Pointer to TIM
 *
 * @return errno
 **/
extern cs_error_t drv_tim_uninit(CS_TIM_Type *cs_tim);

/**
 * @brief Register tim callback
 *
 * @param[in] cs_tim      Pointer to TIM
 * @param[in] isr_cb      callback
 **/
extern void drv_tim_register_event_callback(CS_TIM_Type *cs_tim, drv_isr_callback_t isr_cb);

/**
 *******************************************************************************
 * @brief Allocate timer dma channel
 *
 * @param[in] cs_tim    Pointer to timer
 *
 * @return errno
 *******************************************************************************
 */
extern cs_error_t drv_tim_dma_channel_allocate(CS_TIM_Type *cs_tim);

/**
 *******************************************************************************
 * @brief Release timer dma channel
 *
 * @param[in] cs_tim    Pointer to timer
 *
 * @return errno
 *******************************************************************************
 */
extern cs_error_t drv_tim_dma_channel_release(CS_TIM_Type *cs_tim);

/**
 * @brief General Purpose Timer Start
 *
 * @param[in] cs_tim      Pointer to TIM
 * @param[in] cfg         general purpose timer config
 *
 * @return errno
 **/
extern cs_error_t drv_tim_gp_start(CS_TIM_Type *cs_tim, const tim_gp_config_t *cfg);

/**
 * @brief General Purpose Timer stop
 *
 * @param[in] cs_tim      Pointer to TIM
 **/
extern void drv_tim_gp_stop(CS_TIM_Type *cs_tim);

/**
 *******************************************************************************
 * @brief Change PWM period
 *
 * @param[in] cs_tim        Pointer to TIM
 * @param[in] period_cnt    The timer automatic reloading value, MAX:65535
 *******************************************************************************
 **/
__STATIC_FORCEINLINE void drv_tim_pwm_change_period_cnt(CS_TIM_Type *cs_tim, uint16_t period_cnt)
{
    cs_tim->ARR = period_cnt;
}

/**
 *******************************************************************************
 * @brief Change PWM channel output compare val
 *
 * @param[in] cs_tim        Pointer to TIM
 * @param[in] channel       PWM channel index
 * @param[in] oc_val        The timer new output compare val, MAX 65535, should be less than period_count
 *******************************************************************************
 **/
__STATIC_FORCEINLINE void drv_tim_pwm_change_oc_val(CS_TIM_Type *cs_tim, tim_chan_t channel, uint16_t oc_val)
{
    cs_tim->CCR[channel] = oc_val;
}

/**
 * @brief TIM PWM output start, use pwm mode1 by default, channel is active as long as TIMx_CNT <
 * TIMx_CCR else inactive.
 *
 * @param[in] cs_tim        Pointer to TIM
 * @param[in] cfg           TIM PWM Output Configuration
 *
 * @return errno
 **/
extern cs_error_t drv_tim_pwm_output_start(CS_TIM_Type *cs_tim, const tim_pwm_output_config_t *cfg);

/**
 * @brief TIM PWM output stop
 *
 * @param[in] cs_tim        Pointer to TIM
 * @param[in] channel       The timer channel, if TIMER_CHANNEL_ALL, stop the TIMx.
 **/
extern void drv_tim_pwm_output_stop(CS_TIM_Type *cs_tim, tim_chan_t channel);

/**
 * @brief Force PWM channel output
 *
 * @param[in] cs_tim        Pointer to TIM
 * @param[in] channel       Channel index
 * @param[in] level         Force output level
 **/
extern void drv_tim_pwm_force_output(CS_TIM_Type *cs_tim, tim_chan_t channel, tim_force_level_t level);

/**
 * @brief Timer capture start
 *
 * @param[in] cs_tim        Pointer to TIM, NOTE:only TIM0 support capature in CST92F41
 * @param[in] cfg           TIM Capture Configuration
 *
 * @return errno
 **/
extern cs_error_t drv_tim_capture_start(CS_TIM_Type *cs_tim, const tim_capture_config_t *cfg);

/**
 * @brief Timer capture stop
 *
 * @param[in] cs_tim        Pointer to TIM
 * @param[in] channel       The tiemr channel, if TIMER_CHANNEL_ALL, stop the TIMx.
 **/
extern void drv_tim_capture_stop(CS_TIM_Type *cs_tim, tim_chan_t channel);

/**
 * @brief Timer PWM input mode start
 *
 * @param[in] cs_tim        Pointer to TIM
 * @param[in] cfg           TIM PWM Input Configuration
 *
 * @return errno
 **/
extern cs_error_t drv_tim_pwm_input_start(CS_TIM_Type *cs_tim, const tim_pwm_input_config_t *cfg);

/**
 * @brief Timer PWM input mode stop
 *
 * @param[in] cs_tim      Pointer to TIM
 **/
extern void drv_tim_pwm_input_stop(CS_TIM_Type *cs_tim);

/**
 * @brief Timer interrupt service routine
 *
 * @param[in] cs_tim      Pointer to TIM
 **/
extern void drv_tim_isr(CS_TIM_Type *cs_tim);

/**
 * @brief TIM PWM complementary output start, use pwm mode1 by default, channel is active as long as TIMx_CNT <
 * TIMx_CCR else inactive.
 *
 * @param[in] cs_tim        Pointer to TIM
 * @param[in] cfg           TIM PWM Output Configuration
 *
 * @return errno
 **/
extern cs_error_t drv_tim_pwm_complementary_output_start(CS_TIM_Type *cs_tim, tim_pwm_complementary_output_config_t *cfg);


/**
 * @brief tim pwm 32bit output start
 *
 * @param[in] cs_tim              timer instance
 * @param[in] cs_timer_slave      slave tim instance
 * @param[in] cfg                 general 32bit timer config
 *
 * @return status see cs_error_t
 **/
extern cs_error_t drv_tim_pwm_32bit_output_start(CS_TIM_Type *cs_tim, CS_TIM_Type *cs_timer_slave, tim_32bits_config_t *cfg);

/**
 * @brief General Purpose Timer 32bit Start
 *
 * @param[in] cs_tim              tim instance
 * @param[in] cs_timer_slave      slave tim instance
 * @param[in] cfg                 general 32bit timer config
 *
 * @return status, see cs_error_t
 **/
extern cs_error_t drv_tim_gp_32bit_start(CS_TIM_Type *cs_tim, CS_TIM_Type *cs_timer_slave, tim_32bits_config_t *cfg);

#ifdef __cplusplus
}
#endif

#endif  /* (RTE_TIM0 || RTE_TIM1 || RTE_TIM2) */

#endif  /* __DRV_TIM_H */


/** @} */