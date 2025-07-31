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
 * @file     drv_pmu_timer.h
 * @brief    Header file of pmu timer HAL module
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup PMU_TIMER PMU_TIMER
 * @ingroup  DRIVER
 * @brief    PMU TIMER Driver for cst92f41
 * @details  PMU TIMER Driver for cst92f41
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 *
 * @example example_pmu_timer.c
 * This is an example of how to use the pmu timer
 *
 */

#ifndef __DRV_PMU_TIMER_H
#define __DRV_PMU_TIMER_H


/*******************************************************************************
 * INCLUDES
 */
#include "RTE_cst92f4x.h"
#if (RTE_PMU_TIMER)
#include "cs_driver.h"
#include "cs_device.h"


#ifdef __cplusplus
extern "C"
{
#endif


/*******************************************************************************
 * MACROS
 */
/// Max Tick
#define PMU_TIMER_MAX_TICK                          0xFFFFFFFFU
/// Max Delay Tick
#define PMU_TIMER_MAX_DELAY                         (PMU_TIMER_MAX_TICK / 2 - 1)
/// Convert ms to tick
#define PMU_TIMER_MS2TICK(time)                     ((uint32_t)((((uint64_t)(time)) << 12) / 125))
/// Convert us to tick
#define PMU_TIMER_US2TICK(time)                     ((uint32_t)((((uint64_t)(time)) << 9) / 15625))
/// Convert tick to ms
#define PMU_TIMER_TICK2MS(tick)                     ((uint32_t)((((uint64_t)(tick)) * 125) >> 12))
// Convert tick to us
#define PMU_TIMER_TICK2US(tick)                     ((uint32_t)((((uint64_t)(tick)) * 15625) >> 9))

//#define PMU_TIMER_DEBUG

/*******************************************************************************
 * TYPEDEFS
 */
/// Trigger Source
typedef enum {
    /// Trigger for VAL0
    PMU_TIMER_TRIG_VAL0,
    /// Trigger for VAL1
    PMU_TIMER_TRIG_VAL1,
    /// Trigger for none
    PMU_TIMER_TRIG_NONE,
} pmu_timer_trig_t;

/// PMU Timer Control
typedef enum {
    PMU_TIMER_CONTROL_ENABLE,                     /*!< Not used */
    PMU_TIMER_CONTROL_DISABLE,                    /*!< Not used */
    PMU_TIMER_CONTROL_SET_TIMER_VAL,              /*!< Set PMU timer val regsiter, argu is timer val, return CS_ERROR_OK */
    PMU_TIMER_CONTROL_GET_TIMER_VAL,              /*!< Get PMU timer val */
    PMU_TIMER_CONTROL_GET_OVERFLOW,               /*!< Get PMU timer interrupt flag, argu is NULL, return overflow */
    PMU_TIMER_CONTROL_SET_TIMER_INCR,             /*!< Set PMU timer ppm regsiter, argu is timer incr, return CS_ERROR_OK */
    PMU_TIMER_CONTROL_GET_TRIG_INT_STATE,         /*!< Get PMU timer interrupt enable flag */
} pmu_timer_control_t;

/// PMU Timer Env
typedef struct {
    /// trigger callback for val0
    drv_isr_callback_t    isr0_cb;
    /// trigger callback for val1
    drv_isr_callback_t    isr1_cb;

    /// Workaround pmu timer write-delay-read issue for val0
    __IO uint32_t         TIMER_SET0;
    /// Workaround pmu timer write-delay-read issue for val1
    __IO uint32_t         TIMER_SET1;

#ifdef PMU_TIMER_DEBUG
    struct {
        __IO bool       is_enabled;
        __IO uint32_t   value;
        __IO uint32_t   cur_cnt_when_set;
        __IO uint32_t   cur_cnt_when_overflow;
    }record[2];
#endif
} pmu_timer_env_t;


/*******************************************************************************
 * EXTERN VARIABLES
 */
extern pmu_timer_env_t pmu_timer_env;


/*******************************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief initialize pmu_timer
 *******************************************************************************
 */
extern void drv_pmu_timer_init(void);

/**
 *******************************************************************************
 * @brief pmu timer register callback
 *
 * @param[in] trig_type     trigger type
 * @param[in] cb            callback
 *******************************************************************************
 */
extern void drv_pmu_timer_register_isr_callback(pmu_timer_trig_t trig_type, drv_isr_callback_t cb);

/**
 *******************************************************************************
 * @brief The interrupt callback for PMU Timer driver. It is a weak function. User should define
 *        their own callback in user file, other than modify it in the PMU Timer driver.
 *
 * @param cs_pmu_timer      The PMU Timer device address
 * @param trig_type         Trigger type
 * @param event             Driver event
 *                           - DRV_EVENT_COMMON_GENERAL
 *******************************************************************************
 */
extern __WEAK void drv_pmu_timer_isr_callback(CS_PMU_Type *cs_pmu_timer, pmu_timer_trig_t trig_type, drv_event_t event);

/**
 *******************************************************************************
 * @brief get pmu timer left time(ms)
 *
 * @param[in] trig_type    trigger type
 *
 * @return
 *  - 0xFFFFFFFF : invalid time
 *  - others : left time(tick)
 *******************************************************************************
 */
extern uint32_t drv_pmu_timer_left_time_get(pmu_timer_trig_t trig_type);

/**
 *******************************************************************************
 * @brief pmu timer interrupt service routine
 *
 *******************************************************************************
 */
extern void drv_pmu_timer_isr(void);

/**
 *******************************************************************************
 * @brief calculate counter increment by pmu timer frequency
 *
 * @param[in] real_freq    real frequency
 *
 * @return incr
 *******************************************************************************
 */
__STATIC_FORCEINLINE uint32_t pmu_timer_calc_incr_by_real_freq(uint32_t real_freq)
{
    uint32_t old_incr, new_incr;
    uint64_t tmp_val;
    uint32_t tmp_val_lo, tmp_val_hi;

    old_incr = CS_PMU->TIMER_PPM & 0x1FFFF;

    tmp_val = (uint64_t)old_incr * (((uint64_t)32768 << 16) / real_freq);

    // binary point position is at [bit32 . bit31]
    tmp_val_lo = tmp_val & 0xFFFFFFFF;
    tmp_val_hi = (tmp_val >> 32) & 0xFFFFFFFF;

    new_incr = ((tmp_val_hi & 0x1) << 16) | (tmp_val_lo >> 16);

    CS_ASSERT(new_incr != 0U);

    return new_incr;
}

/**
 *******************************************************************************
 * @brief calculate counter increment by recording pmu timer's count value within time_us
 *
 * @param[in] time_us       record time
 * @param[in] pmu_tim_cnt   record count
 *
 * @return incr
 *******************************************************************************
 */
__STATIC_FORCEINLINE uint32_t pmu_timer_calc_incr_by_time(uint32_t time_us, uint32_t pmu_tim_cnt)
{
    return pmu_timer_calc_incr_by_real_freq((uint64_t)(pmu_tim_cnt) * 1000000 / time_us);
}

/**
 *******************************************************************************
 * @brief get pmu timer count
 *
 * @return current count
 *******************************************************************************
 */
__STATIC_FORCEINLINE uint32_t drv_pmu_timer_cnt_get(void)
{
    CS_PMU_Type *pmu_timer = CS_PMU;

    return pmu_timer->TIMER_READ;
}

/**
 *******************************************************************************
 * @brief pmu timer control
 *
 * @param[in] trig_type     trigger type
 * @param[in] control       control type
 * @param[in] argu          control content
 *
 * @return control status
 *******************************************************************************
 */
__STATIC_FORCEINLINE void *drv_pmu_timer_control(pmu_timer_trig_t trig_type, pmu_timer_control_t control, void *argu)
{
    CS_PMU_Type *pmu_timer = CS_PMU;

    switch (control) {
        case PMU_TIMER_CONTROL_ENABLE:
            break;
        case PMU_TIMER_CONTROL_DISABLE:
            CS_CRITICAL_BEGIN();
            if (trig_type == PMU_TIMER_TRIG_VAL0) {
                REGW0(&pmu_timer->TIMER_CTRL, PMU_TIMER_CTRL_PMU_TIMER_INT_VAL0_EN_MASK);
            } else if (trig_type == PMU_TIMER_TRIG_VAL1) {
                REGW0(&pmu_timer->TIMER_CTRL, PMU_TIMER_CTRL_PMU_TIMER_INT_VAL1_EN_MASK);
            }
#ifdef PMU_TIMER_DEBUG
            pmu_timer_env.record[trig_type].is_enabled = false;
#endif
            CS_CRITICAL_END();
            break;
        case PMU_TIMER_CONTROL_SET_TIMER_VAL:
            CS_CRITICAL_BEGIN();
            if (trig_type == PMU_TIMER_TRIG_VAL0) {
                pmu_timer->TIMER_SET0 = pmu_timer_env.TIMER_SET0 = (uint32_t)argu;
                REGW1(&pmu_timer->TIMER_CTRL, PMU_TIMER_CTRL_PMU_TIMER_INT_VAL0_EN_MASK);
            } else if (trig_type == PMU_TIMER_TRIG_VAL1) {
                pmu_timer->TIMER_SET1 = pmu_timer_env.TIMER_SET1 = (uint32_t)argu;
                REGW1(&pmu_timer->TIMER_CTRL, PMU_TIMER_CTRL_PMU_TIMER_INT_VAL1_EN_MASK);
            }
#ifdef PMU_TIMER_DEBUG
            pmu_timer_env.record[trig_type].is_enabled = true;
            pmu_timer_env.record[trig_type].cur_cnt_when_set = drv_pmu_timer_cnt_get();
            pmu_timer_env.record[trig_type].cur_cnt_when_overflow = 0;
            pmu_timer_env.record[trig_type].value = trig_type==PMU_TIMER_TRIG_VAL0 ? pmu_timer_env.TIMER_SET0 : pmu_timer_env.TIMER_SET1;
#endif
            CS_CRITICAL_END();
            break;
        case PMU_TIMER_CONTROL_GET_TIMER_VAL:
            if (trig_type == PMU_TIMER_TRIG_VAL0) {
                return (void *)pmu_timer_env.TIMER_SET0;
            } else if (trig_type == PMU_TIMER_TRIG_VAL1) {
                return (void *)pmu_timer_env.TIMER_SET1;
            }
            break;
        case PMU_TIMER_CONTROL_GET_OVERFLOW:
            if (trig_type == PMU_TIMER_TRIG_VAL0) {
                return (void *)(pmu_timer->TIMER_CTRL & PMU_TIMER_CTRL_PMU_TIMER_INT0_MASK);
            } else if (trig_type == PMU_TIMER_TRIG_VAL1) {
                return (void *)(pmu_timer->TIMER_CTRL & PMU_TIMER_CTRL_PMU_TIMER_INT1_MASK);
            }
            break;
        case PMU_TIMER_CONTROL_SET_TIMER_INCR:
            pmu_timer->TIMER_PPM = (uint32_t)argu;
            break;
        case PMU_TIMER_CONTROL_GET_TRIG_INT_STATE:
            if (trig_type == PMU_TIMER_TRIG_VAL0) {
                return (void *)(pmu_timer->TIMER_CTRL & PMU_TIMER_CTRL_PMU_TIMER_INT_VAL0_EN_MASK);
            } else if (trig_type == PMU_TIMER_TRIG_VAL1) {
                return (void *)(pmu_timer->TIMER_CTRL & PMU_TIMER_CTRL_PMU_TIMER_INT_VAL1_EN_MASK);
            }
            break;
        default:
            break;
    }

    return (void *)((uint32_t)CS_ERROR_OK);
}


#ifdef __cplusplus
}
#endif

#endif  /* RTE_PMU_TIMER */

#endif  /* __DRV_PMU_TIMER_H */


/** @} */
