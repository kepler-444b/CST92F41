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
 * @file     drv_gpio.c
 * @brief
 * @date     3 Feb 2023
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
#include "RTE_cst92f4x.h"
#if (RTE_GPIO0)
#include <stdint.h>
#include "cs_device.h"
#include "cs_driver.h"


/*******************************************************************************
 * MACROS
 */


/*******************************************************************************
 * TYPEDEFS
 */

typedef struct {
    drv_isr_callback_t      event_cb;
    uint32_t               gpio_using;  // bit field, each bit indicates a gpio
} gpio_env_t;

/*******************************************************************************
 * CONST & VARIABLES
 */
#if (RTE_GPIO0)
static gpio_env_t gpio0_env = {
    .event_cb   = NULL,
    .gpio_using = 0U,
};

static const drv_resource_t gpio0_resource = {
    .cap      = CAP_GPIO0,
    .reg      = (void *)CS_GPIO0,
    .irq_num  = GPIO_IRQn,
    .irq_prio = RTE_GPIO0_IRQ_PRIORITY,
    .env      = (void *)(&gpio0_env),
};
#endif  /* (RTE_GPIO0) */


/*******************************************************************************
 * LOCAL FUNCTIONS
 */
static const drv_resource_t *gpio_get_resource(CS_GPIO_Type *cs_gpio)
{
    #if (RTE_GPIO0)
    if ((uint32_t)cs_gpio == (uint32_t)(gpio0_resource.reg)) {
        return (&gpio0_resource);
    }
    #endif  /* (RTE_GPIO0) */

    CS_ASSERT(0);
    return NULL;
}



/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
void drv_gpio_init(const gpio_config_t *gpio_config, uint32_t gpio_config_num)
{
    const drv_resource_t *resource;
    CS_GPIO_Type         *cs_gpio;
    gpio_env_t           *env;

    for (uint32_t i = 0; i < gpio_config_num; ++i) {
        if(gpio_config[i].cs_gpio) {
            resource = gpio_get_resource(gpio_config[i].cs_gpio);
            if (resource == NULL) {
                return;
            }
            env = (gpio_env_t *)(resource->env);
            cs_gpio = (CS_GPIO_Type *)(resource->reg);
            drv_gpio_control(cs_gpio, GPIO_CONTROL_CLK_ENABLE, NULL);

            CS_CRITICAL_BEGIN();
            if (env->gpio_using == 0) {
                // Clear and Enable GPIO IRQ
                NVIC_ClearPendingIRQ(resource->irq_num);
                NVIC_SetPriority(resource->irq_num, resource->irq_prio);
                NVIC_EnableIRQ(resource->irq_num);
            }
            env->gpio_using |= GPIO_MASK(gpio_config[i].gpio_idx);

            drv_gpio_set_dir(cs_gpio, GPIO_MASK(gpio_config[i].gpio_idx), gpio_config[i].dir);
            if(gpio_config[i].dir == GPIO_DIR_OUTPUT) {
                drv_gpio_write(cs_gpio, GPIO_MASK(gpio_config[i].gpio_idx), gpio_config[i].out_val);
            } else {
                drv_gpio_set_trig(cs_gpio, GPIO_MASK(gpio_config[i].gpio_idx), gpio_config[i].trig_type);
            }
            CS_CRITICAL_END();
        }
    }
}

#if (RTE_GPIO_REGISTER_CALLBACK)
void drv_gpio_register_isr_callback(CS_GPIO_Type *cs_gpio, drv_isr_callback_t event_cb)
{
    const drv_resource_t *resource;
    gpio_env_t           *env;

    resource = gpio_get_resource(cs_gpio);
    if (resource) {
        env = (gpio_env_t *)(resource->env);
        env->event_cb = event_cb;
    }
}
#endif

__WEAK void drv_gpio_isr_callback(CS_GPIO_Type *cs_gpio, drv_event_t event, uint32_t int_status, uint32_t gpio_data)
{
    #if (RTE_GPIO_REGISTER_CALLBACK)
    const drv_resource_t *resource;
    gpio_env_t           *env;

    resource = gpio_get_resource(cs_gpio);
    if (resource == NULL) {
        return;
    }

    env = (gpio_env_t *)(resource->env);

    if (env->event_cb != NULL) {
        env->event_cb(cs_gpio, event, (void *)int_status, (void *)gpio_data);
    }
    #endif
}

void drv_gpio_set_trig(CS_GPIO_Type *cs_gpio, gpio_mask_t gpio_mask, gpio_trig_type_t trig)
{
    cs_gpio->INTSTATUS  = gpio_mask;    /* clear irq status */

    switch(trig) {
        case GPIO_TRIG_NONE:
            cs_gpio->INTENCLR   = gpio_mask;
            break;
        case GPIO_TRIG_FALLING_EDGE:
            cs_gpio->INTTYPESET = gpio_mask;
            cs_gpio->INTPOLCLR  = gpio_mask;
            cs_gpio->INTBOTHCLR = gpio_mask;
            cs_gpio->INTENSET   = gpio_mask;
            break;
        case GPIO_TRIG_RISING_EDGE:
            cs_gpio->INTTYPESET = gpio_mask;
            cs_gpio->INTPOLSET  = gpio_mask;
            cs_gpio->INTBOTHCLR = gpio_mask;
            cs_gpio->INTENSET   = gpio_mask;
            break;
        case GPIO_TRIG_RISING_FAILING_EDGE:
            cs_gpio->INTTYPESET = gpio_mask;
            cs_gpio->INTPOLCLR  = gpio_mask;
            cs_gpio->INTBOTHSET = gpio_mask;
            cs_gpio->INTENSET   = gpio_mask;
            break;
        case GPIO_TRIG_LOW_LEVEL:
            cs_gpio->INTTYPECLR = gpio_mask;
            cs_gpio->INTPOLCLR  = gpio_mask;
            cs_gpio->INTBOTHCLR = gpio_mask;
            cs_gpio->INTENSET   = gpio_mask;
            break;
        case GPIO_TRIG_HIGH_LEVEL:
            cs_gpio->INTTYPECLR = gpio_mask;
            cs_gpio->INTPOLSET  = gpio_mask;
            cs_gpio->INTBOTHCLR = gpio_mask;
            cs_gpio->INTENSET   = gpio_mask;
            break;
        default:
            CS_ASSERT(0U);
            break;
    }
}

gpio_trig_type_t drv_gpio_get_trig(CS_GPIO_Type *cs_gpio, gpio_mask_t gpio_mask)
{
    if (cs_gpio->INTENSET & gpio_mask) {
        if ((cs_gpio->INTTYPESET & gpio_mask) && (cs_gpio->INTPOLSET & gpio_mask)) {
            if (cs_gpio->INTBOTHSET & gpio_mask) {
                return GPIO_TRIG_RISING_FAILING_EDGE;
            } else {
                return GPIO_TRIG_RISING_EDGE;
            }
        } else if (((cs_gpio->INTTYPESET & gpio_mask) == 0) && (cs_gpio->INTPOLSET & gpio_mask)) {
            return GPIO_TRIG_HIGH_LEVEL;
        } else if (((cs_gpio->INTPOLSET & gpio_mask) == 0) && (cs_gpio->INTTYPESET & gpio_mask)) {
            if (cs_gpio->INTBOTHSET & gpio_mask) {
                return GPIO_TRIG_RISING_FAILING_EDGE;
            } else {
                return GPIO_TRIG_FALLING_EDGE;
            }
        } else if (((cs_gpio->INTPOLSET & gpio_mask) == 0) && ((cs_gpio->INTTYPESET & gpio_mask) == 0)) {
            return GPIO_TRIG_LOW_LEVEL;
        }
    }
    return GPIO_TRIG_NONE;
}

void *drv_gpio_control(CS_GPIO_Type *cs_gpio, gpio_control_t control, void *argu)
{
    switch (control) {
        case GPIO_CONTROL_CLK_DISABLE:
            do {
                #if (RTE_GPIO0)
                if ((uint32_t)cs_gpio == (uint32_t)CS_GPIO0) {
                    DRV_RCC_CLOCK_ENABLE(RCC_CLK_GPIO0, 0U);
                    break;
                }
                #endif  /* (RTE_GPIO0) */
            } while(0);
            break;
        case GPIO_CONTROL_CLK_ENABLE:
            do {
                #if (RTE_GPIO0)
                if ((uint32_t)cs_gpio == (uint32_t)CS_GPIO0) {
                    DRV_RCC_CLOCK_ENABLE(RCC_CLK_GPIO0, 1U);
                    break;
                }
                #endif  /* (RTE_GPIO0) */
            } while(0);
            break;
        case GPIO_CONTROL_RESET:
            do {
                #if (RTE_GPIO0)
                if ((uint32_t)cs_gpio == (uint32_t)CS_GPIO0) {
                    DRV_RCC_RESET(RCC_CLK_GPIO0);
                    gpio0_env.gpio_using = 0U;
                    break;
                }
                #endif  /* (RTE_GPIO0) */
            } while(0);
            break;
        case GPIO_CONTROL_CLEAR_INT:
            cs_gpio->INTSTATUS = (uint32_t)argu;
            break;
        default:
            break;
    }

    return (void *)CS_ERROR_OK;
}

void drv_gpio_isr(CS_GPIO_Type *cs_gpio)
{
    drv_event_t            event;
    const drv_resource_t   *resource;
    uint32_t int_status;

    DRV_IRQ_BEGIN();

    resource = gpio_get_resource(cs_gpio);
    if (resource == NULL) {
        return;
    }

    // clear int status
    int_status = cs_gpio->INTSTATUS;
    cs_gpio->INTSTATUS = int_status;

    event = DRV_EVENT_COMMON_GENERAL;
    drv_gpio_isr_callback(cs_gpio, event, int_status, cs_gpio->DATA);

    // disable wakeup irq event
    drv_pmu_pin_wakeup_out_of_date();

    DRV_IRQ_END();
}

#endif

/** @} */

