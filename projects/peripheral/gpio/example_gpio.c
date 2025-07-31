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
 * @file     example_gpio.c
 * @brief    example for using gpio
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup EXAMPLE_GPIO GPIO
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using gpio
 * @details
 * There is an example to use gpio as follows: read, write, trigger
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
/// Test pad for gpio output
#define PAD_GPIO_WRITE          23
/// Test pad for gpio input
#define PAD_GPIO_READ           24
/// Test pad for gpio input trigger
#define PAD_GPIO_TRIG           25


/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * CONST & VARIABLES
 */
/// Pinmux Configuration
static pin_config_t pin_config[] = {
    {PAD_GPIO_WRITE, {PINMUX_GPIO_MODE_CFG}, PMU_PIN_MODE_PP, PMU_PIN_DRIVER_CURRENT_NORMAL},
    {PAD_GPIO_READ,  {PINMUX_GPIO_MODE_CFG}, PMU_PIN_MODE_PU, PMU_PIN_DRIVER_CURRENT_NORMAL},
    {PAD_GPIO_TRIG,  {PINMUX_GPIO_MODE_CFG}, PMU_PIN_MODE_PU, PMU_PIN_DRIVER_CURRENT_NORMAL},
};

/// GPIO Configuration
static gpio_config_t gpio_config[] = {
    {CS_GPIO0, PAD_GPIO_WRITE,  GPIO_DIR_OUTPUT, GPIO_LEVEL_LOW, GPIO_TRIG_NONE},
    {CS_GPIO0, PAD_GPIO_READ,   GPIO_DIR_INPUT,  GPIO_LEVEL_LOW, GPIO_TRIG_NONE},
    {CS_GPIO0, PAD_GPIO_TRIG,   GPIO_DIR_INPUT,  GPIO_LEVEL_LOW, GPIO_TRIG_RISING_FAILING_EDGE},
};


/*******************************************************************************
 * LOCAL FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief gpio trigger callback
 *
 * @param[in] cs_gpio       Pointer to GPIO
 * @param[in] event         Event
 *                          - DRV_EVENT_COMMON_GENERAL
 * @param[in] int_status    Indicate which pin generated the interrupt
 * @param[in] gpio_data     gpio level data
 *
 *******************************************************************************
 */
static void test_gpio_cb(void *cs_gpio, drv_event_t event, void *int_status, void *gpio_data)
{
    cs_printf("gpio trigger happens, int_status:%x, gpio_data:%x\r\n", (uint32_t)int_status, (uint32_t)gpio_data);
}


/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of using gpio
 *        1. make the pin output high
 *        2. read the pin level
 *        3. configure pin double edge trigger, print gpio information when triggered.
 *
 *******************************************************************************
 */
void example_gpio(void)
{
    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));
    drv_gpio_init(gpio_config, sizeof(gpio_config) / sizeof(gpio_config[0]));

    // output high level
    drv_gpio_write(CS_GPIO0, GPIO_MASK(PAD_GPIO_WRITE), GPIO_LEVEL_HIGH);

    // read gpio level
    uint32_t level = drv_gpio_read(CS_GPIO0, GPIO_MASK(PAD_GPIO_READ));
    cs_printf("pin%d level is %x\r\n", PAD_GPIO_READ, level);

    // trigger: both edge
    drv_gpio_register_isr_callback(CS_GPIO0, test_gpio_cb);
}


/** @} */
