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
 * @file     example_adc.c
 * @brief    example for adc
 * @date     3 Feb 2023
 * @author   chipsea
 *
 * @defgroup EXAMPLE_ADC ADC
 * @ingroup  DRIVER_EXAMPLES
 * @brief    Example for using adc
 * @details
 * There is an example to use adc as follows: read blocking, interrupt, and DMA
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
#include "cs_common_utils.h"


/*******************************************************************************
 * MACROS
 */
#define TEST_SIZE               20
#define PAD_ADC_CH_GPIO2        2
#define MUX_ADC_CH_GPIO2        PINMUX_PAD2_INPUT_MODE_CFG
#define PAD_ADC_CH_GPIO3        3
#define MUX_ADC_CH_GPIO3        PINMUX_PAD3_INPUT_MODE_CFG
#define PAD_ADC_CH_GPIO8        8
#define MUX_ADC_CH_GPIO8        PINMUX_PAD8_INPUT_MODE_CFG

#define TEMPERATURRE_COMPEN_EN
/*******************************************************************************
 * TYPEDEFS
 */


/*******************************************************************************
 * CONST & VARIABLES
 */
/// Pin configuration
static const pin_config_t pin_config[] = {
    {PAD_ADC_CH_GPIO2, {MUX_ADC_CH_GPIO2}, PMU_PIN_MODE_FLOAT, PMU_PIN_DRIVER_CURRENT_NORMAL},
};

static const pin_config_t multi_pin_config[] = {
    {PAD_ADC_CH_GPIO2, {MUX_ADC_CH_GPIO2}, PMU_PIN_MODE_FLOAT, PMU_PIN_DRIVER_CURRENT_NORMAL},
    {PAD_ADC_CH_GPIO3, {MUX_ADC_CH_GPIO3}, PMU_PIN_MODE_FLOAT, PMU_PIN_DRIVER_CURRENT_NORMAL},
};

static const pin_config_t diff_pin_config[] = {
    {PAD_ADC_CH_GPIO2, {MUX_ADC_CH_GPIO2}, PMU_PIN_MODE_FLOAT, PMU_PIN_DRIVER_CURRENT_NORMAL},
    {PAD_ADC_CH_GPIO8, {MUX_ADC_CH_GPIO8}, PMU_PIN_MODE_FLOAT, PMU_PIN_DRIVER_CURRENT_NORMAL},
};

/// Buffer that stores the data to be received
static int16_t out[TEST_SIZE];
static int16_t out_ex[TEST_SIZE];
static int16_t out_multi[3*TEST_SIZE];
/// Read finish flag
static volatile uint8_t adc_done;

/*******************************************************************************
 * LOCAL FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief callback for ADC interrupt
 *
 * @param[in] cs_gpadc    Pointer to ADC
 * @param[in] event       ADC event
 *                        - DRV_EVENT_COMMON_READ_COMPLETED
 *                        - DRV_EVENT_COMMON_ABORT
 *                        - DRV_EVENT_COMMON_ERROR
 * @param[in] read_buf    Pointer to receive buffer
 * @param[in] read_cnt    The number of received data
 *
 *******************************************************************************
 */
static void adc_read_cb(void *cs_gpadc, drv_event_t event, void *read_buf, void *read_cnt)
{
    if (event == DRV_EVENT_COMMON_READ_COMPLETED) {
        adc_done = 1;
    }
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief example of read in blocking mode
 *
 *******************************************************************************
 */
void example_adc_read(void)
{
    drv_adc_config_t config;
    config.channel = ADC_CH_GPIO2;
    config.mode = ADC_MODE_SINGLE;
    config.gain = ADC_GAIN_1_3;
    config.sum_num = ADC_SUM_NUM_256;
    config.sampling_cycles = ADC_SAMPLING_CYCLES_32;

    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));
    drv_adc_init(&config);

    drv_adc_read(config.channel, &out[0], TEST_SIZE);
    CS_LOG_DEBUG("%s: %s\r\n", __func__, bt_hex(out, TEST_SIZE));

    drv_adc_control(ADC_CONTROL_DISABLE_CLOCK, NULL);
}

/**
 *******************************************************************************
 * @brief example of read in interrupt mode
 *
 *******************************************************************************
 */
void example_adc_read_int(void)
{
    drv_adc_config_t config;
    config.channel = ADC_CH_GPIO2;
    config.mode = ADC_MODE_SINGLE;
    config.gain = ADC_GAIN_1_3;
    config.sum_num = ADC_SUM_NUM_256;
    config.sampling_cycles = ADC_SAMPLING_CYCLES_32;
    
    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));
    drv_adc_init(&config);
    drv_adc_register_isr_callback(adc_read_cb);

    drv_adc_read_int(config.channel, &out[0], TEST_SIZE);

    while(!adc_done);
    adc_done = 0;
    CS_LOG_DEBUG("%s: %s\r\n", __func__, bt_hex(out, TEST_SIZE));

    drv_adc_control(ADC_CONTROL_DISABLE_CLOCK, NULL);
}

/**
 *******************************************************************************
 * @brief example of read in dma mode
 *
 *******************************************************************************
 */
void example_adc_read_dma(void)
{
#if (RTE_DMA)
    drv_adc_config_t config;
    config.channel = ADC_CH_GPIO2;
    config.mode = ADC_MODE_SINGLE;
    config.gain = ADC_GAIN_1_3;
    config.sum_num = ADC_SUM_NUM_256;
    config.sampling_cycles = ADC_SAMPLING_CYCLES_32;

    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));
    drv_adc_init(&config);
    drv_adc_register_isr_callback(adc_read_cb);

    drv_adc_dma_channel_allocate();
    drv_adc_read_dma(config.channel, &out[0], TEST_SIZE);

    while(!adc_done);
    adc_done = 0;
    CS_LOG_DEBUG("%s: %s\r\n", __func__, bt_hex(out, TEST_SIZE));

    drv_adc_dma_channel_release();
    drv_adc_control(ADC_CONTROL_DISABLE_CLOCK, NULL);
#endif
}

/**
 *******************************************************************************
 * @brief example of read temperature in blocking mode
 *
 *******************************************************************************
 */
void example_adc_read_temperature(void)
{
    drv_adc_config_t config;
    config.channel = ADC_CH_TEMPERATURE;
    config.mode = ADC_MODE_SINGLE;
    config.gain = ADC_GAIN_1_3;
    config.sum_num = ADC_SUM_NUM_256;
    config.sampling_cycles = ADC_SAMPLING_CYCLES_32;

    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));
    drv_adc_init(&config);

    drv_adc_read(config.channel, &out[0], TEST_SIZE);
    CS_LOG_DEBUG("%s: %s\r\n", __func__, bt_hex(out, TEST_SIZE));

#ifdef TEMPERATURRE_COMPEN_EN
    config.channel = ADC_CH_VBAT;
    drv_adc_init(&config);

    drv_adc_read(config.channel, &out_ex[0], TEST_SIZE);
    
    drv_adc_temperature_compen_t compen;
    for (int i = 0; i < TEST_SIZE; i++) {
        compen.temp_data = out[i];
        compen.vbat_data = out_ex[i];
        out[i] = (int)drv_adc_control(ADC_CONTROL_TEMPERATURE_COMPEN, &compen);
    }
#endif

    drv_adc_control(ADC_CONTROL_DISABLE_CLOCK, NULL);
}

/**
 *******************************************************************************
 * @brief example of read vbat in blocking mode
 *
 *******************************************************************************
 */
void example_adc_read_battery(void)
{
    drv_adc_config_t config;
    config.channel = ADC_CH_VBAT;
    config.mode = ADC_MODE_SINGLE;
    config.gain = ADC_GAIN_1_3;
    config.sum_num = ADC_SUM_NUM_256;
    config.sampling_cycles = ADC_SAMPLING_CYCLES_32;

    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));
    drv_adc_init(&config);

    drv_adc_read(config.channel, &out[0], TEST_SIZE);
    CS_LOG_DEBUG("%s: %s\r\n", __func__, bt_hex(out, TEST_SIZE));

    drv_adc_control(ADC_CONTROL_DISABLE_CLOCK, NULL);
}

/**
 *******************************************************************************
* @brief example of read multi channels in blocking mode
         NOTE: do not support ADC_CH_TEMPERATURE | ADC_CH_VBAT or ADC_CH_TEMPERATURE | ADC_CH_GPIOx
 *
 *******************************************************************************
 */
void example_adc_read_multi(void)
{
    drv_adc_config_t config;
    config.channel = ADC_CH_VBAT | ADC_CH_GPIO2 | ADC_CH_GPIO3;
    config.mode = ADC_MODE_SINGLE;
    config.gain = ADC_GAIN_1_3;
    config.sum_num = ADC_SUM_NUM_256;
    config.sampling_cycles = ADC_SAMPLING_CYCLES_32;

    drv_pin_init(multi_pin_config, sizeof(multi_pin_config) / sizeof(multi_pin_config[0]));
    drv_adc_init(&config);

    drv_adc_read(config.channel, &out_multi[0], 3*TEST_SIZE);
    CS_LOG_DEBUG("%s: %s\r\n", __func__, bt_hex(out_multi, 3*TEST_SIZE));

    drv_adc_control(ADC_CONTROL_DISABLE_CLOCK, NULL);
}

/**
 *******************************************************************************
* @brief example of read multi channels in interrupt mode
         NOTE: do not support ADC_CH_TEMPERATURE | ADC_CH_VBAT or ADC_CH_TEMPERATURE | ADC_CH_GPIOx
 *
 *******************************************************************************
 */
void example_adc_read_int_multi(void)
{
    drv_adc_config_t config;
    config.channel = ADC_CH_VBAT | ADC_CH_GPIO2 | ADC_CH_GPIO3;
    config.mode = ADC_MODE_SINGLE;
    config.gain = ADC_GAIN_1_3;
    config.sum_num = ADC_SUM_NUM_256;
    config.sampling_cycles = ADC_SAMPLING_CYCLES_32;
    
    drv_pin_init(multi_pin_config, sizeof(multi_pin_config) / sizeof(multi_pin_config[0]));
    drv_adc_init(&config);
    drv_adc_register_isr_callback(adc_read_cb);

    drv_adc_read_int(config.channel, &out_multi[0], 3*TEST_SIZE);

    while(!adc_done);
    adc_done = 0;
    CS_LOG_DEBUG("%s: %s\r\n", __func__, bt_hex(out_multi, 3*TEST_SIZE));

    drv_adc_control(ADC_CONTROL_DISABLE_CLOCK, NULL);
}

 /**
 *******************************************************************************
* @brief example of read multi channels in dma mode
         NOTE: do not support ADC_CH_TEMPERATURE | ADC_CH_VBAT or ADC_CH_TEMPERATURE | ADC_CH_GPIOx
 *
 *******************************************************************************
 */
void example_adc_read_dma_multi(void)
{
#if (RTE_DMA)
    drv_adc_config_t config;
    config.channel = ADC_CH_VBAT | ADC_CH_GPIO2 | ADC_CH_GPIO3;
    config.mode = ADC_MODE_SINGLE;
    config.gain = ADC_GAIN_1_3;
    config.sum_num = ADC_SUM_NUM_256;
    config.sampling_cycles = ADC_SAMPLING_CYCLES_32;

    drv_pin_init(multi_pin_config, sizeof(multi_pin_config) / sizeof(multi_pin_config[0]));
    drv_adc_init(&config);
    drv_adc_register_isr_callback(adc_read_cb);

    drv_adc_dma_channel_allocate();
    drv_adc_read_dma(config.channel, &out_multi[0], 3*TEST_SIZE);

    while(!adc_done);
    adc_done = 0;
    CS_LOG_DEBUG("%s: %s\r\n", __func__, bt_hex(out_multi, 3*TEST_SIZE));

    drv_adc_dma_channel_release();
    drv_adc_control(ADC_CONTROL_DISABLE_CLOCK, NULL);
#endif
}

void example_adc(void)
{
    example_adc_read();
    example_adc_read_int();
    example_adc_read_dma();
    example_adc_read_temperature();
    example_adc_read_battery();
    example_adc_read_multi();
    example_adc_read_int_multi();
    example_adc_read_dma_multi();
}
/** @} */
