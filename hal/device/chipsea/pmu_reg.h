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
 * @file     pmu_reg.h
 * @brief    PMU register file
 * @date     21. July 2020
 * @author   chipsea
 *
 * @defgroup DOC DOC
 * @ingroup  DOCUMENT
 * @brief    templete
 * @details  Power Manager Unit register file define
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __PMU_REG_H
#define __PMU_REG_H


/*******************************************************************************
 * INCLUDES
 */
#include <stdint.h>
#include "../common/common_reg.h"


/*******************************************************************************
 * MACROS
 */
// PMU_BASIC
#define PMU_BASIC_PMU_STATE_POS                                        27
#define PMU_BASIC_PMU_STATE_MASK                                       (0x1fU << 27)
#define PMU_BASIC_CRY32K_READY_POS                                     26
#define PMU_BASIC_CRY32K_READY_MASK                                    (0x1U << 26)
#define PMU_BASIC_WAKEUPB_DIS_POS                                      16
#define PMU_BASIC_WAKEUPB_DIS_MASK                                     (0x1U << 16)
#define PMU_BASIC_CLK_OFF_ONLY_POS                                     15
#define PMU_BASIC_CLK_OFF_ONLY_MASK                                    (0x1U << 15)
#define PMU_BASIC_SLEEP_WO_32K_POS                                     14
#define PMU_BASIC_SLEEP_WO_32K_MASK                                    (0x1U << 14)
#define PMU_BASIC_LPTIM_32K_CLK_GATE_POS                               11
#define PMU_BASIC_LPTIM_32K_CLK_GATE_MASK                              (0x1U << 11)
#define PMU_BASIC_BB_32K_CLK_GATE_POS                                  10
#define PMU_BASIC_BB_32K_CLK_GATE_MASK                                 (0x1U << 10)
#define PMU_BASIC_CPU_32K_CLK_GATE_POS                                 9
#define PMU_BASIC_CPU_32K_CLK_GATE_MASK                                (0x1U << 9)
#define PMU_BASIC_RTC_CLK_GATE_POS                                     8
#define PMU_BASIC_RTC_CLK_GATE_MASK                                    (0x1U << 8)
#define PMU_BASIC_PSO_ON_EN_POS                                        6
#define PMU_BASIC_PSO_ON_EN_MASK                                       (0x1U << 6)
#define PMU_BASIC_PIN_WAKEUP_MODE_1_POS                                5
#define PMU_BASIC_PIN_WAKEUP_MODE_1_MASK                               (0x1U << 5)
#define PMU_BASIC_PIN_WAKEUP_MODE_POS                                  4
#define PMU_BASIC_PIN_WAKEUP_MODE_MASK                                 (0x1U << 4)
#define PMU_BASIC_DCDC_ON_DELAY_POS                                    0
#define PMU_BASIC_DCDC_ON_DELAY_MASK                                   (0xfU << 0)

// PMU_PSO_PM
#define PMU_PSO_PM_LPTIM_POWER_ON_POS                                  16
#define PMU_PSO_PM_LPTIM_POWER_ON_MASK                                 (0x1U << 16)
#define PMU_PSO_PM_RTC_POWER_ON_POS                                    15
#define PMU_PSO_PM_RTC_POWER_ON_MASK                                   (0x1U << 15)
#define PMU_PSO_PM_RAM_POWER_ON_POS                                    14
#define PMU_PSO_PM_RAM_POWER_ON_MASK                                   (0x1U << 14)
#define PMU_PSO_PM_ICACHE_POWER_ON_POS                                 13
#define PMU_PSO_PM_ICACHE_POWER_ON_MASK                                (0x1U << 13)
#define PMU_PSO_PM_ICACHE_RST_N_POS                                    12
#define PMU_PSO_PM_ICACHE_RST_N_MASK                                   (0x1U << 12)
#define PMU_PSO_PM_PSO_CLAMP_DELAY_POS                                 8
#define PMU_PSO_PM_PSO_CLAMP_DELAY_MASK                                (0xfU << 8)
#define PMU_PSO_PM_LPTIM_POWER_STATUS_POS                              4
#define PMU_PSO_PM_LPTIM_POWER_STATUS_MASK                             (0x1U << 4)
#define PMU_PSO_PM_RTC_POWER_STATUS_POS                                3
#define PMU_PSO_PM_RTC_POWER_STATUS_MASK                               (0x1U << 3)
#define PMU_PSO_PM_RAM_POWER_STATUS_POS                                2
#define PMU_PSO_PM_RAM_POWER_STATUS_MASK                               (0x1U << 2)
#define PMU_PSO_PM_ICACHE_POWER_STATUS_POS                             1
#define PMU_PSO_PM_ICACHE_POWER_STATUS_MASK                            (0x1U << 1)
#define PMU_PSO_PM_PSO_POWER_STATUS_POS                                0
#define PMU_PSO_PM_PSO_POWER_STATUS_MASK                               (0x1U << 0)

// XTAL32M_CNS0
#define PMU_XTAL32M_CNS0_DISTURB_M123_NUM_CFG_POS                      24
#define PMU_XTAL32M_CNS0_DISTURB_M123_NUM_CFG_MASK                     (0xffU << 24)
#define PMU_XTAL32M_CNS0_SEL_COMP_IBIAS_POS                            22
#define PMU_XTAL32M_CNS0_SEL_COMP_IBIAS_MASK                           (0x1U << 22)
#define PMU_XTAL32M_CNS0_EN_OSC32M_ME_POS                              19
#define PMU_XTAL32M_CNS0_EN_OSC32M_ME_MASK                             (0x1U << 19)
#define PMU_XTAL32M_CNS0_EN_OSC32M_MO_POS                              18
#define PMU_XTAL32M_CNS0_EN_OSC32M_MO_MASK                             (0x1U << 18)
#define PMU_XTAL32M_CNS0_SEL_CPUCLK_POS                                16
#define PMU_XTAL32M_CNS0_SEL_CPUCLK_MASK                               (0x1U << 16)
#define PMU_XTAL32M_CNS0_EN_OSC32M_CHIRPRAMP_ME_POS                    15
#define PMU_XTAL32M_CNS0_EN_OSC32M_CHIRPRAMP_ME_MASK                   (0x1U << 15)
#define PMU_XTAL32M_CNS0_EN_OSC32M_CHIRPRAMP_MO_POS                    14
#define PMU_XTAL32M_CNS0_EN_OSC32M_CHIRPRAMP_MO_MASK                   (0x1U << 14)
#define PMU_XTAL32M_CNS0_EN_XTAL32M_NRB_ME_POS                         11
#define PMU_XTAL32M_CNS0_EN_XTAL32M_NRB_ME_MASK                        (0x1U << 11)
#define PMU_XTAL32M_CNS0_EN_XTAL32M_NRB_MO_POS                         8
#define PMU_XTAL32M_CNS0_EN_XTAL32M_NRB_MO_MASK                        (0x3U << 8)
#define PMU_XTAL32M_CNS0_EN_OSC32M_BIAS_ME_POS                         7
#define PMU_XTAL32M_CNS0_EN_OSC32M_BIAS_ME_MASK                        (0x1U << 7)
#define PMU_XTAL32M_CNS0_EN_OSC32M_BIAS_MO_POS                         6
#define PMU_XTAL32M_CNS0_EN_OSC32M_BIAS_MO_MASK                        (0x1U << 6)
#define PMU_XTAL32M_CNS0_EN_32M_DGLITCH_POS                            4
#define PMU_XTAL32M_CNS0_EN_32M_DGLITCH_MASK                           (0x1U << 4)
#define PMU_XTAL32M_CNS0_PD_XTAL32M_DIGBUF_MO_POS                      3
#define PMU_XTAL32M_CNS0_PD_XTAL32M_DIGBUF_MO_MASK                     (0x1U << 3)
#define PMU_XTAL32M_CNS0_PD_XTAL32M_DIGBUF_ME_POS                      2
#define PMU_XTAL32M_CNS0_PD_XTAL32M_DIGBUF_ME_MASK                     (0x1U << 2)
#define PMU_XTAL32M_CNS0_PD_XTAL32M_PLLBUF_MO_POS                      1
#define PMU_XTAL32M_CNS0_PD_XTAL32M_PLLBUF_MO_MASK                     (0x1U << 1)
#define PMU_XTAL32M_CNS0_PD_XTAL32M_PLLBUF_ME_POS                      0
#define PMU_XTAL32M_CNS0_PD_XTAL32M_PLLBUF_ME_MASK                     (0x1U << 0)

// PMU_ANA_PD
#define PMU_ANA_PD_PD_FLASH_ME_POS                                     31
#define PMU_ANA_PD_PD_FLASH_ME_MASK                                    (0x1U << 31)
#define PMU_ANA_PD_PMU_LDO_ANA1P2_TRIM_POS                             28
#define PMU_ANA_PD_PMU_LDO_ANA1P2_TRIM_MASK                            (0x7U << 28)
#define PMU_ANA_PD_DCDC_CTUNE_POS                                      25
#define PMU_ANA_PD_DCDC_CTUNE_MASK                                     (0x3U << 25)
#define PMU_ANA_PD_EN_64M_POS                                          23
#define PMU_ANA_PD_EN_64M_MASK                                         (0x1U << 23)
#define PMU_ANA_PD_BUCK_OSC_PD_POS                                     18
#define PMU_ANA_PD_BUCK_OSC_PD_MASK                                    (0x1U << 18)
#define PMU_ANA_PD_ANA_1P2LDO_DIS_POS                                  17
#define PMU_ANA_PD_ANA_1P2LDO_DIS_MASK                                 (0x1U << 17)
#define PMU_ANA_PD_DCDC_DIS_POS                                        16
#define PMU_ANA_PD_DCDC_DIS_MASK                                       (0x1U << 16)
#define PMU_ANA_PD_DCDC_SLOPE_POS                                      12
#define PMU_ANA_PD_DCDC_SLOPE_MASK                                     (0xfU << 12)
#define PMU_ANA_PD_DCDC_ILIMIT_POS                                     11
#define PMU_ANA_PD_DCDC_ILIMIT_MASK                                    (0x1U << 11)
#define PMU_ANA_PD_DCDC_LIMIT_POS                                      8
#define PMU_ANA_PD_DCDC_LIMIT_MASK                                     (0x7U << 8)
#define PMU_ANA_PD_REG_PD_DCDC_OSC_POS                                 7
#define PMU_ANA_PD_REG_PD_DCDC_OSC_MASK                                (0x1U << 7)
#define PMU_ANA_PD_DCDC_SHORT_POS                                      6
#define PMU_ANA_PD_DCDC_SHORT_MASK                                     (0x1U << 6)
#define PMU_ANA_PD_CON_SWITCH_FLASH_SEL_POS                            5
#define PMU_ANA_PD_CON_SWITCH_FLASH_SEL_MASK                           (0x1U << 5)
#define PMU_ANA_PD_PD_FLASH_REG_POS                                    4
#define PMU_ANA_PD_PD_FLASH_REG_MASK                                   (0x1U << 4)
#define PMU_ANA_PD_REG_PD_BIAS_POS                                     3
#define PMU_ANA_PD_REG_PD_BIAS_MASK                                    (0x1U << 3)
#define PMU_ANA_PD_REG_PD_DCDC_POS                                     2
#define PMU_ANA_PD_REG_PD_DCDC_MASK                                    (0x1U << 2)
#define PMU_ANA_PD_REG_PD_DIGITAL_LDO_POS                              1
#define PMU_ANA_PD_REG_PD_DIGITAL_LDO_MASK                             (0x1U << 1)
#define PMU_ANA_PD_OSC_CLK1M_HW_DIS_POS                                0
#define PMU_ANA_PD_OSC_CLK1M_HW_DIS_MASK                               (0x1U << 0)

// PMU_GPIO_POL
#define PMU_GPIO_POL_REG_GPIO_POL_25_0_POS                             0
#define PMU_GPIO_POL_REG_GPIO_POL_25_0_MASK                            (0x3ffffffU << 0)

// GPIO_PU_CTRL_1
#define PMU_GPIO_PU_CTRL_1_GPIO_PL_CTRL_1_25_0_POS                     0
#define PMU_GPIO_PU_CTRL_1_GPIO_PL_CTRL_1_25_0_MASK                    (0x3ffffffU << 0)

// PMU_MISC_CTRL
#define PMU_MISC_CTRL_RTC_APB_SOFT_RESET_POS                           30
#define PMU_MISC_CTRL_RTC_APB_SOFT_RESET_MASK                          (0x1U << 30)
#define PMU_MISC_CTRL_O_JTAG_ENABLE_POS                                29
#define PMU_MISC_CTRL_O_JTAG_ENABLE_MASK                               (0x1U << 29)
#define PMU_MISC_CTRL_CLR_PIN_WAKEUP_NOCLK_POS                         26
#define PMU_MISC_CTRL_CLR_PIN_WAKEUP_NOCLK_MASK                        (0x1U << 26)
#define PMU_MISC_CTRL_CLR_PMU_INT_POS                                  25
#define PMU_MISC_CTRL_CLR_PMU_INT_MASK                                 (0x1U << 25)
#define PMU_MISC_CTRL_MAIN_CLK_SEL_POS                                 24
#define PMU_MISC_CTRL_MAIN_CLK_SEL_MASK                                (0x1U << 24)
#define PMU_MISC_CTRL_LPTIM_APB_SOFT_RESET_POS                         22
#define PMU_MISC_CTRL_LPTIM_APB_SOFT_RESET_MASK                        (0x1U << 22)
#define PMU_MISC_CTRL_BB_2_APB_RESET_POS                               21
#define PMU_MISC_CTRL_BB_2_APB_RESET_MASK                              (0x1U << 21)
#define PMU_MISC_CTRL_CON_SWITCH_FLASH_STEP_POS                        19
#define PMU_MISC_CTRL_CON_SWITCH_FLASH_STEP_MASK                       (0x3U << 19)
#define PMU_MISC_CTRL_CON_SWITCH_FLASH_REG_POS                         16
#define PMU_MISC_CTRL_CON_SWITCH_FLASH_REG_MASK                        (0x7U << 16)
#define PMU_MISC_CTRL_GPIO_AUTO_LATCH_FSM_DIS_POS                      15
#define PMU_MISC_CTRL_GPIO_AUTO_LATCH_FSM_DIS_MASK                     (0x1U << 15)
#define PMU_MISC_CTRL_PD_CRY32K_POS                                    14
#define PMU_MISC_CTRL_PD_CRY32K_MASK                                   (0x1U << 14)
#define PMU_MISC_CTRL_FIRST_RUN_REG_POS                                13
#define PMU_MISC_CTRL_FIRST_RUN_REG_MASK                               (0x1U << 13)
#define PMU_MISC_CTRL_GPIO_AUTO_LATCH_CTRL_POS                         12
#define PMU_MISC_CTRL_GPIO_AUTO_LATCH_CTRL_MASK                        (0x1U << 12)
#define PMU_MISC_CTRL_BB_32K_SOFT_RSTN_POS                             11
#define PMU_MISC_CTRL_BB_32K_SOFT_RSTN_MASK                            (0x1U << 11)
#define PMU_MISC_CTRL_BB_2_RESET_POS                                   10
#define PMU_MISC_CTRL_BB_2_RESET_MASK                                  (0x1U << 10)
#define PMU_MISC_CTRL_LPTIM_CLK_SFOT_RESET_POS                         9
#define PMU_MISC_CTRL_LPTIM_CLK_SFOT_RESET_MASK                        (0x1U << 9)
#define PMU_MISC_CTRL_BB_1_RESET_POS                                   8
#define PMU_MISC_CTRL_BB_1_RESET_MASK                                  (0x1U << 8)
#define PMU_MISC_CTRL_RTC_SOFT_RESET_POS                               7
#define PMU_MISC_CTRL_RTC_SOFT_RESET_MASK                              (0x1U << 7)
#define PMU_MISC_CTRL_RC_32K_GATE_POS                                  5
#define PMU_MISC_CTRL_RC_32K_GATE_MASK                                 (0x1U << 5)
#define PMU_MISC_CTRL_CRY_32K_GATE_POS                                 4
#define PMU_MISC_CTRL_CRY_32K_GATE_MASK                                (0x1U << 4)
#define PMU_MISC_CTRL_PMU_CLK_GATE_POS                                 3
#define PMU_MISC_CTRL_PMU_CLK_GATE_MASK                                (0x1U << 3)
#define PMU_MISC_CTRL_CLK_32K_SEL_POS                                  0
#define PMU_MISC_CTRL_CLK_32K_SEL_MASK                                 (0x3U << 0)

// PMU_WAKE_DEB
#define PMU_WAKE_DEB_PIN_DEB_RST_POS                                   16
#define PMU_WAKE_DEB_PIN_DEB_RST_MASK                                  (0x1U << 16)
#define PMU_WAKE_DEB_PMU_GPIO_INT_DISABLE_POS                          15
#define PMU_WAKE_DEB_PMU_GPIO_INT_DISABLE_MASK                         (0x1U << 15)
#define PMU_WAKE_DEB_PIN_WAKE_LEVEL_EDGE_SEL_POS                       13
#define PMU_WAKE_DEB_PIN_WAKE_LEVEL_EDGE_SEL_MASK                      (0x3U << 13)
#define PMU_WAKE_DEB_PIN_DEBOUNCE_COEFF_WAKE_POS                       8
#define PMU_WAKE_DEB_PIN_DEBOUNCE_COEFF_WAKE_MASK                      (0x3U << 8)
#define PMU_WAKE_DEB_PIN_DEBOUNCE_CYCLE_WAKE_POS                       0
#define PMU_WAKE_DEB_PIN_DEBOUNCE_CYCLE_WAKE_MASK                      (0xfU << 0)

// GPIO_OE_CTRL
#define PMU_GPIO_OE_CTRL_GPIO_OEB_SEL_POS                              26
#define PMU_GPIO_OE_CTRL_GPIO_OEB_SEL_MASK                             (0x1U << 26)
#define PMU_GPIO_OE_CTRL_GPIO_OEB_REG_25_0_POS                         0
#define PMU_GPIO_OE_CTRL_GPIO_OEB_REG_25_0_MASK                        (0x3ffffffU << 0)

// GPIO_PU_CTRL
#define PMU_GPIO_PU_CTRL_GPIO_PL_UP_31_0_POS                           0
#define PMU_GPIO_PU_CTRL_GPIO_PL_UP_31_0_MASK                          (0xffffffffU << 0)

// XTAL32M_CNS1
#define PMU_XTAL32M_CNS1_T5_CFG_POS                                    25
#define PMU_XTAL32M_CNS1_T5_CFG_MASK                                   (0x7U << 25)
#define PMU_XTAL32M_CNS1_DISTURB_M23_T1_CFG_POS                        22
#define PMU_XTAL32M_CNS1_DISTURB_M23_T1_CFG_MASK                       (0x7U << 22)
#define PMU_XTAL32M_CNS1_XTAL32M_RESTART_POS                           12
#define PMU_XTAL32M_CNS1_XTAL32M_RESTART_MASK                          (0x1U << 12)
#define PMU_XTAL32M_CNS1_XTAL32M_NRB_POR_POS                           0
#define PMU_XTAL32M_CNS1_XTAL32M_NRB_POR_MASK                          (0x3U << 0)

// AHB_REMAP
#define PMU_AHB_REMAP_LPTIM_INT_POS                                    16
#define PMU_AHB_REMAP_LPTIM_INT_MASK                                   (0x1U << 16)
#define PMU_AHB_REMAP_NATIVE_INT_POS                                   15
#define PMU_AHB_REMAP_NATIVE_INT_MASK                                  (0x1U << 15)
#define PMU_AHB_REMAP_BB_INT_POS                                       14
#define PMU_AHB_REMAP_BB_INT_MASK                                      (0x1U << 14)
#define PMU_AHB_REMAP_RTC_INT_POS                                      13
#define PMU_AHB_REMAP_RTC_INT_MASK                                     (0x1U << 13)
#define PMU_AHB_REMAP_PIN_WAKEUP_LATCH_POS                             12
#define PMU_AHB_REMAP_PIN_WAKEUP_LATCH_MASK                            (0x1U << 12)
#define PMU_AHB_REMAP_PIN_WAKEUP_POS                                   11
#define PMU_AHB_REMAP_PIN_WAKEUP_MASK                                  (0x1U << 11)
#define PMU_AHB_REMAP_CPU_RESET_SOURCE_POS                             8
#define PMU_AHB_REMAP_CPU_RESET_SOURCE_MASK                            (0x7U << 8)
#define PMU_AHB_REMAP_WDT_RESET_FLAG_POS                               7
#define PMU_AHB_REMAP_WDT_RESET_FLAG_MASK                              (0x1U << 7)
#define PMU_AHB_REMAP_WAIT_DEEP_SLEEP_EN_POS                           6
#define PMU_AHB_REMAP_WAIT_DEEP_SLEEP_EN_MASK                          (0x1U << 6)
#define PMU_AHB_REMAP_CHIP_AHB_BUS_REMAP_EN_POS                        5
#define PMU_AHB_REMAP_CHIP_AHB_BUS_REMAP_EN_MASK                       (0x1U << 5)
#define PMU_AHB_REMAP_CPU_SOFT_RESET_POS                               4
#define PMU_AHB_REMAP_CPU_SOFT_RESET_MASK                              (0x1U << 4)
#define PMU_AHB_REMAP_CHIP_AHB_BUS_REMAP_POS                           0
#define PMU_AHB_REMAP_CHIP_AHB_BUS_REMAP_MASK                          (0xfU << 0)

// GPIO_ODA_CTRL
#define PMU_GPIO_ODA_CTRL_GPIO_ODA_CTRL_25_0_POS                       0
#define PMU_GPIO_ODA_CTRL_GPIO_ODA_CTRL_25_0_MASK                      (0x3ffffffU << 0)

// XTAL32M_CNS2
#define PMU_XTAL32M_CNS2_POR_WAIT_CTRL_POS                             24
#define PMU_XTAL32M_CNS2_POR_WAIT_CTRL_MASK                            (0xffU << 24)
#define PMU_XTAL32M_CNS2_LAST_WAIT_CFG_POS                             8
#define PMU_XTAL32M_CNS2_LAST_WAIT_CFG_MASK                            (0x7U << 8)
#define PMU_XTAL32M_CNS2_DN_M1T7_M2T1_CFG_POS                          5
#define PMU_XTAL32M_CNS2_DN_M1T7_M2T1_CFG_MASK                         (0x7U << 5)
#define PMU_XTAL32M_CNS2_DN_MODE_POS                                   4
#define PMU_XTAL32M_CNS2_DN_MODE_MASK                                  (0x1U << 4)

// PMU_ANA_REG
#define PMU_ANA_REG_DIG_LDO_UPDATE_POS                                 19
#define PMU_ANA_REG_DIG_LDO_UPDATE_MASK                                (0x1U << 19)
#define PMU_ANA_REG_GPIO_REUSE_POS                                     18
#define PMU_ANA_REG_GPIO_REUSE_MASK                                    (0x1U << 18)
#define PMU_ANA_REG_PMU_DIG_LDO_TRIM_POS                               14
#define PMU_ANA_REG_PMU_DIG_LDO_TRIM_MASK                              (0xfU << 14)
#define PMU_ANA_REG_RXADC_LDO_TRIM_POS                                 12
#define PMU_ANA_REG_RXADC_LDO_TRIM_MASK                                (0x3U << 12)
#define PMU_ANA_REG_IF_LDO_TRIM_POS                                    10
#define PMU_ANA_REG_IF_LDO_TRIM_MASK                                   (0x3U << 10)
#define PMU_ANA_REG_RF_LDO_TRIM_POS                                    8
#define PMU_ANA_REG_RF_LDO_TRIM_MASK                                   (0x3U << 8)
#define PMU_ANA_REG_PA_LDO_TRIM_POS                                    4
#define PMU_ANA_REG_PA_LDO_TRIM_MASK                                   (0xfU << 4)
#define PMU_ANA_REG_DCDC_CAP_TRIM_POS                                  3
#define PMU_ANA_REG_DCDC_CAP_TRIM_MASK                                 (0x1U << 3)
#define PMU_ANA_REG_DCDC_TEST_EN_POS                                   1
#define PMU_ANA_REG_DCDC_TEST_EN_MASK                                  (0x3U << 1)
#define PMU_ANA_REG_PMU_TEST_EN_POS                                    0
#define PMU_ANA_REG_PMU_TEST_EN_MASK                                   (0x1U << 0)

// PMU_CLK_CTRL_1
#define PMU_CLK_CTRL_1_CTUNE_OSC_SEL_POS                               31
#define PMU_CLK_CTRL_1_CTUNE_OSC_SEL_MASK                              (0x1U << 31)
#define PMU_CLK_CTRL_1_CT_OSC32M_RAMP_POS                              28
#define PMU_CLK_CTRL_1_CT_OSC32M_RAMP_MASK                             (0x7U << 28)
#define PMU_CLK_CTRL_1_CT_OSC32M_POS                                   20
#define PMU_CLK_CTRL_1_CT_OSC32M_MASK                                  (0xffU << 20)
#define PMU_CLK_CTRL_1_RCTUNE_RC32K_UPDATE_REG_POS                     19
#define PMU_CLK_CTRL_1_RCTUNE_RC32K_UPDATE_REG_MASK                    (0x1U << 19)
#define PMU_CLK_CTRL_1_SEL_OSC32M_FREQ_RANGE_POS                       18
#define PMU_CLK_CTRL_1_SEL_OSC32M_FREQ_RANGE_MASK                      (0x1U << 18)
#define PMU_CLK_CTRL_1_CTRL_XTAL32M_LDO_POS                            14
#define PMU_CLK_CTRL_1_CTRL_XTAL32M_LDO_MASK                           (0x3U << 14)
#define PMU_CLK_CTRL_1_RC_32K_RCTUNE_SEL_POS                           12
#define PMU_CLK_CTRL_1_RC_32K_RCTUNE_SEL_MASK                          (0x1U << 12)
#define PMU_CLK_CTRL_1_SEL_XTAL32M_GM_POS                              9
#define PMU_CLK_CTRL_1_SEL_XTAL32M_GM_MASK                             (0x7U << 9)
#define PMU_CLK_CTRL_1_CT_FDOUBLER_POS                                 0
#define PMU_CLK_CTRL_1_CT_FDOUBLER_MASK                                (0xfU << 0)

// PMU_CLK_CTRL_2
#define PMU_CLK_CTRL_2_XTAL32K_ISEL_POS                                25
#define PMU_CLK_CTRL_2_XTAL32K_ISEL_MASK                               (0x3U << 25)
#define PMU_CLK_CTRL_2_CT_XTAL32M_POS                                  19
#define PMU_CLK_CTRL_2_CT_XTAL32M_MASK                                 (0x3fU << 19)
#define PMU_CLK_CTRL_2_SEL_RCOSC_POS                                   15
#define PMU_CLK_CTRL_2_SEL_RCOSC_MASK                                  (0x1U << 15)
#define PMU_CLK_CTRL_2_CTUNE_RC32K_REG_POS                             8
#define PMU_CLK_CTRL_2_CTUNE_RC32K_REG_MASK                            (0x7fU << 8)
#define PMU_CLK_CTRL_2_RTUNE_RC32K_REG_POS                             0
#define PMU_CLK_CTRL_2_RTUNE_RC32K_REG_MASK                            (0xffU << 0)

// GPIO_IE_CTRL
#define PMU_GPIO_IE_CTRL_GPIO_SF_CTRL_POS                              26
#define PMU_GPIO_IE_CTRL_GPIO_SF_CTRL_MASK                             (0x3fU << 26)
#define PMU_GPIO_IE_CTRL_GPIO_IE_CTRL_POS                               0
#define PMU_GPIO_IE_CTRL_GPIO_IE_CTRL_MASK                              (0x3ffffffU << 0)

// XTAL32K_CTRL
#define PMU_XTAL32K_CTRL_XTAL32K_CTUNE_POS                             5
#define PMU_XTAL32K_CTRL_XTAL32K_CTUNE_MASK                            (0x3U << 5)
#define PMU_XTAL32K_CTRL_XTAL32K_DEG_PD_POS                            4
#define PMU_XTAL32K_CTRL_XTAL32K_DEG_PD_MASK                           (0x1U << 4)
#define PMU_XTAL32K_CTRL_XTAL32K_VBAT_SEL_POS                          3
#define PMU_XTAL32K_CTRL_XTAL32K_VBAT_SEL_MASK                         (0x1U << 3)

// PMU_ANA_PD_1
#define PMU_ANA_PD_1_DCDC_PSM_VREF_POS                                 21
#define PMU_ANA_PD_1_DCDC_PSM_VREF_MASK                                (0x3U << 21)
#define PMU_ANA_PD_1_DIG_RETLDO_TRIM_POS                               18
#define PMU_ANA_PD_1_DIG_RETLDO_TRIM_MASK                              (0x7U << 18)
#define PMU_ANA_PD_1_DIG_LDO_ISEL_POS                                  16
#define PMU_ANA_PD_1_DIG_LDO_ISEL_MASK                                 (0x3U << 16)
#define PMU_ANA_PD_1_PD_XTAL32M_PLLBUF_SEL_POS                         15
#define PMU_ANA_PD_1_PD_XTAL32M_PLLBUF_SEL_MASK                        (0x1U << 15)
#define PMU_ANA_PD_1_PD_XTAL32M_PLLBUF_POS                             14
#define PMU_ANA_PD_1_PD_XTAL32M_PLLBUF_MASK                            (0x1U << 14)
#define PMU_ANA_PD_1_PD_XTAL32M_DIGBUF_SEL_POS                         13
#define PMU_ANA_PD_1_PD_XTAL32M_DIGBUF_SEL_MASK                        (0x1U << 13)
#define PMU_ANA_PD_1_PD_XTAL32M_DIGBUF_POS                             12
#define PMU_ANA_PD_1_PD_XTAL32M_DIGBUF_MASK                            (0x1U << 12)
#define PMU_ANA_PD_1_EN_RC32K_TST_POS                                  9
#define PMU_ANA_PD_1_EN_RC32K_TST_MASK                                 (0x1U << 9)
#define PMU_ANA_PD_1_PD_LDO_RET1P2_POS                                 8
#define PMU_ANA_PD_1_PD_LDO_RET1P2_MASK                                (0x1U << 8)
#define PMU_ANA_PD_1_PD_PSM_DCDC_POS                                   7
#define PMU_ANA_PD_1_PD_PSM_DCDC_MASK                                  (0x1U << 7)
#define PMU_ANA_PD_1_DCDC_ZCS_TRIM_POS                                 5
#define PMU_ANA_PD_1_DCDC_ZCS_TRIM_MASK                                (0x1U << 5)
#define PMU_ANA_PD_1_DCDC_VOUT_POS                                     0
#define PMU_ANA_PD_1_DCDC_VOUT_MASK                                    (0x1fU << 0)

// PMU_MISC_CTRL_1
#define PMU_MISC_CTRL_1_EN_RSTB_POS                                    24
#define PMU_MISC_CTRL_1_EN_RSTB_MASK                                   (0x1U << 24)
#define PMU_MISC_CTRL_1_CPU_JTAG_ICG_EN_POS                            23
#define PMU_MISC_CTRL_1_CPU_JTAG_ICG_EN_MASK                           (0x1U << 23)
#define PMU_MISC_CTRL_1_CPU_JTAG_TEST_EN_POS                           22
#define PMU_MISC_CTRL_1_CPU_JTAG_TEST_EN_MASK                          (0x1U << 22)
#define PMU_MISC_CTRL_1_RC32K_PSO_GATE_POS                             20
#define PMU_MISC_CTRL_1_RC32K_PSO_GATE_MASK                            (0x1U << 20)
#define PMU_MISC_CTRL_1_OSC_CLK1M_GATE_POS                             19
#define PMU_MISC_CTRL_1_OSC_CLK1M_GATE_MASK                            (0x1U << 19)
#define PMU_MISC_CTRL_1_CRY32M_EN_POS                                  18
#define PMU_MISC_CTRL_1_CRY32M_EN_MASK                                 (0x1U << 18)
#define PMU_MISC_CTRL_1_REG_PD_RC32M_POS                               13
#define PMU_MISC_CTRL_1_REG_PD_RC32M_MASK                              (0x1U << 13)
#define PMU_MISC_CTRL_1_XTAL32M_BUF_CTRL_SEL_POS                       7
#define PMU_MISC_CTRL_1_XTAL32M_BUF_CTRL_SEL_MASK                      (0x1U << 7)
#define PMU_MISC_CTRL_1_GPIO_LATCH_CLK_EN_POS                          5
#define PMU_MISC_CTRL_1_GPIO_LATCH_CLK_EN_MASK                         (0x1U << 5)
#define PMU_MISC_CTRL_1_CRY32M_KEEP_ON_POS                             4
#define PMU_MISC_CTRL_1_CRY32M_KEEP_ON_MASK                            (0x1U << 4)
#define PMU_MISC_CTRL_1_WDT_RESET_ALL_EN_POS                           3
#define PMU_MISC_CTRL_1_WDT_RESET_ALL_EN_MASK                          (0x1U << 3)
#define PMU_MISC_CTRL_1_WDT_RESET_PSO_EN_POS                           2
#define PMU_MISC_CTRL_1_WDT_RESET_PSO_EN_MASK                          (0x1U << 2)
#define PMU_MISC_CTRL_1_WDT_RESET_REMAP_DIS_POS                        1
#define PMU_MISC_CTRL_1_WDT_RESET_REMAP_DIS_MASK                       (0x1U << 1)
#define PMU_MISC_CTRL_1_DEEP_SLEEP_CPU_INT_DIS_POS                     0
#define PMU_MISC_CTRL_1_DEEP_SLEEP_CPU_INT_DIS_MASK                    (0x1U << 0)

// PMU_RAM_CTRL_1
#define PMU_RAM_CTRL_1_RAM_IC_LS_REG_POS                               18
#define PMU_RAM_CTRL_1_RAM_IC_LS_REG_MASK                              (0x1U << 18)
#define PMU_RAM_CTRL_1_RAM_IC_SD_REG_POS                               17
#define PMU_RAM_CTRL_1_RAM_IC_SD_REG_MASK                              (0x1U << 17)
#define PMU_RAM_CTRL_1_RAM_IC_DS_REG_POS                               16
#define PMU_RAM_CTRL_1_RAM_IC_DS_REG_MASK                              (0x1U << 16)
#define PMU_RAM_CTRL_1_RAM_BLE_SD_REG_POS                              15
#define PMU_RAM_CTRL_1_RAM_BLE_SD_REG_MASK                             (0x1U << 15)
#define PMU_RAM_CTRL_1_RAM_BLE_DS_REG_POS                              14
#define PMU_RAM_CTRL_1_RAM_BLE_DS_REG_MASK                             (0x1U << 14)
#define PMU_RAM_CTRL_1_RAM_PSO_SD_REG_POS                              13
#define PMU_RAM_CTRL_1_RAM_PSO_SD_REG_MASK                             (0x1U << 13)
#define PMU_RAM_CTRL_1_RAM_PSO_DS_REG_POS                              12
#define PMU_RAM_CTRL_1_RAM_PSO_DS_REG_MASK                             (0x1U << 12)
#define PMU_RAM_CTRL_1_ROM_LS_POS                                      11
#define PMU_RAM_CTRL_1_ROM_LS_MASK                                     (0x1U << 11)
#define PMU_RAM_CTRL_1_RAM_BLE_LS_POS                                  10
#define PMU_RAM_CTRL_1_RAM_BLE_LS_MASK                                 (0x1U << 10)
#define PMU_RAM_CTRL_1_RAM_PSO_LS_POS                                  9
#define PMU_RAM_CTRL_1_RAM_PSO_LS_MASK                                 (0x1U << 9)
#define PMU_RAM_CTRL_1_RAM2_SD_POS                                     8
#define PMU_RAM_CTRL_1_RAM2_SD_MASK                                    (0x1U << 8)
#define PMU_RAM_CTRL_1_RAM1_SD_POS                                     7
#define PMU_RAM_CTRL_1_RAM1_SD_MASK                                    (0x1U << 7)
#define PMU_RAM_CTRL_1_RAM0_SD_POS                                     6
#define PMU_RAM_CTRL_1_RAM0_SD_MASK                                    (0x1U << 6)
#define PMU_RAM_CTRL_1_RAM2_DS_RAM2_LS_POS                             4
#define PMU_RAM_CTRL_1_RAM2_DS_RAM2_LS_MASK                            (0x3U << 4)
#define PMU_RAM_CTRL_1_RAM1_DS_RAM1_LS_POS                             2
#define PMU_RAM_CTRL_1_RAM1_DS_RAM1_LS_MASK                            (0x3U << 2)
#define PMU_RAM_CTRL_1_RAM0_DS_RAM0_LS_POS                             0
#define PMU_RAM_CTRL_1_RAM0_DS_RAM0_LS_MASK                            (0x3U << 0)

// PMU_RAM_CTRL_2
#define PMU_RAM_CTRL_2_RAM_TIMING_REG_CTRL_POS                         30
#define PMU_RAM_CTRL_2_RAM_TIMING_REG_CTRL_MASK                        (0x1U << 30)
#define PMU_RAM_CTRL_2_RAM_BLE_SD_HW_CTRL_EN_POS                       29
#define PMU_RAM_CTRL_2_RAM_BLE_SD_HW_CTRL_EN_MASK                      (0x1U << 29)
#define PMU_RAM_CTRL_2_RAM_BLE_DS_HW_CTRL_EN_POS                       28
#define PMU_RAM_CTRL_2_RAM_BLE_DS_HW_CTRL_EN_MASK                      (0x1U << 28)
#define PMU_RAM_CTRL_2_RAM_IC_SD_HW_CTRL_EN_POS                        27
#define PMU_RAM_CTRL_2_RAM_IC_SD_HW_CTRL_EN_MASK                       (0x1U << 27)
#define PMU_RAM_CTRL_2_RAM_IC_DS_HW_CTRL_EN_POS                        26
#define PMU_RAM_CTRL_2_RAM_IC_DS_HW_CTRL_EN_MASK                       (0x1U << 26)
#define PMU_RAM_CTRL_2_RAM_PSO_SD_HW_CTRL_EN_POS                       25
#define PMU_RAM_CTRL_2_RAM_PSO_SD_HW_CTRL_EN_MASK                      (0x1U << 25)
#define PMU_RAM_CTRL_2_RAM_PSO_DS_HW_CTRL_EN_POS                       24
#define PMU_RAM_CTRL_2_RAM_PSO_DS_HW_CTRL_EN_MASK                      (0x1U << 24)
#define PMU_RAM_CTRL_2_RAM2_SD_HW_CTRL_EN_POS                          23
#define PMU_RAM_CTRL_2_RAM2_SD_HW_CTRL_EN_MASK                         (0x1U << 23)
#define PMU_RAM_CTRL_2_RAM1_SD_HW_CTRL_EN_POS                          22
#define PMU_RAM_CTRL_2_RAM1_SD_HW_CTRL_EN_MASK                         (0x1U << 22)
#define PMU_RAM_CTRL_2_RAM0_SD_HW_CTRL_EN_POS                          21
#define PMU_RAM_CTRL_2_RAM0_SD_HW_CTRL_EN_MASK                         (0x1U << 21)
#define PMU_RAM_CTRL_2_RAM2_DS_HW_CTRL_EN_POS                          20
#define PMU_RAM_CTRL_2_RAM2_DS_HW_CTRL_EN_MASK                         (0x1U << 20)
#define PMU_RAM_CTRL_2_RAM1_DS_HW_CTRL_EN_POS                          19
#define PMU_RAM_CTRL_2_RAM1_DS_HW_CTRL_EN_MASK                         (0x1U << 19)
#define PMU_RAM_CTRL_2_RAM0_DS_HW_CTRL_EN_POS                          18
#define PMU_RAM_CTRL_2_RAM0_DS_HW_CTRL_EN_MASK                         (0x1U << 18)
#define PMU_RAM_CTRL_2_ROM_RM_POS                                      14
#define PMU_RAM_CTRL_2_ROM_RM_MASK                                     (0xfU << 14)
#define PMU_RAM_CTRL_2_RAM_TEST1_POS                                   13
#define PMU_RAM_CTRL_2_RAM_TEST1_MASK                                  (0x1U << 13)
#define PMU_RAM_CTRL_2_RAM_RME_POS                                     12
#define PMU_RAM_CTRL_2_RAM_RME_MASK                                    (0x1U << 12)
#define PMU_RAM_CTRL_2_RAM_RM_POS                                      8
#define PMU_RAM_CTRL_2_RAM_RM_MASK                                     (0xfU << 8)
#define PMU_RAM_CTRL_2_RAM_RA_POS                                      6
#define PMU_RAM_CTRL_2_RAM_RA_MASK                                     (0x3U << 6)
#define PMU_RAM_CTRL_2_RAM_WA_POS                                      3
#define PMU_RAM_CTRL_2_RAM_WA_MASK                                     (0x7U << 3)
#define PMU_RAM_CTRL_2_RAM_WPULSE_POS                                  0
#define PMU_RAM_CTRL_2_RAM_WPULSE_MASK                                 (0x7U << 0)

// PMU_TIMER_SET0
#define PMU_TIMER_SET0_PMU_TIMER_VAL0_POS                              0
#define PMU_TIMER_SET0_PMU_TIMER_VAL0_MASK                             (0xffffffffU << 0)

// PMU_TIMER_SET1
#define PMU_TIMER_SET1_PMU_TIMER_VAL1_POS                              0
#define PMU_TIMER_SET1_PMU_TIMER_VAL1_MASK                             (0xffffffffU << 0)

// PMU_TIMER_PPM
#define PMU_TIMER_PPM_PMU_TIMER_INCR_POS                               0
#define PMU_TIMER_PPM_PMU_TIMER_INCR_MASK                              (0x1ffffU << 0)

// PMU_GPIO_STATUS_READ
#define PMU_GPIO_STATUS_READ_GPIO_OEB_AUTO_LATCH_25_0_POS              0
#define PMU_GPIO_STATUS_READ_GPIO_OEB_AUTO_LATCH_25_0_MASK             (0x3ffffffU << 0)

// PMU_GPIO_STATUS_READ_2
#define PMU_GPIO_STATUS_READ_2_GPIO_OUT_AUTO_LATCH_25_0_POS            0
#define PMU_GPIO_STATUS_READ_2_GPIO_OUT_AUTO_LATCH_25_0_MASK           (0x3ffffffU << 0)

// WDT_STATUS
#define PMU_WDT_STATUS_WDT_TIMER_POS                                   12
#define PMU_WDT_STATUS_WDT_TIMER_MASK                                  (0xfffffU << 12)
#define PMU_WDT_STATUS_WDT_FLAG_CLR_POS                                9
#define PMU_WDT_STATUS_WDT_FLAG_CLR_MASK                               (0x1U << 9)
#define PMU_WDT_STATUS_WDT_INT_CPU_EN_POS                              8
#define PMU_WDT_STATUS_WDT_INT_CPU_EN_MASK                             (0x1U << 8)
#define PMU_WDT_STATUS_WDT_INT_CPU_POS                                 5
#define PMU_WDT_STATUS_WDT_INT_CPU_MASK                                (0x1U << 5)
#define PMU_WDT_STATUS_WDT_FLAG_POS                                    4
#define PMU_WDT_STATUS_WDT_FLAG_MASK                                   (0x1U << 4)
#define PMU_WDT_STATUS_LD_WDT_KR_STATUS_POS                            0
#define PMU_WDT_STATUS_LD_WDT_KR_STATUS_MASK                           (0x1U << 0)

// PMU_GPIO_WAKEUP
#define PMU_GPIO_WAKEUP_GPIO_WAKE_EN_25_0_POS                          0
#define PMU_GPIO_WAKEUP_GPIO_WAKE_EN_25_0_MASK                         (0x3ffffffU << 0)

// GPIO_ODE_CTRL
#define PMU_GPIO_ODE_CTRL_GPIO_PMU_DBG_POS                             26
#define PMU_GPIO_ODE_CTRL_GPIO_PMU_DBG_MASK                            (0x1U << 26)
#define PMU_GPIO_ODE_CTRL_GPIO_ODE_REG_25_0_POS                        0
#define PMU_GPIO_ODE_CTRL_GPIO_ODE_REG_25_0_MASK                       (0x3ffffffU << 0)

// GPIO_PD_CTRL
#define PMU_GPIO_PD_CTRL_GPIO_PL_DN_31_0_POS                           0
#define PMU_GPIO_PD_CTRL_GPIO_PL_DN_31_0_MASK                          (0xffffffffU << 0)

// PMU_GPIO_LATCH
#define PMU_GPIO_LATCH_GPIO_INT_LATCH_25_0_POS                         0
#define PMU_GPIO_LATCH_GPIO_INT_LATCH_25_0_MASK                        (0x3ffffffU << 0)

// PMU_GPIO_NOCLK_LATCH
#define PMU_GPIO_NOCLK_LATCH_GPIO_INT_NOCLK_LATCH_25_0_POS             0
#define PMU_GPIO_NOCLK_LATCH_GPIO_INT_NOCLK_LATCH_25_0_MASK            (0x3ffffffU << 0)

// WDT_RLR_CFG
#define PMU_WDT_RLR_CFG_WDT_RLR_REG_POS                                0
#define PMU_WDT_RLR_CFG_WDT_RLR_REG_MASK                               (0xfffffU << 0)

// PMU_STATUS_READ
#define PMU_STATUS_READ_BB_INT_STATUS_POS                              23
#define PMU_STATUS_READ_BB_INT_STATUS_MASK                             (0x1U << 23)
#define PMU_STATUS_READ_SEL_RC32K_TUNE_POS                             22
#define PMU_STATUS_READ_SEL_RC32K_TUNE_MASK                            (0x1U << 22)
#define PMU_STATUS_READ_RCTUNE_RC32K_UPDATE_STATUS_POS                 21
#define PMU_STATUS_READ_RCTUNE_RC32K_UPDATE_STATUS_MASK                (0x1U << 21)
#define PMU_STATUS_READ_CON_SWITCH_FLASH_FSM_POS                       18
#define PMU_STATUS_READ_CON_SWITCH_FLASH_FSM_MASK                      (0x7U << 18)
#define PMU_STATUS_READ_RC32M_READY_POS                                16
#define PMU_STATUS_READ_RC32M_READY_MASK                               (0x1U << 16)
#define PMU_STATUS_READ_RC32K_READY_POS                                15
#define PMU_STATUS_READ_RC32K_READY_MASK                               (0x1U << 15)
#define PMU_STATUS_READ_PIN_WAKEUP_ORG_SYNC_POS                        14
#define PMU_STATUS_READ_PIN_WAKEUP_ORG_SYNC_MASK                       (0x1U << 14)
#define PMU_STATUS_READ_PIN_WAKEUP_DIGITAL_POS                         13
#define PMU_STATUS_READ_PIN_WAKEUP_DIGITAL_MASK                        (0x1U << 13)
#define PMU_STATUS_READ_NATIVE_INT_32K_POS                             11
#define PMU_STATUS_READ_NATIVE_INT_32K_MASK                            (0x1U << 11)
#define PMU_STATUS_READ_FIRST_RUN_POS                                  10
#define PMU_STATUS_READ_FIRST_RUN_MASK                                 (0x1U << 10)
#define PMU_STATUS_READ_CLK_32K_RC_CRY_DONE_POS                        9
#define PMU_STATUS_READ_CLK_32K_RC_CRY_DONE_MASK                       (0x1U << 9)
#define PMU_STATUS_READ_CLK_32K_DIV_DONE_POS                           8
#define PMU_STATUS_READ_CLK_32K_DIV_DONE_MASK                          (0x1U << 8)
#define PMU_STATUS_READ_CTRL_LDO_DIG_POS                               4
#define PMU_STATUS_READ_CTRL_LDO_DIG_MASK                              (0xfU << 4)
#define PMU_STATUS_READ_CLK_32K_CRY_DIN_STATUS_POS                     3
#define PMU_STATUS_READ_CLK_32K_CRY_DIN_STATUS_MASK                    (0x1U << 3)
#define PMU_STATUS_READ_CLK_32K_RC_DIN_STATUS_POS                      2
#define PMU_STATUS_READ_CLK_32K_RC_DIN_STATUS_MASK                     (0x1U << 2)
#define PMU_STATUS_READ_CLR_PMU_INT_SYNC_APB_POS                       1
#define PMU_STATUS_READ_CLR_PMU_INT_SYNC_APB_MASK                      (0x1U << 1)
#define PMU_STATUS_READ_APB_BUS_UPD_DATA_POS                           0
#define PMU_STATUS_READ_APB_BUS_UPD_DATA_MASK                          (0x1U << 0)

// GPIO_DRV_CTRL_0
#define PMU_GPIO_DRV_CTRL_0_GPIO_DRV_CTRL_0_31_26_POS                  26
#define PMU_GPIO_DRV_CTRL_0_GPIO_DRV_CTRL_0_31_26_MASK                 (0x3fU << 26)
#define PMU_GPIO_DRV_CTRL_0_GPIO_DRV_CTRL_25_0_POS                     0
#define PMU_GPIO_DRV_CTRL_0_GPIO_DRV_CTRL_25_0_MASK                    (0x3ffffffU << 0)

// GPIO_DRV_CTRL_2
#define PMU_GPIO_DRV_CTRL_2_GPIO_DRV_CTRL_1_31_26_POS                  26
#define PMU_GPIO_DRV_CTRL_2_GPIO_DRV_CTRL_1_31_26_MASK                 (0x3fU << 26)
#define PMU_GPIO_DRV_CTRL_2_GPIO_PL_CTRL_0_25_0_POS                    0
#define PMU_GPIO_DRV_CTRL_2_GPIO_PL_CTRL_0_25_0_MASK                   (0x3ffffffU << 0)

// PMU_TIMER_READ
#define PMU_TIMER_READ_PMU_TIMER_CNT_POS                               0
#define PMU_TIMER_READ_PMU_TIMER_CNT_MASK                              (0xffffffffU << 0)

// 2.4G_PID
#define PMU_PID_24G_2P4G_PID_POS                                       0
#define PMU_PID_24G_2P4G_PID_MASK                                      (0x3U << 0)

// WDT_KR_CFG
#define PMU_WDT_KR_CFG_WDT_KR_POS                                      16
#define PMU_WDT_KR_CFG_WDT_KR_MASK                                     (0xffffU << 16)
#define PMU_WDT_KR_CFG_WDT_KR_REG_POS                                  0
#define PMU_WDT_KR_CFG_WDT_KR_REG_MASK                                 (0xffffU << 0)

// PMU_RESERVED0
#define PMU_RESERVED0_SW_STATUS_POS                                    0
#define PMU_RESERVED0_SW_STATUS_MASK                                   (0xffffffffU << 0)

// PMU_RESERVED1
#define PMU_RESERVED1_CPU_STATUS_POS                                   0
#define PMU_RESERVED1_CPU_STATUS_MASK                                  (0xffffffffU << 0)

// PMU_RESERVED2
#define PMU_RESERVED2_LDR_HOOK_POS                                     0
#define PMU_RESERVED2_LDR_HOOK_MASK                                    (0xffffffffU << 0)

// PMU_RESERVED3
#define PMU_RESERVED3_BOOT_SELECT_POS                                  0
#define PMU_RESERVED3_BOOT_SELECT_MASK                                 (0xffffffffU << 0)

// FLASH_LOW_VOL_CTRL_0
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_0_POWER_READY_DIS_POS           10
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_0_POWER_READY_DIS_MASK          (0x1U << 10)
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_LOW_VOL_FLAG_CLR_POS            9
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_LOW_VOL_FLAG_CLR_MASK           (0x1U << 9)
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_LOW_VOL_FLAG_POS                7
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_LOW_VOL_FLAG_MASK               (0x1U << 7)
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_POWER_ON_FLAG_POS               5
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_POWER_ON_FLAG_MASK              (0x1U << 5)
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_POWER_READY_SYNC_POS            3
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_POWER_READY_SYNC_MASK           (0x1U << 3)
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_LOW_VOL_RSTB_DIS_POS            1
#define PMU_FLASH_LOW_VOL_CTRL_0_FLASH_LOW_VOL_RSTB_DIS_MASK           (0x1U << 1)

// FLASH_LOW_VOL_CTRL_1

// PMU_TIMER_CTRL
#define PMU_TIMER_CTRL_PMU_TIMER_INT1_POS                              7
#define PMU_TIMER_CTRL_PMU_TIMER_INT1_MASK                             (0x1U << 7)
#define PMU_TIMER_CTRL_PMU_TIMER_INT0_POS                              6
#define PMU_TIMER_CTRL_PMU_TIMER_INT0_MASK                             (0x1U << 6)
#define PMU_TIMER_CTRL_PMU_TIMER_INT_POS                               5
#define PMU_TIMER_CTRL_PMU_TIMER_INT_MASK                              (0x1U << 5)
#define PMU_TIMER_CTRL_PMU_TIMER_INT_VAL1_CLR_POS                      4
#define PMU_TIMER_CTRL_PMU_TIMER_INT_VAL1_CLR_MASK                     (0x1U << 4)
#define PMU_TIMER_CTRL_PMU_TIMER_INT_VAL0_CLR_POS                      3
#define PMU_TIMER_CTRL_PMU_TIMER_INT_VAL0_CLR_MASK                     (0x1U << 3)
#define PMU_TIMER_CTRL_PMU_TIMER_INT_VAL1_EN_POS                       2
#define PMU_TIMER_CTRL_PMU_TIMER_INT_VAL1_EN_MASK                      (0x1U << 2)
#define PMU_TIMER_CTRL_PMU_TIMER_INT_VAL0_EN_POS                       1
#define PMU_TIMER_CTRL_PMU_TIMER_INT_VAL0_EN_MASK                      (0x1U << 1)
#define PMU_TIMER_CTRL_PMU_TIMER_EN_POS                                0
#define PMU_TIMER_CTRL_PMU_TIMER_EN_MASK                               (0x1U << 0)

// PMU_TRIM_CHANGE
#define PMU_TRIM_CHANGE_TRIM_CHANGE_POS                                0
#define PMU_TRIM_CHANGE_TRIM_CHANGE_MASK                               (0x1U << 0)

// PMU_TRIM_SET
#define PMU_TRIM_SET_PMU_TRIM_VREF_POS                                 0
#define PMU_TRIM_SET_PMU_TRIM_VREF_MASK                                (0xfU << 0)

// ANA_TEST_CTRL
#define PMU_ANA_TEST_CTRL_REG_ANA_TEST_POS                             0
#define PMU_ANA_TEST_CTRL_REG_ANA_TEST_MASK                            (0x3ffU << 0)

// RSVD_REG_0
#define PMU_RSVD_REG_0_RSVD_REG_0_POS                                  0
#define PMU_RSVD_REG_0_RSVD_REG_0_MASK                                 (0xffffffffU << 0)

// RSVD_REG_1
#define PMU_RSVD_REG_1_RSVD_REG_1_POS                                  0
#define PMU_RSVD_REG_1_RSVD_REG_1_MASK                                 (0xffffffffU << 0)

// SLEEP_WAKE_CTRL
#define PMU_SLEEP_WAKE_CTRL_RETENTION_RESTORE_EN_POS                   7
#define PMU_SLEEP_WAKE_CTRL_RETENTION_RESTORE_EN_MASK                  (0x1U << 7)
#define PMU_SLEEP_WAKE_CTRL_RETENTION_SAVE_EN_POS                      6
#define PMU_SLEEP_WAKE_CTRL_RETENTION_SAVE_EN_MASK                     (0x1U << 6)
#define PMU_SLEEP_WAKE_CTRL_SF_RDI_EN_POS                              5
#define PMU_SLEEP_WAKE_CTRL_SF_RDI_EN_MASK                             (0x1U << 5)
#define PMU_SLEEP_WAKE_CTRL_SF_DP_EN_POS                               4
#define PMU_SLEEP_WAKE_CTRL_SF_DP_EN_MASK                              (0x1U << 4)
#define PMU_SLEEP_WAKE_CTRL_SF_RDI_WAIT_CTRL_POS                       2
#define PMU_SLEEP_WAKE_CTRL_SF_RDI_WAIT_CTRL_MASK                      (0x3U << 2)
#define PMU_SLEEP_WAKE_CTRL_SF_DP_WAIT_CTRL_POS                        0
#define PMU_SLEEP_WAKE_CTRL_SF_DP_WAIT_CTRL_MASK                       (0x3U << 0)

// BLE_CLK_SWITCH
#define PMU_BLE_CLK_SWITCH_BLE_CLK_SWITCH_POS                           0
#define PMU_BLE_CLK_SWITCH_BLE_CLK_SWITCH_MASK                          (0x1U << 0)
#define PMU_BLE_CLK_SWITCH_BLE_TIM_NATIVE_INT_EN_POS                    3
#define PMU_BLE_CLK_SWITCH_BLE_TIM_NATIVE_INT_EN_MASK                   (0x1U << 3)
#define PMU_BLE_CLK_SWITCH_BLE_SLEEP_MODE_POS                           4
#define PMU_BLE_CLK_SWITCH_BLE_SLEEP_MODE_MASK                          (0x1U << 4)

// BLE_RECOVER_FLAG
#define PMU_BLE_RECOVER_FLAG_RECOVER_DONE_FLAG_POS                      0
#define PMU_BLE_RECOVER_FLAG_RECOVER_DONE_FLAG_MASK                     (0x1U << 0)

// BLE_TIMER_CALI

// OSC_INT_CTRL
#define PMU_OSC_INI_CTRL_BLE_WAKEUP_INT_CLR_POS                         0
#define PMU_OSC_INI_CTRL_BLE_WAKEUP_INT_CLR_MASK                        (0x1U << 0)
#define PMU_OSC_INI_CTRL_BLE_WAKEUP_INT_MASK_PMU_POS                    4
#define PMU_OSC_INI_CTRL_BLE_WAKEUP_INT_MASK_PMU_MASK                   (0x1U << 4)
#define PMU_OSC_INI_CTRL_BLE_WAKEUP_INT_MASK_CPU_POS                    5
#define PMU_OSC_INI_CTRL_BLE_WAKEUP_INT_MASK_CPU_MASK                   (0x1U << 5)

// DVDD_CHG_CTRL
#define PMU_DVDD_CHG_CTRL_DVDD_VOL_SEL_POS                             1
#define PMU_DVDD_CHG_CTRL_DVDD_VOL_SEL_MASK                            (0x1U << 1)
#define PMU_DVDD_CHG_CTRL_DVDD_CHANGE_START_POS                        0
#define PMU_DVDD_CHG_CTRL_DVDD_CHANGE_START_MASK                       (0x1U << 0)

// FUNC_FBD_CTRL
#define PMU_FUNC_FBD_CTRL_RAM_32K_DIS_POS                              1
#define PMU_FUNC_FBD_CTRL_RAM_32K_DIS_MASK                             (0x1U << 1)
#define PMU_FUNC_FBD_CTRL_BLE_FUNC_DIS_POS                             0
#define PMU_FUNC_FBD_CTRL_BLE_FUNC_DIS_MASK                            (0x1U << 0)

// PMU_POF_INT_CTRL
#define PMU_POF_INT_CTRL_PMU_PD_POF_REG_POS                            7
#define PMU_POF_INT_CTRL_PMU_PD_POF_REG_MASK                           (0x1U << 7)
#define PMU_POF_INT_CTRL_PMU_PD_POF_ME_POS                             6
#define PMU_POF_INT_CTRL_PMU_PD_POF_ME_MASK                            (0x1U << 6)
#define PMU_POF_INT_CTRL_PMU_PD_POF_EN_POS                             5
#define PMU_POF_INT_CTRL_PMU_PD_POF_EN_MASK                            (0x1U << 5)
#define PMU_POF_INT_CTRL_PMU_POF_TH_REG_POS                            2
#define PMU_POF_INT_CTRL_PMU_POF_TH_REG_MASK                           (0x7U << 2)
#define PMU_POF_INT_CTRL_PMU_POF_INT_CLR_POS                           1
#define PMU_POF_INT_CTRL_PMU_POF_INT_CLR_MASK                          (0x1U << 1)
#define PMU_POF_INT_CTRL_PMU_POF_INT_EN_POS                            0
#define PMU_POF_INT_CTRL_PMU_POF_INT_EN_MASK                           (0x1U << 0)

// XTAL32M_HSEDET_CTRL
#define PMU_XTAL32M_HSEDET_CTRL_XTAL32M_HSEDET_FLAG_POS                3
#define PMU_XTAL32M_HSEDET_CTRL_XTAL32M_HSEDET_FLAG_MASK               (0x1U << 3)
#define PMU_XTAL32M_HSEDET_CTRL_XTAL32M_HSEDET_INT_POS                 2
#define PMU_XTAL32M_HSEDET_CTRL_XTAL32M_HSEDET_INT_MASK                (0x1U << 2)
#define PMU_XTAL32M_HSEDET_CTRL_XTAL32M_HSEDET_INT_DIS_POS             1
#define PMU_XTAL32M_HSEDET_CTRL_XTAL32M_HSEDET_INT_DIS_MASK            (0x1U << 1)
#define PMU_XTAL32M_HSEDET_CTRL_XTAL32M_HSEDET_EN_POS                  0
#define PMU_XTAL32M_HSEDET_CTRL_XTAL32M_HSEDET_EN_MASK                 (0x1U << 0)

// PMU_REG_SW_CTRL_1
#define PMU_REG_SW_CTRL_1_PMU_RC32K_PD_MO_POS                          18
#define PMU_REG_SW_CTRL_1_PMU_RC32K_PD_MO_MASK                         (0x1U << 18)
#define PMU_REG_SW_CTRL_1_PMU_RC32K_PD_ME_POS                          17
#define PMU_REG_SW_CTRL_1_PMU_RC32K_PD_ME_MASK                         (0x1U << 17)
#define PMU_REG_SW_CTRL_1_PMU_OSC32M_EN_MO_POS                         16
#define PMU_REG_SW_CTRL_1_PMU_OSC32M_EN_MO_MASK                        (0x1U << 16)
#define PMU_REG_SW_CTRL_1_PMU_OSC32M_EN_ME_POS                         15
#define PMU_REG_SW_CTRL_1_PMU_OSC32M_EN_ME_MASK                        (0x1U << 15)
#define PMU_REG_SW_CTRL_1_PMU_OSC32M_EN_BIAS_MO_POS                    14
#define PMU_REG_SW_CTRL_1_PMU_OSC32M_EN_BIAS_MO_MASK                   (0x1U << 14)
#define PMU_REG_SW_CTRL_1_PMU_OSC32M_EN_BIAS_ME_POS                    13
#define PMU_REG_SW_CTRL_1_PMU_OSC32M_EN_BIAS_ME_MASK                   (0x1U << 13)
#define PMU_REG_SW_CTRL_1_PMU_XTAL32M_PD_CORE_MO_POS                   12
#define PMU_REG_SW_CTRL_1_PMU_XTAL32M_PD_CORE_MO_MASK                  (0x1U << 12)
#define PMU_REG_SW_CTRL_1_PMU_XTAL32M_PD_CORE_ME_POS                   11
#define PMU_REG_SW_CTRL_1_PMU_XTAL32M_PD_CORE_ME_MASK                  (0x1U << 11)
#define PMU_REG_SW_CTRL_1_PMU_DCDC_PD_OSC_MO_POS                       10
#define PMU_REG_SW_CTRL_1_PMU_DCDC_PD_OSC_MO_MASK                      (0x1U << 10)
#define PMU_REG_SW_CTRL_1_PMU_DCDC_PD_OSC_ME_POS                       9
#define PMU_REG_SW_CTRL_1_PMU_DCDC_PD_OSC_ME_MASK                      (0x1U << 9)
#define PMU_REG_SW_CTRL_1_PMU_DCDC_PD_MO_POS                           8
#define PMU_REG_SW_CTRL_1_PMU_DCDC_PD_MO_MASK                          (0x1U << 8)
#define PMU_REG_SW_CTRL_1_PMU_DCDC_PD_ME_POS                           7
#define PMU_REG_SW_CTRL_1_PMU_DCDC_PD_ME_MASK                          (0x1U << 7)
#define PMU_REG_SW_CTRL_1_PMU_PD_BOR_MO_POS                            6
#define PMU_REG_SW_CTRL_1_PMU_PD_BOR_MO_MASK                           (0x1U << 6)
#define PMU_REG_SW_CTRL_1_PMU_PD_BOR_ME_POS                            5
#define PMU_REG_SW_CTRL_1_PMU_PD_BOR_ME_MASK                           (0x1U << 5)
#define PMU_REG_SW_CTRL_1_PMU_LDO_ANA1P2_PD_MO_POS                     4
#define PMU_REG_SW_CTRL_1_PMU_LDO_ANA1P2_PD_MO_MASK                    (0x1U << 4)
#define PMU_REG_SW_CTRL_1_PMU_LDO_ANA1P2_PD_ME_POS                     3
#define PMU_REG_SW_CTRL_1_PMU_LDO_ANA1P2_PD_ME_MASK                    (0x1U << 3)
#define PMU_REG_SW_CTRL_1_ENB_EFUSE_DVDD_PWRSW_MO_POS                  1
#define PMU_REG_SW_CTRL_1_ENB_EFUSE_DVDD_PWRSW_MO_MASK                 (0x3U << 1)
#define PMU_REG_SW_CTRL_1_ENB_EFUSE_DVDD_PWRSW_ME_POS                  0
#define PMU_REG_SW_CTRL_1_ENB_EFUSE_DVDD_PWRSW_ME_MASK                 (0x1U << 0)


// SW_STATUS
#define PMU_SW_STATUS_SYS_CALIBED_MASK              0x00000001
#define PMU_SW_STATUS_RF_CALIBED_MASK               0x00000002
#define PMU_SW_STATUS_APP_RUNNED_MASK               0x00000004
#define PMU_SW_STATUS_REBOOT_FORCE_MASK             0x00000008
#define PMU_SW_STATUS_REBOOT_FROM_ISP_MASK          0x00000010
#define PMU_SW_STATUS_REBOOT_FROM_SLEEP_MASK        0x00000020
#define PMU_SW_STATUS_REBOOT_SW_MASK                (PMU_SW_STATUS_REBOOT_FORCE_MASK | PMU_SW_STATUS_REBOOT_FROM_ISP_MASK | PMU_SW_STATUS_REBOOT_FROM_SLEEP_MASK)
#define PMU_SW_STATUS_REBOOT_FROM_WDT_MASK          0x00000040
#define PMU_SW_STATUS_REBOOT_FROM_SOFT_RESET_MASK   0x00000080
#define PMU_SW_STATUS_REBOOT_FROM_IFLASH_LOW_V_MASK 0x00000100
#define PMU_SW_STATUS_REBOOT_HW_MASK                (PMU_SW_STATUS_REBOOT_FROM_SLEEP_MASK | PMU_SW_STATUS_REBOOT_FROM_WDT_MASK | PMU_SW_STATUS_REBOOT_FROM_SOFT_RESET_MASK | PMU_SW_STATUS_REBOOT_FROM_IFLASH_LOW_V_MASK)
#define PMU_SW_STATUS_REBOOT_MASK                   (PMU_SW_STATUS_REBOOT_SW_MASK | PMU_SW_STATUS_REBOOT_HW_MASK)
//#define PMU_SW_STATUS_XTAL32M_OPENED_MASK           0x00000100
#define PMU_SW_STATUS_AGGRESSIVE_CPUGATE_MASK       0x00000200
#define PMU_SW_STATUS_XTAL32M_ABSENT_MASK           0x00000400
#define PMU_SW_STATUS_DCDC_ENABLED_MASK             0x00000800
#define PMU_SW_STATUS_FLASH_OPEND_MASK              0x00001000
#define PMU_SW_STATUS_OTP_OPEND_MASK                0x00002000
#define PMU_SW_STATUS_SCBOOT_IN_FLASH_MASK          0x00004000
#define PMU_SW_STATUS_USER_RETENTION_POS            16
#define PMU_SW_STATUS_USER_RETENTION_MASK           0xFFFF0000

/*******************************************************************************
 * TYPEDEFS
 */
typedef struct
{
    __IO uint32_t BASIC;                                    // offset:0x00
    __IO uint32_t PSO_PM;                                   // offset:0x04
    __IO uint32_t XTAL32M_CNS0;                             // offset:0x08
         uint8_t  Reserved0c_14[0x14 - 0x0c + 4];
    __IO uint32_t ANA_PD;                                   // offset:0x18
    __IO uint32_t GPIO_POL;                                 // offset:0x1C
    __IO uint32_t GPIO_PU_CTRL_1;                           // offset:0x20
    __IO uint32_t MISC_CTRL;                                // offset:0x24
    __IO uint32_t WAKE_DEB;                                 // offset:0x28
    __IO uint32_t GPIO_OE_CTRL;                             // offset:0x2C
    __IO uint32_t GPIO_OE_CTRL_1;                           // offset:0x30
    __IO uint32_t GPIO_PU_CTRL;                             // offset:0x34
    __IO uint32_t XTAL32M_CNS1;                             // offset:0x38
    __IO uint32_t AHB_REMAP;                                // offset:0x3C
    __IO uint32_t GPIO_ODA_CTRL;                            // offset:0x40
    __IO uint32_t XTAL32M_CNS2;                             // offset:0x44
    __IO uint32_t ANA_REG;                                  // offset:0x48
    __IO uint32_t CLK_CTRL_1;                               // offset:0x4C
    __IO uint32_t CLK_CTRL_2;                               // offset:0x50
    __IO uint32_t GPIO_IE_CTRL;                             // offset:0x54
         uint32_t Reserved58;
    __IO uint32_t XTAL32K_CTRL;                             // offset:0x5C
    __IO uint32_t ANA_PD_1;                                 // offset:0x60
    __IO uint32_t MISC_CTRL_1;                              // offset:0x64
    __IO uint32_t RAM_CTRL_1;                               // offset:0x68
    __IO uint32_t RAM_CTRL_2;                               // offset:0x6C
         uint8_t  Reserved70_78[0x78 - 0x70 + 4];
    __IO uint32_t TIMER_SET0;                               // offset:0x7C
    __IO uint32_t TIMER_SET1;                               // offset:0x80
         uint32_t TIMER_PPM;                                // offset:0x84
    __IO uint32_t GPIO_STATUS_READ;                         // offset:0x88
         uint32_t Reserved8c;
    __IO uint32_t GPIO_STATUS_READ_2;                       // offset:0x90
    __IO uint32_t WDT_STATUS;                               // offset:0x94
    __IO uint32_t GPIO_WAKEUP;                              // offset:0x98
         uint32_t Reserved9c;
    __IO uint32_t GPIO_ODE_CTRL;                            // offset:0xA0
         uint32_t Reserveda4;
    __IO uint32_t GPIO_PD_CTRL;                             // offset:0xA8
         uint32_t Reservedac;
    __IO uint32_t GPIO_LATCH;                               // offset:0xB0
         uint32_t Reservedb4;
    __IO uint32_t GPIO_NOCLK_LATCH;                         // offset:0xB8
    __IO uint32_t WDT_RLR_CFG;                              // offset:0xBC
    __IO uint32_t STATUS_READ;                              // offset:0xC0
    __IO uint32_t GPIO_DRV_CTRL_0;                          // offset:0xC4
         uint32_t Reservedc8;
    __IO uint32_t GPIO_DRV_CTRL_2;                          // offset:0xCC
         uint32_t Reservedd0;
    __IO uint32_t TIMER_READ;                               // offset:0xD4
    __IO uint32_t PID_24G;                                  // offset:0xD8
         uint8_t  Reserveddc_e4[0xe4 - 0xdc + 4];
    __IO uint32_t WDT_KR_CFG;                               // offset:0xE8
    __IO uint32_t SW_STATUS;                                // offset:0xEC
    __IO uint32_t CPU_STATUS;                               // offset:0xF0
    __IO uint32_t LDR_HOOK;                                 // offset:0xF4
    __IO uint32_t BOOT_SEL;                                 // offset:0xF8
         uint32_t Reservedfc;
    __IO uint32_t FLASH_LOW_VOL_CTRL_0;                     // offset:0x0100
    __IO uint32_t FLASH_LOW_VOL_CTRL_1;                     // offset:0x0104
         uint32_t Reserved108;
    __IO uint32_t TIMER_CTRL;                               // offset:0x010C
    __IO uint32_t TRIM_CHANGE;                              // offset:0x0110
    __IO uint32_t TRIM_SET;                                 // offset:0x0114
    __IO uint32_t LP_TIMER_OUT_CTRL;                        // offset:0x0118
         uint8_t  Reserved11c_184[0x184 - 0x11c + 4];
    __IO uint32_t ANA_TEST_CTRL;                            // offset:0x0188
    __IO uint32_t RSVD_REG_0;                               // offset:0x018C
    __IO uint32_t RSVD_REG_1;                               // offset:0x0190
    __IO uint32_t SLEEP_WAKE_CTRL;                          // offset:0x0194
    __IO uint32_t BLE_CLK_SWITCH;                           // offset:0x0198
    __IO uint32_t BLE_RECOVER_FLAG;                         // offset:0x019c
    __IO uint32_t BLE_TIMER_CALI;                           // offset:0x01a0
    __IO uint32_t OSC_INT_CTRL;                             // offset:0x01a4
         uint8_t  Reserved194_1ac[0x1ac - 0x1a4];
    __IO uint32_t DVDD_CHG_CTRL;                            // offset:0x1b0
    __IO uint32_t FUNC_FBD_CTRL;                            // offset:0x1b4
         uint8_t  Reserved1b4_1bc[0x1bc - 0x1b4];
    __IO uint32_t POF_INT_CTRL;                             // offset:0x1c0
    __IO uint32_t XTAL32M_HSEDET_CTRL;                      // offset:0x1c4
    __IO uint32_t REG_SW_CTRL_1;                            // offset:0x1c8
} CS_PMU_Type;

#endif  /* __PMU_REG_H */


/** @} */
