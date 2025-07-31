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
 * @file     cst92f41.h
 * @brief    CMSIS Core Peripheral Access Layer Header File for cst92f41
 * @date     20. Mar 2023
 * @author   chipsea
 *
 * @defgroup DOC DOC
 * @ingroup  DOCUMENT
 * @brief
 * @details
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __CST92F41_H
#define __CST92F41_H

#ifdef __cplusplus
extern "C"
{
#endif


/*********************************************************************
 * Interrupt Number Definition
 */
typedef enum IRQn
{
    /* Processor Exceptions Numbers */
    NonMaskableInt_IRQn         = -14,     /*  2 Non Maskable Interrupt */
    HardFault_IRQn              = -13,     /*  3 HardFault Interrupt */
    MemoryManagement_IRQn       = -12,     /*  4 Memory Management Interrupt */
    BusFault_IRQn               = -11,     /*  5 Bus Fault Interrupt */
    UsageFault_IRQn             = -10,     /*  6 Usage Fault Interrupt */
    SVCall_IRQn                 =  -5,     /* 11 SV Call Interrupt */
    DebugMonitor_IRQn           =  -4,     /* 12 Debug Monitor Interrupt */
    PendSV_IRQn                 =  -2,     /* 14 Pend SV Interrupt */
    SysTick_IRQn                =  -1,     /* 15 System Tick Interrupt */

    /* Processor Interrupt Numbers */
    BT_IRQn                     = 0,
    BT_WAKEUP_IRQn              = 1,
    DMA_IRQn                    = 2,
    PIN_WAKEUP_IRQn             = 3,
    TIM_IRQn                    = 4,
    CS_24G_RF_IRQn               = 5,
    LE_TIM_IRQn                 = 6,
    PMU_TIMER_IRQn              = 7,
    WDT_IRQn                    = 8,
    UART1_IRQn                  = 9,
    EFUSE_IRQn                  = 10,
    GPIO_IRQn                   = 11,
    ADC_IRQn                    = 12,
    I2C0_IRQn                   = 13,
    SF_IRQn                     = 14,
    SOFT0_IRQn                  = 15,
    SOFT1_IRQn                  = 16,
    SOFT2_IRQn                  = 17,
    SOFT3_IRQn                  = 18,
    SOFT4_IRQn                  = 19,
    SOFT5_IRQn                  = 20,
    SOFT6_IRQn                  = 21,
    SOFT7_IRQn                  = 22,
    VTRACK_IRQn                 = 23,
    CRY32M_RDY_IRQn             = 24,
    UART0_IRQn                  = 25,
    SPI0_IRQn                   = 26,
    CC_INTR_IRQn                = 27,
    TIM0_IRQn                   = 28,
    TIM1_IRQn                   = 29,
    TIM2_IRQn                   = 30,
    SPI1_IRQn                   = 31,
    RTC_1HZ_IRQn                = 32,
    RTC_AF_IRQn                 = 33,
    LP_TIMER_IRQn               = 34,

    /* Number of IRQ */
    EXTERNAL_IRQn_Num,
} IRQn_Type;


/* ================================================================================ */
/* ================      Processor and Core Peripheral Section     ================ */
/* ================================================================================ */

/* Start of section using anonymous unions and disabling warnings */
#if   defined (__CC_ARM)
    #pragma push
    #pragma anon_unions
#elif defined (__ICCARM__)
    #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
    #pragma clang diagnostic push
    #pragma clang diagnostic ignored "-Wc11-extensions"
    #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
    /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
    /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
    #pragma warning 586
#elif defined (__CSMC__)
    /* anonymous unions are enabled by default */
#else
    #warning Not supported compiler type
#endif


/* Configuration of Core Peripherals */
#define __CHECK_DEVICE_DEFINES    1
#define __CM4_REV                 0x0001    /*!< CM4 r0p1 */
#define __FPU_PRESENT             0U        /* FPU present */
#define __MPU_PRESENT             1U        /* MPU present */
#define __NVIC_PRIO_BITS          4U        /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U        /* Set to 1 if different SysTick Config is used */
#define __VTOR_PRESENT            1U        /* VTOR present */


/*********************************************************************
 * INCLUDES
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "core_cm4.h"

#include "../common/common_reg.h"
#include "../common/system_device.h"

#include "../chipsea/adc_reg.h"
#include "../chipsea/cpm_reg.h"
#include "../chipsea/pmu_reg.h"
#include "../chipsea/daif_reg.h"

#include "../common/gpio_reg.h"
#include "../common/i2c_reg.h"
#include "../common/spi_reg.h"
#include "../common/timer_reg.h"
#include "../common/uart_reg.h"
#include "../common/uart_ex_reg.h"
#include "../common/cs_24g_reg.h"
#include "../common/efuse_reg.h"
#include "../common/rtc_reg.h"
#include "../common/btphy_reg.h"
#include "../common/btmac_reg.h"
#include "../common/aes_hw_reg.h"
#include "../common/sf_reg.h"
#include "../common/cache_reg.h"
#include "../common/dma_reg.h"
#include "../common/sys_reg.h"
#include "../common/rng_reg.h"
#include "../common/lp_timer_reg.h"


/*********************************************************************
 * MACROS
 */
/* Memory base address */
#define CS_MEM_ROM_BASE             0x00000000U
#define CS_MEM_RAM_BASE             0x20000000U

/* System Peripheral Base address */
#define CS_SYS_BASE                 0x40000000U
#define CS_CPM_BASE                 0x40001000U
#define CS_PMU_BASE                 0x400e0000U
#define CS_DAIF_BASE                0x400a0000U
#define CS_RNG_BASE                 0x40004000U

/* Application Core Peripheral Base address */
#define CS_EFUSE_BASE               0x40002000U
#define CS_LP_TIM_BASE              0x40003000U
#define CS_SPI0_BASE                0x40005000U
#define CS_SPI1_BASE                0x40006000U
#define CS_I2C0_BASE                0x40009000U
#define CS_RF_2_4G_BASE             0x4000b000U
#define CS_RTC_BASE                 0x4000c000U
#define CS_USART1_BASE              0x40040000U
#define CS_USART0_BASE              0x40080000U
#define CS_TIM0_BASE                0x400c0000U
#define CS_TIM1_BASE                0x400c0100U
#define CS_TIM2_BASE                0x400c0200U
#define CS_GPIO_BASE                0x41200000U
#define CS_BB_BASE                  0x41300000U
#define CS_PHY_BASE                 0x40020000U
#define CS_BT_BASE                  0x41300000U
#define CS_AES_HW_BASE              0x41308000U
#define CS_SF_BASE                  0x51000000U
#define CS_ICACHE_BASE              0xE0042000U
#define CS_DMA_BASE                 0x41100000U
#define CS_GPADC_BASE               0x400A2000U

/* System Peripheral Type definition */
#define CS_SYS                      ((CS_SYS_Type *)        CS_SYS_BASE)
#define CS_CPM                      ((CS_CPM_Type *)        CS_CPM_BASE)
#define CS_PMU                      ((CS_PMU_Type *)        CS_PMU_BASE)
#define CS_DAIF                     ((CS_DAIF_Type *)       CS_DAIF_BASE)

/* Application Core Peripheral Type definition */
#define CS_EFUSE                    ((CS_EFUSE_Type *)      CS_EFUSE_BASE)
#define CS_LP_TIM                   ((CS_LP_TIM_Type *)     CS_LP_TIM_BASE)
#define CS_RNG                      ((CS_RNG_Type *)        CS_RNG_BASE)
#define CS_SPI0                     ((CS_SPI_Type *)        CS_SPI0_BASE)
#define CS_SPI1                     ((CS_SPI_Type *)        CS_SPI1_BASE)
#define CS_I2C0                     ((CS_I2C_Type *)        CS_I2C0_BASE)
#define CS_24G                      ((CS_24G_Type *)        CS_RF_2_4G_BASE)
#define CS_RTC                      ((CS_RTC_Type *)        CS_RTC_BASE)
#define CS_USART0                   ((CS_USART_EX_Type *)   CS_USART0_BASE)
#define CS_USART1                   ((CS_USART_Type *)      CS_USART1_BASE)
#define CS_TIM0                     ((CS_TIM_Type *)        CS_TIM0_BASE)
#define CS_TIM1                     ((CS_TIM_Type *)        CS_TIM1_BASE)
#define CS_TIM2                     ((CS_TIM_Type *)        CS_TIM2_BASE)
#define CS_GPIO0                    ((CS_GPIO_Type *)       CS_GPIO_BASE)
#define CS_PHY                      ((CS_BTPHY_Type *)      CS_PHY_BASE)
#define CS_BT                       ((CS_BTMAC_Type *)      CS_BT_BASE)
#define CS_AES                      ((CS_AES_HW_Type *)     CS_AES_HW_BASE)
#define CS_SF                       ((CS_SF_Type *)         CS_SF_BASE)
#define CS_ICACHE                   ((CS_ICACHE_Type *)     CS_ICACHE_BASE)
#define CS_DMA                      ((CS_DMA_Type *)        CS_DMA_BASE)
#define CS_GPADC                    ((CS_GPADC_Type *)      CS_GPADC_BASE)

/* peripheral capabilities define */
#define CAP_USART0        ( (0U << CAP_USART_DMA_TX_POS)                       \
                          | (0U << CAP_USART_DMA_RX_POS)                       \
                          | (0U << CAP_USART_CTS_RTS_FLOW_CONTROL_POS)         \
                          | (0U << CAP_USART_FIFO_LEVEL_POS))

#define CAP_USART1        ( (1U << CAP_USART_DMA_TX_POS)                       \
                          | (1U << CAP_USART_DMA_RX_POS)                       \
                          | (1U << CAP_USART_CTS_RTS_FLOW_CONTROL_POS)         \
                          | (16U << CAP_USART_FIFO_LEVEL_POS))

#define CAP_SPI0          ( (1U << CAP_SPI_MASTER_MODE_POS)                    \
                          | (1U << CAP_SPI_SLAVE_MODE_POS)                     \
                          | (1U << CAP_SPI_GPDMA_TX_POS)                       \
                          | (1U << CAP_SPI_GPDMA_RX_POS)                       \
                          | (0x20U << CAP_SPI_FIFO_LEVEL_POS))

#define CAP_SPI1          ( (1U << CAP_SPI_MASTER_MODE_POS)                    \
                          | (1U << CAP_SPI_SLAVE_MODE_POS)                     \
                          | (1U << CAP_SPI_GPDMA_TX_POS)                       \
                          | (1U << CAP_SPI_GPDMA_RX_POS)                       \
                          | (0x20U << CAP_SPI_FIFO_LEVEL_POS))

#define CAP_I2C0          ( (1U << CAP_I2C_MASTER_MODE_POS)                    \
                          | (0U << CAP_I2C_SLAVE_MODE_POS)                     \
                          | (1U << CAP_I2C_DMA_TX_POS)                         \
                          | (1U << CAP_I2C_DMA_RX_POS)                         \
                          | (0x10U << CAP_I2C_FIFO_LEVEL_POS))

#define CAP_GPIO0         ( (0U << CAP_GPIO_PIN_NUM_MIN_POS)                   \
                          | (25U << CAP_GPIO_PIN_NUM_MAX_POS))

#define CAP_TIM0          ( (1U << CAP_TIM_CAPTURE_POS)                        \
                          | (1U << CAP_TIM_PWM_POS)                            \
                          | (1U << CAP_TIM_BDT_POS)                            \
                          | (1U << CAP_TIM_DMA_POS))

#define CAP_TIM1          ( (0U << CAP_TIM_CAPTURE_POS)                        \
                          | (1U << CAP_TIM_PWM_POS)                            \
                          | (1U << CAP_TIM_BDT_POS)                            \
                          | (1U << CAP_TIM_DMA_POS))

#define CAP_TIM2          ( (0U << CAP_TIM_CAPTURE_POS)                        \
                          | (1U << CAP_TIM_PWM_POS)                            \
                          | (1U << CAP_TIM_BDT_POS)                            \
                          | (1U << CAP_TIM_DMA_POS))

/* End of section using anonymous unions and disabling warnings */
#if   defined (__CC_ARM)
  #pragma pop
#elif defined (__ICCARM__)
  /* leave anonymous unions enabled */
#elif (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
  #pragma clang diagnostic pop
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#elif defined (__TMS470__)
  /* anonymous unions are enabled by default */
#elif defined (__TASKING__)
  #pragma warning restore
#elif defined (__CSMC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif


#ifdef __cplusplus
}
#endif

#endif  /* __cst92f41_H */


/** @} */
