;******************************************************************************
;
;  Copyright (C), 2024, Chipsea Co., Ltd.
;
;******************************************************************************
; * @file     startup.s
; * @brief    CMSIS Cortex-M4 Core Device Startup File for
; *           Device CST92F41
; * @version  V1.00
; * @date     11. Sep 2024
; *
; * @note
; * Copyright (C) 2024 ARM Limited. All rights reserved.
; *
; * @par
; * ARM Limited (ARM) is supplying this software for use with Cortex-M
; * processor based microcontrollers.  This file can be freely distributed
; * within development tools that are supporting such ARM based processors.
; *
; * @par
; * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
; * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
; * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
; * ARM SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR
; * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
; *
; ******************************************************************************/

                        PRESERVE8
                        AREA    RESET, CODE, READONLY, ALIGN=2
                        THUMB

                        IMPORT  ||Image$$ARM_LIB_STACK$$ZI$$Limit||
                        IMPORT  NMI_Handler
                        IMPORT  HardFault_Handler
                        IMPORT  MemManage_Handler
                        IMPORT  BusFault_Handler
                        IMPORT  UsageFault_Handler
                        IMPORT  SoftFault_Handler
                        IMPORT  SVC_Handler
                        IMPORT  DebugMon_Handler
                        IMPORT  PendSV_Handler
                        IMPORT  SysTick_Handler
                        IMPORT  BT_IRQHandler
                        IMPORT  BT_WAKEUP_IRQHandler
                        IMPORT  DMA_IRQHandler
                        IMPORT  PIN_WAKEUP_IRQHandler
                        IMPORT  TIM_IRQHandler
                        IMPORT  CS_24G_RF_IRQHandler
                        IMPORT  PMU_POF_IRQHandler
                        IMPORT  PMU_TIMER_IRQHandler
                        IMPORT  WDT_IRQHandler
                        IMPORT  UART1_IRQHandler
                        IMPORT  EFUSE_IRQHandler
                        IMPORT  GPIO_IRQHandler
                        IMPORT  ADC_IRQHandler
                        IMPORT  I2C0_IRQHandler
                        IMPORT  SF_IRQHandler
                        IMPORT  SOFT0_IRQHandler
                        IMPORT  SOFT1_IRQHandler
                        IMPORT  SOFT2_IRQHandler
                        IMPORT  SOFT3_IRQHandler
                        IMPORT  SOFT4_IRQHandler
                        IMPORT  SOFT5_IRQHandler
                        IMPORT  SOFT6_IRQHandler
                        IMPORT  SOFT7_IRQHandler
                        IMPORT  VTRACK_IRQHandler
                        IMPORT  CRY32M_RDY_IRQHandler
                        IMPORT  UART0_IRQHandler
                        IMPORT  SPI0_IRQHandler
                        IMPORT  CC_INTR_IRQHandler
                        IMPORT  TIM0_IRQHandler
                        IMPORT  TIM1_IRQHandler
                        IMPORT  TIM2_IRQHandler
                        IMPORT  SPI1_IRQHandler
                        IMPORT  RTC_1HZ_IRQHandler
                        IMPORT  RTC_AF_IRQHandler
                        IMPORT  LP_TIMER_IRQHandler

                        EXPORT  __Vectors
                        EXPORT  __Vectors_End
                        EXPORT  __Vectors_Size

__Vectors
                        DCD     ||Image$$ARM_LIB_STACK$$ZI$$Limit||     ; Top of stack
                        DCD     Reset_Handler                           ; -15.
                        DCD     NMI_Handler                             ; -14
                        DCD     HardFault_Handler                       ; -13
                        DCD     MemManage_Handler                       ; -12
                        DCD     BusFault_Handler                        ; -11
                        DCD     UsageFault_Handler                      ; -10
                        DCD     SoftFault_Handler                       ; -9
                        DCD     0xFFFFFFFF                              ; -8
                        DCD     __Vectors                               ; -7. Base Address.
                        DCD     0x46533252                              ; -6. RUN2SF_MAGIC_FLAG.
                        DCD     SVC_Handler                             ; -5.
                        DCD     DebugMon_Handler                        ; -4.
                        DCD     0x6626a100                              ; -3. ROM ID.
                        DCD     PendSV_Handler                          ; -2.
                        DCD     SysTick_Handler                         ; -1.

                        ; Interrupts
                        DCD     BT_IRQHandler                           ; 0
                        DCD     BT_WAKEUP_IRQHandler                    ; 1
                        DCD     DMA_IRQHandler                          ; 2
                        DCD     PIN_WAKEUP_IRQHandler                   ; 3
                        DCD     TIM_IRQHandler                          ; 4
                        DCD     CS_24G_RF_IRQHandler                    ; 5
                        DCD     PMU_POF_IRQHandler                      ; 6
                        DCD     PMU_TIMER_IRQHandler                    ; 7
                        DCD     WDT_IRQHandler                          ; 8
                        DCD     UART1_IRQHandler                        ; 9
                        DCD     EFUSE_IRQHandler                        ; 10
                        DCD     GPIO_IRQHandler                         ; 11
                        DCD     ADC_IRQHandler                          ; 12
                        DCD     I2C0_IRQHandler                         ; 13
                        DCD     SF_IRQHandler                           ; 14
                        DCD     SOFT0_IRQHandler                        ; 15
                        DCD     SOFT1_IRQHandler                        ; 16
                        DCD     SOFT2_IRQHandler                        ; 17
                        DCD     SOFT3_IRQHandler                        ; 18
                        DCD     SOFT4_IRQHandler                        ; 19
                        DCD     SOFT5_IRQHandler                        ; 20
                        DCD     SOFT6_IRQHandler                        ; 21
                        DCD     SOFT7_IRQHandler                        ; 22
                        DCD     VTRACK_IRQHandler                       ; 23
                        DCD     CRY32M_RDY_IRQHandler                   ; 24
                        DCD     UART0_IRQHandler                        ; 25
                        DCD     SPI0_IRQHandler                         ; 26
                        DCD     CC_INTR_IRQHandler                      ; 27
                        DCD     TIM0_IRQHandler                         ; 28
                        DCD     TIM1_IRQHandler                         ; 29
                        DCD     TIM2_IRQHandler                         ; 30
                        DCD     SPI1_IRQHandler                         ; 31
                        DCD     RTC_1HZ_IRQHandler                      ; 32
                        DCD     RTC_AF_IRQHandler                       ; 33
                        DCD     LP_TIMER_IRQHandler                     ; 34

__Vectors_End
__Vectors_Size          EQU     __Vectors_End - __Vectors


                        AREA    |.text|, CODE, READONLY, ALIGN=2
                        THUMB

; Reset Handler
Reset_Handler           PROC
                        EXPORT  Reset_Handler
                        IMPORT  SystemInit
                        IMPORT  __main

                        CPSID   I
                        BL      SystemInit
                        LDR     R0, =__main
                        BX      R0

                        ALIGN
                        ENDP

; Sub main (from __main)
|$Sub$$main|            PROC
                        EXPORT  |$Sub$$main|
                        IMPORT  SystemInitPost
                        IMPORT  |$Super$$main|

                        BL      SystemInitPost
                        LDR     R0, =|$Super$$main|
                        BX      R0
                        ENDP

                        ALIGN
                        END


