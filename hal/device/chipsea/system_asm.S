;******************************************************************************
;
;  Copyright (C), 2024, Chipsea Co., Ltd.
;
;******************************************************************************
; * @file     system_asm.S
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

__SYSTEM_CPU_STATUS     EQU     0x400E00F0
__SYSTEM_PMU_STATE_REG  EQU     0X400E0000
__SYSTEM_CPU_SCR        EQU     0xE000ED10

    AREA    |.rodata|, DATA, READONLY, ALIGN=2
    THUMB

__SYSTEM_REG_SAVE_ADDR
    DCD     0xE000E100
    DCD     2
    DCD     0xE000E400
    DCD     9
    DCD     0xE000ED08
    DCD     0xE000ED88
__SYSTEM_REG_SAVE_ADDR_END


    AREA    |.data|, DATA, READWRITE, ALIGN=2
    THUMB

__SYSTEM_CPU_REG_SAVE_ADDR
    DCD     0
    DCD     0
    DCD     0
    DCD     0
__SYSTEM_CPU_REG_SAVE_ADDR_END


    AREA    |.text|, CODE, READONLY, ALIGN=2
    THUMB

    EXPORT  SystemEnterDeepSleep
    EXPORT  SystemExitDeepSleep
    EXPORT  SystemFromRomExitDeepSleepEnable
    EXPORT  SystemRunTo

SystemEnterDeepSleep    PROC
                        PUSH {R0-R12, LR}
                        LDR R4, =__SYSTEM_REG_SAVE_ADDR
                        LDR R6, =__SYSTEM_REG_SAVE_ADDR_END
__SYSTEM_REG_SAVE_LOOP_0
                        CMP R4, R6
                        BCC __SYSTEM_REG_SAVE_LOOP_1
                        B __SYSTEM_REG_SAVE_LOOP_END
__SYSTEM_REG_SAVE_LOOP_1
                        MOV R2, R4
                        LDR R7, [R2], #4
                        CMP R2, R6
                        BCS __SYSTEM_REG_SAVE_LOOP_4
                        LDR R5, [R4, #4]
                        CMP R5, #4096
                        BCS __SYSTEM_REG_SAVE_LOOP_4
                        MOV R4, R2
__SYSTEM_REG_SAVE_LOOP_2
                        ADD R5, R7, R5, LSL #2
__SYSTEM_REG_SAVE_LOOP_3
                        CMP R5, R7
                        BNE __SYSTEM_REG_SAVE_LOOP_5
                        ADDS R4, #4
                        B __SYSTEM_REG_SAVE_LOOP_0
__SYSTEM_REG_SAVE_LOOP_4
                        MOVS R5, #1
                        B __SYSTEM_REG_SAVE_LOOP_2
__SYSTEM_REG_SAVE_LOOP_5
                        LDR R0, [R7], #4
                        PUSH {R0}
                        B __SYSTEM_REG_SAVE_LOOP_3
__SYSTEM_REG_SAVE_LOOP_END

                        LDR R0, =__SYSTEM_CPU_REG_SAVE_ADDR

                        MRS R1, CONTROL
                        STR R1, [R0, #0]

                        MRS R1, MSP
                        STR R1, [R0, #4]

                        MRS R1, PSP
                        STR R1, [R0, #8]

                        LDR	R0, =__SYSTEM_CPU_STATUS
                        LDR R1, [R0]
                        ORR R1, R1, #0x80000000
                        STR R1, [R0]

                        LDR	R0, =__SYSTEM_CPU_SCR
                        MOV R1, #4
                        STR R1, [R0]

                        WFI

                        LDR R0, =__SYSTEM_PMU_STATE_REG
__SYSTEM_REG_PMU_STATE_CHECK
                        LDR R1, [R0]
                        LSR R1, R1, #27
                        CMP R1, #7
                        BNE __SYSTEM_REG_PMU_STATE_CHECK

__SYSTEM_CPU_SUSPEND_RESUME
                        CPSID I

                        LDR	R0, =__SYSTEM_CPU_SCR
                        MOV R1, #0
                        STR R1, [R0]

                        LDR R0, =__SYSTEM_CPU_REG_SAVE_ADDR

                        LDR R1, [R0, #0]
                        MSR CONTROL, R1

                        LDR R1, [R0, #4]
                        MSR MSP, R1

                        LDR R1, [R0, #8]
                        MSR PSP, R1

                        LDR R0, =__SYSTEM_CPU_STATUS
                        LDR R1, [R0]

                        BIC R1, R1, #0x80000000
                        STR R1, [R0]

                        LDR R6, =__SYSTEM_REG_SAVE_ADDR
                        LDR R4, =__SYSTEM_REG_SAVE_ADDR_END
                        SUBS R4, R4, #4
__SYSTEM_REG_RESTORE_LOOP_0
                        CMP R4, R6
                        BCS __SYSTEM_REG_RESTORE_LOOP_1
                        B __SYSTEM_REG_RESTORE_LOOP_END
__SYSTEM_REG_RESTORE_LOOP_1
                        LDR R5, [R4, #0]
                        CMP R5, #4096
                        ITE CC
                        SUBCC R4, #4
                        MOVCS R5, #1
                        LDR R7, [R4, #0]
__SYSTEM_REG_RESTORE_LOOP_2
                        SUBS R5, #1
                        BCS __SYSTEM_REG_RESTORE_LOOP_3
                        SUBS R4, #4
                        B __SYSTEM_REG_RESTORE_LOOP_0
__SYSTEM_REG_RESTORE_LOOP_3
                        ADD R0, R7, R5, LSL #2
                        POP {R1}
                        STR R1, [R0]
                        B __SYSTEM_REG_RESTORE_LOOP_2
__SYSTEM_REG_RESTORE_LOOP_END

                        POP {R0-R12, PC}
                        ALIGN
                        ENDP


SystemExitDeepSleep     PROC
                        LDR R0, =__SYSTEM_CPU_STATUS
                        LDR R1, [R0]
                        CMP	R1, #0

                        BGE __SYSTEM_CPU_POWER_UP_RESET

                        LSLS R0, R1, #1
                        BPL __SYSTEM_CPU_SUSPEND_RESUME

                        BIC R0, R1, #0xc0000000
                        BX R0
                        B .

                        ALIGN
                        ENDP

__SYSTEM_CPU_POWER_UP_RESET      PROC
                                 BX LR

                                 ALIGN
                                 ENDP

SystemFromRomExitDeepSleepEnable PROC
                                 LDR R1, =__SYSTEM_CPU_SUSPEND_RESUME
                                 ORR R1, R1, #0x40000000
                                 ORR R1, R1, #0x00000001
                                 LDR	R0, =__SYSTEM_CPU_STATUS
                                 STR R1, [R0]
                                 BX LR

                                 ALIGN
                                 ENDP

SystemRunTo             PROC
                        CPSID I
                        LDR R1, [R0, #0]
                        LDR R2, [R0, #4]
                        MSR MSP, R1
                        BX  R2
                        B .

                        ALIGN
                        ENDP

                        ALIGN
                        END

