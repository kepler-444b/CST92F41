
.syntax unified
.section .text

.extern chipsea_fault
.global HardFault_Handler
.global MemManage_Handler
.global BusFault_Handler
.global UsageFault_Handler
.global SoftFault_Handler

/***
 *NOTE: If use this file's HardFault_Handler, please comments the HardFault_Handler code on other file.
 */

.type HardFault_Handler, %function
HardFault_Handler:
    MOV     r0, lr                  // get lr
    MOV     r1, sp                  // get stack pointer (current is MSP)
    MOV     r2, #0
    BL      chipsea_fault

.type MemManage_Handler, %function
MemManage_Handler:
    MOV     r0, lr                  // get lr
    MOV     r1, sp                  // get stack pointer (current is MSP)
    MOV     r2, #1
    BL      chipsea_fault

.type BusFault_Handler, %function
BusFault_Handler:
    MOV     r0, lr                  // get lr
    MOV     r1, sp                  // get stack pointer (current is MSP)
    MOV     r2, #2
    BL      chipsea_fault

.type UsageFault_Handler, %function
UsageFault_Handler:
    MOV     r0, lr                  // get lr
    MOV     r1, sp                  // get stack pointer (current is MSP)
    MOV     r2, #3
    BL      chipsea_fault

.type SoftFault_Handler, %function
SoftFault_Handler:
    MOV     r0, lr                  // get lr
    MOV     r1, sp                  // get stack pointer (current is MSP)
    MOV     r2, #4
    BL      chipsea_fault
