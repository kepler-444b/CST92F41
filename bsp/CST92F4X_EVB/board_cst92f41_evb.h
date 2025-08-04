/**
 * @file board.h
 * @brief
 * @date Tue 31 Jan 2023 09:35:08 AM CST
 * @author chipsea
 *
 * @defgroup
 * @ingroup
 * @brief
 * @details
 *
 * @{
 */

#ifndef __BOARD_H__
#define __BOARD_H__

#ifdef __cplusplus
extern "C" { /*}*/
#endif

/*********************************************************************
 * INCLUDES
 */
#include "cs_driver.h"

/*********************************************************************
 * MACROS
 */

#define LED_OFF_LEVEL GPIO_LEVEL_HIGH

// IO7-IO12 used as xflash

#if 1
#define PAD_UART0_TXD 3
#define PAD_UART0_RXD 8
#define PAD_UART1_TXD 5
#define PAD_UART1_RXD 6
#else
#define PAD_UART0_TXD 22
#define PAD_UART0_RXD 21
#define PAD_UART1_TXD 5
#define PAD_UART1_RXD 6
#endif

#define PAD_LED_0    17 /// EVB

#define PAD_BUTTON_0 4 /// EVB
#define PAD_BUTTON_1 4 /// EVB

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * EXTERN VARIABLES
 */

/*********************************************************************
 * EXTERN FUNCTIONS
 */

#ifdef __cplusplus
/*{*/ }
#endif

#endif

/** @} */
