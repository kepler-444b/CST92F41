/**
 * @file     sdk_config.h
 * @brief    config the app
 * @date     18 September 2024
 * @author   Chipsea Ble Group
 *
 * @version
 * Version 1.0
 *  - Initial release
 * @{
 */

#ifndef SDK_CONFIG_H
#define SDK_CONFIG_H

#include "features.h"

// <<< Use Configuration Wizard in Context Menu >>>\n

// <h> General parameter config

// <s.18> DEFAULT_DEVICE_NAME  - Config Default Device Name
#ifndef DEFAULT_DEVICE_NAME
#define DEFAULT_DEVICE_NAME "CST92F41-FREERTOS"
#endif

// <e> CFG_SLEEP_ENABLE - Enable sleep mode
// <i> 0: full speed to run,  1: run-sleep-run-sleep
#ifndef CFG_SLEEP_ENABLE
#define CFG_SLEEP_ENABLE 1
#endif

// </e>

// <o> CFG_DCDC_ENABLE  - Config Buck DCDC enable
// <i> CFG_DCDC_ENABLE: DCDC_DISABLE or DCDC_ENABLE
// <0=> DCDC_DISABLE
// <1=> DCDC_ENABLE
#ifndef CFG_DCDC_ENABLE
#define CFG_DCDC_ENABLE 1
#endif

// <o> CFG_RESET_PIN_ENABLE  - Config reset pin enable
// <i> CFG_RESET_PIN_ENABLE: RESET_PIN_DISABLE or RESET_PIN_ENABLE
// <0=> RESET_PIN_DISABLE
// <1=> RESET_PIN_ENABLE
#ifndef CFG_RESET_PIN_ENABLE
#define CFG_RESET_PIN_ENABLE 1
#endif

// <o> DEFAULT_SYS_CLK_SRC  - Config default system clock source
// <i> SYS_CLK_XTAL_DBL64M:XTAL DBL 64MHz, SYS_CLK_XTAL_32M: XTAL 32MHz
// <0=> SYS_CLK_XTAL_DBL64M
// <1=> SYS_CLK_XTAL_32M
// <2=> SYS_CLK_XTAL_DIV16M

// 外部 3MHz 晶振,不倍频
#ifndef DEFAULT_SYS_CLK_SRC
#define DEFAULT_SYS_CLK_SRC 1
#endif
#if (DEFAULT_SYS_CLK_SRC == 0) // SYS_CLK_XTAL_DBL64M:XTAL DBL 64MHz
#define CFG_CLK_DBL_ENABLE 1
#define CFG_CPM_CLK_VALUE 32000000
#elif (DEFAULT_SYS_CLK_SRC == 1) // SYS_CLK_XTAL_32M: XTAL 32MHz
#define CFG_CLK_DBL_ENABLE 0
#define CFG_CPM_CLK_VALUE 32000000
#elif (DEFAULT_SYS_CLK_SRC == 2) // SYS_CLK_XTAL_DIV16M: XTAL DIV to 16MHz
#define CFG_CLK_DBL_ENABLE 0
#define CFG_CPM_CLK_VALUE 16000000
#endif

// <o> DEFAULT_32K_CLK_SRC  - Config default 32KHz clock source
// <i> PMU_32K_SEL_RC: use internal RC oscillator. PMU_32K_SEL_DIV: use divided clock from external 32M crystal
// <0=> PMU_32K_SEL_RC
// <1=> PMU_32K_SEL_32768HZ_XTAL
// <2=> PMU_32K_SEL_DIV

// 使用内部 RC 振荡器
#ifndef DEFAULT_32K_CLK_SRC
#define DEFAULT_32K_CLK_SRC 0
#endif
#if (DEFAULT_32K_CLK_SRC == 0) // PMU_32K_SEL_RC
#define CFG_PMU_32K_SEL PMU_32K_SEL_RC
#elif (DEFAULT_32K_CLK_SRC == 1) // PMU_32K_SEL_32768HZ_XTAL
#define CFG_PMU_32K_SEL PMU_32K_SEL_32768HZ_XTAL
#elif (DEFAULT_32K_CLK_SRC == 2) // PMU_32K_SEL_DIV
#define CFG_PMU_32K_SEL PMU_32K_SEL_DIV
#else
#define CFG_PMU_32K_SEL PMU_32K_SEL_RC
#warning Don't support current DEFAULT_32K_CLK_SRC value, use PMU_32K_SEL_RC configuration
#endif

// </h>

// <h> BLE parameter config

/********************************************************************
 * Define APP config
 *
 * You can define CS_LE_XXX here.
 */

/********************************************************************
 * Use the default config
 *
 * If CS_LE_XXX not defined, use default.
 */

#if !defined(CS_LE_BROADCASTER)
/// Broadcaster
#if (defined(CONFIG_LE_BROADCASTER) || defined(CONFIG_LE_PERIPHERAL))
#define CS_LE_BROADCASTER 1
#else
#define CS_LE_BROADCASTER 0
#endif
#endif // CS_LE_BROADCASTER

#if !defined(CS_LE_OBSERVER)
/// Observer
#if (defined(CONFIG_LE_OBSERVER) || defined(CONFIG_LE_CENTRAL))
#define CS_LE_OBSERVER 1
#else
#define CS_LE_OBSERVER 0
#endif
#endif // CS_LE_OBSERVER

#if !defined(CS_LE_CENTRAL)
/// Central
#if (defined(CONFIG_LE_CENTRAL))
#define CS_LE_CENTRAL 1
#else
#define CS_LE_CENTRAL 0
#endif
#endif // CS_LE_CENTRAL

#if !defined(CS_LE_PERIPHERAL)
/// Peripheral
#if (defined(CONFIG_LE_PERIPHERAL))
#define CS_LE_PERIPHERAL 1
#else
#define CS_LE_PERIPHERAL 0
#endif
#endif // CS_LE_PERIPHERAL

#if !defined(CS_LE_LL_SEC_CON)
/// LL ECC
#if defined(CONFIG_LE_LL_SEC_CON)
#define CS_LE_LL_SEC_CON 1
#else
#define CS_LE_LL_SEC_CON 0
#endif
#endif // CS_LE_LL_SEC_CON

#if !defined(CS_LE_MAX_NB_ADV_REP_FRAG)
/// Maximum number of ADV reports in the HCI queue to Host
#define CS_LE_MAX_NB_ADV_REP_FRAG 2
#endif // CS_LE_MAX_NB_ADV_REP_FRAG

#if !defined(CS_LE_HOST_ATT_MTU)
/// Host ATT MTU
#if defined(CONFIG_LE_HOST_ATT_MTU)
#define CS_LE_HOST_ATT_MTU CONFIG_LE_HOST_ATT_MTU
#else
#define CS_LE_HOST_ATT_MTU 247
#endif
#endif // CS_LE_HOST_ATT_MTU

#if !defined(CS_LE_HOST_ATT_WRITE_CACHE_SIZE)
/// Host ATT prepare write cache size
#if defined(CONFIG_LE_HOST_ATT_WRITE_CACHE_SIZE)
#define CS_LE_HOST_ATT_WRITE_CACHE_SIZE CONFIG_LE_HOST_ATT_WRITE_CACHE_SIZE
#else
#define CS_LE_HOST_ATT_WRITE_CACHE_SIZE 23
#endif
#endif // CS_LE_HOST_ATT_WRITE_CACHE_SIZE

#if !defined(CS_LE_HOST_MAX_GATT_SERV_NUM)
/// Host GATT max service number
#if defined(CONFIG_LE_HOST_MAX_GATT_SERV_NUM)
#define CS_LE_HOST_MAX_GATT_SERV_NUM CONFIG_LE_HOST_MAX_GATT_SERV_NUM
#else
#define CS_LE_HOST_MAX_GATT_SERV_NUM 8
#endif
#endif // CS_LE_HOST_MAX_GATT_SERV_NUM

#if !defined(CS_LE_HOST_SC_PAIRING)
/// Host support secure connection pairing
#if defined(CONFIG_LE_HOST_SC_PAIRING)
#define CS_LE_HOST_SC_PAIRING 1
#else
#define CS_LE_HOST_SC_PAIRING 0
#endif
#endif // CS_LE_HOST_SC_PAIRING

#if !defined(CS_LE_HOST_ADV_SET_NUM)
/// Host max advertise number
#if defined(CONFIG_LE_HOST_ADV_SET_NUM)
#define CS_LE_HOST_ADV_SET_NUM CONFIG_LE_HOST_ADV_SET_NUM
#else
#define CS_LE_HOST_ADV_SET_NUM 4
#endif
#endif // CS_LE_HOST_ADV_SET_NUM

#if !defined(CS_LE_HOST_CONNECTION_NB)
/// Host max connection number
#if defined(CONFIG_LE_HOST_CONNECTION_NB)
#define CS_LE_HOST_CONNECTION_NB CONFIG_LE_HOST_CONNECTION_NB
#else
#define CS_LE_HOST_CONNECTION_NB 8
#endif
#endif // CS_LE_HOST_CONNECTION_NB

#if !defined(CS_LE_HOST_MSG_SIZE)
/// Host reserved message size
#if defined(CONFIG_LE_HOST_MSG_SIZE)
#define CS_LE_HOST_MSG_SIZE CONFIG_LE_HOST_MSG_SIZE
#else
#define CS_LE_HOST_MSG_SIZE 512
#endif
#endif // CS_LE_HOST_MSG_SIZE

// </h>

#endif // SDK_CONFIG_H

/** @} */
