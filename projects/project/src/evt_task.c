/**
 * @file  examples/ble_app_simple_server/src/main.c
 * @brief  simple server
 * @date Wed, Sep  5, 2018  5:19:05 PM
 * @author liqiang
 *
 * @addtogroup APP_SIMPLE_SERVER_MAIN main.c
 * @ingroup APP_SIMPLE_SERVER
 * @details simple server
 *
 * @{
 */

/*********************************************************************
 * INCLUDES
 */
#include "cs.h"
#include "cs_driver.h"
#include "shell.h"
#include "evt.h"
#include "pm.h"
#include "board.h"
#include "mbr.h"
#include "nvds.h"

#include "sdk_config.h"
#include "cs_ble.h"

/* Kernel includes. */
#include "cmsis_os2.h"

/*********************************************************************
 * MACROS
 */

#define EVENT_BLUETOOTH_MASK      0x0001
#define EVENT_SYSTEM_RESERVE_MASK 0x00FF

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static osEventFlagsId_t xEvtEvent = NULL;
static evt_timer_t evt_timer_0;

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void app_adv_init();
void app_sec_init();
void service_common_init(void);
void app_chipsea_dfu_init(void);
void ancs_client_init(void);
void app_media_hid_init(void);
void app_24g_init(void);

/**
 *******************************************************************************
 * @brief  evt timer 0 handler
 *
 * @param[in] timer  timer
 * @param[in] param  param
 *******************************************************************************
 */
static void evt_timer_0_handler(evt_timer_t *timer, void *param)
{
    CS_LOG_DEBUG("evt timer: %08X\n", timer->time);
}

/**
 * @brief  bluetooth event handler
 **/
static void vEvtEventHandler(void)
{
    if (xEvtEvent)
        osEventFlagsSet(xEvtEvent, EVENT_BLUETOOTH_MASK);
}

/**
 * @brief  bluetooth schedule task
 *
 * @param[in] pvParameters  pv parameters
 **/
static void vEvtScheduleTask(void *argument)
{

    uint32_t status;
    uint32_t uxBits;
    struct cs_stack_param param = {
        .max_connection       = CS_LE_HOST_CONNECTION_NB,
        .max_ext_adv_set      = CS_LE_HOST_ADV_SET_NUM,
        .max_att_mtu          = CS_LE_HOST_ATT_MTU,
        .max_gatt_serv_num    = CS_LE_HOST_MAX_GATT_SERV_NUM,
        .max_gatt_write_cache = CS_LE_HOST_ATT_WRITE_CACHE_SIZE,
        .smp_sc_support       = CS_LE_HOST_SC_PAIRING,
    };

    nvds_init(0);              // 非易失性存储初始化
    status = ble_init(&param); // 蓝牙协议栈初始化
    if (status == 0) {
        CS_LOG_DEBUG("ble_init ok\n");
    } else {
        CS_LOG_DEBUG("ble_init error:%d\n", status);
    }
    app_adv_init();         // 初始化蓝牙广播
    app_sec_init();         // 初始化蓝牙安全功能
    service_common_init();  // 初始化通用服务
    app_chipsea_dfu_init(); // 初始化固件更新(DFU)
    ancs_client_init();     // 初始化苹果通知中心服务(ANCS)
    // app_media_hid_init();   // 初始化媒体和 HID 服务

    app_24g_init(); // 初始化2.4g私有协议
    uint32_t address, length;
    uint16_t crc16;
    mbr_read_part(PART_TYPE_APP, &address, &length, &crc16); // 读取应用分区信息​
    CS_LOG_DEBUG("running(%d): 0x%08X, len: %d, crc: 0x%04X\n", drv_pmu_reboot_reason(), address, length, crc16);

    // simple event timer
    evt_timer_set(&evt_timer_0, 2000, EVT_TIMER_REPEAT, evt_timer_0_handler, NULL);

    // enable sleep
    pm_sleep_enable(CFG_SLEEP_ENABLE); // 全局启用睡眠模式
    pm_sleep_allow(PM_ID_SHELL);       // PM_ID_SHELL 启用休眠

    // Create event
    xEvtEvent = osEventFlagsNew(NULL); // 创建事件组组(用于任务间通信)

    // 设置事件回调
    evt_schedule_trigger_callback_set(vEvtEventHandler);

    while (1) {
        evt_schedule(); // 执行任务调度

        // 任务挂起,等待信号唤醒
        uxBits = osEventFlagsWait(xEvtEvent, 0xFFFF, osFlagsWaitAny, osWaitForever);
        (void)uxBits;
    }
}

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/**
 * @brief  v start bluetooth task
 **/
void vStartEvtTask(void)
{
    const osThreadAttr_t bluetoothThreadAttr =
        {
            .name       = NULL,
            .attr_bits  = 0,
            .cb_mem     = NULL,
            .cb_size    = 0,
            .stack_mem  = NULL,
            .stack_size = 2048,
            .priority   = osPriorityRealtime,
            .tz_module  = 0,
        };

    // Create ble Task
    osThreadNew(vEvtScheduleTask, NULL, &bluetoothThreadAttr);
}

/** @} */

// vim: fdm=marker
