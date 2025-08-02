#include "cs.h"
#include "cs_driver.h"
#include "shell.h"
#include "evt.h"
#include "pm.h"
#include "board.h"
#include "mbr.h"
#include "sdk_config.h"
#include "cs_ble.h"

/* Kernel includes. */
#include "cmsis_os2.h"

void vStartEvtTask(void);

static void drv_gpio_isr_handler(void *cs_reg, drv_event_t event, void *int_status, void *data)
{
    CS_LOG_DEBUG("gpio: 0x%08X\n", (uint32_t)int_status);
}

static void drv_pin_wakeup_isr_handler(void *cs_reg, drv_event_t event, void *int_status, void *data)
{
    CS_LOG_DEBUG("pinwakeup: 0x%08X\n", (uint32_t)int_status);
}

static void pm_sleep_notify_handler(pm_sleep_state_t sleep_state, pm_status_t power_status)
{
    switch (sleep_state) {
        case PM_SLEEP_ENTRY:
            // CS_LOG_DEBUG("> %d\n", power_status);
            break;

        case PM_SLEEP_LEAVE_TOP_HALF:
            break;

        case PM_SLEEP_LEAVE_BOTTOM_HALF:
            // CS_LOG_DEBUG("< %d\n", power_status);
            break;

        default:
            break;
    }
}

static void hardware_init(void)
{
    // 注册GPIO中断回调函数(任何GPIO类的中断都会触发回调)
    drv_gpio_register_isr_callback(CS_GPIO0, drv_gpio_isr_handler);
    // 设置唤醒引脚(低电平唤醒)
    drv_pmu_wakeup_pin_set(BITMASK(PAD_BUTTON_0) | BITMASK(PAD_BUTTON_1), PMU_PIN_WAKEUP_LOW_LEVEL);
    // 注册唤醒引脚中断回调
    drv_pmu_wakeup_pin_register_callback(drv_pin_wakeup_isr_handler);
    // 注册睡眠状态变化回调
    pm_sleep_notify_user_callback_register(pm_sleep_notify_handler);
}

// 定时器回调函数
static void vTimerCallback(void *argument)
{
    drv_gpio_toggle(CS_GPIO0, BITMASK(PAD_LED_0));
}

// 测试任务
static void vTestTask(void *argument)
{
    osTimerId_t xTimer;

    xTimer = osTimerNew(vTimerCallback, osTimerPeriodic, NULL, NULL);

    osTimerStart(xTimer, 125 * osKernelGetTickFreq() / 1000);

    for (;;) {
        //        drv_gpio_toggle(CS_GPIO0, BIT_MASK(PAD_LED_3));
        CS_LOG_DEBUG("vTestTask delay 500ms, TickFreq=%dHz\n", osKernelGetTickFreq());
        osDelay(500 * osKernelGetTickFreq() / 1000);
    }
}

/**
 * @brief  v start test task
 **/
static void vStartTestTask(void)
{
    // Create ble Task
    osThreadNew(vTestTask, NULL, NULL);
}

int main(void)
{
    drv_wdt_init(0);  // 看门狗初始化
    board_init();     // 板极初始化
    hardware_init();  // 硬件初始化
    drv_rf_init();    // 射频初始化
    evt_init();       // 事件系统初始化
    evt_timer_init(); // 事件定时器初始化
    shell_init(NULL); // 命令行shell初始化(暂用于调试)

    osKernelInitialize(); // 初始化 RTOS 内核

    vStartEvtTask();  // 启动事件任务
    vStartTestTask(); // 启动测试任务

    // Start thread execution
    if (osKernelGetState() == osKernelReady)
        osKernelStart(); // 启动 RTOS 调度器

    for (;;) // 主循环(实际不会执行到这里)
        ;
}
