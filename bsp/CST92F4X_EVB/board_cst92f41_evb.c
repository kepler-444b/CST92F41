#include "RTE_cst92f4x.h"
#include "cs_driver.h"
#include "board_cst92f41_evb.h"
#include "sdk_config.h"
#ifdef AT_ENABLE
#include "uart_process.h"
#endif

#define BOARD_32K_SELECT         CFG_PMU_32K_SEL               // 使用内部 RC 振荡器
#define BOARD_PIN_DRIVER_CURRENT PMU_PIN_DRIVER_CURRENT_NORMAL // 引脚驱动电流"正常"

static const pin_config_t pin_config[] = {
#if (1 && RTE_USART0) // 调试串口
    {PAD_UART0_TXD, {PINMUX_UART0_TX_CFG}, PMU_PIN_MODE_PP, BOARD_PIN_DRIVER_CURRENT},
    {PAD_UART0_RXD, {PINMUX_UART0_RX_CFG}, PMU_PIN_MODE_PU, BOARD_PIN_DRIVER_CURRENT},
#endif

#if (1 && RTE_USART1)
    {PAD_UART1_TXD, {PINMUX_UART1_TX_CFG}, PMU_PIN_MODE_PP, BOARD_PIN_DRIVER_CURRENT},
    {PAD_UART1_RXD, {PINMUX_UART1_RX_CFG}, PMU_PIN_MODE_PU, BOARD_PIN_DRIVER_CURRENT},
#endif

#if (defined(PAD_RF_TXEN) && defined(PAD_RF_RXEN))
    {PAD_RF_RXEN, {PINMUX_PAD3_RX_EXT_PD_CFG}, PMU_PIN_MODE_PP, BOARD_PIN_DRIVER_CURRENT},
    {PAD_RF_TXEN, {PINMUX_PAD4_TX_EXT_PD_CFG}, PMU_PIN_MODE_PP, BOARD_PIN_DRIVER_CURRENT},
#endif

    {PAD_LED_0, {PINMUX_GPIO_MODE_CFG}, PMU_PIN_MODE_PP, BOARD_PIN_DRIVER_CURRENT},

    {PAD_BUTTON_0, {PINMUX_GPIO_MODE_CFG}, PMU_PIN_MODE_PU, BOARD_PIN_DRIVER_CURRENT},
    {PAD_BUTTON_1, {PINMUX_GPIO_MODE_CFG}, PMU_PIN_MODE_PU, BOARD_PIN_DRIVER_CURRENT},
#ifdef AT_ENABLE
    {PIN_AT_WAKEUP, {PINMUX_GPIO_MODE_CFG}, PMU_PIN_MODE_PU, BOARD_PIN_DRIVER_CURRENT},
#endif
};

static const gpio_config_t gpio_config[] = {
    {CS_GPIO0, PAD_LED_0, GPIO_DIR_OUTPUT, LED_OFF_LEVEL, GPIO_TRIG_NONE},
    {CS_GPIO0, PAD_BUTTON_0, GPIO_DIR_INPUT, LED_OFF_LEVEL, GPIO_TRIG_RISING_FAILING_EDGE},

#ifdef AT_ENABLE
    {CS_GPIO0, PIN_AT_WAKEUP, GPIO_DIR_INPUT, GPIO_LEVEL_HIGH, GPIO_TRIG_RISING_FAILING_EDGE},
#endif
};

void board_init(void)
{
    // 启用 DCDC (默认是LDO模式)
    drv_pmu_dcdc_enable(CFG_DCDC_ENABLE);
    // Enable clock divider
    drv_rcc_clock_set(RCC_CLK_CPU, CFG_CPM_CLK_VALUE);
#if CFG_CLK_DBL_ENABLE            // 启用倍频
    drv_pmu_xtal32m_x2_startup(); // 依赖外部 32MHz 晶振
#endif
    // Use RC32K
    drv_pmu_select_32k(BOARD_32K_SELECT);
    // 所有引脚均为上拉(默认所有 IO 均为浮空输入,若启用了外部32KHz,则排除p24 p25)
    drv_pmu_pin_mode_set(PMU_PIN_ALL_MASK & ((BOARD_32K_SELECT == PMU_32K_SEL_32768HZ_XTAL) ? ~(BITMASK(24) | BITMASK(25)) : ~0U), PMU_PIN_MODE_PU);
    // 所有引脚的驱动电流强度(配置为"正常")
    drv_pmu_pin_driven_current_set(PMU_PIN_ALL_MASK, BOARD_PIN_DRIVER_CURRENT);
    // 引脚复用初始化
    drv_pin_init(pin_config, sizeof(pin_config) / sizeof(pin_config[0]));
    // GPIO初始化
    drv_gpio_init(gpio_config, sizeof(gpio_config) / sizeof(gpio_config[0]));
    // 设置引脚高低电平
    drv_gpio_write(CS_GPIO0, BITMASK(PAD_LED_0), GPIO_LEVEL_HIGH);
}
