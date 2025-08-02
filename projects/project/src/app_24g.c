#include "cs.h"
#include "cs_driver.h"
#include "shell.h"
#include "evt.h"
#include "pm.h"
#include "board.h"
#include "mbr.h"
#include "nvds.h"
#include "app_24g.h"

#include "cs_bc.h"
#include "cs_ble.h"

#define TX_ROLE           0 // 使用接收模式
#define CS_24G_ACK_MODE   0 // 禁用ack
#define ENABLE_SLEEP_MODE 0 // 禁用睡眠

static volatile uint16_t error_count = 0;
static volatile uint16_t right_count = 0;
static volatile uint16_t tx_count    = 0;

static uint8_t cs_24g_tx_payload[32];
static uint8_t cs_24g_rx_payload[64]; // 使用双重缓冲

/**
 * @brief 2.4G无线配置(结构A模式)
 *
 * 特点:高可靠性,ACK确认,动态长度,专为CST92F41/Nordic优化
 * 数据包格式：
 * [前导码0xAA] + [同步字0x817e817e84] + [动态包头] + [Payload 1-32字节] + [2字节CRC]
 * 同步字:5字节(sync_word0 + 地址字段)
 * 包头：9比特(6比特长度 + 2比特PID + 1比特NO_ACK标志)
 */
cs_24g_config_t cs_24g_config_a = {
    .tx_data           = cs_24g_tx_payload,
    .rx_data           = cs_24g_rx_payload,
    .packet_struct_sel = CS_24G_STRUCTURE_A,
    .preamble          = 0xaa,
    .preamble_len      = 0x01,
    .sync_word_sel     = CS_24G_SYNCWORD0,
    .sync_word0        = 0x817e817e,
    .sync_word1        = 0x817e817e,
    .tx_addr           = 0x84,
    .rx_addr           = 0x84,
    .addr_chk          = 0x01,
    .static_len        = 0x20,
    .hdr_bits          = 0x00,
    .addr1_bits        = 0x00,
    .len_bits          = 0x00,
    .addr1_pos         = 0x00,
    .len_pos           = 0x00,
    .endian            = CS_24G_ENDIAN_MSB,
    .freq              = 2404,
    .data_rate         = CS_24G_RATE_1M,
    .ack_en            = 1,
    .dpl_en            = 1,
    .white_en          = 0,
    .white_skip_hdr    = 1,
    .white_skip_addr   = 1,
    .white_skip_crc    = 1,
    .white_sel         = 0x00,
    .white_seed        = 0x80,
    .white_obit        = 0x07,
    .crc_len           = CS_24G_CRC_2BYTE,
    .crc_en            = 1,
    .crc_mode          = 0,
    .crc_poly          = 0x1021, // 0x07,
    .crc_init          = 0xFFFF, // 0xFF,
    .crc_skip_sync     = 0,
    .crc_skip_len      = 0,
    .crc_skip_addr     = 0,
    .modulation_mode   = CS_24G_MODULATION_FSK,
    .detect_mode       = CS_24G_SOFT_DETECTION,
};

/**
 * @brief 2.4G无线配置()结构B模式)
 *
 * 特点:高速率,无ACK,兼容多品牌芯片(Nordic/TI/Silicon Labs)
 * 数据包格式：
 * [前导码0xAA] + [同步字0xEDD47656] + [1字节长度头] + [Payload 1-32字节] + [2字节CRC]
 * 同步字：固定4字节(0xEDD47656)
 * 白化编码：作用于包头,Payload和CRC字段
 */
cs_24g_config_t cs_24g_config_b = {
    .tx_data           = cs_24g_tx_payload,
    .rx_data           = cs_24g_rx_payload,
    .packet_struct_sel = CS_24G_STRUCTURE_B,
    .preamble          = 0xaa,
    .preamble_len      = 0x01,
    .sync_word_sel     = CS_24G_SYNCWORD0,
    .sync_word0        = 0xEDD47656,
    .sync_word1        = 0xEDD47656,
    .tx_addr           = 0x02,
    .rx_addr           = 0x02,
    .addr_chk          = 0x00,
    .static_len        = 0x20,
    .hdr_bits          = 0x08,
    .addr1_bits        = 0x00,
    .len_bits          = 0x08,
    .addr1_pos         = 0x00,
    .len_pos           = 0x00,
    .endian            = CS_24G_ENDIAN_MSB,
    .freq              = 2480,
    .data_rate         = CS_24G_RATE_2M,
    .ack_en            = 0,
    .dpl_en            = 1,
    .white_en          = 1,
    .white_skip_hdr    = 0,
    .white_skip_addr   = 1,
    .white_skip_crc    = 0,
    .white_sel         = 0x00,
    .white_seed        = 0x80,
    .white_obit        = 0x07,
    .crc_len           = CS_24G_CRC_2BYTE,
    .crc_en            = 1,
    .crc_mode          = 0,
    .crc_poly          = 0x1021, // 0x07,
    .crc_init          = 0xFFFF, // 0xFF,
    .crc_skip_sync     = 0,
    .crc_skip_len      = 0,
    .crc_skip_addr     = 0,
    .modulation_mode   = CS_24G_MODULATION_GFSK,
    .detect_mode       = CS_24G_SOFT_DETECTION,
};

// 函数声明
static void app_24g_rx(uint8_t *data, uint8_t length);

static void cs_24g_callback(void *cs_reg, drv_event_t drv_event, void *buff, void *num)
{
    uint16_t payload_lenth = 0;
    bool error_flag        = false;

    switch (drv_event) {
        case DRV_EVENT_COMMON_RECEIVE_COMPLETED: // 接收完成(没有启用ack不会执行操作)
            // payload_lenth = (uint32_t)num;
            app_24g_rx((uint8_t *)buff, (uint8_t)(uintptr_t)num);

#if (TX_ROLE)
            CS_LOG_DEBUG_ARRAY_EX("rcv ack", buff, payload_lenth);
#else
#if (CS_24G_ACK_MODE)
            for (uint8_t i = 0; i < payload_lenth; i++) {
                cs_24g_tx_payload[i] = *((uint8_t *)buff + i);
            }
            cs_24g_write_ack(payload_lenth);
            CS_LOG_DEBUG_ARRAY_EX("send ack", cs_24g_tx_payload, payload_lenth);
            CS_LOG_DEBUG_ARRAY_EX("rcv tx", buff, payload_lenth);
#endif
#endif

#if ENABLE_SLEEP_MODE
            CS_24G_CE_LOW();                                  // 拉低射频模块 CE 脚
            pm_sleep_allow(PM_ID_24G);                        // 允许 2.4g 模块进入睡眠模式
            cs_24g_control(CS_24G_CONTROL_CLK_DISABLE, NULL); // 禁用 2.4g 模块时钟

            drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_SET_TIMER_VAL, (void *)(drv_pmu_timer_cnt_get() + PMU_TIMER_MS2TICK(100))); // PMU_TIMER_US2TICK
            drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_ENABLE, NULL);
#endif

            break;
        case DRV_EVENT_COMMON_RX_OVERFLOW: // 接收溢出
            CS_LOG_DEBUG("Max_Retry\r\n");
            pm_sleep_allow(PM_ID_24G);
            cs_24g_control(CS_24G_CONTROL_CLK_DISABLE, NULL);
            drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_SET_TIMER_VAL, (void *)(drv_pmu_timer_cnt_get() + PMU_TIMER_MS2TICK(500))); // PMU_TIMER_US2TICK
            drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_ENABLE, NULL);
            break;

        case DRV_EVENT_COMMON_TRANSMIT_COMPLETED: // 数据发送完成
            cs_24g_switch_role(CS_24G_ROLE_PRX);  // 切换到PRX(主接收)模式
            CS_24G_CE_HIGH();                     // 使能射频芯片(CE引脚拉高)
#if 0
            //            pm_sleep_allow(PM_ID_24G);
            //            cs_24g_control(CS_24G_CONTROL_CLK_DISABLE, NULL);
            //            drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_SET_TIMER_VAL, (void *)(drv_pmu_timer_cnt_get() + PMU_TIMER_MS2TICK(130))); // PMU_TIMER_US2TICK
            //            drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_ENABLE, NULL);
            tx_count++;
            CS_LOG_DEBUG("tx_cnt: %d\r\n", tx_count);
            if (tx_count == 1000) {
                tx_count = 0;
                drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_DISABLE, NULL);
            }
#endif
            break;
        case DRV_EVENT_COMMON_GENERAL:
            break;
        default:
            CS_ASSERT(0);
            break;
    }
}

// ble 与 2.4g 共存
static void app_24g_ble_bb_frame_ongoing_handler(bool is_ongoing)
{
    // 用于跟踪射频模块的仲裁状态
    static bool low_rate_enable = false;
    if (is_ongoing) {
        // ble 正在运行,关闭2.4g
        if (REGR(&CS_24G->RF_DR, MASK_POS(CS_24G_EN_TX_ARB)) == true) {
            low_rate_enable = true;
            // 关闭 2.4g 的TX/RX仲裁
            REGW(&CS_24G->RF_DR, MASK_2REG(CS_24G_EN_RX_ARB, false, CS_24G_EN_TX_ARB, false));
            // 切换到软件检测模式(降低功耗)
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x00)); // CS_24G_SOFT_DETECTION
        }
        // 调整接收调制指数 0.5 (优化 ble 接收功能)
        REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x40, PHY_H_RX_CTRL_RVS_FXP, 0x80)); // RX modulation index = 0.5
        // 关闭 2.4g 时钟
        cs_24g_control(CS_24G_CONTROL_CLK_DISABLE, NULL);
    } else {
        // ble 停止运行,2.4g 恢复
        if (low_rate_enable) {
            // 启用 2.4g 时钟
            DRV_RCC_CLOCK_ENABLE(RCC_CLK_2P4, 1U);
            // 恢复2.4G的TX/RX仲裁
            REGW(&CS_24G->RF_DR, MASK_2REG(CS_24G_EN_RX_ARB, true, CS_24G_EN_TX_ARB, true));
            // 恢复硬件检测模式(高性能)
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x03)); // CS_24G_DPLL_DETECTION
        }
        // 启用2.4G时钟
        cs_24g_control(CS_24G_CONTROL_CLK_ENABLE, NULL);
#if RTE_CS_24G_NORDIC
        REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x29, PHY_H_RX_CTRL_RVS_FXP, 0xC8)); // RX modulation index = 0.32
#endif
        cs_24g_read_int(cs_24g_rx_payload, 32); // 启用 2.4g 中断接收
    }
}

/*Packet structure B receives packets in interrupt mode and can communicate with function cs_24g_write_int_structure_b(void). */
static void cs_24g_read_int_structure_b(void)
{
    CS_LOG_DEBUG("cs_24g_read_int_structure_b \r\n");
    cs_24g_init(&cs_24g_config_b);
    // cs_24g_control(CS_24G_CONTROL_DUMP_RF_REGISTER, NULL);  // 暂不调试,注释此行
    cs_24g_register_event_callback(cs_24g_callback); // 注册事件回调函数
    cs_24g_read_int(cs_24g_rx_payload, 32);          // 启用中断接收
}

/*Packet structure A receives packets in interrupt mode and can communicate with function cs_24g_write_int_structure_a(void). */
static void cs_24g_read_int_structure_a(void)
{
    CS_LOG_DEBUG("cs_24g_read_int_structure_a \r\n");
    cs_24g_init(&cs_24g_config_a);
    // cs_24g_control(CS_24G_CONTROL_DUMP_RF_REGISTER, NULL);
    cs_24g_register_event_callback(cs_24g_callback);
    cs_24g_read_int(cs_24g_rx_payload, 32);
}

// 在非 ack 模式下发送 2.4g 无线数据包
static void cs_24g_write_it(void)
{
    static uint16_t j = 1;

    j++;

    for (uint8_t i = 0; i < 32; i++) {
        cs_24g_tx_payload[i] = i + 1;
    }
    // cs_24g_control(CS_24G_CONTROL_DUMP_RF_REGISTER, NULL);
    cs_24g_write_int(cs_24g_tx_payload, 32);
    j++;
    if (j > 32) {
        j = 1;
    }
}

// 在 ack 模式下发送 2.4g 无线数据包
static void cs_24g_write_it_ack_mode(void)
{
    static uint16_t j = 1; // 记录当前发送的数据长度(1-32字节)

    for (uint8_t i = 0; i < 32; i++) {
        cs_24g_tx_payload[i] = i + 1;
    }
    // 注册事件回调函数,处理发送完成,接收等事件
    cs_24g_register_event_callback(cs_24g_callback);
    // 发送数据包,长度为j字节
    cs_24g_write_int(cs_24g_tx_payload, j);
    j++;
    if (j > 32) {
        j = 1;
    }
}

static void cs_24g_write_int_structure_a(void)
{
    CS_LOG_DEBUG("cs_24g_write_int_structure_a \r\n");
    cs_24g_init(&cs_24g_config_a);
    // cs_24g_control(CS_24G_CONTROL_DUMP_RF_REGISTER, NULL);
    // cs_24g_write_it_ack_mode();
    // 设置定时器
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_SET_TIMER_VAL, (void *)(drv_pmu_timer_cnt_get() + PMU_TIMER_MS2TICK(500)));
    // 启定时器
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_ENABLE, NULL);
}

static void timer1_callback(void *cs_reg, drv_event_t drv_event, void *param0, void *param1)
{
    CS_24G_CE_LOW();                                 // 拉低无线模块的CE脚
    cs_24g_control(CS_24G_CONTROL_CLK_ENABLE, NULL); // 启用射频时钟
    pm_sleep_prevent(PM_ID_24G);                     // 阻止系统进入睡眠
#if TX_ROLE
#if CS_24G_ACK_MODE
    cs_24g_write_it_ack_mode();
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_SET_TIMER_VAL, (void *)(drv_pmu_timer_cnt_get() + PMU_TIMER_MS2TICK(500)));
    drv_pmu_timer_control(PMU_TIMER_TRIG_VAL1, PMU_TIMER_CONTROL_ENABLE, NULL);
#else
    cs_24g_write_it();
#endif
#else
#if ENABLE_SLEEP_MODE
#if CS_24G_ACK_MODE
    cs_24g_register_event_callback(cs_24g_callback);
#else
    cs_24g_register_event_callback(cs_24g_callback);
#endif
    cs_24g_read_int(cs_24g_rx_payload, 32);
#endif

#endif
    // CS_LOG_DEBUG("tim \r\n");
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */

void app_24g_init(void)
{
    // 注册回调函数,用于处理正在进行的基带帧
    cs_bc_bb_frame_ongoing_callback_register(app_24g_ble_bb_frame_ongoing_handler);
    // 注册一个定时器中断服务例程(ISR)回调函数 PMU_TIMER_TRIG_VAL1
    drv_pmu_timer_register_isr_callback(PMU_TIMER_TRIG_VAL1, timer1_callback);

    CS_ASSERT_WHILE(true, RTE_CS_24G_CS92F4X); // 断言检查,确保该宏存在
    CS_ASSERT_WHILE(true, (sizeof(cs_24g_rx_payload) >= (2 * cs_24g_config_b.static_len)));

    cs_24g_read_int_structure_b(); // 使用无ack接收方式
#if 0
#if (TX_ROLE)
    cs_24g_write_int_structure_a();
#else
    cs_24g_read_int_structure_a();
#endif
#endif
}

void app_24g_tx(uint8_t *data, uint8_t length)
{
    if (data == NULL || length == 0 || length > 32) {
        APP_ERROR("Invalid data");
        return;
    }
    if (!cs_24g_tx_idle()) {
        APP_ERROR("TX is busy");
        return;
    }

    CS_24G_CE_LOW();
    cs_24g_control(CS_24G_CONTROL_CLK_ENABLE, NULL); // 启用射频时钟
    pm_sleep_prevent(PM_ID_24G);                     // 阻止休眠

    memcpy(cs_24g_tx_payload, data, length); // 填充发送缓冲区
    cs_24g_write_int(cs_24g_tx_payload, length);
    APP_PRINTF_BUF("[tx data]", cs_24g_tx_payload, length);
}

static void app_24g_rx(uint8_t *data, uint8_t length)
{
    APP_PRINTF_BUF("rcv tx", data, length); // 打印接收到的消息
}
