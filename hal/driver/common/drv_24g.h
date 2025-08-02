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
 * @file     drv_24g.h
 * @brief    CS24G Driver
 * @date     24. July 2021
 * @author   chipsea
 *
 * @defgroup CS24G CS24G
 * @ingroup  DRIVER
 * @brief    Private 24G protocol
 * @details  Private 24G protocol
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

#ifndef __CS_24G_H__
#define __CS_24G_H__

/*********************************************************************
 * INCLUDES
 */
#include "RTE_cst92f4x.h"
#if (RTE_CS24G)
#include <stdint.h>
#ifdef __cplusplus

extern "C" {
#endif

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * TYPEDEFS
 */

/// CS24G PACKET STRUCTURE
typedef enum {
    /// PACKET STRUCTURE A
    CS_24G_STRUCTURE_A = 0,
    /// PACKET STRUCTURE B
    CS_24G_STRUCTURE_B = 1,
} cs_24g_structure_t;

/// CS24G SYNCWORD SEL
typedef enum {
    /// Select sync word 0
    CS_24G_SYNCWORD0 = 0,
    /// Select sync word 1
    CS_24G_SYNCWORD1 = 1,
} cs_24g_syncword_t;

/// CS24G transfer mode
typedef enum {
    /// RX mode
    CS_24G_ROLE_PRX,
    /// TX mode
    CS_24G_ROLE_PTX,
} cs_24g_role_t;

/// CS24G DEVIATION
typedef enum {
    /// 62.5K TX frequency offset
    CS_24G_DEVIATION_62P5K,
    /// 125K TX frequency offset
    CS_24G_DEVIATION_125K,
    /// 250K TX frequency offset
    CS_24G_DEVIATION_250K,
    /// 500K TX frequency offset
    CS_24G_DEVIATION_500K,
} cs_24g_deviation_t;

/// CS24G RATE
typedef enum {
    /// 250K data rate
    CS_24G_RATE_250K,
    /// 500K data rate
    CS_24G_RATE_500K,
    /// 1M data rate
    CS_24G_RATE_1M,
    /// 2M data rate
    CS_24G_RATE_2M,
    /// 100K data rate
    CS_24G_RATE_100K,
    /// 125K data rate
    CS_24G_RATE_125K,
    /// 50K data rate
    CS_24G_RATE_50K,
    /// 25K data rate
    CS_24G_RATE_25K,
} cs_24g_rate_t;

/// CS24G CRC_BYTE
typedef enum {
    /// 1 byte CRC
    CS_24G_CRC_1BYTE,
    /// 2 byte CRC
    CS_24G_CRC_2BYTE,
    /// 3 byte CRC
    CS_24G_CRC_3BYTE,
    /// 4 byte CRC
    CS_24G_CRC_4BYTE,
} cs_24g_crc_byte_t;

/// CS24G MODULATION MODE
typedef enum {
    /// GFSK MODULATION MODE
    CS_24G_MODULATION_GFSK = 0,
    /// FSK MODULATION MODE
    CS_24G_MODULATION_FSK = 1,
} cs_24g_modulation_t;

/// CS24G ENDIAN
typedef enum {
    /// LSB BYTE FIRST
    CS_24G_ENDIAN_LSB = 0,
    /// MSB BYTE FIRST
    CS_24G_ENDIAN_MSB = 1,
} cs_24g_endian_t;

/// CS24G DEMODULATION MODE
typedef enum {
    /// Software demodulation
    CS_24G_SOFT_DETECTION,
    /// Hardware demodulation
    CS_24G_HARD_DETECTION,
    /// Dpll demodulation
    CS_24G_DPLL_DETECTION,
} cs_24g_detect_mode_t;

/// CS24G config
typedef struct {
    uint8_t *tx_data; // 发送数据缓冲区指针
    uint8_t *rx_data; // 接收数据缓冲区指针

    cs_24g_structure_t packet_struct_sel; // 数据包结构A/B
    uint8_t preamble;                     // 前导码
    uint8_t preamble_len;                 // 前导码长度
    cs_24g_syncword_t sync_word_sel;      // 同步字选择(SYNCWORD0 或 SYNCWORD1)
    uint32_t sync_word0;                  // ​​同步字低32位​​
    uint32_t sync_word1;                  // 同步字高32位​
    uint8_t tx_addr;                      // 发送地址
    uint8_t rx_addr;                      // 接收地址
    uint8_t addr_chk;                     // 地址校验​​(1=启用,0=禁用)
    /// The number of bits in the header. if hdr_bits = 8, static_len = 257, It can avoid the problem of AGC lock up, but it will increase RAM usage, and it is necessary to set cs_24d_rx_cyload [257 * 2].
    uint8_t hdr_bits; // 包头位数字,如8就代表1字节包头
    /// In fixed length mode, it is the length of the receiving and sending. In dynamic length mode, it is the maximum packet length that can be sent.
    uint16_t static_len; // ​​固定包长度(静态模式)或最大包长度(动态模式)
    uint8_t addr1_bits;  // 接收地址的位数
    uint8_t len_bits;    // 长度字段的位数​​(动态长度模式下)
    /// The location of rx pipe address.
    uint8_t addr1_pos;       // 接收地址在数据包中的位置​​(比特偏移量)
    uint8_t addr1_loc;       // ​​接收地址的存储位置​​(在包头,包尾或独立字段)
    uint8_t len_pos;         // ​​长度字段在数据包中的位置​​(比特偏移量)
    cs_24g_endian_t endian;  // ​​字节序​​(MSB=大端，LSB=小端)
    uint16_t freq;           // ​​射频频率​
    cs_24g_rate_t data_rate; // 数据传输速度
    uint8_t ack_en;          // 自动确认(ACK)​​(1=启用,0=禁用)
    uint8_t dpl_en;          // ​动态负载长度​​(1=启用,0=禁用)

    uint8_t white_en;        // 白化使能(1=启用,0=禁用)
    uint8_t white_skip_hdr;  // 跳过包头白化​​(1=跳过,0=处理)
    uint8_t white_skip_addr; // 跳过地址白化​​(1=跳过,0=处理)
    uint8_t white_skip_crc;  // 跳过CRC白化​​(1=跳过,0=处理)
    uint8_t white_sel;       // 白化算法选择​​(0~7,适配不同芯片)
    uint16_t white_seed;     // ​​白化初始种子值
    uint8_t white_obit;      // 白化输出位控制​​(调整打乱强度)

    cs_24g_crc_byte_t crc_len; // crc长度
    uint8_t crc_en;            // CRC 使能(1=启用,0=禁用)
    uint8_t crc_mode;          // CRC 计算模式​​
    uint32_t crc_poly;         // CRC 多项式​
    uint32_t crc_init;         // ​​CRC 初始值​​
    uint8_t crc_skip_sync;     // ​​跳过同步字CRC校验​​(1=跳过)
    uint8_t crc_skip_len;      // ​​跳过长度字段CRC校验​​(1=跳过)
    uint8_t crc_skip_addr;     // ​​跳过地址字段CRC校验(1=跳过)

    cs_24g_modulation_t modulation_mode; //	​​调制方式​​(FSK/GFSK)
    cs_24g_detect_mode_t detect_mode;    // 解调模式(软件/硬件/锁相环)
} cs_24g_config_t;

/// CS24G PIPE
typedef enum {
    /// pipe 0
    CS_24G_PIPE0,
    /// pipe 1
    CS_24G_PIPE1,
    /// pipe 2
    CS_24G_PIPE2,
    /// pipe 3
    CS_24G_PIPE3,
    /// pipe 4
    CS_24G_PIPE4,
    /// pipe 5
    CS_24G_PIPE5,
    /// pipe 6
    CS_24G_PIPE6,
    /// pipe 7
    CS_24G_PIPE7,
    /// pipe tx
    CS_24G_PIPE_TX,
} cs_24g_pipe_t;

/// cs_24g control
typedef enum {
    CS_24G_CONTROL_CLK_DISABLE      = 0U, /**< Disable clock CS24G controller, argu is bool, return CS_ERROR_OK */
    CS_24G_CONTROL_CLK_ENABLE       = 1U, /**< Enable clock CS24G controller, argu is bool, return CS_ERROR_OK */
    CS_24G_CONTROL_RESET            = 2U, /**< Reseet CS24G controller, argu is bool, return CS_ERROR_OK */
    CS_24G_CONTROL_SWITCH_ROLE      = 3U, /**< rx/tx mode switch, argu is cs_24g_role_t, return CS_ERROR_OK */
    CS_24G_CONTROL_DUMP_RF_REGISTER = 4U, /**< Dump 2.4G rf register, argu is NULL, return CS_ERROR_OK */
} cs_24g_control_t;

/// cs_24g transfer case
typedef enum {
    CS_24G_SENSITIVITY,
    CS_24G_POLL_STRUCTURE_A,
    CS_24G_INT_STRUCTURE_A,
    CS_24G_INT_STRUCTURE_B,
    CS_24G_ACK_MODE_STRUCTURE_A,
    CS_24G_INT_ACK_MODE_STRUCTURE_A,
    CS_24G_TX_RX_SWITCH_POLL,
    CS_24G_TX_RX_SWITCH_INT,
    CS_24G_RX_ADDR_CHECK,
} cs_24g_transfer_t;

/*******************************************************************************
 * EXTERN VARIABLES
 */

/*********************************************************************
 * EXTERN FUNCTIONS
 */
/**
 *******************************************************************************
 * @brief set RX  check_address value.
 *
 * @param[in] *pipenum          pipe nummber.
 * @param[in] *addr             rx address.
 *******************************************************************************
 */
extern void cs_24g_set_addr(uint8_t pipenum, uint8_t addr);

/**
 *******************************************************************************
 * @brief Switch between transmitting and receiving devices..
 *
 * @param[in] role  CS_24G_ROLE_PRX or CS_24G_ROLE_PTX.
 *******************************************************************************
 */
extern void cs_24g_switch_role(cs_24g_role_t role);

/**
 *******************************************************************************
 * @brief Set the 2.4G data rate us. Please use function cs_24g_set_rf_parameters to set the rate, cs_24g_set_rate did not configure the frequency offset.
 *
 * @param[in] data_rate       data rate.
 * @param[in] en_arb          Enable low speed mode. if is true, rx_rate = data_rate / n_avr, tx_rate = data_rate / n_rep.
 * @param[in] n_avr           Receive rate division value.
 * @param[in] n_rep           Transmission rate division value.
 *******************************************************************************
 */
extern void cs_24g_set_rate(uint8_t data_rate, bool en_arb, uint8_t n_avr, uint8_t n_rep);

/**
 *******************************************************************************
 * @brief Set transmission rate, frequency point.
 * The speed only allows the use of speeds listed in enumeration cs_24g_rate_t in drv_24g.h. Please do not configure them arbitrarily.
 * For special requirements, please contact the developer.
 *
 * @param[in] data_rate         tx/rx data rate.
 * @param[in] frequency         Integer frequency point.
 * @param[in] fract_freq        Decimal frequency point.
 *******************************************************************************
 */
extern void cs_24g_set_rf_parameters(cs_24g_rate_t data_rate, uint16_t frequency, float fract_freq);

/**
 *******************************************************************************
 * @brief Set demodulation mode.
 *
 * @param[in] detc_mode   CS_24G_SOFT_DETECTION，CS24G_HARD_DETECTION or CS_24G_DPLL_DETECTION.
 *******************************************************************************
 */
extern void cs_24g_detection_mode(uint8_t detc_mode);

/**
 *******************************************************************************
 * @brief Obtain the rssi value..
 *
 * @return rssi_value
 *******************************************************************************
 */
extern int8_t cs_24g_get_rssi(void);

/**
 *******************************************************************************
 * @brief init the 2.4g transerive when the event begin.
 * If the deviation is dynamically switched, it is prohibited to repeatedly call cs_24g_init.
 *
 * @param[in] cfg          2.4G register configuration.
 *******************************************************************************
 */
extern void cs_24g_init(const cs_24g_config_t *cfg);

/**
 *******************************************************************************
 * @brief Registering 2.4G Callbacks In interrupt mode.
 *
 * @param[in] event_cb     call back
 *******************************************************************************
 */
extern void cs_24g_register_event_callback(drv_isr_callback_t event_cb);

/**
 *******************************************************************************
 * @brief Set 2.4G frequency point.
 *
 * @param[in]  freq_channel      Integer frequency point
 * @param[in]  fract_freq         Decimal frequency point, If set freq to 2402.123456MHZ, freq_channel = 2402, fractFreq = 0.123456
 *******************************************************************************
 */
extern void cs_24g_set_freq(uint16_t freq_channel, float fract_freq);

/**
 *******************************************************************************
 * @brief sending data to transmit FIFO by polling mode.
 *
 * @param[in]  tx_payload      Data to be written to transmit FIFO
 * @param[in]  tx_num      Length of payload
 *
 * @return true:  send success
 *         false: send fali, Trigger maximum retransmission interrupt
 *******************************************************************************
 */
extern bool cs_24g_write(const uint8_t *tx_payload, uint16_t tx_num);

/**
 *******************************************************************************
 * @brief sending data to transmit FIFO by interrupt mode.
 *
 * @param[in]  tx_payload       Data to be written to transmit FIFO
 * @param[in]  tx_num      Number of bytes to write
 *******************************************************************************
 */
extern void cs_24g_write_int(const uint8_t *tx_payload, uint16_t tx_num);

/**
 *******************************************************************************
 * @brief Receive data from receive FIFO by polling mode.
 *
 * @param[in] rx_payload      Data read from receive buffer
 * @param[in] timeout_ms      rx timeout value
 *
 * @return Length of data received
 *******************************************************************************
 */
extern uint16_t cs_24g_read(uint8_t *rx_payload, uint32_t timeout_ms);

/**
 *******************************************************************************
 * @brief Receive data from receive FIFO by interrupt mode. The received data and length can be obtained from the callback function.

 * @param[in] rx_payload      Data read from receive buffer
 * @param[in] max_rx_num      The maximum number of receives for this application.

 *******************************************************************************
 */
extern void cs_24g_read_int(uint8_t *rx_payload, uint16_t max_rx_num);

/**
 *******************************************************************************
 * @brief Receive ack data from receive FIFO.
 *
 *  @return Length of data received.
 *******************************************************************************
 */
extern uint16_t cs_24g_read_ack(void);

/**
 *******************************************************************************
 * @brief write ack data to transmit FIFO.
 *
 * @param[in]  tx_num       Number of bytes to write
 *******************************************************************************
 */
__STATIC_FORCEINLINE void cs_24g_write_ack(uint8_t tx_num)
{
    CS_24G->DMA_TX_LEN = tx_num;
}

/**
 *******************************************************************************
 * @brief cs_24g interrupt service routine
 *******************************************************************************
 */
extern void drv_cs_24g_isr(void);

/**
 *******************************************************************************
 * @brief Control the CS24G CLK and Printing registers.
 * cs_24g_control() contains drv_pmu_ana_enable(), so turning on or turning off the clock only requires calling cs_24g_control().
 *
 * @param[in] control        Control options
 * @param[in] argu           argument for control options, Not used, always set NULL
 *
 * @return                   status:
 *                           execution_status
 *******************************************************************************
 */
extern void *cs_24g_control(cs_24g_control_t control, void *argu);

/**
 *******************************************************************************
 * @brief Check if the TX is idle
 *
 * @return   TRUE: IDLE  FALSE: BUSY
 *******************************************************************************
 */
extern bool cs_24g_tx_idle(void);

/**
 *******************************************************************************
 * @brief Check if the RX is idle
 *
 * @return   TRUE: IDLE  FALSE: BUSY
 *******************************************************************************
 */
extern bool cs_24g_rx_idle(void);

/**
 *******************************************************************************
 * @brief Continuously turn on PLL to shorten the packet transmission time.
 *
 * @param[in] enable      Is the PLL continuously turned on.
 *******************************************************************************
 */
extern void cs_24g_pll_continue_open(bool enable);

/**
 *******************************************************************************
 * @brief Initialization sequence for Ripple radio
 *******************************************************************************
 */
extern void cs_24g_rf_init_seq(void);
#ifdef __cplusplus
}
#endif

#endif /*RTE_CS24G*/
#endif /*__CS_24G_H*/

/** @} */
