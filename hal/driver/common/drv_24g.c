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
 * @file     cs_24g.c
 * @brief    CS24G driver
 * @date     1. Dec. 2022
 * @author   chipsea
 *
 * @version
 * Version 1.0
 *  - Initial release
 *
 * @{
 */

/*******************************************************************************
 * INCLUDES
 */
#include "RTE_cst92f4x.h"
#if (RTE_CS24G)
#include <stddef.h>
#include "cs_device.h"
#include "cs_driver.h"
/*********************************************************************
 * MACROS
 */
#define PLL_CONTIMUE_OPEN 0
/*******************************************************************************
 * TYPEDEFS
 */
typedef struct
{
    drv_isr_callback_t event_cb; /**< event callback                      */
    uint16_t tx_cnt;             /**< Count of data sent*/
    uint16_t max_rx_num;         /**< num of data sent*/
    const uint8_t *tx_buf;       /**< Pointer to out data buffer, system to peripheral direction */
    uint8_t *rx_buf;             /**< Pointer to in data buffer            */
    bool deviation_below_250K;   /**< Flag with frequency deviation less than 250K */
    uint8_t rx_count;            /**< rx_count is used for alternate counting of ping pong buffers, not the number of received packets. */
    struct
    {
        uint32_t RX_GFSK_SYNC_CTRL;
        uint32_t TX_CTRL0;
        uint32_t PLL_CTRL1;
        uint32_t FREQ_CFG3;
        uint32_t FREQ_CFG0;
        uint32_t PKTCTRL0;
        uint32_t FB_PKTCTRL;
        uint32_t SYNC_WORD0;
        uint32_t SYNC_WORD1;
        uint32_t DMA_TX_ADDR;
        uint32_t DMA_RX_ADDR;
        uint32_t CRCPOLY;
        uint32_t CRCINIT;
        uint32_t SETUP_VALUE;
        uint32_t RXCRC;
        uint32_t PACKET_LEN;
        uint8_t DYNPD;
        uint8_t FEATURE;
        uint8_t INT_MASK;
        uint8_t EN_AA;
        uint8_t EN_RXADDR;
        uint8_t FA_PKTCTRL;
        uint8_t FA_SETUP_RETR;
        uint8_t PREAMBLE;
        uint8_t PREAMBLE_LEN;
        uint16_t PRE_GUARD;
        uint8_t TX_ADDR;
        uint8_t SYNC_WORD_SEL;
        uint8_t RX_ADDR_P0;
        uint8_t RX_ADDR_P1;
        uint8_t RX_ADDR_P2;
        uint8_t RX_ADDR_P3;
        uint8_t RX_ADDR_P4;
        uint8_t RX_ADDR_P5;
        uint8_t RX_ADDR_P6;
        uint8_t RX_ADDR_P7;
        uint16_t RF_DR;
        uint8_t CRCCFG;
        uint8_t CRCSKIP;
        uint8_t WHITECFG;
        uint8_t WHITESEL;
        uint8_t WHITEOUT;
        uint8_t ACK_MODE;
        uint8_t FPGA_TX_INDEX;
        uint16_t WHITESEED;
        uint16_t ENDIAN;
        uint8_t FD_CFO_CMP;
        uint8_t DET_MODE;
        uint16_t H_RX_CTRL;
        uint16_t RF_PD_AHEAD;
        uint32_t KDCO_LUT_2M_REG0;
        uint32_t KDCO_LUT_2M_REG1;
        uint32_t PD_CFG2;
        uint32_t STR_CTRL;
    } reg_store;
} cs_24g_env_t;

/*******************************************************************************
 * CONST & VARIABLES
 */

/* cs_24g information */
#if (RTE_CS24G)
static cs_24g_env_t cs_24g_env = {
    .event_cb             = NULL,
    .tx_cnt               = 0U,
    .tx_buf               = NULL,
    .rx_buf               = NULL,
    .deviation_below_250K = false,
    .rx_count             = 0,
};

static uint8_t devia_offset[22][2] = {{0xA, 0x1}, {0xB, 0x1}, {0xC, 0x2}, {0xD, 0x2}, {0xE, 0x3}, {0xF, 0x3}, {0x10, 0x4}, {0x11, 0x4}, {0x12, 0x5}, {0x13, 0x5}, {0x14, 0x6}, {0x15, 0x6}, {0x16, 0x7}, {0x17, 0x7}, {0x18, 0x8}, {0x19, 0x8}, {0x1A, 0x9}, {0x1B, 0x9}, {0x1C, 0xA}, {0x1D, 0xA}, {0x1E, 0xB}, {0x1E, 0xB}};
#endif /* RTE_CS24G */

/*******************************************************************************
 * LOCAL FUNCTIONS
 */
__RAM_CODES("24G")
static void cs_24g_ce_high_pulse(void)
{
    // CE HIGH
    CS_24G->MAC_EN = 1;
    DRV_DELAY_US(1);
    CS_24G->DMA_CMD = 0xA0;
    DRV_DELAY_US(3);
    // CE LOW
    CS_24G->MAC_EN = 0;
}

__RAM_CODES("24G")
static uint16_t cs_24g_read_rx_payload_width(void)
{
    uint16_t Width;
    uint8_t en_dpl;
    en_dpl = REGR(&CS_24G->FEATURE, CS_24G_EN_DPL_MASK, CS_24G_EN_DPL_POS);
    if (en_dpl) {
        Width = REGR(&CS_24G->RX_DYN_LEN, CS_24G_RX_DYN_LEN_MASK, CS_24G_RX_DYN_LEN_POS);
    } else {
        Width = REGR(&CS_24G->PACKET_LEN, CS_24G_PACKET_LEN_MASK, CS_24G_PACKET_LEN_POS);
    }
    return Width;
}

__RAM_CODES("24G")
static void cs_24g_reg_store(void)
{
    DRV_RCC_CLOCK_ENABLE(RCC_CLK_2P4, 1U);
    DRV_RCC_ANA_CLK_ENABLE_NOIRQ();
    cs_24g_env.reg_store.PKTCTRL0          = CS_24G->PKTCTRL0;
    cs_24g_env.reg_store.FB_PKTCTRL        = CS_24G->FB_PKTCTRL;
    cs_24g_env.reg_store.DYNPD             = CS_24G->DYNPD;
    cs_24g_env.reg_store.FEATURE           = CS_24G->FEATURE;
    cs_24g_env.reg_store.INT_MASK          = CS_24G->INT_MASK;
    cs_24g_env.reg_store.EN_AA             = CS_24G->EN_AA;
    cs_24g_env.reg_store.EN_RXADDR         = CS_24G->EN_RXADDR;
    cs_24g_env.reg_store.FA_PKTCTRL        = CS_24G->FA_PKTCTRL;
    cs_24g_env.reg_store.FA_SETUP_RETR     = CS_24G->FA_SETUP_RETR;
    cs_24g_env.reg_store.PREAMBLE          = CS_24G->PREAMBLE;
    cs_24g_env.reg_store.PREAMBLE_LEN      = CS_24G->PREAMBLE_LEN;
    cs_24g_env.reg_store.PRE_GUARD         = CS_24G->PRE_GUARD;
    cs_24g_env.reg_store.SYNC_WORD0        = CS_24G->SYNC_WORD0;
    cs_24g_env.reg_store.SYNC_WORD1        = CS_24G->SYNC_WORD1;
    cs_24g_env.reg_store.TX_ADDR           = CS_24G->TX_ADDR;
    cs_24g_env.reg_store.SYNC_WORD_SEL     = CS_24G->SYNC_WORD_SEL;
    cs_24g_env.reg_store.RX_ADDR_P0        = CS_24G->RX_ADDR_P0;
    cs_24g_env.reg_store.RX_ADDR_P1        = CS_24G->RX_ADDR_P1;
    cs_24g_env.reg_store.RX_ADDR_P2        = CS_24G->RX_ADDR_P2;
    cs_24g_env.reg_store.RX_ADDR_P3        = CS_24G->RX_ADDR_P3;
    cs_24g_env.reg_store.RX_ADDR_P4        = CS_24G->RX_ADDR_P4;
    cs_24g_env.reg_store.RX_ADDR_P5        = CS_24G->RX_ADDR_P5;
    cs_24g_env.reg_store.RX_ADDR_P6        = CS_24G->RX_ADDR_P6;
    cs_24g_env.reg_store.RX_ADDR_P7        = CS_24G->RX_ADDR_P7;
    cs_24g_env.reg_store.RF_DR             = CS_24G->RF_DR;
    cs_24g_env.reg_store.CRCCFG            = CS_24G->CRCCFG;
    cs_24g_env.reg_store.CRCPOLY           = CS_24G->CRCPOLY;
    cs_24g_env.reg_store.CRCINIT           = CS_24G->CRCINIT;
    cs_24g_env.reg_store.CRCSKIP           = CS_24G->CRCSKIP;
    cs_24g_env.reg_store.WHITECFG          = CS_24G->WHITECFG;
    cs_24g_env.reg_store.WHITESEL          = CS_24G->WHITESEL;
    cs_24g_env.reg_store.WHITESEED         = CS_24G->WHITESEED;
    cs_24g_env.reg_store.WHITEOUT          = CS_24G->WHITEOUT;
    cs_24g_env.reg_store.DMA_TX_ADDR       = CS_24G->DMA_TX_ADDR;
    cs_24g_env.reg_store.DMA_RX_ADDR       = CS_24G->DMA_RX_ADDR;
    cs_24g_env.reg_store.PACKET_LEN        = CS_24G->PACKET_LEN;
    cs_24g_env.reg_store.SETUP_VALUE       = CS_24G->SETUP_VALUE;
    cs_24g_env.reg_store.ENDIAN            = CS_24G->ENDIAN;
    cs_24g_env.reg_store.ACK_MODE          = CS_24G->ACK_MODE;
    cs_24g_env.reg_store.FPGA_TX_INDEX     = CS_24G->FPGA_TX_INDEX;
    cs_24g_env.reg_store.FREQ_CFG0         = CS_DAIF->FREQ_CFG0;
    cs_24g_env.reg_store.FREQ_CFG3         = CS_DAIF->FREQ_CFG3;
    cs_24g_env.reg_store.PLL_CTRL1         = CS_DAIF->PLL_CTRL1;
    cs_24g_env.reg_store.PD_CFG2           = CS_DAIF->PD_CFG2;
    cs_24g_env.reg_store.H_RX_CTRL         = CS_PHY->H_RX_CTRL;
    cs_24g_env.reg_store.FD_CFO_CMP        = CS_PHY->FD_CFO_CMP;
    cs_24g_env.reg_store.DET_MODE          = CS_PHY->DET_MODE;
    cs_24g_env.reg_store.TX_CTRL0          = CS_PHY->TX_CTRL0;
    cs_24g_env.reg_store.RX_GFSK_SYNC_CTRL = CS_PHY->RX_GFSK_SYNC_CTRL;
    cs_24g_env.reg_store.STR_CTRL          = CS_PHY->STR_CTRL;
    cs_24g_env.reg_store.RF_PD_AHEAD       = CS_24G->RF_PD_AHEAD;
    DRV_RCC_ANA_CLK_RESTORE_NOIRQ();
    DRV_RCC_CLOCK_ENABLE(RCC_CLK_2P4, 0U);
}

__RAM_CODES("24G")
static void cs_24g_reg_restore(void)
{
    DRV_RCC_CLOCK_ENABLE(RCC_CLK_2P4, 1U);
    DRV_RCC_ANA_CLK_ENABLE_NOIRQ();
    CS_24G->PKTCTRL0          = cs_24g_env.reg_store.PKTCTRL0;
    CS_24G->PRE_GUARD         = cs_24g_env.reg_store.PRE_GUARD;
    CS_24G->PREAMBLE          = cs_24g_env.reg_store.PREAMBLE;
    CS_24G->PREAMBLE_LEN      = cs_24g_env.reg_store.PREAMBLE_LEN;
    CS_24G->SYNC_WORD_SEL     = cs_24g_env.reg_store.SYNC_WORD_SEL;
    CS_24G->SYNC_WORD0        = cs_24g_env.reg_store.SYNC_WORD0;
    CS_24G->SYNC_WORD1        = cs_24g_env.reg_store.SYNC_WORD1;
    CS_24G->TX_ADDR           = cs_24g_env.reg_store.TX_ADDR;
    CS_24G->RX_ADDR_P0        = cs_24g_env.reg_store.RX_ADDR_P0;
    CS_24G->RX_ADDR_P1        = cs_24g_env.reg_store.RX_ADDR_P1;
    CS_24G->RX_ADDR_P2        = cs_24g_env.reg_store.RX_ADDR_P2;
    CS_24G->RX_ADDR_P3        = cs_24g_env.reg_store.RX_ADDR_P3;
    CS_24G->RX_ADDR_P4        = cs_24g_env.reg_store.RX_ADDR_P4;
    CS_24G->RX_ADDR_P5        = cs_24g_env.reg_store.RX_ADDR_P5;
    CS_24G->RX_ADDR_P6        = cs_24g_env.reg_store.RX_ADDR_P6;
    CS_24G->RX_ADDR_P7        = cs_24g_env.reg_store.RX_ADDR_P7;
    CS_24G->PACKET_LEN        = cs_24g_env.reg_store.PACKET_LEN;
    CS_24G->FB_PKTCTRL        = cs_24g_env.reg_store.FB_PKTCTRL;
    CS_24G->RF_DR             = cs_24g_env.reg_store.RF_DR;
    CS_DAIF->FREQ_CFG0        = cs_24g_env.reg_store.FREQ_CFG0;
    CS_DAIF->FREQ_CFG3        = cs_24g_env.reg_store.FREQ_CFG3;
    CS_DAIF->PLL_CTRL1        = cs_24g_env.reg_store.PLL_CTRL1;
    CS_DAIF->PD_CFG2          = cs_24g_env.reg_store.PD_CFG2;
    CS_24G->FPGA_TX_INDEX     = cs_24g_env.reg_store.FPGA_TX_INDEX;
    CS_PHY->H_RX_CTRL         = cs_24g_env.reg_store.H_RX_CTRL;
    CS_24G->EN_AA             = cs_24g_env.reg_store.EN_AA;
    CS_24G->FEATURE           = cs_24g_env.reg_store.FEATURE;
    CS_24G->DYNPD             = cs_24g_env.reg_store.DYNPD;
    CS_24G->WHITECFG          = cs_24g_env.reg_store.WHITECFG;
    CS_24G->WHITESEL          = cs_24g_env.reg_store.WHITESEL;
    CS_24G->WHITESEED         = cs_24g_env.reg_store.WHITESEED;
    CS_24G->WHITEOUT          = cs_24g_env.reg_store.WHITEOUT;
    CS_24G->CRCCFG            = cs_24g_env.reg_store.CRCCFG;
    CS_24G->CRCPOLY           = cs_24g_env.reg_store.CRCPOLY;
    CS_24G->CRCINIT           = cs_24g_env.reg_store.CRCINIT;
    CS_24G->CRCSKIP           = cs_24g_env.reg_store.CRCSKIP;
    CS_24G->ENDIAN            = cs_24g_env.reg_store.ENDIAN;
    CS_24G->FA_SETUP_RETR     = cs_24g_env.reg_store.FA_SETUP_RETR;
    CS_24G->DMA_TX_ADDR       = cs_24g_env.reg_store.DMA_TX_ADDR;
    CS_24G->DMA_RX_ADDR       = cs_24g_env.reg_store.DMA_RX_ADDR;
    CS_24G->INT_MASK          = cs_24g_env.reg_store.INT_MASK;
    CS_24G->SETUP_VALUE       = cs_24g_env.reg_store.SETUP_VALUE;
    CS_PHY->TX_CTRL0          = cs_24g_env.reg_store.TX_CTRL0;
    CS_PHY->FD_CFO_CMP        = cs_24g_env.reg_store.FD_CFO_CMP;
    CS_PHY->DET_MODE          = cs_24g_env.reg_store.DET_MODE;
    CS_PHY->RX_GFSK_SYNC_CTRL = cs_24g_env.reg_store.RX_GFSK_SYNC_CTRL;
    CS_PHY->STR_CTRL          = cs_24g_env.reg_store.STR_CTRL;
    CS_24G->FA_PKTCTRL        = cs_24g_env.reg_store.FA_PKTCTRL;
    CS_24G->EN_RXADDR         = cs_24g_env.reg_store.EN_RXADDR;
    CS_24G->ACK_MODE          = cs_24g_env.reg_store.ACK_MODE;
    CS_24G->RF_PD_AHEAD       = cs_24g_env.reg_store.RF_PD_AHEAD;
    DRV_RCC_ANA_CLK_RESTORE_NOIRQ();
    DRV_RCC_CLOCK_ENABLE(RCC_CLK_2P4, 0U);
}

#if (RTE_PM)
__RAM_CODES("24G")
static void cs_24g_pm_sleep_store_restore_handler(pm_sleep_state_t sleep_state, pm_status_t power_status)
{
    if (sleep_state == PM_SLEEP_STORE) {
        cs_24g_reg_store();
        // CS_LOG_DEBUG("cs_24g_reg_store\r\n");
    } else if (sleep_state == PM_SLEEP_RESTORE_HSI) {
        cs_24g_reg_restore();
        // CS_LOG_DEBUG("cs_24g_reg_restore_HSI\r\n");
    } else if (sleep_state == PM_SLEEP_RESTORE_HSE) {
        // cs_24g_reg_restore();
        // CS_LOG_DEBUG("cs_24g_reg_restore_HSE\r\n");
    }
}
#endif

static void cs_24g_dump_rf_register(void)
{
    CS_LOG_DEBUG("PKTCTRL0: %08x\r\n", CS_24G->PKTCTRL0);
    CS_LOG_DEBUG("FB_PKTCTRL: %08x\r\n", CS_24G->FB_PKTCTRL);
    CS_LOG_DEBUG("DYNPD: %02x\r\n", CS_24G->DYNPD);
    CS_LOG_DEBUG("FEATURE: %02x\r\n", CS_24G->FEATURE);
    CS_LOG_DEBUG("INT_ST: %02x\r\n", CS_24G->INT_ST);
    CS_LOG_DEBUG("EN_AA: %02x\r\n", CS_24G->EN_AA);
    CS_LOG_DEBUG("EN_RXADDR: %02x\r\n", CS_24G->EN_RXADDR);
    CS_LOG_DEBUG("FA_PKTCTRL: %02x\r\n", CS_24G->FA_PKTCTRL);
    CS_LOG_DEBUG("SETUP_RETR: %02x\r\n", CS_24G->FA_SETUP_RETR);
    CS_LOG_DEBUG("FA_OBSERVE_TX: %02x\r\n", CS_24G->FA_OBSERVE_TX);
    CS_LOG_DEBUG("PREAMBLE: %02x\r\n", CS_24G->PREAMBLE);
    CS_LOG_DEBUG("PREAMBLE_LEN: %02x\r\n", CS_24G->PREAMBLE_LEN);
    CS_LOG_DEBUG("PRE_GUARD: %02x\r\n", CS_24G->PRE_GUARD);
    CS_LOG_DEBUG("SYNC_WORD0: %08x\r\n", CS_24G->SYNC_WORD0);
    CS_LOG_DEBUG("SYNC_WORD1: %08x\r\n", CS_24G->SYNC_WORD1);
    CS_LOG_DEBUG("TX_ADDR: %02x\r\n", CS_24G->TX_ADDR);
    CS_LOG_DEBUG("SYNC_WORD_SEL: %02x\r\n", CS_24G->SYNC_WORD_SEL);
    CS_LOG_DEBUG("RX_ADDR_P0: %02x\r\n", CS_24G->RX_ADDR_P0);
    CS_LOG_DEBUG("RX_ADDR_P1: %02x\r\n", CS_24G->RX_ADDR_P1);
    CS_LOG_DEBUG("RX_ADDR_P2: %02x\r\n", CS_24G->RX_ADDR_P2);
    CS_LOG_DEBUG("RX_ADDR_P3: %02x\r\n", CS_24G->RX_ADDR_P3);
    CS_LOG_DEBUG("RX_ADDR_P4: %02x\r\n", CS_24G->RX_ADDR_P4);
    CS_LOG_DEBUG("RX_ADDR_P5: %02x\r\n", CS_24G->RX_ADDR_P5);
    CS_LOG_DEBUG("RX_ADDR_P6: %02x\r\n", CS_24G->RX_ADDR_P6);
    CS_LOG_DEBUG("RX_ADDR_P7: %02x\r\n", CS_24G->RX_ADDR_P7);
    CS_LOG_DEBUG("RF_DR: %02x\r\n", CS_24G->RF_DR);
    CS_LOG_DEBUG("RX_P_NO: %02x\r\n", CS_24G->RX_P_NO);
    CS_LOG_DEBUG("CRCCFG: %08x\r\n", CS_24G->CRCCFG);
    CS_LOG_DEBUG("CRCPOLY: %08x\r\n", CS_24G->CRCPOLY);
    CS_LOG_DEBUG("CRCINIT: %08x\r\n", CS_24G->CRCINIT);
    CS_LOG_DEBUG("CRCSKIP: %02x\r\n", CS_24G->CRCSKIP);
    CS_LOG_DEBUG("WHITECFG: %02x\r\n", CS_24G->WHITECFG);
    CS_LOG_DEBUG("WHITESEL: %02x\r\n", CS_24G->WHITESEL);
    CS_LOG_DEBUG("WHITESEED: %02x\r\n", CS_24G->WHITESEED);
    CS_LOG_DEBUG("WHITEOUT: %02x\r\n", CS_24G->WHITEOUT);
    CS_LOG_DEBUG("DMA_CMD: %02x\r\n", CS_24G->DMA_CMD);
    CS_LOG_DEBUG("DMA_TX_LEN: %02x\r\n", CS_24G->DMA_TX_LEN);
    CS_LOG_DEBUG("RX_DYN_LEN: %02x\r\n", CS_24G->RX_DYN_LEN);
    CS_LOG_DEBUG("DMA_TX_ADDR: %02x\r\n", CS_24G->DMA_TX_ADDR);
    CS_LOG_DEBUG("DMA_RX_ADDR: %08x\r\n", CS_24G->DMA_RX_ADDR);
    CS_LOG_DEBUG("PACKET_LEN: %02x\r\n", CS_24G->PACKET_LEN);
    CS_LOG_DEBUG("BCC: %02x\r\n", CS_24G->BCC);
    CS_LOG_DEBUG("TIMESTAMP_RT: %02x\r\n", CS_24G->TIMESTAMP_RT);
    CS_LOG_DEBUG("TIMESTAMP_TRIGER: %02x\r\n", CS_24G->TIMESTAMP_TRIGER);
    CS_LOG_DEBUG("TIMESTAMP_CFG: %02x\r\n", CS_24G->TIMESTAMP_CFG);
    CS_LOG_DEBUG("SETUP_VALUE: %02x\r\n", CS_24G->SETUP_VALUE);
    CS_LOG_DEBUG("ENDIAN: %02x\r\n", CS_24G->ENDIAN);
    CS_LOG_DEBUG("FLUSH: %02x\r\n", CS_24G->FLUSH);
    CS_LOG_DEBUG("TESTCTRL: %02x\r\n", CS_24G->TESTCTRL);
    CS_LOG_DEBUG("STATE: %02x\r\n", CS_24G->STATE);
    CS_LOG_DEBUG("FREQ: %02d\r\n", REGR(&CS_DAIF->FREQ_CFG0, DAIF_FREQ_REG_MO_MASK, DAIF_FREQ_REG_MO_POS));
    CS_LOG_DEBUG("FRACT_FREQ: %x\r\n", CS_DAIF->FREQ_CFG3);
    CS_LOG_DEBUG("CS_PHY->REG_PHY_RST_N: %02x\r\n", CS_PHY->REG_PHY_RST_N);
    CS_LOG_DEBUG("CS_PHY->TX_CTRL0: %02x\r\n", CS_PHY->TX_CTRL0);
    CS_LOG_DEBUG("CS_PHY->DET_MODE: %02x\r\n", CS_PHY->DET_MODE);
    CS_LOG_DEBUG("CS_PHY->H_RX_CTRL: %02x\r\n", CS_PHY->H_RX_CTRL);
    CS_LOG_DEBUG("CS_PHY->FD_CFO_CMP: %02x\r\n", CS_PHY->FD_CFO_CMP);
    CS_LOG_DEBUG("CS_PHY->STR_CTRL: %02x\r\n", CS_PHY->STR_CTRL);
    CS_LOG_DEBUG("CS_PHY->RX_GFSK_SYNC_CTRL: %08x\r\n", CS_PHY->RX_GFSK_SYNC_CTRL);
}

static uint8_t cs_24g_set_kdco(uint8_t kdco_value)
{
    if (kdco_value >= 0xA) {
        for (uint8_t i = 0; i < 22; i++) {
            if (kdco_value == devia_offset[i][0]) {
                return kdco_value = devia_offset[i][1];
            }
        }
    }
    return kdco_value;
}

static void cs_24g_set_deviation(cs_24g_deviation_t deviation)
{
    if (REGR(&CS_24G->FEATURE, MASK_POS(CS_24G_EN_ACK_PAY)) == 0x1) {
        deviation = CS_24G_DEVIATION_250K;
    }
    switch (deviation) {
        case CS_24G_DEVIATION_62P5K:
            cs_24g_env.deviation_below_250K = false;
            REGW0(&CS_DAIF->PLL_CTRL1, DAIF_FREQ_DEVIA_BYPASS_MASK);           // freq devia bypass=0d
            REGW(&CS_DAIF->PLL_CTRL1, MASK_1REG(DAIF_FREQ_DEVIA_COEFF, 0xFF)); // TX deviation value = 62.5k
            REGW(&CS_DAIF->PD_CFG2, MASK_2REG(DAIF_RFPLL_PD_TXDAC_ME, 1, DAIF_RFPLL_PD_TXDAC_MO, 1));
            break;
        case CS_24G_DEVIATION_125K:
            cs_24g_env.deviation_below_250K = true;
            REGW0(&CS_DAIF->PLL_CTRL1, DAIF_FREQ_DEVIA_BYPASS_MASK);           // freq devia bypass=0d
            REGW(&CS_DAIF->PLL_CTRL1, MASK_1REG(DAIF_FREQ_DEVIA_COEFF, 0xFF)); // TX deviation value = 125k
            REGW(&CS_DAIF->KDCO_LUT_2M_REG0, MASK_1REG(DAIF_KDCO_LUT_2M_0, cs_24g_set_kdco((cs_24g_env.reg_store.KDCO_LUT_2M_REG0 & DAIF_KDCO_LUT_2M_0_MASK) >> DAIF_KDCO_LUT_2M_0_POS)));
            REGW(&CS_DAIF->KDCO_LUT_2M_REG0, MASK_1REG(DAIF_KDCO_LUT_2M_1, cs_24g_set_kdco((cs_24g_env.reg_store.KDCO_LUT_2M_REG0 & DAIF_KDCO_LUT_2M_1_MASK) >> DAIF_KDCO_LUT_2M_1_POS)));
            REGW(&CS_DAIF->KDCO_LUT_2M_REG0, MASK_1REG(DAIF_KDCO_LUT_2M_2, cs_24g_set_kdco((cs_24g_env.reg_store.KDCO_LUT_2M_REG0 & DAIF_KDCO_LUT_2M_2_MASK) >> DAIF_KDCO_LUT_2M_2_POS)));
            REGW(&CS_DAIF->KDCO_LUT_2M_REG0, MASK_1REG(DAIF_KDCO_LUT_2M_3, cs_24g_set_kdco((cs_24g_env.reg_store.KDCO_LUT_2M_REG0 & DAIF_KDCO_LUT_2M_3_MASK) >> DAIF_KDCO_LUT_2M_3_POS)));
            REGW(&CS_DAIF->KDCO_LUT_2M_REG1, MASK_1REG(DAIF_KDCO_LUT_2M_4, cs_24g_set_kdco((cs_24g_env.reg_store.KDCO_LUT_2M_REG1 & DAIF_KDCO_LUT_2M_4_MASK) >> DAIF_KDCO_LUT_2M_4_POS)));
            REGW(&CS_DAIF->KDCO_LUT_2M_REG1, MASK_1REG(DAIF_KDCO_LUT_2M_5, cs_24g_set_kdco((cs_24g_env.reg_store.KDCO_LUT_2M_REG1 & DAIF_KDCO_LUT_2M_5_MASK) >> DAIF_KDCO_LUT_2M_5_POS)));
            break;
        case CS_24G_DEVIATION_250K:
        case CS_24G_DEVIATION_500K:
            cs_24g_env.deviation_below_250K = false;
            REGW1(&CS_DAIF->PLL_CTRL1, DAIF_FREQ_DEVIA_BYPASS_MASK); // freq devia bypass=1d
            if (SYS_IS_A3()) {                                       // A3 CHIP
                REGW(&CS_DAIF->PLL_CTRL2, MASK_1REG(DAIF_EN_KVCO2, 0x3));
            } else { // A2 CHIP
                REGW(&CS_DAIF->PLL_CTRL2, MASK_1REG(DAIF_EN_KVCO2, 0x7));
            }
            REGW(&CS_DAIF->VCO_CTRL1, MASK_2REG(DAIF_BLE_1M_2M_SEL_ME, 0x0, DAIF_BLE_1M_2M_SEL_MO, 0x0));
            CS_DAIF->KDCO_LUT_2M_REG0 = cs_24g_env.reg_store.KDCO_LUT_2M_REG0;
            CS_DAIF->KDCO_LUT_2M_REG1 = cs_24g_env.reg_store.KDCO_LUT_2M_REG1;
            break;
        default:
            break;
    }
}

/*******************************************************************************
 * PUBLIC FUNCTIONS
 */
void cs_24g_register_event_callback(drv_isr_callback_t event_cb)
{
    cs_24g_env.event_cb = event_cb;
}

__RAM_CODES("24G")
void cs_24g_switch_role(cs_24g_role_t role)
{
    CS_24G_CE_LOW();
    CS_24G_FLUSH_TX_FIFO();
    CS_24G_FLUSH_RX_FIFO();
    cs_error_t ret; // Max: 150us
    DRV_WAIT_US_UNTIL_TO(!((REGR(&CS_24G->STATE, MASK_POS(CS_24G_MAIN_STATE)) == 0x02) || (REGR(&CS_24G->STATE, MASK_POS(CS_24G_MAIN_STATE)) == 0x00)), 150, ret);
    (void)ret;
    DRV_DELAY_US(1);
    if (role == CS_24G_ROLE_PTX) {
        REGW0(&CS_24G->PKTCTRL0, CS_24G_PKTCTRL0_PRIM_RX_MASK);
    } else {
        REGW1(&CS_24G->PKTCTRL0, CS_24G_PKTCTRL0_PRIM_RX_MASK);
    }
    DRV_DELAY_US(1);
}

__RAM_CODES("24G")
static void cs_24g_write_tx_payload(uint16_t dma_tx_num, const uint8_t *payload)
{
    CS_24G->DMA_TX_ADDR = (uint32_t)payload;
    CS_24G->DMA_TX_LEN  = dma_tx_num;
}

__RAM_CODES("24G")
bool cs_24g_write(const uint8_t *tx_payload, uint16_t tx_num)
{
    uint8_t status = 0x00;

    if (cs_24g_env.deviation_below_250K) {
        REGW(&CS_DAIF->PLL_CTRL2, MASK_1REG(DAIF_EN_KVCO2, 0x0));
        REGW(&CS_DAIF->VCO_CTRL1, MASK_2REG(DAIF_BLE_1M_2M_SEL_ME, 0x1, DAIF_BLE_1M_2M_SEL_MO, 0x1));
    }
#if RTE_CS_24G_SILICONLAB
    REGW(&CS_PHY->TX_CTRL0, MASK_2REG(PHY_TX_CTRL0_EN_INTERP, 0x1, PHY_TX_CTRL0_BDR_PPM_TX, 0x220));
#endif
    cs_24g_switch_role(CS_24G_ROLE_PTX);
    cs_24g_write_tx_payload(tx_num, tx_payload);
    cs_24g_ce_high_pulse();
    while (1) {
        status = CS_24G->INT_ST;
        if ((CS_24G_STATUS_TX_DS & status) || (CS_24G_STATUS_MAX_RT & status)) {
            CS_24G->INT_ST = status;
            if (CS_24G_STATUS_TX_DS & status) {
                status = 0;
                return true;
            } else {
                status = 0;
                return false;
            }
        }
    }
}

__RAM_CODES("24G")
void cs_24g_write_int(const uint8_t *tx_payload, uint16_t tx_num)
{
    if (cs_24g_env.deviation_below_250K) { // 偏频补偿
        REGW(&CS_DAIF->PLL_CTRL2, MASK_1REG(DAIF_EN_KVCO2, 0x0));
        REGW(&CS_DAIF->VCO_CTRL1, MASK_2REG(DAIF_BLE_1M_2M_SEL_ME, 0x1, DAIF_BLE_1M_2M_SEL_MO, 0x1));
    }
#if RTE_CS_24G_SILICONLAB
    REGW(&CS_PHY->TX_CTRL0, MASK_2REG(PHY_TX_CTRL0_EN_INTERP, 0x1, PHY_TX_CTRL0_BDR_PPM_TX, 0x220));
#endif
    cs_24g_switch_role(CS_24G_ROLE_PTX);         // 切换为发送模式
    cs_24g_write_tx_payload(tx_num, tx_payload); // 填充发送缓冲区
    cs_24g_ce_high_pulse();                      // 通过拉高 CE 引脚,触发射频模块开始发送数据
}

__RAM_CODES("24G")
uint16_t cs_24g_read(uint8_t *rx_payload, uint32_t timeout_ms)
{
    uint16_t rx_cnt    = 0;
    bool rx_right_flag = false;
    cs_error_t error;

    if (cs_24g_env.deviation_below_250K) {
        REGW(&CS_DAIF->VCO_CTRL1, MASK_2REG(DAIF_BLE_1M_2M_SEL_ME, 0x0, DAIF_BLE_1M_2M_SEL_MO, 0x0));
    }
    CS_24G->DMA_RX_ADDR = (uint32_t)rx_payload;
    cs_24g_switch_role(CS_24G_ROLE_PRX);
    CS_24G_CE_HIGH();

    while (1) {
        DRV_WAIT_MS_UNTIL_TO(!(CS_24G->INT_ST & (CS_24G_STATUS_RX_DR | CS_24G_STATUS_CRC_ERR)), timeout_ms, error);
        if (error != CS_ERROR_OK) {
            CS_24G_CE_LOW();
            return 0;
        }
        if (CS_24G_STATUS_RX_DR & CS_24G->INT_ST) {
            REGW1(&CS_24G->INT_ST, CS_24G_STATUS_RX_DR);
            CS_24G_CE_LOW();
            if ((!(CS_PHY->FYSNC_DET_INFO & PHY_FYSNC_DET_INFO_FYSNC_CFO_EST_MASK)) && (!(CS_24G_STATUS_CRC_ERR & CS_24G->INT_ST))) {
                rx_cnt        = cs_24g_read_rx_payload_width();
                rx_right_flag = true;
            } else {
                rx_right_flag = false;
                CS_24G_CE_HIGH();
            }
        }
        if (rx_right_flag) {
            break;
        }
        if (CS_24G_STATUS_CRC_ERR & CS_24G->INT_ST) {
            REGW1(&CS_24G->INT_ST, CS_24G_STATUS_CRC_ERR);
        }
    }
    return rx_cnt;
}

/**
 *******************************************************************************
 * @brief Receive data from receive FIFO by interrupt mode. The received data and length can be obtained from the callback function.

 * @param[in] rx_payload      Data read from receive buffer.
 * @param[in] max_rx_num      The maximum number of receives for this application.

 *******************************************************************************
 */
__RAM_CODES("24G")
void cs_24g_read_int(uint8_t *rx_payload, uint16_t max_rx_num)
{
    cs_24g_env.rx_buf     = rx_payload;
    cs_24g_env.max_rx_num = max_rx_num;

    if (cs_24g_env.deviation_below_250K) {
        REGW(&CS_DAIF->VCO_CTRL1, MASK_2REG(DAIF_BLE_1M_2M_SEL_ME, 0x0, DAIF_BLE_1M_2M_SEL_MO, 0x0));
    }

    cs_24g_switch_role(CS_24G_ROLE_PRX); // 切换到PRX(主接收)模式
    CS_24G_CE_HIGH();                    // 使能射频芯片(CE引脚拉高)
}

uint16_t cs_24g_read_ack(void)
{
    uint16_t payload_lenth = 0;

    payload_lenth = cs_24g_read_rx_payload_width();
    CS_24G_CLEAR_ALL_IRQ();

    return payload_lenth;
}

void cs_24g_set_addr(uint8_t pipenum, uint8_t addr)
{
    switch (pipenum) {
        case CS_24G_PIPE_TX:
            REGW(&CS_24G->TX_ADDR, MASK_1REG(CS_24G_TX_ADDR, addr));
            break;
        case CS_24G_PIPE0:
            REGW(&CS_24G->RX_ADDR_P0, MASK_1REG(CS_24G_RX_ADDR_P0, addr));
            break;
        case CS_24G_PIPE1:
            REGW(&CS_24G->RX_ADDR_P1, MASK_1REG(CS_24G_RX_ADDR_P1, addr));
            break;
        case CS_24G_PIPE2:
            REGW(&CS_24G->RX_ADDR_P2, MASK_1REG(CS_24G_RX_ADDR_P2, addr));
            break;
        case CS_24G_PIPE3:
            REGW(&CS_24G->RX_ADDR_P3, MASK_1REG(CS_24G_RX_ADDR_P3, addr));
            break;
        case CS_24G_PIPE4:
            REGW(&CS_24G->RX_ADDR_P4, MASK_1REG(CS_24G_RX_ADDR_P4, addr));
            break;
        case CS_24G_PIPE5:
            REGW(&CS_24G->RX_ADDR_P5, MASK_1REG(CS_24G_RX_ADDR_P5, addr));
            break;
        case CS_24G_PIPE6:
            REGW(&CS_24G->RX_ADDR_P6, MASK_1REG(CS_24G_RX_ADDR_P6, addr));
            break;
        case CS_24G_PIPE7:
            REGW(&CS_24G->RX_ADDR_P7, MASK_1REG(CS_24G_RX_ADDR_P7, addr));
            break;
        default:
            CS_ASSERT(0);
            break;
    }
}

/*Please use function cs_24g_set_rf_parameters to set the rate, cs_24g_set_rate() did not configure the frequency offset.*/
void cs_24g_set_rate(uint8_t data_rate, bool en_arb, uint8_t n_avr, uint8_t n_rep)
{

#if RTE_CS_24G_SILICONLAB
    REGW(&CS_24G->RF_DR, MASK_4REG(CS_24G_EN_RX_ARB, true, CS_24G_EN_TX_ARB, false, CS_24G_N_AVR, 2, CS_24G_N_REP, 2));
    REGW(&CS_24G->RF_DR, MASK_2REG(CS_24G_RF_DR_TX_RF_DR, CS_24G_RATE_250K, CS_24G_RF_DR_RX_RF_DR, CS_24G_RATE_500K));
#else
    REGW(&CS_24G->RF_DR, MASK_4REG(CS_24G_EN_RX_ARB, en_arb, CS_24G_EN_TX_ARB, en_arb, CS_24G_N_AVR, n_avr, CS_24G_N_REP, n_rep));
    REGW(&CS_24G->RF_DR, MASK_2REG(CS_24G_RF_DR_TX_RF_DR, data_rate, CS_24G_RF_DR_RX_RF_DR, data_rate));
#endif
    if (((data_rate == CS_24G_RATE_1M) || (data_rate == CS_24G_RATE_2M)) && !en_arb) {
        REGW1(&CS_DAIF->PLL_CTRL1, DAIF_FREQ_DEVIA_BYPASS_MASK); // freq devia bypass=1d
    } else {
        REGW0(&CS_DAIF->PLL_CTRL1, DAIF_FREQ_DEVIA_BYPASS_MASK); // freq devia bypass=0d
    }
}

/*The speed only allows the use of speeds listed in enumeration cs_24g_rate_t in drv_24g.h. Please do not configure them arbitrarily.
For special requirements, please contact the developer.*/
void cs_24g_set_rf_parameters(cs_24g_rate_t data_rate, uint16_t frequency, float fract_freq)
{
    cs_24g_set_freq(frequency, fract_freq);
    switch (data_rate) {
        case CS_24G_RATE_25K:
            cs_24g_set_rate(CS_24G_RATE_250K, true, 10, 10);
            if (SYS_IS_FPGA()) {
                CS_24G->FPGA_TX_INDEX = 0x02;
            } else {
                cs_24g_set_deviation(CS_24G_DEVIATION_62P5K);
            }
            REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x40, PHY_H_RX_CTRL_RVS_FXP, 0x80)); // RX modulation index = 0.5
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x00));                           // CS_24G_HARD_DETECTION
            break;
        case CS_24G_RATE_50K:
            cs_24g_set_rate(CS_24G_RATE_500K, true, 10, 10);
            if (SYS_IS_FPGA()) {
                CS_24G->FPGA_TX_INDEX = 0x02;
            } else {
                cs_24g_set_deviation(CS_24G_DEVIATION_125K);
            }
            REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x40, PHY_H_RX_CTRL_RVS_FXP, 0x80)); // RX modulation index = 0.5
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x00));                           // CS_24G_HARD_DETECTION
            break;
        case CS_24G_RATE_100K:
            cs_24g_set_rate(CS_24G_RATE_500K, true, 5, 5);
            if (SYS_IS_FPGA()) {
                CS_24G->FPGA_TX_INDEX = 0x03;
            } else {
                cs_24g_set_deviation(CS_24G_DEVIATION_125K);
            }
            REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x60, PHY_H_RX_CTRL_RVS_FXP, 0x55)); // RX modulation index = 0.75
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x00));                           // CS_24G_HARD_DETECTION
            break;
        case CS_24G_RATE_125K:
            cs_24g_set_rate(CS_24G_RATE_500K, true, 4, 4); //  500K/4 = 125k
            if (SYS_IS_FPGA()) {
                CS_24G->FPGA_TX_INDEX = 0x03;
            } else {
                cs_24g_set_deviation(CS_24G_DEVIATION_125K);
            }
            REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x60, PHY_H_RX_CTRL_RVS_FXP, 0x55)); // RX modulation index = 0.75
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x00));                           // CS_24G_HARD_DETECTION
            break;
        case CS_24G_RATE_250K:
            cs_24g_set_rate(CS_24G_RATE_250K, false, 0, 0);
            if (SYS_IS_FPGA()) {
                CS_24G->FPGA_TX_INDEX = 0x06;
            } else {
                cs_24g_set_deviation(CS_24G_DEVIATION_125K);
            }
            REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x60, PHY_H_RX_CTRL_RVS_FXP, 0x55)); // RX modulation index = 0.75
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x00));                           // CS_24G_HARD_DETECTION
            break;
        case CS_24G_RATE_500K:
            cs_24g_set_rate(data_rate, 0, 0, 0);
            if (SYS_IS_FPGA()) {
                CS_24G->FPGA_TX_INDEX = 0x03;
            } else {
                cs_24g_set_deviation(CS_24G_DEVIATION_125K);
            }
            REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x60, PHY_H_RX_CTRL_RVS_FXP, 0x55)); // RX modulation index = 0.75
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x00));                           // CS_24G_HARD_DETECTION
            break;
        default:
            cs_24g_set_rate(data_rate, 0, 0, 0);
#if RTE_CS_24G_NORDIC
            if (SYS_IS_FPGA()) {
                CS_24G->FPGA_TX_INDEX = 0x01;
            }
            REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x29, PHY_H_RX_CTRL_RVS_FXP, 0xC8)); // RX modulation index = 0.32
#else
            if (SYS_IS_FPGA()) {
                CS_24G->FPGA_TX_INDEX = 0x02;
            }
            REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x40, PHY_H_RX_CTRL_RVS_FXP, 0x80)); // RX modulation index = 0.5
            // REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x29, PHY_H_RX_CTRL_RVS_FXP, 0xC8)); // RX modulation index = 0.32
            // REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x60, PHY_H_RX_CTRL_RVS_FXP, 0x55)); //RX modulation index = 0.75
            // REGW(&CS_PHY->H_RX_CTRL, MASK_2REG(PHY_H_RX_CTRL_FXP, 0x80, PHY_H_RX_CTRL_RVS_FXP, 0x40)); //RX modulation index = 1.0
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x01)); // CS_24G_SOFT_DETECTION
#endif
            cs_24g_set_deviation(CS_24G_DEVIATION_250K);
            break;
    }
}

void cs_24g_detection_mode(uint8_t detc_mode)
{
    switch (detc_mode) {
        case CS_24G_SOFT_DETECTION: // 0.5 0.32  support 1M,2M 500K,250K, not support LOW RATE.  highest accruacy, highest latency. Soft demodulation does not support phy mode 1 or above.
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x01));
            break;
        case CS_24G_HARD_DETECTION: //   suport all modulation index 1M,500K,250K ,LOW RATE,  2M not suport, middle accruacy, middle latency.
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x00));
            break;
        case CS_24G_DPLL_DETECTION: // support all modulation index and rate, lowest accruacy, lowest latency.
            REGW(&CS_PHY->DET_MODE, MASK_1REG(PHY_DET_MODE_DET_MODE, 0x03));
            break;
        default:
            CS_ASSERT(0);
            break;
    }
#if RTE_CS_24G_SILICONLAB
    REGW(&CS_PHY->FD_CFO_CMP, MASK_1REG(PHY_FD_CFO_CMP_FD_CFO_CMP, 0));
    REGW(&CS_PHY->TX_CTRL0, MASK_2REG(PHY_TX_CTRL0_TX_GFSK_MODE, 0x04, PHY_TX_CTRL0_COEF_GFSK, 0x0D));  // phy mode4
    REGW(&CS_PHY->TX_CTRL0, MASK_2REG(PHY_TX_CTRL0_EN_INTERP, 0x01, PHY_TX_CTRL0_BDR_PPM_TX, 0x00220)); // 0x00100 0x00520
    REGW(&CS_PHY->STR_CTRL, MASK_1REG(PHY_STR_CTRL_BDR_PPM_RX, 0x0088));
    REGW(&CS_PHY->RX_GFSK_SYNC_CTRL, MASK_2REG(PHY_RX_GFSK_SYNC_CTRL_XCORR_TH_32, 0x30, PHY_RX_GFSK_SYNC_CTRL_SBE_MAX_TH_32, 1));
#else
    REGW(&CS_PHY->FD_CFO_CMP, MASK_1REG(PHY_FD_CFO_CMP_FD_CFO_CMP, 1));
    REGW(&CS_PHY->TX_CTRL0, MASK_1REG(PHY_TX_CTRL0_TX_GFSK_MODE, 0x00)); // phy mode0: GFSK MODE
#endif
}

int8_t cs_24g_get_rssi(void)
{
    int rssi = (int8_t)CS_PHY->SIG_DBM_EST_O;
    rssi     = (rssi * 100 - 1100) / 105;
    if (rssi > -25)
        rssi -= 1;

    return rssi;
}

void cs_24g_set_freq(uint16_t freq, float fract_freq)
{
    uint32_t fract_value;

    REGW(&CS_DAIF->FREQ_CFG0, MASK_1REG(DAIF_FREQ_REG_MO, freq));
    if (fract_freq) {
        fract_value = (uint32_t)(fract_freq * 0x3FFFFF);
        REGW(&CS_DAIF->FREQ_CFG3, MASK_2REG(DAIF_FREQ_FRAC_REG, fract_value, DAIF_FREQ_0P5_EN, 1));
    } else {
        REGW(&CS_DAIF->FREQ_CFG3, MASK_2REG(DAIF_FREQ_FRAC_REG, 0, DAIF_FREQ_0P5_EN, 0));
    }
}

void cs_24g_pll_continue_open(bool enable)
{
    if (enable) {
        REGW(&CS_DAIF->PD_CFG1, MASK_2REG(DAIF_RFPLL_PD_ALL_ME, 1, DAIF_RFPLL_PD_ALL_MO, 0));
        REGW(&CS_DAIF->MAIN_ST_CFG2, MASK_1REG(DAIF_RX_PLL_WAIT, 0xA0)); // PLL calib = 10us
        CS_DAIF->MAIN_ST_CFG0 = 0;
        REGW(&CS_DAIF->MAIN_ST_CFG1, MASK_1REG(DAIF_RXLDO_WAIT, 0x190)); // LDO calib = 25us
        DRV_DELAY_US(80);
        REGW(&CS_24G->SETUP_VALUE, MASK_3REG(CS_24G_RX_SETUP_VALUE, 0x28, CS_24G_TX_SETUP_VALUE, 0x1A, CS_24G_RX_TM_CNT, 0xFF));
    } else {
        REGW(&CS_DAIF->PD_CFG1, MASK_2REG(DAIF_RFPLL_PD_ALL_ME, 0, DAIF_RFPLL_PD_ALL_MO, 1));
        REGW(&CS_DAIF->MAIN_ST_CFG2, MASK_1REG(DAIF_RX_PLL_WAIT, 0x320));
        CS_DAIF->MAIN_ST_CFG0 = 0x1300180;
        REGW(&CS_DAIF->MAIN_ST_CFG1, MASK_1REG(DAIF_RXLDO_WAIT, 0x1E0));
        REGW(&CS_24G->SETUP_VALUE, MASK_3REG(CS_24G_RX_SETUP_VALUE, 0x62, CS_24G_TX_SETUP_VALUE, 0x5B, CS_24G_RX_TM_CNT, 0xFF));
    }
}

// If the deviation is dynamically switched, it is prohibited to repeatedly call cs_24g_init.
void cs_24g_init(const cs_24g_config_t *cfg)
{
    // RF clock open
    cs_24g_control(CS_24G_CONTROL_CLK_ENABLE, NULL);
#if RTE_CS_24G_SILICONLAB
    REGW(&CS_24G->PKTCTRL0, MASK_3REG(CS_24G_PKTCTRL0_STRUCT_SEL, cfg->packet_struct_sel, CS_24G_PKTCTRL0_RX_0_1_RVS, 1, CS_24G_PKTCTRL0_TX_0_1_RVS, 1));
#else
    REGW(&CS_24G->PKTCTRL0, MASK_3REG(CS_24G_PKTCTRL0_STRUCT_SEL, cfg->packet_struct_sel, CS_24G_PKTCTRL0_RX_0_1_RVS, 0, CS_24G_PKTCTRL0_TX_0_1_RVS, 0));
#endif
    REGW(&CS_24G->PKTCTRL0, MASK_1REG(CS_24G_PKTCTRL0_CE_H_THRE, 1));
    REGW(&CS_24G->PRE_GUARD, MASK_5REG(CS_24G_PREGRD_CNT, 0x07, CS_24G_PREGRD_EN, 0x00, CS_24G_TAIL_CTL, 0x10, CS_24G_GUARD_EN, 0x00, CS_24G_TAIL_PATERN, 0X02));
    CS_24G->PREAMBLE      = cfg->preamble;
    CS_24G->PREAMBLE_LEN  = cfg->preamble_len;
    CS_24G->SYNC_WORD_SEL = cfg->sync_word_sel;
    CS_24G->SYNC_WORD0    = cfg->sync_word0;
    CS_24G->SYNC_WORD1    = cfg->sync_word1;
    CS_24G->TX_ADDR       = cfg->tx_addr;
    CS_24G->RX_ADDR_P0    = cfg->rx_addr;
    REGW(&CS_24G->PKTCTRL0, MASK_2REG(CS_24G_PKTCTRL0_NUM_HDR_BITS, cfg->hdr_bits, CS_24G_PKTCTRL0_ADDR_CHK, cfg->addr_chk));
    CS_24G->PACKET_LEN = cfg->static_len;
    REGW(&CS_24G->FB_PKTCTRL, MASK_5REG(CS_24G_FB_PKTCTRL_NUM_ADDR1_BITS, cfg->addr1_bits, CS_24G_FB_PKTCTRL_NUM_LEN_BITS, cfg->len_bits, CS_24G_FB_PKTCTRL_ADDR1_POS, cfg->addr1_pos, CS_24G_FB_PKTCTRL_LEN_POS, cfg->len_pos, CS_24G_FB_PKTCTRL_ADDR1_LOC, cfg->addr1_loc));
    REGW(&CS_24G->EN_AA, 0xFF, (cfg->ack_en ? 0xFF : 0));
    REGW(&CS_24G->FEATURE, MASK_2REG(CS_24G_EN_DPL, (cfg->dpl_en ? 1 : 0), CS_24G_EN_ACK_PAY, (cfg->ack_en ? 1 : 0)));
    REGW(&CS_24G->DYNPD, 0xFF, (cfg->dpl_en ? 0xFF : 0));

    REGW(&CS_24G->WHITECFG, MASK_4REG(CS_24G_WHITE_EN, cfg->white_en, CS_24G_WHITE_SKIP_HEADER, cfg->white_skip_hdr, CS_24G_WHITE_SKIP_ADDR, cfg->white_skip_addr, CS_24G_WHITE_SKIP_CRC, cfg->white_skip_crc));
    CS_24G->WHITESEL  = cfg->white_sel;
    CS_24G->WHITESEED = cfg->white_seed;
    CS_24G->WHITEOUT  = cfg->white_obit;

    REGW(&CS_24G->CRCCFG, MASK_3REG(CS_24G_CRC_EN, cfg->crc_en, CS_24G_CRC_LEN, cfg->crc_len, CS_24G_CRC_MODE, cfg->crc_mode));
    if (cfg->endian == CS_24G_ENDIAN_MSB) {
#if RTE_CS_24G_SILICONLAB
        REGW(&CS_24G->ENDIAN, MASK_7REG(CS_24G_EDIBIT_CRC, CS_24G_ENDIAN_MSB, CS_24G_EDIBIT_PL, CS_24G_ENDIAN_MSB, CS_24G_EDIBYTE_SW, CS_24G_ENDIAN_LSB, CS_24G_EDIBIT_SW, CS_24G_ENDIAN_MSB, CS_24G_EDIBIT_ADDR, CS_24G_ENDIAN_MSB, CS_24G_EDIBIT_HDR, CS_24G_ENDIAN_MSB, CS_24G_EDIBYTE_CRC, CS_24G_ENDIAN_MSB));
#else
        REGW(&CS_24G->ENDIAN, MASK_7REG(CS_24G_EDIBIT_CRC, CS_24G_ENDIAN_MSB, CS_24G_EDIBIT_PL, CS_24G_ENDIAN_MSB, CS_24G_EDIBYTE_SW, CS_24G_ENDIAN_MSB, CS_24G_EDIBIT_SW, CS_24G_ENDIAN_MSB, CS_24G_EDIBIT_ADDR, CS_24G_ENDIAN_MSB, CS_24G_EDIBIT_HDR, CS_24G_ENDIAN_MSB, CS_24G_EDIBYTE_CRC, CS_24G_ENDIAN_MSB));
#endif
    } else {
        REGW(&CS_24G->ENDIAN, MASK_7REG(CS_24G_EDIBIT_CRC, CS_24G_ENDIAN_MSB, CS_24G_EDIBIT_PL, CS_24G_ENDIAN_LSB, CS_24G_EDIBYTE_SW, CS_24G_ENDIAN_LSB, CS_24G_EDIBIT_SW, CS_24G_ENDIAN_LSB, CS_24G_EDIBIT_ADDR, CS_24G_ENDIAN_LSB, CS_24G_EDIBIT_HDR, CS_24G_ENDIAN_LSB, CS_24G_EDIBYTE_CRC, CS_24G_ENDIAN_MSB));
    }
    CS_24G->CRCPOLY = cfg->crc_poly;
    CS_24G->CRCINIT = cfg->crc_init;
    REGW(&CS_24G->CRCSKIP, MASK_3REG(CS_24G_CRC_SKIP_SYNC, cfg->crc_skip_sync, CS_24G_CRC_SKIP_LEN, cfg->crc_skip_len, CS_24G_CRC_SKIP_ADDR, cfg->crc_skip_addr));

    REGW(&CS_24G->FA_SETUP_RETR, MASK_2REG(CS_24G_FA_SETUP_RETR_ARC, 0x03, CS_24G_FA_SETUP_RETR_ARD, 0x00));
    CS_24G->DMA_RX_ADDR = (uint32_t)cfg->rx_data;
    CS_24G->DMA_TX_ADDR = (uint32_t)cfg->tx_data;
    cs_24g_env.rx_buf   = cfg->rx_data;
    cs_24g_env.rx_count = 0;
    REGW(&CS_DAIF->MAIN_ST_CFG1, MASK_1REG(DAIF_TX_WAIT, 0));
#if PLL_CONTIMUE_OPEN
    REGW(&CS_DAIF->MAIN_ST_CFG2, MASK_1REG(DAIF_RX_PLL_WAIT, 0xA0)); // PLL calib = 10us
#if 1                                                                // Single send or single receive mode
    CS_DAIF->MAIN_ST_CFG0 = 0;
    REGW(&CS_DAIF->MAIN_ST_CFG1, MASK_1REG(DAIF_RXLDO_WAIT, 0x190)); // LDO calib = 25us
    DRV_DELAY_US(80);
    REGW(&CS_24G->SETUP_VALUE, MASK_3REG(CS_24G_RX_SETUP_VALUE, 0x28, CS_24G_TX_SETUP_VALUE, 0x1A, CS_24G_RX_TM_CNT, 0xFF));
#else // Receive and transmit conversion in ACK mode
    CS_DAIF->MAIN_ST_CFG0 = 0X00a00140;
    DRV_DELAY_US(80);
    REGW(&CS_24G->SETUP_VALUE, MASK_3REG(CS_24G_RX_SETUP_VALUE, 0x3A, CS_24G_TX_SETUP_VALUE, 0x33, CS_24G_RX_TM_CNT, 0xFF));
#endif
#else
#if RTE_CS_24G_NORDIC
    // Communicate with NORDIC: 1M RX:6A, TX:63  250k RX:78, TX:63, nordic_ack_timeout:300. 2M: RX:6A, TX:6F
    REGW(&CS_24G->SETUP_VALUE, MASK_3REG(CS24G_RX_SETUP_VALUE, 0x6A, CS_24G_TX_SETUP_VALUE, 0x63, CS_24G_RX_TM_CNT, 0xFF)); // 1M
// REGW(&CS_24G->SETUP_VALUE, MASK_3REG(CS24G_RX_SETUP_VALUE, 0x6A, CS_24G_TX_SETUP_VALUE, 0x6F, CS_24G_RX_TM_CNT, 0xFF)); // 2M
// REGW(&CS_24G->SETUP_VALUE, MASK_3REG(CS24G_RX_SETUP_VALUE, 0x78, CS_24G_TX_SETUP_VALUE, 0x63, CS_24G_RX_TM_CNT, 0xFF)); // 250K
#else
    REGW(&CS_24G->SETUP_VALUE, MASK_3REG(CS_24G_RX_SETUP_VALUE, 0x62, CS_24G_TX_SETUP_VALUE, 0x5B, CS_24G_RX_TM_CNT, 0xFF));
#endif
#endif
    // GFSK OR FSK
    if (cfg->modulation_mode) {
        REGW1(&CS_PHY->TX_CTRL0, PHY_TX_CTRL0_BP_GAU_MASK); // FSK
    } else {
        REGW0(&CS_PHY->TX_CTRL0, PHY_TX_CTRL0_BP_GAU_MASK);
    }
    cs_24g_detection_mode(cfg->detect_mode);
    // sync word: symble-bit-error criterion
    // REGW0(&CS_PHY->RX_GFSK_SYNC_CTRL, PHY_RX_GFSK_SYNC_CTRL_SBE_MAX_TH_32_MASK);
    // REGW(&CS_PHY->RX_GFSK_SYNC_CTRL, MASK_1REG(PHY_RX_GFSK_SYNC_CTRL_SBE_MAX_TH_32, 1));
    //  1 - enable double address synchronization
    REGW(&CS_PHY->RX_GFSK_SYNC_CTRL, MASK_1REG(PHY_RX_GFSK_SYNC_CTRL_EN_2ND_ADDR, 0));
    // 0 - All packages are forced to ACK, ignoring NO_ ACK bit.
    REGW(&CS_24G->FA_PKTCTRL, MASK_1REG(CS_24G_FA_PKTCTRL_EN_DYN_ACK, 1));
// Read RSSI background noise
// OM_PHY->RSSI_CAP_MODE = 1;
#if (RTE_PM)
    pm_sleep_store_restore_callback_register(cs_24g_pm_sleep_store_restore_handler);
#endif
    NVIC_ClearPendingIRQ(CS_24G_RF_IRQn);                      // 清除中断挂起位
    NVIC_SetPriority(CS_24G_RF_IRQn, RTE_CS_24G_IRQ_PRIORITY); // 设置中断优先级
    NVIC_EnableIRQ(CS_24G_RF_IRQn);                            // 使能中断
    REGW(&CS_24G->RF_PD_AHEAD, MASK_2REG(CS_24G_RF_PD_AHEAD_EN, 1, CS_24G_RF_PD_AHEAD, 1));

    static bool used_once = true; // 初始化完成标志
    if (used_once) {
        cs_24g_env.reg_store.KDCO_LUT_2M_REG0 = CS_DAIF->KDCO_LUT_2M_REG0;
        cs_24g_env.reg_store.KDCO_LUT_2M_REG1 = CS_DAIF->KDCO_LUT_2M_REG1;
        used_once                             = false;
    }
    cs_24g_set_rf_parameters(cfg->data_rate, cfg->freq, 0);
}

__RAM_CODES("24G") // 此函数要在RAM中执行
void drv_cs_24g_isr(void)
{
    uint8_t status     = 0;     // 中断状态寄存器值
    uint16_t rx_cnt    = 0;     // 接收到的数据长度
    bool rx_right_flag = false; // 是否成功接收标志

    DRV_IRQ_BEGIN();                 // 标记进入中断
    status         = CS_24G->INT_ST; // 读取中断状态寄存器
    CS_24G->INT_ST = status;         // 写回状态以清除中断标志
    // CS_LOG_DEBUG("status=%02x\r\n", status);
    if ((CS_24G_STATUS_TX_DS & status)) { // 发送完成
        if (cs_24g_env.event_cb) {
            cs_24g_env.event_cb(CS_24G, DRV_EVENT_COMMON_TRANSMIT_COMPLETED, NULL, NULL);
        }
    }
    if (CS_24G_STATUS_MAX_RT & status) { // 数据溢出
        if (cs_24g_env.event_cb) {
            cs_24g_env.event_cb(CS_24G, DRV_EVENT_COMMON_RX_OVERFLOW, NULL, NULL);
        }
    }
    if ((CS_24G_STATUS_RX_DR & status) && (!(CS_24G_STATUS_CRC_ERR & status))) { // 接收成功且校验成功

        CS_CRITICAL_BEGIN(); // 进入临界区
        if ((!(CS_PHY->FYSNC_DET_INFO & PHY_FYSNC_DET_INFO_FYSNC_CFO_EST_MASK)) && (!(CS_24G_STATUS_CRC_ERR & CS_24G->INT_ST))) {
            // 双缓冲切换:交替使用两个缓冲区接收数据,防止数据覆盖
            cs_24g_env.rx_count = (cs_24g_env.rx_count + 1) % 2;
            rx_right_flag       = true;
            rx_cnt              = cs_24g_read_rx_payload_width();
            // 设置下一个接收缓冲区的 DMA 地址
            CS_24G->DMA_RX_ADDR = (uint32_t)(cs_24g_env.rx_buf + cs_24g_env.max_rx_num * cs_24g_env.rx_count);
        } else {
            rx_right_flag = false;
        }
        CS_CRITICAL_END();
        // 通知上层数据接收成功
        if (cs_24g_env.event_cb && rx_right_flag) {
            if (CS_24G->MAC_EN) {
                CS_24G_CE_LOW();
                CS_24G_CE_HIGH();
            }
            // 回调参数:数据缓冲区地址 + 数据长度
            cs_24g_env.event_cb(CS_24G, DRV_EVENT_COMMON_RECEIVE_COMPLETED, (void *)(cs_24g_env.rx_buf + cs_24g_env.max_rx_num * (cs_24g_env.rx_count ? 0 : 1)), (void *)((uint32_t)rx_cnt));
        }
    }

    if (CS_24G_STATUS_TIME_OUT & status) { // ack 接收超时
        REGW1(&CS_24G->INT_ST, CS_24G_STATUS_TIME_OUT);
        if (cs_24g_env.event_cb) {
            cs_24g_env.event_cb(CS_24G, DRV_EVENT_COMMON_GENERAL, NULL, NULL);
        }
    }
    if (CS_24G_STATUS_CRC_ERR & status) { // crc 错误
        if (CS_24G->MAC_EN) {
            CS_24G_CE_LOW(); // CE 脚拉低后拉高,重新激活模块
            CS_24G_CE_HIGH();
        }

        REGW1(&CS_24G->INT_ST, CS_24G_STATUS_CRC_ERR); // 清除中断标志位
    }
    DRV_IRQ_END(); // 标记结束中断
}

/* cs_24g_control() contains drv_pmu_ana_enable(), so turning on or turning off the clock only requires calling cs_24g_control(). */
void *cs_24g_control(cs_24g_control_t control, void *argu)
{
    uint32_t ret;

    ret = (uint32_t)CS_ERROR_OK;

    CS_CRITICAL_BEGIN();
    switch (control) {
        case CS_24G_CONTROL_CLK_DISABLE: {
            DRV_RCC_CLOCK_ENABLE(RCC_CLK_2P4, 1U);
            DRV_RCC_ANA_CLK_ENABLE_NOIRQ();
            CS_24G_CE_LOW();
            CS_24G_FLUSH_TX_FIFO();
            CS_24G_FLUSH_RX_FIFO();
            cs_error_t ret; // Max: 150us
            DRV_WAIT_US_UNTIL_TO(!((REGR(&CS_24G->STATE, MASK_POS(CS_24G_MAIN_STATE)) == 0x02) || (REGR(&CS_24G->STATE, MASK_POS(CS_24G_MAIN_STATE)) == 0x00)), 150, ret);
            (void)ret;
            // Wait for the digital state machine to finish running before turning off the clock.
            REGW(&CS_DAIF->DBG_REG, MASK_2REG(DAIF_DBG_EN, 1, DAIF_DBG_IDX, 3));
            DRV_WAIT_US_UNTIL_TO((REGR(&CS_DAIF->DBG_REG, MASK_POS(DAIF_DBG_DATA)) & 0xF), 30, ret);
            (void)ret;
            REGW(&CS_DAIF->DBG_REG, MASK_1REG(DAIF_DBG_EN, 0));
#if PLL_CONTIMUE_OPEN
            REGW(&CS_DAIF->PD_CFG1, MASK_2REG(DAIF_RFPLL_PD_ALL_ME, 0, DAIF_RFPLL_PD_ALL_MO, 1));
#endif
            REGW0(&CS_24G->PKTCTRL0, CS_24G_PKTCTRL0_MAC_SEL_MASK); // switch phy between 2.4G and ble.
            REGW(&CS_DAIF->FREQ_CFG0, MASK_1REG(DAIF_FREQ_REG_ME, 0));
            REGW1(&CS_DAIF->PLL_CTRL1, DAIF_FREQ_DEVIA_BYPASS_MASK);
            REGW(&CS_DAIF->PD_CFG2, MASK_1REG(DAIF_RFPLL_PD_TXDAC_ME, 0));
            CS_24G->INT_MASK = 0xFF;
            DRV_RCC_ANA_CLK_RESTORE_NOIRQ();
            DRV_RCC_CLOCK_ENABLE(RCC_CLK_2P4, 0U);
            drv_pmu_ana_enable(false, PMU_ANA_RF_24G);
        } break;
        case CS_24G_CONTROL_CLK_ENABLE: {
            drv_pmu_ana_enable(true, PMU_ANA_RF_24G);
            DRV_RCC_CLOCK_ENABLE(RCC_CLK_DAIF, 1U);
            DRV_RCC_CLOCK_ENABLE(RCC_CLK_2P4, 1U);
            CS_24G_CE_LOW();
            REGW1(&CS_24G->PKTCTRL0, CS_24G_PKTCTRL0_MAC_SEL_MASK);
            REGW(&CS_DAIF->FREQ_CFG0, MASK_1REG(DAIF_FREQ_REG_ME, 1));
            if (cs_24g_env.deviation_below_250K) {
                REGW0(&CS_DAIF->PLL_CTRL1, DAIF_FREQ_DEVIA_BYPASS_MASK);
            }
            if (CS_24G->RF_DR == 0xA4A4) { // DATA RATE = CS_24G_RATE_25K
                REGW(&CS_DAIF->PD_CFG2, MASK_2REG(DAIF_RFPLL_PD_TXDAC_ME, 1, DAIF_RFPLL_PD_TXDAC_MO, 1));
            }
            // Block unnecessary interrupts
            CS_24G->INT_MASK = 0x1E;
#if PLL_CONTIMUE_OPEN
            REGW(&CS_DAIF->PD_CFG1, MASK_2REG(DAIF_RFPLL_PD_ALL_ME, 1, DAIF_RFPLL_PD_ALL_MO, 0));
#endif
        } break;
        case CS_24G_CONTROL_RESET: {
            DRV_RCC_RESET(RCC_CLK_2P4);
            if ((uint32_t)argu) {
                DRV_RCC_RESET(RCC_CLK_PHY);
            }
            REGW1(&CS_24G->PKTCTRL0, CS_24G_PKTCTRL0_MAC_SEL_MASK);
        } break;
        case CS_24G_CONTROL_SWITCH_ROLE: {
            cs_24g_switch_role((cs_24g_role_t)((uint32_t)argu));
        } break;
        case CS_24G_CONTROL_DUMP_RF_REGISTER: {
            cs_24g_dump_rf_register();
        } break;
        default:
            break;
    }
    CS_CRITICAL_END();

    return (void *)ret;
}

// true: idle  false: tx busy
bool cs_24g_tx_idle(void)
{
    bool tx_status = false;

    REGW(&CS_DAIF->DBG_REG, MASK_2REG(DAIF_DBG_EN, 1, DAIF_DBG_IDX, 3));
    tx_status = ((REGR(&CS_DAIF->DBG_REG, MASK_POS(DAIF_DBG_DATA)) & 0x20) || (REGR(&CS_24G->STATE, MASK_POS(CS_24G_MAIN_STATE)) == 0x03)) ? false : true;
    REGW(&CS_DAIF->DBG_REG, MASK_1REG(DAIF_DBG_EN, 0));

    return tx_status;
}

bool cs_24g_rx_idle(void)
{
    bool rx_status = false;

    if (!(CS_PHY->FYSNC_DET_INFO & PHY_FYSNC_DET_INFO_FYSNC_CFO_EST_MASK)) {
        rx_status = true;
    }

    return rx_status;
}

/*****************************************************************************************
 * @brief Ripple specific write access
 *
 * @param[in] addr    register address
 * @param[in] value   value to write
 *
 * @return uint32_t value
 ****************************************************************************************/
static void cs_24g_rf_reg_wr(uint32_t addr, uint32_t value)
{
    REGW(&CS_24G->DBG_SPI0, MASK_2REG(CS_24G_DBG_SPI0_ADDRESS, addr, CS_24G_DBG_SPI0_DATA, value));
    REGW(&CS_24G->DBG_SPI1, MASK_1REG(CS_24G_DBG_SPI1_START, 1));
    DRV_DELAY_US(10);
    // Waiting SW driven SPI Access completion
    while (REGR(&CS_24G->DBG_SPI1, MASK_POS(CS_24G_DBG_SPI1_COMP)) == 0);
}

/***************************************************************************************
 * @brief Static Ripple radio Calibration function.
 ***************************************************************************************
 */
static void cs_24g_rf_calib(void)
{
    // Automatic VCO subband selection
    //  1. set D8=0(Addr:0x05) to enable the automatic VCO sub-band selection;
    cs_24g_rf_reg_wr(0x05, 0x1824);
    // 2. Enable the PLL and VCO if required.
    // 3. set D7=1(Addr:0x05) to start the FSM
    cs_24g_rf_reg_wr(0x05, (0x1824 | (1 << 7)));
    // 4. The VCO sub-band selection and PLL setting time takes less than 300us
    DRV_DELAY_US(300);
    // 5. Reset the FSM
    cs_24g_rf_reg_wr(0x05, 0x1824);
}

void cs_24g_rf_init_seq(void)
{
    /****************************
     * MAX2829 Initialise
     ****************************/
    cs_24g_rf_reg_wr(0x00, 0x1140); /*Register 0*/
    cs_24g_rf_reg_wr(0x01, 0x00CA); /*Register 1*/
    cs_24g_rf_reg_wr(0x02, 0x1007); /*standby */
    cs_24g_rf_reg_wr(0x03, 0x30A2); /*Intger-Divider Ratio*/
    cs_24g_rf_reg_wr(0x04, 0x1DDD); /*Fractional-Divider Ratio*/
    cs_24g_rf_reg_wr(0x05, 0x1824); /*Band select and PLL*/
    cs_24g_rf_reg_wr(0x06, 0x1C00); /*calibration*/
    cs_24g_rf_reg_wr(0x07, 0x002A); /*lowpass filter*/
    cs_24g_rf_reg_wr(0x08, 0x1C25); /*Rx control/RSSI*/
    cs_24g_rf_reg_wr(0x09, 0x0603); /*Tx linearity/baseband gain*/
    cs_24g_rf_reg_wr(0x0A, 0x03C0); /*PA bias DAC*/
    cs_24g_rf_reg_wr(0x0B, 0x004B); /*Rx Gain, LNA gain=15dB, VGA=22dB*/
                                    //    cs_24g_rf_reg_wr(0x0B, 0x0006);                    /*Rx Gain, for Shielding Box*/
                                    //    cs_24g_rf_reg_wr(0x0B, 0x007F);                    /*Rx Gain, LNA gain=30dB, VGA=64dB, input loss -22dB*/
    cs_24g_rf_reg_wr(0x0C, 0x003F); /*Tx VGA Gain = 0dB*/

    //    CS_PHY->EN_SDET = 1;
    //    *(volatile uint32_t *)(CS_PHY_BASE + 0xC4) = 0;

    // calibration procedure
    cs_24g_rf_calib();
}

#endif /*RTE_CS24G*/

/*-------------------------------------------End Of File---------------------------------------------*/

/** @} */
