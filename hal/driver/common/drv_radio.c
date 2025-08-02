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
 * @file     radio.c
 * @brief    radio driver
 * @date     14. December 2021
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
#if (RTE_RADIO)
#include <stdint.h>
#include "cs_device.h"
#include "cs_driver.h"

/*******************************************************************************
 * MACROS
 */

/*******************************************************************************
 * TYPEDEFS
 */

/*******************************************************************************
 * CONST & VARIABLES
 */

/*******************************************************************************
 * LOCAL FUNCTIONS
 */

/**
 *******************************************************************************
 * @brief  drv rf init
 *******************************************************************************
 */
void drv_rf_init(void)
{
    drv_rf_tx_power_set(false, RF_TX_POWER_8DBM); // 设置发射功率

    drv_calib_repair_init(); // 校准与补偿机制的初始化

    drv_calib_rf(); // 射频校准
}

/**
 *******************************************************************************
 * @brief  rf txrx pin enable
 *
 * @param[in] enable  enable
 * @param[in] pol  polarity, 0 or 1
 *******************************************************************************
 **/
void drv_rf_txrx_pin_enable(bool enable, int pol)
{
    DRV_RCC_ANA_CLK_ENABLE();

    REGW(&CS_DAIF->TRX_EXT_PD_CFG, MASK_2REG(DAIF_TX_EXT_PD_EN, 0, DAIF_RX_EXT_PD_EN, 0));

    if (enable) {
        // Digital issue: DAIF_TX_EXT_PD_POL REG must open follow clock
        uint32_t clk_ens_save = CS_DAIF->CLK_ENS;
        REGW1(&CS_DAIF->CLK_ENS, DAIF_PLL_CLK_REF_EN_MASK | DAIF_MAIN_FSM_CLK_EN_MASK);
        uint32_t clk_cfg_save = CS_DAIF->CLK_CFG;
        REGW1(&CS_DAIF->CLK_CFG, DAIF_XTAL32M_EN_CKO16M_DIG_MASK);
        // setup pol
        REGW(&CS_DAIF->TRX_EXT_PD_CFG, MASK_4REG(DAIF_TX_EXT_PD_POL, pol, DAIF_RX_EXT_PD_POL, pol, DAIF_TX_EXT_PD_TCFG, 0, DAIF_RX_EXT_PD_TCFG, 0));
        // restore clock
        CS_DAIF->CLK_ENS = clk_ens_save;
        CS_DAIF->CLK_CFG = clk_cfg_save;

        // Enable
        REGW1(&CS_DAIF->TRX_EXT_PD_CFG, DAIF_TX_EXT_PD_EN_MASK | DAIF_RX_EXT_PD_EN_MASK);
    }

    DRV_RCC_ANA_CLK_RESTORE();
}

/**
 *******************************************************************************
 * @brief  rf carrier enable
 *
 * @param[in] enable  enable
 * @param[in] freq  2402MHz ... 2480MHz, If set freq to 2402.123456MHZ, freq_channel = 2402, fractFreq = 0.123456
 *******************************************************************************
 **/
void drv_rf_carrier_enable(bool enable, uint32_t freq, float fract_freq)
{
    if (enable) {
        drv_pmu_ana_enable(enable, PMU_ANA_RF);

        DRV_RCC_ANA_CLK_ENABLE();

        // frequency
        REGW(&CS_DAIF->FREQ_CFG0, MASK_2REG(DAIF_FREQ_REG_ME, 1, DAIF_FREQ_REG_MO, freq));
        if (fract_freq) {
            REGW(&CS_DAIF->FREQ_CFG3, MASK_2REG(DAIF_FREQ_FRAC_REG, (uint32_t)(fract_freq * 0x3FFFFF), DAIF_FREQ_0P5_EN, 1));
        } else {
            REGW(&CS_DAIF->FREQ_CFG3, MASK_2REG(DAIF_FREQ_FRAC_REG, 0, DAIF_FREQ_0P5_EN, 0));
        }

        // SDM
        REGW(&CS_DAIF->PLL_CTRL1, MASK_2REG(DAIF_DIGI_DIN_BYPASS, 1, DAIF_DIGI_DIN_REG, 0));
        //        REGW(&CS_DAIF->PLL_CTRL2, MASK_2REG(DAIF_DIN_SDM_TX_ME, 1, DAIF_DATA_SYNC_BYPASS, 1));

        // do TX
        REGW(&CS_DAIF->VCO_CTRL0, MASK_3REG(DAIF_TRX_DBG, 1, DAIF_RX_EN_MO, 0, DAIF_TX_EN_MO, 0));
        DRV_DELAY_US(100);
        REGW(&CS_DAIF->VCO_CTRL0, MASK_3REG(DAIF_TRX_DBG, 1, DAIF_RX_EN_MO, 0, DAIF_TX_EN_MO, 1));

        DRV_RCC_ANA_CLK_RESTORE();
    } else {
        DRV_RCC_ANA_CLK_ENABLE();
        REGW(&CS_DAIF->FREQ_CFG0, MASK_1REG(DAIF_FREQ_REG_ME, 0));
        REGW(&CS_DAIF->PLL_CTRL1, MASK_2REG(DAIF_DIGI_DIN_BYPASS, 0, DAIF_DIGI_DIN_REG, 0));
        //        REGW(&CS_DAIF->PLL_CTRL2, MASK_2REG(DAIF_DIN_SDM_TX_ME, 0, DAIF_DATA_SYNC_BYPASS, 0));
        REGW(&CS_DAIF->VCO_CTRL0, MASK_3REG(DAIF_TRX_DBG, 1, DAIF_RX_EN_MO, 0, DAIF_TX_EN_MO, 0));
        //        REGW(&CS_DAIF->PD_CFG0, MASK_4REG(DAIF_PD_LDO_PA_ME,1, DAIF_PD_LDO_PA_MO,1,DAIF_PD_PA_ME,1,DAIF_PD_PA_MO,1));
        DRV_RCC_ANA_CLK_RESTORE();

        drv_pmu_ana_enable(enable, PMU_ANA_RF);
    }
}

/**
 *******************************************************************************
 * @brief  rf single tone enable
 *
 * @param[in] enable  enable
 * @param[in] freq  freq
 * @param[in] payload  payload (0-255)
 *******************************************************************************
 */
void drv_rf_single_tone_enable(bool enable, uint32_t freq, uint8_t payload)
{
    if (enable) {
        DRV_RCC_CLOCK_ENABLE(RCC_CLK_2P4, 1U);
        DRV_RCC_CLOCK_ENABLE(RCC_CLK_PHY, 1U);
        drv_pmu_ana_enable(true, PMU_ANA_RF);
        REGW1(&CS_24G->PKTCTRL0, CS_24G_PKTCTRL0_MAC_SEL_MASK);

        REGW0(&CS_PHY->TX_CTRL0, PHY_TX_CTRL0_BP_GAU_MASK);                  // GFSK
        REGW0(&CS_PHY->REG_PHY_RST_N, PHY_REG_PHY_RST_N_REG_PHY_RST_N_MASK); // reset phy
        REGW(&CS_24G->TESTCTRL, MASK_1REG(CS_24G_CONT_WAVE, 1));
        REGW(&CS_DAIF->FREQ_CFG0, MASK_2REG(DAIF_FREQ_REG_MO, freq, DAIF_FREQ_REG_ME, 1));
        REGW(&CS_24G->TESTCTRL, MASK_1REG(CS_24G_TEST_PAT_EN, 1));
        REGW(&CS_24G->TESTCTRL, MASK_1REG(CS_24G_TEST_PAT, payload));
        // CE High
        CS_24G_CE_HIGH();
    } else {
        // CE Low
        CS_24G_CE_LOW();
        // diable single tone
        REGW1(&CS_PHY->REG_PHY_RST_N, PHY_REG_PHY_RST_N_REG_PHY_RST_N_MASK); // reset phy
        REGW(&CS_24G->TESTCTRL, MASK_1REG(CS_24G_CONT_WAVE, 0));
        REGW(&CS_24G->TESTCTRL, MASK_1REG(CS_24G_TEST_PAT_EN, 0));

        drv_pmu_ana_enable(false, PMU_ANA_RF);
        REGW0(&CS_24G->PKTCTRL0, CS_24G_PKTCTRL0_MAC_SEL_MASK);
        DRV_RCC_CLOCK_ENABLE(RCC_CLK_2P4, 0U);
        DRV_RCC_CLOCK_ENABLE(RCC_CLK_PHY, 0U);
    }
}

/**
 *******************************************************************************
 * @brief  rf full rx enable
 *
 * @param[in] enable  enable
 * @param[in] freq  2402MHz ... 2480MHz
 *******************************************************************************
 **/
void drv_rf_full_rx_enable(bool enable, uint32_t freq)
{
    if (enable) {
        drv_pmu_ana_enable(enable, PMU_ANA_RF);

        DRV_RCC_ANA_CLK_ENABLE();

        // frequency
        REGW(&CS_DAIF->FREQ_CFG0, MASK_2REG(DAIF_FREQ_REG_ME, 1, DAIF_FREQ_REG_MO, freq));

        // Do RX
        REGW(&CS_DAIF->VCO_CTRL0, MASK_3REG(DAIF_TRX_DBG, 1, DAIF_RX_EN_MO, 0, DAIF_TX_EN_MO, 0));
        DRV_DELAY_US(100);
        REGW(&CS_DAIF->VCO_CTRL0, MASK_3REG(DAIF_TRX_DBG, 1, DAIF_RX_EN_MO, 1, DAIF_TX_EN_MO, 0));

        DRV_RCC_ANA_CLK_RESTORE();
    } else {
        DRV_RCC_ANA_CLK_ENABLE();
        REGW(&CS_DAIF->FREQ_CFG0, MASK_1REG(DAIF_FREQ_REG_ME, 0));
        REGW(&CS_DAIF->VCO_CTRL0, MASK_3REG(DAIF_TRX_DBG, 0, DAIF_RX_EN_MO, 0, DAIF_TX_EN_MO, 0));
        DRV_RCC_ANA_CLK_RESTORE();

        drv_pmu_ana_enable(enable, PMU_ANA_RF);
    }
}

/**
 *******************************************************************************
 * @brief  rf tx power set
 *
 * @param[in] auto_ctrl_by_ble  deprecated and must be set to 'false'
 * @param[in] power  power
 *******************************************************************************
 **/
void drv_rf_tx_power_set(bool auto_ctrl_by_ble, rf_tx_power_t power)
{
    bool high_tx_power_mode = false;
    uint32_t pmu_ldo_v1p2_vbat;
    uint32_t pmu_dcdc_vout;

    DRV_RCC_ANA_CLK_ENABLE();

    if (auto_ctrl_by_ble) {
        // DBG=1: MO_REG ctrl PA
        // DBG=0: SEL ctrl PA
        //   SEL=0: REG_TABLE ctrl PA
        //   SEL=1: RW ctrl PA
        REGW(&CS_DAIF->PA_CNS, MASK_2REG(DAIF_PA_DBG, 0, DAIF_PA_GAIN_IDX_REG, 1));
    } else {
        // FIXME: NOT 3DBM
        if (power >= RF_TX_POWER_3DBM) {
            switch (power) {
                case RF_TX_POWER_8P5DBM:
                    REGW(&CS_DAIF->PA_CNS, MASK_2REG(DAIF_EN_BYPASS_PALDO, 1, DAIF_PA_GAIN_IDX_REG, 18));
                    pmu_dcdc_vout      = drv_calib_repair_env.dcdc_vout + DRV_CALIB_REPAIR_DCDC_MAXDBM_DELTA;
                    pmu_ldo_v1p2_vbat  = drv_calib_repair_env.ana_ldo + DRV_CALIB_REPAIR_LDO_MAXDBM_DELTA;
                    high_tx_power_mode = true;
                    break;

                case RF_TX_POWER_8DBM:
                    REGW(&CS_DAIF->PA_CNS, MASK_2REG(DAIF_EN_BYPASS_PALDO, 1, DAIF_PA_GAIN_IDX_REG, 18));
                    pmu_dcdc_vout      = drv_calib_repair_env.dcdc_vout + DRV_CALIB_REPAIR_DCDC_8DBM_DELTA;
                    pmu_ldo_v1p2_vbat  = drv_calib_repair_env.ana_ldo + DRV_CALIB_REPAIR_LDO_8DBM_DELTA;
                    high_tx_power_mode = true;
                    break;

                case RF_TX_POWER_7DBM:
                    REGW(&CS_DAIF->PA_CNS, MASK_2REG(DAIF_EN_BYPASS_PALDO, 1, DAIF_PA_GAIN_IDX_REG, 18));
                    pmu_dcdc_vout      = drv_calib_repair_env.dcdc_vout + DRV_CALIB_REPAIR_DCDC_7DBM_DELTA;
                    pmu_ldo_v1p2_vbat  = drv_calib_repair_env.ana_ldo + DRV_CALIB_REPAIR_LDO_7DBM_DELTA;
                    high_tx_power_mode = true;
                    break;

                case RF_TX_POWER_6DBM:
                    REGW(&CS_DAIF->PA_CNS, MASK_2REG(DAIF_EN_BYPASS_PALDO, 1, DAIF_PA_GAIN_IDX_REG, 15));
                    pmu_dcdc_vout      = drv_calib_repair_env.dcdc_vout;
                    pmu_ldo_v1p2_vbat  = drv_calib_repair_env.ana_ldo;
                    high_tx_power_mode = true;
                    break;

                case RF_TX_POWER_5P5DBM:
                    REGW(&CS_DAIF->PA_CNS, MASK_2REG(DAIF_EN_BYPASS_PALDO, 1, DAIF_PA_GAIN_IDX_REG, 13));
                    pmu_dcdc_vout      = drv_calib_repair_env.dcdc_vout;
                    pmu_ldo_v1p2_vbat  = drv_calib_repair_env.ana_ldo;
                    high_tx_power_mode = true;
                    break;

                default:
                    REGW(&CS_DAIF->PA_CNS, MASK_2REG(DAIF_EN_BYPASS_PALDO, 0, DAIF_PA_GAIN_IDX_REG, power));
                    break;
            }
        } else {
            REGW(&CS_DAIF->PA_CNS, MASK_2REG(DAIF_EN_BYPASS_PALDO, 0, DAIF_PA_GAIN_IDX_REG, power));
        }
    }

    if (!high_tx_power_mode) {
        if (drv_calib_repair_env.temperature > 70) {
            pmu_dcdc_vout     = drv_calib_repair_env.dcdc_vout + DRV_CALIB_REPAIR_DCDC_HIGHT_T_DELTA;
            pmu_ldo_v1p2_vbat = drv_calib_repair_env.ana_ldo + DRV_CALIB_REPAIR_LDO_HIGHT_T_DELTA;
        } else {
            pmu_dcdc_vout     = drv_calib_repair_env.dcdc_vout;
            pmu_ldo_v1p2_vbat = drv_calib_repair_env.ana_ldo;
        }
    }

    REGW(&CS_PMU->ANA_PD_1, MASK_1REG(PMU_ANA_PD_1_DCDC_VOUT, pmu_dcdc_vout));
    REGW(&CS_PMU->ANA_PD, MASK_1REG(PMU_ANA_PD_PMU_LDO_ANA1P2_TRIM, pmu_ldo_v1p2_vbat));

    DRV_RCC_ANA_CLK_RESTORE();
}

#endif /* RTE_RADIO */

/** @} */
