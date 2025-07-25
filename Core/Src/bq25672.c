/*
 * bq25672.c
 *
 *  Created on: Jul 9, 2025
 *      Author: YED
 */

#include "main.h"
#include "stm32g0xx_hal.h"
// Write function (asynchronous)
bq25672_status_t write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val)
{
    i2c_tx_complete = 0; // Reset completion flag
    i2c_error = 0;       // Reset error flag

    if (HAL_I2C_Mem_Write_DMA(hi2c, BQ25672_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &val, 1) != HAL_OK) {
        return BQ25672_COMM_FAIL;
    }

    return BQ25672_OK;
}

// Read function (asynchronous)
bq25672_status_t read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t* val)
{
    i2c_rx_complete = 0; // Reset completion flag
    i2c_error = 0;       // Reset error flag

    if (HAL_I2C_Mem_Read_DMA(hi2c, BQ25672_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, val, 1) != HAL_OK) {
        return BQ25672_COMM_FAIL;
    }

    return BQ25672_OK;
}

/**
 * @brief Configure charger system-level features (ICO, HIZ, Termination, ACDRV, Freq)
 *
 * Write system charger options via REG0F, REG12, REG13.
 * Controls:
 * - EN_TERM (Charge Termination Detection)
 * - EN_HIZ (Input High-Z Mode)
 * - FORCE_ICO / EN_ICO (Input Current Optimizer control)
 * - Disable PFM / LDO / OOA modes
 * - ACDRV1 / ACDRV2 FET drivers
 * - PWM frequency 750kHz / 1.5MHz
 *
 * @param hi2c I2C handle
 * @param cfg Configuration parameters
 * @retval BQ25672_OK success
 * @retval BQ25672_COMM_FAIL communication error
 */
bq25672_status_t bq25672_set_system_config(I2C_HandleTypeDef *hi2c, const bq25672_system_config_t *cfg)
{
    // REG0F: Charger Control 0
    uint8_t reg0f = 0;
    reg0f |= (cfg->enable_termination ? BQ25672_EN_TERM_MASK : 0U); // Bit 2: EN_TERM, enables charge termination
    reg0f |= (cfg->enable_hiz ? BQ25672_EN_HIZ_MASK : 0U);          // Bit 1: EN_HIZ, enables High Impedance input disconnect
    reg0f |= (cfg->force_ico ? BQ25672_FORCE_ICO_MASK : 0U);        // Bit 5: FORCE_ICO, forces ICO run
    reg0f |= (cfg->enable_ico ? BQ25672_EN_ICO_MASK : 0U);          // Bit 3: EN_ICO, enables ICO tracking

    if (write_reg(hi2c, REG0F_Charger_Control_0, reg0f) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    // REG12: Charger Control 3
    uint8_t reg12 = 0;
    reg12 |= (cfg->disable_pfm ? BQ25672_PFM_FWD_DIS_MASK : 0U);    // Bit 4: PFM_FWD_DIS, disables PFM
    reg12 |= (cfg->disable_ldo ? BQ25672_DIS_LDO_MASK : 0U);        // Bit 2: DIS_LDO, disables internal LDO
    reg12 |= (cfg->disable_ooa ? BQ25672_DIS_FWD_OOA_MASK : 0U);    // Bit 0: DIS_FWD_OOA, disables Out-of-Audible

    if (write_reg(hi2c, REG12_Charger_Control_3, reg12) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    // REG13: Charger Control 4
    uint8_t reg13 = 0;
    reg13 |= (cfg->use_1500kHz ? BQ25672_PWM_FREQ_MASK : 0U);       // Bit 5: PWM_FREQ, 1 = 1.5MHz, 0 = 750kHz
    reg13 |= (cfg->enable_acdrv1 ? BQ25672_EN_ACDRV1_MASK : 0U);    // Bit 6: EN_ACDRV1, enables ACDRV1 driver
    reg13 |= (cfg->enable_acdrv2 ? BQ25672_EN_ACDRV2_MASK : 0U);    // Bit 7: EN_ACDRV2, enables ACDRV2 driver

    return write_reg(hi2c, REG13_Charger_Control_4, reg13);
}

/**
 * @brief Read system-level charger configuration (ICO, HIZ, Termination, ACDRV, Freq)
 *
 * Reads BQ25672 Charger Control registers (REG0F, REG12, REG13) and updates user config.
 * This reflects hardware status for:
 * - Charge termination detection (EN_TERM)
 * - Input High Impedance (EN_HIZ)
 * - Input Current Optimizer (FORCE_ICO, EN_ICO)
 * - PFM, LDO, OOA modes
 * - ACDRV1 / ACDRV2 drivers
 * - PWM frequency (750kHz or 1.5MHz)
 *
 * @param hi2c I2C handle
 * @param cfg  Output configuration structure
 * @retval BQ25672_OK success
 * @retval BQ25672_COMM_FAIL communication error
 */
bq25672_status_t bq25672_get_system_config(I2C_HandleTypeDef *hi2c, bq25672_system_config_t *cfg)
{
    uint8_t reg0f = 0;
    if (read_reg(hi2c, REG0F_Charger_Control_0, &reg0f) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    cfg->enable_termination = !!(reg0f & BQ25672_EN_TERM_MASK);    // Bit 2: EN_TERM, enables termination
    cfg->enable_hiz         = !!(reg0f & BQ25672_EN_HIZ_MASK);     // Bit 1: EN_HIZ, enables High-Z input
    cfg->force_ico          = !!(reg0f & BQ25672_FORCE_ICO_MASK);  // Bit 5: FORCE_ICO, forces ICO
    cfg->enable_ico         = !!(reg0f & BQ25672_EN_ICO_MASK);     // Bit 3: EN_ICO, enables ICO tracking

    uint8_t reg12 = 0;
    if (read_reg(hi2c, REG12_Charger_Control_3, &reg12) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    cfg->disable_pfm = !!(reg12 & BQ25672_PFM_FWD_DIS_MASK);       // Bit 4: PFM_FWD_DIS disables PFM
    cfg->disable_ldo = !!(reg12 & BQ25672_DIS_LDO_MASK);           // Bit 2: DIS_LDO disables LDO
    cfg->disable_ooa = !!(reg12 & BQ25672_DIS_FWD_OOA_MASK);       // Bit 0: DIS_FWD_OOA disables OOA

    uint8_t reg13 = 0;
    if (read_reg(hi2c, REG13_Charger_Control_4, &reg13) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    cfg->use_1500kHz   = !!(reg13 & BQ25672_PWM_FREQ_MASK);        // Bit 5: PWM_FREQ sets 1.5MHz / 750kHz
    cfg->enable_acdrv1 = !!(reg13 & BQ25672_EN_ACDRV1_MASK);       // Bit 6: EN_ACDRV1 enables ACDRV1
    cfg->enable_acdrv2 = !!(reg13 & BQ25672_EN_ACDRV2_MASK);       // Bit 7: EN_ACDRV2 enables ACDRV2

    return BQ25672_OK;
}

/**
 * @brief Configure charge parameters: charge voltage, current, input limits, termination current.
 *
 * This function writes to BQ25672 charge-related registers:
 * - REG01 / REG02: Charge Voltage Limit
 * - REG03 / REG04: Charge Current Limit
 * - REG05: Input Voltage Limit
 * - REG06: Input Current Limit
 * - REG09: Termination Current / Precharge Threshold
 *
 * @param hi2c I2C handle
 * @param cfg  Pointer to user config struct for charging limits
 * @retval BQ25672_OK on success
 */
bq25672_status_t bq25672_set_charge_config(I2C_HandleTypeDef *hi2c, const bq25672_charge_config_t *cfg)
{
    // REG01 / REG02: Charge Voltage Limit (10mV step)
    uint16_t vreg = (cfg->charge_voltage_max_mv / 10U);
    uint8_t reg01 = (uint8_t)((vreg & 0xFFU) << BQ25672_VREG_SHIFT);       // LSB: VREG[7:0]
    uint8_t reg02 = (uint8_t)(((vreg >> 8U) & 0x07U) << BQ25672_VREG_SHIFT); // MSB: VREG[10:8]

    if (write_reg(hi2c, REG01_Charge_Voltage_Limit_L, reg01) != BQ25672_OK) return BQ25672_COMM_FAIL;
    if (write_reg(hi2c, REG02_Charge_Voltage_Limit_H, reg02) != BQ25672_OK) return BQ25672_COMM_FAIL;

    // REG03 / REG04: Charge Current Limit (10mA step, 9-bit total)
    uint16_t ichg = (cfg->charge_current_max_ma / 10U);
    uint8_t reg03 = (uint8_t)((ichg & 0xFFU) << BQ25672_ICHG_SHIFT);       // LSB: ICHG[7:0]
    uint8_t reg04 = (uint8_t)(((ichg >> 8U) & 0x01U) << BQ25672_ICHG_SHIFT); // MSB: ICHG[8]

    if (write_reg(hi2c, REG03_Charge_Current_Limit_L, reg03) != BQ25672_OK) return BQ25672_COMM_FAIL;
    if (write_reg(hi2c, REG04_Charge_Current_Limit_H, reg04) != BQ25672_OK) return BQ25672_COMM_FAIL;

    // REG05: Input Voltage Limit (100mV step starting from 3.6V)
    uint8_t vindpm = (uint8_t)(((cfg->advanced.input_voltage_max_mv - 3600U) / 100U) << BQ25672_VINDPM_SHIFT);
    if (write_reg(hi2c, REG05_Input_Voltage_Limit, (vindpm & BQ25672_VINDPM_MASK)) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    // REG06: Input Current Limit (10mA step)
    uint8_t iindpm = (uint8_t)((cfg->advanced.input_current_max_ma / 10U) << BQ25672_IINDPM_SHIFT);
    if (write_reg(hi2c, REG06_Input_Current_Limit, (iindpm & BQ25672_IINDPM_MASK)) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    // REG09: Termination Current, Precharge Threshold
    uint8_t iterm = (uint8_t)((cfg->advanced.termination_current_ma / 40U) << BQ25672_ITERM_SHIFT);    // 40mA step
    uint8_t prechg = (uint8_t)((cfg->advanced.precharge_pct & 0x03U) << BQ25672_VBAT_LOWV_SHIFT);
    uint8_t reg09 = (iterm & BQ25672_ITERM_MASK) | (prechg & BQ25672_VBAT_LOWV_MASK);

    if (write_reg(hi2c, REG09_Termination_Control, reg09) != BQ25672_OK) return BQ25672_COMM_FAIL;

    // REG00: Minimum System Voltage (250mV step starting from 2.5V)
    uint8_t vsysmin = (uint8_t)(((cfg->advanced.input_voltage_min_mv - 2500U) / 250U) << BQ25672_VSYSMIN_SHIFT);
    if (write_reg(hi2c, REG00_Minimal_System_Voltage, (vsysmin & BQ25672_VSYSMIN_MASK)) != BQ25672_OK) return BQ25672_COMM_FAIL;

    return BQ25672_OK;
}

/**
 * @brief Get charge parameters: charge voltage, current, input limits, termination current.
 *
 * This function writes to BQ25672 charge-related registers:
 * - REG01 / REG02: Charge Voltage Limit
 * - REG03 / REG04: Charge Current Limit
 * - REG05: Input Voltage Limit
 * - REG06: Input Current Limit
 * - REG09: Termination Current / Precharge Threshold
 *
 * @param hi2c I2C handle
 * @param cfg  Pointer to user config struct for charging limits
 * @retval BQ25672_OK on success
 */
bq25672_status_t bq25672_get_charge_config(I2C_HandleTypeDef *hi2c, bq25672_charge_config_t *cfg)
{
    uint8_t val;
    bq25672_status_t status;

    // REG01 / REG02: Charge Voltage Limit (10mV step, 12-bit total)
    uint16_t vreg = 0;
    status = read_reg(hi2c, REG01_Charge_Voltage_Limit_L, &val);              // Read LSB VREG[7:0]
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;
    vreg |= ((uint16_t)(val & 0xFFU));

    status = read_reg(hi2c, REG02_Charge_Voltage_Limit_H, &val);              // Read MSB VREG[10:8]
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;
    vreg |= ((uint16_t)(val & 0x07U) << 8U);

    cfg->charge_voltage_max_mv = vreg * 10U;                                  // VREG = (reg value * 10mV)

    // REG03 / REG04: Charge Current Limit (10mA step, 9-bit total)
    uint16_t ichg = 0;
    status = read_reg(hi2c, REG03_Charge_Current_Limit_L, &val);              // Read LSB ICHG[7:0]
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;
    ichg |= ((uint16_t)(val & 0xFFU));

    status = read_reg(hi2c, REG04_Charge_Current_Limit_H, &val);              // Read MSB ICHG[8]
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;
    ichg |= ((uint16_t)(val & 0x01U) << 8U);

    cfg->charge_current_max_ma = ichg * 10U;                                  // ICHG = (reg value * 10mA)

    // REG05: Input Voltage Limit (100mV step starting from 3.6V)
    status = read_reg(hi2c, REG05_Input_Voltage_Limit, &val);                 // Read VINDPM
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;
    cfg->advanced.input_voltage_max_mv = 3600U + (((val & BQ25672_VINDPM_MASK) >> BQ25672_VINDPM_SHIFT) * 100U);

    // REG06: Input Current Limit (10mA step)
    status = read_reg(hi2c, REG06_Input_Current_Limit, &val);                 // Read IINDPM
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;
    cfg->advanced.input_current_max_ma = ((val & BQ25672_IINDPM_MASK) >> BQ25672_IINDPM_SHIFT) * 10U;

    // REG09: Termination Current, Precharge Threshold
    status = read_reg(hi2c, REG09_Termination_Control, &val);                 // Read ITERM, VBAT_LOWV
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;

    cfg->advanced.termination_current_ma = ((val & BQ25672_ITERM_MASK) >> BQ25672_ITERM_SHIFT) * 40U;
    cfg->advanced.precharge_pct = (bq25672_precharge_pct_t)((val & BQ25672_VBAT_LOWV_MASK) >> BQ25672_VBAT_LOWV_SHIFT);

    // REG00: Minimum System Voltage (250mV step starting from 2.5V)
    status = read_reg(hi2c, REG00_Minimal_System_Voltage, &val);              // Read VSYSMIN
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;
    cfg->advanced.input_voltage_min_mv = 2500U + (((val & BQ25672_VSYSMIN_MASK) >> BQ25672_VSYSMIN_SHIFT) * 250U);

    return BQ25672_OK;
}

/**
 * @brief    Configure BQ25672 USB-OTG output voltage, current and behavior.
 *
 * This function sets the OTG output voltage (2800mV - 22000mV), current limit (0 - 3360mA),
 * disables/enables PFM and OOA, and enables OTG function if requested.
 *
 * Registers affected:
 * - REG0B: OTG Output Voltage Regulation
 * - REG0D: OTG Output Current Regulation
 * - REG12: Charger Control 3 (PFM / OOA disable)
 * - REG0F: Charger Control 0 (OTG enable)
 *
 * @param[in] hi2c   Pointer to I2C peripheral handle.
 * @param[in] cfg    Pointer to USB configuration structure.
 *
 * @retval BQ25672_OK          Operation successful.
 * @retval BQ25672_COMM_FAIL    I2C communication failure.
 */
bq25672_status_t bq25672_set_usb_config(I2C_HandleTypeDef *hi2c, const bq25672_usb_config_t *cfg)
{
    // REG0B: OTG Voltage Regulation (Offset 2800mV, 10mV step)
    uint8_t votg = (uint8_t)(((cfg->voltage_mv - 2800U) / 10U) & BQ25672_VOTG_MASK); // VOTG = (Vout - 2800) / 10mV

    if (write_reg(hi2c, REG0B_VOTG_Regulation, votg) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    // REG0D: OTG Current Regulation (0mA offset, 40mA step)
    uint8_t iotg = (uint8_t)((cfg->current_max_ma / 40U) & BQ25672_IOTG_MASK);        // IOTG = Iout / 40mA

    if (write_reg(hi2c, REG0D_IOTG_Regulation, iotg) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    // REG12: Disable PFM / Disable OOA
    uint8_t reg12 = 0;
    reg12 |= (cfg->advanced.disable_pfm ? BQ25672_PFM_OTG_DIS_MASK : 0U);  // Disable PFM if requested
    reg12 |= (cfg->advanced.disable_ooa ? BQ25672_DIS_OTG_OOA_MASK : 0U);  // Disable OOA if requested

    if (write_reg(hi2c, REG12_Charger_Control_3, reg12) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    // REG0F: OTG Enable
    uint8_t reg0f = 0;
    reg0f |= (cfg->enable_otg ? BQ25672_EN_OTG_MASK : 0U); // Enable OTG if requested

    return write_reg(hi2c, REG0F_Charger_Control_0, reg0f);
}

/**
 * @brief    Read BQ25672 current USB-OTG output voltage, current and behavior configuration.
 *
 * This function reads the OTG output voltage, current limit, PFM/OOA disable states,
 * and whether OTG function is enabled.
 *
 * Registers read:
 * - REG0B: OTG Output Voltage Regulation
 * - REG0D: OTG Output Current Regulation
 * - REG12: Charger Control 3 (PFM / OOA disable)
 * - REG0F: Charger Control 0 (OTG enable)
 *
 * @param[in]  hi2c   Pointer to I2C peripheral handle.
 * @param[out] cfg    Pointer to structure where current configuration will be stored.
 *
 * @retval BQ25672_OK          Operation successful.
 * @retval BQ25672_COMM_FAIL    I2C communication failure.
 */
bq25672_status_t bq25672_get_usb_config(I2C_HandleTypeDef *hi2c, bq25672_usb_config_t *cfg)
{
    uint8_t val;
    bq25672_status_t status;

    // REG0B: Read OTG Output Voltage
    status = read_reg(hi2c, REG0B_VOTG_Regulation, &val);
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;
    cfg->voltage_mv = 2800U + ((val & BQ25672_VOTG_MASK) * 10U); // VOTG = 2800 + (reg * 10mV)

    // REG0D: Read OTG Output Current
    status = read_reg(hi2c, REG0D_IOTG_Regulation, &val);
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;
    cfg->current_max_ma = (val & BQ25672_IOTG_MASK) * 40U;        // IOTG = reg * 40mA

    // REG12: Read PFM / OOA Disable States
    status = read_reg(hi2c, REG12_Charger_Control_3, &val);
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;
    cfg->advanced.disable_pfm = !!(val & BQ25672_PFM_OTG_DIS_MASK);
    cfg->advanced.disable_ooa = !!(val & BQ25672_DIS_OTG_OOA_MASK);

    // REG0F: Read OTG Enable State
    status = read_reg(hi2c, REG0F_Charger_Control_0, &val);
    if (status != BQ25672_OK) return BQ25672_COMM_FAIL;
    cfg->enable_otg = !!(val & BQ25672_EN_OTG_MASK);

    return BQ25672_OK;
}

/**
 * @brief   Configure BQ25672 charge timing parameters.
 *
 * Configures BQ25672 timers for Top-off, Trickle, Fast-Charge, Pre-Charge and slow timer options.
 * Timer settings protect battery by limiting each charging phase duration.
 *
 * Affected Register:
 * - REG0E: Timer Control
 *
 * @param[in] hi2c  Pointer to I2C peripheral handle.
 * @param[in] cfg   Pointer to timer configuration structure.
 *
 * @retval BQ25672_OK          Operation successful.
 * @retval BQ25672_COMM_FAIL    I2C communication failure.
 */
bq25672_status_t bq25672_set_timer_config(I2C_HandleTypeDef *hi2c, const bq25672_timer_config_t *cfg)
{
    uint8_t reg = 0;

    reg |= ((cfg->top_off_time << BQ25672_TOP_OFF_TIMER_SHIFT) & BQ25672_TOP_OFF_TIMER_MASK);  // Bits 7:6 Top-off timer duration
    reg |= (cfg->trickle_enable       ? BQ25672_TRICKLE_TIMER_ENABLE_MASK     : 0U);           // Bit 5 Trickle enable
    reg |= (cfg->advanced.precharge_enable ? BQ25672_PRECHARGE_TIMER_ENABLE_MASK : 0U);        // Bit 4 Pre-charge enable
    reg |= (cfg->fast_charge_enable   ? BQ25672_CHARGE_TIMER_ENABLE_MASK      : 0U);           // Bit 3 Fast-charge enable
    reg |= ((cfg->fast_charge_time << BQ25672_FAST_CHARGE_TIMER_SHIFT) & BQ25672_FAST_CHARGE_TIMER_MASK); // Bits 2:1 Fast-charge duration
    reg |= (cfg->advanced.timer_slow  ? BQ25672_SLOW_TIMERS_MASK              : 0U);           // Bit 0 Slow timers enable

    return write_reg(hi2c, REG0E_Timer_Control, reg);
}

/**
 * @brief   Read BQ25672 current charge timing configurations.
 *
 * Reads Top-off, Trickle, Fast-Charge, Pre-Charge and slow timer enable status and durations.
 *
 * Affected Register:
 * - REG0E: Timer Control
 *
 * @param[in]  hi2c  Pointer to I2C peripheral handle.
 * @param[out] cfg   Pointer to timer configuration structure.
 *
 * @retval BQ25672_OK          Operation successful.
 * @retval BQ25672_COMM_FAIL    I2C communication failure.
 */
bq25672_status_t bq25672_get_timer_config(I2C_HandleTypeDef *hi2c, bq25672_timer_config_t *cfg)
{
    uint8_t val;
    bq25672_status_t status;

    status = read_reg(hi2c, REG0E_Timer_Control, &val);
    if (status != BQ25672_OK)		return BQ25672_COMM_FAIL;

    cfg->top_off_time               = (bq25672_topoff_time_t)((val & BQ25672_TOP_OFF_TIMER_MASK) >> BQ25672_TOP_OFF_TIMER_SHIFT);
    cfg->trickle_enable             = !!(val & BQ25672_TRICKLE_TIMER_ENABLE_MASK);
    cfg->advanced.precharge_enable  = !!(val & BQ25672_PRECHARGE_TIMER_ENABLE_MASK);
    cfg->fast_charge_enable         = !!(val & BQ25672_CHARGE_TIMER_ENABLE_MASK);
    cfg->fast_charge_time           = (bq25672_fastchg_time_t)((val & BQ25672_FAST_CHARGE_TIMER_MASK) >> BQ25672_FAST_CHARGE_TIMER_SHIFT);
    cfg->advanced.timer_slow        = !!(val & BQ25672_SLOW_TIMERS_MASK);

    return BQ25672_OK;
}

/**
 * @brief   Configure BQ25672 I2C Watchdog timer behavior.
 *
 * This function configures:
 * - Watchdog timeout period (0.5s to 160s)
 * - Whether the device should immediately reset the watchdog
 * - Whether watchdog expiration should stop charging
 *
 * The I2C Watchdog Timer protects the system against bus hangs and software faults.
 * If the MCU fails to reset the watchdog within the specified timeout, the BQ25672 resets its internal registers.
 * Optional STOP_WD_CHG bit can also disable charging when timeout expires.
 *
 * Affected Registers:
 * - REG10: Charger Control 1 (timeout, immediate reset)
 * - REG09: Termination Control (STOP_WD_CHG, disables charge on expiry)
 *
 * @param[in] hi2c  Pointer to I2C peripheral handle.
 * @param[in] cfg   Pointer to watchdog configuration structure.
 *
 * @retval BQ25672_OK          Operation successful.
 * @retval BQ25672_COMM_FAIL    I2C communication failure.
 */
bq25672_status_t bq25672_set_watchdog_config(I2C_HandleTypeDef *hi2c, const bq25672_watchdog_config_t *cfg)
{
    uint8_t reg10 = 0;

    reg10 |= (cfg->timeout << BQ25672_WATCHDOG_TIMER_SHIFT) & BQ25672_WATCHDOG_TIMER_MASK;  				// Bits 2:0: Select Watchdog timeout period (0.5s ~ 160s)
    if (cfg->reset_watchdog_now)     																		// Bit 3: If requested, immediately reset the watchdog
        reg10 |= BQ25672_WDOG_RESET_MASK;
    if (write_reg(hi2c, REG10_Charger_Control_1, reg10) != BQ25672_OK)     	return BQ25672_COMM_FAIL;		// Write timeout & reset control into REG10
    uint8_t reg09;
    if (read_reg(hi2c, REG09_Termination_Control, &reg09) != BQ25672_OK)    return BQ25672_COMM_FAIL; 		// Read REG09 first to preserve unrelated bits
    if (cfg->disable_on_expiry)     																		// Bit 5: Whether watchdog expiry disables charging
        reg09 |= BQ25672_STOP_WD_CHG_MASK;
    else
        reg09 &= ~BQ25672_STOP_WD_CHG_MASK;

    return write_reg(hi2c, REG09_Termination_Control, reg09);
}

/**
 * @brief   Read current BQ25672 I2C Watchdog timer configuration.
 *
 * Reads:
 * - Current watchdog timeout period (0.5s to 160s)
 * - Whether immediate reset trigger is enabled
 * - Whether watchdog expiry disables charging
 *
 * Affected Registers:
 * - REG10: Charger Control 1 (timeout, immediate reset)
 * - REG09: Termination Control (STOP_WD_CHG)
 *
 * @param[in]  hi2c  Pointer to I2C peripheral handle.
 * @param[out] cfg   Pointer to structure where current watchdog configuration is stored.
 *
 * @retval BQ25672_OK          Operation successful.
 * @retval BQ25672_COMM_FAIL    I2C communication failure.
 */
bq25672_status_t bq25672_get_watchdog_config(I2C_HandleTypeDef *hi2c, bq25672_watchdog_config_t *cfg)
{
    uint8_t val;
    bq25672_status_t status;

    status = read_reg(hi2c, REG10_Charger_Control_1, &val); // Read watchdog timeout and immediate reset flag from REG10
    if (status != BQ25672_OK)       return BQ25672_COMM_FAIL;

    cfg->timeout = (bq25672_watchdog_timeout_t)((val & BQ25672_WATCHDOG_TIMER_MASK) >> BQ25672_WATCHDOG_TIMER_SHIFT);   // Bits 2:0: Timeout period (0.5s ~ 160s)
    cfg->reset_watchdog_now = !!(val & BQ25672_WDOG_RESET_MASK);     													// Bit 3: Immediate reset trigger

    status = read_reg(hi2c, REG09_Termination_Control, &val);     														// Read STOP_WD_CHG bit from REG09
    if (status != BQ25672_OK)		return BQ25672_COMM_FAIL;
    cfg->disable_on_expiry = !!(val & BQ25672_STOP_WD_CHG_MASK);     													// Bit 5: Disable charging on watchdog expiry

    return BQ25672_OK;
}

/**
 * @brief   Configure BQ25672 charger and fault interrupt masks.
 *
 * Configures interrupt sources for charger events (DPM, watchdog, etc.) and fault events (OVP, OCP).
 * If mask bit is 1 -> interrupt is DISABLED.
 * If mask bit is 0 -> interrupt is ENABLED.
 *
 * REG28: Charger Event Masks
 * REG2C: Fault Event Masks
 *
 * @param[in] hi2c  Pointer to I2C handle.
 * @param[in] cfg   Pointer to interrupt mask configuration structure.
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_set_interrupt_config(I2C_HandleTypeDef *hi2c, const bq25672_interrupt_config_t *cfg)
{
    uint8_t reg28 = 0;

    /* Charger Event Interrupts (REG28) */
    if (!cfg->iindpm)       reg28 |= BQ25672_IINDPM_MASK_MASK;        // Bit 7: Input current DPM interrupt mask
    if (!cfg->vindpm)       reg28 |= BQ25672_VINDPM_MASK_MASK;        // Bit 6: Input voltage DPM interrupt mask
    if (!cfg->watchdog)     reg28 |= BQ25672_WD_MASK_MASK;            // Bit 5: Watchdog timer interrupt mask
    if (!cfg->poor_source)  reg28 |= BQ25672_POORSRC_MASK_MASK;       // Bit 4: Poor source detection interrupt mask
    if (!cfg->power_good)   reg28 |= BQ25672_PG_MASK_MASK;            // Bit 3: Power-good interrupt mask
    if (!cfg->ac2_present)  reg28 |= BQ25672_AC2_PRESENT_MASK_MASK;   // Bit 2: VAC2 present interrupt mask
    if (!cfg->ac1_present)  reg28 |= BQ25672_AC1_PRESENT_MASK_MASK;   // Bit 1: VAC1 present interrupt mask
    if (!cfg->vbus_present) reg28 |= BQ25672_VBUS_PRESENT_MASK_MASK;  // Bit 0: VBUS present interrupt mask

    if (write_reg(hi2c, REG28_Charger_Mask_0, reg28) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    uint8_t reg2c = 0;

    /* Fault Event Interrupts (REG2C) */
    if (!cfg->ibat_reg)     reg2c |= BQ25672_IBAT_REG_MASK_MASK;      // Bit 7: IBAT regulation fault interrupt mask
    if (!cfg->vbus_ovp)     reg2c |= BQ25672_VBUS_OVP_MASK_MASK;      // Bit 6: VBUS over-voltage fault interrupt mask
    if (!cfg->vbat_ovp)     reg2c |= BQ25672_VBAT_OVP_MASK_MASK;      // Bit 5: VBAT over-voltage fault interrupt mask
    if (!cfg->ibus_ocp)     reg2c |= BQ25672_IBUS_OCP_MASK_MASK;      // Bit 4: IBUS over-current fault interrupt mask
    if (!cfg->ibat_ocp)     reg2c |= BQ25672_IBAT_OCP_MASK_MASK;      // Bit 3: IBAT over-current fault interrupt mask
    if (!cfg->conv_ocp)     reg2c |= BQ25672_CONV_OCP_MASK_MASK;      // Bit 2: Converter over-current fault interrupt mask
    if (!cfg->vac2_ovp)     reg2c |= BQ25672_VAC2_OVP_MASK_MASK;      // Bit 1: VAC2 over-voltage fault interrupt mask
    if (!cfg->vac1_ovp)     reg2c |= BQ25672_VAC1_OVP_MASK_MASK;      // Bit 0: VAC1 over-voltage fault interrupt mask

    return write_reg(hi2c, REG2C_FAULT_Mask_0, reg2c);
}

/**
 * @brief   Read current BQ25672 interrupt mask configuration.
 *
 * Reads both charger event masks (DPM, watchdog, power-good, etc.) and fault masks (OVP, OCP).
 * 0 = interrupt enabled
 * 1 = interrupt masked (disabled)
 *
 * REG28: Charger Event Masks
 * REG2C: Fault Event Masks
 *
 * @param[in]  hi2c  Pointer to I2C handle.
 * @param[out] cfg   Pointer to structure where masks will be stored.
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_get_interrupt_config(I2C_HandleTypeDef *hi2c, bq25672_interrupt_config_t *cfg)
{
    uint8_t reg28, reg2c;
    bq25672_status_t status;

    status = read_reg(hi2c, REG28_Charger_Mask_0, &reg28);
    if (status != BQ25672_OK)	return BQ25672_COMM_FAIL;

    status = read_reg(hi2c, REG2C_FAULT_Mask_0, &reg2c);
    if (status != BQ25672_OK)	return BQ25672_COMM_FAIL;

    /* Charger Event Interrupts (REG28) */
    cfg->iindpm       = !(reg28 & BQ25672_IINDPM_MASK_MASK);        // Bit 7: Input current DPM interrupt
    cfg->vindpm       = !(reg28 & BQ25672_VINDPM_MASK_MASK);        // Bit 6: Input voltage DPM interrupt
    cfg->watchdog     = !(reg28 & BQ25672_WD_MASK_MASK);            // Bit 5: Watchdog timer interrupt
    cfg->poor_source  = !(reg28 & BQ25672_POORSRC_MASK_MASK);       // Bit 4: Poor source detection interrupt
    cfg->power_good   = !(reg28 & BQ25672_PG_MASK_MASK);            // Bit 3: Power-good interrupt
    cfg->ac2_present  = !(reg28 & BQ25672_AC2_PRESENT_MASK_MASK);   // Bit 2: VAC2 present interrupt
    cfg->ac1_present  = !(reg28 & BQ25672_AC1_PRESENT_MASK_MASK);   // Bit 1: VAC1 present interrupt
    cfg->vbus_present = !(reg28 & BQ25672_VBUS_PRESENT_MASK_MASK);  // Bit 0: VBUS present interrupt

    /* Fault Event Interrupts (REG2C) */
    cfg->ibat_reg     = !(reg2c & BQ25672_IBAT_REG_MASK_MASK);      // Bit 7: IBAT regulation fault
    cfg->vbus_ovp     = !(reg2c & BQ25672_VBUS_OVP_MASK_MASK);      // Bit 6: VBUS over-voltage fault
    cfg->vbat_ovp     = !(reg2c & BQ25672_VBAT_OVP_MASK_MASK);      // Bit 5: VBAT over-voltage fault
    cfg->ibus_ocp     = !(reg2c & BQ25672_IBUS_OCP_MASK_MASK);      // Bit 4: IBUS over-current fault
    cfg->ibat_ocp     = !(reg2c & BQ25672_IBAT_OCP_MASK_MASK);      // Bit 3: IBAT over-current fault
    cfg->conv_ocp     = !(reg2c & BQ25672_CONV_OCP_MASK_MASK);      // Bit 2: Converter over-current fault
    cfg->vac2_ovp     = !(reg2c & BQ25672_VAC2_OVP_MASK_MASK);      // Bit 1: VAC2 over-voltage fault
    cfg->vac1_ovp     = !(reg2c & BQ25672_VAC1_OVP_MASK_MASK);      // Bit 0: VAC1 over-voltage fault

    return BQ25672_OK;
}

/**
 * @brief Configure BQ25672 MPPT (Maximum Power Point Tracking) settings.
 *
 * This function sets:
 * - VBUS VOC target ratio (percentage)
 * - Delay after switching before VOC measurement
 * - VOC measurement interval period
 * - Enable/Disable MPPT function
 *
 * Register: REG15 - MPPT Control
 *  [7:5] VOC_PCT (% of VOC)
 *  [4:3] VOC_DLY (delay ms)
 *  [2:1] VOC_RATE (sampling interval)
 *  [0]   EN_MPPT (enable)
 *
 * @param[in] hi2c  Pointer to I2C handle.
 * @param[in] cfg   MPPT configuration structure.
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_set_mppt_config(I2C_HandleTypeDef *hi2c, const bq25672_mppt_config_t *cfg)
{
    uint8_t reg = 0;

    /* Bits 7:5 - VOC % ratio selection (MPPT efficiency point) */
    reg |= ((cfg->ratio << BQ25672_VOC_PCT_SHIFT) & BQ25672_VOC_PCT_MASK);

    /* Bits 4:3 - VOC delay selection (wait time after switching before VOC sampling) */
    uint8_t delay_bits = 0;
    if (cfg->advanced.delay_ms <= 50U)
        delay_bits = 0x0U;   // 50ms
    else if (cfg->advanced.delay_ms <= 300U)
        delay_bits = 0x1U;   // 300ms
    else if (cfg->advanced.delay_ms <= 2000U)
        delay_bits = 0x2U;   // 2s
    else
        delay_bits = 0x3U;   // 5s
    reg |= ((delay_bits << BQ25672_VOC_DLY_SHIFT) & BQ25672_VOC_DLY_MASK);

    /* Bits 2:1 - VOC interval selection (re-check interval for MPPT) */
    uint8_t rate_bits = 0;
    if (cfg->advanced.interval_s <= 30U)
        rate_bits = 0x0U;   // 30 seconds
    else if (cfg->advanced.interval_s <= 120U)
        rate_bits = 0x1U;   // 2 minutes
    else if (cfg->advanced.interval_s <= 600U)
        rate_bits = 0x2U;   // 10 minutes
    else
        rate_bits = 0x3U;   // 30 minutes
    reg |= ((rate_bits << BQ25672_VOC_RATE_SHIFT) & BQ25672_VOC_RATE_MASK);

    /* Bit 0 - MPPT enable */
    reg |= (cfg->enable_mppt ? BQ25672_EN_MPPT_MASK : 0U);

    return write_reg(hi2c, REG15_MPPT_Control, reg);
}

/**
 * @brief Read BQ25672 MPPT (Maximum Power Point Tracking) configuration.
 *
 * This function reads:
 * - MPPT enable status
 * - VOC ratio (target %)
 * - Delay after switching before VOC measurement
 * - VOC measurement interval period
 *
 * Register: REG15 - MPPT Control
 *
 * @param[in]  hi2c  Pointer to I2C handle.
 * @param[out] cfg   MPPT configuration structure to populate.
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_get_mppt_config(I2C_HandleTypeDef *hi2c, bq25672_mppt_config_t *cfg)
{
    uint8_t val;
    bq25672_status_t status;

    status = read_reg(hi2c, REG15_MPPT_Control, &val);
    if (status != BQ25672_OK)		return BQ25672_COMM_FAIL;

    /* Bits 7:5 - VOC ratio (target % of VOC) */
    cfg->ratio = (bq25672_mppt_pct_t)((val & BQ25672_VOC_PCT_MASK) >> BQ25672_VOC_PCT_SHIFT);

    /* Bits 4:3 - VOC delay (in ms) */
    switch ((val & BQ25672_VOC_DLY_MASK) >> BQ25672_VOC_DLY_SHIFT)
    {
        case 0: cfg->advanced.delay_ms = 50U;   break;   // 50ms
        case 1: cfg->advanced.delay_ms = 300U;  break;   // 300ms
        case 2: cfg->advanced.delay_ms = 2000U; break;   // 2s
        case 3: cfg->advanced.delay_ms = 5000U; break;   // 5s
    }

    /* Bits 2:1 - VOC sampling interval (in seconds) */
    switch ((val & BQ25672_VOC_RATE_MASK) >> BQ25672_VOC_RATE_SHIFT)
    {
        case 0: cfg->advanced.interval_s = 30U;    break;   // 30s
        case 1: cfg->advanced.interval_s = 120U;   break;   // 2min
        case 2: cfg->advanced.interval_s = 600U;   break;   // 10min
        case 3: cfg->advanced.interval_s = 1800U;  break;   // 30min
    }

    /* Bit 0 - MPPT enable status */
    cfg->enable_mppt = !!(val & BQ25672_EN_MPPT_MASK);

    return BQ25672_OK;
}

/**
 * @brief Configure BQ25672 JEITA (NTC) thresholds and behaviors. (Negative Temp Coeff.)
 *
 * Configures:
 * - REG17 NTC Control 0:
 *     [7:5] VREG reduction (voltage drop on high temp)
 *     [4:3] Charge current reduction on high temp
 *     [2:1] Charge current reduction on low temp
 * - REG18 NTC Control 1:
 *     [7:6] TS cool rising threshold (5, 10, 15, 20°C)
 *     [5:4] TS warm falling threshold (40, 45, 50, 55°C)
 *     [3:2] BHOT threshold (OTG mode 55, 60, 65°C, disable)
 *     [1]   BCOLD threshold (OTG mode -10°C or -20°C)
 *     [0]   Ignore TS comparator
 *
 * @param[in] hi2c  I2C handle
 * @param[in] cfg   NTC config structure
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_set_ntc_config(I2C_HandleTypeDef *hi2c, const bq25672_ntc_config_t *cfg)
{
    uint8_t reg17 = 0;
    uint8_t reg18 = 0;

    reg17 |= (cfg->vreg_reduce_mv << BQ25672_JEITA_VSET_SHIFT) & BQ25672_JEITA_VSET_MASK;    // Bits 7:5 - VREG reduction mV
    reg17 |= (cfg->ichg_hot_reduce << BQ25672_JEITA_ISETH_SHIFT) & BQ25672_JEITA_ISETH_MASK; // Bits 4:3 - HOT ICHG reduction %
    reg17 |= (cfg->ichg_cold_reduce << BQ25672_JEITA_ISETC_SHIFT) & BQ25672_JEITA_ISETC_MASK; // Bits 2:1 - COLD ICHG reduction %

    if (write_reg(hi2c, REG17_NTC_Control_0, reg17) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    if (cfg->ts_cool_deg <= 5U)
        reg18 |= (0x0U << BQ25672_TS_COOL_SHIFT); // Bits 7:6 - 5°C
    else if (cfg->ts_cool_deg <= 10U)
        reg18 |= (0x1U << BQ25672_TS_COOL_SHIFT); // 10°C
    else if (cfg->ts_cool_deg <= 15U)
        reg18 |= (0x2U << BQ25672_TS_COOL_SHIFT); // 15°C
    else
        reg18 |= (0x3U << BQ25672_TS_COOL_SHIFT); // 20°C

    if (cfg->ts_warm_deg <= 40U)
        reg18 |= (0x0U << BQ25672_TS_WARM_SHIFT); // Bits 5:4 - 40°C
    else if (cfg->ts_warm_deg <= 45U)
        reg18 |= (0x1U << BQ25672_TS_WARM_SHIFT); // 45°C
    else if (cfg->ts_warm_deg <= 50U)
        reg18 |= (0x2U << BQ25672_TS_WARM_SHIFT); // 50°C
    else
        reg18 |= (0x3U << BQ25672_TS_WARM_SHIFT); // 55°C

    if (cfg->bhot_deg <= 55U)
        reg18 |= (0x0U << BQ25672_BHOT_SHIFT); // Bits 3:2 - 55°C
    else if (cfg->bhot_deg <= 60U)
        reg18 |= (0x1U << BQ25672_BHOT_SHIFT); // 60°C
    else if (cfg->bhot_deg <= 65U)
        reg18 |= (0x2U << BQ25672_BHOT_SHIFT); // 65°C
    else
        reg18 |= (0x3U << BQ25672_BHOT_SHIFT); // Disabled

    reg18 |= (cfg->bcold_20c ? BQ25672_BCOLD_MASK : 0U); // Bit 1 - BCOLD 1 = -20°C, 0 = -10°C
    reg18 |= (cfg->ignore_ts ? BQ25672_TS_IGNORE_MASK : 0U); // Bit 0 - Ignore TS comparator

    return write_reg(hi2c, REG18_NTC_Control_1, reg18);
}

/**
 * @brief   Read BQ25672 JEITA (NTC) thresholds and behaviors. (Japan ... Temp Standart)
 *
 * Reads:
 * - VREG and ICHG reductions (high/low temp)
 * - TS thresholds (cool, warm)
 * - BHOT / BCOLD thresholds
 * - Ignore TS flag
 *
 * @param[in]  hi2c  I2C handle
 * @param[out] cfg   NTC config structure
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_get_ntc_config(I2C_HandleTypeDef *hi2c, bq25672_ntc_config_t *cfg)
{
    uint8_t val;
    bq25672_status_t status;

    status = read_reg(hi2c, REG17_NTC_Control_0, &val);
    if (status != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    cfg->vreg_reduce_mv   = (bq25672_ntc_vreduce_t)((val & BQ25672_JEITA_VSET_MASK) >> BQ25672_JEITA_VSET_SHIFT);   // Bits 7:5 VREG reduction
    cfg->ichg_hot_reduce  = (bq25672_ntc_ichg_reduce_t)((val & BQ25672_JEITA_ISETH_MASK) >> BQ25672_JEITA_ISETH_SHIFT); // Bits 4:3 HOT ICHG
    cfg->ichg_cold_reduce = (bq25672_ntc_cold_current_t)((val & BQ25672_JEITA_ISETC_MASK) >> BQ25672_JEITA_ISETC_SHIFT); // Bits 2:1 COLD ICHG

    status = read_reg(hi2c, REG18_NTC_Control_1, &val);
    if (status != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    switch ((val & BQ25672_TS_COOL_MASK) >> BQ25672_TS_COOL_SHIFT)
    {
        case 0: cfg->ts_cool_deg = 5U;  break; // Bits 7:6 5°C
        case 1: cfg->ts_cool_deg = 10U; break;
        case 2: cfg->ts_cool_deg = 15U; break;
        case 3: cfg->ts_cool_deg = 20U; break;
    }

    switch ((val & BQ25672_TS_WARM_MASK) >> BQ25672_TS_WARM_SHIFT)
    {
        case 0: cfg->ts_warm_deg = 40U; break; // Bits 5:4 40°C
        case 1: cfg->ts_warm_deg = 45U; break;
        case 2: cfg->ts_warm_deg = 50U; break;
        case 3: cfg->ts_warm_deg = 55U; break;
    }

    switch ((val & BQ25672_BHOT_MASK) >> BQ25672_BHOT_SHIFT)
    {
        case 0: cfg->bhot_deg = 55U; break; // Bits 3:2 55°C
        case 1: cfg->bhot_deg = 60U; break;
        case 2: cfg->bhot_deg = 65U; break;
        case 3: cfg->bhot_deg = 0U;  break; // Disabled
    }

    cfg->bcold_20c = !!(val & BQ25672_BCOLD_MASK);       // Bit 1 - BCOLD 1 = -20°C, 0 = -10°C
    cfg->ignore_ts = !!(val & BQ25672_TS_IGNORE_MASK);   // Bit 0 - Ignore TS

    return BQ25672_OK;
}

/**
 * @brief Configure BQ25672 Temperature Control (REG16).
 *
 * Configures:
 * - TREG: Thermal regulation threshold (60, 80, 100, 120°C)
 * - TSHUT: Thermal shutdown threshold (150, 130, 120, 85°C)
 * - VBUS / VAC1 / VAC2 pull-down resistor enables
 *
 * @param[in] hi2c  I2C handle
 * @param[in] cfg   Temperature configuration structure
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_set_temp_config(I2C_HandleTypeDef *hi2c, const bq25672_temp_config_t *cfg)
{
    uint8_t reg = 0;

    reg |= ((cfg->treg  << BQ25672_TREG_SHIFT)  & BQ25672_TREG_MASK);   // Bits 7:6 - Thermal regulation °C
    reg |= ((cfg->tshut << BQ25672_TSHUT_SHIFT) & BQ25672_TSHUT_MASK);  // Bits 5:4 - Thermal shutdown °C
    reg |= (cfg->vbus_pd_en ? BQ25672_VBUS_PD_EN_MASK : 0U);            // Bit 3 - Enable VBUS pull-down resistor
    reg |= (cfg->vac1_pd_en ? BQ25672_VAC1_PD_EN_MASK : 0U);            // Bit 2 - Enable VAC1 pull-down resistor
    reg |= (cfg->vac2_pd_en ? BQ25672_VAC2_PD_EN_MASK : 0U);            // Bit 1 - Enable VAC2 pull-down resistor

    return write_reg(hi2c, REG16_Temperature_Control, reg);
}

/**
 * @brief Read BQ25672 Temperature Control (REG16).
 *
 * Reads:
 * - TREG: Thermal regulation threshold
 * - TSHUT: Thermal shutdown threshold
 * - VBUS / VAC1 / VAC2 pull-down resistor enables
 *
 * @param[in]  hi2c  I2C handle
 * @param[out] cfg   Temperature configuration structure
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_get_temp_config(I2C_HandleTypeDef *hi2c, bq25672_temp_config_t *cfg)
{
    uint8_t val;
    bq25672_status_t status;

    status = read_reg(hi2c, REG16_Temperature_Control, &val);
    if (status != BQ25672_OK)	return BQ25672_COMM_FAIL;

    cfg->treg        = (bq25672_temp_regulation_t)((val & BQ25672_TREG_MASK) >> BQ25672_TREG_SHIFT);   // Bits 7:6
    cfg->tshut       = (bq25672_temp_shutdown_t)((val & BQ25672_TSHUT_MASK) >> BQ25672_TSHUT_SHIFT);   // Bits 5:4
    cfg->vbus_pd_en  = !!(val & BQ25672_VBUS_PD_EN_MASK);   // Bit 3
    cfg->vac1_pd_en  = !!(val & BQ25672_VAC1_PD_EN_MASK);   // Bit 2
    cfg->vac2_pd_en  = !!(val & BQ25672_VAC2_PD_EN_MASK);   // Bit 1

    return BQ25672_OK;
}

/**
 * @brief Reads complete BQ25672 device configuration into one structure.
 *
 * This function collects the entire configuration from the IC and stores it in a single structure.
 * Covers all operational blocks: system, charge, USB, timers, watchdog, MPPT, NTC, JEITA, interrupt masks.
 *
 * Registers accessed:
 * - REG00 to REG0F: System & Charger behavior
 * - REG10 to REG1F: JEITA, USB, Timer, NTC, MPPT, etc.
 * - REG28 / REG2C: Interrupt masks
 *
 * @param[in]  hi2c   I2C handle
 * @param[out] out    Pointer to filled config structure
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_read_config(I2C_HandleTypeDef *hi2c, bq25672_config_t *out)
{
    bq25672_status_t st;

    // System configuration: EN_HIZ, EN_TERM, ACDRV1/2, FORCE_ICO, EN_ICO, PFM etc. (REG0F and others)
    if ((st = bq25672_get_system_config(hi2c, &out->system)) != BQ25672_OK)		return st;

    // Charging parameters: charge voltage, charge current, input voltage/current limits (REG01-06)
    if ((st = bq25672_get_charge_config(hi2c, &out->charge)) != BQ25672_OK)		return st;

    // USB / OTG configuration: IOTG, VOTG, EN_OTG, disable PFM / OOA for OTG (REG08, REG0A)
    if ((st = bq25672_get_usb_config(hi2c, &out->usb)) != BQ25672_OK)			return st;

    // Timer configuration: enable/disable precharge, fast charge, top-off timers (REG0E)
    if ((st = bq25672_get_timer_config(hi2c, &out->timer)) != BQ25672_OK)		return st;

    // I2C Watchdog Timer: timeout value, stop charging on expiry (REG10), reset bit (REG09)
    if ((st = bq25672_get_watchdog_config(hi2c, &out->watchdog)) != BQ25672_OK)	return st;

    // MPPT tracking: VOC %, delay, interval, enable status for solar/weak source (REG15)
    if ((st = bq25672_get_mppt_config(hi2c, &out->mppt)) != BQ25672_OK)			return st;

    // JEITA NTC configuration: thresholds (cool, warm), BHOT, BCOLD, TS ignore, VREG and ICHG reductions (REG17, REG18)
    if ((st = bq25672_get_ntc_config(hi2c, &out->ntc)) != BQ25672_OK)			return st;

    // Internal temperature protection: TREG (thermal regulation), TSHUT (shutdown), VBUS/VAC1/VAC2 pull-down enables (REG16)
    if ((st = bq25672_get_temp_config(hi2c, &out->jeita)) != BQ25672_OK)		return st;

    // Interrupt configuration: charger event masks, fault event masks (REG28, REG2C)
    if ((st = bq25672_get_interrupt_config(hi2c, &out->interrupt)) != BQ25672_OK)return st;

    return BQ25672_OK;
}

/**
 * @brief Applies all BQ25672 configuration fields to hardware registers.
 *
 * Writes system behavior, charging parameters, OTG settings, timers, watchdog,
 * MPPT tracking, JEITA thresholds, temperature protections, and interrupt masks
 * into the relevant registers. This function complements bq25672_read_config().
 *
 * @param[in] hi2c    I2C handle used for communication.
 * @param[in] config  Pointer to configuration structure.
 *
 * @retval BQ25672_OK         All configurations written successfully.
 * @retval BQ25672_COMM_FAIL  Communication error occurred.
 */
bq25672_status_t bq25672_apply_config(I2C_HandleTypeDef *hi2c, const bq25672_config_t *config)
{
    bq25672_status_t st;

    // REG0F, REG12-13: System features - EN_HIZ, EN_TERM, ACDRV1/2, PFM, LDO, OOA, switching frequency etc.
    if ((st = bq25672_set_system_config(hi2c, &config->system)) != BQ25672_OK) return st;

    // REG01-06: Charging parameters - charge voltage, current, input voltage limit (VINDPM), input current limit (IINDPM)
    if ((st = bq25672_set_charge_config(hi2c, &config->charge)) != BQ25672_OK) return st;

    // REG0B, REG0D, REG12: USB OTG settings - VOTG, IOTG, EN_OTG, disable PFM/OOA for OTG boost mode
    if ((st = bq25672_set_usb_config(hi2c, &config->usb)) != BQ25672_OK) return st;

    // REG0E: Timers - precharge, fast charge, top-off timers and their enables
    if ((st = bq25672_set_timer_config(hi2c, &config->timer)) != BQ25672_OK) return st;

    // REG09-10: Watchdog timer - timeout duration, reset behavior, stop-charging on expiry
    if ((st = bq25672_set_watchdog_config(hi2c, &config->watchdog)) != BQ25672_OK) return st;

    // REG15: MPPT - VOC %, switching delay, interval, MPPT enable for weak sources
    if ((st = bq25672_set_mppt_config(hi2c, &config->mppt)) != BQ25672_OK) return st;

    // REG17-18: JEITA NTC thresholds - TS cool/warm regions, BHOT, BCOLD, ignore TS, VREG & ICHG reductions for JEITA
    if ((st = bq25672_set_ntc_config(hi2c, &config->ntc)) != BQ25672_OK) return st;

    // REG16: Temperature protection - TREG (thermal regulation limit), TSHUT (shutdown limit), pull-down enables for VBUS, VAC1/2
    if ((st = bq25672_set_temp_config(hi2c, &config->jeita)) != BQ25672_OK) return st;

    // REG28-2B (Charger masks), REG2C-2D (Fault masks): Interrupt sources enable/disable configuration
    if ((st = bq25672_set_interrupt_config(hi2c, &config->interrupt)) != BQ25672_OK) return st;

    return BQ25672_OK;
}

/**
 * @brief Reads a 10-bit ADC value from two consecutive BQ25672 registers.
 *
 * This function reads the ADC result which is stored across two registers:
 * - LSB: Lower 8 bits
 * - MSB: Upper 2 bits (placed in [9:8])
 * The function combines them into a 16-bit result for further processing.
 *
 * Typical usage: VBAT, IBUS, VBUS ADC readings.
 *
 * @param[in]  hi2c      I2C handle used for communication.
 * @param[in]  reg_lo    Lower byte register address (LSB).
 * @param[in]  reg_hi    Upper byte register address (MSB).
 * @param[out] raw_adc   Pointer to store the combined 16-bit ADC result.
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
static bq25672_status_t read_adc(I2C_HandleTypeDef *hi2c, uint8_t reg_lo, uint8_t reg_hi, uint16_t *raw_adc)
{
    uint8_t lsb; // Lower byte (LSB)
    uint8_t msb; // Upper byte (MSB)
    bq25672_status_t st;

    // Read lower byte (LSB): Contains bits [7:0]
    st = read_reg(hi2c, reg_lo, &lsb);
    if (st != BQ25672_OK)	return st;

    // Read upper byte (MSB): Contains bits [9:8]
    st = read_reg(hi2c, reg_hi, &msb);
    if (st != BQ25672_OK)	return st;

    // Combine MSB and LSB into a 16-bit raw ADC value
    *raw_adc = ((uint16_t)msb << 8) | lsb; // MSB shifted to [15:8], OR with LSB [7:0]

    return BQ25672_OK;
}

/**
 * @brief Reads the VBUS input current (IBUS) from BQ25672.
 *
 * Reads REG31 (LSB) and REG32 (MSB) for IBUS ADC value.
 * 1 LSB = 1mA (0 - 5000mA range), 2's complement format.
 *
 * @param[in]  hi2c I2C handle.
 * @param[out] ma   Pointer to store input current in mA.
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_read_ibus_current_ma(I2C_HandleTypeDef *hi2c, int16_t *ma)
{
    uint16_t raw;
    bq25672_status_t st;

    // IBUS current ADC: REG31 (LSB), REG32 (MSB)
    st = read_adc(hi2c, REG31_IBUS_ADC_L, REG32_IBUS_ADC_H, &raw);
    if (st != BQ25672_OK)	return st;

    *ma = (int16_t)raw; // 2's complement, 1 LSB = 1mA
    return BQ25672_OK;
}

/**
 * @brief Reads the VBUS input voltage from BQ25672.
 *
 * Reads REG35 (LSB) and REG36 (MSB) for VBUS ADC value.
 * 1 LSB = 1mV (0 - 30000mV range).
 *
 * @param[in]  hi2c I2C handle.
 * @param[out] mv   Pointer to store input voltage in mV.
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_read_vbus_voltage_mv(I2C_HandleTypeDef *hi2c, uint16_t *mv)
{
    uint16_t raw;
    bq25672_status_t st;

    // VBUS voltage ADC: REG35 (LSB), REG36 (MSB)
    st = read_adc(hi2c, REG35_VBUS_ADC_L, REG36_VBUS_ADC_H, &raw);
    if (st != BQ25672_OK)	return st;

    *mv = raw; // 1 LSB = 1mV
    return BQ25672_OK;
}

/**
 * @brief Reads the battery charging/discharging current (IBAT) from BQ25672.
 *
 * Reads REG33 (LSB) and REG34 (MSB) for IBAT ADC value.
 * 1 LSB = 1mA (0 - 8000mA range), 2's complement format.
 *
 * @param[in]  hi2c I2C handle.
 * @param[out] ma   Pointer to store battery current in mA.
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_read_ibat_current_ma(I2C_HandleTypeDef *hi2c, int16_t *ma)
{
    uint16_t raw;
    bq25672_status_t st;

    // IBAT current ADC: REG33 (LSB), REG34 (MSB)
    st = read_adc(hi2c, REG33_IBAT_ADC_L, REG34_IBAT_ADC_H, &raw);
    if (st != BQ25672_OK)	return st;

    *ma = (int16_t)raw; // 2's complement, 1 LSB = 1mA
    return BQ25672_OK;
}

/**
 * @brief Reads the battery voltage (VBAT) from BQ25672.
 *
 * Reads REG3B (LSB) and REG3C (MSB) for VBAT ADC value.
 * 1 LSB = 1mV (0 - 20000mV range).
 *
 * @param[in]  hi2c I2C handle.
 * @param[out] mv   Pointer to store battery voltage in mV.
 *
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_read_vbat_voltage_mv(I2C_HandleTypeDef *hi2c, uint16_t *mv)
{
    uint16_t raw;
    bq25672_status_t st;

    // VBAT voltage ADC: REG3B (LSB), REG3C (MSB)
    st = read_adc(hi2c, REG3B_VBAT_ADC_L, REG3C_VBAT_ADC_H, &raw);
    if (st != BQ25672_OK)	return st;

    *mv = raw; // 1 LSB = 1mV
    return BQ25672_OK;
}

/**
 * @brief Reads BQ25672 charger and system status registers (REG1B-1F)
 *
 * Retrieves charger status, input DPM status, VBUS presence, AC1/AC2 presence,
 * watchdog expiry, ICO, thermal, battery presence, VSYS regulation, ADC status,
 * safety timers, and TS temperature zones directly from hardware registers.
 *
 * @param[in]  hi2c    I2C handle for communication.
 * @param[out] status  Pointer to device status structure.
 *
 * @retval BQ25672_OK        Successful read.
 * @retval BQ25672_COMM_FAIL I2C communication failure.
 */
bq25672_status_t bq25672_get_device_status(I2C_HandleTypeDef *hi2c, bq25672_device_status_t *status)
{
    uint8_t reg;

    // REG1B: Charger Status 0
    if (read_reg(hi2c, REG1B_Charger_Status_0, &reg) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    status->vbus_present    = !!(reg & BQ25672_VBUS_PRESENT_STAT_MASK);  ///< VBUS present status (REG1B[0])
    status->charger_active  = !!(reg & BQ25672_PG_STAT_MASK);            ///< Power-Good (Charger active) status (REG1B[3])
    status->otg_mode        = false; // REG1B bit5 WD_STAT is NOT OTG indication, ignored here.

    // REG1C: Charger Status 1
    if (read_reg(hi2c, REG1C_Charger_Status_1, &reg) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    status->charge_phase = (bq25672_charge_status_t)((reg & BQ25672_CHG_STAT_MASK) >> BQ25672_CHG_STAT_SHIFT); ///< Charge state (REG1C[7:5])
    status->vbus_type    = (bq25672_vbus_status_t)((reg & BQ25672_VBUS_STAT_MASK) >> BQ25672_VBUS_STAT_SHIFT); ///< VBUS type (REG1C[4:1])
    status->bc12_done    = !!(reg & BQ25672_BC12_DONE_MASK); ///< BC1.2 detection completed (REG1C[0])

    // REG1D: Charger Status 2
    if (read_reg(hi2c, REG1D_Charger_Status_2, &reg) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    status->advanced.ico_state          = (bq25672_ico_status_t)((reg & BQ25672_ICO_STAT_MASK) >> BQ25672_ICO_STAT_SHIFT); ///< ICO state (REG1D[7:6])
    status->advanced.thermal_regulation = !!(reg & BQ25672_TREG_STAT_MASK); ///< Thermal regulation active (REG1D[2])
    status->advanced.dpdm_detection     = !!(reg & BQ25672_DPDM_STAT_MASK); ///< D+/D- detection ongoing (REG1D[1])
    status->advanced.vbat_present       = !!(reg & BQ25672_VBAT_PRESENT_STAT_MASK); ///< Battery present status (REG1D[0])

    // REG1E: Charger Status 3
    if (read_reg(hi2c, REG1E_Charger_Status_3, &reg) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    status->advanced.acrb2_placed           = !!(reg & BQ25672_ACRB2_STAT_MASK); ///< ACDRV2 FET placed (REG1E[7])
    status->advanced.acrb1_placed           = !!(reg & BQ25672_ACRB1_STAT_MASK); ///< ACDRV1 FET placed (REG1E[6])
    status->advanced.adc_done               = !!(reg & BQ25672_ADC_DONE_STAT_MASK); ///< ADC conversion complete (REG1E[5])
    status->advanced.vsys_regulation        = !!(reg & BQ25672_VSYS_STAT_MASK); ///< VSYS regulation active (REG1E[4])
    status->advanced.fast_timer_expired     = !!(reg & BQ25672_CHG_TMR_STAT_MASK); ///< Fast charge timer expired (REG1E[3])
    status->advanced.trickle_timer_expired  = !!(reg & BQ25672_TRICKLE_TMR_STAT_MASK); ///< Trickle charge timer expired (REG1E[2])
    status->advanced.precharge_timer_expired= !!(reg & BQ25672_PRECHG_TMR_STAT_MASK); ///< Pre-charge timer expired (REG1E[1])

    // REG1F: Charger Status 4
    if (read_reg(hi2c, REG1F_Charger_Status_4, &reg) != BQ25672_OK)	return BQ25672_COMM_FAIL;

    status->advanced.vbat_otg_too_low = !!(reg & BQ25672_VBATOTG_LOW_STAT_MASK); ///< VBAT too low for OTG (REG1F[4])
    status->advanced.ts_cold          = !!(reg & BQ25672_TS_COLD_STAT_MASK); ///< TS cold zone active (REG1F[3])
    status->advanced.ts_cool          = !!(reg & BQ25672_TS_COOL_STAT_MASK); ///< TS cool zone active (REG1F[2])
    status->advanced.ts_warm          = !!(reg & BQ25672_TS_WARM_STAT_MASK); ///< TS warm zone active (REG1F[1])
    status->advanced.ts_hot           = !!(reg & BQ25672_TS_HOT_STAT_MASK); ///< TS hot zone active (REG1F[0])

    return BQ25672_OK;
}

/**
 * @brief Reads BQ25672 fault protection flags from REG26-27 using existing shift/mask defines.
 *
 * Reads FAULT_FLAG_0 and FAULT_FLAG_1 registers to retrieve information
 * about over-voltage, over-current, thermal shutdown, and short-circuit faults.
 *
 * @param[in]  hi2c   I2C handle for communication.
 * @param[out] status Pointer to fault status structure.
 *
 * @retval BQ25672_OK        Successful read.
 * @retval BQ25672_COMM_FAIL I2C communication failure.
 */
bq25672_status_t bq25672_get_fault_status(I2C_HandleTypeDef *hi2c, bq25672_fault_status_t *status)
{
    uint8_t reg;

    // REG26: FAULT_FLAG_0
    if (read_reg(hi2c, REG26_FAULT_Flag_0, &reg) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    status->ibat_regulation = !!(reg & BQ25672_IBAT_REG_FLAG_MASK); ///< IBAT regulation fault flag (REG26[7])
    status->vbus_ovp        = !!(reg & BQ25672_VBUS_OVP_FLAG_MASK); ///< VBUS over-voltage fault flag (REG26[6])
    status->vbat_ovp        = !!(reg & BQ25672_VBAT_OVP_FLAG_MASK); ///< VBAT over-voltage fault flag (REG26[5])
    status->ibus_ocp        = !!(reg & BQ25672_IBUS_OCP_FLAG_MASK); ///< IBUS over-current fault flag (REG26[4])
    status->ibat_ocp        = !!(reg & BQ25672_IBAT_OCP_FLAG_MASK); ///< IBAT over-current fault flag (REG26[3])
    status->conv_ocp        = !!(reg & BQ25672_CONV_OCP_FLAG_MASK); ///< Converter over-current fault flag (REG26[2])
    status->vac2_ovp        = !!(reg & BQ25672_VAC2_OVP_FLAG_MASK); ///< VAC2 input over-voltage fault flag (REG26[1])
    status->vac1_ovp        = !!(reg & BQ25672_VAC1_OVP_FLAG_MASK); ///< VAC1 input over-voltage fault flag (REG26[0])

    // REG27: FAULT_FLAG_1
    if (read_reg(hi2c, REG27_FAULT_Flag_1, &reg) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    status->vsys_short = !!(reg & BQ25672_VSYS_SHORT_FLAG_MASK); ///< VSYS short-circuit fault flag (REG27[7])
    status->vsys_ovp   = !!(reg & BQ25672_VSYS_OVP_FLAG_MASK);   ///< VSYS over-voltage fault flag (REG27[6])
    status->otg_ovp    = !!(reg & BQ25672_OTG_OVP_FLAG_MASK);    ///< OTG over-voltage fault flag (REG27[5])
    status->otg_uvp    = !!(reg & BQ25672_OTG_UVP_FLAG_MASK);    ///< OTG under-voltage fault flag (REG27[4])
    status->tshut      = !!(reg & BQ25672_TSHUT_FLAG_MASK);      ///< Thermal shutdown fault flag (REG27[2])

    return BQ25672_OK;
}

/**
 * @brief Enable or disable battery charging via EN_CHG bit (REG0F[5]).
 *
 * EN_CHG = 1 enables charging (default).
 * EN_CHG = 0 disables charging.
 *
 * @param[in] hi2c    I2C handle
 * @param[in] enabled true = enable charging, false = disable charging
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_set_charging_state(I2C_HandleTypeDef *hi2c, bool enabled)
{
    uint8_t reg;
    if (read_reg(hi2c, REG0F_Charger_Control_0, &reg) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    if (enabled)
        reg |= BQ25672_EN_CHG_MASK;   // Enable charging
    else
        reg &= ~BQ25672_EN_CHG_MASK;  // Disable charging

    return write_reg(hi2c, REG0F_Charger_Control_0, reg);
}

/**
 * @brief Get current battery charging enable state (EN_CHG bit).
 *
 * Reads REG0F[5] to determine if charging is enabled.
 *
 * @param[in]  hi2c    I2C handle
 * @param[out] enabled true = charging enabled, false = charging disabled
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_get_charging_state(I2C_HandleTypeDef *hi2c, bool *enabled)
{
    uint8_t reg;
    if (read_reg(hi2c, REG0F_Charger_Control_0, &reg) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    *enabled = !!(reg & BQ25672_EN_CHG_MASK);
    return BQ25672_OK;
}

/**
 * @brief Enter or exit High Impedance Input Mode (Hi-Z).
 *
 * EN_HIZ = 1 forces input FETs off (Hi-Z mode).
 * EN_HIZ = 0 enables input (normal operation).
 *
 * @param[in] hi2c    I2C handle
 * @param[in] enabled true = enter HIZ, false = normal
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_set_hiz_state(I2C_HandleTypeDef *hi2c, bool enabled)
{
    uint8_t reg;
    if (read_reg(hi2c, REG0F_Charger_Control_0, &reg) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    if (enabled)
        reg |= BQ25672_EN_HIZ_MASK;
    else
        reg &= ~BQ25672_EN_HIZ_MASK;

    return write_reg(hi2c, REG0F_Charger_Control_0, reg);
}

/**
 * @brief Enable or disable OTG (VBUS boost mode).
 *
 * EN_OTG = 1 enables VBUS boost from battery.
 * EN_OTG = 0 disables boost.
 *
 * @param[in] hi2c    I2C handle
 * @param[in] enabled true = enable OTG, false = disable OTG
 * @retval BQ25672_OK
 * @retval BQ25672_COMM_FAIL
 */
bq25672_status_t bq25672_set_otg_state(I2C_HandleTypeDef *hi2c, bool enabled)
{
    uint8_t reg;
    if (read_reg(hi2c, REG0F_Charger_Control_0, &reg) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    if (enabled)
        reg |= BQ25672_EN_OTG_MASK;   // Enable OTG
    else
        reg &= ~BQ25672_EN_OTG_MASK;  // Disable OTG

    return write_reg(hi2c, REG0F_Charger_Control_0, reg);
}

/**
 * @brief Perform software reset (toggles RESET bit in REG09 Termination Control register).
 *
 * This function performs a software reset on the BQ25672 by writing 1 to the REG_RST bit (bit 6)
 * in the Termination Control register (REG09). After the reset is triggered, the device will
 * automatically clear this bit back to 0 and restore all registers to their default power-on state.
 *
 * After issuing this reset, all configurations must be re-applied as the IC resets to default values.
 *
 * @param[in] hi2c  I2C handle for communication.
 *
 * @retval BQ25672_OK         Reset command successfully issued.
 * @retval BQ25672_COMM_FAIL  I2C communication failure.
 */
bq25672_status_t bq25672_soft_reset(I2C_HandleTypeDef *hi2c)
{
    uint8_t reg;

    // Read REG09 (Termination Control) to preserve other bits.
    if (read_reg(hi2c, REG09_Termination_Control, &reg) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    reg |= BQ25672_REG_RST_MASK; // Set REG_RST bit (bit 6) to trigger reset

    if (write_reg(hi2c, REG09_Termination_Control, reg) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    // After reset, all registers return to their datasheet default values automatically.
    return BQ25672_OK;
}

/**
 * @brief   Perform hard reset (System Power Reset via SDRV_CTRL in REG11).
 *
 * @param[in] hi2c  I2C handle for communication.
 * @retval BQ25672_OK         Reset command issued.
 * @retval BQ25672_COMM_FAIL  I2C comm. failure.
 */
bq25672_status_t bq25672_hard_reset(I2C_HandleTypeDef *hi2c)
{
    uint8_t reg;

    if (read_reg(hi2c, REG11_Charger_Control_2, &reg) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    reg &= ~BQ25672_SDRV_CTRL_MASK;     // Clear SDRV_CTRL[2:1]
    reg |= BQ25672_SDRV_CTRL_MASK; 		// Set SDRV_CTRL = 0b11

    if (write_reg(hi2c, REG11_Charger_Control_2, reg) != BQ25672_OK)
        return BQ25672_COMM_FAIL;

    return BQ25672_OK;
}

/**
 * @brief Initialize BQ25672 hardware and apply configuration.
 *
 * This function initializes the BQ25672 by performing a software reset, waiting for the
 * hardware startup time, and applying the user-provided or default configuration.
 *
 * Initialization steps (datasheet sequence):
 * 1️- Issue software reset (REG09 REG_RST bit = 1)
 * 2️- Wait BQ25672_STARTUP_DELAY_MS for hardware startup stabilization (typ. 20-25ms after reset)
 * 3️- Apply user configuration or default configuration to all registers
 *
 * After this function executes successfully, BQ25672 is fully configured and ready.
 *
 * @param[in] hi2c    I2C handle used for communication.
 * @param[in] config  Configuration struct. Pass NULL to use default config.
 *
 * @retval BQ25672_OK         Initialization successful.
 * @retval BQ25672_COMM_FAIL  I2C communication failure.
 */
bq25672_status_t bq25672_init(I2C_HandleTypeDef *hi2c, const bq25672_config_t *config)
{
    bq25672_status_t st;

    // Perform software reset to ensure registers return to default state (REG_RST = 1)
    if ((st = bq25672_soft_reset(hi2c)) != BQ25672_OK)
        return st;
    // Wait for IC startup stabilization time (datasheet min. 20ms, defined as 25ms for safety)
    HAL_Delay(BQ25672_STARTUP_DELAY_MS);
    return bq25672_apply_config(hi2c, (config != NULL) ? config : &BQ25672_DEFAULT_CONFIG);
}



