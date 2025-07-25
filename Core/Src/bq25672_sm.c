/*
 * bq25672_sm.c
 *
 *  Created on: Jul 24, 2025
 *      Author: YED
 */

#include "bq25672_sm.h"

// I2C timeout (ms)
#define BQ25672_I2C_TIMEOUT 100
// Watchdog reset interval (ms)
#define BQ25672_WATCHDOG_RESET_MS 20000

// ADC-based thresholds
#define VBAT_MIN_THRESHOLD_MV 3000  // 3.0V
#define IBAT_MAX_THRESHOLD_MA 1200  // 1.2A
#define VBUS_MIN_THRESHOLD_MV 4500  // 4.5V
#define VBUS_MAX_THRESHOLD_MV 5500  // 5.5V

// Global state variables
static volatile bq25672_state_t current_state = BQ25672_STATE_IDLE;
static volatile bool i2c_busy = false;
static volatile bq25672_status_t i2c_status = BQ25672_OK;
static volatile bool i2c_tx_complete = false;
static volatile bool i2c_rx_complete = false;
static uint32_t last_watchdog_reset = 0;
static bq25672_config_t current_config;

// Static function declarations
static void bq25672_sm_handle_fault(I2C_HandleTypeDef *hi2c, bq25672_fault_status_t *fault);
static void bq25672_sm_reset_watchdog(I2C_HandleTypeDef *hi2c);
static bq25672_status_t bq25672_sm_check_thermal(I2C_HandleTypeDef *hi2c);
static bool bq25672_sm_check_adc_conditions(I2C_HandleTypeDef *hi2c);

// I2C Callback Functions
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        i2c_tx_complete = true;
        i2c_busy = false;
        i2c_status = BQ25672_OK;
    }
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        i2c_rx_complete = true;
        i2c_busy = false;
        i2c_status = BQ25672_OK;
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        i2c_busy = false;
        uint32_t error_code = HAL_I2C_GetError(hi2c);
        if (error_code & HAL_I2C_ERROR_TIMEOUT) {
            i2c_status = BQ25672_COMM_FAIL;
        } else if (error_code & (HAL_I2C_ERROR_BERR | HAL_I2C_ERROR_ARLO)) {
            i2c_status = BQ25672_COMM_FAIL;
        } else {
            i2c_status = BQ25672_ERROR;
        }
        current_state = BQ25672_STATE_FAULT;
    }
}

// Handle fault conditions
static void bq25672_sm_handle_fault(I2C_HandleTypeDef *hi2c, bq25672_fault_status_t *fault) {
    if (fault->vbus_ovp || fault->vbat_ovp || fault->ibus_ocp || fault->ibat_ocp ||
        fault->conv_ocp || fault->vac1_ovp || fault->vac2_ovp || fault->vsys_short ||
        fault->vsys_ovp || fault->otg_ovp || fault->otg_uvp || fault->tshut) {
        // Disable charging and OTG
        bq25672_set_charging_state(hi2c, false);
        bq25672_set_otg_state(hi2c, false);
        current_state = BQ25672_STATE_FAULT;
    }
}

// Reset watchdog timer
static void bq25672_sm_reset_watchdog(I2C_HandleTypeDef *hi2c) {
    if (HAL_GetTick() - last_watchdog_reset >= BQ25672_WATCHDOG_RESET_MS) {
        bq25672_watchdog_config_t wd_config = current_config.watchdog;
        wd_config.reset_watchdog_now = true;
        bq25672_set_watchdog_config(hi2c, &wd_config);
        last_watchdog_reset = HAL_GetTick();
    }
}

// Check thermal conditions (JEITA)
static bq25672_status_t bq25672_sm_check_thermal(I2C_HandleTypeDef *hi2c) {
    bq25672_device_status_t status;
    bq25672_status_t ret = bq25672_get_device_status(hi2c, &status);
    if (ret != BQ25672_OK) return ret;

    if (status.advanced.ts_hot || status.advanced.ts_cold) {
        // Suspend charging on hot/cold conditions
        bq25672_set_charging_state(hi2c, false);
        current_state = BQ25672_STATE_FAULT;
        return BQ25672_ERROR;
    } else if (status.advanced.ts_warm || status.advanced.ts_cool) {
        // Apply JEITA settings
        bq25672_ntc_config_t ntc_config = current_config.ntc;
        bq25672_charge_config_t charge_config = current_config.charge;

        if (status.advanced.ts_warm) {
            charge_config.charge_voltage_max_mv -= ntc_config.vreg_reduce_mv * 100;
            if (ntc_config.ichg_hot_reduce != BQ25672_NTC_ICHG_REDUCE_UNCHANGED) {
                charge_config.charge_current_max_ma *= (100 - ntc_config.ichg_hot_reduce * 20) / 100;
            }
        } else if (status.advanced.ts_cool) {
            if (ntc_config.ichg_cold_reduce != BQ25672_NTC_COLD_UNCHANGED) {
                charge_config.charge_current_max_ma *= (100 - ntc_config.ichg_cold_reduce * 20) / 100;
            }
        }
        return bq25672_set_charge_config(hi2c, &charge_config);
    }
    return BQ25672_OK;
}

// Check ADC conditions and update state if necessary
static bool bq25672_sm_check_adc_conditions(I2C_HandleTypeDef *hi2c) {
    uint16_t vbus_mv, vbat_mv;
    int16_t ibus_ma, ibat_ma;
    bq25672_status_t status;

    // Read ADC values from BQ25672
    status = bq25672_read_vbus_voltage_mv(hi2c, &vbus_mv);
    if (status != BQ25672_OK) return false;
    status = bq25672_read_vbat_voltage_mv(hi2c, &vbat_mv);
    if (status != BQ25672_OK) return false;
    status = bq25672_read_ibus_current_ma(hi2c, &ibus_ma);
    if (status != BQ25672_OK) return false;
    status = bq25672_read_ibat_current_ma(hi2c, &ibat_ma);
    if (status != BQ25672_OK) return false;

    // Check conditions
    if (vbat_mv < VBAT_MIN_THRESHOLD_MV) {
        // Too low Vbat -> trickle charge
        bq25672_set_charging_state(hi2c, true); // Trickle şarjı etkinleştir
        current_state = BQ25672_STATE_CHARGE_TRICKLE;
        return true;
    }
    if (ibat_ma > IBAT_MAX_THRESHOLD_MA) {
        // Overcurrent, stop the charge
        bq25672_set_charging_state(hi2c, false);
        current_state = BQ25672_STATE_FAULT;
        return true;
    }
    if (vbus_mv < VBUS_MIN_THRESHOLD_MV || vbus_mv > VBUS_MAX_THRESHOLD_MV) {
        // VBUS not normal ! Fault state
        bq25672_set_charging_state(hi2c, false);
        current_state = BQ25672_STATE_FAULT;
        return true;
    }

    return false;
}

// Initialize state machine
bq25672_status_t bq25672_sm_init(I2C_HandleTypeDef *hi2c, const bq25672_config_t *config) {
    if (hi2c == NULL || config == NULL) return BQ25672_ERROR;

    // Store configuration
    memcpy(&current_config, config, sizeof(bq25672_config_t));

    // Reset device
    bq25672_status_t status = bq25672_soft_reset(hi2c);
    if (status != BQ25672_OK) return status;

    // Apply configuration
    status = bq25672_apply_config(hi2c, config);
    if (status != BQ25672_OK) return status;

    // Initialize state
    current_state = BQ25672_STATE_INIT;
    last_watchdog_reset = HAL_GetTick();

    return BQ25672_OK;
}

// Update state machine
bq25672_status_t bq25672_sm_update(I2C_HandleTypeDef *hi2c) {
    if (hi2c == NULL) return BQ25672_ERROR;

    bq25672_device_status_t dev_status;
    bq25672_fault_status_t fault_status;
    bq25672_status_t status;

    // Check ADC conditions first
    if (bq25672_sm_check_adc_conditions(hi2c)) {
        return BQ25672_OK;
    }

    // Check for faults second
    status = bq25672_get_fault_status(hi2c, &fault_status);
    if (status != BQ25672_OK) {
        current_state = BQ25672_STATE_FAULT;
        return status;
    }
    bq25672_sm_handle_fault(hi2c, &fault_status);

    // Reset watchdog periodically
    bq25672_sm_reset_watchdog(hi2c);

    // Check thermal conditions
    status = bq25672_sm_check_thermal(hi2c);
    if (status != BQ25672_OK && current_state != BQ25672_STATE_FAULT) {
        return status;
    }

    // Get device status
    status = bq25672_get_device_status(hi2c, &dev_status);
    if (status != BQ25672_OK) {
        current_state = BQ25672_STATE_FAULT;
        return status;
    }

    // State machine
    switch (current_state) {
        case BQ25672_STATE_IDLE:
            if (dev_status.vbus_present || dev_status.advanced.acrb1_placed || dev_status.advanced.acrb2_placed) {
                current_state = BQ25672_STATE_VBUS_DETECT;
            }
            break;

        case BQ25672_STATE_INIT:
            // Ensure configuration is applied
            status = bq25672_apply_config(hi2c, &current_config);
            if (status != BQ25672_OK) {
                current_state = BQ25672_STATE_FAULT;
                return status;
            }
            current_state = BQ25672_STATE_VBUS_DETECT;
            break;

        case BQ25672_STATE_VBUS_DETECT:
            if (dev_status.bc12_done) {
                // VBUS type detected, configure charge parameters
                current_state = BQ25672_STATE_CHARGE_CONFIG;
            } else if (!dev_status.vbus_present) {
                current_state = BQ25672_STATE_IDLE;
            }
            break;

        case BQ25672_STATE_CHARGE_CONFIG:
            // Enable charging
            status = bq25672_set_charging_state(hi2c, true);
            if (status != BQ25672_OK) {
                current_state = BQ25672_STATE_FAULT;
                return status;
            }
            current_state = BQ25672_STATE_CHARGE_TRICKLE;
            break;

        case BQ25672_STATE_CHARGE_TRICKLE:
            if (dev_status.charge_phase == BQ25672_CHARGE_STATUS_PRECHARGE) {
                current_state = BQ25672_STATE_CHARGE_PRECHARGE;
            } else if (dev_status.charge_phase == BQ25672_CHARGE_STATUS_NOT_CHARGING) {
                current_state = BQ25672_STATE_IDLE;
            }
            break;

        case BQ25672_STATE_CHARGE_PRECHARGE:
            if (dev_status.charge_phase == BQ25672_CHARGE_STATUS_FAST) {
                current_state = BQ25672_STATE_CHARGE_FAST;
            } else if (dev_status.charge_phase == BQ25672_CHARGE_STATUS_NOT_CHARGING) {
                current_state = BQ25672_STATE_IDLE;
            }
            break;

        case BQ25672_STATE_CHARGE_FAST:
            if (dev_status.charge_phase == BQ25672_CHARGE_STATUS_TAPER) {
                current_state = BQ25672_STATE_CHARGE_TOPOFF;
            } else if (dev_status.charge_phase == BQ25672_CHARGE_STATUS_NOT_CHARGING) {
                current_state = BQ25672_STATE_IDLE;
            }
            break;

        case BQ25672_STATE_CHARGE_TOPOFF:
            if (dev_status.charge_phase == BQ25672_CHARGE_STATUS_DONE) {
                current_state = BQ25672_STATE_CHARGE_DONE;
            } else if (dev_status.charge_phase == BQ25672_CHARGE_STATUS_NOT_CHARGING) {
                current_state = BQ25672_STATE_IDLE;
            }
            break;

        case BQ25672_STATE_CHARGE_DONE:
            if (!dev_status.vbus_present) {
                current_state = BQ25672_STATE_IDLE;
            } else if (dev_status.charge_phase != BQ25672_CHARGE_STATUS_DONE) {
                current_state = BQ25672_STATE_CHARGE_CONFIG;
            }
            break;

        case BQ25672_STATE_OTG_MODE:
            if (!dev_status.otg_mode) {
                current_state = BQ25672_STATE_IDLE;
            }
            break;

        case BQ25672_STATE_FAULT:
            // Attempt recovery by resetting
            status = bq25672_soft_reset(hi2c);
            if (status == BQ25672_OK) {
                current_state = BQ25672_STATE_INIT;
            }
            break;

        default:
            current_state = BQ25672_STATE_FAULT;
            break;
    }

    return BQ25672_OK;
}

// Get current state
bq25672_status_t bq25672_sm_get_state(bq25672_state_t *state) {
    if (state == NULL) return BQ25672_ERROR;
    *state = current_state;
    return BQ25672_OK;
}

// Force state transition
bq25672_status_t bq25672_sm_force_state(I2C_HandleTypeDef *hi2c, bq25672_state_t state) {
    if (hi2c == NULL) return BQ25672_ERROR;

    switch (state) {
        case BQ25672_STATE_OTG_MODE:
            // Enable OTG
            return bq25672_set_otg_state(hi2c, true);
        case BQ25672_STATE_IDLE:
            // Disable charging and OTG
            bq25672_set_charging_state(hi2c, false);
            bq25672_set_otg_state(hi2c, false);
            current_state = BQ25672_STATE_IDLE;
            return BQ25672_OK;
        case BQ25672_STATE_INIT:
            // Reinitialize
            return bq25672_sm_init(hi2c, &current_config);
        default:
            return BQ25672_ERROR;
    }
}
