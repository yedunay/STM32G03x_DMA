/*
 * bq25672_sm.h
 *
 *  Created on: Jul 24, 2025
 *      Author: YED
 */

#ifndef INC_BQ25672_SM_H_
#define INC_BQ25672_SM_H_

#include "bq25672.h"
#include "stm32g0xx_hal.h"
#include <string.h>

/* #######################################################################################################*\
 *  																									   *
 * 									           ENUMS   													   *
 *  																									   *
\*########################################################################################################*/

typedef enum {
    BQ25672_STATE_IDLE = 0,          ///< No power or device not initialized
    BQ25672_STATE_INIT,              ///< Initializing device with configuration
    BQ25672_STATE_VBUS_DETECT,       ///< Detecting VBUS/VAC1/VAC2 and BC1.2
    BQ25672_STATE_CHARGE_CONFIG,     ///< Configuring charge parameters
    BQ25672_STATE_CHARGE_TRICKLE,    ///< Trickle charge phase
    BQ25672_STATE_CHARGE_PRECHARGE,  ///< Precharge phase
    BQ25672_STATE_CHARGE_FAST,       ///< Fast charge (CC/CV) phase
    BQ25672_STATE_CHARGE_TOPOFF,     ///< Top-off timer active
    BQ25672_STATE_CHARGE_DONE,       ///< Charge completed
    BQ25672_STATE_OTG_MODE,          ///< OTG (boost) mode active
    BQ25672_STATE_FAULT              ///< Fault condition (OVP, OCP, TSHUT, etc.)
} bq25672_state_t;

/* #######################################################################################################*\
 *  																									   *
 * 									          FUNCTIONS												   *
 *  																									   *
\*########################################################################################################*/

/**
 * @brief Initialize the state machine with a given configuration
 * @param hi2c I2C handle
 * @param config Pointer to BQ25672 configuration
 * @return bq25672_status_t Operation status
 */
bq25672_status_t bq25672_sm_init(I2C_HandleTypeDef *hi2c, const bq25672_config_t *config);

/**
 * @brief Update the state machine (to be called periodically)
 * @param hi2c I2C handle
 * @return bq25672_status_t Operation status
 */
bq25672_status_t bq25672_sm_update(I2C_HandleTypeDef *hi2c);

/**
 * @brief Get the current state of the state machine
 * @param state Pointer to store the current state
 * @return bq25672_status_t Operation status
 */
bq25672_status_t bq25672_sm_get_state(bq25672_state_t *state);

/**
 * @brief Force transition to a specific state (e.g., for OTG or fault recovery)
 * @param hi2c I2C handle
 * @param state Target state
 * @return bq25672_status_t Operation status
 */
bq25672_status_t bq25672_sm_force_state(I2C_HandleTypeDef *hi2c, bq25672_state_t state);

#endif /* INC_BQ25672_SM_H_ */
