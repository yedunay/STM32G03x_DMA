/*
 * bq25672.h
 *
 *  Created on: Jul 9, 2025
 *      Author: YED
 */
#ifndef INC_BQ25672_H_
#define INC_BQ25672_H_

#include <stdbool.h>


/* #######################################################################################################*\
 *  																									   *
 * 									           ENUMS   													   *
 *  																									   *
\*########################################################################################################*/

typedef enum
{
    BQ25672_OK             =  0,   ///< Operation successful
    BQ25672_ERROR          = -1,   ///< General failure
    BQ25672_COMM_FAIL      = -2    ///< I2C transaction failed
} bq25672_status_t;

typedef enum
{
    BQ25672_PRECHARGE_15PCT  = 0x0,  // Pre-charge to 15% of charge voltage
    BQ25672_PRECHARGE_62P2   = 0x1,  // Pre-charge to 62.2% of charge voltage
    BQ25672_PRECHARGE_66P7   = 0x2,  // Pre-charge to 66.7% of charge voltage
    BQ25672_PRECHARGE_71P4   = 0x3   // Pre-charge to 71.4% of charge voltage
} bq25672_precharge_pct_t;

typedef enum
{
    BQ25672_TOPOFF_DISABLED = 0,
    BQ25672_TOPOFF_15MIN     = 1,
    BQ25672_TOPOFF_30MIN     = 2,
    BQ25672_TOPOFF_45MIN     = 3
} bq25672_topoff_time_t;

typedef enum
{
    BQ25672_FASTCHG_5H  = 0,
    BQ25672_FASTCHG_8H  = 1,
    BQ25672_FASTCHG_12H = 2,
    BQ25672_FASTCHG_24H = 3
} bq25672_fastchg_time_t;

typedef enum
{
    BQ25672_WATCHDOG_DISABLE = 0x0, ///< Disable
    BQ25672_WATCHDOG_0_5S    = 0x1, ///< 0.5 seconds
    BQ25672_WATCHDOG_1S      = 0x2, ///< 1 second
    BQ25672_WATCHDOG_2S      = 0x3, ///< 2 seconds
    BQ25672_WATCHDOG_20S     = 0x4, ///< 20 seconds
    BQ25672_WATCHDOG_40S     = 0x5, ///< 40 seconds
    BQ25672_WATCHDOG_80S     = 0x6, ///< 80 seconds
    BQ25672_WATCHDOG_160S    = 0x7  ///< 160 seconds
} bq25672_watchdog_timeout_t;

typedef enum
{
    BQ25672_MPPT_56P25 = 0x0, ///< 56.25%
    BQ25672_MPPT_62P5  = 0x1, ///< 62.5%
    BQ25672_MPPT_68P75 = 0x2, ///< 68.75%
    BQ25672_MPPT_75P0  = 0x3, ///< 75.0%
    BQ25672_MPPT_81P25 = 0x4, ///< 81.25%
    BQ25672_MPPT_87P5  = 0x5, ///< 87.5%
    BQ25672_MPPT_93P75 = 0x6, ///< 93.75%
    BQ25672_MPPT_100P0 = 0x7  ///< 100.0%
} bq25672_mppt_pct_t;

typedef enum {
    BQ25672_NTC_VREDUCE_SUSPEND   = 0x0, ///< Charge suspend
    BQ25672_NTC_VREDUCE_800MV     = 0x1, ///< VREG - 800mV
    BQ25672_NTC_VREDUCE_600MV     = 0x2, ///< VREG - 600mV
    BQ25672_NTC_VREDUCE_400MV     = 0x3, ///< VREG - 400mV (default)
    BQ25672_NTC_VREDUCE_300MV     = 0x4, ///< VREG - 300mV
    BQ25672_NTC_VREDUCE_200MV     = 0x5, ///< VREG - 200mV
    BQ25672_NTC_VREDUCE_100MV     = 0x6, ///< VREG - 100mV
    BQ25672_NTC_VREDUCE_UNCHANGED = 0x7  ///< VREG unchanged
} bq25672_ntc_vreduce_t;

typedef enum {
    BQ25672_NTC_ICHG_REDUCE_SUSPEND   = 0x0, ///< Charge suspend
    BQ25672_NTC_ICHG_REDUCE_20PERCENT = 0x1, ///< ICHG 20%
    BQ25672_NTC_ICHG_REDUCE_40PERCENT = 0x2, ///< ICHG 40%
    BQ25672_NTC_ICHG_REDUCE_UNCHANGED = 0x3  ///< ICHG unchanged (default)
} bq25672_ntc_ichg_reduce_t;

typedef enum {
    BQ25672_NTC_COLD_SUSPEND     = 0x0, ///< Charge suspend
    BQ25672_NTC_COLD_20PERCENT   = 0x1, ///< ICHG 20% (default)
    BQ25672_NTC_COLD_40PERCENT   = 0x2, ///< ICHG 40%
    BQ25672_NTC_COLD_UNCHANGED   = 0x3  ///< ICHG unchanged
} bq25672_ntc_cold_current_t;

typedef enum {
    BQ25672_TREG_60C  = 0x0, ///< 60°C
    BQ25672_TREG_80C  = 0x1, ///< 80°C
    BQ25672_TREG_100C = 0x2, ///< 100°C
    BQ25672_TREG_120C = 0x3  ///< 120°C (default)
} bq25672_temp_regulation_t;

typedef enum {
    BQ25672_TSHUT_150C = 0x0, ///< 150°C (default)
    BQ25672_TSHUT_130C = 0x1, ///< 130°C
    BQ25672_TSHUT_120C = 0x2, ///< 120°C
    BQ25672_TSHUT_85C  = 0x3  ///< 85°C
} bq25672_temp_shutdown_t;

/**
 * @brief Input Current Optimizer (ICO) status definitions. (REG1D[7:6])
 */
typedef enum
{
    BQ25672_ICO_DISABLED      = 0, ///< ICO disabled
    BQ25672_ICO_IN_PROGRESS   = 1, ///< ICO optimization in progress
    BQ25672_ICO_PEAK_DETECTED = 2, ///< Maximum input current detected
    BQ25672_ICO_RESERVED      = 3  ///< Reserved (invalid)
} bq25672_ico_status_t;

/**
 * @brief Charge status definitions. (REG1C[7:5])
 */
typedef enum
{
    BQ25672_CHARGE_STATUS_NOT_CHARGING = 0, ///< 0h: Not charging
    BQ25672_CHARGE_STATUS_TRICKLE      = 1, ///< 1h: Trickle charge
    BQ25672_CHARGE_STATUS_PRECHARGE    = 2, ///< 2h: Pre-charge
    BQ25672_CHARGE_STATUS_FAST         = 3, ///< 3h: Fast charge (CC mode)
    BQ25672_CHARGE_STATUS_TAPER        = 4, ///< 4h: Taper charge (CV mode)
    BQ25672_CHARGE_STATUS_RESERVED1    = 5, ///< Reserved
    BQ25672_CHARGE_STATUS_TOP_OFF      = 6, ///< 6h: Top-off timer active charging
    BQ25672_CHARGE_STATUS_DONE         = 7  ///< 7h: Charge Termination Done
} bq25672_charge_status_t;

/**
 * @brief VBUS status definitions. (REG1C[4:1])
 */
typedef enum
{
    BQ25672_VBUS_NONE         = 0x0, ///< No input / BHOT / BCOLD in OTG
    BQ25672_VBUS_USB_SDP      = 0x1, ///< USB SDP (500mA)
    BQ25672_VBUS_USB_CDP      = 0x2, ///< USB CDP (1.5A)
    BQ25672_VBUS_USB_DCP      = 0x3, ///< USB DCP (3.25A)
    BQ25672_VBUS_HVDCP        = 0x4, ///< Adjustable High Voltage DCP (1.5A)
    BQ25672_VBUS_UNKNOWN_3A   = 0x5, ///< Unknown 3A adaptor
    BQ25672_VBUS_NON_STANDARD = 0x6, ///< Non-standard adaptor (1/2/2.1/2.4A)
    BQ25672_VBUS_OTG          = 0x7, ///< OTG mode
    BQ25672_VBUS_NOT_QUALIFIED= 0x8, ///< Not qualified adaptor
    BQ25672_VBUS_RESERVED9    = 0x9, ///< Reserved
    BQ25672_VBUS_RESERVEDA    = 0xA, ///< Reserved
    BQ25672_VBUS_DIRECT_VBUS  = 0xB, ///< Device powered from VBUS
    BQ25672_VBUS_BACKUP_MODE  = 0xC, ///< Backup Mode
    BQ25672_VBUS_RESERVEDD    = 0xD, ///< Reserved
    BQ25672_VBUS_RESERVEDE    = 0xE, ///< Reserved
    BQ25672_VBUS_RESERVEDF    = 0xF  ///< Reserved
} bq25672_vbus_status_t;


/* #######################################################################################################*\
 *  																									   *
 * 									           STRUCTS												       *
 *  																									   *
\*########################################################################################################*/

typedef struct
{
    bool enable_termination; ///< Enable battery charge termination
    bool enable_hiz;         ///< Enable HIZ mode (zero input current)
    bool force_ico;          ///< Force input current optimization
    bool enable_ico;         ///< Enable ICO mode
    bool disable_pfm;        ///< Disable Pulse Frequency Modulation
    bool disable_ldo;        ///< Disable internal LDO regulator
    bool disable_ooa;        ///< Disable out-of-audio-mode (light load optimization)
    bool use_1500kHz;        ///< Use 1.5 MHz switching frequency instead of 750 kHz
    bool enable_acdrv1;      ///< Enable external ACDRV1 gate driver
    bool enable_acdrv2;      ///< Enable external ACDRV2 gate driver
} bq25672_system_config_t;

typedef struct
{
    uint16_t charge_voltage_max_mv;   // Charge voltage limit in millivolts (e.g., 8400 = 8.4V)
    uint16_t charge_current_max_ma;   // Charge current limit in milliamperes (e.g., 1000 = 1A)

    struct
    {
        uint16_t input_voltage_max_mv;       // VINDPM: maximum input voltage (mV), e.g., 9000
        uint16_t input_voltage_min_mv;       // VBUS UVLO: minimum input voltage (mV), usually 3200–3600
        uint16_t input_current_max_ma;       // IINDPM: maximum input current (mA), e.g., 2000
        uint8_t  termination_current_ma;     // ITERM: termination current threshold (mA), e.g., 50
        bq25672_precharge_pct_t precharge_pct; // VRECHG: recharge threshold as percentage of full voltage

    } advanced;

} bq25672_charge_config_t;

typedef struct
{
    bool                        enable_otg;       ///< Enable OTG
    uint16_t                    voltage_mv;       ///< VOTG target (mV)
    uint16_t                    current_max_ma;   ///< IOTG limit (mA)
    struct
    {
        bool disable_pfm;      ///< Disable PFM in OTG
        bool disable_ooa;      ///< Disable OOA in OTG
    } advanced;
} bq25672_usb_config_t;

typedef struct
{
    bq25672_topoff_time_t    top_off_time;       ///< Top-off timer duration
    bool                     trickle_enable;     ///< Enable trickle timer (1h)
    bool                     fast_charge_enable; ///< Enable fast-charge timer
    bq25672_fastchg_time_t    fast_charge_time;  ///< Fast-charge timer duration
    struct
    {
        bool precharge_enable; ///< Enable pre-charge timer
        bool timer_slow;       ///< Enable timer slowdown (2x mode under DPM / Thermal)
    } advanced;
} bq25672_timer_config_t;

typedef struct
{
    bq25672_watchdog_timeout_t timeout;   ///< Watchdog timeout period
    bool reset_watchdog_now;              ///< Force immediate watchdog reset (REG10)
    bool disable_on_expiry;               ///< Stop charging when WDT expires (REG09)
} bq25672_watchdog_config_t;

typedef struct
{
    /* Charger Event Interrupt Masks (REG28) */
    bool iindpm;        ///< Enable IINDPM / IOTG interrupt
    bool vindpm;        ///< Enable VINDPM / VOTG interrupt
    bool watchdog;      ///< Enable I2C watchdog timer interrupt
    bool poor_source;   ///< Enable Poor Source interrupt
    bool power_good;    ///< Enable Power Good interrupt
    bool ac2_present;   ///< Enable VAC2 Present interrupt
    bool ac1_present;   ///< Enable VAC1 Present interrupt
    bool vbus_present;  ///< Enable VBUS Present interrupt

    /* Fault Event Interrupt Masks (REG2C) */
    bool ibat_reg;      ///< Enable IBAT regulation fault interrupt
    bool vbus_ovp;      ///< Enable VBUS over-voltage fault interrupt
    bool vbat_ovp;      ///< Enable VBAT over-voltage fault interrupt
    bool ibus_ocp;      ///< Enable IBUS over-current fault interrupt
    bool ibat_ocp;      ///< Enable IBAT over-current fault interrupt
    bool conv_ocp;      ///< Enable Converter over-current fault interrupt
    bool vac2_ovp;      ///< Enable VAC2 over-voltage fault interrupt
    bool vac1_ovp;      ///< Enable VAC1 over-voltage fault interrupt

} bq25672_interrupt_config_t;

typedef struct {
    bool enable_mppt;                  ///< Enable MPPT VOC measurement
    bq25672_mppt_pct_t ratio;          ///< VOC target ratio

    struct {
        uint32_t delay_ms;             ///< Delay after switching (0: 50ms, 1: 300ms, 2: 2s, 3: 5s)
        uint32_t interval_s;           ///< VOC measurement interval (0: 30s, 1: 2min, 2: 10min, 3: 30min)
    } advanced;
} bq25672_mppt_config_t;

typedef struct
{
    bq25672_ntc_vreduce_t      vreg_reduce_mv; ///< JEITA High Temp VREG reduction
    bq25672_ntc_ichg_reduce_t  ichg_hot_reduce;///< JEITA High Temp current reduction
    bq25672_ntc_cold_current_t ichg_cold_reduce; ///< JEITA Low Temp current reduction

    uint8_t ts_cool_deg; ///< TS cool threshold (rising)
    uint8_t ts_warm_deg; ///< TS warm threshold (falling)
    uint8_t bhot_deg;    ///< OTG BHOT threshold
    bool    bcold_20c;   ///< OTG BCOLD threshold: 0 = -10°C, 1 = -20°C
    bool    ignore_ts;   ///< Ignore TS comparators entirely
} bq25672_ntc_config_t;

typedef struct
{
    bq25672_temp_regulation_t treg;   ///< Thermal regulation threshold (°C)
    bq25672_temp_shutdown_t   tshut;  ///< Thermal shutdown threshold (°C)
    bool vbus_pd_en;                  ///< Enable VBUS pull-down resistor
    bool vac1_pd_en;                  ///< Enable VAC1 pull-down resistor
    bool vac2_pd_en;                  ///< Enable VAC2 pull-down resistor
} bq25672_temp_config_t;

typedef struct
{
    bq25672_system_config_t     system;    ///< System behaviors: EN_HIZ, EN_TERM, ACDRV, PFM etc.
    bq25672_charge_config_t     charge;    ///< Charge settings: voltages, currents, input limits
    bq25672_usb_config_t        usb;       ///< USB OTG settings: IOTG, VOTG, EN_OTG
    bq25672_timer_config_t      timer;     ///< Charging timer enables: fast charge, precharge, top-off
    bq25672_watchdog_config_t   watchdog;  ///< I2C watchdog timer
    bq25672_mppt_config_t       mppt;      ///< Maximum Power Point Tracking parameters
    bq25672_ntc_config_t        ntc;       ///< JEITA NTC thresholds, TS comparator, OTG thresholds
    bq25672_temp_config_t       jeita;     ///< Temperature regulation (TREG), shutdown (TSHUT), pull-downs
    bq25672_interrupt_config_t  interrupt; ///< Charger and fault interrupt masks
} bq25672_config_t;

typedef struct
{
    bool vbus_present;        ///< VBUS_PRESENT_STAT (REG1D[0]): VBUS present
    bool charger_active;      ///< CHARGER_ACTIVE_STAT (REG1D[6]): Charger active
    bool otg_mode;            ///< OTG_MODE_STAT (REG1D[5]): OTG mode active
    bool bc12_done;           ///< BC1.2_DONE_STAT (REG1C[0]): BC1.2 detection complete

    bq25672_charge_status_t charge_phase; ///< Charging phase status (REG1C[7:5])
    bq25672_vbus_status_t vbus_type;      ///< VBUS adapter type (REG1C[4:1])

    struct
    {
        bq25672_ico_status_t ico_state;     ///< ICO status (REG1D[7:6])
        bool thermal_regulation;            ///< Thermal regulation active (REG1D[2])
        bool dpdm_detection;                ///< D+/D- detection ongoing (REG1D[1])
        bool vbat_present;                  ///< VBAT present (REG1D[0])

        bool acrb2_placed;      ///< ACDRV2 FET active (REG1E[7])
        bool acrb1_placed;      ///< ACDRV1 FET active (REG1E[6])
        bool adc_done;          ///< ADC one-shot complete (REG1E[5])
        bool vsys_regulation;   ///< VSYS regulation active (REG1E[4])
        bool fast_timer_expired;///< Fast charge safety timer expired (REG1E[3])
        bool trickle_timer_expired; ///< Trickle charge timer expired (REG1E[2])
        bool precharge_timer_expired; ///< Precharge timer expired (REG1E[1])

        bool vbat_otg_too_low; ///< VBAT too low for OTG (REG1F[4])
        bool ts_cold;          ///< TS cold region active (REG1F[3])
        bool ts_cool;          ///< TS cool region active (REG1F[2])
        bool ts_warm;          ///< TS warm region active (REG1F[1])
        bool ts_hot;           ///< TS hot region active (REG1F[0])
    } advanced;
} bq25672_device_status_t;

typedef struct
{
    bool ibat_regulation;    ///< REG26[7]: IBAT regulation fault flag
    bool vbus_ovp;           ///< REG26[6]: VBUS over-voltage protection triggered
    bool vbat_ovp;           ///< REG26[5]: VBAT over-voltage protection triggered
    bool ibus_ocp;           ///< REG26[4]: IBUS over-current protection triggered
    bool ibat_ocp;           ///< REG26[3]: IBAT over-current protection triggered
    bool conv_ocp;           ///< REG26[2]: Converter over-current protection triggered
    bool vac2_ovp;           ///< REG26[1]: VAC2 input over-voltage protection triggered
    bool vac1_ovp;           ///< REG26[0]: VAC1 input over-voltage protection triggered

    bool vsys_short;         ///< REG27[7]: VSYS short-circuit protection triggered
    bool vsys_ovp;           ///< REG27[6]: VSYS over-voltage protection triggered
    bool otg_ovp;            ///< REG27[5]: OTG over-voltage protection triggered
    bool otg_uvp;            ///< REG27[4]: OTG under-voltage protection triggered
    bool tshut;              ///< REG27[2]: Thermal shutdown triggered
} bq25672_fault_status_t;

/* #######################################################################################################*\
 *  																									   *
 * 									           CONFIGS													   *
 *  																									   *
\*########################################################################################################*/

/** @brief High-level default driver configuration (Datasheet Reset Defaults) */
static const bq25672_config_t BQ25672_DEFAULT_CONFIG =
{
    .system =
    {
        .enable_termination = true,     ///< EN_TERM enabled (default)
        .enable_hiz         = false,    ///< HIZ disabled by default also disabled when the adapter plugged in VBUS
        .force_ico          = false,    ///< ICO force disabled and this bit only valid when EN_ICO=1
        .enable_ico         = false,    ///< ICO disabled by default (Input Current Optimizer)
        .disable_pfm        = false,    ///< PFM enabled by default
        .disable_ldo        = false,    ///< LDO enabled by default
        .disable_ooa        = false,    ///< OOA enabled by default
        .use_1500kHz        = false,    ///< Default 750kHz chosen (true = 1.5kHz)
        .enable_acdrv1      = false,    ///< External FET ACDRV1 disabled
        .enable_acdrv2      = false     ///< External FET ACDRV2 disabled
    },

    .charge =
    {
        .charge_voltage_max_mv   = 5000u,   ///< VREG chosen 5V
        .charge_current_max_ma   = 1000u,   ///< ICHG default 1A
        .advanced = {
            .input_voltage_max_mv    = 22000u,  ///< VINDPM default 22.0V
            .input_voltage_min_mv    = 3600u,  ///< VAC UVLO ~3.6V (fixed hardware)
            .input_current_max_ma    = 3000u,   ///< IINDPM default 3000mA
            .termination_current_ma  = 200u,   ///< ITERM default 200mA
            .precharge_pct           = BQ25672_PRECHARGE_71P4 ///< default 71.4%*VREG precharge
        }
    },

    .usb =
    {
        .enable_otg     = false, ///< OTG disabled
        .voltage_mv     = 5000u, ///< OTG VBUS 5.0V
        .current_max_ma = 3040u, ///< OTG current limit 3040mA
        .advanced =
        {
            .disable_pfm = false, ///< PFM allowed in OTG mode
            .disable_ooa = false  ///< OOA allowed in OTG mode
        }
    },

    .timer =
    {
        .top_off_time        = BQ25672_TOPOFF_DISABLED, ///< Top-off disabled (default)
        .trickle_enable      = true,   ///< Trickle timer enabled
        .fast_charge_enable  = true,   ///< Fast charge timer enabled
        .fast_charge_time    = BQ25672_FASTCHG_12H, ///< 12h fast charge timer (default)
        .advanced = {
            .precharge_enable = true,  ///< Precharge timer enabled
            .timer_slow       = true  ///< Timer slowdown enabled  (slowed 2x during input DPM or thermal regualtion)
        }
    },

    .watchdog =
    {
        .timeout           = BQ25672_WATCHDOG_40S, ///< 40s watchdog timeout (default)
        .reset_watchdog_now= false,                ///< No immediate reset request
        .disable_on_expiry = false                 ///< WDT expiry does NOT stop charging
    },

    .mppt =
    {
        .enable_mppt = false, ///< MPPT disabled (default)
        .ratio       = BQ25672_MPPT_87P5, ///< 0.875 VOC
        .advanced =
        {
            .delay_ms   = 300u, 	///< 0.3s delay default
            .interval_s = 120u    ///< 2 mins interval default
        }
    },

    .ntc =
    {
        .vreg_reduce_mv   = BQ25672_NTC_VREDUCE_400MV, ///< VREG - 400mV (default)
        .ichg_hot_reduce  = BQ25672_NTC_ICHG_REDUCE_UNCHANGED, ///< No change on hot
        .ichg_cold_reduce = BQ25672_NTC_COLD_20PERCENT, ///< ICHG reduced 20% in cold (default)
        .ts_cool_deg      = 10u,   ///< 1h = (%68.4 - 10°C))
        .ts_warm_deg      = 45u,   ///< 1h = 44.8% (45°C)
        .bhot_deg         = 60u,   ///< 1h = 60°C
        .bcold_20c        = false, ///< -10°C threshold selected
        .ignore_ts        = false  ///< TS comparator active (default)
    },

    .jeita =
    {
        .treg   = BQ25672_TREG_120C, ///< 120°C regulation (default)
        .tshut  = BQ25672_TSHUT_150C,///< 150°C shutdown (default)
        .vbus_pd_en = false,          ///< VBUS pull-down disabled by default
        .vac1_pd_en = false,          ///< VAC1 pull-down disabled by default
        .vac2_pd_en = false           ///< VAC2 pull-down disabled by default
    },

    .interrupt =
    {
        // REG28 - Charger events mask
        .iindpm      = false,
        .vindpm      = false,
        .watchdog    = false,
        .poor_source = false,
        .power_good  = false,
        .ac2_present = false,
        .ac1_present = false,
        .vbus_present= false,

        // REG2C - Fault events mask
        .ibat_reg = false,
        .vbus_ovp = false,
        .vbat_ovp = false,
        .ibus_ocp = false,
        .ibat_ocp = false,
        .conv_ocp = false,
        .vac2_ovp = false,
        .vac1_ovp = false
    }
};

/** @brief Optimized configuration for maximum efficiency operation (No OTG, highest charger efficiency) */
static const bq25672_config_t BQ25672_MAX_EFFICIENCY_CONFIG =
{
    .system =
    {
        .enable_termination = true,     ///< Enable charge termination when battery full (saves energy)
        .enable_hiz         = false,    ///< HIZ disabled, input path always active for max efficiency
        .force_ico          = true,     ///< Force ICO to optimize input current at startup
        .enable_ico         = true,     ///< Enable ICO continuously for best input performance
        .disable_pfm        = true,     ///< PFM disabled for lower ripple, fixed-frequency continuous mode (better EMI, efficiency)
        .disable_ldo        = false,    ///< LDO enabled (default behavior), no benefit in disabling here
        .disable_ooa        = true,     ///< OOA disabled to ensure no light-load pulse skipping, smooth ripple for efficiency
        .use_1500kHz        = false,    ///< Use 750kHz switching for highest efficiency (lower switching losses than 1.5MHz)
        .enable_acdrv1      = false,    ///< External ACDRV1 gate driver not used
        .enable_acdrv2      = false     ///< External ACDRV2 gate driver not used
    },

    .charge =
    {
        .charge_voltage_max_mv   = 4200u, ///< 4.2V battery full voltage; standard lithium-ion value
        .charge_current_max_ma   = 2000u, ///< 2A charging for optimal efficiency balance (datasheet efficiency peaks)
        .advanced = {
            .input_voltage_max_mv    = 9000u,  ///< 9V adapter input for optimal buck conversion efficiency
            .input_voltage_min_mv    = 3600u,  ///< UVLO fixed hardware limit at 3.6V
            .input_current_max_ma    = 3000u,  ///< Max input current limit 3A to fully utilize adapter capability
            .termination_current_ma  = 100u,   ///< 100mA termination current for precise charge completion
            .precharge_pct           = BQ25672_PRECHARGE_15PCT ///< 15% VREG precharge voltage threshold
        }
    },

    .usb =
    {
        .enable_otg     = false, ///< OTG/Boost disabled for max efficiency in charger-only use case
        .voltage_mv     = 5000u, ///< Unused, kept at default
        .current_max_ma = 0u,    ///< No OTG current required
        .advanced =
        {
            .disable_pfm = true, ///< Disable PFM for clean OTG if it were used; no impact here
            .disable_ooa = true  ///< Disable OOA for OTG if active; no impact here
        }
    },

    .timer =
    {
        .top_off_time        = BQ25672_TOPOFF_15MIN, ///< Short 15min top-off to avoid excess trickle after full
        .trickle_enable      = true,    ///< Enable trickle timer for safety
        .fast_charge_enable  = true,    ///< Enable fast charge timer to protect battery
        .fast_charge_time    = BQ25672_FASTCHG_8H, ///< 8h max fast charge to limit battery stress
        .advanced = {
            .precharge_enable = true, ///< Enable precharge timer to limit time at low battery voltage
            .timer_slow       = false ///< Disable slowdown for maximum efficiency under stable conditions
        }
    },

    .watchdog =
    {
        .timeout           = BQ25672_WATCHDOG_40S, ///< 40s timeout typical for I2C keep-alive
        .reset_watchdog_now= false,                ///< No forced reset needed here
        .disable_on_expiry = false                 ///< Do not stop charging on watchdog expiry, keep charging safe
    },

    .mppt =
    {
        .enable_mppt = false, ///< MPPT disabled, not useful with stable DC adapter
        .ratio       = BQ25672_MPPT_75P0, ///< Default placeholder
        .advanced =
        {
            .delay_ms   = 50u, 	///< 50ms delay setting default, unused
            .interval_s = 30u    ///< 30s interval default, unused
        }
    },

    .ntc =
    {
        .vreg_reduce_mv   = BQ25672_NTC_VREDUCE_UNCHANGED, ///< No VREG derating via NTC
        .ichg_hot_reduce  = BQ25672_NTC_ICHG_REDUCE_UNCHANGED, ///< No current reduction hot
        .ichg_cold_reduce = BQ25672_NTC_COLD_20PERCENT, ///< Light cold derating 20%
        .ts_cool_deg      = 10u,   ///< Cool threshold ~10°C typical
        .ts_warm_deg      = 45u,   ///< Warm threshold ~45°C typical
        .bhot_deg         = 60u,   ///< BHOT threshold ~60°C typical
        .bcold_20c        = false, ///< -10°C selected for BCOLD threshold
        .ignore_ts        = false  ///< TS active for battery protection
    },

    .jeita =
    {
        .treg   = BQ25672_TREG_120C, ///< Thermal regulation at 120°C standard
        .tshut  = BQ25672_TSHUT_150C,///< Shutdown at 150°C for safety
        .vbus_pd_en = false,          ///< No VBUS pulldown needed
        .vac1_pd_en = false,          ///< No VAC1 pulldown needed
        .vac2_pd_en = false           ///< No VAC2 pulldown needed
    },

    .interrupt =
    {
        .iindpm      = false, ///< Enable interrupt for input current DPM events
        .vindpm      = false, ///< Enable interrupt for input voltage DPM events
        .watchdog    = false, ///< Enable watchdog interrupt
        .poor_source = false, ///< Enable poor source detection interrupt
        .power_good  = false, ///< Enable PG status change interrupt
        .ac2_present = false, ///< Enable VAC2 present detection interrupt
        .ac1_present = false, ///< Enable VAC1 present detection interrupt
        .vbus_present= false, ///< Enable VBUS present detection interrupt

        .ibat_reg = false, ///< Enable IBAT regulation fault interrupt
        .vbus_ovp = false, ///< Enable VBUS OVP fault interrupt
        .vbat_ovp = false, ///< Enable VBAT OVP fault interrupt
        .ibus_ocp = false, ///< Enable IBUS OCP fault interrupt
        .ibat_ocp = false, ///< Enable IBAT OCP fault interrupt
        .conv_ocp = false, ///< Enable converter OCP fault interrupt
        .vac2_ovp = false, ///< Enable VAC2 OVP fault interrupt
        .vac1_ovp = false  ///< Enable VAC1 OVP fault interrupt
    }
};


/* #######################################################################################################*\
 *  																									   *
 * 									           REGISTERS												   *
 *  																									   *
\*########################################################################################################*/

#define REG00_Minimal_System_Voltage    0x00  // VSYSMIN: Minimum system voltage limit
#define REG01_Charge_Voltage_Limit_L    0x01  // VREG[7:0]: Charge voltage limit LSB
#define REG02_Charge_Voltage_Limit_H    0x02  // VREG[10:8]: Charge voltage limit MSB (bits 2:0)
#define REG03_Charge_Current_Limit_L    0x03  // ICHG[7:0]: Charge current limit LSB
#define REG04_Charge_Current_Limit_H    0x04  // ICHG[8]: Charge current limit MSB (bit 0)
#define REG05_Input_Voltage_Limit       0x05  // VINDPM: Input voltage limit
#define REG06_Input_Current_Limit       0x06  // IINDPM: Input current limit
#define REG08_Precharge_Control         0x08  // Pre-charge current and threshold
#define REG09_Termination_Control       0x09  // Termination current and disable bit
#define REG0A_Recharge_Control          0x0A  // CELL config + recharge thresholds
#define REG0B_VOTG_Regulation           0x0B  // OTG mode voltage regulation
#define REG0D_IOTG_Regulation           0x0D  // OTG mode current regulation
#define REG0E_Timer_Control             0x0E  // Precharge, fast charge timers
#define REG0F_Charger_Control_0         0x0F  // HIZ, CE, watchdog, STAT config
#define REG10_Charger_Control_1         0x10  // Battery detection, force charge, input FETs
#define REG11_Charger_Control_2         0x11  // Safety timers, current foldback, JEITA enable
#define REG12_Charger_Control_3         0x12  // OTG enable, LDO_CTRL, ILIM_HIZ enable
#define REG13_Charger_Control_4         0x13  // ADC enable, TS configuration
#define REG14_Charger_Control_5         0x14  // IBAT compensation, battery missing action
#define REG15_MPPT_Control              0x15  // Maximum power point tracking (MPPT) control
#define REG16_Temperature_Control       0x16  // JEITA temperature ranges configuration
#define REG17_NTC_Control_0             0x17  // TS cold/hot thresholds
#define REG18_NTC_Control_1             0x18  // TS warm/cool thresholds
#define REG19_ICO_Current_Limit         0x19  // ICO: Input current optimizer limit
#define REG1B_Charger_Status_0          0x1B  // Status flags: PG, charge state, input status
#define REG1C_Charger_Status_1          0x1C  // Input source type, D+/D- status
#define REG1D_Charger_Status_2          0x1D  // ADC battery, VBUS, SYS voltages
#define REG1E_Charger_Status_3          0x1E  // IBAT, IIN current sense readings
#define REG1F_Charger_Status_4          0x1F  // TS voltage, temperature status
#define REG20_FAULT_Status_0            0x20  // Input/charging faults
#define REG21_FAULT_Status_1            0x21  // TS, JEITA, SYS faults
#define REG22_Charger_Flag_0            0x22  // Flag bits: charge ready, PG, ADC
#define REG23_Charger_Flag_1            0x23  // Input flags, D+/D-, source
#define REG24_Charger_Flag_2            0x24  // ADC data flags
#define REG25_Charger_Flag_3            0x25  // TS & temp zone flags
#define REG26_FAULT_Flag_0              0x26  // Fault flags: input, charger, OTG
#define REG27_FAULT_Flag_1              0x27  // Fault flags: TS, ADC, SYS
#define REG28_Charger_Mask_0            0x28  // Mask register for Charger_Flag_0
#define REG29_Charger_Mask_1            0x29  // Mask register for Charger_Flag_1
#define REG2A_Charger_Mask_2            0x2A  // Mask register for Charger_Flag_2
#define REG2B_Charger_Mask_3            0x2B  // Mask register for Charger_Flag_3
#define REG2C_FAULT_Mask_0              0x2C  // Mask register for FAULT_Flag_0
#define REG2D_FAULT_Mask_1              0x2D  // Mask register for FAULT_Flag_1
#define REG2E_ADC_Control               0x2E  // Controls ADC function, start/stop conversions and continuous mode
#define REG2F_ADC_Function_Disable_0    0x2F  // Disables specific ADC functions (group 0)
#define REG30_ADC_Function_Disable_1    0x30  // Disables specific ADC functions (group 1)
#define REG31_IBUS_ADC_L                0x31  // VBUS input current (LSB)
#define REG32_IBUS_ADC_H                0x32  // VBUS input current (MSB)
#define REG33_IBAT_ADC_L                0x33  // Battery current (LSB)
#define REG34_IBAT_ADC_H                0x34  // Battery current (MSB)
#define REG35_VBUS_ADC_L                0x35  // VBUS voltage (LSB)
#define REG36_VBUS_ADC_H                0x36  // VBUS voltage (MSB)
#define REG37_VAC1_ADC_L                0x37  // VAC1 voltage (LSB)
#define REG38_VAC1_ADC_H                0x38  // VAC1 voltage (MSB)
#define REG39_VAC2_ADC_L                0x39  // VAC2 voltage (LSB)
#define REG3A_VAC2_ADC_H                0x3A  // VAC2 voltage (MSB)
#define REG3B_VBAT_ADC_L                0x3B  // Battery voltage (LSB)
#define REG3C_VBAT_ADC_H                0x3C  // Battery voltage (MSB)
#define REG3D_VSYS_ADC_L                0x3D  // System voltage (LSB)
#define REG3E_VSYS_ADC_H                0x3E  // System voltage (MSB)
#define REG3F_TS_ADC_L                  0x3F  // TS pin voltage (LSB)
#define REG40_TS_ADC_H                  0x40  // TS pin voltage (MSB)
#define REG41_TDIE_ADC_L                0x41  // Die temperature (LSB)
#define REG42_TDIE_ADC_H                0x42  // Die temperature (MSB)
#define REG43_DP_ADC_L                  0x43  // D+ USB pin voltage (LSB)
#define REG44_DP_ADC_H                  0x44  // D+ USB pin voltage (MSB)
#define REG45_DM_ADC_L                  0x45  // D− USB pin voltage (LSB)
#define REG46_DM_ADC_H                  0x46  // D− USB pin voltage (MSB)
#define REG47_DPDM_Driver_Control       0x47  // Controls active drive on D+/D− lines (for BC1.2 detection)
#define REG48_Part_Information          0x48  // Part number and silicon revision ID (read-only)

/* #######################################################################################################*\
 *  																									   *
 * 									          BIT SHIFTS && MASKS										   *
 *  																									   *
\*########################################################################################################*/

// REG00: Minimum System Voltage (VSYSMIN)
#define BQ25672_VSYSMIN_SHIFT        0U    // Start bit of minimum system voltage (VSYSMIN)
#define BQ25672_VSYSMIN_MASK         (0x3FU << BQ25672_VSYSMIN_SHIFT)    // 6-bit mask for VSYSMIN (bits 5:0)

// REG01: Charge Voltage Limit (VREG)
#define BQ25672_VREG_SHIFT           0U    // Start bit of charge voltage limit (VREG)
#define BQ25672_VREG_MASK            (0x7FFU << BQ25672_VREG_SHIFT)      // 11-bit mask for VREG (bits 10:0)

// REG03: Charge Current Limit (ICHG)
#define BQ25672_ICHG_SHIFT           0U    // Start bit of charge current limit (ICHG)
#define BQ25672_ICHG_MASK            (0x1FFU << BQ25672_ICHG_SHIFT)      // 9-bit mask for ICHG (bits 8:0)

// REG05: Input Voltage Limit (VINDPM)
#define BQ25672_VINDPM_SHIFT         0U    // Start bit of input voltage limit (VINDPM)
#define BQ25672_VINDPM_MASK          (0xFFU << BQ25672_VINDPM_SHIFT)     // 8-bit mask for VINDPM (bits 7:0)

// REG06: Input Current Limit (IINDPM)
#define BQ25672_IINDPM_SHIFT         0U    // Start bit of input current limit (IINDPM)
#define BQ25672_IINDPM_MASK          (0x1FFU << BQ25672_IINDPM_SHIFT)    // 9-bit mask for IINDPM (bits 8:0)

// REG08: Precharge Control (VBAT_LOWV / IPRECHG)
#define BQ25672_VBAT_LOWV_SHIFT      6U    // Start bit of precharge voltage threshold (VBAT_LOWV)
#define BQ25672_VBAT_LOWV_MASK       (0x03U << BQ25672_VBAT_LOWV_SHIFT)  // 2-bit mask for VBAT_LOWV (bits 7:6)
#define BQ25672_IPRECHG_SHIFT        0U    // Start bit of precharge current (IPRECHG)
#define BQ25672_IPRECHG_MASK         (0x3FU << BQ25672_IPRECHG_SHIFT)    // 6-bit mask for IPRECHG (bits 5:0)

// REG09: Termination Control (ITERM / STOP_WD / RST)
#define BQ25672_ITERM_SHIFT           0U   // Bits 4:0 - Termination current regulation setting (mA)
#define BQ25672_ITERM_MASK            (0x1FU << BQ25672_ITERM_SHIFT)   // 5-bit mask for ITERM (bits 4:0)
#define BQ25672_STOP_WD_CHG_SHIFT     5U   // Bit 5 - Stop charging on watchdog expiry
#define BQ25672_STOP_WD_CHG_MASK      (1U << BQ25672_STOP_WD_CHG_SHIFT) // 1-bit mask for STOP_WD_CHG (bit 5)
#define BQ25672_REG_RST_SHIFT         6U   // Bit 6 - Reset all registers to default
#define BQ25672_REG_RST_MASK          (1U << BQ25672_REG_RST_SHIFT)    // 1-bit mask for REG_RST (bit 6)

// REG0A: Re-charge Control (CELL / TRECHG / VRECHG)
#define BQ25672_CELL_SHIFT              6U  // Bits 7:6 - Number of battery cells selection (1 or 2 cells)
#define BQ25672_CELL_MASK               (0x03U << BQ25672_CELL_SHIFT)
#define BQ25672_TRECHG_SHIFT            4U  // Bits 5:4 - Recharge deglitch timing (prevent false recharge)
#define BQ25672_TRECHG_MASK             (0x03U << BQ25672_TRECHG_SHIFT)
#define BQ25672_VRECHG_SHIFT            0U  // Bits 3:0 - Recharge voltage threshold offset (mV)
#define BQ25672_VRECHG_MASK             (0x0FU << BQ25672_VRECHG_SHIFT)

// REG0B: OTG VBUS Output Voltage Regulation
#define BQ25672_VOTG_SHIFT        0U    // Bits 6:0 - Sets OTG output VBUS voltage (mV)
#define BQ25672_VOTG_MASK         (0x7FU << BQ25672_VOTG_SHIFT)  // 7-bit mask for VOTG field (0b01111111)

// REG0D: OTG VBUS Output Current Regulation
#define BQ25672_IOTG_SHIFT        0U    // Bits 6:0 - Sets OTG output VBUS current limit (mA)
#define BQ25672_IOTG_MASK         (0x7FU << BQ25672_IOTG_SHIFT)  // 7-bit mask for IOTG field (0b01111111)
#define BQ25672_PRECHRG_TMR_SHIFT 7U    // Bit 7 - Precharge timer enable
#define BQ25672_PRECHRG_TMR_MASK  (1U << BQ25672_PRECHRG_TMR_SHIFT)

// REG0E: Charge Timer Control
#define BQ25672_TOP_OFF_TIMER_SHIFT         6U  // Bits 7:6 - Top-off timer duration selection
#define BQ25672_TOP_OFF_TIMER_MASK          (0x3U << BQ25672_TOP_OFF_TIMER_SHIFT)  // 2-bit mask for top-off timer
#define BQ25672_TRICKLE_TIMER_ENABLE_SHIFT  5U  // Bit 5 - Enable trickle charge timer
#define BQ25672_TRICKLE_TIMER_ENABLE_MASK   (1U << BQ25672_TRICKLE_TIMER_ENABLE_SHIFT)
#define BQ25672_PRECHARGE_TIMER_ENABLE_SHIFT 4U  // Bit 4 - Enable precharge timer
#define BQ25672_PRECHARGE_TIMER_ENABLE_MASK (1U << BQ25672_PRECHARGE_TIMER_ENABLE_SHIFT)
#define BQ25672_CHARGE_TIMER_ENABLE_SHIFT   3U  // Bit 3 - Enable charge safety timer
#define BQ25672_CHARGE_TIMER_ENABLE_MASK    (1U << BQ25672_CHARGE_TIMER_ENABLE_SHIFT)
#define BQ25672_FAST_CHARGE_TIMER_SHIFT     1U  // Bits 2:1 - Fast charge timer duration selection
#define BQ25672_FAST_CHARGE_TIMER_MASK      (0x3U << BQ25672_FAST_CHARGE_TIMER_SHIFT) // 2-bit mask
#define BQ25672_SLOW_TIMERS_SHIFT           0U  // Bit 0 - Enable slow timers
#define BQ25672_SLOW_TIMERS_MASK            (1U << BQ25672_SLOW_TIMERS_SHIFT)

// REG0F: Charger Control 0
#define BQ25672_EN_DISCHARGE_SHIFT    7U  // Bit 7 - Enable battery discharge path to system
#define BQ25672_EN_DISCHARGE_MASK     (1U << BQ25672_EN_DISCHARGE_SHIFT)
#define BQ25672_FORCE_DISCHARGE_SHIFT 6U  // Bit 6 - Force battery discharge
#define BQ25672_FORCE_DISCHARGE_MASK  (1U << BQ25672_FORCE_DISCHARGE_SHIFT)
#define BQ25672_FORCE_ICO_SHIFT       5U  // Bit 5 - Trigger Input Current Optimizer (ICO)
#define BQ25672_FORCE_ICO_MASK        (1U << BQ25672_FORCE_ICO_SHIFT)
#define BQ25672_EN_CHG_SHIFT          4U  // Bit 4 - Enable battery charging
#define BQ25672_EN_CHG_MASK           (1U << BQ25672_EN_CHG_SHIFT)
#define BQ25672_EN_ICO_SHIFT          3U  // Bit 3 - Enable ICO (Input Current Optimization)
#define BQ25672_EN_ICO_MASK           (1U << BQ25672_EN_ICO_SHIFT)
#define BQ25672_EN_TERM_SHIFT         2U  // Bit 2 - Enable charge termination
#define BQ25672_EN_TERM_MASK          (1U << BQ25672_EN_TERM_SHIFT)
#define BQ25672_EN_HIZ_SHIFT          1U  // Bit 1 - Enable High Impedance (HiZ) input mode
#define BQ25672_EN_HIZ_MASK           (1U << BQ25672_EN_HIZ_SHIFT)

// REG10: Charger Control 1
#define BQ25672_INPUT_VOLTAGE_DPM_SHIFT 4U // Bits 7:4 - Input voltage DPM threshold setting (mV)
#define BQ25672_INPUT_VOLTAGE_DPM_MASK  (0xFU << BQ25672_INPUT_VOLTAGE_DPM_SHIFT) // 4-bit mask
#define BQ25672_HVDCP_SHIFT            0U // Bits 2:0 - HVDCP adapter detection selection
#define BQ25672_HVDCP_MASK             (0x7U << BQ25672_HVDCP_SHIFT) // 3-bit mask
#define BQ25672_WDOG_RESET_SHIFT       3U // Bit 3 - Watchdog reset trigger
#define BQ25672_WDOG_RESET_MASK        (1U << BQ25672_WDOG_RESET_SHIFT)
#define BQ25672_WATCHDOG_TIMER_SHIFT   0U // Bits 2:0 - Watchdog timer timeout selection
#define BQ25672_WATCHDOG_TIMER_MASK    (0x7U << BQ25672_WATCHDOG_TIMER_SHIFT) // 3-bit mask
#define BQ25672_VAC_OVP_THRESH_SHIFT   4U // Bits 5:4 - VAC over-voltage protection threshold
#define BQ25672_VAC_OVP_THRESH_MASK    (0x3U << BQ25672_VAC_OVP_THRESH_SHIFT) // 2-bit mask

// REG11: Charger Control 2
#define BQ25672_FORCE_INDET_SHIFT       7U  // Bit 7: Force input detection (forces adapter detection sequence)
#define BQ25672_FORCE_INDET_MASK        (1U << BQ25672_FORCE_INDET_SHIFT)
#define BQ25672_AUTO_INDET_SHIFT        6U  // Bit 6: Enable automatic input detection (D+/D- BC1.2, SDP, CDP detection)
#define BQ25672_AUTO_INDET_MASK         (1U << BQ25672_AUTO_INDET_SHIFT)
#define BQ25672_EN_12V_SHIFT            5U  // Bit 5: Enable 12V adapter voltage output (HVDCP negotiation)
#define BQ25672_EN_12V_MASK             (1U << BQ25672_EN_12V_SHIFT)
#define BQ25672_EN_9V_SHIFT             4U  // Bit 4: Enable 9V adapter voltage output (HVDCP negotiation)
#define BQ25672_EN_9V_MASK              (1U << BQ25672_EN_9V_SHIFT)
#define BQ25672_HVDCP_EN_SHIFT          3U  // Bit 3: Enable HVDCP high-voltage charging protocol
#define BQ25672_HVDCP_EN_MASK           (1U << BQ25672_HVDCP_EN_SHIFT)
#define BQ25672_SDRV_CTRL_SHIFT         1U  // Bits 2:1: System driver control (power path management)
#define BQ25672_SDRV_CTRL_MASK          (0x3U << BQ25672_SDRV_CTRL_SHIFT)
#define BQ25672_SDRV_DLY_SHIFT          0U  // Bit 0: System driver delay enable
#define BQ25672_SDRV_DLY_MASK           (1U << BQ25672_SDRV_DLY_SHIFT)

// REG12: Charger Control 3
#define BQ25672_DIS_ACDRV_SHIFT         7U  // Bit 7: Disable ACDRV drivers (disconnects input FETs)
#define BQ25672_DIS_ACDRV_MASK          (1U << BQ25672_DIS_ACDRV_SHIFT)
#define BQ25672_EN_OTG_SHIFT            6U  // Bit 6: Enable OTG boost mode (VBUS output)
#define BQ25672_EN_OTG_MASK             (1U << BQ25672_EN_OTG_SHIFT)
#define BQ25672_PFM_OTG_DIS_SHIFT       5U  // Bit 5: Disable PFM mode during OTG
#define BQ25672_PFM_OTG_DIS_MASK        (1U << BQ25672_PFM_OTG_DIS_SHIFT)
#define BQ25672_PFM_FWD_DIS_SHIFT       4U  // Bit 4: Disable PFM mode during forward charging
#define BQ25672_PFM_FWD_DIS_MASK        (1U << BQ25672_PFM_FWD_DIS_SHIFT)
#define BQ25672_WKUP_DLY_SHIFT          3U  // Bit 3: Enable wake-up delay on VAC1/VAC2 insertion
#define BQ25672_WKUP_DLY_MASK           (1U << BQ25672_WKUP_DLY_SHIFT)
#define BQ25672_DIS_LDO_SHIFT           2U  // Bit 2: Disable internal LDO for power saving
#define BQ25672_DIS_LDO_MASK            (1U << BQ25672_DIS_LDO_SHIFT)
#define BQ25672_DIS_OTG_OOA_SHIFT       1U  // Bit 1: Disable OOA (Out of Audible) mode during OTG
#define BQ25672_DIS_OTG_OOA_MASK        (1U << BQ25672_DIS_OTG_OOA_SHIFT)
#define BQ25672_DIS_FWD_OOA_SHIFT       0U  // Bit 0: Disable OOA mode during forward path operation
#define BQ25672_DIS_FWD_OOA_MASK        (1U << BQ25672_DIS_FWD_OOA_SHIFT)

// REG13: Charger Control 4
#define BQ25672_EN_ACDRV2_SHIFT         7U  // Bit 7: Enable ACDRV2 output driver (input FET)
#define BQ25672_EN_ACDRV2_MASK          (1U << BQ25672_EN_ACDRV2_SHIFT)
#define BQ25672_EN_ACDRV1_SHIFT         6U  // Bit 6: Enable ACDRV1 output driver (input FET)
#define BQ25672_EN_ACDRV1_MASK          (1U << BQ25672_EN_ACDRV1_SHIFT)
#define BQ25672_PWM_FREQ_SHIFT          5U  // Bit 5: Set switching frequency (0: 750kHz, 1: 1.5MHz)
#define BQ25672_PWM_FREQ_MASK           (1U << BQ25672_PWM_FREQ_SHIFT)
#define BQ25672_DIS_STAT_SHIFT          4U  // Bit 4: Disable STAT pin output function
#define BQ25672_DIS_STAT_MASK           (1U << BQ25672_DIS_STAT_SHIFT)
#define BQ25672_DIS_VSYS_SHORT_SHIFT    3U  // Bit 3: Disable VSYS short detection
#define BQ25672_DIS_VSYS_SHORT_MASK     (1U << BQ25672_DIS_VSYS_SHORT_SHIFT)
#define BQ25672_DIS_VOTG_UVP_SHIFT      2U  // Bit 2: Disable VOTG UVP (undervoltage protection)
#define BQ25672_DIS_VOTG_UVP_MASK       (1U << BQ25672_DIS_VOTG_UVP_SHIFT)
#define BQ25672_FORCE_VINDPM_DET_SHIFT  1U  // Bit 1: Force VINDPM detection (minimum input voltage)
#define BQ25672_FORCE_VINDPM_DET_MASK   (1U << BQ25672_FORCE_VINDPM_DET_SHIFT)
#define BQ25672_EN_IBUS_OCP_SHIFT       0U  // Bit 0: Enable IBUS overcurrent protection
#define BQ25672_EN_IBUS_OCP_MASK        (1U << BQ25672_EN_IBUS_OCP_SHIFT)

// REG14: Charger Control 5
#define BQ25672_SFET_PRESENT_SHIFT      7U  // Bit 7: System FET presence indicator
#define BQ25672_SFET_PRESENT_MASK       (1U << BQ25672_SFET_PRESENT_SHIFT)
#define BQ25672_EN_IBAT_SHIFT           5U  // Bit 5: Enable IBAT regulation (current regulation)
#define BQ25672_EN_IBAT_MASK            (1U << BQ25672_EN_IBAT_SHIFT)
#define BQ25672_IBAT_REG_SHIFT          3U  // Bits 4:3: IBAT regulation level
#define BQ25672_IBAT_REG_MASK           (0x3U << BQ25672_IBAT_REG_SHIFT)
#define BQ25672_EN_IINDPM_SHIFT         2U  // Bit 2: Enable IINDPM current limit
#define BQ25672_EN_IINDPM_MASK          (1U << BQ25672_EN_IINDPM_SHIFT)
#define BQ25672_EN_EXTILIM_SHIFT        1U  // Bit 1: Enable external input current limit
#define BQ25672_EN_EXTILIM_MASK         (1U << BQ25672_EN_EXTILIM_SHIFT)
#define BQ25672_EN_BATOC_SHIFT          0U  // Bit 0: Enable battery overcurrent protection
#define BQ25672_EN_BATOC_MASK           (1U << BQ25672_EN_BATOC_SHIFT)

// REG15: MPPT Control
#define BQ25672_VOC_PCT_SHIFT      5U  // MPPT VOC percentage shift
#define BQ25672_VOC_PCT_MASK       (0x07U << BQ25672_VOC_PCT_SHIFT)  // Mask for VOC_PCT bits [7:5]
#define BQ25672_VOC_DLY_SHIFT      3U  // MPPT delay shift
#define BQ25672_VOC_DLY_MASK       (0x03U << BQ25672_VOC_DLY_SHIFT)  // Mask for delay bits [4:3]
#define BQ25672_VOC_RATE_SHIFT     1U  // MPPT sample rate shift
#define BQ25672_VOC_RATE_MASK      (0x03U << BQ25672_VOC_RATE_SHIFT) // Mask for rate bits [2:1]
#define BQ25672_EN_MPPT_SHIFT      0U  // MPPT enable shift
#define BQ25672_EN_MPPT_MASK       (1U << BQ25672_EN_MPPT_SHIFT)     // Mask for enable bit [0]

// REG16: Temperature Control
#define BQ25672_TREG_SHIFT         6U  // Thermal regulation threshold shift
#define BQ25672_TREG_MASK          (0x03U << BQ25672_TREG_SHIFT)  // [7:6]
#define BQ25672_TSHUT_SHIFT        4U  // Thermal shutdown threshold shift
#define BQ25672_TSHUT_MASK         (0x03U << BQ25672_TSHUT_SHIFT) // [5:4]
#define BQ25672_VBUS_PD_EN_SHIFT   3U  // VBUS pull-down enable
#define BQ25672_VBUS_PD_EN_MASK    (1U << BQ25672_VBUS_PD_EN_SHIFT)
#define BQ25672_VAC1_PD_EN_SHIFT   2U  // VAC1 pull-down enable
#define BQ25672_VAC1_PD_EN_MASK    (1U << BQ25672_VAC1_PD_EN_SHIFT)
#define BQ25672_VAC2_PD_EN_SHIFT   1U  // VAC2 pull-down enable
#define BQ25672_VAC2_PD_EN_MASK    (1U << BQ25672_VAC2_PD_EN_SHIFT)

// REG17: NTC Control 0
#define BQ25672_JEITA_VSET_SHIFT   5U  // JEITA voltage setting shift
#define BQ25672_JEITA_VSET_MASK    (0x07U << BQ25672_JEITA_VSET_SHIFT) // [7:5]
#define BQ25672_JEITA_ISETH_SHIFT  3U  // JEITA high-temp current setting shift
#define BQ25672_JEITA_ISETH_MASK   (0x03U << BQ25672_JEITA_ISETH_SHIFT) // [4:3]
#define BQ25672_JEITA_ISETC_SHIFT  1U  // JEITA low-temp current setting shift
#define BQ25672_JEITA_ISETC_MASK   (0x03U << BQ25672_JEITA_ISETC_SHIFT) // [2:1]

// REG18: NTC Control 1
#define BQ25672_TS_COOL_SHIFT      6U  // TS cool threshold shift
#define BQ25672_TS_COOL_MASK       (0x03U << BQ25672_TS_COOL_SHIFT) // [7:6]
#define BQ25672_TS_WARM_SHIFT      4U  // TS warm threshold shift
#define BQ25672_TS_WARM_MASK       (0x03U << BQ25672_TS_WARM_SHIFT) // [5:4]
#define BQ25672_BHOT_SHIFT         2U  // TS hot threshold shift
#define BQ25672_BHOT_MASK          (0x03U << BQ25672_BHOT_SHIFT) // [3:2]
#define BQ25672_BCOLD_SHIFT        1U  // TS cold fault shift
#define BQ25672_BCOLD_MASK         (1U << BQ25672_BCOLD_SHIFT)
#define BQ25672_TS_IGNORE_SHIFT    0U  // TS ignore fault shift
#define BQ25672_TS_IGNORE_MASK     (1U << BQ25672_TS_IGNORE_SHIFT)

// REG19: ICO Current Limit
#define BQ25672_ICO_ILIM_SHIFT     0U  // ICO current limit shift
#define BQ25672_ICO_ILIM_MASK      (0x1FU << BQ25672_ICO_ILIM_SHIFT) // [4:0]

// REG1B: Charger Status 0
#define BQ25672_IINDPM_STAT_SHIFT      7U
#define BQ25672_IINDPM_STAT_MASK       (1U << BQ25672_IINDPM_STAT_SHIFT)
#define BQ25672_VINDPM_STAT_SHIFT      6U
#define BQ25672_VINDPM_STAT_MASK       (1U << BQ25672_VINDPM_STAT_SHIFT)
#define BQ25672_WD_STAT_SHIFT          5U
#define BQ25672_WD_STAT_MASK           (1U << BQ25672_WD_STAT_SHIFT)
#define BQ25672_AC2_PRESENT_STAT_SHIFT 4U
#define BQ25672_AC2_PRESENT_STAT_MASK  (1U << BQ25672_AC2_PRESENT_STAT_SHIFT)
#define BQ25672_PG_STAT_SHIFT          3U
#define BQ25672_PG_STAT_MASK           (1U << BQ25672_PG_STAT_SHIFT)
#define BQ25672_AC1_PRESENT_STAT_SHIFT 1U
#define BQ25672_AC1_PRESENT_STAT_MASK  (1U << BQ25672_AC1_PRESENT_STAT_SHIFT)
#define BQ25672_VBUS_PRESENT_STAT_SHIFT 0U
#define BQ25672_VBUS_PRESENT_STAT_MASK  (1U << BQ25672_VBUS_PRESENT_STAT_SHIFT)

// REG1C: Charger Status 1
#define BQ25672_CHG_STAT_SHIFT        5U  // Charge status bits start at bit 5 (charging phases)
#define BQ25672_CHG_STAT_MASK         (0x07U << BQ25672_CHG_STAT_SHIFT)  // 3-bit mask for charge status
#define BQ25672_VBUS_STAT_SHIFT       1U  // VBUS adapter detection bits start at bit 1
#define BQ25672_VBUS_STAT_MASK        (0x0FU << BQ25672_VBUS_STAT_SHIFT)  // 4-bit mask for VBUS source type
#define BQ25672_BC12_DONE_SHIFT       0U  // BC1.2 USB detection done flag
#define BQ25672_BC12_DONE_MASK        (1U << BQ25672_BC12_DONE_SHIFT)

// REG1D: Charger Status 2
#define BQ25672_ICO_STAT_SHIFT        6U  // Input Current Optimizer status (2 bits)
#define BQ25672_ICO_STAT_MASK         (0x03U << BQ25672_ICO_STAT_SHIFT)
#define BQ25672_TREG_STAT_SHIFT       2U  // Thermal regulation active
#define BQ25672_TREG_STAT_MASK        (1U << BQ25672_TREG_STAT_SHIFT)
#define BQ25672_DPDM_STAT_SHIFT       1U  // DPDM detection completed
#define BQ25672_DPDM_STAT_MASK        (1U << BQ25672_DPDM_STAT_SHIFT)
#define BQ25672_VBAT_PRESENT_STAT_SHIFT 0U  // Battery present detection
#define BQ25672_VBAT_PRESENT_STAT_MASK  (1U << BQ25672_VBAT_PRESENT_STAT_SHIFT)

// REG1E: Charger Status 3
#define BQ25672_ACRB2_STAT_SHIFT      7U  // ACDRV2 FET active
#define BQ25672_ACRB2_STAT_MASK       (1U << BQ25672_ACRB2_STAT_SHIFT)
#define BQ25672_ACRB1_STAT_SHIFT      6U  // ACDRV1 FET active
#define BQ25672_ACRB1_STAT_MASK       (1U << BQ25672_ACRB1_STAT_SHIFT)
#define BQ25672_ADC_DONE_STAT_SHIFT   5U  // ADC conversion complete
#define BQ25672_ADC_DONE_STAT_MASK    (1U << BQ25672_ADC_DONE_STAT_SHIFT)
#define BQ25672_VSYS_STAT_SHIFT       4U  // VSYS above VSYSMIN
#define BQ25672_VSYS_STAT_MASK        (1U << BQ25672_VSYS_STAT_SHIFT)
#define BQ25672_CHG_TMR_STAT_SHIFT    3U  // Charge timer expired
#define BQ25672_CHG_TMR_STAT_MASK     (1U << BQ25672_CHG_TMR_STAT_SHIFT)
#define BQ25672_TRICKLE_TMR_STAT_SHIFT 2U  // Trickle charge timer expired
#define BQ25672_TRICKLE_TMR_STAT_MASK  (1U << BQ25672_TRICKLE_TMR_STAT_SHIFT)
#define BQ25672_PRECHG_TMR_STAT_SHIFT 1U  // Precharge timer expired
#define BQ25672_PRECHG_TMR_STAT_MASK  (1U << BQ25672_PRECHG_TMR_STAT_SHIFT)

// REG1F: Charger Status 4
#define BQ25672_VBATOTG_LOW_STAT_SHIFT 4U  // VBAT too low for OTG operation
#define BQ25672_VBATOTG_LOW_STAT_MASK  (1U << BQ25672_VBATOTG_LOW_STAT_SHIFT)
#define BQ25672_TS_COLD_STAT_SHIFT     3U  // TS pin indicates battery cold
#define BQ25672_TS_COLD_STAT_MASK      (1U << BQ25672_TS_COLD_STAT_SHIFT)
#define BQ25672_TS_COOL_STAT_SHIFT     2U  // TS pin indicates battery cool
#define BQ25672_TS_COOL_STAT_MASK      (1U << BQ25672_TS_COOL_STAT_SHIFT)
#define BQ25672_TS_WARM_STAT_SHIFT     1U  // TS pin indicates battery warm
#define BQ25672_TS_WARM_STAT_MASK      (1U << BQ25672_TS_WARM_STAT_SHIFT)
#define BQ25672_TS_HOT_STAT_SHIFT      0U  // TS pin indicates battery hot
#define BQ25672_TS_HOT_STAT_MASK       (1U << BQ25672_TS_HOT_STAT_SHIFT)

// REG20: Fault Status 0
#define BQ25672_IBAT_REG_STAT_SHIFT    7U  // Battery regulation fault
#define BQ25672_IBAT_REG_STAT_MASK     (1U << BQ25672_IBAT_REG_STAT_SHIFT)
#define BQ25672_VBUS_OVP_STAT_SHIFT    6U  // VBUS overvoltage fault
#define BQ25672_VBUS_OVP_STAT_MASK     (1U << BQ25672_VBUS_OVP_STAT_SHIFT)
#define BQ25672_VBAT_OVP_STAT_SHIFT    5U  // VBAT overvoltage fault
#define BQ25672_VBAT_OVP_STAT_MASK     (1U << BQ25672_VBAT_OVP_STAT_SHIFT)
#define BQ25672_IBUS_OCP_STAT_SHIFT    4U  // IBUS overcurrent fault
#define BQ25672_IBUS_OCP_STAT_MASK     (1U << BQ25672_IBUS_OCP_STAT_SHIFT)
#define BQ25672_IBAT_OCP_STAT_SHIFT    3U  // IBAT overcurrent fault
#define BQ25672_IBAT_OCP_STAT_MASK     (1U << BQ25672_IBAT_OCP_STAT_SHIFT)
#define BQ25672_CONV_OVP_STAT_SHIFT    2U  // Converter overvoltage fault
#define BQ25672_CONV_OVP_STAT_MASK     (1U << BQ25672_CONV_OVP_STAT_SHIFT)
#define BQ25672_VAC2_OVP_STAT_SHIFT    1U  // VAC2 overvoltage fault
#define BQ25672_VAC2_OVP_STAT_MASK     (1U << BQ25672_VAC2_OVP_STAT_SHIFT)
#define BQ25672_VAC1_OVP_STAT_SHIFT    0U  // VAC1 overvoltage fault
#define BQ25672_VAC1_OVP_STAT_MASK     (1U << BQ25672_VAC1_OVP_STAT_SHIFT)

// REG21: Fault Status 1
#define BQ25672_VSYS_SHORT_STAT_SHIFT  7U  // VSYS short circuit fault
#define BQ25672_VSYS_SHORT_STAT_MASK   (1U << BQ25672_VSYS_SHORT_STAT_SHIFT)
#define BQ25672_VSYS_OVP_STAT_SHIFT    6U  // VSYS overvoltage fault
#define BQ25672_VSYS_OVP_STAT_MASK     (1U << BQ25672_VSYS_OVP_STAT_SHIFT)
#define BQ25672_OTG_OVP_STAT_SHIFT     5U  // OTG overvoltage fault
#define BQ25672_OTG_OVP_STAT_MASK      (1U << BQ25672_OTG_OVP_STAT_SHIFT)
#define BQ25672_OTG_UVP_STAT_SHIFT     4U  // OTG undervoltage fault
#define BQ25672_OTG_UVP_STAT_MASK      (1U << BQ25672_OTG_UVP_STAT_SHIFT)
#define BQ25672_TSHUT_STAT_SHIFT       2U  // Thermal shutdown triggered
#define BQ25672_TSHUT_STAT_MASK        (1U << BQ25672_TSHUT_STAT_SHIFT)

// REG22: Charger Flag 0 - Input / Power flags
#define BQ25672_IINDPM_FLAG_SHIFT          7U  // Input current DPM triggered
#define BQ25672_IINDPM_FLAG_MASK           (1U << BQ25672_IINDPM_FLAG_SHIFT)
#define BQ25672_VINDPM_FLAG_SHIFT          6U  // Input voltage DPM triggered
#define BQ25672_VINDPM_FLAG_MASK           (1U << BQ25672_VINDPM_FLAG_SHIFT)
#define BQ25672_WATCHDOG_FLAG_SHIFT        5U  // Watchdog timer expired
#define BQ25672_WATCHDOG_FLAG_MASK         (1U << BQ25672_WATCHDOG_FLAG_SHIFT)
#define BQ25672_POORSRC_FLAG_SHIFT         4U  // Poor source detected
#define BQ25672_POORSRC_FLAG_MASK          (1U << BQ25672_POORSRC_FLAG_SHIFT)
#define BQ25672_PG_FLAG_SHIFT              3U  // Power-good status
#define BQ25672_PG_FLAG_MASK               (1U << BQ25672_PG_FLAG_SHIFT)
#define BQ25672_VAC2_PRESENT_FLAG_SHIFT    2U  // VAC2 present detection
#define BQ25672_VAC2_PRESENT_FLAG_MASK     (1U << BQ25672_VAC2_PRESENT_FLAG_SHIFT)
#define BQ25672_VAC1_PRESENT_FLAG_SHIFT    1U  // VAC1 present detection
#define BQ25672_VAC1_PRESENT_FLAG_MASK     (1U << BQ25672_VAC1_PRESENT_FLAG_SHIFT)
#define BQ25672_VBUS_PRESENT_FLAG_SHIFT    0U  // VBUS present detection
#define BQ25672_VBUS_PRESENT_FLAG_MASK     (1U << BQ25672_VBUS_PRESENT_FLAG_SHIFT)

// REG23: Charger Flag 1 - Charge / Detection flags
#define BQ25672_CHG_FLAG_SHIFT             7U  // Charging status active
#define BQ25672_CHG_FLAG_MASK              (1U << BQ25672_CHG_FLAG_SHIFT)
#define BQ25672_ICO_FLAG_SHIFT             6U  // ICO (input current optimization) complete
#define BQ25672_ICO_FLAG_MASK              (1U << BQ25672_ICO_FLAG_SHIFT)
#define BQ25672_VBUS_FLAG_SHIFT            4U  // VBUS input detected
#define BQ25672_VBUS_FLAG_MASK             (1U << BQ25672_VBUS_FLAG_SHIFT)
#define BQ25672_TREG_FLAG_SHIFT            2U  // Thermal regulation active
#define BQ25672_TREG_FLAG_MASK             (1U << BQ25672_TREG_FLAG_SHIFT)
#define BQ25672_VBAT_PRESENT_FLAG_SHIFT    1U  // Battery present detection
#define BQ25672_VBAT_PRESENT_FLAG_MASK     (1U << BQ25672_VBAT_PRESENT_FLAG_SHIFT)
#define BQ25672_BC12_DONE_FLAG_SHIFT       0U  // BC1.2 detection complete
#define BQ25672_BC12_DONE_FLAG_MASK        (1U << BQ25672_BC12_DONE_FLAG_SHIFT)

// REG24: Charger Flag 2 - Timers / ADC flags
#define BQ25672_DPDM_DONE_FLAG_SHIFT       6U  // DPDM detection done
#define BQ25672_DPDM_DONE_FLAG_MASK        (1U << BQ25672_DPDM_DONE_FLAG_SHIFT)
#define BQ25672_ADC_DONE_FLAG_SHIFT        5U  // ADC operation completed
#define BQ25672_ADC_DONE_FLAG_MASK         (1U << BQ25672_ADC_DONE_FLAG_SHIFT)
#define BQ25672_VSYS_FLAG_SHIFT            4U  // VSYS output present
#define BQ25672_VSYS_FLAG_MASK             (1U << BQ25672_VSYS_FLAG_SHIFT)
#define BQ25672_CHG_TMR_FLAG_SHIFT         3U  // Charge timer expired
#define BQ25672_CHG_TMR_FLAG_MASK          (1U << BQ25672_CHG_TMR_FLAG_SHIFT)
#define BQ25672_TRICHG_TMR_FLAG_SHIFT      2U  // Trickle charge timer expired
#define BQ25672_TRICHG_TMR_FLAG_MASK       (1U << BQ25672_TRICHG_TMR_FLAG_SHIFT)
#define BQ25672_PRECHG_TMR_FLAG_SHIFT      1U  // Precharge timer expired
#define BQ25672_PRECHG_TMR_FLAG_MASK       (1U << BQ25672_PRECHG_TMR_FLAG_SHIFT)
#define BQ25672_TOPOFF_TMR_FLAG_SHIFT      0U  // Top-off timer expired
#define BQ25672_TOPOFF_TMR_FLAG_MASK       (1U << BQ25672_TOPOFF_TMR_FLAG_SHIFT)

// REG25: Fault Flag 3 - Temperature / Faults
#define BQ25672_VBATOTG_LOW_FLAG_SHIFT     4U  // VBAT too low for OTG mode
#define BQ25672_VBATOTG_LOW_FLAG_MASK      (1U << BQ25672_VBATOTG_LOW_FLAG_SHIFT)
#define BQ25672_TS_COLD_FLAG_SHIFT         3U  // TS pin indicates battery cold
#define BQ25672_TS_COLD_FLAG_MASK          (1U << BQ25672_TS_COLD_FLAG_SHIFT)
#define BQ25672_TS_COOL_FLAG_SHIFT         2U  // TS pin indicates battery cool
#define BQ25672_TS_COOL_FLAG_MASK          (1U << BQ25672_TS_COOL_FLAG_SHIFT)
#define BQ25672_TS_WARM_FLAG_SHIFT         1U  // TS pin indicates battery warm
#define BQ25672_TS_WARM_FLAG_MASK          (1U << BQ25672_TS_WARM_FLAG_SHIFT)
#define BQ25672_TS_HOT_FLAG_SHIFT          0U  // TS pin indicates battery hot
#define BQ25672_TS_HOT_FLAG_MASK           (1U << BQ25672_TS_HOT_FLAG_SHIFT)
#define BQ25672_INPUT_SHORT_FLAG_SHIFT     0U  // Input short detected (duplicate mask)
#define BQ25672_INPUT_SHORT_FLAG_MASK      (1U << BQ25672_INPUT_SHORT_FLAG_SHIFT)

// REG26: Fault Flag 0
#define BQ25672_IBAT_REG_FLAG_SHIFT     7U  // Bit 7: Battery regulation fault
#define BQ25672_IBAT_REG_FLAG_MASK      (1U << BQ25672_IBAT_REG_FLAG_SHIFT)
#define BQ25672_VBUS_OVP_FLAG_SHIFT     6U  // Bit 6: VBUS overvoltage fault
#define BQ25672_VBUS_OVP_FLAG_MASK      (1U << BQ25672_VBUS_OVP_FLAG_SHIFT)
#define BQ25672_VBAT_OVP_FLAG_SHIFT     5U  // Bit 5: VBAT overvoltage fault
#define BQ25672_VBAT_OVP_FLAG_MASK      (1U << BQ25672_VBAT_OVP_FLAG_SHIFT)
#define BQ25672_IBUS_OCP_FLAG_SHIFT     4U  // Bit 4: IBUS overcurrent fault
#define BQ25672_IBUS_OCP_FLAG_MASK      (1U << BQ25672_IBUS_OCP_FLAG_SHIFT)
#define BQ25672_IBAT_OCP_FLAG_SHIFT     3U  // Bit 3: IBAT overcurrent fault
#define BQ25672_IBAT_OCP_FLAG_MASK      (1U << BQ25672_IBAT_OCP_FLAG_SHIFT)
#define BQ25672_CONV_OCP_FLAG_SHIFT     2U  // Bit 2: Converter overcurrent fault
#define BQ25672_CONV_OCP_FLAG_MASK      (1U << BQ25672_CONV_OCP_FLAG_SHIFT)
#define BQ25672_VAC2_OVP_FLAG_SHIFT     1U  // Bit 1: VAC2 overvoltage fault
#define BQ25672_VAC2_OVP_FLAG_MASK      (1U << BQ25672_VAC2_OVP_FLAG_SHIFT)
#define BQ25672_VAC1_OVP_FLAG_SHIFT     0U  // Bit 0: VAC1 overvoltage fault
#define BQ25672_VAC1_OVP_FLAG_MASK      (1U << BQ25672_VAC1_OVP_FLAG_SHIFT)

// REG27: Fault Flag 1
#define BQ25672_VSYS_SHORT_FLAG_SHIFT   7U  // Bit 7: VSYS short fault
#define BQ25672_VSYS_SHORT_FLAG_MASK    (1U << BQ25672_VSYS_SHORT_FLAG_SHIFT)
#define BQ25672_VSYS_OVP_FLAG_SHIFT     6U  // Bit 6: VSYS overvoltage fault
#define BQ25672_VSYS_OVP_FLAG_MASK      (1U << BQ25672_VSYS_OVP_FLAG_SHIFT)
#define BQ25672_OTG_OVP_FLAG_SHIFT      5U  // Bit 5: OTG overvoltage fault
#define BQ25672_OTG_OVP_FLAG_MASK       (1U << BQ25672_OTG_OVP_FLAG_SHIFT)
#define BQ25672_OTG_UVP_FLAG_SHIFT      4U  // Bit 4: OTG undervoltage fault
#define BQ25672_OTG_UVP_FLAG_MASK       (1U << BQ25672_OTG_UVP_FLAG_SHIFT)
#define BQ25672_TSHUT_FLAG_SHIFT        2U  // Bit 2: Thermal shutdown fault
#define BQ25672_TSHUT_FLAG_MASK         (1U << BQ25672_TSHUT_FLAG_SHIFT)

// REG28: Charger Mask 0
#define BQ25672_IINDPM_MASK_SHIFT       7U  // Bit 7: IINDPM event mask
#define BQ25672_IINDPM_MASK_MASK        (1U << BQ25672_IINDPM_MASK_SHIFT)
#define BQ25672_VINDPM_MASK_SHIFT       6U  // Bit 6: VINDPM event mask
#define BQ25672_VINDPM_MASK_MASK        (1U << BQ25672_VINDPM_MASK_SHIFT)
#define BQ25672_WD_MASK_SHIFT           5U  // Bit 5: Watchdog event mask
#define BQ25672_WD_MASK_MASK            (1U << BQ25672_WD_MASK_SHIFT)
#define BQ25672_POORSRC_MASK_SHIFT      4U  // Bit 4: Poor source mask
#define BQ25672_POORSRC_MASK_MASK       (1U << BQ25672_POORSRC_MASK_SHIFT)
#define BQ25672_PG_MASK_SHIFT           3U  // Bit 3: Power-good mask
#define BQ25672_PG_MASK_MASK            (1U << BQ25672_PG_MASK_SHIFT)
#define BQ25672_AC2_PRESENT_MASK_SHIFT  2U  // Bit 2: VAC2 present mask
#define BQ25672_AC2_PRESENT_MASK_MASK   (1U << BQ25672_AC2_PRESENT_MASK_SHIFT)
#define BQ25672_AC1_PRESENT_MASK_SHIFT  1U  // Bit 1: VAC1 present mask
#define BQ25672_AC1_PRESENT_MASK_MASK   (1U << BQ25672_AC1_PRESENT_MASK_SHIFT)
#define BQ25672_VBUS_PRESENT_MASK_SHIFT 0U  // Bit 0: VBUS present mask
#define BQ25672_VBUS_PRESENT_MASK_MASK  (1U << BQ25672_VBUS_PRESENT_MASK_SHIFT)

// REG29: Charger Mask 1
#define BQ25672_CHG_MASK_SHIFT          7U  // Bit 7: Charge status mask
#define BQ25672_CHG_MASK_MASK           (1U << BQ25672_CHG_MASK_SHIFT)
#define BQ25672_ICO_MASK_SHIFT          6U  // Bit 6: ICO status mask
#define BQ25672_ICO_MASK_MASK           (1U << BQ25672_ICO_MASK_SHIFT)
#define BQ25672_VBUS_MASK_SHIFT         4U  // Bit 4: VBUS status mask
#define BQ25672_VBUS_MASK_MASK          (1U << BQ25672_VBUS_MASK_SHIFT)
#define BQ25672_TREG_MASK_SHIFT         2U  // Bit 2: Thermal regulation mask
#define BQ25672_TREG_MASK_MASK          (1U << BQ25672_TREG_MASK_SHIFT)
#define BQ25672_VBAT_PRESENT_MASK_SHIFT 1U  // Bit 1: VBAT present mask
#define BQ25672_VBAT_PRESENT_MASK_MASK  (1U << BQ25672_VBAT_PRESENT_MASK_SHIFT)
#define BQ25672_BC_12_DONE_MASK_SHIFT   0U  // Bit 0: BC1.2 detection done mask
#define BQ25672_BC_12_DONE_MASK_MASK    (1U << BQ25672_BC_12_DONE_MASK_SHIFT)

// REG2C: Fault Mask 0
#define BQ25672_IBAT_REG_MASK_SHIFT    7U  // Bit 7: IBAT regulation fault mask shift
#define BQ25672_IBAT_REG_MASK_MASK     (1U << BQ25672_IBAT_REG_MASK_SHIFT)
#define BQ25672_VBUS_OVP_MASK_SHIFT    6U  // Bit 6: VBUS overvoltage fault mask shift
#define BQ25672_VBUS_OVP_MASK_MASK     (1U << BQ25672_VBUS_OVP_MASK_SHIFT)
#define BQ25672_VBAT_OVP_MASK_SHIFT    5U  // Bit 5: VBAT overvoltage fault mask shift
#define BQ25672_VBAT_OVP_MASK_MASK     (1U << BQ25672_VBAT_OVP_MASK_SHIFT)
#define BQ25672_IBUS_OCP_MASK_SHIFT    4U  // Bit 4: IBUS overcurrent fault mask shift
#define BQ25672_IBUS_OCP_MASK_MASK     (1U << BQ25672_IBUS_OCP_MASK_SHIFT)
#define BQ25672_IBAT_OCP_MASK_SHIFT    3U  // Bit 3: IBAT overcurrent fault mask shift
#define BQ25672_IBAT_OCP_MASK_MASK     (1U << BQ25672_IBAT_OCP_MASK_SHIFT)
#define BQ25672_CONV_OCP_MASK_SHIFT    2U  // Bit 2: Converter overcurrent fault mask shift
#define BQ25672_CONV_OCP_MASK_MASK     (1U << BQ25672_CONV_OCP_MASK_SHIFT)
#define BQ25672_VAC2_OVP_MASK_SHIFT    1U  // Bit 1: VAC2 overvoltage fault mask shift
#define BQ25672_VAC2_OVP_MASK_MASK     (1U << BQ25672_VAC2_OVP_MASK_SHIFT)
#define BQ25672_VAC1_OVP_MASK_SHIFT    0U  // Bit 0: VAC1 overvoltage fault mask shift
#define BQ25672_VAC1_OVP_MASK_MASK     (1U << BQ25672_VAC1_OVP_MASK_SHIFT)

// REG2D: Fault Mask 1
#define BQ25672_VSYS_SHORT_MASK_SHIFT  7U  // Bit 7: VSYS short fault mask shift
#define BQ25672_VSYS_SHORT_MASK_MASK   (1U << BQ25672_VSYS_SHORT_MASK_SHIFT)
#define BQ25672_VSYS_OVP_MASK_SHIFT    6U  // Bit 6: VSYS overvoltage fault mask shift
#define BQ25672_VSYS_OVP_MASK_MASK     (1U << BQ25672_VSYS_OVP_MASK_SHIFT)
#define BQ25672_OTG_OVP_MASK_SHIFT     5U  // Bit 5: OTG overvoltage fault mask shift
#define BQ25672_OTG_OVP_MASK_MASK      (1U << BQ25672_OTG_OVP_MASK_SHIFT)
#define BQ25672_OTG_UVP_MASK_SHIFT     4U  // Bit 4: OTG undervoltage fault mask shift
#define BQ25672_OTG_UVP_MASK_MASK      (1U << BQ25672_OTG_UVP_MASK_SHIFT)
#define BQ25672_TSHUT_MASK_SHIFT       2U  // Bit 2: Thermal shutdown fault mask shift
#define BQ25672_TSHUT_MASK_MASK        (1U << BQ25672_TSHUT_MASK_SHIFT)

// REG2E: ADC Control
#define BQ25672_ADC_EN_SHIFT           7U  // Bit 7: Enable ADC
#define BQ25672_ADC_EN_MASK            (1U << BQ25672_ADC_EN_SHIFT)
#define BQ25672_ADC_RATE_SHIFT         6U  // Bit 6: ADC fast/slow mode
#define BQ25672_ADC_RATE_MASK          (1U << BQ25672_ADC_RATE_SHIFT)
#define BQ25672_ADC_SAMPLE_SHIFT       4U  // Bits 5-4: ADC sample bits
#define BQ25672_ADC_SAMPLE_MASK        (3U << BQ25672_ADC_SAMPLE_SHIFT)
#define BQ25672_ADC_AVG_SHIFT          3U  // Bit 3: ADC averaging enable
#define BQ25672_ADC_AVG_MASK           (1U << BQ25672_ADC_AVG_SHIFT)
#define BQ25672_ADC_AVG_INIT_SHIFT     2U  // Bit 2: ADC averaging initialize
#define BQ25672_ADC_AVG_INIT_MASK      (1U << BQ25672_ADC_AVG_INIT_SHIFT)

// REG2F: ADC Function Disable 0
#define BQ25672_IBUS_ADC_DIS_SHIFT    7U   // Bit 7: Disable IBUS current ADC reading
#define BQ25672_IBUS_ADC_DIS_MASK     (1U << BQ25672_IBUS_ADC_DIS_SHIFT)  // Mask for IBUS ADC disable
#define BQ25672_IBAT_ADC_DIS_SHIFT    6U   // Bit 6: Disable IBAT current ADC reading
#define BQ25672_IBAT_ADC_DIS_MASK     (1U << BQ25672_IBAT_ADC_DIS_SHIFT)  // Mask for IBAT ADC disable
#define BQ25672_VBUS_ADC_DIS_SHIFT    5U   // Bit 5: Disable VBUS voltage ADC reading
#define BQ25672_VBUS_ADC_DIS_MASK     (1U << BQ25672_VBUS_ADC_DIS_SHIFT)  // Mask for VBUS ADC disable
#define BQ25672_VBAT_ADC_DIS_SHIFT    4U   // Bit 4: Disable VBAT voltage ADC reading
#define BQ25672_VBAT_ADC_DIS_MASK     (1U << BQ25672_VBAT_ADC_DIS_SHIFT)  // Mask for VBAT ADC disable
#define BQ25672_SYS_ADC_DIS_SHIFT     3U   // Bit 3: Disable VSYS voltage ADC reading
#define BQ25672_SYS_ADC_DIS_MASK      (1U << BQ25672_SYS_ADC_DIS_SHIFT)   // Mask for VSYS ADC disable
#define BQ25672_TS_ADC_DIS_SHIFT      2U   // Bit 2: Disable TS (temperature sense) ADC reading
#define BQ25672_TS_ADC_DIS_MASK       (1U << BQ25672_TS_ADC_DIS_SHIFT)    // Mask for TS ADC disable
#define BQ25672_TDIE_ADC_DIS_SHIFT    1U   // Bit 1: Disable internal die temperature ADC reading
#define BQ25672_TDIE_ADC_DIS_MASK     (1U << BQ25672_TDIE_ADC_DIS_SHIFT)  // Mask for TDIE ADC disable

// REG30: ADC Function Disable 1
#define BQ25672_DP_ADC_DIS_SHIFT      7U   // Bit 7: Disable D+ ADC reading
#define BQ25672_DP_ADC_DIS_MASK       (1U << BQ25672_DP_ADC_DIS_SHIFT)    // Mask for D+ ADC disable
#define BQ25672_DM_ADC_DIS_SHIFT      6U   // Bit 6: Disable D- ADC reading
#define BQ25672_DM_ADC_DIS_MASK       (1U << BQ25672_DM_ADC_DIS_SHIFT)    // Mask for D- ADC disable
#define BQ25672_VAC2_ADC_DIS_SHIFT    5U   // Bit 5: Disable VAC2 ADC reading
#define BQ25672_VAC2_ADC_DIS_MASK     (1U << BQ25672_VAC2_ADC_DIS_SHIFT)  // Mask for VAC2 ADC disable
#define BQ25672_VAC1_ADC_DIS_SHIFT    4U   // Bit 4: Disable VAC1 ADC reading
#define BQ25672_VAC1_ADC_DIS_MASK     (1U << BQ25672_VAC1_ADC_DIS_SHIFT)  // Mask for VAC1 ADC disable

// REG47: DPDM Driver
#define BQ25672_DPLUS_DAC_SHIFT        (0x5)
#define BQ25672_DPLUS_DAC_MASK         (0x7 << BQ25672_DPLUS_DAC_SHIFT)
#define BQ25672_DMINUS_DAC_SHIFT       (0x2)
#define BQ25672_DMINUS_DAC_MASK        (0x7 << BQ25672_DMINUS_DAC_SHIFT)

// REG48: Part Information
#define BQ25672_PN_SHIFT            3U            // Bit position for part number (bits 5:3)
#define BQ25672_PN_MASK             (7U << BQ25672_PN_SHIFT)   // 3-bit mask for part number field
#define BQ25672_DEV_REV_SHIFT       0U            // Bit position for device revision (bits 2:0)
#define BQ25672_DEV_REV_MASK        (7U << BQ25672_DEV_REV_SHIFT)  // 3-bit mask for device revision field

#define BQ25672_I2C_ADDR (0x6B << 1)
#define BQ25672_STARTUP_DELAY_MS    25

/* #######################################################################################################*\
 *  																									   *
 * 									          FUNCTIONS													   *
 *  																									   *
\*########################################################################################################*/

bq25672_status_t read_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t* val);
bq25672_status_t write_reg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t val);

bq25672_status_t bq25672_set_system_config(I2C_HandleTypeDef *hi2c, const bq25672_system_config_t *cfg);
bq25672_status_t bq25672_get_system_config(I2C_HandleTypeDef *hi2c, bq25672_system_config_t *cfg);

bq25672_status_t bq25672_set_charge_config(I2C_HandleTypeDef *hi2c, const bq25672_charge_config_t *cfg);
bq25672_status_t bq25672_get_charge_config(I2C_HandleTypeDef *hi2c, bq25672_charge_config_t *cfg);

bq25672_status_t bq25672_set_usb_config(I2C_HandleTypeDef *hi2c, const bq25672_usb_config_t *cfg);
bq25672_status_t bq25672_get_usb_config(I2C_HandleTypeDef *hi2c, bq25672_usb_config_t *cfg);

bq25672_status_t bq25672_set_timer_config(I2C_HandleTypeDef *hi2c, const bq25672_timer_config_t *cfg);
bq25672_status_t bq25672_get_timer_config(I2C_HandleTypeDef *hi2c, bq25672_timer_config_t *cfg);

bq25672_status_t bq25672_set_watchdog_config(I2C_HandleTypeDef *hi2c, const bq25672_watchdog_config_t *cfg);
bq25672_status_t bq25672_get_watchdog_config(I2C_HandleTypeDef *hi2c, bq25672_watchdog_config_t *cfg);

bq25672_status_t bq25672_set_mppt_config(I2C_HandleTypeDef *hi2c, const bq25672_mppt_config_t *cfg);
bq25672_status_t bq25672_get_mppt_config(I2C_HandleTypeDef *hi2c, bq25672_mppt_config_t *cfg);

bq25672_status_t bq25672_set_ntc_config(I2C_HandleTypeDef *hi2c, const bq25672_ntc_config_t *cfg);
bq25672_status_t bq25672_get_ntc_config(I2C_HandleTypeDef *hi2c, bq25672_ntc_config_t *cfg);

bq25672_status_t bq25672_set_temp_config(I2C_HandleTypeDef *hi2c, const bq25672_temp_config_t *cfg);
bq25672_status_t bq25672_get_temp_config(I2C_HandleTypeDef *hi2c, bq25672_temp_config_t *cfg);

bq25672_status_t bq25672_set_interrupt_config(I2C_HandleTypeDef *hi2c, const bq25672_interrupt_config_t *cfg);
bq25672_status_t bq25672_get_interrupt_config(I2C_HandleTypeDef *hi2c, bq25672_interrupt_config_t *cfg);

bq25672_status_t bq25672_read_ibus_current_ma(I2C_HandleTypeDef *hi2c, int16_t *ma);
bq25672_status_t bq25672_read_vbus_voltage_mv(I2C_HandleTypeDef *hi2c, uint16_t *mv);
bq25672_status_t bq25672_read_ibat_current_ma(I2C_HandleTypeDef *hi2c, int16_t *ma);
bq25672_status_t bq25672_read_vbat_voltage_mv(I2C_HandleTypeDef *hi2c, uint16_t *mv);

bq25672_status_t bq25672_get_device_status(I2C_HandleTypeDef *hi2c, bq25672_device_status_t *status);
bq25672_status_t bq25672_get_fault_status(I2C_HandleTypeDef *hi2c, bq25672_fault_status_t *status);

bq25672_status_t bq25672_set_charging_state(I2C_HandleTypeDef *hi2c, bool enabled);
bq25672_status_t bq25672_get_charging_state(I2C_HandleTypeDef *hi2c, bool *enabled);
bq25672_status_t bq25672_set_hiz_state(I2C_HandleTypeDef *hi2c, bool enabled);
bq25672_status_t bq25672_set_otg_state(I2C_HandleTypeDef *hi2c, bool enabled);

bq25672_status_t bq25672_apply_config(I2C_HandleTypeDef *hi2c, const bq25672_config_t *config);
bq25672_status_t bq25672_read_config(I2C_HandleTypeDef *hi2c, bq25672_config_t *out);

bq25672_status_t bq25672_init(I2C_HandleTypeDef *hi2c, const bq25672_config_t *config);
bq25672_status_t bq25672_soft_reset(I2C_HandleTypeDef *hi2c);

#endif /* INC_BQ25672_H_ */
