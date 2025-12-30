/**
 * @file ld2410_enhanced.h
 * @brief Enhanced LD2410C mmWave Radar Driver for ESP32-C6 Zigbee Sensor
 *
 * This enhanced driver extracts ALL available data from the LD2410C sensor:
 * - Target states (moving, static, occupancy)
 * - Distance measurements (moving, static, detection)
 * - Energy values (moving, static)
 * - Per-gate energy values (engineering mode)
 * - Per-gate sensitivity configuration
 * - Firmware version
 *
 * Based on SmartHomeScene SHS01 firmware with significant additions.
 * Protocol reference: HLK-LD2410 Serial Communication Protocol v1.02
 *
 * @author Enhanced by Claude for Thibault
 * @date 2025
 */

#ifndef LD2410_ENHANCED_H
#define LD2410_ENHANCED_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/uart.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================ */

#define LD2410_UART_NUM             UART_NUM_1
#define LD2410_UART_TX_PIN          5
#define LD2410_UART_RX_PIN          4
#define LD2410_UART_BAUD            256000
#define LD2410_UART_BUF_SIZE        512

#define LD2410_MAX_GATES            9           // Gates 0-8
#define LD2410_GATE_RESOLUTION_CM   75          // Each gate = 0.75m = 75cm

/* Frame markers */
#define LD2410_DATA_FRAME_HEADER    0xF4F3F2F1  // Data output frame header
#define LD2410_DATA_FRAME_FOOTER    0xF8F7F6F5  // Data output frame footer
#define LD2410_CMD_FRAME_HEADER     0xFDFCFBFA  // Command frame header
#define LD2410_CMD_FRAME_FOOTER     0x04030201  // Command frame footer

/* Data types in report frames */
#define LD2410_DATA_TYPE_ENGINEERING    0x01
#define LD2410_DATA_TYPE_BASIC          0x02

/* Command words */
#define LD2410_CMD_ENABLE_CONFIG        0x00FF
#define LD2410_CMD_END_CONFIG           0x00FE
#define LD2410_CMD_SET_MAX_GATE_TIMEOUT 0x0060
#define LD2410_CMD_READ_PARAMS          0x0061
#define LD2410_CMD_ENABLE_ENGINEERING   0x0062
#define LD2410_CMD_DISABLE_ENGINEERING  0x0063
#define LD2410_CMD_SET_GATE_SENSITIVITY 0x0064
#define LD2410_CMD_READ_FIRMWARE        0x00A0
#define LD2410_CMD_SET_BAUD             0x00A1
#define LD2410_CMD_FACTORY_RESET        0x00A2
#define LD2410_CMD_RESTART              0x00A3

/* Target state bits */
#define LD2410_STATE_NO_TARGET          0x00
#define LD2410_STATE_MOVING             0x01
#define LD2410_STATE_STATIC             0x02
#define LD2410_STATE_MOVING_AND_STATIC  0x03

/* ============================================================================
 * DATA STRUCTURES
 * ============================================================================ */

/**
 * @brief Per-gate sensitivity settings
 */
typedef struct {
    uint8_t move_sensitivity;   // Motion sensitivity (0-100)
    uint8_t still_sensitivity;  // Static sensitivity (0-100)
} ld2410_gate_config_t;

/**
 * @brief Per-gate energy readings (engineering mode)
 */
typedef struct {
    uint8_t move_energy;        // Motion energy value (0-100)
    uint8_t still_energy;       // Static energy value (0-100)
} ld2410_gate_energy_t;

/**
 * @brief Firmware version information
 */
typedef struct {
    uint8_t major;              // Major version
    uint8_t minor;              // Minor version
    uint32_t build;             // Build number (date encoded)
    bool valid;                 // Whether version info is valid
} ld2410_firmware_t;

/**
 * @brief Basic target data (always available)
 */
typedef struct {
    uint8_t target_state;       // 0=none, 1=moving, 2=static, 3=both
    uint16_t moving_distance;   // Moving target distance in cm
    uint8_t moving_energy;      // Moving target energy (0-100)
    uint16_t static_distance;   // Static target distance in cm
    uint8_t static_energy;      // Static target energy (0-100)
    uint16_t detection_distance;// Detection distance in cm
} ld2410_target_data_t;

/**
 * @brief Engineering mode data (per-gate values)
 */
typedef struct {
    uint8_t max_moving_gate;    // Configured max moving gate
    uint8_t max_static_gate;    // Configured max static gate
    ld2410_gate_energy_t gates[LD2410_MAX_GATES];  // Energy per gate
    bool valid;                 // Whether engineering data is valid
} ld2410_engineering_data_t;

/**
 * @brief Configuration parameters
 */
typedef struct {
    uint8_t max_moving_gate;    // Max gate for motion detection (0-8)
    uint8_t max_static_gate;    // Max gate for static detection (2-8)
    uint16_t timeout_seconds;   // No-one duration in seconds
    ld2410_gate_config_t gates[LD2410_MAX_GATES];  // Per-gate sensitivity
    bool valid;                 // Whether config is valid
} ld2410_config_t;

/**
 * @brief Complete sensor state
 */
typedef struct {
    // Basic target data
    ld2410_target_data_t target;

    // Derived boolean states (with cooldown applied)
    bool moving_detected;
    bool static_detected;
    bool occupancy_detected;

    // Engineering mode data
    ld2410_engineering_data_t engineering;
    bool engineering_mode_enabled;

    // Configuration
    ld2410_config_t config;

    // Firmware info
    ld2410_firmware_t firmware;

    // Connection status
    bool connected;
    uint32_t last_frame_time;
    uint32_t frame_count;
    uint32_t error_count;

    // Cooldown state
    uint32_t moving_cooldown_until;
    uint16_t moving_cooldown_seconds;
    uint16_t occupancy_clear_delay;
} ld2410_state_t;

/* ============================================================================
 * CALLBACK TYPES
 * ============================================================================ */

/**
 * @brief Callback for target state changes
 */
typedef void (*ld2410_state_callback_t)(const ld2410_state_t *state);

/**
 * @brief Callback for distance updates
 */
typedef void (*ld2410_distance_callback_t)(
    uint16_t moving_distance,
    uint16_t static_distance,
    uint16_t detection_distance
);

/**
 * @brief Callback for energy updates
 */
typedef void (*ld2410_energy_callback_t)(
    uint8_t moving_energy,
    uint8_t static_energy
);

/**
 * @brief Callback for engineering mode gate data
 */
typedef void (*ld2410_gate_callback_t)(const ld2410_engineering_data_t *data);

/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

/**
 * @brief Initialize the LD2410 driver
 * @return ESP_OK on success
 */
esp_err_t ld2410_init(void);

/**
 * @brief Deinitialize the LD2410 driver
 */
void ld2410_deinit(void);

/**
 * @brief Get current sensor state
 * @return Pointer to current state (read-only)
 */
const ld2410_state_t* ld2410_get_state(void);

/**
 * @brief Process incoming UART data (call from main loop or task)
 */
void ld2410_process(void);

/* Configuration Commands */

/**
 * @brief Enable engineering mode (per-gate energy values)
 * @return ESP_OK on success
 */
esp_err_t ld2410_enable_engineering_mode(void);

/**
 * @brief Disable engineering mode
 * @return ESP_OK on success
 */
esp_err_t ld2410_disable_engineering_mode(void);

/**
 * @brief Set maximum detection gates and timeout
 * @param max_moving_gate Max gate for motion (0-8)
 * @param max_static_gate Max gate for static (2-8)
 * @param timeout_seconds No-one duration in seconds
 * @return ESP_OK on success
 */
esp_err_t ld2410_set_max_gate_timeout(
    uint8_t max_moving_gate,
    uint8_t max_static_gate,
    uint16_t timeout_seconds
);

/**
 * @brief Set sensitivity for a specific gate
 * @param gate Gate number (0-8)
 * @param move_sensitivity Motion sensitivity (0-100)
 * @param still_sensitivity Static sensitivity (0-100)
 * @return ESP_OK on success
 */
esp_err_t ld2410_set_gate_sensitivity(
    uint8_t gate,
    uint8_t move_sensitivity,
    uint8_t still_sensitivity
);

/**
 * @brief Set sensitivity for all gates at once
 * @param move_sensitivity Motion sensitivity (0-100)
 * @param still_sensitivity Static sensitivity (0-100)
 * @return ESP_OK on success
 */
esp_err_t ld2410_set_all_sensitivity(
    uint8_t move_sensitivity,
    uint8_t still_sensitivity
);

/**
 * @brief Read current configuration from sensor
 * @return ESP_OK on success
 */
esp_err_t ld2410_read_config(void);

/**
 * @brief Read firmware version
 * @return ESP_OK on success
 */
esp_err_t ld2410_read_firmware_version(void);

/**
 * @brief Factory reset the sensor
 * @return ESP_OK on success
 */
esp_err_t ld2410_factory_reset(void);

/**
 * @brief Restart the sensor
 * @return ESP_OK on success
 */
esp_err_t ld2410_restart(void);

/* Cooldown Configuration */

/**
 * @brief Set movement detection cooldown
 * @param seconds Cooldown time (0 to disable)
 */
void ld2410_set_moving_cooldown(uint16_t seconds);

/**
 * @brief Set occupancy clear delay
 * @param seconds Delay before occupancy clears
 */
void ld2410_set_occupancy_delay(uint16_t seconds);

/* Callback Registration */

/**
 * @brief Register callback for state changes
 */
void ld2410_register_state_callback(ld2410_state_callback_t callback);

/**
 * @brief Register callback for distance updates
 */
void ld2410_register_distance_callback(ld2410_distance_callback_t callback);

/**
 * @brief Register callback for energy updates
 */
void ld2410_register_energy_callback(ld2410_energy_callback_t callback);

/**
 * @brief Register callback for gate data (engineering mode)
 */
void ld2410_register_gate_callback(ld2410_gate_callback_t callback);

/* Utility Functions */

/**
 * @brief Convert gate number to distance in cm
 */
static inline uint16_t ld2410_gate_to_cm(uint8_t gate) {
    return gate * LD2410_GATE_RESOLUTION_CM;
}

/**
 * @brief Convert distance in cm to gate number
 */
static inline uint8_t ld2410_cm_to_gate(uint16_t cm) {
    return cm / LD2410_GATE_RESOLUTION_CM;
}

/**
 * @brief Check if sensor is connected and responding
 */
bool ld2410_is_connected(void);

/**
 * @brief Get string representation of target state
 */
const char* ld2410_state_to_string(uint8_t state);

#ifdef __cplusplus
}
#endif

#endif /* LD2410_ENHANCED_H */
