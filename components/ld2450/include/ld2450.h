/**
 * @file ld2450.h
 * @brief HLK-LD2450 mmWave Radar Multi-Target Tracking Driver for ESP32-C6
 *
 * This driver implements the complete LD2450 protocol for multi-target tracking
 * with zone-based detection and filtering.
 *
 * Features:
 * - Up to 3 simultaneous target tracking
 * - X/Y coordinate tracking (-3000mm to +3000mm X, 0-6000mm Y)
 * - Speed, distance, and angle measurements per target
 * - Up to 3 configurable rectangular zones
 * - Zone types: disabled, detection (include), filter (exclude)
 * - Real-time zone occupancy detection
 *
 * Protocol reference: HLK-LD2450 Serial Communication Protocol
 *
 * @author Claude for Thibault
 * @date 2025-12-23
 */

#ifndef LD2450_H
#define LD2450_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * CONFIGURATION CONSTANTS
 * ============================================================================ */

#define LD2450_UART_NUM             UART_NUM_0
#define LD2450_UART_TX_PIN          18
#define LD2450_UART_RX_PIN          19
#define LD2450_UART_BAUD            256000  /* Sensor default baud rate */
#define LD2450_UART_BUF_SIZE        2048    /* Increased from 512 to reduce frame corruption */

#define LD2450_MAX_TARGETS          3
#define LD2450_MAX_ZONES            3

/* Coordinate system limits (millimeters) */
#define LD2450_X_MIN                -3000
#define LD2450_X_MAX                3000
#define LD2450_Y_MIN                0
#define LD2450_Y_MAX                6000

/* Detection range */
#define LD2450_MAX_RANGE_MM         6000
#define LD2450_FOV_DEGREES          120

/* Frame markers */
#define LD2450_FRAME_HEADER         0xAA
#define LD2450_FRAME_FOOTER         0x55  /* Standard footer */
#define LD2450_FRAME_FOOTER_ALT     0x00  /* Alternative footer (some firmware versions) */
#define LD2450_FRAME_CHECK          0x00

/* Command frame markers */
#define LD2450_CMD_HEADER           0xFD
#define LD2450_CMD_FOOTER           0x04

/* Command words */
#define LD2450_CMD_ENABLE_CONFIG    0x00FF
#define LD2450_CMD_END_CONFIG       0x00FE
#define LD2450_CMD_SET_ZONE         0x0080
#define LD2450_CMD_READ_VERSION     0x00A0
#define LD2450_CMD_RESTART          0x00A3
#define LD2450_CMD_FACTORY_RESET    0x00A2
#define LD2450_CMD_SET_BAUD         0x00A1

/* ============================================================================
 * ENUMERATIONS
 * ============================================================================ */

/**
 * @brief Zone operation modes
 */
typedef enum {
    LD2450_ZONE_DISABLED = 0,   // No zone filtering
    LD2450_ZONE_DETECTION = 1,  // Only detect targets INSIDE zones (inclusion)
    LD2450_ZONE_FILTER = 2      // Exclude targets INSIDE zones (exclusion)
} ld2450_zone_type_t;

/* ============================================================================
 * DATA STRUCTURES
 * ============================================================================ */

/**
 * @brief Single target tracking data
 */
typedef struct {
    int16_t x;              // X position in mm (-3000 to +3000)
    int16_t y;              // Y position in mm (0 to 6000)
    int16_t speed;          // Speed in mm/s (signed, -/+ indicates direction)
    uint16_t distance;      // Distance from sensor in mm
    uint16_t angle;         // Angle in degrees * 10 (e.g., 450 = 45.0°)
    bool active;            // Whether this target slot is active
} ld2450_target_t;

/**
 * @brief Rectangular zone definition
 */
typedef struct {
    int16_t x1;             // Near-left X coordinate (mm)
    int16_t y1;             // Near-left Y coordinate (mm)
    int16_t x2;             // Far-right X coordinate (mm)
    int16_t y2;             // Far-right Y coordinate (mm)
    bool occupied;          // Whether zone currently has target(s)
    bool enabled;           // Whether zone is defined/enabled
    uint8_t target_count;   // Number of targets currently in this zone (0-3)
} ld2450_zone_t;

/**
 * @brief Zone configuration
 */
typedef struct {
    ld2450_zone_type_t type;        // Zone operation mode
    ld2450_zone_t zones[LD2450_MAX_ZONES];  // Zone definitions
} ld2450_zone_config_t;

/**
 * @brief Firmware version information
 */
typedef struct {
    uint8_t type;           // Firmware type
    uint8_t major;          // Major version
    uint8_t minor;          // Minor version
    uint32_t build;         // Build number
    bool valid;             // Whether version is valid
} ld2450_firmware_t;

/**
 * @brief Complete sensor state
 */
typedef struct {
    // Target tracking
    ld2450_target_t targets[LD2450_MAX_TARGETS];
    uint8_t active_targets;         // Number of currently active targets

    // Zone configuration and status
    ld2450_zone_config_t zone_config;

    // Derived occupancy state (after zone filtering)
    bool occupancy_detected;        // Any target present (after zone filtering)

    // Firmware info
    ld2450_firmware_t firmware;

    // Connection status
    bool connected;
    uint32_t last_frame_time;
    uint32_t frame_count;
    uint32_t error_count;
} ld2450_state_t;

/* ============================================================================
 * CALLBACK TYPES
 * ============================================================================ */

/**
 * @brief Callback for target updates
 * @param targets Array of target data
 * @param active_count Number of active targets
 */
typedef void (*ld2450_target_callback_t)(
    const ld2450_target_t *targets,
    uint8_t active_count
);

/**
 * @brief Callback for zone occupancy changes
 * @param zones Array of zones with occupancy status
 * @param occupancy Overall occupancy state
 */
typedef void (*ld2450_zone_callback_t)(
    const ld2450_zone_t *zones,
    bool occupancy
);

/**
 * @brief Callback for state changes
 */
typedef void (*ld2450_state_callback_t)(const ld2450_state_t *state);

/* ============================================================================
 * PUBLIC API
 * ============================================================================ */

/**
 * @brief Initialize the LD2450 driver
 * @return ESP_OK on success
 */
esp_err_t ld2450_init(void);

/**
 * @brief Deinitialize the LD2450 driver
 */
void ld2450_deinit(void);

/**
 * @brief Get current sensor state
 * @return Pointer to current state (read-only)
 */
const ld2450_state_t* ld2450_get_state(void);

/**
 * @brief Process incoming UART data (call from main loop)
 */
void ld2450_process(void);

/**
 * @brief Check if sensor is connected and responding
 * @return true if connected
 */
bool ld2450_is_connected(void);

/**
 * @brief Get UART bytes received count (for diagnostics)
 * @return Total bytes received since init
 */
uint32_t ld2450_get_bytes_received(void);

/* Configuration Commands */

/**
 * @brief Set zone configuration
 * @param zone_num Zone number (0-2)
 * @param x1 Near-left X coordinate (mm)
 * @param y1 Near-left Y coordinate (mm)
 * @param x2 Far-right X coordinate (mm)
 * @param y2 Far-right Y coordinate (mm)
 * @return ESP_OK on success
 */
esp_err_t ld2450_set_zone(
    uint8_t zone_num,
    int16_t x1, int16_t y1,
    int16_t x2, int16_t y2
);

/**
 * @brief Clear/disable a zone
 * @param zone_num Zone number (0-2)
 * @return ESP_OK on success
 */
esp_err_t ld2450_clear_zone(uint8_t zone_num);

/**
 * @brief Set zone operation type
 * @param type Zone type (disabled/detection/filter)
 * @return ESP_OK on success
 */
esp_err_t ld2450_set_zone_type(ld2450_zone_type_t type);

/**
 * @brief Apply zone configuration to sensor (internal use)
 * Called after zone changes to update sensor
 */
esp_err_t ld2450_apply_zones(void);

/**
 * @brief Read firmware version from sensor
 * @return ESP_OK on success
 */
esp_err_t ld2450_read_firmware_version(void);

/**
 * @brief Restart the sensor
 * @return ESP_OK on success
 */
esp_err_t ld2450_restart(void);

/**
 * @brief Factory reset the sensor
 * @return ESP_OK on success
 */
esp_err_t ld2450_factory_reset(void);

/* Callback Registration */

/**
 * @brief Register callback for target updates
 */
void ld2450_register_target_callback(ld2450_target_callback_t callback);

/**
 * @brief Register callback for zone occupancy changes
 */
void ld2450_register_zone_callback(ld2450_zone_callback_t callback);

/**
 * @brief Register callback for state changes
 */
void ld2450_register_state_callback(ld2450_state_callback_t callback);

/* Utility Functions */

/**
 * @brief Calculate distance from X/Y coordinates
 * @param x X coordinate (mm)
 * @param y Y coordinate (mm)
 * @return Distance in mm
 */
static inline uint16_t ld2450_calc_distance(int16_t x, int16_t y) {
    /* Pythagorean distance: sqrt(x² + y²) */
    uint32_t dist_sq = ((int32_t)x * x) + ((int32_t)y * y);

    /* Integer square root using Newton's method */
    if (dist_sq == 0) return 0;

    uint32_t result = dist_sq;
    uint32_t temp;

    /* Initial guess: half the number of bits */
    while (result > 0xFFFF) {
        result >>= 1;
    }
    if (result == 0) result = 1;

    /* Newton's method iterations */
    for (int i = 0; i < 8; i++) {
        temp = (result + dist_sq / result) >> 1;
        if (temp >= result) break;
        result = temp;
    }

    return (uint16_t)result;
}

/**
 * @brief Calculate angle from X/Y coordinates
 * @param x X coordinate (mm)
 * @param y Y coordinate (mm)
 * @return Angle in degrees * 10
 */
uint16_t ld2450_calc_angle(int16_t x, int16_t y);

/**
 * @brief Check if point is inside zone
 * @param x X coordinate (mm)
 * @param y Y coordinate (mm)
 * @param zone Zone definition
 * @return true if point is inside zone
 */
bool ld2450_point_in_zone(int16_t x, int16_t y, const ld2450_zone_t *zone);

/**
 * @brief Get string representation of zone type
 */
const char* ld2450_zone_type_to_string(ld2450_zone_type_t type);

/**
 * @brief Enable/disable verbose logging (target position logs)
 * @param enable true to enable verbose logging, false to disable
 */
void ld2450_set_verbose_logging(bool enable);

#ifdef __cplusplus
}
#endif

#endif /* LD2450_H */
