/**
 * @file ld2450.c
 * @brief HLK-LD2450 mmWave Radar Multi-Target Tracking Driver Implementation
 *
 * Complete implementation of the LD2450 protocol with multi-target tracking,
 * zone-based detection, and real-time occupancy monitoring.
 */

#include "ld2450.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <math.h>

static const char *TAG = "LD2450";

/* ============================================================================
 * PRIVATE DATA
 * ============================================================================ */

static ld2450_state_t s_state = {0};
static uint8_t s_rx_buffer[LD2450_UART_BUF_SIZE];
static uint8_t s_frame_buffer[128];
static int s_frame_pos = 0;
static bool s_in_frame = false;

/* Diagnostic counter */
static uint32_t s_total_bytes_received = 0;
static uint32_t s_last_status_time = 0;
#define LD2450_STATUS_INTERVAL_MS  10000  /* Log status every 10 seconds */

/* Frame timeout and watchdog */
#define LD2450_FRAME_TIMEOUT_MS     200
#define LD2450_WATCHDOG_TIMEOUT_MS  10000  /* Increased from 5s to 10s to reduce false positives */
static uint32_t s_last_valid_frame_time = 0;
static uint32_t s_frame_start_time = 0;

/* Callbacks */
static ld2450_target_callback_t s_target_callback = NULL;
static ld2450_zone_callback_t s_zone_callback = NULL;
static ld2450_state_callback_t s_state_callback = NULL;

/* Command response handling */
static bool s_waiting_response = false;
static uint16_t s_expected_cmd = 0;
static uint8_t s_response_buffer[64];
static int s_response_len = 0;
static bool s_response_received = false;

/* Zone configuration is now maintained in the main state structure */

/* Verbose logging flag - only log target positions when enabled */
static bool s_verbose_logging = false;

/* ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================ */

/**
 * @brief Calculate angle from X/Y coordinates using atan2
 */
uint16_t ld2450_calc_angle(int16_t x, int16_t y) {
    if (y == 0 && x == 0) return 0;

    // atan2 returns radians, convert to degrees * 10
    float angle_rad = atan2f((float)x, (float)y);  // Note: atan2(x, y) for our coordinate system
    float angle_deg = angle_rad * 180.0f / M_PI;

    // Normalize to 0-360 range
    if (angle_deg < 0) angle_deg += 360.0f;

    return (uint16_t)(angle_deg * 10.0f);
}

/**
 * @brief Check if point is inside rectangular zone
 */
bool ld2450_point_in_zone(int16_t x, int16_t y, const ld2450_zone_t *zone) {
    if (!zone || !zone->enabled) return false;

    int16_t x_min = zone->x1 < zone->x2 ? zone->x1 : zone->x2;
    int16_t x_max = zone->x1 > zone->x2 ? zone->x1 : zone->x2;
    int16_t y_min = zone->y1 < zone->y2 ? zone->y1 : zone->y2;
    int16_t y_max = zone->y1 > zone->y2 ? zone->y1 : zone->y2;

    return (x >= x_min && x <= x_max && y >= y_min && y <= y_max);
}

/**
 * @brief Get string representation of zone type
 */
const char* ld2450_zone_type_to_string(ld2450_zone_type_t type) {
    switch (type) {
        case LD2450_ZONE_DISABLED:  return "disabled";
        case LD2450_ZONE_DETECTION: return "detection";
        case LD2450_ZONE_FILTER:    return "filter";
        default:                    return "unknown";
    }
}

/* ============================================================================
 * FRAME PARSING
 * ============================================================================ */

/**
 * @brief Reset frame parser state
 */
static void reset_frame_parser(void) {
    s_in_frame = false;
    s_frame_pos = 0;
    s_frame_start_time = 0;
}

/* Flush and reset removed - handled by watchdog in process loop */

/* Track previous zone states to detect individual zone changes */
static bool s_prev_zone_occupied[LD2450_MAX_ZONES] = {false, false, false};
static uint8_t s_prev_zone_target_count[LD2450_MAX_ZONES] = {0, 0, 0};

/**
 * @brief Update zone occupancy based on current targets
 */
static void update_zone_occupancy(void) {
    // Clear all zone occupancy flags and target counts
    for (int i = 0; i < LD2450_MAX_ZONES; i++) {
        s_state.zone_config.zones[i].occupied = false;
        s_state.zone_config.zones[i].target_count = 0;
    }

    // Check each active target against zones
    for (int t = 0; t < LD2450_MAX_TARGETS; t++) {
        if (!s_state.targets[t].active) continue;

        int16_t x = s_state.targets[t].x;
        int16_t y = s_state.targets[t].y;

        // Check which zones contain this target
        for (int z = 0; z < LD2450_MAX_ZONES; z++) {
            if (s_state.zone_config.zones[z].enabled) {
                if (ld2450_point_in_zone(x, y, &s_state.zone_config.zones[z])) {
                    s_state.zone_config.zones[z].occupied = true;
                    s_state.zone_config.zones[z].target_count++;
                }
            }
        }
    }

    // Calculate overall occupancy based on zone type
    bool prev_occupancy = s_state.occupancy_detected;

    switch (s_state.zone_config.type) {
        case LD2450_ZONE_DISABLED:
            // Any active target counts as occupancy
            s_state.occupancy_detected = (s_state.active_targets > 0);
            break;

        case LD2450_ZONE_DETECTION:
            // Only targets INSIDE zones count
            s_state.occupancy_detected = false;
            for (int i = 0; i < LD2450_MAX_ZONES; i++) {
                if (s_state.zone_config.zones[i].occupied) {
                    s_state.occupancy_detected = true;
                    break;
                }
            }
            break;

        case LD2450_ZONE_FILTER:
            // Targets OUTSIDE zones count (exclusion mode)
            s_state.occupancy_detected = false;
            for (int t = 0; t < LD2450_MAX_TARGETS; t++) {
                if (!s_state.targets[t].active) continue;

                bool in_any_zone = false;
                int16_t x = s_state.targets[t].x;
                int16_t y = s_state.targets[t].y;

                for (int z = 0; z < LD2450_MAX_ZONES; z++) {
                    if (s_state.zone_config.zones[z].enabled &&
                        ld2450_point_in_zone(x, y, &s_state.zone_config.zones[z])) {
                        in_any_zone = true;
                        break;
                    }
                }

                if (!in_any_zone) {
                    s_state.occupancy_detected = true;
                    break;
                }
            }
            break;

        case LD2450_ZONE_INTERFERENCE:
            // Similar to filter - targets in interference zones are noise/false positives
            // Only targets OUTSIDE interference zones count as real occupancy
            s_state.occupancy_detected = false;
            for (int t = 0; t < LD2450_MAX_TARGETS; t++) {
                if (!s_state.targets[t].active) continue;

                bool in_any_zone = false;
                int16_t x = s_state.targets[t].x;
                int16_t y = s_state.targets[t].y;

                for (int z = 0; z < LD2450_MAX_ZONES; z++) {
                    if (s_state.zone_config.zones[z].enabled &&
                        ld2450_point_in_zone(x, y, &s_state.zone_config.zones[z])) {
                        in_any_zone = true;
                        break;
                    }
                }

                if (!in_any_zone) {
                    s_state.occupancy_detected = true;
                    break;
                }
            }
            break;
    }

    // Check if any zone state changed (occupancy or target count)
    bool zones_changed = false;
    for (int i = 0; i < LD2450_MAX_ZONES; i++) {
        if (s_state.zone_config.zones[i].enabled) {
            if (s_prev_zone_occupied[i] != s_state.zone_config.zones[i].occupied ||
                s_prev_zone_target_count[i] != s_state.zone_config.zones[i].target_count) {
                zones_changed = true;
            }
        }
        // Update previous state
        s_prev_zone_occupied[i] = s_state.zone_config.zones[i].occupied;
        s_prev_zone_target_count[i] = s_state.zone_config.zones[i].target_count;
    }

    // Fire zone callback if any zone changed OR overall occupancy changed
    if ((zones_changed || prev_occupancy != s_state.occupancy_detected) && s_zone_callback) {
        s_zone_callback(s_state.zone_config.zones, s_state.occupancy_detected);
    }
}

/**
 * @brief Parse target data frame
 *
 * Frame structure (LD2450 data output - 30 bytes total):
 * [0-3]   = Header: 0xAA 0xFF 0x03 0x00
 * [4-5]   = Target 1 X (little-endian, signed, mm)
 * [6-7]   = Target 1 Y (little-endian, signed, mm)
 * [8-9]   = Target 1 Speed (little-endian, signed, cm/s)
 * [10-11] = Target 1 Resolution (distance resolution)
 * [12-13] = Target 2 X
 * [14-15] = Target 2 Y
 * [16-17] = Target 2 Speed
 * [18-19] = Target 2 Resolution
 * [20-21] = Target 3 X
 * [22-23] = Target 3 Y
 * [24-25] = Target 3 Speed
 * [26-27] = Target 3 Resolution
 * [28-29] = Footer: 0x55 0xCC
 */
static void parse_target_frame(const uint8_t *data, int len) {
    if (len < 30) {
        ESP_LOGW(TAG, "Target frame too short: %d bytes (need 30)", len);
        return;
    }

    /* Validate full header: AA FF 03 00 */
    if (data[0] != 0xAA || data[1] != 0xFF || data[2] != 0x03 || data[3] != 0x00) {
        ESP_LOGW(TAG, "Invalid frame header: %02X %02X %02X %02X",
                 data[0], data[1], data[2], data[3]);
        return;
    }

    /* Validate footer: 55 CC */
    if (data[28] != 0x55 || data[29] != 0xCC) {
        ESP_LOGW(TAG, "Invalid frame footer: %02X %02X", data[28], data[29]);
        return;
    }

    // Parse all 3 targets - VALIDATE before accepting frame
    uint8_t active_count = 0;
    bool targets_changed = false;

    // First pass: validate all target data to detect corrupted frames
    for (int i = 0; i < LD2450_MAX_TARGETS; i++) {
        int offset = 4 + (i * 8);  /* Data starts at byte 4 after header */

        /* LD2450 coordinate format:
         * X: sign-magnitude (bit 15 = sign, bits 14-0 = magnitude) - can be negative (left/right)
         * Y: magnitude only (bit 15 ignored) - always positive (forward distance)
         * Speed: sign-magnitude - can be negative (towards/away from sensor)
         */
        uint16_t x_raw = (uint16_t)(data[offset] | (data[offset + 1] << 8));
        uint16_t y_raw = (uint16_t)(data[offset + 2] | (data[offset + 3] << 8));
        uint16_t speed_raw = (uint16_t)(data[offset + 4] | (data[offset + 5] << 8));

        /* Convert to standard signed/unsigned integers
         * NOTE: X axis is NOT inverted - raw sensor coordinates used directly
         */
        int16_t x = (x_raw & 0x8000) ? -(int16_t)(x_raw & 0x7FFF) : (int16_t)x_raw;
        int16_t y = (int16_t)(y_raw & 0x7FFF);  /* Y is always positive, mask off bit 15 */
        int16_t speed = (speed_raw & 0x8000) ? -(int16_t)(speed_raw & 0x7FFF) : (int16_t)speed_raw;

        // Skip inactive targets (0,0)
        if (x == 0 && y == 0) continue;

        // Validate sensor ranges
        if (x < -3000 || x > 3000 || y < 0 || y > 6000 || speed < -5000 || speed > 5000) {
            ESP_LOGW(TAG, "T%d: raw=[%02X %02X %02X %02X %02X %02X] X=%d Y=%d Spd=%d (REJECTED)",
                     i+1, data[offset], data[offset+1], data[offset+2], data[offset+3],
                     data[offset+4], data[offset+5], x, y, speed);
            s_state.error_count++;
            return;
        }
    }

    // Second pass: store validated data
    for (int i = 0; i < LD2450_MAX_TARGETS; i++) {
        int offset = 4 + (i * 8);  /* Data starts at byte 4 after header */

        /* LD2450 uses sign-magnitude format: bit 15 = sign, bits 14-0 = magnitude */
        uint16_t x_raw = (uint16_t)(data[offset] | (data[offset + 1] << 8));
        uint16_t y_raw = (uint16_t)(data[offset + 2] | (data[offset + 3] << 8));
        uint16_t speed_raw = (uint16_t)(data[offset + 4] | (data[offset + 5] << 8));

        /* Convert sign-magnitude to standard signed integer
         * X: sign-magnitude (bit 15 = sign, bits 14-0 = magnitude) - can be negative
         * Y: magnitude only (bit 15 ignored) - always positive (forward distance)
         * Speed: sign-magnitude - can be negative
         */
        int16_t x = (x_raw & 0x8000) ? -(int16_t)(x_raw & 0x7FFF) : (int16_t)x_raw;
        int16_t y = (int16_t)(y_raw & 0x7FFF);  /* Y is always positive, mask off bit 15 */
        int16_t speed = (speed_raw & 0x8000) ? -(int16_t)(speed_raw & 0x7FFF) : (int16_t)speed_raw;
        // Resolution data available but not currently used
        // uint16_t resolution = (uint16_t)(data[offset + 6] | (data[offset + 7] << 8));

        // Target is active if X and Y are non-zero (sensor reports 0,0 for inactive)
        // Note: False positives can be filtered using zone configuration in web app
        bool active = (x != 0 || y != 0);

        // Check if target state changed
        if (s_state.targets[i].active != active ||
            s_state.targets[i].x != x ||
            s_state.targets[i].y != y) {
            targets_changed = true;
        }

        s_state.targets[i].x = x;
        s_state.targets[i].y = y;
        s_state.targets[i].speed = speed;
        s_state.targets[i].active = active;

        if (active) {
            // Calculate derived values
            s_state.targets[i].distance = ld2450_calc_distance(x, y);
            s_state.targets[i].angle = ld2450_calc_angle(x, y);
            active_count++;
        } else {
            s_state.targets[i].distance = 0;
            s_state.targets[i].angle = 0;
        }
    }

    s_state.active_targets = active_count;

    // Update connection status
    uint32_t now = esp_timer_get_time() / 1000;
    s_state.connected = true;
    s_state.last_frame_time = now;
    s_state.frame_count++;
    s_last_valid_frame_time = now;

    // Update zone occupancy
    update_zone_occupancy();

    // Log target status only when verbose logging is enabled (position reporting mode)
    if (s_verbose_logging && (s_state.frame_count % 10 == 0 || targets_changed)) {
        ESP_LOGI(TAG, "Frame #%lu: %d active targets",
                 (unsigned long)s_state.frame_count, active_count);
        for (int i = 0; i < LD2450_MAX_TARGETS; i++) {
            if (s_state.targets[i].active) {
                int offset = 4 + (i * 8);
                uint16_t x_raw = (uint16_t)(data[offset] | (data[offset + 1] << 8));
                /* DIAGNOSTIC: Show raw X bytes and sign bit status */
                ESP_LOGI(TAG, "  T%d: raw_X=[%02X %02X]=0x%04X (bit15=%d) => X=%d Y=%d",
                         i + 1,
                         data[offset], data[offset + 1], x_raw,
                         (x_raw & 0x8000) ? 1 : 0,
                         s_state.targets[i].x,
                         s_state.targets[i].y);
            }
        }
    }

    // Fire callbacks
    if (targets_changed) {
        if (s_target_callback) {
            s_target_callback(s_state.targets, active_count);
        }
        if (s_state_callback) {
            s_state_callback(&s_state);
        }
    }
}

/**
 * @brief Parse command response frame
 */
static void parse_command_response(const uint8_t *data, int len) {
    if (len < 6) return;

    // Command response structure:
    // [0-1] = Command word (little-endian)
    // [2] = Status (0x00 = success, 0x01 = failure)
    // [3+] = Response data (varies by command)

    uint16_t cmd = data[0] | (data[1] << 8);
    uint8_t status = data[2];

    if (s_waiting_response && cmd == s_expected_cmd) {
        memcpy(s_response_buffer, data, len < 64 ? len : 64);
        s_response_len = len;
        s_response_received = true;
        s_waiting_response = false;

        ESP_LOGI(TAG, "Command 0x%04X response: %s", cmd, status == 0 ? "OK" : "FAILED");
    }
}

/* ============================================================================
 * UART PROCESSING
 * ============================================================================ */

/**
 * @brief Process incoming UART data and parse frames
 */
void ld2450_process(void) {
    uint32_t now = esp_timer_get_time() / 1000;

    /* Periodic status log for debugging */
    if ((now - s_last_status_time) > LD2450_STATUS_INTERVAL_MS) {
        s_last_status_time = now;
        ESP_LOGI(TAG, "Status: bytes=%lu, frames=%lu, errors=%lu, connected=%d, targets=%d",
                 (unsigned long)s_total_bytes_received,
                 (unsigned long)s_state.frame_count,
                 (unsigned long)s_state.error_count,
                 s_state.connected,
                 s_state.active_targets);
    }

    int len = uart_read_bytes(LD2450_UART_NUM, s_rx_buffer, LD2450_UART_BUF_SIZE, 0);
    if (len > 0) {
        s_total_bytes_received += len;  /* Track total bytes for diagnostics */
    }
    if (len <= 0) {
        // Check watchdog timeout
        if (s_state.connected && (now - s_last_valid_frame_time) > LD2450_WATCHDOG_TIMEOUT_MS) {
            ESP_LOGW(TAG, "Watchdog timeout - no frames for %lu ms",
                     (unsigned long)(now - s_last_valid_frame_time));
            s_state.connected = false;
            s_state.active_targets = 0;
            for (int i = 0; i < LD2450_MAX_TARGETS; i++) {
                s_state.targets[i].active = false;
            }
        }
        return;
    }

    // Process each byte
    for (int i = 0; i < len; i++) {
        uint8_t byte = s_rx_buffer[i];

        if (!s_in_frame) {
            // Look for frame header
            if (byte == LD2450_FRAME_HEADER || byte == LD2450_CMD_HEADER) {
                s_in_frame = true;
                s_frame_buffer[0] = byte;
                s_frame_pos = 1;
                s_frame_start_time = now;
            }
        } else {
            // Accumulate frame data
            if (s_frame_pos < sizeof(s_frame_buffer)) {
                s_frame_buffer[s_frame_pos++] = byte;
            }

            // Check for frame timeout
            if ((now - s_frame_start_time) > LD2450_FRAME_TIMEOUT_MS) {
                ESP_LOGW(TAG, "Frame timeout");
                reset_frame_parser();
                continue;
            }

            // Check for complete data frame (30 bytes: 4-byte header + 24 data + 2 footer)
            if (s_frame_buffer[0] == LD2450_FRAME_HEADER && s_frame_pos >= 30) {
                /* Validate footer: 0x55 0xCC */
                if (s_frame_buffer[28] == 0x55 && s_frame_buffer[29] == 0xCC) {
                    parse_target_frame(s_frame_buffer, s_frame_pos);
                } else {
                    ESP_LOGW(TAG, "Invalid data frame footer: %02X %02X", s_frame_buffer[28], s_frame_buffer[29]);
                }
                reset_frame_parser();
            }
            // Check for command response (varies by command, minimum 6 bytes)
            else if (s_frame_buffer[0] == LD2450_CMD_HEADER && s_frame_pos >= 6) {
                // Simple heuristic: if we have header + reasonable data, parse it
                if (s_waiting_response) {
                    parse_command_response(s_frame_buffer + 1, s_frame_pos - 1);
                }
                reset_frame_parser();
            }
        }
    }
}

/* ============================================================================
 * COMMAND TRANSMISSION
 * ============================================================================ */

/**
 * @brief Send command to sensor and optionally wait for response
 */
static esp_err_t send_command(uint16_t cmd, const uint8_t *params, int param_len, bool wait_response) {
    uint8_t cmd_buf[64];
    int pos = 0;

    // Build command frame
    cmd_buf[pos++] = LD2450_CMD_HEADER;     // Header
    cmd_buf[pos++] = LD2450_CMD_FOOTER;     // Footer (appears at start in LD2450)
    cmd_buf[pos++] = param_len + 2;         // Length (cmd + params)
    cmd_buf[pos++] = cmd & 0xFF;            // Command low byte
    cmd_buf[pos++] = (cmd >> 8) & 0xFF;     // Command high byte

    // Add parameters
    if (params && param_len > 0) {
        memcpy(&cmd_buf[pos], params, param_len);
        pos += param_len;
    }

    // Write to UART
    int written = uart_write_bytes(LD2450_UART_NUM, cmd_buf, pos);
    if (written != pos) {
        ESP_LOGE(TAG, "Failed to write command 0x%04X", cmd);
        return ESP_FAIL;
    }

    if (wait_response) {
        s_waiting_response = true;
        s_expected_cmd = cmd;
        s_response_received = false;

        // Wait for response (timeout 1 second)
        uint32_t start = esp_timer_get_time() / 1000;
        while (!s_response_received && (esp_timer_get_time() / 1000 - start) < 1000) {
            ld2450_process();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        s_waiting_response = false;

        if (!s_response_received) {
            ESP_LOGW(TAG, "Command 0x%04X timeout", cmd);
            return ESP_ERR_TIMEOUT;
        }

        // Check response status
        if (s_response_len >= 3 && s_response_buffer[2] != 0) {
            ESP_LOGW(TAG, "Command 0x%04X failed", cmd);
            return ESP_FAIL;
        }
    }

    return ESP_OK;
}

/* ============================================================================
 * CONFIGURATION COMMANDS
 * ============================================================================ */

/**
 * @brief Enter configuration mode
 */
static esp_err_t enter_config_mode(void) {
    return send_command(LD2450_CMD_ENABLE_CONFIG, NULL, 0, true);
}

/**
 * @brief Exit configuration mode
 */
static esp_err_t exit_config_mode(void) {
    return send_command(LD2450_CMD_END_CONFIG, NULL, 0, true);
}

/**
 * @brief Set zone configuration
 */
esp_err_t ld2450_set_zone(uint8_t zone_num, int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    if (zone_num >= LD2450_MAX_ZONES) {
        ESP_LOGE(TAG, "Invalid zone number: %d", zone_num);
        return ESP_ERR_INVALID_ARG;
    }

    // Validate coordinates
    if (x1 < LD2450_X_MIN || x1 > LD2450_X_MAX ||
        x2 < LD2450_X_MIN || x2 > LD2450_X_MAX ||
        y1 < LD2450_Y_MIN || y1 > LD2450_Y_MAX ||
        y2 < LD2450_Y_MIN || y2 > LD2450_Y_MAX) {
        ESP_LOGE(TAG, "Zone coordinates out of range");
        return ESP_ERR_INVALID_ARG;
    }

    // Update local zone config
    s_state.zone_config.zones[zone_num].x1 = x1;
    s_state.zone_config.zones[zone_num].y1 = y1;
    s_state.zone_config.zones[zone_num].x2 = x2;
    s_state.zone_config.zones[zone_num].y2 = y2;
    s_state.zone_config.zones[zone_num].enabled = true;

    ESP_LOGI(TAG, "Zone %d set: (%d,%d) to (%d,%d)", zone_num, x1, y1, x2, y2);

    // Apply to sensor
    return ld2450_apply_zones();
}

/**
 * @brief Clear/disable a zone
 */
esp_err_t ld2450_clear_zone(uint8_t zone_num) {
    if (zone_num >= LD2450_MAX_ZONES) {
        return ESP_ERR_INVALID_ARG;
    }

    s_state.zone_config.zones[zone_num].enabled = false;
    s_state.zone_config.zones[zone_num].x1 = 0;
    s_state.zone_config.zones[zone_num].y1 = 0;
    s_state.zone_config.zones[zone_num].x2 = 0;
    s_state.zone_config.zones[zone_num].y2 = 0;
    s_state.zone_config.zones[zone_num].occupied = false;

    ESP_LOGI(TAG, "Zone %d cleared", zone_num);

    return ld2450_apply_zones();
}

/**
 * @brief Set zone operation type
 */
esp_err_t ld2450_set_zone_type(ld2450_zone_type_t type) {
    s_state.zone_config.type = type;
    ESP_LOGI(TAG, "Zone type set to: %s", ld2450_zone_type_to_string(type));

    // Re-evaluate occupancy with new zone type
    update_zone_occupancy();

    return ESP_OK;
}

/**
 * @brief Apply zone configuration to sensor
 *
 * Note: The LD2450 zone configuration protocol may vary by firmware version.
 * This implements a generic approach. Adjust based on actual protocol specs.
 */
esp_err_t ld2450_apply_zones(void) {
    esp_err_t ret;

    // Enter config mode
    ret = enter_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enter config mode");
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    // Send zone configuration for each zone
    // Protocol: zone commands with coordinates
    for (int i = 0; i < LD2450_MAX_ZONES; i++) {
        uint8_t params[10];
        int param_len = 0;

        params[param_len++] = i;  // Zone number

        if (s_state.zone_config.zones[i].enabled) {
            // Zone coordinates (little-endian)
            params[param_len++] = s_state.zone_config.zones[i].x1 & 0xFF;
            params[param_len++] = (s_state.zone_config.zones[i].x1 >> 8) & 0xFF;
            params[param_len++] = s_state.zone_config.zones[i].y1 & 0xFF;
            params[param_len++] = (s_state.zone_config.zones[i].y1 >> 8) & 0xFF;
            params[param_len++] = s_state.zone_config.zones[i].x2 & 0xFF;
            params[param_len++] = (s_state.zone_config.zones[i].x2 >> 8) & 0xFF;
            params[param_len++] = s_state.zone_config.zones[i].y2 & 0xFF;
            params[param_len++] = (s_state.zone_config.zones[i].y2 >> 8) & 0xFF;
            params[param_len++] = 0x01;  // Enable flag
        } else {
            // Disabled zone - all zeros
            for (int j = 0; j < 9; j++) params[param_len++] = 0x00;
        }

        ret = send_command(LD2450_CMD_SET_ZONE, params, param_len, false);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to send zone %d config", i);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }

    // Exit config mode
    ret = exit_config_mode();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to exit config mode");
        return ret;
    }

    ESP_LOGI(TAG, "Zone configuration applied");
    return ESP_OK;
}

/**
 * @brief Read firmware version
 */
esp_err_t ld2450_read_firmware_version(void) {
    esp_err_t ret = send_command(LD2450_CMD_READ_VERSION, NULL, 0, true);
    if (ret == ESP_OK && s_response_len >= 8) {
        s_state.firmware.type = s_response_buffer[3];
        s_state.firmware.major = s_response_buffer[4];
        s_state.firmware.minor = s_response_buffer[5];
        s_state.firmware.build = s_response_buffer[6] |
                                (s_response_buffer[7] << 8);
        s_state.firmware.valid = true;

        ESP_LOGI(TAG, "Firmware: Type=%02X V%d.%02d Build=%lu",
                 s_state.firmware.type,
                 s_state.firmware.major,
                 s_state.firmware.minor,
                 (unsigned long)s_state.firmware.build);
    }
    return ret;
}

/**
 * @brief Restart sensor
 */
esp_err_t ld2450_restart(void) {
    ESP_LOGI(TAG, "Restarting sensor");
    return send_command(LD2450_CMD_RESTART, NULL, 0, false);
}

/**
 * @brief Factory reset sensor
 */
esp_err_t ld2450_factory_reset(void) {
    ESP_LOGW(TAG, "Factory reset sensor");
    return send_command(LD2450_CMD_FACTORY_RESET, NULL, 0, true);
}

/* ============================================================================
 * INITIALIZATION
 * ============================================================================ */

/**
 * @brief Initialize LD2450 driver
 */
esp_err_t ld2450_init(void) {
    ESP_LOGI(TAG, "Initializing LD2450 driver");

    // Configure UART
    uart_config_t uart_config = {
        .baud_rate = LD2450_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(LD2450_UART_NUM, LD2450_UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LD2450_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(LD2450_UART_NUM, LD2450_UART_TX_PIN, LD2450_UART_RX_PIN,
                                  UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Initialize state
    memset(&s_state, 0, sizeof(s_state));
    s_state.zone_config.type = LD2450_ZONE_DISABLED;

    // Wait for sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(500));

    // Flush any garbage data in UART buffer from power-on
    uart_flush(LD2450_UART_NUM);
    ESP_LOGI(TAG, "UART buffer flushed");

    // Note: Firmware version will be read later by the processing task
    // Don't block initialization with command/response

    ESP_LOGI(TAG, "LD2450 initialized on UART%d (TX:%d RX:%d @ %d baud)",
             LD2450_UART_NUM, LD2450_UART_TX_PIN, LD2450_UART_RX_PIN, LD2450_UART_BAUD);

    return ESP_OK;
}

/**
 * @brief Deinitialize LD2450 driver
 */
void ld2450_deinit(void) {
    uart_driver_delete(LD2450_UART_NUM);
    ESP_LOGI(TAG, "LD2450 deinitialized");
}

/* ============================================================================
 * STATE ACCESS
 * ============================================================================ */

const ld2450_state_t* ld2450_get_state(void) {
    return &s_state;
}

bool ld2450_is_connected(void) {
    return s_state.connected;
}

uint32_t ld2450_get_bytes_received(void) {
    return s_total_bytes_received;
}

/* ============================================================================
 * CALLBACK REGISTRATION
 * ============================================================================ */

void ld2450_register_target_callback(ld2450_target_callback_t callback) {
    s_target_callback = callback;
}

void ld2450_register_zone_callback(ld2450_zone_callback_t callback) {
    s_zone_callback = callback;
}

void ld2450_register_state_callback(ld2450_state_callback_t callback) {
    s_state_callback = callback;
}

/* ============================================================================
 * LOGGING CONTROL
 * ============================================================================ */

void ld2450_set_verbose_logging(bool enable) {
    s_verbose_logging = enable;
    ESP_LOGI(TAG, "Verbose logging %s", enable ? "ENABLED" : "DISABLED");
}
