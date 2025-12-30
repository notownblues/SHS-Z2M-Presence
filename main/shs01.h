/*
 * SPDX-FileCopyrightText: 2021-2025
 * SPDX-License-Identifier: CC0-1.0
 *
 * SHS01 Presence Sensor Header
 * Based on SmartHomeScene firmware with distance and energy data
 */

#ifndef SHS01_H
#define SHS01_H

#include "esp_zigbee_core.h"
#include "driver/gpio.h"
#include "driver/uart.h"

/* ---------------- Zigbee device & endpoints ---------------- */
#define SHS_MAX_CHILDREN                10
#define SHS_INSTALLCODE_POLICY_ENABLE   false

/* Endpoints */
#define SHS_EP_LIGHT                    1   /* genOnOff Light + Config Cluster */
#define SHS_EP_OCC                      2   /* Occupancy Sensing + Distance (LD2410C) */

/* LD2450 endpoints (standard clusters only - no flooding) */
#define SHS_EP_LD2450_OCC               3   /* LD2450 Occupancy (msOccupancySensing) */
#define SHS_EP_LD2450_TARGET_COUNT      4   /* Target count 0-3 (genAnalogInput) */
#define SHS_EP_LD2450_ZONE1             5   /* Zone 1 occupancy (genBinaryInput) */
#define SHS_EP_LD2450_ZONE2             6   /* Zone 2 occupancy (genBinaryInput) */
#define SHS_EP_LD2450_ZONE3             7   /* Zone 3 occupancy (genBinaryInput) */
#define SHS_EP_LD2450_ZONE4             22  /* Zone 4 occupancy (genBinaryInput) */
#define SHS_EP_LD2450_ZONE5             23  /* Zone 5 occupancy (genBinaryInput) */

/* LD2450 position data endpoints (genAnalogInput) - OPTIONAL, only when config mode enabled */
#define SHS_EP_LD2450_T1_X              8   /* Target 1 X coordinate in mm (genAnalogInput) */
#define SHS_EP_LD2450_T1_Y              9   /* Target 1 Y coordinate in mm (genAnalogInput) */
#define SHS_EP_LD2450_T1_DIST           10  /* Target 1 Distance in mm (genAnalogInput) */
#define SHS_EP_LD2450_T2_X              11  /* Target 2 X coordinate in mm (genAnalogInput) */
#define SHS_EP_LD2450_T2_Y              12  /* Target 2 Y coordinate in mm (genAnalogInput) */
#define SHS_EP_LD2450_T2_DIST           13  /* Target 2 Distance in mm (genAnalogInput) */
#define SHS_EP_LD2450_T3_X              14  /* Target 3 X coordinate in mm (genAnalogInput) */
#define SHS_EP_LD2450_T3_Y              15  /* Target 3 Y coordinate in mm (genAnalogInput) */
#define SHS_EP_LD2450_T3_DIST           16  /* Target 3 Distance in mm (genAnalogInput) */

/* LD2410C target state endpoints (genBinaryInput) - more reliable than mfr-specific attrs */
#define SHS_EP_LD2410C_MOVING           17  /* Moving target state (genBinaryInput) */
#define SHS_EP_LD2410C_STATIC           18  /* Static target state (genBinaryInput) */

/* Zone target count endpoints (genAnalogInput) - same pattern as EP4 */
#define SHS_EP_ZONE1_TARGETS            19  /* Zone 1 target count (genAnalogInput) */
#define SHS_EP_ZONE2_TARGETS            20  /* Zone 2 target count (genAnalogInput) */
#define SHS_EP_ZONE3_TARGETS            21  /* Zone 3 target count (genAnalogInput) */
#define SHS_EP_ZONE4_TARGETS            24  /* Zone 4 target count (genAnalogInput) */
#define SHS_EP_ZONE5_TARGETS            25  /* Zone 5 target count (genAnalogInput) */

/* Router config - mains powered, always listening, no polling needed */
#define SHS_ZR_CONFIG()                                         \
    {                                                           \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,               \
        .install_code_policy = SHS_INSTALLCODE_POLICY_ENABLE,   \
        .nwk_cfg.zczr_cfg = {                                   \
            .max_children = 10,                                 \
        },                                                      \
    }

/* Channels */
#define SHS_PRIMARY_CHANNEL_MASK        ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK

/* Manufacturer / Model strings for Basic cluster (length-prefixed ASCII) */
#define SHS_MANUFACTURER_NAME           "\x0E""SmartHomeScene"
#define SHS_MODEL_IDENTIFIER            "\x10""SHS-Z2M-Presence"

/* Optional Basic metadata (length-prefixed ZCL char strings) */
#define SHS_BASIC_DATE_CODE             "\x0A""2025-12-12"
#define SHS_BASIC_SW_BUILD_ID           "\x0B""SHS01-2.0.0"

/* ============================================================================
 * CLUSTER IDS
 * ============================================================================ */

/* Standard Zigbee clusters */
#define SHS_CLUSTER_ANALOG_INPUT        0x000C  /* genAnalogInput - for target count */
#define SHS_CLUSTER_BINARY_INPUT        0x000F  /* genBinaryInput - for zone occupancy */
#define SHS_ATTR_PRESENT_VALUE          0x0055  /* presentValue attribute (analog) */
#define SHS_ATTR_PRESENT_VALUE_BINARY   0x0055  /* presentValue attribute (binary) */

/* Config cluster */
#define SHS_CL_CFG_ID                   0xFDCD

/* ============================================================================
 * CONFIGURATION CLUSTER ATTRIBUTES (0xFDCD on EP1)
 * ============================================================================ */

#define SHS_ATTR_MOVEMENT_COOLDOWN      0x0001  /* uint16, seconds (moving target cooldown) */
#define SHS_ATTR_OCC_CLEAR_COOLDOWN     0x0002  /* uint16, seconds (occupancy delay) */
#define SHS_ATTR_MOVING_SENS_0_10       0x0003  /* uint16, 0-10 scale */
#define SHS_ATTR_STATIC_SENS_0_10       0x0004  /* uint16, 0-10 scale */
#define SHS_ATTR_MOVING_MAX_GATE        0x0005  /* uint16, 0-8 (movement detection range) */
#define SHS_ATTR_STATIC_MAX_GATE        0x0006  /* uint16, 2-8 (static detection range) */
#define SHS_ATTR_POSITION_REPORTING     0x0008  /* bool, enable X/Y position reporting (config mode) */
#define SHS_ATTR_MIN_MOVING_ENERGY      0x0009  /* uint16, 0-100 (minimum energy for moving detection) */
#define SHS_ATTR_MIN_STATIC_ENERGY      0x000A  /* uint16, 0-100 (minimum energy for static detection) */

/* Zone configuration attributes (on Config cluster 0xFDCD) */
/* Zone type values: 0=off, 1=detection (include), 2=filter (exclude), 3=interference */
#define SHS_ATTR_ZONE_TYPE_CFG          0x0020  /* uint8: global zone mode (0=disabled, 1=detection, 2=filter) */
#define SHS_ATTR_ZONE1_ENABLED          0x0021  /* bool */
#define SHS_ATTR_ZONE1_X1_CFG           0x0022  /* int16, mm */
#define SHS_ATTR_ZONE1_Y1_CFG           0x0023  /* int16, mm */
#define SHS_ATTR_ZONE1_X2_CFG           0x0024  /* int16, mm */
#define SHS_ATTR_ZONE1_Y2_CFG           0x0025  /* int16, mm */
#define SHS_ATTR_ZONE1_TARGETS_CFG      0x0026  /* uint8, 0-3, read-only */
#define SHS_ATTR_ZONE1_TYPE_CFG         0x0027  /* uint8: per-zone type (0=off, 1=detection, 2=filter, 3=interference) */
#define SHS_ATTR_ZONE2_ENABLED          0x0030  /* bool */
#define SHS_ATTR_ZONE2_X1_CFG           0x0031  /* int16, mm */
#define SHS_ATTR_ZONE2_Y1_CFG           0x0032  /* int16, mm */
#define SHS_ATTR_ZONE2_X2_CFG           0x0033  /* int16, mm */
#define SHS_ATTR_ZONE2_Y2_CFG           0x0034  /* int16, mm */
#define SHS_ATTR_ZONE2_TARGETS_CFG      0x0035  /* uint8, 0-3, read-only */
#define SHS_ATTR_ZONE2_TYPE_CFG         0x0036  /* uint8: per-zone type (0=off, 1=detection, 2=filter, 3=interference) */
#define SHS_ATTR_ZONE3_ENABLED          0x0040  /* bool */
#define SHS_ATTR_ZONE3_X1_CFG           0x0041  /* int16, mm */
#define SHS_ATTR_ZONE3_Y1_CFG           0x0042  /* int16, mm */
#define SHS_ATTR_ZONE3_X2_CFG           0x0043  /* int16, mm */
#define SHS_ATTR_ZONE3_Y2_CFG           0x0044  /* int16, mm */
#define SHS_ATTR_ZONE3_TARGETS_CFG      0x0045  /* uint8, 0-3, read-only */
#define SHS_ATTR_ZONE3_TYPE_CFG         0x0046  /* uint8: per-zone type (0=off, 1=detection, 2=filter, 3=interference) */
#define SHS_ATTR_ZONE4_ENABLED          0x0050  /* bool */
#define SHS_ATTR_ZONE4_X1_CFG           0x0051  /* int16, mm */
#define SHS_ATTR_ZONE4_Y1_CFG           0x0052  /* int16, mm */
#define SHS_ATTR_ZONE4_X2_CFG           0x0053  /* int16, mm */
#define SHS_ATTR_ZONE4_Y2_CFG           0x0054  /* int16, mm */
#define SHS_ATTR_ZONE4_TARGETS_CFG      0x0055  /* uint8, 0-3, read-only */
#define SHS_ATTR_ZONE4_TYPE_CFG         0x0056  /* uint8: per-zone type (0=off, 1=detection, 2=filter, 3=interference) */
#define SHS_ATTR_ZONE5_ENABLED          0x0060  /* bool */
#define SHS_ATTR_ZONE5_X1_CFG           0x0061  /* int16, mm */
#define SHS_ATTR_ZONE5_Y1_CFG           0x0062  /* int16, mm */
#define SHS_ATTR_ZONE5_X2_CFG           0x0063  /* int16, mm */
#define SHS_ATTR_ZONE5_Y2_CFG           0x0064  /* int16, mm */
#define SHS_ATTR_ZONE5_TARGETS_CFG      0x0065  /* uint8, 0-3, read-only */
#define SHS_ATTR_ZONE5_TYPE_CFG         0x0066  /* uint8: per-zone type (0=off, 1=detection, 2=filter, 3=interference) */

/* ============================================================================
 * OCCUPANCY CLUSTER CUSTOM ATTRIBUTES (on EP2)
 * ============================================================================ */

#define SHS_ATTR_OCC_MOVING_TARGET      0xF001  /* bool, manufacturer-specific */
#define SHS_ATTR_OCC_STATIC_TARGET      0xF002  /* bool, manufacturer-specific */

/* Standard occupancy cluster attribute */
#define SHS_ZCL_ATTR_OCC_PIR_OU_DELAY   0x0010

/* ============================================================================
 * BOOT BUTTON & MISC
 * ============================================================================ */

#define SHS_BOOT_BUTTON_GPIO            GPIO_NUM_9

/* NVS debounce */
#define SHS_NVS_DEBOUNCE_MS             500
#define SHS_COOLDOWN_MAX_SEC            300

/* ============================================================================
 * LD2450 TARGET CLUSTER ATTRIBUTES (0xFDCF on EP3)
 * ============================================================================ */

/* Target 1 attributes */
#define SHS_ATTR_T1_X                   0x0001  /* int16, mm */
#define SHS_ATTR_T1_Y                   0x0002  /* int16, mm */
#define SHS_ATTR_T1_SPEED               0x0003  /* int16, mm/s */
#define SHS_ATTR_T1_DISTANCE            0x0004  /* uint16, mm */
#define SHS_ATTR_T1_ANGLE               0x0005  /* uint16, deg*10 */
#define SHS_ATTR_T1_ACTIVE              0x0006  /* bool */

/* Target 2 attributes */
#define SHS_ATTR_T2_X                   0x0011  /* int16, mm */
#define SHS_ATTR_T2_Y                   0x0012  /* int16, mm */
#define SHS_ATTR_T2_SPEED               0x0013  /* int16, mm/s */
#define SHS_ATTR_T2_DISTANCE            0x0014  /* uint16, mm */
#define SHS_ATTR_T2_ANGLE               0x0015  /* uint16, deg*10 */
#define SHS_ATTR_T2_ACTIVE              0x0016  /* bool */

/* Target 3 attributes */
#define SHS_ATTR_T3_X                   0x0021  /* int16, mm */
#define SHS_ATTR_T3_Y                   0x0022  /* int16, mm */
#define SHS_ATTR_T3_SPEED               0x0023  /* int16, mm/s */
#define SHS_ATTR_T3_DISTANCE            0x0024  /* uint16, mm */
#define SHS_ATTR_T3_ANGLE               0x0025  /* uint16, deg*10 */
#define SHS_ATTR_T3_ACTIVE              0x0026  /* bool */

/* ============================================================================
 * LD2450 ZONE CLUSTER ATTRIBUTES (0xFDD0 on EP3)
 * ============================================================================ */

/* Zone configuration */
#define SHS_ATTR_ZONE_TYPE              0x0001  /* uint8: 0=disabled, 1=detection, 2=filter */

/* Zone 1 */
#define SHS_ATTR_ZONE1_X1               0x0010  /* int16, mm */
#define SHS_ATTR_ZONE1_Y1               0x0011  /* int16, mm */
#define SHS_ATTR_ZONE1_X2               0x0012  /* int16, mm */
#define SHS_ATTR_ZONE1_Y2               0x0013  /* int16, mm */
#define SHS_ATTR_ZONE1_OCCUPIED         0x0014  /* bool */
#define SHS_ATTR_ZONE1_TARGET_COUNT     0x0015  /* uint8, 0-3 */

/* Zone 2 */
#define SHS_ATTR_ZONE2_X1               0x0020  /* int16, mm */
#define SHS_ATTR_ZONE2_Y1               0x0021  /* int16, mm */
#define SHS_ATTR_ZONE2_X2               0x0022  /* int16, mm */
#define SHS_ATTR_ZONE2_Y2               0x0023  /* int16, mm */
#define SHS_ATTR_ZONE2_OCCUPIED         0x0024  /* bool */
#define SHS_ATTR_ZONE2_TARGET_COUNT     0x0025  /* uint8, 0-3 */

/* Zone 3 */
#define SHS_ATTR_ZONE3_X1               0x0030  /* int16, mm */
#define SHS_ATTR_ZONE3_Y1               0x0031  /* int16, mm */
#define SHS_ATTR_ZONE3_X2               0x0032  /* int16, mm */
#define SHS_ATTR_ZONE3_Y2               0x0033  /* int16, mm */
#define SHS_ATTR_ZONE3_OCCUPIED         0x0034  /* bool */
#define SHS_ATTR_ZONE3_TARGET_COUNT     0x0035  /* uint8, 0-3 */

/* Overall occupancy (after zone filtering) */
#define SHS_ATTR_LD2450_OCCUPANCY       0x0040  /* bool */

/* Diagnostic counters */
#define SHS_ATTR_LD2450_CALLBACK_COUNT  0x0050  /* uint16, increments on each callback */
#define SHS_ATTR_LD2450_UART_COUNT      0x0051  /* uint16, increments on UART data */

#endif /* SHS01_H */
