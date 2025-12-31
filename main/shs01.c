/*
 * SPDX-FileCopyrightText: 2021-2025
 * SPDX-License-Identifier: CC0-1.0
 *
 * SHS01 Presence Sensor
 * Based on SmartHomeScene firmware with distance and energy data
 *
 * Features:
 * - Occupancy/Moving/Static target detection
 * - Distance measurements (moving, static, detection)
 * - Energy values (confidence 0-100)
 * - Configurable: cooldown, delay, sensitivity, detection range
 */

#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_timer.h"

#include "shs01.h"
#include "ld2410_enhanced.h"
#include "ld2450.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "zcl_utility.h"
#include "light_driver.h"

#include "esp_zigbee_attribute.h"
#include "esp_zigbee_cluster.h"

#if !defined CONFIG_ZB_ZCZR
#error "Enable Router: set CONFIG_ZB_ZCZR=y (menuconfig)"
#endif

static const char *SHS_TAG = "SHS01";

/* Current target data (updated by LD2450 callback) */
static ld2450_target_t current_targets[3] = {0};
static uint8_t current_target_count = 0;
static SemaphoreHandle_t target_data_mutex = NULL;

/* ============================================================================
 * NVS KEYS
 * ============================================================================ */

#define SHS_NVS_NAMESPACE       "cfg"
#define SHS_NVS_KEY_MV_CD       "mv_cd"
#define SHS_NVS_KEY_OCC_CD      "occ_cd"
#define SHS_NVS_KEY_MV_SENS     "mv_sens"
#define SHS_NVS_KEY_ST_SENS     "st_sens"
#define SHS_NVS_KEY_MV_GATE     "mv_gate"
#define SHS_NVS_KEY_ST_GATE     "st_gate"

/* LD2450 zone configuration keys */
#define SHS_NVS_KEY_ZONE_TYPE   "z_type"
#define SHS_NVS_KEY_Z1_EN       "z1_en"
#define SHS_NVS_KEY_Z1_X1       "z1_x1"
#define SHS_NVS_KEY_Z1_Y1       "z1_y1"
#define SHS_NVS_KEY_Z1_X2       "z1_x2"
#define SHS_NVS_KEY_Z1_Y2       "z1_y2"
#define SHS_NVS_KEY_Z1_TYPE     "z1_type"
#define SHS_NVS_KEY_Z2_EN       "z2_en"
#define SHS_NVS_KEY_Z2_X1       "z2_x1"
#define SHS_NVS_KEY_Z2_Y1       "z2_y1"
#define SHS_NVS_KEY_Z2_X2       "z2_x2"
#define SHS_NVS_KEY_Z2_Y2       "z2_y2"
#define SHS_NVS_KEY_Z2_TYPE     "z2_type"
#define SHS_NVS_KEY_Z3_EN       "z3_en"
#define SHS_NVS_KEY_Z3_X1       "z3_x1"
#define SHS_NVS_KEY_Z3_Y1       "z3_y1"
#define SHS_NVS_KEY_Z3_X2       "z3_x2"
#define SHS_NVS_KEY_Z3_Y2       "z3_y2"
#define SHS_NVS_KEY_Z3_TYPE     "z3_type"
#define SHS_NVS_KEY_Z4_EN       "z4_en"
#define SHS_NVS_KEY_Z4_X1       "z4_x1"
#define SHS_NVS_KEY_Z4_Y1       "z4_y1"
#define SHS_NVS_KEY_Z4_X2       "z4_x2"
#define SHS_NVS_KEY_Z4_Y2       "z4_y2"
#define SHS_NVS_KEY_Z4_TYPE     "z4_type"
#define SHS_NVS_KEY_Z5_EN       "z5_en"
#define SHS_NVS_KEY_Z5_X1       "z5_x1"
#define SHS_NVS_KEY_Z5_Y1       "z5_y1"
#define SHS_NVS_KEY_Z5_X2       "z5_x2"
#define SHS_NVS_KEY_Z5_Y2       "z5_y2"
#define SHS_NVS_KEY_Z5_TYPE     "z5_type"

/* ============================================================================
 * CONFIGURATION STORAGE
 * ============================================================================ */

/* Basic config (original) */
static uint16_t shs_movement_cooldown_sec = 0;
static uint16_t shs_occupancy_clear_sec   = 0;
static uint32_t shs_moving_cooldown_until = 0;  /* Cooldown timestamp for moving target */
static uint32_t shs_static_cooldown_until = 0;  /* Cooldown timestamp for static target */
static uint8_t  shs_moving_sens_0_100     = 60;
static uint8_t  shs_static_sens_0_100     = 50;
static uint16_t shs_moving_max_gate       = 8;
static uint16_t shs_static_max_gate       = 8;
static uint16_t shs_sens_mv_0_10          = 4;  /* Matches threshold 60: 10 - (60/10) = 4 */
static uint16_t shs_sens_st_0_10          = 5;  /* Matches threshold 50: 10 - (50/10) = 5 */

/* Position reporting mode - controls X/Y coordinate reporting for zone configuration */
static bool     shs_position_reporting    = false;

/* Energy thresholds for false positive filtering (DISABLED - see #if 0 block in callback) */
static uint16_t shs_min_moving_energy     = 40;   /* 0-100, NOT ACTIVE - kept for Zigbee attribute compatibility */
static uint16_t shs_min_static_energy     = 40;   /* 0-100, NOT ACTIVE - kept for Zigbee attribute compatibility */

/* Firmware version (read from LD2410) */
static char     shs_firmware_version[20]  = "Unknown";

/* ============================================================================
 * LIVE DATA (updated from LD2410)
 * ============================================================================ */

/* Occupancy states */
static bool shs_moving_state    = false;
static bool shs_static_state    = false;
static bool shs_occupancy_state = false;

/* Zigbee ready flag */
static volatile bool shs_zb_ready = false;

/* Network connectivity tracking for auto-recovery */
static volatile bool shs_zb_connected = false;
static volatile bool shs_zb_rejoin_pending = false;  /* Prevents multiple concurrent rejoin attempts */
static uint32_t shs_last_successful_tx = 0;
#define SHS_ZB_LOCK_TIMEOUT_MS     100    /* Timeout for Zigbee lock acquisition */
#define SHS_ZB_CONNECTIVITY_CHECK_MS  60000  /* Check connectivity every 60s */

/* Diagnostic counters for lock acquisition */
static uint32_t shs_lock_success_count = 0;
static uint32_t shs_lock_fail_count = 0;
static uint32_t shs_lock_consecutive_fails = 0;
static uint32_t shs_tx_success_count = 0;
static uint32_t shs_tx_fail_count = 0;

/* Helper macro: acquire Zigbee lock with timeout, returns on failure */
#define SHS_ZB_LOCK_ACQUIRE_OR_RETURN() \
    do { \
        if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_ZB_LOCK_TIMEOUT_MS)) != true) { \
            shs_lock_fail_count++; \
            shs_lock_consecutive_fails++; \
            if (shs_lock_consecutive_fails == 1 || shs_lock_consecutive_fails == 10 || \
                shs_lock_consecutive_fails == 50 || (shs_lock_consecutive_fails % 100) == 0) { \
                ESP_LOGW(SHS_TAG, "Zigbee lock timeout #%lu (consecutive: %lu)", \
                         (unsigned long)shs_lock_fail_count, (unsigned long)shs_lock_consecutive_fails); \
            } \
            return; \
        } \
        shs_lock_success_count++; \
        shs_lock_consecutive_fails = 0; \
    } while(0)

/* ============================================================================
 * LD2450 LIVE DATA - Standard clusters only (no flooding)
 * ============================================================================ */

/* LD2450 occupancy and target tracking */
static bool shs_ld2450_occupancy = false;      /* Overall presence (any target) */
static uint8_t shs_ld2450_target_count = 0;    /* 0-3 active targets */

/* Zone occupancy states */
static bool shs_zone1_occupied = false;
static bool shs_zone2_occupied = false;
static bool shs_zone3_occupied = false;
static bool shs_zone4_occupied = false;
static bool shs_zone5_occupied = false;

/* Zone target counts (how many targets in each zone) */
static uint8_t shs_zone1_targets = 0;
static uint8_t shs_zone2_targets = 0;
static uint8_t shs_zone3_targets = 0;
static uint8_t shs_zone4_targets = 0;
static uint8_t shs_zone5_targets = 0;

/* Zone configuration (received from Zigbee, applied to LD2450) */
/* Global zone type: 0=disabled, 1=detection, 2=filter */
static uint8_t shs_zone_type = 0;

/* Per-zone types: 0=off, 1=detection, 2=filter, 3=interference */
static uint8_t shs_zone1_type = 0;
static uint8_t shs_zone2_type = 0;
static uint8_t shs_zone3_type = 0;
static uint8_t shs_zone4_type = 0;
static uint8_t shs_zone5_type = 0;

/* Zone 1 */
static bool    shs_zone1_enabled = false;
static int16_t shs_zone1_x1 = -1500, shs_zone1_y1 = 0;
static int16_t shs_zone1_x2 = 1500, shs_zone1_y2 = 3000;
/* Zone 2 */
static bool    shs_zone2_enabled = false;
static int16_t shs_zone2_x1 = -1500, shs_zone2_y1 = 0;
static int16_t shs_zone2_x2 = 1500, shs_zone2_y2 = 3000;
/* Zone 3 */
static bool    shs_zone3_enabled = false;
static int16_t shs_zone3_x1 = -1500, shs_zone3_y1 = 0;
static int16_t shs_zone3_x2 = 1500, shs_zone3_y2 = 3000;
/* Zone 4 */
static bool    shs_zone4_enabled = false;
static int16_t shs_zone4_x1 = -1500, shs_zone4_y1 = 0;
static int16_t shs_zone4_x2 = 1500, shs_zone4_y2 = 3000;
/* Zone 5 */
static bool    shs_zone5_enabled = false;
static int16_t shs_zone5_x1 = -1500, shs_zone5_y1 = 0;
static int16_t shs_zone5_x2 = 1500, shs_zone5_y2 = 3000;

/* Zone config debounce - wait for all attributes to arrive before applying */
#define SHS_ZONE_CFG_DEBOUNCE_MS  500  /* Wait 500ms after last attribute before applying */
static uint32_t shs_zone_cfg_pending_until = 0;
static bool shs_zone_cfg_pending = false;
static bool shs_zone_cfg_save_needed = false;  /* Only save to NVS when config was changed via Zigbee */

/* LD2450 position smoothing and rate limiting */
#define POSITION_UPDATE_INTERVAL_MS  300   /* Minimum ms between Zigbee updates */
#define POSITION_CHANGE_THRESHOLD    30    /* Minimum mm change to trigger update */
/* DIAGNOSTIC: Smoothing disabled to test raw sensor data
 * Normal value: 0.3f (range: 0.1=very smooth, 0.9=responsive)
 * Value of 1.0f = no smoothing, raw sensor data passed through */
#define EMA_ALPHA                    1.0f  /* DIAGNOSTIC: Disabled for raw data testing */

static uint32_t last_position_update_ms = 0;
static float smoothed_x[3] = {0, 0, 0};
static float smoothed_y[3] = {0, 0, 0};
static int16_t last_reported_x[3] = {0, 0, 0};
static int16_t last_reported_y[3] = {0, 0, 0};
static uint16_t last_reported_dist[3] = {0, 0, 0};

/* ============================================================================
 * NVS SAVE WORKER
 * ============================================================================ */

typedef enum {
    SHS_SAVE_IMMEDIATE_U16,
    SHS_SAVE_DEBOUNCE_SENS_MOVE,
    SHS_SAVE_DEBOUNCE_SENS_STATIC,
    SHS_SAVE_DEBOUNCE_GATE_MOVE,
    SHS_SAVE_DEBOUNCE_GATE_STATIC,
} shs_save_evt_t;

typedef struct {
    shs_save_evt_t type;
    uint16_t       u16;
} shs_save_msg_t;

static QueueHandle_t shs_save_q;

/* ============================================================================
 * HELPER FUNCTIONS
 * ============================================================================ */

static inline bool shs_time_reached(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

static void shs_cfg_save_u16(const char *key, uint16_t v) {
    nvs_handle_t h;
    if (nvs_open(SHS_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u16(h, key, v);
    nvs_commit(h);
    nvs_close(h);
}

static void shs_cfg_save_u8(const char *key, uint8_t v) {
    nvs_handle_t h;
    if (nvs_open(SHS_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_u8(h, key, v);
    nvs_commit(h);
    nvs_close(h);
}

__attribute__((unused)) static void shs_cfg_save_i16(const char *key, int16_t v) {
    nvs_handle_t h;
    if (nvs_open(SHS_NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK) return;
    nvs_set_i16(h, key, v);
    nvs_commit(h);
    nvs_close(h);
}

static inline void shs_save_enqueue(shs_save_evt_t t, uint16_t v) {
    if (!shs_save_q) return;
    shs_save_msg_t m = {.type = t, .u16 = v};
    (void)xQueueSend(shs_save_q, &m, 0);
}

static void shs_cfg_sync_sens_proxies(void) {
    /* Invert: sensor threshold 0 → user sensitivity 10 (max)
     *         sensor threshold 100 → user sensitivity 0 (min) */
    uint8_t mv_thresh = (shs_moving_sens_0_100 > 100) ? 100 : shs_moving_sens_0_100;
    uint8_t st_thresh = (shs_static_sens_0_100 > 100) ? 100 : shs_static_sens_0_100;
    shs_sens_mv_0_10 = (uint16_t)(10 - (mv_thresh / 10));
    shs_sens_st_0_10 = (uint16_t)(10 - (st_thresh / 10));
}

static void shs_cfg_load_from_nvs(void) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(SHS_NVS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGI(SHS_TAG, "NVS open RO failed (%s), using defaults", esp_err_to_name(err));
        return;
    }

    uint16_t u16tmp; uint8_t u8tmp;

    if (nvs_get_u16(h, SHS_NVS_KEY_MV_CD, &u16tmp) == ESP_OK)
        shs_movement_cooldown_sec = (u16tmp > SHS_COOLDOWN_MAX_SEC) ? SHS_COOLDOWN_MAX_SEC : u16tmp;
    if (nvs_get_u16(h, SHS_NVS_KEY_OCC_CD, &u16tmp) == ESP_OK)
        shs_occupancy_clear_sec = u16tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_MV_SENS, &u8tmp) == ESP_OK)
        shs_moving_sens_0_100 = (u8tmp > 100) ? 100 : u8tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_ST_SENS, &u8tmp) == ESP_OK)
        shs_static_sens_0_100 = (u8tmp > 100) ? 100 : u8tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_MV_GATE, &u8tmp) == ESP_OK) {
        /* Minimum 1 to avoid the gate 0 false positive issue */
        if (u8tmp < 1) u8tmp = 8;  /* 0 is invalid, use default 8 */
        else if (u8tmp > 8) u8tmp = 8;
        shs_moving_max_gate = u8tmp;
    }
    if (nvs_get_u8(h, SHS_NVS_KEY_ST_GATE, &u8tmp) == ESP_OK) {
        if (u8tmp < 2) u8tmp = 2; else if (u8tmp > 8) u8tmp = 8;
        shs_static_max_gate = u8tmp;
    }

    nvs_close(h);
    shs_cfg_sync_sens_proxies();

    ESP_LOGI(SHS_TAG, "NVS loaded: mv_cd=%us, occ_cd=%us, mv_sens=%u, st_sens=%u, mv_gate=%u, st_gate=%u",
             (unsigned)shs_movement_cooldown_sec, (unsigned)shs_occupancy_clear_sec,
             (unsigned)shs_moving_sens_0_100, (unsigned)shs_static_sens_0_100,
             (unsigned)shs_moving_max_gate, (unsigned)shs_static_max_gate);
}

/* ============================================================================
 * LD2450 ZONE CONFIGURATION NVS
 * ============================================================================ */

/* Save zone config to NVS (debounced, called after zone apply) */
static void shs_zone_cfg_save_to_nvs(void) {
    nvs_handle_t h;
    if (nvs_open("shs_cfg", NVS_READWRITE, &h) != ESP_OK) return;

    nvs_set_u8(h, SHS_NVS_KEY_ZONE_TYPE, shs_zone_type);
    /* Zone 1 */
    nvs_set_u8(h, SHS_NVS_KEY_Z1_EN, shs_zone1_enabled ? 1 : 0);
    nvs_set_i16(h, SHS_NVS_KEY_Z1_X1, shs_zone1_x1);
    nvs_set_i16(h, SHS_NVS_KEY_Z1_Y1, shs_zone1_y1);
    nvs_set_i16(h, SHS_NVS_KEY_Z1_X2, shs_zone1_x2);
    nvs_set_i16(h, SHS_NVS_KEY_Z1_Y2, shs_zone1_y2);
    nvs_set_u8(h, SHS_NVS_KEY_Z1_TYPE, shs_zone1_type);
    /* Zone 2 */
    nvs_set_u8(h, SHS_NVS_KEY_Z2_EN, shs_zone2_enabled ? 1 : 0);
    nvs_set_i16(h, SHS_NVS_KEY_Z2_X1, shs_zone2_x1);
    nvs_set_i16(h, SHS_NVS_KEY_Z2_Y1, shs_zone2_y1);
    nvs_set_i16(h, SHS_NVS_KEY_Z2_X2, shs_zone2_x2);
    nvs_set_i16(h, SHS_NVS_KEY_Z2_Y2, shs_zone2_y2);
    nvs_set_u8(h, SHS_NVS_KEY_Z2_TYPE, shs_zone2_type);
    /* Zone 3 */
    nvs_set_u8(h, SHS_NVS_KEY_Z3_EN, shs_zone3_enabled ? 1 : 0);
    nvs_set_i16(h, SHS_NVS_KEY_Z3_X1, shs_zone3_x1);
    nvs_set_i16(h, SHS_NVS_KEY_Z3_Y1, shs_zone3_y1);
    nvs_set_i16(h, SHS_NVS_KEY_Z3_X2, shs_zone3_x2);
    nvs_set_i16(h, SHS_NVS_KEY_Z3_Y2, shs_zone3_y2);
    nvs_set_u8(h, SHS_NVS_KEY_Z3_TYPE, shs_zone3_type);
    /* Zone 4 */
    nvs_set_u8(h, SHS_NVS_KEY_Z4_EN, shs_zone4_enabled ? 1 : 0);
    nvs_set_i16(h, SHS_NVS_KEY_Z4_X1, shs_zone4_x1);
    nvs_set_i16(h, SHS_NVS_KEY_Z4_Y1, shs_zone4_y1);
    nvs_set_i16(h, SHS_NVS_KEY_Z4_X2, shs_zone4_x2);
    nvs_set_i16(h, SHS_NVS_KEY_Z4_Y2, shs_zone4_y2);
    nvs_set_u8(h, SHS_NVS_KEY_Z4_TYPE, shs_zone4_type);
    /* Zone 5 */
    nvs_set_u8(h, SHS_NVS_KEY_Z5_EN, shs_zone5_enabled ? 1 : 0);
    nvs_set_i16(h, SHS_NVS_KEY_Z5_X1, shs_zone5_x1);
    nvs_set_i16(h, SHS_NVS_KEY_Z5_Y1, shs_zone5_y1);
    nvs_set_i16(h, SHS_NVS_KEY_Z5_X2, shs_zone5_x2);
    nvs_set_i16(h, SHS_NVS_KEY_Z5_Y2, shs_zone5_y2);
    nvs_set_u8(h, SHS_NVS_KEY_Z5_TYPE, shs_zone5_type);

    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(SHS_TAG, "Zone config saved to NVS");
}

/* Load zone config from NVS on boot */
static void shs_zone_cfg_load_from_nvs(void) {
    nvs_handle_t h;
    if (nvs_open("shs_cfg", NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(SHS_TAG, "No zone config in NVS, using defaults");
        return;
    }

    uint8_t u8tmp;
    int16_t i16tmp;

    if (nvs_get_u8(h, SHS_NVS_KEY_ZONE_TYPE, &u8tmp) == ESP_OK)
        shs_zone_type = u8tmp;

    /* Zone 1 */
    if (nvs_get_u8(h, SHS_NVS_KEY_Z1_EN, &u8tmp) == ESP_OK)
        shs_zone1_enabled = (u8tmp != 0);
    if (nvs_get_i16(h, SHS_NVS_KEY_Z1_X1, &i16tmp) == ESP_OK)
        shs_zone1_x1 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z1_Y1, &i16tmp) == ESP_OK)
        shs_zone1_y1 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z1_X2, &i16tmp) == ESP_OK)
        shs_zone1_x2 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z1_Y2, &i16tmp) == ESP_OK)
        shs_zone1_y2 = i16tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_Z1_TYPE, &u8tmp) == ESP_OK)
        shs_zone1_type = u8tmp;

    /* Zone 2 */
    if (nvs_get_u8(h, SHS_NVS_KEY_Z2_EN, &u8tmp) == ESP_OK)
        shs_zone2_enabled = (u8tmp != 0);
    if (nvs_get_i16(h, SHS_NVS_KEY_Z2_X1, &i16tmp) == ESP_OK)
        shs_zone2_x1 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z2_Y1, &i16tmp) == ESP_OK)
        shs_zone2_y1 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z2_X2, &i16tmp) == ESP_OK)
        shs_zone2_x2 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z2_Y2, &i16tmp) == ESP_OK)
        shs_zone2_y2 = i16tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_Z2_TYPE, &u8tmp) == ESP_OK)
        shs_zone2_type = u8tmp;

    /* Zone 3 */
    if (nvs_get_u8(h, SHS_NVS_KEY_Z3_EN, &u8tmp) == ESP_OK)
        shs_zone3_enabled = (u8tmp != 0);
    if (nvs_get_i16(h, SHS_NVS_KEY_Z3_X1, &i16tmp) == ESP_OK)
        shs_zone3_x1 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z3_Y1, &i16tmp) == ESP_OK)
        shs_zone3_y1 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z3_X2, &i16tmp) == ESP_OK)
        shs_zone3_x2 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z3_Y2, &i16tmp) == ESP_OK)
        shs_zone3_y2 = i16tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_Z3_TYPE, &u8tmp) == ESP_OK)
        shs_zone3_type = u8tmp;

    /* Zone 4 */
    if (nvs_get_u8(h, SHS_NVS_KEY_Z4_EN, &u8tmp) == ESP_OK)
        shs_zone4_enabled = (u8tmp != 0);
    if (nvs_get_i16(h, SHS_NVS_KEY_Z4_X1, &i16tmp) == ESP_OK)
        shs_zone4_x1 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z4_Y1, &i16tmp) == ESP_OK)
        shs_zone4_y1 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z4_X2, &i16tmp) == ESP_OK)
        shs_zone4_x2 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z4_Y2, &i16tmp) == ESP_OK)
        shs_zone4_y2 = i16tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_Z4_TYPE, &u8tmp) == ESP_OK)
        shs_zone4_type = u8tmp;

    /* Zone 5 */
    if (nvs_get_u8(h, SHS_NVS_KEY_Z5_EN, &u8tmp) == ESP_OK)
        shs_zone5_enabled = (u8tmp != 0);
    if (nvs_get_i16(h, SHS_NVS_KEY_Z5_X1, &i16tmp) == ESP_OK)
        shs_zone5_x1 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z5_Y1, &i16tmp) == ESP_OK)
        shs_zone5_y1 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z5_X2, &i16tmp) == ESP_OK)
        shs_zone5_x2 = i16tmp;
    if (nvs_get_i16(h, SHS_NVS_KEY_Z5_Y2, &i16tmp) == ESP_OK)
        shs_zone5_y2 = i16tmp;
    if (nvs_get_u8(h, SHS_NVS_KEY_Z5_TYPE, &u8tmp) == ESP_OK)
        shs_zone5_type = u8tmp;

    nvs_close(h);

    ESP_LOGI(SHS_TAG, "Zone config loaded: type=%d, z1=%d, z2=%d, z3=%d, z4=%d, z5=%d",
             shs_zone_type, shs_zone1_enabled, shs_zone2_enabled, shs_zone3_enabled,
             shs_zone4_enabled, shs_zone5_enabled);
}

/* Forward declaration for zone config apply */
static void shs_zone_cfg_apply_to_sensor(void);

/**
 * @brief Schedule zone config to be applied after debounce period
 * Called when any zone attribute changes - delays actual application
 * until all attributes have arrived (prevents flooding LD2450)
 */
static void shs_zone_cfg_schedule_apply(void) {
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    shs_zone_cfg_pending_until = now_ms + SHS_ZONE_CFG_DEBOUNCE_MS;
    shs_zone_cfg_pending = true;
    shs_zone_cfg_save_needed = true;  /* Mark that config was changed via Zigbee */
    ESP_LOGD(SHS_TAG, "Zone config scheduled (debounce %dms)", SHS_ZONE_CFG_DEBOUNCE_MS);
}

/**
 * @brief Check if debounced zone config should be applied now
 * Call this periodically from main loop or task
 */
static void shs_zone_cfg_check_pending(void) {
    if (!shs_zone_cfg_pending) return;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
    if (now_ms >= shs_zone_cfg_pending_until) {
        shs_zone_cfg_pending = false;
        shs_zone_cfg_apply_to_sensor();
    }
}

static void shs_zone_cfg_apply_to_sensor(void) {
    ESP_LOGI(SHS_TAG, "Applying zone config to LD2450: type=%d, z1=%d, z2=%d, z3=%d, z4=%d, z5=%d",
             shs_zone_type, shs_zone1_enabled, shs_zone2_enabled, shs_zone3_enabled,
             shs_zone4_enabled, shs_zone5_enabled);

    /* Set global zone type (disabled/detection/filter) */
    ld2450_set_zone_type((ld2450_zone_type_t)shs_zone_type);

    /* Configure each zone */
    if (shs_zone1_enabled) {
        ld2450_set_zone(0, shs_zone1_x1, shs_zone1_y1, shs_zone1_x2, shs_zone1_y2);
        ESP_LOGI(SHS_TAG, "Zone 1: (%d,%d) to (%d,%d) type=%d",
                 shs_zone1_x1, shs_zone1_y1, shs_zone1_x2, shs_zone1_y2, shs_zone1_type);
    } else {
        ld2450_clear_zone(0);
    }

    if (shs_zone2_enabled) {
        ld2450_set_zone(1, shs_zone2_x1, shs_zone2_y1, shs_zone2_x2, shs_zone2_y2);
        ESP_LOGI(SHS_TAG, "Zone 2: (%d,%d) to (%d,%d) type=%d",
                 shs_zone2_x1, shs_zone2_y1, shs_zone2_x2, shs_zone2_y2, shs_zone2_type);
    } else {
        ld2450_clear_zone(1);
    }

    if (shs_zone3_enabled) {
        ld2450_set_zone(2, shs_zone3_x1, shs_zone3_y1, shs_zone3_x2, shs_zone3_y2);
        ESP_LOGI(SHS_TAG, "Zone 3: (%d,%d) to (%d,%d) type=%d",
                 shs_zone3_x1, shs_zone3_y1, shs_zone3_x2, shs_zone3_y2, shs_zone3_type);
    } else {
        ld2450_clear_zone(2);
    }

    if (shs_zone4_enabled) {
        ld2450_set_zone(3, shs_zone4_x1, shs_zone4_y1, shs_zone4_x2, shs_zone4_y2);
        ESP_LOGI(SHS_TAG, "Zone 4: (%d,%d) to (%d,%d) type=%d",
                 shs_zone4_x1, shs_zone4_y1, shs_zone4_x2, shs_zone4_y2, shs_zone4_type);
    } else {
        ld2450_clear_zone(3);
    }

    if (shs_zone5_enabled) {
        ld2450_set_zone(4, shs_zone5_x1, shs_zone5_y1, shs_zone5_x2, shs_zone5_y2);
        ESP_LOGI(SHS_TAG, "Zone 5: (%d,%d) to (%d,%d) type=%d",
                 shs_zone5_x1, shs_zone5_y1, shs_zone5_x2, shs_zone5_y2, shs_zone5_type);
    } else {
        ld2450_clear_zone(4);
    }

    /* Apply zones to sensor (sends command to LD2450) */
    ld2450_apply_zones();

    /* Save zone config to NVS only if changed via Zigbee (not on startup) */
    if (shs_zone_cfg_save_needed) {
        shs_zone_cfg_save_to_nvs();
        shs_zone_cfg_save_needed = false;
    }
}

/* Forward declarations */
static bool is_valid_target(const ld2450_target_t *target);
static void shs_zigbee_task(void *pvParameters);
static void shs_zb_set_binary_value(uint8_t endpoint, bool value);
static void shs_bdb_start_top_level_commissioning_cb(uint8_t mode_mask);

/* ============================================================================
 * ZIGBEE ATTRIBUTE HELPERS
 * ============================================================================ */

static void shs_zb_set_occ_bitmap(uint8_t endpoint, bool occupied) {
    if (!shs_zb_ready) return;
    uint8_t v = occupied ? 1 : 0;
    SHS_ZB_LOCK_ACQUIRE_OR_RETURN();
    esp_zb_zcl_set_attribute_val(endpoint,
                                 ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
                                 &v, false);
    esp_zb_lock_release();
}

static void shs_zb_set_bool_attr(uint8_t endpoint, uint16_t cluster, uint16_t attr_id, bool value) {
    if (!shs_zb_ready) return;
    bool v = value;
    SHS_ZB_LOCK_ACQUIRE_OR_RETURN();
    esp_zb_zcl_set_attribute_val(endpoint, cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr_id, &v, false);
    esp_zb_lock_release();
}

/* Explicit attribute report sender - sends report directly to coordinator */
static void shs_zb_report_attr(uint8_t endpoint, uint16_t cluster, uint16_t attr_id, bool is_mfr_specific) {
    if (!shs_zb_ready) return;

    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .src_endpoint = endpoint,
            .dst_endpoint = 1,  /* Coordinator endpoint */
            .dst_addr_u.addr_short = 0x0000,  /* Coordinator address */
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = cluster,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .dis_default_resp = 1,
        .manuf_specific = is_mfr_specific ? 1 : 0,
        .manuf_code = is_mfr_specific ? 0x115F : 0,  /* Xiaomi manufacturer code for mfr-specific */
        .attributeID = attr_id,
    };

    SHS_ZB_LOCK_ACQUIRE_OR_RETURN();
    esp_zb_zcl_report_attr_cmd_req(&cmd);
    shs_last_successful_tx = (uint32_t)(esp_timer_get_time() / 1000);
    shs_tx_success_count++;
    esp_zb_lock_release();
}

/* Generic Zigbee attribute setters for signed int16 values (for position reporting) */
__attribute__((unused)) static void shs_zb_set_i16_attr(uint8_t endpoint, uint16_t cluster, uint16_t attr_id, int16_t value) {
    if (!shs_zb_ready) return;
    SHS_ZB_LOCK_ACQUIRE_OR_RETURN();
    esp_zb_zcl_set_attribute_val(endpoint, cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, attr_id, &value, true);
    esp_zb_lock_release();
}

static void shs_zb_set_ou_delay_ep2(uint16_t seconds) {
    if (!shs_zb_ready) return;
    SHS_ZB_LOCK_ACQUIRE_OR_RETURN();
    esp_zb_zcl_set_attribute_val(SHS_EP_OCC,
                                 ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 SHS_ZCL_ATTR_OCC_PIR_OU_DELAY,
                                 &seconds, false);
    esp_zb_lock_release();
}

/* ============================================================================
 * LD2410 CALLBACKS - Update Zigbee attributes on sensor changes
 * ============================================================================ */

static void shs_on_state_change(const ld2410_state_t *state) {
    /* Extract RAW states directly from sensor */
    bool raw_moving = (state->target.target_state & 0x01) != 0;
    bool raw_static = (state->target.target_state & 0x02) != 0;

    /* DISABLED: Gate 0 and energy filters - relying on LD2450 cross-validation instead.
     * Uncomment if false positives occur when LD2450 sees targets.
     */
#if 0
    /* Gate 0 hard block:
     * Always ignore detections at gate 0 (< 75cm) - these are PCB/housing reflections.
     * The LD2410's hardware gate 0 sensitivity setting is unreliable on some firmware.
     */
    if (raw_moving && state->target.moving_distance < 75) {
        raw_moving = false;
    }
    if (raw_static && state->target.static_distance < 75) {
        raw_static = false;
    }

    /* Minimum energy filter:
     * Filter out low-energy ghost detections at ANY distance.
     * This catches remaining interference when LD2450 also sees targets.
     * Default threshold is 40 (background noise is typically 13-25).
     */
    if (raw_moving && state->target.moving_energy < shs_min_moving_energy) {
        raw_moving = false;
    }
    if (raw_static && state->target.static_energy < shs_min_static_energy) {
        raw_static = false;
    }
#endif

    /* LD2450 cross-validation:
     * If LD2450 sees 0 targets, ignore LD2410 detections - they're likely interference.
     * The LD2450 is more accurate and doesn't suffer from the same noise issues.
     * This effectively uses LD2450 as a "sanity check" for LD2410.
     */
    if (shs_ld2450_target_count == 0) {
        raw_moving = false;
        raw_static = false;
    }

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    /* MOVING TARGET with optional cooldown */
    bool report_moving;
    if (shs_movement_cooldown_sec == 0) {
        report_moving = raw_moving;
    } else {
        if (raw_moving) {
            report_moving = true;
            shs_moving_cooldown_until = now_ms + (shs_movement_cooldown_sec * 1000);
        } else if (now_ms < shs_moving_cooldown_until) {
            report_moving = true;
        } else {
            report_moving = false;
        }
    }

    if (report_moving != shs_moving_state) {
        shs_moving_state = report_moving;
        if (shs_moving_state) {
            /* Log the values that passed the filter for debugging false positives */
            ESP_LOGI(SHS_TAG, "Moving Target -> DETECTED (dist=%dcm, energy=%d)",
                     state->target.moving_distance, state->target.moving_energy);
        } else {
            ESP_LOGI(SHS_TAG, "Moving Target -> CLEAR");
        }
        shs_zb_set_bool_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                            SHS_ATTR_OCC_MOVING_TARGET, shs_moving_state);
    }

    /* STATIC TARGET with cooldown (uses occupancy_clear_sec) */
    bool report_static;
    if (shs_occupancy_clear_sec == 0) {
        report_static = raw_static;
    } else {
        if (raw_static) {
            report_static = true;
            shs_static_cooldown_until = now_ms + (shs_occupancy_clear_sec * 1000);
        } else if (now_ms < shs_static_cooldown_until) {
            report_static = true;
        } else {
            report_static = false;
        }
    }

    if (report_static != shs_static_state) {
        shs_static_state = report_static;
        if (shs_static_state) {
            /* Log the values that passed the filter for debugging false positives */
            ESP_LOGI(SHS_TAG, "Static Target -> DETECTED (dist=%dcm, energy=%d)",
                     state->target.static_distance, state->target.static_energy);
        } else {
            ESP_LOGI(SHS_TAG, "Static Target -> CLEAR");
        }
        shs_zb_set_bool_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                            SHS_ATTR_OCC_STATIC_TARGET, shs_static_state);
    }

    /* OCCUPANCY = presence with cooldowns applied */
    bool presence = report_moving || report_static;

    if (presence != shs_occupancy_state) {
        shs_occupancy_state = presence;
        ESP_LOGI(SHS_TAG, "Occupancy -> %s", presence ? "DETECTED" : "CLEAR");
        shs_zb_set_occ_bitmap(SHS_EP_OCC, presence);
    }
}

/* ============================================================================
 * HELPER: Set attributes for standard clusters
 * ============================================================================ */

/* Set analog input presentValue (float) - for target count */
static void shs_zb_set_analog_value(uint8_t endpoint, float value) {
    if (!shs_zb_ready) return;

    SHS_ZB_LOCK_ACQUIRE_OR_RETURN();
    esp_zb_zcl_set_attribute_val(
        endpoint,
        SHS_CLUSTER_ANALOG_INPUT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        SHS_ATTR_PRESENT_VALUE,
        &value,
        false  /* Don't rely on automatic report */
    );
    esp_zb_lock_release();
}

/* Explicit analog attribute report sender */
static void shs_zb_report_analog_attr(uint8_t endpoint) {
    if (!shs_zb_ready) return;

    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .src_endpoint = endpoint,
            .dst_endpoint = 1,
            .dst_addr_u.addr_short = 0x0000,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = SHS_CLUSTER_ANALOG_INPUT,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .dis_default_resp = 1,
        .manuf_specific = 0,
        .manuf_code = 0,
        .attributeID = SHS_ATTR_PRESENT_VALUE,
    };

    SHS_ZB_LOCK_ACQUIRE_OR_RETURN();
    esp_zb_zcl_report_attr_cmd_req(&cmd);
    shs_last_successful_tx = (uint32_t)(esp_timer_get_time() / 1000);
    shs_tx_success_count++;
    esp_zb_lock_release();
}

/* Set binary input presentValue (bool) - for zone occupancy */
static void shs_zb_set_binary_value(uint8_t endpoint, bool value) {
    if (!shs_zb_ready) return;

    uint8_t val = value ? 1 : 0;
    SHS_ZB_LOCK_ACQUIRE_OR_RETURN();
    esp_zb_zcl_set_attribute_val(
        endpoint,
        SHS_CLUSTER_BINARY_INPUT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        SHS_ATTR_PRESENT_VALUE_BINARY,
        &val,
        false  /* Don't rely on auto-report, use explicit report instead */
    );
    esp_zb_lock_release();
}

/* Explicit binary attribute report sender */
static void shs_zb_report_binary_attr(uint8_t endpoint) {
    if (!shs_zb_ready) return;

    esp_zb_zcl_report_attr_cmd_t cmd = {
        .zcl_basic_cmd = {
            .src_endpoint = endpoint,
            .dst_endpoint = 1,
            .dst_addr_u.addr_short = 0x0000,
        },
        .address_mode = ESP_ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
        .clusterID = SHS_CLUSTER_BINARY_INPUT,
        .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI,
        .dis_default_resp = 1,
        .manuf_specific = 0,
        .manuf_code = 0,
        .attributeID = SHS_ATTR_PRESENT_VALUE_BINARY,
    };

    SHS_ZB_LOCK_ACQUIRE_OR_RETURN();
    esp_zb_zcl_report_attr_cmd_req(&cmd);
    shs_last_successful_tx = (uint32_t)(esp_timer_get_time() / 1000);
    shs_tx_success_count++;
    esp_zb_lock_release();
}

/* No longer needed - using shs_zb_set_occ_bitmap instead which has proper locking */

/* ============================================================================
 * LD2450 CALLBACKS - Update occupancy and zones only (no position flooding)
 * ============================================================================ */

/* Validate target coordinates - filter garbage data from sensor */
static bool is_valid_target(const ld2450_target_t *target) {
    if (!target->active) return false;

    /* Check X range: -3000 to +3000 mm */
    if (target->x < -3000 || target->x > 3000) return false;

    /* Check Y range: 0 to 6000 mm */
    if (target->y < 0 || target->y > 6000) return false;

    /* Check distance is reasonable (< 6m) */
    if (target->distance > 6000) return false;

    return true;
}

/* Check if a point is within a rectangular zone */
static bool shs_point_in_zone(int16_t x, int16_t y, int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    int16_t min_x = (x1 < x2) ? x1 : x2;
    int16_t max_x = (x1 > x2) ? x1 : x2;
    int16_t min_y = (y1 < y2) ? y1 : y2;
    int16_t max_y = (y1 > y2) ? y1 : y2;
    return (x >= min_x && x <= max_x && y >= min_y && y <= max_y);
}

/* Check if a target is in any enabled interference zone */
static bool shs_target_in_interference_zone(int16_t x, int16_t y) {
    /* Check zone 1 */
    if (shs_zone1_enabled && shs_zone1_type == LD2450_ZONE_INTERFERENCE) {
        if (shs_point_in_zone(x, y, shs_zone1_x1, shs_zone1_y1, shs_zone1_x2, shs_zone1_y2)) {
            return true;
        }
    }
    /* Check zone 2 */
    if (shs_zone2_enabled && shs_zone2_type == LD2450_ZONE_INTERFERENCE) {
        if (shs_point_in_zone(x, y, shs_zone2_x1, shs_zone2_y1, shs_zone2_x2, shs_zone2_y2)) {
            return true;
        }
    }
    /* Check zone 3 */
    if (shs_zone3_enabled && shs_zone3_type == LD2450_ZONE_INTERFERENCE) {
        if (shs_point_in_zone(x, y, shs_zone3_x1, shs_zone3_y1, shs_zone3_x2, shs_zone3_y2)) {
            return true;
        }
    }
    /* Check zone 4 */
    if (shs_zone4_enabled && shs_zone4_type == LD2450_ZONE_INTERFERENCE) {
        if (shs_point_in_zone(x, y, shs_zone4_x1, shs_zone4_y1, shs_zone4_x2, shs_zone4_y2)) {
            return true;
        }
    }
    /* Check zone 5 */
    if (shs_zone5_enabled && shs_zone5_type == LD2450_ZONE_INTERFERENCE) {
        if (shs_point_in_zone(x, y, shs_zone5_x1, shs_zone5_y1, shs_zone5_x2, shs_zone5_y2)) {
            return true;
        }
    }
    return false;
}

static void shs_on_ld2450_target_update(const ld2450_target_t *targets, uint8_t active_count) {
    /* Calculate effective count excluding targets in interference zones */
    uint8_t effective_count = 0;
    for (int i = 0; i < 3; i++) {
        if (is_valid_target(&targets[i]) && targets[i].active) {
            /* Check if target is in any interference zone */
            if (!shs_target_in_interference_zone(targets[i].x, targets[i].y)) {
                effective_count++;
            }
        }
    }

    /* Update target count using effective count (excludes interference zones) */
    if (shs_ld2450_target_count != effective_count) {
        shs_ld2450_target_count = effective_count;
        shs_zb_set_analog_value(SHS_EP_LD2450_TARGET_COUNT, (float)effective_count);
        shs_zb_report_analog_attr(SHS_EP_LD2450_TARGET_COUNT);  /* Explicit report */
        ESP_LOGI(SHS_TAG, "LD2450 target count: %d (raw: %d, filtered: %d in interference)",
                 effective_count, active_count, active_count - effective_count);
    }

    /* Update overall occupancy using effective count (excludes interference zones) */
    bool new_occupancy = (effective_count > 0);
    if (shs_ld2450_occupancy != new_occupancy) {
        shs_ld2450_occupancy = new_occupancy;
        shs_zb_set_occ_bitmap(SHS_EP_LD2450_OCC, new_occupancy);
        ESP_LOGI(SHS_TAG, "LD2450 occupancy: %s", new_occupancy ? "OCCUPIED" : "CLEAR");
    }

    /* Store current target data for HTTP polling (thread-safe) - filter garbage data */
    if (xSemaphoreTake(target_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        uint8_t valid_count = 0;
        for (int i = 0; i < 3; i++) {
            if (is_valid_target(&targets[i])) {
                current_targets[valid_count++] = targets[i];
            }
        }
        /* Clear remaining slots */
        for (int i = valid_count; i < 3; i++) {
            memset(&current_targets[i], 0, sizeof(ld2450_target_t));
        }
        current_target_count = valid_count;
        xSemaphoreGive(target_data_mutex);
    }

    /* Send position data to Zigbee endpoints ONLY when position reporting is enabled */
    if (shs_position_reporting) {
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        /* Apply EMA smoothing to all active targets */
        for (int i = 0; i < 3; i++) {
            if (is_valid_target(&targets[i]) && targets[i].active) {
                /* Raw sensor coordinates (no modifications) */
                float raw_x = (float)targets[i].x;
                float raw_y = (float)targets[i].y;

                /* EMA smoothing: smoothed = alpha * new + (1-alpha) * smoothed */
                if (smoothed_x[i] == 0 && smoothed_y[i] == 0) {
                    /* First reading - initialize */
                    smoothed_x[i] = raw_x;
                    smoothed_y[i] = raw_y;
                } else {
                    smoothed_x[i] = EMA_ALPHA * raw_x + (1.0f - EMA_ALPHA) * smoothed_x[i];
                    smoothed_y[i] = EMA_ALPHA * raw_y + (1.0f - EMA_ALPHA) * smoothed_y[i];
                }
            } else {
                /* Target inactive - reset smoothing */
                smoothed_x[i] = 0;
                smoothed_y[i] = 0;
            }
        }

        /* Rate limiting - only send updates every POSITION_UPDATE_INTERVAL_MS */
        if ((now_ms - last_position_update_ms) >= POSITION_UPDATE_INTERVAL_MS) {
            last_position_update_ms = now_ms;

            for (int i = 0; i < 3; i++) {
                uint8_t ep_base = SHS_EP_LD2450_T1_X + (i * 3);  /* EP8, EP11, EP14 */

                if (is_valid_target(&targets[i]) && targets[i].active) {
                    int16_t new_x = (int16_t)smoothed_x[i];
                    int16_t new_y = (int16_t)smoothed_y[i];
                    uint16_t new_dist = targets[i].distance;

                    /* Only send if changed by more than threshold */
                    int16_t dx = new_x - last_reported_x[i];
                    int16_t dy = new_y - last_reported_y[i];
                    int16_t dd = (int16_t)new_dist - (int16_t)last_reported_dist[i];

                    if (dx < 0) dx = -dx;
                    if (dy < 0) dy = -dy;
                    if (dd < 0) dd = -dd;

                    if (dx >= POSITION_CHANGE_THRESHOLD || dy >= POSITION_CHANGE_THRESHOLD || dd >= POSITION_CHANGE_THRESHOLD) {
                        /* WORKAROUND: Add 3000mm bias to X coordinate to avoid negative float
                         * transmission issues in Zigbee stack. Z2M converter subtracts 3000.
                         * Original range: -3000 to +3000 → Biased range: 0 to 6000 */
                        float x_biased = (float)(new_x + 3000);
                        ESP_LOGI(SHS_TAG, "ZB T%d: X=%d (biased=%d) Y=%d D=%d",
                                 i+1, new_x, (int)x_biased, new_y, new_dist);

                        /* Send biased X coordinate (Z2M converter will subtract 3000) */
                        shs_zb_set_analog_value(ep_base, x_biased);
                        shs_zb_report_analog_attr(ep_base);

                        /* Send smoothed Y coordinate */
                        shs_zb_set_analog_value(ep_base + 1, (float)new_y);
                        shs_zb_report_analog_attr(ep_base + 1);

                        /* Send Distance */
                        shs_zb_set_analog_value(ep_base + 2, (float)new_dist);
                        shs_zb_report_analog_attr(ep_base + 2);

                        last_reported_x[i] = new_x;
                        last_reported_y[i] = new_y;
                        last_reported_dist[i] = new_dist;
                    }
                } else if (last_reported_x[i] != 0 || last_reported_y[i] != 0 || last_reported_dist[i] != 0) {
                    /* Target became inactive - send zeros to clear
                     * Apply same 3000 bias to X so Z2M converter gets 0 after subtracting 3000 */
                    shs_zb_set_analog_value(ep_base, 3000.0f);  /* 3000 - 3000 = 0 in Z2M */
                    shs_zb_set_analog_value(ep_base + 1, 0.0f);
                    shs_zb_set_analog_value(ep_base + 2, 0.0f);
                    shs_zb_report_analog_attr(ep_base);
                    shs_zb_report_analog_attr(ep_base + 1);
                    shs_zb_report_analog_attr(ep_base + 2);

                    last_reported_x[i] = 0;
                    last_reported_y[i] = 0;
                    last_reported_dist[i] = 0;
                }
            }
        }
    }
}

/**
 * @brief Zone occupancy callback
 * Updates zone occupancy binary inputs and target counts when zones change state
 * Both binary and analog use explicit reports for reliability
 */
static void shs_on_ld2450_zone_update(const ld2450_zone_t *zones, bool occupancy) {
    /* Update zone 1 occupancy */
    if (zones[0].enabled && shs_zone1_occupied != zones[0].occupied) {
        shs_zone1_occupied = zones[0].occupied;
        shs_zb_set_binary_value(SHS_EP_LD2450_ZONE1, zones[0].occupied);
        shs_zb_report_binary_attr(SHS_EP_LD2450_ZONE1);
        ESP_LOGI(SHS_TAG, "Zone 1: %s", zones[0].occupied ? "OCCUPIED" : "CLEAR");
    }

    /* Update zone 1 target count */
    if (zones[0].enabled && shs_zone1_targets != zones[0].target_count) {
        shs_zone1_targets = zones[0].target_count;
        shs_zb_set_analog_value(SHS_EP_ZONE1_TARGETS, (float)shs_zone1_targets);
        shs_zb_report_analog_attr(SHS_EP_ZONE1_TARGETS);
        ESP_LOGI(SHS_TAG, "Zone 1 targets: %d", shs_zone1_targets);
    }

    /* Update zone 2 occupancy */
    if (zones[1].enabled && shs_zone2_occupied != zones[1].occupied) {
        shs_zone2_occupied = zones[1].occupied;
        shs_zb_set_binary_value(SHS_EP_LD2450_ZONE2, zones[1].occupied);
        shs_zb_report_binary_attr(SHS_EP_LD2450_ZONE2);
        ESP_LOGI(SHS_TAG, "Zone 2: %s", zones[1].occupied ? "OCCUPIED" : "CLEAR");
    }

    /* Update zone 2 target count */
    if (zones[1].enabled && shs_zone2_targets != zones[1].target_count) {
        shs_zone2_targets = zones[1].target_count;
        shs_zb_set_analog_value(SHS_EP_ZONE2_TARGETS, (float)shs_zone2_targets);
        shs_zb_report_analog_attr(SHS_EP_ZONE2_TARGETS);
        ESP_LOGI(SHS_TAG, "Zone 2 targets: %d", shs_zone2_targets);
    }

    /* Update zone 3 occupancy */
    if (zones[2].enabled && shs_zone3_occupied != zones[2].occupied) {
        shs_zone3_occupied = zones[2].occupied;
        shs_zb_set_binary_value(SHS_EP_LD2450_ZONE3, zones[2].occupied);
        shs_zb_report_binary_attr(SHS_EP_LD2450_ZONE3);
        ESP_LOGI(SHS_TAG, "Zone 3: %s", zones[2].occupied ? "OCCUPIED" : "CLEAR");
    }

    /* Update zone 3 target count */
    if (zones[2].enabled && shs_zone3_targets != zones[2].target_count) {
        shs_zone3_targets = zones[2].target_count;
        shs_zb_set_analog_value(SHS_EP_ZONE3_TARGETS, (float)shs_zone3_targets);
        shs_zb_report_analog_attr(SHS_EP_ZONE3_TARGETS);
        ESP_LOGI(SHS_TAG, "Zone 3 targets: %d", shs_zone3_targets);
    }

    /* Update zone 4 occupancy */
    if (zones[3].enabled && shs_zone4_occupied != zones[3].occupied) {
        shs_zone4_occupied = zones[3].occupied;
        shs_zb_set_binary_value(SHS_EP_LD2450_ZONE4, zones[3].occupied);
        shs_zb_report_binary_attr(SHS_EP_LD2450_ZONE4);
        ESP_LOGI(SHS_TAG, "Zone 4: %s", zones[3].occupied ? "OCCUPIED" : "CLEAR");
    }

    /* Update zone 4 target count */
    if (zones[3].enabled && shs_zone4_targets != zones[3].target_count) {
        shs_zone4_targets = zones[3].target_count;
        shs_zb_set_analog_value(SHS_EP_ZONE4_TARGETS, (float)shs_zone4_targets);
        shs_zb_report_analog_attr(SHS_EP_ZONE4_TARGETS);
        ESP_LOGI(SHS_TAG, "Zone 4 targets: %d", shs_zone4_targets);
    }

    /* Update zone 5 occupancy */
    if (zones[4].enabled && shs_zone5_occupied != zones[4].occupied) {
        shs_zone5_occupied = zones[4].occupied;
        shs_zb_set_binary_value(SHS_EP_LD2450_ZONE5, zones[4].occupied);
        shs_zb_report_binary_attr(SHS_EP_LD2450_ZONE5);
        ESP_LOGI(SHS_TAG, "Zone 5: %s", zones[4].occupied ? "OCCUPIED" : "CLEAR");
    }

    /* Update zone 5 target count */
    if (zones[4].enabled && shs_zone5_targets != zones[4].target_count) {
        shs_zone5_targets = zones[4].target_count;
        shs_zb_set_analog_value(SHS_EP_ZONE5_TARGETS, (float)shs_zone5_targets);
        shs_zb_report_analog_attr(SHS_EP_ZONE5_TARGETS);
        ESP_LOGI(SHS_TAG, "Zone 5 targets: %d", shs_zone5_targets);
    }
}

/**
 * @brief Helper macro for force update functions - acquire lock with 500ms timeout
 * Uses longer timeout for initial updates since they only run at boot/rejoin
 */
#define SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS  500

/**
 * @brief Force update all LD2450 attributes to Zigbee
 * Called once when Zigbee becomes ready.
 * Split into individual lock/release pairs to prevent blocking Zigbee task.
 */
static void shs_ld2450_force_update(void) {
    ESP_LOGI(SHS_TAG, "Forcing LD2450 state update to Zigbee: occ=%d count=%d z1=%d z2=%d z3=%d",
             shs_ld2450_occupancy, shs_ld2450_target_count,
             shs_zone1_occupied, shs_zone2_occupied, shs_zone3_occupied);

    /* Give Z2M a moment to finish pairing interview */
    vTaskDelay(pdMS_TO_TICKS(1000));

    /* EP3: LD2450 occupancy */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        uint8_t occ_val = shs_ld2450_occupancy ? 1 : 0;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_LD2450_OCC,
            ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
            &occ_val,
            true  /* report = true to trigger attribute report */
        );
        esp_zb_lock_release();
    }
    vTaskDelay(pdMS_TO_TICKS(10));  /* Brief yield to Zigbee stack */

    /* EP4: Target count */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        float count_val = (float)shs_ld2450_target_count;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_LD2450_TARGET_COUNT,
            SHS_CLUSTER_ANALOG_INPUT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_PRESENT_VALUE,
            &count_val,
            true  /* report = true */
        );
        esp_zb_lock_release();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* EP5: Zone 1 */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        uint8_t z1_val = shs_zone1_occupied ? 1 : 0;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_LD2450_ZONE1,
            SHS_CLUSTER_BINARY_INPUT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_PRESENT_VALUE_BINARY,
            &z1_val,
            true  /* report = true */
        );
        esp_zb_lock_release();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* EP6: Zone 2 */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        uint8_t z2_val = shs_zone2_occupied ? 1 : 0;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_LD2450_ZONE2,
            SHS_CLUSTER_BINARY_INPUT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_PRESENT_VALUE_BINARY,
            &z2_val,
            true  /* report = true */
        );
        esp_zb_lock_release();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* EP7: Zone 3 */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        uint8_t z3_val = shs_zone3_occupied ? 1 : 0;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_LD2450_ZONE3,
            SHS_CLUSTER_BINARY_INPUT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_PRESENT_VALUE_BINARY,
            &z3_val,
            true  /* report = true */
        );
        esp_zb_lock_release();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* EP19: Zone 1 target count */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        float z1_targets_val = (float)shs_zone1_targets;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_ZONE1_TARGETS,
            SHS_CLUSTER_ANALOG_INPUT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_PRESENT_VALUE,
            &z1_targets_val,
            true  /* report = true */
        );
        esp_zb_lock_release();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* EP20: Zone 2 target count */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        float z2_targets_val = (float)shs_zone2_targets;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_ZONE2_TARGETS,
            SHS_CLUSTER_ANALOG_INPUT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_PRESENT_VALUE,
            &z2_targets_val,
            true  /* report = true */
        );
        esp_zb_lock_release();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* EP21: Zone 3 target count */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        float z3_targets_val = (float)shs_zone3_targets;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_ZONE3_TARGETS,
            SHS_CLUSTER_ANALOG_INPUT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_PRESENT_VALUE,
            &z3_targets_val,
            true  /* report = true */
        );
        esp_zb_lock_release();
    }

    shs_last_successful_tx = (uint32_t)(esp_timer_get_time() / 1000);
    ESP_LOGI(SHS_TAG, "LD2450 force update complete (incl. zone target counts)");
}

/**
 * Force update LD2410C and config attributes to Zigbee (for initial reporting)
 * Split into smaller lock sections to prevent blocking Zigbee task.
 */
static void shs_ld2410c_force_update(void) {
    ESP_LOGI(SHS_TAG, "Force update LD2410C: moving=%d static=%d occ=%d",
             shs_moving_state, shs_static_state, shs_occupancy_state);

    /* EP2: LD2410C occupancy */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        uint8_t occ_val = shs_occupancy_state ? 1 : 0;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_OCC,
            ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID,
            &occ_val,
            true
        );
        esp_zb_lock_release();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* EP17: Moving target state (genBinaryInput - reliable) */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        uint8_t moving_val = shs_moving_state ? 1 : 0;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_LD2410C_MOVING,
            SHS_CLUSTER_BINARY_INPUT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_PRESENT_VALUE_BINARY,
            &moving_val,
            true
        );
        esp_zb_lock_release();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* EP18: Static target state (genBinaryInput - reliable) */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        uint8_t static_val = shs_static_state ? 1 : 0;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_LD2410C_STATIC,
            SHS_CLUSTER_BINARY_INPUT,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_PRESENT_VALUE_BINARY,
            &static_val,
            true
        );
        esp_zb_lock_release();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* Also update manufacturer-specific attrs for backwards compatibility */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        uint8_t moving_val = shs_moving_state ? 1 : 0;
        uint8_t static_val = shs_static_state ? 1 : 0;
        esp_zb_zcl_set_attribute_val(
            SHS_EP_OCC,
            ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_OCC_MOVING_TARGET,
            &moving_val,
            true
        );
        esp_zb_zcl_set_attribute_val(
            SHS_EP_OCC,
            ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_OCC_STATIC_TARGET,
            &static_val,
            true
        );
        esp_zb_lock_release();
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    /* EP1: Config cluster attributes - batch these together since they're related */
    if (esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        esp_zb_zcl_set_attribute_val(
            SHS_EP_LIGHT,
            SHS_CL_CFG_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_MOVEMENT_COOLDOWN,
            &shs_movement_cooldown_sec,
            true
        );

        esp_zb_zcl_set_attribute_val(
            SHS_EP_LIGHT,
            SHS_CL_CFG_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_OCC_CLEAR_COOLDOWN,
            &shs_occupancy_clear_sec,
            true
        );

        esp_zb_zcl_set_attribute_val(
            SHS_EP_LIGHT,
            SHS_CL_CFG_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_MOVING_SENS_0_10,
            &shs_sens_mv_0_10,
            true
        );

        esp_zb_zcl_set_attribute_val(
            SHS_EP_LIGHT,
            SHS_CL_CFG_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_STATIC_SENS_0_10,
            &shs_sens_st_0_10,
            true
        );

        esp_zb_zcl_set_attribute_val(
            SHS_EP_LIGHT,
            SHS_CL_CFG_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_MOVING_MAX_GATE,
            &shs_moving_max_gate,
            true
        );

        esp_zb_zcl_set_attribute_val(
            SHS_EP_LIGHT,
            SHS_CL_CFG_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_STATIC_MAX_GATE,
            &shs_static_max_gate,
            true
        );

        esp_zb_zcl_set_attribute_val(
            SHS_EP_LIGHT,
            SHS_CL_CFG_ID,
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            SHS_ATTR_POSITION_REPORTING,
            &shs_position_reporting,
            true
        );
        esp_zb_lock_release();
    }

    shs_last_successful_tx = (uint32_t)(esp_timer_get_time() / 1000);
    ESP_LOGI(SHS_TAG, "LD2410C/Config force update complete (cooldown=%d, delay=%d, sens_mv=%d, sens_st=%d, gate_mv=%d, gate_st=%d)",
             shs_movement_cooldown_sec, shs_occupancy_clear_sec,
             shs_sens_mv_0_10, shs_sens_st_0_10,
             shs_moving_max_gate, shs_static_max_gate);
}

/* ============================================================================
 * APPLY LD2410 CONFIGURATION
 * ============================================================================ */

static void shs_apply_ld2410_config(void) {
    /* Set max gates and timeout */
    ld2410_set_max_gate_timeout(
        (uint8_t)shs_moving_max_gate,
        (uint8_t)shs_static_max_gate,
        shs_occupancy_clear_sec
    );

    /* Set global sensitivity */
    ld2410_set_all_sensitivity(shs_moving_sens_0_100, shs_static_sens_0_100);

    /* DISABLED: Gate 0 blocking - testing detection speed without this filter
     * Uncomment if false positives occur from sensor housing/mounting reflections */
    // ld2410_set_gate_sensitivity(0, 0, 0);

    /* Set cooldowns */
    ld2410_set_moving_cooldown(shs_movement_cooldown_sec);
    ld2410_set_occupancy_delay(shs_occupancy_clear_sec);

    /* Read firmware version */
    ld2410_read_firmware_version();
    const ld2410_state_t *state = ld2410_get_state();
    if (state->firmware.valid) {
        snprintf(shs_firmware_version, sizeof(shs_firmware_version),
                 "V%d.%02d", state->firmware.major, state->firmware.minor);
    }

    /* Read current config from sensor */
    ld2410_read_config();

    ESP_LOGI(SHS_TAG, "LD2410 configuration applied");
}

/* ============================================================================
 * ZIGBEE ATTRIBUTE WRITE HANDLER
 * ============================================================================ */

static esp_err_t shs_zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message) {
    if (!message) return ESP_OK;

    /* EP1: genOnOff (light) */
    if (message->info.dst_endpoint == SHS_EP_LIGHT &&
        message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
        if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
            message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
            bool light_state = *(bool *)message->attribute.data.value;
            ESP_LOGI(SHS_TAG, "Light -> %s", light_state ? "ON" : "OFF");
            light_driver_set_power(light_state);
            return ESP_OK;
        }
    }

    /* EP1: Config cluster (0xFDCD) */
    if (message->info.dst_endpoint == SHS_EP_LIGHT &&
        message->info.cluster == SHS_CL_CFG_ID) {

        uint16_t v = 0;
        uint8_t v8 = 0;
        int16_t v16s = 0;

        if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
            v = *(uint16_t *)message->attribute.data.value;
        } else if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_S16) {
            v16s = *(int16_t *)message->attribute.data.value;
        } else if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8 ||
                   message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
            v8 = *(uint8_t *)message->attribute.data.value;
            v = v8;
        }

        switch (message->attribute.id) {
            case SHS_ATTR_MOVEMENT_COOLDOWN:
                if (v > SHS_COOLDOWN_MAX_SEC) v = SHS_COOLDOWN_MAX_SEC;
                shs_movement_cooldown_sec = v;
                ld2410_set_moving_cooldown(v);
                shs_save_enqueue(SHS_SAVE_IMMEDIATE_U16, (SHS_ATTR_MOVEMENT_COOLDOWN << 8));
                ESP_LOGI(SHS_TAG, "Set Movement Cooldown = %us", (unsigned)v);
                return ESP_OK;

            case SHS_ATTR_OCC_CLEAR_COOLDOWN:
                shs_occupancy_clear_sec = v;
                ld2410_set_max_gate_timeout((uint8_t)shs_moving_max_gate,
                                           (uint8_t)shs_static_max_gate, v);
                shs_zb_set_ou_delay_ep2(v);
                shs_save_enqueue(SHS_SAVE_IMMEDIATE_U16, (SHS_ATTR_OCC_CLEAR_COOLDOWN << 8));
                ESP_LOGI(SHS_TAG, "Set Occupancy Cooldown = %us", (unsigned)v);
                return ESP_OK;

            case SHS_ATTR_MOVING_SENS_0_10:
                if (v > 10) v = 10;
                shs_sens_mv_0_10 = v;
                /* Invert: user's 10 (max sens) → sensor threshold 0 (easiest to trigger)
                 *         user's 0 (min sens)  → sensor threshold 100 (hardest to trigger) */
                shs_moving_sens_0_100 = (uint8_t)((10 - v) * 10);
                ld2410_set_all_sensitivity(shs_moving_sens_0_100, shs_static_sens_0_100);
                shs_save_enqueue(SHS_SAVE_DEBOUNCE_SENS_MOVE, shs_moving_sens_0_100);
                ESP_LOGI(SHS_TAG, "Set Moving Sensitivity = %u/10 (threshold=%u)", (unsigned)v, (unsigned)shs_moving_sens_0_100);
                return ESP_OK;

            case SHS_ATTR_STATIC_SENS_0_10:
                if (v > 10) v = 10;
                shs_sens_st_0_10 = v;
                /* Invert: user's 10 (max sens) → sensor threshold 0 (easiest to trigger)
                 *         user's 0 (min sens)  → sensor threshold 100 (hardest to trigger) */
                shs_static_sens_0_100 = (uint8_t)((10 - v) * 10);
                ld2410_set_all_sensitivity(shs_moving_sens_0_100, shs_static_sens_0_100);
                shs_save_enqueue(SHS_SAVE_DEBOUNCE_SENS_STATIC, shs_static_sens_0_100);
                ESP_LOGI(SHS_TAG, "Set Static Sensitivity = %u/10 (threshold=%u)", (unsigned)v, (unsigned)shs_static_sens_0_100);
                return ESP_OK;

            case SHS_ATTR_MOVING_MAX_GATE:
                if (v > 8) v = 8;
                shs_moving_max_gate = v;
                ld2410_set_max_gate_timeout((uint8_t)v, (uint8_t)shs_static_max_gate,
                                           shs_occupancy_clear_sec);
                shs_save_enqueue(SHS_SAVE_DEBOUNCE_GATE_MOVE, v);
                ESP_LOGI(SHS_TAG, "Set Movement Detection Range = %u", (unsigned)v);
                return ESP_OK;

            case SHS_ATTR_STATIC_MAX_GATE:
                if (v < 2) v = 2; else if (v > 8) v = 8;
                shs_static_max_gate = v;
                ld2410_set_max_gate_timeout((uint8_t)shs_moving_max_gate, (uint8_t)v,
                                           shs_occupancy_clear_sec);
                shs_save_enqueue(SHS_SAVE_DEBOUNCE_GATE_STATIC, v);
                ESP_LOGI(SHS_TAG, "Set Static Detection Range = %u", (unsigned)v);
                return ESP_OK;

            case SHS_ATTR_POSITION_REPORTING:
                shs_position_reporting = (v8 != 0);
                ld2450_set_verbose_logging(shs_position_reporting);  /* Enable verbose logs when position reporting is on */
                light_driver_set_power(shs_position_reporting);  /* Turn light ON when config mode enabled */
                ESP_LOGI(SHS_TAG, "Position Reporting = %s (X/Y coordinate updates %s, light %s)",
                         shs_position_reporting ? "ON" : "OFF",
                         shs_position_reporting ? "enabled" : "disabled",
                         shs_position_reporting ? "ON" : "OFF");
                if (shs_position_reporting) {
                    ESP_LOGW(SHS_TAG, "WARNING: Position reporting will increase Zigbee traffic!");
                }
                return ESP_OK;

            case SHS_ATTR_MIN_MOVING_ENERGY:
                if (v > 100) v = 100;
                shs_min_moving_energy = v;
                /* ld2410_set_min_moving_energy removed - using backup2 driver */
                ESP_LOGI(SHS_TAG, "Min Moving Energy = %d (NOT USED - backup2 driver)", (int)shs_min_moving_energy);
                return ESP_OK;

            case SHS_ATTR_MIN_STATIC_ENERGY:
                if (v > 100) v = 100;
                shs_min_static_energy = v;
                /* ld2410_set_min_static_energy removed - using backup2 driver */
                ESP_LOGI(SHS_TAG, "Min Static Energy = %d (NOT USED - backup2 driver)", (int)shs_min_static_energy);
                return ESP_OK;

            /* Zone configuration attributes - received from web configurator via MQTT/Z2M */
            /* Uses debounced apply to wait for all attributes before configuring LD2450 */
            case SHS_ATTR_ZONE_TYPE_CFG:
                shs_zone_type = v8;
                ESP_LOGI(SHS_TAG, "Zone Type = %d (%s)", shs_zone_type,
                         shs_zone_type == 0 ? "disabled" :
                         shs_zone_type == 1 ? "detection" : "filter");
                shs_zone_cfg_schedule_apply();
                return ESP_OK;

            /* Zone 1 configuration */
            case SHS_ATTR_ZONE1_ENABLED:
                shs_zone1_enabled = (v8 != 0);
                ESP_LOGI(SHS_TAG, "Zone 1 Enabled = %d", shs_zone1_enabled);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE1_X1_CFG:
                shs_zone1_x1 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 1 X1 = %d", shs_zone1_x1);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE1_Y1_CFG:
                shs_zone1_y1 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 1 Y1 = %d", shs_zone1_y1);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE1_X2_CFG:
                shs_zone1_x2 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 1 X2 = %d", shs_zone1_x2);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE1_Y2_CFG:
                shs_zone1_y2 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 1 Y2 = %d", shs_zone1_y2);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE1_TYPE_CFG:
                shs_zone1_type = v8;
                ESP_LOGI(SHS_TAG, "Zone 1 Type = %d", shs_zone1_type);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;

            /* Zone 2 configuration */
            case SHS_ATTR_ZONE2_ENABLED:
                shs_zone2_enabled = (v8 != 0);
                ESP_LOGI(SHS_TAG, "Zone 2 Enabled = %d", shs_zone2_enabled);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE2_X1_CFG:
                shs_zone2_x1 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 2 X1 = %d", shs_zone2_x1);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE2_Y1_CFG:
                shs_zone2_y1 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 2 Y1 = %d", shs_zone2_y1);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE2_X2_CFG:
                shs_zone2_x2 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 2 X2 = %d", shs_zone2_x2);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE2_Y2_CFG:
                shs_zone2_y2 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 2 Y2 = %d", shs_zone2_y2);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE2_TYPE_CFG:
                shs_zone2_type = v8;
                ESP_LOGI(SHS_TAG, "Zone 2 Type = %d", shs_zone2_type);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;

            /* Zone 3 configuration */
            case SHS_ATTR_ZONE3_ENABLED:
                shs_zone3_enabled = (v8 != 0);
                ESP_LOGI(SHS_TAG, "Zone 3 Enabled = %d", shs_zone3_enabled);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE3_X1_CFG:
                shs_zone3_x1 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 3 X1 = %d", shs_zone3_x1);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE3_Y1_CFG:
                shs_zone3_y1 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 3 Y1 = %d", shs_zone3_y1);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE3_X2_CFG:
                shs_zone3_x2 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 3 X2 = %d", shs_zone3_x2);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE3_Y2_CFG:
                shs_zone3_y2 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 3 Y2 = %d", shs_zone3_y2);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE3_TYPE_CFG:
                shs_zone3_type = v8;
                ESP_LOGI(SHS_TAG, "Zone 3 Type = %d", shs_zone3_type);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;

            /* Zone 4 configuration */
            case SHS_ATTR_ZONE4_ENABLED:
                shs_zone4_enabled = (v8 != 0);
                ESP_LOGI(SHS_TAG, "Zone 4 Enabled = %d", shs_zone4_enabled);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE4_X1_CFG:
                shs_zone4_x1 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 4 X1 = %d", shs_zone4_x1);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE4_Y1_CFG:
                shs_zone4_y1 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 4 Y1 = %d", shs_zone4_y1);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE4_X2_CFG:
                shs_zone4_x2 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 4 X2 = %d", shs_zone4_x2);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE4_Y2_CFG:
                shs_zone4_y2 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 4 Y2 = %d", shs_zone4_y2);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE4_TYPE_CFG:
                shs_zone4_type = v8;
                ESP_LOGI(SHS_TAG, "Zone 4 Type = %d", shs_zone4_type);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;

            /* Zone 5 configuration */
            case SHS_ATTR_ZONE5_ENABLED:
                shs_zone5_enabled = (v8 != 0);
                ESP_LOGI(SHS_TAG, "Zone 5 Enabled = %d", shs_zone5_enabled);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE5_X1_CFG:
                shs_zone5_x1 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 5 X1 = %d", shs_zone5_x1);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE5_Y1_CFG:
                shs_zone5_y1 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 5 Y1 = %d", shs_zone5_y1);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE5_X2_CFG:
                shs_zone5_x2 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 5 X2 = %d", shs_zone5_x2);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE5_Y2_CFG:
                shs_zone5_y2 = v16s;
                ESP_LOGI(SHS_TAG, "Zone 5 Y2 = %d", shs_zone5_y2);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;
            case SHS_ATTR_ZONE5_TYPE_CFG:
                shs_zone5_type = v8;
                ESP_LOGI(SHS_TAG, "Zone 5 Type = %d", shs_zone5_type);
                shs_zone_cfg_schedule_apply();
                return ESP_OK;

            default:
                break;
        }
    }

    return ESP_OK;
}

static esp_err_t shs_zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message) {
    if (callback_id == ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID) {
        return shs_zb_attribute_handler((const esp_zb_zcl_set_attr_value_message_t *)message);
    }
    return ESP_OK;
}

/* ============================================================================
 * BOOT BUTTON (CONFIG MODE + FACTORY RESET)
 * ============================================================================ */

#define SHS_FACTORY_RESET_PRESS_MS   6000   /* 6 seconds for factory reset/pairing */
#define SHS_TRIPLE_CLICK_WINDOW_MS   800    /* 800ms window for triple-click */
#define SHS_CLICK_MIN_MS             50     /* Minimum press duration for a click */
#define SHS_CLICK_MAX_MS             400    /* Maximum press duration for a click */
#define SHS_STARTUP_IGNORE_MS        2000   /* Ignore button for 2s after boot */
#define SHS_DEBOUNCE_MS              50     /* Debounce time between state changes */

static void shs_flash_led(int count, int on_ms, int off_ms) {
    for (int i = 0; i < count; i++) {
        light_driver_set_power(true);
        vTaskDelay(pdMS_TO_TICKS(on_ms));
        light_driver_set_power(false);
        if (off_ms > 0) vTaskDelay(pdMS_TO_TICKS(off_ms));
    }
}

static void shs_boot_button_task(void *pv) {
    const TickType_t poll = pdMS_TO_TICKS(20);
    const uint32_t factory_reset_ticks = SHS_FACTORY_RESET_PRESS_MS / 20;
    const uint32_t click_min_ticks = SHS_CLICK_MIN_MS / 20;
    const uint32_t click_max_ticks = SHS_CLICK_MAX_MS / 20;
    const uint32_t triple_click_window_ticks = SHS_TRIPLE_CLICK_WINDOW_MS / 20;
    __attribute__((unused)) const uint32_t startup_ignore_ticks = SHS_STARTUP_IGNORE_MS / 20;
    const uint32_t debounce_ticks = SHS_DEBOUNCE_MS / 20;

    gpio_config_t io = {
        .pin_bit_mask = 1ULL << SHS_BOOT_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    ESP_LOGI(SHS_TAG, "BOOT button: Triple-click for config mode, hold 6s for factory reset");

    /* Wait for startup period - ignore any button activity */
    ESP_LOGI(SHS_TAG, "Ignoring button for %dms startup period...", SHS_STARTUP_IGNORE_MS);
    vTaskDelay(pdMS_TO_TICKS(SHS_STARTUP_IGNORE_MS));

    uint32_t held = 0;
    bool reset_armed = false;
    int last_level = 1;  /* Assume released initially */
    uint32_t level_stable_count = 0;

    /* Triple-click detection */
    uint8_t click_count = 0;
    uint32_t last_click_time = 0;
    uint32_t tick_counter = 0;

    while (1) {
        int raw_level = gpio_get_level(SHS_BOOT_BUTTON_GPIO);
        tick_counter++;

        /* Debounce: only accept level change after stable for debounce period */
        static int debounced_level = 1;
        if (raw_level == debounced_level) {
            level_stable_count = 0;
        } else {
            level_stable_count++;
            if (level_stable_count >= debounce_ticks) {
                debounced_level = raw_level;
                level_stable_count = 0;
            }
        }

        int level = debounced_level;

        if (level == 0) {
            /* Button pressed */
            if (held < factory_reset_ticks + 100) held++;

            /* Factory reset threshold (6 seconds) */
            if (!reset_armed && held >= (factory_reset_ticks - 20) && held < factory_reset_ticks) {
                reset_armed = true;
                click_count = 0;  /* Cancel any click sequence */
                ESP_LOGW(SHS_TAG, "Keep holding for factory reset...");
                light_driver_set_power(true);
            }
            if (held >= factory_reset_ticks) {
                ESP_LOGW(SHS_TAG, "BOOT long-press confirmed: factory reset...");
                light_driver_set_power(false);
                esp_zb_factory_reset();
                vTaskDelay(pdMS_TO_TICKS(100));
                esp_restart();
            }
        } else {
            /* Button released */
            if (reset_armed) {
                light_driver_set_power(false);
                ESP_LOGI(SHS_TAG, "Factory reset cancelled (released too early)");
            }

            /* Check if this was a valid click (short press) - only on transition from pressed */
            if (last_level == 0 && held >= click_min_ticks && held <= click_max_ticks && !reset_armed) {
                /* Valid click detected */
                uint32_t now = tick_counter;

                /* Check if within triple-click window */
                if (click_count > 0 && (now - last_click_time) > triple_click_window_ticks) {
                    /* Window expired, reset count */
                    ESP_LOGI(SHS_TAG, "Click window expired, resetting count");
                    click_count = 0;
                }

                click_count++;
                last_click_time = now;

                ESP_LOGI(SHS_TAG, "Click %d detected (held=%lu ticks)", click_count, (unsigned long)held);

                if (click_count == 4) {
                    /* QUADRUPLE-CLICK: Factory reset LD2410C sensor */
                    click_count = 0;
                    ESP_LOGW(SHS_TAG, "QUADRUPLE-CLICK: Factory resetting LD2410C sensor...");
                    shs_flash_led(4, 100, 100);  /* 4 flashes to confirm */
                    esp_err_t err = ld2410_factory_reset();
                    if (err == ESP_OK) {
                        ESP_LOGI(SHS_TAG, "LD2410C factory reset SUCCESS - sensor will restart");
                        shs_flash_led(1, 1000, 0);  /* Long flash = success */
                    } else {
                        ESP_LOGE(SHS_TAG, "LD2410C factory reset FAILED: %s", esp_err_to_name(err));
                        shs_flash_led(5, 50, 50);  /* Rapid flashes = error */
                    }
                } else if (click_count == 3) {
                    /* Triple-click detected! Toggle config mode */
                    click_count = 0;
                    shs_position_reporting = !shs_position_reporting;
                    ld2450_set_verbose_logging(shs_position_reporting);  /* Enable verbose logs when config mode is on */
                    light_driver_set_power(shs_position_reporting);  /* Turn light ON when config mode enabled */

                    ESP_LOGI(SHS_TAG, "TRIPLE-CLICK: CONFIG MODE %s (light %s)",
                             shs_position_reporting ? "ENABLED" : "DISABLED",
                             shs_position_reporting ? "ON" : "OFF");

                    /* Flash LED: 2 quick flashes = ON, 1 long flash = OFF */
                    if (shs_position_reporting) {
                        shs_flash_led(2, 100, 100);
                    } else {
                        shs_flash_led(1, 500, 0);
                    }

                    /* Update Zigbee attribute */
                    if (shs_zb_ready && esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_ZB_LOCK_TIMEOUT_MS))) {
                        esp_zb_zcl_set_attribute_val(
                            SHS_EP_LIGHT,
                            SHS_CL_CFG_ID,
                            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                            SHS_ATTR_POSITION_REPORTING,
                            &shs_position_reporting,
                            true
                        );
                        esp_zb_lock_release();
                    }
                }
            } else if (last_level == 0 && held > click_max_ticks && !reset_armed) {
                /* Long press but not long enough for factory reset - reset click count */
                if (click_count > 0) {
                    ESP_LOGI(SHS_TAG, "Long press detected, resetting click count");
                    click_count = 0;
                }
            }

            held = 0;
            reset_armed = false;
        }

        last_level = level;

        /* Reset click count if window expired (while button is released) */
        if (click_count > 0 && level == 1 && (tick_counter - last_click_time) > triple_click_window_ticks) {
            ESP_LOGI(SHS_TAG, "Click window timeout, resetting count");
            click_count = 0;
        }

        vTaskDelay(poll);
    }
}

/* ============================================================================
 * LD2410 PROCESSING TASK
 * ============================================================================ */

static void shs_ld2410_task(void *pvParameters) {
    ESP_LOGI(SHS_TAG, "LD2410 processing task started");

    uint32_t last_connected_time = 0;
    bool was_connected = false;
    const uint32_t RECOVERY_INTERVAL_MS = 30000;  // Try recovery every 30s if disconnected

    while (1) {
        ld2410_process();

        /* Monitor connection state and attempt recovery if needed */
        bool connected = ld2410_is_connected();
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (connected) {
            last_connected_time = now;
            if (!was_connected) {
                ESP_LOGI(SHS_TAG, "LD2410 connection restored");
            }
        } else if (was_connected) {
            ESP_LOGW(SHS_TAG, "LD2410 disconnected - will attempt recovery");
        } else if ((now - last_connected_time) > RECOVERY_INTERVAL_MS && last_connected_time > 0) {
            /* Periodically try to restart LD2410 if it stays disconnected */
            ESP_LOGW(SHS_TAG, "LD2410 still disconnected - sending restart command");
            ld2410_restart();
            last_connected_time = now;  // Reset timer
            vTaskDelay(pdMS_TO_TICKS(500));  // Give sensor time to restart
            shs_apply_ld2410_config();  // Re-apply configuration
        }

        was_connected = connected;
        vTaskDelay(pdMS_TO_TICKS(20));  /* 20ms delay - priority 4 gives Zigbee room */
    }
}

/* ============================================================================
 * LD2450 PROCESSING TASK
 * ============================================================================ */

static void shs_ld2450_task(void *pvParameters) {
    ESP_LOGI(SHS_TAG, "LD2450 processing task started");

    uint32_t last_connected_time = 0;
    bool was_connected = false;
    const uint32_t RECOVERY_INTERVAL_MS = 30000;  // Try recovery every 30s if disconnected

    /* Zigbee connectivity monitoring */
    uint32_t last_zb_check_time = 0;

    while (1) {
        ld2450_process();

        /* Check if zone config needs to be applied (debounced) */
        shs_zone_cfg_check_pending();

        /* UART counter disabled in genAnalogInput implementation */

        /* Monitor connection state and attempt recovery if needed */
        bool connected = ld2450_is_connected();
        uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);

        if (connected) {
            last_connected_time = now;
            if (!was_connected) {
                ESP_LOGI(SHS_TAG, "LD2450 connection established");
                /* Read firmware version on first connection */
                vTaskDelay(pdMS_TO_TICKS(500));  // Wait for sensor to stabilize
                ld2450_read_firmware_version();
            }
        } else if (was_connected) {
            ESP_LOGW(SHS_TAG, "LD2450 disconnected - will attempt recovery");
        } else if ((now - last_connected_time) > RECOVERY_INTERVAL_MS && last_connected_time > 0) {
            /* Periodically try to restart LD2450 if it stays disconnected */
            ESP_LOGW(SHS_TAG, "LD2450 still disconnected - sending restart command");
            ld2450_restart();
            last_connected_time = now;  // Reset timer
            vTaskDelay(pdMS_TO_TICKS(500));  // Give sensor time to restart
            shs_zone_cfg_apply_to_sensor();  // Re-apply zone configuration
        }

        was_connected = connected;

        /* Periodic Zigbee connectivity check and heartbeat (every 60 seconds) */
        if (shs_zb_ready && (now - last_zb_check_time) >= SHS_ZB_CONNECTIVITY_CHECK_MS) {
            last_zb_check_time = now;

            /* Log diagnostic stats */
            uint32_t time_since_last_tx = now - shs_last_successful_tx;
            ESP_LOGI(SHS_TAG, "Zigbee stats: lock_ok=%lu lock_fail=%lu tx_ok=%lu last_tx=%lums ago",
                     (unsigned long)shs_lock_success_count, (unsigned long)shs_lock_fail_count,
                     (unsigned long)shs_tx_success_count, (unsigned long)time_since_last_tx);

            /* Send heartbeat report (target count) to keep connection alive */
            /* This prevents false "no TX" detection when room has stable presence */
            if (shs_zb_connected && !shs_zb_rejoin_pending) {
                shs_zb_set_analog_value(SHS_EP_LD2450_TARGET_COUNT, (float)shs_ld2450_target_count);
                shs_zb_report_analog_attr(SHS_EP_LD2450_TARGET_COUNT);
                ESP_LOGD(SHS_TAG, "Zigbee heartbeat sent (target_count=%d)", shs_ld2450_target_count);
            }

            /* Check if we haven't had a successful TX in a while (3 minutes) */
            /* This should now only trigger if heartbeat also fails */
            if (time_since_last_tx > 180000 && shs_last_successful_tx > 0 && !shs_zb_rejoin_pending) {
                ESP_LOGW(SHS_TAG, "Zigbee: No successful TX for %lu ms (even heartbeat failed) - scheduling rejoin",
                         (unsigned long)time_since_last_tx);
                ESP_LOGW(SHS_TAG, "Zigbee: consecutive lock fails: %lu", (unsigned long)shs_lock_consecutive_fails);
                shs_zb_rejoin_pending = true;
                shs_zb_connected = false;
                /* Reset counters after rejoin attempt */
                shs_lock_fail_count = 0;
                shs_lock_success_count = 0;
                shs_tx_success_count = 0;
                esp_zb_scheduler_alarm((esp_zb_callback_t)shs_bdb_start_top_level_commissioning_cb,
                                       ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));  /* 20ms delay - priority 4 gives Zigbee room */
    }
}

/* ============================================================================
 * ZIGBEE SIGNAL HANDLER
 * ============================================================================ */

static void shs_bdb_start_top_level_commissioning_cb(uint8_t mode_mask) {
    if (esp_zb_bdb_start_top_level_commissioning(mode_mask) != ESP_OK) {
        ESP_LOGW(SHS_TAG, "Failed to start Zigbee commissioning");
    }
}

static void shs_basic_publish_metadata_ep1(void) {
    if (!shs_zb_ready) return;

    const char *date_code = SHS_BASIC_DATE_CODE;
    const char *sw_build = SHS_BASIC_SW_BUILD_ID;
    uint8_t power_src = 0x01;

    if (!esp_zb_lock_acquire(pdMS_TO_TICKS(SHS_FORCE_UPDATE_LOCK_TIMEOUT_MS))) {
        ESP_LOGW(SHS_TAG, "Failed to acquire lock for metadata publish");
        return;
    }

    esp_zb_zcl_set_attribute_val(SHS_EP_LIGHT,
                                 ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID,
                                 &power_src, false);

    esp_zb_zcl_set_attribute_val(SHS_EP_LIGHT,
                                 ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BASIC_DATE_CODE_ID,
                                 (void *)date_code, false);
    esp_zb_zcl_set_attribute_val(SHS_EP_LIGHT,
                                 ESP_ZB_ZCL_CLUSTER_ID_BASIC,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_BASIC_SW_BUILD_ID,
                                 (void *)sw_build, false);

    esp_zb_lock_release();
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct) {
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;

    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(SHS_TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            shs_zb_ready = true;
            shs_last_successful_tx = (uint32_t)(esp_timer_get_time() / 1000);

            /* Publish metadata */
            shs_basic_publish_metadata_ep1();
            shs_zb_set_ou_delay_ep2(shs_occupancy_clear_sec);

            /* LD2410C: Push current state to Zigbee (like older backup) */
            shs_zb_set_bool_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                                SHS_ATTR_OCC_MOVING_TARGET, shs_moving_state);
            shs_zb_set_bool_attr(SHS_EP_OCC, ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING,
                                SHS_ATTR_OCC_STATIC_TARGET, shs_static_state);
            shs_zb_set_occ_bitmap(SHS_EP_OCC, shs_occupancy_state);

            ESP_LOGI(SHS_TAG, "Device started up in%s factory-reset mode",
                     esp_zb_bdb_is_factory_new() ? "" : " non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(SHS_TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(SHS_TAG, "Device rebooted - already joined network");
                shs_zb_connected = true;
                /* Device already joined - force update sensor states now */
                shs_ld2410c_force_update();
                shs_ld2450_force_update();
            }
        } else {
            ESP_LOGW(SHS_TAG, "Failed to initialize Zigbee stack (%s)", esp_err_to_name(err_status));
        }
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(SHS_TAG, "Joined network (PAN:0x%04hx, Ch:%d)",
                     esp_zb_get_pan_id(), esp_zb_get_current_channel());
            shs_zb_connected = true;
            shs_zb_rejoin_pending = false;  /* Clear rejoin flag on successful join */
            shs_last_successful_tx = (uint32_t)(esp_timer_get_time() / 1000);
            /* Device just joined - force update sensor states now */
            shs_ld2410c_force_update();
            shs_ld2450_force_update();
        } else {
            ESP_LOGW(SHS_TAG, "Network steering not successful (%s)", esp_err_to_name(err_status));
            shs_zb_connected = false;
            /* Keep rejoin pending and retry */
            esp_zb_scheduler_alarm((esp_zb_callback_t)shs_bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;

    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        /* Device has lost connection to coordinator - attempt recovery */
        shs_zb_connected = false;
        if (!shs_zb_rejoin_pending) {
            ESP_LOGW(SHS_TAG, "Device unavailable signal (0x3c) - scheduling rejoin");
            shs_zb_rejoin_pending = true;
            /* Schedule network rejoin attempt after 5 seconds */
            esp_zb_scheduler_alarm((esp_zb_callback_t)shs_bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
        } else {
            ESP_LOGD(SHS_TAG, "Device unavailable signal - rejoin already pending, ignoring");
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        /* Device is leaving the network */
        shs_zb_connected = false;
        if (!shs_zb_rejoin_pending) {
            ESP_LOGW(SHS_TAG, "Leave signal received - scheduling rejoin");
            shs_zb_rejoin_pending = true;
            /* Schedule rejoin after 2 seconds */
            esp_zb_scheduler_alarm((esp_zb_callback_t)shs_bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 2000);
        } else {
            ESP_LOGD(SHS_TAG, "Leave signal - rejoin already pending, ignoring");
        }
        break;

    case ESP_ZB_NWK_SIGNAL_NO_ACTIVE_LINKS_LEFT:
        /* No parent/links available */
        shs_zb_connected = false;
        if (!shs_zb_rejoin_pending) {
            ESP_LOGW(SHS_TAG, "No active network links - scheduling rejoin");
            shs_zb_rejoin_pending = true;
            esp_zb_scheduler_alarm((esp_zb_callback_t)shs_bdb_start_top_level_commissioning_cb,
                                   ESP_ZB_BDB_MODE_NETWORK_STEERING, 3000);
        } else {
            ESP_LOGD(SHS_TAG, "No active links signal - rejoin already pending, ignoring");
        }
        break;

    default:
        ESP_LOGI(SHS_TAG, "ZDO signal: %s (0x%x), status: %s",
                 esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        break;
    }
}

/* ============================================================================
 * ZIGBEE TASK - ENDPOINT & CLUSTER CREATION
 * ============================================================================ */

static void shs_zigbee_task(void *pvParameters) {
    esp_zb_cfg_t zb_nwk_cfg = SHS_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    zcl_basic_manufacturer_info_t info = {
        .manufacturer_name = SHS_MANUFACTURER_NAME,
        .model_identifier = SHS_MODEL_IDENTIFIER,
    };

    esp_zb_ep_list_t *dev_ep_list = esp_zb_ep_list_create();

    /* ========== EP1: Light + Config Cluster ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        esp_zb_on_off_cluster_cfg_t on_off_cfg = {.on_off = ESP_ZB_ZCL_ON_OFF_ON_OFF_DEFAULT_VALUE};
        esp_zb_attribute_list_t *onoff = esp_zb_on_off_cluster_create(&on_off_cfg);

        esp_zb_cluster_list_add_basic_cluster(cl, esp_zb_basic_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        esp_zb_cluster_list_add_identify_cluster(cl, esp_zb_identify_cluster_create(NULL), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        esp_zb_cluster_list_add_on_off_cluster(cl, onoff, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* Custom Config Cluster (0xFDCD) */
        esp_zb_attribute_list_t *cfg_cl = esp_zb_zcl_attr_list_create(SHS_CL_CFG_ID);

        /* Original attributes */
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_MOVEMENT_COOLDOWN,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_movement_cooldown_sec);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_OCC_CLEAR_COOLDOWN,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_occupancy_clear_sec);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_MOVING_SENS_0_10,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_sens_mv_0_10);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_STATIC_SENS_0_10,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_sens_st_0_10);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_MOVING_MAX_GATE,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_moving_max_gate);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_STATIC_MAX_GATE,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_static_max_gate);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_POSITION_REPORTING,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_position_reporting);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_MIN_MOVING_ENERGY,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_min_moving_energy);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_MIN_STATIC_ENERGY,
            ESP_ZB_ZCL_ATTR_TYPE_U16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_min_static_energy);

        /* Zone configuration attributes */
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE_TYPE_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone_type);

        /* Zone 1 */
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE1_ENABLED,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone1_enabled);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE1_X1_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone1_x1);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE1_Y1_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone1_y1);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE1_X2_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone1_x2);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE1_Y2_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone1_y2);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE1_TARGETS_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &shs_zone1_targets);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE1_TYPE_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone1_type);

        /* Zone 2 */
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE2_ENABLED,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone2_enabled);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE2_X1_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone2_x1);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE2_Y1_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone2_y1);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE2_X2_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone2_x2);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE2_Y2_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone2_y2);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE2_TARGETS_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &shs_zone2_targets);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE2_TYPE_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone2_type);

        /* Zone 3 */
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE3_ENABLED,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone3_enabled);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE3_X1_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone3_x1);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE3_Y1_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone3_y1);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE3_X2_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone3_x2);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE3_Y2_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone3_y2);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE3_TARGETS_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &shs_zone3_targets);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE3_TYPE_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone3_type);

        /* Zone 4 */
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE4_ENABLED,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone4_enabled);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE4_X1_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone4_x1);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE4_Y1_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone4_y1);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE4_X2_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone4_x2);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE4_Y2_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone4_y2);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE4_TARGETS_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &shs_zone4_targets);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE4_TYPE_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone4_type);

        /* Zone 5 */
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE5_ENABLED,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone5_enabled);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE5_X1_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone5_x1);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE5_Y1_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone5_y1);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE5_X2_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone5_x2);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE5_Y2_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone5_y2);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE5_TARGETS_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &shs_zone5_targets);
        esp_zb_custom_cluster_add_custom_attr(cfg_cl, SHS_ATTR_ZONE5_TYPE_CFG,
            ESP_ZB_ZCL_ATTR_TYPE_U8, ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE, &shs_zone5_type);

        esp_zb_cluster_list_add_custom_cluster(cl, cfg_cl, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_LIGHT,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_ON_OFF_LIGHT_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);

        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_LIGHT, &info);
    }

    /* ========== EP2: Occupancy + Distance + Gates ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Standard Occupancy Sensing cluster */
        esp_zb_attribute_list_t *occ = esp_zb_occupancy_sensing_cluster_create(NULL);

        /* Add manufacturer-specific boolean attrs (for moving/static target state) */
        esp_zb_custom_cluster_add_custom_attr(occ, SHS_ATTR_OCC_MOVING_TARGET,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_moving_state);
        esp_zb_custom_cluster_add_custom_attr(occ, SHS_ATTR_OCC_STATIC_TARGET,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &shs_static_state);

        esp_zb_cluster_list_add_occupancy_sensing_cluster(cl, occ, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_OCC,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = 0x0107, /* Occupancy Sensor */
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
    }

    /* ========== EP3: LD2450 Occupancy (msOccupancySensing) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* Occupancy Sensing cluster - occupancy is uint8_t bitmap */
        esp_zb_occupancy_sensing_cluster_cfg_t occ_cfg = {
            .occupancy = shs_ld2450_occupancy ? 0x01 : 0x00,
        };
        esp_zb_attribute_list_t *occ_cluster = esp_zb_occupancy_sensing_cluster_create(&occ_cfg);
        esp_zb_cluster_list_add_occupancy_sensing_cluster(cl, occ_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_LD2450_OCC,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_LD2450_OCC, &info);
    }

    /* ========== EP4: LD2450 Target Count (genAnalogInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genAnalogInput for target count (0-3) */
        esp_zb_analog_input_cluster_cfg_t analog_cfg = {
            .out_of_service = false,
            .present_value = (float)shs_ld2450_target_count,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *analog_input = esp_zb_analog_input_cluster_create(&analog_cfg);
        esp_zb_cluster_list_add_analog_input_cluster(cl, analog_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_LD2450_TARGET_COUNT,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_LD2450_TARGET_COUNT, &info);
    }

    /* ========== EP5: Zone 1 Occupancy (genBinaryInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genBinaryInput for zone occupancy - present_value is uint8_t */
        esp_zb_binary_input_cluster_cfg_t binary_cfg = {
            .out_of_service = false,
            .present_value = shs_zone1_occupied ? 1 : 0,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *binary_input = esp_zb_binary_input_cluster_create(&binary_cfg);
        esp_zb_cluster_list_add_binary_input_cluster(cl, binary_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_LD2450_ZONE1,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_LD2450_ZONE1, &info);
    }

    /* ========== EP6: Zone 2 Occupancy (genBinaryInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genBinaryInput for zone occupancy - present_value is uint8_t */
        esp_zb_binary_input_cluster_cfg_t binary_cfg = {
            .out_of_service = false,
            .present_value = shs_zone2_occupied ? 1 : 0,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *binary_input = esp_zb_binary_input_cluster_create(&binary_cfg);
        esp_zb_cluster_list_add_binary_input_cluster(cl, binary_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_LD2450_ZONE2,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_LD2450_ZONE2, &info);
    }

    /* ========== EP7: Zone 3 Occupancy (genBinaryInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genBinaryInput for zone occupancy - present_value is uint8_t */
        esp_zb_binary_input_cluster_cfg_t binary_cfg = {
            .out_of_service = false,
            .present_value = shs_zone3_occupied ? 1 : 0,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *binary_input = esp_zb_binary_input_cluster_create(&binary_cfg);
        esp_zb_cluster_list_add_binary_input_cluster(cl, binary_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_LD2450_ZONE3,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_LD2450_ZONE3, &info);
    }

    /* ========== EP22: Zone 4 Occupancy (genBinaryInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genBinaryInput for zone occupancy - present_value is uint8_t */
        esp_zb_binary_input_cluster_cfg_t binary_cfg = {
            .out_of_service = false,
            .present_value = shs_zone4_occupied ? 1 : 0,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *binary_input = esp_zb_binary_input_cluster_create(&binary_cfg);
        esp_zb_cluster_list_add_binary_input_cluster(cl, binary_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_LD2450_ZONE4,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_LD2450_ZONE4, &info);
    }

    /* ========== EP23: Zone 5 Occupancy (genBinaryInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genBinaryInput for zone occupancy - present_value is uint8_t */
        esp_zb_binary_input_cluster_cfg_t binary_cfg = {
            .out_of_service = false,
            .present_value = shs_zone5_occupied ? 1 : 0,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *binary_input = esp_zb_binary_input_cluster_create(&binary_cfg);
        esp_zb_cluster_list_add_binary_input_cluster(cl, binary_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_LD2450_ZONE5,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_LD2450_ZONE5, &info);
    }

    /* ========== EP8-16: LD2450 Position Data (genAnalogInput) - Only Active When Position Reporting Enabled ========== */
    /* Target 1: X, Y, Distance */
    for (int i = 0; i < 3; i++) {
        uint8_t ep_base = SHS_EP_LD2450_T1_X + (i * 3);  /* EP8, EP11, EP14 */

        /* X coordinate endpoint */
        {
            esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
            esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
            esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

            esp_zb_analog_input_cluster_cfg_t analog_cfg = {
                .out_of_service = false,
                .present_value = 0.0f,  /* Initial value in mm */
                .status_flags = 0,
            };
            esp_zb_attribute_list_t *analog_input = esp_zb_analog_input_cluster_create(&analog_cfg);
            esp_zb_cluster_list_add_analog_input_cluster(cl, analog_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

            esp_zb_endpoint_config_t ep_cfg = {
                .endpoint = ep_base,
                .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
                .app_device_version = 0
            };
            esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
            esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, ep_base, &info);
        }

        /* Y coordinate endpoint */
        {
            esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
            esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
            esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

            esp_zb_analog_input_cluster_cfg_t analog_cfg = {
                .out_of_service = false,
                .present_value = 0.0f,  /* Initial value in mm */
                .status_flags = 0,
            };
            esp_zb_attribute_list_t *analog_input = esp_zb_analog_input_cluster_create(&analog_cfg);
            esp_zb_cluster_list_add_analog_input_cluster(cl, analog_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

            esp_zb_endpoint_config_t ep_cfg = {
                .endpoint = ep_base + 1,
                .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
                .app_device_version = 0
            };
            esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
            esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, ep_base + 1, &info);
        }

        /* Distance endpoint */
        {
            esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();
            esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
            esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

            esp_zb_analog_input_cluster_cfg_t analog_cfg = {
                .out_of_service = false,
                .present_value = 0.0f,  /* Initial value in mm */
                .status_flags = 0,
            };
            esp_zb_attribute_list_t *analog_input = esp_zb_analog_input_cluster_create(&analog_cfg);
            esp_zb_cluster_list_add_analog_input_cluster(cl, analog_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

            esp_zb_endpoint_config_t ep_cfg = {
                .endpoint = ep_base + 2,
                .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
                .app_device_version = 0
            };
            esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
            esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, ep_base + 2, &info);
        }
    }

    /* ========== EP17: LD2410C Moving Target (genBinaryInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genBinaryInput for moving target state */
        esp_zb_binary_input_cluster_cfg_t binary_cfg = {
            .out_of_service = false,
            .present_value = shs_moving_state ? 1 : 0,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *binary_input = esp_zb_binary_input_cluster_create(&binary_cfg);
        esp_zb_cluster_list_add_binary_input_cluster(cl, binary_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_LD2410C_MOVING,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_LD2410C_MOVING, &info);
    }

    /* ========== EP18: LD2410C Static Target (genBinaryInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genBinaryInput for static target state */
        esp_zb_binary_input_cluster_cfg_t binary_cfg = {
            .out_of_service = false,
            .present_value = shs_static_state ? 1 : 0,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *binary_input = esp_zb_binary_input_cluster_create(&binary_cfg);
        esp_zb_cluster_list_add_binary_input_cluster(cl, binary_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_LD2410C_STATIC,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_LD2410C_STATIC, &info);
    }

    /* ========== EP19: Zone 1 Target Count (genAnalogInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genAnalogInput for zone 1 target count (0-3) */
        esp_zb_analog_input_cluster_cfg_t analog_cfg = {
            .out_of_service = false,
            .present_value = (float)shs_zone1_targets,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *analog_input = esp_zb_analog_input_cluster_create(&analog_cfg);
        esp_zb_cluster_list_add_analog_input_cluster(cl, analog_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_ZONE1_TARGETS,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_ZONE1_TARGETS, &info);
    }

    /* ========== EP20: Zone 2 Target Count (genAnalogInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genAnalogInput for zone 2 target count (0-3) */
        esp_zb_analog_input_cluster_cfg_t analog_cfg = {
            .out_of_service = false,
            .present_value = (float)shs_zone2_targets,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *analog_input = esp_zb_analog_input_cluster_create(&analog_cfg);
        esp_zb_cluster_list_add_analog_input_cluster(cl, analog_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_ZONE2_TARGETS,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_ZONE2_TARGETS, &info);
    }

    /* ========== EP21: Zone 3 Target Count (genAnalogInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genAnalogInput for zone 3 target count (0-3) */
        esp_zb_analog_input_cluster_cfg_t analog_cfg = {
            .out_of_service = false,
            .present_value = (float)shs_zone3_targets,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *analog_input = esp_zb_analog_input_cluster_create(&analog_cfg);
        esp_zb_cluster_list_add_analog_input_cluster(cl, analog_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_ZONE3_TARGETS,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_ZONE3_TARGETS, &info);
    }

    /* ========== EP24: Zone 4 Target Count (genAnalogInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genAnalogInput for zone 4 target count (0-3) */
        esp_zb_analog_input_cluster_cfg_t analog_cfg = {
            .out_of_service = false,
            .present_value = (float)shs_zone4_targets,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *analog_input = esp_zb_analog_input_cluster_create(&analog_cfg);
        esp_zb_cluster_list_add_analog_input_cluster(cl, analog_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_ZONE4_TARGETS,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_ZONE4_TARGETS, &info);
    }

    /* ========== EP25: Zone 5 Target Count (genAnalogInput) ========== */
    {
        esp_zb_cluster_list_t *cl = esp_zb_zcl_cluster_list_create();

        /* Basic cluster */
        esp_zb_attribute_list_t *basic = esp_zb_basic_cluster_create(NULL);
        esp_zb_cluster_list_add_basic_cluster(cl, basic, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        /* genAnalogInput for zone 5 target count (0-3) */
        esp_zb_analog_input_cluster_cfg_t analog_cfg = {
            .out_of_service = false,
            .present_value = (float)shs_zone5_targets,
            .status_flags = 0,
        };
        esp_zb_attribute_list_t *analog_input = esp_zb_analog_input_cluster_create(&analog_cfg);
        esp_zb_cluster_list_add_analog_input_cluster(cl, analog_input, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        esp_zb_endpoint_config_t ep_cfg = {
            .endpoint = SHS_EP_ZONE5_TARGETS,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
        esp_zb_ep_list_add_ep(dev_ep_list, cl, ep_cfg);
        esp_zcl_utility_add_ep_basic_manufacturer_info(dev_ep_list, SHS_EP_ZONE5_TARGETS, &info);
    }

    /* Register device and start */
    esp_zb_device_register(dev_ep_list);
    esp_zb_core_action_handler_register(shs_zb_action_handler);
    esp_zb_set_primary_network_channel_set(SHS_PRIMARY_CHANNEL_MASK);

    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

/* ============================================================================
 * NVS SAVE WORKER TASK
 * ============================================================================ */

static void shs_save_worker(void *pv) {
    TickType_t last_mv_sens = 0, last_st_sens = 0, last_mv_gate = 0, last_st_gate = 0;
    bool pend_mv_sens = false, pend_st_sens = false, pend_mv_gate = false, pend_st_gate = false;
    uint8_t mv_sens_val = shs_moving_sens_0_100, st_sens_val = shs_static_sens_0_100;
    uint8_t mv_gate_val = (uint8_t)shs_moving_max_gate, st_gate_val = (uint8_t)shs_static_max_gate;

    shs_save_msg_t m;
    for (;;) {
        if (xQueueReceive(shs_save_q, &m, pdMS_TO_TICKS(50))) {
            switch (m.type) {
                case SHS_SAVE_IMMEDIATE_U16:
                    if ((m.u16 >> 8) == SHS_ATTR_MOVEMENT_COOLDOWN) {
                        shs_cfg_save_u16(SHS_NVS_KEY_MV_CD, shs_movement_cooldown_sec);
                    } else if ((m.u16 >> 8) == SHS_ATTR_OCC_CLEAR_COOLDOWN) {
                        shs_cfg_save_u16(SHS_NVS_KEY_OCC_CD, shs_occupancy_clear_sec);
                    }
                    break;
                case SHS_SAVE_DEBOUNCE_SENS_MOVE:
                    mv_sens_val = (uint8_t)m.u16; pend_mv_sens = true; last_mv_sens = xTaskGetTickCount();
                    break;
                case SHS_SAVE_DEBOUNCE_SENS_STATIC:
                    st_sens_val = (uint8_t)m.u16; pend_st_sens = true; last_st_sens = xTaskGetTickCount();
                    break;
                case SHS_SAVE_DEBOUNCE_GATE_MOVE:
                    mv_gate_val = (uint8_t)m.u16; pend_mv_gate = true; last_mv_gate = xTaskGetTickCount();
                    break;
                case SHS_SAVE_DEBOUNCE_GATE_STATIC:
                    st_gate_val = (uint8_t)m.u16; pend_st_gate = true; last_st_gate = xTaskGetTickCount();
                    break;
            }
        }

        TickType_t now = xTaskGetTickCount();
        if (pend_mv_sens && (now - last_mv_sens) >= pdMS_TO_TICKS(SHS_NVS_DEBOUNCE_MS)) {
            shs_cfg_save_u8(SHS_NVS_KEY_MV_SENS, mv_sens_val);
            pend_mv_sens = false;
        }
        if (pend_st_sens && (now - last_st_sens) >= pdMS_TO_TICKS(SHS_NVS_DEBOUNCE_MS)) {
            shs_cfg_save_u8(SHS_NVS_KEY_ST_SENS, st_sens_val);
            pend_st_sens = false;
        }
        if (pend_mv_gate && (now - last_mv_gate) >= pdMS_TO_TICKS(SHS_NVS_DEBOUNCE_MS)) {
            shs_cfg_save_u8(SHS_NVS_KEY_MV_GATE, mv_gate_val);
            pend_mv_gate = false;
        }
        if (pend_st_gate && (now - last_st_gate) >= pdMS_TO_TICKS(SHS_NVS_DEBOUNCE_MS)) {
            shs_cfg_save_u8(SHS_NVS_KEY_ST_GATE, st_gate_val);
            pend_st_gate = false;
        }
    }
}

/* ============================================================================
 * APP_MAIN
 * ============================================================================ */

void app_main(void) {
    /* Initialize NVS */
    esp_err_t nvs_rc = nvs_flash_init();
    if (nvs_rc == ESP_ERR_NVS_NO_FREE_PAGES || nvs_rc == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_rc = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_rc);

    /* Load configuration from NVS */
    shs_cfg_load_from_nvs();

    /* Create mutex for target data access */
    target_data_mutex = xSemaphoreCreateMutex();
    if (target_data_mutex == NULL) {
        ESP_LOGE(SHS_TAG, "Failed to create target data mutex");
    }

    /* Initialize light driver */
    light_driver_init(LIGHT_DEFAULT_OFF);

    /* Initialize LD2410 enhanced driver */
    ESP_ERROR_CHECK(ld2410_init());

    /* Register LD2410 callbacks */
    ld2410_register_state_callback(shs_on_state_change);

    /* Apply initial configuration to LD2410 */
    /* Wait 500ms for sensor to fully boot and stabilize data stream */
    ESP_LOGI(SHS_TAG, "Waiting for LD2410 to stabilize before config...");
    vTaskDelay(pdMS_TO_TICKS(500));
    shs_apply_ld2410_config();

    /* Initialize LD2450 */
    ESP_LOGI(SHS_TAG, "Initializing LD2450 on UART0 GPIO18/19...");
    ESP_ERROR_CHECK(ld2450_init());
    ESP_LOGI(SHS_TAG, "LD2450 initialization SUCCESS!");
    ld2450_register_target_callback(shs_on_ld2450_target_update);
    ld2450_register_zone_callback(shs_on_ld2450_zone_update);
    ESP_LOGI(SHS_TAG, "LD2450 callbacks registered");

    /* Create NVS save worker queue and task */
    shs_save_q = xQueueCreate(16, sizeof(shs_save_msg_t));
    xTaskCreate(shs_save_worker, "shs_save_worker", 3072, NULL, 3, NULL);

    /* Create tasks - Start processing BEFORE applying config
     * Sensor tasks at priority 4 (lower than Zigbee at 5) to prevent network issues */
    xTaskCreate(shs_ld2410_task, "shs_ld2410_task", 4096, NULL, 4, NULL);
    xTaskCreate(shs_ld2450_task, "shs_ld2450_task", 4096, NULL, 4, NULL);

    /* Load zone configuration from NVS */
    shs_zone_cfg_load_from_nvs();

    /* Apply zone configuration AFTER task starts - give sensor time to stabilize */
    vTaskDelay(pdMS_TO_TICKS(2000));  /* Wait 2 seconds for sensor to stabilize */
    shs_zone_cfg_apply_to_sensor();
    xTaskCreate(shs_boot_button_task, "shs_boot_button", 8192, NULL, 4, NULL);

    /* Start Zigbee task */
    xTaskCreate(shs_zigbee_task, "shs_zigbee_main", 4096, NULL, 5, NULL);

    ESP_LOGI(SHS_TAG, "SHS01 firmware started - Position reporting via Zigbee (enable in Z2M)");
}
