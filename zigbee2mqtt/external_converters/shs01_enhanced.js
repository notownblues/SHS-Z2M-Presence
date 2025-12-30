/**
 * @file shs01_enhanced.js
 * @brief Zigbee2MQTT External Converter for SHS01 Enhanced
 *
 * Features:
 * - LD2410C: Occupancy detection on EP2 with distance/energy data
 * - LD2450: Occupancy (EP3), Target count (EP4), Zone occupancy (EP5/6/7)
 * - LD2450: Position data (EP8-16) - X/Y coordinates for 3 targets (optional, toggle with position_reporting)
 *
 * Position Reporting:
 * - Enable with position_reporting toggle in Z2M
 * - Provides target1_x, target1_y, target1_distance (and target2/3)
 * - For zone configuration in web app
 * - Disable when not configuring to reduce Zigbee traffic
 *
 * Place this file in: zigbee2mqtt/data/external_converters/shs01_enhanced.js
 */

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;
const ea = exposes.access;

// Endpoints
const EP_LIGHT = 1;
const EP_LD2410C_OCC = 2;
const EP_LD2450_OCC = 3;
const EP_LD2450_TARGET_COUNT = 4;
const EP_LD2450_ZONE1 = 5;
const EP_LD2450_ZONE2 = 6;
const EP_LD2450_ZONE3 = 7;

// Position data endpoints (EP8-16) - only active when position_reporting is ON
const EP_T1_X = 8;
const EP_T1_Y = 9;
const EP_T1_DIST = 10;
const EP_T2_X = 11;
const EP_T2_Y = 12;
const EP_T2_DIST = 13;
const EP_T3_X = 14;
const EP_T3_Y = 15;
const EP_T3_DIST = 16;

// LD2410C target state endpoints (genBinaryInput - more reliable than mfr-specific attrs)
const EP_LD2410C_MOVING = 17;
const EP_LD2410C_STATIC = 18;

// Zone target count endpoints (genAnalogInput)
const EP_ZONE1_TARGETS = 19;
const EP_ZONE2_TARGETS = 20;
const EP_ZONE3_TARGETS = 21;
const EP_LD2450_ZONE4 = 22;
const EP_LD2450_ZONE5 = 23;
const EP_ZONE4_TARGETS = 24;
const EP_ZONE5_TARGETS = 25;

// Config cluster (0xFDCD on EP1)
const CLUSTER_CONFIG = 0xFDCD;
const ATTR_MOVING_COOLDOWN = 0x0001;
const ATTR_OCCUPANCY_DELAY = 0x0002;
const ATTR_MOVING_SENSITIVITY = 0x0003;
const ATTR_STATIC_SENSITIVITY = 0x0004;
const ATTR_MOVING_MAX_GATE = 0x0005;
const ATTR_STATIC_MAX_GATE = 0x0006;
const ATTR_POSITION_REPORTING = 0x0008;

// Zone configuration attributes (on Config cluster 0xFDCD)
const ATTR_ZONE1_ENABLED = 0x0021;
const ATTR_ZONE1_X1_CFG = 0x0022;
const ATTR_ZONE1_Y1_CFG = 0x0023;
const ATTR_ZONE1_X2_CFG = 0x0024;
const ATTR_ZONE1_Y2_CFG = 0x0025;
const ATTR_ZONE1_TARGETS_CFG = 0x0026;  // uint8, read-only from firmware
const ATTR_ZONE2_ENABLED = 0x0030;
const ATTR_ZONE2_X1_CFG = 0x0031;
const ATTR_ZONE2_Y1_CFG = 0x0032;
const ATTR_ZONE2_X2_CFG = 0x0033;
const ATTR_ZONE2_Y2_CFG = 0x0034;
const ATTR_ZONE2_TARGETS_CFG = 0x0035;  // uint8, read-only from firmware
const ATTR_ZONE3_ENABLED = 0x0040;
const ATTR_ZONE3_X1_CFG = 0x0041;
const ATTR_ZONE3_Y1_CFG = 0x0042;
const ATTR_ZONE3_X2_CFG = 0x0043;
const ATTR_ZONE3_Y2_CFG = 0x0044;
const ATTR_ZONE3_TARGETS_CFG = 0x0045;  // uint8, read-only from firmware

// Per-zone type attributes (0=off, 1=detection, 2=filter, 3=interference)
const ATTR_ZONE_TYPE_CFG = 0x0020;      // global zone type
const ATTR_ZONE1_TYPE_CFG = 0x0027;
const ATTR_ZONE2_TYPE_CFG = 0x0036;
const ATTR_ZONE3_TYPE_CFG = 0x0046;
const ATTR_ZONE4_TYPE_CFG = 0x0056;
const ATTR_ZONE5_TYPE_CFG = 0x0066;

// Zone 4 configuration attributes
const ATTR_ZONE4_ENABLED = 0x0050;
const ATTR_ZONE4_X1_CFG = 0x0051;
const ATTR_ZONE4_Y1_CFG = 0x0052;
const ATTR_ZONE4_X2_CFG = 0x0053;
const ATTR_ZONE4_Y2_CFG = 0x0054;
const ATTR_ZONE4_TARGETS_CFG = 0x0055;  // uint8, read-only from firmware

// Zone 5 configuration attributes
const ATTR_ZONE5_ENABLED = 0x0060;
const ATTR_ZONE5_X1_CFG = 0x0061;
const ATTR_ZONE5_Y1_CFG = 0x0062;
const ATTR_ZONE5_X2_CFG = 0x0063;
const ATTR_ZONE5_Y2_CFG = 0x0064;
const ATTR_ZONE5_TARGETS_CFG = 0x0065;  // uint8, read-only from firmware

// Manufacturer-specific attributes on Occupancy cluster (EP2) - minimal set
const ATTR_MS_MOVING = 0xF001;
const ATTR_MS_STATIC = 0xF002;

// Zone configuration storage (local to converter)
let zoneConfig = {
    zone1: { enabled: false, x1: 0, y1: 0, x2: 0, y2: 0 },
    zone2: { enabled: false, x1: 0, y1: 0, x2: 0, y2: 0 },
    zone3: { enabled: false, x1: 0, y1: 0, x2: 0, y2: 0 },
    zone4: { enabled: false, x1: 0, y1: 0, x2: 0, y2: 0 },
    zone5: { enabled: false, x1: 0, y1: 0, x2: 0, y2: 0 },
};

// Helper: check if point is in zone
function pointInZone(x, y, zone) {
    if (!zone.enabled) return false;
    const xMin = Math.min(zone.x1, zone.x2);
    const xMax = Math.max(zone.x1, zone.x2);
    const yMin = Math.min(zone.y1, zone.y2);
    const yMax = Math.max(zone.y1, zone.y2);
    return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
}

// Helper: count targets in each zone
function calculateZoneTargets(meta) {
    const result = { zone_1_targets: 0, zone_2_targets: 0, zone_3_targets: 0, zone_4_targets: 0, zone_5_targets: 0 };
    const state = meta.state || {};

    // Get target positions from state
    const targets = [
        { x: state.target1_x, y: state.target1_y },
        { x: state.target2_x, y: state.target2_y },
        { x: state.target3_x, y: state.target3_y },
    ];

    for (let i = 0; i < targets.length; i++) {
        const target = targets[i];
        if (target.x === undefined || target.y === undefined) continue;
        if (target.x === 0 && target.y === 0) continue; // Inactive target

        const inZone1 = pointInZone(target.x, target.y, zoneConfig.zone1);
        const inZone2 = pointInZone(target.x, target.y, zoneConfig.zone2);
        const inZone3 = pointInZone(target.x, target.y, zoneConfig.zone3);
        const inZone4 = pointInZone(target.x, target.y, zoneConfig.zone4);
        const inZone5 = pointInZone(target.x, target.y, zoneConfig.zone5);

        if (inZone1) result.zone_1_targets++;
        if (inZone2) result.zone_2_targets++;
        if (inZone3) result.zone_3_targets++;
        if (inZone4) result.zone_4_targets++;
        if (inZone5) result.zone_5_targets++;

        // Debug log for first target
        if (i === 0 && (inZone1 || inZone2 || inZone3 || inZone4 || inZone5)) {
            console.log(`SHS01: Target ${i+1} at (${target.x}, ${target.y}) in zones: Z1=${inZone1}, Z2=${inZone2}, Z3=${inZone3}, Z4=${inZone4}, Z5=${inZone5}`);
        }
    }

    return result;
}

const definition = {
    // Accept both old and new model IDs for compatibility
    zigbeeModel: ['SHS01', 'SHS01_Enhanced'],
    model: 'SHS01_Enhanced',
    vendor: 'SmartHomeScene',
    description: 'Enhanced Dual-Sensor Presence (LD2410C + LD2450) - Standard clusters only',

    fromZigbee: [
        fz.on_off,
        // Custom analog/binary converters BEFORE standard occupancy to prevent generic names
        {
            cluster: 'genAnalogInput',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const result = {};
                const ep = msg.endpoint.ID;
                const value = msg.data.presentValue;

                // EP4: Target count (0-3)
                if (ep === EP_LD2450_TARGET_COUNT && msg.data.hasOwnProperty('presentValue')) {
                    const count = Math.round(value);
                    result.ld2450_target_count = count;
                    // WORKAROUND: Derive occupancy from target count since EP3 occupancy cluster isn't working
                    result.occupancy_ld2450 = count > 0;
                }

                // EP8-16: Position data (X/Y/Distance for 3 targets, in mm)
                // WORKAROUND: X coordinates are transmitted with +3000 bias to avoid
                // negative float issues in Zigbee stack. Subtract 3000 to restore original.
                if (msg.data.hasOwnProperty('presentValue')) {
                    switch (ep) {
                        case EP_T1_X: result.target1_x = Math.round(value) - 3000; break;
                        case EP_T1_Y: result.target1_y = Math.round(value); break;
                        case EP_T1_DIST: result.target1_distance = Math.round(value); break;
                        case EP_T2_X: result.target2_x = Math.round(value) - 3000; break;
                        case EP_T2_Y: result.target2_y = Math.round(value); break;
                        case EP_T2_DIST: result.target2_distance = Math.round(value); break;
                        case EP_T3_X: result.target3_x = Math.round(value) - 3000; break;
                        case EP_T3_Y: result.target3_y = Math.round(value); break;
                        case EP_T3_DIST: result.target3_distance = Math.round(value); break;
                        // EP19/20/21/24/25: Zone target counts (from firmware)
                        case EP_ZONE1_TARGETS: result.zone_1_targets = Math.round(value); break;
                        case EP_ZONE2_TARGETS: result.zone_2_targets = Math.round(value); break;
                        case EP_ZONE3_TARGETS: result.zone_3_targets = Math.round(value); break;
                        case EP_ZONE4_TARGETS: result.zone_4_targets = Math.round(value); break;
                        case EP_ZONE5_TARGETS: result.zone_5_targets = Math.round(value); break;
                    }
                }

                return result;
            },
        },
        {
            cluster: 'genBinaryInput',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const result = {};
                const ep = msg.endpoint.ID;
                // Handle various data formats for presentValue
                const pv = msg.data.presentValue;
                const value = pv === 1 || pv === true || pv === '1' || pv === 'true';

                // EP5/6/7/22/23: Zone occupancy (LD2450 only)
                if (ep === EP_LD2450_ZONE1 && msg.data.hasOwnProperty('presentValue')) {
                    result.zone1_occupied = value;
                }
                if (ep === EP_LD2450_ZONE2 && msg.data.hasOwnProperty('presentValue')) {
                    result.zone2_occupied = value;
                }
                if (ep === EP_LD2450_ZONE3 && msg.data.hasOwnProperty('presentValue')) {
                    result.zone3_occupied = value;
                }
                if (ep === EP_LD2450_ZONE4 && msg.data.hasOwnProperty('presentValue')) {
                    result.zone4_occupied = value;
                }
                if (ep === EP_LD2450_ZONE5 && msg.data.hasOwnProperty('presentValue')) {
                    result.zone5_occupied = value;
                }

                // NOTE: LD2410C uses ONLY msOccupancySensing on EP2 (like backup)
                // EP17/EP18 genBinaryInput is NOT used for LD2410C

                return result;
            },
        },
        // LD2410C occupancy and target states from msOccupancySensing (EP2 ONLY)
        {
            cluster: 'msOccupancySensing',
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const ep = msg.endpoint.ID;
                const d = msg.data || {};
                const result = {};

                // EP2 = LD2410C occupancy
                if (ep === EP_LD2410C_OCC) {
                    // Standard occupancy attribute (try both formats)
                    const occ = d.occupancy ?? d['0'];
                    if (occ !== undefined) {
                        result.occupancy_ld2410 = (typeof occ === 'number') ? ((occ & 1) === 1) : !!occ;
                    }
                    // Manufacturer-specific moving/static (try numeric and string keys like backup)
                    const mv = d[ATTR_MS_MOVING] ?? d[String(ATTR_MS_MOVING)];
                    const st = d[ATTR_MS_STATIC] ?? d[String(ATTR_MS_STATIC)];
                    if (mv !== undefined) {
                        result.moving_target = (mv === true || mv === 1);
                    }
                    if (st !== undefined) {
                        result.static_target = (st === true || st === 1);
                    }
                }

                // EP3 = LD2450 occupancy
                if (ep === EP_LD2450_OCC) {
                    if (msg.data.hasOwnProperty('occupancy')) {
                        result.occupancy_ld2450 = (msg.data.occupancy & 1) === 1;
                    }
                }

                return result;
            },
        },
        {
            cluster: CLUSTER_CONFIG,
            type: ['attributeReport', 'readResponse'],
            convert: (model, msg, publish, options, meta) => {
                const result = {};
                if (msg.data.hasOwnProperty(ATTR_MOVING_COOLDOWN)) {
                    result.moving_cooldown = msg.data[ATTR_MOVING_COOLDOWN];
                }
                if (msg.data.hasOwnProperty(ATTR_OCCUPANCY_DELAY)) {
                    result.occupancy_delay = msg.data[ATTR_OCCUPANCY_DELAY];
                }
                if (msg.data.hasOwnProperty(ATTR_MOVING_SENSITIVITY)) {
                    result.moving_sensitivity = msg.data[ATTR_MOVING_SENSITIVITY];
                }
                if (msg.data.hasOwnProperty(ATTR_STATIC_SENSITIVITY)) {
                    result.static_sensitivity = msg.data[ATTR_STATIC_SENSITIVITY];
                }
                if (msg.data.hasOwnProperty(ATTR_MOVING_MAX_GATE)) {
                    // Convert gates to meters (1 gate = 0.75m)
                    result.moving_max_distance = msg.data[ATTR_MOVING_MAX_GATE] * 0.75;
                }
                if (msg.data.hasOwnProperty(ATTR_STATIC_MAX_GATE)) {
                    // Convert gates to meters (1 gate = 0.75m)
                    result.static_max_distance = msg.data[ATTR_STATIC_MAX_GATE] * 0.75;
                }
                if (msg.data.hasOwnProperty(ATTR_POSITION_REPORTING)) {
                    result.position_reporting = msg.data[ATTR_POSITION_REPORTING] === 1 || msg.data[ATTR_POSITION_REPORTING] === true;
                }
                // Zone target counts from firmware (these are read-only, calculated by firmware)
                if (msg.data.hasOwnProperty(ATTR_ZONE1_TARGETS_CFG)) {
                    result.zone_1_targets = msg.data[ATTR_ZONE1_TARGETS_CFG];
                    console.log(`SHS01: Zone 1 targets = ${result.zone_1_targets}`);
                }
                if (msg.data.hasOwnProperty(ATTR_ZONE2_TARGETS_CFG)) {
                    result.zone_2_targets = msg.data[ATTR_ZONE2_TARGETS_CFG];
                    console.log(`SHS01: Zone 2 targets = ${result.zone_2_targets}`);
                }
                if (msg.data.hasOwnProperty(ATTR_ZONE3_TARGETS_CFG)) {
                    result.zone_3_targets = msg.data[ATTR_ZONE3_TARGETS_CFG];
                    console.log(`SHS01: Zone 3 targets = ${result.zone_3_targets}`);
                }
                if (msg.data.hasOwnProperty(ATTR_ZONE4_TARGETS_CFG)) {
                    result.zone_4_targets = msg.data[ATTR_ZONE4_TARGETS_CFG];
                    console.log(`SHS01: Zone 4 targets = ${result.zone_4_targets}`);
                }
                if (msg.data.hasOwnProperty(ATTR_ZONE5_TARGETS_CFG)) {
                    result.zone_5_targets = msg.data[ATTR_ZONE5_TARGETS_CFG];
                    console.log(`SHS01: Zone 5 targets = ${result.zone_5_targets}`);
                }
                // Debug: log all received config cluster attributes
                if (Object.keys(result).length === 0) {
                    console.log(`SHS01: Config cluster data received:`, JSON.stringify(msg.data));
                }
                return result;
            },
        },
    ],

    toZigbee: [
        tz.on_off,
        {
            key: ['moving_cooldown', 'occupancy_delay', 'moving_sensitivity', 'static_sensitivity',
                  'moving_max_distance', 'static_max_distance', 'position_reporting'],
            convertSet: async (entity, key, value, meta) => {
                const endpoint = meta.device.getEndpoint(EP_LIGHT);
                if (!endpoint) {
                    console.log(`SHS01: Cannot get endpoint ${EP_LIGHT} for ${key}`);
                    throw new Error(`Endpoint ${EP_LIGHT} not found`);
                }

                // Attribute lookup with data types (0x21 = uint16, 0x10 = bool)
                const lookup = {
                    'moving_cooldown': {id: ATTR_MOVING_COOLDOWN, type: 0x21},
                    'occupancy_delay': {id: ATTR_OCCUPANCY_DELAY, type: 0x21},
                    'moving_sensitivity': {id: ATTR_MOVING_SENSITIVITY, type: 0x21},
                    'static_sensitivity': {id: ATTR_STATIC_SENSITIVITY, type: 0x21},
                    'moving_max_distance': {id: ATTR_MOVING_MAX_GATE, type: 0x21},
                    'static_max_distance': {id: ATTR_STATIC_MAX_GATE, type: 0x21},
                    'position_reporting': {id: ATTR_POSITION_REPORTING, type: 0x10},
                };

                const attr = lookup[key];
                let writeValue = value;

                // Convert meters to gates for distance settings (1 gate = 0.75m)
                if (key === 'moving_max_distance' || key === 'static_max_distance') {
                    writeValue = Math.round(value / 0.75);
                    // Clamp to valid gate range
                    if (key === 'moving_max_distance') {
                        writeValue = Math.max(0, Math.min(8, writeValue));
                    } else {
                        writeValue = Math.max(2, Math.min(8, writeValue));
                    }
                }

                // Convert boolean to number for position_reporting (Zigbee expects 0/1)
                if (key === 'position_reporting') {
                    writeValue = value ? 1 : 0;
                    console.log(`SHS01: Setting position_reporting = ${writeValue} (was: ${value})`);
                }

                // Write with explicit type for manufacturer-specific cluster
                try {
                    console.log(`SHS01: Writing ${key} = ${writeValue} to cluster 0x${CLUSTER_CONFIG.toString(16)}, attr 0x${attr.id.toString(16)}`);
                    await endpoint.write(CLUSTER_CONFIG, {[attr.id]: {value: writeValue, type: attr.type}});
                    console.log(`SHS01: Successfully wrote ${key} = ${writeValue}`);
                } catch (e) {
                    console.log(`SHS01: Failed to write ${key}:`, e.message);
                    throw e;
                }
                return {state: {[key]: value}};
            },
            convertGet: async (entity, key, meta) => {
                const endpoint = meta.device.getEndpoint(EP_LIGHT);
                if (!endpoint) {
                    console.log(`SHS01: Cannot get endpoint ${EP_LIGHT} for reading ${key}`);
                    return;
                }

                const lookup = {
                    'moving_cooldown': ATTR_MOVING_COOLDOWN,
                    'occupancy_delay': ATTR_OCCUPANCY_DELAY,
                    'moving_sensitivity': ATTR_MOVING_SENSITIVITY,
                    'static_sensitivity': ATTR_STATIC_SENSITIVITY,
                    'moving_max_distance': ATTR_MOVING_MAX_GATE,
                    'static_max_distance': ATTR_STATIC_MAX_GATE,
                    'position_reporting': ATTR_POSITION_REPORTING,
                };

                const attrId = lookup[key];
                if (attrId !== undefined) {
                    try {
                        console.log(`SHS01: Reading ${key} from cluster 0x${CLUSTER_CONFIG.toString(16)}, attr 0x${attrId.toString(16)}`);
                        await endpoint.read(CLUSTER_CONFIG, [attrId]);
                    } catch (e) {
                        console.log(`SHS01: Failed to read ${key}:`, e.message);
                    }
                }
            },
        },
        // Zone configuration handler - receives zone_config object from web configurator
        // Writes all zone attributes to firmware via Zigbee cluster
        {
            key: ['zone_config'],
            convertSet: async (entity, key, value, meta) => {
                console.log(`SHS01 ZONE: Received zone_config:`, JSON.stringify(value));
                const endpoint = meta.device.getEndpoint(EP_LIGHT);
                if (!endpoint) {
                    console.log(`SHS01 ZONE: ERROR - endpoint ${EP_LIGHT} not found!`);
                    throw new Error(`Endpoint ${EP_LIGHT} not found`);
                }

                // Attribute lookup with data types (0x10 = bool, 0x20 = uint8, 0x29 = int16)
                const zoneAttrLookup = {
                    'zone_type': {id: ATTR_ZONE_TYPE_CFG, type: 0x20, isZoneType: true},      // uint8 (global)
                    'zone1_type': {id: ATTR_ZONE1_TYPE_CFG, type: 0x20, isZoneType: true},    // uint8 (per-zone)
                    'zone1_enabled': {id: ATTR_ZONE1_ENABLED, type: 0x10},       // bool
                    'zone1_x1': {id: ATTR_ZONE1_X1_CFG, type: 0x29},             // int16
                    'zone1_y1': {id: ATTR_ZONE1_Y1_CFG, type: 0x29},             // int16
                    'zone1_x2': {id: ATTR_ZONE1_X2_CFG, type: 0x29},             // int16
                    'zone1_y2': {id: ATTR_ZONE1_Y2_CFG, type: 0x29},             // int16
                    'zone2_type': {id: ATTR_ZONE2_TYPE_CFG, type: 0x20, isZoneType: true},    // uint8 (per-zone)
                    'zone2_enabled': {id: ATTR_ZONE2_ENABLED, type: 0x10},       // bool
                    'zone2_x1': {id: ATTR_ZONE2_X1_CFG, type: 0x29},             // int16
                    'zone2_y1': {id: ATTR_ZONE2_Y1_CFG, type: 0x29},             // int16
                    'zone2_x2': {id: ATTR_ZONE2_X2_CFG, type: 0x29},             // int16
                    'zone2_y2': {id: ATTR_ZONE2_Y2_CFG, type: 0x29},             // int16
                    'zone3_type': {id: ATTR_ZONE3_TYPE_CFG, type: 0x20, isZoneType: true},    // uint8 (per-zone)
                    'zone3_enabled': {id: ATTR_ZONE3_ENABLED, type: 0x10},       // bool
                    'zone3_x1': {id: ATTR_ZONE3_X1_CFG, type: 0x29},             // int16
                    'zone3_y1': {id: ATTR_ZONE3_Y1_CFG, type: 0x29},             // int16
                    'zone3_x2': {id: ATTR_ZONE3_X2_CFG, type: 0x29},             // int16
                    'zone3_y2': {id: ATTR_ZONE3_Y2_CFG, type: 0x29},             // int16
                    'zone4_type': {id: ATTR_ZONE4_TYPE_CFG, type: 0x20, isZoneType: true},    // uint8 (per-zone)
                    'zone4_enabled': {id: ATTR_ZONE4_ENABLED, type: 0x10},       // bool
                    'zone4_x1': {id: ATTR_ZONE4_X1_CFG, type: 0x29},             // int16
                    'zone4_y1': {id: ATTR_ZONE4_Y1_CFG, type: 0x29},             // int16
                    'zone4_x2': {id: ATTR_ZONE4_X2_CFG, type: 0x29},             // int16
                    'zone4_y2': {id: ATTR_ZONE4_Y2_CFG, type: 0x29},             // int16
                    'zone5_type': {id: ATTR_ZONE5_TYPE_CFG, type: 0x20, isZoneType: true},    // uint8 (per-zone)
                    'zone5_enabled': {id: ATTR_ZONE5_ENABLED, type: 0x10},       // bool
                    'zone5_x1': {id: ATTR_ZONE5_X1_CFG, type: 0x29},             // int16
                    'zone5_y1': {id: ATTR_ZONE5_Y1_CFG, type: 0x29},             // int16
                    'zone5_x2': {id: ATTR_ZONE5_X2_CFG, type: 0x29},             // int16
                    'zone5_y2': {id: ATTR_ZONE5_Y2_CFG, type: 0x29},             // int16
                };

                // Zone type string to number mapping
                const zoneTypeMapping = {
                    'off': 0,
                    'detection': 1,
                    'filter': 2,
                    'interference': 3,
                };

                // Write each attribute from the zone_config object
                for (const [attrKey, attrDef] of Object.entries(zoneAttrLookup)) {
                    if (value.hasOwnProperty(attrKey)) {
                        let writeValue = value[attrKey];
                        // Convert zone type string to number
                        if (attrDef.isZoneType && typeof writeValue === 'string') {
                            writeValue = zoneTypeMapping[writeValue] ?? 0;
                        }
                        // Convert boolean to 0/1 for Zigbee
                        if (attrDef.type === 0x10) {
                            writeValue = writeValue ? 1 : 0;
                        }
                        try {
                            await endpoint.write(CLUSTER_CONFIG, {[attrDef.id]: {value: writeValue, type: attrDef.type}});
                            console.log(`SHS01 ZONE: Written ${attrKey} = ${writeValue}`);
                        } catch (e) {
                            console.log(`SHS01 ZONE: Failed to write ${attrKey}:`, e.message);
                        }
                    }
                }

                // Update local zone config for JS-side tracking
                if (value.zone1_enabled !== undefined) zoneConfig.zone1.enabled = value.zone1_enabled;
                if (value.zone1_x1 !== undefined) zoneConfig.zone1.x1 = value.zone1_x1;
                if (value.zone1_y1 !== undefined) zoneConfig.zone1.y1 = value.zone1_y1;
                if (value.zone1_x2 !== undefined) zoneConfig.zone1.x2 = value.zone1_x2;
                if (value.zone1_y2 !== undefined) zoneConfig.zone1.y2 = value.zone1_y2;
                if (value.zone2_enabled !== undefined) zoneConfig.zone2.enabled = value.zone2_enabled;
                if (value.zone2_x1 !== undefined) zoneConfig.zone2.x1 = value.zone2_x1;
                if (value.zone2_y1 !== undefined) zoneConfig.zone2.y1 = value.zone2_y1;
                if (value.zone2_x2 !== undefined) zoneConfig.zone2.x2 = value.zone2_x2;
                if (value.zone2_y2 !== undefined) zoneConfig.zone2.y2 = value.zone2_y2;
                if (value.zone3_enabled !== undefined) zoneConfig.zone3.enabled = value.zone3_enabled;
                if (value.zone3_x1 !== undefined) zoneConfig.zone3.x1 = value.zone3_x1;
                if (value.zone3_y1 !== undefined) zoneConfig.zone3.y1 = value.zone3_y1;
                if (value.zone3_x2 !== undefined) zoneConfig.zone3.x2 = value.zone3_x2;
                if (value.zone3_y2 !== undefined) zoneConfig.zone3.y2 = value.zone3_y2;
                if (value.zone4_enabled !== undefined) zoneConfig.zone4.enabled = value.zone4_enabled;
                if (value.zone4_x1 !== undefined) zoneConfig.zone4.x1 = value.zone4_x1;
                if (value.zone4_y1 !== undefined) zoneConfig.zone4.y1 = value.zone4_y1;
                if (value.zone4_x2 !== undefined) zoneConfig.zone4.x2 = value.zone4_x2;
                if (value.zone4_y2 !== undefined) zoneConfig.zone4.y2 = value.zone4_y2;
                if (value.zone5_enabled !== undefined) zoneConfig.zone5.enabled = value.zone5_enabled;
                if (value.zone5_x1 !== undefined) zoneConfig.zone5.x1 = value.zone5_x1;
                if (value.zone5_y1 !== undefined) zoneConfig.zone5.y1 = value.zone5_y1;
                if (value.zone5_x2 !== undefined) zoneConfig.zone5.x2 = value.zone5_x2;
                if (value.zone5_y2 !== undefined) zoneConfig.zone5.y2 = value.zone5_y2;

                console.log(`SHS01 ZONE: Config complete`);
                return {state: {zone_config: value}};
            },
        },
    ],

    exposes: [
        // EP1: Light switch
        e.switch().withEndpoint('l1'),

        // EP2: LD2410C Occupancy
        e.binary('occupancy_ld2410', ea.STATE, true, false)
            .withDescription('LD2410C occupancy (moving OR static)'),

        // LD2410C Target States
        e.binary('moving_target', ea.STATE, true, false)
            .withDescription('LD2410C moving target detected'),
        e.binary('static_target', ea.STATE, true, false)
            .withDescription('LD2410C static target detected'),

        // EP3: LD2450 Occupancy
        e.binary('occupancy_ld2450', ea.STATE, true, false)
            .withDescription('LD2450 occupancy (any target detected)'),

        // EP4: LD2450 Target Count
        exposes.numeric('ld2450_target_count', ea.STATE)
            .withValueMin(0)
            .withValueMax(3)
            .withDescription('Number of targets detected by LD2450 (0-3)'),

        // EP5/6/7/22/23: Zone Occupancy
        e.binary('zone1_occupied', ea.STATE, true, false)
            .withDescription('Zone 1 occupancy'),
        e.binary('zone2_occupied', ea.STATE, true, false)
            .withDescription('Zone 2 occupancy'),
        e.binary('zone3_occupied', ea.STATE, true, false)
            .withDescription('Zone 3 occupancy'),
        e.binary('zone4_occupied', ea.STATE, true, false)
            .withDescription('Zone 4 occupancy'),
        e.binary('zone5_occupied', ea.STATE, true, false)
            .withDescription('Zone 5 occupancy'),

        // Zone Target Counts (derived from position data when position_reporting is ON)
        exposes.numeric('zone_1_targets', ea.STATE)
            .withValueMin(0)
            .withValueMax(3)
            .withDescription('Number of targets in Zone 1 (0-3)'),
        exposes.numeric('zone_2_targets', ea.STATE)
            .withValueMin(0)
            .withValueMax(3)
            .withDescription('Number of targets in Zone 2 (0-3)'),
        exposes.numeric('zone_3_targets', ea.STATE)
            .withValueMin(0)
            .withValueMax(3)
            .withDescription('Number of targets in Zone 3 (0-3)'),
        exposes.numeric('zone_4_targets', ea.STATE)
            .withValueMin(0)
            .withValueMax(3)
            .withDescription('Number of targets in Zone 4 (0-3)'),
        exposes.numeric('zone_5_targets', ea.STATE)
            .withValueMin(0)
            .withValueMax(3)
            .withDescription('Number of targets in Zone 5 (0-3)'),

        // EP8-16: Position Data (only active when position_reporting is ON)
        // Target 1
        exposes.numeric('target1_x', ea.STATE)
            .withUnit('mm')
            .withDescription('Target 1 X position (-3000 to +3000)'),
        exposes.numeric('target1_y', ea.STATE)
            .withUnit('mm')
            .withDescription('Target 1 Y position (0 to 6000)'),
        exposes.numeric('target1_distance', ea.STATE)
            .withUnit('mm')
            .withDescription('Target 1 distance from sensor'),
        // Target 2
        exposes.numeric('target2_x', ea.STATE)
            .withUnit('mm')
            .withDescription('Target 2 X position (-3000 to +3000)'),
        exposes.numeric('target2_y', ea.STATE)
            .withUnit('mm')
            .withDescription('Target 2 Y position (0 to 6000)'),
        exposes.numeric('target2_distance', ea.STATE)
            .withUnit('mm')
            .withDescription('Target 2 distance from sensor'),
        // Target 3
        exposes.numeric('target3_x', ea.STATE)
            .withUnit('mm')
            .withDescription('Target 3 X position (-3000 to +3000)'),
        exposes.numeric('target3_y', ea.STATE)
            .withUnit('mm')
            .withDescription('Target 3 Y position (0 to 6000)'),
        exposes.numeric('target3_distance', ea.STATE)
            .withUnit('mm')
            .withDescription('Target 3 distance from sensor'),

        // Configuration
        exposes.numeric('moving_cooldown', ea.ALL)
            .withValueMin(0)
            .withValueMax(300)
            .withUnit('s')
            .withDescription('Movement detection cooldown'),
        exposes.numeric('occupancy_delay', ea.ALL)
            .withValueMin(0)
            .withValueMax(300)
            .withUnit('s')
            .withDescription('Occupancy clear delay'),
        exposes.numeric('moving_sensitivity', ea.ALL)
            .withValueMin(0)
            .withValueMax(10)
            .withDescription('Moving target sensitivity (0-10)'),
        exposes.numeric('static_sensitivity', ea.ALL)
            .withValueMin(0)
            .withValueMax(10)
            .withDescription('Static target sensitivity (0-10)'),
        exposes.numeric('moving_max_distance', ea.ALL)
            .withValueMin(0)
            .withValueMax(6)
            .withValueStep(0.75)
            .withUnit('m')
            .withDescription('Max distance for moving detection (0-6m)'),
        exposes.numeric('static_max_distance', ea.ALL)
            .withValueMin(1.5)
            .withValueMax(6)
            .withValueStep(0.75)
            .withUnit('m')
            .withDescription('Max distance for static detection (1.5-6m)'),
        exposes.binary('position_reporting', ea.ALL, true, false)
            .withDescription('Enable position reporting for zone configuration (increases Zigbee traffic)'),

        // Zone configuration - composite object that accepts all zone settings at once
        exposes.composite('zone_config', 'zone_config', ea.SET)
            .withDescription('Zone configuration object from web configurator')
            .withFeature(exposes.binary('zone1_enabled', ea.SET, true, false))
            .withFeature(exposes.numeric('zone1_x1', ea.SET))
            .withFeature(exposes.numeric('zone1_y1', ea.SET))
            .withFeature(exposes.numeric('zone1_x2', ea.SET))
            .withFeature(exposes.numeric('zone1_y2', ea.SET))
            .withFeature(exposes.binary('zone2_enabled', ea.SET, true, false))
            .withFeature(exposes.numeric('zone2_x1', ea.SET))
            .withFeature(exposes.numeric('zone2_y1', ea.SET))
            .withFeature(exposes.numeric('zone2_x2', ea.SET))
            .withFeature(exposes.numeric('zone2_y2', ea.SET))
            .withFeature(exposes.binary('zone3_enabled', ea.SET, true, false))
            .withFeature(exposes.numeric('zone3_x1', ea.SET))
            .withFeature(exposes.numeric('zone3_y1', ea.SET))
            .withFeature(exposes.numeric('zone3_x2', ea.SET))
            .withFeature(exposes.numeric('zone3_y2', ea.SET))
            .withFeature(exposes.binary('zone4_enabled', ea.SET, true, false))
            .withFeature(exposes.numeric('zone4_x1', ea.SET))
            .withFeature(exposes.numeric('zone4_y1', ea.SET))
            .withFeature(exposes.numeric('zone4_x2', ea.SET))
            .withFeature(exposes.numeric('zone4_y2', ea.SET))
            .withFeature(exposes.binary('zone5_enabled', ea.SET, true, false))
            .withFeature(exposes.numeric('zone5_x1', ea.SET))
            .withFeature(exposes.numeric('zone5_y1', ea.SET))
            .withFeature(exposes.numeric('zone5_x2', ea.SET))
            .withFeature(exposes.numeric('zone5_y2', ea.SET)),
    ],

    meta: {
        multiEndpoint: true,
    },

    endpoint: (device) => {
        return {
            'l1': EP_LIGHT,
            'ld2410c': EP_LD2410C_OCC,
            'ld2450': EP_LD2450_OCC,
        };
    },

    configure: async (device, coordinatorEndpoint) => {
        const endpoint1 = device.getEndpoint(EP_LIGHT);
        const endpoint2 = device.getEndpoint(EP_LD2410C_OCC);
        const endpoint3 = device.getEndpoint(EP_LD2450_OCC);
        const endpoint4 = device.getEndpoint(EP_LD2450_TARGET_COUNT);
        const endpoint5 = device.getEndpoint(EP_LD2450_ZONE1);
        const endpoint6 = device.getEndpoint(EP_LD2450_ZONE2);
        const endpoint7 = device.getEndpoint(EP_LD2450_ZONE3);
        const endpoint17 = device.getEndpoint(EP_LD2410C_MOVING);
        const endpoint18 = device.getEndpoint(EP_LD2410C_STATIC);
        const endpoint19 = device.getEndpoint(EP_ZONE1_TARGETS);
        const endpoint20 = device.getEndpoint(EP_ZONE2_TARGETS);
        const endpoint21 = device.getEndpoint(EP_ZONE3_TARGETS);
        const endpoint22 = device.getEndpoint(EP_LD2450_ZONE4);
        const endpoint23 = device.getEndpoint(EP_LD2450_ZONE5);
        const endpoint24 = device.getEndpoint(EP_ZONE4_TARGETS);
        const endpoint25 = device.getEndpoint(EP_ZONE5_TARGETS);

        // Position data endpoints (EP8-16)
        const positionEndpoints = [];
        for (let ep = EP_T1_X; ep <= EP_T3_DIST; ep++) {
            const endpoint = device.getEndpoint(ep);
            if (endpoint) positionEndpoints.push(endpoint);
        }

        // Bind standard clusters
        await reporting.bind(endpoint1, coordinatorEndpoint, ['genOnOff']);
        await reporting.bind(endpoint2, coordinatorEndpoint, ['msOccupancySensing']);
        await reporting.bind(endpoint3, coordinatorEndpoint, ['msOccupancySensing']);

        // Bind config cluster for zone target count reports
        try {
            await endpoint1.bind(CLUSTER_CONFIG, coordinatorEndpoint);
            console.log('SHS01: Bound config cluster (0xFDCD) for zone target reports');
        } catch (e) {
            console.log('SHS01: Failed to bind config cluster:', e.message);
        }

        // Bind LD2410C moving/static target endpoints (EP17/18)
        if (endpoint17) await reporting.bind(endpoint17, coordinatorEndpoint, ['genBinaryInput']);
        if (endpoint18) await reporting.bind(endpoint18, coordinatorEndpoint, ['genBinaryInput']);

        // Bind LD2450 target count (genAnalogInput)
        await reporting.bind(endpoint4, coordinatorEndpoint, ['genAnalogInput']);

        // Bind LD2450 zones (genBinaryInput)
        await reporting.bind(endpoint5, coordinatorEndpoint, ['genBinaryInput']);
        await reporting.bind(endpoint6, coordinatorEndpoint, ['genBinaryInput']);
        await reporting.bind(endpoint7, coordinatorEndpoint, ['genBinaryInput']);
        if (endpoint22) await reporting.bind(endpoint22, coordinatorEndpoint, ['genBinaryInput']);
        if (endpoint23) await reporting.bind(endpoint23, coordinatorEndpoint, ['genBinaryInput']);

        // Bind zone target count endpoints (EP19/20/21/24/25 - genAnalogInput)
        if (endpoint19) await reporting.bind(endpoint19, coordinatorEndpoint, ['genAnalogInput']);
        if (endpoint20) await reporting.bind(endpoint20, coordinatorEndpoint, ['genAnalogInput']);
        if (endpoint21) await reporting.bind(endpoint21, coordinatorEndpoint, ['genAnalogInput']);
        if (endpoint24) await reporting.bind(endpoint24, coordinatorEndpoint, ['genAnalogInput']);
        if (endpoint25) await reporting.bind(endpoint25, coordinatorEndpoint, ['genAnalogInput']);

        // Bind position data endpoints (EP8-16) - genAnalogInput
        for (const ep of positionEndpoints) {
            try {
                await reporting.bind(ep, coordinatorEndpoint, ['genAnalogInput']);
            } catch (e) {
                console.log(`Failed to bind position endpoint ${ep.ID}:`, e);
            }
        }

        // Configure reporting for LD2410C occupancy (EP2) - like backup
        try {
            await endpoint2.configureReporting('msOccupancySensing', [{
                attribute: {ID: 0x0000, type: 0x18},  // Standard occupancy bitmap
                minimumReportInterval: 0,
                maximumReportInterval: 3600,
                reportableChange: 0,
            }]);
        } catch (e) {
            console.log('Failed to configure LD2410C occupancy reporting:', e);
        }

        // Configure reporting for LD2410C manufacturer-specific attrs (0xF001, 0xF002) - like backup
        const BOOL_DT = 0x10;  // ZCL Boolean data type
        const repCustom = [
            {attribute: ATTR_MS_MOVING, minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 1, dataType: BOOL_DT},
            {attribute: ATTR_MS_STATIC, minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 1, dataType: BOOL_DT},
        ];
        try {
            await endpoint2.configureReporting('msOccupancySensing', repCustom, {manufacturerCode: 0x115F});
        } catch (e) {
            try {
                await endpoint2.configureReporting('msOccupancySensing', repCustom);
            } catch (e2) {
                console.log('Failed to configure LD2410C mfr-specific reporting:', e2);
            }
        }

        // Configure reporting for target count (only on change)
        await endpoint4.configureReporting('genAnalogInput', [{
            attribute: 'presentValue',
            minimumReportInterval: 1,
            maximumReportInterval: 3600,
            reportableChange: 0.5,  // Report when count changes by 0.5 (i.e., any integer change)
        }]);

        // Configure reporting for zone target counts (EP19/20/21/24/25)
        for (const ep of [endpoint19, endpoint20, endpoint21, endpoint24, endpoint25]) {
            if (ep) {
                try {
                    await ep.configureReporting('genAnalogInput', [{
                        attribute: 'presentValue',
                        minimumReportInterval: 0,  // Report immediately
                        maximumReportInterval: 3600,
                        reportableChange: 0.5,  // Report when count changes
                    }]);
                } catch (e) {
                    console.log(`Failed to configure zone target reporting for EP${ep.ID}:`, e);
                }
            }
        }

        // Configure reporting for zone occupancy (only on change)
        for (const ep of [endpoint5, endpoint6, endpoint7, endpoint22, endpoint23]) {
            if (ep) {
                await ep.configureReporting('genBinaryInput', [{
                    attribute: 'presentValue',
                    minimumReportInterval: 1,
                    maximumReportInterval: 3600,
                    reportableChange: 1,  // Report on state change
                }]);
            }
        }

        // Configure reporting for LD2410C moving/static target states (EP17/18)
        for (const ep of [endpoint17, endpoint18]) {
            if (ep) {
                try {
                    await ep.configureReporting('genBinaryInput', [{
                        attribute: 'presentValue',
                        minimumReportInterval: 0,  // Report immediately
                        maximumReportInterval: 3600,
                        reportableChange: 1,  // Report on state change
                    }]);
                } catch (e) {
                    console.log(`Failed to configure LD2410C target reporting for EP${ep.ID}:`, e);
                }
            }
        }

        // Configure reporting for position data (EP8-16) - report on any change
        for (const ep of positionEndpoints) {
            try {
                await ep.configureReporting('genAnalogInput', [{
                    attribute: 'presentValue',
                    minimumReportInterval: 0,  // Report immediately on change
                    maximumReportInterval: 3600,
                    reportableChange: 1,  // Report when value changes by 1mm
                }]);
            } catch (e) {
                console.log(`Failed to configure position reporting for EP${ep.ID}:`, e);
            }
        }

        // Read initial config
        try {
            await endpoint1.read(CLUSTER_CONFIG, [
                ATTR_MOVING_COOLDOWN,
                ATTR_OCCUPANCY_DELAY,
                ATTR_MOVING_SENSITIVITY,
                ATTR_STATIC_SENSITIVITY,
                ATTR_MOVING_MAX_GATE,
                ATTR_STATIC_MAX_GATE,
                ATTR_POSITION_REPORTING,
            ]);
        } catch (e) {
            console.log('Failed to read config:', e);
        }

        // Read zone target counts
        try {
            await endpoint1.read(CLUSTER_CONFIG, [
                ATTR_ZONE1_TARGETS_CFG,
                ATTR_ZONE2_TARGETS_CFG,
                ATTR_ZONE3_TARGETS_CFG,
                ATTR_ZONE4_TARGETS_CFG,
                ATTR_ZONE5_TARGETS_CFG,
            ]);
        } catch (e) {
            console.log('Failed to read zone target counts:', e);
        }

        // Read initial LD2410C occupancy and target states (EP2)
        try {
            await endpoint2.read('msOccupancySensing', ['occupancy']);
        } catch (e) {
            console.log('Failed to read LD2410C occupancy:', e);
        }
        try {
            await endpoint2.read('msOccupancySensing', [
                ATTR_MS_MOVING,
                ATTR_MS_STATIC,
            ]);
        } catch (e) {
            console.log('Failed to read LD2410C target states:', e);
        }

        // Read initial LD2450 occupancy (EP3)
        try {
            await endpoint3.read('msOccupancySensing', ['occupancy']);
        } catch (e) {
            console.log('Failed to read LD2450 occupancy:', e);
        }

        // Read initial LD2450 target count (EP4)
        try {
            await endpoint4.read('genAnalogInput', ['presentValue']);
        } catch (e) {
            console.log('Failed to read LD2450 target count:', e);
        }

        // Read initial zone occupancy values (EP5/6/7/22/23)
        try {
            await endpoint5.read('genBinaryInput', ['presentValue']);
            await endpoint6.read('genBinaryInput', ['presentValue']);
            await endpoint7.read('genBinaryInput', ['presentValue']);
            if (endpoint22) await endpoint22.read('genBinaryInput', ['presentValue']);
            if (endpoint23) await endpoint23.read('genBinaryInput', ['presentValue']);
        } catch (e) {
            console.log('Failed to read zone occupancy:', e);
        }

        // Read initial LD2410C moving/static target states (EP17/18 - reliable genBinaryInput)
        try {
            if (endpoint17) await endpoint17.read('genBinaryInput', ['presentValue']);
            if (endpoint18) await endpoint18.read('genBinaryInput', ['presentValue']);
        } catch (e) {
            console.log('Failed to read LD2410C target states:', e);
        }
    },
};

module.exports = definition;
