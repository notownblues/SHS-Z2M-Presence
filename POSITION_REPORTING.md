# LD2450 Position Reporting via Zigbee

## Overview

The firmware now supports **optional X/Y position reporting** through Zigbee, similar to the erforscht.com German project. This feature is **disabled by default** to avoid flooding your Zigbee network.

## Architecture

### Normal Operation (Position Reporting OFF - Default)
- Endpoints 1-7: Working as before
  - EP1: Light + Config
  - EP2: LD2410C Occupancy + Distance
  - EP3: LD2450 Occupancy
  - EP4: Target Count (0-3)
  - EP5-7: Zone Occupancy
- **No position data sent** - minimal Zigbee traffic

### Configuration Mode (Position Reporting ON)
- Endpoints 8-16: Position data activated
  - **Target 1**: EP8 (X), EP9 (Y), EP10 (Distance)
  - **Target 2**: EP11 (X), EP12 (Y), EP13 (Distance)
  - **Target 3**: EP14 (X), EP15 (Y), EP16 (Distance)
- All endpoints use **genAnalogInput** cluster (attribute 0x0055)
- Values in **millimeters** (mm)
- Updates sent **only when targets move**

## Enabling Position Reporting

### Method 1: Zigbee2MQTT

1. Go to device in Z2M
2. Find **"Exposes"** tab
3. Look for **"Position Reporting"** toggle
4. Set to **ON**
5. Position data will start flowing to EP8-16

### Method 2: Direct Zigbee Attribute Write

Write to:
- **Endpoint**: 1
- **Cluster**: 0xFDCD (Config cluster)
- **Attribute**: 0x0008
- **Type**: Boolean
- **Value**: `true` (enable) or `false` (disable)

## Important Notes

### Traffic Impact
- **With position reporting ON**: Expect ~100-300 messages/min depending on movement
- **With position reporting OFF**: Only ~10-20 messages/min (occupancy, zones)
- Modern Zigbee networks handle this fine, but older/congested networks may lag

### When to Enable
✅ **Enable when**:
- Configuring zones in web app
- Debugging sensor placement
- Testing target tracking
- Using third-party visualization tools

❌ **Disable when**:
- Zones are configured and working
- Only need occupancy detection
- Network is congested
- Battery-powered coordinator

### Auto-Disable Recommendation
Consider setting up an automation in Home Assistant/ioBroker to:
1. Enable position reporting
2. Wait 10 minutes (enough for zone config)
3. Auto-disable to reduce traffic

## Web App Integration

The web app will automatically use whichever data source is available:
1. **Zigbee (EP8-16)** if position_reporting is ON - uses MQTT connection
2. **HTTP polling** if WiFi config mode is active (fallback)

You don't need both - just enable position_reporting in Z2M and the web app will receive position data through your existing MQTT connection!

## Z2M External Converter Updates

You'll need to update your Z2M external converter to expose the new endpoints. I'll provide the updated converter separately.

## Coordinate System

- **X axis**: -3000mm (left) to +3000mm (right)
- **Y axis**: 0mm (sensor) to 6000mm (away from sensor)
- **Distance**: Calculated radial distance in mm

## Comparison to WiFi HTTP Polling

| Feature | Zigbee Position Reporting | WiFi HTTP Polling |
|---------|--------------------------|-------------------|
| Network | Zigbee only | Requires WiFi + Zigbee |
| UART Stability | Perfect (Zigbee only) | Corrupted (radio contention) |
| Setup | Toggle in Z2M | Press BOOT 5s, temporary |
| Update Rate | ~10 updates/sec | 1 update/sec |
| Reliability | High | Low (100% frame corruption) |
| Normal Use | Safe to leave on | Must disable after config |

**Recommendation**: Use Zigbee position reporting. WiFi mode was a failed experiment due to ESP32-C6 radio limitations.
