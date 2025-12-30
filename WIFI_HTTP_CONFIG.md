# WiFi Configuration Mode (On-Demand Only)

The firmware includes **WiFi Configuration Mode** for real-time position data during zone setup. **WiFi is DISABLED by default** to prevent UART corruption from WiFi/Zigbee interference.

## Architecture

### Normal Operation (WiFi OFF)
**Zigbee Only** - via Zigbee2MQTT:
- Occupancy state (on/off)
- Target count (0-3)
- Zone occupancy (zone1, zone2, zone3)
- **UART fully stable** - no corruption
- **Low power** - Zigbee End Device

### Configuration Mode (WiFi ON - Manual Activation)
**HTTP Polling** - Direct to ESP32:
- Real-time target positions (X, Y coordinates)
- Distance and speed for each target
- Web app polls every 1000ms (1 update/sec)
- **Only for zone configuration** - not for normal use
- Auto-disables after 10 minutes
- WARNING: UART may be unreliable during WiFi operation

## Enabling WiFi Configuration Mode

**WiFi is OFF by default.** To enable it for zone configuration:

### Method: Boot Button Long Press (5 seconds)

1. **Press and hold BOOT button** for **5 seconds**
2. **LED turns ON** to indicate WiFi config mode is active
3. WiFi connects to network (credentials configured in firmware)
4. HTTP server starts on port 80
5. **Auto-disables after 10 minutes** or press BOOT for 5s again to disable

### Button Functions:
- **5 seconds**: Toggle WiFi Configuration Mode (LED indicates status)
- **6 seconds**: Factory reset (clears Zigbee pairing and settings)

## Why WiFi is Disabled by Default

**Problem:** ESP32-C6 has ONE 2.4GHz radio shared between Zigbee and WiFi. Running both simultaneously causes severe UART corruption:
- LD2450 data becomes garbage (X=26624, Y=-7424, Speed=27138)
- Every frame rejected due to out-of-range values
- Both sensors disconnect frequently
- Position tracking completely unusable

**Solution:** WiFi Configuration Mode is **only enabled when needed** for zone setup, then automatically disabled for stable normal operation.

## Configuration Required

Before building, configure WiFi credentials in `/main/shs01.c`:

### WiFi Credentials (Lines 56-57)

```c
#define WIFI_SSID               "YOUR_WIFI_SSID"
#define WIFI_PASS               "YOUR_WIFI_PASSWORD"
```

Replace with your actual WiFi network credentials.

## HTTP API Endpoint

The ESP32 exposes a simple HTTP GET endpoint:

**URL:** `http://{ESP32_IP}/api/positions`
**Method:** GET
**Response:** JSON

### Response Format

```json
{
  "targets": [
    {"x": -1200, "y": 2300, "distance": 2500, "speed": 150},
    {"x": 800, "y": 3100, "distance": 3200, "speed": 0}
  ],
  "count": 2,
  "timestamp": 1234567890
}
```

**Field Descriptions:**
- **x**: Horizontal position in mm (-3000 to +3000)
- **y**: Forward distance in mm (0 to 6000)
- **distance**: Distance from sensor in mm
- **speed**: Speed in mm/s (signed)
- **count**: Number of active targets (not always same as targets.length due to filtering)
- **timestamp**: Millisecond timestamp

**Note:** Only **valid** targets are included (garbage data filtered).

## Data Validation

The firmware filters invalid target data to prevent crashes:

- ✅ X range: -3000 to +3000 mm
- ✅ Y range: 0 to 6000 mm
- ✅ Distance: < 6000 mm
- ✅ Active flag must be true
- ❌ Filters: X=26624, Y=-30464, and other garbage values

## Build Instructions

1. Configure WiFi credentials in `main/shs01.c`
2. Build and flash:
   ```bash
   cd SHS01-enhanced
   idf.py build flash monitor
   ```

3. Check logs for:
   - `WiFi connected - IP: xxx.xxx.xxx.xxx`
   - `HTTP server started successfully`
   - `Poll position data: http://{ESP32_IP}/api/positions`

## Web App Setup

The web app uses **two connections**:

1. **MQTT** (for Zigbee data):
   - Occupancy, target count, zone states
   - Enter MQTT broker WebSocket URL: `ws://192.168.1.100:9001`

2. **HTTP Polling** (for position data):
   - Real-time X/Y coordinates
   - Polls `http://{ESP32_IP}/api/positions` every 200ms (5 updates/sec)
   - Enter ESP32 IP address in the form

## Advantages Over WebSocket

✅ **Better Zigbee coexistence** - WiFi only active ~10ms per request
✅ **No beacon timeouts** - WiFi can sleep between polls
✅ **No persistent connections** - Reduces WiFi disconnections
✅ **Lower resource usage** - No connection management overhead
✅ **Validated data** - Garbage values filtered before sending
✅ **CORS enabled** - Works from any origin

## Performance

- **Polling interval:** 200ms (5 updates/sec)
- **Request time:** ~5-15ms per poll
- **WiFi duty cycle:** ~5-7.5% (much better for Zigbee)
- **Data latency:** 0-200ms (average 100ms)

## Troubleshooting

### HTTP Requests Failing

- Verify ESP32 IP address is correct
- Check ESP32 is on same network as browser
- Ensure port 80 is not blocked by firewall
- Check logs: `HTTP server started successfully`
- Try accessing in browser: `http://{ESP32_IP}/api/positions`

### No Position Data

- Check browser console for fetch errors
- Verify targets are valid (not garbage data)
- Check logs for validation failures
- Ensure LD2450 is sending data: `LD2450 target count: X`

### WiFi Still Disconnecting (Beacon Timeout 201)

- Normal with heavy Zigbee routing - WiFi reconnects automatically
- Consider reducing poll rate to 400ms (edit `main.js` line 217)
- Position data will have brief gaps during disconnections
- Data resumes automatically when WiFi reconnects

### Targets Showing on Left Side Only

- This was the mock data issue - now fixed with real sensor data
- If still happening, check logs for "Invalid" target messages
- Try power cycling the LD2450 sensor

## Testing the Endpoint

You can test the HTTP endpoint directly in your browser:

```
http://192.168.1.XXX/api/positions
```

You should see JSON output with current target positions.
