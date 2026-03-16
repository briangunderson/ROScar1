# UWB Indoor Positioning Integration Plan

## Overview

Integrate 4x Makerfabs ESP32 UWB (DW1000) anchors + 1x M5Stack UWB Unit (U100, DW1000) tag to provide absolute indoor position fixes. UWB position will be fused into the existing EKF alongside wheel odometry and IMU, specifically addressing mecanum strafing drift where wheel encoders are unreliable.

---

## 1. Hardware

### 1.1 Anchor Boards: Makerfabs ESP32 UWB (DW1000)

- 4 boards, each configured as a TWR (Two-Way Ranging) anchor with a unique short address
- Powered by USB (5V phone chargers) or 3.7V LiPo (board has JST connector)
- Mount at known map-frame positions, ideally high on walls (~2m), spread around operating area
- Minimum 3 anchors for 2D trilateration; 4 provides redundancy and better geometry (lower DOP)
- Place anchors to form a convex hull around the robot's operating area; avoid collinear placement

**Anchor placement example (basement)**:
```
  A0 (0, 0, 2.0)          A1 (4, 0, 2.0)
         +---------------------+
         |                     |
         |     Robot moves     |
         |       here          |
         |                     |
         +---------------------+
  A2 (0, 4, 2.0)          A3 (4, 4, 2.0)
```

Anchor positions must be measured in the SLAM map frame (or a known transform from map frame). Measure anchor positions relative to the SLAM origin after building an initial map.

### 1.2 Tag: M5Stack UWB Unit (U100)

- DW1000 chip, connects to RPi5 via UART (Grove port: TX/RX at 3.3V logic)
- Connect to Pi via USB-to-UART adapter (e.g., FTDI or CP2102 breakout) OR directly to Pi GPIO UART pins
- Mount on top of the robot, as high as possible, clear of metal obstructions
- Tag outputs range measurements to each anchor over serial

**Preferred connection: USB-to-UART adapter** for udev symlink consistency.

### 1.3 Wiring

| Device | Connection | Pi Interface | Symlink |
|--------|-----------|-------------|---------|
| M5Stack UWB tag | USB-UART (CP2102/CH340) | /dev/ttyUSBx | /dev/uwb_tag |

**udev rule** (add to `scripts/udev/99-roscar.rules`):
```
# M5Stack UWB Unit (U100)
# TODO: Run `udevadm info -a /dev/ttyUSBx` after plugging in to get correct IDs
SUBSYSTEM=="tty", ATTRS{idVendor}=="XXXX", ATTRS{idProduct}=="YYYY", SYMLINK+="uwb_tag", MODE="0666"
```

**CRITICAL**: If the UWB tag uses the same CP2102 chip as the RPLIDAR (10c4:ea60), disambiguate by serial number (`ATTRS{serial}=="XXXXXXXX"`).

---

## 2. Anchor Firmware (Makerfabs ESP32 UWB)

### 2.1 Library

Use **DW1000Ng** by F-Army: `https://github.com/F-Army/arduino-dw1000-ng`
- Actively maintained, cleaner TWR examples, better ranging accuracy

### 2.2 Anchor Sketch

Each anchor runs a TWR responder. Change `ANCHOR_ID` per board (0-3):

```cpp
// anchor_firmware.ino
#include <DW1000Ng.hpp>
#include <DW1000NgRanging.hpp>

const uint8_t ANCHOR_ID = 0;           // CHANGE per board: 0, 1, 2, 3
const uint16_t ANCHOR_SHORT_ADDR = 0x1000 + ANCHOR_ID;
const uint8_t NETWORK_ID = 0x0A;       // Same for all anchors + tag
const uint8_t CHANNEL = 5;             // UWB channel 5 (6.5GHz)

// Makerfabs ESP32 UWB pin mapping (verify against your board revision!)
const uint8_t PIN_RST = 27;
const uint8_t PIN_IRQ = 34;
const uint8_t PIN_SS  = 21;

device_configuration_t DEFAULT_CONFIG = {
    false, SFDMode::STANDARD_SFD, Channel::CHANNEL_5,
    DataRate::RATE_6800KBPS, PulseFrequency::FREQ_64MHZ,
    PreambleLength::LEN_128, PreambleCode::CODE_4
};

void setup() {
    Serial.begin(115200);
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::setNetworkId(NETWORK_ID);
    DW1000Ng::setDeviceAddress(ANCHOR_SHORT_ADDR);
    DW1000Ng::setAntennaDelay(16436);  // Calibrate per board
    DW1000Ng::startReceive();
    Serial.printf("Anchor %d ready (addr=0x%04X)\n", ANCHOR_ID, ANCHOR_SHORT_ADDR);
}

void loop() {
    RangeAcceptResult result = DW1000NgRanging::tagRangeAccept();
    if (result.success) {
        Serial.printf("Range from tag: %.3f m\n", result.range);
    }
}
```

**Antenna delay calibration**: Place tag at known 2.0m distance from each anchor, adjust `setAntennaDelay()` until measured range matches. Start with 16436 (factory default).

---

## 3. Tag Firmware (M5Stack UWB Unit U100)

### 3.1 Option A: Stock Firmware
Check if stock firmware supports TWR tag mode with multi-anchor ranging. If it outputs parseable range data, use it directly.

### 3.2 Option B: Custom Tag Firmware

```cpp
// tag_firmware.ino
#include <DW1000Ng.hpp>
#include <DW1000NgRanging.hpp>

const uint16_t TAG_SHORT_ADDR = 0x2000;
const uint8_t NETWORK_ID = 0x0A;
const uint8_t NUM_ANCHORS = 4;
const uint16_t ANCHOR_ADDRS[NUM_ANCHORS] = {0x1000, 0x1001, 0x1002, 0x1003};

// M5Stack UWB Unit pins (VERIFY against schematic!)
const uint8_t PIN_RST = 25;
const uint8_t PIN_IRQ = 36;
const uint8_t PIN_SS  = 5;

float ranges[NUM_ANCHORS] = {0};
unsigned long lastRangeTime[NUM_ANCHORS] = {0};

void setup() {
    Serial.begin(115200);
    DW1000Ng::initializeNoInterrupt(PIN_SS, PIN_RST);
    DW1000Ng::applyConfiguration(DEFAULT_CONFIG);
    DW1000Ng::setNetworkId(NETWORK_ID);
    DW1000Ng::setDeviceAddress(TAG_SHORT_ADDR);
    DW1000Ng::setAntennaDelay(16436);
    Serial.println("UWB Tag ready");
}

void loop() {
    static uint8_t currentAnchor = 0;
    static unsigned long lastOutput = 0;

    RangeInfrastructureResult result = DW1000NgRanging::tagRangeInfrastructure(
        ANCHOR_ADDRS[currentAnchor], DW1000NgRanging::RANGING_TIMEOUT_US);

    if (result.success && result.range > 0.0 && result.range < 50.0) {
        ranges[currentAnchor] = result.range;
        lastRangeTime[currentAnchor] = millis();
    }

    currentAnchor = (currentAnchor + 1) % NUM_ANCHORS;

    // Output all ranges at ~10Hz
    if (millis() - lastOutput > 100) {
        lastOutput = millis();
        uint8_t fresh = 0;
        for (int i = 0; i < NUM_ANCHORS; i++) {
            if (millis() - lastRangeTime[i] < 500) fresh |= (1 << i);
        }
        // Protocol: $UWBR,<r0>,<r1>,<r2>,<r3>,<freshness_bitmask>
        Serial.printf("$UWBR,%.3f,%.3f,%.3f,%.3f,%d\n",
                       ranges[0], ranges[1], ranges[2], ranges[3], fresh);
    }
    delay(10);
}
```

---

## 4. ROS2 uwb_localizer_node

### 4.1 Node: `roscar_ws/src/roscar_driver/roscar_driver/uwb_localizer_node.py`

**Architecture**:
1. Read serial data from UWB tag at 10Hz
2. Parse range measurements to each anchor
3. Perform 2D trilateration (weighted least-squares) using known anchor positions
4. Publish `nav_msgs/Odometry` on `/uwb/position` for EKF fusion
5. Publish `/uwb/ranges` (JSON) and `/uwb/status` (JSON) for dashboard
6. Outlier rejection: discard ranges deviating >3 sigma from expected

**Published topics**:

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/uwb/position` | nav_msgs/Odometry | 10Hz | 2D position for EKF fusion |
| `/uwb/ranges` | std_msgs/String | 10Hz | JSON: raw ranges + anchor status |
| `/uwb/status` | std_msgs/String | 2Hz | JSON: num_anchors, DOP, last_fix_age |

**Trilateration algorithm** (linearized least-squares, 2D):
- Correct for height: `d_i = sqrt(r_i^2 - dz_i^2)` to project to 2D
- Linearize by subtracting last equation from all others
- Solve `A @ [x, y] = b` via `np.linalg.lstsq`
- Fall back to `scipy.optimize.least_squares` if condition number is bad

**Odometry message**:
- `frame_id: "odom"`, `child_frame_id: "base_footprint"`
- Fuse only x, y position; orientation = identity (UWB gives no heading)
- Covariance: 0.04 m^2 (4 anchors good geometry) to 0.25 m^2 (3 anchors poor)
- Large covariance (1e6) for z, roll, pitch, yaw (not measured)

### 4.2 Config: `roscar_ws/src/roscar_driver/config/uwb_params.yaml`

```yaml
uwb_localizer:
  ros__parameters:
    serial_port: "/dev/uwb_tag"
    serial_baud: 115200
    publish_rate: 10.0
    robot_z: 0.15                # UWB tag height above ground (m)
    min_anchors: 3
    max_range: 15.0
    outlier_sigma: 3.0
    range_sigma: 0.15            # DW1000 typical range noise (m)
    position_covariance_good: 0.04   # m^2 when 4 anchors
    position_covariance_poor: 0.25   # m^2 when 3 anchors
    anchor_positions:
      "0": [0.0, 0.0, 2.0]
      "1": [4.0, 0.0, 2.0]
      "2": [0.0, 4.0, 2.0]
      "3": [4.0, 4.0, 2.0]
```

### 4.3 Entry Point

Add to `roscar_ws/src/roscar_driver/setup.py`:
```python
'uwb_localizer = roscar_driver.uwb_localizer_node:main',
```

---

## 5. EKF Integration

### 5.1 Changes to `ekf.yaml`

Add UWB as third sensor (`odom1`):

```yaml
ekf_filter_node:
  ros__parameters:
    # ... existing odom0 (wheel odometry) and imu0 (gyro) unchanged ...

    odom1: "uwb/position"
    odom1_config: [true,  true,  false,   # x, y, z
                   false, false, false,   # roll, pitch, yaw
                   false, false, false,   # vx, vy, vz
                   false, false, false,   # vroll, vpitch, vyaw
                   false, false, false]   # ax, ay, az
    odom1_differential: false   # absolute position
    odom1_relative: false
```

**Why this works**: EKF currently integrates wheel velocity + IMU yaw rate to estimate position (drifts). UWB provides absolute position that bounds drift. Kalman gain auto-weights based on covariances — during strafing, wheel odom drifts but UWB stays accurate, so EKF follows UWB.

### 5.2 Process Noise Tuning

May need to increase x/y process noise from 0.05 to 0.08 for faster UWB correction during strafing. Tune on hardware.

---

## 6. Launch File Changes

### 6.1 Add `use_uwb` to `robot.launch.py`

```python
use_uwb_arg = DeclareLaunchArgument(
    'use_uwb', default_value='false',
    description='Launch the UWB localizer node',
)

uwb_node = Node(
    package='roscar_driver',
    executable='uwb_localizer',
    name='uwb_localizer',
    parameters=[uwb_config],
    output='screen',
    condition=IfCondition(LaunchConfiguration('use_uwb')),
)
```

### 6.2 Update SLAM/Nav launch files to pass `use_uwb: true`

Use **Option A** (simpler): Always include odom1 in ekf.yaml. robot_localization handles missing topics gracefully with periodic warnings.

---

## 7. Dashboard Integration

Add to `aio-map.js` (following `subscribeLandmarks()` pattern):
- Subscribe to `/uwb/ranges` and `/uwb/status`
- Draw anchor positions as circles (green=ranging, red=lost)
- Draw dashed range lines from robot to each anchor
- Add UWB status row to status panel

---

## 8. UWB + ArUco Landmark Coexistence

| Feature | UWB | ArUco Landmarks |
|---------|-----|----------------|
| Coverage | Full area (if anchors cover it) | Only near marker locations |
| Rate | Continuous 10Hz | Only when markers visible |
| Accuracy | ~10-30cm | ~5-15cm (close range) |
| Fusion | Continuous EKF odom1 | Discrete EKF /set_pose |

Both operate independently. When UWB is running, consider increasing landmark `correction_interval` from 10s to 30s (UWB handles continuous drift, ArUco becomes sanity check).

---

## 9. Implementation Phases

- [ ] **Phase 1: Anchor Firmware** (when hardware arrives)
  - [ ] Install Arduino IDE + DW1000Ng library
  - [ ] Flash 4x anchors (unique ANCHOR_ID each)
  - [ ] Verify serial debug output
  - [ ] Calibrate antenna delay per board (2m reference test)

- [ ] **Phase 2: Tag Firmware**
  - [ ] Check M5Stack U100 stock firmware capabilities
  - [ ] Flash custom firmware if needed
  - [ ] Connect to Pi, verify serial data
  - [ ] Add udev rule for `/dev/uwb_tag`

- [ ] **Phase 3: ROS2 Node**
  - [ ] Create `uwb_localizer_node.py`
  - [ ] Create `config/uwb_params.yaml`
  - [ ] Add entry point to `setup.py`
  - [ ] Test serial parsing and trilateration
  - [ ] Verify `/uwb/position` publishes correct Odometry

- [ ] **Phase 4: EKF Fusion**
  - [ ] Add odom1 to `ekf.yaml`
  - [ ] Add `use_uwb` launch argument
  - [ ] Test: straight line (no oscillation)
  - [ ] Test: strafing (UWB corrects drift)
  - [ ] Tune covariances

- [ ] **Phase 5: Launch Integration**
  - [ ] Update all launch files with `use_uwb`
  - [ ] Test full stack: SLAM + UWB + Nav2

- [ ] **Phase 6: Dashboard**
  - [ ] Add UWB visualization to `aio-map.js`
  - [ ] Add UWB status row to `aio.html`

- [ ] **Phase 7: Tuning & Validation**
  - [ ] Measure accuracy at known waypoints
  - [ ] Compare strafing drift before/after UWB
  - [ ] Tune landmark_localizer thresholds for coexistence

---

## 10. Dependencies

**Pi (runtime)**:
- `pyserial` (likely installed)
- `numpy` (already used)
- `scipy` — `sudo apt install python3-scipy`

**Dev PC (firmware)**:
- Arduino IDE 2.x + ESP32 board support
- DW1000Ng library

**No new ROS2 packages needed.**

---

## 11. Risks

| Risk | Mitigation |
|------|-----------|
| M5Stack U100 stock FW doesn't support multi-anchor TWR | Flash custom firmware |
| NLOS multipath errors >1m | Outlier rejection; mount anchors high; 4 anchors for redundancy |
| USB serial conflict with RPLIDAR (same CP2102) | Disambiguate by serial number in udev rules |
| EKF oscillation between UWB and wheel odom | Increase UWB covariance, tune process noise |
| Anchor position measurement error | Measure carefully; refine using SLAM map coordinates |
