# tilt_controller — ESP32-S2 motor control node

micro-ROS firmware for the tilt-axis motor control node described in the
project brief: owns the RS485 link to the MKS SERVO42D driver, runs the
homing/move/settle state machine, and publishes `sensor_msgs/JointState` +
a simple status string for the tilt joint.

## Layout

- `main/pins.h` — GPIO assignments (RS485 UART, limit switch, LED)
- `main/mks_servo42d.[ch]` — RS485 transport + MKS driver command layer
- `main/tilt_axis.[ch]` — homing state machine, move/settle tracking
- `main/main.c` — micro-ROS node: transport setup, publishers, subscribers,
  control loop
- `components/micro_ros_espidf_component` — vendored (git clone, `jazzy`
  branch), not written by hand
- `components/esp_usbcdc_transport`, `components/esp_usbcdc_logging` —
  copied verbatim from `micro_ros_espidf_component`'s own
  `examples/int32_publisher_custom_transport_usbcdc`; implement the
  USB-CDC custom transport (`esp_usbcdc_open/close/write/read`) and a
  second CDC interface for log output

## Transport: USB-CDC custom transport

Rather than the component's built-in UART or WiFi transports, this node
uses micro-ROS's **custom transport** hook over the ESP32-S2's *native*
USB (GPIO19/20) — the exact pattern in
`components/micro_ros_espidf_component/examples/int32_publisher_custom_transport_usbcdc`,
copied rather than reconstructed from memory, so it matches what that
component version actually compiles against. This keeps UART1 (RS485 to
the MKS driver) and the USB-Serial-JTAG console (GPIO43/44, flashing/
monitor) both free, per the project brief's pin notes.

At runtime the board exposes two USB-CDC ACM ports over its **native USB
connector** (not the USB-to-UART one some dev boards also have):
interface 0 carries the micro-ROS agent link, interface 1 carries
`ESP_LOG` output.

## Build setup

1. Install ESP-IDF v5.x and set the target:
   ```
   idf.py set-target esp32s2
   ```
2. `micro_ros_espidf_component` is already vendored under `components/`
   (cloned from the `jazzy` branch — matches whatever ROS2 distro you're
   running on the PC side). `esp_usbcdc_transport` and `esp_usbcdc_logging`
   are plain local components; their `idf_component.yml` files pull in
   `espressif/esp_tinyusb` automatically via the IDF Component Manager on
   first build.
3. First build compiles the micro-ROS client library itself (a Docker- or
   colcon-based step the component runs automatically) — expect the first
   `idf.py build` to take noticeably longer than later ones:
   ```
   idf.py build
   ```
4. Flash via the USB-to-UART connector if your dev board has one (regular
   ESP-IDF flow), then **move the USB cable to the native USB connector**
   before running the agent — the app's runtime USB-CDC interfaces only
   exist on native USB, not the UART-bridge port:
   ```
   idf.py -p <PORT> flash monitor
   ```
   (`monitor` here will only show early boot logs over USB-Serial-JTAG,
   before `esp_usbcdc_logging_init()` redirects `ESP_LOG` to the native-USB
   CDC interface 1 — reconnect and use that port for logs afterward.)
5. On the PC, once connected via native USB, find the two new CDC ports
   (Linux: `/dev/ttyACM0`/`ttyACM1`; Windows: two new `COM` ports) and run
   the agent against the first one:
   ```
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
   ```

## Topics

- `tilt/joint_state` (`sensor_msgs/JointState`, published every control
  loop tick, ~10 Hz) — single joint named `tilt_joint`, position in
  radians. Feed straight into `robot_state_publisher` per the brief.
- `tilt/status` (`std_msgs/String`) — `unhomed` / `homing` / `idle` /
  `moving` / `sweeping` / `settling` / `settled` / `error`, published on
  state changes.
- `tilt/cmd_position_deg` (`std_msgs/Float32`, subscribe) — commands a
  move to an absolute target angle in degrees. Same topic for both
  operating modes — which one happens depends on `tilt/cmd_sweep_rate_deg_s`.
- `tilt/cmd_sweep_rate_deg_s` (`std_msgs/Float32`, subscribe) — sets the
  operating mode for subsequent `tilt/cmd_position_deg` commands: `0` (the
  default) is step-and-stare (discrete move, then settle); any positive
  value switches to continuous sweep at that angular rate (`moving` state
  is replaced by `sweeping`, streaming `tilt/joint_state` the whole way to
  the target instead of stopping at intermediate stations). Doesn't affect
  a move already in progress.
- `tilt/cmd_home` (`std_msgs/Bool`, subscribe) — publish `true` to trigger
  the homing sequence. Intended to be triggered once per session from the
  PC state machine, not autonomously on boot, so it lands in the session's
  ROS2 logs.

## Known unknowns — verify before running with the motor coupled and powered

The micro-ROS transport layer (this file's main subject) is now copied
from a real, maintained example rather than guessed, so it should build
and link as-is.

1. **MKS SERVO42D register map** (`main/mks_servo42d.c`) is now transcribed
   directly from "MKS SERVO42&57D_Modbus RTU User Manual V1.0.9", not
   guessed — register addresses, function codes (0x04 read-input-registers
   / 0x06 write-single / 0x10 write-multiple), and field layouts all match
   the manual's own worked examples; CRC16 byte order was cross-checked
   against 4 independent example frames from the manual. The driver now
   also: puts the motor into SR_vFOC bus control mode during `mks_init()`
   (required for all the position/speed/status registers used here),
   explicitly enables stall protection during init (without this, the
   `stalled` flag sensorless homing depends on never triggers), and calls
   `mks_release_stall()` after each detected stall in the homing sequence
   (the real driver de-energizes the motor when stall protection trips, so
   the next move would otherwise silently do nothing). What's still
   genuinely unverified against *this specific unit*: which register value
   (0 or 1) for direction corresponds to `direction_positive` in this
   driver's API — the manual documents the values but not which one is
   "up" on this mount; confirm on first bring-up and flip
   `MKS_DIR_CW`/`MKS_DIR_CCW` in `mks_servo42d.c` if needed.
2. **Tuning constants** (`main/tilt_axis.c`, top of file): homing
   current/speeds and `MOVE_SPEED` are now in real RPM (driver's documented
   range: 0-3000), `MOVE_ACCEL` is the driver's real ramp-rate unit
   (0-255, 0 = no ramp) — but the specific numeric values are still
   placeholders needing empirical tuning. `HOMING_DIRECTION_TOWARD_STOP`
   and especially `SETTLE_TIME_MS` are the same category — the brief flags
   settle time as the least-known variable and likely dominant cost in
   total scan time. All marked `TUNE` inline; measure on ~20-30 real stops
   before committing to a full scan run, per the brief's own
   recommendation. Continuous-sweep speed conversion
   (`sweep_rate_to_driver_speed()`) now uses the real unit relationship
   (RPM = deg/s ÷ 6) instead of an arbitrary scale factor.

RS485 GPIO pins in `main/pins.h` are now confirmed against actual wiring
(TX=GPIO33, RX=GPIO18, RTS/EN=GPIO16). Limit switch and status LED pins
are still unverified placeholders. Also still unverified: the RS485 baud
rate (38400) / slave address (1) passed to `mks_init()` in `main.c` are the
driver's documented factory defaults — plausible but not confirmed against
this specific unit — and `MICROSTEPS` in `tilt_axis.c` (the driver's
configured microstep DIP-switch setting, not something this firmware sets
over UART, so it must match whatever the hardware is actually set to).

**Continuous-sweep mode scope note:** this firmware streams `tilt/joint_state`
continuously while sweeping, but that's only half of what continuous
scanning needs. Correlating each LiDAR point's exact timestamp against an
interpolated tilt angle (rather than a single settled angle per stop) is a
PC-side aggregator concern, not something this firmware does — see the
project brief's own note on why step-and-stare avoids that problem. Don't
expect usable continuous-sweep point clouds until that aggregator-side
interpolation exists.
