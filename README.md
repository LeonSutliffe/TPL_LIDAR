# TPL — Terrestrial Panning Lidar

A tripod-mounted Velodyne VLP-16 on a stepper-driven external tilt axis.
The VLP-16 spins internally in azimuth; the external axis rotates the
whole sensor about a second, roughly-perpendicular axis to fill in the
gaps between the VLP-16's 16 fixed laser channels, building up a dense,
near-spherical point cloud from a single stationary tripod station.

Two scan modes, both writing one `.pcd` file per run:

- **Step-and-stare** — home, then move to each tilt step, stop, capture a
  fixed number of VLP-16 revolutions, advance. Motion-blur-free; the
  validated, primary mode.
- **Continuous sweep** — sweep the tilt axis back and forth continuously
  while merging clouds live. Faster coverage, trades some motion blur.

Full project history, every bug found/fixed, and all the "why" behind
non-obvious decisions lives in [`HANDOFF.md`](HANDOFF.md) — this file is
a shorter orientation + the Pi deployment guide. If something here and
`HANDOFF.md` disagree, `HANDOFF.md` is the more current/authoritative one.

## How it works

```
                    ┌─────────────────────────┐
   USB              │  USB<->RS485 adapter     │  RS485 (A/B)
   ─────────────────┤  (FTDI FT232-family)     ├──────────► MKS SERVO42D driver ─► stepper
                    └─────────────────────────┘                                    + tilt axis

ROS2 graph:

  tilt_axis_bridge          scan_aggregator          vlp16_config
  owns the bridge serial    drives step-and-stare/    owns the VLP-16's
  link, homing/move/sweep   sweep, tf2-transforms      HTTP config API +
  state machine             + merges clouds into a     tilt->sensor mount
                             single .pcd per run        offset

  velodyne_driver_node → velodyne_transform_node → /velodyne_points

  rosbridge_websocket (ws://<host>:9090) ── serves the browser GUI
  (web/tilt_axis_gui/index.html — single self-contained HTML file,
  no build step)
```

An off-the-shelf USB↔RS485 adapter (an FTDI FT232-family device, plain
serial to the host — originally a Raspberry Pi Pico running custom
bridge firmware, replaced 2026-08-31; see `HANDOFF.md`'s "Architecture
pivot" section for the full history) sits between the host and the MKS
driver; everything else — all MKS protocol logic, the scan state
machines, tf2 transforms, the point cloud merge — runs as plain ROS2
nodes on whatever machine `ros2 launch` runs on. **That machine is now a
standalone Raspberry Pi 4** — the whole rig runs self-contained in the
field, confirmed end-to-end including a cold-boot test (see "Setting up
from scratch" below). The Windows/WSL2 laptop this project was
originally developed on is retired as a deployment target; its setup
instructions remain in `HANDOFF.md`'s "Running it" section purely as
historical reference.

## Repo layout

| Path | What it is |
|---|---|
| `ros2_ws/src/tilt_axis_bridge` | Owns the bridge serial link; MKS driver protocol, homing/move/sweep state machine |
| `ros2_ws/src/scan_aggregator` | Drives a scan (either mode), tf2-transforms and merges clouds, writes the output `.pcd` |
| `ros2_ws/src/vlp16_config` | VLP-16 hardware config (its own HTTP API) + the tilt→sensor mount-offset transform |
| `ros2_ws/src/scanner_bringup` | Velodyne driver launch + the full-stack `bringup.launch.py` |
| `ros2_ws/src/scanner_description` | URDF/xacro + `robot_state_publisher` |
| `web/tilt_axis_gui/index.html` | The control GUI — connects to rosbridge over WebSocket, no build step |
| `web/tilt_axis_gui/status.html` | The onboard screen's kiosk status display (network/motor/VLP-16/scan) |
| `scripts/pi/` | Onboard-screen kiosk autostart, net-info script, on-demand terminal status view, GUI-facing WiFi config bridge |
| `scripts/calibration/` | Offline mount-angle calibration (fixes double-image overlap artifacts) — see below |
| `scripts/*.ps1`, `scripts/launch_stack.sh` | Windows/WSL2 launch helpers — historical only, that path is retired (see `HANDOFF.md`) |

## Hardware

- Velodyne VLP-16, on Ethernet (its own static IP; the driver filters
  incoming packets by that address)
- MKS SERVO42/57D closed-loop stepper driver on the tilt axis, RS485
- An off-the-shelf USB↔RS485 adapter (FTDI FT232-family, VID:PID
  0403:6001; plain serial device to the host — see `HANDOFF.md`'s
  "Architecture pivot" for what this replaced and why)
- A hard project-wide safety cap of **40 RPM** and a **0–260°** rotation
  limit are enforced in code (`tilt_axis_bridge`), not just convention

## Setting up from scratch (Raspberry Pi 4)

**Status, 2026-09-07: this is now a genuinely self-contained field unit,
confirmed by an actual reboot test.** Every step below except the WiFi
hotspot's live activation (step 8) has been executed against the real
Pi over SSH, not just planned: OS/SSH/repo/RoboStack/`colcon build`,
VLP-16 + tilt-axis wiring (with a real static-IP gap found and fixed),
USB automount rule installed, hotspot profile configured (not yet
switched live), and systemd auto-start for both the ROS2 stack and the
GUI server — verified surviving a cold `sudo reboot` with zero manual
steps: real hardware reconnected, real point cloud data flowing, GUI
reachable. **The Windows/WSL2 laptop path is retired** — per explicit
decision, the Pi is now the sole deployment target; `HANDOFF.md`'s
"Running it" Windows/WSL2 section is kept for historical reference only.

### 1. Flash the OS

Use **Raspberry Pi Imager**, choose **Raspberry Pi OS Lite (64-bit)** —
64-bit matters (aarch64 ROS2 packages); **Lite**, not the Desktop
variant, since the onboard display is a small GPIO/SPI panel that needs
its own lightweight status script, not a full desktop + browser (see
step 10) — no reason to spend RAM/CPU on a desktop environment the Pi
will never actually use.

**What actually got flashed, 2026-09-07** (recorded here since it
deviates from the plan above): `uname -m`/`/etc/os-release` on the real
Pi report `aarch64` / **Debian GNU/Linux 13 (trixie)**, and a desktop
session is present (`~/.Xauthority`, `~/.xsession-errors`, a populated
`~/Desktop`) — meaning either the Desktop variant was used, or a plain
Debian trixie image rather than a Raspberry Pi OS one. Not corrected
retroactively — the GPIO screen setup and RoboStack/`colcon build` (see
step 4) both worked fine on this actual image regardless, so there's no
proven need to reflash Lite/Bookworm specifically; noted here mainly so
a future step that assumes a headless Lite image (RAM budget, no
competing desktop process) isn't surprised by what's actually running.
Before writing a fresh image, open the advanced options (gear icon) and
set:

- a hostname (e.g. `tpl-scanner`)
- SSH enabled, with your public key (or a password if you don't have a
  key pair yet)
- WiFi SSID/password, if the Pi won't be on Ethernet immediately

This gets the Pi to a headless, SSH-reachable state on first boot — no
monitor or keyboard needed for initial setup.

### 2. Connect the hardware (confirmed working, 2026-09-07)

- VLP-16 → the Pi's Ethernet port (direct, or via switch)
- USB<->RS485 bridge adapter → any Pi USB port

**The Ethernet interface needs a static IP on the VLP-16's subnet —
found live, not in the original plan.** The VLP-16 doesn't DHCP, so
`eth0` sits with link up but no IPv4 address until you give it one
manually. This system uses NetworkManager (confirmed via `nmcli`); the
Ethernet profile was named `netplan-eth0` on the actual Pi, but check
`nmcli connection show` for the real name on yours:
```bash
sudo nmcli connection modify <eth0-connection-name> \
    ipv4.method manual ipv4.addresses 192.168.1.100/24 \
    ipv4.gateway "" ipv4.dns ""
sudo nmcli connection up <eth0-connection-name>
```
`192.168.1.100` is arbitrary (anything but `.201`, the VLP-16's own
factory-default address from `scanner_bringup/config/vlp16.yaml`) — no
gateway/DNS needed since this is a direct point-to-point link, not a
real network. Confirmed working: `ping 192.168.1.201` succeeds
(sub-millisecond, direct link) once this is set.

**Full stack confirmed running on real Pi hardware after this**:
`ros2 launch scanner_bringup bringup.launch.py rviz:=false` --
`tilt_axis_bridge` connected to the real MKS driver over the FTDI
bridge (same `set_enable`/`read_config_params` round-trip verification
as the Windows/WSL2 side), and `ros2 topic hz /velodyne_points` showed
a real, live, stable ~16Hz point cloud stream -- not just "no timeout
warning," an actual verified data rate. `ps aux` confirmed exactly one
instance of each node, no orphans. This is the first time the full
scanner stack has run end-to-end on the Pi itself rather than the
laptop.

### 3. Get the code onto the Pi

```bash
git clone https://github.com/LeonSutliffe/TPL_LIDAR.git
cd TPL_LIDAR
```

### 4. Install ROS2 — RoboStack via micromamba (confirmed working, 2026-09-07)

**Resolved, live against the real Pi**: RoboStack does publish
everything this project needs for `linux-aarch64` — `ros-jazzy-velodyne`
(and `-driver`/`-pointcloud`/`-laserscan`/`-msgs`), `ros-jazzy-rosbridge-suite`,
`ros-jazzy-xacro`, `rosdep`, `vcstool`, `pyserial`, and the full build
toolchain, all confirmed by actually resolving and installing them on
the Pi (`micromamba search --platform linux-aarch64` first, then a real
`create`) — not just checked in the abstract. No need for the native-apt
fallback that was the plan B here.

One deliberate change from the Windows/WSL2 side's own package list:
`ros-jazzy-ros-base` instead of `ros-jazzy-desktop` — confirmed via
`--dry-run` that `ros-base` still resolves every package this project's
own nodes actually import (`robot_state_publisher`, `tf2_ros`,
`tf2_sensor_msgs`, `sensor_msgs_py`, `xacro`), while skipping the
`rviz2`/`rqt`/X11 stack that `desktop` pulls in and that the Pi has no
use for (see step 10 — the onboard panel is a plain console, not
something `rviz2` renders into).

```bash
mkdir -p ~/micromamba-bin
curl -Ls https://micro.mamba.pm/api/micromamba/linux-aarch64/latest | tar -xvj -C ~/micromamba-bin --strip-components=1 bin/micromamba

~/micromamba-bin/micromamba create -r ~/micromamba -n ros2 -y \
    -c robostack-jazzy -c conda-forge \
    ros-jazzy-ros-base ros-jazzy-rosbridge-suite ros-jazzy-velodyne ros-jazzy-xacro \
    rosdep vcstool pyserial \
    colcon-common-extensions compilers cmake pkg-config make ninja git
```
~898MB download, ~5.9GB installed; took a few minutes on the Pi's own
connection. No `micromamba shell init` used here either, matching the
Windows/WSL2 side — every invocation passes `-r ~/micromamba -n ros2`
explicitly.

### 5. Build the workspace (confirmed working, 2026-09-07)

```bash
cd ~/TPL_LIDAR/ros2_ws
source ~/micromamba/envs/ros2/setup.bash
colcon build
```
All 5 packages (`scanner_description`, `vlp16_config`, `scan_aggregator`,
`tilt_axis_bridge`, `scanner_bringup`) built clean in ~11 seconds real
time on the Pi 4 — two emit a benign CMake `cmake_minimum_required`
version-syntax deprecation warning (pre-existing, not Pi-specific, not
an actual error). `ros2 pkg list` after `source install/setup.bash`
confirms every custom package and every `velodyne_*` package is
registered correctly.

### 6. USB storage — only needed for exporting finished scans, 2026-09-07

Scans no longer write to USB directly (see "Local-first scan save" below)
— they save to the Pi's own SD card, and USB is purely an on-demand
**export** target from the new Scans tab's "Export to USB" button. There's
nothing to configure for the local save path itself; USB only matters if
you want to copy scans off onto a stick.

- The export target is a fixed path, `/media/tpl/LIDAR`
  (`USB_EXPORT_DIR` in `scan_aggregator/node.py`) — this rig's desktop
  environment (PCManFM/udisks2) already auto-mounts any inserted USB
  storage under `/media/<user>/<volume label>` with no setup needed, so
  **label your export stick `LIDAR`** (case-sensitive) and it lands at
  exactly that path automatically. `blkid`/`lsblk -f` shows a stick's
  current label; relabel an exFAT stick with
  `exfatlabel /dev/sdXN LIDAR` (unmount it first).
- Format it as **exFAT**, not FAT32 (4GB-per-file cap — scans already
  exceed that) or NTFS (weaker Linux write support). exFAT is also
  directly readable on Windows. `sudo apt install exfatprogs` if
  `exfatlabel`/`mkfs.exfat` aren't already present (they are on the
  actual Pi's Debian trixie image).
- If no drive is mounted at `/media/tpl/LIDAR` when you click "Export to
  USB", the GUI reports a clear error rather than silently writing
  anywhere else.

Measured on this rig: the USB stick sustains only ~12 MB/s for durable
writes vs. ~36 MB/s on the Pi's own SD card — the reason local-first save
exists at all. Export happens on a background thread with live progress
in the GUI, so it's fine to kick off after moving on to the next scan.

### 7. Launch

```bash
source install/setup.bash
ros2 launch scanner_bringup bringup.launch.py rviz:=false
```

`rviz:=false` is standard on the Pi, not just an optional flag — there's
no real monitor for it to render into, so it's not just wasted
RAM/CPU/GPU on this deployment, it's pure overhead (the `tpl-scanner.service`
unit in step 9 already always includes it). Other useful launch args
(see `scanner_bringup/launch/bringup.launch.py`): `enable_pointcloud:=false`
(skip point cloud conversion if not needed), `record_bag:=true` (raw
packet + tf recording).

Then open `web/tilt_axis_gui/index.html` in a browser and connect to
`ws://<pi-hostname-or-ip>:9090`.

### 8. Wi-Fi hotspot + controlling it from a phone/tablet

Goal: no external router needed in the field — the Pi broadcasts its own
network, and any phone/tablet/laptop that joins it gets full control via
a normal browser, no app or file transfer needed.

Two things needed for that, both new:

- **The Pi becomes its own access point.** This Pi's actual OS (Debian
  13 "trixie" — see the OS note in step 1) uses NetworkManager by
  default same as the originally-planned Bookworm image, which has this
  built in — no hostapd/dnsmasq hand-rolling needed:

  ```bash
  sudo nmcli connection add type wifi ifname wlan0 con-name TPL-Hotspot \
      autoconnect yes ssid TPL-Scanner
  sudo nmcli connection modify TPL-Hotspot 802-11-wireless.mode ap \
      802-11-wireless.band bg ipv4.method shared
  sudo nmcli connection modify TPL-Hotspot wifi-sec.key-mgmt wpa-psk \
      wifi-sec.psk "change-this-password"
  ```

  **Profile created and configured on the real Pi, 2026-09-07 —
  deliberately not activated yet.** `nmcli connection up TPL-Hotspot`
  actually switches `wlan0` over immediately, which would have cut the
  SSH session doing this setup (it's on the same home WiFi) — so this
  step stopped short of the live switchover. Pick your own real password
  before actually using this (`sudo nmcli connection modify TPL-Hotspot
  wifi-sec.psk "<new password>"`); do this at the Pi itself, or over SSH
  on a connection you're fine losing, since either the modify or the
  first `connection up` can knock you off if you're on `wlan0`.

  `ipv4.method shared` makes NetworkManager act as its own DHCP
  server/gateway for connected clients — the Pi will be reachable at
  `10.42.0.1` (NetworkManager's standard address for a shared connection)
  once this is up. This takes `wlan0` over for AP duty, so the WiFi
  credentials baked into the SD image in step 1 (used for initial
  home-network SSH access) stop being usable on `wlan0` while it's
  active — see "Switching between home WiFi and the hotspot" just below
  for how that's handled without it being a one-way trip. Ethernet stays
  dedicated to the VLP-16 throughout, unaffected either way.

- **Switching between home WiFi and the hotspot.** Both connection
  profiles stay configured on the Pi permanently — switching is about
  which one `wlan0` is actively using, not re-creating either from
  scratch each time. Two ways to do it:

  - **Automatic (recommended)**: give the home-WiFi profile a higher
    `autoconnect-priority` than the hotspot. NetworkManager then handles
    the switching itself with no command needed — it prefers joining
    home WiFi whenever that network is actually in range (so the Pi gets
    real internet/SSH access for updates, `git pull`, etc.), and falls
    back to hosting `TPL-Hotspot` automatically whenever it isn't (i.e.
    out in the field). Find the home-WiFi profile's exact name first
    (`nmcli connection show`, likely just the SSID itself if it came from
    the Imager's preconfigured WiFi), then:

    ```bash
    sudo nmcli connection modify "<home-wifi-profile-name>" connection.autoconnect-priority 10
    sudo nmcli connection modify TPL-Hotspot connection.autoconnect-priority 0 connection.autoconnect yes
    ```

    **Priorities set and confirmed on the real Pi** (`nmcli -f NAME,
    AUTOCONNECT,AUTOCONNECT-PRIORITY connection show` shows home WiFi at
    10, hotspot at 0, both `autoconnect yes`) — the actual "leaves range,
    falls back to hotspot" behavior itself wasn't exercised (would need
    physically taking the Pi out of home WiFi range), so treat the
    fallback logic as configured-and-plausible, not field-proven yet.
    Caveat: this reacts to the connection actually failing/being out of
    range, not instantly on demand — expect a real (if usually short)
    delay switching over, not an immediate cutover.

  - **Manual override**, for forcing one mode on demand regardless of
    what's in range (e.g. testing the hotspot at home before a field
    trip) — two one-line scripts:

    ```bash
    # /usr/local/bin/tpl-wifi-hotspot
    #!/bin/bash
    exec nmcli connection up TPL-Hotspot
    ```
    ```bash
    # /usr/local/bin/tpl-wifi-home
    #!/bin/bash
    exec nmcli connection up "<home-wifi-profile-name>"
    ```
    `sudo chmod +x /usr/local/bin/tpl-wifi-*` after creating both.
    **Both scripts created and confirmed executable on the real Pi.**
    Running `tpl-wifi-home` from an active hotspot-connected SSH session
    will drop that session immediately (expected — the switch itself is
    what disconnects it), but completes on the Pi regardless of whether
    anything was there to see the output.

- **Changing either profile's SSID/password from the GUI, instead of SSH
  + nmcli by hand**: Config → Network on `index.html` (`scripts/pi/
  wifi_config_node.py`, a standalone script + its own
  `tpl-wifi-config.service`, same pattern as `status_display.py` — see
  that service's own setup alongside step 9 below). Reads/writes both
  profiles above directly; saving new home-WiFi credentials while
  currently connected through it can still drop the session at the next
  reconnect, same risk as the manual scripts just described.

- **Serve the GUI itself over HTTP**, so a phone can actually load the
  page (it isn't a file on the phone). `rosbridge_websocket` already
  listens on all interfaces by default (confirmed directly in the
  installed node's source — `address` param defaults to `""`, which
  Tornado binds as all-interfaces, not just localhost), so nothing needed
  there; the GUI's `index.html` file itself is the only thing not yet
  reachable. A plain static file server is enough — no framework, matches
  this GUI's own no-build-step philosophy:

  ```bash
  cd ~/TPL_LIDAR/web/tilt_axis_gui
  python3 -m http.server 8080
  ```

  A phone joined to `TPL-Scanner` then opens `http://10.42.0.1:8080` and
  gets the real GUI. The GUI's rosbridge address field now **defaults to
  whatever host served the page** rather than always `localhost` (fixed
  in `index.html` alongside this — `location.hostname`, which is empty
  for the old `file://` desktop workflow so that path is unaffected) — so
  it should already show `ws://10.42.0.1:9090` and just need Connect
  pressed, no typing an IP by hand.

**HTTP serving confirmed working on the real Pi**, 2026-09-07 —
`curl http://localhost:8080/index.html` returned a real `200`, exact
byte count matching the file's actual size, via the `tpl-gui-http.service`
systemd unit in step 9 (not tested manually with a plain
`python3 -m http.server` invocation, since the service does the same
thing). **Still not tested**: an actual phone/tablet joined to
`TPL-Scanner`, and the whole "leave home WiFi range, hotspot takes over,
phone connects" flow together — the hotspot connection itself was
deliberately never activated this session (see above). The GUI's
smarter address default was independently verified earlier (served over
a plain HTTP server, confirmed the field auto-fills; `file://` fallback
confirmed unaffected).

### 9. Auto-start everything on boot (systemd — confirmed working across a real reboot, 2026-09-07)

Ties the above together into an actual zero-touch device: power it on,
wait, join `TPL-Scanner` from a phone, done — no SSH needed for normal
field use (still available for changes/debugging).

**One real bug caught and fixed here**: the `ExecStart` below only
sources this workspace's own `install/setup.bash`, never the ROS2 distro
environment itself (`~/micromamba/envs/ros2/setup.bash`) — on the
Windows/WSL2 side that step happened implicitly (launched via
`micromamba run -n ros2 ...`), but a systemd service has no such
wrapper, so the original version of this unit would have failed outright
with `ros2: command not found`. Fixed by sourcing both, in order, in the
actual unit installed on the Pi. Also: `pi` was never this Pi's real
user (it's `tpl` — see the OS note in step 1); substitute your own
username and home directory throughout.

`/etc/systemd/system/tpl-scanner.service`:
```ini
[Unit]
Description=TPL scanner ROS2 stack
After=network.target

[Service]
Type=simple
User=tpl
WorkingDirectory=/home/tpl/TPL_LIDAR/ros2_ws
ExecStart=/bin/bash -c 'source /home/tpl/micromamba/envs/ros2/setup.bash && source install/setup.bash && ros2 launch scanner_bringup bringup.launch.py rviz:=false'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

`/etc/systemd/system/tpl-gui-http.service`:
```ini
[Unit]
Description=TPL scanner GUI static file server
After=network.target

[Service]
Type=simple
User=tpl
WorkingDirectory=/home/tpl/TPL_LIDAR/web/tilt_axis_gui
ExecStart=/usr/bin/python3 -m http.server 8080
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

`/etc/systemd/system/tpl-wifi-config.service` (backs Config → Network on
the GUI — see the "Wi-Fi hotspot" section above; not part of the ROS2
launch tree, its own independent node, same reasoning as
`status_display.py`):
```ini
[Unit]
Description=TPL scanner WiFi configuration bridge
After=network.target tpl-scanner.service

[Service]
Type=simple
User=tpl
WorkingDirectory=/home/tpl/TPL_LIDAR
ExecStart=/bin/bash -c 'source /home/tpl/micromamba/envs/ros2/setup.bash && python3 scripts/pi/wifi_config_node.py'
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now tpl-scanner.service tpl-gui-http.service tpl-wifi-config.service
```

Also add `TPL-Hotspot`'s `autoconnect yes` (already set above) so the
hotspot itself comes back on its own after a power cycle too — the three
together (hotspot, ROS2 stack, GUI server) are what make this a genuinely
self-contained device rather than one that still needs an SSH session
after every boot.

**Confirmed with an actual `sudo reboot` on the real Pi, hardware
connected the whole time**: after cold boot, both services came up
`active` on their own, `eth0`'s static IP (step 2) persisted, and —
checked the systemd journal, not just process status — `tilt_axis_bridge`
re-connected to the real MKS driver and `rosbridge_websocket`/
`scan_aggregator` came up clean, all with zero manual steps. `curl
localhost:8080` returned the GUI. `ps aux` confirmed exactly one instance
of every node, no duplicates from old runs. This is the actual "power it
on, wait, done" goal, genuinely proven rather than just planned — the
only piece not exercised in this same pass is the WiFi hotspot's live
activation (see step 8) and joining it from a real phone.

### 10. Onboard screen (Elecrow RR035 / ELEGOO 3.5" GPIO touchscreen)

A 3.5" GPIO/SPI panel, 480×320, XPT2046 resistive touch.

**Panel recognition: confirmed working, 2026-09-07** — the mainline
`piscreen` DRM overlay route (no vendor driver needed) is what's
actually running on the real Pi, confirmed by reading its
`/boot/firmware/config.txt` directly:
```
dtoverlay=piscreen,drm,speed=18000000,invx
```
(`invx` needed for this unit's touch orientation — the other axis flags
`invy`/`swapxy` exist too if a different unit needs them.) The Pi's
actual OS turned out to be a full desktop image (Debian 13 "trixie" with
`lightdm` autologin + the `labwc` Wayland compositor — see the OS note
in step 1), not the planned headless Lite/Bookworm image, so this panel
is the desktop's real, single display — `480x320` confirmed directly via
`xrandr`, not assumed.

**Onboard status display: confirmed working, 2026-09-07** — reverses an
earlier decision on this same day this section was first written, which
had favored "acts like a plain HDMI monitor/console" over a fixed status
readout. Per explicit request, the panel now auto-launches a purpose-built
kiosk status page ([`web/tilt_axis_gui/status.html`](web/tilt_axis_gui/status.html))
fullscreen at startup, showing live network address, motor state, VLP-16
reachability, and scan state — all four sourced the same way the main
GUI is (plain rosbridge WebSocket subscriptions to
`/tilt_axis_bridge/status`, `/vlp16_config/status`,
`/scan_aggregator/status`), with the network line filled from a tiny
separate JSON file (`net_info.json`, refreshed every 5s by
`tpl-net-info.timer`, since a browser page can't query network
interfaces directly). SSH remains the way to actually debug/administer
the Pi regardless of what's on this screen, so nothing about general
debuggability was actually lost by this reversal.

Setup, via [`scripts/pi/write_net_info.sh`](scripts/pi/write_net_info.sh)
and [`scripts/pi/labwc_autostart`](scripts/pi/labwc_autostart) (both
tracked in this repo):
```bash
sudo tee /etc/systemd/system/tpl-net-info.service > /dev/null <<'EOF'
[Unit]
Description=Write current network info for the onboard status display

[Service]
Type=oneshot
ExecStart=/bin/bash /home/tpl/TPL_LIDAR/scripts/pi/write_net_info.sh
EOF

sudo tee /etc/systemd/system/tpl-net-info.timer > /dev/null <<'EOF'
[Unit]
Description=Refresh onboard status display's network info every 5s

[Timer]
OnBootSec=2s
OnUnitActiveSec=5s

[Install]
WantedBy=timers.target
EOF

sudo systemctl enable --now tpl-net-info.timer

cp scripts/pi/labwc_autostart ~/.config/labwc/autostart
chmod +x ~/.config/labwc/autostart
```
The autostart script waits for `tpl-gui-http.service` (step 9) to
actually be serving before launching Chromium in `--kiosk` mode, rather
than a fixed sleep. **One real, non-obvious gotcha it works around**:
without `--password-store=basic`, Chromium's first launch on a fresh
desktop tries to create a system keyring via `gnome-keyring` and blocks
on an interactive "Choose password for new keyring" prompt *instead of
ever showing the kiosk page* — found by actually screenshotting the live
panel (`grim`, the standard wlroots/Wayland screenshot tool, works over
SSH: `XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 grim
out.png`) rather than assuming the autostart script alone was enough
proof. **Verified with a full cold reboot and a second real screenshot**:
the kiosk page comes up on its own with live data (confirmed showing the
real SSID, IP, `idle` motor, `online` VLP-16, `idle` scan) — zero manual
steps, exactly like the systemd services in step 9.

`scripts/pi/status_display.py` (the earlier-planned terminal/ANSI status
view) is still there and still useful as an SSH-only, no-screen-needed
alternative, but is no longer the primary way status is shown on this
device now that the kiosk page above covers that on the actual screen.

### Open questions for this deployment

- Real per-point processing load on a Pi 4 (vs. the dev machine's many
  cores) is unverified — the merge lives entirely in RAM
  (`scan_aggregator`), and a real scan can be tens of millions of points.
  Somewhat de-risked by dropping `rviz2` (step 10 above), since the Pi no
  longer needs to spend any headroom on rendering
  anything itself.

See `HANDOFF.md` for the full decision history on why a Pi 4 was chosen
over a Pi 5, an x86 mini PC, an old Android phone as compute, and an
Intel Compute Stick.

## Local-first scan save (Scans tab: download / export / delete)

**Why**: writing scans directly to the USB stick was measured at ~12 MB/s
on this rig vs. ~36 MB/s on the Pi's own SD card — a real ~3x gap that used
to sit right on the scan-completion critical path (worse, this used to
share the same internal USB2 hub as the tilt axis's FTDI/RS485 adapter,
implicated in real hang/hardware-wedge incidents — see `HANDOFF.md`). Every
scan now saves to the Pi's own SD card first; USB is purely an explicit,
on-demand export target from the GUI, never the live write path. This
isn't a setting — `output_dir` no longer exists as a parameter, and the
GUI's old "choose storage location" folder picker is gone with it.

**Where scans live**: `web/tilt_axis_gui/scans/` (`OUTPUT_DIR` in
`scan_aggregator/node.py`) — deliberately inside the folder
`tpl-gui-http.service`'s plain `python3 -m http.server 8080` already
serves, so every finished scan is downloadable straight from the browser
with zero extra server code.

**Scans tab** (new, alongside Scan/Config): lists every locally-saved
scan with size, timestamp, and whether it's already been exported to USB.
Each row has:
- **Download** — fetches the file directly from this same page's origin.
- **Export to USB** — copies it to `/media/tpl/LIDAR` (see USB storage
  above) on a background thread, with live progress (files can be
  multi-hundred-MB+, so this can take a while at USB write speed) and the
  same fsync-durability guarantee the original write uses (see
  `pcd_writer.fsync_durable`). Re-exporting an already-exported scan is
  fine, not blocked.
- **Delete** — removes only the local copy; an already-exported USB copy
  is untouched.

The post-scan rename popup is unaffected by any of this — it already
worked on whatever `output_dir` happened to be, and still does now that
that's a fixed local path instead of a setting.

## Automatic mount calibration (Scan tab: "Calibrate Mount")

**Recommended first stop** for the double-image/skewed-image symptom
described below — this is a one-button, fully automatic version of the
manual ROI-plane method, built into the GUI. No SSH, no separate capture
script, no manually eyeballing a `--roi` box around a wall.

**What it does**: click **Calibrate Mount** on the Scan tab. It runs a
sweep across whatever Min/Max range is set in Continuous sweep scan
(point it at a real wall first — a wide range gives the most leverage),
then automatically finds a suitable flat surface in the raw scan via a
small hand-rolled RANSAC plane search (`scan_aggregator/
mount_calibration.py`) and solves for `mount_roll_deg`/`mount_pitch_deg`
— the same underlying math as the manual tool below, just with the
`--roi` step automated away. The result (recovered angles, before/after
RMS, how many points and how many degrees of tilt range the found surface
spanned) shows inline; nothing is changed until you review it and click
**Apply**, which pushes the new angles into `vlp16_config` the same way
any other setting on this page is applied.

**Does not touch `mount_yaw_deg`** — see the next section for the full
proof, but in short: a yaw error is mathematically indistinguishable from
rigidly rotating the whole scan about the tilt axis, so no amount of
looking at scan geometry (automatically-found plane or not) can recover
it. That's not a limitation worth working around here either, since yaw
was never the cause of double-image/skewed-image in the first place —
only roll/pitch are, which this fully covers.

**If it fails**: it reports a clear reason rather than a bogus fit — most
commonly "no flat surface with enough points found" (nothing wall-like in
view) or "best flat surface found only spans N deg of tilt" (found a
plane, but too narrow a tilt range to meaningfully constrain roll/pitch —
widen the sweep range or reposition so more of it crosses a real wall).

## Mount-angle calibration (fixes double-image overlap artifacts)

**Symptom**: a duplicate/ghosted copy of part of the scene, offset along
the VLP-16's own spin direction, appearing only in a specific band —
confirmed 2026-09-07 to be a real overlap effect: any scan range past
~150° causes the VLP-16's own 30° vertical FOV to cover some physical
geometry twice, via two different (spin azimuth, tilt angle)
combinations that are supposed to reconstruct onto the same points but
don't, because the physical mount isn't sitting at exactly the assumed
45°-ish angle (`mount_roll_deg`/`mount_pitch_deg` in `vlp16_config`) —
real-world assembly is never perfectly precise. Increasing
`sweep_edge_margin_deg` does **not** fix this — that parameter only
discards points near a *sweep's turnaround*, an unrelated smear source;
this artifact is a geometry/calibration issue present in step-and-stare
too.

**Fix**: for normal use, the "Calibrate Mount" GUI button above already
does this automatically, with no manual `--roi` step. `scripts/
calibration/` has the same underlying math as a standalone, manual
two-part tool (no ROS2 dependency for the actual calibration step — just
numpy) — useful as a fallback if the auto-detected plane isn't finding
the right surface, or for offline analysis away from the rig:

1. **Capture raw calibration data** — run this on the Pi (or anywhere
   on the same ROS2 network) *while* a normal scan with real overlap
   (range past ~150°) runs as usual via the GUI:
   ```bash
   source ~/micromamba/envs/ros2/setup.bash
   source ~/TPL_LIDAR/ros2_ws/install/setup.bash
   python3 scripts/calibration/capture_raw_for_mount_calibration.py -o mount_calibration_raw.npz
   ```
   Ctrl+C once the scan finishes to save. This listens to the *raw*
   `/velodyne_points` (sensor frame, before the mount transform) tagged
   with the tilt angle at capture time — it doesn't drive anything, and
   doesn't touch the running scan.

2. **Calibrate offline** — copy the `.npz` anywhere with Python+numpy
   (doesn't need to be the Pi) and run:
   ```bash
   python3 scripts/calibration/calibrate_mount_angle.py \
       -i mount_calibration_raw.npz --roi <xmin> <xmax> <ymin> <ymax> <zmin> <zmax>
   ```
   `--roi` is a bounding box (base_link-frame metres) around a flat
   surface (a wall) visible in the overlap/double-image region — use
   whatever viewer you already spotted the artifact in to estimate it
   against the *current* calibration. Prints the optimized
   `mount_roll_deg`/`mount_pitch_deg` plus the before/after RMS
   plane-fit error, so you can judge whether the improvement is real
   before adopting the new values (GUI's Config > VLP-16 page, or
   directly in `~/.lidar_scanner_settings.json`).

**`mount_yaw_deg` is deliberately not calibrated by this tool** — not a
missing feature, a real mathematical fact confirmed both algebraically
and with a synthetic test while building this: the tilt joint only ever
rotates about a single fixed axis, and yaw composes with that rotation
as a pure additive offset to every point's effective tilt angle — so any
yaw error is exactly equivalent to rotating the *entire* output rigidly
about that axis, which preserves every internal geometric relationship
(flatness, angles between surfaces, all of it) perfectly. No
self-consistency check, however many non-parallel surfaces you throw at
it, can ever distinguish the true yaw from a wrong one — the same
limitation as a magnetometer-free IMU getting roll/pitch from gravity
but never absolute heading. This doesn't matter for the reported bug
either way: a yaw error can't create or explain internal doubling, only
roll/pitch can, and the double-image symptom is fully explained by those
two alone. See the calibration script's own module docstring for the
full derivation.
