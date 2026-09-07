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
nodes on whatever machine `ros2 launch` runs on. That machine is what
this guide is about moving from a Windows/WSL2 laptop onto a standalone
Raspberry Pi 4, so the whole rig no longer depends on being tethered to
a laptop.

## Repo layout

| Path | What it is |
|---|---|
| `ros2_ws/src/tilt_axis_bridge` | Owns the bridge serial link; MKS driver protocol, homing/move/sweep state machine |
| `ros2_ws/src/scan_aggregator` | Drives a scan (either mode), tf2-transforms and merges clouds, writes the output `.pcd` |
| `ros2_ws/src/vlp16_config` | VLP-16 hardware config (its own HTTP API) + the tilt→sensor mount-offset transform |
| `ros2_ws/src/scanner_bringup` | Velodyne driver launch + the full-stack `bringup.launch.py` |
| `ros2_ws/src/scanner_description` | URDF/xacro + `robot_state_publisher` |
| `web/tilt_axis_gui/index.html` | The control GUI — connects to rosbridge over WebSocket, no build step |
| `scripts/` | Windows/WSL2 launch helpers (not used on the Pi) |

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

**Status, 2026-09-07: steps 1–5 confirmed working on real hardware**
(OS/SSH/repo/RoboStack-via-micromamba/`colcon build`, verified live over
SSH to the actual Pi — see step 4 for exactly what was confirmed).
Step 2 onward (hardware wiring, USB storage, hotspot, systemd, onboard
screen) is not yet exercised together as a full end-to-end run. The
Windows/WSL2 setup (`HANDOFF.md`'s "Running it" section) remains the
proven day-to-day reference in the meantime.

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

### 2. Connect the hardware

- VLP-16 → the Pi's Ethernet port (direct, or via switch)
- USB<->RS485 bridge adapter → any Pi USB port

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

### 6. Set up automatic USB storage (optional, recommended)

Goal: scans land on a USB stick with nothing to do on the Pi each
session — no manual `mount`, and `output_dir` set once and never touched
again regardless of which physical stick is plugged in.

- Format the stick as **exFAT**, not FAT32 (4GB-per-file cap — your scans
  already exceed that) or NTFS (weaker Linux write support). exFAT is
  also directly readable on Windows when you pull the stick to grab data.
- `sudo apt install exfatprogs` (exFAT support) and `sudo mkdir -p
  /mnt/tpl_usb` (a fixed mount point).
- Add a udev rule so *any* USB storage device plugged in automatically
  mounts to that fixed path, via `systemd-mount` (built into Raspberry
  Pi OS already — no extra package needed). Create
  `/etc/udev/rules.d/99-usb-automount.rules`:

  ```
  ACTION=="add", SUBSYSTEM=="block", KERNEL=="sd[a-z][0-9]", ENV{ID_FS_USAGE}=="filesystem", RUN+="/usr/bin/systemd-mount --no-block --collect --automount=yes -o uid=1000,gid=1000 $env{DEVNAME} /mnt/tpl_usb"
  ACTION=="remove", SUBSYSTEM=="block", KERNEL=="sd[a-z][0-9]", RUN+="/usr/bin/systemd-umount /mnt/tpl_usb"
  ```

  then `sudo udevadm control --reload-rules`. `uid=1000,gid=1000` matches
  Raspberry Pi OS's default `pi` user, so the ROS2 process can write to
  the mount without a permissions fight — adjust if the stack actually
  runs as a different user.
- Set `scan_aggregator`'s `output_dir` parameter to `/mnt/tpl_usb`, once
  — either via the GUI (Config > General page) or directly in
  `~/.lidar_scanner_settings.json`. It already persists across restarts
  on its own from there.

**Not yet tested.** Also: this assumes one USB storage device plugged in
at a time — a second one wouldn't get the fixed mount point (the rule
targets one path), which is fine for a single dedicated data stick but
worth knowing if that changes later.

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

- **The Pi becomes its own access point.** Raspberry Pi OS (Bookworm)
  uses NetworkManager by default, which has this built in — no
  hostapd/dnsmasq hand-rolling needed:

  ```bash
  sudo nmcli connection add type wifi ifname wlan0 con-name TPL-Hotspot \
      autoconnect yes ssid TPL-Scanner
  sudo nmcli connection modify TPL-Hotspot 802-11-wireless.mode ap \
      802-11-wireless.band bg ipv4.method shared
  sudo nmcli connection modify TPL-Hotspot wifi-sec.key-mgmt wpa-psk \
      wifi-sec.psk "change-this-password"
  sudo nmcli connection up TPL-Hotspot
  ```

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
    `sudo chmod +x /usr/local/bin/tpl-wifi-*` after creating both. Running
    `tpl-wifi-home` from an active hotspot-connected SSH session will
    drop that session immediately (expected — the switch itself is what
    disconnects it), but completes on the Pi regardless of whether
    anything was there to see the output.

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

**Not yet tested** (no hotspot-capable hardware to test against yet).
The GUI's smarter address default *was* tested locally (served over a
plain HTTP server, confirmed the field auto-fills correctly; confirmed
the `file://` fallback is unaffected, since `location.hostname` is
empty there by spec).

### 9. Auto-start everything on boot (systemd)

Ties the above together into an actual zero-touch device: power it on,
wait, join `TPL-Scanner` from a phone, done — no SSH needed for normal
field use (still available for changes/debugging).

`/etc/systemd/system/tpl-scanner.service`:
```ini
[Unit]
Description=TPL scanner ROS2 stack
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/TPL_LIDAR/ros2_ws
ExecStart=/bin/bash -c 'source install/setup.bash && ros2 launch scanner_bringup bringup.launch.py rviz:=false'
Restart=on-failure

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
User=pi
WorkingDirectory=/home/pi/TPL_LIDAR/web/tilt_axis_gui
ExecStart=/usr/bin/python3 -m http.server 8080
Restart=on-failure

[Install]
WantedBy=multi-user.target
```

```bash
sudo systemctl enable --now tpl-scanner.service tpl-gui-http.service
```

Also add `TPL-Hotspot`'s `autoconnect yes` (already set above) so the
hotspot itself comes back on its own after a power cycle too — the three
together (hotspot, ROS2 stack, GUI server) are what make this a genuinely
self-contained device rather than one that still needs an SSH session
after every boot. **Not yet tested.**

### 10. Onboard screen (Elecrow RR035 / ELEGOO 3.5" GPIO touchscreen)

**Identified 2026-08-29**: a 3.5" GPIO/SPI panel, 480×320, XPT2046
resistive touch (confirmed via the vendor's own product page — not the
380×420 figure floated earlier). Intent: have it act like a real HDMI
monitor would — boot messages, a login prompt, a full interactive shell
for general debugging — not just a fixed custom status readout. That's
a genuinely different, and better-supported, goal than the narrower
"just show status text" plan from earlier the same day: this class of
panel is commonly used exactly this way.

**Getting the panel itself recognized** — researched properly rather
than guessed, since getting this wrong would send a real debugging
session down a dead end. Two viable routes, in order of preference:

1. **Mainline kernel overlay (try first)** — current Raspberry Pi OS
   (Bookworm) ships a `piscreen` DRM overlay that a Raspberry Pi
   engineer confirmed working for this *exact* combination (3.5",
   480×320, XPT2046 touch) in a
   [2026 forum thread](https://forums.raspberrypi.com/viewtopic.php?t=382506).
   No third-party driver package needed — one line in
   `/boot/firmware/config.txt`:
   ```
   dtoverlay=piscreen,drm,speed=18000000
   ```
   If the touch axes come out inverted/swapped, the same overlay takes
   `invx`, `invy`, `swapxy` (append as `,invx` etc.) — confirmed real,
   documented parameters, not a guess.
2. **Vendor driver script (fallback)** — if the mainline overlay doesn't
   play nicely with this specific panel revision. Elecrow's own current
   repo: `git clone https://github.com/Elecrow-keen/Elecrow-LCD35.git &&
   cd Elecrow-LCD35 && sudo ./Elecrow-LCD35` (there's also an older,
   more generic `goodtft/LCD-show` that Elecrow's own wiki still
   references — worth knowing that repo exists too, though it predates
   Bookworm/Pi4 entirely and shouldn't be the first thing tried). **Real,
   specific gotcha found for this route on a Pi 4**: on system images
   after 2021-10-30, `/boot/firmware/config.txt`'s `dtoverlay=vc4-kms-v3d`
   needs changing to `dtoverlay=vc4-fkms-v3d` first, or the installer
   fails to start.

Neither route has been tried against the real panel yet — no hardware.
Whichever works, the result is the same either way: the panel becomes a
normal Linux console, so **no panel-specific Python graphics library is
needed anywhere in this project** — plain terminal output already
reaches it, the same as it would reach any other console.

**Optional extra**: [`scripts/pi/status_display.py`](scripts/pi/status_display.py)
— a small standalone script (not a colcon package, nothing here needs
its own topics/services) that subscribes to `/tilt_axis_bridge/status`
and `/scan_aggregator/status` (the exact same topics the web GUI itself
reads) and prints a compact live status view once a second via plain
ANSI clear-screen + text. Meant to be run **on demand** when you want a
quick glance, not forced onto the console in place of a normal login
prompt — that would work against the "acts like an HDMI monitor for
debugging" goal this whole section exists for. Its ROS2/networking half
is real, tested code (isolated-`ROS_DOMAIN_ID` test: subscription
callbacks against a throwaway publisher, `current_ip()` against both a
real interface and a missing one — see the file's own header); the
terminal rendering is plain ANSI escape codes with no panel-specific
unknown left in it, but hasn't been eyeballed against the real console
yet.

### Open questions for this deployment

- Which of step 10's two panel-recognition routes (mainline `piscreen`
  overlay vs. Elecrow's vendor driver script) actually works on this
  specific panel revision — neither tried against real hardware yet.
- Real per-point processing load on a Pi 4 (vs. the dev machine's many
  cores) is unverified — the merge lives entirely in RAM
  (`scan_aggregator`), and a real scan can be tens of millions of points.
  Somewhat de-risked by dropping `rviz2` (step 10 above), since the Pi no
  longer needs to spend any headroom on rendering
  anything itself.

See `HANDOFF.md` for the full decision history on why a Pi 4 was chosen
over a Pi 5, an x86 mini PC, an old Android phone as compute, and an
Intel Compute Stick.
