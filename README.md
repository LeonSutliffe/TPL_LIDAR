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
   USB              │  Raspberry Pi Pico       │  RS485 (A/B)
   ─────────────────┤  transparent byte bridge ├──────────► MKS SERVO42D driver ─► stepper
                    └─────────────────────────┘                                    + tilt axis

ROS2 graph:

  tilt_axis_bridge          scan_aggregator          vlp16_config
  owns the Pico serial      drives step-and-stare/    owns the VLP-16's
  link, homing/move/sweep   sweep, tf2-transforms      HTTP config API +
  state machine             + merges clouds into a     tilt->sensor mount
                             single .pcd per run        offset

  velodyne_driver_node → velodyne_transform_node → /velodyne_points

  rosbridge_websocket (ws://<host>:9090) ── serves the browser GUI
  (web/tilt_axis_gui/index.html — single self-contained HTML file,
  no build step)
```

A Raspberry Pi Pico runs a minimal USB↔RS485 byte-pipe bridge (firmware
outside this repo — see `HANDOFF.md`'s "Architecture pivot" section for
why); everything else — all MKS protocol logic, the scan state machines,
tf2 transforms, the point cloud merge — runs as plain ROS2 nodes on
whatever machine `ros2 launch` runs on. That machine is what this guide
is about moving from a Windows/WSL2 laptop onto a standalone Raspberry
Pi 4, so the whole rig no longer depends on being tethered to a laptop.

## Repo layout

| Path | What it is |
|---|---|
| `ros2_ws/src/tilt_axis_bridge` | Owns the Pico serial link; MKS driver protocol, homing/move/sweep state machine |
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
- Raspberry Pi Pico as a USB↔RS485 bridge (plain serial device to the host)
- A hard project-wide safety cap of **40 RPM** and a **0–260°** rotation
  limit are enforced in code (`tilt_axis_bridge`), not just convention

## Deploying on a Raspberry Pi 4 (in progress)

**Status: not yet validated end-to-end** — this is the plan as of
2026-08-29, written before the Pi hardware arrived. The Windows/WSL2 setup
(`HANDOFF.md`'s "Running it" section) is the currently-working reference;
treat the steps below as the intended path, not a proven one yet, and
update this section once each step is actually confirmed on real hardware.

### 1. Flash the OS

Use **Raspberry Pi Imager**, choose **Raspberry Pi OS (64-bit)** — 64-bit
matters, that's what gets aarch64 ROS2 packages. Before writing, open the
advanced options (gear icon) and set:

- a hostname (e.g. `tpl-scanner`)
- SSH enabled, with your public key (or a password if you don't have a
  key pair yet)
- WiFi SSID/password, if the Pi won't be on Ethernet immediately

This gets the Pi to a headless, SSH-reachable state on first boot — no
monitor or keyboard needed for initial setup.

### 2. Connect the hardware

- VLP-16 → the Pi's Ethernet port (direct, or via switch)
- Pico → any Pi USB port

### 3. Get the code onto the Pi

```bash
git clone https://github.com/LeonSutliffe/TPL_LIDAR.git
cd TPL_LIDAR
```

### 4. Install ROS2 — method TBD

The Windows/WSL2 setup uses conda/RoboStack (`micromamba`), not a system
package manager. RoboStack does publish `aarch64` builds, but it has
**not yet been confirmed** that every package this project needs —
`velodyne_driver` and `velodyne_pointcloud` specifically — is actually
available in that channel for ARM64. Two paths to try, in order:

1. **RoboStack via micromamba** (same tooling as the laptop, most likely
   to behave identically): install micromamba, create a `ros2` env the
   same way the Windows/WSL2 side does, and see whether
   `velodyne_driver`/`velodyne_pointcloud` resolve for `linux-aarch64`.
2. **Native ROS2 Debian packages** (`apt`) on Raspberry Pi OS, if
   RoboStack comes up short on `aarch64` package availability for this
   dependency set. Not yet investigated.

### 5. Build the workspace

```bash
source /path/to/ros2/setup.bash   # wherever step 4 put it
cd ros2_ws
colcon build
```

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
ros2 launch scanner_bringup bringup.launch.py
```

Useful launch args (see `scanner_bringup/launch/bringup.launch.py`):
`enable_pointcloud:=false` (skip point cloud conversion if not needed),
`rviz:=false` (skip the live-preview window — sensible for a headless
Pi), `record_bag:=true` (raw packet + tf recording).

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

### Open questions for this deployment

- Onboard screen not yet decided — leaning toward either a small
  HDMI/DSI touchscreen on the Pi, or reusing an old phone purely as a
  browser client for the GUI over the Pi's own WiFi hotspot (not yet
  built).
- Real per-point processing load on a Pi 4 (vs. the dev machine's many
  cores) is unverified — the merge lives entirely in RAM
  (`scan_aggregator`), and a real scan can be tens of millions of points.

See `HANDOFF.md` for the full decision history on why a Pi 4 was chosen
over a Pi 5, an x86 mini PC, an old Android phone as compute, and an
Intel Compute Stick.
