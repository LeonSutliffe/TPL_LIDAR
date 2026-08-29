# One-time: install WSL2 (needs admin, can't be done from here)

The ESP32-S2 firmware build needs a real POSIX environment because
`micro_ros_espidf_component` compiles the micro-ROS client library via a
`colcon`-based Makefile that assumes `pwd` and other Unix shell semantics.
Native Windows `make` doesn't provide that, and `idf.py` itself refuses to
run under Git Bash/MSYS. WSL2 is the standard fix.

## 1. Install WSL2 (you need to do this step — requires admin elevation)

Open **PowerShell as Administrator** (right-click Start → "Windows Terminal
(Admin)" or search PowerShell, right-click, Run as administrator) and run:

```powershell
wsl --install --distribution Ubuntu
```

This enables the `Microsoft-Windows-Subsystem-Linux` and
`VirtualMachinePlatform` Windows features and installs Ubuntu. **It will
likely ask you to reboot** — do that, then Ubuntu will finish its first-run
setup (pick a Linux username/password) automatically on next login.

## 2. Everything after that, I can do for you

Once WSL/Ubuntu is up (just confirm it's done, or run `wsl -l -v` from a
normal, non-admin PowerShell/terminal to check), tell me and I'll:

- Install ESP-IDF's Linux prerequisites and clone/set up ESP-IDF inside
  Ubuntu (a fresh clone — the `D:\esp\esp-idf` Windows checkout can't be
  reused directly since its Windows-downloaded toolchain binaries won't run
  under Linux)
- Point the build at this project (`D:\Downloads\LIDAR\firmware\tilt_controller`,
  reachable from WSL at `/mnt/d/Downloads/LIDAR/firmware/tilt_controller`)
- Run `idf.py build` and fix any remaining compile errors

## Why not just keep debugging around it from here

I could keep patching around missing POSIX tools (a `pwd.exe`, a proper
`SHELL` for make, etc.) one at a time, but that's fighting the same
fundamental mismatch repeatedly instead of fixing it once. WSL2 is also
what you'd want long-term for this project anyway, since the PC-side ROS2
work (aggregator node, `rosbag2`, etc.) is far more at home on Linux than
native Windows.
