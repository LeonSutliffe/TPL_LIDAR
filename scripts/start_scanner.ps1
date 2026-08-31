# Starts the full LiDAR scanner stack and opens the control GUI.
#
# What this does, in order:
#   1. Wakes WSL2 (it shuts itself down after being idle, which also drops
#      any usbipd-attached devices -- confirmed repeatedly this session).
#   2. Finds the USB<->RS485 bridge by VID:PID (0403:6001, FTDI FT232-
#      family -- replaced the original Pico bridge 2026-08-31; confirmed
#      via Get-PnpDevice, not CH340 despite initially assumed) and
#      attaches it to WSL2. Looked up by VID:PID rather than a fixed
#      busid, since the busid has changed more than once after replugging.
#   3. Launches scanner_bringup's full bringup.launch.py in its own
#      console window, so you can watch its logs / Ctrl+C it independently
#      of this script.
#   4. Waits for rosbridge to come up, then opens the control GUI in your
#      default browser.
#
# Safe to re-run any time -- attaching an already-attached device or
# launching against an already-running stack just no-ops/creates a second
# window; it won't corrupt anything.

$ErrorActionPreference = 'Continue'

$usbipdExe = "C:\Program Files\usbipd-win\usbipd.exe"
$repoRoot = "D:\Downloads\LIDAR"
$guiPath = Join-Path $repoRoot "web\tilt_axis_gui\index.html"

Write-Host "== Waking WSL2 ==" -ForegroundColor Cyan
wsl.exe -e bash -lc "echo ready" | Out-Null

Write-Host "== Attaching USB<->RS485 bridge (VID:PID 0403:6001) ==" -ForegroundColor Cyan
if (Test-Path $usbipdExe) {
    $line = & $usbipdExe list | Select-String "0403:6001"
    if ($line) {
        $busid = ($line.ToString().Trim() -split '\s+')[0]
        Write-Host "Found at busid $busid, attaching..."
        & $usbipdExe attach --wsl --busid $busid
    } else {
        Write-Host "WARNING: bridge not found in 'usbipd list' -- is it plugged in?" -ForegroundColor Yellow
    }
} else {
    Write-Host "WARNING: usbipd-win not found at '$usbipdExe' -- skipping USB attach." -ForegroundColor Yellow
}

Write-Host "== Launching ROS2 scanner stack (new window) ==" -ForegroundColor Cyan
# Calling a plain script path here, not an inline quoted command string --
# confirmed this session that Start-Process -ArgumentList mangles a string
# with nested quotes/&& by the time it reaches wsl.exe/bash/micromamba.
Start-Process wsl.exe -ArgumentList @("-e", "bash", "/mnt/d/Downloads/LIDAR/scripts/launch_stack.sh")

Write-Host "== Waiting for rosbridge to come up ==" -ForegroundColor Cyan
Start-Sleep -Seconds 8

Write-Host "== Opening control GUI ==" -ForegroundColor Cyan
Start-Process $guiPath

Write-Host ""
Write-Host "Done. The ROS2 stack is running in its own window -- close that" -ForegroundColor Green
Write-Host "window (or Ctrl+C in it) to stop the scanner." -ForegroundColor Green
Write-Host ""
Write-Host "Press Enter to close this window..."
Read-Host | Out-Null
