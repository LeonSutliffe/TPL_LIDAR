# Applies the TPL brand icon to the "Start LiDAR Scanner" desktop shortcut.
# The .lnk itself isn't tracked in this repo (lives on the Desktop, made by
# hand) -- re-run this any time the shortcut gets recreated, to reapply the
# icon without needing to redo it manually in Explorer's Properties dialog.

$ErrorActionPreference = 'Stop'

$shortcutPath = "D:\Desktop\Start LiDAR Scanner.lnk"
$iconPath = "D:\Downloads\LIDAR\branding\tpl_logo.ico"

if (-not (Test-Path $shortcutPath)) {
    Write-Host "No shortcut found at '$shortcutPath' -- nothing to do." -ForegroundColor Yellow
    exit 1
}
if (-not (Test-Path $iconPath)) {
    Write-Host "Icon not found at '$iconPath'." -ForegroundColor Yellow
    exit 1
}

$shell = New-Object -ComObject WScript.Shell
$shortcut = $shell.CreateShortcut($shortcutPath)
$shortcut.IconLocation = "$iconPath,0"
$shortcut.Save()

Write-Host "Set icon on '$shortcutPath' -> '$iconPath'" -ForegroundColor Green
