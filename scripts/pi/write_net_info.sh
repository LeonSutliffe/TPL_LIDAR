#!/bin/bash
# Writes web/tilt_axis_gui/net_info.json for status.html (the onboard
# screen's kiosk status display) to poll every few seconds -- the current
# wlan0 IP and connected SSID. Run periodically by tpl-net-info.timer,
# not baked into status.html itself since a browser page can't query
# network interfaces directly; a tiny static JSON file it can fetch() is
# the simplest bridge. eth0 deliberately omitted -- it's the VLP-16's
# dedicated point-to-point link, not something a phone/tablet ever
# connects through, so not useful on this particular display.
OUT="/home/tpl/TPL_LIDAR/web/tilt_axis_gui/net_info.json"

WLAN_IP=$(ip -4 -o addr show wlan0 2>/dev/null | awk '{print $4}' | cut -d/ -f1)
SSID=$(nmcli -t -f active,ssid dev wifi 2>/dev/null | awk -F: '$1=="yes"{print $2}')

printf '{"wlan0_ip":"%s","ssid":"%s"}\n' "${WLAN_IP:-}" "${SSID:-}" > "$OUT"
