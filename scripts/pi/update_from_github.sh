#!/bin/bash
# Pull the latest code from GitHub, rebuild, and restart the scanner.
#
# This is what the full GUI's Config -> General -> "Update from GitHub"
# button runs (via POST /api/update on range_http_server.py). It is the
# same steps as a manual deploy: git pull, stop tpl-scanner.service,
# colcon build, start the service again.
#
# It must NOT run as a child of any tpl-* service: stopping
# tpl-scanner.service (or restarting the GUI's own HTTP server at the end)
# would kill it mid-update, because systemd kills every process in a
# service's cgroup (KillMode=control-group; see HANDOFF.md's 2026-09-11
# "poweroff genuinely never worked" entry). range_http_server.py therefore
# launches it with `sudo systemd-run --unit=tpl-update --uid=tpl ...`, which
# puts it in its own transient unit owned by PID 1. That unit name doubles
# as a lock: systemd refuses to start a second tpl-update while one is
# running.
#
# Progress is written to $STATE_DIR/status.json (read by GET
# /api/update/status) and the full output to $STATE_DIR/update.log.
#
# Can also be run by hand over SSH: bash scripts/pi/update_from_github.sh
#
# Everything lives inside main() and the last line is `main "$@"; exit`:
# bash reads a script file as it runs, and `git pull` below may rewrite
# this very file. Wrapping it in a function makes bash parse the whole
# thing before the pull happens.

main() {
  REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
  WS_DIR="$REPO_DIR/ros2_ws"
  ROS_SETUP="${TPL_ROS_SETUP:-$HOME/micromamba/envs/ros2/setup.bash}"
  BRANCH="${TPL_UPDATE_BRANCH:-main}"
  STATE_DIR="${TPL_UPDATE_STATE_DIR:-$HOME/.tpl_update}"
  STATUS="$STATE_DIR/status.json"
  LOG="$STATE_DIR/update.log"
  # Set TPL_UPDATE_DRY_RUN=1 to test without touching services or building.
  DRY_RUN="${TPL_UPDATE_DRY_RUN:-0}"

  mkdir -p "$STATE_DIR"
  : > "$LOG"
  exec >>"$LOG" 2>&1

  STARTED="$(date -Is)"
  OLD_REV=""
  NEW_REV=""

  log "Update started (branch $BRANCH, repo $REPO_DIR)"
  cd "$REPO_DIR" || fail "repo folder $REPO_DIR not found"

  OLD_REV="$(git rev-parse --short HEAD)"
  set_status running "Checking for local changes" ""

  # Tracked files edited on the Pi would make the pull fail half way or be
  # silently merged; refuse and say which files instead.
  local dirty
  dirty="$(git status --porcelain --untracked-files=no)"
  if [ -n "$dirty" ]; then
    log "Local changes:"; log "$dirty"
    fail "files in $REPO_DIR were edited on the Pi, so nothing was changed. Commit or discard them first (git status shows which)."
  fi

  set_status running "Downloading from GitHub" ""
  if ! run git fetch origin "$BRANCH"; then
    fail "could not reach GitHub. Is the Pi on home Wi-Fi with internet (not its own hotspot)?"
  fi

  local target
  target="$(git rev-parse --short "origin/$BRANCH")"
  if [ "$target" = "$OLD_REV" ]; then
    NEW_REV="$OLD_REV"
    set_status done "Already up to date ($OLD_REV); nothing to do" ""
    log "Already up to date."
    return 0
  fi

  if ! run git merge --ff-only "origin/$BRANCH"; then
    fail "the Pi's copy has commits that are not on GitHub, so it can't fast-forward. Fix by hand over SSH."
  fi
  NEW_REV="$(git rev-parse --short HEAD)"
  log "Code updated $OLD_REV -> $NEW_REV:"
  git log --oneline "$OLD_REV..$NEW_REV"

  set_status running "Stopping the scanner" ""
  svc stop tpl-scanner.service || log "WARNING: stopping tpl-scanner.service failed; building anyway"

  set_status running "Building (this takes a few minutes)" ""
  local build_ok=1
  if [ "$DRY_RUN" = "1" ]; then
    log "[dry run] would colcon build in $WS_DIR"
  else
    # shellcheck disable=SC1090
    if ! ( cd "$WS_DIR" && source "$ROS_SETUP" && colcon build ); then
      build_ok=0
    fi
  fi

  set_status running "Starting the scanner" ""
  if ! svc start tpl-scanner.service; then
    fail "the new code was built but tpl-scanner.service did not start. See the log."
  fi

  # Give it a few seconds, then make sure it didn't crash straight away.
  if [ "$DRY_RUN" != "1" ]; then
    sleep 8
    if ! systemctl is-active --quiet tpl-scanner.service; then
      fail "tpl-scanner.service stopped again right after starting. See journalctl -u tpl-scanner.service."
    fi
  fi

  if [ "$build_ok" != "1" ]; then
    fail "colcon build failed (scanner restarted anyway, but may be running a mix of old and new code). See the log."
  fi

  set_status done "Updated $OLD_REV -> $NEW_REV. Reload this page to get the new GUI." ""
  log "Update finished OK."

  # Last, restart the GUI's own web servers so changes to
  # range_http_server.py take effect. The status file is already final, so
  # the page just sees a brief gap in polling.
  svc restart tpl-gui-http-full.service tpl-gui-http.service \
    || log "WARNING: restarting the GUI web servers failed"
  return 0
}

log() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

run() { log "\$ $*"; "$@"; }

svc() {
  if [ "$DRY_RUN" = "1" ]; then
    log "[dry run] systemctl $*"
    return 0
  fi
  run sudo -n systemctl "$@"
}

json_str() {
  # Minimal JSON string escaping for our own short messages.
  local s="$1"
  s="${s//\\/\\\\}"; s="${s//\"/\\\"}"; s="${s//$'\n'/ }"; s="${s//$'\t'/ }"
  printf '"%s"' "$s"
}

set_status() {
  local state="$1" step="$2" error="$3"
  local tmp="$STATUS.tmp"
  {
    printf '{"state":%s,"step":%s,"error":%s,' "$(json_str "$state")" "$(json_str "$step")" "$(json_str "$error")"
    printf '"old_rev":%s,"new_rev":%s,' "$(json_str "$OLD_REV")" "$(json_str "$NEW_REV")"
    printf '"started":%s,"updated":%s}\n' "$(json_str "$STARTED")" "$(json_str "$(date -Is)")"
  } > "$tmp" && mv -f "$tmp" "$STATUS"
}

fail() {
  log "ERROR: $1"
  set_status failed "Update failed" "$1"
  exit 1
}

main "$@"; exit
