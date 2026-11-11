#!/usr/bin/env python3
"""Static file server with HTTP Range (partial content) support.

Drop-in replacement for `python3 -m http.server <port>` -- stdlib's
http.server always answers with the whole file from byte 0 and ignores
any `Range` header, which breaks resumable/segmented downloads for
anything large enough that a browser resumes or splits it. Found
2026-09-20 from two real reports against `tpl-gui-http-full.service`
(port 8081, the server both GUIs' scan/zip downloads actually pull
from via `scansOrigin()`): the simplified GUI's "Download All (zip)"
failing outright for project bundles, and any single file over ~1GB
hanging with the browser stuck saying "Resuming..." -- both are the
textbook symptom of a client sending `Range: bytes=...` (Chrome does
this for large downloads, and always on a retry/resume) and getting
back a fresh 200 with the full file instead of the requested slice, so
the download manager can never reconcile what it already has.

Directory listings and everything else are delegated straight to
`http.server.SimpleHTTPRequestHandler` unchanged -- only a GET/HEAD for
an actual file, with or without a `Range` header, is handled here, so
normal (non-range) behavior for the GUIs' own HTML/JS/CSS assets is
identical to before.
"""
import http.server
import json
import os
import re
import subprocess
import sys
import time

RANGE_RE = re.compile(r"^bytes=(\d*)-(\d*)$")

# ---- "Update from GitHub" (full GUI, Config -> General) ----
#
# POST /api/update starts scripts/pi/update_from_github.sh in its own
# transient systemd unit (tpl-update), so it survives the script stopping
# tpl-scanner.service and restarting this very server. GET
# /api/update/status reports its progress. Lives here rather than in a ROS
# node because this server keeps running while the ROS stack is down for
# the rebuild.
REPO_DIR = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
UPDATE_SCRIPT = os.path.join(REPO_DIR, "scripts", "pi", "update_from_github.sh")
UPDATE_UNIT = "tpl-update"
UPDATE_STATE_DIR = os.environ.get("TPL_UPDATE_STATE_DIR", os.path.expanduser("~/.tpl_update"))
UPDATE_STATUS = os.path.join(UPDATE_STATE_DIR, "status.json")
UPDATE_LOG = os.path.join(UPDATE_STATE_DIR, "update.log")
LOG_TAIL_LINES = 60


def _cmd(args, timeout=10):
    try:
        r = subprocess.run(args, capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr).strip()
    except (OSError, subprocess.SubprocessError) as e:
        return -1, str(e)


def _update_unit_active():
    code, out = _cmd(["systemctl", "is-active", UPDATE_UNIT + ".service"])
    return out.strip() in ("active", "activating", "reloading")


def _current_version():
    code, out = _cmd(["git", "-C", REPO_DIR, "log", "-1", "--format=%h%x09%cd%x09%s", "--date=format:%Y-%m-%d %H:%M"])
    if code != 0 or "\t" not in out:
        return None
    rev, date, subject = out.split("\t", 2)
    return {"rev": rev, "date": date, "subject": subject}


def _write_status(status):
    os.makedirs(UPDATE_STATE_DIR, exist_ok=True)
    tmp = UPDATE_STATUS + ".tmp"
    with open(tmp, "w") as f:
        json.dump(status, f)
    os.replace(tmp, UPDATE_STATUS)


def update_status():
    try:
        with open(UPDATE_STATUS) as f:
            status = json.load(f)
    except (OSError, ValueError):
        status = {"state": "idle", "step": "", "error": ""}
    running = _update_unit_active()
    # The script was killed (or the Pi rebooted) before it could record an
    # outcome -- don't leave the GUI saying "running" forever.
    if status.get("state") in ("starting", "running") and not running:
        started = status.get("started_epoch", 0)
        if time.time() - started > 15:
            status["state"] = "failed"
            status["error"] = status.get("error") or "the updater stopped unexpectedly. See the log below."
    try:
        with open(UPDATE_LOG, errors="replace") as f:
            status["log"] = "".join(f.readlines()[-LOG_TAIL_LINES:])
    except OSError:
        status["log"] = ""
    status["running"] = running
    status["version"] = _current_version()
    return status


def start_update():
    if _update_unit_active():
        return 409, {"ok": False, "error": "An update is already running."}
    _write_status({"state": "starting", "step": "Starting updater", "error": "",
                   "started_epoch": time.time()})
    user = os.environ.get("USER") or subprocess.run(["id", "-un"], capture_output=True, text=True).stdout.strip()
    code, out = _cmd([
        "sudo", "-n", "systemd-run",
        "--unit=" + UPDATE_UNIT, "--collect",
        "--uid=" + user,
        "--setenv=HOME=" + os.path.expanduser("~"),
        "--setenv=TPL_UPDATE_STATE_DIR=" + UPDATE_STATE_DIR,
        "/bin/bash", UPDATE_SCRIPT,
    ], timeout=20)
    if code != 0:
        msg = "Could not start the updater: " + (out or "exit code %d" % code)
        _write_status({"state": "failed", "step": "Update failed", "error": msg, "started_epoch": time.time()})
        return 500, {"ok": False, "error": msg}
    return 202, {"ok": True}


class RangeRequestHandler(http.server.SimpleHTTPRequestHandler):
    def _send_json(self, code, obj):
        body = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path.split("?", 1)[0] == "/api/update/status":
            return self._send_json(200, update_status())
        return super().do_GET()

    def do_POST(self):
        if self.path.split("?", 1)[0] == "/api/update":
            code, obj = start_update()
            return self._send_json(code, obj)
        self.send_error(404, "Not found")

    def send_head(self):
        path = self.translate_path(self.path)
        if not os.path.isfile(path):
            self._range_remaining = None
            return super().send_head()

        try:
            f = open(path, "rb")
        except OSError:
            self.send_error(404, "File not found")
            return None

        fs = os.fstat(f.fileno())
        file_len = fs.st_size
        start, end, status = 0, file_len - 1, 200

        range_header = self.headers.get("Range")
        if range_header:
            match = RANGE_RE.match(range_header.strip())
            if not match or (match.group(1) == "" and match.group(2) == ""):
                f.close()
                self.send_error(416, "Invalid Range header")
                return None
            start_s, end_s = match.groups()
            if start_s == "":
                # Suffix range ("bytes=-500" -> last 500 bytes).
                start = max(0, file_len - int(end_s))
                end = file_len - 1
            else:
                start = int(start_s)
                end = int(end_s) if end_s != "" else file_len - 1
            if start >= file_len or start > end:
                f.close()
                self.send_response(416)
                self.send_header("Content-Range", f"bytes */{file_len}")
                self.end_headers()
                return None
            end = min(end, file_len - 1)
            status = 206

        self.send_response(status)
        self.send_header("Content-type", self.guess_type(path))
        self.send_header("Accept-Ranges", "bytes")
        self.send_header("Content-Length", str(end - start + 1))
        if status == 206:
            self.send_header("Content-Range", f"bytes {start}-{end}/{file_len}")
        self.send_header("Last-Modified", self.date_time_string(int(fs.st_mtime)))
        self.end_headers()

        f.seek(start)
        self._range_remaining = end - start + 1
        return f

    def copyfile(self, source, outputfile):
        remaining = getattr(self, "_range_remaining", None)
        if remaining is None:
            return super().copyfile(source, outputfile)
        bufsize = 256 * 1024
        while remaining > 0:
            chunk = source.read(min(bufsize, remaining))
            if not chunk:
                break
            try:
                outputfile.write(chunk)
            except (BrokenPipeError, ConnectionResetError):
                break
            remaining -= len(chunk)


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    server = http.server.ThreadingHTTPServer(("", port), RangeRequestHandler)
    server.serve_forever()


if __name__ == "__main__":
    main()
