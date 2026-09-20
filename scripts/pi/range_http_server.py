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
import os
import re
import sys

RANGE_RE = re.compile(r"^bytes=(\d*)-(\d*)$")


class RangeRequestHandler(http.server.SimpleHTTPRequestHandler):
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
