#!/usr/bin/env bash
set -u
mkdir -p ~/.espressif/dist
cd ~/.espressif/dist

python3 /mnt/d/Downloads/LIDAR/firmware/tilt_controller/gen_dl_list.py > /tmp/dl_list.txt
cat /tmp/dl_list.txt

while IFS='|' read -r url sha fname; do
  [ -z "$fname" ] && continue
  echo "=== $fname ==="
  rm -f "${fname}.tmp"
  ok=0
  for attempt in 1 2 3 4; do
    curl -sL --retry 8 --retry-all-errors -C - -o "$fname" "$url"
    got=$(sha256sum "$fname" 2>/dev/null | cut -d' ' -f1)
    if [ "$got" = "$sha" ]; then
      echo "OK $fname"
      ok=1
      break
    else
      echo "MISMATCH attempt $attempt got=$got expected=$sha"
      rm -f "$fname"
    fi
  done
  if [ "$ok" -ne 1 ]; then
    echo "FAILED $fname after retries"
  fi
done < /tmp/dl_list.txt
