import json
import os

tools_json = os.path.expanduser("~/esp/esp-idf/tools/tools.json")
with open(tools_json) as f:
    data = json.load(f)

names = [
    "xtensa-esp-elf-gdb",
    "xtensa-esp-elf",
    "riscv32-esp-elf",
    "esp32ulp-elf",
    "openocd-esp32",
    "esp-rom-elfs",
]

for t in data["tools"]:
    if t["name"] in names:
        for v in t["versions"]:
            info = v.get("linux-amd64") or v.get("any")
            if info and "url" in info:
                fname = info["url"].split("/")[-1]
                print(f"{info['url']}|{info['sha256']}|{fname}")
