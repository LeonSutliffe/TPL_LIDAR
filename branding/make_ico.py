"""Regenerates tpl_logo.ico from the color PNG export. Re-run this if the
source logo changes -- the .ico isn't hand-maintained separately.

Requires Pillow (`pip install pillow`).
"""
import os

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "export", "tpl_logo_color_2000.png")
OUT = os.path.join(HERE, "tpl_logo.ico")

src = Image.open(SRC).convert("RGBA")
w, h = src.size

# Wordmark is wide (2000x1050) -- pad onto a square transparent canvas
# (side = width) rather than squashing, so a .ico'd Explorer/taskbar icon
# keeps the logo's real proportions instead of stretching it tall.
side = w
canvas = Image.new("RGBA", (side, side), (0, 0, 0, 0))
canvas.paste(src, (0, (side - h) // 2), src)

canvas.save(OUT, format="ICO", sizes=[(s, s) for s in (16, 32, 48, 64, 128, 256)])
print(f"wrote {OUT} from a {canvas.size} canvas")
