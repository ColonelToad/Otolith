#!/usr/bin/env python3
"""Ablation end-card for the demo video: traj plot + RMSE table, 1920x1200."""
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1200
BG = (30, 30, 30)
FG = (230, 230, 230)
DIM = (160, 160, 160)
GREEN = (0, 255, 0)
ORANGE = (255, 136, 0)
RED = (255, 60, 60)
PANEL = (40, 40, 40)

ROOT = Path(__file__).resolve().parents[1]
TRAJ = ROOT / "docs" / "figs" / "traj_ablation.png"
OUT = ROOT / "docs" / "figs" / "ablation_card.png"


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    name = "DejaVuSans-Bold.ttf" if bold else "DejaVuSans.ttf"
    return ImageFont.truetype(f"/usr/share/fonts/truetype/dejavu/{name}", size)


img = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(img)

d.text((80, 56), "Without Otolith — dead reckoning (no leg updates)", font=font(52, True), fill=FG)
d.text((80, 128), "Same 5 s trot log · predict-only · contact updates removed", font=font(30), fill=DIM)

# Traj plot card
traj = Image.open(TRAJ).convert("RGB")
tw = 980
th = int(traj.height * tw / traj.width)
traj = traj.resize((tw, th), Image.LANCZOS)
tx, ty = 80, 210
d.rounded_rectangle([tx - 16, ty - 16, tx + tw + 16, ty + th + 16], radius=12, fill=PANEL)
img.paste(traj, (tx, ty))

# RMSE table card
cx, cy = 1140, 210
cw, ch = 700, th + 32
d.rounded_rectangle([cx, cy, cx + cw, cy + ch], radius=12, fill=PANEL)

d.text((cx + 36, cy + 28), "Position RMSE over 1.00 m travel", font=font(30, True), fill=FG)

rows = [
    ("MEKF (Otolith)", "0.106 m", "20.7% drift", GREEN),
    ("Dead reckoning", "0.528 m", "107.9% drift", RED),
]
ry = cy + 100
for label, rmse, drift, color in rows:
    d.rectangle([cx + 36, ry + 8, cx + 64, ry + 36], fill=color)
    d.text((cx + 84, ry), label, font=font(34, True), fill=FG)
    d.text((cx + 84, ry + 48), f"pos RMSE  {rmse}", font=font(30), fill=DIM)
    d.text((cx + 400, ry + 48), drift, font=font(30), fill=color)
    ry += 140

d.text((cx + 36, cy + ch - 88), "Vel RMSE  0.067 → 0.288 m/s", font=font(28), fill=DIM)
d.text((cx + 36, cy + ch - 48), "IMU double-integrates away in seconds.", font=font(28), fill=DIM)

# Footer
d.text((80, H - 90), "Otolith MEKF · 15-state error-state · fusion @500 Hz", font=font(26), fill=DIM)
d.text((80, H - 52), "GT green · MEKF orange · dead reckoning red — contact updates are the difference",
       font=font(26), fill=DIM)

img.save(OUT)
print(f"wrote {OUT} {img.size}")
