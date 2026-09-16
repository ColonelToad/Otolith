#!/usr/bin/env bash
# MP4 screen capture -> small looping GIF for README instant preview.
# Usage: ./scripts/make_gif.sh assets/Go2Demo.mp4 [assets/go2_demo.gif]
# Requires: ffmpeg (pixi run ffmpeg).
set -euo pipefail
IN="${1:?usage: make_gif.sh <in.mp4> [out.gif]}"
OUT="${2:-assets/go2_demo.gif}"
pixi run ffmpeg -y -loglevel error -i "$IN" -vf "scale=640:-1,fps=12" -pix_fmt rgb24 "$OUT"
ls -lh "$OUT"
