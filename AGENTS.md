# AGENTS.md

All agent guidance for this repository lives in [CLAUDE.md](CLAUDE.md) — environment, commands, conventions, and phase map. Read it before making changes.

Quick rules that bite hardest if skipped:

1. Everything runs through pixi in WSL2 Debian (`pixi shell` / `pixi run …`). Never apt-install ROS 2.
2. The repo builds only on the Linux filesystem. Never `/mnt/c`.
3. Hot path (fusion/): typed fixed-size structs, zero allocation, jitter telemetry.
4. ROS 2 stays at the edges; the deterministic path is typed contracts and shared memory.
5. Architecture decisions get an ADR in `docs/decisions/`.
