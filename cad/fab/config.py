"""SR-3 build configuration switch.

    SR3_CONFIG=relay  (default)  airborne VHF relay: the original fab package, outputs in fab/out
    SR3_CONFIG=scout             Scout v1 (prediction drone, PROVISIONAL): no relay payload / turret / Jetson,
                                 MicaSense Altum-PT nadir + DLS 2, integrated electronics mounts and wiring.
                                 Outputs in $SKY_BUILD/sr3_scout_out
    SR3_CONFIG=mini              smaller own design 2026-11-02 (2.24 m span, 6S6P, CORE One bed). Outputs in
                                 $SKY_BUILD/sr3_mini_out

    SR3_CONFIG=scout ridgeback/cad/run.sh ridgeback/cad/fab/build.py
The relay build is untouched when SR3_CONFIG is unset: every Scout change is behind `config.SCOUT`.
"""
import os
from pathlib import Path

CONFIG = os.environ.get("SR3_CONFIG", "relay").strip().lower()
if CONFIG not in ("relay", "scout", "mini"):
    raise ValueError(f"SR3_CONFIG must be 'relay', 'scout' or 'mini', got {CONFIG!r}")
SCOUT = CONFIG == "scout"
MINI = CONFIG == "mini"          # smaller own design (2.24 m, 8.9 kg, 6S6P), docs/OWN_SMALL_DRONE_2026-10-30.md; design.py overrides itself
HERE = Path(__file__).resolve().parent
OUT = (Path(os.environ.get("SKY_BUILD", "build")) / "sr3_scout_out" if SCOUT else
       Path(os.environ.get("SKY_BUILD", "build")) / "sr3_mini_out" if MINI else HERE / "out")
