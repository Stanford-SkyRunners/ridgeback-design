"""Filament for the smaller own design: the 3.35 m design's printed mass per group and material
(docs/PRINT_MATERIALS_BOM_2026-10-25.md §2 [C CAD]) scaled with the same factors as analysis/mini_structure.py [A], +30 % for failed
prints, rounded up to whole spools. Run: python3 analysis/mini_filament.py -> out/mini_filament.md
"""
import math
from pathlib import Path
F = dict(wing=0.50 / 0.866, tail=0.50 / 0.866, fuse=(36 / 60) ** (2 / 3), boom=4.215 / 8.716, mounts=0.8)   # as mini_structure.py
G0 = {  # group: {material: g} for the 3.35 m design
    "wing": {"LW-ASA": 1329, "PETG-CF": 267}, "fuse": {"LW-ASA": 658, "PETG-CF": 261, "ASA": 13},
    "boom": {"LW-ASA": 202, "PETG-CF": 989, "TPU": 141}, "tail": {"LW-ASA": 287, "PETG-CF": 286},
    "mounts": {"PETG-CF": 179, "ASA": 70, "PETG": 45}}
SPOOL = {  # material: (spool g, $, tag)
    "LW-ASA": (650, 37.50, "V colorfabb.us 10-24"), "PETG-CF": (1000, 35.99, "V hartsmartproducts.com list price; $31.99 on sale 10-27"),
    "TPU": (1000, 33.00, "UNV"), "ASA": (1000, 29.99, "UNV"), "PETG": (1000, 20.00, "UNV, any brand")}
def spools(m, w):  # whole spools for +30 %, plus one more if less than 10 % of a spool would be left [A]
    s = SPOOL[m][0]; n = math.ceil(1.3 * w / s)
    return n + 1 if n * s - 1.3 * w < 0.1 * s else n
need = {}
for g, mats in G0.items():
    for m, w in mats.items(): need[m] = need.get(m, 0) + w * F[g]
L = ["# Filament, smaller own design [C, `analysis/mini_filament.py`]\n",
     "| Material | Part mass g | +30 % g | Spools | Unit $ | Line $ | Tag |", "|---|---|---|---|---|---|---|"]
tot = 0
for m, w in need.items():
    s, p, t = SPOOL[m]; n = spools(m, w); tot += n * p
    L.append(f"| {m} | {w:,.0f} | {1.3 * w:,.0f} | {n} | {p:.2f} | {n * p:.2f} | {t} |")
L.append(f"| CNC Kitchen M5 inserts, 50 | | | 1 | 11.50 | 11.50 | V 10-24 |")
tot += 11.50
L.append(f"| **Total** | {sum(need.values()):,.0f} | | | | **{tot:.2f}** | C |")
L.append(f"\nWithout TPU, ASA and PETG (if the club has leftovers): ${tot - sum(SPOOL[m][1] * spools(m, need[m]) for m in ('TPU', 'ASA', 'PETG')):.2f}.")
(Path(__file__).resolve().parent / "out" / "mini_filament.md").write_text("\n".join(L) + "\n")
print("\n".join(L))
