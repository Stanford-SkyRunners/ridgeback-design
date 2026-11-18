"""Wiring set for the smaller own design (docs/OWN_SMALL_DRONE_2026-10-30.md): runs from the layout, wire gauge check
(voltage drop, heat) and a priced list. Prices read 2026-10-30 [V] on racedayquads.com products.json (catalogue copy in the
session scratchpad wire6s/, node-local) unless tagged. Run: python3 analysis/mini_wiring.py -> out/mini_wiring.md
"""
import json
from pathlib import Path
OUT = Path(__file__).resolve().parent / "out"
lay = json.load(open(OUT / "mini_layout.json"))
row = next(r for r in lay["rows"] if abs(r["tail_chord"] - 0.12) < 1e-6)
BY = lay["geometry"]["boom_y"]
X_PDB = 0.10                                     # [A] power board in the fuselage, near the CG (x 0.094)
SLACK = 0.15                                     # [A] per run
OHM_PER_M = {14: 0.00829, 16: 0.0132, 20: 0.0333, "2x14": 0.00415}   # copper, 20 C [S standard AWG table]
V_PACK = 21.6                                    # [A] 6S Li-ion under load, as mini_sizing
I_HOVER, I_MAX = 13.0, 33.5                      # A per lift motor [C mini_sizing; V store table at 100 %]
runs = [  # name, count, one-way length m, gauge, hover A, max A
    ("lift ESC feeds, power board to each pylon", 4, BY + abs(row["x_front_motor"] - X_PDB) + SLACK, 14, I_HOVER, I_MAX),
    ("pusher ESC feed", 1, 0.25 + SLACK, 14, 11.0, 40.0),          # [A] pusher at the fuselage tail; 238 W loiter / 21.6 V
    ("battery to power board (two 14 AWG per pole)", 1, 0.15 + SLACK, "2x14", 4 * I_HOVER + 3, 4 * I_MAX + 5),
]
L = ["# Wiring, smaller own design [C, `analysis/mini_wiring.py`]\n",
     f"Power board (the owned PM07, 90 A continuous [V Holybro 10-18]) at x {X_PDB} m [A]. ESCs ride in the motor pylons, as on the 3.35 m design.\n",
     "| Run | Count | Length each (one way) | Wire | Drop at hover | Heat at hover | Drop at full throttle | Heat at full throttle |",
     "|---|---|---|---|---|---|---|---|"]
m14 = 0.0
for name, n, l, g, ih, im in runs:
    r = OHM_PER_M[g] * 2 * l
    L.append(f"| {name} | {n} | {l:.2f} m | {g} AWG | {ih * r:.2f} V ({100 * ih * r / V_PACK:.1f} %) | {ih ** 2 * r:.1f} W | "
             f"{im * r:.2f} V ({100 * im * r / V_PACK:.1f} %) | {im ** 2 * r:.0f} W (seconds only) |")
    m14 += n * l * (2 if g == "2x14" else 1) if g in (14, "2x14") else 0
L.append(f"\n14 AWG needed: {m14:.1f} m of red and {m14:.1f} m of black. Two Power Pack kits give 6 m of each.\n")
ITEMS = [  # name, qty, unit $, tag, note
    ("RDQ silicone wire Power Pack kit (14 AWG 3 m red + 3 m black, 16 AWG 4 + 4 m, 20 AWG 10 + 10 m, cutter)", 2, 21.99, "V", "the single 12 and 14 AWG pairs were sold out on 10-30"),
    ("Amass XT90H connector (battery lead + power board input)", 4, 1.49, "V", "replaces the PM07's XT60 input; hover draws about 55 A"),
    ("Gold bullet connectors 3.5 mm, 3-pack (store guide: up to 40 A)", 10, 1.99, "V", "5 motors x 3 phases; 10 packs in case a pack is 3 pieces, not 3 pairs [A]; 4 mm sold out"),
    ("Servo extension cable 60 cm (2 per line: 4 ESC signals, 4 servos)", 16, 1.49, "V", "1.2 m per line; the tail servos are about 1.2 m from the controller, so check in CAD"),
    ("Heat shrink tube set, 530 pieces", 1, 9.99, "V", ""),
    ("Pre-crimped JST-GH cable set, 15 cm (spares for the Sky4 sensors)", 1, 16.99, "V", "insightfpv.com search result, not re-read on the product page"),
    ("Servo BEC, 5-6 V, at least 5 A", 1, 20.00, "A", "not chosen; ALPHA ESCs are listed without a BEC [S?, not checked on the page]; the owned Sky4 UBEC may do (model unknown)"),
    ("Solder, cable ties, braided sleeve", 1, 15.00, "A", ""),
]
L += ["| Item | Qty | Unit $ | Line $ | Tag | Note |", "|---|---|---|---|---|---|"]
tot = 0.0
for name, q, u, t, note in ITEMS:
    tot += q * u; L.append(f"| {name} | {q} | {u:.2f} | {q * u:.2f} | {t} | {note} |")
est = sum(q * u for _, q, u, t, _ in ITEMS if t == "A")
L.append(f"| **Total** | | | **{tot:.2f}** | C | of which estimates [A] {est:.2f}; the doc had $250 [A] |")
(OUT / "mini_wiring.md").write_text("\n".join(L) + "\n")
print("\n".join(L))
