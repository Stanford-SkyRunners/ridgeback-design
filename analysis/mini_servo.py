"""Servo torque for the smaller own design (docs/OWN_SMALL_DRONE_2026-10-30.md §4k). Hinge moment H = Ch * q * S_c * c_c with
a plain-flap hinge-moment slope [A], at a dive speed above the 26 m/s max level speed. Surfaces are not in CAD yet, so their sizes
are scaled from the 3.35 m design (cad/design.py: 25 % chord ailerons, 30 % chord ruddervators) [A].
Run: python3 analysis/mini_servo.py -> out/mini_servo.md
"""
import json, math
from pathlib import Path
OUT = Path(__file__).resolve().parent / "out"
g = json.load(open(OUT / "mini_layout.json"))["geometry"]
RHO, V_DIVE = 1.225, 1.3 * 26.0          # [A] sea level; 1.3 x max level speed
CH_PER_RAD = 0.9                          # [A] |dCh/d delta| plain flap, upper end of the usual 0.4-0.9 per rad range; Ch_alpha ignored
MARGIN = 2.0                              # [A] servo stall torque >= 2 x hinge moment (holding near stall overheats the servo)
semi, cr, ct = g["b"] / 2, g["root_chord"], g["tip_chord"]
chord = lambda y: cr + (ct - cr) * y / semi
Y0, Y1 = 0.50, 0.87                       # [A] aileron from outboard of the boom/prop wash to the 10 deg tip break
c_mid = chord((Y0 + Y1) / 2)
TC, APEX, BY = 0.12, 0.36, g["boom_y"]    # arch tail pick (mini_layout)
panel = math.hypot(BY, APEX)
surfaces = [  # name, chord of surface m, span m, max deflection deg
    ("aileron (each side)", 0.25 * c_mid, Y1 - Y0, 20.0),
    ("ruddervator (each side of the arch)", 0.30 * TC, 0.85 * panel, 25.0),   # [A] 85 % of the panel length
]
q = 0.5 * RHO * V_DIVE ** 2
L = [f"# Servo torque, smaller own design [C, `analysis/mini_servo.py`; inputs A]\n",
     f"Dive speed {V_DIVE:.1f} m/s (q {q:.0f} Pa), hinge-moment slope {CH_PER_RAD}/rad, required servo torque = {MARGIN} x hinge moment, 1:1 linkage.\n",
     "| Surface | Chord | Span | Max deflection | Hinge moment | Servo torque needed |", "|---|---|---|---|---|---|"]
need = []
for name, cc, span, dmax in surfaces:
    h = CH_PER_RAD * math.radians(dmax) * q * cc * span * cc
    t = MARGIN * h / 9.81 * 100       # kg cm
    need.append(t)
    L.append(f"| {name} | {cc*1000:.0f} mm | {span*1000:.0f} mm | {dmax:.0f} deg | {h:.3f} N m ({h/9.81*100:.2f} kg cm) | **{t:.1f} kg cm** |")
L.append(f"\nAny stock thin-wing servo (wing about 18 mm thick at the aileron [A]) with at least **{max(need):.1f} kg cm** at 6 V is enough on paper.\n")
L += ["| Servo (10 mm thick) | Torque at 6 V | Margin over need | Price | Where | Tag |", "|---|---|---|---|---|---|"]
for n, tq, pr, wh, tag in [("KST X10 (in the BOM)", 7.5, 44.99, "BOM line; boomerangrcjets.com $45.00 in stock 10-30", "S torque / V price"),
                           ("KST X10 Mini", 5.5, 40.50, "helidirect.com, out of stock 10-30", "S torque / V price"),
                           ("KST DS145MG", 5.2, 34.99, "kstservos.com, ships by 4PX/DHL (China), 10-20 working days, so duty applies", "V")]:
    L.append(f"| {n} | {tq} kg cm | {tq / max(need):.2f} | ${pr:.2f} | {wh} | {tag} |")
(OUT / "mini_servo.md").write_text("\n".join(L) + "\n")
print("\n".join(L))
