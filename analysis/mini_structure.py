"""Part-by-part structure mass for the smaller own design (docs/OWN_SMALL_DRONE_2026-10-30.md), as a check on the
area scaling in mini_sizing.py. Each group of the 3.35 m design's printed parts [C cad/out_snapshots/scout_2026-10-16/BOM.md]
is scaled by what drives it [A]. Run: python3 analysis/mini_structure.py
"""
S0, S1 = 0.866, 0.50               # wing area, m2
B0, B1 = 3.35, 2.24                # span, m
W0, W1 = 17.23, 8.90               # take-off mass, kg
T0, T1 = 8.716, 4.215              # lift motor max thrust, kg (U8 Lite G28 [V 10-24], MN5008 P18 [V 10-30])
CELLS0, CELLS1 = 60, 36
D0, D1 = 28, 18                    # lift prop diameter, in
printed = [  # group, 3.35 m design g [C BOM], scaling rule, factor
    ("wing skins + spar sockets (LW-ASA 1,329 + CF-PETG 267)", 1596, "skin area ~ wing area", S1 / S0),
    ("tail (LW-ASA 287 + CF-PETG 286)", 573, "tail area ~ wing area", S1 / S0),
    ("fuselage (LW-ASA 658, CF-PETG 261, CF plate 148, ASA 13)", 1080, "surface ~ (battery volume)^(2/3)", (CELLS1 / CELLS0) ** (2 / 3)),
    ("boom pylons, saddles, nacelles, legs, feet (1,430)", 1430, "load ~ lift motor thrust", T1 / T0),
    ("electronics mounts, payload mount, misc (291)", 291, "about the same parts, a bit smaller", 0.8),
]
tubes0 = [  # tube, length m, g/m [store pages 10-30 where V, else A]
    ("booms 30/27", 2 * 1.908, 193.8), ("centre spar 24/20", 0.86, 215), ("outer spars 18/14", 2 * 1.02, 160),
    ("tip spars 12/10", 2 * 0.465, 55), ("rear rods 6/4", 2 * 0.865 + 1.116, 16), ("legs 8/6", 8 * 0.287, 22)]
tubes1 = [  # smaller design [A]: booms shrink with prop size, spars with span (same tube sizes, conservative)
    ("booms 30/27 (length ~ prop diameter)", 2 * 1.908 * D1 / D0, 193.8), ("centre spar 24/20", 0.86 * B1 / B0, 215),
    ("outer spars 18/14", 2 * 1.02 * B1 / B0, 160), ("tip spars 12/10", 2 * 0.465 * B1 / B0, 55),
    ("rear rods 6/4", (2 * 0.865 + 1.116) * D1 / D0, 16), ("legs 8/6", 8 * 0.287 * 0.8, 22)]
print("| Printed group | 3.35 m design g | Rule | Smaller design g |\n|---|---|---|---|")
p1 = 0
for name, g, rule, f in printed:
    p1 += g * f; print(f"| {name} | {g:,} | {rule} | {g * f:,.0f} |")
t0 = sum(l * w for _, l, w in tubes0); t1 = sum(l * w for _, l, w in tubes1)
print(f"| **Printed total** | {sum(g for _, g, _, _ in printed):,} | | **{p1:,.0f}** |")
print(f"| Carbon tubes (same sizes, shorter) | {t0:,.0f} | lengths ~ prop size or span | {t1:,.0f} |")
print(f"| **Structure total** | {sum(g for _, g, _, _ in printed) + t0:,.0f} | | **{p1 + t1:,.0f}** |")
print(f"\nArea scaling in mini_sizing.py gave 3,720 g. Part by part: {p1 + t1:,.0f} g ({(p1 + t1) / 3720 - 1:+.0%}).")
print(f"Bending load on the spars falls to {W1 * B1 / (W0 * B0):.0%} of the 3.35 m design (mass x span), so smaller tubes are possible.")
