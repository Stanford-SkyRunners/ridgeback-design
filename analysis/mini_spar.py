"""Wing spar for the smaller own design (docs/OWN_SMALL_DRONE_2026-10-30.md): lightest set of STOCK 1 m HaoZhong tubes
(analysis/out/haozhong_tubes_1m.json, store json read 2026-10-30 [V]) that keeps margin >= 1.4 everywhere, same method and
assumptions as spar_check_fab.py (3.8 g limit, FOS 1.5, 400 MPa allowable [A], elliptic lift, inertia relief, hover case =
both lift motors on one boom at full throttle with the aircraft held down). Run: python3 analysis/mini_spar.py
"""
import json, math
from itertools import product
from pathlib import Path
import numpy as np
G = 9.81
OUT = Path(__file__).resolve().parent / "out"
M_TOTAL, SEMI = 9.04, 2.24 / 2          # [C] mini_sizing + mini_structure
BOOM_Y = 0.39                           # [A] 600 mm x 18/28 prop ratio, rounded
M_BOOM_SIDE = 0.27 + 0.126 + 0.10 + 0.346 + 0.24   # [A] 2 motors, 2 props (upper), 2 ESCs, printed boom parts, 30/27 boom 1.23 m
M_WING_HALF = 0.921 / 2 + 0.10          # [C mini_structure] printed wing half + spar
T_MAX = 4.215 * G                       # [V] MN5008 KV340 P18, 100 % at 22.5 V (upper bound)
N_LIMIT, FOS, SIGMA = 3.8, 1.5, 400e6
RHO_CF = 1550                           # kg/m3 [A]
Y_C, Y_O, Y_T = 0.30, 0.87, SEMI        # centre tube to |y| 0.30, outer to 0.87, tip to the end
OVL = 0.12                              # m overlap at each joint
L_CUT = dict(c=2 * Y_C, o=Y_O - (Y_C - OVL), t=SEMI - (Y_O - OVL))
TUBES = json.load(open(OUT / "haozhong_tubes_1m.json"))
ys = np.linspace(0, SEMI, 1121); dy = ys[1] - ys[0]

def Z(od, idm):
    od, i = od / 1000, idm / 1000
    return math.pi * (od ** 4 - i ** 4) / (32 * od)

def m_flight(y0):
    L_half = N_LIMIT * M_TOTAL * G / 2
    w = 4 * L_half / (math.pi * SEMI) * np.sqrt(np.clip(1 - (ys / SEMI) ** 2, 0, None))
    o = ys >= y0; M = np.sum(w[o] * (ys[o] - y0)) * dy
    M -= N_LIMIT * G * M_WING_HALF / SEMI * (SEMI - y0) ** 2 / 2
    if y0 < BOOM_Y: M -= N_LIMIT * G * M_BOOM_SIDE * (BOOM_Y - y0)
    return M

def m_hover(y0):
    M = (2 * T_MAX - M_BOOM_SIDE * G) * (BOOM_Y - y0) if y0 < BOOM_Y else 0.0
    return M - M_WING_HALF / SEMI * G * (SEMI - y0) ** 2 / 2

STATIONS = [("root, centre tube", 0.0, "c"), ("centre tube ends, outer tube alone", Y_C, "o"), ("boom", BOOM_Y, "o"),
            ("outer tube ends, tip tube alone", Y_O, "t"), ("tip tube mid", (Y_O + SEMI) / 2, "t")]

def margins(c, o, t):
    tb = dict(c=c, o=o, t=t); res = []
    for name, y, k in STATIONS:
        M = max(m_flight(y), m_hover(y)) * FOS
        res.append((name, y, k, SIGMA * Z(tb[k]["od"], tb[k]["id"]) / M if M > 0 else float("inf")))
    return res

def mass(tb, k):
    return RHO_CF * math.pi / 4 * ((tb["od"] / 1000) ** 2 - (tb["id"] / 1000) ** 2) * L_CUT[k]

fits = lambda inner, outer: 0.0 <= outer["id"] - inner["od"] <= 2.0     # a printed shim fills up to 2 mm
best = None
WALL_MIN = 1.0                          # mm [A]: the 3.35 m design's thinnest tube (12x1); thinner walls dent at clamps and in handling
cands = [x for x in TUBES if 6 <= x["od"] <= 30 and (x["od"] - x["id"]) / 2 >= WALL_MIN]
for c, o, t in product(cands, cands, cands):
    if not (fits(o, c) and fits(t, o)): continue
    mm = margins(c, o, t)
    if min(r[3] for r in mm) < 1.4: continue
    m = mass(c, "c") + 2 * mass(o, "o") + 2 * mass(t, "t")
    if best is None or m < best[0]: best = (m, c, o, t, mm)
same = (dict(od=24, id=20), dict(od=18, id=14), dict(od=12, id=10))
m_same = mass(same[0], "c") + 2 * mass(same[1], "o") + 2 * mass(same[2], "t")
lines = [f"Loads: {M_TOTAL} kg, semispan {SEMI:.2f} m, boom at y {BOOM_Y} m, both lift motors {2 * 4.215:.2f} kg at full throttle.",
         f"Same tube sizes as the 3.35 m design (24/20, 18/14, 12/10), shortened: {m_same * 1000:.0f} g, min margin "
         f"{min(r[3] for r in margins(*same)):.2f}.", ""]
if best:
    m, c, o, t, mm = best
    price = c["price"] + 2 * o["price"] + t["price"]          # centre 1 tube, 2 outer tubes, both tips from 1 tube
    lines += [f"**Lightest stock set with margin >= 1.4: centre {c['od']:.0f}/{c['id']:.0f}, outer {o['od']:.0f}/{o['id']:.0f}, tip "
              f"{t['od']:.0f}/{t['id']:.0f} = {m * 1000:.0f} g ({(m - m_same) * 1000:+.0f} g), ${price:.2f} [V] for 4 tubes.**", "",
              "| Station | y m | Tube | Margin | Sized by |", "|---|---|---|---|---|"]
    tb = dict(c=c, o=o, t=t)
    for name, y, k, mg in mm:
        by = "hover, both motors full" if m_hover(y) > m_flight(y) else "flight, 3.8 g"
        lines.append(f"| {name} | {y:.2f} | {tb[k]['od']:.0f}/{tb[k]['id']:.0f} | {mg:.2f} | {by} |")
(OUT / "mini_spar.md").write_text("\n".join(lines) + "\n"); print("\n".join(lines))
