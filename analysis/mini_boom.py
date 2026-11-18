"""Boom tube for the smaller own design (docs/OWN_SMALL_DRONE_2026-10-30.md): strength, stiffness and first bending
frequency of stock carbon tubes. Same hand beam method as structures.py (no FEA). Geometry from out/mini_layout.json
(tail 0.12 x 0.36 row). Legs sit under the motor pylons (cad/fab/f_boom.py), so landing loads enter the boom at the
motors. Run: python3 analysis/mini_boom.py  -> out/mini_boom.md
"""
import json, math
from pathlib import Path
G = 9.81
OUT = Path(__file__).resolve().parent / "out"
lay = json.load(open(OUT / "mini_layout.json"))
row = next(r for r in lay["rows"] if abs(r["tail_chord"] - 0.12) < 1e-6)
M_TOTAL = 9.04                                   # [C] mini_structure
X_SPAR = 0.26 * lay["geometry"]["root_chord"]    # [A] boom clamp at the spar, 26 % root chord, unswept leading edge
A_F = X_SPAR - row["x_front_motor"]              # front arm, m
A_R = row["x_rear_motor"] - X_SPAR               # rear arm
A_T = row["tail_le_x"] + 0.5 * row["tail_chord"] - X_SPAR   # tail half-panel mass centre
L_F, L_R = A_F + 0.05, row["tail_le_x"] + row["tail_chord"] - X_SPAR   # tube length ahead of / behind the clamp
M_POS = 0.135 + 0.063 + 0.05 + 0.692 / 4         # [V motor, V props, A ESC, C mini_structure pylon/leg share] per motor position
M_TAIL = 0.331 / 2                               # [C mini_structure] half the tail per boom
T_MAX = 4.215 * G                                # [V] MN5008 + P18 full throttle at 22.5 V (upper bound)
N_FLIGHT, FOS, SIGMA = 3.8, 1.5, 400e6           # same as mini_spar.py [A]
LAND_V, STROKE = 1.5, 0.03                       # [A] same as structures.py
N_LAND = 1 + LAND_V ** 2 / (2 * G * STROKE)
E_LO, E_HI = 70e9, 110e9                         # [A] woven roll-wrapped tube, low / structures.py value
RPM_HOVER = (55, 90)                             # [A] 1P band in Hz: hover to full throttle (no rpm in the store table)
RHO_CF = 1550

tubes = []                                        # (store, od, id, $ per piece, piece length m)
for t in json.load(open(OUT / "haozhong_tubes_1m.json")):
    tubes.append(("HaoZhong 1 m", t["od"], t["id"], t["price"], 1.0))
# Easy Composites US 2 m woven tubes [V listing 2026-10-30: sizes, 2 m offered; "from" $/m is the 20+ tier]. 1-4 unit price
# taken as 1.25 x the "from" price, the ratio seen on the 30/27 page ($85.35 vs $68.25) [A for the other sizes].
# 25/22 page read 2026-10-30 [V]: $72.15/m at 0-4 units (1.248 x), 168.5 g/m, more than 50 in stock; 2 m assumed same per metre.
for od, idm, pm in [(16, 14, 29.75), (18, 16, 31.50), (20, 18, 35.00), (22, 20, 38.50), (25, 23, 47.30), (25, 22, 57.80),
                    (15, 12, 29.35), (30, 27, 68.25)]:
    tubes.append(("Easy Composites US 2 m", od, idm, 2 * 72.15 if (od, idm) == (25, 22) else round(2 * pm * 1.2505, 2), 2.0))

def sec(od, idm):
    o, i = od / 1000, idm / 1000
    I = math.pi * (o ** 4 - i ** 4) / 64
    return I, I / (o / 2), math.pi * (o ** 2 - i ** 2) / 4

def moments():
    """Limit bending moment at the clamp, worst of front/rear, per case (N m)."""
    c = {}
    c["hover, one motor at full thrust"] = max(T_MAX - M_POS * G, 0) * A_R
    for legs in (2, 4):
        F = N_LAND * M_TOTAL * G / legs
        rear = F * A_R - N_LAND * G * (M_POS * A_R + M_TAIL * A_T)
        front = F * A_F - N_LAND * G * M_POS * A_F
        c[f"hard landing on {legs} legs ({LAND_V} m/s, {STROKE * 1000:.0f} mm stroke, n {N_LAND:.1f})"] = max(rear, front)
    c[f"flight {N_FLIGHT} g, motor + tail inertia"] = N_FLIGHT * G * (M_POS * A_R + M_TAIL * A_T)
    return c

def freq(E, I, m_l):
    """First bending frequency of the rear cantilever (clamp to tail) by Rayleigh, point masses at motor and tail
    plus the tube's own mass in 20 pieces."""
    pts = [(A_R, M_POS), (A_T, M_TAIL)] + [((k + 0.5) * L_R / 20, m_l * L_R / 20) for k in range(20)]
    def defl(x):          # static deflection at x under all weights (cantilever superposition)
        y = 0.0
        for a, m in pts:
            P = m * G
            y += P * x ** 2 * (3 * a - x) / (6 * E * I) if x <= a else P * a ** 2 * (3 * x - a) / (6 * E * I)
        return y
    ys = [(defl(a), m) for a, m in pts]
    return math.sqrt(G * sum(m * y for y, m in ys) / sum(m * y * y for y, m in ys)) / (2 * math.pi), defl(A_R)

cases = moments(); M_ult = FOS * max(cases.values())
rows = []
for store, od, idm, price, plen in tubes:
    if (od - idm) / 2 < 1.0 - 1e-9 or not 14 <= od <= 32: continue
    I, Z, Ar = sec(od, idm); m_l = Ar * RHO_CF
    f_lo, d_lo = freq(E_LO, I, m_l); f_hi, _ = freq(E_HI, I, m_l)
    one_piece = plen >= L_F + L_R
    rows.append(dict(store=store, tube=f"{od:g}/{idm:g}", margin=SIGMA * Z / M_ult, g_per_m=1000 * m_l,
                     boom_g=1000 * m_l * (L_F + L_R), f_lo=f_lo, f_hi=f_hi, hover_defl_mm=1000 * d_lo * (M_TOTAL / 4) / M_POS,
                     one_piece=one_piece, cost_2=2 * price * (1 if one_piece else math.ceil((L_F + L_R) / plen)), price=price))

def ok(r):
    in_band = RPM_HOVER[0] * 0.7 < r["f_lo"] < RPM_HOVER[1] * 1.3 or RPM_HOVER[0] * 0.7 < r["f_hi"] < RPM_HOVER[1] * 1.3
    return r["margin"] >= 1.4 and r["f_lo"] >= 15 and not in_band

L = [f"# Boom tube, smaller own design [C, `analysis/mini_boom.py`]\n",
     f"Geometry [C mini_layout, A clamp]: clamp at the spar x {X_SPAR:.3f} m; front arm {A_F:.3f} m, rear arm {A_R:.3f} m, tail mass at "
     f"{A_T:.3f} m; boom {L_F + L_R:.2f} m long. Mass per motor position {M_POS:.3f} kg, half tail {M_TAIL:.3f} kg.\n",
     "| Load case (limit, at the clamp) | N m |", "|---|---|"]
L += [f"| {k} | {v:.1f} |" for k, v in cases.items()]
L.append(f"| **Design (x {FOS} ultimate)** | **{M_ult:.1f}** |\n")
L.append(f"Pass = margin >= 1.4 at {SIGMA / 1e6:.0f} MPa, first bending >= 15 Hz and outside 0.7-1.3 x the rotor band "
         f"{RPM_HOVER[0]}-{RPM_HOVER[1]} Hz [A], for E {E_LO / 1e9:.0f} and {E_HI / 1e9:.0f} GPa [A]. Wall >= 1 mm.\n")
L.append("| Store | Tube | Margin | g/m | Boom g | 1st bending Hz (E 70 / 110) | Hover tip deflection mm | One piece? | 2 booms $ | Pass |")
L.append("|---|---|---|---|---|---|---|---|---|---|")
rows.sort(key=lambda r: (not ok(r), not r["one_piece"], r["boom_g"]))
for r in rows[:30]:
    L.append(f"| {r['store']} | {r['tube']} | {r['margin']:.2f} | {r['g_per_m']:.0f} | {r['boom_g']:.0f} | {r['f_lo']:.0f} / {r['f_hi']:.0f} | "
             f"{r['hover_defl_mm']:.1f} | {'yes' if r['one_piece'] else 'no, needs a joint'} | {r['cost_2']:.2f} | {'yes' if ok(r) else 'no'} |")
best = next((r for r in rows if ok(r) and r["one_piece"]), None)
best_any = next((r for r in rows if ok(r)), None)
L.append("")
if best:
    L.append(f"**Lightest passing one-piece boom: {best['tube']} ({best['store']}), {best['boom_g']:.0f} g per boom, "
             f"margin {best['margin']:.2f}, 1st bending {best['f_lo']:.0f}-{best['f_hi']:.0f} Hz, about ${best['cost_2']:.0f} for two booms.**")
if best_any and best_any is not best:
    L.append(f"Lightest passing tube of any length: {best_any['tube']} ({best_any['store']}), {best_any['boom_g']:.0f} g, but it needs a joint.")
L.append(f"Mass model (mini_structure) had {2 * 1.908 * 18 / 28 * 193.8:.0f} g for both booms (30/27 shortened).")
(OUT / "mini_boom.md").write_text("\n".join(L) + "\n")
print("\n".join(L))

# ---- jointed option: two HaoZhong 1 m pieces per boom joined by a bonded inner sleeve, and torsion from the tail
X_JOINT = row["x_front_motor"] - 0.05 + (L_F + L_R - 1.0)   # rear piece is a full 1 m from the tail end; joint this far forward
OVL = 0.15                                       # [A] sleeve overlap each side of the joint
def m_at(x):  # limit bending at a station ahead of the clamp, front 2-leg landing case (worst there)
    a = row["x_front_motor"] - x
    return N_LAND * M_TOTAL * G / 2 * (-a) - N_LAND * G * M_POS * (-a) if a < 0 else 0.0
Mj = FOS * m_at(X_JOINT)
by = {(r["store"], r["tube"]): r for r in rows}
main = by[("HaoZhong 1 m", "30/28")]
J = []
J.append("\n## Jointed option (HaoZhong 1 m pieces) [C]\n")
J.append(f"Rear piece 1.0 m (tail end to x {X_JOINT:.3f}), front piece {L_F + L_R - 1.0:.2f} m; joint {X_SPAR - X_JOINT:.3f} m ahead of the clamp. "
         f"Design moment at the joint {Mj:.1f} N m (front 2-leg landing x {FOS}).\n")
J.append("| Sleeve inside 30/28 | Gap to 28 mm bore | Sleeve margin at the joint | Bond line | Peak bearing pressure on the bond (6M/dL²) | Sleeve g | Sleeve $ per 1 m |")
J.append("|---|---|---|---|---|---|---|")
tz = {(t["od"], t["id"]): t["price"] for t in json.load(open(OUT / "haozhong_tubes_1m.json"))}
for od, idm in [(28, 25), (28, 26), (27, 25), (26, 24)]:
    if (od, idm) not in tz: continue
    I, Z, Ar = sec(od, idm)
    p = 6 * Mj / (od / 1000 * OVL ** 2) / 1e6
    J.append(f"| {od}/{idm} | {28 - od:.0f} mm | {SIGMA * Z / Mj:.2f} | {(28 - od) / 2:.1f} mm | {p:.2f} MPa | {1000 * Ar * RHO_CF * 2 * OVL:.0f} | {tz[(od, idm)]:.2f} |")
sl = sec(27, 25); m_sl = 1000 * sl[2] * RHO_CF * 2 * OVL
cost_j = 3 * tz[(30, 28)] + tz[(27, 25)]
J.append(f"\nWith a 27/25 sleeve (0.5 mm epoxy bond line [A]): per boom {main['boom_g'] + m_sl:.0f} g (one-piece 25/22: {by[('Easy Composites US 2 m', '25/22')]['boom_g']:.0f} g); "
         f"two booms ${cost_j:.2f} [V] (3 x 30/28 + 1 x 27/25, both sleeves from one tube) vs $288.60. The sleeve runs through the clamp, so the clamp sits on a double wall.")
J.append("Bond shear strength of a filled epoxy is about 15-25 MPa [A], far above the pressures above. The joint still needs one bench test (bonded sample, loaded to the design moment).")

# torsion: arch tail panel loaded to CL 1.0 at the top speed, normal force at mid-panel, carried by ONE boom (the apex tie is ignored)
V_MAX, CL_T, G_SH = 26.0, 1.0, (5e9, 20e9)       # [A] max speed (PX4 params of the 3.35 m design), shear modulus woven 0/90 vs +/-45
panel = math.hypot(lay["geometry"]["boom_y"], row["apex_h"])
F_t = 0.5 * 1.225 * V_MAX ** 2 * row["tail_chord"] * panel * CL_T
T_lim = F_t * panel / 2
J.append(f"\n## Torsion from the tail [C, A inputs]\nOne arch-tail panel ({row['tail_chord']} x {panel:.2f} m) at CL {CL_T} and {V_MAX} m/s: {F_t:.0f} N, "
         f"torque on the boom {T_lim:.1f} N m (limit), carried by one boom (the apex tie between the two panels is ignored, so this is pessimistic).\n")
J.append("| Tube | Shear stress at x 1.5 | Twist at the tail, G 5 GPa (woven 0/90) | G 20 GPa |")
J.append("|---|---|---|---|")
for od, idm in [(25, 22), (30, 28)]:
    I, Z, Ar = sec(od, idm); Jp = 2 * I
    tau = FOS * T_lim * (od / 2000) / Jp
    tw = [math.degrees(T_lim * A_T / (g * Jp)) for g in G_SH]
    J.append(f"| {od}/{idm} | {tau / 1e6:.1f} MPa | {tw[0]:.1f} deg | {tw[1]:.1f} deg |")
J.append("\nStrength is not a problem (woven tube shear strength is tens of MPa [A]). The twist at full tail load is the thing to check on the "
         "built aircraft: twist the boom by hand with a 1 kg weight on a 0.5 m arm and measure the angle at the tail.")
(OUT / "mini_boom.md").write_text("\n".join(L + J) + "\n")
print("\n".join(J))
