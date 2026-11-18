"""SR-3 Scout v1 wiring harness (PROVISIONAL): routes as real geometry, wire table and harness mass.

Model: the harness is a graph of BUNDLE SEGMENTS (3D polylines through the airframe, each a swept tube whose diameter
follows from the conductors inside it). Each CABLE (one wire run: from, to, conductors, connectors) is a list of
segments. Cable length = sum of its segment lengths x (1 + SLACK) + connector tails. Mass = sum of conductors x g/m.

Wire data: OD from the 3F QFR-2509 silicone-wire datasheet; g/m is ESTIMATED (copper + silicone jacket) and agrees with
RJXHOBBY reel weights for 12/10/8 AWG (47/69/100 g/m); ampacity column (safe/max) from Dihool. No seller publishes
g/m directly (research pass 2026-10-15):
    https://www.3f-electronics.com/uploads/34760/files/QFR-2509-Silicone-Soft-Wire.pdf?rnd=466
    https://www.dihool.net/detail_2617_340
    https://www.rjxhobby.com/rjxhobby-100meters-6-8-10-12awg-extra-soft-silicone-wire-cable
Coax: RG316 OD 2.5 mm ~16 g/m [S? reseller]; 1.13 mm micro-coax 3 g/m [S Taoglas 300115B000098A].

Current sizing (per lift motor, T-Motor U8 II KV100 + G28x9.2 at 48 V [S63 via analysis/components.py]):
    100 % throttle 1406 W -> 29.3 A at 48 V, 32.7 A at 43 V (motor max rating is a 180 s figure [S store.tmotor 561])
    hover at the Scout mass: see hover_current() [C]  (~ 9-10 A per motor)
  -> 12 AWG pair per lift ESC (safe 28 A / max 33 A [S Dihool]): fine for hover and climb, at its max only in a
     saturated full-throttle burst. 10 AWG would add ~0.15 kg; see the doc.
  pusher: cruise ~16 A, peak ~60 A [C analysis/aero.py via electronics/SYSTEM.md] -> 10 AWG pair.
  battery -> fuse -> PM08D -> bus: 8 AWG (67/80 A); worst case all-motors-max ~190 A for seconds -> see open problems.
"""
import math
import numpy as np
from build123d import *
from core import d, cyl_between
import scout as S
import f_fuse as FF
import f_wing as FW
import f_boom as FB
import f_tail as FT

SLACK = 0.06                                         # service loops, bends [A]
TAIL_MM = 60.0                                        # connector tails + strip per cable end [A]
SLEEVE_FRAC = 0.10                                    # braided sleeve, heat-shrink, ties as a fraction of wire mass [A]

WIRE = {  # AWG: (OD mm, g/m, (safe A, max A))
    8: (6.5, 99.0, (67, 80)), 10: (5.5, 66.0, (42, 50)), 12: (4.5, 44.0, (28, 33)), 14: (3.5, 26.0, (16, 19)),
    16: (3.0, 18.0, (10, 12)), 18: (2.3, 10.0, None), 20: (1.8, 6.7, None), 22: (1.7, 4.9, None), 24: (1.6, 3.9, None),
    26: (1.5, 3.2, None), "RG316": (2.5, 16.0, None), "MC113": (1.13, 3.0, None),
    "16 (ESC lead)": (3.0, 0.0, (10, 12))}     # ALPHA motor leads 16 AWG x 65 mm + U8 II leads 90 mm [S]: mass is in the ESC/motor
CONN_G = {"AS150U": 41.0, "XT60H": 9.0, "XT90H": 14.0, "4mm bullet x3": 4.5, "JST-GH": 0.4, "CLIK-Mate": 0.5, "MMCX": 1.0,
          "RP-SMA": 3.0, "M5 ring lug x2": 4.0, "JST-GH 6 (wing break)": 0.6, "solder": 0.0, "u.FL": 0.2}   # [A] except AS150U [S?]
SUBCOL = {"power": (0.85, 0.12, 0.12), "signal": (0.15, 0.45, 0.95), "rf": (0.65, 0.20, 0.85), "payload": (0.10, 0.70, 0.30)}


def hover_current(mass_kg, V=43.0):
    """[C] per-motor hover current from the U8 II table (thrust g -> elec W) at sea level, 4 motors."""
    import sys
    sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parents[2] / "analysis"))
    import components as C
    rows = C.U8II_28_12S["rows"]
    T = [r[1] for r in rows]; W = [r[2] for r in rows]
    t = mass_kg*1000/4*C.U8II_28_12S["rho"]/1.225
    return float(np.interp(t, T, W))/V, float(np.interp(t, T, W))


# ------------------------------------------------------------------ geometry helpers
def mz(x, y):
    return FW._mid_z(x, abs(y))

def wing_lower(x, y):
    return FW.z_bounds(abs(y), x)[0]

def tail_pt(u, s=1):
    f = FT.SERVO["f"]
    p = FT.pt(u, f*FT.chord(u), 0.0)
    return (p[0], s*p[1], p[2])

def mir(pts, s):
    return [(x, s*y, z) for x, y, z in pts]


def segments():
    """name -> list of 3D knots (aircraft frame). R/L pairs are mirrored."""
    G = {}
    BX = S.BAT_X; TT, TB = S.TRAY_TOP, S.TRAY_BOT; P = S.PDB
    pm = S.items  # noqa
    asx = BX - FF.BAT["l"]/2 - 32.0
    # --- main power, centre line
    G["bat_main"] = [(BX - FF.BAT["l"]/2 - 1, 30, TT + 40), (BX - FF.BAT["l"]/2 - 8, 40, TT + 12), (asx + 26, 40, TT + 9)]
    G["as150_fuse"] = [(asx - 26, 40, TT + 9), (-222, 12, TT + 6), (-214, 0, TT - 1), (-216, 0, P["top"] + 23.5), (-238, 0, P["top"] + 23.5), (-240, 0, P["top"] + 11)]
    G["fuse_pm"] = [(-184, 0, P["top"] + 11), (-180, 0, P["top"] + 13)]
    G["pm_bus"] = [(-84, 0, P["top"] + 13), (-92, 0, P["top"] + 4), (-92, 0, P["bot"] - 5), (-100, 0, P["bot"] - 8)]
    # --- lift power trunks (bus -> side wall -> up at x 152 -> wing cell -> boom)
    for s, k in ((1, "R"), (-1, "L")):
        G[f"bus_side_{k}"] = mir([(-110, 20, P["bot"] - 14), (-70, 50, -140), (-40, 72, -118), (40, 80, -111), (140, 84, -106)], s)
        zw = mz(152, 40); zw2 = mz(152, 574)
        # wing exits at |y| 40: the fuselage top is rounded (n 3), only ~46 mm half-width at z -3, so the cables enter
        # straight under the wing and fan out below z -20 to pass outside the 150-wide battery tray
        G[f"trunk_up_{k}"] = mir([(140, 84, -106), (152, 84, -88), (152, 84, -62), (152, 52, -22), (152, 40, -8), (152, 40, zw)], s)
        G[f"wing_trunk_{k}"] = mir([(152, 40, zw), (152, 360, mz(152, 360)), (152, 574, zw2)], s)
        G[f"trunk_drop_{k}"] = mir([(152, 574, zw2), (152, 575, wing_lower(152, 574) - 3), (146, 576, -25), (145, 600, -25)], s)
        mx0, mx1 = d.FRONT_MOTOR_X, d.REAR_MOTOR_X
        hx = FB.TUBE_HOLES["esc_x"]; ex = FB.ESC["x0"] + FB.ESC["l"]
        G[f"boom_fwd_{k}"] = mir([(145, 600, -25), (mx0 + hx, 600, -25)], s)
        G[f"boom_aft_{k}"] = mir([(145, 600, -25), (mx1 + hx, 600, -25)], s)
        G[f"boom_tail_{k}"] = mir([(mx1 + hx, 600, -25), (tail_pt(0)[0], 600, -25)], s)
        for i, mx in ((0, mx0), (1, mx1)):
            G[f"esc_drop_{k}{i}"] = mir([(mx + hx, 600, -25), (mx + hx, 600, -46), (mx + ex - 4, 600, d.BOOM_Z + FB.ESC["ztop"] - 4)], s)
        for i, mx in ((0, mx0), (1, mx1)):          # motor phase leads: ESC front end up the tube side into the pylon wire hole
            G[f"phase_{k}{i}"] = mir([(mx + FB.ESC["x0"] + 2, 600, d.BOOM_Z + FB.ESC["zc"] + 8), (mx + FB.ESC["x0"] + 2, 621, d.BOOM_Z + FB.ESC["zc"] + 8), (mx + 34, 621, -40), (mx + 31, 621, -10), (mx + 30, 602, -5), (mx + 20, 600, -5)], s)
        ue = FT.SERVO["u"] - FT.SERVO["lu"]/2 + 2
        G[f"tail_cond_{k}"] = [tail_pt(u, s) for u in (0.0, 30.0, 80.0, ue)] + [tail_pt(FT.SERVO["u"] - 6, s)]
        # FC -> trunk junction (ESC + ruddervator signal), FC -> aileron cell, FC -> GNSS (R only)
        fx = S.FC["x"]; fz = S.FC["plate_top"] + 8.5
        G[f"fc_trunk_{k}"] = mir([(fx + 20, 22, fz), (130, 62, -108), (140, 84, -106)], s)
        zc40 = mz(226, 40)
        G[f"fc_ail_{k}"] = mir([(fx + 37, 12, fz), (232, 30, -112), (233, 80, -84), (229, 52, -24), (226, 40, -8), (226, 40, zc40)], s)
        pts = [(226, 40), (226, 360), (226, 600)]
        G[f"wing_ail_in_{k}"] = mir([(x, y, mz(x, y)) for x, y in pts], s)
        c_, _, xle, _ = FW.station(FW.SERVO_Y); xc = 0.5*(xle + FW.SPLIT*c_ + FW.HINGE_X - FW.AIL_GAP/2)
        pts = FW.COND_PATH[3:]
        G[f"wing_ail_out_{k}"] = mir([(x, y, mz(x, y)) for x, y in pts] + [(xc - 4, FW.SERVO_Y - 8, wing_lower(xc, FW.SERVO_Y) + 6)], s)
    fx = S.FC["x"]; fz = S.FC["plate_top"] + 8.5
    zf = FW._mid_z(113, 40)
    G["fc_gnss"] = [(fx - 37, 10, fz), (113, 55, -106), (113, 87, -96), (113, 87, -62), (113, 52, -22), (113, 40, -8), (113, 40, zf)]
    G["wing_gnss"] = [(113, 40, zf), (113, 360, FW._mid_z(113, 360)), (113, 400, FW._mid_z(113, 400)), (112, 400, S.gnss_z())]
    # --- pusher
    G["bus_push_a"] = [(-120, 0, P["bot"] - 14), (-90, 0, -176), (40, 0, -190), (300, 0, -190)]
    pe = next(i for i in S.items() if i["key"] == "esc_push"); px, _, pz = pe["c"]; pl = pe["dims"][0]
    G["bus_push_b"] = [(300, 0, -190), (480, 0, -168), (560, 0, -146), (px - pl/2 - 6, 0, pz - 2)]
    G["fc_push"] = [(fx, 0, S.FC["plate_top"] - 14), (180, 0, -160), (300, 0, -190)]
    G["push_motor"] = [(px + pl/2 + 2, 0, pz), (760, 0, -100), (812, 0, d.PUSHER_Z)]
    # --- avionics bay
    A = (-300, -80, -64)
    G["fc_fwd"] = [(fx - 40, -18, fz), (60, -55, -108), (-80, -50, -108), (-235, -45, -108), (-258, -45, -108), (-275, -80, -100), (-282, -82, -70), A]
    G["pm_fc"] = [(-132, 24, P["top"] + 27), (-60, 30, -104), (60, 26, -110), (fx - 37, 16, fz)]
    G["pm_5v"] = [(-132, -24, P["top"] + 27), (-170, -40, -104), (-235, -45, -108)]
    bec = next(i for i in S.items() if i["key"] == "bec")["c"]
    G["bec_servo"] = [(bec[0] + 20, bec[1], bec[2]), (-170, -40, -140), (-70, -50, -140), (-40, -55, -118), (60, -30, -116), (fx - 37, -16, fz)]
    for key, dst in (("telem", [(-300, -60, -48), (-372, 34, -48), (-379, 35, -60)]), ("pi", [(-290, -60, -50), (-290, 30, -50), (-291, 30, -60)]),
                     ("rid", [(-330, -80, -64), (-360, -60, -63)]), ("elrs", [(-318, -70, -66), (-324, -40, -67)]),
                     ("adsb", [(-350, -80, -64), (-420, -78, -64), (-428, -54, -64)])):
        G[f"trA_{key}"] = [A] + dst
    G["adsb_ant"] = [(-444, -40, -64), (-475, -45, -66), (-490, -40, -80), (-495, -40, -82)]
    G["telem_coax"] = [(-381, 46, -60), (-360, 72, -50), (-250, 72, -36), (82, 72, -36), (95, 60, -80), (200, 60, -80), (245, 60, -45), (300, 62, -30), (480, 55, -20), (540, 0, FF.top_z(540) - 6)]
    G["elrs_ant"] = [(-325, -40, -66), (-300, -40, -30), (-300, -40, -10)]
    # --- payload
    ax, _, az = S.ALT_C; ah = S.ALT["h"]; aft = ax + S.ALT["l"]/2
    G["alt_comm"] = [(aft + 1, 12, az + 15), (aft + 9, 14, S.DECK["z_bot"] - 6), (aft + 9, 14, S.DECK["z_bot"] + 14), (-300, -60, -82), (-300, -80, -80), A]
    G["bec_alt"] = [(bec[0] - 20, bec[1], bec[2]), (-250, -40, -140), (-290, -12, -150), (aft + 1, -12, az + 5)]
    dls = next(i for i in S.items() if i["key"] == "dls")["c"]
    G["dls_cable"] = [(dls[0], 0, dls[2] - 6), (dls[0], 0, FF.top_z(dls[0]) - 4), (dls[0], 40, -20), (-285, 80, -60), (-285, 80, -90),
                      (aft + 9, 24, S.DECK["z_bot"] + 10), (aft + 9, 22, S.DECK["z_bot"] - 6), (aft + 1, 20, az + 22)]
    tip = S.PROBE["x_tip"]
    G["probe_i2c"] = [(-358, 30, -63), (-365, 5, -62), (-470, 5, -62), (-500, 5, -75), (-530, 0, -84), (-552, 0, S.PROBE["z"]), (tip + 12, 0, S.PROBE["z"])]
    return G


def cables():
    """List of cables: dict(name, frm, to, cond [(gauge, n)], sub, conn (from, to), segs, note)."""
    C = []
    def add(name, frm, to, cond, sub, conn, segs, note=""):
        C.append(dict(name=name, frm=frm, to=to, cond=cond, sub=sub, conn=conn, segs=segs, note=note))
    add("Battery main (+/-)", "battery 12S5P", "AS150U", [(8, 2)], "power", ("pack lead", "AS150U"), ["bat_main"], "8 AWG: 67 A safe / 80 A max; pack lead is part of the pack")
    add("AS150U -> main fuse", "AS150U", "MEGA 150 A fuse", [(8, 2)], "power", ("AS150U", "M5 ring lug x2"), ["as150_fuse"], "through the x -210 tray hole")
    add("Fuse -> PM08D", "fuse", "PM08D in", [(8, 2)], "power", ("M5 ring lug x2", "solder"), ["fuse_pm"])
    add("PM08D -> bus bar", "PM08D out", "bus bar", [(8, 2)], "power", ("solder", "M5 ring lug x2"), ["pm_bus"])
    for k in ("R", "L"):
        for i, pos in ((0, "front"), (1, "rear")):
            segs = [f"bus_side_{k}", f"trunk_up_{k}", f"wing_trunk_{k}", f"trunk_drop_{k}", f"boom_{'fwd' if i == 0 else 'aft'}_{k}", f"esc_drop_{k}{i}"]
            add(f"Lift ESC {pos} {k} power", "bus bar", f"ALPHA 60A {pos} {k}", [(12, 2)], "power", ("M5 ring lug x2", "XT60H"), segs,
                "12 AWG: 28 A safe / 33 A max; peak motor 29-33 A; + 2x 220 uF at the ESC")
            add(f"Lift ESC {pos} {k} signal", "FC", f"ALPHA 60A {pos} {k}", [(26, 3)], "signal", ("JST-GH", "solder"),
                [f"fc_trunk_{k}", f"trunk_up_{k}", f"wing_trunk_{k}", f"trunk_drop_{k}", f"boom_{'fwd' if i == 0 else 'aft'}_{k}", f"esc_drop_{k}{i}"],
                "DShot/PWM + GND + telemetry, twisted")
        add(f"Ruddervator servo {k}", "FC servo rail", f"KST X10 ruddervator {k}", [(22, 3)], "signal", ("JST-GH", "servo plug"),
            [f"fc_trunk_{k}", f"trunk_up_{k}", f"wing_trunk_{k}", f"trunk_drop_{k}", f"boom_aft_{k}", f"boom_tail_{k}", f"tail_cond_{k}"],
            "22 AWG for the 2.5 m run (0.13 V drop per A)")
        add(f"Aileron servo {k}", "FC servo rail", f"KST X10 aileron {k}", [(22, 3)], "signal", ("JST-GH", "servo plug"),
            [f"fc_ail_{k}", f"wing_ail_in_{k}", f"wing_ail_out_{k}"], "breaks at the y=600 JST-GH 6 (2 pins each for + and -)")
    for k in ("R", "L"):
        for i, pos in ((0, "front"), (1, "rear")):
            add(f"Lift motor {pos} {k} phases", f"ALPHA 60A {pos} {k}", f"U8 II {pos} {k}", [("16 (ESC lead)", 3)], "power", ("4mm bullet x3", "solder"),
                [f"phase_{k}{i}"], "ESC 65 mm + motor 90 mm leads [S], through the pylon wire hole; mass in the ESC/motor")
    add("Pusher ESC power", "bus bar", "FLAME 80A", [(10, 2)], "power", ("M5 ring lug x2", "XT90H"), ["bus_push_a", "bus_push_b"], "10 AWG: 42 A safe / 50 A max; pusher peak ~60 A is short")
    add("Pusher ESC signal", "FC", "FLAME 80A", [(26, 3)], "signal", ("JST-GH", "solder"), ["fc_push", "bus_push_b"])
    add("Pusher motor phases", "FLAME 80A", "pusher motor", [(14, 3)], "power", ("solder", "4mm bullet x3"), ["push_motor"], "through the firewall centre hole")
    add("PM08D 5.2 V #1 -> FC", "PM08D", "FC POWER1", [(26, 6)], "power", ("CLIK-Mate", "CLIK-Mate"), ["pm_fc"], "includes the INA228 I2C")
    add("PM08D 5.2 V #2 -> avionics hub", "PM08D", "5 V hub (tray A)", [(20, 2)], "power", ("CLIK-Mate", "JST-GH"), ["pm_5v", "fc_fwd"], "radios + Pi, ~2.5 A peak")
    add("BEC 8 V -> servo rail", "BEC", "servo rail (FC)", [(18, 2)], "power", ("solder", "JST-GH"), ["bec_servo"])
    add("BEC 12 V -> Altum-PT", "BEC", "Altum-PT power", [(20, 2)], "payload", ("solder", "JST-GH"), ["bec_alt"], "10 W cont, 12 W peak at 12 V = 1 A")
    add("GNSS CAN", "FC CAN1", "ARK GPS", [(26, 4)], "signal", ("JST-GH", "JST-GH"), ["fc_gnss", "wing_gnss"], "DroneCAN, twisted pair + 5 V")
    for key, n, nm in (("telem", 6, "TELEM1 -> RFD900x"), ("rid", 4, "CAN2 -> Remote ID"), ("adsb", 6, "UART -> pingRX Pro"), ("elrs", 4, "RC IN -> RP4TD"), ("pi", 4, "TELEM2 -> Pi Zero 2 W")):
        add(nm, "FC", key, [(26, n)], "signal", ("JST-GH", "JST-GH"), ["fc_fwd", f"trA_{key}"])
    add("Altum-PT COMM (serial MAVLink, trigger, centre-of-frame)", "Altum-PT COMM", "FC TELEM3 + AUX + Pi GPIO", [(26, 6)], "payload", ("JST-GH", "JST-GH"),
        ["alt_comm", "fc_fwd"], "COF pulse also branches to the Pi on tray A")
    add("DLS 2 cable (stock 60 cm)", "DLS 2", "Altum-PT DLS port", [(26, 6)], "payload", ("JST-GH", "JST-GH"), ["dls_cable"], "check the stock 60 cm reaches")
    add("SHT45 I2C", "SHT45 (probe)", "Pi Zero 2 W", [(26, 4)], "payload", ("solder", "JST-SH / header"), ["probe_i2c"], "I2C at 100 kHz over ~0.6 m: add a P82B715 buffer if it is noisy")
    add("Telemetry coax", "RFD900x ANT1", "915 MHz mast", [("RG316", 1)], "rf", ("RP-SMA", "RP-SMA"), ["telem_coax"], "RG316 ~0.8 dB/m at 915 MHz [A]")
    add("ADS-B antenna lead", "pingRX Pro", "1090 MHz dipole", [("MC113", 1)], "rf", ("MMCX", "solder"), ["adsb_ant"])
    add("ELRS antenna leads", "RP4TD", "2x T-antenna", [("MC113", 2)], "rf", ("u.FL", "solder"), ["elrs_ant"], "stock 65 mm leads")
    return C


def seg_len(pts):
    return float(sum(np.linalg.norm(np.subtract(b, a)) for a, b in zip(pts[:-1], pts[1:])))

def cond_od(c):
    return WIRE[c[0]][0]

def analyse():
    G = segments(); Cs = cables()
    seg_cond = {k: [] for k in G}
    rows = []
    tot = 0.0
    for c in Cs:
        for sname in c["segs"]:
            seg_cond[sname] += [g for g, n in c["cond"] for _ in range(n)]
        L = sum(seg_len(G[sn]) for sn in c["segs"])*(1 + SLACK) + 2*TAIL_MM
        gpm = sum(WIRE[g][1]*n for g, n in c["cond"])
        m_w = gpm*L/1000.0
        m_c = sum(CONN_G.get(cn, 0.5) for cn in c["conn"])
        rows.append(dict(c, length_m=L/1000.0, g_per_m=gpm, mass_wire_g=m_w, mass_conn_g=m_c))
        tot += m_w + m_c
    # break connectors (JST-GH 6 pairs) and ESC XT60s are in the cable rows; sleeving:
    wire_only = sum(r["mass_wire_g"] for r in rows)
    sleeve = SLEEVE_FRAC*wire_only
    bundles = {}
    for k, pts in G.items():
        cs = seg_cond[k]
        if not cs:
            continue
        dia = max(2.0, 1.15*math.sqrt(sum(WIRE[g][0]**2 for g in cs)))
        subs = [c["sub"] for c in Cs if k in c["segs"]]
        sub = "power" if "power" in subs else ("payload" if "payload" in subs else ("rf" if subs and all(s_ == "rf" for s_ in subs) else "signal"))
        bundles[k] = dict(pts=pts, dia=dia, n=len(cs), sub=sub, length_m=seg_len(pts)/1000.0)
    return dict(rows=rows, bundles=bundles, wire_g=wire_only, conn_g=sum(r["mass_conn_g"] for r in rows), sleeve_g=sleeve, total_g=tot + sleeve)


def bundle_solid(pts, dia):
    r = dia/2
    parts = []
    for a, b in zip(pts[:-1], pts[1:]):
        if np.linalg.norm(np.subtract(b, a)) > 0.2:
            parts.append(cyl_between(a, b, r))
    for p in pts[1:-1]:
        parts.append(Sphere(r).moved(Location(tuple(map(float, p)))))
    return Compound(children=parts) if len(parts) > 1 else parts[0]


def harness_cg(res):
    """Mass-weighted centroid: each cable's wire mass spread along its segments by length."""
    G = segments(); m = 0.0; mom = np.zeros(3)
    for r in res["rows"]:
        segs = [G[s] for s in r["segs"]]
        Ltot = sum(seg_len(p) for p in segs)
        mass = r["mass_wire_g"]*(1 + SLEEVE_FRAC) + r["mass_conn_g"]
        for p in segs:
            for a, b in zip(p[:-1], p[1:]):
                l = np.linalg.norm(np.subtract(b, a))
                if Ltot > 0:
                    w = mass*l/Ltot; m += w; mom += w*(np.array(a) + np.array(b))/2
    return mom/m if m else np.zeros(3), m


if __name__ == "__main__":
    res = analyse()
    print(f"wire {res['wire_g']:.0f} g, connectors {res['conn_g']:.0f} g, sleeve {res['sleeve_g']:.0f} g, total {res['total_g']:.0f} g")
    for r in res["rows"]:
        print(f"  {r['name']:58s} {r['length_m']:5.2f} m  {r['mass_wire_g']:6.1f} g")
    print(hover_current(15.0))
