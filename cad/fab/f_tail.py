"""SR-3 arch (Lambda) tail, fabrication version.

Each side is a straight NACA0010 panel from the boom end (y +/-600, z -25) to the apex over the centreline
(z 525), 814 mm long, chord 160 -> 147. Built as:
  tail_socket   (CF-PETG) bonded on the boom tube end + 70 mm root stub, socket for the spar
  3 panel segments (LW-ASA, 0.8 mm skin), hinge at 70 % chord, 10 mm FIBREGLASS spar tube at 30 % chord
     (not carbon: the VHF element runs inside the tail; carbon would detune/shield it)
  3 ruddervator pieces per side (LW-ASA, solid-printed), joined on a 3 mm glass rod, tape/CA hinged
  tail_apex     (CF-PETG) joins both spars at the top; the fairing houses the VHF feed
A 5 mm channel at 12 % chord runs the full length of each panel for the VHF element / coax.
"""
import math
import numpy as np
from build123d import *
from core import d, FabPart, af_piece, hollow, resample, loft, bx, cyl_between, isect, biggest, mirror_part

TILT = math.atan2(d.ARCH_APEX_H, d.BOOM_Y)
L = math.hypot(d.ARCH_APEX_H, d.BOOM_Y)
U_HAT = np.array([0.0, -math.cos(TILT), math.sin(TILT)])       # right panel, root -> apex
N_HAT = np.array([0.0, math.sin(TILT), math.cos(TILT)])        # thickness direction
ROOT = np.array([0.0, d.BOOM_Y, d.BOOM_Z])
AF = d.ARCH_AIRFOIL
HINGE = 1 - d.RUDDERVATOR_CHORD_FRAC                           # 0.70
GAP = 3.0
SKIN, FLANGE = 0.8, 4.0
U_STUB, U_APEX = 70.0, 754.0
U_SEGS = [(70.0, 298.0), (298.0, 526.0), (526.0, 754.0)]
RV_U = (72.0, 752.0)
SPAR = dict(od=10.0, wall=1.0, f=0.30, u0=22.0, u1=789.0)
VHF = dict(d=5.0, f=0.12)
SERVO = dict(u=184.0, f=0.45, lu=36.0, lx=26.0, h=11.0)
SLEEVE = dict(x0=1146.0, x1=1368.0, r=19.0, bore_end=1360.0)
import config
SCOUT = config.SCOUT
# Scout: ruddervator servo lead comes out of the boom tube (hole in the tube wall along U_HAT at the servo chord station)
# and runs in a 4.2 mm conduit (aft of the spar web) straight into the servo pocket. PROVISIONAL.
if SCOUT:   # KST X10 V8 is 30 wide x 42 over the ears [S]: the relay pocket (26 x 36) is too small; f 0.47 keeps 3 mm to the spar tunnel
    SERVO = dict(SERVO, f=0.47, lu=43.0, lx=31.0)
PIN_X = 1300.0                                                 # M4 cross-pin through sleeve + boom
APEX_BOX_X = (1000.0, 1500.0)
FAIR = dict(len=175.0, r=20.0, cav0=20.0, cav1=165.0, cav_r=16.0, z=4.0)
MINI = config.MINI
if MINI:    # smaller own design: tail chord 120, apex 360 above the boom, panel 531 long; 6/4 carbon spar (no VHF element)
    U_STUB, U_APEX = 45.0, 495.0
    U_SEGS = [(45.0, 195.0), (195.0, 345.0), (345.0, 495.0)]
    RV_U = (47.0, 493.0)
    SPAR = dict(od=6.0, wall=1.0, f=0.30, u0=14.0, u1=515.0)
    VHF = dict(d=5.0, f=0.12)                                   # kept as the ruddervator-lead channel
    SERVO = dict(u=120.0, f=0.49, lu=43.0, lx=31.0, h=11.0)     # KST X10 (10 mm thick: about 1 mm proud of the 0010 skin)
    SLEEVE = dict(x0=d.TAIL_LE_X - 4.0, x1=d.BOOM_REAR_X + 8.0, r=18.0, bore_end=d.BOOM_REAR_X)
    PIN_X = d.BOOM_REAR_X - 25.0
    APEX_BOX_X = (700.0, 1200.0)
    FAIR = dict(len=130.0, r=15.0, cav0=15.0, cav1=120.0, cav_r=11.0, z=3.0)
RV_COND = dict(f=SERVO["f"], r=2.1, wall=1.0)
SERVO_BOSS = dict(dy=SERVO["lu"]/2 + 3.5, r=3.0, hole=0.8)      # 2x M2 self-tap bosses (MEASURE ear pitch)

def chord(u):
    return d.ARCH_CHORD - (d.ARCH_CHORD*0.08)*u/L

def xle(u):
    return d.TAIL_LE_X + 10.0*u/L

def pt(u, a, b):
    """Aircraft point: chord coordinate a (mm from LE), thickness coordinate b, span station u (right panel)."""
    c = ROOT + U_HAT*u + N_HAT*b
    return np.array([xle(u) + a, c[1], c[2]])

def place(pts, u):
    return Wire.make_polygon([Vector(*map(float, pt(u, a, b))) for a, b in pts], close=True)

def sec(f0, f1):
    return lambda u, off: af_piece(AF, chord(u), f0, f1, off)

def line(f, u0, u1):
    return pt(u0, f*chord(u0), 0.0), pt(u1, f*chord(u1), 0.0)

def panel_loft(u0, u1, f0=0.0, f1=1.0):
    return loft([place(resample(sec(f0, f1)(u, 0.0)), u) for u in (u0, u1)])

def frame_at(u, f):
    o = pt(u, f*chord(u), 0.0)
    return Location(Plane(origin=tuple(o), x_dir=(1, 0, 0), z_dir=tuple(N_HAT)))


# ------------------------------------------------------------------ parts (right side)
def spar_cuts(u0, u1, outer):
    a, b = line(SPAR["f"], max(u0, SPAR["u0"]) - 0.5, min(u1, SPAR["u1"]) + 0.5)
    r = SPAR["od"]/2 + 0.1
    wall = isect(cyl_between(a, b, r + 1.0), outer)
    xm = xle((u0 + u1)/2) + SPAR["f"]*chord((u0 + u1)/2)
    web = isect(Box(1.2, 2000, 2000).moved(Location((xm, 0, 0))), outer)
    va, vb = line(VHF["f"], u0 - 0.5, u1 + 0.5)
    vwall = isect(cyl_between(va, vb, VHF["d"]/2 + 0.9), outer)
    return [w for w in (wall, web, vwall) if w is not None], [cyl_between(a, b, r), cyl_between(va, vb, VHF["d"]/2)]

def panel_fixed(k):
    u0, u1 = U_SEGS[k]
    f1 = HINGE - (GAP/2)/chord((u0 + u1)/2)
    body, _ = hollow((u0, u1), sec(0.0, f1), place, SKIN, FLANGE)
    outer = panel_loft(u0, u1, 0.0, f1)
    adds, cuts = spar_cuts(u0, u1, outer)
    if u0 < SERVO["u"] < u1:
        loc = frame_at(SERVO["u"], SERVO["f"])
        zt = 3.5                                              # pocket ceiling (thickness coord), opens through the -n skin
        hw = Box(SERVO["lx"] + 3.2, SERVO["lu"] + 3.2, 17.1, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(loc*Location((0, 0, zt + 1.6)))
        adds.append(isect(hw, outer))
        cuts.append(Box(SERVO["lx"], SERVO["lu"], 20.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(loc*Location((0, 0, zt))))
        if SCOUT:
            for sy in (-1, 1):
                adds.append(isect(Cylinder(SERVO_BOSS["r"], 14.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(loc*Location((0, sy*SERVO_BOSS["dy"], 2.0))), outer))
                cuts.append(Cylinder(SERVO_BOSS["hole"], 13.0, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(loc*Location((0, sy*SERVO_BOSS["dy"], 0.5))))
    if SCOUT and u0 < SERVO["u"]:
        ua, ub = u0 - 0.5, min(u1, SERVO["u"] - SERVO["lu"]/2 + 1.0)
        a, b = pt(ua, RV_COND["f"]*chord(ua), 0.0), pt(ub, RV_COND["f"]*chord(ub), 0.0)
        adds.append(isect(cyl_between(a, b, RV_COND["r"] + RV_COND["wall"]), outer))
        cuts.append(cyl_between(a, b, RV_COND["r"]))
    body = body.fuse(*[a for a in adds if a is not None]).clean().cut(*cuts).clean()
    return biggest(body)

def ruddervator(k):
    u0, u1 = U_SEGS[k]
    u0, u1 = max(u0, RV_U[0]), min(u1, RV_U[1])
    f0 = HINGE + (GAP/2)/chord((u0 + u1)/2)
    s = panel_loft(u0, u1, f0, 1.0)
    a, b = line(f0 + 0.08, u0 - 1, u1 + 1)
    return biggest(s - cyl_between(a, b, 1.6))

def tail_socket():
    stub = panel_loft(0.0, U_STUB)
    sl = cyl_between((SLEEVE["x0"], d.BOOM_Y, d.BOOM_Z), (SLEEVE["x1"], d.BOOM_Y, d.BOOM_Z), SLEEVE["r"])
    s = stub.fuse(sl).clean()
    a, b = line(SPAR["f"], SPAR["u0"], U_STUB + 1)
    va, vb = line(VHF["f"], 10.0, U_STUB + 1)
    cuts = [cyl_between((SLEEVE["x0"] - 1, d.BOOM_Y, d.BOOM_Z), (SLEEVE["bore_end"], d.BOOM_Y, d.BOOM_Z), d.BOOM_TUBE[0]/2 + 0.15),
            cyl_between(a, b, SPAR["od"]/2 + 0.1), cyl_between(va, vb, VHF["d"]/2),
            cyl_between((PIN_X, d.BOOM_Y - 40, d.BOOM_Z), (PIN_X, d.BOOM_Y + 40, d.BOOM_Z), 2.2)]     # M4 cross-pin
    if SCOUT:                                                                                         # ruddervator lead from the boom bore
        a, b = pt(0.0, RV_COND["f"]*chord(0.0), 0.0), pt(U_STUB + 1, RV_COND["f"]*chord(U_STUB + 1), 0.0)
        cuts.append(cyl_between(a, b, RV_COND["r"]))
    return biggest(s.cut(*cuts).clean())

def tail_apex():
    right = panel_loft(U_APEX, L + 25.0) & bx(*APEX_BOX_X, 0, 300, -300, 900)
    both = right.fuse(right.mirror(Plane.XZ)).clean()
    zc = d.BOOM_Z + d.ARCH_APEX_H
    fair = cyl_between((d.TAIL_LE_X - 5, 0, zc - FAIR["z"]), (d.TAIL_LE_X + FAIR["len"], 0, zc - FAIR["z"]), FAIR["r"])
    s = both.fuse(fair).clean()
    cuts = []
    for m in (False, True):
        a, b = line(SPAR["f"], U_APEX - 1, SPAR["u1"])
        va, vb = line(VHF["f"], U_APEX - 1, L - 6)
        cs = [cyl_between(a, b, SPAR["od"]/2 + 0.1), cyl_between(va, vb, VHF["d"]/2)]
        cuts += [c.mirror(Plane.XZ) for c in cs] if m else cs
    cuts.append(cyl_between((d.TAIL_LE_X + FAIR["cav0"], 0, zc - FAIR["z"]), (d.TAIL_LE_X + FAIR["cav1"], 0, zc - FAIR["z"]), FAIR["cav_r"]))   # VHF feed / matching box cavity
    return biggest(s.cut(*cuts).clean())


# ------------------------------------------------------------------ jobs
JOBS = ["socket", "apex"] + [f"fixed{k}" for k in range(3)] + [f"rv{k}" for k in range(3)]

def build(job):
    if job == "socket":
        p = FabPart("tail_socket_R", "tail", tail_socket(), "CF-PETG", fill=0.4, up=(0, 0, 1),
                    notes="bonds on the boom tube end (M4 cross-pin) + 70 mm root stub with the " + ("6/4 carbon spar socket and the ruddervator-lead channel" if MINI else "glass-spar socket and VHF channel") + ("; Scout: 4.2 mm ruddervator-lead conduit from the boom bore at 47 % chord" if SCOUT else ""),
                    print_notes="sleeve axis horizontal, tree supports; 5 walls, 20 % gyroid")
        return [p, mirror_part(p, "tail_socket_L")]
    if job == "apex":
        return [FabPart("tail_apex", "tail", tail_apex(), "CF-PETG", fill=0.35, up=(0, 0, -1),
                        notes=("joins both 6/4 carbon spars at the top; the fairing cavity is empty (no VHF on this aircraft)" if MINI else "joins both glass spars at the top; fairing cavity for the VHF feed / matching"), print_notes="top down, tree supports")]
    k = int(job[-1])
    u0, u1 = U_SEGS[k]
    if job.startswith("fixed"):
        p = FabPart(f"tail_panel{k+1}_R", "tail", panel_fixed(k), "LW-ASA", up=tuple(U_HAT), notes=f"arch panel u {u0:.0f}..{u1:.0f} mm" + ("; ruddervator servo bay" if k == 0 else ""),
                    print_notes="2 walls (0.8 mm), 0 % infill; stand on the root end")
    else:
        p = FabPart(f"tail_ruddervator{k+1}_R", "tail", ruddervator(k), "LW-ASA", fill=0.45, up=tuple(U_HAT),
                    notes="ruddervator piece; bond the 3 pieces on a 3 mm glass rod; tape/CA hinge", print_notes="2 walls, 8 % gyroid")
    return [p, mirror_part(p, p.name[:-2] + "_L")]
