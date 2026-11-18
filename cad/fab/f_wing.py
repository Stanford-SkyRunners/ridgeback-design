"""SR-3 wing, fabrication version.

Printed LW-ASA segments (0.9 mm skin), split spanwise to fit the 256 mm bed and chordwise at 45 % chord:
  centre section y -600..600: segments c0 (-120..120), c1 (120..360), c2 (360..600) + mirrors
  outer panel   y 600..1675: o1..o5 (215 mm each) + mirrors
Front piece (LE..0.45c) holds the CF spar tunnel + shear web; the rear piece (0.45c..TE) holds the 6 mm
rear rod. Each piece is closed at the split, so the bonded joint is a double shear web.

Spar: stepped, straight, coaxial at x = 82 (26-30 % chord, the thickest part of SD7037):
  24x2 centre tube y -430..430 -> 18x1.5 outer tube y 280..1300 (150 mm inside the 24, ID 20, 0.9 mm
  glass-wrap shim; bonded in the outer panel, slip fit in the centre section + M4 pin: the outer panel
  slides off at y = 600 like a glider wing rod) -> 12x1 tip tube y 1150..1615 (bonded inside the 18).
  The 24 tube stops at y = 430 because the wing is too thin outboard (0.14 mm skin left at y = 600).
  18x1.5 margin at y = 430 is ~1.24 (elliptic flight load, scaled from the 24x2 root margin 1.45).
  NOTE design.py's ROOT_CHORD is 313 mm (its comment says 396 - stale). With 313 mm the concept spar at
  x = 105 broke out of the skin at y = 600 (24 mm tube) and near the tip (18 mm tube).

Boom station (y = 600): four CF-PETG hardpoint blocks per side (wing_hp_*) slide into pockets that open on
the y = 600 joint face of the c2 / o1 pieces and are bonded there; 8x M5 heat-set inserts per side. Each
boom saddle bolts to BOTH panels, so the saddle bolts also retain the outer panel.

Changes vs the concept CAD (parts/wing.py), all flagged in the report:
  * OUTER DIHEDRAL 3 -> 0 deg: at y = 600 the wing is ~30 mm thick; no socket for a 24 mm spar fits a 3-deg
    break (2 mm walls, bore breaks out of the skin). Raymer: unswept high wing 0-2 deg. Re-check lateral
    stability (aero.py) before flight.
  * FLAPS DELETED: the inboard flap crossed the boom at y = 600 (a deflected flap hits the boom) and the
    sizing never used flap lift (stall 13.5 m/s EAS at C_Lmax 1.4 clean; VTOL landing). Fixed inboard TE.
  * aileron hinge gap 3.5 mm (top tape/CA hinge, about +/-20 deg).
"""
import math
import numpy as np
from shapely.geometry import LineString
from build123d import *
from core import d, FabPart, af_piece, hollow, cyl_between, bx, isect, biggest, mirror_part, resample, loft
import wing as W                                  # concept wing: Z_LE_ROOT, SPAR_X, spar_z()

SKIN = 0.9
FLANGE = 5.0
SPLIT = 0.45                                      # chord split
CH = d.CENTER_HALF                                # 600
SPAR_X = 82.0
TUBES = [(-430.0, 430.0, d.INNER_SPAR[0]), (430.0, 1300.0, d.OUTER_SPAR[0]), (1300.0, 1615.0, 12.0)]   # tunnel OD by span band
SPAR_PARTS = dict(inner=(-430.0, 430.0), outer=(280.0, 1300.0), tip=(1150.0, 1615.0))   # actual tube extents (overlaps 150)
TIP_SPAR = (12.0, 1.0)
DIHEDRAL_FAB = 0.0                                # see header
OVERLAP = 150.0                                   # outer tube inside the centre tube
TUBE_CLR = 0.1                                    # radial clearance on bonded tubes
TUN_WALL = 1.4

SEGS = {"c0": (-120.0, 120.0), "c1": (120.0, 360.0), "c2": (360.0, 600.0),
        "o1": (600.0, 815.0), "o2": (815.0, 1030.0), "o3": (1030.0, 1245.0), "o4": (1245.0, 1460.0), "o5": (1460.0, d.SPAN/2)}

# ailerons (flaps deleted)
AIL = d.AILERON_SPAN                              # 950..1580
AIL_GAP = 3.5
_ym = sum(AIL)/2
HINGE_X = d.le_x(_ym) + (1 - d.CONTROL_CHORD_FRAC)*d.chord_at(_ym)     # straight hinge line, constant x
AIL_PIECES = [(AIL[0] + 1, 1160.0), (1160.0, 1370.0), (1370.0, AIL[1] - 1)]
SERVO_Y = 1350.0                                  # aileron servo bay (in segment o4)
SERVO = dict(x=26.0, y=36.0, h=12.0)              # 10 mm class wing servo envelope (e.g. KST X10 / MKS HV6130) (MEASURE)

# boom-station fittings
BOOM_Y = d.BOOM_Y
SADDLE_X = (82.0, 190.0)                          # front saddle under the spar, rear under the hardpoint
BOLT_DX, BOLT_DY = 22.0, 24.0                     # saddle M5 pattern about (SADDLE_X[i], BOOM_Y)
HP_FWD = dict(x=(50.0, 114.0), dy=40.0, ztop_above_spar=2.0)     # integral blocks, y 560..600 (c2) and 600..640 (o1)
HP_AFT = dict(x=(163.0, 217.0), dy=40.0, h=14.0)
INSERT_M5 = dict(d=6.4, depth=10.0)
DENSE_SEGS = ()                                   # all LW-ASA: hardpoints are separate CF-PETG blocks, bolt tubes get Al sleeves
BOLT_SLEEVE = (8.0, 0.75)                         # Al tube sleeve in each M6 wing-bolt tube (bought)

# wing-to-fuselage: 4x M6 nylon (breakaway) through tubes in c0
WING_BOLTS = [(40.0, 50.0), (40.0, -50.0), (170.0, 50.0), (170.0, -50.0)]
ROD_C_X = 200.0                                   # centre rear rod (straight, y -558..558)
ROD_O = ((200.0, 560.0), (107.5, 1420.0))        # outer rear rod (x, y) ends: through the c2 hardpoint, ahead of the servo bay + hinge
ROD_C_HALF = 558.0                                # centre rear rod half length


# ------------------------------------------------------------------ Scout wiring (config.SCOUT only; PROVISIONAL)
import config
SCOUT = config.SCOUT
# The rear piece of every segment already has closed spanwise cells: split wall | rear-rod web | TE / hinge wall.
# Scout uses them as the harness conduits and only cuts clean bores through what blocks them (the 45-deg bond
# flanges at each joint, pocket walls) plus exits through the lower skin. No tube walls are added: a tube wall
# that runs within ~0.2 mm of the inner skin gives near-tangent faces and invalid OCC solids (tried first).
#  (a) power trunk, cell ahead of the rear rod: 4x 12 AWG (one pair per lift ESC) + ESC / ruddervator leads.
#      Passes between the boom-saddle hardpoints (HP_FWD x<=114, HP_AFT x>=163) and drops to the boom at y 574.
TRUNK = dict(x=152.0, r=7.0, y_drop=40.0, y_out=574.0)
TRUNK_PATH = [(152.0, 30.0), (152.0, 120.0), (152.0, 360.0), (152.0, 581.0)]
#  (b) aileron-servo lead, cell aft of the rear rod: x 226 inboard (behind the HP_AFT blocks at the y=600 break),
#      then forward between the outer rod and the aileron hinge wall into the servo bay.
COND = dict(r=2.1, y_drop=40.0)
COND_PATH = [(226.0, 30.0), (226.0, 120.0), (226.0, 360.0), (226.0, 600.0), (224.0, 640.0), (181.0, 815.0), (168.0, 948.0),
             (160.0, 1030.0), (147.0, 1245.0), (130.0, 1331.0)]
#  break connector at y = +/-600: JST-GH 6-pin (S, 2x +, 2x -, spare; 1 A/pin [S jst eGH.pdf], doubled for the
#  ~1.5 A servo stall [A]). Receptacle 8.75 x 4.15 x 5.7 mm [S]; pocket 12 deep for the crimp + strain relief.
CONN = dict(x=226.0, w=9.2, h=4.6, depth=12.0, block=2.0)
#  (c) GNSS (DroneCAN) cable: pad hole at (112, 400) into the front-piece cell behind the spar, inboard to y 70, down.
GNSS_PATH = [(113.0, 30.0), (113.0, 120.0), (113.0, 360.0), (113.0, 400.0)]
GNSS_HOLE = dict(x=112.0, y=400.0, r=5.0)
FRONT_DROP = dict(x=113.0, y=40.0, r=4.0)
# Scout servo: KST X10 V8, body 30 x 35.5 x 10, 42 over the ears, ear holes 36 apart [S KST X10 V8 spec PDF]. The relay
# pocket (26 x 36) is too small for it. Pocket 31 x 43 x 12; retained by a printed plate (f_elec servo_retainer)
# with 2x M2 into bosses just outside the pocket.
SERVO_SCOUT = dict(x=31.0, y=43.0, h=12.0)
SERVO_BOSS = dict(dy=SERVO_SCOUT["y"]/2 + 3.5, r=3.0, hole=0.8, h=6.0)
if SCOUT:
    SERVO = SERVO_SCOUT

# ------------------------------------------------------------------ smaller own design (config.MINI, 2026-11-02)
# Prusa CORE One bed 250 x 220 x 270: spanwise pieces <= 250, chord split at 45 % (root 122 + 149 mm).
# Spar 21/19 | 17/15 | 13/11 (mini_spar). The tube ends are moved inboard of mini_spar's stations because the tubes did not fit
# the wing depth there [C, depth of SD7037 at x 68: y 300 22.6 mm < 21 + 2.0; y 870 18.0 mm < 17 + 2.0]:
#   21/19  y -200..200 | 17/15  y 100..720 (100 inside the 21/19, printed shim) | 13/11  y 620..870 (100 inside the 17/15)
#   | 13/11 tip tube y 870..1105 on the 10 deg tip polyhedral. Margins at the new stations [C, scaled from mini_spar]: 17/15 at
#   y 200 about 1.43, 13/11 at y 720 about 7.
MINI = config.MINI
MINI_SPAR_Z_Y = 720.0
if MINI:
    SEGS = {"c0": (-100.0, 100.0), "c1": (100.0, 245.0), "c2": (245.0, 390.0),
            "o1": (390.0, 500.0), "o2": (500.0, 685.0), "o3": (685.0, 870.0), "o4": (870.0, d.SPAN/2)}
    SPAR_X = 68.0
    TUBES = [(-200.0, 200.0, d.INNER_SPAR[0]), (200.0, 720.0, d.OUTER_SPAR[0]), (720.0, 870.0, d.TIP_SPAR_T[0]),
             (870.0, 1105.0, d.TIP_SPAR_T[0])]
    SPAR_PARTS = dict(inner=(-200.0, 200.0), outer=(100.0, 720.0), tip_inner=(620.0, 870.0), tip=(870.0, 1105.0))
    TIP_SPAR = d.TIP_SPAR_T
    OVERLAP = 100.0
    AIL = (503.0, 868.0)                                  # cut-out 501..870: no TE strip left at the o3/o4 joint
    _ym = sum(AIL)/2
    HINGE_X = d.le_x(_ym) + (1 - d.CONTROL_CHORD_FRAC)*d.chord_at(_ym)
    AIL_PIECES = [(AIL[0] + 1, 685.0), (685.0, AIL[1] - 1)]
    SERVO_Y = 800.0
    SERVO = SERVO_SCOUT                                   # KST X10: 30 x 35.5 x 10, 42 over the ears [S KST X10 V8 PDF]
    SADDLE_X = (68.0, 165.0)
    BOLT_DX, BOLT_DY = 18.0, 20.0
    HP_FWD = dict(x=(40.0, 96.0), dy=34.0, ztop_above_spar=2.0)
    HP_AFT = dict(x=(141.0, 189.0), dy=34.0, h=12.0)
    WING_BOLTS = [(35.0, 40.0), (35.0, -40.0), (190.0, 40.0), (190.0, -40.0)]
    ROD_C_X = 160.0
    ROD_C_HALF = 358.0
    ROD_O = ((160.0, 360.0), (97.0, 860.0))               # ahead of the servo pocket at y 800 (pocket front x 111.8)


def _mid_z(x, y):
    a, b = z_bounds(abs(y), x)
    return 0.5*(a + b)

def _chain(pts, y0, y1, r, ext=1.5):
    """Cut cylinders (at section mid-height) along plan points [(x, y)] clipped to the piece span [y0, y1]; extended by
    `ext` only where they leave the piece, so at internal kinks the pieces just meet."""
    out = []
    for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:]):
        lo, hi = max(min(ya, yb), y0), min(max(ya, yb), y1)
        if hi - lo < 0.5:
            continue
        def at(y):
            x = xa + (xb - xa)*(y - ya)/(yb - ya)
            return np.array((x, y, _mid_z(x, y)))
        pa, pb = at(lo), at(hi)
        u = (pb - pa)/np.linalg.norm(pb - pa)
        if lo <= y0 + 1e-6:
            pa = pa - ext*u
        if hi >= y1 - 1e-6:
            pb = pb + ext*u
        out.append(cyl_between(tuple(pa), tuple(pb), r))
    return out

def conduit_margin():
    """Smallest distance from a pass-through bore to the OUTER surface along each path (skin is 0.9: needs >= 1.1)."""
    res = {}
    for name, pts, r in (("trunk", TRUNK_PATH, TRUNK["r"]), ("servo", COND_PATH, COND["r"]), ("gnss", GNSS_PATH, 3.0)):
        m = 1e9
        for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:]):
            for t in np.linspace(0, 1, 11):
                x, y = xa + (xb - xa)*t, ya + (yb - ya)*t
                lo, hi = z_bounds(y, x); z = _mid_z(x, y)
                m = min(m, (z - r) - lo, hi - (z + r))
        res[name] = round(float(m), 2)
    return res

def _x_at(pts, y):
    for (xa, ya), (xb, yb) in zip(pts[:-1], pts[1:]):
        if min(ya, yb) <= y <= max(ya, yb):
            return xa + (xb - xa)*(y - ya)/(yb - ya)
    raise ValueError(y)

def _notch(pts, ye, inward, hw, f0=None, f1=1.0):
    """Pass-through in the 45-deg bond flange at the piece end ye: box (half-width hw) cut only inside the skin
    (interior offset SKIN - 0.1, as the hardpoint pockets), from 1 mm outside the end to FLANGE + 1.5 inside."""
    f0 = SPLIT if f0 is None else f0
    x = _x_at(pts, ye)
    ya, yb = sorted((ye - inward*1.0, ye + inward*(FLANGE + 1.5)))
    return bx(x - hw, x + hw, ya, yb, -80, 150) & interior(ya, yb, f0, f1, SKIN - 0.1)

def notched_sec(seg, f0, f1, hw=COND["r"] + 0.6):
    """Cavity sections of an outer rear piece with the bond flange opened over the conduit at each end station
    (built into the face-built hollow: booleans on these flanges give invalid OCC solids)."""
    from shapely.geometry import box as sbox
    base = sec_fn(f0, f1)
    y0, y1 = SEGS[seg]
    ends = [ye for ye in (y0, y1) if ye != CH and COND_PATH[0][1] < ye < COND_PATH[-1][1]]
    def sec(y, off):
        P = base(y, off)
        if off > SKIN + 1.0 and any(abs(y - ye) < 1e-6 for ye in ends):
            c, inc, xle, zle = station(y)
            t = math.radians(inc)
            x = _x_at(COND_PATH, y); z = _mid_z(x, y)
            u = (x - xle)*math.cos(t) - (z - zle)*math.sin(t)
            R = sbox(u - hw, -50, u + hw, 50).intersection(base(y, SKIN + 0.4))
            from core import largest
            Q = largest(P.union(R).buffer(0.2, join_style=2).buffer(-0.2, join_style=2))
            if Q is not None:
                P = Q
        return P
    return sec

def _bay_wall_hole(hw):
    x, y = COND_PATH[-1]
    zm = _mid_z(x, y - 4)
    yw = SERVO_Y - SERVO["y"]/2
    return bx(x - hw, x + hw, yw - 4.5, yw + 1.0, zm - hw, zm + hw)

def _drop(x, y, r):
    """Vertical exit from a cell down through the lower skin."""
    zc = _mid_z(x, y); zl = z_bounds(abs(y), x)[0]
    return cyl_between((x, y, zl - 3), (x, y, zc + r), r)

def scout_feats(seg, front):
    """(adds, cut_fns) for the Scout harness features of one wing piece (right-side frame; c0 gets both sides).
    Each cut_fn(dr) returns solids for one feature with its radius grown by dr; make_piece applies them one at a time
    and keeps only results that are OCC-valid (a bore that just grazes a flange gives a sliver)."""
    y0, y1 = SEGS[seg]
    adds, fns = [], []
    sides = (1, -1) if seg == "c0" else (1,)
    def mir(solids, s):
        return [c.mirror(Plane.XZ) if s < 0 else c for c in solids]
    for s in sides:
        ya, yb = (0.0, y1) if seg == "c0" else (y0, y1)
        if front:
            fns.append(("gnss_cell", lambda dr, s=s, ya=ya, yb=yb: mir(_chain(GNSS_PATH, ya, yb, 3.0 + dr), s)))
            if s > 0 and y0 < GNSS_HOLE["y"] < y1:
                g = GNSS_HOLE; zu = z_bounds(g["y"], g["x"])[1]
                fns.append(("gnss_hole", lambda dr, g=g, zu=zu: [cyl_between((g["x"], g["y"], _mid_z(g["x"], g["y"])), (g["x"], g["y"], zu + 3), g["r"] + dr)]))
            if seg == "c0":
                fns.append(("front_drop", lambda dr, s=s: mir([_drop(FRONT_DROP["x"], FRONT_DROP["y"], FRONT_DROP["r"] + dr)], s)))
            continue
        fns.append(("trunk_cell", lambda dr, s=s, ya=ya, yb=yb: mir(_chain(TRUNK_PATH, ya, yb, TRUNK["r"] + dr), s)))
        if seg.startswith("c"):
            fns.append(("servo_cell", lambda dr, s=s, ya=ya, yb=yb: mir(_chain(COND_PATH, ya, yb, COND["r"] + dr), s)))
        else:                                           # outer panel (washout-twisted): bores fail in OCC -> flange notches
            # flange notches at each joint are built into the hollow (notched_sec); y = 600: the break connector
            if seg == "o4":
                fns.append(("servo_bay_wall", lambda dr: [_bay_wall_hole(COND["r"] + 0.5 + dr/2)]))
        for yd in (TRUNK["y_drop"], TRUNK["y_out"]):
            if ya < yd < yb:
                fns.append((f"trunk_drop_{yd:.0f}", lambda dr, s=s, yd=yd: mir([_drop(TRUNK["x"], yd, TRUNK["r"] + dr)], s)))
        if ya < COND["y_drop"] < yb:
            fns.append(("servo_drop", lambda dr, s=s: mir([_drop(COND_PATH[0][0], COND["y_drop"], COND["r"] + 0.4 + dr)], s)))
        if seg in ("c2", "o1"):                         # break connector: receptacle in c2, plug in o1, open on y = 600
            zc = _mid_z(CONN["x"], CH)
            ca, cb = (CH - CONN["depth"], CH + 1) if seg == "c2" else (CH - 1, CH + CONN["depth"])
            w, h, t = CONN["w"], CONN["h"], CONN["block"]
            adds.append(bx(CONN["x"] - w/2 - t, CONN["x"] + w/2 + t, max(ca, y0), min(cb, y1), -80, 150))   # full height, clipped by the skin
            fns.append(("conn_pocket", lambda dr, ca=ca, cb=cb, zc=zc, w=w, h=h: [bx(CONN["x"] - w/2 - dr/2, CONN["x"] + w/2 + dr/2, ca, cb, zc - h/2 - dr/4, zc + h/2 + dr/4)]))
    if seg == "o4" and not front:                       # servo retainer bosses
        c, _, xle, _ = station(SERVO_Y)
        xc = 0.5*(xle + SPLIT*c + HINGE_X - AIL_GAP/2)
        zl = z_bounds(SERVO_Y, xc)[0]
        for sy in (-1, 1):
            yb_ = SERVO_Y + sy*SERVO_BOSS["dy"]
            adds.append(cyl_between((xc, yb_, zl - 2), (xc, yb_, zl + SERVO_BOSS["h"]), SERVO_BOSS["r"]))
            fns.append((f"boss_hole_{sy}", lambda dr, yb_=yb_, zl=zl, xc=xc: [cyl_between((xc, yb_, zl - 3), (xc, yb_, zl + SERVO_BOSS["h"] - 1.0), SERVO_BOSS["hole"] + dr/4)]))
    return adds, fns

SCOUT_LOG = {}

def apply_scout_cuts(body, seg, front, fns):
    log = []
    for name, fn in fns:
        done = False
        for dr in (0.0, 0.4, -0.3, 0.8):
            try:
                sol = fn(dr)
                if not sol:
                    done = True; break
                v0 = body.volume
                trial = body.cut(*sol).clean()
                if trial.is_valid and abs(trial.volume) > 0.5*abs(v0):
                    log.append(f"{name}:{'ok' if dr == 0 else f'dr{dr:+.1f}'}:{v0 - trial.volume:.0f}mm3")
                    body = trial; done = True; break
            except Exception:
                pass
        if not done:
            log.append(f"{name}:SKIPPED")
    SCOUT_LOG[(seg, front)] = log
    return body


# ------------------------------------------------------------------ stations
def station(y):
    yy = abs(y)
    out = max(0.0, (yy - CH)/(d.SPAN/2 - CH))
    zd = d.dihedral_z(yy) if MINI else max(0.0, yy - CH)*math.tan(math.radians(DIHEDRAL_FAB))
    return d.chord_at(yy), d.WING_INCIDENCE_DEG - d.WASHOUT_DEG*out, d.le_x(yy), W.Z_LE_ROOT + zd

def _xform(pts, y):
    c, inc, xle, zle = station(y)
    t = math.radians(inc)
    x = pts[:, 0]*math.cos(t) + pts[:, 1]*math.sin(t) + xle
    z = -pts[:, 0]*math.sin(t) + pts[:, 1]*math.cos(t) + zle
    return x, z

def place(pts, y):
    x, z = _xform(pts, y)
    return Wire.make_polygon([Vector(float(a), float(y), float(b)) for a, b in zip(x, z)], close=True)

def sec_fn(f0, f1):
    return lambda y, off: af_piece(d.AIRFOIL, station(y)[0], f0, f1, off)

def poly_xz(y, f0=0.0, f1=1.0, off=0.0):
    """Section in aircraft (x, z) at span y, as a shapely polygon."""
    from shapely.geometry import Polygon
    P = af_piece(d.AIRFOIL, station(y)[0], f0, f1, off)
    x, z = _xform(np.asarray(P.exterior.coords), y)
    return Polygon(np.stack([x, z], 1))

def z_bounds(y, x):
    """(lower, upper) z of the section at span y, aircraft x."""
    P = poly_xz(y)
    seg = LineString([(x, -500), (x, 500)]).intersection(P)
    zs = [c[1] for c in (seg.coords if seg.geom_type == "LineString" else [p for g in seg.geoms for p in g.coords])]
    return min(zs), max(zs)

def interior_z(ys, xs, off=SKIN + 0.4):
    """Flat z range that clears the inner skin over all (y, x) samples -> CNC-friendly prismatic blocks."""
    lo, hi = -1e9, 1e9
    for y in ys:
        for x in xs:
            a, b = z_bounds(y, x)
            lo, hi = max(lo, a + off), min(hi, b - off)
    return lo, hi

ZS = 0.5*(sum(z_bounds(0.0, SPAR_X))/2 + sum(z_bounds(600.0, SPAR_X))/2)      # straight spar axis z
if MINI:
    ZS = 0.5*(sum(z_bounds(0.0, SPAR_X))/2 + sum(z_bounds(MINI_SPAR_Z_Y, SPAR_X))/2) - 0.18   # balances the skin left above/below [C]

def spar_axis_z(y, band_start=0.0):
    """Spar tube axis z at span y. Relay/scout: straight, ZS. Mini: the tip tube (band starting at the tip break) follows
    the 10 deg tip polyhedral."""
    if MINI and band_start >= d.TIP_BREAK_Y - 1e-6:
        return ZS + d.dihedral_z(y if y >= 0 else -y)*(1 if y >= 0 else 1)
    return ZS

def full_outer(y0, y1):
    kk = (0.0, CH, -CH) + ((d.TIP_BREAK_Y, -d.TIP_BREAK_Y) if MINI else ())
    ks = sorted({y0, y1, *[k for k in kk if y0 < k < y1]})
    return loft([place(resample(sec_fn(0, 1)(y, 0.0)), y) for y in ks])


# ------------------------------------------------------------------ boom-station hardpoints
def bolt_points(i):
    cx = SADDLE_X[i]
    return [(x, y) for x in (cx - BOLT_DX, cx + BOLT_DX) for y in (BOOM_Y - BOLT_DY, BOOM_Y + BOLT_DY)]

def interior(y0, y1, f0, f1, off):
    return loft([place(resample(sec_fn(f0, f1)(y, off)), y) for y in (y0, y1)])

def hp_box(seg, front):
    ya, yb = (BOOM_Y - HP_FWD["dy"], BOOM_Y) if seg == "c2" else (BOOM_Y, BOOM_Y + HP_FWD["dy"])
    H = HP_FWD if front else HP_AFT
    zl = min(z_bounds(y, x)[0] for y in (ya, yb) for x in H["x"])
    ztop = ZS + HP_FWD["ztop_above_spar"] if front else zl + HP_AFT["h"]
    pts = [(x, y) for x, y in bolt_points(0 if front else 1) if ya <= y <= yb]
    return bx(*H["x"], ya, yb, zl - 5, ztop), (ya, yb), pts

def hardpoint(seg, front):
    """(pocket cut for the wing piece, skin bolt holes) - the block itself is hardpoint_part()."""
    blk, (ya, yb), pts = hp_box(seg, front)
    f0, f1 = (0.0, SPLIT) if front else (SPLIT, 1.0)
    pocket = blk & interior(ya - (0 if seg == "o1" else 1), yb + (1 if seg == "o1" else 0), f0, f1, SKIN - 0.1)   # 0.1 into the skin: no co-incident surfaces
    holes = [cyl_between((x, y, z_bounds(y, x)[0] - 5), (x, y, z_bounds(y, x)[0] + 3), 2.8) for x, y in pts]     # M5 clearance in the skin
    return pocket, holes

def hardpoint_part(seg, front):
    blk, (ya, yb), pts = hp_box(seg, front)
    f0, f1 = (0.0, SPLIT) if front else (SPLIT, 1.0)
    s = blk & interior(ya, yb, f0, f1, SKIN + 0.15)
    adds, cuts = _tunnels(ya - 1, yb + 1, front)
    holes = [cyl_between((x, y, z_bounds(y, x)[0]), (x, y, z_bounds(y, x)[0] + SKIN + INSERT_M5["depth"]), INSERT_M5["d"]/2) for x, y in pts]
    return biggest(s.cut(*cuts, *holes).clean())

# ------------------------------------------------------------------ pieces
def _tunnels(y0, y1, front):
    """(adds, cuts) for spar/rod tunnels and webs crossing [y0, y1] on the right side (y >= 0 frame)."""
    adds, cuts = [], []
    if front:
        for ya_, yb_, od in TUBES:
            ya, yb = max(y0, ya_), min(y1, yb_)
            if yb <= ya:
                continue
            ya2, yb2 = ya - (1 if ya > ya_ else 0), yb + (1 if yb < yb_ else 0)
            p0 = (SPAR_X, ya2, spar_axis_z(ya2, ya_)); p1 = (SPAR_X, yb2, spar_axis_z(yb2, ya_))
            r = od/2 + TUBE_CLR
            adds.append(cyl_between(p0, p1, r + TUN_WALL)); cuts.append(cyl_between(p0, p1, r))
        adds.append(bx(SPAR_X - 0.6, SPAR_X + 0.6, y0, y1, -80, 120))          # shear web through the tube
    else:
        rod_r = d.REAR_ROD/2 + TUBE_CLR
        RC = ROD_C_HALF
        if y0 < RC and y1 > -RC:                         # centre rod: ONE straight 6 mm rod, y -558..558
            xr = ROD_C_X
            zr = 0.5*(sum(z_bounds(0.0, xr))/2 + sum(z_bounds(RC, xr))/2)
            ya, yb = max(y0, -RC), min(y1, RC)
            p0, p1 = (xr, ya - (0.5 if ya > -RC else 0), zr), (xr, yb + (0.5 if yb < RC else 0), zr)
            adds.append(cyl_between(p0, p1, rod_r + 1.2)); cuts.append(cyl_between(p0, p1, rod_r))
            adds.append(web_along(p0, p1))
        if y1 > ROD_O[0][1] and y0 < ROD_O[1][1]:      # outer rod: slides 40 mm into c2 so the panel keys on removal
            (xa_, ya0), (x1, yb1) = ROD_O
            e0 = np.array([xa_, ya0, sum(z_bounds(ya0, xa_))/2]); e1 = np.array([x1, yb1, sum(z_bounds(yb1, x1))/2])
            def at(y):
                return tuple(e0 + (e1 - e0)*(y - ya0)/(yb1 - ya0))
            ya, yb = max(y0, ya0), min(y1, yb1)
            if yb > ya:
                p0, p1 = at(ya - (3.0 if ya > ya0 else 0)), at(yb + (3.0 if yb < yb1 else 0))
                adds.append(cyl_between(p0, p1, rod_r + 1.2)); cuts.append(cyl_between(p0, p1, rod_r))
                adds.append(web_along(p0, p1))
    return adds, cuts

def web_along(p0, p1, t=1.2):
    """Vertical web plate (full height, clipped later) along the plan-view line p0 -> p1."""
    p0, p1 = np.array(p0, float), np.array(p1, float)
    L = float(np.hypot(*(p1 - p0)[:2])); ang = math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))
    c = (p0 + p1)/2
    return Box(L, t, 200).rotate(Axis.Z, ang).moved(Location((c[0], c[1], 20.0)))

def make_piece(seg, front):
    y0, y1 = SEGS[seg]
    f0, f1 = (0.0, SPLIT) if front else (SPLIT, 1.0)
    kinks = [0.0] if y0 < 0 < y1 else []
    sec_ = sec_fn(f0, f1)
    if SCOUT and not front and seg.startswith("o") and seg != "o5":
        sec_ = notched_sec(seg, f0, f1)
    body, hollow_ok = hollow((y0, y1), sec_, place, SKIN, FLANGE, kinks=kinks)
    outer = loft([place(resample(sec_fn(f0, f1)(y, 0.0)), y) for y in sorted({y0, y1, *kinks})])
    adds, cuts = _tunnels(y0, y1, front)
    # wing-to-fuselage bolt tubes
    if seg == "c0":
        for x, y in WING_BOLTS:
            if (x < SPLIT*d.ROOT_CHORD) == front:
                rs = BOLT_SLEEVE[0]/2 + 0.1
                adds.append(cyl_between((x, y, -20), (x, y, 80), rs + 1.6)); cuts.append(cyl_between((x, y, -20), (x, y, 80), rs))
    # aileron hinge wall, end walls, cut-out; servo bay
    ail_cut = None
    if not front and y1 > AIL[0] and y0 < AIL[1]:
        xa = HINGE_X - AIL_GAP/2
        ya, yb = max(y0, AIL[0] - 2), min(y1, AIL[1] + 2)
        adds.append(bx(xa - 1.2, xa + 0.3, ya, yb, -80, 150))         # overlaps the cut: no coplanar faces
        for ye in (AIL[0] - 2, AIL[1] + 2):
            if y0 < ye < y1:
                adds.append(bx(xa - 1.2, 600, ye - (1.2 if ye < _ym else 0), ye + (0 if ye < _ym else 1.2), -80, 150))
        cuts.append(bx(xa, 700, AIL[0] - 2, AIL[1] + 2, -80, 150))       # aileron cut-out (cut-then-fuse with the walls)
    if not front and y0 < SERVO_Y < y1:
        c, _, xle, _ = station(SERVO_Y)
        xc = 0.5*(xle + SPLIT*c + HINGE_X - AIL_GAP/2)
        zl, zu = z_bounds(SERVO_Y, xc)
        pocket = bx(xc - SERVO["x"]/2, xc + SERVO["x"]/2, SERVO_Y - SERVO["y"]/2, SERVO_Y + SERVO["y"]/2, zl - 5, zl + SERVO["h"])
        adds.append(bx(xc - SERVO["x"]/2 - 1.6, xc + SERVO["x"]/2 + 1.6, SERVO_Y - SERVO["y"]/2 - 1.6, SERVO_Y + SERVO["y"]/2 + 1.6, zl - 5, zl + SERVO["h"] + 1.6))
        cuts.append(pocket)
    # boom-station fittings: pockets + saddle bolt holes through the lower skin
    if seg in ("c2", "o1"):
        pocket, holes = hardpoint(seg, front)
        post_cuts = [pocket] + holes
    else:
        post_cuts = []
    scout_fns = []
    if SCOUT:
        sa, scout_fns = scout_feats(seg, front)
        adds += sa
    feats = [isect(a, outer) for a in adds]
    feats = [f.cut(*cuts) if cuts else f for f in feats if f is not None]     # cut-then-fuse keeps OCC faces valid
    if cuts:
        body = body.cut(*cuts)
    if feats:
        body = body.fuse(*feats).clean()
    if ail_cut is not None:
        body = body.cut(ail_cut).clean()
    if post_cuts:
        body = body.cut(*post_cuts).clean()
    if scout_fns:
        body = apply_scout_cuts(body, seg, front, scout_fns)
    return biggest(body), hollow_ok

def make_aileron(i):
    y0, y1 = AIL_PIECES[i]
    o = full_outer(y0, y1)
    s = o & bx(HINGE_X + AIL_GAP/2, 700, y0, y1, -80, 150)
    # 3 mm CF torque/joiner rod along the aileron at 30 % of its chord
    c0 = station(_ym)[0]; xr = HINGE_X + AIL_GAP/2 + 0.22*c0*d.CONTROL_CHORD_FRAC*4/4
    za = sum(z_bounds(y0, xr))/2; zb = sum(z_bounds(y1, xr))/2
    s = s - cyl_between((xr, y0 - 1, za), (xr, y1 + 1, zb), 1.6)
    if y0 < SERVO_Y < y1:                                  # 30x3 slot for the control horn
        xh = HINGE_X + AIL_GAP/2 + 18
        zl = z_bounds(SERVO_Y, xh)[0]
        s = s - bx(xh - 15.2, xh + 15.2, SERVO_Y - 1.7, SERVO_Y + 1.7, zl - 5, zl + 10.3)
    return biggest(s)


# ------------------------------------------------------------------ jobs
JOBS = [f"{s}_{p}" for s in SEGS for p in ("front", "rear")] + [f"ail{i}" for i in range(len(AIL_PIECES))] + [f"hp_{s}_{p}" for s in ("c2", "o1") for p in ("front", "rear")]

def build(job):
    if job.startswith("hp_"):
        _, seg, piece = job.split("_")
        p = FabPart(f"wing_hp_{seg}_{piece}_R", "wing", hardpoint_part(seg, piece == "front"), "CF-PETG", fill=1.0, up=(0, 0, 1),
                    notes=f"boom-saddle hardpoint block ({'inboard' if seg == 'c2' else 'outboard'}, {'under the spar' if piece == 'front' else 'aft'}): slides into the pocket on the y=600 face of wing_{seg}_{piece}, bond; 2x M5 heat-set inserts from below",
                    print_notes="flat bottom down, 100 % infill")
        return [p, mirror_part(p, p.name[:-2] + "_L")]
    if job.startswith("ail"):
        i = int(job[3:])
        p = FabPart(f"wing_aileron{i+1}_R", "wing", make_aileron(i), "LW-ASA", fill=0.45, up=(0, 1, 0),
                    notes=f"aileron piece {i+1}/3, y {AIL_PIECES[i][0]:.0f}..{AIL_PIECES[i][1]:.0f}; bond pieces on a 3 mm CF rod; top tape + CA hinges",
                    print_notes="2 walls, 8 % gyroid")
        return [p, mirror_part(p, p.name[:-2] + "_L")]
    seg, piece = job.split("_")
    s, ok = make_piece(seg, piece == "front")
    y0, y1 = SEGS[seg]
    note = f"y {y0:.0f}..{y1:.0f}, {'LE..45 % chord' if piece == 'front' else '45 % chord..TE'}" + ("" if ok else " (solid: too thin to hollow)")
    if SCOUT and piece == "rear" and seg in ("o1", "o2", "o3", "o4"):
        note += "; Scout: bond-flange notches over the servo-lead cell (built into the hollow)"
    if SCOUT and SCOUT_LOG.get((seg, piece == "front")):
        note += "; Scout harness cuts: " + ", ".join(SCOUT_LOG[(seg, piece == "front")])
    mat = "ASA" if seg in DENSE_SEGS else "LW-ASA"
    walls = "2 walls (0.9 mm), 0 % infill, no top/bottom layers on the open ends"
    if seg == "c0":
        return [FabPart(f"wing_{seg}_{piece}", "wing", s, mat, up=(0, 1, 0), notes=note + "; centre, 4x M6 nylon wing bolts through bonded 8x0.75 Al sleeves",
                        print_notes=walls)]
    if seg in ("c2", "o1"):
        note += "; pocket on the y=600 face for the wing_hp_* blocks"
        up = (0, -1, 0) if seg == "c2" else (0, 1, 0)
        pn = walls + "; y=600 joint face on the bed"
    else:
        up, pn = (0, 1, 0), walls + "; stand on the inboard end"
    p = FabPart(f"wing_{seg}_{piece}_R", "wing", s, mat, up=up, notes=note, print_notes=pn)
    return [p, mirror_part(p, p.name[:-2] + "_L")]
