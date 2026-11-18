"""SR-3 Scout v1 electronics mounts (config.SCOUT only, PROVISIONAL). Printed parts in the aircraft frame.

Positions and datasheet values come from scout.py (each with its source URL). Fab conventions (core.py / CONTRACT.md):
FDM walls >= 1.6, plates >= 2.4, M3 heat-set holes 4.0, M3 clearance 3.3, M2 self-tap 1.6, M2.5 pilot 2.2.
Anything not from a datasheet is (ASSUMED) or (MEASURE).
"""
import math
import numpy as np
from build123d import *
from core import d, FabPart, bx, cyl_between, isect, biggest, mirror_part, resample, loft, af_piece
import config
if config.SCOUT:                      # relay build imports this module only to read JOBS (empty)
    import scout as S
import f_fuse as FF
import f_wing as FW
import f_boom as FB
import f_tail as FT

M3_INS, M3_CLR, M2_TAP, M25_PILOT, M2_CLR = 2.0, 1.65, 0.8, 1.1, 1.2       # radii
GRID_X = [FF.FRAMES["F1"] + FF.FRAME_T/2 + 0.5 + 30 + 20*k for k in range(9)]   # tray A M3 grid, x -447 .. -287
GRID_Y = (-35.0, -15.0, 15.0, 35.0)


def item(key):
    return next(i for i in S.items() if i["key"] == key)

def slab(cx, cy, z0, lx, ly, t, r=3.0):
    with BuildSketch() as sk:
        RectangleRounded(lx, ly, r)
    return extrude(sk.sketch.faces()[0].moved(Location((cx, cy, z0))), t)

def holes_z(pts, z0, z1, r):
    return [cyl_between((x, y, z0), (x, y, z1), r) for x, y in pts]


# ------------------------------------------------------------------ FC isolator (under the battery tray at the CG)
DAMPER = dict(d=10.0, h=8.0, neck=6.0)          # silicone damper balls, M3 class (ASSUMED; tune for the IMU, MEASURE)
FC_PLATE = dict(lx=82.0, ly=52.0, t=3.0, holes=(66.0, 35.5))   # ARK PAB carrier pattern ASSUMED (n.p.; MEASURE vs the STEP)

def fc_iso_frame():
    TB = S.TRAY_BOT
    z_ring1 = S.FC["plate_top"] - FC_PLATE["t"] - DAMPER["h"]          # lower ring top -138
    z_ring0 = z_ring1 - 3.0
    pts = FF.FC_HOLES
    parts = [cyl_between((x, y, z_ring0), (x, y, TB), 4.5) for x, y in pts]
    xs = sorted({p[0] for p in pts}); ys = sorted({p[1] for p in pts})
    for y in ys:
        parts.append(bx(xs[0], xs[1], y - 4, y + 4, z_ring0, z_ring1))
    for x in xs + [S.FC["x"] - 25.0, S.FC["x"] + 25.0]:
        parts.append(bx(x - 4, x + 4, ys[0], ys[1], z_ring0, z_ring1))
    s = parts[0].fuse(*parts[1:]).clean()
    cuts = holes_z(pts, TB - 7.0, TB + 1, M3_INS)                        # M3 heat-set from the tray side
    cuts += holes_z(damper_pts(), z_ring0 - 1, z_ring1 + 1, DAMPER["neck"]/2)
    return s.cut(*cuts).clean()

def damper_pts():
    return [(S.FC["x"] + dx, dy) for dx in (-25.0, 25.0) for dy in (-20.0, 20.0)]

def fc_iso_plate():
    P = FC_PLATE
    z0 = S.FC["plate_top"] - P["t"]
    s = slab(S.FC["x"], 0, z0, P["lx"], P["ly"], P["t"], 4)
    hx, hy = P["holes"]
    cuts = holes_z(damper_pts(), z0 - 1, z0 + P["t"] + 1, DAMPER["neck"]/2)
    cuts += holes_z([(S.FC["x"] + sx*hx/2, sy*hy/2) for sx in (-1, 1) for sy in (-1, 1)], z0 - 1, z0 + P["t"] + 1, 1.45)  # M2.5 clearance
    cuts += [bx(S.FC["x"] + x - 2, S.FC["x"] + x + 2, -12, 12, z0 - 1, z0 + P["t"] + 1) for x in (-10.0, 10.0)]          # strap slots
    return s.cut(*cuts).clean()


# ------------------------------------------------------------------ Altum-PT deck (replaces avionics tray B)
def altum_deck():
    zb = S.DECK["z_bot"]; t = S.DECK["t"]
    x0, x1 = FF.FRAMES["F1"] + FF.FRAME_T/2 + 0.5, FF.FRAMES["F2"] - FF.FRAME_T/2 - 0.5
    hw = min(FF.half_width(x, zb, FF.SKIN + 0.5) for x in np.linspace(x0, x1, 8)) - 1.0
    w = 2*min(hw, 70.0)
    s = slab((x0 + x1)/2, 0, zb, x1 - x0, w, t, 6)
    ribs = [bx(x0 + 14, x1 - 14, sy*44.0 - 1.2, sy*44.0 + 1.2, zb + t, zb + t + 10.0) for sy in (-1, 1)]
    ribs += [bx(x - 1.2, x + 1.2, -44.0, 44.0, zb + t, zb + t + 8.0) for x in (S.ALT["x"] - 62.0, S.ALT["x"] + 60.0)]
    s = s.fuse(*ribs).clean()
    lug = [(x0 + 8.5, sy*50.0) for sy in (1, -1)] + [(x1 - 8.5, sy*50.0) for sy in (1, -1)]
    ax = S.ALT["x"]
    cam = [(ax - 30.0, 0.0), (ax + 30.0, 0.0), (ax, 17.5), (ax, -17.5)]      # Altum-PT 4x M3 cross pattern [S integration guide]
    cuts = holes_z(lug, zb - 1, zb + t + 1, M3_CLR) + holes_z(cam, zb - 1, zb + t + 1, M3_CLR)
    cuts += [bx(ax + 58.0, ax + 70.0, -26.0, 26.0, zb - 1, zb + t + 11)]      # cable slot behind the camera (connectors: MEASURE)
    cuts += [cyl_between((ax + sx*30.0, sy*30.0, zb - 1), (ax + sx*30.0, sy*30.0, zb + t + 1), 9.0) for sx in (-1, 1) for sy in (-1, 1)]  # lightening + fan air
    return s.cut(*cuts).clean()


# ------------------------------------------------------------------ PDB plate (under the battery tray, ex-radio standoffs)
def pdb_plate():
    P = S.PDB
    xc = (P["x0"] + P["x1"])/2
    s = slab(xc, 0, P["bot"], P["x1"] - P["x0"], 2*P["hw"], P["t"], 5)
    ribs = [bx(P["x0"] + 4, P["x1"] - 4, sy*(P["hw"] - 3.0) - 1.2, sy*(P["hw"] - 3.0) + 1.2, P["bot"] - 8.0, P["bot"]) for sy in (-1, 1)]
    s = s.fuse(*ribs).clean()
    pm, fu, bu = item("pm"), item("fuse"), item("bus")
    cuts = holes_z([(-200.0, 32.0), (-200.0, -32.0), (-100.0, 32.0), (-100.0, -32.0)], P["bot"] - 9, P["top"] + 1, M3_CLR)
    cuts += holes_z([(pm["c"][0] + sx*39.5, sy*19.05) for sx in (-1, 1) for sy in (-1, 1)], P["bot"] - 1, P["top"] + 1, M3_CLR)   # PM08D 79 x 38.1 [S]
    cuts += holes_z([(fu["c"][0] + sx*20.0, 0.0) for sx in (-1, 1)], P["bot"] - 1, P["top"] + 1, 2.65)                         # fuse holder 2x M5 (ASSUMED)
    cuts += holes_z([(bu["c"][0] + sx*30.0, 0.0) for sx in (-1, 1)], P["bot"] - 1, P["top"] + 1, 2.15)                         # bus bar 2x M4
    cuts += [cyl_between((P["x1"] - 12.0, 0, P["bot"] - 1), (P["x1"] - 12.0, 0, P["top"] + 1), 8.0)]                           # PM08D out -> bus
    cuts += [bx(-212.0 - 2, -212.0 + 2, -40.0, -16.0, P["bot"] - 1, P["top"] + 1), bx(-188.0 - 2, -188.0 + 2, -40.0, -16.0, P["bot"] - 1, P["top"] + 1)]  # BEC straps
    return s.cut(*cuts).clean()


# ------------------------------------------------------------------ tray-A adapters (M3 20 mm grid)
ADAPT = {   # key: (centre x, y, plate lx, ly, grid points, extra holes [(dx, dy, r)], lip height)
    "telem": (-410.0, 35.0, 62.0, 34.0, [(-427.0, 35.0), (-387.0, 35.0)], [], 5.0),
    "adsb": (-428.0, -33.0, 36.0, 40.0, [(-427.0, -35.0), (-427.0, -15.0)], [], 4.0),
    "rid": (-380.0, -35.0, 44.0, 34.0, [(-387.0, -35.0), (-367.0, -35.0)], [], 4.0),     # ears M2 31 apart [S]: self-tap into the plate edge (MEASURE)
    "elrs": (-337.0, -35.0, 30.0, 24.0, [(-347.0, -35.0), (-327.0, -35.0)], [], 3.0),
    "pi": (-325.0, 30.0, 70.0, 36.0, [(-347.0, 35.0), (-307.0, 35.0)], [(sx*29.0, sy*11.5, M25_PILOT) for sx in (-1, 1) for sy in (-1, 1)], 0.0),
}

def adapter(key):
    cx, cy, lx, ly, grid, extra, lip = ADAPT[key]
    it = item(key)
    z0 = S.ZA; t = S.MOUNT_T
    s = slab(cx, cy, z0, lx, ly, t, 3)
    dl = it["dims"]
    adds = []
    if lip > 0:                                                          # side lips locate the module (+0.3 clearance)
        for sy in (-1, 1):
            yy = cy + sy*(dl[1]/2 + 0.3)
            adds.append(bx(cx - dl[0]/2 + 4, cx + dl[0]/2 - 4, yy - (0 if sy > 0 else 1.6), yy + (1.6 if sy > 0 else 0), z0 + t, z0 + t + lip))
    if key == "pi":                                                      # 4 M2.5 standoffs (58 x 23 [S])
        adds += [cyl_between((cx + dx, cy + dy, z0 + t), (cx + dx, cy + dy, z0 + t + 4.0), 2.6) for dx, dy, _ in extra]
    if key == "rid":                                                     # ear bosses (2x M2, 31 apart [S])
        adds += [cyl_between((cx + dx, cy + dy, z0 + t), (cx + dx, cy + dy, z0 + t + 2.0), 2.6) for dx, dy, _ in extra]
    if adds:
        s = s.fuse(*adds).clean()
    cuts = holes_z(grid, z0 - 1, z0 + t + 1, M3_CLR)
    cuts += [cyl_between((cx + dx, cy + dy, z0 + t - 2.0), (cx + dx, cy + dy, z0 + t + 5.0), r) for dx, dy, r in extra]
    if key not in ("pi",):                                               # zip-tie slots across the module
        cuts += [bx(cx + x - 1.75, cx + x + 1.75, cy - ly/2 + 2.5, cy + ly/2 - 2.5, z0 - 1, z0 + t + 1) for x in ((-dl[0]/4, dl[0]/4) if dl[0] > 30 else (0.0,))]
    s = s.cut(*cuts).clean()
    # keep clear of the grid holes under a slot
    return biggest(s)


# ------------------------------------------------------------------ DLS 2 pedestal (bonded on the avionics hatch)
def dls_pedestal():
    x = S.DLS["x"]; zt = FF.top_z(x)
    top = zt + S.DLS["ped_h"]
    blk = bx(x - 36, x + 36, -27, 27, zt - 8, top)
    s = blk - FF.shell_loft(x - 40, x + 40, 0.0)
    s = biggest(s)
    pocket = bx(x - 30, x + 30, -21, 21, zt + 1.5, top + 1)           # lightening under the top skin
    s = s.cut(pocket.cut(bx(x - 33, x + 33, -24, 24, top - 2.4, top + 2)))
    cuts = [cyl_between((x + sx*40.75/2, 0, top - 6), (x + sx*40.75/2, 0, top + 1), M2_TAP) for sx in (-1, 1)]   # 2x M2 40.75 [S]
    cuts += [cyl_between((x, 0, zt - 10), (x, 0, top + 1), 4.0)]
    return biggest(s.cut(*cuts).clean())


# ------------------------------------------------------------------ pusher ESC mount (bonded into fuse_seg6)
def pusher_esc_mount():
    E = item("esc_push"); (lx, ly, lz) = E["dims"]; x, _, zc = E["c"]
    zdeck = zc - lz/2
    x0, x1 = x - lx/2 - 4, x + lx/2 + 4
    deck = bx(x0, x1, -ly/2 - 3, ly/2 + 3, zdeck - 2.4, zdeck)
    inner = FF.shell_loft(x0 - 2, x1 + 2, FF.SKIN + 0.2)
    webs = [isect(bx(x0 + 2, x1 - 2, sy*10.0 - 1.0, sy*10.0 + 1.0, -400, zdeck), inner) for sy in (-1, 1)]
    feet = [isect(bx(xx - 4, xx + 4, -ly/2 - 3, ly/2 + 3, -400, zdeck), inner) for xx in (x0 + 6, x1 - 6)]
    s = deck.fuse(*[w for w in webs + feet if w is not None]).clean()
    cuts = [bx(xx - 2, xx + 2, -ly/2 + 2, ly/2 - 2, zdeck - 3, zdeck + 1) for xx in (x - lx/4, x + lx/4)]
    cuts += [bx(x - 12, x + 12, -6.5, 6.5, zdeck - 3, zdeck + 1)]
    return biggest(s.cut(*cuts).clean())


# ------------------------------------------------------------------ SHT45 probe: nose socket + radiation shield
def probe_socket():
    od = S.PROBE["tube"][0]; z = S.PROBE["z"]
    x0 = FF.BOUNDS[0]
    base = cyl_between((x0 - 4.0, 0, z), (x0, 0, z), 18.0)
    boss = cyl_between((x0 - 26.0, 0, z), (x0 - 4.0, 0, z), od/2 + 3.0)
    s = base.fuse(boss).clean()
    s = s.cut(cyl_between((x0 - 27.0, 0, z), (x0 - 2.0, 0, z), od/2 + 0.15), cyl_between((x0 - 6, 0, z), (x0 + 1, 0, z), 3.5))
    return s

SHIELD = dict(r=28.0, n=4, pitch=10.0, t=1.6, core_r=15.0, ring_h=4.5)

def probe_shield():
    """4 conical gill rings (45 deg, printable upright) on 3 posts, solid cap, sensor cage in the middle, side socket
    for the probe tube. Own design (no commercial drone shield found [S research pass]); white ASA."""
    Sh = SHIELD; x = S.PROBE["x_tip"]; z = S.PROBE["z"]
    zt = z + 26.0
    parts = [cyl_between((x, 0, zt - 2.4), (x, 0, zt), Sh["r"])]                         # cap
    for k in range(Sh["n"]):
        ztop = zt - 2.4 - k*Sh["pitch"]
        h = Sh["r"] - Sh["core_r"] + 2
        outer = Solid.make_cone(Sh["r"], Sh["core_r"] + 2, h, Plane((x, 0, ztop - h), z_dir=(0, 0, 1)))
        inner = Solid.make_cone(Sh["r"] - Sh["t"]*1.414, Sh["core_r"] + 2 - Sh["t"]*1.414, h, Plane((x, 0, ztop - h - 0.01), z_dir=(0, 0, 1)))
        ring = (outer - inner) & bx(x - 40, x + 40, -40, 40, ztop - h, ztop - h + Sh["ring_h"])
        parts.append(ring)
    zb = zt - 2.4 - Sh["n"]*Sh["pitch"] - 6.0
    for a in (90.0, 210.0, 330.0):
        px, py = x + (Sh["r"] - 3.0)*math.cos(math.radians(a)), (Sh["r"] - 3.0)*math.sin(math.radians(a))
        parts.append(cyl_between((px, py, zb), (px, py, zt - 1), 2.0))
    od = S.PROBE["tube"][0]
    parts.append(cyl_between((x + Sh["r"] - 4, 0, z), (x + Sh["r"] + 22, 0, z), od/2 + 2.5))        # tube socket, aft side
    parts.append(bx(x + 9.2, x + Sh["r"], -3.0, 3.0, z - 3, z + 3))                                  # arm; the SHT45 board (17.7 wide, vertical) is bonded to its end
    s = parts[0].fuse(*parts[1:]).clean()
    cuts = [cyl_between((x + Sh["r"] - 2, 0, z), (x + Sh["r"] + 23, 0, z), od/2 + 0.15),
            cyl_between((x - 1, 0, z), (x + Sh["r"] + 2, 0, z), 1.6)]                               # cable path to the sensor
    return biggest(s.cut(*cuts).clean())


# ------------------------------------------------------------------ servo retainers (cover the pocket, 2x M2 into the bosses)
RET = dict(lx=37.0, ly=2*FW.SERVO_BOSS["dy"] + 8.0, t=2.4, ears=36.0, slot=(14.0, 24.0, -8.0))

def _retainer_local():
    R = RET
    with BuildSketch() as sk:
        RectangleRounded(R["lx"], R["ly"], 4)
        for sy in (-1, 1):
            with Locations((0, sy*FW.SERVO_BOSS["dy"])):
                Circle(M2_CLR, mode=Mode.SUBTRACT)
            with Locations((0, sy*R["ears"]/2)):
                Circle(1.0, mode=Mode.SUBTRACT)                          # servo ear screws, 36 apart [S KST X10]
        with Locations((0, R["slot"][2])):
            RectangleRounded(R["slot"][0], R["slot"][1], 3, mode=Mode.SUBTRACT)   # servo arm (MEASURE shaft position)
    return extrude(sk.sketch.faces()[0], R["t"])

def servo_retainer_wing():
    c, _, xle, _ = FW.station(FW.SERVO_Y); xc = 0.5*(xle + FW.SPLIT*c + FW.HINGE_X - FW.AIL_GAP/2)
    zl = min(FW.z_bounds(FW.SERVO_Y + dy, xc + dx)[0] for dx in (-18, 0, 18) for dy in (-25, 0, 25))
    return _retainer_local().moved(Location((xc, FW.SERVO_Y, zl - RET["t"] - 0.1)))

def servo_retainer_tail():
    loc = FT.frame_at(FT.SERVO["u"], FT.SERVO["f"])
    half = 0.5*0.10*5*(0.2969*math.sqrt(FT.SERVO["f"]) - 0.126*FT.SERVO["f"] - 0.3516*FT.SERVO["f"]**2 + 0.2843*FT.SERVO["f"]**3 - 0.1015*FT.SERVO["f"]**4)*FT.chord(FT.SERVO["u"])*2
    return _retainer_local().moved(loc*Location((0, 0, -half - RET["t"] - 0.6)))


# ------------------------------------------------------------------ strobes (self-powered ARC V, no wiring)
def wingtip_strobe():
    """Closes the open wing-tip end of o5 (front + rear pieces) and carries an ARC V strobe facing outboard."""
    y = d.SPAN/2
    cap = FW.full_outer(y, y + 1.6)
    sp = []
    for f0, f1 in ((0.0, FW.SPLIT), (FW.SPLIT, 1.0)):
        try:
            sp.append(loft([FW.place(resample(af_piece(d.AIRFOIL, FW.station(yy)[0], f0, f1, FW.SKIN + 5.0 + 0.3)), yy) for yy in (y - 6.0, y)]))
        except Exception:
            pass
    sp = [q - bx(FW.SPAR_X - 1.4, FW.SPAR_X + 1.4, y - 7, y + 0.5, -80, 120) for q in sp]          # clear the front-piece shear web
    it = item("strobe_R"); (lx, ly, lz) = it["dims"]; cx, cy, cz = it["c"]
    pod = bx(cx - lx/2 - 1.6, cx + lx/2 + 1.6, y + 1.0, y + 1.6 + ly + 1.0, cz - lz/2 - 1.6, cz + lz/2 + 1.6)
    s = cap.fuse(*sp, pod).clean()
    s = s.cut(bx(cx - lx/2 - 0.2, cx + lx/2 + 0.2, y + 1.6, y + 1.6 + ly + 2.0, cz - lz/2 - 0.2, cz + lz/2 + 0.2))   # strobe pocket, open outboard
    return biggest(s)

def belly_strobe():
    it = item("strobe_belly"); (lx, ly, lz) = it["dims"]; cx, cy, cz = it["c"]
    zb = FF.params(cx)[2] - FF.params(cx)[1]
    blk = bx(cx - lx/2 - 6, cx + lx/2 + 6, -ly/2 - 5, ly/2 + 5, cz - lz/2 - 1.0, zb + 8.0)
    s = biggest(blk - FF.shell_loft(cx - 40, cx + 40, 0.0))
    s = s.cut(bx(cx - lx/2 - 0.2, cx + lx/2 + 0.2, -ly/2 - 0.2, ly/2 + 0.2, cz - lz/2 - 2, cz + lz/2 + 0.2))       # lens faces down
    return biggest(s)


# ------------------------------------------------------------------ telemetry antenna base (RP-SMA bulkhead on fuse_seg5 top)
def ant_base():
    x = S.MAST["x"]; zt = FF.top_z(x)
    blk = cyl_between((x, 0, zt - 6.0), (x, 0, zt + 10.0), 13.0)
    s = biggest(blk - FF.shell_loft(x - 20, x + 20, 0.0))
    s = s.cut(cyl_between((x, 0, zt - 8), (x, 0, zt + 11), 3.3),                    # RP-SMA bulkhead 6.5
              cyl_between((x, 0, zt + 5.0), (x, 0, zt + 11), 5.5))                  # nut / antenna base recess
    return biggest(s)


# ------------------------------------------------------------------ TPU grommets for the boom-tube holes
R_T = d.BOOM_TUBE[0]/2

def _curved_patch(footprint_xy, r0, r1, side):
    """Shell between radii r0 < r1 about the boom axis (local frame), limited to a radial prism of the 2D footprint
    (in the plane tangent to the tube at angle `side`: 'y-' inboard side or 'z-' bottom)."""
    P = footprint_xy
    if side == "y-":
        prism = extrude(Face(Wire.make_polygon([Vector(a, -r0 + 6.0, b) for a, b in P], close=True)), 30.0, dir=(0, -1, 0))
    else:
        prism = extrude(Face(Wire.make_polygon([Vector(a, b, -r0 + 6.0) for a, b in P], close=True)), 30.0, dir=(0, 0, -1))
    shell = cyl_between((-200, 0, 0), (200, 0, 0), r1) - cyl_between((-201, 0, 0), (201, 0, 0), r0)
    return shell & prism

def _stadium(cx, L, W, n=24):
    r = W/2; a = L/2 - r
    pts = [(cx + a + r*math.cos(t), r*math.sin(t)) for t in np.linspace(-math.pi/2, math.pi/2, n)]
    pts += [(cx - a + r*math.cos(t), r*math.sin(t)) for t in np.linspace(math.pi/2, 3*math.pi/2, n)]
    return pts

def grommet_slot():
    H = FB.TUBE_HOLES; L, W = H["slot"]
    flange = _curved_patch(_stadium(0, L + 6, W + 6), R_T, R_T + 1.6, "y-")
    collar = _curved_patch(_stadium(0, L - 0.2, W - 0.2), R_T - 1.5 - 1.2, R_T, "y-")
    s = flange.fuse(collar).clean()
    hole = _curved_patch(_stadium(0, L - 3.0, W - 3.0), R_T - 4, R_T + 3, "y-")
    s = s.cut(hole)
    return biggest(s).moved(Location((H["slot_x"], d.BOOM_Y, d.BOOM_Z)))

def grommet_esc():
    H = FB.TUBE_HOLES; D = H["esc_d"]
    circ = lambda r: [(r*math.cos(t), r*math.sin(t)) for t in np.linspace(0, 2*math.pi, 40, endpoint=False)]
    flange = _curved_patch(circ(D/2 + 3.0), R_T, R_T + 1.6, "z-")
    collar = _curved_patch(circ(D/2 - 0.1), R_T - 1.5 - 1.2, R_T, "z-")
    s = flange.fuse(collar).clean().cut(_curved_patch(circ(D/2 - 1.6), R_T - 4, R_T + 3, "z-"))
    return biggest(s).moved(Location((d.FRONT_MOTOR_X + H["esc_x"], d.BOOM_Y, d.BOOM_Z)))


# ------------------------------------------------------------------ jobs
JOBS = ["fc_frame", "fc_plate", "altum_deck", "pdb_plate", *[f"ad_{k}" for k in ADAPT], "dls", "push_esc", "probe_socket", "probe_shield",
        "ret_wing", "ret_tail", "tip_strobe", "belly_strobe", "ant_base", "gr_slot", "gr_esc"] if config.SCOUT else []

def build(job):
    E = "elec"
    if job == "fc_frame":
        return [FabPart("elec_fc_iso_frame", E, fc_iso_frame(), "CF-PETG", fill=0.6, up=(0, 0, -1),
                        notes="FC isolator frame: hangs under the battery tray on 4x M3 (heat-set, new tray holes) at the CG; 4 damper balls D10x8 (ASSUMED stiffness, MEASURE IMU vibe)",
                        print_notes="top (tray) face on the bed, 5 walls, 60 %")]
    if job == "fc_plate":
        return [FabPart("elec_fc_iso_plate", E, fc_iso_plate(), "CF-PETG", fill=0.8, up=(0, 0, 1),
                        notes="FC plate on the dampers; ARK PAB carrier 4x M2.5 on 66x35.5 (ASSUMED, MEASURE vs the ARK STEP) + strap slots", print_notes="flat")]
    if job == "altum_deck":
        return [FabPart("pay_altum_deck", E, altum_deck(), "CF-PETG", fill=0.7, up=(0, 0, 1),
                        notes="replaces fuse_avionics_tray_B: bolts to the 4 tray-B lugs on F1/F2; MicaSense Altum-PT hangs under it on 4x M3 (cross 60/35 [S]); ribs for the 406 g camera; cable slot aft",
                        print_notes="flat, 4 walls, 40 %")]
    if job == "pdb_plate":
        return [FabPart("elec_pdb_plate", E, pdb_plate(), "CF-PETG", fill=0.7, up=(0, 0, -1),
                        notes="PDB plate on 4x M3x30 standoffs under the battery tray: top = MEGA fuse + PM08D (79x38.1 [S]); bottom = bus bar + BEC; ribs along the edges",
                        print_notes="top face down, 4 walls, 50 %")]
    if job.startswith("ad_"):
        k = job[3:]
        return [FabPart(f"elec_mount_{k}", E, adapter(k), "PETG", fill=0.6, up=(0, 0, 1),
                        notes=f"tray-A adapter for {item(k)['name']}: 2x M3 on the 20 mm grid, side lips + zip-tie slots", print_notes="flat")]
    if job == "dls":
        return [FabPart("elec_dls_pedestal", E, dls_pedestal(), "ASA", fill=0.5, up=(0, 0, 1),
                        notes="DLS 2 pedestal, bonded on the avionics hatch top at x -290 (level top, 2x M2 40.75 [S]); cable through the hatch",
                        print_notes="flat top up, tree supports under the curved base")]
    if job == "push_esc":
        return [FabPart("fuse_pusher_esc_mount", E, pusher_esc_mount(), "ASA", fill=0.6, up=(0, 0, 1),
                        notes="pusher ESC (FLAME 80A) deck, webs bond to the fuse_seg6 floor; belly inlets / side outlets in fuse_seg6",
                        print_notes="deck up, 4 walls")]
    if job == "probe_socket":
        return [FabPart("elec_probe_socket", E, probe_socket(), "ASA", fill=1.0, up=(1, 0, 0),
                        notes="bonds on the nose cap; socket for the 8x6 CF probe tube (I2C cable inside)", print_notes="disc on the bed")]
    if job == "probe_shield":
        return [FabPart("elec_probe_shield", E, probe_shield(), "ASA", fill=1.0, up=(0, 0, 1),
                        notes="SHT45 radiation shield: 4 conical gill rings + cap, sensor cage, tube socket; WHITE ASA; own design (MEASURE radiative error vs a reference)",
                        print_notes="upright (cap up), 45-deg rings print without supports, 2 walls")]
    if job == "ret_wing":
        p = FabPart("elec_servo_retainer_wing_R", E, servo_retainer_wing(), "PETG", fill=1.0, up=(0, 0, 1),
                    notes="covers the aileron-servo pocket (31x43): 2x M2 into the o4 bosses, servo ears 36 apart [S]; arm slot position MEASURE", print_notes="flat")
        return [p, mirror_part(p, "elec_servo_retainer_wing_L")]
    if job == "ret_tail":
        p = FabPart("elec_servo_retainer_tail_R", E, servo_retainer_tail(), "PETG", fill=1.0, up=tuple(-np.array(FT.N_HAT)),
                    notes="covers the ruddervator-servo pocket: 2x M2 into the tail_panel1 bosses", print_notes="flat")
        return [p, mirror_part(p, "elec_servo_retainer_tail_L")]
    if job == "tip_strobe":
        p = FabPart("elec_wingtip_strobe_R", E, wingtip_strobe(), "ASA", fill=0.6, up=(0, 1, 0),
                    notes="wing-tip cap (closes the open o5 end; 2 spigots into the front/rear cavities) + pocket for a Firehouse ARC V (self-powered, VHB)",
                    print_notes="cap face on the bed")
        return [p, mirror_part(p, "elec_wingtip_strobe_L")]
    if job == "belly_strobe":
        return [FabPart("elec_belly_strobe", E, belly_strobe(), "ASA", fill=0.6, up=(0, 0, -1),
                        notes="belly strobe cradle at x 460, bonded on the skin, ARC V lens down", print_notes="pocket up")]
    if job == "ant_base":
        return [FabPart("elec_ant_base", E, ant_base(), "ASA", fill=1.0, up=(0, 0, 1),
                        notes="RP-SMA bulkhead base for the 915 MHz dipole on top of fuse_seg5 (x 540)", print_notes="flat")]
    if job == "gr_slot":
        p = FabPart("boom_grommet_slot_R", "boom", grommet_slot(), "TPU95A", fill=1.0, up=(0, 1, 0),
                    notes="TPU grommet in the 26x16 trunk slot (inboard side, neutral axis) of each boom tube at x 145", print_notes="flange down, 100 %")
        return [p, mirror_part(p, "boom_grommet_slot_L")]
    if job == "gr_esc":
        return [FabPart("boom_grommet_esc", "boom", grommet_esc(), "TPU95A", fill=1.0, up=(0, 0, 1), instances=FB._four(),
                        notes="TPU grommet in the D12 ESC-lead hole (tube bottom, local x 110) at each motor station", print_notes="flange down, 100 %")]
    raise KeyError(job)
