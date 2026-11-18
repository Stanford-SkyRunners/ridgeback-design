"""SR-3 boom system, fabrication version (per side: one 30x1.5 CF tube, x -548..1360 at y = +/-600, z = -25).

On each tube, front to back:
  front motor station (x -458): pylon upper + pylon lower (split clamp, CF-PETG), CNC motor plate, nacelle
      (two LW-ASA halves; the ALPHA ESC rides inside, under the tube), landing leg (2x 8 mm CF rods,
      printed fairing, TPU foot)
  wing saddles at x 82 (under the spar) and x 190: CF-PETG, top conforms to the wing, 4x M5 each into the
      wing's integral hardpoints; tube is bonded + M4 cross-pinned
  rear motor station (x 720): same as the front
  tail socket at the tube end: see f_tail.py
Everything on the tube slides on from the ends before the tail socket is bonded.
"""
import math
import numpy as np
from build123d import *
from core import d, FabPart, af_piece, resample, loft, hollow, bx, cyl_between, isect, biggest, mirror_part
import f_wing as FW

BY, BZ = d.BOOM_Y, d.BOOM_Z
R_TUBE = d.BOOM_TUBE[0]/2
BORE = R_TUBE + 0.15
MX = (d.FRONT_MOTOR_X, d.REAR_MOTOR_X)
PY = dict(lx=52.0, hw=27.0, ztop=2.0, zbot=-70.0)        # motor seat z 2 (concept 24): props are fore/aft of the wing, no clearance issue            # pylon block
CLAMP = [(dx, dy) for dx in (-18.0, 18.0) for dy in (-21.0, 21.0)]   # 4x M4 clamp bolts (from below, into inserts)
PLATE_BOLTS = [(dx, dy) for dx in (-20.0, 20.0) for dy in (-18.0, 18.0)]
PLATE = dict(d=84.0, t=3.0)                         # 3 mm carbon plate
MOTOR_PATTERN = dict(pcd=30.0, hole=4.3, n=4)                 # T-Motor U8 II: PLACEHOLDER - MEASURE the real pattern before cutting
ROD = dict(d=8.0, dx=8.25, top=-43.0, bot=-330.0)             # two 8 mm CF rods per leg
LEG = dict(chord=55.0, z0=-72.0, z1=-300.0)
FOOT = dict(r=20.0, z0=d.GROUND_Z, z1=-300.0, sock=30.0)
NAC = dict(len=280.0, w=76.0, z0=-100.0, z1=14.0, skin=1.2)
SADDLE = dict(lx=58.0, hw=32.0, zbot=-45.0, cb_top=-30.0)

# ------------------------------------------------------------------ Scout (config.SCOUT, PROVISIONAL)
import config
SCOUT = config.SCOUT
NAC_X0 = -NAC["len"]/2                                    # nacelle front, local x
# ALPHA 60A 12S V1.2: 78.7 x 32.5 x 16.2 mm, 73 g [S store.tmotor.com goods 582]. On its side (16.2 across y) under the
# tube, aft of the pylon; local frame (origin tube centre, x from the motor axis).
ESC = dict(l=78.7, w=32.5, t=16.2, x0=41.0, ztop=-17.0)
ESC["zc"] = ESC["ztop"] - ESC["w"]/2
CRADLE = dict(clips=(34.0, 121.0), clip_w=6.0, wall=2.0, open_deg=55.0, floor_t=2.0, hw=ESC["t"]/2 + 2.4)
TUBE_HOLES = dict(esc_x=110.0, esc_d=12.0, slot_x=145.0, slot=(26.0, 16.0), tail_d=6.0)   # boom tube holes (see assemble/harness)
if SCOUT:
    # the stock nacelle (280 long, centred on the motor) tapers to ~24 mm half-width by local x 120: no room for the
    # ESC cradle clips. Scout nacelle: 300 long, x -130..170 (+30 aft, -10 front); 300 is the longest half that still
    # fits the 256 bed diagonally (split face down).
    NAC = dict(NAC, len=300.0)
    NAC_X0 = -130.0


# ------------------------------------------------------------------ smaller own design (config.MINI, 2026-11-02)
MINI = config.MINI
if MINI:
    # MN5008 + P18 lift, 30/28 boom at y +/-390, z -20. Motor hole pattern and ESC size are placeholders (MEASURE).
    PY = dict(lx=46.0, hw=25.0, ztop=2.0, zbot=-65.0)
    CLAMP = [(dx, dy) for dx in (-15.0, 15.0) for dy in (-19.5, 19.5)]
    PLATE_BOLTS = [(dx, dy) for dx in (-16.0, 16.0) for dy in (-16.0, 16.0)]
    PLATE = dict(d=64.0, t=3.0)
    MOTOR_PATTERN = dict(pcd=25.0, hole=3.3, n=4)      # T-Motor MN5008: PLACEHOLDER - MEASURE before cutting
    ROD = dict(d=8.0, dx=8.25, top=-43.0, bot=-235.0)  # 8/6 legs, 192 mm
    LEG = dict(chord=48.0, z0=-67.0, z1=-215.0)
    FOOT = dict(r=18.0, z0=d.GROUND_Z, z1=-215.0, sock=25.0)
    NAC = dict(len=220.0, w=64.0, z0=-85.0, z1=10.0, skin=1.2)
    NAC_X0 = -NAC["len"]/2
    SADDLE = dict(lx=50.0, hw=28.0, zbot=-40.0, cb_top=-27.0)


def at_boom(s, mx=0.0):
    return s.moved(Location((mx, BY, BZ)))


# ------------------------------------------------------------------ pylon (local: tube axis = x, origin at tube centre)
def pylon_upper():
    zt = PY["ztop"] - BZ                                       # 49
    s = bx(-PY["lx"]/2, PY["lx"]/2, -PY["hw"], PY["hw"], 0, zt)
    cuts = [cyl_between((-50, 0, 0), (50, 0, 0), BORE)]
    cuts += [cyl_between((x, y, zt - 8), (x, y, zt + 1), 2.8) for x, y in PLATE_BOLTS]        # M4 heat-set (5.6)
    cuts += [cyl_between((x, y, -1), (x, y, 9), 2.8) for x, y in CLAMP]                      # M4 heat-set from the split face
    cuts += [cyl_between((0, 0, zt + 1), (0, 0, 20), 5.0), cyl_between((0, 0, 20), (40, 0, 20), 5.0)]   # motor wires: down, then aft to the ESC
    cuts += [cyl_between((0, -40, 0), (0, 40, 0), 2.2)]                                       # M4 cross-pin through the tube
    return s.cut(*cuts).clean()

def pylon_lower():
    zb = PY["zbot"] - BZ                                       # -65
    s = bx(-PY["lx"]/2, PY["lx"]/2, -PY["hw"], PY["hw"], zb, 0)
    cuts = [cyl_between((-50, 0, 0), (50, 0, 0), BORE)]
    for x, y in CLAMP:
        cuts += [cyl_between((x, y, zb - 1), (x, y, 1), 2.25), cyl_between((x, y, zb - 1), (x, y, zb + 16), 4.2)]   # M4 clearance + head counterbore
    for sx in (-1, 1):
        cuts.append(cyl_between((sx*ROD["dx"], 0, zb - 1), (sx*ROD["dx"], 0, ROD["top"] - BZ), ROD["d"]/2 + 0.1))   # leg rod sockets
    cuts += [cyl_between((0, -40, 0), (0, 40, 0), 2.2)]
    return s.cut(*cuts).clean()

def motor_plate():
    with BuildSketch() as sk:
        Circle(PLATE["d"]/2)
        for x, y in PLATE_BOLTS:
            with Locations((x, y)):
                Circle(2.15, mode=Mode.SUBTRACT)
        r = MOTOR_PATTERN["pcd"]/2
        for k in range(MOTOR_PATTERN["n"]):
            a = math.radians(45 + 360*k/MOTOR_PATTERN["n"])
            with Locations((r*math.cos(a), r*math.sin(a))):
                Circle(MOTOR_PATTERN["hole"]/2, mode=Mode.SUBTRACT)
        Circle(7.0, mode=Mode.SUBTRACT)
    f = sk.sketch.faces()[0]
    return extrude(f, PLATE["t"]), f

# ------------------------------------------------------------------ nacelle (two halves, split on the boom plane)
def _nac_sec(x, off):
    from shapely.geometry import Polygon
    from core import largest
    u = (x - NAC_X0)/NAC["len"]
    f = max(0.35, math.sin(math.pi*u)**0.3)
    a = NAC["w"]/2*f; b = (NAC["z1"] - NAC["z0"])/2*f; zc = (NAC["z0"] + NAC["z1"])/2 - BZ
    t = np.linspace(0, 2*np.pi, 160, endpoint=False); e = 2/2.6
    P = Polygon(np.stack([a*np.sign(np.cos(t))*np.abs(np.cos(t))**e, zc + b*np.sign(np.sin(t))*np.abs(np.sin(t))**e], 1))
    return largest(P.buffer(-off, join_style=1)) if off > 0 else P

def _nac_place(pts, x):
    return Wire.make_polygon([Vector(float(x), float(a), float(b)) for a, b in pts], close=True)

def nacelle():
    xs = list(np.linspace(NAC_X0, NAC_X0 + NAC["len"], 15))
    s, _ = hollow((xs[0], xs[-1]), _nac_sec, _nac_place, NAC["skin"], closed=(True, True), cap=NAC["skin"], kinks=xs[1:-1])
    cuts = [cyl_between((-200, 0, 0), (200, 0, 0), R_TUBE + 0.6),                              # tube passes through
            cyl_between((0, 0, PY["ztop"] - BZ - 6), (0, 0, 120), PLATE["d"]/2 + 2),            # motor plate / motor
            bx(-LEG["chord"]/2 - 3, LEG["chord"]/2 + 3, -9, 9, -200, LEG["z0"] - BZ + 0.5),     # leg exit
            *([cyl_between((x, -50, NAC["z1"] - BZ - 14), (x, 50, NAC["z1"] - BZ - 14), 3.0) for x in ((75.0, 88.0) if MINI else (95.0, 110.0))] if not SCOUT else [])]   # ESC cooling vents (relay)
    if SCOUT:   # hover downwash in through 4 top slots over the ESC, out through 4 belly slots (cruise: ESC idle)
        for sy in (-1, 1):
            for xa, xb in ((50.0, 78.0), (86.0, 114.0)):
                ya, yb = sorted((sy*4.0, sy*13.0))
                cuts.append(bx(xa, xb, ya, yb, 18.0, 80.0))
                ya, yb = sorted((sy*3.0, sy*11.0))
                cuts.append(bx(xa + 6, xb + 6, ya, yb, -120.0, ESC["ztop"] - ESC["w"] - CRADLE["floor_t"] - 2.5))
    s = s.cut(*cuts).clean()
    return biggest(s & bx(-200, 200, 0, 100, -200, 200)), biggest(s & bx(-200, 200, -100, 0, -200, 200))

# ------------------------------------------------------------------ legs
def leg_fairing():
    def sec(z, off):
        return af_piece("naca0025", LEG["chord"], 0, 1, off)
    def place(pts, z):
        return Wire.make_polygon([Vector(float(a) - 0.4*LEG["chord"], float(b), float(z)) for a, b in pts], close=True)
    s = loft([place(resample(sec(z, 0)), z) for z in (LEG["z1"] - BZ, LEG["z0"] - BZ)])
    cuts = [cyl_between((sx*ROD["dx"], 0, LEG["z1"] - BZ - 1), (sx*ROD["dx"], 0, LEG["z0"] - BZ + 1), ROD["d"]/2 + 0.15) for sx in (-1, 1)]
    return s.cut(*cuts).clean()

def foot():
    z0, z1 = FOOT["z0"] - BZ, FOOT["z1"] - BZ
    s = Cylinder(FOOT["r"], z1 - z0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((0, 0, z0)))
    s = chamfer(s.edges().sort_by(Axis.Z)[0], 6)
    cuts = [cyl_between((sx*ROD["dx"], 0, z1 - FOOT["sock"]), (sx*ROD["dx"], 0, z1 + 1), ROD["d"]/2 - 0.1) for sx in (-1, 1)]   # press fit
    return s.cut(*cuts).clean()

# ------------------------------------------------------------------ wing saddles (aircraft frame, right side)
def saddle(i):
    cx = FW.SADDLE_X[i]
    x0, x1 = cx - SADDLE["lx"]/2, cx + SADDLE["lx"]/2
    zl = max(FW.z_bounds(y, x)[0] for y in (BY - SADDLE["hw"], BY + SADDLE["hw"]) for x in (x0, cx, x1))
    s = bx(x0, x1, BY - SADDLE["hw"], BY + SADDLE["hw"], SADDLE["zbot"], zl + 6)
    s = s - FW.full_outer(BY - SADDLE["hw"] - 5, BY + SADDLE["hw"] + 5)
    cuts = [cyl_between((x0 - 5, BY, BZ), (x1 + 5, BY, BZ), BORE),
            cyl_between((cx, BY - 50, BZ), (cx, BY + 50, BZ), 2.2)]                              # M4 cross-pin
    for x, y in FW.bolt_points(i):
        cuts += [cyl_between((x, y, SADDLE["zbot"] - 1), (x, y, 40), 2.75),
                 cyl_between((x, y, SADDLE["zbot"] - 1), (x, y, SADDLE["cb_top"]), 5.0)]          # M5 + head counterbore
    return biggest(s.cut(*cuts).clean())


# ------------------------------------------------------------------ Scout: ESC cradle (local frame)
def esc_cradle():
    """Two C-clips (open on top) snap on the boom tube either side of the ESC; a floor and two hangers carry the ESC on
    its side with air all round. ESC tied down with 2 zip ties through the floor slots."""
    C, E = CRADLE, ESC
    r_in, r_out = BORE, BORE + C["wall"]
    zf1 = E["ztop"] - E["w"]; zf0 = zf1 - C["floor_t"]
    parts = []
    for xa in C["clips"]:
        ring = cyl_between((xa, 0, 0), (xa + C["clip_w"], 0, 0), r_out) - cyl_between((xa - 1, 0, 0), (xa + C["clip_w"] + 1, 0, 0), r_in)
        a = math.radians(C["open_deg"]); h = 40.0
        wedge = extrude(Face(Wire.make_polygon([Vector(xa - 1, 0, 0), Vector(xa - 1, h*math.sin(a), h*math.cos(a)), Vector(xa - 1, -h*math.sin(a), h*math.cos(a))], close=True)), C["clip_w"] + 2, dir=(1, 0, 0))
        parts.append(ring - wedge)
        parts.append(bx(xa, xa + C["clip_w"], -C["hw"], C["hw"], zf0, -r_in + 0.6) - cyl_between((xa - 1, 0, 0), (xa + C["clip_w"] + 1, 0, 0), r_in))
    x0, x1 = C["clips"][0], C["clips"][1] + C["clip_w"]
    floor = bx(x0, x1, -C["hw"], C["hw"], zf0, zf1)
    lips = [bx(x0, x1, sy*C["hw"] - (2.0 if sy > 0 else 0), sy*C["hw"] + (0 if sy > 0 else 2.0), zf1, zf1 + 6.0) for sy in (1, -1)]
    s = parts[0].fuse(*parts[1:], floor, *lips).clean()
    slots = [bx(x - 2.0, x + 2.0, -C["hw"] + 1.5, C["hw"] - 1.5, zf0 - 1, zf1 + 1) for x in (E["x0"] + 18.0, E["x0"] + E["l"] - 18.0)]
    vent = [bx(E["x0"] + 26.0, E["x0"] + E["l"] - 26.0, -C["hw"] + 2.5, C["hw"] - 2.5, zf0 - 1, zf1 + 1)]
    return biggest(s.cut(*slots, *vent).clean())

def tube_holes_local():
    """Holes in the 30x1.5 boom tube near one motor station (local frame): the ESC power-lead exit (bottom)."""
    H = TUBE_HOLES
    return [cyl_between((H["esc_x"], 0, -25), (H["esc_x"], 0, 0), H["esc_d"]/2)]


# ------------------------------------------------------------------ jobs
def _four(dx_only=False):
    dx = MX[1] - MX[0]
    return [(dx, 0, 0), (0, -2*BY, 0), (dx, -2*BY, 0)]

JOBS = ["pylon_upper", "pylon_lower", "motor_plate", "nacelle", "leg", "foot", "saddle0", "saddle1"]
if SCOUT:
    JOBS = JOBS + ["esc_cradle"]

def build(job):
    inst = _four()
    if job == "pylon_upper":
        return [FabPart("boom_pylon_upper", "boom", at_boom(pylon_upper(), MX[0]), "CF-PETG", fill=0.55, up=(0, 0, -1), instances=inst,
                        notes="motor pylon, upper clamp half; 4x M4 inserts for the motor plate, 4x M4 inserts for the clamp", print_notes="motor-plate face on the bed (half-bore opens upward, no supports); 6 walls, 60 % infill")]
    if job == "pylon_lower":
        return [FabPart("boom_pylon_lower", "boom", at_boom(pylon_lower(), MX[0]), "CF-PETG", fill=0.55, up=(0, 0, 1), instances=inst,
                        notes="motor pylon, lower clamp half + leg rod sockets (2x 8 mm, 27 deep, bond)", print_notes="bottom face down, 6 walls, 60 % infill")]
    if job == "motor_plate":
        s, f = motor_plate()
        s = s.moved(Location((MX[0], BY, PY["ztop"]))); f = f.moved(Location((MX[0], BY, PY["ztop"])))
        return [FabPart("boom_motor_plate", "boom", s, "CF-plate", up=(0, 0, 1), instances=inst, dxf=f,
                        notes="3 mm carbon plate, CNC/waterjet from DXF; MOTOR HOLE PATTERN IS A PLACEHOLDER - measure the " + ("MN5008" if MINI else "U8 II") + " before cutting")]
    if job == "nacelle":
        a, b = nacelle()
        pa = FabPart("boom_nacelle_half_out", "boom", at_boom(a, MX[0]), "LW-ASA", up=(0, -1, 0), instances=inst,
                     notes="nacelle half (outboard on the right boom); " + ("ALPHA 40A 6S ESC (64.5 x 29.4 x 14.7)" if MINI else "ALPHA 60A ESC") + " under the tube aft of the pylon; tape/CA the halves" + ("; Scout: 300 long (x -130..170), 4 top inlet + 4 belly outlet slots" if SCOUT else ""),
                     print_notes="split face on the bed, 2 walls, 0 % infill")
        pb = FabPart("boom_nacelle_half_in", "boom", at_boom(b, MX[0]), "LW-ASA", up=(0, 1, 0), instances=inst,
                     notes="nacelle half (inboard on the right boom)", print_notes="split face on the bed, 2 walls, 0 % infill")
        return [pa, pb]
    if job == "leg":
        return [FabPart("boom_leg_fairing", "boom", at_boom(leg_fairing(), MX[0]), "CF-PETG", fill=0.35, up=(0, 0, 1), instances=inst,
                        notes="streamlined leg fairing over 2x 8 mm CF rods (287 mm); bond to the rods", print_notes="upright, 3 walls, 10 % gyroid")]
    if job == "foot":
        return [FabPart("boom_foot", "boom", at_boom(foot(), MX[0]), "TPU95A", fill=0.4, up=(0, 0, 1), instances=inst,
                        notes="TPU landing foot, press fit on the rods (compliance for the n_land 2.9 case: drop-test it)", print_notes="2 walls, 15 % gyroid, chamfer down")]
    if job == "esc_cradle":
        return [FabPart("boom_esc_cradle", "boom", at_boom(esc_cradle(), MX[0]), "CF-PETG", fill=0.6, up=(0, 1, 0), instances=inst,
                        notes="Scout: ALPHA 60A cradle, 2 C-clips snap on the boom tube aft of the pylon; ESC on its side, 2 zip ties; open floor + nacelle top/belly slots for cooling",
                        print_notes="on its side (clip faces on the bed), 5 walls, 60 % infill")]
    if job.startswith("saddle"):
        i = int(job[-1])
        p = FabPart(f"boom_saddle_{'fwd' if i == 0 else 'aft'}_R", "boom", saddle(i), "CF-PETG", fill=0.5, up=(0, 0, 1),
                    notes=f"wing saddle at x {FW.SADDLE_X[i]:.0f}: 4x M5x40 up into the wing hardpoint inserts; bond + M4 cross-pin on the tube",
                    print_notes="bottom face down, tree supports under the curved wing face; 6 walls, 35 % infill")
        p.up = (0, 0, 1)
        return [p, mirror_part(p, p.name[:-2] + "_L")]
    raise KeyError(job)
