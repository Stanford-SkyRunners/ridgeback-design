"""SR-2 mission bay frame: 2x2 slot cage centred on the CG under the battery (F2 decision D2).

Local frame: origin at (BAY_CENTER_X, 0, BAY_Z_FLOOR) = slot-floor plane, bay centre; placed with no rotation.
  z 0..SLOT.z  slot envelopes (pods hang inside on their winch hooks, 8 mm above the closed doors)
  z SLOT.z..+3 deck plate: winches sit on it, line fairlead holes + pogo connector blocks on its underside
  z -3..0      floor grid with two spring-closed flaps per slot (shown closed). The pod pushes the flaps
               open as it is lowered; the torsion springs close them behind it with the line running
               through the centre notch, so the door IS the line exit.
End plates at x = +/-(bay half-length + 3) are the bulkhead flanges: M3 bolt ring to fuselage ring frames at
aircraft x = BULKHEAD_X (ASSUMED stations: the fuse_* agent must put frames there).
Corner L-guides (2 mm in the 3 mm y-band, 3 mm in the 5 mm x-band) are the slot rails and the posts.
CF-PETG prototype. Units mm.
"""
import math
from build123d import *
import design as d

NAME = "pay_bay_frame"
MATERIAL = "CF-PETG"
INFILL = 0.5
QTY = 1
MIRROR = False

SX, SY, SZ = d.SLOT["x"], d.SLOT["y"], d.SLOT["z"]
PX, PY = d.SLOT_PITCH["x"], d.SLOT_PITCH["y"]
SLOT_CENTERS = [(sx*PX/2, sy*PY/2) for sx in (-1, 1) for sy in (-1, 1)]        # local (x, y)
SLOT_CENTERS_AC = [(d.BAY_CENTER_X + x, y) for x, y in SLOT_CENTERS]          # aircraft (x, y)
DECK_T = 3.0              # (ASSUMED) CF-PETG deck
FLOOR_T = 3.0             # (ASSUMED) floor grid
END_T = 3.0               # (ASSUMED) end plates = bulkhead flanges
GUIDE_T_Y, GUIDE_T_X, GUIDE_LEG = 2.0, 3.0, 22.0     # (ASSUMED) corner rails; pod-to-rail clearance 1 / 2 mm
SPINE_T = 2.4             # (ASSUMED) divider plates in the 10 / 6 mm gaps between slots
LIP = 2.0                 # floor lip around each door opening (ASSUMED)
DOOR_T = 2.4              # (ASSUMED) flap thickness
DOOR_GAP = 0.6            # (ASSUMED) flap-to-frame / flap-to-flap gap
LINE_NOTCH_D = 8.0        # (ASSUMED) line + hook shank pass through the closed flaps
FAIRLEAD_D = 14.0         # deck hole under each winch bellmouth (pay_winch)
CONN = dict(x=60.0, w=24.0, l=12.0, h=2.0)   # 4-pin pogo block (12 V + CAN) under deck at slot (+60, 0); envelope (MEASURE)
DENSITY = 1.30            # (ASSUMED) CF-PETG
RELIEF_Z = 5.0            # local z below which the frame is trimmed to the slot width (+/-139)
SKIN_T = 1.5              # fuselage skin [A] in fuse_shell.py
HALF_L = PX/2 + SX/2                          # 215
HALF_W = PY/2 + SY/2 + 2.0                    # 141: outer rim / end plates (slots end at 139)
DECK_Z = SZ                                   # local deck underside = slot top
DECK_Z_AC = d.BAY_Z_FLOOR + SZ + DECK_T       # aircraft z of the deck top (winch base)
BULKHEAD_X = (d.BAY_CENTER_X - HALF_L - END_T, d.BAY_CENTER_X + HALF_L + END_T)   # (ASSUMED) frame stations
WINCH_HOLES = [(-50 + dx, dy) for dx in (-40, 40) for dy in (-30, 30)]          # M3 inserts per slot (pay_winch base)


def fuse_inner_half_width(x, z):
    """Inner half-width of the superellipse fuselage at aircraft (x, z) (mid-body: FUSE_W x FUSE_H, top z=0)."""
    n = d.fuse_n(x)
    a, b = d.FUSE_W/2 - SKIN_T, d.FUSE_H/2 - SKIN_T
    r = abs(z + d.FUSE_H/2)/b
    return 0.0 if r >= 1 else a*(1 - r**n)**(1/n)


def fit_checks(verbose=True):
    msgs, worst = [], 1e9
    for x in (BULKHEAD_X[0], BULKHEAD_X[1]):
        for zl in (-FLOOR_T, 0.0, SZ, SZ + DECK_T):
            z = d.BAY_Z_FLOOR + zl
            m = fuse_inner_half_width(x, z) - HALF_W
            worst = min(worst, m)
            if m < 0:
                msgs.append(f"FAIL bay frame outside fuselage at x={x:.0f} z={z:.0f}: margin {m:.1f} mm")
    if verbose:
        print(f"[bay] min lateral margin frame-to-inner-skin {worst:.1f} mm over x {BULKHEAD_X[0]:.0f}..{BULKHEAD_X[1]:.0f}")
        bat_bot = d.BATTERY_Z_TOP - d.BATTERY["z"]
        print(f"[bay] deck top z={DECK_Z_AC:.0f}, battery bottom z={bat_bot:.0f}: {bat_bot - DECK_Z_AC:.0f} mm for winches")
        print(f"[bay] keel z={-d.FUSE_H:.0f}, door outer face z={d.BAY_Z_FLOOR - FLOOR_T:.0f}: "
              f"doors recessed {d.BAY_Z_FLOOR - FLOOR_T + d.FUSE_H:.0f} mm above the keel line")
        for m in msgs:
            print(m)
    assert d.POD["x"] < SX and d.POD["y"] + 2*GUIDE_T_Y <= SY and d.POD["z"] < SZ, "pod does not fit slot+rails"
    try:
        import fuse_shell
        v = fuse_shell.make() & make().moved(placements()[0])
        vol = v.volume if v is not None else 0.0
        print(f"[bay] {'ok' if vol < 1 else 'FAIL'}: frame vs fuse_shell solid intersection {vol:.0f} mm3")
    except ImportError:
        pass
    assert worst >= 0, "bay frame outside the fuselage"
    return worst


def _corner_guides(cx, cy):
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            xe, ye = cx + sx*SX/2, cy + sy*SY/2                      # slot corner
            # leg along x (thin in y) and leg along y (thin in x), both inside the slot envelope
            out.append(Pos(xe - sx*GUIDE_LEG/2, ye - sy*GUIDE_T_Y/2, SZ/2)*Box(GUIDE_LEG, GUIDE_T_Y, SZ))
            out.append(Pos(xe - sx*GUIDE_T_X/2, ye - sy*GUIDE_LEG/2, SZ/2)*Box(GUIDE_T_X, GUIDE_LEG, SZ))
    return out


def make_doors():
    """Two flaps per slot, hinged on the slot's outer y edges, meeting on the slot centreline (closed)."""
    flaps = []
    ox, oy = SX - 2*LIP, SY - 2*LIP           # door opening
    for cx, cy in SLOT_CENTERS:
        for s in (-1, 1):
            w = oy/2 - 1.5*DOOR_GAP
            f = Pos(cx, cy + s*(DOOR_GAP/2 + w/2), -FLOOR_T + DOOR_T/2)*Box(ox - 2*DOOR_GAP, w, DOOR_T)
            f -= Pos(cx, cy, -FLOOR_T/2)*Cylinder(LINE_NOTCH_D/2, 10)
            # hinge knuckles on the outer edge (torsion-spring hinge pins along x; springs (MEASURE))
            for kx in (-70, 0, 70):
                f += Pos(cx + kx, cy + s*(oy/2 - DOOR_GAP - 2.5), 0.2)*Rot(0, 90, 0)*Cylinder(2.5, 20)
            flaps.append(f)
    return flaps


def make(with_doors=True):
    # deck
    deck = Pos(0, 0, SZ + DECK_T/2)*Box(2*HALF_L, 2*HALF_W, DECK_T)
    for cx, cy in SLOT_CENTERS:
        deck -= Pos(cx, cy, SZ)*Cylinder(FAIRLEAD_D/2, 20)
        deck -= Pos(cx + CONN["x"], cy, SZ)*Cylinder(4.0, 20)                    # connector cable pass
        for hx, hy in WINCH_HOLES:
            deck -= Pos(cx + hx, cy + hy, SZ)*Cylinder(2.0, 20)                  # M3 insert holes
        for ly in (-38, 38):                                                     # lightening beside the connector
            deck -= Pos(cx + 62, cy + ly, SZ)*Box(56, 36, 20)
        for ly in (-48, 48):                                                     # outboard of the winch base
            deck -= Pos(cx - 50, cy + ly, SZ)*Box(70, 14, 20)
    # floor grid
    floor = Pos(0, 0, -FLOOR_T/2)*Box(2*HALF_L, 2*HALF_W, FLOOR_T)
    for cx, cy in SLOT_CENTERS:
        floor -= Pos(cx, cy, -FLOOR_T/2)*Box(SX - 2*LIP, SY - 2*LIP, 10)
    # end plates (bulkhead flanges), centre spine plates and corner rails
    frame = deck + floor
    for s in (-1, 1):
        ep = Pos(s*(HALF_L + END_T/2), 0, (SZ + DECK_T - FLOOR_T)/2)*Box(END_T, 2*HALF_W, SZ + DECK_T + FLOOR_T)
        for cy in (-PY/2, PY/2):
            ep -= Pos(s*(HALF_L + END_T/2), cy, SZ/2)*Box(10, SY - 40, SZ - 40)    # lightening window
        for yy in (-125, -60, 0, 60, 125):                                        # M3 bolt ring to the ring frame
            for zz in (8.0, SZ - 8.0):
                ep -= Pos(s*(HALF_L + END_T/2), yy, zz)*Rot(0, 90, 0)*Cylinder(1.7, 10)
        frame += ep
    spine_x = Pos(0, 0, SZ/2)*Box(SPINE_T, 2*HALF_W, SZ)                         # between fwd/aft rows
    for cy in (-PY/2, PY/2):
        spine_x -= Pos(0, cy, SZ/2)*Box(20, SY - 30, SZ - 30)
    spine_y = Pos(0, 0, SZ/2)*Box(2*HALF_L, SPINE_T, SZ)                          # between port/stbd columns
    for cx in (-PX/2, PX/2):
        spine_y -= Pos(cx, 0, SZ/2)*Box(SX - 40, 20, SZ - 30)
    frame += spine_x + spine_y
    for s in (-1, 1):   # keel-corner relief: fuse_shell's loft is ~2 mm narrower than the formula near the floor
        y0, z0, z1 = PY/2 + SY/2, -FLOOR_T - 1, RELIEF_Z
        frame -= Pos(0, s*(y0 + 10), (z0 + z1)/2)*Box(2*HALF_L + 20, 20, z1 - z0)
    for cx, cy in SLOT_CENTERS:
        for g in _corner_guides(cx, cy):
            frame += g
        frame += Pos(cx + CONN["x"], cy, SZ - CONN["h"]/2)*Box(CONN["l"], CONN["w"], CONN["h"])
    parts = [frame] + (make_doors() if with_doors else [])
    return Compound(children=parts) if len(parts) > 1 else frame


def placements():
    return [Location((d.BAY_CENTER_X, 0, d.BAY_Z_FLOOR))]


if __name__ == "__main__":
    from common import export, preview
    fit_checks()
    p = make()
    export(p, NAME)
    print(f"[bay] frame+doors mass ~{p.volume/1000*DENSITY*INFILL:.0f} g at {INFILL:.0%} infill (CF-PETG {DENSITY} g/cm3 ASSUMED)")
    preview(NAME, elev=-35, azim=-60, out=str(__import__("common").OUT / f"{NAME}_below.png"))
