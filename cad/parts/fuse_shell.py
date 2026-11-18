"""SR-3 fuselage shell (relay airframe, no pod bay): lofted superellipse sections (boxy mid-body around the 2x2 bay, rounder nose/tail),
1.5 mm composite skin (production intent: glass/carbon over foam-core bulkheads), cutouts for the bay doors,
chin turret, nadir camera, top battery/avionics hatches and the pusher firewall. Units mm.
"""
import numpy as np
from build123d import *
import design as d

NAME = "fuse_shell"
MATERIAL = "CF"          # mass via mass_g(): composite skin areal density, not solid density
INFILL = 1.0
QTY = 1
MIRROR = False
SKIN_T = 1.5             # [A] laminate thickness for the CAD solid
AREAL_G_M2 = 900.0       # [A] 2 plies 200 g/m2 glass + 1 ply carbon + epoxy + foam stiffeners, per m2 of wetted area

# (x, width, height, z_top); z_top = 0 under the wing; the tail rises to put the pusher on its thrust line
STATIONS = [                                    # SR-3 relay airframe (no bay)
    (-550.0,  50.0,  50.0, -60.0),
    (-520.0, 140.0, 130.0, -12.0),
    (-460.0, 195.0, 190.0,  -2.0),
    (-380.0, 210.0, 208.0,   0.0),
    (-250.0, d.FUSE_W, d.FUSE_H, 0.0),
    ( -50.0, d.FUSE_W, d.FUSE_H, 0.0),
    ( 150.0, d.FUSE_W, d.FUSE_H, 0.0),
    ( 350.0, d.FUSE_W, d.FUSE_H, 0.0),
    ( 500.0, 175.0, 170.0,   0.0),
    ( 650.0, 120.0, 115.0, -25.0),
    ( 740.0,  90.0,  85.0, -45.0),
    ( d.FUSE_X1, 70.0, 70.0, d.PUSHER_Z + 35.0),
]

def _wire(x, w, h, zt, shrink=0.0, k=128):
    n = d.fuse_n(x)
    a, b = w/2 - shrink, h/2 - shrink
    zc = zt - h/2
    t = np.linspace(0, 2*np.pi, k, endpoint=False)
    e = 2.0/n
    y = a*np.sign(np.cos(t))*np.abs(np.cos(t))**e
    z = zc + b*np.sign(np.sin(t))*np.abs(np.sin(t))**e
    pts = [Vector(float(x), float(yy), float(zz)) for yy, zz in zip(y, z)]
    return Wire([Spline(*pts, pts[0])])

def _loft(shrink=0.0, ends=(0.0, 0.0)):
    ws = []
    for i, (x, w, h, zt) in enumerate(STATIONS):
        x += ends[0] if i == 0 else ends[1] if i == len(STATIONS) - 1 else 0.0
        ws.append(_wire(x, w, h, zt, shrink))
    return Solid.make_loft(ws)

def outer():
    return _loft()

def make():
    o = outer()
    shell = o - _loft(SKIN_T, (SKIN_T*4, 2.0))
    cuts = []
    # chin turret mount opening
    cuts.append(Cylinder(62, 80).moved(Location((d.TURRET["x"], 0, -d.FUSE_H + 10))))
    # nadir camera window
    cuts.append(Box(60, 50, 60).moved(Location((d.NADIR_CAM_X + 40, 0, -d.FUSE_H + 10))))
    # top hatches: avionics (nose) and the wing-root opening for the battery (covered by the wing centre section)
    cuts.append(Box(d.AVIONICS_BAY[1] - d.AVIONICS_BAY[0] - 40, d.FUSE_W - 60, 60).moved(Location((sum(d.AVIONICS_BAY)/2 + 10, 0, 0))))
    cuts.append(Box(d.ROOT_CHORD - 40, d.FUSE_W - 50, 40).moved(Location((d.ROOT_CHORD/2, 0, 0))))
    # pusher shaft hole in the tail cone
    cuts.append(Cylinder(14, 40, rotation=(0, 90, 0)).moved(Location((d.FUSE_X1, 0, d.PUSHER_Z))))
    return shell.cut(*cuts).clean()

def mass_g():
    """Areal-density estimate from the outer wetted area (the CAD solid's volume is not the mass)."""
    area_m2 = outer().area/1e6
    return area_m2*AREAL_G_M2

def placements():
    return [Location((0, 0, 0))]

if __name__ == "__main__":
    from common import export, preview
    p = make()
    print("valid", p.is_valid, f"wetted area {outer().area/1e6:.2f} m2 -> {mass_g():.0f} g")
    export(p, NAME)
    preview(NAME, elev=-25, azim=-60)
