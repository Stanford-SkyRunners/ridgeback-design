"""SR-2 wing (fixed structure, full span): flat centre section to the booms, outer panels with dihedral
and washout, spar tunnels for the 30x2 inner / 20x1.5 outer CF tubes [E_STRUCTURES.md], cut-outs for
flaps and ailerons (those are wing_controls.py). Production intent: molded skins over foam core.
"""
import math
import numpy as np
from build123d import *
import design as d

NAME = "wing"
MATERIAL = "CF"
INFILL = 1.0
QTY = 1
MIRROR = False
AREAL_G_M2 = 750.0        # [A] molded sandwich skin + foam core + ribs, per m2 wetted (spars are separate parts)
HINGE_FRAC = 1 - d.CONTROL_CHORD_FRAC
SPAR_X = 105.0            # straight inner spar line (28 % of chord at the root ~111, at the boom ~99)

Z_LE_ROOT = float(-d.section_xz(d.ROOT_CHORD, z_le=0.0)[:, 1].min())

def section(y):
    """Placed airfoil wire at span station y (right side), with incidence, washout and dihedral."""
    yy = abs(y)
    out = max(0.0, (yy - d.CENTER_HALF)/(d.SPAN/2 - d.CENTER_HALF))
    inc = d.WING_INCIDENCE_DEG - d.WASHOUT_DEG*out
    p = d.section_xz(d.chord_at(yy), inc, x_le=d.le_x(yy), z_le=Z_LE_ROOT + d.dihedral_z(yy), n=50)
    return Wire.make_polygon([Vector(float(x), float(y), float(z)) for x, z in p], close=True)

def _half(sign):
    ys = [0.0, d.CENTER_HALF, d.SPAN/2]
    inner = Solid.make_loft([section(sign*ys[0]), section(sign*ys[1])], ruled=True)
    outer = Solid.make_loft([section(sign*ys[1]), section(sign*ys[2])], ruled=True)
    return inner, outer

def surfaces_z(x, chord=None, z_le=None):
    """Upper and lower surface z of the root section at aircraft x."""
    p = d.section_xz(chord or d.ROOT_CHORD, z_le=Z_LE_ROOT if z_le is None else z_le)
    h = len(p)//2
    up, lo = p[:h + 1], p[h:]
    zu = float(np.interp(x, *zip(*sorted(map(tuple, up)))))
    zl = float(np.interp(x, *zip(*sorted(map(tuple, lo)))))
    return zu, zl

def _spar_cut(sign):
    zc = 0.5*sum(surfaces_z(SPAR_X))
    inner = Cylinder(d.INNER_SPAR[0]/2 + 0.2, d.CENTER_HALF + 40, rotation=(90, 0, 0)).moved(Location((SPAR_X, sign*(d.CENTER_HALF + 40)/2, zc)))
    ang = d.DIHEDRAL_DEG
    L = d.SPAN/2 - d.CENTER_HALF - 60
    outer = Cylinder(d.OUTER_SPAR[0]/2 + 0.2, L, rotation=(90, 0, 0))
    outer = outer.rotate(Axis.X, sign*ang).moved(Location((SPAR_X + 2, sign*(d.CENTER_HALF + L/2), zc + L/2*math.tan(math.radians(ang)))))
    return [inner, outer], zc

def _control_cut(y0, y1, sign):
    """Box removing everything aft of the hinge line between span stations y0..y1 (plus a 2 mm gap)."""
    yc = sign*(y0 + y1)/2
    c = d.chord_at(yc); xh = d.le_x(yc) + HINGE_FRAC*c - 1.0
    return Box(400, (y1 - y0) + 4, 200, align=(Align.MIN, Align.CENTER, Align.CENTER)).moved(
        Location((xh, yc, Z_LE_ROOT + d.dihedral_z(yc))))

def make():
    parts = []
    for s in (1, -1):
        inner, outer = _half(s)
        cuts, _ = _spar_cut(s)
        cuts += [_control_cut(*d.FLAP_SPAN, s), _control_cut(*d.AILERON_SPAN, s)]
        parts += [inner, outer]
        allcuts = locals().setdefault("allcuts", []); allcuts += cuts
    # panels share faces at y = 0 and y = +-CENTER_HALF: fuse before cutting so the solid is manifold.
    # Manufacturing split (centre section + 2 outer panels) is kept in the docs, not in this solid.
    w = parts[0].fuse(*parts[1:]).clean()
    return w.cut(*allcuts).clean()

def spar_z():
    return _spar_cut(1)[1]

def mass_g():
    """Wetted area x areal density. Closed-solid area includes small spar-tunnel and cut faces (~5 % over)."""
    return sum(p.area for p in make().solids())/1e6*AREAL_G_M2

def placements():
    return [Location((0, 0, 0))]

if __name__ == "__main__":
    from common import export, preview
    w = make()
    print(f"root {d.ROOT_CHORD:.0f}, tip {d.TIP_CHORD:.0f}, Z_LE_ROOT {Z_LE_ROOT:.1f}, spar centre z {spar_z():.1f}; mass {mass_g():.0f} g")
    export(w, NAME)
    preview(NAME, elev=25, azim=-125)
