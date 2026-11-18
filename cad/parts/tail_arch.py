"""SR-2 arch (Lambda) tail: two NACA0010 panels from each boom end up to an apex over the centreline.
Chord 200, apex 550 mm above the boom axis, incidence -3.4 deg [aero.py]. It closes the twin booms into a
frame (stiffer in torsion than two free fins), and its leading edge houses the half-metre VHF scanner antenna.
Ruddervators: aft 30 %, 2 mm gap, deflected together for pitch and differentially for yaw. Units mm."""
import math
import numpy as np
import aerosandbox as asb
from build123d import *
import design as d

NAME = "tail_arch"
MATERIAL = "CF"
INFILL = 1.0
QTY = 1
MIRROR = False
AREAL_G_M2 = 600.0                      # [A] molded sandwich, thin section
AF = np.asarray(asb.Airfoil(d.ARCH_AIRFOIL).repanel(n_points_per_side=40).coordinates)

def _sec(xle, y, z, chord, tilt, sign):
    """Airfoil in the panel's own plane: thickness normal to the panel, which leans in by tilt."""
    t = math.radians(d.ARCH_INCIDENCE_DEG)
    pts = []
    for a, b in AF*chord:
        x = a*math.cos(t) + b*math.sin(t); n = -a*math.sin(t) + b*math.cos(t)
        pts.append(Vector(xle + x, y + sign*n*math.sin(tilt), z + n*math.cos(tilt)))
    return Wire.make_polygon(pts, close=True)

def panels(cut_controls=True):
    tilt = math.atan2(d.ARCH_APEX_H, d.BOOM_Y)
    out = []
    for s in (1, -1):
        w0 = _sec(d.TAIL_LE_X, s*d.BOOM_Y, d.BOOM_Z, d.ARCH_CHORD, tilt, s)
        w1 = _sec(d.TAIL_LE_X + 10, -s*15.0, d.BOOM_Z + d.ARCH_APEX_H - 15*math.tan(tilt), d.ARCH_CHORD*0.92, tilt, s)   # crosses the centreline: robust overlap
        out.append(Solid.make_loft([w0, w1], ruled=True))
    return out

def make():
    return make_split()[0]

def make_split():
    p = panels()
    apex = Cylinder(30, 190, rotation=(0, 90, 0)).moved(Location((d.TAIL_LE_X + 85, 0, d.BOOM_Z + d.ARCH_APEX_H)))   # apex fairing (houses the VHF antenna feed); a sphere here left a mesh seam
    sockets = [Cylinder(26, 60, rotation=(0, 90, 0)).moved(Location((d.TAIL_LE_X + 80, s*d.BOOM_Y, d.BOOM_Z))) -
               Cylinder(d.BOOM_TUBE[0]/2 + 0.2, 80, rotation=(0, 90, 0)).moved(Location((d.TAIL_LE_X + 80, s*d.BOOM_Y, d.BOOM_Z)))
               for s in (1, -1)]
    body = p[0].fuse(p[1], apex, *sockets).clean()
    hinge_x = d.TAIL_LE_X + (1 - d.RUDDERVATOR_CHORD_FRAC)*d.ARCH_CHORD
    rv = Box(200, 2*d.BOOM_Y - 160, 2*d.ARCH_APEX_H, align=(Align.MIN, Align.CENTER, Align.CENTER)).moved(
        Location((hinge_x, 0, d.BOOM_Z + d.ARCH_APEX_H/2)))
    return body - rv, (p[0].fuse(p[1]) & rv.moved(Location((2, 0, 0))))

def make_ruddervators():
    return make_split()[1]

def mass_g():
    fixed, ctrl = make_split()
    return (fixed.area + ctrl.area)/1e6*AREAL_G_M2/1.0 + 2*40   # + 2 servos counted in avionics; 40 g sockets [A]

def placements():
    return [Location((0, 0, 0))]

if __name__ == "__main__":
    from common import export, preview
    fixed, ctrl = make_split()
    print(f"panel tilt {math.degrees(math.atan2(d.ARCH_APEX_H, d.BOOM_Y)):.1f} deg, mass {mass_g():.0f} g")
    export(fixed, NAME)
    export(ctrl, "tail_ruddervators")
    preview(NAME, elev=20, azim=-130)
