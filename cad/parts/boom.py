"""SR-3 boom assembly (one side; MIRROR=True): CF tube, two single lift-motor mounts with
streamlined nacelles, two sprung landing legs, two wing clamps.

Leg stations: with coax props the lower prop spins under each motor, so legs can't sit at the motor
positions (as on SR-1). They sit on the boom just outside both lower prop discs, fore and aft of the CG:
  front leg x = -120 (402 mm from the front motor axis, disc radius 355.5)
  rear  leg x = +420 (380 mm from the rear motor axis)
Units mm.
"""
import math
import numpy as np
import aerosandbox as asb
from build123d import *
import design as d

NAME = "boom"
MATERIAL = "CF"
INFILL = 1.0
QTY = 2
MIRROR = True
LEG_X = (d.FRONT_MOTOR_X, d.REAR_MOTOR_X)    # SR-3: single motor on top, nothing spins below -> legs under the motors
CLAMP_X = (105.0, 190.0)           # wing spar line and the rear rod line (~65 % chord at the boom)
NACELLE = dict(len=230.0, w=70.0)                 # [A] fairs the pylon between the motor seats; motors stay exposed (cooling, props)
MOUNT_Z = (d.BOOM_Z - 30.0, d.COAX_TOP_PROP_Z - 46.0)   # pylon bottom (leg socket) and top motor seat [A]
LEG_CHORD, LEG_T = 55.0, 14.0      # streamlined strut (NACA 0025-ish), two 6 mm CF rods inside [A]
FOOT_STROKE = 60.0                 # [E_STRUCTURES] >= 60 mm compliance -> n_land 2.9

def _strut(x, z0, z1, chord=LEG_CHORD):
    af = np.asarray(asb.Airfoil("naca0025").repanel(n_points_per_side=24).coordinates)*chord
    ws = [Wire.make_polygon([Vector(x - chord*0.4 + a, b, z) for a, b in af], close=True) for z in (z0, z1)]
    return Solid.make_loft(ws, ruled=True)

def make():
    """One boom in its own frame: tube axis along x at y = 0, z = 0 (placements() moves it)."""
    L = d.BOOM_REAR_X - d.BOOM_FRONT_X
    od, t = d.BOOM_TUBE
    tube = Cylinder(od/2, L, rotation=(0, 90, 0)).moved(Location((d.BOOM_FRONT_X + L/2, 0, 0))) - \
           Cylinder(od/2 - t, L + 2, rotation=(0, 90, 0)).moved(Location((d.BOOM_FRONT_X + L/2, 0, 0)))
    parts = [tube]
    zb = -d.BOOM_Z                                      # boom-frame z of the aircraft's z=0
    for mx in (d.FRONT_MOTOR_X, d.REAR_MOTOR_X):
        # pylon between the two motor seats, clamping the tube
        lo, hi = MOUNT_Z[0] - d.BOOM_Z, MOUNT_Z[1] - d.BOOM_Z
        pylon = Box(60, 46, hi - lo).moved(Location((mx, 0, (lo + hi)/2)))
        pylon = pylon - Cylinder(od/2 + 0.2, 80, rotation=(0, 90, 0)).moved(Location((mx, 0, 0)))
        seats = [Cylinder(40, 4).moved(Location((mx, 0, hi - 2)))]
        # nacelle: streamlined body of revolution-ish around the pylon (split shells in production)
        xs = np.linspace(-NACELLE["len"]/2, NACELLE["len"]/2, 7)
        prof = np.sin(np.pi*(xs - xs[0])/(xs[-1] - xs[0]))**0.6
        secs = []
        for xi, f in zip(xs, prof):
            a, b = max(NACELLE["w"]/2*f, 3), max((hi - lo - 8)/2*f, 3)
            t_ = np.linspace(0, 2*np.pi, 32, endpoint=False)
            pts = [Vector(mx + xi, a*math.cos(u), (lo + hi)/2 + b*math.sin(u)) for u in t_]
            secs.append(Wire([Spline(*pts, pts[0])]))
        nac = Solid.make_loft(secs)
        nac = nac - Box(64, 50, hi - lo + 2).moved(Location((mx, 0, (lo + hi)/2)))   # hollow for the pylon
        parts += [pylon, *seats, nac]
    for lx in LEG_X:
        z_top, z_foot = 0.0, d.GROUND_Z - d.BOOM_Z + FOOT_STROKE
        parts.append(_strut(lx, z_top, z_foot))
        parts.append(Cylinder(26, FOOT_STROKE).moved(Location((lx, 0, z_foot - FOOT_STROKE/2))))   # TPU/sprung foot
        parts.append(Box(70, 44, 30).moved(Location((lx, 0, 0))) - Cylinder(od/2 + 0.2, 80, rotation=(0, 90, 0)).moved(Location((lx, 0, 0))))
    for cx in CLAMP_X:
        # saddle from the boom top up to the wing lower surface at the boom station (z = 0 there: flat centre section)
        h = zb - od/2
        sad = Box(60, 50, h + od/2).moved(Location((cx, 0, (h + od/2)/2)))
        parts.append(sad - Cylinder(od/2 + 0.2, 80, rotation=(0, 90, 0)).moved(Location((cx, 0, 0))))
    return parts[0].fuse(*parts[1:]).clean()

def mass_g():
    od, t = d.BOOM_TUBE
    tube = math.pi*(od**2 - (od - 2*t)**2)/4*(d.BOOM_REAR_X - d.BOOM_FRONT_X)/1e3*1.55   # mm3 -> g at 1.55 g/cm3
    return tube + 2*60 + 2*50 + 2*(110 + 30) + 2*50            # mounts, nacelles, legs+feet, clamps [A] (lighter than SR-2)

def placements():
    return [Location((0, d.BOOM_Y, d.BOOM_Z))]

if __name__ == "__main__":
    from common import export, preview
    b = make(); print(f"mass {mass_g():.0f} g per side; legs under the motors (no lower props on SR-3)")
    export(b, NAME)
    preview(NAME, elev=15, azim=-70)
