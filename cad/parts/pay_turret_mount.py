"""Chin turret mount at design.TURRET with the common kit adapter plate (F2 "Turret position").

Local frame: origin at the turret ball centre (TURRET x, 0, TURRET z = -320), z up.
Stack, top down:
  - mount cradle (CF-PETG): a 3 mm deck at local z 81..84 with two saddle walls that follow the fuselage
    inner skin and are bonded to it; a 100 mm cable/through hole.
  - 4 elastomer isolators, D 10 x 8 mm (ASSUMED; MEASURE the stiffness for the turret's gimbal mode).
  - common adapter plate (CF-PETG, 3 mm, local 70..73): kit side 4x M3 on a 60x60 square (ASSUMED);
    cradle side 4x M3 isolator studs at x -30/+70, y +/-50 (ASSUMED). Plate and cradle are biased aft
    because the nose skin rises forward of x = -470.
  - payload, hung below through the fuselage's D124 chin opening (fuse_shell.py):
      fire: Trillium HD25-LV CLASS ENVELOPE. Ball D110 (design.TURRET), plus a pan base D90 x 18 that is
            ASSUMED. The real mounting interface and connector exit are not available; get the drawing (MEASURE).
      ag:   MicaSense RedEdge-P CLASS ENVELOPE, 90 x 63 x 54 mm (from the task brief), lens face down. The
            real mount hole pattern is not available (MEASURE).
At x = -470 the fuselage is still in its nose taper, so the saddle conforms to a section interpolated from
the fuse_shell stations. Bond with thickened epoxy and check the fit on the real part (MEASURE).
Units mm.
"""
import math
import numpy as np
from build123d import *
import design as d

NAME = "pay_turret_mount"
MATERIAL = "CF-PETG"
INFILL = 0.5
QTY = 1
MIRROR = False
KIT = "fire"              # "fire" | "ag": which payload make_payload() returns

X0, Z0 = d.TURRET["x"], d.TURRET["z"]
BALL_D = d.TURRET["ball_d"]
FIRE_BASE = (90.0, 18.0)                # pan-base envelope D x h (ASSUMED)
FIRE_G, AG_G = 350, 363                 # F2 / components.py
AG_BOX = (90.0, 63.0, 54.0)             # RedEdge-P class envelope from the brief
ADAPT = dict(x0=-40.0, x1=80.0, w=120.0, t=3.0, z=BALL_D/2 + FIRE_BASE[1] - 3.0)   # plate bottom local z 70; biased aft: the nose skin is lower forward
ISO = dict(d=10.0, h=8.0, xs=(-30.0, 70.0), ys=(-50.0, 50.0))      # isolator studs (ASSUMED)
CRADLE = dict(t=3.0, x0=-30.0, x1=90.0, wall_h=45.0, hole=100.0)
KIT_SQ = 60.0
SKIN_T = 1.5


def _section(x=None):
    """(half-width a, half-height b, centre z, n) of the fuselage OUTER section at aircraft x (default X0)."""
    x = X0 if x is None else x
    try:
        from fuse_shell import STATIONS
        xs = [s[0] for s in STATIONS]
        w = np.interp(x, xs, [s[1] for s in STATIONS]); h = np.interp(x, xs, [s[2] for s in STATIONS])
        zt = np.interp(x, xs, [s[3] for s in STATIONS])
    except Exception:                                 # fallback: full mid-body section (ASSUMED)
        w, h, zt = d.FUSE_W, d.FUSE_H, 0.0
    return w/2, h/2, zt - h/2, d.fuse_n(x)


def inner_half_width(z_ac, x=None):
    a, b, zc, n = _section(x)
    a, b = a - SKIN_T, b - SKIN_T
    r = abs(z_ac - zc)/b
    return 0.0 if r >= 1 else a*(1 - r**n)**(1/n)


def belly_z(x=None):
    a, b, zc, n = _section(x)
    return zc - b


def _saddle(z_lo, z_hi, side):
    """Wall following the inner skin from z_lo..z_hi (local), thickness CRADLE.t, extruded along x."""
    zs = np.linspace(z_lo, z_hi, 16)
    xf = X0 + CRADLE["x0"]                     # conform to the narrowest (forward) end; aft gap = bond fillet
    outer = [(side*(inner_half_width(Z0 + z, xf) - 1.0), z) for z in zs]
    inner = [(y - side*CRADLE["t"], z) for y, z in outer[::-1]]
    pts = [(float(y), float(z)) for y, z in outer + inner]
    face = Plane.YZ*Polygon(*pts, align=None)
    L = CRADLE["x1"] - CRADLE["x0"]
    return Pos((CRADLE["x0"] + CRADLE["x1"])/2, 0, 0)*extrude(face, amount=L/2, both=True)


def make():
    zt = ADAPT["z"] + ADAPT["t"] + ISO["h"]                 # cradle deck bottom (local)
    hw = inner_half_width(Z0 + zt + CRADLE["t"], X0 + CRADLE["x0"]) - 1.5
    L, xm = CRADLE["x1"] - CRADLE["x0"], (CRADLE["x0"] + CRADLE["x1"])/2
    deck = Pos(xm, 0, zt + CRADLE["t"]/2)*Box(L, 2*hw, CRADLE["t"])
    deck -= Pos(0, 0, zt)*Cylinder(CRADLE["hole"]/2, 20)
    for ix in ISO["xs"]:
        for iy in ISO["ys"]:
            deck -= Pos(ix, iy, zt)*Cylinder(1.7, 20)
    walls = _saddle(zt, zt + CRADLE["wall_h"], 1) + _saddle(zt, zt + CRADLE["wall_h"], -1)
    # gussets deck -> walls
    for x in (CRADLE["x0"] + 1.5, xm, CRADLE["x1"] - 1.5):
        for s in (-1, 1):
            g = Pos(x, s*(hw - 12), zt + CRADLE["t"] + 10)*Box(3, 24, 20)
            walls += g
    return deck + walls


def make_adapter():
    z = ADAPT["z"]
    p = Pos((ADAPT["x0"] + ADAPT["x1"])/2, 0, z + ADAPT["t"]/2)*Box(ADAPT["x1"] - ADAPT["x0"], ADAPT["w"], ADAPT["t"])
    p -= Pos(0, 0, z)*Cylinder(20.0, 20)                                      # cable pass
    for sx in (-1, 1):
        for sy in (-1, 1):
            p -= Pos(sx*KIT_SQ/2, sy*KIT_SQ/2, z)*Cylinder(1.7, 20)
    isos = [Pos(ix, iy, z + ADAPT["t"])*Cylinder(ISO["d"]/2, ISO["h"], align=(Align.CENTER, Align.CENTER, Align.MIN))
            for ix in ISO["xs"] for iy in ISO["ys"]]
    for i in isos:
        p += i
    return p


def make_fire_turret():
    # poles trimmed 0.5 mm: OCC's sphere poles tessellate non-manifold
    ball = Sphere(BALL_D/2) - Pos(0, 0, BALL_D/2 + 9.5)*Box(200, 200, 20) - Pos(0, 0, -BALL_D/2 - 9.5)*Box(200, 200, 20)
    ball = ball - Pos(BALL_D/2 - 4, 0, 0)*Rot(0, 90, 0)*Cylinder(22, 10)    # window recess (illustrative)
    base = Pos(0, 0, ADAPT["z"] - FIRE_BASE[1])*Cylinder(FIRE_BASE[0]/2, FIRE_BASE[1], align=(Align.CENTER, Align.CENTER, Align.MIN))
    return ball + base + make_adapter()


def make_ag_camera():
    x, y, h = AG_BOX
    cam = Pos(0, 0, ADAPT["z"] - h/2)*Box(x, y, h)
    cam -= Pos(0, 0, ADAPT["z"] - h)*Box(x - 16, y - 16, 4)             # lens-face recess (illustrative, not lens layout)
    return cam + make_adapter()


def make_payload():
    return make_fire_turret() if KIT == "fire" else make_ag_camera()


def fit_checks():
    zb = belly_z()
    ok = True
    def chk(c, m):
        nonlocal ok
        print(("  ok  " if c else "  FAIL ") + m); ok &= c
    a, b, zc, n = _section()
    print(f"[turret] section at x={X0:.0f}: {2*a:.0f} x {2*b:.0f}, belly z={zb:.1f}, n={n:.2f}")
    chk(Z0 - BALL_D/2 - d.GROUND_Z > 0, f"ball bottom z={Z0 - BALL_D/2:.0f} clears ground z={d.GROUND_Z:.0f} by {Z0 - BALL_D/2 - d.GROUND_Z:.0f} mm")
    chk(Z0 + ADAPT["z"] > zb, f"adapter plate (z={Z0 + ADAPT['z']:.0f}) above the belly skin (z={zb:.0f})")
    chk(FIRE_BASE[0] < 124 and math.hypot(AG_BOX[0], AG_BOX[1]) < 124, "fire base D90 and ag box diagonal pass the D124 chin opening")
    xf = X0 + ADAPT["x0"]
    chk(Z0 + ADAPT["z"] > belly_z(xf) + 2 and ADAPT["w"]/2 < inner_half_width(Z0 + ADAPT["z"], xf),
        f"adapter plate forward edge (x={xf:.0f}) inside skin: belly there z={belly_z(xf):.0f}, half-width {inner_half_width(Z0 + ADAPT['z'], xf):.0f}")
    try:
        import fuse_shell
        fs = fuse_shell.make()
        for nm, part in (("mount", make()), ("fire payload", make_fire_turret()), ("ag payload", make_ag_camera())):
            v = fs & part.moved(placements()[0])
            vol = v.volume if v is not None else 0.0
            chk(vol < 1.0, f"{nm} vs fuse_shell solid: intersection {vol:.0f} mm3")
    except ImportError:
        print("  (fuse_shell not available: skin interference not checked)")
    m = make()
    print(f"[turret] mount {m.volume/1000:.0f} cm3 -> ~{m.volume/1000*1.3*INFILL:.0f} g; adapter+isolators ~{make_adapter().volume/1000*1.3:.0f} g")
    return ok


def placements():
    return [Location((X0, 0, Z0))]


if __name__ == "__main__":
    from common import export, preview, OUT
    fit_checks()
    export(make(), NAME)
    export(make_fire_turret(), "pay_turret_fire")
    export(make_ag_camera(), "pay_turret_ag")
    preview("pay_turret_fire", elev=10, azim=-60)
