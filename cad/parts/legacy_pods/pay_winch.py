"""Per-slot spool winch module (F2: one per slot, 4 per aircraft, ~150 g each [A]).

Local frame: origin at the slot centre on the bay DECK TOP (aircraft z = pay_bay_frame.DECK_Z_AC = -132);
the line drops through the deck at (0, 0) onto the pod's hook eye. The deck top is 45 mm under the battery,
so this is a flat "pancake" layout:
  - spool, axis vertical at x = -50: lower flange = worm wheel, driven by a worm on a 22 mm gearmotor
    lying along x. The worm is self-locking, so the pod is held with the motor unpowered.
  - level wind: a traveller fork rides two vertical rods on a reversing (diamond) screw. A belt from the
    spool hub (not modelled) turns the screw. It moves the line over the 18 mm drum height.
  - bellmouth fairlead post at (0, 0): the line comes in horizontally and leaves vertically down through
    the deck and the door's centre notch. The flare takes the changing lead angle as the drum radius
    changes from 15 to 36 mm. A nichrome line cutter sits at its exit for emergency jettison (12 V).
  - hook: a tension-latched J-hook with touchdown release. With the pod hanging, line tension pulls the
    gate (latch tongue) closed against a small spring. When the pod lands the line goes slack, the spring
    swings the gate open, and the winch hauls in so the hook slides off the pod's eye pin. "Command
    release" means paying out to slack on the ground; in flight the only release is the cutter.

Spool calc (25 m of 1.5 mm Dyneema):
  V_line  = 25 000 * pi/4 * 1.5^2           = 44 179 mm3
  V_wound = V_line / packing 0.75 (ASSUMED, level-wound braid 0.7-0.8)  = 58 905 mm3
  drum height W = 18 mm, core D0 = 30 mm  ->  D_full = sqrt(4 V/(pi W) + D0^2) = 71.2 mm
  flange 78 mm (3.4 mm freeboard).  Torque to hold the 1.78 kg pod at full radius: 17.5 N * 35.6 mm = 0.62 N m.
Units mm.
"""
import math
from build123d import *
import design as d

NAME = "pay_winch"
MATERIAL = "CF-PETG"
INFILL = 0.6
QTY = 4
MIRROR = False

LINE_LEN, LINE_D, PACKING = 25000.0, 1.5, 0.75        # PACKING (ASSUMED)
DRUM_W, CORE_D, FREEBOARD = 18.0, 30.0, 3.4            # (ASSUMED) sizing choices
V_LINE = LINE_LEN*math.pi/4*LINE_D**2
V_WOUND = V_LINE/PACKING
D_FULL = math.sqrt(4*V_WOUND/(math.pi*DRUM_W) + CORE_D**2)
FLANGE_D = round(D_FULL + 2*FREEBOARD)
SPOOL_X = -50.0
BASE_T = 2.4
WHEEL_Z, WHEEL_T = 10.0, 8.0                           # worm-wheel flange (bottom flange of the spool)
FLANGE_T = 2.0
MOTOR = dict(d=22.0, l=50.0, y=FLANGE_D/2 + 4.0, z=14.0, x0=-25.0)   # 22 mm gearmotor envelope (ASSUMED, no product chosen)
WORM_D = 10.0
FAIRLEAD_D, FAIRLEAD_H = 18.0, 36.0
HOOK_EYE_Z = -13.0      # pod eye-pin centre: pod top 2 mm under the deck underside (deck 3 mm), pin 8 mm deep
MASS = {"gearmotor 22 mm (ASSUMED)": 60, "worm + shaft": 8, "spool / worm wheel": 25, "base, level wind, fairlead": 30,
        "Dyneema 25 m x ~1.4 g/m (MEASURE)": 35, "hook + latch": 15, "nichrome cutter": 5}

def mass_g():
    return sum(MASS.values())


def _cyl(r, h, z0, x=0.0, y=0.0):
    return Pos(x, y, z0)*Cylinder(r, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def make_hook():
    body = Pos(0, 0, -6.5)*Box(10, 8, 7)                     # latch housing, z -10..-3 (spring inside)
    shank = Pos(5.5, 0, -14.0)*Box(3, 6, 10)                 # J shank, clear of the eye pin (x -3.7..2.3)
    lip = Pos(0.5, 0, -18.0)*Box(13, 6, 2)                   # lip under the pin
    gate = Pos(-6.0, 0, -13.5)*Rot(0, -15, 0)*Box(2, 5, 8)   # latch tongue (closed under tension)
    pivot = Pos(-6.0, 0, -9.5)*Rot(90, 0, 0)*Cylinder(1.5, 9)
    return body + shank + lip + gate + pivot


def make():
    zb = BASE_T
    base = Pos(-32.5, (MOTOR["y"] + MOTOR["d"]/2 + 3 - 45)/2, zb/2)*Box(125, 45 + MOTOR["y"] + MOTOR["d"]/2 + 3, zb)
    base -= _cyl(7.0, 10, -1)                                                  # line pass
    from pay_bay_frame import WINCH_HOLES
    for hx, hy in WINCH_HOLES:
        base -= _cyl(1.7, 10, -1, hx, hy)
    base -= _cyl(25.0, 10, -1, SPOOL_X, 0) - _cyl(12.0, 12, -2, SPOOL_X, 0)   # lightening ring under spool
    base += _cyl(12.0, WHEEL_Z - zb, zb, SPOOL_X, 0)                           # bearing boss
    spool = _cyl(FLANGE_D/2, WHEEL_T, WHEEL_Z, SPOOL_X)                        # worm wheel / lower flange
    spool += _cyl(D_FULL/2, DRUM_W, WHEEL_Z + WHEEL_T, SPOOL_X)                 # wound line (full)
    spool += _cyl(FLANGE_D/2, FLANGE_T, WHEEL_Z + WHEEL_T + DRUM_W, SPOOL_X)   # top flange
    spool += _cyl(3.0, 1.5, WHEEL_Z + WHEEL_T + DRUM_W + FLANGE_T, SPOOL_X)      # axle end
    m = MOTOR
    motor = Pos(m["x0"] + m["l"]/2, m["y"], m["z"])*Rot(0, 90, 0)*Cylinder(m["d"]/2, m["l"])
    worm_y = FLANGE_D/2 + WORM_D/2 - 1.0
    worm = Pos(SPOOL_X, worm_y, WHEEL_Z + WHEEL_T/2)*Rot(0, 90, 0)*Cylinder(WORM_D/2, 24)
    shaft = Pos((SPOOL_X + m["x0"])/2, worm_y, WHEEL_Z + WHEEL_T/2)*Rot(0, 90, 0)*Cylinder(2.0, m["x0"] - SPOOL_X + 2)
    # gearbox steps from the worm axis to the motor axis
    step = Pos(m["x0"] + 3, (worm_y + m["y"])/2, (WHEEL_Z + WHEEL_T/2 + m["z"])/2)*Box(6, abs(m["y"] - worm_y) + 12, 12)
    clamps = [Pos(m["x0"] + f*m["l"], m["y"], (zb + m["z"])/2)*Box(6, m["d"] + 4, m["z"] - zb) for f in (0.25, 0.8)]
    drive = motor + worm + shaft + step
    for c in clamps:
        drive += c
    # level wind: two rods, diamond screw, traveller fork with the line eye on y = 0
    lw_x = -12.0
    top = WHEEL_Z + WHEEL_T + DRUM_W + FLANGE_T
    lw = _cyl(1.5, top - zb, zb, lw_x, 10) + _cyl(1.5, top - zb, zb, lw_x, -10) + _cyl(2.5, top - zb, zb, lw_x, -16)
    lw += Pos(lw_x, -3, WHEEL_Z + WHEEL_T + DRUM_W/2)*Box(6, 30, 6) - Pos(lw_x, 0, WHEEL_Z + WHEEL_T + DRUM_W/2)*Box(10, 3, 3)
    lw += Pos(lw_x, -3, top)*Box(8, 34, 3)                               # top bridge
    # bellmouth fairlead post at (0,0): side entry slot facing -x, vertical bore, flare below the deck
    fl = _cyl(FAIRLEAD_D/2, FAIRLEAD_H - zb, zb)
    fl -= Pos(-FAIRLEAD_D/4, 0, WHEEL_Z + WHEEL_T + DRUM_W/2)*Box(FAIRLEAD_D/2 + 1, 4, DRUM_W + 6)
    fl -= _cyl(2.0, FAIRLEAD_H, 0)
    fl += _cyl(6.5, 3 + zb, -3) - _cyl(2.0, 10, -5)                            # spigot through deck hole
    cutter = Pos(7.5, 0, -1.5 + zb + 3)*Box(5, 10, 4)                          # nichrome cutter block on the post
    # line: spool -> fairlead (horizontal), fairlead -> hook
    lz = WHEEL_Z + WHEEL_T + DRUM_W/2
    line = Pos((SPOOL_X + D_FULL/2 + 0)/2, 0, lz)*Rot(0, 90, 0)*Cylinder(LINE_D/2, -(SPOOL_X + D_FULL/2) + 0.5)
    line += _cyl(LINE_D/2, lz - (-3.5), -3.5)
    parts = [base + spool + fl + line + cutter, drive, lw, make_hook()]
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def fit_checks():
    from pay_bay_frame import SX, SY, DECK_Z_AC
    bb = make().bounding_box()
    bat_bot = d.BATTERY_Z_TOP - d.BATTERY["z"]
    ok = True
    def chk(c, m):
        nonlocal ok
        print(("  ok  " if c else "  FAIL ") + m); ok &= c
    print(f"[winch] spool: V_line {V_LINE:.0f} mm3, wound {V_WOUND:.0f} mm3, D_full {D_FULL:.1f} mm, flange {FLANGE_D} mm; "
          f"hold torque {d.POD['g']*9.81e-3*D_FULL/2/1000:.2f} N m; mass {mass_g()} g (F2 budget 150 g)")
    print(f"[winch] bbox {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f}, local z {bb.min.Z:.1f}..{bb.max.Z:.1f}")
    clr = bat_bot - (DECK_Z_AC + bb.max.Z)
    chk(clr >= 3.0, f"winch top z={DECK_Z_AC + bb.max.Z:.0f} under battery bottom z={bat_bot:.0f}: {clr:.1f} mm clearance")
    chk(bb.min.X >= -SX/2 and bb.max.X <= SX/2 and bb.min.Y >= -SY/2 and bb.max.Y <= SY/2,
        "module footprint inside its own slot footprint (no clash with neighbour winches)")
    chk(FLANGE_D >= D_FULL + 4, "flange freeboard >= 2 mm")
    pod_top = -3.0 - 2.0
    chk(bb.min.Z > pod_top - 16.0, f"hook (bottom {bb.min.Z:.1f}) stays inside the pod's 16 mm hook pocket (pod top {pod_top})")
    return ok


def placements():
    from pay_bay_frame import SLOT_CENTERS_AC, DECK_Z_AC
    return [Location((x, y, DECK_Z_AC)) for x, y in SLOT_CENTERS_AC]


if __name__ == "__main__":
    from common import export, preview, OUT
    fit_checks()
    export(make(), NAME)
    preview(NAME, elev=20, azim=30, out=str(OUT / f"{NAME}_b.png"))
