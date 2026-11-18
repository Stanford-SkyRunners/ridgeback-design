"""RidgePod (fire kit): ground VHF repeater lowered onto a ridgetop by the slot winch (F2, decision D3 revised).

Local frame: origin at the pod's bottom-face centre, z up; x along the aircraft x when stowed.
Stack (bottom -> top): Telewave TMND3 duplexer block (175x114x35, 1000 g) on the floor for a low CG;
3x 21700 (3S P45B) + BMS on the -x side and the radio board on the +x side above it; controller/GNSS/mesh
on top of the radio. Top face: hook eye (a pin across a 16 mm pocket) on the computed CG, a 4-pad target
for the bay pogo connector at (+60, 0), and a racetrack groove holding the whip.

Whip: a 450 mm quarter-wave made from steel tape-spring (measuring-tape section, 12.7 x 0.15 mm (ASSUMED)).
Stowed one turn round a racetrack groove (perimeter = 450 mm) and self-erects when the groove clip opens.
A round spring-steel wire can't be coiled this tight: strain d/D = 1.0/90 = 1.1 % > ~0.8 % yield strain of
music wire. The tape's strain is t/D = 0.15/90 = 0.17 %, so it can.
Legs: three 5-section telescoping conductive tubes (radio-antenna style, 8 mm max OD (ASSUMED)), 110 mm
stowed and 450 mm extended. They double as the quarter-wave radials, so they hinge at the TOP edge, at the
whip feed height, on skewed spring hinges. Each lies horizontally in a 10x10 channel along the top edge of a
side wall (two on the long sides, one on the -x end) and swings out to azimuths +60, -60 and 180 deg,
drooping 30 deg below horizontal. That droop is also the classic angle that brings a ground-plane antenna
to ~50 ohm. The pod then hangs on a tripod with the duplexer below the hinge plane (pendulum-stable).
Shell: ASA, 1.6 mm walls (SKILL.md >= 1.6). make() = stowed pod with contents (for the assembly);
make_shell() = printable shell; make_deployed() = render.
"""
import math
from build123d import *
import design as d

NAME = "pay_ridgepod"
MATERIAL = "ASA"
INFILL = 1.0              # shell is all perimeters at 2 mm; real mass comes from mass_g()
QTY = 3
MIRROR = False

L, W, H = d.POD["x"], d.POD["y"], d.POD["z"]
WALL = 1.6                # SKILL.md minimum (4 perimeters); 2.0 would cost +55 g
CORNER_R = 8.0            # (ASSUMED) vertical edge fillet
DUP = d.POD_DUPLEXER
CELL_D, CELL_L = 21.0, 70.0                     # 21700 envelope
BATT = dict(x=-40.0, size=(3*CELL_D + 4, CELL_L + 8, CELL_D + 3))   # 3 cells side by side + BMS strip (ASSUMED layout)
RADIO = dict(x=45.0, size=(100.0, 60.0, 20.0))  # Part-90 5 W VHF repeater board + heat spreader: envelope (MEASURE, no product chosen)
CTRL = dict(x=60.0, size=(60.0, 40.0, 10.0))    # controller + GNSS + mesh + LED: envelope (ASSUMED)
LEG_OD, LEG_STOW, LEG_SECTIONS = 8.0, 110.0, 5  # (ASSUMED) telescoping radial/leg
LEG_CH = 10.0                                   # leg channel square
LEG_Z = H - 13.0                                # leg axis height (stowed) = hinge height
LEG_DROOP = 30.0
WHIP_W, WHIP_T = 12.7, 0.15                     # tape-spring section (ASSUMED)
WHIP_D = 90.0                                   # racetrack end diameter (centreline)
WHIP_STRAIGHT = (d.POD_WHIP_LEN - math.pi*WHIP_D)/2     # 83.6 mm so the centreline perimeter = 450 mm
GROOVE_W, GROOVE_DEPTH = 16.0, 4.0
POCKET = (30.0, 20.0, 16.0)                     # hook pocket (x, y, depth); eye pin dia 6 at 8 mm depth
EYE_PIN_D = 6.0
PAD = (24.0, 12.0, 0.6)                         # pogo target pads at (+60, 0)

# itemised F2 mass list (g)
MASS = {
    "duplexer Telewave TMND3": 1000, "VHF repeater radio board": 150, "3x P45B 3S + BMS": 230,
    "controller, GNSS, mesh radio, LED": 40, "45 cm tape-spring whip": 40, "3 legs / radials": 120,
    "shell": 180, "hook fitting": 20,
}

def mass_g():
    return sum(MASS.values())


# stowed leg geometry: (hinge point, unit direction stowed, deployed azimuth deg)
LEGS = [
    ((L/2 - 12, W/2 - LEG_CH/2, LEG_Z), (-1, 0, 0), 60.0),
    ((L/2 - 12, -(W/2 - LEG_CH/2), LEG_Z), (-1, 0, 0), -60.0),
    ((-(L/2 - LEG_CH/2), -(W/2 - 10), LEG_Z), (0, 1, 0), 180.0),
]


def _items():
    """(name, mass, centre (x,y,z), size) of the internal blocks, for geometry + CG."""
    zb = WALL
    bx, by, bz = BATT["size"]; rx, ry, rz = RADIO["size"]; cx, cy, cz = CTRL["size"]
    top = zb + DUP["z"]
    return [
        ("duplexer", MASS["duplexer Telewave TMND3"], (0, 0, zb + DUP["z"]/2), (DUP["x"], DUP["y"], DUP["z"])),
        ("battery", MASS["3x P45B 3S + BMS"], (BATT["x"], 0, top + bz/2), (bx, by, bz)),
        ("radio", MASS["VHF repeater radio board"], (RADIO["x"], 0, top + rz/2), (rx, ry, rz)),
        ("ctrl", MASS["controller, GNSS, mesh radio, LED"], (CTRL["x"], 0, top + rz + cz/2), (cx, cy, cz)),
    ]


def cg():
    pts = [(m, c) for _, m, c, _ in _items()]
    pts += [(MASS["shell"], (0, 0, H*0.5)), (MASS["hook fitting"], (0, 0, H - 8)),
            (MASS["45 cm tape-spring whip"], (0, 0, H - 2))]                 # coiled loop is centred
    for (hp, u, _) in LEGS:
        pts.append((MASS["3 legs / radials"]/3, tuple(hp[i] + u[i]*LEG_STOW/2 for i in range(3))))
    m = sum(p[0] for p in pts)
    return tuple(sum(p[0]*p[1][i] for p in pts)/m for i in range(3))


CG = cg()
HOOK_XY = (round(CG[0], 1), round(CG[1], 1))


def _racetrack(sep, h, depth, z0):
    return Pos(0, 0, z0)*extrude(SlotCenterToCenter(sep, h), amount=depth)


def _channels(grow=0.0):
    g = grow
    out = []
    for (hp, u, _) in LEGS:
        if u[0]:
            x0, x1 = hp[0] - LEG_STOW - 6, hp[0] + 6
            yc = math.copysign(W/2 - LEG_CH/2 + 1, hp[1])
            out.append(Pos((x0 + x1)/2, yc - math.copysign(g/2, hp[1]), LEG_Z)*Box(x1 - x0 + 2*g, LEG_CH + 2 + g, LEG_CH + 2*g))
        else:
            y0, y1 = hp[1] - 6, hp[1] + LEG_STOW + 6
            out.append(Pos(-L/2 + LEG_CH/2 - 1 + g/2, (y0 + y1)/2, LEG_Z)*Box(LEG_CH + 2 + g, y1 - y0 + 2*g, LEG_CH + 2*g))
    return out


def _groove(grow=0.0):
    return _racetrack(WHIP_STRAIGHT, WHIP_D + GROOVE_W + 2*grow, GROOVE_DEPTH + grow + 1, H - GROOVE_DEPTH - grow) - \
        _racetrack(WHIP_STRAIGHT, WHIP_D - GROOVE_W - 2*grow, GROOVE_DEPTH + grow + 2, H - GROOVE_DEPTH - grow - 0.5)


def _pocket(grow=0.0):
    px, py, pd = POCKET
    return Pos(HOOK_XY[0], HOOK_XY[1], H - (pd + grow)/2 + 0.5)*Box(px + 2*grow, py + 2*grow, pd + grow + 1)


def make_shell():
    outer = fillet(Box(L, W, H).edges().filter_by(Axis.Z), CORNER_R)
    outer = Pos(0, 0, H/2)*outer
    inner = Pos(0, 0, H/2)*fillet(Box(L - 2*WALL, W - 2*WALL, H - 2*WALL).edges().filter_by(Axis.Z), CORNER_R - WALL)
    for c in _channels():
        outer -= c
    outer -= _groove()
    outer -= _pocket()
    for c in _channels(WALL):
        inner -= c
    inner -= _groove(WALL)
    inner -= _pocket(WALL)
    shell = outer - inner
    # hook eye pin across the pocket (along y) and its bosses
    shell += Pos(HOOK_XY[0], HOOK_XY[1], H - 8)*Rot(90, 0, 0)*Cylinder(EYE_PIN_D/2, POCKET[1] + 2)
    shell -= Pos(60, 0, H - PAD[2]/2)*Box(PAD[0], PAD[1], PAD[2] + 0.01)          # pad recess
    # whip base (feed + clip) at the +x apex of the racetrack
    shell += Pos(WHIP_STRAIGHT/2 + WHIP_D/2, 0, H - GROOVE_DEPTH)*Cylinder(7.0, 3.0, align=(Align.CENTER,)*2 + (Align.MIN,))
    return shell


def _leg_solid(hp, direction, length):
    dx, dy, dz = direction
    yaw = math.degrees(math.atan2(dy, dx))
    pitch = math.degrees(math.atan2(-dz, math.hypot(dx, dy)))
    # telescoping sections drawn stepping down in OD
    secs = []
    n = LEG_SECTIONS
    seg = length/n
    for i in range(n):
        r = LEG_OD/2 - i*0.7
        secs.append(Pos(seg*(i + 0.5), 0, 0)*Rot(0, 90, 0)*Cylinder(r, seg + (2 if i < n - 1 else 0)))
    leg = secs[0]
    for s in secs[1:]:
        leg += s
    leg += Cylinder(4.5, 9.0)                                                    # hinge barrel (axis shown vertical)
    return Pos(*hp)*Rot(0, 0, yaw)*Rot(0, pitch, 0)*leg


def _contents():
    out = []
    for name, _, c, s in _items():
        out.append(Pos(*c)*Box(*s))
    return out


def _whip_stowed():
    z0 = H - GROOVE_DEPTH
    return _racetrack(WHIP_STRAIGHT, WHIP_D + WHIP_W, 1.0, z0) - _racetrack(WHIP_STRAIGHT, WHIP_D - WHIP_W, 3.0, z0 - 1)


def _fuse(parts):
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


def make():
    parts = [make_shell(), _whip_stowed()] + _contents()
    for hp, u, _ in LEGS:
        parts.append(_leg_solid(hp, u, LEG_STOW))
    return _fuse(parts)


def make_deployed():
    parts = [make_shell()] + _contents()
    wx = WHIP_STRAIGHT/2 + WHIP_D/2
    parts.append(Pos(wx, 0, H - 1)*Cylinder(3.0, d.POD_WHIP_LEN, align=(Align.CENTER,)*2 + (Align.MIN,)))
    for hp, _, az in LEGS:
        a, e = math.radians(az), math.radians(LEG_DROOP)
        parts.append(_leg_solid(hp, (math.cos(a)*math.cos(e), math.sin(a)*math.cos(e), -math.sin(e)), d.POD_LEG_LEN))
    return _fuse(parts)


def deployed_checks():
    tips = []
    for hp, _, az in LEGS:
        a, e = math.radians(az), math.radians(LEG_DROOP)
        r = d.POD_LEG_LEN
        tips.append((hp[0] + r*math.cos(a)*math.cos(e), hp[1] + r*math.sin(a)*math.cos(e), hp[2] - r*math.sin(e)))
    (x1, y1, _), (x2, y2, _), (x3, y3, _) = tips
    def side(ax, ay, bx, by, px, py):
        return (bx - ax)*(py - ay) - (by - ay)*(px - ax)
    s = [side(x1, y1, x2, y2, CG[0], CG[1]), side(x2, y2, x3, y3, CG[0], CG[1]), side(x3, y3, x1, y1, CG[0], CG[1])]
    inside = all(v > 0 for v in s) or all(v < 0 for v in s)
    clearance = -tips[0][2]
    return tips, inside, clearance


def fit_checks():
    from pay_bay_frame import SX, SY, SZ, GUIDE_T_Y, GUIDE_T_X
    bb = make().bounding_box()
    ok = True
    def chk(cond, msg):
        nonlocal ok
        print(("  ok  " if cond else "  FAIL ") + msg); ok &= cond
    print(f"[ridgepod] stowed bbox {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f}  (envelope {L} x {W} x {H})")
    chk(bb.size.X <= L + 1e-6 and bb.size.Y <= W + 1e-6 and bb.size.Z <= H + 1e-6, "stowed pod (legs, whip incl.) inside POD envelope")
    chk(L + 2*GUIDE_T_X < SX and W + 2*GUIDE_T_Y <= SY and H < SZ, f"POD + rails inside SLOT {SX}x{SY}x{SZ}")
    chk(DUP["x"] <= L - 2*WALL and DUP["y"] <= W - 2*WALL, "duplexer fits inside the shell")
    chk(LEG_STOW + 12 + 6 <= L and LEG_STOW + 20 <= W, "legs fold within the pod sides")
    top = WALL + DUP["z"] + max(RADIO["size"][2] + CTRL["size"][2], BATT["size"][2])
    chk(top <= LEG_Z - LEG_CH/2 - WALL, f"internal stack top z={top:.0f} under the leg channels / groove")
    chk(math.hypot(*HOOK_XY) < 3.0, f"hook eye on CG: CG=({CG[0]:.1f},{CG[1]:.1f},{CG[2]:.1f}) hook at {HOOK_XY}")
    chk(abs(mass_g() - d.POD["g"]) < 1, f"mass_g() = {mass_g()} g vs design.POD {d.POD['g']} g")
    tips, inside, clr = deployed_checks()
    reach = math.hypot(tips[0][0] - LEGS[0][0][0], tips[0][1] - LEGS[0][0][1])
    chk(inside, f"deployed: CG inside the leg-tip triangle; horizontal reach {reach:.0f} mm, pod bottom {clr:.0f} mm above ground")
    sh = make_shell()
    print(f"[ridgepod] printed shell {sh.volume/1000:.0f} cm3 -> ~{sh.volume/1000*1.07:.0f} g ASA (budget {MASS['shell']} g)")
    return ok


def placements():
    from pay_bay_frame import SLOT_CENTERS_AC
    z = d.BAY_Z_FLOOR + 8.0            # hangs 8 mm above the closed doors; top 2 mm under the deck (pogo contact)
    # carry 3 of 4 (F2 D2): leave the aft-port slot empty (ASSUMED choice); release diagonally
    return [Location((x, y, z)) for (x, y) in SLOT_CENTERS_AC if not (x > d.BAY_CENTER_X and y < 0)]


if __name__ == "__main__":
    from common import export, preview, OUT
    fit_checks()
    export(make(), NAME)
    preview(NAME, elev=70, azim=-60, out=str(OUT / f"{NAME}_top.png"))
    export(make_shell(), NAME + "_shell")
    export(make_deployed(), NAME + "_deployed")
