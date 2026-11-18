"""FieldPod (ag kit): soil-sensor node, <= 500 g (F2). It uses the same slot, rails, pogo target and hook as the
RidgePod, so it is lowered by the same winches.

Local frame matches pay_ridgepod: origin at the bottom-face centre of the POD envelope (z 0..110), so it
hangs at the same height. Body: a 100 mm ASA cylinder at z 40..110. Four diagonal guide fins reach the
envelope corners, which is where the slot's corner rails run, and a radial arm carries the pogo pad target
at (+60, 0). Inside: 2x 21700 lying side by side, plus the LoRaWAN board and BMS. Top: solar cap with the
hook pocket and eye pin in its centre.
Soil probe spike (soil moisture/temperature): rides on the body axis in a guide tube. Stowed, its tip
sits 38 mm below the body, still inside the envelope. It telescopes out a further 60 mm on deployment. OPEN ISSUE: 500 g of pod weight (~5 N) will not push a
spike into firm soil. Insertion needs a drop from height or a screw tip; resolve after the N7
grower interviews.
Units mm.
"""
import math
from build123d import *
import design as d

NAME = "pay_fieldpod"
MATERIAL = "ASA"
INFILL = 1.0
QTY = 4                   # ag sortie: 4 pods (F2)
MIRROR = False

L, W, H = d.POD["x"], d.POD["y"], d.POD["z"]
BODY_D, BODY_Z0 = 100.0, 40.0          # (ASSUMED)
WALL = 1.6
FIN_T, FIN_H = 2.0, 30.0               # guide fins in the top 30 mm (ASSUMED)
CAP_T = 4.0                            # solar cap: panel + cover (ASSUMED)
SPIKE_D = 10.0                         # probe spike (ASSUMED; no probe product chosen)
SPIKE_EXT = 60.0                       # telescoped extension when deployed (ASSUMED)
CELL_D, CELL_L = 21.0, 70.0
POCKET = (30.0, 20.0, 16.0)            # same as RidgePod
EYE_PIN_D = 6.0
PAD = (24.0, 12.0, 0.6)
MASS = {"shell + fins": 75, "2x 21700 P45B": 140, "BMS + charger": 10, "LoRaWAN board": 20, "solar cap": 35,
        "probe spike + sensor electronics": 70, "hook fitting": 20}   # all (ASSUMED)

def mass_g():
    return sum(MASS.values())


def _cyl(r, h, z0, x=0.0, y=0.0):
    return Pos(x, y, z0)*Cylinder(r, h, align=(Align.CENTER, Align.CENTER, Align.MIN))


def make(deployed=False):
    hb = H - BODY_Z0 - CAP_T
    body = _cyl(BODY_D/2, hb, BODY_Z0) - _cyl(BODY_D/2 - WALL, hb - WALL, BODY_Z0 + WALL)
    # guide fins to the four envelope corners (inset 6 mm for the RidgePod's corner fillet)
    ang = math.atan2(W/2, L/2)
    reach = math.hypot(L/2, W/2) - 6.0
    for sx in (-1, 1):
        for sy in (-1, 1):
            a = math.atan2(sy*W/2, sx*L/2)
            r0 = BODY_D/2 - 2
            fin = Pos((r0 + reach)/2*math.cos(a), (r0 + reach)/2*math.sin(a), H - CAP_T - FIN_H/2)*Rot(0, 0, math.degrees(a))*Box(reach - r0, FIN_T, FIN_H)
            body += fin
            body += Pos(reach*math.cos(a), reach*math.sin(a), H - CAP_T - FIN_H/2)*Cylinder(3.0, FIN_H)   # rail pad
    # pad arm to the pogo target at (+60, 0)
    body += Pos(BODY_D/2 + 8, 0, H - CAP_T - 3)*Box(36, PAD[1] + 6, 6)
    # solar cap with hook pocket + eye pin
    cap = _cyl(BODY_D/2, CAP_T, H - CAP_T)
    cap += _cyl(BODY_D/2 - WALL, 1.0, H - CAP_T - 1.0)          # bond lip
    px, py, pd = POCKET
    cup = Pos(0, 0, H - pd/2)*Box(px + 2*WALL, py + 2*WALL, pd)
    body += cup
    body -= Pos(0, 0, H - pd/2 + 0.5)*Box(px, py, pd + 1)
    cap -= Pos(0, 0, H - pd/2 + 0.5)*Box(px, py, pd + 1)
    cap -= Pos(0, 0, H - 0.5)*Box(74, 54, 1.0) - Pos(0, 0, H - 0.5)*Box(px + 2*WALL, py + 2*WALL, 2)   # cell recess
    body += Pos(0, 0, H - 8)*Rot(90, 0, 0)*Cylinder(EYE_PIN_D/2, py + 2)
    body -= Pos(60, 0, H - PAD[2]/2)*Box(PAD[0], PAD[1], PAD[2] + 0.01)
    # contents
    cells = [_cyl(CELL_D/2, CELL_L, 0) for _ in range(2)]
    zc = BODY_Z0 + WALL + CELL_D/2 + 0.5
    inside = [Pos(0, sy*(SPIKE_D/2 + WALL + CELL_D/2 + 0.5), zc)*Rot(0, 90, 0)*Pos(0, 0, -CELL_L/2)*c for sy, c in zip((-1, 1), cells)]   # either side of the spike tube
    inside.append(Pos(31, 0, zc + CELL_D/2 + 4)*Box(34, 30, 6))        # LoRaWAN board envelope (ASSUMED)
    inside.append(Pos(31, 0, zc + CELL_D/2 + 10)*Box(26, 20, 4))         # BMS
    # probe spike on the body axis in a guide tube; stowed tip at z = 2 (inside the envelope),
    # deployed = lower section telescoped out SPIKE_EXT below (drive mechanism OPEN, see docstring)
    tube_top = H - POCKET[2] - 6.0
    body += _cyl(SPIKE_D/2 + WALL, tube_top - BODY_Z0, BODY_Z0)
    tip_z = 2.0 - (SPIKE_EXT if deployed else 0.0)
    spike = _cyl(SPIKE_D/2, tube_top - tip_z - 12, tip_z + 12)
    spike += Pos(0, 0, tip_z)*Cone(1.0, SPIKE_D/2, 12, align=(Align.CENTER, Align.CENTER, Align.MIN))
    hinge = _cyl(SPIKE_D/2 + 3, 4, BODY_Z0 - 4)                       # tube nose / wiper
    out = body + cap + hinge + spike
    for p in inside:
        out = out + p
    return out


def fit_checks():
    from pay_bay_frame import SX, SY, SZ
    bb = make().bounding_box()
    ok = True
    def chk(c, m):
        nonlocal ok
        print(("  ok  " if c else "  FAIL ") + m); ok &= c
    print(f"[fieldpod] stowed bbox {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f}, z {bb.min.Z:.1f}..{bb.max.Z:.1f}")
    chk(bb.min.X >= -L/2 and bb.max.X <= L/2 and bb.min.Y >= -W/2 and bb.max.Y <= W/2 and bb.min.Z >= -1e-3 and bb.max.Z <= H + 1e-3,
        "stowed FieldPod (incl. spike) inside POD envelope")
    chk(mass_g() <= 500, f"mass {mass_g()} g <= 500 g")
    return ok


def placements():
    return []          # ag kit not in the default (fire) assembly; same slots/height as pay_ridgepod.placements()


if __name__ == "__main__":
    from common import export, preview, OUT
    fit_checks()
    export(make(), NAME)
    preview(NAME, elev=-25, azim=-50, out=str(OUT / f"{NAME}_below.png"))
    export(make(deployed=True), NAME + "_deployed")
