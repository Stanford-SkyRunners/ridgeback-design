"""Small SR-3 parts: GNSS antenna pads (wing top, 800 mm baseline) and control horns (ailerons, ruddervators)."""
from build123d import *
from core import d, FabPart, bx, cyl_between, mirror_part
import f_wing as FW

GNSS = dict(x=200.0, y=400.0, r=34.0, t=4.0)
import config
if config.SCOUT:
    # Scout: pad moves forward to x 112 (front piece, behind the spar) so its cable hole lands in the hollow front
    # piece; at x 200 it sat right over the rear-rod tunnel and the wing skin had no hole under it.
    # R pad = DroneCAN GNSS puck, L pad = MicaSense DLS 2 adapter (f_elec.dls_adapter).
    GNSS = dict(x=112.0, y=400.0, r=34.0, t=4.0)
if config.MINI:
    GNSS = dict(x=125.0, y=200.0, r=27.0, t=4.0)          # Sky4 Holybro M9N puck (50 mm, MEASURE) on the right centre wing

def gnss_pad():
    x, y = GNSS["x"], GNSS["y"]
    zu = FW.z_bounds(y, x)[1]
    s = cyl_between((x, y, zu - 8), (x, y, zu + GNSS["t"]), GNSS["r"]) - FW.full_outer(y - 50, y + 50)
    s = s.cut(*[cyl_between((x + dx, y + dy, zu - 9), (x + dx, y + dy, zu + GNSS["t"] + 1), 2.0)
                for dx, dy in ((22, 0), (-22, 0), (0, 22), (0, -22))])                     # M3 heat-set inserts (MEASURE vs antenna)
    return s.cut(cyl_between((x, y, zu - 9), (x, y, zu + GNSS["t"] + 1), 5.0))         # cable

def horn():
    base = bx(-15, 15, -1.5, 1.5, -12, 0)                                                # bonds into a 30x3 slot
    arm = extrude(Face(Wire.make_polygon([Vector(-12, 0, 0), Vector(12, 0, 0), Vector(3, 0, 20), Vector(-3, 0, 20)], close=True)), 3, dir=(0, 1, 0)).moved(Location((0, -1.5, 0)))
    s = base.fuse(arm).clean()
    return s.cut(*[cyl_between((0, -5, z), (0, 5, z), 0.75) for z in (12.0, 17.0)])       # 1.5 mm clevis holes

JOBS = ["gnss", "horn"]

def build(job):
    if job == "gnss":
        p = FabPart("misc_gnss_pad_R", "misc", gnss_pad(), "CF-PETG", fill=0.6, up=(0, 0, 1),
                    notes=(f"GNSS antenna pad, bonds on the wing top (x {GNSS['x']:.0f}, y 400)" + ("; Scout: R = DroneCAN GNSS, L = DLS 2 adapter" if config.SCOUT else ("; R = Sky4 M9N GPS (single antenna), L = spare pad: Jetson/thermal GNSS or leave off" if config.MINI else "; dual-antenna heading, 800 mm baseline"))), print_notes="flat face up, supports under the curved face")
        return [p] if config.SCOUT else [p, mirror_part(p, "misc_gnss_pad_L")]      # Scout: one GNSS; DLS 2 sits on the hatch
    if job == "horn":
        xh = FW.HINGE_X + FW.AIL_GAP/2 + 18
        zl = FW.z_bounds(FW.SERVO_Y, xh)[0]
        s = horn().rotate(Axis.X, 180).moved(Location((xh, FW.SERVO_Y, zl - 2)))      # base 10 mm into the aileron slot
        return [FabPart("misc_control_horn", "misc", s, "PETG", fill=1.0, up=(0, 1, 0), instances=[(0, -2*FW.SERVO_Y, 0)],
                        notes="control horn: 2 for the ailerons (shown) + 2 for the ruddervators (print qty 4)", print_notes="on its side, 100 %")]
    raise KeyError(job)
