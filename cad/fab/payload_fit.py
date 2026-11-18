"""Does the sensor payload fit in the smaller own design (SR3_CONFIG=mini)? (2026-11-10, user: "the CAD having the space for all of our payloads")
Payload boxes are PLACEHOLDERS with [S]/[A] sizes (MEASURE the real units). The script checks, for each box:
  1. inside the fuselage inner skin (section offset SKIN + 2 mm) at its x ends and middle,
  2. no overlap with any CAD solid (fuselage parts, purchased items, wing, booms, tail sub-assemblies),
  3. for down-looking sensors: the column from the sensor down to the belly is clear except the shell (so a belly window can be cut),
then mass / CG / hover T/W with the payload, and writes payload boxes for the animation script.
Run: cd ridgeback/cad/fab && SR3_CONFIG=mini ../run.sh payload_fit.py  -> $SKY_BUILD/payload_fit/{fit.json, payload.step}
"""
import json, math, os, sys
from pathlib import Path
sys.path[:0] = [".", "..", "../parts"]
os.environ.setdefault("SR3_CONFIG", "mini")
from build123d import Box, Location, Compound, import_step, export_step
from shapely.geometry import Point
import f_fuse as FF

O = Path(os.environ.get("SKY_BUILD", "build")) / "sr3_mini_out"
OUT = Path(os.environ.get("SKY_BUILD", "build")) / "payload_fit"; OUT.mkdir(exist_ok=True)

# name: (dims x,y,z mm, centre x,y,z mm, mass g, looks down?, size tag)
PAY = {
    "Jetson Orin Nano dev kit (with fan)": ((100, 79, 31), (110, 0, -100 + 15.5 + 0.5), 176, False, "[S] 100x79x21 board; fan height and mass [A]"),
    "Lepton 3.5 + PureThermal 3 (thermal)": ((40, 30, 22), (60, -35, -126), 25, True, "[A]"),
    "Colour camera (global shutter)": ((38, 38, 28), (60, 30, -124), 35, True, "[A]"),
    "1610 nm spot sensor (lens + photodiode board)": ((50, 40, 45), (125, 0, -125), 80, True, "[A]"),
    "SWIR scanner (future, mirror + lens + diodes)": ((120, 80, 70), (275, 0, -110), 250, True, "[A] concept only"),
}
CORE = ["Jetson Orin Nano dev kit (with fan)", "Lepton 3.5 + PureThermal 3 (thermal)", "Colour camera (global shutter)",
        "1610 nm spot sensor (lens + photodiode board)"]

def box(dims, c):
    return Box(*dims).moved(Location(c))

subs = {n: import_step(str(O / f"SR3_sub_{n}.step")) for n in ("fuselage", "purchased", "wing", "booms", "tail")}
solids = [(n, s) for n, c in subs.items() for s in c.solids()]

def overlaps(shape, ignore_shell=False):
    hits = []
    sb = shape.bounding_box()
    for n, s in solids:
        bb = s.bounding_box()
        if bb.max.X < sb.min.X or bb.min.X > sb.max.X or bb.max.Y < sb.min.Y or bb.min.Y > sb.max.Y or bb.max.Z < sb.min.Z or bb.min.Z > sb.max.Z:
            continue
        v = (shape & s).volume
        if v > 1.0:
            hits.append((n, round(v)))
    return hits

def inside_skin(dims, c, margin=2.0):
    bad = []
    for x in (c[0] - dims[0]/2, c[0], c[0] + dims[0]/2):
        P = FF.sec(x, FF.SKIN + margin)
        for y in (c[1] - dims[1]/2, c[1] + dims[1]/2):
            for z in (c[2] - dims[2]/2, c[2] + dims[2]/2):
                if not P.contains(Point(y, z)):
                    bad.append((round(x), y, z))
    return bad

res = {}
for name, (dims, c, g, down, tag) in PAY.items():
    b = box(dims, c)
    r = dict(dims_mm=dims, centre_mm=c, mass_g=g, tag=tag, outside_skin_corners=inside_skin(dims, c), overlaps=overlaps(b))
    if down:   # column from the sensor bottom to below the belly
        z0 = c[2] - dims[2]/2
        col = box((dims[0] - 4, dims[1] - 4, z0 + 175), (c[0], c[1], (z0 - 175)/2))
        r["view_blocked_by"] = [h for h in overlaps(col)]
    res[name] = r
    print(name, "| outside skin:", len(r["outside_skin_corners"]), "| overlaps:", r["overlaps"], "| view:", r.get("view_blocked_by"))

summ = json.load(open(O / "assembly_summary.json"))
M0, cg0 = summ["mass_g"], summ["cg"]
def mass_cg(names):
    m = M0 + sum(PAY[n][2] for n in names)
    x = (M0 * cg0[0] + sum(PAY[n][2] * PAY[n][1][0] for n in names)) / m
    return m, x
# hover T/W, same method as analysis/mini_sizing.py: MN5008 + P18 table max 4215 g at 22.51 V [V], 21.6 V under load [A], rho test 1.20 [A]
def tw(m_g, rho):
    return 4 * 4215 * (21.6 / 22.51) ** 2 * rho / 1.20 / m_g
rho_hot = 101325 * (1 - 2.25577e-5 * 800) ** 5.25588 / (287.05 * (288.15 - 0.0065 * 800 + 20))
for label, names in (("core payload", CORE), ("core + scanner", list(PAY))):
    m, x = mass_cg(names)
    res[label] = dict(mass_kg=round(m / 1000, 3), cg_x_mm=round(x, 1), cg_shift_mm=round(x - cg0[0], 1),
                      tw_sea_level=round(tw(m, 1.225), 2), tw_800m_hot=round(tw(m, rho_hot), 2),
                      max_mass_kg_for_tw15_800m_hot=round(4 * 4215 * (21.6 / 22.51) ** 2 * rho_hot / 1.20 / 1.5 / 1000, 2))
    print(label, res[label])
res["empty_cad"] = dict(mass_kg=round(M0 / 1000, 3), cg_x_mm=round(cg0[0], 1), tw_800m_hot=round(tw(M0, rho_hot), 2))
json.dump(res, open(OUT / "fit.json", "w"), indent=1)
export_step(Compound([box(PAY[n][0], PAY[n][1]) for n in PAY]), str(OUT / "payload.step"))
json.dump({n: dict(dims=PAY[n][0], c=PAY[n][1], down=PAY[n][3]) for n in PAY}, open(OUT / "payload_boxes.json", "w"), indent=1)
