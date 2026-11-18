"""Tip polyhedral for the smaller own design (tail 0.12 x 0.36, CG from out/mini_layout.json): roll stability Cl_beta (negative = stable).
Run: $SKY_BUILD/cadenv/bin/python analysis/mini_polyhedral.py (after mini_layout.py)
"""
import sys, math, json
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.argv = ["x"]
import mini_layout as L, aerosandbox as asb
r = [x for x in json.load(open(L.OUT / "mini_layout.json"))["rows"] if x["tail_chord"] == 0.12][0]
for yb, ang in [(0.87, 0), (0.87, 8), (0.87, 10), (0.87, 12), (0.75, 10)]:
    x = lambda y: 0.03 * (L.CR - L.CT) * y / L.SEMI; c = lambda y: L.CR + (L.CT - L.CR) * y / L.SEMI
    tw = lambda y: 2.0 - 1.5 * max(0, (y - L.BY) / (L.SEMI - L.BY))
    xs = [asb.WingXSec(xyz_le=[x(y), y, 0], chord=c(y), twist=tw(y), airfoil=L.AF) for y in (0, L.BY, yb)]
    dy = L.SEMI - yb
    xs.append(asb.WingXSec(xyz_le=[x(L.SEMI), yb + dy * math.cos(math.radians(ang)), dy * math.sin(math.radians(ang))], chord=L.CT, twist=tw(L.SEMI), airfoil=L.AF))
    L.wing = asb.Wing(name="Wing", symmetric=True, xsecs=xs)
    a, rr = L.trim_run(L.plane(r["x_cg"], r["tail_le_x"], 0.12, 0.36, ti=r["tail_incidence_trim"]))
    print(yb, ang, {k: round(rr[k], 4) for k in ("Clb", "Cnb", "Cm")}, "spiral", round(rr["Clb"] * rr["Cnr"] / (rr["Cnb"] * rr["Clr"]), 2))
