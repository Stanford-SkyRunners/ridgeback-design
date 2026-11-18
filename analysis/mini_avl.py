"""Dynamic modes of the smaller own design with AVL 3.40 (same method as avl_modes.py for the 3.35 m design).
Geometry from mini_layout.py (tail 0.12 x 0.36, CG for 15 % static margin), flat wing and 10 deg tips from y 0.87.
Inertia: point masses at assumed positions [A] (no CAD yet) + slender-body terms for the wing, booms and battery.
Run: $SKY_BUILD/cadenv/bin/python mini_avl.py -> out_avl_mini/modes.json
"""
import os
import json, math, subprocess, warnings
warnings.filterwarnings("ignore")
from pathlib import Path
import aerosandbox as asb
import mini_layout as L
H = Path(__file__).resolve().parent
AVL = os.path.join(os.environ.get("SKY_BUILD", "build"), "avl_build/Avl/bin/avl")
OUT = H / "out_avl_mini"; OUT.mkdir(exist_ok=True)
lay = [r for r in json.load(open(H / "out/mini_layout.json"))["rows"] if r["tail_chord"] == 0.12][0]
XCG, TX, TI = lay["x_cg"], lay["tail_le_x"], lay["tail_incidence_trim"]
XF, XR = lay["x_front_motor"], lay["x_rear_motor"]
CDP = 0.025 + 4 * 0.0019 * (18 / 28) ** 2 / L.S                 # mini_sizing.py CD0
BY = L.BY

def wing(tip_deg):
    x = lambda y: 0.03 * (L.CR - L.CT) * y / L.SEMI; c = lambda y: L.CR + (L.CT - L.CR) * y / L.SEMI
    tw = lambda y: 2.0 - 1.5 * max(0, (y - BY) / (L.SEMI - BY)); yb = 0.87; dy = L.SEMI - yb
    xs = [asb.WingXSec(xyz_le=[x(y), y, 0], chord=c(y), twist=tw(y), airfoil=L.AF) for y in (0, BY, yb)]
    xs.append(asb.WingXSec(xyz_le=[x(L.SEMI), yb + dy * math.cos(math.radians(tip_deg)), dy * math.sin(math.radians(tip_deg))],
                           chord=L.CT, twist=tw(L.SEMI), airfoil=L.AF))
    return asb.Wing(name="Wing", symmetric=True, xsecs=xs)

# point masses [A]: (name, kg, x, y, z); battery x solved so the CG lands on XCG
pts = [("wing printed + spar", 0.92 + 0.21, 0.30 * L.CR, 0, 0.0), ("tail", 0.33, TX + 0.06, 0, 0.15),
       ("fuselage", 0.77, 0.05, 0, -0.09), ("sensor kit", 0.45, -0.20, 0, -0.14), ("avionics", 0.22, -0.10, 0, -0.06),
       ("pusher motor + prop + ESC", 0.30, 0.60, 0, -0.08), ("harness", 0.40, 0.10, 0, -0.06), ("servos", 0.09, 0.20, 0, 0)]
for sgn in (1, -1):
    pts += [(f"boom tube + printed boom parts {sgn}", 0.24 + 0.346 + 0.05, (XF + TX) / 2, sgn * BY, -0.03),
            (f"front lift set {sgn}", 0.248, XF, sgn * BY, 0.03), (f"rear lift set {sgn}", 0.248, XR, sgn * BY, 0.03)]
M_BAT = 2.72
m0 = sum(p[1] for p in pts); x_bat = (XCG * (m0 + M_BAT) - sum(p[1] * p[2] for p in pts)) / M_BAT
pts.append(("battery 6S6P", M_BAT, x_bat, 0, -0.08))
M = sum(p[1] for p in pts); cg = [sum(p[1] * p[i] for p in pts) / M for i in (2, 3, 4)]
Ixx = Iyy = Izz = Ixz = 0.0
for n, m, x, y, z in pts:
    dx, dy_, dz = x - cg[0], y - cg[1], z - cg[2]
    Ixx += m * (dy_ ** 2 + dz ** 2); Iyy += m * (dx ** 2 + dz ** 2); Izz += m * (dx ** 2 + dy_ ** 2); Ixz += m * dx * dz
Ixx += 1.13 * L.B ** 2 / 12; Izz += 1.13 * L.B ** 2 / 12                        # wing as a slender rod along y
lb = TX + 0.12 - XF; Iyy += 2 * 0.64 * lb ** 2 / 12; Izz += 2 * 0.64 * lb ** 2 / 12  # booms along x
(OUT / "mini.mass").write_text("Lunit = 1.0 m\nMunit = 1.0 kg\nTunit = 1.0 s\ng = 9.81\nrho = 1.225\n"
    f"{M:.4f} {cg[0]:.4f} {cg[1]:.4f} {cg[2]:.4f} {Ixx:.4f} {Iyy:.4f} {Izz:.4f} 0 {-Ixz:.4f} 0\n")

def geom(tip_deg, name):
    p = L.plane(XCG, TX, 0.12, 0.36, ti=TI); p.wings[0] = wing(tip_deg)
    f = OUT / f"{name}_raw.avl"; p.export_AVL(str(f))
    clean, skip = [], 0
    for l in f.read_text().splitlines():
        if skip: skip -= 1; continue
        if l.strip() == "CDCL": skip = 2; continue
        clean.append(l)
    t = "\n".join(clean) + "\n"
    if "# CDp\n0\n" in t: t = t.replace("# CDp\n0\n", f"# CDp\n{CDP:.4f}\n")
    return t

def run(name, txt, cl):
    g = OUT / f"{name}.avl"; g.write_text(txt); e = OUT / f"eig_{name}.txt"; e.unlink(missing_ok=True)
    cmd = f"PLOP\nG\n\nLOAD {g}\nMASS {OUT / 'mini.mass'}\nMSET\n0\nOPER\nC1\nC {cl}\n\nX\n\nMODE\nN\nW\n{e}\n\nQUIT\n"
    rr = subprocess.run([AVL], input=cmd, capture_output=True, text=True, cwd=OUT, timeout=60)
    (OUT / f"avl_{name}.log").write_text(rr.stdout + "\n---stderr---\n" + rr.stderr)
    out = []
    for l in e.read_text().splitlines():
        if not l.strip() or l.startswith("#"): continue
        re_, im = map(float, l.split()[1:3])
        if im < 0: continue
        wn = math.hypot(re_, im)
        out.append(dict(real=round(re_, 3), imag=round(im, 3), wn=round(wn, 3), damping=round(-re_ / wn, 3) if wn else None,
                        t_half_or_double_s=round(math.log(2) / abs(re_), 2) if abs(re_) > 1e-6 else None, stable=re_ < 0))
    return out

if __name__ == "__main__":
    CL_T = L.CL_T
    res = dict(mass_kg=round(M, 3), cg=[round(c, 4) for c in cg], x_battery=round(x_bat, 3), Ixx=round(Ixx, 3), Iyy=round(Iyy, 3),
               Izz=round(Izz, 3), Ixz=round(Ixz, 4), CL_loiter=round(CL_T, 3), CDp=round(CDP, 4), cases={})
    for tip in (0, 10):
        t = geom(tip, f"tip{tip}")
        res["cases"][f"tips {tip} deg, loiter CL {CL_T:.2f}"] = run(f"tip{tip}", t, CL_T)
        res["cases"][f"tips {tip} deg, slow CL 1.10"] = run(f"tip{tip}_slow", t, 1.10)
    json.dump(res, open(OUT / "modes.json", "w"), indent=1)
    print(json.dumps({k: v for k, v in res.items() if k != "cases"}))
    for k, v in res["cases"].items():
        print(k)
        for e in v: print("   ", e)
