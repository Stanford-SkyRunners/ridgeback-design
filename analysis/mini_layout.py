"""Layout + static stability of the smaller own design (docs/OWN_SMALL_DRONE_2026-10-30.md), before any CAD.
Same planform rules as the 3.35 m design (taper 0.65, SD7037, 2 deg incidence, 1.5 deg washout, flat wing, arch tail from the
boom ends to a centreline apex). Places the CG for a 15 % MAC static margin [A], the lift motors around the CG with the same
clearances as the 3.35 m design (rear disc 50 mm behind the root trailing edge), the tail behind the rear discs, then runs
AeroSandbox VLM with stability derivatives for a few tail sizes. Wing + tail only (fuselage and booms not in the VLM).
Run: $SKY_BUILD/cadenv/bin/python mini_layout.py -> out/mini_layout.json
"""
import json, math, warnings
warnings.filterwarnings("ignore")
import aerosandbox as asb
from pathlib import Path
OUT = Path(__file__).resolve().parent / "out"
S, B, TAPER = 0.50, 2.24, 0.65
CR = 2 * S / (B * (1 + TAPER)); CT = CR * TAPER; SEMI = B / 2
MAC = 2 / 3 * CR * (1 + TAPER + TAPER ** 2) / (1 + TAPER)
BY, BZ = 0.39, -0.02                 # [A] boom at 600 x 18/28 mm; just under the wing
R_PROP = 18 * 0.0254 / 2
M, RHO, V = 8.87, 1.225, 19.3        # [C] mini_sizing / mini_spar
CL_T = M * 9.81 / (0.5 * RHO * V ** 2 * S)
SM_TARGET = 0.15
AF, AF_T = asb.Airfoil("sd7037"), asb.Airfoil("naca0010")
wing = asb.Wing(name="Wing", symmetric=True, xsecs=[
    asb.WingXSec(xyz_le=[0, 0, 0], chord=CR, twist=2.0, airfoil=AF),
    asb.WingXSec(xyz_le=[0.03 * (CR - CT) * BY / SEMI, BY, 0], chord=CR + (CT - CR) * BY / SEMI, twist=2.0, airfoil=AF),
    asb.WingXSec(xyz_le=[0.03 * (CR - CT), SEMI, 0], chord=CT, twist=0.5, airfoil=AF)])

def plane(xcg, tx, tc, th, ti=0.0):
    arch = asb.Wing(name="Arch", symmetric=True, xsecs=[
        asb.WingXSec(xyz_le=[tx + 0.015, 0, BZ + th], chord=tc * 0.9, twist=ti, airfoil=AF_T),
        asb.WingXSec(xyz_le=[tx, BY, BZ], chord=tc, twist=ti, airfoil=AF_T)])
    return asb.Airplane(name="mini", xyz_ref=[xcg, 0, -0.03], wings=[wing, arch], s_ref=S, b_ref=B, c_ref=MAC)

def trim_run(p):
    vlm = lambda a: asb.VortexLatticeMethod(p, asb.OperatingPoint(velocity=V, alpha=a, atmosphere=asb.Atmosphere(altitude=0)),
                                            spanwise_resolution=10, chordwise_resolution=6)
    a0, a1 = 0.0, 4.0; c0, c1 = float(vlm(a0).run()["CL"]), float(vlm(a1).run()["CL"])
    for _ in range(5):
        a2 = a1 + (CL_T - c1) * (a1 - a0) / (c1 - c0); a0, c0 = a1, c1; a1 = a2; c1 = float(vlm(a1).run()["CL"])
    r = vlm(a1).run_with_stability_derivatives(alpha=True, beta=True, p=True, q=False, r=True)
    return a1, {k: float(r[k]) for k in ("CL", "Cm", "Cma", "x_np", "Clb", "Cnb", "Clr", "Cnr")}

if __name__ == "__main__":
  rows = []
  for tc, th in [(0.10, 0.30), (0.12, 0.36), (0.14, 0.40)]:
      xcg = 0.25 * MAC                                    # first guess
      for _ in range(3):
          x_rear = CR + 0.05 + R_PROP; x_front = 2 * xcg - x_rear
          tx = x_rear + R_PROP + 0.05                     # tail LE 50 mm behind the rear disc
          a, r = trim_run(plane(xcg, tx, tc, th))
          xcg = r["x_np"] - SM_TARGET * MAC
      a, r = trim_run(plane(xcg, tx, tc, th))
      # tail incidence that trims Cm to 0 (linear in incidence, two runs)
      _, r2 = trim_run(plane(xcg, tx, tc, th, ti=-2.0))
      ti = -2.0 * r["Cm"] / (r["Cm"] - r2["Cm"]) if r["Cm"] != r2["Cm"] else float("nan")
      lt = tx + 0.25 * tc - xcg; s_arch = 2 * tc * math.hypot(BY, th)
      rows.append(dict(tail_chord=tc, apex_h=th, tail_le_x=round(tx, 3), x_cg=round(xcg, 3), x_front_motor=round(x_front, 3),
                       x_rear_motor=round(x_rear, 3), front_disc_clear_le=round(-(x_front + R_PROP), 3), alpha=round(a, 2),
                       static_margin=round((r["x_np"] - xcg) / MAC, 3), Cm=round(r["Cm"], 4), tail_incidence_trim=round(ti, 2),
                       Cnb=round(r["Cnb"], 4), Clb=round(r["Clb"], 4),
                       spiral_ratio=round(r["Clb"] * r["Cnr"] / (r["Cnb"] * r["Clr"]), 2) if r["Cnb"] * r["Clr"] else None,
                       V_h=round(s_arch * math.cos(math.atan2(th, BY)) ** 2 * lt / (S * MAC), 3),
                       V_v=round(s_arch * math.sin(math.atan2(th, BY)) ** 2 * lt / (S * B), 4), boom_len=round(tx + tc - x_front + 0.05, 3)))
  geo = dict(S=S, b=B, root_chord=round(CR, 4), tip_chord=round(CT, 4), MAC=round(MAC, 4), boom_y=BY, prop_d_m=round(2 * R_PROP, 3),
             CL_trim_target=round(CL_T, 3), Re_MAC=round(RHO * V * MAC / 1.81e-5))
  json.dump(dict(geometry=geo, rows=rows), open(OUT / "mini_layout.json", "w"), indent=1)
  print(geo)
  for r in rows: print(r)
