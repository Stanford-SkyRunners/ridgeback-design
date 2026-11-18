"""Dynamic modes of the smaller own design, re-run with CAD masses (closes the limit in OWN_SMALL_DRONE §4e:
"assumed mass positions. Re-run with CAD masses").
Mass: every printed/CNC part from the mini build meta (mass_g, centre, bbox, instances) plus every purchased item from
assemble_mini.purchased_mini() (solid centre + bbox, or the lump cg). Each item adds its own box inertia m(a^2+b^2)/12
from its bounding box (thin-walled parts are over-estimated slightly; tubes are close to a slender rod).
Frame: CAD mm, same x origin as mini_layout (CAD CG target 94 mm = layout x_cg 0.094 m); converted to m.
Geometry, trim CL and AVL commands: unchanged, imported from mini_avl.py.
Run: cd ridgeback/analysis && SR3_CONFIG=mini $SKY_BUILD/cadenv/bin/python mini_avl_cad.py
  -> out_avl_mini_cad/{mini.mass, eig_*.txt, modes.json}
"""
import json, os, sys
from pathlib import Path
import numpy as np

H = Path(__file__).resolve().parent
FAB = H.parent / "cad" / "fab"
sys.path[:0] = [str(FAB), str(FAB.parent), str(FAB.parent / "parts"), str(H)]
os.environ.setdefault("SR3_CONFIG", "mini")
META = Path(os.environ.get("SKY_BUILD", "build")) / "sr3_mini_out/meta"

rows = []  # (name, kg, centre_m[3], box_dims_m[3])
for f in sorted(META.glob("_job_*.json")):
    for p in json.load(open(f)).get("parts", []):
        lo, hi = np.array(p["bbox"][0]), np.array(p["bbox"][1])
        for inst in p.get("instances", [[0, 0, 0]]):
            rows.append((p["name"], p["mass_g"] / 1000, (np.array(p["center"]) + inst) / 1000, (hi - lo) / 1000))
n_print = len(rows)

from assemble_mini import purchased_mini
P, _ = purchased_mini()
for p in P:
    if p["solid"] is not None:
        c = p["solid"].center(); b = p["solid"].bounding_box()
        cen = np.array([c.X, c.Y, c.Z]); dims = np.array([b.max.X - b.min.X, b.max.Y - b.min.Y, b.max.Z - b.min.Z])
    else:
        cen = np.array(p["cg"]); dims = np.zeros(3)
    rows.append((p["name"], p["mass_g"] / 1000, cen / 1000, dims / 1000))

m = np.array([r[1] for r in rows]); x = np.array([r[2] for r in rows]); dm = np.array([r[3] for r in rows])
M = m.sum(); cg = (m[:, None] * x).sum(0) / M; d = x - cg
Ixx = (m * (d[:, 1]**2 + d[:, 2]**2 + (dm[:, 1]**2 + dm[:, 2]**2) / 12)).sum()
Iyy = (m * (d[:, 0]**2 + d[:, 2]**2 + (dm[:, 0]**2 + dm[:, 2]**2) / 12)).sum()
Izz = (m * (d[:, 0]**2 + d[:, 1]**2 + (dm[:, 0]**2 + dm[:, 1]**2) / 12)).sum()
Ixz = (m * d[:, 0] * d[:, 2]).sum()

import mini_avl as A                      # its module-level point-mass model is kept for comparison
OUT = H / "out_avl_mini_cad"; OUT.mkdir(exist_ok=True); A.OUT = OUT
(OUT / "mini.mass").write_text("Lunit = 1.0 m\nMunit = 1.0 kg\nTunit = 1.0 s\ng = 9.81\nrho = 1.225\n"
    f"{M:.4f} {cg[0]:.4f} {cg[1]:.4f} {cg[2]:.4f} {Ixx:.4f} {Iyy:.4f} {Izz:.4f} 0 {-Ixz:.4f} 0\n")

res = dict(source="CAD mini build meta + purchased_mini()", n_items=len(rows), n_printed_instances=n_print,
           mass_kg=round(M, 3), cg=[round(c, 4) for c in cg], Ixx=round(Ixx, 3), Iyy=round(Iyy, 3), Izz=round(Izz, 3), Ixz=round(Ixz, 4),
           old_point_mass_model=dict(mass_kg=round(A.M, 3), cg=[round(c, 4) for c in A.cg], Ixx=round(A.Ixx, 3), Iyy=round(A.Iyy, 3),
                                     Izz=round(A.Izz, 3), Ixz=round(A.Ixz, 4)),
           CL_loiter=round(A.L.CL_T, 3), CDp=round(A.CDP, 4), cases={})
for tip in (0, 10):
    t = A.geom(tip, f"tip{tip}")
    res["cases"][f"tips {tip} deg, loiter CL {A.L.CL_T:.2f}"] = A.run(f"tip{tip}", t, A.L.CL_T)
    res["cases"][f"tips {tip} deg, slow CL 1.10"] = A.run(f"tip{tip}_slow", t, 1.10)
json.dump(res, open(OUT / "modes.json", "w"), indent=1)
print(json.dumps({k: v for k, v in res.items() if k != "cases"}, indent=1))
for k, v in res["cases"].items():
    print(k)
    for e in v: print("   ", e)
