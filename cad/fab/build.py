"""Build every SR-3 fabrication part, then the integrated assembly, BOM and report.

    ridgeback/cad/run.sh ridgeback/cad/fab/build.py            # everything
    ridgeback/cad/run.sh ridgeback/cad/fab/build.py f_wing     # one module (parts only)
    ridgeback/cad/run.sh ridgeback/cad/fab/build.py --assemble # assembly/BOM from existing part files

Outputs (fab/out -> $SKY_BUILD/sr3_fab_out):
    step/<part>.step     each part IN PLACE in the aircraft frame (Fusion 360: Insert > Insert into current design, they line up)
    stl/<part>.stl       each part in PRINT ORIENTATION, centred on a 256 mm bed
    dxf/<part>.dxf       CNC profiles (motor plates, battery tray)
    png/<part>.png       preview
    SR3_assembly.step    integrated aircraft, one named component per part + purchased envelopes
"""
import importlib, json, math, sys, time, traceback
from concurrent.futures import ProcessPoolExecutor, as_completed
from multiprocessing import get_context
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import core  # noqa: E402

MODULES = ["f_wing", "f_fuse", "f_boom", "f_tail", "f_misc", "f_elec"]     # f_elec has jobs only when SR3_CONFIG=scout
OUT = core.OUT
for sub in ("step", "stl", "dxf", "png", "meta"):
    (OUT / sub).mkdir(parents=True, exist_ok=True)


def render(mesh, path, title, elev=28, azim=-60):
    import numpy as np
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    el, az = np.radians(elev), np.radians(azim)
    light = np.array([np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)]) + np.array([0.3, -0.2, 0.6])
    light /= np.linalg.norm(light)
    shade = 0.35 + 0.65*np.clip(np.abs(mesh.face_normals @ light), 0, 1)
    colors = np.clip(np.array([0.42, 0.62, 0.80])[None, :]*shade[:, None], 0, 1)
    fig = plt.figure(figsize=(6, 4.5), dpi=110)
    ax = fig.add_subplot(111, projection="3d")
    ax.add_collection3d(Poly3DCollection(mesh.triangles, facecolors=colors, edgecolor="none"))
    lo, hi = mesh.bounds
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(np.maximum(hi - lo, 1e-3)); ax.view_init(elev, azim); ax.set_axis_off()
    ax.set_title(title, fontsize=8)
    fig.savefig(path, bbox_inches="tight", pad_inches=0.05); plt.close(fig)


def export_part(p):
    import trimesh
    from build123d import export_step
    t0 = time.time()
    export_step(p.solid, str(OUT / "step" / f"{p.name}.step"))
    m = None
    for tol, ang in ((0.05, 0.2), (0.02, 0.1), (0.008, 0.05)):
        m = core.mesh_of(p.solid, tol, ang)
        if m.is_watertight:
            break
    if not m.is_watertight:
        trimesh.repair.fill_holes(m); m.merge_vertices()
    mo, dims, fits, up_used = core.print_orient(m, p.up)
    mo.export(str(OUT / "stl" / f"{p.name}.stl"))
    if p.dxf is not None:
        from build123d import ExportDXF
        ex = ExportDXF(); ex.add_shape(p.dxf); ex.write(str(OUT / "dxf" / f"{p.name}.dxf"))
    mat = core.MATERIALS[p.material]
    vol = p.solid.volume/1000.0
    mass = vol*mat["rho"]*p.fill
    render(mo, OUT / "png" / f"{p.name}.png", f"{p.name}  {dims[0]:.0f}x{dims[1]:.0f}x{dims[2]:.0f} mm  {p.material}  {mass:.0f} g")
    bb = p.solid.bounding_box()
    c = p.solid.center()
    if p.instances:
        p.qty = 1 + len(p.instances)
    meta = dict(name=p.name, group=p.group, material=p.material, process=mat["proc"], qty=p.qty, fill=p.fill,
                vol_cm3=round(vol, 2), mass_g=round(mass, 1), print_dims=[round(float(x), 1) for x in dims],
                fits_bed=bool(fits), up_used=list(up_used), watertight=bool(m.is_watertight),
                valid=bool(p.solid.is_valid), n_solids=len(core.solids_of(p.solid)),
                bbox=[[bb.min.X, bb.min.Y, bb.min.Z], [bb.max.X, bb.max.Y, bb.max.Z]], center=[c.X, c.Y, c.Z],
                notes=p.notes, print_notes=p.print_notes, instances=[[0, 0, 0]] + [list(t) for t in (p.instances or [])], dxf=p.dxf is not None, secs=round(time.time() - t0, 1))
    (OUT / "meta" / f"{p.name}.json").write_text(json.dumps(meta, indent=1))
    return meta


def run_job(mod, job):
    sys.path.insert(0, str(HERE))
    t0 = time.time()
    try:
        m = importlib.import_module(mod)
        parts = m.build(job)
        metas = [export_part(p) for p in parts]
        return dict(mod=mod, job=job, ok=True, parts=metas, secs=round(time.time() - t0, 1))
    except Exception as e:
        return dict(mod=mod, job=job, ok=False, err="".join(traceback.format_exception(type(e), e, e.__traceback__))[-3000:])


def build_parts(mods, workers=7):
    """Each job runs in its own interpreter (an OCC segfault then only loses that job)."""
    import subprocess
    from concurrent.futures import ThreadPoolExecutor
    jobs = []
    for mod in mods:
        m = importlib.import_module(mod)
        jobs += [(mod, j) for j in m.JOBS]
    print(f"{len(jobs)} jobs on {workers} workers")
    def one(mj):
        mod, j = mj
        rf = OUT / "meta" / f"_job_{mod}_{j}.json"
        rf.unlink(missing_ok=True)
        p = subprocess.run([sys.executable, __file__, "--job", mod, j], capture_output=True, text=True, timeout=3600)
        if rf.exists():
            return json.loads(rf.read_text())
        return dict(mod=mod, job=j, ok=False, err=f"exit {p.returncode}\n{p.stderr[-2000:]}")
    res = []
    with ThreadPoolExecutor(workers) as ex:
        for r in ex.map(one, jobs):
            res.append(r)
            if r["ok"]:
                for p in r["parts"]:
                    flag = "" if ((p["fits_bed"] or p["process"] != "print") and p["watertight"] and p["valid"]) else "  <-- CHECK"
                    print(f"  {p['name']:34s} {p['material']:8s} {p['mass_g']:7.1f} g  {'x'.join(f'{v:.0f}' for v in p['print_dims']):>13s}"
                          f"  fit={p['fits_bed']} wt={p['watertight']} n={p['n_solids']} {p['secs']}s{flag}", flush=True)
            else:
                print(f"  FAILED {r['mod']}:{r['job']}\n{r['err']}", flush=True)
    return res


if __name__ == "__main__":
    if sys.argv[1:2] == ["--job"]:
        mod, j = sys.argv[2], sys.argv[3]
        r = run_job(mod, j)
        (OUT / "meta" / f"_job_{mod}_{j}.json").write_text(json.dumps(r))
        sys.exit(0)
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if "--assemble" not in sys.argv:
        mods = args or [m for m in MODULES if (HERE / f"{m}.py").exists()]
        res = build_parts(mods)
        bad = [r for r in res if not r["ok"]]
        print(f"done: {sum(len(r['parts']) for r in res if r['ok'])} parts, {len(bad)} failed jobs")
    if not args:
        import assemble
        assemble.main()
