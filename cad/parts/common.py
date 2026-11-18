"""Shared helpers: export STEP + STL to cad/out and sanity-check the mesh."""
from pathlib import Path
import trimesh
from build123d import export_step, export_stl

OUT = Path(__file__).resolve().parents[1] / "out"   # ridgeback/cad/out -> scratch
OUT.mkdir(exist_ok=True)

def export(part, name):
    step, stl = OUT / f"{name}.step", OUT / f"{name}.stl"
    export_step(part, str(step))
    export_stl(part, str(stl), tolerance=0.01, angular_tolerance=0.1)
    if not trimesh.load(str(stl)).is_watertight:          # near-tangent cuts: retry finer
        export_stl(part, str(stl), tolerance=0.004, angular_tolerance=0.05)
    m = trimesh.load(str(stl))
    bb = part.bounding_box()
    print(f"{name}: {bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, "
          f"vol {part.volume/1000:.1f} cm3, ~{part.volume/1000*1.27:.0f} g PETG solid-equiv, "
          f"watertight={m.is_watertight}")
    assert m.is_watertight, f"{name} mesh is not watertight"
    preview(name)
    return step, stl

def preview(name, elev=30, azim=-60, mesh=None, out=None, size=(7, 5)):
    """Render cad/out/<name>.png (headless, Lambert-shaded, true proportions)."""
    import numpy as np
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    m = mesh if mesh is not None else trimesh.load(str(OUT / f"{name}.stl"))
    el, az = np.radians(elev), np.radians(azim)
    light = np.array([np.cos(el)*np.cos(az), np.cos(el)*np.sin(az), np.sin(el)]) + np.array([0.3, -0.2, 0.6])
    light /= np.linalg.norm(light)
    shade = 0.35 + 0.65*np.clip(np.abs(m.face_normals @ light), 0, 1)
    base = np.array([0.42, 0.62, 0.80])
    colors = np.clip(base[None, :]*shade[:, None], 0, 1)
    fig = plt.figure(figsize=size, dpi=140)
    ax = fig.add_subplot(111, projection="3d")
    ax.add_collection3d(Poly3DCollection(m.triangles, facecolors=colors, edgecolor="none"))
    lo, hi = m.bounds
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2])
    ax.set_box_aspect(np.maximum(hi - lo, 1e-3))
    ax.view_init(elev, azim); ax.set_title(name); ax.set_axis_off()
    fig.savefig(out or (OUT / f"{name}.png"), bbox_inches="tight", pad_inches=0.05); plt.close(fig)
