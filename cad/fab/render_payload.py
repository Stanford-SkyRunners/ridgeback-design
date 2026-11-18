"""Animations of the sensor payload inside the smaller own design (2026-11-10). Reads the CAD sub-assembly STEPs and payload_fit.py output.
Run after payload_fit.py: cd ridgeback/cad/fab && SR3_CONFIG=mini ../run.sh render_payload.py
 -> $SKY_BUILD/payload_fit/{payload_turntable.gif, payload_install.gif, payload_side.png}
"""
import json, os, sys, math
from pathlib import Path
import numpy as np
sys.path[:0] = [".", "..", "../parts"]
os.environ.setdefault("SR3_CONFIG", "mini")
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.patches import Patch
from PIL import Image

O = Path(os.environ.get("SKY_BUILD", "build")) / "sr3_mini_out"; OUT = Path(os.environ.get("SKY_BUILD", "build")) / "payload_fit"
CACHE = OUT / "mesh_cache.npz"

def load():
    if CACHE.exists():
        z = np.load(CACHE, allow_pickle=True); return list(z["groups"])
    from build123d import import_step
    groups = []
    for sub in ("fuselage", "purchased", "wing", "booms", "tail"):
        for s in import_step(str(O / f"SR3_sub_{sub}.step")).solids():
            bb = s.bounding_box()
            vs, fs = s.tessellate(1.5, 0.6)
            tri = np.array([[(v.X, v.Y, v.Z) for v in (vs[a], vs[b], vs[c])] for a, b, c in fs], dtype=np.float32)
            dx, dy, dz = bb.max.X - bb.min.X, bb.max.Y - bb.min.Y, bb.max.Z - bb.min.Z
            kind = sub
            if sub == "fuselage" and dy > 120 and dz > 90 and dx > 90: kind = "shell"
            if sub == "purchased" and dz < 6 and dx > 250: kind = "prop"
            if sub == "purchased" and abs(dx - 225) < 90 and abs(dz - 75) < 3 and dy > 120: kind = "battery"
            groups.append((kind, tri, (bb.min.X, bb.max.X, bb.min.Y, bb.max.Y, bb.min.Z, bb.max.Z)))
    np.savez(CACHE, groups=np.array(groups, dtype=object))
    return groups

STYLE = {"shell": ("#9aa7b4", 0.10), "fuselage": ("#6f7c88", 0.55), "purchased": ("#3b3f45", 0.9), "battery": ("#c9a227", 0.85),
         "wing": ("#d5dbe1", 0.18), "booms": ("#59626b", 0.6), "tail": ("#c5ccd3", 0.25), "prop": ("#8899aa", 0.12)}
PCOL = ["#1f77b4", "#d62728", "#2ca02c", "#9467bd", "#ff7f0e"]

def box_tris(dims, c):
    dx, dy, dz = (d / 2 for d in dims); x, y, z = c
    V = np.array([[x + sx*dx, y + sy*dy, z + sz*dz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    F = [(0, 1, 3, 2), (4, 5, 7, 6), (0, 1, 5, 4), (2, 3, 7, 6), (0, 2, 6, 4), (1, 3, 7, 5)]
    return [V[list(f)] for f in F]

groups = load()
pay = json.load(open(OUT / "payload_boxes.json")); fit = json.load(open(OUT / "fit.json"))
names = list(pay)

def draw(ax, elev, azim, xlim, ylim, zlim, offsets=None, kinds=None, cones=False):
    ax.cla()
    for kind, tri, bb in groups:
        if kinds and kind not in kinds: continue
        if bb[1] < xlim[0] or bb[0] > xlim[1] or bb[3] < ylim[0] or bb[2] > ylim[1]: continue
        col, a = STYLE[kind]
        step = 1 if len(tri) < 4000 else int(len(tri) / 4000)
        ax.add_collection3d(Poly3DCollection(tri[::step], facecolor=col, edgecolor="none", alpha=a))
    for i, n in enumerate(names):
        p = pay[n]; dz = (offsets or {}).get(n, 0.0)
        c = (p["c"][0], p["c"][1], p["c"][2] + dz)
        ax.add_collection3d(Poly3DCollection(box_tris(p["dims"], c), facecolor=PCOL[i], edgecolor="k", linewidths=0.4, alpha=0.95))
        if cones and p["down"] and dz == 0:
            z0 = c[2] - p["dims"][2] / 2; L = 260; h = L * math.tan(math.radians(25))
            base = [(c[0] + h*math.cos(t), c[1] + h*math.sin(t), z0 - L) for t in np.linspace(0, 2*math.pi, 13)]
            ax.add_collection3d(Poly3DCollection([[(c[0], c[1], z0), base[k], base[k + 1]] for k in range(12)], facecolor=PCOL[i], alpha=0.12, edgecolor="none"))
    ax.set_xlim(*xlim); ax.set_ylim(*ylim); ax.set_zlim(*zlim)
    ax.set_box_aspect((xlim[1] - xlim[0], ylim[1] - ylim[0], zlim[1] - zlim[0]), zoom=1.7)
    ax.view_init(elev=elev, azim=azim); ax.set_axis_off()

def legend(fig, extra=""):
    h = [Patch(color=PCOL[i], label=n) for i, n in enumerate(names)] + [Patch(color=STYLE["battery"][0], label="6S6P battery (in CAD)")]
    fig.legend(handles=h, loc="lower center", ncol=2, fontsize=8, frameon=False)
    c = fit["core payload"]; s = fit["core + scanner"]
    fig.text(0.5, 0.965, "Payload fits the fuselage: no overlaps with any CAD part" + extra, ha="center", fontsize=11, weight="bold")
    fig.text(0.5, 0.935, f"Core payload: {c['mass_kg']} kg, CG shift {c['cg_shift_mm']} mm, hover T/W {c['tw_800m_hot']} at 800 m hot (limit 1.5).  "
             f"+ scanner: {s['mass_kg']} kg, T/W {s['tw_800m_hot']}", ha="center", fontsize=8)

def gif(frames, path, ms):
    strip = Image.new("RGB", (frames[0].width, frames[0].height * 3))       # one shared palette so colours stay fixed across frames
    for k, f in enumerate((frames[0], frames[len(frames) // 2], frames[-1])): strip.paste(f, (0, k * f.height))
    pal = strip.quantize(colors=200, method=Image.Quantize.MEDIANCUT)
    q = [f.quantize(palette=pal, dither=Image.Dither.NONE) for f in frames]
    q[0].save(path, save_all=True, append_images=q[1:], duration=ms, loop=0)

def grab(fig):
    fig.canvas.draw(); return Image.fromarray(np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy())

FX, FY, FZ = (-420, 620), (-260, 260), (-330, 120)
fig = plt.figure(figsize=(8, 5.6), dpi=100); ax = fig.add_axes([0, 0.1, 1, 0.82], projection="3d")

# 1. turntable around the fuselage, with view cones
frames = []
for k in range(36):
    draw(ax, 18, -60 + 10*k, FX, FY, FZ, kinds={"shell", "fuselage", "purchased", "battery", "wing", "booms"}, cones=True)
    if k == 0: legend(fig, "")
    frames.append(grab(fig))
gif(frames, OUT / "payload_turntable.gif", 140)

# 2. install: each box drops into place in turn, side-cutaway view
frames = []
for i, n in enumerate(names):
    for t in np.linspace(1, 0, 8):
        off = {m: (0 if j < i else (200 * t if j == i else 200)) for j, m in enumerate(names)}
        draw(ax, 12, -75, FX, FY, (-200, 300), offsets=off, kinds={"shell", "fuselage", "purchased", "battery"})
        frames.append(grab(fig))
    frames += [frames[-1]] * 4
gif(frames, OUT / "payload_install.gif", 110)

# 3. still: side view
draw(ax, 0, -90, FX, FY, FZ, kinds={"shell", "fuselage", "purchased", "battery", "wing", "booms"}, cones=True)
fig.savefig(OUT / "payload_side.png", dpi=130)
print("ok")
