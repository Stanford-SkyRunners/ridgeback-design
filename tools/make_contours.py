"""Draw a sketched hillside as contour lines with a creek, for page backgrounds (an illustration, not real terrain).
Run (needs numpy + matplotlib): python3 tools/make_contours.py  -> docs/assets/contours.svg, docs/assets/contours_scene.json
"""
import json
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT = Path(__file__).resolve().parents[1] / "docs/assets"
W, H = 1600, 900
x = np.linspace(0, W, 400); y = np.linspace(0, H, 225); X, Y = np.meshgrid(x, y)
rng = np.random.default_rng(7)
Z = 0.0 * X
for cx, cy, a, sx, sy in [(1150, 260, 260, 330, 230), (420, 640, 170, 260, 200), (820, 520, 120, 180, 140), (1450, 760, 90, 160, 140), (180, 180, 110, 220, 160)]:
    Z += a * np.exp(-(((X - cx) / sx) ** 2 + ((Y - cy) / sy) ** 2))
Z += 25 * np.sin(X / 140 + 1.3) * np.cos(Y / 170) + 0.04 * Y                       # ridges and a gentle slope
creek = 260 + 120 * np.sin(np.linspace(0, 3.2, x.size)) + np.linspace(0, 380, x.size)   # creek line (y as a function of x)
Z -= 60 * np.exp(-((Y - np.interp(X, x, creek)) / 45) ** 2)                         # valley along the creek
fig = plt.figure(figsize=(W / 100, H / 100), dpi=100); ax = fig.add_axes([0, 0, 1, 1]); ax.set_xlim(0, W); ax.set_ylim(H, 0); ax.axis("off")
cs = ax.contour(X, Y, Z, levels=np.arange(Z.min(), Z.max(), 12))
paths = []
for i, segs in enumerate(cs.allsegs):
    for seg in segs:
        if len(seg) < 8: continue
        d = "M" + " L".join(f"{p[0]:.0f},{p[1]:.0f}" for p in seg[::2])
        paths.append((i, d))
plt.close(fig)
idx = [i for i, _ in paths]; top = max(idx)
svg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" preserveAspectRatio="xMidYMid slice">']
for i, d in paths:
    major = i % 5 == 0
    svg.append(f'<path d="{d}" fill="none" stroke="#8a5a2b" stroke-opacity="{0.55 if major else 0.28}" stroke-width="{1.4 if major else 0.8}"/>')
cd = "M" + " L".join(f"{xx:.0f},{yy:.0f}" for xx, yy in zip(x[::4], creek[::4]))
svg.append(f'<path d="{cd}" fill="none" stroke="#2f5d8a" stroke-opacity="0.6" stroke-width="2.2"/>')
svg.append("</svg>")
(OUT / "contours.svg").write_text("\n".join(svg))
json.dump({"W": W, "H": H, "creek": [[float(a), float(b)] for a, b in zip(x[::8], creek[::8])]}, open(OUT / "contours_scene.json", "w"))
print("paths", len(paths), "bytes", (OUT / "contours.svg").stat().st_size)
