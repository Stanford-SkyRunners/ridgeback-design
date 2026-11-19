"""Draw the mission page's two map sheets from real terrain, in the site's survey-map style.

Elevation: AWS Terrain Tiles (Terrarium PNG, public, https://registry.opendata.aws/terrain-tiles/), contoured here.
Fires: docs/assets/map/fires.geojson (CAL FIRE historic perimeters, NIFC prescribed burns). Preserve: jrbp.geojson (OSM).
Writes docs/assets/map/region.svg and docs/assets/map/site.svg (inlined into mission.html by gen_site.py).
Run (needs numpy, scipy, matplotlib, pillow): python tools/make_map.py   [tiles cached in $MAP_CACHE or build/terrain]
"""
import io, json, math, os, urllib.request
from pathlib import Path
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parent.parent
OUT = ROOT / "docs/assets/map"
CACHE = Path(os.environ.get("MAP_CACHE", ROOT / "build/terrain")); CACHE.mkdir(parents=True, exist_ok=True)
TILE = "https://s3.amazonaws.com/elevation-tiles-prod/terrarium/{z}/{x}/{y}.png"


def gx(lon, z): return (lon + 180) / 360 * 256 * 2 ** z
def gy(lat, z):
    s = math.radians(lat); return (1 - math.log(math.tan(s) + 1 / math.cos(s)) / math.pi) / 2 * 256 * 2 ** z


def dem(lon0, lat0, lon1, lat1, z):
    """Elevation grid (m) covering the box, in global pixel coords at zoom z; returns (grid, x0, y0)."""
    X0, X1 = int(gx(lon0, z) // 256), int(gx(lon1, z) // 256)
    Y0, Y1 = int(gy(lat1, z) // 256), int(gy(lat0, z) // 256)
    rows = []
    for ty in range(Y0, Y1 + 1):
        row = []
        for tx in range(X0, X1 + 1):
            f = CACHE / f"{z}_{tx}_{ty}.png"
            if not f.exists():
                f.write_bytes(urllib.request.urlopen(TILE.format(z=z, x=tx, y=ty), timeout=30).read())
            a = np.asarray(Image.open(f).convert("RGB"), dtype=np.float64)
            row.append(a[..., 0] * 256 + a[..., 1] + a[..., 2] / 256 - 32768)
        rows.append(np.hstack(row))
    return np.vstack(rows), X0 * 256, Y0 * 256


def contours(grid, levels, ox, oy, sx, sy, step=1, nd=1):
    fig = plt.figure(); cs = plt.contour(grid, levels=levels); plt.close(fig)
    out = []
    for lev, segs in zip(cs.levels, cs.allsegs):
        for s in segs:
            if len(s) < 6: continue
            s = s[::step] if len(s) > 4 * step else s
            out.append((lev, " ".join(f"{(p[0] + ox) * sx:.{nd}f},{(p[1] + oy) * sy:.{nd}f}" for p in s)))
    return out


def water(grid, ox, oy, sx, sy, level=0.5):
    fig = plt.figure(); cs = plt.contourf(grid, levels=[-1e5, level]); plt.close(fig)
    d = []
    for path in cs.get_paths():
        for poly in path.to_polygons():
            if len(poly) < 4: continue
            d.append("M" + " L".join(f"{(p[0] + ox) * sx:.1f},{(p[1] + oy) * sy:.1f}" for p in poly) + "Z")
    return " ".join(d)


class Sheet:
    def __init__(self, lon0, lat0, lon1, lat1, z, W):
        self.z = z; self.px0, self.py0 = gx(lon0, z), gy(lat1, z)
        self.k = W / (gx(lon1, z) - self.px0); self.W = W; self.H = (gy(lat0, z) - self.py0) * self.k
        self.box = (lon0, lat0, lon1, lat1)
    def xy(self, lon, lat): return ((gx(lon, self.z) - self.px0) * self.k, (gy(lat, self.z) - self.py0) * self.k)
    def ring(self, coords): return "M" + " L".join("%.1f,%.1f" % self.xy(*c) for c in coords) + "Z"
    def terrain(self, interval, index, smooth, step, nd=1):
        lon0, lat0, lon1, lat1 = self.box
        g, X0, Y0 = dem(lon0 - .01, lat0 - .01, lon1 + .01, lat1 + .01, self.z)
        g = gaussian_filter(np.where(g < -50, -50, g), smooth)
        ox, oy = X0 - self.px0, Y0 - self.py0
        top = int(np.nanmax(g) // interval * interval)
        lines = contours(g, list(range(interval, top + 1, interval)), ox, oy, self.k, self.k, step, nd)
        return lines, water(g, ox, oy, self.k, self.k), g


def geo(f):
    return json.loads((OUT / f).read_text())["features"]


def polys(geom):
    return geom["coordinates"] if geom["type"] == "MultiPolygon" else [geom["coordinates"]]


def text(x, y, s, cls, anchor="middle", rot=0):
    r = f' transform="rotate({rot} {x:.0f} {y:.0f})"' if rot else ""
    return f'<text x="{x:.0f}" y="{y:.0f}" class="{cls}" text-anchor="{anchor}"{r}>{s}</text>'


def scalebar(sh, x, y, km, lat):
    m_per_px = 156543.03392 * math.cos(math.radians(lat)) / 2 ** sh.z / sh.k
    L = km * 1000 / m_per_px
    half = L / 2
    return (f'<g class="scale"><rect x="{x}" y="{y}" width="{half:.1f}" height="5" class="sb1"/><rect x="{x + half:.1f}" y="{y}" width="{half:.1f}" height="5" class="sb2"/>'
            f'{text(x, y - 6, "0", "tick")}{text(x + L, y - 6, f"{km * 1000:g} m" if km < 1 else f"{km:g} km", "tick", "end")}</g>')


def north(x, y):
    return f'<g class="north" transform="translate({x} {y})"><path d="M0,-22 L7,8 L0,3 L-7,8Z"/><text y="22" text-anchor="middle" class="tick">N</text></g>'


def ticks(sh, step):
    lon0, lat0, lon1, lat1 = sh.box; out = []
    v = math.ceil(lon0 / step) * step
    while v < lon1:
        x, _ = sh.xy(v, lat0)
        if 40 < x < sh.W - 70: out.append(f'<line x1="{x:.1f}" y1="0" x2="{x:.1f}" y2="8" class="grat"/>{text(x, 20, f"{abs(v):.2f}°W", "tick")}')
        v += step
    v = math.ceil(lat0 / step) * step
    while v < lat1:
        _, y = sh.xy(lon0, v)
        if 30 < y < sh.H - 50: out.append(f'<line x1="0" y1="{y:.1f}" x2="8" y2="{y:.1f}" class="grat"/>{text(12, y + 4, f"{v:.2f}°N", "tick", "start")}')
        v += step
    return "".join(out)


# ---------------------------------------------------------------- region sheet
R = Sheet(-122.62, 36.95, -121.80, 37.62, 11, 1200)
lines, sea, _ = R.terrain(100, 500, 1.2, 2, 0)
fires = geo("fires.geojson"); jr = geo("jrbp.geojson")
parts = [f'<svg viewBox="0 0 {R.W:.0f} {R.H:.0f}" class="mapsvg" role="img" aria-label="Map: wildfires and prescribed burns around Stanford since 2000, on the terrain of the Santa Cruz Mountains">',
         '<defs><clipPath id="rclip"><rect width="100%" height="100%"/></clipPath>'
         '<pattern id="hatchFire" width="7" height="7" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><line x1="0" y1="0" x2="0" y2="7" class="hf"/></pattern>'
         '<pattern id="hatchRx" width="5" height="5" patternUnits="userSpaceOnUse" patternTransform="rotate(-45)"><line x1="0" y1="0" x2="0" y2="5" class="hr"/></pattern>'
         '<pattern id="waves" width="40" height="10" patternUnits="userSpaceOnUse"><path d="M0,6 q10,-5 20,0 t20,0" class="wv"/></pattern></defs>',
         f'<g clip-path="url(#rclip)"><rect width="100%" height="100%" class="land"/><path d="{sea}" class="sea"/><path d="{sea}" class="seaw"/>']
parts += [f'<polyline points="{p}" class="ct{" ix" if lev % 500 == 0 else ""}"/>' for lev, p in lines]
for f in fires:
    pr = f["properties"]; kind = pr["kind"]
    d = " ".join(R.ring(r) for poly in polys(f["geometry"]) for r in poly)
    name = pr["name"].replace("Czu", "CZU").replace("Scu", "SCU").replace("Pg&E", "PG&amp;E")
    parts.append(f'<path d="{d}" class="{"fire" if kind == "wildfire" else "rx"}" data-tip="{name}, {pr["year"]}: {pr["acres"]:,} acres, {"wildfire" if kind == "wildfire" else "prescribed burn"}"/>')
for f in jr:
    d = " ".join(R.ring(r) for poly in polys(f["geometry"]) for r in poly)
    parts.append(f'<path d="{d}" class="jrbp" data-tip="Jasper Ridge Biological Preserve: where we plan to fly first"/>')
sx, sy = R.xy(-122.1697, 37.4275)
parts.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="4" class="pt"/>{text(sx + 8, sy + 4, "Stanford", "lab", "start")}')
jx, jy = R.xy(-122.24, 37.404)
parts.append(f'<line x1="{jx - 12:.0f}" y1="{jy - 14:.0f}" x2="{jx - 2:.0f}" y2="{jy - 3:.0f}" class="grat"/>' + text(jx - 14, jy - 18, "Jasper Ridge: first flights", "lab b", "end"))
ox_, oy_ = R.xy(-122.50, 37.15); parts.append(text(ox_, oy_, "PACIFIC OCEAN", "lab sea", "middle"))
bx, by = R.xy(-122.13, 37.56); parts.append(text(bx, by, "SAN FRANCISCO BAY", "lab sea", "middle"))
mx, my = R.xy(-122.13, 37.29); parts.append(text(mx, my, "SANTA CRUZ MOUNTAINS", "lab mtn", "middle", -38))
cx_, cy_ = R.xy(-122.24, 37.13); parts.append(text(cx_, cy_, "CZU Lightning Complex, 2020", "lab fireL", "middle"))
parts.append(text(cx_, cy_ + 15, "86,553 acres", "lab fireS", "middle"))
ux, uy = R.xy(-121.86, 37.40); parts.append(text(ux, uy, "SCU Lightning Complex, 2020 →", "lab fireL", "end"))
parts.append(text(ux, uy + 15, "396,824 acres, most of it further east", "lab fireS", "end"))
parts.append("</g>")
parts.append(f'<rect x="0.5" y="0.5" width="{R.W - 1:.0f}" height="{R.H - 1:.0f}" class="neat"/>{ticks(R, 0.2)}')
parts.append(scalebar(R, R.W - 190, R.H - 26, 10, 37.3) + north(R.W - 40, 52))
parts.append(text(16, R.H - 16, "Contours every 100 m (bold every 500 m), from AWS Terrain Tiles. Fire perimeters: CAL FIRE; prescribed burns: NIFC.", "tick", "start"))
parts.append("</svg>")
(OUT / "region.svg").write_text("".join(parts))
print("region", len(lines), "contours", f"{R.W:.0f}x{R.H:.0f}")

# ---------------------------------------------------------------- site sheet: one survey flight at Jasper Ridge, to scale
T0 = (-122.2343, 37.4049)                     # example take-off point near the preserve centre (for scale only)
mlat = 111320; mlon = 111320 * math.cos(math.radians(T0[1]))
at = lambda dx, dy: (T0[0] + dx / mlon, T0[1] + dy / mlat)
lo0, la0 = at(-1750, -1120); lo1, la1 = at(2150, 1180)
S = Sheet(lo0, la0, lo1, la1, 15, 1200)
lines, _, _ = S.terrain(10, 50, 2.0, 3, 0)
s = [f'<svg viewBox="0 0 {S.W:.0f} {S.H:.0f}" class="mapsvg site" role="img" aria-label="Map: one survey flight at Jasper Ridge drawn to scale on 10 m contours">',
     '<defs><clipPath id="sclip"><rect width="100%" height="100%"/></clipPath></defs><g clip-path="url(#sclip)">',
     f'<rect width="100%" height="100%" class="land"/>']
s += [f'<polyline points="{p}" class="ct{" ix" if lev % 50 == 0 else ""}"/>' for lev, p in lines]
for f in jr:
    d = " ".join(S.ring(r) for poly in polys(f["geometry"]) for r in poly)
    s.append(f'<path d="{d}" class="jrbp" data-tip="Jasper Ridge Biological Preserve boundary"/>')
x0, y0 = S.xy(*T0)
r_px = S.xy(*at(1000, 0))[0] - x0
s.append(f'<circle cx="{x0:.1f}" cy="{y0:.1f}" r="{r_px:.1f}" class="ring" data-tip="1 km: how far we expect to keep the drone in sight (14 CFR 107.31)"/>')
rx_, ry_ = S.xy(*at(-700, 760)); s.append(text(rx_, ry_, "1 km: the drone stays in sight", "lab b", "middle", -42))
cx, cy, half = 420, 180, 120.5
a = S.xy(*at(cx - half, cy + half)); b_ = S.xy(*at(cx + half, cy - half))
s.append(f'<rect x="{a[0]:.1f}" y="{a[1]:.1f}" width="{b_[0] - a[0]:.1f}" height="{b_[1] - a[1]:.1f}" class="plot" data-tip="Survey plot: 2 ha of chaparral plus a 50 m buffer (241 m square)"/>')
pts = [S.xy(*at(0, 0))]
n = 18; st = 2 * half / (n - 1)
for i in range(n):
    x = cx - half + i * st
    ends = [(x, cy - half), (x, cy + half)]
    if i % 2: ends.reverse()
    pts += [S.xy(*at(*e)) for e in ends]
pts.append(S.xy(*at(0, 0)))
s.append(f'<polyline id="flightpath" points="{" ".join("%.1f,%.1f" % p for p in pts)}" class="path" data-tip="Flight path: 18 lines 14 m apart, about 7 minutes at 19 m/s"/>')
s.append(text(b_[0] + 8, a[1] + 14, "2 ha plot, 18 survey lines", "lab b", "start"))
s.append(f'<g class="to" transform="translate({x0:.1f} {y0:.1f})" data-tip="Take-off and landing (example point)"><circle r="9"/><path d="M-6,-6 L6,6 M6,-6 L-6,6"/></g>')
s.append(text(x0 - 14, y0 + 26, "take-off and landing", "lab", "end"))
s.append("</g>")
s.append(f'<rect x="0.5" y="0.5" width="{S.W - 1:.0f}" height="{S.H - 1:.0f}" class="neat"/>')
s.append(scalebar(S, S.W - 190, S.H - 26, 0.5, T0[1]) + north(S.W - 40, 52))
s.append(text(16, S.H - 16, "Contours every 10 m (bold every 50 m). Take-off point and plot placed for scale only.", "tick", "start"))
s.append("</svg>")
(OUT / "site.svg").write_text("".join(s))
print("site", len(lines), "contours", f"{S.W:.0f}x{S.H:.0f}")

# ---------------------------------------------------------------- map-table backdrop: contours only, faint, for the page background
T = Sheet(-122.45, 37.20, -122.05, 37.50, 12, 1600)
tl, tsea, _ = T.terrain(40, 200, 1.5, 3, 0)
bg = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {T.W:.0f} {T.H:.0f}" width="{T.W:.0f}" height="{T.H:.0f}">',
      f'<path d="{tsea}" fill="#e7ecee"/>']
bg += [f'<polyline points="{p}" fill="none" stroke="{"#cdbfa8" if lev % 200 == 0 else "#e2d9ca"}" stroke-width="{1.1 if lev % 200 == 0 else 0.7}"/>' for lev, p in tl]
bg.append("</svg>")
(OUT / "table.svg").write_text("".join(bg))
print("table", len(tl), "contours")
