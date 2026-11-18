"""SR-3 fabrication layer: shared geometry + export helpers.

Everything here is in the AIRCRAFT frame of ridgeback/cad/design.py (x aft from the wing-root LE, y right,
z up, mm). Each generator module (wing.py, fuselage.py, booms.py, tail.py, misc.py) exposes
    JOBS = [job_id, ...]      and      build(job_id) -> [FabPart, ...]
build.py runs the jobs in parallel, exports every part and assembles the aircraft.

Construction (prototype, 256 mm printer bed):
  * wing / tail / fuselage skins: hollow printed segments (LW-ASA), keyed on bought CF / glass tubes,
    bonded with epoxy. Every segment has 45-degree chamfered internal end flanges (bond area, no supports).
  * fittings that carry flight or landing loads: CF-PETG (printed, 100 % / 6 walls) or 6061 (CNC).
"""
from __future__ import annotations
import math, sys
from dataclasses import dataclass, field
from pathlib import Path
import numpy as np
from build123d import *
from shapely.geometry import Polygon, Point, LineString, box as sbox      # after build123d: it also exports Polygon
from shapely.geometry.polygon import orient
from shapely import affinity

HERE = Path(__file__).resolve().parent
sys.path[:0] = [str(HERE.parent), str(HERE.parent / "parts")]
import design as d            # noqa: E402  (single source of truth)

import config                 # noqa: E402  relay | scout (SR3_CONFIG env var)
OUT = config.OUT
BED = 256.0                   # Bambu X1/P1 class, 256 x 256 x 256
BED_MARGIN = 5.0
BED_XYZ = None                # rectangular bed (mini only): Prusa CORE One 250 x 220 x 270 [S]
if config.MINI:
    BED_XYZ = (250.0, 220.0, 270.0)
    BED = 250.0
NPTS = 200                    # points per section polygon (uniform arc length)

# material: density g/cm3 (as printed / as bought), process
MATERIALS = {
    "LW-ASA":  dict(rho=0.60, proc="print", note="foaming ASA (e.g. colorFabb LW-ASA / Polymaker PolyLite LW-ASA); density at ~45 % foaming (MEASURE on a test cube)"),
    "CF-PETG": dict(rho=1.30, proc="print", note="carbon-filled PETG; 6 walls, 40-100 % infill as noted"),
    "PETG":    dict(rho=1.27, proc="print", note=""),
    "ASA":     dict(rho=1.07, proc="print", note=""),
    "TPU95A":  dict(rho=1.21, proc="print", note="15 % gyroid = soft landing foot"),
    "AL6061":  dict(rho=2.70, proc="cnc", note="6061-T6, machined"),
    "G10":     dict(rho=1.85, proc="cnc", note="G10/FR4 sheet, router/waterjet from DXF"),
    "CF-plate": dict(rho=1.55, proc="cnc", note="3K twill carbon plate, router/waterjet from DXF"),
}


@dataclass
class FabPart:
    name: str                      # file basename, e.g. wing_c2_front_R
    group: str                     # wing | fuselage | boom | tail | misc
    solid: object                  # build123d Solid/Compound in the aircraft frame
    material: str
    qty: int = 1
    fill: float = 1.0              # mass factor on the CAD volume (1.0 = walls modelled; <1 = slicer infill)
    up: tuple = (0, 0, 1)          # aircraft-frame vector that points +Z on the print bed
    notes: str = ""
    dxf: object = None             # optional build123d Face/Sketch for CNC profile export
    print_notes: str = ""
    instances: list = None         # extra copies in the assembly: list of (dx, dy, dz) translations of `solid`


# ------------------------------------------------------------------ 2D sections (shapely)
_AF: dict = {}

def airfoil_poly(name: str) -> Polygon:
    """Unit-chord airfoil, CCW, LE at (0,0)."""
    if name not in _AF:
        import aerosandbox as asb
        c = np.asarray(asb.Airfoil(name).repanel(n_points_per_side=120).coordinates)
        _AF[name] = orient(Polygon(c).buffer(0), 1)
    return _AF[name]

def largest(g):
    if g is None or g.is_empty:
        return None
    if g.geom_type == "MultiPolygon" or g.geom_type == "GeometryCollection":
        polys = [p for p in g.geoms if p.geom_type == "Polygon"]
        if not polys:
            return None
        g = max(polys, key=lambda p: p.area)
    return orient(g, 1) if g.geom_type == "Polygon" else None

def af_piece(name, chord, f0=0.0, f1=1.0, offset=0.0):
    """Airfoil section (mm, chord frame) clipped to chord fractions [f0, f1], inset by `offset`."""
    P = affinity.scale(airfoil_poly(name), chord, chord, origin=(0, 0))
    if f0 > 0 or f1 < 1:
        P = largest(P.intersection(sbox(f0*chord, -chord, f1*chord, chord)))
    if P is not None and offset > 0:
        P = largest(P.buffer(-offset, join_style=1, quad_segs=6))
    if P is not None and P.area < 6.0:
        P = None
    return P

def resample(P: Polygon, n=NPTS):
    """n points, CCW, uniform arc length, starting at the boundary point facing +local-x at centroid height."""
    P = orient(P, 1)
    ext = P.exterior
    minx, miny, maxx, maxy = P.bounds
    probe = LineString([(P.centroid.x, P.centroid.y), (maxx + 1e3, P.centroid.y)])
    hit = probe.intersection(ext)
    if hit.is_empty:
        s0 = 0.0
    else:
        pts = [hit] if hit.geom_type == "Point" else list(getattr(hit, "geoms", [hit]))
        far = max(pts, key=lambda p: p.x if hasattr(p, "x") else p.centroid.x)
        s0 = ext.project(far if hasattr(far, "x") else far.centroid)
    L = ext.length
    return np.array([ext.interpolate((s0 + i*L/n) % L).coords[0] for i in range(n)])

def loft(wires, ruled=True):
    return Solid.make_loft(wires, ruled=ruled)


def _lateral(solid):
    """Side faces of a ruled polygon loft (caps are the faces with more than 4 edges)."""
    return [f for f in solid.faces() if len(f.edges()) <= 4]

def hollow(stations, sec, place, skin, flange=5.0, closed=(False, False), cap=1.6, kinks=()):
    """Generic hollow segment between stations s0 < s1, built face-by-face (no boolean: two dense lofts
    subtracted from each other cost OCC ~16 GB).
    sec(s, offset) -> shapely Polygon (local 2D) or None;  place(pts2d, s) -> build123d Wire.
    Ends open with a 45-deg chamfered inner flange of width `flange` (bond face), or closed with a cap.
    Returns (solid, is_hollow)."""
    s0, s1 = stations
    ks = [k for k in kinks if s0 + flange < k < s1 - flange]
    so = sorted({s0, s1, *[k for k in kinks if s0 < k < s1]})
    ow = [place(resample(sec(s, 0.0)), s) for s in so]
    outer = loft(ow)
    if flange <= 0:                                   # straight-through band (no flanges)
        ks = [k for k in kinks if s0 < k < s1]
        cav = [(s0, skin), *[(k, skin) for k in ks], (s1, skin)]
    else:
        cav = []
        cav += [(s0 + cap, skin)] if closed[0] else [(s0, skin + flange), (s0 + flange, skin)]
        cav += [(k, skin) for k in ks]
        cav += [(s1 - cap, skin)] if closed[1] else [(s1 - flange, skin), (s1, skin + flange)]
    polys = [(s, sec(s, o)) for s, o in cav]
    if any(p is None for _, p in polys):
        return outer, False
    iw = [place(resample(p), s) for s, p in polys]
    inner = loft(iw)
    faces = _lateral(outer) + _lateral(inner)
    def annulus(o, i):
        return (Face(o) - Face(i)).faces()[0]          # planar boolean: correct inner-wire orientation
    faces.append(Face(ow[0]) if closed[0] else annulus(ow[0], iw[0]))
    faces.append(Face(ow[-1]) if closed[1] else annulus(ow[-1], iw[-1]))
    if closed[0]:
        faces.append(Face(iw[0]))
    if closed[1]:
        faces.append(Face(iw[-1]))
    try:
        sol = Solid(Shell(faces))
        sol = sol.fix()
        ok = sol.is_valid and abs(abs(sol.volume) - (outer.volume - inner.volume)) < 0.01*outer.volume
    except Exception:
        ok = False
    if not ok:
        return outer - inner, True
    if sol.volume < 0:
        sol = Solid(sol.wrapped.Reversed())
    return sol, True


# ------------------------------------------------------------------ 3D helpers
def cyl_between(p0, p1, r):
    p0, p1 = np.array(p0, float), np.array(p1, float)
    L = float(np.linalg.norm(p1 - p0)); ax = (p1 - p0)/L
    c = Cylinder(r, L, align=(Align.CENTER, Align.CENTER, Align.MIN))
    z = np.array([0, 0, 1.0]); v = np.cross(z, ax)
    ang = math.degrees(math.acos(float(np.clip(z @ ax, -1, 1))))
    if np.linalg.norm(v) > 1e-9:
        c = c.rotate(Axis((0, 0, 0), tuple(v/np.linalg.norm(v))), ang)
    elif ax[2] < 0:
        c = c.rotate(Axis.X, 180)
    return c.moved(Location(tuple(p0)))

def tube_solid(p0, p1, od, wall):
    return cyl_between(p0, p1, od/2) - cyl_between(p0, p1, od/2 - wall)

def bx(x0, x1, y0, y1, z0, z1):
    return Box(x1 - x0, y1 - y0, z1 - z0).moved(Location(((x0 + x1)/2, (y0 + y1)/2, (z0 + z1)/2)))

def isect(a, b):
    r = a & b
    return r if (r is not None and r.volume > 1e-3) else None

def solids_of(s):
    try:
        return list(s.solids())
    except Exception:
        return [s]

def biggest(s):
    """Drop slivers left by booleans."""
    ss = solids_of(s)
    if len(ss) <= 1:
        return s
    ss = sorted(ss, key=lambda q: q.volume, reverse=True)
    keep = [q for q in ss if q.volume > 0.02*ss[0].volume]
    return keep[0] if len(keep) == 1 else Compound(keep)

def mirror_y(s):
    return s.mirror(Plane.XZ)

def mirror_part(p: FabPart, name):
    q = FabPart(**{**p.__dict__})
    q.name = name
    q.solid = mirror_y(p.solid)
    q.up = (p.up[0], -p.up[1], p.up[2])
    return q


# ------------------------------------------------------------------ export
def mesh_of(solid, tol=0.05, ang=0.2):
    import trimesh, tempfile, os
    fd, tmp = tempfile.mkstemp(suffix=".stl", dir=str(OUT)); os.close(fd)
    try:
        export_stl(solid, tmp, tolerance=tol, angular_tolerance=ang)
        m = trimesh.load(tmp)
    finally:
        os.remove(tmp)
    return m

def _rot_to_z(u):
    u = np.array(u, float); u /= np.linalg.norm(u)
    z = np.array([0, 0, 1.0]); v = np.cross(u, z); c = float(u @ z)
    if np.linalg.norm(v) < 1e-9:
        return np.eye(3) if c > 0 else np.diag([1, -1, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx*(1/(1 + c))

def print_orient(m, up):
    """Rotate mesh so `up` -> +Z, then yaw for the smallest footprint; centre it on the bed.
    Tries the preferred `up` first, then the 6 axis directions. Returns (mesh, dims, fits, up_used)."""
    lim = BED - 2*BED_MARGIN
    cands = [tuple(up)] + [c for c in [(0, 0, 1), (0, 0, -1), (1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0)] if c != tuple(up)]
    results = []
    for c in cands:
        R = _rot_to_z(c)
        v = m.vertices @ R.T
        best = None
        for yaw in np.radians(np.arange(0, 180, 2.0)):
            Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
            w = v[:, :2] @ Rz[:2, :2].T
            dxy = w.max(0) - w.min(0)
            if best is None or max(dxy) < best[0]:
                best = (max(dxy), Rz, dxy)
        dz = v[:, 2].max() - v[:, 2].min()
        dims = np.array([best[2][0], best[2][1], dz])
        fits = bool((dims <= lim).all())
        if BED_XYZ is not None:
            bx_, by_, bz_ = (v - 2*BED_MARGIN for v in BED_XYZ)
            fits = bool(max(dims[:2]) <= bx_ and min(dims[:2]) <= by_ and dz <= bz_)
        results.append((c, best[1] @ R, dims, fits))
        if fits:
            break
    ok = [r for r in results if r[3]]
    c, R, dims, fits = ok[0] if ok else min(results, key=lambda r: r[2].max())
    mm = m.copy()
    mm.vertices = m.vertices @ R.T
    lo, hi = mm.vertices.min(0), mm.vertices.max(0)
    mm.vertices -= np.array([(lo[0] + hi[0])/2 - BED/2, (lo[1] + hi[1])/2 - BED/2, lo[2]])
    return mm, dims, fits, c
