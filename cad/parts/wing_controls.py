"""Flaps (inner, y 180..1000) and ailerons (outer, y 1050..1700): the wing aft of the 75 % hinge line,
2 mm hinge gap, both sides. Same molded construction as the wing. Units mm."""
from build123d import *
import design as d
import wing as W

NAME = "wing_controls"
MATERIAL = "CF"
INFILL = 1.0
QTY = 1
MIRROR = False

def make():
    halves = []
    for s in (1, -1):
        inner, outer = W._half(s)
        halves.append(inner.fuse(outer).clean())
    full = halves[0].fuse(halves[1]).clean()
    pieces = []
    for s in (1, -1):
        for span in (d.FLAP_SPAN, d.AILERON_SPAN):
            box = W._control_cut(span[0] + 2, span[1] - 2, s).moved(Location((3, 0, 0)))   # 2 mm gaps each side
            pieces.append(full & box)
    return Compound(pieces)

def mass_g():
    return sum(p.area for p in make().solids())/1e6*W.AREAL_G_M2

def placements():
    return [Location((0, 0, 0))]

if __name__ == "__main__":
    from common import export
    m = make(); print(f"{len(m.solids())} surfaces, {mass_g():.0f} g")
    export(m, NAME)
