"""Scout renders (matplotlib): (i) full iso with see-through skins, (ii) fuselage cutaway, (iii) boom/nacelle close-up,
(iv) wing-break close-up, (v) top-down harness map. Colours by subsystem (power red, signal blue, RF purple, payload
green, lights yellow); structure grey/transparent; Scout printed mounts orange."""
import math
from pathlib import Path
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.lines import Line2D
from core import d
import f_wing as FW
import f_fuse as FF
import scout as S

SUBCOL = {"power": (0.85, 0.12, 0.12), "signal": (0.15, 0.45, 0.95), "rf": (0.65, 0.20, 0.85), "payload": (0.10, 0.70, 0.30),
          "lights": (0.95, 0.80, 0.10)}
MOUNT = (0.98, 0.55, 0.10)
NEW_MOUNTS = ("elec_", "pay_altum", "boom_esc_cradle", "boom_grommet", "fuse_pusher_esc_mount")


def style(it, view):
    """(rgb, alpha) or None to hide."""
    n, g, k = it["name"], it["group"], it["kind"]
    if k == "harness" or g.startswith("elec_") and it.get("sub"):
        return SUBCOL.get(it["sub"], (0.5, 0.5, 0.5)), 1.0
    if n.startswith(NEW_MOUNTS):
        return MOUNT, 0.95
    if g == "prop":
        return None
    if g == "battery":
        return (0.90, 0.72, 0.20), 0.55 if view != "iso" else 0.45
    if g == "motor":
        return (0.35, 0.40, 0.50), 0.8
    if g in ("tube", "glass"):
        return (0.10, 0.10, 0.12), (0.30 if view in ("nacelle", "break") else 0.18)
    if g in ("wing", "fuselage", "tail"):
        if view == "iso":
            return (0.70, 0.72, 0.76), 0.05
        return (0.70, 0.72, 0.76), (0.20 if view == "break" else 0.10)
    if g == "boom":
        return (0.55, 0.57, 0.60), (0.12 if view == "iso" else 0.25)
    return (0.6, 0.6, 0.6), 0.2


def _slice(m, lo, hi, extra=()):
    """Clip a trimesh to the box lo..hi (+ extra (normal, origin) half-spaces kept on the +normal side)."""
    from trimesh.intersections import slice_mesh_plane
    planes = [((1, 0, 0), (lo[0], 0, 0)), ((-1, 0, 0), (hi[0], 0, 0)), ((0, 1, 0), (0, lo[1], 0)), ((0, -1, 0), (0, hi[1], 0)),
              ((0, 0, 1), (0, 0, lo[2])), ((0, 0, -1), (0, 0, hi[2]))] + list(extra)
    b0, b1 = m.bounds
    for n, o in planes:
        n = np.array(n, float); o = np.array(o, float)
        corners = np.array([[x, y, z] for x in (b0[0], b1[0]) for y in (b0[1], b1[1]) for z in (b0[2], b1[2])])
        dd = (corners - o) @ n
        if (dd >= 0).all():
            continue
        if (dd < 0).all():
            return None
        try:
            m = slice_mesh_plane(m, n, o, cap=False)
        except Exception:
            return None
        if m is None or len(m.faces) == 0:
            return None
        b0, b1 = m.bounds
    return m


def _draw(ax, items, meshes, view, keep=None, drop_tri=None, light=(0.3, -0.4, 0.8), box=None, extra=None):
    light = np.array(light, float); light /= np.linalg.norm(light)
    tris, cols = [], []
    for it, m in zip(items, meshes):
        if m is None:
            continue
        st = style(it, view)
        if st is None or (keep is not None and not keep(it)):
            continue
        if box is not None:
            m = _slice(m, box[0], box[1], extra(it) if extra else ())
            if m is None:
                continue
        T = m.triangles; N = m.face_normals
        sel = np.ones(len(T), bool)
        if drop_tri is not None:
            sel = ~drop_tri(it, T.mean(1))
        if not sel.any():
            continue
        T, N = T[sel], N[sel]
        c, a = st
        sh = 0.45 + 0.55*np.clip(np.abs(N @ light), 0, 1)
        rgba = np.concatenate([np.clip(np.array(c)[None, :]*sh[:, None], 0, 1), np.full((len(T), 1), a)], 1)
        tris.append(T); cols.append(rgba)
    T = np.concatenate(tris); C = np.concatenate(cols)
    ax.add_collection3d(Poly3DCollection(T, facecolors=C, edgecolor="none"))
    return T


def _frame(ax, lo, hi, el, az, title, zoom=1.0):
    lo, hi = np.array(lo, float), np.array(hi, float)
    ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2]); ax.set_box_aspect(hi - lo, zoom=zoom)
    ax.view_init(el, az); ax.set_axis_off(); ax.set_title(title, fontsize=11)


def _legend(fig):
    h = [Line2D([0], [0], color=c, lw=6, label=k) for k, c in SUBCOL.items()] + [Line2D([0], [0], color=MOUNT, lw=6, label="Scout printed mounts")]
    fig.legend(handles=h, loc="lower center", ncol=6, fontsize=9, frameon=False)


def _in_box(lo, hi):
    lo, hi = np.array(lo), np.array(hi)
    return lambda P: np.all((P >= lo) & (P <= hi), axis=1)


def render_all(items, meshes, outdir, hres):
    outdir = Path(outdir); outdir.mkdir(parents=True, exist_ok=True)
    # (i) full aircraft
    fig = plt.figure(figsize=(16, 10), dpi=130); ax = fig.add_subplot(111, projection="3d")
    T = _draw(ax, items, meshes, "iso")
    lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
    _frame(ax, lo, hi, 26, -122, "SR-3 Scout v1 (PROVISIONAL): electronics + harness, skins 5 % opaque", zoom=1.45)
    _legend(fig); fig.savefig(outdir / "scout_1_iso.png", bbox_inches="tight"); plt.close(fig)
    # (ii) fuselage cutaway: drop the +y half of the fuselage shells, hide the wing and the right-side boom items
    lo, hi = (-800, -150, -235), (330, 150, 70)
    inb = _in_box(lo, hi)
    def ex2(it):
        if it["group"] == "fuselage" and not it["name"].startswith(("fuse_battery_tray", "fuse_avionics_tray", "fuse_frame", "fuse_pusher", "fuse_battery_stop")):
            return [((0, -1, 0), (0, 0, 0))]
        return []
    fig = plt.figure(figsize=(16, 9), dpi=130); ax = fig.add_subplot(111, projection="3d")
    _draw(ax, items, meshes, "cut", keep=lambda it: it["group"] not in ("wing", "tube", "glass", "motor"), box=(lo, hi), extra=ex2)
    _frame(ax, lo, hi, 18, 72, "Fuselage cutaway (+y half of the shell removed, wing hidden): trays, FC isolator, PDB plate, battery, Altum-PT, DLS 2", zoom=1.5)
    _legend(fig); fig.savefig(outdir / "scout_2_fuselage_cutaway.png", bbox_inches="tight"); plt.close(fig)
    # (iii) boom / nacelle close-up, right front station, inboard nacelle half removed, seen from inboard
    mx = d.FRONT_MOTOR_X
    lo, hi = (mx - 140, 545, -110), (mx + 190, 660, 60)
    inb = _in_box(lo, hi)
    fig = plt.figure(figsize=(16, 9), dpi=130); ax = fig.add_subplot(111, projection="3d")
    _draw(ax, items, meshes, "nacelle", keep=lambda it: not it["name"].startswith("boom_nacelle_half_in"), box=(lo, hi))
    _frame(ax, lo, hi, 16, -70, "Right front nacelle (inboard half removed): ALPHA 60A in its cradle, top inlet / belly outlet slots, 12 AWG feed out of the tube, motor leads to the pylon", zoom=1.45)
    _legend(fig); fig.savefig(outdir / "scout_3_nacelle.png", bbox_inches="tight"); plt.close(fig)
    # (iv) wing break close-up
    lo, hi = (60, 540, -50), (300, 660, 45)
    inb = _in_box(lo, hi)
    fig = plt.figure(figsize=(16, 9), dpi=130); ax = fig.add_subplot(111, projection="3d")
    _draw(ax, items, meshes, "break", box=(lo, hi))
    _frame(ax, lo, hi, 30, -58, "Wing break y = +600: JST-GH connector pocket aft of the HP_AFT block, servo-lead cell, power trunk dropping to the boom slot", zoom=1.4)
    _legend(fig); fig.savefig(outdir / "scout_4_wing_break.png", bbox_inches="tight"); plt.close(fig)
    # (v) top-down harness map (2D)
    harness_map(items, outdir / "scout_5_harness_map.png", hres)


def harness_map(items, path, hres):
    fig, ax = plt.subplots(figsize=(18, 11), dpi=120)
    ys = np.linspace(-d.SPAN/2, d.SPAN/2, 200)
    ax.plot([d.le_x(y) for y in ys], ys, color="0.6", lw=1); ax.plot([d.le_x(y) + d.chord_at(y) for y in ys], ys, color="0.6", lw=1)
    xs = np.linspace(FF.BOUNDS[0], FF.BOUNDS[-1], 200)
    w = np.array([FF.params(x)[0]/2 for x in xs])
    ax.fill_between(xs, -w, w, color="0.9", zorder=0); ax.plot(xs, w, color="0.6", lw=1); ax.plot(xs, -w, color="0.6", lw=1)
    for s in (1, -1):
        ax.plot([d.BOOM_FRONT_X, d.BOOM_REAR_X], [s*d.BOOM_Y]*2, color="0.25", lw=4, alpha=0.5)
        for mx in (d.FRONT_MOTOR_X, d.REAR_MOTOR_X):
            ax.add_patch(plt.Circle((mx, s*d.BOOM_Y), d.LIFT_PROP_D/2, fill=False, ls=":", color="0.7"))
    for x in FF.FRAMES.values():
        ax.axvline(x, ymin=0.45, ymax=0.55, color="0.75", lw=0.8)
    for it in items:
        if it["kind"] == "harness":
            P = np.array(it["pts"])
            ax.plot(P[:, 0], P[:, 1], color=SUBCOL[it["sub"]], lw=max(1.0, it["dia"]*0.55), solid_capstyle="round", alpha=0.85)
    for it in S.items():
        if it["key"] == "caps":
            continue
        x, y, z = it["c"]; lx, ly = it["dims"][0], it["dims"][1]
        ax.add_patch(plt.Rectangle((x - lx/2, y - ly/2), lx, ly, color=SUBCOL.get(it["sub"], (0.5, 0.5, 0.5)), alpha=0.35, zorder=3))
        if not it["key"].startswith(("esc", "servo", "strobe", "ant")) or it["key"] in ("esc0", "esc_push", "servo_ail_R", "servo_rv_R", "ant_telem"):
            ax.annotate(it["key"], (x, y), fontsize=7, ha="center", va="center", zorder=5)
    B = FF.BAT
    ax.add_patch(plt.Rectangle((B["x"] - B["l"]/2, -B["w"]/2), B["l"], B["w"], color=(0.9, 0.72, 0.2), alpha=0.35, zorder=2)); ax.annotate("battery 12S5P", (B["x"], 0), fontsize=8, ha="center")
    ax.annotate("JST-GH break\nconnector", (226, 600), xytext=(300, 760), fontsize=8, arrowprops=dict(arrowstyle="->"))
    ax.annotate("trunk slot +\nTPU grommet", (145, 585), xytext=(-120, 820), fontsize=8, arrowprops=dict(arrowstyle="->"))
    ax.set_aspect("equal"); ax.set_xlabel("x aft (mm)"); ax.set_ylabel("y right (mm)")
    ax.set_title(f"SR-3 Scout v1 harness map (top view, PROVISIONAL). Line width ~ bundle diameter. Harness {hres['total_g']:.0f} g [C]")
    h = [Line2D([0], [0], color=c, lw=4, label=k) for k, c in SUBCOL.items() if k != "lights"]
    ax.legend(handles=h, loc="lower right")
    ax.set_xlim(-800, 1450); ax.set_ylim(-d.SPAN/2 - 40, d.SPAN/2 + 40)
    fig.savefig(path, bbox_inches="tight"); plt.close(fig)
