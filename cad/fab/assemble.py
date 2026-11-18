"""SR-3 fabrication assembly: places every exported part (+ instances) and purchased envelopes, checks mass/CG and
interferences, writes SR3_assembly.step (one named component per part), a GLB, renders, BOM and report.
Reads fab/out/meta/*.json + fab/out/step/*.step written by build.py."""
import json, math, sys, time, csv, zipfile
from pathlib import Path
import numpy as np

FAB = Path(__file__).resolve().parent
sys.path[:0] = [str(FAB), str(FAB.parent), str(FAB.parent / "parts")]
from build123d import *
import core
from core import d, cyl_between, bx, OUT
import f_wing as FW, f_fuse as FF, f_boom as FB, f_tail as FT

COL = {"wing": (0.80, 0.82, 0.85), "fuselage": (0.80, 0.82, 0.85), "tail": (0.80, 0.82, 0.85), "boom": (0.15, 0.16, 0.18),
       "misc": (0.95, 0.45, 0.10), "tube": (0.05, 0.05, 0.06), "glass": (0.85, 0.85, 0.70), "motor": (0.20, 0.45, 0.85),
       "prop": (0.55, 0.57, 0.60), "battery": (0.90, 0.70, 0.15), "avionics": (0.20, 0.65, 0.35), "payload": (0.95, 0.45, 0.10)}
CNC_COL = (0.10, 0.10, 0.12)

# ------------------------------------------------------------------ printed / CNC parts
def load_parts():
    metas = [json.loads(p.read_text()) for p in sorted((OUT / "meta").glob("*.json")) if not p.name.startswith("_job_")]
    return metas

# ------------------------------------------------------------------ purchased items: (name, group, solid, mass_g, qty-for-BOM, bom key)
def tube(p0, p1, od, wall):
    return cyl_between(p0, p1, od/2) - cyl_between(p0, p1, od/2 - wall)

def purchased():
    P = []
    def add(name, grp, solid, g, cg=None):
        P.append(dict(name=name, group=grp, solid=solid, mass_g=g, cg=cg))
    rho = 1.55
    def tmass(p0, p1, od, wall):
        L = float(np.linalg.norm(np.subtract(p1, p0))); return math.pi*(od**2 - (od - 2*wall)**2)/4*L/1e3*rho, L
    cut = []                                               # cut list: (item, od, wall, length, qty)
    # spars
    zs, xs = FW.ZS, FW.SPAR_X
    a, b = FW.SPAR_PARTS["inner"]; m, Lc = tmass((xs, a, zs), (xs, b, zs), *d.INNER_SPAR)
    add("tube_spar_centre_24x2", "tube", tube((xs, a, zs), (xs, b, zs), *d.INNER_SPAR), m); cut.append(("CF tube spar, centre", 24, 2, Lc, 1))
    for s in (1, -1):
        for key, (od, w), lab in (("outer", d.OUTER_SPAR, "outer"), ("tip", FW.TIP_SPAR, "tip")):
            a, b = FW.SPAR_PARTS[key]; p0, p1 = (xs, s*a, zs), (xs, s*b, zs)
            m, Lc = tmass(p0, p1, od, w); add(f"tube_spar_{lab}_{'R' if s > 0 else 'L'}", "tube", tube(p0, p1, od, w), m)
    cut += [("CF tube spar, outer (wing rod)", 18, 1.5, FW.SPAR_PARTS["outer"][1] - FW.SPAR_PARTS["outer"][0], 2),
            ("CF tube spar, tip", 12, 1.0, FW.SPAR_PARTS["tip"][1] - FW.SPAR_PARTS["tip"][0], 2)]
    # booms
    for s in (1, -1):
        p0, p1 = (d.BOOM_FRONT_X, s*d.BOOM_Y, d.BOOM_Z), (d.BOOM_REAR_X, s*d.BOOM_Y, d.BOOM_Z)
        m, Lc = tmass(p0, p1, *d.BOOM_TUBE); add(f"tube_boom_{'R' if s > 0 else 'L'}", "tube", tube(p0, p1, *d.BOOM_TUBE), m)
    cut.append(("CF tube boom", 30, 1.5, Lc, 2))
    # rear rods (6 mm solid)
    xr = FW.ROD_C_X; zr = 0.5*(sum(FW.z_bounds(0, xr))/2 + sum(FW.z_bounds(558, xr))/2)
    RR, LR = (6.0, 1.0), (8.0, 1.0)                       # pultruded CF tubes 6x4 (rear rods) and 8x6 (legs)
    a6 = math.pi*(RR[0]**2 - (RR[0] - 2*RR[1])**2)/4; a8 = math.pi*(LR[0]**2 - (LR[0] - 2*LR[1])**2)/4
    add("rod_rear_centre", "tube", tube((xr, -558, zr), (xr, 558, zr), *RR), a6*1116/1e3*rho)
    (xa, ya), (xb, yb) = FW.ROD_O
    za, zb = sum(FW.z_bounds(ya, xa))/2, sum(FW.z_bounds(yb, xb))/2
    Lo = math.dist((xa, ya, za), (xb, yb, zb))
    for s in (1, -1):
        add(f"rod_rear_outer_{'R' if s > 0 else 'L'}", "tube", tube((xa, s*ya, za), (xb, s*yb, zb), *RR), a6*Lo/1e3*rho)
    cut += [("CF tube rear rod, centre", 6, 1.0, 1116, 1), ("CF tube rear rod, outer", 6, 1.0, Lo, 2)]
    # leg rods
    for i, (dx, dy, _) in enumerate([(0, 0, 0)] + FB._four()):
        for sx in (-1, 1):
            x = d.FRONT_MOTOR_X + dx + sx*FB.ROD["dx"]; y = d.BOOM_Y + dy
            add(f"rod_leg_{i}_{sx}", "tube", tube((x, y, FB.ROD["bot"]), (x, y, FB.ROD["top"]), *LR), a8*(FB.ROD["top"] - FB.ROD["bot"])/1e3*rho)
    cut.append(("CF tube landing leg", 8, 1.0, FB.ROD["top"] - FB.ROD["bot"], 8))
    # tail glass spars
    for s in (1, -1):
        a_, b_ = FT.line(FT.SPAR["f"], FT.SPAR["u0"], FT.SPAR["u1"])
        sol = tube(a_, b_, FT.SPAR["od"], FT.SPAR["wall"]); sol = sol if s > 0 else sol.mirror(Plane.XZ)
        add(f"tube_tail_glass_{'R' if s > 0 else 'L'}", "glass", sol, math.pi*(100 - 64)/4*(FT.SPAR["u1"] - FT.SPAR["u0"])/1e3*1.9)
    cut += [("Fibreglass tube tail spar", 10, 1.0, FT.SPAR["u1"] - FT.SPAR["u0"], 2),
            ("CF rod aileron joiner", 3, None, FW.AIL[1] - FW.AIL[0] - 4, 2), ("Fibreglass rod ruddervator joiner", 3, None, FT.RV_U[1] - FT.RV_U[0], 2),
            ("Glass-wrap shim on outer spar end (in 24 tube)", 18, 0.9, 150, 2), ("Shim on tip spar end (in 18 tube)", 12, 1.4, 150, 2)]
    # motors, props, ESCs
    zseat = FB.PY["ztop"] + FB.PLATE["t"]
    for i, (dx, dy, _) in enumerate([(0, 0, 0)] + FB._four()):
        x, y = d.FRONT_MOTOR_X + dx, d.BOOM_Y + dy
        add(f"motor_U8II_{i}", "motor", Cylinder(44, 36, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((x, y, zseat))), 272)
        add(f"prop_28in_{i}", "prop", Cylinder(d.LIFT_PROP_D/2, 4).moved(Location((x, y, zseat + 46))), 75)
        add(f"esc_alpha60_{i}", "avionics", bx(x + 32, x + 105, y - 11, y + 11, d.BOOM_Z - 58, d.BOOM_Z - 20), 73)   # on its side under the tube (MEASURE ESC)
    add("motor_pusher", "motor", Cylinder(25, 55, rotation=(0, 90, 0), align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((d.PUSHER_X + 5, 0, d.PUSHER_Z))), 330)
    add("prop_pusher_16in", "prop", Cylinder(d.PUSH_PROP_D/2, 4, rotation=(0, 90, 0)).moved(Location((d.PUSHER_X + 72, 0, d.PUSHER_Z))), 60)
    add("esc_pusher", "avionics", None, 100, cg=(700, 0, -100))
    # battery + avionics + payload
    B = FF.BAT
    add("battery_12S5P_P45B", "battery", bx(B["x"] - B["l"]/2, B["x"] + B["l"]/2, -B["w"]/2, B["w"]/2, FF.TRAY["ztop"], FF.TRAY["ztop"] + B["h"]), d.BATTERY["g"])
    zA, zB = FF.AV_TRAYS["A"], FF.AV_TRAYS["B"]
    av = [("jetson_ark_fc", (116, 72, 23), (-400, 0, zA + 11.5 + 3), 166), ("manet_radio", (60, 40, 15), (-300, 45, zA + 7.5 + 3), 52),
          ("gnss_rx_x2", (50, 40, 12), (-300, -45, zA + 6 + 3), 42), ("satcom_rockblock", (45, 45, 15), (-330, 0, -20), 50),
          ("pdb_power_module", (80, 60, 20), (-400, 0, zB + 10 + 3), 120), ("lidar_lw20", (40, 40, 15), (FF.LIDAR_X, 0, zB - 7.5 - 3 - FF.AV_T), 19),
          ("adsb_remote_id", (40, 40, 15), (-440, 0, zB - 7.5 - 3 - FF.AV_T), 24), ("mesh_vhf_scanner", (60, 30, 15), (-360, 0, zB - 7.5 - 3 - FF.AV_T), 70)]
    for n, dims, c, g in av:
        add(n, "avionics", Box(*dims).moved(Location(c)), g)
    R = d.RELAY
    zdup = -d.FUSE_H + 40
    add("relay_duplexer_TMND3", "payload", Box(R["duplexer"]["x"], R["duplexer"]["y"], R["duplexer"]["z"]).moved(Location((R["x"], 0, zdup))), R["duplexer"]["g"])
    zr_ = FF.TRAY["ztop"] - FF.TRAY["t"] - 20 - R["radio"]["z"]/2
    add("relay_repeater_radio", "payload", Box(R["radio"]["x"], R["radio"]["y"], R["radio"]["z"]).moved(Location((R["x"], 0, zr_))), R["radio"]["g"])
    import pay_turret_mount as TM
    add("eoir_turret_HD25", "payload", TM.make_payload().moved(TM.placements()[0]), 350 + 54)
    zb = FF.params(d.NADIR_CAM_X)[2] - FF.params(d.NADIR_CAM_X)[1]
    add("nadir_thermal_boson", "payload", Box(45, 45, 40).moved(Location((d.NADIR_CAM_X + 40, 0, zb + 25))), 55)
    for s in (1, -1):
        zu = FW.z_bounds(400, 200)[1]
        add(f"gnss_antenna_{'R' if s > 0 else 'L'}", "avionics", Cylinder(30, 15, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location((200, s*400, zu + 4))), 40)
    # servos (aileron bay + ruddervator bay)
    c, _, xle, _ = FW.station(FW.SERVO_Y); xc = 0.5*(xle + FW.SPLIT*c + FW.HINGE_X - FW.AIL_GAP/2)
    for s in (1, -1):
        zl = FW.z_bounds(FW.SERVO_Y, xc)[0]
        add(f"servo_aileron_{'R' if s > 0 else 'L'}", "avionics", Box(23, 32, 10).moved(Location((xc, s*FW.SERVO_Y, zl + 6))), 30)
        loc = FT.frame_at(FT.SERVO["u"], FT.SERVO["f"])
        sv = Box(23, 32, 10).moved(loc*Location((0, 0, -3)))
        add(f"servo_ruddervator_{'R' if s > 0 else 'L'}", "avionics", sv if s > 0 else sv.mirror(Plane.XZ), 30)
    # lumps without geometry
    add("wiring_harness", "lump", None, 0.04*d.MTOW_G, cg=(60, 0, -90))
    add("hardware_inserts_bolts", "lump", None, 220, cg=(d.CG_X, 0, -40))
    add("epoxy_bond_lines", "lump", None, 300, cg=(150, 0, 0))     # [A] ~120 bonded joints
    add("strobes_nav_lights", "lump", None, 40, cg=(400, 0, 0))
    add("vhf_element_coax", "lump", None, 80, cg=(d.TAIL_LE_X + 30, 0, d.BOOM_Z + 300))
    return P, cut


def main():
    import config
    if config.SCOUT:                                      # SR3_CONFIG=scout: separate assembly, relay path below untouched
        import assemble_scout
        return assemble_scout.main()
    t0 = time.time()
    metas = load_parts()
    print(f"{len(metas)} part files")
    items = []                                              # placed solids for the STEP assembly
    mass_rows = []
    for m in metas:
        s = import_step(str(OUT / "step" / f"{m['name']}.step"))
        col = CNC_COL if m["process"] == "cnc" else COL.get(m["group"], (0.7, 0.7, 0.7))
        for k, (dx, dy, dz) in enumerate(m["instances"]):
            si = s.moved(Location((dx, dy, dz)))
            si.label = m["name"] if len(m["instances"]) == 1 else f"{m['name']}#{k+1}"
            si.color = Color(*col)
            items.append(dict(name=si.label, part=m["name"], group=m["group"], solid=si, kind=m["process"], col=col))
            c = np.array(m["center"]) + [dx, dy, dz]
            mass_rows.append((m["name"], m["group"], m["mass_g"], c))
    if config.MINI:
        from assemble_mini import purchased_mini
        P, cutlist = purchased_mini()
    else:
        P, cutlist = purchased()
    for p in P:
        if p["solid"] is not None:
            p["solid"].label = p["name"]; p["solid"].color = Color(*COL.get(p["group"], (0.5, 0.5, 0.5)))
            items.append(dict(name=p["name"], part=p["name"], group=p["group"], solid=p["solid"], kind="bought", col=COL.get(p["group"], (0.5, 0.5, 0.5))))
            c = p["solid"].center(); cg = np.array([c.X, c.Y, c.Z])
        else:
            cg = np.array(p["cg"])
        mass_rows.append((p["name"], p["group"], p["mass_g"], cg))
    M = sum(r[2] for r in mass_rows)
    cg = sum(r[2]*r[3] for r in mass_rows)/M
    mb = d.BATTERY["g"]
    xb_now = FF.BAT["x"]
    x_need = xb_now + (d.CG_X - cg[0])*M/mb
    print(f"mass {M/1000:.2f} kg, CG ({cg[0]:.0f}, {cg[1]:.1f}, {cg[2]:.0f}); battery centre x for CG {d.CG_X:.0f}: {x_need:.0f} (now {xb_now:.0f})")
    globals()["X_BAT_NEED"] = x_need

    # ---------------- interference: bought vs everything, boom/fuselage internals vs shells (bbox prefilter)
    def bb(s):
        b = s.bounding_box(); return np.array([b.min.X, b.min.Y, b.min.Z]), np.array([b.max.X, b.max.Y, b.max.Z])
    boxes = [bb(it["solid"]) for it in items]
    press_fit = ("boom_foot", "rod_leg")              # TPU feet are a 0.1 mm press fit on the rods (intended)
    hits = []
    n = len(items)
    for i in range(n):
        for j in range(i + 1, n):
            a, b = items[i], items[j]
            if a["kind"] != "bought" and b["kind"] != "bought" and a["group"] == b["group"] and a["group"] in ("wing", "tail", "fuselage"):
                continue                                    # bonded neighbours share faces by design
            if a["part"] == b["part"]:
                continue
            if any(a["part"].startswith(p) for p in press_fit) and any(b["part"].startswith(p) for p in press_fit):
                continue
            (l1, h1), (l2, h2) = boxes[i], boxes[j]
            if np.any(h1 < l2 - 0.01) or np.any(h2 < l1 - 0.01):
                continue
            try:
                r = a["solid"] & b["solid"]
                v = r.volume if r is not None else 0.0
            except Exception:
                v = -1
            if v > 20 or v < 0:
                hits.append((a["name"], b["name"], round(v, 1)))
    print(f"interference pairs > 20 mm3: {len(hits)}")
    for h in hits[:60]:
        print("   ", h)

    # ---------------- exports
    comp = Compound(label=("Ridgeback smaller own design quadplane" if config.MINI else "SR-3 Ridgeback relay quadplane"), children=[it["solid"] for it in items])
    export_step(comp, str(OUT / "SR3_assembly.step"))
    groups = {"wing": ("wing",), "fuselage": ("fuselage", "misc"), "booms": ("boom",), "tail": ("tail",),
              "purchased": ("tube", "glass", "motor", "prop", "battery", "avionics", "payload")}
    for gname, gs in groups.items():
        ch = [it["solid"] for it in items if it["group"] in gs]
        export_step(Compound(label=f"SR-3 {gname}", children=ch), str(OUT / f"SR3_sub_{gname}.step"))
    try:
        export_gltf(comp, str(OUT / "SR3_assembly.glb"), binary=True, linear_deflection=0.5, angular_deflection=0.5)
    except Exception as e:
        print("gltf export failed:", e)
    render_views(items)
    write_bom(metas, P, cutlist, M, cg, hits)
    with zipfile.ZipFile(OUT / "SR3_print_pack.zip", "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted((OUT / "stl").glob("*.stl")) + sorted((OUT / "dxf").glob("*.dxf")):
            z.write(f, f"{f.parent.name}/{f.name}")
        for f in ("BOM.md", "BOM_printed_parts.csv"):
            z.write(OUT / f, f)
    with zipfile.ZipFile(OUT / "SR3_fusion360_step_pack.zip", "w", zipfile.ZIP_DEFLATED) as z:
        for f in sorted((OUT / "step").glob("*.step")):
            z.write(f, f"step/{f.name}")
        for f in sorted(OUT.glob("SR3_sub_*.step")):
            z.write(f, f.name)
    (OUT / "assembly_summary.json").write_text(json.dumps(dict(mass_g=M, cg=list(map(float, cg)), x_bat_need=X_BAT_NEED, hits=hits), indent=1))
    print(f"assembly done in {time.time() - t0:.0f}s")


def render_views(items):
    import trimesh
    import matplotlib; matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    tris, cols = [], []
    for it in items:
        try:
            m = core.mesh_of(it["solid"], 0.6, 0.6)
        except Exception:
            continue
        if it["group"] == "prop":
            alpha = 0.25
        else:
            alpha = 1.0
        c = np.array(it["col"])
        tris.append(m.triangles); cols.append((np.repeat(c[None], len(m.triangles), 0), m.face_normals, alpha))
    T = np.concatenate(tris)
    for name, el, az in (("iso", 24, -128), ("top", 89.9, -90), ("side", 0, -90), ("front", 0, 180)):
        light = np.array([np.cos(np.radians(el))*np.cos(np.radians(az)), np.cos(np.radians(el))*np.sin(np.radians(az)), np.sin(np.radians(el))]) + [0.3, -0.2, 0.7]
        light /= np.linalg.norm(light)
        fc = []
        for c, nrm, a in cols:
            sh = 0.35 + 0.65*np.clip(np.abs(nrm @ light), 0, 1)
            rgba = np.concatenate([np.clip(c*sh[:, None], 0, 1), np.full((len(c), 1), a)], 1); fc.append(rgba)
        fc = np.concatenate(fc)
        fig = plt.figure(figsize=(12, 8), dpi=120); ax = fig.add_subplot(111, projection="3d")
        ax.add_collection3d(Poly3DCollection(T, facecolors=fc, edgecolor="none"))
        lo, hi = T.reshape(-1, 3).min(0), T.reshape(-1, 3).max(0)
        ax.set_xlim(lo[0], hi[0]); ax.set_ylim(lo[1], hi[1]); ax.set_zlim(lo[2], hi[2]); ax.set_box_aspect(hi - lo)
        ax.view_init(el, az); ax.set_axis_off(); ax.set_title(f"SR-3 fabrication assembly - {name}")
        fig.savefig(OUT / f"SR3_{name}.png", bbox_inches="tight"); plt.close(fig)


RATE = {"LW-ASA": 10.0, "ASA": 14.0, "CF-PETG": 14.0, "PETG": 15.0, "TPU95A": 6.0}     # cm3/h of deposited material [A] (0.4 mm nozzle)
PRICE = {"LW-ASA": 60.0, "ASA": 30.0, "CF-PETG": 40.0, "PETG": 22.0, "TPU95A": 35.0}   # $/kg [A]

BOUGHT = [  # item, qty, unit $, source (SR-3 relay config; prices from docs/BOM_COST.md / research/FACTS.md)
    ("T-Motor U8 II KV100 lift motor", 4, 310, "[S63] verified"), ("T-Motor G28x9.2 CF prop (2 CW + 2 CCW) + 1 spare pair", 6, 120, "[A] price not found"),
    ("T-Motor ALPHA 60A 12S ESC", 4, 110, "[S65]"), ("Pusher: AT4130-class motor + 80 A ESC + 16x10 folding prop", 1, 350, "[S82] L / [A]"),
    ("ARKV6X flight controller", 1, 400, "[S72]"), ("ARK Jetson PAB carrier", 1, 800, "[S71] verified"), ("Jetson Orin NX 16 GB", 1, 700, "[A]"),
    ("12S5P Molicel P45B pack (60 cells + BMS + welding)", 1, 800, "[A] cell price not found"),
    ("VHF repeater radio + controller (Part 90, agency-programmed)", 1, None, "QUOTE NEEDED"), ("Telewave TMND3 duplexer", 1, None, "QUOTE NEEDED [S126]"),
    ("EO/IR turret (Trillium HD25-LV class)", 1, 12000, "[A]; prototype can fly a Boson-only nadir camera first"),
    ("Nadir thermal (Boson+ 640 + interface)", 1, 3000, "[A]"), ("MANET datalink pair (Silvus/Doodle class)", 1, 10000, "[A] quote needed; ELRS/Wi-Fi for first flights"),
    ("RockBLOCK 9704 + activation", 1, 319, "[S89][S90]"), ("Dual GNSS (2x ARK MOSAIC-X5) + 2 antennas", 1, 2680, "[S92]"),
    ("ADS-B In (pingRX Pro)", 1, 349, "[S91]"), ("Remote ID (Dronetag Beacon)", 1, 200, "[S95]"), ("Downward lidar LW20/C", 1, 290, "[S96]"),
    ("10 mm-thick wing servo (KST X10 / MKS HV6130 class)", 4, 60, "[A] (2 ailerons + 2 ruddervators; flaps deleted)"),
    ("Power module / PDB 12S", 1, 120, "[A]"), ("Strobes + nav lights", 1, 80, "[A]"), ("Wiring, connectors (XT90/AS150), coax", 1, 250, "[A]"),
]
HARDWARE = [
    ("M5 heat-set insert (wing hardpoints)", 16), ("M5x40 socket head, boom saddle -> wing", 16), ("M6 nylon bolt (breakaway wing mount) + M6 heat-set insert", 4),
    ("M4 heat-set insert (pylons: motor plate + clamp)", 32), ("M4x8 button head, motor plate -> pylon", 16), ("M4x35 socket head, pylon clamp", 16),
    ("M4 bolt + nyloc, tube cross-pins (pylons 4, saddles 4, tail sockets 2) + outer-spar pins 2", 12), ("M3 heat-set insert (frames, trays, cradle, GNSS pads)", 40),
    ("M3 screws/standoffs assortment" + ("" if __import__("config").MINI else " (incl. 4x M3x20 radio standoffs)"), 1), ("6x3 mm N52 magnets (hatches)", 16), ("CA hinges / Blenderm hinge tape (ailerons, ruddervators)", 1),
    ("Clevises + 1.5 mm pushrods", 4), ("Battery straps 25 mm", 3), ("Structural epoxy (e.g. 3M DP490) + microballoons", 1),
]

def write_bom(metas, P, cutlist, M, cg, hits):
    rows = sorted(metas, key=lambda m: (m["process"], m["group"], m["name"]))
    printed = [m for m in rows if m["process"] == "print"]
    cnc = [m for m in rows if m["process"] == "cnc"]
    def qty(m):
        return 4 if m["name"] == "misc_control_horn" else m["qty"]
    fil = {}; hours = 0.0
    for m in printed:
        g = m["mass_g"]*qty(m); fil[m["material"]] = fil.get(m["material"], 0) + g
        hours += m["vol_cm3"]*m["fill"]*qty(m)/RATE.get(m["material"], 12)
    with open(OUT / "BOM_printed_parts.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["file", "group", "qty", "material", "g_each", "g_total", "print_dims_mm", "est_hours_each", "notes", "print_settings"])
        for m in printed:
            w.writerow([m["name"], m["group"], qty(m), m["material"], m["mass_g"], round(m["mass_g"]*qty(m), 1), "x".join(f"{v:.0f}" for v in m["print_dims"]),
                        round(m["vol_cm3"]*m["fill"]/RATE.get(m["material"], 12), 1), m["notes"], m["print_notes"]])
    L = []
    import config
    L.append("# Ridgeback smaller own design (2.24 m, 6S6P) - fabrication BOM\n" if config.MINI else "# SR-3 Ridgeback (airborne relay) - fabrication BOM\n")
    L.append(f"Generated by `ridgeback/cad/fab` on {time.strftime('%Y-%m-%d %H:%M')}. Every part file is in `fab/out/` "
             "(STEP in place for Fusion 360, STL in print orientation for the slicer, DXF for CNC). Prices: [V] verified, [A] assumed.\n")
    L.append(f"**All-up mass (CAD + listed items): {M/1000:.2f} kg** vs sizing closure {d.MTOW_G/1000:.1f} kg" + ("" if config.MINI else " and concept CAD 15.57 kg") + ". "
             f"CG x = {cg[0]:.0f} mm (target {d.CG_X:.0f}), y = {cg[1]:+.1f}, z = {cg[2]:.0f}.\n")
    L.append("## 1. Printed parts (" + ("Prusa CORE One bed 250 x 220 x 270" if config.MINI else "256 mm bed") + ")\n")
    L.append(f"{len(printed)} files, {sum(qty(m) for m in printed)} prints, ~{hours:.0f} printer-hours (estimate at {', '.join(f'{k} {v:.0f}' for k, v in RATE.items())} cm3/h).\n")
    L.append("| File (stl/ + step/) | Qty | Material | g each | Print size mm | Notes |\n|---|---|---|---|---|---|")
    for m in printed:
        L.append(f"| `{m['name']}` | {qty(m)} | {m['material']} | {m['mass_g']:.0f} | {'x'.join(f'{v:.0f}' for v in m['print_dims'])} | {m['notes']} *{m['print_notes']}* |")
    L.append("\n**Filament:** " + ", ".join(f"{k} {v/1000:.2f} kg (~${v/1000*PRICE[k]*1.3:.0f} incl. 30 % scrap)" for k, v in fil.items()) + "\n")
    L.append("## 2. CNC parts\n\n| File (dxf/ + step/) | Qty | Material | g each | Notes |\n|---|---|---|---|---|")
    for m in cnc:
        L.append(f"| `{m['name']}` | {m['qty']} | {m['material']} | {m['mass_g']:.0f} | {m['notes']} |")
    L.append("\n## 3. Tubes and rods (cut list)\n\n| Item | OD mm | Wall mm | Cut length mm | Qty |\n|---|---|---|---|---|")
    for it, od, wall, length, q in cutlist:
        L.append(f"| {it} | {od} | {wall if wall else 'solid'} | {length:.0f} | {q} |")
    if config.MINI:
        L.append("\nBuy: see `ridgeback/docs/BUY_LIST_2026-11-02.md` (HaoZhong 1 m tubes). 13/11 cut list uses 978 of the 1 m tube: cut carefully.\n")
    else:
      L.append("\nBuy: 24x20 and 18x15 roll-wrapped CF (1 m / 2 m lengths); 30x27 x 2000 mm boom tubes (2); 12x10 x 500 mm (2); 6x4 and 8x6 pultruded CF tube; 10x8 fibreglass tube x 1 m (2); 3 mm CF rod + 3 mm glass rod; 8x6.5 aluminium tube for the 4 wing-bolt sleeves.\n")
    import config
    if config.MINI:
        L.append("## 4. Purchased components\n\nSee `ridgeback/docs/BUY_LIST_2026-11-02.md` (store links, $2,946 total) and `analysis/project_bom.py` stage MINI.\n")
    else:
        L.append("## 4. Purchased components (relay configuration)\n\n| Item | Qty | Unit $ | Total $ | Source |\n|---|---|---|---|---|")
        tot = 0
        for it, q, u, src in BOUGHT:
            t = q*u if u else None; tot += t or 0
            L.append(f"| {it} | {q} | {u if u else '-'} | {t if t else 'quote'} | {src} |")
        L.append(f"\nPurchased subtotal **${tot:,.0f}** + repeater radio + duplexer (quotes). The turret + MANET are ${22000:,}: a first-flight airframe can drop both (~${tot - 22000:,.0f}).\n")
    L.append("## 5. Hardware\n\n| Item | Qty |\n|---|---|")
    for it, q in HARDWARE:
        L.append(f"| {it} | {q} |")
    L.append("\n## 6. Mass by group\n")
    grp = {}
    for m in metas:
        grp[m["group"]] = grp.get(m["group"], 0) + m["mass_g"]*m["qty"]
    for p in P:
        grp[p["group"]] = grp.get(p["group"], 0) + p["mass_g"]
    L.append("| Group | g |\n|---|---|")
    for k, v in sorted(grp.items(), key=lambda kv: -kv[1]):
        L.append(f"| {k} | {v:.0f} |")
    L.append(f"| **total** | **{M:.0f}** |\n")
    L.append("## 7. Checks\n")
    bad = [m["name"] for m in printed if not (m["fits_bed"] and m["watertight"] and m["valid"])]
    L.append(f"- Printed parts: {len(printed) - len(bad)}/{len(printed)} fit the " + ("CORE One bed (240 x 210 x 260 after 5 mm margins)" if config.MINI else "256 mm bed") + f", are watertight and valid" + (f"; CHECK: {bad}" if bad else "") + ".")
    L.append(f"- Interference (bought items vs all parts, boom/payload vs airframe; bonded neighbours excluded): {len(hits)} pairs > 20 mm3.")
    for h in hits:
        L.append(f"  - {h[0]} x {h[1]}: {h[2]} mm3")
    (OUT / "BOM.md").write_text("\n".join(L) + "\n")
    print("BOM written")


if __name__ == "__main__":
    main()
