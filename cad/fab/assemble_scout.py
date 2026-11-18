"""SR-3 Scout v1 assembly (SR3_CONFIG=scout): places the Scout part files, the purchased items (relay list minus the relay
payload, plus scout.items()), the boom tubes with their harness holes and the harness bundles; checks mass / CG /
interference; writes SR3_scout_assembly.step, a working GLB (trimesh), renders, BOM and summary json.
Called from assemble.main() when config.SCOUT. PROVISIONAL."""
import json, math, sys, time, csv
from pathlib import Path
import numpy as np

FAB = Path(__file__).resolve().parent
sys.path[:0] = [str(FAB), str(FAB.parent), str(FAB.parent / "parts")]
from build123d import *
import core
from core import d, cyl_between, bx, OUT
import assemble as AS
import scout as S
import harness as H
import f_boom as FB
import f_tail as FT
import f_fuse as FF

SUBCOL = dict(H.SUBCOL, lights=(0.95, 0.80, 0.10), propulsion=(0.30, 0.30, 0.30))
SKIN_GROUPS = ("wing", "fuselage", "tail")
COL = dict(AS.COL, elec=(0.95, 0.55, 0.15))


def boom_tube(s):
    """30x1.5 boom tube with the Scout harness holes (right side s=1, left s=-1)."""
    p0, p1 = (d.BOOM_FRONT_X, s*d.BOOM_Y, d.BOOM_Z), (d.BOOM_REAR_X, s*d.BOOM_Y, d.BOOM_Z)
    t = AS.tube(p0, p1, *d.BOOM_TUBE)
    Hh = FB.TUBE_HOLES; L, W = Hh["slot"]
    yb = s*d.BOOM_Y; zb = d.BOOM_Z
    cuts = []
    # trunk slot, inboard side on the neutral axis, axial stadium L x W
    r = W/2
    slot = cyl_between((Hh["slot_x"] - L/2 + r, yb, zb), (Hh["slot_x"] - L/2 + r, yb - s*20, zb), r).fuse(
        cyl_between((Hh["slot_x"] + L/2 - r, yb, zb), (Hh["slot_x"] + L/2 - r, yb - s*20, zb), r),
        bx(Hh["slot_x"] - L/2 + r, Hh["slot_x"] + L/2 - r, *sorted((yb, yb - s*20)), zb - r, zb + r))
    cuts.append(slot)
    for mx in (d.FRONT_MOTOR_X, d.REAR_MOTOR_X):                        # ESC power-lead holes, bottom
        cuts.append(cyl_between((mx + Hh["esc_x"], yb, zb - 20), (mx + Hh["esc_x"], yb, zb), Hh["esc_d"]/2))
    a = H.tail_pt(0.0, s); b = H.tail_pt(25.0, s)                       # ruddervator-lead hole, along U_HAT
    cuts.append(cyl_between(a, b, Hh["tail_d"]/2))
    return t.cut(*cuts), cuts


def tube_strength():
    """[C] boom tube 30x1.5 at the trunk slot (neutral axis of vertical bending) and at the ESC hole (bottom)."""
    ro, ri = 15.0, 13.5; rm = (ro + ri)/2; t = 1.5
    I = math.pi/64*(30**4 - 27**4)
    L, W = FB.TUBE_HOLES["slot"]
    th = (W/2)/rm
    dI = t*rm**3*(th - math.sin(th)*math.cos(th))
    T = 8716*9.81/1000                                                 # 100 % thrust per motor [S63], N
    arm = 82 - d.FRONT_MOTOR_X                                         # front motor to the front saddle, mm
    M = T*arm                                                          # N mm at the saddles (both cantilevers hog)
    s_int = M*ro/I
    z_edge = rm*math.sin(th)
    s_edge = M*z_edge/(I - dI)
    Me = T*FB.TUBE_HOLES["esc_x"]                                      # at the ESC hole, cantilever from the motor
    s_esc = Me*ro/I
    return dict(I=I, dI_pct=100*dI/I, M_Nm=M/1000, sigma_intact=s_int, sigma_slot_edge=s_edge, Kt=3.0, sigma_slot_peak=3*s_edge,
                Kt_cfrp=5.0, sigma_slot_peak_cfrp=5*s_edge, M_esc_Nm=Me/1000, sigma_esc_hole_peak=5*s_esc)


def purchased_scout():
    P, cut = AS.purchased()
    P = [p for p in P if p["name"] not in S.REMOVED_RELAY and not p["name"].startswith("tube_boom_")]
    for p in P:
        p["sub"] = None
    for s, k in ((1, "R"), (-1, "L")):
        sol, _ = boom_tube(s)
        rho = 1.55
        L = d.BOOM_REAR_X - d.BOOM_FRONT_X
        m = math.pi*(30**2 - 27**2)/4*L/1e3*rho
        P.append(dict(name=f"tube_boom_{k}", group="tube", solid=sol, mass_g=m, cg=None, sub=None))
    items = S.items()
    esc_c = np.mean([np.array(i["c"]) for i in items if i["key"].startswith("esc") and i["key"] != "esc_push"], axis=0)
    for it in items:
        k = it["key"]
        if k == "caps":
            P.append(dict(name="esc_caps", group="elec_power", solid=None, mass_g=it["mass_g"], cg=tuple(esc_c), sub="power")); continue
        if k.startswith("servo_rv"):
            s = 1 if k.endswith("R") else -1
            loc = FT.frame_at(FT.SERVO["u"], FT.SERVO["f"])
            sv = Box(*it["dims"]).moved(loc*Location((0, 0, -1.6)))
            sol = sv if s > 0 else sv.mirror(Plane.XZ)
        elif k.startswith("ant_"):
            dx, dy, dz = it["dims"]
            sol = Cylinder(dx/2, dz).moved(Location(it["c"]))
        else:
            sol = Box(*it["dims"]).moved(Location(it["c"]))
        sub = it["sub"]
        P.append(dict(name=f"{k}", group=f"elec_{sub}", solid=sol, mass_g=it["mass_g"], cg=None, sub=sub, item=it))
    P.append(dict(name="mount_hardware_elec", group="lump", solid=None, mass_g=45.0, cg=(-100.0, 0.0, -110.0), sub=None))   # [A] standoffs, dampers, ties, VHB, M2/M3
    return P, cut


def bundles():
    res = H.analyse()
    out = []
    for k, b in res["bundles"].items():
        sol = H.bundle_solid(b["pts"], b["dia"])
        out.append(dict(name=f"harness_{k}", group=f"harness_{b['sub']}", solid=sol, sub=b["sub"], dia=b["dia"], pts=b["pts"]))
    return out, res


def main():
    t0 = time.time()
    metas = AS.load_parts()
    print(f"{len(metas)} Scout part files")
    items, mass_rows = [], []
    for m in metas:
        s = import_step(str(OUT / "step" / f"{m['name']}.step"))
        col = AS.CNC_COL if m["process"] == "cnc" else COL.get(m["group"], (0.7, 0.7, 0.7))
        for k, (dx, dy, dz) in enumerate(m["instances"]):
            si = s.moved(Location((dx, dy, dz)))
            si.label = m["name"] if len(m["instances"]) == 1 else f"{m['name']}#{k+1}"
            items.append(dict(name=si.label, part=m["name"], group=m["group"], solid=si, kind=m["process"], col=col, sub=None))
            mass_rows.append((m["name"], m["group"], m["mass_g"], np.array(m["center"]) + [dx, dy, dz]))
    P, cutlist = purchased_scout()
    for p in P:
        col = SUBCOL.get(p["sub"]) if p.get("sub") else COL.get(p["group"], (0.5, 0.5, 0.5))
        if p["solid"] is not None:
            p["solid"].label = p["name"]
            items.append(dict(name=p["name"], part=p["name"], group=p["group"], solid=p["solid"], kind="bought", col=col, sub=p.get("sub")))
            c = p["solid"].center(); cg = np.array([c.X, c.Y, c.Z])
        else:
            cg = np.array(p["cg"])
        mass_rows.append((p["name"], p["group"], p["mass_g"], cg))
    B, hres = bundles()
    hcg, hm = H.harness_cg(hres)
    mass_rows.append(("wiring_harness_scout", "harness", hres["total_g"], hcg))
    for b in B:
        items.append(dict(name=b["name"], part=b["name"], group=b["group"], solid=b["solid"], kind="harness", col=SUBCOL[b["sub"]], sub=b["sub"], pts=b["pts"], dia=b["dia"]))
    M = sum(r[2] for r in mass_rows)
    cg = sum(r[2]*r[3] for r in mass_rows)/M
    mb = d.BATTERY["g"]
    x_need = FF.BAT["x"] + (d.CG_X - cg[0])*M/mb
    print(f"Scout mass {M/1000:.2f} kg, CG ({cg[0]:.1f}, {cg[1]:.1f}, {cg[2]:.0f}); battery centre x for CG {d.CG_X:.0f}: {x_need:.1f} (now {FF.BAT['x']:.1f})")
    print(f"harness {hres['total_g']:.0f} g (wire {hres['wire_g']:.0f}, connectors {hres['conn_g']:.0f}, sleeve {hres['sleeve_g']:.0f}), CG {hcg.round(0)}")

    hits = interference(items)
    # exports
    comp = Compound(label="SR-3 Scout v1 (provisional)", children=[it["solid"] for it in items])
    export_step(comp, str(OUT / "SR3_scout_assembly.step"))
    elec = [it["solid"] for it in items if it["group"].startswith(("elec", "harness")) or it["kind"] == "harness"]
    export_step(Compound(label="SR-3 Scout electronics + harness", children=elec), str(OUT / "SR3_scout_sub_electronics.step"))
    meshes = mesh_items(items)
    write_glb(items, meshes, OUT / "SR3_scout_assembly.glb")
    import render_scout as RS
    RS.render_all(items, meshes, OUT / "renders", hres)
    st = tube_strength()
    write_bom(metas, P, hres, M, cg, hits, st, x_need)
    summ = dict(mass_g=M, cg=list(map(float, cg)), x_bat_now=FF.BAT["x"], x_bat_need=float(x_need), harness_g=hres["total_g"],
                harness_wire_g=hres["wire_g"], harness_conn_g=hres["conn_g"], harness_sleeve_g=hres["sleeve_g"], harness_cg=list(map(float, hcg)),
                hits=hits, tube=st, n_parts=len(metas),
                mass_by_group={g: float(sum(r[2] for r in mass_rows if r[1] == g)) for g in sorted({r[1] for r in mass_rows})},
                rows=[(r[0], r[1], float(r[2]), [float(v) for v in r[3]]) for r in mass_rows])
    (OUT / "scout_summary.json").write_text(json.dumps(summ, indent=1))
    print(f"Scout assembly done in {time.time() - t0:.0f}s")


def interference(items):
    def bb(s):
        b = s.bounding_box(); return np.array([b.min.X, b.min.Y, b.min.Z]), np.array([b.max.X, b.max.Y, b.max.Z])
    boxes = [bb(it["solid"]) for it in items]
    press_fit = ("boom_foot", "rod_leg")
    hits = []
    n = len(items)
    for i in range(n):
        for j in range(i + 1, n):
            a, b = items[i], items[j]
            if a["kind"] != "bought" and b["kind"] != "bought" and a["group"] == b["group"] and a["group"] in ("wing", "tail", "fuselage") :
                continue
            if a["kind"] == "harness" and b["kind"] == "harness":
                continue
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
                c = r.center() if (r is not None and v > 0) else None
            except Exception:
                v, c = -1, None
            thr = 2.0 if "harness" in (a["kind"], b["kind"]) else 20.0
            if v > thr or v < 0:
                tag = ""
                for it in (a, b):
                    if it["kind"] == "harness" and c is not None:
                        ends = [np.array(it["pts"][0]), np.array(it["pts"][-1])]
                        if min(np.linalg.norm(np.array([c.X, c.Y, c.Z]) - e) for e in ends) < 25.0:
                            tag = "endpoint (cable termination, intended)"
                hits.append((a["name"], b["name"], round(v, 1), tag))
    real = [h for h in hits if not h[3]]
    print(f"interference: {len(hits)} pairs ({len(real)} not at a cable termination)")
    for h in real[:80]:
        print("   ", h)
    return hits


def mesh_items(items, tol=0.4, ang=0.5):
    out = []
    for it in items:
        try:
            m = core.mesh_of(it["solid"], tol, ang)
        except Exception as e:
            m = None
        out.append(m)
    return out


def write_glb(items, meshes, path):
    """GLB through trimesh (one node per item, PBR colour). build123d's export_gltf tessellates with RELATIVE deflection
    (assemble.py passes 0.5 -> 4-vertex faces) and wrote a 1-6-mesh file for the 300 MB assembly; this path is explicit."""
    import trimesh
    sc = trimesh.Scene()
    for it, m in zip(items, meshes):
        if m is None or len(m.faces) == 0:
            continue
        mm = m.copy()
        mm.apply_scale(0.001)                                             # glTF is metres
        mm.apply_transform(trimesh.transformations.rotation_matrix(-math.pi/2, [1, 0, 0]))   # z-up -> y-up
        rgba = list(it["col"]) + [1.0]
        mm.visual = trimesh.visual.TextureVisuals(material=trimesh.visual.material.PBRMaterial(baseColorFactor=rgba, metallicFactor=0.1, roughnessFactor=0.7))
        sc.add_geometry(mm, node_name=it["name"], geom_name=it["name"])
    sc.export(str(path))
    chk = trimesh.load(str(path))
    print(f"GLB {path.name}: {path.stat().st_size/1e6:.1f} MB, {len(chk.geometry)} meshes, {sum(len(g.faces) for g in chk.geometry.values())} triangles")


def write_bom(metas, P, hres, M, cg, hits, st, x_need):
    rows = sorted(metas, key=lambda m: (m["process"], m["group"], m["name"]))
    L = ["# SR-3 Scout v1 (PROVISIONAL) - fabrication BOM and checks\n",
         f"Generated by `ridgeback/cad/fab` (SR3_CONFIG=scout) on {time.strftime('%Y-%m-%d %H:%M')}. See docs/SR3_SCOUT_ELECTRONICS.md.\n",
         f"**All-up mass {M/1000:.2f} kg [C]**, CG x {cg[0]:.1f} (target {d.CG_X:.0f}), y {cg[1]:+.1f}, z {cg[2]:.0f}. Battery centre x {FF.BAT['x']:.1f} (x for exact CG: {x_need:.1f}).\n",
         "## Printed / CNC parts\n", "| File | Qty | Material | g each | Print size | fits | watertight | valid |", "|---|---|---|---|---|---|---|---|"]
    for m in rows:
        L.append(f"| `{m['name']}` | {m['qty']} | {m['material']} | {m['mass_g']:.1f} | {'x'.join(f'{v:.0f}' for v in m['print_dims'])} | {m['fits_bed']} | {m['watertight']} | {m['valid']} |")
    L += ["\n## Electronics (scout.items())\n", "| Item | Dims mm | g | W avg / peak | Source | Tag |", "|---|---|---|---|---|---|"]
    for it in S.items():
        L.append(f"| {it['name']} | {'x'.join(f'{v:g}' for v in it['dims'])} | {it['mass_g']:g} | {it['w_avg']:g} / {it['w_peak']:g} | {it['src']} | {it['tag']} |")
    L += ["\n## Wire table\n", "| Cable | From | To | Conductors | Length m | g | Connectors | Note |", "|---|---|---|---|---|---|---|---|"]
    for r in hres["rows"]:
        cond = " + ".join(f"{n}x {g if isinstance(g, str) else str(g) + ' AWG'}" for g, n in r["cond"])
        L.append(f"| {r['name']} | {r['frm']} | {r['to']} | {cond} | {r['length_m']:.2f} | {r['mass_wire_g'] + r['mass_conn_g']:.1f} | {r['conn'][0]} / {r['conn'][1]} | {r['note']} |")
    L.append(f"\nHarness: wire {hres['wire_g']:.0f} g + connectors {hres['conn_g']:.0f} g + sleeve/ties {hres['sleeve_g']:.0f} g = **{hres['total_g']:.0f} g**\n")
    bad = [m["name"] for m in rows if m["process"] == "print" and not (m["fits_bed"] and m["watertight"] and m["valid"])]
    L.append(f"## Checks\n- printed parts failing bed/watertight/valid: {bad or 'none'}")
    real = [h for h in hits if not h[3]]
    L.append(f"- interference pairs: {len(hits)} ({len(real)} not at a cable termination)")
    for h in hits:
        L.append(f"  - {h[0]} x {h[1]}: {h[2]} mm3 {h[3]}")
    L.append(f"- boom tube at the trunk slot: I loss {st['dI_pct']:.1f} %, sigma at the slot edge {st['sigma_slot_edge']:.0f} MPa x Kt 5 = {st['sigma_slot_peak_cfrp']:.0f} MPa (intact max fibre {st['sigma_intact']:.0f} MPa) at 100 % thrust")
    (OUT / "BOM.md").write_text("\n".join(L) + "\n")
    with open(OUT / "wire_table.csv", "w", newline="") as f:
        w = csv.writer(f); w.writerow(["cable", "from", "to", "conductors", "length_m", "mass_g", "conn_from", "conn_to", "subsystem", "note"])
        for r in hres["rows"]:
            w.writerow([r["name"], r["frm"], r["to"], "; ".join(f"{n}x{g}" for g, n in r["cond"]), round(r["length_m"], 3),
                        round(r["mass_wire_g"] + r["mass_conn_g"], 1), r["conn"][0], r["conn"][1], r["sub"], r["note"]])
