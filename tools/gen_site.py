"""Build the static site in docs/ (served by GitHub Pages).

Inputs: tools/parts.py (every part), data/calc_values.json + data/refs.json (results of the calculation sheets),
data/sheet_links.json (Google Sheets links), docs/assets/ridgeback.glb (from tools/export_model.py).
Run: python3 tools/gen_site.py
"""
import json, re, html, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))
from parts import PARTS, SYSTEMS, PART_LINE
GH = "https://github.com/Stanford-SkyRunners/ridgeback-design"

E = html.escape
VALS = json.load(open(ROOT / "data/calc_values.json"))
R = json.load(open(ROOT / "data/refs.json"))
LINKS = json.load(open(ROOT / "data/sheet_links.json")) if (ROOT / "data/sheet_links.json").exists() else {}
FN = "RIDGEBACK_MINI_VTOL_DESIGN_2026-11-17.XLSX"


def val(key):
    m = re.match(r"'(.+)'!\$?([A-Z]+)\$?(\d+)", R[key])
    return VALS[f"'[{FN}]{m.group(1).upper()}'!{m.group(2)}{m.group(3)}"]


FMT = {"Icell_hot": "{:.1f}", "tloit_sl": "{:.0f}", "tloit_hot": "{:.0f}", "flights_pack": "{:.1f}", "Ipack_hot": "{:.0f}", "Ipack_sl": "{:.0f}",
       "m_to": "{:.1f}", "Thov_sl": "{:,.0f}", "throttle_sl": "{:.0%}", "throttle_hot": "{:.0%}", "TW_hot": "{:.2f}", "TW_sl": "{:.2f}",
       "Imot_hot": "{:.1f}", "I_full": "{:.1f}", "I_push": "{:.1f}", "D_sl": "{:.1f}", "V_sl": "{:.0f}", "Tav_sl": "{:.0f}",
       "range_km": "{:.0f}", "Vs_sl": "{:.1f}", "CD0_build": "{:.3f}", "cost_total": "${:,.0f}", "E_use": "{:.0f}", "E_flight": "{:.0f}",
       "Pcr_sl": "{:.0f}", "Phov_sl": "{:,.0f}", "Phov_hot": "{:,.0f}"}


def fill(text):
    return re.sub(r"\{(\w+)\}", lambda m: FMT.get(m.group(1), "{:.3g}").format(val(m.group(1))), text)


def sheet(name, tab=None):
    u = LINKS.get(name)
    label = f"Sheet {name[:2]}" + (f", tab {tab}" if tab else "")
    return f'<a href="{u}" target="_blank" rel="noopener">{E(label)}</a>' if u else E(label)


# ------------------------------------------------------------------ parts data for the browser
g0 = 9.80665
CALCS = {
    "battery": dict(step="step-8", sheet=("04_Propulsion_Battery_Energy", "Mission_Energy"),
        lines=[(r"E_{flight} = E_{hover} + E_{climb} + P\,(t_{survey} + t_{transit}) = %.0f\ \text{Wh}" % val("E_flight"), "energy for one 15-minute survey, 1 km out and back"),
               (r"E_{pack,\,needed} = \dfrac{3 \times %.0f}{0.85 \times 0.8} = %.0f\ \text{Wh}" % (val("E_flight"), 3 * val("E_flight") / 0.68), "three flights, keeping the cells between 95 % and 10 % and 20 % in reserve"),
               (r"%.0f\ \text{Wh needed} \;>\; 648\ \text{Wh in 6S6P}" % (3 * val("E_flight") / 0.68), "so the pack covers %.1f flights, not 3; a 7P pack would fail the hover-thrust rule (Battery_Options)" % val("flights_pack"))]),
    "motor_lift": dict(step="step-7", sheet=("04_Propulsion_Battery_Energy", "Lift_Hover"),
        lines=[(r"T_{hover} = \dfrac{m g}{4} = \dfrac{%.2f \times 9.81}{4} = %.1f\ \text{N per motor}" % (val("m_to"), val("m_to") * g0 / 4), "thrust each motor holds in a hover"),
               (r"\dfrac{T}{W} = \dfrac{4\,T_{max}}{m g} = \dfrac{4 \times %.1f}{%.1f} = %.2f \;\geq\; 1.5" % (val("Tmax_hot"), val("m_to") * g0, val("TW_hot")), "hot day at 800 m, from T-Motor's test table")]),
    "prop_lift": dict(step="step-7", sheet=("04_Propulsion_Battery_Energy", "Lift_Hover"),
        lines=[(r"\dfrac{T}{W} = %.2f \;\text{on a hot day}" % val("TW_hot"), "the maker's thrust figures are for this exact propeller")]),
    "esc_lift": dict(step="step-6", sheet=("04_Propulsion_Battery_Energy", "Lift_Hover"),
        lines=[(r"I_{max} = \dfrac{P_{100\%%}}{V} = \dfrac{754\ \text{W}}{22.5\ \text{V}} = %.1f\ \text{A} \;<\; 40\ \text{A}" % val("I_full"), "full throttle, from the motor test table"),
               (r"I_{hover} = %.1f\ \text{A}" % val("Imot_hot"), "hot-day hover, per motor")]),
    "pm07": dict(step="step-6", sheet=("04_Propulsion_Battery_Energy", "Lift_Hover"),
        lines=[(r"I_{pack} = \dfrac{P_{hover}}{V} = \dfrac{%.0f\ \text{W}}{21.6\ \text{V}} = %.0f\ \text{A} \;<\; 90\ \text{A}" % (val("Phov_hot"), val("Ipack_hot")), "hot-day hover through the power module")]),
    "motor_pusher": dict(step="step-4", sheet=("04_Propulsion_Battery_Energy", "Pusher"),
        lines=[(r"D = \tfrac12 \rho V^2 S\, C_D = %.1f\ \text{N}" % val("D_sl"), "drag at the loiter speed of %.0f m/s" % val("V_sl")),
               (r"\dfrac{D}{T(V)} = \dfrac{%.1f}{%.0f} = %.2f" % (val("D_sl"), val("Tav_sl"), val("Tratio_sl")), "share of the pusher's thrust used in level flight (estimate)")]),
    "wing": dict(step="step-2", sheet=("03_Aerodynamics", "Aero_Cruise"),
        lines=[(r"V_s = \sqrt{\dfrac{2W}{\rho S C_{L,\max}}} = %.1f\ \text{m/s}" % val("Vs_sl"), "stall speed with 0.50 m² and the SD7037 airfoil"),
               (r"V_{loiter} = 1.3\,V_s = %.1f\ \text{m/s}" % val("V_sl"), "loiter speed, kept above stall for turns and gusts")]),
    "servos": dict(step=None, sheet=("05_Structure_and_Stability", "Structure_Stability"),
        lines=[(r"\tau_{needed} = 2 \times M_{hinge} = 2 \times 2.34 = 4.7\ \text{kg·cm} \;<\; 7.5", "aileron at a 34 m/s dive, with a factor of 2")]),
}
pj = []
for p in PARTS:
    q = dict(p)
    q["why"] = fill(p["why"]); q["job"] = fill(p["job"])
    q["systemName"], q["color"] = SYSTEMS[p["system"]]
    q["edit"] = f"{GH}/edit/main/data/parts.toml"; q["editLine"] = PART_LINE[p["id"]]
    if p["id"] in CALCS:
        c = CALCS[p["id"]]
        q["calc"] = [list(x) for x in c["lines"]]
        q["calcStep"] = c["step"]; q["calcSheet"] = LINKS.get(c["sheet"][0]); q["calcSheetName"] = f"Sheet {c['sheet'][0][:2]}, tab {c['sheet'][1]}"
    pj.append(q)
(ROOT / "docs/assets/parts.json").write_text(json.dumps(dict(parts=pj, systems={k: {"name": a, "color": b} for k, (a, b) in SYSTEMS.items()}), indent=1))

FONTS = ('<link rel="preconnect" href="https://fonts.googleapis.com"><link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>'
         '<link href="https://fonts.googleapis.com/css2?family=Archivo:wdth,wght@75..100,400..700&family=IBM+Plex+Mono:wght@400;500&display=swap" rel="stylesheet">')


def page(title, slug, body, extra_head=""):
    nav = [("index.html", "Drone"), ("mission.html", "Mission and range"), ("calculations.html", "Calculations"), ("files.html", "Sheets, code, CAD")]
    links = "".join(f'<a href="{h}"{" aria-current=page" if h == slug else ""}>{t}</a>' for h, t in nav)
    return f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{E(title)}</title>{FONTS}<link rel="stylesheet" href="assets/style.css">{extra_head}</head>
<body><div class="sheet"><header class="collar"><div class="mast"><a class="home" href="index.html">Ridgeback Scout</a>
<span class="who">SkyRunners, Stanford Student Robotics</span></div><nav>{links}</nav></header>
<main>{body}</main>
<footer class="collar foot"><span>Designed in CAD, not yet built or flown. Numbers from the calculation sheets, updated 17 Nov 2026.</span>
<a href="https://github.com/Stanford-SkyRunners/ridgeback-design">Source on GitHub</a></footer></div></body></html>"""


def status_row():
    rows = [("Hover thrust ÷ weight, hot day", f"{val('TW_hot'):.2f}", "at least 1.5", val("TW_hot") >= 1.5),
            ("Loiter time, 20 % reserve left", f"{val('tloit_sl'):.0f} min", "at least 80 min", val("tloit_sl") >= 80),
            ("Survey flights per charge", f"{val('flights_pack'):.1f}", "at least 3", val("flights_pack") >= 3),
            ("Stall speed, clean wing", f"{val('Vs_sl'):.1f} m/s", "at most 15 m/s", val("Vs_sl") <= 15),
            ("Aircraft cost", f"${val('cost_total'):,.0f}", "at most $3,000", val("cost_total") <= 3000)]
    tr = "".join(f'<tr><td>{a}</td><td class="num">{b}</td><td>{c}</td><td class="{"ok" if ok else "no"}">{"meets it" if ok else "short"}</td></tr>' for a, b, c, ok in rows)
    return f'<table class="status"><thead><tr><th>Requirement</th><th class="num">Estimate</th><th>Target</th><th>Result</th></tr></thead><tbody>{tr}</tbody></table>'


# ------------------------------------------------------------------ index: the drone explorer
groups = {}
for p in PARTS:
    groups.setdefault(p["system"], []).append(p)
rows = []
for sk, (sname, col) in SYSTEMS.items():
    if sk not in groups: continue
    rows.append(f'<tr class="grp"><th colspan="6"><span class="key" style="--c:{col}"></span>{E(sname)}</th></tr>')
    for p in groups[sk]:
        price = "owned" if p["status"].startswith("reuse") else (f"${p['price']:,.2f}" if p["price"] else "")
        buy = p["buy"][0] if p["buy"] else None
        b = f'<a href="{E(buy[1])}" target="_blank" rel="noopener">{E(buy[0].split(" (")[0])}</a>' if buy else ""
        rows.append(f'<tr id="row-{p["id"]}" data-id="{p["id"]}" tabindex="0"><td>{E(p["name"])}</td><td class="num">{p["qty"]}</td><td>{E(p["status"])}</td>'
                    f'<td class="num">{price}</td><td class="num">{p["mass"]:,.0f} g</td><td>{b}</td></tr>')
index_body = f"""
<section class="lead"><h1>Inside the drone</h1>
<p>Our own 2.24 m VTOL drone for measuring how much water chaparral shrubs lose between dawn and midday, for planning prescribed burns.
Click any part on the drone or in the list to see what it does, why we chose it, what it connects to and where to buy it.</p></section>
<section class="explorer">
  <div class="viewer"><div class="viewbar"><div class="mapbar" role="group" aria-label="Flight mode"><button type="button" data-mode="hover">Hover</button><button type="button" data-mode="cruise">Cruise</button><button type="button" data-mode="still">Still</button></div>
  <button type="button" id="reset" class="plainbtn">Whole drone</button></div>
  <div class="picks" role="group" aria-label="Show a part"><span>Show:</span><button type="button" data-pick="battery">Battery</button><button type="button" data-pick="motor_lift">Lift motors</button><button type="button" data-pick="motor_pusher">Pusher</button><button type="button" data-pick="pixhawk">Flight controller</button><button type="button" data-pick="wing">Wing</button></div>
  <div class="viewwrap"><div id="view" aria-label="3D model of the drone. Drag to turn, scroll to zoom, click a part."></div><div id="tip" class="tip" hidden></div></div>
  <div class="legend">{''.join(f'<span><i style="--c:{c}"></i>{E(n)}</span>' for k, (n, c) in SYSTEMS.items())}</div>
  <p class="hint">Drag to turn, scroll to zoom. Hover spins the lift propellers; Cruise parks them along the booms and runs the pusher. Every part sits at its CAD position and size.</p></div>
  <aside id="detail" class="detail" aria-live="polite"><h2>Six systems</h2>
  <p>Pick a part on the drone or below; it turns purple and lines show what it is wired or bolted to.</p>
  {''.join(f'<h3><span class="key" style="--c:{SYSTEMS[k][1]}"></span>{E(SYSTEMS[k][0])}</h3><p>{E(t)}</p>' for k, t in [
    ("power", "Battery, power module and servo regulator: 648 Wh at 21.6 V, about 56 A in a hot hover."),
    ("lift", "Four motors, props and speed controllers that lift the drone off a hillside and land it."),
    ("cruise", "The pusher that flies the drone on the wing over the plot."),
    ("control", "Flight controller, GPS, airspeed, radios and Remote ID."),
    ("surfaces", "Four servos for the ailerons and the arch tail."),
    ("airframe", f"Printed wing, fuselage and tail on carbon tubes. Takeoff mass {val('m_to'):.2f} kg.")])}</aside>
</section>
<section><h2>Requirements</h2>{status_row()}</section>
<section><h2>Every part</h2>
<div class="tablewrap"><table class="parts"><thead><tr><th>Part</th><th class="num">Qty</th><th>Status</th><th class="num">Unit price</th><th class="num">Mass each</th><th>Buy</th></tr></thead>
<tbody>{''.join(rows)}</tbody></table></div>
<p class="small">Connectors, wire and solder for every joint are listed on {sheet('06_Electronics_BOM_Cost', 'Connectors_Wiring')}; the full parts list with duty and shipping on {sheet('06_Electronics_BOM_Cost', 'BOM_Cost')}.</p></section>
<script type="importmap">{{"imports":{{"three":"https://cdn.jsdelivr.net/npm/three@0.160.0/build/three.module.js","three/addons/":"https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/"}}}}</script>
<script type="module" src="assets/app.js"></script>"""
katex_idx = ('<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css">'
             '<script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.js"></script>')
(ROOT / "docs/index.html").write_text(page("Ridgeback Scout", "index.html", index_body, katex_idx))

# ------------------------------------------------------------------ mission and range
mission_body = f"""
<figure class="scene" aria-label="Sketch of one survey flight over a hillside">
<svg viewBox="0 0 1600 640" preserveAspectRatio="xMidYMid slice" role="img">
<image href="assets/contours.svg" x="0" y="-130" width="1600" height="900"/>
<circle cx="560" cy="420" r="330" class="ring"/>
<text x="560" y="78" class="sk" text-anchor="middle">1 km: the drone stays in sight</text>
<rect x="690" y="250" width="150" height="150" class="plot"/>
<path class="lines" d="{''.join(f"M{690 + i * 150 / 17:.1f},{250 if i % 2 == 0 else 400} L{690 + i * 150 / 17:.1f},{400 if i % 2 == 0 else 250} " for i in range(18))}"/>
<path class="leg" d="M560,420 L690,400 M840,250 L560,420"/>
<g class="to"><circle cx="560" cy="420" r="10"/><line x1="548" y1="408" x2="572" y2="432"/><line x1="572" y1="408" x2="548" y2="432"/></g>
<text x="575" y="455" class="sk">take-off and landing</text>
<text x="765" y="235" class="sk" text-anchor="middle">2 ha plot, 18 survey lines</text>
<text x="1290" y="560" class="sk quiet" text-anchor="middle">sketch, not to scale</text>
</svg></figure>
<section class="lead"><h1>Mission and range</h1>
<p>We plan around an operating radius of about 1 km, set by the rule that the drone must stay in sight.
So the motors and battery are sized for hover thrust and for time over the plants, not for distance.</p></section>
<section class="mapsec"><h2>Where fire meets our flights</h2>
<p>Every wildfire over 50 acres since 2000 within about 40 km of Stanford, and the prescribed burns on record. In August 2020 the CZU Lightning Complex
burned 86,553 acres of the Santa Cruz Mountains, about 25 km from Jasper Ridge. Prescribed burns are the tool for lowering that risk, and they
are planned around how dry the living plants are.</p>
<div class="mapbar" role="group" aria-label="Map view"><button type="button" data-view="region" aria-pressed="true">Fires since 2000</button><button type="button" data-view="site" aria-pressed="false">One survey flight at Jasper Ridge</button></div>
<div id="map" aria-label="Map of fire perimeters near Stanford and an example survey flight at Jasper Ridge"></div>
<div class="maplegend"><span><i class="sw fire"></i>Wildfire, 2000–2024</span><span><i class="sw rx"></i>Prescribed burn</span><span><i class="sw line"></i>Jasper Ridge Biological Preserve</span><span><i class="sw ring"></i>1 km line-of-sight radius</span><span><i class="sw plot"></i>2 ha survey plot</span><span><i class="sw path"></i>Flight path</span></div>
<p class="small">Wildfire perimeters: CAL FIRE historic fire perimeters. Prescribed burns: NIFC interagency fire perimeter history. Preserve boundary: OpenStreetMap.
The take-off point and plot are placed for scale only; the real plot will be chosen with the preserve.</p></section>
<section><h2>Operating radius, range and endurance</h2>
<table class="plain"><thead><tr><th>Term</th><th class="num">Ours</th><th>What sets it</th></tr></thead><tbody>
<tr><td>Operating radius: how far from the take-off point</td><td class="num">about 1 km</td><td>FAA rule 14 CFR 107.31: the pilot or an observer must see the drone at all times
(<a href="https://www.ecfr.gov/current/title-14/chapter-I/subchapter-F/part-107/subpart-B/section-107.31" target="_blank" rel="noopener">text of the rule</a>). The rule sets no distance; 1 km is how far we expect to see a 2.2 m drone reliably.</td></tr>
<tr><td>Range: distance on one charge</td><td class="num">{val('range_km'):.0f} km</td><td>Loiter speed × loiter time.</td></tr>
<tr><td>Endurance: time in the air</td><td class="num">{val('tloit_sl'):.0f} min</td><td>Battery energy against drag. This is the number that matters for the mission.</td></tr>
</tbody></table></section>
<section><h2>A field day</h2>
<p>Water loss shows up as the difference between dawn and midday. We target three 15-minute surveys of the same 2 ha chaparral plot on one charge,
within 1 km of the take-off point. The current estimate is {val('flights_pack'):.1f} flights per charge.</p>
<svg class="day" viewBox="0 0 900 120" role="img" aria-label="Three flights: pre-dawn, mid-morning, midday">
<line x1="40" y1="70" x2="860" y2="70" class="axis"/>
{''.join(f'<line x1="{x}" y1="64" x2="{x}" y2="76" class="tick"/><text x="{x}" y="96" class="t">{h}</text>' for x, h in [(40, "5:00"), (177, "7:00"), (313, "9:00"), (450, "11:00"), (587, "13:00"), (723, "15:00"), (860, "17:00")])}
{''.join(f'<rect x="{40 + (h - 5) * 68.33:.0f}" y="48" width="{0.25 * 68.33:.0f}" height="22" class="fl"/><text x="{40 + (h - 5 + 0.125) * 68.33:.0f}" y="38" class="l">{lab}</text>' for h, lab in [(6.25, "pre-dawn, 6:15"), (9.75, "mid-morning, 9:45"), (12.75, "midday, 12:45")])}
</svg>
<table class="plain"><tbody>
<tr><td>Plot</td><td>2 ha of chaparral plus a 50 m buffer: 18 lines of 240 m, about 7 minutes at 19 m/s, plus hovering for the spot sensor</td></tr>
<tr><td>Energy per flight</td><td><span class="num">{val('E_flight'):.0f} Wh</span>: hover, climb, 15 min survey, 1 km out and back</td></tr>
<tr><td>Usable energy per charge</td><td><span class="num">{val('E_use'):.0f} Wh</span>, keeping 20 % in reserve</td></tr>
<tr><td>Flights per charge</td><td><span class="num">{val('flights_pack'):.1f}</span>, against the 3 we target</td></tr>
</tbody></table>
<p class="small">Numbers from {sheet('04_Propulsion_Battery_Energy', 'Mission_Energy')}. The plot size and flight plan come from our field-day plan (base station document, October 2026).</p></section>
<section><h2>Sources behind the numbers</h2>
<h3 class="refh">Why measure plant water at all</h3>
<ul class="refs">
<li><a href="https://doi.org/10.1071/WF08055" target="_blank" rel="noopener">Dennison and Moritz, Int. J. Wildland Fire 18:1021 (2009)</a>: "strong evidence for a LFM threshold near 79%" for fire in chaparral.</li>
<li><a href="https://doi.org/10.3390/fire5040110" target="_blank" rel="noopener">Park, Fauss and Moritz, Fire 5:110 (2022)</a>: burned area and number of fires "increased dramatically when LFM fell below 62%" in chamise.</li>
</ul>
<h3 class="refh">Why dawn and midday</h3>
<ul class="refs">
<li><a href="https://doi.org/10.1111/gcb.12477" target="_blank" rel="noopener">Pratt et al., Global Change Biology 20:893 (2014)</a>: chaparral shrubs sampled "at predawn when plants were at their most hydrated and at midday … when their tissues would be maximally dehydrated", the same pair of times we fly.</li>
<li><a href="https://doi.org/10.5194/bg-20-1789-2023" target="_blank" rel="noopener">Humphrey and Frankenberg, Biogeosciences 20:1789 (2023)</a>: "28 % of the pre-dawn canopy water content is depleted over the course of the day", measured at a forested site in southern California, not in chaparral.</li>
<li><a href="https://doi.org/10.3390/rs11161853" target="_blank" rel="noopener">Easterday et al., Remote Sensing 11:1853 (2019)</a>: drone imagery of coyote brush separated water stress best "in the morning".</li>
<li><a href="https://gacc.nifc.gov/rmcc/predictive/fuels_fire-danger/live_fm/Live_moisture_sampling_procedures.pdf" target="_blank" rel="noopener">NIFC live fuel moisture sampling procedures</a>: "Samples should be collected between 1100 and 1600 hours", so our midday flight can be compared with agency samples.</li>
</ul>
<h3 class="refh">Why about 1 km</h3>
<ul class="refs">
<li><a href="https://www.law.cornell.edu/cfr/text/14/107.31" target="_blank" rel="noopener">14 CFR 107.31</a>: the pilot or observer must be able to "determine the unmanned aircraft's attitude, altitude, and direction of flight".</li>
<li><a href="https://doi.org/10.1016/j.apergo.2019.102898" target="_blank" rel="noopener">Li et al., Applied Ergonomics 81:102898 (2019)</a> and <a href="https://doi.org/10.3390/app10165501" target="_blank" rel="noopener">Li et al., Applied Sciences 10:5501 (2020)</a>:
small quadcopters were seen at 245–307 m, a visual angle of about 0.065°. At that angle our 2.24 m wingspan would be seen at about 2 km (our scaling, not a measurement), so 1 km leaves margin for judging which way it is pointing.</li>
</ul>
<h3 class="refh">Survey lines and overlap</h3>
<ul class="refs">
<li><a href="https://web.archive.org/web/2023/https://support.micasense.com/hc/en-us/articles/224893167" target="_blank" rel="noopener">MicaSense guidance</a>: "75% or higher overlap in both directions" for multispectral mapping, the basis of our 14 m line spacing.</li>
<li><a href="https://support.pix4d.com/hc/en-us/articles/202557459" target="_blank" rel="noopener">Pix4D image acquisition plans</a>: thermal needs "90% front and side image overlap"; dense vegetation at least 85 %. Our plan at 75 % is too sparse for the thermal camera: an open item that adds survey time.</li>
</ul>
<h3 class="refh">How the drone is sized</h3>
<ul class="refs">
<li><a href="https://doi.org/10.1016/j.jestch.2020.02.002" target="_blank" rel="noopener">Dündar, Bilici and Ünler, Eng. Sci. Tech. 23:1182 (2020)</a>: a battery VTOL with four lift rotors, sized with momentum theory for hover and cruise power for endurance, the same method as our sheets.</li>
<li><a href="https://doi.org/10.3390/aerospace9020071" target="_blank" rel="noopener">An et al., Aerospace 9:71 (2022)</a>: sizing method for lift-plus-cruise electric VTOL drones.</li>
<li>Textbooks: Raymer, Aircraft Design: A Conceptual Approach (AIAA, 2018), for drag buildup and wing CLmax; Leishman, Principles of Helicopter Aerodynamics (Cambridge, 2005), for hover momentum theory; Gundlach, Designing Unmanned Aircraft Systems (AIAA, 2014).</li>
</ul></section>
<section><h2>People we talked to</h2>
<table class="plain"><tbody>
<tr><td>A Stanford geophysics lab that flies a thermal drone at Jasper Ridge</td><td>Video call on 16 Nov about their camera and flights. Their typical flight distance and duration: still to ask.</td></tr>
<tr><td>A UNLV researcher who flew a SWIR hyperspectral camera on a drone</td><td>Offered to share what they learned setting it up and flying it.</td></tr>
<tr><td>Jasper Ridge Biological Preserve research staff</td><td>A camera bought for field work fits their student research grant; meeting to be set up. Flight permission at the preserve goes through them.</td></tr>
</tbody></table>
<p class="small">Names left out on purpose on this public page.</p></section>
<section><h2>Still to find out</h2>
<ul class="plain">
<li>How far and how long the Jasper Ridge drone flights run, from the lab that flies there.</li>
<li>Whether 15 minutes over 2 ha is enough once the thermal camera's 90 % overlap and the spot sensor's hover points are planned.</li>
<li>Whether the preserve allows take-off within 1 km of the plots we want.</li>
</ul></section>"""
leaflet = ('<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">'
           '<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>')
mission_body += '<script type="module" src="assets/mission_map.js"></script>'
(ROOT / "docs/mission.html").write_text(page("Ridgeback mission and range", "mission.html", mission_body, leaflet))

# ------------------------------------------------------------------ calculations with rendered equations
def step(title, why, tex, numbers, result, where):
    sid = "step-" + title.split(".")[0]
    return (f'<section class="calc" id="{sid}"><h2>{title}</h2><p>{why}</p><div class="eq">\\[{tex}\\]</div>'
            f'<div class="eq sub">\\[{numbers}\\]</div><p class="res">{result}</p><p class="small">In the sheets: {where}</p></section>')


g = 9.80665
m = val("m_to"); W = m * g; S = 0.5
rho = val("rho_sl"); CLmax = val("CLmax"); Vs = val("Vs_sl"); K = val("K"); CD0 = val("CD0_build"); V = val("V_sl")
CL = val("CL_sl"); CD = val("CD_sl"); D = val("D_sl"); Pcr = val("Pcr_sl")
calc_body = "".join([
    '<section class="lead"><h1>Calculations</h1><p>Each step of the sizing, with the equation, our numbers and where the step lives in the sheets. Sea level, standard day, unless noted.</p></section>',
    step("1. Weight", "Everything the rotors and the wing must carry: the CAD parts plus the sensor payload.",
         r"W = m\,g", rf"W = {m:.2f}\ \text{{kg}} \times 9.81\ \text{{m/s}}^2", f"W = {W:.1f} N", sheet("02_Mass_and_CG", "Mass_CG")),
    step("2. Stall speed", "The slowest the wing can fly. The wing's maximum lift coefficient is 0.9 times the SD7037 airfoil's (Raymer).",
         r"V_s = \sqrt{\dfrac{2W}{\rho\, S\, C_{L,\max}}}", rf"V_s = \sqrt{{\dfrac{{2 \times {W:.1f}\ \text{{N}}}}{{{rho:.3f}\ \text{{kg/m}}^3 \times {S:.2f}\ \text{{m}}^2 \times {CLmax:.3f}}}}}",
         f"V<sub>s</sub> = {Vs:.1f} m/s, against the 15 m/s we wanted.", sheet("03_Aerodynamics", "Aero_Cruise")),
    step("3. Drag of the whole drone", "Summed over every component: skin friction on the wing, tail, fuselage, booms and nacelles, and pressure drag on the motors, feet and stopped propellers.",
         r"C_{D0} = \dfrac{\sum C_f\, FF\, Q\, S_{wet} + \sum C_d\, A_{front}}{S}", rf"C_{{D0}} = {CD0:.4f}",
         "The motors, landing feet and stopped propellers make about 45 % of it.", sheet("03_Aerodynamics", "Drag_Buildup")),
    step("4. Loiter speed and drag", "The drone loiters at 1.3 times the stall speed, which here is faster than its minimum-power speed.",
         r"\begin{aligned} V &= 1.3\,V_s \\ C_L &= \dfrac{2W}{\rho V^2 S} \\ C_D &= C_{D0} + K C_L^2 \\ D &= \tfrac12 \rho V^2 S\, C_D \end{aligned}",
         rf"V = {V:.1f},\quad C_L = {CL:.3f},\quad C_D = {CD0:.4f} + {K:.4f}\times{CL:.3f}^2 = {CD:.4f}",
         f"Drag D = {D:.1f} N at {V:.1f} m/s.", sheet("03_Aerodynamics", "Aero_Cruise")),
    step("5. Loiter power", "Thrust power through the pusher (efficiency η = 0.55, assumed), plus 20 W for the electronics.",
         r"P = \dfrac{D\,V}{\eta} + P_{av}", rf"P = \dfrac{{{D:.1f} \times {V:.1f}}}{{0.55}} + 20", f"P = {Pcr:.0f} W", sheet("03_Aerodynamics", "Aero_Cruise")),
    step("6. Hover power", "From T-Motor's own MN5008 test table, fitted as a power law and corrected for air density.",
         r"P_{motor} = e^{c}\, T^{k} \sqrt{\rho_{test}/\rho},\qquad P_{hover} = 4P_{motor} + P_{av}",
         rf"T = {val('Thov_sl'):,.0f}\ \text{{g per motor}},\quad k = {val('fit_k'):.3f}",
         f"P<sub>hover</sub> = {val('Phov_sl'):,.0f} W at sea level, {val('Phov_hot'):,.0f} W on a hot day at 800 m.", sheet("04_Propulsion_Battery_Energy", "Lift_Hover")),
    step("7. Thrust to weight", "Can the four rotors hold the drone steady in gusts on a hot day at 800 m? We require at least 1.5.",
         r"\dfrac{T}{W} = \dfrac{4\,T_{max}\,(V_{load}/V_{test})^2\,(\rho/\rho_{test})}{W}",
         rf"\dfrac{{T}}{{W}} = \dfrac{{4 \times {val('Tmax_hot'):.1f}\ \text{{N}}}}{{{W:.1f}\ \text{{N}}}}",
         f"T/W = {val('TW_hot'):.2f} on the hot day ({val('TW_sl'):.2f} at sea level).", sheet("04_Propulsion_Battery_Energy", "Lift_Hover")),
    step("8. Energy and flights per charge", "Usable energy keeps the cells between 95 % and 10 % charge and leaves 20 % in reserve.",
         r"E_u = E_{pack}\times 0.85 \times 0.8,\qquad n = \dfrac{E_u}{E_{hover} + E_{climb} + P\,(t_{survey} + t_{transit})}",
         rf"E_u = 648 \times 0.85 \times 0.8 = {val('E_use'):.0f}\ \text{{Wh}},\quad n = \dfrac{{{val('E_use'):.0f}}}{{{val('E_flight'):.0f}}}",
         f"{val('flights_pack'):.1f} flights per charge; loiter time {val('tloit_sl'):.0f} min.", sheet("04_Propulsion_Battery_Energy", "Mission_Energy")),
    step("9. Range", "Distance on one charge at loiter speed. Shown to make the point that it never limits us.",
         r"\text{range} = V\, t_{loiter}", rf"\text{{range}} = {V:.1f}\ \text{{m/s}} \times {val('tloit_sl') * 60:,.0f}\ \text{{s}}",
         f"{val('range_km'):.0f} km, against a 1 km operating radius.", sheet("04_Propulsion_Battery_Energy", "Mission_Energy")),
])
katex = ('<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css">'
         '<script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.js"></script>'
         '<script defer src="https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/contrib/auto-render.min.js" onload="renderMathInElement(document.body)"></script>')
(ROOT / "docs/calculations.html").write_text(page("Ridgeback calculations", "calculations.html", calc_body, katex))

# ------------------------------------------------------------------ files
sheets = [("01_Overview_Requirements_Decisions", "Requirements, summary of results, decisions and why, open checks"),
          ("02_Mass_and_CG", "Mass and position of every CAD part, centre of gravity"),
          ("03_Aerodynamics", "Atmosphere, airfoil, drag of every part, loiter speed and power"),
          ("04_Propulsion_Battery_Energy", "Hover motors, pusher, battery options, energy and flights per charge"),
          ("05_Structure_and_Stability", "Spar, boom, servo and stability checks"),
          ("06_Electronics_BOM_Cost", "Electronics, connectors and wiring, full parts list with duty, ways to save")]
srows = "".join(f'<tr><td>{sheet(n)}</td><td>{E(d)}</td><td><a href="sheets/Ridgeback_{n}.xlsx">.xlsx</a></td></tr>' for n, d in sheets)
files_body = f"""
<section class="lead"><h1>Sheets, code and CAD</h1><p>Everything behind the numbers on this site. The sheets and pages are rebuilt from two plain files, so a change made there is permanent and shows up everywhere.</p></section>
<section id="edit"><h2>Change a value or a link</h2>
<table class="plain"><thead><tr><th>To change</th><th>Edit this file</th></tr></thead><tbody>
<tr><td>A part: price, quantity, mass, shop link, why we chose it, what it connects to</td><td><a href="{GH}/edit/main/data/parts.toml">data/parts.toml</a></td></tr>
<tr><td>A design number: requirements, air, wing, drag, motors, battery, mission (60 typed numbers)</td><td><a href="{GH}/edit/main/data/inputs.toml">data/inputs.toml</a></td></tr>
</tbody></table>
<ol class="steps"><li>Open the file (link above) and sign in to GitHub. Each line says what the number is and where it comes from.</li>
<li>Change the value and press <b>Commit changes</b>. That saves it for good, with your name and the date in the history.</li>
<li>About 2 minutes later the six sheets, every calculation and these pages are rebuilt from it. Progress: <a href="{GH}/actions">Actions</a>. If a value breaks a formula or a name is mistyped, the rebuild stops, the site keeps the old version and the Actions page says why.</li></ol>
<p class="small">Saving needs write access to the repository (ask a lead); anyone else can propose the change and a lead accepts it. Edits made inside the Google Sheets copies are not kept: the next rebuild replaces them. Numbers in the long tables (drag of each part, the motor maker's test table, connector ratings) are still in <a href="{GH}/blob/main/sheets/build_sheet.py">sheets/build_sheet.py</a>.</p></section>
<section><h2>Calculation sheets</h2>
<table class="plain"><thead><tr><th>Sheet</th><th>What it answers</th><th>Download</th></tr></thead><tbody>{srows}</tbody></table>
<p class="small">The Google Sheets live in the team Drive; ask a lead for access. The .xlsx copies here open in Excel, Numbers or Google Sheets.</p></section>
<section><h2>Code</h2>
<table class="plain"><tbody>
<tr><td><a href="https://github.com/Stanford-SkyRunners/ridgeback-design/tree/main/sheets">sheets/</a></td><td>Builds every sheet with live formulas, splits it into the six files and checks every number.</td></tr>
<tr><td><a href="https://github.com/Stanford-SkyRunners/ridgeback-design/tree/main/analysis">analysis/</a></td><td>Sizing sweep, spar, boom, servo, wiring, filament, duty and stability scripts.</td></tr>
<tr><td><a href="https://github.com/Stanford-SkyRunners/ridgeback-design/tree/main/cad">cad/</a></td><td>The CAD as code (build123d): every printed part, the assembly, the payload fit.</td></tr>
<tr><td><a href="https://github.com/Stanford-SkyRunners/ridgeback-design/tree/main/tools">tools/</a></td><td>This site: the parts list, the 3D model export and the page generator.</td></tr>
</tbody></table></section>
<section><h2>CAD files</h2>
<table class="plain"><tbody>
<tr><td><a href="cad/Ridgeback_print_pack_STL.zip">Print pack, STL (5 MB)</a></td><td>Every printed part in print orientation, with print settings.</td></tr>
<tr><td><a href="https://github.com/Stanford-SkyRunners/ridgeback-design/releases">STEP pack for Fusion 360 (82 MB)</a></td><td>Every part and sub-assembly, attached to the latest release.</td></tr>
<tr><td><a href="assets/ridgeback.glb">3D model, GLB</a></td><td>The model on the drone page, viewable in any glTF viewer.</td></tr>
</tbody></table></section>
<section><h2>Rebuild on your own computer</h2>
<pre>pip install openpyxl formulas
tools/rebuild.sh     # sheets, checks (0 errors, split files equal the full model) and this site</pre></section>"""
(ROOT / "docs/files.html").write_text(page("Ridgeback sheets, code and CAD", "files.html", files_body))
print("site written")
