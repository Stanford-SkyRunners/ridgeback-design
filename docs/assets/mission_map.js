// Mission map: fires around Stanford since 2000, and one survey flight at Jasper Ridge drawn to scale.
const css = getComputedStyle(document.documentElement);
const C = n => css.getPropertyValue(n).trim();
const map = L.map('map', { zoomControl: true, scrollWheelZoom: false, attributionControl: true });
map.setView([37.35, -122.2], 10);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/Canvas/World_Light_Gray_Base/MapServer/tile/{z}/{y}/{x}',
  { maxZoom: 16, attribution: 'Base: Esri, HERE, Garmin, USGS' }).addTo(map);
L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/Elevation/World_Hillshade/MapServer/tile/{z}/{y}/{x}',
  { maxZoom: 16, opacity: 0.45, attribution: 'Hillshade: Esri, USGS' }).addTo(map);
const label = (ll, text, cls = 'maplabel') => L.tooltip({ permanent: true, direction: 'center', className: cls, interactive: false }).setLatLng(ll).setContent(text).addTo(map);
label([37.18, -122.55], 'Pacific Ocean', 'maplabel water'); label([37.56, -122.17], 'San Francisco Bay', 'maplabel water');
label([37.33, -122.25], 'Santa Cruz Mountains', 'maplabel quiet');
label([37.392, -122.262], 'Jasper Ridge', 'maplabel');

const [fires, jrbp] = await Promise.all(['assets/map/fires.geojson', 'assets/map/jrbp.geojson'].map(u => fetch(u).then(r => r.json())));
const fireLayer = L.geoJSON(fires, {
  style: f => f.properties.kind === 'wildfire'
    ? { color: C('--red'), weight: 1.2, fillColor: C('--red'), fillOpacity: 0.22 }
    : { color: C('--brown'), weight: 1.2, fillColor: C('--brown'), fillOpacity: 0.35, dashArray: '3 3' },
  onEachFeature: (f, l) => { if (f.properties.kind === 'wildfire' && f.properties.acres > 50000) l.on('add', () => label(l.getBounds().getCenter(), `${f.properties.name.replace(/^(Czu|Scu)/, m => m.toUpperCase())}, ${f.properties.year}`, 'maplabel fire')); },
  onEachFeatureTip: null
}).addTo(map);
fireLayer.eachLayer(l => { const f = l.feature; l.bindTooltip(`${f.properties.name}, ${f.properties.year}<br>${f.properties.acres.toLocaleString()} acres, ${f.properties.kind === 'wildfire' ? 'wildfire' : 'prescribed burn'}`, { sticky: true }); });
fireLayer.eachLayer(l => { const t = l.getTooltip(); if (t) t.setContent(t.getContent().replace(/^(Czu|Scu)/, m => m.toUpperCase())); });
const preserve = L.geoJSON(jrbp, { style: { color: C('--ink'), weight: 2, fill: false } }).addTo(map)
  .bindTooltip('Jasper Ridge Biological Preserve', { sticky: true });
L.circleMarker([37.4275, -122.1697], { radius: 4, color: C('--ink'), fillColor: C('--ink'), fillOpacity: 1 }).addTo(map)
  .bindTooltip('Stanford main quad', { permanent: true, direction: 'right', className: 'maplabel' });

// one survey flight, to scale (local metres around the take-off point)
const T0 = [37.4049, -122.2343];                       // example take-off point near the preserve centre, for scale only
const mPerDegLat = 111320, mPerDegLon = 111320 * Math.cos(T0[0] * Math.PI / 180);
const at = (dx, dy) => [T0[0] + dy / mPerDegLat, T0[1] + dx / mPerDegLon];
const site = L.layerGroup();
L.circle(T0, { radius: 1000, color: C('--blue'), weight: 1.5, dashArray: '6 6', fill: false }).addTo(site)
  .bindTooltip('1 km: how far we expect to keep the drone in sight', { sticky: true });
L.circleMarker(T0, { radius: 6, color: C('--ink'), fillColor: C('--paper'), fillOpacity: 1, weight: 2 }).addTo(site)
  .bindTooltip('Take-off and landing (example)', { permanent: true, direction: 'left', className: 'maplabel' });
const cx = 420, cy = 180, half = 120.5;                 // plot centre 420 m east, 180 m north; 2 ha + 50 m buffer = 241 m square
L.rectangle([at(cx - half, cy - half), at(cx + half, cy + half)], { color: C('--green'), weight: 1.5, fillColor: C('--green'), fillOpacity: 0.12 }).addTo(site)
  .bindTooltip('Survey plot: 2 ha of chaparral plus a 50 m buffer', { sticky: true });
const pts = [at(0, 0)];
const n = 18, step = (2 * half) / (n - 1);
for (let i = 0; i < n; i++) {
  const x = cx - half + i * step, y0 = cy - half, y1 = cy + half;
  if (i % 2 === 0) { pts.push(at(x, y0), at(x, y1)); } else { pts.push(at(x, y1), at(x, y0)); }
}
pts.push(at(0, 0));
const path = L.polyline(pts, { color: C('--purple'), weight: 2 }).addTo(site);
path.bindTooltip('Flight path: 18 lines, 14 m apart, about 7 minutes at 19 m/s', { sticky: true });

const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
function drawPath() {                                 // trace the flight path once when the site view opens
  const el = path.getElement(); if (!el || reduce) return;
  const len = el.getTotalLength(); el.style.transition = 'none'; el.style.strokeDasharray = len; el.style.strokeDashoffset = len;
  el.getBoundingClientRect(); el.style.transition = 'stroke-dashoffset 6s linear'; el.style.strokeDashoffset = 0;
}
const views = {
  region: () => { map.removeLayer(site); fireLayer.addTo(map); map.flyToBounds(fireLayer.getBounds().pad(0.05), { duration: reduce ? 0 : 1.2 }); },
  site: () => { site.addTo(map); map.flyTo(at(200, 60), 15, { duration: reduce ? 0 : 1.6 }); map.once('moveend', drawPath); }
};
document.querySelectorAll('[data-view]').forEach(b => b.addEventListener('click', () => {
  document.querySelectorAll('[data-view]').forEach(x => x.setAttribute('aria-pressed', x === b));
  views[b.dataset.view]();
}));
map.fitBounds(fireLayer.getBounds().pad(0.05));
