// Drone explorer: click a part on the model or in the table; the model highlights it and draws its connections.
import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const data = await (await fetch('assets/parts.json')).json();
const parts = Object.fromEntries(data.parts.map(p => [p.id, p]));
const nodeToPart = {};
const prefixes = [];
data.parts.forEach(p => p.nodes.forEach(n => { if (n.endsWith('*')) prefixes.push([n.slice(0, -1), p.id]); else nodeToPart[n] = p.id; }));
const lookup = n => nodeToPart[n] || (prefixes.find(([pre]) => n.startsWith(pre)) || [])[1];
const css = getComputedStyle(document.documentElement);
const PURPLE = new THREE.Color(css.getPropertyValue('--purple').trim() || '#7b3fa0');

const el = document.getElementById('view');
const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
el.appendChild(renderer.domElement);
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(32, 1, 0.05, 50);
camera.position.set(-2.4, 1.9, 2.6);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true; controls.target.set(0.25, 0, 0);
scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8678, 2.2));
const sun = new THREE.DirectionalLight(0xffffff, 1.6); sun.position.set(-3, 5, 2); scene.add(sun);

const root = new THREE.Group(); root.rotation.x = -Math.PI / 2; scene.add(root); // CAD z up -> screen up
const meshesByPart = {}; const pickables = []; const base = new Map();
const gltf = await new GLTFLoader().loadAsync('assets/ridgeback.glb');
root.add(gltf.scene);
gltf.scene.traverse(o => {
  if (!o.isMesh) return;
  if (!o.geometry.attributes.normal) o.geometry.computeVertexNormals();
  const raw = o.name || o.parent?.name || '';            // GLTFLoader turns ':' into '_'
  const isAir = /^airframe/.test(raw);
  const name = raw.replace(/^(part|airframe)[_:]?/, "");
  const full = isAir ? 'airframe:' + name : name;
  const pid = lookup(full) || lookup(name);
  if (full === 'airframe:prop') { o.visible = false; return; }
  const sys = pid ? parts[pid].system : 'airframe';
  o.userData.sys = sys;
  const col = new THREE.Color(data.systems[sys].color);
  o.material = new THREE.MeshStandardMaterial({ color: col, roughness: 0.85, metalness: 0.0, flatShading: sys !== 'airframe', transparent: true, opacity: sys === 'airframe' ? 0.92 : 1 });
  if (name.startsWith('prop_')) { o.material.opacity = 0.35; o.material.depthWrite = false; o.userData.disc = true; }
  base.set(o, { color: col.clone(), opacity: o.material.opacity });
  if (pid) { (meshesByPart[pid] ||= []).push(o); pickables.push(o); o.userData.pid = pid; }
});
const box = new THREE.Box3().setFromObject(root); const c = box.getCenter(new THREE.Vector3());
controls.target.copy(c); camera.position.copy(c).add(new THREE.Vector3(-1.75, 2.35, -1.55));

const centerOf = pid => { const b = new THREE.Box3(); (meshesByPart[pid] || []).forEach(m => b.expandByObject(m)); return b.isEmpty() ? null : b.getCenter(new THREE.Vector3()); };
let lines = null;
function select(pid, fromHash = false) {
  base.forEach((b, m) => {
    m.material.color.copy(b.color); m.material.emissive.set(0x000000); m.material.depthTest = true; m.renderOrder = 0;
    m.material.opacity = !pid ? b.opacity : (m.userData.sys === 'airframe' && m.userData.pid !== pid ? 0.14 : Math.min(b.opacity, 0.3));
  });
  if (lines) { scene.remove(lines); lines = null; }
  document.querySelectorAll('tr[data-id]').forEach(r => r.classList.toggle('sel', r.dataset.id === pid));
  if (!pid) return;
  const p = parts[pid]; const pts = [];
  const ctr = m => new THREE.Box3().setFromObject(m).getCenter(new THREE.Vector3());
  p.connects.forEach(([to]) => {
    const targets = (meshesByPart[to] || []).map(m => { m.material.opacity = base.get(m).opacity; return ctr(m); });
    (meshesByPart[pid] || []).forEach(m => {            // one line from each copy of this part to the nearest connected part
      const a = ctr(m); let best = null, d = Infinity;
      targets.forEach(t => { const dd = a.distanceTo(t); if (dd < d) { d = dd; best = t; } });
      if (best && d > 1e-4) pts.push(a, best);
    });
  });
  (meshesByPart[pid] || []).forEach(m => { m.material.color.copy(PURPLE); m.material.emissive.copy(PURPLE).multiplyScalar(0.75); m.material.opacity = 1; m.material.depthTest = false; m.renderOrder = 10; });
  if (pts.length) { const lm = new THREE.LineBasicMaterial({ color: PURPLE, depthTest: false }); lines = new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(pts), lm); lines.renderOrder = 11; scene.add(lines); }
  showDetail(p);
  if (!fromHash) history.replaceState(null, '', '#' + pid);
}
function esc(s) { return String(s).replace(/[&<>"]/g, ch => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[ch])); }
function showDetail(p) {
  const price = p.status.startsWith('reuse') ? 'owned' : (p.price ? `$${p.price.toFixed(2)} each` : '');
  const conn = p.connects.length ? '<ul>' + p.connects.map(([to, how]) => `<li><button class="link" data-go="${to}">${esc(parts[to]?.name || to)}</button>: ${esc(how)}</li>`).join('') + '</ul>' : '<p>Nothing wired; it runs on its own.</p>';
  const buy = p.buy.length ? '<ul>' + p.buy.map(([s, u, pr]) => `<li><a href="${esc(u)}" target="_blank" rel="noopener">${esc(s)}</a>, ${esc(pr)}</li>`).join('') + '</ul>' : '<p>Not bought.</p>';
  document.getElementById('detail').innerHTML = `<h2>${esc(p.name)}</h2>
   <div class="meta"><span class="key" style="--c:${p.color}"></span>${esc(p.systemName)}. ${esc(p.status)}. ${p.qty} × ${price}${price ? ', ' : ''}${p.mass} g each.</div>
   <h3>What it does</h3><p>${esc(p.job)}</p><h3>Why we chose it</h3><p>${esc(p.why)}</p>
   <h3>Connects to</h3>${conn}<h3>Where to buy</h3>${buy}
   ${p.instead ? `<h3>Considered instead</h3><p>${esc(p.instead)}</p>` : ''}${p.source ? `<p class="small">Numbers from: ${esc(p.source)}</p>` : ''}<p class="small"><a href="#row-${p.id}" data-row="${p.id}">Show in the parts table</a></p>`;
  document.querySelectorAll('#detail [data-go]').forEach(b => b.addEventListener('click', () => select(b.dataset.go)));
}
document.querySelectorAll('tr[data-id]').forEach(r => {
  const go = () => { select(r.dataset.id); el.scrollIntoView({ block: 'nearest', behavior: 'smooth' }); };
  r.addEventListener('click', go); r.addEventListener('keydown', e => { if (e.key === 'Enter') go(); });
});
const ray = new THREE.Raycaster(); const ptr = new THREE.Vector2(); let down = null;
renderer.domElement.addEventListener('pointerdown', e => { down = [e.clientX, e.clientY]; });
renderer.domElement.addEventListener('pointerup', e => {
  if (!down || Math.hypot(e.clientX - down[0], e.clientY - down[1]) > 5) return;
  const r = renderer.domElement.getBoundingClientRect();
  ptr.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
  ray.setFromCamera(ptr, camera); const hit = ray.intersectObjects(pickables, false).find(h => !h.object.userData.disc);
  select(hit ? hit.object.userData.pid : null);
});
function size() { const w = el.clientWidth, h = el.clientHeight; renderer.setSize(w, h); camera.aspect = w / h; camera.updateProjectionMatrix(); }
new ResizeObserver(size).observe(el); size();
renderer.setAnimationLoop(() => { controls.update(); renderer.render(scene, camera); });
const fromHash = () => { const id = location.hash.slice(1); if (parts[id]) select(id, true); };
addEventListener('hashchange', fromHash); fromHash();
