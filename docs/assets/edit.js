// Edit the site in place. Changes are saved to the GitHub repository (data/parts.toml, data/inputs.toml) with the
// editor's own GitHub token; GitHub then rebuilds the sheets and the pages (.github/workflows/rebuild.yml).
const REPO = 'Stanford-SkyRunners/ridgeback-design';
const API = `https://api.github.com/repos/${REPO}`;
const TOKEN_URL = 'https://github.com/settings/tokens/new?scopes=public_repo&description=Ridgeback%20site%20editing';
const store = {
  get(k) { try { return localStorage.getItem(k); } catch { return null; } },
  set(k, v) { try { v == null ? localStorage.removeItem(k) : localStorage.setItem(k, v); } catch { /* private window */ } },
};
let token = store.get('rb-token') || sessionToken();
function sessionToken() { try { return sessionStorage.getItem('rb-token'); } catch { return null; } }

const btn = document.getElementById('editToggle');
const bar = document.getElementById('editbar');
const esc = s => String(s ?? '').replace(/[&<>"]/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;' }[c]));
let editing = false;

// ---------- GitHub
async function gh(path, opts = {}) {
  const r = await fetch(API + path, { ...opts, headers: { Accept: 'application/vnd.github+json', Authorization: `Bearer ${token}`, ...(opts.headers || {}) } });
  if (!r.ok) { const t = await r.text(); const e = new Error(`${r.status} ${t.slice(0, 200)}`); e.status = r.status; throw e; }
  return r.json();
}
const b64dec = s => new TextDecoder().decode(Uint8Array.from(atob(s.replace(/\n/g, '')), c => c.charCodeAt(0)));
const b64enc = s => { const b = new TextEncoder().encode(s); let x = ''; b.forEach(c => { x += String.fromCharCode(c); }); return btoa(x); };
async function getFile(path) { const f = await gh(`/contents/${path}?ref=main&t=${Date.now()}`); return { sha: f.sha, text: b64dec(f.content) }; }
async function saveFile(path, change, message) {          // change(text) -> new text; retries once if someone saved in between
  for (let i = 0; i < 2; i++) {
    const f = await getFile(path);
    const text = change(f.text);
    if (text === f.text) return null;
    try {
      const r = await gh(`/contents/${path}`, { method: 'PUT', body: JSON.stringify({ message, content: b64enc(text), sha: f.sha, branch: 'main' }) });
      return r.commit.sha;
    } catch (e) { if (e.status !== 409 || i) throw e; }
  }
}

// ---------- TOML for one part (same layout as the file)
const tv = v => Array.isArray(v) ? '[' + v.map(tv).join(', ') + ']'
  : (v && typeof v === 'object') ? '{ ' + Object.entries(v).map(([k, x]) => `${k} = ${tv(x)}`).join(', ') + ' }'
  : typeof v === 'number' ? String(v) : JSON.stringify(String(v));
function partBlock(raw) {
  const out = ['[[part]]'];
  for (const [k, v] of Object.entries(raw)) {
    if (k === 'buy') out.push(v.length ? 'buy = [\n' + v.map(b => '  ' + tv(b) + ',').join('\n') + '\n]' : 'buy = []');
    else out.push(`${k} = ${tv(v)}`);
  }
  return out.join('\n') + '\n';
}
function replacePart(text, raw) {
  const blocks = text.split(/^(?=\[\[part\]\]\s*$)/m);
  const i = blocks.findIndex(b => new RegExp(`^id = "${raw.id}"\\s*$`, 'm').test(b));
  if (i < 0) throw new Error(`part "${raw.id}" not found in data/parts.toml`);
  const tail = blocks[i].match(/\n*$/)[0];
  blocks[i] = partBlock(raw).replace(/\n$/, '') + (tail.length > 1 ? tail : '\n\n');
  if (i === blocks.length - 1) blocks[i] = blocks[i].replace(/\n+$/, '\n');
  return blocks.join('');
}

// ---------- status bar and token
function say(html, kind = '') { bar.hidden = false; bar.className = 'editbar ' + kind; bar.innerHTML = html; }
function askToken() {
  say(`<b>Editing saves straight to the team's GitHub repository.</b> It needs your GitHub token once (with write access to the repo; ask a lead if you have none).
    <a href="${TOKEN_URL}" target="_blank" rel="noopener">Make a token</a> (tick <i>public_repo</i>, then copy it) and paste it here:
    <input id="tok" type="password" autocomplete="off" placeholder="ghp_… or github_pat_…" size="28"> <label><input id="tokKeep" type="checkbox" checked> remember in this browser</label>
    <button type="button" id="tokOk">Start editing</button> <button type="button" id="tokNo" class="plainbtn">Cancel</button>`);
  document.getElementById('tokOk').onclick = async () => {
    const v = document.getElementById('tok').value.trim(); if (!v) return;
    token = v;
    try {
      const repo = await gh('');
      if (!repo.permissions?.push) { say('This token works, but its GitHub account cannot save to the repository. Ask a lead to add you, or propose the change on GitHub.', 'bad'); token = null; return; }
    } catch (e) { say(`GitHub did not accept the token (${esc(e.message.slice(0, 60))}).`, 'bad'); token = null; return; }
    if (document.getElementById('tokKeep').checked) store.set('rb-token', token); else { try { sessionStorage.setItem('rb-token', token); } catch { } }
    start();
  };
  document.getElementById('tokNo').onclick = stop;
}
function start() {
  editing = true; btn.setAttribute('aria-pressed', 'true'); btn.textContent = 'Done editing';
  document.body.classList.add('editing');
  say(`<b>Editing.</b> ${document.getElementById('detail') ? 'Pick a part; its panel turns into a form.' : ''} ${document.getElementById('inputs') ? 'Change any value in the design numbers table, then press Save.' : ''}
    Saving writes data/parts.toml or data/inputs.toml on GitHub with your name; the sheets and pages rebuild in about two minutes. Every save is a commit in <a href="https://github.com/${REPO}/commits/main" target="_blank" rel="noopener">the history</a>, so it can be undone. <button type="button" id="forget" class="plainbtn">Forget my token</button>`);
  document.getElementById('forget').onclick = () => { store.set('rb-token', null); try { sessionStorage.removeItem('rb-token'); } catch { } token = null; stop(); };
  window.RB?.rerender?.();
  inputsMode(true);
}
function stop() {
  editing = false; btn.setAttribute('aria-pressed', 'false'); btn.textContent = 'Edit';
  document.body.classList.remove('editing'); bar.hidden = true;
  window.RB?.rerender?.();
  inputsMode(false);
}
btn?.addEventListener('click', () => editing ? stop() : (token ? start() : askToken()));

// ---------- after a save: follow the rebuild, then reload
async function follow(sha, what) {
  say(`Saved ${esc(what)} (<a href="https://github.com/${REPO}/commit/${sha}" target="_blank" rel="noopener">${sha.slice(0, 7)}</a>). Rebuilding the sheets and pages…`, 'busy');
  const t0 = Date.now();
  while (Date.now() - t0 < 6 * 60e3) {
    await new Promise(r => setTimeout(r, 8000));
    try {
      const runs = await gh(`/actions/runs?head_sha=${sha}&t=${Date.now()}`);
      const run = runs.workflow_runs.find(r => r.name.startsWith('Rebuild'));
      if (!run) continue;
      if (run.status !== 'completed') { say(`Saved ${esc(what)}. Rebuilding… (<a href="${run.html_url}" target="_blank" rel="noopener">progress</a>)`, 'busy'); continue; }
      if (run.conclusion !== 'success') { say(`The rebuild failed, so the site keeps the previous version. <a href="${run.html_url}" target="_blank" rel="noopener">See why</a>; fix the value and save again.`, 'bad'); return; }
      say(`Rebuilt. The page reloads when GitHub has published it (about a minute)…`, 'busy');
      for (let k = 0; k < 30; k++) {
        await new Promise(r => setTimeout(r, 6000));
        const b = await gh(`/pages/builds/latest?t=${Date.now()}`).catch(() => null);
        if (b && b.status === 'built' && new Date(b.updated_at) > new Date(run.updated_at)) { location.reload(); return; }
      }
      say('Rebuilt and saved. Reload the page in a minute to see it everywhere.', 'ok'); return;
    } catch (e) { /* keep waiting */ }
  }
  say('Saved. The rebuild is taking longer than usual; check the Actions tab on GitHub.', 'ok');
}

// ---------- the part form (drone page)
const FIELDS = [['name', 'Name', 'text'], ['status', 'Status', 'text'], ['qty', 'Quantity', 'num'], ['price', 'Price each (US$)', 'num'],
  ['mass', 'Mass each (g)', 'num'], ['size', 'Size', 'text'], ['power_w', 'Power (W)', 'num'],
  ['job', 'What it does', 'area'], ['why', 'Why we chose it', 'area'], ['instead', 'Considered instead', 'area'], ['source', 'Numbers from', 'text']];
function partForm(p, el) {
  if (!editing) return false;
  const raw = structuredClone(p.raw);
  const f = FIELDS.filter(([k]) => k in raw || ['size', 'instead', 'source'].includes(k)).map(([k, label, kind]) => {
    const v = raw[k] ?? '';
    const input = kind === 'area' ? `<textarea name="${k}" rows="${k === 'why' ? 6 : 3}">${esc(v)}</textarea>`
      : `<input name="${k}" ${kind === 'num' ? 'inputmode="decimal"' : ''} value="${esc(v)}">`;
    return `<label class="fld"><span>${label}</span>${input}</label>`;
  }).join('');
  const buyRow = (b = { store: '', link: '', price: '' }) => `<div class="buyrow"><input name="store" placeholder="Shop" value="${esc(b.store)}"><input name="link" placeholder="https://…" value="${esc(b.link)}"><input name="bprice" placeholder="$0.00" value="${esc(b.price)}"><button type="button" class="plainbtn rm" aria-label="Remove shop">remove</button></div>`;
  el.innerHTML = `<form class="pform"><h2>Edit: ${esc(p.name)}</h2>${f}
    <div class="fld"><span>Where to buy</span><div class="buys">${(raw.buy || []).map(buyRow).join('')}</div><button type="button" class="plainbtn addbuy">+ add a shop</button></div>
    <p class="small">Words in {curly braces} are numbers filled in from the sheets (for example {TW_hot}); keep the braces.</p>
    <p class="err" hidden></p><div class="actions"><button type="submit">Save to GitHub</button> <button type="button" class="plainbtn cancel">Cancel</button></div></form>`;
  const form = el.querySelector('form');
  form.querySelector('.addbuy').onclick = () => form.querySelector('.buys').insertAdjacentHTML('beforeend', buyRow());
  form.addEventListener('click', e => { if (e.target.classList.contains('rm')) e.target.closest('.buyrow').remove(); });
  form.querySelector('.cancel').onclick = () => stop();
  form.onsubmit = async e => {
    e.preventDefault();
    const err = form.querySelector('.err'); err.hidden = true;
    for (const [k, , kind] of FIELDS) {
      const inp = form.elements[k]; if (!inp) continue;
      const s = inp.value.trim();
      if (kind === 'num') {
        if (s === '' && !(k in p.raw)) continue;
        const n = Number(s.replace(/[$,]/g, ''));
        if (!Number.isFinite(n) || n < 0) { err.textContent = `“${FIELDS.find(x => x[0] === k)[1]}” must be a number.`; err.hidden = false; return; }
        raw[k] = n;
      } else if (s !== '' || k in p.raw) raw[k] = s;
    }
    raw.buy = [...form.querySelectorAll('.buyrow')].map(r => ({ store: r.querySelector('[name=store]').value.trim(), link: r.querySelector('[name=link]').value.trim(), price: r.querySelector('[name=bprice]').value.trim() }))
      .filter(b => b.store || b.link);
    const bad = raw.buy.find(b => !/^https?:\/\/\S+$/.test(b.link));
    if (bad) { err.textContent = `The link for “${bad.store || 'a shop'}” must start with https://`; err.hidden = false; return; }
    const sub = form.querySelector('[type=submit]'); sub.disabled = true; sub.textContent = 'Saving…';
    try {
      const sha = await saveFile('data/parts.toml', t => replacePart(t, raw), `Edit ${raw.id} on the website`);
      if (!sha) { say('Nothing changed.', 'ok'); sub.disabled = false; sub.textContent = 'Save to GitHub'; return; }
      p.raw = raw; follow(sha, raw.name);
    } catch (x) { err.textContent = 'Could not save: ' + x.message.slice(0, 160); err.hidden = false; sub.disabled = false; sub.textContent = 'Save to GitHub'; }
  };
  return true;
}
window.RB = Object.assign(window.RB || {}, { partForm, isEditing: () => editing });

// ---------- design numbers (calculations page)
function inputsMode(on) {
  const sec = document.getElementById('inputs'); if (!sec) return;
  sec.querySelectorAll('tr[data-key]').forEach(tr => {
    const td = tr.querySelector('.val');
    if (on && !td.querySelector('input')) { const v = td.textContent; td.dataset.orig = v; td.innerHTML = `<input value="${esc(v)}" size="8" aria-label="${esc(tr.cells[0].textContent)}">`; }
    if (!on && td.querySelector('input')) td.textContent = td.dataset.orig;
  });
  let save = document.getElementById('saveInputs');
  if (on && !save) {
    sec.insertAdjacentHTML('beforeend', '<p class="actions" id="saveInputs"><button type="button">Save changed numbers to GitHub</button> <span class="err" hidden></span></p>');
    save = document.getElementById('saveInputs');
    save.querySelector('button').onclick = saveInputs;
  }
  if (!on && save) save.remove();
}
async function saveInputs() {
  const err = document.querySelector('#saveInputs .err'); err.hidden = true;
  const changes = [];
  for (const tr of document.querySelectorAll('#inputs tr[data-key]')) {
    const td = tr.querySelector('.val'), inp = td.querySelector('input'); if (!inp) continue;
    const v = inp.value.trim(); if (v === td.dataset.orig) continue;
    const isNum = /^-?\d+(\.\d+)?([eE]-?\d+)?$/.test(v), wasNum = /^-?\d/.test(td.dataset.orig);
    if (wasNum && !isNum) { err.textContent = `“${tr.cells[0].textContent}” must be a number.`; err.hidden = false; return; }
    changes.push([tr.dataset.key, isNum ? v : JSON.stringify(v.replace(/^"|"$/g, ''))]);
  }
  if (!changes.length) { say('Nothing changed.', 'ok'); return; }
  try {
    const sha = await saveFile('data/inputs.toml', t => changes.reduce((s, [k, v]) => s.replace(new RegExp(`^(${k} = )(.+?)(\\s+#.*)?$`, 'm'), (m, a, b, c) => a + v + (c || '')), t),
      `Change ${changes.map(c => c[0]).join(', ')} on the website`);
    if (sha) follow(sha, changes.map(c => c[0]).join(', '));
  } catch (x) { err.textContent = 'Could not save: ' + x.message.slice(0, 160); err.hidden = false; }
}
