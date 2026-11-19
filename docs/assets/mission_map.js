// Mission page map sheets: drawn by tools/make_map.py from real terrain; this adds hover notes, layer toggles and the flight animation.
const reduce = matchMedia('(prefers-reduced-motion: reduce)').matches;
const tip = document.getElementById('maptip');

async function load(holder) {
  holder.innerHTML = await (await fetch(holder.dataset.src)).text();
  holder.removeAttribute('aria-busy');
  const svg = holder.querySelector('svg');
  svg.addEventListener('pointermove', e => {
    const t = e.target.closest('[data-tip]');
    if (!t) { tip.hidden = true; return; }
    tip.textContent = t.dataset.tip; tip.hidden = false;
    tip.style.left = (e.pageX + 14) + 'px'; tip.style.top = (e.pageY + 12) + 'px';
  });
  svg.addEventListener('pointerleave', () => { tip.hidden = true; });
  return svg;
}

function fly(path) {                                  // trace the survey flight: out to the plot, 18 lines, back
  if (!path || reduce) return;
  const len = path.getTotalLength();
  path.style.transition = 'none'; path.style.strokeDasharray = len; path.style.strokeDashoffset = len;
  path.getBoundingClientRect();
  path.style.transition = 'stroke-dashoffset 7s linear'; path.style.strokeDashoffset = 0;
}

const site = document.querySelector('#siteSheet .mapholder');
const region = document.querySelector('#regionSheet .mapholder');
load(site).then(svg => {
  const path = svg.querySelector('#flightpath');
  new IntersectionObserver((es, ob) => { if (es[0].isIntersecting) { fly(path); ob.disconnect(); } }, { threshold: 0.4 }).observe(site);
  document.getElementById('replay').addEventListener('click', () => fly(path));
});
load(region).then(svg => {
  document.querySelectorAll('[data-layer]').forEach(cb => cb.addEventListener('change', () => {
    svg.querySelectorAll('.' + cb.dataset.layer).forEach(el => { el.style.display = cb.checked ? '' : 'none'; });
  }));
});
