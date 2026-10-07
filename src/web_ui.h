#pragma once

// Single-page controller UI. Talks to the firmware over the /ws WebSocket.
static const char INDEX_HTML[] = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Robotic Arm</title>
<style>
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body {
    background: #0f0f12; color: #e8e6e0;
    font-family: 'SF Mono', 'Fira Code', monospace;
    min-height: 100vh; padding: 16px; max-width: 560px; margin: 0 auto;
  }
  header {
    display: flex; align-items: center; gap: 12px;
    margin-bottom: 16px; padding-bottom: 16px; border-bottom: 1px solid #2a2a30;
  }
  .logo {
    width: 36px; height: 36px; background: #00e5a0; border-radius: 8px;
    display: flex; align-items: center; justify-content: center;
    font-size: 18px; color: #0f0f12;
  }
  h1 { font-size: 18px; font-weight: 600; color: #fff; }
  .subtitle { font-size: 11px; color: #666670; letter-spacing: 0.08em; text-transform: uppercase; margin-top: 2px; }
  .conn { margin-left: auto; display: flex; align-items: center; gap: 8px; font-size: 11px; color: #888890; }
  .dot { width: 8px; height: 8px; border-radius: 50%; background: #ff5555; }
  .dot.ok   { background: #00e5a0; box-shadow: 0 0 8px #00e5a066; }
  .dot.warn { background: #f0a030; box-shadow: 0 0 8px #f0a03066; }

  .banner {
    display: none; border-radius: 10px; padding: 12px 14px; margin-bottom: 12px;
    font-size: 12px; font-weight: 600; border: 1px solid;
  }
  .banner.show { display: block; }
  .banner.err  { background: #ff555518; border-color: #ff5555; color: #ff7777; }
  .banner.info { background: #f0a03018; border-color: #f0a030; color: #f0a030; }

  .estop {
    width: 100%; padding: 18px; margin-bottom: 16px; border: none; border-radius: 12px;
    background: #e03030; color: #fff; font-family: inherit; font-size: 16px;
    font-weight: 800; letter-spacing: 0.12em; cursor: pointer;
  }
  .estop:active { opacity: 0.8; }
  .estop.resume { background: #222228; color: #f0a030; border: 2px solid #f0a030; }

  .panel {
    background: #18181f; border: 1px solid #2a2a30;
    border-radius: 12px; padding: 14px; margin-bottom: 12px;
  }
  .prow { display: flex; justify-content: space-between; align-items: center; margin-bottom: 10px; }
  .plabel { font-size: 11px; text-transform: uppercase; letter-spacing: 0.1em; color: #888890; }
  .pval { font-size: 15px; font-weight: 700; color: #f0a030; }
  .grid { display: grid; gap: 6px; }
  .btn {
    background: #222228; border: 1px solid #2a2a30; border-radius: 8px;
    color: #aaa; font-family: inherit; font-size: 11px; padding: 9px 4px;
    cursor: pointer; text-align: center; transition: all 0.15s;
  }
  .btn:active, .btn.on { background: #00e5a022; border-color: #00e5a0; color: #00e5a0; }
  .spd.on { background: #f0a03022; border-color: #f0a030; color: #f0a030; }
  .btn:disabled { opacity: 0.35; cursor: not-allowed; }

  .card {
    background: #18181f; border: 1px solid #2a2a30; border-radius: 12px;
    padding: 16px; margin-bottom: 12px; transition: border-color 0.2s, opacity 0.2s;
  }
  .card.moving { border-color: #00e5a055; }
  .card.off { opacity: 0.45; }
  .row { display: flex; justify-content: space-between; align-items: baseline; margin-bottom: 12px; }
  .jname { font-size: 12px; text-transform: uppercase; letter-spacing: 0.1em; color: #888890; }
  .jval { font-size: 22px; font-weight: 700; color: #00e5a0; }
  .jval small { font-size: 12px; color: #555560; margin-left: 2px; }
  .jtgt { font-size: 11px; color: #666670; margin-left: 8px; }
  input[type=range] {
    -webkit-appearance: none; appearance: none; width: 100%; height: 6px;
    background: #2a2a30; border-radius: 3px; outline: none; margin-bottom: 10px;
  }
  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none; width: 24px; height: 24px; border-radius: 50%;
    background: #00e5a0; cursor: pointer; box-shadow: 0 0 12px #00e5a066;
  }
  input[type=range]::-moz-range-thumb {
    width: 24px; height: 24px; border: none; border-radius: 50%; background: #00e5a0; cursor: pointer;
  }
  input[type=range]:disabled { opacity: 0.4; }

  .toast {
    position: fixed; bottom: 20px; left: 50%;
    transform: translateX(-50%) translateY(80px);
    background: #00e5a0; color: #0f0f12; padding: 10px 22px; border-radius: 20px;
    font-size: 12px; font-weight: 600; z-index: 99; white-space: nowrap;
    transition: transform 0.3s cubic-bezier(.34,1.56,.64,1);
  }
  .toast.err { background: #ff5555; color: #fff; }
  .toast.show { transform: translateX(-50%) translateY(0); }
</style>
</head>
<body>

<header>
  <div class="logo">&#9881;</div>
  <div>
    <h1>Robotic Arm</h1>
    <div class="subtitle">6 DOF &middot; ESP32</div>
  </div>
  <div class="conn"><span id="connText">Connecting</span><div class="dot" id="dot"></div></div>
</header>

<div class="banner err" id="faultBanner"></div>
<div class="banner info" id="infoBanner"></div>

<button class="estop" id="estop">EMERGENCY STOP</button>

<div class="panel">
  <div class="prow">
    <div class="plabel">Movement speed</div>
    <div class="pval" id="spLabel">-</div>
  </div>
  <div class="grid" style="grid-template-columns: repeat(4,1fr)" id="speeds"></div>
</div>

<div class="panel">
  <div class="prow"><div class="plabel">Poses</div></div>
  <div class="grid" style="grid-template-columns: repeat(2,1fr)" id="poses"></div>
</div>

<div id="joints"></div>

<div class="toast" id="toast"></div>

<script>
const SPEEDS = [['Slow', 15], ['Normal', 50], ['Fast', 125], ['Max', 300]];
const SEND_INTERVAL_MS = 50;
const PING_MS = 500;
const STALE_MS = 2500;

const $ = id => document.getElementById(id);
let ws = null, cfg = null, state = null, lastMsgAt = 0;
const dragging = {};      // joint index -> true while the user holds the slider
const pending = {};       // joint index -> latest value waiting to be sent
const lastSent = {};      // joint index -> timestamp

function el(tag, attrs = {}, text) {
  const e = document.createElement(tag);
  Object.entries(attrs).forEach(([k, v]) => e.setAttribute(k, v));
  if (text !== undefined) e.textContent = text;
  return e;
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) { ws.send(JSON.stringify(obj)); return true; }
  return false;
}

// ---- Build UI from the firmware's config ----
function build() {
  const sp = $('speeds'); sp.replaceChildren();
  SPEEDS.forEach(([name, v]) => {
    const b = el('button', { class: 'btn spd', 'data-v': v }, name);
    b.onclick = () => send({ t: 'speed', v });
    sp.appendChild(b);
  });

  const po = $('poses'); po.replaceChildren();
  cfg.poses.forEach(n => {
    const b = el('button', { class: 'btn motion' }, n.charAt(0).toUpperCase() + n.slice(1));
    b.onclick = () => { if (send({ t: 'pose', n })) toast('Moving to ' + n); };
    po.appendChild(b);
  });

  const box = $('joints'); box.replaceChildren();
  cfg.joints.forEach((j, i) => {
    const card = el('div', { class: 'card', id: 'c' + i });
    const row = el('div', { class: 'row' });
    row.appendChild(el('div', { class: 'jname' }, j.name));
    const val = el('div');
    const v = el('span', { class: 'jval', id: 'v' + i }, '-');
    val.append(v, el('span', { class: 'jtgt', id: 't' + i }));
    row.appendChild(val);

    const s = el('input', { type: 'range', min: j.min, max: j.max, step: 1, id: 's' + i, class: 'motion' });
    s.addEventListener('pointerdown', () => dragging[i] = true);
    ['pointerup', 'pointercancel', 'change', 'blur'].forEach(ev =>
      s.addEventListener(ev, () => { dragging[i] = false; flush(i); }));
    s.addEventListener('input', () => queue(i, +s.value));

    const q = el('div', { class: 'grid', style: 'grid-template-columns: repeat(3,1fr)' });
    [j.min, (j.min + j.max) / 2, j.max].forEach(a => {
      a = Math.round(a);
      const b = el('button', { class: 'btn motion' }, a + '°');
      b.onclick = () => { s.value = a; queue(i, a); flush(i); };
      q.appendChild(b);
    });

    card.append(row, s, q);
    box.appendChild(card);
  });
}

// Throttle slider traffic: at most one message per joint every SEND_INTERVAL_MS,
// and the final position is always sent.
function queue(i, a) {
  pending[i] = a;
  $('t' + i).textContent = '→ ' + a + '°';
  const now = Date.now();
  if (!lastSent[i] || now - lastSent[i] >= SEND_INTERVAL_MS) flush(i);
  else setTimeout(() => flush(i), SEND_INTERVAL_MS - (now - lastSent[i]));
}

function flush(i) {
  if (pending[i] === undefined) return;
  if (send({ t: 'set', j: i, a: pending[i] })) lastSent[i] = Date.now();
  delete pending[i];
}

// ---- Render state pushed by the firmware ----
function render() {
  if (!cfg || !state) return;
  const blocked = state.estop || !!state.fault;

  cfg.joints.forEach((_, i) => {
    $('v' + i).replaceChildren(document.createTextNode(Math.round(state.cur[i])), el('small', {}, 'deg'));
    const moving = Math.abs(state.cur[i] - state.tgt[i]) > 0.5;
    $('t' + i).textContent = moving ? '→ ' + Math.round(state.tgt[i]) + '°' : '';
    $('c' + i).classList.toggle('moving', moving);
    $('c' + i).classList.toggle('off', !state.on[i]);
    const s = $('s' + i);
    if (!dragging[i] && pending[i] === undefined) s.value = state.tgt[i];
  });

  document.querySelectorAll('.motion').forEach(e => e.disabled = blocked);
  document.querySelectorAll('.spd').forEach(b => b.classList.toggle('on', +b.dataset.v === state.spd));
  const preset = SPEEDS.find(([, v]) => v === state.spd);
  $('spLabel').textContent = preset ? preset[0] : state.spd + ' deg/s';

  const es = $('estop');
  es.textContent = state.estop ? 'RESUME' : 'EMERGENCY STOP';
  es.classList.toggle('resume', state.estop);

  const fb = $('faultBanner');
  fb.textContent = state.fault ? 'FAULT: ' + state.fault : '';
  fb.classList.toggle('show', !!state.fault);

  const starting = state.on.some(o => !o) && !state.fault;
  const ib = $('infoBanner');
  ib.textContent = state.estop ? 'E-STOP active — press RESUME to re-enable motion'
                 : starting ? 'Powering up servos one at a time…' : '';
  ib.classList.toggle('show', state.estop || starting);

  setConn(state.fault || state.estop ? 'warn' : 'ok');
}

function setConn(level) {
  const labels = { ok: 'Online', warn: 'Online', off: 'Offline' };
  $('dot').className = 'dot' + (level === 'off' ? '' : ' ' + level);
  $('connText').textContent = labels[level];
}

// ---- Connection ----
function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onmessage = ev => {
    lastMsgAt = Date.now();
    const m = JSON.parse(ev.data);
    if (m.t === 'cfg') { cfg = m; build(); }
    else if (m.t === 'state') { state = m; render(); }
    else if (m.t === 'err') toast(m.msg, true);
  };
  ws.onclose = () => { setConn('off'); setTimeout(connect, 1000); };
  ws.onerror = () => ws.close();
}

$('estop').onclick = () => {
  if (!send({ t: state && state.estop ? 'resume' : 'stop' })) toast('Not connected', true);
};

setInterval(() => {
  send({ t: 'ping' });
  if (ws && ws.readyState === WebSocket.OPEN && Date.now() - lastMsgAt > STALE_MS) ws.close();
}, PING_MS);

let toastTimer;
function toast(msg, isErr) {
  const t = $('toast');
  t.textContent = msg;
  t.classList.toggle('err', !!isErr);
  t.classList.add('show');
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => t.classList.remove('show'), 2200);
}

connect();
</script>
</body>
</html>
)rawhtml";
