/* ── Pickup Winder — UI Controller ──────────────────────── */
let ws, lastStatus = null, activeTab = 'operate';
let rodageWasActive = false;
let manualActive = false, captureActive = false, captureData = [];
const byId = id => document.getElementById(id);

/* ── WebSocket ─────────────────────────────────────────── */
function cmd(c, v = '') {
  if (ws && ws.readyState === 1) ws.send(JSON.stringify({ cmd: c, val: '' + v }));
}

function connect() {
  ws = new WebSocket('ws://' + location.host + '/ws');
  ws.onopen  = () => setWs(true);
  ws.onclose = () => { setWs(false); setTimeout(connect, 2000); };
  ws.onmessage = e => {
    const d = JSON.parse(e.data);
    if (d.type === 'cap') { onCapture(d); return; }
    update(d);
  };
}

function setWs(ok) {
  byId('ws-dot').className = 'dot' + (ok ? ' ok' : '');
  byId('ws-lbl').textContent = ok ? 'Connected' : 'Offline';
}

/* ── Sync helpers ──────────────────────────────────────── */
function syncInput(id, val) {
  const el = byId(id);
  if (el && document.activeElement !== el && val !== undefined) el.value = val;
}
function syncRange(id, val, lbl, fix) {
  const el = byId(id);
  if (el && document.activeElement !== el && val !== undefined) el.value = val;
  if (lbl && val !== undefined) byId(lbl).textContent = parseFloat(val).toFixed(fix);
}

/* ── Tab switching ─────────────────────────────────────── */
function showTab(t) {
  activeTab = t;
  ['operate', 'setup', 'tools'].forEach(id => {
    byId('tab-' + id).classList.toggle('active', id === t);
    byId('pane-' + id).classList.toggle('show', id === t);
  });
}

/* ── Target adjustment from run view ───────────────────── */
function adjustTarget(delta) {
  const cur = parseInt(byId('ti-run').value || '0', 10);
  cmd('target', String(Math.max(1, cur + delta)));
}

/* ── Manual mode ───────────────────────────────────────── */
function toggleManual() {
  if (!manualActive) {
    cmd('manual');
    manualActive = true;
    byId('manual-controls').style.display = '';
    byId('btn-manual-toggle').textContent = '■ Exit manual';
    byId('btn-manual-toggle').className = 'btn btn-danger btn-s';
  } else {
    if (captureActive) toggleCapture();
    cmd('manual_stop');
    manualActive = false;
    byId('manual-controls').style.display = 'none';
    byId('btn-manual-toggle').textContent = '▶ Enter manual';
    byId('btn-manual-toggle').className = 'btn btn-go btn-s';
  }
}
function setManualStep(s) {
  cmd('manual_step', '' + s);
}

/* ── Capture ───────────────────────────────────────────── */
function toggleCapture() {
  if (!captureActive) {
    captureData = [];
    captureActive = true;
    cmd('manual_capture_start');
    byId('btn-capture').textContent = '⏹ Stop capture';
    byId('btn-capture').style.background = '#dc2626';
    byId('btn-dl-capture').style.display = 'none';
    byId('manual-cap-status').textContent = '⏺ 0 pts';
  } else {
    captureActive = false;
    cmd('manual_capture_stop');
    byId('btn-capture').textContent = '⏺ Capture';
    byId('btn-capture').style.background = '#0369a1';
    byId('manual-cap-status').textContent = '✓ ' + captureData.length + ' pts';
    if (captureData.length > 0) byId('btn-dl-capture').style.display = '';
  }
}
function onCapture(d) {
  if (captureActive) {
    captureData.push({ t: d.t, pos: d.pos, turns: d.turns });
    byId('manual-cap-status').textContent = '⏺ ' + captureData.length + ' pts';
  }
  byId('manual-pos').textContent = parseFloat(d.pos).toFixed(2) + ' mm';
  byId('manual-turns').textContent = d.turns;
}
function downloadCapture() {
  const blob = new Blob([JSON.stringify({ capture: captureData }, null, 2)], { type: 'application/json' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'capture_' + Date.now() + '.json';
  a.click();
}

/* ── Recipe import/export ──────────────────────────────── */
function importRecipe() {
  byId('recipe-file').click();
}
function handleRecipeFile(input) {
  const file = input.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => {
    cmd('recipe_import', reader.result);
    input.value = '';
  };
  reader.readAsText(file);
}

/* ── Break-in ──────────────────────────────────────────── */
function startRodage() {
  cmd('rodage_dist', byId('rod-dist').value);
  cmd('rodage_passes', byId('rod-passes').value);
  cmd('rodage');
  byId('rod-msg').textContent = '';
}

/* ── Main update ───────────────────────────────────────── */
function update(d) {
  lastStatus = d;
  const st = d.state || 'IDLE';
  const isManual = !!d.manualMode;
  const isRodage = !!d.rodageMode;

  /* ── State badge ── */
  const badge = byId('state-badge');
  badge.textContent = isManual ? 'MANUAL' : st;
  badge.className = 'badge' +
    (st === 'WINDING' ? ' run' : '') +
    (st === 'PAUSED' ? ' warn' : '') +
    (st === 'TARGET_REACHED' ? ' err' : '');

  /* ── Gauges ── */
  byId('rpm').textContent = Math.round(d.rpm || 0);
  byId('turns').textContent = d.turns || 0;
  byId('pass').textContent = d.pass || 0;
  byId('tgt').textContent = d.target || 0;
  byId('active-tpp').textContent = d.activeTpp ?? d.tpp ?? 0;
  byId('lat-scale').textContent = (d.latScale ?? 1).toFixed(2) + '×';
  byId('lat-progress').textContent = Math.round((d.latProgress ?? 0) * 100) + ' %';

  const pos = parseFloat(d.latPos ?? 0);
  byId('lat-pos-run').textContent = pos.toFixed(2) + ' mm';
  byId('lat-pos').textContent = pos.toFixed(2);
  byId('manual-pos').textContent = pos.toFixed(2) + ' mm';
  byId('manual-turns').textContent = d.turns ?? 0;

  byId('w-start-run').textContent = parseFloat(d.wStart ?? 0).toFixed(2);
  byId('w-end-run').textContent   = parseFloat(d.wEnd ?? 0).toFixed(2);
  byId('w-start').textContent     = parseFloat(d.wStart ?? 0).toFixed(2);
  byId('w-end').textContent       = parseFloat(d.wEnd ?? 0).toFixed(2);
  byId('tpp').textContent         = d.tpp ?? 0;
  byId('w-start-trim-run').textContent = (d.wStartTrim ?? 0).toFixed(2);
  byId('w-end-trim-run').textContent   = (d.wEndTrim ?? 0).toFixed(2);

  /* ── Progress bar ── */
  const pct = d.target > 0 ? Math.min(100, (d.turns / d.target) * 100) : 0;
  byId('bar').style.width = pct.toFixed(1) + '%';
  byId('pct').textContent = pct.toFixed(1) + ' %';

  /* ── Sync setup inputs ── */
  byId('r-cw').checked  = !!d.cw;
  byId('r-ccw').checked = !d.cw;
  byId('m-target').checked = !d.freerun;
  byId('m-free').checked   = !!d.freerun;
  syncInput('max-rpm', d.maxRpm);
  syncInput('ti', d.target);
  syncInput('ti-run', d.target);
  syncInput('w-seed', d.seed);
  syncInput('lat-ofs-input', d.latOfs);
  syncInput('g-total', d.gt); syncInput('g-margin', d.gm);
  syncInput('g-bottom', d.gb); syncInput('g-top', d.gtp);
  syncInput('g-start-trim', d.wStartTrim);
  syncInput('g-end-trim', d.wEndTrim);
  syncInput('g-wire', d.gw);

  if (d.wStyle) {
    byId('ws-straight').checked = d.wStyle === 'straight';
    byId('ws-scatter').checked  = d.wStyle === 'scatter';
    byId('ws-human').checked    = d.wStyle === 'human';
    updateSliderVisibility(d.wStyle);
  }
  const epStr = ['none','high','low'][d.endPos ?? 0] || 'none';
  byId('ep-none').checked = epStr === 'none';
  byId('ep-high').checked = epStr === 'high';
  byId('ep-low').checked  = epStr === 'low';
  byId('ep-turns').disabled = (epStr === 'none');
  syncInput('ep-turns', d.endPosTurns);

  syncRange('g-scatter', d.scatter, 'scatter-lbl', 1);
  if (byId('g-tpp-ofs') && document.activeElement !== byId('g-tpp-ofs') && d.tppOfs !== undefined)
    byId('g-tpp-ofs').value = d.tppOfs;
  byId('tpp-ofs-lbl').textContent = d.tppOfs ?? 0;
  syncRange('w-layer-jitter', d.layerJitter, 'lj', 2);
  syncRange('w-layer-speed', d.layerSpeed, 'ls', 2);
  syncRange('w-human-traverse', d.humanTraverse, 'ht', 2);
  syncRange('w-human-speed', d.humanSpeed, 'hs', 2);
  syncRange('w-first-pass-traverse', d.firstPassTraverse, 'fp', 2);

  /* ── Break-in UI ── */
  const rod = isRodage;
  byId('btn-rod-start').style.display = rod ? 'none' : '';
  byId('btn-rod-stop').style.display  = rod ? '' : 'none';
  byId('rod-dist').disabled   = rod;
  byId('rod-passes').disabled = rod;
  byId('rod-progress').style.display = rod ? '' : 'none';
  if (rod) {
    const p = d.rodagePass || 0, t = d.rodagePasses || 1;
    byId('rod-pass-lbl').textContent = p + ' / ' + t;
    byId('rod-bar').style.width = Math.round(p / t * 100) + '%';
    byId('rod-msg').textContent = '';
  } else if (rodageWasActive) {
    byId('rod-msg').textContent = '✓ Break-in done';
  }
  rodageWasActive = rod;

  /* ── Context-sensitive panels ── */
  updateContextPanels(st, d);

  /* ── Lock state for Setup pane ── */
  updateLockState(st, d);

  /* ── Status line ── */
  updateStatusLine(st, d, pos);
}

/* ── Context panels: show only what's relevant ─────────── */
function updateContextPanels(st, d) {
  const idle    = st === 'IDLE' && !d.manualMode;
  const isRunning = !!d.running;
  const paused = (st === 'PAUSED') && !isRunning;
  const winding = st === 'WINDING' || st === 'PAUSED' || isRunning;
  const target  = st === 'TARGET_REACHED';

  byId('ctx-idle').style.display    = idle    ? '' : 'none';
  byId('ctx-winding').style.display = winding ? '' : 'none';
  byId('ctx-target').style.display  = target  ? '' : 'none';
  // While running: only Pause. While paused: Resume + Stop. Never show Start and Pause together.
  byId('btn-start-run').style.display  = paused ? '' : 'none';
  byId('btn-start-run').textContent    = '▶ Resume';
  byId('btn-pause-run').style.display  = isRunning ? '' : 'none';
  byId('btn-stop-paused').style.display = paused ? '' : 'none';
}

/* ── Style-dependent slider visibility ─────────────────── */
function updateSliderVisibility(style) {
  const scatterOn = style === 'scatter' || style === 'human';
  const humanOn   = style === 'human';
  document.querySelectorAll('.sl-scatter').forEach(el => el.classList.toggle('sl-dim', !scatterOn));
  document.querySelectorAll('.sl-human').forEach(el   => el.classList.toggle('sl-dim', !humanOn));
}

/* ── Lock state: gray out locked fields during session ─── */
function updateLockState(st, d) {
  const sessionActive = st !== 'IDLE' && st !== 'TARGET_REACHED';
  byId('lock-banner').style.display = sessionActive ? '' : 'none';
  const lockTag = byId('lock-profile');
  if (lockTag) lockTag.style.display = sessionActive ? '' : 'none';

  // These fields are locked during a session (direction, mode, seed, pattern)
  document.querySelectorAll('.lockable').forEach(el => {
    el.classList.toggle('locked', sessionActive);
  });
}

/* ── Status line ───────────────────────────────────────── */
function updateStatusLine(st, d, pos) {
  const s = byId('status-line');
  if (d.running) {
    s.textContent = '▶ Winding…';
    return;
  }
  switch (st) {
    case 'WINDING':
      if (d.verifyLow) s.textContent = '🔍 Going to LOW bound…';
      else if (d.verifyHigh) s.textContent = '🔍 Going to HIGH bound…';
      else s.textContent = d.running ? '▶ Winding…' : '⏸ Pot at zero — raise to resume';
      break;
    case 'PAUSED':
      if (d.verifyLow) s.textContent = '🔍 Positioning to low bound…';
      else if (d.verifyHigh) s.textContent = '⏸ High bound reached — Resume to start winding';
      else s.textContent = '⏸ Paused — raise pot to resume';
      break;
    case 'TARGET_REACHED':
      s.textContent = '✓ Target reached — raise target or reset';
      break;
    case 'IDLE':
      if (d.manualMode)       s.textContent = '⚙ Manual mode — encoder = carriage, pot = motor';
      else if (d.rodageMode)  s.textContent = '🔧 Break-in in progress…';
      else if (Math.abs(pos) > 0.03) s.textContent = 'Carriage not at home — press Start to begin';
      else                    s.textContent = 'Ready — press Start';
      break;
    default:
      s.textContent = st;
  }
}

/* ── Init ──────────────────────────────────────────────── */
updateSliderVisibility('straight'); // default until first WS update
connect();
