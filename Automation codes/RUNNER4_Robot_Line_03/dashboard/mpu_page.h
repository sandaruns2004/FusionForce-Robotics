#pragma once
// =============================================================================
// dashboard/mpu_page.h  — MPU6050 Gyroscope Dashboard Page for RUNNER4
// Served at GET /mpu  — Live yaw, Gz, accel, calibration, history chart, I2C scan
// =============================================================================

#include <WebServer.h>

static const char MPU_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RUNNER4 — MPU6050 Gyro</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#0f1117;color:#e2e8f0;min-height:100vh}
header{background:linear-gradient(135deg,#4f46e5,#7c3aed);padding:14px 20px;display:flex;align-items:center;gap:14px}
header h1{font-size:1.3rem;font-weight:700;color:#fff}
.badge{background:rgba(255,255,255,.18);border-radius:999px;padding:3px 10px;font-size:.75rem;color:#e9d5ff}
nav{background:#1e1b4b;padding:8px 20px;display:flex;gap:8px;flex-wrap:wrap}
nav a{color:#a5b4fc;text-decoration:none;padding:4px 12px;border-radius:6px;font-size:.82rem;transition:.2s}
nav a:hover,nav a.active{background:#3730a3;color:#fff}
.container{max-width:900px;margin:0 auto;padding:20px}
.status-bar{display:flex;align-items:center;gap:10px;margin-bottom:20px;padding:10px 16px;background:#1e293b;border-radius:10px;border:1px solid #334155}
.pill{padding:4px 14px;border-radius:999px;font-size:.8rem;font-weight:600}
.pill.online{background:#064e3b;color:#34d399}
.pill.offline{background:#450a0a;color:#f87171}
.pill.cal{background:#1e3a5f;color:#60a5fa}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:16px;margin-bottom:20px}
.card{background:#1e293b;border-radius:12px;padding:20px;border:1px solid #334155;text-align:center}
.card-label{font-size:.75rem;color:#94a3b8;text-transform:uppercase;letter-spacing:.06em;margin-bottom:6px}
.card-val{font-size:2.4rem;font-weight:800;line-height:1.1}
.card-unit{font-size:.85rem;color:#64748b;margin-top:4px}
.yaw-val{color:#a78bfa}
.gz-val{color:#34d399}
.accel-val{color:#60a5fa}
.chart-wrap{background:#1e293b;border-radius:12px;padding:16px;border:1px solid #334155;margin-bottom:20px}
.chart-wrap h3{font-size:.9rem;color:#94a3b8;margin-bottom:10px}
canvas{width:100%;height:140px;display:block}
.btn-row{display:flex;gap:10px;flex-wrap:wrap;margin-bottom:20px}
button{padding:9px 20px;border:none;border-radius:8px;cursor:pointer;font-size:.85rem;font-weight:600;transition:.2s}
.btn-zero{background:#4f46e5;color:#fff}
.btn-zero:hover{background:#4338ca}
.btn-cal{background:#065f46;color:#fff}
.btn-cal:hover{background:#047857}
.btn-cal.running{background:#78350f;color:#fff;cursor:wait}
.btn-rescan{background:#0284c7;color:#fff}
.btn-rescan:hover{background:#0369a1}
.info-grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:20px}
.info-card{background:#1e293b;border-radius:10px;padding:14px;border:1px solid #334155}
.info-card h4{font-size:.78rem;color:#64748b;margin-bottom:8px;text-transform:uppercase}
.info-row{display:flex;justify-content:space-between;padding:4px 0;border-bottom:1px solid #1e293b;font-size:.82rem}
.info-row:last-child{border:none}
.info-key{color:#94a3b8}
.info-val{color:#e2e8f0;font-weight:600;font-family:monospace}
.compass{width:120px;height:120px;margin:0 auto 10px;position:relative}
.compass svg{width:100%;height:100%}
#compass-needle{transform-origin:50% 50%;transition:transform .2s}
.i2c-tags{display:flex;flex-wrap:wrap;gap:6px;margin-top:6px}
.i2c-tag{font-size:11px;font-family:monospace;padding:3px 8px;border-radius:6px;background:#1e3a5f;color:#7dd3fc}
.i2c-tag.mpu{background:#065f46;color:#34d399;font-weight:700}
.i2c-tag.other{background:#334155;color:#94a3b8}
@media(max-width:600px){.info-grid{grid-template-columns:1fr}.card-val{font-size:1.8rem}}
</style>
</head>
<body>
<header>
  <div>
    <h1>&#127959; RUNNER4 — MPU6050 Gyroscope</h1>
    <div class="badge" id="badge-hdr">I2C (0x68 / 0x69) | Wire (SDA=GPIO 11, SCL=GPIO 12)</div>
  </div>
</header>
<nav>
  <a href="/">&#127968; Robot</a>
  <a href="/color">&#127752; Color</a>
  <a href="/line">&#9135; Line</a>
  <a href="/linefollow">&#11835; LineFollow</a>
  <a href="/tof">&#128269; ToF</a>
  <a href="/arm">&#129534; Arm</a>
  <a href="/mpu" class="active">&#127996; Gyro</a>
  <a href="/comp">&#127942; Comp</a>
</nav>
<div class="container">
  <div class="status-bar">
    <span id="pill-status" class="pill offline">OFFLINE</span>
    <span id="pill-cal"    class="pill cal" style="display:none">Calibrated</span>
    <span id="pill-addr"   class="pill cal" style="display:none">--</span>
    <span id="last-update" style="font-size:.78rem;color:#64748b;margin-left:auto">--</span>
  </div>

  <!-- Compass + big Yaw card -->
  <div style="display:flex;gap:16px;align-items:flex-start;margin-bottom:20px;flex-wrap:wrap">
    <div class="card" style="flex:0 0 160px">
      <div class="card-label">Compass</div>
      <div class="compass">
        <svg viewBox="0 0 120 120">
          <circle cx="60" cy="60" r="56" fill="#0f172a" stroke="#334155" stroke-width="2"/>
          <text x="60" y="18" text-anchor="middle" fill="#94a3b8" font-size="12">N</text>
          <text x="60" y="110" text-anchor="middle" fill="#94a3b8" font-size="12">S</text>
          <text x="12" y="65" text-anchor="middle" fill="#94a3b8" font-size="12">W</text>
          <text x="108" y="65" text-anchor="middle" fill="#94a3b8" font-size="12">E</text>
          <g id="compass-needle">
            <polygon points="60,20 56,60 60,65 64,60" fill="#f87171"/>
            <polygon points="60,100 56,60 60,65 64,60" fill="#94a3b8"/>
          </g>
        </svg>
      </div>
    </div>
    <div class="card" style="flex:1;min-width:140px">
      <div class="card-label">Yaw (Integrated)</div>
      <div class="card-val yaw-val" id="yaw-val">0.0</div>
      <div class="card-unit">degrees</div>
    </div>
    <div class="card" style="flex:1;min-width:140px">
      <div class="card-label">Gz (filtered)</div>
      <div class="card-val gz-val" id="gz-val">0.000</div>
      <div class="card-unit">deg/s</div>
    </div>
  </div>

  <!-- Accel cards -->
  <div class="grid">
    <div class="card">
      <div class="card-label">Ax</div>
      <div class="card-val accel-val" id="ax-val">--</div>
      <div class="card-unit">g</div>
    </div>
    <div class="card">
      <div class="card-label">Ay</div>
      <div class="card-val accel-val" id="ay-val">--</div>
      <div class="card-unit">g</div>
    </div>
    <div class="card">
      <div class="card-label">Az</div>
      <div class="card-val accel-val" id="az-val">--</div>
      <div class="card-unit">g</div>
    </div>
    <div class="card">
      <div class="card-label">Gx</div>
      <div class="card-val gz-val" id="gx-val">--</div>
      <div class="card-unit">deg/s</div>
    </div>
  </div>

  <!-- Yaw history chart -->
  <div class="chart-wrap">
    <h3>Yaw History (last 60 samples @ 100ms)</h3>
    <canvas id="yawChart"></canvas>
  </div>

  <!-- Gz history chart -->
  <div class="chart-wrap">
    <h3>Gz (Angular Rate) History</h3>
    <canvas id="gzChart"></canvas>
  </div>

  <!-- Buttons -->
  <div class="btn-row">
    <button class="btn-rescan" id="btn-rescan" onclick="doRescan()">&#128269; Detect / Rescan I2C</button>
    <button class="btn-zero" onclick="doZero()">&#8635; Zero Yaw</button>
    <button class="btn-cal" id="btn-cal" onclick="doCal()">&#9881; Calibrate (~10s)</button>
  </div>

  <!-- Live I2C scan panel -->
  <div class="card" style="margin-bottom:20px;text-align:left">
    <div class="card-label">Live I2C Bus Devices (SDA=GPIO 11, SCL=GPIO 12)</div>
    <div class="i2c-tags" id="i2c-tags">Click "Detect / Rescan I2C" to scan...</div>
  </div>

  <!-- Info -->
  <div class="info-grid">
    <div class="info-card">
      <h4>Sensor Info</h4>
      <div class="info-row"><span class="info-key">I2C Address</span><span class="info-val" id="info-addr">Auto (0x68 / 0x69)</span></div>
      <div class="info-row"><span class="info-key">WHO_AM_I</span><span class="info-val" id="info-who">--</span></div>
      <div class="info-row"><span class="info-key">Bus</span><span class="info-val">Wire (SDA=11, SCL=12)</span></div>
      <div class="info-row"><span class="info-key">Poll Rate</span><span class="info-val">100 Hz</span></div>
      <div class="info-row"><span class="info-key">Gyro Range</span><span class="info-val">±500 dps</span></div>
      <div class="info-row"><span class="info-key">Accel Range</span><span class="info-val">±2 g</span></div>
      <div class="info-row"><span class="info-key">Clock Source</span><span class="info-val">PLL X-Gyro</span></div>
    </div>
    <div class="info-card">
      <h4>Filter Config</h4>
      <div class="info-row"><span class="info-key">Gyro EMA α</span><span class="info-val">0.90</span></div>
      <div class="info-row"><span class="info-key">Accel EMA α</span><span class="info-val">0.90</span></div>
      <div class="info-row"><span class="info-key">Drift Threshold</span><span class="info-val">3.5 dps</span></div>
      <div class="info-row"><span class="info-key">Turn Tolerance</span><span class="info-val">±3 deg</span></div>
      <div class="info-row"><span class="info-key">Auto-reconnect</span><span class="info-val">Active (every 2.5s)</span></div>
      <div class="info-row"><span class="info-key">Yaw Integrator</span><span class="info-val">Gz × dt</span></div>
    </div>
  </div>
</div>

<script>
const YAW_LEN = 60, GZ_LEN = 60;
let yawHist = new Array(YAW_LEN).fill(null);
let gzHist  = new Array(GZ_LEN).fill(null);
let calRunning = false;

function fmt(v, d=2) { return (typeof v==='number') ? v.toFixed(d) : '--'; }

function drawChart(canvasId, data, color, label, minY, maxY) {
  const c = document.getElementById(canvasId);
  if (!c) return;
  const W = c.offsetWidth || 600, H = 140;
  c.width = W; c.height = H;
  const ctx = c.getContext('2d');
  ctx.clearRect(0,0,W,H);
  ctx.strokeStyle = '#1e293b'; ctx.lineWidth = 1;
  for (let i=0;i<4;i++){
    const y = H/4*i;
    ctx.beginPath(); ctx.moveTo(0,y); ctx.lineTo(W,y); ctx.stroke();
  }
  if (minY < 0 && maxY > 0) {
    const zy = H - ((0-minY)/(maxY-minY))*H;
    ctx.strokeStyle='#334155'; ctx.lineWidth=1.5;
    ctx.beginPath(); ctx.moveTo(0,zy); ctx.lineTo(W,zy); ctx.stroke();
  }
  const pts = data.filter(v=>v!==null);
  if (!pts.length) return;
  const mn = Math.min(minY, ...pts), mx = Math.max(maxY, ...pts);
  const range = mx-mn || 1;
  ctx.strokeStyle = color; ctx.lineWidth = 2;
  ctx.beginPath();
  let first = true;
  data.forEach((v,i) => {
    if (v===null) { first=true; return; }
    const x = (i/data.length)*W;
    const y = H - ((v-mn)/range)*H;
    if (first) { ctx.moveTo(x,y); first=false; } else ctx.lineTo(x,y);
  });
  ctx.stroke();
  ctx.fillStyle='#64748b'; ctx.font='11px monospace';
  ctx.fillText(label+': '+fmt(pts[pts.length-1]), 8, 16);
}

function update(d) {
  document.getElementById('yaw-val').textContent = fmt(d.yaw,1);
  document.getElementById('gz-val').textContent  = fmt(d.gz,3);
  document.getElementById('ax-val').textContent  = fmt(d.ax,3);
  document.getElementById('ay-val').textContent  = fmt(d.ay,3);
  document.getElementById('az-val').textContent  = fmt(d.az,3);
  document.getElementById('gx-val').textContent  = fmt(d.gx,3);

  const needle = document.getElementById('compass-needle');
  if (needle) needle.setAttribute('transform', `rotate(${d.yaw},60,60)`);

  const sp = document.getElementById('pill-status');
  const cp = document.getElementById('pill-cal');
  const ap = document.getElementById('pill-addr');

  sp.textContent = d.ready ? 'ONLINE' : 'OFFLINE';
  sp.className = 'pill ' + (d.ready ? 'online' : 'offline');

  if (d.addr) {
    ap.textContent = d.addr + (d.who ? ' (' + d.who + ')' : '');
    ap.style.display = '';
    document.getElementById('info-addr').textContent = d.addr;
    if (d.who) document.getElementById('info-who').textContent = d.who;
  }
  if (d.cal) { cp.style.display=''; } else { cp.style.display='none'; }

  document.getElementById('last-update').textContent = new Date().toLocaleTimeString();

  yawHist.push(d.yaw); if (yawHist.length>YAW_LEN) yawHist.shift();
  gzHist.push(d.gz);   if (gzHist.length>GZ_LEN)   gzHist.shift();

  drawChart('yawChart', yawHist, '#a78bfa', 'Yaw', -180, 180);
  drawChart('gzChart',  gzHist,  '#34d399', 'Gz',  -50,  50);
}

function poll() {
  fetch('/mpu/data').then(r=>r.json()).then(d=>update(d)).catch(()=>{});
}
function doZero() {
  fetch('/mpu/zero').then(r=>r.json()).then(d=>{ if(d.ok) alert('Yaw zeroed!'); });
}
function doCal() {
  if (calRunning) return;
  calRunning = true;
  const btn = document.getElementById('btn-cal');
  btn.textContent = 'Calibrating... (keep still)';
  btn.className = 'btn-cal running';
  fetch('/mpu/cal').then(r=>r.json()).then(d=>{
    calRunning = false;
    btn.textContent = 'Calibrate (~10s)';
    btn.className = 'btn-cal';
    alert(d.ok ? 'Calibration complete!' : 'Calibration failed: ' + d.msg);
  }).catch(()=>{ calRunning=false; btn.textContent='Calibrate (~10s)'; btn.className='btn-cal'; });
}
function doRescan() {
  const btn = document.getElementById('btn-rescan');
  btn.textContent = 'Scanning...';
  fetch('/mpu/rescan').then(r=>r.json()).then(d=>{
    btn.textContent = 'Detect / Rescan I2C';
    let html = '';
    if (d.i2c_scan && d.i2c_scan.length) {
      d.i2c_scan.forEach(a => {
        const isMpu = (a === '0x68' || a === '0x69');
        const desc = (a === '0x40') ? ' (PCA9685)' :
                     (a === '0x30') ? ' (ToF Front)' :
                     (a === '0x31') ? ' (ToF Right)' :
                     (a === '0x32') ? ' (ToF Left)' :
                     isMpu ? ' (MPU IMU)' : '';
        html += '<span class="i2c-tag ' + (isMpu ? 'mpu' : 'other') + '">' + a + desc + '</span>';
      });
    } else {
      html = '<span style="color:#ef4444;font-size:12px">No I2C devices found! Check 3.3V and GND wires.</span>';
    }
    document.getElementById('i2c-tags').innerHTML = html;
    if (d.ok) {
      alert('MPU detected at ' + d.addr + ' (WHO_AM_I=' + d.who + ')!');
    } else {
      alert('MPU not responding. Check wiring to SDA=11, SCL=12, VCC, GND.');
    }
  }).catch(()=>{ btn.textContent = 'Detect / Rescan I2C'; });
}

setInterval(poll, 100);
poll();
setTimeout(doRescan, 500); // Auto-scan I2C bus tags on page load
</script>
</body>
</html>
)rawliteral";

static WebServer *_mpuPageSrv = nullptr;

static void _mpuServePage() {
    _mpuPageSrv->sendHeader("Cache-Control","no-store");
    _mpuPageSrv->send_P(200, "text/html", MPU_PAGE);
}

inline void mpuPageBegin(WebServer &server) {
    _mpuPageSrv = &server;
    server.on("/mpu", HTTP_GET, _mpuServePage);
}