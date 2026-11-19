#pragma once
#include <Arduino.h>

// ToF sensor dashboard page  -  served at  http://192.168.4.1/tof
// Single VL53L0X sensor - shows live distance reading only.
const char TOF_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ToF Distance - RUNNER4</title>
<style>
  @import url('https://fonts.googleapis.com/css2?family=Inter:wght@300;400;600;700;800&family=Outfit:wght@300;400;700;800&display=swap');
  :root{
    --bg:#080d18;--card:#0e1624;--line:#1e2d47;
    --mut:#6b82a0;--ac:#06b6d4;--ac2:#8b5cf6;
    --red:#ef4444;--grn:#22c55e;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:#e2eaf8;font-family:'Inter',system-ui,sans-serif;min-height:100vh}
  .wrap{max-width:640px;margin:0 auto;padding:24px 16px}
  /* header */
  .hdr{display:flex;align-items:center;gap:14px;margin-bottom:6px}
  .hdr-icon{width:46px;height:46px;border-radius:14px;
    background:linear-gradient(135deg,var(--ac),var(--ac2));
    display:flex;align-items:center;justify-content:center;font-size:22px;flex-shrink:0}
  .hdr h1{font-family:'Outfit',sans-serif;font-size:24px;font-weight:800}
  .hdr h1 small{color:var(--mut);font-size:13px;font-weight:400;margin-left:8px}
  .nav{display:flex;gap:14px;font-size:13px;margin-bottom:24px;flex-wrap:wrap}
  .nav a{color:var(--ac);text-decoration:none;opacity:.85;transition:opacity .2s}.nav a:hover{opacity:1}
  .nav .sep{color:var(--line)}
  .pill{display:inline-flex;align-items:center;gap:5px;padding:4px 11px;border-radius:99px;
    font-size:12px;font-weight:600;background:#1e2d47;transition:all .3s}
  .pill.ok{background:#064e3b;color:#6ee7b7}.pill.bad{background:#4c0519;color:#fda4af}
  /* big distance card */
  .dist-card{background:var(--card);border:1px solid var(--line);border-radius:24px;
    padding:36px 28px;text-align:center;position:relative;overflow:hidden;margin-bottom:20px}
  .dist-card::before{content:'';position:absolute;inset:0;border-radius:24px;
    background:radial-gradient(circle at 50% 30%,rgba(6,182,212,.12),transparent 70%);pointer-events:none}
  .dist-label{font-size:13px;font-weight:600;color:var(--mut);text-transform:uppercase;letter-spacing:.06em;margin-bottom:16px}
  .dist-val{font-family:'Outfit',sans-serif;font-size:88px;font-weight:800;line-height:1;
    background:linear-gradient(135deg,var(--ac),#fff);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
  .dist-unit{font-family:'Outfit',sans-serif;font-size:28px;font-weight:400;color:var(--mut);margin-top:4px}
  .dist-raw{font-size:13px;color:var(--mut);margin-top:12px}
  /* proximity bar */
  .prox-wrap{margin:24px 0 8px}
  .prox-label{display:flex;justify-content:space-between;font-size:12px;color:var(--mut);margin-bottom:8px}
  .prox-bar{height:14px;background:#111827;border-radius:8px;overflow:hidden;position:relative}
  .prox-fill{height:100%;border-radius:8px;width:0%;
    background:linear-gradient(90deg,var(--grn),var(--ac));transition:width .3s,background .3s}
  .prox-fill.warn{background:linear-gradient(90deg,#f59e0b,#f97316)}
  .prox-fill.alert{background:linear-gradient(90deg,var(--red),#f97316)}
  /* stats row */
  .stats-row{display:grid;grid-template-columns:1fr 1fr 1fr;gap:12px;margin-top:20px}
  .stat-box{background:var(--card);border:1px solid var(--line);border-radius:14px;
    padding:14px;text-align:center}
  .stat-box .st-label{font-size:11px;font-weight:600;color:var(--mut);text-transform:uppercase;letter-spacing:.05em;margin-bottom:6px}
  .stat-box .st-val{font-family:'Outfit',sans-serif;font-size:22px;font-weight:700;color:#e2eaf8}
  /* history chart */
  .chart-wrap{background:var(--card);border:1px solid var(--line);border-radius:18px;
    padding:20px;margin-top:20px}
  .chart-title{font-size:13px;font-weight:600;color:var(--mut);margin-bottom:12px}
  canvas#hist{width:100%;height:120px;display:block}
  /* alert banner */
  .alert-banner{display:none;align-items:center;gap:10px;padding:12px 16px;border-radius:12px;
    background:#422006;border:1px solid #f97316;color:#fed7aa;font-size:13px;font-weight:600;margin-bottom:16px}
  .alert-banner.show{display:flex}
  /* offline overlay */
  .offline-banner{display:none;align-items:center;gap:10px;padding:12px 16px;border-radius:12px;
    background:#4c0519;border:1px solid var(--red);color:#fda4af;font-size:13px;font-weight:600;margin-bottom:16px}
  .offline-banner.show{display:flex}
</style>
</head><body>
<div class="wrap">
  <div class="hdr">
    <div class="hdr-icon">📡</div>
    <h1>ToF Distance <small>VL53L0X</small></h1>
  </div>
  <nav class="nav">
    <a href="/">🤖 Robot</a><span class="sep">|</span>
    <a href="/color">🎨 Colour</a><span class="sep">|</span>
    <a href="/line">📊 Line</a><span class="sep">|</span>
    <a href="/linefollow">🏁 Line Follow</a><span class="sep">|</span>
    <a href="/comp" style="color:#f59e0b;font-weight:700">🏆 Competition</a><span class="sep">|</span>
    <span class="pill" id="st">⏳ Connecting…</span>
  </nav>

  <div class="offline-banner" id="offBanner">⚠️ Sensor OFFLINE — check SDA=GPIO11, SCL=GPIO12, VIN=3.3V</div>
  <div class="alert-banner"   id="altBanner">⚠️ Object very close! (&lt; 40 mm)</div>

  <!-- big distance display -->
  <div class="dist-card">
    <div class="dist-label">📡 Front Distance</div>
    <div class="dist-val" id="dv">—</div>
    <div class="dist-unit">mm</div>
    <div class="dist-raw" id="dr">Raw: — mm</div>

    <!-- proximity bar -->
    <div class="prox-wrap">
      <div class="prox-label"><span>Close (30mm)</span><span>Far (200mm)</span></div>
      <div class="prox-bar"><div class="prox-fill" id="pf"></div></div>
    </div>
  </div>

  <!-- stats -->
  <div class="stats-row">
    <div class="stat-box"><div class="st-label">Min</div><div class="st-val" id="smin">—</div></div>
    <div class="stat-box"><div class="st-label">Max</div><div class="st-val" id="smax">—</div></div>
    <div class="stat-box"><div class="st-label">Avg</div><div class="st-val" id="savg">—</div></div>
  </div>

  <!-- history chart -->
  <div class="chart-wrap">
    <div class="chart-title">Distance History (last 60 readings)</div>
    <canvas id="hist" width="580" height="120"></canvas>
  </div>
</div>

<script>
const N_HIST = 60, MAX_MM = 200, NEAR_THR = 40;
let hist = [], mn = Infinity, mx = -Infinity, sum = 0, cnt = 0;

function lerp(a,b,t){return a+(b-a)*t}
function drawChart(){
  const c=document.getElementById('hist'),ctx=c.getContext('2d');
  const W=c.offsetWidth||580,H=120;
  c.width=W;c.height=H;
  ctx.clearRect(0,0,W,H);
  ctx.fillStyle='#0e1624';ctx.fillRect(0,0,W,H);
  // grid
  ctx.strokeStyle='#1e2d47';ctx.lineWidth=1;
  for(let y=0;y<=4;y++){const py=H-y*(H/4);ctx.beginPath();ctx.moveTo(0,py);ctx.lineTo(W,py);ctx.stroke();}
  if(hist.length<2)return;
  // line
  ctx.beginPath();ctx.lineWidth=2.5;
  const dx=W/(N_HIST-1);
  hist.forEach((v,i)=>{
    const px=i*dx,py=H-((v/MAX_MM)*H*.9+H*.05);
    i===0?ctx.moveTo(px,py):ctx.lineTo(px,py);
  });
  const g=ctx.createLinearGradient(0,0,W,0);
  g.addColorStop(0,'#8b5cf6');g.addColorStop(1,'#06b6d4');
  ctx.strokeStyle=g;ctx.stroke();
  // fill
  ctx.lineTo((hist.length-1)*dx,H);ctx.lineTo(0,H);ctx.closePath();
  const gf=ctx.createLinearGradient(0,0,0,H);
  gf.addColorStop(0,'rgba(6,182,212,.25)');gf.addColorStop(1,'rgba(6,182,212,0)');
  ctx.fillStyle=gf;ctx.fill();
}

async function poll(){
  try{
    const r=await fetch('/tof/data');
    const d=await r.json();
    const ok=d.ok;
    // status pill
    const st=document.getElementById('st');
    st.textContent=ok?'✅ Online':'❌ Offline';
    st.className='pill '+(ok?'ok':'bad');
    // offline banner
    document.getElementById('offBanner').classList.toggle('show',!ok);
    if(!ok){setTimeout(poll,3000);return;}
    const v=d.front, raw=d.front_raw;
    // big value
    document.getElementById('dv').textContent=v.toFixed(1);
    document.getElementById('dr').textContent='Raw: '+raw.toFixed(1)+' mm';
    // alert
    document.getElementById('altBanner').classList.toggle('show',v<NEAR_THR&&v>0);
    // proximity bar
    const pct=Math.min(100,Math.max(0,((v-30)/(MAX_MM-30))*100));
    const pf=document.getElementById('pf');
    pf.style.width=pct+'%';
    pf.className='prox-fill'+(v<60?' alert':v<100?' warn':'');
    // stats
    if(v>0){mn=Math.min(mn,v);mx=Math.max(mx,v);sum+=v;cnt++;}
    document.getElementById('smin').textContent=isFinite(mn)?mn.toFixed(0)+'mm':'—';
    document.getElementById('smax').textContent=isFinite(mx)?mx.toFixed(0)+'mm':'—';
    document.getElementById('savg').textContent=cnt>0?(sum/cnt).toFixed(0)+'mm':'—';
    // history
    hist.push(v);if(hist.length>N_HIST)hist.shift();
    drawChart();
  }catch(e){
    document.getElementById('st').textContent='⚠️ No Response';
    document.getElementById('st').className='pill bad';
  }
  // Slow to 3 s when tab is hidden, 500 ms when visible (was 200ms)
  setTimeout(poll, document.hidden ? 3000 : 500);
}
poll();
window.addEventListener('resize',drawChart);
</script>
</body></html>
)rawliteral";