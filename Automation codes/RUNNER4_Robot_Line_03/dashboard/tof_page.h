#pragma once
#include <Arduino.h>

// ToF sensor dashboard page - served at http://192.168.4.1/tof
// 3x VL53L0X - Front (GPIO13 XSHUT -> 0x30), Right (GPIO14 -> 0x31), Left (GPIO21 -> 0x32)
const char TOF_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ToF Sensors - RUNNER4</title>
<style>
:root{--bg:#080d18;--card:#0e1624;--line:#1e2d47;--mut:#6b82a0;
  --ac:#06b6d4;--ac2:#8b5cf6;--red:#ef4444;--grn:#22c55e;--warn:#f59e0b}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:#e2eaf8;font-family:'Inter',system-ui,sans-serif;min-height:100vh}
.wrap{max-width:860px;margin:0 auto;padding:20px 14px}
.hdr{display:flex;align-items:center;gap:12px;margin-bottom:4px}
.hdr-icon{width:44px;height:44px;border-radius:12px;background:linear-gradient(135deg,var(--ac),var(--ac2));display:flex;align-items:center;justify-content:center;font-size:20px;flex-shrink:0}
.hdr h1{font-size:22px;font-weight:800}
.hdr h1 small{color:var(--mut);font-size:12px;font-weight:400;margin-left:6px}
.nav{display:flex;gap:12px;font-size:13px;margin-bottom:18px;flex-wrap:wrap}
.nav a{color:var(--ac);text-decoration:none;opacity:.8}.nav a:hover{opacity:1}
.nav .sep{color:var(--line)}
/* status bar */
.sbar{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:16px}
.pill{display:inline-flex;align-items:center;gap:5px;padding:4px 11px;border-radius:99px;font-size:12px;font-weight:600;background:#1e2d47;transition:all .3s}
.pill.ok{background:#064e3b;color:#6ee7b7}.pill.bad{background:#4c0519;color:#fda4af}.pill.warn{background:#451a03;color:#fdba74}
/* 3-col sensor grid */
.grid3{display:grid;grid-template-columns:repeat(3,1fr);gap:12px;margin-bottom:14px}
@media(max-width:600px){.grid3{grid-template-columns:1fr}}
.scard{background:var(--card);border:2px solid var(--line);border-radius:18px;padding:18px;text-align:center;position:relative;transition:border-color .3s}
.scard.alive{border-color:var(--ac)}.scard.dead{border-color:var(--red);opacity:.7}
.scard .sc-label{font-size:11px;font-weight:700;color:var(--mut);text-transform:uppercase;letter-spacing:.07em;margin-bottom:8px}
.scard .sc-icon{font-size:28px;margin-bottom:4px}
.scard .sc-dist{font-size:52px;font-weight:800;line-height:1;background:linear-gradient(135deg,var(--ac),#fff);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.scard.dead .sc-dist{background:none;-webkit-text-fill-color:#ef4444;font-size:28px;margin-top:12px}
.scard .sc-unit{color:var(--mut);font-size:14px;margin-top:2px}
.scard .sc-raw{font-size:11px;color:var(--mut);margin-top:6px}
.scard .sc-addr{font-size:11px;color:var(--ac2);margin-top:4px;font-family:monospace}
/* proximity bar */
.pbar-wrap{margin:10px 0 4px}
.pbar-labels{display:flex;justify-content:space-between;font-size:10px;color:var(--mut);margin-bottom:4px}
.pbar{height:8px;background:#111827;border-radius:4px;overflow:hidden}
.pbar-fill{height:100%;border-radius:4px;transition:width .3s,background .3s;background:linear-gradient(90deg,var(--grn),var(--ac))}
.pbar-fill.warn{background:linear-gradient(90deg,var(--warn),#f97316)}
.pbar-fill.alert{background:linear-gradient(90deg,var(--red),#f97316)}
/* debug panel */
.dbg{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin-bottom:14px}
.dbg h2{font-size:13px;font-weight:700;color:var(--mut);text-transform:uppercase;letter-spacing:.06em;margin-bottom:10px;display:flex;align-items:center;gap:8px}
.dbg-row{display:grid;grid-template-columns:repeat(auto-fit,minmax(200px,1fr));gap:8px}
.dbg-box{background:#060d1a;border:1px solid var(--line);border-radius:10px;padding:10px}
.dbg-box .dt{font-size:10px;color:var(--mut);text-transform:uppercase;letter-spacing:.05em;margin-bottom:4px}
.dbg-box .dv{font-size:13px;font-weight:700;font-family:monospace;color:#e2eaf8}
.i2c-tags{display:flex;flex-wrap:wrap;gap:6px;margin-top:6px}
.i2c-tag{font-size:11px;font-family:monospace;padding:3px 8px;border-radius:6px;background:#1e3a5f;color:#7dd3fc}
.i2c-tag.known{background:#14532d;color:#6ee7b7}
.i2c-tag.tof{background:#3b1f6b;color:#c4b5fd}
/* history chart */
.chart-wrap{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin-bottom:14px}
.chart-wrap h2{font-size:13px;font-weight:700;color:var(--mut);text-transform:uppercase;letter-spacing:.06em;margin-bottom:10px}
.chart-legend{display:flex;gap:16px;font-size:11px;margin-bottom:8px}
.chart-legend span{display:flex;align-items:center;gap:5px}
.leg-dot{width:10px;height:10px;border-radius:50%;flex-shrink:0}
canvas#hist{width:100%;height:140px;display:block}
/* rescan button */
.btn{font:inherit;border-radius:9px;border:0;padding:7px 14px;cursor:pointer;font-weight:700;font-size:12px;background:#1e3a5f;color:#7dd3fc;margin-left:auto}
.btn:hover{background:#2563eb;color:#fff}
</style>
</head><body>
<div class="wrap">

<div class="hdr">
  <div class="hdr-icon">📡</div>
  <h1>ToF Distance Sensors <small>3x VL53L0X · RUNNER-4</small></h1>
</div>
<nav class="nav">
  <a href="/">&#129302; Robot</a><span class="sep">|</span>
  <a href="/color">&#127912; Colour</a><span class="sep">|</span>
  <a href="/line">&#128202; Line</a><span class="sep">|</span>
  <a href="/linefollow">&#127937; Line Follow</a><span class="sep">|</span>
  <a href="/comp" style="color:#f59e0b;font-weight:700">&#127942; Competition</a>
</nav>

<div class="sbar">
  <span class="pill" id="p_conn">Connecting...</span>
  <span class="pill" id="p_front">Front ?</span>
  <span class="pill" id="p_right">Right ?</span>
  <span class="pill" id="p_left">Left ?</span>
</div>

<!-- 3-sensor cards -->
<div class="grid3">
  <div class="scard" id="card_front">
    <div class="sc-label">&#9679; Front</div>
    <div class="sc-icon">&#128249;</div>
    <div class="sc-dist" id="d_front">-</div>
    <div class="sc-unit">mm</div>
    <div class="sc-raw" id="r_front">Raw: -</div>
    <div class="sc-addr" id="a_front">0x30 · GPIO 13</div>
    <div class="pbar-wrap">
      <div class="pbar-labels"><span>Close</span><span>Far</span></div>
      <div class="pbar"><div class="pbar-fill" id="pb_front" style="width:0%"></div></div>
    </div>
  </div>
  <div class="scard" id="card_right">
    <div class="sc-label">&#9654; Right</div>
    <div class="sc-icon">&#128249;</div>
    <div class="sc-dist" id="d_right">-</div>
    <div class="sc-unit">mm</div>
    <div class="sc-raw" id="r_right">Raw: -</div>
    <div class="sc-addr" id="a_right">0x31 · GPIO 14</div>
    <div class="pbar-wrap">
      <div class="pbar-labels"><span>Close</span><span>Far</span></div>
      <div class="pbar"><div class="pbar-fill" id="pb_right" style="width:0%"></div></div>
    </div>
  </div>
  <div class="scard" id="card_left">
    <div class="sc-label">&#9664; Left</div>
    <div class="sc-icon">&#128249;</div>
    <div class="sc-dist" id="d_left">-</div>
    <div class="sc-unit">mm</div>
    <div class="sc-raw" id="r_left">Raw: -</div>
    <div class="sc-addr" id="a_left">0x32 · GPIO 21</div>
    <div class="pbar-wrap">
      <div class="pbar-labels"><span>Close</span><span>Far</span></div>
      <div class="pbar"><div class="pbar-fill" id="pb_left" style="width:0%"></div></div>
    </div>
  </div>
</div>

<!-- Distance history chart -->
<div class="chart-wrap">
  <h2>Distance History (last 80 readings)</h2>
  <div class="chart-legend">
    <span><div class="leg-dot" style="background:#06b6d4"></div>Front</span>
    <span><div class="leg-dot" style="background:#22c55e"></div>Right</span>
    <span><div class="leg-dot" style="background:#a78bfa"></div>Left</span>
  </div>
  <canvas id="hist" width="800" height="140"></canvas>
</div>

<!-- Debug panel -->
<div class="dbg">
  <h2>&#128270; Debug &amp; I2C Bus Scan <button class="btn" onclick="runDiag()">&#8635; Rescan I2C</button></h2>
  <div class="dbg-row">
    <div class="dbg-box">
      <div class="dt">Expected addresses</div>
      <div class="dv">0x30 Front · 0x31 Right · 0x32 Left<br>0x40 PCA9685 (servos)</div>
    </div>
    <div class="dbg-box">
      <div class="dt">XSHUT pins</div>
      <div class="dv">Front: GPIO 13 &nbsp; Right: GPIO 14 &nbsp; Left: GPIO 21</div>
    </div>
  </div>
  <div style="margin-top:10px">
    <div class="dt" style="font-size:10px;color:var(--mut);text-transform:uppercase;letter-spacing:.05em;margin-bottom:6px">Live I2C bus scan (0x01-0x7F)</div>
    <div class="i2c-tags" id="i2c_scan">-</div>
  </div>
  <div class="dbg-row" style="margin-top:10px" id="sensor_detail"></div>
</div>

</div>

<script>
const N=80, MAX_MM=200;
let hFront=[], hRight=[], hLeft=[];
const KNOWN = {'0x40':'PCA9685','0x30':'ToF Front','0x31':'ToF Right','0x32':'ToF Left','0x29':'VL53L0X (pre-remap)'};

function pill(id,txt,cls){const e=document.getElementById(id);e.textContent=txt;e.className='pill '+cls}
function pct(mm){return Math.min(100,Math.max(0,((mm-30)/(MAX_MM-30))*100))}
function barCls(mm){return mm<60?'alert':mm<100?'warn':''}

function updateCard(key,dist,raw,ok,addr){
  const dn=document.getElementById('d_'+key);
  const card=document.getElementById('card_'+key);
  const pb=document.getElementById('pb_'+key);
  document.getElementById('r_'+key).textContent='Raw: '+(ok?raw.toFixed(1)+' mm':'—');
  document.getElementById('a_'+key).textContent=addr+' · GPIO '+(key=='front'?13:key=='right'?14:21);
  if(!ok){
    card.className='scard dead';
    dn.style.cssText='';
    dn.textContent='OFFLINE';
    pb.style.width='0%';pb.className='pbar-fill';
    return;
  }
  card.className='scard alive';
  dn.style.cssText='font-size:52px;font-weight:800;line-height:1;background:linear-gradient(135deg,#06b6d4,#fff);-webkit-background-clip:text;-webkit-text-fill-color:transparent';
  dn.textContent=dist.toFixed(0);
  pb.style.width=pct(dist)+'%';
  pb.className='pbar-fill '+barCls(dist);
}

async function poll(){
  try{
    const r=await fetch('/tof/data',{cache:'no-store',signal:AbortSignal.timeout(2000)});
    const d=await r.json();
    pill('p_conn','Connected','ok');
    // status pills
    pill('p_front','Front '+(d.front_ok?d.front.toFixed(0)+'mm':'OFFLINE'),d.front_ok?'ok':'bad');
    pill('p_right','Right '+(d.right_ok?d.right.toFixed(0)+'mm':'OFFLINE'),d.right_ok?'ok':'bad');
    pill('p_left', 'Left ' +(d.left_ok ?d.left.toFixed(0)+'mm' :'OFFLINE'),d.left_ok ?'ok':'bad');
    // cards
    updateCard('front',d.front,d.front_raw,d.front_ok,d.front_addr);
    updateCard('right',d.right,d.right_raw,d.right_ok,d.right_addr);
    updateCard('left', d.left, d.left_raw, d.left_ok, d.left_addr);
    // history
    if(d.front_ok){hFront.push(d.front);if(hFront.length>N)hFront.shift();}
    if(d.right_ok){hRight.push(d.right);if(hRight.length>N)hRight.shift();}
    if(d.left_ok) {hLeft.push(d.left); if(hLeft.length>N)hLeft.shift();}
    drawChart();
  }catch(e){
    pill('p_conn','No Response','bad');
  }
  setTimeout(poll,document.hidden?3000:500);
}

async function runDiag(){
  try{
    const r=await fetch('/tof/diag',{cache:'no-store',signal:AbortSignal.timeout(5000)});
    const d=await r.json();
    // I2C scan tags
    const tags=document.getElementById('i2c_scan');
    if(!d.i2c_scan.length){tags.innerHTML='<span style="color:var(--red)">No devices found!</span>';return;}
    tags.innerHTML=d.i2c_scan.map(a=>{
      const label=KNOWN[a.toLowerCase()]||KNOWN[a]||'';
      const cls=a==='0x40'?'known':a.startsWith('0x3')?'tof':'';
      return `<span class="i2c-tag ${cls}">${a}${label?' \u2014 '+label:''}</span>`;
    }).join('');
    // sensor detail boxes
    const det=document.getElementById('sensor_detail');
    det.innerHTML=d.sensors.map(s=>`
      <div class="dbg-box">
        <div class="dt">${s.name} sensor</div>
        <div class="dv" style="color:${s.ok?'#6ee7b7':'#fda4af'}">${s.ok?'ONLINE':'OFFLINE'}</div>
        <div class="dv" style="font-size:11px;margin-top:4px;color:var(--mut)">
          XSHUT: GPIO ${s.xshut_gpio} &nbsp; Addr: ${s.target_addr}<br>
          Dist: ${s.ok?s.dist.toFixed(1)+' mm':'—'} &nbsp; Raw: ${s.ok?s.raw.toFixed(1)+' mm':'—'}
        </div>
      </div>`).join('');
  }catch(e){document.getElementById('i2c_scan').innerHTML='<span style="color:var(--red)">Scan failed</span>';}
}

function drawChart(){
  const c=document.getElementById('hist'),ctx=c.getContext('2d');
  const W=c.offsetWidth||800,H=140;
  c.width=W;c.height=H;
  ctx.fillStyle='#0e1624';ctx.fillRect(0,0,W,H);
  ctx.strokeStyle='#1e2d47';ctx.lineWidth=1;
  for(let y=0;y<=4;y++){const py=H-y*(H/4);ctx.beginPath();ctx.moveTo(0,py);ctx.lineTo(W,py);ctx.stroke();}
  function drawLine(hist,color){
    if(hist.length<2)return;
    ctx.beginPath();ctx.lineWidth=2;ctx.strokeStyle=color;
    const dx=W/(N-1);
    hist.forEach((v,i)=>{const px=i*dx,py=H-((Math.min(v,MAX_MM)/MAX_MM)*(H*.9)+H*.05);i?ctx.lineTo(px,py):ctx.moveTo(px,py);});
    ctx.stroke();
  }
  drawLine(hFront,'#06b6d4');
  drawLine(hRight,'#22c55e');
  drawLine(hLeft, '#a78bfa');
}

poll();
runDiag();
window.addEventListener('resize',drawChart);
</script>
</body></html>
)rawliteral";