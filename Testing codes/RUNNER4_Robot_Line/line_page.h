#pragma once
#include <Arduino.h>

// Line array web page, served at  http://192.168.4.1/line
const char LINE_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Line Array Lab</title>
<style>
:root{--bg:#0b1220;--card:#131c2e;--in:#0b1424;--line:#243049;--mut:#8da0bd;--ac:#38bdf8}
*{box-sizing:border-box}body{margin:0;font:15px system-ui,Arial;background:var(--bg);color:#e8eefc}
.wrap{max-width:1000px;margin:auto;padding:16px}h1{margin:0;font-size:24px}h1 small{color:var(--mut);font-size:13px}
h2{margin:0 0 12px;font-size:16px}.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin-top:14px}
.pill{display:inline-block;padding:4px 10px;border-radius:99px;font-size:12px;background:#334155;margin:6px 6px 0 0}
.ok{background:#14532d;color:#bbf7d0}.bad{background:#7f1d1d;color:#fecaca}.warn{background:#713f12;color:#fde68a}
.hero{display:flex;gap:24px;align-items:center;flex-wrap:wrap}.big{font-size:40px;font-weight:800}.mut{color:var(--mut);font-size:13px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(120px,1fr));gap:10px}
.m{background:var(--in);border-radius:10px;padding:10px}.m .t{color:var(--mut);font-size:12px}.m .v{font-size:22px;font-weight:700}
.track{position:relative;height:18px;background:var(--in);border-radius:9px;margin-top:14px;border:1px solid var(--line)}
.track:after{content:'';position:absolute;left:50%;top:0;bottom:0;width:2px;background:#475569}
#mk{position:absolute;top:-3px;width:24px;height:24px;margin-left:-12px;border-radius:50%;background:var(--ac);box-shadow:0 0 14px var(--ac);left:50%;transition:left .08s}
.sens{display:grid;grid-template-columns:repeat(9,1fr);gap:6px}.sc{background:var(--in);border-radius:10px;padding:8px 4px;text-align:center;border:2px solid transparent}
.sc.on{border-color:#facc15}.sc.dead{opacity:.35}.bw{position:relative;height:110px;background:#020617;border-radius:6px;margin:6px auto;width:60%;overflow:hidden}
.bw i{position:absolute;left:0;right:0;bottom:0;background:linear-gradient(#e2e8f0,#64748b)}.bw u{position:absolute;left:0;right:0;border-top:2px dashed #f87171}
.rv{font-size:11px;color:var(--mut)}.nv{font-size:13px;font-weight:700}.sn{font-size:12px;color:var(--ac)}
canvas{width:100%;height:150px;background:var(--in);border-radius:10px}
button{font:inherit;border-radius:9px;border:0;padding:10px 12px;background:#2563eb;color:#fff;cursor:pointer}button.s{background:#334155}
.btns{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:8px;margin-top:8px}
.bar{height:8px;background:#020617;border-radius:6px;margin-top:6px;overflow:hidden}.bar i{display:block;height:100%;width:0;background:var(--ac)}
.ctl{margin-bottom:12px}.ch{display:flex;justify-content:space-between;font-size:13px}.ch b{color:var(--ac)}input[type=range]{width:100%}
table{width:100%;border-collapse:collapse}td{padding:6px 4px;border-bottom:1px solid var(--line);font-size:13px}td:first-child{color:var(--mut)}
label{font-size:13px;display:flex;gap:6px;align-items:center;margin-top:6px}
</style></head><body><div class="wrap">
<h1>📏 Line Array Lab <small>RUNNER-4 · 9× TCRT5000 · ESP32-S3</small></h1>
<a href="/" style="color:#38bdf8;font-size:13px">← Robot control</a> &nbsp; <a href="/color" style="color:#38bdf8;font-size:13px">Colour lab →</a><br>
<span id="pk" class="pill">Array…</span><span id="lk" class="pill">WiFi…</span><span id="rt" class="pill">RTT</span><span id="rate" class="pill"></span><span id="st" class="pill"></span>

<div class="card"><div class="hero">
<div><div class="mut">LINE ERROR (− left · + right)</div><div id="err" class="big">–</div><div class="mut">mm</div></div>
<div><div class="mut">Wz (PD output)</div><div id="wz" class="big">–</div><div class="mut">&lt;0 left · &gt;0 right</div></div>
<div><div class="mut">Active sensors</div><div id="act" class="big">0</div><div class="mut">centroid <b id="cen">-</b></div></div>
<div style="margin-left:auto"><div class="mut">JUNCTION</div><div id="jun" class="pill">–</div><div class="mut" style="margin-top:6px">LOST</div><div id="lost" class="pill">–</div></div></div>
<div class="track"><div id="mk"></div></div></div>

<div class="card"><h2>Sensors (S1 left → S9 right)</h2><div id="sens" class="sens"></div>
<div class="mut" style="margin-top:8px">Bar = normalised 0..1 · dashed red = threshold · yellow frame = active · faded = dead/flat sensor</div>
<canvas id="cv" width="900" height="150" style="margin-top:12px"></canvas>
<div class="mut">Error history (±45 mm, last 150 cycles)</div></div>

<div class="card"><h2>🧪 Calibration (replaces the 5 s power-up sweep)</h2>
<div class="mut">Press Start, then slide the array across BLACK floor and the WHITE line for 5 s. Check that every sensor shows a good span, then Save to flash.</div>
<div class="btns"><button onclick="cmd('cal_start')">▶ Start 5 s sweep</button><button class="s" onclick="cmd('cal_stop')">■ Finish now</button><button class="s" onclick="cmd('cal_clear')">Clear calibration</button></div>
<div class="bar" style="height:10px;margin-top:12px"><i id="calbar"></i></div><div id="calmsg" class="mut" style="margin-top:6px"></div><div id="msg" class="mut"></div></div>

<div class="card"><h2>🎚️ Tuning</h2><div id="sl"></div>
<label><input type="checkbox" id="white" onchange="sv('white',this.checked?1:0)">White line is HIGH (uncheck if white reads low)</label>
<label><input type="checkbox" id="log" onchange="sv('log',this.checked?1:0)">Serial log (err / active / Wz, like the standalone sketch)</label></div>

<div class="card"><h2>📡 System check</h2><table id="sys"></table>
<div class="btns"><button onclick="cmd('save')">💾 Save to flash</button><button class="s" onclick="cmd('flush')">Flush filters</button>
<button class="s" onclick="window.open('/line/export')">Export constants</button><button class="s" onclick="if(confirm('Reset tuning (keeps calibration)?'))cmd('reset')">Reset tuning</button></div></div>
</div>
<script>
const $=id=>document.getElementById(id);
const SL=[['thr','Line threshold (0-1)',.05,.95,.01],['kp','Kp (Wz per mm)',0,.3,.005],['kd','Kd',0,1,.01],['alpha','Raw EMA α (1 = off)',.1,1,.05],
['wzmax','Wz limit',.1,5,.1],['spacing','Sensor spacing mm',5,20,.5],['minspan','Dead-sensor ADC span',20,1000,10],['cyc','Control cycle ms',5,100,1],
['lost','Lost failsafe ms',500,10000,100],['samples','ADC oversample',1,16,1],['jmin','Junction min sensors',3,9,1],['jcyc','Junction cycles',1,10,1]];
let hist=[],tm={};
$('sl').innerHTML=SL.map(s=>`<div class=ctl><div class=ch><span>${s[1]}</span><b id=v_${s[0]}></b></div><input type=range id=s_${s[0]} min=${s[2]} max=${s[3]} step=${s[4]} oninput="sv('${s[0]}',this.value)"></div>`).join('');
$('sens').innerHTML=Array.from({length:9},(_,i)=>`<div class=sc id=sc${i}><div class=rv id=rv${i}>0</div><div class=bw><i id=bar${i}></i><u id=th${i}></u></div><div class=nv id=nv${i}>0.00</div><div class=sn>S${i+1}</div></div>`).join('');
function sv(k,v){if($('v_'+k))$('v_'+k).textContent=v;clearTimeout(tm[k]);tm[k]=setTimeout(()=>fetch('/line/set?'+k+'='+v),120)}
function cmd(n){fetch('/line/cmd?n='+n).then(r=>r.text()).then(t=>{$('msg').textContent=t;if(n=='reset')loadCfg()})}
async function loadCfg(){const c=await(await fetch('/line/config')).json();
for(const s of SL){$('s_'+s[0]).value=c[s[0]];$('v_'+s[0]).textContent=c[s[0]]}$('white').checked=c.white;$('log').checked=c.log}
function pill(id,t,cls){$(id).textContent=t;$(id).className='pill '+cls}
function draw(){const cv=$('cv'),x=cv.getContext('2d'),W=cv.width,H=cv.height;x.clearRect(0,0,W,H);x.strokeStyle='#243049';x.lineWidth=1;
for(let i=0;i<5;i++){x.beginPath();x.moveTo(0,H*i/4);x.lineTo(W,H*i/4);x.stroke()}
x.strokeStyle='#38bdf8';x.lineWidth=2;x.beginPath();hist.forEach((e,i)=>{const X=i*W/149,Y=H/2-(Math.max(-45,Math.min(45,e))/45)*H/2;i?x.lineTo(X,Y):x.moveTo(X,Y)});x.stroke()}
function render(d){
pill('pk',d.cal?'Calibrated':'NO CALIBRATION',d.cal?'ok':'bad');
pill('st','Status: '+d.status,d.status=='OK'?'ok':(d.status=='JUNCTION'?'warn':'bad'));
$('rate').textContent=d.rate.toFixed(1)+' Hz · scan '+d.scanus+' µs';
$('err').textContent=d.found?d.err.toFixed(1):'–';$('wz').textContent=d.found?d.wz.toFixed(2):'–';
$('act').textContent=d.act;$('cen').textContent=d.found?d.cen.toFixed(2):'-';
$('mk').style.left=(d.found?d.cen/8*100:50)+'%';$('mk').style.opacity=d.found?1:.25;
if(d.jun)pill('jun','JUNCTION · '+d.jt,'warn');else pill('jun','none','');
if(d.safe)pill('lost','SAFE STOP · last side '+d.side,'bad');else if(d.lost)pill('lost','LOST '+d.lostms+' ms · last '+d.side,'warn');else pill('lost','line OK','ok');
for(let i=0;i<9;i++){const n=d.norm[i]/1000;$('rv'+i).textContent=d.raw[i];$('nv'+i).textContent=n.toFixed(2);
$('bar'+i).style.height=(n*100)+'%';$('th'+i).style.bottom=(d.thr*100)+'%';
$('sc'+i).className='sc'+((d.mask>>i)&1?' on':'')+((d.okmask>>i)&1?'':' dead')}
$('calbar').style.width=(d.calact?(1-d.calleft/5000)*100:0)+'%';$('calmsg').textContent=d.calmsg;
if(d.found){hist.push(d.err);if(hist.length>150)hist.shift();draw()}}
async function upd(){const t=performance.now();
try{const d=await(await fetch('/line/data')).json();pill('rt',Math.round(performance.now()-t)+' ms round-trip','ok');pill('lk','WiFi link OK','ok');render(d)}
catch(e){pill('lk','WiFi / server lost','bad')}setTimeout(upd,150)}
async function sys(){try{const w=await(await fetch('/line/wifi')).json();
$('sys').innerHTML=Object.entries(w).map(([k,v])=>`<tr><td>${k}</td><td>${v}</td></tr>`).join('')}catch(e){}setTimeout(sys,3000)}
loadCfg();upd();sys();
</script></body></html>
)rawliteral";
