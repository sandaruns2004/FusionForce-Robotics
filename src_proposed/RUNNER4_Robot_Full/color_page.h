#pragma once
#include <Arduino.h>

// Colour sensor web page, served at  http://192.168.4.1/color
const char COLOR_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>TCS34725 Colour Lab</title>
<style>
:root{--bg:#0b1220;--card:#131c2e;--in:#0b1424;--line:#243049;--mut:#8da0bd;--ac:#38bdf8}
*{box-sizing:border-box}body{margin:0;font:15px system-ui,Arial;background:var(--bg);color:#e8eefc}
.wrap{max-width:1000px;margin:auto;padding:16px}h1{margin:0;font-size:24px}h1 small{color:var(--mut);font-size:13px}
h2{margin:0 0 12px;font-size:16px}.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:16px;margin-top:14px}
.pill{display:inline-block;padding:4px 10px;border-radius:99px;font-size:12px;background:#334155;margin:6px 6px 0 0}
.ok{background:#14532d;color:#bbf7d0}.bad{background:#7f1d1d;color:#fecaca}.warn{background:#713f12;color:#fde68a}
.hero{display:flex;gap:20px;align-items:center;flex-wrap:wrap}#circ{width:90px;height:90px;border-radius:50%;background:#64748b}
.big{font-size:40px;font-weight:800}.mut{color:var(--mut);font-size:13px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(130px,1fr));gap:10px}
.m{background:var(--in);border-radius:10px;padding:10px}.m .t{color:var(--mut);font-size:12px}.m .v{font-size:22px;font-weight:700}
.bar{height:8px;background:#020617;border-radius:6px;margin-top:6px;overflow:hidden}.bar i{display:block;height:100%;width:0}
canvas{width:100%;height:150px;background:var(--in);border-radius:10px}
.chips span{display:inline-block;width:26px;height:26px;border-radius:6px;margin:2px;background:#475569}
button,select{font:inherit;border-radius:9px;border:0;padding:10px 12px;background:#2563eb;color:#fff;cursor:pointer}
button.s{background:#334155}button:disabled{opacity:.4}select{background:var(--in);border:1px solid var(--line)}
.btns{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:8px;margin-top:8px}
.ctl{margin-bottom:12px}.ch{display:flex;justify-content:space-between;font-size:13px}.ch b{color:var(--ac)}input[type=range]{width:100%}
table{width:100%;border-collapse:collapse}td{padding:6px 4px;border-bottom:1px solid var(--line);font-size:13px}td:first-child{color:var(--mut)}
label{font-size:13px;display:flex;gap:6px;align-items:center}
</style></head><body><div class="wrap">
<h1>🎨 TCS34725 Colour Lab <small>RUNNER-4 · ESP32-S3</small></h1><a href="/" style="color:#38bdf8;font-size:13px">← Back to robot control</a>
<span id="pk" class="pill">Sensor…</span><span id="lk" class="pill">WiFi…</span><span id="rt" class="pill">RTT</span><span id="rate" class="pill"></span><span id="st" class="pill"></span>

<div class="card hero"><div id="circ"></div>
<div><div class="mut">CONFIRMED (stable)</div><div id="conf" class="big">–</div><div class="mut">instant <b id="inst">-</b> · voted <b id="voted">-</b> · stable <b id="stable">0</b></div></div>
<div style="margin-left:auto"><div class="mut">JUNCTION TEST (ball = <b id="ball">none</b>)</div><div id="match" class="pill">–</div></div></div>

<div class="card"><h2>Live data</h2><div class="grid">
<div class="m"><div class="t">RED</div><div id="r" class="v">0</div></div><div class="m"><div class="t">GREEN</div><div id="g" class="v">0</div></div>
<div class="m"><div class="t">BLUE</div><div id="b" class="v">0</div></div><div class="m"><div class="t">CLEAR <span id="cpct"></span></div><div id="c" class="v">0</div></div>
<div class="m"><div class="t">r/c (WB)</div><div id="nr" class="v">0</div><div class="bar"><i id="br" style="background:#ef4444"></i></div></div>
<div class="m"><div class="t">g/c (WB)</div><div id="ng" class="v">0</div><div class="bar"><i id="bg" style="background:#22c55e"></i></div></div>
<div class="m"><div class="t">b/c (WB)</div><div id="nb" class="v">0</div><div class="bar"><i id="bb" style="background:#3b82f6"></i></div></div>
<div class="m"><div class="t">HUE °</div><div id="h" class="v">0</div></div><div class="m"><div class="t">SAT</div><div id="s" class="v">0</div></div>
<div class="m"><div class="t">LUX (approx)</div><div id="lux" class="v">0</div></div></div>
<canvas id="cv" width="900" height="150" style="margin-top:12px"></canvas>
<div class="mut" style="margin-top:8px">Vote window (newest right):</div><div id="votes" class="chips"></div></div>

<div class="card"><h2>🎯 Ball colour memory (Task 1 / Task 4 test)</h2>
<div class="mut">Point at the ball, wait for a CONFIRMED colour, store it. Then point at floor zones: the badge shows TAKE BRANCH when they match.</div>
<div class="btns"><button onclick="cmd('ball_store')">Store confirmed as ball</button><button class="s" onclick="cmd('ball_clear')">Clear ball</button></div></div>

<div class="card"><h2>🧪 Calibration wizard</h2>
<div class="mut">Order matters: 1 White → 2 Black → 3 R, G, B. Keep the same height, gain and lighting. Each capture averages 20 samples.</div>
<div class="btns"><button onclick="cmd('cap_white')">1 · White card</button><button onclick="cmd('cap_black')">2 · Black / dark</button>
<button onclick="cmd('cap_red')" style="background:#b91c1c">3a · Red</button><button onclick="cmd('cap_green')" style="background:#15803d">3b · Green</button><button onclick="cmd('cap_blue')" style="background:#1d4ed8">3c · Blue</button></div>
<div class="bar" style="height:10px;margin-top:12px"><i id="capbar" style="background:var(--ac)"></i></div><div id="capmsg" class="mut" style="margin-top:6px"></div>
<div id="msg" class="mut"></div></div>

<div class="card"><h2>⚙️ Sensor &amp; mode</h2><div class="btns">
<div><div class="mut">Integration</div><select id="atime" onchange="sv('atime',this.value)"><option value="0">2.4 ms</option><option value="1">24 ms</option><option value="2">50 ms</option><option value="3">103 ms</option><option value="4">154 ms</option></select></div>
<div><div class="mut">Gain</div><select id="gain" onchange="sv('gain',this.value)"><option value="0">1×</option><option value="1">4×</option><option value="2">16×</option><option value="3">60×</option></select></div>
<div><div class="mut">Classifier</div><select id="mode" onchange="sv('mode',this.value)"><option value="0">Ratio dominance</option><option value="1">HSV hue</option><option value="2">Both must agree</option></select></div>
<div><label><input type="checkbox" id="led" onchange="sv('led',this.checked?1:0)">Sensor LED</label><label><input type="checkbox" id="auto" onchange="sv('auto',this.checked?1:0)">Auto-gain</label><label><input type="checkbox" id="wb" onchange="sv('wb',this.checked?1:0)">White balance</label></div></div></div>

<div class="card"><h2>🎚️ Tuning</h2><div id="sl"></div></div>

<div class="card"><h2>📡 System &amp; WiFi check</h2><table id="sys"></table>
<div class="btns"><button onclick="cmd('save')">💾 Save to flash</button><button class="s" onclick="cmd('flush')">Flush filters</button><button class="s" onclick="cmd('reinit')">Re-init sensor</button>
<button class="s" onclick="window.open('/color/export')">Export STM32 code</button><button class="s" onclick="if(confirm('Reset all settings?'))cmd('reset')">Reset defaults</button></div></div>
</div>
<script>
const $=id=>document.getElementById(id),COL={RED:'#ef4444',GREEN:'#22c55e',BLUE:'#3b82f6'};
const SL=[['red','Red min r/c',0,1,.01],['green','Green min g/c',0,1,.01],['blue','Blue min b/c',0,1,.01],
['domr','Red dominance ×',1,3,.01],['domg','Green dominance ×',1,3,.01],['domb','Blue dominance ×',1,3,.01],
['smin','HSV min saturation',0,1,.01],['htol','HSV hue tolerance °',5,90,1],['alpha','EMA α',.1,1,.05],
['minc','Min clear (dark reject)',0,5000,10],['vote','Vote window',3,15,1],['conf','Confirm count',1,10,1]];
let hist=[],tm={};
$('sl').innerHTML=SL.map(s=>`<div class=ctl><div class=ch><span>${s[1]}</span><b id=v_${s[0]}></b></div><input type=range id=s_${s[0]} min=${s[2]} max=${s[3]} step=${s[4]} oninput="sv('${s[0]}',this.value)"></div>`).join('');
function sv(k,v){if($('v_'+k))$('v_'+k).textContent=v;clearTimeout(tm[k]);tm[k]=setTimeout(()=>fetch('/color/set?'+k+'='+v),120)}
function cmd(n){fetch('/color/cmd?n='+n).then(r=>r.text()).then(t=>{$('msg').textContent=t;if(n=='reset')loadCfg()})}
async function loadCfg(){const c=await(await fetch('/color/config')).json();
for(const s of SL){$('s_'+s[0]).value=c[s[0]];$('v_'+s[0]).textContent=c[s[0]]}
for(const k of['atime','gain','mode'])$(k).value=c[k];$('led').checked=c.led;$('auto').checked=c.auto;$('wb').checked=c.wb}
function pill(id,t,cls){$(id).textContent=t;$(id).className='pill '+cls}
function draw(){const cv=$('cv'),x=cv.getContext('2d'),W=cv.width,Hh=cv.height;x.clearRect(0,0,W,Hh);x.strokeStyle='#243049';
for(let i=1;i<4;i++){x.beginPath();x.moveTo(0,Hh*i/4);x.lineTo(W,Hh*i/4);x.stroke()}
['#ef4444','#22c55e','#3b82f6'].forEach((col,k)=>{x.strokeStyle=col;x.lineWidth=2;x.beginPath();
hist.forEach((p,i)=>{const X=i*W/149,Y=Hh-p[k]*Hh;i?x.lineTo(X,Y):x.moveTo(X,Y)});x.stroke()})}
function render(d){const col=COL[d.conf]||'#64748b';$('circ').style.background=col;$('circ').style.boxShadow='0 0 40px '+col;
$('conf').textContent=d.conf;$('inst').textContent=d.inst;$('voted').textContent=d.voted;$('stable').textContent=d.stable;
for(const k of['r','g','b','c'])$(k).textContent=d[k];
$('nr').textContent=d.nr.toFixed(3);$('ng').textContent=d.ng.toFixed(3);$('nb').textContent=d.nb.toFixed(3);
$('br').style.width=d.nr*100+'%';$('bg').style.width=d.ng*100+'%';$('bb').style.width=d.nb*100+'%';
$('h').textContent=d.h.toFixed(0);$('s').textContent=d.s.toFixed(2);$('lux').textContent=d.lux.toFixed(0);
$('cpct').textContent='('+Math.round(d.c/d.mx*100)+'% of '+d.mx+')';
pill('pk',d.ok?'Sensor online':'Sensor OFFLINE',d.ok?'ok':'bad');
pill('st','Status: '+d.status,d.status=='OK'?'ok':'warn');$('rate').textContent=d.rate.toFixed(1)+' Hz · '+d.it+' ms · '+d.gain+'× gain';
$('ball').textContent=d.ball;const m=$('match');
if(d.match==-2)pill('match','No ball stored','');else if(d.match==-1)pill('match','Waiting for stable colour…','warn');
else if(d.match==1)pill('match','✔ MATCH – TAKE BRANCH','ok');else pill('match','✘ WRONG BRANCH','bad');
$('votes').innerHTML=d.votes.map(v=>`<span style="background:${['#475569','#ef4444','#22c55e','#3b82f6'][v]}"></span>`).join('');
$('capbar').style.width=(d.cap?d.capn/20*100:0)+'%';$('capmsg').textContent=d.capmsg;
hist.push([d.nr,d.ng,d.nb]);if(hist.length>150)hist.shift();draw()}
async function upd(){const t=performance.now();
try{const d=await(await fetch('/color/data')).json();pill('rt',Math.round(performance.now()-t)+' ms round-trip','ok');pill('lk','WiFi link OK','ok');render(d)}
catch(e){pill('lk','WiFi / server lost','bad')}setTimeout(upd,250)}
async function sys(){try{const w=await(await fetch('/color/wifi')).json();
$('sys').innerHTML=Object.entries(w).map(([k,v])=>`<tr><td>${k}</td><td>${v}</td></tr>`).join('')}catch(e){}setTimeout(sys,3000)}
loadCfg();upd();sys();
</script></body></html>
)rawliteral";
