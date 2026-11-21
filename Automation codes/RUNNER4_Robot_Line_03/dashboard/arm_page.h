#pragma once
#include <Arduino.h>
#include <WebServer.h>

// Arm control dashboard — served at http://192.168.4.1/arm
// Pins: Lift=GPIO47, Grip=GPIO44 (direct LEDC PWM, no PCA9685)
// Updated: UP/DOWN buttons with inline angle sliders + independent lift control
const char ARM_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>RUNNER4 — Robot Arm</title>
<style>
:root{--bg:#10151c;--card:#1a222d;--line:#2b3644;--text:#e8eef5;--mute:#8da0b5;
  --blue:#3d8bfd;--green:#2fb36d;--amber:#f0a93a;--red:#e5534b;--pur:#9b6dff;--teal:#22d3ee}
*{box-sizing:border-box;margin:0;padding:0}
body{background:var(--bg);color:var(--text);font-family:system-ui,sans-serif;padding:12px;max-width:640px;margin-inline:auto}
/* header */
.hdr{display:flex;justify-content:space-between;align-items:center;margin-bottom:10px}
.hdr h1{font-size:19px;font-weight:700}
.chip{font-size:12px;padding:4px 12px;border-radius:99px;background:var(--line);color:var(--mute);font-weight:600;transition:.3s}
body.busy .chip{background:var(--amber);color:#111}
/* nav */
.nav{font-size:12px;margin-bottom:12px;display:flex;gap:8px;flex-wrap:wrap}
.nav a{color:var(--blue);text-decoration:none;opacity:.8}.nav a:hover{opacity:1}
.nav a.active{color:var(--amber);font-weight:700;opacity:1}
/* pin info */
.pinbar{background:#0d1724;border:1px solid var(--line);border-radius:9px;padding:7px 12px;font-size:11px;color:var(--mute);font-family:monospace;margin-bottom:10px}
.pinbar b{color:var(--pur)}
/* card */
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px;margin-bottom:10px}
.card-title{font-size:11px;font-weight:700;text-transform:uppercase;letter-spacing:.07em;color:var(--mute);margin-bottom:10px}
/* svg visualizer */
.viz{width:100%;height:auto;display:block;background:var(--bg);border-radius:9px}
.read{display:flex;justify-content:space-around;margin-top:9px;font-size:13px;color:var(--mute)}
.read b{display:block;color:var(--text);font-size:15px;text-align:center;font-weight:700}
/* action section: 2-col gripper / arm */
.action-wrap{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.action-col{background:var(--bg);border:1px solid var(--line);border-radius:11px;padding:12px}
.action-col h4{font-size:11px;text-transform:uppercase;color:var(--mute);font-weight:700;margin-bottom:8px;letter-spacing:.06em}
/* buttons */
button.big{padding:13px 0;border:0;border-radius:10px;font-size:16px;font-weight:700;color:#fff;background:var(--blue);cursor:pointer;transition:.12s;width:100%;margin-bottom:7px}
button.big:last-child{margin-bottom:0}
button.big:active{transform:scale(.96)}
button.big:disabled,body.busy button.big{opacity:.4;cursor:default;pointer-events:none}
.btn-open{background:var(--amber)!important;color:#111!important}
.btn-close{background:#3d4f6b!important}
.btn-up{background:var(--teal)!important}
.btn-down{background:var(--pur)!important}
.btn-pick{background:var(--green)!important}
.btn-place{background:#0e6d4a!important}
/* angle sliders inside arm col */
.aslider{margin-bottom:9px}
.aslider label{display:flex;justify-content:space-between;font-size:12px;color:var(--mute);margin-bottom:3px}
.aslider label output{color:var(--blue);font-weight:700;font-family:monospace}
.aslider .ctl{display:flex;align-items:center;gap:6px}
.aslider input[type=range]{flex:1;accent-color:var(--blue);height:26px;cursor:pointer}
.aslider input[type=range]:disabled{opacity:.4;cursor:default}
.mini{width:34px;height:34px;border-radius:8px;border:1px solid var(--line);background:var(--bg);color:var(--text);font-size:16px;cursor:pointer;flex-shrink:0;font-weight:700;transition:.15s}
.mini:hover{border-color:var(--blue);color:var(--blue)}
body.busy .mini{opacity:.4;pointer-events:none}
/* advanced sections */
details summary{cursor:pointer;font-size:14px;font-weight:600;color:var(--text);padding:3px 0;user-select:none}
details[open] summary{color:var(--blue)}
.row{margin:10px 0}
.row label{display:flex;justify-content:space-between;font-size:13px;margin-bottom:3px;color:var(--mute)}
.row label output{color:var(--blue);font-weight:700;font-family:monospace}
.hint{font-size:11px;color:var(--mute);margin-top:2px}
/* preview checkbox */
.tog{display:flex;gap:7px;align-items:center;font-size:12px;margin:8px 0;color:var(--mute)}
.tog input{accent-color:var(--blue)}
/* toast */
#toast{position:fixed;left:50%;bottom:20px;transform:translate(-50%,80px);
  background:#000d;color:#fff;padding:9px 16px;border-radius:10px;transition:transform .25s;font-size:13px;z-index:999}
#toast.show{transform:translate(-50%,0)}
@media(max-width:420px){.action-wrap{grid-template-columns:1fr}}
</style>
</head>
<body>

<div class="hdr">
  <h1>&#129482; Robot Arm</h1>
  <span class="chip" id="state">Connecting...</span>
</div>

<nav class="nav">
  <a href="/">&#129302; Robot</a>
  <a href="/tof">&#128249; ToF</a>
  <a href="/color">&#127912; Colour</a>
  <a href="/line">&#128202; Line</a>
  <a href="/mpu">&#127996; Gyro</a>
  <a href="/comp" style="color:var(--amber);font-weight:700">&#127942; Competition</a>
  <a href="/arm" class="active">&#129482; Arm</a>
</nav>

<div class="pinbar">
  Lift servo &rarr; <b>GPIO 47</b> (LEDC ch 6, 50 Hz) &nbsp;
  Grip servo &rarr; <b>GPIO 44</b> (LEDC ch 7, 50 Hz) &nbsp;
  Power: <b>5-6V external BEC</b>
</div>

<!-- SVG visualizer -->
<div class="card">
  <svg class="viz" viewBox="0 0 300 160">
    <rect x="8" y="90" width="34" height="62" rx="6" fill="#2b3644"/>
    <text x="25" y="162" text-anchor="middle" font-size="9" fill="#6b82a0">Base</text>
    <g id="arm" style="transform-origin:28px 83px;transition:transform .5s ease">
      <rect x="28" y="76" width="150" height="14" rx="5" fill="#3d8bfd"/>
      <g transform="translate(178,83)">
        <rect id="f1" x="0" y="-10" width="42" height="9" rx="3" fill="#f0a93a" style="transition:transform .4s"/>
        <rect id="f2" x="0" y="1"   width="42" height="9" rx="3" fill="#f0a93a" style="transition:transform .4s"/>
      </g>
    </g>
    <circle cx="28" cy="83" r="11" fill="#e8eef5"/>
    <circle cx="28" cy="83" r="5"  fill="#3d8bfd"/>
    <text x="103" y="72"  text-anchor="middle" font-size="9" fill="#8da0b5">Lift arm</text>
    <text x="197" y="65"  text-anchor="middle" font-size="9" fill="#8da0b5">Gripper</text>
    <!-- angle arc hint -->
    <text x="265" y="20" font-size="9" fill="#334155" id="yawHint"></text>
  </svg>
  <div class="read">
    <div>Gripper<b id="gv">-</b></div>
    <div>Lift<b id="lv">-</b></div>
    <div>State<b id="sv">-</b></div>
  </div>
</div>

<!-- ========= ACTION SECTION ========= -->
<div class="card">
  <div class="card-title">Controls</div>
  <div class="action-wrap">

    <!-- LEFT: Gripper -->
    <div class="action-col">
      <h4>&#127918; Gripper</h4>
      <button class="big btn-open"  onclick="api('/arm/open','Opening...')">OPEN</button>
      <button class="big btn-close" onclick="api('/arm/close','Closing...')">CLOSE</button>
    </div>

    <!-- RIGHT: Lift + angle sliders -->
    <div class="action-col">
      <h4>&#8597; Lift Arm</h4>

      <!-- UP angle slider -->
      <div class="aslider">
        <label>UP angle <output id="o_lup_act">-</output></label>
        <div class="ctl">
          <button class="mini" onclick="adj('lup_act',-1)">-</button>
          <input type="range" id="r_lup_act" min="20" max="160" step="1" value="75"
            oninput="showAct('lup_act')" onchange="sendLiftAngle('lup','lup_act')">
          <button class="mini" onclick="adj('lup_act',1)">+</button>
        </div>
      </div>
      <button class="big btn-up" onclick="armMoveUp()">&#8679; UP</button>

      <!-- DOWN angle slider -->
      <div class="aslider" style="margin-top:8px">
        <label>DOWN angle <output id="o_ldown_act">-</output></label>
        <div class="ctl">
          <button class="mini" onclick="adj('ldown_act',-1)">-</button>
          <input type="range" id="r_ldown_act" min="20" max="160" step="1" value="160"
            oninput="showAct('ldown_act')" onchange="sendLiftAngle('ldown','ldown_act')">
          <button class="mini" onclick="adj('ldown_act',1)">+</button>
        </div>
      </div>
      <button class="big btn-down" onclick="armMoveDown()">&#8681; DOWN</button>
    </div>

  </div>

  <!-- Sequences row -->
  <div style="display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:10px">
    <button class="big btn-pick"  onclick="api('/arm/pick','Picking up...')">&#129354; PICK UP</button>
    <button class="big btn-place" onclick="api('/arm/place','Placing...')">&#128230; PLACE</button>
  </div>
</div>

<!-- Angle fine-tuning (saved to flash) -->
<div class="card">
  <details open>
    <summary>&#127919; Saved Angle Presets</summary>
    <div class="hint" style="margin:7px 0">These are the stored target angles for each action. Saved to flash (NVS).</div>
    <label class="tog"><input type="checkbox" id="prev" checked> Move servo while adjusting (live preview)</label>
    <div id="ang"></div>
    <div class="hint">Use &minus;/+ for 1&deg; steps. Changes save automatically.</div>
  </details>
</div>

<!-- Speed -->
<div class="card">
  <details>
    <summary>&#9201; Speed &amp; Timing</summary>
    <div id="spd"></div>
  </details>
</div>

<!-- Manual raw control -->
<div class="card">
  <details>
    <summary>&#128295; Manual Raw Control</summary>
    <div class="hint" style="margin:7px 0">Move servos to any raw angle for calibration. No sweep — instant move.</div>
    <div id="man"></div>
  </details>
</div>

<!-- Reset -->
<div class="card">
  <button class="big" style="background:var(--red);font-size:13px"
    onclick="if(confirm('Reset ALL angles and speeds to factory defaults?'))api('/arm/reset','Resetting...')">
    &#9888; Reset to Factory Defaults
  </button>
</div>

<div id="toast"></div>

<script>
const $=id=>document.getElementById(id);
const DEG='\u00b0';
const UNITS={};
let chain=Promise.resolve(),pending=0,tt;

function toast(m){const t=$('toast');t.textContent=m;t.classList.add('show');clearTimeout(tt);tt=setTimeout(()=>t.classList.remove('show'),2500);}

function api(path,msg){chain=chain.then(()=>run(path,msg));return chain;}
async function run(path,msg){
  pending++;document.body.classList.add('busy');$('state').textContent=msg||'Working...';
  try{
    const r=await fetch(path,{cache:'no-store',signal:AbortSignal.timeout(10000)});
    if(!r.ok)throw new Error('HTTP '+r.status);
    render(await r.json());
  }catch(e){toast('No response from robot arm');}
  pending--;
  if(!pending){document.body.classList.remove('busy');$('state').textContent='Ready';}
}

// ---- Action slider helpers --------------------------------------------------
function showAct(id){
  const r=$('r_'+id), o=$('o_'+id);
  if(r&&o) o.textContent=r.value+DEG;
}
function adj(id,delta){
  const r=$('r_'+id);
  if(!r)return;
  r.value=Math.min(+r.max,Math.max(+r.min,+r.value+delta));
  showAct(id);
  // trigger onchange equivalent
  const ev=new Event('change'); r.dispatchEvent(ev);
}
// Save the action slider value as the new lup/ldown config
function sendLiftAngle(cfgKey,sliderId){
  const v=$('r_'+sliderId).value;
  api('/arm/cfg?k='+cfgKey+'&v='+v+'&p=0','Saving '+cfgKey+'='+v+'...');
  // Also update the config panel slider to stay in sync
  const cr=$('r_'+cfgKey);if(cr){cr.value=v;$('o_'+cfgKey).textContent=v+DEG;}
}

// UP button: use the action slider value
function armMoveUp(){
  const angle=$('r_lup_act').value;
  api('/arm/liftup?a='+angle, 'Moving UP to '+angle+String.fromCharCode(176)+'...');
}
// DOWN button: use the action slider value (lift only, no gripper change)
function armMoveDown(){
  const angle=$('r_ldown_act').value;
  api('/arm/liftdown?a='+angle, 'Moving DOWN to '+angle+String.fromCharCode(176)+'...');
}

// ---- Generic slider row (for config panel) ----------------------------------
function row(box,id,label,unit,min,max,step,hint,cb){
  UNITS[id]=unit;
  const d=document.createElement('div');d.className='row';
  d.innerHTML='<label>'+label+'<output id="o_'+id+'">-</output></label>'+
    '<div class="ctl" style="display:flex;align-items:center;gap:6px">'+
    '<button class="mini">&minus;</button>'+
    '<input type="range" id="r_'+id+'" min="'+min+'" max="'+max+'" step="'+step+'" value="'+min+'" style="flex:1;accent-color:var(--blue);height:26px;cursor:pointer">'+
    '<button class="mini">+</button></div>'+(hint?'<div class="hint">'+hint+'</div>':'');
  box.appendChild(d);
  const r=d.querySelector('input'),o=d.querySelector('output'),btns=d.querySelectorAll('button');
  const show=()=>{o.textContent=r.value+unit;};
  r.oninput=show;
  r.onchange=()=>cb(r.value);
  btns[0].onclick=()=>{r.value=Math.max(min,+r.value-step);show();cb(r.value);};
  btns[1].onclick=()=>{r.value=Math.min(max,+r.value+step);show();cb(r.value);};
}
function send(k,v,isAngle){
  const preview=isAngle&&$('prev').checked;
  api('/arm/cfg?k='+k+'&v='+v+'&p='+(preview?1:0),preview?'Moving...':'Saving...');
}

// Angle rows (saved presets)
[['gopen','Gripper OPEN angle'],['gclose','Gripper CLOSE angle'],
 ['lup','Lift UP angle'],['ldown','Lift DOWN angle']]
  .forEach(([k,l])=>row($('ang'),k,l,DEG,0,180,1,'',v=>send(k,v,true)));

// Speed rows
row($('spd'),'gs','Gripper delay per degree',' ms',1,50,1,'Lower = faster',v=>send('gs',v,false));
row($('spd'),'ls','Lift delay per degree',' ms',1,50,1,'Lower = faster',v=>send('ls',v,false));
row($('spd'),'pause','Pause between steps',' ms',0,2000,50,'Wait between sub-steps in PICK/PLACE',v=>send('pause',v,false));

// Manual control rows
row($('man'),'g','Gripper angle (raw)',DEG,0,180,1,'',v=>api('/arm/raw?g='+v,'Moving...'));
row($('man'),'l','Lift angle (raw)',DEG,20,160,1,'',v=>api('/arm/raw?l='+v,'Moving...'));

// ---- Render from server JSON ------------------------------------------------
function setUI(k,v){
  const r=$('r_'+k); if(!r||document.activeElement===r)return;
  r.value=v; $('o_'+k).textContent=v+(UNITS[k]||DEG);
}
function render(s){
  if(!s)return;
  // Sync action sliders with current saved angles (only if not being dragged)
  ['r_lup_act','r_ldown_act'].forEach((id,i)=>{
    const e=$(id); if(!e||document.activeElement===e)return;
    const val=i===0?s.lup:s.ldown;
    e.value=val; showAct(id==='r_lup_act'?'lup_act':'ldown_act');
  });
  // Update lift/grip hard limits
  ['r_lup','r_ldown','r_l','r_lup_act','r_ldown_act'].forEach(id=>{
    const e=$(id);if(e){e.min=s.lmin||20;e.max=s.lmax||160;}
  });
  // Config panel sliders
  ['gopen','gclose','lup','ldown','gs','ls','pause'].forEach(k=>setUI(k,s[k]));
  setUI('g',s.g); setUI('l',s.l);
  // Readout bar
  $('gv').textContent=s.g+DEG+(s.g==s.gopen?' (open)':s.g==s.gclose?' (closed)':'');
  $('lv').textContent=s.l+DEG+(s.l==s.lup?' (up)':s.l==s.ldown?' (down)':'');
  $('sv').textContent=s.open?'Open':'Closed';
  // SVG animation
  const cl=(x,a,b)=>Math.min(b,Math.max(a,x));
  const tg=cl((s.g-s.gclose)/((s.gopen-s.gclose)||1),0,1);
  const tl=cl((s.l-s.lup)/((s.ldown-s.lup)||1),0,1);
  $('arm').style.transform='rotate('+(-28+56*tl)+'deg)';
  const gap=24*tg;
  $('f1').style.transform='translateY('+(-gap/2)+'px)';
  $('f2').style.transform='translateY('+(gap/2)+'px)';
  $('yawHint').textContent='Lift: '+s.l+DEG;
}

api('/arm/status','Connecting...');
setInterval(()=>{if(!pending)api('/arm/status');},3500);
</script>
</body>
</html>
)rawliteral";