#pragma once
#include <Arduino.h>

// =============================================================================
// ROBOT CONTROL PAGE  (served at  http://192.168.4.1/ )
// Same UI as v3.0; only change: "Colour Sensor" button in the header -> /color
// =============================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>RUNNER4 · Pose Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#0d1117;color:#e6edf3;min-height:100vh;overflow-x:hidden}
.hdr{background:linear-gradient(135deg,#161b22,#21262d);border-bottom:1px solid #30363d;padding:12px 20px;display:flex;align-items:center;justify-content:space-between;position:sticky;top:0;z-index:99}
.brand h1{font-size:1.2rem;font-weight:800;letter-spacing:3px;background:linear-gradient(135deg,#58a6ff,#a5d6ff);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.brand p{font-size:.65rem;color:#8b949e;margin-top:2px;letter-spacing:.5px}
.hdr-r{display:flex;gap:8px;align-items:center}
.sdot{width:8px;height:8px;border-radius:50%;background:#f85149;display:inline-block;margin-right:5px;transition:background .4s}
.sdot.on{background:#3fb950;animation:blink 2s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.35}}
.stxt{font-size:.72rem;color:#8b949e}
.btn{padding:7px 14px;border:none;border-radius:8px;cursor:pointer;font-size:.78rem;font-weight:600;transition:all .18s;text-decoration:none;display:inline-block}
.btn-all{background:linear-gradient(135deg,#1f6feb,#58a6ff);color:#fff}
.btn-all:hover{transform:translateY(-1px);box-shadow:0 4px 14px rgba(88,166,255,.35)}
.btn-sw{background:rgba(255,255,255,.06);color:#c9d1d9;border:1px solid #30363d}
.btn-sw:hover{background:rgba(255,255,255,.11)}
.btn-col{background:linear-gradient(135deg,#7c3aed,#ec4899);color:#fff}
.layout{display:flex;align-items:flex-start}
.sidebar{width:210px;flex-shrink:0;background:#161b22;border-right:1px solid #30363d;padding:12px 10px 20px;display:flex;flex-direction:column;gap:5px;position:sticky;top:57px;height:calc(100vh - 57px);overflow-y:auto}
.sidebar::-webkit-scrollbar{width:4px}
.sidebar::-webkit-scrollbar-track{background:transparent}
.sidebar::-webkit-scrollbar-thumb{background:#30363d;border-radius:4px}
.sb-label{font-size:.58rem;font-weight:700;letter-spacing:1.8px;color:#484f58;text-transform:uppercase;padding:6px 5px 3px;margin-top:4px}
.pose-btn{width:100%;padding:9px 11px;border:1px solid #30363d;border-radius:8px;background:rgba(255,255,255,.03);color:#c9d1d9;cursor:pointer;font-size:.76rem;font-weight:600;text-align:left;transition:all .2s;display:flex;align-items:center;gap:9px;position:relative;overflow:hidden}
.pose-btn::before{content:'';position:absolute;inset:0;background:linear-gradient(135deg,rgba(88,166,255,.0),rgba(88,166,255,.0));transition:all .2s;border-radius:8px}
.pose-btn:hover{border-color:#58a6ff55;color:#a5d6ff;transform:translateX(2px)}
.pose-btn:hover::before{background:linear-gradient(135deg,rgba(88,166,255,.08),rgba(88,166,255,.03))}
.pose-btn.active{background:rgba(88,166,255,.14);border-color:#58a6ff;color:#58a6ff;transform:translateX(2px)}
.pose-btn.active::before{background:linear-gradient(135deg,rgba(88,166,255,.1),rgba(88,166,255,.05))}
.pose-icon{font-size:1rem;flex-shrink:0;width:20px;text-align:center}
.pose-name{flex:1}
.pose-badge{font-size:.55rem;background:rgba(255,255,255,.08);padding:2px 5px;border-radius:4px;color:#6e7681;margin-left:auto}
.pose-btn.active .pose-badge{background:rgba(88,166,255,.2);color:#58a6ff}
.pose-save{background:none;border:none;color:#8b949e;cursor:pointer;padding:2px 4px;font-size:.8rem;border-radius:4px;transition:all .2s;display:flex;align-items:center;justify-content:center}
.pose-save:hover{background:rgba(255,255,255,.1);color:#e6edf3;transform:scale(1.1)}
.pose-item{display:flex;align-items:center;gap:4px;width:100%}
.sb-div{border:none;border-top:1px solid #21262d;margin:8px 0}
.play-section{display:flex;flex-direction:column;gap:7px;padding:4px 2px}
.play-btn{width:100%;padding:10px;border:none;border-radius:9px;cursor:pointer;font-size:.82rem;font-weight:700;transition:all .2s;letter-spacing:.5px}
.play-btn.stopped{background:linear-gradient(135deg,#1a7f37,#3fb950);color:#fff}
.play-btn.stopped:hover{box-shadow:0 4px 14px rgba(63,185,80,.4);transform:translateY(-1px)}
.play-btn.playing{background:linear-gradient(135deg,#b91c1c,#f85149);color:#fff;animation:pulse-red 1.5s infinite}
@keyframes pulse-red{0%,100%{box-shadow:0 0 0 0 rgba(248,81,73,.4)}50%{box-shadow:0 0 0 6px rgba(248,81,73,.0)}}
.speed-row{display:flex;align-items:center;gap:7px;font-size:.63rem;color:#6e7681}
.speed-sl{flex:1;-webkit-appearance:none;appearance:none;height:3px;border-radius:3px;background:#21262d;outline:none;cursor:pointer}
.speed-sl::-webkit-slider-thumb{-webkit-appearance:none;width:13px;height:13px;border-radius:50%;background:#58a6ff;cursor:pointer;border:2px solid #0d1117}
.step-count{font-size:.62rem;color:#484f58;text-align:center;font-family:monospace;min-height:14px}
.play-phase{font-size:.65rem;text-align:center;color:#6e7681;min-height:14px}
.fk-mini{background:rgba(255,255,255,.025);border:1px solid #21262d;border-radius:7px;padding:8px 9px;margin-top:4px}
.fk-mini-title{font-size:.57rem;font-weight:700;text-transform:uppercase;letter-spacing:1px;color:#484f58;margin-bottom:6px}
.fk-row{display:flex;justify-content:space-between;align-items:center;margin-bottom:3px}
.fk-leg-dot{width:7px;height:7px;border-radius:50%;flex-shrink:0}
.fk-leg-name{font-size:.6rem;color:#6e7681;flex:1;margin-left:5px}
.fk-xyz{font-size:.58rem;font-family:monospace;color:#8b949e}
.content{flex:1;padding:14px 16px;min-width:0}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;max-width:900px;margin:0 auto}
@media(max-width:900px){.grid{grid-template-columns:1fr}}
@media(max-width:640px){.layout{flex-direction:column}.sidebar{width:100%;height:auto;position:static;flex-direction:row;flex-wrap:wrap;gap:6px}.sidebar .pose-btn{width:calc(50% - 3px)}.fk-mini{display:none}}
.card{background:#161b22;border:1px solid #30363d;border-radius:12px;overflow:hidden;transition:box-shadow .2s,border-color .3s}
.card:hover{box-shadow:0 5px 22px rgba(0,0,0,.45)}
.card.pose-anim{border-color:#58a6ff66;animation:card-flash .4s ease-out}
@keyframes card-flash{0%{box-shadow:0 0 0 2px rgba(88,166,255,.5)}100%{box-shadow:none}}
.ch{display:flex;align-items:center;gap:9px;padding:10px 14px;border-bottom:1px solid #21262d}
.badge{font-size:.88rem;font-weight:800;letter-spacing:1px;padding:3px 9px;border-radius:7px}
.cn{font-size:.8rem;color:#8b949e;flex:1}
.btn-lh{padding:4px 10px;border-radius:6px;border:none;cursor:pointer;font-size:.68rem;font-weight:600;transition:all .18s}
.cb{padding:12px 14px}
.jr{display:flex;align-items:center;gap:8px;margin-bottom:10px}
.jl{font-size:.66rem;color:#8b949e;width:86px;flex-shrink:0;font-weight:700;letter-spacing:.5px}
.sl{flex:1;-webkit-appearance:none;appearance:none;height:4px;border-radius:4px;background:#21262d;outline:none;cursor:pointer}
.sl::-webkit-slider-thumb{-webkit-appearance:none;width:15px;height:15px;border-radius:50%;cursor:pointer;border:2px solid #0d1117;box-shadow:0 1px 6px rgba(0,0,0,.5);transition:transform .1s}
.sl::-webkit-slider-thumb:hover{transform:scale(1.25)}
.av{font-size:.78rem;font-weight:700;font-family:monospace;min-width:36px;text-align:right;color:#e6edf3}
.btn-step{background:#21262d;border:none;border-radius:4px;color:#c9d1d9;width:22px;height:22px;cursor:pointer;font-size:.88rem;font-weight:700;line-height:20px;text-align:center;transition:background .2s;flex-shrink:0}
.btn-step:hover{background:#30363d}
.pnl{background:rgba(255,255,255,.025);border:1px solid #21262d;border-radius:8px;padding:9px 11px;margin-bottom:9px}
.pt{font-size:.6rem;font-weight:700;text-transform:uppercase;letter-spacing:1px;color:#6e7681;margin-bottom:5px}
.fkr{display:flex;gap:10px;flex-wrap:wrap}
.fkv{font-size:.74rem}.fkv span{color:#6e7681}.fkv b{font-family:monospace;color:#e6edf3}
.ikr{display:flex;gap:5px;margin-bottom:7px}
.iki{flex:1;background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:5px 7px;color:#e6edf3;font-size:.74rem;text-align:center;outline:none}
.iki:focus{border-color:#58a6ff}
.iki::placeholder{color:#3d444d}
.btn-ik{width:100%;padding:6px;border-radius:7px;border:1px solid #30363d;cursor:pointer;font-size:.74rem;font-weight:600;background:rgba(255,255,255,.05);color:#e6edf3;transition:all .18s}
.btn-ik:hover{background:rgba(255,255,255,.1)}
.ikst{font-size:.64rem;margin-top:4px;min-height:13px;text-align:center}
.ok{color:#3fb950}.er{color:#f85149}
.fl .badge{background:rgba(88,166,255,.14);color:#58a6ff}
.fl .btn-lh{background:rgba(88,166,255,.07);color:#58a6ff;border:1px solid rgba(88,166,255,.22)}
.fl .sl::-webkit-slider-thumb{background:#58a6ff}
.fr .badge{background:rgba(63,185,80,.14);color:#3fb950}
.fr .btn-lh{background:rgba(63,185,80,.07);color:#3fb950;border:1px solid rgba(63,185,80,.22)}
.fr .sl::-webkit-slider-thumb{background:#3fb950}
.bl .badge{background:rgba(167,139,250,.14);color:#a78bfa}
.bl .btn-lh{background:rgba(167,139,250,.07);color:#a78bfa;border:1px solid rgba(167,139,250,.22)}
.bl .sl::-webkit-slider-thumb{background:#a78bfa}
.br .badge{background:rgba(245,158,11,.14);color:#f59e0b}
.br .btn-lh{background:rgba(245,158,11,.07);color:#f59e0b;border:1px solid rgba(245,158,11,.22)}
.br .sl::-webkit-slider-thumb{background:#f59e0b}
.log{max-width:900px;margin:12px auto 20px;background:#161b22;border:1px solid #30363d;border-radius:10px;overflow:hidden}
.logh{padding:7px 14px;border-bottom:1px solid #21262d;font-size:.65rem;color:#6e7681;font-weight:700;text-transform:uppercase;letter-spacing:1px;display:flex;justify-content:space-between;align-items:center}
.logb{padding:8px 14px;font-family:monospace;font-size:.68rem;color:#6e7681;max-height:80px;overflow-y:auto}
.li{padding:1px 0}.li.inf{color:#58a6ff}.li.ok2{color:#3fb950}.li.er2{color:#f85149}.li.pose{color:#a78bfa}
.walk-variant{flex:1;padding:5px 4px;border:1px solid #30363d;border-radius:6px;background:rgba(255,255,255,.04);color:#6e7681;cursor:pointer;font-size:.68rem;font-weight:600;transition:all .18s}
.walk-variant.active{background:rgba(88,166,255,.15);border-color:#58a6ff;color:#58a6ff}
.walk-variant:hover:not(.active){background:rgba(255,255,255,.08);color:#c9d1d9}
.walk-steps-row{display:flex;align-items:center;gap:6px;margin-bottom:6px}
.walk-steps-in{width:52px;background:#0d1117;border:1px solid #30363d;border-radius:6px;color:#e6edf3;padding:4px 6px;font-size:.75rem;text-align:center;outline:none}
.walk-steps-in:focus{border-color:#58a6ff}
</style>
</head>
<body>

<header class="hdr">
  <div class="brand">
    <h1>RUNNER4</h1>
    <p>12-DOF Quadruped &nbsp;·&nbsp; Pose Presets + FK/IK &nbsp;·&nbsp; ESP32-S3 &nbsp;·&nbsp; v4.0</p>
  </div>
  <div class="hdr-r">
    <div><span class="sdot" id="sd"></span><span class="stxt" id="st">Connecting...</span></div>
    <a class="btn btn-col" href="/color">&#127912; Colour Sensor</a>
    <button class="btn btn-sw" onclick="doSweep()">&#9654; Sweep</button>
    <button class="btn btn-all" onclick="allHome()">&#8962; All HOME</button>
  </div>
</header>

<div class="layout">

<nav class="sidebar" id="sidebar">
  <div class="sb-label">Pose Presets</div>

  <div class="pose-item">
    <button class="pose-btn active" id="pb0" onclick="applyPose(0)">
      <span class="pose-icon">&#9881;&#65039;</span><span class="pose-name">Initial</span><span class="pose-badge">HOME</span>
    </button>
    <button class="pose-save" onclick="savePose(0)" title="Save current sliders to Initial">&#128190;</button>
  </div>

  <div class="pose-item">
    <button class="pose-btn" id="pb1" onclick="applyPose(1)">
      <span class="pose-icon">🦾</span><span class="pose-name">Standing</span><span class="pose-badge">STD</span>
    </button>
    <button class="pose-save" onclick="savePose(1)" title="Save current sliders to Standing">&#128190;</button>
  </div>

  <div class="pose-item">
    <button class="pose-btn" id="pb3" onclick="applyPose(3)">
      <span class="pose-icon">🔽</span><span class="pose-name">Low Stand</span>
    </button>
    <button class="pose-save" onclick="savePose(3)" title="Save current sliders to Low Stand">&#128190;</button>
  </div>

  <hr class="sb-div">
  <div class="sb-label">Transition Speed</div>
  <div class="play-section" style="gap:5px">
    <div class="speed-row">
      <span>Instant</span>
      <input type="range" class="speed-sl" id="transitionSl" min="0" max="3000" value="0" oninput="updTransDisplay()">
      <span>Slow</span>
    </div>
    <div style="font-size:.6rem;color:#484f58;text-align:center"><span id="transitionVal">Instant (0ms)</span></div>
  </div>

  <hr class="sb-div">
  <div class="sb-label">Walk Forward</div>
  <div class="play-section">
    <div style="display:flex;gap:4px;margin-bottom:4px">
      <button id="wvBtn0" class="walk-variant active" onclick="setWalkVariant(0)">Basic</button>
      <button id="wvBtn1" class="walk-variant" onclick="setWalkVariant(1)">Enhanced</button>
    </div>
    <div style="font-size:.55rem;color:#484f58;margin-bottom:6px;padding:0 2px" id="wvDesc">Tibia unchanged &mdash; safe starting mode</div>
    <div class="walk-steps-row">
      <span style="font-size:.63rem;color:#6e7681;flex:1">Steps</span>
      <input type="number" id="walkStepsIn" class="walk-steps-in" min="0" max="200" value="10">
      <span style="font-size:.6rem;color:#484f58">(0=&infin;)</span>
    </div>
    <button class="play-btn stopped" id="walkBtn" onclick="toggleWalk()">&#128694; Walk Forward</button>
    <div class="play-phase" id="walkPhase"></div>
    <div class="step-count" id="walkStepCount"></div>
    <div class="speed-row" style="margin-top:6px">
      <span>Fast</span>
      <input type="range" class="speed-sl" id="walkSpeedSl" min="20" max="3000" value="600" oninput="updWalkSpeed()">
      <span>Slow</span>
    </div>
    <div style="font-size:.6rem;color:#484f58;text-align:center"><span id="walkSpeedVal">600</span>ms / step</div>
  </div>

  <hr class="sb-div">
  <div class="sb-label">Turn Left</div>
  <div class="play-section">
    <div class="walk-steps-row">
      <span style="font-size:.63rem;color:#6e7681;flex:1">Steps</span>
      <input type="number" id="turnLStepsIn" class="walk-steps-in" min="1" max="200" value="4">
    </div>
    <button class="play-btn stopped" id="turnLBtn" onclick="toggleTurnLeft()">&#8634; Turn Left</button>
    <div class="play-phase" id="turnLPhase"></div>
    <div class="step-count" id="turnLStepCount"></div>
    <div class="speed-row" style="margin-top:6px">
      <span>Fast</span>
      <input type="range" class="speed-sl" id="turnLSpeedSl" min="20" max="3000" value="500" oninput="updTurnLSpeed()">
      <span>Slow</span>
    </div>
    <div style="font-size:.6rem;color:#484f58;text-align:center"><span id="turnLSpeedVal">500</span>ms / step</div>
  </div>

  <hr class="sb-div">
  <div class="sb-label">Turn Right</div>
  <div class="play-section">
    <div class="walk-steps-row">
      <span style="font-size:.63rem;color:#6e7681;flex:1">Steps</span>
      <input type="number" id="turnRStepsIn" class="walk-steps-in" min="1" max="200" value="4">
    </div>
    <button class="play-btn stopped" id="turnRBtn" onclick="toggleTurnRight()">&#8635; Turn Right</button>
    <div class="play-phase" id="turnRPhase"></div>
    <div class="step-count" id="turnRStepCount"></div>
    <div class="speed-row" style="margin-top:6px">
      <span>Fast</span>
      <input type="range" class="speed-sl" id="turnRSpeedSl" min="20" max="3000" value="500" oninput="updTurnRSpeed()">
      <span>Slow</span>
    </div>
    <div style="font-size:.6rem;color:#484f58;text-align:center"><span id="turnRSpeedVal">500</span>ms / step</div>
  </div>

  <hr class="sb-div">
  <div class="sb-label">Foot Positions</div>
  <div class="fk-mini">
    <div class="fk-mini-title">Live FK (mm)</div>
    <div class="fk-row"><div class="fk-leg-dot" style="background:#58a6ff"></div><span class="fk-leg-name">FL</span><span class="fk-xyz" id="mfk0">—</span></div>
    <div class="fk-row"><div class="fk-leg-dot" style="background:#3fb950"></div><span class="fk-leg-name">FR</span><span class="fk-xyz" id="mfk1">—</span></div>
    <div class="fk-row"><div class="fk-leg-dot" style="background:#a78bfa"></div><span class="fk-leg-name">BL</span><span class="fk-xyz" id="mfk2">—</span></div>
    <div class="fk-row"><div class="fk-leg-dot" style="background:#f59e0b"></div><span class="fk-leg-name">BR</span><span class="fk-xyz" id="mfk3">—</span></div>
  </div>
</nav>

<main class="content">
<div class="grid">

<div class="card fl" id="card0">
<div class="ch"><span class="badge">FL</span><span class="cn">Front-Left</span><button class="btn-lh" onclick="legHome(0)">HOME</button></div>
<div class="cb">
<div class="jr"><span class="jl">HIP &middot; CH0</span><button class="btn-step" onclick="step(0,-1)">-</button><input type="range" min="0" max="180" value="45"  class="sl" id="s0"  oninput="mv(0,this.value)"><button class="btn-step" onclick="step(0,1)">+</button><span class="av" id="v0">45&deg;</span></div>
<div class="jr"><span class="jl">FEMUR &middot; CH1</span><button class="btn-step" onclick="step(1,-1)">-</button><input type="range" min="0" max="180" value="80"  class="sl" id="s1"  oninput="mv(1,this.value)"><button class="btn-step" onclick="step(1,1)">+</button><span class="av" id="v1">80&deg;</span></div>
<div class="jr"><span class="jl">TIBIA &middot; CH2</span><button class="btn-step" onclick="step(2,-1)">-</button><input type="range" min="0" max="180" value="160" class="sl" id="s2"  oninput="mv(2,this.value)"><button class="btn-step" onclick="step(2,1)">+</button><span class="av" id="v2">160&deg;</span></div>
<div class="pnl"><div class="pt">Forward Kinematics — Foot Position</div><div class="fkr"><div class="fkv"><span>X: </span><b id="fx0">—</b> mm</div><div class="fkv"><span>Y: </span><b id="fy0">—</b> mm</div><div class="fkv"><span>Z: </span><b id="fz0">—</b> mm</div></div></div>
<div class="pnl"><div class="pt">Inverse Kinematics — Target (mm)</div>
<div class="ikr"><input class="iki" type="number" id="ix0" placeholder="X" value="50"><input class="iki" type="number" id="iy0" placeholder="Y" value="20"><input class="iki" type="number" id="iz0" placeholder="Z" value="-100"></div>
<button class="btn-ik" onclick="applyIK(0)">Apply Inverse Kinematics &#8594;</button><div class="ikst" id="is0"></div></div>
</div></div>

<div class="card fr" id="card1">
<div class="ch"><span class="badge">FR</span><span class="cn">Front-Right</span><button class="btn-lh" onclick="legHome(1)">HOME</button></div>
<div class="cb">
<div class="jr"><span class="jl">HIP &middot; CH3</span><button class="btn-step" onclick="step(3,-1)">-</button><input type="range" min="0" max="180" value="94"  class="sl" id="s3"  oninput="mv(3,this.value)"><button class="btn-step" onclick="step(3,1)">+</button><span class="av" id="v3">94&deg;</span></div>
<div class="jr"><span class="jl">FEMUR &middot; CH4</span><button class="btn-step" onclick="step(4,-1)">-</button><input type="range" min="0" max="180" value="80"  class="sl" id="s4"  oninput="mv(4,this.value)"><button class="btn-step" onclick="step(4,1)">+</button><span class="av" id="v4">80&deg;</span></div>
<div class="jr"><span class="jl">TIBIA &middot; CH5</span><button class="btn-step" onclick="step(5,-1)">-</button><input type="range" min="0" max="180" value="0"   class="sl" id="s5"  oninput="mv(5,this.value)"><button class="btn-step" onclick="step(5,1)">+</button><span class="av" id="v5">0&deg;</span></div>
<div class="pnl"><div class="pt">Forward Kinematics — Foot Position</div><div class="fkr"><div class="fkv"><span>X: </span><b id="fx1">—</b> mm</div><div class="fkv"><span>Y: </span><b id="fy1">—</b> mm</div><div class="fkv"><span>Z: </span><b id="fz1">—</b> mm</div></div></div>
<div class="pnl"><div class="pt">Inverse Kinematics — Target (mm)</div>
<div class="ikr"><input class="iki" type="number" id="ix1" placeholder="X" value="50"><input class="iki" type="number" id="iy1" placeholder="Y" value="-20"><input class="iki" type="number" id="iz1" placeholder="Z" value="-100"></div>
<button class="btn-ik" onclick="applyIK(1)">Apply Inverse Kinematics &#8594;</button><div class="ikst" id="is1"></div></div>
</div></div>

<div class="card bl" id="card2">
<div class="ch"><span class="badge">BL</span><span class="cn">Back-Left</span><button class="btn-lh" onclick="legHome(2)">HOME</button></div>
<div class="cb">
<div class="jr"><span class="jl">HIP &middot; CH10</span><button class="btn-step" onclick="step(10,-1)">-</button><input type="range" min="0" max="180" value="94"  class="sl" id="s10" oninput="mv(10,this.value)"><button class="btn-step" onclick="step(10,1)">+</button><span class="av" id="v10">94&deg;</span></div>
<div class="jr"><span class="jl">FEMUR &middot; CH11</span><button class="btn-step" onclick="step(11,-1)">-</button><input type="range" min="0" max="180" value="111"  class="sl" id="s11" oninput="mv(11,this.value)"><button class="btn-step" onclick="step(11,1)">+</button><span class="av" id="v11">111&deg;</span></div>
<div class="jr"><span class="jl">TIBIA &middot; CH14</span><button class="btn-step" onclick="step(14,-1)">-</button><input type="range" min="0" max="180" value="55" class="sl" id="s14" oninput="mv(14,this.value)"><button class="btn-step" onclick="step(14,1)">+</button><span class="av" id="v14">55&deg;</span></div>
<div class="pnl"><div class="pt">Forward Kinematics — Foot Position</div><div class="fkr"><div class="fkv"><span>X: </span><b id="fx2">—</b> mm</div><div class="fkv"><span>Y: </span><b id="fy2">—</b> mm</div><div class="fkv"><span>Z: </span><b id="fz2">—</b> mm</div></div></div>
<div class="pnl"><div class="pt">Inverse Kinematics — Target (mm)</div>
<div class="ikr"><input class="iki" type="number" id="ix2" placeholder="X" value="50"><input class="iki" type="number" id="iy2" placeholder="Y" value="20"><input class="iki" type="number" id="iz2" placeholder="Z" value="-100"></div>
<button class="btn-ik" onclick="applyIK(2)">Apply Inverse Kinematics &#8594;</button><div class="ikst" id="is2"></div></div>
</div></div>

<div class="card br" id="card3">
<div class="ch"><span class="badge">BR</span><span class="cn">Back-Right</span><button class="btn-lh" onclick="legHome(3)">HOME</button></div>
<div class="cb">
<div class="jr"><span class="jl">HIP &middot; CH6</span><button class="btn-step" onclick="step(6,-1)">-</button><input type="range" min="0" max="180" value="45"  class="sl" id="s6"  oninput="mv(6,this.value)"><button class="btn-step" onclick="step(6,1)">+</button><span class="av" id="v6">45&deg;</span></div>
<div class="jr"><span class="jl">FEMUR &middot; CH8</span><button class="btn-step" onclick="step(8,-1)">-</button><input type="range" min="0" max="180" value="92"  class="sl" id="s8"  oninput="mv(8,this.value)"><button class="btn-step" onclick="step(8,1)">+</button><span class="av" id="v8">92&deg;</span></div>
<div class="jr"><span class="jl">TIBIA &middot; CH9</span><button class="btn-step" onclick="step(9,-1)">-</button><input type="range" min="0" max="180" value="160" class="sl" id="s9"  oninput="mv(9,this.value)"><button class="btn-step" onclick="step(9,1)">+</button><span class="av" id="v9">160&deg;</span></div>
<div class="pnl"><div class="pt">Forward Kinematics — Foot Position</div><div class="fkr"><div class="fkv"><span>X: </span><b id="fx3">—</b> mm</div><div class="fkv"><span>Y: </span><b id="fy3">—</b> mm</div><div class="fkv"><span>Z: </span><b id="fz3">—</b> mm</div></div></div>
<div class="pnl"><div class="pt">Inverse Kinematics — Target (mm)</div>
<div class="ikr"><input class="iki" type="number" id="ix3" placeholder="X" value="50"><input class="iki" type="number" id="iy3" placeholder="Y" value="-20"><input class="iki" type="number" id="iz3" placeholder="Z" value="-100"></div>
<button class="btn-ik" onclick="applyIK(3)">Apply Inverse Kinematics &#8594;</button><div class="ikst" id="is3"></div></div>
</div></div>

</div>

<div class="log">
  <div class="logh"><span>Event Log</span><button onclick="document.getElementById('lb').innerHTML=''" style="background:none;border:none;color:#6e7681;cursor:pointer;font-size:.65rem">Clear</button></div>
  <div class="logb" id="lb"><div class="li inf">Initialising RUNNER4 v4.0 — Pose Presets + FK/IK + Colour module...</div></div>
</div>
</main>
</div>

<script>
const L1=30, L2=60, L3=80;
const D2R=Math.PI/180, R2D=180/Math.PI;
const CH=[[0,1,2],[3,4,5],[10,11,14],[6,8,9]];
const MIR=[1,-1,1,-1];

const POSES=[
  { name:'Initial',    icon:'⚙️', angles:[[45,80,160],[94,80,0],[94,56,10],[45,92,160]] },
  { name:'Standing',   icon:'🦾', angles:[[45,25,115],[94,135,45],[94,111,55],[45,37,115]] },
  { name:'Low Crouch', icon:'⬇️', angles:[[45,110,140],[94,110,40],[94,110,140],[45,110,140]] },
  { name:'Low Stand',  icon:'🔽', angles:[[45,50,130],[94,110,30],[94,86,40],[45,62,130]] },
  { name:'FL Leg Up',  icon:'↖️', angles:[[45,38,115],[94,125,35],[94,101,45],[45,80,160]] },
  { name:'FR Leg Up',  icon:'↗️', angles:[[45,80,160],[94,55,20],[94,56,160],[45,92,160]] },
  { name:'Trot A',     icon:'🔄', angles:[[45,60,120],[94,80,0],[94,56,160],[45,72,140]] },
  { name:'Trot B',     icon:'🔃', angles:[[45,80,160],[94,55,20],[94,36,130],[45,92,160]] },
  { name:'Stretch',    icon:'↔️', angles:[[45,90,90],[94,90,90],[94,90,90],[45,90,90]] }
];

let A=new Array(16).fill(0);
let dbt={};
let activePose=0;
let pollTimer=null;
let pollBusy=false;

function lg(m,t){
  const b=document.getElementById('lb');
  const d=document.createElement('div');
  d.className='li '+(t||'inf');
  d.textContent=new Date().toLocaleTimeString()+' \u00bb '+m;
  b.appendChild(d); b.scrollTop=b.scrollHeight;
}

function cst(ok){
  document.getElementById('sd').className='sdot'+(ok?' on':'');
  document.getElementById('st').textContent=ok?'Connected \u00b7 192.168.4.1':'Disconnected';
}

function fetchWT(url, ms) {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), ms);
  return fetch(url, { signal: ctrl.signal })
    .then(r => { clearTimeout(timer); return r; })
    .catch(e => { clearTimeout(timer); throw e; });
}

function pausePoll(){ pollBusy=true; }
function resumePoll(){ pollBusy=false; }

function mv(ch,val){
  val=parseInt(val);
  document.getElementById('v'+ch).textContent=val+'\u00b0';
  A[ch]=val;
  let leg=-1;
  for(let i=0;i<4;i++){if(CH[i].includes(ch)){leg=i;break;}}
  if(leg>=0) updFK(leg);
  clearTimeout(dbt[ch]);
  dbt[ch]=setTimeout(()=>send(ch,val),75);
}

function step(ch,d){
  const el=document.getElementById('s'+ch);
  let v=parseInt(el.value)+d;
  v=Math.max(0,Math.min(180,v));
  el.value=v; mv(ch,v);
}

async function send(ch,a){
  try{
    const r=await fetchWT('/set?ch='+ch+'&angle='+a, 3000);
    if(!r.ok) throw new Error('bad status');
    cst(true);
    lg('CH'+String(ch).padStart(2,'0')+' \u2192 '+a+'\u00b0','ok2');
  }catch(e){cst(false);lg('TX fail CH'+ch+' (robot busy?)','er2');}
}

function updTransDisplay(){
  const ms=parseInt(document.getElementById('transitionSl').value);
  document.getElementById('transitionVal').textContent = ms===0 ? 'Instant (0ms)' : ms+'ms';
}
function getPoseSpeed(){ return parseInt(document.getElementById('transitionSl').value); }

async function applyPose(id){
  activePose=id;
  for(let i=0;i<POSES.length;i++){
    const el=document.getElementById('pb'+i);
    if(el) el.classList.toggle('active',i===id);
  }
  for(let c=0;c<4;c++){
    const card=document.getElementById('card'+c);
    card.classList.remove('pose-anim');
    void card.offsetWidth;
    card.classList.add('pose-anim');
  }
  pausePoll();
  let success=false;
  const speedMs = getPoseSpeed();
  const timeoutMs = 5000 + speedMs;
  for(let attempt=0; attempt<2; attempt++){
    try{
      const r=await fetchWT('/pose?id='+id+'&speed='+speedMs, timeoutMs);
      if(!r.ok) throw new Error('bad status');
      const d=await r.json();
      cst(true);
      for(let i=0;i<16;i++){
        A[i]=d.a[i];
        const el=document.getElementById('s'+i);
        if(el){ el.value=d.a[i]; document.getElementById('v'+i).textContent=d.a[i]+'\u00b0'; }
      }
      updAllFK();
      lg('\u2605 Pose \u2192 '+POSES[id].name+(speedMs>0?' ('+speedMs+'ms)':''),'pose');
      success=true;
      break;
    }catch(e){
      if(attempt===0){
        lg('Pose attempt 1 failed, retrying...','inf');
        await new Promise(res=>setTimeout(res,400));
      }
    }
  }
  if(!success){
    lg('\u26a0 Cannot reach ESP32 or robot busy — UI updated only, motors unchanged','er2');
    const ang=POSES[id].angles;
    for(let leg=0;leg<4;leg++){
      for(let j=0;j<3;j++){
        const ch=CH[leg][j];
        A[ch]=ang[leg][j];
        const el=document.getElementById('s'+ch);
        if(el){el.value=ang[leg][j];document.getElementById('v'+ch).textContent=ang[leg][j]+'\u00b0';}
      }
    }
    updAllFK();
    cst(false);
  }
  resumePoll();
}

async function savePose(id){
  const newAng = [];
  let urlArgs = `?id=${id}`;
  for(let leg=0; leg<4; leg++){
    const legAng = [];
    for(let j=0; j<3; j++){
      const ch = CH[leg][j];
      const val = A[ch];
      legAng.push(val);
      urlArgs += `&a${leg}${j}=${val}`;
    }
    newAng.push(legAng);
  }
  const cppCode = `{ {${newAng[0].join(',')}}, {${newAng[1].join(',')}}, {${newAng[2].join(',')}}, {${newAng[3].join(',')}} }`;
  POSES[id].angles = newAng;
  pausePoll();
  try {
    const r = await fetchWT('/savepose' + urlArgs, 4000);
    if(!r.ok) throw new Error('bad status');
    lg(`Saved ${POSES[id].name} successfully in RAM!`,'ok2');
    lg('COPY THIS TO C++ CODE: ','inf');
    lg(cppCode, 'pose');
  } catch(e) {
    lg('Failed to save pose to ESP32','er2');
  }
  resumePoll();
}

let walkRunning=false, walkVariant=0, walkPollTimer=null;

function setWalkVariant(v){
  walkVariant=v;
  document.getElementById('wvBtn0').classList.toggle('active',v===0);
  document.getElementById('wvBtn1').classList.toggle('active',v===1);
  document.getElementById('wvDesc').textContent=v===0
    ?'Tibia unchanged \u2014 safe starting mode'
    :'Tibia retracts \u2014 better ground clearance';
}
function updWalkSpeed(){
  document.getElementById('walkSpeedVal').textContent=document.getElementById('walkSpeedSl').value;
}

async function toggleWalk(){
  if(walkRunning){
    try{await fetchWT('/walk?stop=1',3000);}catch(e){}
    stopWalkUI('Stopped.');
    return;
  }
  const steps  =parseInt(document.getElementById('walkStepsIn').value)||0;
  const speed  =parseInt(document.getElementById('walkSpeedSl').value);
  const variant=walkVariant;
  const btn=document.getElementById('walkBtn');
  btn.className='play-btn playing';
  btn.innerHTML='\u23f9 Stop Walk';
  walkRunning=true;
  document.getElementById('walkPhase').textContent='Starting\u2026';
  document.getElementById('walkStepCount').textContent='';
  try{
    const r=await fetchWT('/walk?steps='+steps+'&speed='+speed+'&variant='+variant, 8000);
    const d=await r.json();
    if(!d.ok) throw new Error(d.msg||'rejected');
    document.getElementById('walkPhase').textContent=steps===0?'Continuous walk\u2026':'Phase: FL+BR up';
    document.getElementById('walkStepCount').textContent=steps===0?'Steps: 0 (\u221e)':'Steps: 0 / '+steps;
    walkPollTimer=setInterval(pollWalkStatus,400);
    lg('Walk '+(variant===0?'Basic':'Enhanced')+' started \u2014 '+(steps===0?'continuous':steps+' steps')+' @ '+speed+'ms/step','inf');
  }catch(e){
    walkRunning=false;
    btn.className='play-btn stopped';
    btn.innerHTML='\ud83d\udeb6 Walk Forward';
    lg('Walk command failed \u2014 '+e.message,'er2');
  }
}

async function pollWalkStatus(){
  try{
    const r=await fetchWT('/walkstatus',2000);
    const d=await r.json();
    const phase=(d.steps%2===0)?'FL+BR up':'FR+BL up';
    document.getElementById('walkPhase').textContent='Phase: '+phase;
    document.getElementById('walkStepCount').textContent='Steps: '+d.steps+(d.target>0?' / '+d.target:' (\u221e)');
    if(!d.running) stopWalkUI('Walk complete \u2713');
  }catch(e){}
}

function stopWalkUI(msg){
  walkRunning=false;
  clearInterval(walkPollTimer);walkPollTimer=null;
  const btn=document.getElementById('walkBtn');
  btn.className='play-btn stopped';
  btn.innerHTML='\ud83d\udeb6 Walk Forward';
  document.getElementById('walkPhase').textContent=msg||'';
  document.getElementById('walkStepCount').textContent='';
  lg('Walk Forward \u2014 '+(msg||'stopped'),'ok2');
}

let turnLRunning=false, turnLPollTimer=null;
function updTurnLSpeed(){ document.getElementById('turnLSpeedVal').textContent=document.getElementById('turnLSpeedSl').value; }

async function toggleTurnLeft(){
  if(turnLRunning){
    try{await fetchWT('/turnleft?stop=1',3000);}catch(e){}
    stopTurnLUI('Stopped.');
    return;
  }
  const steps=parseInt(document.getElementById('turnLStepsIn').value)||4;
  const speed=parseInt(document.getElementById('turnLSpeedSl').value);
  const btn=document.getElementById('turnLBtn');
  btn.className='play-btn playing';
  btn.innerHTML='\u23f9 Stop Turn';
  turnLRunning=true;
  document.getElementById('turnLPhase').textContent='Starting\u2026';
  document.getElementById('turnLStepCount').textContent='';
  try{
    const r=await fetchWT('/turnleft?steps='+steps+'&speed='+speed, 8000);
    const d=await r.json();
    if(!d.ok) throw new Error(d.msg||'rejected');
    document.getElementById('turnLPhase').textContent='Turning Left\u2026';
    document.getElementById('turnLStepCount').textContent='Steps: 0 / '+steps;
    turnLPollTimer=setInterval(pollTurnLStatus,400);
    lg('Turn Left started \u2014 '+steps+' steps @ '+speed+'ms/step','inf');
  }catch(e){
    turnLRunning=false;
    btn.className='play-btn stopped';
    btn.innerHTML='\u21ba Turn Left';
    lg('Turn Left command failed \u2014 '+e.message,'er2');
  }
}

async function pollTurnLStatus(){
  try{
    const r=await fetchWT('/turnstatus?dir=left',2000);
    const d=await r.json();
    document.getElementById('turnLPhase').textContent='Phase: '+d.phase;
    document.getElementById('turnLStepCount').textContent='Steps: '+d.steps+' / '+d.target;
    if(!d.running) stopTurnLUI('Turn Left complete \u2713');
  }catch(e){}
}

function stopTurnLUI(msg){
  turnLRunning=false;
  clearInterval(turnLPollTimer);turnLPollTimer=null;
  const btn=document.getElementById('turnLBtn');
  btn.className='play-btn stopped';
  btn.innerHTML='\u21ba Turn Left';
  document.getElementById('turnLPhase').textContent=msg||'';
  document.getElementById('turnLStepCount').textContent='';
  lg('Turn Left \u2014 '+(msg||'stopped'),'ok2');
}

let turnRRunning=false, turnRPollTimer=null;
function updTurnRSpeed(){ document.getElementById('turnRSpeedVal').textContent=document.getElementById('turnRSpeedSl').value; }

async function toggleTurnRight(){
  if(turnRRunning){
    try{await fetchWT('/turnright?stop=1',3000);}catch(e){}
    stopTurnRUI('Stopped.');
    return;
  }
  const steps=parseInt(document.getElementById('turnRStepsIn').value)||4;
  const speed=parseInt(document.getElementById('turnRSpeedSl').value);
  const btn=document.getElementById('turnRBtn');
  btn.className='play-btn playing';
  btn.innerHTML='\u23f9 Stop Turn';
  turnRRunning=true;
  document.getElementById('turnRPhase').textContent='Starting\u2026';
  document.getElementById('turnRStepCount').textContent='';
  try{
    const r=await fetchWT('/turnright?steps='+steps+'&speed='+speed, 8000);
    const d=await r.json();
    if(!d.ok) throw new Error(d.msg||'rejected');
    document.getElementById('turnRPhase').textContent='Turning Right\u2026';
    document.getElementById('turnRStepCount').textContent='Steps: 0 / '+steps;
    turnRPollTimer=setInterval(pollTurnRStatus,400);
    lg('Turn Right started \u2014 '+steps+' steps @ '+speed+'ms/step','inf');
  }catch(e){
    turnRRunning=false;
    btn.className='play-btn stopped';
    btn.innerHTML='\u21bb Turn Right';
    lg('Turn Right command failed \u2014 '+e.message,'er2');
  }
}

async function pollTurnRStatus(){
  try{
    const r=await fetchWT('/turnstatus?dir=right',2000);
    const d=await r.json();
    document.getElementById('turnRPhase').textContent='Phase: '+d.phase;
    document.getElementById('turnRStepCount').textContent='Steps: '+d.steps+' / '+d.target;
    if(!d.running) stopTurnRUI('Turn Right complete \u2713');
  }catch(e){}
}

function stopTurnRUI(msg){
  turnRRunning=false;
  clearInterval(turnRPollTimer);turnRPollTimer=null;
  const btn=document.getElementById('turnRBtn');
  btn.className='play-btn stopped';
  btn.innerHTML='\u21bb Turn Right';
  document.getElementById('turnRPhase').textContent=msg||'';
  document.getElementById('turnRStepCount').textContent='';
  lg('Turn Right \u2014 '+(msg||'stopped'),'ok2');
}

function fk(h,f,t,mir){
  const a1=(h-90)*D2R, a2=(f-90)*D2R, a3=(t-90)*D2R;
  const reach=L1+L2*Math.cos(a2)+L3*Math.cos(a2+a3);
  const z=L2*Math.sin(a2)+L3*Math.sin(a2+a3);
  return{ x:(reach*Math.cos(a1)).toFixed(1), y:(reach*Math.sin(a1)*mir).toFixed(1), z:z.toFixed(1) };
}
function updFK(l){
  const c=CH[l];
  const p=fk(A[c[0]],A[c[1]],A[c[2]],MIR[l]);
  document.getElementById('fx'+l).textContent=p.x;
  document.getElementById('fy'+l).textContent=p.y;
  document.getElementById('fz'+l).textContent=p.z;
  document.getElementById('mfk'+l).textContent=p.x+', '+p.y+', '+p.z;
}
function updAllFK(){for(let i=0;i<4;i++)updFK(i);}

function ik(px,py,pz,mir){
  py*=mir;
  const a1=Math.atan2(py,px);
  const rT=Math.sqrt(px*px+py*py);
  const r=rT-L1;
  const d=Math.sqrt(r*r+pz*pz);
  if(d>L2+L3-0.5||d<Math.abs(L2-L3)+0.5) return null;
  let c3=(d*d-L2*L2-L3*L3)/(2*L2*L3);
  c3=Math.max(-1,Math.min(1,c3));
  const a3=Math.acos(c3);
  const a2=Math.atan2(pz,r)-Math.atan2(L3*Math.sin(a3),L2+L3*Math.cos(a3));
  const hip=Math.round(a1*R2D+90);
  const fem=Math.round(a2*R2D+90);
  const tib=Math.round(a3*R2D+90);
  if(hip<0||hip>180||fem<0||fem>180||tib<0||tib>180) return null;
  return{hip,fem,tib};
}

async function applyIK(l){
  const x=+document.getElementById('ix'+l).value;
  const y=+document.getElementById('iy'+l).value;
  const z=+document.getElementById('iz'+l).value;
  const st=document.getElementById('is'+l);
  const res=ik(x,y,z,MIR[l]);
  if(!res){
    st.className='ikst er';
    st.textContent='\u2717 Unreachable (max='+(L1+L2+L3)+'mm)';
    lg('IK L'+l+' ('+x+','+y+','+z+') unreachable','er2');
    return;
  }
  const c=CH[l];
  const ang=[res.hip,res.fem,res.tib];
  pausePoll();
  for(let i=0;i<3;i++){
    document.getElementById('s'+c[i]).value=ang[i];
    document.getElementById('v'+c[i]).textContent=ang[i]+'\u00b0';
    A[c[i]]=ang[i];
    await send(c[i],ang[i]);
  }
  resumePoll();
  updFK(l);
  st.className='ikst ok';
  st.textContent='\u2713 H='+res.hip+'\u00b0 F='+res.fem+'\u00b0 T='+res.tib+'\u00b0';
  lg('IK '+['FL','FR','BL','BR'][l]+' \u2192 H:'+res.hip+' F:'+res.fem+' T:'+res.tib,'ok2');
}

async function legHome(l){
  pausePoll();
  try{
    const r=await fetchWT('/leghome?leg='+l, 4000);
    if(!r.ok) throw new Error('busy');
    const d=await r.json();
    const c=CH[l];
    for(let i=0;i<3;i++){
      A[c[i]]=d.a[i];
      document.getElementById('s'+c[i]).value=d.a[i];
      document.getElementById('v'+c[i]).textContent=d.a[i]+'\u00b0';
    }
    updFK(l);
    cst(true);
    lg('Leg '+['FL','FR','BL','BR'][l]+' \u2192 HOME','ok2');
  }catch(e){cst(false);lg('legHome failed — check WiFi / robot busy','er2');}
  resumePoll();
}

async function allHome(){
  pausePoll();
  try{
    const r=await fetchWT('/home', 6000);
    if(!r.ok) throw new Error('busy');
    const d=await r.json();
    for(let i=0;i<16;i++){
      A[i]=d.a[i];
      const el=document.getElementById('s'+i);
      if(el){el.value=d.a[i];document.getElementById('v'+i).textContent=d.a[i]+'\u00b0';}
    }
    updAllFK();
    for(let i=0;i<POSES.length;i++){const el=document.getElementById('pb'+i);if(el)el.classList.toggle('active',i===0);}
    activePose=0;
    cst(true);
    lg('All 12 servos \u2192 HOME (Initial)','ok2');
  }catch(e){cst(false);lg('allHome failed — check WiFi / robot busy','er2');}
  resumePoll();
}

async function doSweep(){
  lg('Sweep test started on BR_FEMUR (CH8)...','inf');
  try{await fetchWT('/sweep', 3000);lg('Sweep running...','ok2');}
  catch(e){lg('Sweep trigger failed','er2');}
}

async function poll(){
  if(pollBusy) return;
  try{
    const r=await fetchWT('/state', 2000);
    const d=await r.json();
    cst(true);
    let changed=false;
    for(let i=0;i<16;i++){
      if(A[i]!==d.a[i]){
        A[i]=d.a[i];
        const el=document.getElementById('s'+i);
        if(el){el.value=d.a[i];document.getElementById('v'+i).textContent=d.a[i]+'\u00b0';}
        changed=true;
      }
    }
    if(changed) updAllFK();
  }catch(e){cst(false);}
}

window.onload=function(){
  updAllFK();
  poll();
  pollTimer=setInterval(poll, 3000);
  lg('RUNNER4 v4.0 — L1='+L1+' L2='+L2+' L3='+L3+' mm  |  Pose Presets + Walk + Turn + Colour (/color)','inf');
  lg('Tip: if motors not moving, check WiFi connection (status top-right)','inf');
};
</script>
</body>
</html>
)rawliteral";
