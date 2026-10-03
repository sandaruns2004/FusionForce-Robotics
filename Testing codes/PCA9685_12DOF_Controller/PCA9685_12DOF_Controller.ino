// =============================================================================
// FusionForce RUNNER4 — 12 DOF Quadruped Robot
// PCA9685 Web Servo Control + Forward & Inverse Kinematics Engine
// BOARD:   ESP32-S3 (DevKitC-1 or XIAO ESP32-S3)
// VERSION: 2.0 — WiFi Web Interface + FK/IK
// =============================================================================
//
// HOW TO USE:
//   1. Flash this sketch to ESP32-S3 (USB CDC On Boot = Enabled)
//   2. Open Serial Monitor (115200 baud) — wait for "URL: http://192.168.4.1"
//   3. On any phone/laptop/tablet connect to WiFi network:
//        SSID     : RUNNER4-Config
//        Password : runner4robot
//   4. Open any browser and navigate to:
//        http://192.168.4.1
//   5. Use the 12 sliders to control each servo in real time.
//      FK (foot position in mm) updates live on every slider move.
//      IK: enter a target foot position (X, Y, Z mm) → click Apply.
//
// REQUIRED LIBRARIES (Arduino Library Manager):
//   1. "Adafruit PWM Servo Driver Library" by Adafruit
//   2. Wire        — built-in, ESP32 Arduino core
//   3. WiFi        — built-in, ESP32 Arduino core
//   4. WebServer   — built-in, ESP32 Arduino core
//
// ARDUINO IDE BOARD SETTINGS:
//   Board            : ESP32S3 Dev Module
//   USB CDC On Boot  : Enabled   ← CRITICAL for Serial output
//   Upload Speed     : 921600
//   Flash Mode       : QIO 80MHz
//   PSRAM            : Disabled
//
// WIRING: ESP32-S3 → PCA9685 (I2C logic)
//   ESP32-S3 3.3V  →  PCA9685 VCC
//   ESP32-S3 GND   →  PCA9685 GND
//   GPIO 8 (SDA)   →  PCA9685 SDA
//   GPIO 9 (SCL)   →  PCA9685 SCL
//
// SERVO POWER (NEVER from ESP32-S3 3.3V):
//   6V BEC (+)  →  PCA9685 V+  screw terminal
//   6V BEC (-)  →  PCA9685 GND screw terminal
//
// CHANNEL MAP:
//   CH 0:FL_HIP  CH 1:FL_FEMUR  CH 2:FL_TIBIA
//   CH 3:FR_HIP  CH 4:FR_FEMUR  CH 5:FR_TIBIA
//   CH 6:BL_HIP  CH 7:BL_FEMUR  CH 8:BL_TIBIA
//   CH 9:BR_HIP  CH10:BR_FEMUR  CH11:BR_TIBIA
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WebServer.h>
#include <math.h>

// ─────────────────────────────────────────────────────────────────────────────
// WiFi — Access Point mode (no external router required)
// ─────────────────────────────────────────────────────────────────────────────
const char* AP_SSID = "RUNNER4-Config";   // WiFi network name
const char* AP_PASS = "runner4robot";     // WiFi password (min 8 chars)
// After connecting → open http://192.168.4.1 in any browser

// ─────────────────────────────────────────────────────────────────────────────
// ESP32-S3 I2C Pins
// Change I2C_SDA/I2C_SCL if using XIAO ESP32-S3 (use 5 and 6 instead)
// ─────────────────────────────────────────────────────────────────────────────
#define I2C_SDA  8
#define I2C_SCL  9

// ─────────────────────────────────────────────────────────────────────────────
// PCA9685 Servo Driver
// ─────────────────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN    150    // Pulse tick at   0 deg (~732 us)
#define SERVOMAX    600    // Pulse tick at 180 deg (~2930 us)
#define SERVO_FREQ   50    // 50 Hz standard servo update rate

// ─────────────────────────────────────────────────────────────────────────────
// Robot Leg Geometry — Link Lengths (mm)
// These match the RUNNER4 confirmed BOM (spider-robot-roadmap.md)
// ─────────────────────────────────────────────────────────────────────────────
#define L1_MM  30.0f    // Coxa  (hip offset from body to femur pivot)
#define L2_MM  60.0f    // Femur (upper leg link)
#define L3_MM  80.0f    // Tibia (lower leg link)
// Maximum reach: L1+L2+L3 = 170 mm
// Minimum reach: L1+|L2-L3| = 50 mm

// ─────────────────────────────────────────────────────────────────────────────
// PCA9685 Channel Definitions
// ─────────────────────────────────────────────────────────────────────────────
#define FL_HIP    0    // Front-Left  Hip   (001)
#define FL_FEMUR  1    // Front-Left  Femur (002)
#define FL_TIBIA  2    // Front-Left  Tibia (003)
#define FR_HIP    3    // Front-Right Hip   (001)
#define FR_FEMUR  4    // Front-Right Femur (002)
#define FR_TIBIA  5    // Front-Right Tibia (003)
#define BL_HIP    6    // Back-Left   Hip   (001)
#define BL_FEMUR  7    // Back-Left   Femur (002)
#define BL_TIBIA  8    // Back-Left   Tibia (003)
#define BR_HIP    9    // Back-Right  Hip   (001)
#define BR_FEMUR 10    // Back-Right  Femur (002)
#define BR_TIBIA 11    // Back-Right  Tibia (003)

// ─────────────────────────────────────────────────────────────────────────────
// Home / Standing Angles
// These are the initial angles when robot is in standing pose.
// NOTE: BR leg angles are all 0 — needs physical calibration.
// Use the web slider interface to find correct home angles, then update here.
// ─────────────────────────────────────────────────────────────────────────────
#define INIT_FL_HIP    45
#define INIT_FL_FEMUR 100
#define INIT_FL_TIBIA 180
#define INIT_FR_HIP   135
#define INIT_FR_FEMUR  80
#define INIT_FR_TIBIA   0
#define INIT_BL_HIP   135
#define INIT_BL_FEMUR  80
#define INIT_BL_TIBIA   0
#define INIT_BR_HIP     0    // TODO: calibrate using slider
#define INIT_BR_FEMUR   0    // TODO: calibrate using slider
#define INIT_BR_TIBIA   0    // TODO: calibrate using slider

// Leg 0=FL, 1=FR, 2=BL, 3=BR
// legChannels[leg][0=hip, 1=femur, 2=tibia]
const uint8_t legChannels[4][3] = {
  {FL_HIP, FL_FEMUR, FL_TIBIA},
  {FR_HIP, FR_FEMUR, FR_TIBIA},
  {BL_HIP, BL_FEMUR, BL_TIBIA},
  {BR_HIP, BR_FEMUR, BR_TIBIA}
};

const int homeAngles[4][3] = {
  {INIT_FL_HIP, INIT_FL_FEMUR, INIT_FL_TIBIA},
  {INIT_FR_HIP, INIT_FR_FEMUR, INIT_FR_TIBIA},
  {INIT_BL_HIP, INIT_BL_FEMUR, INIT_BL_TIBIA},
  {INIT_BR_HIP, INIT_BR_FEMUR, INIT_BR_TIBIA}
};

// Mirror: Left legs (+1 Y), Right legs (-1 Y)
// Right-side legs are physically mirrored so the Y axis flips sign in FK/IK.
const int legMirror[4] = {1, -1, 1, -1};

// Runtime servo angle state (updated on every setServo call)
int servoAngles[12] = {
  INIT_FL_HIP, INIT_FL_FEMUR, INIT_FL_TIBIA,
  INIT_FR_HIP, INIT_FR_FEMUR, INIT_FR_TIBIA,
  INIT_BL_HIP, INIT_BL_FEMUR, INIT_BL_TIBIA,
  INIT_BR_HIP, INIT_BR_FEMUR, INIT_BR_TIBIA
};

// ─────────────────────────────────────────────────────────────────────────────
// Web Server on port 80
// ─────────────────────────────────────────────────────────────────────────────
WebServer server(80);
volatile bool sweepRequested = false;

// =============================================================================
// EMBEDDED HTML PAGE
// The entire web interface lives here as a raw string literal.
// It is stored automatically in ESP32 flash (not RAM).
// =============================================================================
const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>RUNNER4 · Servo Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#0d1117;color:#e6edf3;min-height:100vh}
/* ── Header ── */
.hdr{background:linear-gradient(135deg,#161b22,#21262d);border-bottom:1px solid #30363d;padding:13px 20px;display:flex;align-items:center;justify-content:space-between;position:sticky;top:0;z-index:99}
.brand h1{font-size:1.25rem;font-weight:800;letter-spacing:3px;background:linear-gradient(135deg,#58a6ff,#a5d6ff);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.brand p{font-size:.68rem;color:#8b949e;margin-top:2px;letter-spacing:.5px}
.hdr-r{display:flex;gap:8px;align-items:center}
.sdot{width:8px;height:8px;border-radius:50%;background:#f85149;display:inline-block;margin-right:5px;transition:background .4s}
.sdot.on{background:#3fb950;animation:blink 2s infinite}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.35}}
.stxt{font-size:.72rem;color:#8b949e}
.btn{padding:7px 14px;border:none;border-radius:8px;cursor:pointer;font-size:.78rem;font-weight:600;transition:all .18s}
.btn-all{background:linear-gradient(135deg,#1f6feb,#58a6ff);color:#fff}
.btn-all:hover{transform:translateY(-1px);box-shadow:0 4px 14px rgba(88,166,255,.35)}
.btn-sw{background:rgba(255,255,255,.06);color:#c9d1d9;border:1px solid #30363d}
.btn-sw:hover{background:rgba(255,255,255,.11)}
/* ── Grid ── */
main{padding:15px 18px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:13px;max-width:1080px;margin:0 auto}
@media(max-width:700px){.grid{grid-template-columns:1fr}}
/* ── Card ── */
.card{background:#161b22;border:1px solid #30363d;border-radius:13px;overflow:hidden;transition:box-shadow .2s}
.card:hover{box-shadow:0 5px 22px rgba(0,0,0,.45)}
.ch{display:flex;align-items:center;gap:9px;padding:11px 15px;border-bottom:1px solid #21262d}
.badge{font-size:.9rem;font-weight:800;letter-spacing:1px;padding:3px 9px;border-radius:7px}
.cn{font-size:.82rem;color:#8b949e;flex:1}
.btn-lh{padding:4px 10px;border-radius:6px;border:none;cursor:pointer;font-size:.7rem;font-weight:600;transition:all .18s}
.cb{padding:13px 15px}
/* ── Slider Row ── */
.jr{display:flex;align-items:center;gap:8px;margin-bottom:11px}
.jl{font-size:.68rem;color:#8b949e;width:86px;flex-shrink:0;font-weight:700;letter-spacing:.5px}
.sl{flex:1;-webkit-appearance:none;appearance:none;height:4px;border-radius:4px;background:#21262d;outline:none;cursor:pointer}
.sl::-webkit-slider-thumb{-webkit-appearance:none;width:16px;height:16px;border-radius:50%;cursor:pointer;border:2px solid #0d1117;box-shadow:0 1px 6px rgba(0,0,0,.5);transition:transform .1s}
.sl::-webkit-slider-thumb:hover{transform:scale(1.25)}
.av{font-size:.8rem;font-weight:700;font-family:monospace;min-width:38px;text-align:right;color:#e6edf3}
/* ── Info Panels ── */
.pnl{background:rgba(255,255,255,.025);border:1px solid #21262d;border-radius:9px;padding:10px 12px;margin-bottom:10px}
.pt{font-size:.62rem;font-weight:700;text-transform:uppercase;letter-spacing:1px;color:#6e7681;margin-bottom:6px}
.fkr{display:flex;gap:12px;flex-wrap:wrap}
.fkv{font-size:.76rem}.fkv span{color:#6e7681}.fkv b{font-family:monospace;color:#e6edf3}
.ikr{display:flex;gap:6px;margin-bottom:8px}
.iki{flex:1;background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:5px 7px;color:#e6edf3;font-size:.76rem;text-align:center;outline:none}
.iki:focus{border-color:#58a6ff}
.iki::placeholder{color:#3d444d}
.btn-ik{width:100%;padding:7px;border-radius:7px;border:1px solid #30363d;cursor:pointer;font-size:.76rem;font-weight:600;background:rgba(255,255,255,.05);color:#e6edf3;transition:all .18s}
.btn-ik:hover{background:rgba(255,255,255,.1)}
.ikst{font-size:.66rem;margin-top:5px;min-height:13px;text-align:center}
.ok{color:#3fb950}.er{color:#f85149}
/* ── Leg Colors ── */
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
/* ── Event Log ── */
.log{max-width:1080px;margin:13px auto 20px;background:#161b22;border:1px solid #30363d;border-radius:11px;overflow:hidden}
.logh{padding:8px 15px;border-bottom:1px solid #21262d;font-size:.68rem;color:#6e7681;font-weight:700;text-transform:uppercase;letter-spacing:1px;display:flex;justify-content:space-between;align-items:center}
.logb{padding:9px 15px;font-family:monospace;font-size:.7rem;color:#6e7681;max-height:88px;overflow-y:auto}
.li{padding:1px 0}.li.inf{color:#58a6ff}.li.ok2{color:#3fb950}.li.er2{color:#f85149}
</style>
</head>
<body>
<header class="hdr">
  <div class="brand">
    <h1>RUNNER4</h1>
    <p>12-DOF Quadruped &nbsp;·&nbsp; Web Servo Control + FK/IK &nbsp;·&nbsp; ESP32-S3</p>
  </div>
  <div class="hdr-r">
    <div><span class="sdot" id="sd"></span><span class="stxt" id="st">Connecting...</span></div>
    <button class="btn btn-sw" onclick="doSweep()">&#9654; Sweep Test</button>
    <button class="btn btn-all" onclick="allHome()">&#8962; All HOME</button>
  </div>
</header>

<main>
<div class="grid">

<!-- ═══════ FL ═══════ -->
<div class="card fl">
<div class="ch"><span class="badge">FL</span><span class="cn">Front-Left</span><button class="btn-lh" onclick="legHome(0)">HOME</button></div>
<div class="cb">
<div class="jr"><span class="jl">HIP &middot; 001</span><input type="range" min="0" max="180" value="45"  class="sl" id="s0"  oninput="mv(0,this.value)"><span class="av" id="v0">45&deg;</span></div>
<div class="jr"><span class="jl">FEMUR &middot; 002</span><input type="range" min="0" max="180" value="100" class="sl" id="s1"  oninput="mv(1,this.value)"><span class="av" id="v1">100&deg;</span></div>
<div class="jr"><span class="jl">TIBIA &middot; 003</span><input type="range" min="0" max="180" value="180" class="sl" id="s2"  oninput="mv(2,this.value)"><span class="av" id="v2">180&deg;</span></div>
<div class="pnl"><div class="pt">Forward Kinematics — Foot Position</div><div class="fkr"><div class="fkv"><span>X: </span><b id="fx0">—</b> mm</div><div class="fkv"><span>Y: </span><b id="fy0">—</b> mm</div><div class="fkv"><span>Z: </span><b id="fz0">—</b> mm</div></div></div>
<div class="pnl"><div class="pt">Inverse Kinematics — Target Foot Position (mm)</div>
<div class="ikr"><input class="iki" type="number" id="ix0" placeholder="X" value="50"><input class="iki" type="number" id="iy0" placeholder="Y" value="20"><input class="iki" type="number" id="iz0" placeholder="Z" value="-100"></div>
<button class="btn-ik" onclick="applyIK(0)">Apply Inverse Kinematics &#8594;</button><div class="ikst" id="is0"></div></div>
</div></div>

<!-- ═══════ FR ═══════ -->
<div class="card fr">
<div class="ch"><span class="badge">FR</span><span class="cn">Front-Right</span><button class="btn-lh" onclick="legHome(1)">HOME</button></div>
<div class="cb">
<div class="jr"><span class="jl">HIP &middot; 001</span><input type="range" min="0" max="180" value="135" class="sl" id="s3"  oninput="mv(3,this.value)"><span class="av" id="v3">135&deg;</span></div>
<div class="jr"><span class="jl">FEMUR &middot; 002</span><input type="range" min="0" max="180" value="80"  class="sl" id="s4"  oninput="mv(4,this.value)"><span class="av" id="v4">80&deg;</span></div>
<div class="jr"><span class="jl">TIBIA &middot; 003</span><input type="range" min="0" max="180" value="0"   class="sl" id="s5"  oninput="mv(5,this.value)"><span class="av" id="v5">0&deg;</span></div>
<div class="pnl"><div class="pt">Forward Kinematics — Foot Position</div><div class="fkr"><div class="fkv"><span>X: </span><b id="fx1">—</b> mm</div><div class="fkv"><span>Y: </span><b id="fy1">—</b> mm</div><div class="fkv"><span>Z: </span><b id="fz1">—</b> mm</div></div></div>
<div class="pnl"><div class="pt">Inverse Kinematics — Target Foot Position (mm)</div>
<div class="ikr"><input class="iki" type="number" id="ix1" placeholder="X" value="50"><input class="iki" type="number" id="iy1" placeholder="Y" value="-20"><input class="iki" type="number" id="iz1" placeholder="Z" value="-100"></div>
<button class="btn-ik" onclick="applyIK(1)">Apply Inverse Kinematics &#8594;</button><div class="ikst" id="is1"></div></div>
</div></div>

<!-- ═══════ BL ═══════ -->
<div class="card bl">
<div class="ch"><span class="badge">BL</span><span class="cn">Back-Left</span><button class="btn-lh" onclick="legHome(2)">HOME</button></div>
<div class="cb">
<div class="jr"><span class="jl">HIP &middot; 001</span><input type="range" min="0" max="180" value="135" class="sl" id="s6"  oninput="mv(6,this.value)"><span class="av" id="v6">135&deg;</span></div>
<div class="jr"><span class="jl">FEMUR &middot; 002</span><input type="range" min="0" max="180" value="80"  class="sl" id="s7"  oninput="mv(7,this.value)"><span class="av" id="v7">80&deg;</span></div>
<div class="jr"><span class="jl">TIBIA &middot; 003</span><input type="range" min="0" max="180" value="0"   class="sl" id="s8"  oninput="mv(8,this.value)"><span class="av" id="v8">0&deg;</span></div>
<div class="pnl"><div class="pt">Forward Kinematics — Foot Position</div><div class="fkr"><div class="fkv"><span>X: </span><b id="fx2">—</b> mm</div><div class="fkv"><span>Y: </span><b id="fy2">—</b> mm</div><div class="fkv"><span>Z: </span><b id="fz2">—</b> mm</div></div></div>
<div class="pnl"><div class="pt">Inverse Kinematics — Target Foot Position (mm)</div>
<div class="ikr"><input class="iki" type="number" id="ix2" placeholder="X" value="50"><input class="iki" type="number" id="iy2" placeholder="Y" value="20"><input class="iki" type="number" id="iz2" placeholder="Z" value="-100"></div>
<button class="btn-ik" onclick="applyIK(2)">Apply Inverse Kinematics &#8594;</button><div class="ikst" id="is2"></div></div>
</div></div>

<!-- ═══════ BR ═══════ -->
<div class="card br">
<div class="ch"><span class="badge">BR</span><span class="cn">Back-Right</span><button class="btn-lh" onclick="legHome(3)">HOME</button></div>
<div class="cb">
<div class="jr"><span class="jl">HIP &middot; 001</span><input type="range" min="0" max="180" value="0"   class="sl" id="s9"  oninput="mv(9,this.value)"><span class="av" id="v9">0&deg;</span></div>
<div class="jr"><span class="jl">FEMUR &middot; 002</span><input type="range" min="0" max="180" value="0"   class="sl" id="s10" oninput="mv(10,this.value)"><span class="av" id="v10">0&deg;</span></div>
<div class="jr"><span class="jl">TIBIA &middot; 003</span><input type="range" min="0" max="180" value="0"   class="sl" id="s11" oninput="mv(11,this.value)"><span class="av" id="v11">0&deg;</span></div>
<div class="pnl"><div class="pt">Forward Kinematics — Foot Position</div><div class="fkr"><div class="fkv"><span>X: </span><b id="fx3">—</b> mm</div><div class="fkv"><span>Y: </span><b id="fy3">—</b> mm</div><div class="fkv"><span>Z: </span><b id="fz3">—</b> mm</div></div></div>
<div class="pnl"><div class="pt">Inverse Kinematics — Target Foot Position (mm)</div>
<div class="ikr"><input class="iki" type="number" id="ix3" placeholder="X" value="50"><input class="iki" type="number" id="iy3" placeholder="Y" value="-20"><input class="iki" type="number" id="iz3" placeholder="Z" value="-100"></div>
<button class="btn-ik" onclick="applyIK(3)">Apply Inverse Kinematics &#8594;</button><div class="ikst" id="is3"></div></div>
</div></div>

</div><!-- .grid -->

<div class="log">
  <div class="logh"><span>Event Log</span><button onclick="document.getElementById('lb').innerHTML=''" style="background:none;border:none;color:#6e7681;cursor:pointer;font-size:.68rem">Clear</button></div>
  <div class="logb" id="lb"><div class="li inf">Initialising RUNNER4 web interface...</div></div>
</div>
</main>

<script>
// ── Constants (must match C++ defines) ──
const L1=30, L2=60, L3=80;
const D2R=Math.PI/180, R2D=180/Math.PI;
// Leg channel map: CH[leg][hip, femur, tibia]
const CH=[[0,1,2],[3,4,5],[6,7,8],[9,10,11]];
// Mirror factor: +1=left leg, -1=right leg
const MIR=[1,-1,1,-1];
// Home angles per leg
const HOME=[[45,100,180],[135,80,0],[135,80,0],[0,0,0]];

// Runtime state
let A=[45,100,180,135,80,0,135,80,0,0,0,0];
let dbt={};

// ── Logging ──
function lg(m,t){
  const b=document.getElementById('lb');
  const d=document.createElement('div');
  d.className='li '+(t||'inf');
  d.textContent=new Date().toLocaleTimeString()+' \u00bb '+m;
  b.appendChild(d); b.scrollTop=b.scrollHeight;
}

// ── Connection status ──
function cst(ok){
  document.getElementById('sd').className='sdot'+(ok?' on':'');
  document.getElementById('st').textContent=ok?'Connected \u00b7 192.168.4.1':'Disconnected';
}

// ── Slider move handler ──
function mv(ch,val){
  val=parseInt(val);
  document.getElementById('v'+ch).textContent=val+'\u00b0';
  A[ch]=val;
  updFK(Math.floor(ch/3));
  clearTimeout(dbt[ch]);
  dbt[ch]=setTimeout(()=>send(ch,val),75);
}

// ── Send angle to ESP32 ──
async function send(ch,a){
  try{
    const r=await fetch('/set?ch='+ch+'&angle='+a,{signal:AbortSignal.timeout(2000)});
    if(!r.ok) throw 0;
    cst(true);
    lg('CH'+String(ch).padStart(2,'0')+' \u2192 '+a+'\u00b0','ok2');
  }catch(e){cst(false);lg('TX fail CH'+ch,'er2');}
}

// ── Forward Kinematics (must mirror C++ computeFK) ──
// Convention: servo 90 deg = 0 rad joint angle (neutral)
// a1 = hip yaw, a2 = femur pitch, a3 = tibia pitch
function fk(h,f,t,mir){
  const a1=(h-90)*D2R, a2=(f-90)*D2R, a3=(t-90)*D2R;
  const reach = L1 + L2*Math.cos(a2) + L3*Math.cos(a2+a3);
  const z     = L2*Math.sin(a2)      + L3*Math.sin(a2+a3);
  return {
    x:(reach*Math.cos(a1)).toFixed(1),
    y:(reach*Math.sin(a1)*mir).toFixed(1),
    z:z.toFixed(1)
  };
}
function updFK(l){
  const c=CH[l];
  const p=fk(A[c[0]],A[c[1]],A[c[2]],MIR[l]);
  document.getElementById('fx'+l).textContent=p.x;
  document.getElementById('fy'+l).textContent=p.y;
  document.getElementById('fz'+l).textContent=p.z;
}
function updAllFK(){for(let i=0;i<4;i++)updFK(i);}

// ── Inverse Kinematics (must mirror C++ computeIK) ──
function ik(px,py,pz,mir){
  py*=mir;                                    // un-mirror right legs
  const a1=Math.atan2(py,px);
  const rT=Math.sqrt(px*px+py*py);
  const r=rT-L1;
  const d=Math.sqrt(r*r+pz*pz);
  if(d>L2+L3-0.5 || d<Math.abs(L2-L3)+0.5) return null;
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
    st.textContent='\u2717 Unreachable (max='+(L1+L2+L3)+'mm, min='+Math.abs(L2-L3+L1)+'mm)';
    lg('IK L'+l+' ('+x+','+y+','+z+') unreachable','er2');
    return;
  }
  const c=CH[l];
  const ang=[res.hip,res.fem,res.tib];
  for(let i=0;i<3;i++){
    document.getElementById('s'+c[i]).value=ang[i];
    document.getElementById('v'+c[i]).textContent=ang[i]+'\u00b0';
    A[c[i]]=ang[i];
    await send(c[i],ang[i]);
  }
  updFK(l);
  st.className='ikst ok';
  st.textContent='\u2713 H='+res.hip+'\u00b0 F='+res.fem+'\u00b0 T='+res.tib+'\u00b0';
  lg('IK L'+['FL','FR','BL','BR'][l]+' \u2192 H:'+res.hip+' F:'+res.fem+' T:'+res.tib,'ok2');
}

// ── Leg HOME ──
async function legHome(l){
  try{
    const r=await fetch('/leghome?leg='+l,{signal:AbortSignal.timeout(3000)});
    const d=await r.json();
    const c=CH[l];
    for(let i=0;i<3;i++){
      A[c[i]]=d.a[i];
      document.getElementById('s'+c[i]).value=d.a[i];
      document.getElementById('v'+c[i]).textContent=d.a[i]+'\u00b0';
    }
    updFK(l);
    lg('Leg '+['FL','FR','BL','BR'][l]+' \u2192 HOME','ok2');
  }catch(e){lg('legHome failed','er2');}
}

// ── All HOME ──
async function allHome(){
  try{
    const r=await fetch('/home',{signal:AbortSignal.timeout(5000)});
    const d=await r.json();
    for(let i=0;i<12;i++){
      A[i]=d.a[i];
      document.getElementById('s'+i).value=d.a[i];
      document.getElementById('v'+i).textContent=d.a[i]+'\u00b0';
    }
    updAllFK();
    lg('All 12 servos \u2192 HOME','ok2');
  }catch(e){lg('allHome failed','er2');}
}

// ── Sweep Test ──
async function doSweep(){
  lg('Sweep test started on BR_FEMUR (CH10)...','inf');
  try{await fetch('/sweep',{signal:AbortSignal.timeout(2000)});lg('Sweep running...','ok2');}
  catch(e){lg('Sweep trigger failed','er2');}
}

// ── State Poll (sync ESP32 state back to browser) ──
async function poll(){
  try{
    const r=await fetch('/state',{signal:AbortSignal.timeout(1500)});
    const d=await r.json();
    cst(true);
    let changed=false;
    for(let i=0;i<12;i++){
      if(A[i]!==d.a[i]){
        A[i]=d.a[i];
        document.getElementById('s'+i).value=d.a[i];
        document.getElementById('v'+i).textContent=d.a[i]+'\u00b0';
        changed=true;
      }
    }
    if(changed) updAllFK();
  }catch(e){cst(false);}
}

// ── Init ──
window.onload=function(){
  updAllFK();
  poll();
  setInterval(poll,1800);
  lg('RUNNER4 Web Interface ready \u2014 L1=30 L2=60 L3=80 mm','inf');
};
</script>
</body>
</html>
)rawliteral";

// =============================================================================
// SERVO HELPERS
// =============================================================================

uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return (uint16_t)map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Sets one channel and updates state array
void setServo(uint8_t ch, int angle) {
  angle = constrain(angle, 0, 180);
  servoAngles[ch] = angle;
  pwm.setPWM(ch, 0, angleToPulse(angle));
  Serial.printf("  CH%02d -> %d deg (tick=%d)\n", ch, angle, angleToPulse(angle));
}

void setAllServosHome() {
  Serial.println("[HOME] All 12 servos -> HOME:");
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      setServo(legChannels[leg][joint], homeAngles[leg][joint]);
    }
  }
}

void setLegHome(int leg) {
  for (int joint = 0; joint < 3; joint++) {
    setServo(legChannels[leg][joint], homeAngles[leg][joint]);
  }
}

// =============================================================================
// FORWARD KINEMATICS (C++ implementation — mirrors JavaScript fk() exactly)
//
// Servo angle convention:
//   90 deg servo = 0 rad physical joint angle (neutral)
//   0 deg servo  = -90 deg physical angle
//   180 deg servo = +90 deg physical angle
//
// Reference frame (leg frame):
//   +X  = forward (direction leg points)
//   +Y  = lateral outward (left for left legs, right for right legs)
//   +Z  = up (away from ground)
//
// Link chain:
//   Hip servo rotates in XY plane (yaw)
//   Femur servo rotates in the leg's vertical plane (pitch)
//   Tibia servo continues the knee angle
// =============================================================================
struct Vec3f { float x, y, z; };

Vec3f computeFK(int hipDeg, int femurDeg, int tibiaDeg, int mirror) {
  const float a1 = (hipDeg   - 90) * DEG_TO_RAD;   // hip yaw
  const float a2 = (femurDeg - 90) * DEG_TO_RAD;   // femur pitch
  const float a3 = (tibiaDeg - 90) * DEG_TO_RAD;   // tibia pitch

  // Radial reach in the leg vertical plane from body origin
  const float reach = L1_MM + L2_MM * cosf(a2) + L3_MM * cosf(a2 + a3);
  // Vertical offset (positive = up)
  const float z     = L2_MM * sinf(a2) + L3_MM * sinf(a2 + a3);

  Vec3f pos;
  pos.x = reach * cosf(a1);
  pos.y = reach * sinf(a1) * (float)mirror;   // flip Y for right legs
  pos.z = z;
  return pos;
}

// =============================================================================
// INVERSE KINEMATICS (C++ — mirrors JavaScript ik() exactly)
//
// Given target foot position (px, py, pz) in leg frame (mm):
//   1. Hip angle  = atan2(py, px)
//   2. Distance d = sqrt((r-L1)^2 + pz^2)  where r = sqrt(px^2+py^2)
//   3. Tibia (a3) = acos((d^2 - L2^2 - L3^2) / (2*L2*L3))  [law of cosines]
//   4. Femur (a2) = atan2(pz, r-L1) - atan2(L3*sin(a3), L2+L3*cos(a3))
//
// Returns false if position is outside kinematic workspace.
// =============================================================================
bool computeIK(float px, float py, float pz, int mirror,
               int* hipOut, int* femurOut, int* tibiaOut) {
  py *= (float)mirror;   // un-mirror right legs before computing

  const float a1    = atan2f(py, px);
  const float rTotal = sqrtf(px*px + py*py);
  const float r     = rTotal - L1_MM;
  const float d     = sqrtf(r*r + pz*pz);

  // Workspace boundary check
  const float maxReach = L2_MM + L3_MM;
  const float minReach = fabsf(L2_MM - L3_MM);
  if (d > maxReach - 0.5f || d < minReach + 0.5f) return false;

  // Law of cosines for tibia angle
  float cosA3 = (d*d - L2_MM*L2_MM - L3_MM*L3_MM) / (2.0f * L2_MM * L3_MM);
  cosA3 = constrain(cosA3, -1.0f, 1.0f);
  const float a3 = acosf(cosA3);

  // Femur angle
  const float a2 = atan2f(pz, r) - atan2f(L3_MM * sinf(a3), L2_MM + L3_MM * cosf(a3));

  // Convert physical angles back to servo angles (add 90 deg offset)
  int hip   = (int)roundf(a1 * RAD_TO_DEG) + 90;
  int femur = (int)roundf(a2 * RAD_TO_DEG) + 90;
  int tibia = (int)roundf(a3 * RAD_TO_DEG) + 90;

  // Clamp and validate servo range
  if (hip < 0 || hip > 180 || femur < 0 || femur > 180 || tibia < 0 || tibia > 180) {
    return false;
  }

  *hipOut   = hip;
  *femurOut = femur;
  *tibiaOut = tibia;
  return true;
}

// =============================================================================
// I2C SCANNER — run at boot to confirm PCA9685 (0x40) is visible
// =============================================================================
void i2cScan() {
  Serial.println("-------------------------------------------------");
  Serial.println("I2C Bus Scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  [OK] Device found at 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  [!!] No I2C devices found!");
    Serial.println("       Check SDA/SCL wiring and 4.7k pull-ups.");
  }
  Serial.printf("  Total: %d device(s)\n", found);
  Serial.println("-------------------------------------------------");
}

// =============================================================================
// JSON HELPERS
// =============================================================================
String stateJSON() {
  String j = "{\"a\":[";
  for (int i = 0; i < 12; i++) {
    j += servoAngles[i];
    if (i < 11) j += ',';
  }
  j += "]}";
  return j;
}

String legJSON(int leg) {
  String j = "{\"a\":[";
  j += servoAngles[legChannels[leg][0]]; j += ',';
  j += servoAngles[legChannels[leg][1]]; j += ',';
  j += servoAngles[legChannels[leg][2]];
  j += "]}";
  return j;
}

void addCORS(WebServer& s) {
  s.sendHeader("Access-Control-Allow-Origin", "*");
}

// =============================================================================
// HTTP HANDLERS
// =============================================================================

// GET /  → serve the embedded HTML page
void hRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

// GET /set?ch=X&angle=Y  → move one servo
void hSet() {
  if (!server.hasArg("ch") || !server.hasArg("angle")) {
    server.send(400, "application/json", "{\"error\":\"missing ch or angle\"}");
    return;
  }
  int ch    = server.arg("ch").toInt();
  int angle = server.arg("angle").toInt();
  if (ch < 0 || ch > 11 || angle < 0 || angle > 180) {
    server.send(400, "application/json", "{\"error\":\"out of range\"}");
    return;
  }
  setServo((uint8_t)ch, angle);
  addCORS(server);
  String resp = "{\"ok\":true,\"ch\":";
  resp += ch; resp += ",\"angle\":"; resp += angle; resp += "}";
  server.send(200, "application/json", resp);
}

// GET /state  → return current 12-angle state array
void hState() {
  addCORS(server);
  server.send(200, "application/json", stateJSON());
}

// GET /home  → all 12 servos to HOME
void hHome() {
  setAllServosHome();
  addCORS(server);
  server.send(200, "application/json", stateJSON());
}

// GET /leghome?leg=X  → one leg to HOME
void hLegHome() {
  if (!server.hasArg("leg")) {
    server.send(400, "application/json", "{\"error\":\"missing leg\"}");
    return;
  }
  int leg = server.arg("leg").toInt();
  if (leg < 0 || leg > 3) {
    server.send(400, "application/json", "{\"error\":\"invalid leg (0-3)\"}");
    return;
  }
  setLegHome(leg);
  addCORS(server);
  server.send(200, "application/json", legJSON(leg));
  Serial.printf("[WEB] Leg %d -> HOME\n", leg);
}

// GET /ik?leg=X&x=A&y=B&z=C  → compute IK and apply
void hIK() {
  if (!server.hasArg("leg") || !server.hasArg("x") ||
      !server.hasArg("y")   || !server.hasArg("z")) {
    server.send(400, "application/json", "{\"error\":\"missing args\"}");
    return;
  }
  int   leg = server.arg("leg").toInt();
  float px  = server.arg("x").toFloat();
  float py  = server.arg("y").toFloat();
  float pz  = server.arg("z").toFloat();
  if (leg < 0 || leg > 3) {
    server.send(400, "application/json", "{\"error\":\"invalid leg\"}");
    return;
  }
  int hip, femur, tibia;
  bool ok = computeIK(px, py, pz, legMirror[leg], &hip, &femur, &tibia);
  if (!ok) {
    addCORS(server);
    server.send(200, "application/json", "{\"ok\":false,\"error\":\"unreachable\"}");
    Serial.printf("[IK] Leg %d unreachable (%.1f, %.1f, %.1f)\n", leg, px, py, pz);
    return;
  }
  setServo(legChannels[leg][0], hip);
  setServo(legChannels[leg][1], femur);
  setServo(legChannels[leg][2], tibia);
  addCORS(server);
  String resp = "{\"ok\":true,\"hip\":";
  resp += hip; resp += ",\"femur\":"; resp += femur;
  resp += ",\"tibia\":"; resp += tibia; resp += "}";
  server.send(200, "application/json", resp);
  Serial.printf("[IK] Leg %d -> H:%d F:%d T:%d\n", leg, hip, femur, tibia);
}

// GET /sweep  → trigger non-blocking sweep test on BR_FEMUR
void hSweep() {
  sweepRequested = true;
  addCORS(server);
  server.send(200, "application/json", "{\"ok\":true,\"msg\":\"sweep started\"}");
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  // Wait up to 3s for USB-CDC to enumerate (ESP32-S3 native USB)
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000) delay(10);

  Serial.println();
  Serial.println("=============================================");
  Serial.println("  FusionForce RUNNER4 — ESP32-S3");
  Serial.println("  Web Servo Control + FK/IK  v2.0");
  Serial.println("  L1=30mm  L2=60mm  L3=80mm");
  Serial.println("=============================================");
  Serial.printf("  I2C SDA -> GPIO%d\n", I2C_SDA);
  Serial.printf("  I2C SCL -> GPIO%d\n", I2C_SCL);

  // ── I2C + PCA9685 ─────────────────────────────────────────────────────────
  Wire.begin(I2C_SDA, I2C_SCL);
  i2cScan();

  Serial.println("Initialising PCA9685...");
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("Moving all servos -> HOME...");
  setAllServosHome();
  delay(1500);

  // ── WiFi Access Point ─────────────────────────────────────────────────────
  Serial.println();
  Serial.print("Starting WiFi AP: ");
  Serial.println(AP_SSID);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress apIP = WiFi.softAPIP();

  Serial.println("  WiFi AP started.");
  Serial.print("  SSID    : "); Serial.println(AP_SSID);
  Serial.print("  Password: "); Serial.println(AP_PASS);
  Serial.print("  URL     : http://"); Serial.println(apIP);

  // ── HTTP Routes ───────────────────────────────────────────────────────────
  server.on("/",        hRoot);
  server.on("/set",     hSet);
  server.on("/state",   hState);
  server.on("/home",    hHome);
  server.on("/leghome", hLegHome);
  server.on("/ik",      hIK);
  server.on("/sweep",   hSweep);
  server.begin();

  Serial.println("  HTTP server running on port 80.");
  Serial.println();
  Serial.println("=============================================");
  Serial.println("  1. Connect to WiFi: RUNNER4-Config");
  Serial.println("  2. Open browser: http://192.168.4.1");
  Serial.println("  3. Use sliders to control servos + FK/IK");
  Serial.println("=============================================");
  Serial.println();
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  server.handleClient();   // Process incoming HTTP requests

  // Non-blocking sweep test — triggered by /sweep endpoint
  // Uses server.handleClient() inside the sweep so the web UI stays responsive
  if (sweepRequested) {
    sweepRequested = false;
    Serial.println("[SWEEP] BR FEMUR (CH10): 0 -> 90 -> 0");

    for (int a = 0; a <= 90; a++) {
      setServo(BR_FEMUR, a);
      server.handleClient();   // keep web server alive during sweep
      delay(15);
    }
    delay(200);
    for (int a = 90; a >= 0; a--) {
      setServo(BR_FEMUR, a);
      server.handleClient();
      delay(15);
    }
    delay(200);
    setLegHome(3);   // return BR leg to home after sweep
    Serial.println("[SWEEP] Done.");
  }
}
// =============================================================================
//
// JOINT NAMING CONVENTION:
//   001 = Hip   (rotates leg forward / backward — mounted on the body frame)
//   002 = Femur (upper leg joint — lifts the leg up / down)
//   003 = Tibia (lower leg / knee joint — extends or retracts the foot)
//
// ─────────────────────────────────────────────────────────────────────────────
// ESP32-S3 → PCA9685 I2C WIRING:
//
//   ESP32-S3 Pin      │  PCA9685 Pin  │  Wire Colour  │  Notes
//   ──────────────────┼───────────────┼───────────────┼──────────────────────
//   3.3V (or 5V)      │  VCC          │  Red          │  Logic power only
//   GND               │  GND          │  Black        │  Common ground
//   GPIO 8 (SDA)      │  SDA          │  Blue/White   │  Default I2C SDA pin
//   GPIO 9 (SCL)      │  SCL          │  Yellow       │  Default I2C SCL pin
//
//   WHY GPIO 8 & 9?
//   The ESP32-S3 does NOT have fixed hardware I2C pins — any GPIO can be used.
//   GPIO 8 and 9 are the most commonly used defaults in community examples and
//   are exposed on most ESP32-S3 DevKit boards. They are NOT strapping pins.
//   If using XIAO ESP32-S3: SDA=GPIO5 (D4), SCL=GPIO6 (D5) — change below.
//
//   PULL-UP RESISTORS:
//   The ESP32-S3 has internal pull-ups enabled by Wire.begin().
//   For reliable I2C over longer cables or multiple devices on the same bus,
//   add external 4.7kΩ resistors from SDA and SCL to 3.3V.
//
// ─────────────────────────────────────────────────────────────────────────────
// STEP 2 — EXTERNAL POWER → PCA9685 V+ RAIL:
//   Power Supply +  →  PCA9685 V+  terminal  (big green screw block)
//   Power Supply –  →  PCA9685 GND terminal  (big green screw block)
//   Recommended: 6V DC, min 3A SMPS/BEC  (NEVER use ESP32-S3 3.3V for servos!)
//
//   WHY SEPARATE POWER?
//   A single MG90S servo can draw 500mA stall current. 12 servos could draw
//   up to 6A simultaneously. The ESP32-S3 3.3V LDO is rated for only ~500mA.
//   Using the wrong power source will BROWN OUT the ESP32-S3 instantly.
//
// STEP 3 — SERVO → PCA9685 CHANNEL (plug 3-pin servo connector per channel):
//   Servo Wire             │  PCA9685 Pin Column
//   ───────────────────────┼──────────────────────────────────────────────────
//   Signal (Orange/Yellow) │  PWM  — innermost pin column
//   VCC    (Red)           │  V+   — middle pin column (from external 6V rail)
//   GND    (Brown/Black)   │  GND  — outer pin column
//
// ─────────────────────────────────────────────────────────────────────────────
// CHANNEL-TO-SERVO WIRING TABLE:
// ─────────────────────────────────────────────────────────────────────────────
//  PCA9685   Leg    Joint Label   Code ID    Init Angle
//  Channel          (001/002/003)
//  ──────────────────────────────────────────────────────────────
//   CH  0    FL     001 = HIP     FL_HIP     45 deg
//   CH  1    FL     002 = FEMUR   FL_FEMUR  100 deg
//   CH  2    FL     003 = TIBIA   FL_TIBIA  180 deg
//  ──────────────────────────────────────────────────────────────
//   CH  3    FR     001 = HIP     FR_HIP    135 deg
//   CH  4    FR     002 = FEMUR   FR_FEMUR   80 deg
//   CH  5    FR     003 = TIBIA   FR_TIBIA    0 deg
//  ──────────────────────────────────────────────────────────────
//   CH  6    BL     001 = HIP     BL_HIP    135 deg
//   CH  7    BL     002 = FEMUR   BL_FEMUR   80 deg
//   CH  8    BL     003 = TIBIA   BL_TIBIA    0 deg
//  ──────────────────────────────────────────────────────────────
//   CH  9    BR     001 = HIP     BR_HIP      0 deg
//   CH 10    BR     002 = FEMUR   BR_FEMUR    0 deg
//   CH 11    BR     003 = TIBIA   BR_TIBIA    0 deg
//  ──────────────────────────────────────────────────────────────
//   CH 12-15  (SPARE — sensors, camera pan/tilt, arm, gripper, etc.)
//
// REQUIRED LIBRARIES (Arduino Library Manager / PlatformIO):
//   1. "Adafruit PWM Servo Driver Library" by Adafruit
//   2. Wire (built-in with ESP32 Arduino core — no install needed)
//
// BOARD SETUP (Arduino IDE):
//   Tools → Board → "ESP32S3 Dev Module" (or your specific variant)
//   Tools → Upload Speed → 921600
//   Tools → USB CDC On Boot → Enabled   ← IMPORTANT for Serial.println() output!
//   Tools → Flash Mode → QIO 80MHz
//   Tools → PSRAM → Disabled (unless you have PSRAM)
//
// =============================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ─────────────────────────────────────────────────────────────────────────────
// ESP32-S3 I2C PIN CONFIGURATION
// Change these if you use a different board variant or pinout.
// ─────────────────────────────────────────────────────────────────────────────
#define I2C_SDA  8   // GPIO 8 — SDA (Standard ESP32-S3 DevKit)
#define I2C_SCL  9   // GPIO 9 — SCL (Standard ESP32-S3 DevKit)

// WHY explicit pin definition?
// Unlike Arduino Uno (fixed A4/A5), the ESP32-S3 I2C controller is "flexible" —
// it can be mapped to ANY GPIO. Passing the pins to Wire.begin() makes the
// wiring intent clear and lets you move to other pins without hunting in code.

// ─────────────────────────────────────────────────────────────────────────────
// PCA9685 I2C ADDRESS
// Default address = 0x40 (all address jumpers open).
// If you have soldered the A0 jumper → 0x41
// If you have soldered the A1 jumper → 0x42
// Check your board and match the address below.
// ─────────────────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// ─────────────────────────────────────────────────────────────────────────────
// SERVO PULSE CALIBRATION
// These raw 12-bit tick values map to 0° and 180° on your servo.
// The PCA9685 at 50Hz uses 4096 ticks for 20ms (one full PWM period).
// 1 tick = 20ms / 4096 ≈ 4.88µs
//
// SERVOMIN = 150 ticks → 150 × 4.88µs ≈ 732µs  (some servos need ~600µs)
// SERVOMAX = 600 ticks → 600 × 4.88µs ≈ 2930µs (some servos need ~2400µs)
//
// Servo Tuning:
//   MG90S / SG90  → SERVOMIN=150, SERVOMAX=600  (defaults — try these first)
//   MG996R        → SERVOMIN=120, SERVOMAX=580
//   DS3225 / DS3218 → SERVOMIN=102, SERVOMAX=512
//
// TUNING PROCEDURE:
//   1. Set a servo to 0° and measure the physical angle. Adjust SERVOMIN.
//   2. Set to 180° and measure. Adjust SERVOMAX.
//   3. Verify 90° lands exactly at the midpoint.
// ─────────────────────────────────────────────────────────────────────────────
#define SERVOMIN   150   // 12-bit raw tick at  0 degrees (~732 µs)
#define SERVOMAX   600   // 12-bit raw tick at 180 degrees (~2930 µs)
#define SERVO_FREQ  50   // 50 Hz — standard update rate for analog servos

// WHY 50 Hz?
// Standard RC/analog servos expect a PWM pulse every 20ms (50Hz).
// Digital servos can accept 100-333Hz, but 50Hz is universally safe.
// The PCA9685 prescaler = round(25MHz / (4096 x 50)) - 1 = 121 (auto-computed
// by the Adafruit library from SERVO_FREQ).

// ─────────────────────────────────────────────────────────────────────────────
// CHANNEL DEFINITIONS
// Each leg has 3 joints:  001=HIP  002=FEMUR  003=TIBIA
// ─────────────────────────────────────────────────────────────────────────────

// Front-Left (FL)
#define FL_HIP      0   // FL 001 — Hip servo
#define FL_FEMUR    1   // FL 002 — Femur servo
#define FL_TIBIA    2   // FL 003 — Tibia servo

// Front-Right (FR)
#define FR_HIP      3   // FR 001 — Hip servo
#define FR_FEMUR    4   // FR 002 — Femur servo
#define FR_TIBIA    5   // FR 003 — Tibia servo

// Back-Left (BL)
#define BL_HIP      6   // BL 001 — Hip servo
#define BL_FEMUR    7   // BL 002 — Femur servo
#define BL_TIBIA    8   // BL 003 — Tibia servo

// Back-Right (BR)
#define BR_HIP      9   // BR 001 — Hip servo
#define BR_FEMUR   10   // BR 002 — Femur servo
#define BR_TIBIA   11   // BR 003 — Tibia servo

// ─────────────────────────────────────────────────────────────────────────────
// HOME / STANDING POSITION ANGLES
// These are the angles each joint goes to when the robot "stands".
// Angles are asymmetric across left/right because the servos mirror each other
// physically — a 45° "forward" hip on the left is 135° on the right.
// ─────────────────────────────────────────────────────────────────────────────

// Front-Left
#define INIT_FL_HIP    45    // FL 001 — Hip neutral
#define INIT_FL_FEMUR 100    // FL 002 — Femur slightly lifted
#define INIT_FL_TIBIA 180    // FL 003 — Tibia extended

// Front-Right  (mirror of FL — hip angle reversed)
#define INIT_FR_HIP   135    // FR 001
#define INIT_FR_FEMUR  80    // FR 002
#define INIT_FR_TIBIA   0    // FR 003

// Back-Left
#define INIT_BL_HIP   135    // BL 001
#define INIT_BL_FEMUR  80    // BL 002
#define INIT_BL_TIBIA   0    // BL 003

// Back-Right
#define INIT_BR_HIP     0    // BR 001 — NOTE: all zeros; this leg may need tuning
#define INIT_BR_FEMUR   0    // BR 002
#define INIT_BR_TIBIA   0    // BR 003

// ─────────────────────────────────────────────────────────────────────────────
// DEBUG / DIAGNOSTIC FLAGS
// Set to 1 to enable verbose Serial output during testing.
// Set to 0 to disable for production / final firmware.
// ─────────────────────────────────────────────────────────────────────────────
#define DEBUG_VERBOSE    1   // Prints angle + tick value on every setServo() call
#define I2C_SCAN_ON_BOOT 1   // Scans I2C bus at startup to confirm device found

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: i2cScan
// Scans all 127 I2C addresses and prints any found devices.
// Use this at startup to confirm PCA9685 is wired correctly and responding.
// Expected output:  "I2C device found at 0x40"
// If nothing prints, check wiring, pull-ups, and power to PCA9685 VCC.
// ─────────────────────────────────────────────────────────────────────────────
void i2cScan() {
  Serial.println("-------------------------------------------------");
  Serial.println("I2C Bus Scan:");
  int devicesFound = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  [OK] I2C device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      devicesFound++;
    }
  }
  if (devicesFound == 0) {
    Serial.println("  [!!] No I2C devices found!");
    Serial.println("       Check: SDA/SCL wires, pull-up resistors (4.7k),");
    Serial.println("       PCA9685 VCC power, and I2C_SDA/I2C_SCL pin defs.");
  }
  Serial.print("  Total devices found: ");
  Serial.println(devicesFound);
  Serial.println("-------------------------------------------------");
}

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: angleToPulse
// Converts degrees (0-180) to a PCA9685 12-bit pulse count.
//
// HOW IT WORKS:
//   The map() function linearly interpolates:
//     0°   → SERVOMIN (150 ticks)
//     180° → SERVOMAX (600 ticks)
//     90°  → (150+600)/2 = 375 ticks
//
//   constrain() clamps the angle to [0, 180] so no servo is ever commanded
//   beyond its hardware limits, which would strip gears or stall the motor.
// ─────────────────────────────────────────────────────────────────────────────
uint16_t angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return (uint16_t)map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: setServo(channel, angle)
// Sends a PWM command to a single PCA9685 channel.
//
// setPWM(channel, on, off):
//   'on'  = tick count when the pulse goes HIGH (always 0 here)
//   'off' = tick count when the pulse goes LOW  (= angleToPulse output)
//
// In other words: the HIGH pulse lasts from tick 0 to tick 'off',
// giving a pulse width of (off / 4096) x 20ms.
//
// Example: setServo(FL_HIP, 90)
//   → angleToPulse(90) = 375
//   → setPWM(0, 0, 375)
//   → pulse width = 375/4096 x 20ms ≈ 1.83ms (servo centers at ~1.5ms ideal)
// ─────────────────────────────────────────────────────────────────────────────
void setServo(uint8_t channel, int angle) {
  uint16_t pulse = angleToPulse(angle);

#if DEBUG_VERBOSE
  Serial.print("  CH");
  if (channel < 10) Serial.print("0");
  Serial.print(channel);
  Serial.print(" -> ");
  Serial.print(angle);
  Serial.print(" deg  (tick=");
  Serial.print(pulse);
  Serial.println(")");
#endif

  pwm.setPWM(channel, 0, pulse);
}

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: setAllServosHome
// Sends all 12 servos to their HOME / standing positions simultaneously.
// This is the starting posture for the robot before any gait begins.
//
// WHY call them all quickly without delay in-between?
// The PCA9685 stores the new target tick value in hardware registers the moment
// setPWM() is called over I2C. The servo physically moves to the target over
// ~200ms (MG90S no-load speed). Sending all 12 commands takes ~12ms over
// I2C at 100kHz, so all servos start moving almost simultaneously.
// ─────────────────────────────────────────────────────────────────────────────
void setAllServosHome() {
  Serial.println("Setting all 12 servos -> HOME positions:");

  Serial.println("  [FL] Front-Left:");
  setServo(FL_HIP,   INIT_FL_HIP);
  setServo(FL_FEMUR, INIT_FL_FEMUR);
  setServo(FL_TIBIA, INIT_FL_TIBIA);

  Serial.println("  [FR] Front-Right:");
  setServo(FR_HIP,   INIT_FR_HIP);
  setServo(FR_FEMUR, INIT_FR_FEMUR);
  setServo(FR_TIBIA, INIT_FR_TIBIA);

  Serial.println("  [BL] Back-Left:");
  setServo(BL_HIP,   INIT_BL_HIP);
  setServo(BL_FEMUR, INIT_BL_FEMUR);
  setServo(BL_TIBIA, INIT_BL_TIBIA);

  Serial.println("  [BR] Back-Right:");
  setServo(BR_HIP,   INIT_BR_HIP);
  setServo(BR_FEMUR, INIT_BR_FEMUR);
  setServo(BR_TIBIA, INIT_BR_TIBIA);

  Serial.println("  HOME complete.");
}

// ─────────────────────────────────────────────────────────────────────────────
// HELPER: sweepServo(channel, fromAngle, toAngle, stepDelayMs)
// Sweeps a single servo from one angle to another, one degree at a time.
// stepDelayMs controls the speed — higher = slower, smoother.
//
// WHY step-by-step instead of jumping directly?
// Large angle jumps cause current spikes that can brown out the servo rail,
// and cause mechanical stress on plastic gears. Stepping at 15ms per degree
// gives ~2.7 seconds for a 180° sweep — safe for testing.
// ─────────────────────────────────────────────────────────────────────────────
void sweepServo(uint8_t channel, int fromAngle, int toAngle, uint16_t stepDelayMs) {
  int step = (toAngle >= fromAngle) ? 1 : -1;
  for (int a = fromAngle; a != toAngle + step; a += step) {
    setServo(channel, a);
    delay(stepDelayMs);
  }
}

// =============================================================================
// SETUP
// Runs once after power-on or reset.
// =============================================================================
void setup() {
  // ── Serial Monitor ─────────────────────────────────────────────────────────
  // On ESP32-S3 with USB CDC enabled, Serial goes over the native USB port.
  // The 3000ms timeout waits for the USB host (your PC) to enumerate before
  // printing, so you don't miss the first messages in the Serial Monitor.
  // If you use a UART-to-USB adapter on TX0/RX0, this timeout is not needed.
  Serial.begin(115200);
  uint32_t t = millis();
  while (!Serial && (millis() - t) < 3000) {
    delay(10);  // Wait for USB CDC to connect (max 3s)
  }

  Serial.println();
  Serial.println("==============================================");
  Serial.println("  FusionForce RUNNER4 — ESP32-S3 Boot");
  Serial.println("  PCA9685 12-DOF Controller Test");
  Serial.println("==============================================");
  Serial.print("  I2C SDA -> GPIO"); Serial.println(I2C_SDA);
  Serial.print("  I2C SCL -> GPIO"); Serial.println(I2C_SCL);
  Serial.println("  PCA9685 address: 0x40");
  Serial.println("  SERVOMIN=150  SERVOMAX=600  FREQ=50Hz");
  Serial.println();

  // ── I2C Bus Initialisation ─────────────────────────────────────────────────
  // Wire.begin(SDA, SCL) — ESP32 Arduino core signature with explicit pins.
  // This is DIFFERENT from Arduino Uno where Wire.begin() takes no arguments.
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.println("Wire (I2C) initialised.");

  // ── I2C Device Scan ────────────────────────────────────────────────────────
#if I2C_SCAN_ON_BOOT
  i2cScan();
  // Expected: "[OK] I2C device found at 0x40"
  // If 0x40 is NOT listed → PCA9685 not responding. Check wiring!
#endif

  // ── PCA9685 Initialisation ─────────────────────────────────────────────────
  Serial.println("Initialising PCA9685...");
  pwm.begin();
  // WHAT pwm.begin() DOES:
  //   1. Sends RESET command to PCA9685 (register 0x00 = 0x80 → software reset)
  //   2. Waits 10ms for oscillator to stabilise
  //   3. Leaves all channels at 0 (servos at ~0° or holding last position)

  pwm.setPWMFreq(SERVO_FREQ);
  // WHAT setPWMFreq() DOES:
  //   Computes prescale = round(25 000 000 / (4096 x 50)) - 1 = 121
  //   Writes prescale to register 0xFE while PCA9685 is in SLEEP mode,
  //   then wakes it. The internal 25MHz oscillator is factory-trimmed
  //   but may drift ±1%. For servos, this is acceptable.
  delay(10);  // Extra stabilisation margin after frequency set

  // ── Move All Servos to Home ────────────────────────────────────────────────
  Serial.println();
  setAllServosHome();
  delay(2000);  // Give servos 2 seconds to physically reach HOME position

  Serial.println();
  Serial.println("==============================================");
  Serial.println("  Setup complete. Starting test loop.");
  Serial.println("  Monitor: 115200 baud | USB CDC");
  Serial.println("==============================================");
  Serial.println();
}

// =============================================================================
// LOOP — Continuous Test Sequence
//
// WHAT THIS TEST DOES:
//   1. Sweeps BR FEMUR (CH10) from 0° → 90°  (leg raises)
//   2. Sweeps BR FEMUR (CH10) from 90° → 0°  (leg lowers)
//   3. Returns ALL 12 servos to HOME between cycles
//   4. Repeats forever
//
// WHY BR FEMUR first?
//   It is the easiest to observe — attach one servo to CH10 and watch it move.
//   Once confirmed, expand to full gait logic by replacing this loop.
//
// HOW TO EXPAND:
//   Replace or add to this loop with calls like:
//     sweepServo(FL_HIP, 45, 90, 15);  // Sweep FL Hip from 45° to 90°
//     setServo(FL_HIP, 45);            // Jump FL Hip to 45° instantly
//     setAllServosHome();              // Reset all legs
// =============================================================================
void loop() {
  // ── Test 1: Sweep BR Femur 0° → 90° ────────────────────────────────────────
  Serial.println("-- Test: BR FEMUR (CH10) sweep  0 -> 90 deg --");
  sweepServo(BR_FEMUR, 0, 90, 15);
  // 90 steps x 15ms = 1350ms (1.35 seconds to sweep)
  delay(300);

  // ── Test 2: Sweep BR Femur 90° → 0° ────────────────────────────────────────
  Serial.println("-- Test: BR FEMUR (CH10) sweep 90 ->  0 deg --");
  sweepServo(BR_FEMUR, 90, 0, 15);
  delay(300);

  // ── Test 3: Verify All Legs Return Home ─────────────────────────────────────
  Serial.println("-- All servos -> HOME --");
  setAllServosHome();
  delay(1500);

  Serial.println();
}
