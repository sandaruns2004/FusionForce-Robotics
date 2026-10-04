// =============================================================================
// FusionForce RUNNER4 — 12 DOF Quadruped Robot
// PCA9685 Web Servo Control + Pose Presets + Forward & Inverse Kinematics
// BOARD:   ESP32-S3 (DevKitC-1 or XIAO ESP32-S3)
// VERSION: 3.0 — Pose Preset Sidebar + Play Trot Mode + FK/IK
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
//   5. LEFT SIDEBAR — click any pose preset to move all 12 servos
//      PLAY TROT    — cycles Trot A ↔ Trot B at adjustable speed
//      FK readout   — updates live on every slider or pose change
//      IK           — enter target foot position (X, Y, Z mm) → Apply
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
//   CH10:BL_HIP  CH11:BL_FEMUR  CH14:BL_TIBIA
//   CH 6:BR_HIP  CH 8:BR_FEMUR  CH 9:BR_TIBIA
// =============================================================================

#include <Adafruit_PWMServoDriver.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>
#include <math.h>

struct Vec3f {
  float x, y, z;
};

// ─────────────────────────────────────────────────────────────────────────────
// WiFi — Access Point mode (no external router required)
// ─────────────────────────────────────────────────────────────────────────────
const char *AP_SSID = "RUNNER4-Config";
const char *AP_PASS = "runner4robot";

// ─────────────────────────────────────────────────────────────────────────────
// ESP32-S3 I2C Pins
// ─────────────────────────────────────────────────────────────────────────────
#define I2C_SDA 8
#define I2C_SCL 9

// ─────────────────────────────────────────────────────────────────────────────
// PCA9685 Servo Driver
// ─────────────────────────────────────────────────────────────────────────────
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVOMIN 150  // Pulse tick at   0 deg (~732 us)
#define SERVOMAX 600  // Pulse tick at 180 deg (~2930 us)
#define SERVO_FREQ 50 // 50 Hz standard servo update rate

// ─────────────────────────────────────────────────────────────────────────────
// Robot Leg Geometry — Link Lengths (mm)
// ─────────────────────────────────────────────────────────────────────────────
#define L1_MM 30.0f // Coxa  (hip offset)
#define L2_MM 60.0f // Femur (upper leg)
#define L3_MM 80.0f // Tibia (lower leg)

// ─────────────────────────────────────────────────────────────────────────────
// PCA9685 Channel Definitions
// ─────────────────────────────────────────────────────────────────────────────
#define FL_HIP 0
#define FL_FEMUR 1
#define FL_TIBIA 2
#define FR_HIP 3
#define FR_FEMUR 4
#define FR_TIBIA 5
#define BL_HIP 10
#define BL_FEMUR 11
#define BL_TIBIA 14
#define BR_HIP 6
#define BR_FEMUR 8
#define BR_TIBIA 9

// ─────────────────────────────────────────────────────────────────────────────
// Leg channel map: legChannels[leg][0=hip, 1=femur, 2=tibia]
// Leg index: 0=FL, 1=FR, 2=BL, 3=BR
// ─────────────────────────────────────────────────────────────────────────────
const uint8_t legChannels[4][3] = {{FL_HIP, FL_FEMUR, FL_TIBIA},
                                   {FR_HIP, FR_FEMUR, FR_TIBIA},
                                   {BL_HIP, BL_FEMUR, BL_TIBIA},
                                   {BR_HIP, BR_FEMUR, BR_TIBIA}};

// Mirror: Left legs (+1), Right legs (-1)
const int legMirror[4] = {1, -1, 1, -1};

// ─────────────────────────────────────────────────────────────────────────────
// POSE PRESETS — [pose][leg 0-3][joint: 0=hip, 1=femur, 2=tibia]
//
// Pose 0: STANDING    — calibrated standing angles
// Pose 1: LOW CROUCH  — bent knees, ~50mm body height
// Pose 2: FL LEG UP   — Front-Left raised ~40mm, others planted
// Pose 3: FR LEG UP   — Front-Right raised ~40mm, others planted
// Pose 4: TROT A      — FL+BR diagonal pair raised (trot phase 1)
// Pose 5: TROT B      — FR+BL diagonal pair raised (trot phase 2)
// Pose 6: STRETCH     — all legs extended horizontally (workspace test)
// Pose 7: INITIAL     — raw servo power-on reference (CH14=10, rest=90)
//
// NOTE: Poses 2-5 raise angles are estimated from FK geometry.
//       Use the sliders + FK readout on the web UI to tune them for
//       your specific robot, then update values here and reflash.
// ─────────────────────────────────────────────────────────────────────────────
// Pose presets can be updated dynamically via the Web UI for calibration
int poseAngles[13][4][3] = {
    // Pose 0: INITIAL — original INIT_ values from PCA9685_12DOF_Controller
    {{45, 80, 160}, {94, 80, 0}, {94, 56, 10}, {45, 92, 160}},

    // Pose 1: STANDING
    {{45, 25, 115}, {94, 135, 45}, {94, 111, 55}, {45, 37, 115}},

    // Pose 2: LOW CROUCH (body lowered ~50mm)
    {{45, 110, 140}, {94, 110, 40}, {94, 110, 140}, {45, 110, 140}},

    // Pose 3: LOW STAND — intermediate crouch stance
    {{45, 50, 130}, {94, 110, 30}, {94, 86, 40}, {45, 62, 130}},

    // Pose 4: FL LEG UP (Front-Left raised)
    {{45, 38, 115}, {94, 125, 35}, {94, 101, 45}, {45, 80, 160}},

    // Pose 5: FR LEG UP (Front-Right raised)
    {{45, 80, 160}, {94, 55, 20}, {94, 56, 160}, {45, 92, 160}},

    // Pose 6: TROT A — diagonal FL + BR raised
    {{45, 60, 120}, {94, 80, 0}, {94, 56, 160}, {45, 72, 140}},

    // Pose 7: TROT B — diagonal FR + BL raised
    {{45, 80, 160}, {94, 55, 20}, {94, 36, 130}, {45, 92, 160}},

    // Pose 8: STRETCH — full horizontal reach (workspace test)
    {{45, 90, 90}, {94, 90, 90}, {94, 90, 90}, {45, 90, 90}},

    // ─── WALK FORWARD POSES
    // ───────────────────────────────────────────────────
    // Diagonal trot gait: Phase A = FL+BR swing, Phase B = FR+BL swing
    // VARIANT 1 — BASIC: your hip/femur values, tibia kept at standing angles
    //
    // Pose 9: WALK A — BASIC (FL+BR lifted)
    //   FL: Hip=60  Fem=35  Tib=115(keep)   FR: Hip=94  Fem=135 Tib=45(keep)
    //   BL: Hip=94  Fem=111 Tib=55(keep)    BR: Hip=60  Fem=47  Tib=115(keep)
    {{15, 35, 115}, {94, 135, 45}, {94, 111, 55}, {75, 47, 115}},

    // Pose 10: WALK B — BASIC (FR+BL lifted, FL+BR put down)
    //   FL: Hip=45  Fem=25  Tib=115(stand)  FR: Hip=104 Fem=125 Tib=45(keep)
    //   BL: Hip=104 Fem=101 Tib=55(keep)    BR: Hip=45  Fem=37  Tib=115(stand)
    {{45, 25, 115}, {124, 125, 45}, {64, 101, 55}, {45, 37, 115}},

    // VARIANT 2 — ENHANCED: your hip/femur values + tibia retracted for ground
    // clearance
    //
    // Pose 11: WALK A — ENHANCED (FL+BR lifted, tibia retracts)
    //   FL: Hip=60  Fem=35  Tib=130(retract) FR: Hip=94  Fem=135 Tib=45(keep)
    //   BL: Hip=94  Fem=111 Tib=55(keep)     BR: Hip=60  Fem=47
    //   Tib=130(retract)
    {{15, 35, 130}, {94, 135, 45}, {94, 111, 55}, {75, 47, 130}},

    // Pose 12: WALK B — ENHANCED (FR+BL lifted, tibia retracts)
    //   FL: Hip=45  Fem=25  Tib=115(stand)  FR: Hip=104 Fem=125 Tib=30(retract)
    //   BL: Hip=104 Fem=101 Tib=30(retract)  BR: Hip=45  Fem=37  Tib=115(stand)
    {{45, 25, 115}, {124, 125, 30}, {64, 101, 30}, {45, 37, 115}}};

#define POSE_COUNT 13

// Runtime servo angle state
int servoAngles[16] = {0};

// Web Server
WebServer server(80);
volatile bool sweepRequested = false;

// =============================================================================
// EMBEDDED HTML PAGE — v3.0 with Pose Preset Sidebar
// =============================================================================
const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>RUNNER4 · Pose Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:#0d1117;color:#e6edf3;min-height:100vh;overflow-x:hidden}

/* ── Header ── */
.hdr{background:linear-gradient(135deg,#161b22,#21262d);border-bottom:1px solid #30363d;padding:12px 20px;display:flex;align-items:center;justify-content:space-between;position:sticky;top:0;z-index:99}
.brand h1{font-size:1.2rem;font-weight:800;letter-spacing:3px;background:linear-gradient(135deg,#58a6ff,#a5d6ff);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.brand p{font-size:.65rem;color:#8b949e;margin-top:2px;letter-spacing:.5px}
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

/* ── Layout: Sidebar + Content ── */
.layout{display:flex;align-items:flex-start}

/* ── Left Sidebar ── */
.sidebar{
  width:210px;flex-shrink:0;
  background:#161b22;
  border-right:1px solid #30363d;
  padding:12px 10px 20px;
  display:flex;flex-direction:column;gap:5px;
  position:sticky;top:57px;
  height:calc(100vh - 57px);
  overflow-y:auto
}
.sidebar::-webkit-scrollbar{width:4px}
.sidebar::-webkit-scrollbar-track{background:transparent}
.sidebar::-webkit-scrollbar-thumb{background:#30363d;border-radius:4px}
.sb-label{font-size:.58rem;font-weight:700;letter-spacing:1.8px;color:#484f58;text-transform:uppercase;padding:6px 5px 3px;margin-top:4px}

/* ── Pose Buttons ── */
.pose-btn{
  width:100%;padding:9px 11px;
  border:1px solid #30363d;border-radius:8px;
  background:rgba(255,255,255,.03);
  color:#c9d1d9;cursor:pointer;
  font-size:.76rem;font-weight:600;text-align:left;
  transition:all .2s;
  display:flex;align-items:center;gap:9px;
  position:relative;overflow:hidden
}
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

/* ── Divider ── */
.sb-div{border:none;border-top:1px solid #21262d;margin:8px 0}

/* ── Play Mode Section ── */
.play-section{display:flex;flex-direction:column;gap:7px;padding:4px 2px}
.play-btn{
  width:100%;padding:10px;border:none;border-radius:9px;
  cursor:pointer;font-size:.82rem;font-weight:700;
  transition:all .2s;letter-spacing:.5px
}
.play-btn.stopped{background:linear-gradient(135deg,#1a7f37,#3fb950);color:#fff}
.play-btn.stopped:hover{box-shadow:0 4px 14px rgba(63,185,80,.4);transform:translateY(-1px)}
.play-btn.playing{background:linear-gradient(135deg,#b91c1c,#f85149);color:#fff;animation:pulse-red 1.5s infinite}
@keyframes pulse-red{0%,100%{box-shadow:0 0 0 0 rgba(248,81,73,.4)}50%{box-shadow:0 0 0 6px rgba(248,81,73,.0)}}
.speed-row{display:flex;align-items:center;gap:7px;font-size:.63rem;color:#6e7681}
.speed-sl{flex:1;-webkit-appearance:none;appearance:none;height:3px;border-radius:3px;background:#21262d;outline:none;cursor:pointer}
.speed-sl::-webkit-slider-thumb{-webkit-appearance:none;width:13px;height:13px;border-radius:50%;background:#58a6ff;cursor:pointer;border:2px solid #0d1117}
.step-count{font-size:.62rem;color:#484f58;text-align:center;font-family:monospace;min-height:14px}
.play-phase{font-size:.65rem;text-align:center;color:#6e7681;min-height:14px}

/* ── FK mini-summary in sidebar ── */
.fk-mini{background:rgba(255,255,255,.025);border:1px solid #21262d;border-radius:7px;padding:8px 9px;margin-top:4px}
.fk-mini-title{font-size:.57rem;font-weight:700;text-transform:uppercase;letter-spacing:1px;color:#484f58;margin-bottom:6px}
.fk-row{display:flex;justify-content:space-between;align-items:center;margin-bottom:3px}
.fk-leg-dot{width:7px;height:7px;border-radius:50%;flex-shrink:0}
.fk-leg-name{font-size:.6rem;color:#6e7681;flex:1;margin-left:5px}
.fk-xyz{font-size:.58rem;font-family:monospace;color:#8b949e}

/* ── Main content ── */
.content{flex:1;padding:14px 16px;min-width:0}

/* ── Grid ── */
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;max-width:900px;margin:0 auto}
@media(max-width:900px){.grid{grid-template-columns:1fr}}
@media(max-width:640px){.layout{flex-direction:column}.sidebar{width:100%;height:auto;position:static;flex-direction:row;flex-wrap:wrap;gap:6px}.sidebar .pose-btn{width:calc(50% - 3px)}.fk-mini{display:none}}

/* ── Card ── */
.card{background:#161b22;border:1px solid #30363d;border-radius:12px;overflow:hidden;transition:box-shadow .2s,border-color .3s}
.card:hover{box-shadow:0 5px 22px rgba(0,0,0,.45)}
.card.pose-anim{border-color:#58a6ff66;animation:card-flash .4s ease-out}
@keyframes card-flash{0%{box-shadow:0 0 0 2px rgba(88,166,255,.5)}100%{box-shadow:none}}
.ch{display:flex;align-items:center;gap:9px;padding:10px 14px;border-bottom:1px solid #21262d}
.badge{font-size:.88rem;font-weight:800;letter-spacing:1px;padding:3px 9px;border-radius:7px}
.cn{font-size:.8rem;color:#8b949e;flex:1}
.btn-lh{padding:4px 10px;border-radius:6px;border:none;cursor:pointer;font-size:.68rem;font-weight:600;transition:all .18s}
.cb{padding:12px 14px}

/* ── Slider Row ── */
.jr{display:flex;align-items:center;gap:8px;margin-bottom:10px}
.jl{font-size:.66rem;color:#8b949e;width:86px;flex-shrink:0;font-weight:700;letter-spacing:.5px}
.sl{flex:1;-webkit-appearance:none;appearance:none;height:4px;border-radius:4px;background:#21262d;outline:none;cursor:pointer}
.sl::-webkit-slider-thumb{-webkit-appearance:none;width:15px;height:15px;border-radius:50%;cursor:pointer;border:2px solid #0d1117;box-shadow:0 1px 6px rgba(0,0,0,.5);transition:transform .1s}
.sl::-webkit-slider-thumb:hover{transform:scale(1.25)}
.av{font-size:.78rem;font-weight:700;font-family:monospace;min-width:36px;text-align:right;color:#e6edf3}
.btn-step{background:#21262d;border:none;border-radius:4px;color:#c9d1d9;width:22px;height:22px;cursor:pointer;font-size:.88rem;font-weight:700;line-height:20px;text-align:center;transition:background .2s;flex-shrink:0}
.btn-step:hover{background:#30363d}

/* ── Info Panels ── */
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
.log{max-width:900px;margin:12px auto 20px;background:#161b22;border:1px solid #30363d;border-radius:10px;overflow:hidden}
.logh{padding:7px 14px;border-bottom:1px solid #21262d;font-size:.65rem;color:#6e7681;font-weight:700;text-transform:uppercase;letter-spacing:1px;display:flex;justify-content:space-between;align-items:center}
.logb{padding:8px 14px;font-family:monospace;font-size:.68rem;color:#6e7681;max-height:80px;overflow-y:auto}
.li{padding:1px 0}.li.inf{color:#58a6ff}.li.ok2{color:#3fb950}.li.er2{color:#f85149}.li.pose{color:#a78bfa}

/* ── Walk Variant Toggle Buttons ── */
.walk-variant{
  flex:1;padding:5px 4px;border:1px solid #30363d;
  border-radius:6px;background:rgba(255,255,255,.04);
  color:#6e7681;cursor:pointer;font-size:.68rem;font-weight:600;
  transition:all .18s
}
.walk-variant.active{
  background:rgba(88,166,255,.15);border-color:#58a6ff;color:#58a6ff
}
.walk-variant:hover:not(.active){
  background:rgba(255,255,255,.08);color:#c9d1d9
}
.walk-steps-row{display:flex;align-items:center;gap:6px;margin-bottom:6px}
.walk-steps-in{
  width:52px;background:#0d1117;border:1px solid #30363d;
  border-radius:6px;color:#e6edf3;padding:4px 6px;
  font-size:.75rem;text-align:center;outline:none
}
.walk-steps-in:focus{border-color:#58a6ff}
</style>
</head>
<body>

<header class="hdr">
  <div class="brand">
    <h1>RUNNER4</h1>
    <p>12-DOF Quadruped &nbsp;·&nbsp; Pose Presets + FK/IK &nbsp;·&nbsp; ESP32-S3 &nbsp;·&nbsp; v3.0</p>
  </div>
  <div class="hdr-r">
    <div><span class="sdot" id="sd"></span><span class="stxt" id="st">Connecting...</span></div>
    <button class="btn btn-sw" onclick="doSweep()">&#9654; Sweep</button>
    <button class="btn btn-all" onclick="allHome()">&#8962; All HOME</button>
  </div>
</header>

<div class="layout">

<!-- ════════════════════ LEFT SIDEBAR ════════════════════ -->
<nav class="sidebar" id="sidebar">
  <div class="sb-label">Pose Presets</div>

  <!-- Pose 0: Initial -->
  <div class="pose-item">
    <button class="pose-btn" id="pb0" onclick="applyPose(0)">
      <span class="pose-icon">&#9881;&#65039;</span>
      <span class="pose-name">Initial</span>
      <span class="pose-badge">REF</span>
    </button>
    <button class="pose-save" onclick="savePose(0)" title="Save current sliders to Initial">&#128190;</button>
  </div>

  <!-- Pose 1: Standing -->
  <div class="pose-item">
    <button class="pose-btn active" id="pb1" onclick="applyPose(1)">
      <span class="pose-icon">🦾</span>
      <span class="pose-name">Standing</span>
      <span class="pose-badge">HOME</span>
    </button>
    <button class="pose-save" onclick="savePose(1)" title="Save current sliders to Standing">&#128190;</button>
  </div>

  <!-- Pose 2: Low Crouch -->
  <div class="pose-item">
    <button class="pose-btn" id="pb2" onclick="applyPose(2)">
      <span class="pose-icon">⬇️</span>
      <span class="pose-name">Low Crouch</span>
    </button>
    <button class="pose-save" onclick="savePose(2)" title="Save current sliders to Low Crouch">&#128190;</button>
  </div>

  <!-- Pose 3: Low Stand -->
  <div class="pose-item">
    <button class="pose-btn" id="pb3" onclick="applyPose(3)">
      <span class="pose-icon">🔽</span>
      <span class="pose-name">Low Stand</span>
    </button>
    <button class="pose-save" onclick="savePose(3)" title="Save current sliders to Low Stand">&#128190;</button>
  </div>

  <!-- Pose 4: FL Leg Up -->
  <div class="pose-item">
    <button class="pose-btn" id="pb4" onclick="applyPose(4)">
      <span class="pose-icon">↖️</span>
      <span class="pose-name">FL Leg Up</span>
      <span class="pose-badge">STEP</span>
    </button>
    <button class="pose-save" onclick="savePose(4)" title="Save current sliders to FL Leg Up">&#128190;</button>
  </div>

  <!-- Pose 5: FR Leg Up -->
  <div class="pose-item">
    <button class="pose-btn" id="pb5" onclick="applyPose(5)">
      <span class="pose-icon">↗️</span>
      <span class="pose-name">FR Leg Up</span>
      <span class="pose-badge">STEP</span>
    </button>
    <button class="pose-save" onclick="savePose(5)" title="Save current sliders to FR Leg Up">&#128190;</button>
  </div>

  <!-- Pose 6: Trot A -->
  <div class="pose-item">
    <button class="pose-btn" id="pb6" onclick="applyPose(6)">
      <span class="pose-icon">🔄</span>
      <span class="pose-name">Trot A</span>
      <span class="pose-badge">FL+BR</span>
    </button>
    <button class="pose-save" onclick="savePose(6)" title="Save current sliders to Trot A">&#128190;</button>
  </div>

  <!-- Pose 7: Trot B -->
  <div class="pose-item">
    <button class="pose-btn" id="pb7" onclick="applyPose(7)">
      <span class="pose-icon">🔃</span>
      <span class="pose-name">Trot B</span>
      <span class="pose-badge">FR+BL</span>
    </button>
    <button class="pose-save" onclick="savePose(7)" title="Save current sliders to Trot B">&#128190;</button>
  </div>

  <!-- Pose 8: Stretch -->
  <div class="pose-item">
    <button class="pose-btn" id="pb8" onclick="applyPose(8)">
      <span class="pose-icon">&#8596;&#65039;</span>
      <span class="pose-name">Stretch</span>
      <span class="pose-badge">TEST</span>
    </button>
    <button class="pose-save" onclick="savePose(8)" title="Save current sliders to Stretch">&#128190;</button>
  </div>

  <hr class="sb-div">
  <div class="sb-label">Transition Speed</div>
  <div class="play-section" style="gap:5px">
    <div class="speed-row">
      <span>Instant</span>
      <input type="range" class="speed-sl" id="transitionSl" min="0" max="3000" value="0" oninput="updTransDisplay()">
      <span>Slow</span>
    </div>
    <div style="font-size:.6rem;color:#484f58;text-align:center">
      <span id="transitionVal">Instant (0ms)</span>
    </div>
  </div>

  <hr class="sb-div">
  <div class="sb-label">Play Mode</div>

  <!-- Play Trot -->
  <div class="play-section">
    <button class="play-btn stopped" id="playBtn" onclick="togglePlay()">&#9654; Play Trot</button>
    <div class="play-phase" id="playPhase"></div>
    <div class="speed-row">
      <span>Fast</span>
      <input type="range" class="speed-sl" id="speedSl" min="200" max="2000" value="700">
      <span>Slow</span>
    </div>
    <div style="font-size:.6rem;color:#484f58;text-align:center">
      <span id="speedVal">700</span>ms / step
    </div>
    <div class="step-count" id="stepCount"></div>
  </div>

  <hr class="sb-div">
  <div class="sb-label">Walk Forward</div>

  <!-- ─── Walk Forward Section ─── -->
  <div class="play-section">

    <!-- Variant selector: Basic vs Enhanced -->
    <div style="display:flex;gap:4px;margin-bottom:4px">
      <button id="wvBtn0" class="walk-variant active" onclick="setWalkVariant(0)">Basic</button>
      <button id="wvBtn1" class="walk-variant"        onclick="setWalkVariant(1)">Enhanced</button>
    </div>
    <div style="font-size:.55rem;color:#484f58;margin-bottom:6px;padding:0 2px" id="wvDesc">
      Tibia unchanged &mdash; safe starting mode
    </div>

    <!-- Step count input -->
    <div class="walk-steps-row">
      <span style="font-size:.63rem;color:#6e7681;flex:1">Steps</span>
      <input type="number" id="walkStepsIn" class="walk-steps-in" min="0" max="200" value="10">
      <span style="font-size:.6rem;color:#484f58">(0=&infin;)</span>
    </div>

    <!-- Walk Play / Stop button -->
    <button class="play-btn stopped" id="walkBtn" onclick="toggleWalk()">
      &#128694; Walk Forward
    </button>

    <!-- Live phase indicator -->
    <div class="play-phase" id="walkPhase"></div>
    <!-- Live step counter -->
    <div class="step-count" id="walkStepCount"></div>

    <!-- Walk speed slider -->
    <div class="speed-row" style="margin-top:6px">
      <span>Fast</span>
      <input type="range" class="speed-sl" id="walkSpeedSl"
             min="100" max="3000" value="600"
             oninput="updWalkSpeed()">
      <span>Slow</span>
    </div>
    <div style="font-size:.6rem;color:#484f58;text-align:center">
      <span id="walkSpeedVal">600</span>ms / step
    </div>

  </div><!-- end Walk Forward section -->

  <hr class="sb-div">
  <div class="sb-label">Foot Positions</div>

  <!-- FK mini summary -->
  <div class="fk-mini">
    <div class="fk-mini-title">Live FK (mm)</div>
    <div class="fk-row">
      <div class="fk-leg-dot" style="background:#58a6ff"></div>
      <span class="fk-leg-name">FL</span>
      <span class="fk-xyz" id="mfk0">—</span>
    </div>
    <div class="fk-row">
      <div class="fk-leg-dot" style="background:#3fb950"></div>
      <span class="fk-leg-name">FR</span>
      <span class="fk-xyz" id="mfk1">—</span>
    </div>
    <div class="fk-row">
      <div class="fk-leg-dot" style="background:#a78bfa"></div>
      <span class="fk-leg-name">BL</span>
      <span class="fk-xyz" id="mfk2">—</span>
    </div>
    <div class="fk-row">
      <div class="fk-leg-dot" style="background:#f59e0b"></div>
      <span class="fk-leg-name">BR</span>
      <span class="fk-xyz" id="mfk3">—</span>
    </div>
  </div>

</nav>
<!-- ════════════════════ END SIDEBAR ════════════════════ -->

<!-- ════════════════════ MAIN CONTENT ════════════════════ -->
<main class="content">
<div class="grid">

<!-- ═══════ FL ═══════ -->
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

<!-- ═══════ FR ═══════ -->
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

<!-- ═══════ BL ═══════ -->
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

<!-- ═══════ BR ═══════ -->
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

</div><!-- .grid -->

<div class="log">
  <div class="logh"><span>Event Log</span><button onclick="document.getElementById('lb').innerHTML=''" style="background:none;border:none;color:#6e7681;cursor:pointer;font-size:.65rem">Clear</button></div>
  <div class="logb" id="lb"><div class="li inf">Initialising RUNNER4 v3.0 — Pose Presets + FK/IK...</div></div>
</div>
</main>
<!-- ════════════════════ END CONTENT ════════════════════ -->

</div><!-- .layout -->

<script>
// ── Constants ──
const L1=30, L2=60, L3=80;
const D2R=Math.PI/180, R2D=180/Math.PI;

// Channel map: CH[leg][hip, femur, tibia]
const CH=[[0,1,2],[3,4,5],[10,11,14],[6,8,9]];
const MIR=[1,-1,1,-1];  // mirror: +1=left, -1=right

// Pose data — mirrors C++ poseAngles[9][4][3]
// [leg 0-3][hip, femur, tibia]
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

// Runtime state
let A=new Array(16).fill(0);
let dbt={};
let activePose=0;
let playMode=false;
let playInterval=null;
let playStep=0;
let stepCount=0;
let pollTimer=null;      // holds setInterval id so we can pause/resume
let pollBusy=false;      // true while a pose/home command is in-flight

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

// ── Cross-browser fetch with timeout (replaces AbortSignal.timeout) ──
// Works on Chrome, Firefox, Safari, Android WebView, all versions
function fetchWT(url, ms) {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), ms);
  return fetch(url, { signal: ctrl.signal })
    .then(r => { clearTimeout(timer); return r; })
    .catch(e => { clearTimeout(timer); throw e; });
}

// ── Pause / Resume background poll ──
function pausePoll(){
  pollBusy=true;
}
function resumePoll(){
  pollBusy=false;
}

// ── Slider move handler ──
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

// ── Step +/- ──
function step(ch,d){
  const el=document.getElementById('s'+ch);
  let v=parseInt(el.value)+d;
  v=Math.max(0,Math.min(180,v));
  el.value=v; mv(ch,v);
}

// ── Send single channel to ESP32 ──
async function send(ch,a){
  try{
    const r=await fetchWT('/set?ch='+ch+'&angle='+a, 3000);
    if(!r.ok) throw new Error('bad status');
    cst(true);
    lg('CH'+String(ch).padStart(2,'0')+' \u2192 '+a+'\u00b0','ok2');
  }catch(e){cst(false);lg('TX fail CH'+ch,'er2');}
}

// ── Transition speed helpers ──
function updTransDisplay(){
  const ms=parseInt(document.getElementById('transitionSl').value);
  document.getElementById('transitionVal').textContent = ms===0 ? 'Instant (0ms)' : ms+'ms';
}
function getPoseSpeed(){
  return parseInt(document.getElementById('transitionSl').value);
}

// ── Apply Pose — sends /pose?id=N&speed=X, pauses poll while in flight ──
async function applyPose(id){
  activePose=id;

  // Update sidebar button styles
  for(let i=0;i<POSES.length;i++){
    document.getElementById('pb'+i).classList.toggle('active',i===id);
  }

  // Flash leg cards
  for(let c=0;c<4;c++){
    const card=document.getElementById('card'+c);
    card.classList.remove('pose-anim');
    void card.offsetWidth;
    card.classList.add('pose-anim');
  }

  pausePoll();
  let success=false;
  const speedMs = getPoseSpeed();
  // Timeout = base 5s + transition duration so we don't abort mid-move
  const timeoutMs = 5000 + speedMs;

  // Try up to 2 times
  for(let attempt=0; attempt<2; attempt++){
    try{
      const r=await fetchWT('/pose?id='+id+'&speed='+speedMs, timeoutMs);
      if(!r.ok) throw new Error('bad status');
      const d=await r.json();
      cst(true);
      // Sync all sliders from returned state
      for(let i=0;i<16;i++){
        A[i]=d.a[i];
        const el=document.getElementById('s'+i);
        if(el){
          el.value=d.a[i];
          document.getElementById('v'+i).textContent=d.a[i]+'\u00b0';
        }
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
    // Final fallback: update UI only, motors won't move
    lg('\u26a0 Cannot reach ESP32 — UI updated only, motors unchanged','er2');
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

// ── Save Pose (Current sliders to Preset) ──
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

// ── Play Trot Mode ──
function updateSpeedDisplay(){
  const ms=parseInt(document.getElementById('speedSl').value);
  document.getElementById('speedVal').textContent=ms;
  if(playMode){
    clearInterval(playInterval);
    startTrotCycle(ms);
  }
}

function startTrotCycle(ms){
  playInterval=setInterval(async()=>{
    playStep=(playStep+1)%2;
    stepCount++;
    const poseId=playStep===0?6:7; // 6=Trot A, 7=Trot B
    document.getElementById('playPhase').textContent='Phase: '+(playStep===0?'FL+BR up':'FR+BL up');
    document.getElementById('stepCount').textContent='Steps: '+stepCount;
    await applyPose(poseId);
  },ms);
}

function togglePlay(){
  playMode=!playMode;
  const btn=document.getElementById('playBtn');
  if(playMode){
    btn.className='play-btn playing';
    btn.innerHTML='&#9646;&#9646; Stop Trot';
    playStep=0; stepCount=0;
    const ms=parseInt(document.getElementById('speedSl').value);
    applyPose(4);
    document.getElementById('playPhase').textContent='Phase: FL+BR up';
    document.getElementById('stepCount').textContent='Steps: 0';
    startTrotCycle(ms);
    lg('Play Trot started \u2014 '+ms+'ms/step','inf');
  }else{
    btn.className='play-btn stopped';
    btn.innerHTML='&#9654; Play Trot';
    clearInterval(playInterval); playInterval=null;
    document.getElementById('playPhase').textContent='';
    document.getElementById('stepCount').textContent='';
    applyPose(0);
    lg('Trot stopped \u2192 Standing','inf');
  }
}

// ── Walk Forward ──
let walkRunning   = false;
let walkVariant   = 0;   // 0=Basic (poses 9+10), 1=Enhanced (poses 11+12)
let walkPollTimer = null;

function setWalkVariant(v){
  walkVariant=v;
  document.getElementById('wvBtn0').classList.toggle('active',v===0);
  document.getElementById('wvBtn1').classList.toggle('active',v===1);
  document.getElementById('wvDesc').textContent=v===0
    ?'Tibia unchanged \u2014 safe starting mode'
    :'Tibia retracts \u2014 better ground clearance';
}

function updWalkSpeed(){
  document.getElementById('walkSpeedVal').textContent=
    document.getElementById('walkSpeedSl').value;
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
    // Timeout = 6s base + 2x speed for very slow walks
    await fetchWT('/walk?steps='+steps+'&speed='+speed+'&variant='+variant, 8000);
    document.getElementById('walkPhase').textContent=
      steps===0?'Continuous walk\u2026':'Phase: FL+BR up';
    document.getElementById('walkStepCount').textContent=
      steps===0?'Steps: 0 (\u221e)':'Steps: 0 / '+steps;
    walkPollTimer=setInterval(pollWalkStatus,400);
    lg('Walk '+(variant===0?'Basic':'Enhanced')+' started \u2014 '+
       (steps===0?'continuous':steps+' steps')+' @ '+speed+'ms/step','inf');
  }catch(e){
    walkRunning=false;
    btn.className='play-btn stopped';
    btn.innerHTML='\ud83d\udeb6 Walk Forward';
    lg('Walk command failed \u2014 check WiFi','er2');
  }
}

async function pollWalkStatus(){
  try{
    const r=await fetchWT('/walkstatus',2000);
    const d=await r.json();
    const phase=(d.steps%2===0)?'FL+BR up':'FR+BL up';
    document.getElementById('walkPhase').textContent='Phase: '+phase;
    document.getElementById('walkStepCount').textContent=
      'Steps: '+d.steps+(d.target>0?' / '+d.target:' (\u221e)');
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

// ── Forward Kinematics ──
function fk(h,f,t,mir){
  const a1=(h-90)*D2R, a2=(f-90)*D2R, a3=(t-90)*D2R;
  const reach=L1+L2*Math.cos(a2)+L3*Math.cos(a2+a3);
  const z=L2*Math.sin(a2)+L3*Math.sin(a2+a3);
  return{
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
  document.getElementById('mfk'+l).textContent=p.x+', '+p.y+', '+p.z;
}
function updAllFK(){for(let i=0;i<4;i++)updFK(i);}

// ── Inverse Kinematics ──
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

// ── Leg HOME ──
async function legHome(l){
  pausePoll();
  try{
    const r=await fetchWT('/leghome?leg='+l, 4000);
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
  }catch(e){cst(false);lg('legHome failed — check WiFi','er2');}
  resumePoll();
}

// ── All HOME ──
async function allHome(){
  pausePoll();
  try{
    const r=await fetchWT('/home', 6000);
    const d=await r.json();
    for(let i=0;i<16;i++){
      A[i]=d.a[i];
      const el=document.getElementById('s'+i);
      if(el){el.value=d.a[i];document.getElementById('v'+i).textContent=d.a[i]+'\u00b0';}
    }
    updAllFK();
    for(let i=0;i<POSES.length;i++) document.getElementById('pb'+i).classList.toggle('active',i===0);
    activePose=0;
    cst(true);
    lg('All 12 servos \u2192 HOME (Standing)','ok2');
  }catch(e){cst(false);lg('allHome failed — check WiFi','er2');}
  resumePoll();
}

// ── Sweep Test ──
async function doSweep(){
  lg('Sweep test started on BR_FEMUR (CH8)...','inf');
  try{await fetchWT('/sweep', 3000);lg('Sweep running...','ok2');}
  catch(e){lg('Sweep trigger failed','er2');}
}

// ── State Poll — skips if a command is in-flight ──
async function poll(){
  if(pollBusy) return;   // don't collide with pose / home commands
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

// ── Speed slider live update ──
document.getElementById('speedSl').addEventListener('input', updateSpeedDisplay);

// ── Init ──
window.onload=function(){
  updAllFK();
  poll();
  pollTimer=setInterval(poll, 3000);  // 3s poll — less aggressive, avoids collision
  lg('RUNNER4 v3.0 — L1='+L1+' L2='+L2+' L3='+L3+' mm  |  8 Pose Presets + Play Trot','inf');
  lg('Tip: if motors not moving, check WiFi connection (status top-right)','inf');
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

void setServo(uint8_t ch, int angle) {
  angle = constrain(angle, 0, 180);
  servoAngles[ch] = angle;
  pwm.setPWM(ch, 0, angleToPulse(angle));
  // Note: Serial.printf removed here — it was blocking the main loop
  // and causing WiFi TCP connections to time out while waiting for HTTP
  // responses. Use the Serial output in setup/loop only for debug at boot.
}

void setAllServosHome() {
  Serial.println("[HOME] All 12 servos -> STANDING:");
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      setServo(legChannels[leg][joint],
               poseAngles[1][leg][joint]); // 1=Standing
      delay(10); // stagger servo starts to reduce current spike
    }
  }
  Serial.println("[HOME] Done.");
}

void setLegHome(int leg) {
  for (int joint = 0; joint < 3; joint++) {
    setServo(legChannels[leg][joint], poseAngles[1][leg][joint]); // 1=Standing
  }
}

// =============================================================================
// MOVE TO POSE — Linear interpolation over speedMs milliseconds
// Called AFTER HTTP response is sent so WiFi stays alive during movement.
// speedMs=0 means instant jump (no interpolation)
// =============================================================================
void moveToPose(int id, int speedMs) {
  if (speedMs <= 0) {
    // Instant: just jump to target
    for (int leg = 0; leg < 4; leg++) {
      for (int joint = 0; joint < 3; joint++) {
        setServo(legChannels[leg][joint], poseAngles[id][leg][joint]);
        delay(8);
      }
      yield();
    }
    return;
  }

  // Find the largest angle difference among all 12 servos
  // This determines number of interpolation steps (1 degree per step)
  int maxDiff = 0;
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      uint8_t ch = legChannels[leg][joint];
      int diff = abs(poseAngles[id][leg][joint] - servoAngles[ch]);
      if (diff > maxDiff)
        maxDiff = diff;
    }
  }

  if (maxDiff == 0)
    return; // already at target, nothing to do

  // Snapshot start angles before we begin moving
  int startAngles[16];
  memcpy(startAngles, servoAngles, sizeof(servoAngles));

  // Interpolate: step through from 0 to maxDiff
  // Each step moves every servo proportionally toward its target
  unsigned long moveStart = millis();
  for (int step = 1; step <= maxDiff; step++) {
    float t = (float)step / (float)maxDiff; // 0.0 -> 1.0

    for (int leg = 0; leg < 4; leg++) {
      for (int joint = 0; joint < 3; joint++) {
        uint8_t ch = legChannels[leg][joint];
        int angle =
            startAngles[ch] +
            (int)roundf(t * (poseAngles[id][leg][joint] - startAngles[ch]));
        setServo(ch, angle);
      }
    }

    // Precise timing: hold until this step's time slot
    unsigned long stepTarget =
        moveStart + (unsigned long)((long)speedMs * step / maxDiff);
    unsigned long now = millis();
    if (stepTarget > now)
      delay(stepTarget - now);
    yield(); // keep WiFi background task alive
  }

  Serial.printf("[POSE] Transition done. Steps=%d, duration=%lums\n", maxDiff,
                millis() - moveStart);
}

// =============================================================================
// FORWARD KINEMATICS
// =============================================================================
Vec3f computeFK(int hipDeg, int femurDeg, int tibiaDeg, int mirror) {
  const float a1 = (hipDeg - 90) * DEG_TO_RAD;
  const float a2 = (femurDeg - 90) * DEG_TO_RAD;
  const float a3 = (tibiaDeg - 90) * DEG_TO_RAD;
  const float reach = L1_MM + L2_MM * cosf(a2) + L3_MM * cosf(a2 + a3);
  const float z = L2_MM * sinf(a2) + L3_MM * sinf(a2 + a3);
  Vec3f pos;
  pos.x = reach * cosf(a1);
  pos.y = reach * sinf(a1) * (float)mirror;
  pos.z = z;
  return pos;
}

// =============================================================================
// INVERSE KINEMATICS
// =============================================================================
bool computeIK(float px, float py, float pz, int mirror, int *hipOut,
               int *femurOut, int *tibiaOut) {
  py *= (float)mirror;
  const float a1 = atan2f(py, px);
  const float rTotal = sqrtf(px * px + py * py);
  const float r = rTotal - L1_MM;
  const float d = sqrtf(r * r + pz * pz);
  const float maxR = L2_MM + L3_MM;
  const float minR = fabsf(L2_MM - L3_MM);
  if (d > maxR - 0.5f || d < minR + 0.5f)
    return false;
  float cosA3 =
      (d * d - L2_MM * L2_MM - L3_MM * L3_MM) / (2.0f * L2_MM * L3_MM);
  cosA3 = constrain(cosA3, -1.0f, 1.0f);
  const float a3 = acosf(cosA3);
  const float a2 =
      atan2f(pz, r) - atan2f(L3_MM * sinf(a3), L2_MM + L3_MM * cosf(a3));
  int hip = (int)roundf(a1 * RAD_TO_DEG) + 90;
  int femur = (int)roundf(a2 * RAD_TO_DEG) + 90;
  int tibia = (int)roundf(a3 * RAD_TO_DEG) + 90;
  if (hip < 0 || hip > 180 || femur < 0 || femur > 180 || tibia < 0 ||
      tibia > 180)
    return false;
  *hipOut = hip;
  *femurOut = femur;
  *tibiaOut = tibia;
  return true;
}

// =============================================================================
// I2C SCANNER
// =============================================================================
void i2cScan() {
  Serial.println("-------------------------------------------------");
  Serial.println("I2C Bus Scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  [OK] 0x%02X\n", addr);
      found++;
    }
  }
  if (found == 0)
    Serial.println("  [!!] No I2C devices found!");
  Serial.printf("  Total: %d device(s)\n", found);
  Serial.println("-------------------------------------------------");
}

// =============================================================================
// JSON HELPERS
// =============================================================================
String stateJSON() {
  String j = "{\"a\":[";
  for (int i = 0; i < 16; i++) {
    j += servoAngles[i];
    if (i < 15)
      j += ',';
  }
  j += "]}";
  return j;
}

String legJSON(int leg) {
  String j = "{\"a\":[";
  j += servoAngles[legChannels[leg][0]];
  j += ',';
  j += servoAngles[legChannels[leg][1]];
  j += ',';
  j += servoAngles[legChannels[leg][2]];
  j += "]}";
  return j;
}

void addCORS(WebServer &s) { s.sendHeader("Access-Control-Allow-Origin", "*"); }

// =============================================================================
// HTTP HANDLERS
// =============================================================================

void hRoot() { server.send(200, "text/html", INDEX_HTML); }

void hSet() {
  if (!server.hasArg("ch") || !server.hasArg("angle")) {
    server.send(400, "application/json", "{\"error\":\"missing ch or angle\"}");
    return;
  }
  int ch = server.arg("ch").toInt();
  int angle = server.arg("angle").toInt();
  if (ch < 0 || ch > 15 || angle < 0 || angle > 180) {
    server.send(400, "application/json", "{\"error\":\"out of range\"}");
    return;
  }
  setServo((uint8_t)ch, angle);
  addCORS(server);
  String resp = "{\"ok\":true,\"ch\":";
  resp += ch;
  resp += ",\"angle\":";
  resp += angle;
  resp += "}";
  server.send(200, "application/json", resp);
}

void hState() {
  addCORS(server);
  server.send(200, "application/json", stateJSON());
}

// hHome — send response FIRST, then move (keeps WiFi alive)
void hHome() {
  addCORS(server);
  // Build target JSON from pose 0 (standing) before moving
  server.send(200, "application/json", stateJSON());
  // Now move servos after HTTP response is already sent
  setAllServosHome();
  Serial.println("[WEB] All HOME done.");
}

// hLegHome — send response FIRST, then move
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
  addCORS(server);
  server.send(200, "application/json", legJSON(leg));
  // Move AFTER response sent
  setLegHome(leg);
  Serial.printf("[WEB] Leg %d -> HOME done.\n", leg);
}

void hIK() {
  if (!server.hasArg("leg") || !server.hasArg("x") || !server.hasArg("y") ||
      !server.hasArg("z")) {
    server.send(400, "application/json", "{\"error\":\"missing args\"}");
    return;
  }
  int leg = server.arg("leg").toInt();
  float px = server.arg("x").toFloat();
  float py = server.arg("y").toFloat();
  float pz = server.arg("z").toFloat();
  if (leg < 0 || leg > 3) {
    server.send(400, "application/json", "{\"error\":\"invalid leg\"}");
    return;
  }
  int hip, femur, tibia;
  bool ok = computeIK(px, py, pz, legMirror[leg], &hip, &femur, &tibia);
  if (!ok) {
    addCORS(server);
    server.send(200, "application/json",
                "{\"ok\":false,\"error\":\"unreachable\"}");
    return;
  }
  setServo(legChannels[leg][0], hip);
  setServo(legChannels[leg][1], femur);
  setServo(legChannels[leg][2], tibia);
  addCORS(server);
  String resp = "{\"ok\":true,\"hip\":";
  resp += hip;
  resp += ",\"femur\":";
  resp += femur;
  resp += ",\"tibia\":";
  resp += tibia;
  resp += "}";
  server.send(200, "application/json", resp);
  Serial.printf("[IK] Leg %d -> H:%d F:%d T:%d\n", leg, hip, femur, tibia);
}

// ─────────────────────────────────────────────────────────────────────────────
// GET /pose?id=N  — apply one of 8 named pose presets to all 12 servos
// KEY: HTTP response is sent FIRST, servos move AFTER — prevents WiFi timeout
// ─────────────────────────────────────────────────────────────────────────────
void hPose() {
  if (!server.hasArg("id")) {
    server.send(400, "application/json", "{\"error\":\"missing id\"}");
    return;
  }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= POSE_COUNT) {
    server.send(400, "application/json",
                "{\"error\":\"invalid pose id (0-8)\"}");
    return;
  }

  const char *poseNames[13] = {"Initial",       "Standing",    "LowCrouch",
                               "LowStand",      "FL_Up",       "FR_Up",
                               "Trot_A",        "Trot_B",      "Stretch",
                               "WalkA_Basic",   "WalkB_Basic", "WalkA_Enhanced",
                               "WalkB_Enhanced"};

  // ── STEP 1: Build JSON with TARGET angles from pose table ──
  // Build using poseAngles[id] so client gets correct target positions
  // immediately
  int targetAngles[16];
  memcpy(targetAngles, servoAngles, sizeof(servoAngles)); // start from current
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      targetAngles[legChannels[leg][joint]] = poseAngles[id][leg][joint];
    }
  }
  String respJson = "{\"a\":[";
  for (int i = 0; i < 16; i++) {
    respJson += targetAngles[i];
    if (i < 15)
      respJson += ',';
  }
  respJson += "]}";

  // ── STEP 2: Send HTTP response IMMEDIATELY — browser is unblocked ──
  addCORS(server);
  server.send(200, "application/json", respJson);
  Serial.printf("[POSE] -> %s (%d) — response sent, moving servos now\n",
                poseNames[id], id);

  // ── STEP 3: Smooth interpolated move AFTER response — WiFi stays alive ──
  int speedMs = server.hasArg("speed")
                    ? constrain(server.arg("speed").toInt(), 0, 5000)
                    : 0;
  moveToPose(id, speedMs);
  Serial.printf("[POSE] -> %s done.\n", poseNames[id]);
}

// ─────────────────────────────────────────────────────────────────────────────
// GET /savepose?id=N&a00=..&a01=..  — save current angles into a pose preset
// ─────────────────────────────────────────────────────────────────────────────
void hSavePose() {
  if (!server.hasArg("id")) {
    server.send(400, "application/json", "{\"error\":\"missing id\"}");
    return;
  }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= POSE_COUNT) {
    server.send(400, "application/json", "{\"error\":\"invalid pose id\"}");
    return;
  }

  // Read arguments for each leg and joint: a{leg}{joint}
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      char argName[4];
      sprintf(argName, "a%d%d", leg, joint);
      if (server.hasArg(argName)) {
        poseAngles[id][leg][joint] = server.arg(argName).toInt();
      }
    }
  }

  Serial.printf("[POSE] Updated Pose %d in RAM.\n", id);
  addCORS(server);
  server.send(200, "application/json", "{\"ok\":true}");
}

// =============================================================================
// WALK FORWARD — State globals
// =============================================================================
volatile bool walkRunning = false;
volatile bool walkStop = false;
int walkStepsTarget = 0;
int walkStepsDone = 0;

// =============================================================================
// WALK FORWARD — Diagonal trot gait loop
// variant 0: Basic    (poses  9+10 — your hip/femur values, tibia unchanged)
// variant 1: Enhanced (poses 11+12 — your values + tibia retraction)
// steps:   number of full gait cycles; 0 = run until walkStop is set
// speedMs: duration (ms) of each moveToPose() transition
// IMPORTANT: must be called AFTER the HTTP response has been sent so WiFi
//            stays alive during the potentially long blocking gait loop.
// =============================================================================
void walkForward(int steps, int speedMs, int variant) {
  int phaseA = (variant == 0) ? 9 : 11;  // FL+BR swing up
  int phaseB = (variant == 0) ? 10 : 12; // FR+BL swing up (FL+BR put down)

  walkRunning = true;
  walkStop = false;
  walkStepsDone = 0;

  int totalCycles = (steps == 0) ? INT_MAX : steps;
  for (int i = 0; i < totalCycles && !walkStop; i++) {
    moveToPose(phaseA, speedMs); // Phase A: FL+BR swing up
    if (walkStop)
      break;
    yield();

    moveToPose(1, speedMs); // Stand: all legs down
    if (walkStop)
      break;
    yield();

    moveToPose(phaseB, speedMs); // Phase B: FR+BL swing up
    if (walkStop)
      break;
    yield();

    moveToPose(1, speedMs); // Stand: all legs down
    if (walkStop)
      break;
    yield();

    walkStepsDone++;
    Serial.printf("[WALK] Step %d/%d done\n", walkStepsDone,
                  (steps == 0) ? -1 : steps);
  }

  // Always return to standing pose at end / stop
  moveToPose(1, 500);
  walkRunning = false;
  Serial.printf("[WALK] Complete. Steps=%d Variant=%s\n", walkStepsDone,
                variant == 0 ? "Basic" : "Enhanced");
}

// ─────────────────────────────────────────────────────────────────────────────
// GET /walk?steps=N&speed=X&variant=0|1   — start walk forward
// GET /walk?stop=1                        — stop currently running walk
// ─────────────────────────────────────────────────────────────────────────────
void hWalk() {
  // ── Stop request ──
  if (server.hasArg("stop")) {
    walkStop = true;
    addCORS(server);
    server.send(200, "application/json", "{\"ok\":true,\"msg\":\"stopping\"}");
    return;
  }

  if (walkRunning) {
    addCORS(server);
    server.send(200, "application/json",
                "{\"ok\":false,\"msg\":\"already walking\"}");
    return;
  }

  int steps = server.hasArg("steps") ? server.arg("steps").toInt() : 4;
  int speed = server.hasArg("speed")
                  ? constrain(server.arg("speed").toInt(), 100, 3000)
                  : 600;
  int variant = server.hasArg("variant")
                    ? constrain(server.arg("variant").toInt(), 0, 1)
                    : 0;
  walkStepsTarget = (steps < 0) ? 0 : steps;

  Serial.printf("[WALK] Requested: steps=%d speed=%dms variant=%s\n", steps,
                speed, variant == 0 ? "Basic" : "Enhanced");

  // ── Send HTTP 200 IMMEDIATELY so browser is unblocked ──
  addCORS(server);
  server.send(200, "application/json",
              "{\"ok\":true,\"msg\":\"walk started\"}");

  // ── Run walk loop AFTER response is sent ──
  walkForward(walkStepsTarget, speed, variant);
}

// ─────────────────────────────────────────────────────────────────────────────
// GET /walkstatus — returns current walk state for browser polling
// Response: {"running":bool, "steps":N, "target":N}
// ─────────────────────────────────────────────────────────────────────────────
void hWalkStatus() {
  addCORS(server);
  String j = "{\"running\":";
  j += walkRunning ? "true" : "false";
  j += ",\"steps\":";
  j += walkStepsDone;
  j += ",\"target\":";
  j += walkStepsTarget;
  j += "}";
  server.send(200, "application/json", j);
}

void hSweep() {
  sweepRequested = true;
  addCORS(server);
  server.send(200, "application/json",
              "{\"ok\":true,\"msg\":\"sweep started\"}");
}

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 3000)
    delay(10);

  Serial.println();
  Serial.println("=============================================");
  Serial.println("  FusionForce RUNNER4 — ESP32-S3");
  Serial.println("  Pose Presets + FK/IK  v3.0");
  Serial.println("  L1=30mm  L2=60mm  L3=80mm");
  Serial.println("  7 Poses: Standing/Crouch/FL-Up/FR-Up/TrotA/TrotB/Stretch");
  Serial.println("=============================================");
  Serial.printf("  I2C SDA -> GPIO%d\n", I2C_SDA);
  Serial.printf("  I2C SCL -> GPIO%d\n", I2C_SCL);

  // ── I2C + PCA9685 ──
  Wire.begin(I2C_SDA, I2C_SCL);
  i2cScan();

  Serial.println("Initialising PCA9685...");
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("Moving all servos -> STANDING pose...");
  setAllServosHome();
  delay(1500);

  // ── WiFi AP ──
  Serial.println();
  Serial.print("Starting WiFi AP: ");
  Serial.println(AP_SSID);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress apIP = WiFi.softAPIP();

  Serial.println("  WiFi AP started.");
  Serial.print("  SSID    : ");
  Serial.println(AP_SSID);
  Serial.print("  Password: ");
  Serial.println(AP_PASS);
  Serial.print("  URL     : http://");
  Serial.println(apIP);

  // ── HTTP Routes ──
  server.on("/", hRoot);
  server.on("/set", hSet);
  server.on("/state", hState);
  server.on("/home", hHome);
  server.on("/leghome", hLegHome);
  server.on("/ik", hIK);
  server.on("/pose", hPose);             // ← NEW: pose preset endpoint
  server.on("/savepose", hSavePose);     // ← save pose to RAM
  server.on("/walk", hWalk);             // ← NEW: walk forward gait
  server.on("/walkstatus", hWalkStatus); // ← NEW: walk status poll
  server.on("/sweep", hSweep);
  server.begin();

  Serial.println("  HTTP server running on port 80.");
  Serial.println();
  Serial.println("=============================================");
  Serial.println("  1. Connect WiFi: RUNNER4-Config");
  Serial.println("  2. Open browser: http://192.168.4.1");
  Serial.println("  3. LEFT SIDEBAR: click any pose preset");
  Serial.println("  4. PLAY TROT: cycles TrotA <-> TrotB");
  Serial.println("  5. Sliders + FK/IK still fully available");
  Serial.println("=============================================");
  Serial.println();
}

// =============================================================================
// LOOP
// =============================================================================
void loop() {
  server.handleClient();

  // Non-blocking sweep test on BR_FEMUR (CH8)
  if (sweepRequested) {
    sweepRequested = false;
    Serial.println("[SWEEP] BR FEMUR (CH8): 0 -> 90 -> 0");

    for (int a = 0; a <= 90; a++) {
      setServo(BR_FEMUR, a);
      server.handleClient();
      delay(15);
    }
    delay(200);
    for (int a = 90; a >= 0; a--) {
      setServo(BR_FEMUR, a);
      server.handleClient();
      delay(15);
    }
    delay(200);
    setLegHome(3);
    Serial.println("[SWEEP] Done.");
  }
}
