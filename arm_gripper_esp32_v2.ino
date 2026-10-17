/*
  ESP32 arm v2: gripper servo + lift (up/down) servo, with a full web control panel

  Library: "ESP32Servo"      Board: ESP32 Dev Module

  Wiring (unchanged):
    Gripper servo signal -> GPIO 18
    Lift servo signal    -> GPIO 19
    Both servos red      -> 5V power (external 5V supply recommended)
    Both servos brown    -> GND (shared with ESP32 GND)
    Optional button      -> GPIO 4 and GND (toggles open/close)

  Behaviour:
    - On power-up: the hand goes UP first, then the gripper CLOSES.
    - "down": the gripper OPENS first, then the hand goes down.
    - "pick":  open -> down -> grab -> up
    - "place": down (still gripping) -> release -> up -> close again
    - Angles, speeds and pause are editable on the web page and are
      saved in the ESP32's memory (they survive a power cut).

  Wi-Fi: network "ArmGripper", password "12345678", then open http://192.168.4.1

  Serial commands (115200 baud, "Newline"):
    open | close | up | down | pick | place | status
    g <deg>   raw gripper angle      l <deg>   raw lift angle
    gspeed <ms>   lspeed <ms>        (delay per degree; higher = slower)
*/

#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

const int GRIP_PIN   = 18;
const int LIFT_PIN   = 19;
const int BUTTON_PIN = 4;

// ---- Factory defaults (your calibrated values). The web page can change
// ---- them; "Reset" on the page restores these.
const int DEF_GRIP_OPEN  = 89;
const int DEF_GRIP_CLOSE = 53;
const int DEF_LIFT_UP    = 75;
const int DEF_LIFT_DOWN  = 160;
const int DEF_GRIP_DELAY = 15;   // ms per degree
const int DEF_LIFT_DELAY = 15;   // ms per degree
const int DEF_PAUSE      = 300;  // ms between steps in down / pick / place

const int LIFT_MIN = 20;         // hard safety limits for the lift servo
const int LIFT_MAX = 160;

const char* AP_SSID = "ArmGripper";
const char* AP_PASS = "12345678";

// ---- Live settings (loaded from memory at start-up) ----
int GRIP_OPEN  = DEF_GRIP_OPEN;
int GRIP_CLOSE = DEF_GRIP_CLOSE;
int LIFT_UP    = DEF_LIFT_UP;
int LIFT_DOWN  = DEF_LIFT_DOWN;
int gripDelay  = DEF_GRIP_DELAY;
int liftDelay  = DEF_LIFT_DELAY;
int stepPause  = DEF_PAUSE;

Servo grip, lift;
WebServer server(80);
Preferences prefs;

int gripPos = 90;
int liftPos = 90;
bool gripIsOpen = false;
bool lastBtn = HIGH;

// ---------- saved settings ----------
void loadSettings() {
  prefs.begin("arm", false);
  GRIP_OPEN  = constrain(prefs.getInt("gopen",  DEF_GRIP_OPEN),  0, 180);
  GRIP_CLOSE = constrain(prefs.getInt("gclose", DEF_GRIP_CLOSE), 0, 180);
  LIFT_UP    = constrain(prefs.getInt("lup",    DEF_LIFT_UP),    LIFT_MIN, LIFT_MAX);
  LIFT_DOWN  = constrain(prefs.getInt("ldown",  DEF_LIFT_DOWN),  LIFT_MIN, LIFT_MAX);
  gripDelay  = constrain(prefs.getInt("gs",     DEF_GRIP_DELAY), 1, 100);
  liftDelay  = constrain(prefs.getInt("ls",     DEF_LIFT_DELAY), 1, 100);
  stepPause  = constrain(prefs.getInt("pause",  DEF_PAUSE),      0, 3000);
  prefs.end();
}

void saveSetting(const char* key, int v) {
  prefs.begin("arm", false);
  prefs.putInt(key, v);
  prefs.end();
}

void resetDefaults() {
  GRIP_OPEN  = DEF_GRIP_OPEN;
  GRIP_CLOSE = DEF_GRIP_CLOSE;
  LIFT_UP    = DEF_LIFT_UP;
  LIFT_DOWN  = DEF_LIFT_DOWN;
  gripDelay  = DEF_GRIP_DELAY;
  liftDelay  = DEF_LIFT_DELAY;
  stepPause  = DEF_PAUSE;
  prefs.begin("arm", false);
  prefs.clear();
  prefs.end();
}

// ---------- movement ----------
void moveServo(Servo &s, int &cur, int target, int d, int lo, int hi) {
  target = constrain(target, lo, hi);
  int dir = (target > cur) ? 1 : -1;
  while (cur != target) {
    cur += dir;
    s.write(cur);
    delay(d);
  }
}

void moveGrip(int a) { moveServo(grip, gripPos, a, gripDelay, 0, 180); }
void moveLift(int a) { moveServo(lift, liftPos, a, liftDelay, LIFT_MIN, LIFT_MAX); }

void pauseStep() { delay(stepPause); }

void openHand()  { moveGrip(GRIP_OPEN);  gripIsOpen = true;  }
void closeHand() { moveGrip(GRIP_CLOSE); gripIsOpen = false; }
void handUp()    { moveLift(LIFT_UP); }

void handDown() {          // gripper opens FIRST, then the hand goes down
  openHand();
  pauseStep();
  moveLift(LIFT_DOWN);
}

void pickUp() {            // open -> down -> grab -> up
  handDown();
  pauseStep();
  closeHand();
  pauseStep();
  handUp();
}

void placeDown() {         // down (still gripping) -> release -> up -> close
  moveLift(LIFT_DOWN);
  pauseStep();
  openHand();
  pauseStep();
  handUp();
  pauseStep();
  closeHand();
}

// ---------- web helpers ----------
String statusJson() {
  char b[256];
  snprintf(b, sizeof(b),
    "{\"g\":%d,\"l\":%d,\"open\":%s,\"gopen\":%d,\"gclose\":%d,\"lup\":%d,\"ldown\":%d,"
    "\"gs\":%d,\"ls\":%d,\"pause\":%d,\"lmin\":%d,\"lmax\":%d}",
    gripPos, liftPos, gripIsOpen ? "true" : "false",
    GRIP_OPEN, GRIP_CLOSE, LIFT_UP, LIFT_DOWN,
    gripDelay, liftDelay, stepPause, LIFT_MIN, LIFT_MAX);
  return String(b);
}

void sendStatus() { server.send(200, "application/json", statusJson()); }

bool applySetting(const String& k, int v, bool preview) {
  if (k == "gopen") {
    GRIP_OPEN = constrain(v, 0, 180);
    saveSetting("gopen", GRIP_OPEN);
    if (preview) openHand();
  } else if (k == "gclose") {
    GRIP_CLOSE = constrain(v, 0, 180);
    saveSetting("gclose", GRIP_CLOSE);
    if (preview) closeHand();
  } else if (k == "lup") {
    LIFT_UP = constrain(v, LIFT_MIN, LIFT_MAX);
    saveSetting("lup", LIFT_UP);
    if (preview) handUp();
  } else if (k == "ldown") {
    LIFT_DOWN = constrain(v, LIFT_MIN, LIFT_MAX);
    saveSetting("ldown", LIFT_DOWN);
    if (preview) moveLift(LIFT_DOWN);
  } else if (k == "gs") {
    gripDelay = constrain(v, 1, 100);
    saveSetting("gs", gripDelay);
  } else if (k == "ls") {
    liftDelay = constrain(v, 1, 100);
    saveSetting("ls", liftDelay);
  } else if (k == "pause") {
    stepPause = constrain(v, 0, 3000);
    saveSetting("pause", stepPause);
  } else {
    return false;
  }
  return true;
}

const char PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Arm</title>
<style>
:root{--bg:#10151c;--card:#1a222d;--line:#2b3644;--text:#e8eef5;--mute:#8da0b5;--blue:#3d8bfd;--green:#2fb36d;--amber:#f0a93a;--red:#e5534b}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--text);font-family:system-ui,sans-serif;max-width:560px;margin-inline:auto}
h1{font-size:20px;margin:4px 0 12px;display:flex;justify-content:space-between;align-items:center}
.chip{font-size:12px;padding:4px 12px;border-radius:99px;background:var(--line);color:var(--mute);font-weight:600}
body.busy .chip{background:var(--amber);color:#000}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px;margin-bottom:12px}
body.busy .card button,body.busy .card input{pointer-events:none;opacity:.55}
.viz{width:100%;height:auto;display:block;background:var(--bg);border-radius:10px}
.read{display:flex;justify-content:space-around;margin-top:10px;font-size:14px;color:var(--mute)}
.read b{display:block;color:var(--text);font-size:16px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
button.big{padding:16px;border:0;border-radius:12px;font-size:17px;font-weight:600;color:#fff;background:var(--blue);transition:transform .1s}
button.big:active{transform:scale(.96)}
.go{background:var(--green)}
.danger{background:var(--red);width:100%}
h3{margin:0 0 8px;font-size:13px;color:var(--mute);text-transform:uppercase;letter-spacing:.06em}
details summary{cursor:pointer;font-weight:600;font-size:16px}
.row{margin:12px 0}
.lab{display:flex;justify-content:space-between;font-size:14px}
.lab output{color:var(--blue);font-weight:700}
.ctl{display:flex;align-items:center;gap:8px;margin-top:6px}
.ctl input{flex:1;accent-color:var(--blue);height:28px}
.mini{width:40px;height:40px;border-radius:10px;border:1px solid var(--line);background:var(--bg);color:var(--text);font-size:20px}
.hint{font-size:12px;color:var(--mute);margin-top:2px}
.tog{display:flex;gap:8px;align-items:center;font-size:14px;margin:10px 0}
#toast{position:fixed;left:50%;bottom:20px;transform:translate(-50%,80px);background:#000c;color:#fff;padding:10px 18px;border-radius:10px;transition:transform .25s;font-size:14px}
#toast.show{transform:translate(-50%,0)}
</style></head><body>
<h1>Robot Arm <span class="chip" id="state">Connecting...</span></h1>

<div class="card">
<svg class="viz" viewBox="0 0 240 160">
  <rect x="8" y="88" width="30" height="60" rx="5" fill="#2b3644"/>
  <g id="arm" style="transform-origin:30px 80px;transition:transform .5s ease">
    <rect x="30" y="74" width="118" height="12" rx="5" fill="#3d8bfd"/>
    <g transform="translate(148,80)">
      <rect id="f1" x="0" y="-9" width="34" height="8" rx="3" fill="#f0a93a" style="transition:transform .4s"/>
      <rect id="f2" x="0" y="1" width="34" height="8" rx="3" fill="#f0a93a" style="transition:transform .4s"/>
    </g>
  </g>
  <circle cx="30" cy="80" r="9" fill="#e8eef5"/>
</svg>
<div class="read"><div>Gripper<b id="gv">-</b></div><div>Lift<b id="lv">-</b></div></div>
</div>

<div class="card">
<h3>Controls</h3>
<div class="grid">
  <button class="big" onclick="api('/open','Opening...')">OPEN</button>
  <button class="big" onclick="api('/close','Closing...')">CLOSE</button>
  <button class="big" onclick="api('/up','Going up...')">UP</button>
  <button class="big" onclick="api('/down','Opening, then down...')">DOWN</button>
  <button class="big go" onclick="api('/pick','Picking up...')">PICK UP</button>
  <button class="big go" onclick="api('/place','Placing...')">PLACE</button>
</div>
</div>

<div class="card"><details open><summary>Angles</summary>
<label class="tog"><input type="checkbox" id="prev" checked> Move the servo while I adjust (preview)</label>
<div id="ang"></div>
<div class="hint">Changes are saved automatically. Use the - and + buttons for 1 degree steps.</div>
</details></div>

<div class="card"><details><summary>Speed &amp; delay</summary>
<div id="spd"></div>
</details></div>

<div class="card"><details><summary>Manual control</summary>
<div id="man"></div>
</details></div>

<button class="big danger" onclick="if(confirm('Restore the default angles and speeds?'))api('/reset','Resetting...')">Reset to defaults</button>
<div id="toast"></div>

<script>
const $=id=>document.getElementById(id);
const D='\u00b0';
const U={};
let chain=Promise.resolve(),pending=0,tt;

function toast(m){const t=$('toast');t.textContent=m;t.classList.add('show');clearTimeout(tt);tt=setTimeout(()=>t.classList.remove('show'),2200);}
function api(path,msg){chain=chain.then(()=>run(path,msg));return chain;}
async function run(path,msg){
  pending++;document.body.classList.add('busy');$('state').textContent=msg||'Working...';
  try{const r=await fetch(path);if(!r.ok)throw new Error('bad');render(await r.json());}
  catch(e){toast('No connection to the robot');}
  pending--;
  if(!pending){document.body.classList.remove('busy');$('state').textContent='Ready';}
}

function row(box,id,label,unit,min,max,step,hint,cb){
  U[id]=unit;
  const d=document.createElement('div');d.className='row';
  d.innerHTML='<div class="lab"><span>'+label+'</span><output id="o_'+id+'">-</output></div>'+
    '<div class="ctl"><button class="mini">&minus;</button>'+
    '<input type="range" id="r_'+id+'" min="'+min+'" max="'+max+'" step="'+step+'" value="'+min+'">'+
    '<button class="mini">+</button></div>'+(hint?'<div class="hint">'+hint+'</div>':'');
  box.appendChild(d);
  const r=d.querySelector('input'),o=d.querySelector('output'),b=d.querySelectorAll('button');
  const show=()=>{o.textContent=r.value+unit;};
  r.oninput=show;
  r.onchange=()=>cb(r.value);
  b[0].onclick=()=>{r.value=+r.value-step;show();cb(r.value);};
  b[1].onclick=()=>{r.value=+r.value+step;show();cb(r.value);};
}
function send(k,v,ang){
  const p=ang&&$('prev').checked;
  api('/set?k='+k+'&v='+v+'&p='+(p?1:0),p?'Moving...':'Saving...');
}

[['gopen','Gripper OPEN angle'],['gclose','Gripper CLOSE angle'],['lup','Lift UP angle'],['ldown','Lift DOWN angle']]
  .forEach(([k,l])=>row($('ang'),k,l,D,0,180,1,'',v=>send(k,v,true)));
row($('spd'),'gs','Gripper delay per degree',' ms',1,50,1,'Lower number = faster',v=>send('gs',v,false));
row($('spd'),'ls','Lift delay per degree',' ms',1,50,1,'Lower number = faster',v=>send('ls',v,false));
row($('spd'),'pause','Pause between steps',' ms',0,2000,50,'Wait between actions in down, pick and place',v=>send('pause',v,false));
row($('man'),'g','Gripper angle',D,0,180,1,'',v=>api('/g?v='+v,'Moving...'));
row($('man'),'l','Lift angle',D,20,160,1,'',v=>api('/l?v='+v,'Moving...'));

function setUI(k,v){
  const r=$('r_'+k);
  if(!r||document.activeElement===r)return;
  r.value=v;$('o_'+k).textContent=v+U[k];
}
function render(s){
  ['lup','ldown','l'].forEach(k=>{const r=$('r_'+k);r.min=s.lmin;r.max=s.lmax;});
  ['gopen','gclose','lup','ldown','gs','ls','pause'].forEach(k=>setUI(k,s[k]));
  setUI('g',s.g);setUI('l',s.l);
  const cl=(x,a,b)=>Math.min(b,Math.max(a,x));
  const tg=cl((s.g-s.gclose)/((s.gopen-s.gclose)||1),0,1);
  const tl=cl((s.l-s.lup)/((s.ldown-s.lup)||1),0,1);
  $('arm').style.transform='rotate('+(-25+50*tl)+'deg)';
  const gap=22*tg;
  $('f1').style.transform='translateY('+(-gap/2)+'px)';
  $('f2').style.transform='translateY('+(gap/2)+'px)';
  $('gv').textContent=s.g+D+(s.g===s.gopen?' (open)':s.g===s.gclose?' (closed)':'');
  $('lv').textContent=s.l+D+(s.l===s.lup?' (up)':s.l===s.ldown?' (down)':'');
}

api('/status','Connecting...');
setInterval(()=>{if(!pending)api('/status');},4000);
</script>
</body></html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  loadSettings();

  grip.setPeriodHertz(50);
  lift.setPeriodHertz(50);

  // Start-up pose: hand goes UP first, then the gripper CLOSES.
  liftPos = LIFT_UP;
  gripPos = GRIP_CLOSE;
  gripIsOpen = false;
  lift.attach(LIFT_PIN, 500, 2400);
  lift.write(liftPos);
  delay(800);
  grip.attach(GRIP_PIN, 500, 2400);
  grip.write(gripPos);
  delay(500);

  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/status", []() { sendStatus(); });
  server.on("/open",  []() { openHand();   sendStatus(); });
  server.on("/close", []() { closeHand();  sendStatus(); });
  server.on("/up",    []() { handUp();     sendStatus(); });
  server.on("/down",  []() { handDown();   sendStatus(); });
  server.on("/pick",  []() { pickUp();     sendStatus(); });
  server.on("/place", []() { placeDown();  sendStatus(); });
  server.on("/g", []() { moveGrip(server.arg("v").toInt()); sendStatus(); });
  server.on("/l", []() { moveLift(server.arg("v").toInt()); sendStatus(); });
  server.on("/set", []() {
    String k = server.arg("k");
    int v = server.arg("v").toInt();
    bool p = (server.arg("p") == "1");
    if (!applySetting(k, v, p)) {
      server.send(400, "text/plain", "bad setting");
      return;
    }
    sendStatus();
  });
  server.on("/reset", []() { resetDefaults(); sendStatus(); });
  server.begin();

  Serial.print("Ready. Wi-Fi: "); Serial.print(AP_SSID);
  Serial.print("  IP: "); Serial.println(WiFi.softAPIP());
}

void loop() {
  server.handleClient();

  bool btn = digitalRead(BUTTON_PIN);
  if (lastBtn == HIGH && btn == LOW) {
    gripIsOpen ? closeHand() : openHand();
    delay(50);
  }
  lastBtn = btn;

  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim(); cmd.toLowerCase();

    if (cmd == "open")        openHand();
    else if (cmd == "close")  closeHand();
    else if (cmd == "up")     handUp();
    else if (cmd == "down")   handDown();
    else if (cmd == "pick")   pickUp();
    else if (cmd == "place")  placeDown();
    else if (cmd.startsWith("g "))      moveGrip(cmd.substring(2).toInt());
    else if (cmd.startsWith("l "))      moveLift(cmd.substring(2).toInt());
    else if (cmd.startsWith("gspeed ")) gripDelay = max(1, (int)cmd.substring(7).toInt());
    else if (cmd.startsWith("lspeed ")) liftDelay = max(1, (int)cmd.substring(7).toInt());
    else if (cmd != "status") { Serial.println("Unknown command"); return; }

    Serial.print("Gripper: "); Serial.print(gripPos);
    Serial.print("  Lift: ");  Serial.println(liftPos);
  }
}
