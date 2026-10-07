import re

filepath = r"c:\Users\ADMIN\Desktop\Projects\FusionForce-Robotics\Testing codes\Combined_Controller\Combined_Controller.ino"
with open(filepath, "r", encoding="utf-8") as f:
    content = f.read()

# 1. Add #include
content = content.replace(
    "#include <Wire.h>",
    "#include <Wire.h>\n#include \"Adafruit_TCS34725.h\""
)

# 2. Add button in HTML
content = content.replace(
    "<button class=\"btn btn-sw\" onclick=\"doSweep()\">&#9654; Sweep</button>",
    "<button class=\"btn btn-sw\" onclick=\"doSweep()\">&#9654; Sweep</button>\n    <button class=\"btn btn-sw\" onclick=\"window.location.href='/color'\">🎨 Color Sensor</button>"
)

# 3. Add Color Sensor code before hSweep()
color_code = """
// =============================================================================
// COLOR SENSOR INTEGRATION
// =============================================================================
#define CS_SDA_PIN 11
#define CS_SCL_PIN 12
#define CS_LED_PIN 4

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

enum ColorID { COLOR_UNKNOWN, COLOR_RED, COLOR_GREEN, COLOR_BLUE };
uint16_t rawR = 0, rawG = 0, rawB = 0, rawC = 0;
float normR = 0, normG = 0, normB = 0;
ColorID detectedColor = COLOR_UNKNOWN;
float redThreshold = 0.40, greenThreshold = 0.35, blueThreshold = 0.30, dominanceRatio = 1.20;
unsigned long sampleInterval = 500, lastSample = 0;
bool sensorLED = true;

String getColorName(ColorID id) {
  switch (id) {
    case COLOR_RED: return "RED";
    case COLOR_GREEN: return "GREEN";
    case COLOR_BLUE: return "BLUE";
    default: return "UNKNOWN";
  }
}

ColorID classifyColor(uint16_t r, uint16_t g, uint16_t b, uint16_t c) {
  if (c == 0) return COLOR_UNKNOWN;
  float r_n = (float)r / c;
  float g_n = (float)g / c;
  float b_n = (float)b / c;
  if (r_n > redThreshold && r_n > g_n * dominanceRatio && r_n > b_n * dominanceRatio) return COLOR_RED;
  if (g_n > greenThreshold && g_n > r_n * dominanceRatio && g_n > b_n * dominanceRatio) return COLOR_GREEN;
  if (b_n > blueThreshold && b_n > r_n * dominanceRatio && b_n > g_n * dominanceRatio) return COLOR_BLUE;
  return COLOR_UNKNOWN;
}

void readSensor() {
  if (sensorLED) { digitalWrite(CS_LED_PIN, HIGH); delay(5); }
  tcs.getRawData(&rawR, &rawG, &rawB, &rawC);
  if (sensorLED) { digitalWrite(CS_LED_PIN, LOW); }
  if (rawC > 0) {
    normR = (float)rawR / rawC; normG = (float)rawG / rawC; normB = (float)rawB / rawC;
  } else {
    normR = 0; normG = 0; normB = 0;
  }
  detectedColor = classifyColor(rawR, rawG, rawB, rawC);
}

// ── COLOR HTML ──
const char COLOR_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>TCS34725 Color Lab</title>
<style>
* { box-sizing: border-box; }
body { margin: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Arial; background: linear-gradient(135deg, #0f172a, #111827, #020617); color: #f8fafc; min-height: 100vh; }
.container { max-width: 1100px; margin: auto; padding: 25px; }
.header { text-align: center; margin-bottom: 25px; }
.header h1 { margin: 0; font-size: 32px; }
.header p { color: #94a3b8; margin-top: 8px; }
.card { background: rgba(30,41,59,0.75); border: 1px solid rgba(148,163,184,0.15); border-radius: 18px; padding: 22px; margin-bottom: 20px; box-shadow: 0 10px 30px rgba(0,0,0,0.25); }
.grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(250px, 1fr)); gap: 18px; }
.big-color { min-height: 180px; display: flex; flex-direction: column; align-items: center; justify-content: center; border-radius: 18px; background: #1e293b; transition: 0.3s; }
.color-name { font-size: 38px; font-weight: 800; margin-top: 10px; }
.color-circle { width: 80px; height: 80px; border-radius: 50%; background: #64748b; box-shadow: 0 0 35px rgba(255,255,255,0.15); }
.metric { background: #0f172a; border-radius: 14px; padding: 16px; }
.metric-title { color: #94a3b8; font-size: 14px; }
.metric-value { font-size: 28px; font-weight: 700; margin-top: 5px; }
.bar-container { height: 12px; background: #020617; border-radius: 10px; overflow: hidden; margin-top: 10px; }
.bar { height: 100%; width: 0%; border-radius: 10px; transition: width 0.2s; }
.red-bar { background: #ef4444; } .green-bar { background: #22c55e; } .blue-bar { background: #3b82f6; }
input[type=range] { width: 100%; }
.control { margin-bottom: 20px; }
.control-header { display: flex; justify-content: space-between; margin-bottom: 8px; }
.value { font-weight: 700; color: #38bdf8; }
button { width: 100%; padding: 13px; border: none; border-radius: 10px; background: #2563eb; color: white; font-size: 15px; font-weight: 600; cursor: pointer; margin-top: 8px; }
button:hover { background: #1d4ed8; }
.btn-back { background: #334155; margin-bottom: 20px; width: auto; padding: 10px 20px; display: inline-block; text-decoration: none; color: white; font-weight: bold; border-radius: 10px; }
.btn-back:hover { background: #475569; }
.status { display: inline-block; padding: 6px 12px; border-radius: 20px; background: #166534; color: #bbf7d0; font-size: 13px; }
table { width: 100%; border-collapse: collapse; }
td, th { padding: 10px; text-align: left; border-bottom: 1px solid #334155; }
th { color: #94a3b8; }
footer { text-align: center; color: #64748b; padding: 20px; }
</style>
</head>
<body>
<div class="container">
<a href="/" class="btn-back">⬅ Back to 12 DOF Robot</a>
<div class="header">
<h1>🎨 TCS34725 Color Lab</h1>
<p>ESP32-S3 Color Sensor Calibration & Testing Dashboard</p>
<span class="status">● SENSOR ONLINE</span>
</div>

<div class="card">
<div class="big-color">
<div id="colorCircle" class="color-circle"></div>
<div id="colorName" class="color-name">UNKNOWN</div>
</div>
</div>

<div class="card">
<h2>📊 Raw Sensor Data</h2>
<div class="grid">
<div class="metric"><div class="metric-title">RED</div><div id="rawR" class="metric-value">0</div></div>
<div class="metric"><div class="metric-title">GREEN</div><div id="rawG" class="metric-value">0</div></div>
<div class="metric"><div class="metric-title">BLUE</div><div id="rawB" class="metric-value">0</div></div>
<div class="metric"><div class="metric-title">CLEAR</div><div id="rawC" class="metric-value">0</div></div>
</div>
</div>

<div class="card">
<h2>🌈 Normalized RGB</h2>
<div class="grid">
<div><b>Red</b><div id="nr">0.000</div><div class="bar-container"><div id="rb" class="bar red-bar"></div></div></div>
<div><b>Green</b><div id="ng">0.000</div><div class="bar-container"><div id="gb" class="bar green-bar"></div></div></div>
<div><b>Blue</b><div id="nb">0.000</div><div class="bar-container"><div id="bb" class="bar blue-bar"></div></div></div>
</div>
</div>

<div class="card">
<h2>⚙️ Classification Tuning</h2>
<div class="control">
<div class="control-header"><span>Red Threshold</span><span id="redValue" class="value">0.40</span></div>
<input type="range" min="0" max="1" step="0.01" value="0.40" id="redSlider" oninput="updateSetting('red',this.value)">
</div>
<div class="control">
<div class="control-header"><span>Green Threshold</span><span id="greenValue" class="value">0.35</span></div>
<input type="range" min="0" max="1" step="0.01" value="0.35" id="greenSlider" oninput="updateSetting('green',this.value)">
</div>
<div class="control">
<div class="control-header"><span>Blue Threshold</span><span id="blueValue" class="value">0.30</span></div>
<input type="range" min="0" max="1" step="0.01" value="0.30" id="blueSlider" oninput="updateSetting('blue',this.value)">
</div>
<div class="control">
<div class="control-header"><span>Dominance Ratio</span><span id="dominanceValue" class="value">1.20</span></div>
<input type="range" min="1" max="3" step="0.01" value="1.20" id="dominanceSlider" oninput="updateSetting('dominance',this.value)">
</div>
</div>

<div class="card">
<h2>💡 Sensor Control</h2>
<div class="grid">
<div><button onclick="toggleLED()">Toggle Sensor LED</button></div>
<div><button onclick="readNow()">Read Sensor Now</button></div>
</div>
<p>LED: <span id="ledStatus">ON</span></p>
</div>

<div class="card">
<h2>🔬 Sensor Information</h2>
<table>
<tr><th>Parameter</th><th>Value</th></tr>
<tr><td>Integration Time</td><td>50 ms</td></tr>
<tr><td>Gain</td><td>4X</td></tr>
<tr><td>I²C SDA</td><td>GPIO 11 (Wire1)</td></tr>
<tr><td>I²C SCL</td><td>GPIO 12 (Wire1)</td></tr>
<tr><td>Sampling Interval</td><td id="interval">500 ms</td></tr>
</table>
</div>
<footer>ESP32-S3 • TCS34725 Color Laboratory</footer>
</div>

<script>
function updateData() {
  fetch('/colordata').then(response => response.json()).then(data => {
    document.getElementById('rawR').innerText = data.r;
    document.getElementById('rawG').innerText = data.g;
    document.getElementById('rawB').innerText = data.b;
    document.getElementById('rawC').innerText = data.c;
    document.getElementById('nr').innerText = data.nr.toFixed(3);
    document.getElementById('ng').innerText = data.ng.toFixed(3);
    document.getElementById('nb').innerText = data.nb.toFixed(3);
    document.getElementById('rb').style.width = Math.min(data.nr * 100,100) + "%";
    document.getElementById('gb').style.width = Math.min(data.ng * 100,100) + "%";
    document.getElementById('bb').style.width = Math.min(data.nb * 100,100) + "%";
    document.getElementById('colorName').innerText = data.color;
    let circle = document.getElementById('colorCircle');
    if(data.color === "RED") { circle.style.background = "#ef4444"; circle.style.boxShadow = "0 0 40px #ef4444"; }
    else if(data.color === "GREEN") { circle.style.background = "#22c55e"; circle.style.boxShadow = "0 0 40px #22c55e"; }
    else if(data.color === "BLUE") { circle.style.background = "#3b82f6"; circle.style.boxShadow = "0 0 40px #3b82f6"; }
    else { circle.style.background = "#64748b"; circle.style.boxShadow = "0 0 20px #64748b"; }
  });
}

function updateSetting(type,value) {
  if(type === "red") document.getElementById("redValue").innerText = parseFloat(value).toFixed(2);
  if(type === "green") document.getElementById("greenValue").innerText = parseFloat(value).toFixed(2);
  if(type === "blue") document.getElementById("blueValue").innerText = parseFloat(value).toFixed(2);
  if(type === "dominance") document.getElementById("dominanceValue").innerText = parseFloat(value).toFixed(2);
  fetch("/colorset?" + type + "=" + value);
}

function toggleLED() {
  fetch("/colorled").then(() => {
    let status = document.getElementById("ledStatus");
    status.innerText = status.innerText === "ON" ? "OFF" : "ON";
  });
}

function readNow() {
  fetch("/colorread").then(() => updateData());
}

setInterval(updateData, 300);
updateData();
</script>
</body>
</html>
)rawliteral";

void hColorRoot() { server.send(200, "text/html", COLOR_PAGE); }
void hColorData() {
  String json = "{";
  json += "\\"r\\":" + String(rawR) + ",\\"g\\":" + String(rawG) + ",\\"b\\":" + String(rawB) + ",\\"c\\":" + String(rawC) + ",";
  json += "\\"nr\\":" + String(normR, 4) + ",\\"ng\\":" + String(normG, 4) + ",\\"nb\\":" + String(normB, 4) + ",";
  json += "\\"color\\":\\"" + getColorName(detectedColor) + "\\"}";
  server.send(200, "application/json", json);
}
void hColorSet() {
  if (server.hasArg("red")) redThreshold = server.arg("red").toFloat();
  if (server.hasArg("green")) greenThreshold = server.arg("green").toFloat();
  if (server.hasArg("blue")) blueThreshold = server.arg("blue").toFloat();
  if (server.hasArg("dominance")) dominanceRatio = server.arg("dominance").toFloat();
  detectedColor = classifyColor(rawR, rawG, rawB, rawC);
  server.send(200, "text/plain", "OK");
}
void hColorLed() {
  sensorLED = !sensorLED;
  digitalWrite(CS_LED_PIN, sensorLED ? HIGH : LOW);
  server.send(200, "text/plain", sensorLED ? "ON" : "OFF");
}
void hColorRead() {
  readSensor();
  server.send(200, "text/plain", "READ");
}

void hSweep() {
"""
content = content.replace("void hSweep() {", color_code)

# 4. Setup I2C and Color Sensor
setup_code = """  // ── I2C + PCA9685 ──
  Wire.begin(I2C_SDA, I2C_SCL);
  i2cScan();

  // ── TCS34725 COLOR SENSOR (Wire1) ──
  Wire1.begin(CS_SDA_PIN, CS_SCL_PIN);
  pinMode(CS_LED_PIN, OUTPUT);
  digitalWrite(CS_LED_PIN, LOW);
  if (!tcs.begin(0x29, &Wire1)) {
    Serial.println("  WARNING: TCS34725 not found on Wire1! Check wiring.");
  } else {
    Serial.println("  TCS34725 initialized on Wire1.");
  }
"""
content = content.replace("  // ── I2C + PCA9685 ──\n  Wire.begin(I2C_SDA, I2C_SCL);\n  i2cScan();", setup_code)

# 5. Route server
route_code = """  server.on("/sweep", hSweep);
  
  // Color Sensor routes
  server.on("/color", hColorRoot);
  server.on("/colordata", hColorData);
  server.on("/colorset", hColorSet);
  server.on("/colorled", hColorLed);
  server.on("/colorread", hColorRead);
"""
content = content.replace("  server.on(\"/sweep\", hSweep);", route_code)

# 6. Loop run
loop_code = """  server.handleClient();

  if (millis() - lastSample >= sampleInterval) {
    lastSample = millis();
    readSensor();
  }
"""
content = content.replace("  server.handleClient();", loop_code)

# Change WiFi AP name
content = content.replace("RUNNER4-Config", "RUNNER4-Combined")

with open(filepath, "w", encoding="utf-8") as f:
    f.write(content)
print("done")
