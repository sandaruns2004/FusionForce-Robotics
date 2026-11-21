// =============================================================================
// RGB_LED_Test.ino — ESP32-S3 RGB LED Test Code
//
// Pins (configured to match RUNNER4 robot pinout):
//   RED   LED -> GPIO 16 (via 330 ohm resistor)
//   GREEN LED -> GPIO 17 (via 330 ohm resistor)
//   BLUE  LED -> GPIO 18 (via 330 ohm resistor)
//   CATHODE   -> GND (for Common-Cathode LED or individual discrete LEDs)
//
// If using Common-Anode (+ to 3.3V), set COMMON_ANODE to true below.
//
// Features:
//   1. Automated test cycle: Red -> Green -> Blue -> Yellow -> Cyan -> Magenta -> White
//   2. Serial command control: Type 'r', 'g', 'b', 'w', 'y', 'c', 'm', 'off' in Serial Monitor
// =============================================================================

#include <Arduino.h>

#define LED_RED_PIN    16
#define LED_GREEN_PIN  17
#define LED_BLUE_PIN   18

// Set to true if you are using a Common-Anode LED (+ to 3.3V, LOW = ON)
// Set to false for Common-Cathode (- to GND, HIGH = ON) or 3 discrete LEDs
const bool COMMON_ANODE = false;

// Helper to write to an LED pin taking polarity into account
void writeLed(uint8_t pin, bool state) {
  if (COMMON_ANODE) {
    digitalWrite(pin, state ? LOW : HIGH);
  } else {
    digitalWrite(pin, state ? HIGH : LOW);
  }
}

// Set all three color channels simultaneously
void setColor(bool r, bool g, bool b) {
  writeLed(LED_RED_PIN,   r);
  writeLed(LED_GREEN_PIN, g);
  writeLed(LED_BLUE_PIN,  b);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n=============================================");
  Serial.println("   ESP32-S3 RGB LED Diagnostic Test");
  Serial.println("=============================================");
  Serial.printf("Pins: RED=GPIO%d, GREEN=GPIO%d, BLUE=GPIO%d\n",
                LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN);
  Serial.println("Type commands in Serial Monitor (115200 baud):");
  Serial.println("  'r'   -> Red only");
  Serial.println("  'g'   -> Green only");
  Serial.println("  'b'   -> Blue only");
  Serial.println("  'y'   -> Yellow (Red + Green)");
  Serial.println("  'c'   -> Cyan (Green + Blue)");
  Serial.println("  'm'   -> Magenta (Red + Blue)");
  Serial.println("  'w'   -> White (All ON)");
  Serial.println("  'off' -> Turn all OFF");
  Serial.println("  'auto'-> Resume automatic cycle");
  Serial.println("=============================================\n");

  pinMode(LED_RED_PIN,   OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN,  OUTPUT);

  // Start with all OFF
  setColor(false, false, false);
}

bool autoMode = true;
uint32_t lastStepMs = 0;
uint8_t currentStep = 0;

void loop() {
  // Check for serial user commands
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "r") {
      autoMode = false;
      setColor(true, false, false);
      Serial.println("[MANUAL] RED ON");
    } else if (cmd == "g") {
      autoMode = false;
      setColor(false, true, false);
      Serial.println("[MANUAL] GREEN ON");
    } else if (cmd == "b") {
      autoMode = false;
      setColor(false, false, true);
      Serial.println("[MANUAL] BLUE ON");
    } else if (cmd == "y") {
      autoMode = false;
      setColor(true, true, false);
      Serial.println("[MANUAL] YELLOW ON (R+G)");
    } else if (cmd == "c") {
      autoMode = false;
      setColor(false, true, true);
      Serial.println("[MANUAL] CYAN ON (G+B)");
    } else if (cmd == "m") {
      autoMode = false;
      setColor(true, false, true);
      Serial.println("[MANUAL] MAGENTA ON (R+B)");
    } else if (cmd == "w") {
      autoMode = false;
      setColor(true, true, true);
      Serial.println("[MANUAL] WHITE ON (R+G+B)");
    } else if (cmd == "off") {
      autoMode = false;
      setColor(false, false, false);
      Serial.println("[MANUAL] ALL OFF");
    } else if (cmd == "auto") {
      autoMode = true;
      currentStep = 0;
      lastStepMs = millis();
      Serial.println("[MODE] Resumed Automatic Cycling");
    } else if (cmd.length() > 0) {
      Serial.printf("Unknown command '%s'. Send 'r', 'g', 'b', 'w', 'off', or 'auto'.\n", cmd.c_str());
    }
  }

  // Automatic demo cycle every 1000 ms
  if (autoMode && (millis() - lastStepMs >= 1000)) {
    lastStepMs = millis();

    switch (currentStep) {
      case 0:
        Serial.println("[AUTO] 1. RED");
        setColor(true, false, false);
        break;
      case 1:
        Serial.println("[AUTO] 2. GREEN");
        setColor(false, true, false);
        break;
      case 2:
        Serial.println("[AUTO] 3. BLUE");
        setColor(false, false, true);
        break;
      case 3:
        Serial.println("[AUTO] 4. YELLOW (Red + Green)");
        setColor(true, true, false);
        break;
      case 4:
        Serial.println("[AUTO] 5. CYAN (Green + Blue)");
        setColor(false, true, true);
        break;
      case 5:
        Serial.println("[AUTO] 6. MAGENTA (Red + Blue)");
        setColor(true, false, true);
        break;
      case 6:
        Serial.println("[AUTO] 7. WHITE (All ON)");
        setColor(true, true, true);
        break;
      case 7:
        Serial.println("[AUTO] 8. OFF");
        setColor(false, false, false);
        break;
    }

    currentStep = (currentStep + 1) % 8;
  }
}