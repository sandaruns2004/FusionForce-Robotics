# RUNNER4 ESP32-S3 Complete Pinout & Wiring Map

This document defines the complete hardware pin allocation for the **RUNNER4 Quadruped Robot** running on the **ESP32-S3** controller (`RUNNER4_Robot_Line_03`).

It covers:
1. **9-channel TCRT5000 line sensor array** (ADC1 only — WiFi safe)
2. **Two I2C buses** (PCA9685 + VL53L0X on GPIO 11/12; TCS34725 on GPIO 41/42)
3. **VL53L0X ToF distance sensors & XSHUT wiring** (Front=GPIO 13, Right=GPIO 14, Left=GPIO 21)
4. **2-DOF Robotic Arm servos** (GPIO 47 Lift + GPIO 44 Grip direct LEDC PWM)
5. **TCS34725 color sensor LED control** (GPIO 15)
6. **4-pin status RGB LED** (GPIO 16, 17, 18 common cathode)
7. **3× Hardware pushbuttons** (GPIO 38, 39, 40 with internal pull-up)
8. **Full ESP32-S3 DevKit header pinout diagram**

---

## 1. Master GPIO Assignment Table

| ESP32-S3 GPIO | Function / Signal | Hardware Component | Mode / Type | Notes |
| :---: | :--- | :--- | :---: | :--- |
| **1** | Line S1 (Far Right / Rightmost) | 9× TCRT5000 Array | Analog In | ADC1_CH0 (WiFi AP safe) |
| **2** | Line S2 | 9× TCRT5000 Array | Analog In | ADC1_CH1 (WiFi AP safe) |
| **4** | Line S3 | 9× TCRT5000 Array | Analog In | ADC1_CH3 (WiFi AP safe) |
| **5** | Line S4 | 9× TCRT5000 Array | Analog In | ADC1_CH4 (WiFi AP safe) |
| **6** | Line S5 (Center) | 9× TCRT5000 Array | Analog In | ADC1_CH5 (WiFi AP safe) |
| **7** | Line S6 | 9× TCRT5000 Array | Analog In | ADC1_CH6 (WiFi AP safe) |
| **8** | Line S7 | 9× TCRT5000 Array | Analog In | ADC1_CH7 (WiFi AP safe) |
| **9** | Line S8 | 9× TCRT5000 Array | Analog In | ADC1_CH8 (WiFi AP safe) |
| **10** | Line S9 (Far Left / Leftmost) | 9× TCRT5000 Array | Analog In | ADC1_CH9 (WiFi AP safe) |
| **11** | I2C SDA0 | PCA9685 + VL53L0X | I2C Data | 4.7kΩ pull-up to 3.3V |
| **12** | I2C SCL0 | PCA9685 + VL53L0X | I2C Clock | 4.7kΩ pull-up to 3.3V, 400kHz |
| **13** | **VL53L0X Front XSHUT** | Front ToF | Digital Out | Active LOW reset; sensor remapped to `0x30` |
| **14** | **VL53L0X Right XSHUT** | Right ToF | Digital Out | Active LOW reset; sensor remapped to `0x31` |
| **21** | **VL53L0X Left XSHUT** | Left ToF | Digital Out | Active LOW reset; sensor remapped to `0x32` |
| **47** | **ARM Lift Servo PWM** | Arm lift servo (direct) | PWM Out (LEDC ch 6) | Free GPIO on right header (pin 17). Direct servo signal to lift servo. |
| **44** | **ARM Grip Servo PWM** | Arm gripper servo (direct) | PWM Out (LEDC ch 7) | Right header pin 3 (U0RXD). Free when USB CDC is used. (Alternate: GPIO 43) |
| **15** | TCS34725 LED Control | TCS34725 Colour Sensor | Digital Out | HIGH = white LED on, LOW = off |
| **16** | Status LED — Red | 4-Pin Common-Cathode RGB | Digital Out | Connect via 330Ω resistor |
| **17** | Status LED — Green | 4-Pin Common-Cathode RGB | Digital Out | Connect via 330Ω resistor |
| **18** | Status LED — Blue | 4-Pin Common-Cathode RGB | Digital Out | Connect via 330Ω resistor |
| **38** | **Button 1 (SW1)** | Pushbutton (e.g. Start / Stop) | Digital In | `INPUT_PULLUP` (Press = LOW to GND) |
| **39** | **Button 2 (SW2)** | Pushbutton (e.g. Calibrate / Stand) | Digital In | `INPUT_PULLUP` (Press = LOW to GND) |
| **40** | **Button 3 (SW3)** | Pushbutton (e.g. Mode / Home) | Digital In | `INPUT_PULLUP` (Press = LOW to GND) |
| **41** | I2C SDA1 | TCS34725 colour sensor | I2C Data | Second hardware I2C bus |
| **42** | I2C SCL1 | TCS34725 colour sensor | I2C Clock | Second hardware I2C bus |

> [!IMPORTANT]
> **ADC1 vs ADC2 Constraint:**
> On ESP32-S3, **ADC2 (GPIO 11–20) cannot be used for analog reads when WiFi is active**.
> All 9 analog line sensors are strictly placed on **ADC1 (GPIO 1–10)**.
> GPIO 13, 14, 15, 16, 17, 18 are purely digital outputs, so WiFi does not interfere with them.

---

## 2. Separate I2C Buses & Address Map

The PCA9685 and all three VL53L0X ToF sensors share the main bus (**SDA: GPIO 11**, **SCL: GPIO 12**). The TCS34725 uses the ESP32-S3's second I2C controller (**SDA: GPIO 41**, **SCL: GPIO 42**). The VL53L0X sensors boot at address `0x29`; `tofBegin()` enumerates them one-at-a-time using XSHUT pins and assigns each a unique runtime address.

| Device | Default Boot Address | Runtime Address | How Address Is Set |
| :--- | :---: | :---: | :--- |
| **PCA9685** (12 Servo Driver) | `0x40` | **`0x40`** | Hardware address pins A0–A5 grounded |
| **TCS34725** (Colour Sensor) | `0x29` | **`0x29`** | Dedicated I2C1 bus, GPIO 41/42 |
| **VL53L0X** (Front ToF) | `0x29` | **`0x30`** | Main I2C0 bus, GPIO 11/12 |

### Why XSHUT Is Required
The VL53L0X and TCS34725 may share the same numeric address because they are physically on separate ESP32-S3 I2C controllers and separate SDA/SCL wires. The PCA9685 remains on the main bus at `0x40`.

---

## 3. VL53L0X ToF Sensor Wiring

### Sensor Wiring

| VL53L0X Pin | Connects To | Notes |
| :--- | :--- | :--- |
| **VIN** | ESP32 **3.3V** | Never connect to 5V |
| **GND** | Common **GND** | Must share ground with ESP32 |
| **SDA** | **GPIO 11** | Main I2C0 SDA shared with PCA9685 |
| **SCL** | **GPIO 12** | Main I2C0 SCL shared with PCA9685 |
| **XSHUT (Front)** | **GPIO 13** | Dedicated shutdown line |
| **XSHUT (Right)** | **GPIO 14** | Dedicated shutdown line (Right ToF) |
| **XSHUT (Left)** | **GPIO 21** | Dedicated shutdown line (Left ToF) |
| **GPIO1 (Interrupt)** | *Leave unconnected* | Not needed for polling |

---

## 4. Full ESP32-S3 DevKit Header Pin Diagram

Visual layout of the ESP32-S3 DevKit (44-pin board):

```
                       ┌───────────────────────┐
                       │      ESP32-S3         │
                       │     (USB at top)      │
                       └───────────────────────┘
          [LEFT HEADER]                         [RIGHT HEADER]
  3.3V Logic ────────── 3V3                 GND ────────── Common Ground
  3.3V Logic ────────── 3V3                GPIO43 ──────── (UART0 TX / Arm Alt)
  Board Reset ───────── EN                 GPIO44 ──────── ARM Grip Servo (LEDC ch7)
  Line S3 (ADC1) ────── GPIO4               GPIO1 ──────── Line S1 (ADC1)
  Line S4 (ADC1) ────── GPIO5               GPIO2 ──────── Line S2 (ADC1)
  Line S5 Center ────── GPIO6              GPIO42 ──────── Color I2C1 SCL
  Line S6 (ADC1) ────── GPIO7              GPIO41 ──────── Color I2C1 SDA
  TCS LED Ctrl ──────── GPIO15             GPIO40 ──────── Button 3 (INPUT_PULLUP)
  Status LED Red ────── GPIO16             GPIO39 ──────── Button 2 (INPUT_PULLUP)
  Status LED Green ──── GPIO17             GPIO38 ──────── Button 1 (INPUT_PULLUP)
  Status LED Blue ───── GPIO18             GPIO37 ──────── (SPI/PSRAM)
  Line S7 (ADC1) ────── GPIO8              GPIO36 ──────── (SPI/PSRAM)
  USB D- (Native) ───── GPIO19             GPIO35 ──────── (SPI/PSRAM)
  USB D+ (Native) ───── GPIO20              GPIO0 ──────── (Strapping / Boot)
  (Strapping) ───────── GPIO3              GPIO45 ──────── (Strapping)
  (Strapping) ───────── GPIO46             GPIO47 ──────── ARM Lift Servo (LEDC ch6)
  Line S8 (ADC1) ────── GPIO9              GPIO21 ──────── VL53L0X Left XSHUT
  Line S9 (ADC1) ────── GPIO10              20/19 ──────── (USB D+/D-)
  I2C SDA ───────────── GPIO11               GND  ──────── Common Ground
  I2C SCL ───────────── GPIO12               5V   ──────── 5V Reg In / VBUS
  ToF Front XSHUT ───── GPIO13               GND  ──────── Common Ground
  ToF Right XSHUT ───── GPIO14               NC   ──────── (Not Connected)
                       └───────────────────────┘
```

---

## 5. 4-Pin Common-Cathode Status RGB LED

### Wiring Details

| LED Leg | Connects to ESP32-S3 | Resistor | Description |
| :--- | :--- | :---: | :--- |
| **Red Leg** | **GPIO 16** | 330 Ω | Active HIGH |
| **Common Cathode** | **GND** | *Direct* | Shared negative return |
| **Green Leg** | **GPIO 17** | 330 Ω | Active HIGH |
| **Blue Leg** | **GPIO 18** | 330 Ω | Active HIGH |

> **Warning:** Never connect an LED leg directly to a GPIO without a current-limiting resistor (330 Ω recommended).

### Helper Code

```cpp
constexpr uint8_t RGB_RED_PIN   = 16;
constexpr uint8_t RGB_GREEN_PIN = 17;
constexpr uint8_t RGB_BLUE_PIN  = 18;

void setRgb(bool red, bool green, bool blue) {
  digitalWrite(RGB_RED_PIN,   red   ? HIGH : LOW);
  digitalWrite(RGB_GREEN_PIN, green ? HIGH : LOW);
  digitalWrite(RGB_BLUE_PIN,  blue  ? HIGH : LOW);
}

// Common status presets:
// setRgb(false, false, false); // OFF
// setRgb(true,  false, false); // RED: Error / Line Lost
// setRgb(false, true,  false); // GREEN: Ready / Tracking Line
// setRgb(false, false, true);  // BLUE: Calibrating / Web Access
// setRgb(true,  true,  false); // YELLOW: Obstacle / Caution
// setRgb(true,  false, true);  // MAGENTA: Walking Gait
```

---

## 6. 3× Hardware Pushbuttons Wiring & Code

### Recommended Pin Selection: **GPIO 38, GPIO 39, GPIO 40**
- **Why these pins?**
  - Adjacent, consecutive pins grouped on the DevKit right header.
  - 100% free digital GPIOs with internal pull-up and pull-down support.
  - Completely independent of boot strapping pins (0, 3, 45, 46) and native USB (19, 20).
  - Safe for active-LOW switches: **no external resistors needed** (`pinMode(pin, INPUT_PULLUP)`).
  - *(If a 4th button or toggle switch is ever added, use another GPIO marked free in the pin table).*

### Hardware Wiring (Active-LOW to GND)

Each tactile button has 2 terminals (connect one terminal to the GPIO, the other to common GND):

```
  ESP32-S3 GPIO 38 ───────┐
                          │   [ Button 1 / SW1 ]
  ESP32-S3 GND     ───────┴───◯                ◯───┐
                                                   │
  ESP32-S3 GPIO 39 ───────┐                        │
                          │   [ Button 2 / SW2 ]   │
  ESP32-S3 GND     ───────┴───◯                ◯───┼───► Common GND
                                                   │
  ESP32-S3 GPIO 40 ───────┐                        │
                          │   [ Button 3 / SW3 ]   │
  ESP32-S3 GND     ───────┴───◯                ◯───┘
```
- **Idle (Not Pressed)**: Internal pull-up holds the GPIO input at **HIGH** (3.3V).
- **Pressed**: Switch connects GPIO directly to GND, reading **LOW** (0V).

### Firmware Implementation & Debounce Example

```cpp
constexpr uint8_t BTN_PIN_1 = 38; // e.g. Start / Stop Line Follow or Walk
constexpr uint8_t BTN_PIN_2 = 39; // e.g. Return to STANDING pose
constexpr uint8_t BTN_PIN_3 = 40; // e.g. Start Line Sensor Calibration sweep

void initButtons() {
  pinMode(BTN_PIN_1, INPUT_PULLUP);
  pinMode(BTN_PIN_2, INPUT_PULLUP);
  pinMode(BTN_PIN_3, INPUT_PULLUP);
}

// Non-blocking button press detector with 200ms debounce
bool isButtonPressed(uint8_t pin) {
  static unsigned long lastPressTime[3] = {0, 0, 0};
  uint8_t idx = (pin == BTN_PIN_1) ? 0 : (pin == BTN_PIN_2) ? 1 : 2;

  if (digitalRead(pin) == LOW) {
    if (millis() - lastPressTime[idx] > 200) {
      lastPressTime[idx] = millis();
      return true;
    }
  }
  return false;
}
```

---

## 7. Power Distribution Architecture

```
[2S LiPo Battery: 7.4V - 8.4V]
       │
       ├──► [6V 5A-8A High-Current BEC / Buck] ────► PCA9685 V+ (Leg Servos) & Arm Servos VCC
       │                                              (Powers 12× Leg Servos + 2× Arm Servos)
       │
       └──► [5V 2A-3A Logic Buck Regulator] ───────► ESP32-S3 5V / VIN pin
                                                       │
                                                       ▼ [ESP32 Onboard 3.3V LDO]
                                                       ├─► 3.3V: 9× TCRT5000 Line Array
                                                       ├─► 3.3V: TCS34725 Colour Sensor
                                                       ├─► 3.3V: 3× VL53L0X ToF Sensors
                                                       └─► 3.3V: PCA9685 VCC (Logic)
```

> [!CAUTION]
> **Ground Isolation / Ground Loops:**
> All grounds (Battery (-), 6V BEC GND, 5V Logic GND, ESP32 GND, and PCA9685 GND) must be tied together into a **single common ground point**.
> **NEVER** power servos from the ESP32 3.3V pin or ESP32 5V pin.

---

## 8. Reserved & Strapping Pins (Do Not Wire)

| Pin | Reason to Avoid |
| :--- | :--- |
| **GPIO 0, 3, 45, 46** | Boot mode strapping pins; external pull up/down can prevent ESP32 from booting. |
| **GPIO 19, 20** | Native USB (D- / D+) used for USB CDC serial monitor and sketch flashing. |
| **GPIO 26–32** | Hard-wired to internal Quad-SPI Flash memory. |
| **GPIO 33–37** | Connected to internal Octal-SPI PSRAM/Flash on most ESP32-S3 modules. |
| **GPIO 43, 44** | Hardware UART0 (TX/RX) serial interface. Free for general I/O or PWM (such as ARM Grip on **GPIO 44**) when native USB CDC is enabled for programming and monitoring. |
| **GPIO 48** | Pre-wired to onboard addressable WS2812 RGB LED on DevKit boards. |
