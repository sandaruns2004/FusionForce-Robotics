# PINOUT AND CONNECTIONS

## 1. Authoritative Hardware Mapping

> [!WARNING]
> Do not deviate from these mappings without updating this document. If a conflict arises between code and this document, the hardware must be re-verified.

---

### STM32F411CEU6 (Black Pill) — Complete Pin Assignment

| STM32 Pin | Function | Connected To | Voltage | Notes |
|-----------|----------|-------------|---------|-------|
| **PA0** | GPIO IN — Line S1 | TCRT5000 Ch1 OUT | 3.3V | Leftmost IR sensor |
| **PA1** | GPIO IN — Line S2 | TCRT5000 Ch2 OUT | 3.3V | |
| **PA2** | GPIO IN — Line S3 | TCRT5000 Ch3 OUT | 3.3V | |
| **PA3** | GPIO IN — Line S4 | TCRT5000 Ch4 OUT | 3.3V | Left-centre |
| **PA4** | GPIO IN — Line S5 | TCRT5000 Ch5 OUT | 3.3V | Right-centre |
| **PA5** | GPIO IN — Line S6 | TCRT5000 Ch6 OUT | 3.3V | |
| **PA6** | GPIO IN — Line S7 | TCRT5000 Ch7 OUT | 3.3V | |
| **PA7** | GPIO IN — Line S8 | TCRT5000 Ch8 OUT | 3.3V | Rightmost IR sensor |
| **PA9** | USART1 TX | Debug PC (RX) | 3.3V | **Dev only — disconnect at competition** |
| **PA10** | USART1 RX | Debug PC (TX) | 3.3V | **Dev only — disconnect at competition** |
| **PB3** | I2C2 SDA | 3× VL53L0X SDA | 3.3V | ToF sensor bus |
| **PB6** | I2C1 SCL | PCA9685, MPU6050, TCS34725 SCL | 3.3V | Main I2C bus |
| **PB7** | I2C1 SDA | PCA9685, MPU6050, TCS34725 SDA | 3.3V | Main I2C bus |
| **PB10** | I2C2 SCL | 3× VL53L0X SCL | 3.3V | ToF sensor bus |
| **PB12** | GPIO OUT — XSHUT1 | VL53L0X Front XSHUT | 3.3V | Address remap control |
| **PB13** | GPIO OUT — XSHUT2 | VL53L0X Left XSHUT | 3.3V | Address remap control |
| **PB14** | GPIO OUT — XSHUT3 | VL53L0X Right XSHUT | 3.3V | Address remap control |
| **PC0** | GPIO OUT — LED | TCS34725 LED pin | 3.3V | Controls built-in illuminator |
| **PC1** | ADC1 IN11 | Battery voltage divider | 3.3V | Low-battery detection |
| **PA0 (Boot0)** | Start Button | Tactile switch to GND | 3.3V | Single start button (pull-up) |
| **GND** | Ground | All sensor and peripheral GND | 0V | Star ground topology |
| **3V3** | Power Out | STM32 → Sensors (LDO) | 3.3V | MPU, ToF, TCS34725, Line Array |

> [!WARNING]
> PA0 is used **both** as Line Sensor S1 input and Boot0. Use a jumper or switch to disconnect Boot0 during competition operation. Alternatively, configure UART bootloader permanently disabled in STM32 option bytes so PA0 is free for GPIO.

---

### 8-Channel TCRT5000 Line Array Connections

| Line Array Module Pin | Connected To | Notes |
|-----------------------|-------------|-------|
| **VCC** | STM32 3.3V | Check module VCC rating (some need 5V — use level shifter) |
| **GND** | Common GND | |
| **S1 OUT** | STM32 PA0 | Leftmost; configure pull-down |
| **S2 OUT** | STM32 PA1 | |
| **S3 OUT** | STM32 PA2 | |
| **S4 OUT** | STM32 PA3 | |
| **S5 OUT** | STM32 PA4 | |
| **S6 OUT** | STM32 PA5 | |
| **S7 OUT** | STM32 PA6 | |
| **S8 OUT** | STM32 PA7 | Rightmost; configure pull-down |

---

### TCS34725 Colour Sensor Connections

| TCS34725 Pin | Connected To | Voltage | Notes |
|-------------|-------------|---------|-------|
| **VCC** | STM32 3.3V | 3.3V | |
| **GND** | Common GND | 0V | |
| **SCL** | STM32 PB6 | 3.3V | I2C1 bus |
| **SDA** | STM32 PB7 | 3.3V | I2C1 bus |
| **LED** | STM32 PC0 | 3.3V | GPIO output; HIGH = LED ON during reads |
| **INT** | Not connected | — | Interrupt pin (unused — polling mode) |

---

### PCA9685 Servo Controller

| PCA Pin | Function | Connected To | Notes |
|---------|----------|-------------|-------|
| **VCC** | Logic Power | STM32 3.3V | I2C pull-ups |
| **SDA** | Data | STM32 PB7 | I2C1 |
| **SCL** | Clock | STM32 PB6 | I2C1 |
| **GND** | Logic Ground | Common GND | |
| **V+ (Terminal)** | Motor Power | 5V–6V / 15A BEC | High-current servo rail |
| **GND (Terminal)** | Motor Ground | Common GND | |
| **PWM 0–2** | Leg FL | Front Left (Coxa, Femur, Tibia) | |
| **PWM 3–5** | Leg FR | Front Right (Coxa, Femur, Tibia) | |
| **PWM 6–8** | Leg BL | Back Left (Coxa, Femur, Tibia) | |
| **PWM 9–11** | Leg BR | Back Right (Coxa, Femur, Tibia) | |
| **PWM 12** | Arm Pitch | Ball arm elevation servo | MODE A (0°) / MODE B (−70°) / Home |
| **PWM 13** | Gripper | Claw open/close | |
| **PWM 14** | Storage Gate | Ball compartment gate | |
| **PWM 15** | *Spare* | Available | |

---

### VL53L0X Time-of-Flight Sensors (×3)

All three share I2C2 (PB10/PB3). Each has a dedicated XSHUT pin for boot-time address reassignment.

| Sensor | XSHUT Pin | Final I2C Address | Mounting Position |
|--------|----------|------------------|------------------|
| **Front** | PB12 | `0x30` | Front-centre, 25mm above ground, 0° (forward) |
| **Left** | PB13 | `0x31` | Front-left corner, 25mm above ground, 90° (left) |
| **Right** | PB14 | `0x32` | Front-right corner, 25mm above ground, 90° (right) |

---

## 2. Complete I2C Address Map

| Device | Bus | Default Addr | Runtime Addr | Changeable? |
|--------|-----|-------------|-------------|-------------|
| **PCA9685** | I2C1 | `0x40` | `0x40` | Yes (solder A0–A5) |
| **MPU6050** | I2C1 | `0x68` | `0x68` | Yes (AD0 pin) |
| **TCS34725** | I2C1 | `0x29` | `0x29` | No |
| **VL53L0X Front** | I2C2 | `0x29` | `0x30` | Via XSHUT remap at boot |
| **VL53L0X Left** | I2C2 | `0x29` | `0x31` | Via XSHUT remap at boot |
| **VL53L0X Right** | I2C2 | `0x29` | `0x32` | Via XSHUT remap at boot |

> [!NOTE]
> TCS34725 (0x29 on I2C1) and VL53L0X (0x29 default on I2C2) share the same address by coincidence but are on **different I2C buses** — no conflict exists.
