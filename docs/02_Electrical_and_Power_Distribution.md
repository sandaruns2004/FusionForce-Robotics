# 02. Electrical Architecture & Power Distribution

## 1. Executive Summary & Power Architecture

A stable, low-noise power distribution network is essential for an autonomous quadruped robot. The 12 leg servos and 2 arm servos draw high-current, pulsed loads that can cause voltage dips and severe inductive noise on the power rails. 

To ensure zero brownout resets of the **Raspberry Pi 4B** and **STM32F401 Black Pill**, the electrical architecture isolates the high-current actuator bus from the sensitive logic bus using dual step-down regulators connected to a common **3S LiPo Battery (11.1V–12.6V nominal)**.

```
                           POWER TREE SCHEMATIC (Star-Grounded)
                           
                            +-------------------------------+
                            |   3S LiPo Battery (11.1V)     |
                            |   1200mAh - 1500mAh 45C+      |
                            +---------------+---------------+
                                            |
                         [Main Power Switch & XT60 Connector]
                                            |
                      +---------------------+---------------------+
                      |                                           |
                      v                                           v
       +------------------------------+            +------------------------------+
       |   5V / 10A UBEC Regulator    |            |  5V / 5A Buck Logic Reg      |
       |  (High-Current Servo Bus)    |            |   (Clean Logic Bus)          |
       +--------------+---------------+            +--------------+---------------+
                      |                                           |
                      +-------------------+                       +---------------+
                      |                   |                                       |
                      v                   v                                       v
        +-------------------+   +-------------------+              +------------------------------+
        | PCA9685 V+ Power  |   | 2 x Arm Servos    |              |  Raspberry Pi 4B (5V Pin 2)  |
        | (12 x Leg Servos) |   | (Direct 5V Rail)  |              +--------------+---------------+
        +-------------------+   +-------------------+                             |
                                                                                  | (USB-C or 5V Pin)
                                                                                  v
                                                                   +------------------------------+
                                                                   |  STM32F401CCEU Black Pill    |
                                                                   +--------------+---------------+
                                                                                  |
                                                                   +--------------+---------------+
                                                                   |  3.3V LDO Logic Sub-Rail     |
                                                                   +--------------+---------------+
                                                                                  |
                                                                                  +---> 3 x VL53L0X ToF
                                                                                  +---> MPU6050 IMU
                                                                                  +---> SSD1306 OLED
```

---

## 2. Power Specifications & Budget Analysis

### 2.1 Voltage & Current Requirements

| Subsystem / Component | Nominal Voltage | Max Peak Current | Power Bus Assignment |
| :--- | :--- | :--- | :--- |
| **12 × Leg Servos** (MG996R + MG90) | **5.0V – 6.0V** | **4.8A – 7.2A** (Stall peak) | **5V / 10A UBEC** (Actuator Rail) |
| **2 × Arm/Gripper Servos** (MG90) | **5.0V – 6.0V** | **0.8A** | **5V / 10A UBEC** (Actuator Rail) |
| **Raspberry Pi 4B (4GB/8GB)** | **5.0V ± 5%** | **2.5A – 3.0A** (100% CPU + Cam) | **5V / 5A Buck** (Logic Rail) |
| **STM32F401 Black Pill** | **5.0V (VBUS) / 3.3V** | **0.15A** | **5V / 5A Buck** $\rightarrow$ STM32 LDO |
| **3 × VL53L0X ToF Sensors** | **3.3V / 5.0V** | **0.06A** | STM32 3.3V Bus |
| **MPU6050 IMU & OLED Display** | **3.3V** | **0.05A** | STM32 3.3V Bus |
| **Total System Load** | -- | **~8.5A (Continuous ~4.2A)** | **3S LiPo 45C (54A continuous capability)** |

### 2.2 Star Grounding & Noise Suppression Rules
1. **Common Ground Point:** All ground wires (3S LiPo GND, UBEC GND, Logic Buck GND, Raspberry Pi GND, STM32 GND, and PCA9685 GND) must meet at a **single copper star-ground junction board** or distribution terminal.
2. **Bulk Decoupling Capacitors:** Place a **4700µF / 16V electrolytic capacitor** directly across the PCA9685 V+ and GND terminals to absorb servo back-EMF spikes during foot impacts.
3. **Ferrite Beads:** Install clamp-on ferrite beads around the UART signal leads connecting the Raspberry Pi 4B to the STM32F401.

---

## 3. Full System Pinout Schematics

### 3.1 Inter-Board UART & Control Wiring (Pi 4B $\leftrightarrow$ STM32F401)
The Pi 4B acts as the master mission controller, communicating with the STM32F401 via high-speed hardware UART at **115200 baud** (upgradable to 921600 baud for 50Hz telemetry loops).

| Signal Name | Raspberry Pi 4B Physical Pin (BCM GPIO) | STM32F401CCEU Pin | Wire Description / Function |
| :--- | :--- | :--- | :--- |
| **UART0 TX** | Pin 8 (GPIO 14 - TXD) | **PA10 (USART1 RX)** | High-speed command stream (Velocity, Action, E-Stop) |
| **UART0 RX** | Pin 10 (GPIO 15 - RXD) | **PA9 (USART1 TX)** | High-speed telemetry stream (ToF distances, IMU, state) |
| **GND Reference** | Pin 6 (GND) | **GND (Header)** | Essential common reference point for UART signal integrity |
| **ESTOP / IRQ** | Pin 16 (GPIO 23) | **PB0 (EXTI0)** | Dedicated emergency stop / mission start hardware sync line |

---

### 3.2 STM32F401 Low-Level I2C & Sensor Bus Allocation

The STM32F401 uses two separate I2C peripherals to avoid bus contention between the high-frequency servo driver (PCA9685) and the time-of-flight (ToF) distance sensors.

```
       STM32F401CCEU (I2C1 - Fast Mode 400 kHz)
          |--- SCL: PB6  -------------------+---> PCA9685 Servo Driver (Addr 0x40)
          |--- SDA: PB7  -------------------+---> MPU6050 6-Axis IMU    (Addr 0x68)
          
       STM32F401CCEU (I2C2 - 100/400 kHz via TCA9548A Multiplexer)
          |--- SCL: PB10 -------------------> TCA9548A 8-Ch Mux (Addr 0x70)
          |--- SDA: PB11 -------------------|      |
                                                   |---> Mux SD0/SC0 -> VL53L0X Front (0x29)
                                                   |---> Mux SD1/SC1 -> VL53L0X Left  (0x29)
                                                   +---> Mux SD2/SC2 -> VL53L0X Right (0x29)
```

| Device Name | STM32 Pin Assignment | Bus Address / Pin ID | Notes & Alternative XSHUT Configuration |
| :--- | :--- | :--- | :--- |
| **PCA9685 16-Ch Driver** | **PB6 (SCL) / PB7 (SDA)** | `0x40` (7-bit) | Controls all 12 leg servos + 2 arm servos at 50Hz PWM |
| **MPU6050 6-Axis IMU** | **PB6 (SCL) / PB7 (SDA)** | `0x68` (7-bit) | Provides body pitch/roll for active dynamic leveling |
| **TCA9548A I2C Mux** | **PB10 (SCL) / PB11 (SDA)** | `0x70` (7-bit) | Eliminates I2C address conflict between identical VL53L0X sensors |
| **VL53L0X Front ToF** | Via TCA9548A Channel 0 | `0x29` | *Alt without Mux:* Connect XSHUT to STM32 **PA4** |
| **VL53L0X Left ToF** | Via TCA9548A Channel 1 | `0x29` | *Alt without Mux:* Connect XSHUT to STM32 **PA5** |
| **VL53L0X Right ToF** | Via TCA9548A Channel 2 | `0x29` | *Alt without Mux:* Connect XSHUT to STM32 **PA6** |
| **Start Button (Push)** | **PA0 (WKUP/EXTI0)** | Active High | Triggers transition from `STATE_0` to mission run |
| **Status RGB LED** | **PA1 (R), PA2 (G), PA3 (B)**| PWM Output | Visual debugging: Green=Walking, Blue=Searching, Red=Fault |

---

### 3.3 PCA9685 Servo Channel Allocations (16-Channel Board)

All 12 leg servos and 2 arm/gripper servos connect directly to the 3-pin headers of the **PCA9685 16-Channel PWM Servo Driver**.

| PCA9685 Channel | Assigned Servo Joint | Leg / Manipulator Group | Recommended Servo Model |
| :--- | :--- | :--- | :--- |
| **CH 0** | Front-Left Coxa (Hip Yaw - $\theta_1$) | **Front-Left Leg (FL)** | MG90 Micro Servo |
| **CH 1** | Front-Left Femur (Hip Pitch - $\theta_2$) | **Front-Left Leg (FL)** | MG996R Standard Servo |
| **CH 2** | Front-Left Tibia (Knee Pitch - $\theta_3$) | **Front-Left Leg (FL)** | MG996R Standard Servo |
| **CH 3** | Front-Right Coxa (Hip Yaw - $\theta_1$) | **Front-Right Leg (FR)** | MG90 Micro Servo |
| **CH 4** | Front-Right Femur (Hip Pitch - $\theta_2$) | **Front-Right Leg (FR)** | MG996R Standard Servo |
| **CH 5** | Front-Right Tibia (Knee Pitch - $\theta_3$) | **Front-Right Leg (FR)** | MG996R Standard Servo |
| **CH 6** | Back-Left Coxa (Hip Yaw - $\theta_1$) | **Back-Left Leg (BL)** | MG90 Micro Servo |
| **CH 7** | Back-Left Femur (Hip Pitch - $\theta_2$) | **Back-Left Leg (BL)** | MG996R Standard Servo |
| **CH 8** | Back-Left Tibia (Knee Pitch - $\theta_3$) | **Back-Left Leg (BL)** | MG996R Standard Servo |
| **CH 9** | Back-Right Coxa (Hip Yaw - $\theta_1$) | **Back-Right Leg (BR)** | MG90 Micro Servo |
| **CH 10** | Back-Right Femur (Hip Pitch - $\theta_2$) | **Back-Right Leg (BR)** | MG996R Standard Servo |
| **CH 11** | Back-Right Tibia (Knee Pitch - $\theta_3$) | **Back-Right Leg (BR)** | MG996R Standard Servo |
| **CH 12** | Arm Pitch (Elevation / Lowering) | **Front Manipulator** | MG90 Micro Servo |
| **CH 13** | Gripper Jaws (Open / Close) | **Front Manipulator** | MG90 Micro Servo |
| **CH 14 – CH 15** | *Reserved / Spare Channels* | Optional Belly Trapdoor | MG90 Micro Servo |

---

## 4. Raspberry Pi 4B Peripheral Topologies

```
                     RASPBERRY PI 4B PERIPHERAL TOPOLOGY
                     
                        +---------------------------+
                        |      Raspberry Pi 4B      |
                        +-------------+-------------+
                                      |
       +------------------------------+------------------------------+
       |                              |                              |
       v (CSI Ribbon Cable)           v (GPIO 14/15 UART)            v (USB 3.0 / GPIO 23)
+------------------------+  +------------------------+  +------------------------+
|   Camera Module 3      |  |  STM32F401 Black Pill  |  |  User Control Switch   |
|  (640x480 @ 60 FPS)    |  |  (IPC Command/Telemet) |  |  (Start / Mode Toggle) |
+------------------------+  +------------------------+  +------------------------+
```

### 4.1 Camera & Vision Hardware Configuration
- **Camera Interface:** Raspberry Pi Camera Module 3 connected via a 15-pin CSI-2 ribbon cable to the `CAM1` port.
- **Resolution & Frame Rate:** Operating at **640×480 resolution at 60 FPS** for low-latency line following and HSV color segmentation.
- **Exposure Control:** Fixed manual exposure or locked white balance during competition run to prevent HSV drift under arena spotlight changes.
