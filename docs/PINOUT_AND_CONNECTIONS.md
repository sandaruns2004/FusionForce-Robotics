# PINOUT AND CONNECTIONS

## 1. Authoritative Hardware Mapping

> [!WARNING]
> Do not deviate from these mappings without updating this document. If a conflict arises between code and this document, the hardware must be re-verified.

### STM32F411 (Black Pill) Connections

| STM32 Pin | Function | Connected To | Voltage | Notes |
|---|---|---|---|---|
| **PA9** | UART1 TX | Raspberry Pi RX (GPIO 15) | 3.3V | Connects Pi and STM32 |
| **PA10** | UART1 RX | Raspberry Pi TX (GPIO 14) | 3.3V | Connects Pi and STM32 |
| **PB6** | I2C1 SCL | PCA9685, ToF, IMU | 3.3V | Main I2C Bus |
| **PB7** | I2C1 SDA | PCA9685, ToF, IMU | 3.3V | Main I2C Bus |
| **GND** | Ground | Common Ground Rail | 0V | Star Grounding Point |
| **3V3** | Power In | Raspberry Pi 3.3V Out | 3.3V | Powers STM32 |

### Raspberry Pi 4B Connections

| Pi Pin | Function | Connected To | Voltage | Notes |
|---|---|---|---|---|
| **GPIO 14** (Pin 8) | UART TX | STM32 PA10 (RX) | 3.3V | |
| **GPIO 15** (Pin 10)| UART RX | STM32 PA9 (TX) | 3.3V | |
| **Pin 6** | Ground | Common Ground Rail | 0V | |
| **Pin 1** | 3.3V Out | STM32 3V3 Pin | 3.3V | Supplies logic power |
| **CSI Port** | Camera | Camera Module 3 | - | Ribbon Cable |
| **USB-C** | Power In | 5V 5A Buck Converter | 5.0V | Main Pi Power |

### PCA9685 Servo Controller

| PCA Pin | Function | Connected To | Voltage | Notes |
|---|---|---|---|---|
| **VCC** | Logic Power | STM32 3.3V | 3.3V | Pulls up I2C |
| **SDA** | Data | STM32 PB7 | 3.3V | |
| **SCL** | Clock | STM32 PB6 | 3.3V | |
| **GND** | Logic Ground| Common Ground Rail | 0V | |
| **V+ (Terminal)**| Motor Power| 10A UBEC Output | 5.0V | High Current Rail |
| **GND (Terminal)**| Motor Ground| Common Ground Rail | 0V | High Current Return |
| **PWM 0-2** | Leg 1 | Front Left Servos | 5.0V | Coxa, Femur, Tibia |
| **PWM 3-5** | Leg 2 | Front Right Servos | 5.0V | Coxa, Femur, Tibia |
| **PWM 6-8** | Leg 3 | Back Left Servos | 5.0V | Coxa, Femur, Tibia |
| **PWM 9-11**| Leg 4 | Back Right Servos | 5.0V | Coxa, Femur, Tibia |
| **PWM 14** | Gripper | Claw Open/Close | 5.0V | |
| **PWM 15** | Gripper | Claw Pitch (Up/Down)| 5.0V | |

## 2. I2C Address Map

| Device | Default Address | Hex | Changeable? |
|---|---|---|---|
| **PCA9685** | 64 | `0x40` | Yes (Solder Pads) |
| **MPU6050** | 104 | `0x68` | Yes (AD0 pin to 0x69) |
| **VL53L0X** (x3) | 41 | `0x29` | **YES** (Via XSHUT pin on boot) |

*Note: Since three VL53L0X sensors share the `0x29` address by default, the STM32 must hold two of their `XSHUT` pins LOW on boot, initialize the first one, change its address via software to `0x30`, release the next `XSHUT` pin, initialize it, change it to `0x31`, etc.*
