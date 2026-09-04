# COMMUNICATION PROTOCOL

## 1. Overview
All communication in the RUNNER-4 system is **internal to the STM32F411CEU6**. There is no Raspberry Pi, no inter-processor UART, and no external communication of any kind. The STM32 is the sole compute node and communicates with all peripherals via two I2C buses and 8 GPIO pins.

> [!NOTE]
> A debug UART (USART1, PA9/PA10, 115200 baud) is used during development only for serial monitor output. It MUST be physically disconnected before competition (competition rules prohibit external communication).

---

## 2. I2C1 Bus (PB6/PB7 — 400kHz Fast Mode)

Hosts servo driver, IMU, and colour sensor.

| Device | Address (7-bit) | Read Cycle | Write Cycle | Notes |
|--------|----------------|-----------|------------|-------|
| **PCA9685** | `0x40` | Status read (optional) | 16 × PWM updates per 20ms loop | All 15 servo channels |
| **MPU6050** | `0x68` | Accel + Gyro 6 registers | Config on init | Complementary filter processed on STM32 |
| **TCS34725** | `0x29` | RGBC (4 × 16-bit) after integration | Integration time, gain, enable on init | Non-blocking: start, wait 60ms, read |

### PCA9685 Write Cycle (Per Channel)
```
START → [0x40 W] → ACK → REG(LED_ON_L + ch×4) → DATA×4 → STOP
```
Each servo channel requires 4 bytes (ON_L, ON_H, OFF_L, OFF_H). Burst write using auto-increment: update all 15 channels in one I2C transaction (~2ms total).

### TCS34725 Read Sequence
```
1. Write ENABLE register → PON | AEN (start ADC integration)
2. Wait 50–60ms (3 control cycles)
3. Read STATUS register → check AVALID bit
4. Read CDATA, RDATA, GDATA, BDATA (8 bytes, command+auto-increment)
5. Pass to ClassifyColor() → COLOR_RED | COLOR_GREEN | COLOR_BLUE
```

---

## 3. I2C2 Bus (PB10/PB3 — 400kHz Fast Mode)

Hosts three VL53L0X ToF sensors. They share the same I2C bus but have unique addresses assigned during boot via XSHUT pin sequencing.

| Device | Final Address | XSHUT Pin | Init Order |
|--------|--------------|-----------|-----------|
| **VL53L0X Front** | `0x30` | PB12 | 1st |
| **VL53L0X Left** | `0x31` | PB13 | 2nd |
| **VL53L0X Right** | `0x32` | PB14 | 3rd |

### XSHUT Address Assignment Procedure (Runs Once at Boot)
```c
// 1. Hold all XSHUT LOW → all sensors in reset (no I2C activity)
GPIO_Write(PB12, LOW); GPIO_Write(PB13, LOW); GPIO_Write(PB14, LOW);
HAL_Delay(10);

// 2. Release Front sensor → default address 0x29
GPIO_Write(PB12, HIGH); HAL_Delay(2);
VL53L0X_SetAddress(0x29, 0x30);  // Reassign to 0x30

// 3. Release Left sensor → default address 0x29
GPIO_Write(PB13, HIGH); HAL_Delay(2);
VL53L0X_SetAddress(0x29, 0x31);  // Reassign to 0x31

// 4. Release Right sensor → default address 0x29
GPIO_Write(PB14, HIGH); HAL_Delay(2);
VL53L0X_SetAddress(0x29, 0x32);  // Reassign to 0x32

// All three sensors now respond at unique addresses 0x30, 0x31, 0x32
```

### VL53L0X Read Cycle (Per Sensor)
```
Write: START_RANGE command → wait RESULT_READY → Read 2 bytes (uint16 mm)
Median filter: buffer last 5 readings; return median (noise rejection)
```

---

## 4. GPIO Line Array Interface (PA0–PA7)

The 8-channel TCRT5000 line array connects via 8 digital GPIO input pins. This is not I2C — it is the fastest possible interface.

| STM32 Pin | GPIO Port | Sensor Channel | Description |
|-----------|----------|---------------|-------------|
| PA0 | GPIOA bit 0 | S1 | Leftmost sensor |
| PA1 | GPIOA bit 1 | S2 | |
| PA2 | GPIOA bit 2 | S3 | |
| PA3 | GPIOA bit 3 | S4 | Left-centre |
| PA4 | GPIOA bit 4 | S5 | Right-centre |
| PA5 | GPIOA bit 5 | S6 | |
| PA6 | GPIOA bit 6 | S7 | |
| PA7 | GPIOA bit 7 | S8 | Rightmost sensor |

**Read cycle**: One 32-bit GPIOA->IDR register read, masked with `0xFF`. Total latency: **<100ns**. No protocol overhead.

```c
uint8_t LineArray_Read(void) {
    return (uint8_t)(GPIOA->IDR & 0x00FF);  // Read all 8 at once
}
```

---

## 5. I2C Address Map (Complete)

| Device | Bus | Address | Changeable? |
|--------|-----|---------|-------------|
| PCA9685 | I2C1 | `0x40` | Yes (solder pads A0–A5) |
| MPU6050 | I2C1 | `0x68` | Yes (AD0 pin → 0x69) |
| TCS34725 | I2C1 | `0x29` | No (fixed, single address) |
| VL53L0X Front | I2C2 | `0x30` (runtime remap) | Via XSHUT |
| VL53L0X Left | I2C2 | `0x31` (runtime remap) | Via XSHUT |
| VL53L0X Right | I2C2 | `0x32` (runtime remap) | Via XSHUT |

> [!WARNING]
> TCS34725 default address 0x29 conflicts with VL53L0X default 0x29. They are on **separate I2C buses** (TCS34725 on I2C1, VL53L0X on I2C2), so there is no conflict.
