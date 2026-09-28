# 05 Power Distribution
## RUNNER-4 | Dual-Rail Power System | 6V Servo + 3.3V Logic

## Objective
Route high-current servo power and low-noise logic power safely, avoiding brownouts, ground loops, and servo damage to the STM32.

## The Two Power Rails

```
  LiPo Battery (2S=7.4V or 3S=11.1V)
       |
       +---[Kill Switch 15A]---+
                               |
               +---------------+---------------+
               |                               |
   [Switching BEC ≥5A]               [LDO/SMPS 3.3V]
   Output: 6.0V                      Output: 3.3V
               |                               |
   +-----------+----------+         +----------+-----------+
   |           |          |         |          |           |
  PCA9685   Servo V+   Servo V+   STM32    PCA9685   Sensors
  (servo)   CH0-CH11  CH12-14    VCC       (logic)  (I2C/ADC)

  GND: ALL above share one thick common GND wire (star ground)
```

## Power Consumption Analysis

| Component | Rail | Typical | Peak (stall) |
|-----------|------|---------|-------------|
| STM32F411CEU6 | 3.3V | 30 mA | 50 mA |
| PCA9685 logic | 3.3V | 10 mA | 15 mA |
| MPU6050 | 3.3V | 3.9 mA | 4 mA |
| TCS34725 + LED | 3.3V | 11 mA | 11 mA |
| 3× VL53L0X | 3.3V | 30 mA | 30 mA |
| 9× TCRT5000 IR | 3.3V | 81 mA | 81 mA |
| **3.3V Total** | | **~166 mA** | ~191 mA |
| 12× MG90S legs | 6V | 1200–2400 mA | 8400 mA |
| 3× MG90S arm/grip/gate | 6V | 200–400 mA | 2100 mA |
| **6V Servo Total** | | **1.4–2.8 A** | **~10.5 A stall** |

> **BEC Selection:** Must be switching type, ≥5A continuous, ≥10A peak. Linear BEC overheats and causes brownout at high current. Recommended: 5A or 8A SBEC module.

## Wiring Steps

### Step 1: Battery and Kill Switch
```
1. Solder XT60 male connector to LiPo leads
2. Splice 15A latching toggle switch onto RED (positive) lead
   - Switch in series, NOT across the battery
3. Add 10A automotive blade fuse inline to servo BEC output
```

### Step 2: 6V Servo Rail (BEC)
```
1. Connect BEC input (raw) to battery positive rail
2. Set BEC output to 6.0V ± 0.1V (measure with multimeter under load)
3. Connect BEC output (+6V) to PCA9685 V+ terminal block
4. DO NOT connect BEC output to PCA9685 VCC pin (that is logic power only)
5. All 15 servo red wires connect to the same +6V rail
```

### Step 3: 3.3V Logic Rail
```
1. Connect LDO/SMPS input to battery positive rail
2. Set output to 3.3V ± 0.05V
3. Connect to STM32 3V3 pin
4. Connect to PCA9685 VCC (logic)
5. Connect to all sensor VCC pins (MPU6050, VL53L0X ×3, TCS34725)
   NOTE: TCRT5000 IR sensors — use 3.3V with 10kΩ pull-up resistors
```

### Step 4: Star Ground
```
ALL of the following GND wires must join at ONE point:
  - BEC output GND
  - LDO/SMPS output GND
  - STM32 GND pin
  - PCA9685 GND (both logic GND and servo GND rails)
  - All sensor GND pins
  - LiPo battery negative

Use a thick (18AWG) wire from LiPo negative to this star point.
From star point use thinner wires to individual components.
This eliminates ground loops that cause servo jitter and IMU noise.
```

## Pull-up Resistors (I2C)

```
I2C1 (PB6 SCL, PB7 SDA): 4.7kΩ to 3.3V (×2 resistors)
I2C2 (PB10 SCL, PB3 SDA): 4.7kΩ to 3.3V (×2 resistors)
Total: 4 resistors (4.7kΩ 0603 or through-hole)
```

> **Never omit pull-ups.** Without them, I2C signals float and all I2C devices (PCA9685, MPU6050, VL53L0X, TCS34725) will fail intermittently.

## Safety Features

| Protection | Component | Spec | Purpose |
|-----------|-----------|------|---------|
| Main fuse | Automotive blade | 15A | Protects battery leads |
| Servo fuse | Automotive blade | 10A | Protects BEC output |
| Kill switch | Latching toggle | 15A | Emergency off |
| Decoupling caps | Ceramic 100nF | X7R | One per sensor/IC for noise |
| Servo rail cap | Electrolytic 470µF | 16V | Absorbs servo current spikes |

## Voltage Checks (before first power-on)

```
[ ] Battery voltage > 7.4V (2S) or > 11.1V (3S)
[ ] BEC output = 6.0V ± 0.1V  (under servo load)
[ ] 3.3V rail  = 3.30V ± 0.05V
[ ] All sensor VCC pins read 3.3V
[ ] PCA9685 V+  reads 6.0V
[ ] PCA9685 VCC reads 3.3V
[ ] All GND connections continuity-checked
[ ] No short between 6V rail and 3.3V rail
```

## Common Mistakes

| Mistake | Consequence | Fix |
|---------|------------|-----|
| Servo wire into STM32 5V/3.3V pin | STM32 burned instantly | Use BEC output only |
| Linear BEC at high servo current | Overheating, brownout | Use switching BEC only |
| Missing star ground | Ground loops, servo jitter, IMU noise | Wire all GNDs together |
| Missing I2C pull-ups | I2C bus failure, all sensors fail | Add 4.7kΩ per bus |
| Missing servo rail cap | Servo spike brownouts | Add 470µF on servo rail |

---
🔙 **[Back to Mechanical](../README.md)**
