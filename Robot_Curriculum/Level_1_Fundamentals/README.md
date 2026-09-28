# Level 1 — Fundamentals
## Prerequisites for RUNNER-4 Firmware Development

## Why This Level Matters
Every higher-level skill (IK math, gait engines, sensor drivers) depends on these fundamentals. Gaps here cause mysterious bugs later. Invest the time.

## Topics

### 1. C Programming for Embedded Systems
| Topic | Key Concepts | Application on RUNNER-4 |
|-------|-------------|------------------------|
| Pointers & memory | `*ptr`, `&addr`, arrays as pointers | IK solver passes `FootTarget_t *foot` |
| Structs | `typedef struct { float x,y,z; } FootTarget_t` | All data passed as structs |
| Fixed-width types | `uint8_t`, `uint16_t`, `int16_t` | ADC buffers, I2C register maps |
| Bitwise operations | `&`, `\|`, `<<`, `>>` | PCA9685 register config, ADC status |
| `volatile` keyword | Prevents compiler optimisation of ISR variables | `volatile uint8_t update_flag` in main loop |
| `const` arrays | Read-only flash data | `IK_NeutralStance[4]`, calibration tables |
| Math.h | `atan2f()`, `sqrtf()`, `sinf()`, `acosf()` | Entire IK + gait engine |

### 2. Electronics Fundamentals
| Topic | Key Concepts | Application |
|-------|-------------|-------------|
| Ohm's Law | V=IR, power P=VI | BEC current sizing (P=6V×10A=60W) |
| Pull-up resistors | R to VCC, open-drain signals | I2C bus: 4.7kΩ to 3.3V |
| Decoupling capacitors | 100nF ceramic per IC | Servo noise rejection |
| PWM signals | Frequency, duty cycle, pulse width | Servo control: 50Hz, 500–2400µs |
| I2C protocol | SDA/SCL, address, ACK/NACK | All sensors + PCA9685 |
| ADC conversion | 12-bit, reference voltage, sampling time | TCRT5000 line array |
| DMA transfers | Memory-to-peripheral without CPU | ADC1 circular DMA for IR array |

### 3. Git & Version Control
```bash
# Essential commands for this project:
git clone <repo>
git checkout -b feature/gait-engine    # Always branch for new features
git add firmware/Motion/gait_engine.c
git commit -m "feat: add trot gait diagonal phase"
git push origin feature/gait-engine
git pull origin main                   # Before starting new work
```

### 4. STM32CubeMX Setup
Learn to configure:
- **Clock**: HSE 25MHz → PLL → 100MHz SYSCLK
- **I2C1**: PB6(SCL), PB7(SDA), 400kHz, interrupt mode
- **I2C2**: PB10(SCL), PB3(SDA), 400kHz
- **ADC1**: IN0-8 (PA0-PA7, PB0), DMA1 Stream0 Ch0, circular
- **TIM2**: 20ms period (50Hz) → `update_flag` interrupt
- **TIM3**: 10ms period (100Hz) → IMU update interrupt
- **USART1**: PA9/PA10, 115200 baud, debug

## Practical Exercise
Write a bare-metal STM32 program that:
1. Blinks PC13 (built-in LED) at 1Hz using TIM2 interrupt
2. Sends `"Hello RUNNER-4\r\n"` via UART every 500ms
3. Reads a single ADC channel (PA0) and prints the value

## Prerequisite for Level 2
- Can write and compile a C program from scratch
- Understands pointers and structs
- Can generate a CubeMX project and add custom code in the `USER CODE` sections
- Can flash STM32 via ST-Link

---
🔙 **[Back to Curriculum](../README.md)**
