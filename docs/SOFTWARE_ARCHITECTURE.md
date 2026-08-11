# SOFTWARE ARCHITECTURE

## 1. Domain Separation
The software system is distributed across two vastly different computing environments. They communicate exclusively over a predefined UART Interface Contract.

## 2. High-Level Software (Raspberry Pi 4B)
The Raspberry Pi runs Ubuntu or Raspberry Pi OS (Debian based). Its primary language is Python 3.

### Directory Structure Hypothesis
```text
raspberry_pi/
├── main.py                  # Entry point, initializes threads
├── vision/
│   ├── line_tracker.py      # OpenCV Contour/Centroid math
│   ├── color_detector.py    # HSV filtering & Hough circles
│   └── camera_config.json   # Distortion/perspective matrices
├── perception/
│   └── spatial_mapper.py    # Integrates ToF (from UART) with Vision
├── decision/
│   ├── state_machine.py     # HFSM (Task 01 -> 04 transitions)
│   └── task_planner.py      # Translates state to velocity vectors
├── communication/
│   └── uart_transceiver.py  # Threaded UART reader/writer + CRC
└── tests/
    └── mock_stm32.py        # Simulates STM32 for Pi-only testing
```

### Process Flow (Pi)
1. `main.py` spawns three primary threads: **Vision Thread**, **Logic Thread**, and **UART Thread**.
2. **Vision Thread**: Continuously pulls frames from `picamera`, applies perspective warp, thresholds for lines and colors, and updates a shared `WorldState` object with vector offsets.
3. **Logic Thread**: Reads `WorldState`. Depending on the current HFSM state (e.g., `TASK_01`), it calculates the required forward velocity `Vx` and angular velocity `Wz` to keep the line centroid centered.
4. **UART Thread**: Asynchronously packages `[Vx, Vy, Wz, Action]` into the binary protocol, calculates the CRC, and sends it to the STM32. It also reads incoming status packets (e.g., ToF distances) and updates `WorldState`.

## 3. Low-Level Firmware (STM32)
The STM32 runs Bare-Metal C code generated initially by STM32CubeMX, utilizing the Hardware Abstraction Layer (HAL).

### Directory Structure Hypothesis
```text
firmware/
├── Core/
│   ├── Src/main.c
│   ├── Src/stm32f4xx_it.c   # Interrupt Handlers
│   └── ...
├── Drivers/
│   ├── PCA9685/             # I2C PWM Driver
│   ├── VL53L0X/             # I2C ToF Driver
│   └── MPU6050/             # I2C IMU Driver
├── Control/
│   ├── kinematics.c         # Forward/Inverse Kinematics Math
│   ├── gait_generator.c     # Trot/Crawl Bezier curve generation
│   └── pid_posture.c        # MPU6050 feedback loop
└── Communication/
    └── protocol_parser.c    # UART RX/TX ring buffer handling
```

### Process Flow (STM32)
1. **Interrupts (Asynchronous)**: 
   - `UART_Rx_ISR` fires when the Pi sends a byte. It's placed into a Ring Buffer.
   - `TIM2_ISR` fires at 50Hz (every 20ms) to trigger the Main Control Loop.
2. **Main Control Loop (Synchronous)**:
   - Poll IMU for Roll/Pitch.
   - Parse UART ring buffer for new `Vx, Wz` targets.
   - Update Gait Phase (Time `t` inside the step cycle).
   - Calculate desired `(X, Y, Z)` foot positions based on Gait Phase and `Vx, Wz`.
   - Add postural correction offsets to `(X, Y, Z)` from IMU PID.
   - Run Inverse Kinematics to calculate 12 joint angles (`theta1, theta2, theta3` for each leg).
   - Convert joint angles to PWM ticks.
   - Send PWM values to PCA9685 via I2C blocking/DMA call.

## 4. End-to-End Data Flow
1. Camera sees a left curve.
2. OpenCV calculates the centroid is at `x = -50` pixels.
3. Pi Logic Thread computes `Wz = +0.5 rad/s`.
4. Pi UART Thread sends `<HEADER><VEL><Vx=0.2><Wz=0.5><CRC>`.
5. STM32 UART ISR receives packet and validates CRC.
6. STM32 Gait Generator increases step length on right legs and decreases on left legs to induce a turn.
7. STM32 IK converts step vectors to joint angles.
8. PCA9685 moves the servos.
9. Robot physically turns left, centering the line in the camera's next frame.
