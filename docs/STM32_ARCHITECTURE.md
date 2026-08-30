# STM32 LOW-LEVEL ARCHITECTURE

## 1. Role in the System
The STM32F411 (Black Pill) is the deterministic heart of the robot. While the Raspberry Pi decides *where* to go, the STM32 decides *how* to move the legs to get there without falling over. It runs a single, continuous, real-time control loop.

## 2. Firmware Architecture
The codebase should be structured using STM32CubeMX HAL (Hardware Abstraction Layer) in C.

```text
firmware/
├── Core/
│   ├── Src/main.c
│   ├── Src/stm32f4xx_it.c   (Interrupts)
│   └── Src/system.c
├── Drivers/
│   ├── PCA9685/             (I2C PWM)
│   ├── VL53L0X/             (I2C ToF)
│   └── MPU6050/             (I2C IMU)
├── Control/
│   ├── kinematics.c         (Forward & Inverse Kinematics)
│   ├── gait.c               (Bezier trajectory generator)
│   └── pid.c                (Postural stabilization)
└── Comms/
    └── uart_parser.c        (Pi protocol handler)
```

## 3. The 50Hz Main Control Loop
The servos expect a PWM signal at 50Hz (every 20ms). Therefore, the entire control loop must execute in under 20ms.

1. **Hardware Timer Interrupt (`TIM2`) fires every 20ms.**
2. Set a flag `update_gait = 1`.
3. In the `while(1)` main loop:
   ```c
   if (update_gait) {
       update_gait = 0;
       
       1. Read_IMU();
       2. Read_UART_Commands_Buffer();
       
       3. Run_PID_Stabilization();
       
       4. Calculate_Next_Foot_Positions(Vx, Vy, Wz, time_t);
       
       5. Calculate_Inverse_Kinematics();
       
       6. Send_I2C_PWM_To_PCA9685();
   }
   ```

## 4. Hardware Peripherals Used
* **I2C1**: Master bus for PCA9685, MPU6050, and ToF sensors. Run at 400kHz Fast Mode to minimize blocking delay.
* **USART1 or USART2**: Communication with Raspberry Pi. Configured at 115200 baud. Must use `UART_Receive_IT` (Interrupt mode) or DMA to prevent dropping bytes.
* **TIM2**: Hardware timer configured to generate a 50Hz interrupt for the control loop pacing.
* **TIM3**: (Optional) Hardware PWM generation for onboard debug LEDs or a buzzer.

## 5. Failsafes & Safety
* **UART Timeout**: Maintain a `last_uart_time` variable. If `(current_time - last_uart_time) > 500ms`, immediately override `Vx = 0, Wz = 0` and command the robot to a resting stance.
* **I2C Watchdog**: I2C buses can freeze if a wire is jostled. Implement an I2C timeout. If the bus hangs, reinitialize the I2C peripheral in software automatically.
