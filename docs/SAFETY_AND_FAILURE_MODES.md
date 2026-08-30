# SAFETY AND FAILURE MODES

## 1. Electrical Safety
- **Battery Fire Risk**: 3S LiPo batteries can combust if punctured or over-discharged. Never let a cell drop below 3.2V. Always use a balance charger.
- **Servo Stalls**: If a leg gets stuck against the wall, the servos will pull massive current, potentially melting the jumper wires or the PCA9685 traces. 
- **Mitigation**: A 10A automotive blade fuse is required between the UBEC and the PCA9685. A hardware kill switch must be easily accessible on the top of the chassis.

## 2. Software Safety (Failsafes)
- **UART Watchdog**: If the Raspberry Pi freezes (e.g., OpenCV memory leak), the robot must not continue walking blindly into a wall. The STM32 keeps a timestamp of the last valid UART packet. If 500ms elapse without a packet, the STM32 forcibly sets `Vx=0, Vy=0, Wz=0` and stops the gait generator.
- **I2C Watchdog**: If the I2C bus locks up (common with cheap MPU6050 clones), the STM32 must detect the HAL timeout and physically reset the I2C peripheral.

## 3. Failure Mode and Effects Analysis (FMEA)

| Component | Failure Mode | Effect on System | Detection Mechanism | Recovery |
|---|---|---|---|---|
| **Pi Camera** | Ribbon cable disconnects | Vision processing halts; robot blind. | Python `cv2.VideoCapture.read()` returns False. | Software triggers Emergency Stop over UART. Competition restart required. |
| **DS3218 Servo** | Stripped internal gear | One leg loses lifting power. Robot limps or tips over. | Visual observation. | Hardware replacement required. |
| **VL53L0X** | Dust on lens | Falsely reports object at 0mm. Robot backs up indefinitely. | Readings fluctuate wildly. | Wipe sensor. Rely on vision if ToF is erratic. |
| **MPU6050** | I2C Hang | Postural PID receives no updates. Robot may drift or tip. | STM32 I2C timeout flag. | Re-init I2C peripheral. |
| **Battery** | Undervoltage | Pi brownouts, SD card corruption. | Voltage alarm buzzer sounds. | Halt immediately. Swap battery. |
