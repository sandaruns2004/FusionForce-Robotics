# RISK REGISTER

## FMEA & Risk Analysis

| Risk | Domain | Probability | Impact | Detection | Mitigation | Recovery | Owner | Status |
|---|---|---|---|---|---|---|---|---|
| **Servo Brownout** | Power | High | Critical | STM32 / robot collapses mid-walk | Dedicated 5V/15A BEC for servos; separate 3.3V LDO for logic; 4700µF cap on servo rail | Reboot robot; ensure battery fully charged | Team B | In Progress |
| **Ground Loop Issue** | Power | Med | High | Erroneous sensor readings, I2C errors | Star grounding: all GNDs (BEC, LDO, STM32, sensors) to single point | Physically rewire | Team B | Open |
| **IR Array Ambient Light** | Sensors | Med | Med | Wrong centroid reading; false HIGH on black surface | Mount 5–8mm above floor; use digital threshold (not analog); adjust pots pre-competition | Re-tune threshold potentiometers | Team B | Open |
| **TCS34725 Ambient Light Wash** | Sensors | Med | High | UNKNOWN colour classification despite correct target | Use built-in LED illuminator (PC0 HIGH); 4× gain; 50ms integration; shroud sensor | Increase gain to 16× in 2-min prep; adjust arm angle | Team A | Open |
| **Intersection False Positive** | Navigation | Low | Med | Robot turns at wrong location mid-grid | Require ≥6 sensors for ≥3 consecutive cycles (60ms minimum) | Tighten to ≥7 sensors if needed after testing | Team A | Open |
| **Colour Memory Lost on Restart** | Firmware | Low | Critical | Robot reaches Task 4 with no stored colour → SAFE_STOP | Write `stored_ball_color` to STM32 Flash after Task 1 ID; read on boot | Flash read on restart; if 0xFF (blank), re-enter TASK1_COLOR_ID | Team B | Open |
| **STM32 Loop Timing Overflow** | Firmware | Low | High | 50Hz loop exceeds 20ms; servo jitter or missed cycles | Profile with DWT cycle counter; TCS34725 reads are non-blocking; ToF reads staggered | Reduce I2C clock retries; skip ToF alternate cycles | Team B | Open |
| **Gait Instability / Falling** | Mech | Med | High | Robot tips over while raising a leg | Static crawl gait keeps CoG within support polygon; mass <550g | IMU detects tilt >30° → emergency stop; competition restart | Team C | Open |
| **Gripper Malfunction** | Mech | Low | High | Ball slips during transit | Rubberised grip pads; servo maintains holding torque | Retry once; skip if persistent | Team C | Open |
| **Block Pushing Failure** | Mech | Low | Med | Robot lacks torque to push obstacle | Passive bumper maximises contact area; low body stance + wide stance during push | Reverse; retry with momentum | Team C | Open |
| **MG90S Torque Insufficient** | Mech | High | Critical | Robot collapses under its own weight in crawl | Mass strictly <550g; 6V servo supply; crawl only | Upgrade femur/tibia to MG996R hybrid | Team C | Open |
| **TCS34725 Wrong Arm Angle** | Mech/FW | Med | High | Sensor too far from target; UNKNOWN reading | Pre-calibrate MODE A (0°) and MODE B (−70°) PWM constants; verify before competition | Manually adjust arm angle in 2-min prep window | Team A | Open |
| **Line Array Dirt/Contamination** | Sensors | Med | Med | Wrong centroid or line-lost false trigger | Clean sensors with dry cloth before each run | Re-clean; re-check threshold | Team B | Open |
| **I2C Bus Hang (any device)** | Firmware | Low | High | STM32 HAL I2C timeout flag; loop stalls | Auto re-init I2C peripheral on timeout flag | If persistent: power-cycle sensors | Team B | Open |
