# RUNNER-4 — Q&A Preparation (30 Questions + Answers)

---

### 1. Why did you choose a quadruped instead of a hexapod or biped?

A quadruped uses 12 servos versus 18 for a hexapod, saving 80g in servo weight, reducing power draw, and simplifying the control software. A hexapod would be difficult to fit within the 250 mm footprint with adequate leg workspace. A biped is impractical — it requires dynamic balance control far beyond what MG90S servos and a simple IMU can achieve. The quadruped with crawl gait (3 feet always on ground) provides guaranteed static stability with the minimum servo count.

### 2. Why 12 DOF? Why not 8 DOF (2 per leg)?

Three DOF per leg (coxa yaw, femur pitch, tibia pitch) provides full 3D foot positioning (x, y, z). A 2-DOF leg cannot produce lateral movement, body roll compensation, or adjust stride width — all critical for wall following, turning, and pushing stability. The third joint (coxa) also enables the robot to tuck its legs inward to meet the 250 mm footprint at the starting position.

### 3. Can MG90S servos actually support this robot?

At 6V, MG90S provides 2.2 kg·cm torque. Our estimated robot mass is ~550g. During crawl gait (3 stance legs), each femur joint bears approximately 0.77 kg·cm static torque. With a 2× dynamic safety factor, the requirement is ~1.53 kg·cm — within the 2.2 kg·cm rating with a 1.4× margin. This is tight but achievable if we keep mass under 550g and use crawl gait exclusively. We will validate this with a physical single-leg test before full assembly.

### 4. What if MG90S is not enough?

Our backup plan is a hybrid configuration: upgrade the 8 femur and tibia servos to MG996R (10 kg·cm, 55g each) while keeping MG90S for the 4 low-load coxa joints. This adds ~340g but provides a 5× torque margin. The PCA9685 and wiring support either servo — it's a drop-in swap.

### 5. Why one PCA9685 instead of two?

We use 15 of 16 channels (12 legs + 3 mechanism servos). One PCA9685 is sufficient. A second board would add 8g, an extra I2C address, and unnecessary complexity with zero benefit.

### 6. Why STM32F401 and not an Arduino?

The STM32F401 runs at 84 MHz with hardware floating-point (Cortex-M4F), enabling real-time IK computation for 4 legs and IMU sensor fusion within a 20ms control loop. An Arduino Uno (16 MHz, no FPU) cannot compute 12 joint angles plus IMU filtering at 50 Hz. The STM32 also provides multiple I2C and UART peripherals needed for simultaneous PCA9685, IMU, ToF, and Raspberry Pi communication.

### 7. Why Raspberry Pi 4 instead of Pi Zero 2 W?

The Pi Zero 2 W achieves only 8–12 FPS for our OpenCV pipeline (HSV + contours + centroid at 640×480). This is too slow for responsive line following. The Pi 4B provides 25–35 FPS with comfortable headroom. The weight penalty (37g extra) is acceptable. The Pi 4 also has a dedicated hardware UART (Pi Zero shares UART with Bluetooth) and USB ports for debugging.

### 8. Why not Raspberry Pi 5?

Pi 5 draws 3.5A peak (vs 2.5A for Pi 4) and requires active cooling — adding a fan increases weight, complexity, and failure risk. Our vision pipeline runs comfortably on Pi 4 at 25+ FPS. The Pi 5's extra performance provides no benefit for this competition while increasing power consumption and heat.

### 9. Why a camera instead of IR sensors for line following?

A single camera replaces three types of sensors: IR line sensors (line following), colour sensors (ball colour detection), and dedicated ball detectors. This reduces wiring, weight, and sensor integration complexity. The camera also provides richer information — junction detection, obstacle recognition, and colour sorting — that discrete sensors cannot. The trade-off is higher software complexity and power consumption, but the Pi 4B handles this well.

### 10. How do you detect the ball's colour?

We convert camera frames to HSV colour space and apply separate masks for Red (H=0–10 and 165–179), Green (H=40–85), and Blue (H=95–135), all with high saturation (S>100) and moderate value (V>70). We count pixels in each mask and classify the colour with the highest qualifying count, requiring 2× dominance over the next-highest colour for confident classification.

### 11. How do you handle the ball being on a 5 cm pedestal?

The ball sits 50 mm above the ground. A floor-level scoop or funnel cannot reach it. Our 2-servo arm has a pitch joint that lowers the gripper from body level down to pedestal height. The arm reach is ~50–60 mm from pivot to gripper, sufficient to reach the ball. The gripper opens to >40 mm to accommodate the 40 mm diameter ball.

### 12. How do you store the ball securely?

After grasping, the arm lifts the ball over the body and drops it into an internal belly compartment with an opening at the top. A servo-controlled gate locks the ball inside. The compartment walls are fully enclosed — the ball cannot escape during walking, turning, wall following, or pushing.

### 13. How do you release the ball in Subtask 4?

The compartment has a servo-operated gate at the bottom. To release, the robot positions over the target colour zone, opens the gate, and the ball drops by gravity. The same servo that locks the gate during storage opens it for release — no additional actuator.

### 14. Why a passive bumper instead of a robotic arm for pushing?

A passive bumper has zero additional actuators, zero extra weight, zero additional failure modes, and zero extra power consumption. The quadruped's legs provide the pushing force through the crawl gait — the bumper simply transmits this force to the obstacle. A robotic arm for pushing would add complexity without benefit.

### 15. How do you stabilize the robot during pushing?

Before pushing, the robot lowers its body height from ~70 mm to ~45 mm (reducing the tipping moment), widens its leg stance (increasing the support polygon), and switches to slow crawl gait at 50 mm/s. The IMU continuously monitors pitch — if pitch exceeds 15°, the controller reduces push speed to prevent tipping.

### 16. How do 3 ToF sensors handle wall following with gaps?

We use a gap rejection filter. During normal wall following, the lateral error is e = d_left − d_right. When a gap appears (one sensor reads >400 mm), the robot switches to single-wall following using only the solid wall sensor. The filter requires 3 consecutive out-of-range readings before switching, preventing noise-triggered false gaps. When both walls return, normal dual-wall centering resumes.

### 17. What if both walls have gaps simultaneously?

If both sensors read >400 mm simultaneously, the robot holds its current heading using the IMU gyroscope and continues straight until at least one wall reappears. This is a valid scenario because the competition rules indicate gaps may appear on either side.

### 18. How do you guarantee the robot is fully autonomous?

No wireless modules are active. Wi-Fi and Bluetooth are disabled at the OS level (`dtoverlay=disable-wifi` and `dtoverlay=disable-bt` in `/boot/config.txt`). The Pi has no Wi-Fi or Bluetooth functionality during operation. All processing occurs onboard. The robot uses only UART (wired, internal) between Pi and STM32. There is no laptop, phone, or remote controller dependency.

### 19. How do you meet the 250 × 250 mm footprint?

The body is designed at approximately 160 × 140 mm. Before competition, we measure the standing footprint using a jig/template. The coxa angles at the starting position are adjusted to keep all feet within 250 × 250 mm. During the run, the legs extend beyond this footprint during walking, which the rules explicitly permit ("Expansion during the run is allowed").

### 20. What is your biggest risk?

MG90S servo torque margin. At 1.4× safety margin for crawl gait, there is limited headroom for mechanical imperfections, heavier-than-estimated 3D prints, or higher friction conditions. Mitigation: keep mass under 550g, validate with a physical leg test early, and have MG996R hybrid as a defined backup path.

### 21. What happens if the camera fails mid-run?

The STM32 detects UART command timeout (no commands from Pi for 500 ms) and enters SAFE_STOP mode — all servos hold current position. The Pi attempts to reinitialise the camera. If vision is permanently lost, the robot can switch to ToF-only degraded navigation for corridor tasks (wall following still works). Ball and colour tasks would fail without camera.

### 22. What happens if a servo stalls?

The STM32 monitors IMU data — if servo commands are issued but no body movement is detected (no angular change in IMU), a stall is suspected. The controller reduces the gait amplitude, retries the movement, and if the stall persists, enters SAFE_STOP with a red status LED.

### 23. What happens if the battery voltage drops?

The STM32 monitors battery voltage via ADC voltage divider. At 6.8V (3.4V/cell), the status LED flashes red and servo speed is reduced. At 6.4V (3.2V/cell), the robot enters SAFE_STOP to prevent LiPo damage. Expected runtime is ~24 minutes with 1300mAh — well above the 15-minute competition limit.

### 24. How do you calibrate within 2 minutes?

Calibration is mostly pre-loaded in firmware (servo trim offsets, HSV thresholds, ToF offsets). During the 2-minute prep:
1. Power on → automatic self-test (3 seconds)
2. Status LED confirms ready (green = all OK)
3. Place robot on start line
4. Minor sensor adjustments only if needed (ToF angle)
No code changes are made at competition.

### 25. How do you handle a restart?

The state machine has a defined START state. On restart, the robot returns to START and begins from Subtask 1. The `stored_ball_color` variable is reset. The timer continues running (not paused). Up to 3 restarts are allowed within 10 minutes.

### 26. Why crawl gait and not trot?

Trot gait requires each femur servo to support 50% of robot weight (2 legs in stance). At our estimated mass, this demands 2.3 kg·cm — exceeding the MG90S 2.2 kg·cm rating. Crawl gait requires only 1.53 kg·cm per femur (3 legs in stance), which is within the servo's capability. Crawl is slower but the competition evaluates completion, not speed.

### 27. How does the colour junction work in Subtask 4?

The camera detects coloured lines emerging from the main white line. The robot recalls `stored_ball_color` from memory (stored in Subtask 1). It identifies the matching coloured branch using HSV segmentation (same thresholds used for ball detection). The robot turns onto the matching branch, navigates to the drop zone, releases the ball, reverses back to the main line, and continues to the finish.

### 28. What is your power architecture?

2S LiPo (7.4V) → toggle switch → 10A fuse → three isolated rails:
1. **5V/15A BEC** → PCA9685 V+ → all 15 servos (with 4700µF bulk capacitor)
2. **5V/3A buck regulator** → Raspberry Pi 4B (via GPIO 5V pins)
3. **STM32 VBUS** → onboard 3.3V LDO → STM32 logic + MPU6050 + 3× VL53L0X

Servo power NEVER flows through the STM32 or PCA9685 logic supply.

### 29. What is your backup plan overall?

- **Servo failure**: MG996R hybrid swap (pre-designed)
- **Camera failure**: ToF-only degraded mode (wall following still works)
- **Pi crash**: STM32 watchdog → SAFE_STOP; use restart allowance
- **Ball pickup failure**: Retry once; skip ball (partial score, continue other tasks)
- **Competition restart**: 3 restarts available within 10 minutes

### 30. How do you verify ball release in Subtask 4?

After opening the compartment gate, the camera checks a downward ROI to see if the ball is visible on the ground beneath the robot. If the ball is not detected within 3 seconds, the gate opens/closes again and the body tilts forward to assist gravity. If still unsuccessful after 2 attempts, the robot logs the failure and continues to the finish.
