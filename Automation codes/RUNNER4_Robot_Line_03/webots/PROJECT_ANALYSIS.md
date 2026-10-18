# Project analysis and simulation decisions

Analysis date: 19 October 2026. This is a repository architecture and active-code review for building the requested simulation, not a compile certification or complete hardware audit. Existing local changes were present; the original firmware files were left untouched.

## First Webots run correction

The first R2025a run showed `Skipped unknown 'solid' field in IndexedFaceSet node` at world line 14. The unsupported field was removed, clearing the parser error. The earlier conclusion that this explained the blank view was incorrect: the second screenshot showed the blank view and dark sensors persisted without any parser errors. Both the Viewpoint and sensor cameras had identity rotation; R2025a uses local +X as their forward viewing direction. The view at z=2.9 m was looking horizontally into empty space and the sensors looked parallel to the floor. Both orientations are now explicitly downward. Regression tests compute rays from the saved rotations and verify floor intersections, initial sensor classification, and upward track normals. The controller did start successfully and its lost-line timeout operated correctly; this was a camera-orientation defect in the generated scene.

## Repository context

| Area | Finding |
|---|---|
| Root README, `docs`, `Mechanical`, `Proposal` | Describe the broader RUNNER4 competition design, often using STM32F411 and 15 total servos |
| `firmware` | Separate C drivers and navigation code; not the selected ESP32 Arduino application |
| `Testing codes` | Component experiments, calibration sketches, older combined robot code |
| `Automation codes` | Successive startup, Wi-Fi, and line-following Arduino versions |
| `Automation codes/RUNNER4_Robot_Line_03` | User-selected current implementation; primary source for this simulation |
| `3D designs` | Native SolidWorks assemblies/parts; not directly converted or dimension-verified in this work |
| `Task` | Competition specification; includes a 30 mm white line and a more complex mission field |
| `Robot_Curriculum` | Learning material rather than executable simulation code |

The root README's 160×120 mm chassis is superseded for this model by your approximate **100×60 mm** measurement. The body thickness, brackets, and leg cross-sections are estimates. Links retain the active sketch's 30/60/80 mm geometry; this does not establish exact CAD fidelity.

## Active application

`RUNNER4_Robot_Line_03.ino` targets ESP32-S3, drives a PCA9685 at 50 Hz over GPIO11/12 I2C, hosts a Wi-Fi access point and HTTP controls, stores 13 pose presets, and provides geometric FK/IK helpers plus walk/turn motions. HTTP handlers queue jobs; `loop()` executes them. `serviceBackground()` polls the web server and sensors during interpolated motion waits.

The channel map used by this version is:

| Leg | Hip | Femur | Tibia | Webots motor names |
|---|---:|---:|---:|---|
| FL | 0 | 1 | 2 | FL_hip, FL_femur, FL_tibia |
| FR | 3 | 4 | 5 | FR_hip, FR_femur, FR_tibia |
| BL | 10 | 11 | 14 | BL_hip, BL_femur, BL_tibia |
| BR | 6 | 8 | 9 | BR_hip, BR_femur, BR_tibia |

Webots uses descriptive motor names; `core.py` retains the physical channel map for reference. It does not send PCA9685 PWM.

`line_sensor.cpp/.h` implement nine ADC channels on GPIO 1,2,4,5,6,7,8,9,10. Processing includes repeated ADC sampling, EMA, per-channel min/max calibration, selectable white polarity, thresholding, centroid, error and PD output. Default spacing is 10 mm; polling interval is 20 ms. Six active channels sustained over three cycles flag a junction, and a lost-line timer defaults to 3000 ms. The web pages expose calibration, graphs, configuration, and export functions.

`line_follow.h` computes a separate steering correction from `lineErrorMm()` (rather than directly using the sensor module's `lineWz()`). Defaults are 20-degree hip swing, 20-degree femur lift, six 200 ms phases, proportional gain 0.30, derivative gain 0.10, and ±15-degree correction. The derivative is a sample difference, without division by elapsed seconds. Left/right hip amplitudes vary around the base swing. It can slow on large errors, hold the last correction while line detection is absent, and stop after the sensor failsafe. After five cycles it explicitly transitions to INITIAL, then STANDING, before continuing.

`color_sensor.cpp/.h` handle TCS34725 I2C acquisition, calibration, ratio/HSV classification, voting and confirmed colour. `color_page.h` presents the lab interface. Main-loop RGB status uses GPIO16/17/18. `tof_sensor.h` currently implements **one** VL53L0X front sensor, calibration interpolation, and median sampling, with its own dashboard. `robot_page.h`, `line_page.h`, and `line_follow_page.h` are embedded browser interfaces, not Webots controllers.

## Findings that affect a faithful port

1. **Multiple hardware generations coexist.** STM32 pin maps, sequential servo channels, and three-ToF architecture from older documents must not override the selected sketch's current mappings.
2. **Sensor/steering orientation is inconsistent in comments.** The sensor header says S1-left to S9-right; the implementation comments describe S9-left and negate the centroid. Some following comments describe steering opposite to the side on which the line is said to lie. This cannot establish the physical mounting direction. The simulation explicitly uses S1-left, positive-left error and positive-left yaw, with direction tests. No firmware sign was silently changed.
3. **ToF comments are stale.** Main-file comments still refer to three sensors, but the module initializes one at 0x29. The colour module also uses 0x29 on the same bus. The ToF header itself warns about the address conflict when both are attached. The simulation does not emulate either device.
4. **ToF polling can block.** `_readMed()` performs five `readRangeSingleMillimeters()` calls with a 200 ms timing budget and configured timeout. Rate-limiting the call to 80 ms does not make these reads asynchronous. Since this polling also runs inside motion background service, responsiveness and gait timing need hardware measurement. The demo uses a fixed 20 ms update without hardware waits.
5. **Crawl is not wired into the selected sketch's dispatcher.** The folder contains `crawl_gait.h` and an enum value, but the inspected main file does not include/register the crawl module or handle `JOB_CRAWL`. The active line follower uses diagonal pairs.
6. **Servo absolute angles depend on assembly calibration.** Per-leg standing angles and signs reflect mounting offsets. They cannot be treated as universal mechanical joint rotations. This model uses a new neutral reference and a smooth animation; it does not replay these absolute presets.

## What the simulation preserves and changes

Preserved: four legs and twelve articulated joints; link lengths; channel/leg identity; nine samples with 10 mm spacing; white-on-black sensing; EMA/centroid/PD structure; diagonal gait grouping; 1.2-second gait period; a three-second lost-line safety latch.

Adapted: cameras provide ideal optical reflectance instead of ADC voltages; positive-left sign is explicit; derivative uses seconds; steering commands yaw rate rather than servo degrees; motion runs at 50 Hz; forward speed stops immediately on line loss and a bounded search precedes the latched stop. The simulated motion uses planar integration, not a measured relation between hip motion and displacement.

Omitted: firmware's five-cycle home/stand checkpoint, NVS settings, Wi-Fi/HTTP, physical servo offsets, hardware calibration workflow, ToF/colour/gripper devices, obstacle physics, and competition route decisions. A wide-line flag is diagnostic only and does not choose junction branches.

## Evidence and limits

The standalone suite checks steering direction, timeout/latch, reacquisition, joint limits, expected device names/counts and balanced asset delimiters. Its independent analytic stadium test completes over two laps for each of three initial offsets. Python modules were syntax-checked. These are offline checks, not a Webots parser, rendering, or physics test. See README for the required first Webots run checks and PC setup.
