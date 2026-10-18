# RUNNER4 Webots line-following demonstration

This project contains a four-legged, 12-joint robot on a black arena with a white loop. It uses the active `RUNNER4_Robot_Line_03` design as its reference. Your requested chassis size is approximately **100 mm long × 60 mm wide**. The leg links use the sketch's **30 / 60 / 80 mm** values.

The robot appearance was rebuilt using your four hardware photographs and the assembly screenshots in `quadruped-spider-2.snapshot.2 (1).zip`. The active Webots robot uses the previous generated kinematic leg mechanism with diagonal brackets, curved tibia visuals, stacked boards, a battery/strap, coloured wires and the front IR strip. SolidWorks extraction utilities are kept separately for later reference but are not used by the active robot.

**Simulation type:** a sensor-driven kinematic demonstration, as agreed. Nine downward cameras simulate optical reflectance samples. The controller calculates steering from their rendered images. A Supervisor integrates forward speed and yaw into body position, while twelve joint motors animate a diagonal trot. Feet do not propel the body. This does not validate balance, friction, servo torque, weight, or the real robot's gait. The controller does not follow hidden waypoints.

## 1. Install and open on Windows

1. Install Webots from the [official Cyberbotics download page](https://cyberbotics.com/#download). The files target **R2025a syntax**; use R2025a or a newer compatible release.
2. Install 64-bit Python 3.10 or newer if necessary. No pip packages, Arduino libraries, ESP32 board, or USB connection are needed for this demo.
3. In Webots, open **Tools → Preferences → General → Python command** and select the full path to a working Python executable. On this PC, the interpreter successfully used for tests was:

   ```text
   C:\Users\ADMIN\AppData\Local\Programs\Python\Python310\python.exe
   ```

   The bare `python` command on this PC resolved to an unusable `C:\Windows\system32\python` entry. Set the explicit executable above rather than relying on that PATH entry. On another PC, use its own installed Python path. Python selection is described in the [Webots Python guide](https://cyberbotics.com/doc/guide/using-python).

4. Select **File → Open World** and open:

   ```text
   Automation codes\RUNNER4_Robot_Line_03\webots\worlds\runner4_white_line.wbt
   ```

5. Press Webots' **Run / real-time** button. The controller starts automatically. Watch the robot move to the right along the bottom straight, turn around the right end, and travel counter-clockwise.
6. Click the 3D view before using keyboard commands. One lap takes roughly **100 seconds of simulation time**. Fast simulation can shorten the wait.

Keep the whole `webots` folder together when copying it. There are no remote PROTO or texture dependencies. The world uses its sibling `protos` and `controllers` folders.

Optional launcher, from PowerShell in this folder:

```powershell
.\open_webots.ps1
# If Webots is installed elsewhere:
.\open_webots.ps1 -WebotsExe 'D:\Applications\Webots\msys64\mingw64\bin\webots.exe'
```

If PowerShell blocks scripts, simply open the world from Webots; changing execution policy is unnecessary.

## 2. Controls and expected behaviour

| Control | Effect |
|---|---|
| P | Pause/resume the robot controller; Webots itself continues running |
| R | Return to the start and clear stopped state, filters, and blackout test |
| L | Toggle artificial sensor blackout to test line loss |
| Webots Pause | Pause the entire simulation |
| Webots Reset/Reload | Restart the world and controller |

The on-screen label shows state, nine readings, line error, and speed. `W` means a white sample; `.` means a dark sample. Readings are ordered **S1 on the robot's left through S9 on its right**, independent of its heading on the screen. Console diagnostics print every two simulated seconds.

For a line-loss test, press L and leave it enabled for over three simulated seconds. Forward motion stops immediately; the robot may search slowly in its last observed line direction. After the timeout it latches `SAFE STOP`. Turning L off does not clear that latch; press R. If the line reappears before the timeout, following resumes.

To inspect the robot, pause and zoom/orbit with Webots' normal view controls. The default camera is overhead so the complete loop is visible. Scene tree `RUNNER4` contains the custom robot; its prototype has 12 motors and 9 cameras. Small camera overlays can be closed without disabling the sensors.

## 3. How it works

```text
Rendered black floor / white strip
       ↓
9 downward 3×3 pixel samples, 10 mm spacing
       ↓
Grayscale reflectance → EMA smoothing → threshold → weighted centroid
       ↓
PD steering + slow-down on large error + lost-line timeout
       ↓
Forward velocity / yaw → planar body motion + animated diagonal legs
```

The body coordinate frame is **+X forward, +Y left, +Z up**. Sensor positions are +40, +30, …, -40 mm across the body, mounted 85 mm forward of its centre. A positive error means the line is on the left; positive yaw turns left.

For active reflectance weights `w[i]` and lateral positions `y[i]`:

```text
error_mm = 1000 × sum(y[i] × w[i]) / sum(w[i])
yaw_rate = clamp(kp × error_mm + kd × change_in_error / dt)
x += speed × cos(heading + yaw_rate × dt/2) × dt
y += speed × sin(heading + yaw_rate × dt/2) × dt
heading += yaw_rate × dt
```

R2025a cameras look along local +X by default. Each sensor is explicitly rotated +90 degrees around Y so it looks down along world -Z from 12 mm above the floor. The main Viewpoint is also rotated downward, with world +X to screen right and +Y to screen up. This is an optical proxy for TCRT5000 reflectance, not its electrical transfer function. Cameras read actual scene pixels; only the offline tests use an analytic track model. Webots documents camera rendering in its [Camera reference](https://cyberbotics.com/doc/reference/camera).

The model has hip, femur, and tibia joints on each leg. Motor angles are radians relative to this simplified model's standing geometry. The 1.2-second diagonal animation uses FL+BR and FR+BL pairs. It is not a direct replay of the physical servo absolute angles. A null Physics field selects Webots kinematics mode; see the [Solid reference](https://cyberbotics.com/doc/reference/solid). Body pose updates use the [Supervisor API](https://cyberbotics.com/doc/reference/supervisor).

## 4. Files and tuning

| File | Purpose |
|---|---|
| `worlds/runner4_white_line.wbt` | Ready-to-open arena and robot instance |
| `protos/Runner4.proto` | Robot body, four articulated legs, twelve motors, nine cameras |
| `controllers/runner4_line/runner4_line.py` | Webots devices, keyboard, display, body motion |
| `controllers/runner4_line/core.py` | Sensor processing, steering, parameters, gait targets |
| `tools/build_world.py` | Reproducibly generates the world and robot PROTO |
| `tools/extract_cad.py` | Optional read-only SolidWorks display-tessellation extraction |
| `tools/normalize_cad.py` | Optional centering helper for extracted meshes |
| `tools/robot_visuals.py` | Photo-reference robot meshes, materials and electronics detail |
| `tools/preview_robot.py` | Optional static preview from the same neutral meshes (requires matplotlib) |
| `tests/test_controller.py` | Standalone control and asset checks |
| `PROJECT_ANALYSIS.md` | Repository findings and firmware-to-simulation mapping |
| `open_webots.ps1` | Optional Windows launcher |

Edit `Config` in `core.py`, save, then reset/reload the simulation:

| Parameter | Default | Meaning |
|---|---|---|
| `speed` | 0.045 | Forward speed in m/s |
| `kp` | 0.032 | Steering rad/s per mm error |
| `kd` | 0.0015 | Derivative gain, using elapsed seconds |
| `max_yaw_rate` | 0.65 | Maximum steering rad/s |
| `alpha` | 0.5 | EMA response; higher reacts faster |
| `threshold` | 0.45 | Minimum normalized reflectance for white |
| `lost_timeout` | 3.0 | Seconds before latched stop |

Begin at the defaults. If increasing speed, check curves and line loss before increasing it further. Excessive proportional gain causes oscillation. These gains have different units from the Arduino hip-angle correction and must not be copied into the firmware.

Arena: **2.4 × 1.8 m**, line width **30 mm**, curve radius **450 mm**, straight lengths **800 mm**. This is a small practice loop, not the full competition field. Border walls are visual; the controller applies a centre-position arena limit. There is no obstacle collision test.

Geometry edits belong in `tools/build_world.py`. From the `webots` folder, regenerate with:

```powershell
py -3.10 tools/build_world.py
```

This overwrites the generated `.wbt` and `.proto`. It does not overwrite the controller or Arduino source. If changing sensor coordinates or body height in `core.py`, regenerate the assets too. Reload Webots afterwards.

## 5. Validation and troubleshooting

Run standalone tests from this folder:

```powershell
py -3.10 -m unittest discover -s tests -v
```

Eight tests passed on Python 3.10: left/right/centre response, loss timeout and latch, reacquisition, joint target limits, asset device contracts, saved camera rays and track visibility, visual mesh outward winding and sensor clearance, and more than two idealized laps for three starting offsets (-20/0/+20 mm). The camera geometry test reads the generated rotations and track mesh; it confirms the initial centre-ray pattern is `...WWW...`. These tests do not invoke Webots or validate rendered optical measurements.

The first user run in Webots R2025a exposed an incompatible `solid` field in the generated `IndexedFaceSet`. Removing it cleared the parser error, but did not solve the blank view. The second run revealed the actual viewing problem: both the main Viewpoint and the sensors had identity rotation and looked horizontally along +X. On 19 October 2026, the generator, world, and robot PROTO were corrected to point the view and sensors downward. Geometric regression checks now cover camera-ray/floor intersections, the initial white-sensor pattern, and track face winding. A complete visual acceptance run is still required.

After updating, close and reopen `runner4_white_line.wbt` without saving the old in-memory world over the corrected file. Pressing R only resets the controller; it cannot reload changed world or PROTO files. The startup console should now say `optics-v2 (+X cameras rotated down)`.

First-run checks:

1. Confirm the world loads without red parser/device errors.
2. Confirm the initial sensor pattern detects the white line near the middle.
3. Observe two complete laps and watch for unwanted stops.
4. Test P, R, and the L blackout timeout.

| Symptom | Check |
|---|---|
| Python could not start | Set the full working `python.exe` path in Webots preferences |
| `No module named controller` in a terminal | Open the `.wbt` in Webots; do not run `runner4_line.py` directly or install an unrelated pip package named controller |
| Missing PROTO or controller | Preserve the complete folder structure; robot controller must be `runner4_line` |
| World version error | Use R2025a or a compatible newer Webots release |
| All sensors dark | Press R, ensure blackout is off, and inspect downward camera views; check camera height and white strip visibility |
| Keyboard does nothing | Click the 3D view to give it focus |
| Stops after L | Expected safety latch; press R |
| Slow rendering | Close camera overlays and use real-time mode; keep the 20 ms world timestep |

Colour classification, Wi-Fi pages, I2C devices, ToF, gripper, and competition mission logic are outside this first line-following demo. A future physical walker needs measured joint axes/zero offsets, masses/inertias, collision shapes, foot friction, and a validated stance/swing trajectory before removing Supervisor body motion.
