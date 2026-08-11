# COMPUTER VISION

## 1. Overview
The Computer Vision (CV) subsystem resides entirely on the Raspberry Pi 4B. It utilizes the Python `OpenCV` library to interpret the environment and provide actionable vectors to the decision-making layer.

## 2. Requirements & Tasks
- **Task 01**: Follow a 3cm white line on a black background, detect intersection crosses, detect a 4cm colored ball, and identify if the ball is Red, Blue, or Green.
- **Task 02**: Detect wall boundaries to assist ToF sensors in navigating gaps.
- **Task 04**: Identify color-coded floor paths (Red, Blue, Green) to match the memorized ball.

## 3. Hardware Configuration
- **Camera**: Raspberry Pi Camera Module 3.
- **Mounting**: Angled downwards at ~45 degrees to view the floor immediately ahead (approx 10cm to 40cm lookahead).
- **Resolution**: 640x480 or 320x240 to minimize processing time. Target: 30 FPS.

## 4. The Vision Pipeline

### Step 1: Image Acquisition
```python
import cv2
cap = cv2.VideoCapture(0)
ret, frame = cap.read()
```

### Step 2: Perspective Transformation (Bird's Eye View)
Because the camera is angled down, parallel lines appear to converge.
- Apply `cv2.getPerspectiveTransform()` and `cv2.warpPerspective()` to warp the trapezoidal Region of Interest (ROI) into a top-down flat rectangle.
- *Why:* This makes calculating steering angles linear and accurate.

### Step 3: Line Tracking (Tasks 01, 04)
- **Grayscale Conversion**: `cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)`
- **Thresholding**: Apply `cv2.threshold` to isolate the white lines (pixel values > 200).
- **Contour Extraction**: `cv2.findContours()` to find the largest white blobs.
- **Centroid Calculation**: Calculate Image Moments (`M = cv2.moments(c)`) to find the `(cx, cy)` of the line.
- **Output**: An error vector `dx = frame_center_x - cx`. This is passed to the state machine as the steering command.

### Step 4: Intersection Detection (Task 01)
- While tracking the line, count the number of exits in the contour. A straight line has 2 exits (top and bottom of frame). A cross intersection has 3 or 4 exits.
- When an intersection is detected, halt forward movement and initiate a localized search for the ball.

### Step 5: Color & Ball Detection (Task 01, 04)
- **Color Space**: Convert image to HSV (`cv2.COLOR_BGR2HSV`). Do NOT use BGR/RGB for color detection as it is highly susceptible to lighting changes.
- **Masking**: 
  - Create three separate masks using `cv2.inRange()` for Red, Green, and Blue HSV thresholds.
  - Combine with morphological operations (`cv2.erode` and `cv2.dilate`) to remove noise.
- **Shape Validation**: Apply `cv2.HoughCircles` on the color mask to ensure the detected color is actually a spherical ball and not background noise.
- **Color Memorization**: Once the highest confidence circle is found, store the dominant color string ("RED") in a global variable for use in Task 04.

## 5. Failure Modes & Mitigations
- **Motion Blur**: If the robot shakes heavily while walking, the image blurs. *Mitigation:* Increase shutter speed in picamera settings, and heavily low-pass filter the centroid output vector.
- **Lighting Changes**: Shadows across the arena can break fixed RGB thresholding. *Mitigation:* Strict use of HSV space, and running a pre-competition calibration script to lock the `H` (Hue) bounds.
- **Processing Lag**: If CV processing takes >33ms, FPS drops. *Mitigation:* Downsample the resolution to 320x240 and only process the bottom 50% of the image (ROI).

## 6. Implementation Steps for Vision Team
1. **Learn**: Complete `/Raspberry_Pi/04_opencv`.
2. **Setup**: Mount camera and write script to stream video to a laptop for tuning.
3. **Calibrate**: Write a Python script with trackbars to tune HSV min/max values for the arena lighting.
4. **Develop**: Implement the pipeline described above.
5. **Test**: Print a 3cm white line on black paper and verify the script outputs correct `dx` steering values.
