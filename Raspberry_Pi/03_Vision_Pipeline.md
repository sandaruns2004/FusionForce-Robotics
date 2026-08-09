# 03. Vision Pipeline (OpenCV)

## Objective
Process the live 60 FPS camera feed to extract line-following heading errors, detect grid intersections, and classify ball colors for Task 01 and Task 04.

## Reference Curriculum
- [14. OpenCV Fundamentals](../Robot_Curriculum/Module_4_Vision/14_opencv_fundamentals.md)
- [15. Advanced Vision](../Robot_Curriculum/Module_4_Vision/15_advanced_vision_apriltags.md)

## Steps to Implement

1. **Line Tracking Logic**
   - Capture frames using `cv2.VideoCapture`.
   - Convert the frame from BGR to **HSV color space** (`cv2.cvtColor`).
   - Create a binary mask that isolates the floor guideline (e.g., thresholding for black or white lines using `cv2.inRange`).
   - Use `cv2.findContours` and image moments (`cv2.moments`) to calculate the $(X, Y)$ centroid of the largest line contour.
   - Calculate the heading error ($e_{heading}$) based on how far the centroid X is from the center of the camera frame.

2. **Intersection Detection**
   - Apply horizontal morphological kernels or scan specific rows in the binary mask to detect the "crosses" that indicate grid nodes.
   - Set flags when an intersection is reached to trigger the State Machine to halt and scan.

3. **Color Classification Pipeline**
   - When stopped at a grid node, crop the image to a Region of Interest (ROI) where the ball is expected to sit.
   - Calculate the HSV histogram of this ROI.
   - Classify the dominant color as `RED`, `GREEN`, or `BLUE`.
   - Store this result in a persistent Python variable (e.g., `stored_ball_color`) for use in Task #04.

4. **Performance Tuning**
   - Ensure the pipeline executes well within the 33ms window (30 Hz). 
   - Down-sample the frame if necessary (e.g., to 320x240) to maintain high frame rates, as low latency is critical for smooth line following.

## Next Step
Proceed to [04. State Machine](./04_State_Machine.md) to link the Vision outputs to high-level robot states.
