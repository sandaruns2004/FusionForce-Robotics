# 05 Perception Algorithms & Intersections

## Objective
Detect grid intersections (for Task 01) and classify ball colors for Task 01 and Task 04.

## Intersection Detection
While tracking the line, apply horizontal morphological kernels or scan specific horizontal rows (slices) in the binary mask to detect the "crosses" that indicate grid nodes. 
A straight line has 1 intersection at a scan row; a T-junction or cross will have a much wider bounding box or multiple distinct lines entering the frame sides.
Set flags when an intersection is reached to trigger the State Machine to halt and scan.

## Color Classification Pipeline
1. **Halt & Scan**: When stopped at a grid node, crop the image to a Region of Interest (ROI) where the ball is expected to sit.
2. **HSV Conversion**: Convert the ROI to HSV color space.
3. **Masking**: Create strict `cv2.inRange()` masks for RED, GREEN, and BLUE.
4. **Validation**: Use `cv2.HoughCircles` on the color mask to ensure the detected shape is a sphere, eliminating background noise.
5. **Memory**: Classify the dominant color and store this result in a persistent Python variable (e.g., `stored_ball_color = "RED"`) so the robot remembers it for the final sorting junction in Task 04.

## Code Implementation
```python
import cv2
import numpy as np

# HSV Color Filtering (Looking for RED)
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Red wraps around the Hue cylinder (0 and 180)
lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
red_mask = cv2.inRange(hsv, lower_red1, upper_red1) + cv2.inRange(hsv, lower_red2, upper_red2)

# Clean up mask
red_mask = cv2.erode(red_mask, None, iterations=2)
red_mask = cv2.dilate(red_mask, None, iterations=2)

# Verify shape
circles = cv2.HoughCircles(red_mask, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=10, maxRadius=50)
```

## Debugging
- **Lighting changed?** Always use HSV. Never hardcode HSV limits permanently. Write a script with Trackbars to tune them on the actual competition floor.
