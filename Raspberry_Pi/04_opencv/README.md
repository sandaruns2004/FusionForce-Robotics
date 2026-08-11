# 04 OpenCV Basics & Line Tracking

## Objective
Process the live 60 FPS camera feed to extract line-following heading errors.

## Line Tracking Logic
1. **Capture**: Grab frames using `cv2.VideoCapture`.
2. **Color Space**: Convert the frame from BGR to **HSV color space** (`cv2.cvtColor`). HSV handles lighting variations much better than BGR.
3. **Thresholding**: Create a binary mask that isolates the white floor guideline against the black background using `cv2.inRange()` or `cv2.threshold()`.
4. **Contours**: Use `cv2.findContours` to identify the outlines of the white line blobs.
5. **Centroid**: Calculate the Image Moments (`cv2.moments(c)`) of the largest contour to find the `(X, Y)` centroid.
6. **Error Calculation**: Calculate the heading error ($e_{heading}$) based on how far the centroid X is from the center of the camera frame (`dx = center_x - cx`).

## Code Implementation
```python
import cv2

# 1. Read image
img = cv2.imread('test_frame.jpg')

# 2. Convert to Grayscale
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

# 3. Threshold (values > 200 become 255)
ret, thresh = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

# 4. Find Contours
contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 5. Calculate Centroid and Error
if len(contours) > 0:
    c = max(contours, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] != 0:
        cx = int(M["m10"] / M["m00"])
        error_x = 320 - cx # Assuming 640x480 resolution
        print(f"Heading Error: {error_x}")
```

## Performance Tuning
Ensure the pipeline executes well within the 33ms window (30 Hz minimum). Down-sample the frame to 320x240 if the Raspberry Pi CPU begins to lag. Low latency is critical for smooth line following.
