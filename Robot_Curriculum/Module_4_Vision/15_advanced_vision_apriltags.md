# Module 4: Advanced Vision with AprilTags

Tracking a colored ball is fun, but it's not very useful for precise robotics. What if the robot needs to walk exactly 10 cm away from a charging dock and turn around? A colored ball can't tell you the angle or exact distance.

Enter **Fiducial Markers**, specifically **AprilTags**.

## What is an AprilTag?

An AprilTag looks like a simplified QR code. It is a 2D barcode printed on paper.
Because it has a very specific, known geometry (a high-contrast black square with a specific inner pattern), a computer vision algorithm can detect it with extreme accuracy and speed.

## The Magic of Pose Estimation

Unlike color tracking which just gives you a 2D (X, Y) coordinate on your screen, an AprilTag gives you **6 Degrees of Freedom (6D Pose)**.

Because the software knows the tag is perfectly square in real life, it looks at how the tag is skewed or warped in the camera image to mathematically calculate:
1.  **Distance (Z)**: How far away the tag is in millimeters.
2.  **Translation (X, Y)**: How far left/right and up/down the tag is.
3.  **Rotation (Pitch, Roll, Yaw)**: Exactly what angle the tag is facing relative to the camera.

### Real-World Application
You stick an AprilTag ID=0 on your robot's charging station.
The Raspberry Pi sees the tag.
The software outputs: `Distance: 50cm, Angle: 15 degrees right, Tilt: 0 degrees`.
Your State Machine passes these precise measurements to your Inverse Kinematics algorithm, which perfectly calculates a trot gait to walk 50cm forward and turn 15 degrees.

## Implementing AprilTags in Python

AprilTags are notoriously difficult to write from scratch, but there are excellent Python libraries that wrap the C-level code.
You can install the library via pip: `pip install pupil-apriltags`

```python
import cv2
from pupil_apriltags import Detector

# Initialize the camera and the detector
cap = cv2.VideoCapture(0)
detector = Detector(families='tag36h11') # Standard AprilTag family

while True:
    ret, frame = cap.read()
    # AprilTags only work on grayscale images!
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Detect the tags
    results = detector.detect(gray)
    
    for r in results:
        # Extract the bounding box and center
        (ptA, ptB, ptC, ptD) = r.corners
        center_x, center_y = r.center
        
        # Print the tag ID
        print(f"Detected Tag ID: {r.tag_id}")
        
        # To get distance/pose, you must calibrate your camera lens first!
        # r.pose_t contains the 3D translation matrix
        
    cv2.imshow("Frame", frame)
    if cv2.waitKey(1) == ord('q'): break
```

## 📺 Recommended Viewing
*   Search YouTube for: `"AprilTags Python OpenCV tutorial"`
*   Search YouTube for: `"Camera Calibration OpenCV"` (Crucial: Pose estimation only works if you calibrate your specific camera lens to remove fish-eye distortion).
