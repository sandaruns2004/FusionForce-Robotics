# 03 Camera Configuration

## Objective
The Pi Camera Module is the "eyes" of the robot. We must configure the Pi to interface with it and validate its operation.

## Hardware Verification
1. Lift the black tab on the Pi's CSI port.
2. Insert the ribbon cable with the bare contacts facing the HDMI ports.
3. Push the black tab down.
4. Ensure the camera is mounted at a 45-degree angle pointing downwards for optimal line tracking.

## Software Configuration
On newer Raspberry Pi OS versions, the `libcamera` stack has replaced the older `picamera` library. 
```bash
# Enable the camera interface
sudo raspi-config
# Navigate to Interface Options -> Camera -> Enable
```

## Camera Verification Script
Run a simple Python script to capture and save a frame. Validate that the resolution is set correctly. 640x480 at 60 FPS is recommended to keep processing latency low.

```python
import cv2

# Initialize the camera (0 is usually the Pi Camera)
cap = cv2.VideoCapture(0)

# Lower resolution for faster processing!
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 60)

ret, frame = cap.read()
if ret:
    cv2.imwrite('test_frame.jpg', frame)

cap.release()
```

## Debugging
- **`cv2.VideoCapture(0)` fails?** Run `libcamera-hello` in the terminal to verify the hardware is actually detected by the OS.
- **Image is upside down?** Use `cv2.flip(frame, -1)` in code to rotate it 180 degrees.
