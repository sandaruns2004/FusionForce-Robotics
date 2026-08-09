# 01. Environment Setup

## Objective
Establish the baseline operating system and software environment for the Raspberry Pi 4B, ensuring it can run headless Python scripts and interface with the camera.

## Reference Curriculum
- [11. Pi Linux and Python](../Robot_Curriculum/Module_3_Pi_Logic/11_pi_linux_and_python.md)

## Steps to Implement

1. **Operating System Installation**
   - Flash **Raspberry Pi OS (64-bit)** onto a high-speed MicroSD card.
   - Configure Wi-Fi and enable SSH during the flashing process (using Raspberry Pi Imager).

2. **Headless Access**
   - Power on the Pi and connect via SSH: `ssh pi@<raspberry-pi-ip>`
   - Alternatively, set up VNC for remote desktop access if visual debugging (e.g., viewing OpenCV frames) is required without an external monitor.

3. **Software Dependencies**
   - Update the package list: `sudo apt update && sudo apt upgrade -y`
   - Install Python dependencies required for the project:
     ```bash
     pip3 install numpy opencv-python pyserial
     ```

4. **Camera Verification**
   - Ensure the Camera Module 3 is connected to the CSI port.
   - Run a simple Python script using `cv2.VideoCapture(0)` to capture and save a frame.
   - Validate that the resolution is set correctly (640x480 at 60 FPS is recommended to keep processing latency low).

## Next Step
Proceed to [02. IPC Communication](./02_IPC_Communication.md) to establish serial links with the STM32.
