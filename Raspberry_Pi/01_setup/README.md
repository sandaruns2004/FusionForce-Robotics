# 01 Setup & Environment Initialization

## Objective
Establish the baseline operating system and software environment for the Raspberry Pi 4B, ensuring it can run headless Python scripts and interface with the camera.

## Operating System Installation
- Flash **Raspberry Pi OS (64-bit)** (or Ubuntu) onto a high-speed MicroSD card using the Raspberry Pi Imager.
- Configure Wi-Fi and enable SSH directly during the flashing process (using the advanced settings cog in the imager).

## Headless Access
- Power on the Pi (Connect to 5V/3A power supply, like the Buck Converter). Do not power from laptop USB.
- Connect via SSH: `ssh pi@<raspberry-pi-ip>`
- Alternatively, set up **VNC (Virtual Network Computing)** for remote desktop access. This is highly recommended for visual debugging (e.g., viewing OpenCV frames) without needing to attach an external HDMI monitor to the robot.

## Software Dependencies
Run the following commands to update the system and install the required tools:
```bash
# Update the package lists
sudo apt update && sudo apt upgrade -y

# Install essential Python tools
sudo apt install python3-pip python3-venv -y

# Create and activate a virtual environment
python3 -m venv fusion_env
source fusion_env/bin/activate

# Install project dependencies
pip3 install numpy opencv-python pyserial
```

## Hardware Setup
- Insert the flashed MicroSD card into the Pi.
- Connect power and wait 60 seconds before attempting SSH.

## Debugging
- **Cannot SSH?** Ensure both your laptop and Pi are on the same WiFi network, or use a crossover Ethernet cable.
- **Under-voltage warning?** The Pi requires at least 3 Amps to safely run OpenCV without crashing.
