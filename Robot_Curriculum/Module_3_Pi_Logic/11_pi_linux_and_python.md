# Module 3: Raspberry Pi, Linux, & Python Setup

The Raspberry Pi is the High-Level brain of the robot. It does not run real-time motor loops. Instead, it runs a full Linux operating system to handle computationally heavy tasks like AI, Computer Vision, and Kinematics math.

## Headless Setup

Because this Pi will be buried inside the body of a robot, you won't have a monitor, keyboard, or mouse attached to it. You must control it "headlessly" over Wi-Fi.

### Step 1: Flashing the OS
1. Download the **Raspberry Pi Imager** on your PC.
2. Insert a microSD card.
3. Choose the OS: **Raspberry Pi OS (Legacy, 64-bit)** is usually the most stable for OpenCV and Python libraries.
4. **CRITICAL**: Before clicking write, click the gear icon (Advanced Settings).
   *   Enable SSH (Use password authentication).
   *   Set a username (e.g., `pi`) and password.
   *   Configure your home Wi-Fi SSID and Password.

### Step 2: Connecting via SSH (Secure Shell)
Once the Pi boots up and connects to your Wi-Fi, you need to access its terminal.

1. Open PowerShell or Command Prompt on your Windows PC.
2. Type: `ssh pi@raspberrypi.local` (or use the Pi's specific IP address).
3. Enter your password. You are now controlling the Pi's terminal!

## Python Virtual Environments

Python libraries frequently conflict with each other. It is a terrible practice to install everything globally using `sudo pip install`. You must use Virtual Environments (venvs).

### Setting up the Robot Environment
Run these commands in the Pi's terminal:

1.  Update the system:
    `sudo apt update && sudo apt upgrade -y`
2.  Create a folder for the project:
    `mkdir quadruped_robot && cd quadruped_robot`
3.  Create the virtual environment:
    `python3 -m venv env`
4.  Activate the environment:
    `source env/bin/activate`
    *(Notice the `(env)` prefix appear in your terminal prompt. This means you are safely inside the bubble).*
5.  Install required libraries:
    `pip install pyserial numpy opencv-python`

## Auto-Starting Code on Boot (systemd)

When you turn the robot on, you don't want to have to SSH into it every time just to start the Python script. You want the script to run automatically as soon as the Pi boots.

We do this using a Linux service called `systemd`.

1.  Create a service file:
    `sudo nano /etc/systemd/system/robot.service`
2.  Paste in this configuration (adjusting paths as needed):
    ```ini
    [Unit]
    Description=Quadruped Main AI Loop
    After=network.target

    [Service]
    ExecStart=/home/pi/quadruped_robot/env/bin/python /home/pi/quadruped_robot/main.py
    WorkingDirectory=/home/pi/quadruped_robot/
    StandardOutput=inherit
    StandardError=inherit
    Restart=always
    User=pi

    [Install]
    WantedBy=multi-user.target
    ```
3.  Enable and start the service:
    `sudo systemctl enable robot.service`
    `sudo systemctl start robot.service`

## 📺 Recommended Viewing
*   Search YouTube for: `"Raspberry Pi headless setup Windows"`
*   Search YouTube for: `"Core Electronics run python script on boot raspberry pi systemd"`
