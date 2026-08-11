# 10 Final Deployment Curriculum

## Concept
The competition has started. You place the robot down. You cannot SSH into it. The code must run automatically the moment the power switch is flipped.

## Explanation
We use Linux `systemd` to create a service. This tells the OS: "As soon as you finish booting, activate this Python virtual environment, and run `main.py`."

## Commands & Code
Create a file: `sudo nano /etc/systemd/system/robot.service`

```ini
[Unit]
Description=FusionForce Robot Control
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/FusionForce-Robotics/Raspberry_Pi
ExecStart=/home/pi/fusion_env/bin/python3 main.py
Restart=always

[Install]
WantedBy=multi-user.target
```

Enable the service:
```bash
sudo systemctl enable robot.service
sudo systemctl start robot.service
```

## Hardware Setup
- Ensure the main battery kill switch is easily accessible.
- Disconnect all external monitors and keyboards.

## Practical Exercise
1. Create the `systemd` service for a simple script that blinks an LED.
2. Reboot the Pi.
3. Verify the LED starts blinking automatically without you touching the computer.

## Debugging
- **Script fails on boot?** Check the logs using `journalctl -u robot.service -f`.
- **OpenCV crashes on boot?** Sometimes the service starts *before* the camera hardware is fully initialized. Add `ExecStartPre=/bin/sleep 5` to delay the script start.

## Robot Application
This is the final step. Once this works, the robot is truly autonomous and ready to execute the BREACH PROTOCOL circuit.
