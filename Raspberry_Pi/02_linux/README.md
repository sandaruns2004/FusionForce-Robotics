# 02 Linux Fundamentals Curriculum

## Concept
The Raspberry Pi runs Linux. You must be comfortable navigating the file system, editing files from the terminal, and managing running processes without a graphical interface.

## Explanation
Everything in Linux is a file, including hardware devices (like the serial port `/dev/ttyS0`). Permissions dictate who can read, write, or execute these files.

## Commands & Code
```bash
# Navigation
cd /path/to/directory   # Change directory
ls -la                  # List all files, including hidden

# Process Management
htop                    # View CPU and RAM usage
kill -9 <PID>           # Force stop a stuck program

# Permissions
chmod +x script.py      # Make a Python script executable
sudo usermod -a -G dialout $USER # Grant serial port access
```

## Hardware Setup
No specific hardware required, just an SSH connection to the Pi.

## Practical Exercise
1. Use `nano` to create a `hello.py` file.
2. Make it executable using `chmod +x hello.py`.
3. Run it in the background using `python3 hello.py &`.
4. Find its Process ID (PID) using `htop` and kill it.

## Debugging
- **"Permission Denied" error?** You likely need `sudo` or to fix file ownership with `chown`.
- **Script froze your terminal?** Press `Ctrl+C` to terminate the foreground process.

## Robot Application
If the vision script freezes during a test run, you need to know how to SSH in, find the stuck process using `htop`, and `kill` it to regain control of the robot.
