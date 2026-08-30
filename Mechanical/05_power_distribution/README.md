# 05 Power Distribution

## Objective
Route the high-current power required by 14 servos safely, avoiding brownouts and ground loops.

## The Golden Rule of Wiring
**NEVER power servos from the Raspberry Pi or STM32 5V pins.** Doing so will instantly burn out the microcontrollers.

## Wiring Steps
1. **Main Battery Lead**: Solder an XT60 connector to the main power lines.
2. **Kill Switch**: Splice a 15A latching toggle switch onto the RED (Positive) battery line.
3. **Splitting Power**: After the switch, split the power to the 10A UBEC and the 5A Buck Converter.
4. **Logic Power**: Route the 5V output of the Buck Converter to the Raspberry Pi USB-C or 5V GPIO pins. Route Pi 3.3V out to the STM32.
5. **Servo Power**: Route the 5V output of the UBEC to the green terminal block on the PCA9685.
6. **Star Grounding**: Connect a thick AWG wire joining the GND of the UBEC, the GND of the Buck Converter, the Pi GND, the STM32 GND, and the PCA9685 GND logic pin.

## Safety Fuses
Install a 10A automotive blade fuse inline between the UBEC and the PCA9685. If a leg gets stuck on a wall and all servos stall, the fuse will blow before the servos melt or catch fire.
