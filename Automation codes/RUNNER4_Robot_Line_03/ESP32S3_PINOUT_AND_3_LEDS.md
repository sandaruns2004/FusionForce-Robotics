# RUNNER4 ESP32-S3 pin map and four-pin RGB LED

This map is for the active `RUNNER4_Robot_Line` ESP32-S3 sketch. The STM32 pin maps elsewhere in the repository describe a different proposed controller. Verify that GPIO16, GPIO17, and GPIO18 are broken out on the exact ESP32-S3 board before wiring them.

## GPIOs currently assigned by the sketch

| ESP32-S3 GPIO | Current connection | Defined in |
| --- | --- | --- |
| 1 | Line sensor S1 (far left), analog | `line_sensor.h` |
| 2 | Line sensor S2, analog | `line_sensor.h` |
| 4 | Line sensor S3, analog | `line_sensor.h` |
| 5 | Line sensor S4, analog | `line_sensor.h` |
| 6 | Line sensor S5 (center), analog | `line_sensor.h` |
| 7 | Line sensor S6, analog | `line_sensor.h` |
| 8 | Line sensor S7, analog | `line_sensor.h` |
| 9 | Line sensor S8, analog | `line_sensor.h` |
| 10 | Line sensor S9 (far right), analog | `line_sensor.h` |
| 11 | I2C SDA, shared by PCA9685 and TCS34725 | `RUNNER4_Robot_Line.ino` |
| 12 | I2C SCL, shared by PCA9685 and TCS34725 | `RUNNER4_Robot_Line.ino` |
| 15 | TCS34725 illuminator control, high means on | `color_sensor.h` |

The 12 leg servos connect to **PCA9685 channels**, not to 12 more ESP32 GPIOs. The current sketch does not assign GPIOs to an IMU or wall-distance sensors.

## Proposed pins for the common-cathode RGB LED

| RGB color leg | ESP32-S3 GPIO | Connection |
| --- | --- | --- |
| Red | 16 | GPIO16 -> 330 ohm resistor -> RGB red leg |
| Green | 17 | GPIO17 -> 330 ohm resistor -> RGB green leg |
| Blue | 18 | GPIO18 -> 330 ohm resistor -> RGB blue leg |
| Common cathode | — | RGB common leg -> GND |

The package contains three LED elements sharing one cathode. **Each color leg needs its own resistor.** These pins are digital outputs even though they also have ADC2 functions. The proposed LED pins do not conflict with any pin assigned in the current sketch. GPIO15 already controls the color sensor's illuminator; it is not part of this RGB LED.

Do not connect a color leg directly to a GPIO or connect 5 V to a GPIO. Use the GPIO's 3.3 V logic output and ground. The **physical order of R, common, G, and B legs depends on the RGB LED model**; identify them from its datasheet or test them with a resistor before soldering. Do not assume the longest leg or a particular left-to-right order without checking.

## Code to add after choosing what the LEDs mean

Near the other pin definitions in `RUNNER4_Robot_Line.ino`:

```cpp
constexpr uint8_t RGB_RED_PIN   = 16;
constexpr uint8_t RGB_GREEN_PIN = 17;
constexpr uint8_t RGB_BLUE_PIN  = 18;
```

In the existing `setup()` function:

```cpp
pinMode(RGB_RED_PIN, OUTPUT);
pinMode(RGB_GREEN_PIN, OUTPUT);
pinMode(RGB_BLUE_PIN, OUTPUT);
digitalWrite(RGB_RED_PIN, LOW);
digitalWrite(RGB_GREEN_PIN, LOW);
digitalWrite(RGB_BLUE_PIN, LOW);
```

For a common-cathode RGB LED, `HIGH` turns that color on and `LOW` turns it off. This helper can be added to the sketch:

```cpp
void setRgb(bool red, bool green, bool blue) {
  digitalWrite(RGB_RED_PIN,   red   ? HIGH : LOW);
  digitalWrite(RGB_GREEN_PIN, green ? HIGH : LOW);
  digitalWrite(RGB_BLUE_PIN,  blue  ? HIGH : LOW);
}

setRgb(true,  false, false); // red
setRgb(false, true,  false); // green
setRgb(false, false, true);  // blue
setRgb(true,  true,  false); // yellow (red + green)
setRgb(false, false, false); // off
```

The `setRgb(...)` calls above are examples; put them inside an existing function rather than at file scope. The current firmware does not yet initialize or control GPIO16–18, because the intended status meanings of the colors have not been specified.

## Power connections to verify

| Connection | Destination |
| --- | --- |
| Servo regulator output, within the MG90 supply rating | PCA9685 `V+` servo-power terminal |
| ESP32-S3 board 3V3 | PCA9685 `VCC` logic pin and 3.3 V sensors, subject to regulator capacity |
| Common ground | ESP32, PCA9685, sensors, and both regulators |
| Logic regulator output | ESP32 board's specified `5V`/`VIN` input if the board supports it |

The PCA9685 `VCC` and `V+` pins serve different purposes. Do not feed 5 V into the ESP32's `3V3` pin. Follow the exact development board's power-input markings.

## Pins deliberately avoided for these LEDs

- GPIO0, GPIO3, GPIO45, GPIO46: boot strapping pins.
- GPIO19, GPIO20: native USB connections.
- GPIO26–32: commonly used by flash/PSRAM.
- GPIO33–37: may be used by octal flash/PSRAM, depending on module.
- GPIO43, GPIO44: commonly used for UART serial.
- GPIO48: onboard RGB LED on some ESP32-S3 development boards.

