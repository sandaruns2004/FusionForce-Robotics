# Module 2: STM32 & Arduino IDE Setup

The STM32 is the "spinal cord" of our robot. It runs the real-time, microsecond-accurate loops needed to keep the robot from falling over.

While professionals use ST's official tool (STM32CubeIDE), it is notoriously difficult for beginners. Instead, we will use the **Arduino IDE** with the **STM32duino core**. This lets you write standard, easy-to-read C++ Arduino code but run it on the ultra-fast STM32 processor.

## Step 1: Install the Arduino IDE
Download and install Arduino IDE 2.x from the official website.

## Step 2: Install the STM32 Core
1. Open Arduino IDE and go to `File > Preferences`.
2. In the "Additional Boards Manager URLs" field, paste this URL:
   `https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json`
3. Go to `Tools > Board > Boards Manager...`.
4. Search for "STM32" and install the `STM32 MCU based boards` package by STMicroelectronics.

## Step 3: Select Your Board
1. Go to `Tools > Board > STM32 MCU based boards`.
2. Select your specific board line (e.g., "Generic STM32F4 series" if using a Black Pill).
3. Under `Tools > Board part number`, select your exact chip (e.g., "BlackPill F401CC").

## Step 4: Flashing the Code (Uploading)
STM32s don't always upload as easily as a standard Arduino Uno. Depending on your board, you have a few options in `Tools > Upload method`:

*   **STM32CubeProgrammer (DFU)**: Plugs in via USB. You usually have to hold the `BOOT0` button, press the `NRST` (reset) button, let go of `NRST`, then let go of `BOOT0` to put it in upload mode.
*   **STLink**: Uses a cheap external USB dongle (ST-Link V2) connected to the SWDIO, SWCLK, GND, and 3.3V pins. *This is the most reliable method and highly recommended.*

## The "Hello World" (Blink Test)

Copy this code, upload it, and make sure the onboard LED blinks. If it does, your environment is perfectly set up!

```cpp
// PC13 is the built-in LED on the STM32F401 Black Pill
// Note: It might be a different pin for Nucleo boards (often PA5)
const int LED_PIN = PC13; 

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, LOW);  // Turn LED ON (PC13 is usually inverted)
  delay(500);
  digitalWrite(LED_PIN, HIGH); // Turn LED OFF
  delay(500);
}
```

## Next Steps
Now that the STM32 is working, we need to learn how to make it talk to the motors without using the `delay()` function, which freezes the whole processor!

## 📺 Recommended Viewing
*   Search YouTube for: `"How to program STM32 in Arduino IDE Blackpill"`
