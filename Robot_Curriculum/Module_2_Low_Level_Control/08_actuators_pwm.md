# Module 2: Actuators & PWM

To move the robot's legs, the STM32 must send electrical signals to the motors. For standard RC Servos, this signal is called **PWM (Pulse Width Modulation)**.

## What is PWM?
A microcontroller's digital pins can only output HIGH (3.3V) or LOW (0V). They cannot output a "half-voltage". 
PWM fakes an analog signal by turning the pin HIGH and LOW extremely fast.

### PWM for Standard RC Servos
Servos are very picky. They don't just care about the *average* voltage; they care about the exact *timing* of the HIGH pulse.

*   **Frequency**: Standard servos expect a pulse every 20 milliseconds (50 Hz).
*   **Pulse Width**: The length of time the pin stays HIGH during that 20ms window determines the angle.
    *   `1.0 ms` HIGH = 0 degrees (far left)
    *   `1.5 ms` HIGH = 90 degrees (center)
    *   `2.0 ms` HIGH = 180 degrees (far right)

## Controlling Servos with STM32 (The Arduino Way)

In the Arduino IDE, you don't have to write the complex timer code to generate these exact millisecond pulses. You use the `<Servo.h>` library.

```cpp
#include <Servo.h>

Servo coxaMotor;
Servo femurMotor;

void setup() {
  // Attach the servos to specific PWM-capable GPIO pins on the STM32
  coxaMotor.attach(PA0);
  femurMotor.attach(PA1);
}

void loop() {
  // Move to center
  coxaMotor.write(90); 
  femurMotor.write(90);
  delay(1000);
  
  // Move to 45 degrees
  coxaMotor.write(45);
  delay(1000);
}
```

## The "No Delay" Rule (CRITICAL)

The code above is terrible for a legged robot. 
Why? Because `delay(1000)` freezes the STM32 for a full second. During that second:
*   It cannot read the IMU to see if the robot is falling.
*   It cannot read the Serial port to hear commands from the Raspberry Pi.

**Real-time robots must never wait.** We must use `millis()` for non-blocking timing.

### The Correct Way (Non-Blocking)

```cpp
#include <Servo.h>

Servo coxaMotor;
unsigned long previousMillis = 0;
const long interval = 20; // Update motor every 20ms
int currentAngle = 0;

void setup() {
  coxaMotor.attach(PA0);
}

void loop() {
  unsigned long currentMillis = millis();

  // Has 20ms passed? If yes, update the motor.
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    // Smoothly sweep the motor without freezing the loop
    currentAngle++;
    if(currentAngle > 180) currentAngle = 0;
    
    coxaMotor.write(currentAngle);
  }
  
  // You can put IMU reading code here, and it will run instantly!
}
```

## 📺 Recommended Viewing
*   Search YouTube for: `"Blink without delay Arduino explained"`
*   Search YouTube for: `"How PWM works and how to use it"`
