# Module 2: IMU & Sensor Fusion

If a legged robot doesn't know which way is "down", it will fall over immediately. The STM32 figures this out using an **IMU (Inertial Measurement Unit)**, specifically the popular MPU6050.

## The MPU6050

The MPU6050 is a 6-axis sensor connected to the STM32 via I2C (SDA and SCL pins).
It contains two sensors inside one tiny chip:
1.  **3-Axis Accelerometer**: Measures the force of gravity and linear acceleration.
2.  **3-Axis Gyroscope**: Measures the rate of rotation (angular velocity).

## The Sensor Flaws

You cannot just read one of these sensors and trust it completely. They both have fatal flaws.

### Accelerometer Flaw: Noise
The accelerometer is great at finding gravity when the robot is standing perfectly still. But as soon as the robot takes a step, the vibrations of the motors send the accelerometer readings completely haywire. It is incredibly **noisy** in the short term.

### Gyroscope Flaw: Drift
The gyroscope is immune to vibrations and tracks fast rotations perfectly. However, over time, the math accumulates tiny rounding errors. If you leave the robot sitting still for 10 minutes, the gyroscope might eventually think the robot has rotated 90 degrees. This is called **Gyroscopic Drift**.

## Sensor Fusion: The Complementary Filter

To get a perfect, stable angle, we must merge the two sensors together. We use the Accelerometer for long-term accuracy, and the Gyroscope for short-term accuracy.

The easiest way to do this on the STM32 is using a **Complementary Filter**.

**The Formula:**
`Filtered_Angle = (0.98 * Gyro_Angle) + (0.02 * Accel_Angle)`

*   *Why this works*: 98% of our angle calculation comes from the Gyroscope, meaning the robot's vibrations (Accelerometer noise) are almost completely ignored. But, we constantly pull 2% of the calculation from the Accelerometer. Over the span of a few seconds, this 2% is enough to cancel out the Gyroscope's drift, pulling the reading back to true gravity.

## Implementing in Arduino IDE

Don't write the raw I2C register reading code yourself. Use a library!
In the Arduino IDE Library Manager, search for and install:
`Adafruit MPU6050`

```cpp
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Start I2C
  
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) { delay(10); }
  }
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Print raw Accelerometer X and Gyroscope Y values
  Serial.print("Accel X: "); Serial.print(a.acceleration.x);
  Serial.print(", Gyro Y: "); Serial.println(g.gyro.y);
  
  delay(10);
}
```

Once you can read the raw values, you will apply the Complementary Filter math in your main `loop()` to calculate the true Pitch and Roll of the robot.

## 📺 Recommended Viewing
*   Search YouTube for: `"Arduino IMU Sensor Fusion Complementary Filter"`
*   Search YouTube for: `"DroneBot Workshop MPU6050"`
