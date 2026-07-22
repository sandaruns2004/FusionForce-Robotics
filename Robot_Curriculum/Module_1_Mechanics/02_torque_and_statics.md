# Module 1: Torque & Statics

One of the most common mistakes in robotics is buying motors that are too weak. Before you buy any servos or BLDC motors, you must calculate the required **Torque**.

## What is Torque?

Torque is a twisting force that causes rotation. 
**Formula:** `Torque (τ) = Force (F) × Distance (d)`

*   **Force**: The weight the motor has to lift (measured in Newtons, or kg/lbs).
*   **Distance**: The length of the lever arm from the motor shaft to where the force is applied (measured in cm or meters).

Motor torque is usually rated in **kg-cm** (kilogram-centimeters) or **N-m** (Newton-meters).
*Example*: A servo rated at `20 kg-cm` can lift a 20 kg weight hanging on an arm that is 1 cm long. Or, it can lift a 1 kg weight on an arm that is 20 cm long.

## Calculating Leg Torque

Imagine your robot weighs **2 kg** total.
When the robot is standing on all 4 legs, each leg supports 0.5 kg.
However, when the robot is walking (Trot gait), it balances on only 2 legs. Therefore, each leg must be able to support **1 kg**.

Let's look at the **Femur Motor** (Hip Pitch). This motor experiences the highest torque because it holds the robot up against gravity.

1.  Assume the Femur (upper leg bone) is **10 cm** long.
2.  Assume the leg needs to lift **1 kg** of weight.
3.  Required Torque = `1 kg × 10 cm = 10 kg-cm`.

### The Safety Margin (Crucial!)

If you buy a motor rated for exactly 10 kg-cm (its "stall torque"), it will burn out immediately. Stall torque is the force at which the motor completely stops and begins melting.

**Rule of Thumb**: The operating torque should be roughly **1/3** of the stall torque.
Therefore, if your math says you need 10 kg-cm, you should buy a motor rated for **30 kg-cm** stall torque.

## Motor Recommendations

1.  **MG996R (Budget)**: Rated at ~10 kg-cm. Good for very small, lightweight, plastic robots (under 1 kg total weight). Leg lengths must be very short.
2.  **DS3218 / 20kg-30kg Digital Servos (Intermediate)**: Rated at 20-30 kg-cm. Great for medium-sized 3D printed dogs.
3.  **Serial Bus Servos (Advanced)**: e.g., LewanSoul LX-16A (17 kg-cm) or STS3032 (30 kg-cm). Highly recommended because they allow you to chain wires together and read the exact angle/temperature back from the servo.
4.  **Hoverboard / BLDC Motors (Expert)**: Massive torque, but require expensive motor drivers (like ODrive) and advanced PID tuning.

## 📺 Recommended Viewing
*   Search YouTube for: `"How to calculate servo torque for a robot arm"`
*   Search YouTube for: `"MG996R vs DS3218 torque test"`
