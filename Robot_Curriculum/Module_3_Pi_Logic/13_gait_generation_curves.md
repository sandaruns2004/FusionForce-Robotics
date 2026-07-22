# Module 3: Gait Generation (Walking Curves)

We know how to convert an (X,Y,Z) foot coordinate into motor angles using Inverse Kinematics (IK). But how do we generate the actual sequence of (X,Y,Z) coordinates that makes a smooth walking motion?

This is called **Trajectory Generation** or **Gait Generation**.

## The Phases of a Step

Every step a leg takes is divided into two distinct phases:
1.  **Swing Phase**: The foot is lifted off the ground, moved forward through the air, and placed down.
2.  **Stance Phase**: The foot is firmly on the ground. The body pushes backward against this foot to propel the robot forward.

## Bezier Curves (The Secret to Smooth Motion)

If you simply tell the foot to move from Point A (back) to Point B (front), it will drag along the floor.
If you tell it to move from A -> UP -> B, the motion will be harsh, robotic, and jerky, causing the whole robot to shake.

To achieve organic, animal-like movement, we use **Bezier Curves**.
A Bezier curve uses mathematical control points to draw a perfectly smooth arc through the air.

*   `P0`: Start point (Foot on the ground, back).
*   `P1`: Control point (Pulling the curve upwards).
*   `P2`: Control point (Pulling the curve forward).
*   `P3`: End point (Foot strikes the ground, front).

By feeding a time variable `t` (from 0.0 to 1.0) into a cubic Bezier equation in your Python code, you get a perfectly smooth array of (X, Y, Z) targets to feed into your IK algorithm.

## Types of Gaits

A "Gait" defines the timing of when each of the 4 legs enters the Swing phase vs the Stance phase.

### 1. The Creep Gait (Static Balancing)
*   **How it works**: Only one leg lifts off the ground at a time. The other 3 legs form a triangle of support.
*   **Math**: The Pi shifts the Center of Mass (moves all 3 grounded legs slightly) so the body leans directly over the triangle before lifting the 4th leg.
*   **Pros**: Impossible to fall over. Great for learning.
*   **Cons**: Extremely slow. Looks robotic.

### 2. The Trot Gait (Dynamic Balancing)
*   **How it works**: Diagonal legs move together. (Front-Left and Back-Right lift up simultaneously).
*   **Math**: The robot is only supported by 2 legs (a straight line) for a fraction of a second. The Pi must calculate the Bezier curves fast enough that the swinging legs strike the ground *before* gravity pulls the robot over.
*   **Pros**: Fast, fluid, and organic looking.
*   **Cons**: Requires tight integration with the STM32's IMU to make micro-adjustments.

## 📺 Recommended Viewing
*   Search YouTube for: `"Bezier Curves explained mathematically"` (Freya Holmér has an incredible video on this).
*   Search YouTube for: `"Quadruped robot trot gait generation Python"`
