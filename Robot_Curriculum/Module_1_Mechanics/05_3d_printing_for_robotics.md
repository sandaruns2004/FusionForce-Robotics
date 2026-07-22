# Module 1: 3D Printing for Robotics

When building a robot, the structural integrity of your physical parts is just as important as the code. A robot that breaks its own legs when it jumps is useless.

## Choosing the Right Material

Not all 3D printing filaments are created equal.

1.  **PLA (Polylactic Acid)**:
    *   *Pros*: Easiest to print, very stiff, cheap.
    *   *Cons*: Brittle. If the robot falls, PLA tends to shatter or snap. It also melts in hot cars.
    *   *Verdict*: Good for prototyping, but not ideal for the final load-bearing leg parts.
2.  **PLA+ / Tough PLA**:
    *   *Pros*: Modified to be much less brittle than standard PLA. High impact resistance.
    *   *Verdict*: **Highly Recommended** for beginner robot builds.
3.  **PETG**:
    *   *Pros*: Flexible and extremely durable. It will bend before it breaks. High temperature resistance.
    *   *Cons*: Harder to print (stringing issues).
    *   *Verdict*: Excellent for final robotic parts, especially protective bumpers or body shells.
4.  **TPU (Flexible)**:
    *   *Pros*: Rubber-like.
    *   *Verdict*: Use this strictly for printing the "feet" of the robot to give it grip on slippery floors.

## Print Settings for Strength

Do not just use the default "Standard" settings in your slicer (Cura, PrusaSlicer). Robot parts experience high torque and shear forces.

### 1. Perimeters (Wall Thickness)
This is the **most important setting** for strength.
*   Increasing infill percentage doesn't make a part much stronger. Increasing the wall thickness does.
*   *Recommendation*: Change your Wall Line Count / Perimeters from 2 to **4 or 5**.

### 2. Infill Pattern
*   Do not use Grid or Lines.
*   *Recommendation*: Use **Gyroid** or **Cubic** infill. These provide 3D strength in all directions, which is crucial for parts like the Coxa joint that twist on multiple axes.
*   *Density*: 20% to 30% is usually plenty if you have thick walls.

### 3. Print Orientation (The Secret Weapon)
3D printed parts are weakest along the Z-axis (the layer lines). If you try to snap a 3D printed stick, it will always snap cleanly along a layer line.

*   **Rule**: Never align the stress forces of the robot parallel to the layer lines.
*   When orienting the Femur (upper leg) on the build plate, lay it flat on its side. Do not print it standing straight up. If it's printed flat, the continuous lines of plastic run the entire length of the leg, giving it immense bending strength.

## Brass Inserts

Do not screw metal screws directly into plastic holes. The plastic will strip immediately after a few repairs.
*   *Recommendation*: Use **Heat-Set Threaded Brass Inserts**. You design a slightly undersized hole in your CAD, and use a soldering iron to melt a brass nut into the plastic. This gives you incredibly strong, reusable metal threads.

## 📺 Recommended Viewing
*   Search YouTube for: `"CNC Kitchen wall thickness vs infill"` - The ultimate scientific proof on how to print strong parts.
*   Search YouTube for: `"How to use heat set inserts 3d printing"`
