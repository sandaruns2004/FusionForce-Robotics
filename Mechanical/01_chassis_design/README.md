# 01 Chassis Design

## Objective
Design and 3D print a robust, perfectly symmetrical chassis that maintains a low Center of Gravity (CoG).

## Materials & Printing
- **Material**: Do not use standard PLA for structural leg mounts; it deforms under continuous servo heat and shatters on impact. Use **PETG** or **ABS**.
- **Infill**: 40-50% Gyroid infill for the main body. 100% infill for servo mounting brackets.
- **Wall Thickness**: Minimum 4 perimeters for structural rigidity.

## Center of Gravity (CoG)
A quadruped must keep its CoG strictly within the triangle formed by its 3 grounded legs during a crawl gait. 
- The battery (the heaviest component) must be mounted perfectly in the geometric center of the bottom deck.
- Do not mount the Raspberry Pi entirely on the back edge, as it will shift the CoG backward, causing the front legs to lose traction.

## Tasks for the Team
1. Export STLs from CAD (Fusion360/SolidWorks).
2. Slice using Cura or PrusaSlicer with the recommended PETG profiles.
3. Print the base plate, top plate, and 12 servo brackets.
4. Tap M3 threads into the plastic where necessary (or use brass heat-set inserts for maximum durability).
