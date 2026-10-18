# RUNNER4 reference reconstruction

Rebuilt on 19 October 2026 from the user's four photographs of the assembled black robot and the two assembly screenshots extracted from `quadruped-spider-2.snapshot.2 (1).zip`.

The ZIP inventory contains `Base up.SLDPRT`, `Base Down.SLDPRT`, `coxa left.SLDPRT`, `coxa right.SLDPRT`, `Fumer.SLDPRT`, `tibia Left.SLDPRT`, `tibia right.SLDPRT`, `Hold.SLDPRT`, `Servo SG90.SLDPRT`, `SPIDER.SLDASM`, `Assem1.SLDASM`, and two PNG screenshots. No STL, OBJ or STEP exchange meshes were present, and no working SolidWorks converter was found. The native CAD files have not been imported or reverse-engineered; the reconstruction uses the archive's assembly views plus the hardware photographs. Exact CAD fidelity is not claimed.

| Reference feature | Representation |
|---|---|
| Open central chassis and upper/lower plates | Two black rectangular frames with corner posts, approximately 100 × 60 mm |
| Four legs in diagonal layout | Hip assemblies yawed outward/fore-aft by 35 degrees |
| Coxa and femur brackets | Paired black rails and transverse bridges with visible pivot heads |
| Swept tibia shape | Thin curved blade narrowing toward each foot |
| Micro servos and pale horns | Dark housings, mounting flanges, silver pivots and pale horn bars |
| Top electronics | Brown perfboard, ESP32-style development board, RF shield, blue PWM board, headers and three LEDs |
| Power assembly | Silver battery, black strap, yellow connector and red/black leads |
| Wiring | Looped three-conductor servo cables and six-colour ribbon to the front array |
| Front line array | Brown strip with nine pairs of IR packages above nine unobstructed camera apertures |

Firmware link lengths remain 30/60/80 mm. Plate thickness, bracket thickness, blade profile, electronics, cable routing and diagonal angles are visual estimates. The electronics and wiring are decorative geometry. The sensor mount is extended downward to maintain the demo's 12 mm sensing height; its bracket is an approximation rather than a measured mounting design.

All twelve named motors and the nine downward cameras are retained. The legs remain kinematic, and the Supervisor still moves the body according to sensor-driven planar control. This update does not introduce physical walking, additional working electronics, or a calibrated mechanical replica.

`robot_preview.png` is a static render of the same generated visual meshes at the neutral joint pose. It is not a screenshot of a Webots run. The preview was inspected, and tests check outward mesh winding and unobstructed sensor positions in the neutral pose. A full animated Webots rendering remains unverified here.

To see the changes, close and reopen `worlds/runner4_white_line.wbt`, without saving the old loaded scene over the regenerated files. Zoom toward the robot to inspect the details; the default camera shows the whole arena.
