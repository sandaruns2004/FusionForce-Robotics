# 07 Maintenance & Repair

## Objective
Diagnose physical hardware failures during testing and prepare the robot for a competition run.

## Common Failures

### 1. Stripped Servo Gears
- **Symptom**: A leg clicks loudly under load and sags. It moves freely when powered off without resistance.
- **Cause**: Impact or stalling. Even metal gears strip under high shock loads.
- **Fix**: Remove the leg, open the servo casing, and replace the gear set (or replace the entire servo). Re-zero the new servo (Module 06).

### 2. Loose Linkages
- **Symptom**: The robot wobbles or the foot placement is inaccurate by 1-2cm.
- **Cause**: M3 locknuts vibrating loose.
- **Fix**: Apply a small drop of Blue Loctite (Threadlocker) to all metal-to-metal screws. Do NOT apply Loctite to plastic, it will melt it.

### 3. Jittering / Twitching Servos
- **Symptom**: Servos jitter violently when standing still.
- **Cause**: Deadband issues or ground loops.
- **Fix**: Verify your star ground wiring (Module 05). If grounding is fine, the servo potentiometer may be worn out, requiring servo replacement.

## Pre-Run Competition Checklist
Before placing the robot on the starting line:
- [ ] Battery voltage checked (>11.5V).
- [ ] All 12 locknuts on the leg joints verified tight.
- [ ] Camera lens wiped with a microfiber cloth.
- [ ] ToF sensor lenses blown clear of dust.
- [ ] Gripper manually tested for smooth opening/closing.
- [ ] Rubber feet inspected for wear.
