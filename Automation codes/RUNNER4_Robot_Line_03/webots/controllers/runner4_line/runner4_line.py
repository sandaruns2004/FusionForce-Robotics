"""Run through Webots, not directly with python. No third-party packages."""
import math
from controller import Supervisor
from core import BODY_Z, CHANNELS, START, LineFollower, gait_targets, integrate


def main():
    robot = Supervisor()
    step = int(robot.getBasicTimeStep())
    dt = step / 1000.0
    body = robot.getSelf()
    translation = body.getField('translation')
    rotation = body.getField('rotation')
    cameras = [robot.getDevice('line_s%d' % i) for i in range(1, 10)]
    for camera in cameras:
        camera.enable(step)
    motors = {leg: [robot.getDevice(leg+'_'+joint) for joint in ('hip', 'femur', 'tibia')]
              for leg in CHANNELS}
    for group in motors.values():
        for motor in group:
            motor.setVelocity(3.0)
            motor.setPosition(0)
    keyboard = robot.getKeyboard()
    keyboard.enable(step)
    follower = LineFollower()
    pose = START
    paused = False
    blackout = False
    last_keys = set()
    gait_time = 0.0
    tick = 0
    print('RUNNER4: KINEMATIC demonstration; no foot-contact physics.')
    print('Scene revision: optics-v2 (+X cameras rotated down)')
    print('Click the 3D view. P pause/resume | R reset | L toggle sensor blackout test')
    while robot.step(step) != -1:
        keys = set()
        key = keyboard.getKey()
        while key != -1:
            keys.add(key)
            key = keyboard.getKey()
        pressed = keys - last_keys
        last_keys = keys
        if ord('P') in pressed:
            paused = not paused
        if ord('L') in pressed:
            blackout = not blackout
        if ord('R') in pressed:
            follower = LineFollower()
            pose, paused, blackout, gait_time = START, False, False, 0.0
            translation.setSFVec3f([pose[0], pose[1], BODY_Z])
            rotation.setSFRotation([0, 0, 1, pose[2]])
            for group in motors.values():
                for motor in group:
                    motor.setPosition(0)
            # Camera images belong to the old pose until the next simulation step.
            continue
        samples = []
        ready = True
        for cam in cameras:
            data = cam.getImage()
            if not data:
                ready = False
                samples.append(0.0)
                continue
            values = [cam.imageGetGray(data, cam.getWidth(), x, y)/255.0
                      for x in range(cam.getWidth()) for y in range(cam.getHeight())]
            samples.append(sum(values)/len(values))
        if blackout:
            samples = [0.0]*9
        speed, yaw = (0.0, 0.0) if paused or not ready else follower.update(samples, dt)
        # Heading and position are outputs of the sensor controller. No track map
        # or waypoint coordinates are used to choose steering.
        pose = integrate(pose, speed, yaw, dt)
        if abs(pose[0]) > 1.04 or abs(pose[1]) > 0.74:
            follower.stopped = True
            follower.state = 'ARENA LIMIT - press R'
            speed = yaw = 0.0
            pose = (max(-1.04, min(1.04, pose[0])), max(-0.74, min(0.74, pose[1])), pose[2])
        translation.setSFVec3f([pose[0], pose[1], BODY_Z])
        rotation.setSFRotation([0, 0, 1, pose[2]])
        moving = abs(speed)+abs(yaw) > 0
        if moving:
            gait_time += dt
        for leg, angles in gait_targets(gait_time, moving, yaw).items():
            for motor, angle in zip(motors[leg], angles):
                motor.setPosition(angle)
        status = 'PAUSED' if paused else follower.state
        bars = ''.join('W' if v >= follower.cfg.threshold else '.' for v in samples)
        robot.setLabel(0, 'RUNNER4 | KINEMATIC DEMO\n'+status+'\nS1 [ '+bars+' ] S9\n'
                       +f'error {follower.error_mm:+.1f} mm | speed {speed:.3f} m/s\n'
                       +'P pause | R reset | L sensor blackout',
                       0.02, 0.02, 0.032, 0xFFFFFF, 0.0, 'Arial')
        if tick % 100 == 0:
            print(f'{status}: sensors={bars} error={follower.error_mm:+.1f} mm yaw={yaw:+.3f} rad/s')
        tick += 1


if __name__ == '__main__':
    main()
