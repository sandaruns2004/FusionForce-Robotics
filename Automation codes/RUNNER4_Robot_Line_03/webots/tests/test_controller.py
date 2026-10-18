"""Logic/integration tests, not a substitute for a Webots rendered-sensor run."""
import math
import re
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'controllers' / 'runner4_line'))
from core import BODY_Z, LineFollower, SENSOR_X, SENSOR_Y, START, integrate, gait_targets


def rotate(vector, axis_angle):
    axis = axis_angle[:3]
    norm = math.sqrt(sum(v*v for v in axis))
    a = [v/norm for v in axis]
    c, s = math.cos(axis_angle[3]), math.sin(axis_angle[3])
    dot = sum(x*y for x, y in zip(a, vector))
    cross = (a[1]*vector[2]-a[2]*vector[1],
             a[2]*vector[0]-a[0]*vector[2], a[0]*vector[1]-a[1]*vector[0])
    return [v*c+k*s+u*dot*(1-c) for v, k, u in zip(vector, cross, a)]


def numbers(text):
    return [float(v) for v in text.split()]


def line_distance(x, y):
    # Independent analytic stadium oracle, used ONLY by tests.
    cx = max(-.4, min(.4, x))
    return abs(math.hypot(x-cx, y)-.45)


def reflectance(pose):
    x, y, heading = pose
    c, s = math.cos(heading), math.sin(heading)
    return [1.0 if line_distance(x+c*SENSOR_X-s*sy, y+s*SENSOR_X+c*sy) <= .015 else 0.0
            for sy in SENSOR_Y]


class ControllerTests(unittest.TestCase):
    def test_visual_meshes_face_outward_and_leave_optics_clear(self):
        sys.path.insert(0, str(ROOT/'tools'))
        from robot_visuals import neutral_scene
        # Solid signed volume catches inverted blade faces that disappear under culling.
        for vertices, faces, _ in neutral_scene(SENSOR_X, SENSOR_Y, BODY_Z).meshes:
            volume6 = 0
            for face in faces:
                a = vertices[face[0]]
                for j in range(1,len(face)-1):
                    b,c = vertices[face[j]],vertices[face[j+1]]
                    cross = (b[1]*c[2]-b[2]*c[1],b[2]*c[0]-b[0]*c[2],b[0]*c[1]-b[1]*c[0])
                    volume6 += sum(x*y for x,y in zip(a,cross))
            self.assertGreater(volume6, 0)
            # Conservative AABB check: no decorative part may lie below a sensor.
            for y in SENSOR_Y:
                overlaps = min(v[0] for v in vertices) <= SENSOR_X <= max(v[0] for v in vertices)
                overlaps &= min(v[1] for v in vertices) <= y <= max(v[1] for v in vertices)
                if overlaps:
                    self.assertGreater(min(v[2] for v in vertices)+BODY_Z, .012)

    def test_saved_camera_rays_and_track_visibility(self):
        world = (ROOT/'worlds/runner4_white_line.wbt').read_text()
        proto = (ROOT/'protos/Runner4.proto').read_text()
        view = re.search(r'Viewpoint\s*\{([^}]+)', world).group(1)
        orientation = numbers(re.search(r'orientation\s+([\d.eE+\-\s]+)', view).group(1))
        position = numbers(re.search(r'position\s+([\d.eE+\-\s]+)', view).group(1))
        direction = rotate((1, 0, 0), orientation)
        self.assertLess(direction[2], -.9999, 'Main view must face the floor')
        distance = -position[2]/direction[2]
        hit = [p+distance*d for p, d in zip(position, direction)]
        self.assertLess(math.hypot(hit[0], hit[1]), 1e-6)
        # Both entire arena extents fit at the screenshot's ~1.49 aspect ratio.
        right = rotate((0, -1, 0), orientation)
        up = rotate((0, 0, 1), orientation)
        half_width = position[2]*math.tan(.9/2)
        for x in [-1.213, 1.213]:
            for y in [-.913, .913]:
                self.assertLess(abs(x*right[0]+y*right[1]), half_width)
                self.assertLess(abs(x*up[0]+y*up[1]), half_width/1.49)

        raw_vertices = re.search(r'point\s*\[([^]]+)', world).group(1)
        vertices = [numbers(v) for v in raw_vertices.strip().split(',')]
        indices = [int(v) for v in re.search(r'coordIndex\s*\[([^]]+)', world).group(1).split()]
        faces = [indices[i:i+4] for i in range(0, len(indices), 5)]

        def cross2(a, b, p):
            return (b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])

        for face in faces:
            a, b, c = (vertices[j] for j in face[:3])
            self.assertGreater(cross2(a, b, c), 0, 'Track faces must face upward')
        readings = []
        for block in re.findall(r'Camera\s*\{([^}]+)', proto):
            local = numbers(re.search(r'translation\s+([\d.eE+\-\s]+)', block).group(1))
            rotation = re.search(r'rotation\s+([\d.eE+\-\s]+)', block)
            direction = rotate((1, 0, 0), numbers(rotation.group(1)) if rotation else [0, 0, 1, 0])
            self.assertLess(direction[2], -.9999, 'Optical axis must point down')
            origin = [START[0]+local[0], START[1]+local[1], BODY_Z+local[2]]
            distance = (.001-origin[2])/direction[2]
            self.assertTrue(.001 < distance < .04, 'Track must lie between near/far planes')
            hit = [p+distance*d for p, d in zip(origin, direction)]
            visible = False
            for face in faces:
                a, b, c, d = (vertices[j] for j in face)
                for triangle in [(a, b, c), (a, c, d)]:
                    if all(cross2(triangle[j], triangle[(j+1)%3], hit) >= -1e-10 for j in range(3)):
                        visible = True
            readings.append(visible)
        self.assertEqual(readings, [False, False, False, True, True, True, False, False, False])

    def test_left_right_and_center(self):
        for index, sign in [(0, 1), (4, 0), (8, -1)]:
            samples = [0.0]*9
            samples[index] = 1
            f = LineFollower()
            speed, turn = f.update(samples, .02)
            self.assertGreater(speed, 0)
            self.assertAlmostEqual(f.error_mm, (4-index)*10)
            self.assertEqual(0 if turn == 0 else int(math.copysign(1, turn)), sign)

    def test_loss_is_latched_until_reset(self):
        f = LineFollower()
        for _ in range(151):
            speed, turn = f.update([0]*9, .02)
            self.assertEqual(speed, 0)
        self.assertTrue(f.stopped)
        self.assertEqual(f.update([1]*9, .02), (0, 0))
        self.assertGreater(LineFollower().update([0, 0, 0, 0, 1, 0, 0, 0, 0], .02)[0], 0)

    def test_reacquisition_before_timeout(self):
        f = LineFollower()
        for _ in range(20):
            f.update([0]*9, .02)
        self.assertGreater(f.update([0, 0, 0, 1, 1, 1, 0, 0, 0], .02)[0], 0)
        self.assertFalse(f.stopped)
        self.assertEqual(f.lost_time, 0)

    def test_two_laps_with_offsets(self):
        for offset in [-.02, 0, .02]:
            f = LineFollower()
            pose = START[0], START[1]+offset, START[2]
            travel = 0
            worst = 0
            for _ in range(11000):  # 220 s, > 2 laps at 45 mm/s
                speed, turn = f.update(reflectance(pose), .02)
                pose = integrate(pose, speed, turn, .02)
                travel += speed*.02
                worst = max(worst, line_distance(pose[0], pose[1]))
                self.assertFalse(f.stopped, (offset, pose))
            self.assertGreater(travel, 2*(1.6+2*math.pi*.45))
            self.assertLess(worst, .035)
            self.assertGreater(pose[2], 4*math.pi)

    def test_gait_limits(self):
        for i in range(300):
            for angles in gait_targets(i*.02, True, .65).values():
                self.assertTrue(all(abs(v) < 1.2 for v in angles))
        self.assertTrue(all(a == (0, 0, 0) for a in gait_targets(1, False).values()))

    def test_asset_device_contract(self):
        proto = (ROOT / 'protos/Runner4.proto').read_text()
        self.assertEqual(proto.count('RotationalMotor {'), 12)
        self.assertEqual(proto.count('Camera {'), 9)
        for leg in ('FL', 'FR', 'BL', 'BR'):
            for joint in ('hip', 'femur', 'tibia'):
                self.assertIn(f'name "{leg}_{joint}"', proto)
        for i in range(1, 10):
            self.assertIn(f'name "line_s{i}"', proto)
        for file in [ROOT/'protos/Runner4.proto', ROOT/'worlds/runner4_white_line.wbt']:
            text = file.read_text()
            self.assertEqual(text.count('{'), text.count('}'))
            self.assertEqual(text.count('['), text.count(']'))
        world = (ROOT / 'worlds/runner4_white_line.wbt').read_text()
        self.assertNotIn('IndexedFaceSet { solid', world)
        self.assertIn('geometry IndexedFaceSet { coord Coordinate', world)



if __name__ == '__main__':
    unittest.main(verbosity=2)
