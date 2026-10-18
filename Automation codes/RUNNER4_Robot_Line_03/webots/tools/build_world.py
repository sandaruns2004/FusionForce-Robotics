"""Regenerate local Webots assets, using only the Python standard library."""
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'controllers' / 'runner4_line'))
from core import BODY_Z, SENSOR_X, SENSOR_Y, START
from robot_visuals import body, leg_parts


def vec(values):
    return ' '.join(f'{v:.8f}' for v in values)


def box(size, color, position=(0, 0, 0)):
    return f'''Pose {{ translation {vec(position)} children [ Shape {{
      appearance PBRAppearance {{ baseColor {color} roughness 0.8 metalness 0 }}
      geometry Box {{ size {vec(size)} }} }} ] }}'''


def rod(delta, color):
    length = math.sqrt(sum(v*v for v in delta))
    x, y, z = delta
    axis_length = math.hypot(x, y)
    axis = (-y/axis_length, x/axis_length, 0) if axis_length else (1, 0, 0)
    return f'''Pose {{ translation {vec(v/2 for v in delta)}
      rotation {vec(axis)} {math.acos(z/length):.8f}
      children [ {box((0.009, 0.009, length), color)} ] }}'''


def joint(name, anchor, axis, children, yaw=0):
    return f'''HingeJoint {{
      jointParameters HingeJointParameters {{ anchor {vec(anchor)} axis {vec(axis)} }}
      device [ RotationalMotor {{ name "{name}" minPosition -1.2 maxPosition 1.2
                 maxVelocity 5 maxTorque 0.22 }}
               PositionSensor {{ name "{name}_position" }} ]
      endPoint Solid {{ translation {vec(anchor)} name "{name}_link"
        rotation 0 0 1 {yaw:.8f}
        children [ {children} ] }}
    }}'''


def leg(name, x, side):
    hip_visual, thigh_visual, knee_visual, coxa, femur, _ = leg_parts(side)
    knee = joint(name+'_tibia', femur, (1, 0, 0), knee_visual.vrml())
    thigh = joint(name+'_femur', coxa, (1, 0, 0), thigh_visual.vrml()+knee)
    yaw = -side*(1 if x > 0 else -1)*math.radians(35)
    return joint(name+'_hip', (x, side*0.030, 0), (0, 0, 1),
                 hip_visual.vrml()+thigh, yaw)


def robot_proto():
    legs = '\n'.join(leg(n, x, s) for n, x, s in
                     [('FL', .038, 1), ('FR', .038, -1), ('BL', -.038, 1), ('BR', -.038, -1)])
    cameras = '\n'.join(f'''Camera {{ translation {SENSOR_X} {y} {0.012-BODY_Z}
      rotation 0 1 0 1.57079632679
      name "line_s{i+1}" width 3 height 3 fieldOfView 0.38 near 0.001 far 0.04
      noise 0 antiAliasing TRUE }}''' for i, y in enumerate(SENSOR_Y))
    # R2025a cameras look along local +X. Rotate +90 degrees about Y to look -Z.
    return f'''#VRML_SIM R2025a utf8
# RUNNER4 demonstration: 100x60 mm chassis; 30/60/80 mm links.
# Photo-reference rebuild: diagonal black brackets, blade tibias, stacked boards.
# Reconstructed from photos and CAD screenshots, not imported SolidWorks BREP.
# All joints are kinematic. No mass or foot-contact dynamics are claimed.
PROTO Runner4 [
  field SFVec3f translation {START[0]} {START[1]} {BODY_Z}
  field SFRotation rotation 0 0 1 0
  field SFString controller "runner4_line"
] {{
  Robot {{ translation IS translation rotation IS rotation controller IS controller
    name "RUNNER4 - kinematic line follower" supervisor TRUE
    children [
      {body(SENSOR_X, SENSOR_Y, BODY_Z).vrml()}
      {legs}
      {cameras}
    ]
    boundingObject Box {{ size .1 .06 .018 }}
  }}
}}
'''


def track():
    # Counter-clockwise stadium: bottom straight, right arc, top straight, left arc.
    points = [(-.4, -.45), (.4, -.45)]
    points += [(.4+.45*math.cos(a), .45*math.sin(a))
               for a in [-math.pi/2+i*math.pi/100 for i in range(1, 101)]]
    points += [(-.4, .45)]
    points += [(-.4+.45*math.cos(a), .45*math.sin(a))
               for a in [math.pi/2+i*math.pi/100 for i in range(1, 100)]]
    vertices, faces = [], []
    for i, p in enumerate(points):
        prev, nxt = points[i-1], points[(i+1) % len(points)]
        dx, dy = nxt[0]-prev[0], nxt[1]-prev[1]
        length = math.hypot(dx, dy)
        nx, ny = -dy/length*.015, dx/length*.015
        vertices += [(p[0]+nx, p[1]+ny, .001), (p[0]-nx, p[1]-ny, .001)]
        j = (i+1) % len(points)
        faces.append(f'{2*i} {2*i+1} {2*j+1} {2*j} -1')
    return '''Shape {
      appearance PBRAppearance { baseColor 1 1 1 emissiveColor 1 1 1 roughness 1 metalness 0 }
      geometry IndexedFaceSet { coord Coordinate { point [
      ''' + ',\n'.join(vec(v) for v in vertices) + '\n] } coordIndex [\n' + '\n'.join(faces) + '\n] } }'


def world():
    walls = '\n'.join(box(size, '0.18 0.23 0.29', p) for size, p in [
        ((2.4, .025, .06), (0, -.9, .03)), ((2.4, .025, .06), (0, .9, .03)),
        ((.025, 1.8, .06), (-1.2, 0, .03)), ((.025, 1.8, .06), (1.2, 0, .03))])
    return f'''#VRML_SIM R2025a utf8
EXTERNPROTO "../protos/Runner4.proto"
WorldInfo {{ basicTimeStep 20 coordinateSystem "ENU" title "RUNNER4 | white-line demonstration" }}
Viewpoint {{ orientation -0.57735026919 0.57735026919 0.57735026919 2.09439510239
  position 0 0 3.5 fieldOfView 0.9 description "Arena overhead" }}
Background {{ skyColor [ 0.04 0.06 0.09 ] }}
DirectionalLight {{ direction 0.2 0.1 -1 intensity 1 castShadows FALSE }}
Solid {{ translation 0 0 -.01 name "black arena 2.4 x 1.8 m"
  children [ {box((2.4, 1.8, .02), '0.005 0.005 0.005')} ]
  boundingObject Box {{ size 2.4 1.8 .02 }} }}
{track()}
{walls}
DEF RUNNER4 Runner4 {{ }}
'''


if __name__ == '__main__':
    for name, content in [('protos/Runner4.proto', robot_proto()), ('worlds/runner4_white_line.wbt', world())]:
        path = ROOT / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding='utf-8')
        print(path)
