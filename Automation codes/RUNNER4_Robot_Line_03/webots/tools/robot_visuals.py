"""Photo/CAD-screenshot reconstruction. Metres; no proprietary CAD conversion.

Meshes are shared by Webots and the optional static preview renderer.
"""
import math

BLACK = (0.055, 0.060, 0.068)
SERVO = (0.075, 0.080, 0.090)
SILVER = (0.65, 0.68, 0.71)
BOARD = (0.48, 0.25, 0.10)
WIRE = [(0.22, .12, .07), (.78, .045, .025), (1, .62, .05)]
RIBBON = [(.65, .05, .7), (.05, .22, .85), (.04, .7, .22), (.95, .85, .06), (1, .36, .02), (.8, .04, .04)]


class Visual:
    def __init__(self):
        self.meshes = []

    def mesh(self, vertices, faces, color):
        self.meshes.append((vertices, faces, color))
        return self

    def box(self, size, center, color):
        x, y, z = (s/2 for s in size)
        vertices = [(center[0]+a*x, center[1]+b*y, center[2]+c*z)
                    for a, b, c in [(-1,-1,-1),(1,-1,-1),(1,1,-1),(-1,1,-1),
                                     (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)]]
        return self.mesh(vertices, [(0,3,2,1),(4,5,6,7),(0,1,5,4),
                                    (1,2,6,5),(2,3,7,6),(3,0,4,7)], color)

    def beam(self, start, end, radius, color, sides=6):
        d = [b-a for a,b in zip(start,end)]
        length = math.sqrt(sum(v*v for v in d))
        d = [v/length for v in d]
        helper = (0,0,1) if abs(d[2]) < .9 else (1,0,0)
        u = (d[1]*helper[2]-d[2]*helper[1], d[2]*helper[0]-d[0]*helper[2], d[0]*helper[1]-d[1]*helper[0])
        norm = math.sqrt(sum(v*v for v in u))
        u = [v/norm for v in u]
        v = (d[1]*u[2]-d[2]*u[1], d[2]*u[0]-d[0]*u[2], d[0]*u[1]-d[1]*u[0])
        vertices = [tuple(p[k]+radius*(u[k]*math.cos(i*2*math.pi/sides)+v[k]*math.sin(i*2*math.pi/sides)) for k in range(3))
                    for p in (start,end) for i in range(sides)]
        faces = [tuple(reversed(range(sides))), tuple(range(sides,2*sides))]
        faces += [(i,(i+1)%sides,(i+1)%sides+sides,i+sides) for i in range(sides)]
        return self.mesh(vertices, faces, color)

    def cable(self, points, color, radius=.00065):
        for a,b in zip(points,points[1:]):
            self.beam(a,b,radius,color)
        return self

    def add(self, other, offset=(0,0,0), yaw=0):
        c,s = math.cos(yaw),math.sin(yaw)
        for vertices, faces, color in other.meshes:
            self.mesh([(c*x-s*y+offset[0],s*x+c*y+offset[1],z+offset[2]) for x,y,z in vertices],faces,color)
        return self

    def vrml(self):
        # Group by material to keep draw calls low despite small electronics parts.
        groups = {}
        for vertices,faces,color in self.meshes:
            dest,indices = groups.setdefault(color, ([],[]))
            n = len(dest)
            dest.extend(vertices)
            indices.extend([tuple(i+n for i in f) for f in faces])
        result = []
        for color,(vertices,faces) in groups.items():
            coords = ', '.join(' '.join(f'{v:.7f}' for v in p) for p in vertices)
            indices = ', '.join(' '.join(map(str,f))+' -1' for f in faces)
            rgb = ' '.join(map(str,color))
            result.append(f'Shape {{ appearance PBRAppearance {{ baseColor {rgb} roughness 0.65 metalness 0 }} '
                          f'geometry IndexedFaceSet {{ coord Coordinate {{ point [ {coords} ] }} coordIndex [ {indices} ] }} }}')
        return '\n'.join(result)


def servo(horizontal=False):
    v = Visual()
    v.box((.023,.013,.025),(0,0,-.010),SERVO)
    v.box((.031,.013,.003),(0,0,-.002),BLACK)
    v.box((.018,.001,.009),(0,-.007,-.011),(.35,.04,.04))
    if horizontal:
        v.beam((-.014,0,0),(.014,0,0),.0045,SILVER,12)
        v.box((.002,.019,.004),(.015,.005,0),(.80,.82,.79))
        v.beam((.016,0,0),(.017,0,0),.0018,BLACK,10)
    else:
        v.beam((0,0,.002),(0,0,.006),.0045,SILVER,12)
        v.box((.004,.022,.002),(0,.007,.006),(.80,.82,.79))
    return v


def neutral_scene(sensor_x, sensor_y, body_z):
    model = body(sensor_x, sensor_y, body_z)
    for front in [1,-1]:
        for side in [1,-1]:
            hip, thigh, knee, coxa, femur, _ = leg_parts(side)
            local = Visual().add(hip).add(thigh,coxa)
            local.add(knee,tuple(a+b for a,b in zip(coxa,femur)))
            model.add(local,(front*.038,side*.030,0),-side*front*math.radians(35))
    return model


def bracket(delta, width=.029):
    v = Visual()
    # Two rails and two end bridges leave the central servo opening visible.
    for x in [-width/2,width/2]:
        v.beam((x,0,0),(x,delta[1],delta[2]),.0028,BLACK,4)
    for p in [(0,0,0),delta]:
        v.beam((-width/2,p[1],p[2]),(width/2,p[1],p[2]),.0028,BLACK,4)
        for side in [-1,1]:
            v.beam((side*(width/2+.001),p[1],p[2]),
                   (side*(width/2+.002),p[1],p[2]),.0034,SILVER,10)
    return v


def leg_parts(side):
    coxa = (0,side*.030,0)
    femur = (0,side*.048,-.036)
    tibia = (0,side*.016,-math.sqrt(.080**2-.016**2))
    hip = bracket(coxa)
    thigh = bracket(femur)
    thigh.add(servo(True))
    knee = servo(True)
    # Thin curved blade, swept out at the upper end and narrowing to the tip.
    for i in range(12):
        vertices = []
        for t in [i/12,(i+1)/12]:
            centre = side*(.016*t+.009*math.sin(math.pi*t))
            half = .012*(1-t)+.0016*t
            z = tibia[2]*t
            vertices.extend([(x,centre+dy,z) for x,dy in [(-.003,-half),(.003,-half),(.003,half),(-.003,half)]])
        # Sections descend along -Z, so their outward winding is opposite a box.
        knee.mesh(vertices,[(0,1,2,3),(7,6,5,4),(4,5,1,0),(5,6,2,1),(6,7,3,2),(7,4,0,3)],BLACK)
    # Orange/red/brown servo leads on the bracket, attached to its moving link.
    for i,color in enumerate(WIRE):
        points = [(i*.0015-.0015,side*(.048*t),-.036*t+.008*math.sin(math.pi*t)+.005)
                  for t in [j/12 for j in range(13)]]
        thigh.cable(points,color)
    return hip,thigh,knee,coxa,femur,tibia


def body(sensor_x, sensor_y, body_z):
    v = Visual()
    # Two black chassis frames around an open central slot (CAD Base up/down).
    for z in [-.017,.008]:
        for y in [-.026,.026]:
            v.box((.100,.008,.004),(0,y,z),BLACK)
        for x in [-.046,.046]:
            v.box((.008,.044,.004),(x,0,z),BLACK)
    for x in [-.043,.043]:
        for y in [-.024,.024]:
            v.beam((x,y,-.019),(x,y,.012),.003,BLACK,10)
            v.beam((x,y,.012),(x,y,.014),.002,SILVER,10)
    # Battery in the chassis, black retaining strap, perfboard on standoffs.
    v.box((.065,.032,.020),(0,0,-.004),(.72,.73,.74))
    v.box((.014,.035,.022),(-.008,0,-.004),BLACK)
    for x in [-.031,.031]:
        for y in [-.020,.020]:
            v.beam((x,y,.010),(x,y,.032),.0016,(.78,.76,.65),8)
    v.box((.078,.050,.002),(0,0,.033),BOARD)
    # Copper pad grid remains visible around the electronics.
    for ix in range(28):
        for iy in range(18):
            x,y = -.035+ix*.00254,-.022+iy*.00254
            if abs(y) > .013 or x < -.023:
                v.box((.0012,.0012,.0003),(x,y,.0343),(.83,.65,.30))
    # ESP32 dev board, RF shield, USB socket, header rails.
    v.box((.052,.023,.002),(.008,.005,.040),(.025,.055,.047))
    v.box((.019,.017,.003),(-.006,.005,.043),SILVER)
    v.box((.009,.011,.001),(.014,.005,.042),(.025,.025,.028))
    v.box((.006,.008,.004),(.034,.005,.043),SILVER)
    for y in [-.008,.018]:
        v.box((.049,.0025,.004),(.008,y,.038),BLACK)
        for i in range(19):
            v.box((.0007,.0007,.004),(-.016+i*.00254,y,.035),SILVER)
    # Blue PCA9685 board and its rows of black servo connectors.
    v.box((.032,.018,.002),(-.021,-.014,.039),(.025,.22,.55))
    for i in range(8):
        v.box((.003,.005,.006),(-.034+i*.0036,-.018,.043),BLACK)
    v.box((.008,.006,.006),(-.030,-.008,.043),(.12,.48,.20))
    for i,color in enumerate([(.9,.025,.02),(.02,.7,.05),(.03,.1,.9)]):
        v.beam((-.031+i*.006,.016,.035),(-.031+i*.006,.016,.041),.002,color,10)
    # Front strip has nine pairs of IR packages; optics sit below it, unobstructed.
    z = .018-body_z
    v.box((.012,.096,.002),(sensor_x,0,z),BOARD)
    for y in sensor_y:
        for dx in [-.0025,.0025]:
            v.box((.003,.004,.004),(sensor_x+dx,y,z-.003),(.025,.025,.030))
        v.box((.001,.004,.001),(sensor_x,y,z+.002),SILVER)
    for y in [-.026,.026]:
        v.beam((.043,y,-.015),(sensor_x-.007,y,z+.003),.0022,BLACK,4)
    # Large ribbon loop from top electronics to front sensor assembly.
    for i,color in enumerate(RIBBON):
        points = []
        for j in range(29):
            t=j/28
            points.append((.023+(sensor_x-.023)*t,
                           -.007+i*.0016+.025*math.sin(math.pi*t),
                           .044*(1-t)+(z+.004)*t+.090*math.sin(math.pi*t)))
        v.cable(points,color,.0007)
    # Four loosely looped three-conductor servo harnesses.
    for front in [-1,1]:
        for side in [-1,1]:
            for i,color in enumerate(WIRE):
                points = [(front*(.015+.024*t),side*(.010+.022*t)+i*.0014,
                           .044*(1-t)+.006*t+.030*math.sin(math.pi*t)) for t in [j/16 for j in range(17)]]
                v.cable(points,color)
            v.add(servo(),(front*.038,side*.030,0))
    # Yellow XT-style power connector and red/black battery leads.
    v.box((.012,.008,.008),(.017,-.039,.018),(.95,.73,.015))
    for i,color in enumerate([(.8,.025,.025),(.018,.018,.018)]):
        v.cable([(-.018,-.014+i*.004,.004),(-.010,-.036+i*.004,.026),(.014,-.039+i*.003,.020)],color,.0012)
    return v
