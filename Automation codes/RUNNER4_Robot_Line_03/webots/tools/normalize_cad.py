"""Center named SolidWorks OBJ meshes while preserving their metre dimensions."""
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / 'protos' / 'meshes' / 'cad'
TARGET = SOURCE / 'normalized'
PARTS = ('base_down', 'base_up', 'coxa_left', 'coxa_right', 'fumer',
         'tibia_left', 'tibia_right', 'hold', 'servo_sg90')


def read_obj(path):
    vertices, faces = [], []
    for line in path.read_text(encoding='ascii').splitlines():
        if line.startswith('v '):
            vertices.append([float(v) for v in line.split()[1:4]])
        elif line.startswith('f '):
            faces.append(line)
    return vertices, faces


def write_centered(name):
    vertices, faces = read_obj(SOURCE / f'{name}.obj')
    mins = [min(v[i] for v in vertices) for i in range(3)]
    maxs = [max(v[i] for v in vertices) for i in range(3)]
    center = [(mins[i] + maxs[i]) / 2 for i in range(3)]
    lines = [f'# Centered SolidWorks display tessellation: {name}']
    lines += ['v ' + ' '.join(f'{v[i] - center[i]:.9f}' for i in range(3))
              for v in vertices]
    lines += faces
    (TARGET / f'{name}.obj').write_text('\n'.join(lines) + '\n', encoding='ascii')
    dims = tuple(maxs[i] - mins[i] for i in range(3))
    print(name, 'dimensions_m=', tuple(round(v, 6) for v in dims))


if __name__ == '__main__':
    TARGET.mkdir(parents=True, exist_ok=True)
    for part in PARTS:
        write_centered(part)
