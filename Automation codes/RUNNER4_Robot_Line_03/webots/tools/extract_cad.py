"""Extract cached native SolidWorks tessellations locally. Source files read-only."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).parent/'vendor'))
from sldprt.blocks import read_blocks
from sldprt.savedmesh import read_saved_mesh
from sldprt.assembly import read_assembly


def obj(mesh, path):
    # The reader outputs mm; Webots Mesh assets use metres.
    lines = ['# Actual SolidWorks saved tessellation; units metres']
    lines += ['v '+' '.join(f'{v/1000:.9f}' for v in p) for p in mesh.vertices]
    lines += ['f '+' '.join(str(i+1) for i in face) for face in mesh.triangles]
    path.write_text('\n'.join(lines)+'\n',encoding='ascii')


def bounds(mesh):
    return [[min(v[i] for v in mesh.vertices) for i in range(3)],
            [max(v[i] for v in mesh.vertices) for i in range(3)]]


def main():
    source = ROOT.parents[2]/'3D designs/quadruped-spider-2.snapshot.2 (1)'
    output = ROOT/'protos/meshes/cad'
    output.mkdir(parents=True,exist_ok=True)
    report = {}
    for path in sorted(source.glob('*.SLDPRT')):
        data = path.read_bytes()
        blocks = read_blocks(data)
        display = [b for b in blocks if b.name in ('Contents/DisplayLists','FaceTessellations')]
        if not display or not all(b.verify() for b in display):
            raise RuntimeError(f'No CRC-verified display block: {path.name}')
        mesh = read_saved_mesh(blocks)
        if mesh is None or not mesh.triangles:
            raise RuntimeError(f'No decoded mesh: {path.name}')
        name = path.stem.lower().replace(' ','_')+'.obj'
        obj(mesh,output/name)
        report[path.name] = {'sha256':hashlib.sha256(data).hexdigest(),
                            'obj':name,'triangles':len(mesh.triangles),
                            'vertices':len(mesh.vertices),'bounds_mm':bounds(mesh),
                            'display_crc_verified':True}
        print(path.name,report[path.name]['triangles'],report[path.name]['bounds_mm'])
    parts = {p.name.lower():p.read_bytes() for p in source.glob('*.SLDPRT')}
    assembly_path = source/'Assem1.SLDASM'
    mesh,components,summary = read_assembly(read_blocks(assembly_path.read_bytes()),resolve=lambda name:parts.get(name.lower()))
    obj(mesh,output/'assembly_reference.obj')
    report['assembly'] = {'summary':summary.summary(),'complete':summary.complete,
                          'bounds_mm':bounds(mesh),'components':[]}
    for i,component in enumerate(components):
        obj(component.mesh, output/f'component_{i:02d}.obj')
        item = {'name':component.name,'obj':f'component_{i:02d}.obj','bounds_mm':bounds(component.mesh)}
        report['assembly']['components'].append(item)
        print(i,item['name'],item['bounds_mm'])
    (output/'extraction.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(summary.summary())


if __name__ == '__main__':
    main()
