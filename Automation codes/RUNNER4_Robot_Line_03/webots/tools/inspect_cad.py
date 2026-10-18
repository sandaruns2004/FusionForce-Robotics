"""Developer diagnostic: orthographic plots of extracted CAD coordinates (metres)."""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection

ROOT=Path(__file__).resolve().parents[1]


def read_obj(path):
    vertices,faces=[],[]
    for line in path.read_text().splitlines():
        if line.startswith('v '): vertices.append(tuple(map(float,line.split()[1:])))
        elif line.startswith('f '): faces.append(tuple(int(i.split('/')[0])-1 for i in line.split()[1:]))
    return vertices,faces


if __name__=='__main__':
    names=['coxa_left','fumer','tibia_left','servo_sg90']
    fig,axes=plt.subplots(4,3,figsize=(13,13))
    for row,name in enumerate(names):
        vertices,faces=read_obj(ROOT/'protos/meshes/cad'/f'{name}.obj')
        for col,(a,b) in enumerate([(0,1),(0,2),(2,1)]):
            ax=axes[row,col]
            polygons=[[(vertices[i][a]*1000,vertices[i][b]*1000) for i in f] for f in faces]
            ax.add_collection(PolyCollection(polygons,facecolors='#65788a',edgecolors='#253342',linewidths=.1,alpha=.6))
            ax.autoscale_view(); ax.set_aspect('equal'); ax.grid(alpha=.25)
            ax.set_title(name+' | '+'XYZ'[a]+' / '+'XYZ'[b]); ax.set_xlabel('XYZ'[a]+' mm'); ax.set_ylabel('XYZ'[b]+' mm')
    fig.tight_layout(); fig.savefig(ROOT/'references/cad_parts_inspection.png',dpi=140)
