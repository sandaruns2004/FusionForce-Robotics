"""Render the same neutral visual meshes used in the PROTO; not a Webots capture.

Optional developer dependency: matplotlib. Running Webots needs no such package.
"""
import math
from pathlib import Path
import sys
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from robot_visuals import neutral_scene

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'controllers/runner4_line'))
from core import BODY_Z, SENSOR_X, SENSOR_Y


def scene():
    return neutral_scene(SENSOR_X, SENSOR_Y, BODY_Z)


if __name__ == '__main__':
    fig = plt.figure(figsize=(14,7),facecolor='#e6e9ec')
    model = scene()
    for index,(elev,azim,label) in enumerate([(24,-38,'ASSEMBLED VIEW'),(90,-90,'TOP VIEW')],1):
        ax = fig.add_subplot(1,2,index,projection='3d',computed_zorder=False)
        ax.set_facecolor('#e6e9ec')
        polygons,colors = [],[]
        for vertices,faces,color in model.meshes:
            for face in faces:
                polygons.append([vertices[i] for i in face])
                colors.append(color)
        collection = Poly3DCollection(polygons,facecolors=colors,edgecolors='none',zsort='average')
        ax.add_collection3d(collection)
        ax.set(xlim=(-.16,.16),ylim=(-.16,.16),zlim=(-.12,.11))
        ax.set_box_aspect((.32,.32,.23))
        ax.view_init(elev=elev,azim=azim)
        ax.set_axis_off()
        ax.set_title(label,fontsize=12,color='#344454')
    fig.suptitle('RUNNER4 | Photo-reference robot rebuild',fontsize=21,color='#172a3c',y=.97)
    fig.text(.5,.06,'Same generated visual meshes as the Webots robot • Neutral pose • Approximate dimensions',ha='center',fontsize=11,color='#344454')
    fig.subplots_adjust(left=0,right=1,bottom=.10,top=.88,wspace=0)
    path=ROOT/'references/robot_preview.png'
    fig.savefig(path,dpi=150)
    print(path)
