import bpy
import sys
import os
import bmesh
import mathutils
import math

context = bpy.context
scene = context.scene
viewlayer = context.view_layer

#BoundingBox_size
bbox_set = set()
sfac = 1.5

for o in scene.objects:
    if "Bounding" in o.name:
        bbox_set.add(o)
#print(bbox_set)

for bbox in bbox_set:  # <<<<<<<<<<
    bbox.select_set(False)  # <<<<<<<<<<

while len(bbox_set) >= 1:
    box = bbox_set.pop()
    box.scale = sfac,sfac,1.0


for o in scene.objects:
    if "Bounding" in o.name:
        bbox_set.add(o)


def setupCamera(scene, c):
    scene.camera.location.x = c[0]
    scene.camera.location.y = c[1]
    scene.camera.location.z = c[2]

    return

scene = bpy.data.scenes["Scene"]

for ob in bbox_set:
    #bpy.ops.object.mode_set(mode="EDIT")
    wmtx = ob.matrix_world.copy()
    print('wmtx',wmtx)
    for face in ob.data.polygons:
        #normal_vec = face.normal
        center_vec = face.center
        bpy.ops.mesh.primitive_ico_sphere_add(radius=0.1,location= wmtx @ center_vec)

        config = wmtx @ center_vec
        bpy.ops.object.camera_add()
        cam = bpy.data.objects['Camera']
        scene.camera =cam
        setupCamera(scene=scene,c=config)

        #print(center_vec,normal_vec)

# coordinates as tuples
#plain_verts = [vert.to_tuple() for vert in verts]
#print(plain_verts)
'''
for obj in bpy.data.objects:
    if obj.type == 'MESH':
        obj.name = obj.data.name
        wmtx = mathutils.Matrix(obj.matrix_world)
        wmtx.invert()

        face_list = [f for f in obj.data.faces if obj.material_slots[f.material_index].name == 'Torch']

        for face in face_list:
            norm = mathutils.Vector(face.normal)
            if norm.dot((0, 0, 1)) == 1:
                bpy.ops.object.add(type='LAMP', location=face.center * wmtx)
'''
'''
dir = os.path.dirname(bpy.data.filepath)
if not dir in sys.path:
    sys.path.append(dir)
    print(sys.path)

path = os.path.abspath("C:/Users/User/AppData/Roaming/Blender Foundation/Blender/2.93/scripts/addons")
if path not in sys.path:
    sys.path.append(path)
    print('appended')
'''
